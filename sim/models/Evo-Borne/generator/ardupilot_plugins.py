# Generates the IMU sensor + LiftDrag (thrust) + ArduPilotPlugin XML.
# Executed from build_final_sdf.py; uses CORNERS/_T/_rot/np from there.
# Corner keys in CORNERS are the OLD names; the output is renamed afterwards
# (see the rename block at the end of build_final_sdf.py), so map
# final-name -> CORNERS key here and write the XML with OLD names too
# (the final rename fixes them consistently).
NEW2OLD = {"front_left": "front_left", "front_right": "rear_right",
           "rear_left": "front_right", "rear_right": "rear_left"}

# ---------------- IMU: MicoAir743 v2 primary IMU = Bosch BMI088 ----------------
# BMI088 datasheet noise density: gyro 0.014 deg/s/sqrt(Hz), accel 175 ug/sqrt(Hz)
# -> sigma at 1 kHz sample rate (500 Hz bandwidth): gyro ~0.0055 rad/s, accel ~0.038 m/s^2
# Sensor frame = ArduPilot body frame (X forward, Y right, Z down); our model
# has forward = -Y, left = +X, up = +Z  ->  roll 180, yaw -90.
def _noise(sd, bias):
    return (f"<noise type=\"gaussian\"><mean>0</mean><stddev>{sd}</stddev>"
            f"<bias_mean>{bias}</bias_mean><bias_stddev>{bias/10}</bias_stddev></noise>")
IMU_XML = f"""      <sensor name="imu_sensor" type="imu">
        <pose degrees="true">0 0 0 180 0 -90</pose>
        <always_on>1</always_on>
        <update_rate>1000</update_rate>
        <imu>
          <angular_velocity>
            <x>{_noise(0.0055, 0.0001)}</x><y>{_noise(0.0055, 0.0001)}</y><z>{_noise(0.0055, 0.0001)}</z>
          </angular_velocity>
          <linear_acceleration>
            <x>{_noise(0.038, 0.002)}</x><y>{_noise(0.038, 0.002)}</y><z>{_noise(0.038, 0.002)}</z>
          </linear_acceleration>
        </imu>
      </sensor>"""

# ---------------- Propeller thrust: EMAX ECO II 2207 1900KV + Gemfan 51466 (5.1", 3-blade) ----
# Calibration targets (ESTIMATES - replace with a thrust-stand table):
#   max thrust 2020 g per motor (vendor, 6S), at ~29,000 rpm (3037 rad/s)
#   -> kT = 19.8 N / 3037^2 = 2.15e-6 N/(rad/s)^2
#   max shaft torque ~0.2 N*m (~650 W mechanical at 3037 rad/s)
# LiftDrag per blade (3 per rotor), centre of pressure at 70% radius (0.7*64.8mm):
#   T = 3 * 0.5*rho*(w*r)^2 * A * cla*a0  -> A = 4.53e-4 m^2 with a0=0.3, cla=4.25
#   blade drag tuned (cda) so drag torque ~0.2 N*m at max speed
# EMAX ECO II 2207-1900KV datasheet, GF 5045-3 @25.2V (closest listed prop to Gemfan 51466):
#   300 g @ 14,900 rpm, 500 g @ 19,200 rpm, 700 g @ 22,800 rpm, max 1,890 g @ 37.2 A
#   -> kT = 1.21e-6 N/(rad/s)^2 (all three points agree within 1%)
#   -> max 1,890 g reached at ~3,914 rad/s (37,400 rpm)
#   torque from power (motor eff ~0.8): kQ ~ 1.48e-8 N*m/(rad/s)^2  (kQ/kT ~ 0.012 m)
# per-blade LiftDrag (3 blades, cp at 70% of 64.8 mm radius):
#   kT = 1.5*rho*r^2*A*cla*a0 -> A = 2.55e-4 m^2 ;  kQ = 1.5*rho*r^3*A*cda*a0 -> cda = 1.15
W_MAX = 3914.0
R_CP = 0.0454
BLADE = dict(a0=0.3, cla=4.25, cda=1.15, area=2.55e-4)

# ArduPilot Quad-X motor order & spin seen from above:
#   M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW
# Link names match the real position (same side in car and flight mode).
MOTORS = [("front_right", +1), ("rear_left", +1), ("front_left", -1), ("rear_right", -1)]

def _flight_axes(c):
    """Rotor joint axis & link +Z, expressed in chassis, with the leg unfolded (hip = 90 deg)."""
    P = np.array([float(x) for x in c['hip_pivot'].split()])
    A = np.array([float(x) for x in c['hip_axis'].split()])
    hp, hR = _T(c['hip']); _, rR = _T(c['rotor'])
    Rt = _rot(A, np.pi / 2)
    ax_link = np.array([0, 0, float(c['rotor_axis'].split()[2])])
    return (Rt @ hR @ rR @ ax_link), (Rt @ hR @ rR @ np.array([0, 0, 1.0]))

lift_xml, ctrl_xml = [], []
for ch, (newname, ccw) in enumerate(MOTORS):
    old = NEW2OLD[newname]; c = CORNERS[old]
    joint_up, linkz_up = _flight_axes(c)
    mult_sign = int(ccw * np.sign(joint_up[2]))          # +joint speed -> which way about world up
    # spin of the rotor link about its own +Z (for the blade velocity direction)
    w_sign = mult_sign * int(np.sign(float(c['rotor_axis'].split()[2])))
    up = "0 0 1" if linkz_up[2] > 0 else "0 0 -1"
    for k in range(3):
        th = 2 * np.pi * k / 3
        cp = (R_CP * np.cos(th), R_CP * np.sin(th))
        fwd = (w_sign * -np.sin(th), w_sign * np.cos(th))
        lift_xml.append(f"""    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>{old}_rotor</link_name>
      <a0>{BLADE['a0']}</a0><cla>{BLADE['cla']}</cla><cda>{BLADE['cda']}</cda><cma>0</cma>
      <alpha_stall>1.4</alpha_stall><cla_stall>-0.025</cla_stall><cda_stall>0</cda_stall><cma_stall>0</cma_stall>
      <area>{BLADE['area']}</area><air_density>1.2041</air_density>
      <cp>{cp[0]:.4f} {cp[1]:.4f} 0</cp>
      <forward>{fwd[0]:.4f} {fwd[1]:.4f} 0</forward>
      <upward>{up}</upward>
    </plugin>""")
    # rotor inertia izz 5.44e-6 at 1 ms step: keep p*dt/I < 1  -> p = 0.002
    ctrl_xml.append(f"""      <!-- SERVO{ch+1} = Motor {ch+1}: {old} ({'CCW' if ccw>0 else 'CW'}) -->
      <control channel="{ch}">
        <jointName>{old}_rotor_joint</jointName>
        <useForce>1</useForce>
        <type>VELOCITY</type>
        <multiplier>{mult_sign*W_MAX:.0f}</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min><servo_max>1900</servo_max>
        <p_gain>0.002</p_gain><i_gain>0</i_gain><d_gain>0</d_gain><i_max>0</i_max><i_min>0</i_min>
        <cmd_max>0.35</cmd_max><cmd_min>-0.35</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>""")

# ---------------- Hip servos: Waveshare ST3020 ----------------
# 25 kg*cm stall (2.45 N*m) @12V, 0.167 s/60deg (6.28 rad/s). PWM 1000-2000 -> 0..90 deg.
for i, newname in enumerate(["front_left", "front_right", "rear_left", "rear_right"]):
    old = NEW2OLD[newname]
    ctrl_xml.append(f"""      <!-- SERVO{5+i}: {old} hip (ST3020), 1000us = legs down, 2000us = unfolded 90 deg -->
      <control channel="{4+i}">
        <jointName>{old}_hip_joint</jointName>
        <useForce>1</useForce>
        <type>POSITION</type>
        <multiplier>1.5708</multiplier>
        <offset>0</offset>
        <servo_min>1000</servo_min><servo_max>2000</servo_max>
        <p_gain>20</p_gain><i_gain>10</i_gain><d_gain>0.3</d_gain><i_max>2.45</i_max><i_min>-2.45</i_min>
        <cmd_max>2.45</cmd_max><cmd_min>-2.45</cmd_min>
      </control>""")

# ---------------- Wheels: Pololu N20 HPCB 12V 150:1 -> 12T pinion -> 160T ring (13.33:1) ----------------
# motor: 220 rpm no-load, 1.8 kg*cm stall @12V
# wheel: 220/13.33 = 16.5 rpm = 1.728 rad/s  (~0.14 m/s on the 164 mm wheel)
#        1.8*13.33 = 24 kg*cm = 2.35 N*m stall, x0.9 spur-gear efficiency -> 2.12 N*m
# PWM 1000..1500..2000 -> -1.728..0..+1.728 rad/s (positive = forward on all 4).
for i, newname in enumerate(["front_left", "front_right", "rear_left", "rear_right"]):
    old = NEW2OLD[newname]
    ctrl_xml.append(f"""      <!-- SERVO{9+i}: {old} wheel (N20), 1500us = stop -->
      <control channel="{8+i}">
        <jointName>{old}_wheel_joint</jointName>
        <useForce>1</useForce>
        <type>VELOCITY</type>
        <multiplier>3.456</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min><servo_max>2000</servo_max>
        <p_gain>0.2</p_gain><i_gain>2</i_gain><d_gain>0</d_gain><i_max>2.12</i_max><i_min>-2.12</i_min>
        <cmd_max>2.12</cmd_max><cmd_min>-2.12</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>""")

PLUGINS_XML = "\n    <!-- ===== propeller thrust: 3 LiftDrag blades per rotor ===== -->\n" + "\n".join(lift_xml) + f"""

    <!-- ===== joint states: makes Gazebo track joint position/velocity, which the
         ArduPilot plugin's PID controllers read (without it, no joint moves) ===== -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>

    <!-- ===== ArduPilot SITL bridge ===== -->
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <!-- model: forward=-Y, left=+X, up=+Z  ->  ArduPilot body X fwd, Y right, Z down -->
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 -90</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_sensor</imuName>
""" + "\n".join(ctrl_xml) + """
    </plugin>"""
