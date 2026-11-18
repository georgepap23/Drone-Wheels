# Rebuild model.sdf using the real joint-pose values from the user's
# urdfbase.com-tuned model-2.sdf (captured earlier when the file was read in
# full), in its joint-centric pose convention, plus the N20-as-own-link split.

MESH_PATH = "model://Evo-Borne/meshes"

# name -> (leg_mesh, hip_pose, rotor_pose, wheel_pose, servo_pose)
# pose tuples are the exact "x y z roll pitch yaw" strings from model-2.sdf.
CORNERS = {
    "front_left": dict(
        leg_mesh="Left_leg_final.stl",
        hip="0.0224 -0.1317 0.0002 0 0 -1.5708",
        rotor="-0.0219 0.0416 -0.1212 3.1416 -1.5708 1.5708",
        wheel="-0.0226 0.0444 -0.1208 0 -1.5708 0",
        servo="0.0293 -0.0692 0.025 3.1416 0 0",
        hip_upper="1.5708",
        n20="0 0 -0.035 0 0 0",
    ),
    "front_right": dict(
        leg_mesh="Right_leg_final.stl",
        hip="0.0644 0.1106 -0.0028 0 0 -1.5708",
        rotor="0 0.0004 -0.1159 0 1.5708 1.5708",
        wheel="0 0.0002 -0.1151 0 1.5708 0",
        servo="0.0276 0.0687 0.0045 0 0 0",
        hip_upper="1.570796327",
        n20="-0.0219 -0.0428 -0.0293 0 0 0",
    ),
    "rear_left": dict(
        leg_mesh="Left_leg_final.stl",
        hip="-0.021 0.1295 0.0009 0 0 1.5708",
        rotor="-0.0214 0.0425 -0.1203 3.1416 -1.5708 1.5708",
        wheel="-0.022 0.0435 -0.1218 0 -1.5708 0",
        servo="-0.0277 0.0682 0.025 -3.1416 0 -3.1416",
        hip_upper="1.5708",
        n20="0 0 -0.035 0 0 0",
    ),
    "rear_right": dict(
        leg_mesh="Right_leg_final.stl",
        hip="-0.0631 -0.1093 -0.0038 0 0 1.5708",
        rotor="0.0011 -0.0005 -0.1161 3.1416 -1.5708 1.5708",
        wheel="0.0007 0.0007 -0.1166 -0.0002 -1.5403 0",
        servo="-0.0268 -0.0684 0.0043 0 0 -3.1416",
        hip_upper="1.570796327",
        n20="-0.0219 -0.0428 -0.0293 0 0 0",
    ),
}


# ---- hip pivot = servo output shaft (Mg995.stl native: x=12.54mm, z=10mm, top y=30.5mm, axis +Y)
import numpy as np
def _R(r,p,y):
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
            @np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))
def _T(st):
    v=[float(x) for x in st.split()]; return np.array(v[:3]),_R(*v[3:])
def _rot(a,t):
    K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]]); return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*K@K
SHAFT=np.array([0.01254,0.0305,0.010])
for _n,_c in CORNERS.items():
    sp,sR=_T(_c['servo']); P=sp+sR@SHAFT; A=sR@np.array([0,1.0,0])
    hp,hR=_T(_c['hip']); W=hp+hR@_T(_c['wheel'])[0]
    out=1 if _n.startswith('front') else -1          # +90deg swings the wheel outward (fore/aft)
    if np.sign((P+_rot(A,np.pi/2)@(W-P))[0]-P[0])!=out: A=-A
    A=np.round(A,4)+0.0
    _c['hip_pivot']=" ".join(f"{x:.4f}" for x in P)
    # wheel/rotor axes: same chassis direction (+X) on all 4 corners, so a positive
    # wheel command drives every wheel forward (-Y = front) and rotors share one convention
    _c['wheel_axis']="0 1 0" if (hR@_T(_c['wheel'])[1]@[0,1,0])[0]>0 else "0 -1 0"
    _c['rotor_axis']="0 0 1" if (hR@_T(_c['rotor'])[1]@[0,0,1])[0]>0 else "0 0 -1"
    _c['hip_axis']=" ".join(f"{x:g}" for x in A)

# ---- level the wheels (legs down): move each leg in chassis Z so every wheel
#      centre matches the lowest one (all four touch the ground together)
def _wheel_z(c):
    hp,hR=_T(c['hip']); return (hp+hR@_T(c['wheel'])[0])[2]
_zmin=min(_wheel_z(c) for c in CORNERS.values())
for _c in CORNERS.values():
    v=_c['hip'].split(); v[2]=f"{float(v[2])-(_wheel_z(_c)-_zmin):.4f}"; _c['hip']=" ".join(v)


def mesh_geom(filename, pose=None):
    p = f"<pose>{pose}</pose>" if pose else ""
    return (f"<geometry><mesh><uri>{MESH_PATH}/{filename}</uri>"
            f"<scale>0.001 0.001 0.001</scale></mesh></geometry>")


def corner_block(name, c):
    leg_visual_pose = "0 0 -0.1156 0 -1.5708 0"
    return f"""
    <!-- ============ {name} ============ -->
    <link name="{name}_leg">
      <pose relative_to="chassis">{c['hip']}</pose>
      <inertial>
        <!-- N20+bracket+gear+bearing (19.02g) moved out to its own {name}_n20 link -->
        {c['_in_leg']}
      </inertial>
      <visual name="{name}_leg_visual_0">
        <pose>{leg_visual_pose}</pose>
        {mesh_geom(c['leg_mesh'])}
      </visual>
      <collision name="{name}_leg_collision_0">
        <pose>{leg_visual_pose}</pose>
        {mesh_geom(c['leg_mesh'])}
      </collision>
    </link>

    <link name="{name}_n20">
      <pose relative_to="{name}_leg">{c['n20']}</pose>
      <inertial>
        {IN_N20}
      </inertial>
      <visual name="{name}_n20_visual_0">
        {mesh_geom("N20 DC MOTOR.stl")}
      </visual>
    </link>
    <joint name="{name}_n20_fixed" type="fixed">
      <parent>{name}_leg</parent>
      <child>{name}_n20</child>
    </joint>

    <link name="{name}_rotor">
      <pose relative_to="{name}_rotor_joint">0 0 0 0 0 0</pose>
      <inertial>
        {IN_ROTOR}
      </inertial>
      <visual name="{name}_rotor_visual_0">
        {mesh_geom("brushless emax series eco ii 2207 v23.stl")}
      </visual>
      <visual name="{name}_rotor_visual_1">
        <pose>0 0 0.027 1.5708 0 0</pose>
        {mesh_geom("gemfan propeller 51466.stl")}
      </visual>
    </link>

    <link name="{name}_wheel">
      <pose relative_to="{name}_wheel_joint">0 0 0 0 0 0</pose>
      <inertial>
        {IN_WHEEL}
      </inertial>
      <visual name="{name}_wheel_visual_0">
        {mesh_geom("Wheel_morpho.stl")}
      </visual>
      <collision name="{name}_wheel_collision_0">
        <!-- Ø165 mm x 30.5 mm cylinder along the wheel axle (link Y) -->
        <pose>0 0.0102 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.0825</radius><length>0.0305</length></cylinder></geometry>
        <surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface>
      </collision>
    </link>

    <link name="{name}_servo">
      <pose relative_to="{name}_servo_fixed">0 0 0 0 0 0</pose>
      <inertial>
        {IN_SERVO}
      </inertial>
      <visual name="{name}_servo_visual_0">
        {mesh_geom("Mg995.stl")}
      </visual>
    </link>

    <joint name="{name}_hip_joint" type="revolute">
      <parent>chassis</parent>
      <child>{name}_leg</child>
      <!-- pivot = servo output shaft; axis = shaft axis (in chassis frame) -->
      <pose relative_to="chassis">{c['hip_pivot']} 0 0 0</pose>
      <axis>
        <xyz>{c['hip_axis']}</xyz>
        <limit><lower>0</lower><upper>1.5708</upper><effort>2.45</effort><velocity>6.28</velocity></limit>
        <dynamics><damping>0.3</damping><friction>0.1</friction></dynamics>
      </axis>
    </joint>
    <joint name="{name}_rotor_joint" type="revolute">
      <parent>{name}_leg</parent>
      <child>{name}_rotor</child>
      <pose relative_to="{name}_leg">{c['rotor']}</pose>
      <axis><xyz>{c['rotor_axis']}</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>
    <joint name="{name}_wheel_joint" type="revolute">
      <parent>{name}_leg</parent>
      <child>{name}_wheel</child>
      <pose relative_to="{name}_leg">{c['wheel']}</pose>
      <axis><xyz>{c['wheel_axis']}</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <!-- ~10% of N20 stall (1.8 kg*cm) x 13.33 ring ratio: gearbox won't back-drive -->
        <dynamics><friction>0.25</friction></dynamics></axis>
    </joint>
    <joint name="{name}_servo_fixed" type="fixed">
      <parent>chassis</parent>
      <child>{name}_servo</child>
      <pose relative_to="chassis">{c['servo']}</pose>
    </joint>
"""


exec(open(__file__.replace("build_final_sdf.py","link_inertials.py")).read())
corners_xml = "\n".join(corner_block(n, c) for n, c in CORNERS.items())

exec(open(__file__.replace("build_final_sdf.py","ardupilot_plugins.py")).read())

sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="evo_borne">
    <link name="chassis">
      <inertial>
        {IN_CHASSIS}
      </inertial>
      <visual name="chassis_visual_0">
        <pose>0 0 0 1.5708 0 0</pose>
        {mesh_geom("chassis_only.stl")}
      </visual>
      <collision name="chassis_collision_0">
        <pose>0 0 0 1.5708 0 0</pose>
        {mesh_geom("chassis_only.stl")}
      </collision>
{IMU_XML}
    </link>
{corners_xml}
{PLUGINS_XML}
  </model>
</sdf>
"""

# rename only (geometry unchanged): CORNERS keys -> names that match the real
# position (model forward = -Y, left = +X). Same side in car and flight mode.
#   old rear_right  -> front_right
#   old front_right -> rear_left
#   old rear_left   -> rear_right
#   old front_left  -> front_left (unchanged)
for _old, _tmp in [("front_right", "@@A"), ("rear_left", "@@B"), ("rear_right", "@@C")]:
    sdf = sdf.replace(_old, _tmp)
sdf = sdf.replace("@@A", "rear_left").replace("@@B", "rear_right").replace("@@C", "front_right")

# ---- colours (visual only, no effect on physics)
import re as _re
COLORS = {                      # visual-name suffix -> RGB
    "chassis_visual_0": (0.20, 0.22, 0.25),   # dark graphite frame
    "_leg_visual_0":    (0.95, 0.45, 0.10),   # orange PLA legs
    "_wheel_visual_0":  (0.08, 0.08, 0.08),   # black wheels
    "_rotor_visual_0":  (0.70, 0.72, 0.75),   # silver BLDC
    "_rotor_visual_1":  (0.85, 0.10, 0.10),   # red propellers
    "_servo_visual_0":  (0.10, 0.30, 0.75),   # blue servos
    "_n20_visual_0":    (0.85, 0.70, 0.25),   # brass N20 gear motors
}
def _material(rgb):
    r,g,b=rgb
    return (f"<material><ambient>{r*0.6:.2f} {g*0.6:.2f} {b*0.6:.2f} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>")
def _colorize(m):
    name, body = m.group(1), m.group(2)
    for suf, rgb in COLORS.items():
        if name.endswith(suf):
            return f'<visual name="{name}">{body}  {_material(rgb)}\n      </visual>'
    return m.group(0)
sdf = _re.sub(r'<visual name="([^"]+)">(.*?)\s*</visual>', _colorize, sdf, flags=_re.S)

import os
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "model.sdf"), "w") as f:
    f.write(sdf)

print("wrote model.sdf,", len(sdf), "bytes")
