#!/usr/bin/env python3
"""Spin all 4 BLDC motors together at a low throttle for a few seconds (SITL test).

Motors (SERVO1-4) belong to ArduPilot's mixer, so unlike the hips/wheels they
can't be set with `servo set`; ArduPilot only drives them all together when
ARMED. So this script: STABILIZE -> arm -> hold throttle stick (RC override)
-> throttle to zero -> disarm.

!! Unfold the legs first (python3 scripts/legs.py) so the props point up.
!! Keep THROTTLE_PCT low: in STABILIZE about 50% stick = hover, and the robot
   will lift off well before that on light props. 10-20% is a safe test.

Usage:  python3 scripts/spin_motors.py            # 15% for 5 s
        python3 scripts/spin_motors.py 20 3       # 20% for 3 s
"""
import sys
import time
from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5763"
THROTTLE_PCT = float(sys.argv[1]) if len(sys.argv) > 1 else 15
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
MAX_PCT = 50                       # refuse anything near hover

if not 0 < THROTTLE_PCT <= MAX_PCT:
    sys.exit(f"Throttle must be between 0 and {MAX_PCT}%")

# source_system 255 = the GCS id ArduPilot accepts RC overrides from (SYSID_MYGCS)
m = mavutil.mavlink_connection(CONN, source_system=255)
m.wait_heartbeat(timeout=10)
print(f"Connected to system {m.target_system}")


def throttle(pwm):
    # channel 3 = throttle; 0 = "no override" for the others
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component, 0, 0, pwm, 0, 0, 0, 0, 0)


def wait_armed(state, timeout=5):
    t = time.time()
    while time.time() - t < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.get_srcSystem() == m.target_system and \
                bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == state:
            return True
    return False


m.set_mode("STABILIZE")
throttle(1000)                     # stick must be low to arm
time.sleep(0.5)
m.arducopter_arm()
if not wait_armed(True):
    throttle(0)
    sys.exit("Arming failed - check the MAVProxy console for the PreArm reason")

pwm = int(1000 + THROTTLE_PCT * 10)
print(f"Armed. Throttle {THROTTLE_PCT:.0f}% (PWM {pwm}) for {DURATION:.0f} s")
try:
    t0 = time.time()
    while time.time() - t0 < DURATION:
        throttle(pwm)              # resend: overrides time out after ~1 s
        time.sleep(0.1)
finally:
    throttle(1000)
    time.sleep(0.5)
    m.arducopter_disarm()
    print("Disarmed" if wait_armed(False) else "!! Still armed - disarm in MAVProxy: disarm force")
    throttle(0)                    # release the override back to the RC/GCS
