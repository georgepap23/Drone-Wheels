#!/usr/bin/env python3
"""GUIDED takeoff test: arm, climb to ALT, hover HOLD seconds, then LAND.

!! Unfold the legs first (python3 scripts/legs.py).
Prints altitude and roll/pitch once a second so you can see if it's stable.

Usage:  python3 scripts/takeoff.py          # 2 m, hover 10 s
        python3 scripts/takeoff.py 3 15     # 3 m, hover 15 s
"""
import math
import sys
import time
from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5763"
ALT = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

m = mavutil.mavlink_connection(CONN, source_system=255)
m.wait_heartbeat(timeout=10)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)


def armed():
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    while hb is not None and hb.get_srcSystem() != m.target_system:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


state = {"alt": 0.0, "roll": 0.0, "pitch": 0.0}   # last known values


def status(label, seconds):
    """Print alt / roll / pitch for `seconds`; also echo autopilot messages."""
    t0, last = time.time(), time.time()
    while time.time() - t0 < seconds:
        r = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE", "STATUSTEXT"],
                         blocking=True, timeout=1)
        if r is None:
            continue
        if r.get_type() == "GLOBAL_POSITION_INT":
            state["alt"] = r.relative_alt / 1000
        elif r.get_type() == "ATTITUDE":
            state["roll"], state["pitch"] = math.degrees(r.roll), math.degrees(r.pitch)
        else:
            print("  AP:", r.text)
        if time.time() - last >= 1:
            last = time.time()
            print(f"  {label:7s} alt {state['alt']:5.2f} m   "
                  f"roll {state['roll']:+5.1f}   pitch {state['pitch']:+5.1f}")


m.set_mode("GUIDED")
time.sleep(1)
m.arducopter_arm()
t = time.time()
while not armed():
    if time.time() - t > 5:
        sys.exit("Arming failed - see the AP: messages in the MAVProxy console")
print("Armed, taking off to", ALT, "m")

m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                        0, 0, 0, 0, 0, 0, ALT)
try:
    status("climb", 8)
    status("hover", HOLD)
finally:
    print("Landing")
    m.set_mode("LAND")
    t = time.time()
    while armed() and time.time() - t < 30:
        status("land", 2)
    print("Landed and disarmed" if not armed() else "!! Still armed - check MAVProxy")
