#!/usr/bin/env python3
"""Unfold (drone mode) or fold (car mode) all 4 Evo-Borne legs at once.

Hips are SERVO5-8 (FL, FR, RL, RR). In model.sdf PWM 1000 = 0 deg (folded),
2000 = 90 deg (unfolded). Uses spare SITL port 5763.

Usage:  python3 scripts/legs.py            # unfold
        python3 scripts/legs.py fold       # fold back
"""
import sys
from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5763"
HIP_SERVOS = [5, 6, 7, 8]        # FL, FR, RL, RR
PWM = {"unfold": 2000, "fold": 1000}

action = sys.argv[1] if len(sys.argv) > 1 else "unfold"
if action not in PWM:
    sys.exit(f"Unknown action '{action}', use: unfold | fold")

m = mavutil.mavlink_connection(CONN, source_system=254)
m.wait_heartbeat(timeout=10)

# Send all 4 back-to-back: they arrive within the same few ms,
# so the legs start moving together.
for s in HIP_SERVOS:
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        s, PWM[action], 0, 0, 0, 0, 0)

# Wait for ArduPilot to accept all 4 (catches lost/rejected commands).
acks = 0
while acks < len(HIP_SERVOS):
    r = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if r is None:
        sys.exit(f"Only {acks}/4 commands acknowledged - is SITL running?")
    if r.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
        if r.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            sys.exit(f"Command rejected (result {r.result})")
        acks += 1

print(f"{action}: SERVO{HIP_SERVOS[0]}-{HIP_SERVOS[-1]} -> {PWM[action]} (accepted)")
