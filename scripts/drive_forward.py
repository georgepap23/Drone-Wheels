#!/usr/bin/env python3
"""Drive Evo-Borne on its 4 wheels for a few seconds (SITL test, Copter firmware).

  python3 scripts/drive_forward.py fwd              # straight ahead
  python3 scripts/drive_forward.py str left         # curve left
  python3 scripts/drive_forward.py str right 8      # curve right for 8 s

Wheels are SERVO9-12 (FL, FR, RL, RR). In model.sdf each wheel maps
PWM 1000..2000 -> -1.728..+1.728 rad/s, so 1500 = stop, >1500 = forward.
Steering = skid steer: the outer side runs faster than the inner side
(no pivot on the spot: the track is narrower than the wheelbase).
In Rover firmware SERVO9-12 belong to the autopilot, so use RC/GUIDED there.
Uses spare SITL port 5763 so MAVProxy/QGC on 5760 keep working.
"""
import sys
import time
from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5763"
LEFT_SERVOS = [9, 11]            # FL, RL
RIGHT_SERVOS = [10, 12]          # FR, RR
PWM_FORWARD = 1650               # low speed: 30% of max (~0.5 rad/s)
PWM_OUTER = 1750                 # steering: outer side faster
PWM_INNER = 1550                 #           inner side slower
PWM_STOP = 1500
USAGE = "usage: drive_forward.py fwd [seconds] | str left|right [seconds]"


def set_servo(m, servo, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        servo, pwm, 0, 0, 0, 0, 0)


def set_sides(m, left_pwm, right_pwm):
    for s in LEFT_SERVOS:
        set_servo(m, s, left_pwm)
    for s in RIGHT_SERVOS:
        set_servo(m, s, right_pwm)


args = sys.argv[1:]
mode = args.pop(0) if args else "fwd"
if mode == "fwd":
    left, right, label = PWM_FORWARD, PWM_FORWARD, "Forward"
elif mode == "str" and args and args[0] in ("left", "right"):
    side = args.pop(0)
    if side == "left":
        left, right = PWM_INNER, PWM_OUTER
    else:
        left, right = PWM_OUTER, PWM_INNER
    label = f"Steer {side}"
else:
    sys.exit(USAGE)
duration = float(args[0]) if args else 5.0

m = mavutil.mavlink_connection(CONN, source_system=254)
m.wait_heartbeat(timeout=10)
print(f"Connected to system {m.target_system}")

try:
    print(f"{label}: left PWM {left}, right PWM {right} for {duration:.0f} s")
    set_sides(m, left, right)
    time.sleep(duration)
finally:
    set_sides(m, PWM_STOP, PWM_STOP)
    print("Stopped")
