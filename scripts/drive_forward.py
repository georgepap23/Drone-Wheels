#!/usr/bin/env python3
"""Drive Evo-Borne forward on its 4 wheels for a few seconds (SITL test).

Wheels are SERVO9-12 (FL, FR, RL, RR). In model.sdf each wheel maps
PWM 1000..2000 -> -1.728..+1.728 rad/s, so 1500 = stop, >1500 = forward.
Uses spare SITL port 5763 so MAVProxy/QGC on 5760 keep working.
"""
import time
from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5763"
WHEEL_SERVOS = [9, 10, 11, 12]   # FL, FR, RL, RR
PWM_FORWARD = 1650               # low speed: 30% of max (~0.5 rad/s)
PWM_STOP = 1500
DURATION = 5.0                   # seconds


def set_servo(m, servo, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        servo, pwm, 0, 0, 0, 0, 0)


def set_wheels(m, pwm):
    for s in WHEEL_SERVOS:
        set_servo(m, s, pwm)


m = mavutil.mavlink_connection(CONN, source_system=254)
m.wait_heartbeat(timeout=10)
print(f"Connected to system {m.target_system}")

try:
    print(f"Forward at PWM {PWM_FORWARD} for {DURATION:.0f} s")
    set_wheels(m, PWM_FORWARD)
    time.sleep(DURATION)
finally:
    set_wheels(m, PWM_STOP)
    print("Stopped")
