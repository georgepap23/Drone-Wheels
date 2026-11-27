"""Scenario: take off and hover still for a long time (tuning step 0).

Used for the prerequisites before any PID tuning:
  - MOT_THST_HOVER: with MOT_HOVER_LEARN = 2 the vehicle learns its hover throttle
    while hovering and saves it on disarm. The value is printed before and after.
  - A clean log of a still hover: gyro noise, motor outputs (RCOU), throttle
    (CTUN.ThO vs CTUN.ThH), and how quiet the rate loop is with no input.

The vehicle holds its takeoff position in GUIDED. Needs the legs in FLY first.
"""

import time

from pipeline.flight import arm, land, set_mode, takeoff
from pipeline.params import get_param
from scenarios.hover_step import _current_position, _record, _send_position_target

HOVER_ALTITUDE_M = 5.0
HOVER_S = 60.0   # ArduPilot recommends at least 30 s of hover to learn the hover throttle


def run(link, hover_altitude=HOVER_ALTITUDE_M, hover_s=HOVER_S):
    samples = []
    before, _ = get_param(link, "MOT_THST_HOVER")
    print(f"MOT_THST_HOVER before: {before:.4f}")

    set_mode(link, "GUIDED")
    arm(link)
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)
    takeoff(link, hover_altitude)

    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, hover_altitude)
    print(f"Hovering for {hover_s:.0f} s...")
    _record(link, "hover", hover_s, hover_altitude, start_time=time.time(), samples=samples)

    land(link)
    after, _ = get_param(link, "MOT_THST_HOVER")
    print(f"MOT_THST_HOVER after:  {after:.4f} (learned value, saved on disarm)")
    return samples

