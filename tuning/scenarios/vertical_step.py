"""Scenario: altitude steps up and down (tuning steps 4-6, vertical loops).

Takeoff to 5 m, then: up to 8 m, back down to 5 m, twice, with a hold after
each step. The horizontal position and the heading are held fixed, so only the
throttle side of the controller works.

The vertical controller is a cascade, tuned inside-out like the attitude one:
  altitude error  --PSC_D_POS_P-->  wanted climb rate
  climb-rate error --PSC_D_VEL_P--> wanted vertical acceleration
  accel error   --PSC_D_ACC_P/I-->  throttle (on top of MOT_THST_HOVER)
GUIDED smooths the step into a ramp (speed/jerk limits), like ATC_INPUT_TC did
for the angles, so the "desired" curves are ramps, not jumps.

What to look at afterwards (plot.ardupilot.org), same axis number for each pair:
  altitude:      CTUN.DAlt vs CTUN.Alt
  climb rate:    CTUN.DCRt vs CTUN.CRt
  acceleration:  PIDA.Tar vs PIDA.Act       (and PIDA.P / PIDA.I terms)
  throttle:      CTUN.ThO (should stay well below 1, no flat-topping)
Good: actual follows desired with a small delay, no overshoot past 8 m / 5 m,
no bounce when the step ends, no throttle buzzing.

Needs the legs in FLY.
"""

import math
import time

from pipeline.flight import arm, land, set_mode, takeoff
from scenarios.hover_step import _current_position, _record, _send_position_target

LOW_M = 5.0
HIGH_M = 8.0
REPEATS = 2
HOLD_S = 8.0     # hold after each step: long enough to see it settle


def run(link, low=LOW_M, high=HIGH_M, repeats=REPEATS, hold_s=HOLD_S):
    samples = []
    set_mode(link, "GUIDED")
    arm(link)
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)
    takeoff(link, low)

    x_m, y_m, _ = _current_position(link)
    att = link.recv_match(type="ATTITUDE", blocking=True, timeout=2)
    yaw_deg = math.degrees(att.yaw) if att else None

    start = time.time()
    _send_position_target(link, x_m, y_m, low, yaw_deg)
    _record(link, "hover", 5.0, low, start, samples)
    for n in range(repeats):
        for target, phase in ((high, f"up{n + 1}"), (low, f"down{n + 1}")):
            print(f"{phase}: altitude target {target:.0f} m")
            _send_position_target(link, x_m, y_m, target, yaw_deg)
            _record(link, phase, hold_s, target, start, samples)

    land(link)
    return samples
