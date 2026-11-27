"""Scenario: horizontal position steps (tuning steps 7-8, horizontal loops).

Takeoff to 5 m, then fly 5 m North, back, 5 m East, back (once), holding each
point for 8 s. Altitude and heading are held fixed, so only the horizontal
controller works (it leans the vehicle to move, through the attitude loops tuned
in steps 1-3).

The horizontal controller is the same kind of cascade as the vertical one:
  position error --PSC_NE_POS_P-->       wanted velocity
  velocity error --PSC_NE_VEL_P/I/D-->   wanted acceleration -> lean angle
GUIDED smooths each step into a speed/jerk-limited path, so the desired curves
are smooth S-shapes, not jumps.

What to look at afterwards (plot.ardupilot.org), same axis number for each pair:
  position North: PSCN.TPN vs PSCN.PN     (East: PSCE.TPE vs PSCE.PE)
  velocity North: PSCN.TVN vs PSCN.VN     (East: PSCE.TVE vs PSCE.VE)
  acceleration:   PSCN.TAN vs PSCN.AN
  velocity PID:   PIDN.P / PIDN.I / PIDN.D
Good: actual follows target with a small delay, stops at the point without
overshooting it, and does not swing back and forth while holding.

Needs the legs in FLY.
"""

import math

from pipeline.flight import arm, land, set_mode, takeoff
from scenarios.hover_step import _current_position, _record, _send_position_target
import time

ALTITUDE_M = 5.0
DISTANCE_M = 5.0
REPEATS = 1
HOLD_S = 8.0


def run(link, altitude=ALTITUDE_M, distance=DISTANCE_M, repeats=REPEATS, hold_s=HOLD_S):
    samples = []
    set_mode(link, "GUIDED")
    arm(link)
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)
    takeoff(link, altitude)

    x0, y0, _ = _current_position(link)
    att = link.recv_match(type="ATTITUDE", blocking=True, timeout=2)
    yaw_deg = math.degrees(att.yaw) if att else None

    start = time.time()
    _send_position_target(link, x0, y0, altitude, yaw_deg)
    _record(link, "hover", 5.0, altitude, start, samples)

    moves = [("north", distance, 0.0), ("back", 0.0, 0.0),
             ("east", 0.0, distance), ("back", 0.0, 0.0)]
    for n in range(repeats):
        for name, dn, de in moves:
            print(f"{name}{n + 1}: target {dn:+.0f} m N, {de:+.0f} m E of the hover point")
            _send_position_target(link, x0 + dn, y0 + de, altitude, yaw_deg)
            _record(link, f"{name}{n + 1}", hold_s, altitude, start, samples)

    land(link)
    return samples
