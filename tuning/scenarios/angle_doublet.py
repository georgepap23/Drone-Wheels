"""Scenario: angle-loop doublets on roll, pitch and yaw (tuning step 3).

Same idea as rate_doublet, one loop higher: instead of commanding a rotation
speed, the script commands a lean ANGLE. Roll/pitch: +A deg for T s, then -A deg
for T s, then level. The +/- halves cancel, so the vehicle ends with almost no
speed and stays near its hover point. Yaw: turn +A deg, hold, turn back.

How it is commanded: GUIDED SET_ATTITUDE_TARGET with the body-rate fields marked
"ignore" and an attitude quaternion. ArduPilot then runs the full attitude
controller: input shaping (ATC_INPUT_TC) -> angle P (ATC_ANG_*_P) -> rate PID
(tuned in steps 1-2). Thrust 0.5 = hold altitude.

What to look at afterwards (plot.ardupilot.org), same axis number for each pair:
  ATT.DesRoll vs ATT.Roll, ATT.DesPitch vs ATT.Pitch, ATT.DesYaw vs ATT.Yaw
Good: the actual angle follows the desired one with a small delay, no overshoot
past it and no wobble when it levels out.

Needs the legs in FLY.
"""

import math
import time

from pymavlink import mavutil

from pipeline.flight import _send_command, arm, land, set_mode, takeoff
from scenarios.hover_step import _current_position, _send_position_target

SIGNAL = "angle"           # metrics key: measured angle on the axis under test (deg)

HOVER_ALTITUDE_M = 5.0
# axis, amplitude (deg), time per half (s)
AXES = [
    ("roll", 15.0, 1.0),
    ("pitch", 15.0, 1.0),
    ("yaw", 30.0, 2.0),
]
REPEATS = 3
AFTER_DOUBLET_S = 1.5      # level / heading held after the doublet, to see it settle
HOLD_S = 4.0               # position hold at the hover point between doublets
SEND_PERIOD_S = 0.02       # 50 Hz command stream (GUIDED times out without updates)
STREAM_HZ = 50
THRUST_HOLD = 0.5          # 0.5 = zero climb rate in GUIDED

MASK_RATES_IGNORE = 0b00000111   # ignore body rates, use the attitude quaternion
ANGLE_FIELD = {"roll": "roll", "pitch": "pitch", "yaw": "yaw"}


def _quaternion(roll_deg, pitch_deg, yaw_deg):
    r, p, y = (math.radians(v) / 2 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def _send_attitude(link, roll_deg, pitch_deg, yaw_deg):
    link.mav.set_attitude_target_send(
        0, link.target_system, link.target_component,
        MASK_RATES_IGNORE, _quaternion(roll_deg, pitch_deg, yaw_deg), 0, 0, 0, THRUST_HOLD)


def _wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class _Recorder:
    """Turns ATTITUDE messages into samples of (target, measured angle) in degrees."""

    def __init__(self, link, start_time):
        self.link = link
        self.start_time = start_time
        self.samples = []
        self.yaw_deg = 0.0

    def drain(self, axis, target_deg, phase):
        while True:
            msg = self.link.recv_match(type="ATTITUDE", blocking=False)
            if msg is None:
                return
            self.yaw_deg = math.degrees(msg.yaw)
            angle = math.degrees(getattr(msg, ANGLE_FIELD[axis]))
            if axis == "yaw":   # report yaw relative to the target, without the 360 wrap
                angle = target_deg + _wrap180(angle - target_deg)
            self.samples.append({
                "t": time.time() - self.start_time,
                "axis": axis,
                "phase": phase,
                "target": target_deg,
                "angle": angle,
            })

    def hold(self, seconds, send, axis, target_deg, phase):
        end = time.time() + seconds
        while time.time() < end:
            send()
            next_send = time.time() + SEND_PERIOD_S
            while time.time() < next_send:
                self.drain(axis, target_deg, phase)
                time.sleep(0.002)


def run(link, hover_altitude=HOVER_ALTITUDE_M, axes=AXES, repeats=REPEATS):
    set_mode(link, "GUIDED")
    arm(link)
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)
    takeoff(link, hover_altitude)

    x0, y0, _ = _current_position(link)
    yaw_hold = math.degrees(link.recv_match(type="ATTITUDE", blocking=True, timeout=2).yaw)
    _send_command(link, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                  [mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1e6 / STREAM_HZ])
    rec = _Recorder(link, time.time())
    rec.drain("roll", 0.0, "start")

    for axis, amplitude, half in axes:
        print(f"{axis}: {repeats} doublets of +/-{amplitude:.0f} deg, {half:.1f} s each half")
        for n in range(repeats):
            rec.hold(HOLD_S, lambda: _send_position_target(link, x0, y0, hover_altitude, yaw_hold),
                     axis, 0.0, "hold")
            yaw0 = rec.yaw_deg   # keep the current heading during roll/pitch doublets
            if axis == "yaw":
                steps = [(+amplitude, half), (0.0, half)]
            else:
                steps = [(+amplitude, half), (-amplitude, half), (0.0, AFTER_DOUBLET_S)]
            for target, seconds in steps:
                angles = {"roll": 0.0, "pitch": 0.0, "yaw": yaw0}
                angles[axis] = yaw0 + target if axis == "yaw" else target
                rec_target = angles[axis] if axis != "yaw" else yaw0 + target
                rec.hold(seconds, lambda a=angles: _send_attitude(link, a["roll"], a["pitch"], a["yaw"]),
                         axis, rec_target, f"doublet{n + 1}")

    rec.hold(HOLD_S, lambda: _send_position_target(link, x0, y0, hover_altitude, yaw_hold),
             "roll", 0.0, "hold")
    land(link)
    return rec.samples
