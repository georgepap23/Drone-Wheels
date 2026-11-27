"""Scenario: rate-loop doublets on roll, pitch and yaw (tuning steps 1-2).

A doublet is a short rate command in one direction followed by the same command
in the other direction: +A deg/s for T seconds, then -A deg/s for T seconds, then 0.
The angle comes back to where it started, so the vehicle stays near level, and
the rate loop sees two sharp steps in a row. That is the input used to tune the
rate PID with the D -> P -> I approach.

How it is commanded: GUIDED mode accepts SET_ATTITUDE_TARGET. With the attitude
field marked "ignore" and the three body rates filled in, ArduPilot sends the
rates straight to the rate controller (ModeGuided::angle_control_run). Thrust 0.5
means "zero climb rate", so the altitude loop holds height during the test.
Between doublets the vehicle flies back to its hover point and holds position, so
every doublet starts from rest at the same place. (Holding only a level attitude let
the small leftover speed of each doublet add up: with no air drag on the body in
Gazebo the vehicle glided 200+ m away.)

What to look at afterwards (flight_plots.html, or plot.ardupilot.org):
  RATE.RDes vs RATE.R, RATE.PDes vs RATE.P, RATE.YDes vs RATE.Y
  PIDR/PIDP/PIDY P, I, D terms, and RCOU for motor saturation.
The samples below (target vs measured rate at ~50 Hz) give rough step metrics;
the 400 Hz log is the real data for tuning.

Needs the legs in FLY and LOG_BITMASK with bit 0 (Fast Attitude) set.
"""

import math
import time

from pymavlink import mavutil

from pipeline.flight import _send_command, arm, land, set_mode, takeoff
from scenarios.hover_step import _current_position, _send_position_target

SIGNAL = "rate"            # metrics key: measured rate on the axis under test (deg/s)

HOVER_ALTITUDE_M = 5.0
# axis, amplitude (deg/s), half-period (s). 90 deg/s x 0.25 s = 22.5 deg peak lean.
AXES = [
    ("roll", 90.0, 0.25),
    ("pitch", 90.0, 0.25),
    ("yaw", 60.0, 0.5),
]
REPEATS = 3                # doublets per axis
AFTER_DOUBLET_S = 1.0      # rate 0 after the doublet, to see the loop settle
HOLD_S = 4.0               # position hold at the hover point between doublets
SEND_PERIOD_S = 0.02       # 50 Hz command stream (GUIDED times out without updates)
STREAM_HZ = 50             # requested ATTITUDE message rate
THRUST_HOLD = 0.5          # 0.5 = zero climb rate in GUIDED

# SET_ATTITUDE_TARGET type_mask bits
MASK_ATTITUDE_IGNORE = 0b10000000  # use body rates, ignore attitude
AXIS_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}
RATE_FIELD = {"roll": "rollspeed", "pitch": "pitchspeed", "yaw": "yawspeed"}


def _request_attitude_stream(link, hz):
    _send_command(link, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                  [mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 1e6 / hz])


def _send_rates(link, rates_deg):
    r, p, y = (math.radians(v) for v in rates_deg)
    link.mav.set_attitude_target_send(
        0, link.target_system, link.target_component,
        MASK_ATTITUDE_IGNORE, [1, 0, 0, 0], r, p, y, THRUST_HOLD)


class _Recorder:
    """Keeps the latest attitude and turns ATTITUDE messages into samples."""

    def __init__(self, link, start_time):
        self.link = link
        self.start_time = start_time
        self.samples = []
        self.yaw = 0.0

    def drain(self, axis, target_deg, phase):
        while True:
            msg = self.link.recv_match(type="ATTITUDE", blocking=False)
            if msg is None:
                return
            self.yaw = msg.yaw
            self.samples.append({
                "t": time.time() - self.start_time,
                "axis": axis,
                "phase": phase,
                "target": target_deg,
                "rate": math.degrees(getattr(msg, RATE_FIELD[axis])),
                "roll": msg.roll,
                "pitch": msg.pitch,
            })

    def hold(self, seconds, send, axis, target_deg, phase):
        """Send a command every SEND_PERIOD_S for `seconds`, recording meanwhile."""
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

    x0, y0, _ = _current_position(link)       # hover point: every doublet starts here
    yaw_hold = math.degrees(link.recv_match(type="ATTITUDE", blocking=True, timeout=2).yaw)
    _request_attitude_stream(link, STREAM_HZ)
    rec = _Recorder(link, time.time())
    rec.drain("roll", 0.0, "start")

    for axis, amplitude, half in axes:
        print(f"{axis}: {repeats} doublets of +/-{amplitude:.0f} deg/s, {half:.2f} s each half")
        i = AXIS_INDEX[axis]
        for n in range(repeats):
            rec.hold(HOLD_S, lambda: _send_position_target(link, x0, y0, hover_altitude, yaw_hold),
                     axis, 0.0, "hold")
            for sign in (+1, -1):
                rates = [0.0, 0.0, 0.0]
                rates[i] = sign * amplitude
                rec.hold(half, lambda r=rates: _send_rates(link, r), axis,
                         sign * amplitude, f"doublet{n + 1}")
            rec.hold(AFTER_DOUBLET_S, lambda: _send_rates(link, [0, 0, 0]), axis, 0.0, "settle")

    # back to the hover point before landing
    rec.hold(HOLD_S, lambda: _send_position_target(link, x0, y0, hover_altitude, yaw_hold),
             "roll", 0.0, "hold")

    land(link)
    return rec.samples
