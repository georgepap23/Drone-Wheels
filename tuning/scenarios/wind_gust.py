"""Scenario: hold position in steady wind and gusts (tuning step 9, disturbance test).

The step tests so far only checked how well the vehicle follows commands. This
one checks how well it rejects disturbances: the wind pushes the vehicle away,
and the position/velocity loops (mostly their I terms) must lean into the wind
to stay on the spot.

How the wind is made: the drag model in the Gazebo model (Hydrodynamics plugin on
the chassis) computes drag from the velocity relative to the air. Its "current"
is the air velocity, settable live on the Gazebo topic /ocean_current (world
frame, m/s). This script publishes it through one gz-transport connection kept
open for the whole run (needs GZ_IP like the Gazebo terminals, and Homebrew's gz
Python bindings). A separate `gz topic -p` call per change was tried first and
lost messages: a publisher that exits right after sending can quit before Gazebo
has connected to it. At the end, and on any error, the wind is set back to 0.

Sequence (position and heading held the whole time, 5 m up):
  calm 10 s -> steady 3 m/s 15 s -> steady 6 m/s 15 s -> gusts 2-8 m/s,
  changing direction a little, 30 s -> calm 10 s ->
  a 5 m move and back with a 4 m/s crosswind -> calm -> land

What to look at afterwards (plot.ardupilot.org):
  PSCN.TPN vs PSCN.PN, PSCE.TPE vs PSCE.PE   how far the wind pushes it off the spot
  ATT.Roll, ATT.Pitch                         the lean into the wind
  PIDN.I, PIDE.I                              the I terms learning the wind
  CTUN.ThO                                    throttle (more thrust needed when leaning)
The wind values and their times are in samples.csv ("wind_x", "wind_y").

Needs the legs in FLY.
"""

import math
import random
import sys
import time

from pipeline.flight import arm, land, set_mode, takeoff
from scenarios.hover_step import _current_position, _record, _send_position_target

ALTITUDE_M = 5.0
GUST_SEED = 1          # fixed, so every run gets the same gusts and runs can be compared
TOPIC = "/ocean_current"


GZ_PYTHON_PATH = "/opt/homebrew/lib/python3.12/site-packages"   # Homebrew's gz bindings
_publisher = None


def _wind_publisher():
    """One persistent publisher on the wind topic, connected before first use."""
    global _publisher
    if _publisher is None:
        if GZ_PYTHON_PATH not in sys.path:
            sys.path.append(GZ_PYTHON_PATH)
        from gz.msgs10.vector3d_pb2 import Vector3d
        from gz.transport13 import Node
        node = Node()
        pub = node.advertise(TOPIC, Vector3d)
        end = time.time() + 5
        while not pub.has_connections():
            if time.time() > end:
                raise RuntimeError(f"Gazebo is not subscribed to {TOPIC} (is GZ_IP set, is the sim running?)")
            time.sleep(0.05)
        _publisher = (node, pub, Vector3d)   # keep the node alive, or the publisher closes
    return _publisher


def _set_wind(x_ms, y_ms):
    """Set the air velocity in the Gazebo world frame (m/s)."""
    _, pub, Vector3d = _wind_publisher()
    msg = Vector3d()
    msg.x, msg.y, msg.z = x_ms, y_ms, 0.0
    if not pub.publish(msg):
        raise RuntimeError(f"Publishing the wind on {TOPIC} failed")


def run(link, altitude=ALTITUDE_M):
    samples = []
    wind = {"x": 0.0, "y": 0.0}

    def wind_phase(phase, seconds, x, y):
        _set_wind(x, y)
        wind.update(x=x, y=y)
        print(f"{phase}: wind x {x:+.1f} y {y:+.1f} m/s for {seconds:.1f} s")
        n0 = len(samples)
        _record(link, phase, seconds, altitude, start, samples)
        for s in samples[n0:]:
            s["wind_x"], s["wind_y"] = x, y

    _set_wind(0, 0)   # fails early (before takeoff) if gz is not reachable
    set_mode(link, "GUIDED")
    arm(link)
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)
    takeoff(link, altitude)

    x0, y0, _ = _current_position(link)
    att = link.recv_match(type="ATTITUDE", blocking=True, timeout=2)
    yaw_deg = math.degrees(att.yaw) if att else None
    _send_position_target(link, x0, y0, altitude, yaw_deg)
    start = time.time()

    try:
        wind_phase("calm", 10, 0, 0)
        wind_phase("steady3", 15, 3, 0)
        wind_phase("steady6", 15, 6, 0)

        rng = random.Random(GUST_SEED)
        end = time.time() + 30
        while time.time() < end:
            speed = rng.uniform(2, 8)
            angle = math.radians(rng.uniform(-30, 30))
            wind_phase("gusts", rng.uniform(0.5, 2.0), speed * math.cos(angle), speed * math.sin(angle))

        wind_phase("calm2", 10, 0, 0)

        _set_wind(0, 4)   # crosswind for the move along the other axis
        for name, dn in (("move_out", 5.0), ("move_back", 0.0)):
            _send_position_target(link, x0 + dn, y0, altitude, yaw_deg)
            wind_phase(name, 10, 0, 4)
        wind_phase("calm3", 5, 0, 0)
    finally:
        _set_wind(0, 0)

    land(link)
    return samples
