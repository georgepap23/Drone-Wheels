"""Scenario: hover, step the altitude target, hold, land.

The step tests how well the altitude controller follows a sudden change in
target. Only the altitude changes during the step: the horizontal position is
held at the point where the vehicle took off. The recorded samples (altitude,
target, roll, pitch over time) are what the metrics module analyses.
"""

import time

from pymavlink import mavutil

from pipeline.flight import arm, land, set_mode, takeoff

SAMPLE_PERIOD_S = 0.1  # minimum time between recorded samples

# Position-only target: x, y, z are used; velocity, acceleration and yaw are ignored.
POSITION_ONLY_MASK = 0b0000111111111000


def _current_position(link, timeout=2):
    """Return (x, y, z) in local NED metres from the latest position message."""
    msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    if msg is None:
        raise TimeoutError("No LOCAL_POSITION_NED message")
    return msg.x, msg.y, msg.z


def _send_position_target(link, x_m, y_m, altitude_m):
    """Ask the vehicle to hold (x, y) at a given altitude above home in GUIDED mode.

    NED coordinates are used, where z is positive downwards, so altitude is -z.
    Sending the current (x, y) keeps the vehicle where it is horizontally, and
    replaces any older target that might still be active.
    """
    link.mav.set_position_target_local_ned_send(
        0,                                   # time_boot_ms (0 = not used)
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_ONLY_MASK,
        x_m, y_m, -altitude_m,               # position (z is down)
        0.0, 0.0, 0.0,                       # velocity (ignored)
        0.0, 0.0, 0.0,                       # acceleration (ignored)
        0.0, 0.0,                            # yaw, yaw rate (ignored)
    )


def _record(link, phase, seconds, target_m, start_time, samples):
    """Collect samples for `seconds`, tagging each with the phase and target."""
    roll = pitch = None
    last_sample_time = 0.0
    end_time = time.time() + seconds

    while time.time() < end_time:
        msg = link.recv_match(
            type=["ATTITUDE", "LOCAL_POSITION_NED"],
            blocking=True,
            timeout=0.5,
        )
        if msg is None:
            continue

        if msg.get_type() == "ATTITUDE":
            roll, pitch = msg.roll, msg.pitch
            continue

        now = time.time()
        if now - last_sample_time < SAMPLE_PERIOD_S:
            continue
        last_sample_time = now

        samples.append({
            "t": now - start_time,
            "phase": phase,
            "altitude": -msg.z,              # NED z is down, so flip the sign
            "target": target_m,
            "roll": roll,
            "pitch": pitch,
        })


def run(link, hover_altitude=5.0, step_altitude=8.0,
        hover_before_s=5.0, hover_after_s=5.0):
    """Fly the scenario and return the list of samples.

    The vehicle must be disarmed and on the ground when this starts.
    """
    samples = []

    set_mode(link, "GUIDED")
    arm(link)

    # Hold the current position before climbing, so an old target can't pull the vehicle away.
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)

    takeoff(link, hover_altitude)

    # The hover point is wherever the vehicle is after takeoff. The step keeps this (x, y).
    x_m, y_m, _ = _current_position(link)

    start_time = time.time()

    _record(link, "hover", hover_before_s, hover_altitude, start_time, samples)

    _send_position_target(link, x_m, y_m, step_altitude)
    _record(link, "step", hover_after_s, step_altitude, start_time, samples)

    land(link)
    return samples
