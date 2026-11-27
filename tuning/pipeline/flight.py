"""Flight commands: set mode, arm, disarm, take off, land.

Each command is sent as a MAVLink COMMAND_LONG message. The vehicle replies with
COMMAND_ACK saying whether it accepted the command. Takeoff and landing then
watch the vehicle's altitude until it reaches the target.
"""

import time

from pymavlink import mavutil

ACK_TIMEOUT = 5          # seconds to wait for COMMAND_ACK
MODE_FLAG_CUSTOM = 1     # tells ArduPilot to use its own mode numbers
MODE_FLAG_ARMED = 128    # bit in heartbeat base_mode set while armed


def _send_command(link, command, params):
    """Send a COMMAND_LONG and return True if the vehicle accepted it."""
    padded = (list(params) + [0] * 7)[:7]
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        command,
        0,  # confirmation
        *padded,
    )

    end_time = time.time() + ACK_TIMEOUT
    while time.time() < end_time:
        ack = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack is not None and ack.command == command:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    raise TimeoutError(f"No COMMAND_ACK for command {command} within {ACK_TIMEOUT} s")


def set_mode(link, mode_name):
    """Switch the flight mode, for example 'GUIDED', 'LOITER' or 'LAND'."""
    modes = link.mode_mapping()
    if mode_name not in modes:
        raise ValueError(f"Unknown mode {mode_name}. Available: {sorted(modes)}")

    accepted = _send_command(
        link,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        [MODE_FLAG_CUSTOM, modes[mode_name]],
    )
    if not accepted:
        raise RuntimeError(f"Vehicle refused mode {mode_name}")
    _wait_for_mode(link, modes[mode_name])


def _wait_for_mode(link, mode_id, timeout=ACK_TIMEOUT):
    """Wait until a heartbeat reports the requested mode."""
    end_time = time.time() + timeout
    while time.time() < end_time:
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == mode_id:
            return
    raise TimeoutError(f"Vehicle did not switch to mode id {mode_id}")


def is_armed(link, timeout=2):
    """Return True if the latest heartbeat reports the motors as armed."""
    hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
    if hb is None:
        raise TimeoutError("No heartbeat while checking arm state")
    return bool(hb.base_mode & MODE_FLAG_ARMED)


def arm(link):
    """Arm the motors. Fails with an error if a pre-arm check is failing.

    We never force-arm here. If the vehicle refuses, the cause should be fixed,
    not bypassed.
    """
    accepted = _send_command(
        link,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [1],  # 1 = arm
    )
    if not accepted or not _wait_armed_state(link, True):
        raise RuntimeError("Vehicle refused to arm (check pre-arm messages)")


def _wait_armed_state(link, armed, timeout=3):
    """Wait until a heartbeat reports the wanted arm state.

    The first heartbeat read after the command can be an old one, sent before the
    vehicle armed, so a single read can wrongly say "not armed".
    """
    end_time = time.time() + timeout
    while time.time() < end_time:
        try:
            if is_armed(link, timeout=1) == armed:
                return True
        except TimeoutError:
            pass
    return False


def disarm(link):
    """Stop the motors. Only do this on the ground."""
    accepted = _send_command(
        link,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [0],  # 0 = disarm
    )
    if not accepted:
        raise RuntimeError("Vehicle refused to disarm")


def _relative_altitude(link, timeout=2):
    """Return the height above home in metres, or None if no message arrives."""
    msg = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0  # the message gives millimetres


def takeoff(link, altitude_m, tolerance=0.5, timeout=60):
    """Take off to `altitude_m` metres. The vehicle must be armed and in GUIDED."""
    accepted = _send_command(
        link,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        [0, 0, 0, 0, 0, 0, altitude_m],  # param7 is the target altitude
    )
    if not accepted:
        raise RuntimeError("Vehicle refused takeoff (is it GUIDED and armed?)")

    end_time = time.time() + timeout
    while time.time() < end_time:
        alt = _relative_altitude(link)
        if alt is not None and alt >= altitude_m - tolerance:
            return alt
    raise TimeoutError(f"Did not reach {altitude_m} m within {timeout} s")


def land(link, timeout=120):
    """Switch to LAND and wait until the vehicle is on the ground and disarmed."""
    set_mode(link, "LAND")

    end_time = time.time() + timeout
    while time.time() < end_time:
        alt = _relative_altitude(link)
        if alt is not None and alt < 0.3 and not is_armed(link):
            return
    raise TimeoutError(f"Did not land within {timeout} s")
