"""Open a MAVLink connection to the vehicle and wait until it is alive.

The script listens on a UDP port that MAVProxy forwards to. A vehicle sends a
HEARTBEAT message about once per second, so receiving one proves the link works.
"""

from pymavlink import mavutil

# Identifies this script as a ground station to the flight controller.
# 255 is the conventional MAVLink ID for a ground control station.
GCS_SYSTEM_ID = 255


def connect(connection_string, timeout=10):
    """Return a connected MAVLink object once a heartbeat arrives.

    Raises TimeoutError if no heartbeat arrives within `timeout` seconds.
    """
    link = mavutil.mavlink_connection(
        connection_string,
        source_system=GCS_SYSTEM_ID,
    )

    heartbeat = link.wait_heartbeat(timeout=timeout)
    if heartbeat is None:
        raise TimeoutError(
            f"No heartbeat from {connection_string} within {timeout} s. "
            "Check that SITL is running and MAVProxy has this output."
        )

    print(
        f"Connected: system {link.target_system}, "
        f"component {link.target_component}, "
        f"vehicle type {heartbeat.type}"
    )
    return link
