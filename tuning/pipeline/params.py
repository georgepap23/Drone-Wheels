"""Read, set and verify flight controller parameters over MAVLink.

Parameters are named values stored on the vehicle, such as ATC_RAT_RLL_P.
Setting one needs its type (integer or float), so we read the current type
from the vehicle first and reuse it when writing.
"""

from pathlib import Path

import yaml

PARAM_TIMEOUT = 5  # seconds to wait for each reply


def load_params(path):
    """Load a YAML file of {parameter_name: value} into a dict."""
    with Path(path).open() as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping of parameter names to values")
    return {str(name): float(value) for name, value in data.items()}


def _wait_for_param(link, name, timeout=PARAM_TIMEOUT):
    """Wait for a PARAM_VALUE reply for `name`. Returns the message or raises."""
    while True:
        msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg is None:
            raise TimeoutError(f"No reply for parameter {name} within {timeout} s")
        if msg.param_id.rstrip("\x00") == name:
            return msg


def get_param(link, name):
    """Return (value, mav_type) for one parameter, as reported by the vehicle."""
    link.mav.param_request_read_send(
        link.target_system,
        link.target_component,
        name.encode("ascii"),
        -1,  # -1 means "look up by name, not by index"
    )
    msg = _wait_for_param(link, name)
    return msg.param_value, msg.param_type


def set_param(link, name, value, tolerance=1e-4):
    """Set one parameter and confirm the vehicle stored the value.

    Returns the value the vehicle reports after the write.
    Raises ValueError if the stored value differs from the requested one.
    """
    _, mav_type = get_param(link, name)

    link.mav.param_set_send(
        link.target_system,
        link.target_component,
        name.encode("ascii"),
        float(value),
        mav_type,
    )
    msg = _wait_for_param(link, name)
    stored = msg.param_value

    if abs(stored - float(value)) > tolerance:
        raise ValueError(
            f"{name}: requested {value}, vehicle stored {stored}"
        )
    return stored


def apply_params(link, params):
    """Set every parameter in `params`. Returns {name: stored_value}."""
    results = {}
    for name, value in params.items():
        results[name] = set_param(link, name, value)
    return results
