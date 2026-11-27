"""Scenario: ArduPilot AutoTune in SITL (tuning step 11, cross-check).

AutoTune twitches the vehicle on one axis at a time (roll, pitch, yaw by default,
AUTOTUNE_AXES) and measures the response, raising D and P until it bounces, then
backing off (AUTOTUNE_GMBK). It also sets the angle P and the max rotational
accelerations. It starts from the current gains, so here from our manual tune.

AutoTune is a pilot mode: it reads the RC sticks. With no radio, this script
holds the sticks centred with RC_CHANNELS_OVERRIDE (only channels 1-4; the others,
including the legs on channel 7, are left alone). Sequence:
  GUIDED takeoff to 10 m -> LOITER (so AutoTune also holds position)
  -> AUTOTUNE, wait for "AutoTune: Success" (several minutes)
  -> read the tuned gains (active while still in AUTOTUNE)
  -> descend in AUTOTUNE and disarm on the ground: that is what SAVES the gains
  -> read the saved gains and print them next to the values we started from.

The results are printed and saved in the run folder (autotune_gains.json).
Afterwards every run_test.py run re-applies baseline + config, so the AutoTune
gains are only kept if they are copied into a config file on purpose.

Log: ATUN messages (one per twitch), plus RATE/PIDR/PIDP/PIDY as usual.
Needs the legs in FLY.
"""

import json
import time

from pymavlink import mavutil

from pipeline.flight import arm, disarm, is_armed, set_mode, takeoff
from pipeline.params import get_param
from scenarios.hover_step import _current_position, _send_position_target

SIGNAL = "altitude"
ALTITUDE_M = 10.0          # room for the twitches and some altitude loss
TIMEOUT_S = 20 * 60        # AutoTune usually takes 5-10 minutes in SITL
OVERRIDE_PERIOD_S = 0.1    # RC overrides time out (RC_OVERRIDE_TIME), keep sending

CENTRE = 1500
THROTTLE_HOLD = 1500       # mid stick = hold altitude
THROTTLE_DESCEND = 1250    # below the deadzone = descend
IGNORE = 65535             # RC_CHANNELS_OVERRIDE: leave this channel as it is
RELEASE = 0                # RC_CHANNELS_OVERRIDE: give this channel back to the radio

GAINS = [
    "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
    "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D",
    "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D",
    "ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P",
    "ATC_ACC_R_MAX", "ATC_ACC_P_MAX", "ATC_ACC_Y_MAX",
]


def _sticks(link, throttle):
    """Roll, pitch, yaw centred; throttle as given; channels 5+ untouched."""
    link.mav.rc_channels_override_send(
        link.target_system, link.target_component,
        CENTRE, CENTRE, throttle, CENTRE, IGNORE, IGNORE, IGNORE, IGNORE)


def _release_sticks(link):
    link.mav.rc_channels_override_send(
        link.target_system, link.target_component,
        RELEASE, RELEASE, RELEASE, RELEASE, IGNORE, IGNORE, IGNORE, IGNORE)


def _read_gains(link):
    return {name: round(get_param(link, name)[0], 6) for name in GAINS}


def _hold(link, throttle, seconds, samples, start, phase, until_text=None):
    """Keep the sticks set for `seconds` (or until a STATUSTEXT contains one of
    `until_text`); record altitude; print AutoTune messages. Returns the text found."""
    end = time.time() + seconds
    next_send = 0.0
    while time.time() < end:
        if time.time() >= next_send:
            _sticks(link, throttle)
            next_send = time.time() + OVERRIDE_PERIOD_S
        msg = link.recv_match(type=["STATUSTEXT", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.05)
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            samples.append({"t": time.time() - start, "phase": phase,
                            "altitude": msg.relative_alt / 1000.0, "target": ALTITUDE_M})
        else:
            text = msg.text
            if "AutoTune" in text or "AUTOTUNE" in text.upper():
                print(f"  [{time.time() - start:6.0f} s] {text}")
            if until_text and any(u in text for u in until_text):
                return text
    return None


def _print_table(before, tuned, saved):
    print(f"\n{'parameter':18s} {'ours':>10s} {'AutoTune':>10s} {'saved':>10s}")
    for name in GAINS:
        print(f"{name:18s} {before[name]:10.4f} {tuned[name]:10.4f} {saved[name]:10.4f}")


def run(link, altitude=ALTITUDE_M):
    samples = []
    before = _read_gains(link)

    set_mode(link, "GUIDED")
    arm(link)
    x_m, y_m, _ = _current_position(link)
    _send_position_target(link, x_m, y_m, 0.0)
    takeoff(link, altitude)

    start = time.time()
    try:
        _hold(link, THROTTLE_HOLD, 2, samples, start, "sticks")   # sticks centred before the pilot modes
        set_mode(link, "LOITER")
        _hold(link, THROTTLE_HOLD, 5, samples, start, "loiter")
        set_mode(link, "AUTOTUNE")
        print("AutoTune running (roll, pitch, yaw). This takes several minutes...")
        result = _hold(link, THROTTLE_HOLD, TIMEOUT_S, samples, start, "autotune",
                       until_text=["AutoTune: Success", "AutoTune: Failed", "Failed to level"])
        if result is None:
            raise TimeoutError(f"AutoTune did not finish within {TIMEOUT_S / 60:.0f} min")
        if "Success" not in result:
            raise RuntimeError(f"AutoTune ended with: {result}")

        tuned = _read_gains(link)

        # Descend in AUTOTUNE mode and disarm on the ground: this saves the gains.
        print("Descending in AUTOTUNE to save the gains on disarm...")
        end = time.time() + 120
        while time.time() < end:
            _hold(link, THROTTLE_DESCEND, 1, samples, start, "descend")
            if samples and samples[-1]["altitude"] < 0.2:
                break
        _hold(link, 1000, 3, samples, start, "landed")            # throttle down on the ground
        if is_armed(link):
            disarm(link)
        time.sleep(1)
    finally:
        _release_sticks(link)

    saved = _read_gains(link)
    _print_table(before, tuned, saved)
    run_dirs = sorted((__import__("pathlib").Path(__file__).resolve().parents[1] / "results").glob("*_autotune"))
    if run_dirs:
        (run_dirs[-1] / "autotune_gains.json").write_text(
            json.dumps({"before": before, "autotune": tuned, "saved": saved}, indent=2))
    return samples
