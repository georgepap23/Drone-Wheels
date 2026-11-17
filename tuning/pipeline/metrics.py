"""Compute tracking metrics for any scenario, without knowing the scenario's name.

A scenario only has to return samples, each a dict with:
    t          time in seconds
    <signal>   the measured value, for example 'altitude' (default)
    <target>   the commanded value at that time, for example 'target'
    roll, pitch  attitude in radians (optional, used for the attitude effort)

The metrics are found from the data:
  - Segments: stretches where the commanded target stays constant. A change in the
    target starts a new segment.
  - Steps: every segment after the first one. Each step gets rise time, overshoot,
    settling time and steady-state error.
  - Holds: every segment gets an RMS error once its first HOLD_SETTLE_S seconds have passed.

So a new scenario that changes the target in any way gets metrics automatically.
"""

import json
import math
from pathlib import Path

SETTLE_BAND = 0.05         # settled = within 5% of the step size
STEADY_WINDOW_S = 3.0      # steady-state error is averaged over the last 3 s of a segment
HOLD_SETTLE_S = 2.0        # ignore the first 2 s of each segment while the vehicle settles
TARGET_CHANGE = 0.1        # a target change bigger than this starts a new segment (same units as signal)
MIN_STEP = 0.1             # steps smaller than this are reported but not analysed


def _rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def _mean(values):
    return sum(values) / len(values) if values else None


def _split_segments(samples, target_key):
    """Split the samples into runs where the target does not change."""
    segments = [[samples[0]]]
    for prev, current in zip(samples, samples[1:]):
        if abs(current[target_key] - prev[target_key]) > TARGET_CHANGE:
            segments.append([])
        segments[-1].append(current)
    return segments


def _analyse_step(segment, pre_sample, signal, target_key):
    """Metrics for one step.

    `pre_sample` is the last sample before the change, so progress is measured from
    where the vehicle actually was when the target changed.
    """
    previous_target = pre_sample[target_key]
    new_target = segment[0][target_key]
    step_size = new_target - previous_target
    result = {
        "t_start": segment[0]["t"],
        "from_target": previous_target,
        "to_target": new_target,
        "step_size": step_size,
        "notes": [],
    }
    if abs(step_size) < MIN_STEP:
        result["notes"].append("step too small to analyse")
        return result

    window = [pre_sample] + segment

    def progress(value):
        """0 = at the old target, 1 = at the new target."""
        return (value - previous_target) / step_size

    t10 = next((s["t"] for s in window if progress(s[signal]) >= 0.1), None)
    t90 = next((s["t"] for s in window if progress(s[signal]) >= 0.9), None)
    result["rise_time_s"] = (t90 - t10) if (t10 is not None and t90 is not None) else None
    if result["rise_time_s"] is None:
        result["notes"].append("never reached 90% of the step")

    peak = max(progress(s[signal]) for s in window)
    result["overshoot_pct"] = max(0.0, (peak - 1.0) * 100.0)

    band = SETTLE_BAND * abs(step_size)
    outside = [s["t"] for s in segment if abs(s[signal] - new_target) > band]
    if not outside:
        result["settling_time_s"] = 0.0
    else:
        after = [s["t"] for s in segment if s["t"] > outside[-1]]
        result["settling_time_s"] = (after[0] - segment[0]["t"]) if after else None
        if not after:
            result["notes"].append("did not settle within the segment")

    end = segment[-1]["t"]
    tail = [s[signal] - new_target for s in segment if s["t"] >= end - STEADY_WINDOW_S]
    result["steady_state_error"] = _mean(tail)
    return result


def compute(samples, signal="altitude", target_key="target"):
    """Return metrics for any scenario that produced samples in the standard format."""
    if not samples:
        return {"notes": ["no samples recorded"]}
    if signal not in samples[0] or target_key not in samples[0]:
        return {"notes": [f"samples have no '{signal}' or '{target_key}' field"]}

    segments = _split_segments(samples, target_key)

    segment_results = []
    steps = []
    for index, segment in enumerate(segments):
        hold = [s for s in segment if s["t"] >= segment[0]["t"] + HOLD_SETTLE_S]
        target = segment[0][target_key]
        errors = [s[signal] - target for s in hold]
        segment_results.append({
            "t_start": segment[0]["t"],
            "target": target,
            "duration_s": segment[-1]["t"] - segment[0]["t"],
            "rms_error_after_settle": _rms(errors),
        })
        if index > 0:
            steps.append(_analyse_step(segment, segments[index - 1][-1], signal, target_key))

    roll = [abs(math.degrees(s["roll"])) for s in samples if s.get("roll") is not None]
    pitch = [abs(math.degrees(s["pitch"])) for s in samples if s.get("pitch") is not None]

    settling = [s["settling_time_s"] for s in steps if s.get("settling_time_s") is not None]
    any_unsettled = any(s.get("settling_time_s") is None for s in steps)
    overshoots = [s["overshoot_pct"] for s in steps if "overshoot_pct" in s]
    steady = [abs(s["steady_state_error"]) for s in steps if s.get("steady_state_error") is not None]
    holds = [g["rms_error_after_settle"] for g in segment_results if g["rms_error_after_settle"] is not None]

    summary = {
        "signal": signal,
        "num_steps": len(steps),
        "worst_overshoot_pct": max(overshoots) if overshoots else None,
        "worst_settling_time_s": None if (any_unsettled or not settling) else max(settling),
        "worst_steady_state_error": max(steady) if steady else None,
        "worst_hold_rms_error": max(holds) if holds else None,
        "peak_roll_deg": max(roll) if roll else None,
        "peak_pitch_deg": max(pitch) if pitch else None,
    }
    notes = [f"step at t={s['t_start']:.1f} s: {n}" for s in steps for n in s["notes"]]
    if not steps:
        notes.append("no target changes found, so no steps were analysed")

    return {"summary": summary, "steps": steps, "segments": segment_results, "notes": notes}


def save(metrics, path):
    """Write metrics to a JSON file and return the path."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(metrics, indent=2))
    return path


def _fmt(value):
    if value is None:
        return "n/a"
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def format_table(metrics):
    """Return a readable summary of the metrics."""
    lines = []
    summary = metrics.get("summary", {})
    for key, value in summary.items():
        lines.append(f"  {key:<24} {_fmt(value)}")
    for i, step in enumerate(metrics.get("steps", []), start=1):
        lines.append(f"  step {i} at t={step['t_start']:.1f} s "
                     f"({_fmt(step.get('from_target'))} -> {_fmt(step.get('to_target'))}):")
        for key in ("rise_time_s", "overshoot_pct", "settling_time_s", "steady_state_error"):
            lines.append(f"    {key:<22} {_fmt(step.get(key))}")
    for note in metrics.get("notes", []):
        lines.append(f"  note: {note}")
    return "\n".join(lines)
