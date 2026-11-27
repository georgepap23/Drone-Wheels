"""Plot a flight from its dataflash log as one interactive HTML page.

Reads the .BIN log that SITL wrote and saves flight_plots.html in the run folder.
Open it in a browser: zoom and pan work on every panel, the x-axes move together,
and hovering shows the values at that time.

Four figures are in the page:
  state_tracking    reference vs actual for altitude, position, attitude, plus motor outputs
  estimator_health  EKF test ratios, accuracy, and gyro and accelerometer bias
  rate_tracking     desired vs actual roll, pitch and yaw rate
  pid_terms         P/I/D/FF of the rate PIDs and vertical accel PID, throttle vs hover

A SITL log holds every flight since SITL started, so only the last armed period
(from ARM to DISARM) is plotted.

Usage:
    python tuning/pipeline/plots.py tuning/results/<run_folder>
"""

import json
import sys
from pathlib import Path

import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pymavlink.DFReader import DFReader_binary

# Messages and fields we need. Names come from the log of ArduPilot 4.6.0-beta1.
WANTED = {
    "ATT": ["DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw"],
    "RATE": ["RDes", "R", "PDes", "P", "YDes", "Y"],
    "CTUN": ["DAlt", "Alt", "ThO", "ThH"],  # altitude; throttle out and learned hover throttle
    "PSCN": ["DPN"],           # desired north position (controller reference)
    "PSCE": ["DPE"],           # desired east position
    "XKF1": ["PN", "PE", "GX", "GY", "GZ"],  # EKF position estimate and gyro bias
    "XKF2": ["AX", "AY", "AZ"],              # accelerometer bias
    "XKF4": ["SV", "SP", "SH", "SM"],        # innovation test ratios
    "XKF5": ["ePos", "eVel", "eAng"],        # EKF accuracy estimates
    "RCOU": ["C1", "C2", "C3", "C4"],        # motor PWM outputs
    "PIDR": ["P", "I", "D", "FF"],           # rate PID terms (logged at 400 Hz with LOG_BITMASK bit 0)
    "PIDP": ["P", "I", "D", "FF"],
    "PIDY": ["P", "I", "D", "FF"],
    "PIDA": ["P", "I", "D"],                 # vertical acceleration PID
}
PRIMARY_CORE = 0  # the log holds two EKF cores; plot only core 0


def read_log(log_path):
    """Return (data, arm_events).

    data maps 'MSG.FIELD' to (times_s, values). arm_events is a list of (time_s, armed)
    where armed is 1 for ARM and 0 for DISARM.
    """
    log = DFReader_binary(str(log_path))
    data = {}
    arm_events = []
    while True:
        msg = log.recv_match()
        if msg is None:
            break
        name = msg.get_type()
        if not hasattr(msg, "TimeUS"):
            continue
        if name == "ARM":
            arm_events.append((msg.TimeUS / 1e6, int(msg.ArmState)))
            continue
        if name not in WANTED:
            continue
        if getattr(msg, "C", PRIMARY_CORE) != PRIMARY_CORE:
            continue
        t = msg.TimeUS / 1e6
        for field in WANTED[name]:
            times, values = data.setdefault(f"{name}.{field}", ([], []))
            times.append(t)
            values.append(getattr(msg, field))
    return data, arm_events


def last_armed_window(arm_events, data):
    """Return (start_s, end_s) of the last armed period, or the whole log if none."""
    all_times = [times[-1] for times, _ in data.values() if times]
    log_end = max(all_times) if all_times else 0.0
    start = None
    for t, armed in arm_events:
        if armed == 1:
            start = t
        elif armed == 0 and start is not None:
            return start, t
    if start is not None:
        return start, log_end
    first = min((times[0] for times, _ in data.values() if times), default=0.0)
    return first, log_end


def _window(data, start, end):
    """Keep samples inside [start, end] and shift time so the window starts at zero."""
    clipped = {}
    for key, (times, values) in data.items():
        pairs = [(t - start, v) for t, v in zip(times, values) if start <= t <= end]
        clipped[key] = ([p[0] for p in pairs], [p[1] for p in pairs])
    return clipped


REF = dict(dash="dash", color="#e6780b", width=1.6)
ACT = dict(color="#1f77b4", width=1.6)


def _trace(fig, row, data, key, name, style=None):
    times, values = data.get(key, ([], []))
    if not times:
        return
    fig.add_trace(go.Scatter(x=times, y=values, name=name, mode="lines",
                             line=style or {}, legendgroup=f"{row}-{name}"),
                  row=row, col=1)


def _figure(titles, y_titles, height_per_row=210, title=""):
    fig = make_subplots(rows=len(titles), cols=1, shared_xaxes=True,
                        subplot_titles=titles, vertical_spacing=0.035)
    for i, y in enumerate(y_titles, start=1):
        fig.update_yaxes(title_text=y, row=i, col=1)
    fig.update_xaxes(title_text="time since arming (s)", row=len(titles), col=1)
    fig.update_layout(height=height_per_row * len(titles) + 90, title=title,
                      hovermode="x unified", template="plotly_white",
                      legend=dict(orientation="h", y=-0.04 * len(titles) / 7))
    return fig


def figure_state(data):
    titles = ["Altitude", "North position", "East position", "Roll", "Pitch", "Yaw", "Motor outputs"]
    y = ["m", "m", "m", "deg", "deg", "deg", "PWM (us)"]
    fig = _figure(titles, y, title="State tracking: reference (dashed) vs actual")
    _trace(fig, 1, data, "CTUN.DAlt", "reference", REF)
    _trace(fig, 1, data, "CTUN.Alt", "actual", ACT)
    _trace(fig, 2, data, "PSCN.DPN", "reference", REF)
    _trace(fig, 2, data, "XKF1.PN", "actual", ACT)
    _trace(fig, 3, data, "PSCE.DPE", "reference", REF)
    _trace(fig, 3, data, "XKF1.PE", "actual", ACT)
    _trace(fig, 4, data, "ATT.DesRoll", "reference", REF)
    _trace(fig, 4, data, "ATT.Roll", "actual", ACT)
    _trace(fig, 5, data, "ATT.DesPitch", "reference", REF)
    _trace(fig, 5, data, "ATT.Pitch", "actual", ACT)
    _trace(fig, 6, data, "ATT.DesYaw", "reference", REF)
    _trace(fig, 6, data, "ATT.Yaw", "actual", ACT)
    for i in range(1, 5):
        _trace(fig, 7, data, f"RCOU.C{i}", f"motor {i}")
    return fig


def figure_estimator(data):
    titles = ["Innovation test ratios (below the red line = healthy)",
              "EKF accuracy estimates (smaller = more confident)",
              "Estimated gyro bias", "Estimated accelerometer bias"]
    y = ["ratio", "XKF5 units", "rad/s", "m/s^2"]
    fig = _figure(titles, y, height_per_row=240, title="Estimator health (EKF core 0)")
    for key, name in [("XKF4.SV", "velocity"), ("XKF4.SP", "position"),
                      ("XKF4.SH", "height"), ("XKF4.SM", "magnetometer")]:
        _trace(fig, 1, data, key, name)
    fig.add_hline(y=1.0, row=1, col=1, line=dict(color="red", dash="dash"))
    for key, name in [("XKF5.ePos", "position"), ("XKF5.eVel", "velocity"), ("XKF5.eAng", "attitude")]:
        _trace(fig, 2, data, key, name)
    for key, name in [("XKF1.GX", "x"), ("XKF1.GY", "y"), ("XKF1.GZ", "z")]:
        _trace(fig, 3, data, key, name)
    for key, name in [("XKF2.AX", "x"), ("XKF2.AY", "y"), ("XKF2.AZ", "z")]:
        _trace(fig, 4, data, key, name)
    return fig


def figure_rates(data):
    titles = ["Roll rate", "Pitch rate", "Yaw rate"]
    fig = _figure(titles, ["deg/s"] * 3, title="Rate loop tracking: desired (dashed) vs actual")
    _trace(fig, 1, data, "RATE.RDes", "desired", REF)
    _trace(fig, 1, data, "RATE.R", "actual", ACT)
    _trace(fig, 2, data, "RATE.PDes", "desired", REF)
    _trace(fig, 2, data, "RATE.P", "actual", ACT)
    _trace(fig, 3, data, "RATE.YDes", "desired", REF)
    _trace(fig, 3, data, "RATE.Y", "actual", ACT)
    return fig


def figure_pid_terms(data):
    """P, I, D (and FF) contributions of each rate PID, plus throttle.

    Used for the D -> P -> I tuning: a D or P term that rings after a step means that
    gain is too high; an I term that keeps growing means a steady error the P term
    can't remove.
    """
    titles = ["Roll rate PID terms", "Pitch rate PID terms", "Yaw rate PID terms",
              "Vertical accel PID terms", "Throttle: output vs learned hover"]
    fig = _figure(titles, ["output"] * 4 + ["0..1"], title="Controller terms")
    for row, msg in enumerate(["PIDR", "PIDP", "PIDY"], start=1):
        for term in ["P", "I", "D", "FF"]:
            _trace(fig, row, data, f"{msg}.{term}", term)
    for term in ["P", "I", "D"]:
        _trace(fig, 4, data, f"PIDA.{term}", term)
    _trace(fig, 5, data, "CTUN.ThO", "throttle out", ACT)
    _trace(fig, 5, data, "CTUN.ThH", "hover throttle", REF)
    return fig


def plot_log(log_path, out_dir=None):
    """Make the interactive plot page from one log. Returns the saved path."""
    log_path = Path(log_path)
    out_dir = Path(out_dir) if out_dir else log_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    raw, arm_events = read_log(log_path)
    start, end = last_armed_window(arm_events, raw)
    data = _window(raw, start, end)

    figures = [figure_state(data), figure_rates(data), figure_pid_terms(data), figure_estimator(data)]
    parts = []
    for i, fig in enumerate(figures):
        parts.append(fig.to_html(full_html=False, include_plotlyjs=(i == 0)))
    page = (
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Flight plots</title></head>"
        "<body style='font-family:system-ui,sans-serif;margin:16px'>"
        f"<h2>Flight plots: {log_path.name}</h2>"
        f"<p>Window: {end - start:.1f} s, from ARM to DISARM. Zoom with the mouse, double-click to reset.</p>"
        + "".join(f"<div style='margin-bottom:24px'>{p}</div>" for p in parts)
        + "</body></html>"
    )
    out_path = out_dir / "flight_plots.html"
    out_path.write_text(page)
    return out_path


def plot_run(run_dir):
    """Plot the flight log stored in a results folder."""
    run_dir = Path(run_dir)
    info_path = run_dir / "run_info.json"
    log_path = None
    if info_path.exists():
        info = json.loads(info_path.read_text())
        if info.get("log_file"):
            log_path = Path(info["log_file"])
    if log_path is None or not log_path.exists():
        candidates = sorted(run_dir.glob("*.BIN"))
        if not candidates:
            raise FileNotFoundError(f"No flight log found in {run_dir}")
        log_path = candidates[-1]
    return plot_log(log_path, run_dir)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python tuning/pipeline/plots.py <run_folder>", file=sys.stderr)
        sys.exit(1)
    print(f"Saved {plot_run(sys.argv[1])}")
