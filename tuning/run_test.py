"""Entry point for the SITL tuning pipeline.

One run does this:
  1. load the baseline and tuning parameter files
  2. connect to SITL through MAVProxy's UDP output
  3. apply the baseline, then the tuning values, and read them back
  4. fly a scenario (for example hover_step)
  5. save the samples, metrics, applied parameters, and the flight log in results/

Usage (from the repo root, with the .venv activated):
    python tuning/run_test.py
    python tuning/run_test.py --config tuning/config/tuning_params.yaml --scenario hover_step
"""

import argparse
import csv
import importlib
import json
import sys
import time
from datetime import datetime
from pathlib import Path

TUNING_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TUNING_DIR))  # lets `pipeline` and `scenarios` import as packages

from pipeline import flight, logs, metrics, plots  # noqa: E402
from pipeline.connect import connect  # noqa: E402
from pipeline.params import apply_params, load_params  # noqa: E402

# MAVProxy's second UDP output. The script uses this port so it doesn't clash with QGroundControl.
DEFAULT_CONNECTION = "udp:127.0.0.1:14551"
DEFAULT_BASELINE = TUNING_DIR / "config" / "baseline_params.yaml"
DEFAULT_TUNING = TUNING_DIR / "config" / "tuning_params.yaml"
DEFAULT_RESULTS = TUNING_DIR / "results"

# These only take effect after a reboot. This script does not reboot yet, so it warns.
REBOOT_REQUIRED = {"FRAME_CLASS", "FRAME_TYPE"}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run one SITL tuning test.")
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE,
                        help="Hardware parameters applied before every run.")
    parser.add_argument("--config", type=Path, default=DEFAULT_TUNING,
                        help="Parameters under test for this run.")
    parser.add_argument("--connection", default=DEFAULT_CONNECTION,
                        help=f"MAVLink connection string (default: {DEFAULT_CONNECTION}).")
    parser.add_argument("--scenario", default="hover_step",
                        help="Scenario module name in tuning/scenarios/ (default: hover_step).")
    parser.add_argument("--results", type=Path, default=DEFAULT_RESULTS,
                        help="Folder where each run's results are saved.")
    parser.add_argument("--log-dir", type=Path, default=logs.DEFAULT_LOG_DIR,
                        help="SITL dataflash log folder.")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip making the plots after the run.")
    return parser.parse_args(argv)


def make_run_dir(results_root, scenario):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(results_root) / f"{stamp}_{scenario}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def save_samples_csv(samples, path):
    if not samples:
        path.write_text("")
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
        writer.writeheader()
        writer.writerows(samples)


def land_safely(link):
    """Best effort after a failure: ask for LAND. Errors here are reported, not raised."""
    try:
        flight.set_mode(link, "LAND")
        print("Failure during the scenario: switched to LAND.")
    except Exception as exc:  # the original error is more important
        print(f"Could not switch to LAND after the failure: {exc}")


def main(argv=None):
    args = parse_args(argv)

    for path in (args.baseline, args.config):
        if not path.exists():
            print(f"Config file not found: {path}", file=sys.stderr)
            return 1

    try:
        scenario = importlib.import_module(f"scenarios.{args.scenario}")
    except ModuleNotFoundError:
        print(f"Scenario not found: scenarios/{args.scenario}.py", file=sys.stderr)
        return 1

    baseline = load_params(args.baseline)
    tuning = load_params(args.config)

    run_dir = make_run_dir(args.results, args.scenario)
    run_info = {
        "started": datetime.now().isoformat(timespec="seconds"),
        "connection": args.connection,
        "scenario": args.scenario,
        "baseline_file": str(args.baseline),
        "tuning_file": str(args.config),
    }

    if REBOOT_REQUIRED & set(baseline):
        print("Warning: " + ", ".join(sorted(REBOOT_REQUIRED & set(baseline)))
              + " need a reboot to take effect. This script does not reboot yet.")

    print(f"Results folder: {run_dir}")
    link = connect(args.connection)

    print("Applying baseline:")
    applied_baseline = apply_params(link, baseline)
    print("Applying tuning values:")
    applied_tuning = apply_params(link, tuning)

    (run_dir / "params_applied.json").write_text(json.dumps(
        {"baseline": applied_baseline, "tuning": applied_tuning}, indent=2))

    print(f"Running scenario '{args.scenario}'...")
    start = time.time()
    try:
        samples = scenario.run(link)
    except Exception as exc:
        land_safely(link)
        run_info["error"] = str(exc)
        (run_dir / "run_info.json").write_text(json.dumps(run_info, indent=2))
        print(f"Scenario failed: {exc}", file=sys.stderr)
        return 1
    run_info["duration_s"] = round(time.time() - start, 1)

    save_samples_csv(samples, run_dir / "samples.csv")

    # scenarios may name the signal they record (default: altitude)
    result = metrics.compute(samples, signal=getattr(scenario, "SIGNAL", "altitude"))
    metrics.save(result, run_dir / "metrics.json")
    print("Metrics:")
    print(metrics.format_table(result))

    # SITL keeps writing the log for a moment after disarm; copying at once gave
    # truncated logs (missing the end of the flight).
    time.sleep(5)
    try:
        latest = logs.find_latest_log(args.log_dir)
        copied = logs.copy_log(latest, run_dir)
        run_info["log_file"] = str(copied)
        print(f"Flight log copied: {copied.name}")
    except FileNotFoundError as exc:
        run_info["log_file"] = None
        print(f"Log not copied: {exc}")

    (run_dir / "run_info.json").write_text(json.dumps(run_info, indent=2))

    if not args.no_plot and run_info.get("log_file"):
        try:
            saved = plots.plot_run(run_dir)
            print(f"Plot saved: {saved.name} (open it in a browser)")
        except Exception as exc:  # plotting must never hide the flight results
            print(f"Plotting failed (results are still saved): {exc}")

    print(f"Done. Results in {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
