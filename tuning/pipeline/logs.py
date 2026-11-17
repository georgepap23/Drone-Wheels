"""Find the dataflash log SITL wrote for the latest flight and copy it to results.

SITL writes numbered binary logs (00000001.BIN, 00000002.BIN, ...) into its
logs folder. LASTLOG.TXT holds the number of the most recent one. We use it
when present, and otherwise fall back to the highest-numbered file.
"""

import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_LOG_DIR = REPO_ROOT / "ardupilot" / "ArduCopter" / "logs"


def find_latest_log(log_dir=DEFAULT_LOG_DIR):
    """Return the path to the most recent .BIN log in `log_dir`."""
    log_dir = Path(log_dir)
    if not log_dir.is_dir():
        raise FileNotFoundError(f"Log folder not found: {log_dir}")

    last_log_file = log_dir / "LASTLOG.TXT"
    if last_log_file.exists():
        number = int(last_log_file.read_text().strip())
        candidate = log_dir / f"{number:08d}.BIN"
        if candidate.exists():
            return candidate

    logs = sorted(log_dir.glob("*.BIN"))
    if not logs:
        raise FileNotFoundError(f"No .BIN logs found in {log_dir}")
    return logs[-1]


def copy_log(log_path, results_dir):
    """Copy a log into `results_dir` and return the new path."""
    results_dir = Path(results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    destination = results_dir / Path(log_path).name
    shutil.copy2(log_path, destination)
    return destination
