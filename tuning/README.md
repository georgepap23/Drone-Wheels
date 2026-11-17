# SITL tuning pipeline

Runs a flight scenario in SITL, saves the data, computes metrics, and plots the results. Use it to compare parameter sets for the controllers and the EKF.

## Before you run

1. Start Gazebo (terminal 1 and 2) and SITL with MAVProxy (terminal 3), as in the main README.
2. In the MAVProxy prompt, add the script's output: `output add 127.0.0.1:14551`
3. Close anything else sending commands to the vehicle, such as QGroundControl's virtual joystick.

## Running a test

From the repo root, with the `.venv` active:

```bash
python tuning/run_test.py
```

Arguments (all optional):

| Argument | Default | What it does |
|---|---|---|
| `--scenario` | `hover_step` | Which flight to fly. Matches a file in `tuning/scenarios/`. |
| `--config` | `config/tuning_params.yaml` | The parameters under test for this run. Change values here to tune. |
| `--baseline` | `config/baseline_params.yaml` | Hardware parameters applied before every run. |
| `--connection` | `udp:127.0.0.1:14551` | MAVLink link to SITL. Change only if you change MAVProxy's output port. |
| `--results` | `tuning/results/` | Folder where each run is saved. |
| `--log-dir` | `ardupilot/ArduCopter/logs` | Where SITL writes its flight logs. |
| `--no-plot` | off | Skip making the plots after the run. |

Example: fly a second scenario with a different tuning file:

```bash
python tuning/run_test.py --scenario hover_step --config tuning/config/my_tuning.yaml
```

## What a run saves

Each run creates a folder `tuning/results/<time>_<scenario>/` with:

- `metrics.json`: the performance numbers (overshoot, settling time, errors).
- `samples.csv`: the recorded data from the scenario.
- `params_applied.json`: the values the vehicle confirmed.
- `run_info.json`: the settings and paths for the run.
- The flight log (`.BIN`) and `flight_plots.html`: interactive plots of the flight. Open it in a browser and zoom with the mouse; double-click to reset.

## Re-plotting an old run

```bash
python tuning/pipeline/plots.py tuning/results/<run_folder>
```

## Adding a scenario

Create `tuning/scenarios/<name>.py` with a `run(link)` function that returns a list of samples. Each sample is a dict with `t`, `altitude` (or another signal), and `target`. The metrics detect the steps from the changes in `target`, so no change to `metrics.py` is needed.

## Tuning the parameters

1. Edit values in a copy of `tuning_params.yaml`. Each parameter has a comment describing it.
2. Run the same scenario again with the new file.
3. Compare the `summary` section of each run's `metrics.json`.
4. Check the plots for oscillation, overshoot, or sensor rejection that the summary might not show.

Keep the baseline file fixed while you tune, so only the tuning values change between runs.

## Files

- `run_test.py`: entry point.
- `pipeline/`: connection, parameters, flight commands, logs, metrics, and plots.
- `scenarios/`: flight manoeuvres.
- `config/`: parameter files.
- `results/`: output of each run.
