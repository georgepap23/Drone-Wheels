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
| `--log-dir` | `external/ardupilot/ArduCopter/logs` | Where SITL writes its flight logs. |
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

## Scenarios

| Scenario | Tuning step | What it flies |
|---|---|---|
| `hover_learn` | 0, prerequisites | Takeoff to 5 m and a 60 s still hover in GUIDED. Prints `MOT_THST_HOVER` before and after (learned while hovering, saved on disarm). |
| `rate_doublet` | 1-2, rate loops | 3 rate doublets per axis (roll and pitch +/-90 deg/s, yaw +/-60 deg/s) sent as body-rate targets in GUIDED, with a level hold between them. Look at the "Rate loop tracking" and "Controller terms" plots. |
| `angle_doublet` | 3, angle loop | 3 angle doublets per axis (roll and pitch +/-15 deg, yaw a 30 deg turn and back) sent as attitude targets in GUIDED, with a position hold between them. Compare ATT.DesRoll/Roll, DesPitch/Pitch, DesYaw/Yaw. |
| `vertical_step` | 4-6, vertical loops | Altitude steps 5 -> 8 -> 5 m, twice, 8 s hold after each, position and heading held. Compare CTUN.DAlt/Alt, CTUN.DCRt/CRt, PIDA.Tar/Act, CTUN.ThO. |
| `horizontal_step` | 7-8, horizontal loops | 5 m position steps North, back, East, back (once), 8 s hold each, altitude and heading held. Compare PSCN.TPN/PN, PSCN.TVN/VN (and PSCE for East), PIDN terms. |
| `wind_gust` | 9, disturbance | Position hold in calm, 3 and 6 m/s steady wind, 30 s of 2-8 m/s gusts, then a 5 m move with a 4 m/s crosswind. Wind set live on the Gazebo topic /ocean_current (needs GZ_IP, Homebrew gz Python bindings and `pip install protobuf==7.36.2` in the venv). Look at PSCN/PSCE position error, ATT.Roll/Pitch, PIDN.I/PIDE.I. |
| `autotune` | 11, cross-check | GUIDED takeoff to 10 m, LOITER, then ArduPilot AUTOTUNE (roll, pitch, yaw) with the sticks held centred by RC override; lands in AUTOTUNE so the gains are saved, prints ours vs AutoTune and writes autotune_gains.json. Takes 5-10+ min. |
| `hover_step` | 4-6, vertical loops | Altitude step 5 m -> 8 m. |

All of them need the legs in FLY first (`rc 7 1900` in MAVProxy). Without MAVProxy's extra
output you can connect to SITL's spare port directly: `--connection tcp:127.0.0.1:5763`.

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
