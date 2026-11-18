# Evo-Borne model generator

`../model.sdf` is **generated** — don't edit it by hand, your changes will be
overwritten. Edit these scripts, then regenerate:

```bash
python3 sim/models/Evo-Borne/generator/build_final_sdf.py   # from the repo root
gz sdf -k sim/models/Evo-Borne/model.sdf                    # validate
```

Needs `numpy` (the repo's `.venv` has it).

| File | What it holds |
|---|---|
| `build_final_sdf.py` | Main script: per-corner poses (`CORNERS`), hip pivots/axes, links, joints, joint friction, colours. Writes `../model.sdf`. |
| `mesh_inertia.py` | Volume, centre of mass and inertia tensor computed from the STL meshes in `../meshes/`. |
| `link_inertials.py` | Real part masses (chassis 668 g, leg 43 g, wheel 49 g, …) → `<inertial>` blocks. |
| `ardupilot_plugins.py` | IMU (BMI088 noise), propeller thrust (LiftDrag fit to EMAX ECO II 2207 + Gemfan 51466), `ArduPilotPlugin` channel map and PIDs for rotors, hip servos (ST3020) and wheels (N20 150:1 + 13.33:1 ring). |

Corner names: `CORNERS` in `build_final_sdf.py` uses the **old** names; the
last step of the script swaps `front_right` ↔ `rear_right` in the output.

## ArduPilot channel map

| SITL output | Joint |
|---|---|
| SERVO1–4 | rotors (quad-X: FR, RL, FL, RR) |
| SERVO5–8 | hips FL, FR, RL, RR — 1000 µs folded, 2000 µs unfolded |
| SERVO9–12 | wheels FL, FR, RL, RR — 1500 µs stop, >1500 forward |
