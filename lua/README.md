# Evo-Borne Lua: transformation + ground autonomy

`evoborne.lua` runs inside ArduCopter and turns it into the hybrid vehicle:

- one RC switch folds/unfolds the legs (DRIVE / FLY), with safety interlocks;
- in DRIVE the normal flight modes become **ground modes**, like ArduRover:
  manual tank drive, AUTO missions, GUIDED "go to here", RTL and HOLD;
- one AUTO mission can **drive, fly and drive again**
  (`WP … TAKEOFF … LAND … WP`).

The same file runs in SITL/Gazebo and on the real MicoAir743v2
(`APM/scripts/` on the SD card). No firmware change and no companion computer
is needed: ArduCopter flies, the script drives.

## Transformation and safety

| Feature | Behaviour |
|---|---|
| Switch (RC7, `RC7_OPTION 300`) | low/mid = DRIVE, high = FLY |
| Transformation | all 4 hips move together over 2 s |
| ARM interlock | ARM refused (`EVO: not in FLY position`) unless legs are fully in FLY and 2 s have passed |
| Armed | switch changes ignored (`EVO: DISARM first!`), legs never move |
| Wheels | only in full DRIVE and disarmed |
| RC failsafe | wheels stop (manual and autonomous) |
| No GPS position | autonomous ground modes stop |
| Boot | waits for the switch in DRIVE before doing anything |
| Pilot override | moving the switch always cancels what the script decided by itself |

## Ground modes (in DRIVE, disarmed)

Select them exactly as you select flight modes (transmitter mode switch,
QGroundControl or MAVProxy `mode …`).

| Copter mode | Ground behaviour (ArduRover equivalent) |
|---|---|
| any manual mode (Stabilize, AltHold, Acro…) | **MANUAL**: pitch stick = forward/back, roll stick = turn |
| Loiter, Brake, PosHold | **HOLD**: stop and stay, sticks ignored |
| Auto | **AUTO**: drive the mission waypoints up to the first TAKEOFF |
| Guided | **GUIDED**: drive to the point you click ("Go to location" in QGC, `guided LAT LON 0` in MAVProxy) |
| RTL, Smart RTL | **RTL**: drive straight back to home |

Progress messages appear in the GCS: `EVO: ground AUTO`,
`EVO: AUTO: item 3, 2.5 m`, `EVO: AUTO: reached item 3`,
`EVO: GUIDED: reached`, … The active mission item is also shown in QGC as
usual.

### Mission items understood on the ground

| Item | Ground behaviour |
|---|---|
| Waypoint | drive there; *Hold* (param1) = wait that many seconds |
| Loiter (time) | drive there and wait *Time* seconds |
| Loiter (unlimited) | drive there and stay |
| Return to launch | drive home, mission ends |
| Change speed | new ground speed (m/s) for the rest of the ground leg |
| **Takeoff** | **end of the ground leg** (see below) |
| other commands | skipped (navigation ones with a warning) |

Altitude of ground waypoints is ignored (use 0).

### Drive → fly → drive in one mission

Example in QGroundControl: `WP1, WP2` (ground), `Takeoff 5 m`, `WP3`
(air), `Land`, `WP4` (ground).

1. DRIVE, mode **Auto**: drives WP1, WP2 and stops at Takeoff
   (`EVO: ground leg done: switch FLY, then ARM`).
2. Switch to **FLY**: legs unfold, `ready to ARM`.
3. **ARM** (still in Auto): takes off and flies the mission
   (`AUTO_OPTIONS 3` allows arming in Auto without raising the throttle).
4. After Land it disarms. Switch to **DRIVE**: legs fold, it continues
   with WP4 (`EVO: AUTO: continue at item …`).

With `EVO_AUTO_FLY = 1` steps 2–4 happen by themselves: at Takeoff the script
unfolds the legs, waits until ready, counts 3 s and arms; after the landing it
folds the legs and drives on. Moving the switch at any time cancels this and
gives control back to the pilot. Keep it at 0 until the manual sequence works
reliably on the real vehicle.

## Parameters

They appear in the GCS parameter list as `EVO_*` once the script runs
(refresh the list). Values are saved like any other parameter.

| Parameter | Default | Meaning |
|---|---|---|
| `EVO_SPD_MAX` | 0.14 | ground speed (m/s) at full wheel command. **Measure it** (below) |
| `EVO_SPEED` | 0.12 | cruise speed in AUTO/GUIDED/RTL (like `WP_SPEED`) |
| `EVO_WP_RADIUS` | 1.5 | waypoint reached radius (m). GPS error is 1–2 m, keep ≥ 1.5 |
| `EVO_TURN_P` | 1.2 | steering gain (turn command per rad of heading error) |
| `EVO_TURN_MAX` | 0.8 | max turn command (0–1) |
| `EVO_LOOKAHEAD` | 2.0 | path following look-ahead (m): larger = smoother, cuts corners more |
| `EVO_SPD_P` | 0.5 | speed controller gain |
| `EVO_AUTO_FLY` | 0 | 1 = transform, arm and take off by itself at Takeoff; fold and drive on after Land |

`params/evo_borne_hybrid.parm` also sets `AUTO_OPTIONS 3` (arm in Auto, no
throttle needed for the take-off).

### Tuning on the vehicle (or in Gazebo)

1. **`EVO_SPD_MAX`**: in manual DRIVE push the pitch stick fully forward on a
   straight line for 10 s and read the ground speed in the GCS. Set
   `EVO_SPD_MAX` to that value, and `EVO_SPEED` a bit lower.
2. **Steering**: GUIDED to a point 10 m away, at an angle. If it weaves
   left-right, lower `EVO_TURN_P` (or raise `EVO_LOOKAHEAD`); if it turns too
   slowly and overshoots, raise `EVO_TURN_P`.
3. **Mission**: a square of 4 waypoints 10 m apart, then RTL.

## Install (once)

SITL looks for Lua scripts in `scripts/` next to where it runs. Link the file
there, so edits in `lua/` are picked up without copying (from the repo root):

```bash
mkdir -p ardupilot/ArduCopter/scripts
ln -sf ../../../lua/evoborne.lua ardupilot/ArduCopter/scripts/evoborne.lua
```

Only this script may be in that folder (an older `transform.lua` would fight
over the same servos).

## Run in Gazebo

Terminal 1 and 3 as in the main README (Gazebo server, then GUI). Terminal 2:

```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map \
    --add-param-file=../../params/evo_borne_copter.parm \
    --add-param-file=../../params/evo_borne_hybrid.parm
```

Wait for `EVO: loaded` and `EVO: ready (DRIVE)`.

`--add-param-file` only sets *defaults*: values already saved in SITL's
`eeprom.bin` win. After updating the parameter files, start once with `-w` to
wipe the saved values, or run `param load ../../params/evo_borne_hybrid.parm`
in MAVProxy and then `reboot` (`SCR_ENABLE` only takes effect after a reboot).

## Manual test (MAVProxy, one command at a time)

| Command | Expected |
|---|---|
| `rc 2 1300` | drives forward |
| `rc 1 1700` | turns right |
| `rc 1 1500` then `rc 2 1500` | stops |
| `mode guided` then `guided -35.36318 149.16524 0` (a point ~10 m north of the runway home) | `EVO: ground GUIDED`, drives there, `EVO: GUIDED: reached` |
| `mode rtl` | drives back home |
| `mode loiter` | HOLD: stops |
| `arm throttle` (in DRIVE) | refused: `EVO: not in FLY position` |
| `rc 7 1900` | legs unfold in 2 s, then `ready to ARM` |
| `mode guided`, `arm throttle`, `takeoff 5` | climbs to 5 m |
| `rc 7 1100` (in the air) | `EVO: DISARM first! request ignored`, legs stay open |
| `mode land` | lands and disarms, then folds to DRIVE (switch is already at DRIVE) |

On the MAVProxy map you can also right-click → *Fly to* for GUIDED, and
load/draw a mission with `wp load` / the mission editor; in QGroundControl use
the Plan view.

## Automated tests

With SITL running in Gazebo, from the repo root (spare port 5763; close
QGroundControl's virtual joystick first, it sends its own RC overrides):

```bash
python3 tests/test_evoborne_lua.py        # transformation + interlocks, 23 checks, ~2 min
python3 tests/test_ground_autonomy.py     # ground modes + drive/fly/drive mission, ~10-15 min
```

Restart SITL between the two tests. Both wait for the script's messages
rather than fixed delays, so a slow Gazebo (real-time factor < 1) is fine.
`test_ground_autonomy.py` needs a simulator in which the wheels move the
vehicle (Gazebo with the Evo-Borne model), and uploads its own small mission
(4 m legs) around home.

## Limits compared with ArduRover

- RTL drives straight home (no Smart RTL breadcrumbs), no obstacle avoidance,
  no pivot turns.
- Needs a GPS position: outdoors, or another position source.
- Steering and speed control are simple (pure pursuit + P speed loop); tune
  with the `EVO_*` parameters above.

## Before the real vehicle

- Remove the propellers for every bench test, including all ground-mode tests.
- Calibrate the hip PWM in `HIPS` (DRIVE/FLY values per servo). Start with
  1400-1600 and widen slowly; mirrored servos may need DRIVE and FLY swapped.
- Check the wheel direction per side and swap the sign in `set_wheels` if a
  side runs backwards. In GUIDED check that it turns *towards* the target.
- Calibrate the compass with the wheel motors running if they are close to it:
  ground navigation steers by the compass heading.
- On the FC use one output per side (`SERVO9_FUNCTION 98`, `SERVO10_FUNCTION 99`)
  and set `SERVO11_FUNCTION 0`, `SERVO12_FUNCTION 0`, unless those outputs are
  really wired to wheels.
- RC loss stops the wheels only if ArduPilot sees the failsafe. With ELRS over
  CRSF this works by default; never set the receiver to "hold last position".
- Keep `EVO_AUTO_FLY = 0` until the manual drive → fly sequence is reliable.
