# Evo-Borne Lua: DRIVE/FLY transformation

`evoborne.lua` runs inside ArduCopter and turns it into the hybrid vehicle:
one RC switch folds/unfolds the legs, the sticks drive the wheels in car mode,
and interlocks stop unsafe combinations. The same file runs in SITL/Gazebo and
on the real MicoAir743v2 (`APM/scripts/` on the SD card).

| Feature | Behaviour |
|---|---|
| Switch (RC7, `RC7_OPTION 300`) | low/mid = DRIVE, high = FLY |
| Transformation | all 4 hips move together over 2 s (`TRAVEL_MS`) |
| ARM interlock | ARM refused (`EVO: not in FLY position`) unless legs are fully in FLY and 2 s have passed (`SETTLE_MS`) |
| Armed | switch changes ignored (`EVO: DISARM first!`), legs never move |
| Wheels | tank drive (RC2 pitch = forward/back, RC1 roll = turn), only in full DRIVE and disarmed |
| Boot | waits for the switch in DRIVE before doing anything |
| `AUTO_DRIVE_AFTER_FLIGHT` (off by default) | after landing + disarm the legs fold by themselves; switch to DRIVE and back to FLY to unfold again |

Outputs (`params/evo_borne_hybrid.parm`, same channel map as the Gazebo model):

| SITL output | Joint | Function |
|---|---|---|
| SERVO1-4 | rotors | Motor1-4 (ArduPilot mixer) |
| SERVO5-8 | hips FL, FR, RL, RR | Script1-4 (94-97) |
| SERVO9, 11 | wheels FL, RL (left) | Script5 (98) |
| SERVO10, 12 | wheels FR, RR (right) | Script6 (99) |

On the real FC one output per side is enough (Y-cable to the Scorpion dual ESC);
leave SERVO11/12 unused.

## Install (once)

SITL looks for Lua scripts in `scripts/` next to where it runs. Link the file
there, so edits in `lua/` are picked up without copying (from the repo root):

```bash
mkdir -p ardupilot/ArduCopter/scripts
ln -sf ../../../lua/evoborne.lua ardupilot/ArduCopter/scripts/evoborne.lua
```

## Run in Gazebo

Terminal 1 and 3 as in the main README (Gazebo server, then GUI). Terminal 2:

```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
    --add-param-file=../../params/evo_borne_copter.parm \
    --add-param-file=../../params/evo_borne_hybrid.parm
```

Wait for `EVO: loaded` and `EVO: ready (DRIVE)`.

`--add-param-file` only sets *defaults*: values already saved in SITL's
`eeprom.bin` win. If you ever ran `param load` or `param set` for these
parameters (e.g. `RC7_OPTION 7` from `evo_borne_copter.parm`), start once with
`-w` to wipe the saved values, or run
`param load ../../params/evo_borne_hybrid.parm` in MAVProxy and then `reboot`
(`SCR_ENABLE` only takes effect after a reboot).

## Manual test (MAVProxy, one command at a time)

| Command | Expected |
|---|---|
| `rc 2 1300` | drives forward |
| `rc 1 1700` | turns right |
| `rc 1 1500` then `rc 2 1500` | stops |
| `mode guided` then `arm throttle` | refused: `EVO: not in FLY position` |
| `rc 7 1900` | legs unfold in 2 s, then `ready to ARM` |
| `arm throttle` then `takeoff 5` | climbs to 5 m |
| `rc 7 1100` (in the air) | `EVO: DISARM first! request ignored`, legs stay open |
| `mode land` | lands and disarms, then folds to DRIVE (switch is already at DRIVE) |

## Automated test

With SITL running (Gazebo or the built-in physics), from the repo root:

```bash
python3 tests/test_evoborne_lua.py            # uses the spare port tcp:127.0.0.1:5763
```

It flies the whole scenario above, plus an RC failsafe check, and prints
PASS/FAIL for 23 checks. It waits for the script's messages rather than fixed
delays, so a slow Gazebo (real-time factor < 1) is fine. Close
QGroundControl's virtual joystick first, as it sends its own RC overrides.

Without Gazebo (faster, only checks the logic), use ArduPilot's built-in quad
physics:

```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f quad --console \
    --add-param-file=../../params/evo_borne_hybrid.parm
```

## Before the real vehicle

- Remove the propellers for every bench test.
- Calibrate the hip PWM in `HIPS` (DRIVE/FLY values per servo). Start with
  1400-1600 and widen slowly; mirrored servos may need DRIVE and FLY swapped.
- Check the wheel direction per side and swap the sign in `write_wheels` if a
  side runs backwards.
- On the FC use one output per side (`SERVO9_FUNCTION 98`, `SERVO10_FUNCTION 99`)
  and set `SERVO11_FUNCTION 0`, `SERVO12_FUNCTION 0`, unless those outputs are
  really wired to wheels.
- RC loss stops the wheels only if ArduPilot sees the failsafe. With ELRS over
  CRSF this works by default; never set the receiver to "hold last position".
