#!/usr/bin/env bash
# Start Evo-Borne ArduCopter SITL (copter + hybrid/Lua params) and make sure
# the arducopter process dies with this terminal.
#
#   ./sitl_copter.sh          # normal start
#   ./sitl_copter.sh -w       # wipe saved params first (extra args go to sim_vehicle.py)
#
# Why: sim_vehicle.py starts arducopter in the background. Its own cleanup only
# runs on a clean exit (Ctrl+C / quitting MAVProxy), not when the terminal is
# closed. And once Gazebo is gone, arducopter sits in a loop waiting for
# physics data and ignores a plain `kill`. So on any exit we SIGTERM it, then
# SIGKILL whatever is left.

REPO="$(cd "$(dirname "$0")" && pwd)"
SITL_BIN="$REPO/external/ardupilot/build/sitl/bin/arducopter"

kill_sitl() {
  pkill -TERM -f "$SITL_BIN" 2>/dev/null || return 0
  sleep 1
  pkill -KILL -f "$SITL_BIN" 2>/dev/null
  return 0
}

# leftovers from an earlier run would hold the ports
kill_sitl

trap kill_sitl EXIT
trap 'exit 1' HUP INT TERM

source "$REPO/.venv/bin/activate"
cd "$REPO/external/ardupilot/ArduCopter" || exit 1
"$REPO/external/ardupilot/Tools/autotest/sim_vehicle.py" -v ArduCopter -f gazebo-iris --model JSON --console \
  --add-param-file="$REPO/params/evo_borne_copter.parm" \
  --add-param-file="$REPO/params/evo_borne_hybrid.parm" \
  "$@"
