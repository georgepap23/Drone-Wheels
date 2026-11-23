"""End-to-end SITL test for lua/evoborne.lua.

Checks: boot in DRIVE, tank drive, ARM blocked outside FLY, slow unfold,
settle time, takeoff, switch ignored while armed, wheels locked while armed,
fold after landing. Run it against a SITL that has the Lua loaded (see
lua/README.md). Default port is the spare SITL port 5763 (MAVProxy keeps 5760).

The test waits for the script's own messages instead of fixed delays, so it
also works when Gazebo runs slower than real time.

Usage:  python3 tests/test_evoborne_lua.py [connection]
"""
import sys
import time

from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5763"
# RC override: roll, pitch, throttle, yaw, ch5 (0 = not overridden), ch6, ch7 switch, ch8
rc = [1500, 1500, 1000, 1500, 0, 1500, 1500, 1500]
texts, last, results = [], {}, []

m = mavutil.mavlink_connection(CONN, source_system=255)
m.wait_heartbeat(timeout=60)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)


def pump(sec):
    """Keep the RC override alive and collect messages for `sec` seconds."""
    end = time.time() + sec
    while time.time() < end:
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            texts.append(msg.text)
            print("   [txt]", msg.text)
        last[msg.get_type()] = msg
        if msg.get_type() == "NAMED_VALUE_FLOAT" and msg.name.startswith("EVO"):
            last["EVO"] = msg


def seen(sub):
    return any(sub in t for t in texts)


def wait_text(sub, timeout):
    end = time.time() + timeout
    while not seen(sub) and time.time() < end:
        pump(0.2)
    return seen(sub)


def wait_until(cond, timeout):
    end = time.time() + timeout
    while not cond() and time.time() < end:
        pump(0.2)
    return cond()


def servos():
    s = last["SERVO_OUTPUT_RAW"]
    return [getattr(s, f"servo{i}_raw") for i in range(1, 13)]


def hips():
    return servos()[4:8]


def wheels():
    return servos()[8:12]


def armed():
    return bool(m.motors_armed())


def check(name, ok, detail=""):
    ok = bool(ok)
    results.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name, detail)


# ---- boot -------------------------------------------------------------------
# The script sends NAMED_VALUE_FLOAT "EVO" (0 = DRIVE .. 1 = FLY) at 5 Hz, so
# this works whether the test starts with SITL or long after "EVO: loaded".
def evo_value():
    v = last.get("EVO")
    return v.value if v is not None else None


check("script running", wait_until(lambda: evo_value() is not None, 60))
check("in DRIVE", evo_value() == 0.0, f"EVO = {evo_value()}")
pump(1)
check("hips folded, wheels stopped", hips() == [1000] * 4 and wheels() == [1500] * 4, servos())

# ---- DRIVE: tank steering ---------------------------------------------------
rc[1] = 1300; pump(1.5)                       # pitch stick forward
check("drive forward: all wheels 1700", wheels() == [1700] * 4, wheels())
rc[0] = 1700; pump(1.5)                       # + roll right
w = wheels()
check("turn right: left side faster", w[0] == w[2] > w[1] == w[3], w)
rc[0] = 1500; rc[2] = 900; pump(2)           # throttle below FS_THR_VALUE = RC failsafe
check("RC failsafe stops the wheels (pitch still forward)", wheels() == [1500] * 4, wheels())
rc[2] = 1000; pump(2)
check("wheels resume after RC is back", wheels() == [1700] * 4, wheels())
rc[1] = 1500; pump(1)
check("sticks centred: wheels stop", wheels() == [1500] * 4, wheels())

# ---- ARM interlock in DRIVE -------------------------------------------------
m.set_mode("GUIDED")
# wait until the EKF has an absolute position, so only our check can block ARM
ekf_ok = lambda: "EKF_STATUS_REPORT" in last and last["EKF_STATUS_REPORT"].flags & \
    mavutil.mavlink.EKF_POS_HORIZ_ABS
wait_until(ekf_ok, 120)
pump(3)
texts.clear()
m.arducopter_arm(); pump(3)
check("ARM refused in DRIVE", seen("not in FLY position") and not armed(), texts[-3:])

# ---- DRIVE -> FLY -----------------------------------------------------------
texts.clear()
rc[6] = 1900; pump(0.5)
check("hips move slowly (not jumped to 2000)", all(1000 < x < 2000 for x in hips()), hips())
rc[1] = 1300                                  # try to drive while unfolding
check("FLY position reached", wait_text("FLY position reached", 15))
pump(0.3)
check("hips at FLY (2000)", hips() == [2000] * 4, hips())
check("wheels locked out in FLY", wheels() == [1500] * 4, wheels())
rc[1] = 1500
m.arducopter_arm(); pump(0.5)
check("ARM refused while settling", not armed() and not seen("ready to ARM"))
check("ready to ARM message", wait_text("ready to ARM", 15))

# ---- flight -----------------------------------------------------------------
# ArduPilot's own pre-arm checks can still refuse for a while: in Gazebo the
# body wobbles when the legs unfold, and "Accels inconsistent" clears only
# after ~10 s of steady IMUs. Retry for up to 30 s.
texts.clear()
end = time.time() + 30
while not armed() and time.time() < end:
    m.arducopter_arm()
    wait_until(armed, 3)
check("ARM allowed in FLY", armed(), texts[-3:])
if not armed():
    reasons = sorted({t for t in texts if t.startswith(("Arm:", "PreArm:"))})
    print("\nStopped: ArduPilot refused to arm for a reason outside the script:")
    for r in reasons:
        print("   ", r)
    print("\n%d/%d passed (flight part not run)" % (sum(ok for _, ok in results), len(results)))
    sys.exit(1)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)
alt = lambda: last["GLOBAL_POSITION_INT"].relative_alt / 1000 if "GLOBAL_POSITION_INT" in last else 0
check("took off", wait_until(lambda: alt() > 3, 40), f"{alt():.1f} m")

texts.clear()
rc[6] = 1100
check("switch to DRIVE in the air ignored", wait_text("DISARM first", 5), texts[-2:])
pump(2)
check("legs stay open in the air", hips() == [2000] * 4, hips())
rc[1] = 1300; pump(1)
check("wheels stay off while armed", wheels() == [1500] * 4, wheels())
rc[1] = 1500

# ---- landing ----------------------------------------------------------------
m.set_mode("LAND")
check("landed and disarmed", wait_until(lambda: not armed(), 120))
check("legs fold after disarm (switch at DRIVE)", wait_text("DRIVE position reached", 15))
pump(0.3)
check("hips back at DRIVE (1000)", hips() == [1000] * 4, hips())

print("\n%d/%d passed" % (sum(ok for _, ok in results), len(results)))
sys.exit(0 if all(ok for _, ok in results) else 1)
