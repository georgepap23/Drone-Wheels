"""SITL test for the ground autonomy of lua/evoborne.lua.

Drives the real vehicle model, so it needs a simulator that moves the vehicle
with the wheels (Gazebo with the Evo-Borne model). It checks:
  - EVO_ parameters exist
  - GUIDED: drive to a clicked point and stop there
  - HOLD (LOITER mode): wheels stay stopped even with stick input
  - RTL: drive back home
  - AUTO: drive a mission (DO_CHANGE_SPEED, hold time at a waypoint), stop at
    TAKEOFF, then (EVO_AUTO_FLY = 1) unfold, arm, fly, land, fold and drive
    the waypoints after LAND
  - RC failsafe and moving the switch stop the autonomy

At the model speed (~0.12 m/s) it takes about 10 minutes of simulated time.

Usage:  python3 tests/test_ground_autonomy.py [connection]
"""
import math
import sys
import time

from pymavlink import mavutil

CONN = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5763"
LEG = 4.0          # m between waypoints (small: the model is slow)
rc = [1500, 1500, 1000, 1500, 0, 1500, 1500, 1500]
texts, last, results = [], {}, []
text_ms = {}       # first time (vehicle boot ms) each message was seen

m = mavutil.mavlink_connection(CONN, source_system=255)
m.wait_heartbeat(timeout=60)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)


def pump(sec):
    end = time.time() + sec
    while time.time() < end:
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "STATUSTEXT":
            texts.append(msg.text)
            if "SYSTEM_TIME" in last:
                text_ms.setdefault(msg.text, last["SYSTEM_TIME"].time_boot_ms)
            if not msg.text.startswith(("EKF", "GPS", "Field", "Baro", "PreArm")):
                print("   [txt]", msg.text)
        elif t == "NAMED_VALUE_FLOAT" and msg.name.startswith("EVO"):
            last["EVO"] = msg
        last[t] = msg


def seen(sub):
    return any(sub in t for t in texts)


def wait_until(cond, timeout):
    end = time.time() + timeout
    while not cond() and time.time() < end:
        pump(0.2)
    return cond()


def wait_text(sub, timeout):
    return wait_until(lambda: seen(sub), timeout)


def check(name, ok, detail=""):
    ok = bool(ok)
    results.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name, detail)


def armed():
    return bool(m.motors_armed())


def wheels():
    s = last["SERVO_OUTPUT_RAW"]
    return [s.servo9_raw, s.servo10_raw, s.servo11_raw, s.servo12_raw]


def hips():
    s = last["SERVO_OUTPUT_RAW"]
    return [s.servo5_raw, s.servo6_raw, s.servo7_raw, s.servo8_raw]


home = {}


def ne():
    """position (north, east) in m relative to home"""
    g = last["GLOBAL_POSITION_INT"]
    n = (g.lat / 1e7 - home["lat"]) * 111319.5
    e = (g.lon / 1e7 - home["lon"]) * 111319.5 * math.cos(math.radians(home["lat"]))
    return n, e


def latlon(n, e):
    return (home["lat"] + n / 111319.5,
            home["lon"] + e / (111319.5 * math.cos(math.radians(home["lat"]))))


def dist_to(n, e):
    pn, pe = ne()
    return math.hypot(pn - n, pe - e)


def alt():
    return last["GLOBAL_POSITION_INT"].relative_alt / 1000.0


def set_param(name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), value,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    return wait_until(lambda: "PARAM_VALUE" in last and last["PARAM_VALUE"].param_id == name
                      and abs(last["PARAM_VALUE"].param_value - value) < 1e-4, 10)


def upload(items):
    """items: list of (cmd, n, e, alt, param1, param2); item 0 = home"""
    msgs = []
    for seq, (cmd, n, e, a, p1, p2) in enumerate(items):
        lat, lon = latlon(n, e) if (n is not None) else (0, 0)
        frame = 0 if seq == 0 else 3
        msgs.append(m.mav.mission_item_int_encode(
            m.target_system, m.target_component, seq, frame, cmd, 0, 1, p1, p2, 0, 0,
            int(lat * 1e7), int(lon * 1e7), a, 0))
    m.mav.mission_count_send(m.target_system, m.target_component, len(msgs))
    sent = 0
    end = time.time() + 20
    while time.time() < end:
        r = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                         blocking=True, timeout=1)
        if r is None:
            continue
        if r.get_type() == "MISSION_ACK":
            return r.type == 0 and sent == len(msgs)
        m.mav.send(msgs[r.seq])
        sent += 1
    return False


# ---- setup ------------------------------------------------------------------
check("script running", wait_until(lambda: "EVO" in last, 60))
ekf_ok = lambda: "EKF_STATUS_REPORT" in last and last["EKF_STATUS_REPORT"].flags & \
    mavutil.mavlink.EKF_POS_HORIZ_ABS
check("EKF has position", wait_until(ekf_ok, 120))
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
wait_until(lambda: "HOME_POSITION" in last, 20)
home["lat"] = last["HOME_POSITION"].latitude / 1e7
home["lon"] = last["HOME_POSITION"].longitude / 1e7
pump(2)
check("in DRIVE", last["EVO"].value == 0.0)
m.mav.param_request_read_send(m.target_system, m.target_component, b"EVO_SPEED", -1)
check("EVO_ parameters exist", wait_until(lambda: "PARAM_VALUE" in last
                                          and last["PARAM_VALUE"].param_id == "EVO_SPEED", 10))
set_param("EVO_AUTO_FLY", 0)

# ---- GUIDED -----------------------------------------------------------------
texts.clear()
m.set_mode("GUIDED")
check("ground GUIDED", wait_text("ground GUIDED", 10))
lat, lon = latlon(LEG, 0)
m.mav.mission_item_int_send(m.target_system, m.target_component, 0, 3, 16, 2, 1, 0, 0, 0, 0,
                            int(lat * 1e7), int(lon * 1e7), 0)
pump(5)
check("GUIDED: wheels driving forward", all(w > 1550 for w in wheels()), wheels())
check("GUIDED: reached target", wait_text("GUIDED: reached", 150), f"{dist_to(LEG, 0):.1f} m")
wait_until(lambda: wheels() == [1500] * 4, 3)   # telemetry lags the stop
check("GUIDED: stopped within radius", dist_to(LEG, 0) < 2.0 and wheels() == [1500] * 4,
      f"{dist_to(LEG, 0):.1f} m, {wheels()}")

# ---- HOLD -------------------------------------------------------------------
m.set_mode("LOITER")
wait_text("ground HOLD", 10)
rc[1] = 1200; pump(3)
check("HOLD: sticks ignored, wheels stopped", wheels() == [1500] * 4, wheels())
rc[1] = 1500

# ---- RTL --------------------------------------------------------------------
texts.clear()
m.set_mode("RTL")
check("RTL: reached home", wait_text("RTL: reached", 180), f"{dist_to(0, 0):.1f} m")
check("RTL: within radius of home", dist_to(0, 0) < 2.0, f"{dist_to(0, 0):.1f} m")

# ---- AUTO, manual hand-off (EVO_AUTO_FLY = 0) -------------------------------
WP, DO_SPEED, TAKEOFF, LAND = 16, 178, 22, 21
mission = [
    (WP, 0, 0, 0, 0, 0),                    # 0 home
    (WP, LEG, 0, 0, 0, 0),                  # 1 drive north
    (DO_SPEED, None, None, 0, 1, 0.08),     # 2 slower
    (WP, LEG, LEG, 0, 3, 0),                # 3 drive east, wait 3 s
    (TAKEOFF, None, None, 5, 0, 0),         # 4 fly
    (WP, LEG, LEG, 5, 0, 0),                # 5
    (LAND, None, None, 0, 0, 0),            # 6
    (WP, 0, LEG, 0, 0, 0),                  # 7 drive south after landing
]
check("mission uploaded", upload(mission))
texts.clear()
m.set_mode("AUTO")
check("ground AUTO", wait_text("ground AUTO", 10))
check("AUTO: reached item 1", wait_text("reached item 1", 150), f"{dist_to(LEG, 0):.1f} m")
check("AUTO: DO_CHANGE_SPEED applied", wait_text("speed 0.08", 10))

# RC failsafe pauses the mission
rc[2] = 900; pump(3)
check("AUTO: RC failsafe stops the wheels", wheels() == [1500] * 4, wheels())
rc[2] = 1000; pump(3)
check("AUTO: continues after RC is back", any(w != 1500 for w in wheels()), wheels())

check("AUTO: reached item 3", wait_text("reached item 3", 200), f"{dist_to(LEG, LEG):.1f} m")
check("AUTO: stops at TAKEOFF", wait_text("ground leg done: switch FLY", 30))
waited = (text_ms.get("EVO: ground leg done: switch FLY, then ARM", 0)
          - text_ms.get("EVO: AUTO: reached item 3", 0)) / 1000.0
check("AUTO: waited at item 3 (param1 = 3 s)", 2.8 <= waited <= 4.0, f"{waited:.1f} s")
pump(3)
check("AUTO: legs still DRIVE, disarmed", hips() == [1000] * 4 and not armed(), hips())
check("AUTO: current item is TAKEOFF", last["MISSION_CURRENT"].seq == 4,
      last["MISSION_CURRENT"].seq)

# pilot: switch to FLY and arm (AUTO_OPTIONS 3 = arm in AUTO, no throttle needed)
texts.clear()
rc[6] = 1900
check("legs unfold", wait_text("ready to ARM", 20))
end = time.time() + 30
while not armed() and time.time() < end:
    m.arducopter_arm()
    wait_until(armed, 3)
check("ARM in AUTO", armed())
check("AUTO take-off", wait_until(lambda: alt() > 3, 40), f"{alt():.1f} m")
check("lands and disarms", wait_until(lambda: not armed(), 120))
texts.clear()
rc[6] = 1100
check("pilot folds the legs", wait_text("DRIVE position reached", 20))
check("AUTO: continues after LAND", wait_text("continue at item 7", 20))
check("AUTO: drives item 7 after landing", wait_text("reached item 7", 200),
      f"{dist_to(0, LEG):.1f} m")
check("AUTO: mission complete", wait_text("mission complete", 30))

# ---- AUTO, fully automatic (EVO_AUTO_FLY = 1) ------------------------------
texts.clear()
check("EVO_AUTO_FLY = 1", set_param("EVO_AUTO_FLY", 1))
m.set_mode("GUIDED"); pump(1)
m.mav.mission_set_current_send(m.target_system, m.target_component, 3)
m.set_mode("AUTO")
check("AUTO again from item 3", wait_text("reached item 3", 200), f"{dist_to(LEG, LEG):.1f} m")
check("auto take-off announced", wait_text("auto take-off", 30))
check("auto: legs unfold and arm", wait_until(armed, 40))
check("auto: flies", wait_until(lambda: alt() > 3, 40), f"{alt():.1f} m")
check("auto: lands and folds", wait_text("landed: folding", 150))
check("auto: continues after LAND", wait_text("continue at item 7", 40))
check("auto: drives after landing", wait_text("reached item 7", 200),
      f"{dist_to(0, LEG):.1f} m")

# ---- switch override --------------------------------------------------------
texts.clear()
m.mav.mission_set_current_send(m.target_system, m.target_component, 1)
pump(5)
rc[6] = 1900; pump(1); rc[6] = 1100; pump(3)
check("moving the switch: pilot in control", seen("pilot in control") or seen("jump to item"))
m.set_mode("STABILIZE"); pump(2)
check("STABILIZE = manual, wheels stop with centred sticks", wheels() == [1500] * 4, wheels())

print("\n%d/%d passed" % (sum(ok for _, ok in results), len(results)))
sys.exit(0 if all(ok for _, ok in results) else 1)
