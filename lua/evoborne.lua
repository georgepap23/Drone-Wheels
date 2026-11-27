-- evoborne.lua : DRIVE/FLY transformation + ground driving (manual and autonomous)
--                for Evo-Borne on ArduCopter
--
-- Runs INSIDE ArduPilot, so the same file works in SITL/Gazebo and on the real
-- MicoAir743v2 (copy to APM/scripts/ on the SD card).
--
-- What it does
--   1. RC switch (RCx_OPTION = 300, RC7 by default): low/mid = DRIVE, high = FLY.
--   2. Moves the 4 hip servos SLOWLY (TRAVEL_MS) instead of jumping 1000 -> 2000.
--   3. In DRIVE (legs folded, disarmed) the normal flight modes become ground
--      modes, like ArduRover:
--        AUTO              drive the mission waypoints up to the first TAKEOFF
--        GUIDED            drive to the point clicked on the map ("Go to location")
--        RTL / SMART_RTL   drive back to home
--        LOITER/BRAKE/POSHOLD  HOLD: stop and stay
--        any other mode    MANUAL: pitch stick = forward/back, roll stick = turn
--   4. AUTO missions can mix driving and flying:
--        WP, WP, ..., TAKEOFF, WP, ..., LAND, WP, ...
--      The ground leg ends at TAKEOFF. With EVO_AUTO_FLY = 0 the pilot switches
--      to FLY and arms (in AUTO). With EVO_AUTO_FLY = 1 the script unfolds the
--      legs and arms by itself, and after a landing in AUTO folds the legs and
--      drives the following ground waypoints.
--   5. Safety interlocks
--        - never transforms while ARMED (switch changes are ignored + warned)
--        - blocks ARM unless the legs are fully in FLY and have settled (SETTLE_MS)
--        - wheels only turn in full DRIVE and DISARMED
--        - RC failsafe or no position -> wheels stop
--        - at boot it waits for the switch to be in DRIVE before doing anything
--        - moving the switch always cancels what the script decided by itself
--
-- Tuning parameters (visible in the GCS parameter list after a reboot):
--   EVO_SPD_MAX    ground speed at full wheel command (m/s)      measure it!
--   EVO_SPEED      cruise speed in AUTO/GUIDED/RTL (m/s)         like WP_SPEED
--   EVO_WP_RADIUS  waypoint reached radius (m)                   like WP_RADIUS
--   EVO_TURN_P     steering gain: wheel turn command per rad of heading error
--   EVO_TURN_MAX   max turn command (0..1)
--   EVO_LOOKAHEAD  path-following look-ahead distance (m)
--   EVO_SPD_P      speed controller gain
--   EVO_AUTO_FLY   1 = transform + arm + take off by itself at a TAKEOFF item
--
-- Output map (params/evo_borne_hybrid.parm, matches sim/models/Evo-Borne/model.sdf)
--   SERVO1-4   rotors (ArduPilot mixer, 33-36)
--   SERVO5-8   hips FL, FR, RL, RR   -> Script1-4 (94-97)
--   SERVO9,11  wheels FL, RL         -> Script5 (98)  = left side
--   SERVO10,12 wheels FR, RR         -> Script6 (99)  = right side
--   One function per side drives both wheels on that side, so on the real FC
--   it is fine to use only one output per side (Y-cable to the Scorpion ESC).

-- ================= SETTINGS (calibrate on the bench) =================
local SW_OPTION = 300

-- { name, SERVOx_FUNCTION, PWM in DRIVE, PWM in FLY }
-- Gazebo model: all four use 1000 = folded, 2000 = unfolded 90 deg.
-- REAL VEHICLE: mirrored servos may need DRIVE/FLY swapped. Start with a small
-- range (e.g. 1400-1600) and widen until each leg just reaches its end stop.
local HIPS = {
  { "FL", 94, 1000, 2000 },
  { "FR", 95, 1000, 2000 },
  { "RL", 96, 1000, 2000 },
  { "RR", 97, 1000, 2000 },
}

local FUNC_WHEEL_L = 98
local FUNC_WHEEL_R = 99
local DEADBAND     = 0.06    -- stick dead band (fraction of full throw)

local TRAVEL_MS     = 2000   -- full DRIVE <-> FLY travel time (slow = lower servo current)
local SETTLE_MS     = 2000   -- wait in FLY before ARM is allowed
local AUTO_ARM_MS   = 3000   -- countdown before the script arms by itself (EVO_AUTO_FLY)
local AUTO_ARM_RETRY_MS = 2000   -- retry period if ArduPilot refuses to arm
local AUTO_ARM_TRY_MS   = 60000  -- give up after this long
local STATUS_MS     = 10000  -- progress message period while navigating
local LOOP_MS       = 20     -- 50 Hz

-- ================= PARAMETERS =================
local PARAM_KEY = 47
assert(param:add_table(PARAM_KEY, "EVO_", 8), "EVO: could not add param table")
local function add(i, name, default)
  assert(param:add_param(PARAM_KEY, i, name, default), "EVO: could not add EVO_" .. name)
  return Parameter("EVO_" .. name)
end
local P_SPD_MAX   = add(1, "SPD_MAX", 0.14)
local P_SPEED     = add(2, "SPEED", 0.12)
local P_WP_RADIUS = add(3, "WP_RADIUS", 1.5)
local P_TURN_P    = add(4, "TURN_P", 1.2)
local P_TURN_MAX  = add(5, "TURN_MAX", 0.8)
local P_LOOKAHEAD = add(6, "LOOKAHEAD", 2.0)
local P_SPD_P     = add(7, "SPD_P", 0.5)
local P_AUTO_FLY  = add(8, "AUTO_FLY", 0)

-- ================= CONSTANTS =================
local ERR, WARN, INFO = 3, 4, 6
local MODE_AUTO, MODE_GUIDED, MODE_LOITER, MODE_RTL = 3, 4, 5, 6
local MODE_POSHOLD, MODE_BRAKE, MODE_SMART_RTL = 16, 17, 21
local CMD_WAYPOINT, CMD_LOITER_UNLIM, CMD_LOITER_TURNS, CMD_LOITER_TIME = 16, 17, 18, 19
local CMD_RTL, CMD_LAND, CMD_TAKEOFF, CMD_SPLINE_WP = 20, 21, 22, 82
local CMD_VTOL_TAKEOFF, CMD_VTOL_LAND, CMD_NAV_LAST = 84, 85, 95
local CMD_DO_CHANGE_SPEED = 178
local STEP = LOOP_MS / TRAVEL_MS

-- ================= STATE =================
-- transformation
local pos, target = 0.0, 0.0       -- 0 = DRIVE, 1 = FLY
local settle_left = 0
local need_drive = true            -- true = wait for switch in DRIVE (at boot)
local need_drive_warned = false
local armed_warned = false
local was_armed = false
local sw = nil
local sw_last = nil                -- last switch position, to detect moves
local auto_target = nil            -- leg position chosen by the script (nil = follow switch)
local auth_id = arming:get_aux_auth_id()

-- ground navigation
local nav = {
  mode = nil,            -- ground mode name last used
  idx = nil,             -- AUTO: mission index being driven
  idx_set = nil,         -- AUTO: index we last pushed with set_current_cmd
  origin = nil,          -- start of the current path segment
  hold_until = nil,      -- AUTO: wait at a waypoint until this time (ms)
  done = false,          -- AUTO/GUIDED/RTL: target reached / mission finished
  speed = nil,           -- AUTO: speed set by DO_CHANGE_SPEED
  takeoff_idx = nil,     -- AUTO: ground leg finished at this TAKEOFF item
  guided_tgt = nil,      -- GUIDED: last target, to detect a new one
  next_status = 0,
  arm_at = nil,          -- EVO_AUTO_FLY: time of the next arming attempt
  arm_until = nil,       -- EVO_AUTO_FLY: give up arming after this time
  flight_idx = nil,      -- AUTO: mission item while flying (Copter resets the
                         --       mission to item 1 when it disarms)
  resume_idx = nil,      -- AUTO: continue the ground leg here after a landing
}
local tick = 0

local function say(sev, msg) gcs:send_text(sev, "EVO: " .. msg) end

local function clamp(x, lo, hi)
  if x < lo then return lo elseif x > hi then return hi end
  return x
end

local function wrap_pi(a)
  while a > math.pi do a = a - 2 * math.pi end
  while a < -math.pi do a = a + 2 * math.pi end
  return a
end

local function lerp(a, b, t) return math.floor(a + (b - a) * t + 0.5) end

local function now_ms() return millis():tofloat() end

local function get_yaw()
  if ahrs.get_yaw_rad then return ahrs:get_yaw_rad() end   -- newer ArduPilot
  return ahrs:get_yaw()
end

local function write_hips()
  for _, h in ipairs(HIPS) do
    SRV_Channels:set_output_pwm(h[2], lerp(h[3], h[4], pos))
  end
end

local function set_wheels(left, right)
  local m = math.max(1, math.abs(left), math.abs(right))
  SRV_Channels:set_output_pwm(FUNC_WHEEL_L, lerp(1500, 2000, left / m))
  SRV_Channels:set_output_pwm(FUNC_WHEEL_R, lerp(1500, 2000, right / m))
end

local function stick(chan, reverse)
  local v = rc:get_pwm(chan)
  if not v or v < 900 then return 0 end
  local x = (v - 1500) / 500
  if reverse then x = -x end
  if math.abs(x) < DEADBAND then return 0 end
  return clamp(x, -1, 1)
end

local function update_arming()
  if not auth_id then return end
  if pos >= 1.0 and target >= 1.0 and settle_left <= 0 then
    arming:set_aux_auth_passed(auth_id)
  else
    arming:set_aux_auth_failed(auth_id, "EVO: not in FLY position")
  end
end

-- ================= GROUND NAVIGATION =================
local function item_location(it)
  local l = Location()
  l:lat(it:x())
  l:lng(it:y())
  return l
end

-- Drive towards `tgt`, following the line origin -> tgt (pure pursuit).
-- Returns true when within EVO_WP_RADIUS. `stop_at` = slow down on approach.
local function drive_to(loc, tgt, origin, speed, stop_at)
  local dist = loc:get_distance(tgt)
  if dist <= P_WP_RADIUS:get() then return true, dist end

  local bearing = nil
  if origin then
    local seg = origin:get_distance_NE(tgt)                     -- N, E of the segment
    local seg_len = seg:length()
    if seg_len > 1.0 then
      local p = origin:get_distance_NE(loc)
      local t = clamp((p:x() * seg:x() + p:y() * seg:y()) / (seg_len * seg_len), 0, 1)
      local ahead = t + P_LOOKAHEAD:get() / seg_len
      if ahead < 1 then
        local aim_n = seg:x() * ahead - p:x()
        local aim_e = seg:y() * ahead - p:y()
        bearing = math.atan(aim_e, aim_n)
      end
    end
  end
  if not bearing then
    local ne = loc:get_distance_NE(tgt)
    bearing = math.atan(ne:y(), ne:x())
  end

  local yaw = get_yaw()
  local err = wrap_pi(bearing - yaw)
  local turn_max = P_TURN_MAX:get()
  local turn = clamp(P_TURN_P:get() * err, -turn_max, turn_max)

  -- speed: cruise, slower when pointing the wrong way or approaching a stop
  local v = speed
  if math.abs(err) > math.rad(45) then v = v * 0.3 end
  if stop_at then v = math.min(v, math.max(0.3 * speed, 0.5 * dist)) end

  local spd_max = math.max(P_SPD_MAX:get(), 0.01)
  local gs = ahrs:groundspeed_vector()
  local fwd = gs:x() * math.cos(yaw) + gs:y() * math.sin(yaw)
  local thr = clamp(v / spd_max + P_SPD_P:get() * (v - fwd) / spd_max, 0, 1)

  set_wheels(thr + turn, thr - turn)
  return false, dist
end

local function status(text)
  local t = now_ms()
  if t >= nav.next_status then
    say(INFO, text)
    nav.next_status = t + STATUS_MS
  end
end

local function reset_nav(mode_name)
  nav.mode = mode_name
  nav.idx, nav.idx_set, nav.origin, nav.hold_until = nil, nil, nil, nil
  nav.done, nav.takeoff_idx, nav.guided_tgt, nav.arm_at = false, nil, nil, nil
  nav.next_status = 0
end

-- AUTO: walk the mission from the current item. Returns nothing; writes wheels.
local function run_auto(loc)
  local n = mission:num_commands()
  if n <= 1 then
    if not nav.done then say(WARN, "AUTO: no mission") end
    nav.done = true
    set_wheels(0, 0)
    return
  end
  -- after a landing, continue after the LAND item
  if nav.idx == nil and nav.resume_idx then
    local r = nav.resume_idx
    nav.resume_idx = nil
    if r < n then
      mission:set_current_cmd(r)
      say(INFO, "AUTO: continue at item " .. r)
    else
      say(INFO, "AUTO: mission complete")
      nav.done = true
      nav.idx = r
      set_wheels(0, 0)
      return
    end
  end
  -- follow "set current waypoint" from the GCS
  local cur = mission:get_current_nav_index()
  if nav.idx == nil or (cur ~= nav.idx_set and cur ~= nav.idx and cur > 0) then
    if nav.idx ~= nil then say(INFO, "AUTO: jump to item " .. cur) end
    nav.idx = math.max(cur, 1)
    nav.idx_set = cur
    nav.origin = loc
    nav.hold_until, nav.done, nav.takeoff_idx = nil, false, nil
  end
  if nav.done or nav.takeoff_idx then
    set_wheels(0, 0)
    return
  end

  -- run non-moving items (DO commands, skipped NAV commands) in one go
  for _ = 1, 20 do
    if nav.idx >= n then
      say(INFO, "AUTO: mission complete")
      nav.done = true
      set_wheels(0, 0)
      return
    end
    local it = mission:get_item(nav.idx)
    local cmd = it:command()
    if cmd == CMD_TAKEOFF or cmd == CMD_VTOL_TAKEOFF then
      nav.takeoff_idx = nav.idx
      if mission:get_current_nav_index() ~= nav.idx then mission:set_current_cmd(nav.idx) end
      nav.idx_set = nav.idx
      if P_AUTO_FLY:get() > 0 then
        say(INFO, "ground leg done: auto take-off")
        auto_target = 1.0
      else
        say(INFO, "ground leg done: switch FLY, then ARM")
      end
      set_wheels(0, 0)
      return
    elseif cmd == CMD_DO_CHANGE_SPEED then
      if it:param2() > 0 then
        nav.speed = it:param2()
        say(INFO, string.format("AUTO: speed %.2f m/s", nav.speed))
      end
      nav.idx = nav.idx + 1
    elseif cmd == CMD_WAYPOINT or cmd == CMD_LOITER_UNLIM or cmd == CMD_LOITER_TURNS
        or cmd == CMD_LOITER_TIME or cmd == CMD_SPLINE_WP or cmd == CMD_RTL then
      break                                          -- a place to drive to
    else
      if cmd <= CMD_NAV_LAST then
        say(WARN, "AUTO: skip item " .. nav.idx .. " (cmd " .. cmd .. ")")
      end
      nav.idx = nav.idx + 1
    end
  end

  local it = mission:get_item(nav.idx)
  local cmd = it:command()
  local tgt
  if cmd == CMD_RTL then
    tgt = ahrs:get_home()
  else
    tgt = item_location(it)
    if it:x() == 0 and it:y() == 0 then tgt = nil end       -- "here": nothing to do
  end
  if cmd ~= CMD_RTL and nav.idx_set ~= nav.idx then
    mission:set_current_cmd(nav.idx)                         -- GCS shows the active WP
    nav.idx_set = nav.idx
  end

  -- holding at a reached waypoint?
  if nav.hold_until then
    set_wheels(0, 0)
    if nav.hold_until > 0 and now_ms() >= nav.hold_until then
      nav.hold_until = nil
      nav.origin = tgt or loc
      nav.idx = nav.idx + 1
    end
    return
  end

  local reached, dist = true, 0
  if tgt then
    local nxt = (nav.idx + 1 < n) and mission:get_item(nav.idx + 1) or nil
    local stop_at = cmd ~= CMD_WAYPOINT or it:param1() > 0 or nxt == nil
        or nxt:command() == CMD_TAKEOFF or nxt:command() == CMD_VTOL_TAKEOFF
    reached, dist = drive_to(loc, tgt, nav.origin, nav.speed or P_SPEED:get(), stop_at)
  end
  if not reached then
    status(string.format("AUTO: item %d, %.1f m", nav.idx, dist))
    return
  end

  set_wheels(0, 0)
  say(INFO, "AUTO: reached item " .. nav.idx)
  nav.next_status = 0
  if cmd == CMD_RTL then
    say(INFO, "AUTO: home, mission complete")
    nav.done = true
  elseif cmd == CMD_LOITER_UNLIM then
    nav.hold_until = -1                                       -- forever
  elseif (cmd == CMD_WAYPOINT or cmd == CMD_LOITER_TIME) and it:param1() > 0 then
    nav.hold_until = now_ms() + it:param1() * 1000
  else
    nav.origin = tgt or loc
    nav.idx = nav.idx + 1
  end
end

local function run_goto(loc, tgt, label)
  if not tgt then
    set_wheels(0, 0)
    return
  end
  if nav.guided_tgt == nil or nav.guided_tgt:get_distance(tgt) > 0.5 then
    nav.guided_tgt = tgt
    nav.origin = loc
    nav.done = false
    nav.next_status = 0
  end
  if nav.done then
    set_wheels(0, 0)
    return
  end
  local reached, dist = drive_to(loc, tgt, nav.origin, P_SPEED:get(), true)
  if reached then
    set_wheels(0, 0)
    nav.done = true
    say(INFO, label .. ": reached")
  else
    status(string.format("%s: %.1f m", label, dist))
  end
end

-- One ground-driving step. Only called in full DRIVE, disarmed, switch OK.
local function drive_step()
  local mode = vehicle:get_mode()
  local name
  if mode == MODE_AUTO then name = "AUTO"
  elseif mode == MODE_GUIDED then name = "GUIDED"
  elseif mode == MODE_RTL or mode == MODE_SMART_RTL then name = "RTL"
  elseif mode == MODE_LOITER or mode == MODE_BRAKE or mode == MODE_POSHOLD then name = "HOLD"
  else name = "MANUAL" end
  if name ~= nav.mode then
    reset_nav(name)
    if name ~= "MANUAL" then say(INFO, "ground " .. name) end
  end

  -- RC lost (ArduPilot has detected RC failsafe) -> stop the wheels.
  -- The receiver must not "hold last position" on link loss (with ELRS over
  -- CRSF it stops sending frames, which ArduPilot detects).
  if not rc:has_valid_input() then
    set_wheels(0, 0)
    return
  end

  if name == "MANUAL" then
    local fwd  = stick(2, true)    -- pitch: stick forward = low PWM = drive forward
    local turn = stick(1, false)   -- roll: right = turn right
    set_wheels(fwd + turn, fwd - turn)
    return
  end
  if name == "HOLD" then
    set_wheels(0, 0)
    return
  end

  local loc = ahrs:get_location()
  if not loc or not ahrs:healthy() then
    set_wheels(0, 0)
    status(name .. ": no position, stopped")
    return
  end

  if name == "AUTO" then
    run_auto(loc)
  elseif name == "GUIDED" then
    run_goto(loc, vehicle:get_target_location(), "GUIDED")
  else
    run_goto(loc, ahrs:get_home(), "RTL")
  end
end

-- ================= MAIN LOOP =================
function update()
  if not sw then
    sw = rc:find_channel_for_option(SW_OPTION)
    if not sw then
      say(WARN, "no switch: set RCx_OPTION=300")
      update_arming()
      return update, 1000
    end
  end

  local sw_pos = sw:get_aux_switch_pos()
  local sw_want = (sw_pos == 2) and 1.0 or 0.0
  if sw_last ~= nil and sw_pos ~= sw_last and auto_target ~= nil then
    auto_target = nil                                 -- pilot took over
    nav.arm_at = nil
    say(INFO, "switch moved: pilot in control")
  end
  sw_last = sw_pos
  local want = auto_target or sw_want
  local armed = arming:is_armed()

  -- remember where the mission is while flying in AUTO
  local in_auto = vehicle:get_mode() == MODE_AUTO
  if armed and in_auto then
    nav.flight_idx = mission:get_current_nav_index()
  elseif armed then
    nav.flight_idx = nil
  end
  -- after a landing in AUTO: continue the ground leg after the LAND item
  if was_armed and not armed then
    nav.resume_idx = nil
    if in_auto and nav.flight_idx then
      local it = mission:get_item(nav.flight_idx)
      if it and (it:command() == CMD_LAND or it:command() == CMD_VTOL_LAND) then
        nav.resume_idx = nav.flight_idx + 1
      end
    end
    nav.flight_idx = nil
    -- EVO_AUTO_FLY: fold by itself
    if nav.resume_idx and P_AUTO_FLY:get() > 0 and target == 1.0 then
      auto_target = 0.0
      want = 0.0
      say(INFO, "landed: folding to continue mission")
    end
  end
  was_armed = armed

  -- wait for the switch in DRIVE at boot
  if need_drive then
    if sw_want == 0.0 then
      need_drive = false
      need_drive_warned = false
      say(INFO, "ready (DRIVE)")
    elseif not need_drive_warned then
      say(WARN, "switch to DRIVE first")
      need_drive_warned = true
    end
  end

  if not need_drive and want ~= target then
    if armed then
      if not armed_warned then
        say(ERR, "DISARM first! request ignored")
        armed_warned = true
      end
    else
      target = want
      armed_warned = false
      say(INFO, (target == 1.0) and "-> FLY" or "-> DRIVE")
    end
  end
  if want == target then armed_warned = false end

  local was_moving = (pos ~= target)
  if not armed then
    if pos < target then
      pos = math.min(target, pos + STEP)
    elseif pos > target then
      pos = math.max(target, pos - STEP)
    end
  end
  if was_moving and pos == target then
    if target == 1.0 then
      settle_left = SETTLE_MS
      say(INFO, "FLY position reached, settling")
    else
      settle_left = 0
      say(INFO, "DRIVE position reached, wheels enabled")
      reset_nav(nil)
    end
  end
  if settle_left > 0 then
    settle_left = settle_left - LOOP_MS
    if settle_left <= 0 then
      say(INFO, "ready to ARM")
      if auto_target == 1.0 and not armed then
        nav.arm_at = now_ms() + AUTO_ARM_MS
        nav.arm_until = nav.arm_at + AUTO_ARM_TRY_MS
        say(INFO, string.format("auto ARM in %d s", AUTO_ARM_MS // 1000))
      end
    end
  end

  -- EVO_AUTO_FLY: arm by itself (only in AUTO, legs in FLY, RC present).
  -- ArduPilot's own pre-arm checks may refuse for a few seconds after the legs
  -- move (e.g. "Accels inconsistent" while the body still wobbles), so retry
  -- like a pilot would, for up to AUTO_ARM_TRY_MS.
  if nav.arm_at and not armed then
    if vehicle:get_mode() ~= MODE_AUTO or auto_target ~= 1.0 then
      nav.arm_at = nil
    elseif now_ms() >= nav.arm_at then
      if not rc:has_valid_input() then
        nav.arm_at = nil
        say(WARN, "no RC: auto take-off cancelled")
      elseif arming:arm() then
        nav.arm_at = nil
      elseif now_ms() >= nav.arm_until then
        nav.arm_at = nil
        say(WARN, string.format("auto ARM refused for %d s, see PreArm", AUTO_ARM_TRY_MS // 1000))
      else
        if nav.arm_until - now_ms() > AUTO_ARM_TRY_MS - AUTO_ARM_RETRY_MS then
          say(INFO, "auto ARM: waiting for PreArm checks")
        end
        nav.arm_at = now_ms() + AUTO_ARM_RETRY_MS
      end
    end
  elseif armed then
    nav.arm_at = nil
  end

  write_hips()
  if pos == 0.0 and target == 0.0 and not armed and not need_drive then
    drive_step()
  else
    if nav.mode ~= nil and nav.mode ~= "MANUAL" then reset_nav(nil) end
    set_wheels(0, 0)
  end
  update_arming()

  tick = tick + 1
  if tick % 10 == 0 then gcs:send_named_float("EVO", pos) end   -- 0 = DRIVE .. 1 = FLY

  return update, LOOP_MS
end

if not auth_id then say(ERR, "no arming auth slot: arming NOT blocked!") end
-- at boot: legs in DRIVE, wheels stopped, arming blocked
write_hips()
set_wheels(0, 0)
update_arming()
say(INFO, "loaded")
return update, 1000
