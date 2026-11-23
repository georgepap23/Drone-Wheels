-- evoborne.lua : DRIVE/FLY transformation + skid-steer wheels for Evo-Borne (ArduCopter)
--
-- Runs INSIDE ArduPilot, so the same file works in SITL/Gazebo and on the real
-- MicoAir743v2 (copy to APM/scripts/ on the SD card).
--
-- What it does
--   1. RC switch (RCx_OPTION = 300, RC7 by default): low/mid = DRIVE, high = FLY.
--   2. Moves the 4 hip servos SLOWLY (TRAVEL_MS) instead of jumping 1000 -> 2000.
--   3. In DRIVE, drives the wheels like a tank:
--        pitch stick (RC2) = forward/back, roll stick (RC1) = turn.
--   4. Safety interlocks
--        - never transforms while ARMED (switch changes are ignored + warned)
--        - blocks ARM unless the legs are fully in FLY and have settled (SETTLE_MS)
--        - wheels only turn in full DRIVE and DISARMED
--        - at boot it waits for the switch to be in DRIVE before doing anything
--   5. Optional (AUTO_DRIVE_AFTER_FLIGHT): after landing + disarm the legs fold
--      back to DRIVE by themselves; the switch must then go to DRIVE and back to
--      FLY before the next unfold.
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
local WHEEL_MAX    = 1.0     -- 0..1, wheel speed limit (1.0 = full PWM range)
local DEADBAND     = 0.06    -- stick dead band (fraction of full throw)

local TRAVEL_MS = 2000       -- full DRIVE <-> FLY travel time (slow = lower servo current)
local SETTLE_MS = 2000       -- wait in FLY before ARM is allowed
local LOOP_MS   = 20         -- 50 Hz

local AUTO_DRIVE_AFTER_FLIGHT = false

local ERR, WARN, INFO = 3, 4, 6

-- ================= STATE =================
local STEP = LOOP_MS / TRAVEL_MS
local pos, target = 0.0, 0.0       -- 0 = DRIVE, 1 = FLY
local settle_left = 0
local need_drive = true            -- true = wait for switch in DRIVE (boot / after auto-fold)
local need_drive_warned = false
local armed_warned = false
local was_armed = false
local tick = 0
local sw = nil
local auth_id = arming:get_aux_auth_id()

local function say(sev, msg) gcs:send_text(sev, "EVO: " .. msg) end

local function clamp(x, lo, hi)
  if x < lo then return lo elseif x > hi then return hi end
  return x
end

local function lerp(a, b, t) return math.floor(a + (b - a) * t + 0.5) end

local function write_hips()
  for _, h in ipairs(HIPS) do
    SRV_Channels:set_output_pwm(h[2], lerp(h[3], h[4], pos))
  end
end

local function stick(chan, reverse)
  -- RC lost (ArduPilot has detected RC failsafe) -> stop the wheels.
  -- The receiver must not "hold last position" on link loss (with ELRS over
  -- CRSF it stops sending frames, which ArduPilot detects).
  if not rc:has_valid_input() then return 0 end
  local v = rc:get_pwm(chan)
  if not v or v < 900 then return 0 end
  local x = (v - 1500) / 500
  if reverse then x = -x end
  if math.abs(x) < DEADBAND then return 0 end
  return clamp(x, -1, 1)
end

-- Wheels: only in full DRIVE and disarmed, otherwise stop (1500).
local function write_wheels(armed)
  local left, right = 0, 0
  if pos == 0.0 and target == 0.0 and not armed and not need_drive then
    local fwd  = stick(2, true)    -- pitch: stick forward = low PWM = drive forward
    local turn = stick(1, false)   -- roll: right = turn right
    left, right = fwd + turn, fwd - turn
    local m = math.max(1, math.abs(left), math.abs(right))
    left, right = left / m * WHEEL_MAX, right / m * WHEEL_MAX
  end
  SRV_Channels:set_output_pwm(FUNC_WHEEL_L, lerp(1500, 2000, left))
  SRV_Channels:set_output_pwm(FUNC_WHEEL_R, lerp(1500, 2000, right))
end

local function update_arming()
  if not auth_id then return end
  if pos >= 1.0 and target >= 1.0 and settle_left <= 0 then
    arming:set_aux_auth_passed(auth_id)
  else
    arming:set_aux_auth_failed(auth_id, "EVO: not in FLY position")
  end
end

function update()
  if not sw then
    sw = rc:find_channel_for_option(SW_OPTION)
    if not sw then
      say(WARN, "no switch: set RCx_OPTION=300")
      update_arming()
      return update, 1000
    end
  end

  local want = (sw:get_aux_switch_pos() == 2) and 1.0 or 0.0
  local armed = arming:is_armed()

  -- optional: fold back to DRIVE after a flight, once landed and disarmed
  if AUTO_DRIVE_AFTER_FLIGHT and was_armed and not armed and target == 1.0 then
    target = 0.0
    need_drive = true
    need_drive_warned = true       -- the message below already explains it
    say(INFO, "landed -> DRIVE, cycle switch to unfold")   -- max 50 chars per message
  end
  was_armed = armed

  -- wait for the switch in DRIVE (at boot, or after the auto-fold)
  if need_drive then
    if want == 0.0 then
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
    end
  end
  if settle_left > 0 then
    settle_left = settle_left - LOOP_MS
    if settle_left <= 0 then say(INFO, "ready to ARM") end
  end

  write_hips()
  write_wheels(armed)
  update_arming()

  tick = tick + 1
  if tick % 10 == 0 then gcs:send_named_float("EVO", pos) end   -- 0 = DRIVE .. 1 = FLY

  return update, LOOP_MS
end

if not auth_id then say(ERR, "no arming auth slot: arming NOT blocked!") end
-- at boot: legs in DRIVE, wheels stopped, arming blocked
write_hips()
SRV_Channels:set_output_pwm(FUNC_WHEEL_L, 1500)
SRV_Channels:set_output_pwm(FUNC_WHEEL_R, 1500)
update_arming()
say(INFO, "loaded")
return update, 1000
