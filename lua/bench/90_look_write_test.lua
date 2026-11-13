-- PORT-06 write test (needs the menu's "Script writes" ON). Writes the look angles in camera_pre, before the game's
-- camera manager reads them.
--   "look: hold" (flat test)       freezes the view at the angles seen when it was switched on, plus the offsets.
--                                  If the view stays put while the mouse moves, the game takes the written angles.
--   "look: follow right controller" (headset) aims the game camera along the right controller's ray.
-- Turn both off (or reload) to give the camera back; the game rewrites its own angles every frame.
local m = require("avrmath")
local held = nil

avr.on("camera_pre", function()
  local hold = avr.value("look: hold")
  local follow = avr.value("look: follow right controller")
  if not hold then held = nil end
  if not avr.writes() then return end
  if follow then
    local _, dir = avr.stereo.aim_ray(1)
    if dir then
      local yaw, pitch = m.yaw_pitch(dir)
      avr.set_look(yaw, pitch)
    end
    return
  end
  if hold then
    local p = avr.player()
    if not p or not p.look_ok then return end
    if not held then held = {yaw = p.yaw, pitch = p.pitch} end
    local dy = math.rad(avr.value("look: yaw offset deg") or 0)
    local dp = math.rad(avr.value("look: pitch offset deg") or 0)
    avr.set_look(held.yaw + dy, held.pitch + dp)
  end
end)

-- Rows are registered from the Present thread (avr.toggle/tunable create menu rows there).
avr.on("frame", function()
  avr.toggle("look: hold", false)
  avr.toggle("look: follow right controller", false)
  avr.tunable("look: yaw offset deg", 0, -90, 90, 5)
  avr.tunable("look: pitch offset deg", 0, -45, 45, 5)
end)
