-- Aim rays: controller rays (headset) and a flat stand-in from the right hand bone along the camera forward.
-- Shows the angle between each ray and where the game camera looks (PORT-06 aim ownership).
local m = require("avrmath")
local RED, BLUE, CYAN = m.rgba(255, 90, 90), m.rgba(90, 150, 255), m.rgba(90, 230, 230)

avr.on("frame", function()
  if not avr.toggle("aim: show rays", true) then return end
  local p = avr.player()
  if not p then return end
  local len = avr.tunable("aim: ray length m", 20, 1, 100, 1)
  local y = 0.20
  for hand = 0, 1 do
    local org, dir = avr.stereo.aim_ray(hand)
    if org then
      local col = hand == 1 and RED or BLUE
      avr.draw.line(org, m.add(org, m.scale(dir, len)), col, 3)
      avr.draw.screen_text(0.02, y, string.format("%s controller ray: %.1f deg from camera forward",
        hand == 1 and "right" or "left", m.angle_deg(dir, p.fwd)), col)
      y = y + 0.03
    end
  end
  -- Flat stand-in: right hand (bone 56) along the camera forward, where a held weapon would point.
  local hand = avr.bone(56)
  if hand then
    avr.draw.line(hand.pos, m.add(hand.pos, m.scale(p.fwd, len)), CYAN, 2)
    avr.draw.point(hand.pos, 5, CYAN)
    avr.draw.text(hand.pos, "R hand 56", CYAN)
  end
end)
