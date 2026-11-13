-- Player / camera / look readout and world markers: checks that reading and projection line up (flat or headset).
local m = require("avrmath")
local GREEN, YELLOW, WHITE = m.rgba(90, 230, 140), m.rgba(255, 220, 80), m.rgba(235, 240, 255)

avr.on("frame", function()
  if not avr.toggle("info: show", true) then return end
  local p = avr.player()
  if not p then
    avr.draw.screen_text(0.02, 0.05, "AvatarVR scripts: no player (menu / loading)", WHITE)
    return
  end
  local lines = {
    string.format("body %s   cam %s", m.fmt(p.body), m.fmt(p.pos)),
    string.format("look yaw %.1f  pitch %.1f deg   cam above body %.2f m", math.deg(p.yaw), math.deg(p.pitch),
      p.pos.y - p.body.y),
    string.format("stereo %s   bones %d   writes %s", tostring(avr.stereo.running()), avr.bone_count(),
      avr.writes() and "ON" or "off"),
  }
  for i, s in ipairs(lines) do
    avr.draw.screen_text(0.02, 0.04 + 0.03 * i, s, WHITE)
  end
  -- A marker 3 m ahead at eye height and one at the feet 2 m ahead: both must sit where they should on screen.
  local ahead = m.add(p.pos, m.scale(p.fwd, 3))
  avr.draw.point(ahead, 6, YELLOW)
  avr.draw.text(ahead, "3 m ahead", YELLOW)
  local flat = m.norm({x = p.fwd.x, y = 0, z = p.fwd.z})
  local feet = m.add(p.body, m.scale(flat, 2))
  avr.draw.line(feet, m.add(feet, {x = 0, y = 2.63, z = 0}), GREEN, 2)
  avr.draw.text(feet, "feet + 2 m", GREEN)
end)
