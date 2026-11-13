-- First-person arm chains (research/WEAPONS_AND_HANDS.md "First-person bone map"), drawn over the game image.
local m = require("avrmath")
local LEFT, RIGHT, PROP = m.rgba(110, 170, 255), m.rgba(255, 140, 110), m.rgba(240, 240, 120)
local chains = {
  {16, 17, 18, 19, 20, col = LEFT},   -- left clavicle, upper arm, elbow, hand, hand end
  {53, 54, 55, 56, 57, col = RIGHT},  -- right
}

avr.on("frame", function()
  if not avr.toggle("bones: show arms", true) then return end
  local b = avr.bones()
  if not b[0] then return end
  for _, chain in ipairs(chains) do
    for i = 1, #chain - 1 do
      local a, c = b[chain[i]], b[chain[i + 1]]
      if a and c then avr.draw.line(a, c, chain.col, 3) end
    end
  end
  if b[89] then
    avr.draw.point(b[89], 6, PROP)
    avr.draw.text(b[89], "89 prop", PROP)
  end
  if avr.toggle("bones: all indices", false) then
    for i = 1, avr.bone_count() - 1 do
      if b[i] then avr.draw.text(b[i], tostring(i), m.rgba(255, 255, 255, 170)) end
    end
  end
end)
