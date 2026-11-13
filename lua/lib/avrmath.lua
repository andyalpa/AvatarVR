-- Small vector helpers for AvatarVR scripts (docs/SCRIPTING.md). Vectors are tables {x=, y=, z=}.
-- World frame: large-world metres, +y up. Camera/look convention: yaw = atan2(fwd.x, fwd.z), pitch + up.
local M = {}

function M.v(x, y, z) return {x = x, y = y, z = z} end
function M.add(a, b) return {x = a.x + b.x, y = a.y + b.y, z = a.z + b.z} end
function M.sub(a, b) return {x = a.x - b.x, y = a.y - b.y, z = a.z - b.z} end
function M.scale(a, s) return {x = a.x * s, y = a.y * s, z = a.z * s} end
function M.dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
function M.cross(a, b)
  return {x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x}
end
function M.len(a) return math.sqrt(M.dot(a, a)) end
function M.dist(a, b) return M.len(M.sub(a, b)) end
function M.norm(a)
  local l = M.len(a)
  if l < 1e-9 then return {x = 0, y = 0, z = 0} end
  return M.scale(a, 1 / l)
end
function M.lerp(a, b, t) return M.add(a, M.scale(M.sub(b, a), t)) end
function M.fmt(a) return string.format("(%.2f %.2f %.2f)", a.x, a.y, a.z) end

-- Direction <-> look angles (radians), in the game's convention.
function M.yaw_pitch(dir)
  local d = M.norm(dir)
  return math.atan(d.x, d.z), math.asin(math.max(-1, math.min(1, d.y)))
end
function M.dir(yaw, pitch)
  local cp = math.cos(pitch)
  return {x = math.sin(yaw) * cp, y = math.sin(pitch), z = math.cos(yaw) * cp}
end

-- Angle between two directions, degrees.
function M.angle_deg(a, b)
  local c = M.dot(M.norm(a), M.norm(b))
  return math.deg(math.acos(math.max(-1, math.min(1, c))))
end

-- Rotate v by quaternion q {x, y, z, w} (OpenXR poses from avr.xr.views()).
function M.qrot(q, v)
  local u = {x = q.x, y = q.y, z = q.z}
  local t = M.scale(M.cross(u, v), 2)
  return M.add(M.add(v, M.scale(t, q.w)), M.cross(u, t))
end

-- 0xRRGGBBAA colour from 0..255 components.
function M.rgba(r, g, b, a)
  return ((r & 0xFF) << 24) | ((g & 0xFF) << 16) | ((b & 0xFF) << 8) | ((a or 255) & 0xFF)
end

return M
