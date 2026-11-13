-- Pure gameplay math. Engine collision queries and pose scheduling belong to the native bridge.
local m = require('avrmath')
local G = {}

function G.two_hand_direction(rear, front, fallback, minimum)
  if m.dist(rear, front) < (minimum or 0.10) then return m.norm(fallback), false end
  return m.norm(m.sub(front, rear)), true
end

-- Convergence uses a real collision hit, supplied by the caller; no hit means range fallback.
function G.aim_target(origin, direction, range, hit)
  if hit then return hit, true end
  return m.add(origin, m.scale(m.norm(direction), range)), false
end

function G.converged_angles(camera, target)
  if m.dist(camera, target) < 1e-6 then return nil end
  return m.yaw_pitch(m.sub(target, camera))
end

function G.valid_hit(origin,direction,range,result)
  if not result or not result.executed or not result.hit or not result.pos then return nil end
  local delta=m.sub(result.pos,origin)
  local unit=m.norm(direction)
  local along=m.dot(delta,unit)
  local residual=m.len(m.sub(delta,m.scale(unit,along)))
  -- Reject points behind the ray, outside range, or within the unresolved self-contact region.
  if along~=along or residual~=residual or along<0.25 or along>range+0.02 or residual>0.02 then return nil end
  return result.pos
end

-- Analytic two-bone solve, with a pole direction and reach clamping.
function G.solve_arm(shoulder, target, pole, upper, lower)
  assert(upper > 0 and lower > 0, 'arm lengths must be positive')
  local delta = m.sub(target, shoulder)
  local distance = m.len(delta)
  if distance < 1e-6 then return nil, 'target at shoulder' end
  local axis = m.scale(delta, 1 / distance)
  local reach = math.max(math.abs(upper-lower)+1e-5, math.min(upper+lower-1e-5, distance))
  local projected = m.sub(pole, m.scale(axis, m.dot(pole, axis)))
  if m.len(projected) < 1e-6 then
    local alternative = math.abs(axis.y) < 0.9 and m.v(0,1,0) or m.v(1,0,0)
    projected = m.sub(alternative, m.scale(axis, m.dot(alternative,axis)))
  end
  local along = (upper*upper-lower*lower+reach*reach)/(2*reach)
  local height = math.sqrt(math.max(0,upper*upper-along*along))
  return {
    elbow=m.add(shoulder,m.add(m.scale(axis,along),m.scale(m.norm(projected),height))),
    wrist=m.add(shoulder,m.scale(axis,reach)),
    clamped=math.abs(reach-distance)>1e-5,
  }
end

-- Physical head displacement only; this does not move the character through collision.
function G.roomscale_offset(head, reference, yaw, limit)
  local d=m.sub(head,reference)
  local world=m.v(d.x*math.cos(yaw)-d.z*math.sin(yaw),d.y,-d.x*math.sin(yaw)-d.z*math.cos(yaw))
  local horizontal=math.sqrt(world.x*world.x+world.z*world.z)
  if limit and horizontal>limit then
    world.x=world.x*limit/horizontal; world.z=world.z*limit/horizontal
  end
  return world
end
-- Minimal rotation from one segment direction to another. Handles parallel/antiparallel axes.
function G.segment_rotation(from, to)
  local a,b=m.norm(from),m.norm(to)
  assert(m.len(a)>0.5 and m.len(b)>0.5,'zero segment direction')
  local cosine=math.max(-1,math.min(1,m.dot(a,b)))
  if cosine>1-1e-8 then return function(v) return m.v(v.x,v.y,v.z) end end
  local axis=m.cross(a,b)
  if cosine < -1+1e-8 then
    axis=m.cross(a,math.abs(a.y)<0.9 and m.v(0,1,0) or m.v(1,0,0))
  end
  axis=m.norm(axis)
  local sine=math.sqrt(math.max(0,1-cosine*cosine))
  return function(v)
    return m.add(m.add(m.scale(v,cosine),m.scale(m.cross(axis,v),sine)),
      m.scale(axis,m.dot(axis,v)*(1-cosine)))
  end
end

function G.transform_bone(bone, old_pivot, new_pivot, rotate)
  return {pos=m.add(new_pivot,rotate(m.sub(bone.pos,old_pivot))),
    right=rotate(bone.right),up=rotate(bone.up),fwd=rotate(bone.fwd)}
end
return G
