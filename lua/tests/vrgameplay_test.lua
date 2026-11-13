package.path = 'lua/lib/?.lua;' .. package.path
local m=require('avrmath')
local g=require('vrgameplay')
local function near(a,b) assert(math.abs(a-b)<1e-4, tostring(a)..' != '..tostring(b)) end
local direction,active=g.two_hand_direction(m.v(0,0,0),m.v(0,0,1),m.v(1,0,0))
assert(active); near(direction.z,1)
direction,active=g.two_hand_direction(m.v(0,0,0),m.v(0,0,0),m.v(1,0,0))
assert(not active); near(direction.x,1)
for _,target in ipairs({m.v(0,0,1),m.v(0,0,4),m.v(0.001,0,0)}) do
  local solution=g.solve_arm(m.v(0,0,0),target,m.v(0,-1,0),0.6,0.5)
  near(m.len(solution.elbow),0.6)
  near(m.dist(solution.elbow,solution.wrist),0.5)
end
assert(g.solve_arm(m.v(0,0,0),m.v(0,0,0),m.v(0,1,0),1,1)==nil)
local offset=g.roomscale_offset(m.v(2,0.2,0),m.v(0,0,0),math.pi/2,0.5)
near(offset.x,0); near(offset.z,-0.5); near(offset.y,0.2)
local target,hit=g.aim_target(m.v(0,0,0),m.v(0,0,1),10)
assert(not hit); near(target.z,10)
local collision=m.v(0,0,2)
target,hit=g.aim_target(m.v(0,0,0),m.v(0,0,1),10,collision)
assert(hit and target==collision)
local yaw,pitch=g.converged_angles(m.v(0,0,0),m.v(1,0,1))
near(yaw,math.pi/4); near(pitch,0)
print('vrgameplay math tests passed')
for _,destination in ipairs({m.v(0,1,0),m.v(-1,0,0),m.v(1,0,0)}) do
  local rotate=g.segment_rotation(m.v(1,0,0),destination)
  near(m.dist(rotate(m.v(1,0,0)),destination),0)
  near(m.len(rotate(m.v(0,0,1))),1)
end
local translated=g.transform_bone({pos=m.v(2,0,0),right=m.v(1,0,0),up=m.v(0,1,0),fwd=m.v(0,0,1)},
  m.v(1,0,0),m.v(0,1,0),g.segment_rotation(m.v(1,0,0),m.v(0,1,0)))
near(translated.pos.x,0); near(translated.pos.y,2)
print('bone rigid transform tests passed')
local origin,direction=m.v(0,0,0),m.v(0,0,1)
assert(g.valid_hit(origin,direction,20,{executed=true,hit=true,pos=m.v(0,0,2)}))
for _,point in ipairs({m.v(1,0,2),m.v(0,0,-1),m.v(0,0,21),m.v(0,0,0.1)}) do
  assert(not g.valid_hit(origin,direction,20,{executed=true,hit=true,pos=point}))
end
assert(not g.valid_hit(origin,direction,20,{executed=false,hit=true,pos=m.v(0,0,2)}))
print('collision target acceptance tests passed')
