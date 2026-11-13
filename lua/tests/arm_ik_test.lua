package.path='lua/lib/?.lua;'..package.path
local m=require('avrmath')
local function bone(x,y,z)
  return {pos=m.v(x,y,z),right=m.v(1,0,0),up=m.v(0,1,0),fwd=m.v(0,0,1)}
end
local pose={}
for i=0,89 do pose[i]=bone(0,0,0) end
pose[17],pose[18],pose[19]=bone(-0.3,1,0),bone(-0.3,0.5,0),bone(-0.3,0,0)
pose[54],pose[55],pose[56]=bone(0.3,1,0),bone(0.3,0.5,0),bone(0.3,0,0)
pose[20],pose[57]=bone(-0.3,-0.1,0),bone(0.3,-0.1,0)
local writes,callback={},nil
local allowed,tracking=true,true
avr={
  on=function(event,fn) if event=='camera_post' then callback=fn end end,
  value=function() return true end,
  toggle=function() end,
  writes=function() return allowed end,
  player=function() return {} end,
  bone_count=function() return 90 end,
  bone=function(i) return pose[i] end,
  set_bone=function(i,value) writes[i]=value; return true end,
  stereo={running=function() return tracking end,
    grip_transform=function(hand) return bone(hand==0 and -0.3 or 0.3,0.5,0.5) end},
}
local file=assert(io.open('lua/bench/50_arm_ik.lua','rb'))
local source=file:read('*a'); file:close()
assert(load(source,'arm_ik_test'))()
callback()
assert(writes[0]==nil and writes[89]==nil,'camera and weapon must be untouched')
for _,ids in ipairs({{17,18,19,20},{54,55,56,57}}) do
  assert(math.abs(m.dist(writes[ids[1]].pos,writes[ids[2]].pos)-0.5)<1e-5)
  assert(math.abs(m.dist(writes[ids[2]].pos,writes[ids[3]].pos)-0.5)<1e-5)
  assert(math.abs(m.dist(writes[ids[3]].pos,writes[ids[4]].pos)-0.1)<1e-5)
  for _,i in ipairs(ids) do
    local b=writes[i]
    assert(math.abs(m.len(b.right)-1)<1e-5)
    assert(math.abs(m.dot(b.right,b.up))<1e-5)
  end
end
writes={}; allowed=false; callback(); assert(next(writes)==nil)
allowed=true; tracking=false; callback(); assert(next(writes)==nil)
print('arm IK constraints and write-gate tests passed')
