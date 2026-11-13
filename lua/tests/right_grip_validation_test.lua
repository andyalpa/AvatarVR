package.path='lua/lib/?.lua;'..package.path
local m=require('avrmath')
local callbacks,time,pose={},0,{}
local writes={}
for i=0,89 do
  pose[i]={pos=m.v(i/100,1,0),right=m.v(1,0,0),up=m.v(0,1,0),fwd=m.v(0,0,1)}
end
local original_wrist=pose[56].pos
local original_relative=m.sub(pose[89].pos,pose[56].pos)
avr={on=function(e,f) callbacks[e]=f end,
  writes=function() return true end,value=function() return false end,
  pointers=function() return {pose=1} end,player=function() return {} end,
  bone_count=function() return 90 end,bone=function(i) return pose[i] end,
  set_bone=function(i,b) writes[i]=true; pose[i]=b; return true end,
  time=function() return time end,log=function() end,
  stereo={running=function() return true end,
    grip_transform=function() return {pos=m.v(2,1,0)} end},
}
local file=assert(io.open('lua/bench/55_right_grip_validation.lua','rb'))
local source=file:read('*a'); file:close()
source=source:gsub('local enabled=false','local enabled=true',1)
source=source:gsub("local test_mode='pulse'","local test_mode='follow'",1)
assert(load(source,'right_grip_validation_test'))()
callbacks.camera_post()
assert(math.abs(m.dist(pose[56].pos,original_wrist)-0.15)<1e-6)
assert(m.dist(m.sub(pose[89].pos,pose[56].pos),original_relative)<1e-6)
assert(not writes[0] and not writes[19] and not writes[54])
time=1; callbacks.camera_post()
assert(math.abs(m.dist(pose[56].pos,original_wrist)-0.15)<1e-6,'offset accumulated')
time=21; callbacks.camera_post()
assert(m.dist(pose[56].pos,original_wrist)<1e-6,'owned write was not restored')
print('right grip bounded translation and restoration tests passed')
source=source:gsub("local test_mode='follow'","local test_mode='pulse'",1)
time=0; assert(load(source,'right_grip_pulse_test'))()
callbacks.camera_post(); assert(m.dist(pose[56].pos,original_wrist)<1e-6)
time=3.1; callbacks.camera_post(); assert(math.abs(pose[56].pos.y-original_wrist.y-0.25)<1e-6)
time=6.1; callbacks.camera_post(); assert(m.dist(pose[56].pos,original_wrist)<1e-6)
time=9.1; callbacks.camera_post(); assert(math.abs(pose[56].pos.y-original_wrist.y+0.25)<1e-6)
time=21; callbacks.camera_post(); assert(m.dist(pose[56].pos,original_wrist)<1e-6)
print('right grip pulse phases and restoration tests passed')
