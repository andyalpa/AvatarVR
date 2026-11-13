package.path='lua/lib/?.lua;'..package.path
local callbacks,time,calls={},0,0
local gate=true
avr={
  on=function(e,f) callbacks[e]=f end,
  writes=function() return gate end,
  time=function() return time end,
  player=function() return {pos={x=0,y=1,z=0},fwd={x=0,y=0,z=1}} end,
  stereo={aim_ray=function() return nil end},
  raycast_camera=function(start,finish)
    calls=calls+1
    assert(start.y==1 and finish.z==20)
    return {executed=true,hit=false}
  end,
  log=function() end,
  draw={screen_text=function() end,point=function() error('miss must not draw a point') end},
}
local f=assert(io.open('lua/bench/60_raycast_validation.lua','rb'))
local source=f:read('*a'); f:close()
source=source:gsub('local enabled=false','local enabled=true',1)
assert(load(source,'raycast_validation_test'))()
callbacks.camera_pre(); assert(calls==1)
time=0.05; callbacks.camera_pre(); assert(calls==1)
time=0.11; callbacks.camera_pre(); assert(calls==2)
callbacks.frame()
gate=false; time=1; callbacks.camera_pre(); assert(calls==2)
print('raycast sampling and gate tests passed')
