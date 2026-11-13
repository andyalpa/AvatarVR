package.path='lua/lib/?.lua;'..package.path
local events,values={},{}
local writes,tracked=true,true
local heading,angle=nil,nil
local holder=1
avr={
  on=function(event,fn) events[event]=fn end,
  toggle=function(name,default) if values[name]==nil then values[name]=default end end,
  value=function(name) return values[name] end,
  writes=function() return writes end,
  time=function() return 0 end,
  log=function() end,
  player=function() return {pos={x=0,y=0,z=0},yaw=1,look_ok=true} end,
  pointers=function() return {holder=holder} end,
  set_look=function(y,p) angle={y,p} end,
  stereo={
    running=function() return true end,
    body_yaw=function(y) heading=y end,
    aim_ray=function() if tracked then return {x=0,y=0,z=0},{x=0,y=0,z=1} end end,
  },
}
dofile('lua/bench/40_controller_aim.lua')
events.frame(); events.camera_pre(); assert(heading==nil and angle==nil)
values['aim: independent view']=true
events.camera_pre(); assert(heading==1 and angle[1]==0 and angle[2]==0)
tracked=false; events.camera_pre(); assert(heading==nil)
tracked=true; events.camera_pre(); assert(heading==1)
values['look: hold']=true; events.camera_pre(); assert(heading==nil)
values['look: hold']=false; writes=false; events.camera_pre(); assert(heading==nil)
writes=true; events.camera_pre(); events.unload(); assert(heading==nil)
print('controller aim ownership tests passed')
