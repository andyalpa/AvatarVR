local frame,logs,attempts=nil,{},0
local valid=false
avr={
  on=function(event,fn) if event=='frame' then frame=fn end end,
  player=function() return valid and {} or nil end,
  pointers=function() return {body=100,entity=999} end,
  read_ptr=function(addr)
    if addr==100 then attempts=attempts+1; return attempts>1 and 200 or nil end
    if addr==200+0x108 then return 300 end
  end,
  read_u32=function() return 0xCCCCCCCC end,
  base=function() return 0 end,
  log=function(message) logs[#logs+1]=message end,
}
dofile('lua/bench/35_collision_owner_probe.lua')
frame(); frame(); assert(#logs==1)
valid=true; frame(); assert(attempts==1)
frame(); assert(attempts==2)
local found=false
for _,line in ipairs(logs) do if line:find('getter words',1,true) then found=true end end
assert(found)
local count=#logs; frame(); assert(#logs==count and attempts==2)
print('collision owner probe retries and status tests passed')
