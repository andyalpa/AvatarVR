local time,writes,count,array=0,true,192,0x10000
local selected=0
local real_open=io.open
io.open=function(path,mode)
  if path==[[H:\Gaming\UbisoftLibrary\AFOP\scripts\AvatarVR_posetiming.txt]] then
    return {write=function() end,close=function() end}
  end
  return real_open(path,mode)
end
local callbacks,memory,logs={},{},{}
local function reset()
  callbacks={}; memory={}
  for i=0,191 do memory[0x10000+i*64+0x34]=1 end
end
reset()
avr={on=function(e,f) callbacks[e]=f end, log=function(line) logs[#logs+1]=line end,time=function() return time end,
  writes=function() return writes end,value=function() return false end,player=function() return {} end,
  pointers=function() return {body=0x1000} end,
  read_ptr=function(a) return ({[0x11A8]=0x2000,[0x24F8]=0x3000,[0x2500]=0x3000,[0x3000]=array,[0x3050]=0x4000})[a] end,
  read_u32=function(a) return ({[0x3008]=count,[0x3084]=selected})[a] end,
  read_f32=function(a) return memory[a] end,
  write_f32=function(a,v) assert(writes); memory[a]=v; return true end}
local function loadtest(variant)
  local file=assert(io.open('lua/bench/67_alternate_hand_pulse.lua','r'))
  local source=file:read('*a');file:close()
  source=source:gsub('local enabled=false','local enabled=true',1)
  source=source:gsub('local variant=90','local variant='..tostring(variant or 192),1)
  assert(load(source,'alternate_hand_pulse_test'))()
end
local wrist=0x10000+77*64+0x34
loadtest(); callbacks.camera_post()
time=3.1; callbacks.camera_post(); assert(memory[wrist]==1.25)
callbacks.camera_pre()
callbacks.camera_post(); assert(memory[wrist]==1.25,'accumulated displacement')
assert(memory[0x10000+39*64+0x34]==1,'left hand changed')
assert(memory[0x10000+191*64+0x34]==1,'world attachment changed')
time=6.1; callbacks.camera_post(); assert(memory[wrist]==1)
time=9.1; callbacks.camera_post(); assert(memory[wrist]==0.75)
memory[wrist]=2; callbacks.unload(); assert(memory[wrist]==2,'new animation overwritten')
reset();time=0;loadtest();callbacks.camera_post();time=3.1;callbacks.camera_post()
time=21;callbacks.frame();assert(memory[wrist]==1,'timeout did not restore')
reset();time=0;count=90;loadtest();callbacks.camera_post();time=4;callbacks.camera_post();assert(memory[wrist]==1)
count=192;writes=false;callbacks.camera_post();assert(memory[wrist]==1)
writes=true;reset();time=0;logs={};loadtest();callbacks.camera_post();time=3.1;callbacks.camera_post()
callbacks.camera_pre();memory[wrist]=2;time=3.2;callbacks.frame();time=21;callbacks.frame()
local joined=table.concat(logs,'\n')
assert(joined:find('event=camera_pre survived=1 changed=0',1,true))
assert(joined:find('first_change_event=frame',1,true))
assert(joined:find('baseline_y=1.000000 expected_y=1.250000 readback_y=1.250000',1,true))
writes=true;reset();time=0;loadtest();callbacks.camera_post();time=3.1;callbacks.camera_post()
array=0x20000;callbacks.camera_post();assert(memory[wrist]==1.25,'stale pose restored')
array=0x10000;count=90;selected=4;time=0;reset();loadtest(90);callbacks.camera_post()
time=3.1;callbacks.camera_post()
assert(memory[0x10000+56*64+0x34]==1.25,'first-person wrist did not pulse')
assert(memory[0x10000+79*64+0x34]==1.25,'right weapon attachment did not follow')
assert(memory[0x10000+19*64+0x34]==1 and memory[0x10000+42*64+0x34]==1,'left hand/attachment changed')
assert(memory[0x10000+89*64+0x34]==1 and memory[0x10000+0x34]==1,'world attachment/camera changed')
time=21;callbacks.frame();assert(memory[0x10000+56*64+0x34]==1,'first-person wrist not restored')
print('alternate hand pulse: phases, no accumulation, exclusions, owned restore, timeout, gates and stale identity passed')
