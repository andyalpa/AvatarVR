local callbacks,logs={},{}
local reads,writes=0,0
local memory={[0x19E8]=0x2000,[0x2000]=0x5BCD900,[0x2098]=0x3000,[0x3000]=0x4000,
 [0x11A8]=0x5000,[0x54F8]=0x6000,[0x5500]=0x7000,[0x6000]=0x8000,[0x7000]=0xC000,
 [0x2040]=0xC000,[0x4000]=0x1234}
local counts={[0x20A0]=1,[0x6008]=192,[0x7008]=90,[0x2048]=90}
avr={on=function(e,f) callbacks[e]=f end,player=function() return {} end,pointers=function() return {body=0x1000} end,
 base=function() return 0 end,time=function() return 0 end,log=function(s) logs[#logs+1]=s end,
 read_ptr=function(a) reads=reads+1;return memory[a] end,read_u32=function(a) reads=reads+1;return counts[a] end,
 read_f32=function(a) reads=reads+1;if a>=0x8000 then return 1 end end,
 write_f32=function() writes=writes+1;error('read-only probe attempted a write') end}
local function loadprobe()
  local file=assert(io.open('lua/bench/68_pose_context_probe.lua','r'))
  local source=file:read('*a');file:close()
  assert(load(source:gsub('local enabled=false','local enabled=true',1),'pose_context_probe_test'))()
end
loadprobe()
callbacks.frame()
local capture=table.concat(logs,'\n')
assert(capture:find('same_pose_array=true',1,true))
assert(capture:find('complete candidates=1',1,true))
local previous=reads;callbacks.frame();assert(reads==previous,'probe repeated after completion')
assert(writes==0 and reads<210,'unexpected writes or unbounded reads')
callbacks={};logs={};local root_reads=0
avr.pointers=function() root_reads=root_reads+1;return {body=root_reads==1 and 0x1000 or 0x1001} end
loadprobe();callbacks.frame()
capture=table.concat(logs,'\n')
assert(capture:find('capture discarded',1,true))
assert(not capture:find('pose context candidate:',1,true),'stale candidate emitted')
print('pose context probe: matching descriptor, bounded reads, no writes and single capture passed')
