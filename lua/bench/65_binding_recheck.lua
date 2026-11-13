-- Read-only, bounded follow-up to the native capture; no engine calls or writes.
-- Runtime addresses are accepted only when the latest capture's pose matches
-- this process's current pose. A new native capture is needed after a mismatch.
local path=[[H:\Gaming\UbisoftLibrary\AFOP\scripts\BONE_COPY_PROBE.log]]
local enabled=false -- completed; retain for a deliberate future capture
if not enabled then return end
local owners,pose={},nil
local f=io.open(path,'r')
if f then
  for line in f:lines() do
    if line:find('persistent%-binding observation window started') then owners={}; pose=nil end
    local owner,p=line:match('pack=(%x+).-pose=(%x+)')
    if owner and p and tonumber(p,16)~=0 then
      owners[tonumber(owner,16)]=true; pose=tonumber(p,16)
    end
  end
  f:close()
end
local elapsed,last,started,start=0,-1,false,nil
local states={}
local function ptr(p) if p and p~=0 then return avr.read_ptr(p) end end
avr.on('frame',function()
  if elapsed>=20 then return end
  if not avr.player() then return end
  local p=avr.pointers()
  if not pose or p.pose~=pose then
    if not started then avr.log('binding recheck: latest capture pose differs; skipped runtime addresses'); started=true end
    elapsed=20; return
  end
  if not started then avr.log('binding recheck: 20 second read-only resampling started'); started=true end
  start=start or avr.time()
  elapsed=avr.time()-start
  if elapsed-last<1 then return end
  last=elapsed
  for owner in pairs(owners) do
    local groups=ptr(owner+0x40) or 0
    local candidate=avr.read_u32(owner+0x48) or 0
    local dirty=ptr(owner+0xA8) or 0
    local count=avr.read_u32(owner+0xB0) or 0
    local pending=ptr(owner+0x70) or 0
    local state=string.format('%X/%u/%X/%u/%X',groups,candidate,dirty,count,pending)
    if states[owner]~=state then
      states[owner]=state
      avr.log(string.format('binding recheck: owner=%X groups=%X candidateCount=%u dirty=%X dirtyCount=%u pending=%X',owner,groups,candidate,dirty,count,pending))
    end
  end
  if elapsed>=20 then avr.log('binding recheck: complete') end
end)
