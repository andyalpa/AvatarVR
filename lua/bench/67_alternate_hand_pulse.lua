-- Session-only visual discriminator. Writes only Y translation in the named
-- named right-hand subtree and C_weaponMain; no camera or collision writes.
local saved,identity,start,last=nil,nil,nil,-1
local enabled=false -- both flat-screen visual checks completed without visible movement
local variant=90 -- 90: first-person pose; 192: completed alternate-pose test
local pose_offset=variant==90 and 0x500 or 0x4F8
local selection=variant==90 and 4 or 0
local wrist_index=variant==90 and 56 or 77
local attachment_index=variant==90 and 79 or 100
if not enabled then return end
local stopped=false
local timing={frame={0,0,0},camera_pre={0,0,0},camera_post={0,0,0}}
local written_at=nil
local first_loss=nil
local status=nil
local ids={variant==90 and 14 or 73}
for i=wrist_index,(variant==90 and 80 or 101) do ids[#ids+1]=i end
local function report(text)
  if status~=text then avr.log('alternate hand pulse: '..text); status=text end
end
local function pointer(address)
  if address and address~=0 then return avr.read_ptr(address) end
end
local function context()
  if not avr.player() then return nil end
  local body=avr.pointers().body
  local comp=body and pointer(body+0x1A8)
  local pose=comp and pointer(comp+pose_offset)
  if not pose or pose==0 or avr.read_u32(pose+8)~=variant or avr.read_u32(pose+0x84)~=selection then return nil end
  local array=pointer(pose)
  if not array or array==0 then return nil end
  return {body=body,pose=pose,array=array,asset=pointer(pose+0x50)}
end
local function same(a,b)
  return a and b and a.body==b.body and a.pose==b.pose and a.array==b.array and a.asset==b.asset
end
local function restore()
  if saved and avr.writes() and same(context(),identity) then
    for address,v in pairs(saved) do
      local current=avr.read_f32(address)
      -- Leave newly animated values alone. Restore only our own exact write.
      if current and math.abs(current-v.written)<0.00002 then avr.write_f32(address,v.original) end
    end
  end
  saved=nil
end
local function observe(event)
  if not saved or not identity or not same(context(),identity) then return end
  local wrist=identity.array+wrist_index*64+0x34
  local record=saved[wrist]
  if not record then return end
  local current=avr.read_f32(wrist)
  local outcome=3
  if current and current==current then
    outcome=math.abs(current-record.written)<0.00002 and 1 or 2
    if outcome==2 and not first_loss then
      first_loss={event=event,age=(avr.time()-(written_at or avr.time()))*1000}
    end
  end
  timing[event][outcome]=timing[event][outcome]+1
end
local function stop(reason)
  restore(); stopped=true; report(reason)
  for _,event in ipairs({'frame','camera_pre','camera_post'}) do
    local t=timing[event]
    avr.log(string.format('alternate hand persistence: event=%s survived=%d changed=%d unreadable=%d',event,t[1],t[2],t[3]))
  end
  if first_loss then avr.log(string.format('alternate hand persistence: first_change_event=%s age_ms=%.3f',first_loss.event,first_loss.age)) end
end
avr.on('unload',restore)
avr.on('camera_pre',function() if not stopped then observe('camera_pre') end end)
avr.on('frame',function()
  if not stopped then observe('frame') end
  if not stopped and start and avr.time()-start>=20 then stop('20 second window complete; writes stopped') end
end)
avr.on('camera_post',function()
  if stopped then return end
  if not avr.writes() then restore(); report('waiting: Script writes off'); return end
  if avr.value('IK: controller arms') or avr.value('aim: two hands') then
    if start then stop('conflicting gameplay feature enabled; stopped') else report('waiting: IK and two-hand aim must be off') end
    return
  end
  local c=context()
  if not c then
    if start then stop('selected pose unavailable; stopped') else report('waiting: selected gameplay pose') end
    return
  end
  if identity and not same(c,identity) then stop('pose identity changed; stopped'); return end
  observe('camera_post')
  restore()
  identity=c
  local values={}
  for _,index in ipairs(ids) do
    local address=c.array+index*64+0x34
    local value=avr.read_f32(address)
    if not value or value~=value or math.abs(value)>1e6 then stop('invalid translation; stopped'); return end
    values[address]={original=value,index=index}
  end
  local now=avr.time()
  if not start then
    -- Native observer was explicitly reset off before this comparison.
    -- Start its read-only capture only after this pulse's existing write gates pass.
    local control=io.open([[H:\Gaming\UbisoftLibrary\AFOP\scripts\AvatarVR_posetiming.txt]],'w')
    if not control then stop('cannot arm phase recorder; stopped before pulse');return end
    control:write('on\n');control:close()
    start=now
  end
  local elapsed=now-start
  if elapsed>=20 then stop('20 second window complete; writes stopped'); return end
  local offset=elapsed>=3 and elapsed<6 and 0.25 or (elapsed>=9 and elapsed<12 and -0.25 or 0)
  local phase=offset>0 and 'UP25cm' or (offset<0 and 'DOWN25cm' or 'BASELINE')
  saved={}
  if offset~=0 then
    for address,v in pairs(values) do
      v.written=v.original+offset
      if not avr.write_f32(address,v.written) then stop('write failed; stopped'); return end
      saved[address]=v
    end
    written_at=now
  end
  if now-last>=1 then
    last=now
    local wrist=c.array+wrist_index*64+0x34
    local readback=avr.read_f32(wrist)
    local error=readback and math.abs(readback-(values[wrist].original+offset)) or -1
    avr.log(string.format('alternate hand pulse: phase=%s elapsed=%.1f pose=%X array=%X bones=%d wrist=%d attachment=%d baseline_y=%.6f expected_y=%.6f readback_y=%.6f readback_error=%.6f',
      phase,elapsed,c.pose,c.array,variant,wrist_index,attachment_index,values[wrist].original,values[wrist].original+offset,readback or 0,error))
  end
end)
