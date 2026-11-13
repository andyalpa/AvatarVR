-- Bounded attachment-path test. Candidate bone89 is inferred, not a verified weapon API.
-- Reload starts one20s window while Script writes is on. No aim/rotation/camera changes.
local enabled=false
local test_mode='pulse' -- pulse isolates visible consumption; follow is the earlier15cm grip test
local m=require('avrmath')
local ids={13,14,42,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,89}
local saved,written,pose_id=nil,nil,nil
local start,last_log,stopped=nil,nil,false
local status=nil
local function report(message)
  if status~=message then avr.log('right grip validation: '..message); status=message end
end
local function equal(a,b)
  if not a or not b then return false end
  for _,field in ipairs({'pos','right','up','fwd'}) do
    if m.dist(a[field],b[field])>0.001 then return false end
  end
  return true
end
local function restore()
  if saved and avr.writes() and avr.pointers().pose==pose_id then
    for i,b in pairs(saved) do
      -- Never overwrite a newly animated matrix that has replaced our test write.
      if equal(avr.bone(i),written[i]) then avr.set_bone(i,b) end
    end
  end
  saved,written=nil,nil
end
avr.on('unload',restore)
avr.on('camera_post',function()
  if not enabled or stopped then return end
  if not avr.writes() then report('waiting: Script writes off'); return end
  if avr.value('IK: controller arms') or avr.value('aim: two hands') then
    restore(); report('waiting: turn IK and two-hand aim off'); return
  end
  if not avr.player() or not avr.stereo.running() or not avr.stereo.grip_transform or avr.bone_count()~=90 then
    restore(); report('waiting: gameplay/stereo/pose unavailable'); return
  end
  local grip=avr.stereo.grip_transform(1)
  if not grip then restore(); report('waiting: right grip tracking'); return end
  local now=avr.time()
  if not start then start=now end
  if now-start>=20 then restore(); stopped=true; report('20 second window complete; writes stopped'); return end
  local current_pose=avr.pointers().pose
  if pose_id and current_pose~=pose_id then saved,written=nil,nil; stopped=true; report('pose changed; stopped'); return end
  restore() -- prevents accumulated offsets if the engine has not regenerated the pose
  pose_id=current_pose
  local hand=avr.bone(56)
  if not hand then report('right wrist unavailable'); return end
  local delta=m.sub(grip.pos,hand.pos)
  local gap=m.len(delta)
  if gap~=gap or gap>5 then report('invalid grip gap; no write'); return end
  if gap>0.15 then delta=m.scale(delta,0.15/gap) end
  local phase='follow'
  if test_mode=='pulse' then
    local step=math.floor((now-start)/3)
    local height=step==1 and 0.25 or (step==3 and -0.25 or 0)
    delta=m.v(0,height,0)
    phase=step==1 and 'UP25cm' or (step==3 and 'DOWN25cm' or 'BASELINE')
  end
  local original,pending={},{}
  for _,i in ipairs(ids) do
    local b=avr.bone(i)
    if not b then report('candidate bone unavailable; no write'); return end
    original[i]=b
    pending[i]={pos=m.add(b.pos,delta),right=b.right,up=b.up,fwd=b.fwd}
  end
  saved,written=original,{}
  for _,i in ipairs(ids) do
    if not avr.set_bone(i,pending[i]) then restore(); stopped=true; report('write failed; stopped'); return end
    written[i]=pending[i]
  end
  if not last_log or now-last_log>=1 then
    last_log=now
    local readback=avr.bone(56)
    local error=readback and m.dist(readback.pos,pending[56].pos) or -1
    avr.log(string.format('right grip validation: phase=%s gap=%.3fm applied=%s wrist=%s prop=%s readback_error=%.5f',
      phase,gap,m.fmt(delta),m.fmt(pending[56].pos),m.fmt(pending[89].pos),error))
  end
end)
