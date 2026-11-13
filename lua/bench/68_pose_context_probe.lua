-- Read-only descriptor discovery. Candidate vector layout is INFERENCE.
-- A matching array/transform does not prove rendering consumption.
local finished=false
local enabled=false -- completed live capture; retain for deliberate rechecks
if not enabled then return end
local elapsed_start=nil
local function ptr(address)
  if address and address~=0 then return avr.read_ptr(address) end
end
local function position(array,index)
  if not array or array==0 then return nil end
  local address=array+index*64+0x30
  local values={avr.read_f32(address),avr.read_f32(address+4),avr.read_f32(address+8)}
  for i=1,3 do
    if not values[i] or values[i]~=values[i] or math.abs(values[i])>1e7 then return nil end
  end
  return values
end
local function distance(a,b)
  if not a or not b then return nil end
  local sum=0;for i=1,3 do sum=sum+(a[i]-b[i])^2 end
  return math.sqrt(sum)
end
avr.on('frame',function()
  if finished or not avr.player() then return end
  elapsed_start=elapsed_start or avr.time()
  if avr.time()-elapsed_start>20 then avr.log('pose context probe: context unavailable after 20s; stopped');finished=true;return end
  local body=avr.pointers().body
  if not body or body==0 then return end
  local context=ptr(body+0x9E8)
  local vt=context and ptr(context)
  if not vt or vt-avr.base()~=0x5BCD900 then return end
  local references=ptr(context+0x98)
  local count=avr.read_u32(context+0xA0)
  if not references or references==0 or not count or count>8 then return end
  local comp=ptr(body+0x1A8)
  if not comp or comp==0 then return end
  local poses,headers={},{}
  for _,offset in ipairs({0x4F8,0x500}) do
    local pose=ptr(comp+offset)
    local n=pose and avr.read_u32(pose+8)
    if pose and (n==90 or n==192) then
      local array=ptr(pose)
      if array and array~=0 then poses[n]=array;headers[n]={pose=pose,offset=offset} end
    end
  end
  if not poses[90] or not poses[192] then return end
  local objects={context}
  for i=0,count-1 do
    local child=ptr(references+i*8)
    if child and child~=0 then objects[#objects+1]=child end
  end
  local output={}
  local function emit(line) output[#output+1]=line end
  emit(string.format('pose context probe: body=%X context=%X references=%d pose90=%X pose192=%X',body,context,count,poses[90],poses[192]))
  local candidates=0
  for object_index,object in ipairs(objects) do
    -- Only captured context size 0x260 and first 0x100 observed child bytes.
    local limit=object_index==1 and 0x250 or 0xF0
    emit(string.format('pose context probe: object=%X vtableRva=%X bytes=%X',object,(ptr(object) or avr.base())-avr.base(),limit+16))
    for offset=0,limit,8 do
      local n=avr.read_u32(object+offset+8)
      if n==90 or n==192 then
        local array=ptr(object+offset)
        local root=position(array,0)
        local wrist_index=n==90 and 56 or 77
        local wrist=position(array,wrist_index)
        local dr=distance(root,position(poses[n],0))
        local dw=distance(wrist,position(poses[n],wrist_index))
        if dr and dw then
          candidates=candidates+1
          emit(string.format('pose context candidate: object=%X offset=%X array=%X count=%d same_pose_array=%s root_delta=%.6f wrist_delta=%.6f',
            object,offset,array,n,tostring(array==poses[n]),dr,dw))
        end
      end
    end
  end
  -- A guarded read prevents faults, but does not establish a stable snapshot.
  local stable=avr.pointers().body==body and ptr(body+0x9E8)==context and ptr(context)==vt and
    ptr(context+0x98)==references and avr.read_u32(context+0xA0)==count and ptr(body+0x1A8)==comp
  for n,h in pairs(headers) do
    stable=stable and ptr(comp+h.offset)==h.pose and ptr(h.pose)==poses[n] and avr.read_u32(h.pose+8)==n
  end
  local child_index=2
  for i=0,count-1 do
    local child=ptr(references+i*8)
    if child and child~=0 then
      stable=stable and child==objects[child_index];child_index=child_index+1
    end
  end
  stable=stable and child_index==#objects+1
  if stable then
    for _,line in ipairs(output) do avr.log(line) end
    avr.log(string.format('pose context probe: complete candidates=%d; descriptor layout and render ownership unverified',candidates))
  else
    avr.log('pose context probe: identity changed during sampling; capture discarded; stopped')
  end
  finished=true
end)
