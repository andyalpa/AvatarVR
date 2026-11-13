-- Read-only owner discovery. No virtual calls, physics calls or writes.
-- Logs once per entity lifetime to SCRIPTS.log; permits offline analysis of its world getter.
local seen=nil
local captured=nil
local last_status=nil
local function status(message)
  if message~=last_status then avr.log('collision owner probe: '..message); last_status=message end
end
avr.on('frame',function()
  local p=avr.player()
  if not p then seen=nil; captured=nil; status('waiting for valid gameplay player'); return end
  local pointers=avr.pointers()
  -- Camera collision caller 144198AA0 reads this object's +94/+9C position,
  -- then virtual +108. The intermediate "entity" context is not this object.
  local entity=pointers.body
  if not entity or entity==0 then status('body pointer missing'); return end
  if seen==entity and captured==entity then return end
  local vt=avr.read_ptr(entity)
  if not vt or vt==0 then status(string.format('entity=%X vtable unreadable or null',entity)); return end
  local getter=avr.read_ptr(vt+0x108)
  if not getter or getter==0 then status(string.format('entity=%X vt=%X slot108 unreadable or null',entity,vt)); return end
  if seen~=entity then
  seen=entity; captured=nil
  last_status='getter found'
  avr.log(string.format('collision owner probe: body=%X vtRva=%X getter108Rva=%X',entity,vt-avr.base(),getter-avr.base()))
  local bytes={}
  for offset=0,12,4 do
    local word=avr.read_u32(getter+offset)
    if word then bytes[#bytes+1]=string.format('%08X',word) end
  end
  avr.log('collision owner probe: getter words '..table.concat(bytes,' '))
  end
  -- Exact live getter guard: mov rax,[rcx+D8]; ret. Other implementations remain unknown.
  if getter-avr.base()~=0x9630D0 or avr.read_u32(getter)~=0xD8818B48 or
     avr.read_u32(getter+4)~=0xC3000000 then
    status('world getter differs from confirmed accessor; chain skipped'); captured=entity; return
  end
  local world=avr.read_ptr(entity+0xD8)
  local subsystem=world and world~=0 and avr.read_ptr(world+0xD0)
  local owner=subsystem and subsystem~=0 and avr.read_ptr(subsystem+0x18)
  local query=owner and owner~=0 and avr.read_ptr(owner+8)
  local query_vt=query and query~=0 and avr.read_ptr(query)
  local ray=query_vt and query_vt~=0 and avr.read_ptr(query_vt+0x10)
  if not ray or ray==0 then status('world collision chain incomplete; retrying'); return end
  captured=entity
  avr.log(string.format('collision owner probe: world=%X subsystem=%X owner=%X query=%X queryVtRva=%X rayRva=%X',
    world,subsystem,owner,query,query_vt-avr.base(),ray-avr.base()))
end)

-- New native bridge: compare readiness on the actual camera game thread, not Present.
local thread_status=nil
avr.on('camera_pre',function()
  if not avr.collision_context then return end
  local context=avr.collision_context()
  local message=string.format('collision thread probe: known=%s ready=%s query=%X',
    tostring(context.known_interface),tostring(context.thread_ready),context.query or 0)
  if message~=thread_status then avr.log(message); thread_status=message end
end)
