-- Read-only comparison of native +4F8 and +500 pose consumers.
-- Bone indices are observed candidates, not named matches across skeletons.
local start,last=nil,-1
local done=false
local enabled=false -- named skeleton resources captured; avoid repeated sampling
if not enabled then return end
local seen={}
local function pointer(address)
  if address and address~=0 then return avr.read_ptr(address) end
end
local function header(pose)
  if not pose or pose==0 then return nil end
  local array=pointer(pose)
  local count=avr.read_u32(pose+8)
  if not array or array==0 or not count or count<1 or count>512 then return nil end
  return {pose=pose,array=array,count=count}
end
local function position(h,index)
  if not h or index>=h.count then return nil end
  local address=h.array+index*64+0x30
  local x,y,z=avr.read_f32(address),avr.read_f32(address+4),avr.read_f32(address+8)
  if not x or not y or not z then return nil end
  for _,v in ipairs({x,y,z}) do if v~=v or math.abs(v)>1e7 then return nil end end
  return {x,y,z}
end
local function describe(h)
  if not h then return 'unavailable' end
  local items={string.format('pose=%X array=%X count=%d',h.pose,h.array,h.count)}
  for _,index in ipairs({0,19,56,89}) do
    local p=position(h,index)
    if p then items[#items+1]=string.format('b%d=(%.3f %.3f %.3f)',index,p[1],p[2],p[3]) end
  end
  return table.concat(items,' ')
end
local function resource_context(h,label)
  if not h or seen[h.pose] then return end
  seen[h.pose]=true
  -- STATIC lookup uses the interface stored at *(pose+50)+850, then
  -- its first virtual method. Observe its address/code; never invoke it.
  local asset=pointer(h.pose+0x50)
  local interface=asset and asset~=0 and pointer(asset+0x850)
  local getter=interface and interface~=0 and pointer(interface)
  local selection=avr.read_u32(h.pose+0x84)
  local words={}
  if getter and getter~=0 then
    for offset=0,12,4 do
      local value=avr.read_u32(getter+offset)
      if value then words[#words+1]=string.format('%08X',value) end
    end
  end
  avr.log(string.format('pose resource: %s pose=%X asset=%X interface=%X getterRva=%X selection=%s words=%s',
    label,h.pose,asset or 0,interface or 0,getter and getter~=0 and getter-avr.base() or 0,
    tostring(selection),table.concat(words,' ')))
end
avr.on('camera_post',function()
  if done or not avr.player() then return end
  start=start or avr.time()
  local elapsed=avr.time()-start
  if elapsed>=20 then avr.log('pose compare: complete'); done=true; return end
  if elapsed-last<1 then return end
  last=elapsed
  local p=avr.pointers()
  local comp=p.body and pointer(p.body+0x1A8)
  if not comp or comp==0 then return end
  local a,b=header(pointer(comp+0x4F8)),header(pointer(comp+0x500))
  resource_context(a,'4F8'); resource_context(b,'500')
  avr.log('pose compare: 4F8 '..describe(a)..' | 500 '..describe(b)..
    ' sharedArray='..tostring(a and b and a.array==b.array or false))
end)
