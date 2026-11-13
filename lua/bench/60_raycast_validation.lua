-- Bounded first cast validation: auto-stops after 20 seconds; F7 starts a new window.
-- Uses the engine CAMERA filter. Weapon collision/self-ignore is not yet established.
local enabled=false
local m=require('avrmath')
local last,result,last_log=nil,nil,nil
local started=nil
local stopped=false
local waiting=nil
local function wait_reason(message)
  if waiting~=message then avr.log('raycast validation: '..message); waiting=message end
end
avr.on('camera_pre',function()
  if not enabled then result=nil; return end
  if not avr.writes() then result=nil; wait_reason('waiting: Script writes is off'); return end
  if not avr.raycast_camera then result=nil; wait_reason('waiting: native raycast API missing'); return end
  waiting=nil
  local now=avr.time()
  if stopped then return end
  if not started then started=now end
  if now-started>=20 then
    stopped=true; result=nil; avr.log('raycast validation: 20 second window complete'); return
  end
  if last and now-last<0.1 then return end -- bounded ten queries per second
  last=now
  local p=avr.player(); if not p then result=nil; return end
  local origin,direction=avr.stereo.aim_ray(1)
  if not origin then origin,direction=p.pos,p.fwd end
  result=avr.raycast_camera(origin,m.add(origin,m.scale(direction,20)))
  if result and (not last_log or now-last_log>=1) then
    last_log=now
    local geometry=''
    if result.hit then
      local delta=m.sub(result.pos,origin)
      local unit=m.norm(direction)
      local along=m.dot(delta,unit)
      local perpendicular=m.len(m.sub(delta,m.scale(unit,along)))
      geometry=string.format(' origin=%s dir=%s along=%.3f residual=%.5f normal_len=%.4f',
        m.fmt(origin),m.fmt(unit),along,perpendicular,m.len(result.normal))
    end
    avr.log('raycast validation: executed='..tostring(result.executed)..' hit='..tostring(result.hit)..
      (result.hit and (' pos='..m.fmt(result.pos)..' normal='..m.fmt(result.normal)) or '')..geometry)
  end
end)
avr.on('frame',function()
  if not enabled or not result then return end
  local message=not result.executed and 'Collision query unavailable' or
    (result.hit and 'Camera-filter collision hit' or 'Camera-filter query: no hit')
  avr.draw.screen_text(0.02,0.24,message,0xFFD040FF)
  if result.hit then
    avr.draw.point(result.pos,8,0xFFD040FF)
    avr.draw.line(result.pos,m.add(result.pos,m.scale(result.normal,0.3)),0x40FFFFFF,2)
  end
end)
