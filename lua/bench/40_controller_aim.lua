-- Experimental aim ownership: fixed view/body yaw, game look follows the controller.
-- Camera-filter convergence experiment; weapon/self-ignore filtering remains OPEN.
-- Re-enable after a level change to capture a new view basis.
local m=require('avrmath')
local g=require('vrgameplay')
local yaw,identity=nil,nil
local last_query,query_origin,query_direction,query_result=nil,nil,nil,nil
local last_log=nil
local last_turn=nil
local turn_rate=2.6 -- radians per second at full stick
local function release()
  yaw,identity=nil,nil
  last_query,query_origin,query_direction,query_result=nil,nil,nil,nil
  if avr.stereo.body_yaw then avr.stereo.body_yaw(nil) end
end
avr.on('unload',release)
avr.on('frame',function()
  avr.toggle('aim: independent view',false)
  avr.toggle('aim: two hands',false)
end)
avr.on('camera_pre',function()
  if not avr.stereo.body_yaw or not avr.value('aim: independent view') or not avr.writes() then
    release(); return
  end
  -- Do not fight the earlier test script, which is another owner of the same angles.
  if avr.value('look: hold') or avr.value('look: follow right controller') then release(); return end
  local p=avr.player()
  if not p or not p.look_ok or not avr.stereo.running() then release(); return end
  local current=avr.pointers().holder
  if identity and identity~=current then release(); return end
  local weapon=avr.weapon_hand and avr.weapon_hand() or 1  -- run 59: left-handed mode aims with the left controller
  local origin,direction=avr.stereo.aim_ray(weapon)
  if not origin or not direction then release(); return end
  if not yaw then yaw=p.yaw; identity=current end
  -- Mod-owned turning (2026-10-15): the view yaw is pinned, so the stick used to rotate only the character under it.
  -- The right stick's X is withheld from the game and turns the pinned view instead (smooth turn, 0.2 deadzone).
  if avr.stereo.turn_input then
    local rx=avr.stereo.turn_input()
    local t=avr.time()
    local dt=last_turn and math.min(0.1,math.max(0,t-last_turn)) or 0
    last_turn=t
    if math.abs(rx)>0.2 then yaw=yaw+(rx-0.2*(rx>0 and 1 or -1))/0.8*turn_rate*dt end
  end
  avr.stereo.body_yaw(yaw)
  if avr.value('aim: two hands') and avr.stereo.grip_transform then
    local rear,front=avr.stereo.grip_transform(weapon),avr.stereo.grip_transform(1-weapon)  -- weapon hand, support hand
    if rear and front then direction=g.two_hand_direction(rear.pos,front.pos,direction,0.15) end
  end
  local now=avr.time()
  if avr.raycast_camera and (not last_query or now-last_query>=0.1) then
    last_query=now; query_origin=origin; query_direction=direction
    query_result=avr.raycast_camera(origin,m.add(origin,m.scale(direction,20)))
  end
  local hit=nil
  if last_query and now-last_query<=0.12 and query_origin and
     m.dist(origin,query_origin)<=0.02 and m.angle_deg(direction,query_direction)<=0.5 then
    hit=g.valid_hit(origin,direction,20,query_result)
  end
  local target=g.aim_target(origin,direction,20,hit)
  if not last_log or now-last_log>=1 then
    last_log=now
    avr.log('controller aim convergence: '..(hit and ('collision '..m.fmt(hit)) or 'range fallback'))
  end
  local aim_yaw,aim_pitch=g.converged_angles(p.pos,target)
  if aim_yaw then avr.set_look(aim_yaw,aim_pitch) end
end)
