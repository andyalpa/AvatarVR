-- Read-only preview. Disabled until explicitly enabled here, then F7 reload.
-- The yellow point is a range fallback, NOT a collision hit.
local enabled = false
local m=require('avrmath')
local g=require('vrgameplay')
local reference=nil
avr.on('frame',function()
  if not enabled then return end
  local p=avr.player()
  if not p then reference=nil; return end
  local rear,dir=avr.stereo.aim_ray(1)
  local front=avr.stereo.aim_ray(0)
  if rear and dir then
    local twohand=false
    if front then dir,twohand=g.two_hand_direction(rear,front,dir,0.15) end
    local target=g.aim_target(rear,dir,10)
    avr.draw.line(rear,target,0xFFD040FF,2)
    avr.draw.text(target,twohand and 'Two-hand range preview (no collision)' or 'Aim range preview (no collision)',0xFFD040FF)
    local grip=avr.stereo.grip_transform and avr.stereo.grip_transform(1)
    -- Older DLLs can still show the ray; IK previews require the world-space grip bridge.
    local shoulder=avr.bone(54)
    local elbow=avr.bone(55)
    local wrist=avr.bone(56)
    if grip and shoulder and elbow and wrist then
      local solution=g.solve_arm(shoulder.pos,grip.pos,m.sub(elbow.pos,shoulder.pos),
        m.dist(shoulder.pos,elbow.pos),m.dist(elbow.pos,wrist.pos))
      if solution then
        avr.draw.line(shoulder.pos,solution.elbow,0x40FFFFFF,2)
        avr.draw.line(solution.elbow,solution.wrist,0x40FFFFFF,2)
      end
    end
  end
  local views=avr.xr.views()
  if views and views.head and views.head.valid then
    if not reference then reference={pos=views.head.pos,yaw=p.yaw} end
    local offset=g.roomscale_offset(views.head.pos,reference.pos,reference.yaw,0.5)
    avr.draw.screen_text(0.02,0.20,'Physical offset preview '..m.fmt(offset)..' (no camera/body writes)',0x40FFFFFF)
  else reference=nil end
end)
