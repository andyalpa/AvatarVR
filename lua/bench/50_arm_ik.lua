-- LIVE named arm map, disabled by default. Camera-post timing/mesh consumption is OPEN.
-- Enable "IK: controller arms" and Script writes for a controlled test.
-- No camera/root bone changes; hand orientation preserves animated twist, not controller roll yet.
local m=require('avrmath')
local g=require('vrgameplay')
local skeletons=require('vr_skeletons')
local arms={
  {hand=0,shoulder=17,elbow=18,wrist=19,children={20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,43},
    upper_twist={49,50,51},lower_twist={44,45,46,47,48}},
  {hand=1,shoulder=54,elbow=55,wrist=56,children={57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80},
    upper_twist={86,87,88},lower_twist={81,82,83,84,85}},
}
-- Parent-derived hand descendants include the named weapon attachments on both hands.
for _,arm in ipairs(arms) do arm.children=skeletons.descendants(skeletons.layouts[90],arm.wrist) end
avr.on('frame',function() avr.toggle('IK: controller arms',false) end)
avr.on('camera_post',function()
  if not avr.value('IK: controller arms') or not avr.writes() or not avr.stereo.running() or not avr.stereo.grip_transform then return end
  if not avr.player() or avr.bone_count()~=90 then return end
  for _,arm in ipairs(arms) do
    local target=avr.stereo.grip_transform(arm.hand)
    local shoulder,elbow,wrist=avr.bone(arm.shoulder),avr.bone(arm.elbow),avr.bone(arm.wrist)
    if target and shoulder and elbow and wrist then
      local upper=m.sub(elbow.pos,shoulder.pos)
      local lower=m.sub(wrist.pos,elbow.pos)
      local solution=g.solve_arm(shoulder.pos,target.pos,m.sub(elbow.pos,shoulder.pos),m.len(upper),m.len(lower))
      if solution then
        local rotate_upper=g.segment_rotation(upper,m.sub(solution.elbow,shoulder.pos))
        local rotate_lower=g.segment_rotation(lower,m.sub(solution.wrist,solution.elbow))
        -- Capture all matrices before writing, so child transforms use one source pose.
        local pending={}
        local function put(i,old,new,rotation)
          local bone=avr.bone(i)
          if bone then pending[i]=g.transform_bone(bone,old,new,rotation) end
        end
        put(arm.shoulder,shoulder.pos,shoulder.pos,rotate_upper)
        put(arm.elbow,elbow.pos,solution.elbow,rotate_lower)
        put(arm.wrist,wrist.pos,solution.wrist,rotate_lower)
        for _,i in ipairs(arm.children) do put(i,wrist.pos,solution.wrist,rotate_lower) end
        for _,i in ipairs(arm.upper_twist) do put(i,shoulder.pos,shoulder.pos,rotate_upper) end
        for _,i in ipairs(arm.lower_twist) do put(i,elbow.pos,solution.elbow,rotate_lower) end
        for i,bone in pairs(pending) do avr.set_bone(i,bone) end
      end
    end
  end
end)
