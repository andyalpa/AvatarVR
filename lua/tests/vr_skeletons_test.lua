package.path='lua/lib/?.lua;'..package.path
local s=require('vr_skeletons')
for _,count in ipairs({90,192}) do
  local layout=s.layouts[count]
  for index=0,count-1 do
    local p=layout.parents[index]
    assert(p and p>=-1 and p<count and p~=index)
    local seen={}
    while p>=0 do assert(not seen[p],'parent cycle');seen[p]=true;p=layout.parents[p] end
  end
  for _,side in ipairs({'L','R'}) do
    local upper,lower,hand=layout.names[side..'_upperArm'],layout.names[side..'_lowerArm'],layout.names[side..'_hand']
    assert(layout.parents[lower]==upper and layout.parents[hand]==lower)
    local attachment=layout.names[side..'_weaponAttachmentHand']
    assert(layout.parents[attachment]==hand)
    local found={};for _,i in ipairs(s.descendants(layout,hand)) do found[i]=true end
    assert(found[attachment] and not found[upper] and not found[layout.names.C_weaponAttachmentWorld])
  end
end
print('named skeleton parent chains, attachment ancestry, bounds and cycle checks passed')
