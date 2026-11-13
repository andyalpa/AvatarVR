#pragma once
#include <cstdint>
#include <limits>
#include "hand_gesture_policy.hpp"

namespace hand_gestures {
// Read-only candidates, not write authority. Independent reads are essential:
// optional/missing tool state must not hide sequence or camera observations.
// Provenance/limits: INPUT_OWNERSHIP.md, Empty-hand gestures: context gate.
struct ContextAudit {
  const void* registry=nullptr;
  const void* tool_component=nullptr;
  const void* camera_mode=nullptr;
  unsigned registry_count=std::numeric_limits<unsigned>::max();
  int tool=-1,mode=-1;
  unsigned read_mask=0; // registry ptr/count, tool ptr/value, mode ptr/id
  bool readable=false;
};
template<class Read> ContextAudit sample_context(const void* body,const void* holder,Read read) {
  ContextAudit out;
  const bool registry_ptr=read(body,0x1F0,out.registry);
  if(registry_ptr) out.read_mask|=1;
  bool registry_count=false;
  if(registry_ptr && out.registry) {
    registry_count=read(out.registry,0x20,out.registry_count);
    if(registry_count) out.read_mask|=2;
  } else if(registry_ptr) out.registry_count=0; // absent registry, explicitly tagged by pointer/mask

  const bool tool_ptr=read(body,0x258,out.tool_component);
  if(tool_ptr) out.read_mask|=4;
  bool tool_value=false;
  if(tool_ptr && out.tool_component) {
    unsigned char value=0;
    tool_value=read(out.tool_component,0x10,value);
    if(tool_value) {out.read_mask|=8; out.tool=value;}
  } else if(tool_ptr) out.tool=0; // native node returns false for absent component

  const bool mode_ptr=read(holder,0x40,out.camera_mode);
  if(mode_ptr) out.read_mask|=16;
  bool mode_id=false;
  if(mode_ptr && out.camera_mode) {
    mode_id=read(out.camera_mode,0xC,out.mode);
    if(mode_id) out.read_mask|=32;
  }
  out.readable=registry_ptr && (!out.registry || (registry_count && out.registry_count<1024)) &&
    tool_ptr && (!out.tool_component || (tool_value && out.tool<=1)) && mode_ptr && mode_id;
  return out;
}

// Native player paths, from GetTool::Execute, Is[Un]Holstering and the native
// camera-state classifier. Byte guards live in hand_gestures.cpp. These are
// experimental STATIC contracts; a readable snapshot is not headset acceptance.
struct PlayerOwnership {
  const void* equipment=nullptr; const void* held=nullptr; const void* definition=nullptr;
  const void* animation=nullptr; const void* camera_control=nullptr; const void* transition=nullptr;
  int active=-1,archetype=-1,transition_state=-1,camera_override=-1;
  bool equipment_readable=false,camera_readable=false;
};
template<class Read> PlayerOwnership sample_player_ownership(const void* body,Read read) {
  PlayerOwnership out;
  if(read(body,0x250,out.equipment) && out.equipment) {
    unsigned char active=0;
    const bool held=read(out.equipment,0x3E40,out.held);
    const bool flag=read(out.equipment,0x3F64,active);
    if(flag) out.active=active;
    const bool definition=read(out.equipment,0x3E58,out.definition);
    bool type=flag && active==0;
    if(flag && active==0) out.archetype=0;
    if(flag && active==1 && definition && out.definition) type=read(out.definition,0x3A8,out.archetype);
    const bool transition=read(out.equipment,0x3E88,out.transition);
    bool state=false;
    if(transition && out.transition) state=read(out.transition,0x98,out.transition_state);
    // A null transition object is only accepted for the explicitly empty case.
    if(transition && !out.transition && flag && active==0 && held && !out.held) {
      state=true; out.transition_state=0;
    }
    out.equipment_readable=held && flag && active<=1 && type && state && out.archetype>=0;
  }
  unsigned char native_camera=0;
  if(read(body,0x1A8,out.animation) && out.animation &&
     read(out.animation,0x538,out.camera_control) && out.camera_control &&
     read(out.camera_control,0x80,native_camera) && native_camera<=1) {
    out.camera_override=native_camera; out.camera_readable=true;
  }
  return out;
}
struct Interactions {
  int dominant=1,support=-1;
  bool swap=false,support_held=false,support_only=false;
  bool harness[2]{},busy[2]{};
  bool stable=false,named_weapon=false;
};
inline Context resolve_context(Context c,const ContextAudit& a,const PlayerOwnership& p,const Interactions& i) {
  c.owner[0]=c.owner[1]=Owner::Unknown;
  c.verified=a.readable && p.equipment_readable && p.camera_readable;
  // Only observed on-foot/riding camera modes. Suppress ALL native sequence
  // bindings and native-camera ownership, not just an assumed cinematic id.
  c.cinematic=!c.verified || (a.mode!=4 && a.mode!=7) || p.camera_override!=0 || a.registry_count!=0 ||
    (c.mounted && a.mode!=7) || (!c.mounted && a.mode!=4);
  if(c.cinematic) return c;
  if(i.swap) {c.owner[0]=c.owner[1]=Owner::Swap; return c;}
  if(!i.stable || p.transition_state==1 || p.transition_state==2) {
    c.owner[0]=c.owner[1]=Owner::Item; return c;
  }
  if(p.active==0 && !p.held) c.owner[0]=c.owner[1]=Owner::Empty;
  else if(p.active==1 && p.held && p.archetype!=14 && i.named_weapon && i.dominant>=0 && i.dominant<2) {
    c.weapon=true; c.owner[0]=c.owner[1]=Owner::Empty;
    c.owner[i.dominant]=Owner::Weapon;
    if(i.support_held && i.support>=0 && i.support<2 && i.support!=i.dominant) {
      c.owner[i.support]=i.support_only?Owner::Weapon:Owner::Support;
      if(i.support_only) c.owner[i.dominant]=Owner::Empty;
    }
  } else c.owner[0]=c.owner[1]=Owner::Item; // unknown tools/items retain both native hands
  for(int h=0;h<2;++h) {
    if(i.harness[h]) c.owner[h]=Owner::Harness;
    else if(i.busy[h]) c.owner[h]=Owner::Item;
  }
  return c;
}
} // namespace hand_gestures
