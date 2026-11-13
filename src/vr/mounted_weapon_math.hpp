#pragma once
#include "../diagnostics/skin_arm_ik.hpp"
#include <cmath>
#include <cstring>
#include <cstdint>

// Mounted C1: the native source socket is independent of the selected VR hand.
namespace mounted_weapon_math {
struct Identity {
  const void* owner=nullptr; const void* partner=nullptr; const void* candidate=nullptr;
  std::uint64_t id[2]{};
};
inline bool same_identity(const Identity& a,const Identity& b) {
  return a.owner==b.owner && a.partner==b.partner && a.candidate==b.candidate && a.id[0]==b.id[0] && a.id[1]==b.id[1];
}
inline bool fresh(std::uint64_t now,std::uint64_t stamp) {return stamp && now>=stamp && now-stamp<=250;}
inline bool rigid(const float* m) {
  using namespace skin_arm_ik;
  for(int i=0;i<16;++i) if(!std::isfinite(m[i])) return false;
  for(int i=0;i<3;++i) {
    if(std::fabs(length(row(m,i))-1.f)>.02f) return false;
    for(int j=i+1;j<3;++j) if(std::fabs(dot(row(m,i),row(m,j)))>.02f) return false;
  }
  return dot(cross(row(m,0),row(m,1)),row(m,2))>.97f;
}
inline bool same_frame(const float* a,const float* b,float metres=.03f) {
  using namespace skin_arm_ik;
  if(!rigid(a)||!rigid(b) || length(sub(position(a),position(b)))>metres) return false;
  float trace=0.f; for(int i=0;i<3;++i) trace+=dot(row(a,i),row(b,i));
  return trace>2.9923894f; // relative rotation under five degrees
}
inline bool shot_ready(bool verified_hold,bool measured_aim,bool committed,std::uint64_t now,
                       std::uint64_t bound_at,std::uint64_t committed_at) {
  // A TP attachment proves placement ownership, not the native ray's barrel axes.
  return verified_hold && measured_aim && committed && fresh(now,bound_at) && fresh(now,committed_at);
}
// A palette may be built from the preceding successful placement while the
// current query already published a newer controller sample. Identity belongs
// to the enclosing weapon proof. Admitting an already-placed transient mesh
// additionally requires the same resource/identity and a fresh binding/commit.
struct PlacedFrames {
  static constexpr unsigned kSlots=8;  // two entities a tick
  float target[kSlots][16]{},entity[kSlots][16]{};
  std::uint64_t stamp[kSlots]{};
  unsigned next=0;
  bool entity_offset(const float* native,std::uint64_t now,float* out) const;
  void push(const float* frame,std::uint64_t now,const float* placed=nullptr) {
    if(!placed) placed=frame;
    if(!rigid(frame) || !rigid(placed) || !now) return;
    if(next && !std::memcmp(target[(next-1)%kSlots],frame,64) && !std::memcmp(entity[(next-1)%kSlots],placed,64)) {stamp[(next-1)%kSlots]=now; return;}
    const unsigned slot=next++%kSlots; std::memcpy(target[slot],frame,64); std::memcpy(entity[slot],placed,64); stamp[slot]=now;
  }
  bool matches(const float* native,const float* part_offset,bool root_space,std::uint64_t now) const {
    for(unsigned i=0;i<kSlots;++i) if(fresh(now,stamp[i])) {
      if(same_frame(native,entity[i])) return true;
      float expected[16];
      if(root_space) std::memcpy(expected,target[i],64);
      else skin_arm_ik::multiply_affine(expected,part_offset,target[i]);
      if(same_frame(native,expected)) return true;
    }
    return false;
  }
};
// Anatomical frame, independent of FP/TP bone-axis conventions. Row 0 is the
// palm normal (axial); rows 1/2 are pinky->index and wrist->middle (polar).
inline bool palm(const float* wrist,skin_arm_ik::Vec middle,skin_arm_ik::Vec index,
                 skin_arm_ik::Vec pinky,float* out) {
  using namespace skin_arm_ik;
  Vec f=sub(middle,position(wrist)),s=sub(index,pinky);
  const float fl=length(f); if(!(fl>1e-5f)) return false;
  f=scale(f,1.f/fl); s=sub(s,scale(f,dot(s,f)));
  const float sl=length(s); if(!(sl>1e-5f)) return false;
  s=scale(s,1.f/sl); const Vec n=cross(s,f),rows[3]={n,s,f};
  std::memcpy(out,wrist,64);
  for(int r=0;r<3;++r) {out[4*r]=rows[r].x; out[4*r+1]=rows[r].y; out[4*r+2]=rows[r].z; out[4*r+3]=0.f;}
  out[15]=1.f; return rigid(out);
}
// Reflect a palm hold across weapon X. Reflect its axial normal too, keeping
// a proper rotation; this never mirrors mesh vertices or native bone matrices.
inline void mirror_palm_hold(const float* in,float* out) {
  float next[16]; std::memcpy(next,in,64);
  for(int r=0;r<3;++r) for(int c=0;c<3;++c)
    next[4*r+c]=in[4*r+c]*(r==0?-1.f:1.f)*(c==0?-1.f:1.f);
  next[12]=-in[12]; std::memcpy(out,next,64);
}
inline bool relative(const float* native,const float* source,float* out) {
  if(!rigid(native)||!rigid(source)) return false;
  // Subtract the world origins before multiplication, avoiding cancellation of large world coordinates.
  float rotation[16],local[16],inverse[16]; std::memcpy(rotation,source,64); std::memcpy(local,native,64);
  for(int k=0;k<3;++k) {rotation[12+k]=0.f; local[12+k]-=source[12+k];}
  if(!skin_arm_ik::inverse_affine(inverse,rotation)) return false;
  skin_arm_ik::multiply_affine(out,local,inverse); return rigid(out);
}
inline bool PlacedFrames::entity_offset(const float* native,std::uint64_t now,float* out) const {
  // Match actual successful entity writes only, not a predicted palette root.
  // Recover from that write's own target, never from a newer animated TP socket.
  for(unsigned n=0;n<kSlots && n<next;++n) {
    const unsigned i=(next-1-n)%kSlots;
    if(fresh(now,stamp[i]) && same_frame(native,entity[i]))
      return relative(entity[i],target[i],out);
  }
  return false;
}
// [LOG 2026-10-30] Native recoil/aim can move the attachment past same_frame between the TP pose
// snapshot and the entity query, and flight moves it ~18 cm per tick. The already-placed object keeps
// its ownership inside the in-hand gate; holstered guns sit beyond it (same 0.9 m rule as on foot).
inline bool bound_carry(bool have_placed,bool same_object,const float* native,const float* socket) {
  using namespace skin_arm_ik;
  if(!have_placed || !same_object) return false;
  for(int k=12;k<15;++k) if(!std::isfinite(native[k]) || !std::isfinite(socket[k])) return false;
  return length(sub(position(native),position(socket)))<=.9f;
}
// Carry a committed world frame with the rider, so an older commit does not trail by speed x age.
inline bool carry(float* frame,const float* body_then,const float* body_now) {
  float shift[3],sum=0.f;
  for(int k=0;k<3;++k) {shift[k]=body_now[k]-body_then[k]; sum+=shift[k]*shift[k];}
  if(!std::isfinite(sum) || sum>25.f) return false; // teleport/reset, not flight within the 250 ms freshness
  for(int k=0;k<3;++k) frame[12+k]+=shift[k];
  return true;
}
// Keep a committed frame's relation to the free palm it was built from, on the free palm of this draw.
// The empty hand follows the live solve; a commit is older by up to a tick plus the rider's animation.
inline bool follow_palm(float* frame,const float* palm_then,const float* palm_now) {
  float offset[16],next[16];
  if(!rigid(palm_now) || !relative(frame,palm_then,offset)) return false;
  skin_arm_ik::multiply_affine(next,offset,palm_now);
  if(!rigid(next)) return false;
  std::memcpy(frame,next,64); return true;
}
// One palette bone. Bones may be scaled (a hidden firing part has zero scale): only the root and
// target must be rigid, so one such bone cannot send the whole gun back to the native pose.
inline bool map_part(const float* bone,const float* root,const float* target,float* out) {
  for(int i=0;i<16;++i) if(!std::isfinite(bone[i])) return false;
  if(!rigid(root) || !rigid(target)) return false;
  float rotation[16],local[16],inverse[16],offset[16];
  std::memcpy(rotation,root,64); std::memcpy(local,bone,64);
  for(int k=0;k<3;++k) {rotation[12+k]=0.f; local[12+k]-=root[12+k];}
  if(!skin_arm_ik::inverse_affine(inverse,rotation)) return false;
  skin_arm_ik::multiply_affine(offset,local,inverse);
  skin_arm_ik::multiply_affine(out,offset,target);
  for(int i=0;i<16;++i) if(!std::isfinite(out[i])) return false;
  return true;
}
inline bool map(const float* native,const float* source,const float* target,float* out) {
  if(!rigid(target)) return false;
  float offset[16],next[16];
  if(!relative(native,source,offset)) return false;
  skin_arm_ik::multiply_affine(next,offset,target);
  if(!rigid(next) || skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(next),skin_arm_ik::position(native)))>3.f) return false;
  std::memcpy(out,next,64); return true;
}
}
