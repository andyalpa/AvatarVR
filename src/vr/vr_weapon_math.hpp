#pragma once
// Pure math for vr_weapon (unit-tested in tools/tests/vr_weapon_test.cpp).
#include "../diagnostics/skin_arm_ik.hpp"
#include <cmath>
#include <cstring>

namespace vr_weapon_math {
using skin_arm_ik::Vec;

// One weapon object's place relative to the hand socket. It follows the game only while a new weapon comes up (tau 0.1 s)
// or the player stands still (tau 0.5 s), and is frozen otherwise, so no locomotion animation or tick skew reaches the
// gun: the hand was already built this way, see HandFrame. An offset this close to the socket is the socket itself.
struct BoundOffset {
  float rel[16]{}; bool have=false,on_socket=false;
  const void* weapon=nullptr; unsigned long long window=0,last=0;
};
inline void relative_to(const float* m,const float* frame,float* out) {
  float rotation[16],local[16],inverse[16];
  std::memcpy(rotation,frame,64); std::memcpy(local,m,64);
  for(int k=0;k<3;++k) {rotation[12+k]=0.f; local[12+k]-=frame[12+k];}  // large world origins cancel before any product
  if(!skin_arm_ik::inverse_affine(inverse,rotation)) {std::memcpy(out,local,64); return;}
  skin_arm_ik::multiply_affine(out,local,inverse);
}
template<class Blend>
inline void update_bound(BoundOffset& b,const void* weapon,const float* m,const float* socket,bool still,unsigned long long now,Blend blend) {
  float live[16]; relative_to(m,socket,live);
  for(float v:live) if(!std::isfinite(v)) return;
  if(!b.have || b.weapon!=weapon) {b.weapon=weapon; b.window=now+1000; std::memcpy(b.rel,live,64); b.have=true; b.last=now;}
  else {
    const float dt=static_cast<float>(now-b.last)*0.001f;
    const float tau=now<b.window?0.1f:still?0.5f:0.f;
    if(tau>0.f && dt>0.f) {float mixed[16]; blend(b.rel,live,1.f-std::exp(-dt/tau),mixed); bool fine=true; for(float v:mixed) fine=fine&&std::isfinite(v); if(fine) std::memcpy(b.rel,mixed,64);}
    b.last=now;
  }
  const float d2=b.rel[12]*b.rel[12]+b.rel[13]*b.rel[13]+b.rel[14]*b.rel[14], trace=b.rel[0]+b.rel[5]+b.rel[10];
  b.on_socket=d2<0.02f*0.02f && trace>2.995f;
}

// Learn the native on-foot muzzle/barrel in the weapon's actual root frame,
// rather than guessing an asset axis or borrowing a controller's forward row.
inline bool local_ray(const float* start,const float* target,const float* root,Vec& origin,Vec& direction) {
  using namespace skin_arm_ik;
  const Vec s{start[0],start[1],start[2]},t{target[0],target[1],target[2]};
  if(!finite(s)||!finite(t)) return false;
  const Vec offset=sub(s,position(root)),ray=sub(t,s); const float len=length(ray);
  if(!(len>1e-4f)||length(offset)>1.5f) return false;
  origin={dot(offset,row(root,0)),dot(offset,row(root,1)),dot(offset,row(root,2))};
  direction={dot(ray,row(root,0))/len,dot(ray,row(root,1))/len,dot(ray,row(root,2))/len};
  const float n=length(direction);
  if(!finite(origin)||!finite(direction)||std::fabs(n-1.f)>=.02f) return false;
  direction=scale(direction,1.f/n); return true;
}
// The aim ray in the hand's frame, kept steady against the game's animations. Each sample is the game's aim at that moment, so recoil and
// reload swing it. Only a sample that has held still counts: within 3 degrees it refines the ray slowly (far targets only, near ones have
// parallax), and further away it replaces the ray only after 2 s, which reload poses do not hold. True when the ray was replaced.
// `settle` (just after an equip) refines 8x faster: a ray kept from an earlier equip can sit a degree or two off this equip's hold.
struct SteadyRay {
  skin_arm_ik::Vec origin{},forward{},try_origin{},try_forward{};
  unsigned long long try_since=0,try_last=0;
  bool locked=false;
};
inline bool steady_ray_sample(SteadyRay& ray,skin_arm_ik::Vec origin,skin_arm_ik::Vec forward,bool distant,unsigned long long now,
                              bool settle=false) {
  using namespace skin_arm_ik;
  if(!ray.try_last || now-ray.try_last>100 || dot(forward,ray.try_forward)<.99985f) {
    ray.try_origin=origin; ray.try_forward=forward; ray.try_since=ray.try_last=now; return false;
  }
  ray.try_last=now;
  const unsigned long long held=now-ray.try_since;
  if(ray.locked && dot(forward,ray.forward)>.99863f) {
    if(distant && held>=100) {
      const float g=settle?.16f:.02f;
      const Vec mixed=add(scale(ray.forward,1.f-g),scale(forward,g));
      ray.forward=scale(mixed,1.f/length(mixed)); ray.origin=add(scale(ray.origin,1.f-g),scale(origin,g));
    }
    return false;
  }
  if(ray.locked?!(distant && held>=2000):held<500) return false;
  ray.origin=origin; ray.forward=forward; ray.locked=true; return true;
}
inline bool calibrated_shot(const float* start,const float* target,const float* root,Vec origin,Vec direction,
                            float max_shift,float* out_start,float* out_target) {
  using namespace skin_arm_ik;
  const Vec s{start[0],start[1],start[2]},t{target[0],target[1],target[2]};
  const float range=length(sub(t,s));
  auto rotated=[&](Vec p){return Vec{p.x*root[0]+p.y*root[4]+p.z*root[8],p.x*root[1]+p.y*root[5]+p.z*root[9],p.x*root[2]+p.y*root[6]+p.z*root[10]};};
  const Vec muzzle=add(rotated(origin),position(root)),forward=rotated(direction); const float n=length(forward);
  if(!finite(s)||!finite(t)||!finite(muzzle)||!(range>1e-4f)||!std::isfinite(range)||!(n>1e-4f)||length(sub(muzzle,s))>max_shift) return false;
  const Vec aim=add(muzzle,scale(forward,range/n)); if(!finite(aim)) return false;
  out_start[0]=muzzle.x; out_start[1]=muzzle.y; out_start[2]=muzzle.z;
  out_target[0]=aim.x; out_target[1]=aim.y; out_target[2]=aim.z; return true;
}

// The game's shot is a start point S (its camera-relative muzzle) and an aim point T. Both are carried rigidly from the
// flat (animated) weapon socket to the IK weapon socket: S' = S * inv(flat) * ik, and T' = S' + the aim direction rotated
// the same way, at the same distance. Shots then leave the tracked gun and point where its barrel points.
// Row-vector affine matrices (translation in row 3). False when the inputs are degenerate or the shift exceeds max_shift.
inline bool map_shot(const float* start, const float* target, const float* flat, const float* ik, float max_shift,
                     float* start_out, float* target_out) {
  float inverse[16], delta[16];
  if (!skin_arm_ik::inverse_affine(inverse, flat)) return false;
  skin_arm_ik::multiply_affine(delta, inverse, ik);
  const Vec s{start[0], start[1], start[2]}, t{target[0], target[1], target[2]};
  // Point: p * delta (w = 1). Direction: d * delta rotation (w = 0).
  const Vec s2{s.x * delta[0] + s.y * delta[4] + s.z * delta[8] + delta[12],
               s.x * delta[1] + s.y * delta[5] + s.z * delta[9] + delta[13],
               s.x * delta[2] + s.y * delta[6] + s.z * delta[10] + delta[14]};
  const Vec d = skin_arm_ik::sub(t, s);
  const float len = skin_arm_ik::length(d);
  if (!(len > 1e-4f) || !skin_arm_ik::finite(s2)) return false;
  Vec d2{d.x * delta[0] + d.y * delta[4] + d.z * delta[8], d.x * delta[1] + d.y * delta[5] + d.z * delta[9],
         d.x * delta[2] + d.y * delta[6] + d.z * delta[10]};
  const float len2 = skin_arm_ik::length(d2);
  if (!(len2 > 1e-6f)) return false;
  d2 = skin_arm_ik::scale(d2, len / len2);  // keep the game's distance; drop any scale in the socket matrices
  if (skin_arm_ik::length(skin_arm_ik::sub(s2, s)) > max_shift) return false;
  const Vec t2 = skin_arm_ik::add(s2, d2);
  if (!skin_arm_ik::finite(t2)) return false;
  start_out[0] = s2.x; start_out[1] = s2.y; start_out[2] = s2.z;
  target_out[0] = t2.x; target_out[1] = t2.y; target_out[2] = t2.z;
  return true;
}
}  // namespace vr_weapon_math
