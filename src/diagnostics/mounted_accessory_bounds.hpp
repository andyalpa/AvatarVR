#pragma once
#include "skin_arm_ik.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace mounted_accessory_bounds {
using skin_arm_ik::Vec;
// MeshSkinned graph record +20/+30: local/world spheres; +40: center and three half-axis vectors.
// Layout established from 4580B0, 1AC0A0 and 3216B30; no mesh pointer is part of this 80-byte span.
struct Bounds { float local[4],world[4],box[12]; };
static_assert(sizeof(Bounds)==80);
inline bool include(Bounds& out,const Bounds& native,Vec low,Vec high,float world_scale) {
  using namespace skin_arm_ik;
  float values[20]; std::memcpy(values,&native,sizeof(values));
  for(float value:values) if(!std::isfinite(value)) return false;
  if(!finite(low)||!finite(high)||low.x>high.x||low.y>high.y||low.z>high.z ||
     !std::isfinite(world_scale)||world_scale<1e-4f||world_scale>100.f ||
     native.local[3]<0.f||native.world[3]<0.f||native.world[3]>8.f) return false;
  const Vec center{native.world[0],native.world[1],native.world[2]};
  // Keep the complete native mesh, plus a skin margin around the relocated arm's bone centers.
  constexpr float margin=0.15f;
  float lo[3]={low.x-margin,low.y-margin,low.z-margin};
  float hi[3]={high.x+margin,high.y+margin,high.z+margin};
  const float c[3]={center.x,center.y,center.z};
  for(int k=0;k<3;++k) {
    const float extent=std::fabs(native.box[3+k])+std::fabs(native.box[6+k])+std::fabs(native.box[9+k]);
    lo[k]=std::min(lo[k],std::min(c[k]-native.world[3],native.box[k]-extent));
    hi[k]=std::max(hi[k],std::max(c[k]+native.world[3],native.box[k]+extent));
  }
  float radius=native.world[3];
  for(int i=0;i<8;++i) radius=std::max(radius,length(sub(
    {(i&1)?hi[0]:lo[0],(i&2)?hi[1]:lo[1],(i&4)?hi[2]:lo[2]},center)));
  if(!std::isfinite(radius)||radius>8.f) return false; // unrelated/invalid coordinate space: no write
  Bounds next=native;
  next.world[3]=radius; next.local[3]=std::max(native.local[3],radius/world_scale);
  for(int k=0;k<12;++k) next.box[k]=0.f;
  for(int k=0;k<3;++k) {next.box[k]=(lo[k]+hi[k])*0.5f; next.box[3+k*4]=(hi[k]-lo[k])*0.5f;}
  out=next;
  return true;
}
} // namespace mounted_accessory_bounds
