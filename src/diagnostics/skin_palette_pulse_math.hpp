#pragma once
#include <cmath>
#include <cstring>
#include <initializer_list>
namespace skin_palette_pulse {
constexpr unsigned count=211;
inline float offset(unsigned age_ms) {
  if(age_ms>=3000 && age_ms<6000) return 0.25f;
  if(age_ms>=9000 && age_ms<12000) return -0.25f;
  return 0.f;
}
inline bool copy_right(float (&out)[count][16],const float (&source)[count][16],float delta) {
  if(&out==&source || !std::isfinite(delta) || std::fabs(delta)>0.25f) return false;
  for(unsigned bone:{113u,114u,115u})
    for(float value:source[bone]) if(!std::isfinite(value)) return false;
  std::memcpy(out,source,sizeof(out));
  for(unsigned bone:{113u,114u,115u}) out[bone][13]+=delta;
  return true;
}
}
