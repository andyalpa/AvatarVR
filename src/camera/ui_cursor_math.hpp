#pragma once
#include <cmath>
namespace ui_cursor {
inline bool layout_point(const float* point, float eye_w,float eye_h,float logical_w,float logical_h,float* out) {
  if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
      !std::isfinite(eye_w) || !std::isfinite(eye_h) || !std::isfinite(logical_w) || !std::isfinite(logical_h) ||
      eye_w<64.f || eye_h<64.f || logical_w<64.f || logical_h<64.f ||
      eye_w>16384.f || eye_h>16384.f || logical_w>16384.f || logical_h>16384.f) return false;
  // Native virtual cursor is normalized to the full eye image when displayed.
  // Hit polygons use logical pixels with the same lower-left Y origin.
  // Preserve edge/out-of-range coordinates: no clipping or extra Y inversion.
  out[0]=point[0]*(logical_w/eye_w);
  out[1]=point[1]*(logical_h/eye_h);
  return std::isfinite(out[0]) && std::isfinite(out[1]);
}
}
