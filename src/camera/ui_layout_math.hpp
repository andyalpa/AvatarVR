#pragma once
#include <cmath>

namespace ui_layout {
inline bool dimension(float v) { return std::isfinite(v) && v >= 16.f && v <= 16384.f; }
inline bool close(float a, float b) { return std::fabs(a - b) <= 2.f; }
// Rectangular canvas conversions must be reciprocal on both axes. The scene
// perspective is deliberately outside this operation.
inline void pixel_to_viewport(float* m, float w, float h) { m[0] = 1.f / w; m[5] = 1.f / h; }
inline void viewport_to_pixel(float* m, float w, float h) { m[0] = w; m[5] = h; }
}
