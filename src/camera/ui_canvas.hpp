#pragma once

#include <algorithm>
#include <cmath>

namespace ui_canvas {
struct Fit { float x, y, scale; };

// Fit a logical rectangle already embedded in a larger raster canvas. In the
// window-sized UI / eye-sized projection path its Y origin is raster_h-ui_h,
// not zero. Subtract that origin before centring the actual UI content.
inline Fit place_rect(float left, float top, float source_w, float source_h,
                      float target_w, float target_h, float centre_x, float centre_y, float size) {
  const float scale = size * std::min(target_w/source_w, target_h/source_h);
  return {centre_x - scale*(left+source_w*.5f),
          centre_y - scale*(top+source_h*.5f), scale};
}

// Map an entire source canvas into a destination without stretching or clipping.
inline Fit fit(float source_w, float source_h, float target_w, float target_h) {
  const float scale = std::min(target_w / source_w, target_h / source_h);
  return {(target_w - source_w * scale) * 0.5f,
          (target_h - source_h * scale) * 0.5f, scale};
}
constexpr unsigned panel_width = 1920;
constexpr unsigned panel_height = 1080;
}  // namespace ui_canvas
