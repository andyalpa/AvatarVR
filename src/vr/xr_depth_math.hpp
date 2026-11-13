#pragma once
// Pure math for the optional per-eye depth submission (xr_depth.cpp, 2026-11-12). No D3D12 or OpenXR here, so
// tools/tests/xr_depth_test.cpp can check it on its own.
#include <cmath>
#include <cstdint>
#include <limits>

namespace xr_depth_math {

// The source rectangle inside the captured depth texture. A zero width or height (Streamline: "whole resource")
// falls back to the texture size; an extent that would leave the texture is clamped to it.
struct Rect {
  uint32_t x = 0, y = 0, w = 0, h = 0;
};
inline Rect source_rect(uint32_t ext_x, uint32_t ext_y, uint32_t ext_w, uint32_t ext_h, uint32_t tex_w, uint32_t tex_h) {
  Rect r;
  if (tex_w == 0 || tex_h == 0) return r;
  if (ext_w == 0 || ext_h == 0) {
    r.w = tex_w;
    r.h = tex_h;
    return r;
  }
  r.x = ext_x < tex_w ? ext_x : 0;
  r.y = ext_y < tex_h ? ext_y : 0;
  r.w = ext_w <= tex_w - r.x ? ext_w : tex_w - r.x;
  r.h = ext_h <= tex_h - r.y ? ext_h : tex_h - r.y;
  return r;
}

// Nearest source texel for output pixel (px, py) of a dst_w x dst_h image (pixel centres at +0.5, as SV_Position).
// Same formula as the pixel shader in xr_depth.cpp.
inline void source_texel(float px, float py, uint32_t dst_w, uint32_t dst_h, const Rect& src, uint32_t* sx, uint32_t* sy) {
  const float fx = px * static_cast<float>(src.w) / static_cast<float>(dst_w);
  const float fy = py * static_cast<float>(src.h) / static_cast<float>(dst_h);
  uint32_t ix = fx > 0.f ? static_cast<uint32_t>(fx) : 0u;
  uint32_t iy = fy > 0.f ? static_cast<uint32_t>(fy) : 0u;
  if (ix >= src.w) ix = src.w - 1;
  if (iy >= src.h) iy = src.h - 1;
  *sx = src.x + ix;
  *sy = src.y + iy;
}

// XrCompositionLayerDepthInfoKHR values for the game's reverse-Z infinite projection: depth 1 at the near plane,
// 0 at infinity. OpenXR: nearZ is the distance of minDepth, farZ the distance of maxDepth; nearZ > farZ marks an
// inverted range and nearZ may be +infinity. [INFERENCE] near plane 0.01 game units = metres
// (research/CAMERA_FINDINGS.md; mv_repair_shaders.hpp uses view z = 0.01 / d).
struct DepthInfo {
  float min_depth, max_depth, near_z, far_z;
};
inline DepthInfo reverse_z_info(float near_plane = 0.01f) {
  return {0.f, 1.f, std::numeric_limits<float>::infinity(), near_plane};
}

// A capture is used only while it is younger than max_age_ms (a pause or a load leaves an old one behind).
inline bool fresh(uint64_t now_ms, uint64_t captured_ms, uint64_t max_age_ms) {
  return captured_ms != 0 && now_ms >= captured_ms && now_ms - captured_ms <= max_age_ms;
}

}  // namespace xr_depth_math
