#pragma once

#include <cmath>

namespace lens_projection {

struct Projection {
  float aspect;
  float fov_rad;
  float m8, m9;
  float tan4[4];
};

// Optical geometry is independent of texture dimensions. Pixel aspect need not
// equal frustum aspect: the compositor maps the image through the same angles.
inline bool from_fov(const float* fov, Projection* out) {
  if (fov == nullptr || out == nullptr) return false;
  const float l = std::tan(fov[0]), r = std::tan(fov[1]);
  const float u = std::tan(fov[2]), d = std::tan(fov[3]);
  if (!std::isfinite(l) || !std::isfinite(r) || !std::isfinite(u) || !std::isfinite(d) ||
      l >= 0.f || r <= 0.f || u <= 0.f || d >= 0.f || r - l < 0.2f || u - d < 0.2f) return false;
  const float th = (r - l) * 0.5f, tv = (u - d) * 0.5f;
  out->aspect = th / tv;
  out->fov_rad = 2.f * std::atan(th);
  out->m8 = -(r + l) / (r - l);
  out->m9 = -(u + d) / (u - d);
  out->tan4[0] = l; out->tan4[1] = r;
  out->tan4[2] = u; out->tan4[3] = d;
  return out->aspect > 0.3f && out->aspect < 4.f && out->fov_rad < 3.1f;
}

}  // namespace lens_projection
