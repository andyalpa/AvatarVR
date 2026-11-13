#include "../../src/vr/lens_projection.hpp"
#include <cstdio>
#include <limits>
#include <initializer_list>

static bool close(float a, float b) { return std::fabs(a-b) < 0.00001f; }
int main() {
  constexpr float deg = 0.017453292519943295f;
  const float eyes[2][4] = {{-52.5f*deg, 38.4f*deg, 42.4f*deg, -53.5f*deg},
                          {-38.4f*deg, 52.5f*deg, 42.4f*deg, -53.5f*deg}};
  for (const auto& f : eyes) {
    lens_projection::Projection p = {};
    if (!lens_projection::from_fov(f, &p)) return 1;
    const float th = std::tan(p.fov_rad * 0.5f), tv = th / p.aspect;
    if (!close((-1-p.m8)*th, p.tan4[0]) || !close((1-p.m8)*th, p.tan4[1]) ||
        !close((1-p.m9)*tv, p.tan4[2]) || !close((-1-p.m9)*tv, p.tan4[3])) return 2;
    for (int height : {1200, 1840, 2160, 2400, 3200, 4096}) {
      const int width = static_cast<int>(height * (2908.f/3088.f)) & ~7;
      // Scale and rounding change pixel density, not the ray at a given NDC.
      for (float fraction : {0.f, 0.25f, 0.5f, 0.75f, 1.f}) {
        const float ndc_x = 2.f * (fraction*width / width) - 1.f;
        const float ray_x = (ndc_x-p.m8)*th;
        if (!close(ray_x, p.tan4[0] + fraction*(p.tan4[1]-p.tan4[0]))) return 3;
      }
    }
  }
  lens_projection::Projection p = {};
  const float invalid[4] = {-0.7f, 0.7f, std::numeric_limits<float>::quiet_NaN(), -0.7f};
  if (lens_projection::from_fov(invalid, &p)) return 4;
  std::puts("Lens rays match both asymmetric eyes at six resolutions; invalid FOV rejected.");
  return 0;
}
