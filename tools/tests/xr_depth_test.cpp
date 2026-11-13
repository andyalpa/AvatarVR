// Unit test for src/vr/xr_depth_math.hpp (per-eye depth submission, 2026-11-12).
#include "../../src/vr/xr_depth_math.hpp"

#include <cmath>
#include <cstdio>

namespace {
int g_failures = 0;
void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}
}  // namespace

int main() {
  using namespace xr_depth_math;
  // Whole-texture extent falls back to the texture size.
  Rect r = source_rect(0, 0, 0, 0, 1936, 2059);
  check(r.x == 0 && r.y == 0 && r.w == 1936 && r.h == 2059, "zero extent = whole texture");
  // A sub-rectangle is kept; one that leaves the texture is clamped.
  r = source_rect(10, 20, 637, 452, 1000, 800);
  check(r.x == 10 && r.y == 20 && r.w == 637 && r.h == 452, "sub-rectangle kept");
  r = source_rect(900, 700, 637, 452, 1000, 800);
  check(r.x == 900 && r.y == 700 && r.w == 100 && r.h == 100, "sub-rectangle clamped");
  r = source_rect(5000, 0, 10, 10, 1000, 800);
  check(r.x == 0 && r.w == 10, "out-of-range origin reset");
  check(source_rect(0, 0, 5, 5, 0, 0).w == 0, "empty texture gives an empty rect");

  // Output pixel centres map to the nearest source texel and stay inside the rect (2904 out from 1936 in = 1.5x).
  Rect s = source_rect(0, 0, 1936, 2059, 1936, 2059);
  uint32_t x = 0, y = 0;
  source_texel(0.5f, 0.5f, 2904, 3088, s, &x, &y);
  check(x == 0 && y == 0, "first pixel -> first texel");
  source_texel(2903.5f, 3087.5f, 2904, 3088, s, &x, &y);
  check(x == 1935 && y == 2058, "last pixel -> last texel");
  source_texel(1452.5f, 1544.5f, 2904, 3088, s, &x, &y);
  check(x == 968 && y == 1029, "centre pixel -> centre texel");
  Rect off = source_rect(10, 20, 100, 50, 1000, 800);
  source_texel(199.5f, 99.5f, 200, 100, off, &x, &y);
  check(x == 109 && y == 69, "offset rect: last pixel stays inside");

  // Reverse-Z infinite: minDepth 0 is at infinity, maxDepth 1 at the near plane.
  const DepthInfo d = reverse_z_info();
  check(d.min_depth == 0.f && d.max_depth == 1.f, "depth range 0..1");
  check(std::isinf(d.near_z) && d.near_z > 0.f, "nearZ = +infinity (minDepth)");
  check(d.far_z == 0.01f, "farZ = near plane (maxDepth)");
  check(d.near_z > d.far_z, "inverted range");

  check(fresh(1000, 950, 100), "50 ms old is fresh");
  check(!fresh(1000, 850, 100), "150 ms old is stale");
  check(!fresh(1000, 0, 100), "never captured is stale");
  check(!fresh(1000, 1200, 100), "future capture rejected");

  if (g_failures == 0) std::printf("xr_depth_test: all checks passed\n");
  return g_failures == 0 ? 0 : 1;
}
