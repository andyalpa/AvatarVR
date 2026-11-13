// Unit test for src/vr/frame_gen_math.hpp (in-mod frame extrapolation, 2026-11-12).
#include "../../src/vr/frame_gen_math.hpp"

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
bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }
}  // namespace

int main() {
  using namespace frame_gen_math;
  // k: half a game frame (90 Hz headset, 45 fps game), clamp, invalid intervals.
  check(near(extrapolation_k(1011111111, 1000000000, 22222222), 0.5f, 1e-3f), "11.1 ms of a 22.2 ms frame = 0.5");
  check(near(extrapolation_k(1100000000, 1000000000, 22222222), 1.5f), "far ahead clamps to 1.5");
  check(extrapolation_k(1000000000, 1000000000, 22222222) == 0.f, "same display time = 0");
  check(extrapolation_k(1010000000, 1000000000, 2000000) == 0.f, "2 ms interval rejected");
  check(extrapolation_k(1010000000, 1000000000, 200000000) == 0.f, "200 ms interval (pause) rejected");

  // Gather: a pixel moving right by 0.01 UV per frame has mv_uv = -0.01 (current -> previous); half a frame later the
  // content at u = 0.5 came from u = 0.495. NDC -> UV with the live mvecScale (0.5, -0.5).
  float su = 0.f, sv = 0.f;
  gather_uv(0.5f, 0.5f, -0.02f, 0.f, 0.5f, -0.5f, 0.5f, &su, &sv);
  check(near(su, 0.495f) && near(sv, 0.5f), "rightward motion gathers from the left");
  gather_uv(0.5f, 0.5f, 0.f, 0.04f, 0.5f, -0.5f, 1.f, &su, &sv);
  check(near(su, 0.5f) && near(sv, 0.48f), "NDC y delta flips to UV");

  // Pose: linear position, slerp orientation by k.
  const Pose prev = {0.f, 0.f, 0.f, 1.f, 0.f, 1.6f, 0.f};
  const float a = 0.1f;  // 0.1 rad yaw per frame about +y
  const Pose last = {0.f, std::sin(a / 2), 0.f, std::cos(a / 2), 0.02f, 1.6f, -0.01f};
  const Pose half = extrapolate(prev, last, 0.5f);
  check(near(half.px, 0.03f) && near(half.py, 1.6f) && near(half.pz, -0.015f), "position extrapolated by k");
  check(near(half.qy, std::sin(0.15f / 2)) && near(half.qw, std::cos(0.15f / 2)) && near(half.qx, 0.f) && near(half.qz, 0.f),
        "yaw extrapolated by k (0.1 + 0.05 rad)");
  const Pose same = extrapolate(last, last, 1.f);
  check(near(same.qy, last.qy) && near(same.qw, last.qw) && near(same.px, last.px), "no motion = unchanged");
  const Pose flipped_prev = {-prev.qx, -prev.qy, -prev.qz, -prev.qw, prev.px, prev.py, prev.pz};
  const Pose f = extrapolate(flipped_prev, last, 0.5f);
  check(near(std::fabs(f.qy), std::sin(0.15f / 2)) && near(std::fabs(f.qw), std::cos(0.15f / 2)), "sign-flipped quaternion takes the short arc");

  // Banks: publish only complete banks.
  Banks b;
  b.mark(0);
  check(!b.publish() && b.ready == -1 && b.write == 1, "half-captured bank is not published");
  b.mark(0);
  b.mark(1);
  check(b.publish() && b.ready == 1 && b.write == 2, "complete bank published");
  b.mark(1);
  b.mark(0);
  check(b.publish() && b.ready == 2 && b.write == 0 && b.captured[0] == 0, "ring wraps and clears");

  if (g_failures == 0) std::printf("frame_gen_test: all checks passed\n");
  return g_failures == 0 ? 0 : 1;
}
