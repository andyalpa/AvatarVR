// vr_recoil_math: kick envelope, kick transform in the wrist frame, un-kicking the aim ray.
#include "../../src/vr/vr_recoil_math.hpp"
#include <cmath>
#include <cstdio>
using namespace vr_recoil_math;
namespace {
int failures = 0;
void check(bool ok, const char* what) { if (!ok) { std::printf("FAIL: %s\n", what); ++failures; } }
bool near(float a, float b, float e = 1e-4f) { return std::fabs(a - b) < e; }
void point(const float* p, const float* m, float* out) {
  for (int k = 0; k < 3; ++k) out[k] = p[0] * m[k] + p[1] * m[4 + k] + p[2] * m[8 + k] + m[12 + k];
}
}  // namespace
int main() {
  // Pulse: 0 at the shot, peak 1 at 1/omega, gone after a few tenths of a second.
  check(near(pulse(0.f, 28.f), 0.f), "pulse starts at 0");
  check(near(pulse(1.f / 28.f, 28.f), 1.f, 1e-3f), "pulse peaks at 1");
  check(pulse(0.3f, 28.f) < 0.02f, "pulse settles by 300 ms");
  check(pulse(-0.01f, 28.f) == 0.f, "no pulse before the shot");
  // Envelope stacks and caps.
  {
    Shot shots[kShots];
    shots[0] = Shot{1.0, 1.f, 1.f};
    shots[1] = Shot{1.0 + 1.0 / 28.0, -1.f, 1.f};
    const Envelope one = envelope(shots, 1, 1.0 + 1.0 / 28.0, 28.f, 10.f);
    check(near(one.pitch, 1.f, 1e-3f) && near(one.yaw, 1.f, 1e-3f), "single shot envelope");
    const Envelope two = envelope(shots, kShots, 1.0 + 2.0 / 28.0, 28.f, 10.f);
    check(two.pitch > 1.f, "full auto stacks");
    const Envelope capped = envelope(shots, kShots, 1.0 + 2.0 / 28.0, 28.f, 1.2f);
    check(near(capped.pitch, 1.2f) && std::fabs(capped.yaw) <= 1.2f, "stack capped");
    check(envelope(shots, kShots, 0.5, 28.f, 2.f).pitch == 0.f, "nothing before the shots");
  }
  // Kick: barrel along +Z, up +Y (and a mirrored frame); the barrel rises, the hand moves back, the pivot stays put.
  for (int flip = 0; flip < 2; ++flip) {
    const float fwd[3] = {0.f, 0.f, flip ? -1.f : 1.f}, up[3] = {0.f, 1.f, 0.f}, pivot[3] = {0.f, 0.f, 0.f};
    float w[16];
    check(kick_matrix(fwd, up, pivot, 0.1f, 0.f, 0.f, w), "kick builds");
    float f2[3]; dir_mul(fwd, w, f2);
    check(f2[1] > 0.09f, "barrel rises");
    const float angle = std::acos(f2[0] * fwd[0] + f2[1] * fwd[1] + f2[2] * fwd[2]);
    check(near(angle, 0.1f, 1e-3f), "pitch angle");
    float p[3]; point(pivot, w, p);
    check(near(p[0], 0.f) && near(p[1], 0.f) && near(p[2], 0.f), "pivot fixed without back");
    check(kick_matrix(fwd, up, pivot, 0.f, 0.f, 0.03f, w), "back builds");
    point(pivot, w, p);
    check(near(p[2], -0.03f * fwd[2]), "moves back along the barrel");
  }
  // Pivot elsewhere: the pivot point stays fixed under pure rotation.
  {
    const float fwd[3] = {1.f, 0.f, 0.f}, up[3] = {0.f, 0.f, 1.f}, pivot[3] = {0.05f, -0.02f, 0.1f};
    float w[16], p[3];
    check(kick_matrix(fwd, up, pivot, 0.2f, 0.05f, 0.f, w), "pivot kick builds");
    point(pivot, w, p);
    check(near(p[0], pivot[0]) && near(p[1], pivot[1]) && near(p[2], pivot[2]), "pivot stays");
  }
  // Degenerate: barrel along up.
  {
    const float fwd[3] = {0.f, 1.f, 0.f}, up[3] = {0.f, 1.f, 0.f}, pivot[3] = {0.f, 0.f, 0.f};
    float w[16];
    check(!kick_matrix(fwd, up, pivot, 0.1f, 0.f, 0.f, w) && near(w[0], 1.f) && near(w[5], 1.f) && near(w[12], 0.f), "degenerate = identity");
  }
  // Un-kick: a ray in the kicked wrist frame, carried by the kicked wrist, lands on the un-kicked ray in the world.
  {
    const float fwd[3] = {0.f, 0.f, 1.f}, up[3] = {0.f, 1.f, 0.f}, pivot[3] = {0.f, 0.f, 0.f};
    float w[16], wrist[16], kicked[16];
    kick_matrix(fwd, up, pivot, 0.12f, -0.03f, 0.04f, w);
    axis_angle(0.267f, 0.534f, 0.802f, 0.9f, wrist);  // some wrist orientation
    wrist[12] = 3.f; wrist[13] = -1.f; wrist[14] = 2.f;
    mul(kicked, w, wrist);
    const float o0[3] = {0.02f, 0.05f, 0.3f}, f0[3] = {0.f, 0.f, 1.f};
    float expect_o[3], expect_f[3]; point(o0, wrist, expect_o); dir_mul(f0, wrist, expect_f);
    float o[3] = {o0[0], o0[1], o0[2]}, f[3] = {f0[0], f0[1], f0[2]};
    unkick_ray(w, o, f);
    float got_o[3], got_f[3]; point(o, kicked, got_o); dir_mul(f, kicked, got_f);
    for (int k = 0; k < 3; ++k) {
      check(near(got_o[k], expect_o[k], 1e-4f), "unkicked origin");
      check(near(got_f[k], expect_f[k], 1e-4f), "unkicked direction");
    }
  }
  // Native scale and edges.
  check(near(native_scale(0.f, 2.f), 1.f) && near(native_scale(2.f, 2.f), 1.f) && near(native_scale(10.f, 2.f), 1.5f) &&
        near(native_scale(0.1f, 2.f), 0.3f), "native scale clamps");
  {
    Edge e;
    check(!e.step(0.f, 0.3f, 0.6f, 100, 70), "edge idle");
    check(e.step(0.9f, 0.3f, 0.6f, 110, 70), "edge fires");
    check(!e.step(0.9f, 0.3f, 0.6f, 120, 70), "edge holds while pressed");
    check(!e.step(0.1f, 0.3f, 0.6f, 130, 70) && !e.step(0.9f, 0.3f, 0.6f, 140, 70), "refractory");
    check(!e.step(0.1f, 0.3f, 0.6f, 150, 70) && e.step(0.9f, 0.3f, 0.6f, 200, 70), "fires again after refractory");
  }
  if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
  std::printf("vr_recoil_test: all passed\n");
  return 0;
}
