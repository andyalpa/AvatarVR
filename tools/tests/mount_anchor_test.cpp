#include "../../src/camera/mount_anchor.hpp"
#include "../../src/game/mount_state.hpp"
#include <cmath>
#include <cstdio>

using mount_anchor::Anchor;
using mount_anchor::Input;
using mount_state::Phase;

namespace {
constexpr float kDt = 1.f / 60.f;
float dist(const float* a, const float* b) {
  return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}
// Eye base the stereo path builds on: game camera + anchor delta.
void eye(const Input& in, const float* d, float* out) { for (int k = 0; k < 3; ++k) out[k] = in.cam[k] + d[k]; }
Input frame(bool wanted, float x, float cam_back, float sway = 0.f) {
  Input in;
  in.wanted = wanted; in.body_ok = in.bone_ok = true; in.dt = kDt;
  in.body[0] = x; in.body[1] = 100.f; in.body[2] = 5.f;
  in.bone[0] = x + sway; in.bone[1] = 102.64f; in.bone[2] = 5.f;
  in.cam[0] = x - cam_back; in.cam[1] = 102.64f + 0.1f * cam_back; in.cam[2] = 5.f;
  return in;
}
}  // namespace

int main() {
  // Phase classification = the game's IsOnMount test (state 7/8 with a partner), phase from the camera mode.
  if (mount_state::classify(7, true, 20) != Phase::Mounting) return 1;
  if (mount_state::classify(7, true, 7) != Phase::Riding) return 2;
  if (mount_state::classify(7, true, 21) != Phase::Dismounting) return 3;
  if (mount_state::classify(8, true, 99) != Phase::Riding) return 4;   // unseen (Ikran?) mode still counts as mounted
  if (mount_state::classify(7, true, 4) != Phase::Riding) return 5;    // mounted, game camera already first person
  if (mount_state::classify(7, false, 7) != Phase::None) return 6;
  if (mount_state::classify(2, true, 7) != Phase::None) return 7;
  if (mount_state::classify(-1, true, 7) != Phase::None) return 8;

  // Not wanted: the genuine game camera (delta exactly 0).
  {
    Anchor a;
    for (int i = 0; i < 60; ++i) {
      const float* d = a.step(frame(false, 0.1f * i, 9.f));
      if (d[0] != 0.f || d[1] != 0.f || d[2] != 0.f || a.weight != 0.f) return 10;
    }
  }
  // Mount: the game camera swings out to 9 m behind over 1.4 s while riding at 12 m/s; the eyes stay on the head.
  Anchor a;
  float x = 0.f, worst_entry = 0.f;
  for (int i = 0; i < 300; ++i) {
    x += 12.f * kDt;
    const float back = std::fmin(9.f, 9.f * (i * kDt) / 1.4f);
    const Input in = frame(true, x, back);
    float e[3];
    eye(in, a.step(in), e);
    const float err = dist(e, in.bone);
    if (i >= 6 && err > worst_entry) worst_entry = err;  // after the 0.1 s blend-in
  }
  if (a.weight != 1.f || worst_entry > 1e-3f) { std::printf("entry err %f\n", worst_entry); return 11; }
  // Riding: the eyes follow the head bone exactly, including the rider tilting (default tau 0).
  {
    float worst = 0.f;
    for (int i = 0; i < 600; ++i) {
      x += 12.f * kDt;
      const Input in = frame(true, x, 9.f, 0.4f * std::sin(6.2831853f * 2.f * i * kDt));
      float e[3];
      eye(in, a.step(in), e);
      worst = std::fmax(worst, dist(e, in.bone));
    }
    if (worst > 1e-3f) { std::printf("ride err %f\n", worst); return 12; }
  }
  // Optional low-pass (offline tuning): a 0.25 s filter takes most of a 2 Hz +-0.4 m sway out.
  {
    Anchor f;
    f.tau = 0.25f;
    float lo = 1e9f, hi = -1e9f, fx = 0.f;
    for (int i = 0; i < 600; ++i) {
      fx += 12.f * kDt;
      const Input in = frame(true, fx, 9.f, 0.4f * std::sin(6.2831853f * 2.f * i * kDt));
      float e[3];
      eye(in, f.step(in), e);
      if (i > 120) { lo = std::fmin(lo, e[0] - in.body[0]); hi = std::fmax(hi, e[0] - in.body[0]); }
    }
    if (hi - lo > 0.35f || hi - lo <= 0.f) { std::printf("sway %f\n", hi - lo); return 20; }
  }
  // Dismount end: camera mode 4 snaps the game camera back onto the head. No overshoot during the blend-out.
  {
    for (int i = 0; i < 90; ++i) {
      x += 2.f * kDt;
      const Input in = frame(false, x, 0.f);
      float e[3];
      eye(in, a.step(in), e);
      if (dist(e, in.bone) > 0.05f) { std::printf("exit overshoot %f at %d\n", dist(e, in.bone), i); return 13; }
    }
    if (a.weight != 0.f || a.have) return 14;
  }
  // Bone unreadable: held for 0.2 s, then the correction fades to the game camera.
  {
    Anchor b;
    for (int i = 0; i < 60; ++i) b.step(frame(true, 0.f, 9.f));
    Input in = frame(true, 0.f, 9.f);
    in.bone_ok = false;
    for (int i = 0; i < 6; ++i) b.step(in);  // 0.1 s
    if (b.weight != 1.f || std::fabs(b.delta[0] - 9.f) > 1e-3f) return 15;
    for (int i = 0; i < 60; ++i) b.step(in);  // +1 s
    if (b.weight != 0.f || b.delta[0] != 0.f) return 16;
  }
  // Teleport / load: a 100 m correction is rejected and faded out; bad dt is harmless.
  {
    Anchor c;
    for (int i = 0; i < 60; ++i) c.step(frame(true, 0.f, 9.f));
    for (int i = 0; i < 60; ++i) c.step(frame(true, 0.f, 100.f));
    if (c.rejects == 0 || c.weight != 0.f) return 17;
    Input in = frame(true, 0.f, 9.f);
    in.dt = std::nanf("");
    const float* d = c.step(in);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return 18;
    in.bone[1] = 110.f;  // 10 m from the body: not a head
    Anchor fresh;
    fresh.step(in);
    if (fresh.have || fresh.weight != 0.f) return 19;
  }
  // Head frame: an eye learned on foot in the head bone's axes stays on the head when the rider pitches forward.
  {
    const float c = std::cos(0.6f), s = std::sin(0.6f);
    const float stand[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 10,102.5f,5,1};
    const float eye_world[3] = {10.f, 102.62f, 5.11f};  // 12 cm up, 11 cm forward of the head joint
    mount_anchor::EyeCalib cal;
    float obs[3];
    if (!mount_anchor::to_local(stand, eye_world, obs)) return 30;
    for (int i = 0; i < 40; ++i) cal.add(obs, kDt);
    if (!cal.ready() || std::fabs(cal.local[1] - 0.12f) > 1e-4f || std::fabs(cal.local[2] - 0.11f) > 1e-4f) return 31;
    const float pitched[16] = {1,0,0,0, 0,c,s,0, 0,-s,c,0, 20,101.f,8,1};  // pitched about right, moved
    float w[3];
    mount_anchor::to_world(pitched, cal.local, w);
    const float want[3] = {20.f, 101.f + 0.12f * c - 0.11f * s, 8.f + 0.12f * s + 0.11f * c};
    if (dist(w, want) > 1e-4f) return 32;
    const float far_obs[3] = {0.f, 0.9f, 0.f};  // not a head/eye relation: ignored
    cal.add(far_obs, kDt);
    if (std::fabs(cal.local[1] - 0.12f) > 1e-4f) return 33;
    const float degenerate[16] = {};
    if (mount_anchor::to_local(degenerate, eye_world, obs)) return 34;
  }
  std::puts("mount_anchor_test ok");
  return 0;
}
