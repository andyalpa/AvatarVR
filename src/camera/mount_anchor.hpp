#pragma once

#include <algorithm>
#include <cmath>

// FEATURE-06 stage A: where the eyes go while mounted. Portable (no engine types); unit-tested in
// tools/tests/mount_anchor_test.cpp. LIVE basis (CAMERA_FINDINGS.md 2026-10-24): in the mount camera modes the game
// camera sits 6-10 m behind the rider, while first-person bone 0 stays a rigid 2.65 m from the body root and moves
// with it; its direction turns as the rider tilts (Ikran take-off). The anchor is body + the bone's body-relative
// offset. Replaying the capture: a 0.25 s low-pass lagged the tilt by up to 3.2 m and cut second-difference jitter by
// only 6 % (0.169 -> 0.158), so the default follows the bone exactly, as the game's own first-person camera does; `tau`
// stays for offline tuning. The output is a world delta added to the game camera position the eye cameras, grips and
// aim rays are built on; 0 means "genuine game camera" (today's behaviour).
namespace mount_anchor {

constexpr float kOffsetTau = 0.f;    // s: body-relative head offset low-pass (0 = follow the bone)
constexpr float kBlendIn = 0.1f;     // s: 0 -> 1 on entry (the game camera is still near the head then)
constexpr float kBlendOut = 0.35f;   // s: 1 -> 0 on exit or loss of the anchor
constexpr float kHold = 0.2f;        // s: keep the last offset through brief unreadable bone samples
constexpr float kMaxBone = 3.5f;     // m: bone farther than this from the body is not a head
constexpr float kMaxDelta = 25.f;    // m: larger camera corrections are rejected (teleport / load)

struct Input {
  bool wanted = false;   // toggle on, mounted and the game camera is a mount camera (not first person)
  bool body_ok = false;
  bool bone_ok = false;
  float body[3] = {};
  float bone[3] = {};    // first-person camera bone, world
  float cam[3] = {};     // game camera position, world
  float dt = 0.f;        // s since the previous step
};

struct Anchor {
  float tau = kOffsetTau;  // tunable for offline replay
  bool have = false;
  float offset[3] = {};  // bone - body (low-passed when tau > 0)
  float weight = 0.f;
  float held = 0.f;
  float delta[3] = {};   // last output
  unsigned rejects = 0;

  // Returns the world delta for the camera position (also kept in `delta`).
  const float* step(const Input& in) {
    const float dt = std::clamp(std::isfinite(in.dt) ? in.dt : 0.f, 0.f, 0.1f);
    bool bone = in.body_ok && in.bone_ok;
    float target[3] = {};
    if (bone) {
      float d2 = 0.f;
      for (int k = 0; k < 3; ++k) {
        target[k] = in.bone[k] - in.body[k];
        d2 += target[k] * target[k];
      }
      bone = std::isfinite(d2) && d2 < kMaxBone * kMaxBone;
    }
    if (bone) {
      const float a = have && tau > 0.f ? 1.f - std::exp(-dt / tau) : 1.f;
      for (int k = 0; k < 3; ++k) offset[k] += (target[k] - offset[k]) * a;
      have = true;
      held = 0.f;
    } else {
      held += dt;
    }
    // Eyes = lerp(game camera, anchor, weight). While the anchor is tracked the blend uses the live anchor in both
    // directions: at the 21 -> 4 switch the game camera snaps back to the head, so a stored delta would overshoot.
    float desired[3] = {};
    bool tracked = in.body_ok && have && held <= kHold;
    if (tracked) {
      float d2 = 0.f;
      for (int k = 0; k < 3; ++k) {
        desired[k] = in.body[k] + offset[k] - in.cam[k];
        d2 += desired[k] * desired[k];
      }
      if (!std::isfinite(d2) || d2 > kMaxDelta * kMaxDelta) {
        tracked = false;
        if (in.wanted) ++rejects;
      }
    }
    const bool on = in.wanted && tracked;
    weight = on ? std::min(1.f, weight + dt / kBlendIn) : std::max(0.f, weight - dt / kBlendOut);
    if (tracked) {
      for (int k = 0; k < 3; ++k) delta[k] = weight * desired[k];
    } else {
      // Anchor lost: fade the last correction out over the game camera the game keeps driving.
      const float before = weight + dt / kBlendOut;
      const float s = before > 0.f ? weight / before : 0.f;
      for (int k = 0; k < 3; ++k) delta[k] *= s;
    }
    if (weight == 0.f) {
      for (int k = 0; k < 3; ++k) delta[k] = 0.f;
      if (!in.wanted) have = false;  // re-seed the offset on the next mount
    }
    return delta;
  }
};

// Head-bone frame helpers (engine rows: 0 right, 1 up, 2 forward, 3 position; rows may carry a uniform scale).
inline bool to_local(const float* m, const float* world, float* local) {
  for (int r = 0; r < 3; ++r) {
    const float* a = m + 4 * r;
    const float n2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    if (!(n2 > 1e-6f) || !std::isfinite(n2)) return false;
    local[r] = (a[0] * (world[0] - m[12]) + a[1] * (world[1] - m[13]) + a[2] * (world[2] - m[14])) / n2;
  }
  return std::isfinite(local[0]) && std::isfinite(local[1]) && std::isfinite(local[2]);
}
inline void to_world(const float* m, const float* local, float* world) {
  for (int k = 0; k < 3; ++k) world[k] = m[12 + k] + local[0] * m[k] + local[1] * m[4 + k] + local[2] * m[8 + k];
}

// [HEADSET 2026-10-24] While mounted only the third-person pose (192 bones) is animated: first-person bone 0 kept a
// standing pose behind and above the riding head. The eye point is therefore taken on the third-person C_head bone.
// Its offset in that bone's frame is learned on foot, where the game camera is the eye (camera mode 4).
constexpr float kMaxEyeFromHead = 0.5f;  // m: larger on-foot offsets are not a head/eye relation
constexpr float kCalibTau = 2.f;         // s: running average after the first kCalibMin samples
constexpr unsigned kCalibMin = 30;
struct EyeCalib {
  float local[3] = {};
  unsigned samples = 0;
  void add(const float* obs, float dt) {
    const float d2 = obs[0] * obs[0] + obs[1] * obs[1] + obs[2] * obs[2];
    if (!std::isfinite(d2) || d2 > kMaxEyeFromHead * kMaxEyeFromHead) return;
    const float a = samples < kCalibMin ? 1.f / static_cast<float>(samples + 1)
                                        : 1.f - std::exp(-std::clamp(dt, 0.f, 0.1f) / kCalibTau);
    for (int k = 0; k < 3; ++k) local[k] += (obs[k] - local[k]) * a;
    ++samples;
  }
  bool ready() const { return samples >= kCalibMin; }
};

}  // namespace mount_anchor
