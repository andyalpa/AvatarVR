#pragma once
// VR control scheme math (2026-10-17, run 64): pure, unit-tested pieces of the gesture controls (vr_controls.cpp).
// Hand offsets come from vr_arms::Sample (grip - headset centre, eye-camera axes: x right, y up, z forward) and are
// turned into the head's yaw-only frame, so the zones follow where the player faces.
#include <cmath>

namespace vr_controls_math {

// head_rot9 as eye_transform returns it (row r of the head matrix = (m[r], m[3+r], m[6+r])); d = hand - head.
// out = (right, up, forward) in the head's yaw-only frame. False when the head looks straight up/down.
inline bool head_frame(const float* head_rot9, const float* d, float* out) {
  float fx = head_rot9[2], fz = head_rot9[8];
  const float l = std::sqrt(fx * fx + fz * fz);
  if (l < 1e-3f) return false;
  fx /= l; fz /= l;
  const float rx = fz, rz = -fx;  // right = (fz, 0, -fx) for x right / y up / z forward
  out[0] = d[0] * rx + d[2] * rz;
  out[1] = d[1];
  out[2] = d[0] * fx + d[2] * fz;
  return true;
}

// Empty hand at the headset (scan). Hysteresis: enter 0.20 m, leave 0.26 m.
inline bool in_head(const float* p, bool was) {
  const float r = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  return r < (was ? 0.26f : 0.20f);
}
// Hand over its own shoulder (hand 0 left, 1 right): above -0.30 m, behind the face plane (+0.02 m), on its own side
// beyond 0.05 m, within 0.45 m of the headset. Leaving needs 5 cm more on every bound.
inline bool in_shoulder(int hand, const float* p, bool was) {
  const float m = was ? 0.05f : 0.f;
  const float side = hand ? p[0] : -p[0];
  const float r = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  return p[1] > -0.30f - m && p[2] < 0.02f + m && side > 0.05f - m && r < 0.45f + m;
}

// Right-stick flick: +1 once past +0.70 (jump), -1 once past -0.70 (crouch); re-armed when back under 0.30.
struct Flick { bool armed = true; };
inline int flick(Flick& f, float y) {
  if (std::fabs(y) < 0.30f) {f.armed = true; return 0;}
  if (!f.armed) return 0;
  if (y > 0.70f) {f.armed = false; return 1;}
  if (y < -0.70f) {f.armed = false; return -1;}
  return 0;
}

// Shoulder grip tap: what it does.
enum class Equip { kEquipToHand, kHolster, kSwitchHand };
inline Equip equip_decision(int hand, bool weapon_held, int weapon_hand) {
  if (!weapon_held) return Equip::kEquipToHand;
  return hand == weapon_hand ? Equip::kHolster : Equip::kSwitchHand;
}

constexpr unsigned kHoldMs = 250;  // shoulder grip held longer than this = weapon wheel, shorter = tap
constexpr unsigned kTapMs = 120;   // synthetic button taps last this long

}  // namespace vr_controls_math
