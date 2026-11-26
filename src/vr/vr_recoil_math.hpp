#pragma once
// Weapon recoil on the tracked gun (2026-11-17). Pure math, no game state: the kick envelope from recent shots and the
// kick as a transform in the weapon wrist's own frame.
//
// Row-vector matrices as everywhere in the mod (p_world = p_local * M, rows 0-2 axes, row 3 position). The kick W acts on
// wrist-local points: wrist' = W * wrist, so the hand, the gun (attach = rel * wrist) and anything else carried by the
// wrist move as one piece. Undoing it for the aim: a ray given in the wrist frame is mapped through inverse(W) first.
#include <cmath>
#include <cstring>

namespace vr_recoil_math {

// Critically damped response to one shot, normalised so the peak is 1 at age 1/omega (seconds).
inline float pulse(float age, float omega) {
  if (!(age >= 0.f) || !(omega > 0.f)) return 0.f;
  const float x = omega * age;
  if (x > 20.f) return 0.f;
  return 2.718281828f * x * std::exp(-x);
}

struct Shot { double at = -1.0; float yaw_sign = 1.f; float scale = 1.f; };
constexpr int kShots = 16;

struct Envelope { float pitch = 0.f, yaw = 0.f; };  // pitch/back amount (0..cap) and signed yaw amount
// Sum of the pulses of recent shots, capped so full auto stacks without running away.
inline Envelope envelope(const Shot* shots, int count, double now, float omega, float cap) {
  Envelope e;
  for (int i = 0; i < count; ++i) {
    if (shots[i].at < 0.0) continue;
    const float p = pulse(static_cast<float>(now - shots[i].at), omega) * shots[i].scale;
    e.pitch += p;
    e.yaw += p * shots[i].yaw_sign;
  }
  if (e.pitch > cap) {
    const float k = cap / e.pitch;
    e.pitch = cap;
    e.yaw *= k;
  }
  return e;
}

// Row-vector rotation rows for angle a about unit axis (x,y,z): v' = v * R.
inline void axis_angle(float x, float y, float z, float a, float* r) {
  const float c = std::cos(a), s = std::sin(a), t = 1.f - c;
  const float m[3][3] = {{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
                         {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
                         {t * x * z - s * y, t * y * z + s * x, t * z * z + c}};  // column-vector matrix
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) r[i * 4 + j] = m[j][i];
    r[i * 4 + 3] = 0.f;
  }
  r[12] = r[13] = r[14] = 0.f;
  r[15] = 1.f;
}
inline void mul(float* out, const float* a, const float* b) {
  float r[16];
  for (int row = 0; row < 4; ++row)
    for (int j = 0; j < 3; ++j)
      r[row * 4 + j] = a[row * 4] * b[j] + a[row * 4 + 1] * b[4 + j] + a[row * 4 + 2] * b[8 + j] + (row == 3 ? b[12 + j] : 0.f);
  r[3] = r[7] = r[11] = 0.f;
  r[15] = 1.f;
  std::memcpy(out, r, sizeof(r));
}
inline void dir_mul(const float* v, const float* m, float* out) {
  float r[3];
  for (int k = 0; k < 3; ++k) r[k] = v[0] * m[k] + v[1] * m[4 + k] + v[2] * m[8 + k];
  std::memcpy(out, r, sizeof(r));
}
inline bool normalize(float* v) {
  const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (!(n > 1e-6f) || !std::isfinite(n)) return false;
  v[0] /= n; v[1] /= n; v[2] /= n;
  return true;
}

// The kick in the wrist frame. forward = barrel direction, up = world up, both in wrist-local axes; pivot = the point it
// turns about (wrist-local, e.g. the grip). The barrel turns `pitch` radians toward up, `yaw` radians about up, and the
// whole hand moves `back` metres against the barrel. False (W = identity) when the axes are degenerate.
inline bool kick_matrix(const float* forward, const float* up, const float* pivot, float pitch, float yaw, float back, float* w) {
  axis_angle(1.f, 0.f, 0.f, 0.f, w);
  float f[3] = {forward[0], forward[1], forward[2]}, u[3] = {up[0], up[1], up[2]};
  if (!normalize(f) || !normalize(u)) return false;
  float a[3] = {f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]};
  if (!normalize(a)) return false;  // barrel straight up or down: no defined pitch axis
  float rp[16], ry[16], r[16];
  axis_angle(a[0], a[1], a[2], pitch, rp);
  // Rotating about f x u turns f away from or toward u depending on handedness: pick the sign that raises the barrel.
  float turned[3];
  dir_mul(f, rp, turned);
  const float before = f[0] * u[0] + f[1] * u[1] + f[2] * u[2], after = turned[0] * u[0] + turned[1] * u[1] + turned[2] * u[2];
  if (pitch != 0.f && after < before) axis_angle(a[0], a[1], a[2], -pitch, rp);
  axis_angle(u[0], u[1], u[2], yaw, ry);
  mul(r, rp, ry);
  // About the pivot: p' = (p - pivot) * R + pivot - f * back
  float shift[3];
  dir_mul(pivot, r, shift);
  for (int k = 0; k < 3; ++k) r[12 + k] = pivot[k] - shift[k] - f[k] * back;
  for (float v : r) if (!std::isfinite(v)) { axis_angle(1.f, 0.f, 0.f, 0.f, w); return false; }
  std::memcpy(w, r, sizeof(r));
  return true;
}

// Rigid inverse (rotation rows orthonormal).
inline void rigid_inverse(const float* m, float* out) {
  float r[16];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) r[i * 4 + j] = m[j * 4 + i];
    r[i * 4 + 3] = 0.f;
  }
  for (int j = 0; j < 3; ++j) r[12 + j] = -(m[12] * r[j] + m[13] * r[4 + j] + m[14] * r[8 + j]);
  r[15] = 1.f;
  std::memcpy(out, r, sizeof(r));
}

// A ray in the kicked wrist frame -> the same world ray as the un-kicked wrist would give: map through inverse(W).
inline void unkick_ray(const float* w, float* origin, float* forward) {
  float inv[16], o[3];
  rigid_inverse(w, inv);
  for (int k = 0; k < 3; ++k) o[k] = origin[0] * inv[k] + origin[1] * inv[4 + k] + origin[2] * inv[8 + k] + inv[12 + k];
  std::memcpy(origin, o, sizeof(o));
  dir_mul(forward, inv, forward);
}

// Per-weapon strength from the game's own fire animation: the peak attach<-wrist residual after a shot (cm), relative to a
// reference, clamped. No reading -> 1.
inline float native_scale(float residual_cm, float reference_cm) {
  if (!(residual_cm > 0.f) || !(reference_cm > 0.f)) return 1.f;
  const float s = residual_cm / reference_cm;
  return s < 0.3f ? 0.3f : s > 1.5f ? 1.5f : s;
}

// Rising-edge detector with a refractory time (ms): true when value crosses `high` from at or below `low`.
struct Edge {
  bool armed = true;
  unsigned long long last = 0;
  bool step(float value, float low, float high, unsigned long long now_ms, unsigned long long refractory_ms) {
    if (value <= low) armed = true;
    if (armed && value >= high && (last == 0 || now_ms - last >= refractory_ms)) {
      armed = false;
      last = now_ms;
      return true;
    }
    return false;
  }
};

}  // namespace vr_recoil_math
