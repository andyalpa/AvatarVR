#pragma once
// Clean arm rig math (2026-10-17, run 54). One target function for every hand, evaluated from one controller sample
// stream, so the free hand, the gun hand and the game's gun placement can never disagree by construction.
//
// Inputs:
//   sample  : controller grip relative to the headset centre, in the eye camera's own axes (tracking space after
//             recentre; rows 0-2 = grip axes, row 3 = grip - head). No game camera position is involved.
//   basis   : world axes the eyes are built on for the frame (rows 0-2). The basis of a given game tick is the latest
//             eye basis turned about up onto that tick's body-root yaw ([LIVE run 48]: drawn root yaw == eye camera yaw,
//             200/200 draws), so it never depends on whether the eye camera of that tick was built before or after.
//   body    : that tick's root rows (body axes), the hand-side Na'vi shoulder position and arm length.
//   hand    : the hand's own layout from any drawn pose (wrist matrix, three knuckles, attach<-wrist relation); only
//             relative vectors are used, so a layout from another frame gives the same result.
#include "../diagnostics/skin_arm_ik.hpp"
#include <cmath>
#include <cstring>

namespace vr_arms_math {
using skin_arm_ik::Vec;

// Same composition as native_stereo::compute_aim: world row r = sum_k local[r][k] * basis row k; the position row is a
// direction offset (no translation).
inline void to_world(const float* local16, const float* basis16, float* out16) {
  for (int r = 0; r < 4; ++r) {
    for (int k = 0; k < 3; ++k)
      out16[r * 4 + k] = local16[r * 4] * basis16[k] + local16[r * 4 + 1] * basis16[4 + k] + local16[r * 4 + 2] * basis16[8 + k];
    out16[r * 4 + 3] = r == 3 ? 1.f : 0.f;
  }
}
// "Lock arms to view": the gun is placed either side of the eye camera step within a frame, so every hand state and the
// eyes use the offset of the sample before this frame's. A sample published in this Present generation answers with its
// previous offset, an older one (placed before the step) with its own: both give the same value.
inline const float* shared_view_offset(unsigned sample_gen, unsigned now_gen, const float* raw, const float* previous_raw) {
  return sample_gen == now_gen ? previous_raw : raw;
}
inline float yaw_of(const float* m) { return std::atan2(m[8], m[10]); }  // forward = row 2, game yaw (sin, 0, cos)
inline float median3(float a, float b, float c) { return a < b ? (b < c ? b : (a < c ? c : a)) : (a < c ? a : (b < c ? c : b)); }
// [LIVE 2026-10-31] running, the camera-minus-body offset jumps by a whole frame of travel for one frame when either was read a tick
// apart: the anchor of the arms and eyes jerked over 3 cm in 88 of 2714 moving frames, the game camera itself in 32. A one-frame median of the
// offset and this per-frame tracker of the world path replay at 30 such frames (p90 1.19 cm against 1.70). Per frame, not per second: the display
// shows frames. Doubles: world coordinates do not fit a float at this precision. It never strays over 25 cm; a jump over 60 cm is a teleport.
struct ViewTracker {
  double g[3] = {}, w[3] = {}; bool primed = false;
  void step(const double* m, bool track) {
    double far2 = 0.0; for (int k = 0; k < 3; ++k) far2 += (m[k] - g[k]) * (m[k] - g[k]);
    if (!primed || !track || far2 > 0.6 * 0.6) { for (int k = 0; k < 3; ++k) { g[k] = m[k]; w[k] = 0.0; } primed = true; return; }
    for (int k = 0; k < 3; ++k) { const double p = g[k] + w[k], r = m[k] - p; g[k] = p + 0.45 * r; w[k] += 0.12 * r; }
    double e2 = 0.0; for (int k = 0; k < 3; ++k) e2 += (g[k] - m[k]) * (g[k] - m[k]);
    if (e2 > 0.25 * 0.25) { const double s = 0.25 / std::sqrt(e2); for (int k = 0; k < 3; ++k) g[k] = m[k] + (g[k] - m[k]) * s; }
  }
};
// A basis grip_basis would refuse: one such sample drops the palm offset from every hand and the gun for a frame.
inline bool basis_good(const float* m) {
  for (int i = 0; i < 12; ++i) if (!std::isfinite(m[i])) return false;
  for (int r = 0; r < 3; ++r) {
    const float n = m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2];
    if (std::fabs(n - 1.f) > 0.04f) return false;
    const int o = (r + 1) % 3;
    if (std::fabs(m[r * 4] * m[o * 4] + m[r * 4 + 1] * m[o * 4 + 1] + m[r * 4 + 2] * m[o * 4 + 2]) > 0.02f) return false;
  }
  return true;
}
// A wrist target that lost its alignment or jumped with a still controller keeps its last place for one generation, for
// the free hand, the gun hand and the placed gun alike. Positions are relative to the shoulder frame.
inline bool hold_target(bool have, bool held, bool same_gen, bool oriented, bool had_oriented, bool has_knuckles,
                        float sample_move2, float sample_turn, float target_move2) {
  if (!have || (held && !same_gen)) return false;
  if (!oriented && had_oriented && has_knuckles) return true;
  return sample_move2 < 0.005f * 0.005f && sample_turn > 2.99f && target_move2 > 0.03f * 0.03f;
}
// The eye basis turned about up so its forward yaw equals the root's (rotation rows only; position cleared).
inline void basis_for_root(const float* eye_basis16, const float* root16, float* out16) {
  std::memcpy(out16, eye_basis16, 16 * sizeof(float));
  out16[12] = out16[13] = out16[14] = 0.f;
  float d = yaw_of(root16) - yaw_of(eye_basis16);
  while (d > 3.14159265f) d -= 6.28318531f;
  while (d < -3.14159265f) d += 6.28318531f;
  skin_arm_ik::rotate_about_up(out16, 0.f, 0.f, d);
}

// Left-handed weapon mode (run 59): the gun is held with a right-hand grip at the LEFT controller (a rigid gun cannot be
// mirrored), so the right hand's tune is used with its sideways body offset mirrored to the left side.
inline skin_arm_ik::HandTune left_grip_tune(skin_arm_ik::HandTune right) { right.side = -right.side; return right; }

// [LIVE run 62 mirror probe] L/R bones are mirrored with all axes negated (D = -I: child offsets v_L = -v_R in 17/17
// pairs on x), and the gun socket's lateral axis is its x row. So, with M the reflection across the socket's sagittal
// plane, a left bone's axes are -(right axes * M):
//   mirror_hold: the right wrist's pose in the socket frame (rows = axes, row 3 = position; wrist = H * socket) ->
//                the left wrist's mirrored hold. Rows (x,y,z) -> (x,-y,-z) (det stays +1), position x negated.
//   mirror_wrist_local: a finger's pose relative to its wrist -> the partner finger's (rotation equal, position negated).
inline void mirror_hold(const float* h, float* out) {
  for (int r = 0; r < 3; ++r) {out[r * 4] = h[r * 4]; out[r * 4 + 1] = -h[r * 4 + 1]; out[r * 4 + 2] = -h[r * 4 + 2]; out[r * 4 + 3] = 0.f;}
  out[12] = -h[12]; out[13] = h[13]; out[14] = h[14]; out[15] = 1.f;
}
inline void mirror_wrist_local(const float* f, float* out) {
  std::memcpy(out, f, 16 * sizeof(float));
  out[12] = -f[12]; out[13] = -f[13]; out[14] = -f[14];
}

// Run 65 (smooth hand swap): blend two rigid transforms (rows = axes, row 3 = position): position lerp, rotation slerp.
inline void quat_from_rows(const float* m, float* q) {  // q = (w, x, y, z) of the 3x3 part, consistent with rows_from_quat
  const float tr = m[0] + m[5] + m[10];
  if (tr > 0.f) {const float s = std::sqrt(tr + 1.f) * 2.f; q[0] = 0.25f * s; q[1] = (m[6] - m[9]) / s; q[2] = (m[8] - m[2]) / s; q[3] = (m[1] - m[4]) / s;}
  else if (m[0] > m[5] && m[0] > m[10]) {const float s = std::sqrt(1.f + m[0] - m[5] - m[10]) * 2.f; q[0] = (m[6] - m[9]) / s; q[1] = 0.25f * s; q[2] = (m[4] + m[1]) / s; q[3] = (m[8] + m[2]) / s;}
  else if (m[5] > m[10]) {const float s = std::sqrt(1.f + m[5] - m[0] - m[10]) * 2.f; q[0] = (m[8] - m[2]) / s; q[1] = (m[4] + m[1]) / s; q[2] = 0.25f * s; q[3] = (m[9] + m[6]) / s;}
  else {const float s = std::sqrt(1.f + m[10] - m[0] - m[5]) * 2.f; q[0] = (m[1] - m[4]) / s; q[1] = (m[8] + m[2]) / s; q[2] = (m[9] + m[6]) / s; q[3] = 0.25f * s;}
}
inline void rows_from_quat(const float* q, float* m) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y + w * z);     m[2] = 2 * (x * z - w * y);
  m[4] = 2 * (x * y - w * z);     m[5] = 1 - 2 * (x * x + z * z); m[6] = 2 * (y * z + w * x);
  m[8] = 2 * (x * z + w * y);     m[9] = 2 * (y * z - w * x);     m[10] = 1 - 2 * (x * x + y * y);
}
// Steady hand frame: fingers follow the animation faster than the knuckles that steer the rig. Each knuckle bone is a
// finger's first segment, so mixing the two rates pulls fingers apart; move the whole fast hand onto the slow palm
// instead: bone_out = bone_fast * correction. Matrices are relative to the wrist; false on a degenerate palm.
inline bool knuckle_palm(const float* middle, const float* index, const float* pinky, float* out) {
  const Vec m = skin_arm_ik::position(middle), p = skin_arm_ik::position(pinky);
  const Vec across = skin_arm_ik::sub(skin_arm_ik::position(index), p), along = skin_arm_ik::sub(m, p);
  const Vec normal{across.y * along.z - across.z * along.y, across.z * along.x - across.x * along.z, across.x * along.y - across.y * along.x};
  const float la = skin_arm_ik::length(across), ln = skin_arm_ik::length(normal);
  if (!(la > 1e-4f) || !(ln > 1e-8f)) return false;
  const Vec x = skin_arm_ik::scale(across, 1.f / la), z = skin_arm_ik::scale(normal, 1.f / ln);
  const Vec y{z.y * x.z - z.z * x.y, z.z * x.x - z.x * x.z, z.x * x.y - z.y * x.x};
  const float rows[16] = {x.x, x.y, x.z, 0.f, y.x, y.y, y.z, 0.f, z.x, z.y, z.z, 0.f, m.x, m.y, m.z, 1.f};
  std::memcpy(out, rows, sizeof(rows));
  return true;
}
inline bool steady_hand_correction(const float* const fast[3], const float* const slow[3], float* out) {
  float palm_fast[16], palm_slow[16], inverse_fast[16];
  if (!knuckle_palm(fast[0], fast[1], fast[2], palm_fast) || !knuckle_palm(slow[0], slow[1], slow[2], palm_slow) ||
      !skin_arm_ik::inverse_affine(inverse_fast, palm_fast)) return false;
  skin_arm_ik::multiply_affine(out, inverse_fast, palm_slow);
  for (int i = 0; i < 16; ++i) if (!std::isfinite(out[i])) return false;
  return true;
}
inline void blend_affine(const float* a, const float* b, float t, float* out) {
  float qa[4], qb[4], q[4];
  quat_from_rows(a, qa); quat_from_rows(b, qb);
  float d = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
  if (d < 0.f) {for (float& v : qb) v = -v; d = -d;}
  float wa = 1.f - t, wb = t;
  if (d < 0.9995f) {const float th = std::acos(d), s = std::sin(th); wa = std::sin((1.f - t) * th) / s; wb = std::sin(t * th) / s;}
  float n = 0.f;
  for (int i = 0; i < 4; ++i) {q[i] = wa * qa[i] + wb * qb[i]; n += q[i] * q[i];}
  n = std::sqrt(n); for (float& v : q) v /= n;
  rows_from_quat(q, out);
  out[3] = out[7] = out[11] = 0.f;
  for (int k = 0; k < 3; ++k) out[12 + k] = a[12 + k] + (b[12 + k] - a[12 + k]) * t;
  out[15] = 1.f;
}
inline float smoothstep(float t) { t = t < 0.f ? 0.f : t > 1.f ? 1.f : t; return t * t * (3.f - 2.f * t); }

// Run 67 two-hand aiming (Cyberpunk VR port TwoHandGrip rules): the gun swings about the dominant grip so the line
// dominant grip -> support point turns toward dominant grip -> support controller. Shortest arc (no roll about the
// barrel: the roll stays with the dominant wrist), the first `deadzone` radians of error ignored, `fraction` of the
// rest applied, capped at `ceiling`. Row-vector rotation rows: v' = v * R.
inline void axis_angle_rows(float x, float y, float z, float a, float* r) {
  const float c = std::cos(a), s = std::sin(a), t = 1.f - c;
  const float m[3][3] = {{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
                         {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
                         {t * x * z - s * y, t * y * z + s * x, t * z * z + c}};  // column-vector matrix
  for (int i = 0; i < 3; ++i) {for (int j = 0; j < 3; ++j) r[i * 4 + j] = m[j][i]; r[i * 4 + 3] = 0.f;}
  r[12] = r[13] = r[14] = 0.f; r[15] = 1.f;
}
// Returns the applied angle (radians); r = identity when nothing applies.
inline float swing_between(const float* from, const float* to, float fraction, float deadzone, float ceiling, float* r) {
  axis_angle_rows(1.f, 0.f, 0.f, 0.f, r);
  const float lf = std::sqrt(from[0] * from[0] + from[1] * from[1] + from[2] * from[2]);
  const float lt = std::sqrt(to[0] * to[0] + to[1] * to[1] + to[2] * to[2]);
  if (lf < 1e-4f || lt < 1e-4f || fraction <= 0.f) return 0.f;
  const float a[3] = {from[0] / lf, from[1] / lf, from[2] / lf}, b[3] = {to[0] / lt, to[1] / lt, to[2] / lt};
  float ax[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  const float ls = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
  const float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const float angle = std::atan2(ls, d);
  if (ls < 1e-6f) return 0.f;
  float applied = (angle - deadzone) * fraction;
  if (applied <= 0.f) return 0.f;
  if (applied > ceiling) applied = ceiling;
  axis_angle_rows(ax[0] / ls, ax[1] / ls, ax[2] / ls, applied, r);
  return applied;
}
// out = m rotated by r about pivot (axes rows and position).
inline void rotate_about(const float* m, const float* pivot, const float* r, float* out) {
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) out[i * 4 + k] = m[i * 4] * r[k] + m[i * 4 + 1] * r[4 + k] + m[i * 4 + 2] * r[8 + k];
    out[i * 4 + 3] = 0.f;
  }
  const float p[3] = {m[12] - pivot[0], m[13] - pivot[1], m[14] - pivot[2]};
  for (int k = 0; k < 3; ++k) out[12 + k] = pivot[k] + p[0] * r[k] + p[1] * r[4 + k] + p[2] * r[8 + k];
  out[15] = 1.f;
}
// Leverage = support hold distance from the dominant grip / 0.35 m (a rifle handguard). Pistol ~0.2, rifle 1.
inline float two_hand_gain(float leverage) { return leverage < 0.2f ? 0.2f : leverage > 1.f ? 1.f : leverage; }

struct HandLocal {
  float wrist[16] = {};
  Vec middle{}, index{}, pinky{};
  float rel[16] = {};  // attach = rel * wrist
  bool knuckles = false, attach = false, valid = false;
};

// Wrist target (axes + position) and attach socket for one hand. Positions come out in the space of `shoulder`
// (world or cell-local; the caller is consistent). Returns false only on non-finite input.
inline bool hand_target(const float* sample16, const float* basis16, const float* root16, Vec shoulder, float navi_length,
                        unsigned hand, const skin_arm_ik::ArmScale& scale, const skin_arm_ik::HandTune& tune,
                        const HandLocal& local, float* wrist16, float* attach16, float* factor, bool* oriented) {
  float grip[16];
  to_world(sample16, basis16, grip);
  const Vec hmd_rel{grip[12], grip[13], grip[14]};
  const Vec p = skin_arm_ik::model_hand_point(hmd_rel, root16, hand, scale, shoulder, navi_length, factor);
  if (!skin_arm_ik::finite(p)) return false;
  grip[12] = p.x; grip[13] = p.y; grip[14] = p.z;
  std::memcpy(wrist16, local.wrist, 16 * sizeof(float));
  Vec target = p;
  bool ok = false;
  if (local.knuckles) ok = skin_arm_ik::align_hand(wrist16, target, local.middle, local.index, local.pinky, grip, tune);
  if (!ok) { std::memcpy(wrist16, local.wrist, 16 * sizeof(float)); target = p; }
  const Vec point = skin_arm_ik::body_offset(target, root16, tune);
  if (!skin_arm_ik::finite(point)) return false;
  wrist16[12] = point.x; wrist16[13] = point.y; wrist16[14] = point.z; wrist16[15] = 1.f;
  if (local.attach) skin_arm_ik::multiply_affine(attach16, local.rel, wrist16);
  else std::memcpy(attach16, wrist16, 16 * sizeof(float));
  if (oriented) *oriented = ok;
  return true;
}
}  // namespace vr_arms_math
