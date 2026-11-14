#include "../../src/vr/vr_arms_math.hpp"
#include <cmath>
#include <cstring>
#include <cstdio>
using namespace vr_arms_math;
using skin_arm_ik::Vec;
namespace {
bool close(float a, float b, float e = 1e-4f) { return std::fabs(a - b) < e; }
void identity(float* m) { for (int i = 0; i < 16; ++i) m[i] = (i % 5 == 0) ? 1.f : 0.f; }
void yaw_matrix(float* m, float yaw) {  // forward (sin, 0, cos), up y, right = up x forward
  identity(m);
  m[8] = std::sin(yaw); m[10] = std::cos(yaw);
  m[0] = std::cos(yaw); m[2] = -std::sin(yaw);
}
}
int main() {
  // 1. basis_for_root: the eye basis is turned onto the root's yaw; rows stay orthonormal.
  float eye[16], root[16], basis[16];
  yaw_matrix(eye, 0.3f); eye[12] = 5.f;
  yaw_matrix(root, 1.1f);
  basis_for_root(eye, root, basis);
  if (!close(yaw_of(basis), 1.1f)) return 1;
  if (!close(basis[12], 0.f)) return 2;
  if (!close(basis[0] * basis[8] + basis[1] * basis[9] + basis[2] * basis[10], 0.f)) return 3;

  // 2. to_world: identity basis keeps the sample; a yaw basis turns the offset with it.
  float sample[16]; identity(sample); sample[12] = 0.f; sample[13] = -0.3f; sample[14] = 0.4f;
  float w[16], id[16]; identity(id);
  to_world(sample, id, w);
  if (!close(w[13], -0.3f) || !close(w[14], 0.4f)) return 4;
  float b90[16]; yaw_matrix(b90, 1.5707963f);
  to_world(sample, b90, w);
  if (!close(w[12], 0.4f) || !close(w[14], 0.f, 1e-3f) || !close(w[13], -0.3f)) return 5;

  // 3. hand_target without arm scaling: target = shoulder + (hmd_rel - user_shoulder); turning the body turns the hand
  //    about the shoulder; the attach socket follows the wrist rigidly.
  skin_arm_ik::ArmScale scale; scale.enabled = false;
  skin_arm_ik::HandTune tune;
  HandLocal local; identity(local.wrist); identity(local.rel); local.rel[14] = 0.1f; local.attach = true; local.valid = true;
  float wrist[16], attach[16], factor = 0.f; bool oriented = false;
  const Vec shoulder{0.2f, 1.5f, 0.f};
  if (!hand_target(sample, id, id, shoulder, 0.7f, 1, scale, tune, local, wrist, attach, &factor, &oriented)) return 6;
  const Vec user = skin_arm_ik::real_shoulder(Vec{0, 0, 0}, id, 1, scale);
  if (!close(wrist[12], shoulder.x + 0.f - user.x) || !close(wrist[14], shoulder.z + 0.4f - user.z)) return 7;
  if (!close(attach[14], wrist[14] + 0.1f)) return 8;
  float wrist_b[16], attach_b[16];
  float root90[16]; yaw_matrix(root90, 1.5707963f);
  if (!hand_target(sample, b90, root90, shoulder, 0.7f, 1, scale, tune, local, wrist_b, attach_b, &factor, &oriented)) return 9;
  const float d0 = std::hypot(wrist[12] - shoulder.x, wrist[14] - shoulder.z), d1 = std::hypot(wrist_b[12] - shoulder.x, wrist_b[14] - shoulder.z);
  if (!close(d0, d1, 1e-3f)) return 10;  // same distance from the shoulder: rotated about it

  // 4. determinism: the same inputs on two "threads" give bit-identical sockets.
  float wrist_c[16], attach_c[16];
  hand_target(sample, b90, root90, shoulder, 0.7f, 1, scale, tune, local, wrist_c, attach_c, &factor, &oriented);
  for (int i = 0; i < 16; ++i) if (attach_c[i] != attach_b[i]) return 11;

  // 5. left-handed grip: right tune with the side offset mirrored; the left-mode gun sits on the left side of the same
  //    target (side moves the wrist along the body's right axis) and the right-mode result is unchanged.
  skin_arm_ik::HandTune rt; rt.side = 0.18f; rt.yaw = -30.f; rt.roll = -35.f;
  const skin_arm_ik::HandTune lt = left_grip_tune(rt);
  if (!close(lt.side, -0.18f) || !close(lt.yaw, -30.f) || !close(lt.roll, -35.f)) return 12;
  float wr[16], ar[16], wl[16], al[16];
  hand_target(sample, id, id, shoulder, 0.7f, 1, scale, rt, local, wr, ar, &factor, &oriented);
  hand_target(sample, id, id, shoulder, 0.7f, 1, scale, lt, local, wl, al, &factor, &oriented);
  if (!close(wr[12] - wl[12], 0.36f, 1e-3f)) return 13;  // mirrored side: 2 x 0.18 m apart along body right
  if (!close(al[14], wl[14] + 0.1f)) return 14;           // attach still rigid to the wrist

  // 5b. hybrid reach (2026-11-13): near the body the wrist is eye + hmd_rel with the hand offsets faded out; without an
  //     eye (riding) the result is bit-identical to the old call; the socket stays rigid to the wrist.
  {skin_arm_ik::ArmScale s2; s2.enabled = true; s2.user_length = 0.634f; s2.width = 0.317f; s2.drop = 0.238f; s2.back = 0.03f; s2.reach = 1.15f;
    skin_arm_ik::HybridReach hy; hy.enabled = true;
    float near_s[16]; identity(near_s); near_s[12] = 0.08f; near_s[13] = 0.f; near_s[14] = 0.12f;  // beside the face
    const Vec eye_pt{1.f, 3.f, -2.f}, sh{1.115f, 2.81f, -1.98f};
    float wa[16], aa[16], wb[16], ab[16], wgt = 0.f;
    if (!hand_target(near_s, id, id, sh, 0.99f, 1, s2, rt, local, wa, aa, &factor, &oriented, &eye_pt, &hy, &wgt)) return 60;
    if (wgt > 0.05f || !close(wa[12], 1.08f, 0.01f) || !close(wa[13], 3.f, 0.01f) || !close(wa[14], -1.88f, 0.01f)) return 61;  // within 1 cm  // side 0.18 faded out
    if (!close(aa[14], wa[14] + 0.1f)) return 62;
    hand_target(near_s, id, id, sh, 0.99f, 1, s2, rt, local, wb, ab, &factor, &oriented);
    float wc[16], ac[16];
    hand_target(near_s, id, id, sh, 0.99f, 1, s2, rt, local, wc, ac, &factor, &oriented, nullptr, &hy, &wgt);
    for (int i = 0; i < 16; ++i) if (wb[i] != wc[i] || ab[i] != ac[i]) return 63;
    if (close(wb[13], wa[13], 0.1f)) return 64;}  // the old mapping is far off here (the bug)

  // 6. mirrored hold: proper rotation (det +1), mirrored twice = identity, the hand moves to the other side of the gun.
  {float h[16]; yaw_matrix(h, 0.4f); h[12] = 0.06f; h[13] = -0.05f; h[14] = -0.12f;  // right wrist right of / below the socket
    float m[16], mm[16]; mirror_hold(h, m); mirror_hold(m, mm);
    for (int i = 0; i < 16; ++i) if (!close(mm[i], h[i])) return 15;
    const float det = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) + m[2] * (m[4] * m[9] - m[5] * m[8]);
    if (!close(det, 1.f, 1e-3f)) return 16;
    if (!close(m[12], -0.06f) || !close(m[13], -0.05f) || !close(m[14], -0.12f)) return 17;
    float f[16], g[16]; yaw_matrix(f, -0.7f); f[12] = 0.1f; f[13] = 0.02f; f[14] = -0.03f; mirror_wrist_local(f, g);
    if (!close(g[0], f[0]) || !close(g[10], f[10]) || !close(g[12], -0.1f) || !close(g[14], 0.03f)) return 18;}

  // 7. smooth hand swap blend: ends exact, middle = half the turn and half the way, rotation stays orthonormal.
  {float a[16], b[16], o[16]; yaw_matrix(a, 0.2f); yaw_matrix(b, 1.4f); a[12] = 0.f; b[12] = 0.4f; b[14] = -0.2f;
    blend_affine(a, b, 0.f, o); for (int i = 0; i < 16; ++i) if (!close(o[i], a[i], 1e-4f)) return 19;
    blend_affine(a, b, 1.f, o); for (int i = 0; i < 16; ++i) if (!close(o[i], b[i], 1e-4f)) return 20;
    blend_affine(a, b, 0.5f, o);
    if (!close(yaw_of(o), 0.8f, 1e-3f) || !close(o[12], 0.2f) || !close(o[14], -0.1f)) return 21;
    if (!close(o[0] * o[0] + o[1] * o[1] + o[2] * o[2], 1.f, 1e-4f)) return 22;
    if (!close(smoothstep(0.5f), 0.5f) || !close(smoothstep(0.f), 0.f) || !close(smoothstep(2.f), 1.f)) return 23;}

  // 7b. steady hand frame: fast hand moved onto the slow palm. Knuckles land on their slow positions and a finger keeps
  //     its native offset from its knuckle; mixing rates per bone (the old way) stretched that offset.
  {const auto at = [](float* m, float a, float x, float y, float z) {yaw_matrix(m, a); m[12] = x; m[13] = y; m[14] = z;};
    float sm[16], si[16], sp[16], fm[16], fi[16], fp[16], fix[16], identity[16];
    at(sm, 0.f, 0.f, 0.f, 0.1f); at(si, 0.f, 0.03f, 0.f, 0.09f); at(sp, 0.f, -0.04f, 0.f, 0.08f);
    float move[16]; at(move, 0.5f, 0.02f, 0.01f, -0.09f);
    skin_arm_ik::multiply_affine(fm, sm, move); skin_arm_ik::multiply_affine(fi, si, move); skin_arm_ik::multiply_affine(fp, sp, move);
    const float* const fast[3] = {fm, fi, fp}; const float* const slow[3] = {sm, si, sp};
    if (!steady_hand_correction(slow, slow, identity)) return 90;
    for (int i = 0; i < 16; ++i) if (!close(identity[i], i % 5 == 0 ? 1.f : 0.f, 1e-4f)) return 91;
    if (!steady_hand_correction(fast, slow, fix)) return 92;
    for (int k = 0; k < 3; ++k) {float out[16]; skin_arm_ik::multiply_affine(out, fast[k], fix);
      if (!close(out[12], slow[k][12], 1e-4f) || !close(out[13], slow[k][13], 1e-4f) || !close(out[14], slow[k][14], 1e-4f)) return 93;}
    float finger_local[16], finger[16], knuckle[16], finger_out[16];
    at(finger_local, 0.f, 0.f, 0.f, 0.04f); skin_arm_ik::multiply_affine(finger, finger_local, fm);
    skin_arm_ik::multiply_affine(knuckle, fm, fix); skin_arm_ik::multiply_affine(finger_out, finger, fix);
    const Vec gap = skin_arm_ik::sub(skin_arm_ik::position(finger_out), skin_arm_ik::position(knuckle));
    if (!close(skin_arm_ik::length(gap), 0.04f, 1e-4f)) return 94;
    const Vec mixed_gap = skin_arm_ik::sub(skin_arm_ik::position(finger), skin_arm_ik::position(sm));
    if (skin_arm_ik::length(mixed_gap) < 0.06f) return 95;
    const float* const flat[3] = {sm, sm, sm};
    if (steady_hand_correction(flat, slow, fix)) return 96;}

  // 8. two-hand swing: full fraction aligns the line; the swing axis stays fixed (no roll about it); fraction,
  //    deadzone and ceiling; zero error (the no-snap grab instant) gives no swing; rotation about a pivot.
  {const float from[3] = {0.f, 0.f, 1.f}, to[3] = {1.f, 0.f, 0.f}; float r[16];
    const float a1 = swing_between(from, to, 1.f, 0.f, 10.f, r);
    const float v[3] = {from[0] * r[0] + from[1] * r[4] + from[2] * r[8], from[0] * r[1] + from[1] * r[5] + from[2] * r[9],
                        from[0] * r[2] + from[1] * r[6] + from[2] * r[10]};
    if (!close(a1, 1.5707963f, 1e-4f) || !close(v[0], 1.f, 1e-4f) || !close(v[2], 0.f, 1e-4f)) return 24;
    const float axis_y[3] = {0.f, 1.f, 0.f};  // cross(from,to) is +-y: unchanged by the swing
    if (!close(axis_y[0] * r[0] + axis_y[1] * r[4] + axis_y[2] * r[8], 0.f) || !close(axis_y[0] * r[1] + axis_y[1] * r[5] + axis_y[2] * r[9], 1.f)) return 25;
    if (!close(swing_between(from, to, 0.5f, 0.f, 10.f, r), 0.7853982f, 1e-4f)) return 26;
    if (!close(swing_between(from, to, 1.f, 0.5f, 10.f, r), 1.0707963f, 1e-4f)) return 27;  // deadzone removed
    if (!close(swing_between(from, to, 1.f, 0.f, 0.2f, r), 0.2f, 1e-4f)) return 28;           // ceiling
    if (swing_between(from, from, 1.f, 0.f, 10.f, r) != 0.f || !close(r[0], 1.f)) return 29;   // no error: identity
    float m[16]; identity(m); m[12] = 0.f; m[13] = 0.f; m[14] = 1.f; const float pivot[3] = {0.f, 0.f, 0.f}; float o[16];
    swing_between(from, to, 1.f, 0.f, 10.f, r); rotate_about(m, pivot, r, o);
    if (!close(o[12], 1.f, 1e-4f) || !close(o[14], 0.f, 1e-4f)) return 30;                     // the point swings with the line
    if (!close(two_hand_gain(0.074f / 0.35f), 0.2114f, 1e-3f) || !close(two_hand_gain(1.3f), 1.f) || !close(two_hand_gain(0.05f), 0.2f)) return 31;}

  // shared_view_offset: placed before the camera step (sample N-1, generation N-1) or after it (sample N, generation N),
  // the gun, both hands and the eyes (previous raw of sample N) all resolve to the offset of sample N-1.
  {const float n2[3] = {0.f, 2.5f, 0.f}, n1[3] = {0.f, 2.6f, 0.f}, n0[3] = {0.f, 2.7f, 0.f};
    if (shared_view_offset(7u, 8u, n1, n2) != n1) return 32;
    if (shared_view_offset(8u, 8u, n0, n1) != n1) return 33;
    if (shared_view_offset(6u, 8u, n2, n2) != n2) return 34;}

  // 2026-10-31 audit: a malformed eye basis is refused; a lost alignment or a still-controller jump holds for one generation only.
  {float good[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 5,6,7,1}; if (!basis_good(good)) return 40;
    float scaled[16]; std::memcpy(scaled, good, 64); scaled[0] = 1.3f; if (basis_good(scaled)) return 41;
    float skew[16]; std::memcpy(skew, good, 64); skew[4] = 0.2f; skew[5] = 0.98f; if (basis_good(skew)) return 42;
    float nan_rows[16]; std::memcpy(nan_rows, good, 64); nan_rows[9] = std::nanf(""); if (basis_good(nan_rows)) return 43;
    if (!hold_target(true, false, false, false, true, true, 1.f, 0.f, 0.f)) return 44;   // alignment lost: held whatever the sample did
    if (hold_target(true, false, false, false, false, true, 1.f, 0.f, 0.f)) return 45;   // it was never aligned: nothing to keep
    if (!hold_target(true, false, false, true, true, true, 1e-6f, 3.f, 0.0121f)) return 46;  // 11 cm jump, controller still
    if (hold_target(true, false, false, true, true, true, 0.01f, 3.f, 0.0121f)) return 47;   // the controller really moved
    if (hold_target(true, true, false, true, true, true, 1e-6f, 3.f, 0.0121f)) return 48;    // never a second generation
    if (!hold_target(true, true, true, true, true, true, 1e-6f, 3.f, 0.0121f)) return 49;    // the solve of the gun's generation agrees
  }
  // 2026-10-31 audit: the tuned rotation filter steps less on a still hand and lags little on a real turn.
  {auto run = [](bool tuned, float rate, float* lag) {
      skin_arm_ik::OneEuro f; if (tuned) skin_arm_ik::set_rotation_tuning(f); else {f.min_cutoff = 2.f; f.beta = 60.f; f.d_cutoff = 5.f;}
      const float dt = 1.f / 65.f; unsigned seed = 7u; float last = 0.f, step2 = 0.f, err = 0.f; int n = 0;
      for (int i = 0; i < 4000; ++i) {
        seed = seed * 1664525u + 1013904223u;
        const float noise = (rate > 0.f ? 0.f : 1.f) * 0.004f * (static_cast<float>(seed >> 16 & 0xFFFF) / 65535.f - 0.5f);
        const float truth = rate * dt * static_cast<float>(i % 200);
        if (rate > 0.f && i % 200 == 0) f.primed = false;
        const float out = f.filter(truth + noise, dt);
        if (i > 100) {step2 += (out - last) * (out - last); err += truth - out; ++n;}
        last = out;
      }
      if (lag) *lag = rate > 0.f ? err / n / rate * 1000.f : 0.f;
      return std::sqrt(step2 / n);
    };
    const float tuned = run(true, 0.f, nullptr), wide = run(false, 0.f, nullptr);
    if (!(tuned < 0.6f * wide)) return 50;
    float lag = 0.f; run(true, 2.f, &lag);
    if (!(lag < 20.f)) return 51;  // ms at 2 rad/s
  }

  // 2026-10-31 follow-up: a one-frame offset spike leaves the anchor; steady travel has no lag; it stays within 25 cm; a teleport snaps.
  {if (median3(1.f, 5.f, 2.f) != 2.f || median3(5.f, 1.f, 2.f) != 2.f || median3(1.f, 2.f, 5.f) != 2.f) return 70;
    ViewTracker t; double m[3] = {-7284.0, 0.0, 0.0}; for (int i = 0; i < 60; ++i) { t.step(m, true); m[0] += 0.07; }
    if (std::fabs(t.g[0] - (m[0] - 0.07)) > 0.005) return 71;
    const double before = t.g[0]; double spike[3] = {m[0] + 0.15, 0.0, 0.0}; t.step(spike, true);
    if (std::fabs(t.g[0] - (before + 0.07)) > 0.08) return 72;
    ViewTracker c; double a[3] = {0.0, 0.0, 0.0}; c.step(a, true); double b[3] = {0.5, 0.0, 0.0}; c.step(b, true);
    if (std::fabs(c.g[0] - 0.5) > 0.2501) return 73;
    double far[3] = {5.0, 0.0, 0.0}; c.step(far, true); if (c.g[0] != 5.0) return 74;
    ViewTracker off; off.step(a, false); off.step(b, false); if (off.g[0] != 0.5) return 75;
  }
  std::puts("vr_arms: basis onto root yaw, to_world, hand target about the shoulder, attach rigid, deterministic, left grip, mirrored hold, swap blend, two-hand swing, shared view offset pass");
  return 0;
}
