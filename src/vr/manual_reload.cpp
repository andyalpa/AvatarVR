#include "manual_reload.hpp"
#include "manual_reload_math.hpp"
#include "reload_probe.hpp"
#include "vr_weapon.hpp"
#include "xr_input.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../camera/mount_camera.hpp"
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <mutex>

namespace manual_reload {
namespace {
bool g_enabled = true;
bool g_pace = true;  // Stage 2 timer write (author approval 2026-11-19)
bool g_anim_pace = true;  // reload animation length write (author approval 2026-11-19, experiment)
// The reload animation's clip, paced by its length (comp+0x3F38): the game's own length while it should play, very long
// while it should wait. g_anim_a = estimated clip position (0..1), integrated with the length in force.
long long g_anim_orig = 0; float g_anim_a = 0.f; ULONGLONG g_anim_ms = 0; int g_anim_mode = -1;
unsigned g_anim_writes = 0, g_anim_fails = 0;
// Rack: the off hand is pinned to the charging handle (HEADSET 2026-11-19: the hand did not attach to the rack).
float g_pin_rel[16]{}; float g_pin_dir[3]{}; bool g_pin_have = false; ULONGLONG g_pin_on_ms = 0, g_pin_off_ms = 0;
int g_pin_hand = -1;
float g_lever_s = 0.f; ULONGLONG g_lever_s_ms = 0;  // the handle's pull, smoothed (controller jitter)
manual_reload_math::State g_state;
std::atomic<int> g_native_hand{-1};  // the off hand shows the game's own fingers (its reload hold), -1 none
float g_hand_offset[16]{}; bool g_have_offset = false;  // magazine = offset * wrist (row vectors), set at the grab
float g_mag_rel[16]{};  // the held magazine in the in-hand C_root frame, this tick
float g_last_jump = -1.f, g_last_hold = -1.f, g_prev_lever_pull = 0.f;
unsigned g_writes = 0, g_write_fails = 0;
std::mutex g_visual_mutex;
Visual g_visual;
bool g_hip_was = false;
bool g_was_active = false;
const char* g_last_why = "";

void log(const char* fmt, const char* a = "", double x = 0, double y = 0, double z = 0) {
  char line[400]; std::snprintf(line, sizeof(line), fmt, a, x, y, z); reload_probe::log_text(line);
}

void mul(float (&o)[16], const float* a, const float* b) {  // o = a * b (row vectors: a first)
  float r[16];
  for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
    float t = 0.f; for (int k = 0; k < 4; ++k) t += a[i * 4 + k] * b[k * 4 + j]; r[i * 4 + j] = t;}
  std::memcpy(o, r, 64);
}
bool inverse_affine(float (&o)[16], const float* m) {
  const float a = m[0], b = m[1], c = m[2], d = m[4], e = m[5], f = m[6], g = m[8], h = m[9], i = m[10];
  const float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const float det = a * A + b * B + c * C;
  if (!std::isfinite(det) || std::fabs(det) < 1e-12f) return false;
  const float s = 1.f / det;
  const float r[9] = {A * s, -(b * i - c * h) * s, (b * f - c * e) * s, B * s, (a * i - c * g) * s, -(a * f - c * d) * s,
                      C * s, -(a * h - b * g) * s, (a * e - b * d) * s};
  std::memset(o, 0, sizeof(o));
  for (int row = 0; row < 3; ++row) for (int k = 0; k < 3; ++k) o[row * 4 + k] = r[row * 3 + k];
  for (int k = 0; k < 3; ++k) o[12 + k] = -(m[12] * r[k] + m[13] * r[3 + k] + m[14] * r[6 + k]);
  o[15] = 1.f;
  return true;
}
// world = rel * root (row vectors): the translation of a bone at rest, world.
void rest_point(const float* rel, const float* root, float* out) {
  for (int k = 0; k < 3; ++k) out[k] = rel[12] * root[k] + rel[13] * root[4 + k] + rel[14] * root[8 + k] + root[12 + k];
}
bool unit_axes(const float* root, float* axes) {
  for (int r = 0; r < 3; ++r) {
    const float* v = root + r * 4; const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l < 1e-6f) return false;
    for (int k = 0; k < 3; ++k) axes[r * 3 + k] = v[k] / l;
  }
  return true;
}
// Smoothing (HEADSET 2026-11-19: "add some lerp so it looks natural"): the seated magazine glides from where the hand
// let it go into the well, and the charging handle springs back after the rack, instead of jumping.
constexpr unsigned kSeatMs = 140, kLeverBackMs = 110;
float g_seat_from[16]{}; ULONGLONG g_seat_ms = 0;
float g_lever_from = 0.f; ULONGLONG g_lever_ms = 0;
float g_rest_rel[16]{};  // C_magazine at rest in the root frame (this tick)
float smooth01(float x) {x = std::fmin(1.f, std::fmax(0.f, x)); return x * x * (3.f - 2.f * x);}
void to_quat(const float* m, float* q) {  // rows of m (scale removed) -> quaternion (x y z w)
  float r[9];
  for (int i = 0; i < 3; ++i) {const float* v = m + i * 4; const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    for (int k = 0; k < 3; ++k) r[i * 3 + k] = l > 1e-6f ? v[k] / l : 0.f;}
  const float tr = r[0] + r[4] + r[8];
  if (tr > 0.f) {const float s = std::sqrt(tr + 1.f) * 2.f; q[3] = 0.25f * s; q[0] = (r[5] - r[7]) / s; q[1] = (r[6] - r[2]) / s; q[2] = (r[1] - r[3]) / s;}
  else if (r[0] > r[4] && r[0] > r[8]) {const float s = std::sqrt(1.f + r[0] - r[4] - r[8]) * 2.f; q[3] = (r[5] - r[7]) / s; q[0] = 0.25f * s; q[1] = (r[3] + r[1]) / s; q[2] = (r[6] + r[2]) / s;}
  else if (r[4] > r[8]) {const float s = std::sqrt(1.f + r[4] - r[0] - r[8]) * 2.f; q[3] = (r[6] - r[2]) / s; q[0] = (r[3] + r[1]) / s; q[1] = 0.25f * s; q[2] = (r[7] + r[5]) / s;}
  else {const float s = std::sqrt(1.f + r[8] - r[0] - r[4]) * 2.f; q[3] = (r[1] - r[3]) / s; q[0] = (r[6] + r[2]) / s; q[1] = (r[7] + r[5]) / s; q[2] = 0.25f * s;}
}
void from_quat(const float* q, float scale, float* m) {  // inverse of to_quat, rows scaled by scale
  const float x = q[0], y = q[1], z = q[2], w = q[3];
  const float r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
                      2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                      2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)};
  for (int i = 0; i < 3; ++i) for (int k = 0; k < 3; ++k) m[i * 4 + k] = r[i * 3 + k] * scale;
}
// a -> b by t: position lerp, rotation slerp (row-vector affine, uniform scale kept from b).
void blend(float (&o)[16], const float* a, const float* b, float t) {
  float qa[4], qb[4]; to_quat(a, qa); to_quat(b, qb);
  float d = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
  if (d < 0.f) {for (float& v : qb) v = -v; d = -d;}
  float q[4];
  if (d > 0.9995f) {for (int i = 0; i < 4; ++i) q[i] = qa[i] + t * (qb[i] - qa[i]);}
  else {const float th = std::acos(d), s = std::sin(th), wa = std::sin((1.f - t) * th) / s, wb = std::sin(t * th) / s;
    for (int i = 0; i < 4; ++i) q[i] = wa * qa[i] + wb * qb[i];}
  const float l = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); for (float& v : q) v /= l;
  const float scale = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
  std::memset(o, 0, sizeof(o)); from_quat(q, scale, o);
  for (int k = 0; k < 3; ++k) o[12 + k] = a[12 + k] + t * (b[12 + k] - a[12 + k]);
  o[15] = 1.f;
}
void publish(const manual_reload_math::Output& o, bool active) {
  Visual v; v.active = active && (o.mag != manual_reload_math::MagDraw::kGame || o.lever_override);
  switch (o.mag) {
    case manual_reload_math::MagDraw::kRest: v.mag = Visual::kRest; break;
    case manual_reload_math::MagDraw::kHand: v.mag = Visual::kHand; break;
    case manual_reload_math::MagDraw::kHidden: v.mag = Visual::kHidden; break;
    default: v.mag = Visual::kGame; break;
  }
  std::memcpy(v.mag_rel, g_mag_rel, 64);
  v.lever = o.lever_override; v.lever_pull = o.lever_override ? g_lever_s : 0.f;
  const ULONGLONG now = GetTickCount64();
  if (v.active && v.mag == Visual::kRest && g_seat_ms && now - g_seat_ms < kSeatMs) {
    v.mag = Visual::kHand; blend(v.mag_rel, g_seat_from, g_rest_rel, smooth01(static_cast<float>(now - g_seat_ms) / kSeatMs));}
  if (v.active && v.lever && v.lever_pull == 0.f && g_lever_ms && now - g_lever_ms < kLeverBackMs)
    v.lever_pull = g_lever_from * (1.f - smooth01(static_cast<float>(now - g_lever_ms) / kLeverBackMs));
  // The rack pin: eased on over 120 ms at the grab, off over 150 ms after the release (the hand rides the handle back).
  if (g_pin_have) {
    float w = 0.f;
    if (g_pin_on_ms && !g_pin_off_ms) w = smooth01(static_cast<float>(now - g_pin_on_ms) / 120.f);
    else if (g_pin_off_ms && now - g_pin_off_ms < 150) w = 1.f - smooth01(static_cast<float>(now - g_pin_off_ms) / 150.f);
    if (w > 0.f && active) {
      v.pin_hand = g_pin_hand; v.pin_weight = w;
      std::memcpy(v.pin_rel, g_pin_rel, 64);
      for (int k = 0; k < 3; ++k) v.pin_rel[12 + k] += v.lever_pull * g_pin_dir[k];
    } else if (g_pin_off_ms) {g_pin_have = false;}
  }
  std::lock_guard<std::mutex> lock(g_visual_mutex); g_visual = v;
}
}  // namespace

bool* enabled_flag() { return &g_enabled; }
bool* pace_flag() { return &g_pace; }
bool* anim_pace_flag() { return &g_anim_pace; }
bool native_hand(int hand) { return hand >= 0 && g_native_hand.load(std::memory_order_relaxed) == hand; }
Visual visual() { std::lock_guard<std::mutex> lock(g_visual_mutex); return g_visual; }

void reset() {
  manual_reload_math::Input in; in.now_ms = GetTickCount64();
  const auto o = manual_reload_math::step(g_state, in);
  if (o.event == manual_reload_math::Event::kCancel) log("reload cancelled (left gameplay)");
  g_was_active = false; g_last_why = ""; g_have_offset = false; g_native_hand.store(-1); g_pin_have = false;
  publish(o, false);
}

Frame update(int weapon_hand, bool off_grip, bool reload_button, bool have_head, const float* off_head, bool haptics) {
  Frame f;
  const int off = 1 - weapon_hand;
  manual_reload_math::Input in;
  in.now_ms = GetTickCount64();
  in.grip = off_grip;
  in.button = reload_button;
  const int game_state = reload_probe::game_reload_state();
  in.game_reloading = game_state == 0;
  reload_probe::WeaponFrame wf;
  const char* why = "";
  if (!g_enabled) why = "menu off";
  else if (mount_camera::rider_view()) why = "riding";
  else if (!vr_weapon::weapon_placed_by_game()) why = "no weapon placed";
  else if (skin_palette_probe::hand_swap_active()) why = "hand swap";
  else if (!reload_probe::weapon_frame(wf)) why = "no weapon drawn in hand";
  else if (!wf.rifle) why = "not the assault rifle";
  else if (!wf.have_mag || !wf.have_lever) why = "magazine/handle rest not learned yet";
  in.active = !*why;
  if (in.active != g_was_active || (!in.active && std::strcmp(why, g_last_why) != 0)) {
    if (in.active) log("manual reload ready (off hand %s)", off == 0 ? "LEFT" : "RIGHT");
    else log("manual reload off: %s", why);
    g_was_active = in.active; g_last_why = why;
  }
  if (in.active) {
    float wrist[3];
    if (skin_palette_probe::support_wrist_world(off, wrist) && unit_axes(wf.root_world, in.axes)) {
      in.have_hand = true;
      for (int k = 0; k < 3; ++k) in.hand[k] = wrist[k];
      rest_point(wf.mag_rest, wf.root_world, in.mag_rest);
      rest_point(wf.lever_rest, wf.root_world, in.lever_rest);
    }
    in.support_d = skin_palette_probe::support_point_distance(off);
    in.have_support = skin_palette_probe::support_point_world(off, in.support_pt);
    in.handle_d = skin_palette_probe::handle_point_distance(off);
    g_hip_was = have_head && manual_reload_math::in_hip(off, off_head, g_hip_was);
    in.at_hip = g_hip_was;
  }
  // The held magazine rides the wrist rigidly (rotation too): offset taken at the grab (HEADSET 2026-11-19: following the
  // wrist position in the gun's axes looked disconnected when the hand turned).
  float wrist_m[16]; const bool have_wrist_m = in.active && skin_palette_probe::support_wrist_matrix(off, wrist_m);
  const auto held_phase = [](manual_reload_math::Phase ph) {
    return ph == manual_reload_math::Phase::kHolding || ph == manual_reload_math::Phase::kOut || ph == manual_reload_math::Phase::kNewMag;};
  if (have_wrist_m && g_have_offset && held_phase(g_state.phase)) {
    float held[16]; mul(held, g_hand_offset, wrist_m);
    in.have_held = true; for (int k = 0; k < 3; ++k) in.held[k] = held[12 + k];
  }
  const bool press = off_grip && !g_state.grip_was;
  const auto phase_before = g_state.phase;
  const auto o = manual_reload_math::step(g_state, in);
  if (o.event == manual_reload_math::Event::kGrab) {
    float rest_world[16]; mul(rest_world, wf.mag_rest, wf.root_world);
    float inv_wrist[16];
    g_have_offset = have_wrist_m && inverse_affine(inv_wrist, wrist_m);
    if (g_have_offset) mul(g_hand_offset, rest_world, inv_wrist);
    else log("no wrist rotation at the grab: the magazine follows the wrist position only");
  }
  {float inv_root[16]; if (in.active && inverse_affine(inv_root, wf.root_world)) std::memcpy(g_rest_rel, wf.mag_rest, 64);}
  if ((o.event == manual_reload_math::Event::kSeated || o.event == manual_reload_math::Event::kDone) &&
      phase_before == manual_reload_math::Phase::kNewMag) {std::memcpy(g_seat_from, g_mag_rel, 64); g_seat_ms = GetTickCount64();}
  if (phase_before == manual_reload_math::Phase::kRacking && g_state.phase != manual_reload_math::Phase::kRacking) {
    g_lever_from = g_prev_lever_pull; g_lever_ms = GetTickCount64();}
  {  // the handle's pull, smoothed with a 25 ms time constant (it moved in controller-jitter steps)
    const ULONGLONG now = GetTickCount64();
    const float dt = g_lever_s_ms ? static_cast<float>(now - g_lever_s_ms) : 0.f; g_lever_s_ms = now;
    if (g_state.phase != manual_reload_math::Phase::kRacking) g_lever_s = 0.f;
    else g_lever_s += (o.lever_pull - g_lever_s) * (1.f - std::exp(-dt / 25.f));
  }
  if (o.event == manual_reload_math::Event::kRackGrab) {
    // The wrist target on the handle, in the weapon socket's frame: the controller wrist's rotation at the grab, its
    // position at most 9 cm from the handle's rest point (the palm on the handle), sliding back with the pull.
    float sock[16], inv_sock[16];
    g_pin_have = false;
    if (have_wrist_m && vr_weapon::written_socket(sock, nullptr) && inverse_affine(inv_sock, sock)) {
      float lp[3]; rest_point(wf.lever_rest, wf.root_world, lp);
      float w[16]; std::memcpy(w, wrist_m, 64);
      float d[3], l = 0.f; for (int k = 0; k < 3; ++k) {d[k] = w[12 + k] - lp[k]; l += d[k] * d[k];}
      l = std::sqrt(l); const float s = l > 0.09f ? 0.09f / l : 1.f;
      for (int k = 0; k < 3; ++k) w[12 + k] = lp[k] + d[k] * s;
      mul(g_pin_rel, w, inv_sock);
      const float zl = std::sqrt(wf.root_world[8] * wf.root_world[8] + wf.root_world[9] * wf.root_world[9] + wf.root_world[10] * wf.root_world[10]);
      float zu[3]; for (int k = 0; k < 3; ++k) zu[k] = zl > 1e-6f ? -wf.root_world[8 + k] / zl : 0.f;  // back along the gun
      for (int k = 0; k < 3; ++k) g_pin_dir[k] = zu[0] * inv_sock[k] + zu[1] * inv_sock[4 + k] + zu[2] * inv_sock[8 + k];
      g_pin_hand = off; g_pin_have = true; g_pin_on_ms = GetTickCount64(); g_pin_off_ms = 0;
      char line[160]; std::snprintf(line, sizeof(line), "rack: off hand pinned to the charging handle (wrist was %.3f m from it)", l);
      reload_probe::log_text(line);
    } else log("rack: no wrist matrix or weapon socket: the hand is not pinned");
  }
  if (g_pin_have && !g_pin_off_ms && g_state.phase != manual_reload_math::Phase::kRacking) g_pin_off_ms = GetTickCount64();
  g_prev_lever_pull = g_lever_s;
  if (o.mag == manual_reload_math::MagDraw::kHand) {  // the held magazine in the gun's root frame (each draw has its own root)
    float held[16], inv_root[16];
    if (have_wrist_m && g_have_offset) mul(held, g_hand_offset, wrist_m);
    else {mul(held, wf.mag_rest, wf.root_world); for (int k = 0; k < 3; ++k) held[12 + k] = o.mag_world[k];}
    if (inverse_affine(inv_root, wf.root_world)) mul(g_mag_rel, held, inv_root);
  }
  // Stage 2: the hands pace the game's reload (its timer end, author approval 2026-11-19).
  const manual_reload_math::Pace pc = manual_reload_math::pace(g_state, in);
  if (g_pace && pc.active) {
    long long now_c = 0, end = 0, dur = 0;
    if (reload_probe::reload_timer(now_c, end, dur)) {
      const float p = 1.f - static_cast<float>(end - now_c) / static_cast<float>(dur);
      const float t = manual_reload_math::paced(p, pc);
      if (std::fabs(t - p) > 0.004f) {
        const long long want = now_c + static_cast<long long>((1.f - t) * static_cast<float>(dur));
        if (reload_probe::write_reload_end(want)) ++g_writes;
        else if (++g_write_fails <= 5) log("reload pace: timer write refused (not reloading or duration changed)");
      }
      if (pc.jump != g_last_jump || pc.hold != g_last_hold) {
        char line[200];
        std::snprintf(line, sizeof(line), "reload pace: progress %.2f -> keep %.2f..%.2f (%u writes so far)", p, pc.jump, pc.hold, g_writes);
        reload_probe::log_text(line);
        g_last_jump = pc.jump; g_last_hold = pc.hold;
      }
    }
  } else if (!pc.active) {g_last_jump = g_last_hold = -1.f;}
  // Experiment (author approval 2026-11-19): the reload animation (and the sounds on its clip) follow the hands. [LIVE
  // 2026-11-19] it played the whole clip on its own clock while the timer was held. Its length (the graph's
  // ToolReloadingTime, comp+0x3F38) is the game's own while the clip should run and very long while it should wait. OPEN:
  // whether the graph plays at length-scaled speed (holds) or maps elapsed / length (would jump back): the bone log tells.
  {
    long long cur = 0;
    const bool ours = g_pace && g_anim_pace && pc.active && reload_probe::reload_anim_time(cur);
    const ULONGLONG now = GetTickCount64();
    if (ours) {
      long long nc = 0, e = 0, d = 0;
      const bool have_t = reload_probe::reload_timer(nc, e, d) && d > 0;
      const float p = have_t ? 1.f - static_cast<float>(e - nc) / static_cast<float>(d) : 0.f;
      if (!g_anim_orig) {g_anim_orig = cur < 60000 ? cur : 3300; g_anim_ms = now; g_anim_mode = -1; g_anim_a = p;}
      g_anim_a += static_cast<float>(now - g_anim_ms) / static_cast<float>(cur > 0 ? cur : g_anim_orig); g_anim_ms = now;
      float t = have_t ? p : g_anim_a;
      // an automatic reload waits before the magazine moves (clip 0.15) until the hand takes it
      if (g_state.phase == manual_reload_math::Phase::kIdle && g_state.empty) t = std::fmin(t, 0.10f);
      int mode; long long want;
      if (g_anim_a < t - 0.08f) {mode = 2; want = g_anim_orig / 2;}       // well behind: catch up at double speed
      else if (g_anim_a < t - 0.005f) {mode = 1; want = g_anim_orig;}     // play at the game's speed
      else {mode = 0; want = g_anim_orig * 1000;}                          // hold
      if (want != cur) {
        if (reload_probe::write_reload_anim_time(want)) ++g_anim_writes;
        else if (++g_anim_fails <= 5) log("reload anim: length write refused");
      }
      if (mode != g_anim_mode) {
        char line[200];
        std::snprintf(line, sizeof(line), "reload anim: %s at clip %.2f (progress %.2f, length %lld ms, %u writes)",
                      mode == 0 ? "holds" : mode == 1 ? "plays" : "catches up", g_anim_a, t, want, g_anim_writes);
        reload_probe::log_text(line); g_anim_mode = mode;
      }
    } else if (g_anim_orig) {
      if (reload_probe::reload_anim_time(cur) && cur != g_anim_orig) reload_probe::write_reload_anim_time(g_anim_orig);
      g_anim_orig = 0; g_anim_mode = -1;
    }
  }
  // While the off hand holds the magazine or the handle, its fingers are the game's own (the first-person animation keeps
  // that hand closed on the rifle's front grip, or on the magazine while the game's reload animation plays) instead of
  // the controller gestures (HEADSET 2026-11-19: no holding pose).
  g_native_hand.store(in.active && (g_state.phase == manual_reload_math::Phase::kHolding || g_state.phase == manual_reload_math::Phase::kOut ||
                      g_state.phase == manual_reload_math::Phase::kNewMag || g_state.phase == manual_reload_math::Phase::kRacking) ? off : -1,
                      std::memory_order_relaxed);
  if (press && in.active && in.have_hand) {  // tuning: where the grip was, in the gun's axes (x right, y up, z forward)
    float v[3], m[3], l[3], sp[3]{};
    for (int k = 0; k < 3; ++k) v[k] = in.hand[k] - in.mag_rest[k];
    manual_reload_math::to_local(in.axes, v, m);
    for (int k = 0; k < 3; ++k) v[k] = in.hand[k] - in.lever_rest[k];
    manual_reload_math::to_local(in.axes, v, l);
    if (in.have_support) {for (int k = 0; k < 3; ++k) v[k] = in.hand[k] - in.support_pt[k]; manual_reload_math::to_local(in.axes, v, sp);}
    char line[400];
    std::snprintf(line, sizeof(line), "off-hand grip (phase %d -> %d): wrist - magazine (%.2f %.2f %.2f) %.2f m, wrist - handle "
                  "(%.2f %.2f %.2f) %.2f m, wrist - front grip (%.2f %.2f %.2f)%s, hip %d",
                  static_cast<int>(phase_before), static_cast<int>(g_state.phase), m[0], m[1], m[2],
                  manual_reload_math::dist3(in.hand, in.mag_rest), l[0], l[1], l[2], manual_reload_math::dist3(in.hand, in.lever_rest),
                  sp[0], sp[1], sp[2], in.have_support ? "" : " (unknown)", in.at_hip ? 1 : 0);
    reload_probe::log_text(line);
  }
  if (press && in.active && g_state.phase == manual_reload_math::Phase::kEmptyHand && !in.at_hip && have_head)
    log("off-hand grip away from the hip%s (head frame %.2f %.2f %.2f m): no magazine", "", off_head[0], off_head[1], off_head[2]);
  if (o.event != manual_reload_math::Event::kNone) {
    log("reload: %s (game reload state %.0f)", manual_reload_math::event_name(o.event), game_state);
    if (haptics) {
      using E = manual_reload_math::Event;
      switch (o.event) {
        case E::kGrab: case E::kFetched: case E::kRackGrab: xr_input::pulse(off, 0.04f, 0.45f); break;
        case E::kEjected: xr_input::pulse(off, 0.05f, 0.6f); xr_input::pulse(weapon_hand, 0.04f, 0.3f); break;
        case E::kSeated: case E::kDone: case E::kRacked:
          xr_input::pulse(off, 0.07f, 0.8f); xr_input::pulse(weapon_hand, 0.06f, 0.6f); break;
        case E::kAutoReload: xr_input::pulse(weapon_hand, 0.05f, 0.5f); break;
        default: break;
      }
    }
  }
  if (o.press_x) log("reload: game reload started (pad X)");
  publish(o, in.active);
  f.press_x = o.press_x; f.block_fire = o.block_fire; f.owns_grip = o.owns_grip;
  return f;
}
}  // namespace manual_reload
