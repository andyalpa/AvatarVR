#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>

// Laser pointer for the game's menus: either controller's aim ray drives the native virtual cursor, its trigger is pad A.
// XR worker: step() + publish(). Game's cursor updater (ui_cursor.cpp): cursor(). XInput poll (virtual_pad.cpp): apply().
// "nolaser" in [native] turns it all off; the left stick and pad buttons then work as before.
namespace menu_pointer {

// Ray from (o, unit d) against a quad at pose (q = x,y,z,w; p), size w x h, normal +z. u is 0..1 left to right, v_up is
// 0..1 bottom to top. Only the side the panel faces counts, as a ray from behind it cannot be a menu click.
inline void rotate(const float* q, const float* v, float* out) {
  const float t[3] = {2.f * (q[1] * v[2] - q[2] * v[1]), 2.f * (q[2] * v[0] - q[0] * v[2]), 2.f * (q[0] * v[1] - q[1] * v[0])};
  out[0] = v[0] + q[3] * t[0] + q[1] * t[2] - q[2] * t[1];
  out[1] = v[1] + q[3] * t[1] + q[2] * t[0] - q[0] * t[2];
  out[2] = v[2] + q[3] * t[2] + q[0] * t[1] - q[1] * t[0];
}
inline bool hit_quad(const float* o, const float* d, const float* q, const float* p, float w, float h, float* u, float* v_up,
                     float* dist) {
  if (!(w > 0.f) || !(h > 0.f)) return false;
  const float ax[3] = {1.f, 0.f, 0.f}, ay[3] = {0.f, 1.f, 0.f}, az[3] = {0.f, 0.f, 1.f};
  float x[3], y[3], n[3];
  rotate(q, ax, x); rotate(q, ay, y); rotate(q, az, n);
  const float denom = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
  if (!(denom < -1e-4f)) return false;
  const float rel[3] = {p[0] - o[0], p[1] - o[1], p[2] - o[2]};
  const float t = (rel[0] * n[0] + rel[1] * n[1] + rel[2] * n[2]) / denom;
  if (!(t > 0.f) || !std::isfinite(t)) return false;
  const float at[3] = {o[0] + d[0] * t - p[0], o[1] + d[1] * t - p[1], o[2] + d[2] * t - p[2]};
  const float a = (at[0] * x[0] + at[1] * x[1] + at[2] * x[2]) / w + 0.5f;
  const float b = (at[0] * y[0] + at[1] * y[1] + at[2] * y[2]) / h + 0.5f;
  if (!(a >= 0.f && a <= 1.f && b >= 0.f && b <= 1.f)) return false;
  *u = a; *v_up = b; *dist = t;
  return true;
}

// Which hand points and when its trigger is A. Pulling the other hand's trigger hands the pointer over without a click, so
// the cursor never jumps and selects at once. A click starts only on the panel; a held A lasts until the trigger lets go.
struct Selector {
  int active = 1;  // 0 left, 1 right
  bool down[2] = {false, false};
  bool clicking = false, swallow = false;
};
struct Selection { int hand; bool hit, a, press, suppress; };
inline Selection step(Selector& s, const bool hit[2], const float trigger[2]) {
  bool rising[2];
  for (int i = 0; i < 2; ++i) {
    const bool now = s.down[i] ? trigger[i] > 0.4f : trigger[i] > 0.6f;
    rising[i] = now && !s.down[i];
    s.down[i] = now;
  }
  const int other = 1 - s.active;
  if (!s.clicking && hit[other] && (rising[other] || !hit[s.active])) {
    s.swallow = rising[other];
    s.active = other;
    rising[s.active] = false;
  }
  if (!s.down[s.active]) s.swallow = false;
  const bool press = rising[s.active] && hit[s.active] && !s.swallow;
  if (press) s.clicking = true;
  if (!s.down[s.active]) s.clicking = false;
  return {s.active, hit[s.active], s.clicking, press, hit[0] || hit[1] || s.clicking || s.swallow};
}

inline std::atomic<bool> g_enabled{true};
inline std::atomic<float> g_u{0.f}, g_v{0.f};
inline std::atomic<bool> g_hit{false}, g_a{false}, g_suppress{false};
inline std::atomic<std::uint64_t> g_at{0};
inline std::atomic<std::uint64_t> g_native_cursor_at{0};  // the updater last ran with the virtual cursor mode on
// The cursor last rested on a widget that can be hovered. A loading screen should have none, so the beam, the dot and the
// trigger wait for one, while the cursor keeps following the ray to find it. A click starts the wait over, as it may load.
inline std::atomic<std::uint64_t> g_hover_at{0};
constexpr std::uint64_t kHoverHoldMs = 4000;
inline bool interactive(std::uint64_t now) { return now - g_hover_at.load(std::memory_order_relaxed) < kHoverHoldMs; }

inline void publish(bool hit, float u, float v, bool a, bool suppress, std::uint64_t now) {
  g_u.store(u, std::memory_order_relaxed); g_v.store(v, std::memory_order_relaxed);
  g_hit.store(hit, std::memory_order_relaxed); g_a.store(a, std::memory_order_relaxed);
  g_suppress.store(suppress, std::memory_order_relaxed); g_at.store(now, std::memory_order_relaxed);
}
inline bool fresh(std::uint64_t now) { return now - g_at.load(std::memory_order_relaxed) < 250; }
inline bool cursor(float* u, float* v, std::uint64_t now) {
  if (!fresh(now) || !g_hit.load(std::memory_order_relaxed)) return false;
  *u = g_u.load(std::memory_order_relaxed); *v = g_v.load(std::memory_order_relaxed);
  return true;
}
// lt/rt/buttons of a pad state. Off the panel the triggers stay LT/RT, for the menus' tab keys.
template <class Pad> void apply(Pad* pad, std::uint64_t now) {
  if (!fresh(now)) return;
  if (g_suppress.load(std::memory_order_relaxed)) pad->lt = pad->rt = 0.f;
  if (g_a.load(std::memory_order_relaxed)) pad->buttons |= 0x1000;  // XINPUT_GAMEPAD_A
}

}  // namespace menu_pointer
