#include "../runtime_log.hpp"
#include "reload_probe.hpp"
#include "manual_reload.hpp"
#include "../game/player.hpp"
#include <atomic>
#include <windows.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace reload_probe {
namespace {
bool g_probe = true;

void log_line(const char* fmt, ...) {
  static std::wstring path;
  if (path.empty()) {
    wchar_t module_path[MAX_PATH]{}; HMODULE self = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&log_line), &self);
    GetModuleFileNameW(self, module_path, MAX_PATH);
    path = module_path;
    const auto slash = path.find_last_of(L"\\/");
    path = path.substr(0, slash == std::wstring::npos ? 0 : slash + 1) + L"VR_RELOAD.log";
  }
  char line[1024];
  va_list args; va_start(args, fmt); std::vsnprintf(line, sizeof(line), fmt, args); va_end(args);
  SYSTEMTIME time{};
  GetLocalTime(&time);
  char stamp[24];
  std::snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u.%03u ", time.wHour, time.wMinute, time.wSecond, time.wMilliseconds);
  runtime_log::append(path, {stamp, line});
}

// ---- weapon bones (render thread writes, poll thread reads)
constexpr int kBones = 4;
const char* const kBoneNames[kBones] = {"C_magazine", "C_reloadLevel", "C_magazine_End", "C_charge"};
struct Draw {
  char weapon[48]{};
  bool have[kBones]{};
  float rel[kBones][16]{};   // bone in the C_root frame (row vectors: rel = bone * inverse(root))
  float root_world[16]{};    // in-hand draw only
  ULONGLONG tick = 0, root_tick = 0;
};
std::mutex g_mutex;
Draw g_draw;
long long g_frame_us = 0; float g_frame_best = 1e9f;  // g_mutex: this frame's chosen draw

bool inverse_affine(float (&o)[16], const float* m) {
  const float a = m[0], b = m[1], c = m[2], d = m[4], e = m[5], f = m[6], g = m[8], h = m[9], i = m[10];
  const float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const float det = a * A + b * B + c * C;
  if (!std::isfinite(det) || std::fabs(det) < 1e-12f) return false;
  const float s = 1.f / det;
  float r[9] = {A * s, -(b * i - c * h) * s, (b * f - c * e) * s,
                B * s, (a * i - c * g) * s, -(a * f - c * d) * s,
                C * s, -(a * h - b * g) * s, (a * e - b * d) * s};
  std::memset(o, 0, sizeof(o));
  for (int row = 0; row < 3; ++row) for (int k = 0; k < 3; ++k) o[row * 4 + k] = r[row * 3 + k];
  for (int k = 0; k < 3; ++k) o[12 + k] = -(m[12] * r[k] + m[13] * r[3 + k] + m[14] * r[6 + k]);
  o[15] = 1.f;
  return true;
}
void multiply(float (&o)[16], const float* a, const float* b) {  // o = a * b (row vectors: a first)
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
    float s = 0.f; for (int k = 0; k < 4; ++k) s += a[r * 4 + k] * b[k * 4 + c]; o[r * 4 + c] = s;}
}
// Angle (deg) between the rotation parts of two matrices (scale removed per row).
float angle_between(const float* a, const float* b) {
  float tr = 0.f;
  for (int r = 0; r < 3; ++r) {
    const float la = std::sqrt(a[r * 4] * a[r * 4] + a[r * 4 + 1] * a[r * 4 + 1] + a[r * 4 + 2] * a[r * 4 + 2]);
    const float lb = std::sqrt(b[r * 4] * b[r * 4] + b[r * 4 + 1] * b[r * 4 + 1] + b[r * 4 + 2] * b[r * 4 + 2]);
    if (la < 1e-6f || lb < 1e-6f) return -1.f;
    tr += (a[r * 4] * b[r * 4] + a[r * 4 + 1] * b[r * 4 + 1] + a[r * 4 + 2] * b[r * 4 + 2]) / (la * lb);
  }
  const float c = std::fmax(-1.f, std::fmin(1.f, (tr - 1.f) * 0.5f));
  return std::acos(c) * 57.29578f;
}

// ---- reload fields (poll thread only)
struct Fields {
  int state = -1; std::int64_t duration = 0, end = 0, remaining = 0; std::uintptr_t clock_fn = 0;
  std::int64_t dur3f38 = 0, d3fa0 = 0, d3fa8 = 0, d3fb0 = 0, d3fb8 = 0;
  unsigned char f3f40 = 0, f3f65 = 0, f3f66 = 0, f3f67 = 0, f3f68 = 0;
  int net_state = -1;
};
bool read_fields(Fields& f) {
  const game::Pointers gp = game::pointers();
  if (!gp.body) return false;
  unsigned char* comp = nullptr;
  if (!game::mem_read(gp.body + 0x250, &comp, sizeof(comp)) || !comp) return false;
  unsigned char timer[0x28]{};
  if (!game::mem_read(comp + 0x3EA8, timer, sizeof(timer))) return false;
  std::memcpy(&f.state, timer, 4); std::memcpy(&f.duration, timer + 8, 8); std::memcpy(&f.end, timer + 0x10, 8);
  std::memcpy(&f.remaining, timer + 0x18, 8); std::memcpy(&f.clock_fn, timer + 0x20, 8);
  unsigned char block[0x3FC0 - 0x3F38]{};
  if (!game::mem_read(comp + 0x3F38, block, sizeof(block))) return false;
  auto i64 = [&](unsigned off) {std::int64_t v; std::memcpy(&v, block + (off - 0x3F38), 8); return v;};
  auto u8 = [&](unsigned off) {return block[off - 0x3F38];};
  f.dur3f38 = i64(0x3F38); f.f3f40 = u8(0x3F40);
  f.f3f65 = u8(0x3F65); f.f3f66 = u8(0x3F66); f.f3f67 = u8(0x3F67); f.f3f68 = u8(0x3F68);
  f.d3fa0 = i64(0x3FA0); f.d3fa8 = i64(0x3FA8); f.d3fb0 = i64(0x3FB0); f.d3fb8 = i64(0x3FB8);
  unsigned char* net = nullptr;
  if (game::mem_read(comp + 0x3FE8, &net, sizeof(net)) && net) game::mem_read(net + 0x30, &f.net_state, 4);
  return true;
}
// A deadline as a fraction of the timer (start = end - duration). Meaningful only if both use one clock (OPEN).
double frac(std::int64_t deadline, const Fields& f) {
  if (f.duration <= 0 || deadline == 0 || deadline == INT64_MAX) return -1.0;
  return static_cast<double>(deadline - (f.end - f.duration)) / static_cast<double>(f.duration);
}

// The player's animation-state object (vtable RVA 0x633D270, STATIC 2026-11-19: binder 0x143E2B610 registers its graph
// variables, 0x143E8CAB0 pushes them each frame; each slot = name +0x8, graph index +0x18, value +0x1C). Found once
// from the player body by its vtable (read-only scan), then re-checked every tick.
constexpr std::uintptr_t kAnimStateVtRva = 0x633D270;
struct AnimVars { bool ok = false; float reload_time = 0.f, reload_progress = 0.f, weapon_reload_time = 0.f, chamber_time = 0.f;
                  unsigned char is_reloading = 0, abort = 0, out_of_ammo = 0, unload = 0, chambering = 0; };
unsigned char* g_anim_obj = nullptr; ULONGLONG g_anim_next_scan = 0; unsigned g_anim_scans = 0;
bool is_anim_state(unsigned char* p, std::uintptr_t vt) {
  std::uintptr_t v = 0;
  return p && game::mem_read(p, &v, sizeof(v)) && v == vt;
}
unsigned char* scan_for(unsigned char* base, std::size_t bytes, std::uintptr_t vt, int depth) {
  static std::uintptr_t buf[2][0x2000 / 8];
  if (!base || depth > 1) return nullptr;
  const std::size_t n = bytes / 8;
  if (!game::mem_read(base, buf[depth], n * 8)) return nullptr;
  for (std::size_t i = 0; i < n; ++i) {
    const std::uintptr_t q = buf[depth][i];
    if (q < 0x10000 || q > 0x7FFFFFFFFFFFull || (q & 7)) continue;
    unsigned char* p = reinterpret_cast<unsigned char*>(q);
    if (is_anim_state(p, vt)) return p;
  }
  if (depth == 0)
    for (std::size_t i = 0; i < n; ++i) {
      const std::uintptr_t q = buf[0][i];
      if (q < 0x10000 || q > 0x7FFFFFFFFFFFull || (q & 7)) continue;
      if (unsigned char* hit = scan_for(reinterpret_cast<unsigned char*>(q), 0x400, vt, 1)) return hit;
    }
  return nullptr;
}
AnimVars read_anim_vars() {
  AnimVars a;
  const game::Pointers gp = game::pointers();
  if (!gp.body) return a;
  const std::uintptr_t vt = reinterpret_cast<std::uintptr_t>(game::module_base()) + kAnimStateVtRva;
  if (!is_anim_state(g_anim_obj, vt)) {
    g_anim_obj = nullptr;
    const ULONGLONG now = GetTickCount64();
    if (now < g_anim_next_scan) return a;
    g_anim_next_scan = now + (g_anim_scans < 10 ? 3000 : 30000); ++g_anim_scans;
    unsigned char* anim = nullptr; game::mem_read(gp.body + 0x1A8, &anim, sizeof(anim));
    g_anim_obj = scan_for(gp.body, 0x2000, vt, 0);
    if (!g_anim_obj && anim) g_anim_obj = scan_for(anim, 0x2000, vt, 0);
    char line[160]; std::snprintf(line, sizeof(line), "anim state object (vt rva 0x633D270): %s %p (scan %u)",
                                  g_anim_obj ? "found" : "not found", static_cast<void*>(g_anim_obj), g_anim_scans);
    log_line("%s", line);
    if (!g_anim_obj) return a;
  }
  unsigned char* o = g_anim_obj;
  a.ok = game::mem_read(o + 0x1CBC, &a.reload_time, 4) && game::mem_read(o + 0x1CDC, &a.reload_progress, 4) &&
         game::mem_read(o + 0x1C7C, &a.is_reloading, 1);
  game::mem_read(o + 0x399C, &a.weapon_reload_time, 4);
  game::mem_read(o + 0x3A7C, &a.abort, 1);
  return a;
}

Fields g_prev;
bool g_have_prev = false, g_clock_logged = false;
ULONGLONG g_start = 0, g_ended = 0, g_last_line = 0;
float g_rest[kBones][16]{}; bool g_rest_have[kBones]{};
LARGE_INTEGER g_qpf{};
// Rest learning: a candidate pose per bone must hold within 2 mm / 2 deg for 300 ms (the first reload of the 2026-11-19
// run started from a rest taken mid-animation: C_magazine 14 cm and 91 deg off).
float g_cand[kBones][16]{}; ULONGLONG g_cand_since[kBones]{};
std::mutex g_rest_mutex;  // g_rest / g_rest_have, read by the render thread and the controls
std::atomic<int> g_state{-1};
struct TimerView { int state = -1; long long end = 0, duration = 0, start_clock = 0; ULONGLONG start_tick = 0, read_tick = 0; bool have_start = false; };
std::mutex g_timer_mutex; TimerView g_timer;
}  // namespace

bool* probe_flag() { return &g_probe; }
bool capture_on() { return g_probe || *manual_reload::enabled_flag(); }
void log_text(const char* line) { log_line("%s", line); }
int game_reload_state() { return g_state.load(std::memory_order_relaxed); }
bool reload_timer(long long& now_clock, long long& end, long long& duration) {
  std::lock_guard<std::mutex> lock(g_timer_mutex);
  if (g_timer.state != 0 || !g_timer.have_start || g_timer.duration <= 0) return false;
  now_clock = g_timer.start_clock + static_cast<long long>(GetTickCount64() - g_timer.start_tick);
  end = g_timer.end; duration = g_timer.duration;
  return true;
}
// Author approval 2026-11-19: the hands pace the game's reload. Only the timer's end (+0x10) is written, through the
// chain re-read every time, and only while the timer runs (state 0) with the duration that was read.
bool write_reload_end(long long end) {
  const game::Pointers gp = game::pointers();
  if (!gp.body) return false;
  unsigned char* comp = nullptr;
  if (!game::mem_read(gp.body + 0x250, &comp, sizeof(comp)) || !comp) return false;
  int state = -1; long long duration = 0;
  if (!game::mem_read(comp + 0x3EA8, &state, 4) || state != 0) return false;
  if (!game::mem_read(comp + 0x3EA8 + 8, &duration, 8)) return false;
  {std::lock_guard<std::mutex> lock(g_timer_mutex); if (duration != g_timer.duration) return false;}
  if (!game::mem_write(comp + 0x3EA8 + 0x10, &end, 8)) return false;
  std::lock_guard<std::mutex> lock(g_timer_mutex); g_timer.end = end;
  return true;
}
// [STATIC 0x1427BC290 / 0x142839AE0 / 0x144275xxx] the animation graph's ToolReloadingTime / WeaponReloadTime =
// comp+0x3F38 (int64 ms) while the byte comp+0x3F40 is set (it is during a reload, LIVE 3300); the game recomputes it only
// when its own reload-speed condition changes. [LIVE 2026-11-19] the reload clip and its sounds play on their own clock
// from the start, ignoring the timer's progress.
bool reload_anim_time(long long& ms) {
  const game::Pointers gp = game::pointers();
  if (!gp.body) return false;
  unsigned char* comp = nullptr;
  if (!game::mem_read(gp.body + 0x250, &comp, sizeof(comp)) || !comp) return false;
  int state = -1; unsigned char set = 0;
  if (!game::mem_read(comp + 0x3EA8, &state, 4) || state != 0) return false;
  if (!game::mem_read(comp + 0x3F40, &set, 1) || !set) return false;
  return game::mem_read(comp + 0x3F38, &ms, 8);
}
bool write_reload_anim_time(long long ms) {
  if (ms < 100 || ms > 100000000) return false;
  const game::Pointers gp = game::pointers();
  if (!gp.body) return false;
  unsigned char* comp = nullptr;
  if (!game::mem_read(gp.body + 0x250, &comp, sizeof(comp)) || !comp) return false;
  int state = -1; unsigned char set = 0; long long duration = 0;
  if (!game::mem_read(comp + 0x3EA8, &state, 4) || state != 0) return false;
  if (!game::mem_read(comp + 0x3F40, &set, 1) || !set) return false;
  if (!game::mem_read(comp + 0x3EA8 + 8, &duration, 8)) return false;
  {std::lock_guard<std::mutex> lock(g_timer_mutex); if (duration != g_timer.duration) return false;}
  return game::mem_write(comp + 0x3F38, &ms, 8);
}
bool weapon_frame(WeaponFrame& out) {
  Draw d; {std::lock_guard<std::mutex> lock(g_mutex); d = g_draw;}
  const ULONGLONG now = GetTickCount64();
  if (!d.root_tick || now - d.root_tick > 200 || !d.tick || now - d.tick > 200) return false;
  out = WeaponFrame{};
  out.rifle = !std::strcmp(d.weapon, "rifle");
  std::memcpy(out.root_world, d.root_world, 64);
  std::lock_guard<std::mutex> lock(g_rest_mutex);
  out.have_mag = g_rest_have[0]; out.have_lever = g_rest_have[1];
  std::memcpy(out.mag_rest, g_rest[0], 64); std::memcpy(out.lever_rest, g_rest[1], 64);
  return true;
}

void note_weapon_draw(const char* path, const char* const* names, const float (*mats)[16], unsigned n,
                      const std::int16_t (&cells)[3], float socket_d) {
  if (!capture_on() || !path || n < 2) return;
  // [LIVE 2026-11-19] the rifle is drawn 6-8 times a frame (~560 in-hand draws/s, ~350 others: eyes, shadows, the
  // third-person shadow body). Taking the last one made the gun frame jump (a rack registered 8 ms after the grip).
  // Per frame (8 ms window) only the draw whose root is nearest the tracked weapon socket counts.
  if (socket_d < 0.f || socket_d >= 0.2f) return;
  {LARGE_INTEGER q, fq; QueryPerformanceCounter(&q); QueryPerformanceFrequency(&fq);
    const long long us = static_cast<long long>(q.QuadPart * 1000000.0 / static_cast<double>(fq.QuadPart));
    std::lock_guard<std::mutex> lock(g_mutex);
    const bool new_frame = !g_frame_us || us - g_frame_us > 8000;
    if (!new_frame && socket_d >= g_frame_best) return;
    if (new_frame) g_frame_us = us;
    g_frame_best = socket_d;}
  const char* kind = std::strstr(path, "ASSAULT_RIFLE") ? "rifle" : std::strstr(path, "SHOTGUN") ? "shotgun" : nullptr;
  if (!kind) return;
  float inv_root[16];
  if (!inverse_affine(inv_root, mats[0])) return;
  Draw d; std::snprintf(d.weapon, sizeof(d.weapon), "%s", kind);
  for (unsigned i = 1; i < n; ++i) {
    if (!names[i]) continue;
    for (int b = 0; b < kBones; ++b)
      if (!std::strcmp(names[i], kBoneNames[b])) {multiply(d.rel[b], mats[i], inv_root); d.have[b] = true;}
  }
  const ULONGLONG now = GetTickCount64();
  d.tick = now;
  std::lock_guard<std::mutex> lock(g_mutex);
  std::memcpy(d.root_world, mats[0], 64);
  for (int k = 0; k < 3; ++k) d.root_world[12 + k] += 1024.f * cells[k];
  d.root_tick = now;
  g_draw = d;
}

void tick() {
  if (!capture_on()) return;
  Fields f;
  if (!read_fields(f)) {g_state.store(-1, std::memory_order_relaxed); return;}
  g_state.store(f.state, std::memory_order_relaxed);
  {std::lock_guard<std::mutex> lock(g_timer_mutex);
    const ULONGLONG t = GetTickCount64();
    // [LIVE 2026-11-19] the timer clock is milliseconds and runs 1:1 with wall time: at the start (state 1 -> 0) the clock
    // is end - duration; from then on clock = that + elapsed ticks. Writing end later does not move the clock.
    if (f.state == 0 && (g_timer.state != 0 || !g_timer.have_start)) {
      g_timer.start_clock = f.end - f.duration; g_timer.start_tick = t; g_timer.have_start = true;}
    if (f.state != 0) g_timer.have_start = false;
    g_timer.state = f.state; g_timer.end = f.end; g_timer.duration = f.duration; g_timer.read_tick = t;}
  const ULONGLONG now = GetTickCount64();
  if (!g_qpf.QuadPart) QueryPerformanceFrequency(&g_qpf);
  Draw d; {std::lock_guard<std::mutex> lock(g_mutex); d = g_draw;}
  const bool fresh = d.tick && now - d.tick < 200;
  const bool reloading = f.state == 0;

  // Rest pose of the weapon bones: learned while not reloading, from a pose held still for 300 ms.
  if (f.state != 0 && fresh && (!g_ended || now - g_ended > 1500)) {
    std::lock_guard<std::mutex> lock(g_rest_mutex);
    for (int b = 0; b < kBones; ++b) {
      if (!d.have[b]) continue;
      float moved = 0.f; for (int k = 0; k < 3; ++k) moved += (d.rel[b][12 + k] - g_cand[b][12 + k]) * (d.rel[b][12 + k] - g_cand[b][12 + k]);
      const float turned = angle_between(d.rel[b], g_cand[b]);
      if (!g_cand_since[b] || std::sqrt(moved) > 0.002f || turned < 0.f || turned > 2.f) {std::memcpy(g_cand[b], d.rel[b], 64); g_cand_since[b] = now;}
      else if (now - g_cand_since[b] >= 300) {std::memcpy(g_rest[b], g_cand[b], 64); g_rest_have[b] = true;}
    }
  }
  if (!reloading && g_have_prev && g_prev.state == 0) g_ended = now;
  if (!g_probe) {g_prev = f; g_have_prev = true; return;}

  if (!g_clock_logged && f.clock_fn) {
    const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(game::module_base());
    log_line("reload probe on: timer clock fn rva 0x%llX (comp+0x3EA8+0x20)",
             static_cast<unsigned long long>(f.clock_fn - base));
    g_clock_logged = true;
  }

  const bool changed = !g_have_prev || f.state != g_prev.state || f.f3f66 != g_prev.f3f66 || f.f3f65 != g_prev.f3f65 ||
                       f.net_state != g_prev.net_state || f.f3f67 != g_prev.f3f67 || f.f3f68 != g_prev.f3f68;
  // (+0x3FA0..+0x3FB8 are not deadlines in this object and change every frame: LIVE 2026-11-19, not a change trigger)
  if (reloading && (!g_have_prev || g_prev.state != 0)) {g_start = now; g_ended = 0;
    log_line("---- reload start (%s held, empty flag %u)", fresh ? d.weapon : "no weapon draw", f.f3f66);}
  if (!reloading && g_have_prev && g_prev.state == 0) {g_ended = now;
    log_line("---- reload end after %llu ms (state %d)", static_cast<unsigned long long>(now - g_start), f.state);}
  const bool window = reloading || (g_ended && now - g_ended <= 1500);
  const AnimVars av = (window || !g_anim_obj) ? read_anim_vars() : AnimVars{};
  if (changed || (window && now - g_last_line >= 33)) {
    LARGE_INTEGER q; QueryPerformanceCounter(&q);
    std::string bones;
    if (fresh)
      for (int b = 0; b < kBones; ++b) {
        if (!d.have[b]) continue;
        char s[128];
        if (g_rest_have[b]) {
          float dt[3]; for (int k = 0; k < 3; ++k) dt[k] = (d.rel[b][12 + k] - g_rest[b][12 + k]) * 100.f;
          std::snprintf(s, sizeof(s), " %s d=(%.1f %.1f %.1f)cm rot=%.0fdeg", kBoneNames[b], dt[0], dt[1], dt[2],
                        angle_between(d.rel[b], g_rest[b]));
        } else {
          std::snprintf(s, sizeof(s), " %s at=(%.1f %.1f %.1f)cm (no rest)", kBoneNames[b], d.rel[b][12] * 100.f,
                        d.rel[b][13] * 100.f, d.rel[b][14] * 100.f);
        }
        bones += s;
      }
    log_line("t=%lld ms qpc_ms=%.1f state=%d dur=%lld end=%lld rem=%lld | 3F38=%lld 3F40=%u 3F65=%u empty3F66=%u 3F67=%u 3F68=%u "
             "net=%d | anim%s IsReloading=%u ToolReloadingTime=%.3f ToolReloadProgress=%.3f WeaponReloadTime=%.3f ShouldAbortReload=%u |%s%s",
             g_start ? static_cast<long long>(now - g_start) : -1LL,
             static_cast<double>(q.QuadPart) * 1000.0 / static_cast<double>(g_qpf.QuadPart ? g_qpf.QuadPart : 1),
             f.state, static_cast<long long>(f.duration), static_cast<long long>(f.end),
             static_cast<long long>(f.remaining), static_cast<long long>(f.dur3f38), f.f3f40, f.f3f65, f.f3f66, f.f3f67,
             f.f3f68, f.net_state, av.ok ? "" : "(unread)", av.is_reloading, av.reload_time, av.reload_progress,
             av.weapon_reload_time, av.abort, fresh ? "" : " (no fresh weapon draw)", bones.c_str());
    g_last_line = now;
  }
  g_prev = f; g_have_prev = true;
}
}  // namespace reload_probe
