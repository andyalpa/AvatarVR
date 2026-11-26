// vr_recoil: see vr_recoil.hpp.
#include "../runtime_log.hpp"
#include "vr_recoil.hpp"
#include "vr_recoil_math.hpp"
#include "vr_weapon.hpp"
#include "xr_input.hpp"
#include "../diagnostics/skin_arm_ik.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../camera/mount_camera.hpp"
#include "../game/player.hpp"
#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace {
namespace rm = vr_recoil_math;

bool g_enabled = true;  // AUTHOR 2026-11-17 test: expected to see it without hunting for the toggle
float g_strength = 1.f;
bool g_haptic = false;
float g_signal = 0.f;
float g_reference_cm = 2.f;

// Kick shape at strength 1 and native scale 1. INFERENCE (no headset test yet): a light pistol-to-rifle kick.
constexpr float kPitchDeg = 6.f, kYawDeg = 1.2f, kBackM = 0.035f, kOmega = 28.f, kCap = 2.2f;
// [LIVE VR_RECOIL.log 2026-11-17, one semi-auto gun] each shot = a game rumble rise (0.32-0.34) at the moment of fire, then a
// second rise 63-94 ms later; the game fires at ~0.25 trigger, before a 0.6 edge, and light pulls (0.31) fired with no 0.6 edge.
// So a shot is the first rumble rise while the trigger is past 0.15; later rises of the same shot (one press showed three over
// 172 ms) fall in the refractory time. Cost: full auto faster than ~330 rounds/min kicks on every other round.
// [LIVE 2026-11-17 second test, HEADSET "sometimes it triggers, sometimes not"] 7 presses lost their first kick: the rumble came
// ~15 ms before the next Present's trigger poll, so the trigger still read as released. Full auto showed rumble rises every
// 171-188 ms (one per round), and after release the tail pulses (0.15-0.27, one 0.36 with the trigger long released) fired kicks
// while the 150 ms hold window was open. So: the trigger is read fresh at the rumble, a rumble seen up to 80 ms before the
// trigger reads as pulled still counts, a shot rumble is >= 0.29, and the shot refractory is 140 ms (above the 63-94 ms second
// rise of one semi-auto shot, below the 171 ms full-auto period).
constexpr unsigned long long kRefractoryMs = 140, kRumbleEdgeMs = 50, kTriggerHoldMs = 150, kResidualWindowMs = 250, kEarlyRumbleMs = 80;
constexpr float kTriggerHeld = 0.15f, kShotRumble = 0.29f;
unsigned long long g_early_rumble = 0;  // a shot-sized rumble that came while the trigger still read as released
float g_early_value = 0.f;
// [LIVE 2026-11-17 third test, HEADSET "every other bullet on automatic weapons"] a held burst gave shot rumble rises 78-94 ms
// apart, and some rounds gave no rise at all (gaps of 156-172 ms: the rumble stayed up, so there was no new edge). The 140 ms
// refractory dropped every second round. [STATIC] no ammo counter to read instead: Agent::ClipAmmo::Execute (0x141C748E0) reads
// FUN_142A66FE0(agent+0x740), which is `xor eax,eax; ret` in this build.
// So each weapon learns whether it is automatic: two gaps of 55-130 ms between shot rises in one hold (a round with no rise,
// up to 260 ms, does not break the run). An automatic weapon gets a
// refractory of 0.6 x its round interval, and while the trigger stays held and the rumble stays up, a round with no rise of its
// own is filled in once 1.4 intervals have passed since the last kick. Semi-auto weapons (at most two rises per shot, 63-94 ms
// apart) keep the 140 ms refractory.
constexpr unsigned long long kAutoGapMin = 55, kAutoGapMax = 130;
struct FireRate { const void* weapon = nullptr; bool automatic = false; float interval_ms = 0.f; };
FireRate g_rates[8];
unsigned long long g_rise_prev = 0;  // previous shot-sized held rumble rise in this hold
int g_rise_run = 0;                  // rises in a row 55-130 ms apart
float g_rumble_level = 0.f;          // last rumble value the game set
unsigned long long g_last_kick_ms = 0;
unsigned long long g_last_rise_ms = 0;  // last shot-sized rumble rise (fill-in stops 3 intervals after it)
unsigned g_count_filled = 0;
// [LIVE 2026-11-18 fourth test, HEADSET "only the first bullet kicks on automatic weapons"] every hold gave exactly two shot rises
// 94-110 ms apart and then no new rise while firing went on: after the first two rounds the game holds the rumble steady.
// INFERENCE (all four logs): a rise is one round, rounds ~94 ms apart, and the "pair" seen with quick taps is two rounds. So
// every rise kicks (refractory 0.6 x the learned interval, default 60 ms), and while the trigger stays held with the rumble
// still >= 0.29 a kick follows every interval ("sustained"). Raw trace (`trace` lines) records every Present while firing:
// trigger, rumble level, rumble calls since the last line and the game's look pitch, to check this and the native climb.
// Released, the rumble decays through 0.15-0.27 (LIVE), so while held it may also sit below the 0.29 rise level: any rumble
// still on (>= 0.15) counts as firing.
constexpr float kDefaultIntervalMs = 100.f, kSustainRumble = 0.15f;
unsigned long long g_hold_start = 0;
unsigned g_hold_kicks = 0, g_count_sustained = 0, g_rumble_calls = 0, g_trace_lines = 0;
unsigned long long g_release_at = 0;

std::mutex g_mutex;  // everything below
rm::Shot g_shots[rm::kShots];
int g_next_shot = 0;
unsigned g_rand = 0x9E3779B9u;
rm::Edge g_trigger_edge, g_rumble_edge, g_anim_edge;
float g_fire = 0.f;                  // weapon hand's trigger, last Present
unsigned long long g_fire_at = 0;    // last time the trigger was held (>= 0.6)
const void* g_weapon = nullptr;      // held weapon (body+0x250), last Present
bool g_eligible = false;             // weapon held, on foot, not a bow
int g_hand = 1;
// Native strength: peak residual after each shot, averaged per weapon.
struct WeaponScale { const void* weapon = nullptr; float ema_cm = 0.f; unsigned shots = 0; };
WeaponScale g_scales[8];
unsigned long long g_window_until = 0;
float g_window_peak = 0.f;
// Last applied kick per hand.
float g_w[2][16];
unsigned long long g_w_at[2] = {0, 0};
std::atomic<bool> g_active[2] = {false, false};
// Status counters.
std::atomic<unsigned> g_try = 0, g_no_aim = 0, g_off_hand = 0;
unsigned g_count_trigger = 0, g_count_rumble = 0, g_count_anim = 0, g_count_shots = 0, g_count_applied = 0, g_probe_lines = 0;
unsigned long long g_status_next = 0;

double now_seconds() {
  static LARGE_INTEGER freq{};
  if (!freq.QuadPart) QueryPerformanceFrequency(&freq);
  LARGE_INTEGER c; QueryPerformanceCounter(&c);
  return static_cast<double>(c.QuadPart) / static_cast<double>(freq.QuadPart);
}

void log(const char* text) {
  static std::wstring dir;
  if (dir.empty()) {
    HMODULE self = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            reinterpret_cast<LPCWSTR>(&log), &self)) return;
    wchar_t path[MAX_PATH];
    if (!GetModuleFileNameW(self, path, MAX_PATH)) return;
    dir = path; dir = dir.substr(0, dir.find_last_of(L"\\/"));
  }
  runtime_log::append(dir + L"\\VR_RECOIL.log", {text, "\r\n"});
}

WeaponScale* scale_slot(const void* weapon, bool create) {
  for (auto& s : g_scales) if (s.weapon == weapon) return &s;
  if (!create) return nullptr;
  for (int i = 7; i > 0; --i) g_scales[i] = g_scales[i - 1];
  g_scales[0] = WeaponScale{weapon, 0.f, 0};
  return &g_scales[0];
}

FireRate* rate_slot(const void* weapon, bool create) {
  for (auto& r : g_rates) if (r.weapon == weapon) return &r;
  if (!create) return nullptr;
  for (int i = 7; i > 0; --i) g_rates[i] = g_rates[i - 1];
  g_rates[0] = FireRate{weapon, false, 0.f};
  return &g_rates[0];
}
float interval_ms() {
  const FireRate* r = rate_slot(g_weapon, false);
  return r && r->interval_ms > 0.f ? r->interval_ms : kDefaultIntervalMs;
}
unsigned long long refractory_ms() { return static_cast<unsigned long long>(0.6f * interval_ms()); }

int signal_mode() { return std::clamp(static_cast<int>(std::lround(g_signal)), 0, 3); }

// g_mutex held. A candidate event from one source; records a shot when it is the chosen signal.
void candidate(const char* source, int mode_bit, unsigned long long now_ms, float value) {
  const bool held = g_fire_at && now_ms - g_fire_at <= kTriggerHoldMs;
  if (g_probe_lines < 800 && g_weapon) {
    ++g_probe_lines;
    char line[200];
    std::snprintf(line, sizeof(line), "probe %llu ms: %s value=%.3f trigger=%.2f held=%d eligible=%d weapon=%p",
                  now_ms, source, value, g_fire, held ? 1 : 0, g_eligible ? 1 : 0, g_weapon);
    log(line);
  }
  const int mode = signal_mode();
  const bool chosen = ((mode == 0 || mode == 2) && mode_bit == 2 && held && value >= kShotRumble) || (mode == 1 && mode_bit == 1) ||
                      (mode == 3 && mode_bit == 3 && held);
  if (!chosen || !g_eligible) return;
  // One shot per refractory time across sources (a trigger edge and its own rumble are the same shot).
  const double now = now_seconds();
  const rm::Shot& last = g_shots[(g_next_shot + rm::kShots - 1) % rm::kShots];
  if (last.at >= 0.0 && (now - last.at) * 1000.0 < static_cast<double>(refractory_ms())) return;
  const WeaponScale* ws = scale_slot(g_weapon, false);
  // [LIVE 2026-11-17] the fire animation's residual was 0.17 cm (1 window): too small to mean anything, so it only scales once
  // it is measured (3+ shots) and at least 0.5 cm; otherwise strength 1.
  const float scale = ws && ws->shots >= 3 && ws->ema_cm >= 0.5f ? rm::native_scale(ws->ema_cm, g_reference_cm) : 1.f;
  g_rand ^= g_rand << 13; g_rand ^= g_rand >> 17; g_rand ^= g_rand << 5;
  g_shots[g_next_shot] = rm::Shot{now, (g_rand & 1u) ? 1.f : -1.f, scale};
  g_next_shot = (g_next_shot + 1) % rm::kShots;
  ++g_count_shots;
  ++g_hold_kicks;
  g_last_kick_ms = now_ms;
  g_window_until = now_ms + kResidualWindowMs; g_window_peak = 0.f;
  if (g_enabled && g_haptic) xr_input::pulse(g_hand, 0.035f, std::clamp(0.55f * scale * g_strength, 0.1f, 1.f));
}
}  // namespace

namespace vr_recoil {
bool* enabled_flag() { return &g_enabled; }
float* strength_value() { return &g_strength; }
bool* haptic_flag() { return &g_haptic; }
float* signal_value() { return &g_signal; }
float* reference_value() { return &g_reference_cm; }

void on_present() {
  const unsigned long long now_ms = GetTickCount64();
  const void* weapon = nullptr;
  {const game::Pointers p = game::pointers();
    if (p.body) game::mem_read(p.body + 0x250, &weapon, sizeof(weapon));}
  const int hand = skin_palette_probe::weapon_hand_index();
  const bool eligible = weapon && !mount_camera::rider_view() && !vr_weapon::held_is_bow();
  xr_input::PadState pad;
  const float fire = xr_input::latest(&pad) ? (hand ? pad.rt : pad.lt) : 0.f;
  std::lock_guard<std::mutex> lock(g_mutex);
  if (weapon != g_weapon) { g_window_until = 0; g_trigger_edge = rm::Edge{}; }
  g_weapon = weapon; g_eligible = eligible; g_hand = hand; g_fire = fire;
  if (fire >= kTriggerHeld) {
    g_fire_at = now_ms;
    if (g_early_rumble && now_ms - g_early_rumble <= kEarlyRumbleMs) candidate("rumble-early", 2, now_ms, g_early_value);
    g_early_rumble = 0;
  }
  if (fire < kTriggerHeld) {
    if (g_hold_start) {
      char line[160]; std::snprintf(line, sizeof(line), "hold end: %llu ms, %u kicks, rumble=%.3f, interval=%.0f ms",
                                    now_ms - g_hold_start, g_hold_kicks, g_rumble_level, interval_ms());
      if (weapon) log(line);
      g_release_at = now_ms;
    }
    g_hold_start = 0; g_rise_prev = 0; g_rise_run = 0;
  } else if (!g_hold_start) {  // the first round's rumble often lands just before the hold is seen
    g_hold_start = now_ms; g_hold_kicks = g_last_kick_ms && now_ms - g_last_kick_ms <= kTriggerHoldMs ? 1u : 0u;
  }
  // Sustained fire: the game holds the rumble up after the first two rounds, so the rounds that follow give no rise.
  if (weapon && fire >= kTriggerHeld && g_hold_kicks > 0 && g_rumble_level >= kSustainRumble && g_last_kick_ms &&
      now_ms - g_last_kick_ms >= static_cast<unsigned long long>(interval_ms())) {
    ++g_count_sustained;
    candidate("sustained", 2, now_ms, std::max(g_rumble_level, kShotRumble));
  }
  // Raw trace while firing and 400 ms after release.
  if (weapon && g_trace_lines < 3000 && (fire >= kTriggerHeld || (g_release_at && now_ms - g_release_at <= 400))) {
    ++g_trace_lines;
    float pitch = 0.f;
    {const game::Pointers p = game::pointers(); if (p.look) game::mem_read(p.look + 0x38, &pitch, sizeof(pitch));}
    char line[160]; std::snprintf(line, sizeof(line), "trace %llu ms: trigger=%.2f rumble=%.3f calls=%u pitch=%.4f kicks=%u",
                                  now_ms, fire, g_rumble_level, g_rumble_calls, -pitch, g_hold_kicks);
    log(line);
  }
  g_rumble_calls = 0;
  if (weapon && g_trigger_edge.step(fire, 0.1f, 0.25f, now_ms, kRefractoryMs)) { ++g_count_trigger; candidate("trigger", 1, now_ms, fire); }
  // Close a residual window: the fire animation's size for this weapon.
  if (g_window_until && now_ms >= g_window_until) {
    g_window_until = 0;
    if (weapon && g_window_peak > 0.05f) {
      WeaponScale* ws = scale_slot(weapon, true);
      ws->ema_cm = ws->shots ? 0.7f * ws->ema_cm + 0.3f * g_window_peak : g_window_peak;
      ++ws->shots;
    }
  }
  if (!weapon || now_ms < g_status_next) return;
  g_status_next = now_ms + 5000;
  const WeaponScale* ws = scale_slot(weapon, false);
  char line[320];
  std::snprintf(line, sizeof(line),
                "status: on=%d signal=%d strength=%.2f eligible=%d hand=%d edges trigger=%u rumble=%u anim=%u shots=%u applied=%u "
                "weapon=%p native_cm=%.2f (%u shots) scale=%.2f | kicks tried=%u no_aim_ray=%u other_hand=%u | automatic=%d interval=%.0f ms sustained=%u",
                g_enabled ? 1 : 0, signal_mode(), g_strength, eligible ? 1 : 0, hand, g_count_trigger, g_count_rumble, g_count_anim,
                g_count_shots, g_count_applied, weapon, ws ? ws->ema_cm : 0.f, ws ? ws->shots : 0u,
                ws && ws->shots >= 3 && ws->ema_cm >= 0.5f ? rm::native_scale(ws->ema_cm, g_reference_cm) : 1.f,
                g_try.load(), g_no_aim.load(), g_off_hand.load(), rate_slot(weapon, false) && rate_slot(weapon, false)->automatic ? 1 : 0,
                interval_ms(), g_count_sustained);
  log(line);
}

void note_rumble(std::uint16_t left, std::uint16_t right) {
  const float value = static_cast<float>(std::max(left, right)) / 65535.f;
  const unsigned long long now_ms = GetTickCount64();
  // The trigger as the game just read it (the Present poll can be a frame old).
  xr_input::PadState pad;
  const bool pad_ok = xr_input::latest(&pad);
  std::lock_guard<std::mutex> lock(g_mutex);
  g_rumble_level = value;
  ++g_rumble_calls;
  if (!g_rumble_edge.step(value, 0.08f, 0.15f, now_ms, kRumbleEdgeMs)) return;
  ++g_count_rumble;
  if (value >= kShotRumble) g_last_rise_ms = now_ms;
  const float fire = pad_ok ? (g_hand ? pad.rt : pad.lt) : 0.f;
  if (fire >= kTriggerHeld) { g_fire_at = now_ms; g_fire = fire; }
  const bool held = g_fire_at && now_ms - g_fire_at <= kTriggerHoldMs;
  // Learn the weapon's fire mode from the raw rises (before the refractory hides any).
  if (held && value >= kShotRumble && g_weapon && fire >= kTriggerHeld) {
    const unsigned long long gap = g_rise_prev ? now_ms - g_rise_prev : 0;
    if (gap >= kAutoGapMin && gap <= kAutoGapMax) {
      ++g_rise_run;
      FireRate* r = rate_slot(g_weapon, true);
      r->interval_ms = r->interval_ms > 0.f ? 0.8f * r->interval_ms + 0.2f * static_cast<float>(gap) : static_cast<float>(gap);
      if (g_rise_run >= 2 && !r->automatic) {
        r->automatic = true;
        char line[128]; std::snprintf(line, sizeof(line), "weapon %p is automatic: round interval %.0f ms", g_weapon, r->interval_ms); log(line);
      }
    } else if (gap > 2 * kAutoGapMax) g_rise_run = 0;  // a gap of two rounds (one round with no rise) keeps the run
    g_rise_prev = now_ms;
  }
  if (!held && value >= kShotRumble) { g_early_rumble = now_ms; g_early_value = value; }
  candidate("rumble", 2, now_ms, value);
}

void note_anim(int side, float cm) {
  if (!std::isfinite(cm)) return;
  const unsigned long long now_ms = GetTickCount64();
  std::lock_guard<std::mutex> lock(g_mutex);
  if (side != g_hand) return;
  if (g_window_until && now_ms < g_window_until) g_window_peak = std::max(g_window_peak, cm);
  if (g_anim_edge.step(cm, 0.4f, 1.f, now_ms, kRefractoryMs)) { ++g_count_anim; candidate("anim", 3, now_ms, cm); }
}

bool apply(unsigned hand, const float* root, float* wrist16, float* attach16) {
  if (hand > 1) return false;
  rm::Envelope e;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_enabled || !g_eligible || g_strength <= 0.f) { g_active[hand] = false; return false; }
    e = rm::envelope(g_shots, rm::kShots, now_seconds(), kOmega, kCap);
    if (e.pitch >= 1e-3f && static_cast<int>(hand) != g_hand) { ++g_off_hand; g_active[hand] = false; return false; }
  }
  if (e.pitch < 1e-3f) { g_active[hand] = false; return false; }
  ++g_try;
  float origin[3], forward[3], distance = 0.f;
  float inverse_wrist[16];
  if (!vr_weapon::shot_aim(origin, forward, &distance) || !skin_arm_ik::inverse_affine(inverse_wrist, wrist16)) {
    ++g_no_aim; g_active[hand] = false; return false;
  }
  float up[3] = {root[4], root[5], root[6]};
  rm::dir_mul(up, inverse_wrist, up);
  const float pivot[3] = {0.f, 0.f, 0.f};  // the wrist joint: the barrel rises about the hand
  const float deg = 0.0174532925f;
  float w[16], rel[16], kicked[16];
  if (!rm::kick_matrix(forward, up, pivot, e.pitch * kPitchDeg * deg * g_strength, e.yaw * kYawDeg * deg * g_strength,
                       e.pitch * kBackM * g_strength, w)) { g_active[hand] = false; return false; }
  skin_arm_ik::multiply_affine(rel, attach16, inverse_wrist);  // attach = rel * wrist
  skin_arm_ik::multiply_affine(kicked, w, wrist16);
  for (float v : kicked) if (!std::isfinite(v)) { g_active[hand] = false; return false; }
  std::memcpy(wrist16, kicked, 64);
  skin_arm_ik::multiply_affine(attach16, rel, wrist16);
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    std::memcpy(g_w[hand], w, 64); g_w_at[hand] = GetTickCount64(); ++g_count_applied;
  }
  g_active[hand] = true;
  return true;
}

bool unkick_ray(int hand, float* origin, float* forward) {
  if (hand < 0 || hand > 1 || !g_active[hand].load(std::memory_order_relaxed)) return false;
  float w[16];
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_w_at[hand] || GetTickCount64() - g_w_at[hand] > 100) return false;
    std::memcpy(w, g_w[hand], 64);
  }
  rm::unkick_ray(w, origin, forward);
  return true;
}

bool active(int hand) { return hand >= 0 && hand <= 1 && g_active[hand].load(std::memory_order_relaxed); }
}  // namespace vr_recoil
