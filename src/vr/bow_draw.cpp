#include "../runtime_log.hpp"
#include "bow_draw.hpp"
#include "bow_draw_math.hpp"
#include "vr_weapon.hpp"
#include "xr_input.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../camera/mount_camera.hpp"
#include "../game/player.hpp"
#include <windows.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <mutex>
#include <string>

namespace bow_draw {
namespace {
bool g_enabled = true;
bool g_hold_mode = false;
bool g_press_fire = false;
bool g_probe = false;
bool g_visual = true;
bool g_charge_write = true;
std::atomic<ULONGLONG> g_drive_tick{0};
std::mutex g_visual_mutex;
Visual g_visual_now;
unsigned g_nock_count = 0;
bool g_nock_pending = false;     // the off hand took the two-hand hold at the arrow (vr_controls, same thread)
std::atomic<bool> g_pulling{false};
std::atomic<bool> g_native_full{false};
float g_native_rest = -1.f, g_native_last = -1.f; ULONGLONG g_native_still = 0;  // a string is held (aim and hand code, other threads)
float g_draw_length = 0.60f;
bow_draw_math::State g_state;
ULONGLONG g_last_buzz = 0, g_last_probe = 0;
int g_was_active = -1;  // -1 = nothing logged yet
const char* g_last_why = "";
float g_probe_fraction = 0.f, g_probe_native = -1.f, g_probe_string = -1.f;
char g_probe_bow[96]{};
float g_cand_d[3]{-1.f, -1.f, -1.f};  // at the last grip press: wrist to string / limb arrow bone / visible arrow (m)

void log_line(const char* fmt, ...) {
  static std::wstring path;
  if (path.empty()) {
    wchar_t module_path[MAX_PATH]{}; HMODULE self = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&log_line), &self);
    GetModuleFileNameW(self, module_path, MAX_PATH);
    path = module_path;
    const auto slash = path.find_last_of(L"\\/");
    path = path.substr(0, slash == std::wstring::npos ? 0 : slash + 1) + L"VR_BOW.log";
  }
  char line[512];
  va_list args; va_start(args, fmt); std::vsnprintf(line, sizeof(line), fmt, args); va_end(args);
  SYSTEMTIME time{};
  GetLocalTime(&time);
  char stamp[24];
  std::snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u.%03u ", time.wHour, time.wMinute, time.wSecond, time.wMilliseconds);
  runtime_log::append(path, {stamp, line});
}

float dist(const float* a, const float* b) { return bow_draw_math::distance3(a, b); }

// Stage 2 probe (read-only): the game's weapon charge, as BClient::WeaponNode::GetWeaponChargeProgress reads it
// (STATIC, dump 98588da2: Execute 0x143F4FB50 -> 0x1427BBBB0 = *(float*)(*(comp + 16000) + 0x5C), comp = *(entity + 0x250),
// entity = local player body 0x1435A16B0; "full" 0x141DAB730 = charge > 0.9999999). Whether it is the bow tension is OPEN.
bool read_charge(float& c) {
  const game::Pointers gp = game::pointers();
  if (!gp.body) return false;
  unsigned char* comp = nullptr; unsigned char* charge_obj = nullptr;
  if (!game::mem_read(gp.body + 0x250, &comp, sizeof(comp)) || !comp) return false;
  if (!game::mem_read(comp + 16000, &charge_obj, sizeof(charge_obj)) || !charge_obj) return false;
  return game::mem_read(charge_obj + 0x5C, &c, sizeof(c)) && std::isfinite(c);
}
// Charge write (author approval 2026-11-18): the same field, re-read through the chain every time.
bool write_charge(float c) {
  const game::Pointers gp = game::pointers();
  if (!gp.body) return false;
  unsigned char* comp = nullptr; unsigned char* charge_obj = nullptr;
  if (!game::mem_read(gp.body + 0x250, &comp, sizeof(comp)) || !comp) return false;
  if (!game::mem_read(comp + 16000, &charge_obj, sizeof(charge_obj)) || !charge_obj) return false;
  float now_c = 0.f; if (!game::mem_read(charge_obj + 0x5C, &now_c, sizeof(now_c)) || !std::isfinite(now_c)) return false;
  return game::mem_write(charge_obj + 0x5C, &c, sizeof(c));
}
// The hand's string travel (0..1 of a full draw) -> the charge whose drawn string sits there. LIVE 2026-11-18 run 203601
// (heavy bow, charge vs game string 0.485 -> 1.079 m): the string lags the charge (half charge = 19 % of the travel).
// INFERENCE: one curve for all bows, and the string may lag in time rather than by a fixed curve (probe shows which).
float charge_for_travel(float t) {
  static const float kTravel[] = {0.f, .02f, .052f, .11f, .19f, .27f, .36f, .47f, .58f, .66f, .77f, .82f, .87f, .93f, .97f, .99f, 1.f};
  static const float kCharge[] = {0.f, .249f, .358f, .443f, .525f, .568f, .61f, .656f, .7f, .738f, .783f, .827f, .871f, .914f, .951f, .981f, 1.f};
  t = std::fmin(std::fmax(t, 0.f), 1.f);
  for (int i = 1; i < 17; ++i)
    if (t <= kTravel[i]) {const float a = (t - kTravel[i - 1]) / (kTravel[i] - kTravel[i - 1]); return kCharge[i - 1] + a * (kCharge[i] - kCharge[i - 1]);}
  return 1.f;
}
ULONGLONG g_last_write_log = 0; unsigned g_writes = 0, g_write_fails = 0;
ULONGLONG g_last_charge_log = 0;

// The held bow: limb mesh drawn within 200 ms with its string, and the bow's root near the weapon hand's IK socket
// (a bow drawn elsewhere, e.g. on the back, is not held). Root, not string: the string travels ~0.9 m back in the game's
// own draw (HEADSET run 152157: every draw was cancelled 0.2-1.1 s after the grab, and the cancel fired the arrow). why = reason when false (log only).
struct BowView { float string[16]{}, arrow[16]{}, root[16]{}; char path[96]{}; };
bool held_bow(int weapon_hand, BowView& v, const char** why) {
  if (mount_camera::rider_view()) {*why = "riding"; return false;}
  if (!vr_weapon::weapon_placed_by_game()) {*why = "no weapon placed"; return false;}
  if (skin_palette_probe::hand_swap_active()) {*why = "hand swap"; return false;}
  if (!skin_palette_probe::held_bow(v.string, v.arrow, v.root, v.path)) {*why = "no bow drawn"; return false;}
  float flat[16], ik[16];
  if (!skin_palette_probe::weapon_attach_matrices(weapon_hand, flat, ik)) {*why = "no weapon socket"; return false;}
  if (dist(v.root + 12, ik + 12) > 1.0f) {*why = "bow not in hand"; return false;}
  *why = "";
  return true;
}
}  // namespace

bool* enabled_flag() { return &g_enabled; }
bool* hold_mode_flag() { return &g_hold_mode; }
bool* press_fire_flag() { return &g_press_fire; }
bool* probe_flag() { return &g_probe; }
float* draw_length_value() { return &g_draw_length; }
bool* visual_flag() { return &g_visual; }
bool* charge_flag() { return &g_charge_write; }
// LIVE 2026-11-19 run 141102: the written charge holds (0.57 at a 0.28 pull) but the game's string still went to full in ~0.9 s:
// the visible draw is a timed animation, not this charge. So the render overrides never stand down (always false).
bool native_drive() {return false;}
void request_nock() { g_nock_pending = true; }
bool pulling() { return g_pulling.load(std::memory_order_relaxed); }
bool native_full() { return g_native_full.load(std::memory_order_relaxed); }
Visual visual() {std::lock_guard<std::mutex> lock(g_visual_mutex); return g_visual_now;}

void reset() {
  if (g_state.phase == bow_draw_math::Phase::kDrawing) log_line("draw dropped (left gameplay)");
  g_state = bow_draw_math::State{}; g_nock_pending = false; g_pulling.store(false, std::memory_order_relaxed);
  g_was_active = -1; g_last_why = "";
  std::lock_guard<std::mutex> lock(g_visual_mutex); g_visual_now = Visual{};
}

Frame update(int weapon_hand, bool off_grip, bool haptics) {
  Frame f;
  const ULONGLONG now = GetTickCount64();
  const int off = 1 - weapon_hand;
  f.off_hand = off;
  BowView v; const char* why = "";
  const bool bow = held_bow(weapon_hand, v, &why);
  // Probe values (Stage 0), whether or not the pull owns the trigger: the native draw is the bow's own arrow bone
  // distance from its root (rest ~0.31-0.54 m, full ~0.90 m, research/WEAPONS_AND_HANDS.md), string from root likewise.
  if (bow) {
    g_probe_native = dist(v.arrow + 12, v.root + 12); g_probe_string = dist(v.string + 12, v.root + 12);
    std::memcpy(g_probe_bow, v.path, sizeof(g_probe_bow));
  } else {g_probe_native = g_probe_string = -1.f;}

  bow_draw_math::Input in;
  in.active = g_enabled && bow;
  in.grip = off_grip;
  in.nock = g_nock_pending; g_nock_pending = false;
  in.now_ms = now;
  if (in.active) {
    float wrist[3];
    // Draw direction: -Z of the visible arrow mesh's C_arrow_01 (LIVE 2026-11-07: game aim runs along its +Z within
    // 2-7 deg when drawn), else the limb's own C_arrow_01.
    float arrow[16], root[16], string[16]; char apath[96]; bool have_string = false;
    const bool have_arrow = skin_palette_probe::drawn_arrow(arrow, root, string, &have_string, apath);
    const float* z = have_arrow ? arrow + 8 : v.arrow + 8;
    const float zl = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    if (skin_palette_probe::support_wrist_world(off, wrist) && zl > 1e-6f) {
      in.have_hand = true;
      // Grab point: the nearest of the string, the limb's arrow bone and the visible arrow. The short bow
      // (WPN_P_SHORT_BOW_LIMB_10, extra C_gripArrow/C_arrowBase bones) read 0.39-0.53 m from C_bowString at every grip.
      const float* cand[3] = {v.string + 12, v.arrow + 12, have_arrow ? arrow + 12 : nullptr};
      const float* best = cand[0]; float best_d = 1e9f;
      for (int i = 0; i < 3; ++i) {
        g_cand_d[i] = cand[i] ? dist(wrist, cand[i]) : -1.f;
        if (cand[i] && g_cand_d[i] < best_d) {best_d = g_cand_d[i]; best = cand[i];}
      }
      for (int k = 0; k < 3; ++k) {in.hand[k] = wrist[k]; in.string_pt[k] = best[k]; in.draw_dir[k] = -z[k] / zl;}
    }
  }
  // Every change of state, and of the reason it is off (why is a string literal), so a silent miss is visible.
  if (static_cast<int>(in.active) != g_was_active || (!in.active && std::strcmp(why, g_last_why) != 0)) {
    if (in.active) log_line("bow pull ready: %s, off hand %s", v.path, off == 0 ? "LEFT" : "RIGHT");
    else log_line("bow pull off: %s", !g_enabled ? "menu off" : why);
    g_was_active = in.active; g_last_why = why;
  }
  if (in.active && !in.have_hand && in.nock)
    log_line("two-hand hold on the bow, but no off-hand position: no nock");
  bow_draw_math::Params p;
  p.draw_length = g_draw_length;
  p.mode = g_hold_mode ? bow_draw_math::Mode::kHold : bow_draw_math::Mode::kAnalog;
  p.press_fire = g_press_fire;
  const bool was_started = g_state.started;
  const auto out = bow_draw_math::step(g_state, in, p);
  if (g_state.started && !was_started) log_line("draw started: trigger pressed after %.2f m of pull", g_state.pull);
  f.owns_trigger = out.owns_trigger; f.rt = out.rt; f.ready = in.active; f.cancel_button = out.cancel_button;
  f.drawing = g_state.phase == bow_draw_math::Phase::kDrawing;
  f.nocked_now = out.event == bow_draw_math::Event::kNock;
  if (f.nocked_now) ++g_nock_count;
  g_pulling.store(f.drawing && in.active, std::memory_order_relaxed);
  // The game's own draw (its string from the bow root, before the 1:1 redraw): full = moved >= 8 cm from where it was
  // at the nock and still (< 2 mm) for 150 ms.
  if (f.nocked_now) {g_native_rest = g_probe_string; g_native_last = g_probe_string; g_native_still = now;}
  bool full = false;
  if (f.drawing && in.active && g_state.started && g_native_rest >= 0.f && g_probe_string >= 0.f) {
    if (std::fabs(g_probe_string - g_native_last) > 0.002f) g_native_still = now;
    g_native_last = g_probe_string;
    full = std::fabs(g_probe_string - g_native_rest) >= 0.08f && now - g_native_still >= 150;
    // The visible full draw (string still at its end) is the aim's reference. Not the charge: with the charge write it reads
    // ours or the game's per frame (run 141102 flickered).
    float c = 0.f;
    if (!g_charge_write && read_charge(c)) full = full && c > 0.999f;
  }
  if (!f.drawing) g_native_rest = -1.f;
  if (full && !g_native_full.load(std::memory_order_relaxed))
    log_line("game draw full (string %.3f m from the bow root, %.3f at the nock): aim ray learned", g_probe_string, g_native_rest);
  g_native_full.store(full, std::memory_order_relaxed);
  {Visual vis; vis.holding = g_visual && f.drawing && in.active; vis.nock = g_nock_count;  // 1:1: the hand's own travel, up to the draw length
    vis.pull = std::fmin(g_state.pull, g_draw_length);
    for (int k = 0; k < 3; ++k) vis.dir[k] = in.draw_dir[k];
    if (vis.holding && !in.have_hand) {std::lock_guard<std::mutex> lock(g_visual_mutex); vis.pull = g_visual_now.pull;
      for (int k = 0; k < 3; ++k) vis.dir[k] = g_visual_now.dir[k];}
    std::lock_guard<std::mutex> lock(g_visual_mutex); g_visual_now = vis;}
  g_probe_fraction = out.fraction;
  // The charge write: while the trigger holds the draw, the game's charge follows the hand (the game's timer would fill it in
  // ~1.8 s on its own, LIVE run 203601), so the game draws string, arrow, limbs and hand at the pull and fires at that power.
  if (g_charge_write && in.active && f.drawing && g_state.started && out.event != bow_draw_math::Event::kCancelShot) {
    const float want = charge_for_travel(g_state.fraction);
    if (write_charge(want)) {++g_writes; g_drive_tick.store(now, std::memory_order_relaxed);}
    else if (++g_write_fails <= 5) log_line("charge write failed (chain not readable)");
  }
  {float c = -1.f; const bool have_c = in.active && read_charge(c);  // Stage 2 probe: native charge vs the hand's pull
    if (have_c && (out.event != bow_draw_math::Event::kNone || (f.drawing && now - g_last_charge_log >= 100))) {
      g_last_charge_log = now;
      log_line("charge probe: game charge %.3f, hand pull %.2f (%.2f m), trigger %.2f, game string %.3f m from root%s", c,
               out.fraction, g_state.pull, out.rt, g_probe_string,
               out.event == bow_draw_math::Event::kNock ? " [nock]" : out.event == bow_draw_math::Event::kRelease ? " [release]" : "");
    }}

  if (out.grab_distance >= 0.f)
    log_line("nock via two-hand hold: wrist %.2f m from the nearest bow point (string %.2f, bow arrow bone %.2f, arrow %.2f)",
             out.grab_distance, g_cand_d[0], g_cand_d[1], g_cand_d[2]);
  switch (out.event) {
    case bow_draw_math::Event::kNock:
      log_line("nock: off hand %d grabbed the string at %.2f m (%s); trigger waits for %.2f m of pull", off,
               out.grab_distance, v.path, p.start_pull);
      if (haptics) {xr_input::pulse(off, 0.04f, 0.5f); xr_input::pulse(weapon_hand, 0.03f, 0.3f);}
      break;
    case bow_draw_math::Event::kRelease:
      log_line("release: shot at draw %.2f (pull %.2f of %.2f m)%s", out.fraction, out.fraction * p.draw_length,
               p.draw_length, p.press_fire ? " [press-fire tap]" : "");
      if (haptics) {xr_input::pulse(weapon_hand, 0.07f, 0.4f + 0.5f * out.fraction); xr_input::pulse(off, 0.04f, 0.4f);}
      break;
    case bow_draw_math::Event::kDrop:
      log_line("let go before pulling: no shot (pull %.2f m)", out.fraction * p.draw_length);
      break;
    case bow_draw_math::Event::kCancelShot:
      log_line("pushed back to the bow: Cancel Shot (X with the trigger held), string still held");
      if (haptics) xr_input::pulse(off, 0.05f, 0.3f);
      break;
    case bow_draw_math::Event::kCancel:
      log_line("draw cancelled (%s)", why);
      break;
    default: break;
  }
  if (f.drawing && haptics && now - g_last_buzz >= 50) {  // string tension: stronger as it is pulled
    g_last_buzz = now;
    xr_input::pulse(off, 0.05f, 0.08f + 0.45f * out.fraction);
    if (out.fraction > 0.9f) xr_input::pulse(weapon_hand, 0.05f, 0.15f);
  }
  return f;
}

void note_sent(unsigned char rt) {
  if (!g_probe || g_probe_native < 0.f) return;
  const ULONGLONG now = GetTickCount64();
  if (now - g_last_probe < 33) return;
  g_last_probe = now;
  log_line("probe rt=%u pull=%.2f native_draw=%.3f string_root=%.3f mode=%s%s owns=%d bow=%s", rt, g_probe_fraction,
           g_probe_native, g_probe_string, g_hold_mode ? "hold" : "analog", g_press_fire ? "+press" : "",
           g_enabled ? 1 : 0, g_probe_bow);
}
}  // namespace bow_draw
