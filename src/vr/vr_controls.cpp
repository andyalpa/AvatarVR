#include "../runtime_log.hpp"
#include "vr_controls.hpp"
#include "vr_controls_math.hpp"
#include "vr_arms.hpp"
#include "vr_weapon.hpp"
#include "bow_draw.hpp"
#include "reload_probe.hpp"
#include "manual_reload.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../camera/mount_camera.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace vr_controls {
namespace {
bool g_enabled = true;
bool g_haptics = true;
std::mutex g_mutex;

void log_line(const char* fmt, ...) {
  static std::wstring path;
  if (path.empty()) {
    wchar_t module_path[MAX_PATH]{}; HMODULE self = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&log_line), &self);
    GetModuleFileNameW(self, module_path, MAX_PATH);
    path = module_path;
    const auto slash = path.find_last_of(L"\\/");
    path = path.substr(0, slash == std::wstring::npos ? 0 : slash + 1) + L"VR_CONTROLS.log";
  }
  char line[512];
  va_list args; va_start(args, fmt); std::vsnprintf(line, sizeof(line), fmt, args); va_end(args);
  SYSTEMTIME time{};
  GetLocalTime(&time);
  char stamp[24];
  std::snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u.%03u ", time.wHour, time.wMinute, time.wSecond, time.wMilliseconds);
  runtime_log::append(path, {stamp, line});
}

enum class Gesture { kNone, kShoulderPress, kWheel, kScan, kTwoHand, kBowString, kOther };
struct HandState {
  bool grip = false, head = false, shoulder = false;
  Gesture gesture = Gesture::kNone;
  ULONGLONG since = 0;
};
HandState g_hand[2];
vr_controls_math::Flick g_flick;
ULONGLONG g_tap_until_a = 0, g_tap_until_b = 0, g_tap_until_y = 0, g_tap_until_x = 0;
int g_wheel_hand = -1;
std::atomic<bool> g_wheel{false};
int g_pending_hand = -1;  // weapon hand to set once the equip/wheel completes
ULONGLONG g_pending_until = 0;

SHORT to_axis(float v) {
  const float c = std::clamp(v, -1.f, 1.f);
  return static_cast<SHORT>(c < 0.f ? c * 32768.f : c * 32767.f);
}
BYTE to_trigger(float v) { return static_cast<BYTE>(std::clamp(v, 0.f, 1.f) * 255.f + 0.5f); }

void set_weapon_hand(int hand, const char* why) {
  const bool left = hand == 0;
  if (*skin_palette_probe::left_handed_flag() != left) {
    *skin_palette_probe::left_handed_flag() = left;
    log_line("weapon hand -> %s (%s)", left ? "LEFT" : "RIGHT", why);
  }
}
}  // namespace

bool* enabled_flag() { return &g_enabled; }
bool* haptics_flag() { return &g_haptics; }
bool wheel_active() { return g_wheel.load(std::memory_order_relaxed); }
bool hand_busy(int hand) {
  if(hand<0 || hand>1) return true;
  std::lock_guard<std::mutex> lock(g_mutex);
  if(manual_reload::native_hand(hand)) return true;  // FEATURE-05: the game's own reload hold on the fingers
  return g_wheel.load(std::memory_order_relaxed) || g_hand[hand].gesture==Gesture::kScan ||
    g_hand[hand].gesture==Gesture::kShoulderPress;
}

bool compose(const xr_input::PadState& s, XINPUT_GAMEPAD* out, bool gameplay) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_enabled) {bow_draw::reset(); manual_reload::reset(); return false;}
  const ULONGLONG now = GetTickCount64();
  // [HEADSET run 64] the scheme also replaced the default inputs in menus. Outside gameplay the plain mapping is used;
  // a wheel or scan that is still held keeps going until its grip is released (the wheel may pause the 3D view).
  const bool gesture_held = g_wheel.load() || g_hand[0].gesture == Gesture::kScan || g_hand[1].gesture == Gesture::kScan;
  static bool was_gameplay = true;
  if (gameplay != was_gameplay) {log_line(gameplay ? "gameplay: VR controls" : "menu/not in control: default inputs"); was_gameplay = gameplay;}
  if (!gameplay && !gesture_held) {
    for (auto& hs : g_hand) {hs.gesture = Gesture::kNone; hs.grip = (s.buttons & (&hs == &g_hand[0] ? XINPUT_GAMEPAD_LEFT_SHOULDER : XINPUT_GAMEPAD_RIGHT_SHOULDER)) != 0;}
    g_flick.armed = std::fabs(s.ry) < 0.30f;
    bow_draw::reset();
    manual_reload::reset();
    return false;
  }
  const int weapon_hand = skin_palette_probe::weapon_hand_index();  // 1 right, 0 left
  const bool weapon_held = vr_weapon::weapon_placed_by_game();

  // ---- hand positions in the head's yaw frame
  vr_arms::Sample smp;
  const bool have = vr_arms::latest(smp) && smp.head_valid;
  const bool grips[2] = {(s.buttons & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0, (s.buttons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0};
  // FEATURE-07: with a bow held, the other hand's grip at the string draws it and opening the grip shoots.
  const bow_draw::Frame bow = bow_draw::update(weapon_hand, grips[1 - weapon_hand], g_haptics);
  reload_probe::tick();  // FEATURE-05 Stage 0: read-only log of the game's own reload (VR_RELOAD.log)
  // FEATURE-05 Stage 1: the off hand reloads the rifle (magazine out, new one from the hip, seat, rack after empty).
  const WORD reload_button = weapon_hand == 0 ? XINPUT_GAMEPAD_Y : XINPUT_GAMEPAD_B;  // physical upper button of the weapon hand
  float off_head[3]{};
  const bool off_head_ok = have && smp.valid[1 - weapon_hand] &&
                           vr_controls_math::head_frame(smp.head_rot, smp.local[1 - weapon_hand] + 12, off_head);
  const manual_reload::Frame rl = manual_reload::update(weapon_hand, grips[1 - weapon_hand], (s.buttons & reload_button) != 0,
                                                        off_head_ok, off_head, g_haptics);
  if (rl.press_x) g_tap_until_x = now + vr_controls_math::kTapMs;
  // Run 65 hand-to-hand swap: the empty hand within 0.15 m of the weapon hand (grip to grip) + its grip press.
  float hands_apart = 1e9f;
  if (have && smp.valid[0] && smp.valid[1]) {
    float d2 = 0.f; for (int k = 0; k < 3; ++k) {const float v = smp.local[0][12 + k] - smp.local[1][12 + k]; d2 += v * v;}
    hands_apart = std::sqrt(d2);
  }
  for (int h = 0; h < 2; ++h) {
    HandState& hs = g_hand[h];
    float p[3];
    const bool pos = have && smp.valid[h] && vr_controls_math::head_frame(smp.head_rot, smp.local[h] + 12, p);
    const bool head = pos && vr_controls_math::in_head(p, hs.head);
    const bool shoulder = pos && vr_controls_math::in_shoulder(h, p, hs.shoulder);
    const bool empty = !(weapon_held && h == weapon_hand);
    if (g_haptics && hs.gesture == Gesture::kNone && ((head && !hs.head && empty) || (shoulder && !hs.shoulder)))
      xr_input::pulse(h, 0.03f, 0.35f);
    if (shoulder != hs.shoulder || head != hs.head)
      log_line("hand %d zones: head %d shoulder %d at (%.2f %.2f %.2f)", h, head, shoulder, pos ? p[0] : 0.f, pos ? p[1] : 0.f, pos ? p[2] : 0.f);
    hs.head = head; hs.shoulder = shoulder;

    // ---- grip gestures
    const bool grip = grips[h];
    // FEATURE-05: the off hand's grip is the reload's (magazine, hip, charging handle): no other grip gesture.
    if (rl.owns_grip && h == 1 - weapon_hand) {
      if (grip && !hs.grip) {hs.since = now; hs.gesture = Gesture::kOther;}
      hs.grip = grip; if (!grip) hs.gesture = Gesture::kNone;
      continue;
    }
    // FEATURE-07: with a bow ready, the bow hand's grip is just holding the bow (support-only would hand it to the string hand).
    if (grip && !hs.grip && weapon_held && h == weapon_hand && bow.ready) {hs.since = now; hs.gesture = Gesture::kOther; hs.grip = grip; continue;}
    if (grip && !hs.grip && weapon_held && h == weapon_hand) {  // run 68: the trigger hand's grip while two-handed
      const int r = skin_palette_probe::dominant_grip_press();
      if (r == 1) {log_line("hand %d lets go of the gun: support hand only, no fire", h); if (g_haptics) xr_input::pulse(h, 0.04f, 0.4f);}
      if (r == 2) {log_line("hand %d re-grabs the gun handle", h); if (g_haptics) {xr_input::pulse(0, 0.04f, 0.5f); xr_input::pulse(1, 0.04f, 0.5f);}}
      if (r) {hs.since = now; hs.gesture = Gesture::kOther; hs.grip = grip; continue;}
    }
    // FEATURE-07: with a bow ready, only the native two-hand hold at the arrow nocks (the same grab as two-hand aiming);
    // a grip in the air does nothing, a grip at the handle still swaps hands below.
    if (grip && !hs.grip && bow.ready && h == bow.off_hand && !skin_palette_probe::hand_swap_active()) {
      const int grab = skin_palette_probe::grab_two_hand(h);
      if (grab == 1) {
        hs.since = now; hs.gesture = Gesture::kBowString; hs.grip = grip;
        bow_draw::request_nock();
        log_line("hand %d takes the bow string (two-hand hold at the arrow)", h);
        continue;
      }
      // Not at the arrow: no nock; the press goes on as usual (handle = swap, shoulder, scan).
      if (grab != 2) log_line("hand %d grip with a bow, away from the arrow: no nock", h);
    }
    if (grip && !hs.grip && mount_camera::harness_near(h)) {  // FEATURE-06: a press at the harness grabs it, nothing else
      hs.since = now; hs.gesture = Gesture::kOther; hs.grip = grip;
      log_line("hand %d grip at the harness: no gesture", h);
      continue;
    }
    if (grip && !hs.grip) {  // press
      hs.since = now;
      // Run 67: the empty hand's grip near the gun - nearest point wins: the native support point = two-hand hold,
      // the trigger handle = hand swap (run 66).
      const int grab = (weapon_held && empty && !skin_palette_probe::hand_swap_active()) ? skin_palette_probe::grab_two_hand(h) : 0;
      if (grab == 1) {
        hs.gesture = Gesture::kTwoHand;
        log_line("hand %d two-hand grab", h);
        if (g_haptics) {xr_input::pulse(0, 0.04f, 0.45f); xr_input::pulse(1, 0.04f, 0.45f);}
      } else if (weapon_held && empty && (grab == 2 || hands_apart < 0.15f) && !skin_palette_probe::hand_swap_active()) {
        hs.gesture = Gesture::kOther;
        if (skin_palette_probe::begin_hand_swap(h)) {
          log_line("hand %d grabs the weapon from the other hand (%.2f m apart): smooth swap", h, hands_apart);
          if (g_haptics) {xr_input::pulse(0, 0.06f, 0.6f); xr_input::pulse(1, 0.06f, 0.6f);}
        }
      }
      else if (shoulder) hs.gesture = Gesture::kShoulderPress;
      else if (head && empty) {hs.gesture = Gesture::kScan; log_line("hand %d scan start", h);}
      else hs.gesture = Gesture::kOther;
    }
    if (grip && hs.gesture == Gesture::kShoulderPress && now - hs.since > vr_controls_math::kHoldMs && g_wheel_hand < 0) {
      hs.gesture = Gesture::kWheel; g_wheel_hand = h; g_wheel.store(true);
      log_line("hand %d weapon wheel open", h);
      if (g_haptics) xr_input::pulse(h, 0.05f, 0.6f);
    }
    if (!grip && hs.grip) {  // release
      if (hs.gesture == Gesture::kShoulderPress) {  // tap
        const auto action = vr_controls_math::equip_decision(h, weapon_held, weapon_hand);
        if (action == vr_controls_math::Equip::kEquipToHand) {
          g_tap_until_y = now + vr_controls_math::kTapMs; set_weapon_hand(h, "shoulder tap: equip");
          log_line("hand %d shoulder tap: equip (Y)", h);
        } else if (action == vr_controls_math::Equip::kHolster) {
          g_tap_until_y = now + vr_controls_math::kTapMs; log_line("hand %d shoulder tap: holster (Y)", h);
        } else {
          if (!skin_palette_probe::begin_hand_swap(h)) set_weapon_hand(h, "shoulder tap: switch hands");
          log_line("hand %d shoulder tap: weapon moves to this hand (smooth)", h);
        }
        if (g_haptics) xr_input::pulse(h, 0.05f, 0.6f);
      } else if (hs.gesture == Gesture::kWheel) {
        g_wheel_hand = -1; g_wheel.store(false);
        g_pending_hand = h; g_pending_until = now + 1500;  // the game equips on wheel release; follow with the hand
        set_weapon_hand(h, "weapon wheel");
        log_line("hand %d weapon wheel release", h);
      } else if (hs.gesture == Gesture::kScan) {
        log_line("hand %d scan end", h);
      } else if (hs.gesture == Gesture::kTwoHand) {
        skin_palette_probe::release_two_hand(h);
        log_line("hand %d two-hand release", h);
      } else if (hs.gesture == Gesture::kBowString) {
        skin_palette_probe::release_two_hand(h);
        log_line("hand %d lets go of the bow string", h);
      }
      hs.gesture = Gesture::kNone;
    }
    hs.grip = grip;
  }
  if (g_pending_hand >= 0 && now > g_pending_until) g_pending_hand = -1;

  // ---- right stick flicks (raw stick, before pitch levelling)
  if (!g_wheel.load()) {
    const int f = vr_controls_math::flick(g_flick, s.ry);
    if (f > 0) {g_tap_until_a = now + vr_controls_math::kTapMs; log_line("flick up: jump (A)");}
    if (f < 0) {g_tap_until_b = now + vr_controls_math::kTapMs; log_line("flick down: crouch (B)");}
  }

  // ---- compose the game's pad
  std::memset(out, 0, sizeof(*out));
  const WORD pass = XINPUT_GAMEPAD_START | XINPUT_GAMEPAD_BACK | XINPUT_GAMEPAD_LEFT_THUMB | XINPUT_GAMEPAD_RIGHT_THUMB |
                    XINPUT_GAMEPAD_DPAD_UP | XINPUT_GAMEPAD_DPAD_DOWN | XINPUT_GAMEPAD_DPAD_LEFT | XINPUT_GAMEPAD_DPAD_RIGHT;
  WORD b = s.buttons & pass;
  // World interactions keep their buttons: A and X pass through (interact, confirm, gather), and the upper button of
  // the hand WITHOUT the weapon passes as itself (Y right-handed, B left-handed). The weapon hand's upper button reloads.
  b |= s.buttons & (XINPUT_GAMEPAD_A | XINPUT_GAMEPAD_X);
  const WORD reload = weapon_hand == 0 ? XINPUT_GAMEPAD_Y : XINPUT_GAMEPAD_B;  // physical upper button of the weapon hand
  const WORD other = weapon_hand == 0 ? XINPUT_GAMEPAD_B : XINPUT_GAMEPAD_Y;
  if (s.buttons & reload) b |= XINPUT_GAMEPAD_X;
  b |= s.buttons & other;
  if (now < g_tap_until_a) b |= XINPUT_GAMEPAD_A;
  if (now < g_tap_until_b) b |= XINPUT_GAMEPAD_B;
  if (now < g_tap_until_y) b |= XINPUT_GAMEPAD_Y;
  if (now < g_tap_until_x) b |= XINPUT_GAMEPAD_X;  // FEATURE-05: the magazine came out by hand
  if (g_wheel.load()) b |= XINPUT_GAMEPAD_LEFT_SHOULDER;
  if (bow.cancel_button) b |= XINPUT_GAMEPAD_X;  // FEATURE-07: the game's Cancel Shot (string pushed back to the bow)
  if (g_hand[0].gesture == Gesture::kScan || g_hand[1].gesture == Gesture::kScan) b |= XINPUT_GAMEPAD_RIGHT_SHOULDER;
  out->wButtons = b;
  out->bRightTrigger = bow.owns_trigger ? to_trigger(bow.rt)  // FEATURE-07: the string pull, the bow hand's trigger ignored
                     : skin_palette_probe::fire_blocked() ? 0  // run 68: the trigger hand is off the gun
                     : rl.block_fire ? 0                        // FEATURE-05: the hands are mid-reload
                                                          : to_trigger(weapon_hand == 0 ? s.lt : s.rt);  // fire = the weapon hand's trigger
  bow_draw::note_sent(out->bRightTrigger);
  out->bLeftTrigger = 0;                                            // ADS removed (automatic later)
  if (g_wheel.load()) {  // wheel selection with the left stick, no walking while choosing
    out->sThumbRX = to_axis(s.lx); out->sThumbRY = to_axis(s.ly);
  } else {
    out->sThumbLX = to_axis(s.lx); out->sThumbLY = to_axis(s.ly);
    out->sThumbRX = to_axis(s.rx); out->sThumbRY = 0;  // right Y = flicks only (pitch levelling writes it after)
  }
  return true;
}
}  // namespace vr_controls
