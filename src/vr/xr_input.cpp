#include "../runtime_log.hpp"
#include "xr_input.hpp"
#include "hand_gesture_bindings.hpp"

#include <windows.h>
#include <Xinput.h>

#include <openxr/openxr.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>

// Bindings follow the OpenXR 1.0 interaction profile paths ([SPEC], OpenXR specification section 6.4 and the
// extension chapters for HP / Pico / Touch Pro / Touch Plus). The pad controls they feed are the game's own pad
// bindings; this file does not decide what the game does with them.

namespace {

std::mutex g_log_mutex;
std::wstring g_log_path;

void log_line(const char* fmt, ...) {
  char buffer[600];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty()) {
    HMODULE self = nullptr;
    wchar_t path[MAX_PATH] = {};
    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCWSTR>(&log_line), &self) &&
        GetModuleFileNameW(self, path, MAX_PATH) > 0) {
      std::wstring dir(path);
      dir = dir.substr(0, dir.find_last_of(L"\\/"));
      g_log_path = dir + L"\\XR_INPUT.log";
    }
  }
  SYSTEMTIME t;
  GetLocalTime(&t);
  char line[700];
  const int n = snprintf(line, sizeof(line), "%02u:%02u:%02u.%03u %s\r\n", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds, buffer);
  runtime_log::append(g_log_path, {std::string_view(line, n > 0 && n < static_cast<int>(sizeof(line)) ? n : 0)});
}

// ---- actions --------------------------------------------------------------------------------------------

enum ActionId {
  kMove, kLook, kLT, kRT, kLB, kRB, kA, kB, kX, kY, kStart, kBack, kLSClick, kRSClick,
  kDpadUp, kDpadDown, kDpadLeft, kDpadRight, kHaptic, kGripPose, kAimPose, kMenuToggle, kActionCount
};

struct ActionDef {
  const char* name;
  const char* localized;
  XrActionType type;
  std::uint16_t button;  // XINPUT_GAMEPAD_* for boolean actions
};

const ActionDef kActions[kActionCount] = {
    {"pad_left_stick", "Pad left stick (move)", XR_ACTION_TYPE_VECTOR2F_INPUT, 0},
    {"pad_right_stick", "Pad right stick (look/turn)", XR_ACTION_TYPE_VECTOR2F_INPUT, 0},
    {"pad_left_trigger", "Pad left trigger", XR_ACTION_TYPE_FLOAT_INPUT, 0},
    {"pad_right_trigger", "Pad right trigger", XR_ACTION_TYPE_FLOAT_INPUT, 0},
    {"pad_left_bumper", "Pad left bumper (LB)", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_LEFT_SHOULDER},
    {"pad_right_bumper", "Pad right bumper (RB)", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_RIGHT_SHOULDER},
    {"pad_a", "Pad A", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_A},
    {"pad_b", "Pad B", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_B},
    {"pad_x", "Pad X", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_X},
    {"pad_y", "Pad Y", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_Y},
    {"pad_start", "Pad Start (Menu)", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_START},
    {"pad_back", "Pad Back (View)", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_BACK},
    {"pad_left_stick_click", "Pad left stick click (LS)", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_LEFT_THUMB},
    {"pad_right_stick_click", "Pad right stick click (RS)", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_RIGHT_THUMB},
    {"pad_dpad_up", "Pad D-pad up", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_DPAD_UP},
    {"pad_dpad_down", "Pad D-pad down", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_DPAD_DOWN},
    {"pad_dpad_left", "Pad D-pad left", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_DPAD_LEFT},
    {"pad_dpad_right", "Pad D-pad right", XR_ACTION_TYPE_BOOLEAN_INPUT, XINPUT_GAMEPAD_DPAD_RIGHT},
    {"pad_rumble", "Pad rumble", XR_ACTION_TYPE_VIBRATION_OUTPUT, 0},
    {"hand_grip", "Hand grip pose", XR_ACTION_TYPE_POSE_INPUT, 0},
    {"hand_aim", "Hand aim pose (pointing ray)", XR_ACTION_TYPE_POSE_INPUT, 0},
    // Not a pad control: hold to open the mod menu, press to close it. Default: the left upper face button.
    {"mod_menu", "Mod menu (hold to open, press to close)", XR_ACTION_TYPE_BOOLEAN_INPUT, 0},
};

constexpr ULONGLONG kMenuHoldMs = 600;  // hold the menu button this long to open the menu
constexpr ULONGLONG kTapMs = 120;       // a short press is replayed to the game as a tap this long

struct Binding {
  ActionId action;
  const char* path;
};

struct Profile {
  const char* path;
  const char* extension;  // nullptr: core profile
  std::vector<Binding> bindings;
};

#define HL "/user/hand/left/"
#define HR "/user/hand/right/"

// Touch-style layout shared by Quest Touch, Touch Pro, Touch Plus, HP Reverb G2 and Pico 4: left stick moves,
// right stick looks, triggers are LT/RT, grips are LB/RB, A/B on the right hand, X/Y on the left, left menu Start.
std::vector<Binding> touch_style(bool right_menu_is_back) {
  std::vector<Binding> b = {
      {kMove, HL "input/thumbstick"},          {kLook, HR "input/thumbstick"},
      {kLT, HL "input/trigger/value"},         {kRT, HR "input/trigger/value"},
      {kLB, HL "input/squeeze/value"},         {kRB, HR "input/squeeze/value"},
      {kA, HR "input/a/click"},                {kB, HR "input/b/click"},
      {kX, HL "input/x/click"},                {kY, HL "input/y/click"},
      {kStart, HL "input/menu/click"},         {kLSClick, HL "input/thumbstick/click"},
      {kRSClick, HR "input/thumbstick/click"}, {kHaptic, HL "output/haptic"},
      {kHaptic, HR "output/haptic"},           {kMenuToggle, HL "input/y/click"},
  };
  if (right_menu_is_back) {
    b.push_back({kBack, HR "input/menu/click"});
  }
  return b;
}

// Grip and aim poses exist in every profile above ([SPEC] OpenXR 1.0 section 6.4: "grip/pose", "aim/pose").
std::vector<Profile> with_poses(std::vector<Profile> p) {
  for (Profile& x : p) {
    x.bindings.push_back({kGripPose, "/user/hand/left/input/grip/pose"});
    x.bindings.push_back({kGripPose, "/user/hand/right/input/grip/pose"});
    x.bindings.push_back({kAimPose, "/user/hand/left/input/aim/pose"});
    x.bindings.push_back({kAimPose, "/user/hand/right/input/aim/pose"});
  }
  return p;
}

const std::vector<Profile>& profiles() {
  static const std::vector<Profile> p = with_poses({
      {"/interaction_profiles/oculus/touch_controller", nullptr, touch_style(false)},
      {"/interaction_profiles/facebook/touch_controller_pro", "XR_FB_touch_controller_pro", touch_style(false)},
      {"/interaction_profiles/meta/touch_controller_plus", "XR_META_touch_controller_plus", touch_style(false)},
      {"/interaction_profiles/hp/mixed_reality_controller", "XR_EXT_hp_mixed_reality_controller", touch_style(true)},
      {"/interaction_profiles/bytedance/pico4_controller", "XR_BD_controller_interaction", touch_style(false)},
      {"/interaction_profiles/valve/index_controller", nullptr,
       {{kMove, HL "input/thumbstick"},          {kLook, HR "input/thumbstick"},
        {kLT, HL "input/trigger/value"},         {kRT, HR "input/trigger/value"},
        {kLB, HL "input/squeeze/value"},         {kRB, HR "input/squeeze/value"},
        {kA, HR "input/a/click"},                {kB, HR "input/b/click"},
        {kX, HL "input/a/click"},                {kY, HL "input/b/click"},
        {kStart, HL "input/trackpad/force"},     {kBack, HR "input/trackpad/force"},
        {kLSClick, HL "input/thumbstick/click"}, {kRSClick, HR "input/thumbstick/click"},
        {kHaptic, HL "output/haptic"},           {kHaptic, HR "output/haptic"},
        {kMenuToggle, HL "input/b/click"}}},
      {"/interaction_profiles/htc/vive_controller", nullptr,
       {{kMove, HL "input/trackpad"},            {kLook, HR "input/trackpad"},
        {kLT, HL "input/trigger/value"},         {kRT, HR "input/trigger/value"},
        {kLB, HL "input/squeeze/click"},         {kRB, HR "input/squeeze/click"},
        {kA, HR "input/trackpad/click"},         {kB, HR "input/menu/click"},
        {kX, HL "input/trackpad/click"},         {kStart, HL "input/menu/click"},
        {kHaptic, HL "output/haptic"},           {kHaptic, HR "output/haptic"},
        {kMenuToggle, HL "input/menu/click"}}},
      {"/interaction_profiles/microsoft/motion_controller", nullptr,
       {{kMove, HL "input/thumbstick"},          {kLook, HR "input/thumbstick"},
        {kLT, HL "input/trigger/value"},         {kRT, HR "input/trigger/value"},
        {kLB, HL "input/squeeze/click"},         {kRB, HR "input/squeeze/click"},
        {kA, HR "input/trackpad/click"},         {kB, HR "input/menu/click"},
        {kX, HL "input/trackpad/click"},         {kStart, HL "input/menu/click"},
        {kLSClick, HL "input/thumbstick/click"}, {kRSClick, HR "input/thumbstick/click"},
        {kHaptic, HL "output/haptic"},           {kHaptic, HR "output/haptic"},
        {kMenuToggle, HL "input/menu/click"}}},
      {"/interaction_profiles/khr/simple_controller", nullptr,
       {{kRT, HR "input/select/click"},          {kA, HL "input/select/click"},
        {kStart, HL "input/menu/click"},         {kB, HR "input/menu/click"},
        {kHaptic, HL "output/haptic"},           {kHaptic, HR "output/haptic"},
        {kMenuToggle, HL "input/menu/click"}}},
  });
  return p;
}

#undef HL
#undef HR

XrInstance g_instance = XR_NULL_HANDLE;
XrActionSet g_set = XR_NULL_HANDLE;
XrAction g_actions[kActionCount] = {};
using namespace hand_gestures;
XrAction g_sensors[kSensorCount]{};
constexpr const char* kSensorNames[]={"finger_squeeze","finger_trigger","finger_index_touch","finger_thumb_touch","finger_thumb_press"};
hand_gestures::Input g_sensor_state;
hand_gestures::Input g_pending_sensor; // XR worker only; published after matching pose location
std::uint64_t g_sensor_sequence=0;
bool g_sensor_supported[2][kSensorCount]{}; // XR worker only
XrPath path_of(const char* text);
void sensor_support(XrSession session) {
  for(auto& hand:g_sensor_supported) std::fill(std::begin(hand),std::end(hand),false);
  for(int i=0;i<kSensorCount;++i) {
    if(g_sensors[i]==XR_NULL_HANDLE) continue;
    XrBoundSourcesForActionEnumerateInfo info={XR_TYPE_BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO}; info.action=g_sensors[i];
    std::uint32_t count=0;
    if(XR_FAILED(xrEnumerateBoundSourcesForAction(session,&info,0,&count,nullptr)) || count>64) continue;
    XrPath paths[64]{};
    if(count && XR_SUCCEEDED(xrEnumerateBoundSourcesForAction(session,&info,count,&count,paths)))
      for(unsigned j=0;j<count;++j) {
        char path[XR_MAX_PATH_LENGTH]{}; std::uint32_t size=0;
        if(XR_FAILED(xrPathToString(g_instance,paths[j],sizeof(path),&size,path))) continue;
        if(!std::strncmp(path,"/user/hand/left/",16)) g_sensor_supported[0][i]=true;
        if(!std::strncmp(path,"/user/hand/right/",17)) g_sensor_supported[1][i]=true;
      }
  }
}
// Suggested sensor paths are separate from the established pad bindings. A
// runtime rejecting optional sensors gets a second suggestion with pad only.
void sensor_bindings(const Profile& profile,std::vector<XrActionSuggestedBinding>& out) {
  for(const auto& b:hand_gestures::sensor_bindings(profile.path))
    out.push_back({g_sensors[b.id],path_of(b.path.c_str())});
}
XrPath g_hand[2] = {XR_NULL_PATH, XR_NULL_PATH};
XrSpace g_pose_space[2][2] = {};  // [grip, aim][left, right]
bool g_attached = false;
ULONGLONG g_last_pose_log = 0;

std::mutex g_state_mutex;
xr_input::PadState g_state;  // what the game gets (neutral while the menu is open)
xr_input::PadState g_raw;    // what the controllers report (the menu reads this)
ULONGLONG g_state_tick = 0;
bool g_focused = false;

// Mod-menu button (XR worker thread only, except the atomics).
std::atomic<bool> g_menu_open{false};
std::atomic<int> g_menu_events{0};
bool g_toggle_down = false;
bool g_toggle_used = false;          // this press already opened or closed the menu
ULONGLONG g_toggle_t0 = 0;
std::uint16_t g_toggle_mask = 0;     // pad buttons that went down with the menu button (same physical input)
std::uint16_t g_prev_raw_buttons = 0;
std::uint16_t g_tap_buttons = 0;
ULONGLONG g_tap_until = 0;

// The menu button shares its physical input with a pad control by default (left Y on Touch). Whatever pad buttons
// go down together with it are held back until the press is known: held kMenuHoldMs -> opens the menu and the game
// never sees it; released earlier -> replayed to the game as a kTapMs tap. With the menu open a press closes it, and
// the game gets a neutral pad. Rebinding-proof: nothing here names a physical button.
xr_input::PadState menu_filter(const xr_input::PadState& raw, bool toggle, ULONGLONG now) {
  const std::uint16_t newly = raw.buttons & static_cast<std::uint16_t>(~g_prev_raw_buttons);
  g_prev_raw_buttons = raw.buttons;
  const bool open = g_menu_open.load(std::memory_order_relaxed);
  if (toggle && !g_toggle_down) {
    g_toggle_down = true;
    g_toggle_t0 = now;
    g_toggle_used = false;
    g_toggle_mask = newly;
    if (open) {
      g_menu_events.fetch_or(xr_input::kMenuClose, std::memory_order_relaxed);
      g_toggle_used = true;
      log_line("menu button: press -> close");
    }
  } else if (toggle && !g_toggle_used && now - g_toggle_t0 >= kMenuHoldMs) {
    g_menu_events.fetch_or(xr_input::kMenuOpen, std::memory_order_relaxed);
    g_toggle_used = true;
    log_line("menu button: held %llu ms -> open", static_cast<unsigned long long>(now - g_toggle_t0));
  } else if (!toggle && g_toggle_down) {
    g_toggle_down = false;
    if (!g_toggle_used && g_toggle_mask != 0) {
      g_tap_buttons = g_toggle_mask;  // short press while closed: the game gets its button after all
      g_tap_until = now + kTapMs;
    }
  }
  if (open) {
    return xr_input::PadState();
  }
  xr_input::PadState game = raw;
  if (g_toggle_down) {
    game.buttons &= static_cast<std::uint16_t>(~g_toggle_mask);
  }
  if (now < g_tap_until) {
    game.buttons |= g_tap_buttons;
  }
  return game;
}

std::atomic<std::uint32_t> g_rumble{0};  // left << 16 | right
std::uint32_t g_rumble_applied = 0;
ULONGLONG g_rumble_tick = 0;
ULONGLONG g_last_log = 0;
std::uint16_t g_logged_buttons = 0;

XrPath path_of(const char* text) {
  XrPath p = XR_NULL_PATH;
  if (XR_FAILED(xrStringToPath(g_instance, text, &p))) {
    log_line("xrStringToPath(%s) failed", text);
  }
  return p;
}

std::string result_text(XrResult r) {
  char text[XR_MAX_RESULT_STRING_SIZE] = {};
  if (g_instance == XR_NULL_HANDLE || XR_FAILED(xrResultToString(g_instance, r, text))) {
    snprintf(text, sizeof(text), "%d", static_cast<int>(r));
  }
  return text;
}

bool has(const std::vector<std::string>& v, const char* s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

float clamp1(float v) {
  return v < -1.f ? -1.f : v > 1.f ? 1.f : v;
}

void apply_pulses(XrSession session);
void apply_rumble(XrSession session) {
  apply_pulses(session);
  const std::uint32_t want = g_rumble.load(std::memory_order_relaxed);
  const ULONGLONG now = GetTickCount64();
  // Re-issue every 100 ms while non-zero (each pulse lasts 120 ms), stop once when it drops to zero.
  if (want == g_rumble_applied && (want == 0 || now - g_rumble_tick < 100)) {
    return;
  }
  g_rumble_applied = want;
  g_rumble_tick = now;
  const float amp[2] = {static_cast<float>(want >> 16) / 65535.f, static_cast<float>(want & 0xFFFF) / 65535.f};
  for (int h = 0; h < 2; ++h) {
    XrHapticActionInfo info = {XR_TYPE_HAPTIC_ACTION_INFO};
    info.action = g_actions[kHaptic];
    info.subactionPath = g_hand[h];
    if (amp[h] <= 0.f) {
      xrStopHapticFeedback(session, &info);
      continue;
    }
    XrHapticVibration vib = {XR_TYPE_HAPTIC_VIBRATION};
    vib.amplitude = amp[h];
    vib.duration = 120000000;  // ns
    vib.frequency = XR_FREQUENCY_UNSPECIFIED;
    xrApplyHapticFeedback(session, &info, reinterpret_cast<const XrHapticBaseHeader*>(&vib));
  }
}

}  // namespace

namespace xr_input {
extern std::atomic<std::uint32_t> g_pulse[2];
}
namespace {
void apply_pulses(XrSession session) {
  for (int h = 0; h < 2; ++h) {
    const std::uint32_t p = xr_input::g_pulse[h].exchange(0, std::memory_order_relaxed);
    if (!p) continue;
    XrHapticActionInfo info = {XR_TYPE_HAPTIC_ACTION_INFO};
    info.action = g_actions[kHaptic];
    info.subactionPath = g_hand[h];
    XrHapticVibration vib = {XR_TYPE_HAPTIC_VIBRATION};
    vib.amplitude = static_cast<float>(p & 0xFFFF) / 1000.f;
    vib.duration = static_cast<XrDuration>(p >> 16) * 1000000;  // ms -> ns
    vib.frequency = XR_FREQUENCY_UNSPECIFIED;
    xrApplyHapticFeedback(session, &info, reinterpret_cast<const XrHapticBaseHeader*>(&vib));
  }
}
}  // namespace

namespace xr_input {

const std::vector<const char*>& optional_extensions() {
  static const std::vector<const char*> ext = {"XR_FB_touch_controller_pro", "XR_META_touch_controller_plus",
                                               "XR_EXT_hp_mixed_reality_controller", "XR_BD_controller_interaction"};
  return ext;
}

void create(XrInstance instance, XrSession session, const std::vector<std::string>& enabled) {
  destroy();
  g_instance = instance;
  g_hand[0] = path_of("/user/hand/left");
  g_hand[1] = path_of("/user/hand/right");

  XrActionSetCreateInfo set_info = {XR_TYPE_ACTION_SET_CREATE_INFO};
  std::strncpy(set_info.actionSetName, "avatarvr_pad", XR_MAX_ACTION_SET_NAME_SIZE - 1);
  std::strncpy(set_info.localizedActionSetName, "AvatarVR gamepad", XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE - 1);
  XrResult r = xrCreateActionSet(instance, &set_info, &g_set);
  if (XR_FAILED(r)) {
    log_line("xrCreateActionSet failed: %s", result_text(r).c_str());
    g_set = XR_NULL_HANDLE;
    return;
  }
  for (int i = 0; i < kActionCount; ++i) {
    XrActionCreateInfo info = {XR_TYPE_ACTION_CREATE_INFO};
    std::strncpy(info.actionName, kActions[i].name, XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(info.localizedActionName, kActions[i].localized, XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    info.actionType = kActions[i].type;
    if (i == kHaptic || i == kGripPose || i == kAimPose) {  // per-hand output / poses
      info.countSubactionPaths = 2;
      info.subactionPaths = g_hand;
    }
    r = xrCreateAction(g_set, &info, &g_actions[i]);
    if (XR_FAILED(r)) {
      log_line("xrCreateAction(%s) failed: %s", kActions[i].name, result_text(r).c_str());
      destroy();
      return;
    }
  }

  int suggested = 0;
  for(int i=0;i<kSensorCount;++i) {
    XrActionCreateInfo info={XR_TYPE_ACTION_CREATE_INFO};
    std::strncpy(info.actionName,kSensorNames[i],XR_MAX_ACTION_NAME_SIZE-1);
    std::strncpy(info.localizedActionName,kSensorNames[i],XR_MAX_LOCALIZED_ACTION_NAME_SIZE-1);
    info.actionType=i<=kSenseTrigger?XR_ACTION_TYPE_FLOAT_INPUT:XR_ACTION_TYPE_BOOLEAN_INPUT;
    info.countSubactionPaths=2; info.subactionPaths=g_hand;
    r=xrCreateAction(g_set,&info,&g_sensors[i]);
    if(XR_FAILED(r)) {log_line("sensor action %s unavailable: %s",kSensorNames[i],result_text(r).c_str()); break;}
  }
  const bool have_sensors=std::all_of(std::begin(g_sensors),std::end(g_sensors),[](XrAction a){return a!=XR_NULL_HANDLE;});
  for (const Profile& p : profiles()) {
    if (p.extension != nullptr && !has(enabled, p.extension)) {
      log_line("profile %s skipped (%s not enabled)", p.path, p.extension);
      continue;
    }
    std::vector<XrActionSuggestedBinding> b;
    for (const Binding& x : p.bindings) {
      b.push_back({g_actions[x.action], path_of(x.path)});
    }
    const auto pad_bindings=b;
    if(have_sensors) sensor_bindings(p,b);
    XrInteractionProfileSuggestedBinding info = {XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    info.interactionProfile = path_of(p.path);
    info.countSuggestedBindings = static_cast<uint32_t>(b.size());
    info.suggestedBindings = b.data();
    r = xrSuggestInteractionProfileBindings(instance, &info);
    if(XR_FAILED(r) && have_sensors) {
      log_line("optional sensor bindings %s rejected: %s; retrying existing pad bindings",p.path,result_text(r).c_str());
      info.countSuggestedBindings=static_cast<uint32_t>(pad_bindings.size()); info.suggestedBindings=pad_bindings.data();
      r=xrSuggestInteractionProfileBindings(instance,&info);
    }
    log_line("bindings %s (%zu): %s", p.path, b.size(), XR_SUCCEEDED(r) ? "ok" : result_text(r).c_str());
    suggested += XR_SUCCEEDED(r) ? 1 : 0;
  }

  XrSessionActionSetsAttachInfo attach = {XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
  attach.countActionSets = 1;
  attach.actionSets = &g_set;
  r = xrAttachSessionActionSets(session, &attach);
  g_attached = XR_SUCCEEDED(r);
  log_line("action set attached: %s (%d profiles suggested)", g_attached ? "ok" : result_text(r).c_str(), suggested);
  for (int k = 0; g_attached && k < 2; ++k) {
    for (int h = 0; h < 2; ++h) {
      XrActionSpaceCreateInfo info = {XR_TYPE_ACTION_SPACE_CREATE_INFO};
      info.action = g_actions[k == 0 ? kGripPose : kAimPose];
      info.subactionPath = g_hand[h];
      info.poseInActionSpace.orientation.w = 1.f;
      r = xrCreateActionSpace(session, &info, &g_pose_space[k][h]);
      if (XR_FAILED(r)) {
        log_line("xrCreateActionSpace(%s %s) failed: %s", k == 0 ? "grip" : "aim", h == 0 ? "left" : "right",
                 result_text(r).c_str());
        g_pose_space[k][h] = XR_NULL_HANDLE;
      }
    }
  }
}

void sync(XrSession session) {
  if (!g_attached) {
    return;
  }
  XrActiveActionSet active = {g_set, XR_NULL_PATH};
  XrActionsSyncInfo info = {XR_TYPE_ACTIONS_SYNC_INFO};
  info.countActiveActionSets = 1;
  info.activeActionSets = &active;
  const XrResult r = xrSyncActions(session, &info);
  const bool focused = r == XR_SUCCESS;

  PadState s;
  hand_gestures::Input sensor;
  bool pad_active[kActionCount]{};
  bool toggle = false;
  if (focused) {
    for (int i = 0; i < kActionCount; ++i) {
      XrActionStateGetInfo get = {XR_TYPE_ACTION_STATE_GET_INFO};
      get.action = g_actions[i];
      switch (kActions[i].type) {
        case XR_ACTION_TYPE_VECTOR2F_INPUT: {
          XrActionStateVector2f v = {XR_TYPE_ACTION_STATE_VECTOR2F};
          if (XR_SUCCEEDED(xrGetActionStateVector2f(session, &get, &v)) && v.isActive) {
            pad_active[i]=true;
            (i == kMove ? s.lx : s.rx) = clamp1(v.currentState.x);
            (i == kMove ? s.ly : s.ry) = clamp1(v.currentState.y);
          }
          break;
        }
        case XR_ACTION_TYPE_FLOAT_INPUT: {
          XrActionStateFloat v = {XR_TYPE_ACTION_STATE_FLOAT};
          if (XR_SUCCEEDED(xrGetActionStateFloat(session, &get, &v)) && v.isActive) {
            pad_active[i]=true;
            (i == kLT ? s.lt : s.rt) = std::clamp(v.currentState, 0.f, 1.f);
          }
          break;
        }
        case XR_ACTION_TYPE_BOOLEAN_INPUT: {
          XrActionStateBoolean v = {XR_TYPE_ACTION_STATE_BOOLEAN};
          if (XR_SUCCEEDED(xrGetActionStateBoolean(session, &get, &v)) && v.isActive) {
            pad_active[i]=true;
            if(!v.currentState) break;
            if (i == kMenuToggle) {
              toggle = true;
            } else {
              s.buttons |= kActions[i].button;
            }
          }
          break;
        }
        default:  // poses are located in locate_hands(); haptics are output
          break;
      }
    }
    apply_rumble(session);
    for(int h=0;h<2;++h) {
      hand_gestures::Signal* fields[]={&sensor.hand[h].grip,&sensor.hand[h].trigger,&sensor.hand[h].index_touch,
        &sensor.hand[h].thumb_touch,&sensor.hand[h].thumb_press};
      for(int i=0;i<kSensorCount;++i) {
        if(g_sensors[i]==XR_NULL_HANDLE) continue;
        XrActionStateGetInfo get={XR_TYPE_ACTION_STATE_GET_INFO}; get.action=g_sensors[i]; get.subactionPath=g_hand[h];
        auto& field=*fields[i];
        if(i<=kSenseTrigger) {
          XrActionStateFloat v={XR_TYPE_ACTION_STATE_FLOAT};
          if(XR_SUCCEEDED(xrGetActionStateFloat(session,&get,&v))) {
            field.supported=g_sensor_supported[h][i] || v.isActive!=0; field.active=v.isActive!=0;
            field.value=v.isActive?std::clamp(v.currentState,0.f,1.f):0.f;
          }
        } else {
          XrActionStateBoolean v={XR_TYPE_ACTION_STATE_BOOLEAN};
          if(XR_SUCCEEDED(xrGetActionStateBoolean(session,&get,&v))) {
            field.supported=g_sensor_supported[h][i] || v.isActive!=0; field.active=v.isActive!=0;
            field.value=v.isActive && v.currentState?1.f:0.f;
          }
        }
      }
      // If optional sensor suggestions were rejected, existing active pad
      // actions still supply press/hold fallback. Never synthesize touch/lift.
      auto& hand=sensor.hand[h];
      if(!hand.grip.active && pad_active[h?kRB:kLB])
        hand.grip={(s.buttons&(h?XINPUT_GAMEPAD_RIGHT_SHOULDER:XINPUT_GAMEPAD_LEFT_SHOULDER))?1.f:0.f,true,true};
      if(!hand.trigger.active && pad_active[h?kRT:kLT]) hand.trigger={h?s.rt:s.lt,true,true};
      const std::uint16_t thumb_mask=h?(XINPUT_GAMEPAD_A|XINPUT_GAMEPAD_B|XINPUT_GAMEPAD_RIGHT_THUMB):
        (XINPUT_GAMEPAD_X|XINPUT_GAMEPAD_Y|XINPUT_GAMEPAD_LEFT_THUMB);
      if(!hand.thumb_press.active && (pad_active[h?kA:kX] || pad_active[h?kB:kY] || pad_active[h?kRSClick:kLSClick]))
        hand.thumb_press={(s.buttons&thumb_mask)?1.f:0.f,true,true};
    }
  }
  const ULONGLONG now = GetTickCount64();
  const PadState game = menu_filter(s, toggle, now);
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_state = game;
    g_raw = s;
    g_state_tick = now;
    g_focused = focused;
    // [AUTHOR 2026-11-17] Menu open: gestures keep running (the Hands tab previews poses on the hand) but see
    // neutral controls, so menu buttons never move fingers. Was: focused=false, which switched gestures off.
    if(g_menu_open.load(std::memory_order_relaxed))
      for(auto& hand:sensor.hand) {const bool tracked=hand.tracked; hand=hand_gestures::Sensors{}; hand.tracked=tracked;}
    sensor.at=now; sensor.sequence=++g_sensor_sequence; sensor.focused=focused;
    g_pending_sensor=sensor;
    if(!sensor.focused) g_sensor_state={};
  }
  if (s.buttons != g_logged_buttons || now - g_last_log >= 5000) {
    log_line("sync %s buttons=0x%04X L=(%.2f,%.2f) R=(%.2f,%.2f) LT=%.2f RT=%.2f",
             focused ? "focused" : result_text(r).c_str(), s.buttons, s.lx, s.ly, s.rx, s.ry, s.lt, s.rt);
    g_logged_buttons = s.buttons;
    g_last_log = now;
  }
}

void on_profile_changed(XrSession session) {
  {std::lock_guard<std::mutex> lock(g_state_mutex); g_sensor_state={}; g_pending_sensor={};}
  sensor_support(session);
  for (int h = 0; h < 2; ++h) {
    XrInteractionProfileState st = {XR_TYPE_INTERACTION_PROFILE_STATE};
    char name[XR_MAX_PATH_LENGTH] = "none";
    if (XR_SUCCEEDED(xrGetCurrentInteractionProfile(session, g_hand[h], &st)) && st.interactionProfile != XR_NULL_PATH) {
      uint32_t n = 0;
      xrPathToString(g_instance, st.interactionProfile, sizeof(name), &n, name);
    }
    log_line("%s hand interaction profile: %s", h == 0 ? "left" : "right", name);
  }
}

void locate_hands(XrSpace base, XrTime time, head_pose::Snapshot* out) {
  const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT;
  for (int k = 0; k < 2; ++k) {
    for (int h = 0; h < 2; ++h) {
      head_pose::Hand& d = (k == 0 ? out->grip : out->aim)[h];
      d.valid = false;
      if (!g_attached || g_pose_space[k][h] == XR_NULL_HANDLE) {
        continue;
      }
      XrSpaceLocation loc = {XR_TYPE_SPACE_LOCATION};
      if (XR_FAILED(xrLocateSpace(g_pose_space[k][h], base, time, &loc)) || (loc.locationFlags & need) != need) {
        continue;
      }
      d.qx = loc.pose.orientation.x;
      d.qy = loc.pose.orientation.y;
      d.qz = loc.pose.orientation.z;
      d.qw = loc.pose.orientation.w;
      d.px = loc.pose.position.x;
      d.py = loc.pose.position.y;
      d.pz = loc.pose.position.z;
      d.valid = true;
    }
  }
  {std::lock_guard<std::mutex> lock(g_state_mutex);
    for(int h=0;h<2;++h) g_pending_sensor.hand[h].tracked=out->grip[h].valid;
    g_sensor_state=g_pending_sensor;}
  const ULONGLONG now = GetTickCount64();
  if (now - g_last_pose_log < 5000) {
    return;
  }
  g_last_pose_log = now;
  for (int h = 0; h < 2; ++h) {
    const head_pose::Hand& a = out->aim[h];
    const head_pose::Hand& g = out->grip[h];
    // Aim ray direction = the pose's -z axis in LOCAL space.
    const float fx = -2.f * (a.qx * a.qz + a.qy * a.qw);
    const float fy = -2.f * (a.qy * a.qz - a.qx * a.qw);
    const float fz = -(1.f - 2.f * (a.qx * a.qx + a.qy * a.qy));
    log_line("pose %s: aim %s p=(%.3f,%.3f,%.3f) fwd=(%.3f,%.3f,%.3f) | grip %s p=(%.3f,%.3f,%.3f) | grip->aim %.3f m",
             h == 0 ? "left" : "right", a.valid ? "ok" : "--", a.px, a.py, a.pz, fx, fy, fz, g.valid ? "ok" : "--", g.px,
             g.py, g.pz,
             std::sqrt((a.px - g.px) * (a.px - g.px) + (a.py - g.py) * (a.py - g.py) + (a.pz - g.pz) * (a.pz - g.pz)));
  }
}

void destroy() {
  for (auto& k : g_pose_space) {
    for (XrSpace& sp : k) {
      if (sp != XR_NULL_HANDLE) {
        xrDestroySpace(sp);
      }
      sp = XR_NULL_HANDLE;
    }
  }
  if (g_set != XR_NULL_HANDLE) {
    xrDestroyActionSet(g_set);  // also destroys its actions
  }
  g_set = XR_NULL_HANDLE;
  std::fill(std::begin(g_actions), std::end(g_actions), XrAction{XR_NULL_HANDLE});
  std::fill(std::begin(g_sensors),std::end(g_sensors),XrAction{XR_NULL_HANDLE});
  g_attached = false;
  g_rumble_applied = 0;
  std::lock_guard<std::mutex> lock(g_state_mutex);
  g_state = PadState();
  g_raw = PadState();
  g_focused = false;
  g_state_tick = 0;
  g_sensor_state={};
  g_pending_sensor={};
  for(auto& hand:g_sensor_supported) std::fill(std::begin(hand),std::end(hand),false);
}

std::atomic<bool> g_left_handed{false};
void set_left_handed(bool on) { g_left_handed.store(on, std::memory_order_relaxed); }

bool latest(PadState* out) {
  std::lock_guard<std::mutex> lock(g_state_mutex);
  if (!g_focused || GetTickCount64() - g_state_tick > 250) {
    return false;
  }
  *out = g_state;
  return true;
}
std::atomic<std::uint32_t> g_pulse[2] = {};  // (duration ms << 16) | amplitude permille
void pulse(int hand, float seconds, float amplitude) {
  if (hand < 0 || hand > 1) return;
  const std::uint32_t ms = static_cast<std::uint32_t>(std::clamp(seconds, 0.005f, 1.f) * 1000.f);
  const std::uint32_t amp = static_cast<std::uint32_t>(std::clamp(amplitude, 0.f, 1.f) * 1000.f);
  g_pulse[hand].store((ms << 16) | amp, std::memory_order_relaxed);
}

void rumble(std::uint16_t left, std::uint16_t right) {
  g_rumble.store((static_cast<std::uint32_t>(left) << 16) | right, std::memory_order_relaxed);
}

bool controls(PadState* out) {
  std::lock_guard<std::mutex> lock(g_state_mutex);
  if (!g_focused || GetTickCount64() - g_state_tick > 250) {
    return false;
  }
  *out = g_raw;
  return true;
}

bool sensors(hand_gestures::Input* out) {
  if(!out) return false;
  std::lock_guard<std::mutex> lock(g_state_mutex);
  *out={};
  if(!g_sensor_state.focused || !hand_gestures::fresh(GetTickCount64(),g_sensor_state.at)) return false;
  *out=g_sensor_state; return true;
}

void set_menu_open(bool open) {
  g_menu_open.store(open, std::memory_order_relaxed);
}

int take_menu_events() {
  return g_menu_events.exchange(0, std::memory_order_relaxed);
}

}  // namespace xr_input
