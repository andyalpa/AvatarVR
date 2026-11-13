#pragma once

#include <openxr/openxr.h>

#include "head_pose.hpp"
#include "hand_gesture_policy.hpp"

#include <cstdint>
#include <string>
#include <vector>

// PORT-10 headset side: OpenXR controller actions that emulate an Xbox pad. The game keeps its own pad bindings;
// the mod only reports which pad control is pressed (src/vr/virtual_pad.cpp feeds slot 0 from latest()).
// One action set, suggested bindings for every common interaction profile (Quest Touch / Touch Pro / Touch Plus,
// Valve Index, HTC Vive, Windows MR, HP Reverb G2, Pico 4, KHR simple), so any OpenXR runtime maps its controllers
// onto the same actions and the user can rebind them in the runtime (SteamVR binding UI). Logs to XR_INPUT.log.
// All calls except latest()/rumble() come from the XR worker thread in src/vr/xr_runtime.cpp.
namespace xr_input {

// Interaction-profile extensions worth enabling when the runtime offers them (instance creation).
const std::vector<const char*>& optional_extensions();
// After xrCreateSession: create actions, suggest bindings, attach. `enabled` = instance extensions enabled.
void create(XrInstance instance, XrSession session, const std::vector<std::string>& enabled);
// Once per XR frame while the session runs (after xrBeginFrame): xrSyncActions, read states, apply rumble.
void sync(XrSession session);
// Same XR frame, after sync(): grip and aim poses of both hands in `base` (the LOCAL space the eyes use) at `time`
// (the frame's predicted display time) into out->grip / out->aim. Logs both hands every 5 s.
void locate_hands(XrSpace base, XrTime time, head_pose::Snapshot* out);
// XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: log which profile each hand uses.
void on_profile_changed(XrSession session);
// Before xrDestroySession / xrDestroyInstance.
void destroy();

// Pad state from the controllers, XInput conventions: sticks -1..1 (+y up), triggers 0..1, buttons as
// XINPUT_GAMEPAD_* bits.
struct PadState {
  float lx = 0.f, ly = 0.f, rx = 0.f, ry = 0.f;
  float lt = 0.f, rt = 0.f;
  std::uint16_t buttons = 0;
};
// Any thread. The pad the game should see: neutral while the mod menu is open, and the mod-menu button's shared
// pad control held back until the press is known (see menu_filter in xr_input.cpp). False when no session is
// focused or the last sync is older than 250 ms.
bool latest(PadState* out);
// Left-handed weapon mode (run 59). Since run 64 the trigger routing lives in the pad composition (vr_controls, or the
// plain mapping in virtual_pad); latest() reports the controllers as they are. Present thread, every frame.
void set_left_handed(bool on);
// Run 64: a short haptic tick on one controller (hand 0 left, 1 right), applied at the next action sync.
void pulse(int hand, float seconds, float amplitude);
// Any thread. The controllers as they are (no menu filtering), for navigating the mod menu. Same freshness rule.
bool controls(PadState* out);
// Independent sensor stream; never filtered/remapped into gameplay buttons.
// False/zeroed when unfocused or older than 250 ms. isActive is per signal.
bool sensors(hand_gestures::Input* out);

// Mod menu button ("mod_menu" action; default the left upper face button: Y on Touch/Pico/HP, left B on Index,
// left menu on Vive/WMR/simple). Hold 0.6 s opens, a press while open closes.
enum MenuEvent { kMenuOpen = 1, kMenuClose = 2 };
void set_menu_open(bool open);  // the menu tells xr_input whether it is showing (Present thread, every frame)
int take_menu_events();         // MenuEvent bits since the last call (Present thread)
// Any thread. XInput motor speeds 0..65535 (left = low frequency, right = high frequency) -> controller haptics.
void rumble(std::uint16_t left, std::uint16_t right);

}  // namespace xr_input
