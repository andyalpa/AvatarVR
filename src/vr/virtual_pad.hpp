#pragma once

// Virtual gamepad for the game's own input layer (research/INPUT_OWNERSHIP.md "Gamepad backends").
// The game reads pads through Windows.Gaming.Input and falls back to XInput only when that fails. In an active
// mode this module makes RoGetActivationFactory("Windows.Gaming.Input.Gamepad") fail inside the process, so the
// game loads XInput, and answers XInputGetState for user index 0 itself. Only OS exports are hooked (combase,
// xinput*), no game addresses. Control: scripts\AvatarVR_pad.txt, read once at start-up:
//   off   (default, or no file) nothing is hooked
//   test  scripted pad on slot 0: left stick forward for 2 s of every 10 s (flat-screen check, no headset)
//   xr    slot 0 = OpenXR controller actions (src/vr/xr_input.cpp) merged with a real pad on slot 0; game rumble
//         goes to the controllers' haptics and the real pad. Needs an XR session (AvatarVR_xr.txt) for the actions.
// Logs to VIRTUAL_PAD.log.
namespace virtual_pad {

// Idempotent. Call after MH_Initialize and before the game creates its gamepad backend (DLL start-up thread).
void install();
// Called once per IDXGISwapChain::Present: periodic call counters in the log.
void on_present();
// Mod menu "Keep game pitch level" (default on): in VR gameplay the right stick's up/down is replaced by a value that
// steers the game's own pitch back to level (src/vr/pitch_level.hpp).
bool* pitch_level_flag();
// Right-stick X (-1..1) for mod-owned turning; withholds it from the game for the next 250 ms (call every frame while
// owning the turn, e.g. 40_controller_aim.lua "aim: independent view").
float take_turn_input();
// Mod menu "Default game pitch": the pitch (degrees, + up) the game is held at instead of level.
float* pitch_target_value();

}  // namespace virtual_pad
