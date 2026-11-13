#pragma once

#include <unknwn.h>
#include <dxgi.h>

// In-game VR mod menu (Dear ImGui, Avatar-style). F10 opens and closes it.
//
// The menu is drawn with ImGui into a private 1280x960 canvas on the game's queue. In the headset it is an OpenXR
// quad layer seen by both eyes, placed in front of the head when it opens (xr_runtime overlay API). On the monitor
// the same layout is drawn over the backbuffer AFTER xr_runtime has copied the frame to the eyes, so it never bakes
// into an eye image.
//
// Keyboard (polled with GetAsyncKeyState while the game window is focused; the legacy key messages for these keys
// are swallowed while the menu is open so the game does not also act on them):
//   F10 open/close   Up/Down select   Left/Right change   Enter activate   PgUp/PgDn switch tab
//
// Motion controllers (any OpenXR controller, through src/vr/xr_input.cpp; no window focus needed):
//   hold the "mod_menu" button 0.6 s to open (default left Y / Index left B / Vive-WMR left menu), press it again
//   or pad B to close; either stick or the D-pad selects and changes, A activates, LB/RB (grips) switch tabs.
//   While the menu is open the game gets a neutral pad.
//
// Every setting is the control-file setting it replaces: a change rewrites scripts\AvatarVR_native.txt or
// scripts\AvatarVR_xr.txt and re-applies it immediately, so the files always show the live state and a test can
// still be driven by editing them.
namespace vr_menu {

void on_present(IDXGISwapChain* swapchain);  // Present thread, after xr_runtime::on_present()

}  // namespace vr_menu
