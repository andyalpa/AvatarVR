#pragma once

#include <unknwn.h>
#include <dxgi.h>
#include <d3d12.h>

#include <cstdint>

#include "head_pose.hpp"

// PORT-08: OpenXR runtime probe and a pose-logging session that does NOT drive the game camera yet.
// Everything runs on a private worker thread, so the game's Present thread is never blocked by
// xrWaitFrame. Default OFF. Controlled by scripts\AvatarVR_xr.txt:
//   off      -> no OpenXR calls at all (a running session is asked to exit and is torn down)
//   probe    -> one-shot: runtime name, HMD, recommended per-eye size, FOV limits, and whether the
//               headset's required GPU (LUID) is the GPU the game renders on
//   session  -> D3D12 session on the game's device and swapchain queue; frame loop with zero layers;
//               head and per-eye poses logged once a second to scripts\XR_RUNTIME.log
//   cinema [swap] [width_m] [distance_m]
//            -> session + a flat 3D screen: each Present is copied into the eye it was rendered for
//               (eye_write alt mode; mono frames go to both eyes) and shown as one quad layer per eye
// The loader is the official Khronos loader built from third_party/openxr-sdk and linked statically.
namespace xr_runtime {

void install();                      // remember paths; does not touch OpenXR
void set_queue(IUnknown* queue);     // the swapchain's command queue (CreateSwapChain* "device" argument)
void set_mirror_refresh(unsigned hz); // 0 = every frame; copies only, no headset pacing
// 0-100 %: AMD FidelityFX RCAS on both eye images when copied to the headset (0 = plain copies, identical pixels).
void set_sharpness(float percent);
void poll();                         // re-read the control file (Present thread, once a second)
void on_present(IDXGISwapChain* swapchain);  // Present thread, BEFORE eye_write::on_present()
// Latest per-eye poses from the running session (LOCAL space); false when no session or no valid pose yet.
bool latest_views(head_pose::Snapshot* out);
// The game's swapchain queue (AddRef'd; caller releases), or nullptr before the swapchain exists.
ID3D12CommandQueue* acquire_game_queue();
bool session_running();  // an XR session is in its frame loop
// The runtime's recommended per-eye image size (view 0); false before the system has been queried.
bool recommended_size(uint32_t* w, uint32_t* h);
// Live one-line state of a frame-generation OpenXR layer in this process (OFXR-Bridge, VectorXR): which one is
// loaded, and from its status.json (when that tool exports one) whether it generates and at what rates. For the mod
// menu's Headset tab; refreshed once a second on the Present thread. 2026-11-12.
const char* framegen_status();
// Headset display period in milliseconds from the last xrWaitFrame (0 = no live session).
double display_period_ms();
// Present thread, first thing in Present: with "halfrate" (opt-in) and stereo running, waits for every second headset frame
// boundary so each stereo image is shown for exactly two headset frames.
void pace_present();
// Mod menu overlay quad (src/ui/vr_menu.cpp). set_overlay: shown or not, distance and width in metres.
void set_overlay(bool visible, float distance_m, float width_m);
void recentre_overlay();  // place the panel in front of the current head pose on the next XR frame
// Present thread: the overlay swapchain image to copy the canvas into (in RENDER_TARGET state), or nullptr. On
// success the XR state lock is held until overlay_release(), which must follow on the same thread right after the
// copy has been submitted to the game queue.
ID3D12Resource* overlay_acquire(uint32_t width, uint32_t height);
void overlay_release(bool written);
// Game UI panel (main menu layer, camera.cpp): shown or not, and its placement in metres (distance in front of the head,
// width, height above head height). Fed once per StartFrame by native_stereo. Copied and shown by this module.
void set_gui_panel(bool visible, float distance_m, float width_m, float height_m);
void recentre_gui_panel();  // place the game UI panel in front of the current head pose on the next XR frame

}  // namespace xr_runtime
