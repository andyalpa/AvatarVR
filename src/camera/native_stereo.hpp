#pragma once

#include <d3d12.h>

#include "../vr/head_pose.hpp"

#include <cstddef>
#include <cstdint>

// Rung 1 probe: Snowdrop's built-in stereo render path (research/STEREO_ROUTE.md, 2026-10-02 static map).
//   Gfx_View::Render              afop.exe+0x57CEC0  if (view[+0x1458] != 0 && view[+0x1500] == 0) -> stereo path
//   stereo render path            afop.exe+0x58B270  sets view[+0x1500]=1, queues the eye stepper (+0x5A7530),
//                                                    re-enters Gfx_View::Render per eye, queues the capture task
//   per-eye capture task          afop.exe+0x594F40  runs for view[+0x1458] in 2..4; copies the eye image into
//                                                    "Gfx_RenderTargets/EyeLeft" / "EyeRight" (eye = [viewdata+0x1BFA0])
// On PC the mode field stays 0, so none of this runs. This module hooks the two functions (signature-checked,
// readable code), logs every Gfx_View it sees, and on request writes the mode field (a reversible data write).
// Controlled by scripts\AvatarVR_native.txt:
//   off / observe      -> never write (default; a previously written value is restored)
//   mode <N> [view K]  -> write N (1..4) into view K's +0x1458 before each render (K = first-seen order, default 0)
namespace native_stereo {

// The created second view's latest output texture (AddRef'd; caller releases), or nullptr.
ID3D12Resource* acquire_view2_output();
// Eye mode ("eyesize W H both" while stereo runs): both views render offscreen at W x H, independent of the window.
bool eye_size(std::uint32_t* w, std::uint32_t* h);
// Eye mode: the main view's (left eye's) latest output texture (AddRef'd; caller releases), or nullptr.
ID3D12Resource* acquire_main_eye_output();
// Head mode: the pose snapshot applied `lag` StartFrames ago (the one the images at this Present were rendered from).
bool views_for_present(int lag, head_pose::Snapshot* out);
// An XR eye pose relative to the game camera, as the eye cameras are built: axes m9 (column = eye axis), offset off3.
void eye_in_camera(const head_pose::Eye& eye, float* m9, float* off3);
// Base game camera (rows, world) of one applied pose, and that pose's id. source 0: newest applied; 1: the frame the main view
// is rendering (its render-side slot), falling back to newest. applied_pose reads the eyes for an id from this.
bool frame_camera(int source, float* cam16, std::uint64_t* id);
bool applied_pose(std::uint64_t id, head_pose::Snapshot* out);
// Head mode, read-only: the latest aim ray of a controller (0 = left, 1 = right) in the game camera description's
// world frame (origin, unit direction), on the same basis the eye cameras are built on. False without a pose.
bool aim_ray(int hand, float* org, float* dir);
// Grip pose transformed with the exact same recentre and camera basis as the eye/aim poses.
// Returns row-major right/up/forward/position, or false when the cached sample is stale.
bool grip_transform(int hand, float* matrix16);
// Headset centre minus the game camera position (world units, same basis as grip_transform; 0 at recentre).
bool head_offset(float* offset3);
// Roomscale (move_probe, default off): the player body was moved by this world delta under the headset. Queues it;
// the next head step shifts the headset reference by the same amount so the eyes stay put. Any thread.
void rebase_head(const float* world_delta3);
void pending_rebase(float* world_delta3);  // queued, not yet applied (world units)
std::uint64_t rebases_applied();
// Body follows head yaw (move_probe): the game look yaw was turned by this many radians (engine convention, + =
// right). Queues it; the next head step turns the headset reference the opposite way so the view does not rotate.
void rebase_yaw(float engine_yaw);
// Headset yaw minus game camera yaw (radians, engine convention, + = right) from the latest head step, net of queued
// turns. False when stale (no head tracking, looking straight up/down).
bool head_yaw_relative(float* yaw, std::uint64_t* seq = nullptr);  // seq: head steps published so far
// The game camera description matrix (world, rows) the controller grips of the latest aim sample were built in.
bool grip_camera(float* matrix16);
// One consistent read of the camera, body and grips published by compute_aim. Any thread; same freshness as grips.
struct GripFrame {
  float camera[16]{}, body[3]{}, grip[2][16]{};
  bool valid[2]{};
  std::uint64_t sequence=0;
};
bool grip_frame(GripFrame* out);
// Headset centre in world units, same frame and head step as grip_transform (game camera position + head offset).
bool head_position(float* pos3);
// Player body position read in the same head step as grip_transform (world units).
bool grip_body(float* body3);
// Game look yaw read in the same head step as grip_transform (radians).
bool grip_yaw(float* yaw);
// Experimental yaw-only view basis; callers must renew it (expires after 250 ms).
// Does not write the retail game camera or player position. clear on script unload/error.
void set_body_yaw(float yaw);
void clear_body_yaw();
// Frustum tangents (left, right, up, down) of the game's current main-camera projection.
bool game_frustum(float* tan4);

void install();  // hook (call once the module is mapped; safe to call again)
void poll();     // control file + log (Present thread, once a second)
void on_present();  // Present counter for the hang heartbeat
void recentre();    // head mode: take the current headset yaw/position as the new forward on the next frame
void recentre_menu_panel();  // place the main menu panel in front of the head on the next frame
bool front_end();            // true until the first load after the 3D main menu (main menu panel instead of the HUD)
// The size the views actually render at (the swapchain backbuffer; the second view's output follows it). The game
// camera's own aspect (+0x104) stays at the game's resolution setting (1.804 for 2564x1421) even when a VR render
// shape makes the backbuffer tall, which warped the image; the head-tracked eye cameras get this aspect instead.
void set_render_size(unsigned width, unsigned height);
// Pose identity: the applied head-pose snapshot the newest second-view output was rendered with (both eyes are built
// from the same snapshot). False when unknown; callers fall back to the lag-based views_for_present().
bool presented_views(head_pose::Snapshot* out);
// Retire the second view and build a fresh one on the next StartFrame (what a loading screen does). Used by the
// dark-right-eye watchdog in xr_runtime.
void request_rebuild(const char* why);
// True while the second view renders the right eye (gameplay); false on the start-up screens, in the game's menus
// (pause, inventory, character), while switched off, and before the second view exists.
bool stereo_running();
// True while gameplay has been seen and stereo is about to start (the gate's on-streak, or the second view not built
// yet): the flat picture on the headset is then gameplay, which xr_runtime hides behind black (menu_transition.hpp).
bool stereo_pending();
// True when the last stereo pause was a pause menu (main view stopped), not a load, the front end or a resize.
bool stereo_paused_by_menu();
// The Streamline viewport ids of the main view and the created second view (view +0x20; [LIVE] 2026-10-08: 13 / 17).
// False while stereo is not running or either view is missing.
bool stereo_viewport_ids(unsigned* main_id, unsigned* second_id);
// Gfx_View kinds whose Render is running right now on any thread: 1 main view, 2 the second view, 4 other views.
unsigned rendering_kinds();
// Pause stereo for `ms` (window / swapchain resize): the second view and the shared UI pass let go of their frames, so
// IDXGISwapChain::ResizeBuffers can succeed.
void hold_for_resize(unsigned ms);
// The main view's latest output texture (AddRef'd; caller releases), or nullptr. Only tracked while the second view
// exists (the output callback is filtered on it).
ID3D12Resource* acquire_main_output();
// Ask for the main view's output to be held from its next output callback (it is the swapchain backbuffer, so it is
// never held otherwise: an outstanding reference makes ResizeBuffers fail).
void want_main_output();
// Slot-sharing experiment, driven by the automatic sweep: -1 = follow the control file, 0 = off, 1..N = that slot,
// 99 = all. share_slot_count() is 0 until the view diff has run (3 s into stereo).
void set_share_override(int pick);
int share_slot_count();
// Foliage sweep: both eyes from the centre point with one frustum while on (same as "zerosep").
void force_zero_sep(bool on);
// Plant constants probe: records small per-draw constant blocks for a moment with both eyes at one point and logs
// ("cbprobe:") the offsets that differ between otherwise equal blocks of the same frame.
bool request_cb_probe();
// Wind search: samples the second view's own objects with both eyes at one point; logs "wind:" lines.
bool request_wind_search();
bool cb_probe_recording();  // the CBV observer runs at full rate while true
int share_slot_offset(int pick);
// Field experiment: variant 1..N copies one small main-view field into the second view (N = all), 0 = off.
void set_field_variant(int variant);
int field_variant_count();
// Attribute a 1536-byte camera CBV (created on this thread) to the Gfx_View whose Render is running here.
void note_cbv(const void* cpu, std::size_t size);
// Every 1536-byte CBV (not thinned): first-person arm projection override, thread-agnostic.
void note_camera_cbv(const void* cpu, std::size_t size);
// True for the next few draws recorded on a thread that just created an FP arm camera block (read-only depth probe).
bool fp_draw_window();
// Mod menu "Arm projection fix" (Present thread writes, render threads read; default on).
bool* fp_proj_menu_flag();
// Mod menu "Arm depth test (experimental)": also give the FP arm block the eye's +0x4F0/+0x4F4 pair (default off).
bool* fp_depth_menu_flag();
// FP arm depth occlusion (camera.cpp): FP arm pre-pass draws use a GREATER_EQUAL clone of their ALWAYS depth PSO, so
// nearer scene geometry hides the arms. Menu "Arm depth occlusion" (default on); "nofpocclude" in the native control
// turns it off, "fpocclude" forces it on.
bool fp_occlude_enabled();
// "posewrite" token: write the IK hand subtree into the game's 90/192 poses after the camera update (experiment).
bool pose_write_enabled();
bool* fp_occlude_menu_flag();
// UI probe: on this thread, 1 while the main view's UI pass runs, 2 while the second view's runs, else 0.
int ui_pass_eye();
// UI placement for that eye's UI pass at render-target size rt_w x rt_h: layout pixel (x, y) is drawn at
// (ox + x * sx, oy + y * sy). False when placement is off or stereo is not running (the UI is then left alone).
bool ui_map(int eye, float rt_w, float rt_h, float* ox, float* oy, float* sx, float* sy,
            float source_w = 0.f, float source_h = 0.f);
// Main menu panel: the main view's UI pass draws into camera.cpp's layer texture (shown as an OpenXR quad layer).
bool ui_layer_active();
// Read-only logical layout dimensions used by the existing UI rectangle map.
bool ui_canvas_size(float* w, float* h);
// 0 while the UI pass runs on this thread, 1 while one of its neighbour passes (0x55EE10 / 0x55F150) runs.
int ui_pass_kind();
// Left-image pixel -> right-image pixel (ox + x * sx, oy + y * sy) from the two eyes' rendered frusta, for the main
// view's scan highlights drawn into the right eye. False when off ("noreproj") or the frusta are unknown.
bool scan_reproject(float rt_w, float rt_h, float* ox, float* oy, float* sx, float* sy);
// The same left-image -> right-image map without the "reproj" switch (for the scan mask shift).
bool scan_offsets(float rt_w, float rt_h, float* ox, float* oy, float* sx, float* sy);
bool scan_plane_uv(int eye, float width, float* uv); // decorative plane; independent of highlight mask
// Full-view effects: the canvas on the scanner's head-forward plane, size times the lens span. Out: left, top, right, bottom as
// fractions of that eye's image (beyond 0..1 when the plane is wider than the eye).
bool fx_plane_rect(int eye, float size, float* rect);

}  // namespace native_stereo
