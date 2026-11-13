#pragma once
#include <windows.h>
#include <d3d12.h>
#include <cstdint>

// In-mod frame extrapolation (2026-11-12, ISSUE-05; "framegen" in [xr], Headset -> "Frame generation (mod)", default
// off). When the headset needs a frame and the game has not delivered a new one, the last real eye images are warped
// forward with the game's own DLSS motion vectors (nearest-depth 3x3 pick against disocclusion) by k game frames, and
// submitted with an extrapolated pose label (xr_runtime.cpp). No added latency (extrapolation, not interpolation).
//
// Data flow:
//  - render threads, DLSS evaluate (dlss_twin.cpp): capture() copies each view's motion vectors and depth into the
//    current capture bank (3 banks, private buffers, no barriers: COMMON promotion / decay);
//  - Present thread (copy_present): keep_real() copies both eye sources into a colour bank (2 per eye,
//    simultaneous-access textures) and publishes the capture bank; gate() makes the game queue wait for the worker
//    before a colour bank it may still read is overwritten;
//  - XR worker: generate() records the warp into both XR images on its own HIGH-priority queue, after a GPU wait on
//    the copy fence of the real frame.
// Phase 1 limits: the HUD (drawn after DLSS, not in the motion vectors) moves with the scene under it; eye mode with
// the second view only; not with the private present queue ("ownqueue").
namespace frame_gen {

void set_log(void (*log)(const char* fmt, ...));
void set_enabled(bool on);
bool enabled();

struct Rect {
  uint32_t x, y, w, h;
};
// view: 0 = main view, 1 = second view (call the second view after mv_repair so it gets the repaired vectors).
// mvec_scale: sl::Constants mvecScale (UV per NDC unit). Extents: the tags' (0 width = whole texture).
void capture(int view, ID3D12GraphicsCommandList* list, ID3D12Resource* mv, D3D12_RESOURCE_STATES mv_state, ID3D12Resource* depth,
             D3D12_RESOURCE_STATES depth_state, const float mvec_scale[2], Rect mv_ext, Rect depth_ext);

// Render thread, at the first draw of a view's gameplay UI pass (camera.cpp prehud_note): copy the eye image (`target`,
// in RENDER_TARGET, restored after) before the HUD is drawn. Generated frames then warp this HUD-free scene and blend
// the unwarped HUD of the final image back in (where final and pre-HUD differ). Used only when `target` is the very
// resource keep_real() gets for that eye and has the eye's size.
void capture_prehud(int view, ID3D12GraphicsCommandList* list, ID3D12Resource* target);

// Present thread, on the game-queue copy list after the eye copies. src[e]: that eye's source (left: backbuffer in
// COPY_SOURCE; right: the second view's simultaneous-access texture). view_of_eye[e]: which view rendered eye e.
// Returns whether a real frame was kept (colour copied and a complete capture bank published).
bool keep_real(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* const src[2], const int view_of_eye[2],
               uint32_t w, uint32_t h);
// Present thread, before ExecuteCommandLists of that list: GPU-wait for the worker if it may still read the bank.
void gate(ID3D12CommandQueue* game_queue);
// Present thread, after that list was submitted and the copy fence signalled with `value`.
void real_submitted(ID3D12Fence* copy_fence, uint64_t value);

// XR worker, before acquiring the XR images: true when generate() has everything it needs for targets of this format
// and size (a ready real frame, the worker queue, the pipeline), so an acquired image is never left unwritten.
bool can_generate(ID3D12Device* device, DXGI_FORMAT target_format, uint32_t w, uint32_t h);
// XR worker (holding xr_runtime's g_xr): warp the last real frame k game frames ahead into targets[e] (the acquired XR
// images, RENDER_TARGET). sharpness: RCAS strength 0..1, the same as the real frames get. False = nothing recorded.
bool generate(ID3D12Device* device, ID3D12Resource* const targets[2], uint32_t w, uint32_t h, float k, float sharpness);

// XR worker, after generate() and BEFORE releasing the XR images: the OpenXR session's queue (the one in the graphics
// binding) waits on the GPU for the warp. The runtime synchronises image reads with that queue only, so without this it
// could show a generated image before it was drawn (AUTHOR 2026-11-12: dark flash of the whole view). GPU wait only.
void sync_to(ID3D12CommandQueue* session_queue);

struct Stats {
  uint64_t captures[2], skipped_layout, kept, not_kept, generated, gen_failures;
  double gpu_ms;  // average warp GPU time over the last report window (0 = no samples)
  double luma[2];  // mean 0..255 of a 128x128 centre crop of the last measured generated frame per eye (-1 = none);
                   // the same crop as xr_runtime's "eye brightness" line for real frames (diagnostic, once a second)
  uint64_t prehud_captures, hud_frames, hud_mismatch;  // HUD-free copies; generated frames with a steady HUD; unusable copies
};
Stats stats();  // also resets the GPU time average

}  // namespace frame_gen
