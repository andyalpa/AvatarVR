#pragma once
#include <windows.h>
#include <d3d12.h>
#include <cstdint>

// Optional per-eye depth for the headset layer (2026-11-12, ISSUE-05; "xrdepth" in [xr], default off).
// Frame-generation layers (VectorXR) and runtimes can use depth submitted with XR_KHR_composition_layer_depth to keep
// near detail (hands, gun) steady in generated frames. Source: the game's DLSS input depth (render resolution,
// reverse-Z, R32G8X24_TYPELESS), copied at each view's DLSS evaluate (dlss_twin.cpp) into a private buffer; at the
// game's Present (xr_runtime.cpp copy_present) a small pass writes it, resampled to the eye size, into a D32_FLOAT
// OpenXR depth swapchain image. The private buffers are used without barriers: buffers are promoted from COMMON on
// first use in an ExecuteCommandLists and decay back to COMMON after it, and the copy and the read are in different
// submissions on the same queue.
namespace xr_depth {

void set_log(void (*log)(const char* fmt, ...));
void set_enabled(bool on);
bool enabled();

// view: 0 = the main view, 1 = the second view. Called on the game's evaluate command list (DLSS input tags).
// ext_*: the depth tag's extent (0 width/height = whole texture). Skips anything but R32G8X24_TYPELESS in
// NON_PIXEL | PIXEL_SHADER_RESOURCE (the gameplay layout; a wrong "before" state would be undefined on the GPU).
void capture(int view, ID3D12GraphicsCommandList* list, ID3D12Resource* depth, D3D12_RESOURCE_STATES state, uint32_t ext_x,
             uint32_t ext_y, uint32_t ext_w, uint32_t ext_h);

// Present thread: whether both views have a capture younger than max_age_ms.
bool ready(uint64_t max_age_ms);

// Present thread: write view's depth into target (an OpenXR D32_FLOAT swapchain image, in DEPTH_WRITE as the
// runtime hands it over; left there) at w x h. Records into list. False if nothing was recorded.
bool record(ID3D12Device* device, ID3D12GraphicsCommandList* list, int view, ID3D12Resource* target, uint32_t w, uint32_t h);

// Counters for the status line.
struct Stats {
  uint64_t captures[2], skipped_layout, skipped_alloc, records, record_failures;
};
Stats stats();

}  // namespace xr_depth
