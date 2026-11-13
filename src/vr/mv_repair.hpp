#pragma once

#include <windows.h>
#include <d3d12.h>

// Right-eye motion-vector repair (2026-10-09). [LIVE] mvcap 2026-10-08 18:02: the second view's motion vectors carry
// no object motion for animated characters (camera motion only), which makes DLSS smear NPCs and turn their hair
// grainy in the right eye. [LIVE] offline check 2026-10-09 (mv/dp/consts 060033): for each right-eye pixel, its depth
// and the two eyes' camera (sl::Constants projection, position and axes) give the matching left-eye pixel; where the
// left depth agrees, the left eye's motion vector is the right one (both eyes share the head, so their camera motion at
// one world point agrees to within millimetres, and the left one also carries the object motion). 91% of pixels
// matched, 8% were occluded in one eye and keep their own vector.
// GPU: at the left eye's DLSS evaluate its motion vectors and depth are copied into private buffers; at the right
// eye's evaluate (before DLSS reads them) a compute shader writes the repaired vectors back into the right eye's
// motion-vector texture. Root descriptors only (no descriptor heaps are touched). Off unless "mvrepair" is in
// scripts\AvatarVR_dlss.txt.
namespace mv_repair {

struct EyeInput {
  ID3D12Resource* mv = nullptr;
  D3D12_RESOURCE_STATES mv_state = D3D12_RESOURCE_STATE_COMMON;
  ID3D12Resource* depth = nullptr;
  D3D12_RESOURCE_STATES depth_state = D3D12_RESOURCE_STATE_COMMON;
  const float* consts = nullptr;  // sl::Constants as floats (projection @8, camera position @94, up @97, right @100, fwd @103)
};

void set_log(void (*log)(const char* fmt, ...));
void set_enabled(bool on);
bool enabled();
// "mvfast": tiled shader with output identical to the reference (mv_repair_shaders.hpp). "mvab": alternate the two
// every 5 s; with AvatarVR_gpuprof.txt "repair" the log reports dispatch time per variant ("mvrepair GPU A/B").
void set_variant(bool fast, bool ab);
// Opt-in sampled GPU timings. Submission callback must run AFTER the real queue submit.
void set_profiling(bool on);
void submitted(ID3D12CommandQueue* queue, UINT count, ID3D12CommandList* const* lists);
// Left eye's evaluate: keep its motion vectors, depth and constants for this frame.
void capture_left(ID3D12GraphicsCommandList* list, const EyeInput& left);
// Right eye's evaluate, before the original evaluate: repair its motion-vector texture in place.
void repair_right(ID3D12GraphicsCommandList* list, const EyeInput& right);
// Present thread: status line (counts) every few seconds.
void report();

}  // namespace mv_repair
