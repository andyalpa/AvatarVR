#pragma once

#include <d3d12.h>
#include <dxgi1_4.h>

// On-request capture of the presented frame to scripts\captures\cap_<tag>.bmp.
// Trigger: write a new tag (letters/digits/_-) into scripts\AvatarVR_capture.txt; the next Present is
// captured once. The copy runs on the game's own swapchain queue, then waits for that copy only.
namespace frame_capture {

void install();
// Remember the swapchain's command queue (the "device" argument of CreateSwapChainForHwnd under D3D12).
void set_queue(IUnknown* device_or_queue);
// Re-read the trigger file. Present thread, at most once a second.
void poll();
// Called from the Present hook BEFORE the original Present.
void on_present(IDXGISwapChain* swapchain);
// Capture both eyes at the next Present: the backbuffer (left eye, main view) and the second view's output (right eye)
// side by side into scripts\captures\eyes_<date>_<time>.jpg. Any thread.
void request_eyes();
// Diagnostic: capture three consecutive presented stereo pairs with unique names.
void request_eyes_burst(unsigned long long id);
// Slot sweep: share each main-view slot with the second view in turn, grab the eyes after each, and write one
// contact sheet (scripts\captures\sweep_<time>.jpg) plus per-slot numbers in CAPTURE.log. Any thread.
void request_sweep();
// The same for the small view-field copies (left-only effects experiment); the sheet adds the left eye as "L".
void request_field_sweep();
// Four consecutive eye pairs; logs which left frame each right image matches best (CAPTURE.log).
void request_lag_test();
// Slot sweep with both eyes at one point: logs the foliage mismatch per shared main-view slot (CAPTURE.log).
void request_foliage_sweep();
// Field sweep with both eyes at one point (character face / hair investigation, 2026-10-08). Slot and field sweeps
// with zero eye distance also save every step's eye pair as scripts\captures\sweepstep_<mode>_<step>_<time>.jpg.
void request_zero_field_sweep();

}  // namespace frame_capture
