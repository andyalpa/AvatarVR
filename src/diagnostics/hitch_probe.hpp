#pragma once

#include <d3d12.h>

#include <cstdint>

// Hitch probe (performance pass 2026-10-23, read-only, opt-in: scripts\AvatarVR_hitch.txt = "on").
// Times calls that usually cause frame spikes -- pipeline-state / state-object creation, resource and heap creation,
// residency changes, file open/read/write -- and attributes them to the frame (Present to Present) they ran in. Frames
// longer than 40 ms in gameplay get one HITCH_PROBE.log line with the per-kind count / total / longest call and its
// thread; a summary follows every 10 s. Calls are passed through unchanged.
namespace hitch_probe {

enum Kind : int {
  kGraphicsPso, kComputePso, kStreamPso, kStateObject, kPipelineLibrary, kCommittedResource, kPlacedResource,
  kReservedResource, kHeap, kResidency, kFileOpen, kFileRead, kFileWrite, kKindCount
};

// Reads the control file and hooks the kernel32 file calls (call once early).
void install(const wchar_t* scripts_dir);
// Hooks the D3D12 device methods (call once with the game's device; graphics PSO creation is timed by camera.cpp).
void watch_device(ID3D12Device* device);
bool enabled();
// A timed call from another hook (microseconds).
void note(Kind kind, std::uint64_t us);
// Present thread, once per Present: closes the frame. gameplay = native stereo running outside the front end.
void on_present(bool gameplay);
// QPC now in microseconds (for callers timing their own calls).
std::uint64_t now_us();

}  // namespace hitch_probe
