#pragma once
#include <d3d12.h>
#include <cstdint>

namespace render_activity {
constexpr unsigned kKinds = 7; // draw, dispatch, copy, indirect, rays, acceleration build, barrier calls
struct Sample {
  std::uint64_t count[kKinds] = {};
  std::uintptr_t stack[4] = {}; // actual afop RVAs from the first recorded operation
  UINT x = 0, y = 0, z = 0; // first compute/ray dispatch dimensions (diagnostic only)
};
void watch(ID3D12GraphicsCommandList* list);
void set_enabled(bool on);
// Reuse the existing camera hooks for these methods; do not install a second MinHook detour.
void note(void* list, unsigned kind, UINT x = 0, UINT y = 0, UINT z = 0);
Sample take(ID3D12CommandList* const* lists, UINT count);
unsigned extra_hook_coverage(); // Reset, CopyTextureRegion, CopyResource, DispatchRays, ASbuild bits

// ---- census shader identity (2026-10-17; only installed for AvatarVR_gpuprof.txt "census") ----
// Compute PSOs are identified by the first 8 bytes of their DXBC/DXIL container digest (FNV-64 of the bytecode when
// the container has none). Each unique bytecode is written once to <dump_dir>\<hash>.cso for offline reading
// (build/dxil_inspect.py). Dispatch and ExecuteIndirect in recorded direct/compute lists are bracketed by two
// timestamps in the game's own list; the profiler resolves them with the submission. Incremental time between
// two stamps includes overlap with earlier unfinished work: it is attribution, not an isolated shader cost.
struct DispatchStamp {
  std::uint64_t shader = 0; // 0 = PSO not created through a hooked path
  UINT x = 0, y = 0, z = 0;
  UINT query = ~0u;         // first of the two timestamps
  bool indirect = false;
};
constexpr unsigned kMaxListStamps = 64;
void install_device(ID3D12Device* device, const wchar_t* dump_dir);
void note_pso(void* list, ID3D12PipelineState* pso);
UINT stamp_before(ID3D12GraphicsCommandList* list, UINT x, UINT y, UINT z, bool indirect);
void stamp_after(ID3D12GraphicsCommandList* list, UINT query);
unsigned take_stamps(ID3D12CommandList* const* lists, UINT count, DispatchStamp* out, unsigned cap);
void resolve_stamps(ID3D12GraphicsCommandList* into, const DispatchStamp* stamps, unsigned n);
bool stamp_ticks(UINT query, std::uint64_t* begin, std::uint64_t* end); // after the submission's fence
unsigned shader_count();

// ---- stage attribution (2026-10-23; AvatarVR_gpuprof.txt "stages") ----
// Loads scripts\AvatarVR_stagemap.txt and switches the stamps from per-dispatch to per-stage runs (see the .cpp). Call
// before the first list is watched. Stage stamps carry shader = 0x5A << 56 | stage id.
bool set_stage_mode(const wchar_t* map_path);
bool stage_mode();
const char* stage_name(unsigned id);
unsigned stage_count();
std::uint64_t stage_overflow_count();
}
