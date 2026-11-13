#include "render_activity.hpp"
#include "../camera/scanner_geometry.hpp"
#include <windows.h>
#include <MinHook.h>
#include <atomic>
#include <cstring>
#include <cwchar>
#include <mutex>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <cstdio>
#include <vector>

namespace {
std::atomic<bool> enabled{false};
std::mutex mutex;
struct Record {
  render_activity::Sample sample;
  ID3D12PipelineState* pso = nullptr; // currently bound (identity only, no reference held)
  render_activity::DispatchStamp stamps[render_activity::kMaxListStamps];
  unsigned stamp_n = 0;
  int type = -1; // D3D12_COMMAND_LIST_TYPE once known
  std::uint64_t stage = ~0ull;  // stage mode: stage key of the open segment (~0 = none yet)
  UINT open = ~0u;              // stage mode: first query of the open segment's stamp pair
};
std::unordered_map<void*, Record> records;
std::uintptr_t base = 0, limit = 0;
using CL = ID3D12GraphicsCommandList;
using Reset = HRESULT(STDMETHODCALLTYPE*)(CL*, ID3D12CommandAllocator*, ID3D12PipelineState*);
using Copy = void(STDMETHODCALLTYPE*)(CL*, const D3D12_TEXTURE_COPY_LOCATION*, UINT, UINT, UINT, const D3D12_TEXTURE_COPY_LOCATION*, const D3D12_BOX*);
using CopyResource = void(STDMETHODCALLTYPE*)(CL*, ID3D12Resource*, ID3D12Resource*);
using Rays = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList4*, const D3D12_DISPATCH_RAYS_DESC*);
using Build = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList4*, const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC*, UINT, const D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_DESC*);
Reset reset = nullptr;
Copy copy = nullptr; CopyResource copy_resource = nullptr; Rays rays = nullptr; Build build = nullptr;

// Caller holds mutex. A discarded/unsubmitted list cannot grow diagnostic storage indefinitely.
Record* find_or_add(void* list) {
  auto it = records.find(list);
  if (it == records.end()) {
    if (records.size() >= 4096) return nullptr;
    it = records.emplace(list, Record{}).first;
    void* frames[12] = {};
    const USHORT n = RtlCaptureStackBackTrace(2, 12, frames, nullptr);
    unsigned used = 0;
    for (USHORT i = 0; i < n && used < 4; ++i) {
      const auto a = reinterpret_cast<std::uintptr_t>(frames[i]);
      if (a >= base && a < limit) it->second.sample.stack[used++] = a-base;
    }
  }
  return &it->second;
}
void record(void* list, unsigned kind, UINT x = 0, UINT y = 0, UINT z = 0) {
  if (!enabled.load(std::memory_order_relaxed)) return;
  std::lock_guard<std::mutex> lock(mutex);
  Record* r = find_or_add(list);
  if (r == nullptr) return;
  auto& s = r->sample;
  ++s.count[kind];
  if ((kind == 1 || kind == 4) && s.x == 0) { s.x = x; s.y = y; s.z = z; }
}
HRESULT STDMETHODCALLTYPE reset_hook(CL* l, ID3D12CommandAllocator* a, ID3D12PipelineState* p) {
  const HRESULT hr = reset(l,a,p);
  if(SUCCEEDED(hr)) scanner_geometry::reset(l);
  if (SUCCEEDED(hr) && enabled.load()) {
    std::lock_guard<std::mutex> lock(mutex);
    records.erase(l);
    if (p != nullptr) if (Record* r = find_or_add(l)) r->pso = p;
  }
  return hr;
}
void STDMETHODCALLTYPE copy_hook(CL* l, const D3D12_TEXTURE_COPY_LOCATION* d, UINT x, UINT y, UINT z, const D3D12_TEXTURE_COPY_LOCATION* s, const D3D12_BOX* b) {
  record(l,2); copy(l,d,x,y,z,s,b);
}
void STDMETHODCALLTYPE copy_resource_hook(CL* l, ID3D12Resource* d, ID3D12Resource* s) { record(l,2); copy_resource(l,d,s); }
void STDMETHODCALLTYPE rays_hook(ID3D12GraphicsCommandList4* l, const D3D12_DISPATCH_RAYS_DESC* d) {
  if (d != nullptr) record(l,4,d->Width,d->Height,d->Depth);
  rays(l,d);
}
void STDMETHODCALLTYPE build_hook(ID3D12GraphicsCommandList4* l, const D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC* d, UINT n,
                                const D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_DESC* p) {
  record(l,5); build(l,d,n,p);
}
// ---- census shader identity and per-dispatch timestamps ----
using CreateCompute = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, const D3D12_COMPUTE_PIPELINE_STATE_DESC*, REFIID, void**);
CreateCompute create_compute = nullptr;
std::mutex shader_mutex;
std::unordered_map<const void*, std::uint64_t> pso_shader; // PSO identity -> shader hash
std::unordered_map<std::uint64_t, bool> shaders_seen;
std::wstring dump_path;
constexpr UINT kQueries = 1u << 17; // even and a power of two: a stamp pair never wraps
ID3D12QueryHeap* stamp_heap = nullptr;
ID3D12Resource* stamp_readback = nullptr;
const std::uint64_t* stamp_data = nullptr; // persistently mapped readback
std::atomic<UINT> next_query{0};
std::atomic<unsigned> coverage{0};

std::uint64_t shader_hash(const void* code, SIZE_T size) {
  const auto* b = static_cast<const unsigned char*>(code);
  if (size >= 20 && b[0] == 'D' && b[1] == 'X' && b[2] == 'B' && b[3] == 'C') {
    std::uint64_t digest = 0;
    std::memcpy(&digest, b + 4, sizeof(digest));
    if (digest != 0) return digest; // container digest (what dxc -dumpbin reports as the shader hash)
  }
  std::uint64_t h = 1469598103934665603ull;
  for (SIZE_T i = 0; i < size; ++i) { h ^= b[i]; h *= 1099511628211ull; }
  return h;
}
void dump_shader(std::uint64_t hash, const void* code, SIZE_T size) {
  if (dump_path.empty() || shaders_seen.size() > 4096) return;
  wchar_t name[32];
  swprintf(name, 32, L"\\%016llX.cso", static_cast<unsigned long long>(hash));
  const std::wstring path = dump_path + name;
  HANDLE f = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (f == INVALID_HANDLE_VALUE) return; // exists from an earlier run, or unwritable
  DWORD wrote = 0;
  WriteFile(f, code, static_cast<DWORD>(size), &wrote, nullptr);
  CloseHandle(f);
}
HRESULT STDMETHODCALLTYPE create_compute_hook(ID3D12Device* d, const D3D12_COMPUTE_PIPELINE_STATE_DESC* desc, REFIID riid, void** out) {
  const HRESULT hr = create_compute(d, desc, riid, out);
  if (SUCCEEDED(hr) && desc != nullptr && out != nullptr && *out != nullptr && desc->CS.pShaderBytecode != nullptr &&
      desc->CS.BytecodeLength > 0 && riid == __uuidof(ID3D12PipelineState)) {
    const std::uint64_t h = shader_hash(desc->CS.pShaderBytecode, desc->CS.BytecodeLength);
    std::lock_guard<std::mutex> lock(shader_mutex);
    pso_shader[*out] = h;
    if (shaders_seen.emplace(h, true).second) dump_shader(h, desc->CS.pShaderBytecode, desc->CS.BytecodeLength);
  }
  return hr;
}
std::uint64_t shader_of(const void* pso) {
  if (pso == nullptr) return 0;
  std::lock_guard<std::mutex> lock(shader_mutex);
  const auto it = pso_shader.find(pso);
  return it == pso_shader.end() ? 0 : it->second;
}

// ---- stage attribution (2026-10-23, AvatarVR_gpuprof.txt "stages") ----
// At each SetPipelineState the recording stack is walked; the deepest engine function named from a profiler scope string
// (scripts\AvatarVR_stagemap.txt, tools/re/make_stagemap.py) names the stage. A list gets a timestamp pair per run of
// consecutive commands with the same stage (opened at the PSO change that enters the stage, closed at the next stage
// change or at Close). Time between a pair includes overlap with earlier unfinished work: attribution, not isolated cost.
std::atomic<bool> stage_on{false};
struct StageFn { std::uint32_t rva, size; std::uint16_t id; };
std::vector<StageFn> stage_fns;          // sorted by rva; written once before stage_on is set
std::vector<std::string> stage_names{"(no scope on stack)"};
std::atomic<std::uint64_t> stage_overflows{0};
using Close = HRESULT(STDMETHODCALLTYPE*)(CL*);
Close close_list = nullptr;

// Stage key: 0x5A << 56 | outer stage id << 16 | deepest stage id (the next distinct stage function further up the stack
// names the caller, e.g. RenderStaticMesh < RenderShadows), or 0x5B << 56 | first afop RVA when no stage function is on
// the stack (for a later sdk_lookup).
std::uint64_t stage_of_stack() {
  void* frames[48] = {};
  const USHORT n = RtlCaptureStackBackTrace(3, 48, frames, nullptr);
  std::uint16_t deep = 0, outer = 0;
  std::uint32_t first_rva = 0;
  for (USHORT i = 0; i < n; ++i) {
    const auto a = reinterpret_cast<std::uintptr_t>(frames[i]);
    if (a < base || a >= limit) continue;
    const auto rva = static_cast<std::uint32_t>(a - base);
    if (first_rva == 0) first_rva = rva;
    auto it = std::upper_bound(stage_fns.begin(), stage_fns.end(), rva,
                               [](std::uint32_t v, const StageFn& f) { return v < f.rva; });
    if (it == stage_fns.begin()) continue;
    --it;
    if (rva - it->rva >= it->size) continue;
    if (deep == 0) {
      deep = it->id;
    } else if (it->id != deep) {
      outer = it->id;
      break;
    }
  }
  if (deep == 0) return (0x5Bull << 56) | (first_rva & 0xFFFFFFFull);
  return (0x5Aull << 56) | (static_cast<std::uint64_t>(outer) << 16) | deep;
}
// Caller holds mutex. Closes the open segment of r (if any) in list.
void close_segment(CL* list, Record* r) {
  if (r->open == ~0u) return;
  list->EndQuery(stamp_heap, D3D12_QUERY_TYPE_TIMESTAMP, r->open + 1);
  r->open = ~0u;
}
void stage_transition(CL* list) {
  if (stamp_heap == nullptr) return;
  const std::uint64_t id = stage_of_stack();
  std::lock_guard<std::mutex> lock(mutex);
  Record* r = find_or_add(list);
  if (r == nullptr || r->stage == id) return;
  if (r->type < 0) r->type = static_cast<int>(list->GetType());
  if (r->type != D3D12_COMMAND_LIST_TYPE_DIRECT && r->type != D3D12_COMMAND_LIST_TYPE_COMPUTE) return;
  close_segment(list, r);
  r->stage = id;
  if (r->stamp_n >= render_activity::kMaxListStamps) {
    stage_overflows.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const UINT q = next_query.fetch_add(2, std::memory_order_relaxed) & (kQueries - 1);
  auto& st = r->stamps[r->stamp_n++];
  st = render_activity::DispatchStamp{};
  st.shader = id;
  st.query = q;
  r->open = q;
  list->EndQuery(stamp_heap, D3D12_QUERY_TYPE_TIMESTAMP, q);
}
HRESULT STDMETHODCALLTYPE close_hook(CL* l) {
  if (stage_on.load(std::memory_order_relaxed) && enabled.load(std::memory_order_relaxed)) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto it = records.find(l);
    if (it != records.end()) close_segment(l, &it->second);
  }
  return close_list(l);
}

template<class T> void hook(void* object, unsigned slot, unsigned bit, void* detour, T* original) {
  if (*original != nullptr) return;
  void* target = (*reinterpret_cast<void***>(object))[slot];
  if (MH_CreateHook(target, detour, reinterpret_cast<void**>(original)) == MH_OK && MH_EnableHook(target) == MH_OK)
    coverage.fetch_or(1u << bit);
}
}
namespace render_activity {
void watch(ID3D12GraphicsCommandList* list) {
  if (list == nullptr) return;
  static std::mutex install_mutex;
  std::lock_guard<std::mutex> lock(install_mutex);
  if (base == 0) {
    const auto* module = reinterpret_cast<const unsigned char*>(GetModuleHandleW(nullptr));
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(module);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(module + dos->e_lfanew);
    base = reinterpret_cast<std::uintptr_t>(module); limit = base + nt->OptionalHeader.SizeOfImage;
  }
  // Slots verified against Windows SDK 10.0.26100.0 ID3D12GraphicsCommandList4Vtbl.
  hook(list,10,0,reinterpret_cast<void*>(&reset_hook),&reset);
  if (stage_on.load()) hook(list,9,7,reinterpret_cast<void*>(&close_hook),&close_list);
  hook(list,16,1,reinterpret_cast<void*>(&copy_hook),&copy);
  hook(list,17,2,reinterpret_cast<void*>(&copy_resource_hook),&copy_resource);
  ID3D12GraphicsCommandList4* list4 = nullptr;
  if (SUCCEEDED(list->QueryInterface(IID_PPV_ARGS(&list4))) && list4 != nullptr) {
    hook(list4,72,4,reinterpret_cast<void*>(&build_hook),&build);
    hook(list4,76,3,reinterpret_cast<void*>(&rays_hook),&rays);
    list4->Release();
  }
}
void set_enabled(bool on) { enabled.store(on); }
unsigned extra_hook_coverage() { return coverage.load(); }
void note(void* list, unsigned kind, UINT x, UINT y, UINT z) { record(list,kind,x,y,z); }
Sample take(ID3D12CommandList* const* lists, UINT count) {
  Sample total;
  if (!enabled.load()) return total;
  std::lock_guard<std::mutex> lock(mutex);
  for (UINT i = 0; i < count; ++i) {
    const auto it = records.find(lists[i]);
    if (it == records.end()) continue;
    const Sample& s = it->second.sample;
    for (unsigned k = 0; k < kKinds; ++k) total.count[k] += s.count[k];
    if (total.stack[0] == 0) for (unsigned k = 0; k < 4; ++k) total.stack[k] = s.stack[k];
    if (total.x == 0) { total.x = s.x; total.y = s.y; total.z = s.z; }
    // Do not erase here: an engine bundle may be referenced more than once.
    // Reset invalidates its recording, and the cap bounds lifetime storage.
  }
  return total;
}
void install_device(ID3D12Device* device, const wchar_t* dir) {
  if (device == nullptr || stamp_heap != nullptr) return;
  if (dir != nullptr) {
    dump_path = dir;
    CreateDirectoryW(dump_path.c_str(), nullptr);
  }
  // Slot 11 = ID3D12Device::CreateComputePipelineState (Windows SDK 10.0.26100.0 ID3D12DeviceVtbl).
  hook(device,11,5,reinterpret_cast<void*>(&create_compute_hook),&create_compute);
  D3D12_QUERY_HEAP_DESC qd = {};
  qd.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
  qd.Count = kQueries;
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC rd = {};
  rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  rd.Width = static_cast<UINT64>(kQueries) * sizeof(UINT64);
  rd.Height = 1; rd.DepthOrArraySize = 1; rd.MipLevels = 1; rd.SampleDesc.Count = 1;
  rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  ID3D12QueryHeap* heap = nullptr;
  ID3D12Resource* readback = nullptr;
  void* mapped = nullptr;
  if (SUCCEEDED(device->CreateQueryHeap(&qd, IID_PPV_ARGS(&heap))) &&
      SUCCEEDED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                IID_PPV_ARGS(&readback))) &&
      SUCCEEDED(readback->Map(0, nullptr, &mapped)) && mapped != nullptr) {
    stamp_data = static_cast<const std::uint64_t*>(mapped);
    stamp_readback = readback;
    stamp_heap = heap; // published last: stamp_before checks it
    coverage.fetch_or(1u << 6);
    return;
  }
  if (readback != nullptr) readback->Release();
  if (heap != nullptr) heap->Release();
}
void note_pso(void* list, ID3D12PipelineState* pso) {
  if (!enabled.load(std::memory_order_relaxed)) return;
  if (stage_on.load(std::memory_order_relaxed)) {
    stage_transition(static_cast<CL*>(list));
    return;
  }
  std::lock_guard<std::mutex> lock(mutex);
  if (Record* r = find_or_add(list)) r->pso = pso;
}
UINT stamp_before(ID3D12GraphicsCommandList* list, UINT x, UINT y, UINT z, bool indirect) {
  if (!enabled.load(std::memory_order_relaxed) || stamp_heap == nullptr || list == nullptr) return ~0u;
  if (stage_on.load(std::memory_order_relaxed)) return ~0u;  // stage mode stamps stage runs, not dispatches
  UINT q = ~0u;
  {
    std::lock_guard<std::mutex> lock(mutex);
    Record* r = find_or_add(list);
    if (r == nullptr || r->stamp_n >= kMaxListStamps) return ~0u;
    if (r->type < 0) r->type = static_cast<int>(list->GetType());
    // Bundles cannot record queries; copy lists have no dispatches.
    if (r->type != D3D12_COMMAND_LIST_TYPE_DIRECT && r->type != D3D12_COMMAND_LIST_TYPE_COMPUTE) return ~0u;
    q = next_query.fetch_add(2, std::memory_order_relaxed) & (kQueries - 1);
    auto& st = r->stamps[r->stamp_n++];
    st.shader = shader_of(r->pso);
    if (indirect && st.shader == 0) {
      // Performance pass 2026-10-22: an indirect dispatch with an unknown PSO took ~31 ms/frame outdoors (late
      // submission 40). Tag it by its recording call site instead: 0xEE << 56 | first afop RVA << 28 | second RVA
      // (28 bits each), so the census prints which engine functions issued it.
      void* frames[16] = {};
      const USHORT n = RtlCaptureStackBackTrace(1, 16, frames, nullptr);
      std::uint64_t rva[2] = {0, 0};
      unsigned used = 0;
      for (USHORT i = 0; i < n && used < 2; ++i) {
        const auto a = reinterpret_cast<std::uintptr_t>(frames[i]);
        if (a >= base && a < limit) rva[used++] = (a - base) & 0xFFFFFFFull;
      }
      if (used > 0) st.shader = (0xEEull << 56) | (rva[0] << 28) | rva[1];
    }
    st.x = x; st.y = y; st.z = z; st.query = q; st.indirect = indirect;
  }
  list->EndQuery(stamp_heap, D3D12_QUERY_TYPE_TIMESTAMP, q);
  return q;
}
void stamp_after(ID3D12GraphicsCommandList* list, UINT query) {
  if (query == ~0u || list == nullptr) return;
  list->EndQuery(stamp_heap, D3D12_QUERY_TYPE_TIMESTAMP, query + 1);
}
unsigned take_stamps(ID3D12CommandList* const* lists, UINT count, DispatchStamp* out, unsigned cap) {
  if (!enabled.load() || stamp_heap == nullptr) return 0;
  unsigned n = 0;
  std::lock_guard<std::mutex> lock(mutex);
  for (UINT i = 0; i < count && n < cap; ++i) {
    const auto it = records.find(lists[i]);
    if (it == records.end()) continue;
    for (unsigned k = 0; k < it->second.stamp_n && n < cap; ++k) out[n++] = it->second.stamps[k];
  }
  return n;
}
void resolve_stamps(ID3D12GraphicsCommandList* into, const DispatchStamp* stamps, unsigned n) {
  if (into == nullptr || stamp_heap == nullptr) return;
  for (unsigned i = 0; i < n;) {
    if (stamps[i].query == ~0u) { ++i; continue; }
    // Merge consecutive pairs (one list's stamps are usually adjacent in the ring).
    unsigned j = i + 1;
    while (j < n && stamps[j].query == stamps[j - 1].query + 2) ++j;
    const UINT first = stamps[i].query, total = (stamps[j - 1].query + 2) - first;
    into->ResolveQueryData(stamp_heap, D3D12_QUERY_TYPE_TIMESTAMP, first, total, stamp_readback,
                           static_cast<UINT64>(first) * sizeof(UINT64));
    i = j;
  }
}
bool stamp_ticks(UINT query, std::uint64_t* begin, std::uint64_t* end) {
  if (stamp_data == nullptr || query == ~0u || query + 1 >= kQueries) return false;
  *begin = stamp_data[query];
  *end = stamp_data[query + 1];
  return *begin != 0 && *end >= *begin;
}
bool set_stage_mode(const wchar_t* map_path) {
  FILE* f = _wfopen(map_path, L"r");
  if (f == nullptr) return false;
  char line[512];
  while (std::fgets(line, sizeof(line), f) != nullptr) {
    unsigned rva = 0, size = 0;
    char name[400] = {};
    if (line[0] == '#' || std::sscanf(line, "%x %x %399s", &rva, &size, name) != 3) continue;
    if (stage_names.size() >= 0xFFFE) break;
    stage_fns.push_back({rva, size, static_cast<std::uint16_t>(stage_names.size())});
    stage_names.emplace_back(name);
  }
  std::fclose(f);
  std::sort(stage_fns.begin(), stage_fns.end(), [](const StageFn& a, const StageFn& b) { return a.rva < b.rva; });
  stage_on.store(!stage_fns.empty());
  return stage_on.load();
}
bool stage_mode() { return stage_on.load(std::memory_order_relaxed); }
const char* stage_name(unsigned id) { return id < stage_names.size() ? stage_names[id].c_str() : "?"; }
unsigned stage_count() { return static_cast<unsigned>(stage_fns.size()); }
std::uint64_t stage_overflow_count() { return stage_overflows.load(); }
unsigned shader_count() {
  std::lock_guard<std::mutex> lock(shader_mutex);
  return static_cast<unsigned>(shaders_seen.size());
}
}
