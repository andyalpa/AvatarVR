#include "mv_repair.hpp"
#include "mv_repair_shaders.hpp"

#include <d3dcommon.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace {

void (*g_log)(const char* fmt, ...) = nullptr;
#define LOGF(...)            \
  do {                       \
    if (g_log != nullptr) {  \
      g_log(__VA_ARGS__);    \
    }                        \
  } while (0)

constexpr int kConstCount = 28;  // 4 uint + 8 projection + 9 rotation + 3 translation + padding
constexpr size_t kConstsFloats = 114;

std::atomic<bool> g_enabled{false};
std::mutex g_mutex;
bool g_init_tried = false, g_ready = false;
ID3D12Device* g_device = nullptr;
ID3D12RootSignature* g_root = nullptr;
ID3D12PipelineState* g_pso = nullptr;
ID3D12PipelineState* g_pso_fast = nullptr;  // tiled variant, identical output (mv_repair_shaders.hpp); null if it failed
std::atomic<bool> g_fast{false};            // "mvfast": use the tiled variant
std::atomic<bool> g_ab{false};              // "mvab": alternate reference / tiled every 5 s (profiling A/B)

struct Buffer {
  ID3D12Resource* res = nullptr;
  D3D12_RESOURCE_STATES state = D3D12_RESOURCE_STATE_COMMON;
  UINT64 size = 0;
};
Buffer g_mvL, g_dL, g_dR, g_mvR, g_out;
// Lists may still be unsubmitted/in flight when the repair buffers grow.
// Retain replaced ownership. Geometric capacity growth bounds retained memory
// below the sum of the largest active capacities over the process lifetime.
std::vector<Buffer> g_retired_buffers;
UINT64 g_retired_bytes = 0;
D3D12_PLACED_SUBRESOURCE_FOOTPRINT g_mv_fp = {}, g_d_fp = {};
UINT g_w = 0, g_h = 0;
DXGI_FORMAT g_mv_fmt = DXGI_FORMAT_UNKNOWN, g_d_fmt = DXGI_FORMAT_UNKNOWN;
float g_left_consts[kConstsFloats] = {};
std::uint64_t g_left_seq = 0, g_used_seq = 0;
ULONGLONG g_left_at = 0;  // when the left copy was recorded; older than 100 ms (a pause, a load) is never used
std::atomic<std::uint64_t> g_repairs_ref{0}, g_repairs_fast{0};
std::atomic<std::uint64_t> g_left_captures{0}, g_repairs{0}, g_skipped_stale{0}, g_skipped_size{0}, g_failed{0};

// Bounded samples recorded in the existing engine list, no new lists or waits.
// Each slot owns its completion fence so different queues cannot complete each
// other's samples. An unsubmitted/failed sample stays occupied and is never reused.
std::atomic<bool> g_profile{false};
constexpr UINT kTimingSlots = 16;
ID3D12QueryHeap* g_timing_heap = nullptr;
ID3D12Resource* g_timing_readback = nullptr;
UINT64* g_timing_data = nullptr;
struct TimingSlot {
  ID3D12CommandList* list = nullptr;
  ID3D12Fence* fence = nullptr;
  bool submitted = false, right = false, fast = false;
  UINT w = 0, h = 0;
  UINT64 frequency = 0, serial = 0;
};
TimingSlot g_timing[kTimingSlots];
ULONGLONG g_sample_at[2] = {};
std::atomic<ULONGLONG> g_profile_started_at{0};
double g_timing_ms[4] = {};
UINT g_timing_samples[2] = {};
UINT g_timing_w[2] = {}, g_timing_h[2] = {};
UINT64 g_timing_calls[2] = {};
double g_variant_ms[2] = {};    // right-eye dispatch only, by variant (0 reference, 1 tiled)
UINT g_variant_samples[2] = {};

int timing_begin(ID3D12GraphicsCommandList* list, bool right) {
  ++g_timing_calls[right];
  const ULONGLONG now = GetTickCount64();
  if (!g_profile.load() || now - g_sample_at[right] < 500) return -1;
  const auto type = list->GetType();
  if (type != D3D12_COMMAND_LIST_TYPE_DIRECT && type != D3D12_COMMAND_LIST_TYPE_COMPUTE) {
    static bool warned = false;
    if (!warned) { LOGF("mvrepair GPU: unsupported command list type %u", static_cast<unsigned>(type)); warned = true; }
    return -1;
  }
  // Log a setup failure once and stop profiling; repair itself remains intact.
  auto failure = [](const char* operation, HRESULT hr) {
    LOGF("mvrepair GPU: %s failed 0x%08X; profiling disabled, repair unchanged", operation, static_cast<unsigned>(hr));
    g_profile.store(false);
  };
  if (g_timing_heap == nullptr) {
    D3D12_QUERY_HEAP_DESC q = {}; q.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP; q.Count = kTimingSlots * 4;
    HRESULT hr = g_device->CreateQueryHeap(&q, IID_PPV_ARGS(&g_timing_heap));
    if (FAILED(hr)) { failure("CreateQueryHeap", hr); return -1; }
    D3D12_HEAP_PROPERTIES hp = {}; hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd = {}; rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    rd.Width = kTimingSlots * 4 * sizeof(UINT64); rd.Height = 1; rd.DepthOrArraySize = 1;
    rd.MipLevels = 1; rd.SampleDesc.Count = 1; rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    hr = g_device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST,
                                          nullptr, IID_PPV_ARGS(&g_timing_readback));
    if (FAILED(hr)) {
      failure("CreateCommittedResource(readback)", hr);
      g_timing_heap->Release(); g_timing_heap = nullptr; return -1;
    }
    const D3D12_RANGE range = {0, static_cast<SIZE_T>(rd.Width)};
    hr = g_timing_readback->Map(0, &range, reinterpret_cast<void**>(&g_timing_data));
    if (FAILED(hr)) {
      failure("Map(readback)", hr);
      g_timing_readback->Release(); g_timing_readback = nullptr;
      g_timing_heap->Release(); g_timing_heap = nullptr; return -1;
    }
  }
  for (UINT i = 0; i < kTimingSlots; ++i) {
    auto& s = g_timing[i];
    if (s.list != nullptr) continue;
    if (s.fence == nullptr) {
      const HRESULT hr = g_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&s.fence));
      if (FAILED(hr)) { failure("CreateFence", hr); return -1; }
    }
    s.list = list; s.submitted = false; s.right = right; s.w = g_w; s.h = g_h;
    ++s.serial; g_sample_at[right] = now;
    if (g_profile_started_at == 0) g_profile_started_at = now;
    list->EndQuery(g_timing_heap, D3D12_QUERY_TYPE_TIMESTAMP, i * 4);
    return static_cast<int>(i);
  }
  return -1;
}

void timing_stamp(ID3D12GraphicsCommandList* list, int slot, UINT phase) {
  if (slot >= 0) list->EndQuery(g_timing_heap, D3D12_QUERY_TYPE_TIMESTAMP, static_cast<UINT>(slot) * 4 + phase);
}
void timing_end(ID3D12GraphicsCommandList* list, int slot, UINT count) {
  if (slot >= 0) list->ResolveQueryData(g_timing_heap, D3D12_QUERY_TYPE_TIMESTAMP, static_cast<UINT>(slot) * 4,
                                     count, g_timing_readback, static_cast<UINT64>(slot) * 4 * sizeof(UINT64));
}

void barrier(ID3D12GraphicsCommandList* list, ID3D12Resource* res, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  if (before == after) {
    return;
  }
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = res;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = before;
  b.Transition.StateAfter = after;
  list->ResourceBarrier(1, &b);
}

void to_state(ID3D12GraphicsCommandList* list, Buffer& b, D3D12_RESOURCE_STATES s) {
  barrier(list, b.res, b.state, s);
  b.state = s;
}

void release(Buffer& b) {
  if (b.res != nullptr) {
    b.res->Release();
  }
  b = Buffer{};
}

bool make_buffer(Buffer& b, UINT64 size, bool uav) {
  if (b.res != nullptr && b.size >= size) {
    return true;
  }
  UINT64 capacity = 16ull * 1024 * 1024;
  while (capacity < size) capacity *= 2;
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = capacity;
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  desc.Flags = uav ? D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS : D3D12_RESOURCE_FLAG_NONE;
  ID3D12Resource* next = nullptr;
  if (FAILED(g_device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                               IID_PPV_ARGS(&next)))) {
    return false;
  }
  if (b.res != nullptr) {
    g_retired_buffers.push_back(b);
    g_retired_bytes += b.size;
    LOGF("mvrepair: retained %zu prior buffers, %.1f MiB total (resize lifetime safety)",
         g_retired_buffers.size(), static_cast<double>(g_retired_bytes)/(1024.0*1024.0));
  }
  b.res = next;
  b.size = capacity;
  b.state = D3D12_RESOURCE_STATE_COMMON;
  return true;
}

using D3DCompileFn = HRESULT(WINAPI*)(LPCVOID, SIZE_T, LPCSTR, const D3D_SHADER_MACRO*, ID3DInclude*, LPCSTR, LPCSTR, UINT, UINT,
                                      ID3DBlob**, ID3DBlob**);

bool init_pipeline(ID3D12Device* device) {
  g_init_tried = true;
  HMODULE compiler = LoadLibraryW(L"d3dcompiler_47.dll");
  HMODULE d3d12 = GetModuleHandleW(L"d3d12.dll");
  auto compile = compiler != nullptr ? reinterpret_cast<D3DCompileFn>(GetProcAddress(compiler, "D3DCompile")) : nullptr;
  auto serialize = d3d12 != nullptr ? reinterpret_cast<PFN_D3D12_SERIALIZE_ROOT_SIGNATURE>(GetProcAddress(d3d12, "D3D12SerializeRootSignature"))
                                    : nullptr;
  if (compile == nullptr || serialize == nullptr) {
    LOGF("mvrepair: d3dcompiler_47 / D3D12SerializeRootSignature not available (%p %p)", reinterpret_cast<void*>(compile),
         reinterpret_cast<void*>(serialize));
    return false;
  }
  ID3DBlob* cs = nullptr;
  ID3DBlob* errors = nullptr;
  constexpr UINT kOptimizationLevel3 = 1u << 15;  // D3DCOMPILE_OPTIMIZATION_LEVEL3
  using namespace mv_repair_shaders;
  HRESULT hr = compile(kShaderReference, sizeof(kShaderReference) - 1, "mv_repair", nullptr, nullptr, "main", "cs_5_0",
                       kOptimizationLevel3, 0, &cs, &errors);
  if (FAILED(hr)) {
    LOGF("mvrepair: shader compile failed 0x%08X %s", static_cast<unsigned>(hr),
         errors != nullptr ? static_cast<const char*>(errors->GetBufferPointer()) : "");
    if (errors != nullptr) errors->Release();
    return false;
  }
  if (errors != nullptr) {
    errors->Release();
    errors = nullptr;
  }
  D3D12_ROOT_PARAMETER params[6] = {};
  params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
  params[0].Constants.ShaderRegister = 0;
  params[0].Constants.Num32BitValues = kConstCount;
  for (int i = 0; i < 4; ++i) {
    params[1 + i].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1 + i].Descriptor.ShaderRegister = static_cast<UINT>(i);
  }
  params[5].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
  params[5].Descriptor.ShaderRegister = 0;
  D3D12_ROOT_SIGNATURE_DESC rs = {};
  rs.NumParameters = 6;
  rs.pParameters = params;
  ID3DBlob* blob = nullptr;
  hr = serialize(&rs, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &errors);
  if (FAILED(hr)) {
    LOGF("mvrepair: root signature serialize failed 0x%08X", static_cast<unsigned>(hr));
    if (errors != nullptr) errors->Release();
    cs->Release();
    return false;
  }
  hr = device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g_root));
  blob->Release();
  if (FAILED(hr)) {
    LOGF("mvrepair: CreateRootSignature failed 0x%08X", static_cast<unsigned>(hr));
    cs->Release();
    return false;
  }
  D3D12_COMPUTE_PIPELINE_STATE_DESC pd = {};
  pd.pRootSignature = g_root;
  pd.CS = {cs->GetBufferPointer(), cs->GetBufferSize()};
  hr = device->CreateComputePipelineState(&pd, IID_PPV_ARGS(&g_pso));
  cs->Release();
  if (FAILED(hr)) {
    LOGF("mvrepair: CreateComputePipelineState failed 0x%08X", static_cast<unsigned>(hr));
    return false;
  }
  // The tiled variant is optional: any failure leaves the reference shader in use.
  ID3DBlob* fast = nullptr;
  hr = compile(kShaderTiled, sizeof(kShaderTiled) - 1, "mv_repair_tiled", nullptr, nullptr, "main", "cs_5_0",
               kOptimizationLevel3 | kTiledExtraFlags, 0, &fast, &errors);
  if (SUCCEEDED(hr)) {
    pd.CS = {fast->GetBufferPointer(), fast->GetBufferSize()};
    hr = device->CreateComputePipelineState(&pd, IID_PPV_ARGS(&g_pso_fast));
    fast->Release();
  }
  if (FAILED(hr)) {
    g_pso_fast = nullptr;
    LOGF("mvrepair: tiled variant unavailable 0x%08X %s; reference shader only", static_cast<unsigned>(hr),
         errors != nullptr ? static_cast<const char*>(errors->GetBufferPointer()) : "");
  }
  if (errors != nullptr) errors->Release();
  g_device = device;
  g_device->AddRef();
  LOGF("mvrepair: pipeline ready");
  return true;
}

// Buffers for one eye size; footprints from the textures themselves.
bool ensure_sizes(ID3D12Resource* mv, ID3D12Resource* depth) {
  const D3D12_RESOURCE_DESC md = mv->GetDesc(), dd = depth->GetDesc();
  if (md.Width != dd.Width || md.Height != dd.Height) {
    return false;
  }
  if (g_w == md.Width && g_h == md.Height && g_mv_fmt == md.Format && g_d_fmt == dd.Format && g_out.res != nullptr) {
    return true;
  }
  UINT64 mv_total = 0, d_total = 0;
  g_left_seq = g_used_seq = 0; // invalidate pairing if allocating a new layout fails midway
  g_device->GetCopyableFootprints(&md, 0, 1, 0, &g_mv_fp, nullptr, nullptr, &mv_total);
  g_device->GetCopyableFootprints(&dd, 0, 1, 0, &g_d_fp, nullptr, nullptr, &d_total);
  // Whole rows for the shader (the last row of a footprint is not padded).
  const UINT64 mv_size = static_cast<UINT64>(g_mv_fp.Footprint.RowPitch) * md.Height;
  const UINT64 d_size = static_cast<UINT64>(g_d_fp.Footprint.RowPitch) * dd.Height;
  if (!make_buffer(g_mvL, mv_size, false) || !make_buffer(g_mvR, mv_size, false) || !make_buffer(g_dL, d_size, false) ||
      !make_buffer(g_dR, d_size, false) || !make_buffer(g_out, mv_size, true)) {
    LOGF("mvrepair: buffer creation failed (%llu / %llu bytes)", static_cast<unsigned long long>(mv_size),
         static_cast<unsigned long long>(d_size));
    return false;
  }
  g_w = static_cast<UINT>(md.Width);
  g_h = md.Height;
  g_mv_fmt = md.Format;
  g_d_fmt = dd.Format;
  LOGF("mvrepair: buffers for %ux%u (motion fmt %d pitch %u, depth fmt %d pitch %u)", g_w, g_h, static_cast<int>(g_mv_fmt),
       g_mv_fp.Footprint.RowPitch, static_cast<int>(g_d_fmt), g_d_fp.Footprint.RowPitch);
  return true;
}

void copy_tex_to_buffer(ID3D12GraphicsCommandList* list, ID3D12Resource* tex, D3D12_RESOURCE_STATES tex_state, Buffer& dst,
                        const D3D12_PLACED_SUBRESOURCE_FOOTPRINT& fp) {
  barrier(list, tex, tex_state, D3D12_RESOURCE_STATE_COPY_SOURCE);
  to_state(list, dst, D3D12_RESOURCE_STATE_COPY_DEST);
  D3D12_TEXTURE_COPY_LOCATION d = {};
  d.pResource = dst.res;
  d.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  d.PlacedFootprint = fp;
  d.PlacedFootprint.Offset = 0;
  D3D12_TEXTURE_COPY_LOCATION s = {};
  s.pResource = tex;
  s.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  s.SubresourceIndex = 0;
  list->CopyTextureRegion(&d, 0, 0, 0, &s, nullptr);
  barrier(list, tex, D3D12_RESOURCE_STATE_COPY_SOURCE, tex_state);
}

// Only the layout seen in gameplay (2026-10-09): R16G16_FLOAT motion vectors and R32G8X24 depth, both declared in
// NON_PIXEL | PIXEL_SHADER_RESOURCE. Anything else (front end, loading, other paths) is left alone: a wrong "before"
// state in our barriers would be undefined behaviour on the GPU.
constexpr D3D12_RESOURCE_STATES kExpectedState =
    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
std::atomic<std::uint64_t> g_skipped_layout{0};

bool valid(const mv_repair::EyeInput& e) {
  if (e.mv == nullptr || e.depth == nullptr || e.consts == nullptr) {
    return false;
  }
  if (e.mv_state != kExpectedState || e.depth_state != kExpectedState || e.mv->GetDesc().Format != DXGI_FORMAT_R16G16_FLOAT ||
      e.depth->GetDesc().Format != DXGI_FORMAT_R32G8X24_TYPELESS) {
    g_skipped_layout.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

}  // namespace

namespace mv_repair {

void set_log(void (*log)(const char* fmt, ...)) { g_log = log; }

void set_enabled(bool on) {
  if (on != g_enabled.exchange(on)) {
    LOGF("mvrepair: %s", on ? "ON" : "off");
  }
}

bool enabled() { return g_enabled.load(std::memory_order_relaxed); }

void set_variant(bool fast, bool ab) {
  if (fast != g_fast.exchange(fast)) LOGF("mvrepair: tiled variant %s", fast ? "ON (mvfast)" : "off");
  if (ab != g_ab.exchange(ab)) LOGF("mvrepair: A/B alternation %s", ab ? "ON (mvab, 5 s per variant)" : "off");
}

void set_profiling(bool on) { g_profile.store(on); }
void submitted(ID3D12CommandQueue* queue, UINT count, ID3D12CommandList* const* lists) {
  if (!g_profile.load() || queue == nullptr) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  for (auto& s : g_timing) {
    if (s.list == nullptr || s.submitted) continue;
    for (UINT i = 0; i < count; ++i) {
      if (lists[i] != s.list) continue;
      s.submitted = true;
      if (FAILED(queue->GetTimestampFrequency(&s.frequency)) || s.frequency == 0 ||
          FAILED(queue->Signal(s.fence, s.serial))) s.frequency = 0; // leave slot occupied on failure
      break;
    }
  }
}

void capture_left(ID3D12GraphicsCommandList* list, const EyeInput& left) {
  if (!enabled() || list == nullptr || !valid(left)) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_ready) {
    if (g_init_tried) {
      return;
    }
    ID3D12Device* device = nullptr;
    if (FAILED(left.mv->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) {
      return;
    }
    g_ready = init_pipeline(device);
    device->Release();
    if (!g_ready) {
      return;
    }
  }
  if (!ensure_sizes(left.mv, left.depth)) {
    g_skipped_size.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const int timing = timing_begin(list, false);
  copy_tex_to_buffer(list, left.mv, left.mv_state, g_mvL, g_mv_fp);
  copy_tex_to_buffer(list, left.depth, left.depth_state, g_dL, g_d_fp);
  timing_stamp(list, timing, 1);
  timing_end(list, timing, 2);
  std::memcpy(g_left_consts, left.consts, sizeof(g_left_consts));
  ++g_left_seq;
  g_left_at = GetTickCount64();
  g_left_captures.fetch_add(1, std::memory_order_relaxed);
}

void repair_right(ID3D12GraphicsCommandList* list, const EyeInput& right) {
  if (!enabled() || list == nullptr || !valid(right)) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_ready || g_left_seq == 0) {
    return;
  }
  if (g_left_seq == g_used_seq || GetTickCount64() - g_left_at > 100) {
    g_skipped_stale.fetch_add(1, std::memory_order_relaxed);  // no new left image since the last repair, or too old
    return;
  }
  const D3D12_RESOURCE_DESC md = right.mv->GetDesc(), dd = right.depth->GetDesc();
  if (md.Width != g_w || md.Height != g_h || md.Format != g_mv_fmt || dd.Format != g_d_fmt) {
    g_skipped_size.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  g_used_seq = g_left_seq;
  // Constants: projections and the right-camera -> left-camera transform (mv_repair_shaders::build_constants).
  mv_repair_shaders::Constants k = {};
  static_assert(sizeof(k) / 4 <= kConstCount, "root constants");
  if (!mv_repair_shaders::build_constants(right.consts, g_left_consts, g_w, g_h, g_mv_fp.Footprint.RowPitch,
                                          g_d_fp.Footprint.RowPitch, &k)) {
    g_failed.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  // Variant: "mvfast" selects the tiled shader; "mvab" alternates both every 5 s for a same-scene timing A/B.
  bool use_fast = g_fast.load(std::memory_order_relaxed);
  if (g_ab.load(std::memory_order_relaxed)) use_fast = (GetTickCount64() / 5000) % 2 == 1;
  ID3D12PipelineState* pso = use_fast && g_pso_fast != nullptr ? g_pso_fast : g_pso;
  use_fast = pso == g_pso_fast;
  const int timing = timing_begin(list, true);
  if (timing >= 0) g_timing[timing].fast = use_fast;
  copy_tex_to_buffer(list, right.mv, right.mv_state, g_mvR, g_mv_fp);
  copy_tex_to_buffer(list, right.depth, right.depth_state, g_dR, g_d_fp);
  const auto srv = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
  to_state(list, g_mvL, srv);
  to_state(list, g_dL, srv);
  to_state(list, g_mvR, srv);
  to_state(list, g_dR, srv);
  to_state(list, g_out, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
  list->SetComputeRootSignature(g_root);
  list->SetPipelineState(pso);
  list->SetComputeRoot32BitConstants(0, sizeof(k) / 4, &k, 0);
  list->SetComputeRootShaderResourceView(1, g_mvL.res->GetGPUVirtualAddress());
  list->SetComputeRootShaderResourceView(2, g_dL.res->GetGPUVirtualAddress());
  list->SetComputeRootShaderResourceView(3, g_dR.res->GetGPUVirtualAddress());
  list->SetComputeRootShaderResourceView(4, g_mvR.res->GetGPUVirtualAddress());
  list->SetComputeRootUnorderedAccessView(5, g_out.res->GetGPUVirtualAddress());
  timing_stamp(list, timing, 1);
  list->Dispatch((g_w + 7) / 8, (g_h + 7) / 8, 1);
  timing_stamp(list, timing, 2);
  to_state(list, g_out, D3D12_RESOURCE_STATE_COPY_SOURCE);
  // Repaired vectors back into the right eye's motion-vector texture, which DLSS reads next.
  barrier(list, right.mv, right.mv_state, D3D12_RESOURCE_STATE_COPY_DEST);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = right.mv;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  dst.SubresourceIndex = 0;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = g_out.res;
  src.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  src.PlacedFootprint = g_mv_fp;
  src.PlacedFootprint.Offset = 0;
  list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  barrier(list, right.mv, D3D12_RESOURCE_STATE_COPY_DEST, right.mv_state);
  timing_stamp(list, timing, 3);
  timing_end(list, timing, 4);
  g_repairs.fetch_add(1, std::memory_order_relaxed);
  (use_fast ? g_repairs_fast : g_repairs_ref).fetch_add(1, std::memory_order_relaxed);
}

void report() {
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (UINT i = 0; i < kTimingSlots; ++i) {
      auto& s = g_timing[i];
      if (s.list == nullptr || !s.submitted || s.frequency == 0) continue;
      const UINT64 completed = s.fence->GetCompletedValue();
      if (completed == ~UINT64{0} || completed < s.serial) continue;
      const UINT64* t = g_timing_data + i * 4;
      const UINT count = s.right ? 4 : 2;
      bool valid_ticks = true;
      for (UINT j = 1; j < count; ++j) valid_ticks &= t[j] >= t[j-1];
      if (valid_ticks) {
        const unsigned eye = s.right ? 1 : 0;
        // Keep resolution changes out of the same sample average.
        if (g_timing_w[eye] != s.w || g_timing_h[eye] != s.h) {
          g_timing_samples[eye] = 0;
          for (UINT j = 0; j < count-1; ++j) g_timing_ms[s.right ? j+1 : 0] = 0;
          g_timing_w[eye] = s.w; g_timing_h[eye] = s.h;
        }
        for (UINT j = 0; j < count-1; ++j)
          g_timing_ms[s.right ? j+1 : 0] += static_cast<double>(t[j+1]-t[j]) * 1000.0 / s.frequency;
        ++g_timing_samples[eye];
        if (s.right) {
          g_variant_ms[s.fast ? 1 : 0] += static_cast<double>(t[2]-t[1]) * 1000.0 / s.frequency;
          ++g_variant_samples[s.fast ? 1 : 0];
        }
      }
      s.list = nullptr;
    }
  }
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (g_profile_started_at != 0 && now - g_profile_started_at > 60000 && g_profile.exchange(false)) {
    LOGF("mvrepair GPU profiling: automatic stop after 60s of gameplay; repair remains enabled");
  }
  if (now - last < 5000 || (!enabled() && g_repairs.load() == 0)) {
    return;
  }
  last = now;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    unsigned pending = 0, submitted_slots = 0;
    for (const auto& s : g_timing) {
      if (s.list != nullptr) { ++pending; if (s.submitted) ++submitted_slots; }
    }
    LOGF("mvrepair GPU health: enabled %d, timing calls L=%llu R=%llu, pending %u submitted %u, heap %d",
         g_profile.load() ? 1 : 0, static_cast<unsigned long long>(g_timing_calls[0]),
         static_cast<unsigned long long>(g_timing_calls[1]), pending, submitted_slots, g_timing_heap != nullptr ? 1 : 0);
    if (g_timing_samples[0] > 0 || g_timing_samples[1] > 0) {
      const double nl = g_timing_samples[0] ? g_timing_samples[0] : 1;
      const double nr = g_timing_samples[1] ? g_timing_samples[1] : 1;
      LOGF("mvrepair GPU: left copies %.3f ms (%u samples %ux%u), right input %.3f, dispatch %.3f, output %.3f ms (%u samples %ux%u)",
           g_timing_ms[0]/nl, g_timing_samples[0], g_timing_w[0], g_timing_h[0], g_timing_ms[1]/nr,
           g_timing_ms[2]/nr, g_timing_ms[3]/nr, g_timing_samples[1], g_timing_w[1], g_timing_h[1]);
      for (auto& ms : g_timing_ms) ms = 0;
      g_timing_samples[0] = g_timing_samples[1] = 0;
    }
    // Cumulative over the profiling window so the A/B averages use every sample of both variants.
    if (g_variant_samples[0] > 0 || g_variant_samples[1] > 0) {
      LOGF("mvrepair GPU A/B: reference dispatch %.3f ms (%u samples), tiled dispatch %.3f ms (%u samples) at %ux%u",
           g_variant_samples[0] ? g_variant_ms[0] / g_variant_samples[0] : 0.0, g_variant_samples[0],
           g_variant_samples[1] ? g_variant_ms[1] / g_variant_samples[1] : 0.0, g_variant_samples[1], g_w, g_h);
    }
  }
  LOGF("mvrepair: variant %s%s (tiled %s), repairs reference %llu / tiled %llu",
       g_ab.load() ? "A/B alternate" : g_fast.load() ? "tiled" : "reference", g_ab.load() ? " every 5 s" : "",
       g_pso_fast != nullptr ? "ready" : "unavailable", static_cast<unsigned long long>(g_repairs_ref.load()),
       static_cast<unsigned long long>(g_repairs_fast.load()));
  LOGF("mvrepair: %s, ready %d, left captures %llu, repairs %llu, skipped stale %llu, size %llu, layout %llu, failed %llu (%ux%u)",
       enabled() ? "on" : "off", g_ready ? 1 : 0, static_cast<unsigned long long>(g_left_captures.load()),
       static_cast<unsigned long long>(g_repairs.load()), static_cast<unsigned long long>(g_skipped_stale.load()),
       static_cast<unsigned long long>(g_skipped_size.load()), static_cast<unsigned long long>(g_skipped_layout.load()),
       static_cast<unsigned long long>(g_failed.load()), g_w, g_h);
}

}  // namespace mv_repair
