#include "frame_gen.hpp"
#include "frame_gen_math.hpp"

#include <d3dcommon.h>

#include <atomic>
#include <cstring>
#include <mutex>
#include <vector>

// See frame_gen.hpp. Pipeline creation follows xr_depth.cpp / mv_repair.cpp (run-time D3DCompile); the warp pass is a
// full-screen triangle like the RCAS sharpen pass in xr_runtime.cpp and repeats its RCAS math so generated frames are
// sharpened like the real ones.
namespace {

void (*g_log)(const char* fmt, ...) = nullptr;
#define LOGF(...)                 \
  do {                            \
    if (g_log != nullptr) {       \
      g_log(__VA_ARGS__);         \
    }                             \
  } while (0)

std::atomic<bool> g_enabled{false};
std::mutex g_mutex;

constexpr D3D12_RESOURCE_STATES kExpectedState =
    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

// ---- capture banks (render threads) ----
struct Plane {
  ID3D12Resource* buffer = nullptr;
  UINT64 size = 0;
  UINT pitch = 0;
  frame_gen::Rect rect = {0, 0, 0, 0};
};
struct CapView {
  Plane mv, depth;
  float scale[2] = {0.5f, -0.5f};
};
CapView g_cap[3][2];
// HUD-free copies of the eye images, per capture bank and view (taken at the first UI draw).
struct PreHud {
  ID3D12Resource* tex = nullptr;
  ID3D12Resource* src = nullptr;  // the game resource it was copied from (identity check at keep_real)
  bool ok = false;
};
PreHud g_pre[3][2];
bool g_bank_hud[2] = {false, false};  // colour bank has matching HUD-free copies for both eyes
std::atomic<uint64_t> g_prehud_caps{0}, g_hud_frames{0}, g_hud_mismatch{0};
frame_gen_math::Banks g_banks;
std::vector<ID3D12Resource*> g_retired;  // replaced resources: lists that reference them may still be in flight

// ---- colour banks (Present thread) ----
ID3D12Resource* g_color[2][2] = {};  // [bank][eye]
uint32_t g_color_w = 0, g_color_h = 0;
DXGI_FORMAT g_color_fmt[2] = {DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_UNKNOWN};
int g_pending_bank = -1, g_ready_bank = -1;
int g_bank_cap[2] = {-1, -1};          // capture bank that belongs to each colour bank
int g_bank_view[2][2] = {{0, 1}, {0, 1}};  // [bank][eye] -> view
uint64_t g_bank_copy_value[2] = {0, 0};
uint64_t g_bank_worker_value[2] = {0, 0};
ID3D12Fence* g_copy_fence = nullptr;  // xr_runtime's, not owned

// ---- worker objects ----
ID3D12CommandQueue* g_wqueue = nullptr;
ID3D12CommandAllocator* g_walloc[3] = {};
uint64_t g_walloc_value[3] = {0, 0, 0};
ID3D12GraphicsCommandList* g_wlist = nullptr;
ID3D12Fence* g_wfence = nullptr;
uint64_t g_wfence_next = 0;
HANDLE g_wevent = nullptr;
int g_wslot = 0;
ID3D12QueryHeap* g_wqueries = nullptr;
ID3D12Resource* g_wreadback = nullptr;
const uint64_t* g_wstamps = nullptr;
bool g_wstamp_pending[3] = {false, false, false};
UINT64 g_wfreq = 0;
double g_gpu_ms_sum = 0.0;
uint64_t g_gpu_samples = 0;
bool g_worker_failed = false;
// Brightness of generated frames (diagnostic, 2026-11-12 dark-flash report): once a second a 128x128 centre crop of
// both generated eye images is copied to a readback buffer and averaged after its fence completes.
constexpr UINT kLumaCrop = 128;
ID3D12Resource* g_luma_rb = nullptr;
uint64_t g_luma_fence = 0;  // 0 = no copy pending
ULONGLONG g_luma_last = 0;
double g_luma[2] = {-1.0, -1.0};
void luma_poll() {
  if (g_luma_fence == 0 || g_luma_rb == nullptr || g_wfence == nullptr || g_wfence->GetCompletedValue() < g_luma_fence) return;
  g_luma_fence = 0;
  const D3D12_RANGE range = {0, static_cast<SIZE_T>(2) * kLumaCrop * kLumaCrop * 4};
  void* mapped = nullptr;
  if (FAILED(g_luma_rb->Map(0, &range, &mapped))) return;
  const auto* bytes = static_cast<const unsigned char*>(mapped);
  for (int e = 0; e < 2; ++e) {
    const unsigned char* b = bytes + static_cast<size_t>(e) * kLumaCrop * kLumaCrop * 4;
    uint64_t sum = 0;
    for (UINT i = 0; i < kLumaCrop * kLumaCrop; ++i) sum += static_cast<uint64_t>(b[i * 4]) + b[i * 4 + 1] + b[i * 4 + 2];
    g_luma[e] = static_cast<double>(sum) / (3.0 * kLumaCrop * kLumaCrop);
  }
  const D3D12_RANGE none = {0, 0};
  g_luma_rb->Unmap(0, &none);
}

ID3D12RootSignature* g_root = nullptr;
ID3D12PipelineState* g_pso[2] = {};
DXGI_FORMAT g_pso_fmt[2] = {DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_UNKNOWN};
ID3D12DescriptorHeap* g_srv_heap = nullptr;  // [bank * 2 + eye], shader visible
ID3D12DescriptorHeap* g_rtv_heap = nullptr;  // [eye]
UINT g_srv_stride = 0, g_rtv_stride = 0;
bool g_pipeline_failed = false;
ID3DBlob* g_vs = nullptr;
ID3DBlob* g_ps = nullptr;

std::atomic<uint64_t> g_captures[2] = {}, g_skipped_layout{0}, g_kept{0}, g_not_kept{0}, g_generated{0}, g_gen_failures{0};

constexpr char kShader[] = R"(
cbuffer C : register(b0) {
  uint out_w; uint out_h; float k; float con;
  uint mv_pitch; uint mv_x; uint mv_y; uint mv_w;
  uint mv_h; uint d_pitch; uint d_x; uint d_y;
  uint d_w; uint d_h; float scale_x; float scale_y;
  uint srgb_out; uint use_hud; uint pad1; uint pad2;
};
Texture2D<float4> T : register(t0);
ByteAddressBuffer MV : register(t1);
ByteAddressBuffer D : register(t2);
Texture2D<float4> P : register(t3);  // HUD-free copy of the last real frame (use_hud)
SamplerState S : register(s0);
float4 vs(uint id : SV_VertexID) : SV_Position {
  float2 q = float2((id << 1) & 2, id & 2);
  return float4(q * float2(2, -2) + float2(-1, 1), 0, 1);
}
float2 mv_at(int2 t) {
  t = clamp(t, int2(0, 0), int2(mv_w - 1, mv_h - 1));
  uint v = MV.Load((mv_y + t.y) * mv_pitch + (mv_x + t.x) * 4);
  return float2(f16tof32(v & 0xffff), f16tof32(v >> 16));
}
float depth_at(float2 uv) {
  int2 t = clamp(int2(uv * float2(d_w, d_h)), int2(0, 0), int2(d_w - 1, d_h - 1));
  return asfloat(D.Load((d_y + t.y) * d_pitch + (d_x + t.x) * 4));
}
float3 to_linear(float3 c) { return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4); }
float3 at(float2 uv) { return use_hud != 0 ? P.SampleLevel(S, saturate(uv), 0).rgb : T.SampleLevel(S, saturate(uv), 0).rgb; }
float4 ps(float4 pos : SV_Position) : SV_Target {
  float2 uv = pos.xy / float2(out_w, out_h);
  // Nearest surface of a 3x3 neighbourhood in the motion-vector grid (reverse-Z: larger = nearer): a moving
  // foreground object keeps its own motion at its edges instead of the background's.
  int2 c = int2(uv * float2(mv_w, mv_h));
  float best = -1.0;
  int2 pick = c;
  [unroll] for (int y = -1; y <= 1; ++y) {
    [unroll] for (int x = -1; x <= 1; ++x) {
      int2 t = c + int2(x, y);
      float d = depth_at((float2(t) + 0.5) / float2(mv_w, mv_h));
      if (d > best) { best = d; pick = t; }
    }
  }
  float2 m = mv_at(pick) * float2(scale_x, scale_y);  // UV delta, current -> previous
  float2 src = uv + k * m;                             // gather from where the content was
  float2 px = 1.0 / float2(out_w, out_h);
  float3 e = at(src);
  float3 o = e;
  if (con > 0.0) {  // RCAS (FSR 1), as the sharpen pass applies to real frames
    float3 b = at(src + float2(0, -px.y)), d = at(src + float2(-px.x, 0)), f = at(src + float2(px.x, 0)), h = at(src + float2(0, px.y));
    float3 mn4 = min(min(b, d), min(f, h)), mx4 = max(max(b, d), max(f, h));
    float3 hitMin = min(mn4, e) / max(4.0 * mx4, 1e-5);
    float3 hitMax = (1.0 - max(mx4, e)) / min(4.0 * mn4 - 4.0, -1e-5);
    float3 lobeC = max(-hitMin, hitMax);
    float lobe = max(-(0.25 - 1.0 / 16.0), min(max(lobeC.r, max(lobeC.g, lobeC.b)), 0.0)) * con;
    o = saturate((lobe * (b + d + f + h) + e) / (4.0 * lobe + 1.0));
  }
  if (use_hud != 0) {
    // The HUD is where the final image differs from the HUD-free copy of the same frame: keep it unwarped there.
    int3 ip = int3(int2(pos.xy), 0);
    float3 fin = T.Load(ip).rgb, pre = P.Load(ip).rgb, dd = abs(fin - pre);
    o = lerp(o, fin, saturate(max(dd.r, max(dd.g, dd.b)) * 24.0));
  }
  return float4(srgb_out != 0 ? to_linear(o) : o, 1.0);
}
)";

using D3DCompileFn = HRESULT(WINAPI*)(LPCVOID, SIZE_T, LPCSTR, const D3D_SHADER_MACRO*, ID3DInclude*, LPCSTR, LPCSTR, UINT, UINT,
                                      ID3DBlob**, ID3DBlob**);

DXGI_FORMAT unorm_view(DXGI_FORMAT f) {
  switch (f) {
    case DXGI_FORMAT_R8G8B8A8_TYPELESS:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
      return DXGI_FORMAT_R8G8B8A8_UNORM;
    case DXGI_FORMAT_B8G8R8A8_TYPELESS:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
      return DXGI_FORMAT_B8G8R8A8_UNORM;
    case DXGI_FORMAT_R10G10B10A2_TYPELESS:
      return DXGI_FORMAT_R10G10B10A2_UNORM;
    default:
      return f;
  }
}
bool srgb_typed(DXGI_FORMAT f) { return f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB; }

bool ensure_buffer(ID3D12Device* device, Plane& p, UINT64 size) {
  if (p.buffer != nullptr && p.size >= size) return true;
  UINT64 capacity = 8ull * 1024 * 1024;
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
  ID3D12Resource* next = nullptr;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                             IID_PPV_ARGS(&next)))) {
    return false;
  }
  next->SetName(L"AvatarVR_frame_gen_capture");
  if (p.buffer != nullptr) g_retired.push_back(p.buffer);
  p.buffer = next;
  p.size = capacity;
  return true;
}

void barrier(ID3D12GraphicsCommandList* list, ID3D12Resource* res, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = res;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = before;
  b.Transition.StateAfter = after;
  list->ResourceBarrier(1, &b);
}

// Copies plane 0 of `tex` into `p` (footprint), restoring the declared state. Caller holds g_mutex.
bool copy_plane(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* tex, D3D12_RESOURCE_STATES state, Plane& p,
                frame_gen::Rect ext) {
  const D3D12_RESOURCE_DESC desc = tex->GetDesc();
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp = {};
  UINT64 total = 0;
  device->GetCopyableFootprints(&desc, 0, 1, 0, &fp, nullptr, nullptr, &total);
  const UINT64 size = static_cast<UINT64>(fp.Footprint.RowPitch) * desc.Height;
  if (fp.Footprint.RowPitch == 0 || !ensure_buffer(device, p, size)) return false;
  barrier(list, tex, state, D3D12_RESOURCE_STATE_COPY_SOURCE);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = p.buffer;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = fp;
  dst.PlacedFootprint.Offset = 0;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = tex;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  src.SubresourceIndex = 0;
  list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  barrier(list, tex, D3D12_RESOURCE_STATE_COPY_SOURCE, state);
  p.pitch = fp.Footprint.RowPitch;
  const uint32_t tw = static_cast<uint32_t>(desc.Width), th = desc.Height;
  if (ext.w == 0 || ext.h == 0 || ext.x >= tw || ext.y >= th) {
    p.rect = {0, 0, tw, th};
  } else {
    p.rect = {ext.x, ext.y, ext.w <= tw - ext.x ? ext.w : tw - ext.x, ext.h <= th - ext.y ? ext.h : th - ext.y};
  }
  return true;
}

bool ensure_pipeline(ID3D12Device* device, DXGI_FORMAT rtv_fmt, ID3D12PipelineState** out) {
  for (int i = 0; i < 2; ++i) {
    if (g_pso[i] != nullptr && g_pso_fmt[i] == rtv_fmt) { *out = g_pso[i]; return true; }
  }
  if (g_pipeline_failed) return false;
  if (g_vs == nullptr) {
    HMODULE compiler = LoadLibraryW(L"d3dcompiler_47.dll");
    auto compile = compiler != nullptr ? reinterpret_cast<D3DCompileFn>(GetProcAddress(compiler, "D3DCompile")) : nullptr;
    ID3DBlob* errors = nullptr;
    constexpr UINT kOptimizationLevel3 = 1u << 15;
    HRESULT hr = compile != nullptr ? compile(kShader, sizeof(kShader) - 1, "frame_gen", nullptr, nullptr, "vs", "vs_5_0",
                                              kOptimizationLevel3, 0, &g_vs, &errors)
                                    : E_FAIL;
    if (SUCCEEDED(hr)) {
      if (errors != nullptr) { errors->Release(); errors = nullptr; }
      hr = compile(kShader, sizeof(kShader) - 1, "frame_gen", nullptr, nullptr, "ps", "ps_5_0", kOptimizationLevel3, 0, &g_ps, &errors);
    }
    if (FAILED(hr)) {
      LOGF("framegen: shader compile failed 0x%08X %s", static_cast<unsigned>(hr),
           errors != nullptr ? static_cast<const char*>(errors->GetBufferPointer()) : "(no d3dcompiler_47)");
      if (errors != nullptr) errors->Release();
      g_pipeline_failed = true;
      return false;
    }
    if (errors != nullptr) errors->Release();
  }
  if (g_root == nullptr) {
    HMODULE d3d12 = GetModuleHandleW(L"d3d12.dll");
    auto serialize = d3d12 != nullptr
                         ? reinterpret_cast<PFN_D3D12_SERIALIZE_ROOT_SIGNATURE>(GetProcAddress(d3d12, "D3D12SerializeRootSignature"))
                         : nullptr;
    D3D12_DESCRIPTOR_RANGE range = {D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0, 0};
    D3D12_DESCRIPTOR_RANGE range_hud = {D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 3, 0, 0};
    D3D12_ROOT_PARAMETER params[5] = {};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.ShaderRegister = 0;
    params[0].Constants.Num32BitValues = 20;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[1].DescriptorTable.NumDescriptorRanges = 1;
    params[1].DescriptorTable.pDescriptorRanges = &range;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    params[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[2].Descriptor.ShaderRegister = 1;
    params[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    params[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[3].Descriptor.ShaderRegister = 2;
    params[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    params[4].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[4].DescriptorTable.NumDescriptorRanges = 1;
    params[4].DescriptorTable.pDescriptorRanges = &range_hud;
    params[4].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    D3D12_STATIC_SAMPLER_DESC sampler = {};
    sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    sampler.AddressU = sampler.AddressV = sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.MaxLOD = D3D12_FLOAT32_MAX;
    sampler.ShaderRegister = 0;
    sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    D3D12_ROOT_SIGNATURE_DESC rs = {5, params, 1, &sampler, D3D12_ROOT_SIGNATURE_FLAG_NONE};
    ID3DBlob* blob = nullptr;
    ID3DBlob* errors = nullptr;
    if (serialize == nullptr || FAILED(serialize(&rs, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &errors)) ||
        FAILED(device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g_root)))) {
      LOGF("framegen: root signature failed");
      if (blob != nullptr) blob->Release();
      if (errors != nullptr) errors->Release();
      g_pipeline_failed = true;
      return false;
    }
    blob->Release();
    if (errors != nullptr) errors->Release();
  }
  D3D12_GRAPHICS_PIPELINE_STATE_DESC pd = {};
  pd.pRootSignature = g_root;
  pd.VS = {g_vs->GetBufferPointer(), g_vs->GetBufferSize()};
  pd.PS = {g_ps->GetBufferPointer(), g_ps->GetBufferSize()};
  pd.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
  pd.SampleMask = UINT_MAX;
  pd.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
  pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
  pd.RasterizerState.DepthClipEnable = TRUE;
  pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  pd.NumRenderTargets = 1;
  pd.RTVFormats[0] = rtv_fmt;
  pd.SampleDesc.Count = 1;
  ID3D12PipelineState* pso = nullptr;
  if (FAILED(device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&pso)))) {
    LOGF("framegen: pipeline for format %d failed", static_cast<int>(rtv_fmt));
    g_pipeline_failed = true;
    return false;
  }
  const int i = g_pso[0] == nullptr ? 0 : 1;  // a third format replaces slot 1; the old PSO is kept alive (not released)
  g_pso[i] = pso;
  g_pso_fmt[i] = rtv_fmt;
  LOGF("framegen: warp pipeline ready (target format %d)", static_cast<int>(rtv_fmt));
  *out = pso;
  return true;
}

void write_srvs(ID3D12Device* device);

bool ensure_worker(ID3D12Device* device) {
  if (g_wqueue != nullptr) return true;
  if (g_worker_failed) return false;
  D3D12_COMMAND_QUEUE_DESC qd = {};
  qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  qd.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
  bool ok = SUCCEEDED(device->CreateCommandQueue(&qd, IID_PPV_ARGS(&g_wqueue)));
  for (int i = 0; ok && i < 3; ++i) {
    ok = SUCCEEDED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g_walloc[i])));
  }
  ok = ok && SUCCEEDED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_walloc[0], nullptr, IID_PPV_ARGS(&g_wlist)));
  if (ok) g_wlist->Close();
  ok = ok && SUCCEEDED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_wfence)));
  if (ok) g_wevent = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  if (ok) {
    D3D12_QUERY_HEAP_DESC q = {};
    q.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
    q.Count = 6;
    D3D12_HEAP_PROPERTIES hp = {};
    hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd = {};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    rd.Width = 6 * sizeof(UINT64);
    rd.Height = 1;
    rd.DepthOrArraySize = 1;
    rd.MipLevels = 1;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    void* mapped = nullptr;
    if (SUCCEEDED(device->CreateQueryHeap(&q, IID_PPV_ARGS(&g_wqueries))) &&
        SUCCEEDED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                  IID_PPV_ARGS(&g_wreadback))) &&
        SUCCEEDED(g_wreadback->Map(0, nullptr, &mapped))) {
      g_wstamps = static_cast<const uint64_t*>(mapped);
      g_wqueue->GetTimestampFrequency(&g_wfreq);
    }  // timing is optional
  }
  if (ok && g_srv_heap == nullptr) {
    D3D12_DESCRIPTOR_HEAP_DESC hd = {D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, 10, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE, 0};
    D3D12_DESCRIPTOR_HEAP_DESC rd = {D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 2, D3D12_DESCRIPTOR_HEAP_FLAG_NONE, 0};
    ok = SUCCEEDED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_srv_heap))) &&
         SUCCEEDED(device->CreateDescriptorHeap(&rd, IID_PPV_ARGS(&g_rtv_heap)));
    g_srv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    g_rtv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    // The colour banks are created first (Present thread, keep_real), before this heap existed, so their views were
    // never written and the warp read nothing: every generated frame was black (LIVE 2026-11-12: gen_luma 0.0/0.0,
    // AUTHOR: dark blink every generated frame). Write them now.
    if (ok) write_srvs(device);
  }
  if (!ok) {
    LOGF("framegen: worker queue setup failed; frame generation stays off");
    g_worker_failed = true;
    return false;
  }
  LOGF("framegen: worker queue ready (DIRECT, HIGH priority)");
  return true;
}

void write_srvs(ID3D12Device* device) {
  if (g_srv_heap == nullptr) return;
  for (int b = 0; b < 2; ++b) {
    for (int e = 0; e < 2; ++e) {
      if (g_color[b][e] == nullptr) continue;
      D3D12_SHADER_RESOURCE_VIEW_DESC sd = {};
      sd.Format = unorm_view(g_color_fmt[e]);
      sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
      sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
      sd.Texture2D.MipLevels = 1;
      D3D12_CPU_DESCRIPTOR_HANDLE h = g_srv_heap->GetCPUDescriptorHandleForHeapStart();
      h.ptr += static_cast<SIZE_T>(b * 2 + e) * g_srv_stride;
      device->CreateShaderResourceView(g_color[b][e], &sd, h);
    }
  }
  for (int b = 0; b < 3; ++b) {
    for (int v = 0; v < 2; ++v) {
      if (g_pre[b][v].tex == nullptr) continue;
      D3D12_SHADER_RESOURCE_VIEW_DESC sd = {};
      sd.Format = unorm_view(g_pre[b][v].tex->GetDesc().Format);
      sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
      sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
      sd.Texture2D.MipLevels = 1;
      D3D12_CPU_DESCRIPTOR_HANDLE h = g_srv_heap->GetCPUDescriptorHandleForHeapStart();
      h.ptr += static_cast<SIZE_T>(4 + b * 2 + v) * g_srv_stride;
      device->CreateShaderResourceView(g_pre[b][v].tex, &sd, h);
    }
  }
}

bool ensure_colors(ID3D12Device* device, ID3D12Resource* const src[2], uint32_t w, uint32_t h) {
  const DXGI_FORMAT f0 = src[0]->GetDesc().Format, f1 = src[1]->GetDesc().Format;
  if (g_color[0][0] != nullptr && g_color_w == w && g_color_h == h && g_color_fmt[0] == f0 && g_color_fmt[1] == f1) return true;
  for (auto& bank : g_color) {
    for (ID3D12Resource*& t : bank) {
      if (t != nullptr) g_retired.push_back(t);
      t = nullptr;
    }
  }
  g_ready_bank = g_pending_bank = -1;
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  for (int b = 0; b < 2; ++b) {
    for (int e = 0; e < 2; ++e) {
      D3D12_RESOURCE_DESC d = {};
      d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
      d.Width = w;
      d.Height = h;
      d.DepthOrArraySize = 1;
      d.MipLevels = 1;
      d.Format = e == 0 ? f0 : f1;
      d.SampleDesc.Count = 1;
      d.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS;
      if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                                 IID_PPV_ARGS(&g_color[b][e])))) {
        LOGF("framegen: colour texture %ux%u format %d failed", w, h, static_cast<int>(d.Format));
        g_color[b][e] = nullptr;
        return false;
      }
      g_color[b][e]->SetName(L"AvatarVR_frame_gen_last_real");
    }
  }
  g_color_w = w;
  g_color_h = h;
  g_color_fmt[0] = f0;
  g_color_fmt[1] = f1;
  write_srvs(device);
  LOGF("framegen: last-real textures %ux%u (formats %d / %d)", w, h, static_cast<int>(f0), static_cast<int>(f1));
  return true;
}

}  // namespace

namespace frame_gen {

void set_log(void (*log)(const char* fmt, ...)) { g_log = log; }

void set_enabled(bool on) {
  if (on != g_enabled.exchange(on)) LOGF("framegen: %s", on ? "ON (in-mod frame extrapolation)" : "off");
}

bool enabled() { return g_enabled.load(std::memory_order_relaxed); }

void capture(int view, ID3D12GraphicsCommandList* list, ID3D12Resource* mv, D3D12_RESOURCE_STATES mv_state, ID3D12Resource* depth,
             D3D12_RESOURCE_STATES depth_state, const float mvec_scale[2], Rect mv_ext, Rect depth_ext) {
  if (!enabled() || list == nullptr || mv == nullptr || depth == nullptr || view < 0 || view > 1) return;
  if (mv_state != kExpectedState || depth_state != kExpectedState || mv->GetDesc().Format != DXGI_FORMAT_R16G16_FLOAT ||
      depth->GetDesc().Format != DXGI_FORMAT_R32G8X24_TYPELESS) {
    g_skipped_layout.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(mv->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  CapView& c = g_cap[g_banks.write][view];
  const bool ok = copy_plane(device, list, mv, mv_state, c.mv, mv_ext) && copy_plane(device, list, depth, depth_state, c.depth, depth_ext);
  device->Release();
  if (!ok) {
    g_skipped_layout.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (mvec_scale != nullptr && std::isfinite(mvec_scale[0]) && std::isfinite(mvec_scale[1]) && mvec_scale[0] != 0.f) {
    c.scale[0] = mvec_scale[0];
    c.scale[1] = mvec_scale[1];
  }
  g_banks.mark(view);
  g_captures[view].fetch_add(1, std::memory_order_relaxed);
}

void capture_prehud(int view, ID3D12GraphicsCommandList* list, ID3D12Resource* target) {
  if (!enabled() || list == nullptr || target == nullptr || view < 0 || view > 1) return;
  const D3D12_RESOURCE_DESC td = target->GetDesc();
  if (td.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || td.SampleDesc.Count != 1 || td.DepthOrArraySize != 1) return;
  ID3D12Device* device = nullptr;
  if (FAILED(target->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  PreHud& p = g_pre[g_banks.write][view];
  bool ok = true;
  if (p.tex == nullptr || p.tex->GetDesc().Width != td.Width || p.tex->GetDesc().Height != td.Height ||
      p.tex->GetDesc().Format != td.Format) {
    if (p.tex != nullptr) g_retired.push_back(p.tex);
    p.tex = nullptr;
    D3D12_HEAP_PROPERTIES heap = {};
    heap.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC d = {};
    d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width = td.Width;
    d.Height = td.Height;
    d.DepthOrArraySize = 1;
    d.MipLevels = 1;
    d.Format = td.Format;
    d.SampleDesc.Count = 1;
    d.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS;
    ok = SUCCEEDED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                                   IID_PPV_ARGS(&p.tex)));
    if (ok) {
      p.tex->SetName(L"AvatarVR_frame_gen_prehud");
      write_srvs(device);
      LOGF("framegen: HUD-free copy %llux%u format %d (view %d)", static_cast<unsigned long long>(td.Width), td.Height,
           static_cast<int>(td.Format), view);
    } else {
      p.tex = nullptr;
    }
  }
  device->Release();
  if (!ok) return;
  barrier(list, target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = p.tex;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  D3D12_TEXTURE_COPY_LOCATION s = {};
  s.pResource = target;
  s.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  list->CopyTextureRegion(&dst, 0, 0, 0, &s, nullptr);  // simultaneous access: promoted to COPY_DEST, decays to COMMON
  barrier(list, target, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
  p.src = target;
  p.ok = true;
  g_prehud_caps.fetch_add(1, std::memory_order_relaxed);
}

bool keep_real(ID3D12Device* device, ID3D12GraphicsCommandList* list, ID3D12Resource* const src[2], const int view_of_eye[2],
               uint32_t w, uint32_t h) {
  if (!enabled() || device == nullptr || list == nullptr || src[0] == nullptr || src[1] == nullptr || w == 0 || h == 0) return false;
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!ensure_colors(device, src, w, h)) {
    g_not_kept.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const int b = g_ready_bank < 0 ? 0 : 1 - g_ready_bank;
  for (int e = 0; e < 2; ++e) {
    D3D12_TEXTURE_COPY_LOCATION s = {};
    s.pResource = src[e];
    s.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    D3D12_TEXTURE_COPY_LOCATION d = {};
    d.pResource = g_color[b][e];
    d.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    const D3D12_BOX box = {0, 0, 0, w, h, 1};
    list->CopyTextureRegion(&d, 0, 0, 0, &s, &box);  // simultaneous access: promoted to COPY_DEST, decays to COMMON
    g_bank_view[b][e] = view_of_eye[e];
  }
  const int captured_bank = g_banks.write;
  {
    bool hud = true, any = false;
    for (int e = 0; e < 2; ++e) {
      const PreHud& p = g_pre[captured_bank][view_of_eye[e]];
      any = any || p.ok;
      const D3D12_RESOURCE_DESC pd = p.tex != nullptr ? p.tex->GetDesc() : D3D12_RESOURCE_DESC{};
      hud = hud && p.ok && p.src == src[e] && pd.Width == w && pd.Height == h;
    }
    g_bank_hud[b] = hud;
    if (any && !hud) g_hud_mismatch.fetch_add(1, std::memory_order_relaxed);
  }
  const bool published = g_banks.publish();
  for (PreHud& p : g_pre[g_banks.write]) p.ok = false;  // the bank being written next starts empty
  if (!published) {
    g_not_kept.fetch_add(1, std::memory_order_relaxed);
    g_pending_bank = -1;
    return false;
  }
  g_bank_cap[b] = captured_bank;
  g_pending_bank = b;
  g_kept.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void gate(ID3D12CommandQueue* game_queue) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (game_queue == nullptr || g_pending_bank < 0 || g_wfence == nullptr) return;
  const uint64_t v = g_bank_worker_value[g_pending_bank];
  if (v != 0 && g_wfence->GetCompletedValue() < v) game_queue->Wait(g_wfence, v);  // GPU wait only
}

void real_submitted(ID3D12Fence* copy_fence, uint64_t value) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_pending_bank < 0) return;
  g_copy_fence = copy_fence;
  g_bank_copy_value[g_pending_bank] = value;
  g_ready_bank = g_pending_bank;
  g_pending_bank = -1;
}

bool can_generate(ID3D12Device* device, DXGI_FORMAT target_format, uint32_t w, uint32_t h) {
  if (!enabled() || device == nullptr) return false;
  std::lock_guard<std::mutex> lock(g_mutex);
  const int r = g_ready_bank;
  if (r < 0 || g_copy_fence == nullptr || g_bank_cap[r] < 0 || w != g_color_w || h != g_color_h || !ensure_worker(device)) return false;
  if (g_wfence->GetCompletedValue() + 2 < g_wfence_next) return false;  // GPU more than two generated frames behind
  const DXGI_FORMAT rtv_fmt = srgb_typed(target_format) ? target_format : unorm_view(target_format);
  ID3D12PipelineState* pso = nullptr;
  return ensure_pipeline(device, rtv_fmt, &pso);
}

bool generate(ID3D12Device* device, ID3D12Resource* const targets[2], uint32_t w, uint32_t h, float k, float sharpness) {
  if (!enabled() || device == nullptr || targets[0] == nullptr || targets[1] == nullptr) return false;
  std::lock_guard<std::mutex> lock(g_mutex);
  const int r = g_ready_bank;
  if (r < 0 || g_copy_fence == nullptr || g_bank_cap[r] < 0 || w != g_color_w || h != g_color_h || !ensure_worker(device)) {
    g_gen_failures.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const DXGI_FORMAT tf = targets[0]->GetDesc().Format;
  const bool srgb_out = srgb_typed(tf);
  const DXGI_FORMAT rtv_fmt = srgb_out ? tf : unorm_view(tf);
  ID3D12PipelineState* pso = nullptr;
  if (!ensure_pipeline(device, rtv_fmt, &pso)) {
    g_gen_failures.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const int slot = g_wslot;
  if (g_wfence->GetCompletedValue() < g_walloc_value[slot]) {
    g_wfence->SetEventOnCompletion(g_walloc_value[slot], g_wevent);
    if (WaitForSingleObject(g_wevent, 20) != WAIT_OBJECT_0) {  // the GPU is far behind: skip this refresh
      g_gen_failures.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
  }
  if (g_wstamp_pending[slot] && g_wstamps != nullptr && g_wfreq != 0) {
    const uint64_t t0 = g_wstamps[slot * 2], t1 = g_wstamps[slot * 2 + 1];
    if (t1 > t0) {
      g_gpu_ms_sum += static_cast<double>(t1 - t0) * 1000.0 / static_cast<double>(g_wfreq);
      ++g_gpu_samples;
    }
    g_wstamp_pending[slot] = false;
  }
  g_walloc[slot]->Reset();
  g_wlist->Reset(g_walloc[slot], pso);
  const bool timed = g_wqueries != nullptr && g_wstamps != nullptr;
  if (timed) g_wlist->EndQuery(g_wqueries, D3D12_QUERY_TYPE_TIMESTAMP, static_cast<UINT>(slot) * 2);
  g_wlist->SetGraphicsRootSignature(g_root);
  g_wlist->SetDescriptorHeaps(1, &g_srv_heap);
  g_wlist->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  const D3D12_VIEWPORT vp = {0.f, 0.f, static_cast<float>(w), static_cast<float>(h), 0.f, 1.f};
  const D3D12_RECT sc = {0, 0, static_cast<LONG>(w), static_cast<LONG>(h)};
  g_wlist->RSSetViewports(1, &vp);
  g_wlist->RSSetScissorRects(1, &sc);
  const float con = sharpness < 0.f ? 0.f : sharpness > 1.f ? 1.f : sharpness;
  for (int e = 0; e < 2; ++e) {
    const CapView& c = g_cap[g_bank_cap[r]][g_bank_view[r][e]];
    D3D12_CPU_DESCRIPTOR_HANDLE rtv = g_rtv_heap->GetCPUDescriptorHandleForHeapStart();
    rtv.ptr += static_cast<SIZE_T>(e) * g_rtv_stride;
    D3D12_RENDER_TARGET_VIEW_DESC rd = {};
    rd.Format = rtv_fmt;
    rd.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    device->CreateRenderTargetView(targets[e], &rd, rtv);
    D3D12_GPU_DESCRIPTOR_HANDLE srv = g_srv_heap->GetGPUDescriptorHandleForHeapStart();
    srv.ptr += static_cast<UINT64>(r * 2 + e) * g_srv_stride;
    struct {
      UINT out_w, out_h;
      float k, con;
      UINT mv_pitch, mv_x, mv_y, mv_w, mv_h, d_pitch, d_x, d_y, d_w, d_h;
      float scale_x, scale_y;
      UINT srgb_out, use_hud, pad1, pad2;
    } consts = {w, h, k, con, c.mv.pitch, c.mv.rect.x, c.mv.rect.y, c.mv.rect.w, c.mv.rect.h, c.depth.pitch, c.depth.rect.x,
                c.depth.rect.y, c.depth.rect.w, c.depth.rect.h, c.scale[0], c.scale[1], srgb_out ? 1u : 0u, g_bank_hud[r] ? 1u : 0u, 0, 0};
    static_assert(sizeof(consts) == 20 * 4, "root constants");
    g_wlist->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
    g_wlist->SetGraphicsRoot32BitConstants(0, 20, &consts, 0);
    g_wlist->SetGraphicsRootDescriptorTable(1, srv);
    g_wlist->SetGraphicsRootShaderResourceView(2, c.mv.buffer->GetGPUVirtualAddress());
    g_wlist->SetGraphicsRootShaderResourceView(3, c.depth.buffer->GetGPUVirtualAddress());
    D3D12_GPU_DESCRIPTOR_HANDLE pre = srv;  // a valid view even when unused
    if (g_bank_hud[r]) {
      pre = g_srv_heap->GetGPUDescriptorHandleForHeapStart();
      pre.ptr += static_cast<UINT64>(4 + g_bank_cap[r] * 2 + g_bank_view[r][e]) * g_srv_stride;
    }
    g_wlist->SetGraphicsRootDescriptorTable(4, pre);
    g_wlist->DrawInstanced(3, 1, 0, 0);
  }
  // Brightness sample of this generated pair (diagnostic), once a second.
  bool luma_now = false;
  {
    luma_poll();
    const ULONGLONG now = GetTickCount64();
    if (g_luma_fence == 0 && now - g_luma_last >= 1000 && w >= kLumaCrop && h >= kLumaCrop) {
      if (g_luma_rb == nullptr) {
        D3D12_HEAP_PROPERTIES hp = {};
        hp.Type = D3D12_HEAP_TYPE_READBACK;
        D3D12_RESOURCE_DESC rd = {};
        rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
        rd.Width = static_cast<UINT64>(2) * kLumaCrop * kLumaCrop * 4;
        rd.Height = 1;
        rd.DepthOrArraySize = 1;
        rd.MipLevels = 1;
        rd.SampleDesc.Count = 1;
        rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
        if (FAILED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                   IID_PPV_ARGS(&g_luma_rb)))) {
          g_luma_rb = nullptr;
        }
      }
      if (g_luma_rb != nullptr) {
        g_luma_last = now;
        luma_now = true;
        const UINT x = (w - kLumaCrop) / 2, y = (h - kLumaCrop) / 2;
        const D3D12_BOX box = {x, y, 0, x + kLumaCrop, y + kLumaCrop, 1};
        for (int e = 0; e < 2; ++e) {
          barrier(g_wlist, targets[e], D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
          D3D12_TEXTURE_COPY_LOCATION dst = {};
          dst.pResource = g_luma_rb;
          dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
          dst.PlacedFootprint.Offset = static_cast<UINT64>(e) * kLumaCrop * kLumaCrop * 4;
          dst.PlacedFootprint.Footprint = {targets[e]->GetDesc().Format, kLumaCrop, kLumaCrop, 1, kLumaCrop * 4};
          D3D12_TEXTURE_COPY_LOCATION src = {};
          src.pResource = targets[e];
          src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
          g_wlist->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
          barrier(g_wlist, targets[e], D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
        }
      }
    }
  }
  if (timed) {
    g_wlist->EndQuery(g_wqueries, D3D12_QUERY_TYPE_TIMESTAMP, static_cast<UINT>(slot) * 2 + 1);
    g_wlist->ResolveQueryData(g_wqueries, D3D12_QUERY_TYPE_TIMESTAMP, static_cast<UINT>(slot) * 2, 2, g_wreadback,
                              static_cast<UINT64>(slot) * 2 * sizeof(UINT64));
  }
  g_wlist->Close();
  g_wqueue->Wait(g_copy_fence, g_bank_copy_value[r]);  // the real frame's colour copy and captures are complete
  ID3D12CommandList* lists[] = {g_wlist};
  g_wqueue->ExecuteCommandLists(1, lists);
  g_wqueue->Signal(g_wfence, ++g_wfence_next);
  g_walloc_value[slot] = g_wfence_next;
  g_bank_worker_value[r] = g_wfence_next;
  g_wstamp_pending[slot] = timed;
  if (luma_now) g_luma_fence = g_wfence_next;
  g_wslot = (slot + 1) % 3;
  g_generated.fetch_add(1, std::memory_order_relaxed);
  if (g_bank_hud[r]) g_hud_frames.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void sync_to(ID3D12CommandQueue* session_queue) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (session_queue != nullptr && g_wfence != nullptr && g_wfence_next != 0) session_queue->Wait(g_wfence, g_wfence_next);
}

Stats stats() {
  std::lock_guard<std::mutex> lock(g_mutex);
  Stats s = {{g_captures[0].load(), g_captures[1].load()}, g_skipped_layout.load(), g_kept.load(), g_not_kept.load(),
             g_generated.load(), g_gen_failures.load(), g_gpu_samples > 0 ? g_gpu_ms_sum / static_cast<double>(g_gpu_samples) : 0.0,
             {g_luma[0], g_luma[1]}, g_prehud_caps.load(), g_hud_frames.load(), g_hud_mismatch.load()};
  luma_poll();
  g_gpu_ms_sum = 0.0;
  g_gpu_samples = 0;
  return s;
}

}  // namespace frame_gen
