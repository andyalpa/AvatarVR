#include "xr_depth.hpp"
#include "xr_depth_math.hpp"

#include <d3dcommon.h>

#include <atomic>
#include <cstring>
#include <mutex>
#include <vector>

// See xr_depth.hpp. Pipeline creation follows mv_repair.cpp (run-time D3DCompile, root descriptors only, no
// descriptor heaps touched on the game's lists).
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

struct View {
  ID3D12Resource* buffer = nullptr;
  UINT64 size = 0;
  UINT pitch = 0;  // bytes per row of the copied depth plane
  xr_depth_math::Rect rect;
  uint64_t captured_at = 0;  // GetTickCount64 of the last copy; 0 = none
};
View g_views[2];
std::vector<ID3D12Resource*> g_retired;  // replaced buffers: lists that still reference them may be in flight

ID3D12RootSignature* g_root = nullptr;
ID3D12PipelineState* g_pso = nullptr;
ID3D12DescriptorHeap* g_dsv_heap = nullptr;
UINT g_dsv_stride = 0;
bool g_pipeline_tried = false;

std::atomic<uint64_t> g_captures[2] = {}, g_skipped_layout{0}, g_skipped_alloc{0}, g_records{0}, g_record_failures{0};

constexpr D3D12_RESOURCE_STATES kExpectedState =
    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;

constexpr char kShader[] = R"(
ByteAddressBuffer g_depth : register(t0);
cbuffer C : register(b0) { uint pitch; uint sx; uint sy; uint sw; uint sh; uint dw; uint dh; uint pad; };
float4 vs(uint id : SV_VertexID) : SV_Position {
  float2 uv = float2((id << 1) & 2, id & 2);
  return float4(uv * float2(2, -2) + float2(-1, 1), 0, 1);
}
void ps(float4 pos : SV_Position, out float depth : SV_Depth) {
  uint ix = min((uint)(pos.x * (float)sw / (float)dw), sw - 1);
  uint iy = min((uint)(pos.y * (float)sh / (float)dh), sh - 1);
  depth = saturate(asfloat(g_depth.Load((sy + iy) * pitch + (sx + ix) * 4)));
}
)";

using D3DCompileFn = HRESULT(WINAPI*)(LPCVOID, SIZE_T, LPCSTR, const D3D_SHADER_MACRO*, ID3DInclude*, LPCSTR, LPCSTR, UINT, UINT,
                                      ID3DBlob**, ID3DBlob**);

bool init_pipeline(ID3D12Device* device) {
  g_pipeline_tried = true;
  HMODULE compiler = LoadLibraryW(L"d3dcompiler_47.dll");
  HMODULE d3d12 = GetModuleHandleW(L"d3d12.dll");
  auto compile = compiler != nullptr ? reinterpret_cast<D3DCompileFn>(GetProcAddress(compiler, "D3DCompile")) : nullptr;
  auto serialize = d3d12 != nullptr
                       ? reinterpret_cast<PFN_D3D12_SERIALIZE_ROOT_SIGNATURE>(GetProcAddress(d3d12, "D3D12SerializeRootSignature"))
                       : nullptr;
  if (compile == nullptr || serialize == nullptr) {
    LOGF("xrdepth: d3dcompiler_47 / D3D12SerializeRootSignature not available");
    return false;
  }
  ID3DBlob* vs = nullptr;
  ID3DBlob* ps = nullptr;
  ID3DBlob* errors = nullptr;
  constexpr UINT kOptimizationLevel3 = 1u << 15;
  HRESULT hr = compile(kShader, sizeof(kShader) - 1, "xr_depth", nullptr, nullptr, "vs", "vs_5_0", kOptimizationLevel3, 0, &vs, &errors);
  if (SUCCEEDED(hr)) {
    if (errors != nullptr) { errors->Release(); errors = nullptr; }
    hr = compile(kShader, sizeof(kShader) - 1, "xr_depth", nullptr, nullptr, "ps", "ps_5_0", kOptimizationLevel3, 0, &ps, &errors);
  }
  if (FAILED(hr)) {
    LOGF("xrdepth: shader compile failed 0x%08X %s", static_cast<unsigned>(hr),
         errors != nullptr ? static_cast<const char*>(errors->GetBufferPointer()) : "");
    if (errors != nullptr) errors->Release();
    if (vs != nullptr) vs->Release();
    return false;
  }
  if (errors != nullptr) { errors->Release(); errors = nullptr; }
  D3D12_ROOT_PARAMETER params[2] = {};
  params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
  params[0].Descriptor.ShaderRegister = 0;
  params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
  params[1].Constants.ShaderRegister = 0;
  params[1].Constants.Num32BitValues = 8;
  params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
  D3D12_ROOT_SIGNATURE_DESC rs = {};
  rs.NumParameters = 2;
  rs.pParameters = params;
  ID3DBlob* blob = nullptr;
  hr = serialize(&rs, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &errors);
  if (FAILED(hr)) {
    LOGF("xrdepth: root signature serialize failed 0x%08X", static_cast<unsigned>(hr));
    if (errors != nullptr) errors->Release();
    vs->Release(); ps->Release();
    return false;
  }
  hr = device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g_root));
  blob->Release();
  if (FAILED(hr)) {
    LOGF("xrdepth: CreateRootSignature failed 0x%08X", static_cast<unsigned>(hr));
    vs->Release(); ps->Release();
    return false;
  }
  D3D12_GRAPHICS_PIPELINE_STATE_DESC pd = {};
  pd.pRootSignature = g_root;
  pd.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
  pd.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
  pd.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
  pd.SampleMask = UINT_MAX;
  pd.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
  pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
  pd.RasterizerState.DepthClipEnable = TRUE;
  pd.DepthStencilState.DepthEnable = TRUE;
  pd.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
  pd.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_ALWAYS;
  pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  pd.NumRenderTargets = 0;
  pd.DSVFormat = DXGI_FORMAT_D32_FLOAT;
  pd.SampleDesc.Count = 1;
  hr = device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&g_pso));
  vs->Release();
  ps->Release();
  if (FAILED(hr)) {
    LOGF("xrdepth: CreateGraphicsPipelineState failed 0x%08X", static_cast<unsigned>(hr));
    return false;
  }
  D3D12_DESCRIPTOR_HEAP_DESC hd = {};
  hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
  hd.NumDescriptors = 2;
  hr = device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_dsv_heap));
  if (FAILED(hr)) {
    LOGF("xrdepth: CreateDescriptorHeap(DSV) failed 0x%08X", static_cast<unsigned>(hr));
    return false;
  }
  g_dsv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);
  LOGF("xrdepth: pipeline ready");
  return true;
}

bool ensure_buffer(ID3D12Device* device, View& v, UINT64 size) {
  if (v.buffer != nullptr && v.size >= size) return true;
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
  ID3D12Resource* next = nullptr;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                             IID_PPV_ARGS(&next)))) {
    return false;
  }
  next->SetName(L"AvatarVR_xr_depth");
  if (v.buffer != nullptr) g_retired.push_back(v.buffer);
  v.buffer = next;
  v.size = capacity;
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

}  // namespace

namespace xr_depth {

void set_log(void (*log)(const char* fmt, ...)) { g_log = log; }

void set_enabled(bool on) {
  if (on != g_enabled.exchange(on)) LOGF("xrdepth: %s", on ? "ON (per-eye depth to the headset layer)" : "off");
}

bool enabled() { return g_enabled.load(std::memory_order_relaxed); }

void capture(int view, ID3D12GraphicsCommandList* list, ID3D12Resource* depth, D3D12_RESOURCE_STATES state, uint32_t ext_x,
             uint32_t ext_y, uint32_t ext_w, uint32_t ext_h) {
  if (!enabled() || list == nullptr || depth == nullptr || view < 0 || view > 1) return;
  const D3D12_RESOURCE_DESC desc = depth->GetDesc();
  if (state != kExpectedState || desc.Format != DXGI_FORMAT_R32G8X24_TYPELESS || desc.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D) {
    g_skipped_layout.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(depth->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp = {};
  UINT64 total = 0;
  device->GetCopyableFootprints(&desc, 0, 1, 0, &fp, nullptr, nullptr, &total);  // subresource 0 = the depth plane
  View& v = g_views[view];
  const UINT64 size = static_cast<UINT64>(fp.Footprint.RowPitch) * desc.Height;
  const bool ok = fp.Footprint.RowPitch != 0 && ensure_buffer(device, v, size);
  device->Release();
  if (!ok) {
    g_skipped_alloc.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  barrier(list, depth, state, D3D12_RESOURCE_STATE_COPY_SOURCE);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = v.buffer;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = fp;
  dst.PlacedFootprint.Offset = 0;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = depth;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  src.SubresourceIndex = 0;
  list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);  // buffer promoted from COMMON to COPY_DEST
  barrier(list, depth, D3D12_RESOURCE_STATE_COPY_SOURCE, state);
  v.pitch = fp.Footprint.RowPitch;
  v.rect = xr_depth_math::source_rect(ext_x, ext_y, ext_w, ext_h, static_cast<uint32_t>(desc.Width), desc.Height);
  v.captured_at = GetTickCount64();
  g_captures[view].fetch_add(1, std::memory_order_relaxed);
}

bool ready(uint64_t max_age_ms) {
  if (!enabled()) return false;
  std::lock_guard<std::mutex> lock(g_mutex);
  const uint64_t now = GetTickCount64();
  for (const View& v : g_views) {
    if (v.buffer == nullptr || v.rect.w == 0 || !xr_depth_math::fresh(now, v.captured_at, max_age_ms)) return false;
  }
  return true;
}

bool record(ID3D12Device* device, ID3D12GraphicsCommandList* list, int view, ID3D12Resource* target, uint32_t w, uint32_t h) {
  if (device == nullptr || list == nullptr || target == nullptr || view < 0 || view > 1 || w == 0 || h == 0) return false;
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_pso == nullptr) {
    if (g_pipeline_tried || !init_pipeline(device)) {
      g_record_failures.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
  }
  const View& v = g_views[view];
  if (v.buffer == nullptr || v.rect.w == 0 || v.rect.h == 0) return false;
  D3D12_CPU_DESCRIPTOR_HANDLE dsv = g_dsv_heap->GetCPUDescriptorHandleForHeapStart();
  dsv.ptr += static_cast<SIZE_T>(view) * g_dsv_stride;
  D3D12_DEPTH_STENCIL_VIEW_DESC dd = {};
  dd.Format = DXGI_FORMAT_D32_FLOAT;
  dd.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
  device->CreateDepthStencilView(target, &dd, dsv);
  const UINT consts[8] = {v.pitch, v.rect.x, v.rect.y, v.rect.w, v.rect.h, w, h, 0};
  list->SetPipelineState(g_pso);
  list->SetGraphicsRootSignature(g_root);
  list->SetGraphicsRootShaderResourceView(0, v.buffer->GetGPUVirtualAddress());  // promoted from COMMON (new submission)
  list->SetGraphicsRoot32BitConstants(1, 8, consts, 0);
  const D3D12_VIEWPORT vp = {0.f, 0.f, static_cast<float>(w), static_cast<float>(h), 0.f, 1.f};
  const D3D12_RECT sc = {0, 0, static_cast<LONG>(w), static_cast<LONG>(h)};
  list->RSSetViewports(1, &vp);
  list->RSSetScissorRects(1, &sc);
  list->OMSetRenderTargets(0, nullptr, FALSE, &dsv);
  list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  list->DrawInstanced(3, 1, 0, 0);
  g_records.fetch_add(1, std::memory_order_relaxed);
  return true;
}

Stats stats() {
  return {{g_captures[0].load(), g_captures[1].load()}, g_skipped_layout.load(), g_skipped_alloc.load(), g_records.load(),
          g_record_failures.load()};
}

}  // namespace xr_depth
