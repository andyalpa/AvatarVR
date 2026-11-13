// Offline equivalence and timing test for the right-eye motion-vector repair shaders (2026-10-17).
// Loads a captured left/right pair (scripts\captures: mv_vp13_<t>.bin, dp_vp13_<t>.bin, mv_vp17_<t>.bin, dp_vp17_<t>.bin
// and their _consts.bin; header {w, h, format, pitch} then pitch-strided rows, last row unpadded), runs the reference
// and the tiled shader from src/vr/mv_repair_shaders.hpp on the GPU with the plugin's root signature and constants, and
// requires bit-identical output. Stress cases scale the eye offset (more disoccluded pixels and a wider unmatched
// border) to exercise the neighbour search harder than the capture does. Timing: N dispatches of each, timestamps.
// Usage: mv_repair_test.exe <captures dir> <time> [<time> ...]   exit 0 = identical everywhere.
#include "../../src/vr/mv_repair_shaders.hpp"

#include <d3d12.h>
#include <d3dcompiler.h>
#include <dxgi1_4.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

template <class T> void release(T*& p) { if (p) { p->Release(); p = nullptr; } }

struct Image {
  UINT w = 0, h = 0, fmt = 0, pitch = 0;
  std::vector<unsigned char> data;  // pitch * h (last row padded with zeros here)
};

bool load_image(const std::string& path, Image* out) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  unsigned hdr[4] = {};
  bool ok = std::fread(hdr, sizeof(hdr), 1, f) == 1;
  if (ok) {
    out->w = hdr[0]; out->h = hdr[1]; out->fmt = hdr[2]; out->pitch = hdr[3];
    out->data.assign(static_cast<size_t>(out->pitch) * out->h, 0);
    const size_t want = static_cast<size_t>(out->pitch) * (out->h - 1) + static_cast<size_t>(out->w) * 4;
    ok = std::fread(out->data.data(), 1, want, f) == want;
  }
  std::fclose(f);
  return ok;
}

bool load_consts(const std::string& path, float* out) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  const bool ok = std::fread(out, sizeof(float), 114, f) == 114;
  std::fclose(f);
  return ok;
}

struct Gpu {
  ID3D12Device* device = nullptr;
  ID3D12CommandQueue* queue = nullptr;
  ID3D12CommandAllocator* alloc = nullptr;
  ID3D12GraphicsCommandList* list = nullptr;
  ID3D12Fence* fence = nullptr;
  UINT64 fence_value = 0;
  ID3D12RootSignature* root = nullptr;
  // 0 = reference as shipped, 1 = tiled as shipped (strict), 2 = reference compiled strict (exactness oracle for 1)
  ID3D12PipelineState* pso[3] = {};
  bool warp = false;
};

bool run_and_wait(Gpu& g) {
  if (FAILED(g.list->Close())) return false;
  ID3D12CommandList* lists[] = {g.list};
  g.queue->ExecuteCommandLists(1, lists);
  g.queue->Signal(g.fence, ++g.fence_value);
  HANDLE ev = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  g.fence->SetEventOnCompletion(g.fence_value, ev);
  WaitForSingleObject(ev, INFINITE);
  CloseHandle(ev);
  if (g.device->GetDeviceRemovedReason() != S_OK) return false;
  g.alloc->Reset();
  g.list->Reset(g.alloc, nullptr);
  return true;
}

ID3D12Resource* buffer(Gpu& g, UINT64 size, D3D12_HEAP_TYPE type, bool uav, D3D12_RESOURCE_STATES state) {
  D3D12_HEAP_PROPERTIES hp = {}; hp.Type = type;
  D3D12_RESOURCE_DESC rd = {}; rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; rd.Width = size; rd.Height = 1;
  rd.DepthOrArraySize = 1; rd.MipLevels = 1; rd.SampleDesc.Count = 1; rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  rd.Flags = uav ? D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS : D3D12_RESOURCE_FLAG_NONE;
  ID3D12Resource* r = nullptr;
  g.device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, state, nullptr, IID_PPV_ARGS(&r));
  return r;
}

ID3D12Resource* upload_to_default(Gpu& g, const std::vector<unsigned char>& bytes, std::vector<ID3D12Resource*>& keep) {
  ID3D12Resource* up = buffer(g, bytes.size(), D3D12_HEAP_TYPE_UPLOAD, false, D3D12_RESOURCE_STATE_GENERIC_READ);
  ID3D12Resource* dst = buffer(g, bytes.size(), D3D12_HEAP_TYPE_DEFAULT, false, D3D12_RESOURCE_STATE_COMMON);
  if (!up || !dst) return nullptr;
  void* p = nullptr;
  up->Map(0, nullptr, &p);
  std::memcpy(p, bytes.data(), bytes.size());
  up->Unmap(0, nullptr);
  g.list->CopyBufferRegion(dst, 0, up, 0, bytes.size());
  keep.push_back(up);
  return dst;
}

void transition(ID3D12GraphicsCommandList* l, ID3D12Resource* r, D3D12_RESOURCE_STATES a, D3D12_RESOURCE_STATES b) {
  D3D12_RESOURCE_BARRIER x = {};
  x.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  x.Transition.pResource = r; x.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  x.Transition.StateBefore = a; x.Transition.StateAfter = b;
  l->ResourceBarrier(1, &x);
}

bool init(Gpu& g) {
  IDXGIFactory4* factory = nullptr;
  CreateDXGIFactory1(IID_PPV_ARGS(&factory));
  if (FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g.device)))) {
    IDXGIAdapter* warp = nullptr;
    if (factory) factory->EnumWarpAdapter(IID_PPV_ARGS(&warp));
    if (!warp || FAILED(D3D12CreateDevice(warp, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g.device)))) return false;
    g.warp = true;
    release(warp);
  }
  release(factory);
  D3D12_COMMAND_QUEUE_DESC qd = {}; qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  g.device->CreateCommandQueue(&qd, IID_PPV_ARGS(&g.queue));
  g.device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g.alloc));
  g.device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g.alloc, nullptr, IID_PPV_ARGS(&g.list));
  g.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g.fence));
  // Root signature identical to the plugin's (mv_repair.cpp init_pipeline).
  D3D12_ROOT_PARAMETER params[6] = {};
  params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
  params[0].Constants.ShaderRegister = 0;
  params[0].Constants.Num32BitValues = 28;
  for (int i = 0; i < 4; ++i) {
    params[1 + i].ParameterType = D3D12_ROOT_PARAMETER_TYPE_SRV;
    params[1 + i].Descriptor.ShaderRegister = static_cast<UINT>(i);
  }
  params[5].ParameterType = D3D12_ROOT_PARAMETER_TYPE_UAV;
  D3D12_ROOT_SIGNATURE_DESC rs = {}; rs.NumParameters = 6; rs.pParameters = params;
  ID3DBlob* blob = nullptr;
  ID3DBlob* err = nullptr;
  if (FAILED(D3D12SerializeRootSignature(&rs, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &err))) return false;
  g.device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g.root));
  release(blob);
  using namespace mv_repair_shaders;
  const char* src[3] = {kShaderReference, kShaderTiled, kShaderReference};
  const size_t len[3] = {sizeof(kShaderReference) - 1, sizeof(kShaderTiled) - 1, sizeof(kShaderReference) - 1};
  const UINT flags[3] = {D3DCOMPILE_OPTIMIZATION_LEVEL3, D3DCOMPILE_OPTIMIZATION_LEVEL3 | kTiledExtraFlags,
                         D3DCOMPILE_OPTIMIZATION_LEVEL3 | kTiledExtraFlags};
  for (int i = 0; i < 3; ++i) {
    ID3DBlob* cs = nullptr;
    // cs_5_0 with the plugin's flags (mv_repair.cpp init_pipeline) for 0 and 1.
    if (FAILED(D3DCompile(src[i], len[i], i == 1 ? "mv_repair_tiled" : "mv_repair", nullptr, nullptr, "main", "cs_5_0",
                          flags[i], 0, &cs, &err))) {
      std::printf("compile %d failed: %s\n", i, err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
      return false;
    }
    D3D12_COMPUTE_PIPELINE_STATE_DESC pd = {};
    pd.pRootSignature = g.root;
    pd.CS = {cs->GetBufferPointer(), cs->GetBufferSize()};
    if (FAILED(g.device->CreateComputePipelineState(&pd, IID_PPV_ARGS(&g.pso[i])))) return false;
    release(cs);
  }
  return true;
}

float half_to_float(unsigned short h) {
  const unsigned s = (h >> 15) & 1, e = (h >> 10) & 31, m = h & 1023;
  float v = e == 0 ? m / 1024.0f * 6.103515625e-05f : e == 31 ? 65504.0f : (1.0f + m / 1024.0f) * std::ldexp(1.0f, static_cast<int>(e) - 15);
  return s ? -v : v;
}

struct Result {
  unsigned long long exact_mismatches = 0;  // tiled vs strict reference: must be 0
  // Below: tiled vs the shipped (non-strict) reference, information only.
  unsigned long long mismatches = 0, changed = 0, pixels = 0;
  unsigned long long both_left = 0, one_input = 0;  // mismatch kinds: both took a left vector / one kept the input
  double max_delta = 0, sum_delta = 0;  // |difference| of the two outputs, in motion-vector units (R16G16 float)
  double ms[2] = {};
};

bool run_case(Gpu& g, const Image& mvL, const Image& dL, const Image& mvR, const Image& dR,
              const mv_repair_shaders::Constants& k, int timing_reps, Result* res) {
  std::vector<ID3D12Resource*> keep;
  ID3D12Resource* in[4] = {upload_to_default(g, mvL.data, keep), upload_to_default(g, dL.data, keep),
                           upload_to_default(g, dR.data, keep), upload_to_default(g, mvR.data, keep)};
  const UINT64 out_size = mvR.data.size();
  ID3D12Resource* out[3] = {};
  ID3D12Resource* rb[3] = {};
  for (int v = 0; v < 3; ++v) {
    out[v] = buffer(g, out_size, D3D12_HEAP_TYPE_DEFAULT, true, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    rb[v] = buffer(g, out_size, D3D12_HEAP_TYPE_READBACK, false, D3D12_RESOURCE_STATE_COPY_DEST);
  }
  for (auto* r : in) transition(g.list, r, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
  ID3D12QueryHeap* qh = nullptr;
  D3D12_QUERY_HEAP_DESC qd = {}; qd.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP; qd.Count = 4;
  g.device->CreateQueryHeap(&qd, IID_PPV_ARGS(&qh));
  ID3D12Resource* qrb = buffer(g, 4 * sizeof(UINT64), D3D12_HEAP_TYPE_READBACK, false, D3D12_RESOURCE_STATE_COPY_DEST);
  const UINT gx = (k.w + 7) / 8, gy = (k.h + 7) / 8;
  for (int v = 0; v < 3; ++v) {
    g.list->SetComputeRootSignature(g.root);
    g.list->SetPipelineState(g.pso[v]);
    g.list->SetComputeRoot32BitConstants(0, sizeof(k) / 4, &k, 0);
    for (int i = 0; i < 4; ++i) g.list->SetComputeRootShaderResourceView(1 + i, in[i]->GetGPUVirtualAddress());
    g.list->SetComputeRootUnorderedAccessView(5, out[v]->GetGPUVirtualAddress());
    g.list->Dispatch(gx, gy, 1);  // the result that is compared
    D3D12_RESOURCE_BARRIER u = {}; u.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV; u.UAV.pResource = out[v];
    g.list->ResourceBarrier(1, &u);
    if (v < 2) g.list->EndQuery(qh, D3D12_QUERY_TYPE_TIMESTAMP, v * 2);
    for (int r = 0; v < 2 && r < timing_reps; ++r) {  // same output each time; UAV barrier keeps dispatches serial like in game
      g.list->Dispatch(gx, gy, 1);
      g.list->ResourceBarrier(1, &u);
    }
    if (v < 2) g.list->EndQuery(qh, D3D12_QUERY_TYPE_TIMESTAMP, v * 2 + 1);
    transition(g.list, out[v], D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    g.list->CopyBufferRegion(rb[v], 0, out[v], 0, out_size);
  }
  g.list->ResolveQueryData(qh, D3D12_QUERY_TYPE_TIMESTAMP, 0, 4, qrb, 0);
  if (!run_and_wait(g)) { std::printf("GPU error (device removed 0x%08lX)\n", g.device->GetDeviceRemovedReason()); return false; }
  UINT64 freq = 0;
  g.queue->GetTimestampFrequency(&freq);
  UINT64* t = nullptr;
  qrb->Map(0, nullptr, reinterpret_cast<void**>(&t));
  for (int v = 0; v < 2; ++v)
    res->ms[v] = timing_reps > 0 ? static_cast<double>(t[v * 2 + 1] - t[v * 2]) * 1000.0 / freq / timing_reps : 0.0;
  qrb->Unmap(0, nullptr);
  const unsigned char* a = nullptr;
  const unsigned char* b = nullptr;
  const unsigned char* c = nullptr;
  rb[0]->Map(0, nullptr, (void**)&a);
  rb[1]->Map(0, nullptr, (void**)&b);
  rb[2]->Map(0, nullptr, (void**)&c);
  for (UINT y = 0; y < k.h; ++y) {
    for (UINT x = 0; x < k.w; ++x) {
      const size_t o = static_cast<size_t>(y) * k.mv_pitch + x * 4;
      unsigned va, vb, vc, vr;
      std::memcpy(&va, a + o, 4); std::memcpy(&vb, b + o, 4); std::memcpy(&vc, c + o, 4); std::memcpy(&vr, mvR.data.data() + o, 4);
      res->exact_mismatches += vb != vc;
      if (va != vb) {
        ++res->mismatches;
        if (va == vr || vb == vr) ++res->one_input; else ++res->both_left;
        const float dx = half_to_float(va & 0xFFFF) - half_to_float(vb & 0xFFFF);
        const float dy = half_to_float(va >> 16) - half_to_float(vb >> 16);
        const double dd = std::sqrt(static_cast<double>(dx) * dx + static_cast<double>(dy) * dy);
        res->sum_delta += dd;
        if (dd > res->max_delta) res->max_delta = dd;
      }
      res->changed += va != vr;
      ++res->pixels;
    }
  }
  for (auto* r : rb) r->Unmap(0, nullptr);
  for (auto* r : in) r->Release();
  for (auto* r : keep) r->Release();
  for (auto* r : out) r->Release();
  for (auto* r : rb) r->Release();
  qh->Release();
  qrb->Release();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::printf("usage: mv_repair_test <captures dir> <time> [<time> ...]\n");
    return 2;
  }
  const int first = 2;
  Gpu g;
  if (!init(g)) { std::printf("D3D12 init failed\n"); return 2; }
  std::printf("device: %s\n", g.warp ? "WARP (timing not representative)" : "hardware");
  bool all_identical = true;
  const std::string dir = argv[1];
  for (int a = first; a < argc; ++a) {
    const std::string t = argv[a];
    Image mvL, dL, mvR, dR;
    float cl[114] = {}, cr[114] = {};
    if (!load_image(dir + "\\mv_vp13_" + t + ".bin", &mvL) || !load_image(dir + "\\dp_vp13_" + t + ".bin", &dL) ||
        !load_image(dir + "\\mv_vp17_" + t + ".bin", &mvR) || !load_image(dir + "\\dp_vp17_" + t + ".bin", &dR) ||
        !load_consts(dir + "\\mv_vp13_" + t + "_consts.bin", cl) || !load_consts(dir + "\\mv_vp17_" + t + "_consts.bin", cr)) {
      std::printf("%s: capture files missing or short\n", t.c_str());
      all_identical = false;
      continue;
    }
    if (mvL.w != mvR.w || mvL.h != mvR.h || dL.w != mvL.w || dR.w != mvL.w || mvL.pitch != mvR.pitch || dL.pitch != dR.pitch) {
      std::printf("%s: left/right layouts differ\n", t.c_str());
      all_identical = false;
      continue;
    }
    // Case scale 1 = the captured cameras. Larger scales move the left camera further along the eye offset
    // (more disocclusion, wider unmatched border): a harder workload for the neighbour search.
    const float scales[] = {1.0f, 2.0f, 4.0f, 8.0f};
    for (float s : scales) {
      float l2[114];
      std::memcpy(l2, cl, sizeof(l2));
      for (int i = 94; i < 97; ++i) l2[i] = static_cast<float>(cr[i] + (static_cast<double>(cl[i]) - cr[i]) * s);
      mv_repair_shaders::Constants k = {};
      if (!mv_repair_shaders::build_constants(cr, l2, mvR.w, mvR.h, mvR.pitch, dR.pitch, &k)) {
        std::printf("%s x%.0f: unusable projection\n", t.c_str(), s);
        all_identical = false;
        continue;
      }
      Result r;
      if (!run_case(g, mvL, dL, mvR, dR, k, 50, &r)) return 2;
      std::printf("%s eye offset x%.0f (%ux%u): tiled vs strict reference: %llu mismatches | vs shipped reference: %llu of %llu "
                  "differ (both-left %llu, one-kept-input %llu, |dmv| max %.3g mean %.3g) | repaired %.1f%% | shipped reference %.3f ms, "
                  "tiled %.3f ms (%.0f%%)\n",
                  t.c_str(), s, k.w, k.h, r.exact_mismatches, r.mismatches, r.pixels, r.both_left, r.one_input, r.max_delta,
                  r.mismatches ? r.sum_delta / r.mismatches : 0.0, 100.0 * r.changed / r.pixels, r.ms[0], r.ms[1],
                  r.ms[0] > 0 ? 100.0 * r.ms[1] / r.ms[0] : 0.0);
      all_identical &= r.exact_mismatches == 0;
    }
  }
  std::printf("%s\n", all_identical ? "PASS: tiled output bit-identical to the strict reference" : "FAIL");
  return all_identical ? 0 : 1;
}
