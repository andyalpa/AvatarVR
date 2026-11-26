#include "../runtime_log.hpp"
#include "xess_twin.hpp"

#include "../camera/eye_write.hpp"

#include <d3d12.h>
#include <MinHook.h>

#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>

namespace {

// Minimal declarations of the public XeSS 2.x API (inc/xess/xess.h, xess_d3d12.h; 8-byte packing).
typedef struct _xess_context_handle_t* xess_context_handle_t;
typedef int xess_result_t;  // XESS_RESULT_SUCCESS == 0
#pragma pack(push, 8)
struct xess_2d_t {
  uint32_t x, y;
};
struct xess_coord_t {
  uint32_t x, y;
};
struct xess_d3d12_init_params_t {
  xess_2d_t outputResolution;
  int qualitySetting;
  uint32_t initFlags;
  uint32_t creationNodeMask;
  uint32_t visibleNodeMask;
  ID3D12Heap* pTempBufferHeap;
  uint64_t bufferHeapOffset;
  ID3D12Heap* pTempTextureHeap;
  uint64_t textureHeapOffset;
  ID3D12PipelineLibrary* pPipelineLibrary;
};
struct xess_d3d12_execute_params_t {
  ID3D12Resource* pColorTexture;
  ID3D12Resource* pVelocityTexture;
  ID3D12Resource* pDepthTexture;
  ID3D12Resource* pExposureScaleTexture;
  ID3D12Resource* pResponsivePixelMaskTexture;
  ID3D12Resource* pOutputTexture;
  float jitterOffsetX;
  float jitterOffsetY;
  float exposureScale;
  uint32_t resetHistory;
  uint32_t inputWidth;
  uint32_t inputHeight;
  xess_coord_t inputColorBase;
  xess_coord_t inputMotionVectorBase;
  xess_coord_t inputDepthBase;
  xess_coord_t inputResponsiveMaskBase;
  xess_coord_t reserved0;
  xess_coord_t outputColorBase;
  ID3D12DescriptorHeap* pDescriptorHeap;
  uint32_t descriptorHeapOffset;
};
#pragma pack(pop)
static_assert(sizeof(xess_d3d12_init_params_t) == 64, "XeSS init params layout");
static_assert(offsetof(xess_d3d12_execute_params_t, resetHistory) == 60, "XeSS execute params layout");

using CreateContextFn = xess_result_t (*)(ID3D12Device*, xess_context_handle_t*);
using InitFn = xess_result_t (*)(xess_context_handle_t, const xess_d3d12_init_params_t*);
using ExecuteFn = xess_result_t (*)(xess_context_handle_t, ID3D12GraphicsCommandList*, const xess_d3d12_execute_params_t*);
using BuildPipelinesFn = xess_result_t (*)(xess_context_handle_t, ID3D12PipelineLibrary*, bool, uint32_t);
using DestroyFn = xess_result_t (*)(xess_context_handle_t);
using Set2fFn = xess_result_t (*)(xess_context_handle_t, float, float);
using Set1fFn = xess_result_t (*)(xess_context_handle_t, float);
using SetBoolFn = xess_result_t (*)(xess_context_handle_t, bool);

CreateContextFn g_create = nullptr;
InitFn g_init = nullptr;
ExecuteFn g_execute = nullptr;
BuildPipelinesFn g_build = nullptr;
DestroyFn g_destroy = nullptr;
Set2fFn g_set_jitter_scale = nullptr;
Set2fFn g_set_velocity_scale = nullptr;
Set1fFn g_set_exposure = nullptr;
Set1fFn g_set_max_responsive = nullptr;
SetBoolFn g_force_legacy = nullptr;

struct Twin {
  xess_context_handle_t twin = nullptr;
  bool initialized = false;
  bool needs_reset = true;  // the twin's first frame after (re)activation starts a fresh history
};
std::mutex g_mutex;
std::map<xess_context_handle_t, Twin> g_twins;  // game context -> twin
std::atomic<bool> g_hooked{false};
std::atomic<bool> g_enabled{false};
std::wstring g_control_path;
std::wstring g_log_path;
std::atomic<uint64_t> g_exec_game{0};
std::atomic<uint64_t> g_exec_twin{0};
std::atomic<uint64_t> g_exec_mono{0};
std::atomic<uint64_t> g_exec_disagree{0};  // camera-write eye != present parity at Execute time

void log_line(const char* fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  if (g_log_path.empty()) {
    return;
  }
  runtime_log::append(g_log_path, {buffer, "\r\n"});
}

std::wstring self_dir() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&self_dir), &self)) {
    return {};
  }
  wchar_t path[MAX_PATH] = {};
  const DWORD n = GetModuleFileNameW(self, path, MAX_PATH);
  if (n == 0 || n >= MAX_PATH) {
    return {};
  }
  std::wstring dir(path, n);
  const auto slash = dir.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : dir.substr(0, slash);
}

xess_context_handle_t twin_of(xess_context_handle_t game) {
  std::lock_guard<std::mutex> lock(g_mutex);
  const auto it = g_twins.find(game);
  return it == g_twins.end() ? nullptr : it->second.twin;
}

// ---- hooks -----------------------------------------------------------------------------------------

xess_result_t CreateContext_hook(ID3D12Device* device, xess_context_handle_t* out) {
  const xess_result_t r = g_create(device, out);
  if (r != 0 || out == nullptr || *out == nullptr) {
    log_line("xessD3D12CreateContext -> %d", r);
    return r;
  }
  xess_context_handle_t twin = nullptr;
  const xess_result_t tr = g_create(device, &twin);
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_twins[*out] = Twin{tr == 0 ? twin : nullptr, false, true};
  }
  log_line("xessD3D12CreateContext game=%p twin=%p (twin create %d)", *out, twin, tr);
  return r;
}

xess_result_t Init_hook(xess_context_handle_t ctx, const xess_d3d12_init_params_t* params) {
  const xess_result_t r = g_init(ctx, params);
  if (params == nullptr) {
    return r;
  }
  log_line("xessD3D12Init game=%p -> %d out=%ux%u quality=%d flags=0x%X tempBufferHeap=%p tempTextureHeap=%p pipelineLib=%p",
           ctx, r, params->outputResolution.x, params->outputResolution.y, params->qualitySetting, params->initFlags,
           params->pTempBufferHeap, params->pTempTextureHeap, params->pPipelineLibrary);
  const xess_context_handle_t twin = twin_of(ctx);
  if (r != 0 || twin == nullptr) {
    return r;
  }
  xess_d3d12_init_params_t copy = *params;
  copy.pTempBufferHeap = nullptr;   // XeSS allocates the twin's own storage: no shared history memory
  copy.bufferHeapOffset = 0;
  copy.pTempTextureHeap = nullptr;
  copy.textureHeapOffset = 0;
  const xess_result_t tr = g_init(twin, &copy);
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_twins.find(ctx);
    if (it != g_twins.end()) {
      it->second.initialized = tr == 0;
      it->second.needs_reset = true;
    }
  }
  log_line("  twin %p init -> %d", twin, tr);
  return r;
}

xess_result_t Execute_hook(xess_context_handle_t ctx, ID3D12GraphicsCommandList* list, const xess_d3d12_execute_params_t* params) {
  const int eye = eye_write::parity_eye();  // 0 when not alternating
  if (eye != 0 && eye_write::presented_eye() != eye) {
    g_exec_disagree.fetch_add(1, std::memory_order_relaxed);
  }
  if (!g_enabled.load(std::memory_order_relaxed) || eye == 0 || params == nullptr) {
    g_exec_mono.fetch_add(1, std::memory_order_relaxed);
    return g_execute(ctx, list, params);
  }
  if (eye < 0) {
    g_exec_game.fetch_add(1, std::memory_order_relaxed);
    return g_execute(ctx, list, params);
  }
  xess_context_handle_t twin = nullptr;
  bool reset = false;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_twins.find(ctx);
    if (it != g_twins.end() && it->second.initialized) {
      twin = it->second.twin;
      reset = it->second.needs_reset;
      it->second.needs_reset = false;
    }
  }
  if (twin == nullptr) {
    g_exec_mono.fetch_add(1, std::memory_order_relaxed);
    return g_execute(ctx, list, params);
  }
  g_exec_twin.fetch_add(1, std::memory_order_relaxed);
  if (!reset) {
    return g_execute(twin, list, params);
  }
  xess_d3d12_execute_params_t copy = *params;
  copy.resetHistory = 1;
  return g_execute(twin, list, &copy);
}

xess_result_t BuildPipelines_hook(xess_context_handle_t ctx, ID3D12PipelineLibrary* library, bool blocking, uint32_t flags) {
  const xess_result_t r = g_build(ctx, library, blocking, flags);
  if (const xess_context_handle_t twin = twin_of(ctx)) {
    const xess_result_t tr = g_build(twin, library, blocking, flags);
    log_line("xessD3D12BuildPipelines game=%p -> %d, twin -> %d", ctx, r, tr);
  }
  return r;
}

xess_result_t Destroy_hook(xess_context_handle_t ctx) {
  xess_context_handle_t twin = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_twins.find(ctx);
    if (it != g_twins.end()) {
      twin = it->second.twin;
      g_twins.erase(it);
    }
  }
  if (twin != nullptr) {
    g_destroy(twin);
  }
  log_line("xessDestroyContext game=%p twin=%p", ctx, twin);
  return g_destroy(ctx);
}

xess_result_t SetJitterScale_hook(xess_context_handle_t ctx, float x, float y) {
  if (const xess_context_handle_t twin = twin_of(ctx)) {
    g_set_jitter_scale(twin, x, y);
  }
  return g_set_jitter_scale(ctx, x, y);
}
xess_result_t SetVelocityScale_hook(xess_context_handle_t ctx, float x, float y) {
  if (const xess_context_handle_t twin = twin_of(ctx)) {
    g_set_velocity_scale(twin, x, y);
  }
  return g_set_velocity_scale(ctx, x, y);
}
xess_result_t SetExposure_hook(xess_context_handle_t ctx, float v) {
  if (const xess_context_handle_t twin = twin_of(ctx)) {
    g_set_exposure(twin, v);
  }
  return g_set_exposure(ctx, v);
}
xess_result_t SetMaxResponsive_hook(xess_context_handle_t ctx, float v) {
  if (const xess_context_handle_t twin = twin_of(ctx)) {
    g_set_max_responsive(twin, v);
  }
  return g_set_max_responsive(ctx, v);
}
xess_result_t ForceLegacy_hook(xess_context_handle_t ctx, bool v) {
  if (const xess_context_handle_t twin = twin_of(ctx)) {
    g_force_legacy(twin, v);
  }
  return g_force_legacy(ctx, v);
}

bool hook_export(HMODULE module, const char* name, void* detour, void** original) {
  void* target = reinterpret_cast<void*>(GetProcAddress(module, name));
  if (target == nullptr) {
    log_line("export %s missing", name);
    return false;
  }
  MH_STATUS s = MH_CreateHook(target, detour, original);
  if (s == MH_OK) {
    s = MH_EnableHook(target);
  }
  log_line("hook %s target=%p status=%d", name, target, static_cast<int>(s));
  return s == MH_OK;
}

}  // namespace

namespace xess_twin {

void install() {
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_control_path = dir + L"\\AvatarVR_xess.txt";
  g_log_path = dir + L"\\XESS_TWIN.log";
  log_line("xess twin build %s %s pid=%lu control=AvatarVR_xess.txt (twin|off; missing = off)", __DATE__, __TIME__, GetCurrentProcessId());
}

void hook(HMODULE libxess) {
  if (libxess == nullptr || g_hooked.exchange(true)) {
    return;
  }
  // The core three must all hook, or nothing is routed (Execute without Create/Init would find no twin).
  const bool core = hook_export(libxess, "xessD3D12CreateContext", reinterpret_cast<void*>(&CreateContext_hook), reinterpret_cast<void**>(&g_create)) &&
                    hook_export(libxess, "xessD3D12Init", reinterpret_cast<void*>(&Init_hook), reinterpret_cast<void**>(&g_init)) &&
                    hook_export(libxess, "xessDestroyContext", reinterpret_cast<void*>(&Destroy_hook), reinterpret_cast<void**>(&g_destroy)) &&
                    hook_export(libxess, "xessD3D12Execute", reinterpret_cast<void*>(&Execute_hook), reinterpret_cast<void**>(&g_execute));
  hook_export(libxess, "xessD3D12BuildPipelines", reinterpret_cast<void*>(&BuildPipelines_hook), reinterpret_cast<void**>(&g_build));
  hook_export(libxess, "xessSetJitterScale", reinterpret_cast<void*>(&SetJitterScale_hook), reinterpret_cast<void**>(&g_set_jitter_scale));
  hook_export(libxess, "xessSetVelocityScale", reinterpret_cast<void*>(&SetVelocityScale_hook), reinterpret_cast<void**>(&g_set_velocity_scale));
  hook_export(libxess, "xessSetExposureMultiplier", reinterpret_cast<void*>(&SetExposure_hook), reinterpret_cast<void**>(&g_set_exposure));
  hook_export(libxess, "xessSetMaxResponsiveMaskValue", reinterpret_cast<void*>(&SetMaxResponsive_hook), reinterpret_cast<void**>(&g_set_max_responsive));
  hook_export(libxess, "xessForceLegacyScaleFactors", reinterpret_cast<void*>(&ForceLegacy_hook), reinterpret_cast<void**>(&g_force_legacy));
  log_line("libxess hooked core=%d", core ? 1 : 0);
}

void poll() {
  bool want = false;
  if (!g_control_path.empty()) {
    HANDLE file = CreateFileW(g_control_path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file != INVALID_HANDLE_VALUE) {
      char text[16] = {};
      DWORD read = 0;
      ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
      CloseHandle(file);
      want = std::strncmp(text, "twin", 4) == 0;
    }
  }
  if (want != g_enabled.exchange(want)) {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto& entry : g_twins) {
      entry.second.needs_reset = true;
    }
    log_line("MODE -> %s", want ? "twin" : "off");
  }
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last >= 5000) {
    last = now;
    size_t contexts = 0;
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      contexts = g_twins.size();
    }
    log_line("xess mode=%s contexts=%zu executes: left/game=%llu right/twin=%llu mono=%llu eye_disagree=%llu", want ? "twin" : "off", contexts,
             static_cast<unsigned long long>(g_exec_game.load()), static_cast<unsigned long long>(g_exec_twin.load()),
             static_cast<unsigned long long>(g_exec_mono.load()), static_cast<unsigned long long>(g_exec_disagree.load()));
  }
}

}  // namespace xess_twin
