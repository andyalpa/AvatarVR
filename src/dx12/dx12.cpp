#include "../runtime_log.hpp"
#include "dx12.hpp"

#include "../camera/camera.hpp"
#include "../diagnostics/upload_sampler.hpp"
#include "../diagnostics/eye_probe.hpp"
#include "../diagnostics/input_probe.hpp"
#include "../diagnostics/player_probe.hpp"
#include "../script/lua_host.hpp"
#include "../diagnostics/frame_capture.hpp"
#include "../diagnostics/resource_names.hpp"
#include "../diagnostics/hitch_probe.hpp"
#include "../diagnostics/render_activity.hpp"
#include "../camera/eye_write.hpp"
#include "../camera/time_sync.hpp"
#include "../camera/native_stereo.hpp"
#include "../vr/xr_runtime.hpp"
#include "../ui/vr_menu.hpp"
#include "../vr/xess_twin.hpp"
#include "../vr/dlss_twin.hpp"
#include "../vr/mv_repair.hpp"
#include "../vr/virtual_pad.hpp"

#include <MinHook.h>

#include <d3d12.h>
#include <dxgi1_4.h>
#include <cstdlib>
#include <intrin.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "../settings_file.hpp"

namespace {

constexpr int kFactoryCreateSwapChain = 10;
constexpr int kFactory2CreateSwapChainForHwnd = 15;
constexpr int kFactory2CreateSwapChainForCoreWindow = 16;
constexpr int kSwapChainPresent = 8;
constexpr int kSwapChainResizeBuffers = 13;
constexpr int kSwapChain3ResizeBuffers1 = 39;
constexpr int kDeviceCreateCommandQueue = 8;
constexpr int kDeviceCreateCommandList = 12;

using CreateFactoryFn = HRESULT(WINAPI*)(REFIID, void**);
using CreateFactory2Fn = HRESULT(WINAPI*)(UINT, REFIID, void**);
using LoadLibraryWFn = HMODULE(WINAPI*)(LPCWSTR);
using LoadLibraryExWFn = HMODULE(WINAPI*)(LPCWSTR, HANDLE, DWORD);
using D3D12CreateDeviceFn = HRESULT(WINAPI*)(IUnknown*, D3D_FEATURE_LEVEL, REFIID, void**);
using CreateSwapChainFn = HRESULT(STDMETHODCALLTYPE*)(IDXGIFactory*, IUnknown*, DXGI_SWAP_CHAIN_DESC*, IDXGISwapChain**);
using CreateSwapChainForHwndFn = HRESULT(STDMETHODCALLTYPE*)(IDXGIFactory2*, IUnknown*, HWND, const DXGI_SWAP_CHAIN_DESC1*, const DXGI_SWAP_CHAIN_FULLSCREEN_DESC*, IDXGIOutput*, IDXGISwapChain1**);
using CreateSwapChainForCoreWindowFn = HRESULT(STDMETHODCALLTYPE*)(IDXGIFactory2*, IUnknown*, IUnknown*, const DXGI_SWAP_CHAIN_DESC1*, IDXGIOutput*, IDXGISwapChain1**);
using PresentFn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain*, UINT, UINT);
using ResizeBuffersFn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain*, UINT, UINT, UINT, DXGI_FORMAT, UINT);
using ResizeBuffers1Fn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain3*, UINT, UINT, UINT, DXGI_FORMAT, UINT, const UINT*, IUnknown* const*);
using CreateCommandQueueFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, const D3D12_COMMAND_QUEUE_DESC*, REFIID, void**);
using CreateCommandListFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, D3D12_COMMAND_LIST_TYPE, ID3D12CommandAllocator*, ID3D12PipelineState*, REFIID, void**);

CreateFactoryFn g_create_factory = nullptr;
CreateFactoryFn g_create_factory1 = nullptr;
CreateFactory2Fn g_create_factory2 = nullptr;
LoadLibraryWFn g_load_library_w = nullptr;
LoadLibraryExWFn g_load_library_ex_w = nullptr;
D3D12CreateDeviceFn g_d3d12_create_device = nullptr;
CreateSwapChainFn g_create_swapchain = nullptr;
CreateSwapChainForHwndFn g_create_swapchain_hwnd = nullptr;
CreateSwapChainForCoreWindowFn g_create_swapchain_core = nullptr;
PresentFn g_present = nullptr;
ResizeBuffersFn g_resize_buffers = nullptr;
ResizeBuffers1Fn g_resize_buffers1 = nullptr;
CreateCommandQueueFn g_create_command_queue = nullptr;
CreateCommandListFn g_create_command_list = nullptr;

std::mutex g_log_mutex;
std::mutex g_hook_mutex;
std::wstring g_log_path;
void* g_hooked_targets[24] = {};
int g_hooked_count = 0;

std::atomic<std::uint64_t> g_present_count{0};
std::atomic<std::uint64_t> g_last_reported_present{0};
std::atomic<ULONGLONG> g_last_present_log_tick{0};
std::atomic<std::uint64_t> g_list_count[8] = {};
std::atomic<int> g_logged_list[8] = {};

IDXGISwapChain* g_stats_swapchain = nullptr;
ID3D12CommandQueue* g_compute_queue = nullptr;  // first compute queue the game created (GPU profiler)

std::wstring module_path(HMODULE module) {
  std::wstring buf(MAX_PATH, L'\0');
  for (;;) {
    const DWORD wrote = GetModuleFileNameW(module, buf.data(), static_cast<DWORD>(buf.size()));
    if (wrote == 0) {
      return {};
    }
    if (wrote < buf.size() - 1) {
      buf.resize(wrote);
      return buf;
    }
    buf.resize(buf.size() * 2);
  }
}

std::wstring parent_dir(const std::wstring& path) {
  const auto slash = path.find_last_of(L"\\/");
  if (slash == std::wstring::npos) {
    return {};
  }
  return path.substr(0, slash);
}

void log_line(const std::string& text) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty()) {
    return;
  }
  runtime_log::append(g_log_path, {text, "\r\n"});
}

void logf(const char* fmt, ...) {
  char buffer[1400];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  log_line(buffer);
}

const char* format_name(DXGI_FORMAT format) {
  switch (format) {
    case DXGI_FORMAT_R16G16B16A16_FLOAT:
      return "R16G16B16A16_FLOAT";
    case DXGI_FORMAT_R10G10B10A2_UNORM:
      return "R10G10B10A2_UNORM";
    case DXGI_FORMAT_R8G8B8A8_UNORM:
      return "R8G8B8A8_UNORM";
    case DXGI_FORMAT_B8G8R8A8_UNORM:
      return "B8G8R8A8_UNORM";
    default:
      return "other";
  }
}

const char* swap_effect_name(DXGI_SWAP_EFFECT effect) {
  switch (effect) {
    case DXGI_SWAP_EFFECT_DISCARD:
      return "DISCARD";
    case DXGI_SWAP_EFFECT_SEQUENTIAL:
      return "SEQUENTIAL";
    case DXGI_SWAP_EFFECT_FLIP_SEQUENTIAL:
      return "FLIP_SEQUENTIAL";
    case DXGI_SWAP_EFFECT_FLIP_DISCARD:
      return "FLIP_DISCARD";
    default:
      return "other";
  }
}

const char* list_type_name(D3D12_COMMAND_LIST_TYPE type) {
  switch (type) {
    case D3D12_COMMAND_LIST_TYPE_DIRECT:
      return "DIRECT";
    case D3D12_COMMAND_LIST_TYPE_BUNDLE:
      return "BUNDLE";
    case D3D12_COMMAND_LIST_TYPE_COMPUTE:
      return "COMPUTE";
    case D3D12_COMMAND_LIST_TYPE_COPY:
      return "COPY";
    default:
      return "other";
  }
}

const char* queue_type_name(D3D12_COMMAND_LIST_TYPE type) {
  return list_type_name(type);
}

void guid_text(REFIID iid, char* out, size_t out_size) {
  snprintf(out, out_size, "%08lX-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X",
           iid.Data1, iid.Data2, iid.Data3,
           iid.Data4[0], iid.Data4[1], iid.Data4[2], iid.Data4[3],
           iid.Data4[4], iid.Data4[5], iid.Data4[6], iid.Data4[7]);
}

void* vtable_slot(void* object, int index) {
  if (object == nullptr || index < 0) {
    return nullptr;
  }
  auto** table = *reinterpret_cast<void***>(object);
  if (table == nullptr) {
    return nullptr;
  }
  return table[index];
}

bool remember_target(void* target) {
  for (int i = 0; i < g_hooked_count; ++i) {
    if (g_hooked_targets[i] == target) {
      return false;
    }
  }
  if (g_hooked_count >= static_cast<int>(sizeof(g_hooked_targets) / sizeof(g_hooked_targets[0]))) {
    return false;
  }
  g_hooked_targets[g_hooked_count++] = target;
  return true;
}

bool hook_target(void* target, void* detour, void** original, const char* name) {
  if (target == nullptr || detour == nullptr || original == nullptr) {
    logf("hook skip %s null", name);
    return false;
  }
  bool already = false;
  MH_STATUS created = MH_OK;
  MH_STATUS enabled = MH_OK;
  {
    std::lock_guard<std::mutex> lock(g_hook_mutex);
    for (int i = 0; i < g_hooked_count; ++i) {
      if (g_hooked_targets[i] == target) {
        already = true;
        break;
      }
    }
    if (!already) {
      created = MH_CreateHook(target, detour, original);
      if (created == MH_OK) {
        enabled = MH_EnableHook(target);
        if (enabled == MH_OK) {
          remember_target(target);
        }
      }
    }
  }
  if (already) {
    return true;
  }
  if (created != MH_OK) {
    logf("hook create failed %s status=%d target=%p", name, static_cast<int>(created), target);
    return false;
  }
  if (enabled != MH_OK) {
    logf("hook enable failed %s status=%d target=%p", name, static_cast<int>(enabled), target);
    return false;
  }
  logf("hook enabled %s target=%p thread=%lu", name, target, GetCurrentThreadId());
  return true;
}

bool hook_slot(void* object, int index, void* detour, void** original, const char* name) {
  void* target = vtable_slot(object, index);
  logf("vtable %s index=%d target=%p thread=%lu", name, index, target, GetCurrentThreadId());
  return hook_target(target, detour, original, name);
}

void retain_swapchain(IDXGISwapChain* swapchain) {
  std::lock_guard<std::mutex> lock(g_hook_mutex);
  if (g_stats_swapchain == swapchain) {
    return;
  }
  if (g_stats_swapchain != nullptr) {
    g_stats_swapchain->Release();
  }
  g_stats_swapchain = swapchain;
  if (g_stats_swapchain != nullptr) {
    g_stats_swapchain->AddRef();
  }
}

void describe_swapchain(IDXGISwapChain* swapchain, const char* source) {
  if (swapchain == nullptr) {
    return;
  }
  logf("swapchain source=%s ptr=%p thread=%lu", source, swapchain, GetCurrentThreadId());
  DXGI_SWAP_CHAIN_DESC desc = {};
  const HRESULT desc_hr = swapchain->GetDesc(&desc);
  if (FAILED(desc_hr)) {
    logf("swapchain GetDesc hr=0x%08lX", static_cast<unsigned long>(desc_hr));
  } else {
    logf("swapchain buffers=%u %ux%u format=%u:%s windowed=%d effect=%u:%s flags=0x%X refresh=%u/%u",
         desc.BufferCount,
         desc.BufferDesc.Width,
         desc.BufferDesc.Height,
         static_cast<unsigned>(desc.BufferDesc.Format),
         format_name(desc.BufferDesc.Format),
         desc.Windowed ? 1 : 0,
         static_cast<unsigned>(desc.SwapEffect),
         swap_effect_name(desc.SwapEffect),
         desc.Flags,
         desc.BufferDesc.RefreshRate.Numerator,
         desc.BufferDesc.RefreshRate.Denominator);
    camera::note_backbuffer(desc.BufferDesc.Width, desc.BufferDesc.Height);
  }

  IDXGISwapChain1* swapchain1 = nullptr;
  if (SUCCEEDED(swapchain->QueryInterface(IID_PPV_ARGS(&swapchain1)))) {
    DXGI_SWAP_CHAIN_DESC1 desc1 = {};
    if (SUCCEEDED(swapchain1->GetDesc1(&desc1))) {
      logf("swapchain1 buffers=%u %ux%u format=%u:%s stereo=%d scaling=%u effect=%u:%s alpha=%u flags=0x%X",
           desc1.BufferCount, desc1.Width, desc1.Height,
           static_cast<unsigned>(desc1.Format), format_name(desc1.Format),
           desc1.Stereo ? 1 : 0, static_cast<unsigned>(desc1.Scaling),
           static_cast<unsigned>(desc1.SwapEffect), swap_effect_name(desc1.SwapEffect),
           static_cast<unsigned>(desc1.AlphaMode), desc1.Flags);
    }
    swapchain1->Release();
  }

  const UINT buffer_count = SUCCEEDED(desc_hr) ? desc.BufferCount : 0;
  for (UINT index = 0; index < buffer_count && index < 8; ++index) {
    ID3D12Resource* resource = nullptr;
    const HRESULT buffer_hr = swapchain->GetBuffer(index, IID_PPV_ARGS(&resource));
    if (FAILED(buffer_hr)) {
      logf("backbuffer %u GetBuffer ID3D12Resource hr=0x%08lX", index, static_cast<unsigned long>(buffer_hr));
      continue;
    }
    const D3D12_RESOURCE_DESC resource_desc = resource->GetDesc();
    logf("backbuffer %u %llux%u format=%u:%s dimension=%u",
         index,
         static_cast<unsigned long long>(resource_desc.Width),
         resource_desc.Height,
         static_cast<unsigned>(resource_desc.Format),
         format_name(resource_desc.Format),
         static_cast<unsigned>(resource_desc.Dimension));
    resource->Release();
  }

  ID3D12Device* device = nullptr;
  const HRESULT device_hr = swapchain->GetDevice(IID_PPV_ARGS(&device));
  if (FAILED(device_hr)) {
    logf("swapchain GetDevice ID3D12Device hr=0x%08lX", static_cast<unsigned long>(device_hr));
  } else {
    logf("swapchain device=%p", device);
    device->Release();
  }

  DXGI_FRAME_STATISTICS stats = {};
  const HRESULT stats_hr = swapchain->GetFrameStatistics(&stats);
  if (FAILED(stats_hr)) {
    logf("GetFrameStatistics hr=0x%08lX", static_cast<unsigned long>(stats_hr));
  } else {
    logf("GetFrameStatistics present=%u refresh=%u sync_refresh=%u",
         stats.PresentCount, stats.PresentRefreshCount, stats.SyncRefreshCount);
  }
  retain_swapchain(swapchain);
}

// Research hooks (per-draw command-list observers in camera.cpp, full-rate CBV observer in upload_sampler.cpp) are
// off unless scripts\AvatarVR_research.txt says "on", or AER per-eye CBV writes are active (AvatarVR_eye.txt not
// "off"). They sit on per-draw/per-CBV engine calls and cost frame rate.
bool control_file_starts_with(const wchar_t* name, const char* word, bool missing) {
  const std::wstring path = parent_dir(g_log_path) + L"\\" + name;
  HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return missing;
  }
  char text[32] = {};
  DWORD read = 0;
  ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
  CloseHandle(file);
  const char* t = text;
  while (*t == ' ' || *t == '\t' || *t == '\r' || *t == '\n') {
    ++t;
  }
  return std::strncmp(t, word, std::strlen(word)) == 0;
}

// Profiler window length in seconds after gameplay stereo starts: "on 300" / "census 300" in AvatarVR_gpuprof.txt
// (performance pass 2026-10-22: the 60 s default ended in the cave before the user reached the slow outdoor scene).
ULONGLONG prof_window_ms() {
  static const ULONGLONG ms = [] {
    const std::wstring path = parent_dir(g_log_path) + L"\\AvatarVR_gpuprof.txt";
    HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) return 60000ull;
    char text[32] = {};
    DWORD read = 0;
    ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
    CloseHandle(file);
    const char* t = text;
    while (*t != 0 && !(*t == ' ' || *t == '\t')) ++t;  // skip the mode word
    const long s = std::strtol(t, nullptr, 10);
    return s >= 10 && s <= 1800 ? static_cast<ULONGLONG>(s) * 1000ull : 60000ull;
  }();
  return ms;
}

bool research_hooks() {
  static const bool on = control_file_starts_with(L"AvatarVR_research.txt", "on", false);
  return on;
}

bool eye_writes_active() {
  static const bool off = control_file_starts_with(L"AvatarVR_eye.txt", "off", true);
  return !off;
}

void note_list(D3D12_COMMAND_LIST_TYPE type, HRESULT hr, void* list) {
  const int bucket = static_cast<int>(type);
  const int index = (bucket >= 0 && bucket < 8) ? bucket : 7;
  g_list_count[index].fetch_add(1);
  if (SUCCEEDED(hr) && g_logged_list[index].exchange(1) == 0) {
    logf("command_list first type=%d:%s hr=0x%08lX list=%p thread=%lu",
         bucket, list_type_name(type), static_cast<unsigned long>(hr), list, GetCurrentThreadId());
    if (type == D3D12_COMMAND_LIST_TYPE_DIRECT) {
      camera::watch_list(list, research_hooks());  // UI probe always; research observers only when asked
    }
  }
}

HRESULT STDMETHODCALLTYPE CreateCommandQueue_hook(ID3D12Device* device, const D3D12_COMMAND_QUEUE_DESC* desc, REFIID riid, void** queue) {
  char iid[80];
  guid_text(riid, iid, sizeof(iid));
  if (desc != nullptr) {
    logf("CreateCommandQueue device=%p type=%u:%s priority=%d flags=0x%X node=%u iid=%s thread=%lu",
         device, static_cast<unsigned>(desc->Type), queue_type_name(desc->Type),
         desc->Priority, desc->Flags, desc->NodeMask, iid, GetCurrentThreadId());
  } else {
    logf("CreateCommandQueue device=%p desc=null iid=%s thread=%lu", device, iid, GetCurrentThreadId());
  }
  const HRESULT hr = g_create_command_queue(device, desc, riid, queue);
  logf("CreateCommandQueue hr=0x%08lX queue=%p", static_cast<unsigned long>(hr), (SUCCEEDED(hr) && queue != nullptr) ? *queue : nullptr);
  if (SUCCEEDED(hr) && queue != nullptr && *queue != nullptr && desc != nullptr && desc->Type == D3D12_COMMAND_LIST_TYPE_COMPUTE &&
      g_compute_queue == nullptr && riid == __uuidof(ID3D12CommandQueue)) {
    g_compute_queue = static_cast<ID3D12CommandQueue*>(*queue);
    g_compute_queue->AddRef();  // kept for the GPU profiler (game lifetime)
  }
  return hr;
}

bool activity_wanted() {
  static const bool on = control_file_starts_with(L"AvatarVR_gpuprof.txt", "census", false) ||
                         control_file_starts_with(L"AvatarVR_gpuprof.txt", "stages", false);
  return on;
}
HRESULT STDMETHODCALLTYPE CreateCommandList_hook(ID3D12Device* device, UINT node_mask, D3D12_COMMAND_LIST_TYPE type, ID3D12CommandAllocator* allocator, ID3D12PipelineState* state, REFIID riid, void** list) {
  const HRESULT hr = g_create_command_list(device, node_mask, type, allocator, state, riid, list);
  note_list(type, hr, (SUCCEEDED(hr) && list != nullptr) ? *list : nullptr);
  if (SUCCEEDED(hr) && list != nullptr && *list != nullptr && activity_wanted()) {
    ID3D12GraphicsCommandList* graphics = nullptr;
    if (SUCCEEDED(static_cast<IUnknown*>(*list)->QueryInterface(IID_PPV_ARGS(&graphics))) && graphics != nullptr) {
      render_activity::watch(graphics); graphics->Release();
    }
  }
  return hr;
}

void watch_device(IUnknown* object) {
  if (object == nullptr) {
    return;
  }
  ID3D12Device* device = nullptr;
  const HRESULT hr = object->QueryInterface(IID_PPV_ARGS(&device));
  if (FAILED(hr)) {
    logf("device QueryInterface ID3D12Device hr=0x%08lX thread=%lu", static_cast<unsigned long>(hr), GetCurrentThreadId());
    return;
  }
  logf("device ptr=%p thread=%lu", device, GetCurrentThreadId());
  D3D12_FEATURE_DATA_D3D12_OPTIONS6 vrs = {};
  if (SUCCEEDED(device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS6, &vrs, sizeof(vrs)))) {
    logf("VRS capability: tier %u, tile %u, additional rates %u (capability only; no shading changes)",
         static_cast<unsigned>(vrs.VariableShadingRateTier), vrs.ShadingRateImageTileSize,
         static_cast<unsigned>(vrs.AdditionalShadingRatesSupported));
  }
  hook_slot(device, kDeviceCreateCommandQueue, reinterpret_cast<void*>(&CreateCommandQueue_hook), reinterpret_cast<void**>(&g_create_command_queue), "ID3D12Device::CreateCommandQueue");
  hook_slot(device, kDeviceCreateCommandList, reinterpret_cast<void*>(&CreateCommandList_hook), reinterpret_cast<void**>(&g_create_command_list), "ID3D12Device::CreateCommandList");
  upload::set_full_rate(research_hooks() || eye_writes_active());
  logf("research hooks %s, CBV observer %s rate (AvatarVR_research.txt \"on\" enables the full research set)",
       research_hooks() ? "ON" : "off", research_hooks() || eye_writes_active() ? "full" : "light 1/32");
  upload::watch_device(device);
  if (activity_wanted()) {
    // Census only: compute shader identity (dumped to scripts\shaders) and per-dispatch timestamp heap.
    render_activity::install_device(device, (parent_dir(g_log_path) + L"\\shaders").c_str());
    // "stages": per-engine-stage GPU time instead of per-dispatch (2026-10-23, read-only).
    if (control_file_starts_with(L"AvatarVR_gpuprof.txt", "stages", false)) {
      const bool ok = render_activity::set_stage_mode((parent_dir(g_log_path) + L"\\AvatarVR_stagemap.txt").c_str());
      logf("gpustages: %s (%u stage functions from AvatarVR_stagemap.txt)", ok ? "on" : "map missing; per-dispatch census",
           render_activity::stage_count());
    }
    logf("gpucensus shader identity: hook coverage=0x%X (bit5 CreateComputePipelineState, bit6 stamp heap)",
         render_activity::extra_hook_coverage());
  }
  camera::watch_device(device);  // PSO blend states for the scan highlight rule
  hitch_probe::watch_device(device);
  eye_probe::install();
  time_sync::install();
  native_stereo::install();
  device->Release();
}

void log_present_sample(IDXGISwapChain* swapchain, UINT sync, UINT flags, HRESULT hr) {
  const auto count = g_present_count.load();
  const auto previous = g_last_reported_present.exchange(count);
  IDXGISwapChain* stats_target = swapchain;
  if (stats_target != nullptr) {
    stats_target->AddRef();
  }
  DXGI_FRAME_STATISTICS stats = {};
  HRESULT stats_hr = E_FAIL;
  if (stats_target != nullptr) {
    stats_hr = stats_target->GetFrameStatistics(&stats);
    stats_target->Release();
  }
  logf("present n=%llu delta=%llu sync=%u flags=0x%X hr=0x%08lX thread=%lu lists=%llu/%llu/%llu/%llu stats_hr=0x%08lX present_count=%u refresh=%u sync_refresh=%u",
       static_cast<unsigned long long>(count),
       static_cast<unsigned long long>(count - previous),
       sync, flags, static_cast<unsigned long>(hr), GetCurrentThreadId(),
       static_cast<unsigned long long>(g_list_count[0].load()),
       static_cast<unsigned long long>(g_list_count[1].load()),
       static_cast<unsigned long long>(g_list_count[2].load()),
       static_cast<unsigned long long>(g_list_count[3].load()),
       static_cast<unsigned long>(stats_hr),
       SUCCEEDED(stats_hr) ? stats.PresentCount : 0,
       SUCCEEDED(stats_hr) ? stats.PresentRefreshCount : 0,
       SUCCEEDED(stats_hr) ? stats.SyncRefreshCount : 0);
  frame_capture::poll();
  time_sync::poll();
  xr_runtime::poll();
  resource_names::poll();
  xess_twin::poll();
  dlss_twin::poll();
  native_stereo::poll();
  camera::sample();
  upload::sample();
}

// ---- eye buffers (eye mode, 2026-10-09) ------------------------------------------------------------------------
// Run 18:20-18:24: the main view's final image always goes to the display, whatever the offscreen byte says, so in a
// widescreen window the eye-sized left image was clipped. The native stereo size override separates render resolution
// from swapchain size: with "eyesize W H both", the swapchain buffers are
// made at least W x H while the window keeps its own size. The engine renders into the top-left of the buffers (eye
// size in gameplay, the window's client size in menus), and IDXGISwapChain2::SetSourceSize presents only the
// client-size top-left region, so the monitor shows it 1:1. xr_runtime copies the left eye from the top-left W x H and
// draws the monitor mirror into the client region. Nothing holds the backbuffer (that broke ResizeBuffers).
std::atomic<UINT> g_eyebuf_w{0}, g_eyebuf_h{0};
std::atomic<bool> g_ui_spoof_wanted{false};  // see GetClientRect_hook
std::atomic<UINT> g_client_w{0}, g_client_h{0};
std::atomic<bool> g_buffers_grown{false};
ULONGLONG g_eyebuf_read = 0;
UINT g_source_w = 0, g_source_h = 0;

void read_eye_size() {
  const ULONGLONG now = GetTickCount64();
  if (g_eyebuf_read != 0 && now - g_eyebuf_read < 1000) {
    return;
  }
  g_eyebuf_read = now;
  const std::wstring path = parent_dir(g_log_path) + L"\\AvatarVR_native.txt";
  UINT w = 0, h = 0;
  char text[1024] = {};
  if (settings_file::read(path, text, sizeof(text))) {
    g_ui_spoof_wanted.store(std::strstr(text, "nouispoof") == nullptr);
    const char* es = std::strstr(text, "eyesize");
    if (es != nullptr && std::strstr(es, "both") != nullptr) {
      char* e1 = nullptr;
      const unsigned long a = std::strtoul(es + 7, &e1, 10);
      const unsigned long b = e1 != nullptr ? std::strtoul(e1, nullptr, 10) : 0;
      if (a >= 256 && a <= 8192 && b >= 256 && b <= 8192) {
        w = static_cast<UINT>(a & ~7ul);
        h = static_cast<UINT>(b & ~7ul);
      }
    }
  }
  g_eyebuf_w.store(w);
  g_eyebuf_h.store(h);
}

// Buffer size for a swapchain of `window` asked at w x h (0 = the client area): at least the eye size in eye mode.
// ---- the game's window size while the eyes run (eye mode, 2026-10-09) ---------------------------------------------
// [AUTHOR] runs 20:1x-20:2x: with the window widescreen the game's UI changed size and position with Render height,
// in the HUD and the 3D main menu. The game lays its UI out from the window's client size (GetClientRect, read about
// 11k times a session per the 2026-10-05 spoof run, which also showed the swapchain and the render size do not
// follow it), so while the eyes run, calls from afop.exe for the game window get the eye size, as when the window
// itself had the eye shape. Calls from this DLL and from DXGI keep the real size (grown buffers, SetSourceSize,
// mirror). "nouispoof" in AvatarVR_native.txt turns it off.
using GetClientRectFn = BOOL(WINAPI*)(HWND, LPRECT);
GetClientRectFn g_get_client_rect = nullptr;
std::atomic<HWND> g_game_window{nullptr};
std::uintptr_t g_game_lo = 0, g_game_hi = 0;
std::atomic<std::uint64_t> g_client_spoofs{0};

BOOL WINAPI GetClientRect_hook(HWND hwnd, LPRECT rect) {
  const BOOL ok = g_get_client_rect(hwnd, rect);
  if (!ok || rect == nullptr || hwnd == nullptr || hwnd != g_game_window.load(std::memory_order_relaxed) ||
      !g_ui_spoof_wanted.load(std::memory_order_relaxed)) {
    return ok;
  }
  const auto ra = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
  if (ra < g_game_lo || ra >= g_game_hi) {
    return ok;  // this DLL, DXGI, the OS: the real window
  }
  std::uint32_t w = 0, h = 0;
  if (native_stereo::eye_size(&w, &h)) {
    rect->left = 0;
    rect->top = 0;
    rect->right = static_cast<LONG>(w);
    rect->bottom = static_cast<LONG>(h);
    g_client_spoofs.fetch_add(1, std::memory_order_relaxed);
  }
  return ok;
}

void hook_client_rect() {
  HMODULE game = GetModuleHandleW(nullptr);
  if (game != nullptr) {
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(game);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(reinterpret_cast<const unsigned char*>(game) + dos->e_lfanew);
    g_game_lo = reinterpret_cast<std::uintptr_t>(game);
    g_game_hi = g_game_lo + nt->OptionalHeader.SizeOfImage;
  }
  HMODULE user32 = GetModuleHandleW(L"user32.dll");
  void* target = user32 != nullptr ? reinterpret_cast<void*>(GetProcAddress(user32, "GetClientRect")) : nullptr;
  hook_target(target, reinterpret_cast<void*>(&GetClientRect_hook), reinterpret_cast<void**>(&g_get_client_rect), "user32!GetClientRect");
}

}  // namespace
namespace dx12 {
void* game_window() { return g_game_window.load(std::memory_order_relaxed); }
}  // namespace dx12
namespace {

bool grow_buffers(HWND window, UINT* w, UINT* h) {
  read_eye_size();
  const UINT ew = g_eyebuf_w.load(), eh = g_eyebuf_h.load();
  if (ew == 0 || eh == 0) {
    return false;
  }
  if ((*w == 0 || *h == 0) && window != nullptr) {
    RECT rc = {};
    GetClientRect(window, &rc);
    if (*w == 0) {
      *w = static_cast<UINT>(rc.right - rc.left);
    }
    if (*h == 0) {
      *h = static_cast<UINT>(rc.bottom - rc.top);
    }
  }
  UINT nw = *w > ew ? *w : ew, nh = *h > eh ? *h : eh;
  if (window != nullptr) {  // never smaller than the real window (this DLL's GetClientRect is not redirected)
    RECT rc = {};
    GetClientRect(window, &rc);
    nw = nw > static_cast<UINT>(rc.right - rc.left) ? nw : static_cast<UINT>(rc.right - rc.left);
    nh = nh > static_cast<UINT>(rc.bottom - rc.top) ? nh : static_cast<UINT>(rc.bottom - rc.top);
  }
  if (nw == *w && nh == *h) {
    return false;
  }
  *w = nw;
  *h = nh;
  return true;
}

// Present thread: present only the client-size top-left region of grown buffers.
void update_source_size(IDXGISwapChain* swapchain) {
  DXGI_SWAP_CHAIN_DESC d = {};
  if (FAILED(swapchain->GetDesc(&d))) {
    return;
  }
  g_game_window.store(d.OutputWindow, std::memory_order_relaxed);
  read_eye_size();  // also refreshes the UI spoof switch (once a second)
  {
    static ULONGLONG last = 0;
    const ULONGLONG now = GetTickCount64();
    if (now - last >= 5000) {
      last = now;
      logf("ui window size: game GetClientRect calls given the eye size %llu (spoof %s)",
           static_cast<unsigned long long>(g_client_spoofs.exchange(0)), g_ui_spoof_wanted.load() ? "on" : "off");
    }
  }
  RECT rc = {};
  GetClientRect(d.OutputWindow, &rc);
  UINT cw = static_cast<UINT>(rc.right - rc.left), ch = static_cast<UINT>(rc.bottom - rc.top);
  const bool grown = cw > 0 && ch > 0 && (d.BufferDesc.Width > cw || d.BufferDesc.Height > ch);
  g_buffers_grown.store(grown);
  cw = cw > d.BufferDesc.Width ? d.BufferDesc.Width : cw;
  ch = ch > d.BufferDesc.Height ? d.BufferDesc.Height : ch;
  g_client_w.store(cw);
  g_client_h.store(ch);
  const UINT sw = grown ? cw : d.BufferDesc.Width, sh = grown ? ch : d.BufferDesc.Height;
  if (sw == g_source_w && sh == g_source_h) {
    return;
  }
  IDXGISwapChain2* sc2 = nullptr;
  if (SUCCEEDED(swapchain->QueryInterface(IID_PPV_ARGS(&sc2))) && sc2 != nullptr) {
    const HRESULT hr = sc2->SetSourceSize(sw, sh);
    logf("SetSourceSize %ux%u (buffers %ux%u, client %ux%u) hr=0x%08lX", sw, sh, d.BufferDesc.Width, d.BufferDesc.Height, cw, ch,
         static_cast<unsigned long>(hr));
    sc2->Release();
    if (SUCCEEDED(hr)) {
      g_source_w = sw;
      g_source_h = sh;
    }
  }
}

// Present-thread timing, reported once a second as a "perf" line: where a frame's time goes. "game" is the time
// from the previous Present's return to this Present (the engine's own frame work on this thread), each stage is
// one mod hook, "present" is the original Present (blocking there = GPU or display bound).
enum PerfStage { kPerfUpload, kPerfEyeProbe, kPerfCapture, kPerfXr, kPerfMenu, kPerfNative, kPerfEye, kPerfTime, kPerfPresent,
                 kPerfGame, kPerfStages };
const char* const kPerfNames[kPerfStages] = {"upload", "eyeprobe", "capture", "xr", "menu", "native", "eye", "time", "present",
                                             "game"};
double g_perf_sum[kPerfStages] = {};
double g_perf_max[kPerfStages] = {};
double g_perf_interval_max = 0.0;
unsigned g_perf_frames = 0;
LARGE_INTEGER g_perf_freq = {}, g_perf_last_return = {}, g_perf_last_entry = {}, g_perf_window = {};

// GPU lag: a fence is signalled on the game's queue right before each Present (after all of that frame's direct-queue
// work); a watcher thread records when the GPU reaches it. Lag = completion time - signal time. GPU-bound frames show
// a lag of about a frame or more (the GPU is still working through earlier frames); CPU/engine-bound frames show a few
// ms at most. Observer only: one Signal per Present on the game's own queue.
ID3D12Fence* g_lag_fence = nullptr;
ID3D12CommandQueue* g_lag_queue = nullptr;
HANDLE g_lag_event = nullptr;
std::atomic<UINT64> g_lag_signalled{0};
constexpr int kLagRing = 256;
LONGLONG g_lag_signal_qpc[kLagRing] = {};
std::atomic<std::uint64_t> g_lag_sum_us{0}, g_lag_max_us{0}, g_lag_count{0};
bool g_lag_failed = false;

DWORD WINAPI lag_watcher(void*) {
  LARGE_INTEGER freq;
  QueryPerformanceFrequency(&freq);
  UINT64 next = 1;
  for (;;) {
    if (next > g_lag_signalled.load(std::memory_order_acquire)) {
      Sleep(1);
      continue;
    }
    if (g_lag_signalled.load() - next >= kLagRing / 2) {
      next = g_lag_signalled.load();  // fell behind: skip ahead instead of reading overwritten slots
    }
    if (g_lag_fence->GetCompletedValue() < next) {
      g_lag_fence->SetEventOnCompletion(next, g_lag_event);
      if (WaitForSingleObject(g_lag_event, 2000) != WAIT_OBJECT_0) {
        continue;
      }
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    const LONGLONG signalled = g_lag_signal_qpc[next % kLagRing];
    if (signalled != 0 && now.QuadPart >= signalled) {
      const auto us = static_cast<std::uint64_t>((now.QuadPart - signalled) * 1000000 / freq.QuadPart);
      g_lag_sum_us.fetch_add(us, std::memory_order_relaxed);
      g_lag_count.fetch_add(1, std::memory_order_relaxed);
      std::uint64_t prev = g_lag_max_us.load(std::memory_order_relaxed);
      while (us > prev && !g_lag_max_us.compare_exchange_weak(prev, us)) {
      }
    }
    ++next;
  }
}

void prof_arm(ID3D12CommandQueue* game_queue);  // GPU submission profiler, below
void wait_skip_arm(ID3D12CommandQueue* game_queue);  // queue wait skip experiment, below

void lag_signal() {
  if (g_lag_failed) {
    return;
  }
  if (g_lag_fence == nullptr) {
    g_lag_queue = xr_runtime::acquire_game_queue();
    if (g_lag_queue == nullptr) {
      return;  // not recorded yet
    }
    ID3D12Device* device = nullptr;
    if (FAILED(g_lag_queue->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr ||
        FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_lag_fence)))) {
      g_lag_failed = true;
      logf("gpu lag: fence creation failed; gpu lag not measured");
      if (device != nullptr) {
        device->Release();
      }
      return;
    }
    device->Release();
    g_lag_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    HANDLE thread = g_lag_event != nullptr ? CreateThread(nullptr, 0, &lag_watcher, nullptr, 0, nullptr) : nullptr;
    if (thread == nullptr) {
      g_lag_failed = true;
      logf("gpu lag: watcher thread failed; gpu lag not measured");
      return;
    }
    CloseHandle(thread);
    logf("gpu lag: fence on game queue %p", static_cast<void*>(g_lag_queue));
    wait_skip_arm(g_lag_queue);
    prof_arm(g_lag_queue);
  }
  const UINT64 value = g_lag_signalled.load() + 1;
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  g_lag_signal_qpc[value % kLagRing] = now.QuadPart;
  if (SUCCEEDED(g_lag_queue->Signal(g_lag_fence, value))) {
    g_lag_signalled.store(value, std::memory_order_release);
  }
}

// ---- GPU submission profiler (2026-10-08; scripts\AvatarVR_gpuprof.txt "on", read at start-up; off by default) ----
// Question: which GPU work makes the stereo frame cost about twice the mono frame (GPU-bound, RENDER_PASS_CENSUS.md)?
// Each ExecuteCommandLists on the game's direct and compute queues is bracketed by two tiny command lists of our own
// that write a GPU timestamp, so every submission gets a GPU duration. Results are read back a few frames later (no
// stall; a frame whose slot is still in flight is not profiled). Each submission is tagged with the Gfx_View kinds
// whose Render is running at that moment (native_stereo::rendering_kinds: 1 main, 2 second, 4 other views), the
// submitting thread and its position in the frame. Reported every 5 s as "gpuprof" lines: per tag and per position
// (avg ms per frame). Adds commands only (two timestamps per submission, one resolve per frame); game lists unchanged.
constexpr int kQueueExecuteCommandLists = 10;
using ExecuteCommandListsFn = void(STDMETHODCALLTYPE*)(ID3D12CommandQueue*, UINT, ID3D12CommandList* const*);
ExecuteCommandListsFn g_execute_command_lists = nullptr;

constexpr int kProfFrames = 4;
constexpr int kProfMaxSubs = 128;  // submissions profiled per frame and queue
constexpr UINT kProfMaxLists = 1024;  // command lists per ExecuteCommandLists that can be bracketed
constexpr int kProfTags = 6;       // none, main, second, other, mixed, (unused)
const char* const kProfTagNames[kProfTags] = {"none", "main", "second", "other", "mixed", "?"};

struct ProfSub {
  render_activity::Sample activity;
  DWORD tid;
  unsigned char tag;
  unsigned short lists;
  short dlss_vp;  // Streamline viewport of a DLSS evaluate in this submission, -1 none
  char ui_eye;    // UI pass of eye 1 / 2 in this submission, 0 none
  float cpu_ms;  // submit time after the previous Present
  LONGLONG qpc;  // submit time (QPC)
  unsigned short waits;  // ID3D12CommandQueue::Wait calls on this queue since its previous submission
  unsigned char nwait;              // timeline: up to 3 of those waits (fence id, value)
  unsigned char wait_f[3];
  UINT64 wait_v[3];
  unsigned char sig_f;              // timeline: last Signal on this queue after this submission (0 = none)
  UINT64 sig_v;
  unsigned short stamp_first, stamp_count;  // census dispatch stamps in ProfFrame::stamps
};

constexpr unsigned kFrameStamps = 4096;  // census dispatch stamps per frame and queue
struct ShaderAgg {
  double ms = 0.0;
  unsigned n = 0;
  double groups = 0.0;
  std::uint64_t positions[2] = {};  // bit per submission position 0..127
  bool indirect = false;
  double ms_block[3] = {};  // main-view block, second-view block, after both (split at the UI-pass submissions)
};

struct ProfFrame {
  ID3D12CommandAllocator* alloc = nullptr;
  ID3D12GraphicsCommandList* lists[kProfMaxSubs * 2 + 1] = {};
  int used_lists = 0;
  int subs = 0;
  bool open = false;
  bool pending = false;
  UINT64 fence_value = 0;
  UINT64 cal_gpu = 0;  // GetClockCalibration at frame end: GPU timestamp <-> QPC
  UINT64 cal_cpu = 0;
  ProfSub meta[kProfMaxSubs] = {};
  render_activity::DispatchStamp stamps[kFrameStamps];
  unsigned stamp_n = 0;
  LONGLONG open_qpc = 0;   // timeline: QPC when the slot opened (at a Present)
  unsigned frame_no = 0;   // timeline: Present number that opened the slot (same for both queues)
};

// Two-queue timeline (2026-10-23, read-only): one frame of submissions per queue, on the CPU clock.
struct TimelineEntry {
  double begin_ms = 0.0, end_ms = 0.0;  // after the Present that opened the frame
  int sub = 0;
  std::string desc;
};
struct Timeline {
  unsigned frame_no = 0;
  std::vector<TimelineEntry> entries;
};

struct ProfQueue {
  const char* name = "";
  ID3D12CommandQueue* queue = nullptr;
  D3D12_COMMAND_LIST_TYPE type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  ID3D12Device* device = nullptr;
  ID3D12QueryHeap* heap = nullptr;
  ID3D12Resource* readback = nullptr;
  ID3D12Fence* fence = nullptr;
  UINT64 fence_value = 0;
  double ticks_per_ms = 0.0;
  bool ready = false;
  bool failed = false;
  int cur = 0;
  ProfFrame frames[kProfFrames];
  std::mutex mutex;
  // 5 s accumulators (Present thread only)
  double tag_ms[kProfTags] = {};
  unsigned tag_subs[kProfTags] = {};
  double pos_ms[kProfMaxSubs] = {};
  double pos_cpu[kProfMaxSubs] = {};
  unsigned pos_n[kProfMaxSubs] = {};
  unsigned pos_tag[kProfMaxSubs][kProfTags] = {};
  unsigned pos_lists[kProfMaxSubs] = {};
  unsigned pos_dlss[kProfMaxSubs] = {};       // frames with a DLSS evaluate at this position
  short pos_dlss_vp[kProfMaxSubs] = {};       // its latest viewport id
  unsigned pos_ui[kProfMaxSubs][3] = {};      // frames with a UI pass of eye 1 / 2
  DWORD pos_tid[kProfMaxSubs] = {};
  std::uint64_t pos_activity[kProfMaxSubs][render_activity::kKinds] = {};
  render_activity::Sample pos_activity_last[kProfMaxSubs] = {};
  double span_ms = 0.0, busy_ms = 0.0;
  // Gaps between consecutive submissions inside a frame (GPU idle on this queue), by cause:
  //   starved = the GPU began the next submission within 0.25 ms of the CPU submitting it (it waited for the game)
  //   wait    = not starved, and the game issued a queue Wait before that submission (cross-queue fence)
  //   other   = neither (unexplained)
  double gap_ms[3] = {};
  unsigned gap_n[3] = {};
  double latency_ms = 0.0;  // GPU begin - CPU submit, summed over submissions (queue backlog)
  unsigned latency_n = 0;
  double pos_gap[kProfMaxSubs][3] = {};  // gap before this position, by cause
  double pos_latency[kProfMaxSubs] = {};
  unsigned pos_waits[kProfMaxSubs] = {};
  std::atomic<unsigned> waits_since_submit{0};
  unsigned char pw_f[3] = {};  // timeline: waits since the previous submission (guarded by mutex)
  UINT64 pw_v[3] = {};
  unsigned pw_n = 0;
  Timeline tl[4];              // timeline ring (Present thread)
  int tl_next = 0;
  unsigned frames_read = 0, frames_skipped = 0, subs_dropped = 0;
  std::unordered_map<std::uint64_t, ShaderAgg> shader_agg;  // census: per compute shader, Present thread only
  double stamped_ms = 0.0;
  unsigned stamped_n = 0, stamp_misses = 0;
};

// Failure counters with the first HRESULT of each step (reported with every gpuprof block).
enum ProfFail { kFailCreate, kFailReset, kFailClose, kFailAllocReset, kFailMap, kFailResolve, kFailCount };
const char* const kFailNames[kFailCount] = {"create", "reset", "close", "allocreset", "map", "resolve"};
std::atomic<unsigned> g_prof_fail[kFailCount];
std::atomic<long> g_prof_fail_hr[kFailCount];
std::atomic<std::uint64_t> g_prof_stamped{0}, g_prof_reads{0}, g_prof_empty_reads{0};

void prof_fail(int step, HRESULT hr) {
  if (g_prof_fail[step].fetch_add(1, std::memory_order_relaxed) == 0) {
    g_prof_fail_hr[step].store(hr, std::memory_order_relaxed);
  }
}

// Marks set by the recording threads (dlss_twin, camera), consumed when the list is submitted.
struct ListMark {
  void* list;
  char kind;
  int value;
};
constexpr int kMaxMarks = 64;
ListMark g_marks[kMaxMarks] = {};
std::mutex g_mark_mutex;

void take_marks(ID3D12CommandList* const* lists, UINT count, short* dlss_vp, char* ui_eye) {
  *dlss_vp = -1;
  *ui_eye = 0;
  std::lock_guard<std::mutex> lock(g_mark_mutex);
  for (int m = 0; m < kMaxMarks; ++m) {
    if (g_marks[m].list == nullptr) {
      continue;
    }
    for (UINT i = 0; i < count; ++i) {
      if (g_marks[m].list == lists[i]) {
        if (g_marks[m].kind == 'D') {
          *dlss_vp = static_cast<short>(g_marks[m].value);
        } else if (g_marks[m].kind == 'U') {
          *ui_eye = static_cast<char>(g_marks[m].value);
        }
        g_marks[m].list = nullptr;
        break;
      }
    }
  }
}

ProfQueue g_prof[2];  // 0 = direct (the game's swapchain queue), 1 = compute
std::atomic<bool> g_prof_armed{false};
LARGE_INTEGER g_prof_present_qpc = {};
ULONGLONG g_prof_last_report = 0;

bool gpu_prof_wanted() {
  static const bool on = control_file_starts_with(L"AvatarVR_gpuprof.txt", "on", false) || activity_wanted();
  return on;
}

bool prof_init(ProfQueue& p) {
  if (FAILED(p.queue->GetDevice(IID_PPV_ARGS(&p.device))) || p.device == nullptr) {
    return false;
  }
  UINT64 freq = 0;
  if (FAILED(p.queue->GetTimestampFrequency(&freq)) || freq == 0) {
    return false;
  }
  p.ticks_per_ms = static_cast<double>(freq) / 1000.0;
  D3D12_QUERY_HEAP_DESC qd = {};
  qd.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
  qd.Count = kProfFrames * kProfMaxSubs * 2;
  if (FAILED(p.device->CreateQueryHeap(&qd, IID_PPV_ARGS(&p.heap)))) {
    return false;
  }
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC rd = {};
  rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  rd.Width = static_cast<UINT64>(qd.Count) * sizeof(UINT64);
  rd.Height = 1;
  rd.DepthOrArraySize = 1;
  rd.MipLevels = 1;
  rd.SampleDesc.Count = 1;
  rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  if (FAILED(p.device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                               IID_PPV_ARGS(&p.readback)))) {
    return false;
  }
  if (FAILED(p.device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&p.fence)))) {
    return false;
  }
  for (ProfFrame& f : p.frames) {
    if (FAILED(p.device->CreateCommandAllocator(p.type, IID_PPV_ARGS(&f.alloc)))) {
      return false;
    }
  }
  p.frames[0].open = true;
  return true;
}

// A closed command list holding one EndQuery (or, for the frame's resolve, the ResolveQueryData). Caller holds p.mutex.
ID3D12GraphicsCommandList* prof_list(ProfQueue& p, ProfFrame& f) {
  if (f.used_lists >= static_cast<int>(sizeof(f.lists) / sizeof(f.lists[0]))) {
    return nullptr;
  }
  ID3D12GraphicsCommandList*& list = f.lists[f.used_lists];
  if (list == nullptr) {
    const HRESULT hr = p.device->CreateCommandList(0, p.type, f.alloc, nullptr, IID_PPV_ARGS(&list));
    if (FAILED(hr) || list == nullptr) {
      prof_fail(kFailCreate, hr);
      list = nullptr;
      return nullptr;
    }
  } else {
    const HRESULT hr = list->Reset(f.alloc, nullptr);
    if (FAILED(hr)) {
      prof_fail(kFailReset, hr);
      return nullptr;
    }
  }
  ++f.used_lists;
  return list;
}

int prof_index(ID3D12CommandQueue* queue) {
  if (!g_prof_armed.load(std::memory_order_acquire)) {
    return -1;
  }
  for (int i = 0; i < 2; ++i) {
    if (g_prof[i].queue == queue && g_prof[i].ready) {
      return i;
    }
  }
  return -1;
}

std::atomic<std::uint64_t> g_ecl_calls{0}, g_ecl_other_queue{0}, g_ecl_lists{0};
std::atomic<unsigned> g_ecl_max_lists{0};

// Timeline fence ids (1..250) and the profiled queue that last signalled each ('d' direct, 'c' compute, '?' other).
std::mutex g_fence_mutex;
std::unordered_map<ID3D12Fence*, unsigned char> g_fence_ids;
char g_fence_queue[256] = {};
std::atomic<unsigned> g_prof_present_no{0};
unsigned char fence_id(ID3D12Fence* fence) {
  std::lock_guard<std::mutex> lock(g_fence_mutex);
  const auto it = g_fence_ids.find(fence);
  if (it != g_fence_ids.end()) return it->second;
  if (g_fence_ids.size() >= 250) return 255;
  const auto id = static_cast<unsigned char>(g_fence_ids.size() + 1);
  g_fence_ids.emplace(fence, id);
  return id;
}

constexpr int kQueueSignal = 14;
using QueueSignalFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12CommandQueue*, ID3D12Fence*, UINT64);
QueueSignalFn g_queue_signal = nullptr;

HRESULT STDMETHODCALLTYPE QueueSignal_hook(ID3D12CommandQueue* queue, ID3D12Fence* fence, UINT64 value) {
  const int qi = prof_index(queue);
  if (qi < 0 && g_prof_armed.load(std::memory_order_relaxed) && fence != nullptr) {
    // Other queues: label their fences by queue type (x copy, o other direct, k other compute) so a wait on them reads.
    const D3D12_COMMAND_QUEUE_DESC qd = queue->GetDesc();
    const unsigned char id = fence_id(fence);
    g_fence_queue[id] = qd.Type == D3D12_COMMAND_LIST_TYPE_COPY ? 'x' : qd.Type == D3D12_COMMAND_LIST_TYPE_COMPUTE ? 'k' : 'o';
  }
  if (qi >= 0 && fence != g_prof[qi].fence) {  // the profiler's own frame fence is signalled under p.mutex: skip it
    ProfQueue& p = g_prof[qi];
    const unsigned char id = fence_id(fence);
    g_fence_queue[id] = qi == 0 ? 'd' : 'c';
    std::lock_guard<std::mutex> lock(p.mutex);
    ProfFrame& f = p.frames[p.cur];
    if (f.open && f.subs > 0) {
      f.meta[f.subs - 1].sig_f = id;
      f.meta[f.subs - 1].sig_v = value;
    }
  }
  return g_queue_signal(queue, fence, value);
}

constexpr int kQueueWait = 15;
using QueueWaitFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12CommandQueue*, ID3D12Fence*, UINT64);
QueueWaitFn g_queue_wait = nullptr;

// ---- queue wait skip (2026-11-09 experiment; scripts\AvatarVR_queue.txt "waitskip", read at start-up; off by default) ----
// LIVE gpuprof timeline 2026-11-09 (16 frames, 619 direct submissions): the direct queue idles ~4-5 ms per frame in
// gaps of nearly fixed size (0.21-0.24 and 0.32-0.34 ms), next to cross-queue Waits / Signals, while the compute queue is
// busy during only 4% of that idle time. INFERENCE: OS scheduling latency per sync point (Windows hardware-accelerated
// GPU scheduling is off on this PC), not GPU dependencies. This drops a queue Wait whose fence has ALREADY reached the
// value when the game issues it (the GPU would pass it at once; the work it waits for is complete), so the OS no longer
// has to process it. Waits that are not yet satisfied go through unchanged. A fence seen going backwards (non-monotonic
// use, where a skip could be wrong) turns the experiment off for the session. Counts every 5 s: "queue waits:".
std::atomic<bool> g_wait_skip{false};
std::atomic<bool> g_wait_skip_broken{false};
std::atomic<std::uint64_t> g_waits_seen{0}, g_waits_skipped{0};
std::mutex g_wait_mutex;
std::unordered_map<ID3D12Fence*, UINT64> g_wait_last;  // highest completed value seen per fence (guarded by g_wait_mutex)

// True = skip this Wait (already satisfied on a fence that has behaved monotonically).
bool wait_already_satisfied(ID3D12Fence* fence, UINT64 value) {
  const UINT64 done = fence->GetCompletedValue();
  if (done == UINT64_MAX) {
    return false;  // device removed: let the runtime handle it
  }
  bool monotonic = true;
  UINT64 previous = 0;
  {
    std::lock_guard<std::mutex> lock(g_wait_mutex);
    if (g_wait_last.size() > 4096) {
      g_wait_last.clear();  // fences come and go with loads; a reused address only risks a false "backwards" (safe)
    }
    UINT64& last = g_wait_last[fence];
    previous = last;
    if (done < last) {
      monotonic = false;
    } else {
      last = done;
    }
  }
  if (!monotonic) {
    g_wait_skip.store(false, std::memory_order_relaxed);
    if (!g_wait_skip_broken.exchange(true)) {
      logf("queue waits: fence %p went backwards (%llu after %llu); wait skip OFF for this session", static_cast<void*>(fence),
           static_cast<unsigned long long>(done), static_cast<unsigned long long>(previous));
    }
    return false;
  }
  return value <= done;
}

// Wait caller check (2026-11-09, read-only): which module issues queue Waits, on which queue type. Logged once per
// (module, queue type); the per-call cost is one compare against a few cached return addresses.
std::atomic<void*> g_wait_callers[32] = {};
std::mutex g_wait_caller_mutex;
void note_wait_caller(void* ret, ID3D12CommandQueue* queue) {
  for (const auto& c : g_wait_callers) {
    void* v = c.load(std::memory_order_relaxed);
    if (v == ret) return;
    if (v == nullptr) break;
  }
  std::lock_guard<std::mutex> lock(g_wait_caller_mutex);
  int free_slot = -1;
  for (int i = 0; i < 32; ++i) {
    void* v = g_wait_callers[i].load(std::memory_order_relaxed);
    if (v == ret) return;
    if (v == nullptr) {
      free_slot = i;
      break;
    }
  }
  if (free_slot < 0) return;
  g_wait_callers[free_slot].store(ret, std::memory_order_relaxed);
  HMODULE module = nullptr;
  char name[MAX_PATH] = "?";
  if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                         static_cast<LPCSTR>(ret), &module) && module != nullptr) {
    GetModuleFileNameA(module, name, MAX_PATH);
  }
  const char* base = std::strrchr(name, '\\');
  const D3D12_COMMAND_QUEUE_DESC qd = queue->GetDesc();
  const char* type = qd.Type == D3D12_COMMAND_LIST_TYPE_DIRECT ? "direct" : qd.Type == D3D12_COMMAND_LIST_TYPE_COMPUTE ? "compute"
                     : qd.Type == D3D12_COMMAND_LIST_TYPE_COPY ? "copy" : "other";
  const int qi = prof_index(queue);
  logf("queue Wait caller: %s+0x%llX on %s queue %p%s", base != nullptr ? base + 1 : name,
       static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(ret) - reinterpret_cast<std::uintptr_t>(module)), type,
       static_cast<void*>(queue), qi == 0 ? " (the game's swapchain queue)" : qi == 1 ? " (the game's compute queue)" : "");
}

HRESULT STDMETHODCALLTYPE QueueWait_hook(ID3D12CommandQueue* queue, ID3D12Fence* fence, UINT64 value) {
  note_wait_caller(_ReturnAddress(), queue);
  if (fence != nullptr && g_wait_skip.load(std::memory_order_relaxed)) {
    g_waits_seen.fetch_add(1, std::memory_order_relaxed);
    if (wait_already_satisfied(fence, value)) {
      g_waits_skipped.fetch_add(1, std::memory_order_relaxed);
      return S_OK;  // nothing to wait for; the profiler's wait bookkeeping is left out too (no wait reached the queue)
    }
  }
  const int qi = prof_index(queue);
  if (qi >= 0) {
    g_prof[qi].waits_since_submit.fetch_add(1, std::memory_order_relaxed);
    ProfQueue& p = g_prof[qi];
    const unsigned char id = fence_id(fence);
    std::lock_guard<std::mutex> lock(p.mutex);
    if (p.pw_n < 3) {
      p.pw_f[p.pw_n] = id;
      p.pw_v[p.pw_n++] = value;
    }
  }
  return g_queue_wait(queue, fence, value);
}

void STDMETHODCALLTYPE ExecuteCommandLists_hook(ID3D12CommandQueue* queue, UINT count, ID3D12CommandList* const* lists) {
  if (!g_prof_armed.load(std::memory_order_acquire)) {
    g_execute_command_lists(queue, count, lists);
    mv_repair::submitted(queue, count, lists);
    return;
  }
  g_ecl_calls.fetch_add(1, std::memory_order_relaxed);
  g_ecl_lists.fetch_add(count, std::memory_order_relaxed);
  unsigned seen = g_ecl_max_lists.load(std::memory_order_relaxed);
  while (count > seen && !g_ecl_max_lists.compare_exchange_weak(seen, count)) {
  }
  const int qi = prof_index(queue);
  if (qi < 0) {
    g_ecl_other_queue.fetch_add(1, std::memory_order_relaxed);
  }
  if (qi < 0 || count == 0 || count > kProfMaxLists) {
    g_execute_command_lists(queue, count, lists);
    mv_repair::submitted(queue, count, lists);
    return;
  }
  ProfQueue& p = g_prof[qi];
  ID3D12CommandList* wrapped[kProfMaxLists + 2];
  bool profiled = false;
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    ProfFrame& f = p.frames[p.cur];
    if (f.open && f.subs < kProfMaxSubs) {
      const UINT q = static_cast<UINT>((p.cur * kProfMaxSubs + f.subs) * 2);
      // One allocator records one list at a time (a second Create/Reset on it fails with E_INVALIDARG while the first
      // list is still open), so each list is recorded and closed before the next one is taken.
      ID3D12GraphicsCommandList* begin = prof_list(p, f);
      HRESULT hb = E_FAIL, he = E_FAIL;
      if (begin != nullptr) {
        begin->EndQuery(p.heap, D3D12_QUERY_TYPE_TIMESTAMP, q);
        hb = begin->Close();
      }
      ID3D12GraphicsCommandList* end = SUCCEEDED(hb) ? prof_list(p, f) : nullptr;
      unsigned stamps = 0;
      if (end != nullptr) {
        end->EndQuery(p.heap, D3D12_QUERY_TYPE_TIMESTAMP, q + 1);
        stamps = render_activity::take_stamps(lists, count, f.stamps + f.stamp_n, kFrameStamps - f.stamp_n);
        render_activity::resolve_stamps(end, f.stamps + f.stamp_n, stamps);
        he = end->Close();
      }
      if (begin != nullptr && end != nullptr) {
        if (FAILED(hb) || FAILED(he)) {
          prof_fail(kFailClose, FAILED(hb) ? hb : he);
        } else {
          g_prof_stamped.fetch_add(1, std::memory_order_relaxed);
          LARGE_INTEGER now;
          QueryPerformanceCounter(&now);
          const unsigned kinds = native_stereo::rendering_kinds();
          ProfSub& m = f.meta[f.subs++];
          m.tid = GetCurrentThreadId();
          m.tag = static_cast<unsigned char>(kinds == 0 ? 0 : kinds == 1 ? 1 : kinds == 2 ? 2 : kinds == 4 ? 3 : 4);
          m.lists = static_cast<unsigned short>(count);
          take_marks(lists, count, &m.dlss_vp, &m.ui_eye);
          m.activity = render_activity::take(lists,count);
          m.qpc = now.QuadPart;
          m.stamp_first = static_cast<unsigned short>(f.stamp_n);
          m.stamp_count = static_cast<unsigned short>(stamps);
          f.stamp_n += stamps;
          m.waits = static_cast<unsigned short>(p.waits_since_submit.exchange(0, std::memory_order_relaxed));
          m.nwait = static_cast<unsigned char>(p.pw_n);
          for (unsigned w = 0; w < p.pw_n; ++w) {
            m.wait_f[w] = p.pw_f[w];
            m.wait_v[w] = p.pw_v[w];
          }
          p.pw_n = 0;
          m.sig_f = 0;
          m.sig_v = 0;
          m.cpu_ms = g_perf_freq.QuadPart != 0 && g_prof_present_qpc.QuadPart != 0
                         ? static_cast<float>((now.QuadPart - g_prof_present_qpc.QuadPart) * 1000.0 / g_perf_freq.QuadPart)
                         : 0.f;
          wrapped[0] = begin;
          for (UINT i = 0; i < count; ++i) {
            wrapped[i + 1] = lists[i];
          }
          wrapped[count + 1] = end;
          profiled = true;
        }
      }
    } else if (f.open) {
      ++p.subs_dropped;
    }
  }
  if (profiled) {
    g_execute_command_lists(queue, count + 2, wrapped);
  } else {
    g_execute_command_lists(queue, count, lists);
  }
  mv_repair::submitted(queue, count, lists);
}

void prof_read(ProfQueue& p, int slot) {
  ProfFrame& f = p.frames[slot];
  if (f.subs == 0) {
    g_prof_empty_reads.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const SIZE_T base = static_cast<SIZE_T>(slot) * kProfMaxSubs * 2 * sizeof(UINT64);
  D3D12_RANGE range = {base, base + static_cast<SIZE_T>(f.subs) * 2 * sizeof(UINT64)};
  void* mapped = nullptr;
  const HRESULT hr = p.readback->Map(0, &range, &mapped);
  if (FAILED(hr) || mapped == nullptr) {
    prof_fail(kFailMap, hr);
    return;
  }
  g_prof_reads.fetch_add(1, std::memory_order_relaxed);
  const UINT64* ts = reinterpret_cast<const UINT64*>(static_cast<const unsigned char*>(mapped) + base);
  UINT64 first = ~0ull, last = 0;
  double busy = 0.0;
  UINT64 prev_end = 0;
  const double qpc_per_tick = g_perf_freq.QuadPart != 0 ? static_cast<double>(g_perf_freq.QuadPart) / (p.ticks_per_ms * 1000.0) : 0.0;
  const double qpc_per_ms = static_cast<double>(g_perf_freq.QuadPart) / 1000.0;
  int block = 0;  // 0 main view, 1 second view, 2 after both: advanced after each eye's UI-pass submission
  Timeline* tl = nullptr;
  if ((f.frame_no & 63) == 0 && f.cal_gpu != 0 && qpc_per_tick > 0.0 && f.open_qpc != 0) {
    tl = &p.tl[p.tl_next];
    p.tl_next = (p.tl_next + 1) % 4;
    tl->frame_no = f.frame_no;
    tl->entries.clear();
  }
  for (int i = 0; i < f.subs; ++i) {
    if (i > 0 && f.meta[i - 1].ui_eye == 1 && block == 0) block = 1;
    if (i > 0 && f.meta[i - 1].ui_eye == 2) block = 2;
    const UINT64 b = ts[i * 2], e = ts[i * 2 + 1];
    if (b == 0 || e < b) {
      prev_end = 0;
      continue;
    }
    if (f.cal_gpu != 0 && qpc_per_tick > 0.0) {
      // GPU begin on the CPU clock, minus the CPU submit time: ~0 = the GPU was idle and took the work at once
      const double begin_qpc = static_cast<double>(f.cal_cpu) + (static_cast<double>(b) - static_cast<double>(f.cal_gpu)) * qpc_per_tick;
      const double latency = (begin_qpc - static_cast<double>(f.meta[i].qpc)) / qpc_per_ms;
      p.latency_ms += latency;
      p.latency_n += 1;
      p.pos_latency[i] += latency;
      if (prev_end != 0 && b > prev_end) {
        const double gap = static_cast<double>(b - prev_end) / p.ticks_per_ms;
        const int cause = latency < 0.25 ? 0 : f.meta[i].waits > 0 ? 1 : 2;
        p.gap_ms[cause] += gap;
        p.gap_n[cause] += 1;
        p.pos_gap[i][cause] += gap;
      }
    }
    p.pos_waits[i] += f.meta[i].waits;
    if (tl != nullptr) {
      const ProfSub& tm = f.meta[i];
      TimelineEntry te;
      const double gb = static_cast<double>(f.cal_cpu) + (static_cast<double>(b) - static_cast<double>(f.cal_gpu)) * qpc_per_tick;
      const double ge = static_cast<double>(f.cal_cpu) + (static_cast<double>(e) - static_cast<double>(f.cal_gpu)) * qpc_per_tick;
      te.begin_ms = (gb - static_cast<double>(f.open_qpc)) / qpc_per_ms;
      te.end_ms = (ge - static_cast<double>(f.open_qpc)) / qpc_per_ms;
      te.sub = i;
      char d[200];
      int dn = snprintf(d, sizeof(d), "cpu %.2f", (static_cast<double>(tm.qpc) - static_cast<double>(f.open_qpc)) / qpc_per_ms);
      for (unsigned w = 0; w < tm.nwait && dn > 0 && dn < static_cast<int>(sizeof(d)); ++w) {
        const char src = g_fence_queue[tm.wait_f[w]] ? g_fence_queue[tm.wait_f[w]] : '?';
        dn += snprintf(d + dn, sizeof(d) - dn, " wait[%c%u:%llu]", src, tm.wait_f[w], static_cast<unsigned long long>(tm.wait_v[w]));
      }
      if (tm.sig_f != 0 && dn > 0 && dn < static_cast<int>(sizeof(d))) {
        dn += snprintf(d + dn, sizeof(d) - dn, " signal[%c%u:%llu]", p.name[0], tm.sig_f, static_cast<unsigned long long>(tm.sig_v));
      }
      if (tm.dlss_vp >= 0 && dn > 0 && dn < static_cast<int>(sizeof(d))) dn += snprintf(d + dn, sizeof(d) - dn, " DLSS vp%d", tm.dlss_vp);
      if (tm.ui_eye != 0 && dn > 0 && dn < static_cast<int>(sizeof(d))) dn += snprintf(d + dn, sizeof(d) - dn, " UI eye%d", tm.ui_eye);
      te.desc = d;
      tl->entries.push_back(std::move(te));
    }
    prev_end = e;
    first = b < first ? b : first;
    last = e > last ? e : last;
    const double ms = static_cast<double>(e - b) / p.ticks_per_ms;
    busy += ms;
    const ProfSub& m = f.meta[i];
    p.tag_ms[m.tag] += ms;
    p.tag_subs[m.tag] += 1;
    p.pos_ms[i] += ms;
    p.pos_cpu[i] += m.cpu_ms;
    p.pos_n[i] += 1;
    p.pos_tag[i][m.tag] += 1;
    p.pos_lists[i] += m.lists;
    if (m.dlss_vp >= 0) {
      p.pos_dlss[i] += 1;
      p.pos_dlss_vp[i] = m.dlss_vp;
    }
    if (m.ui_eye == 1 || m.ui_eye == 2) {
      p.pos_ui[i][m.ui_eye] += 1;
    }
    p.pos_tid[i] = m.tid;
    for (unsigned k = 0; k < render_activity::kKinds; ++k) p.pos_activity[i][k] += m.activity.count[k];
    p.pos_activity_last[i] = m.activity;
    for (unsigned k = 0; k < m.stamp_count; ++k) {
      const render_activity::DispatchStamp& d = f.stamps[m.stamp_first + k];
      std::uint64_t sb = 0, se = 0;
      if (!render_activity::stamp_ticks(d.query, &sb, &se)) {
        ++p.stamp_misses;
        continue;
      }
      const double dms = static_cast<double>(se - sb) / p.ticks_per_ms;
      ShaderAgg& a = p.shader_agg[d.shader != 0 ? d.shader : (d.indirect ? 1 : 0)];
      a.ms += dms;
      a.n += 1;
      a.groups += static_cast<double>(d.x) * (d.indirect ? 1.0 : static_cast<double>(d.y) * d.z);
      a.positions[i / 64] |= 1ull << (i % 64);
      a.indirect = a.indirect || d.indirect;
      a.ms_block[block] += dms;
      p.stamped_ms += dms;
      p.stamped_n += 1;
    }
  }
  D3D12_RANGE none = {0, 0};
  p.readback->Unmap(0, &none);
  if (last > first) {
    p.span_ms += static_cast<double>(last - first) / p.ticks_per_ms;
  }
  p.busy_ms += busy;
  ++p.frames_read;
}

// Present thread: close this frame's slot (resolve + fence), open the next one if the GPU is done with it.
void prof_frame_end(ProfQueue& p, const LARGE_INTEGER& frame_origin) {
  std::lock_guard<std::mutex> lock(p.mutex);
  ProfFrame& f = p.frames[p.cur];
  if (f.open && f.used_lists > 0) {
    if (FAILED(p.queue->GetClockCalibration(&f.cal_gpu, &f.cal_cpu))) {
      f.cal_gpu = f.cal_cpu = 0;
    }
    bool resolved = false;
    ID3D12GraphicsCommandList* resolve = f.subs > 0 ? prof_list(p, f) : nullptr;
    if (resolve != nullptr) {
      const UINT first = static_cast<UINT>(p.cur * kProfMaxSubs * 2);
      resolve->ResolveQueryData(p.heap, D3D12_QUERY_TYPE_TIMESTAMP, first, static_cast<UINT>(f.subs * 2), p.readback,
                                static_cast<UINT64>(first) * sizeof(UINT64));
      const HRESULT hr = resolve->Close();
      if (FAILED(hr)) {
        prof_fail(kFailResolve, hr);
      } else {
        ID3D12CommandList* one[1] = {resolve};
        g_execute_command_lists(p.queue, 1, one);
        resolved = true;
      }
    }
    // Our lists are in flight: the slot's allocator may only be reset after this fence.
    if (FAILED(p.queue->Signal(p.fence, ++p.fence_value))) {
      p.ready = false;  // cannot tell when the GPU is done with our lists: stop profiling this queue for good
      logf("gpuprof %s: fence signal failed; profiling stopped", p.name);
      return;
    }
    f.fence_value = p.fence_value;
    f.pending = true;
    if (!resolved) {
      f.subs = 0;  // nothing to read
      f.stamp_n = 0;
    }
  }
  f.open = false;
  p.cur = (p.cur + 1) % kProfFrames;
  ProfFrame& n = p.frames[p.cur];
  if (n.pending) {
    if (p.fence->GetCompletedValue() < n.fence_value) {
      ++p.frames_skipped;  // still in flight: skip this frame rather than wait
      return;
    }
    prof_read(p, p.cur);
    n.pending = false;
  }
  if (n.used_lists > 0) {
    const HRESULT hr = n.alloc->Reset();
    if (FAILED(hr)) {
      prof_fail(kFailAllocReset, hr);
      return;
    }
  }
  n.used_lists = 0;
  n.subs = 0;
  n.stamp_n = 0;
  n.open_qpc = frame_origin.QuadPart;
  n.frame_no = g_prof_present_no.load(std::memory_order_relaxed);
  n.open = true;
}

void prof_report(ProfQueue& p) {
  if (p.frames_read == 0) {
    logf("gpuprof %s: no frames read (skipped %u)", p.name, p.frames_skipped);
    return;
  }
  const double fr = static_cast<double>(p.frames_read);
  char line[1024];
  int n = snprintf(line, sizeof(line), "gpuprof %s: %u frames (skipped %u, dropped subs %u) | busy %.2f span %.2f ms/frame |",
                   p.name, p.frames_read, p.frames_skipped, p.subs_dropped, p.busy_ms / fr, p.span_ms / fr);
  for (int t = 0; t < kProfTags - 1 && n > 0 && n < static_cast<int>(sizeof(line)); ++t) {
    n += snprintf(line + n, sizeof(line) - n, " %s %.2f (%.1f subs)", kProfTagNames[t], p.tag_ms[t] / fr, p.tag_subs[t] / fr);
  }
  logf("%s", line);
  logf("gpuprof %s gaps per frame: starved %.2f ms (%.1f), after queue Wait %.2f ms (%.1f), other %.2f ms (%.1f) | queue latency "
       "avg %.2f ms (GPU begin - CPU submit)",
       p.name, p.gap_ms[0] / fr, p.gap_n[0] / fr, p.gap_ms[1] / fr, p.gap_n[1] / fr, p.gap_ms[2] / fr, p.gap_n[2] / fr,
       p.latency_n > 0 ? p.latency_ms / p.latency_n : 0.0);
  // Per position in the frame: only positions present in most frames and costing at least 0.05 ms.
  for (int i = 0; i < kProfMaxSubs; ++i) {
    const bool marked = p.pos_dlss[i] > 0 || p.pos_ui[i][1] > 0 || p.pos_ui[i][2] > 0;
    if (p.pos_n[i] == 0 || (!marked && (p.pos_n[i] * 2 < p.frames_read || p.pos_ms[i] / p.pos_n[i] < 0.05))) {
      continue;
    }
    int best = 0;
    for (int t = 1; t < kProfTags; ++t) {
      best = p.pos_tag[i][t] > p.pos_tag[i][best] ? t : best;
    }
    char marks[96] = "";
    int mk = 0;
    if (p.pos_dlss[i] > 0) {
      mk += snprintf(marks + mk, sizeof(marks) - mk, " | DLSS vp%d %u/%u", p.pos_dlss_vp[i], p.pos_dlss[i], p.pos_n[i]);
    }
    for (int e = 1; e <= 2 && mk >= 0 && mk < static_cast<int>(sizeof(marks)); ++e) {
      if (p.pos_ui[i][e] > 0) {
        mk += snprintf(marks + mk, sizeof(marks) - mk, " | UI %s %u/%u", e == 1 ? "main" : "second", p.pos_ui[i][e], p.pos_n[i]);
      }
    }
    logf("gpuprof %s  sub %3d: gpu %.3f ms, cpu at %.2f ms after Present, %.1f lists, tag %s (%u/%u), thread %lu | gap before "
         "%.3f starved / %.3f wait / %.3f other, latency %.2f, waits %.1f%s", p.name, i,
         p.pos_ms[i] / p.pos_n[i], p.pos_cpu[i] / p.pos_n[i], static_cast<double>(p.pos_lists[i]) / p.pos_n[i],
         kProfTagNames[best], p.pos_tag[i][best], p.pos_n[i], p.pos_tid[i], p.pos_gap[i][0] / p.pos_n[i],
         p.pos_gap[i][1] / p.pos_n[i], p.pos_gap[i][2] / p.pos_n[i], p.pos_latency[i] / p.pos_n[i],
         static_cast<double>(p.pos_waits[i]) / p.pos_n[i], marks);
  }
  if (activity_wanted()) {
    logf("gpucensus extra hook coverage=0x%X (bits Reset/CopyTexture/CopyResource/Rays/ASbuild; draws/dispatch/indirect/barriers via existing camera hooks)", render_activity::extra_hook_coverage());
    for (int i = 0; i < kProfMaxSubs; ++i) {
      if (p.pos_n[i] == 0 || p.pos_ms[i]/p.pos_n[i] < 0.5) continue;
      const auto& a = p.pos_activity_last[i];
      const double samples = p.pos_n[i];
      logf("gpucensus %s sub %d: draw %.1f dispatch %.1f copy %.1f indirect %.1f rays %.1f ASbuild %.1f barrierCalls %.1f | first dimensions %ux%ux%u | afop stack RVAs %llX %llX %llX %llX (last recording; partial coverage)",
           p.name, i, p.pos_activity[i][0]/samples, p.pos_activity[i][1]/samples, p.pos_activity[i][2]/samples,
           p.pos_activity[i][3]/samples, p.pos_activity[i][4]/samples, p.pos_activity[i][5]/samples, p.pos_activity[i][6]/samples,
           a.x,a.y,a.z, static_cast<unsigned long long>(a.stack[0]), static_cast<unsigned long long>(a.stack[1]),
           static_cast<unsigned long long>(a.stack[2]), static_cast<unsigned long long>(a.stack[3]));
    }
    // Per compute shader: incremental GPU time between the stamps around each dispatch (includes overlap with earlier
    // unfinished work on this queue; attribution, not isolated cost). Hash 0 = PSO not seen at creation, 1 = indirect
    // with an unknown PSO (may be draws). Positions = submission positions above (main block, second block, late).
    logf("gpucensus %s shaders: %.1f stamped dispatches/frame, %.2f ms/frame attributed, %u unreadable, %u compute shaders seen",
         p.name, p.stamped_n / fr, p.stamped_ms / fr, p.stamp_misses, render_activity::shader_count());
    if (render_activity::stage_mode()) {
      logf("gpustages %s: stage runs above are the 'stamped dispatches'; %llu runs dropped so far (64 per list cap)", p.name,
           static_cast<unsigned long long>(render_activity::stage_overflow_count()));
    }
    std::vector<std::pair<std::uint64_t, const ShaderAgg*>> top;
    top.reserve(p.shader_agg.size());
    for (const auto& kv : p.shader_agg) top.emplace_back(kv.first, &kv.second);
    std::sort(top.begin(), top.end(), [](const auto& a, const auto& b) { return a.second->ms > b.second->ms; });
    for (size_t t = 0; t < top.size() && t < (render_activity::stage_mode() ? 60u : 32u); ++t) {
      const ShaderAgg& a = *top[t].second;
      if (a.ms / fr < 0.02) break;
      char pos[96] = "";
      int pn = 0, shown = 0;
      for (int b = 0; b < kProfMaxSubs && shown < 10 && pn >= 0 && pn < static_cast<int>(sizeof(pos)); ++b) {
        if (a.positions[b / 64] & (1ull << (b % 64))) {
          pn += snprintf(pos + pn, sizeof(pos) - pn, "%s%d", shown ? "," : "", b);
          ++shown;
        }
      }
      const unsigned tag = static_cast<unsigned>(top[t].first >> 56);
      if (tag == 0x5A || tag == 0x5B) {
        char stage[200];
        if (tag == 0x5A) {
          const unsigned outer = static_cast<unsigned>((top[t].first >> 16) & 0xFFFF);
          snprintf(stage, sizeof(stage), "%s%s%s", render_activity::stage_name(static_cast<unsigned>(top[t].first & 0xFFFF)),
                   outer ? " < " : "", outer ? render_activity::stage_name(outer) : "");
        } else {
          snprintf(stage, sizeof(stage), "(no scope on stack) first afop RVA 0x%llX",
                   static_cast<unsigned long long>(top[t].first & 0xFFFFFFFull));
        }
        logf("gpustages %s %s: %.3f ms/frame (main view %.3f | second view %.3f | after %.3f), %.1f runs/frame, positions %s",
             p.name, stage, a.ms / fr, a.ms_block[0] / fr, a.ms_block[1] / fr, a.ms_block[2] / fr, a.n / fr, pos);
        continue;
      }
      logf("gpucensus %s shader %016llX: %.3f ms/frame, %.2f dispatches/frame, avg %s %.0f, positions %s%s", p.name,
           static_cast<unsigned long long>(top[t].first), a.ms / fr, a.n / fr, a.indirect ? "maxcmds" : "groups",
           a.n ? a.groups / a.n : 0.0, pos, a.indirect ? " | indirect" : "");
    }
  }
  p.shader_agg.clear();
  p.stamped_ms = 0.0;
  p.stamped_n = p.stamp_misses = 0;
  for (int t = 0; t < kProfTags; ++t) {
    p.tag_ms[t] = 0.0;
    p.tag_subs[t] = 0;
  }
  for (int i = 0; i < kProfMaxSubs; ++i) {
    p.pos_ms[i] = p.pos_cpu[i] = 0.0;
    p.pos_n[i] = p.pos_lists[i] = p.pos_dlss[i] = 0;
    p.pos_ui[i][1] = p.pos_ui[i][2] = 0;
    p.pos_gap[i][0] = p.pos_gap[i][1] = p.pos_gap[i][2] = 0.0;
    p.pos_latency[i] = 0.0;
    p.pos_waits[i] = 0;
    for (unsigned k = 0; k < render_activity::kKinds; ++k) p.pos_activity[i][k] = 0;
    for (int t = 0; t < kProfTags; ++t) {
      p.pos_tag[i][t] = 0;
    }
  }
  p.span_ms = p.busy_ms = 0.0;
  for (int c = 0; c < 3; ++c) {
    p.gap_ms[c] = 0.0;
    p.gap_n[c] = 0;
  }
  p.latency_ms = 0.0;
  p.latency_n = 0;
  p.frames_read = p.frames_skipped = p.subs_dropped = 0;
}

void prof_arm(ID3D12CommandQueue* game_queue) {
  static bool tried = false;
  static const bool repair_only = control_file_starts_with(L"AvatarVR_gpuprof.txt", "repair", false);
  if (tried || (!gpu_prof_wanted() && !repair_only)) {
    return;
  }
  tried = true;
  render_activity::set_enabled(activity_wanted());
  if (repair_only) {
    const bool hooked = hook_slot(game_queue, kQueueExecuteCommandLists, reinterpret_cast<void*>(&ExecuteCommandLists_hook),
                                  reinterpret_cast<void**>(&g_execute_command_lists), "ID3D12CommandQueue::ExecuteCommandLists (repair samples)");
    mv_repair::set_profiling(hooked);
    logf("mvrepair GPU profiling: %s (sampled; no submission wrapping)", hooked ? "enabled" : "hook failed");
    return;
  }
  g_prof[0].name = "direct";
  g_prof[0].queue = game_queue;
  g_prof[0].type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  g_prof[1].name = "compute";
  g_prof[1].queue = g_compute_queue;
  g_prof[1].type = D3D12_COMMAND_LIST_TYPE_COMPUTE;
  for (ProfQueue& p : g_prof) {
    p.ready = p.queue != nullptr && prof_init(p);
    logf("gpuprof %s queue %p: %s", p.name, static_cast<void*>(p.queue), p.ready ? "ready" : "not profiled");
  }
  if (!g_prof[0].ready && !g_prof[1].ready) {
    return;
  }
  {
    HMODULE owner = nullptr;
    wchar_t path[MAX_PATH] = L"?";
    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           static_cast<LPCWSTR>(vtable_slot(game_queue, kQueueExecuteCommandLists)), &owner)) {
      GetModuleFileNameW(owner, path, MAX_PATH);
    }
    logf("gpuprof ExecuteCommandLists entry %p in %ls", vtable_slot(game_queue, kQueueExecuteCommandLists), path);
  }
  // Every hardware queue shares d3d12core's ExecuteCommandLists, so one hook serves both (the detour filters by queue).
  if (hook_slot(game_queue, kQueueExecuteCommandLists, reinterpret_cast<void*>(&ExecuteCommandLists_hook),
                reinterpret_cast<void**>(&g_execute_command_lists), "ID3D12CommandQueue::ExecuteCommandLists")) {
    if (g_prof[1].ready && vtable_slot(g_compute_queue, kQueueExecuteCommandLists) != vtable_slot(game_queue, kQueueExecuteCommandLists)) {
      g_prof[1].ready = false;
      logf("gpuprof compute: different ExecuteCommandLists entry; compute not profiled");
    }
    hook_slot(game_queue, kQueueSignal, reinterpret_cast<void*>(&QueueSignal_hook), reinterpret_cast<void**>(&g_queue_signal),
              "ID3D12CommandQueue::Signal (timeline)");
    if (g_queue_wait == nullptr) {  // wait_skip_arm may have installed it already
      hook_slot(game_queue, kQueueWait, reinterpret_cast<void*>(&QueueWait_hook), reinterpret_cast<void**>(&g_queue_wait),
                "ID3D12CommandQueue::Wait");
    }
    g_prof_armed.store(true, std::memory_order_release);
  }
}

void wait_skip_arm(ID3D12CommandQueue* game_queue) {
  static bool tried = false;
  if (tried || game_queue == nullptr) {
    return;
  }
  tried = true;
  if (!control_file_starts_with(L"AvatarVR_queue.txt", "waitskip", false)) {
    return;
  }
  // Every hardware queue shares d3d12core's Wait entry, so this covers the direct, compute and copy queues.
  if (g_queue_wait == nullptr &&
      !hook_slot(game_queue, kQueueWait, reinterpret_cast<void*>(&QueueWait_hook), reinterpret_cast<void**>(&g_queue_wait),
                 "ID3D12CommandQueue::Wait (wait skip)")) {
    logf("queue waits: hook failed; wait skip not active");
    return;
  }
  g_wait_skip.store(true, std::memory_order_relaxed);
  logf("queue waits: wait skip ON (already-satisfied queue Waits are dropped; AvatarVR_queue.txt)");
}

void wait_skip_report() {
  static ULONGLONG last = 0;
  static std::uint64_t seen0 = 0, skipped0 = 0;
  if (!g_wait_skip.load(std::memory_order_relaxed) && !g_wait_skip_broken.load(std::memory_order_relaxed)) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (last == 0) {
    last = now;
    return;
  }
  if (now - last < 5000) {
    return;
  }
  const double secs = static_cast<double>(now - last) / 1000.0;
  last = now;
  const std::uint64_t seen = g_waits_seen.load(), skipped = g_waits_skipped.load();
  const std::uint64_t ds = seen - seen0, dk = skipped - skipped0;
  seen0 = seen;
  skipped0 = skipped;
  logf("queue waits: %.0f/s seen, %.0f/s skipped (%.0f%%), %.0f/s passed to the queue | skip %s",
       ds / secs, dk / secs, ds ? 100.0 * static_cast<double>(dk) / static_cast<double>(ds) : 0.0, (ds - dk) / secs,
       g_wait_skip.load() ? "on" : "OFF (fence went backwards)");
}

// Every second report: one frame of both queues merged by GPU begin time (latest frame present in both rings).
void prof_print_timeline() {
  static unsigned calls = 0;
  if ((++calls & 1) != 0 || !g_prof[0].ready) return;
  const Timeline* best[2] = {nullptr, nullptr};
  for (int a = 0; a < 4; ++a) {
    const Timeline& t0 = g_prof[0].tl[a];
    if (t0.entries.empty()) continue;
    const Timeline* t1 = nullptr;
    for (int b = 0; b < 4 && g_prof[1].ready; ++b) {
      if (g_prof[1].tl[b].frame_no == t0.frame_no && !g_prof[1].tl[b].entries.empty()) t1 = &g_prof[1].tl[b];
    }
    if (best[0] == nullptr || t0.frame_no > best[0]->frame_no) {
      best[0] = &t0;
      best[1] = t1;
    }
  }
  if (best[0] == nullptr) return;
  struct Row { double b, e; char q; const TimelineEntry* te; };
  std::vector<Row> rows;
  for (int q = 0; q < 2; ++q) {
    if (best[q] == nullptr) continue;
    for (const TimelineEntry& te : best[q]->entries) rows.push_back({te.begin_ms, te.end_ms, q == 0 ? 'D' : 'C', &te});
  }
  std::sort(rows.begin(), rows.end(), [](const Row& x, const Row& y) { return x.b < y.b; });
  logf("gputimeline frame %u: %zu submissions (D direct, C compute%s); common Present QPC origin; ms after the Present that opened the frame; "
       "wait[<signalling queue><fence id>:value], signal after the submission",
       best[0]->frame_no, rows.size(), best[1] ? "" : " missing for this frame");
  double busy_end[2] = {0.0, 0.0};
  for (const Row& r : rows) {
    const int qi = r.q == 'D' ? 0 : 1;
    const double idle = busy_end[qi] > 0.0 && r.b > busy_end[qi] ? r.b - busy_end[qi] : 0.0;
    busy_end[qi] = r.e > busy_end[qi] ? r.e : busy_end[qi];
    logf("gputimeline %c s%02d %7.3f-%7.3f (%.3f ms, idle before %.3f) %s", r.q, r.te->sub, r.b, r.e, r.e - r.b, idle,
         r.te->desc.c_str());
  }
}

void prof_on_present() {
  if (!g_prof_armed.load(std::memory_order_acquire)) {
    return;
  }
  // Bound the diagnostic run. Start after gameplay stereo is ready rather
  // than spending the sample budget on the front end/loading screens.
  static ULONGLONG gameplay_started = 0;
  unsigned main_id = 0, second_id = 0;
  if (gameplay_started == 0 && native_stereo::stereo_viewport_ids(&main_id, &second_id)) gameplay_started = GetTickCount64();
  if (gameplay_started != 0 && GetTickCount64() - gameplay_started >= prof_window_ms()) {
    render_activity::set_enabled(false);
    g_prof_armed.store(false, std::memory_order_release);
    logf("gpuprof: automatic stop after %llus gameplay; rendering unchanged, submission wrapping disabled",
         static_cast<unsigned long long>(prof_window_ms() / 1000));
    return;
  }
  g_prof_present_no.fetch_add(1, std::memory_order_relaxed);
  // Both queues must use the SAME origin before their calibrated timestamps
  // are merged. Per-queue QPC calls shifted the old cross-queue timeline.
  LARGE_INTEGER frame_origin;
  QueryPerformanceCounter(&frame_origin);
  for (ProfQueue& p : g_prof) {
    if (p.ready) {
      prof_frame_end(p, frame_origin);
    }
  }
  QueryPerformanceCounter(&g_prof_present_qpc);
  const ULONGLONG now = GetTickCount64();
  if (g_prof_last_report == 0) {
    g_prof_last_report = now;
  } else if (now - g_prof_last_report >= 5000) {
    g_prof_last_report = now;
    logf("gpuprof stereo %s (tags: Gfx_View Render running at submit; main / second / other views) | hook calls %llu "
         "(other queues %llu), lists %llu, largest batch %u",
         native_stereo::stereo_running() ? "running" : "off", static_cast<unsigned long long>(g_ecl_calls.exchange(0)),
         static_cast<unsigned long long>(g_ecl_other_queue.exchange(0)), static_cast<unsigned long long>(g_ecl_lists.exchange(0)),
         g_ecl_max_lists.exchange(0));
    char fails[400];
    int n = snprintf(fails, sizeof(fails), "gpuprof health: stamped %llu, reads %llu (empty %llu), device %08lX | failures",
                     static_cast<unsigned long long>(g_prof_stamped.exchange(0)), static_cast<unsigned long long>(g_prof_reads.exchange(0)),
                     static_cast<unsigned long long>(g_prof_empty_reads.exchange(0)),
                     g_prof[0].device != nullptr ? static_cast<unsigned long>(g_prof[0].device->GetDeviceRemovedReason()) : 0ul);
    for (int k = 0; k < kFailCount && n > 0 && n < static_cast<int>(sizeof(fails)); ++k) {
      n += snprintf(fails + n, sizeof(fails) - n, " %s %u (first hr %08lX)", kFailNames[k], g_prof_fail[k].exchange(0),
                    static_cast<unsigned long>(g_prof_fail_hr[k].load()));
    }
    logf("%s", fails);
    for (ProfQueue& p : g_prof) {
      if (p.ready) {
        prof_report(p);
      }
    }
    prof_print_timeline();
  }
}

double perf_ms(const LARGE_INTEGER& a, const LARGE_INTEGER& b) {
  return static_cast<double>(b.QuadPart - a.QuadPart) * 1000.0 / static_cast<double>(g_perf_freq.QuadPart);
}

void perf_add(int stage, double ms) {
  g_perf_sum[stage] += ms;
  if (ms > g_perf_max[stage]) {
    g_perf_max[stage] = ms;
  }
}

// Read-only budget telemetry inside the game process. Retired stereo views and
// their temporal histories remain allocated, so budget pressure is a candidate
// for worsening performance after repeated loads (not yet proven).
void report_video_memory() {
  static ULONGLONG last = 0;
  static IDXGIAdapter3* adapter = nullptr;
  const ULONGLONG now = GetTickCount64();
  if (now-last < 5000 || g_stats_swapchain == nullptr) return;
  last = now;
  if (adapter == nullptr) {
    ID3D12Device* device = nullptr;
    IDXGIFactory4* factory = nullptr;
    if (SUCCEEDED(g_stats_swapchain->GetDevice(IID_PPV_ARGS(&device))) && device != nullptr) {
      if (SUCCEEDED(CreateDXGIFactory1(IID_PPV_ARGS(&factory))) && factory != nullptr) {
        factory->EnumAdapterByLuid(device->GetAdapterLuid(), IID_PPV_ARGS(&adapter));
        factory->Release();
      }
      device->Release();
    }
  }
  if (adapter == nullptr) return;
  DXGI_QUERY_VIDEO_MEMORY_INFO local = {}, nonlocal = {};
  if (SUCCEEDED(adapter->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &local))) {
    adapter->QueryVideoMemoryInfo(0, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, &nonlocal);
    constexpr double mib = 1024.0*1024.0;
    logf("video memory: process local %.0f MiB / budget %.0f MiB | nonlocal %.0f MiB / budget %.0f MiB",
         local.CurrentUsage/mib, local.Budget/mib, nonlocal.CurrentUsage/mib, nonlocal.Budget/mib);
  }
}

void perf_report(const LARGE_INTEGER& now) {
  const double window = perf_ms(g_perf_window, now);
  if (g_perf_window.QuadPart == 0 || window < 1000.0) {
    if (g_perf_window.QuadPart == 0) {
      g_perf_window = now;
    }
    return;
  }
  char line[1024];
  int n = snprintf(line, sizeof(line), "perf fps=%.1f interval avg=%.1f max=%.1f ms |", g_perf_frames * 1000.0 / window,
                   g_perf_frames > 0 ? window / g_perf_frames : 0.0, g_perf_interval_max);
  for (int k = 0; k < kPerfStages && n > 0 && n < static_cast<int>(sizeof(line)); ++k) {
    n += snprintf(line + n, sizeof(line) - n, " %s %.2f/%.1f", kPerfNames[k], g_perf_frames > 0 ? g_perf_sum[k] / g_perf_frames : 0.0,
                  g_perf_max[k]);
  }
  const std::uint64_t lag_n = g_lag_count.exchange(0), lag_sum = g_lag_sum_us.exchange(0), lag_max = g_lag_max_us.exchange(0);
  if (lag_n > 0 && n > 0 && n < static_cast<int>(sizeof(line))) {
    snprintf(line + n, sizeof(line) - n, " | gpu lag %.1f/%.1f", lag_sum / 1000.0 / lag_n, lag_max / 1000.0);
  }
  logf("%s (avg/max ms per frame)", line);
  report_video_memory();
  for (int k = 0; k < kPerfStages; ++k) {
    g_perf_sum[k] = 0.0;
    g_perf_max[k] = 0.0;
  }
  g_perf_interval_max = 0.0;
  g_perf_frames = 0;
  g_perf_window = now;
}

HRESULT STDMETHODCALLTYPE Present_hook(IDXGISwapChain* swapchain, UINT sync, UINT flags) {
  xr_runtime::pace_present();  // frame pacing (off unless "halfrate"); before the copies so the image leaves on time
  g_present_count.fetch_add(1);
  if (g_perf_freq.QuadPart == 0) {
    QueryPerformanceFrequency(&g_perf_freq);
  }
  LARGE_INTEGER t0, t1;
  QueryPerformanceCounter(&t0);
  if (g_perf_last_return.QuadPart != 0) {
    perf_add(kPerfGame, perf_ms(g_perf_last_return, t0));
    const double interval = perf_ms(g_perf_last_entry, t0);
    if (interval > g_perf_interval_max) {
      g_perf_interval_max = interval;
    }
  }
  g_perf_last_entry = t0;
  auto stage = [&](int k) {
    QueryPerformanceCounter(&t1);
    perf_add(k, perf_ms(t0, t1));
    t0 = t1;
  };
  upload::on_present();
  stage(kPerfUpload);
  eye_probe::on_present();
  input_probe::on_present();
  player_probe::on_present();
  lua_host::on_present();  // scripts/avr: "frame" event and debug drawing, before vr_menu draws them
  virtual_pad::on_present();
  stage(kPerfEyeProbe);
  frame_capture::on_present(swapchain);
  stage(kPerfCapture);
  xr_runtime::on_present(swapchain);
  stage(kPerfXr);
  vr_menu::on_present(swapchain);  // after the eye copies, so the menu never bakes into an eye image
  stage(kPerfMenu);
  native_stereo::on_present();
  hitch_probe::on_present(native_stereo::stereo_running() && !native_stereo::front_end());
  stage(kPerfNative);
  eye_write::on_present();
  stage(kPerfEye);
  time_sync::on_present();
  stage(kPerfTime);
  prof_on_present();  // off unless AvatarVR_gpuprof.txt says "on"
  wait_skip_report();  // off unless AvatarVR_queue.txt says "waitskip"
  update_source_size(swapchain);  // eye mode: present the client-size top-left of grown buffers
  lag_signal();
  const HRESULT hr = g_present(swapchain, sync, flags);
  stage(kPerfPresent);
  ++g_perf_frames;
  perf_report(t0);
  const ULONGLONG now = GetTickCount64();
  ULONGLONG last = g_last_present_log_tick.load();
  if (last == 0 || now - last >= 1000) {
    if (g_last_present_log_tick.compare_exchange_strong(last, now)) {
      log_present_sample(swapchain, sync, flags, hr);
    }
  }
  QueryPerformanceCounter(&g_perf_last_return);
  return hr;
}

// Observer only: how the game sizes its buffers after a window resize (explicit size, or 0x0 = "take the client
// area"), next to the window's client and outer size at that moment. Arguments are passed through unchanged.
void log_resize(const char* name, IDXGISwapChain* swapchain, UINT count, UINT width, UINT height, DXGI_FORMAT format, UINT flags) {
  RECT client = {}, outer = {};
  DXGI_SWAP_CHAIN_DESC desc = {};
  if (swapchain != nullptr && SUCCEEDED(swapchain->GetDesc(&desc)) && desc.OutputWindow != nullptr) {
    GetClientRect(desc.OutputWindow, &client);
    GetWindowRect(desc.OutputWindow, &outer);
  }
  logf("%s count=%u size=%ux%u format=%u:%s flags=0x%X (current %ux%u) client=%ldx%ld window=%ldx%ld thread=%lu", name, count,
       width, height, static_cast<unsigned>(format), format_name(format), flags, desc.BufferDesc.Width, desc.BufferDesc.Height,
       client.right - client.left, client.bottom - client.top, outer.right - outer.left, outer.bottom - outer.top,
       GetCurrentThreadId());
}

HWND swapchain_window(IDXGISwapChain* swapchain) {
  DXGI_SWAP_CHAIN_DESC d = {};
  return SUCCEEDED(swapchain->GetDesc(&d)) ? d.OutputWindow : nullptr;
}

HRESULT STDMETHODCALLTYPE ResizeBuffers_hook(IDXGISwapChain* swapchain, UINT count, UINT width, UINT height, DXGI_FORMAT format, UINT flags) {
  g_eyebuf_read = 0;  // a resize must use the just-written eye request, not the one-second cache
  log_resize("ResizeBuffers", swapchain, count, width, height, format, flags);
  if (grow_buffers(swapchain_window(swapchain), &width, &height)) {
    logf("ResizeBuffers: eye mode, buffers grown to %ux%u", width, height);
  }
  g_source_w = g_source_h = 0;  // a resize resets the source size
  const HRESULT hr = g_resize_buffers(swapchain, count, width, height, format, flags);
  logf("ResizeBuffers hr=0x%08lX", static_cast<unsigned long>(hr));
  if (hr == DXGI_ERROR_INVALID_CALL) {
    // Buffers still referenced (stereo's second view / shared UI pass): pause stereo so the game's next try succeeds.
    native_stereo::hold_for_resize(3000);
    logf("ResizeBuffers failed with buffers in use: stereo paused for 3 s");
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE ResizeBuffers1_hook(IDXGISwapChain3* swapchain, UINT count, UINT width, UINT height, DXGI_FORMAT format, UINT flags,
                                              const UINT* node_masks, IUnknown* const* queues) {
  g_eyebuf_read = 0;
  log_resize("ResizeBuffers1", swapchain, count, width, height, format, flags);
  if (grow_buffers(swapchain_window(swapchain), &width, &height)) {
    logf("ResizeBuffers1: eye mode, buffers grown to %ux%u", width, height);
  }
  g_source_w = g_source_h = 0;
  const HRESULT hr = g_resize_buffers1(swapchain, count, width, height, format, flags, node_masks, queues);
  logf("ResizeBuffers1 hr=0x%08lX", static_cast<unsigned long>(hr));
  if (hr == DXGI_ERROR_INVALID_CALL) {
    // Buffers still referenced (stereo's second view / shared UI pass): pause stereo so the game's next try succeeds.
    native_stereo::hold_for_resize(3000);
    logf("ResizeBuffers1 failed with buffers in use: stereo paused for 3 s");
  }
  return hr;
}

void on_swapchain(IDXGISwapChain* swapchain, const char* source) {
  describe_swapchain(swapchain, source);
  hook_slot(swapchain, kSwapChainPresent, reinterpret_cast<void*>(&Present_hook), reinterpret_cast<void**>(&g_present), "IDXGISwapChain::Present");
  hook_slot(swapchain, kSwapChainResizeBuffers, reinterpret_cast<void*>(&ResizeBuffers_hook), reinterpret_cast<void**>(&g_resize_buffers),
            "IDXGISwapChain::ResizeBuffers");
  IDXGISwapChain3* sc3 = nullptr;
  if (SUCCEEDED(swapchain->QueryInterface(IID_PPV_ARGS(&sc3))) && sc3 != nullptr) {
    hook_slot(sc3, kSwapChain3ResizeBuffers1, reinterpret_cast<void*>(&ResizeBuffers1_hook), reinterpret_cast<void**>(&g_resize_buffers1),
              "IDXGISwapChain3::ResizeBuffers1");
    sc3->Release();
  }
}

HRESULT STDMETHODCALLTYPE CreateSwapChain_hook(IDXGIFactory* factory, IUnknown* device, DXGI_SWAP_CHAIN_DESC* desc, IDXGISwapChain** out_swapchain) {
  DXGI_SWAP_CHAIN_DESC grown = {};
  if (desc != nullptr) {
    grown = *desc;
    if (grow_buffers(desc->OutputWindow, &grown.BufferDesc.Width, &grown.BufferDesc.Height)) {
      logf("CreateSwapChain: eye mode, buffers grown to %ux%u", grown.BufferDesc.Width, grown.BufferDesc.Height);
      desc = &grown;
    }
  }
  g_source_w = g_source_h = 0;
  const HRESULT hr = g_create_swapchain(factory, device, desc, out_swapchain);
  if (SUCCEEDED(hr) && out_swapchain != nullptr && *out_swapchain != nullptr) {
    frame_capture::set_queue(device);
    xr_runtime::set_queue(device);
    on_swapchain(*out_swapchain, "CreateSwapChain");
  } else {
    logf("CreateSwapChain hr=0x%08lX thread=%lu", static_cast<unsigned long>(hr), GetCurrentThreadId());
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE CreateSwapChainForHwnd_hook(IDXGIFactory2* factory, IUnknown* device, HWND window, const DXGI_SWAP_CHAIN_DESC1* desc, const DXGI_SWAP_CHAIN_FULLSCREEN_DESC* fullscreen, IDXGIOutput* restrict_output, IDXGISwapChain1** out_swapchain) {
  DXGI_SWAP_CHAIN_DESC1 grown = {};
  if (desc != nullptr) {
    grown = *desc;
    if (grow_buffers(window, &grown.Width, &grown.Height)) {
      logf("CreateSwapChainForHwnd: eye mode, buffers grown to %ux%u", grown.Width, grown.Height);
      desc = &grown;
    }
  }
  g_source_w = g_source_h = 0;
  const HRESULT hr = g_create_swapchain_hwnd(factory, device, window, desc, fullscreen, restrict_output, out_swapchain);
  if (SUCCEEDED(hr) && out_swapchain != nullptr && *out_swapchain != nullptr) {
    frame_capture::set_queue(device);
    xr_runtime::set_queue(device);
    on_swapchain(*out_swapchain, "CreateSwapChainForHwnd");
  } else {
    logf("CreateSwapChainForHwnd hr=0x%08lX thread=%lu", static_cast<unsigned long>(hr), GetCurrentThreadId());
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE CreateSwapChainForCoreWindow_hook(IDXGIFactory2* factory, IUnknown* device, IUnknown* window, const DXGI_SWAP_CHAIN_DESC1* desc, IDXGIOutput* restrict_output, IDXGISwapChain1** out_swapchain) {
  const HRESULT hr = g_create_swapchain_core(factory, device, window, desc, restrict_output, out_swapchain);
  if (SUCCEEDED(hr) && out_swapchain != nullptr && *out_swapchain != nullptr) {
    frame_capture::set_queue(device);
    xr_runtime::set_queue(device);
    on_swapchain(*out_swapchain, "CreateSwapChainForCoreWindow");
  } else {
    logf("CreateSwapChainForCoreWindow hr=0x%08lX thread=%lu", static_cast<unsigned long>(hr), GetCurrentThreadId());
  }
  return hr;
}

void watch_factory(IUnknown* object) {
  if (object == nullptr) {
    return;
  }
  IDXGIFactory* factory = nullptr;
  if (SUCCEEDED(object->QueryInterface(IID_PPV_ARGS(&factory)))) {
    hook_slot(factory, kFactoryCreateSwapChain, reinterpret_cast<void*>(&CreateSwapChain_hook), reinterpret_cast<void**>(&g_create_swapchain), "IDXGIFactory::CreateSwapChain");
    factory->Release();
  }
  IDXGIFactory2* factory2 = nullptr;
  if (SUCCEEDED(object->QueryInterface(IID_PPV_ARGS(&factory2)))) {
    hook_slot(factory2, kFactory2CreateSwapChainForHwnd, reinterpret_cast<void*>(&CreateSwapChainForHwnd_hook), reinterpret_cast<void**>(&g_create_swapchain_hwnd), "IDXGIFactory2::CreateSwapChainForHwnd");
    hook_slot(factory2, kFactory2CreateSwapChainForCoreWindow, reinterpret_cast<void*>(&CreateSwapChainForCoreWindow_hook), reinterpret_cast<void**>(&g_create_swapchain_core), "IDXGIFactory2::CreateSwapChainForCoreWindow");
    factory2->Release();
  }
}

HRESULT WINAPI CreateDXGIFactory_hook(REFIID iid, void** factory) {
  const HRESULT hr = g_create_factory(iid, factory);
  char text[80];
  guid_text(iid, text, sizeof(text));
  logf("CreateDXGIFactory hr=0x%08lX iid=%s factory=%p thread=%lu", static_cast<unsigned long>(hr), text, (SUCCEEDED(hr) && factory != nullptr) ? *factory : nullptr, GetCurrentThreadId());
  if (SUCCEEDED(hr) && factory != nullptr && *factory != nullptr) {
    watch_factory(static_cast<IUnknown*>(*factory));
  }
  return hr;
}

HRESULT WINAPI CreateDXGIFactory1_hook(REFIID iid, void** factory) {
  const HRESULT hr = g_create_factory1(iid, factory);
  char text[80];
  guid_text(iid, text, sizeof(text));
  logf("CreateDXGIFactory1 hr=0x%08lX iid=%s factory=%p thread=%lu", static_cast<unsigned long>(hr), text, (SUCCEEDED(hr) && factory != nullptr) ? *factory : nullptr, GetCurrentThreadId());
  if (SUCCEEDED(hr) && factory != nullptr && *factory != nullptr) {
    watch_factory(static_cast<IUnknown*>(*factory));
  }
  return hr;
}

HRESULT WINAPI CreateDXGIFactory2_hook(UINT flags, REFIID iid, void** factory) {
  const HRESULT hr = g_create_factory2(flags, iid, factory);
  char text[80];
  guid_text(iid, text, sizeof(text));
  logf("CreateDXGIFactory2 flags=0x%X hr=0x%08lX iid=%s factory=%p thread=%lu", flags, static_cast<unsigned long>(hr), text, (SUCCEEDED(hr) && factory != nullptr) ? *factory : nullptr, GetCurrentThreadId());
  if (SUCCEEDED(hr) && factory != nullptr && *factory != nullptr) {
    watch_factory(static_cast<IUnknown*>(*factory));
  }
  return hr;
}

HRESULT WINAPI D3D12CreateDevice_hook(IUnknown* adapter, D3D_FEATURE_LEVEL level, REFIID iid, void** device) {
  const HRESULT hr = g_d3d12_create_device(adapter, level, iid, device);
  char text[80];
  guid_text(iid, text, sizeof(text));
  logf("D3D12CreateDevice hr=0x%08lX feature=0x%X iid=%s adapter=%p device=%p thread=%lu",
       static_cast<unsigned long>(hr), static_cast<unsigned>(level), text, adapter,
       (SUCCEEDED(hr) && device != nullptr) ? *device : nullptr, GetCurrentThreadId());
  if (SUCCEEDED(hr) && device != nullptr && *device != nullptr) {
    watch_device(static_cast<IUnknown*>(*device));
  }
  return hr;
}

bool module_leaf_is(HMODULE module, const wchar_t* leaf) {
  const std::wstring path = module_path(module);
  const auto slash = path.find_last_of(L"\\/");
  const std::wstring name = slash == std::wstring::npos ? path : path.substr(slash + 1);
  return _wcsicmp(name.c_str(), leaf) == 0;
}

void hook_dxgi_exports() {
  HMODULE dxgi = GetModuleHandleW(L"dxgi.dll");
  if (dxgi == nullptr) {
    logf("dxgi.dll not loaded");
    return;
  }
  hook_target(GetProcAddress(dxgi, "CreateDXGIFactory"), reinterpret_cast<void*>(&CreateDXGIFactory_hook), reinterpret_cast<void**>(&g_create_factory), "CreateDXGIFactory");
  hook_target(GetProcAddress(dxgi, "CreateDXGIFactory1"), reinterpret_cast<void*>(&CreateDXGIFactory1_hook), reinterpret_cast<void**>(&g_create_factory1), "CreateDXGIFactory1");
  hook_target(GetProcAddress(dxgi, "CreateDXGIFactory2"), reinterpret_cast<void*>(&CreateDXGIFactory2_hook), reinterpret_cast<void**>(&g_create_factory2), "CreateDXGIFactory2");
}

void hook_d3d12_export() {
  HMODULE d3d12 = GetModuleHandleW(L"d3d12.dll");
  if (d3d12 == nullptr) {
    logf("d3d12.dll not loaded");
    return;
  }
  hook_target(GetProcAddress(d3d12, "D3D12CreateDevice"), reinterpret_cast<void*>(&D3D12CreateDevice_hook), reinterpret_cast<void**>(&g_d3d12_create_device), "D3D12CreateDevice");
}

void note_loaded_module(HMODULE module) {
  if (module == nullptr) {
    return;
  }
  if (module_leaf_is(module, L"dxgi.dll")) {
    hook_dxgi_exports();
  }
  if (module_leaf_is(module, L"d3d12.dll")) {
    hook_d3d12_export();
  }
  if (module_leaf_is(module, L"libxess.dll")) {
    xess_twin::hook(module);
  }
  if (module_leaf_is(module, L"sl.interposer.dll")) {
    dlss_twin::hook(module);
  }
}

HMODULE WINAPI LoadLibraryW_hook(LPCWSTR name) {
  const HMODULE module = g_load_library_w(name);
  note_loaded_module(module);
  return module;
}

HMODULE WINAPI LoadLibraryExW_hook(LPCWSTR name, HANDLE file, DWORD flags) {
  const HMODULE module = g_load_library_ex_w(name, file, flags);
  note_loaded_module(module);
  return module;
}

void hook_loader_exports() {
  HMODULE kernel = GetModuleHandleW(L"kernel32.dll");
  if (kernel == nullptr) {
    logf("kernel32.dll not loaded");
    return;
  }
  hook_target(GetProcAddress(kernel, "LoadLibraryW"), reinterpret_cast<void*>(&LoadLibraryW_hook), reinterpret_cast<void**>(&g_load_library_w), "LoadLibraryW");
  hook_target(GetProcAddress(kernel, "LoadLibraryExW"), reinterpret_cast<void*>(&LoadLibraryExW_hook), reinterpret_cast<void**>(&g_load_library_ex_w), "LoadLibraryExW");
}

DWORD WINAPI install_thread(LPVOID) {
  logf("dx12 observer build %s %s thread=%lu", __DATE__, __TIME__, GetCurrentThreadId());
  frame_capture::install();
  xr_runtime::install();
  resource_names::install();
  xess_twin::install();
  dlss_twin::install();
  const MH_STATUS init = MH_Initialize();
  if (init != MH_OK && init != MH_ERROR_ALREADY_INITIALIZED) {
    logf("MH_Initialize status=%d", static_cast<int>(init));
    return 0;
  }
  hitch_probe::install(parent_dir(g_log_path).c_str());  // opt-in (AvatarVR_hitch.txt "on"): file calls timed from here
  hook_loader_exports();
  hook_client_rect();
  input_probe::install();  // early: the game enumerates pads at start-up and on hot-plug
  virtual_pad::install();  // before the game creates its gamepad backend (WGI vs XInput is decided once)
  hook_dxgi_exports();
  hook_d3d12_export();
  if (HMODULE xess = GetModuleHandleW(L"libxess.dll")) {
    xess_twin::hook(xess);  // statically imported by afop.exe, so usually already mapped
  }
  if (HMODULE sl = GetModuleHandleW(L"sl.interposer.dll")) {
    dlss_twin::hook(sl);
  }
  logf("dx12 observer armed");
  return 0;
}

}  // namespace

namespace dx12 {

bool ui_window_is_eye() {
  std::uint32_t w = 0, h = 0;
  return g_ui_spoof_wanted.load() && g_get_client_rect != nullptr && native_stereo::eye_size(&w, &h);
}

bool client_size(unsigned* w, unsigned* h) {
  *w = g_client_w.load();
  *h = g_client_h.load();
  return g_buffers_grown.load() && *w > 0 && *h > 0;
}

void mark_list(void* list, char kind, int value) {
  if (list == nullptr || !g_prof_armed.load(std::memory_order_relaxed)) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_mark_mutex);
  int slot = -1;
  for (int m = 0; m < kMaxMarks; ++m) {
    if (g_marks[m].list == list && g_marks[m].kind == kind) {
      slot = m;  // same list marked again before submission
      break;
    }
    if (slot < 0 && g_marks[m].list == nullptr) {
      slot = m;
    }
  }
  if (slot < 0) {
    static int victim = 0;
    slot = victim;  // full (marks never submitted): overwrite round-robin
    victim = (victim + 1) % kMaxMarks;
  }
  g_marks[slot] = {list, kind, value};
}

void install() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&install), &self)) {
    return;
  }
  g_log_path = parent_dir(module_path(self)) + L"\\DX12_OBSERVE.log";
  HANDLE thread = CreateThread(nullptr, 0, install_thread, nullptr, 0, nullptr);
  if (thread != nullptr) {
    CloseHandle(thread);
  }
}

}  // namespace dx12
