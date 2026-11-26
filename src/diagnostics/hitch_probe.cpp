#include "../runtime_log.hpp"
#include "hitch_probe.hpp"

#include <windows.h>

#include <MinHook.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>

namespace {

const char* const kKindNames[hitch_probe::kKindCount] = {
    "gfxPSO", "computePSO", "streamPSO", "stateObject", "psoLibrary", "committedRes", "placedRes",
    "reservedRes", "heap", "residency", "fileOpen", "fileRead", "fileWrite"};

std::atomic<bool> g_on{false};
std::wstring g_log_path;
std::mutex g_log_mutex;
double g_qpc_per_us = 0.0;

struct Tally {
  std::atomic<std::uint64_t> count{0}, total_us{0}, max_us{0};
  std::atomic<DWORD> max_tid{0};
};
Tally g_frame[hitch_probe::kKindCount];   // current frame (any thread)
std::uint64_t g_win_count[hitch_probe::kKindCount] = {};  // 10 s window (Present thread)
std::uint64_t g_win_us[hitch_probe::kKindCount] = {};
std::uint64_t g_win_max[hitch_probe::kKindCount] = {};

void logf(const char* fmt, ...) {
  char buffer[1600];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty()) return;
  // Opened per line through the ORIGINAL file API pointers would still be counted as our own writes; the log is only
  // written on hitches and every 10 s, so its own cost is visible and tiny.
  SYSTEMTIME t;
  GetLocalTime(&t);
  char stamp[32];
  snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u.%03u ", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds);
  runtime_log::append(g_log_path, {stamp, buffer, "\r\n"});
}

std::uint64_t qpc_us() {
  LARGE_INTEGER c;
  QueryPerformanceCounter(&c);
  return static_cast<std::uint64_t>(static_cast<double>(c.QuadPart) / g_qpc_per_us);
}

void add(hitch_probe::Kind k, std::uint64_t us) {
  Tally& t = g_frame[k];
  t.count.fetch_add(1, std::memory_order_relaxed);
  t.total_us.fetch_add(us, std::memory_order_relaxed);
  std::uint64_t prev = t.max_us.load(std::memory_order_relaxed);
  while (us > prev && !t.max_us.compare_exchange_weak(prev, us, std::memory_order_relaxed)) {
  }
  if (us >= prev) t.max_tid.store(GetCurrentThreadId(), std::memory_order_relaxed);
}

// Timed pass-through detours. Each keeps the original pointer in a template-static slot.
template <hitch_probe::Kind K, class Fn>
struct Timed;
template <hitch_probe::Kind K, class R, class... A>
struct Timed<K, R(STDMETHODCALLTYPE*)(A...)> {
  static inline R(STDMETHODCALLTYPE* original)(A...) = nullptr;
  static R STDMETHODCALLTYPE detour(A... a) {
    const std::uint64_t t0 = qpc_us();
    R r = original(a...);
    add(K, qpc_us() - t0);
    return r;
  }
};

template <class T>
void hook_slot(void* object, int slot, const char* what) {
  void* target = (*reinterpret_cast<void***>(object))[slot];
  const MH_STATUS s = MH_CreateHook(target, reinterpret_cast<void*>(&T::detour), reinterpret_cast<void**>(&T::original));
  if (s == MH_OK) MH_EnableHook(target);
  logf("hook %s (slot %d): %d", what, slot, static_cast<int>(s));
}
template <class T>
void hook_proc(void* target, const char* what) {
  if (target == nullptr) return;
  const MH_STATUS s = MH_CreateHook(target, reinterpret_cast<void*>(&T::detour), reinterpret_cast<void**>(&T::original));
  if (s == MH_OK) MH_EnableHook(target);
  logf("hook %s: %d", what, static_cast<int>(s));
}

using D = ID3D12Device;
using ComputePsoFn = HRESULT(STDMETHODCALLTYPE*)(D*, const D3D12_COMPUTE_PIPELINE_STATE_DESC*, REFIID, void**);
using CommittedFn = HRESULT(STDMETHODCALLTYPE*)(D*, const D3D12_HEAP_PROPERTIES*, D3D12_HEAP_FLAGS, const D3D12_RESOURCE_DESC*,
                                                D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*, REFIID, void**);
using HeapFn = HRESULT(STDMETHODCALLTYPE*)(D*, const D3D12_HEAP_DESC*, REFIID, void**);
using PlacedFn = HRESULT(STDMETHODCALLTYPE*)(D*, ID3D12Heap*, UINT64, const D3D12_RESOURCE_DESC*, D3D12_RESOURCE_STATES,
                                             const D3D12_CLEAR_VALUE*, REFIID, void**);
using ReservedFn = HRESULT(STDMETHODCALLTYPE*)(D*, const D3D12_RESOURCE_DESC*, D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*,
                                               REFIID, void**);
using ResidencyFn = HRESULT(STDMETHODCALLTYPE*)(D*, UINT, ID3D12Pageable* const*);
using LibraryFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device1*, const void*, SIZE_T, REFIID, void**);
using StreamPsoFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device2*, const D3D12_PIPELINE_STATE_STREAM_DESC*, REFIID, void**);
using StateObjectFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device5*, const D3D12_STATE_OBJECT_DESC*, REFIID, void**);
using Committed1Fn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device4*, const D3D12_HEAP_PROPERTIES*, D3D12_HEAP_FLAGS,
                                                 const D3D12_RESOURCE_DESC*, D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*,
                                                 ID3D12ProtectedResourceSession*, REFIID, void**);
using Heap1Fn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device4*, const D3D12_HEAP_DESC*, ID3D12ProtectedResourceSession*, REFIID, void**);
using Committed2Fn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device8*, const D3D12_HEAP_PROPERTIES*, D3D12_HEAP_FLAGS,
                                                 const D3D12_RESOURCE_DESC1*, D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*,
                                                 ID3D12ProtectedResourceSession*, REFIID, void**);
using Placed1Fn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device8*, ID3D12Heap*, UINT64, const D3D12_RESOURCE_DESC1*,
                                              D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*, REFIID, void**);
using CreateFileWFn = HANDLE(WINAPI*)(LPCWSTR, DWORD, DWORD, LPSECURITY_ATTRIBUTES, DWORD, DWORD, HANDLE);
using ReadFileFn = BOOL(WINAPI*)(HANDLE, LPVOID, DWORD, LPDWORD, LPOVERLAPPED);
using WriteFileFn = BOOL(WINAPI*)(HANDLE, LPCVOID, DWORD, LPDWORD, LPOVERLAPPED);

std::atomic<int> g_frames_long{0};
std::uint64_t g_last_present_us = 0;
std::uint64_t g_window_start_us = 0;
std::uint64_t g_window_frames = 0, g_window_long = 0, g_window_worst_us = 0;

}  // namespace

namespace hitch_probe {

std::uint64_t now_us() { return qpc_us(); }
bool enabled() { return g_on.load(std::memory_order_relaxed); }

void install(const wchar_t* scripts_dir) {
  if (scripts_dir == nullptr || !g_log_path.empty()) return;
  LARGE_INTEGER f;
  QueryPerformanceFrequency(&f);
  g_qpc_per_us = static_cast<double>(f.QuadPart) / 1e6;
  char text[16] = {};
  HANDLE c = CreateFileW((std::wstring(scripts_dir) + L"\\AvatarVR_hitch.txt").c_str(), GENERIC_READ,
                         FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (c == INVALID_HANDLE_VALUE) return;
  DWORD got = 0;
  ReadFile(c, text, sizeof(text) - 1, &got, nullptr);
  CloseHandle(c);
  if (strncmp(text, "on", 2) != 0) return;
  g_log_path = std::wstring(scripts_dir) + L"\\HITCH_PROBE.log";
  logf("hitch probe build %s %s pid=%lu: frames > 40 ms in gameplay are itemised (count / total ms / longest ms @thread)",
       __DATE__, __TIME__, GetCurrentProcessId());
  HMODULE k = GetModuleHandleW(L"kernelbase.dll");
  if (k == nullptr) k = GetModuleHandleW(L"kernel32.dll");
  hook_proc<Timed<kFileOpen, CreateFileWFn>>(reinterpret_cast<void*>(GetProcAddress(k, "CreateFileW")), "CreateFileW");
  hook_proc<Timed<kFileRead, ReadFileFn>>(reinterpret_cast<void*>(GetProcAddress(k, "ReadFile")), "ReadFile");
  hook_proc<Timed<kFileWrite, WriteFileFn>>(reinterpret_cast<void*>(GetProcAddress(k, "WriteFile")), "WriteFile");
  g_on.store(true);
}

void watch_device(ID3D12Device* device) {
  static bool done = false;
  if (!g_on.load() || device == nullptr || done) return;
  done = true;
  // ID3D12Device vtable slots (Windows SDK 10.0.26100.0): 11 CreateComputePipelineState, 27 CreateCommittedResource,
  // 28 CreateHeap, 29 CreatePlacedResource, 30 CreateReservedResource, 34 MakeResident, 35 Evict; Device1 44
  // CreatePipelineLibrary; Device2 47 CreatePipelineState; Device4 53 CreateCommittedResource1, 54 CreateHeap1; Device5
  // 62 CreateStateObject; Device8 69 CreateCommittedResource2, 70 CreatePlacedResource1. Slot 10 (graphics PSO) is
  // camera.cpp's hook, which reports its time through note(). A slot hooked already (census) logs MH status 9.
  hook_slot<Timed<kComputePso, ComputePsoFn>>(device, 11, "CreateComputePipelineState");
  hook_slot<Timed<kCommittedResource, CommittedFn>>(device, 27, "CreateCommittedResource");
  hook_slot<Timed<kHeap, HeapFn>>(device, 28, "CreateHeap");
  hook_slot<Timed<kPlacedResource, PlacedFn>>(device, 29, "CreatePlacedResource");
  hook_slot<Timed<kReservedResource, ReservedFn>>(device, 30, "CreateReservedResource");
  hook_slot<Timed<kResidency, ResidencyFn>>(device, 34, "MakeResident");
  hook_slot<Timed<kResidency, ResidencyFn>>(device, 35, "Evict");
  ID3D12Device8* d8 = nullptr;
  if (SUCCEEDED(device->QueryInterface(IID_PPV_ARGS(&d8))) && d8 != nullptr) {
    hook_slot<Timed<kPipelineLibrary, LibraryFn>>(d8, 44, "CreatePipelineLibrary");
    hook_slot<Timed<kStreamPso, StreamPsoFn>>(d8, 47, "CreatePipelineState (stream)");
    hook_slot<Timed<kCommittedResource, Committed1Fn>>(d8, 53, "CreateCommittedResource1");
    hook_slot<Timed<kHeap, Heap1Fn>>(d8, 54, "CreateHeap1");
    hook_slot<Timed<kStateObject, StateObjectFn>>(d8, 62, "CreateStateObject");
    hook_slot<Timed<kCommittedResource, Committed2Fn>>(d8, 69, "CreateCommittedResource2");
    hook_slot<Timed<kPlacedResource, Placed1Fn>>(d8, 70, "CreatePlacedResource1");
    d8->Release();
  } else {
    logf("ID3D12Device8 unavailable: only base device methods hooked");
  }
}

void note(Kind kind, std::uint64_t us) {
  if (g_on.load(std::memory_order_relaxed) && kind >= 0 && kind < kKindCount) add(kind, us);
}

void on_present(bool gameplay) {
  if (!g_on.load(std::memory_order_relaxed)) return;
  const std::uint64_t now = qpc_us();
  const std::uint64_t interval = g_last_present_us != 0 ? now - g_last_present_us : 0;
  g_last_present_us = now;
  std::uint64_t count[kKindCount], total[kKindCount], longest[kKindCount];
  DWORD tid[kKindCount];
  for (int k = 0; k < kKindCount; ++k) {
    count[k] = g_frame[k].count.exchange(0, std::memory_order_relaxed);
    total[k] = g_frame[k].total_us.exchange(0, std::memory_order_relaxed);
    longest[k] = g_frame[k].max_us.exchange(0, std::memory_order_relaxed);
    tid[k] = g_frame[k].max_tid.exchange(0, std::memory_order_relaxed);
    g_win_count[k] += count[k];
    g_win_us[k] += total[k];
    if (longest[k] > g_win_max[k]) g_win_max[k] = longest[k];
  }
  if (!gameplay) return;
  ++g_window_frames;
  if (interval > g_window_worst_us) g_window_worst_us = interval;
  if (interval > 40000) {
    ++g_window_long;
    char line[1400];
    int n = snprintf(line, sizeof(line), "HITCH frame %.1f ms (present thread %lu):", interval / 1000.0, GetCurrentThreadId());
    for (int k = 0; k < kKindCount && n > 0 && n < static_cast<int>(sizeof(line)); ++k) {
      if (count[k] == 0) continue;
      n += snprintf(line + n, sizeof(line) - n, " %s %llu/%.1f/%.1f@%lu", kKindNames[k], static_cast<unsigned long long>(count[k]),
                    total[k] / 1000.0, longest[k] / 1000.0, tid[k]);
    }
    logf("%s", line);
  }
  if (g_window_start_us == 0) g_window_start_us = now;
  if (now - g_window_start_us >= 10000000) {
    char line[1400];
    int n = snprintf(line, sizeof(line), "summary 10 s: %llu gameplay frames, %llu > 40 ms, worst %.1f ms | per kind count/total ms/longest ms:",
                     static_cast<unsigned long long>(g_window_frames), static_cast<unsigned long long>(g_window_long),
                     g_window_worst_us / 1000.0);
    for (int k = 0; k < kKindCount && n > 0 && n < static_cast<int>(sizeof(line)); ++k) {
      if (g_win_count[k] == 0) continue;
      n += snprintf(line + n, sizeof(line) - n, " %s %llu/%.1f/%.1f", kKindNames[k], static_cast<unsigned long long>(g_win_count[k]),
                    g_win_us[k] / 1000.0, g_win_max[k] / 1000.0);
    }
    logf("%s", line);
    g_window_start_us = now;
    g_window_frames = g_window_long = g_window_worst_us = 0;
    for (int k = 0; k < kKindCount; ++k) g_win_count[k] = g_win_us[k] = g_win_max[k] = 0;
  }
}

}  // namespace hitch_probe
