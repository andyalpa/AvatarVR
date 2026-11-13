#include "resource_names.hpp"

#include <windows.h>

#include <MinHook.h>

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cwchar>
#include <map>
#include <mutex>
#include <string>

namespace {

// ID3D12Object vtable: 0 QueryInterface, 1 AddRef, 2 Release, 3 GetPrivateData, 4 SetPrivateData,
// 5 SetPrivateDataInterface, 6 SetName.
constexpr int kSetPrivateData = 4;
constexpr int kSetName = 6;
// {429b8c22-9188-4b0c-8742-acb0bf85c200} WKPDID_D3DDebugObjectName (char*)
constexpr GUID kDebugName = {0x429b8c22, 0x9188, 0x4b0c, {0x87, 0x42, 0xac, 0xb0, 0xbf, 0x85, 0xc2, 0x00}};
// {4cca5fd8-921f-42c8-8566-70caf2a9c3c4} WKPDID_D3DDebugObjectNameW (wchar_t*)
constexpr GUID kDebugNameW = {0x4cca5fd8, 0x921f, 0x42c8, {0x85, 0x66, 0x70, 0xca, 0xf2, 0xa9, 0xc3, 0xc4}};

using SetNameFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Object*, LPCWSTR);
using SetPrivateDataFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Object*, REFGUID, UINT, const void*);
SetNameFn g_set_name = nullptr;
SetPrivateDataFn g_set_private_data = nullptr;
std::atomic<bool> g_armed{false};

std::wstring g_log_path;
std::mutex g_mutex;  // guards everything below
struct Entry {
  ID3D12Resource* resource = nullptr;  // not owned; may dangle once the engine frees it
  uint64_t renames = 0;
};
std::map<std::wstring, Entry> g_names;
uint64_t g_name_calls = 0;
uint64_t g_logged = 0;
constexpr uint64_t kMaxLogged = 4000;

void log_line(const char* fmt, ...) {
  char buffer[768];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  if (g_log_path.empty()) {
    return;
  }
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, buffer, static_cast<DWORD>(std::strlen(buffer)), &written, nullptr);
  WriteFile(file, "\r\n", 2, &written, nullptr);
  CloseHandle(file);
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

void record(ID3D12Object* object, const std::wstring& name) {
  if (object == nullptr || name.empty()) {
    return;
  }
  ID3D12Resource* resource = nullptr;
  if (FAILED(object->QueryInterface(IID_PPV_ARGS(&resource))) || resource == nullptr) {
    return;  // a heap, PSO, queue, ... -- not interesting here
  }
  const D3D12_RESOURCE_DESC desc = resource->GetDesc();
  resource->Release();  // we keep a weak pointer only
  std::lock_guard<std::mutex> lock(g_mutex);
  ++g_name_calls;
  Entry& entry = g_names[name];
  const bool first = entry.renames == 0;
  entry.resource = resource;
  ++entry.renames;
  if (!first || desc.Dimension == D3D12_RESOURCE_DIMENSION_BUFFER || g_logged >= kMaxLogged) {
    return;
  }
  ++g_logged;
  char narrow[256] = {};
  WideCharToMultiByte(CP_UTF8, 0, name.c_str(), -1, narrow, sizeof(narrow) - 1, nullptr, nullptr);
  log_line("name \"%s\" res=%p dim=%d %llux%u depth=%u mips=%u fmt=%d flags=0x%X samples=%u",
           narrow, resource, static_cast<int>(desc.Dimension), static_cast<unsigned long long>(desc.Width), desc.Height,
           desc.DepthOrArraySize, desc.MipLevels, static_cast<int>(desc.Format), static_cast<unsigned>(desc.Flags),
           desc.SampleDesc.Count);
}

HRESULT STDMETHODCALLTYPE SetName_hook(ID3D12Object* self, LPCWSTR name) {
  const HRESULT hr = g_set_name(self, name);
  if (SUCCEEDED(hr) && name != nullptr) {
    record(self, std::wstring(name, wcsnlen(name, 256)));
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE SetPrivateData_hook(ID3D12Object* self, REFGUID guid, UINT size, const void* data) {
  const HRESULT hr = g_set_private_data(self, guid, size, data);
  if (SUCCEEDED(hr) && data != nullptr && size > 0) {
    if (IsEqualGUID(guid, kDebugNameW)) {
      const auto* w = static_cast<const wchar_t*>(data);
      record(self, std::wstring(w, wcsnlen(w, size / sizeof(wchar_t))));
    } else if (IsEqualGUID(guid, kDebugName)) {
      const auto* a = static_cast<const char*>(data);
      const size_t n = strnlen(a, size);
      wchar_t wide[256] = {};
      MultiByteToWideChar(CP_UTF8, 0, a, static_cast<int>(n < 255 ? n : 255), wide, 255);
      record(self, wide);
    }
  }
  return hr;
}

bool hook(void* target, void* detour, void** original, const char* what) {
  MH_STATUS s = MH_CreateHook(target, detour, original);
  if (s != MH_OK) {
    log_line("hook %s create status=%d", what, static_cast<int>(s));
    return false;
  }
  s = MH_EnableHook(target);
  if (s != MH_OK) {
    log_line("hook %s enable status=%d", what, static_cast<int>(s));
    return false;
  }
  log_line("hook %s target=%p", what, target);
  return true;
}

}  // namespace

namespace resource_names {

void install() {
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_log_path = dir + L"\\RESOURCE_NAMES.log";
  MoveFileExW(g_log_path.c_str(), (dir + L"\\RESOURCE_NAMES.prev.log").c_str(), MOVEFILE_REPLACE_EXISTING);
  log_line("resource names build %s %s pid=%lu (read-only)", __DATE__, __TIME__, GetCurrentProcessId());
}

std::atomic<void*> g_resource_vtable{nullptr};

void* resource_vtable() {
  return g_resource_vtable.load(std::memory_order_acquire);
}

void arm(ID3D12Resource* any_resource) {
  if (any_resource == nullptr || g_armed.exchange(true)) {
    return;
  }
  void** vtable = *reinterpret_cast<void***>(any_resource);
  g_resource_vtable.store(vtable, std::memory_order_release);
  hook(vtable[kSetName], reinterpret_cast<void*>(&SetName_hook), reinterpret_cast<void**>(&g_set_name), "ID3D12Resource::SetName");
  hook(vtable[kSetPrivateData], reinterpret_cast<void*>(&SetPrivateData_hook), reinterpret_cast<void**>(&g_set_private_data), "ID3D12Resource::SetPrivateData");
}

void poll() {
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last < 10000) {
    return;
  }
  last = now;
  std::lock_guard<std::mutex> lock(g_mutex);
  log_line("summary: distinct names=%zu name calls=%llu textures logged=%llu", g_names.size(),
           static_cast<unsigned long long>(g_name_calls), static_cast<unsigned long long>(g_logged));
}

}  // namespace resource_names
