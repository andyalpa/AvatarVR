#include "input_probe.hpp"

#include <windows.h>

#include <MinHook.h>

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

// Evidence: research/INPUT_OWNERSHIP.md "Gamepad backends" ([STATIC], Ghidra on build 98588DA2...).
//   0x1432B62B0  device enumeration (rcx = device manager: list +0x850, count +0x858)
//   0x149A83260  backend mode (int): XInput pads enumerated when mode < 2 or == 2, WGI pads when < 2 or == 5
//   0x149A831C8  Windows.Gaming.Input state (0x88 bytes): +0x8..+0x20 four IGamepad slots, +0x80 init ok
//   0x149A830B8  XInput function table (Enable, GetState, SetState, GetCapabilities); null when not loaded
// Device kinds by vtable: HID pad 0x1460C5E18, XInput slot 0x1460C6080, WGI slot 0x1460C5F90, keyboard/mouse
// 0x1460C5CE0; pad slot index at device +0x20. This module writes no game memory except MinHook's own jump.

namespace {

constexpr std::uintptr_t kEnumRva = 0x32B62B0;
// afop.exe 98588DA2...: push r12; push r13; sub rsp,0A8h; mov edx,[rip+X]; mov r13,rcx (tools/re/make_sig.py,
// unique in readable code). The rip displacement is masked.
constexpr unsigned char kEnumSig[] = {0x41, 0x54, 0x41, 0x55, 0x48, 0x81, 0xEC, 0xA8, 0x00, 0x00,
                                      0x00, 0x8B, 0x15, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x8B, 0xE9};
constexpr unsigned char kEnumMask[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1};

constexpr std::uintptr_t kModeRva = 0x9A83260;
constexpr std::uintptr_t kWgiRva = 0x9A831C8;
constexpr std::uintptr_t kXinputTableRva = 0x9A830B8;
constexpr std::uint32_t kListOff = 0x850;
constexpr std::uint32_t kCountOff = 0x858;
constexpr std::uint32_t kPendingCountOff = 0x900;
constexpr std::uint32_t kSlotOff = 0x20;
constexpr int kMaxDevices = 16;

struct Kind {
  std::uintptr_t vtable_rva;
  const char* name;
  bool pad;
};
constexpr Kind kKinds[] = {
    {0x60C5E18, "hid_pad", true},
    {0x60C6080, "xinput_pad", true},
    {0x60C5F90, "wgi_pad", true},
    {0x60C5CE0, "kbm", false},
};

using EnumFn = void(__fastcall*)(void*);
EnumFn g_enum_original = nullptr;
std::atomic<void*> g_manager{nullptr};
std::atomic<std::uint64_t> g_enum_calls{0};
std::atomic<bool> g_install_attempted{false};
unsigned char* g_base = nullptr;

std::mutex g_log_mutex;
std::wstring g_log_path;
ULONGLONG g_last_tick = 0;
ULONGLONG g_last_full = 0;
std::string g_last_state;

void write_line(const char* text) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty()) {
    return;
  }
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, text, static_cast<DWORD>(std::strlen(text)), &written, nullptr);
  WriteFile(file, "\r\n", 2, &written, nullptr);
  CloseHandle(file);
}

void logf(const char* fmt, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  write_line(buffer);
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

// SEH-guarded reads. Plain C types only in these frames.
bool read_bytes_seh(const void* src, void* dst, std::size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

template <typename T>
bool read(const void* p, T* out) {
  return p != nullptr && read_bytes_seh(p, out, sizeof(T));
}

void __fastcall enum_detour(void* manager) {
  g_manager.store(manager, std::memory_order_relaxed);
  g_enum_calls.fetch_add(1, std::memory_order_relaxed);
  g_enum_original(manager);
}

const Kind* kind_of(std::uintptr_t vtable) {
  for (const Kind& k : kKinds) {
    if (vtable == reinterpret_cast<std::uintptr_t>(g_base) + k.vtable_rva) {
      return &k;
    }
  }
  return nullptr;
}

void append(std::string* s, const char* fmt, ...) {
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  *s += buffer;
}

// One line describing the current state; addresses only where they identify something, so the line changes
// only when the state does.
std::string snapshot() {
  std::string s;
  int mode = -1;
  append(&s, "mode=%d", read(g_base + kModeRva, &mode) ? mode : -999);

  unsigned char* wgi = nullptr;
  if (read(g_base + kWgiRva, &wgi) && wgi != nullptr) {
    unsigned char ok = 0xFF;
    read(wgi + 0x80, &ok);
    int mask = 0;
    for (int i = 0; i < 4; ++i) {
      void* pad = nullptr;
      if (read(wgi + 0x8 + 8 * i, &pad) && pad != nullptr) {
        mask |= 1 << i;
      }
    }
    append(&s, " wgi=on init_ok=%d pads=0x%X", ok, mask);
  } else {
    s += " wgi=null";
  }

  void* xtable = nullptr;
  s += (read(g_base + kXinputTableRva, &xtable) && xtable != nullptr) ? " xinput_table=set" : " xinput_table=null";

  for (const wchar_t* dll : {L"xinput1_4.dll", L"xinput9_1_0.dll", L"xinput1_3.dll", L"Windows.Gaming.Input.dll", L"hid.dll"}) {
    if (GetModuleHandleW(dll) != nullptr) {
      char name[64] = {};
      WideCharToMultiByte(CP_UTF8, 0, dll, -1, name, sizeof(name) - 1, nullptr, nullptr);
      append(&s, " +%s", name);
    }
  }

  unsigned char* m = static_cast<unsigned char*>(g_manager.load(std::memory_order_relaxed));
  if (m == nullptr) {
    s += " manager=not_seen";
    return s;
  }
  std::uint32_t count = 0;
  std::uint32_t pending = 0;
  unsigned char** list = nullptr;
  if (!read(m + kCountOff, &count) || !read(m + kListOff, &list)) {
    s += " manager=unreadable";
    return s;
  }
  read(m + kPendingCountOff, &pending);
  append(&s, " devices=%u pending=%u [", count, pending);
  for (std::uint32_t i = 0; i < count && i < static_cast<std::uint32_t>(kMaxDevices); ++i) {
    unsigned char* dev = nullptr;
    std::uintptr_t vt = 0;
    if (!read(list + i, &dev) || dev == nullptr || !read(dev, &vt)) {
      s += " ?";
      continue;
    }
    const Kind* k = kind_of(vt);
    if (k == nullptr) {
      append(&s, " vt+0x%llX", static_cast<unsigned long long>(vt - reinterpret_cast<std::uintptr_t>(g_base)));
      continue;
    }
    if (k->pad) {
      std::int32_t slot = -1;
      read(dev + kSlotOff, &slot);
      append(&s, " %s#%d", k->name, slot);
    } else {
      append(&s, " %s", k->name);
    }
  }
  s += " ]";
  return s;
}

bool sig_matches(const unsigned char* target) {
  unsigned char got[sizeof(kEnumSig)] = {};
  if (!read_bytes_seh(target, got, sizeof(got))) {
    return false;
  }
  for (std::size_t i = 0; i < sizeof(kEnumSig); ++i) {
    if (kEnumMask[i] && got[i] != kEnumSig[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace input_probe {

void install() {
  if (g_install_attempted.exchange(true)) {
    return;
  }
  const std::wstring dir = self_dir();
  if (!dir.empty()) {
    g_log_path = dir + L"\\INPUT_PROBE.log";
    const std::wstring prev = dir + L"\\INPUT_PROBE.prev.log";
    MoveFileExW(g_log_path.c_str(), prev.c_str(), MOVEFILE_REPLACE_EXISTING);
  }
  logf("input probe build %s %s pid=%lu", __DATE__, __TIME__, GetCurrentProcessId());
  g_base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
  if (g_base == nullptr) {
    logf("input probe SKIP: no main module");
    return;
  }
  logf("at install: %s", snapshot().c_str());
  unsigned char* target = g_base + kEnumRva;
  if (!sig_matches(target)) {
    logf("enum hook SKIP: guard mismatch at base+0x%llX (different build?); globals are still logged",
         static_cast<unsigned long long>(kEnumRva));
    return;
  }
  if (MH_CreateHook(target, reinterpret_cast<void*>(&enum_detour), reinterpret_cast<void**>(&g_enum_original)) != MH_OK ||
      MH_EnableHook(target) != MH_OK) {
    logf("enum hook SKIP: MinHook failed");
    return;
  }
  logf("enum hook armed at base+0x%llX", static_cast<unsigned long long>(kEnumRva));
}

void on_present() {
  if (g_base == nullptr) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (now - g_last_tick < 1000) {
    return;
  }
  g_last_tick = now;
  std::string state = snapshot();
  const std::uint64_t calls = g_enum_calls.load(std::memory_order_relaxed);
  if (state != g_last_state || now - g_last_full >= 10000) {
    logf("t=%llus enum_calls=%llu %s", static_cast<unsigned long long>(now / 1000),
         static_cast<unsigned long long>(calls), state.c_str());
    g_last_state = std::move(state);
    g_last_full = now;
  }
}

}  // namespace input_probe
