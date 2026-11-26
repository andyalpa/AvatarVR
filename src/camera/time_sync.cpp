#include "../runtime_log.hpp"
#include "time_sync.hpp"

#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

// Exact bytes of the reset handler (build 98588DA2...) up to and including the second call. Its operands
// are rip-relative within the image, so the signature is ASLR-independent.
constexpr std::uintptr_t kHandlerRva = 0xD896E0;
constexpr unsigned char kHandlerSig[] = {
    0x40, 0x53, 0x48, 0x83, 0xEC, 0x50, 0xF3, 0x0F, 0x10, 0x0D, 0x9A, 0xA9, 0xA3, 0x04, 0xC6, 0x41, 0x2A,
    0x00, 0x48, 0x8B, 0x0D, 0x67, 0x9A, 0xCF, 0x08, 0xE8, 0x92, 0x81, 0x57, 0x02, 0x48, 0x8B, 0x0D, 0x5B,
    0x9A, 0xCF, 0x08, 0x33, 0xD2, 0xE8, 0xE4, 0x20, 0x56, 0x02};
constexpr unsigned char kSetterSig[] = {0xF3, 0x0F, 0x11, 0x49, 0x4C, 0xC3};  // movss [rcx+0x4C], xmm1 ; ret
constexpr std::uint32_t kSpeedOffset = 0x4C;
constexpr std::uint32_t kFlagOffset = 0x48;

enum class Mode { Off, Read, Speed, Pair, PairInv };

std::wstring g_control_path;
std::wstring g_log_path;
void** g_slot = nullptr;  // address of the global object pointer
Mode g_mode = Mode::Off;
float g_speed_arg = 1.f;
bool g_have_saved = false;
float g_saved = 1.f;
std::uint64_t g_presents = 0;
std::uint64_t g_writes = 0;

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

// SEH-guarded memory helpers. Plain C types only in these frames.
bool read_bytes(const void* src, void* dst, std::size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

unsigned char* speed_object() {
  if (g_slot == nullptr) {
    return nullptr;
  }
  void* object = nullptr;
  if (!read_bytes(g_slot, &object, sizeof(object))) {
    return nullptr;
  }
  return static_cast<unsigned char*>(object);
}

bool read_speed(float* speed, unsigned char* flag) {
  unsigned char* object = speed_object();
  if (object == nullptr) {
    return false;
  }
  return read_bytes(object + kSpeedOffset, speed, sizeof(*speed)) && read_bytes(object + kFlagOffset, flag, sizeof(*flag));
}

bool write_speed(float value) {
  unsigned char* object = speed_object();
  if (object == nullptr) {
    return false;
  }
  __try {
    *reinterpret_cast<volatile float*>(object + kSpeedOffset) = value;
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

void restore_if_needed() {
  if (g_have_saved) {
    write_speed(g_saved);
    log_line("time sync restored game speed %.4f", g_saved);
    g_have_saved = false;
  }
}

}  // namespace

namespace time_sync {

bool read(float* speed, unsigned char* flag) { return read_speed(speed, flag); }

void install() {
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_control_path = dir + L"\\AvatarVR_time.txt";
  g_log_path = dir + L"\\TIME_SYNC.log";
  log_line("time sync build %s %s pid=%lu control=AvatarVR_time.txt (missing file = off)", __DATE__, __TIME__, GetCurrentProcessId());

  auto* base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
  if (base == nullptr) {
    return;
  }
  unsigned char* handler = base + kHandlerRva;
  unsigned char got[sizeof(kHandlerSig)] = {};
  if (!read_bytes(handler, got, sizeof(got)) || std::memcmp(got, kHandlerSig, sizeof(kHandlerSig)) != 0) {
    log_line("time sync DISABLED: handler signature mismatch at base+0x%llX", static_cast<unsigned long long>(kHandlerRva));
    return;
  }
  // mov rcx, [rip+disp32] at handler+0x12; call rel32 at handler+0x19 must reach the setter.
  std::int32_t slot_disp = 0;
  std::int32_t call_disp = 0;
  std::memcpy(&slot_disp, handler + 0x12 + 3, 4);
  std::memcpy(&call_disp, handler + 0x19 + 1, 4);
  unsigned char* slot = handler + 0x12 + 7 + slot_disp;
  unsigned char* setter = handler + 0x19 + 5 + call_disp;
  unsigned char setter_bytes[sizeof(kSetterSig)] = {};
  if (!read_bytes(setter, setter_bytes, sizeof(setter_bytes)) || std::memcmp(setter_bytes, kSetterSig, sizeof(kSetterSig)) != 0) {
    log_line("time sync DISABLED: setter signature mismatch at %p", setter);
    return;
  }
  g_slot = reinterpret_cast<void**>(slot);
  log_line("time sync ARMED slot=%p (base+0x%llX) setter=%p", slot,
           static_cast<unsigned long long>(slot - base), setter);
}

void poll() {
  Mode mode = Mode::Off;
  float arg = 1.f;
  char text[64] = {};
  HANDLE file = g_control_path.empty() ? INVALID_HANDLE_VALUE
                                       : CreateFileW(g_control_path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file != INVALID_HANDLE_VALUE) {
    DWORD read = 0;
    ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
    CloseHandle(file);
    text[read < sizeof(text) ? read : sizeof(text) - 1] = '\0';
    if (std::strncmp(text, "read", 4) == 0) {
      mode = Mode::Read;
    } else if (std::strncmp(text, "pairinv", 7) == 0) {
      mode = Mode::PairInv;
    } else if (std::strncmp(text, "pair", 4) == 0) {
      mode = Mode::Pair;
    } else if (std::strncmp(text, "speed", 5) == 0) {
      const float value = static_cast<float>(std::atof(text + 5));
      if (std::isfinite(value) && value >= 0.05f && value <= 4.f) {
        mode = Mode::Speed;
        arg = value;
      }
    }
  }
  if (g_slot == nullptr) {
    mode = Mode::Off;
  }
  if (mode != g_mode) {
    if (mode == Mode::Off || mode == Mode::Read) {
      restore_if_needed();
    }
    log_line("time sync MODE -> %s arg=%.3f", mode == Mode::Off ? "off" : mode == Mode::Read ? "read" : mode == Mode::Speed ? "speed" : mode == Mode::Pair ? "pair" : "pairinv", arg);
  }
  g_mode = mode;
  g_speed_arg = arg;
  float speed = 0.f;
  unsigned char flag = 0;
  const bool ok = read_speed(&speed, &flag);
  log_line("time sync mode=%s speed=%s%.4f flag=%u saved=%.4f writes=%llu presents=%llu",
           mode == Mode::Off ? "off" : mode == Mode::Read ? "read" : mode == Mode::Speed ? "speed" : mode == Mode::Pair ? "pair" : "pairinv",
           ok ? "" : "UNREADABLE ", speed, static_cast<unsigned>(flag), g_saved,
           static_cast<unsigned long long>(g_writes), static_cast<unsigned long long>(g_presents));
}

void on_present() {
  ++g_presents;
  if (g_mode != Mode::Speed && g_mode != Mode::Pair && g_mode != Mode::PairInv) {
    return;
  }
  if (!g_have_saved) {
    float speed = 0.f;
    unsigned char flag = 0;
    if (!read_speed(&speed, &flag) || !(speed > 0.f && speed < 10.f)) {
      return;  // only take over from a sane running value
    }
    g_saved = speed;
    g_have_saved = true;
  }
  float target = g_saved;
  if (g_mode == Mode::Speed) {
    target = g_speed_arg;
  } else {
    const bool freeze = ((g_presents & 1) != 0) != (g_mode == Mode::PairInv);
    target = freeze ? 0.f : 2.f * g_saved;
  }
  if (write_speed(target)) {
    ++g_writes;
  }
}

}  // namespace time_sync
