#include "eye_probe.hpp"

#include <windows.h>

#include <MinHook.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

// Evidence: research/STEREO_ROUTE.md "Snowdrop's built-in Stereo3D path" and ADDRESS_REGISTRY.md LEAD-01.
// The stepper at afop.exe+0x5A7530 is:
//   sub rsp,28h ; call getter ; mov edx,[rax+1BFA0h] ; test edx,edx ; je L0 ; cmp edx,1 ; jne Lret
//   mov dword [rax+1BFA0h],2 ; add rsp,28h ; ret
//   L0: mov dword [rax+1BFA0h],1 ; Lret: add rsp,28h ; ret
// rax still holds the view object on return, so the detour can read the eye field after the step.
// This module never writes game memory except the MinHook trampoline/jump that MinHook installs.

namespace {

constexpr std::uintptr_t kStepperRva = 0x5A7530;
constexpr std::uint32_t kEyeFieldOffset = 0x1BFA0;
// Exact bytes of the stepper in afop.exe SHA-256 98588DA283E8DCF4BB8BA420AC220DC70892222329DFCE115EB1DD6D42DB1E91.
// No absolute addresses inside (the call is rel32), so the signature is ASLR-independent.
constexpr unsigned char kSignature[] = {
    0x48, 0x83, 0xEC, 0x28, 0xE8, 0xD7, 0x61, 0xFA, 0xFF, 0x8B, 0x90, 0xA0, 0xBF, 0x01, 0x00, 0x85, 0xD2, 0x74,
    0x14, 0x83, 0xFA, 0x01, 0x75, 0x19, 0xC7, 0x80, 0xA0, 0xBF, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x48, 0x83,
    0xC4, 0x28, 0xC3, 0xC7, 0x80, 0xA0, 0xBF, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x48, 0x83, 0xC4, 0x28, 0xC3};

// View-data getter (ADDRESS_REGISTRY LEAD-02) at afop.exe+0x266B10. Exact bytes, ASLR-independent
// (its only rip-relative operand points inside the same image). It returns
//   [owner+0x9EB0] + frameIndex * 0x1C070, frameIndex = [owner+0x9EC8] (render thread) or [owner+0x9EC4].
// Each frame object holds three eye slots at +0xEEF0 (stride 0x4590: mono, left, right) and the eye field
// at +0x1BFA0 (= 0xEEF0 + 3 * 0x4590). The detour only remembers which owners call it.
constexpr std::uintptr_t kGetterRva = 0x266B10;
constexpr unsigned char kGetterSignature[] = {
    0x8B, 0x15, 0x2A, 0x1C, 0x9E, 0x09, 0x65, 0x48, 0x8B, 0x04, 0x25, 0x58, 0x00, 0x00, 0x00, 0x41, 0xB8, 0x8A, 0x05,
    0x00, 0x00, 0x48, 0x8B, 0x04, 0xD0, 0x41, 0x80, 0x3C, 0x00, 0x00, 0x74, 0x15, 0x8B, 0x81, 0xC8, 0x9E, 0x00, 0x00,
    0x48, 0x69, 0xC0, 0x70, 0xC0, 0x01, 0x00, 0x48, 0x03, 0x81, 0xB0, 0x9E, 0x00, 0x00, 0xC3, 0x8B, 0x81, 0xC4, 0x9E,
    0x00, 0x00, 0x48, 0x69, 0xC0, 0x70, 0xC0, 0x01, 0x00, 0x48, 0x03, 0x81, 0xB0, 0x9E, 0x00, 0x00, 0xC3};
constexpr std::uint32_t kOwnerFramesPtr = 0x9EB0;
constexpr std::uint32_t kOwnerIndexMain = 0x9EC4;
constexpr std::uint32_t kOwnerIndexRender = 0x9EC8;
constexpr std::size_t kFrameStride = 0x1C070;
constexpr std::uint32_t kSlotsOffset = 0xEEF0;
constexpr std::size_t kSlotStride = 0x4590;
constexpr int kSlotCount = 3;
constexpr int kOwnerMax = 8;
constexpr int kFramesToRead = 3;
constexpr int kSlotDumpEverySeconds = 10;
constexpr int kSlotDumpMax = 6;

using GetterFn = void*(__fastcall*)(void*);
GetterFn g_getter_original = nullptr;
std::atomic<bool> g_getter_installed{false};
std::atomic<void*> g_owner_key[kOwnerMax] = {};
std::atomic<std::uint64_t> g_owner_calls[kOwnerMax] = {};
std::atomic<std::uint64_t> g_owner_overflow{0};
int g_slot_dumps = 0;
int g_log_seconds = 0;
// One frame object's slot region (3 slots + eye field), copied under SEH on the Present thread.
unsigned char g_slot_copy[kSlotStride * kSlotCount + 16];

using StepperFn = void*(__fastcall*)(void*, void*, void*, void*);
StepperFn g_original = nullptr;

std::atomic<bool> g_installed{false};
std::atomic<bool> g_install_attempted{false};
std::atomic<std::uint64_t> g_calls{0};
std::atomic<std::uint64_t> g_eye_hist[4] = {};  // eye value after the step: 0,1,2,other/unreadable
std::atomic<void*> g_last_object{nullptr};
std::atomic<std::uint64_t> g_presents{0};
std::atomic<ULONGLONG> g_last_log_tick{0};
std::uint64_t g_frame_prev_calls = 0;  // per-present comparison
std::uint64_t g_prev_calls = 0;        // per-log-line comparison
std::uint64_t g_prev_presents = 0;
std::uint64_t g_frames_with_calls = 0;

std::mutex g_log_mutex;
std::wstring g_log_path;

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
  char buffer[512];
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

int read_eye_seh(const void* object) {
  __try {
    return *reinterpret_cast<const volatile int*>(static_cast<const unsigned char*>(object) + kEyeFieldOffset);
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return -1;
  }
}

void* __fastcall stepper_detour(void* a, void* b, void* c, void* d) {
  void* object = g_original(a, b, c, d);
  g_calls.fetch_add(1, std::memory_order_relaxed);
  if (object != nullptr) {
    g_last_object.store(object, std::memory_order_relaxed);
    const int eye = read_eye_seh(object);
    const int bucket = (eye >= 0 && eye <= 2) ? eye : 3;
    g_eye_hist[bucket].fetch_add(1, std::memory_order_relaxed);
  } else {
    g_eye_hist[3].fetch_add(1, std::memory_order_relaxed);
  }
  return object;
}

void* __fastcall getter_detour(void* owner) {
  if (owner != nullptr) {
    for (int i = 0; i < kOwnerMax; ++i) {
      void* key = g_owner_key[i].load(std::memory_order_relaxed);
      if (key == owner) {
        g_owner_calls[i].fetch_add(1, std::memory_order_relaxed);
        break;
      }
      if (key == nullptr) {
        void* expected = nullptr;
        if (g_owner_key[i].compare_exchange_strong(expected, owner) || expected == owner) {
          g_owner_calls[i].fetch_add(1, std::memory_order_relaxed);
          break;
        }
      }
      if (i == kOwnerMax - 1) {
        g_owner_overflow.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
  return g_getter_original(owner);
}

bool read_u64_seh(const void* p, std::uint64_t* out) {
  __try {
    *out = *reinterpret_cast<const volatile std::uint64_t*>(p);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

bool read_u32_seh(const void* p, std::uint32_t* out) {
  __try {
    *out = *reinterpret_cast<const volatile std::uint32_t*>(p);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

bool finite16(const float* m) {
  for (int i = 0; i < 16; ++i) {
    if (!(m[i] == m[i]) || m[i] > 1e30f || m[i] < -1e30f) {
      return false;
    }
  }
  return true;
}

// Row-vector rigid transform: orthonormal 3x3, last column (0,0,0,1).
bool is_rigid_row(const float* m) {
  if (!finite16(m)) {
    return false;
  }
  if (std::fabs(m[3]) > 1e-3f || std::fabs(m[7]) > 1e-3f || std::fabs(m[11]) > 1e-3f || std::fabs(m[15] - 1.f) > 1e-3f) {
    return false;
  }
  for (int r = 0; r < 3; ++r) {
    const float len = m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2];
    if (len < 0.95f || len > 1.05f) {
      return false;
    }
  }
  const float d01 = m[0] * m[4] + m[1] * m[5] + m[2] * m[6];
  const float d02 = m[0] * m[8] + m[1] * m[9] + m[2] * m[10];
  const float d12 = m[4] * m[8] + m[5] * m[9] + m[6] * m[10];
  return std::fabs(d01) < 0.05f && std::fabs(d02) < 0.05f && std::fabs(d12) < 0.05f;
}

// Row-vector perspective: m11 = +-1, m15 = 0, plausible y scale.
bool is_perspective_row(const float* m, float* fov) {
  if (!finite16(m)) {
    return false;
  }
  if (std::fabs(std::fabs(m[11]) - 1.f) > 0.02f || std::fabs(m[15]) > 0.02f || m[5] < 0.05f || m[5] > 20.f || m[0] < 0.05f) {
    return false;
  }
  *fov = 2.f * std::atan(1.f / m[5]) * 57.29578f;
  return true;
}

// Logs what each of the three eye slots of one frame object contains. Read-only.
void analyze_frame_object(int owner_index, int frame, const unsigned char* frame_object) {
  const std::size_t bytes = kSlotStride * kSlotCount + 4;
  if (!read_bytes_seh(frame_object + kSlotsOffset, g_slot_copy, bytes)) {
    logf("slots owner#%d frame=%d obj=%p UNREADABLE", owner_index, frame, frame_object);
    return;
  }
  int eye = 0;
  std::memcpy(&eye, g_slot_copy + kSlotStride * kSlotCount, sizeof(eye));
  logf("slots owner#%d frame=%d obj=%p eye_field=%d", owner_index, frame, frame_object, eye);
  for (int slot = 0; slot < kSlotCount; ++slot) {
    const unsigned char* base = g_slot_copy + kSlotStride * slot;
    std::size_t nonzero = 0;
    for (std::size_t i = 0; i + 4 <= kSlotStride; i += 4) {
      std::uint32_t v = 0;
      std::memcpy(&v, base + i, 4);
      if (v != 0) {
        ++nonzero;
      }
    }
    char line[900] = {};
    int used = snprintf(line, sizeof(line), "  slot%d nonzero_dwords=%llu/%llu", slot,
                        static_cast<unsigned long long>(nonzero), static_cast<unsigned long long>(kSlotStride / 4));
    int found = 0;
    for (std::size_t off = 0; off + 64 <= kSlotStride && found < 10 && used > 0 && used < static_cast<int>(sizeof(line)) - 80; off += 16) {
      float m[16];
      std::memcpy(m, base + off, sizeof(m));
      float fov = 0;
      if (is_rigid_row(m)) {
        const float tm = std::sqrt(m[12] * m[12] + m[13] * m[13] + m[14] * m[14]);
        if (tm > 1.f) {
          used += snprintf(line + used, sizeof(line) - used, " R@0x%llx(t=%.2f,%.2f,%.2f|%.1f)",
                           static_cast<unsigned long long>(off), m[12], m[13], m[14], tm);
          ++found;
          off += 48;
        }
      } else if (is_perspective_row(m, &fov)) {
        used += snprintf(line + used, sizeof(line) - used, " P@0x%llx(fov=%.2f sk=%.5f,%.5f)",
                         static_cast<unsigned long long>(off), fov, m[8], m[9]);
        ++found;
        off += 48;
      }
    }
    logf("%s", line);
  }
}

void dump_slots() {
  for (int i = 0; i < kOwnerMax; ++i) {
    void* owner = g_owner_key[i].load(std::memory_order_relaxed);
    if (owner == nullptr) {
      continue;
    }
    std::uint64_t frames = 0;
    std::uint32_t idx_main = 0;
    std::uint32_t idx_render = 0;
    const unsigned char* o = static_cast<const unsigned char*>(owner);
    if (!read_u64_seh(o + kOwnerFramesPtr, &frames) || !read_u32_seh(o + kOwnerIndexMain, &idx_main) ||
        !read_u32_seh(o + kOwnerIndexRender, &idx_render)) {
      logf("owner#%d %p unreadable", i, owner);
      continue;
    }
    logf("owner#%d %p calls=%llu frames=%p idx_main=%u idx_render=%u", i, owner,
         static_cast<unsigned long long>(g_owner_calls[i].load()), reinterpret_cast<void*>(frames), idx_main, idx_render);
    if (frames == 0 || idx_main > 16 || idx_render > 16) {
      continue;
    }
    for (int f = 0; f < kFramesToRead; ++f) {
      analyze_frame_object(i, f, reinterpret_cast<const unsigned char*>(frames) + kFrameStride * static_cast<std::size_t>(f));
    }
  }
}

}  // namespace

namespace eye_probe {

void install() {
  if (g_install_attempted.exchange(true)) {
    return;
  }
  const std::wstring dir = self_dir();
  if (!dir.empty()) {
    g_log_path = dir + L"\\EYE_PROBE.log";
    const std::wstring prev = dir + L"\\EYE_PROBE.prev.log";
    MoveFileExW(g_log_path.c_str(), prev.c_str(), MOVEFILE_REPLACE_EXISTING);
  }
  logf("eye probe build %s %s pid=%lu", __DATE__, __TIME__, GetCurrentProcessId());

  auto* base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
  if (base == nullptr) {
    logf("eye probe SKIP: no main module");
    return;
  }
  unsigned char* target = base + kStepperRva;
  unsigned char actual[sizeof(kSignature)] = {};
  if (!read_bytes_seh(target, actual, sizeof(actual))) {
    logf("eye probe SKIP: target unreadable base=%p target=%p", base, target);
    return;
  }
  if (std::memcmp(actual, kSignature, sizeof(kSignature)) != 0) {
    char hex[3 * sizeof(kSignature) + 1] = {};
    for (std::size_t i = 0; i < sizeof(kSignature); ++i) {
      snprintf(hex + 3 * i, 4, "%02X ", actual[i]);
    }
    logf("eye probe SKIP: signature mismatch at base+0x%llX (different build?) bytes=%s",
         static_cast<unsigned long long>(kStepperRva), hex);
    return;
  }
  const MH_STATUS created = MH_CreateHook(target, reinterpret_cast<void*>(&stepper_detour), reinterpret_cast<void**>(&g_original));
  if (created != MH_OK) {
    logf("eye probe SKIP: MH_CreateHook=%d", static_cast<int>(created));
    return;
  }
  const MH_STATUS enabled = MH_EnableHook(target);
  if (enabled != MH_OK) {
    logf("eye probe SKIP: MH_EnableHook=%d", static_cast<int>(enabled));
    MH_RemoveHook(target);
    return;
  }
  g_installed.store(true);
  logf("eye probe ARMED base=%p stepper=%p (signature matched, read-only detour)", base, target);

  unsigned char* getter = base + kGetterRva;
  unsigned char got[sizeof(kGetterSignature)] = {};
  if (!read_bytes_seh(getter, got, sizeof(got)) || std::memcmp(got, kGetterSignature, sizeof(kGetterSignature)) != 0) {
    logf("slot probe SKIP: getter signature mismatch at base+0x%llX", static_cast<unsigned long long>(kGetterRva));
    return;
  }
  if (MH_CreateHook(getter, reinterpret_cast<void*>(&getter_detour), reinterpret_cast<void**>(&g_getter_original)) != MH_OK) {
    logf("slot probe SKIP: MH_CreateHook failed");
    return;
  }
  if (MH_EnableHook(getter) != MH_OK) {
    logf("slot probe SKIP: MH_EnableHook failed");
    MH_RemoveHook(getter);
    return;
  }
  g_getter_installed.store(true);
  logf("slot probe ARMED getter=%p (signature matched, read-only detour)", getter);
}

void on_present() {
  const std::uint64_t presents = g_presents.fetch_add(1, std::memory_order_relaxed) + 1;
  if (!g_installed.load(std::memory_order_relaxed)) {
    return;
  }
  const std::uint64_t calls = g_calls.load(std::memory_order_relaxed);
  if (calls != g_frame_prev_calls) {
    ++g_frames_with_calls;
  }
  g_frame_prev_calls = calls;
  const ULONGLONG now = GetTickCount64();
  ULONGLONG last = g_last_log_tick.load(std::memory_order_relaxed);
  if (last != 0 && now - last < 1000) {
    return;
  }
  g_last_log_tick.store(now, std::memory_order_relaxed);
  void* object = g_last_object.load(std::memory_order_relaxed);
  const int eye_now = object != nullptr ? read_eye_seh(object) : -2;
  logf("eye presents=%llu dpresents=%llu stepper_calls=%llu dcalls=%llu frames_with_calls=%llu after_step{0=%llu 1=%llu 2=%llu other=%llu} last_obj=%p eye_now=%d",
       static_cast<unsigned long long>(presents),
       static_cast<unsigned long long>(presents - g_prev_presents),
       static_cast<unsigned long long>(calls),
       static_cast<unsigned long long>(calls - g_prev_calls),
       static_cast<unsigned long long>(g_frames_with_calls),
       static_cast<unsigned long long>(g_eye_hist[0].load()),
       static_cast<unsigned long long>(g_eye_hist[1].load()),
       static_cast<unsigned long long>(g_eye_hist[2].load()),
       static_cast<unsigned long long>(g_eye_hist[3].load()),
       object, eye_now);
  g_prev_presents = presents;
  g_prev_calls = calls;

  if (g_getter_installed.load(std::memory_order_relaxed)) {
    ++g_log_seconds;
    if (g_slot_dumps < kSlotDumpMax && (g_log_seconds % kSlotDumpEverySeconds) == 0) {
      ++g_slot_dumps;
      logf("slot dump %d/%d", g_slot_dumps, kSlotDumpMax);
      dump_slots();
    }
  }
}

unsigned long long stepper_calls() {
  return static_cast<unsigned long long>(g_calls.load());
}

}  // namespace eye_probe
