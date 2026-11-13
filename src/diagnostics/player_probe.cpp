#include "player_probe.hpp"

#include "../game/player.hpp"

#include <windows.h>

#include <MinHook.h>
#include <intrin.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

// Evidence: research/CAMERA_FINDINGS.md "Local player and camera roots from script nodes" ([STATIC], function SDK on
// build 98588DA2..., research/sdk/98588da2/raw/sdk_decomp.c). The script nodes read:
//   root    *(afop.exe+0x9ADB058) -> +0x128 -> +0x28 = local player handle H
//   body    0x1438E3300(H) = *(H+0x3E8) -> +0x10 -> +0x28 -> floats +0x94/+0x98/+0x9C   (Local Player Position)
//   camera  0x1438DEC50(H) = *(*(H+0x418)+0x48) = camera object C; C->vtable[+0x30](C) returns a transform:
//           3 axis rows of 4 floats, position at +0x30                                   (Camera Transform)
// The probe repeats the two tiny getters as reads. It does not call the camera's virtual getter: when that getter is
// a one-instruction accessor (lea/mov rax,[rcx+imm]; ret) the transform address is taken from its bytes; otherwise the
// getter's RVA and first bytes are logged for offline reading and the camera transform stays OPEN, unless the control
// file says "on call": then the getter is called on the Present thread inside an SEH guard (a game call, so opt-in).
// The root global is in .xtext, which the offline analysis never saw, so every read is guarded. This module writes no
// game memory.
// Control (scripts\AvatarVR_player.txt, read once a second): "on" = log, "on call" = log and call the camera getter
// when it is not a plain accessor; anything else or no file = off.
//
// Setter census ("on" + the word "setters"): pass-through MinHook detours on three camera-class methods, installed
// once, only after each guard matches. Each detour calls the original unchanged; when `this` is the local player's
// camera it records the return address (who updates the camera) and, for the angle setter, the three input floats.
// Camera vtable afop+0x6300910 (live 2026-10-10), slots read in Ghidra ([STATIC]):
//   +0x108 (slot 33) 0x143B7DD70  rotation rows +0x20..+0x4C from three angles (float[3], sin/cos of each)
//   +0x090 (slot 18) 0x143B82850  copy a 4x4 transform into +0x20..+0x5C
//   +0x088 (slot 17) 0x143E76BE0  copy transform, FoV-like floats (+0xA8..) and more from another camera object
// The setters are only reached through the vtable (no direct callers in readable code), so callers come from here.
//
// Mode scan ("on" + the word "mode"): the camera manager update 0x144271330 runs the active camera mode object at
// holder+0x40 (holder = H+0x418) before copying into the player camera, so look yaw/pitch lives there. Logs the mode
// object's vtable and id (+0xC) on change, and once a second the float offsets of its first 0x800 bytes that changed
// since the last second, next to the camera's yaw/pitch from the view row (to match them by eye). Reads only.
//
// Look/pose ("on" + the word "look"): first-person camera mode (id 4, vtable afop+0x646F2C8) update 0x14412B530 reads
//   body = 0x1435ADC10(mode+0x10) (the body object above), comp = *(body+0x1A8)
//   look = *(comp+0x30); 0x1428CF050(look) copies 3 floats at look+0x34..+0x3C (look values)
//   pose = 0x143DFEC40(comp, 1) = *(comp+0x500), else *(comp+0x4F8); bone matrices at *pose, 0x40 bytes each,
//          count (int) at pose+0x8, camera bone index (int) at pose+0x7C (0x143DF0D30)
// and copies that bone's matrix into the mode's transform: the first-person camera is a skeleton bone.
// Logs once a second: look floats next to the camera yaw/pitch, pose pointer/count/camera bone index, and the
// camera bone's position next to the camera position. Reads only.
//
// Bones ("on" + the word "bones"): every first-person pose bone's position in camera-bone (bone 0) axes, metres,
// twice a second, to find the arm and hand chains by moving them. Once per pose: the pose header (32 qwords) and
// any ASCII string the header's pointers lead to (bone names, skeleton resource). Reads only.

namespace {

constexpr std::uint32_t kCamGetterSlot = 0x30;
constexpr std::uint32_t kBodyPosOff = 0x94;
constexpr ULONGLONG kSampleMs = 100;

// Setter guards (tools/re/make_sig.py, unique in readable code on 98588DA2...).
constexpr std::uintptr_t kSetAnglesRva = 0x3B7DD70;
constexpr unsigned char kSetAnglesSig[] = {0x4C, 0x8B, 0xDC, 0x48, 0x81, 0xEC, 0xA8, 0x00,
                                           0x00, 0x00, 0xF3, 0x0F, 0x10, 0x1A, 0x0F, 0x57};
constexpr std::uintptr_t kSetMatrixRva = 0x3B82850;
constexpr unsigned char kSetMatrixSig[] = {0x0F, 0x10, 0x02, 0x0F, 0x11, 0x41, 0x20, 0x0F, 0x10, 0x42, 0x10,
                                           0x0F, 0x11, 0x41, 0x30, 0x0F, 0x10, 0x42, 0x20, 0x0F, 0x11, 0x41,
                                           0x40, 0x0F, 0x10, 0x42, 0x30, 0x0F, 0x11, 0x41, 0x50, 0xC3};
constexpr std::uintptr_t kCopyFromRva = 0x3E76BE0;
constexpr unsigned char kCopyFromSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x83,
                                          0xEC, 0x40, 0x48, 0x8B, 0x02, 0x48, 0x8B, 0xF9};
constexpr int kSetters = 3;
constexpr const char* kSetterNames[kSetters] = {"angles", "matrix", "copy"};
constexpr int kCallerSlots = 8;
constexpr std::uint32_t kModeOff = 0x40;
constexpr std::size_t kModeBytes = 0x800;
constexpr int kModeMaxHits = 40;

unsigned char* g_base = nullptr;
std::mutex g_log_mutex;
std::wstring g_log_path;
std::wstring g_control_path;
bool g_started = false;
bool g_on = false;
bool g_call = false;
ULONGLONG g_last_control = 0;
ULONGLONG g_last_sample = 0;
ULONGLONG g_last_rows = 0;
ULONGLONG g_t0 = 0;
std::string g_last_ptrs;
float g_prev_body[3] = {};
bool g_have_prev = false;

// Setter census state. The detours run on game threads; the Present thread reads and resets once a second.
using SetFn = void(__fastcall*)(void*, void*);
SetFn g_set_original[kSetters] = {};
std::atomic<void*> g_player_cam{nullptr};
bool g_setters_wanted = false;
bool g_setters_tried = false;
std::mutex g_census_mutex;
struct Caller {
  std::uintptr_t ret = 0;
  unsigned count = 0;
};
Caller g_callers[kSetters][kCallerSlots];
unsigned g_other_this[kSetters] = {};
float g_last_angles[3] = {};
bool g_have_angles = false;
ULONGLONG g_last_census = 0;

bool g_mode_wanted = false;
bool g_look_wanted = false;
bool g_bones_wanted = false;
ULONGLONG g_last_bones = 0;
unsigned char* g_bones_pose = nullptr;
ULONGLONG g_last_look = 0;
unsigned char* g_mode_obj = nullptr;
float g_mode_prev[kModeBytes / 4] = {};
bool g_mode_have_prev = false;
ULONGLONG g_last_mode = 0;

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

unsigned char* ptr_at(const unsigned char* p, std::uint32_t off) {
  unsigned char* v = nullptr;
  return p != nullptr && read(p + off, &v) ? v : nullptr;
}

unsigned long long rva_of(const void* p) {
  return static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(p) - reinterpret_cast<std::uintptr_t>(g_base));
}

bool in_module(const void* p) {
  const auto a = reinterpret_cast<std::uintptr_t>(p);
  const auto b = reinterpret_cast<std::uintptr_t>(g_base);
  return a >= b && a < b + 0x16000000;
}

// The camera getter's bytes when it is a plain accessor: lea/mov rax,[rcx+imm8|imm32]; ret.
// Returns 1 for lea (transform = C + off), 2 for mov (transform = *(C + off)), 0 otherwise.
int decode_getter(const unsigned char* fn, std::int32_t* off) {
  unsigned char b[8] = {};
  if (!read_bytes_seh(fn, b, sizeof(b)) || b[0] != 0x48 || (b[1] != 0x8D && b[1] != 0x8B)) {
    return 0;
  }
  const int kind = b[1] == 0x8D ? 1 : 2;
  if (b[2] == 0x41 && b[4] == 0xC3) {  // [rcx+imm8]
    *off = static_cast<std::int8_t>(b[3]);
    return kind;
  }
  if (b[2] == 0x81 && b[7] == 0xC3) {  // [rcx+imm32]
    std::memcpy(off, b + 3, 4);
    return kind;
  }
  if (b[2] == 0x01 && b[3] == 0xC3) {  // [rcx]
    *off = 0;
    return kind;
  }
  return 0;
}

using GetterFn = const unsigned char*(__fastcall*)(void*);

// The camera's virtual transform getter, called only with "on call". Plain C types only in this frame.
const unsigned char* call_getter_seh(const unsigned char* fn, void* cam) {
  __try {
    return reinterpret_cast<GetterFn>(const_cast<unsigned char*>(fn))(cam);
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}

void record(int which, void* self, void* arg, void* ret) {
  if (self != g_player_cam.load(std::memory_order_relaxed)) {
    std::lock_guard<std::mutex> lock(g_census_mutex);
    ++g_other_this[which];
    return;
  }
  float angles[3] = {};
  const bool got = which == 0 && read_bytes_seh(arg, angles, sizeof(angles));
  std::lock_guard<std::mutex> lock(g_census_mutex);
  if (got) {
    std::memcpy(g_last_angles, angles, sizeof(angles));
    g_have_angles = true;
  }
  const auto r = reinterpret_cast<std::uintptr_t>(ret);
  for (Caller& c : g_callers[which]) {
    if (c.ret == r || c.ret == 0) {
      c.ret = r;
      ++c.count;
      return;
    }
  }
}

void __fastcall set_angles_detour(void* self, void* arg) {
  record(0, self, arg, _ReturnAddress());
  g_set_original[0](self, arg);
}
void __fastcall set_matrix_detour(void* self, void* arg) {
  record(1, self, arg, _ReturnAddress());
  g_set_original[1](self, arg);
}
void __fastcall copy_from_detour(void* self, void* arg) {
  record(2, self, arg, _ReturnAddress());
  g_set_original[2](self, arg);
}

bool guard_ok(std::uintptr_t rva, const unsigned char* sig, std::size_t n) {
  unsigned char got[64] = {};
  return n <= sizeof(got) && read_bytes_seh(g_base + rva, got, n) && std::memcmp(got, sig, n) == 0;
}

// Once, from the Present thread. MinHook is initialised by dx12.cpp before the first Present.
void install_setters() {
  g_setters_tried = true;
  struct Target {
    std::uintptr_t rva;
    const unsigned char* sig;
    std::size_t n;
    void* detour;
  };
  const Target targets[kSetters] = {
      {kSetAnglesRva, kSetAnglesSig, sizeof(kSetAnglesSig), reinterpret_cast<void*>(&set_angles_detour)},
      {kSetMatrixRva, kSetMatrixSig, sizeof(kSetMatrixSig), reinterpret_cast<void*>(&set_matrix_detour)},
      {kCopyFromRva, kCopyFromSig, sizeof(kCopyFromSig), reinterpret_cast<void*>(&copy_from_detour)},
  };
  for (int i = 0; i < kSetters; ++i) {
    const Target& t = targets[i];
    if (!guard_ok(t.rva, t.sig, t.n)) {
      logf("setters: %s SKIP guard mismatch at +0x%llX", kSetterNames[i], static_cast<unsigned long long>(t.rva));
      continue;
    }
    void* target = g_base + t.rva;
    const MH_STATUS c = MH_CreateHook(target, t.detour, reinterpret_cast<void**>(&g_set_original[i]));
    const MH_STATUS e = c == MH_OK ? MH_EnableHook(target) : c;
    logf("setters: %s +0x%llX hook %s (%d)", kSetterNames[i], static_cast<unsigned long long>(t.rva),
         e == MH_OK ? "on" : "FAILED", static_cast<int>(e));
  }
}

// Once a second: who set the player camera, how often, and the last angles; then reset.
void log_census(ULONGLONG now) {
  g_last_census = now;
  std::string line = "  setters:";
  char b[160];
  std::lock_guard<std::mutex> lock(g_census_mutex);
  for (int i = 0; i < kSetters; ++i) {
    snprintf(b, sizeof(b), " %s[", kSetterNames[i]);
    line += b;
    for (Caller& c : g_callers[i]) {
      if (c.ret != 0) {
        snprintf(b, sizeof(b), " +0x%llX x%u", rva_of(reinterpret_cast<void*>(c.ret)), c.count);
        line += b;
      }
      c = Caller{};
    }
    snprintf(b, sizeof(b), " | other cams %u ]", g_other_this[i]);
    line += b;
    g_other_this[i] = 0;
  }
  if (g_have_angles) {
    snprintf(b, sizeof(b), " angles=(%.4f %.4f %.4f)", g_last_angles[0], g_last_angles[1], g_last_angles[2]);
    line += b;
    g_have_angles = false;
  }
  write_line(line.c_str());
}

struct Sample {
  unsigned char* holder = nullptr;
  unsigned char* handle = nullptr;
  unsigned char* entity = nullptr;
  unsigned char* body_obj = nullptr;
  unsigned char* cam = nullptr;
  const unsigned char* getter = nullptr;
  int getter_kind = 0;
  std::int32_t getter_off = 0;
  bool body_ok = false;
  bool cam_ok = false;
  float body[3] = {};
  float xf[16] = {};
};

Sample take() {
  Sample s;
  const game::Pointers gp = game::pointers();  // the chains live in src/game/player.cpp
  s.handle = gp.handle;
  if (s.handle == nullptr) {
    return s;
  }
  s.entity = gp.entity;
  s.body_obj = gp.body;
  s.body_ok = s.body_obj != nullptr && read_bytes_seh(s.body_obj + kBodyPosOff, s.body, sizeof(s.body));
  s.holder = gp.holder;
  s.cam = gp.camera;
  unsigned char* vt = ptr_at(s.cam, 0);
  if (vt != nullptr && in_module(vt)) {
    s.getter = ptr_at(vt, kCamGetterSlot);
    if (s.getter != nullptr && in_module(s.getter)) {
      s.getter_kind = decode_getter(s.getter, &s.getter_off);
      unsigned char* xf = nullptr;
      if (s.getter_kind == 1) {
        xf = s.cam + s.getter_off;
      } else if (s.getter_kind == 2) {
        xf = ptr_at(s.cam, static_cast<std::uint32_t>(s.getter_off));
      } else if (g_call) {
        xf = const_cast<unsigned char*>(call_getter_seh(s.getter, s.cam));
        s.getter_kind = 3;
      }
      s.cam_ok = xf != nullptr && read_bytes_seh(xf, s.xf, sizeof(s.xf));
    }
  }
  return s;
}

// Identity line: logged at start and whenever an object pointer changes (load, respawn, mount, menus).
std::string pointers(const Sample& s) {
  char b[640];
  unsigned char* cam_vt = ptr_at(s.cam, 0);
  unsigned char code[16] = {};
  char hex[64] = {};
  if (s.getter != nullptr && read_bytes_seh(s.getter, code, sizeof(code))) {
    for (int i = 0; i < 16; ++i) {
      snprintf(hex + 3 * i, sizeof(hex) - 3 * i, "%02X ", code[i]);
    }
  }
  snprintf(b, sizeof(b), "ptrs: handle=%p entity=%p body_obj=%p cam=%p cam_vt=%s+0x%llX getter=+0x%llX kind=%s off=0x%X bytes=%s",
           static_cast<void*>(s.handle), static_cast<void*>(s.entity), static_cast<void*>(s.body_obj),
           static_cast<void*>(s.cam), cam_vt && in_module(cam_vt) ? "afop" : "?", cam_vt ? rva_of(cam_vt) : 0ULL,
           s.getter ? rva_of(s.getter) : 0ULL,
           s.getter_kind == 1 ? "lea" : s.getter_kind == 2 ? "mov" : s.getter_kind == 3 ? "called" : "not_an_accessor(OPEN)",
           static_cast<unsigned>(s.getter_off), hex);
  return b;
}

bool plausible(float v) {
  const float a = std::fabs(v);
  return std::isfinite(v) && (a == 0.0f || (a > 1e-6f && a < 1e6f));
}

// Once a second: mode object identity and the float offsets that moved since the last second.
void scan_mode(const Sample& s, ULONGLONG now) {
  g_last_mode = now;
  unsigned char* mode = ptr_at(s.holder, kModeOff);
  if (mode != g_mode_obj) {
    unsigned char* vt = ptr_at(mode, 0);
    std::int32_t id = -1;
    if (mode != nullptr) {
      read(mode + 0xC, &id);
    }
    logf("t=%.1f mode: obj=%p vt=%s+0x%llX id=%d", (now - g_t0) / 1000.0, static_cast<void*>(mode),
         vt && in_module(vt) ? "afop" : "?", vt ? rva_of(vt) : 0ULL, id);
    g_mode_obj = mode;
    g_mode_have_prev = false;
  }
  if (mode == nullptr) {
    return;
  }
  float cur[kModeBytes / 4] = {};
  if (!read_bytes_seh(mode, cur, sizeof(cur))) {
    return;
  }
  std::string line;
  char b[96];
  if (s.cam_ok) {  // view row (row 2) -> yaw/pitch in degrees, for matching by eye
    const float* r2 = s.xf + 8;
    snprintf(b, sizeof(b), "  mode: cam yaw=%.2f pitch=%.2f |", std::atan2(r2[0], r2[2]) * 57.29578f,
             std::asin(std::fmax(-1.0f, std::fmin(1.0f, r2[1]))) * 57.29578f);
    line = b;
  } else {
    line = "  mode: cam=OPEN |";
  }
  int hits = 0;
  if (g_mode_have_prev) {
    for (std::size_t i = 2; i < kModeBytes / 4 && hits < kModeMaxHits; ++i) {  // skip the vtable qword
      const float a = g_mode_prev[i], c = cur[i];
      if (plausible(a) && plausible(c) && std::fabs(c - a) > 0.002f) {
        snprintf(b, sizeof(b), " +0x%X=%.4f(%+.4f)", static_cast<unsigned>(i * 4), c, c - a);
        line += b;
        ++hits;
      }
    }
  }
  if (g_mode_have_prev) {
    write_line(line.c_str());
  }
  std::memcpy(g_mode_prev, cur, sizeof(cur));
  g_mode_have_prev = true;
}

void scan_look(const Sample& s, ULONGLONG now) {
  g_last_look = now;
  unsigned char* comp = ptr_at(s.body_obj, 0x1A8);
  unsigned char* look = ptr_at(comp, 0x30);
  unsigned char* pose = ptr_at(comp, 0x500);
  const bool pose_fp = pose != nullptr;
  if (pose == nullptr) {
    pose = ptr_at(comp, 0x4F8);
  }
  char line[512];
  int n = snprintf(line, sizeof(line), "  look:");
  float lv[3] = {};
  if (look != nullptr && read_bytes_seh(look + 0x34, lv, sizeof(lv))) {
    n += snprintf(line + n, sizeof(line) - n, " look=(%.4f %.4f %.4f)", lv[0], lv[1], lv[2]);
  } else {
    n += snprintf(line + n, sizeof(line) - n, " look=unreadable comp=%p", static_cast<void*>(comp));
  }
  if (s.cam_ok) {
    const float* r2 = s.xf + 8;
    n += snprintf(line + n, sizeof(line) - n, " cam yaw=%.2f pitch=%.2f (rad %.4f %.4f)",
                  std::atan2(r2[0], r2[2]) * 57.29578f, std::asin(std::fmax(-1.0f, std::fmin(1.0f, r2[1]))) * 57.29578f,
                  std::atan2(r2[0], r2[2]), std::asin(std::fmax(-1.0f, std::fmin(1.0f, r2[1]))));
  }
  std::int32_t count = -1, cam_bone = -1;
  unsigned char* bones = ptr_at(pose, 0);
  if (pose != nullptr) {
    read(pose + 0x8, &count);
    read(pose + 0x7C, &cam_bone);
  }
  n += snprintf(line + n, sizeof(line) - n, " | pose=%s %p bones=%d cam_bone=%d", pose_fp ? "+0x500" : "+0x4F8",
                static_cast<void*>(pose), count, cam_bone);
  float m[16] = {};
  if (bones != nullptr && cam_bone >= 0 && cam_bone < count && count < 4096 &&
      read_bytes_seh(bones + 0x40 * cam_bone, m, sizeof(m))) {
    n += snprintf(line + n, sizeof(line) - n, " cam_bone_pos=(%.3f %.3f %.3f)", m[12], m[13], m[14]);
    if (s.cam_ok) {
      n += snprintf(line + n, sizeof(line) - n, " vs cam (%.3f %.3f %.3f)", s.xf[12], s.xf[13], s.xf[14]);
    }
  }
  write_line(line);
}

// A short printable ASCII string at p, or "".
std::string ascii_at(const unsigned char* p) {
  char buf[48] = {};
  if (p == nullptr || !read_bytes_seh(p, buf, sizeof(buf) - 1)) {
    return {};
  }
  std::size_t n = 0;
  while (n < sizeof(buf) - 1 && buf[n] >= 0x20 && buf[n] < 0x7F) {
    ++n;
  }
  return n >= 4 && (n == sizeof(buf) - 1 || buf[n] == 0) ? std::string(buf, n) : std::string();
}

void dump_pose_header(unsigned char* pose) {
  std::uint64_t q[32] = {};
  if (!read_bytes_seh(pose, q, sizeof(q))) {
    return;
  }
  std::string line = "  pose header:";
  char b[64];
  for (int i = 0; i < 32; ++i) {
    snprintf(b, sizeof(b), " %02X:%llX", i * 8, static_cast<unsigned long long>(q[i]));
    line += b;
  }
  write_line(line.c_str());
  for (int i = 0; i < 32; ++i) {  // strings one or two pointer hops away (name tables are often arrays of pointers)
    auto* p1 = reinterpret_cast<unsigned char*>(q[i]);
    std::string s1 = ascii_at(p1);
    std::string s2;
    unsigned char* p2 = ptr_at(p1, 0);
    if (s1.empty()) {
      s2 = ascii_at(p2);
    }
    if (!s1.empty() || !s2.empty()) {
      logf("  pose +0x%02X -> \"%s\"%s%s", i * 8, s1.empty() ? "*" : s1.c_str(), s2.empty() ? "" : " -> ",
           s2.c_str());
    }
  }
}

void scan_bones(ULONGLONG now) {
  g_last_bones = now;
  const Sample s = take();
  unsigned char* pose = ptr_at(ptr_at(s.body_obj, 0x1A8), 0x500);
  unsigned char* bones = ptr_at(pose, 0);
  std::int32_t count = 0;
  if (pose == nullptr || bones == nullptr || !read(pose + 0x8, &count) || count <= 0 || count > 512) {
    return;
  }
  if (pose != g_bones_pose) {
    g_bones_pose = pose;
    logf("t=%.1f bones: pose=%p count=%d", (now - g_t0) / 1000.0, static_cast<void*>(pose), count);
    dump_pose_header(pose);
  }
  static float m[512 * 16];
  if (!read_bytes_seh(bones, m, sizeof(float) * 16 * count)) {
    return;
  }
  const float* r0 = m;  // bone 0 axes (rows) and position
  const float* r1 = m + 4;
  const float* r2 = m + 8;
  const float* o = m + 12;
  std::string line;
  char b[64];
  snprintf(b, sizeof(b), "  bones t=%.1f:", (now - g_t0) / 1000.0);
  line = b;
  for (int i = 1; i < count; ++i) {
    const float* q = m + 16 * i + 12;
    const float d[3] = {q[0] - o[0], q[1] - o[1], q[2] - o[2]};
    snprintf(b, sizeof(b), " %d:%.2f,%.2f,%.2f", i, d[0] * r0[0] + d[1] * r0[1] + d[2] * r0[2],
             d[0] * r1[0] + d[1] * r1[1] + d[2] * r1[2], d[0] * r2[0] + d[1] * r2[1] + d[2] * r2[2]);
    line += b;
  }
  write_line(line.c_str());
}

void sample(ULONGLONG now) {
  const Sample s = take();
  if (s.handle == nullptr) {
    if (g_last_ptrs != "none") {
      logf("t=%.1f no local player (root or handle null: menus / loading)", (now - g_t0) / 1000.0);
      g_last_ptrs = "none";
      g_have_prev = false;
    }
    return;
  }
  g_player_cam.store(s.cam, std::memory_order_relaxed);
  if (g_setters_wanted && !g_setters_tried && s.cam != nullptr) {
    install_setters();
  }
  if (g_setters_tried && now - g_last_census >= 1000) {
    log_census(now);
  }
  if (g_mode_wanted && now - g_last_mode >= 1000) {
    scan_mode(s, now);
  }
  if (g_look_wanted && now - g_last_look >= 1000) {
    scan_look(s, now);
  }
  if (g_bones_wanted && now - g_last_bones >= 500) {
    scan_bones(now);
  }
  const std::string ptrs = pointers(s);
  if (ptrs != g_last_ptrs) {
    logf("t=%.1f %s", (now - g_t0) / 1000.0, ptrs.c_str());
    g_last_ptrs = ptrs;
    g_have_prev = false;
  }
  char line[512];
  int n = snprintf(line, sizeof(line), "t=%.1f", (now - g_t0) / 1000.0);
  if (s.body_ok) {
    n += snprintf(line + n, sizeof(line) - n, " body=(%.3f %.3f %.3f)", s.body[0], s.body[1], s.body[2]);
    if (g_have_prev) {
      const float dx = s.body[0] - g_prev_body[0], dy = s.body[1] - g_prev_body[1], dz = s.body[2] - g_prev_body[2];
      n += snprintf(line + n, sizeof(line) - n, " step=%.3f", std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    std::memcpy(g_prev_body, s.body, sizeof(g_prev_body));
    g_have_prev = true;
  } else {
    n += snprintf(line + n, sizeof(line) - n, " body=unreadable");
  }
  if (s.cam_ok) {
    const float* p = s.xf + 12;
    n += snprintf(line + n, sizeof(line) - n, " cam=(%.3f %.3f %.3f)", p[0], p[1], p[2]);
    if (s.body_ok) {
      const float d[3] = {p[0] - s.body[0], p[1] - s.body[1], p[2] - s.body[2]};
      n += snprintf(line + n, sizeof(line) - n, " cam-body=(%.3f %.3f %.3f) |d|=%.3f", d[0], d[1], d[2],
                    std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    }
  } else {
    n += snprintf(line + n, sizeof(line) - n, " cam=OPEN");
  }
  write_line(line);
  if (s.cam_ok && now - g_last_rows >= 1000) {  // all 16 floats once a second: which rows are the axes
    g_last_rows = now;
    const float* m = s.xf;
    logf("  cam rows: [%.3f %.3f %.3f %.3f] [%.3f %.3f %.3f %.3f] [%.3f %.3f %.3f %.3f] [%.3f %.3f %.3f %.3f]", m[0],
         m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
  }
}

void read_control(ULONGLONG now) {
  g_last_control = now;
  bool on = false;
  bool call = false;
  HANDLE file = CreateFileW(g_control_path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file != INVALID_HANDLE_VALUE) {
    char text[64] = {};
    DWORD got = 0;
    ReadFile(file, text, sizeof(text) - 1, &got, nullptr);
    CloseHandle(file);
    on = std::strncmp(text, "on", 2) == 0;
    call = on && std::strstr(text, "call") != nullptr;
    g_setters_wanted = on && std::strstr(text, "setters") != nullptr;
    g_mode_wanted = on && std::strstr(text, "mode") != nullptr;
    g_look_wanted = on && std::strstr(text, "look") != nullptr;
    g_bones_wanted = on && std::strstr(text, "bones") != nullptr;
  }
  if (on != g_on || call != g_call) {
    logf("t=%.1f control: %s%s", (now - g_t0) / 1000.0, on ? "on" : "off", call ? " call" : "");
    g_on = on;
    g_call = call;
    g_last_ptrs.clear();
    g_have_prev = false;
  }
}

void start() {
  g_started = true;
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
  g_control_path = dir + L"\\AvatarVR_player.txt";
  g_log_path = dir + L"\\PLAYER_PROBE.log";
  const std::wstring prev = dir + L"\\PLAYER_PROBE.prev.log";
  MoveFileExW(g_log_path.c_str(), prev.c_str(), MOVEFILE_REPLACE_EXISTING);
  g_t0 = GetTickCount64();
  logf("player probe build %s %s pid=%lu base=%p (read-only; control AvatarVR_player.txt: on|off)", __DATE__, __TIME__,
       GetCurrentProcessId(), static_cast<void*>(g_base));
}

}  // namespace

namespace player_probe {

void on_present() {
  if (!g_started) {
    start();
  }
  if (g_base == nullptr) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (now - g_last_control >= 1000) {
    read_control(now);
  }
  if (g_on && now - g_last_sample >= kSampleMs) {
    g_last_sample = now;
    sample(now);
  }
}

}  // namespace player_probe
