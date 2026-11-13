#include "mount_state.hpp"
#include "../camera/mount_camera_capture.hpp"
#include "player.hpp"

#include <windows.h>
#include <cstring>
#include <mutex>

namespace mount_state {
namespace {
// Guard: the IsOnMount node's prologue (0x143A52D90, research/sdk/98588da2/mount_camera_static_20261023.txt). The
// offsets below were read on this build only; a different build reads nothing.
constexpr std::uintptr_t kOnMountRva = 0x3A52D90;
constexpr unsigned char kOnMountSig[] = {
  0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0x5A,0x20,0x8B,0x43,
  0x1C,0x25,0xFF,0xFF,0x00,0x10,0x83,0xF8,0x01,0x74,0x15,0xBA,0x01,0x00,0x00,0x00,
  0x48,0x8B,0xCB,0xE8,0x08,0xCB,0x63,0xFC};
constexpr std::size_t kRing = 32;

std::mutex mutex;
State state;  // mutex
Transition ring[kRing];
std::size_t ring_head = 0, ring_count = 0;  // mutex
int guard = -1;  // game thread: -1 unchecked, 0 mismatch, 1 ok
const void* cached_getter = nullptr;  // game thread: last decoded state getter
int cached_value = -1;

template<class T> bool read(const void* p, std::size_t off, T& out) {
  return p && game::mem_read(static_cast<const unsigned char*>(p) + off, &out, sizeof(out));
}
const unsigned char* ptr(const void* p, std::size_t off) {
  const unsigned char* value = nullptr;
  read(p, off, value);
  return value;
}

// Only a side-effect-free constant-return leaf is decoded; any other getter is never called.
int movement_state(const unsigned char* body) {
  const auto getter = ptr(ptr(ptr(ptr(body, 0x388), 0x1C0), 0), 0x10);
  if (!getter) return -1;
  if (getter == cached_getter) return cached_value;
  unsigned char code[16]{};
  int value = -1;
  if (!image_code(getter, sizeof(code)) || !game::mem_read(getter, code, sizeof(code)) ||
      !mount_camera::capture::constant_state(code, sizeof(code), value))
    value = -1;
  cached_getter = getter;
  cached_value = value;
  return value;
}
}  // namespace

const char* name(Phase phase) {
  switch (phase) {
    case Phase::None: return "none";
    case Phase::Mounting: return "mounting";
    case Phase::Riding: return "riding";
    case Phase::Dismounting: return "dismounting";
  }
  return "?";
}

bool image_code(const void* p, std::size_t n) {
  const auto base = game::module_base();
  IMAGE_DOS_HEADER dos{}; IMAGE_NT_HEADERS64 nt{};
  if (!read(base, 0, dos) || dos.e_magic != IMAGE_DOS_SIGNATURE || dos.e_lfanew <= 0 || dos.e_lfanew > 0x1000 ||
      !read(base, static_cast<unsigned>(dos.e_lfanew), nt) || nt.Signature != IMAGE_NT_SIGNATURE ||
      nt.FileHeader.NumberOfSections > 96)
    return false;
  const auto address = reinterpret_cast<std::uintptr_t>(p), start = reinterpret_cast<std::uintptr_t>(base);
  if (address < start || address - start >= nt.OptionalHeader.SizeOfImage) return false;
  const auto offset = address - start;
  const auto section_start = static_cast<unsigned>(dos.e_lfanew) + sizeof(DWORD) + sizeof(IMAGE_FILE_HEADER) +
                             nt.FileHeader.SizeOfOptionalHeader;
  for (unsigned i = 0; i < nt.FileHeader.NumberOfSections; ++i) {
    IMAGE_SECTION_HEADER section{};
    if (!read(base, section_start + i * sizeof(section), section)) return false;
    const bool allowed = std::memcmp(section.Name, ".text\0\0\0", 8) == 0 || std::memcmp(section.Name, ".xpdata\0", 8) == 0;
    if (allowed && offset >= section.VirtualAddress && offset - section.VirtualAddress <= section.Misc.VirtualSize &&
        n <= section.Misc.VirtualSize - (offset - section.VirtualAddress))
      return true;
  }
  return false;  // unknown/protected sections stay unread
}

void update_camera_post(void* holder) {
  if (guard < 0) {
    unsigned char got[sizeof(kOnMountSig)]{};
    guard = game::mem_read(game::module_base() + kOnMountRva, got, sizeof(got)) &&
            std::memcmp(got, kOnMountSig, sizeof(got)) == 0 ? 1 : 0;
  }
  const auto p = game::pointers();
  if (guard != 1 || !holder || holder != p.holder || !p.body) {
    std::lock_guard<std::mutex> lock(mutex);
    state.guard_ok = guard == 1;
    return;  // not refreshed: get() goes stale and reports None
  }
  State next;
  next.guard_ok = true;
  next.movement_state = movement_state(p.body);
  next.partner = ptr(p.body, 0x278);
  if (next.partner) {
    read(next.partner, 0x18, next.partner_id);
    read(next.partner, 0x30, next.mount_type);
  }
  read(ptr(holder, 0x40), 0xC, next.camera_mode);
  next.phase = classify(next.movement_state, next.partner != nullptr, next.camera_mode);
  next.updated_ms = GetTickCount64();
  std::lock_guard<std::mutex> lock(mutex);
  const bool fresh = state.updated_ms != 0 && next.updated_ms - state.updated_ms < kStaleMs;
  const Phase before = fresh ? state.phase : Phase::None;
  next.transitions = state.transitions;
  next.since_ms = state.since_ms ? state.since_ms : next.updated_ms;
  if (next.phase != before) {
    next.since_ms = next.updated_ms;
    ++next.transitions;
    ring[(ring_head + ring_count) % kRing] = {next.updated_ms, before, next.phase, next.camera_mode,
                                              next.movement_state, next.mount_type};
    if (ring_count < kRing) ++ring_count; else ring_head = (ring_head + 1) % kRing;
  }
  state = next;
}

State get() {
  std::lock_guard<std::mutex> lock(mutex);
  State out = state;
  if (out.updated_ms == 0 || GetTickCount64() - out.updated_ms >= kStaleMs) out.phase = Phase::None;
  return out;
}

std::size_t drain_transitions(Transition* out, std::size_t max) {
  std::lock_guard<std::mutex> lock(mutex);
  std::size_t n = 0;
  while (ring_count && n < max) {
    out[n++] = ring[ring_head];
    ring_head = (ring_head + 1) % kRing;
    --ring_count;
  }
  return n;
}

}  // namespace mount_state
