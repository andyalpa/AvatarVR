#pragma once

#include <cstddef>
#include <cstdint>

// Shared "is the player on a mount" state (FEATURE-06; research/CAMERA_FINDINGS.md "2026-10-24 FEATURE-06 live mount
// capture"). Read-only: refreshed once per local camera update (post phase) from three [LIVE] fields:
//   movement state  *(body+0x388)+0x1C0, vtable +0x10 getter (constant-return leaf): 7 while mounted, 2 on foot
//   mount partner   body+0x278: non-null only while mounted (partner +0x18 id, +0x30 type)
//   camera mode id  *(holder+0x40)+0xC: 20 mounting, 7 riding, 21 dismounting, 4 first person
// Mounted = state 7 or 8 (the game's own "Player Is On Mount" node test, 0x143A52D90) with a partner. The phase comes
// from the camera mode; an unknown mode while mounted counts as Riding and is logged (Ikran modes are UNKNOWN).
// Gameplay features that behave differently on a mount (camera anchor, gesture flight, harness grab) gate on this.
namespace mount_state {

enum class Phase : int { None = 0, Mounting = 1, Riding = 2, Dismounting = 3 };

struct State {
  Phase phase = Phase::None;
  int camera_mode = -1;           // -1 unreadable
  int movement_state = -1;        // -1 unreadable or not a constant-return getter
  const void* partner = nullptr;  // the mount, null on foot
  std::uint64_t partner_id[2] = {};
  int mount_type = -1;            // partner +0x30: 0 for the one mount observed (probably a direhorse)
  std::uint64_t since_ms = 0;     // GetTickCount64 when the phase last changed
  std::uint64_t updated_ms = 0;   // last refresh; get() reports None once this is older than kStaleMs
  std::uint64_t transitions = 0;  // phase changes since load
  bool guard_ok = false;          // IsOnMount prologue matches this build (otherwise nothing is read)
};

constexpr std::uint64_t kStaleMs = 500;

// Portable classification (unit-tested in tools/tests/mount_anchor_test.cpp).
inline Phase classify(int movement_state, bool partner, int camera_mode) {
  if (!partner || (movement_state != 7 && movement_state != 8)) return Phase::None;
  if (camera_mode == 20) return Phase::Mounting;
  if (camera_mode == 21) return Phase::Dismounting;
  return Phase::Riding;
}

const char* name(Phase phase);

// Game thread, local camera holder only (lua_host cam_update_detour, post phase).
void update_camera_post(void* holder);
// Any thread. Stale or pre-gameplay state reads as Phase::None.
State get();
inline bool mounted() { return get().phase != Phase::None; }

// Phase changes since the last call, oldest first (Present thread logs them). Returns the number written.
struct Transition {
  std::uint64_t ms = 0;
  Phase from = Phase::None, to = Phase::None;
  int camera_mode = -1, movement_state = -1, mount_type = -1;
};
std::size_t drain_transitions(Transition* out, std::size_t max);

// True when [p, p+n) lies in the game image's .text/.xpdata (safe to read as code). Shared with mount_camera.
bool image_code(const void* p, std::size_t n);

}  // namespace mount_state
