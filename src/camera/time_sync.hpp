#pragma once

// Synchronized alternate-eye rendering: advance the world once per eye pair (the BioshockVR "delta
// clamp" idea) through Snowdrop's global game-speed float. The float is the one the developer command
// "GlobalGameSpeed" resets to 1.0 (handler at afop.exe+0xD896E0 stores it with the setter at
// afop.exe+0x3301890: movss [rcx+0x4C], xmm1). The object pointer is read from the handler's own
// rip-relative operand after a byte-signature check, so no absolute address is hard-coded.
// Controlled by scripts\AvatarVR_time.txt:
//   off            -> never touch the game speed (default; restores the saved value if it was changed)
//   read           -> log the current game speed and flag only
//   speed <x>      -> hold game speed at x (0.05..4), e.g. a visible slow-motion check
//   pair           -> 2x on even presents, 0x on odd presents (one world advance per eye pair)
//   pairinv        -> same, opposite phase (the sim-to-render latency decides which phase matches)
namespace time_sync {

void install();   // resolve and verify the pointer slot (call once, after the module is mapped)
void poll();      // re-read the control file and log (Present thread, once a second)
void on_present();  // apply the per-present speed (Present thread, before the original Present)
// Current GlobalGameSpeed float and the byte at +0x48 of its object; false when unreadable. Any thread.
bool read(float* speed, unsigned char* flag);

}  // namespace time_sync
