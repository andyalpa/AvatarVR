#pragma once
// FEATURE-05 manual reload, Stage 0 (2026-11-19): read-only probe of the game's own reload, so the manual reload can copy
// its timeline. Writes nothing to the game. Log scripts\VR_RELOAD.log.
//
// [STATIC, ledger 0x3F4F3C0 / 0x32C9250 / 0xEC4680 / 0xEC5B90] comp = *(player body + 0x250) (the weapon component, the
// same one the bow charge chain starts from). The reload is a timer at comp+0x3EA8 (+0 state, IsReloading = state 0;
// +8 duration; +0x10 end time; +0x18 remaining while paused; +0x20 clock fn). The start event stores comp+0x3F38
// (duration), +0x3FA0 / +0x3FA8 (two absolute deadlines, likely magazine refilled / ready to fire) and +0x3F66 (empty
// reload, which uses +0x3FB0 / +0x3FB8). The fire handler drops shots until t/ready deadline >= 0.99.
// What each field does live is OPEN until this probe's log is read.
#include <cstdint>

namespace reload_probe {
bool* probe_flag();  // [arms] reload_probe (default on; logs only around a reload)
bool capture_on();   // probe or manual reload on: weapon draws are captured
void log_text(const char* line);  // VR_RELOAD.log

// Render thread (palette hook): the held rifle/shotgun as drawn this frame (the game's pose, before any reload override).
// names[i] = bone name of mats[i]; mats are row-vector world matrices without the cell offset (world = mat + 1024 *
// cells). socket_d: this draw's root to the tracked weapon socket (m), -1 unknown; per frame only the nearest counts
// (the game draws the gun 6-8 times a frame: eyes, shadows, the third-person shadow body).
void note_weapon_draw(const char* path, const char* const* names, const float (*mats)[16], unsigned n,
                      const std::int16_t (&cells)[3], float socket_d);

// From vr_controls::compose (game XInput poll thread), gameplay only: read the reload fields, learn the rest pose of the
// reload bones, log while reloading (probe flag).
void tick();

// The held weapon for the manual reload: in-hand C_root (world, within 200 ms) and the rest pose of C_magazine /
// C_reloadLevel in the C_root frame (learned while the game is not reloading and the bones hold still for 300 ms).
struct WeaponFrame {
  bool rifle = false;
  float root_world[16]{};
  bool have_mag = false, have_lever = false;
  float mag_rest[16]{}, lever_rest[16]{};
};
bool weapon_frame(WeaponFrame& out);
int game_reload_state();  // comp+0x3EA8 state as last read (0 = reloading), -1 unknown
// While the game reloads: the timer clock now (estimated from the start, ms), its end and duration.
bool reload_timer(long long& now_clock, long long& end, long long& duration);
// Engine write (author approval 2026-11-19): set the reload timer's end, so progress = 1 - (end - now) / duration.
bool write_reload_end(long long end);
// The reload animation's own length (comp+0x3F38, ms; the graph's ToolReloadingTime), while the game reloads.
bool reload_anim_time(long long& ms);
// Engine write (author approval 2026-11-19, experiment): set it, to slow / hold the reload animation and its sounds.
// Refused when not reloading or the timer's duration changed.
bool write_reload_anim_time(long long ms);
}  // namespace reload_probe
