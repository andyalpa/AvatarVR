#pragma once
#include <cstdint>
namespace vr_weapon {
// Installs the shot-producer hook once in gameplay and refreshes the write permission (call every Present).
void on_present();
// The ray the player's shot uses, in the gun socket's own axes, and how far it runs before hitting something (metres).
// False when shots do not follow the tracked gun, or while mounted: the reticle then stays on the flat HUD.
// The steady shot ray in the weapon hand's wrist frame, and how far it runs. False until it has settled or when no shot went through lately.
bool shot_aim(float* origin,float* forward,float* distance);
bool shot_distance(float* distance);  // mounted: how far the gun's ray runs (the ray itself is in HudCameraPose)
bool shot_ray_on();  // shots are moved onto the gun, so an aim ray is expected
bool held_is_bow();  // the current weapon has shown a drawn arrow (bows get no recoil kick)
// Menu "Shoot from tracked gun": shots and aim leave the IK gun. On by default.
bool* shoot_from_tracked_flag();
// Menu "Weapon follows tracked gun": the game places its weapon entity at the IK hand (0x141859AD0 hook). On by default.
bool* effects_flag();
// True while the weapon component source is being moved onto the IK socket (last 150 ms): the game then builds the
// weapon pose at the tracked gun itself, so the renderer must not apply the hand change a second time.
bool weapon_placed_by_game();
// The IK weapon socket (world) used for the latest weapon entity move, and how far it moved since the previous one.
bool written_socket(float (&ik_world)[16], float* motion, unsigned long long* age_ms=nullptr);
// The same socket in the caller's cell, computed without passing through world floats (false: use written_socket).
bool written_socket_in_cell(float (&m)[16], const std::int16_t* cell);
unsigned rig_far_skips();    // status log: other characters' weapons refused by the entity hook
unsigned rig_bound_moves();  // status log: weapon entity moves placed from the fixed per-object offset
// The controller-driven socket (world) of the latest placement, before the anti-sway correction.
bool target_socket(float (&ik_world)[16]);
// Render thread: gap (world, metres) between the drawn target root of the in-hand weapon and the game's placement.
void report_render_error(const float* world_error);
// Game thread, after the camera update: re-applies the weapon world matrix move.
void on_camera_post();
// Gun timing recorder (run 44, read-only): one 3 s window per launch, started the first time the view turns while the
// gun has been in hand >= 5 s. Records E (entity placement), S (arm solve), H (gun hand takes its socket), G (gun palette
// draw), B (FP body palette draw), P (Present) and every read/write (R/W) of the gun's bone-0 matrix (guard page), with
// thread id and QPC microseconds, into scripts/GUN_TIMING.csv. No game value is changed.
void timing_mark(char kind,const float* world_pos=nullptr,unsigned value=0);  // value lands in the rva column
void timing_offer(const void* gun_bone0);
// Game tick counter of the weapon placement (run 45): +1 for each game tick that placed the gun (the two placement calls of
// one tick come within 0.1 ms; ticks are >= 13.5 ms apart, [LIVE run 44]).
unsigned placement_tick();
}
