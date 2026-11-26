#pragma once
// FEATURE-05 manual rifle reload, Stage 1 (2026-11-19). With the assault rifle held, the off hand grabs the magazine,
// pulls it out (the game's own reload starts), drops it, takes a new one at its hip, seats it, and racks the charging
// handle after an empty magazine (the game's automatic reload). Firing waits for the hands. Logic in
// manual_reload_math.hpp; log scripts\VR_RELOAD.log.
// Game side: one pad X tap (the game's reload) at the grab; with pacing on, the reload timer's end is written so the
// game's progress waits at each step (manual_reload_math::pace). Render side: C_magazine / C_reloadLevel are drawn from
// the hands while the hands own the reload (skin_palette_probe reload_visual). The reload button keeps the game's reload.
namespace manual_reload {
bool* enabled_flag();  // menu "Manual reload" / [arms] manual_reload (default on; off = the game's reload only)
// menu "Hands pace the game's reload" / [arms] reload_pace (default on): engine write of the reload timer's end (author
// approval 2026-11-19), so the game's reload sound, HUD refill and ammo follow the hands' steps.
bool* pace_flag();
// menu "Hands pace the reload animation" / [arms] reload_anim_pace (default on, needs reload_pace): engine write of the
// reload animation's length (comp+0x3F38, author approval 2026-11-19, experiment), so the game's reload animation and its
// sounds wait for the hands instead of playing on their own clock.
bool* anim_pace_flag();
// Any thread: this hand shows the game's own fingers (its reload hold) instead of controller gestures.
bool native_hand(int hand);

struct Frame {
  bool press_x = false;     // tap the game's reload now
  bool block_fire = false;
  bool owns_grip = false;   // the off hand's grip belongs to the reload this tick
};
// From vr_controls::compose (game XInput poll thread, under its lock), gameplay only. off_head: the off hand in the head's
// yaw frame (x right, y up, z forward), valid when have_head.
Frame update(int weapon_hand, bool off_grip, bool reload_button, bool have_head, const float* off_head, bool haptics);
void reset();  // outside gameplay / VR controls off

// Render thread: how to draw the rifle's reload bones this frame.
struct Visual {
  enum Mag { kGame, kRest, kHand, kHidden };
  bool active = false;
  Mag mag = kGame;
  float mag_rel[16]{};      // kHand: the magazine bone in the gun's C_root frame (world = mag_rel * root, row vectors)
  bool lever = false;       // draw C_reloadLevel at rest + lever_pull
  float lever_pull = 0.f;   // m, back along the gun
  // Rack: pin this hand's wrist target to the charging handle (world = pin_rel * weapon socket, row vectors), blended by
  // pin_weight over its own target.
  int pin_hand = -1;
  float pin_weight = 0.f;
  float pin_rel[16]{};
};
Visual visual();
}  // namespace manual_reload
