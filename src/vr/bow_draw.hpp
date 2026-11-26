#pragma once
// FEATURE-07 bow string pull (2026-11-18). With a bow in the weapon hand, the other hand grips near the string, pulls
// back and opens the grip to shoot; the bow hand's trigger does nothing. Input only: it decides the pad's RT and never
// writes game memory, IK or weapon placement. Logic in bow_draw_math.hpp; log scripts\VR_BOW.log.
// Not while riding (the riding rig measures hands in ride-local space, not world): bows keep the native trigger there.
namespace bow_draw {
bool* enabled_flag();       // menu "Bow string pull" (default on; off = native trigger)
bool* hold_mode_flag();     // menu "Bow tension": off = pull distance sets the trigger, on = hold to charge
bool* press_fire_flag();    // menu "Bow fires on press": a trigger tap on release instead of holding while drawing
bool* probe_flag();         // [arms] bow_draw_probe: per-poll log of RT sent and the native draw (Stage 0)
float* draw_length_value(); // menu "Bow draw length" (m for a full draw)
bool* charge_flag();        // menu "Bow pull sets shot power": write the game's charge from the pull (engine write)
bool* visual_flag();        // menu "Bow string follows hand": the string and arrow are drawn at the hand's pull, 1:1

struct Frame {
  bool owns_trigger = false;  // send rt as the pad's RT; the bow hand's trigger is ignored
  float rt = 0.f;
  bool nocked_now = false;    // this update nocked the string
  bool cancel_button = false; // press the game's Cancel Shot (pad X)
  bool ready = false;         // a bow is held and the pull owns the trigger (a two-hand grab here nocks)
  bool drawing = false;
  int off_hand = -1;
};
// vr_controls: the off hand's grip press took the native two-hand hold (grab_two_hand == 1) with a bow ready; the
// next update nocks. The only way to nock: a grip in the air does nothing.
void request_nock();
// Any thread: a string is held (skip the two-hand aim swing, freeze the support hold and the bow's aim ray).
bool pulling();
// Any thread: during a pull, the game's own draw has reached full and settled (its arrow is then the true aim).
bool native_full();
// Any thread: the game's own draw is being set from the pull this moment (charge write working): the mod's render
// overrides (string, limbs, arrow, hand pin, frozen hold) stand down and the game draws the bow itself.
bool native_drive();
// From vr_controls::compose (the game's XInput poll thread, under its lock), in gameplay only.
Frame update(int weapon_hand, bool off_grip, bool haptics);
// Outside gameplay or with VR controls off: drop any draw without owning the trigger.
void reset();
// Render thread: while the string is held, how far back (m, world, along dir) to draw it. Render-only.
struct Visual { bool holding = false; float pull = 0.f; float dir[3]{}; unsigned nock = 0; };  // nock: +1 per grab
Visual visual();
// After the pad is composed: the RT actually sent (probe log only).
void note_sent(unsigned char rt);
}  // namespace bow_draw
