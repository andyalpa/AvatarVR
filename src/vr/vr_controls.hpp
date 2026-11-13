#pragma once
// VR control scheme (2026-10-17, run 64; research/INPUT_OWNERSHIP.md "VR controls"). Builds the game's XInput pad from
// the controllers with gestures and weapon-hand-aware buttons:
//   reload   = the weapon hand's upper face button (left Y / right B) -> game X
//   jump     = right stick flick up -> A tap;  crouch = flick down -> B tap
//   fire     = the weapon hand's trigger -> RT; the support trigger sends nothing (ADS removed, automatic later)
//   scan     = empty hand at the headset + hold its grip -> RB held
//   wheel    = hand over its own shoulder + hold grip -> LB held, left stick -> right stick; release equips to that hand
//   equip    = hand over its own shoulder + grip tap -> Y tap (empty hand: equip into it; weapon hand: holster);
//              the empty hand while the other holds the weapon -> the weapon moves to it
// In gameplay grips and the support trigger never pass through directly; A, X and the free hand's upper button do.
// Menus and the front end always get the plain mapping. Off ("VR controls" menu) = the plain pad mapping.
#include <windows.h>
#include <xinput.h>
#include "xr_input.hpp"

namespace vr_controls {
bool* enabled_flag();   // menu "VR controls" (default on)
bool* haptics_flag();   // menu "Gesture haptics" (default on)
// Any thread (the game's XInput poll). gameplay = the player is in control (stereo running, not the front end, player
// valid). False when disabled or outside gameplay (menus, map, inventory, loading): the caller then uses the plain
// mapping, so every default menu input works. A gesture already in progress (wheel, scan) is finished first.
bool compose(const xr_input::PadState& s, XINPUT_GAMEPAD* out, bool gameplay);
// True while the weapon wheel is held: the right stick carries the wheel selection (no pitch levelling, no turn hold).
bool wheel_active();
// Read-only ownership for cosmetic fingers: scan/equip/wheel keep native hands.
bool hand_busy(int hand);
}
