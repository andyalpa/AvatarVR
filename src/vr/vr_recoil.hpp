#pragma once
// Weapon recoil on the tracked gun (2026-11-17, research/WEAPONS_AND_HANDS.md "Recoil on the tracked gun").
//
// The game's own recoil is camera-driven (shake and aim climb) and its fire animation of the gun against the wrist is
// filtered out by the hand-frame filter, so neither reaches the VR gun. This adds a short kick to the weapon hand's one
// wrist target (guarded_hand_target), so hand, gun and arm move as one piece, and takes it back out of the aim: bullets
// and the in-world reticle keep the un-kicked ray (visual only). Strength per weapon follows the game's fire animation.
//
// Shot signal (setting recoil_signal): 0 auto = game rumble rise while the trigger is past 0.15 (LIVE 2026-11-17: one per shot,
// later rises of the shot inside the 180 ms refractory); 1 trigger edge only; 2 same as 0; 3 fire-animation spike while held. Every candidate is logged to
// scripts\VR_RECOIL.log whether or not the feature is on (read-only probe), so the signal can be checked against real shots.
// Rollback: menu Scripts > "Weapon recoil" off (default on since 2026-11-17) = no transform is applied anywhere.
#include <cstdint>

namespace vr_recoil {
bool* enabled_flag();      // menu "Weapon recoil" (default on)
float* strength_value();   // menu "Recoil strength" (0..2, default 1)
bool* haptic_flag();       // menu "Recoil haptic accent" (default off: the game's rumble already reaches the controllers)
float* signal_value();     // settings recoil_signal (see above)
float* reference_value();  // settings recoil_reference_cm: fire-animation residual that counts as strength 1 (default 2)

// Present thread, every frame: trigger poll, held weapon, residual windows, status log.
void on_present();
// Any thread: the game's XInput rumble (virtual_pad set_state detour).
void note_rumble(std::uint16_t left, std::uint16_t right);
// Render thread: this frame's attach<-wrist residual of one side against its filtered hold (cm).
void note_anim(int side, float cm);

// Any thread (guarded_hand_target): kicks the weapon hand's wrist target and its attach socket in place. root = that
// solve's body rows (row 1 = up). False and untouched when no kick applies.
bool apply(unsigned hand, const float* root, float* wrist16, float* attach16);
// Any thread: a ray given in the weapon wrist's frame (as drawn or placed, i.e. kicked) -> the un-kicked ray. False and
// untouched when no kick is active.
bool unkick_ray(int hand, float* origin, float* forward);
// True while a kick is being applied to this hand (health checks skip it).
bool active(int hand);
}  // namespace vr_recoil
