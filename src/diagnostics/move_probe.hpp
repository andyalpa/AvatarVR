#pragma once
// ROOMSCALE (permanent, on at launch; "noroomscale" in scripts\AvatarVR_moveprobe.txt turns it off, Script writes
// required): the player's Havok capsule follows the headset (one injected integrate step per movement call, view
// rebased by what the capsule actually moved), and the body's yaw follows the head beyond a dead zone ("turndz=<deg>",
// default 15; on by default, "nobodyturn" off: steered through the virtual pad's right-stick X, the view rebased by the measured
// game yaw change). Summary each second in MOVE_ROOMSCALE.log. Receipts:
// research/ROOMSCALE_BODY_20261022.md.
//
// Roomscale discriminator (research/ROOMSCALE_BODY_20261022.md "Next discriminator"): read-only observer of the
// player movement update 0x144271680. Inert, with no hook installed, unless AvatarVR_moveprobe.txt says "on"; then
// one 20 s window logs the local player's body / motion target / proxy fields around each call, and the camera
// update phases for ordering, to MOVE_PROBE.csv (+ summary MOVE_PROBE.log). The capture never writes game memory.
// Nudge test (the one write): with "nudge" in the same file and Script writes on, each F11 press (game window
// focused, at most one per second) moves the local capsule 10 cm along the game camera's horizontal forward by adding
// velocity d/h to one Havok integrate step (removed again afterwards); that call and the next six go to MOVE_NUDGE.log.
namespace move_probe {
void on_camera(bool post);  // from the camera update detour (game thread), local holder only
// Virtual pad (XInputGetState, after pitch levelling): the right-stick X to report to the game. Returns user_rx
// unchanged unless the body turn (default on, "nobodyturn" off) is steering the game's yaw toward the head.
float body_turn_stick(float user_rx);
// Virtual pad, VR gameplay: with "Movement direction: Head" (word "headmove") the left stick is rotated so forward is
// where the headset looks; otherwise unchanged.
void head_move_stick(short* lx, short* ly);
// VR menu rows (Present thread), saved as words in AvatarVR_moveprobe.txt.
bool* body_turn_flag();  // "Body turns with head" (default on; off writes "nobodyturn")
bool* head_move_flag();  // "Movement direction" Game (default) / Head (writes "headmove")
void on_present();
}
