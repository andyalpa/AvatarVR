#pragma once

#include <cstdint>

// FEATURE-06. Stage A (2026-10-24): while mounted (mount_state) and the toggle is on, the eye cameras, grips and aim
// rays are built on the rider's head instead of the mount chase camera (mount_anchor.hpp). Only our private eye
// cameras move; nothing is written to the game. The read-only transition capture is kept for later stages.
namespace mount_camera {
bool* first_person_flag();  // Present/menu thread only
bool* mounted_arms_flag();  // Present/menu thread only: stage B arm IK while mounted (default on, saved)
bool mounted_arms();        // any thread
// Harness grip learned from the game's riding pose (192-pose palm, in the rider's pelvis frame; rider_ik.hpp). False
// until enough steady samples were recorded this session. For the later grab / release and gesture flight.
bool harness_grip(int hand, float* pelvis_local3, float* spread_m = nullptr);
// Harness grab (any thread). harness_near: that controller is within reach of its harness point this step (a grip
// press there grabs; vr_controls then starts no other gesture). arm_ik_weight: 1 = arm on the controller (stage B IK),
// 0 = game riding pose holding the harness, smooth in between.
bool harness_near(int hand);
// Any thread: the eye point for a third-person C_head matrix (same calibration the eye anchor uses; the head joint
// itself until calibrated). Space in = space out.
void eye_point(const float* head16, float* eye3);
bool harness_held(int hand);
float arm_ik_weight(int hand);
// Game thread (native_stereo head step): world delta from the game camera position `cam_pos` to the head anchor.
// Writes 0 and returns false while inactive (toggle off, on foot, first-person camera mode, anchor unreadable).
bool eye_anchor(const float* cam_pos, float* delta3);
// Any thread: the eye anchor is active. While it is, the palette hook hides the rider's third-person head and hair and
// does not apply the arm IK (it solves on the first-person rig, which the game does not animate while mounted).
bool rider_view();
void request_capture();    // queues one bounded capture; waits for gameplay
void cancel_capture();
void on_present();
void on_camera(void* holder, bool post);  // existing local camera hook, unchanged arguments
void observe_palette(const void* mesh, const void* root, const void* bones, const void* cells,
                     std::uintptr_t caller_rva);
// Diagnostic only. Called at the existing, caller-filtered weapon placement seam; native16 is copied
// before the mod changes it. Records are queued to the same bounded mount capture, never written here.
bool capture_active();
void observe_weapon_entity(const void* object,std::uintptr_t caller_rva,bool native_valid,const float* native16,
                           const char* reason,float flat_gap,float game_gap,bool game_socket,bool wrote);
// Render-side read-only wrist observations; one paired sample per body resource at <=4 Hz.
void observe_rider_hands(const void* mesh,const void* resource,const void* source,const void* owner,const short* cell3,
                         const bool* valid2,const float (*native)[16],const float (*rendered)[16],
                         const float* weight2,std::uint64_t sequence);
}  // namespace mount_camera
