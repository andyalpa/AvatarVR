#pragma once
#include <cstdint>
#include <string>
#include "skin_arm_ik.hpp"
namespace skin_palette_probe {
// Mounted-only aggregate observation. No pose/shot writes or per-call file I/O.
void note_mounted_weapon_attempt(const char* reason,bool wrote);
void on_present();
// Read-only counters for correlating mount transitions with actual IK work.
// These are independently sampled atomics, not a render-completion guarantee.
struct Counters { unsigned fp_body = 0, accessories = 0, weapon = 0, solves = 0, gun_places = 0; };
Counters counters();
// Read-only snapshot for mount ownership diagnostics. Ages use UINT64_MAX for missing data.
struct WeaponCaptureState {
  std::uint64_t fp_age_ms=UINT64_MAX,support_age_us=UINT64_MAX,measure_age_us=UINT64_MAX;
  bool fp_draw_valid=false;
  float fp_draw[2][16]{};
  bool support_valid=false,two_hand_held=false,support_only=false;
  int support_hand=-1,swap_target=-1;
  float support[16]{},dominant[16]{},support_distance[2]{},handle_distance[2]{};
  std::uint64_t mounted_bind_age_ms=UINT64_MAX,mounted_commit_age_ms=UINT64_MAX;
  const void* mounted_owner=nullptr; const void* mounted_candidate=nullptr; const void* mounted_resource=nullptr;
  const void* mounted_object=nullptr;
  bool mounted_bound=false,mounted_committed=false;
  std::string mounted_weapon_path;
  bool mounted_grip_profile=false,mounted_support_profile=false;
  bool mounted_onfoot_hold=false,mounted_finger_profile[2]{};
  bool mounted_aim_profile=false;
  unsigned mounted_profiles=0,mounted_disk_profiles=0,mounted_commit_failures=0;
  unsigned mounted_finger_apps=0,mounted_finger_rejects=0,mounted_pin_expired=0,mounted_pin_distant=0;
  unsigned mounted_palette_adopted=0,mounted_palette_unbound=0,mounted_palette_not_ready=0,mounted_palette_invalid=0;
  float mounted_pin[2]{};
  unsigned mounted_source=0,mounted_hand=1;
  std::uint64_t mounted_frame_sequence=0,mounted_commit_sequence=0;
  float mounted_flat[16]{},mounted_target[16]{},mounted_placed[16]{};
};
WeaponCaptureState weapon_capture_state();
// Bounded name hints for diagnostics only; indexes are candidates and their actual names are reported.
std::string capture_palette_names(const void* root);
// Game thread, after the camera-manager update: "posewrite" experiment (IK hands into the game's own poses).
void on_camera_post();
// Mod menu "Arm IK (controllers)": while on (and Script writes on, in gameplay) the controller arm IK runs with no time
// limit. Off at every launch. AvatarVR_skinik.txt "on" does the same.
bool* ik_menu_flag();
// Hand alignment tuning and "Shoulders follow headset" (mod menu, reset at launch).
skin_arm_ik::HandTune* tune();
bool* shoulders_flag();
bool* twist_flag();   // "Forearm twist"
bool* anchor_flag();  // "Steady hands while moving"
void reset_tune();    // back to the tuned defaults
bool* stable_flag();             // "Stable shoulders (rest pose)"
bool* native_hands_flag();       // "Native hand poses" (live finger/grip animation)
bool* arm_scale_flag();          // "Arm scaling (Na'vi reach)"
float* user_arm_length_value();  // "Your arm length" (m)
float* user_shoulder_drop_value();   // "Your shoulder drop" (m below the eyes)
float* user_shoulder_width_value();  // "Your shoulder width" (m)
float* reach_multiplier_value();     // "Reach multiplier"
bool* rig_lock_flag();
bool* hybrid_reach_flag();             // "True hands near body" (on-foot hybrid reach, 2026-11-13)
float* reach_blend_near_value();
float* reach_blend_far_value();
bool* soft_reach_flag();               // "Soft arm reach"
bool* elbow_smoothing_flag();          // "Smooth elbows"
float* elbow_smoothing_ms_value();
bool* stabilize_flag();
bool* tick_resolve_flag();
bool* root_carry_flag();
bool* shoulder_socket_flag();
bool* clean_rig_flag();           // "Clean arm rig" (run 54)
bool* left_handed_flag();         // "Weapon hand" Right/Left (run 59)
bool* mounted_weapon_flag();      // C1: native mounted gun to the selected tracked hand, saved/default on
struct MountedShotFrame {
  float flat[16]{},root[16]{};
  skin_arm_ik::Vec muzzle{},forward{};
  bool calibrated=false;
  std::uint64_t sequence=0,age_ms=0;
};
// Shot producer only. Never exposes mounted sockets to generic FP effect/world writers.
bool mounted_weapon_shot_frame(const void* weapon,MountedShotFrame& out);
void observe_onfoot_weapon_ray(const void* weapon,const float* flat,const float* start,const float* target);
// Existing entity seam only. Requires a fresh named player-weapon palette/TP socket binding.
bool mounted_weapon_entity_target(const void* object,const float* native16,float* moved16,const char** reason);
void mounted_weapon_entity_commit(const void* object,const float* moved16,bool success);
// Run 65: start a smooth hand swap (the gun travels to `to_hand`, 0 left / 1 right, poses cross-fade; the weapon hand
// flips when it ends). False when one is running or the gun is already in that hand.
bool begin_hand_swap(int to_hand);
bool hand_swap_active();
// Run 67 two-hand aiming. grab_two_hand(empty hand) on its grip press: 1 = two-hand hold started (support point nearer),
// 2 = nearer the trigger handle (caller does the hand swap), 0 = neither. release_two_hand on its grip release.
int grab_two_hand(int hand);
void release_two_hand(int hand);
bool* two_hand_flag();            // "Two-hand aiming" (default on)
// Run 68 support-hand-only hold: the trigger hand's grip press while two-handed. 1 = let go, 2 = re-grabbed the handle.
int dominant_grip_press();
bool support_only();
bool fire_blocked();              // the trigger hand is off the gun (or easing back): no shots
bool* view_height_flag();         // "Shoulders follow view height" (run 56)
bool* arms_view_lock_flag();      // "Lock arms to view" (2026-10-30)
bool* hand_freeze_flag();         // "Freeze hand animation while moving" (2026-10-30)
// Entity hook only (once per game tick): the gun socket evaluated from the shared controller sample and this tick's body.
bool mounted_placed_pair(float (&flat)[16],float (&placed)[16]);  // riding: native socket -> placed gun of the last commit
bool mounted_entity_local(const std::int16_t* entity_cell,float* local3);  // riding: the pending gun position in that cell, exact
bool rig_coherent_on();            // [arms] rig_coherent, see research/IK_PIPELINE_AUDIT_20261031.md
bool player_standing_still();      // body still for 0.5 s, no weapon change
bool placed_socket_local(float* local3,std::int16_t* cell3);  // the gun record's position in its own cell; only within 3 ms of the placement
bool weapon_attach_place(int hand, float (&flat)[16], float (&ik)[16]);     // "Gun hand follows shoulder and controller"          // "Body turn in step"        // "Arms follow every game tick"
bool* model_hands_flag();           // "Model-space hands"             // "Stabilize hands while moving"               // "Lock body to view"
float* weapon_trail_value();         // "Weapon hand trail" (0 = full trail, 1 = none)
void calibrate_arms();               // "Calibrate arms (T-pose)"
bool* accessory_physics_flag();      // "Accessory physics" (mod-side charm/cord spring)
bool* smoothing_flag();              // "Hand smoothing" (One-Euro)
bool* pose_write_flag();
bool* weapon_pose_write_flag();     // old weapon pose-array write (not in the menu)
float* shoulder_spread_value();       // "Shoulder separation" (m)
float* hand_separation_value();      // "Hand separation" (m, mirrored side offset)
float* shoulder_height_value();  // "Shoulder height" (m)
float* shoulder_forward_value(); // "Shoulder forward" (m)
void recapture_rest();           // "Recapture rest pose" (stand still)
// Last solved frame's weapon attachment bone (hand 0 left, 1 right) as world matrices: flat (animated) and IK.
// False when the IK has not solved within 200 ms or no held weapon has been seen yet (vr_weapon).
bool weapon_attach_matrices(int hand, float (&flat)[16], float (&ik)[16]);
bool held_weapon_drawn();  // a named weapon mesh was drawn in hand within 200 ms, under either weapon placement path
// The held bow's C_arrow_01 / C_root / C_bowString palette matrices as drawn, world; false when no bow drew in 200 ms (read-only).
bool drawn_arrow(float (&arrow)[16],float (&bow_root)[16],float (&string)[16],bool* have_string,char (&path)[96],char (*bow)[96]=nullptr);
std::string arrow_report();  // log only: arrow/bow captures since the last call, and the angle between their C_arrow_01 +Z; resets
bool held_weapon_root(float (&out)[16]);  // flat (animated) C_root of the in-hand weapon, world; false if stale (200 ms)
unsigned rider_hidden_builds();  // FEATURE-06: head/hair palette builds collapsed while the eyes ride on the head
// FEATURE-06 stage B: hidden head/hair builds, meshes posed by the rider arm IK, arm solves, meshes skipped because
// their arm bones did not match the 192 pose, solves without a fresh controller grip, unnamed bones anchored to an arm
// segment, largest rejected mismatch (mm).
struct RiderCounters { unsigned hidden, arm_apps, solves, mismatch, no_grip, anchored; int mismatch_mm, mismatch_bone;
                       unsigned self_solves, rest_solves, rest_captures; };
RiderCounters rider_counters();
// Harness frame census since the last call (non-player meshes near the rider, sorted by how little the game's palms
// move in their frame), one line per mesh; clears it. Present thread.
std::string rider_frame_report();
bool weapon_hold(int hand,float (&rel)[16]);  // gun socket = rel * wrist, this frame's animated relation; clean rig only
int weapon_hand_index();         // persisted "weapon_hand": 1 right (default), 0 left
// Wrist HUD: the drawn wrist and elbow relative to that hand's grip, in grip axes (x right, y up, z forward; metres).
// False when the first-person rig has not drawn within 150 ms (mounted, or its bones were not applied).
// attach3 is the weapon attachment bone (the wrist before it is known). Smoothed, so a weapon's recoil does not shake panels.
bool hud_arm_anchor(int hand,float* wrist3,float* elbow3,float* attach3=nullptr);
// Native arm HUD: bones as last drawn, unfiltered, in camera axes (x right, y up, z forward). Which camera is a live choice.
constexpr int HudBoneSlots=16;
struct HudCameraPose {
  bool hand_ok[2]{},bone_ok[HudBoneSlots]{};
  float wrist[2][3]{},elbow[2][3]{},attach[2][3]{},attach_axes[2][9]{},wrist_axes[2][9]{},outward[2][3]{};
  float grip_axes[2][9]{};  // controller x, y and forward, as the rig solves with
  float bone[HudBoneSlots][3]{},bone_axes[HudBoneSlots][9]{};
  float age_us=0.f;
  std::uint64_t frame_id=0;  // applied pose this was measured against: use its eyes
  bool rider=false;          // mounted rig: bone axes differ from the first-person rig
  // Mounted: the gun's calibrated shot ray from the drawn palm, in camera axes. ray_flat: no mounted gun or calibration, the game's own mark stays.
  bool ray_ok=false,ray_flat=false;
  float ray_start[3]{},ray_way[3]{};
};
void set_hud_frame_method(int method);  // 0 newest camera, 1 rendering camera's rotation at the arm rig's anchor (default), 2 arm rig frame, 3 rendering frame's camera
void hud_camera_gaps(float* mm,float* degrees);  // log only: largest rig frame vs rendering camera gap since the last call
void set_hud_bone(int slot,int bone);  // -1 = none
bool hud_camera_pose(HudCameraPose* out);  // false when not drawn within 150 ms
void hud_camera_range(float* left_wrist_mm3);  // log only: left wrist travel in this frame per axis since the last call
bool hud_camera_seen();                    // drawn at least once since load
// Bone name for the wizard's list, or empty when out of range or not read from the game yet.
std::string hud_bone_name(int bone);
}
