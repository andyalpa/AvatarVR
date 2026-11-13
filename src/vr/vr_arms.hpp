#pragma once
// Clean arm rig input stream (2026-10-17, run 54): one controller sample per eye-camera build (game frame), relative to
// the headset centre, in the eye camera's own axes, One-Euro filtered once here. Every consumer (free-hand solve, gun
// hand, game-thread gun placement) reads the same sample. Readers never fail on a busy lock (blocking, tiny section).
namespace vr_arms {
struct Sample {
  float local[2][16] = {};   // rows 0-2 grip axes, row 3 = grip - head (camera axes)
  float raw_local[2][16] = {}; // same input before the one shared hand filter; diagnostics only
  bool valid[2] = {false, false};
  float basis[16] = {};      // eye basis the sample was taken with (rows 0-2), for basis_for_root
  float view_offset[3] = {}; // game camera position minus body position at this sample (crouch drop, run dip), world
  float raw_view_offset[3] = {}; // before the shared view-offset filter; diagnostics only
  float previous_view_offset[3] = {}; // raw offset of the sample before this one
  unsigned view_gen = 0;     // Present generation this sample was published in
  bool view_locked = false;  // "Lock arms to view": read the anchor through anchor_view_offset
  bool view_valid = false;
  float head_rot[9] = {};    // headset rotation as eye_transform returns it (run 64: gesture zones in the head frame)
  bool head_valid = false;
  float head_world[3] = {};  // headset-centre offset in the sampled game-camera basis
  bool head_world_valid = false;
  unsigned long long seq = 0;
  long long sampled_qpc_us = 0; // camera-step publication time; diagnostics only
};
// native_stereo::compute_aim: rotation9/offset3 exactly as eye_transform returns them for each grip, the headset centre
// offset (mean of both eyes, same convention) and the eye basis matrix.
void publish(const float (&rotation)[2][9], const float (&offset)[2][3], const bool (&valid)[2], const float* head_offset3,
             const float* basis16, const float* head_rot9);
bool latest(Sample& out);
void set_smoothing(bool on);
void set_view_lock(bool on);  // on: free hands, gun hand, gun and eyes share one camera offset per frame; off: filtered
void set_rotation_tuned(bool on);  // on: grip rotation has its own One-Euro setting (skin_arm_ik::set_rotation_tuning)
unsigned basis_rejects();          // eye bases refused by publish (status log)
// One anchor per game tick for the gun, the arms and the eyes, whichever asks first (gun placement, or the eye step at publish).
// offset3 = anchor - body3, so each caller lands on the same world point even with a different body read.
void set_view_stable(bool on, bool track);  // on: the lock uses this anchor, also while riding; track: world path smoothing (on foot)
bool view_stable();
bool stable_view_anchor(const float* body3, float* offset3);
// The eye shift of the sample published in this Present generation; false (zero) when the latest sample is older.
bool frame_eye_shift(float* out3);
// The input the armed rig uses, for every hand state: the sample published before this frame's camera step (the gun placement runs just
// before that step and takes it), with this frame's anchor offset. Falls back to the latest sample when there is no such pair.
bool armed_sample(Sample& out, float* anchor3);
unsigned published_steps();  // camera steps published so far: the bare-hand twin of the gun's placement tick
void note_present();          // Present generation for anchor_view_offset
const float* anchor_view_offset(const Sample& s);  // the shoulder/gun anchor offset of this frame, for every hand state
void eye_view_delta(float* out3);  // world shift that puts the eyes on the same offset (latest sample; 0 when unlocked)
// Filter the game's camera offset from the body for the shared shoulder/gun anchor.
// Returns false for a non-finite or implausible offset.
bool stable_view_offset(const float* raw3, float* out3);
}
