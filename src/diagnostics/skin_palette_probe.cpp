#include "skin_palette_probe.hpp"
#include "skin_palette_pulse_math.hpp"
#include "skin_arm_ik.hpp"
#include "rider_ik.hpp"
#include "mounted_accessory_bounds.hpp"
#include "pose_names.hpp"
#include "../game/player.hpp"
#include "../script/lua_host.hpp"
#include "../camera/native_stereo.hpp"
#include "../camera/mount_camera.hpp"
#include "../game/mount_state.hpp"
#include "../vr/virtual_pad.hpp"
#include "../vr/vr_weapon.hpp"
#include "../vr/vr_arms.hpp"
#include "../vr/wrist_hud.hpp"
#include "../vr/vr_controls.hpp"
#include "../vr/xr_input.hpp"
#include "../vr/hand_gestures.hpp"
#include "../vr/hand_gesture_pose.hpp"
#include <memory>
#include "../vr/vr_arms_math.hpp"
#include "../vr/mounted_weapon_math.hpp"
#include "../vr/mounted_hand_pose.hpp"
#include "../vr/mounted_weapon_profiles.hpp"
#include "../vr/vr_weapon_math.hpp"
#include <windows.h>
#include <intrin.h>
#include <MinHook.h>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <array>
#include "../settings_file.hpp"

namespace {
constexpr std::uintptr_t kPaletteRva=0x2C2A70;
constexpr unsigned char kPaletteSig[]={0x48,0x8B,0xC4,0x48,0x89,0x58,0x18,0x48,0x89,0x78,0x20,0x41,0x56,0x48,0x81,0xEC,0xC0,0x00,0x00,0x00,0x0F,0x10,0x02,0x48};
using Palette=bool(__fastcall*)(void*,const void*,const void*,const void*,const void*);
Palette original=nullptr;
using MeshPrimary=void(__fastcall*)(void*,void*);
MeshPrimary primary_original=nullptr;
struct MeshPrimaryScope {const void* record=nullptr; const void* mesh=nullptr;};
thread_local MeshPrimaryScope primary_scope;
std::atomic<ULONGLONG> deadline{0};
std::atomic<unsigned> calls{0};
std::atomic<const void*> window_body{nullptr};
std::atomic<bool> pulse_permission{false};
std::atomic<ULONGLONG> pulse_until{0},pulse_started{0};
std::atomic<const void*> pulse_body{nullptr},pulse_mesh{nullptr};
std::atomic<unsigned> pulse_calls{0};
bool pulse_wanted_before=false;
bool ik_menu=true; // mod menu "Arm IK (controllers)", Present thread (acts only with Script writes on)
// Mod menu tuning (Present thread writes, render threads read single floats/bools).
skin_arm_ik::HandTune make_default_tune() {
  // [AUTHOR] 2026-10-15 headset tuning with arm scaling (AvatarVR_arms.txt 09:50): pitch -65, yaw -30, roll -35,
  // hand separation (side, mirrored) 0.18, forward/up 0.
  skin_arm_ik::HandTune t; t.pitch=-65.f; t.yaw=-30.f; t.roll=-35.f; t.forward=0.f; t.up=0.f; t.side=0.18f; return t;
}
skin_arm_ik::ArmScale make_default_scale() {
  // [AUTHOR] 2026-10-15 T-pose calibration (span 1.61-1.65 m): arm 0.65, shoulder width 0.32, drop 0.20, back 0.03.
  skin_arm_ik::ArmScale a; a.user_length=0.65f; a.width=0.32f; a.drop=0.20f; a.back=0.03f; a.reach=1.f; return a;
}
skin_arm_ik::HandTune hand_tune=make_default_tune();
bool forearm_twist=true; // spread the wrist's twist over the lowerArm_twist bones
bool body_anchor=true;   // shift grip targets by the body's movement since the grips were sampled
std::atomic<unsigned> frame_gen{1};  // advances on Present (moved up: used by the solve and the weapon draw)
// [AUTHOR run 35] Model-space hands (Cyberpunk VR port style, `VRIK_BuildHandTarget`): the hand target is the controller
// relative to the headset of the same sample (bob/position cancel), rotated into the drawn frame, minus the user's
// calibrated shoulder offset, scaled and added to a body-fixed Na'vi shoulder (rest pose anchored to the FP root, not
// the camera bone). Replaces the compensation layers (body anchor, yaw delta, frame-camera re-expression, stabilize,
// rig shift) and the gun's camera-rigid placement (now body-rigid). Saved toggle "Model-space hands", default on.
bool model_space_hands=true;
bool rest_on_root=false;  // which reference the captured rest pose belongs to
std::atomic<unsigned> model_hand_solves{0},body_rigid_moves{0};
signed char bone_twist[211]{}; // 1 for *_lowerArm_twistNN
// Accessories with their own (non-211) bone vectors: bones whose source matrix equals an FP bone are replaced by that
// FP bone's IK result. The FP rig is static, so copies made from it match exactly across frames.
alignas(16) float fp_raw_last[211][16];
alignas(16) float fp_out_last[211][16];
float fp_root_last[16]{}; // FP mesh root of the frame fp_out_last was built for
// Latest FP body root (set by the FP mesh's palette build). Other meshes solve and re-root against it, never against
// their own root: [LIVE] 2026-10-15 a held weapon (root ~2 m up at the hand) solved first, and the FP mesh then re-rooted
// the result by weapon root -> FP root, dropping the arms ~2 m ("IK hands go to floor level").
float fp_body_last[3]{};          // body position of the frame fp_out_last was solved for (cell-local, palette cell)
std::int16_t fp_cell_last[3]{};
float fp_root_latest[16]{};
bool fp_root_valid=false;
ULONGLONG fp_last_at=0;
std::mutex fp_last_mutex;
// FP root rotation with this frame's body position as translation (cell-local); false until the FP mesh was seen.
// run 46 timing: rotation angle of a re-root (centi-degrees) and the drawn weapon-hand attach bone in world units.
unsigned reroot_centideg(const float* m) {
  const float c=std::clamp((m[0]+m[5]+m[10]-1.f)*0.5f,-1.f,1.f); return static_cast<unsigned>(std::acos(c)*5729.578f);
}
// [LIVE run 46 GUN_TIMING] while stick turning the FP body re-rooted the shared solve by 2.2 deg (median, max 3.8) on
// every draw, and its drawn weapon hand sat 2.5 cm (median, max 8.6) from the gun; gear meshes drew the hand elsewhere
// again (p95 4.3 cm). Cause: a gear mesh usually solves first, with fp_root_latest = the root of the LAST FP body draw
// (one game tick old in yaw); the FP body then rotates that solve to its true root, carrying the gun hand off the gun.
// "Body turn in step" (default on): the stored root rotation is carried into this tick by the game's own Marker change
// (90-pose bone 1, the same rule the gun placement uses): root_now = root_then * inverse(Marker_then) * Marker_now
// (rotation only; position = body now). Solve, gear and FP body then share the current yaw.
bool root_marker_carry=true;
float fp_root_marker[16]{}; bool fp_root_marker_valid=false;  // fp_last_mutex: 90-pose Marker when fp_root_latest was stored
std::atomic<unsigned> root_carries{0};
// [AUTHOR run 51, SOURCE] the gun hand lags while moving with the weapon equipped: its target is the socket saved at the
// PREVIOUS solve, carried into the game tick by the body root (Marker turn + body position) only. The empty hands are
// solved each frame from THIS frame's shoulder plus the live controller. So the gun hand misses (a) the shoulder's walk
// animation of the new tick and (b) one solve of controller motion. "Gun hand follows shoulder and controller"
// (default on): the socket is carried about the right upper arm (drawn 211 bone -> same bone in this tick's 90-pose,
// found once by position), rotation still by the Marker change; and the controller's tracking-space motion since that
// solve is added (clamped 6 cm / 15 deg). Falls back to the run 47 body-root carry when either is unavailable.
bool shoulder_socket=true;
int shoulder90[2]={-2,-2};  // 90-pose index of the 211 upper arm per hand (-2 unknown); fp_last_mutex
int shoulder90_candidate[2]={-1,-1}; unsigned shoulder90_hits[2]={0,0};
std::atomic<unsigned> shoulder_carries{0},controller_deltas{0};
// ---- Clean arm rig (run 54, plan "clean arm rig") ----
// One controller sample stream (vr_arms, filtered once), one target function (vr_arms_math::hand_target) for the free
// hand, the gun hand and the game-thread gun placement, the per-tick basis taken from the tick's own body root, and no
// flat-pose fallback while the IK is on (the last good rig is reused instead). Replaces, while on: the frame_cam map and
// its per-frame recalibration, "Stabilize hands", the legacy non-model path, the carried gun socket (Marker/shoulder
// carry, controller delta) and the root-frame hand filter. Off = the run 37-53 code path (A/B).
bool clean_rig=true;
// [HEADSET run 55] crouching left the arms and gun at standing height, and running bounced them against the view: the rest
// pose was rigid to the body root (feet), while the view follows the animated camera (crouch, run dip/bob). "Shoulders
// follow view height" (default on): the rest-pose anchor keeps the root's axes but takes the CameraMarker's position
// (211 bone 1 / 90-pose bone 0), so the shoulders - and with them the hands and the gun - move with the view's height.
bool view_height_anchor=true;
bool arms_lock_to_view=true;  // "Lock arms to view" (2026-10-30): anchor offset unfiltered, see vr_arms::publish
// [2026-10-31 audit, research/IK_PIPELINE_AUDIT_20261031.md] One order for every hand state, on foot and mounted: the sample of the
// eye step -> the body of the camera step -> one guarded wrist target -> the gun in cell space -> gear rigid on the finished bones.
// rig_coherent 0 / accessory_rigid 0 in [arms] restore the previous behaviour.
bool rig_coherent=true;
bool accessory_rigid=true;
// [HEADSET 2026-11-08] the held bow's shadow does not follow the tracked bow. [LIVE, WEAPONS_AND_HANDS 2026-10-15] each held weapon
// is drawn twice a frame: the main draw at the placed socket (gap < 0.2 m) and a second draw ~0.31-0.35 m off; [LIVE 2026-11-08]
// arrows likewise (d_ik 0 and ~0.4 m). INFERENCE: the second draw is the shadow caster's copy. On: a second in-hand draw (0.2-0.8 m)
// of the same mesh resource is carried rigidly onto this frame's main draw. shadow_weapon_follow 0 in [arms] restores it.
bool shadow_weapon_follow=true;
// [HEADSET 2026-11-08] shadow pieces float near the shadow's arms/head and move with the tracked arms, while nothing is seen there in the
// headset. [LIVE] the full body RNF_BODY_01_M (only seen as the shadow) and the torso mesh each had 10 bones without an FP name and
// without a body-part name pinned rigidly to an arm (anchor radius 0.25 m); INFERENCE: those are body pieces near the shoulder (not
// gear), carried with the arm. On: in skeleton meshes (full body, or >= 300 bones) only *accessory* bones may anchor to an arm; the
// rest keep the game's body pose. skeleton_anchor_strict 0 in [arms] restores the previous behaviour.
// [HEADSET 2026-11-08 n] no change with it on; [LIVE] the bones were only L/R_upperArm_expand helpers: default off (old behaviour).
bool skeleton_anchor_strict=false;
bool shadow_gear_follow=true;  // see the body arm shift in accessory_call
// Share of the full body's arm shift toward its own shoulders (1 = since 2026-10-15). 0: the shadow's arms are the tracked arms exactly
// (its shoulders stretch toward the FP shoulders); test value for the shadow blob of the visible arm gear (2026-11-08 p).
float body_arm_shift=1.f;
std::atomic<unsigned> body_arm_shift_mm[2]{};  // read-only: largest body arm-chain shift (own upperArm - IK upperArm) per side, 5 s
// [AUTHOR 2026-11-07] riding must hold like on foot. On foot: one solve per frame for every mesh, and every gun part drawn where the entity
// was placed. Riding (SOURCE): each draw chose its own input and body, gear took the draw-time body while the body arms took the solve's,
// and gun palettes followed the live palm while the entity-placed parts stayed at the commit. mounted_parity 0 in [arms] restores that.
bool mounted_parity=true;
// Seventeenth step (2026-11-08, AUTHOR asked after the Cyberpunk VR port comparison): one rider solve per game tick, made once and replayed
// to every draw of that tick (their animation-batch clock), the gun placement included. Off: the per-camera-step latch (steps 8-16).
bool rider_tick_solve=true;
std::atomic<unsigned> rider_tick_predicted{0},rider_tick_turned{0}; std::atomic<int> rider_tick_turn_cdeg{0};  // provisional samples moved by the camera offset's own velocity
std::atomic<unsigned> rider_tick_frozen{0};  // provisional solves kept because a draw of that tick already used them
std::atomic<unsigned> rider_tick_exact{0},rider_tick_provisional{0},rider_tick_refreshed{0},rider_gun_hand_led{0},rider_gun_hand_far{0};
std::atomic<unsigned> rider_tick_made{0},rider_tick_hits{0},rider_tick_newest{0},rider_tick_gun_made{0},rider_tick_gun_shared{0},rider_tick_stationary{0};
std::atomic<unsigned> rider_commit_latches{0},rider_commit_held{0};  // mounted_parity: latches on the gun's input / of those, on an earlier frame's placement
std::atomic<unsigned> rider_grip_holds{0};  // mounted_parity: rider inputs whose lost controller grip was held from its last tracked pose
std::atomic<unsigned> rider_prelatches{0};  // mounted_parity: draws whose mesh root showed the next camera step before it was published
std::atomic<unsigned> rider_latches{0},rider_relatches{0},rider_input_splits{0},rider_body_drift_mm{0},rider_palm_commit_mm{0};
// Per-mesh moment (2026-11-07, second parity step): meshes drawn at another tick than the solve, and how far.
std::atomic<unsigned> rider_mesh_moments{0},rider_mesh_moment_mm{0},rider_gun_rooted{0},rider_gun_unrooted{0};
std::atomic<bool> rider_gun_follows_mesh{false};
// Third step: the tick of each mesh found from its root, and the yaw it is turned by.
std::atomic<unsigned> rider_tick_matched{0},rider_tick_unmatched{0},rider_mesh_yaw_cdeg{0};
constexpr bool kRiderMeshMoment=false;  // steps 9-10 (each mesh at its own tick): measured, not applied (fourth parity step)
// [AUTHOR 2026-11-07] a right-arm accessory stays at the flat position while riding. [LIVE] G_UTILITY_TORSO_03B (rooted at the hand, 1.16 m
// from the body) has no arm bones, so the rider path drew the game's pose. On foot item_call binds it once, rigidly, to its nearest arm bone.
struct RiderItemBind {const void* resource=nullptr; unsigned n=0,generation=0; int side=-1,role=-1; std::vector<float> local;};
std::vector<RiderItemBind> rider_item_binds;  // rider_mutex
std::atomic<unsigned> rider_item_apps{0},rider_item_misses{0};  // the last gun palette took its own tick's body: the gun hand may take its mesh's too
bool bare_hand_static=true;  // bare_hand_static 0 in [arms]: empty hands keep the game's finger animation again
std::mutex item_bind_mutex;
struct ItemBind {int bone=-1; std::shared_ptr<const std::vector<float>> local;};
std::unordered_map<std::uint64_t,ItemBind> item_binds;  // key: resource, bone count
std::atomic<unsigned> rig_align_fails{0},rig_align_logs{0},rig_gun_holds{0},rig_body_stale{0},rig_antipodal{0},rig_twist_wraps{0};
std::mutex rig_body_mutex; float rig_body[3]{}; long long rig_body_us=0;
// The entity hook runs in this tick before its camera step, so the stored body is the last tick's: it reads the live one.
thread_local bool t_rig_body_live=false;
std::atomic<unsigned> rig_camera_tick{0},mounted_held_last{0},mounted_arm_why[6]{};
// Every way a player mesh leaves the override and draws the game's flat pose for a frame: fp_root_far, gear_far, pose_changed, solve_failed.
std::atomic<unsigned> rig_fallback[4]{};
std::atomic<unsigned> rig_stale_mm_max{0};
int rest_anchor_mode=-1;  // rest_mutex: 0 camera bone, 1 root, 2 root axes + camera position
std::atomic<unsigned> jump_logs{0},cm_fallbacks{0},tick_view_offsets{0};
struct CleanHandCache { vr_arms_math::HandLocal local; float navi_length=0.f; float shoulder_root[3]{}; bool view_anchor=false; bool valid=false; };
CleanHandCache clean_cache[2];  // clean_mutex: the hand layout, Na'vi arm length and the shoulder (root-local, no head shift)
std::mutex clean_mutex;
struct GunRecord { long long us=0; float flat[16]{}; float ik[16]{}; float root[16]{}; float view_offset[3]{}; float local[3]{}; std::int16_t cell[3]{}; bool has_local=false; vr_arms::Sample sample; bool valid=false; };
GunRecord gun_record;  // clean_mutex: socket placed this game tick (entity hook)
std::mutex armed_trace_mutex;
FILE* armed_trace=nullptr; long long armed_trace_start=0,armed_trace_next=0;
bool armed_trace_done=false;
long long armed_trace_body_us=0; float armed_trace_body[3]{};
extern std::wstring dir;  // initialized by on_present before the palette hook is enabled
thread_local vr_arms::Sample t_clean; thread_local bool t_clean_ok=false;  // the sample of the running solve
thread_local float t_clean_view[3]{};  // the anchor offset of the running solve: the placed gun's when a gun is placed
// The newest solve's sample and anchor, for the HUD: palette jobs run on many threads, and the thread-locals above keep an old solve on any other one.
std::mutex hud_solve_mutex; vr_arms::Sample hud_solve_sample; float hud_solve_view[3]{}; bool hud_solve_ok=false;
// [LIVE 2026-10-30] a controller still and the shoulder steady, yet a free wrist target jumps 5-12 cm for one frame and
// returns (source of the jump OPEN). Such a frame keeps the last wrist relative to the shoulder; a real change passes on
// the next generation, so the hold can never last.
struct HandHold { float local[16]{}; float sample[16]{}; unsigned gen=0; bool oriented=false,held=false,have=false; };
HandHold hand_hold[2];  // clean_mutex
std::atomic<unsigned> hand_holds{0},hand_hold_logs{0};
// [LIVE 2026-10-30] the game's hand pose still reaches the IK: knuckles and the gun attach bone (relative to the wrist) set
// the wrist orientation, the palm offset and the gun hold; the attach bone roamed up to 197 cm from the wrist. Every hand
// state goes through this: the hand pose is frozen while the player moves, jumps or turns, follows slowly while still, and
// quickly for 1 s after a weapon change, so no locomotion or action animation can steer the wrist, gun or support hold.
struct HandFrame { float smooth[211][16]{}; float fast_knuckle[3][16]{}; float bare_pose[211][16]{}; bool bare_ready=false; ULONGLONG smooth_tick=0,report_tick=0,bare_tick=0; unsigned hold_gen=0,smooth_gen=0; float attach_cm=0.f,knuckle_cm=0.f; bool held=false,have=false; };
HandFrame hand_frame[2];  // rest_mutex
std::atomic<unsigned> hand_frame_holds{0},hand_frame_logs{0};
bool hand_freeze_moving=true;  // "Freeze hand animation while moving"
thread_local bool hand_layout_open=true,hand_layout_changing=false;  // this solve: pose may update / a weapon just changed
const void* hand_layout_weapon=nullptr; ULONGLONG hand_layout_window=0; bool hand_layout_primed=false; float last_body_yaw=0.f;  // rest_mutex
ULONGLONG hand_layout_placed_at=0; bool hand_layout_bare=false;  // rest_mutex: last gun placement seen by the solve; no gun for 0.5 s
std::atomic<unsigned> clean_solves{0},clean_gun_places{0},clean_gun_reuses{0},rig_reuse{0},flat_frames{0},
  hand_jump_still{0},inputs_mismatch{0};
float jump_last_point[2][3]{}; float jump_last_sample[2][3]{}; float jump_last_body[3]{}; bool jump_have[2]{};  // clean_mutex
long long clean_qpc_us() {static LARGE_INTEGER f{}; if(!f.QuadPart) QueryPerformanceFrequency(&f); LARGE_INTEGER n; QueryPerformanceCounter(&n);
  return static_cast<long long>(n.QuadPart/f.QuadPart*1000000+(n.QuadPart%f.QuadPart)*1000000/f.QuadPart);}
// One bounded, read-only capture per process. Pose rows are emitted by the palette consumer, so the
// sample age and frame identity describe the hand actually offered for skinning, not merely an XR poll.
struct IkTraceRow {
  const char* phase=""; int hand=-1; unsigned gen=0; const vr_arms::Sample* sample=nullptr;
  const void* object_id=nullptr; float body[3]{},shoulder[3]{},head_used[3]{},free_target[3]{},final_target[3]{},object[3]{};
  float gap=0.f,rotation_trace=0.f,source_delta=0.f,source_rotation_trace=0.f;
  unsigned long long gun_seq=0,socket_age_ms=0;
  std::uint64_t xr_seq=0; std::int64_t display_time=0;
};
extern bool hand_smoothing;
std::mutex ik_trace_mutex;
FILE* ik_trace_file=nullptr;
long long ik_trace_start_us=0,ik_trace_flush_us=0;
unsigned ik_trace_rows=0;
std::atomic<bool> ik_trace_done{false};
std::atomic<bool> ik_trace_active{false};
std::atomic<bool> ik_trace_requested{false}; // opt-in scripts/AvatarVR_iktrace.txt: capture
std::atomic<bool> ik_sync_head{false}; // experimental "capture sync_head"; false in ordinary play
std::atomic<bool> ik_trace_weapon_start{false}; // "capture weapon": begin when the clean gun socket is placed
std::atomic<bool> ik_frame_sample_lock{false}; // "capture weapon_lock": A/B one input sample per Present generation
std::mutex ik_frame_sample_mutex;
unsigned ik_frame_sample_gen=0;
vr_arms::Sample ik_frame_sample;
bool ik_frame_sample_valid=false;
bool clean_input_sample(vr_arms::Sample& out) {
  if(!ik_frame_sample_lock.load(std::memory_order_relaxed)) return vr_arms::latest(out);
  const unsigned gen=frame_gen.load(std::memory_order_acquire);
  std::lock_guard<std::mutex> lock(ik_frame_sample_mutex);
  const long long us=clean_qpc_us();
  if(ik_frame_sample_valid && ik_frame_sample_gen==gen && us-ik_frame_sample.sampled_qpc_us>=0 &&
     us-ik_frame_sample.sampled_qpc_us<50000) {
    out=ik_frame_sample; return true;
  }
  if(!vr_arms::latest(out)) return false;
  ik_frame_sample=out; ik_frame_sample_gen=gen; ik_frame_sample_valid=true;
  return true;
}
ULONGLONG ik_trace_control_next=0; // Present thread only
void ik_trace(const IkTraceRow& row) {
  if(dir.empty() || !ik_trace_requested.load(std::memory_order_acquire)) return;
  const long long us=clean_qpc_us();
  std::lock_guard<std::mutex> lock(ik_trace_mutex);
  if(ik_trace_done.load(std::memory_order_acquire)) return;
  if(!ik_trace_start_us) {
    if(std::strcmp(row.phase,ik_trace_weapon_start.load(std::memory_order_relaxed)?"place":"solve")!=0) return;
    ik_trace_file=_wfopen((dir+L"\\IK_MOTION_TIMING.csv").c_str(),L"w");
    if(!ik_trace_file) {ik_trace_done.store(true,std::memory_order_release); return;}
    std::setvbuf(ik_trace_file,nullptr,_IOFBF,128*1024);
    std::fprintf(ik_trace_file,"# build=%s_%s pid=%lu capture_limit_s=180 row_limit=40000 frame_lock=%d\n",
      __DATE__,__TIME__,static_cast<unsigned long>(GetCurrentProcessId()),
      ik_frame_sample_lock.load(std::memory_order_relaxed)?1:0);
    std::fputs("t_ms,phase,gen,hand,sample_seq,sample_age_ms,smoothing,view_valid,sync_head,body_x,body_y,body_z,shoulder_x,shoulder_y,shoulder_z,head_used_x,head_used_y,head_used_z,sample_head_x,sample_head_y,sample_head_z,raw_grip_x,raw_grip_y,raw_grip_z,filtered_grip_x,filtered_grip_y,filtered_grip_z,raw_view_x,raw_view_y,raw_view_z,filtered_view_x,filtered_view_y,filtered_view_z,free_x,free_y,free_z,final_x,final_y,final_z,object_x,object_y,object_z,gap_m,rotation_trace,object_id,placement_tick,xr_seq,display_time,source_delta_m,source_rotation_trace,gun_seq,socket_age_ms\n",ik_trace_file);
    ik_trace_start_us=us;
    ik_trace_active.store(true,std::memory_order_release);
  }
  if(us-ik_trace_start_us>=180000000 || ik_trace_rows>=40000) {
    std::fflush(ik_trace_file); std::fclose(ik_trace_file); ik_trace_file=nullptr;
    ik_trace_done.store(true,std::memory_order_release);
    ik_trace_active.store(false,std::memory_order_release); return;
  }
  const vr_arms::Sample* s=row.sample;
  const bool hand=s && row.hand>=0 && row.hand<2;
  const long long age=s && s->sampled_qpc_us ? us-s->sampled_qpc_us : -1;
  std::fprintf(ik_trace_file,"%.3f,%s,%u,%d,%llu,%.3f,%d,%d,%d",
    (us-ik_trace_start_us)*0.001,row.phase,row.gen?row.gen:frame_gen.load(std::memory_order_relaxed),row.hand,
    s?s->seq:0ull,age<0?-1.f:age*0.001f,hand_smoothing?1:0,s&&s->view_valid?1:0,
    ik_sync_head.load(std::memory_order_relaxed)?1:0);
  auto vec=[&](const float* p) {for(int k=0;k<3;++k) std::fprintf(ik_trace_file,",%.6f",p?p[k]:0.f);};
  vec(row.body); vec(row.shoulder); vec(row.head_used); vec(s&&s->head_world_valid?s->head_world:nullptr);
  vec(hand?s->raw_local[row.hand]+12:nullptr);
  vec(hand?s->local[row.hand]+12:nullptr); vec(s?s->raw_view_offset:nullptr);
  vec(s?s->view_offset:nullptr); vec(row.free_target); vec(row.final_target); vec(row.object);
  std::fprintf(ik_trace_file,",%.6f,%.6f,%p,%u,%llu,%lld,%.6f,%.6f,%llu,%llu\n",row.gap,row.rotation_trace,row.object_id,
    vr_weapon::placement_tick(),static_cast<unsigned long long>(row.xr_seq),static_cast<long long>(row.display_time),
    row.source_delta,row.source_rotation_trace,row.gun_seq,row.socket_age_ms);
  ++ik_trace_rows;
  if(us-ik_trace_flush_us>=1000000) {std::fflush(ik_trace_file); ik_trace_flush_us=us;}
}
void trace_armed_target(unsigned hand, unsigned long long sample_seq, unsigned long long gun_seq,
                        long long gun_age_us, unsigned long long socket_age_ms,
                        const float* free_pos, const float* pinned_pos, const float* socket_pos,
                        const float* body_pos, const float* view_offset) {
  const long long us=clean_qpc_us();
  std::lock_guard<std::mutex> lock(armed_trace_mutex);
  if(armed_trace_done || us<armed_trace_next) return;
  if(!armed_trace_start) {
    if(!armed_trace_body_us) {std::memcpy(armed_trace_body,body_pos,12); armed_trace_body_us=us; return;}
    if(us-armed_trace_body_us<100000) return;
    float movement2=0.f; for(int k=0;k<3;++k) {const float d=body_pos[k]-armed_trace_body[k]; movement2+=d*d;}
    std::memcpy(armed_trace_body,body_pos,12); armed_trace_body_us=us;
    if(movement2<0.015f*0.015f) return;  // begin on locomotion, not when a gun is first equipped
    armed_trace_start=us;
    const std::wstring path=dir+L"\\ARMED_TARGET_TIMING.csv";
    armed_trace=_wfopen(path.c_str(),L"w");
    if(!armed_trace) {armed_trace_done=true; return;}
    std::fputs("ms,hand,sample_seq,gun_seq,gun_age_ms,socket_age_ms,free_x,free_y,free_z,pinned_x,pinned_y,pinned_z,socket_x,socket_y,socket_z,body_x,body_y,body_z,view_x,view_y,view_z\n",armed_trace);
  }
  armed_trace_next=us+14000;  // at most ~72 samples/s, one 25 s window per launch
  std::fprintf(armed_trace,"%.3f,%u,%llu,%llu,%.3f,%llu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
               (us-armed_trace_start)*0.001f,hand,sample_seq,gun_seq,gun_age_us*0.001f,socket_age_ms,
               free_pos[0],free_pos[1],free_pos[2],pinned_pos[0],pinned_pos[1],pinned_pos[2],
               socket_pos[0],socket_pos[1],socket_pos[2],body_pos[0],body_pos[1],body_pos[2],
               view_offset[0],view_offset[1],view_offset[2]);
  if(us-armed_trace_start>=25000000) {std::fclose(armed_trace); armed_trace=nullptr; armed_trace_done=true;}
}
bool read_marker90(float* m) {
  const game::Pointers mp=game::pointers(); const unsigned char* data=nullptr; std::int32_t count=0;
  return mp.pose && game::mem_read(mp.pose,&data,sizeof(data)) && game::mem_read(mp.pose+8,&count,sizeof(count)) && data &&
         count==90 && game::mem_read(data+1*64,m,64);
}
bool fp_root_now(float* out16,const float* body_position,const std::int16_t* cell) {
  float marker_then[16]; bool have_then=false;
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    if(!fp_root_valid) return false;
    std::memcpy(out16,fp_root_latest,64);
    have_then=fp_root_marker_valid; if(have_then) std::memcpy(marker_then,fp_root_marker,64);}
  float marker_now[16],inverse_then[16],delta[16];
  if(root_marker_carry && have_then && read_marker90(marker_now) && skin_arm_ik::inverse_affine(inverse_then,marker_then)) {
    // positions: same cell rounding against the stored Marker (only rotation is used)
    for(int k=0;k<3;++k) marker_now[12+k]+=std::round((marker_then[12+k]-marker_now[12+k])/1024.f)*1024.f;
    skin_arm_ik::multiply_affine(delta,inverse_then,marker_now); delta[12]=delta[13]=delta[14]=0.f;
    const float c=(delta[0]+delta[5]+delta[10]-1.f)*0.5f;
    if(c>0.9f) {  // < ~25 deg since the last FP draw; otherwise keep the stored rotation
      float rotated[16]; float base[16]; std::memcpy(base,out16,64); base[12]=base[13]=base[14]=0.f;
      skin_arm_ik::multiply_affine(rotated,base,delta);
      for(int i=0;i<12;++i) out16[i]=rotated[i];
      root_carries.fetch_add(1,std::memory_order_relaxed);
    }
  }
  for(unsigned k=0;k<3;++k) out16[12+k]=body_position[k]-1024.f*cell[k];
  return true;
}
// [LIVE] 2026-10-15 census: the shadow-casting full body RNF_BODY_01_M (424 bones) and head P_HEAD_10_M (899) are
// skinned here too; the gear path now takes up to 1024 bones so the full body's arm bones follow the IK (shadow).
constexpr unsigned kMaxGearBones=1024;
struct AccessoryMap {const void* mesh=nullptr; unsigned n=0; ULONGLONG checked=0; unsigned matched=0; unsigned anchored=0; unsigned body=0;
  bool body_mesh=false; int upper_index[2]={-1,-1}; // full body: its own upperArm bones (chain shift to its own shoulders)
  bool weapon=false; int held=-2;                   // weapon mesh (/WEAPON/ path); held: -2 unknown, -1 holstered, 0 left, 1 right
  bool item=false;                                  // player utility item (/GEAR/PLAYER/UTILITY/): follows its nearest hand bone
  std::int16_t to_fp[1024]; std::int16_t anchor[1024]; // anchor: unmapped bone -> nearest mapped bone of this mesh
  std::int16_t anchor_fp[1024]; unsigned fp_anchored=0; // fallback: unnamed bone -> nearest FP arm bone (FP raw pose)
  std::int16_t group[1024];};  // anchored accessory part -> its charm group's representative bone (-1: on its own)
// Distinct mesh paths seen near the player while IK runs (logged once each, with the palette caller).
std::vector<std::string> seen_paths;
std::mutex seen_paths_mutex;
// FP bone names (read_sides) for name-mapping accessory bones; written once before side_state becomes 2.
std::string fp_names[211];

AccessoryMap accessory_maps[128];
// Meshes not rooted at the player body (NPCs, props): decided once whether they are the player's held weapon, so they
// never enter accessory_maps. [LIVE/AUTHOR] 2026-10-15: NPC maps filled the 48-slot cache, evicted the player's gear
// maps and forced rebuilds, and the rebuilt frames briefly showed the flat pose (left-arm flicker near characters).
std::unordered_map<const void*, bool> far_meshes;
std::mutex far_mutex;
std::mutex accessory_mutex;
std::atomic<unsigned> accessory_logged{0},accessory_meshes{0};
struct AccessoryRest {std::shared_ptr<const std::vector<float>> local; bool taken_still=false;};
std::mutex accessory_rest_mutex;
std::unordered_map<std::uint64_t,AccessoryRest> accessory_rest;  // key: resource, bone count; survives transient draw meshes
std::atomic<bool> rig_player_still{false};
bool shoulders_follow=true; // FP rig follows the headset's 6DoF offset from the game camera
skin_arm_ik::ArmScale arm_scale=make_default_scale();                 // menu "Arm scaling" + user measurements (persisted)
std::atomic<bool> calibrate_request{false};      // menu "Calibrate arm length"
std::atomic<int> scale_milli[2]{};               // last factor per hand (status log)
std::mutex pole_mutex;
skin_arm_ik::Vec last_bend[2]{};                 // last solved elbow bend direction per hand (pole continuity)
// Hand on the gun (2026-10-15, run 21): the controller-driven weapon socket (cell-local, its cell) the game should place
// the gun with, and the attachment bone indices (set by weapon_call from the names).
float weapon_hand_setting=1.f; // persisted "weapon_hand" (legacy, read once at load: 2 = left)
// Left-handed weapon mode (run 59, menu "Weapon hand"): the gun is placed at the LEFT controller with a right-hand grip
// (clean_place), only the right-handed mode pins the hand to the gun socket, the triggers swap (xr_input) and the aim
// script follows the weapon hand (avr.weapon_hand()). Off = right-handed (default).
bool left_handed=false;
// run 63: name partners (L_x <-> R_x) of the 211 FP pose, built once from the names; -1 = none.
int fp_partner[211]; bool fp_partner_ready=false;
void build_partners() {
  if(fp_partner_ready) return;
  for(int& p:fp_partner) p=-1;
  unsigned pairs=0;
  for(unsigned j=0;j<211;++j) {const std::string& a=fp_names[j]; if(a.size()<3 || a.compare(0,2,"L_")!=0) continue;
    const std::string b="R_"+a.substr(2);
    for(unsigned k=0;k<211;++k) if(fp_names[k]==b) {fp_partner[j]=static_cast<int>(k); fp_partner[k]=static_cast<int>(j); ++pairs; break;}}
  fp_partner_ready=pairs>20;
}
std::atomic<unsigned> finger_swaps{0},left_pins{0};
// ---- Smooth hand swap (run 65): the gun travels from one hand's hold to the other's and the hand poses cross-fade over
// kSwapMs, driven by one eased "left amount" (0 = right-handed, 1 = left-handed) that the gun placement, the pin and the
// finger-pose swap all read. Fire/reload/aim switch hand (left_handed) when the transition ends.
constexpr long long kSwapUs=350000;
std::atomic<long long> swap_start_us{0};
std::atomic<int> swap_to{-1};  // 0 left, 1 right, -1 none
std::atomic<unsigned> hand_swaps{0};
// ---- Two-hand aiming (run 67). The support hold is NATIVE: the game's FP animation keeps the left hand on each weapon's
// foregrip/pistol grip, and both weaponAttachmentHand bones are the gun socket frame (run 62), so
// H_sup = raw[left wrist] * inverse(raw[attach]) is the per-weapon support hold, read live each solve. Left mode uses the
// mirrored pair (mirror_hold). Grab = the support hand's grip near the support point (nearest point wins vs the handle
// = hand swap). The gun swings about the dominant grip toward the support controller with Cyberpunk-port leverage rules;
// a grab offset decays to 0 so a grab from a distance never snaps; one eased weight drives the gun and the hand slide.
bool two_hand_enabled=true;
bool mounted_weapon_menu=true;
std::atomic<bool> mounted_weapon_enabled{true};
struct SupportHold { float h_sup_l[16]{}; float h_dom_r[16]{}; float baseline=0.f, leverage=0.f; bool valid=false; };
SupportHold support_hold;  // clean_mutex
long long support_hold_us=0;  // diagnostic freshness only; clean_mutex
struct TwoHandState { int support=-1; bool held=false; long long start_us=0, release_us=0; float w_release=0.f; float offset_gun[3]{}; };
TwoHandState two_hand;     // clean_mutex
float th_d_support[2]={1e9f,1e9f},th_d_handle[2]={1e9f,1e9f},th_offset_gun[2][3]{}; long long th_measured_us=0;  // clean_mutex
std::atomic<unsigned> two_hand_grabs{0},two_hand_pins{0}; std::atomic<int> two_hand_w_milli{0},two_hand_lev_milli{0},two_hand_deg_milli{0};
constexpr long long kTwoHandInUs=200000, kTwoHandOutUs=80000, kTwoHandOffsetUs=250000;
// Run 68 "support-hand only": while two-handed, the trigger hand's grip press lets that hand go; the gun eases into the
// support hand alone (at the game's support hold, socket = inverse(H_sup) * support wrist) and cannot fire. The trigger
// hand's grip press back at the handle re-grabs; the support hand letting go returns the gun to the trigger hand.
// so = the eased "held by the support hand only" amount (0..1), driving the gun placement, the trigger hand's pin and
// the fire block together.
constexpr long long kSupportOnlyUs=250000;
bool so_target=false; float so_from=0.f; long long so_change_us=0;  // clean_mutex
float th_d_dom_handle=1e9f;                                          // clean_mutex: trigger-hand controller to the handle
std::atomic<unsigned> support_only_entries{0},support_only_regrabs{0};
float support_only_locked(long long now) {  // caller holds clean_mutex
  if(!so_change_us) return so_target?1.f:0.f;
  const float e=vr_arms_math::smoothstep(static_cast<float>(now-so_change_us)/kSupportOnlyUs);
  return so_from+((so_target?1.f:0.f)-so_from)*e;
}
void set_support_only_locked(bool on, long long now) {
  if(so_target==on) return;
  so_from=support_only_locked(now); so_target=on; so_change_us=now;
}
float two_hand_weight_locked(long long now) {  // caller holds clean_mutex
  if(two_hand.held) return vr_arms_math::smoothstep(static_cast<float>(now-two_hand.start_us)/kTwoHandInUs);
  if(two_hand.release_us && now-two_hand.release_us<kTwoHandOutUs)
    return two_hand.w_release*(1.f-vr_arms_math::smoothstep(static_cast<float>(now-two_hand.release_us)/kTwoHandOutUs));
  return 0.f;
}
bool holds_for(int dom, const SupportHold& sh, float* h_sup, float* h_dom) {
  if(!sh.valid) return false;
  if(dom==1) {std::memcpy(h_sup,sh.h_sup_l,64); std::memcpy(h_dom,sh.h_dom_r,64);}
  else {vr_arms_math::mirror_hold(sh.h_sup_l,h_sup); vr_arms_math::mirror_hold(sh.h_dom_r,h_dom);}
  return true;
}
long long swap_now_us() {static LARGE_INTEGER f{}; if(!f.QuadPart) QueryPerformanceFrequency(&f); LARGE_INTEGER n; QueryPerformanceCounter(&n);
  return static_cast<long long>(n.QuadPart/f.QuadPart*1000000+(n.QuadPart%f.QuadPart)*1000000/f.QuadPart);}
float left_amount() {
  const int to=swap_to.load(std::memory_order_acquire);
  if(to<0) return left_handed?1.f:0.f;
  const float t=static_cast<float>(swap_now_us()-swap_start_us.load(std::memory_order_acquire))/static_cast<float>(kSwapUs);
  if(t>=1.f) {
    int expected=to;
    if(swap_to.compare_exchange_strong(expected,-1)) left_handed=to==0;  // the transition ends: the weapon hand flips
    return to==0?1.f:0.f;
  }
  const float e=vr_arms_math::smoothstep(t);
  return to==0?e:1.f-e;
}
float fresh_attach[2][16]{}; std::int16_t fresh_cell[2][3]{}; bool fresh_valid[2]{};
float fresh_grip[2][16]{},fresh_grip_cam[2][16]{}; bool fresh_grip_valid[2]{};  // fresh_mutex: controller sample + its camera (run 52)
float fresh_body[2][3]{};      // body position (world) of the solve that produced fresh_attach
float fresh_cam[2][16]{}; bool fresh_cam_valid[2]{};  // game camera (world) the grips of that solve were built in
float fresh_root[2][16]{};    // FP mesh root (world) of the solve that produced fresh_attach (model-space hands)
// [LIVE run 29 trace] 37 single-frame 12-18 cm hand-vs-camera spikes while running at 8-11 m/s = one frame of travel:
// the weapon placement runs where game::read().cam is sometimes this frame's camera and sometimes the previous one.
// The camera is snapshotted with the body position right after each camera update (game thread); at placement the
// camera is estimated as that snapshot moved by the body travel since.
std::mutex cam_post_mutex; float cam_post[16]{}; float cam_post_body[3]{}; bool cam_post_valid=false;
std::atomic<unsigned> cam_estimates{0};
// Menu "Weapon hand trail" (run 26): the held gun (and the hand drawn on it) trails the body by the solve->game delay;
// this share of the body travel since the solve is added back (0 = full trail, 1 = none). Persisted.
float weapon_trail_comp=0.8f;
std::mutex fresh_mutex;
int attach_bone[2]={-1,-1};    // FP indices of L/R_weaponAttachmentHand (from the names)
std::atomic<unsigned> hand_on_gun{0},hand_on_gun_root{0};
// [HEADSET run 22] walking now stays attached, turning with the stick does not: the game adds its own motion (sway) to
// the gun after our placement. The hand therefore follows the gun's actual root bone, read from the game's weapon pose
// array of the in-hand draw, through the root->socket offset measured while still.
const void* gun_source=nullptr; std::int16_t gun_cell[3]{}; float gun_root_in_socket[16]{}; bool gun_offset_valid=false;
ULONGLONG gun_source_since=0;  // [HEADSET run 25] offset differed after weapon switches: no calibration for 600 ms after a change
// [LIVE run 29] hand_on_gun=6802/4970: ~27% of frames fell back from the gun's actual root to the placed socket (up to
// ~7 cm apart under sway) because the gun is drawn twice and a single offset slot was reset whenever the draw swapped:
// the weapon arm flickered between two targets. One offset per gun draw; the hand follows the draw nearest the socket.
struct GunSlot { const void* src=nullptr; float offset[16]; bool valid=false; std::int16_t cell[3]{}; ULONGLONG since=0,seen=0; };
GunSlot gun_slots[4];
// [HEADSET run 37] turning with the stick: grip and gun separate and flicker. The hand read the gun root from the pose
// buffer at solve time, which during fast turns can be another frame's pose than the one drawn. Like the Cyberpunk port
// (arm solve and weapon bones in the same pose pass), the weapon hand now uses only the gun socket recorded at the gun's
// own draw in the SAME frame (frame_gen); otherwise the placed socket, never a possibly stale buffer read.
float gun_actual_world[16]{}; unsigned gun_actual_gen=0;  // gun_mutex
std::atomic<unsigned> gun_same_frame{0},gun_other_frame{0},fp_after_gun{0},fp_before_gun{0};
FILE* trace_file=nullptr; ULONGLONG trace_start=0; bool trace_done=false;  // WEAPON_TRACE.csv, 25 s once per launch
std::mutex gun_mutex;
bool last_bend_valid[2]{};
float upper_twist_share=0.3f;                    // share of the wrist roll given to the upper-arm twist bones
bool hand_smoothing=true;                        // menu "Hand smoothing": One-Euro filter on the grips in the body frame
bool accessory_physics=true;                     // menu "Accessory physics": mod-side spring on anchored charms/cords
std::mutex dangle_mutex;
std::unordered_map<std::uint64_t,skin_arm_ik::Dangle> dangles;  // key: mesh pointer, bone index
LARGE_INTEGER dangle_last{};
skin_arm_ik::OneEuro grip_filters[2][12];
LARGE_INTEGER filter_last[2]{};
std::mutex filter_mutex;
// Stable shoulders: the FP rig is re-posed from a rest pose captured relative to the camera bone (1 CameraMarker) while
// the player stands still, so running/weapon animation no longer moves the shoulders relative to the view (the camera
// bone still carries crouching). Shoulder height/forward shift the whole rig in root axes (metres).
bool stable_rig=true;
// [AUTHOR/HEADSET] 2026-10-15: with "Stable shoulders" the whole 211-bone rest pose replaced the live pose, so the hands
// froze in the grip of the capture moment (no shotgun/other weapon hold poses). On: the hand subtree (fingers, weapon
// attachment, hand props) keeps the game's live animation relative to the wrist; the rest pose still holds the
// shoulders/arms. Off: the captured (frozen) hand pose, kept for future controller finger gestures.
bool native_hand_poses=true;
float shoulder_height=0.16f,shoulder_forward=0.f; // [AUTHOR] 2026-10-15 defaults
float shoulder_spread=-0.16f; // menu "Shoulder separation": each side's arm chain moves +-spread/2 along the body's right axis
float rest_rel[211][16];
bool have_rest=false;
float last_body[3]{};
ULONGLONG last_body_tick=0,still_since=0;
std::mutex rest_mutex;
std::atomic<bool> rest_recapture{false};
std::atomic<unsigned> rest_captures{0};
// Accessory meshes (bracelets, wraps) skinned to the same 211-bone pose vector as the FP body: same source pointer.
std::atomic<const void*> fp_source{nullptr};
std::atomic<unsigned> fp_body_apps{0},accessory_apps{0},other211{0};
char other211_path[128]{};
std::atomic<bool> other211_logged{false};
int anatomy[2][3]={{-1,-1,-1},{-1,-1,-1}}; // per hand: fingerMiddle_01, fingerIndex_01, fingerPinky_01 (from names)
// Rig-independent support geometry learned from a verified on-foot held draw.
// Palm-to-palm cancels FP/TP weapon-root and wrist-axis conventions.
std::mutex mounted_profile_mutex;
using MountedProfile=mounted_weapon_profiles::Profile;
mounted_weapon_profiles::Cache mounted_support_profiles;
std::atomic<std::uint64_t> mounted_profile_revision{0};
std::atomic<unsigned> mounted_profile_disk_count{0},mounted_commit_failures{0};
struct ProfileBinding {const void* owner=nullptr; const void* weapon=nullptr; std::string path; ULONGLONG at=0;};
ProfileBinding mounted_profile_binding; // mounted_profile_mutex; verified on-foot held draw only
std::atomic<unsigned> mounted_finger_apps{0},mounted_finger_rejects{0},mounted_pin_expired{0},mounted_pin_distant{0};
std::atomic<unsigned> mounted_bound_carried{0},mounted_palette_no_frame{0};
std::atomic<unsigned> mounted_palette_adopted{0},mounted_palette_unbound{0},mounted_palette_not_ready{0},mounted_palette_invalid{0};
bool read_path(const void* root,char (&path)[128]);
bool read(const void* p,std::size_t off,void* dst,std::size_t size);
void mounted_learn_support(const void* root,const float (&raw)[211][16],const float* gun_root,const float* used_socket) {
  if(!mounted_weapon_enabled.load(std::memory_order_relaxed)) return;
  struct Learned {const void* resource=nullptr; std::string path;};
  static thread_local Learned learned[32]; static thread_local unsigned next_learned=0;
  const void* resource=nullptr; if(!read(root,0x58,&resource,sizeof(resource)) || !resource) return;
  const auto player=game::pointers(); const void* weapon=nullptr;
  if(!player.body || !read(player.body,0x250,&weapon,sizeof(weapon)) || !weapon) return;
  std::string key;
  for(const auto& item:learned) if(item.resource==resource) {key=item.path; break;}
  if(key.empty()) {char path[128]{}; if(!read_path(root,path)) return; key=path;}
  if(!mounted_weapon_profiles::path_valid(key)) return;
  const auto now=GetTickCount64();
  {std::lock_guard<std::mutex> lock(mounted_profile_mutex);
    if(mounted_support_profiles.count(key)) {
      mounted_profile_binding={player.body,weapon,key,now};
      auto& slot=learned[next_learned++%32]; slot={resource,key}; return;
    }
  }
  float palms[2][16],relative[16];
  for(int h=0;h<2;++h) {
    for(int k:anatomy[h]) if(k<0 || k>=211) return;
    if(!mounted_weapon_math::palm(raw[h?115:16],skin_arm_ik::position(raw[anatomy[h][0]]),
       skin_arm_ik::position(raw[anatomy[h][1]]),skin_arm_ik::position(raw[anatomy[h][2]]),palms[h])) return;
  }
  if(!mounted_weapon_math::relative(palms[0],palms[1],relative)) return;
  const float reach=skin_arm_ik::length(skin_arm_ik::position(relative));
  if(!(reach>.05f && reach<1.2f)) return;
  MountedProfile profile; std::memcpy(profile.relative,relative,64);
  float root_in_socket[16],palm_in_socket[16],inverse[16];
  if(attach_bone[1]<0 || !mounted_weapon_math::relative(gun_root,used_socket,root_in_socket) ||
     !mounted_weapon_math::relative(palms[1],raw[attach_bone[1]],palm_in_socket) ||
     !skin_arm_ik::inverse_affine(inverse,root_in_socket)) return;
  skin_arm_ik::multiply_affine(profile.trigger_root,palm_in_socket,inverse);
  std::memcpy(profile.fp_root_socket,root_in_socket,64);
  if(!mounted_weapon_math::rigid(profile.trigger_root)) return;
  for(int h=0;h<2;++h) {
    bool found[mounted_hand_pose::count]{};
    for(unsigned i=0;i<211;++i) if(fp_names[i].size()>2 && fp_names[i][0]==(h?'R':'L')) {
      const int node=mounted_hand_pose::index(fp_names[i].c_str()); if(node<0) continue;
      float local[16]; if(!mounted_weapon_math::relative(raw[i],palms[h],local)) continue;
      profile.fingers[h].point[node]=skin_arm_ik::position(local); found[node]=true;
    }
    profile.fingers[h].valid=true;
    for(bool valid:found) if(!valid) profile.fingers[h].valid=false;
  }
  profile.valid=true;
  if(!mounted_weapon_profiles::valid(profile)) return;
  const void* current_weapon=nullptr;
  if(mount_camera::rider_view() || game::pointers().body!=player.body ||
     !read(player.body,0x250,&current_weapon,sizeof(current_weapon)) || current_weapon!=weapon) return;
  std::lock_guard<std::mutex> lock(mounted_profile_mutex);
  // One native reference window per path across all palette workers. Worker
  // scheduling must not restart a settled grip or mix weapons into one window.
  static std::map<std::string,mounted_weapon_profiles::StableHold> pending;
  if(mounted_support_profiles.count(key)) return;
  if(!pending.count(key) && pending.size()>=mounted_weapon_profiles::max_profiles) return;
  auto& capture=pending[key];
  if(!capture.observe(player.body,weapon,profile,GetTickCount64())) return;
  if(mounted_support_profiles.size()>=mounted_weapon_profiles::max_profiles) return;
  if(mounted_support_profiles.emplace(key,capture.profile).second) mounted_profile_revision.fetch_add(1,std::memory_order_relaxed);
  pending.erase(key);
  mounted_profile_binding={player.body,weapon,key,now};
  learned[next_learned++%32]={resource,key};
}
std::atomic<bool> controller_mode{false};
struct GripCache {float matrix[2][16]{};bool valid[2]{};ULONGLONG tick=0;float body[3]{};bool body_valid=false;float head[3]{};bool head_valid=false;float yaw=0.f;bool yaw_valid=false;
  float cam[16]{};bool cam_valid=false;  // run 28: camera the grips were built in
  float map[16]{};bool map_valid=false;}; // run 35: rotation sample camera -> drawn frame camera (model-space hands)
GripCache grips;
std::mutex grip_mutex;
skin_arm_ik::WristCalibration wrist_calibration[2];
std::mutex wrist_mutex;
signed char bone_side[211]{};
signed char bone_role[211]{};
std::atomic<ULONGLONG> rig_log_next{0};
std::atomic<int> side_state{0}; // 0 unread, 1 reading, 2 ready, 3 failed
std::atomic<unsigned> follow_logged{0}; // bit per hand, first frame of each activation
std::atomic<bool> follow_enabled{true}; // "nofollow" in AvatarVR_skinik.txt keeps the three-joint-only behaviour for A/B.
struct Sample {
  const void *mesh,*header,*data,*remap,*body,*cpu90,*cpu192;
  unsigned count,map_count,thread,read_flags,name_count;
  char bone_name[64];
  int arms[6];
  float arm_positions[6][3];
  unsigned arm_read_mask,parent_read;
  const void* parent_candidate;
  std::uint64_t parent_words[2];
  std::uint16_t indices[16];
  float hand[3];
  std::int16_t cells[3];
};
Sample samples[64]{};
unsigned pending=0,total=0;
const void* seen_sources[64]{};
unsigned source_count=0;
ULONGLONG next_sample=0;
std::mutex mutex;
std::wstring dir;
ULONGLONG last_poll=0;
bool wanted_before=false,tried=false,installed=false;
void log(const char* line) {
  HANDLE f=CreateFileW((dir+L"\\SKIN_PALETTE_PROBE.log").c_str(),FILE_APPEND_DATA,FILE_SHARE_READ,nullptr,OPEN_ALWAYS,0,nullptr);
  if(f==INVALID_HANDLE_VALUE) return;
  DWORD n; WriteFile(f,line,static_cast<DWORD>(std::strlen(line)),&n,nullptr);
  WriteFile(f,"\r\n",2,&n,nullptr); CloseHandle(f);
}
// Present owns file I/O. Palette/shot callbacks only publish immutable numeric
// profiles under their mutex; no native pointers are persisted.
void mounted_profile_io() {
  static bool loaded=false; static std::uint64_t saved=0; static ULONGLONG retry_at=0;
  if(dir.empty()) return;
  const auto path=dir+L"\\AvatarVR_weapon_holds.txt";
  if(!loaded) {
    loaded=true; std::string text; mounted_weapon_profiles::Cache restored;
    HANDLE file=CreateFileW(path.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,0,nullptr);
    if(file!=INVALID_HANDLE_VALUE) {
      LARGE_INTEGER size{}; DWORD bytes=0;
      if(GetFileSizeEx(file,&size) && size.QuadPart>0 && size.QuadPart<=mounted_weapon_profiles::max_bytes) {
        text.resize(static_cast<std::size_t>(size.QuadPart));
        if(!ReadFile(file,text.data(),static_cast<DWORD>(text.size()),&bytes,nullptr)||bytes!=text.size()) text.clear();
      }
      CloseHandle(file);
      if(mounted_weapon_profiles::decode(text,restored)) {
        {std::lock_guard<std::mutex> lock(mounted_profile_mutex);
          for(const auto& item:restored) if(mounted_support_profiles.size()<mounted_weapon_profiles::max_profiles) mounted_support_profiles.emplace(item);
        }
        mounted_profile_disk_count.store(static_cast<unsigned>(restored.size()),std::memory_order_relaxed);
        char note[128]; std::snprintf(note,sizeof(note),"mounted weapon profiles: loaded %zu validated on-foot holds",restored.size()); log(note);
      } else log("mounted weapon profiles: invalid/incompatible cache ignored");
    }
  }
  const auto revision=mounted_profile_revision.load(std::memory_order_relaxed);
  if(revision==saved || GetTickCount64()<retry_at) return;
  mounted_weapon_profiles::Cache snapshot;
  {std::lock_guard<std::mutex> lock(mounted_profile_mutex); snapshot=mounted_support_profiles;}
  const auto text=mounted_weapon_profiles::encode(snapshot); if(text.empty()) return;
  const auto temporary=path+L".tmp";
  HANDLE file=CreateFileW(temporary.c_str(),GENERIC_WRITE,0,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
  bool success=false;
  if(file!=INVALID_HANDLE_VALUE) {
    DWORD bytes=0; success=WriteFile(file,text.data(),static_cast<DWORD>(text.size()),&bytes,nullptr) && bytes==text.size(); CloseHandle(file);
    if(success) success=MoveFileExW(temporary.c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING)!=FALSE;
  }
  if(success) {saved=revision; mounted_profile_disk_count.store(static_cast<unsigned>(snapshot.size()),std::memory_order_relaxed); log("mounted weapon profiles: saved validated holds/aim calibration");}
  else retry_at=GetTickCount64()+5000;
}
bool read(const void* p,std::size_t off,void* dst,std::size_t size) {
  return p && game::mem_read(static_cast<const unsigned char*>(p)+off,dst,size);
}
bool read_name(const void* root,unsigned index,Sample& s) {
  const void* header=nullptr; const void* data=nullptr;
  if(!read(root,0x50,&header,sizeof(header)) || !read(header,0,&data,sizeof(data)) ||
     !read(header,8,&s.name_count,sizeof(s.name_count)) || !data ||
     s.name_count>512 || index>=s.name_count) return false;
  unsigned char text[16]{};
  if(!read(data,index*16,text,sizeof(text))) return false;
  if(text[15]) {
    const void* heap=nullptr; std::memcpy(&heap,text,sizeof(heap));
    bool terminated=false;
    for(unsigned i=0;i<sizeof(s.bone_name)-1;++i) {
      if(!read(heap,i,s.bone_name+i,1)) return false;
      if(s.bone_name[i]==0) {terminated=true; break;}
    }
    if(!terminated) return false;
  } else {std::memcpy(s.bone_name,text,15); s.bone_name[15]=0;}
  for(unsigned i=0;s.bone_name[i];++i)
    if(s.bone_name[i]<32 || s.bone_name[i]>126) return false;
  return true;
}
// Finger overrides are applied only to each draw's private palette, AFTER all
// shared IK/weapon snapshots have been saved. They cannot feed posewrite, weapon
// calibration, attachment placement or mounted fallback caches.
void gesture_palette(const void* root,float (*mats)[16],unsigned n) {
  static std::mutex frame_mutex;
  static hand_gestures::Frame frame;
  static unsigned generation=0;
  const auto live=hand_gestures::latest();
  if(!live.at) { // invalidate cache too, including loss/regain in one Present
    std::lock_guard<std::mutex> lock(frame_mutex); generation=0; frame={}; return;
  }
  hand_gestures::Frame pose;
  {std::lock_guard<std::mutex> lock(frame_mutex);
    const unsigned gen=frame_gen.load(std::memory_order_acquire);
    if(generation!=gen || frame.identity!=live.identity || frame.mounted!=live.mounted) {generation=gen; frame=live;}
    pose=frame;}
  for(int h=0;h<2;++h) if(live.hand[h].weight==0.f) pose.hand[h].weight=0.f;
  const int capture=hand_gestures::capture_wanted();
  if(!(pose.hand[0].weight>0.f || pose.hand[1].weight>0.f) && capture<0 &&
     !live.hand[0].reference_ready && !live.hand[1].reference_ready) return;
  const void* owner=game::pointers().body;
  if(reinterpret_cast<std::uint64_t>(owner)!=pose.identity || n>512) return;
  const void* resource=nullptr;
  if(!read(root,0x58,&resource,sizeof(resource)) || !resource) return;
  struct Binding {
    const void* resource=nullptr;const void* owner=nullptr;bool mounted=false;
    std::vector<std::string> names;
    hand_gestures::HandReference reference[2];
    bool has_hand[2]{},load_tried[2]{};
  };
  // Shared by ALL palette workers and both eyes. A thread-local calibration
  // would freeze a different native animation frame on each worker and flicker.
  static std::mutex bind_mutex;
  static std::vector<std::shared_ptr<const Binding>> cache;
  std::shared_ptr<const Binding> binding;
  {
    std::lock_guard<std::mutex> lock(bind_mutex);
    for(const auto& v:cache) if(v->resource==resource && v->owner==owner && v->mounted==pose.mounted && v->names.size()==n) {
      binding=v;break;
    }
    if(!binding) {
      auto entry=std::make_shared<Binding>();entry->resource=resource;entry->owner=owner;entry->mounted=pose.mounted;
      entry->names.reserve(n);
      for(unsigned i=0;i<n;++i) {
        Sample s;if(!read_name(root,i,s) || s.name_count!=n) return;
        entry->names.emplace_back(s.bone_name);
      }
      const char* names[512]{};
      for(unsigned i=0;i<n;++i) names[i]=entry->names[i].c_str();
      for(int h=0;h<2;++h) entry->has_hand[h]=hand_gestures::hand_bones_present(names,n,h);
      // Never evict a live rig to recapture it from a different animation frame.
      // At capacity leave unrecognized resources native until player replacement.
      cache.erase(std::remove_if(cache.begin(),cache.end(),[&](const auto& v) {return v->owner!=owner;}),cache.end());
      if(cache.size()>=64) return;
      cache.push_back(entry);binding=entry;
    }
    // Each hand calibrates on its own from a settled empty frame. Saved references
    // survive game restarts and load even with a weapon out; a new one never freezes
    // from the free hand's weapon stance, so that loads once per binding, then waits.
    // [AUTHOR 2026-11-11] mounted gestures as on foot. [LIVE run 170625] every mounted gesture was rejected (applications flat, rejects
    // +1650/s): AvatarVR_hand_references.txt holds only on-foot (mounted 0) references, and a new one needs a settled hand, which a flying
    // mount never is. Riding loads this rig's saved on-foot reference (same mesh) once, at once, and never freezes a new one.
    for(int h=0;h<2;++h) if(binding->has_hand[h] && !binding->reference[h].valid &&
                            (pose.mounted ? !binding->load_tried[h] :
                             live.hand[h].reference_ready && (live.hand[h].reference_new || !binding->load_tried[h]))) {
      const char* names[512]{};
      for(unsigned i=0;i<n;++i) names[i]=binding->names[i].c_str();
      auto next=std::make_shared<Binding>(*binding);
      const bool loaded=pose.mounted ? hand_gestures::calibrate(next->reference[h],mats,n,names,h,false,false)
                                     : hand_gestures::calibrate(next->reference[h],mats,n,names,h,false,live.hand[h].reference_new);
      if(!loaded && !pose.mounted && live.hand[h].reference_new) continue;
      next->load_tried[h]=true;
      for(auto& v:cache) if(v==binding) v=next;
      binding=next;
    }
  }
  // Tuner capture reads this draw's palette before any gesture is written to it.
  if(capture==0 || capture==1)
    hand_gestures::capture(mats,n,capture,pose.mounted,binding->reference[capture]);
  for(int h=0;h<2;++h) if(pose.hand[h].weight>0.f)
    hand_gestures::apply(mats,n,h,pose.hand[h],binding->reference[h]);
}
void arm_names(const void* root,Sample& s) {
  const char* names[]={"L_upperArm","L_lowerArm","L_hand","R_upperArm","R_lowerArm","R_hand"};
  for(auto& index:s.arms) index=-1;
  // +58 is preserved graph metadata; do not assume a parent-table layout.
  if(read(root,0x58,&s.parent_candidate,sizeof(s.parent_candidate))) s.parent_read|=1u;
  if(read(s.parent_candidate,0,s.parent_words,sizeof(s.parent_words))) s.parent_read|=2u;
  if(s.count==0 || s.count>512 || s.name_count!=s.count) return;
  const void* name_header=nullptr; const void* name_data=nullptr;
  if(!read(root,0x50,&name_header,sizeof(name_header)) ||
     !read(name_header,0,&name_data,sizeof(name_data))) return;
  bool complete=true;
  for(unsigned index=0;index<s.count;++index) {
    Sample name{};
    if(!read_name(root,index,name) || name.name_count!=s.count) {complete=false; break;}
    for(unsigned k=0;k<6;++k) if(std::strcmp(name.bone_name,names[k])==0) {
      if(s.arms[k]!=-1) {complete=false; break;}
      s.arms[k]=static_cast<int>(index);
    }
    if(!complete) break;
  }
  const void* checked_header=nullptr; const void* checked_data=nullptr;
  unsigned checked_count=0;
  complete=complete && read(root,0x50,&checked_header,sizeof(checked_header)) &&
    checked_header==name_header && read(name_header,0,&checked_data,sizeof(checked_data)) &&
    checked_data==name_data && read(name_header,8,&checked_count,sizeof(checked_count)) && checked_count==s.count;
  if(!complete) {for(auto& index:s.arms) index=-1; return;}
  s.read_flags|=256u;
  for(unsigned k=0;k<6;++k)
    if(s.arms[k]>=0 && read(s.data,static_cast<unsigned>(s.arms[k])*64+48,s.arm_positions[k],12))
      s.arm_read_mask|=1u<<k;
}
// One-time read of the validated FP skeleton's 211 names: L_/R_ prefix gives the side table and the full list is logged.
void read_sides(const void* root) {
  int expected=0;
  if(!side_state.compare_exchange_strong(expected,1)) return;
  std::string line="skin ik names:";
  bool complete=true; unsigned left=0,right=0;
  for(unsigned index=0;index<211;++index) {
    Sample entry{};
    if(!read_name(root,index,entry)||entry.name_count!=211) {complete=false; break;}
    fp_names[index]=entry.bone_name;
    bone_side[index]=std::strncmp(entry.bone_name,"L_",2)==0?0:std::strncmp(entry.bone_name,"R_",2)==0?1:-1;
    bone_role[index]=static_cast<signed char>(skin_arm_ik::name_role(entry.bone_name));
    {const char* knuckles[3]={"fingerMiddle_01","fingerIndex_01","fingerPinky_01"};
      if(bone_side[index]>=0) for(int k=0;k<3;++k) if(!std::strcmp(entry.bone_name+2,knuckles[k])) anatomy[bone_side[index]][k]=static_cast<int>(index);}
    bone_twist[index]=bone_side[index]<0?0:!std::strncmp(entry.bone_name+2,"lowerArm_twist",14)?1:!std::strncmp(entry.bone_name+2,"upperArm_twist",14)?2:0;
    left+=bone_side[index]==0; right+=bone_side[index]==1;
    line+=" "+std::to_string(index)+"="+entry.bone_name;
    if(line.size()>900) {log(line.c_str()); line="skin ik names:";}
  }
  if(line.size()>15) log(line.c_str());
  char summary[120];
  std::snprintf(summary,sizeof(summary),"skin ik side table %s: left=%u right=%u",complete?"ready":"FAILED",left,right);
  log(summary);
  side_state.store(complete?2:3);
}
void log_followers(const int (&owner)[211],unsigned hand,unsigned assigned,unsigned moved) {
  std::string line=std::string("skin ik followers ")+(hand?"right":"left")+": assigned="+std::to_string(assigned)+
    " moved="+std::to_string(moved);
  for(unsigned bone=0;bone<211;++bone) if(owner[bone]>=0)
    line+=" "+std::to_string(bone)+"->"+std::to_string(owner[bone]);
  log(line.c_str());
}
// Full FP solve for one frame (rest pose, shoulder offsets, headset shift, IK, hand alignment, followers, forearm
// twist). snapshot: the FP pose vector as read (modified in place); raw: copy before any change; copy: result.
// Current look yaw minus the yaw sampled with the grips (wrapped), 0 when unknown or steady hands is off.
float yaw_delta(const GripCache& target) {
  if(!body_anchor || !target.yaw_valid) return 0.f;
  const game::Player now=game::read();
  if(!now.look_ok || !std::isfinite(now.yaw)) return 0.f;
  const float d=std::remainder(now.yaw-target.yaw,6.2831853f);
  return std::fabs(d)<0.6f?d:0.f; // ignore teleports / snaps larger than ~35 degrees
}
// The body of this camera step, so every mesh of a frame and the arm solve share one body. [LIVE 2026-10-31] about 36 player
// palette builds a frame each read it live; at flight speed (18 cm a tick) one late read parts a mesh from the others.
bool rig_body_read(const void* body,float* out) {
  float live[3];
  if(!read(body,0x94,live,sizeof(live))) return false;
  std::memcpy(out,live,12);
  if(!rig_coherent || t_rig_body_live) return true;
  // Riding: the camera-post body can be up to a tick or two from this frame's eye step (body_behind_max 887 mm in boost, LIVE Nov 7).
  // The rider's eyes and grips are built on the body read at the eye step itself, so every rider mesh takes that one.
  if(mount_camera::rider_view()) {float eye_body[3]; if(native_stereo::grip_body(eye_body)) {std::memcpy(out,eye_body,12); return true;}}
  const long long us=clean_qpc_us();
  std::lock_guard<std::mutex> lock(rig_body_mutex);
  if(!rig_body_us || us-rig_body_us>40000) return true;  // no recent camera step: menus, pause
  float d2=0.f; for(int k=0;k<3;++k) d2+=(live[k]-rig_body[k])*(live[k]-rig_body[k]);
  if(d2>25.f) return true;  // teleport
  if(d2>0.0005f*0.0005f) {
    rig_body_stale.fetch_add(1,std::memory_order_relaxed);
    const unsigned mm=static_cast<unsigned>(std::sqrt(d2)*1000.f);
    unsigned seen=rig_stale_mm_max.load(std::memory_order_relaxed);
    while(mm>seen && !rig_stale_mm_max.compare_exchange_weak(seen,mm,std::memory_order_relaxed)) {}
  }
  std::memcpy(out,rig_body,12);
  return true;
}
// The one wrist target of every hand state: free hand, gun hand, support hand and the placed gun. Everything applied here (the
// one-generation hold on a lost alignment or a jump with a still controller) reaches the gun without a second path.
bool guarded_hand_target(const float* sample16,const float* basis,const float* root,skin_arm_ik::Vec shoulder,float navi_length,unsigned hand,
                         const vr_arms_math::HandLocal& local,float* wrist16,float* attach16,float* factor,bool* oriented) {
  bool ok=false;
  if(!vr_arms_math::hand_target(sample16,basis,root,shoulder,navi_length,hand,arm_scale,skin_arm_ik::for_hand(hand_tune,hand),
                                local,wrist16,attach16,factor,&ok)) return false;
  if(oriented) *oriented=ok;
  if(!ok && local.knuckles) {
    rig_align_fails.fetch_add(1,std::memory_order_relaxed);
    if(rig_align_logs.fetch_add(1,std::memory_order_relaxed)<40) {  // [2026-10-31] the 10-30 log printed the held flag, not this failure
      float grip[16]; vr_arms_math::to_world(sample16,basis,grip);
      char line[220]; std::snprintf(line,sizeof(line),"ik align lost: hand=%u grip_ok=%d grip rows %.3f %.3f %.3f basis rows %.3f %.3f %.3f wrist-middle %.3f",hand,skin_arm_ik::grip_basis(grip)?1:0,
        skin_arm_ik::length(skin_arm_ik::row(grip,0)),skin_arm_ik::length(skin_arm_ik::row(grip,1)),skin_arm_ik::length(skin_arm_ik::row(grip,2)),
        skin_arm_ik::length(skin_arm_ik::row(basis,0)),skin_arm_ik::length(skin_arm_ik::row(basis,1)),skin_arm_ik::length(skin_arm_ik::row(basis,2)),
        skin_arm_ik::length(skin_arm_ik::sub(local.middle,skin_arm_ik::position(local.wrist))));
      log(line);
    }
  }
  float frame[16],inverse_frame[16],target[16];  // the wrist on root axes at this shoulder
  std::memcpy(frame,root,64); frame[12]=shoulder.x; frame[13]=shoulder.y; frame[14]=shoulder.z;
  if(!skin_arm_ik::inverse_affine(inverse_frame,frame)) return true;
  skin_arm_ik::multiply_affine(target,wrist16,inverse_frame);
  const unsigned gen_now=frame_gen.load(std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(clean_mutex);
  HandHold& hh=hand_hold[hand];
  float ds=0.f,dl=0.f,turn=0.f;
  for(int k=0;k<3;++k) {
    ds+=(sample16[12+k]-hh.sample[12+k])*(sample16[12+k]-hh.sample[12+k]);
    dl+=(target[12+k]-hh.local[12+k])*(target[12+k]-hh.local[12+k]);
    for(int c=0;c<3;++c) turn+=sample16[k*4+c]*hh.sample[k*4+c];
  }
  if(vr_arms_math::hold_target(hh.have,hh.held,hh.gen==gen_now,ok,hh.oriented,local.knuckles,ds,turn,dl)) {
    skin_arm_ik::multiply_affine(wrist16,hh.local,frame);
    if(local.attach) skin_arm_ik::multiply_affine(attach16,local.rel,wrist16);
    else std::memcpy(attach16,wrist16,64);
    if(oriented) *oriented=hh.oriented;
    hh.held=true; hh.gen=gen_now;
    hand_holds.fetch_add(1,std::memory_order_relaxed);
    if(hand_hold_logs.fetch_add(1,std::memory_order_relaxed)<40) {
      char line[160]; std::snprintf(line,sizeof(line),"ik hand hold: hand=%u jump=%.1f cm align_lost=%d knuckles=%d",hand,std::sqrt(dl)*100.f,ok?0:1,local.knuckles?1:0);
      log(line);
    }
  } else {
    std::memcpy(hh.local,target,64); std::memcpy(hh.sample,sample16,64);
    hh.oriented=ok; hh.gen=gen_now; hh.held=false; hh.have=true;
  }
  return true;
}
bool solve_fp(float (&snapshot)[211][16],float (&raw)[211][16],float (&copy)[211][16],const float* root_matrix,
              const std::int16_t* cell,const float* body_position,const GripCache& target,ULONGLONG now,bool sides) {
  std::memcpy(raw,snapshot,sizeof(raw));
    if(stable_rig) {
      // Model-space hands: the rest pose is anchored to the FP root (body), not the camera bone (whose head-bob arcs the
      // Cyberpunk port avoids for the shoulder pivot).
      const bool on_root=model_space_hands||clean_rig;  // clean rig: shoulders always on the body root
      const int anchor_mode=clean_rig&&view_height_anchor?2:on_root?1:0;
      float cam[16],inv_cam[16]; std::memcpy(cam,on_root?root_matrix:snapshot[1],sizeof(cam));
      // [HEADSET run 56] anchoring to the CameraMarker bone bounced with the weapon (the game sways/bobs that bone with a gun)
      // and its game-thread twin needed a fallback (567 switches = gun flicker). The anchor is now the root plus the VIEW's
      // own offset from the body, taken with the controller sample: identical for the solve and the gun placement.
      if(anchor_mode==2) {
        if(t_clean_ok && t_clean.view_valid) for(int k=0;k<3;++k) cam[12+k]=root_matrix[12+k]+t_clean_view[k];
        else {cam[12]=snapshot[1][12]; cam[13]=snapshot[1][13]; cam[14]=snapshot[1][14];}
      }
      if(skin_arm_ik::inverse_affine(inv_cam,cam)) {
        std::lock_guard<std::mutex> lock(rest_mutex);
        if(rest_on_root!=on_root || rest_anchor_mode!=anchor_mode) {rest_on_root=on_root; rest_anchor_mode=anchor_mode; have_rest=false; still_since=now;}
        if(rest_recapture.exchange(false)) {have_rest=false; still_since=now;}
        // Stillness from the body position: under 0.3 m/s for half a second.
        if(now-last_body_tick>=50) {
          const float dt=last_body_tick?(now-last_body_tick)*0.001f:1.f;
          float d=0.f; for(unsigned k=0;k<3;++k){const float v=body_position[k]-last_body[k]; d+=v*v;}
          const float body_yaw=std::atan2(root_matrix[8],root_matrix[10]);
          if(!last_body_tick || std::sqrt(d)/dt>0.3f || std::fabs(std::remainder(body_yaw-last_body_yaw,6.2831853f))/dt>0.35f) still_since=now;
          last_body_yaw=body_yaw;
          std::memcpy(last_body,body_position,sizeof(last_body)); last_body_tick=now;
        }
        {const void* weapon=nullptr; const game::Pointers wp=game::pointers();
          if(wp.body) game::mem_read(wp.body+0x250,&weapon,sizeof(weapon));
          if(!hand_layout_primed || weapon!=hand_layout_weapon) {hand_layout_window=now+1000; hand_layout_weapon=weapon; hand_layout_primed=true;}
          if(vr_weapon::weapon_placed_by_game()) {
            if(hand_layout_bare) hand_layout_window=now+1000;  // a gun came up: its native hold returns quickly
            hand_layout_placed_at=now; hand_layout_bare=false;
          } else if(!hand_layout_placed_at || now-hand_layout_placed_at>=1500) hand_layout_bare=rig_coherent;
          hand_layout_changing=now<hand_layout_window;
          hand_layout_open=!hand_freeze_moving || hand_layout_changing || now-still_since>=500;}
        rig_player_still.store(now-still_since>=500 && !hand_layout_changing,std::memory_order_relaxed);
        if(!have_rest && now-still_since>=500) {
          for(unsigned b=0;b<211;++b) skin_arm_ik::multiply_affine(rest_rel[b],snapshot[b],inv_cam);
          have_rest=true; rest_captures.fetch_add(1);
          log("skin ik rest pose captured (player still); shoulders now anchored to the camera bone");
        }
        if(have_rest) {
          for(unsigned b=0;b<211;++b) skin_arm_ik::multiply_affine(snapshot[b],rest_rel[b],cam);
          // Live hand animation under the rest-pose wrist: snapshot = live_b * inv(live_wrist) * rest_wrist.
          if(native_hand_poses || hand_gestures::native_base_required()) for(unsigned side=0;side<2;++side) {
            const unsigned wrist=side?115:16;
            float inverse_live[16],to_rest[16];
            if(!skin_arm_ik::inverse_affine(inverse_live,raw[wrist])) continue;
            skin_arm_ik::multiply_affine(to_rest,inverse_live,snapshot[wrist]);
            // Steady hand frame: see HandFrame. live = this frame's hand subtree relative to its own wrist.
            HandFrame& hf=hand_frame[side];
            const int frame_bones[4]={anatomy[side][0],anatomy[side][1],anatomy[side][2],attach_bone[side]};
            static thread_local float live[211][16];
            for(unsigned b=0;b<211;++b) if(b!=wrist && bone_side[b]==static_cast<signed char>(side) && bone_role[b]==2) skin_arm_ik::multiply_affine(live[b],raw[b],inverse_live);
            // The knuckles are rigid to the wrist; only they can show a bad pose (the attach bone roams freely).
            float worst=0.f;
            if(hf.have) for(int i=0;i<3;++i) {const int fb=frame_bones[i]; if(fb<0) continue;
              float d=0.f; for(int k=0;k<3;++k) d+=(live[fb][12+k]-hf.smooth[fb][12+k])*(live[fb][12+k]-hf.smooth[fb][12+k]);
              worst=std::max(worst,d);}
            if(hf.have) {
              if(frame_bones[3]>=0) hf.attach_cm=std::max(hf.attach_cm,skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(live[frame_bones[3]]),skin_arm_ik::position(hf.smooth[frame_bones[3]])))*100.f);
              hf.knuckle_cm=std::max(hf.knuckle_cm,std::sqrt(worst)*100.f);}
            const unsigned gen_now=frame_gen.load(std::memory_order_relaxed);
            // [HEADSET 2026-10-31] on foot the armed hands are the steadiest: a held gun keeps the game's hand still against the wrist. Bare, the
            // animated knuckles move up to 17 cm [LIVE ik hand frame], and they steer the wrist and palm offset. Bare hands now take the palm
            // from the rest pose, as the mounted rig already does. Fingers still animate on it. Armed hands never come through here.
            const bool bare_palm=hand_layout_bare;
            float rest_knuckle[3][16]; bool rest_ok=false;
            if(bare_palm) {
              float inverse_rest[16];
              rest_ok=frame_bones[0]>=0 && frame_bones[1]>=0 && frame_bones[2]>=0 && skin_arm_ik::inverse_affine(inverse_rest,rest_rel[wrist]);
              if(rest_ok) for(int i=0;i<3;++i) skin_arm_ik::multiply_affine(rest_knuckle[i],rest_rel[frame_bones[i]],inverse_rest);
            }
            // [HEADSET 2026-10-31] the palm alone did not help, and the wrist target is as steady bare as armed (|wrist - shoulder| p99 0.13 cm).
            // What a held gun also stops is the fingers. An empty hand now keeps one still pose from a settled moment, as still as the hold on a
            // gun; gestures still move it. Never reached with a gun out.
            const bool bare_static=bare_palm && rest_ok && bare_hand_static;
            if(bare_static && !hf.bare_ready && hf.have && hand_layout_open && !hand_layout_changing) {
              for(unsigned b=0;b<211;++b) if(b!=wrist && bone_side[b]==static_cast<signed char>(side) && bone_role[b]==2) std::memcpy(hf.bare_pose[b],live[b],64);
              hf.bare_ready=true;
              char line[80]; std::snprintf(line,sizeof(line),"ik bare hand: side=%u still pose taken",side); log(line);
            }
            const bool bad_pose=!(bare_palm && rest_ok) && hf.have && hand_layout_open && worst>0.03f*0.03f && !hand_layout_changing && (!hf.held || hf.hold_gen==gen_now);
            if(bad_pose) {
              hf.held=true; hf.hold_gen=gen_now; hand_frame_holds.fetch_add(1,std::memory_order_relaxed);
              if(hand_frame_logs.fetch_add(1,std::memory_order_relaxed)<40) {
                char line[120]; std::snprintf(line,sizeof(line),"ik hand frame hold: side=%u moved=%.1f cm",side,std::sqrt(worst)*100.f); log(line);}
            } else if(!(bare_static && hf.bare_ready) && (!hf.have || (hand_layout_open && hf.smooth_gen!=gen_now))) {
              hf.held=false;
              const float dt=hf.have?static_cast<float>(now-hf.smooth_tick)*0.001f:0.f;
              const float a_finger=1.f-std::exp(-dt/0.10f), a_frame=hand_layout_changing?a_finger:1.f-std::exp(-dt/1.f);
              for(int i=0;i<3;++i) {const int fb=frame_bones[i]; if(fb<0) continue;
                if(!hf.have) std::memcpy(hf.fast_knuckle[i],live[fb],64);
                else {float mixed[16]; vr_arms_math::blend_affine(hf.fast_knuckle[i],live[fb],a_finger,mixed); bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v); if(fin) std::memcpy(hf.fast_knuckle[i],mixed,64);}}
              for(unsigned b=0;b<211;++b) if(b!=wrist && bone_side[b]==static_cast<signed char>(side) && bone_role[b]==2) {
                const bool frame_bone=static_cast<int>(b)==frame_bones[0]||static_cast<int>(b)==frame_bones[1]||static_cast<int>(b)==frame_bones[2]||static_cast<int>(b)==frame_bones[3];
                if(hf.have && bare_palm && rest_ok && (static_cast<int>(b)==frame_bones[0]||static_cast<int>(b)==frame_bones[1]||static_cast<int>(b)==frame_bones[2])) continue;
                if(!hf.have) std::memcpy(hf.smooth[b],live[b],64);
                else {float mixed[16]; vr_arms_math::blend_affine(hf.smooth[b],live[b],frame_bone?a_frame:a_finger,mixed); bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v); if(fin) std::memcpy(hf.smooth[b],mixed,64);}
              }
              hf.smooth_gen=gen_now; hf.smooth_tick=now; hf.have=true;
            } else if(!hand_layout_open) hf.smooth_tick=now;
            if(bare_palm && rest_ok && hf.have && hf.bare_tick!=now) {  // also while moving, so a holstered gun's hold never stays on the hand
              const float dt=hf.bare_tick?std::min(static_cast<float>(now-hf.bare_tick)*0.001f,0.1f):0.f;
              const float a=1.f-std::exp(-dt/0.25f);
              if(a>0.f && bare_static && hf.bare_ready) {
                for(unsigned b=0;b<211;++b) if(b!=wrist && bone_side[b]==static_cast<signed char>(side) && bone_role[b]==2 &&
                   static_cast<int>(b)!=frame_bones[0] && static_cast<int>(b)!=frame_bones[1] && static_cast<int>(b)!=frame_bones[2]) {
                  float mixed[16]; vr_arms_math::blend_affine(hf.smooth[b],hf.bare_pose[b],a,mixed);
                  bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v); if(fin) std::memcpy(hf.smooth[b],mixed,64);
                }
                for(int i=0;i<3;++i) {
                  float mixed[16]; vr_arms_math::blend_affine(hf.fast_knuckle[i],hf.bare_pose[frame_bones[i]],a,mixed);
                  bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v); if(fin) std::memcpy(hf.fast_knuckle[i],mixed,64);
                }
              }
              if(a>0.f) for(int i=0;i<3;++i) {
                float mixed[16]; vr_arms_math::blend_affine(hf.smooth[frame_bones[i]],rest_knuckle[i],a,mixed);
                bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v); if(fin) std::memcpy(hf.smooth[frame_bones[i]],mixed,64);
              }
              hf.bare_tick=now;
            } else if(!bare_palm) hf.bare_tick=0;
            if(now-hf.report_tick>=5000) {
              char line[160]; std::snprintf(line,sizeof(line),"ik hand frame: side=%u holds=%u open=%d native vs rig: attach max %.1f cm knuckle max %.1f cm",side,hand_frame_holds.load(),hand_layout_open?1:0,hf.attach_cm,hf.knuckle_cm); log(line);
              hf.attach_cm=hf.knuckle_cm=0.f; hf.report_tick=now;}
            // The rig keeps steering from the slow knuckle positions and the slow attach bone (gun hold); every other
            // bone is the fast hand moved rigidly onto that palm, so no finger separates from its knuckle.
            float palm_fix[16]; bool have_palm_fix=frame_bones[0]>=0 && frame_bones[1]>=0 && frame_bones[2]>=0;
            if(have_palm_fix) {
              const float* const fast[3]={hf.fast_knuckle[0],hf.fast_knuckle[1],hf.fast_knuckle[2]};
              const float* const slow[3]={hf.smooth[frame_bones[0]],hf.smooth[frame_bones[1]],hf.smooth[frame_bones[2]]};
              have_palm_fix=vr_arms_math::steady_hand_correction(fast,slow,palm_fix);
            }
            for(unsigned b=0;b<211;++b) {
              if(b==wrist || bone_side[b]!=static_cast<signed char>(side) || bone_role[b]!=2) continue;
              float moved[16],steady[16]; const float* pose=hf.smooth[b];
              if(have_palm_fix && static_cast<int>(b)!=frame_bones[3]) {
                for(int i=0;i<3;++i) if(static_cast<int>(b)==frame_bones[i]) pose=hf.fast_knuckle[i];
                skin_arm_ik::multiply_affine(steady,pose,palm_fix); pose=steady;
              }
              skin_arm_ik::multiply_affine(moved,pose,snapshot[wrist]);
              bool ok=true; for(float v:moved) ok=ok&&std::isfinite(v);
              if(ok) std::memcpy(snapshot[b],moved,sizeof(moved));
            }
          }
        }
      }
    }
    if(shoulder_height!=0.f || shoulder_forward!=0.f) {
      skin_arm_ik::Vec up=skin_arm_ik::row(root_matrix,1),fwd=skin_arm_ik::row(root_matrix,2);
      const float lu=skin_arm_ik::length(up),lf=skin_arm_ik::length(fwd);
      if(lu>1e-4f && lf>1e-4f) {
        const skin_arm_ik::Vec shift=skin_arm_ik::add(skin_arm_ik::scale(up,shoulder_height/lu),skin_arm_ik::scale(fwd,shoulder_forward/lf));
        for(auto& m:snapshot){m[12]+=shift.x;m[13]+=shift.y;m[14]+=shift.z;}
      }
    }
    if(shoulder_spread!=0.f && side_state.load()==2) {
      skin_arm_ik::Vec right=skin_arm_ik::row(root_matrix,0); const float lr=skin_arm_ik::length(right);
      if(lr>1e-4f) {
        right=skin_arm_ik::scale(right,0.5f*shoulder_spread/lr);
        for(unsigned b=0;b<211;++b) if(bone_side[b]>=0) {
          const float sign=bone_side[b]?1.f:-1.f;
          snapshot[b][12]+=sign*right.x; snapshot[b][13]+=sign*right.y; snapshot[b][14]+=sign*right.z;
        }
      }
    }
    float head[3]{};
    const bool sampled_head=(rig_coherent || ik_sync_head.load(std::memory_order_relaxed)) && clean_rig && t_clean_ok &&
                            t_clean.head_world_valid;
    bool have_head=false;
    if(shoulders_follow)
      have_head=sampled_head ? (std::memcpy(head,t_clean.head_world,sizeof(head)),true)
                             : native_stereo::head_offset(head);
    if(have_head)
      for(auto& m:snapshot){m[12]+=head[0];m[13]+=head[1];m[14]+=head[2];}
    // run 63: left-handed mode with the gun held: the hands swap hold poses. Each hand bone takes its partner's pose
    // relative to the partner's wrist, mirrored (rotation equal, position negated: D = -I, run 62), on its own wrist.
    if(clean_rig && attach_bone[1]>=0) {  // run 67: the game's own support hold of this frame (raw = its animation)
      float inverse_attach[16];
      if(skin_arm_ik::inverse_affine(inverse_attach,raw[attach_bone[1]])) {
        SupportHold sh; skin_arm_ik::multiply_affine(sh.h_sup_l,raw[16],inverse_attach); skin_arm_ik::multiply_affine(sh.h_dom_r,raw[115],inverse_attach);
        float d2=0.f; for(int k=0;k<3;++k) d2+=(sh.h_sup_l[12+k]-sh.h_dom_r[12+k])*(sh.h_sup_l[12+k]-sh.h_dom_r[12+k]);
        sh.baseline=std::sqrt(d2); sh.leverage=std::clamp(sh.baseline/0.35f,0.f,1.f);
        bool fin=std::isfinite(sh.baseline); for(float v:sh.h_sup_l) fin=fin&&std::isfinite(v);
        sh.valid=fin && sh.baseline<1.2f;
        // Moving: the hold keeps its last value (native sway must not steer the two-hand swing), but stays "fresh".
        std::lock_guard<std::mutex> lock(clean_mutex); if(sh.valid && (hand_layout_open || !support_hold.valid)) support_hold=sh; if(support_hold.valid) support_hold_us=clean_qpc_us();
        two_hand_lev_milli.store(static_cast<int>(sh.leverage*1000.f),std::memory_order_relaxed);
      }
    }
    const float finger_amount=left_amount();  // run 65: the hold poses cross-fade during a hand swap
    if(finger_amount>0.f && vr_weapon::weapon_placed_by_game()) {
      build_partners();
      if(fp_partner_ready) {
        float inverse_w[2][16]; bool ok=skin_arm_ik::inverse_affine(inverse_w[0],snapshot[16]) && skin_arm_ik::inverse_affine(inverse_w[1],snapshot[115]);
        static thread_local float swapped[211][16]; bool any=false;
        if(ok) for(unsigned b=0;b<211;++b) {
          const int p=fp_partner[b];
          if(p<0 || bone_side[b]<0 || bone_role[b]!=2 || b==16 || b==115 || fp_names[b].find("weaponAttachment")!=std::string::npos) continue;
          const unsigned side=static_cast<unsigned>(bone_side[b]), other=1u-side;
          float local_p[16],mirrored[16];
          skin_arm_ik::multiply_affine(local_p,snapshot[p],inverse_w[other]);   // partner relative to its wrist
          vr_arms_math::mirror_wrist_local(local_p,mirrored);
          skin_arm_ik::multiply_affine(swapped[b],mirrored,snapshot[side?115:16]);
          bool fin=true; for(float v:swapped[b]) fin=fin&&std::isfinite(v);
          if(fin) any=true; else std::memcpy(swapped[b],snapshot[b],64);
        }
        if(any) {
          for(unsigned b=0;b<211;++b) {const int p=fp_partner[b];
            if(p<0 || bone_side[b]<0 || bone_role[b]!=2 || b==16 || b==115 || fp_names[b].find("weaponAttachment")!=std::string::npos) continue;
            if(finger_amount>=1.f) std::memcpy(snapshot[b],swapped[b],64);
            else {float mixed[16]; vr_arms_math::blend_affine(snapshot[b],swapped[b],finger_amount,mixed); std::memcpy(snapshot[b],mixed,64);}}
          finger_swaps.fetch_add(1,std::memory_order_relaxed);
        }
      }
    }
    std::memcpy(copy,snapshot,sizeof(copy)); bool changed=false;
    const skin_arm_ik::HandTune tune=hand_tune;
    // T-pose calibration (menu "Calibrate arms (T-pose)"): both grips + headset, same frame and corrections as the solve.
    if(calibrate_request.load() && target.valid[0] && target.valid[1] && target.head_valid) {
      skin_arm_ik::Vec pts[3];
      const float* src3[3]={target.matrix[0]+12,target.matrix[1]+12,target.head};
      for(unsigned k=0;k<3;++k) {
        pts[k]={src3[k][0]-1024.f*cell[0],src3[k][1]-1024.f*cell[1],src3[k][2]-1024.f*cell[2]};
        if(body_anchor && target.body_valid) pts[k]=skin_arm_ik::add(pts[k],skin_arm_ik::Vec{body_position[0]-target.body[0],body_position[1]-target.body[1],body_position[2]-target.body[2]});
      }
      const skin_arm_ik::TPose t=skin_arm_ik::tpose_calibration(pts[0],pts[1],pts[2],root_matrix);
      calibrate_request.store(false);
      char line[260];
      if(t.ok) {
        arm_scale.user_length=t.length; arm_scale.width=t.width; arm_scale.drop=t.drop; arm_scale.back=t.back; arm_scale.reach=1.f;
        // Hand offsets are left alone (hand separation is user-tuned after calibrating).
        std::snprintf(line,sizeof(line),"arm calibration (T-pose) OK: span %.2f m, arm %.2f m, shoulder width %.2f m, drop %.2f m, back %.2f m",
                      t.span,t.length,t.width,t.drop,t.back);
      } else {
        std::snprintf(line,sizeof(line),"arm calibration REJECTED: span %.2f m lateral %.2f drop %.2f back %.2f arm %.2f (stand straight, arms level out to the sides)",
                      t.span,t.lateral,t.drop,t.back,t.length);
      }
      log(line);
    }
    for(unsigned hand=0;hand<2;++hand) if(target.valid[hand]) {
      float grip[16]; std::memcpy(grip,target.matrix[hand],sizeof(grip));
      grip[12]-=1024.f*cell[0]; grip[13]-=1024.f*cell[1]; grip[14]-=1024.f*cell[2];
      const bool model=model_space_hands && target.head_valid;
      const unsigned wrist=hand?115:16;
      // Clean arm rig (run 54): the wrist target comes from the shared sample through vr_arms_math::hand_target, the
      // same function the game-thread gun placement uses; the cache keeps this hand's layout and shoulder for it.
      bool clean_done=false,clean_oriented=false; float clean_wrist[16];
      if(clean_rig && t_clean_ok && t_clean.valid[hand]) {
        vr_arms_math::HandLocal hl; std::memcpy(hl.wrist,snapshot[wrist],64);
        if(sides && anatomy[hand][0]>=0 && anatomy[hand][1]>=0 && anatomy[hand][2]>=0) {
          hl.middle=skin_arm_ik::position(snapshot[anatomy[hand][0]]); hl.index=skin_arm_ik::position(snapshot[anatomy[hand][1]]);
          hl.pinky=skin_arm_ik::position(snapshot[anatomy[hand][2]]); hl.knuckles=true;}
        float inverse_wrist0[16];
        if(attach_bone[hand]>=0 && skin_arm_ik::inverse_affine(inverse_wrist0,snapshot[wrist])) {
          skin_arm_ik::multiply_affine(hl.rel,snapshot[attach_bone[hand]],inverse_wrist0); hl.attach=true;}
        hl.valid=true;
        const unsigned upper=hand?113:14;
        const skin_arm_ik::Vec s0=skin_arm_ik::position(snapshot[upper]),e0=skin_arm_ik::position(snapshot[upper+1]),w0=skin_arm_ik::position(snapshot[upper+2]);
        float palm=0.1f;
        if(anatomy[hand][0]>=0) palm=hand_tune.palm*skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(snapshot[anatomy[hand][0]]),w0));
        const float navi_length=skin_arm_ik::length(skin_arm_ik::sub(e0,s0))+skin_arm_ik::length(skin_arm_ik::sub(w0,e0))+palm;
        float basis[16]; vr_arms_math::basis_for_root(t_clean.basis,root_matrix,basis);
        float attach16[16],factor=0.f;
        if(guarded_hand_target(t_clean.local[hand],basis,root_matrix,s0,navi_length,hand,hl,clean_wrist,attach16,&factor,&clean_oriented)) {
          clean_done=true; clean_solves.fetch_add(1,std::memory_order_relaxed);
          scale_milli[hand].store(arm_scale.enabled?static_cast<int>(factor*1000.f):0);
          // cache for the gun placement: shoulder in root-local coordinates without the headset shift
          float anchor0[16]; std::memcpy(anchor0,root_matrix,64);
          const bool view_anchor=view_height_anchor;
          if(view_anchor) {  // root axes, root + the sample's view offset (same anchor as the rest pose and the gun)
            if(t_clean.view_valid) for(int k=0;k<3;++k) anchor0[12+k]=root_matrix[12+k]+t_clean_view[k];
            else {anchor0[12]=raw[1][12]; anchor0[13]=raw[1][13]; anchor0[14]=raw[1][14];}}
          float inverse_root0[16];
          if(skin_arm_ik::inverse_affine(inverse_root0,anchor0)) {
            float sh[16]{}; sh[0]=sh[5]=sh[10]=sh[15]=1.f; sh[12]=s0.x-head[0]; sh[13]=s0.y-head[1]; sh[14]=s0.z-head[2];
            float shl[16]; skin_arm_ik::multiply_affine(shl,sh,inverse_root0);
            std::lock_guard<std::mutex> lock(clean_mutex);
            clean_cache[hand].local=hl; clean_cache[hand].navi_length=navi_length;
            clean_cache[hand].shoulder_root[0]=shl[12]; clean_cache[hand].shoulder_root[1]=shl[13]; clean_cache[hand].shoulder_root[2]=shl[14];
            clean_cache[hand].view_anchor=view_anchor; clean_cache[hand].valid=true;
            // health: a hand that jumps > 1 cm in one solve while the controller sample and the body are still
            const float bw[3]={body_position[0],body_position[1],body_position[2]};
            if(jump_have[hand]) {
              float dp=0.f,ds=0.f,db=0.f;
              for(int k=0;k<3;++k) {dp+=(clean_wrist[12+k]+1024.f*cell[k]-jump_last_point[hand][k])*(clean_wrist[12+k]+1024.f*cell[k]-jump_last_point[hand][k]);
                ds+=(t_clean.local[hand][12+k]-jump_last_sample[hand][k])*(t_clean.local[hand][12+k]-jump_last_sample[hand][k]);
                db+=(bw[k]-jump_last_body[k])*(bw[k]-jump_last_body[k]);}
              if(ds<0.002f*0.002f && db<0.002f*0.002f && dp>0.01f*0.01f) {
                hand_jump_still.fetch_add(1,std::memory_order_relaxed);
                static float last_root[16]{},last_basis[16]{},last_shoulder[3]{}; static unsigned long long last_seq=0; static unsigned last_gen=0;
                if(jump_logs.fetch_add(1)<60) {
                  float ry=std::remainder(vr_arms_math::yaw_of(root_matrix)-vr_arms_math::yaw_of(last_root),6.2831853f)*57.2958f;
                  float bp=0.f; for(int k=0;k<3;++k) bp+=std::fabs(basis[4+k]-last_basis[4+k]);
                  float sd=0.f; for(int k=0;k<3;++k) sd+=(s0.x*(k==0)+s0.y*(k==1)+s0.z*(k==2)-last_shoulder[k])*(s0.x*(k==0)+s0.y*(k==1)+s0.z*(k==2)-last_shoulder[k]);
                  char line[260]; std::snprintf(line,sizeof(line),"clean jump: hand %u %.3f m | root yaw %.2f deg, basis up-row change %.4f, shoulder %.3f m, sample seq %llu->%llu, gen %u->%u, tick %u",
                    hand,std::sqrt(dp),ry,bp,std::sqrt(sd),last_seq,static_cast<unsigned long long>(t_clean.seq),last_gen,frame_gen.load(),vr_weapon::placement_tick());
                  log(line);
                }
                std::memcpy(last_root,root_matrix,64); std::memcpy(last_basis,basis,64);
                last_shoulder[0]=s0.x; last_shoulder[1]=s0.y; last_shoulder[2]=s0.z; last_seq=t_clean.seq; last_gen=frame_gen.load();
              }
            }
            for(int k=0;k<3;++k) {jump_last_point[hand][k]=clean_wrist[12+k]+1024.f*cell[k]; jump_last_sample[hand][k]=t_clean.local[hand][12+k]; jump_last_body[k]=bw[k];}
            jump_have[hand]=true;
          }
        }
      }
      if(model && !clean_done) {
        skin_arm_ik::Vec hmd_rel{target.matrix[hand][12]-target.head[0],target.matrix[hand][13]-target.head[1],target.matrix[hand][14]-target.head[2]};
        if(target.map_valid) {hmd_rel=skin_arm_ik::rotate_dir(hmd_rel,target.map); skin_arm_ik::rotate_rows(grip,target.map);}
        const unsigned upper=hand?113:14;
        const skin_arm_ik::Vec s=skin_arm_ik::position(snapshot[upper]),e=skin_arm_ik::position(snapshot[upper+1]),w=skin_arm_ik::position(snapshot[upper+2]);
        float palm=0.1f;
        if(anatomy[hand][0]>=0) palm=hand_tune.palm*skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(snapshot[anatomy[hand][0]]),w));
        const float navi_length=skin_arm_ik::length(skin_arm_ik::sub(e,s))+skin_arm_ik::length(skin_arm_ik::sub(w,e))+palm;
        float factor=0.f;
        const skin_arm_ik::Vec p=skin_arm_ik::model_hand_point(hmd_rel,root_matrix,hand,arm_scale,s,navi_length,&factor);
        if(skin_arm_ik::finite(p)) {grip[12]=p.x; grip[13]=p.y; grip[14]=p.z;}
        scale_milli[hand].store(arm_scale.enabled?static_cast<int>(factor*1000.f):0);
        if(hand_smoothing) {  // One-Euro in the root frame (the target is body-relative already)
          float inverse_root[16],local[16];
          if(skin_arm_ik::inverse_affine(inverse_root,root_matrix)) {
            skin_arm_ik::multiply_affine(local,grip,inverse_root);
            LARGE_INTEGER now_qpc,freq; QueryPerformanceCounter(&now_qpc); QueryPerformanceFrequency(&freq);
            std::lock_guard<std::mutex> lock(filter_mutex);
            const float dt=filter_last[hand].QuadPart?static_cast<float>(now_qpc.QuadPart-filter_last[hand].QuadPart)/static_cast<float>(freq.QuadPart):0.f;
            filter_last[hand]=now_qpc;
            skin_arm_ik::smooth_grip(local,grip_filters[hand],dt);
            skin_arm_ik::multiply_affine(grip,local,root_matrix);
          }
        }
        model_hand_solves.fetch_add(1);
      }
      if(!model && !clean_done) {
      if(body_anchor && target.body_valid) for(unsigned k=0;k<3;++k) grip[12+k]+=body_position[k]-target.body[k];
      // Turning compensation: the grips were sampled with the look yaw of the head step; the rig is posed with the
      // current one. Rotate the grip about the body by the difference (right-stick turn lag/jitter, 2026-10-15).
      const float dyaw=yaw_delta(target);
      if(dyaw!=0.f) skin_arm_ik::rotate_about_up(grip,body_position[0]-1024.f*cell[0],body_position[2]-1024.f*cell[2],dyaw);
      // Hand smoothing (One-Euro) in the root frame, so walking and turning add no filter lag.
      if(hand_smoothing) {
        float inverse_root[16],local[16];
        if(skin_arm_ik::inverse_affine(inverse_root,root_matrix)) {
          skin_arm_ik::multiply_affine(local,grip,inverse_root);
          LARGE_INTEGER now_qpc,freq; QueryPerformanceCounter(&now_qpc); QueryPerformanceFrequency(&freq);
          std::lock_guard<std::mutex> lock(filter_mutex);
          const float dt=filter_last[hand].QuadPart?static_cast<float>(now_qpc.QuadPart-filter_last[hand].QuadPart)/static_cast<float>(freq.QuadPart):0.f;
          filter_last[hand]=now_qpc;
          skin_arm_ik::smooth_grip(local,grip_filters[hand],dt);
          skin_arm_ik::multiply_affine(grip,local,root_matrix);
        }
      }
      // Arm scaling: replay the controller's offset from the user's real shoulder from the Na'vi shoulder.
      if(target.head_valid) {
        skin_arm_ik::Vec head{target.head[0]-1024.f*cell[0],target.head[1]-1024.f*cell[1],target.head[2]-1024.f*cell[2]};
        if(body_anchor && target.body_valid) head=skin_arm_ik::add(head,skin_arm_ik::Vec{body_position[0]-target.body[0],body_position[1]-target.body[1],body_position[2]-target.body[2]});
        if(dyaw!=0.f) {float hm[16]{}; hm[0]=hm[5]=hm[10]=hm[15]=1.f; hm[12]=head.x; hm[13]=head.y; hm[14]=head.z;
          skin_arm_ik::rotate_about_up(hm,body_position[0]-1024.f*cell[0],body_position[2]-1024.f*cell[2],dyaw); head={hm[12],hm[13],hm[14]};}
        const skin_arm_ik::ArmScale scaling=arm_scale;
        const skin_arm_ik::Vec user_shoulder=skin_arm_ik::real_shoulder(head,root_matrix,hand,scaling);
        const skin_arm_ik::Vec grip_point{grip[12],grip[13],grip[14]};
        if(scaling.enabled) {
          const unsigned upper=hand?113:14;
          const skin_arm_ik::Vec s=skin_arm_ik::position(snapshot[upper]),e=skin_arm_ik::position(snapshot[upper+1]),w=skin_arm_ik::position(snapshot[upper+2]);
          float palm=0.1f;
          if(anatomy[hand][0]>=0) palm=hand_tune.palm*skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(snapshot[anatomy[hand][0]]),w));
          const float navi_length=skin_arm_ik::length(skin_arm_ik::sub(e,s))+skin_arm_ik::length(skin_arm_ik::sub(w,e))+palm;
          float factor=0.f;
          const skin_arm_ik::Vec scaled=skin_arm_ik::scale_reach(grip_point,user_shoulder,s,navi_length,scaling,&factor);
          if(skin_arm_ik::finite(scaled)) {grip[12]=scaled.x; grip[13]=scaled.y; grip[14]=scaled.z;}
          scale_milli[hand].store(static_cast<int>(factor*1000.f));
        } else scale_milli[hand].store(0);
      }
      }  // !model
      // Anatomical alignment: wrist axes and wrist target from the grip; needs the three knuckle bones from the names.
      float aligned[16]; std::memcpy(aligned,snapshot[wrist],sizeof(aligned));
      skin_arm_ik::Vec point{grip[12],grip[13],grip[14]};
      bool oriented=false;
      const skin_arm_ik::HandTune hand_t=skin_arm_ik::for_hand(tune,hand);
      if(clean_done) {
        std::memcpy(aligned,clean_wrist,64); point={clean_wrist[12],clean_wrist[13],clean_wrist[14]}; oriented=clean_oriented;
        if(!oriented) std::memcpy(aligned,snapshot[wrist],48);  // keep the animated wrist axes, the target position stays
      } else {
      if(sides && anatomy[hand][0]>=0 && anatomy[hand][1]>=0 && anatomy[hand][2]>=0)
        oriented=skin_arm_ik::align_hand(aligned,point,skin_arm_ik::position(snapshot[anatomy[hand][0]]),
          skin_arm_ik::position(snapshot[anatomy[hand][1]]),skin_arm_ik::position(snapshot[anatomy[hand][2]]),grip,hand_t);
      // Position offsets in the body frame (do not swing with the controller's rotation).
      point=skin_arm_ik::body_offset(point,root_matrix,hand_t);
      }
      const skin_arm_ik::Vec free_point=point;
      // [HEADSET runs 15-20] the gun's skinned frame can be drawn at this frame's hand, but its rigid parts only where the
      // game places them (a frame earlier), so every scheme moving the gun to the hand split the gun or lagged. Instead
      // the hand goes onto the gun: (1) the controller-driven weapon socket (wrist target * fixed wrist->socket offset)
      // is stored for the game's placement (weapon_attach_matrices), keeping aim live; (2) while the game places the gun,
      // the weapon hand's wrist target becomes the socket the game actually used * inverse offset, so the drawn hand
      // holds every gun part one-to-one (walk effect included, same matrix).
      if(attach_bone[hand]>=0) {
        float rel[16],inverse_wrist[16];
        if(skin_arm_ik::inverse_affine(inverse_wrist,snapshot[wrist])) {
          skin_arm_ik::multiply_affine(rel,snapshot[attach_bone[hand]],inverse_wrist);  // attach = rel * wrist
          float ctrl_wrist[16]; std::memcpy(ctrl_wrist,aligned,64);
          ctrl_wrist[12]=point.x; ctrl_wrist[13]=point.y; ctrl_wrist[14]=point.z;
          {std::lock_guard<std::mutex> lock(fresh_mutex);
            skin_arm_ik::multiply_affine(fresh_attach[hand],rel,ctrl_wrist);
            for(int k=0;k<3;++k) {fresh_cell[hand][k]=cell[k]; fresh_body[hand][k]=body_position[k];} fresh_valid[hand]=true;
            fresh_cam_valid[hand]=target.cam_valid; if(target.cam_valid) std::memcpy(fresh_cam[hand],target.cam,64);
            fresh_grip_valid[hand]=target.cam_valid; if(target.cam_valid) {std::memcpy(fresh_grip[hand],target.matrix[hand],64); std::memcpy(fresh_grip_cam[hand],target.cam,64);}
            std::memcpy(fresh_root[hand],root_matrix,64); for(int k=0;k<3;++k) fresh_root[hand][12+k]+=1024.f*cell[k];}
          const int weapon_hand=(left_handed?0:1);
          float used_world[16];
          // run 63: left mode pins the left wrist with the MIRRORED right-hand hold (vr_arms_math::mirror_hold of
          // inverse(rel_R)); the game's own L attach relation is the foregrip hold and is not used.
          float pin_rel[16]; std::memcpy(pin_rel,rel,64); bool pin_ok=true;
          // run 65: pin only at the ends of a hand swap (while the gun travels both hands are free)
          const float la=left_amount(); const int pin_hand=la<=0.f?1:(la>=1.f?0:-1);
          if(hand==0) {
            pin_ok=false; float inverse_wr[16],rel_r[16],h_r[16],h_l[16];
            if(attach_bone[1]>=0 && skin_arm_ik::inverse_affine(inverse_wr,snapshot[115])) {
              skin_arm_ik::multiply_affine(rel_r,snapshot[attach_bone[1]],inverse_wr);
              if(skin_arm_ik::inverse_affine(h_r,rel_r)) {vr_arms_math::mirror_hold(h_r,h_l); pin_ok=skin_arm_ik::inverse_affine(pin_rel,h_l);}
            }
          }
          unsigned long long socket_age_ms=0;
          const bool pin_candidate=static_cast<int>(hand)==pin_hand && pin_ok;
          const bool game_placed=pin_candidate && vr_weapon::weapon_placed_by_game();
          const bool socket_valid=game_placed && vr_weapon::written_socket(used_world,nullptr,&socket_age_ms);
          float used_cell[16]; const bool used_precise=socket_valid && vr_weapon::written_socket_in_cell(used_cell,cell);
          if(pin_candidate && clean_rig && ik_trace_active.load(std::memory_order_acquire) && !socket_valid) {
            GunRecord last; bool recent=false;
            {std::lock_guard<std::mutex> lock(clean_mutex);
              if(gun_record.valid && clean_qpc_us()-gun_record.us<500000) {last=gun_record; recent=true;}}
            if(recent) {
              IkTraceRow tr; tr.phase=game_placed?"pin_socket_miss":"pin_placement_miss";
              tr.hand=static_cast<int>(hand); tr.sample=&t_clean; tr.gun_seq=last.sample.seq;
              for(int k=0;k<3;++k) {tr.free_target[k]=((&point.x)[k])+1024.f*cell[k]; tr.object[k]=last.ik[12+k];}
              ik_trace(tr);
            }
          }
          if(socket_valid) {
            float used[16]; std::memcpy(used,used_world,64); for(int k=0;k<3;++k) used[12+k]-=1024.f*cell[k];
            if(used_precise) std::memcpy(used,used_cell,64);
            vr_weapon::timing_mark('H',used_world+12);
            if(clean_rig && t_clean_ok) {std::lock_guard<std::mutex> lock(clean_mutex);
              if(gun_record.valid && gun_record.sample.seq!=t_clean.seq) inputs_mismatch.fetch_add(1,std::memory_order_relaxed);}
            if(!clean_rig) {float actual[16]; bool same=false;
              {std::lock_guard<std::mutex> lock(gun_mutex);
                if(gun_actual_gen==frame_gen.load()) {std::memcpy(actual,gun_actual_world,64); same=true;}}
              if(same) {
                for(int k=0;k<3;++k) actual[12+k]-=1024.f*cell[k];
                if(skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(actual),skin_arm_ik::position(used)))<0.3f) {
                  std::memcpy(used,actual,64); hand_on_gun_root.fetch_add(1);}
                gun_same_frame.fetch_add(1);
              } else gun_other_frame.fetch_add(1);}
            float inverse_rel[16],wrist_target[16];
            if(skin_arm_ik::inverse_affine(inverse_rel,pin_rel)) {
              if(left_handed) left_pins.fetch_add(1,std::memory_order_relaxed);
              skin_arm_ik::multiply_affine(wrist_target,inverse_rel,used);
              const skin_arm_ik::Vec w{wrist_target[12],wrist_target[13],wrist_target[14]};
              if(skin_arm_ik::finite(w) && skin_arm_ik::length(skin_arm_ik::sub(w,point))<1.f) {  // run 24: 0.5 m was too tight for fast moves
                if(clean_rig && t_clean_ok) {
                  unsigned long long gun_seq=0; long long gun_age_us=0;
                  {std::lock_guard<std::mutex> lock(clean_mutex);
                    if(gun_record.valid) {gun_seq=gun_record.sample.seq; gun_age_us=clean_qpc_us()-gun_record.us;}}
                  const float free_world[3]={point.x+1024.f*cell[0],point.y+1024.f*cell[1],point.z+1024.f*cell[2]};
                  const float pin_world[3]={w.x+1024.f*cell[0],w.y+1024.f*cell[1],w.z+1024.f*cell[2]};
                  trace_armed_target(hand,t_clean.seq,gun_seq,gun_age_us,socket_age_ms,free_world,pin_world,
                                     used_world+12,body_position,t_clean.view_offset);
                  const float pin_gap=skin_arm_ik::length(skin_arm_ik::sub(w,point));
                  if(pin_gap>0.03f && ik_trace_active.load(std::memory_order_acquire)) {
                    IkTraceRow tr; tr.phase="pin_gap"; tr.hand=static_cast<int>(hand); tr.sample=&t_clean;
                    tr.gap=pin_gap; tr.gun_seq=gun_seq; tr.socket_age_ms=socket_age_ms;
                    std::memcpy(tr.free_target,free_world,12); std::memcpy(tr.final_target,pin_world,12);
                    std::memcpy(tr.object,used_world+12,12); ik_trace(tr);
                  }
                }
                float so=0.f; {std::lock_guard<std::mutex> lock(clean_mutex); so=support_only_locked(clean_qpc_us());}
                if(so>0.f) {  // run 68: the trigger hand lets go of the gun (eased back to its own controller target)
                  float own[16],mixed[16]; std::memcpy(own,aligned,64); own[12]=point.x; own[13]=point.y; own[14]=point.z; own[15]=1.f;
                  vr_arms_math::blend_affine(own,wrist_target,1.f-so,mixed);
                  point={mixed[12],mixed[13],mixed[14]}; std::memcpy(aligned,mixed,64); oriented=true;
                } else {point=w; std::memcpy(aligned,wrist_target,64); oriented=true;}
                hand_on_gun.fetch_add(1);}
            }
          }
        }
      }
      {  // run 67: the support hand slides onto the native support point while the two-hand hold's weight rises
        const float la2=left_amount(); const int dom_now=la2<=0.f?1:(la2>=1.f?0:-1);
        float w=0.f; TwoHandState th; SupportHold sh;
        {std::lock_guard<std::mutex> lock(clean_mutex); w=two_hand_weight_locked(clean_qpc_us()); th=two_hand; sh=support_hold;}
        float sock_w[16];
        if(w>0.f && dom_now>=0 && static_cast<int>(hand)==1-dom_now && th.support==static_cast<int>(hand) &&
           vr_weapon::weapon_placed_by_game() && vr_weapon::written_socket(sock_w,nullptr)) {
          for(int k=0;k<3;++k) sock_w[12+k]-=1024.f*cell[k];
          {float precise[16]; if(vr_weapon::written_socket_in_cell(precise,cell)) std::memcpy(sock_w,precise,64);}
          float h_sup[16],h_dom[16];
          if(holds_for(dom_now,sh,h_sup,h_dom)) {
            float pinned[16],own[16],mixed[16];
            skin_arm_ik::multiply_affine(pinned,h_sup,sock_w);
            std::memcpy(own,aligned,64); own[12]=point.x; own[13]=point.y; own[14]=point.z; own[15]=1.f;
            vr_arms_math::blend_affine(own,pinned,w,mixed);
            bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v);
            if(fin) {std::memcpy(aligned,mixed,64); point={mixed[12],mixed[13],mixed[14]}; oriented=true; two_hand_pins.fetch_add(1,std::memory_order_relaxed);}
          }
        }
      }
      if(clean_done && t_clean_ok) {
        IkTraceRow tr; tr.phase="solve"; tr.hand=static_cast<int>(hand); tr.sample=&t_clean;
        std::memcpy(tr.head_used,head,sizeof(head));
        for(int k=0;k<3;++k) {
          tr.body[k]=body_position[k]; tr.shoulder[k]=snapshot[hand?113:14][12+k]+1024.f*cell[k];
          tr.free_target[k]=((&free_point.x)[k])+1024.f*cell[k];
          tr.final_target[k]=((&point.x)[k])+1024.f*cell[k];
        }
        ik_trace(tr);
      }
      const float outward=hand?0.4f:-0.4f;
      skin_arm_ik::Vec prev_bend{0,0,0}; bool prev_valid=false;
      {std::lock_guard<std::mutex> lock(pole_mutex); prev_bend=last_bend[hand]; prev_valid=last_bend_valid[hand];}
      const skin_arm_ik::Vec pole{outward*root_matrix[0]-root_matrix[4]-0.2f*root_matrix[8],
        outward*root_matrix[1]-root_matrix[5]-0.2f*root_matrix[9],outward*root_matrix[2]-root_matrix[6]-0.2f*root_matrix[10]};
      // Elbow continuity: bias towards last frame's bend so a straight raised arm does not flip the elbow inward.
      const skin_arm_ik::Vec pole_used=prev_valid?skin_arm_ik::add(skin_arm_ik::scale(pole,1.f/std::max(1e-4f,skin_arm_ik::length(pole))),
                                                                 skin_arm_ik::scale(prev_bend,2.f)):pole;
      const bool applied=skin_arm_ik::apply(copy,snapshot,hand,point,pole_used);
      if(applied) {
        const unsigned up=hand?113:14;
        const skin_arm_ik::Vec s0=skin_arm_ik::position(copy[up]),e0=skin_arm_ik::position(copy[up+1]),w0=skin_arm_ik::position(copy[up+2]);
        skin_arm_ik::Vec ax=skin_arm_ik::sub(w0,s0); const float la=skin_arm_ik::length(ax);
        if(la>1e-4f) {
          ax=skin_arm_ik::scale(ax,1.f/la);
          skin_arm_ik::Vec b=skin_arm_ik::sub(skin_arm_ik::sub(e0,s0),skin_arm_ik::scale(ax,skin_arm_ik::dot(skin_arm_ik::sub(e0,s0),ax)));
          const float lb=skin_arm_ik::length(b);
          std::lock_guard<std::mutex> lock(pole_mutex);
          if(lb>0.02f) {last_bend[hand]=skin_arm_ik::scale(b,1.f/lb); last_bend_valid[hand]=true;}
        }
      }
      if(applied && ik_trace_active.load(std::memory_order_acquire)) {  // read-only: what the position columns cannot show
        static float last_wrist[2][16],last_elbow[2][16]; static bool last_arm_have[2]{};
        float inverse_root[16],wrist_local[16],elbow_local[16];
        if(skin_arm_ik::inverse_affine(inverse_root,root_matrix)) {
          skin_arm_ik::multiply_affine(wrist_local,copy[wrist],inverse_root); skin_arm_ik::multiply_affine(elbow_local,copy[wrist-1],inverse_root);
          std::lock_guard<std::mutex> lock(pole_mutex);
          if(last_arm_have[hand]) {
            IkTraceRow tr; tr.phase="arm"; tr.hand=static_cast<int>(hand); tr.sample=t_clean_ok?&t_clean:nullptr;
            for(int r=0;r<3;++r) for(int c=0;c<3;++c) tr.rotation_trace+=wrist_local[r*4+c]*last_wrist[hand][r*4+c];
            float d=0.f; for(int k=0;k<3;++k) d+=(elbow_local[12+k]-last_elbow[hand][12+k])*(elbow_local[12+k]-last_elbow[hand][12+k]);
            tr.source_delta=std::sqrt(d);
            for(int k=0;k<3;++k) tr.body[k]=body_position[k];
            ik_trace(tr);
          }
          std::memcpy(last_wrist[hand],wrist_local,64); std::memcpy(last_elbow[hand],elbow_local,64); last_arm_have[hand]=true;
        }
      }
      if(applied){
        {const unsigned up=hand?113:14;  // read-only: near 180 deg from the rest direction the shortest-arc swing has no stable axis
          const skin_arm_ik::Vec a=skin_arm_ik::sub(skin_arm_ik::position(snapshot[up+1]),skin_arm_ik::position(snapshot[up])),b=skin_arm_ik::sub(skin_arm_ik::position(copy[up+1]),skin_arm_ik::position(copy[up]));
          const float la=skin_arm_ik::length(a),lb=skin_arm_ik::length(b);
          if(la>1e-4f && lb>1e-4f && skin_arm_ik::dot(a,b)/(la*lb)<-0.95f) rig_antipodal.fetch_add(1,std::memory_order_relaxed);}
        float rigid[16]; std::memcpy(rigid,copy[wrist],sizeof(rigid)); // hand carried rigidly by the solved forearm
        if(oriented) for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) copy[wrist][i*4+j]=aligned[i*4+j];
        if(sides) {
          int owner[211];
          const unsigned assigned=skin_arm_ik::owners_from_roles(owner,bone_side,bone_role,hand);
          const unsigned moved=skin_arm_ik::follow(copy,snapshot,owner);
          // Forearm twist: the wrist's roll about the forearm is shared by the twist bones by their position along it.
          if(forearm_twist && oriented) {
            const unsigned elbow=wrist-1;
            const skin_arm_ik::Vec e=skin_arm_ik::position(copy[elbow]),w=skin_arm_ik::position(copy[wrist]);
            skin_arm_ik::Vec axis=skin_arm_ik::sub(w,e); const float len=skin_arm_ik::length(axis);
            if(len>1e-4f) {
              axis=skin_arm_ik::scale(axis,1.f/len);
              float twist=skin_arm_ik::twist_angle(rigid,copy[wrist],axis);
              if(rig_coherent) {  // atan2 wraps at 180 deg: without this a roll past it spins the twist bones a whole turn in one frame
                static float twist_last[2]{}; static bool twist_have[2]{};
                std::lock_guard<std::mutex> lock(pole_mutex);
                if(twist_have[hand]) {const float wrapped=twist; twist=rider_ik::continuous_twist(twist,twist_last[hand]);
                  if(std::fabs(twist)>6.2831853f) twist=wrapped;
                  if(std::fabs(twist-wrapped)>3.f) rig_twist_wraps.fetch_add(1,std::memory_order_relaxed);}
                twist_last[hand]=twist; twist_have[hand]=true;
              }
              const unsigned shoulder_j=wrist-2;
              const skin_arm_ik::Vec sj=skin_arm_ik::position(copy[shoulder_j]);
              skin_arm_ik::Vec uax=skin_arm_ik::sub(e,sj); const float ul=skin_arm_ik::length(uax);
              for(unsigned bone=0;bone<211;++bone) if(bone_twist[bone] && bone_side[bone]==static_cast<signed char>(hand)) {
                if(bone_twist[bone]==1) {
                  const float t=std::clamp(skin_arm_ik::dot(skin_arm_ik::sub(skin_arm_ik::position(copy[bone]),e),axis)/len,0.f,1.f);
                  skin_arm_ik::twist_bone(copy[bone],axis,twist*t);
                } else if(ul>1e-4f) {
                  // Upper-arm twist bones take a share of the wrist roll (AAA-style arm roll), more towards the elbow.
                  const skin_arm_ik::Vec ua=skin_arm_ik::scale(uax,1.f/ul);
                  const float t=std::clamp(skin_arm_ik::dot(skin_arm_ik::sub(skin_arm_ik::position(copy[bone]),sj),ua)/ul,0.f,1.f);
                  skin_arm_ik::twist_bone(copy[bone],ua,twist*upper_twist_share*t);
                }
              }
            }
          }
          if(!(follow_logged.fetch_or(1u<<hand)&(1u<<hand))) log_followers(owner,hand,assigned,moved);
        }
      }
      changed=applied||changed;
    }
  return changed;
}
// Per-frame FP result shared by the FP body and the gear meshes: whichever palette build comes first in a frame solves,
// the rest reuse it, so the gear never uses the previous frame's arms. frame_gen advances on Present.
// [HEADSET run 30] the gun hand (placed camera-rigid) no longer trails, but both shoulders and the left arm still slide
// back/forward while moving: the FP rig comes from the game frame being drawn, the grips from the camera of the aim
// sample (trace: CameraMarker vs camera up to 0.17 m in some running frames). "Lock body to view": each frame the FP
// rig (and the gear/shadow skinned with it) is shifted so its CameraMarker (bone 1) sits on the camera the grips were
// built in; the arms then solve from shoulders and grips in one camera frame.
bool rig_lock=false;  // [HEADSET run 32] shifting the skinned body per frame flickered it and detached gear: removed
bool live_grips=true;  // run 31: read the grips at solve time (not the one-frame-old Present copy)
// [AUTHOR run 33] "Stabilize hands while moving": the game camera bobs up to ~15 cm against the body while running or
// jumping (trace run 28); hands and gun placed in that camera bounce. The camera position used for the grips and the
// gun placement becomes body + a smoothed (camera - body) offset (time constant ~0.25 s); rotation is unchanged.
bool hand_stabilize=true;
std::mutex stab_mutex; float stab_offset[3]{}; bool stab_have=false; LARGE_INTEGER stab_last{};
// Updates the smoothed camera-body offset with this camera/body pair (any thread) and returns the stabilized position.
void stabilized_position(const float* cam_pos,const float* body,float* out) {
  std::lock_guard<std::mutex> lock(stab_mutex);
  LARGE_INTEGER now,freq; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&freq);
  const float dt=stab_last.QuadPart?static_cast<float>(now.QuadPart-stab_last.QuadPart)/static_cast<float>(freq.QuadPart):0.f;
  stab_last=now;
  float off[3]; for(int k=0;k<3;++k) off[k]=cam_pos[k]-body[k];
  float d2=0.f; for(int k=0;k<3;++k) d2+=(off[k]-stab_offset[k])*(off[k]-stab_offset[k]);
  if(!stab_have || d2>1.f || !(dt>0.f) || dt>0.5f) {std::memcpy(stab_offset,off,12); stab_have=true;}
  else {const float a=1.f-std::exp(-dt/0.25f); for(int k=0;k<3;++k) stab_offset[k]+=a*(off[k]-stab_offset[k]);}
  for(int k=0;k<3;++k) out[k]=body[k]+stab_offset[k];
}
bool frame_cam=true;   // run 32: grips re-expressed in the drawn frame's camera (menu "Lock body to view" now drives this)
std::mutex frame_cam_mutex; float frame_cam_conversion[16]{}; bool frame_cam_have=false; bool frame_cam_still=false;
float frame_cam_ref_body[3]{}; ULONGLONG frame_cam_ref_tick=0;
std::atomic<unsigned> frame_cam_calibrations{0},frame_cam_applied{0};
std::atomic<unsigned> live_grip_reads{0};
float fp_shift_last[3]{}; bool fp_shift_valid=false;  // fp_last_mutex
std::atomic<unsigned> rig_shifts{0};
unsigned fp_last_gen=0; // fp_last_mutex
// [LIVE run 44 GUN_TIMING] order per tick: E (gun placed) -> W (game writes gun pose, 0x142C398A5) -> S/H (arm solve takes
// the socket) -> G (gun draws): gun-hand 0.5 mm. Game ticks and Presents both run ~69 Hz but drift: in 11/123 frames a
// second tick placed and posed the gun after the solve, and the following draws showed the new gun with the hand still
// cached from the old tick (up to 7.8 cm). "Arms follow every game tick": the cached solve is also keyed by the weapon
// placement tick; a new tick re-solves with the SAME controller sample as this frame's first solve (run 41 re-read live
// grips per re-solve -> left-hand flicker), so only the body pose and the gun socket advance. Normal frames unchanged.
bool tick_resolve=true;
unsigned fp_last_tick=0; GripCache fp_last_target; bool fp_last_target_valid=false;  // fp_last_mutex
std::atomic<unsigned> tick_resolves{0};
bool shared_fp(const float* root_matrix,const std::int16_t* cell,const float* body_position,const void* source,
               ULONGLONG now,float (&raw)[211][16],float (&out)[211][16],float* out_root,bool* solved) {
  const unsigned gen=frame_gen.load();
  const bool gun_placed=vr_weapon::weapon_placed_by_game();
  // Armed: the placement tick, unchanged. Without a gun no placement counts ticks, so a tick starting mid-frame was never re-solved.
  // [LIVE Nov 7 15:08 trace, moving] armed frames re-solved for the new tick in 81% of generations, bare in 0%: the camera-post counter
  // did not change between those solves, so bare frames drew with the last tick's arms carried by the body (elbow step p95 2.06 cm
  // against 0.95 armed). Bare hands now re-solve on the camera step's sample publication, within 0.5 ms of where the gun is placed.
  const unsigned tick=gun_placed||!rig_coherent?vr_weapon::placement_tick():vr_arms::published_steps();
  *solved=false;
  GripCache reuse; bool have_reuse=false;
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    if(fp_last_gen==gen && fp_last_at) {
      if(!tick_resolve || fp_last_tick==tick || !fp_last_target_valid || (!gun_placed && !rig_coherent)) {
        std::memcpy(raw,fp_raw_last,sizeof(raw)); std::memcpy(out,fp_out_last,sizeof(out)); std::memcpy(out_root,fp_root_last,64);
        return true;
      }
      reuse=fp_last_target; have_reuse=true;
    }}
  // Clean rig: the last good rig is reused whenever this solve cannot run (never the game's flat arms for one frame).
  auto reuse_last=[&]()->bool {
    if(!clean_rig) return false;
    std::lock_guard<std::mutex> lock(fp_last_mutex);
    if(!fp_last_at || now-fp_last_at>500) return false;
    std::memcpy(raw,fp_raw_last,sizeof(raw)); std::memcpy(out,fp_out_last,sizeof(out)); std::memcpy(out_root,fp_root_last,64);
    rig_reuse.fetch_add(1,std::memory_order_relaxed);
    return true;
  };
  GripCache target;
  t_clean_ok=false;
  if(clean_rig) {
     // The gun of this tick was placed from a sample: both hands use that sample and its exact anchor offset while it is
     // fresh. [LIVE 2026-10-30] a placement at the end of the previous Present generation otherwise met a newer sample
     // here: the shoulder stepped against the gun and the pinned hand gapped up to 5.8 cm for a frame.
     {std::lock_guard<std::mutex> lock(clean_mutex);
       if(gun_record.valid && clean_qpc_us()-gun_record.us<50000) {
         t_clean=gun_record.sample; std::memcpy(t_clean_view,gun_record.view_offset,12); t_clean_ok=true;}}
     if(!t_clean_ok && rig_coherent) {
       // [HEADSET 2026-11-07] both arms are steadiest with a gun out. [LIVE 10-31 trace] the one input difference: armed solves take the
       // sample from the gun placement, just before the camera step (age p50 15-17 ms, every frame the same step), bare solves the newest
       // (p50 0.1 ms, whichever step the first draw met). Bare hands now take the armed choice; the gun record above still wins when fresh.
       t_clean_ok=vr_arms::armed_sample(t_clean,t_clean_view);
     }
     if(!t_clean_ok) {
       t_clean_ok=ik_frame_sample_lock.load(std::memory_order_relaxed)?clean_input_sample(t_clean):vr_arms::latest(t_clean);
       if(t_clean_ok) std::memcpy(t_clean_view,vr_arms::anchor_view_offset(t_clean),12);
     }
    if(t_clean_ok) {std::lock_guard<std::mutex> lock(hud_solve_mutex); hud_solve_sample=t_clean; std::memcpy(hud_solve_view,t_clean_view,12); hud_solve_ok=true;}
    vr_arms::set_smoothing(hand_smoothing);
    vr_arms::set_view_lock(arms_lock_to_view && view_height_anchor);
    vr_arms::set_rotation_tuned(rig_coherent);
    if(!t_clean_ok) return reuse_last();
    target.tick=now;
    for(unsigned hand=0;hand<2;++hand) {
      target.valid[hand]=t_clean.valid[hand];
      if(!native_stereo::grip_transform(static_cast<int>(hand),target.matrix[hand])) std::memset(target.matrix[hand],0,64);  // T-pose calibration only
    }
    target.head_valid=native_stereo::head_position(target.head);
    if(side_state.load()!=2 || !follow_enabled) return reuse_last();
  } else {
  if(have_reuse) target=reuse;
  else {std::unique_lock<std::mutex> lock(grip_mutex,std::try_to_lock);
    if(!lock.owns_lock()) return false;
    target=grips;}
  if((!have_reuse && now-target.tick>100) || side_state.load()!=2 || !follow_enabled) return false;
  }
  // [LIVE run 31 trace] running: the rig shifted to the cached grip camera sat 7.5 cm (mean) from the live camera, while
  // the latest aim camera was 0.7 cm from it: the Present-time grip copy is one frame old (one frame of travel), so the
  // left arm and both shoulders trailed the view. The grips are now read live at solve time (the cache stays as the
  // fallback when a live read fails).
  if(live_grips && !have_reuse && !clean_rig) {
    GripCache live; live.tick=now; bool any=false;
    for(unsigned hand=0;hand<2;++hand) {live.valid[hand]=native_stereo::grip_transform(static_cast<int>(hand),live.matrix[hand]); any=any||live.valid[hand];}
    live.body_valid=native_stereo::grip_body(live.body);
    live.head_valid=native_stereo::head_position(live.head);
    live.yaw_valid=native_stereo::grip_yaw(&live.yaw);
    live.cam_valid=native_stereo::grip_camera(live.cam);
    if(any) {target=live; live_grip_reads.fetch_add(1);}
  }
  const GripCache sample=target;  // controller sample before the per-frame camera mapping (reused by a tick re-solve)
  static thread_local float snapshot[211][16];
  if(!source || !read(source,0,snapshot,sizeof(snapshot))) return clean_rig?reuse_last():false;
  // [HEADSET runs 28-32] Root cause of the hand/shoulder slide, bounce and flicker: the first-person body (and the view)
  // belong to the game frame being drawn, while the grips are built in the camera of a separate aim sample that is
  // sometimes a frame newer or older. The grips are therefore re-expressed in the drawn frame's own camera:
  //   grip = grip_sampled * inverse(sample camera) * (CameraMarker of this snapshot * conversion)
  // conversion = inverse(CameraMarker) * game camera, measured while standing still (same frame then). Hands,
  // shoulders, gear and the gun placement (fresh_cam) then share one camera by construction.
  if(frame_cam && target.cam_valid && !clean_rig) {
    float mark[16]; std::memcpy(mark,snapshot[1],64); for(int k=0;k<3;++k) mark[12+k]+=1024.f*cell[k];
    float body_world[3]; for(int k=0;k<3;++k) body_world[k]=root_matrix[12+k]+1024.f*cell[k];
    {std::lock_guard<std::mutex> lock(frame_cam_mutex);
      // calibration: body still (< 3 mm over >= 100 ms) and grip camera close to the marker (same frame)
      if(!frame_cam_ref_tick || now-frame_cam_ref_tick>=100) {
        float mv=0.f; for(int k=0;k<3;++k) mv+=(body_world[k]-frame_cam_ref_body[k])*(body_world[k]-frame_cam_ref_body[k]);
        frame_cam_still=frame_cam_ref_tick && mv<0.003f*0.003f;
        std::memcpy(frame_cam_ref_body,body_world,12); frame_cam_ref_tick=now;
      }
      float inverse_mark[16];
      if(frame_cam_still && skin_arm_ik::inverse_affine(inverse_mark,mark)) {
        float d=0.f; for(int k=0;k<3;++k) d+=(target.cam[12+k]-mark[12+k])*(target.cam[12+k]-mark[12+k]);
        if(d<0.05f*0.05f) {skin_arm_ik::multiply_affine(frame_cam_conversion,inverse_mark,target.cam); frame_cam_have=true; frame_cam_calibrations.fetch_add(1);}
      }
      if(frame_cam_have && model_space_hands) {
        float drawn_cam[16],inverse_sample[16];
        skin_arm_ik::multiply_affine(drawn_cam,mark,frame_cam_conversion);
        if(skin_arm_ik::inverse_affine(inverse_sample,target.cam)) {
          skin_arm_ik::multiply_affine(target.map,inverse_sample,drawn_cam);
          target.map[12]=target.map[13]=target.map[14]=0.f; target.map_valid=true;
        }
      } else if(frame_cam_have) {
        float drawn_cam[16],inverse_sample[16],rel[16];
        skin_arm_ik::multiply_affine(drawn_cam,mark,frame_cam_conversion);
        if(hand_stabilize) {float sp[3]; stabilized_position(drawn_cam+12,body_world,sp); for(int k=0;k<3;++k) drawn_cam[12+k]=sp[k];}
        if(skin_arm_ik::inverse_affine(inverse_sample,target.cam)) {
          skin_arm_ik::multiply_affine(rel,inverse_sample,drawn_cam);
          float jump=0.f; for(int k=0;k<3;++k) jump+=(drawn_cam[12+k]-target.cam[12+k])*(drawn_cam[12+k]-target.cam[12+k]);
          if(jump<0.5f*0.5f) {
            for(unsigned hand=0;hand<2;++hand) if(target.valid[hand]) {float g[16]; skin_arm_ik::multiply_affine(g,target.matrix[hand],rel); std::memcpy(target.matrix[hand],g,64);}
            if(target.head_valid) {float h[16]{}; h[0]=h[5]=h[10]=h[15]=1.f; for(int k=0;k<3;++k) h[12+k]=target.head[k];
              float hm[16]; skin_arm_ik::multiply_affine(hm,h,rel); for(int k=0;k<3;++k) target.head[k]=hm[12+k];}
            // [AUTHOR run 34] "the logic is fighting": the old compensations (Steady hands: + live body travel; turn
            // compensation: + live look-yaw change) re-added motion measured against the NEWEST game state on top of
            // grips that now belong to the drawn frame, pushing the hands a frame ahead. Both are off for re-expressed grips.
            target.body_valid=false; target.yaw_valid=false;
            std::memcpy(target.cam,drawn_cam,64);
            frame_cam_applied.fetch_add(1);
          }
        }
      }
    }
  }
  float shift[3]{}; bool shifted=false;
  if(rig_lock && target.cam_valid) {
    for(int k=0;k<3;++k) shift[k]=target.cam[12+k]-(snapshot[1][12+k]+1024.f*cell[k]);
    const float len=std::sqrt(shift[0]*shift[0]+shift[1]*shift[1]+shift[2]*shift[2]);
    if(std::isfinite(len) && len<0.5f) {
      for(auto& b:snapshot) for(int k=0;k<3;++k) b[12+k]+=shift[k];
      shifted=true; rig_shifts.fetch_add(1);
    }
  }
  vr_weapon::timing_mark('S');
  if(!solve_fp(snapshot,raw,out,root_matrix,cell,body_position,target,now,true)) return clean_rig?reuse_last():false;
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    std::memcpy(fp_shift_last,shift,12); fp_shift_valid=shifted;
    std::memcpy(fp_raw_last,raw,sizeof(raw)); std::memcpy(fp_out_last,out,sizeof(out)); std::memcpy(fp_root_last,root_matrix,64);
    fp_last_at=now; fp_last_gen=gen; fp_last_tick=tick;
    if(!have_reuse) {fp_last_target=sample; fp_last_target_valid=true;} else tick_resolves.fetch_add(1);
    for(unsigned k=0;k<3;++k){fp_body_last[k]=body_position[k]-1024.f*cell[k]; fp_cell_last[k]=cell[k];}}
  std::memcpy(out_root,root_matrix,64);
  *solved=true;
  return true;
}
// [LIVE] 2026-10-15: on the full body RNF_BODY_01_M, nearest-arm anchoring (1 m) pulled 85 of its own bones (legs, tail,
// hair...) onto the hands. Bones whose own name marks a body part are never anchored to the arms.
bool body_part_name(const char* name) {
  if(!name || !name[0]) return false;
  // [LIVE] 2026-10-15 (attachment research session): the cord and pouch left at the flat hands are songcord bones
  // C_accessory* / C_accessory_string*; the C_ rule wrongly kept them on the body pose. Accessory bones may anchor.
  if(std::strstr(name,"accessory")) return false;
  if(!std::strncmp(name,"C_",2)) return true; // centre line: pelvis, spine, neck, head, hair, tail
  static const char* parts[]={"leg","Leg","thigh","Thigh","knee","Knee","calf","Calf","foot","Foot","toe","Toe","ankle","Ankle",
    "hip","Hip","tail","Tail","spine","Spine","neck","Neck","head","Head","hair","Hair","pelvis","Pelvis","butt","Butt","glute",
    "Glute","breast","Breast","belly","Belly","eye","Eye","jaw","Jaw","clavicle","Clavicle","shoulder",
    "Shoulder","chest","Chest","stomach","Stomach","abdomen","Abdomen"}; // no "ear"/"rib"/"pec": they match forearm/ribbon
  for(const char* part:parts) if(std::strstr(name,part)) return true;
  return false;
}
bool read_path(const void* root,char (&path)[128]) {
  const void* resource=nullptr; unsigned char name[16]{};
  if(!read(root,0x58,&resource,sizeof(resource)) || !read(resource,0x18,name,sizeof(name))) return false;
  if(name[15]) {
    const void* heap=nullptr; std::memcpy(&heap,name,sizeof(heap));
    for(unsigned i=0;i<sizeof(path)-1;++i) {if(!read(heap,i,path+i,1)) return false; if(!path[i]) return true;}
    path[127]=0; return true;
  }
  std::memcpy(path,name,15); path[15]=0; return true;
}
// Non-211 meshes near the player while IK runs: bones that are exact copies of FP bones take the FP bone's IK result.
// Weapons (2026-10-15): [LIVE] held weapons are skinned through this builder too (WPN_P_ASSAULT_RIFLE_FRAME_01 n=4,
// WPN_P_SHOTGUN_FRAME_01 n=5, roots ~2 m above the feet). The game places a held weapon from its animated
// *_weaponAttachmentHand bone, so a weapon whose bones lie within 15 cm of that bone (raw FP pose) is held by that hand
// and takes the same rigid change the IK gave the attachment bone: native grip, IK hand pose. Holstered weapons stay.
std::atomic<unsigned> weapon_apps{0};
// "Weapon effects follow tracked gun (experimental)" (2026-10-15): [HEADSET] with the shot producer moved (vr_weapon),
// aim follows the tracked gun but tracer/smoke/flash still start at the flat gun. Strings MuzzleNode/tracerEffect suggest
// the effects attach to a node of the weapon model, i.e. the weapon's own pose array (`source`), which the IK never
// changed (renderer copy only). On: the IK weapon bones are also written into that game array. If the game did not
// rebuild it before the next palette build (content == what we wrote), the remembered flat bones are used instead, so
// the IK change is never applied twice. Off at every launch.
bool weapon_pose_write=false;
struct WeaponWrite { const void* source=nullptr; unsigned n=0; float raw[8][16]; float written[8][16]; };
WeaponWrite weapon_writes[4];
std::mutex weapon_write_mutex;
std::atomic<unsigned> weapon_pose_writes{0},weapon_pose_reuses{0};
float held_root_world[16]{}; ULONGLONG held_root_tick=0;
std::atomic<ULONGLONG> held_weapon_draw_tick{0};  // any in-hand draw of a named weapon mesh, both weapon_call paths
// Game-placed weapon latency fix (2026-10-15): per weapon mesh, its root bone relative to the IK socket the game used.
struct GripOffset { const void* mesh=nullptr; float offset[16]; bool have=false; };
// [HEADSET run 17] only the skinned frame reaches this redraw; the rigid parts stay at the game's placement, so the gun
// split apart. Off: every part renders where the game places it (kept for reference / a future rigid-part redraw).
bool grip_redraw=false;  // [HEADSET run 20] redraw + feedback split the gun (two draws, rotation); replaced by hand-on-gun (run 21)
bool weapon_predict=false; // [HEADSET run 18] wobbled while standing (jitter amplified). One-solve-ahead prediction of the game placement (weapon_attach_matrices)
GripOffset grip_offsets[8];
std::atomic<unsigned> grip_calibrations{0},grip_redraws{0};  // flat C_root of the in-hand weapon draw, world (vr_weapon scan)
// ---- Mirror-convention probe (run 61, read-only; once per launch, ~20 s after the first FP draw) ----
// Goal: everything needed to mirror the right hand's hold onto the left (and swap finger poses) in left-handed mode.
// (1) L/R name pairs of the 211 FP pose. (2) The engine skeleton object (loader 0x1417F70E0: parentBoneIndexes +0x28/+0x30,
// mirrorBoneIndexes int16 +0x38/+0x40, mirrorBoneTranslates +0x48/+0x50) searched from the palette root record (two
// pointer levels), accepted only if its mirror table is an involution and parents precede children. (3) The axis sign
// pattern D: a bone's offset from its parent, in the parent's frame, is a constant (bone length vector); mirrored pairs
// give v_L = D * v_R. Parents from the engine table if found, else name chains (finger _01.._03, hand, arm). (4) The gun
// socket axes against the body axes (which socket axis is lateral). Only memory reads; nothing is written.
std::atomic<bool> mirror_probe_done{false}; ULONGLONG mirror_probe_after=0;
bool skeleton_candidate(const void* p, unsigned& n, std::vector<std::int16_t>& parent, std::vector<std::int16_t>& mirror,
                        std::vector<std::uint32_t>& flags) {
  const void* pp=nullptr; const void* mp=nullptr; const void* tp=nullptr; std::uint32_t pn=0,mn=0,tn=0;
  if(!read(p,0x28,&pp,8) || !read(p,0x30,&pn,4) || !read(p,0x38,&mp,8) || !read(p,0x40,&mn,4)) return false;
  if(!pp || !mp || pn!=mn || pn<20 || pn>1500) return false;
  parent.resize(pn); mirror.resize(mn);
  if(!read(pp,0,parent.data(),pn*2) || !read(mp,0,mirror.data(),mn*2)) return false;
  unsigned bad=0;
  for(unsigned i=0;i<mn;++i) {const int m=mirror[i]; if(m<0 || m>=static_cast<int>(mn) || mirror[m]!=static_cast<std::int16_t>(i)) ++bad;}
  if(bad) return false;
  unsigned order=0; for(unsigned i=0;i<pn;++i) {const int q=parent[i]; if(q>=static_cast<int>(i) && q!=-1 && q!=0xFFFF) ++order;}
  if(order>pn/20) return false;
  n=pn; flags.clear();
  if(read(p,0x48,&tp,8) && read(p,0x50,&tn,4) && tp && tn==pn) {flags.resize(tn); if(!read(tp,0,flags.data(),tn*4)) flags.clear();}
  return true;
}
void mirror_probe(const void* root_record) {
  if(mirror_probe_done.load()) return;
  const ULONGLONG now=GetTickCount64();
  if(!mirror_probe_after) {mirror_probe_after=now+20000; return;}
  if(now<mirror_probe_after || mirror_probe_done.exchange(true)) return;
  char line[400];
  // (1) name pairs
  int partner[211]; unsigned pairs=0;
  for(unsigned j=0;j<211;++j) {partner[j]=-1;}
  for(unsigned j=0;j<211;++j) {
    const std::string& a=fp_names[j]; if(a.size()<3 || a.compare(0,2,"L_")!=0) continue;
    const std::string b="R_"+a.substr(2);
    for(unsigned k=0;k<211;++k) if(fp_names[k]==b) {partner[j]=static_cast<int>(k); partner[k]=static_cast<int>(j); ++pairs; break;}
  }
  std::snprintf(line,sizeof(line),"mirror probe: %u L/R name pairs in the 211 FP pose",pairs); log(line);
  // (2) engine skeleton object from the root record (two pointer levels)
  unsigned n=0; std::vector<std::int16_t> parent,mirror; std::vector<std::uint32_t> flags; const void* found=nullptr; unsigned found_off=0,found_off2=0;
  for(unsigned off=0;off<0x200 && !found;off+=8) {
    const void* p=nullptr; if(!read(root_record,off,&p,8) || !p) continue;
    if(skeleton_candidate(p,n,parent,mirror,flags)) {found=p; found_off=off; found_off2=0xFFFF; break;}
    for(unsigned off2=0;off2<0x100;off2+=8) {
      const void* q=nullptr; if(!read(p,off2,&q,8) || !q) continue;
      if(skeleton_candidate(q,n,parent,mirror,flags)) {found=q; found_off=off; found_off2=off2; break;}
    }
  }
  if(found) {
    unsigned self=0; for(unsigned i=0;i<n;++i) if(mirror[i]==static_cast<std::int16_t>(i)) ++self;
    std::snprintf(line,sizeof(line),"mirror probe: engine skeleton at root+0x%X%s%s: %u bones, mirror involution OK, %u self-mirrored, translate flags %s",
      found_off,found_off2==0xFFFF?"":" -> +0x",found_off2==0xFFFF?"":std::to_string(found_off2).c_str(),n,self,flags.empty()?"absent":"present");
    log(line);
    if(!flags.empty()) {std::string h="mirror probe: translate flag values:"; std::uint32_t seen[8]; unsigned cnt[8]{}, ns=0;
      for(auto v:flags) {unsigned k=0; for(;k<ns;++k) if(seen[k]==v) break; if(k==ns && ns<8) {seen[ns]=v; ++ns;} if(k<8) ++cnt[k];}
      for(unsigned k=0;k<ns;++k) {char it[48]; std::snprintf(it,sizeof(it)," 0x%X x%u",seen[k],cnt[k]); h+=it;} log(h.c_str());}
    if(n==211) {unsigned agree=0,disagree=0;
      for(unsigned j=0;j<211;++j) if(partner[j]>=0) {(mirror[j]==partner[j]?agree:disagree)++;}
      std::snprintf(line,sizeof(line),"mirror probe: engine mirror vs name pairs: agree %u, disagree %u",agree,disagree); log(line);}
  } else log("mirror probe: engine skeleton object not found from the root record (name pairs + name chains used)");
  // parents: engine table when it is the 211 skeleton, else name chains
  int par[211]; for(int& p:par) p=-1;
  if(found && n==211) for(unsigned j=0;j<211;++j) par[j]=parent[j]<211?parent[j]:-1;
  else for(unsigned j=0;j<211;++j) {
    const std::string& a=fp_names[j]; const std::size_t u=a.rfind('_');
    if(u!=std::string::npos && u+3==a.size() && a[u+1]=='0' && a[u+2]>'1' && a[u+2]<='9') {  // X_02 -> X_01
      std::string q=a; q[u+2]=static_cast<char>(a[u+2]-1); for(unsigned k=0;k<211;++k) if(fp_names[k]==q) {par[j]=static_cast<int>(k); break;}
    } else if(a.size()>3 && a.find("finger")!=std::string::npos && a.compare(a.size()-3,3,"_01")==0) {
      const std::string hand=a.substr(0,2)+"hand"; for(unsigned k=0;k<211;++k) if(fp_names[k]==hand) {par[j]=static_cast<int>(k); break;}
    } else {
      const char* chain[][2]={{"lowerArm","upperArm"},{"hand","lowerArm"}};
      for(auto& c:chain) if(a.size()>2 && a.substr(2)==c[0]) {const std::string q=a.substr(0,2)+c[1]; for(unsigned k=0;k<211;++k) if(fp_names[k]==q) par[j]=static_cast<int>(k);}
    }
  }
  // (3) D from child offsets in the parent frame
  float raw[211][16];
  {std::lock_guard<std::mutex> lock(fp_last_mutex); if(!fp_last_at) {log("mirror probe: no FP pose yet"); return;} std::memcpy(raw,fp_raw_last,sizeof(raw));}
  auto local_offset=[&](int b,int p,float* v){const skin_arm_ik::Vec d=skin_arm_ik::sub(skin_arm_ik::position(raw[b]),skin_arm_ik::position(raw[p]));
    for(int k=0;k<3;++k) {const skin_arm_ik::Vec r=skin_arm_ik::row(raw[p],k); const float l=skin_arm_ik::length(r); v[k]=l>1e-6f?skin_arm_ik::dot(d,r)/l:0.f;}};
  int same[3]{},flip[3]{}; unsigned used=0; std::string detail="mirror probe: pairs (bone: vR -> vL):";
  for(unsigned j=0;j<211;++j) {
    if(fp_names[j].compare(0,2,"R_")!=0 || partner[j]<0 || par[j]<0) continue;
    const int l=partner[j], pl=par[l]; if(pl<0 || partner[par[j]]!=pl) continue;
    float vr[3],vl[3]; local_offset(static_cast<int>(j),par[j],vr); local_offset(l,pl,vl);
    for(int k=0;k<3;++k) if(std::fabs(vr[k])>0.003f && std::fabs(vl[k])>0.003f) {(vr[k]*vl[k]>0.f?same:flip)[k]++;}
    ++used;
    if(used<=12) {char it[120]; std::snprintf(it,sizeof(it)," %s:(%.3f %.3f %.3f)->(%.3f %.3f %.3f)",fp_names[j].c_str()+2,vr[0],vr[1],vr[2],vl[0],vl[1],vl[2]); detail+=it;}
  }
  std::snprintf(line,sizeof(line),"mirror probe: D from %u child offsets: x same %d flip %d | y same %d flip %d | z same %d flip %d  => D=(%+d %+d %+d)",
    used,same[0],flip[0],same[1],flip[1],same[2],flip[2],same[0]>=flip[0]?1:-1,same[1]>=flip[1]?1:-1,same[2]>=flip[2]?1:-1);
  log(line); log(detail.c_str());
  // (4) gun socket axes vs body axes, and the L/R attach bone frames
  float sock[16];
  if(vr_weapon::written_socket(sock,nullptr)) {
    float rootm[16]; {std::lock_guard<std::mutex> lock(fp_last_mutex); std::memcpy(rootm,fp_root_last,64);}
    char g[300]; int o=std::snprintf(g,sizeof(g),"mirror probe: gun socket axes . body (right,up,fwd):");
    for(int a=0;a<3;++a) {const skin_arm_ik::Vec r=skin_arm_ik::row(sock,a); const float lr=std::max(1e-6f,skin_arm_ik::length(r));
      o+=std::snprintf(g+o,sizeof(g)-o," row%d=(%.2f %.2f %.2f)",a,
        skin_arm_ik::dot(r,skin_arm_ik::row(rootm,0))/lr/std::max(1e-6f,skin_arm_ik::length(skin_arm_ik::row(rootm,0))),
        skin_arm_ik::dot(r,skin_arm_ik::row(rootm,1))/lr/std::max(1e-6f,skin_arm_ik::length(skin_arm_ik::row(rootm,1))),
        skin_arm_ik::dot(r,skin_arm_ik::row(rootm,2))/lr/std::max(1e-6f,skin_arm_ik::length(skin_arm_ik::row(rootm,2))));}
    log(g);
  } else log("mirror probe: no gun placed at probe time (hold a gun for the socket axes)");
  if(attach_bone[0]>=0 && attach_bone[1]>=0) {
    float inv_r[16],rel[16];
    if(skin_arm_ik::inverse_affine(inv_r,raw[attach_bone[1]])) {skin_arm_ik::multiply_affine(rel,raw[attach_bone[0]],inv_r);
      std::snprintf(line,sizeof(line),"mirror probe: L attach in R attach frame: rows (%.2f %.2f %.2f)(%.2f %.2f %.2f)(%.2f %.2f %.2f) pos (%.3f %.3f %.3f)",
        rel[0],rel[1],rel[2],rel[4],rel[5],rel[6],rel[8],rel[9],rel[10],rel[12],rel[13],rel[14]); log(line);}
  }
}
void trace_weapon_parts(const void* mesh,const void* source,unsigned n,const float* hand,
                        const float* socket,const std::int16_t* cell) {
  if(n<1 || n>8) return;
  static std::mutex trace_mutex; static FILE* file=nullptr;
  static long long start_us=0,last_flush_us=0; static bool done=false;
  const long long us=clean_qpc_us();
  std::lock_guard<std::mutex> lock(trace_mutex);
  if(done) return;
  if(!start_us) {
    const std::wstring path=dir+L"\\WEAPON_IDLE_PARTS.csv";
    file=_wfopen(path.c_str(),L"w");
    if(!file) {done=true; return;}
    std::fputs("ms,gen,mesh,source,bone,body_x,body_y,body_z,hand_x,hand_y,hand_z,socket_x,socket_y,socket_z,bone_x,bone_y,bone_z,h00,h01,h02,h10,h11,h12,h20,h21,h22,b00,b01,b02,b10,b11,b12,b20,b21,b22\n",file);
    start_us=last_flush_us=us;
  }
  float mats[8][16]; if(!read(source,0,mats,n*64)) return;
  float body[3]{}; const game::Pointers p=game::pointers();
  if(p.body) game::mem_read(p.body+0x94,body,sizeof(body));
  const unsigned char* weapon=nullptr; float entity[16]{};
  const bool have_entity=p.body && game::mem_read(p.body+0x250,&weapon,sizeof(weapon)) && weapon &&
                         game::mem_read(weapon+0x3FB0,entity,sizeof(entity));
  const unsigned gen=frame_gen.load(std::memory_order_relaxed);
  for(int b=have_entity?-1:0;b<static_cast<int>(n);++b) {
    const float* m=b<0?entity:mats[b]; const float cell_shift=b<0?0.f:1024.f;
    std::fprintf(file,"%.3f,%u,%p,%p,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
      (us-start_us)*0.001f,gen,mesh,source,b,body[0],body[1],body[2],hand[12]+1024.f*cell[0],hand[13]+1024.f*cell[1],hand[14]+1024.f*cell[2],
      socket[12],socket[13],socket[14],m[12]+cell_shift*cell[0],m[13]+cell_shift*cell[1],m[14]+cell_shift*cell[2],
      hand[0],hand[1],hand[2],hand[4],hand[5],hand[6],hand[8],hand[9],hand[10],
      m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10]);
  }
  if(us-last_flush_us>1000000) {std::fflush(file); last_flush_us=us;}
  if(us-start_us>=20000000) {std::fclose(file); file=nullptr; done=true;}
}
// ---- The nocked arrow (FEATURE-14 bows, read-only, 2026-11-07) ----
// [LIVE archived SKIN_PALETTE_PROBE.log 2026-10-30] WPN_P_SHORT_BOW_LIMB_10 bones: 0=C_root 1=C_bowString 2=C_arrow_01
// 3/4=C_gripArrow_01/02 5..=C_upperLimb_*; the arrow itself is WPN_P_AMMO_BOW_HUNTING_03 (0=C_root, 1=C_arrow_01, same matrices).
// These are palette (skinning) matrices, so their axes are the bind-relative motion of the arrow, not proven bone axes. Kept as
// drawn (world: cell-local + 1024 * cell) so vr_weapon can express the game's shot in the arrow's frame and log whether it holds.
struct ArrowIndex {const void* mesh=nullptr; unsigned n=0; int arrow=-1,string=-1; char path[96]{};};
ArrowIndex arrow_index[16];
std::mutex arrow_mutex;
float arrow_world[16]{},bow_string_world[16]{},bow_root_world[16]{}; bool bow_string_have=false; ULONGLONG arrow_tick=0;
float bow_arrow_world[16]{},bow_limb_root_world[16]{}; ULONGLONG bow_tick=0;
struct ArrowStats {unsigned arrow_draws=0,bow_draws=0,other_draws=0,near_ik=0,near_flat=0,replaced=0,kept=0; float other_deg=0.f,arrow_z[3]{},bow_z[3]{};
                   float max_ik=0.f,min_flat_gap=99.f; const void* last_mesh=nullptr;};
ArrowStats arrow_stats;  // arrow_mutex
std::uint64_t arrow_frame_us=0; float arrow_frame_best=0.f;  // arrow_mutex: this frame's chosen arrow (nearest the IK socket)
// shadow_weapon_follow: the main (in-hand, gap < 0.2 m) draw of each weapon mesh this frame, world root.
struct MainDraw {const void* res=nullptr; unsigned n=0; std::uint64_t us=0; float root[16]{};};
MainDraw main_draws[8]; std::mutex shadow_mutex;
float tp_hand_raw[3]{}; std::uint64_t tp_hand_us=0;  // shadow_mutex: the full body's weapon hand, game pose, world
float body_upper_world[2][3]{}; std::uint64_t body_upper_us[2]{};  // shadow_mutex: the full body's own upperArm per side, game pose, world
std::atomic<unsigned> shadow_gear_moved{0},shadow_gear_kept{0},shadow_gear_stale{0},shadow_gear_moved_dbody_mm{0},shadow_gear_kept_gap_mm{~0u};
void atomic_max_mm(std::atomic<unsigned>& slot,float metres) {
  const unsigned v=static_cast<unsigned>(std::min(std::max(metres,0.f),100.f)*1000.f); unsigned seen=slot.load(std::memory_order_relaxed);
  while(v>seen && !slot.compare_exchange_weak(seen,v)) {}}
void atomic_min_mm(std::atomic<unsigned>& slot,float metres) {
  const unsigned v=static_cast<unsigned>(std::min(std::max(metres,0.f),100.f)*1000.f); unsigned seen=slot.load(std::memory_order_relaxed);
  while(v<seen && !slot.compare_exchange_weak(seen,v)) {}}
// shadow_gear_follow verdict per gear object and side (see accessory_call). Log every 5 s, one line per object (300 lines max):
// path, side, verdict, flips, draws by reading (shadow/fp/unclear), and the arm's game-pose upperArm in world space.
struct ShadowGearObject {const void* mesh=nullptr; int side=-1; int verdict=-1; unsigned flips=0,shadow=0,fp=0,unclear=0; float at[3]{}; char path[48]{}; unsigned n=0;};
ShadowGearObject shadow_gear_objects[96]; std::mutex shadow_gear_object_mutex; ULONGLONG shadow_gear_object_next=0; unsigned shadow_gear_object_lines=0;
bool shadow_gear_verdict(const void* mesh,const void* root,unsigned n,unsigned side,float d_body,float d_fp,const float (&g)[3]) {
  int reading=-1;
  if(d_body>=0.f && d_fp>=0.f) {
    if(d_fp>=0.05f && d_body<0.5f*d_fp) reading=1;
    else if(d_body>=0.05f && d_fp<0.5f*d_body) reading=0;
  }
  std::lock_guard<std::mutex> lock(shadow_gear_object_mutex);
  ShadowGearObject* slot=nullptr;
  for(auto& o:shadow_gear_objects) if(o.mesh==mesh && o.side==static_cast<int>(side)) {slot=&o; break;}
  if(!slot) for(auto& o:shadow_gear_objects) if(!o.mesh) {slot=&o; break;}
  if(!slot) slot=&shadow_gear_objects[(reinterpret_cast<std::uintptr_t>(mesh)/16*2+side)%96];
  if(slot->mesh!=mesh || slot->side!=static_cast<int>(side)) {
    *slot=ShadowGearObject{}; slot->mesh=mesh; slot->side=static_cast<int>(side); slot->n=n;
    char path[128]{}; if(read_path(root,path)) {const char* s=std::strrchr(path,'/'); std::snprintf(slot->path,sizeof(slot->path),"%s",s?s+1:path);}
  }
  if(reading<0) ++slot->unclear; else if(reading) ++slot->shadow; else ++slot->fp;
  if(reading>=0 && reading!=slot->verdict) {if(slot->verdict>=0) ++slot->flips; slot->verdict=reading;}
  std::memcpy(slot->at,g,12);
  const bool result=slot->verdict==1;
  const ULONGLONG now=GetTickCount64();
  if(!shadow_gear_object_next) shadow_gear_object_next=now+5000;
  if(now>=shadow_gear_object_next && shadow_gear_object_lines<300) {
    shadow_gear_object_next=now+5000;
    for(auto& o:shadow_gear_objects) {
      if(!o.mesh || !(o.shadow+o.fp+o.unclear)) continue;
      char line[256]; std::snprintf(line,sizeof(line),"shadow gear object %p %s n=%u side=%s verdict=%s flips=%u reads shadow=%u fp=%u unclear=%u upper=%.2f,%.2f,%.2f",
        o.mesh,o.path,o.n,o.side?"R":"L",o.verdict==1?"shadow":o.verdict==0?"fp":"none",o.flips,o.shadow,o.fp,o.unclear,o.at[0],o.at[1],o.at[2]);
      log(line); ++shadow_gear_object_lines; o.flips=o.shadow=o.fp=o.unclear=0;
      if(shadow_gear_object_lines>=300) break;
    }
  }
  return result;
}
std::atomic<unsigned> shadow_moves{0},shadow_misses{0},shadow_by_count{0},shadow_by_hand{0},shadow_far_left{0};
std::atomic<float> shadow_max_gap{0.f},shadow_max_hand{0.f},shadow_far_min_hand{99.f};
inline void atomic_max(std::atomic<float>& a,float v) {float p=a.load(std::memory_order_relaxed); while(v>p && !a.compare_exchange_weak(p,v)) {}}
inline void atomic_min(std::atomic<float>& a,float v) {float p=a.load(std::memory_order_relaxed); while(v<p && !a.compare_exchange_weak(p,v)) {}}
char arrow_path[96]{},bow_path[96]{};
void note_arrow(const void* mesh,const void* root,const void* cells,const float (*bones)[16],unsigned n) {
  if(n<2 || n>kMaxGearBones) return;
  ArrowIndex entry; bool known=false;
  {std::lock_guard<std::mutex> lock(arrow_mutex);
    for(const auto& a:arrow_index) if(a.mesh==mesh && a.n==n) {entry=a; known=true; break;}}
  if(!known) {
    entry.mesh=mesh; entry.n=n;
    char path[128]{};
    if(read_path(root,path) && std::strstr(path,"BOW")) {
      std::snprintf(entry.path,sizeof(entry.path),"%s",path);
      for(unsigned i=0;i<n;++i) {Sample e{};
        if(!read_name(root,i,e)) continue;
        if(!std::strcmp(e.bone_name,"C_arrow_01")) entry.arrow=static_cast<int>(i);
        if(!std::strcmp(e.bone_name,"C_bowString")) entry.string=static_cast<int>(i);}
    }
    std::lock_guard<std::mutex> lock(arrow_mutex);
    arrow_index[reinterpret_cast<std::uintptr_t>(mesh)/64%16]=entry;
  }
  if(entry.arrow<0) return;
  std::int16_t c[3]{}; if(!read(cells,0,c,sizeof(c))) return;
  const ULONGLONG now=GetTickCount64(); const std::uint64_t now_us=clean_qpc_us();
  // Distance of this draw's root from the tracked weapon socket and from the game's own (flat) socket, world.
  float d_ik=0.f,d_flat=0.f; bool have_sockets=false;
  if(entry.string<0) {
    float flat_s[16],ik_s[16];
    if(skin_palette_probe::weapon_attach_matrices(skin_palette_probe::weapon_hand_index(),flat_s,ik_s)) {
      float p[3]; for(int k=0;k<3;++k) p[k]=bones[0][12+k]+1024.f*c[k];
      auto d=[&](const float* m){float s=0.f; for(int k=0;k<3;++k) s+=(p[k]-m[12+k])*(p[k]-m[12+k]); return std::sqrt(s);};
      d_ik=d(ik_s); d_flat=d(flat_s); have_sockets=true;
    }
  }
  std::lock_guard<std::mutex> lock(arrow_mutex);
  // [HEADSET 2026-11-07 h] the mark left the arrow only while tensing. SOURCE: the bow limb mesh has its own C_arrow_01 (bone 2) and
  // wrote the same slot as the arrow mesh, last draw winning. Now kept apart: the arrow mesh (the visible arrow) wins while fresh.
  float z[3]; {const float* m=bones[entry.arrow]; const float l=std::sqrt(m[8]*m[8]+m[9]*m[9]+m[10]*m[10]);
    for(int k=0;k<3;++k) z[k]=l>1e-6f?m[8+k]/l:0.f;}
  if(entry.string>=0) {  // the bow itself (limb mesh); the arrow mesh carries no string
    std::memcpy(bow_arrow_world,bones[entry.arrow],64); std::memcpy(bow_limb_root_world,bones[0],64);
    std::memcpy(bow_string_world,bones[entry.string],64);
    for(int k=0;k<3;++k) {const float o=1024.f*c[k]; bow_arrow_world[12+k]+=o; bow_limb_root_world[12+k]+=o; bow_string_world[12+k]+=o;}
    bow_string_have=true; std::memcpy(bow_path,entry.path,sizeof(bow_path)); bow_tick=now; ++arrow_stats.bow_draws;
    std::memcpy(arrow_stats.bow_z,z,sizeof(z));
  } else {
    // [LIVE 2026-11-08 'bow capture'] ~180 arrow draws/s (~3 per frame), each from another mesh than the one before, up to 30 deg
    // apart. [LIVE census, tracker] shadows are cast by a separate third-person body (RNF_BODY_01_M) the game poses itself;
    // [HEADSET 2026-11-08] the bow's shadow does not follow the tracked bow, and the aim seems to follow that shadow. INFERENCE: one
    // of the arrows is the shadow body's. So per frame the arrow nearest the tracked (IK) weapon socket wins, not the last drawn.
    if(arrow_tick && now_us-arrow_frame_us<=8000 && arrow_stats.last_mesh && arrow_stats.last_mesh!=mesh) {  // another arrow mesh this frame
      const float d=z[0]*arrow_stats.arrow_z[0]+z[1]*arrow_stats.arrow_z[1]+z[2]*arrow_stats.arrow_z[2];
      arrow_stats.other_deg=std::max(arrow_stats.other_deg,std::acos(std::clamp(d,-1.f,1.f))*57.2958f); ++arrow_stats.other_draws;
    }
    ++arrow_stats.arrow_draws;
    if(have_sockets) {if(d_ik<d_flat) ++arrow_stats.near_ik; else ++arrow_stats.near_flat;
      arrow_stats.max_ik=std::max(arrow_stats.max_ik,d_ik); arrow_stats.min_flat_gap=std::min(arrow_stats.min_flat_gap,d_flat);}
    const bool new_frame=!arrow_tick || now_us-arrow_frame_us>8000;
    const float score=have_sockets?d_ik:0.f;
    if(new_frame || score<arrow_frame_best) {
      if(!new_frame) ++arrow_stats.replaced;
      std::memcpy(arrow_world,bones[entry.arrow],64); std::memcpy(bow_root_world,bones[0],64);
      for(int k=0;k<3;++k) {arrow_world[12+k]+=1024.f*c[k]; bow_root_world[12+k]+=1024.f*c[k];}
      std::memcpy(arrow_path,entry.path,sizeof(arrow_path)); arrow_tick=now; arrow_frame_best=score;
      if(new_frame) arrow_frame_us=now_us;
    } else ++arrow_stats.kept;
    arrow_stats.last_mesh=mesh; std::memcpy(arrow_stats.arrow_z,z,sizeof(z));
  }
}
bool weapon_call(void* mesh,const void* root,const void* bones,const void* remap,const void* cells,unsigned char* header,
                 const void* source,unsigned n,const game::Pointers& player,AccessoryMap& map,const float (&raw)[211][16],
                 const float (&out)[211][16],const float* out_root,const float* root_matrix) {
  if(attach_bone[0]<0 || attach_bone[1]<0)
    for(unsigned j=0;j<211;++j) {
      if(fp_names[j]=="L_weaponAttachmentHand") attach_bone[0]=static_cast<int>(j);
      if(fp_names[j]=="R_weaponAttachmentHand") attach_bone[1]=static_cast<int>(j);
    }
  if(attach_bone[0]<0 || attach_bone[1]<0) return original(mesh,root,bones,remap,cells);
  // [HEADSET 2026-10-15] with "Weapon effects follow tracked gun" the game itself builds the weapon at the IK socket;
  // applying the hand change here as well moved it twice ("moves very weird"). Render the game's pose unchanged then.
  static thread_local float mats[kMaxGearBones][16];
  if(vr_weapon::weapon_placed_by_game() && !grip_redraw) {
    // the game placed it at the IK socket and it renders unchanged: the source pose is the drawn pose
    float root0[16],used_world[16],motion=1.f; std::int16_t c[3]{};
    if(n>=1 && read(source,0,root0,sizeof(root0)) && vr_weapon::written_socket(used_world,&motion) && read(cells,0,c,sizeof(c))) {
      float used[16]; std::memcpy(used,used_world,64); for(int k=0;k<3;++k) used[12+k]-=1024.f*c[k];
      {float precise[16]; if(vr_weapon::written_socket_in_cell(precise,c)) std::memcpy(used,precise,64);}
      const float gap=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(root0),skin_arm_ik::position(used)));
      // [LIVE 2026-11-07 probe] some captures sat ~5 m from the shot: a bow not in hand. Only the held one (same 0.8 m as below).
      if(gap<0.8f && n>=2 && n<=8 && read(source,0,mats,static_cast<std::size_t>(n)*64)) note_arrow(mesh,root,cells,mats,n);
      // Ownership/root agreement is still required; the native grip-space
      // collector itself rejects equip/reload motion over its 600 ms window.
      if(gap<.2f && !mount_camera::rider_view()) mounted_learn_support(root,raw,root0,used);
      // Diagnostic (run 23): turning with the stick still detaches gun and hand. For each in-hand draw (gap < 0.8 m),
      // compare the socket implied by that draw's root (its own still-measured offset) with the hand socket drawn this
      // frame (out[attach]); log 4/s with the look-yaw rate. Tells which draw is visible and what moves when turning.
      if(gap<0.8f) {
        struct DrawCal { const void* src=nullptr; float offset[16]; bool have=false; };
        static DrawCal cals[4]; static ULONGLONG next_diag=0; static float last_yaw=0.f; static ULONGLONG last_yaw_t=0;
        DrawCal* d=nullptr; for(auto& q:cals) if(q.src==source) {d=&q; break;}
        if(!d) for(auto& q:cals) if(!q.src) {d=&q; d->src=source; break;}
        float inverse_used2[16];
        if(d && motion<0.003f && skin_arm_ik::inverse_affine(inverse_used2,used)) {skin_arm_ik::multiply_affine(d->offset,root0,inverse_used2); d->have=true;}
        if(d && d->have && ik_trace_active.load(std::memory_order_acquire)) {
          float inverse_off[16],implied[16];
          if(skin_arm_ik::inverse_affine(inverse_off,d->offset)) {
            skin_arm_ik::multiply_affine(implied,inverse_off,root0);
            const int h=left_handed?0:1;
            const float* drawn=out[attach_bone[h]];
            const float miss=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(implied),skin_arm_ik::position(drawn)));
            if(miss>0.02f) {
              IkTraceRow tr; tr.phase="weapon_draw_gap"; tr.hand=h; tr.object_id=source;
              tr.gap=miss; tr.source_delta=gap;
              for(int k=0;k<3;++k) {tr.final_target[k]=drawn[12+k]+1024.f*c[k]; tr.object[k]=implied[12+k]+1024.f*c[k];}
              ik_trace(tr);
            }
          }
        }
        const ULONGLONG now_d=GetTickCount64();
        if(d && d->have && now_d>=next_diag) {
          next_diag=now_d+250;
          float inverse_off[16],implied[16];
          if(skin_arm_ik::inverse_affine(inverse_off,d->offset)) {
            skin_arm_ik::multiply_affine(implied,inverse_off,root0);
            const int h=(left_handed?0:1);
            const float* drawn=out[attach_bone[h]];
            const float miss=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(implied),skin_arm_ik::position(drawn)));
            float tr=0.f; for(int r=0;r<3;++r) tr+=implied[r*4]*drawn[r*4]+implied[r*4+1]*drawn[r*4+1]+implied[r*4+2]*drawn[r*4+2];
            vr_arms::Sample gun_smp; bool have_gun_sample=false;
            {std::lock_guard<std::mutex> lock(clean_mutex);
              if(gun_record.valid) {gun_smp=gun_record.sample; have_gun_sample=true;}}
            IkTraceRow draw_trace; draw_trace.phase="weapon"; draw_trace.hand=h;
            draw_trace.sample=have_gun_sample?&gun_smp:nullptr;
            draw_trace.object_id=source; draw_trace.gap=miss; draw_trace.rotation_trace=tr;
            for(int k=0;k<3;++k) {draw_trace.final_target[k]=drawn[12+k]+1024.f*c[k];
              draw_trace.object[k]=implied[12+k]+1024.f*c[k];}
            ik_trace(draw_trace);
            const game::Player pl=game::read();
            float rate=0.f; if(pl.look_ok && last_yaw_t) {float dy=pl.yaw-last_yaw; while(dy>3.14159f) dy-=6.28318f; while(dy<-3.14159f) dy+=6.28318f;
              rate=dy/std::max(0.001f,(now_d-last_yaw_t)*0.001f)*57.2958f;}
            if(pl.look_ok) {last_yaw=pl.yaw; last_yaw_t=now_d;}
            static std::atomic<unsigned> diag_lines{0};
            if(diag_lines.fetch_add(1)<600) {char line[300];
              std::snprintf(line,sizeof(line),"gun vs hand: draw=%p gap=%.3f miss=%.3f m rot_trace=%.3f yaw_rate=%.0f deg/s main=%d",source,gap,miss,tr,rate,source==gun_source?1:0);
              log(line);}
          }
        }
      }
      if(gap<0.6f) {  // both in-hand draws (0.00 and ~0.35 m); holstered > 1 m
        std::lock_guard<std::mutex> lock(gun_mutex);
        {GunSlot* g=nullptr; const ULONGLONG t_now=GetTickCount64();
          for(auto& q:gun_slots) if(q.src==source) {g=&q; break;}
          if(!g) {GunSlot* oldest=&gun_slots[0]; for(auto& q:gun_slots) if(q.seen<oldest->seen) oldest=&q;
            g=oldest; g->src=source; g->valid=false; g->since=t_now;}
          g->seen=t_now; for(int k=0;k<3;++k) g->cell[k]=c[k];
          float inverse_used3[16];
          if(motion<0.003f && t_now-g->since>600 && skin_arm_ik::inverse_affine(inverse_used3,used)) {
            skin_arm_ik::multiply_affine(g->offset,root0,inverse_used3); g->valid=true;}
          float inverse_offset3[16];
          if(g->valid && gap<0.2f && skin_arm_ik::inverse_affine(inverse_offset3,g->offset)) {  // main draw: this frame's gun socket
            skin_arm_ik::multiply_affine(gun_actual_world,inverse_offset3,root0);
            for(int k=0;k<3;++k) gun_actual_world[12+k]+=1024.f*c[k];
            gun_actual_gen=frame_gen.load();
          }}
      }
      if(gap<0.2f) held_weapon_draw_tick.store(GetTickCount64(),std::memory_order_relaxed);
      if(gap<0.2f) {float g3[3]; for(int k=0;k<3;++k) g3[k]=root0[12+k]+1024.f*c[k];  // run 44 timing: G + gun bone 0 watch
        vr_weapon::timing_mark('G',g3); vr_weapon::timing_offer(source);}
      if(gap<0.2f && clean_rig) {
        const int hand=left_handed?0:1;
        trace_weapon_parts(mesh,source,n,out[attach_bone[hand]],used_world,c);
      }
      if(gap<0.2f) {  // the in-hand draw (the second draw sits ~0.35 m away; holstered > 1 m)
        std::lock_guard<std::mutex> lock(gun_mutex);
        if(gun_source!=source) {gun_source=source; gun_offset_valid=false; gun_source_since=GetTickCount64();}
        for(int k=0;k<3;++k) gun_cell[k]=c[k];
        float inverse_used[16];
        if(motion<0.003f && GetTickCount64()-gun_source_since>600 && skin_arm_ik::inverse_affine(inverse_used,used)) {
          skin_arm_ik::multiply_affine(gun_root_in_socket,root0,inverse_used); gun_offset_valid=true;}
        // Per-frame trace (run 25): what moves relative to what while walking/turning/switching. World coordinates.
        if(!trace_done) {
          const ULONGLONG t=GetTickCount64();
          if(!trace_file) {std::wstring path=dir+L"\\WEAPON_TRACE.csv"; trace_file=_wfopen(path.c_str(),L"w"); trace_start=t;
            if(trace_file) std::fputs("ms,yaw_deg,body_x,body_y,body_z,hand_x,hand_y,hand_z,used_x,used_y,used_z,root_x,root_y,root_z,source,qpc_us,cam_x,cam_y,cam_z,aim_cam_x,aim_cam_y,aim_cam_z,cammark_x,cammark_y,cammark_z\n",trace_file);}
          if(trace_file) {
            const game::Player pl=game::read(); float body[3]{}; const game::Pointers tp=game::pointers();
            if(tp.body) game::mem_read(tp.body+0x94,body,sizeof(body));
            const int th=(left_handed?0:1); const float* hd=out[attach_bone[th]];
            // run 27: + QPC microseconds, game camera at draw time, the camera the grips were built with (head_position
            // minus head_offset = desc cam_pos at the aim sample) and the FP CameraMarker bone (bone 1) of the drawn rig.
            LARGE_INTEGER qn,qf; QueryPerformanceCounter(&qn); QueryPerformanceFrequency(&qf);
            float headp[3]{},heado[3]{}; const bool hv=native_stereo::head_position(headp)&&native_stereo::head_offset(heado);
            float camm[3]{}; {std::lock_guard<std::mutex> lk(fp_last_mutex); for(int k=0;k<3;++k) camm[k]=fp_raw_last[1][12+k]+1024.f*fp_cell_last[k];}
            std::fprintf(trace_file,"%llu,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%p,%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",t-trace_start,pl.yaw*57.2958f,
              body[0],body[1],body[2],hd[12]+1024.f*c[0],hd[13]+1024.f*c[1],hd[14]+1024.f*c[2],used_world[12],used_world[13],used_world[14],
              root0[12]+1024.f*c[0],root0[13]+1024.f*c[1],root0[14]+1024.f*c[2],source,
              static_cast<long long>(qn.QuadPart*1000000/qf.QuadPart),pl.cam[12],pl.cam[13],pl.cam[14],
              hv?headp[0]-heado[0]:0.f,hv?headp[1]-heado[1]:0.f,hv?headp[2]-heado[2]:0.f,camm[0],camm[1],camm[2]);
            if(t-trace_start>25000) {std::fclose(trace_file); trace_file=nullptr; trace_done=true;}
          }
        }
        // [LIVE run 23] turning at 150-250 deg/s: hand and gun agree (miss ~0), but the gun root sits 6-7.5 cm from where
        // the placement put it (gap 0.066-0.075, 0 when the turn stops): the game's own turn lag. Report the gap between
        // the expected root (offset * used socket) and the actual root so vr_weapon pre-compensates the placement.
        float target_world[16];
        if(gun_offset_valid && vr_weapon::target_socket(target_world)) {
          float target[16]; std::memcpy(target,target_world,64); for(int k=0;k<3;++k) target[12+k]-=1024.f*c[k];
          // expected root at the uncorrected controller socket: the correction then converges to minus the sway
          float expected[16]; skin_arm_ik::multiply_affine(expected,gun_root_in_socket,target);
          const float err[3]={expected[12]-root0[12],expected[13]-root0[13],expected[14]-root0[14]};
          vr_weapon::report_render_error(err);
        }
      }
      // Shadow copy (see shadow_weapon_follow): the main draw records its root; a second in-hand draw of the same mesh
      // resource (else the same bone count) is carried rigidly onto it, keeping its own bone animation (bow draw, arrow).
      if(shadow_weapon_follow && !mount_camera::rider_view() && n<=kMaxGearBones) {
        const void* res=nullptr; read(root,0x58,&res,sizeof(res));
        const std::uint64_t us=static_cast<std::uint64_t>(clean_qpc_us());
        if(gap<0.2f) {
          std::lock_guard<std::mutex> lock(shadow_mutex);
          MainDraw* slot=nullptr;
          for(auto& m:main_draws) if(m.res==res && m.n==n) {slot=&m; break;}
          if(!slot) {slot=&main_draws[0]; for(auto& m:main_draws) if(m.us<slot->us) slot=&m;}
          slot->res=res; slot->n=n; slot->us=us; std::memcpy(slot->root,root0,64);
          for(int k=0;k<3;++k) slot->root[12+k]+=1024.f*c[k];
        } else {
          // [HEADSET 2026-11-08 k] it followed only while RT was held or crouching. LIVE: in the normal hold the second draw is over
          // 0.8 m from the socket (moves=0). So it also counts when its root is at the full (shadow) body's own weapon hand.
          float hand[3]; bool hand_ok=false;
          {std::lock_guard<std::mutex> lock(shadow_mutex); hand_ok=tp_hand_us && us-tp_hand_us<=40000; std::memcpy(hand,tp_hand_raw,12);}
          float root_world[3]; for(int k=0;k<3;++k) root_world[k]=root0[12+k]+1024.f*c[k];
          const float d_hand=hand_ok?std::sqrt((root_world[0]-hand[0])*(root_world[0]-hand[0])+(root_world[1]-hand[1])*(root_world[1]-hand[1])+
                                               (root_world[2]-hand[2])*(root_world[2]-hand[2])):99.f;
          const bool at_hand=d_hand<0.35f;
          if(gap>=0.8f && !at_hand) {if(gap<3.f) {shadow_far_left.fetch_add(1,std::memory_order_relaxed); atomic_min(shadow_far_min_hand,d_hand);}}
          else {
          MainDraw found; bool have=false,by_res=false;
          {std::lock_guard<std::mutex> lock(shadow_mutex);
            for(const auto& m:main_draws) if(m.us && us-m.us<=25000 && m.n==n && m.res==res && res) {found=m; have=by_res=true; break;}
            if(!have) for(const auto& m:main_draws) if(m.us && us-m.us<=25000 && m.n==n) {found=m; have=true; break;}}
          float inverse_root0[16],main_local[16],delta[16];
          std::memcpy(main_local,found.root,64); for(int k=0;k<3;++k) main_local[12+k]-=1024.f*c[k];
          const void* check=nullptr;
          if(have && skin_arm_ik::inverse_affine(inverse_root0,root0) && read(source,0,mats,static_cast<std::size_t>(n)*64) &&
             read(bones,0,&check,sizeof(check)) && check==source && game::pointers().body==player.body) {
            skin_arm_ik::multiply_affine(delta,inverse_root0,main_local);
            for(unsigned i=0;i<n;++i) {float r[16]; skin_arm_ik::multiply_affine(r,mats[i],delta); std::memcpy(mats[i],r,64);}
            shadow_moves.fetch_add(1,std::memory_order_relaxed); if(!by_res) shadow_by_count.fetch_add(1,std::memory_order_relaxed);
            if(gap>=0.8f) shadow_by_hand.fetch_add(1,std::memory_order_relaxed);
            if(hand_ok) atomic_max(shadow_max_hand,d_hand);
            {float g=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(root0),skin_arm_ik::position(main_local)));
              float prev=shadow_max_gap.load(std::memory_order_relaxed); while(g>prev && !shadow_max_gap.compare_exchange_weak(prev,g)) {}}
            const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
            return original(mesh,root,header,remap,cells);
          }
          shadow_misses.fetch_add(1,std::memory_order_relaxed);
          }
        }
      }
    }
    return original(mesh,root,bones,remap,cells);
  }
  if(vr_weapon::weapon_placed_by_game()) {
    // [HEADSET 2026-10-15] the game builds the weapon from the previous IK solve (game thread), the hand is drawn with
    // this frame's: the gun trailed the hand even standing still. Same fix as the gear: redraw it at this frame's IK
    // socket. offset = root bone relative to the socket the game used, measured while that socket is still (<2 mm per
    // write) so frame skew does not bias it; then bones * inv(root) * offset * IK socket (this frame).
    if(n>8 || !read(source,0,mats,static_cast<std::size_t>(n)*64)) return original(mesh,root,bones,remap,cells);
    float used_world[16],motion=1.f; std::int16_t c[3]{};
    if(!vr_weapon::written_socket(used_world,&motion) || !read(cells,0,c,sizeof(c))) return original(mesh,root,bones,remap,cells);
    float used[16]; std::memcpy(used,used_world,64); for(int k=0;k<3;++k) used[12+k]-=1024.f*c[k];
    const float root_gap=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[0]),skin_arm_ik::position(used)));
    {static const void* listed[16]{}; static std::atomic<unsigned> listed_n{0};  // which weapon meshes reach the redraw
      bool fresh=true; for(auto q:listed) fresh=fresh && q!=mesh;
      if(fresh && listed_n.load()<16) {listed[listed_n.fetch_add(1)%16]=mesh;
        char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
        char line[300]; std::snprintf(line,sizeof(line),"skin ik weapon redraw candidate: %s n=%u root_gap=%.3f m %s",path,n,root_gap,
          root_gap>0.8f?"(skipped: holstered)":"(redrawn)"); log(line);}}
    if(root_gap>0.8f) return original(mesh,root,bones,remap,cells);  // holstered (on the back), not in hand
    GripOffset* slot=nullptr;
    {std::lock_guard<std::mutex> lock(weapon_write_mutex);
      for(auto& g:grip_offsets) if(g.mesh==mesh) {slot=&g; break;}
      if(!slot) for(auto& g:grip_offsets) if(!g.mesh) {slot=&g; slot->mesh=mesh; slot->have=false; break;}
      if(!slot) {slot=&grip_offsets[reinterpret_cast<std::uintptr_t>(mesh)/64%8]; slot->mesh=mesh; slot->have=false;}
      float inverse_used[16];
      if(motion<0.003f && skin_arm_ik::inverse_affine(inverse_used,used)) {
        skin_arm_ik::multiply_affine(slot->offset,mats[0],inverse_used); slot->have=true; grip_calibrations.fetch_add(1);}
      if(!slot->have) return original(mesh,root,bones,remap,cells);}
    float ik_now[16],inverse_root0[16],anchor[16];
    const int hand=(left_handed?0:1);
    if(!skin_arm_ik::inverse_affine(inverse_root0,mats[0])) return original(mesh,root,bones,remap,cells);
    // [HEADSET 2026-10-15] re-anchoring this socket to the live body position (reroot) shifted the gun forward/back
    // against the drawn hand while walking (the game thread is ahead of the frame being drawn). Use exactly the hand
    // matrix the FP body is drawn with this frame (shared_fp: one solve per frame for every mesh).
    std::memcpy(ik_now,out[attach_bone[hand]],64);
    (void)out_root; (void)root_matrix;
    {std::lock_guard<std::mutex> lock(weapon_write_mutex); skin_arm_ik::multiply_affine(anchor,slot->offset,ik_now);}
    float change[16]; skin_arm_ik::multiply_affine(change,inverse_root0,anchor);
    {const float err[3]={anchor[12]-mats[0][12],anchor[13]-mats[0][13],anchor[14]-mats[0][14]};  // cell-local difference = world difference
      vr_weapon::report_render_error(err);}
    for(unsigned i=0;i<n;++i) {float r[16]; skin_arm_ik::multiply_affine(r,mats[i],change); std::memcpy(mats[i],r,64);}
    const void* check2=nullptr;
    if(!read(bones,0,&check2,sizeof(check2)) || check2!=source || game::pointers().body!=player.body) return original(mesh,root,bones,remap,cells);
    const void* replacement2=mats; std::memcpy(header,&replacement2,sizeof(replacement2));
    grip_redraws.fetch_add(1);
    return original(mesh,root,header,remap,cells);
  }
  if(n==0 || n>kMaxGearBones || !read(source,0,mats,static_cast<std::size_t>(n)*64)) return original(mesh,root,bones,remap,cells);
  if(n<=8) {  // our own write still in place (game did not rebuild the weapon pose): start from the remembered flat bones
    std::lock_guard<std::mutex> lock(weapon_write_mutex);
    for(auto& w:weapon_writes) if(w.source==source && w.n==n && !std::memcmp(w.written,mats,static_cast<std::size_t>(n)*64)) {
      std::memcpy(mats,w.raw,static_cast<std::size_t>(n)*64); weapon_pose_reuses.fetch_add(1); break;}
  }
  {static std::atomic<unsigned> name_logs{0};  // which weapon bone is the muzzle? (read-only, once per mesh, 8 lines)
    static const void* named[8]{};
    bool fresh=true; for(auto q:named) fresh=fresh && q!=mesh;
    if(fresh && name_logs.load()<8) {
      named[name_logs.fetch_add(1)%8]=mesh;
      char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
      std::string line=std::string("skin ik weapon bones (")+path+"):";
      for(unsigned i=0;i<n && i<8;++i) {Sample e{}; char item[96];
        std::snprintf(item,sizeof(item)," %u=%s(%.3f %.3f %.3f)",i,read_name(root,i,e)?e.bone_name:"?",mats[i][12],mats[i][13],mats[i][14]);
        line+=item;}
      log(line.c_str());
    }}
  float raw_copy[8][16]; if(n<=8) std::memcpy(raw_copy,mats,static_cast<std::size_t>(n)*64);
  float inverse_then[16],reroot[16];
  if(!skin_arm_ik::inverse_affine(inverse_then,out_root)) return original(mesh,root,bones,remap,cells);
  skin_arm_ik::multiply_affine(reroot,inverse_then,root_matrix);
  // [LIVE] 2026-10-15: both *_weaponAttachmentHand bones sit at the weapon socket (distances L/R always identical), so
  // "nearest hand" flipped every frame and the weapon jumped between the hands. One weapon hand from the setting: right
  // unless weapon_hand = 2 (left-hand mode). Held = within 15 cm of that hand's raw attachment bone.
  const int weapon_hand=(left_handed?0:1);
  int held=-1; float dist[2]={1e9f,1e9f};
  float raw_attach[2][16];
  for(int h=0;h<2;++h) {
    skin_arm_ik::multiply_affine(raw_attach[h],raw[attach_bone[h]],reroot);
    const skin_arm_ik::Vec a=skin_arm_ik::position(raw_attach[h]);
    for(unsigned i=0;i<n;++i) dist[h]=std::min(dist[h],skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[i]),a)));
  }
  // [LIVE] 2026-10-15: a second part of the held weapon (front/ring) measured 0.31 m from the socket and floated as
  // "holstered"; holstered weapons measure >= 2.3 m. 0.6 m separates them.
  if(dist[weapon_hand]<0.6f) held=weapon_hand;
  if(held!=map.held) {
    map.held=held;
    static std::atomic<unsigned> weapon_logs{0};  // [LIVE] the same weapon is drawn twice (held 0.000 m / other 0.31 m)
    {std::lock_guard<std::mutex> lock(accessory_mutex); for(auto& m:accessory_maps) if(m.mesh==mesh) m.held=held;}
    if(weapon_logs.fetch_add(1)<40) {
    char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
    char line[300];
    std::snprintf(line,sizeof(line),"skin ik weapon: %s %s (distance to raw attachment L %.3f R %.3f m)",path,
                  held<0?"holstered/not in hand":held?"held RIGHT -> follows IK hand":"held LEFT -> follows IK hand",dist[0],dist[1]);
    log(line);
    }
  }
  if(held<0) return original(mesh,root,bones,remap,cells);
  if(dist[weapon_hand]<0.05f) {std::int16_t c[3]{};
    held_weapon_draw_tick.store(GetTickCount64(),std::memory_order_relaxed);
    if(read(cells,0,c,sizeof(c))) {std::lock_guard<std::mutex> lock(weapon_write_mutex);
      std::memcpy(held_root_world,mats[0],64); for(int k=0;k<3;++k) held_root_world[12+k]+=1024.f*c[k]; held_root_tick=GetTickCount64();}}
  float inverse_raw[16],ik_attach[16],delta[16];
  if(!skin_arm_ik::inverse_affine(inverse_raw,raw_attach[held])) return original(mesh,root,bones,remap,cells);
  skin_arm_ik::multiply_affine(ik_attach,out[attach_bone[held]],reroot);
  skin_arm_ik::multiply_affine(delta,inverse_raw,ik_attach);
  for(unsigned i=0;i<n;++i) {float r[16]; skin_arm_ik::multiply_affine(r,mats[i],delta); std::memcpy(mats[i],r,64);}
  note_arrow(mesh,root,cells,mats,n);  // as drawn after the hand change
  const void* check=nullptr;
  if(!read(bones,0,&check,sizeof(check)) || check!=source || game::pointers().body!=player.body) return original(mesh,root,bones,remap,cells);
  if(weapon_pose_write && *lua_host::writes_flag() && n<=8 && game::mem_write(const_cast<void*>(source),mats,static_cast<std::size_t>(n)*64)) {
    std::lock_guard<std::mutex> lock(weapon_write_mutex);
    WeaponWrite* slot=nullptr;
    for(auto& w:weapon_writes) if(w.source==source) {slot=&w; break;}
    if(!slot) for(auto& w:weapon_writes) if(!w.source) {slot=&w; break;}
    if(!slot) slot=&weapon_writes[reinterpret_cast<std::uintptr_t>(source)/64%4];
    slot->source=source; slot->n=n;
    std::memcpy(slot->raw,raw_copy,static_cast<std::size_t>(n)*64); std::memcpy(slot->written,mats,static_cast<std::size_t>(n)*64);
    weapon_pose_writes.fetch_add(1);
  }
  const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
  weapon_apps.fetch_add(1);
  return original(mesh,root,header,remap,cells);
}
// Player utility items with their own root at the hand (research session: G_UTILITY_TORSO_03B, 4 bones, root ~2.2 m up,
// the right-wrist knob): the nearest raw FP hand-subtree bone within 0.2 m of any of its bones carries it rigidly.
std::atomic<unsigned> item_apps{0};
bool item_call(void* mesh,const void* root,const void* bones,const void* remap,const void* cells,unsigned char* header,
               const void* source,unsigned n,const game::Pointers& player,const float (&raw)[211][16],
               const float (&out)[211][16],const float* out_root,const float* fp_root) {
  static thread_local float mats[kMaxGearBones][16];
  if(n==0 || n>kMaxGearBones || !read(source,0,mats,static_cast<std::size_t>(n)*64)) return original(mesh,root,bones,remap,cells);
  float inverse_then[16],reroot[16];
  if(!skin_arm_ik::inverse_affine(inverse_then,out_root)) return original(mesh,root,bones,remap,cells);
  skin_arm_ik::multiply_affine(reroot,inverse_then,fp_root);
  // [LIVE 2026-10-31] the search below missed on 2764 of 9534 frames and dropped the item to the game's pose each time, and each hit could
  // pick another bone. Bound once, it stays rigid on that bone, like gear.
  const void* item_resource=nullptr; read(root,0x58,&item_resource,sizeof(item_resource));
  const std::uint64_t item_key=(static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(item_resource?item_resource:mesh))<<11)^n;
  if(rig_coherent) {
    ItemBind bind; {std::lock_guard<std::mutex> lock(item_bind_mutex); auto it=item_binds.find(item_key); if(it!=item_binds.end()) bind=it->second;}
    if(bind.local && bind.bone>=0 && bind.local->size()==static_cast<std::size_t>(n)*16u) {
      float ik[16]; skin_arm_ik::multiply_affine(ik,out[bind.bone],reroot);
      for(unsigned i=0;i<n;++i) {float r[16]; skin_arm_ik::multiply_affine(r,bind.local->data()+i*16,ik); std::memcpy(mats[i],r,64);}
      const void* check=nullptr;
      if(!read(bones,0,&check,sizeof(check)) || check!=source || game::pointers().body!=player.body) return original(mesh,root,bones,remap,cells);
      const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
      item_apps.fetch_add(1);
      return original(mesh,root,header,remap,cells);
    }
  }
  // [HEADSET/LIVE] 2026-10-15: no "skin ik item" line was ever logged, so the hand-subtree-only search (0.2 m) found
  // nothing: candidates are now every arm bone except the shoulders (the knob may sit on the forearm), and a miss logs
  // the nearest arm bone and distance.
  int best_j=-1; float best=0.2f; float best_raw[16];
  int near_j=-1; float near_d=1e9f;
  for(unsigned j=0;j<211;++j) {
    if(bone_side[j]<0 || fp_names[j].find("shoulder")!=std::string::npos) continue;
    float r[16]; skin_arm_ik::multiply_affine(r,raw[j],reroot);
    const skin_arm_ik::Vec a=skin_arm_ik::position(r);
    for(unsigned i=0;i<n;++i) {
      const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[i]),a));
      if(d<near_d) {near_d=d; near_j=static_cast<int>(j);}
      if(d<best) {best=d; best_j=static_cast<int>(j); std::memcpy(best_raw,r,sizeof(r));}
    }
  }
  if(best_j<0) {
    IkTraceRow tr; tr.phase="item_miss"; tr.object_id=source; tr.gap=near_d;
    if(near_j>=0) tr.hand=bone_side[near_j];
    std::memcpy(tr.object,mats[0]+12,12); ik_trace(tr);
    static std::atomic<unsigned> miss_logs{0};
    if(miss_logs.fetch_add(1)<6) {
      char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
      char line[300]; std::snprintf(line,sizeof(line),"skin ik item MISS: %s nearest arm bone %s at %.3f m (limit 0.2)",path,
                                    near_j>=0?fp_names[near_j].c_str():"none",near_d); log(line);
    }
    return original(mesh,root,bones,remap,cells);
  }
  float inverse_raw[16],ik[16],delta[16];
  if(!skin_arm_ik::inverse_affine(inverse_raw,best_raw)) return original(mesh,root,bones,remap,cells);
  if(rig_coherent) {
    auto local=std::make_shared<std::vector<float>>(static_cast<std::size_t>(n)*16u);
    for(unsigned i=0;i<n;++i) skin_arm_ik::multiply_affine(local->data()+i*16,mats[i],inverse_raw);
    std::lock_guard<std::mutex> lock(item_bind_mutex);
    if(item_binds.size()>=64) item_binds.clear();
    item_binds[item_key]=ItemBind{best_j,local};
  }
  skin_arm_ik::multiply_affine(ik,out[best_j],reroot);
  skin_arm_ik::multiply_affine(delta,inverse_raw,ik);
  float raw_item[3]; std::memcpy(raw_item,mats[0]+12,12);
  for(unsigned i=0;i<n;++i) {float r[16]; skin_arm_ik::multiply_affine(r,mats[i],delta); std::memcpy(mats[i],r,64);}
  static std::atomic<unsigned> item_logs{0};
  if(item_logs.fetch_add(1)<8) {
    char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
    char line[260]; std::snprintf(line,sizeof(line),"skin ik item: %s follows %s (%.3f m)",path,fp_names[best_j].c_str(),best); log(line);
  }
  const void* check=nullptr;
  if(!read(bones,0,&check,sizeof(check)) || check!=source || game::pointers().body!=player.body) return original(mesh,root,bones,remap,cells);
  {vr_arms::Sample smp; const bool have_sample=vr_arms::latest(smp);
    IkTraceRow tr; tr.phase="item"; tr.hand=bone_side[best_j]; tr.sample=have_sample?&smp:nullptr;
    tr.object_id=source; tr.gap=best;
    std::memcpy(tr.free_target,raw_item,12); std::memcpy(tr.final_target,ik+12,12);
    std::memcpy(tr.object,mats[0]+12,12); ik_trace(tr);}
  const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
  item_apps.fetch_add(1);
  return original(mesh,root,header,remap,cells);
}
// ---- Gear draw census (read-only, 2026-11-08) ----
// [AUTHOR 2026-11-08] the bracelets' shadows (both arms) are not tracked and random pieces float in the shadow. [LIVE] every player mesh
// reaches this hook from one caller (+4591FA), so the shadow copies cannot be told apart by caller. Per mesh object: draws and which
// way each went (ik = arm transfer, unmatched = no arm bones, game pose; far = not rooted on the player, game pose; other = fell back).
// Every 5 s one line per player gear/body path: objects, draws, ik/unmatched/far/other, distance range. 400 lines max.
enum GearWay {kGearIk=0,kGearUnmatched,kGearFar,kGearOther,kGearWays};
struct GearCensus {const void* mesh=nullptr; char path[96]{}; bool player=false; unsigned way[kGearWays]{}; float dmin=99.f,dmax=0.f;};
GearCensus gear_census[192]; std::mutex gear_census_mutex; ULONGLONG gear_census_next=0; unsigned gear_census_lines=0;
void gear_note(const void* mesh,const void* root,GearWay way,float distance2) {
  if(gear_census_lines>=400) return;
  const float d=std::sqrt(distance2);
  std::lock_guard<std::mutex> lock(gear_census_mutex);
  GearCensus* slot=&gear_census[reinterpret_cast<std::uintptr_t>(mesh)/16%192];
  if(slot->mesh!=mesh) {
    *slot=GearCensus{}; slot->mesh=mesh; char path[128]{};
    if(read_path(root,path)) {
      const char* tail=std::strstr(path,"CHARACTERART/"); std::snprintf(slot->path,sizeof(slot->path),"%s",tail?tail+13:path);
      slot->player=std::strstr(path,"/GEAR/PLAYER/") || std::strstr(path,"CHARACTERART/PLAYER/") || std::strstr(path,"RNF_BODY");
    }
  }
  if(!slot->player) return;
  ++slot->way[way]; slot->dmin=std::min(slot->dmin,d); slot->dmax=std::max(slot->dmax,d);
  const ULONGLONG now=GetTickCount64();
  if(!gear_census_next) gear_census_next=now+5000;
  if(now<gear_census_next) return;
  gear_census_next=now+5000;
  struct Row {std::string path; unsigned objects=0,way[kGearWays]{}; float dmin=99.f,dmax=0.f;};
  std::vector<Row> rows;
  for(auto& g:gear_census) {
    if(!g.mesh || !g.player) continue;
    unsigned total=0; for(unsigned w:g.way) total+=w; if(!total) continue;
    Row* r=nullptr; for(auto& q:rows) if(q.path==g.path) {r=&q; break;}
    if(!r) {rows.push_back(Row{}); r=&rows.back(); r->path=g.path;}
    ++r->objects; for(int w=0;w<kGearWays;++w) r->way[w]+=g.way[w];
    r->dmin=std::min(r->dmin,g.dmin); r->dmax=std::max(r->dmax,g.dmax);
    for(unsigned& w:g.way) w=0; g.dmin=99.f; g.dmax=0.f;
  }
  for(const auto& r:rows) {
    if(gear_census_lines>=400) break;
    char line[300]; std::snprintf(line,sizeof(line),"gear census 5s: %s objects=%u ik=%u unmatched=%u far=%u other=%u dist=%.3f-%.3f",
      r.path.c_str(),r.objects,r.way[kGearIk],r.way[kGearUnmatched],r.way[kGearFar],r.way[kGearOther],r.dmin,r.dmax);
    log(line); ++gear_census_lines;
  }
  if(gear_census_lines<400) {  // the full body's arm chain is moved toward its own shoulders; the gear on the arms is not
    const unsigned kept_gap=shadow_gear_kept_gap_mm.exchange(~0u);
    char line[320]; std::snprintf(line,sizeof(line),"gear census 5s: body arm shift max left=%umm right=%umm skeleton_anchor_strict=%d"
      " | shadow_gear_follow=%d moved=%u (max d_body %umm) kept=%u (min d_fp %dmm) stale=%u",
      body_arm_shift_mm[0].exchange(0),body_arm_shift_mm[1].exchange(0),skeleton_anchor_strict?1:0,shadow_gear_follow?1:0,
      shadow_gear_moved.exchange(0),shadow_gear_moved_dbody_mm.exchange(0),shadow_gear_kept.exchange(0),
      kept_gap==~0u?-1:static_cast<int>(kept_gap),shadow_gear_stale.exchange(0));
    log(line); ++gear_census_lines;
  }
}
bool accessory_call(void* mesh,const void* root,const void* bones,const void* remap,const void* cells,unsigned char* header,
                    const void* source,unsigned n,const game::Pointers& player,std::uintptr_t caller_rva) {
  float root_matrix[16]{},body_position[3]{}; std::int16_t cell[3]{};
  if(!read(root,0,root_matrix,sizeof(root_matrix)) || !read(cells,0,cell,sizeof(cell)) ||
     !rig_body_read(player.body,body_position)) return original(mesh,root,bones,remap,cells);
  // [SOURCE 2026-11-02] this test decides "is this mesh rooted on the player". It takes the live body: a mesh root is live, the shared body can
  // be a tick behind, and at running speed a tick is over the 15 cm below, which drew the game's flat pose for that frame.
  float live_body[3]; if(!read(player.body,0x94,live_body,sizeof(live_body))) std::memcpy(live_body,body_position,12);
  float distance=0.f;
  for(unsigned k=0;k<3;++k){const float d=root_matrix[12+k]+1024.f*cell[k]-live_body[k]; distance+=d*d;}
  if(!(distance<25.f)) return original(mesh,root,bones,remap,cells); // within 5 m of the player
  if(distance>0.15f*0.15f) {
    bool player_weapon=false, known=false;
    {std::lock_guard<std::mutex> lock(far_mutex); auto it=far_meshes.find(mesh); if(it!=far_meshes.end()) {known=true; player_weapon=it->second;}}
    if(!known) {
      char path[128]{}; const bool have=read_path(root,path);
      // Player weapons and player utility items (pouches/knobs attached at the hand, root ~2 m up) may continue.
      player_weapon=have && (std::strstr(path,"/WEAPON/PLAYER/")!=nullptr || std::strstr(path,"/GEAR/PLAYER/UTILITY/")!=nullptr);
      // [AUTHOR 2026-11-08] gear shadows look wrong. Read-only: player gear/body meshes that miss the 'rooted on the player' test are
      // drawn in the game's pose (no IK); log each once with its distance, to see whether the shadow body's gear is one of them.
      if(have && !player_weapon && (std::strstr(path,"/GEAR/PLAYER/") || std::strstr(path,"/CHARACTERART/PLAYER/") || std::strstr(path,"RNF_BODY"))) {
        static std::atomic<unsigned> far_gear_lines{0};
        if(far_gear_lines.fetch_add(1)<60) {char line[256];
          std::snprintf(line,sizeof(line),"skin far player mesh (game pose, no IK): path=%s n=%u dist=%.3f mesh=%p",path,n,std::sqrt(distance),mesh);
          log(line);}
      }
      std::lock_guard<std::mutex> lock(far_mutex);
      if(far_meshes.size()>=4096) far_meshes.clear();
      far_meshes[mesh]=player_weapon;
    }
    if(!player_weapon) {gear_note(mesh,root,kGearFar,distance); return original(mesh,root,bones,remap,cells);}
  }
  static thread_local float raw[211][16],out[211][16],mats[kMaxGearBones][16];
  float out_root[16]{},fp_root[16];
  const ULONGLONG now=GetTickCount64();
  if(!fp_root_now(fp_root,body_position,cell)) {gear_note(mesh,root,kGearOther,distance); return original(mesh,root,bones,remap,cells);}
  {bool solved=false;
    if(!shared_fp(fp_root,cell,body_position,fp_source.load(),now,raw,out,out_root,&solved)) {
      if(clean_rig) flat_frames.fetch_add(1,std::memory_order_relaxed);
      rig_fallback[3].fetch_add(1,std::memory_order_relaxed); gear_note(mesh,root,kGearOther,distance);
      return original(mesh,root,bones,remap,cells);}}
  if(side_state.load()!=2) {gear_note(mesh,root,kGearOther,distance); return original(mesh,root,bones,remap,cells);}
  // Name map (cached per mesh): accessory bone i -> FP bone with the same name. Robust to animation and build order.
  AccessoryMap map; bool cached=false;
  {std::lock_guard<std::mutex> lock(accessory_mutex);
    for(auto& m:accessory_maps) if(m.mesh==mesh && m.n==n) {map=m; cached=true; break;}}
  if(!read(source,0,mats,static_cast<std::size_t>(n)*64)) return original(mesh,root,bones,remap,cells);
  {float sh[3]{}; bool sv=false;  // "Lock body to view": gear and shadow move with the FP rig
    {std::lock_guard<std::mutex> lock(fp_last_mutex); sv=fp_shift_valid; std::memcpy(sh,fp_shift_last,12);}
    if(sv && !(distance>0.15f*0.15f)) for(unsigned i=0;i<n;++i) for(int k=0;k<3;++k) mats[i][12+k]+=sh[k];}
  if(!cached || (!map.matched && now-map.checked>30000)) {
    map.mesh=mesh; map.n=n; map.checked=now; map.matched=0; map.anchored=0; map.body=0; map.fp_anchored=0;
    static thread_local bool body_like[kMaxGearBones];
    static thread_local bool accessory_like[kMaxGearBones]; // C_accessory* cords/pouches: hang further from the arm
    static thread_local bool helper_like[kMaxGearBones];    // *_twist/_expand/_bend/_driver: deform helpers, poor anchors
    static thread_local char bone_names[kMaxGearBones][24];
    for(unsigned i=0;i<n;++i) {
      map.to_fp[i]=-1; // -1 unnamed, -2 named body/shoulder bone (left on the game's pose), >=0 FP arm bone
      body_like[i]=false; accessory_like[i]=false; helper_like[i]=false; bone_names[i][0]=0;
      Sample entry{};
      if(!read_name(root,i,entry) || entry.name_count!=n) continue;
      std::snprintf(bone_names[i],sizeof(bone_names[i]),"%s",entry.bone_name);
      helper_like[i]=std::strstr(entry.bone_name,"twist")||std::strstr(entry.bone_name,"expand")||
                     std::strstr(entry.bone_name,"bend")||std::strstr(entry.bone_name,"driver");
      body_like[i]=body_part_name(entry.bone_name);
      accessory_like[i]=std::strstr(entry.bone_name,"accessory")!=nullptr;
      for(unsigned j=0;j<211;++j) if(fp_names[j]==entry.bone_name) {
        // Only arm bones take the FP result: torso/shoulder gear (straps) stays on the game's body, which the FP rig
        // offsets (rest pose, shoulder sliders, headset shift) must not move.
        if(bone_side[j]>=0 && fp_names[j].find("shoulder")==std::string::npos) {map.to_fp[i]=static_cast<std::int16_t>(j); ++map.matched;}
        else {map.to_fp[i]=-2; ++map.body;}
        break;
      }
    }
    {char body_path[128]{}; const bool have_path=read_path(root,body_path);
      map.body_mesh=have_path && std::strstr(body_path,"/BODY/")!=nullptr;
      map.weapon=have_path && std::strstr(body_path,"/WEAPON/PLAYER/")!=nullptr;
      map.item=have_path && std::strstr(body_path,"/GEAR/PLAYER/UTILITY/")!=nullptr;
      map.upper_index[0]=map.upper_index[1]=-1;
      for(unsigned i=0;i<n;++i) {if(map.to_fp[i]==14) map.upper_index[0]=static_cast<int>(i); if(map.to_fp[i]==113) map.upper_index[1]=static_cast<int>(i);}}
    // Gear-only bones (beads, cords) ride rigidly with an arm bone, measured in this mesh's own pose.
    // [AUTHOR 2026-10-24] "accessories not staying attached together like little charms on the left bracelet": [LIVE]
    // the songcord charm parts (C_accessory_01/02/03, drivers, L/C/R_accessory_string_*) were anchored to the nearest
    // FINGER bone, different parts to different fingers (L_fingerMiddleBase_01 / IndexBase_01 / Thumb_01), and the pick
    // flipped between remaps; the IK curls fingers independently, so one charm was pulled apart. Anchors are now only
    // the arm chain (upperArm / lowerArm / hand) by segment: on the forearm -> lowerArm, on the upper arm -> upperArm,
    // at/after the wrist -> hand. Every part of one charm group (same name stem) then takes the group's majority anchor.
    {
      const bool skeleton=skeleton_anchor_strict && (map.body_mesh || n>=300);
      std::string kept_on_body;  // skeleton bones that would have anchored before skeleton_anchor_strict
      int chain[2][3]={{-1,-1,-1},{-1,-1,-1}};  // this mesh's own upperArm/lowerArm/hand per side
      for(unsigned i=0;i<n;++i) for(int h=0;h<2;++h) for(int r=0;r<3;++r)
        if(map.to_fp[i]==(h?113:14)+r) chain[h][r]=static_cast<int>(i);
      for(unsigned i=0;i<n;++i) {
        map.anchor[i]=-1; map.group[i]=-1;
        if(map.to_fp[i]>=0 || body_like[i]) continue; // the full body's own legs/tail/hair/spine never follow the arms
        const float radius=accessory_like[i]?0.6f:0.25f; // metres: beads/charms sit on the arm; accessory cords hang further
        const skin_arm_ik::Vec p=skin_arm_ik::position(mats[i]);
        float best=radius; int pick=-1;
        for(int h=0;h<2;++h) {
          if(chain[h][0]<0 || chain[h][1]<0 || chain[h][2]<0) continue;
          const skin_arm_ik::Vec S=skin_arm_ik::position(mats[chain[h][0]]),E=skin_arm_ik::position(mats[chain[h][1]]),
                                 W=skin_arm_ik::position(mats[chain[h][2]]);
          float along=0.f;
          const rider_ik::Role r=rider_ik::nearest_role(p,S,E,W,along);
          float d=0.f;
          if(r==rider_ik::kLower) skin_arm_ik::segment_t(p,E,W,d);
          else if(r==rider_ik::kUpper) skin_arm_ik::segment_t(p,S,E,d);
          else if(r==rider_ik::kHand) d=skin_arm_ik::length(skin_arm_ik::sub(p,W));
          else {  // hanging further (cords): nearest chain joint within the radius
            for(int k=0;k<3;++k) {const float dj=skin_arm_ik::length(skin_arm_ik::sub(p,skin_arm_ik::position(mats[chain[h][k]])));
              if(dj<best) {best=dj; pick=chain[h][k];}}
            continue;
          }
          if(d<best) {best=d; pick=chain[h][r==rider_ik::kUpper?0:r==rider_ik::kLower?1:2];}
        }
        if(pick<0) {  // no arm chain of its own in reach: nearest named arm bone that is not a finger or a helper
          best=radius;
          for(unsigned k=0;k<n;++k) {
            if(map.to_fp[k]<0 || helper_like[k] || std::strstr(bone_names[k],"finger") || std::strstr(bone_names[k],"thumb")) continue;
            const float d=skin_arm_ik::length(skin_arm_ik::sub(p,skin_arm_ik::position(mats[k])));
            if(d<best) {best=d; pick=static_cast<int>(k);}
          }
        }
        if(skeleton && pick>=0 && !accessory_like[i]) {
          if(kept_on_body.size()<600) {char item[64]; std::snprintf(item,sizeof(item)," %s->%s(%.3f)",bone_names[i][0]?bone_names[i]:"?",bone_names[pick],best);
            kept_on_body+=item;}
          pick=-1;
        }
        map.anchor[i]=static_cast<std::int16_t>(pick);
      }
      if(!kept_on_body.empty()) {
        static std::mutex kept_mutex; static std::vector<std::string> kept_paths;
        char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
        bool fresh=false;
        {std::lock_guard<std::mutex> lock(kept_mutex);
          if(kept_paths.size()<32 && std::find(kept_paths.begin(),kept_paths.end(),std::string(path))==kept_paths.end()) {kept_paths.emplace_back(path); fresh=true;}}
        if(fresh) log((std::string("skin ik skeleton bones kept on the body pose, no longer pinned to an arm (")+path+"):"+kept_on_body).c_str());
      }
      // Charm groups: name stem up to the first digit, without "_driver" / "_end" / "_e" (C_accessory_01, _03_end and
      // C_accessory_driver_02 are one charm). Majority anchor for all parts; the representative is the part farthest
      // from that anchor (it drives the group's swing).
      std::vector<std::string> stems(n);
      for(unsigned i=0;i<n;++i) if(map.to_fp[i]==-1 && !body_like[i] && bone_names[i][0]) {
        std::string st(bone_names[i]);
        const auto digit=st.find_first_of("0123456789");
        if(digit!=std::string::npos) st.resize(digit);
        for(const char* cut:{"_driver_","_driver","_end","_e"}) {const auto at=st.rfind(cut); if(at!=std::string::npos && at+std::strlen(cut)==st.size()) st.resize(at);}
        while(!st.empty() && st.back()=='_') st.pop_back();
        stems[i]=st;
      }
      std::vector<char> done(n,0);
      for(unsigned i=0;i<n;++i) {
        if(done[i] || stems[i].empty()) continue;
        std::vector<unsigned> members;
        for(unsigned j=i;j<n;++j) if(!done[j] && stems[j]==stems[i]) {members.push_back(j); done[j]=1;}
        if(members.size()<2) continue;
        int votes_best=0,anchor=-1;
        for(unsigned m:members) {const int a=map.anchor[m]; if(a<0) continue; int v=0; for(unsigned q:members) v+=map.anchor[q]==a;
          if(v>votes_best) {votes_best=v; anchor=a;}}
        if(anchor<0) continue;
        int rep_bone=-1; float farthest=-1.f;
        for(unsigned m:members) {
          map.anchor[m]=static_cast<std::int16_t>(anchor);
          const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[m]),skin_arm_ik::position(mats[anchor])));
          if(d>farthest) {farthest=d; rep_bone=static_cast<int>(m);}
        }
        for(unsigned m:members) map.group[m]=static_cast<std::int16_t>(rep_bone);
      }
      for(unsigned i=0;i<n;++i) if(map.anchor[i]>=0) ++map.anchored;
    }
    // [LIVE] 2026-10-15: an FP-proximity fallback (nearest FP arm bone within 12 cm) pinned waist/legs/hair gear bones
    // to the hands (fp_anchored 5-36 per mesh) and was removed. Unanchored bones stay on the game's pose; they are
    // logged by name with their nearest arm/body distances to identify a dangling charm.
    for(unsigned i=0;i<n;++i) map.anchor_fp[i]=-1;
    {std::lock_guard<std::mutex> lock(accessory_mutex);
      AccessoryMap* slot=nullptr;
      for(auto& m:accessory_maps) if(m.mesh==mesh) {slot=&m; break;}
      if(!slot) {for(auto& m:accessory_maps) if(!m.mesh) {slot=&m; accessory_meshes.fetch_add(1); break;}}
      if(!slot) slot=&accessory_maps[reinterpret_cast<std::uintptr_t>(mesh)/64%128];
      *slot=map;}
    if(!cached && map.anchored) {
      static std::mutex anchor_log_mutex; static std::vector<std::string> anchor_log_paths;  // once per path (songcord repeats used up the old 24)
      char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
      bool fresh_path=false;
      {std::lock_guard<std::mutex> lock(anchor_log_mutex);
        if(anchor_log_paths.size()<48 && std::find(anchor_log_paths.begin(),anchor_log_paths.end(),std::string(path))==anchor_log_paths.end()) {
          anchor_log_paths.emplace_back(path); fresh_path=true;}}
      if(fresh_path) {
        std::string line=std::string("skin ik anchors (")+path+"):";
        unsigned listed=0;
        for(unsigned i=0;i<n && listed<24;++i) {const int k=map.anchor[i]; if(k<0) continue;
          const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[i]),skin_arm_ik::position(mats[k])));
          char item[96]; std::snprintf(item,sizeof(item)," %s->%s(%.3f)",bone_names[i][0]?bone_names[i]:"?",bone_names[k],d);
          line+=item; ++listed;}
        log(line.c_str());
      }
    }
    if(!cached) {
      char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
      bool fresh=false;
      {std::lock_guard<std::mutex> lock(seen_paths_mutex);
        if(seen_paths.size()<64 && std::find(seen_paths.begin(),seen_paths.end(),std::string(path))==seen_paths.end()) {
          seen_paths.emplace_back(path); fresh=true;}}
      if(fresh) {
        char rva[24]; std::snprintf(rva,sizeof(rva),"%llX",static_cast<unsigned long long>(caller_rva));
        std::string line=std::string("skin ik mesh: path=")+path+" n="+std::to_string(n)+" caller=+"+rva+
          " arm_named="+std::to_string(map.matched)+" body_named="+std::to_string(map.body)+" anchored="+std::to_string(map.anchored)+
          " fp_anchored="+std::to_string(map.fp_anchored)+" dist="+std::to_string(std::sqrt(distance))+" arm:";
        for(unsigned i=0,shown=0;i<n && shown<12;++i) if(map.to_fp[i]>=0) {line+=" "+fp_names[map.to_fp[i]]; ++shown;}
        log(line.c_str());
        if(map.matched) {
          std::string loose="skin ik mesh loose bones ("+std::string(path)+"):";
          unsigned listed=0;
          for(unsigned i=0;i<n && listed<20;++i) if(map.to_fp[i]==-1 && map.anchor[i]<0) {
            float arm=1e9f,body=1e9f;
            for(unsigned k=0;k<n;++k) if(map.to_fp[k]!=-1) {
              const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[i]),skin_arm_ik::position(mats[k])));
              if(map.to_fp[k]>=0) arm=std::min(arm,d); else body=std::min(body,d);
            }
            Sample entry{}; const bool named=read_name(root,i,entry);
            char item[128]; std::snprintf(item,sizeof(item)," %u=%s(arm %.2f body %.2f)",i,named?entry.bone_name:"?",arm,body);
            loose+=item; ++listed;
          }
          if(listed) log(loose.c_str());
        }
      }
    }
  }
  // [LIVE/AUTHOR] 2026-10-15: NPCs within 5 m share the skeleton names (L_upperArm...), so their arms took the player's
  // IK pose and stretched to the player's hands. Only meshes rooted at the player's body (all player gear and the full
  // body measured dist 0.00) take the transfer; the player's held weapon (root at the hand) is allowed by its path.
  if(!map.weapon && !map.item && distance>0.15f*0.15f) {
    if(map.matched) rig_fallback[1].fetch_add(1,std::memory_order_relaxed);
    return original(mesh,root,bones,remap,cells);
  }
  if(map.item && distance>0.15f*0.15f) return item_call(mesh,root,bones,remap,cells,header,source,n,player,raw,out,out_root,fp_root);
  if(map.weapon) return weapon_call(mesh,root,bones,remap,cells,header,source,n,player,map,raw,out,out_root,fp_root);
  if(!map.matched) {gear_note(mesh,root,kGearUnmatched,distance); return original(mesh,root,bones,remap,cells);}
  gear_note(mesh,root,kGearIk,distance);
  static thread_local float orig[kMaxGearBones][16];
  std::memcpy(orig,mats,static_cast<std::size_t>(n)*64);
  // [LIVE] 2026-10-15: the gear (RNF_SONGCORD_*, 77-bone gear skeleton, same root as the FP body) carries exact copies
  // of the FP bones while idle but its own animation while moving, so a delta transfer kept that animation. Same-named
  // bones take the FP body's final (IK) matrix outright.
  // The FP result may be a frame old (gear built before the FP body): it is in world/cell space, so while moving it
  // trails by a frame of travel and the gear slid along the arms. Re-attach it to this frame's root: out * inv(root_then)
  // * root_now (the gear shares the FP root, dist 0).
  float reroot[16],inverse_then[16];
  if(!skin_arm_ik::inverse_affine(inverse_then,out_root)) return original(mesh,root,bones,remap,cells);
  skin_arm_ik::multiply_affine(reroot,inverse_then,fp_root);
  for(unsigned i=0;i<n;++i) {
    const int j=map.to_fp[i]; if(j<0) continue;
    skin_arm_ik::multiply_affine(mats[i],out[j],reroot);
  }
  {const int wh=(left_handed?0:1);  // A: the weapon hand as a gear mesh draws it (value: re-root centi-degrees)
    if(attach_bone[wh]>=0) for(unsigned i=0;i<n;++i) if(map.to_fp[i]==attach_bone[wh]) {
      float v[3]; for(int k=0;k<3;++k) v[k]=mats[i][12+k]+1024.f*cell[k]; vr_weapon::timing_mark('A',v,reroot_centideg(reroot));
      // shadow_weapon_follow: the full (shadow) body's weapon hand as the game posed it, before the IK transfer, world
      if(map.body_mesh) {std::lock_guard<std::mutex> lock(shadow_mutex);
        for(int k=0;k<3;++k) tp_hand_raw[k]=orig[i][12+k]+1024.f*cell[k]; tp_hand_us=static_cast<std::uint64_t>(clean_qpc_us());}
      break;}}
  // Full body (2026-10-15): its arms hang from its own shoulders, which the FP rig offsets (rest pose, shoulder
  // height/separation, headset shift) moved away from, so its shoulder skin stretched. Shift its arm chain back towards
  // its own upperArm, fully at the upper arm, half at the forearm, none at the hand (the hand stays on the IK hand).
  // [HEADSET 2026-11-08 n] bracelet shadows float off the shadow's arms. [LIVE] this shift is 0.2-0.6 m (up to 1.25 m) per side, and gear
  // never took it, so gear on the shadow body's arms sat on the FP arms instead. shadow_gear_follow: a gear draw whose game-pose upperArm
  // is nearer the full body's own upperArm (this frame) than the FP raw upperArm is INFERENCE the shadow body's copy of that gear: it takes
  // the same shift. Gear on the FP (visible) arms is unchanged. shadow_gear_follow 0 in [arms] restores the previous behaviour.
  for(unsigned side=0;side<2;++side) {
    const int ui=map.upper_index[side]; if(ui<0) continue;
    float g[3]; for(int k=0;k<3;++k) g[k]=orig[ui][12+k]+1024.f*cell[k];
    const std::uint64_t us=static_cast<std::uint64_t>(clean_qpc_us());
    if(map.body_mesh) {
      std::lock_guard<std::mutex> lock(shadow_mutex);
      std::memcpy(body_upper_world[side],g,12); body_upper_us[side]=us;
    } else {
      if(!shadow_gear_follow) continue;
      // [HEADSET 2026-11-08 p] visible arm accessories flash while walking. [LIVE] the torso G_RES_TORSO_BASE_02 (416 bones, arm skin and
      // straps) was judged a shadow copy and took the shift, which swings with the walk animation; before, it followed the IK arms like
      // the visible arms. INFERENCE: it is seen in view. Skeleton-sized gear (>= 300 bones) keeps the IK arms (previous behaviour).
      if(n>=300) continue;
      float body_up[3]; std::uint64_t at=0;
      {std::lock_guard<std::mutex> lock(shadow_mutex); std::memcpy(body_up,body_upper_world[side],12); at=body_upper_us[side];}
      float d_body=-1.f,d_fp=-1.f;
      if(at && us-at<=40000) {
        float fp_up[16]; skin_arm_ik::multiply_affine(fp_up,raw[side?113:14],reroot);
        d_body=0.f; d_fp=0.f;
        for(int k=0;k<3;++k) {const float a=g[k]-body_up[k],b=g[k]-(fp_up[12+k]+1024.f*cell[k]); d_body+=a*a; d_fp+=b*b;}
        d_body=std::sqrt(d_body); d_fp=std::sqrt(d_fp);
      } else shadow_gear_stale.fetch_add(1,std::memory_order_relaxed);
      // [HEADSET 2026-11-08 o] with a per-draw choice the visible arm gear stuttered at random. INFERENCE: when the body's and the FP
      // arm's game poses come close, a visible copy was sometimes taken for the shadow's and jumped 0.2+ m for a frame. Each gear
      // object (an instance) now keeps one verdict: it changes only on a clear reading (the other arm at least 5 cm and twice as far);
      // an unclear or stale reading keeps the last verdict, and an object never judged stays on the FP arms.
      const bool to_shadow=shadow_gear_verdict(mesh,root,n,side,d_body,d_fp,g);
      if(!to_shadow) {shadow_gear_kept.fetch_add(1,std::memory_order_relaxed);
        if(d_fp>=0.f) atomic_min_mm(shadow_gear_kept_gap_mm,d_fp); continue;}
      shadow_gear_moved.fetch_add(1,std::memory_order_relaxed); if(d_body>=0.f) atomic_max_mm(shadow_gear_moved_dbody_mm,d_body);
    }
    const skin_arm_ik::Vec delta=skin_arm_ik::sub(skin_arm_ik::position(orig[ui]),skin_arm_ik::position(mats[ui]));
    if(map.body_mesh) {const float len=skin_arm_ik::length(delta); if(std::isfinite(len)) {const unsigned mm=static_cast<unsigned>(std::min(len,100.f)*1000.f);
      unsigned seen=body_arm_shift_mm[side].load(std::memory_order_relaxed);
      while(mm>seen && !body_arm_shift_mm[side].compare_exchange_weak(seen,mm)) {}}}
    for(unsigned i=0;i<n;++i) {
      const int j=map.to_fp[i]; if(j<0 || bone_side[j]!=static_cast<signed char>(side)) continue;
      const unsigned up=side?113:14;
      const float weight=body_arm_shift*((static_cast<unsigned>(j)==up || bone_role[j]==0)?1.f:(static_cast<unsigned>(j)==up+1 || bone_role[j]==1)?0.5f:0.f);
      if(weight==0.f) continue;
      mats[i][12]+=delta.x*weight; mats[i][13]+=delta.y*weight; mats[i][14]+=delta.z*weight;
    }
  }
  // [AUTHOR 2026-10-31] gear must sit on the arm as one piece. [SOURCE] each anchored bone kept this frame's native offset from its
  // anchor, which carries the game's own sway and dangle of the unused flat arm. The mounted path already froze this relation
  // (helper_local). On foot now too: bone = fixed offset * finished bone, after the solve, re-root and body chain shift.
  std::shared_ptr<const std::vector<float>> rest_gear_hold;
  const std::vector<float>* rest_gear=nullptr;
  if(accessory_rigid && map.anchored) {
    const void* resource=nullptr; read(root,0x58,&resource,sizeof(resource));
    const std::uint64_t key=(static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(resource?resource:mesh))<<11)^n;
    const bool still=rig_player_still.load(std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(accessory_rest_mutex);
    AccessoryRest& rest=accessory_rest[key];
    if(!rest.local || (still && !rest.taken_still)) {
      auto next=std::make_shared<std::vector<float>>(static_cast<std::size_t>(n)*16u,0.f);
      bool fine=true;
      for(unsigned i=0;i<n && fine;++i) {
        const int k=map.anchor[i]; if(k<0) continue;
        float inverse[16];
        if(!skin_arm_ik::inverse_affine(inverse,orig[k])) {fine=false; break;}
        skin_arm_ik::multiply_affine(next->data()+i*16,orig[i],inverse);
        for(int c=0;c<16;++c) fine=fine&&std::isfinite((*next)[i*16+c]);
      }
      if(fine) {rest.local=next; rest.taken_still=still;}
    }
    rest_gear_hold=rest.local;
    if(rest_gear_hold && rest_gear_hold->size()==static_cast<std::size_t>(n)*16u) rest_gear=rest_gear_hold.get();
  }
  for(unsigned i=0;i<n;++i) {
    const int k=map.anchor[i]; if(k<0) continue;
    float inverse[16],delta[16],result[16];
    if(rest_gear) {
      skin_arm_ik::multiply_affine(result,rest_gear->data()+i*16,mats[k]);
    } else {
      if(!skin_arm_ik::inverse_affine(inverse,orig[k])) continue;
      skin_arm_ik::multiply_affine(delta,inverse,mats[k]);
      skin_arm_ik::multiply_affine(result,orig[i],delta);
    }
    bool ok=true; for(float v:result) ok=ok&&std::isfinite(v);
    if(ok) std::memcpy(mats[i],result,64);
  }
  // Accessory physics: a damped spring on each anchored bone's tip, swung about its anchor bone (skin_arm_ik::Dangle).
  // Each bone swings independently about its anchor (parent-chain order is OPEN). Tips more than 15 cm from the anchor
  // (cords) may lag 8 cm, closer ones (charms, beads) 4 cm.
  if(accessory_physics && map.anchored) {
    LARGE_INTEGER now_qpc,freq; QueryPerformanceCounter(&now_qpc); QueryPerformanceFrequency(&freq);
    std::lock_guard<std::mutex> lock(dangle_mutex);
    const float dt=dangle_last.QuadPart?static_cast<float>(now_qpc.QuadPart-dangle_last.QuadPart)/static_cast<float>(freq.QuadPart):0.f;
    // Many meshes per frame share one clock: it advances once per frame (>4 ms); later calls reuse the stepped state.
    const bool advance=dt>0.004f || !dangle_last.QuadPart;
    if(advance) dangle_last=now_qpc;
    if(dangles.size()>8192) dangles.clear();
    // [AUTHOR 2026-10-24] one swing per charm: the group's representative is simulated and every part of the group is
    // turned by that same rotation about the shared anchor, so the charm swings as one piece instead of spreading.
    static thread_local skin_arm_ik::Vec swing_from[kMaxGearBones],swing_to[kMaxGearBones];
    static thread_local bool swing_ok[kMaxGearBones];
    for(unsigned i=0;i<n;++i) swing_ok[i]=false;
    for(unsigned i=0;i<n;++i) {
      const int k=map.anchor[i]; if(k<0) continue;
      const int g=map.group[i];
      if(g>=0 && static_cast<unsigned>(g)!=i) continue;  // grouped parts follow their representative below
      const skin_arm_ik::Vec pivot=skin_arm_ik::position(mats[k]),target=skin_arm_ik::position(mats[i]);
      const std::uint64_t key=(static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(mesh))<<11)^i;
      skin_arm_ik::Dangle& d=dangles[key];
      skin_arm_ik::Vec tip=d.x;
      if(advance || !d.primed) {
        const float reach=skin_arm_ik::length(skin_arm_ik::sub(target,pivot));
        tip=d.step(pivot,target,advance?dt:0.f,2.f,0.35f,reach>0.15f?0.08f:0.04f);
      }
      swing_from[i]=skin_arm_ik::sub(target,pivot); swing_to[i]=skin_arm_ik::sub(tip,pivot); swing_ok[i]=true;
    }
    for(unsigned i=0;i<n;++i) {
      const int k=map.anchor[i]; if(k<0) continue;
      const int g=map.group[i]>=0?map.group[i]:static_cast<int>(i);
      if(!swing_ok[g]) continue;
      skin_arm_ik::swing_about(mats[i],skin_arm_ik::position(mats[k]),swing_from[g],swing_to[g]);
    }
  }
  const void* check=nullptr;
  const bool same_source=read(bones,0,&check,sizeof(check)) && check==source;
  if(!same_source) rig_fallback[2].fetch_add(1,std::memory_order_relaxed);
  // The pose is our own copy by now: a game buffer swap after it does not make it wrong, while the flat pose would be.
  if((!same_source && !rig_coherent) || game::pointers().body!=player.body) return original(mesh,root,bones,remap,cells);
  const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
  accessory_apps.fetch_add(1);
  gesture_palette(root,mats,n);
  return original(mesh,root,header,remap,cells);
}
// Read-only census while IK runs (2026-10-15, shadow-body search): every mesh new to a thread is checked once; meshes
// within 10 m of the player or with a PLAYER/BODY path are logged once per (path, bone count, caller) with how this
// module handles them. Finds a body skinned here with more than 256 bones (skipped silently before) or via another caller.
std::vector<std::string> census_seen;
std::mutex census_mutex;
std::atomic<unsigned> census_lines{0};
void census(void* mesh,const void* root,const void* cells,unsigned n,const game::Pointers& player,std::uintptr_t caller_rva) {
  thread_local std::unordered_set<const void*> seen_here;
  if(census_lines.load()>=128 || GetTickCount64()-pulse_started.load()>60000) return; // first minute of each activation
  if(!seen_here.insert(mesh).second || seen_here.size()>8192) return;
  float root_matrix[16]{},body_position[3]{}; std::int16_t cell[3]{};
  if(!read(root,0,root_matrix,sizeof(root_matrix)) || !read(cells,0,cell,sizeof(cell)) ||
     !read(player.body,0x94,body_position,sizeof(body_position))) return;
  float distance=0.f;
  for(unsigned k=0;k<3;++k){const float d=root_matrix[12+k]+1024.f*cell[k]-body_position[k]; distance+=d*d;}
  distance=std::sqrt(distance);
  char path[128]{}; if(!read_path(root,path)) std::strcpy(path,"<unreadable>");
  const bool body_path=std::strstr(path,"PLAYER/BODY")!=nullptr;
  if(!(distance<10.f) && !body_path) return;
  char key[200]; std::snprintf(key,sizeof(key),"%s|%u|%llX",path,n,static_cast<unsigned long long>(caller_rva));
  {std::lock_guard<std::mutex> lock(census_mutex);
    if(std::find(census_seen.begin(),census_seen.end(),std::string(key))!=census_seen.end()) return;
    census_seen.emplace_back(key);}
  const char* handled=n==211?(_stricmp(path,"BLUE/BAKED/CHARACTERART/PLAYER/BODY/MALE/P_BODY_01_M/P_BODY_01_FP_M.MMB")?"other211(untouched)":"fp_body(IK)")
                      :n>kMaxGearBones?"over1024(UNTOUCHED)":n==0?"empty":distance<5.f?"gear(name map)":"far(untouched)";
  char line[320];
  std::snprintf(line,sizeof(line),"skin ik census: path=%s n=%u caller=+%llX dist=%.2f cell=%d,%d,%d handled=%s",path,n,
                static_cast<unsigned long long>(caller_rva),distance,cell[0],cell[1],cell[2],handled);
  census_lines.fetch_add(1); log(line);
}
// Wrist HUD anchor (2026-10-30): the drawn wrist/elbow relative to the same input sample's raw grip, in grip axes. Both are
// measured from the body-anchored view the rig uses, so walking and turning cancel out. Read-only: the palette is not changed.
std::mutex hud_anchor_mutex;
float hud_anchor_points[2][3][3]{};  // [hand][0 wrist, 1 elbow, 2 weapon attach], x right, y up, z forward
bool hud_anchor_primed[2]{};
std::atomic<int> hud_bone_request[skin_palette_probe::HudBoneSlots];
// Native arm HUD: the bones exactly as this palette draws them, relative to the game camera of the same sample, unfiltered.
std::mutex hud_camera_mutex;
skin_palette_probe::HudCameraPose hud_camera_latest;
ULONGLONG hud_camera_at=0;
long long hud_camera_qpc=0;
float hud_range_low[3]{},hud_range_high[3]{};
bool hud_range_primed=false;
std::atomic<int> hud_frame_method{1};
float hud_gap_mm=0.f,hud_gap_degrees=0.f;  // hud_camera_mutex
void store_hud_camera(const skin_palette_probe::HudCameraPose& next) {
  LARGE_INTEGER now; QueryPerformanceCounter(&now);
  std::lock_guard<std::mutex> lock(hud_camera_mutex);
  hud_camera_latest=next; hud_camera_at=GetTickCount64(); hud_camera_qpc=now.QuadPart;
  for(int k=0;k<3;++k) {
    if(!hud_range_primed||next.wrist[0][k]<hud_range_low[k]) hud_range_low[k]=next.wrist[0][k];
    if(!hud_range_primed||next.wrist[0][k]>hud_range_high[k]) hud_range_high[k]=next.wrist[0][k];
  }
  hud_range_primed=true;
}
void hud_to_camera(const float* cam,const float* world,float* out) {
  float w[3]; for(int k=0;k<3;++k) w[k]=world[k]-cam[12+k];
  for(int r=0;r<3;++r) out[r]=w[0]*cam[r*4]+w[1]*cam[r*4+1]+w[2]*cam[r*4+2];
}
void hud_axes_to_camera(const float* cam,const float* matrix,float* out) {
  for(int a=0;a<3;++a) for(int r=0;r<3;++r) {const float* v=matrix+a*4; out[a*3+r]=v[0]*cam[r*4]+v[1]*cam[r*4+1]+v[2]*cam[r*4+2];}
}
void publish_hud_camera(const float (&pose)[211][16],const float* root_matrix,const std::int16_t* cell,const float* body_position,const vr_arms::Sample& sample,const float* view) {
  float cam[16],rendering[16]; std::uint64_t id=0;
  const int method=hud_frame_method.load(std::memory_order_relaxed);
  const bool slot_camera=method==1||method==3;
  if(!native_stereo::frame_camera(slot_camera?1:0,cam,&id)) return;
  if(!sample.view_valid) {if(method==2) return;}
  else if(native_stereo::frame_camera(1,rendering,&id)) {
    // The solve's own anchor: root + the offset the eyes also take when "Lock arms to view" is on (not the sample's raw offset).
    float rig[16]; vr_arms_math::basis_for_root(sample.basis,root_matrix,rig);
    for(int k=0;k<3;++k) rig[12+k]=root_matrix[12+k]+1024.f*cell[k]+view[k];
    float gap=0.f; for(int k=0;k<3;++k) gap+=(rig[12+k]-rendering[12+k])*(rig[12+k]-rendering[12+k]);
    float turn=vr_arms_math::yaw_of(root_matrix)-vr_arms_math::yaw_of(rendering);
    while(turn>3.14159265f) turn-=6.28318531f;
    while(turn<-3.14159265f) turn+=6.28318531f;
    {std::lock_guard<std::mutex> lock(hud_camera_mutex);
      hud_gap_mm=std::max(hud_gap_mm,std::sqrt(gap)*1000.f); hud_gap_degrees=std::max(hud_gap_degrees,std::fabs(turn)*57.2958f);}
    if(method==2) std::memcpy(cam,rig,sizeof(cam));
    if(!slot_camera) native_stereo::frame_camera(0,rendering,&id);
    // As the gun: measured from the anchor the hands solve on, so body travel between this palette and the camera cannot enter. Rotation stays the rendering camera's.
    if(method==1) for(int k=0;k<3;++k) cam[12+k]=rig[12+k];
  }
  const auto point=[&](int bone,float* out) {
    float w[3]; for(int k=0;k<3;++k) w[k]=pose[bone][12+k]+1024.f*cell[k];
    hud_to_camera(cam,w,out);
  };
  // The gun in the left hand: the mirrored right hold on the left wrist (clean_socket). L_weaponAttachmentHand is the foregrip, not the gun.
  // A frame without the hold keeps the last one: the foregrip bone is elsewhere, so falling back to it would throw the panel for that frame.
  float left_gun[16]; bool left_gun_ok=false;
  {static std::mutex left_hold_mutex; static float left_hold[16]; static bool left_hold_ok=false;
    CleanHandCache grip; {std::lock_guard<std::mutex> lock(clean_mutex); grip=clean_cache[1];}
    float h_r[16],h_l[16],inverse_h_l[16];
    const bool fresh=grip.valid&&grip.local.attach&&skin_arm_ik::inverse_affine(h_r,grip.local.rel);
    if(fresh) vr_arms_math::mirror_hold(h_r,h_l);
    const bool inverted=fresh&&skin_arm_ik::inverse_affine(inverse_h_l,h_l);
    std::lock_guard<std::mutex> lock(left_hold_mutex);
    if(inverted) {std::memcpy(left_hold,inverse_h_l,64); left_hold_ok=true;}
    if(left_hold_ok) {skin_arm_ik::multiply_affine(left_gun,left_hold,pose[16]); left_gun_ok=true;}}
  skin_palette_probe::HudCameraPose next; next.frame_id=id;
  for(int hand=0;hand<2;++hand) {
    next.hand_ok[hand]=sample.valid[hand];
    point(hand?115:16,next.wrist[hand]); point(hand?114:15,next.elbow[hand]);
    const int attach=attach_bone[hand]>=0?attach_bone[hand]:(hand?115:16);
    if(hand==0&&left_gun_ok) {
      float w[3]; for(int k=0;k<3;++k) w[k]=left_gun[12+k]+1024.f*cell[k];
      hud_to_camera(cam,w,next.attach[0]); hud_axes_to_camera(cam,left_gun,next.attach_axes[0]);
    } else {point(attach,next.attach[hand]); hud_axes_to_camera(cam,pose[attach],next.attach_axes[hand]);}
    std::memcpy(next.outward[hand],sample.local[hand],12);  // the same filtered controller roll the rig solves with
    hud_axes_to_camera(cam,pose[hand?115:16],next.wrist_axes[hand]);
    for(int a=0;a<3;++a) std::memcpy(next.grip_axes[hand]+a*3,sample.local[hand]+a*4,12);
  }
  for(int slot=0;slot<skin_palette_probe::HudBoneSlots;++slot) {
    const int bone=hud_bone_request[slot].load(std::memory_order_relaxed);
    next.bone_ok[slot]=bone>=0&&bone<211;
    if(next.bone_ok[slot]) {point(bone,next.bone[slot]); hud_axes_to_camera(cam,pose[bone],next.bone_axes[slot]);}
  }
  store_hud_camera(next);
}
ULONGLONG hud_anchor_smoothed=0;
ULONGLONG hud_anchor_at=0;
void publish_hud_anchor(const float (&pose)[211][16],const float* root_matrix,const std::int16_t* cell,const float* body_position) {
  vr_arms::Sample sample;
  {const auto hud=wrist_hud::current(); if(!hud.on&&!hud.native_arm) return;}
  if(!vr_arms::latest(sample)) return;
  // The sample this thread's solve used, when it solved: the arm and the widget then share one frame exactly.
  {vr_arms::Sample rig=sample; float view[3]; std::memcpy(view,vr_arms::anchor_view_offset(sample),12);
    {std::lock_guard<std::mutex> lock(hud_solve_mutex); if(hud_solve_ok) {rig=hud_solve_sample; std::memcpy(view,hud_solve_view,12);}}
    if(rig.view_valid) publish_hud_camera(pose,root_matrix,cell,body_position,rig,view);}
  if(!sample.view_valid || !sample.head_world_valid) return;
  float basis[16]; vr_arms_math::basis_for_root(sample.basis,root_matrix,basis);
  float head_local[3];
  for(int r=0;r<3;++r) head_local[r]=sample.head_world[0]*sample.basis[r*4]+sample.head_world[1]*sample.basis[r*4+1]+sample.head_world[2]*sample.basis[r*4+2];
  float points[2][3][3]{}; bool valid[2]{};
  for(int hand=0;hand<2;++hand) {
    if(!sample.valid[hand]) continue;
    const float* grip=sample.raw_local[hand];
    const unsigned bones[3]={hand?115u:16u,hand?114u:15u,attach_bone[hand]>=0?static_cast<unsigned>(attach_bone[hand]):(hand?115u:16u)};
    bool finite=true;
    for(int point=0;point<3;++point) {
      float world[3],local[3];
      for(int k=0;k<3;++k) world[k]=pose[bones[point]][12+k]-(body_position[k]-1024.f*cell[k])-sample.raw_view_offset[k];
      for(int r=0;r<3;++r) local[r]=world[0]*basis[r*4]+world[1]*basis[r*4+1]+world[2]*basis[r*4+2]-head_local[r]-grip[12+r];
      for(int r=0;r<3;++r) {
        points[hand][point][r]=local[0]*grip[r*4]+local[1]*grip[r*4+1]+local[2]*grip[r*4+2];
        finite=finite&&std::isfinite(points[hand][point][r])&&std::fabs(points[hand][point][r])<2.f;
      }
    }
    valid[hand]=finite;
  }
  if(!valid[0] && !valid[1]) return;
  std::lock_guard<std::mutex> lock(hud_anchor_mutex);
  // The offset between the drawn arm and the controller only drifts slowly; a short filter takes out recoil kicks.
  const ULONGLONG tick=GetTickCount64();
  const float dt=hud_anchor_smoothed?static_cast<float>(tick-hud_anchor_smoothed)*0.001f:0.f;
  if(dt>0.f||!hud_anchor_smoothed) hud_anchor_smoothed=tick;
  const float blend=1.f-std::exp(-dt/0.15f);
  for(int hand=0;hand<2;++hand) if(valid[hand]) {
    float jump=0.f;
    for(int k=0;k<3;++k) {const float d=points[hand][0][k]-hud_anchor_points[hand][0][k]; jump+=d*d;}
    if(!hud_anchor_primed[hand]||jump>0.3f*0.3f) {std::memcpy(hud_anchor_points[hand],points[hand],sizeof(points[hand])); hud_anchor_primed[hand]=true;}
    else for(int point=0;point<3;++point) for(int k=0;k<3;++k) hud_anchor_points[hand][point][k]+=(points[hand][point][k]-hud_anchor_points[hand][point][k])*blend;
  }
  hud_anchor_at=valid[0]&&valid[1]?GetTickCount64():0;
}
bool pulse_call(void* mesh,const void* root,const void* bones,const void* remap,const void* cells,std::uintptr_t caller_rva) {
  const auto now=GetTickCount64();
  const bool ik=controller_mode.load();
  const float delta=skin_palette_pulse::offset(static_cast<unsigned>(now-pulse_started.load()));
  if(now>=pulse_until.load() || !pulse_permission.load() || (!ik && delta==0.f))
    return original(mesh,root,bones,remap,cells);
  const auto player=game::pointers();
  if(!player.body || player.body!=pulse_body.load()) {pulse_until.store(0); return original(mesh,root,bones,remap,cells);}
  unsigned char header[16]{}; const void* source=nullptr; unsigned n=0;
  if(!read(bones,0,header,sizeof(header))) return original(mesh,root,bones,remap,cells);
  std::memcpy(&source,header,sizeof(source)); std::memcpy(&n,header+8,sizeof(n));
  if(ik) census(mesh,root,cells,n,player,caller_rva);
  if(source && ik && n>0 && n<=kMaxGearBones && n!=211) return accessory_call(mesh,root,bones,remap,cells,header,source,n,player,caller_rva);
  if(!source || n!=211) return original(mesh,root,bones,remap,cells);
  // The FP body latches its pose vector; any other mesh skinned to the same vector (accessories) gets the same pose.
  const void* latched=fp_source.load();
  const bool known=latched && source==latched;
  bool accessory=false;
  if(!known) {
    const void* resource=nullptr; const void* vt=nullptr;
    unsigned char name[16]{};
    if(!read(root,0x58,&resource,sizeof(resource)) || !read(resource,0,&vt,sizeof(vt)) ||
       vt!=game::module_base()+0x57C6D70 || !read(resource,0x18,name,sizeof(name)))
      return original(mesh,root,bones,remap,cells);
    char path[128]{};
    if(name[15]) {
      const void* heap=nullptr; std::memcpy(&heap,name,sizeof(heap));
      bool ended=false;
      for(unsigned i=0;i<sizeof(path)-1;++i) {
        if(!read(heap,i,path+i,1)) return original(mesh,root,bones,remap,cells);
        if(!path[i]) {ended=true; break;}
      }
      if(!ended) return original(mesh,root,bones,remap,cells);
    } else {std::memcpy(path,name,15);}
    if(_stricmp(path,"BLUE/BAKED/CHARACTERART/PLAYER/BODY/MALE/P_BODY_01_M/P_BODY_01_FP_M.MMB")) {
      // A 211-bone mesh with its own pose vector: logged once (evidence for accessory ownership), not modified.
      other211.fetch_add(1);
      bool expected=false;
      if(other211_logged.compare_exchange_strong(expected,true)) {std::memcpy(other211_path,path,sizeof(path)); other211_path[127]=0;}
      return original(mesh,root,bones,remap,cells);
    }
    const char* expected[]={"R_upperArm","R_lowerArm","R_hand"};
    for(unsigned k=0;k<3;++k) {
      Sample entry{};
      if(!read_name(root,113+k,entry) || entry.name_count!=211 || std::strcmp(entry.bone_name,expected[k]))
        return original(mesh,root,bones,remap,cells);
    }
    if(ik) {
      const char* left[]={"L_upperArm","L_lowerArm","L_hand"};
      for(unsigned k=0;k<3;++k) {Sample entry{};
        if(!read_name(root,14+k,entry)||entry.name_count!=211||std::strcmp(entry.bone_name,left[k]))
          return original(mesh,root,bones,remap,cells);
      }
    }
    fp_source.store(source);
    const void* empty=nullptr; pulse_mesh.compare_exchange_strong(empty,mesh);
  } else accessory=mesh!=pulse_mesh.load();
  float root_matrix[16]{},body_position[3]{}; std::int16_t cell[3]{};
  if(!read(root,0,root_matrix,sizeof(root_matrix)) || !read(cells,0,cell,sizeof(cell)) ||
     !rig_body_read(player.body,body_position)) return original(mesh,root,bones,remap,cells);
  {float live_body[3]; if(!read(player.body,0x94,live_body,sizeof(live_body))) std::memcpy(live_body,body_position,12);
    // A mesh on the latched first-person pose vector is the player's: only an absurd root stops it. New meshes keep the tight test.
    const float limit=known && rig_coherent?1.f:0.1f;
    for(unsigned k=0;k<3;++k) {
      const float difference=root_matrix[12+k]+1024.f*cell[k]-live_body[k];
      if(!std::isfinite(difference) || std::fabs(difference)>limit) {rig_fallback[0].fetch_add(1,std::memory_order_relaxed); return original(mesh,root,bones,remap,cells);}
    }}
  float snapshot[211][16],copy[211][16];
  if(!read(source,0,snapshot,sizeof(snapshot)))
    return original(mesh,root,bones,remap,cells);
  if(ik) {
    GripCache target;
    if(clean_rig) {std::lock_guard<std::mutex> lock(grip_mutex); target=grips;}  // log only: wait, never drop to flat
    else {
    {std::unique_lock<std::mutex> lock(grip_mutex,std::try_to_lock);
      if(!lock.owns_lock()) return original(mesh,root,bones,remap,cells);
      target=grips;
    }
    if(now-target.tick>100) return original(mesh,root,bones,remap,cells);
    }
    if(follow_enabled) read_sides(root);
    const bool sides=follow_enabled&&side_state.load()==2;
    // Read-only rig coupling probe, once a second: source (animated) bones and grips in mesh-root space, root scale.
    // Discriminates a camera/look-pitched FP rig and FOV-dependent arm placement from projection effects.
    if(ULONGLONG due=rig_log_next.load(); now>=due && rig_log_next.compare_exchange_strong(due,now+5000)) {
      float inverse_root[16];
      if(skin_arm_ik::inverse_affine(inverse_root,root_matrix)) {
        auto local=[&](const float* m){float r[16];skin_arm_ik::multiply_affine(r,m,inverse_root);return skin_arm_ik::Vec{r[12],r[13],r[14]};};
        auto row_length=[&](unsigned i){return skin_arm_ik::length(skin_arm_ik::row(root_matrix,i));};
        const skin_arm_ik::Vec cam=local(snapshot[1]),head=local(snapshot[8]),ls=local(snapshot[13]),rs=local(snapshot[112]);
        const skin_arm_ik::Vec cam_fwd=skin_arm_ik::row(snapshot[1],1),head_fwd=skin_arm_ik::row(snapshot[8],1);
        skin_arm_ik::Vec grip[2]{};
        for(unsigned hand=0;hand<2;++hand) if(target.valid[hand]) {
          float g[16]; std::memcpy(g,target.matrix[hand],sizeof(g));
          g[12]-=1024.f*cell[0]; g[13]-=1024.f*cell[1]; g[14]-=1024.f*cell[2]; grip[hand]=local(g);
        }
        char rig[520];
        std::snprintf(rig,sizeof(rig),"skin ik rig: root_scale=%.3f,%.3f,%.3f root_fwd=%.3f,%.3f,%.3f cam=%.3f,%.3f,%.3f head=%.3f,%.3f,%.3f "
          "lshoulder=%.3f,%.3f,%.3f rshoulder=%.3f,%.3f,%.3f cam_row1=%.3f,%.3f,%.3f head_row1=%.3f,%.3f,%.3f lgrip=%.3f,%.3f,%.3f rgrip=%.3f,%.3f,%.3f grips=%d,%d",
          row_length(0),row_length(1),row_length(2),root_matrix[4],root_matrix[5],root_matrix[6],cam.x,cam.y,cam.z,head.x,head.y,head.z,
          ls.x,ls.y,ls.z,rs.x,rs.y,rs.z,cam_fwd.x,cam_fwd.y,cam_fwd.z,head_fwd.x,head_fwd.y,head_fwd.z,
          grip[0].x,grip[0].y,grip[0].z,grip[1].x,grip[1].y,grip[1].z,target.valid[0]?1:0,target.valid[1]?1:0);
        log(rig);
      }
    }
    static thread_local float raw[211][16];
    float out_root[16]; bool solved=false;
    {float mk[16]; const bool mv=read_marker90(mk);
      std::lock_guard<std::mutex> lock(fp_last_mutex); std::memcpy(fp_root_latest,root_matrix,64); fp_root_valid=true;
      fp_root_marker_valid=mv; if(mv) std::memcpy(fp_root_marker,mk,64);}
    if(!sides || !shared_fp(root_matrix,cell,body_position,source,now,raw,copy,out_root,&solved)) {
      if(clean_rig) flat_frames.fetch_add(1,std::memory_order_relaxed);
      rig_fallback[3].fetch_add(1,std::memory_order_relaxed);
      return original(mesh,root,bones,remap,cells);}
    // A result solved earlier this frame for a gear mesh shares this root (dist 0); re-root for safety.
    unsigned reroot_angle=0;
    if(!solved) {
      float inverse_then[16],reroot[16];
      if(skin_arm_ik::inverse_affine(inverse_then,out_root)) {
        skin_arm_ik::multiply_affine(reroot,inverse_then,root_matrix);
        reroot_angle=reroot_centideg(reroot);
        for(auto& m:copy){float r[16]; skin_arm_ik::multiply_affine(r,m,reroot); std::memcpy(m,r,sizeof(r));}
      }
    }
    {const float y3[3]={std::atan2(root_matrix[4],root_matrix[6])*57.2958f,std::atan2(root_matrix[8],root_matrix[10])*57.2958f,0.f};
      vr_weapon::timing_mark('Y',y3);}  // run 48: Y = FP body root yaw (x: row 1, y: row 2) as drawn
    {const int wh=(left_handed?0:1);  // V: the weapon hand as the FP body draws it (value: re-root centi-degrees)
      if(attach_bone[wh]>=0) {float v[3]; for(int k=0;k<3;++k) v[k]=copy[attach_bone[wh]][12+k]+1024.f*cell[k]; vr_weapon::timing_mark('V',v,reroot_angle);}}
  } else if(!skin_palette_pulse::copy_right(copy,snapshot,delta)) return original(mesh,root,bones,remap,cells);
  const void* check=nullptr; unsigned check_n=0;
  if(!read(bones,0,&check,sizeof(check)) || check!=source || !read(bones,8,&check_n,sizeof(check_n)) || check_n!=211 ||
     game::pointers().body!=player.body || !pulse_permission.load() || GetTickCount64()>=pulse_until.load())
    return original(mesh,root,bones,remap,cells);
  const void* replacement=copy; std::memcpy(header,&replacement,sizeof(replacement));
  pulse_calls.fetch_add(1);
  if(!accessory) {vr_weapon::timing_mark('B'); mirror_probe(root);}
  {unsigned g; {std::lock_guard<std::mutex> lock(gun_mutex); g=gun_actual_gen;} (g==frame_gen.load()?fp_after_gun:fp_before_gun).fetch_add(1);}
  (accessory?accessory_apps:fp_body_apps).fetch_add(1);
  if(ik) gesture_palette(root,copy,211);
  // Every mesh here shares the FP pose vector, so the bones are the same; "accessory" can stay set for the FP body after a re-arm.
  if(ik) publish_hud_anchor(copy,root_matrix,cell,body_position);
  // The inspected palette builder consumes the vector synchronously; it does not retain this header/data.
  return original(mesh,root,header,remap,cells);
}
// FEATURE-06 (2026-10-24 HEADSET): while the eyes ride on the third-person head (mount_camera::rider_view) the player's
// head and hair meshes are collapsed (zero-scale bones, renderer copy only) and the first-person arm IK is not applied:
// it solves on the first-person rig, which the game does not animate while mounted, so gear floated beside the body.
// Stage B: the arms are solved on the animated 192-bone third-person pose instead (rider_ik.hpp) and every player mesh
// whose L_/R_ arm bones match that pose in place gets the same rigid per-role change (fail closed per mesh).
// Live IK tuning for the third-person rider rig (tools/ik_tuner -> scripts/AvatarVR_ik_tune.txt, re-read 4x a second on
// the Present thread; absent keys inherit the first-person hand tune). rider_tune_mutex guards the parsed copy.
// Bounded diagnostics are queued on palette workers and written only on Present.
std::mutex rider_diag_mutex;
std::vector<std::string> rider_diag;
std::unordered_set<std::string> rider_diag_seen;
struct RiderGearPresence {
  unsigned calls=0, wrist_samples=0; std::uint64_t sequence=0; ULONGLONG last=0;
  float root_gap=0.f,wrist_gap=0.f;
  unsigned divergence_generation=0,divergence_reports=0;
  ULONGLONG next_divergence=0;
  unsigned visibility_generation=0,visibility_reports=0;
  ULONGLONG next_visibility=0;
};
std::unordered_map<std::string,RiderGearPresence> rider_gear_presence; // rider_diag_mutex, up to 24 paths
void rider_note(const std::string& line) {
  std::lock_guard<std::mutex> lock(rider_diag_mutex);
  if(rider_diag.size()<256) rider_diag.push_back(line);
}
std::mutex rider_tune_mutex;
rider_ik::Tune rider_tune;          // rider_tune_mutex
std::string rider_tune_text;        // Present thread: last file content (change log)
ULONGLONG rider_tune_next=0;        // Present thread
rider_ik::Tune rider_tune_now() {std::lock_guard<std::mutex> lock(rider_tune_mutex); return rider_tune;}
// Riding per-draw trace (2026-11-08, read-only): from 3 s into a ride, 20 s or 60000 draws, then RIDER_TRACE.csv. Per rider mesh draw: the
// frame, the camera step the arms were solved for and the newest one, the mesh root, the arms' body, the newest camera, and both hands as
// drawn (world). Tells whether hands jump against the camera, against their own mesh, or between meshes of one frame.
// [2026-11-08 family audit] every exit of rider_call for a player mesh with an arm chain is recorded, IK or game pose, as a ring of the last
// 60000 draws of a ride, written when the ride ends: a frame where one side's gear is on the IK arm while another mesh of that side is not.
struct RiderTraceRow {long long us; unsigned gen; std::uint64_t step_used,step_new; const void* resource; float root[3],body[3],cam[3],hand[2][3]; signed char have[2];
  signed char ik[2]; unsigned char weight[2],held,phase,solved;
  // [2026-11-08 18th] the tick solve's controller target per hand (world: sample grip carried to the solve body), the sample's camera and
  // body, and whether the gun hook made that tick's solve: hand-to-target gap = arm short of the controller (animation pulling the shoulder,
  // reach); sample camera travel against body travel between ticks = camera not rigid with the rider.
  float target[2][3]; float scam[3],sbody[3]; unsigned char by_gun;};
std::mutex rider_trace_mutex; std::vector<RiderTraceRow> rider_trace; std::vector<std::pair<const void*,std::string>> rider_trace_paths;
std::size_t rider_trace_next=0; unsigned rider_trace_files=0; ULONGLONG rider_trace_flushed_at=0;
void rider_trace_add(const RiderTraceRow& row,const std::string& path) {
  std::lock_guard<std::mutex> lock(rider_trace_mutex);
  if(rider_trace_files>=4 || mount_state::get().phase==mount_state::Phase::None) return;
  if(rider_trace.size()<60000) {if(rider_trace.empty()) rider_trace.reserve(60000); rider_trace.push_back(row);}
  else rider_trace[rider_trace_next%60000]=row;
  ++rider_trace_next;
  bool known=false; for(const auto& p:rider_trace_paths) if(p.first==row.resource) {known=true; break;}
  if(!known && rider_trace_paths.size()<64) rider_trace_paths.emplace_back(row.resource,path);
}
void rider_trace_flush() {  // Present thread
  std::vector<RiderTraceRow> rows; std::vector<std::pair<const void*,std::string>> paths;
  {std::lock_guard<std::mutex> lock(rider_trace_mutex);
    // Written when the ride ends, or every 45 s once the ring is full while riding (a game closed mid-ride wrote nothing, run 174252).
    const bool ride_over=mount_state::get().phase==mount_state::Phase::None;
    const bool periodic=rider_trace_next>=60000 && GetTickCount64()-rider_trace_flushed_at>45000;
    if(rider_trace.empty() || rider_trace_files>=6 || !(ride_over || periodic)) return;
    rider_trace_flushed_at=GetTickCount64();
    rows.swap(rider_trace); paths.swap(rider_trace_paths);
    if(rider_trace_next>rows.size()) std::rotate(rows.begin(),rows.begin()+static_cast<std::ptrdiff_t>(rider_trace_next%rows.size()),rows.end());
    rider_trace_next=0; ++rider_trace_files;}
  // [2026-11-08] the first trace (33814 draws) was written and then gone from scripts\ by the next look: the summary goes into this log as
  // well, and the file name is unique per launch.
  {
    // Per Present generation: how many camera steps and bodies its draws used. Per mesh: each hand relative to the newest camera, step
    // between consecutive draws of that mesh (a flash is a one-draw jump). Per generation: spread of one hand across meshes (gear vs arm).
    std::vector<float> jump[2],spread[2],root_gap; unsigned gens=0,multi_step=0,multi_body=0;
    struct Last {bool have[2]{}; float rel[2][3]{}; unsigned gen=0;};
    std::vector<Last> last(paths.size()+1);
    std::size_t i=0;
    while(i<rows.size()) {
      std::size_t j=i; while(j<rows.size() && rows[j].gen==rows[i].gen) ++j;
      ++gens; bool steps=false,bodies=false; float lo[2][3],hi[2][3]; bool any[2]{};
      for(std::size_t k=i;k<j;++k) {
        const auto& r=rows[k];
        if(r.step_used!=rows[i].step_used) steps=true;
        float db=0.f; for(int c=0;c<3;++c) db+=(r.body[c]-rows[i].body[c])*(r.body[c]-rows[i].body[c]);
        if(db>0.01f*0.01f) bodies=true;
        float rg=0.f; for(int c=0;c<3;++c) rg+=(r.root[c]-r.body[c])*(r.root[c]-r.body[c]); root_gap.push_back(std::sqrt(rg));
        unsigned id=0; for(unsigned m=0;m<paths.size();++m) if(paths[m].first==r.resource) {id=m; break;}
        for(int h=0;h<2;++h) if(r.have[h]) {
          float rel[3]; for(int c=0;c<3;++c) rel[c]=r.hand[h][c]-r.cam[c];
          if(!any[h]) {for(int c=0;c<3;++c) lo[h][c]=hi[h][c]=r.hand[h][c]; any[h]=true;}
          else for(int c=0;c<3;++c) {lo[h][c]=std::min(lo[h][c],r.hand[h][c]); hi[h][c]=std::max(hi[h][c],r.hand[h][c]);}
          auto& L=last[id];
          if(L.have[h] && L.gen!=r.gen) {float d=0.f; for(int c=0;c<3;++c) d+=(rel[c]-L.rel[h][c])*(rel[c]-L.rel[h][c]); jump[h].push_back(std::sqrt(d));}
          L.have[h]=true; std::memcpy(L.rel[h],rel,12); L.gen=r.gen;
        }
      }
      for(int h=0;h<2;++h) if(any[h]) {float d=0.f; for(int c=0;c<3;++c) d+=(hi[h][c]-lo[h][c])*(hi[h][c]-lo[h][c]); spread[h].push_back(std::sqrt(d));}
      multi_step+=steps; multi_body+=bodies; i=j;
    }
    auto q=[](std::vector<float>& v,float p){if(v.empty()) return 0.f; std::sort(v.begin(),v.end()); return v[static_cast<std::size_t>(p*(v.size()-1))];};
    auto over=[](const std::vector<float>& v,float t){unsigned n=0; for(float x:v) n+=x>t; return n;};
    char line[600];
    std::snprintf(line,sizeof(line),"rider trace summary: draws=%zu gens=%u gens_two_steps=%u gens_two_bodies=%u | hand-vs-camera step per mesh L p50/p99/max %.1f/%.1f/%.1f cm over5cm=%u R %.1f/%.1f/%.1f cm over5cm=%u | "
      "hand spread across meshes in one gen L p99/max %.1f/%.1f cm R %.1f/%.1f cm | root-to-arm-body p50/p99/max %.1f/%.1f/%.1f cm",
      rows.size(),gens,multi_step,multi_body,
      q(jump[0],.5f)*100,q(jump[0],.99f)*100,q(jump[0],1.f)*100,over(jump[0],.05f),q(jump[1],.5f)*100,q(jump[1],.99f)*100,q(jump[1],1.f)*100,over(jump[1],.05f),
      q(spread[0],.99f)*100,q(spread[0],1.f)*100,q(spread[1],.99f)*100,q(spread[1],1.f)*100,
      q(root_gap,.5f)*100,q(root_gap,.99f)*100,q(root_gap,1.f)*100);
    log(line);
    // Family: per frame and side, meshes drawn on the IK arm against meshes of that side drawn in the game pose.
    unsigned split[2]{},native_all[2]{},ik_all[2]{},held_frames[2]{},unsolved=0; std::vector<unsigned> native_by_mesh(paths.size()+1,0);
    for(std::size_t a=0;a<rows.size();) {
      std::size_t b=a; while(b<rows.size() && rows[b].gen==rows[a].gen) ++b;
      bool any_unsolved=false;
      for(int h=0;h<2;++h) {
        unsigned ik=0,nat=0; bool held=false;
        for(std::size_t k=a;k<b;++k) {const auto& r=rows[k]; if(!r.solved) any_unsolved=true; if(!r.have[h]) continue; if(r.ik[h]) ++ik; else ++nat; held|=((r.held>>h)&1)!=0;}
        if(ik&&nat) {
          ++split[h];
          for(std::size_t k=a;k<b;++k) if(rows[k].have[h] && !rows[k].ik[h]) {
            unsigned id=static_cast<unsigned>(paths.size());
            for(unsigned m=0;m<paths.size();++m) if(paths[m].first==rows[k].resource) {id=m; break;}
            ++native_by_mesh[id];
          }
        } else if(nat) ++native_all[h]; else if(ik) ++ik_all[h];
        held_frames[h]+=held?1u:0u;
      }
      unsolved+=any_unsolved?1u:0u; a=b;
    }
    std::string worst;
    for(unsigned m=0;m<paths.size();++m) if(native_by_mesh[m]) {
      const auto& path=paths[m].second; const auto slash=path.find_last_of('/');
      worst+=" "+path.substr(slash==std::string::npos?0:slash+1)+"="+std::to_string(native_by_mesh[m]);
    }
    std::snprintf(line,sizeof(line),"rider family: frames L ik/native/split %u/%u/%u R %u/%u/%u | harness held frames L %u R %u | frames with an unsolved draw %u | game-pose draws in split frames:%s",
      ik_all[0],native_all[0],split[0],ik_all[1],native_all[1],split[1],held_frames[0],held_frames[1],unsolved,worst.substr(0,300).c_str());
    log(line);
    // Hand drawn against its controller target (IK draws only), and camera travel against body travel per new sample.
    std::vector<float> gap[2],cam_body;
    for(const auto& r:rows) for(int h=0;h<2;++h) if(r.have[h] && r.ik[h] && r.solved) {
      float d=0.f; for(int c=0;c<3;++c) d+=(r.hand[h][c]-r.target[h][c])*(r.hand[h][c]-r.target[h][c]); gap[h].push_back(std::sqrt(d));}
    {const RiderTraceRow* last=nullptr;
      for(const auto& r:rows) if(r.solved && r.step_used) {
        if(last && r.step_used==last->step_used+1) {
          float d=0.f; for(int c=0;c<3;++c) {const float e=(r.scam[c]-last->scam[c])-(r.sbody[c]-last->sbody[c]); d+=e*e;}
          cam_body.push_back(std::sqrt(d));
        }
        if(!last || r.step_used!=last->step_used) last=&r;
      }}
    std::snprintf(line,sizeof(line),"rider reach: hand-to-controller gap L p50/p99/max %.1f/%.1f/%.1f cm over2cm=%u R %.1f/%.1f/%.1f cm over2cm=%u | camera-vs-body travel per sample p50/p99/max %.1f/%.1f/%.1f cm over2cm=%u",
      q(gap[0],.5f)*100,q(gap[0],.99f)*100,q(gap[0],1.f)*100,over(gap[0],.02f),q(gap[1],.5f)*100,q(gap[1],.99f)*100,q(gap[1],1.f)*100,over(gap[1],.02f),
      q(cam_body,.5f)*100,q(cam_body,.99f)*100,q(cam_body,1.f)*100,over(cam_body,.02f));
    log(line);
  }
  wchar_t name[64]; std::swprintf(name,64,L"\\RIDER_TRACE_%llu.csv",static_cast<unsigned long long>(GetTickCount64()));
  FILE* f=nullptr; if(_wfopen_s(&f,(dir+name).c_str(),L"w")!=0 || !f) {log("rider trace: file open failed"); return;}
  std::fputs("us,gen,step_used,step_new,mesh,root_x,root_y,root_z,body_x,body_y,body_z,cam_x,cam_y,cam_z,have_l,lx,ly,lz,have_r,rx,ry,rz,ik_l,ik_r,w_l,w_r,held,phase,solved,tlx,tly,tlz,trx,try,trz,scam_x,scam_y,scam_z,sbody_x,sbody_y,sbody_z,by_gun\n",f);
  for(const auto& r:rows) {
    unsigned id=0; for(unsigned i=0;i<paths.size();++i) if(paths[i].first==r.resource) {id=i; break;}
    std::fprintf(f,"%lld,%u,%llu,%llu,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%d,%d,%u,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u\n",
      r.us-rows.front().us,r.gen,static_cast<unsigned long long>(r.step_used),static_cast<unsigned long long>(r.step_new),id,
      r.root[0],r.root[1],r.root[2],r.body[0],r.body[1],r.body[2],r.cam[0],r.cam[1],r.cam[2],
      r.have[0],r.hand[0][0],r.hand[0][1],r.hand[0][2],r.have[1],r.hand[1][0],r.hand[1][1],r.hand[1][2],
      r.ik[0],r.ik[1],r.weight[0],r.weight[1],r.held,r.phase,r.solved,r.target[0][0],r.target[0][1],r.target[0][2],r.target[1][0],r.target[1][1],r.target[1][2],
      r.scam[0],r.scam[1],r.scam[2],r.sbody[0],r.sbody[1],r.sbody[2],r.by_gun);
  }
  std::fputs("# meshes:",f); for(unsigned i=0;i<paths.size();++i) std::fprintf(f," %u=%s",i,paths[i].second.c_str()); std::fputs("\n",f);
  std::fclose(f);
  char line[200]; std::snprintf(line,sizeof(line),"rider trace: wrote %zu draws to %ls",rows.size(),name+1); log(line);
}
void rider_tune_poll() {
  rider_trace_flush();
  std::vector<std::string> notes;
  const bool mounted=mount_state::mounted();
  {std::lock_guard<std::mutex> lock(rider_diag_mutex); notes.swap(rider_diag);
    static ULONGLONG next_presence=0;
    static unsigned presence_reports=0;
    if(!mounted) {presence_reports=0; next_presence=0;}
    if(GetTickCount64()>=next_presence && mounted && presence_reports<30) {
      ++presence_reports;
      next_presence=GetTickCount64()+2000;
      for(auto& item:rider_gear_presence) {
        auto& p=item.second;
        notes.push_back("rider gear presence "+item.first+" palette_calls="+std::to_string(p.calls)+
          " last_seq="+std::to_string(p.sequence)+" age_ms="+std::to_string(GetTickCount64()-p.last)+
          " root_gap_m="+std::to_string(p.root_gap)+" wrist_gap_m="+std::to_string(p.wrist_gap)+
          " wrist_samples="+std::to_string(p.wrist_samples));
        p.calls=0; p.wrist_samples=0; p.root_gap=0.f; p.wrist_gap=0.f;
      }
    }
  }
  for(const auto& line:notes) log(line.c_str());
  const ULONGLONG now=GetTickCount64();
  if(dir.empty() || now<rider_tune_next) return;
  rider_tune_next=now+250;
  std::string text;
  FILE* f=nullptr;
  if(_wfopen_s(&f,(dir+L"\\AvatarVR_ik_tune.txt").c_str(),L"rb")==0 && f) {
    char buf[4096]; std::size_t n=0;
    while((n=std::fread(buf,1,sizeof(buf),f))>0 && text.size()<65536) text.append(buf,n);
    std::fclose(f);
  }
  const rider_ik::Tune parsed=rider_ik::parse_tune(text.c_str(),hand_tune);
  {std::lock_guard<std::mutex> lock(rider_tune_mutex); rider_tune=parsed;}
  if(text!=rider_tune_text) {
    rider_tune_text=text;
    char line[400];
    std::snprintf(line,sizeof(line),"rider ik tune: hand %.0f/%.0f/%.0f deg %.3f/%.3f/%.3f m palm %.2f; shoulder out/up/fwd %.3f/%.3f/%.3f; "
      "reach x%.2f, clavicle %.3f m; elbow out/down/back %.2f/%.2f/%.2f; twist fore/upper %.2f/%.2f",
      parsed.hand.pitch,parsed.hand.yaw,parsed.hand.roll,parsed.hand.forward,parsed.hand.up,parsed.hand.side,parsed.hand.palm,
      parsed.shoulder[0],parsed.shoulder[1],parsed.shoulder[2],parsed.reach_scale,parsed.max_reach,
      parsed.elbow[0],parsed.elbow[1],parsed.elbow[2],parsed.twist_fore,parsed.twist_upper);
    log(line);
    log(parsed.correction ? "rider correction ON" : "rider correction OFF (preceding mounted solve)");
  }
}
std::mutex rider_mutex;
struct RiderMesh {
  bool correction=true;
  const void* resource=nullptr;
  bool hide=false;
  bool player=false;                   // the rider's own art/gear (else: mount, harness, world)
  bool rooted=false;                   // root seen exactly on the body: its root is the body of the tick it is drawn at
  std::string path;
  bool arms=false;                     // has bones that ride an arm (named or anchored)
  bool anchored=false;                 // unnamed bones were assigned to arm segments (needs a solved pose)
  std::vector<signed char> role,side;  // per mesh bone (rider_ik::Role, 0 left / 1 right)
  std::vector<short> pose_index;       // same-named 192 bone, or -1
  std::vector<float> along;            // position along its segment (twist share)
  std::vector<char> twist_bone;
  std::vector<char> fixed_helper;
  std::vector<float> helper_local,helper_along;
  std::vector<short> helper_anchor;    // unnamed gear decoration -> stable named arm attachment
  std::vector<short> hand_node,hand_anchor; // named finger chains and their deformation helpers only
  std::vector<signed char> hand_family;
  std::vector<float> hand_reference;        // own mesh matrices in its neutral anatomical palm frame
  float hand_palm[2][16]{};
  unsigned hand_generation=0;
  bool hand_valid[2]{};
  unsigned helper_generation=0;
  std::vector<char> candidate;         // no arm name and not a body part: may be anchored
  short joint[2][7]={{-1,-1,-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1,-1,-1}};  // own arm joints (rider_ik::ArmBones order)
};
std::unordered_map<const void*,RiderMesh> rider_meshes;  // rider_mutex
struct RiderGood {
  const void* resource=nullptr;
  const void* owner=nullptr;
  std::uint64_t mount_id[2]{};
  std::vector<float> local;  // complete output in this mesh's root space (cell independent)
  ULONGLONG at[2]{};
};
std::unordered_map<const void*,RiderGood> rider_good; // rider_mutex, bounded to 64 meshes
std::unordered_map<std::string,unsigned> rider_reject_budget; // rider_mutex: 3 reports per mesh path/side, 32 paths
struct RiderSolve {
  float key[2][16]{};                  // pose hand matrices the deltas were solved for
  bool have=false;
  rider_ik::Arm arm[2]{};
  rider_ik::AttachmentPose attachment[2]; // world-space outputs paired atomically with each successful solve
  bool ok[2]{};
  float positions[rider_ik::kPoseBones][3]{};  // animated pose the deltas were solved on (cell-local)
  float body[3]{};                     // body position when that pose was read (world)
  std::int16_t cell[3]{};              // pose-local -> world for gear alignment diagnostics
  skin_arm_ik::Vec bend[2]{};
  float joints[2][4][16]{};            // that pose's arm joints per side, rider_ik::Role order (upper, lower, hand, clavicle)
  float weapon_socket[2][16]{};        // same native pose snapshot: L62 / R100
  bool weapon_ok[2]{};                // C1 never applies a delta retained after a failed arm solve
  rider_ik::Rest rest;                 // rest arm captured this mount (view frame)
  unsigned rest_generation=0;
  native_stereo::GripFrame input;       // one camera/body/grip sample for this solve and its mesh copies
  bool tick_by_gun=false;               // rider_tick_solve: the gun entity hook made this tick's solve (trace only)
  unsigned char tick_kind=0;            // rider_tick_solve: 0 own step's sample, 1 provisional (earlier sample carried), 2 refreshed (trace only)
  float twist_reference[2]{};
  bool twist_valid[2]{};
  ULONGLONG next_twist_report=0;
  unsigned twist_reports=0;
  const void* owner=nullptr;
  const void* partner=nullptr;
  std::uint64_t mount_id[2]{};
  bool correction=true;
  unsigned diagnostic_budget=8;
  mount_state::Phase phase=mount_state::Phase::None;
  ULONGLONG last_call=0;               // last rider_call: a gap means a new mount (rest re-captured)
};
RiderSolve rider_solve;  // rider_mutex
std::atomic<unsigned> rider_hidden{0},rider_arm_apps{0},rider_solves{0},rider_mismatch{0},rider_no_grip{0},rider_anchored{0},rider_self_solves{0},rider_rest_solves{0},rider_rest_captures{0};
std::atomic<int> rider_mismatch_mm{0};  // largest rejected offset, millimetres
std::atomic<int> rider_mismatch_bone{-1};  // 192 index of the last rejected joint
// Current 192 pose (cell-local), its cell, and the solved arms. Called under rider_mutex.
native_stereo::GripFrame rider_commit_input; long long rider_commit_us=0; unsigned rider_commit_gen=0;  // rider_mutex: the input of the placed gun
native_stereo::GripFrame rider_input_now,rider_input_before;  // rider_mutex: the latest grip frame and the one before it
// The mounted gun is solved relative to the cell where this ride began, as the on-foot gun is in its cell: world floats at the flight
// coordinates (|x| ~ 9800) have a 0.98 mm grid. Set once per mount (rider_mutex); read elsewhere only while that mount lasts.
float ride_origin[3]{}; bool ride_origin_set=false;
std::int16_t ride_origin_cell[3]{};
// One rider input and one rider body per Present generation, as shared_fp gives every on-foot mesh one solve per frame (rider_mutex).
// A commit landing later in the same generation replaces a non-committed choice once, as an on-foot gun placement re-solves the tick.
struct RiderFrameLatch {
  unsigned gen=0; bool valid=false,committed=false; const void* owner=nullptr; std::uint64_t step=0;  // step: newest camera step seen
  native_stereo::GripFrame input; float body[3]{};
  bool ahead=false;  // carried to a mesh root of the camera step not yet published (see rider_update_solve)
  long long commit_us=0;  // the gun placement whose input this latch holds (0: none)
};
RiderFrameLatch rider_frame;
// The camera and body of each recent camera step (rider_mutex). [LIVE 10-26 trace] a rider mesh root is the body of its own step, so a
// mesh finds its step here and with it the camera yaw its arms belong to (on foot: the root a mesh is re-rooted to, rotation included).
struct RiderTick {std::uint64_t sequence=0; float yaw=0.f; float body[3]{}; native_stereo::GripFrame frame;};
RiderTick rider_ticks[8]; unsigned rider_tick_next=0;
bool rider_tick_yaw(const float* root_world,float* yaw) {  // rider_mutex
  float best=0.02f*0.02f; bool found=false;
  for(const auto& t:rider_ticks) {
    if(!t.sequence) continue;
    float d=0.f; for(int k=0;k<3;++k) d+=(root_world[k]-t.body[k])*(root_world[k]-t.body[k]);
    if(d<best) {best=d; *yaw=t.yaw; found=true;}
  }
  return found;
}
bool same_grip_input(const native_stereo::GripFrame& a,const native_stereo::GripFrame& b) {
  return a.sequence==b.sequence && !std::memcmp(a.camera,b.camera,sizeof(a.camera)) && !std::memcmp(a.grip,b.grip,sizeof(a.grip));
}
// The newest controller sample, with a lost grip held (sixteenth step), recorded in the tick history (rider_mutex).
bool rider_take_sample(native_stereo::GripFrame& input) {
  if(!native_stereo::grip_frame(&input)) return false;
  // Sixteenth step [HEADSET 2026-11-08, 4853BF66: hands unstable overall, worst with a weapon in both hands]. [LIVE RIDER_TRACE_34711546]
  // the left grip was lost twice (1.4 s and 0.6 s, `no_grip` 896): the body's left arm went to the game pose (`rider missing solve`) while
  // the bracelets kept the last attachment, 62-72 cm apart, in 121 frames. On foot the last good rig is reused (clean_rig, 500 ms) and
  // never the game's arms. Riding: a lost grip is held at its last tracked pose, carried by the body, for up to 1.5 s, here at the one
  // input every member reads (body arms, gear, items, gun placement and pins), so the family holds together.
  if(rig_coherent && mounted_parity) {
    static float held_grip[2][16]; static float held_body[2][3]; static ULONGLONG held_at[2]{};  // rider_mutex
    const ULONGLONG now_ms=GetTickCount64();
    for(int h=0;h<2;++h) {
      if(input.valid[h]) {std::memcpy(held_grip[h],input.grip[h],64); std::memcpy(held_body[h],input.body,12); held_at[h]=now_ms;}
      else if(held_at[h] && now_ms-held_at[h]<1500) {
        std::memcpy(input.grip[h],held_grip[h],64);
        for(int k=0;k<3;++k) input.grip[h][12+k]+=input.body[k]-held_body[h][k];
        input.valid[h]=true; rider_grip_holds.fetch_add(1,std::memory_order_relaxed);
      }
    }
  }
  // [AUTHOR 2026-10-31] mounted must work like on foot. There, arms and gun take one sample per frame, the gun's, and the eyes sit on the same
  // anchor. Here the gun used the older sample and each draw re-solved on the newest, then moved the gun over; the eyes stayed on the raw rider
  // camera, so anything the second step missed left the gun. Now: the placed gun's input while it is fresh, turned to this eye's yaw;
  // a sample published this frame takes the eye shift, as vr_arms::shared_view_offset does.
  if(input.sequence!=rider_input_now.sequence) {rider_input_before=rider_input_now; rider_input_now=input;
    RiderTick& t=rider_ticks[rider_tick_next++%8]; t.sequence=input.sequence; t.yaw=vr_arms_math::yaw_of(input.camera); std::memcpy(t.body,input.body,12); t.frame=input;}
  return true;
}
bool rider_update_solve_core(const game::Pointers& player,const float* ahead_root,const float* tick_body,const native_stereo::GripFrame* forced=nullptr) {
  static float pose[rider_ik::kPoseBones][16];
  const void* comp=nullptr; const void* tp=nullptr; const void* data=nullptr; int count=0; std::int16_t cell[3]{};
  float body_now[3]{};
  if(!read(player.body,0x1A8,&comp,sizeof(comp)) || !read(comp,0x4F8,&tp,sizeof(tp)) || !read(tp,8,&count,sizeof(count)) ||
     count!=rider_ik::kPoseBones || !read(tp,0x74,cell,sizeof(cell)) || !read(tp,0,&data,sizeof(data)) ||
     !read(data,0,pose,sizeof(pose)) || !rig_body_read(player.body,body_now)) return false;
  RiderSolve& r=rider_solve;
  const auto identity=mount_state::get();
  const bool correction=rider_tune_now().correction;
  if(r.owner!=player.body || r.partner!=identity.partner || r.mount_id[0]!=identity.partner_id[0] ||
     r.mount_id[1]!=identity.partner_id[1] || r.correction!=correction ||
     (identity.phase==mount_state::Phase::Mounting && r.phase!=mount_state::Phase::Mounting)) {
    const auto last=r.last_call;
    r=RiderSolve{}; r.last_call=last; r.owner=player.body; r.partner=identity.partner; ride_origin_set=false; rider_frame=RiderFrameLatch{};
    r.mount_id[0]=identity.partner_id[0]; r.mount_id[1]=identity.partner_id[1]; r.correction=correction;
    rider_good.clear(); rider_reject_budget.clear();
  }
  r.phase=identity.phase;
  if(!ride_origin_set) {for(int k=0;k<3;++k) {ride_origin_cell[k]=cell[k]; ride_origin[k]=1024.f*cell[k];} ride_origin_set=true;}
  native_stereo::GripFrame input;
  if(!rider_take_sample(input)) return false;
  if(forced) input=*forced;  // per-tick solve: the sample of this tick's own camera step
  // Twenty-first step [HEADSET 2026-11-08, 5785E58A: armed still unstable; bare flickers and stutters again]. [LIVE RIDER_TRACE_41499031/
  // 41539984] the rider camera sits 1.3-1.4 m from the body origin; per step its offset from the body swings, moves radially and bobs
  // (p99 13-36 cm), so a sample carried by the body's travel alone missed the tick's camera by p50 2-4 cm, p99 12-36 cm. On foot the camera
  // sits over the body and that carry is nearly exact. Offline on the same traces, body travel plus the offset's velocity over the last
  // step missed by p50 0.3-0.8 cm, p99 4-19 cm (over 5 cm: 32 instead of 178, 10 instead of 157). A provisional sample (this tick's own step
  // not published yet) now takes that offset velocity as well; the body travel is added by the solve as before.
  if(tick_body && !forced && rig_coherent && mounted_parity) {
    const native_stereo::GripFrame* prev=nullptr;
    for(const auto& t:rider_ticks) if(t.sequence && t.sequence+1==input.sequence) prev=&t.frame;
    if(prev) {
      float extra[3],len=0.f;
      for(int k=0;k<3;++k) {extra[k]=(input.camera[12+k]-input.body[k])-(prev->camera[12+k]-prev->body[k]); len+=extra[k]*extra[k];}
      if(std::isfinite(len) && len<0.3f*0.3f) {
        for(int k=0;k<3;++k) {input.camera[12+k]+=extra[k]; input.grip[0][12+k]+=extra[k]; input.grip[1][12+k]+=extra[k];}
        rider_tick_predicted.fetch_add(1,std::memory_order_relaxed);
      }
      // Twenty-second step [HEADSET 2026-11-11, 7885F330: much better; hands shake while stick turning with the controllers still].
      // [LIVE RIDER_TRACE_43595171] hand jumps against the camera are on provisional ticks (provisional to provisional p90 7.4, p99 72 cm;
      // exact and refreshed p90 under 1 cm), 71 % of ticks provisional. A stick turn turns the camera every step, and the grips are camera
      // x controller: the position-only prediction missed one step of turn. The camera's own turn over the last step is applied too, about
      // the predicted camera (the same turn the next camera takes at a steady stick).
      float dyaw=std::remainder(vr_arms_math::yaw_of(input.camera)-vr_arms_math::yaw_of(prev->camera),6.2831853f);
      if(std::isfinite(dyaw) && std::fabs(dyaw)>1e-4f && std::fabs(dyaw)<0.25f) {
        const float cx=input.camera[12],cz=input.camera[14];
        for(int h=0;h<2;++h) skin_arm_ik::rotate_about_up(input.grip[h],cx,cz,dyaw);
        skin_arm_ik::rotate_about_up(input.camera,cx,cz,dyaw);
        rider_tick_turned.fetch_add(1,std::memory_order_relaxed);
        const int cd=static_cast<int>(std::fabs(dyaw)*5729.578f); int seen=rider_tick_turn_cdeg.load(std::memory_order_relaxed);
        while(cd>seen && !rider_tick_turn_cdeg.compare_exchange_weak(seen,cd,std::memory_order_relaxed)) {}
      }
    }
  }
  const unsigned gen_now=frame_gen.load(std::memory_order_relaxed);
  // Per-tick solve: the tick's own body and the newest controller sample, carried to that body; no latch, no gun-input choice.
  if(tick_body) std::memcpy(body_now,tick_body,sizeof(body_now));
  const bool latch_on=rig_coherent && mounted_parity && !t_rig_body_live && !tick_body;
  // [LIVE 2026-11-07/08] a camera step often lands inside one Present generation (body 160-330 mm on). On foot rig_body moves at every camera
  // step and shared_fp re-solves on the new tick; so the rider's choice now lasts one camera step within a generation, not the whole generation.
  const bool latch_here=latch_on && rider_frame.valid && rider_frame.gen==gen_now && rider_frame.owner==player.body &&
    rider_frame.step==rider_input_now.sequence;
  bool committed=false;
  if(rig_coherent && !tick_body) {
    float yaw_eye=vr_arms_math::yaw_of(input.camera),shift[3];
    // Only the frame's own placement: a tick with no placement must not put the arms on an older sample than the eyes.
    committed=!t_rig_body_live && rider_commit_us && rider_commit_gen==gen_now && clean_qpc_us()-rider_commit_us<50000;
    // Fifteenth step [HEADSET 2026-11-08, F37C1D59: stable bare-handed, very unstable with a weapon in both hands]. [LIVE run 164741] the
    // gun is placed on about half the frames (placed ~110 per 5 s against ~230 latches): the arms took the gun's input on those frames and
    // the previous sample on the others, a different input every other frame for two hands pinned to the gun. Riding: the latest placement
    // stays the arms' input while it is fresh (100 ms), carried by the body like any input; a newer placement replaces it.
    if(!committed && rig_coherent && mounted_parity && !t_rig_body_live && rider_commit_us && clean_qpc_us()-rider_commit_us<100000) {
      committed=true; rider_commit_held.fetch_add(1,std::memory_order_relaxed);
    }
    // Without a gun: the same choice as the placed gun makes, the input before this frame's eye step.
    const bool previous=!committed && !t_rig_body_live && vr_arms::frame_eye_shift(shift) && rider_input_before.sequence+1==rider_input_now.sequence;
    if(committed || previous) {
      input=committed?rider_commit_input:rider_input_before;
      float d=std::remainder(yaw_eye-vr_arms_math::yaw_of(input.camera),6.2831853f);
      if(std::fabs(d)>0.35f) d=0.f;
      if(d!=0.f) {
        const float cx=input.camera[12],cz=input.camera[14];
        for(int h=0;h<2;++h) skin_arm_ik::rotate_about_up(input.grip[h],cx,cz,d);
        skin_arm_ik::rotate_about_up(input.camera,cx,cz,d);
      }
    } else if(vr_arms::frame_eye_shift(shift)) {
      for(int k=0;k<3;++k) {input.camera[12+k]+=shift[k]; input.grip[0][12+k]+=shift[k]; input.grip[1][12+k]+=shift[k];}
    }
  }
  // Thirteenth step [HEADSET 2026-11-08, DLL 89D4D347: random disconnection everywhere, right-hand HUD and left-arm accessories flicker].
  // [LIVE RIDER_TRACE_13979062, 31482 draws] the game draws a tick's meshes before that tick's camera step is published: the songcord's
  // first draws in 12 % of frames, and every draw of a frame in 1.5 % (each visible 16-19 cm flash of both hands lined up with one). Their
  // roots are already on the new body; the arms were on the previous step's body. So a player mesh rooted on the body whose root has moved
  // a tick past the latched body, onto no published step, moves the latch now to what that step will give: its root as the body, the
  // newest sample as the input (the sample before that step, turned on by the last step's turn). The step's own latch then replaces it.
  if(latch_here && ahead_root) {
    float d2=0.f; for(int k=0;k<3;++k) d2+=(ahead_root[k]-rider_frame.body[k])*(ahead_root[k]-rider_frame.body[k]);
    float known_yaw=0.f;
    if(d2>0.01f*0.01f && d2<0.6f*0.6f && !rider_tick_yaw(ahead_root,&known_yaw)) {
      if(!rider_frame.committed && rider_input_now.sequence) {
        native_stereo::GripFrame next=rider_input_now;
        if(rider_input_before.sequence+1==rider_input_now.sequence) {
          float d=std::remainder(vr_arms_math::yaw_of(rider_input_now.camera)-vr_arms_math::yaw_of(rider_input_before.camera),6.2831853f);
          if(!(std::fabs(d)<0.1f)) d=0.f;
          if(d!=0.f) {
            const float cx=next.camera[12],cz=next.camera[14];
            for(int h=0;h<2;++h) skin_arm_ik::rotate_about_up(next.grip[h],cx,cz,d);
            skin_arm_ik::rotate_about_up(next.camera,cx,cz,d);
          }
        }
        rider_frame.input=next;
      }
      std::memcpy(rider_frame.body,ahead_root,sizeof(rider_frame.body));
      rider_frame.ahead=true;
      rider_prelatches.fetch_add(1,std::memory_order_relaxed);
    }
  }
  if(latch_here && (rider_frame.committed || !committed) && !(committed && rider_frame.commit_us!=rider_commit_us)) {
    // [SOURCE] without this, draws of one frame could each pick another input (newest sample, a new eye yaw, the commit) and another
    // body (a camera step mid-frame): each mesh drew its own arm. Counted, then replaced by the frame's choice.
    if(!same_grip_input(input,rider_frame.input)) rider_input_splits.fetch_add(1,std::memory_order_relaxed);
    if(!rider_frame.ahead) {const float d=std::sqrt((body_now[0]-rider_frame.body[0])*(body_now[0]-rider_frame.body[0])+(body_now[1]-rider_frame.body[1])*(body_now[1]-rider_frame.body[1])+
       (body_now[2]-rider_frame.body[2])*(body_now[2]-rider_frame.body[2]));
     const unsigned mm=std::isfinite(d)?static_cast<unsigned>(std::min(d,100.f)*1000.f):0u;
     unsigned seen=rider_body_drift_mm.load(std::memory_order_relaxed);
     while(mm>seen && !rider_body_drift_mm.compare_exchange_weak(seen,mm,std::memory_order_relaxed)) {}}
    input=rider_frame.input; std::memcpy(body_now,rider_frame.body,sizeof(body_now));
  } else if(latch_on) {
    if(latch_here) rider_relatches.fetch_add(1,std::memory_order_relaxed); else rider_latches.fetch_add(1,std::memory_order_relaxed);
    // A commit within a step already carried ahead keeps that body: the published body is still the step before.
    if(latch_here && rider_frame.ahead) std::memcpy(body_now,rider_frame.body,sizeof(body_now));
    else rider_frame.ahead=false;
    rider_frame.commit_us=committed?rider_commit_us:0; if(committed) rider_commit_latches.fetch_add(1,std::memory_order_relaxed);
    rider_frame.gen=gen_now; rider_frame.valid=true; rider_frame.committed=committed; rider_frame.owner=player.body; rider_frame.step=rider_input_now.sequence;
    rider_frame.input=input; std::memcpy(rider_frame.body,body_now,sizeof(body_now));
  }
  const bool same_input=r.input.sequence==input.sequence && !std::memcmp(r.input.camera,input.camera,sizeof(input.camera));
  r.input=input;
  if(r.have && same_input && (!(latch_on || tick_body) || !std::memcmp(r.body,body_now,sizeof(body_now))) && r.rest.valid[0] && r.rest.valid[1] &&
     !std::memcmp(r.key[0],pose[rider_ik::kArm[0].hand],64) && !std::memcmp(r.key[1],pose[rider_ik::kArm[1].hand],64))
    return true;  // same animated pose: reuse
  // Rest arm: once per mount, after a second of riding, in the eye / level view frame (rows of the anchored, levelled
  // game camera the grips are built on; origin = this pose's eye point from its own C_head, so pose time is consistent).
  {
    const auto ms=mount_state::get();
    if((!r.rest.valid[0] || !r.rest.valid[1]) && ms.phase==mount_state::Phase::Riding && GetTickCount64()-ms.since_ms>=1000 &&
       r.input.sequence!=0) {
      float frame[16]; std::memcpy(frame,r.input.camera,sizeof(frame));
      float eye[3]; mount_camera::eye_point(pose[15],eye);  // 192 C_head
      frame[3]=frame[7]=frame[11]=0.f; frame[12]=eye[0]; frame[13]=eye[1]; frame[14]=eye[2]; frame[15]=1.f;
      rider_ik::Rest next;
      bool valid=true;
      for(unsigned h=0;h<2;++h) {
        const auto& b=rider_ik::kArm[h];
        const float* const joints[rider_ik::kRestJoints]={pose[b.shoulder],pose[b.upper],pose[b.lower],pose[b.hand],
                                                          pose[b.index],pose[b.middle],pose[b.pinky]};
        valid=rider_ik::capture_rest(next,h,joints,frame) && valid;
      }
      if(valid && (!correction || rider_ik::symmetric_rest(next))) {
        r.rest=next; // both sides published together under rider_mutex
        r.twist_valid[0]=r.twist_valid[1]=false;
        r.rest_generation=rider_rest_captures.fetch_add(2,std::memory_order_relaxed)+2;
        char line[480];
        std::snprintf(line,sizeof(line),"rider rest pair mount=%llX:%llX correction=%d L clavicle %.3f,%.3f,%.3f upper %.3f,%.3f,%.3f R clavicle %.3f,%.3f,%.3f upper %.3f,%.3f,%.3f",
          static_cast<unsigned long long>(r.mount_id[0]),static_cast<unsigned long long>(r.mount_id[1]),correction?1:0,
          next.local[0][0][12],next.local[0][0][13],next.local[0][0][14],next.local[0][1][12],next.local[0][1][13],next.local[0][1][14],
          next.local[1][0][12],next.local[1][0][13],next.local[1][0][14],next.local[1][1][12],next.local[1][1][13],next.local[1][1][14]);
        rider_note(line);
        for(unsigned h=0;h<2;++h) for(int joint:{rider_ik::kRUpper,rider_ik::kRLower,rider_ik::kRHand}) {
          const float* m=next.local[h][joint];
          char basis_line[400];
          std::snprintf(basis_line,sizeof(basis_line),"rider rest axes side=%u joint=%d rows=%.6f,%.6f,%.6f/%.6f,%.6f,%.6f/%.6f,%.6f,%.6f pos=%.6f,%.6f,%.6f",
            h,joint,m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10],m[12],m[13],m[14]);
          rider_note(basis_line);
        }
      }
    }
  }
  const float* basis=r.input.camera;
  const float* body_then=r.input.body;
  for(unsigned h=0;h<2;++h) {
    float grip[16];
    if(!r.input.valid[h]) {rider_no_grip.fetch_add(1); r.ok[h]=false; continue;}
    std::memcpy(grip,r.input.grip[h],sizeof(grip));
    // World -> the pose's cell-local space, plus how far the body (and the mount) moved since the grips were sampled.
    for(int k=0;k<3;++k) grip[12+k]+=body_now[k]-body_then[k]-1024.f*cell[k];
    // A failed solve keeps the previous result for this hand: falling back to the native pose for one pose update
    // and back is the IK/native flicker of 2026-10-24.
    rider_ik::Arm next{};
    const auto tune=rider_tune_now();
    bool success=false;
    if(tune.correction && r.rest.valid[h]) {
      const auto& b=rider_ik::kArm[h];
      const float* anim[4]={pose[b.upper],pose[b.lower],pose[b.hand],pose[b.shoulder]};
      float placed[16]; std::memcpy(placed,basis,sizeof(placed));
      for(int k=0;k<3;++k) placed[12+k]+=body_now[k]-body_then[k]-1024.f*cell[k];
      // A fixed pole in the level view frame: never seed the rest elbow with the native riding elbow.
      success=rider_ik::solve_rest(r.rest,h,placed,anim,grip,basis,tune,next,nullptr);
      if(success) {
        if(r.twist_valid[h]) next.twist=rider_ik::continuous_twist(next.twist,r.twist_reference[h]);
        r.twist_reference[h]=next.twist; r.twist_valid[h]=true;
      }
    } else success=rider_ik::solve(pose,h,grip,basis,tune,next,&r.bend[h]);
    r.weapon_ok[h]=success;
    if(success) {
      r.arm[h]=next; r.ok[h]=true;
      const auto& b=rider_ik::kArm[h];
      const float* native[4]={pose[b.upper],pose[b.lower],pose[b.hand],pose[b.shoulder]};
      rider_ik::capture_attachment(r.attachment[h],native,next,
        {1024.f*cell[0],1024.f*cell[1],1024.f*cell[2]},
        {body_now[0],body_now[1],body_now[2]});
    }
  }
  std::memcpy(r.key[0],pose[rider_ik::kArm[0].hand],64); std::memcpy(r.key[1],pose[rider_ik::kArm[1].hand],64);
  for(int i=0;i<rider_ik::kPoseBones;++i) for(int k=0;k<3;++k) r.positions[i][k]=pose[i][12+k];
  std::memcpy(r.body,body_now,sizeof(body_now));
  std::memcpy(r.cell,cell,sizeof(cell));
  for(unsigned h=0;h<2;++h) {
    const auto& b=rider_ik::kArm[h];
    std::memcpy(r.weapon_socket[h],pose[h?100:62],64);
    std::memcpy(r.joints[h][rider_ik::kUpper],pose[b.upper],64); std::memcpy(r.joints[h][rider_ik::kLower],pose[b.lower],64);
    std::memcpy(r.joints[h][rider_ik::kHand],pose[b.hand],64); std::memcpy(r.joints[h][rider_ik::kShoulder],pose[b.shoulder],64);
  }
  r.have=true;
  if(correction && r.twist_reports<8 && GetTickCount64()>=r.next_twist_report) {
    r.next_twist_report=GetTickCount64()+1000; ++r.twist_reports;
    char line[240]; std::snprintf(line,sizeof(line),"rider coherent sample seq=%llu valid=%d,%d twist_deg=%.1f,%.1f rest=%d,%d",
      static_cast<unsigned long long>(r.input.sequence),r.input.valid[0]?1:0,r.input.valid[1]?1:0,
      r.twist_reference[0]*57.29578f,r.twist_reference[1]*57.29578f,r.rest.valid[0]?1:0,r.rest.valid[1]?1:0);
    rider_note(line);
  }
  rider_solves.fetch_add(1,std::memory_order_relaxed);
  return true;
}
// C1 ownership: a named player-weapon palette must match a native TP attachment in position AND rotation.
// The proof is private, short lived and bound to player/mount/current weapon candidate. Never use proximity alone.
struct MountedGunFrame : mounted_weapon_math::Identity {
  float flat[2][16]{},target[2][16]{},palm[2][16]{},palm_local[2][16]{};
  float body[3]{};  // the solve's body (world): every rider mesh of this frame is drawn at it (mounted_parity)
  bool valid[2]{}; unsigned char why[2]{};  // bits: palm, ok, weapon_ok, input, rest, rigid
  std::uint64_t sequence=0; // diagnostic controller identity
};
struct MountedGunProof {
  MountedGunFrame identity; const void* resource=nullptr; unsigned source=0; ULONGLONG at=0;
  float placed[16]{},offset[16]{}; const void* object=nullptr; bool have_placed=false;
  float pending[16]{}; const void* pending_object=nullptr;
  float target[16]{},pending_target[16]{}; unsigned hand=1,pending_hand=1; ULONGLONG committed_at=0;
  std::uint64_t pending_sequence=0,committed_sequence=0;
  float hold[2][16]{},support_relative[16]{},pending_palm[2][16]{},held_palm[2][16]{};
  bool hold_valid=false,support_valid=false;
  float pending_pin[2]{},pin[2]{};
  MountedProfile profile;
  bool onfoot_hold=false;
  const void* profile_resource=nullptr;
  std::string profile_path,bound_path;
  float pending_flat[16]{},committed_flat[16]{};
  float pending_local[3]{};  // the pending entity position, ride-local (exact, unlike the world copy)
  float pending_body[3]{},committed_body[3]{},pending_free[16]{},committed_free[16]{};
  mounted_weapon_math::PlacedFrames render_history;
};
std::mutex mounted_gun_mutex;
MountedGunProof mounted_gun_proof;
// ---- Per-tick rider solve (rider_tick_solve) ----
// [SOURCE cyberpunk-vr-port AnimPose.cpp] one arm solve per animation batch, replayed bit-identical to every pass of it. Here a game tick is
// identified by the body its meshes are posed at ([LIVE 10-26 MOUNT_CAMERA] every rider mesh root equals the body of its own step exactly).
// The first request for a tick (gun placement in the game thread, or that tick's first draw) solves it with the newest controller sample,
// carried to the tick's body; every later request for the same tick gets that solve unchanged. A standing mount gives every tick the same
// body: such a tick is solved again at each new controller sample.
// Nineteenth step [HEADSET 2026-11-08, A7958A71: random shakes, IK jumps, HUD disconnections]. [LIVE RIDER_TRACE_37834500/37879515] the
// hand always reaches its target (hand-to-controller gap 11.9-12.0 cm, the fixed wrist offset), so the flying animation does not pull it.
// The grips are built on each camera step's own camera (native_stereo compute_aim), the camera the eyes use; the rider camera does not move
// with the body (camera minus body travel per step p99 10-11 cm, max 36-45 cm; the eye anchor follows C_head). A tick solved from an earlier
// step's sample, carried by the body (bare 15 % of ticks; the gun hook almost always, carry about 19 cm), put the hands off the view by that
// difference. Now a tick takes its own step's sample when published; a solve made before it (gun hook, early draw) is provisional and is
// solved again once with the exact sample. The gun then follows the exact hand (eighteenth: follow_palm), as the Cyberpunk port's
// weapon follows the hand.
struct RiderTickEntry {bool valid=false,unique=false,by_gun=false,provisional=false,drawn=false; float body[3]{}; std::uint64_t step=0; RiderSolve solve;};
RiderTickEntry rider_tick_cache[6]; unsigned rider_tick_slot=0; int rider_tick_last=-1;  // rider_mutex
bool rider_update_solve(const game::Pointers& player,const float* ahead_root=nullptr) {return rider_update_solve_core(player,ahead_root,nullptr);}
bool rider_tick_mode() {return rig_coherent && mounted_parity && rider_tick_solve;}
// The published sample whose body is this tick's body (newest such), or nullptr. rider_mutex.
const native_stereo::GripFrame* rider_tick_sample(const float* k) {
  const native_stereo::GripFrame* best=nullptr;
  for(const auto& t:rider_ticks) {
    if(!t.sequence) continue;
    float d=0.f; for(int i=0;i<3;++i) d+=(t.body[i]-k[i])*(t.body[i]-k[i]);
    if(d<0.01f*0.01f && (!best || t.sequence>best->sequence)) best=&t.frame;
  }
  return best;
}
// key: the tick's body (world), or nullptr for "the newest tick" (meshes not rooted on the body). rider_mutex held.
bool rider_tick_get(const game::Pointers& player,const float* key,bool by_gun,RiderSolve& out) {
  native_stereo::GripFrame newest;
  if(!rider_take_sample(newest)) return false;
  const std::uint64_t step=newest.sequence;
  const RiderSolve& now=rider_solve;
  auto same_mount=[&](const RiderTickEntry& e){
    return e.valid && e.solve.owner==player.body && e.solve.mount_id[0]==now.mount_id[0] && e.solve.mount_id[1]==now.mount_id[1] &&
      e.solve.correction==now.correction && e.solve.rest_generation==now.rest_generation && e.solve.rest.valid[0]==now.rest.valid[0] &&
      e.solve.rest.valid[1]==now.rest.valid[1];};
  float k[3];
  if(key) std::memcpy(k,key,sizeof(k));
  else if(rider_tick_last>=0 && same_mount(rider_tick_cache[rider_tick_last])) std::memcpy(k,rider_tick_cache[rider_tick_last].body,sizeof(k));
  else if(!rig_body_read(player.body,k)) return false;
  auto same_body=[&](const float* a){float d=0.f; for(int i=0;i<3;++i) d+=(a[i]-k[i])*(a[i]-k[i]); return d<0.01f*0.01f;};
  const native_stereo::GripFrame* exact=rider_tick_sample(k);
  for(int i=0;i<6;++i) {
    auto& e=rider_tick_cache[i];
    if(!same_mount(e) || !same_body(e.body)) continue;
    if(!(e.unique || e.step==step)) continue;
    // Twentieth step [HEADSET 2026-11-08, 3E5A770E: bare much steadier; armed glitchy, left-arm accessory flickers]. [LIVE RIDER_TRACE_40007875]
    // 351 frames drew two solves: in 327 the switch fell inside the songcord's own draws (gun-made provisional, then refreshed), L spread up to
    // 50 cm. As the Cyberpunk port's batch replay: once a draw has used a tick's solve it stays bit-identical for that tick; only a solve no
    // draw has used yet (the gun hook's) is refreshed.
    if(e.provisional && exact && !by_gun && e.drawn) rider_tick_frozen.fetch_add(1,std::memory_order_relaxed);
    if(e.provisional && exact && !by_gun && !e.drawn) {  // the tick's own sample is out now: solve it once more, exactly
      const native_stereo::GripFrame sample=*exact;
      if(rider_update_solve_core(player,nullptr,k,&sample)) {
        e.solve=rider_solve; e.solve.tick_by_gun=e.by_gun; e.solve.tick_kind=2; e.provisional=false;
        rider_tick_refreshed.fetch_add(1,std::memory_order_relaxed);
      }
    }
    if(!by_gun) e.drawn=true;
    out=e.solve; rider_tick_hits.fetch_add(1,std::memory_order_relaxed);
    if(!key) rider_tick_newest.fetch_add(1,std::memory_order_relaxed);
    if(e.by_gun && !by_gun) rider_tick_gun_shared.fetch_add(1,std::memory_order_relaxed);
    return true;
  }
  native_stereo::GripFrame sample; if(exact) sample=*exact;
  if(!rider_update_solve_core(player,nullptr,k,exact?&sample:nullptr)) return false;
  bool unique=true;
  for(auto& e:rider_tick_cache) if(e.valid && same_body(e.body)) {unique=false; e.valid=false;}  // a standing mount: one entry per body
  if(!unique) rider_tick_stationary.fetch_add(1,std::memory_order_relaxed);
  const unsigned slot=rider_tick_slot++%6;
  auto& e=rider_tick_cache[slot];
  e.valid=true; e.unique=unique; e.by_gun=by_gun; e.provisional=!exact; e.drawn=!by_gun; std::memcpy(e.body,k,sizeof(k)); e.step=step;
  e.solve=rider_solve; e.solve.tick_by_gun=by_gun; e.solve.tick_kind=exact?0:1;
  (exact?rider_tick_exact:rider_tick_provisional).fetch_add(1,std::memory_order_relaxed);
  rider_tick_last=static_cast<int>(slot);
  rider_tick_made.fetch_add(1,std::memory_order_relaxed); if(by_gun) rider_tick_gun_made.fetch_add(1,std::memory_order_relaxed);
  out=e.solve; return true;
}
bool mounted_gun_mode() {
  return mounted_weapon_enabled.load(std::memory_order_relaxed) && mount_camera::rider_view() &&
    mount_camera::mounted_arms() && mount_state::get().phase==mount_state::Phase::Riding &&
    controller_mode.load() && pulse_permission.load();
}
bool mounted_gun_identity(const MountedGunFrame& a,const MountedGunFrame& b) {
  return mounted_weapon_math::same_identity(a,b);
}
bool mounted_gun_frame(MountedGunFrame& out) {
  if(!mounted_gun_mode()) return false;
  const auto p=game::pointers(); const auto state=mount_state::get();
  if(!p.body || !read(p.body,0x250,&out.candidate,sizeof(out.candidate)) || !out.candidate) return false;
  out.owner=p.body; out.partner=state.partner; out.id[0]=state.partner_id[0]; out.id[1]=state.partner_id[1];
  {
    std::lock_guard<std::mutex> lock(rider_mutex);
    static RiderSolve gun_solve;  // rider_mutex
    if(rider_tick_mode()) {
      // The entity hook runs in the game thread for the tick being simulated (t_rig_body_live: its live body); its draws come later
      // with that body as their root, so they find this solve. Elsewhere: the newest tick.
      float live[3]; const bool have_live=t_rig_body_live && rig_body_read(p.body,live);
      if(!rider_tick_get(p,have_live?live:nullptr,have_live,gun_solve)) return false;
    } else {
      if(!rider_update_solve(p)) return false;
      gun_solve=rider_solve;
    }
    const auto& r=gun_solve;
    out.sequence=r.input.sequence; std::memcpy(out.body,r.body,sizeof(out.body));
    for(unsigned h=0;h<2;++h) {
      std::memcpy(out.flat[h],r.weapon_socket[h],64);
      skin_arm_ik::multiply_affine(out.target[h],r.weapon_socket[h],r.arm[h].delta[rider_ik::kHand]);
      float anatomical[16],wrist[16];
      const auto& local=r.rest.local[h];
      const bool palm_ok=mounted_weapon_math::palm(local[rider_ik::kRHand],skin_arm_ik::position(local[rider_ik::kRMiddle]),
        skin_arm_ik::position(local[rider_ik::kRIndex]),skin_arm_ik::position(local[rider_ik::kRPinky]),anatomical) &&
        mounted_weapon_math::relative(anatomical,local[rider_ik::kRHand],out.palm_local[h]);
      skin_arm_ik::multiply_affine(wrist,r.joints[h][rider_ik::kHand],r.arm[h].delta[rider_ik::kHand]);
      skin_arm_ik::multiply_affine(out.palm[h],out.palm_local[h],wrist);
      for(int k=0;k<3;++k) {const float o=1024.f*(r.cell[k]-ride_origin_cell[k]); out.flat[h][12+k]+=o; out.target[h][12+k]+=o; out.palm[h][12+k]+=o;}
      // A free hand target is its solved anatomical palm. An unused native TP
      // attachment can be invalid without invalidating the other hand's gun.
      // Native frame matches remain mandatory at the ownership boundary below.
      out.valid[h]=palm_ok && r.ok[h] && r.weapon_ok[h] && r.input.valid[h] && r.rest.valid[h] && mounted_weapon_math::rigid(out.palm[h]);
      out.why[h]=static_cast<unsigned char>((palm_ok?0:1)|(r.ok[h]?0:2)|(r.weapon_ok[h]?0:4)|(r.input.valid[h]?0:8)|(r.rest.valid[h]?0:16)|(mounted_weapon_math::rigid(out.palm[h])?0:32));
    }
  }
  const auto after=mount_state::get(); const void* candidate=nullptr;
  return game::pointers().body==p.body && after.partner==out.partner && after.partner_id[0]==out.id[0] &&
    after.partner_id[1]==out.id[1] && read(p.body,0x250,&candidate,sizeof(candidate)) && candidate==out.candidate;
}
bool mounted_gun_palette(void* mesh,const void* root,const void* bones,const void* remap,const void* cells,bool& handled) {
  handled=false;
  if(!mounted_gun_mode()) return false;
  // Reject distant world/NPC palettes before decoding paths. This is only a cost filter, never ownership proof.
  float root_world[16]{},body[3]{}; short root_cell[3]{};
  const auto player=game::pointers();
  if(!read(root,0,root_world,sizeof(root_world)) || !read(cells,0,root_cell,sizeof(root_cell)) ||
     !rig_body_read(player.body,body)) return false;
  float distance2=0.f;
  for(int k=0;k<3;++k) {const float d=root_world[12+k]+1024.f*root_cell[k]-body[k]; distance2+=d*d;}
  if(!std::isfinite(distance2) || distance2>9.f) return false;
  // Only cache paths on this hook thread. Resource identity, not transient root/stack pointers, owns the cache.
  struct Path {const void* resource=nullptr; bool weapon=false;};
  static thread_local Path paths[64]; static thread_local unsigned replace=0;
  const void* resource=nullptr; if(!read(root,0x58,&resource,sizeof(resource)) || !resource) return false;
  Path* path=nullptr; for(auto& q:paths) if(q.resource==resource) {path=&q; break;}
  if(!path) {
    char text[128]{}; const bool weapon=read_path(root,text) && std::strstr(text,"/WEAPON/PLAYER/");
    path=&paths[replace++%64]; *path={resource,weapon};
  }
  if(!path->weapon) return false;
  handled=true;
  unsigned char header[16]{}; const void* source=nullptr; unsigned count=0; short cell[3]{};
  static thread_local float mats[kMaxGearBones][16];
  if(!read(bones,0,header,sizeof(header)) || !read(cells,0,cell,sizeof(cell))) return original(mesh,root,bones,remap,cells);
  std::memcpy(&source,header,sizeof(source)); std::memcpy(&count,header+8,sizeof(count));
  if(!count || count>kMaxGearBones || !read(source,0,mats,static_cast<std::size_t>(count)*64)) return original(mesh,root,bones,remap,cells);
  float native[16]; std::memcpy(native,mats[0],64); for(int k=0;k<3;++k) native[12+k]+=1024.f*(cell[k]-ride_origin_cell[k]);
  MountedGunFrame frame;
  if(!mounted_gun_frame(frame)) {mounted_palette_no_frame.fetch_add(1,std::memory_order_relaxed); return original(mesh,root,bones,remap,cells);}
  struct BoundPart {const void* mesh=nullptr; const void* resource=nullptr; mounted_weapon_math::Identity identity; float root_in_socket[16]{}; bool valid=false;};
  static thread_local BoundPart parts[64]; static thread_local unsigned replace_part=0;
  BoundPart* part=nullptr;
  for(auto& q:parts) if(q.mesh==mesh && q.resource==resource && mounted_weapon_math::same_identity(q.identity,frame)) {part=&q; break;}
  int from=-1; for(int h=0;h<2;++h) if(mounted_weapon_math::same_frame(native,frame.flat[h])) {from=h; break;}
  if(from<0 && !part) {
    {
      std::lock_guard<std::mutex> lock(mounted_gun_mutex);
      const auto now=GetTickCount64(); float offset[16];
      if(mounted_gun_identity(frame,mounted_gun_proof.identity) && mounted_gun_proof.resource==resource &&
          mounted_gun_proof.have_placed && mounted_weapon_math::fresh(now,mounted_gun_proof.at) &&
          mounted_weapon_math::fresh(now,mounted_gun_proof.committed_at) &&
          mounted_gun_proof.render_history.entity_offset(native,now,offset)) {
        part=&parts[replace_part++%64]; *part=BoundPart{};
        part->mesh=mesh; part->resource=resource; part->identity=frame; part->valid=true;
        std::memcpy(part->root_in_socket,offset,64); mounted_gun_proof.at=now;
        mounted_palette_adopted.fetch_add(1,std::memory_order_relaxed);
      }
    }
    // An already-placed draw can be one successful commit behind. Continue
    // through the same private rebase as cached parts; do not draw its old root.
    if(!part) {
      mounted_palette_unbound.fetch_add(1,std::memory_order_relaxed);
      return original(mesh,root,bones,remap,cells);
    }
  }
  bool entity_ready=false; float committed_target[16]{},committed_body[3]{},committed_free[16]{}; bool root_space=false;
  MountedProfile profile; bool support_valid=false; std::string profile_key,observed_path;
  {char text[128]{}; if(read_path(root,text)) {
    observed_path=text;
    std::lock_guard<std::mutex> lock(mounted_profile_mutex);
    const auto it=mounted_support_profiles.find(text);
    if(it!=mounted_support_profiles.end()) {profile=it->second; support_valid=profile.valid; profile_key=text;}
  }}
  const unsigned hand=left_handed?0u:1u;
  {
    std::lock_guard<std::mutex> lock(mounted_gun_mutex);
    if(!mounted_gun_identity(frame,mounted_gun_proof.identity) ||
       (from>=0 && support_valid && !mounted_gun_proof.profile_path.empty() && mounted_gun_proof.profile_path!=profile_key)) {
      std::lock_guard<std::mutex> state_lock(clean_mutex);
      two_hand=TwoHandState{}; set_support_only_locked(false,clean_qpc_us()); th_measured_us=0;
      mounted_gun_proof=MountedGunProof{};
    }
    if(from>=0) {
      mounted_gun_proof.identity=frame; mounted_gun_proof.resource=resource;
      mounted_gun_proof.bound_path=observed_path;
      mounted_gun_proof.source=static_cast<unsigned>(from); mounted_gun_proof.at=GetTickCount64();
      if(!part) {
        part=&parts[replace_part++%64]; *part=BoundPart{};
        part->mesh=mesh; part->resource=resource; part->identity=frame;
        part->valid=mounted_weapon_math::relative(native,frame.flat[from],part->root_in_socket);
      }
    }
    if(!mounted_gun_proof.hold_valid && frame.valid[0]) {
      float hold[16];
      if(mounted_weapon_math::relative(frame.palm[0],frame.target[0],hold) &&
         skin_arm_ik::length(skin_arm_ik::position(hold))<.6f) {
        std::memcpy(mounted_gun_proof.hold[0],hold,64);
        mounted_weapon_math::mirror_palm_hold(hold,mounted_gun_proof.hold[1]);
        mounted_gun_proof.hold_valid=true;
      }
    }
    if(support_valid) {
      mounted_gun_proof.support_valid=true; std::memcpy(mounted_gun_proof.support_relative,profile.relative,64);
      mounted_gun_proof.profile=profile;
      if(!mounted_gun_proof.onfoot_hold && part && part->valid) {
        // The validated on-foot reference is already in the actual mesh root.
        // Native TP socket/animation phase is ownership evidence, not a new grip offset.
        float hold[16]; std::memcpy(hold,profile.trigger_root,64);
        if(mounted_weapon_math::rigid(hold) && skin_arm_ik::length(skin_arm_ik::position(hold))<.6f) {
          std::memcpy(mounted_gun_proof.hold[1],hold,64);
          mounted_weapon_math::mirror_palm_hold(hold,mounted_gun_proof.hold[0]);
          mounted_gun_proof.hold_valid=true; mounted_gun_proof.onfoot_hold=true;
          mounted_gun_proof.render_history=mounted_weapon_math::PlacedFrames{};
          mounted_gun_proof.profile_resource=resource;
          mounted_gun_proof.profile_path=profile_key;
        }
      }
    }
    const bool primary_root=mounted_gun_proof.onfoot_hold && !profile_key.empty() && profile_key==mounted_gun_proof.profile_path;
    if(from<0 && part && part->valid && mounted_gun_proof.have_placed &&
       mounted_weapon_math::fresh(GetTickCount64(),mounted_gun_proof.committed_at) &&
       mounted_gun_proof.render_history.matches(native,part->root_in_socket,primary_root,GetTickCount64()))
      mounted_gun_proof.at=GetTickCount64(); // still owned: a native source already following our committed root
    entity_ready=mounted_gun_proof.have_placed && mounted_gun_proof.hand==hand &&
      mounted_weapon_math::fresh(GetTickCount64(),mounted_gun_proof.at) &&
      mounted_weapon_math::fresh(GetTickCount64(),mounted_gun_proof.committed_at);
    if(entity_ready) {std::memcpy(committed_target,mounted_gun_proof.target,64); std::memcpy(committed_body,mounted_gun_proof.committed_body,12); std::memcpy(committed_free,mounted_gun_proof.committed_free,64);}
    root_space=mounted_gun_proof.onfoot_hold && !profile_key.empty() && profile_key==mounted_gun_proof.profile_path;
  }
  if(!entity_ready || !part || !part->valid || mount_camera::harness_held(hand) || mount_camera::arm_ik_weight(hand)<.999f) {
    mounted_palette_not_ready.fetch_add(1,std::memory_order_relaxed);
    return original(mesh,root,bones,remap,cells);
  }
  float root_target[16];
  if(root_space) std::memcpy(root_target,committed_target,64);
  else skin_arm_ik::multiply_affine(root_target,part->root_in_socket,committed_target);
  const bool stable=rider_tune_now().weapon_stable;
  if(stable && rig_coherent && mounted_parity) {
    // [SOURCE] the entity, and every part the game draws from it, sits at the commit; these palettes went to the live palm, so the two
    // groups parted by the palm's change since the commit (the detached muzzle ring: INFERENCE). On foot every gun part is the entity's.
    // Measured: how far the old follow would have moved this root.
    float followed[16]; std::memcpy(followed,root_target,64);
    // As the rider meshes (second step): a gun palette rooted on the body takes its root as the body of its own tick, so it stays with the
    // hand drawn at that tick. Whether weapon palettes are rooted there is counted (OPEN); unrooted ones keep the solve's body.
    float gun_body[3]; std::memcpy(gun_body,frame.body,sizeof(gun_body));
    {float root_pos[3],gap=0.f,to_solve=0.f;
      for(int k=0;k<3;++k) {root_pos[k]=root_world[12+k]+1024.f*root_cell[k]; gap+=(root_pos[k]-body[k])*(root_pos[k]-body[k]);
        to_solve+=(root_pos[k]-frame.body[k])*(root_pos[k]-frame.body[k]);}
      const bool gun_rooted=gap<0.02f*0.02f && to_solve<3.f*3.f;
      if(gun_rooted) {std::memcpy(gun_body,root_pos,sizeof(gun_body)); rider_gun_rooted.fetch_add(1,std::memory_order_relaxed);}
      else rider_gun_unrooted.fetch_add(1,std::memory_order_relaxed);
      rider_gun_follows_mesh.store(gun_rooted,std::memory_order_relaxed);}
    // Nineteenth step: per-tick solve. The commit was made from a provisional sample; the draw's tick has the exact hand. The gun follows
    // that hand (a small correction: the camera's own motion within one step) instead of the hands following the gun.
    bool hand_led=false;
    if(rider_tick_mode() && frame.valid[hand] && mounted_weapon_math::follow_palm(followed,committed_free,frame.palm[hand])) {
      const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(followed),skin_arm_ik::position(root_target)));
      float carried[16]; std::memcpy(carried,root_target,64); mounted_weapon_math::carry(carried,committed_body,gun_body);
      const float dc=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(followed),skin_arm_ik::position(carried)));
      if(std::isfinite(dc) && dc<0.15f) {std::memcpy(root_target,followed,64); hand_led=true; rider_gun_hand_led.fetch_add(1,std::memory_order_relaxed);}
      else rider_gun_hand_far.fetch_add(1,std::memory_order_relaxed);
      (void)d; std::memcpy(followed,root_target,64);
    }
    if(!hand_led) mounted_weapon_math::carry(root_target,committed_body,gun_body);
    if(frame.valid[hand] && mounted_weapon_math::follow_palm(followed,committed_free,frame.palm[hand])) {
      const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(followed),skin_arm_ik::position(root_target)));
      const unsigned mm=std::isfinite(d)?static_cast<unsigned>(std::min(d,10.f)*1000.f):0u;
      unsigned seen=rider_palm_commit_mm.load(std::memory_order_relaxed);
      while(mm>seen && !rider_palm_commit_mm.compare_exchange_weak(seen,mm,std::memory_order_relaxed)) {}
    }
  } else if(stable && !(frame.valid[hand] && mounted_weapon_math::follow_palm(root_target,committed_free,frame.palm[hand])))
    mounted_weapon_math::carry(root_target,committed_body,body);
  // Cancel native holding sway at this part's own root, retaining internal gun
  // animation and its once-proven socket relation. All parts use one commit.
  for(unsigned i=0;i<count;++i) {
    float world[16],next[16]; std::memcpy(world,mats[i],64);
    for(int k=0;k<3;++k) world[12+k]+=1024.f*(cell[k]-ride_origin_cell[k]);
    if(!(stable?mounted_weapon_math::map_part(world,native,root_target,next):mounted_weapon_math::map(world,native,root_target,next))) {
      mounted_palette_invalid.fetch_add(1,std::memory_order_relaxed);
      return original(mesh,root,bones,remap,cells);
    }
    for(int k=0;k<3;++k) next[12+k]-=1024.f*(cell[k]-ride_origin_cell[k]); std::memcpy(mats[i],next,64);
  }
  const void* private_source=mats; std::memcpy(header,&private_source,sizeof(private_source));
  return original(mesh,root,header,remap,cells);
}
// Harness frame census (2026-10-24 [LIVE]: the palms in the rider's pelvis frame spread 0.44-0.67 m with the hold
// animation on, so the pelvis is not the harness frame). While riding, for every non-player mesh within 5 m: the game's
// palms in that mesh's root frame and every bone's mean distance to each palm. The mount/harness mesh is the one in
// which the held palms barely move; its nearest bone is the harness handle. Read-only; reported at the end of a ride.
struct FrameCensus {
  std::string path; unsigned n=0; ULONGLONG next=0;
  rider_ik::GripStat palm[2];
  std::vector<double> distance[2];
  std::vector<std::string> names;
  unsigned samples=0;
};
std::mutex frame_census_mutex;
std::vector<FrameCensus> frame_census;  // frame_census_mutex
void rider_frame_sample(const std::string& path,const void* root,const float (*mats)[16],unsigned n,const float* root_matrix,
                        const skin_arm_ik::Vec (&palms)[2]) {
  const ULONGLONG now=GetTickCount64();
  std::lock_guard<std::mutex> lock(frame_census_mutex);
  FrameCensus* f=nullptr;
  for(auto& c:frame_census) if(c.path==path && c.n==n) {f=&c; break;}
  if(!f) {
    if(frame_census.size()>=48) return;
    frame_census.emplace_back(); f=&frame_census.back();
    f->path=path; f->n=n; f->distance[0].assign(n,0.0); f->distance[1].assign(n,0.0); f->names.resize(n);
    for(unsigned i=0;i<n;++i) {Sample entry{}; if(read_name(root,i,entry) && entry.name_count==n) f->names[i]=entry.bone_name;}
  }
  if(now<f->next) return;
  f->next=now+50;
  for(int h=0;h<2;++h) {
    const float world[3]={palms[h].x,palms[h].y,palms[h].z};
    float local[3];
    if(!rider_ik::to_pelvis(root_matrix,world,local)) return;
    const float zero[3]{};
    f->palm[h].add(local,zero,zero);
    for(unsigned i=0;i<n;++i) f->distance[h][i]+=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[i]),palms[h]));
  }
  ++f->samples;
}
bool mounted_gun_fingers(void* mesh,RiderMesh& info,const RiderSolve& solve,const float (*source)[16],
                         float (*mats)[16],unsigned n,unsigned h,const MountedGunProof& proof) {
  using namespace skin_arm_ik;
  if(!proof.profile.valid || info.hand_node.size()!=n || !solve.rest_generation) return true;
  struct Bind {
    const void* resource=nullptr; const void* owner=nullptr; std::uint64_t mount[2]{}; unsigned generation=0;
    std::vector<float> reference; std::vector<short> anchor; float palm[2][16]{}; bool valid[2]{};
  };
  static std::vector<Bind> binds; // rider_mutex; resource + owner/mount/rest, independent of transient draw meshes
  auto matches=[&](const Bind& b) {return b.resource==info.resource && b.owner==solve.owner &&
    b.mount[0]==solve.mount_id[0] && b.mount[1]==solve.mount_id[1] && b.generation==solve.rest_generation && b.reference.size()==n*16u;};
  auto restore=[&](const Bind& b) {
    info.hand_reference=b.reference; info.hand_anchor=b.anchor;
    std::memcpy(info.hand_palm,b.palm,sizeof(b.palm)); std::memcpy(info.hand_valid,b.valid,sizeof(b.valid));
    info.hand_generation=b.generation;
  };
  if(info.hand_generation!=solve.rest_generation) {
    std::lock_guard<std::mutex> lock(rider_mutex);
    for(const auto& b:binds) if(matches(b)) {restore(b); break;}
  }
  if(info.hand_generation!=solve.rest_generation) {
    info.hand_valid[0]=info.hand_valid[1]=false; info.hand_reference.assign(n*16u,0.f);
    info.hand_anchor.assign(n,-1);
    for(unsigned side=0;side<2;++side) {
      const short* j=info.joint[side];
      if(j[3]<0 || j[4]<0 || j[5]<0 || j[6]<0) continue;
      float palm[16];
      if(!mounted_weapon_math::palm(source[j[3]],position(source[j[5]]),position(source[j[4]]),position(source[j[6]]),palm) ||
         !mounted_weapon_math::relative(palm,source[j[3]],info.hand_palm[side])) continue;
      int core[mounted_hand_pose::count]; std::fill(std::begin(core),std::end(core),-1);
      bool valid=true;
      for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(side) && info.hand_family[i]>=0) {
        if(!mounted_weapon_math::relative(source[i],palm,info.hand_reference.data()+i*16)) {valid=false; break;}
        if(info.hand_node[i]>=0) core[info.hand_node[i]]=static_cast<int>(i);
      }
      for(int index:core) if(index<0) valid=false;
      if(!valid) continue;
      for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(side) && info.hand_family[i]>=0 && info.hand_node[i]<0) {
        float best=1e9f;
        for(int node=0;node<mounted_hand_pose::count;++node) if(mounted_hand_pose::family(node)==info.hand_family[i]) {
          const float d=length(sub(position(source[i]),position(source[core[node]])));
          if(d<best) {best=d; info.hand_anchor[i]=static_cast<short>(core[node]);}
        }
      }
      info.hand_valid[side]=true;
    }
    info.hand_generation=solve.rest_generation;
    {
      std::lock_guard<std::mutex> lock(rider_mutex);
      bool existing=false; for(const auto& b:binds) if(matches(b)) {restore(b); existing=true; break;}
      if(!existing) {
        if(binds.size()>=32) binds.erase(binds.begin());
        Bind b; b.resource=info.resource; b.owner=solve.owner; b.mount[0]=solve.mount_id[0]; b.mount[1]=solve.mount_id[1];
        b.generation=solve.rest_generation; b.reference=info.hand_reference; b.anchor=info.hand_anchor;
        std::memcpy(b.palm,info.hand_palm,sizeof(b.palm)); std::memcpy(b.valid,info.hand_valid,sizeof(b.valid)); binds.push_back(std::move(b));
      }
      rider_meshes[mesh]=info;
    }
    rider_note("mounted hand retarget coverage "+info.path+" valid="+std::to_string(info.hand_valid[0])+","+
      std::to_string(info.hand_valid[1])+" generation="+std::to_string(solve.rest_generation));
  }
  // Keep lightweight mesh metadata warm too; the numeric bind above belongs to the resource.
  if(info.hand_generation==solve.rest_generation) {
    std::lock_guard<std::mutex> lock(rider_mutex);
    auto it=rider_meshes.find(mesh);
    if(it!=rider_meshes.end() && it->second.hand_generation!=info.hand_generation) it->second=info;
  }
  if(!info.hand_valid[h] || !(proof.pin[h]>0.f)) return true;
  const int native_hand=proof.hand==h?1:0; // on-foot trigger right / support left, independent of equipped side
  if(!proof.profile.fingers[native_hand].valid) return true;
  float reference[mounted_hand_pose::count][16],retargeted[mounted_hand_pose::count][16];
  for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.hand_node[i]>=0)
    std::memcpy(reference[info.hand_node[i]],info.hand_reference.data()+i*16,64);
  if(!mounted_hand_pose::retarget(reference,proof.profile.fingers[native_hand],native_hand!=static_cast<int>(h),retargeted)) return false;
  const int wrist=info.joint[h][3]; float palm[16];
  multiply_affine(palm,info.hand_palm[h],mats[wrist]);
  static thread_local float next[kMaxGearBones][16];
  // Complete private hand update, including thumb/finger deformation helpers.
  // Decorations without finger ownership retain the accessory fork's attachment path.
  for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.hand_family[i]>=0) {
    const int node=info.hand_node[i],anchor=info.hand_anchor[i]; float local[16],target[16];
    if(node>=0) std::memcpy(local,retargeted[node],64);
    else {
      if(anchor<0 || info.hand_node[anchor]<0) return false;
      float inverse[16],delta[16];
      if(!inverse_affine(inverse,info.hand_reference.data()+anchor*16)) return false;
      multiply_affine(delta,inverse,retargeted[info.hand_node[anchor]]);
      multiply_affine(local,info.hand_reference.data()+i*16,delta);
    }
    multiply_affine(target,local,palm);
    vr_arms_math::blend_affine(mats[i],target,proof.pin[h],next[i]);
    if(!rider_ik::preserves_geometry(source[i],next[i])) return false;
  }
  for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.hand_family[i]>=0) std::memcpy(mats[i],next[i],64);
  mounted_finger_apps.fetch_add(1,std::memory_order_relaxed);
  return true;
}
// Pin only the render copy. The canonical free solve remains controller-driven
// and is the input to weapon placement, avoiding a weapon/hand feedback loop.
void mounted_gun_pins(RiderSolve& solve,float (&wrist_world)[2][16],bool (&pinned)[2],MountedGunProof& draw_proof) {
  if(!mounted_gun_mode() || swap_to.load()>=0) return;
  MountedGunProof proof;
  {std::lock_guard<std::mutex> lock(mounted_gun_mutex); proof=mounted_gun_proof;}
  const auto now=GetTickCount64();
  if(!proof.have_placed) return;
  if(!mounted_weapon_math::fresh(now,proof.committed_at)) {mounted_pin_expired.fetch_add(1,std::memory_order_relaxed); return;}
  if(proof.identity.owner!=solve.owner || proof.identity.partner!=solve.partner ||
     proof.identity.id[0]!=solve.mount_id[0] || proof.identity.id[1]!=solve.mount_id[1]) return;
  const void* candidate=nullptr;
  if(!read(solve.owner,0x250,&candidate,sizeof(candidate)) || candidate!=proof.identity.candidate) return;
  draw_proof=proof;
  const auto tune=rider_tune_now();
  if(tune.weapon_stable && rig_coherent && mounted_parity) {
    // On foot the gun hand is pinned to the written socket, the placement every gun part is drawn from. Riding the same: the held palms of
    // the commit, carried only by the rider's travel since it (none within its own tick). The free palm no longer moves them.
    // Nineteenth step: per-tick solve, the held palms follow this tick's exact gun hand (as the gun palette does); far: carried as before.
    bool led=false;
    if(rider_tick_mode()) {
      const unsigned d=proof.hand<2?proof.hand:0u; const auto& rest=solve.rest.local[d];
      float rest_palm[16],local_palm[16],wrist[16],live[16];
      if(solve.ok[d] && solve.rest.valid[d] &&
         mounted_weapon_math::palm(rest[rider_ik::kRHand],skin_arm_ik::position(rest[rider_ik::kRMiddle]),
           skin_arm_ik::position(rest[rider_ik::kRIndex]),skin_arm_ik::position(rest[rider_ik::kRPinky]),rest_palm) &&
         mounted_weapon_math::relative(rest_palm,rest[rider_ik::kRHand],local_palm)) {
        skin_arm_ik::multiply_affine(wrist,solve.joints[d][rider_ik::kHand],solve.arm[d].delta[rider_ik::kHand]);
        skin_arm_ik::multiply_affine(live,local_palm,wrist);
        for(int k=0;k<3;++k) live[12+k]+=1024.f*(solve.cell[k]-ride_origin_cell[k]);
        float moved[2][16]; std::memcpy(moved,proof.held_palm,sizeof(moved)); bool all=true;
        for(int h=0;h<2;++h) {
          float carried[16]; std::memcpy(carried,proof.held_palm[h],64); mounted_weapon_math::carry(carried,proof.committed_body,solve.body);
          all=all && mounted_weapon_math::follow_palm(moved[h],proof.committed_free,live) &&
            skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(moved[h]),skin_arm_ik::position(carried)))<0.15f;
        }
        if(all) {std::memcpy(proof.held_palm,moved,sizeof(moved)); led=true;}
      }
    }
    if(!led) for(auto& held:proof.held_palm) mounted_weapon_math::carry(held,proof.committed_body,solve.body);
  } else if(tune.weapon_stable) {
    // Same live palm as the empty hand: one-hand pins equal the free solve, two-hand keeps the committed swing.
    const unsigned d=proof.hand<2?proof.hand:0u; const auto& rest=solve.rest.local[d];
    float rest_palm[16],local_palm[16],wrist[16],live[16]; float body[3];
    const bool ok=solve.ok[d] && solve.rest.valid[d] &&
      mounted_weapon_math::palm(rest[rider_ik::kRHand],skin_arm_ik::position(rest[rider_ik::kRMiddle]),
        skin_arm_ik::position(rest[rider_ik::kRIndex]),skin_arm_ik::position(rest[rider_ik::kRPinky]),rest_palm) &&
      mounted_weapon_math::relative(rest_palm,rest[rider_ik::kRHand],local_palm);
    if(ok) {
      skin_arm_ik::multiply_affine(wrist,solve.joints[d][rider_ik::kHand],solve.arm[d].delta[rider_ik::kHand]);
      skin_arm_ik::multiply_affine(live,local_palm,wrist);
      for(int k=0;k<3;++k) live[12+k]+=1024.f*(solve.cell[k]-ride_origin_cell[k]);
    }
    const bool have_body=rig_body_read(solve.owner,body);
    for(auto& held:proof.held_palm) if(!(ok && mounted_weapon_math::follow_palm(held,proof.committed_free,live)) && have_body)
      mounted_weapon_math::carry(held,proof.committed_body,body);
  }
  for(unsigned h=0;h<2;++h) {
    if(!(proof.pin[h]>0.f) || !solve.ok[h] || !solve.rest.valid[h] || mount_camera::harness_held(h) || mount_camera::arm_ik_weight(h)<.999f) continue;
    const auto& local=solve.rest.local[h];
    float palm[16],palm_local[16],inverse[16],desired[16],free[16],mixed[16];
    if(!mounted_weapon_math::palm(local[rider_ik::kRHand],skin_arm_ik::position(local[rider_ik::kRMiddle]),
        skin_arm_ik::position(local[rider_ik::kRIndex]),skin_arm_ik::position(local[rider_ik::kRPinky]),palm) ||
       !mounted_weapon_math::relative(palm,local[rider_ik::kRHand],palm_local) || !skin_arm_ik::inverse_affine(inverse,palm_local)) continue;
    skin_arm_ik::multiply_affine(desired,inverse,proof.held_palm[h]);
    skin_arm_ik::multiply_affine(free,solve.joints[h][rider_ik::kHand],solve.arm[h].delta[rider_ik::kHand]);
    for(int k=0;k<3;++k) free[12+k]+=1024.f*(solve.cell[k]-ride_origin_cell[k]);
    // Never stretch an arm to an obsolete gun during boost/holster transitions.
    if(!mounted_weapon_math::rigid(desired) || skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(free),skin_arm_ik::position(desired)))>.35f) {
      mounted_pin_distant.fetch_add(1,std::memory_order_relaxed); continue;
    }
    vr_arms_math::blend_affine(free,desired,proof.pin[h],mixed);
    std::memcpy(wrist_world[h],mixed,64);
    float view[16],grip[16],wrist[16];
    std::memcpy(view,solve.input.camera,64); std::memcpy(grip,solve.input.grip[h],64); std::memcpy(wrist,mixed,64);
    for(int k=0;k<3;++k) {
      const float shift=solve.body[k]-solve.input.body[k]-1024.f*solve.cell[k];
      view[12+k]+=shift; grip[12+k]+=shift; wrist[12+k]-=1024.f*(solve.cell[k]-ride_origin_cell[k]);
    }
    view[3]=view[7]=view[11]=0.f; view[15]=1.f;
    const float* native[4]={solve.joints[h][0],solve.joints[h][1],solve.joints[h][2],solve.joints[h][3]};
    rider_ik::Arm arm;
    if(!rider_ik::solve_rest(solve.rest,h,view,native,grip,solve.input.camera,tune,arm,nullptr,wrist)) continue;
    if(solve.twist_valid[h]) arm.twist=rider_ik::continuous_twist(arm.twist,solve.twist_reference[h]);
    solve.arm[h]=arm; pinned[h]=true;
    rider_ik::capture_attachment(solve.attachment[h],native,arm,
      {1024.f*solve.cell[0],1024.f*solve.cell[1],1024.f*solve.cell[2]},
      {solve.body[0],solve.body[1],solve.body[2]});
  }
}
bool rider_call(void* mesh,const void* root,const void* bones,const void* remap,const void* cells) {
  const auto player=game::pointers();
  float root_matrix[16]{},body_position[3]{}; std::int16_t cell[3]{};
  if(!player.body || !read(root,0,root_matrix,sizeof(root_matrix)) || !read(cells,0,cell,sizeof(cell)) ||
     !rig_body_read(player.body,body_position)) return original(mesh,root,bones,remap,cells);
  float distance=0.f;
  for(unsigned k=0;k<3;++k){const float d=root_matrix[12+k]+1024.f*cell[k]-body_position[k]; distance+=d*d;}
  if(!(distance<25.f)) return original(mesh,root,bones,remap,cells);  // the local player's, within 5 m
  unsigned char header[16]{}; const void* source=nullptr; unsigned n=0;
  if(!read(bones,0,header,sizeof(header))) return original(mesh,root,bones,remap,cells);
  std::memcpy(&source,header,sizeof(source)); std::memcpy(&n,header+8,sizeof(n));
  static thread_local float mats[kMaxGearBones][16];
  if(!source || n==0 || n>kMaxGearBones || !read(source,0,mats,static_cast<std::size_t>(n)*64))
    return original(mesh,root,bones,remap,cells);
  const bool arms_wanted=mount_camera::mounted_arms() && controller_mode.load();
  const bool correction=rider_tune_now().correction;
  const void* resource=nullptr; read(root,0x58,&resource,sizeof(resource));
  RiderMesh info; bool known=false, solved=false;
  static thread_local RiderSolve solve;  // copy for this build
  {
    std::lock_guard<std::mutex> lock(rider_mutex);
    auto it=rider_meshes.find(mesh);
    if(it!=rider_meshes.end() && it->second.role.size()==n && it->second.correction==correction && it->second.resource==resource) {info=it->second; known=true;}
    const ULONGLONG now_ms=GetTickCount64();
    if(rider_solve.last_call && now_ms-rider_solve.last_call>1500) rider_solve.rest=rider_ik::Rest{};
    rider_solve.last_call=now_ms;
    // mounted_parity: a player mesh rooted on the body passes its root (world); see the thirteenth step in rider_update_solve.
    float root_world[3]; for(int k=0;k<3;++k) root_world[k]=root_matrix[12+k]+1024.f*cell[k];
    const bool ahead_candidate=rig_coherent && mounted_parity && known && info.rooted && info.player;
    if(arms_wanted && rider_tick_mode()) {if(rider_tick_get(player,ahead_candidate?root_world:nullptr,false,solve)) solved=true;}
    else if(arms_wanted && rider_update_solve(player,ahead_candidate?root_world:nullptr)) {solve=rider_solve; solved=true;}
  }
  // [SOURCE] a mesh with its own arm chain is solved at the solve's body (moment = solve.body - input.body), while gear and chainless meshes
  // were carried to the body read at this draw: a camera step between the two parted bracelets and armour from the arm by a tick of flight.
  // On foot every mesh is a copy of one solve. Riding now: every mesh at the solve's body.
  // Second step [HEADSET 2026-11-07, DLL B53DCDD4: accessories and ring fixed, arms still disconnect at random while moving]. [LIVE same run]
  // `body_drift_max` up to 536 mm within one Present generation: meshes of two ticks are drawn under one latched body. [LIVE MOUNT_CAMERA
  // 10-26 trace, 13227+ riding draws] every rider mesh root equals the body of its own camera step exactly (0.000 m; the hand utility item
  // aside). On foot each mesh is re-rooted to its own draw. Riding the same: a mesh rooted on the body takes its root as its body, so arm,
  // gear and gun hand move with the tick that mesh is drawn at; the arm stays the input plus that body's travel since the sample.
  const bool rider_parity=solved && rig_coherent && mounted_parity;
  // Family audit (read-only): at every exit below, a player mesh with an arm chain records whether each hand was drawn on the IK arm.
  struct FamilyAudit {
    const RiderMesh& info; const bool& solved; const RiderSolve& solve; const unsigned char* header; const void* source; const float (*mats)[16];
    unsigned n; const float* root_matrix; const std::int16_t* cell; const float* body; const void* resource;
    ~FamilyAudit() {
      if(!info.player || (info.joint[0][3]<0 && info.joint[1][3]<0) || !rig_coherent || !mounted_parity) return;
      const void* drawn=nullptr; std::memcpy(&drawn,header,sizeof(drawn));
      const bool replaced=drawn==static_cast<const void*>(mats);
      RiderTraceRow row{}; row.us=clean_qpc_us(); row.gen=frame_gen.load(std::memory_order_relaxed); row.resource=resource;
      row.solved=solved?1:0; if(solved) row.step_used=solve.input.sequence;
      row.phase=static_cast<unsigned char>(mount_state::get().phase);
      {native_stereo::GripFrame newest; if(native_stereo::grip_frame(&newest)) {row.step_new=newest.sequence; for(int k=0;k<3;++k) row.cam[k]=newest.camera[12+k];}}
      for(int k=0;k<3;++k) {row.root[k]=root_matrix[12+k]+1024.f*cell[k]; row.body[k]=body[k];}
      for(int h=0;h<2;++h) {
        row.weight[h]=static_cast<unsigned char>(std::clamp(mount_camera::arm_ik_weight(h),0.f,1.f)*100.f+.5f);
        if(mount_camera::harness_held(h)) row.held|=static_cast<unsigned char>(1u<<h);
        const int hi=info.joint[h][3]; if(hi<0 || static_cast<unsigned>(hi)>=n) continue;
        float native[16]; if(!read(source,static_cast<std::size_t>(hi)*64,native,64)) continue;
        const float* m=replaced?mats[hi]:native; float d=0.f;
        for(int k=0;k<3;++k) {row.hand[h][k]=m[12+k]+1024.f*cell[k]; const float e=m[12+k]-native[12+k]; d+=e*e;}
        row.have[h]=1; row.ik[h]=(replaced && d>0.001f*0.001f)?1:0;
        if(solved) for(int k=0;k<3;++k) row.target[h][k]=solve.input.grip[h][12+k]+solve.body[k]-solve.input.body[k];
      }
      if(solved) {for(int k=0;k<3;++k) {row.scam[k]=solve.input.camera[12+k]; row.sbody[k]=solve.input.body[k];} row.by_gun=static_cast<unsigned char>((solve.tick_by_gun?1:0)|(solve.tick_kind<<1));}
      rider_trace_add(row,info.path);
    }
  } family_audit{info,solved,solve,header,source,mats,n,root_matrix,cell,body_position,resource};
  bool mesh_rooted=false; float mesh_yaw=0.f;
  // World frame of the solve -> world frame of this mesh's tick: turn about the solve's body, then that body's travel.
  // `centre` is the solve's body in the matrix's own space (world, pose cell or ride-local).
  auto to_mesh=[&](float* m,const float* centre) {
    if(mesh_yaw!=0.f) skin_arm_ik::rotate_about_up(m,centre[0],centre[2],mesh_yaw);
    for(int k=0;k<3;++k) m[12+k]+=body_position[k]-solve.body[k];
  };
  if(rider_parity) {
    std::memcpy(body_position,solve.body,sizeof(body_position));
    float root_world[3]; for(int k=0;k<3;++k) root_world[k]=root_matrix[12+k]+1024.f*cell[k];
    float to_solve=0.f; for(int k=0;k<3;++k) to_solve+=(root_world[k]-solve.body[k])*(root_world[k]-solve.body[k]);
    mesh_rooted=(distance<0.02f*0.02f || (known && info.rooted)) && to_solve<3.f*3.f;
    if(mesh_rooted) {
      // Fourth step [HEADSET 2026-11-08, DLL 8DFDF87E: still stutter and lost tracking while moving and turning]. [LIVE] the turn found
      // here was 0.00 deg in every window, and the travel came in 160/320 mm steps: whole ticks. [SOURCE] on foot every mesh's arms use one
      // body, the newest camera step's (rig_body; fp_root_now puts each root there), not the mesh's own root: the arms follow the view and
      // the controllers. Moving them with each mesh's tick made them jump against the controllers. Measured only now; see rider_frame.
      if(kRiderMeshMoment) std::memcpy(body_position,root_world,sizeof(body_position));
      // Third step [HEADSET 2026-11-07, DLL 6AB51123: very unstable, stutter worst while turning on the mount]. The second step moved a mesh
      // of another tick by that tick's travel but not its turn. Its tick's camera yaw minus the solve's turns its arm targets about the
      // solve's body before that travel: the free arm stays on the controller in its own tick's frame, as on foot.
      float tick_yaw=0.f; bool matched=false;
      {std::lock_guard<std::mutex> lock(rider_mutex); matched=rider_tick_yaw(root_world,&tick_yaw);}
      if(matched) {
        float d=std::remainder(tick_yaw-vr_arms_math::yaw_of(solve.input.camera),6.2831853f);
        if(!(std::fabs(d)<0.35f)) d=0.f;
        if(kRiderMeshMoment) mesh_yaw=d; rider_tick_matched.fetch_add(1,std::memory_order_relaxed);
        const unsigned cd=static_cast<unsigned>(std::fabs(d)*5729.578f);
        unsigned seen=rider_mesh_yaw_cdeg.load(std::memory_order_relaxed);
        while(cd>seen && !rider_mesh_yaw_cdeg.compare_exchange_weak(seen,cd,std::memory_order_relaxed)) {}
      } else rider_tick_unmatched.fetch_add(1,std::memory_order_relaxed);
      const float d=std::sqrt(to_solve);
      if(d>0.005f) rider_mesh_moments.fetch_add(1,std::memory_order_relaxed);
      const unsigned mm=std::isfinite(d)?static_cast<unsigned>(d*1000.f):0u;
      unsigned seen=rider_mesh_moment_mm.load(std::memory_order_relaxed);
      while(mm>seen && !rider_mesh_moment_mm.compare_exchange_weak(seen,mm,std::memory_order_relaxed)) {}
    }
  }
  float pinned_wrist[2][16]{}; bool pinned[2]{};
  MountedGunProof draw_proof;
  if(solved) mounted_gun_pins(solve,pinned_wrist,pinned,draw_proof);
  // Held outputs already refer to the committed gun's world frame. Do not add
  // another mesh/body travel offset to bracelets on that arm.
  // (mounted_parity: the pins carry the commit to the solve's body, which the capture already holds; this mesh's moment is then added
  // like any other attachment, so the gun hand moves with its mesh as the gun palette moves with its own.)
  // A gun that cannot follow its own tick keeps the solve's body: then its hand and that arm's gear stay on it (the previous rule), and only
  // the shoulder end of that arm moves with the mesh, as on foot the gun hand stays on the written socket.
  const bool pin_follows_mesh=rider_parity && rider_gun_follows_mesh.load(std::memory_order_relaxed);
  for(unsigned h=0;h<2;++h) if(pinned[h] && !pin_follows_mesh) solve.attachment[h].body={body_position[0],body_position[1],body_position[2]};
  if(solved) {  // Native arm HUD while mounted: the solved 192-bone arm, in the same camera axes. Named FP bones do not apply here.
    float cam[16]; std::uint64_t id=0; vr_arms::Sample sample;
    const int method=hud_frame_method.load(std::memory_order_relaxed);
    if(wrist_hud::current().native_arm && vr_arms::latest(sample) && native_stereo::frame_camera(method==1||method==3?1:0,cam,&id)) {
      // [AUTHOR 2026-11-11] HUD elements on the hands a bit unstable while riding. [SOURCE] the wrist came from this draw's tick solve but
      // was measured against the frame's current camera, which can be another step than the one the hands were built on, and every arm
      // mesh draw overwrote it. On foot the HUD is relative to the game camera of the same sample. Per-tick solve: the solve's own sample
      // camera, carried to the tick as its grips are, so every draw of a tick gives one HUD pose.
      if(rider_tick_mode()) {
        std::memcpy(cam,solve.input.camera,sizeof(cam));
        for(int k=0;k<3;++k) cam[12+k]+=solve.body[k]-solve.input.body[k];
      }
      skin_palette_probe::HudCameraPose next; next.frame_id=id; next.rider=true;
      const skin_arm_ik::Vec body{body_position[0],body_position[1],body_position[2]}, no_cell{0.f,0.f,0.f};
      for(int hand=0;hand<2;++hand) {
        float wrist[16],lower[16];
        if(!sample.valid[hand]||!rider_ik::attachment_frame(wrist,solve.attachment[hand],rider_ik::kHand,body,no_cell)||
           !rider_ik::attachment_frame(lower,solve.attachment[hand],rider_ik::kLower,body,no_cell)) continue;
        next.hand_ok[hand]=true;
        hud_to_camera(cam,wrist+12,next.wrist[hand]); hud_to_camera(cam,lower+12,next.elbow[hand]);
        std::memcpy(next.attach[hand],next.wrist[hand],12);
        hud_axes_to_camera(cam,wrist,next.wrist_axes[hand]); std::memcpy(next.attach_axes[hand],next.wrist_axes[hand],36);
        std::memcpy(next.outward[hand],sample.local[hand],12);
        for(int a=0;a<3;++a) std::memcpy(next.grip_axes[hand]+a*3,sample.local[hand]+a*4,12);
      }
      // The reticle: gun in the palm of the same tick (so travel cancels and a two-hand swing stays), on the palm as drawn now, with the
      // on-foot calibrated ray. Fixed against the gun: animation does not enter it.
      {float placed[16],palm_at_commit[16],muzzle[3],forward[3]; unsigned d=1; bool have=false,fresh_gun=false;
        {std::lock_guard<std::mutex> lock(mounted_gun_mutex); const auto& proof=mounted_gun_proof;
          have=proof.onfoot_hold&&proof.profile.valid&&proof.profile.aim_valid;
          fresh_gun=have&&proof.have_placed&&mounted_weapon_math::fresh(GetTickCount64(),proof.committed_at);
          std::memcpy(placed,proof.placed,64); std::memcpy(palm_at_commit,proof.committed_free,64); d=proof.hand<2?proof.hand:0u;
          for(int k=0;k<3;++k) placed[12+k]-=ride_origin[k];  // same space as the committed palm
          const auto m=proof.profile.muzzle,f=proof.profile.forward;
          muzzle[0]=m.x; muzzle[1]=m.y; muzzle[2]=m.z; forward[0]=f.x; forward[1]=f.y; forward[2]=f.z;}
        next.ray_flat=!mounted_gun_mode()||!have;
        float rest_palm[16],local_palm[16],gun_in_palm[16],wrist[16],palm_now[16],gun[16]; const auto& rest=solve.rest.local[d];
        if(fresh_gun&&next.hand_ok[d]&&solve.rest.valid[d]&&!mount_camera::harness_held(d)&&
           mounted_weapon_math::palm(rest[rider_ik::kRHand],skin_arm_ik::position(rest[rider_ik::kRMiddle]),
             skin_arm_ik::position(rest[rider_ik::kRIndex]),skin_arm_ik::position(rest[rider_ik::kRPinky]),rest_palm)&&
           mounted_weapon_math::relative(rest_palm,rest[rider_ik::kRHand],local_palm)&&
           mounted_weapon_math::relative(placed,palm_at_commit,gun_in_palm)&&
           rider_ik::attachment_frame(wrist,solve.attachment[d],rider_ik::kHand,body,no_cell)) {
          skin_arm_ik::multiply_affine(palm_now,local_palm,wrist);
          skin_arm_ik::multiply_affine(gun,gun_in_palm,palm_now);
          float start[3],axes[9];
          for(int k=0;k<3;++k) start[k]=gun[12+k]+muzzle[0]*gun[k]+muzzle[1]*gun[4+k]+muzzle[2]*gun[8+k];
          hud_to_camera(cam,start,next.ray_start); hud_axes_to_camera(cam,gun,axes);
          for(int r=0;r<3;++r) next.ray_way[r]=forward[0]*axes[r]+forward[1]*axes[3+r]+forward[2]*axes[6+r];
          next.ray_ok=true;
        }}
      if(next.hand_ok[0]||next.hand_ok[1]) store_hud_camera(next);
    }
  }
  // The solved pose moved into this build's moment: the whole rider moved with the body (and the mount) since then.
  float shift[3]{};
  if(solved) for(int k=0;k<3;++k) shift[k]=body_position[k]-solve.body[k];
  auto pose_at=[&](int index){return skin_arm_ik::Vec{solve.positions[index][0]+shift[0],solve.positions[index][1]+shift[1],
                                                      solve.positions[index][2]+shift[2]};};
  bool store=false;
  if(!known) {
    char path[128]{};
    const bool have_path=read_path(root,path);
    info.hide=have_path && (std::strstr(path,"/CHARACTERART/PLAYER/HEAD/")!=nullptr ||
                            std::strstr(path,"/CHARACTERART/PLAYER/HAIR/")!=nullptr);
    info.role.assign(n,static_cast<signed char>(rider_ik::kNone)); info.side.assign(n,-1); info.pose_index.assign(n,-1);
    info.hand_node.assign(n,-1); info.hand_anchor.assign(n,-1); info.hand_family.assign(n,-1);
    info.along.assign(n,0.f); info.candidate.assign(n,0); info.twist_bone.assign(n,0); info.fixed_helper.assign(n,0);
    info.correction=correction; info.resource=resource;
    // Only the rider's own meshes: player art/gear, or a body skinned at the player's root (the third-person body is
    // under NPC/RNF). The mount (harness, wings) is near the hands and must never follow them.
    const bool player_gear=have_path && std::strstr(path,"/PLAYER/")!=nullptr;
    const bool player_mesh=player_gear || distance<0.15f*0.15f;
    info.player=player_mesh; info.path=have_path?path:"<unknown>";
    info.rooted=distance<0.02f*0.02f;
    bool record=false;
    {std::lock_guard<std::mutex> lock(rider_diag_mutex);
      if(!info.hide && player_mesh && rider_diag_seen.size()<32) record=rider_diag_seen.insert(info.path).second;}
    std::string names="rider mesh bones "+info.path+" n="+std::to_string(n)+":";
    std::string unknown="rider unmapped hand/arm "+info.path+":";
    unsigned thumb_count=0,twist_count=0,unknown_count=0;
    if(!info.hide && player_mesh) for(unsigned i=0;i<n;++i) {
      Sample entry{};
      if(!read_name(root,i,entry) || entry.name_count!=n) break;
      int side=-1; const rider_ik::Role role=rider_ik::role(entry.bone_name,side,correction);
      if(role==rider_ik::kHand) {
        info.hand_node[i]=static_cast<short>(mounted_hand_pose::index(entry.bone_name));
        info.hand_family[i]=static_cast<signed char>(mounted_hand_pose::family(entry.bone_name));
      }
      info.twist_bone[i]=rider_ik::twist_helper(entry.bone_name);
      info.fixed_helper[i]=rider_ik::arm_skin_helper(entry.bone_name);
      const bool arm_name=!std::strncmp(entry.bone_name,"L_",2)||!std::strncmp(entry.bone_name,"R_",2);
      const bool relevant=arm_name && (role!=rider_ik::kNone || std::strstr(entry.bone_name,"thumb") ||
        std::strstr(entry.bone_name,"finger") || std::strstr(entry.bone_name,"hand") || std::strstr(entry.bone_name,"Arm"));
      if(relevant && record) {
        names+=" "+std::to_string(i)+"="+entry.bone_name+"/role"+std::to_string(static_cast<int>(role));
        if(names.size()>1400) {rider_note(names); names="rider mesh bones continued "+info.path+":";}
        if(role==rider_ik::kNone) {unknown+=" "+std::to_string(i)+"="+entry.bone_name; ++unknown_count;}
        if(std::strstr(entry.bone_name,"thumb")) ++thumb_count;
        if(info.twist_bone[i]) ++twist_count;
      }
      if(role==rider_ik::kNone) {info.candidate[i]=player_gear && !body_part_name(entry.bone_name); continue;}
      info.role[i]=static_cast<signed char>(role); info.side[i]=static_cast<signed char>(side); info.arms=true;
      for(const auto& e:pose_names::k192) if(!std::strcmp(e.name,entry.bone_name)) {info.pose_index[i]=static_cast<short>(e.index); break;}
      if(info.pose_index[i]>=0 && side>=0) {
        const auto& b=rider_ik::kArm[side];
        const int bones7[7]={b.shoulder,b.upper,b.lower,b.hand,b.index,b.middle,b.pinky};
        for(int k=0;k<7;++k) if(info.pose_index[i]==bones7[k]) info.joint[side][k]=static_cast<short>(i);
      }
    }
    if(record) {
      rider_note(names); if(unknown_count) rider_note(unknown);
      rider_note("rider mesh coverage "+info.path+" thumb_helpers="+std::to_string(thumb_count)+" twist_helpers="+
        std::to_string(twist_count)+" unmapped="+std::to_string(unknown_count));
    }
    store=true;
  }
  if(known && !info.rooted && distance<0.02f*0.02f) {info.rooted=true; store=true;}
  if(correction && solved && info.helper_generation && info.helper_generation!=solve.rest_generation) {
    // A new mount/rest capture invalidates inferred decoration ownership as well as its local attachment matrices.
    for(unsigned i=0;i<n;++i) if(info.candidate[i]) {info.role[i]=rider_ik::kNone; info.side[i]=-1; info.fixed_helper[i]=0;}
    info.anchored=false; store=true;
  }
  if(!info.hide && !info.anchored && solved) {
    // Arm segments in this mesh's space (animated pose), then: twist share of named bones, segment of unnamed ones.
    for(unsigned i=0;i<n;++i) {
      const skin_arm_ik::Vec p=skin_arm_ik::position(mats[i]);
      for(int h=0;h<2;++h) {
        const auto& b=rider_ik::kArm[h];
        const skin_arm_ik::Vec s=pose_at(b.upper),e=pose_at(b.lower),w=pose_at(b.hand);
        if(info.side[i]==h && info.role[i]!=rider_ik::kNone) {
          float d=0.f;
          if(info.role[i]==rider_ik::kLower) info.along[i]=std::clamp(skin_arm_ik::segment_t(p,e,w,d),0.f,1.f);
          if(info.role[i]==rider_ik::kUpper) info.along[i]=std::clamp(skin_arm_ik::segment_t(p,s,e,d),0.f,1.f);
        } else if(info.candidate[i] && info.role[i]==rider_ik::kNone) {
          float along=0.f;
          const rider_ik::Role r=rider_ik::nearest_role(p,s,e,w,along);
          if(r!=rider_ik::kNone) {
            info.role[i]=static_cast<signed char>(r); info.side[i]=static_cast<signed char>(h); info.along[i]=along;
            info.arms=true; rider_anchored.fetch_add(1,std::memory_order_relaxed);
          }
        }
      }
    }
    info.anchored=true; store=true;
  }
  if(store) {
    std::lock_guard<std::mutex> lock(rider_mutex);
    if(rider_meshes.size()>=1024) rider_meshes.clear();
    rider_meshes[mesh]=info;
  }
  const bool rider_gear=info.path.find("/GEAR/PLAYER/")!=std::string::npos;
  if(rider_gear && info.arms) {
    // A bracelet/charm belongs to its solved arm segment, not to the live hover/boost pose.
    for(unsigned i=0;i<n;++i) if(info.role[i]>=0 && info.side[i]>=0) info.fixed_helper[i]=1;
    std::lock_guard<std::mutex> lock(rider_diag_mutex);
    if(rider_gear_presence.size()<24 || rider_gear_presence.count(info.path)) {
      auto& p=rider_gear_presence[info.path]; ++p.calls; p.last=GetTickCount64();
      p.root_gap=std::max(p.root_gap,std::sqrt(distance));
      if(solved) p.sequence=solve.input.sequence;
    }
  }
  if(info.hide) {
    for(unsigned i=0;i<n;++i) for(int r=0;r<3;++r) for(int c=0;c<3;++c) mats[i][r*4+c]=0.f;  // every vertex onto its joint
    const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
    rider_hidden.fetch_add(1,std::memory_order_relaxed);
    return original(mesh,root,header,remap,cells);
  }
  if(!info.player && !info.hide && solved) {
    const skin_arm_ik::Vec palms[2]={pose_at(rider_ik::kPalm[0]),pose_at(rider_ik::kPalm[1])};
    rider_frame_sample(info.path,root,mats,n,root_matrix,palms);
  }
  if(rider_parity && correction && !info.arms && !info.hide && info.player && solve.rest_generation &&
     info.path.find("/GEAR/PLAYER/UTILITY/")!=std::string::npos) {
    // Riding twin of item_call: bound once per rest capture to the nearest solved arm segment (hand, forearm, upper arm; no shoulders),
    // within 0.2 m of any of its bones in the native pose; then bone = fixed local * that segment as solved.
    RiderItemBind bind; bool have=false;
    {std::lock_guard<std::mutex> lock(rider_mutex);
      for(const auto& b:rider_item_binds) if(b.resource==resource && b.n==n && b.generation==solve.rest_generation) {bind=b; have=true; break;}}
    if(!have) {
      float best=0.2f; int side=-1,role=-1;
      for(int h=0;h<2;++h) for(int r:{rider_ik::kHand,rider_ik::kLower,rider_ik::kUpper}) {
        float joint[3]; for(int k=0;k<3;++k) joint[k]=solve.joints[h][r][12+k]+1024.f*solve.cell[k];
        for(unsigned i=0;i<n;++i) {
          float d=0.f; for(int k=0;k<3;++k) {const float e=mats[i][12+k]+1024.f*cell[k]-joint[k]; d+=e*e;}
          d=std::sqrt(d); if(d<best) {best=d; side=h; role=r;}
        }
      }
      if(side<0) {rider_item_misses.fetch_add(1,std::memory_order_relaxed); return original(mesh,root,bones,remap,cells);}
      float joint[16],inverse[16]; std::memcpy(joint,solve.joints[side][role],64);
      for(int k=0;k<3;++k) joint[12+k]+=1024.f*(solve.cell[k]-cell[k]);  // into the item's cell
      if(!skin_arm_ik::inverse_affine(inverse,joint)) return original(mesh,root,bones,remap,cells);
      bind.resource=resource; bind.n=n; bind.generation=solve.rest_generation; bind.side=side; bind.role=role; bind.local.resize(n*16u);
      for(unsigned i=0;i<n;++i) skin_arm_ik::multiply_affine(bind.local.data()+i*16,mats[i],inverse);
      std::lock_guard<std::mutex> lock(rider_mutex);
      if(rider_item_binds.size()>=16) rider_item_binds.erase(rider_item_binds.begin());
      rider_item_binds.push_back(bind);
      char line[300]; std::snprintf(line,sizeof(line),"rider item bound %s side=%d role=%d distance=%.3f m",info.path.c_str(),side,role,best);
      rider_note(line);  // rider_mutex -> rider_diag_mutex, the order rider_update_solve already uses
    }
    float target[16];
    const skin_arm_ik::Vec at{body_position[0],body_position[1],body_position[2]},origin{1024.f*cell[0],1024.f*cell[1],1024.f*cell[2]};
    if(!(mount_camera::arm_ik_weight(static_cast<unsigned>(bind.side))>=.999f) ||
       !rider_ik::attachment_frame(target,solve.attachment[bind.side],static_cast<rider_ik::Role>(bind.role),at,origin))
      return original(mesh,root,bones,remap,cells);
    for(unsigned i=0;i<n;++i) skin_arm_ik::multiply_affine(mats[i],bind.local.data()+i*16,target);
    const void* check=nullptr;
    if(!read(bones,0,&check,sizeof(check)) || check!=source) return original(mesh,root,bones,remap,cells);
    const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
    rider_item_apps.fetch_add(1,std::memory_order_relaxed);
    return original(mesh,root,header,remap,cells);
  }
  if(!info.arms || !solved || (!correction && !solve.ok[0] && !solve.ok[1])) return original(mesh,root,bones,remap,cells);
  // Harness grab: each arm eases between the IK (1) and the game's riding pose (0), see mount_camera::update_grab.
  const float weight[2]={mount_camera::arm_ik_weight(0),mount_camera::arm_ik_weight(1)};
  if(!(weight[0]>0.f) && !(weight[1]>0.f)) return original(mesh,root,bones,remap,cells);
  // [HEADSET 2026-10-24] "deforms a lot and very jiggery ... accessories detach ... flat animations fighting the IK":
  // one solve on a pose snapshot was applied to meshes skinned at other moments (the body and the mount move 20 cm a
  // frame at boost speed, the riding animation sways). Each side is now solved on THIS mesh's own arm joints when it
  // has them, with the controller target taken relative to this mesh's root (= the body at this mesh's moment):
  // body, arm armour and bracelets get the same hand. Meshes without their own arm chain use the reference solve (its
  // rigid delta carries any translation). The joint check only counts now; rejecting a mesh detached it.
  rider_ik::Arm arm[2]={solve.arm[0],solve.arm[1]};
  bool ok[2]={solve.ok[0],solve.ok[1]};
  const rider_ik::Tune tune=rider_tune_now();
  {
    const float* basis=solve.input.camera;
    const float* body_then=solve.input.body;
    for(int h=0;h<2;++h) {
      if(correction && rider_gear) continue; // its targets are copied from the shared solve below
      const short* jt=info.joint[h];
      float grip[16];
      if(!(weight[h]>0.f) || !solve.input.valid[h]) continue;
      std::memcpy(grip,solve.input.grip[h],sizeof(grip));
      // All rider meshes share one arm target. A gear root may be at an attachment instead of the feet; using it
      // as the body origin adds that offset to the controller and separates bracelets from the body arm.
      float moment[3];
      // mounted_parity: this mesh's own body (its root when rooted on the body); equal to solve.body otherwise.
      for(int k=0;k<3;++k) moment[k]=(rider_parity?body_position[k]:solve.body[k])-body_then[k];
      float mesh_basis[16]; std::memcpy(mesh_basis,basis,64);
      if(rider_parity) {  // world at the solve's body, into this mesh's tick, into its cell
        for(int k=0;k<3;++k) grip[12+k]+=solve.body[k]-body_then[k];
        to_mesh(grip,solve.body);
        for(int k=0;k<3;++k) grip[12+k]-=1024.f*cell[k];
        if(mesh_yaw!=0.f) skin_arm_ik::rotate_about_up(mesh_basis,basis[12],basis[14],mesh_yaw);  // axes only; the solve reads its rows
      } else
      for(int k=0;k<3;++k) grip[12+k]+=moment[k]-1024.f*cell[k];
      const bool chain=jt[1]>=0 && jt[2]>=0 && jt[3]>=0;
      if(solve.rest.valid[h]) {
        // Animated joints this mesh's bones are expressed against: its own, else the pose's moved to this moment.
        static thread_local float moved[4][16];
        const float* anim[4]{};
        if(chain) {anim[rider_ik::kUpper]=mats[jt[1]]; anim[rider_ik::kLower]=mats[jt[2]]; anim[rider_ik::kHand]=mats[jt[3]];
                   anim[rider_ik::kShoulder]=jt[0]>=0?mats[jt[0]]:nullptr;}
        else for(int role=0;role<4;++role) {
          std::memcpy(moved[role],solve.joints[h][role],64);
          if(rider_parity) {const float c[3]={solve.body[0]-1024.f*solve.cell[0],solve.body[1]-1024.f*solve.cell[1],solve.body[2]-1024.f*solve.cell[2]}; to_mesh(moved[role],c);}
          else for(int k=0;k<3;++k) moved[role][12+k]+=body_position[k]-solve.body[k];
          anim[role]=moved[role];
        }
        float view[16];
        std::memcpy(view,basis,sizeof(view));
        view[3]=view[7]=view[11]=0.f; view[15]=1.f;
        for(int k=0;k<3;++k) view[12+k]=basis[12+k]-1024.f*cell[k]+moment[k];
        if(rider_parity) {
          for(int k=0;k<3;++k) view[12+k]=basis[12+k]+solve.body[k]-body_then[k];
          to_mesh(view,solve.body);
          for(int k=0;k<3;++k) view[12+k]-=1024.f*cell[k];
        }
        skin_arm_ik::Vec bend=solve.bend[h];
        rider_ik::Arm own{};
        float held_wrist[16]; std::memcpy(held_wrist,pinned_wrist[h],64);
        for(int k=0;k<3;++k) held_wrist[12+k]-=1024.f*(cell[k]-ride_origin_cell[k]);
        if(pin_follows_mesh) {  // the pin moves with this mesh's tick (held_wrist: this mesh's cell)
          for(int k=0;k<3;++k) held_wrist[12+k]+=1024.f*cell[k];
          to_mesh(held_wrist,solve.body);
          for(int k=0;k<3;++k) held_wrist[12+k]-=1024.f*cell[k];
        }
        if(rider_ik::solve_rest(solve.rest,static_cast<unsigned>(h),view,anim,grip,rider_parity?mesh_basis:basis,tune,own,correction?nullptr:&bend,pinned[h]?held_wrist:nullptr)) {
          if(correction && solve.twist_valid[h]) own.twist=rider_ik::continuous_twist(own.twist,solve.twist_reference[h]);
          arm[h]=own; ok[h]=true; rider_rest_solves.fetch_add(1,std::memory_order_relaxed);
        }
        continue;
      }
      if(!chain) continue;
      rider_ik::Joints j;
      j.shoulder=jt[0]>=0?mats[jt[0]]:nullptr;
      j.upper=mats[jt[1]]; j.lower=mats[jt[2]]; j.hand=mats[jt[3]];
      const skin_arm_ik::Vec hand_pos=skin_arm_ik::position(mats[jt[3]]);
      const auto& b=rider_ik::kArm[h];
      auto knuckle=[&](int k,int pose_bone){
        if(jt[k]>=0) return skin_arm_ik::position(mats[jt[k]]);
        // Not in this mesh: the animated pose's knuckle offset from its hand (translation-free).
        return skin_arm_ik::add(hand_pos,skin_arm_ik::Vec{solve.positions[pose_bone][0]-solve.positions[b.hand][0],
                                                          solve.positions[pose_bone][1]-solve.positions[b.hand][1],
                                                          solve.positions[pose_bone][2]-solve.positions[b.hand][2]});};
      j.index=knuckle(4,b.index); j.middle=knuckle(5,b.middle); j.pinky=knuckle(6,b.pinky);
      skin_arm_ik::Vec bend=solve.bend[h];
      rider_ik::Arm own{};
      if(rider_ik::solve_joints(j,static_cast<unsigned>(h),grip,basis,tune,own,&bend)) {
        arm[h]=own; ok[h]=true; rider_self_solves.fetch_add(1,std::memory_order_relaxed);
      }
    }
  }
  for(unsigned i=0;i<n;++i) {  // diagnostics only
    const int pi=info.pose_index[i];
    if(pi<0) continue;
    bool joint=false;
    for(const auto& b:rider_ik::kArm) joint=joint||pi==b.upper||pi==b.lower||pi==b.hand;
    if(!joint) continue;
    const float l=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(mats[i]),pose_at(pi)));
    if(!(l<0.06f)) {
      rider_mismatch.fetch_add(1,std::memory_order_relaxed);
      const int mm=static_cast<int>(std::min(l,1000.f)*1000.f);
      int seen=rider_mismatch_mm.load();
      while(mm>seen && !rider_mismatch_mm.compare_exchange_weak(seen,mm)) {}
      rider_mismatch_bone.store(pi,std::memory_order_relaxed);
      break;
    }
  }
  unsigned attachment_status[2]{}; // diagnostic only: 1 fresh, 2 cached, 0 uncommitted
  if(!correction) {
    for(unsigned i=0;i<n;++i) {
      const int role=info.role[i], side=info.side[i];
      if(role<0 || side<0 || !ok[side] || !(weight[side]>0.f)) continue;
      if(rider_ik::blend(mats[i],arm[side].delta[role],weight[side]))
        rider_ik::twist(mats[i],arm[side],static_cast<rider_ik::Role>(role),info.along[i]*weight[side],tune);
    }
  } else {
    static thread_local float source_mats[kMaxGearBones][16];
    std::memcpy(source_mats,mats,static_cast<std::size_t>(n)*64);
    float parents[2][4][16]{};
    for(int h=0;h<2;++h) for(int role=0;role<4;++role) {
      const int slot=role==rider_ik::kShoulder?0:role+1;
      const int index=info.joint[h][slot];
      if(index>=0) std::memcpy(parents[h][role],source_mats[index],64);
      else {
        std::memcpy(parents[h][role],solve.joints[h][role],64);
        if(rider_parity) {const float c[3]={solve.body[0]-1024.f*solve.cell[0],solve.body[1]-1024.f*solve.cell[1],solve.body[2]-1024.f*solve.cell[2]}; to_mesh(parents[h][role],c);}
        else for(int k=0;k<3;++k) parents[h][role][12+k]+=body_position[k]-solve.body[k];
      }
    }
    if(rider_gear) {
      // Gear is an attachment to the shared rider solve, never an independently solved avatar.
      // Its native joints can differ in pose, time and root origin, but its output segment must agree exactly.
      const skin_arm_ik::Vec origin{1024.f*cell[0],1024.f*cell[1],1024.f*cell[2]};
      const skin_arm_ik::Vec body{body_position[0],body_position[1],body_position[2]};
      const float identity[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
      for(unsigned h=0;h<2;++h) {
        arm[h]=solve.attachment[h].arm; ok[h]=solve.attachment[h].valid;
        for(int role=0;role<4 && ok[h];++role) {
          float target[16];
          if(rider_parity && !(pinned[h] && !pin_follows_mesh)) {
            // The solve's world frame turned and moved into this mesh's tick, then into its cell (a pin that stays on the gun is not moved).
            const skin_arm_ik::Vec at_solve{solve.body[0],solve.body[1],solve.body[2]},none{0.f,0.f,0.f};
            ok[h]=rider_ik::attachment_frame(target,solve.attachment[h],static_cast<rider_ik::Role>(role),at_solve,none);
            if(ok[h]) {
              // attachment_frame shifted it by solve.body - pose.body: the matrix is world at solve.body
              to_mesh(target,solve.body);
              for(int k=0;k<3;++k) target[12+k]-=k==0?origin.x:k==1?origin.y:origin.z;
            }
            ok[h]=ok[h] && rider_ik::attachment_delta(arm[h].delta[role],parents[h][role],target,identity,{0,0,0});
          } else
          ok[h]=rider_ik::attachment_frame(target,solve.attachment[h],static_cast<rider_ik::Role>(role),body,origin) &&
            rider_ik::attachment_delta(arm[h].delta[role],parents[h][role],target,identity,{0,0,0});
        }
      }
    }
    if(solve.rest_generation && info.helper_generation!=solve.rest_generation) {
      // Freeze each mesh's own helper-to-segment relationship, never substitute the 192-pose helper list.
      std::vector<float> local(n*16u,0.f),along(n,0.f);
      bool valid=true; unsigned count=0;
      for(unsigned i=0;i<n;++i) if(info.fixed_helper[i] && info.side[i]>=0) {
        const int h=info.side[i],role=info.role[i]; float inverse[16];
        if(!rider_ik::quality(source_mats[i]).valid || !skin_arm_ik::inverse_affine(inverse,parents[h][role])) {valid=false; break;}
        skin_arm_ik::multiply_affine(local.data()+i*16,source_mats[i],inverse);
        const bool lower=role==rider_ik::kLower;
        float distance=0.f;
        along[i]=std::clamp(skin_arm_ik::segment_t(skin_arm_ik::position(source_mats[i]),
          skin_arm_ik::position(parents[h][lower?rider_ik::kLower:rider_ik::kUpper]),
          skin_arm_ik::position(parents[h][lower?rider_ik::kHand:rider_ik::kLower]),distance),0.f,1.f);
        ++count;
      }
      if(valid) {
        std::vector<short> anchors(n,-1);
        if(rider_gear) for(unsigned i=0;i<n;++i) if(info.fixed_helper[i] && info.candidate[i]) {
          const int side=info.side[i],role=info.role[i];
          if(side<0 || role<0) continue;
          // Bind once to the nearest identified twist frame on the same segment. A charm and its bracelet then
          // retain their offsets to that frame through hover/boost rather than twisting separately in place.
          int anchor=info.joint[side][role==rider_ik::kHand?3:role==rider_ik::kLower?2:role==rider_ik::kUpper?1:0];
          float best=1e30f;
          for(unsigned j=0;j<n;++j) if(info.twist_bone[j] && info.side[j]==side && info.role[j]==role) {
            const float d=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(source_mats[i]),skin_arm_ik::position(source_mats[j])));
            if(d<best) {best=d; anchor=static_cast<int>(j);}
          }
          if(anchor<0 || anchor==static_cast<int>(i) || !info.fixed_helper[anchor]) continue;
          float inverse[16];
          if(!skin_arm_ik::inverse_affine(inverse,source_mats[anchor])) {valid=false; break;}
          skin_arm_ik::multiply_affine(local.data()+i*16,source_mats[i],inverse);
          anchors[i]=static_cast<short>(anchor);
        }
        if(!valid) return original(mesh,root,bones,remap,cells);
        unsigned grouped=0; for(short anchor:anchors) if(anchor>=0) ++grouped;
        info.helper_anchor=std::move(anchors);
        info.helper_local=std::move(local); info.helper_along=std::move(along); info.helper_generation=solve.rest_generation;
        {std::lock_guard<std::mutex> lock(rider_mutex); rider_meshes[mesh]=info;}
        bool report=false;
        {std::lock_guard<std::mutex> lock(rider_diag_mutex);
          const std::string key="helper-rest/"+info.path+"/"+std::to_string(solve.rest_generation);
          if(rider_diag_seen.size()<64) report=rider_diag_seen.insert(key).second;}
        if(report && count) rider_note("rider helper rest "+info.path+" count="+std::to_string(count)+
                                      " generation="+std::to_string(solve.rest_generation)+
                                      " grouped_decorations="+std::to_string(grouped));
      }
    }
    static thread_local float along_now[kMaxGearBones];
    std::copy(info.along.begin(),info.along.end(),along_now);
    for(unsigned i=0;i<n;++i) if(info.twist_bone[i] && info.side[i]>=0) {
      const auto* joint=info.joint[info.side[i]];
      const bool lower=info.role[i]==rider_ik::kLower;
      const int from=lower?joint[2]:joint[1],to=lower?joint[3]:joint[2];
      if(from>=0 && to>=0) {
        float distance=0.f;
        along_now[i]=std::clamp(skin_arm_ik::segment_t(skin_arm_ik::position(source_mats[i]),
          skin_arm_ik::position(source_mats[from]),skin_arm_ik::position(source_mats[to]),distance),0.f,1.f);
      }
    }
    bool good[2]{}; unsigned bad[2]{}; rider_ik::Quality rejected[2];
    for(unsigned h=0;h<2;++h) {
      if(!(weight[h]>0.f)) continue;  // released to the native harness pose
      good[h]=ok[h] && rider_ik::transform_arm(mats,source_mats,n,info.role.data(),info.side.data(),
                   info.twist_bone.data(),along_now,h,arm[h],weight[h],tune,&bad[h],&rejected[h]);
      if(good[h] && info.helper_generation==solve.rest_generation && solve.rest_generation) {
        for(unsigned i=0;i<n;++i) if(info.fixed_helper[i] && info.side[i]==static_cast<int>(h)) {
          const auto role=static_cast<rider_ik::Role>(info.role[i]);
          bool applied=false;
          const int anchor=info.helper_anchor.size()==n?info.helper_anchor[i]:-1;
          if(anchor>=0) {
            float target_anchor[16]; rider_ik::Arm identity=arm[h];
            for(int k=0;k<16;++k) identity.delta[role][k]=(k%5==0)?1.f:0.f;
            applied=rider_ik::fixed_helper(target_anchor,source_mats[anchor],info.helper_local.data()+anchor*16,
              parents[h][role],arm[h],role,info.helper_along[anchor],info.twist_bone[anchor]!=0,1.f,tune) &&
              rider_ik::fixed_helper(mats[i],source_mats[i],info.helper_local.data()+i*16,target_anchor,
                identity,role,0.f,false,weight[h],tune);
          } else applied=rider_ik::fixed_helper(mats[i],source_mats[i],info.helper_local.data()+i*16,parents[h][role],arm[h],
                                   role,info.helper_along[i],info.twist_bone[i]!=0,weight[h],tune);
          if(!applied) {
            good[h]=false; bad[h]=i; rejected[h]=rider_ik::quality(mats[i]); break;
          }
        }
        if(!good[h]) for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.role[i]>=0)
          std::memcpy(mats[i],source_mats[i],64);
      }
    }
    for(unsigned h=0;h<2;++h) if(good[h] && pinned[h]) {
      // A rejected optional finger profile must not drop the whole equipped arm
      // back to native animation; this function publishes the hand atomically.
      if(!mounted_gun_fingers(mesh,info,solve,source_mats,mats,n,h,draw_proof))
        mounted_finger_rejects.fetch_add(1,std::memory_order_relaxed);
    }
    float root_inv[16];
    const bool root_valid=skin_arm_ik::inverse_affine(root_inv,root_matrix);
    std::lock_guard<std::mutex> lock(rider_mutex);
    if(rider_good.size()>=64 && rider_good.find(mesh)==rider_good.end()) rider_good.clear();
    auto& saved=rider_good[mesh];
    if(saved.resource!=resource || saved.owner!=player.body || saved.mount_id[0]!=solve.mount_id[0] ||
       saved.mount_id[1]!=solve.mount_id[1] || saved.local.size()!=n*16u) {
      saved=RiderGood{}; saved.resource=resource; saved.owner=player.body;
      saved.mount_id[0]=solve.mount_id[0]; saved.mount_id[1]=solve.mount_id[1]; saved.local.resize(n*16u);
    }
    for(unsigned h=0;h<2;++h) {
      if(!(weight[h]>0.f)) {saved.at[h]=0; continue;}
      if(good[h] && root_valid) {
        attachment_status[h]=1;
        for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.role[i]>=0)
          skin_arm_ik::multiply_affine(saved.local.data()+i*16,mats[i],root_inv);
        saved.at[h]=GetTickCount64();
      } else {
        // A short tracking/solve failure keeps the previous complete arm, carried by the current mesh root.
        if(saved.at[h] && GetTickCount64()-saved.at[h]<=250 && root_valid) {
          bool reusable=true;
          for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.role[i]>=0) {
            skin_arm_ik::multiply_affine(mats[i],saved.local.data()+i*16,root_matrix);
            reusable=reusable && rider_ik::preserves_geometry(source_mats[i],mats[i]);
          }
          if(!reusable) for(unsigned i=0;i<n;++i) if(info.side[i]==static_cast<int>(h) && info.role[i]>=0)
            std::memcpy(mats[i],source_mats[i],64);
          if(reusable) attachment_status[h]=2;
        }
        if(!ok[h]) {
          const std::string key=info.path+"/missing/"+std::to_string(h);
          if(rider_reject_budget.size()<32 || rider_reject_budget.count(key))
            if(rider_reject_budget[key]++<3) rider_note("rider missing solve "+info.path+" side="+std::to_string(h));
          continue;
        }
        const std::string diagnostic_key=info.path+"/"+std::to_string(h);
        if(rider_reject_budget.size()<32 || rider_reject_budget.count(diagnostic_key)) {
          auto& reports=rider_reject_budget[diagnostic_key];
          if(reports++>=3) continue;
          const auto q=rider_ik::quality(source_mats[bad[h]]), a=rejected[h];
          char line[480];
          std::snprintf(line,sizeof(line),"rider quality reject mesh=%s side=%u bone=%u valid=%d/%d axis=%.3f,%.3f,%.3f -> %.3f,%.3f,%.3f ortho=%.4f/%.4f det=%.4f/%.4f",
            info.path.c_str(),h,bad[h],q.valid?1:0,a.valid?1:0,q.axis[0],q.axis[1],q.axis[2],a.axis[0],a.axis[1],a.axis[2],
            q.orthogonality,a.orthogonality,q.determinant,a.determinant);
          rider_note(line);
        }
      }
    }
  }
  const void* check_source=nullptr;
  if(!read(bones,0,&check_source,sizeof(check_source)) || check_source!=source) return original(mesh,root,bones,remap,cells);
  if(correction && rider_gear) {
    std::lock_guard<std::mutex> lock(rider_diag_mutex);
    const auto it=rider_gear_presence.find(info.path);
    if(it!=rider_gear_presence.end() && primary_scope.mesh==mesh && primary_scope.record) {
      auto& v=it->second;
      if(v.visibility_generation!=solve.rest_generation) {
        v.visibility_generation=solve.rest_generation; v.visibility_reports=0; v.next_visibility=0;
      }
      if(v.visibility_reports<60 && GetTickCount64()>=v.next_visibility && rider_diag.size()<256) {
        ++v.visibility_reports; v.next_visibility=GetTickCount64()+500;
        float bounds[12]{}; unsigned char flags[2]{},fade=0,mesh_flags[2]{};
        const bool bounds_ok=read(primary_scope.record,0x20,bounds,sizeof(bounds));
        const bool flags_ok=read(primary_scope.record,0x84,flags,sizeof(flags)) &&
          read(mesh,0x18,mesh_flags,sizeof(mesh_flags)) && read(mesh,0x120,&fade,sizeof(fade));
        skin_arm_ik::Vec low{1e30f,1e30f,1e30f},high{-1e30f,-1e30f,-1e30f};
        unsigned assigned=0;
        for(unsigned i=0;i<n;++i) if(info.side[i]>=0 && info.role[i]>=0) {
          const auto p=skin_arm_ik::add(skin_arm_ik::position(mats[i]),skin_arm_ik::Vec{
            1024.f*cell[0],1024.f*cell[1],1024.f*cell[2]});
          low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};
          high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)}; ++assigned;
        }
        char line[1100];
        std::snprintf(line,sizeof(line),"rider visibility owner mesh=%s object=%p record=%p seq=%llu rest=%u read=%d/%d bounds20=%.4f,%.4f,%.4f,%.4f bounds30=%.4f,%.4f,%.4f,%.4f extent40=%.4f,%.4f,%.4f,%.4f flags84=%02X,%02X mesh18=%02X,%02X scalar120=%u arm_bones=%u arm_world_min=%.4f,%.4f,%.4f arm_world_max=%.4f,%.4f,%.4f",
          info.path.c_str(),mesh,primary_scope.record,static_cast<unsigned long long>(solve.input.sequence),
          solve.rest_generation,bounds_ok?1:0,flags_ok?1:0,bounds[0],bounds[1],bounds[2],bounds[3],
          bounds[4],bounds[5],bounds[6],bounds[7],bounds[8],bounds[9],bounds[10],bounds[11],
          flags[0],flags[1],mesh_flags[0],mesh_flags[1],fade,assigned,low.x,low.y,low.z,high.x,high.y,high.z);
        rider_diag.emplace_back(line);
      }
    }
    if(it!=rider_gear_presence.end()) for(int h=0;h<2;++h) {
      const int hand_index=info.joint[h][3];
      if(hand_index<0 || weight[h]<0.999f || !ok[h]) continue;
      float expected_frame[16];
      if(!rider_ik::attachment_frame(expected_frame,solve.attachment[h],rider_ik::kHand,
         {body_position[0],body_position[1],body_position[2]},{0,0,0})) continue;
      const auto expected=skin_arm_ik::position(expected_frame);
      const auto actual=skin_arm_ik::add(skin_arm_ik::position(mats[hand_index]),skin_arm_ik::Vec{
        1024.f*cell[0],1024.f*cell[1],1024.f*cell[2]});
      it->second.wrist_gap=std::max(it->second.wrist_gap,skin_arm_ik::length(skin_arm_ik::sub(actual,expected)));
      ++it->second.wrist_samples;
      auto& audit=it->second;
      if(audit.divergence_generation!=solve.rest_generation) {
        audit.divergence_generation=solve.rest_generation; audit.divergence_reports=0; audit.next_divergence=0;
      }
      const float gap=skin_arm_ik::length(skin_arm_ik::sub(actual,expected));
      if(gap>0.03f && audit.divergence_reports<12 && GetTickCount64()>=audit.next_divergence && rider_diag.size()<256) {
        ++audit.divergence_reports; audit.next_divergence=GetTickCount64()+500;
        float reference_target[16];
        skin_arm_ik::multiply_affine(reference_target,solve.joints[h][rider_ik::kHand],solve.arm[h].delta[rider_ik::kHand]);
        const float reference_gap=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(reference_target),solve.arm[h].wrist));
        const auto q=rider_ik::quality(mats[hand_index]);
        char line[720];
        std::snprintf(line,sizeof(line),"rider attachment divergence mesh=%s side=%d seq=%llu rest=%u slot=%d status=%u gap_m=%.6f reference_gap_m=%.6f source=%p resource=%p valid=%d determinant=%.6f helper_generation=%u anchor_count=%zu",
          info.path.c_str(),h,static_cast<unsigned long long>(solve.input.sequence),solve.rest_generation,
          hand_index,attachment_status[h],gap,reference_gap,source,resource,q.valid?1:0,q.determinant,
          info.helper_generation,info.helper_anchor.size());
        rider_diag.emplace_back(line);
      }
    }
  }
  const void* replacement=mats; std::memcpy(header,&replacement,sizeof(replacement));
  if(correction && tune.accessory_bounds && rider_gear && primary_scope.mesh==mesh && primary_scope.record) {
    const void* linked_mesh=nullptr; mounted_accessory_bounds::Bounds before{},after{};
    if(read(primary_scope.record,0x70,&linked_mesh,sizeof(linked_mesh)) && linked_mesh==mesh &&
       read(primary_scope.record,0x20,&before,sizeof(before))) {
      skin_arm_ik::Vec low{1e30f,1e30f,1e30f},high{-1e30f,-1e30f,-1e30f}; unsigned count=0;
      for(unsigned i=0;i<n;++i) if(info.side[i]>=0 && info.role[i]>=0 && ok[info.side[i]] && weight[info.side[i]]>0.f) {
        const auto p=skin_arm_ik::add(skin_arm_ik::position(mats[i]),skin_arm_ik::Vec{
          1024.f*cell[0],1024.f*cell[1],1024.f*cell[2]});
        low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};
        high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)}; ++count;
      }
      float scale=0.f;
      for(int k=0;k<3;++k) scale=std::max(scale,skin_arm_ik::length(skin_arm_ik::row(root_matrix,k)));
      // This callback regenerates native bounds before its palette call. Include both poses for later submission.
      if(count && mounted_accessory_bounds::include(after,before,low,high,scale)) {
        const bool written=game::mem_write(static_cast<unsigned char*>(const_cast<void*>(primary_scope.record))+0x20,
                                          &after,sizeof(after));
        bool report=false;
        {std::lock_guard<std::mutex> lock(rider_diag_mutex);
          const std::string key="bounds-union/"+info.path+"/"+std::to_string(solve.rest_generation);
          if(rider_diag_seen.size()<96) report=rider_diag_seen.insert(key).second;}
        if(report) rider_note("rider accessory bounds union "+info.path+" written="+std::to_string(written?1:0)+
          " native_radius="+std::to_string(before.world[3])+" union_radius="+std::to_string(after.world[3]));
      }
    }
  }
  if(mount_camera::capture_active() && info.path.find("/BODY/")!=std::string::npos) {
    bool valid[2]{}; float native_hands[2][16]{},rendered_hands[2][16]{};
    for(int h=0;h<2;++h) {
      const int index=info.joint[h][3];
      if(index>=0 && static_cast<unsigned>(index)<n && read(source,index*64,native_hands[h],64)) {
        std::memcpy(rendered_hands[h],mats[index],64); valid[h]=true;
      }
    }
    mount_camera::observe_rider_hands(mesh,resource,source,solve.owner,cell,valid,native_hands,rendered_hands,weight,solve.input.sequence);
  }
  rider_arm_apps.fetch_add(1,std::memory_order_relaxed);
  gesture_palette(root,mats,n);
  return original(mesh,root,header,remap,cells);
}
void __fastcall mesh_primary_observer(void* node,void* context) {
  const MeshPrimaryScope previous=primary_scope;
  primary_scope=MeshPrimaryScope{};
  if(mount_camera::rider_view()) {
    const void* holder=nullptr; const void* record=nullptr; const void* mesh=nullptr;
    // Static callback provenance: context+840 -> holder -> graph record; record+70 is the palette mesh.
    // The record lives only inside this callback scope and is never retained or written.
    if(read(context,0x840,&holder,sizeof(holder)) && read(holder,0,&record,sizeof(record)) &&
       read(record,0x70,&mesh,sizeof(mesh))) primary_scope={record,mesh};
  }
  primary_original(node,context);
  primary_scope=previous;
}
bool __fastcall detour(void* mesh,const void* root,const void* bones,const void* remap,const void* cells) {
  const auto caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress());
  const auto base=reinterpret_cast<std::uintptr_t>(game::module_base());
  mount_camera::observe_palette(mesh, root, bones, cells, caller-base);
  if(caller==base+0x4591FA && GetTickCount64()<deadline.load()) {
    calls.fetch_add(1,std::memory_order_relaxed);
    std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
    if(lock.owns_lock() && total<64 && pending<64) {
      const auto now=GetTickCount64();
      const void* source=nullptr;
      bool seen=now<next_sample || !read(bones,0,&source,sizeof(source));
      const void* identity=source?source:bones;
      for(unsigned i=0;i<source_count;++i) seen|=seen_sources[i]==identity;
      // Preserve coverage across mesh parts; at most one new source per100ms,64 per window.
      if(!seen) {
        Sample s{}; s.mesh=mesh; s.header=bones; s.thread=GetCurrentThreadId();
        const auto player=game::pointers(); s.body=player.body;
        if(player.body!=window_body.load()) {
          deadline.store(0);
          lock.unlock();
          return original(mesh,root,bones,remap,cells);
        }
        const void* comp=nullptr; const void* pose192=nullptr;
        read(player.pose,0,&s.cpu90,sizeof(s.cpu90));
        read(player.body,0x1A8,&comp,sizeof(comp));
        read(comp,0x4F8,&pose192,sizeof(pose192));
        read(pose192,0,&s.cpu192,sizeof(s.cpu192));
        if(player.body) {
          s.read_flags|=read(bones,0,&s.data,sizeof(s.data))?1u:0u;
          s.read_flags|=read(bones,8,&s.count,sizeof(s.count))?2u:0u;
          s.read_flags|=read(remap,0,&s.remap,sizeof(s.remap))?4u:0u;
          s.read_flags|=read(remap,8,&s.map_count,sizeof(s.map_count))?8u:0u;
          s.read_flags|=read(cells,0,s.cells,sizeof(s.cells))?16u:0u;
          // Remap+8 is an unverified candidate count: record it without using it as a gate.
          if(s.remap) s.read_flags|=read(s.remap,0,s.indices,sizeof(std::uint16_t))?32u:0u;
          const unsigned wrist=0; // Count alone does not identify a skeleton or wrist index.
          if(read_name(root,wrist,s)) s.read_flags|=128u;
          else {std::memcpy(s.bone_name,"<unavailable>",14);}
          arm_names(root,s);
          if(s.data && s.count>0 && s.count<=512)
            s.read_flags|=read(s.data,wrist*64+48,s.hand,sizeof(s.hand))?64u:0u;
          // Reject a changed local-player identity; pointers are logged only, never retained for writes.
          if(game::pointers().body==player.body) {
            samples[pending++]=s; ++total; seen_sources[source_count++]=identity; next_sample=now+100;
          }
        }
      }
    }
  }
  if(mount_camera::rider_view()) {
    bool handled=false; const bool result=mounted_gun_palette(mesh,root,bones,remap,cells,handled);
    if(handled) return result;
    return rider_call(mesh,root,bones,remap,cells);
  }
  // While active every palette build is offered: accessory meshes may come from other MeshSkinned callers.
  if(pulse_until.load()) return pulse_call(mesh,root,bones,remap,cells,caller-base);
  return original(mesh,root,bones,remap,cells);
}
bool install() {
  auto target=game::module_base()+kPaletteRva;
  unsigned char bytes[sizeof(kPaletteSig)]{};
  if(!game::mem_read(target,bytes,sizeof(bytes)) || std::memcmp(bytes,kPaletteSig,sizeof(bytes))) {
    log("guard mismatch; observer disabled"); return false;
  }
  if(MH_CreateHook(target,reinterpret_cast<void*>(&detour),reinterpret_cast<void**>(&original))!=MH_OK) return false;
  if(MH_EnableHook(target)!=MH_OK) {MH_RemoveHook(target); return false;}
  // Optional read-only owner correlation. A mismatch disables only this observer, never the established IK hook.
  constexpr unsigned char primary_sig[]={0x40,0x55,0x53,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x48,
    0x8D,0xAC,0x24,0x90,0xF8,0xFF,0xFF,0x48,0x81,0xEC,0x70,0x08};
  auto primary_target=game::module_base()+0x4580B0;
  unsigned char primary_bytes[sizeof(primary_sig)]{};
  if(game::mem_read(primary_target,primary_bytes,sizeof(primary_bytes)) &&
     !std::memcmp(primary_bytes,primary_sig,sizeof(primary_sig)) &&
     MH_CreateHook(primary_target,reinterpret_cast<void*>(&mesh_primary_observer),reinterpret_cast<void**>(&primary_original))==MH_OK) {
    if(MH_EnableHook(primary_target)==MH_OK) log("rider visibility owner observer installed (read only, bounded)");
    else {MH_RemoveHook(primary_target); primary_original=nullptr; log("rider visibility owner observer enable failed");}
  } else log("rider visibility owner observer unavailable (guard/hook); arm IK remains active");
  log("read-only caller-scoped palette observer installed; original arguments/result preserved"); return true;
}
}
namespace {
// ---- IK hands into the game's own poses (experiment, "posewrite" token; 2026-10-15) ----
// [AUTHOR/LIVE] Bullets leave from the flat weapon position and a weapon piece, a charm and a cord stay at the flat hands:
// the IK only changes the palette copy handed to the renderer, so gameplay readers of the game's poses (FP 90-bone pose
// component+0x500, 192-bone pose +0x4F8; research table in WEAPONS_AND_HANDS.md) still see the animated hands. Right after
// the camera update (game thread; earlier tests showed writes there persist to Render), every same-named HAND-SUBTREE bone
// (wrist + hand-role followers: fingers, weapon attachment) gets the rigid change the IK gave it in the last solved frame.
// Hands only: they are re-targeted from the controllers every frame, so a palette fed from these poses cannot accumulate.
std::atomic<unsigned> posewrite_frames{0},posewrite_bones90{0},posewrite_bones192{0},posewrite_skips{0};
// [LIVE] 2026-10-15: the "posewrite" token was dropped from AvatarVR_native.txt by the menu's native-file save, so the
// experiment never ran (posewrite=0/0/0). It is a saved menu toggle now ("Game hands follow IK (experimental)").
bool pose_write_menu=false;  // menu row removed 2026-10-15 (posewrite had no visible effect); "posewrite" token only
int pose_to_fp90[90],pose_to_fp192[192];
bool pose_maps_ready=false;
void build_pose_maps() {
  auto build=[](const pose_names::Entry* e,int count,int* out){
    for(int i=0;i<count;++i) out[i]=-1;
    for(int i=0;i<count;++i) {
      const int b=e[i].index; if(b<0||b>=count) continue;
      for(unsigned j=0;j<211;++j) if(fp_names[j]==e[i].name) {
        const bool hand_subtree=bone_side[j]>=0 && (bone_role[j]==2 || j==16 || j==115);
        if(hand_subtree) out[b]=static_cast<int>(j);
        break;
      }
    }
  };
  build(pose_names::k90,90,pose_to_fp90);
  build(pose_names::k192,192,pose_to_fp192);
  pose_maps_ready=true;
}
unsigned write_pose(unsigned char* pose,int expected,const int* map,const float (&raw)[211][16],const float (&out)[211][16],
                    const float* body_shift) {
  if(!pose) return 0;
  unsigned char* data=nullptr; std::int32_t count=0;
  if(!game::mem_read(pose,&data,sizeof(data)) || !game::mem_read(pose+8,&count,sizeof(count)) || !data || count!=expected) return 0;
  // Palette-space offset of this pose (whole 1024-unit cells), from the first mapped bone pair.
  float offset[3]{}; bool have_offset=false;
  unsigned written=0;
  for(int b=0;b<expected;++b) {
    const int j=map[b]; if(j<0) continue;
    float m[16];
    if(!game::mem_read(data+64*b,m,sizeof(m))) continue;
    if(!have_offset) {
      for(int k=0;k<3;++k) offset[k]=std::round((raw[j][12+k]-m[12+k])/1024.f)*1024.f;
      have_offset=true;
    }
    float inverse[16],delta[16],pal[16],moved[16];
    if(!skin_arm_ik::inverse_affine(inverse,raw[j])) continue;
    skin_arm_ik::multiply_affine(delta,inverse,out[j]);
    std::memcpy(pal,m,sizeof(pal));
    for(int k=0;k<3;++k) pal[12+k]+=offset[k]-body_shift[k];  // into the solve frame's palette coordinates
    skin_arm_ik::multiply_affine(moved,pal,delta);
    for(int k=0;k<3;++k) moved[12+k]+=body_shift[k]-offset[k];
    bool ok=true; for(float v:moved) ok=ok&&std::isfinite(v);
    if(ok && game::mem_write(data+64*b,moved,sizeof(moved))) ++written;
  }
  return written;
}
}  // namespace
namespace skin_palette_probe {
namespace {
constexpr const char* mounted_attempt_names[]={"placed","mounted_disabled","mounted_unproven_caller","mounted_unavailable",
  "mounted_no_palette_binding","mounted_native_source_unavailable","mounted_no_grip_profile","mounted_entity_not_bound",
  "mounted_position_rejected","mounted_cell_unreadable","mounted_no_frame","mounted_arm_unavailable",
  "mounted_harness_owned","mounted_other_hand_unavailable","other"};
constexpr unsigned mounted_attempt_count=sizeof(mounted_attempt_names)/sizeof(mounted_attempt_names[0]);
std::atomic<unsigned> mounted_attempts[mounted_attempt_count]{};
}
void note_mounted_weapon_attempt(const char* reason,bool wrote) {
  unsigned index=mounted_attempt_count-1;
  if(wrote) index=0;
  else if(reason) for(unsigned i=1;i<mounted_attempt_count-1;++i) if(std::strcmp(reason,mounted_attempt_names[i])==0) {index=i;break;}
  mounted_attempts[index].fetch_add(1,std::memory_order_relaxed);
}
void on_camera_post() {
  {const game::Player pl=game::read(); float body[3]; const game::Pointers bp=game::pointers();
    if(pl.valid && bp.body && game::mem_read(bp.body+0x94,body,sizeof(body))) {
      {std::lock_guard<std::mutex> lock(rig_body_mutex); std::memcpy(rig_body,body,12); rig_body_us=clean_qpc_us();}
      // [SOURCE 2026-11-07] riding, arms and gun land on body + previous offset (rider input + body travel); a smoothed eye anchor would sit
      // up to 25 cm off it. On foot the arms read the smoothed offset itself, so smoothing stays on foot only.
      vr_arms::set_view_stable(rig_coherent,!mount_camera::rider_view());
      rig_camera_tick.fetch_add(1,std::memory_order_relaxed);
      float c16[16]; std::memcpy(c16,pl.cam,64);
      if(hand_stabilize) {float sp[3]; stabilized_position(c16+12,body,sp); for(int k=0;k<3;++k) c16[12+k]=sp[k];}
      std::lock_guard<std::mutex> lock(cam_post_mutex); std::memcpy(cam_post,c16,64); std::memcpy(cam_post_body,body,12); cam_post_valid=true;}}
  if(!(pose_write_menu || native_stereo::pose_write_enabled()) || !controller_mode.load() || side_state.load()!=2) return;
  const ULONGLONG now=GetTickCount64();
  if(!pulse_until.load() || now>=pulse_until.load() || !pulse_permission.load()) return;
  static float raw[211][16],out[211][16];
  float body_last[3];
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    if(!fp_last_at || now-fp_last_at>200) {posewrite_skips.fetch_add(1); return;}
    std::memcpy(raw,fp_raw_last,sizeof(raw)); std::memcpy(out,fp_out_last,sizeof(out)); std::memcpy(body_last,fp_body_last,sizeof(body_last));}
  if(!pose_maps_ready) build_pose_maps();
  const game::Pointers p=game::pointers();
  if(!p.body || !p.pose) {posewrite_skips.fetch_add(1); return;}
  float body_now_abs[3];
  if(!game::mem_read(p.body+0x94,body_now_abs,sizeof(body_now_abs))) return;
  // Body movement since the solve frame, in the palette's cell (whole cells cancel through the per-pose offset).
  float shift[3];
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    for(int k=0;k<3;++k) shift[k]=(body_now_abs[k]-1024.f*fp_cell_last[k])-body_last[k];}
  for(float& v:shift) if(!std::isfinite(v) || std::fabs(v)>2.f) v=0.f;
  posewrite_bones90.fetch_add(write_pose(p.pose,90,pose_to_fp90,raw,out,shift));
  unsigned char* comp=nullptr; unsigned char* pose192=nullptr;
  if(game::mem_read(p.body+0x1A8,&comp,sizeof(comp)) && comp && game::mem_read(comp+0x4F8,&pose192,sizeof(pose192)))
    posewrite_bones192.fetch_add(write_pose(pose192,192,pose_to_fp192,raw,out,shift));
  posewrite_frames.fetch_add(1);
}
}  // namespace skin_palette_probe
namespace {
// Arm settings persist in scripts\AvatarVR_arms.txt ("key value" lines): loaded on the first Present, saved within a
// second of any change. Script writes still resets at every launch (lua_host), so IK only acts after enabling it.
struct Setting {const char* key; float* f; bool* b;};
std::vector<Setting> settings() {
  return {{"arm_ik",nullptr,&ik_menu},{"hand_pitch",&hand_tune.pitch,nullptr},{"hand_yaw",&hand_tune.yaw,nullptr},
          {"hand_roll",&hand_tune.roll,nullptr},{"hand_forward",&hand_tune.forward,nullptr},{"hand_up",&hand_tune.up,nullptr},
          {"hand_side",&hand_tune.side,nullptr},{"shoulder_height",&shoulder_height,nullptr},
          {"shoulder_forward",&shoulder_forward,nullptr},{"shoulders_follow",nullptr,&shoulders_follow},
          {"stable_shoulders",nullptr,&stable_rig},{"native_hand_poses",nullptr,&native_hand_poses},{"forearm_twist",nullptr,&forearm_twist},{"steady_hands",nullptr,&body_anchor},
          {"game_pitch",virtual_pad::pitch_target_value(),nullptr},{"keep_pitch_level",nullptr,virtual_pad::pitch_level_flag()},
          {"arm_projection_fix",nullptr,native_stereo::fp_proj_menu_flag()},{"arm_scaling",nullptr,&arm_scale.enabled},
          {"user_arm_length",&arm_scale.user_length,nullptr},{"user_shoulder_drop",&arm_scale.drop,nullptr},
          {"user_shoulder_width",&arm_scale.width,nullptr},{"user_shoulder_back",&arm_scale.back,nullptr},{"reach_multiplier",&arm_scale.reach,nullptr},
          {"shoulder_separation",&shoulder_spread,nullptr},{"arm_depth_occlusion",nullptr,native_stereo::fp_occlude_menu_flag()},
          {"upper_twist_share",&upper_twist_share,nullptr},{"hand_smoothing",nullptr,&hand_smoothing},{"accessory_physics",nullptr,&accessory_physics},{"rig_coherent",nullptr,&rig_coherent},{"shadow_weapon_follow",nullptr,&shadow_weapon_follow},{"accessory_rigid",nullptr,&accessory_rigid},{"skeleton_anchor_strict",nullptr,&skeleton_anchor_strict},{"shadow_gear_follow",nullptr,&shadow_gear_follow},{"body_arm_shift",&body_arm_shift,nullptr},{"mounted_parity",nullptr,&mounted_parity},{"rider_tick_solve",nullptr,&rider_tick_solve},{"bare_hand_static",nullptr,&bare_hand_static},
          {"weapon_hand",&weapon_hand_setting,nullptr},{"grips_in_frame_camera",nullptr,&frame_cam},{"model_space_hands",nullptr,&model_space_hands},{"stabilize_hands",nullptr,&hand_stabilize},{"arms_follow_game_tick",nullptr,&tick_resolve},{"body_turn_in_step",nullptr,&root_marker_carry},{"gun_hand_shoulder",nullptr,&shoulder_socket},{"clean_arm_rig",nullptr,&clean_rig},{"left_handed",nullptr,&left_handed},{"vr_controls",nullptr,vr_controls::enabled_flag()},{"mounted_weapon_tracking",nullptr,&mounted_weapon_menu},{"two_hand_aiming",nullptr,&two_hand_enabled},{"gesture_haptics",nullptr,vr_controls::haptics_flag()},{"shoulders_view_height",nullptr,&view_height_anchor},{"arms_lock_to_view",nullptr,&arms_lock_to_view},{"hand_freeze_moving",nullptr,&hand_freeze_moving},{"weapon_trail_comp",&weapon_trail_comp,nullptr},{"empty_hand_gestures",nullptr,hand_gestures::enabled_flag()}};  // "game_hands_follow_ik" removed 2026-10-15 (no effect)
}
std::string settings_text() {
  std::string text;
  for(const Setting& e:settings()) {
    char line[96];
    if(e.f) std::snprintf(line,sizeof(line),"%s %.4f\n",e.key,*e.f); else std::snprintf(line,sizeof(line),"%s %d\n",e.key,*e.b?1:0);
    text+=line;
  }
  return text;
}
std::string saved_text;
bool settings_loaded=false;
ULONGLONG settings_next=0;
void persist_settings(ULONGLONG now) {
  const std::wstring file=dir+L"\\AvatarVR_arms.txt";
  if(!settings_loaded) {
    settings_loaded=true;
    static char stored[8192];
    if(settings_file::read(file,stored,sizeof(stored))) {
      char key[64]; float value=0.f; unsigned count=0; int used=0;
      for(const char* at=stored; std::sscanf(at,"%63s %f%n",key,&value,&used)==2; at+=used) {
        for(const Setting& e:settings()) if(!std::strcmp(e.key,key) && std::isfinite(value)) {
          if(e.f) *e.f=value; else *e.b=value!=0.f; ++count;}
      }
      char line[96]; std::snprintf(line,sizeof(line),"arm settings loaded: %u values from AvatarVR_arms.txt",count); log(line);
    }
    saved_text=settings_text();
    return;
  }
  if(now<settings_next) return;
  settings_next=now+1000;
  const std::string text=settings_text();
  if(text==saved_text) return;
  if(settings_file::write(file,text)) saved_text=text;
}
}  // namespace
namespace skin_palette_probe {
bool* ik_menu_flag() { return &ik_menu; }
Counters counters() {
  return {fp_body_apps.load(std::memory_order_relaxed), accessory_apps.load(std::memory_order_relaxed),
          weapon_apps.load(std::memory_order_relaxed), clean_solves.load(std::memory_order_relaxed),
          clean_gun_places.load(std::memory_order_relaxed)};
}
skin_arm_ik::HandTune* tune() { return &hand_tune; }
bool* shoulders_flag() { return &shoulders_follow; }
bool* twist_flag() { return &forearm_twist; }
bool* anchor_flag() { return &body_anchor; }
void reset_tune() { hand_tune=make_default_tune(); }
bool* stable_flag() { return &stable_rig; }
bool* native_hands_flag() { return &native_hand_poses; }
bool* arm_scale_flag() { return &arm_scale.enabled; }
float* user_arm_length_value() { return &arm_scale.user_length; }
float* user_shoulder_drop_value() { return &arm_scale.drop; }
float* user_shoulder_width_value() { return &arm_scale.width; }
float* reach_multiplier_value() { return &arm_scale.reach; }
float* weapon_trail_value() { return &weapon_trail_comp; }
bool* rig_lock_flag() { return &frame_cam; }
bool* stabilize_flag() { return &hand_stabilize; }
bool* tick_resolve_flag() { return &tick_resolve; }
bool* root_carry_flag() { return &root_marker_carry; }
bool* shoulder_socket_flag() { return &shoulder_socket; }
bool* clean_rig_flag() { return &clean_rig; }
bool* left_handed_flag() { return &left_handed; }
bool begin_hand_swap(int to_hand) {
  if(to_hand<0 || to_hand>1 || swap_to.load()>=0) return false;
  if((to_hand==0)==left_handed) return false;  // already there
  {std::lock_guard<std::mutex> lock(clean_mutex);  // a hand swap ends any two-hand hold (it eases out)
    if(two_hand.held) {const long long now=clean_qpc_us(); two_hand.w_release=two_hand_weight_locked(now); two_hand.held=false; two_hand.release_us=now; set_support_only_locked(false,now);}}
  swap_start_us.store(swap_now_us(),std::memory_order_release);
  swap_to.store(to_hand,std::memory_order_release);
  hand_swaps.fetch_add(1,std::memory_order_relaxed);
  return true;
}
bool hand_swap_active() { return swap_to.load()>=0; }
bool* mounted_weapon_flag() { return &mounted_weapon_menu; }
bool mounted_weapon_entity_target(const void* object,const float* native,float* moved,const char** reason) {
  *reason="mounted_unavailable";
  float native_l[16]; std::memcpy(native_l,native,64); for(int k=0;k<3;++k) native_l[12+k]-=ride_origin[k];  // ride-local, as the frame
  MountedGunFrame frame;
  const float la=left_amount();
  const unsigned hand=la>=1.f?0u:1u;
  t_rig_body_live=true;
  const bool have_frame=mounted_gun_frame(frame);
  t_rig_body_live=false;
  // On foot a tick that cannot evaluate reuses the fresh gun record and the gun never returns to the game's pose. Riding, 16% of ticks did
  // [LIVE 01:25 run]: the gun's rigid parts went to the native socket for that frame. Keep the last placement, carried with the rider.
  auto hold_last=[&](const char* why) {
    *reason=why;
    if(!rig_coherent) return false;
    std::lock_guard<std::mutex> lock(mounted_gun_mutex);
    const auto& proof=mounted_gun_proof;
    if(!proof.have_placed || proof.object!=object || !mounted_weapon_math::fresh(GetTickCount64(),proof.committed_at)) return false;
    const auto p=game::pointers(); float body[3];
    std::memcpy(moved,proof.placed,64);
    if(!p.body || !read(p.body,0x94,body,sizeof(body)) || !mounted_weapon_math::carry(moved,proof.committed_body,body)) return false;
    if(skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(moved),skin_arm_ik::position(native)))>3.f) return false;
    mounted_held_last.fetch_add(1,std::memory_order_relaxed);
    *reason="mounted_held_last"; return true;
  };
  if(!have_frame) return hold_last("mounted_no_frame");
  if(!frame.valid[hand]) {
    for(int i=0;i<6;++i) if(frame.why[hand]&(1u<<i)) mounted_arm_why[i].fetch_add(1,std::memory_order_relaxed);
    return hold_last("mounted_arm_unavailable");
  }
  if(mount_camera::harness_held(hand) || mount_camera::arm_ik_weight(hand)<.999f) {*reason="mounted_harness_owned"; return false;}
  std::lock_guard<std::mutex> lock(mounted_gun_mutex);
  auto& proof=mounted_gun_proof;
  if(!proof.resource || !mounted_gun_identity(frame,proof.identity) || !mounted_weapon_math::fresh(GetTickCount64(),proof.at)) {
    *reason="mounted_no_palette_binding"; return false;
  }
  if(proof.source>1 || !mounted_weapon_math::rigid(frame.flat[proof.source])) {*reason="mounted_native_source_unavailable"; return false;}
  if(!proof.hold_valid) {*reason="mounted_no_grip_profile"; return false;}
  if(la>0.f && la<1.f && !frame.valid[1-hand]) {*reason="mounted_other_hand_unavailable"; return false;}
  float sockets[2][16],socket[16];
  for(int h=0;h<2;++h) {
    float inverse[16]; if(!skin_arm_ik::inverse_affine(inverse,proof.hold[h])) return false;
    skin_arm_ik::multiply_affine(sockets[h],inverse,frame.palm[h]);
  }
  if(la<=0.f) std::memcpy(socket,sockets[1],64);
  else if(la>=1.f) std::memcpy(socket,sockets[0],64);
  else vr_arms_math::blend_affine(sockets[1],sockets[0],la,socket);
  proof.pending_pin[0]=proof.pending_pin[1]=0.f;
  if(la<=0.f || la>=1.f) {
    const int dom=hand,sup=1-dom;
    float support_hold[16]{},support_point[16]{},handle[16];
    skin_arm_ik::multiply_affine(handle,proof.hold[dom],socket);
    bool support=two_hand_enabled && proof.support_valid && frame.valid[sup] && !mount_camera::harness_held(sup) && mount_camera::arm_ik_weight(sup)>.999f;
    if(support) {
      float relation[16]; std::memcpy(relation,proof.support_relative,64);
      if(dom==0) mounted_weapon_math::mirror_palm_hold(proof.support_relative,relation);
      skin_arm_ik::multiply_affine(support_hold,relation,proof.hold[dom]);
      skin_arm_ik::multiply_affine(support_point,support_hold,socket);
    }
    const auto now=clean_qpc_us(); float w=0.f,so=0.f,offset_fade=0.f; TwoHandState th;
    {
      std::lock_guard<std::mutex> state_lock(clean_mutex);
      th_d_support[0]=th_d_support[1]=th_d_handle[0]=th_d_handle[1]=1e9f;
      th_d_handle[sup]=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(frame.palm[sup]),skin_arm_ik::position(handle)));
      if(support) {
        const auto off=skin_arm_ik::sub(skin_arm_ik::position(frame.palm[sup]),skin_arm_ik::position(support_point));
        th_d_support[sup]=skin_arm_ik::length(off);
        for(int r=0;r<3;++r) th_offset_gun[sup][r]=skin_arm_ik::dot(off,skin_arm_ik::row(socket,r));
      } else if(two_hand.held) {two_hand=TwoHandState{}; set_support_only_locked(false,now);}
      th_measured_us=now; th=two_hand; w=two_hand_weight_locked(now); so=support_only_locked(now);
      offset_fade=th.held?1.f-vr_arms_math::smoothstep(static_cast<float>(now-th.start_us)/kTwoHandOffsetUs):0.f;
    }
    if(support && w>0.f && th.support==sup) {
      float pivot[3],cur[3],desired[3];
      for(int k=0;k<3;++k) {
        pivot[k]=handle[12+k]; cur[k]=support_point[12+k]-pivot[k];
        const float offset=(th.offset_gun[0]*socket[k]+th.offset_gun[1]*socket[4+k]+th.offset_gun[2]*socket[8+k])*offset_fade;
        desired[k]=frame.palm[sup][12+k]-offset-pivot[k];
      }
      const float leverage=std::clamp(skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(support_hold),skin_arm_ik::position(proof.hold[dom])))/.35f,0.f,1.f);
      float rotation[16],swung[16];
      vr_arms_math::swing_between(cur,desired,vr_arms_math::two_hand_gain(leverage)*w,(4.f-3.f*leverage)*.0174532925f,
        std::max(12.f,60.f*leverage)*.0174532925f,rotation);
      vr_arms_math::rotate_about(socket,pivot,rotation,swung); std::memcpy(socket,swung,64);
      if(so>0.f) {
        float inverse[16],support_socket[16],mixed[16];
        if(!skin_arm_ik::inverse_affine(inverse,support_hold)) return false;
        skin_arm_ik::multiply_affine(support_socket,inverse,frame.palm[sup]);
        vr_arms_math::blend_affine(socket,support_socket,so,mixed); std::memcpy(socket,mixed,64);
      }
      skin_arm_ik::multiply_affine(proof.pending_palm[sup],support_hold,socket); proof.pending_pin[sup]=w;
    }
    skin_arm_ik::multiply_affine(proof.pending_palm[dom],proof.hold[dom],socket); proof.pending_pin[dom]=1.f-so;
    {std::lock_guard<std::mutex> state_lock(clean_mutex);
      th_d_dom_handle=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(frame.palm[dom]),skin_arm_ik::position(proof.pending_palm[dom])));}
  }
  if(!mounted_weapon_math::rigid(socket)) return false;
  if(mounted_weapon_math::same_frame(native_l,frame.flat[proof.source])) {
    // Capture the object-to-socket relation once; animated attachment sway must
    // not become a new grip offset every frame.
    if(!proof.have_placed || proof.object!=object)
      if(!mounted_weapon_math::relative(native_l,frame.flat[proof.source],proof.offset)) return false;
    if(proof.onfoot_hold) std::memcpy(moved,socket,64);
    else skin_arm_ik::multiply_affine(moved,proof.offset,socket);
  } else if(proof.have_placed && proof.object==object && mounted_weapon_math::same_frame(native,proof.placed)) {
    if(proof.onfoot_hold) std::memcpy(moved,socket,64);
    else skin_arm_ik::multiply_affine(moved,proof.offset,socket);
    if(!mounted_weapon_math::rigid(moved) || skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(moved),skin_arm_ik::position(native_l)))>3.f) return false;
  } else if(rider_tune_now().weapon_stable && mounted_weapon_math::bound_carry(proof.have_placed,proof.object==object,native_l,frame.flat[proof.source])) {
    if(proof.onfoot_hold) std::memcpy(moved,socket,64);
    else skin_arm_ik::multiply_affine(moved,proof.offset,socket);
    proof.at=GetTickCount64(); mounted_bound_carried.fetch_add(1,std::memory_order_relaxed);
  } else {*reason="mounted_entity_not_bound"; return false;}  // [LIVE MOUNT_CAMERA 10-26] the other object is the FP entity, parked at the origin while riding
  for(int k=0;k<3;++k) {proof.pending_local[k]=moved[12+k]; moved[12+k]+=ride_origin[k];}  // back to world for the hook
  proof.pending_object=object; std::memcpy(proof.pending,moved,64);
  if(!mounted_weapon_math::rigid(moved) || skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(moved),skin_arm_ik::position(native)))>3.f) return false;
  proof.pending_hand=hand; std::memcpy(proof.pending_target,socket,64);
  proof.pending_sequence=frame.sequence;
  read(frame.owner,0x94,proof.pending_body,sizeof(proof.pending_body));
  std::memcpy(proof.pending_free,frame.palm[hand],64);
  std::memcpy(proof.pending_flat,frame.flat[proof.source],64);
  *reason="mounted_placed"; return true;
}
void mounted_weapon_entity_commit(const void* object,const float* moved,bool success) {
  native_stereo::GripFrame placed_input;
  {std::lock_guard<std::mutex> lock(rider_mutex); placed_input=rider_solve.input;}
  std::lock_guard<std::mutex> lock(mounted_gun_mutex);
  auto& proof=mounted_gun_proof;
  if(proof.pending_object!=object || !mounted_weapon_math::same_frame(moved,proof.pending,.001f)) return;
  proof.pending_object=nullptr;
  if(!success) mounted_commit_failures.fetch_add(1,std::memory_order_relaxed);
  if(success) {
    {std::lock_guard<std::mutex> input_lock(rider_mutex); rider_commit_input=placed_input; rider_commit_us=clean_qpc_us(); rider_commit_gen=frame_gen.load(std::memory_order_relaxed);}
    proof.have_placed=true;
    proof.object=object; std::memcpy(proof.placed,moved,64);
    proof.hand=proof.pending_hand; std::memcpy(proof.target,proof.pending_target,64); proof.committed_at=GetTickCount64();
    proof.committed_sequence=proof.pending_sequence; std::memcpy(proof.committed_body,proof.pending_body,12); std::memcpy(proof.committed_free,proof.pending_free,64);
    {float placed_l[16]; std::memcpy(placed_l,proof.placed,64); for(int k=0;k<3;++k) placed_l[12+k]=proof.pending_local[k];
      proof.render_history.push(proof.target,proof.committed_at,placed_l);}
    std::memcpy(proof.committed_flat,proof.pending_flat,64);
    std::memcpy(proof.held_palm,proof.pending_palm,sizeof(proof.held_palm));
    std::memcpy(proof.pin,proof.pending_pin,sizeof(proof.pin));
  }
}
bool mounted_weapon_shot_frame(const void* weapon,MountedShotFrame& out) {
  if(!mounted_gun_mode()) return false;
  const auto player=game::pointers(); const auto mount=mount_state::get(); const void* current=nullptr;
  if(!player.body || !read(player.body,0x250,&current,sizeof(current)) || current!=weapon) return false;
  MountedGunProof proof;
  {std::lock_guard<std::mutex> lock(mounted_gun_mutex); proof=mounted_gun_proof;}
  const auto now=GetTickCount64();
  if(!mounted_weapon_math::shot_ready(proof.onfoot_hold && proof.profile.valid,proof.profile.aim_valid,
       proof.have_placed,now,proof.at,proof.committed_at) ||
     proof.identity.owner!=player.body || proof.identity.candidate!=weapon || proof.identity.partner!=mount.partner ||
     proof.identity.id[0]!=mount.partner_id[0] || proof.identity.id[1]!=mount.partner_id[1] ||
     proof.hand!=(left_handed?0u:1u) || mount_camera::harness_held(proof.hand) || mount_camera::arm_ik_weight(proof.hand)<.999f ||
     !mounted_weapon_math::rigid(proof.committed_flat) || !mounted_weapon_math::rigid(proof.placed)) return false;
  MountedShotFrame next;
  std::memcpy(next.flat,proof.committed_flat,64); std::memcpy(next.root,proof.placed,64);
  for(int k=0;k<3;++k) next.flat[12+k]+=ride_origin[k];
  next.calibrated=proof.onfoot_hold && proof.profile.aim_valid;
  next.muzzle=proof.profile.muzzle; next.forward=proof.profile.forward;
  next.sequence=proof.committed_sequence; next.age_ms=now-proof.committed_at;
  const auto after=mount_state::get();
  if(after.phase!=mount_state::Phase::Riding || after.partner!=mount.partner || after.partner_id[0]!=mount.partner_id[0] ||
     after.partner_id[1]!=mount.partner_id[1] || game::pointers().body!=player.body ||
     !read(player.body,0x250,&current,sizeof(current)) || current!=weapon) return false;
  out=next; return true;
}
void observe_onfoot_weapon_ray(const void* weapon,const float* flat,const float* start,const float* target) {
  if(!mounted_weapon_enabled.load(std::memory_order_relaxed) || mount_camera::rider_view() || !mounted_weapon_math::rigid(flat)) return;
  const auto player=game::pointers(); const auto now=GetTickCount64();
  std::lock_guard<std::mutex> lock(mounted_profile_mutex);
  const auto& binding=mounted_profile_binding;
  if(binding.owner!=player.body || binding.weapon!=weapon || !mounted_weapon_math::fresh(now,binding.at)) return;
  auto it=mounted_support_profiles.find(binding.path);
  if(it==mounted_support_profiles.end() || it->second.aim_valid) return;
  auto& profile=it->second; float root[16]; skin_arm_ik::multiply_affine(root,profile.fp_root_socket,flat);
  skin_arm_ik::Vec muzzle,forward;
  if(!vr_weapon_math::local_ray(start,target,root,muzzle,forward)) return;
  struct Pending {const void* weapon=nullptr; std::string path; skin_arm_ik::Vec muzzle,forward; ULONGLONG since=0,last=0;};
  static Pending aim_sample; // profile mutex
  if(aim_sample.weapon!=weapon || aim_sample.path!=binding.path || now-aim_sample.last>100 ||
     skin_arm_ik::length(skin_arm_ik::sub(muzzle,aim_sample.muzzle))>.003f || skin_arm_ik::dot(forward,aim_sample.forward)<.99985f) {
    aim_sample={weapon,binding.path,muzzle,forward,now,now}; return;
  }
  aim_sample.last=now; if(now-aim_sample.since<100) return;
  profile.muzzle=aim_sample.muzzle; profile.forward=aim_sample.forward; profile.aim_valid=true;
  mounted_profile_revision.fetch_add(1,std::memory_order_relaxed);
}
bool weapon_hold(int hand,float (&rel)[16]) {
  CleanHandCache grip; {std::lock_guard<std::mutex> lock(clean_mutex); grip=clean_cache[1];}
  if(!grip.valid || !grip.local.attach) return false;
  if(hand) {std::memcpy(rel,grip.local.rel,64); return true;}
  float h_r[16],h_l[16];  // as clean_socket: the left hand holds the gun with the mirrored right hold
  if(!skin_arm_ik::inverse_affine(h_r,grip.local.rel)) return false;
  vr_arms_math::mirror_hold(h_r,h_l);
  return skin_arm_ik::inverse_affine(rel,h_l);
}
void set_hud_frame_method(int method) {hud_frame_method.store(method,std::memory_order_relaxed);}
void hud_camera_gaps(float* mm,float* degrees) {
  std::lock_guard<std::mutex> lock(hud_camera_mutex);
  *mm=hud_gap_mm; *degrees=hud_gap_degrees; hud_gap_mm=hud_gap_degrees=0.f;
}
void set_hud_bone(int slot,int bone) {if(slot>=0&&slot<HudBoneSlots) hud_bone_request[slot].store(bone,std::memory_order_relaxed);}
void hud_camera_range(float* left_wrist_mm3) {
  std::lock_guard<std::mutex> lock(hud_camera_mutex);
  for(int k=0;k<3;++k) left_wrist_mm3[k]=hud_range_primed?(hud_range_high[k]-hud_range_low[k])*1000.f:0.f;
  hud_range_primed=false;
}
bool hud_camera_seen() {
  std::lock_guard<std::mutex> lock(hud_camera_mutex);
  return hud_camera_at!=0;
}
bool hud_camera_pose(HudCameraPose* out) {
  std::lock_guard<std::mutex> lock(hud_camera_mutex);
  if(!hud_camera_at||GetTickCount64()-hud_camera_at>150) return false;
  *out=hud_camera_latest;
  LARGE_INTEGER now,freq; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&freq);
  out->age_us=static_cast<float>(now.QuadPart-hud_camera_qpc)*1000000.f/static_cast<float>(freq.QuadPart);
  return true;
}
std::string hud_bone_name(int bone) {
  if(bone<0||bone>=211) return {};
  return fp_names[bone];
}
bool hud_arm_anchor(int hand,float* wrist3,float* elbow3,float* attach3) {
  if(hand<0 || hand>1) return false;
  std::lock_guard<std::mutex> lock(hud_anchor_mutex);
  if(!hud_anchor_at || GetTickCount64()-hud_anchor_at>150) return false;
  std::memcpy(wrist3,hud_anchor_points[hand][0],12); std::memcpy(elbow3,hud_anchor_points[hand][1],12);
  if(attach3) std::memcpy(attach3,hud_anchor_points[hand][2],12);
  return true;
}
WeaponCaptureState weapon_capture_state() {
  WeaponCaptureState out;
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    const auto now=GetTickCount64();
    if(fp_last_at && now>=fp_last_at) out.fp_age_ms=now-fp_last_at;
    out.fp_draw_valid=out.fp_age_ms<=200;
    if(out.fp_draw_valid) for(int h=0;h<2;++h) {
      std::memcpy(out.fp_draw[h],fp_out_last[h?115:16],64);
      for(int k=0;k<3;++k) out.fp_draw[h][12+k]+=1024.f*fp_cell_last[k];
    }
  }
  {std::lock_guard<std::mutex> lock(clean_mutex);
    const auto now=clean_qpc_us();
    if(support_hold_us>0 && now>=support_hold_us) out.support_age_us=now-support_hold_us;
    if(th_measured_us>0 && now>=th_measured_us) out.measure_age_us=now-th_measured_us;
    out.support_valid=support_hold.valid; out.two_hand_held=two_hand.held;
    out.support_hand=two_hand.support; out.support_only=so_target;
    std::memcpy(out.support,support_hold.h_sup_l,64); std::memcpy(out.dominant,support_hold.h_dom_r,64);
    std::memcpy(out.support_distance,th_d_support,8); std::memcpy(out.handle_distance,th_d_handle,8);
  }
  out.swap_target=swap_to.load(std::memory_order_relaxed);
  {std::lock_guard<std::mutex> lock(mounted_gun_mutex);
    const auto& proof=mounted_gun_proof; const auto now=GetTickCount64();
    out.mounted_owner=proof.identity.owner; out.mounted_candidate=proof.identity.candidate;
    out.mounted_resource=proof.resource; out.mounted_object=proof.object;
    out.mounted_weapon_path=proof.bound_path;
    if(proof.at && now>=proof.at) out.mounted_bind_age_ms=now-proof.at;
    if(proof.committed_at && now>=proof.committed_at) out.mounted_commit_age_ms=now-proof.committed_at;
    out.mounted_bound=proof.resource && mounted_weapon_math::fresh(now,proof.at);
    out.mounted_committed=proof.have_placed && mounted_weapon_math::fresh(now,proof.committed_at);
    out.mounted_grip_profile=proof.hold_valid; out.mounted_support_profile=proof.support_valid;
    out.mounted_onfoot_hold=proof.onfoot_hold;
    out.mounted_aim_profile=proof.profile.aim_valid; out.mounted_commit_failures=mounted_commit_failures.load(std::memory_order_relaxed);
    for(int h=0;h<2;++h) out.mounted_finger_profile[h]=proof.profile.fingers[h].valid;
    out.mounted_finger_apps=mounted_finger_apps.load(std::memory_order_relaxed);
    out.mounted_finger_rejects=mounted_finger_rejects.load(std::memory_order_relaxed);
    out.mounted_pin_expired=mounted_pin_expired.load(std::memory_order_relaxed);
    out.mounted_pin_distant=mounted_pin_distant.load(std::memory_order_relaxed);
    out.mounted_palette_adopted=mounted_palette_adopted.load(std::memory_order_relaxed);
    out.mounted_palette_unbound=mounted_palette_unbound.load(std::memory_order_relaxed);
    out.mounted_palette_not_ready=mounted_palette_not_ready.load(std::memory_order_relaxed);
    out.mounted_palette_invalid=mounted_palette_invalid.load(std::memory_order_relaxed);
    std::memcpy(out.mounted_pin,proof.pin,sizeof(out.mounted_pin));
    out.mounted_source=proof.source; out.mounted_hand=proof.hand;
    out.mounted_frame_sequence=proof.identity.sequence; out.mounted_commit_sequence=proof.committed_sequence;
    if(proof.source<2) std::memcpy(out.mounted_flat,proof.identity.flat[proof.source],64); else out.mounted_bound=false;
    std::memcpy(out.mounted_target,proof.target,64); std::memcpy(out.mounted_placed,proof.placed,64);
    for(int k=0;k<3;++k) {out.mounted_flat[12+k]+=ride_origin[k]; out.mounted_target[12+k]+=ride_origin[k];}  // the capture reads world
  }
  {std::lock_guard<std::mutex> lock(mounted_profile_mutex);
    out.mounted_profiles=static_cast<unsigned>(mounted_support_profiles.size());
    out.mounted_disk_profiles=mounted_profile_disk_count.load(std::memory_order_relaxed);
  }
  return out;
}
std::string capture_palette_names(const void* root) {
  std::string result;
  for(unsigned index:{0u,16u,115u,39u,77u,48u,155u}) {
    Sample entry{}; const bool valid=read_name(root,index,entry);
    if(index==0) result+=" names_count="+std::to_string(entry.name_count);
    char b[160]; std::snprintf(b,sizeof(b)," name%u=%s",index,valid?entry.bone_name:"<unavailable>"); result+=b;
  }
  return result;
}
int grab_two_hand(int hand) {
  if(mount_camera::rider_view() && (!mounted_gun_mode() || !vr_weapon::weapon_placed_by_game())) return 0;
  if(!two_hand_enabled || hand<0 || hand>1 || swap_to.load()>=0) return 0;
  std::lock_guard<std::mutex> lock(clean_mutex);
  const long long now=clean_qpc_us();
  if(!th_measured_us || now-th_measured_us>200000) return 0;
  const float ds=th_d_support[hand],dh=th_d_handle[hand];
  if(ds<0.10f && ds<=dh) {  // nearest point wins: the support point
    two_hand.support=hand; two_hand.held=true; two_hand.start_us=now; two_hand.release_us=0;
    std::memcpy(two_hand.offset_gun,th_offset_gun[hand],12);
    two_hand_grabs.fetch_add(1,std::memory_order_relaxed);
    return 1;
  }
  if(dh<0.15f) return 2;  // nearer the trigger handle: hand swap
  return 0;
}
void release_two_hand(int hand) {
  std::lock_guard<std::mutex> lock(clean_mutex);
  if(two_hand.held && two_hand.support==hand) {
    const long long now=clean_qpc_us();
    two_hand.w_release=two_hand_weight_locked(now); two_hand.held=false; two_hand.release_us=now;
    set_support_only_locked(false,now);  // run 68: no hand left on the gun's support: it returns to the trigger hand
  }
}
// Run 68: the trigger hand's grip press. 1 = it let go (support hand only), 2 = it re-grabbed the handle, 0 = nothing.
int dominant_grip_press() {
  std::lock_guard<std::mutex> lock(clean_mutex);
  const long long now=clean_qpc_us();
  if(!two_hand.held) return 0;
  if(!so_target) {set_support_only_locked(true,now); support_only_entries.fetch_add(1,std::memory_order_relaxed); return 1;}
  if(th_d_dom_handle<0.12f) {set_support_only_locked(false,now); support_only_regrabs.fetch_add(1,std::memory_order_relaxed); return 2;}
  return 0;
}
bool support_only() { std::lock_guard<std::mutex> lock(clean_mutex); return so_target; }
// Fire is blocked while the trigger hand is off the gun or still easing back onto it.
bool fire_blocked() { std::lock_guard<std::mutex> lock(clean_mutex); return so_target || support_only_locked(clean_qpc_us())>0.05f; }
bool* two_hand_flag() { return &two_hand_enabled; }
bool* view_height_flag() { return &view_height_anchor; }
bool* arms_view_lock_flag() { return &arms_lock_to_view; }
bool* hand_freeze_flag() { return &hand_freeze_moving; }
bool* model_hands_flag() { return &model_space_hands; }  // menu row now toggles the frame-camera grips (run 32)
void calibrate_arms() { calibrate_request.store(true); }
bool* smoothing_flag() { return &hand_smoothing; }
bool* accessory_physics_flag() { return &accessory_physics; }
bool* pose_write_flag() { return &pose_write_menu; }
bool* weapon_pose_write_flag() { return &weapon_pose_write; }
int weapon_hand_index() { return (left_handed?0:1); }
unsigned rider_hidden_builds() { return rider_hidden.load(std::memory_order_relaxed); }
std::string rider_frame_report() {
  std::vector<FrameCensus> list;
  {std::lock_guard<std::mutex> lock(frame_census_mutex); list.swap(frame_census);}
  list.erase(std::remove_if(list.begin(),list.end(),[](const FrameCensus& c){return c.samples<20;}),list.end());
  std::sort(list.begin(),list.end(),[](const FrameCensus& a,const FrameCensus& b){
    return a.palm[0].spread()+a.palm[1].spread()<b.palm[0].spread()+b.palm[1].spread();});
  std::string out;
  char b[512];
  for(std::size_t k=0;k<list.size() && k<10;++k) {
    const auto& c=list[k];
    std::snprintf(b,sizeof(b),"  %s n=%u samples=%u palm spread in its root frame L=%.3f R=%.3f m; mean L=(%.3f,%.3f,%.3f) R=(%.3f,%.3f,%.3f)",
                  c.path.c_str(),c.n,c.samples,c.palm[0].spread(),c.palm[1].spread(),c.palm[0].mean[0],c.palm[0].mean[1],
                  c.palm[0].mean[2],c.palm[1].mean[0],c.palm[1].mean[1],c.palm[1].mean[2]);
    out+=b;
    for(int h=0;h<2;++h) {
      unsigned best=0; double d=1e30;
      for(unsigned i=0;i<c.n;++i) if(c.distance[h][i]<d) {d=c.distance[h][i]; best=i;}
      std::snprintf(b,sizeof(b),"; nearest to %s palm: bone %u %s at %.3f m",h?"R":"L",best,
                    best<c.names.size()&&!c.names[best].empty()?c.names[best].c_str():"<unnamed>",c.samples?d/c.samples:0.0);
      out+=b;
    }
    out+="\r\n";
  }
  return out.empty()?"  (no non-player mesh sampled while riding)\r\n":out;
}
RiderCounters rider_counters() {
  return {rider_hidden.load(), rider_arm_apps.load(), rider_solves.load(), rider_mismatch.load(), rider_no_grip.load(),
          rider_anchored.load(), rider_mismatch_mm.load(), rider_mismatch_bone.load(), rider_self_solves.load(),
          rider_rest_solves.load(), rider_rest_captures.load()};
}
bool drawn_arrow(float (&arrow)[16],float (&bow_root)[16],float (&string)[16],bool* have_string,char (&path)[96],char (*bow)[96]) {
  std::lock_guard<std::mutex> lock(arrow_mutex);
  const ULONGLONG now=GetTickCount64();
  if(arrow_tick && now-arrow_tick<=200) {std::memcpy(arrow,arrow_world,64); std::memcpy(bow_root,bow_root_world,64);}
  else if(bow_tick && now-bow_tick<=200) {std::memcpy(arrow,bow_arrow_world,64); std::memcpy(bow_root,bow_limb_root_world,64);}
  else return false;
  std::memcpy(string,bow_string_world,64);
  if(have_string) *have_string=bow_string_have; std::memcpy(path,arrow_path,sizeof(path));
  if(bow) std::memcpy(*bow,bow_path,sizeof(*bow));
  return true;
}
std::string arrow_report() {
  std::lock_guard<std::mutex> lock(arrow_mutex);
  auto& s=arrow_stats; const ULONGLONG now=GetTickCount64();
  const float d=s.arrow_z[0]*s.bow_z[0]+s.arrow_z[1]*s.bow_z[1]+s.arrow_z[2]*s.bow_z[2];
  char line[320]; std::snprintf(line,sizeof(line),"arrow_draws=%u bow_draws=%u other_arrow_draws=%u other_arrow_deg=%.1f arrow_vs_bow_deg=%.1f arrow_age=%llu bow_age=%llu "
    "near_ik=%u near_flat=%u max_d_ik=%.3f min_d_flat=%.3f replaced=%u kept=%u chosen_d_ik=%.3f",
    s.arrow_draws,s.bow_draws,s.other_draws,s.other_deg,(s.arrow_draws&&s.bow_draws)?std::acos(std::clamp(d,-1.f,1.f))*57.2958f:-1.f,
    arrow_tick?now-arrow_tick:9999ull,bow_tick?now-bow_tick:9999ull,s.near_ik,s.near_flat,s.max_ik,s.min_flat_gap,s.replaced,s.kept,arrow_frame_best);
  s.arrow_draws=s.bow_draws=s.other_draws=s.near_ik=s.near_flat=s.replaced=s.kept=0; s.other_deg=0.f; s.max_ik=0.f; s.min_flat_gap=99.f;
  char shadow[320]; std::snprintf(shadow,sizeof(shadow)," | shadow_follow=%d moves=%u by_count=%u by_hand=%u misses=%u max_gap=%.3f max_d_hand=%.3f "
    "far_left=%u far_min_d_hand=%.3f",
    shadow_weapon_follow?1:0,shadow_moves.exchange(0),shadow_by_count.exchange(0),shadow_by_hand.exchange(0),shadow_misses.exchange(0),
    shadow_max_gap.exchange(0.f),shadow_max_hand.exchange(0.f),shadow_far_left.exchange(0),shadow_far_min_hand.exchange(99.f));
  return std::string(line)+shadow;
}
bool held_weapon_root(float (&out)[16]) {
  std::lock_guard<std::mutex> lock(weapon_write_mutex);
  if(!held_root_tick || GetTickCount64()-held_root_tick>200) return false;
  std::memcpy(out,held_root_world,64); return true;
}
bool held_weapon_drawn() {
  const ULONGLONG tick=held_weapon_draw_tick.load(std::memory_order_relaxed);
  return tick && GetTickCount64()-tick<=200;
}
// Clean rig: the entity hook places the gun with the socket evaluated ONCE per game tick (calls within 3 ms reuse it):
// hand_target(sample now, this tick's body frame). Body frame of the tick: root rotation = the stored FP root carried by
// the 90-pose Marker change (same as fp_root_now, run 47), root position = body now, shoulder = the solve's shoulder in
// root-local coordinates on that root plus the headset shift now. Returns false (old path) until a clean solve has run.
// One hand's gun socket (world) for this tick: right = rel_R * wrist_R; left = inverse(mirror_hold(inverse(rel_R))) *
// wrist_L (run 63). Each wrist is that hand's own target from the shared sample.
bool clean_socket(int hand, const vr_arms::Sample& smp, const float* root_tick, const float* anchor_tick, const float* head,
                  const float* basis, float* socket, float* wrist_out=nullptr) {
  CleanHandCache cache,grip_cache;
  {std::lock_guard<std::mutex> lock(clean_mutex); cache=clean_cache[hand]; grip_cache=clean_cache[1];}
  if(!cache.valid || !grip_cache.valid || !smp.valid[hand]) return false;
  float sh_local[16]{}; sh_local[0]=sh_local[5]=sh_local[10]=sh_local[15]=1.f;
  sh_local[12]=cache.shoulder_root[0]; sh_local[13]=cache.shoulder_root[1]; sh_local[14]=cache.shoulder_root[2];
  float sh_world[16]; skin_arm_ik::multiply_affine(sh_world,sh_local,cache.view_anchor?anchor_tick:root_tick);
  const skin_arm_ik::Vec shoulder{sh_world[12]+head[0],sh_world[13]+head[1],sh_world[14]+head[2]};
  float wrist16[16],attach16[16],factor=0.f; bool oriented=false;
  if(rig_coherent) {
    const unsigned holds=hand_holds.load(std::memory_order_relaxed);
    if(!guarded_hand_target(smp.local[hand],basis,root_tick,shoulder,cache.navi_length,static_cast<unsigned>(hand),cache.local,wrist16,attach16,&factor,&oriented)) return false;
    if(hand_holds.load(std::memory_order_relaxed)!=holds) rig_gun_holds.fetch_add(1,std::memory_order_relaxed);
  } else if(!vr_arms_math::hand_target(smp.local[hand],basis,root_tick,shoulder,cache.navi_length,static_cast<unsigned>(hand),arm_scale,
                                skin_arm_ik::for_hand(hand_tune,static_cast<unsigned>(hand)),cache.local,wrist16,attach16,&factor,&oriented))
    return false;
  if(hand==0) {
    if(!grip_cache.local.attach) return false;
    float h_r[16],h_l[16],inverse_h_l[16];
    if(!skin_arm_ik::inverse_affine(h_r,grip_cache.local.rel)) return false;
    vr_arms_math::mirror_hold(h_r,h_l);
    if(!skin_arm_ik::inverse_affine(inverse_h_l,h_l)) return false;
    skin_arm_ik::multiply_affine(attach16,inverse_h_l,wrist16);
  }
  std::memcpy(socket,attach16,64);
  if(wrist_out) std::memcpy(wrist_out,wrist16,64);
  return true;
}
// The gun placed by the entity hook, once per game tick (calls within 3 ms reuse it). The socket is the right hold, the
// left (mirrored) hold, or - during a smooth hand swap (run 65) - the eased blend of both. The hand argument is ignored:
// the left amount decides.
bool clean_place(int hand, float (&flat)[16], float (&ik)[16]) {
  (void)hand;
  if(attach_bone[1]<0) return false;
  const long long us=clean_qpc_us();
  {std::lock_guard<std::mutex> lock(clean_mutex);
    if(gun_record.valid && us-gun_record.us<3000) {std::memcpy(flat,gun_record.flat,64); std::memcpy(ik,gun_record.ik,64);
      clean_gun_reuses.fetch_add(1,std::memory_order_relaxed); return true;}}
  bool view_anchor=false;
  {std::lock_guard<std::mutex> lock(clean_mutex); view_anchor=clean_cache[1].view_anchor||clean_cache[0].view_anchor;}
  vr_arms::Sample smp; if(!clean_input_sample(smp)) return false;
  float body_now[3]; const game::Pointers bp=game::pointers();
  if(!bp.body || !game::mem_read(bp.body+0x94,body_now,sizeof(body_now))) return false;
  std::int16_t cell[3]{}; float flat_local[16];
  {std::lock_guard<std::mutex> lock(fp_last_mutex);
    if(!fp_last_at || GetTickCount64()-fp_last_at>200) return false;
    std::memcpy(flat_local,fp_raw_last[attach_bone[1]],64); for(int k=0;k<3;++k) cell[k]=fp_cell_last[k];}  // the game's gun hand (right)
  float root_tick[16];
  {const float zero[3]={0.f,0.f,0.f}; const std::int16_t c0[3]={0,0,0};
    if(!fp_root_now(root_tick,zero,c0)) return false;}       // rotation carried to this tick (position replaced below)
  // [2026-10-31] Solved in the palette's cell, like the bare hands. In world floats x = -7284 has a 0.49 mm grid against 7.6 um in
  // the cell (WEAPON_TRACE), so only the gun and the hand pinned to it stepped on that grid, by location.
  const float origin[3]={rig_coherent?1024.f*cell[0]:0.f,rig_coherent?1024.f*cell[1]:0.f,rig_coherent?1024.f*cell[2]:0.f};
  for(int k=0;k<3;++k) root_tick[12+k]=body_now[k]-origin[k];
  float head[3]{};
  const bool sampled_head=(rig_coherent || ik_sync_head.load(std::memory_order_relaxed)) && smp.head_world_valid;
  bool have_head=false;
  if(shoulders_follow)
    have_head=sampled_head ? (std::memcpy(head,smp.head_world,sizeof(head)),true)
                             : native_stereo::head_offset(head);
  if(!have_head) head[0]=head[1]=head[2]=0.f;
  float anchor_tick[16]; std::memcpy(anchor_tick,root_tick,64);
  float placed_view[3]{};
  if(view_anchor) {  // Run 57's shared sample: gun and both hands use one stabilized view offset.
    // Re-reading the game camera here mixes two camera phases into vr_arms' filter while the gun is equipped.
    // The body root below still comes from this game tick, so ordinary locomotion is not delayed.
    if(!smp.view_valid) return false;
    if(!(smp.view_locked && vr_arms::stable_view_anchor(body_now,placed_view))) std::memcpy(placed_view,vr_arms::anchor_view_offset(smp),12);
    for(int k=0;k<3;++k) anchor_tick[12+k]=body_now[k]-origin[k]+placed_view[k];
    tick_view_offsets.fetch_add(1,std::memory_order_relaxed);
  }
  float basis[16]; vr_arms_math::basis_for_root(smp.basis,root_tick,basis);
  const float la=left_amount();
  float socket[16];
  if(la<=0.f) {if(!clean_socket(1,smp,root_tick,anchor_tick,head,basis,socket)) return false;}
  else if(la>=1.f) {if(!clean_socket(0,smp,root_tick,anchor_tick,head,basis,socket)) return false;}
  else {
    float s_right[16],s_left[16];
    if(!clean_socket(1,smp,root_tick,anchor_tick,head,basis,s_right) || !clean_socket(0,smp,root_tick,anchor_tick,head,basis,s_left)) return false;
    vr_arms_math::blend_affine(s_right,s_left,la,socket);
  }
  // run 67: two-hand aiming. Measure where the support hand is (for the grab decision) and, while held, swing the gun
  // about the dominant grip toward the support controller (minus the decaying grab offset: no snap).
  {const int dom=la<=0.f?1:(la>=1.f?0:-1);
    if(two_hand_enabled && dom>=0) {
      const int sup=1-dom;
      float sup_socket[16],wrist_sup[16];
      SupportHold sh; {std::lock_guard<std::mutex> lock(clean_mutex); sh=support_hold;}
      float h_sup[16],h_dom[16];
      if(holds_for(dom,sh,h_sup,h_dom) && clean_socket(sup,smp,root_tick,anchor_tick,head,basis,sup_socket,wrist_sup)) {
        float sup_pt[16],handle[16];
        skin_arm_ik::multiply_affine(sup_pt,h_sup,socket); skin_arm_ik::multiply_affine(handle,h_dom,socket);
        float off_w[3],d_s=0.f,d_h=0.f;
        for(int k=0;k<3;++k) {off_w[k]=wrist_sup[12+k]-sup_pt[12+k]; d_s+=off_w[k]*off_w[k]; const float e=wrist_sup[12+k]-handle[12+k]; d_h+=e*e;}
        float off_g[3];  // the offset in the gun's axes (rows of the socket)
        for(int r=0;r<3;++r) {const float l2=socket[r*4]*socket[r*4]+socket[r*4+1]*socket[r*4+1]+socket[r*4+2]*socket[r*4+2];
          off_g[r]=l2>1e-8f?(off_w[0]*socket[r*4]+off_w[1]*socket[r*4+1]+off_w[2]*socket[r*4+2])/l2:0.f;}
        float w=0.f,of=0.f; TwoHandState th;
        {std::lock_guard<std::mutex> lock(clean_mutex);
          th_d_support[sup]=std::sqrt(d_s); th_d_handle[sup]=std::sqrt(d_h); std::memcpy(th_offset_gun[sup],off_g,12); th_measured_us=us;
          w=two_hand_weight_locked(us); th=two_hand;
          of=th.held?1.f-vr_arms_math::smoothstep(static_cast<float>(us-th.start_us)/kTwoHandOffsetUs):0.f;}
        if(w>0.f && th.support==sup) {
          float o_w[3]; for(int k=0;k<3;++k) o_w[k]=(th.offset_gun[0]*socket[k]+th.offset_gun[1]*socket[4+k]+th.offset_gun[2]*socket[8+k])*of;
          const float pivot[3]={handle[12],handle[13],handle[14]};
          const float v_cur[3]={sup_pt[12]-pivot[0],sup_pt[13]-pivot[1],sup_pt[14]-pivot[2]};
          const float v_des[3]={wrist_sup[12]-o_w[0]-pivot[0],wrist_sup[13]-o_w[1]-pivot[1],wrist_sup[14]-o_w[2]-pivot[2]};
          const float lev=sh.leverage, deg=0.0174532925f;
          float rot[16];
          const float applied=vr_arms_math::swing_between(v_cur,v_des,vr_arms_math::two_hand_gain(lev)*w,(4.f-3.f*lev)*deg,std::max(12.f,60.f*lev)*deg,rot);
          float swung[16]; vr_arms_math::rotate_about(socket,pivot,rot,swung);
          bool fin=true; for(float v:swung) fin=fin&&std::isfinite(v);
          if(fin) std::memcpy(socket,swung,64);
          two_hand_deg_milli.store(static_cast<int>(applied*57295.78f),std::memory_order_relaxed);
        }
        two_hand_w_milli.store(static_cast<int>(w*1000.f),std::memory_order_relaxed);
        // run 68: the trigger hand's distance to the handle (re-grab), and the support-hand-only blend of the gun
        float dom_socket[16],wrist_dom[16];
        if(clean_socket(dom,smp,root_tick,anchor_tick,head,basis,dom_socket,wrist_dom)) {
          float handle_now[16]; skin_arm_ik::multiply_affine(handle_now,h_dom,socket);
          float e2=0.f; for(int k=0;k<3;++k) e2+=(wrist_dom[12+k]-handle_now[12+k])*(wrist_dom[12+k]-handle_now[12+k]);
          std::lock_guard<std::mutex> lock(clean_mutex); th_d_dom_handle=std::sqrt(e2);
        }
        float so=0.f; {std::lock_guard<std::mutex> lock(clean_mutex); so=support_only_locked(us);}
        if(so>0.f) {
          float inverse_h_sup[16],held_by_support[16],mixed[16];
          if(skin_arm_ik::inverse_affine(inverse_h_sup,h_sup)) {
            skin_arm_ik::multiply_affine(held_by_support,inverse_h_sup,wrist_sup);
            vr_arms_math::blend_affine(socket,held_by_support,so,mixed);
            bool fin=true; for(float v:mixed) fin=fin&&std::isfinite(v);
            if(fin) std::memcpy(socket,mixed,64);
          }
        }
      }
    }}
  // [LIVE run 54] world space, like the entity hook's matrix.
  for(int k=0;k<3;++k) flat_local[12+k]+=1024.f*cell[k];
  std::memcpy(flat,flat_local,64); std::memcpy(ik,socket,64);
  for(int k=0;k<3;++k) ik[12+k]=static_cast<float>(static_cast<double>(socket[12+k])+origin[k]);
  if(ik_trace_active.load(std::memory_order_acquire)) {
    GunRecord previous; bool have_previous=false;
    {std::lock_guard<std::mutex> lock(clean_mutex);
      if(gun_record.valid && us-gun_record.us>=0 && us-gun_record.us<100000) {
        previous=gun_record; have_previous=true;}}
    if(have_previous) {
      const float jump=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(ik),skin_arm_ik::position(previous.ik)));
      if(jump>0.03f) {
        const int dom=la>=1.f?0:1;
        float grip_trace=0.f,root_trace=0.f;
        for(int r=0;r<3;++r) for(int c=0;c<3;++c) {
          grip_trace+=smp.local[dom][r*4+c]*previous.sample.local[dom][r*4+c];
          root_trace+=root_tick[r*4+c]*previous.root[r*4+c];
        }
        IkTraceRow jump_row; jump_row.phase="place_jump"; jump_row.hand=dom; jump_row.sample=&smp;
        jump_row.gap=jump; jump_row.rotation_trace=grip_trace;
        jump_row.source_delta=skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(flat),skin_arm_ik::position(previous.flat)));
        jump_row.source_rotation_trace=root_trace;
        std::memcpy(jump_row.body,body_now,12); for(int k=0;k<3;++k) jump_row.shoulder[k]=anchor_tick[12+k]+origin[k];
        std::memcpy(jump_row.head_used,head,12);
        std::memcpy(jump_row.free_target,previous.ik+12,12);
        std::memcpy(jump_row.final_target,ik+12,12);
        std::memcpy(jump_row.object,flat+12,12);
        ik_trace(jump_row);
      }
    }
  }
  {std::lock_guard<std::mutex> lock(clean_mutex);
    gun_record.us=us; std::memcpy(gun_record.flat,flat,64); std::memcpy(gun_record.ik,ik,64);
    for(int k=0;k<3;++k) {gun_record.local[k]=socket[12+k]; gun_record.cell[k]=cell[k];} gun_record.has_local=rig_coherent;
    std::memcpy(gun_record.root,root_tick,64); std::memcpy(gun_record.view_offset,placed_view,12); gun_record.sample=smp; gun_record.valid=true;}
  IkTraceRow tr; tr.phase="place"; tr.hand=la>=1.f?0:1; tr.sample=&smp;
  std::memcpy(tr.body,body_now,12); for(int k=0;k<3;++k) tr.shoulder[k]=anchor_tick[12+k]+origin[k];
  std::memcpy(tr.head_used,head,12);
  std::memcpy(tr.free_target,flat+12,12); std::memcpy(tr.final_target,ik+12,12);
  ik_trace(tr);
  clean_gun_places.fetch_add(1,std::memory_order_relaxed);
  return true;
}
bool rig_coherent_on() {return rig_coherent;}
bool mounted_entity_local(const std::int16_t* entity_cell,float* local3) {
  if(!rig_coherent) return false;
  std::lock_guard<std::mutex> lock(mounted_gun_mutex);
  if(!mounted_gun_proof.pending_object) return false;
  for(int k=0;k<3;++k) local3[k]=mounted_gun_proof.pending_local[k]+1024.f*(ride_origin_cell[k]-entity_cell[k]);
  return true;
}
bool mounted_placed_pair(float (&flat)[16],float (&placed)[16]) {
  if(!rig_coherent || !mounted_gun_mode()) return false;
  std::lock_guard<std::mutex> lock(mounted_gun_mutex);
  const auto& proof=mounted_gun_proof;
  if(!proof.have_placed || !mounted_weapon_math::fresh(GetTickCount64(),proof.committed_at) || !mounted_weapon_math::rigid(proof.committed_flat)) return false;
  std::memcpy(flat,proof.committed_flat,64); std::memcpy(placed,proof.placed,64);
  for(int k=0;k<3;++k) flat[12+k]+=ride_origin[k];
  return true;
}
bool player_standing_still() {return rig_player_still.load(std::memory_order_relaxed);}
bool placed_socket_local(float* local3,std::int16_t* cell3) {
  std::lock_guard<std::mutex> lock(clean_mutex);
  if(!gun_record.valid || !gun_record.has_local || clean_qpc_us()-gun_record.us>3000) return false;
  std::memcpy(local3,gun_record.local,12); std::memcpy(cell3,gun_record.cell,6); return true;
}
bool weapon_attach_place(int hand, float (&flat)[16], float (&ik)[16]) {
  if(clean_rig && clean_place(hand,flat,ik)) return true;
  return weapon_attach_matrices(hand,flat,ik);
}
bool weapon_attach_matrices(int hand, float (&flat)[16], float (&ik)[16]) {
  if(mount_camera::rider_view()) return false; // shot/effect mapping needs its own mounted ownership proof
  if(hand<0 || hand>1 || attach_bone[hand]<0) return false;
  if(clean_rig) {  // readers (shot producer, camera post, present): the socket of this tick, never re-evaluated
    std::lock_guard<std::mutex> lock(clean_mutex);
    if(gun_record.valid && clean_qpc_us()-gun_record.us<200000) {std::memcpy(flat,gun_record.flat,64); std::memcpy(ik,gun_record.ik,64); return true;}
  }
  // [HEADSET 2026-10-15] (a) re-anchoring to the live body position put the gun ahead of the drawn hand while walking
  // (the drawn hands trail the body by the walk; the gun did not), and (b) only the skinned frame can be redrawn at
  // this frame's hand: the rigid parts (receiver, barrel, stock, sights, muzzle ring) stay where the game puts them, so
  // the gun split apart. The game placement now uses the hand exactly as drawn (no reroot) and predicts it one IK solve
  // ahead from its last change (translation and rotation, clamped to 5 cm / ~6 deg), covering the game-thread delay.
  std::lock_guard<std::mutex> lock(fp_last_mutex);
  if(!fp_last_at || GetTickCount64()-fp_last_at>200) return false;
  std::memcpy(flat,fp_raw_last[attach_bone[hand]],64); std::memcpy(ik,fp_out_last[attach_bone[hand]],64);
  {std::lock_guard<std::mutex> fresh_lock(fresh_mutex);  // run 21: the controller-driven socket, not the drawn one
    if(fresh_valid[hand] && fresh_cell[hand][0]==fp_cell_last[0] && fresh_cell[hand][1]==fp_cell_last[1] && fresh_cell[hand][2]==fp_cell_last[2]) {
      std::memcpy(ik,fresh_attach[hand],64);
      // [LIVE run 28 trace] running: the game camera bobs 15 cm (std) against the body, but the hand drawn on the gun
      // moved 11.7 cm (std) against the camera; and 6-11 cm single-frame spikes where the grips' camera differed from
      // the drawn one. The controllers are fixed to the head, so the socket must move rigidly with the game camera:
      // socket * inverse(camera the grips were built in) * camera now (position and rotation). This replaces the
      // body-travel trail fix; "Weapon hand trail fix" scales the translation part (rotation always follows).
      // Model-space hands (run 35): carry the drawn socket into this game frame with the body's own rigid motion,
      // socket * inverse(Marker of the drawn 211 pose) * Marker of this frame's 90 pose (pose index 1). Same bone name in
      // both skeletons, so no camera, yaw or convention guesses. Falls through to the camera-rigid path if unavailable.
      bool body_rigid=false;
      if(model_space_hands) {
        static int marker211=-2;
        if(marker211==-2) {marker211=-1; for(unsigned j=0;j<211;++j) if(fp_names[j]=="Marker") {marker211=static_cast<int>(j); break;}}
        const game::Pointers mp=game::pointers(); const unsigned char* data=nullptr; std::int32_t count=0; float mg[16];
        if(marker211>=0 && mp.pose && game::mem_read(mp.pose,&data,sizeof(data)) && game::mem_read(mp.pose+8,&count,sizeof(count)) && data &&
           count==90 && game::mem_read(data+1*64,mg,64)) {
          float md[16]; std::memcpy(md,fp_raw_last[marker211],64); for(int k=0;k<3;++k) md[12+k]+=1024.f*fp_cell_last[k];
          for(int k=0;k<3;++k) mg[12+k]+=std::round((md[12+k]-mg[12+k])/1024.f)*1024.f;
          float inverse_md[16],rel[16],ik_world[16],moved_w[16];
          float body_now[3]; const bool have_body=mp.body && game::mem_read(mp.body+0x94,body_now,sizeof(body_now));
          if(have_body && skin_arm_ik::inverse_affine(inverse_md,md)) {
            // [HEADSET run 36] carrying the socket with the whole Marker transform moved only the gun hand back and forward
            // while running (Marker = root-motion bone, it swings with the steps; the left hand is anchored to the mesh
            // root = body). Rotation from the Marker change (turning), translation exactly like the left hand: about the
            // drawn root, then from the drawn root's position to this frame's body position.
            skin_arm_ik::multiply_affine(rel,inverse_md,mg); rel[12]=rel[13]=rel[14]=0.f;
            std::memcpy(ik_world,ik,64); for(int k=0;k<3;++k) ik_world[12+k]+=1024.f*fp_cell_last[k];
            // run 52 (b): add the controller's own motion since the solve that produced this socket. Both controller poses
            // are taken in tracking space (grip * inverse(its camera)) and re-expressed in the solve's camera, so body
            // travel and turning (handled by the carry below) are not counted twice.
            if(shoulder_socket && fresh_grip_valid[hand]) {
              float g_now[16],c_now[16],inverse_c_now[16],t_now[16],g_now_then[16],inverse_g_then[16],w[16];
              if(native_stereo::grip_transform(hand,g_now) && native_stereo::grip_camera(c_now) &&
                 skin_arm_ik::inverse_affine(inverse_c_now,c_now) && skin_arm_ik::inverse_affine(inverse_g_then,fresh_grip[hand])) {
                skin_arm_ik::multiply_affine(t_now,g_now,inverse_c_now);               // controller in tracking space now
                skin_arm_ik::multiply_affine(g_now_then,t_now,fresh_grip_cam[hand]);   // ... seen through the solve's camera
                skin_arm_ik::multiply_affine(w,inverse_g_then,g_now_then);             // world delta at the solve
                float shift=0.f; for(int k=0;k<3;++k) shift+=(g_now_then[12+k]-fresh_grip[hand][12+k])*(g_now_then[12+k]-fresh_grip[hand][12+k]);
                const float c=(w[0]+w[5]+w[10]-1.f)*0.5f;
                bool ok=shift<0.06f*0.06f && c>0.966f; for(float v:w) ok=ok&&std::isfinite(v);
                if(ok) {float moved_c[16]; skin_arm_ik::multiply_affine(moved_c,ik_world,w); std::memcpy(ik_world,moved_c,64); controller_deltas.fetch_add(1,std::memory_order_relaxed);}
              }
            }
            // run 52 (a): pivot = the weapon-side upper arm (drawn 211 pose -> this tick's 90-pose), else the body root.
            float pivot_then[3],pivot_now[3]; bool shoulder_ok=false;
            {const unsigned upper=hand?113u:14u;
              for(int k=0;k<3;++k) pivot_then[k]=fp_raw_last[upper][12+k]+1024.f*fp_cell_last[k];
              if(shoulder_socket) {
                float pose90[90][16];
                const bool read90=game::mem_read(data,pose90,sizeof(pose90));
                const bool same_tick=read90 && std::fabs(md[12]-mg[12])+std::fabs(md[13]-mg[13])+std::fabs(md[14]-mg[14])<0.0005f;
                if(read90 && shoulder90[hand]<0 && same_tick) {  // find the bone once, on a frame where both poses are one tick
                  int best=-1; float best_d=1e9f;
                  for(int b=0;b<90;++b) {float d=0.f; for(int k=0;k<3;++k) {
                      const float v=pose90[b][12+k]+std::round((pivot_then[k]-pose90[b][12+k])/1024.f)*1024.f; d+=(v-pivot_then[k])*(v-pivot_then[k]);}
                    if(d<best_d) {best_d=d; best=b;}}
                  if(best>=0 && best_d<0.003f*0.003f) {
                    if(best==shoulder90_candidate[hand]) {if(++shoulder90_hits[hand]>=20) {shoulder90[hand]=best;
                        char line[160]; std::snprintf(line,sizeof(line),"gun hand: weapon-side upper arm = 90-pose bone %d (211 bone %u)",best,upper); log(line);}}
                    else {shoulder90_candidate[hand]=best; shoulder90_hits[hand]=1;}
                  }
                }
                if(read90 && shoulder90[hand]>=0) {
                  for(int k=0;k<3;++k) pivot_now[k]=pose90[shoulder90[hand]][12+k]+std::round((pivot_then[k]-pose90[shoulder90[hand]][12+k])/1024.f)*1024.f;
                  float jump=0.f; for(int k=0;k<3;++k) jump+=(pivot_now[k]-pivot_then[k])*(pivot_now[k]-pivot_then[k]);
                  shoulder_ok=jump<0.5f*0.5f;
                }
              }}
            float local_t[16]; std::memcpy(local_t,ik_world,64);
            for(int k=0;k<3;++k) local_t[12+k]-=shoulder_ok?pivot_then[k]:fresh_root[hand][12+k];
            skin_arm_ik::multiply_affine(moved_w,local_t,rel);
            // the FP mesh root sits exactly on the body position of its frame ([LIVE] player gear/body roots measured dist
            // 0.00), so this frame's root position is the body position now (fresh_body is a live render-thread read, not
            // the drawn frame's, so it is not used here). With the shoulder pivot the socket rides the arm's own bone.
            for(int k=0;k<3;++k) moved_w[12+k]+=shoulder_ok?pivot_now[k]:body_now[k];
            if(shoulder_ok) shoulder_carries.fetch_add(1,std::memory_order_relaxed);
            float jump=0.f; for(int k=0;k<3;++k) jump+=(moved_w[12+k]-ik_world[12+k])*(moved_w[12+k]-ik_world[12+k]);
            bool ok=jump<1.f; for(float v:moved_w) ok=ok&&std::isfinite(v);
            if(ok) {std::memcpy(ik,moved_w,64); for(int k=0;k<3;++k) ik[12+k]-=1024.f*fp_cell_last[k]; body_rigid=true; body_rigid_moves.fetch_add(1);}
            static std::atomic<unsigned> marker_logs{0};
            if(marker_logs.fetch_add(1)<3) {float tr=0.f; for(int r=0;r<3;++r) tr+=md[r*4]*mg[r*4]+md[r*4+1]*mg[r*4+1]+md[r*4+2]*mg[r*4+2];
              char line[220]; std::snprintf(line,sizeof(line),"weapon socket body-rigid: Marker211=%d drawn vs game Marker move %.3f m rotation trace %.3f",marker211,std::sqrt(jump),tr); log(line);}
          }
        }
      }
      game::Player cp=game::read();
      if(!body_rigid) {float body_now[3]; const game::Pointers bp=game::pointers(); std::lock_guard<std::mutex> cam_lock(cam_post_mutex);
        if(cam_post_valid && bp.body && game::mem_read(bp.body+0x94,body_now,sizeof(body_now))) {
          float travel2=0.f; for(int k=0;k<3;++k) travel2+=(body_now[k]-cam_post_body[k])*(body_now[k]-cam_post_body[k]);
          if(travel2<1.f) {std::memcpy(cp.cam,cam_post,64); for(int k=0;k<3;++k) cp.cam[12+k]+=body_now[k]-cam_post_body[k];
            cp.valid=true; cam_estimates.fetch_add(1);}
        }}
      if(!body_rigid && fresh_cam_valid[hand] && cp.valid && weapon_trail_comp>0.f) {
        float ik_world[16]; std::memcpy(ik_world,ik,64); for(int k=0;k<3;++k) ik_world[12+k]+=1024.f*fp_cell_last[k];
        float inverse_cam[16],rel[16],rigid[16];
        if(skin_arm_ik::inverse_affine(inverse_cam,fresh_cam[hand])) {
          skin_arm_ik::multiply_affine(rel,inverse_cam,cp.cam);
          skin_arm_ik::multiply_affine(rigid,ik_world,rel);
          float jump=0.f; for(int k=0;k<3;++k) jump+=(rigid[12+k]-ik_world[12+k])*(rigid[12+k]-ik_world[12+k]);
          bool ok=jump<1.f; for(float v:rigid) ok=ok&&std::isfinite(v);
          if(ok) {
            for(int i=0;i<12;++i) ik[i]=rigid[i];
            for(int k=0;k<3;++k) ik[12+k]=ik_world[12+k]+weapon_trail_comp*(rigid[12+k]-ik_world[12+k])-1024.f*fp_cell_last[k];
          }
          static std::atomic<unsigned> cam_logs{0};  // once: do the two camera sources share axes?
          if(cam_logs.fetch_add(1)<3) {float d=0.f; for(int r=0;r<3;++r) d+=fresh_cam[hand][r*4]*cp.cam[r*4]+fresh_cam[hand][r*4+1]*cp.cam[r*4+1]+fresh_cam[hand][r*4+2]*cp.cam[r*4+2];
            char line[200]; std::snprintf(line,sizeof(line),"weapon socket camera-rigid: grip camera vs game camera rotation trace %.3f (3 = same axes), move %.3f m",d,std::sqrt(jump)); log(line);}
        }
      }
    }}
  static float prev_ik[2][16]; static unsigned prev_gen[2]={0,0}; static float last_ik[2][16]; static unsigned last_gen[2]={0,0};
  if(fp_last_gen!=last_gen[hand]) {std::memcpy(prev_ik[hand],last_ik[hand],64); prev_gen[hand]=last_gen[hand];
    std::memcpy(last_ik[hand],ik,64); last_gen[hand]=fp_last_gen;}
  if(weapon_predict && prev_gen[hand] && last_gen[hand]==prev_gen[hand]+1) {
    float inverse_prev[16],step[16],predicted[16];
    if(skin_arm_ik::inverse_affine(inverse_prev,prev_ik[hand])) {
      skin_arm_ik::multiply_affine(step,inverse_prev,last_ik[hand]);  // last = prev * step  ->  predicted = last * step
      const float move=std::sqrt(step[12]*step[12]+step[13]*step[13]+step[14]*step[14]);
      const float trace=step[0]+step[5]+step[10];
      if(std::isfinite(move) && move<0.05f && trace>2.98f) {
        skin_arm_ik::multiply_affine(predicted,last_ik[hand],step); std::memcpy(ik,predicted,64);}
    }
  }
  for(int k=0;k<3;++k) {flat[12+k]+=1024.f*fp_cell_last[k]; ik[12+k]+=1024.f*fp_cell_last[k];}
  return true;
}
float* shoulder_spread_value() { return &shoulder_spread; }
float* hand_separation_value() { return &hand_tune.side; }
float* shoulder_height_value() { return &shoulder_height; }
float* shoulder_forward_value() { return &shoulder_forward; }
void recapture_rest() { rest_recapture.store(true); }
void on_present() {
  hand_gestures::on_present();
  const unsigned completed_gen=frame_gen.fetch_add(1);
  vr_arms::note_present();
  rider_tune_poll();
  if(!ik_trace_done.load(std::memory_order_acquire) && !ik_trace_requested.load(std::memory_order_relaxed) && !dir.empty() &&
     GetTickCount64()>=ik_trace_control_next) {
    ik_trace_control_next=GetTickCount64()+1000;
    char control[96]{};
    if(settings_file::read(dir+L"\\AvatarVR_iktrace.txt",control,sizeof(control))) {
      char word[32]{};
      char option[32]{};
      const int words=std::sscanf(control,"%31s %31s",word,option);
      if(words>=1 && std::strcmp(word,"capture")==0) {
        ik_sync_head.store(words>=2 && std::strcmp(option,"sync_head")==0,std::memory_order_relaxed);
        ik_trace_weapon_start.store(words>=2 && (std::strcmp(option,"weapon")==0 ||
                                    std::strcmp(option,"weapon_lock")==0),std::memory_order_relaxed);
        ik_frame_sample_lock.store(words>=2 && std::strcmp(option,"weapon_lock")==0,std::memory_order_relaxed);
        ik_trace_requested.store(true,std::memory_order_release);
      }
    }
  }
  if(ik_trace_active.load(std::memory_order_acquire)) {
    IkTraceRow trace_present; trace_present.phase="present"; trace_present.gen=completed_gen;
    head_pose::Snapshot displayed;
    if(native_stereo::presented_views(&displayed)) {
      trace_present.xr_seq=displayed.seq; trace_present.display_time=displayed.display_time;
    }
    ik_trace(trace_present);
  }
  if(!dir.empty()) persist_settings(GetTickCount64());
  mounted_weapon_enabled.store(mounted_weapon_menu && *vr_weapon::effects_flag(),std::memory_order_relaxed);
  // Bounded ownership audit for the three mounted regressions. Render/game
  // callbacks publish counters only; Present owns these at most 60 log records.
  static ULONGLONG mounted_audit_next=0;
  static unsigned mounted_audit_records=0;
  const auto mounted_audit_now=GetTickCount64();
  if(!dir.empty()&&mount_camera::rider_view()&&mounted_audit_records<60&&mounted_audit_now>=mounted_audit_next) {
      // Zero attempts is itself evidence: do not suppress a missing-hook run.
      mounted_audit_next=mounted_audit_now+2000;++mounted_audit_records;
      const auto state=weapon_capture_state();
      char line[1600];
      std::snprintf(line,sizeof(line),"mounted equip audit n=%u enabled=%d menu=%d effects=%d writes=%d controllers=%d phase=%d profiles=%u disk=%u bound=%d bind_age=%llu committed=%d commit_age=%llu hold=%d onfoot=%d support=%d aim=%d two_hand_enabled=%d held=%d support_hand=%d measure_age_us=%lld palette=%u/%u/%u/%u",
        mounted_audit_records,mounted_weapon_enabled.load()?1:0,mounted_weapon_menu?1:0,*vr_weapon::effects_flag()?1:0,
        pulse_permission.load()?1:0,controller_mode.load()?1:0,int(mount_state::get().phase),state.mounted_profiles,state.mounted_disk_profiles,
        state.mounted_bound?1:0,static_cast<unsigned long long>(state.mounted_bind_age_ms),state.mounted_committed?1:0,
        static_cast<unsigned long long>(state.mounted_commit_age_ms),state.mounted_grip_profile?1:0,state.mounted_onfoot_hold?1:0,
        state.mounted_support_profile?1:0,state.mounted_aim_profile?1:0,two_hand_enabled?1:0,state.two_hand_held?1:0,state.support_hand,
        static_cast<long long>(state.measure_age_us),state.mounted_palette_adopted,state.mounted_palette_unbound,
        state.mounted_palette_not_ready,state.mounted_palette_invalid);
      std::string detail=line;
      char contact[400];
      std::snprintf(contact,sizeof(contact)," finger=%d,%d finger_apps=%u finger_rejects=%u pin_expired=%u pin_distant=%u support_dist=%.4f,%.4f handle_dist=%.4f,%.4f candidate=%p resource=%p weapon_path=",
        state.mounted_finger_profile[0],state.mounted_finger_profile[1],state.mounted_finger_apps,state.mounted_finger_rejects,
        state.mounted_pin_expired,state.mounted_pin_distant,state.support_distance[0],state.support_distance[1],
        state.handle_distance[0],state.handle_distance[1],state.mounted_candidate,state.mounted_resource);
      detail+=contact; detail+=state.mounted_weapon_path; log(detail.c_str());
      std::string reasons="mounted equip attempts";
      for(unsigned i=0;i<mounted_attempt_count;++i) reasons+=" "+std::string(mounted_attempt_names[i])+"="+std::to_string(mounted_attempts[i].load(std::memory_order_relaxed));
      reasons+=" held_last="+std::to_string(mounted_held_last.load(std::memory_order_relaxed))+" arm_why(palm,ok,weapon_ok,input,rest,rigid)=";
      for(auto& why:mounted_arm_why) reasons+=std::to_string(why.load(std::memory_order_relaxed))+",";
      reasons+=" bound_carried="+std::to_string(mounted_bound_carried.load(std::memory_order_relaxed))+" palette_no_frame="+std::to_string(mounted_palette_no_frame.load(std::memory_order_relaxed));
      log(reasons.c_str());
      // mounted_parity: frames latched, commits that replaced a frame's choice, draws whose own choice differed from the frame's, and the
      // largest body change within a frame and live-palm-vs-commit gun gap in this window (what the old code drew apart).
      char parity[480];
      std::snprintf(parity,sizeof(parity),"rider parity: on=%d frames=%u grip_holds=%u gun_input=%u (held %u) ahead=%u relatched=%u input_splits=%u body_drift_max=%u mm palm_vs_commit_max=%u mm",
        rig_coherent&&mounted_parity?1:0,rider_latches.load(std::memory_order_relaxed),rider_grip_holds.load(std::memory_order_relaxed),rider_commit_latches.load(std::memory_order_relaxed),rider_commit_held.load(std::memory_order_relaxed),rider_prelatches.load(std::memory_order_relaxed),rider_relatches.load(std::memory_order_relaxed),
        rider_input_splits.load(std::memory_order_relaxed),rider_body_drift_mm.exchange(0,std::memory_order_relaxed),
        rider_palm_commit_mm.exchange(0,std::memory_order_relaxed));
      log(parity);
      std::snprintf(parity,sizeof(parity),"rider moments: meshes_off_solve_tick=%u mesh_moment_max=%u mm gun_rooted=%u gun_unrooted=%u gun_hand_follows_mesh=%d",
        rider_mesh_moments.load(std::memory_order_relaxed),rider_mesh_moment_mm.exchange(0,std::memory_order_relaxed),
        rider_gun_rooted.load(std::memory_order_relaxed),rider_gun_unrooted.load(std::memory_order_relaxed),
        rider_gun_follows_mesh.load(std::memory_order_relaxed)?1:0);
      log(parity);
      std::snprintf(parity,sizeof(parity),"rider tick solve: on=%d made=%u (exact %u provisional %u predicted %u turned %u max %.2f deg refreshed %u frozen_draws %u) hits=%u newest_tick_hits=%u standing=%u gun_made=%u gun_shared_with_draws=%u gun_hand_led=%u gun_hand_far=%u",
        rider_tick_mode()?1:0,rider_tick_made.load(std::memory_order_relaxed),rider_tick_exact.load(std::memory_order_relaxed),
        rider_tick_provisional.load(std::memory_order_relaxed),rider_tick_predicted.load(std::memory_order_relaxed),rider_tick_turned.load(std::memory_order_relaxed),rider_tick_turn_cdeg.exchange(0,std::memory_order_relaxed)*0.01f,rider_tick_refreshed.load(std::memory_order_relaxed),rider_tick_frozen.load(std::memory_order_relaxed),rider_tick_hits.load(std::memory_order_relaxed),
        rider_tick_newest.load(std::memory_order_relaxed),rider_tick_stationary.load(std::memory_order_relaxed),
        rider_tick_gun_made.load(std::memory_order_relaxed),rider_tick_gun_shared.load(std::memory_order_relaxed),
        rider_gun_hand_led.load(std::memory_order_relaxed),rider_gun_hand_far.load(std::memory_order_relaxed));
      log(parity);
      std::snprintf(parity,sizeof(parity),"rider ticks: matched=%u unmatched=%u mesh_yaw_max=%.2f deg",
        rider_tick_matched.load(std::memory_order_relaxed),rider_tick_unmatched.load(std::memory_order_relaxed),
        rider_mesh_yaw_cdeg.exchange(0,std::memory_order_relaxed)*0.01f);
      log(parity);
      std::snprintf(parity,sizeof(parity),"rider items: applied=%u misses=%u",rider_item_apps.load(std::memory_order_relaxed),rider_item_misses.load(std::memory_order_relaxed));
      log(parity);
  }
  static bool was_mounted=false;
  const bool is_mounted=mount_camera::rider_view();
  if(is_mounted!=was_mounted) {
    was_mounted=is_mounted;
    std::lock_guard<std::mutex> lock(clean_mutex);
    two_hand=TwoHandState{}; th_measured_us=0; support_hold=SupportHold{}; support_hold_us=0;
    set_support_only_locked(false,clean_qpc_us());
  }
  pulse_permission.store(*lua_host::writes_flag()); // UI flag is read only on its Present thread.
  xr_input::set_left_handed(left_handed);
  if(controller_mode.load() && pulse_until.load()) {
    GripCache fresh; fresh.tick=GetTickCount64();
    for(unsigned hand=0;hand<2;++hand) fresh.valid[hand]=native_stereo::grip_transform(static_cast<int>(hand),fresh.matrix[hand]);
    fresh.body_valid=native_stereo::grip_body(fresh.body);
    fresh.head_valid=native_stereo::head_position(fresh.head);
    fresh.yaw_valid=native_stereo::grip_yaw(&fresh.yaw);
    fresh.cam_valid=native_stereo::grip_camera(fresh.cam);
    {std::lock_guard<std::mutex> lock(grip_mutex); grips=fresh;}
    {std::lock_guard<std::mutex> lock(wrist_mutex);
      for(unsigned hand=0;hand<2;++hand) if(!fresh.valid[hand]) wrist_calibration[hand].valid=false;}
  }
  if(dir.empty()) {
    HMODULE self=nullptr; wchar_t path[MAX_PATH]{};
    if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
       reinterpret_cast<LPCWSTR>(&on_present),&self) || !GetModuleFileNameW(self,path,MAX_PATH)) return;
    dir=path; dir=dir.substr(0,dir.find_last_of(L"\\/"));
  }
  mounted_profile_io();
  const auto now=GetTickCount64(); if(now-last_poll<1000) return; last_poll=now;
  char control[16]{};
  HANDLE f=CreateFileW((dir+L"\\AvatarVR_skinpalette.txt").c_str(),GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,0,nullptr);
  if(f!=INVALID_HANDLE_VALUE) {DWORD n; ReadFile(f,control,sizeof(control)-1,&n,nullptr); CloseHandle(f);}
  const bool wanted=std::strncmp(control,"on",2)==0;
  const bool valid=game::read().valid;
  char pulse_control[16]{};
  HANDLE pulse_file=CreateFileW((dir+L"\\AvatarVR_skinpulse.txt").c_str(),GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,0,nullptr);
  if(pulse_file!=INVALID_HANDLE_VALUE) {DWORD bytes; ReadFile(pulse_file,pulse_control,sizeof(pulse_control)-1,&bytes,nullptr); CloseHandle(pulse_file);}
  const bool pulse_wanted=std::strncmp(pulse_control,"on",2)==0;
  char ik_control[16]{};
  settings_file::read(dir+L"\\AvatarVR_skinik.txt",ik_control,sizeof(ik_control));
  const bool ik_wanted=std::strncmp(ik_control,"on",2)==0||ik_menu;
  const bool any_wanted=pulse_wanted||ik_wanted;
  // An "on" control stays armed until gameplay and Script writes are ready; only a real activation consumes it.
  static bool waiting_logged=false;
  if(any_wanted && !pulse_wanted_before && !(pulse_wanted&&ik_wanted) && !(valid && pulse_permission.load()) && !waiting_logged) {
    char waiting[140];
    std::snprintf(waiting,sizeof(waiting),"skin %s armed: waiting for gameplay=%d and Script writes=%d",
      ik_wanted?"ik":"pulse",valid?1:0,pulse_permission.load()?1:0);
    log(waiting); waiting_logged=true;
  }
  if(!any_wanted) waiting_logged=false;
  if(any_wanted && !pulse_wanted_before && !(pulse_wanted&&ik_wanted) && valid && pulse_permission.load()) {
    if(!tried) {tried=true; installed=install();}
    pulse_wanted_before=true; waiting_logged=false;
    if(installed) {
      pulse_body.store(game::pointers().body); pulse_mesh.store(nullptr); pulse_calls.store(0);
      fp_source.store(nullptr); fp_body_apps.store(0); accessory_apps.store(0); other211.store(0); other211_logged.store(false);
      {std::lock_guard<std::mutex> lock(accessory_mutex); for(auto& m:accessory_maps) m=AccessoryMap{};} accessory_logged.store(0); accessory_meshes.store(0);
      {std::lock_guard<std::mutex> lock(seen_paths_mutex); seen_paths.clear();}
      {std::lock_guard<std::mutex> lock(far_mutex); far_meshes.clear();}
      {std::lock_guard<std::mutex> lock(census_mutex); census_seen.clear();} census_lines.store(0);
      {std::lock_guard<std::mutex> lock(fp_last_mutex); fp_last_at=0;}
      controller_mode.store(ik_wanted);
      follow_enabled.store(std::strstr(ik_control,"nofollow")==nullptr); follow_logged.store(0);
      if(ik_wanted) log(follow_enabled.load()?"skin ik followers: ON (fingers/twist/props ride their solved joint)":"skin ik followers: OFF (nofollow)");
      {std::lock_guard<std::mutex> lock(wrist_mutex);for(auto& calibration:wrist_calibration) calibration.valid=false;}
      // Controller IK runs until switched off (menu or control file); the diagnostic pulse keeps its 20 s window.
      pulse_started.store(now); pulse_until.store(ik_wanted?~0ull:now+20000);
      if(ik_wanted) log("FIRST-PERSON CONTROLLER IK build " __DATE__ " " __TIME__ ";until switched off; arm joints + followers; source untouched");
      else log("FIRST-PERSON PALETTE PULSE build " __DATE__ " " __TIME__ ";20 seconds; private copy right113/114/115 Y; source untouched");
    }
  }
  if(!any_wanted) pulse_wanted_before=false;
  // Unlimited IK that stopped while still wanted (player body changed on a load, Script writes off) re-arms itself.
  if(any_wanted && pulse_wanted_before && controller_mode.load() && pulse_until.load()==0) pulse_wanted_before=false;
  if(!any_wanted || (pulse_wanted&&ik_wanted) || !valid || !pulse_permission.load() ||
     (controller_mode.load()? !ik_wanted:!pulse_wanted)) pulse_until.store(0);
  const auto pulse_end=pulse_until.load();
  static ULONGLONG status_next=0;
  if(pulse_end && now<pulse_end && (!controller_mode.load() || now>=status_next)) {
    status_next=now+5000; // IK runs without a time limit: status every 5 s instead of every poll
    char result[420]; const auto age=static_cast<unsigned>(now-pulse_started.load());
    bool left=false,right=false;
    if(controller_mode.load()){std::lock_guard<std::mutex> lock(grip_mutex);left=grips.valid[0];right=grips.valid[1];}
    float head[3]{}; const bool head_ok=native_stereo::head_offset(head);
    {static ULONGLONG rig_next=0; const ULONGLONG rig_now=GetTickCount64();
      if(rig_now>=rig_next) {char flat_line[160];
        std::snprintf(flat_line,sizeof(flat_line),"rig flat-pose exits: fp_root_far=%u gear_far=%u pose_changed=%u solve_failed=%u body_behind_max=%u mm",
          rig_fallback[0].load(),rig_fallback[1].load(),rig_fallback[2].load(),rig_fallback[3].load(),rig_stale_mm_max.load());
        log(flat_line);}
      if(rig_now>=rig_next) {rig_next=rig_now+5000; char rig_line[240];
        std::snprintf(rig_line,sizeof(rig_line),"rig audit: coherent=%d holds=%u gun_holds=%u align_lost=%u basis_rejects=%u body_stale=%u far_skips=%u bound_moves=%u twist_unwraps=%u arm_antipodal=%u gear_rigid=%d",
          rig_coherent?1:0,hand_holds.load(),rig_gun_holds.load(),rig_align_fails.load(),vr_arms::basis_rejects(),rig_body_stale.load(),vr_weapon::rig_far_skips(),vr_weapon::rig_bound_moves(),rig_twist_wraps.load(),rig_antipodal.load(),accessory_rigid?1:0);
        log(rig_line);}}
    std::snprintf(result,sizeof(result),"palette override mode=%s age_ms=%u delta_y=%.2f applications=%u fp_body=%u accessory=%u weapon=%u wpose=%u/%u grip=%u/%u hand_on_gun=%u/%u gun_frame=%u/%u tick_resolve=%u root_carry=%u shoulder_carry=%u ctrl_delta=%u clean=%d solves=%u gun_place=%u/%u rig_reuse=%u flat_frames=%u hand_jump_still=%u inputs_mismatch=%u cm_fallback=%u tick_view=%u weapon_hand=%d finger_swaps=%u left_pins=%u hand_swaps=%u two_hand=%u/%u w=%.2f lev=%.2f swing=%.1f support_only=%u/%u fp_vs_gun=%u/%u rig_shifts=%u live_grips=%u frame_cam=%u/%u model_hands=%u body_rigid=%u posewrite=%u/%u/%u acc_meshes=%u other211=%u%s%s "
      "mesh=%p grips=%d,%d head=%d %.3f,%.3f,%.3f shoulders=%d scale=%.2f,%.2f user_arm=%.2f rest=%d captures=%u shoulder_h=%.2f shoulder_f=%.2f tune pitch=%.0f yaw=%.0f roll=%.0f fwd=%.2f up=%.2f side=%.2f",
      controller_mode.load()?"controller_ik":"pulse",age,controller_mode.load()?0.f:skin_palette_pulse::offset(age),
      pulse_calls.load(),fp_body_apps.load(),accessory_apps.load(),weapon_apps.load(),weapon_pose_writes.load(),weapon_pose_reuses.load(),grip_calibrations.load(),grip_redraws.load(),hand_on_gun.load(),hand_on_gun_root.load(),gun_same_frame.load(),gun_other_frame.load(),tick_resolves.load(),root_carries.load(),shoulder_carries.load(),controller_deltas.load(),clean_rig?1:0,clean_solves.load(),clean_gun_places.load(),clean_gun_reuses.load(),rig_reuse.load(),flat_frames.load(),hand_jump_still.load(),inputs_mismatch.load(),cm_fallbacks.load(),tick_view_offsets.load(),left_handed?0:1,finger_swaps.load(),left_pins.load(),hand_swaps.load(),two_hand_grabs.load(),two_hand_pins.load(),two_hand_w_milli.load()*0.001f,two_hand_lev_milli.load()*0.001f,two_hand_deg_milli.load()*0.001f,support_only_entries.load(),support_only_regrabs.load(),fp_after_gun.load(),fp_before_gun.load(),rig_shifts.load(),live_grip_reads.load(),frame_cam_calibrations.load(),frame_cam_applied.load(),model_hand_solves.load(),body_rigid_moves.load(),posewrite_frames.load(),posewrite_bones90.load(),posewrite_bones192.load(),accessory_meshes.load(),other211.load(),other211_logged.load()?" other_path=":"",
      other211_logged.load()?other211_path:"",pulse_mesh.load(),left,right,head_ok?1:0,head[0],head[1],head[2],shoulders_follow?1:0,scale_milli[0].load()*0.001f,scale_milli[1].load()*0.001f,arm_scale.user_length,have_rest?1:0,rest_captures.load(),shoulder_height,shoulder_forward,
      hand_tune.pitch,hand_tune.yaw,hand_tune.roll,hand_tune.forward,hand_tune.up,hand_tune.side); log(result);
  }
  if(pulse_end && now>=pulse_end) {
    pulse_until.store(0); char result[180];
    std::snprintf(result,sizeof(result),"palette pulse complete: applications=%u mesh=%p",pulse_calls.load(),pulse_mesh.load()); log(result);
  }
  if(wanted && !wanted_before && valid) {
    if(!tried) {tried=true; installed=install();}
    if(installed) {
      std::lock_guard<std::mutex> lock(mutex); total=0; pending=0; source_count=0; next_sample=0; calls.store(0);
      window_body.store(game::pointers().body);
      deadline.store(now+20000);
      log("skin palette build " __DATE__ " " __TIME__ ";20 second window;64 distinct source maximum; no pose writes");
    }
  }
  wanted_before=wanted; if(!wanted || !valid) deadline.store(0);
  Sample buffer[64]; unsigned n=0;
  {std::lock_guard<std::mutex> lock(mutex); n=pending; std::memcpy(buffer,samples,n*sizeof(Sample)); pending=0;}
  char line[1600];
  for(unsigned i=0;i<n;++i) {
    const auto& s=buffer[i];
    std::snprintf(line,sizeof(line),"mesh=%p header=%p data=%p count=%u remap=%p candidate_map_count=%u read_flags=%u name_count=%u bone_name=%s body=%p cpu90=%p cpu192=%p same90=%d same192=%d thread=%u cells=%d,%d,%d wrist_or_root=(%.6f %.6f %.6f) firstmap=%u",
      s.mesh,s.header,s.data,s.count,s.remap,s.map_count,s.read_flags,s.name_count,s.bone_name,s.body,s.cpu90,s.cpu192,s.data==s.cpu90,s.data==s.cpu192,s.thread,
      s.cells[0],s.cells[1],s.cells[2],s.hand[0],s.hand[1],s.hand[2],s.indices[0]); log(line);
    {
      std::snprintf(line,sizeof(line),"armmap mesh=%p data=%p L=%d,%d,%d R=%d,%d,%d matrix_mask=%u Lpos=(%.5f %.5f %.5f)/(%.5f %.5f %.5f)/(%.5f %.5f %.5f) Rpos=(%.5f %.5f %.5f)/(%.5f %.5f %.5f)/(%.5f %.5f %.5f) candidate58=%p candidate58_flags=%u candidate58_words=%llX,%llX",
        s.mesh,s.data,s.arms[0],s.arms[1],s.arms[2],s.arms[3],s.arms[4],s.arms[5],s.arm_read_mask,
        s.arm_positions[0][0],s.arm_positions[0][1],s.arm_positions[0][2],s.arm_positions[1][0],s.arm_positions[1][1],s.arm_positions[1][2],s.arm_positions[2][0],s.arm_positions[2][1],s.arm_positions[2][2],
        s.arm_positions[3][0],s.arm_positions[3][1],s.arm_positions[3][2],s.arm_positions[4][0],s.arm_positions[4][1],s.arm_positions[4][2],s.arm_positions[5][0],s.arm_positions[5][1],s.arm_positions[5][2],
        s.parent_candidate,s.parent_read,static_cast<unsigned long long>(s.parent_words[0]),static_cast<unsigned long long>(s.parent_words[1])); log(line);
    }
  }
  const auto until=deadline.load();
  if(until && now>=until) {
    deadline.store(0); unsigned captured;
    {std::lock_guard<std::mutex> lock(mutex); captured=total;}
    std::snprintf(line,sizeof(line),"window complete: scoped_calls=%u samples=%u",calls.load(),captured); log(line);
  }
}
}
