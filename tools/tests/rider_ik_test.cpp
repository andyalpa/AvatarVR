#include "../../src/diagnostics/rider_ik.hpp"
#include "../../src/diagnostics/mounted_accessory_bounds.hpp"
#include <cmath>
#include <cstdio>

using namespace rider_ik;
using skin_arm_ik::Vec;

namespace {
float dist(Vec a, Vec b) { return skin_arm_ik::length(skin_arm_ik::sub(a, b)); }
void place(float (*pose)[16], int i, float x, float y, float z) {
  float* m = pose[i];
  for (int k = 0; k < 16; ++k) m[k] = (k % 5 == 0) ? 1.f : 0.f;  // identity rows
  m[12] = x; m[13] = y; m[14] = z;
}
}  // namespace

int main() {
  // Roles from names.
  int side = -2;
  if (role("L_upperArm", side) != kUpper || side != 0) return 1;
  if (role("R_upperArm_twist01", side) != kUpper || side != 1) return 2;
  if (role("R_lowerArm_twist03", side) != kLower) return 3;
  if (role("L_fingerIndex_02", side) != kHand || role("R_palm", side) != kHand || role("R_handWeapon", side) != kHand ||
      role("L_weaponAttachmentHand", side) != kHand) return 4;
  if (role("L_shoulder", side) != kShoulder || side != 0) return 5;
  if (role("L_hip", side) != kNone || side != -1) return 7;
  if (role("L_upperLeg", side) != kNone || role("C_head", side) != kNone || role(nullptr, side) != kNone) return 6;

  // A riding pose: right arm hanging down from a shoulder at (0.2, 2.0, 0); forward = +z, up = +y.
  static float pose[kPoseBones][16];
  for (int i = 0; i < kPoseBones; ++i) place(pose, i, 0.f, 1.f, 0.f);
  const ArmBones& r = kArm[1];
  place(pose, r.upper, 0.2f, 2.0f, 0.f);
  place(pose, r.lower, 0.2f, 1.7f, 0.f);
  place(pose, r.hand, 0.2f, 1.4f, 0.f);
  place(pose, r.middle, 0.2f, 1.3f, 0.f);   // knuckles below the wrist: hand points down
  place(pose, r.index, 0.21f, 1.3f, 0.02f);
  place(pose, r.pinky, 0.21f, 1.3f, -0.02f);
  const float basis[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  // Controller grip held forward at chest height: rows right, up, forward, position.
  const float grip[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0.25f,1.8f,0.4f,1};
  skin_arm_ik::HandTune tune;
  Arm arm;
  Vec bend{};
  if (!solve(pose, 1, grip, basis, tune, arm, &bend)) return 10;
  // The solved wrist sits palm*reach behind the grip along the grip forward (align_hand), elbow keeps the bone lengths.
  const Vec s{0.2f, 2.0f, 0.f};
  if (std::fabs(dist(arm.elbow, s) - 0.3f) > 1e-3f || std::fabs(dist(arm.wrist, arm.elbow) - 0.3f) > 1e-3f) return 11;
  const Vec want{0.25f, 1.8f, 0.4f - tune.palm * 0.1f};
  if (dist(arm.wrist, want) > 1e-3f || arm.clamped) { std::printf("wrist %f\n", dist(arm.wrist, want)); return 12; }
  if (skin_arm_ik::length(bend) < 0.5f) return 13;
  // Applying the deltas moves the joints onto the solution; a finger stays rigid with the hand.
  float up[16], lo[16], ha[16], mid[16];
  std::memcpy(up, pose[r.upper], 64); std::memcpy(lo, pose[r.lower], 64); std::memcpy(ha, pose[r.hand], 64);
  std::memcpy(mid, pose[r.middle], 64);
  if (!apply(up, arm.delta[kUpper]) || !apply(lo, arm.delta[kLower]) || !apply(ha, arm.delta[kHand]) ||
      !apply(mid, arm.delta[kHand])) return 14;
  if (dist(skin_arm_ik::position(up), s) > 1e-4f || dist(skin_arm_ik::position(lo), arm.elbow) > 1e-4f ||
      dist(skin_arm_ik::position(ha), arm.wrist) > 1e-4f) return 15;
  if (std::fabs(dist(skin_arm_ik::position(mid), skin_arm_ik::position(ha)) - 0.1f) > 1e-3f) return 16;
  // Knuckles now point along the grip forward (+z).
  const Vec f = skin_arm_ik::sub(skin_arm_ik::position(mid), skin_arm_ik::position(ha));
  if (f.z < 0.09f) { std::printf("finger dir %f %f %f\n", f.x, f.y, f.z); return 17; }
  // Out of reach: clamped, still finite.
  const float far_grip[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0.2f,2.0f,3.f,1};
  if (!solve(pose, 1, far_grip, basis, tune, arm) || !arm.clamped) return 18;
  // Bad grip basis: refused.
  const float bad[16] = {};
  if (solve(pose, 1, bad, basis, tune, arm)) return 19;
  // A committed weapon wrist bypasses controller offsets exactly once, while
  // preserving segment lengths. The free solve remains unchanged.
  {
    const Joints j=pose_joints(pose,1,false);
    float held[16]; std::memcpy(held,basis,64); held[12]=.25f; held[13]=1.8f; held[14]=.3f;
    Tune tuning; tuning.reach_scale=1.5f;
    Arm pinned,free_a,free_b;
    if(!solve_joints(j,1,grip,basis,tuning,pinned,nullptr,held) || pinned.clamped ||
       dist(pinned.wrist,skin_arm_ik::position(held))>1e-4f ||
       std::fabs(dist(pinned.elbow,s)-.3f)>1e-4f || std::fabs(dist(pinned.wrist,pinned.elbow)-.3f)>1e-4f) return 181;
    float wrist[16]; skin_arm_ik::multiply_affine(wrist,j.hand,pinned.delta[kHand]);
    for(int k=0;k<16;++k) if(std::fabs(wrist[k]-held[k])>1e-4f) return 182;
    if(!solve_joints(j,1,grip,basis,tuning,free_a) || !solve_joints(j,1,grip,basis,tuning,free_b,nullptr,nullptr) ||
       std::memcmp(free_a.delta,free_b.delta,sizeof(free_a.delta))) return 183;
    held[0]=NAN; if(solve_joints(j,1,grip,basis,tuning,pinned,nullptr,held)) return 184;
  }

  // Pelvis frame helpers and the grip statistic.
  const float pelvis[16] = {0,0,-1,0, 0,1,0,0, 1,0,0,0, 5,1,5,1};  // turned 90 degrees
  const float palm_world[3] = {5.4f, 1.3f, 5.1f};
  float local[3];
  if (!to_pelvis(pelvis, palm_world, local)) return 20;
  if (std::fabs(local[0] + 0.1f) > 1e-5f || std::fabs(local[1] - 0.3f) > 1e-5f || std::fabs(local[2] - 0.4f) > 1e-5f) return 21;
  GripStat g;
  const float fwd[3] = {0, 0, 1}, upv[3] = {0, 1, 0};
  for (int i = 0; i < 100; ++i) {
    const float p[3] = {local[0] + 0.002f * ((i % 2) ? 1.f : -1.f), local[1], local[2]};
    g.add(p, fwd, upv);
  }
  if (g.n != 100 || std::fabs(g.mean[0] - local[0]) > 1e-4f || g.spread() > 0.003f || g.spread() <= 0.f) return 22;
  // Twist: a hand rolled about the forearm passes its roll to the forearm bones by position, a share to the upper arm.
  {
    const float rolled[16] = {0,1,0,0, -1,0,0,0, 0,0,1,0, 0.25f,1.8f,0.4f,1};  // grip rolled 90 degrees about forward
    Arm a;
    if (!solve(pose, 1, rolled, basis, tune, a)) return 30;
    if (!(std::fabs(a.twist) > 0.5f)) { std::printf("twist %f\n", a.twist); return 31; }
    float m[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    twist(m, a, kLower, 0.5f);
    float rigid[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    const float half = skin_arm_ik::twist_angle(rigid, m, a.fore_axis);
    if (std::fabs(half - 0.5f * a.twist) > 1e-3f) { std::printf("half %f vs %f\n", half, a.twist); return 32; }
    float h[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    twist(h, a, kHand, 1.f);  // the hand already carries its roll
    if (h[0] != 1.f || h[5] != 1.f) return 33;
  }
  // Anchoring unnamed bones: a bracelet on the forearm, a charm by the wrist, a pouch at the hip stays.
  {
    const Vec s{0.2f, 2.0f, 0.f}, e{0.2f, 1.7f, 0.f}, w{0.2f, 1.4f, 0.f};
    float along = -1.f;
    if (nearest_role({0.23f, 1.5f, 0.f}, s, e, w, along) != kLower || std::fabs(along - 2.f / 3.f) > 1e-3f) return 40;
    if (nearest_role({0.2f, 1.33f, 0.02f}, s, e, w, along) != kHand) return 41;
    if (nearest_role({0.25f, 1.85f, 0.f}, s, e, w, along) != kUpper) return 42;
    if (nearest_role({0.0f, 1.0f, 0.3f}, s, e, w, along) != kNone) return 43;
  }
  // Blend IK <-> game pose: w = 1 is the full IK result, 0 untouched, 0.5 half the rotation and half the move.
  {
    Arm a;
    if (!solve(pose, 1, grip, basis, tune, a)) return 50;
    float full[16], none[16], half[16];
    std::memcpy(full, pose[r.lower], 64); std::memcpy(none, pose[r.lower], 64); std::memcpy(half, pose[r.lower], 64);
    float want[16];
    std::memcpy(want, pose[r.lower], 64);
    if (!apply(want, a.delta[kLower])) return 51;
    if (!blend(full, a.delta[kLower], 1.f) || !blend(none, a.delta[kLower], 0.f) || !blend(half, a.delta[kLower], 0.5f)) return 52;
    for (int k = 0; k < 16; ++k) if (std::fabs(full[k] - want[k]) > 1e-4f || none[k] != pose[r.lower][k]) return 53;
    const Vec mid{0.5f * (pose[r.lower][12] + want[12]), 0.5f * (pose[r.lower][13] + want[13]), 0.5f * (pose[r.lower][14] + want[14])};
    if (dist(skin_arm_ik::position(half), mid) > 1e-4f) return 54;
    // Half rotation: the bone starts with identity rows, so the blended rows are the rotation; angle from the trace.
    auto angle_of = [](const float* m) { return std::acos(std::clamp((m[0] + m[5] + m[10] - 1.f) * 0.5f, -1.f, 1.f)); };
    const float a_full = angle_of(want), a_half = angle_of(half);
    if (a_full > 0.05f && std::fabs(a_half - 0.5f * a_full) > 1e-3f) { std::printf("angles %f %f\n", a_half, a_full); return 55; }
  }
  // Grab: only a press near the harness grabs; held through moving away; release eases back to IK.
  {
    Grab g;
    int edge = 0;
    g.step(true, false, true, 0.01f, edge);   // squeeze away from the harness: nothing
    if (g.held || edge) return 60;
    g.step(false, true, true, 0.01f, edge);
    g.step(true, true, true, 0.01f, edge);    // squeeze at the harness: grab
    if (!g.held || edge != 1) return 61;
    for (int i = 0; i < 30; ++i) g.step(true, false, true, 0.01f, edge);  // hand drifts away, still holding
    if (!g.held || g.ik_weight() != 0.f) return 62;
    g.step(false, false, true, 0.01f, edge);  // release
    if (g.held || edge != -1) return 63;
    const float w1 = g.ik_weight();
    if (!(w1 > 0.f && w1 < 0.05f)) return 64;   // smooth start back to IK
    for (int i = 0; i < 30; ++i) g.step(false, false, true, 0.01f, edge);
    if (g.ik_weight() != 1.f) return 65;
    g.step(false, true, true, 0.01f, edge); g.step(true, true, true, 0.01f, edge);
    g.step(true, true, false, 0.01f, edge);   // dismount while holding: released
    if (g.held || edge != -1) return 66;
  }
  // Shoulder reach: an out-of-reach target swings the clavicle; the shoulder joint moves towards it by at most 12 cm.
  {
    place(pose, r.shoulder, 0.05f, 2.05f, 0.f);  // clavicle root near the neck
    Arm a;
    const float far_grip[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0.3f,1.9f,0.9f,1};
    if (!solve(pose, 1, far_grip, basis, tune, a)) return 70;
    if (!(a.reach > 0.05f && a.reach <= kShoulderReach + 1e-4f)) { std::printf("reach %f\n", a.reach); return 71; }
    float sh[16], up[16];
    std::memcpy(sh, pose[r.shoulder], 64); std::memcpy(up, pose[r.upper], 64);
    if (!apply(sh, a.delta[kShoulder]) || !apply(up, a.delta[kUpper])) return 72;
    // The clavicle root stays; the shoulder joint (upper arm root) moved forward, towards the target.
    if (dist(skin_arm_ik::position(sh), Vec{0.05f, 2.05f, 0.f}) > 1e-4f) return 73;
    if (!(up[14] > 0.04f)) return 74;
    // In reach: no clavicle change.
    Arm b;
    if (!solve(pose, 1, grip, basis, tune, b) || b.reach != 0.f) return 75;
    for (int k = 0; k < 16; ++k) if (std::fabs(b.delta[kShoulder][k] - ((k % 5 == 0) ? 1.f : 0.f)) > 1e-5f) return 76;
  }
  // Rest-pose rig: the result depends on the rest arm, the view frame and the controller only - not on the animation.
  {
    place(pose, r.shoulder, 0.05f, 2.05f, 0.f);
    Rest rest;
    const float frame0[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,2.2f,0.1f,1};  // eye
    const float* const j7[kRestJoints] = {pose[r.shoulder], pose[r.upper], pose[r.lower], pose[r.hand],
                                          pose[r.index], pose[r.middle], pose[r.pinky]};
    if (!capture_rest(rest, 1, j7, frame0) || !rest.valid[1] || rest.valid[0]) return 80;
    // Later: the rider moved 3 m and turned 90 degrees; the controller is held at the same place relative to the eye.
    const float frame1[16] = {0,0,-1,0, 0,1,0,0, 1,0,0,0, 3,2.2f,0.1f,1};
    const float grip1[16] = {0,0,-1,0, 0,1,0,0, 1,0,0,0, 3.f + 0.4f - 0.1f, 1.8f, -0.25f, 1};  // eye-relative as grip
    // Two different animated arms (riding pose sways): both must end on the same solved arm.
    static float animA[4][16], animB[4][16];
    for (int k = 0; k < 4; ++k) { place(animA, k, 3.2f, 1.9f, 0.1f * k); place(animB, k, 2.7f, 2.4f, -0.3f * k); }
    const float* const a4[4] = {animA[0], animA[1], animA[2], animA[3]};
    const float* const b4[4] = {animB[0], animB[1], animB[2], animB[3]};
    Arm ra, rb;
    if (!solve_rest(rest, 1, frame1, a4, grip1, frame1, tune, ra) || !solve_rest(rest, 1, frame1, b4, grip1, frame1, tune, rb)) return 81;
    // Native elbows pointing in opposite directions cannot pull the corrected rest elbow around.
    Vec native_a{0,1,0},native_b{0,-1,0}; Arm stable_a,stable_b;
    if(!solve_rest(rest,1,frame1,a4,grip1,frame1,Tune(tune),stable_a,&native_a) ||
       !solve_rest(rest,1,frame1,a4,grip1,frame1,Tune(tune),stable_b,&native_b)) return 134;
    if(dist(stable_a.elbow,stable_b.elbow)>1e-5f) return 135;
    for (int role = 0; role < 4; ++role) {
      float ma[16], mb[16];
      std::memcpy(ma, animA[role], 64); std::memcpy(mb, animB[role], 64);
      if (!apply(ma, ra.delta[role]) || !apply(mb, rb.delta[role])) return 82;
      for (int k = 0; k < 16; ++k) if (std::fabs(ma[k] - mb[k]) > 1e-3f) { std::printf("role %d k %d %f %f\n", role, k, ma[k], mb[k]); return 83; }
    }
    // The shoulder joint sits where the rest pose put it in the new frame: rest offset (0.2,-0.2,-0.1) from the eye,
    // turned 90 degrees and moved with the frame (unless the clavicle reached).
    float up[16];
    std::memcpy(up, animA[kUpper], 64);
    apply(up, ra.delta[kUpper]);
    const Vec want_up{3.f - 0.1f, 2.0f, 0.1f - 0.2f};  // local (0.2,-0.2,-0.1) * frame1 (origin (3,2.2,0.1))
    if (ra.reach == 0.f && dist(skin_arm_ik::position(up), want_up) > 1e-3f) {
      std::printf("upper %f %f %f\n", up[12], up[13], up[14]); return 84;
    }
  }
  // Live tune: parsing (inherit / override / reject) and each lever's effect; the default equals the untuned solve.
  {
    skin_arm_ik::HandTune base; base.pitch = -65.f; base.side = 0.18f;
    const Tune none = parse_tune("", base);
    if (none.hand.pitch != -65.f || none.hand.side != 0.f || none.reach_scale != 1.f || none.max_reach != kShoulderReach) return 100;
    const Tune t = parse_tune("tp.hand_pitch=-40\r\ntp.reach_scale=0.9\ntp.elbow_down=1.5\ntp.shoulder_up=0.05\n"
                              "tp.max_reach=0.5\ntp.twist_fore=abc\nfp.hand_pitch=10\ntp.unknown=3\n", base);
    if (t.hand.pitch != -40.f || t.hand.side != 0.f || t.reach_scale != 0.9f || t.elbow[1] != 1.5f || t.shoulder[1] != 0.05f) return 101;
    if (t.max_reach != kShoulderReach || t.twist_fore != 1.f) return 102;  // 0.5 out of range, "abc" malformed: ignored
    // Default Tune == the HandTune path.
    Arm a1, a2;
    if (!solve(pose, 1, grip, basis, tune, a1) || !solve(pose, 1, grip, basis, Tune(tune), a2)) return 103;
    for (int r2 = 0; r2 < 4; ++r2) for (int k = 0; k < 16; ++k) if (a1.delta[r2][k] != a2.delta[r2][k]) return 104;
    // Reach scale < 1 pulls the wrist toward the shoulder (more elbow bend).
    Tune near_t(tune); near_t.reach_scale = 0.8f;
    Arm a3;
    if (!solve(pose, 1, grip, basis, near_t, a3)) return 105;
    const Vec sh{0.2f, 2.0f, 0.f};
    if (!(dist(a3.wrist, sh) < dist(a1.wrist, sh) - 0.02f)) return 106;
    // Elbow pole: more "down" weight lowers the elbow.
    Tune down_t(tune); down_t.elbow[0] = 0.f; down_t.elbow[1] = 2.f; down_t.elbow[2] = 0.f;
    Tune out_t(tune); out_t.elbow[0] = 2.f; out_t.elbow[1] = 0.f; out_t.elbow[2] = 0.f;
    Arm ad, ao;
    if (!solve(pose, 1, grip, basis, down_t, ad) || !solve(pose, 1, grip, basis, out_t, ao)) return 107;
    if (!(ad.elbow.y < ao.elbow.y) || !(ao.elbow.x > ad.elbow.x)) { std::printf("elbow d %f %f o %f %f\n", ad.elbow.x, ad.elbow.y, ao.elbow.x, ao.elbow.y); return 108; }
    // Twist shares scale the forearm / upper-arm roll.
    Arm tw; tw.twist = 1.f; tw.fore_axis = {0, 0, 1}; tw.upper_axis = {0, 0, 1};
    Tune half_t; half_t.twist_fore = 0.5f; half_t.twist_upper = 0.f;
    float m1[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}, m2[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    twist(m1, tw, kLower, 1.f, half_t);
    twist(m2, tw, kUpper, 1.f, half_t);
    if (std::fabs(std::atan2(m1[1], m1[0]) - 0.5f) > 1e-4f || m2[0] != 1.f) return 109;
  }
  // Correction switch is strict and defaults on; legacy mapping remains available.
  if(!parse_tune("",tune).correction || parse_tune("tp.mount_correction=0\r\n",tune).correction ||
     !parse_tune("tp.mount_correction=0.5",tune).correction) return 110;
  if(role("L_thumb_01_expand02",side)!=kHand || side!=0 ||
     role("R_thumb_01_driver01",side)!=kHand || side!=1 ||
     role("L_thumb_01_expand02",side,false)!=kNone) return 111;
  if(!twist_helper("L_lowerArm_twist01") || !twist_helper("R_upperArm_twist02") ||
     twist_helper("L_lowerArm_expand01") || twist_helper("R_upperArm_driver01") ||
     twist_helper("L_thumb_01_expand01")) return 112;
  {
    Rest rest;
    for(int h=0;h<2;++h) {
      rest.valid[h]=true;
      for(int j=0;j<kRestJoints;++j) {
        auto& m=rest.local[h][j]; for(int k=0;k<16;++k) m[k]=(k%5==0)?1.f:0.f;
        if(h==0) {m[0]=0.f; m[1]=1.f; m[4]=-1.f; m[5]=0.f;} // different left-side axes must survive
        m[12]=h?0.25f:-0.3f; m[13]=2.f-j*0.1f; m[14]=h?0.1f:-0.35f;
      }
    }
    rest.local[1][kRShoulder][12]=0.1f;
    rest.local[0][kRShoulder][12]=-0.12f;
    const Rest old=rest;
    const float ul=dist(skin_arm_ik::position(rest.local[0][kRUpper]),skin_arm_ik::position(rest.local[0][kRLower]));
    if(!symmetric_rest(rest)) return 113;
    for(int j:{kRShoulder,kRUpper}) {
      const float* l=rest.local[0][j], *rr=rest.local[1][j];
      if(std::fabs(l[12]+rr[12])>1e-5f || l[13]!=rr[13] || l[14]!=rr[14]) return 114;
    }
    if(std::fabs(dist(skin_arm_ik::position(rest.local[0][kRUpper]),skin_arm_ik::position(rest.local[0][kRLower]))-ul)>1e-5f) return 115;
    for(int h=0;h<2;++h) for(int j=0;j<kRestJoints;++j) for(int k=0;k<12;++k)
      if(rest.local[h][j][k]!=old.local[h][j][k]) return 116;
    if(std::memcmp(rest.local[1],old.local[1],sizeof(rest.local[1]))) return 117;
    Rest partial=rest; partial.valid[0]=false; const Rest before=partial;
    if(symmetric_rest(partial) || std::memcmp(&partial,&before,sizeof(partial))) return 118;
    // Mirrored targets have equal reach on equal-length arms after positional normalization.
    skin_arm_ik::Solution left{},right{};
    const Vec sl=skin_arm_ik::position(rest.local[0][kRUpper]), sr=skin_arm_ik::position(rest.local[1][kRUpper]);
    if(!skin_arm_ik::solve(sl,{-0.35f,1.5f,0.3f},{-1,-1,-0.2f},0.3f,0.3f,left) ||
       !skin_arm_ik::solve(sr,{0.35f,1.5f,0.3f},{1,-1,-0.2f},0.3f,0.3f,right)) return 126;
    if(std::fabs(left.wrist.x+right.wrist.x)>1e-5f || std::fabs(left.elbow.x+right.elbow.x)>1e-5f ||
       std::fabs(left.elbow.z-right.elbow.z)>1e-5f) return 127;
  }
  {
    // Entire hand plus thumb helper receives one rigid motion, retaining its local relationship.
    float src[3][16]{}, dst[3][16]{};
    for(int i=0;i<3;++i) {for(int k=0;k<16;++k) src[i][k]=(k%5==0)?1.f:0.f; src[i][12]=0.1f*i;}
    signed char roles[3]={kHand,kHand,kLower}, sides[3]={0,0,0}; char twists[3]={0,0,0}; float along[3]={0,0,1};
    Arm a{}; for(auto& d:a.delta) for(int k=0;k<16;++k) d[k]=(k%5==0)?1.f:0.f;
    const float rot[16]={0,1,0,0,-1,0,0,0,0,0,1,0,0.5f,0.2f,0.1f,1};
    std::memcpy(a.delta[kHand],rot,64); a.twist=1.f;
    if(!transform_arm(dst,src,3,roles,sides,twists,along,0,a,1.f,Tune(tune))) return 119;
    float inv[16],local0[16],local1[16];
    skin_arm_ik::inverse_affine(inv,src[0]); skin_arm_ik::multiply_affine(local0,src[1],inv);
    skin_arm_ik::inverse_affine(inv,dst[0]); skin_arm_ik::multiply_affine(local1,dst[1],inv);
    for(int k=0;k<16;++k) if(std::fabs(local0[k]-local1[k])>1e-5f) return 120;
    if(std::memcmp(src[2],dst[2],64)) return 121; // non-twist forearm bone gets no added roll
    // A bad wrist rejects all arm bones together, including the otherwise-valid forearm.
    a.delta[kHand][0]=NAN;
    if(transform_arm(dst,src,3,roles,sides,twists,along,0,a,1.f,Tune(tune))) return 122;
    if(std::memcmp(src,dst,sizeof(src))) return 123;
    std::memcpy(a.delta[kHand],rot,64); a.delta[kHand][0]=0.f; a.delta[kHand][1]=0.f;
    if(transform_arm(dst,src,3,roles,sides,twists,along,0,a,1.f,Tune(tune))) return 124;
    float scale[16]; std::memcpy(scale,src[0],64); scale[0]=2.f;
    if(preserves_geometry(src[0],scale)) return 125;
    // Large rigid translations and wrist rolls remain finite and preserve mesh scale, including half-grab weights.
    for(int step=0;step<25;++step) {
      const float angle=step*0.25f;
      float* d=a.delta[kHand];
      for(int k=0;k<16;++k) d[k]=(k%5==0)?1.f:0.f;
      d[0]=std::cos(angle); d[1]=std::sin(angle); d[4]=-d[1]; d[5]=d[0];
      d[12]=100.f; d[13]=-20.f;
      if(!transform_arm(dst,src,3,roles,sides,twists,along,0,a,0.5f,Tune(tune))) return 128;
      for(int i=0;i<3;++i) if(!preserves_geometry(src[i],dst[i])) return 129;
    }
  }
  {
    // Measured twist keeps the same sign on mirrored skeleton bases, through forearm swing and a half-turn.
    float base[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    for(int mirror=0;mirror<2;++mirror) for(int step=0;step<25;++step) {
      base[0]=mirror?-1.f:1.f;
      const float angle=-3.f+0.25f*step;
      float rolled[16]; std::memcpy(rolled,base,64);
      skin_arm_ik::twist_bone(rolled,{0,0,1},angle);
      if(std::fabs(wrist_twist(base,rolled,{0,0,1})-angle)>1e-4f) return 130;
      float swing[16]={0,0,-1,0,0,1,0,0,1,0,0,0,0,0,0,1}, a[16],b[16];
      skin_arm_ik::multiply_affine(a,base,swing); skin_arm_ik::multiply_affine(b,rolled,swing);
      if(std::fabs(wrist_twist(a,b,{1,0,0})-angle)>1e-4f) return 131;
    }
    const float pi=3.14159265359f;
    if(std::fabs(continuous_twist(-pi+0.01f,pi-0.01f)-(pi+0.01f))>1e-4f) return 132;
    if(std::fabs(continuous_twist(pi-0.01f,-pi+0.01f)-(-pi-0.01f))>1e-4f) return 133;
  }
  {
    if(!arm_skin_helper("L_lowerArm_expand02") || !arm_skin_helper("R_upperArm_bend01") ||
       !arm_skin_helper("L_lowerArm_twist05") || arm_skin_helper("L_fingerThumb_01") ||
       arm_skin_helper("L_thumb_01_expand01") || arm_skin_helper("C_spine_03")) return 136;
    // Hover and boost animate the helper differently; the fixed local relationship yields identical skin output.
    const float local[16]={1,0,0,0,0,1,0,0,0,0,1,0,0.03f,0.1f,0.02f,1};
    const float parent_a[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    const float parent_b[16]={0,1,0,0,-1,0,0,0,0,0,1,0,1,2,3,1};
    const float solved[16]={1,0,0,0,0,1,0,0,0,0,1,0,0.4f,1.6f,0.7f,1};
    float source_a[16],source_b[16],inverse[16],a[16],b[16]; Arm aa{},ab{};
    skin_arm_ik::multiply_affine(source_a,local,parent_a); skin_arm_ik::multiply_affine(source_b,local,parent_b);
    skin_arm_ik::twist_bone(source_b,{0,1,0},1.2f); source_b[12]+=0.08f; // extra native deformation
    skin_arm_ik::inverse_affine(inverse,parent_a); skin_arm_ik::multiply_affine(aa.delta[kLower],inverse,solved);
    skin_arm_ik::inverse_affine(inverse,parent_b); skin_arm_ik::multiply_affine(ab.delta[kLower],inverse,solved);
    if(!fixed_helper(a,source_a,local,parent_a,aa,kLower,0.3f,false,1.f,Tune(tune)) ||
       !fixed_helper(b,source_b,local,parent_b,ab,kLower,0.3f,false,1.f,Tune(tune))) return 137;
    for(int k=0;k<16;++k) if(std::fabs(a[k]-b[k])>1e-5f) return 138;
    if(!fixed_helper(b,source_b,local,parent_b,ab,kLower,0.3f,false,0.f,Tune(tune)) ||
       std::memcmp(b,source_b,64)) return 139; // native harness endpoint
    aa.twist=ab.twist=0.8f; aa.fore_axis=ab.fore_axis={0,0,1};
    if(!fixed_helper(a,source_a,local,parent_a,aa,kLower,0.3f,true,1.f,Tune(tune)) ||
       !fixed_helper(b,source_b,local,parent_b,ab,kLower,0.3f,true,1.f,Tune(tune))) return 140;
    for(int k=0;k<16;++k) if(std::fabs(a[k]-b[k])>1e-5f) return 141;
    if(!fixed_helper(b,source_b,local,parent_b,ab,kLower,0.3f,true,0.5f,Tune(tune)) ||
       !preserves_geometry(source_b,b)) return 142;
  }
  {
    Tune t;
    t.hand.pitch=12; t.hand.yaw=-23; t.hand.roll=34; t.hand.side=0.03f; t.hand.up=0.04f; t.hand.forward=0.05f;
    t.shoulder[0]=0.06f; t.shoulder[1]=0.07f; t.shoulder[2]=0.08f;
    const auto left=offsets_for_hand(t,0),right=offsets_for_hand(t,1);
    if(left.hand.pitch!=right.hand.pitch || left.hand.yaw!=-right.hand.yaw || left.hand.roll!=-right.hand.roll ||
       left.hand.side!=-right.hand.side || left.hand.up!=right.hand.up || left.hand.forward!=right.hand.forward) return 148;
    if(left.shoulder.x!=-right.shoulder.x || left.shoulder.y!=right.shoulder.y || left.shoulder.z!=right.shoulder.z ||
       left.elbow.x!=-right.elbow.x || left.elbow.y!=right.elbow.y || left.elbow.z!=right.elbow.z) return 149;
    const Vec l=skin_arm_ik::body_offset({0,0,0},basis,left.hand),rr=skin_arm_ik::body_offset({0,0,0},basis,right.hand);
    if(std::fabs(l.x+rr.x)>1e-5f || l.y!=rr.y || l.z!=rr.z) return 150;
    // Positional tuner controls reach the solver, with no change at zero (the previous effective behavior).
    Tune zero(tune); zero.hand.forward=zero.hand.up=zero.hand.side=0.f; zero.max_reach=0.f;
    Tune moved=zero; moved.hand.forward=0.02f; moved.hand.up=0.01f; moved.hand.side=0.01f;
    Arm a,b;
    if(!solve(pose,1,grip,basis,zero,a) || !solve(pose,1,grip,basis,moved,b)) return 151;
    if(dist(skin_arm_ik::sub(b.wrist,a.wrist),{0.01f,0.01f,0.02f})>1e-4f) return 152;
    const auto parsed=parse_tune("tp.hand_side=0.03\ntp.hand_forward=0.02\n",tune);
    if(parsed.hand.side!=0.03f || offsets_for_hand(parsed,0).hand.side!=-0.03f) return 153;
  }
  {
    const Tune shared=parse_tune("tp.hand_roll=-30\ntp.shoulder_up=0.02\n",tune);
    const Tune split=parse_tune("tp.hand_roll=-30\ntp.shoulder_up=0.02\ntp.left.hand_roll=15\n"
      "tp.left.shoulder_forward=0.03\ntp.left.reach_scale=-0.1\ntp.left.forearm_roll=-45\n"
      "tp.right.hand_up=0.02\n",tune);
    const auto l=offsets_for_hand(split,0),r0=offsets_for_hand(shared,1),r1=offsets_for_hand(split,1);
    if(l.hand.roll!=45.f || l.shoulder.z!=0.03f || std::fabs(l.reach-0.9f)>1e-5f ||
       r1.hand.roll!=r0.hand.roll || r1.shoulder.y!=r0.shoulder.y || r1.hand.up!=r0.hand.up+0.02f) return 154;
    Tune left_only=shared; left_only.adjust[0].hand.roll=35; left_only.adjust[0].reach=-0.1f;
    Arm before,after;
    if(!solve(pose,1,grip,basis,shared,before) || !solve(pose,1,grip,basis,left_only,after)) return 155;
    if(std::memcmp(before.delta,after.delta,sizeof(before.delta))) return 156;
    // A per-side neutral trim exactly preserves the current output, and manual skin roll affects only that side.
    const Tune neutral=parse_tune("tp.hand_roll=-30\ntp.shoulder_up=0.02\ntp.left.hand_roll=0\ntp.right.hand_roll=0\n",tune);
    if(!solve(pose,1,grip,basis,neutral,after) || std::memcmp(before.delta,after.delta,sizeof(before.delta))) return 157;
    Arm roll{}; roll.twist=0.f; roll.fore_axis={0,0,1};
    float ml[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},mr[16]; std::memcpy(mr,ml,64);
    roll.side=0; twist(ml,roll,kLower,1.f,split);
    roll.side=1; twist(mr,roll,kLower,1.f,split);
    if(std::fabs(std::atan2(ml[1],ml[0])+0.785398163f)>1e-5f || mr[0]!=1.f || mr[1]!=0.f) return 158;
    const auto invalid=parse_tune("tp.left.forearm_roll=999\ntp.right.hand_up=nan\n",tune);
    if(invalid.adjust[0].forearm_roll!=0.f || invalid.adjust[1].hand.up!=0.f) return 159;
    Tune bounded; bounded.adjust[0].reach=-1.f; bounded.adjust[1].clavicle=0.3f;
    if(offsets_for_hand(bounded,0).reach!=0.5f || offsets_for_hand(bounded,1).clavicle!=0.3f) return 160;
  }
  // The same anatomical arm captured at different bends must produce the same solved roll, not just wrist position.
  {
    using namespace skin_arm_ik;
    float first[kRestJoints][16],second[kRestJoints][16];
    for(int j=0;j<kRestJoints;++j) {
      for(int k=0;k<16;++k) first[j][k]=(k%5==0)?1.f:0.f;
      first[j][12]=0.2f; first[j][13]=1.7f; first[j][14]=0.f;
    }
    first[kRShoulder][12]=0.1f;
    first[kRLower][13]=1.4f; first[kRHand][13]=1.4f; first[kRHand][14]=0.3f;
    for(int j=kRIndex;j<kRestJoints;++j) {first[j][13]=1.4f; first[j][14]=0.4f;}
    first[kRIndex][12]=0.22f; first[kRPinky][12]=0.18f;
    // Reflected skeleton uses its own proper rotation axes, not a negative-determinant world reflection.
    Rest mirrored; mirrored.valid[0]=mirrored.valid[1]=true;
    for(int j=0;j<kRestJoints;++j) {
      std::memcpy(mirrored.local[1][j],first[j],64);
      std::memcpy(mirrored.local[0][j],first[j],64);
      for(int row=0;row<3;++row) for(int col=0;col<3;++col)
        mirrored.local[0][j][row*4+col]*=((row==0)!=(col==0))?-1.f:1.f;
      mirrored.local[0][j][12]*=-1.f;
    }
    // Disturb the left articulation before normalization: length and hand-local geometry must survive.
    const Vec pivot=position(mirrored.local[0][kRUpper]);
    for(int j=kRUpper;j<kRestJoints;++j) {
      auto* m=mirrored.local[0][j]; twist_bone(m,{0,1,0},1.7f);
      const Vec p=add(pivot,rotate_axis(sub(position(m),pivot),{0,1,0},1.7f));
      m[12]=p.x; m[13]=p.y; m[14]=p.z;
    }
    const float old_length=dist(position(mirrored.local[0][kRLower]),position(mirrored.local[0][kRHand]));
    if(!symmetric_rest(mirrored)) return 164;
    if(std::fabs(dist(position(mirrored.local[0][kRLower]),position(mirrored.local[0][kRHand]))-old_length)>1e-5f) return 165;
    for(int j=kRUpper;j<=kRHand;++j) {
      const Vec l=position(mirrored.local[0][j]),r=position(mirrored.local[1][j]);
      if(dist(l,{-r.x,r.y,r.z})>1e-5f || !quality(mirrored.local[0][j]).valid) return 166;
    }
    std::memcpy(second,first,sizeof(first));
    const Vec a{0,-0.3f,0},c{0,0,0.3f},normal=cross(a,c);
    const Vec na{0.18f,0,0.24f},nc{-0.24f,0,0.18f},nn=cross(na,nc);
    const Vec shoulder=position(first[kRUpper]),elbow=add(shoulder,na);
    limb_matrix(second[kRUpper],first[kRUpper],a,na,normal,nn,shoulder);
    limb_matrix(second[kRLower],first[kRLower],c,nc,normal,nn,elbow);
    float inverse[16],carry[16]; inverse_affine(inverse,first[kRLower]);
    multiply_affine(carry,inverse,second[kRLower]);
    for(int j=kRHand;j<kRestJoints;++j) multiply_affine(second[j],first[j],carry);
    auto joints=[](float (*m)[16]) {Joints j; j.shoulder=m[kRShoulder]; j.upper=m[kRUpper];
      j.lower=m[kRLower]; j.hand=m[kRHand]; j.index=position(m[kRIndex]);
      j.middle=position(m[kRMiddle]); j.pinky=position(m[kRPinky]); return j;};
    Tune t; t.hand.palm=0; t.max_reach=0;
    for(int i=0;i<24;++i) {
      float right_grip[16],left_grip[16]; std::memcpy(right_grip,grip,64);
      right_grip[12]=0.3f; right_grip[13]=1.5f; right_grip[14]=0.3f;
      twist_bone(right_grip,{0,0,1},i*0.261799388f);
      std::memcpy(left_grip,right_grip,64);
      for(int row=0;row<3;++row) for(int col=0;col<3;++col)
        left_grip[row*4+col]*=((row==0)!=(col==0))?-1.f:1.f;
      left_grip[12]*=-1.f;
      Arm l,r;
      if(!solve_joints(joints(mirrored.local[0]),0,left_grip,basis,t,l) ||
         !solve_joints(joints(mirrored.local[1]),1,right_grip,basis,t,r)) return 167;
      if(dist(l.elbow,{-r.elbow.x,r.elbow.y,r.elbow.z})>1e-4f ||
         dist(l.wrist,{-r.wrist.x,r.wrist.y,r.wrist.z})>1e-4f ||
         std::fabs(std::remainder(l.twist+r.twist,6.283185307f))>1e-4f) return 168;
    }
    for(unsigned side=0;side<2;++side) for(int i=0;i<24;++i) {
      float target[16]; std::memcpy(target,grip,64);
      target[12]=(side?1.f:-1.f)*0.3f; target[13]=1.5f; target[14]=0.3f;
      twist_bone(target,{0,0,1},i*0.261799388f);
      Arm x,y;
      if(!solve_joints(joints(first),side,target,basis,t,x) ||
         !solve_joints(joints(second),side,target,basis,t,y)) return 161;
      if(dist(x.elbow,y.elbow)>1e-4f || dist(x.wrist,y.wrist)>1e-4f ||
         std::fabs(std::remainder(x.twist-y.twist,6.283185307f))>1e-4f) return 162;
      for(int role=0;role<3;++role) {
        float left[16],right[16];
        multiply_affine(left,first[kRUpper+role],x.delta[role]);
        multiply_affine(right,second[kRUpper+role],y.delta[role]);
        for(int k=0;k<16;++k) if(std::fabs(left[k]-right[k])>1e-4f) return 163;
      }
    }
  }
  // Gear native frames may animate or use different cells; they must land on the shared solved arm exactly.
  {
    using namespace skin_arm_ik;
    float reference[16],solved[16],parent[16],delta[16],result[16],inverse[16],expected[16];
    std::memcpy(reference,basis,64); std::memcpy(solved,basis,64);
    reference[12]=0.2f; reference[13]=1.4f;
    solved[12]=0.4f; solved[13]=1.6f; solved[14]=0.3f; twist_bone(solved,{0,0,1},0.8f);
    inverse_affine(inverse,reference); multiply_affine(delta,inverse,solved);
    for(int pass=0;pass<12;++pass) {
      std::memcpy(parent,reference,64); parent[12]+=pass*0.04f; parent[13]-=pass*0.02f;
      twist_bone(parent,{0,1,0},pass*0.2f);
      if(!attachment_delta(result,parent,reference,delta,{1024,0,-1024})) return 169;
      multiply_affine(expected,parent,result);
      for(int k=0;k<16;++k) {
        const float want=solved[k]+(k==12?1024.f:k==14?-1024.f:0.f);
        if(std::fabs(expected[k]-want)>0.0003f) return 170;
      }
    }
    // A bracelet/charm group follows one already-twisted anchor with a single harness blend.
    float local[16],animated[16],out[16],anchor[16];
    std::memcpy(local,basis,64); local[12]=0.03f; local[14]=0.02f;
    std::memcpy(animated,basis,64); animated[12]=-0.4f;
    Arm identity{}; std::memcpy(identity.delta[kLower],basis,64);
    for(int i=0;i<24;++i) {
      std::memcpy(anchor,solved,64); twist_bone(anchor,{0,0,1},i*0.261799388f);
      if(!fixed_helper(out,animated,local,anchor,identity,kLower,0,false,1.f,Tune{})) return 171;
      multiply_affine(expected,local,anchor);
      for(int k=0;k<16;++k) if(std::fabs(out[k]-expected[k])>1e-5f) return 172;
      if(!fixed_helper(out,animated,local,anchor,identity,kLower,0,false,0.f,Tune{}) ||
         std::memcmp(out,animated,64)) return 173;
      if(!fixed_helper(out,animated,local,anchor,identity,kLower,0,false,0.5f,Tune{}) ||
         !preserves_geometry(animated,out)) return 174;
    }
  }
  // Reproduce the runtime failure: a retained delta plus newer native joints moves the attachment off the arm.
  {
    using namespace skin_arm_ik;
    float native[4][16]; const float* joints[4]; Arm solved{};
    for(int role=0;role<4;++role) {
      std::memcpy(native[role],basis,64); native[role][12]=0.2f; native[role][13]=1.3f;
      std::memcpy(solved.delta[role],basis,64); solved.delta[role][14]=0.4f;
      joints[role]=native[role];
    }
    solved.wrist={0.2f,1.3f,0.4f}; solved.side=0; solved.twist=0.7f;
    AttachmentPose snapshot;
    if(!capture_attachment(snapshot,joints,solved,{1024,0,0},{1030,1,0})) return 175;
    const AttachmentPose previous=snapshot;
    for(int role=0;role<4;++role) native[role][12]+=0.15f; // next native animation, preceding solve retained
    float mismatched[16]; multiply_affine(mismatched,native[kHand],solved.delta[kHand]);
    if(std::fabs(length(sub(position(mismatched),solved.wrist))-0.15f)>1e-5f) return 176;
    // Refuse mismatched publications atomically and retain complete last good output and twist.
    if(capture_attachment(snapshot,joints,solved,{1024,0,0},{1031,1,0}) ||
       std::memcmp(&snapshot,&previous,sizeof(snapshot))) return 177;
    for(int pass=0;pass<20;++pass) {
      float output[16]; const Vec body{1030.f+pass,1,0};
      if(!attachment_frame(output,snapshot,kHand,body,{2048,0,0})) return 178;
      if(length(sub(position(output),{-1023.8f+pass,1.3f,0.4f}))>0.0002f ||
         snapshot.arm.twist!=0.7f) return 179;
    }
    for(int role=0;role<4;++role) native[role][12]-=0.15f;
    native[kLower][0]=0; native[kLower][5]=0; native[kLower][10]=0;
    if(capture_attachment(snapshot,joints,solved,{1024,0,0},{1030,1,0}) ||
       std::memcmp(&snapshot,&previous,sizeof(snapshot))) return 180;
  }
  // Native spheres around 31cm excluded relocated arm bones. Preserve native geometry and include either arm.
  {
    using namespace mounted_accessory_bounds;
    Bounds native{}; native.local[3]=0.31f; native.world[0]=-7534.f;
    native.world[1]=2062.f; native.world[2]=-610.f; native.world[3]=0.31f;
    native.box[0]=-7534.f; native.box[1]=2062.f; native.box[2]=-610.f;
    native.box[3]=0.1f; native.box[7]=0.2f; native.box[11]=0.15f;
    const Bounds original=native; Bounds out{};
    for(int side:{-1,1}) {
      const Vec low{-7534.f+side-0.1f,2061.8f,-609.5f},high{-7534.f+side+0.1f,2062.4f,-609.f};
      if(!include(out,native,low,high,1.f) || std::memcmp(&native,&original,sizeof(native))) return 181;
      if(out.world[3]<native.world[3] || out.local[3]!=out.world[3]) return 182;
      for(int i=0;i<8;++i) {
        const Vec p{(i&1)?high.x+0.15f:low.x-0.15f,(i&2)?high.y+0.15f:low.y-0.15f,
          (i&4)?high.z+0.15f:low.z-0.15f};
        if(skin_arm_ik::length(skin_arm_ik::sub(p,{out.world[0],out.world[1],out.world[2]}))>out.world[3]+0.001f) return 183;
        const float v[3]={p.x,p.y,p.z};
        for(int k=0;k<3;++k) if(std::fabs(v[k]-out.box[k])>out.box[3+k*4]+0.001f) return 184;
      }
    }
    if(!include(out,native,{-7535,2062,-610},{-7533,2063,-609},2.f) ||
       std::fabs(out.local[3]*2.f-out.world[3])>1e-5f) return 185;
    const Bounds previous=out;
    if(include(out,native,{100,0,0},{101,1,1},1.f) || std::memcmp(&out,&previous,sizeof(out)) ||
       include(out,native,{0,0,0},{1,1,1},0.f)) return 186;
    if(!parse_tune("",tune).accessory_bounds || parse_tune("tp.accessory_bounds=0\n",tune).accessory_bounds ||
       !parse_tune("tp.accessory_bounds=0.5\n",tune).accessory_bounds) return 187;
  }
  std::puts("rider_ik_test ok");
  return 0;
}
