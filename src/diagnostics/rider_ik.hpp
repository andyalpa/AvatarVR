#pragma once
#include "skin_arm_ik.hpp"
#include <cstring>
#include <string>
#include <vector>

// FEATURE-06 stage B (2026-10-24): controller arms on the third-person body while mounted. Portable (no engine types);
// unit-tested in tools/tests/rider_ik_test.cpp. While mounted the game animates only the 192-bone third-person pose
// (component +0x4F8, names: WEAPONS_AND_HANDS.md "LIVE named skeletons"), so the arm is solved on that pose: two-bone IK
// from the animated shoulder to the controller, hand orientation from the animated knuckles (skin_arm_ik::align_hand).
// The result is three rigid deltas per arm (upper arm, forearm, hand); every same-side bone of a player mesh whose name
// has that role gets new = old * delta (skin_arm_ik::follow's rule), renderer copy only.
namespace rider_ik {
using skin_arm_ik::Vec;

struct ArmBones { int shoulder, upper, lower, hand, index, middle, pinky; };
constexpr ArmBones kArm[2] = {{36, 37, 38, 39, 46, 51, 56}, {74, 75, 76, 77, 84, 89, 94}};  // 192 pose: left, right
constexpr int kPoseBones = 192;

enum Role : signed char { kNone = -1, kUpper = 0, kLower = 1, kHand = 2, kShoulder = 3 };
// Mesh bone name -> (side, role). Clavicle (shoulder*), upper arm and its helpers, forearm and its twist helpers, and
// the hand subtree (hand, fingers, thumb, palm, handWeapon, weaponAttachmentHand). Legs and everything else stay animated.
inline Role role(const char* name, int& side, bool corrected = true) {
  side = -1;
  if (!name || (std::strncmp(name, "L_", 2) && std::strncmp(name, "R_", 2))) return kNone;
  side = name[0] == 'R' ? 1 : 0;
  const char* rest = name + 2;
  if (!std::strncmp(rest, "shoulder", 8)) return kShoulder;
  if (!std::strncmp(rest, "upperArm", 8)) return kUpper;
  if (!std::strncmp(rest, "lowerArm", 8)) return kLower;
  if ((corrected && !std::strncmp(rest, "thumb", 5)) || !std::strncmp(rest, "hand", 4) || !std::strncmp(rest, "finger", 6) || !std::strncmp(rest, "palm", 4) ||
      !std::strncmp(rest, "weaponAttachmentHand", 20))
    return kHand;
  side = -1;
  return kNone;
}

struct Arm {
  unsigned side=1; // controller side (0 left / 1 right), also selects independent skin-roll adjustments
  float delta[4][16];  // world: new = old * delta[role] (kShoulder: identity unless the shoulder reached)
  Vec elbow, wrist;
  bool clamped = false;
  float reach = 0.f;   // metres the shoulder joint moved towards the target
  float twist = 0.f;      // hand roll about the forearm relative to a forearm-rigid hand (radians)
  Vec fore_axis{0, 0, 1}, upper_axis{0, 0, 1};  // solved forearm / upper-arm directions (unit)
};
constexpr float kUpperTwistShare = 0.3f;  // as the first-person path (skin_palette_probe upper_twist_share)
// [HEADSET 2026-10-24] "left arm does not properly stretch from the shoulder when flying forward": the boost riding pose
// holds the shoulders back. Beyond 97 % of the arm's reach the clavicle swings the shoulder joint towards the target,
// at most this far.
constexpr float kShoulderReach = 0.12f;

// ---- Live tuning (2026-10-25 [AUTHOR]: "a simple tool ... to manually tune the IK arms, hands poses ... see the
// tuning changing live", third person separate from first person). Defaults reproduce the untuned solve exactly.
// tools/ik_tuner writes scripts/AvatarVR_ik_tune.txt; the mod re-reads it when it changes (rider_ik::parse_tune).
struct ArmAdjustment {
  skin_arm_ik::HandTune hand;
  float shoulder[3]{},elbow[3]{};
  float reach=0.f,clavicle=0.f,twist_fore=0.f,twist_upper=0.f;
  float forearm_roll=0.f,upper_roll=0.f; // degrees added to distributed skin roll
  ArmAdjustment() {hand.palm=0.f;} // all adjustments, including palm depth, start at zero
};
struct Tune {
  skin_arm_ik::HandTune hand;      // hand pitch/yaw/roll (deg), forward/up/side (m, body frame), palm (share)
  float shoulder[3] = {0, 0, 0};   // rest shoulder offset in the view frame: out (mirrored per side), up, forward (m)
  float reach_scale = 1.f;         // controller distance from the shoulder x this (< 1: elbows stay bent)
  float max_reach = kShoulderReach;  // clavicle reach limit (m)
  float elbow[3] = {1.f, 1.f, 0.2f}; // elbow pole weights: outward, down, back
  float twist_fore = 1.f;          // share of the hand roll given to the forearm twist bones
  float twist_upper = kUpperTwistShare;  // share given to the upper-arm twist bones
  bool correction = true;          // tp.mount_correction=0 restores preceding mounted behavior
  bool accessory_bounds = true;    // tp.accessory_bounds=0 disables only the mounted gear bounds union
  bool weapon_stable = true;       // tp.weapon_stable=0 restores strict mounted gun ownership and uncarried commits
  ArmAdjustment adjust[2];         // additive corrections after shared offset mirroring; zero preserves current setup
  Tune() = default;
  Tune(const skin_arm_ik::HandTune& h) : hand(h) {}  // NOLINT: implicit, untuned callers pass a HandTune
};
// One right-authored set of IK offsets, resolved for each controller. No skeleton transforms are mirrored here.
struct SideOffsets {
  skin_arm_ik::HandTune hand;
  Vec shoulder{},elbow{}; // level view axes: right, up, forward
  float reach=1.f,clavicle=kShoulderReach;
};
inline SideOffsets offsets_for_hand(const Tune& tune,unsigned hand) {
  const float side=hand?1.f:-1.f;
  SideOffsets out{skin_arm_ik::for_hand(tune.hand,hand),
          {side*tune.shoulder[0],tune.shoulder[1],tune.shoulder[2]},
          {side*tune.elbow[0],-tune.elbow[1],-tune.elbow[2]}};
  const auto& a=tune.adjust[hand?1:0];
  out.hand.pitch+=a.hand.pitch; out.hand.yaw+=a.hand.yaw; out.hand.roll+=a.hand.roll;
  out.hand.forward+=a.hand.forward; out.hand.up+=a.hand.up; out.hand.side+=side*a.hand.side;
  out.hand.palm=std::clamp(out.hand.palm+a.hand.palm,0.f,1.5f);
  out.shoulder=skin_arm_ik::add(out.shoulder,{side*a.shoulder[0],a.shoulder[1],a.shoulder[2]});
  out.elbow=skin_arm_ik::add(out.elbow,{side*a.elbow[0],-a.elbow[1],-a.elbow[2]});
  out.reach=std::clamp(tune.reach_scale+a.reach,0.5f,1.5f);
  out.clavicle=std::clamp(tune.max_reach+a.clavicle,0.f,0.3f);
  return out;
}
// Third-person keys ("tp." prefix; "fp." is reserved for the first-person rig). Absent rotation keys inherit the
// first-person tune; positional offsets default to zero, their previous effective value. Invalid keys are ignored.
inline Tune parse_tune(const char* text, const skin_arm_ik::HandTune& base) {
  Tune t(base);
  // These positional knobs were historically ignored in the mounted solve. Default to their previous EFFECTIVE
  // value (zero); enabling their implementation must not suddenly inherit the on-foot 18 cm lateral correction.
  t.hand.forward=t.hand.up=t.hand.side=0.f;
  // Keep the switch strict: no fractional or malformed boolean values.
  for (const char* p=text; p && *p;) {
    const char* e=std::strchr(p,'\n');
    const std::size_t n=e?static_cast<std::size_t>(e-p):std::strlen(p);
    constexpr const char* key="tp.mount_correction=";
    constexpr std::size_t k=20;
    if(n>=k && !std::strncmp(p,key,k)) {
      std::size_t end=n; while(end>k && (p[end-1]=='\r'||p[end-1]==' ')) --end;
      if(end==k+1 && (p[k]=='0'||p[k]=='1')) t.correction=p[k]=='1';
    }
    constexpr const char* bounds_key="tp.accessory_bounds=";
    constexpr std::size_t bk=20;
    if(n>=bk && !std::strncmp(p,bounds_key,bk)) {
      std::size_t end=n; while(end>bk && (p[end-1]=='\r'||p[end-1]==' ')) --end;
      if(end==bk+1 && (p[bk]=='0'||p[bk]=='1')) t.accessory_bounds=p[bk]=='1';
    }
    constexpr const char* stable_key="tp.weapon_stable=";
    constexpr std::size_t sk=17;
    if(n>=sk && !std::strncmp(p,stable_key,sk)) {
      std::size_t end=n; while(end>sk && (p[end-1]=='\r'||p[end-1]==' ')) --end;
      if(end==sk+1 && (p[sk]=='0'||p[sk]=='1')) t.weapon_stable=p[sk]=='1';
    }
    p=e?e+1:nullptr;
  }
  struct Key { std::string name; float* value; float lo, hi; };
  std::vector<Key> keys = {
    {"tp.hand_pitch", &t.hand.pitch, -180.f, 180.f}, {"tp.hand_yaw", &t.hand.yaw, -180.f, 180.f},
    {"tp.hand_roll", &t.hand.roll, -180.f, 180.f}, {"tp.hand_forward", &t.hand.forward, -0.3f, 0.3f},
    {"tp.hand_up", &t.hand.up, -0.3f, 0.3f}, {"tp.hand_side", &t.hand.side, -0.3f, 0.3f},
    {"tp.hand_palm", &t.hand.palm, 0.f, 1.5f},
    {"tp.shoulder_out", &t.shoulder[0], -0.3f, 0.3f}, {"tp.shoulder_up", &t.shoulder[1], -0.3f, 0.3f},
    {"tp.shoulder_forward", &t.shoulder[2], -0.3f, 0.3f},
    {"tp.reach_scale", &t.reach_scale, 0.5f, 1.5f}, {"tp.max_reach", &t.max_reach, 0.f, 0.3f},
    {"tp.elbow_out", &t.elbow[0], -2.f, 2.f}, {"tp.elbow_down", &t.elbow[1], -2.f, 2.f},
    {"tp.elbow_back", &t.elbow[2], -2.f, 2.f},
    {"tp.twist_fore", &t.twist_fore, 0.f, 1.5f}, {"tp.twist_upper", &t.twist_upper, 0.f, 1.f},
  };
  for(int h=0;h<2;++h) {
    const std::string p=h?"tp.right.":"tp.left."; auto& a=t.adjust[h];
    const Key extra[]={
      {"hand_pitch",&a.hand.pitch,-180,180},{"hand_yaw",&a.hand.yaw,-180,180},{"hand_roll",&a.hand.roll,-180,180},
      {"hand_forward",&a.hand.forward,-0.3f,0.3f},{"hand_up",&a.hand.up,-0.3f,0.3f},{"hand_side",&a.hand.side,-0.3f,0.3f},
      {"hand_palm",&a.hand.palm,-1.5f,1.5f},
      {"shoulder_out",&a.shoulder[0],-0.3f,0.3f},{"shoulder_up",&a.shoulder[1],-0.3f,0.3f},{"shoulder_forward",&a.shoulder[2],-0.3f,0.3f},
      {"reach_scale",&a.reach,-1.f,1.f},{"max_reach",&a.clavicle,-0.3f,0.3f},
      {"elbow_out",&a.elbow[0],-2,2},{"elbow_down",&a.elbow[1],-2,2},{"elbow_back",&a.elbow[2],-2,2},
      {"twist_fore",&a.twist_fore,-1.5f,1.5f},{"twist_upper",&a.twist_upper,-1.f,1.f},
      {"forearm_roll",&a.forearm_roll,-180,180},{"upper_roll",&a.upper_roll,-180,180}};
    for(const auto& k:extra) keys.push_back({p+k.name,k.value,k.lo,k.hi});
  }
  for (const char* line = text; line && *line;) {
    const char* end = std::strchr(line, '\n');
    const std::size_t len = end ? static_cast<std::size_t>(end - line) : std::strlen(line);
    for (const auto& k : keys) {
      const std::size_t kl = k.name.size();
      if (len > kl + 1 && !std::strncmp(line, k.name.c_str(), kl) && line[kl] == '=') {
        char buf[32]{};
        const std::size_t vl = std::min<std::size_t>(len - kl - 1, sizeof(buf) - 1);
        std::memcpy(buf, line + kl + 1, vl);
        while (vl && (buf[std::strlen(buf) - 1] == '\r' || buf[std::strlen(buf) - 1] == ' ')) { buf[std::strlen(buf) - 1] = 0; if (!buf[0]) break; }
        char* stop = nullptr;
        const float v = std::strtof(buf, &stop);
        if (stop && stop != buf && *stop == 0 && std::isfinite(v) && v >= k.lo && v <= k.hi) *k.value = v;
      }
    }
    line = end ? end + 1 : nullptr;
  }
  return t;
}

// Forearm twist (2026-10-24 [HEADSET] the hand rolled while the forearm skin stayed: "a little deform"). Like the
// first-person path, the hand's roll is shared by the forearm bones by how far along the forearm they sit (0 elbow,
// 1 wrist) and, scaled by the upper share, by the upper-arm bones. Rotation only, about the bone's own position.
inline void twist(float* m, const Arm& a, Role r, float along, const Tune& t = Tune{}) {
  const auto& d=t.adjust[a.side?1:0];
  if (r == kLower) skin_arm_ik::twist_bone(m, a.fore_axis,
    (a.twist*std::clamp(t.twist_fore+d.twist_fore,0.f,1.5f)+d.forearm_roll*0.01745329252f)*along);
  else if (r == kUpper) skin_arm_ik::twist_bone(m, a.upper_axis,
    (a.twist*std::clamp(t.twist_upper+d.twist_upper,0.f,1.f)+d.upper_roll*0.01745329252f)*along);
}

// Bones without an arm name (bracelets, arm-gear charms, C_accessory*) ride the arm segment they lie on in the animated
// pose: within 8 cm of the forearm or upper arm, or within 15 cm of the wrist and not behind it. `along` receives the
// position along the segment for the twist share. Callers exclude body-part names first (skin_palette_probe
// body_part_name: the 2026-10-15 proximity fallback pulled waist/legs/hair gear to the hands).
inline Role nearest_role(Vec p, Vec s, Vec e, Vec w, float& along) {
  using namespace skin_arm_ik;
  along = 0.f;
  float d_lower = 0.f, d_upper = 0.f;
  const float t_lower = segment_t(p, e, w, d_lower), t_upper = segment_t(p, s, e, d_upper);
  const Vec fore = sub(w, e);
  const float fl = length(fore);
  const float beyond = fl > 1e-5f ? dot(sub(p, w), fore) / fl : -1.f;
  if (d_lower < 0.08f && t_lower >= 0.f && t_lower < 0.97f) { along = t_lower; return kLower; }
  if (d_upper < 0.08f && t_upper > 0.08f && t_upper <= 1.f) { along = t_upper; return kUpper; }
  if (length(sub(p, w)) < 0.15f && beyond > -0.02f) return kHand;
  return kNone;
}

// One arm's animated joints, all in the same space and moment (one mesh's own bones, or the 192 pose). shoulder may be
// null (no reach). Knuckle positions orient the hand (skin_arm_ik::align_hand).
struct Joints {
  const float* shoulder = nullptr;
  const float* upper = nullptr;
  const float* lower = nullptr;
  const float* hand = nullptr;
  Vec index{}, middle{}, pinky{};
};

// Swing/twist decomposition of the relative rigid rotation. Unlike choosing a different matrix row as the arm
// turns, this measures the same anatomical roll on both sides, including near a half-turn.
inline float wrist_twist(const float* rigid,const float* aligned,Vec axis) {
  using namespace skin_arm_ik;
  float d[3][3]{};
  for(int k=0;k<3;++k) {
    Vec a=row(rigid,k),b=row(aligned,k);
    const float la=length(a),lb=length(b);
    if(la<1e-5f || lb<1e-5f) return 0.f;
    a=scale(a,1.f/la); b=scale(b,1.f/lb);
    const float av[3]={a.x,a.y,a.z},bv[3]={b.x,b.y,b.z};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) d[i][j]+=av[i]*bv[j];
  }
  float q[4]{};
  const float tr=d[0][0]+d[1][1]+d[2][2];
  if(tr>0.f) {
    const float s=2.f*std::sqrt(tr+1.f); q[3]=0.25f*s;
    q[0]=(d[1][2]-d[2][1])/s; q[1]=(d[2][0]-d[0][2])/s; q[2]=(d[0][1]-d[1][0])/s;
  } else {
    int i=0; if(d[1][1]>d[i][i]) i=1; if(d[2][2]>d[i][i]) i=2;
    const int j=(i+1)%3,k=(i+2)%3;
    const float s=2.f*std::sqrt(std::max(0.f,1.f+d[i][i]-d[j][j]-d[k][k]));
    if(s<1e-5f) return 0.f;
    q[i]=0.25f*s; q[j]=(d[i][j]+d[j][i])/s; q[k]=(d[i][k]+d[k][i])/s;
    q[3]=(d[j][k]-d[k][j])/s;
  }
  if(q[3]<0.f) for(float& v:q) v=-v;
  const float n=length(axis); if(n<1e-5f) return 0.f;
  const float p=(q[0]*axis.x+q[1]*axis.y+q[2]*axis.z)/n;
  if(p*p+q[3]*q[3]<1e-8f) return 0.f; // pure 180-degree swing: no determined twist
  return 2.f*std::atan2(p,q[3]);
}
inline float continuous_twist(float angle,float previous) {
  constexpr float pi=3.14159265359f,two_pi=2.f*pi;
  if(!std::isfinite(angle)||!std::isfinite(previous)) return 0.f;
  return previous+std::remainder(angle-previous,two_pi);
}

// grip: controller grip in the joints' space (rows right, up, forward, position). basis: level body frame for the
// elbow pole (rows right, up, forward). hand: 0 left, 1 right. prev_bend (optional, in/out): elbow bend continuity.
// Two directions define limb orientation. A shortest swing alone preserves capture-dependent axial roll.
// Map the bend-plane normal as well, retaining the source bone's local axes and scale.
inline void limb_matrix(float* out,const float* source,Vec from,Vec to,Vec old_normal,Vec new_normal,Vec pos) {
  using namespace skin_arm_ik;
  matrix(out,source,from,to,pos);
  const float tl=length(to),ol=length(old_normal),nl=length(new_normal);
  if(tl<1e-5f || ol<1e-5f || nl<1e-5f) return; // straight capture: retain established swing fallback
  const Vec axis=scale(to,1.f/tl);
  Vec a=rotate(scale(old_normal,1.f/ol),from,to),b=scale(new_normal,1.f/nl);
  a=sub(a,scale(axis,dot(a,axis))); b=sub(b,scale(axis,dot(b,axis)));
  if(length(a)<1e-5f || length(b)<1e-5f) return;
  twist_bone(out,axis,std::atan2(dot(axis,cross(a,b)),dot(a,b)));
}
inline bool solve_joints(const Joints& j, unsigned hand, const float* grip, const float* basis,
                         const Tune& tune, Arm& out, Vec* prev_bend = nullptr,const float* wrist_override=nullptr) {
  using namespace skin_arm_ik;
  if (hand > 1 || !j.upper || !j.lower || !j.hand || !grip || !basis) return false;
  out.side=hand;
  const float* up = j.upper;
  const float* lo = j.lower;
  const float* ha = j.hand;
  float h[16];
  std::memcpy(h, ha, sizeof(h));
  Vec target{};
  const auto offsets=offsets_for_hand(tune,hand);
  if(wrist_override) {
    for(int k=0;k<16;++k) if(!std::isfinite(wrist_override[k])) return false;
    float inverse[16]; if(!inverse_affine(inverse,wrist_override)) return false;
    std::memcpy(h,wrist_override,64); target=position(h);
  } else {
    if (!align_hand(h, target, j.middle, j.index, j.pinky, grip, offsets.hand)) return false;
    target=body_offset(target,basis,offsets.hand);
  }
  const Vec s0 = position(up), e0 = position(lo), w0 = position(ha);
  const float la = length(sub(e0, s0)), lc = length(sub(w0, e0));
  if (!wrist_override && offsets.reach != 1.f) target = add(s0, scale(sub(target, s0), offsets.reach));
  // Shoulder reach: rotate the clavicle about its root so the shoulder joint moves towards an out-of-reach target.
  float up_c[16], lo_c[16], ha_c[16];
  std::memcpy(up_c, up, sizeof(up_c));
  std::memcpy(lo_c, lo, sizeof(lo_c));
  std::memcpy(ha_c, ha, sizeof(ha_c));
  float sh_new[16];
  out.reach = 0.f;
  if (j.shoulder) std::memcpy(sh_new, j.shoulder, sizeof(sh_new));
  if (j.shoulder) {
    const Vec p = position(j.shoulder), arm = sub(s0, p), want = sub(target, p);
    const float excess = length(sub(target, s0)) - 0.97f * (la + lc), al = length(arm);
    const Vec c = cross(arm, want);
    const float cl = length(c);
    if (excess > 0.f && al > 0.02f && cl > 1e-6f) {
      const float full = std::atan2(cl, dot(arm, want));
      const float angle = std::min(full, std::min(excess, offsets.clavicle) / al);
      const Vec axis = scale(c, 1.f / cl);
      auto carry = [&](float* m) {  // rotate a matrix rigidly about the clavicle root
        for (int i = 0; i < 3; ++i) {
          const Vec r = rotate_axis(row(m, i), axis, angle);
          m[i * 4] = r.x; m[i * 4 + 1] = r.y; m[i * 4 + 2] = r.z;
        }
        const Vec q = add(p, rotate_axis(sub(position(m), p), axis, angle));
        m[12] = q.x; m[13] = q.y; m[14] = q.z;
      };
      carry(sh_new); carry(up_c); carry(lo_c); carry(ha_c);
      out.reach = length(sub(position(up_c), s0));
    }
  }
  const Vec s = position(up_c), e = position(lo_c);
  const Vec a = sub(e, s);
  const Vec c = sub(position(ha_c), e);
  const auto p=offsets.elbow;
  Vec pole{p.x*basis[0]+p.y*basis[4]+p.z*basis[8],p.x*basis[1]+p.y*basis[5]+p.z*basis[9],
           p.x*basis[2]+p.y*basis[6]+p.z*basis[10]};
  if (prev_bend && length(*prev_bend) > 0.5f) pole = add(scale(pole, 1.f / std::max(1e-4f, length(pole))), scale(*prev_bend, 2.f));
  Solution sol{};
  if (!skin_arm_ik::solve(s, target, pole, la, lc, sol)) return false;
  float nu[16], nl[16], nh[16];
  matrix(nu, up_c, a, sub(sol.elbow, s), s);
  matrix(nl, lo_c, c, sub(sol.wrist, sol.elbow), sol.elbow);
  if(tune.correction) {
    const Vec normal=cross(a,c),solved_normal=cross(sub(sol.elbow,s),sub(sol.wrist,sol.elbow));
    limb_matrix(nu,up_c,a,sub(sol.elbow,s),normal,solved_normal,s);
    limb_matrix(nl,lo_c,c,sub(sol.wrist,sol.elbow),normal,solved_normal,sol.elbow);
  }
  std::memcpy(nh, h, sizeof(nh));
  nh[12] = sol.wrist.x; nh[13] = sol.wrist.y; nh[14] = sol.wrist.z;
  const float* old[3] = {up, lo, ha};
  const float* now[3] = {nu, nl, nh};
  for (int r = 0; r < 3; ++r) {
    float inv[16];
    if (!inverse_affine(inv, old[r])) return false;
    multiply_affine(out.delta[r], inv, now[r]);
    for (float v : out.delta[r]) if (!std::isfinite(v)) return false;
  }
  if (j.shoulder) {
    float inv[16];
    if (!inverse_affine(inv, j.shoulder)) return false;
    multiply_affine(out.delta[kShoulder], inv, sh_new);
    for (float v : out.delta[kShoulder]) if (!std::isfinite(v)) return false;
  } else {
    for (int k = 0; k < 16; ++k) out.delta[kShoulder][k] = (k % 5 == 0) ? 1.f : 0.f;
  }
  out.elbow = sol.elbow;
  out.wrist = sol.wrist;
  out.clamped = sol.clamped;
  {
    const Vec fa = sub(sol.wrist, sol.elbow), ua = sub(sol.elbow, s);
    const float fl = length(fa), ul = length(ua);
    out.fore_axis = fl > 1e-5f ? scale(fa, 1.f / fl) : Vec{0, 0, 1};
    out.upper_axis = ul > 1e-5f ? scale(ua, 1.f / ul) : Vec{0, 0, 1};
    float rigid[16];
    multiply_affine(rigid, ha, out.delta[kLower]);  // the hand carried rigidly by the forearm
    out.twist = fl > 1e-5f ? (tune.correction ? wrist_twist(rigid,nh,out.fore_axis) : twist_angle(rigid, nh, out.fore_axis)) : 0.f;
    if (!std::isfinite(out.twist)) out.twist = 0.f;
  }
  if (prev_bend) {
    const Vec axis = sub(sol.wrist, s);
    const float l = length(axis);
    if (l > 1e-4f) {
      const Vec n = scale(axis, 1.f / l);
      Vec bend = sub(sub(sol.elbow, s), scale(n, dot(sub(sol.elbow, s), n)));
      const float bl = length(bend);
      if (bl > 1e-4f) *prev_bend = scale(bend, 1.f / bl);
    }
  }
  return true;
}

// ---- Rest-pose rig (2026-10-24 [HEADSET]: "the game moves the 3rd person body in different ways that makes the ik
// look weird, arms stretch unnaturally ... decouple the native animation ... like the ik on foot"). As the on-foot
// clean arm rig (skin_palette_probe run 54/55): the arm is not taken from the animation each frame. A rest arm (clavicle,
// upper arm, forearm, hand, knuckles) captured once per mount in the eye / level view frame is placed in this frame's
// view frame, solved to the controller, and the result is mapped onto the animated bones (delta = inv(animated) *
// result), so the riding animation only decides which bones exist, not where the arm is.
enum RestJoint { kRShoulder, kRUpper, kRLower, kRHand, kRIndex, kRMiddle, kRPinky, kRestJoints };
struct Rest {
  float local[2][kRestJoints][16]{};  // per side, joint matrices in the capture frame (world = local * frame)
  bool valid[2]{};
};
// Mirror articulation in geometric space, carrying the left bone axes and hand rigidly. Never copy right matrices.
inline bool symmetric_rest(Rest& rest) {
  if(!rest.valid[0] || !rest.valid[1]) return false;
  Rest next=rest;
  const auto mirror=[](const float* m){return Vec{-m[12],m[13],m[14]};};
  const Vec upper=mirror(rest.local[1][kRUpper]);
  const Vec delta=skin_arm_ik::sub(upper,skin_arm_ik::position(rest.local[0][kRUpper]));
  for(int j=kRUpper;j<kRestJoints;++j) {
    next.local[0][j][12]+=delta.x; next.local[0][j][13]+=delta.y; next.local[0][j][14]+=delta.z;
  }
  next.local[0][kRUpper][12]=upper.x; next.local[0][kRUpper][13]=upper.y;
  next.local[0][kRUpper][14]=upper.z;
  const Vec clavicle=mirror(rest.local[1][kRShoulder]);
  next.local[0][kRShoulder][12]=clavicle.x; next.local[0][kRShoulder][13]=clavicle.y;
  next.local[0][kRShoulder][14]=clavicle.z;
  using namespace skin_arm_ik;
  const Vec a=sub(position(next.local[0][kRLower]),upper);
  const Vec c=sub(position(next.local[0][kRHand]),position(next.local[0][kRLower]));
  const Vec ra=sub(position(rest.local[1][kRLower]),position(rest.local[1][kRUpper]));
  const Vec rc=sub(position(rest.local[1][kRHand]),position(rest.local[1][kRLower]));
  const auto reflect=[](Vec v){return Vec{-v.x,v.y,v.z};};
  if(length(a)<1e-5f || length(c)<1e-5f || length(ra)<1e-5f || length(rc)<1e-5f) return false;
  const Vec na=scale(reflect(ra),length(a)/length(ra)),nc=scale(reflect(rc),length(c)/length(rc));
  const Vec elbow=add(upper,na),normal=cross(a,c),new_normal=cross(na,nc);
  float lower[16],inverse[16],carry[16];
  limb_matrix(lower,next.local[0][kRLower],c,nc,normal,new_normal,elbow);
  if(!inverse_affine(inverse,next.local[0][kRLower])) return false;
  multiply_affine(carry,inverse,lower);
  for(int j=kRHand;j<kRestJoints;++j) {
    float moved[16]; multiply_affine(moved,next.local[0][j],carry);
    std::memcpy(next.local[0][j],moved,64);
  }
  float upper_matrix[16];
  limb_matrix(upper_matrix,next.local[0][kRUpper],a,na,normal,new_normal,upper);
  std::memcpy(next.local[0][kRUpper],upper_matrix,64);
  std::memcpy(next.local[0][kRLower],lower,64);
  for(const auto& arm:next.local) for(const auto& m:arm) for(float v:m) if(!std::isfinite(v)) return false;
  rest=next;
  return true;
}

// Transform-quality measurements; compare before/after to permit the original mesh scale and handedness.
struct Quality { float axis[3]{}, orthogonality=0.f, determinant=0.f; bool valid=false; };
inline Quality quality(const float* m) {
  using namespace skin_arm_ik;
  Quality q;
  for(int i=0;i<16;++i) if(!std::isfinite(m[i])) return q;
  for(int i=0;i<3;++i) {q.axis[i]=length(row(m,i)); if(q.axis[i]<1e-5f) return q;}
  q.determinant=dot(row(m,0),cross(row(m,1),row(m,2)));
  const float product=q.axis[0]*q.axis[1]*q.axis[2];
  if(!std::isfinite(product) || std::fabs(q.determinant)<1e-5f*product) return q;
  for(int i=0;i<3;++i) for(int j=i+1;j<3;++j)
    q.orthogonality=std::max(q.orthogonality,std::fabs(dot(row(m,i),row(m,j))/(q.axis[i]*q.axis[j])));
  q.valid=std::isfinite(q.orthogonality) && std::isfinite(q.determinant);
  return q;
}
inline bool preserves_geometry(const float* before,const float* after) {
  const auto a=quality(before), b=quality(after);
  if(!a.valid || !b.valid) return false;
  for(int k=0;k<3;++k) if(std::fabs(b.axis[k]/a.axis[k]-1.f)>0.02f) return false;
  return std::fabs(b.determinant/a.determinant-1.f)<=0.03f &&
         std::fabs(b.orthogonality-a.orthogonality)<=0.02f;
}
inline bool twist_helper(const char* name) {
  return name && (!std::strncmp(name,"L_lowerArm_twist",16) || !std::strncmp(name,"R_lowerArm_twist",16) ||
                 !std::strncmp(name,"L_upperArm_twist",16) || !std::strncmp(name,"R_upperArm_twist",16));
}
// Arm skin helpers must not retain hover/boost animation relative to the solved segment. Hand/finger helpers remain
// on their existing hand path so native finger poses and later gestures remain available.
inline bool arm_skin_helper(const char* name) {
  int side=-1; const auto r=role(name,side);
  return (r==kUpper || r==kLower || r==kShoulder) &&
    (std::strstr(name,"twist") || std::strstr(name,"expand") || std::strstr(name,"driver") || std::strstr(name,"bend"));
}
// frame: rows right, up, forward (level view basis), translation = the eye. Row-vector: world = local * frame.
inline bool capture_rest(Rest& rest, unsigned hand, const float* const (&joints)[kRestJoints], const float* frame) {
  using namespace skin_arm_ik;
  float inv[16];
  if (hand > 1 || !inverse_affine(inv, frame)) return false;
  Rest next=rest;
  for (int k = 0; k < kRestJoints; ++k) {
    if (!joints[k] || !quality(joints[k]).valid) return false;
    multiply_affine(next.local[hand][k], joints[k], inv);
    if(!quality(next.local[hand][k]).valid) return false;
  }
  next.valid[hand] = true;
  rest=next;
  return true;
}
// Solve on the rest arm placed in `frame`; out.delta[role] then maps each ANIMATED joint (anim[role], same space and
// moment as the mesh) onto the result: new = old * inv(anim) * rest_world * rest_delta. anim[kShoulder] may be null
// (no clavicle in this mesh: its delta is identity).
inline bool solve_rest(const Rest& rest, unsigned hand, const float* frame, const float* const (&anim)[4],
                       const float* grip, const float* basis, const Tune& tune, Arm& out,
                        Vec* prev_bend = nullptr,const float* wrist_override=nullptr) {
  using namespace skin_arm_ik;
  if (hand > 1 || !rest.valid[hand] || !anim[kUpper] || !anim[kLower] || !anim[kHand]) return false;
  float w[kRestJoints][16];
  for (int k = 0; k < kRestJoints; ++k) multiply_affine(w[k], rest.local[hand][k], frame);
  const Vec shoulder_offsets=offsets_for_hand(tune,hand).shoulder;
  if (shoulder_offsets.x != 0.f || shoulder_offsets.y != 0.f || shoulder_offsets.z != 0.f) {
    // Tuning: the whole rest arm moves in the view frame (out is mirrored: + = away from the body on both sides).
    const Vec offsets=shoulder_offsets;
    const float o=offsets.x,u=offsets.y,f=offsets.z;
    for (int k = 0; k < kRestJoints; ++k)
      for (int c = 0; c < 3; ++c) w[k][12 + c] += o * frame[c] + u * frame[4 + c] + f * frame[8 + c];
  }
  Joints j;
  j.shoulder = w[kRShoulder]; j.upper = w[kRUpper]; j.lower = w[kRLower]; j.hand = w[kRHand];
  j.index = position(w[kRIndex]); j.middle = position(w[kRMiddle]); j.pinky = position(w[kRPinky]);
  Arm r{};
  if (!solve_joints(j, hand, grip, basis, tune, r, tune.correction?nullptr:prev_bend,wrist_override)) return false;
  const float* rest_world[4] = {w[kRUpper], w[kRLower], w[kRHand], w[kRShoulder]};  // Role order
  for (int role = 0; role < 4; ++role) {
    if (!anim[role]) { for (int k = 0; k < 16; ++k) out.delta[role][k] = (k % 5 == 0) ? 1.f : 0.f; continue; }
    float inv[16], placed[16], result[16];
    if (!inverse_affine(inv, anim[role])) return false;
    multiply_affine(placed, rest_world[role], r.delta[role]);  // the solved joint
    multiply_affine(result, inv, placed);
    for (float v : result) if (!std::isfinite(v)) return false;
    std::memcpy(out.delta[role], result, sizeof(result));
  }
  out.elbow = r.elbow; out.wrist = r.wrist; out.clamped = r.clamped; out.reach = r.reach;
  out.side=hand;
  out.fore_axis = r.fore_axis; out.upper_axis = r.upper_axis;
  // Twist share: the solved hand's roll relative to a hand carried rigidly by the solved forearm (as solve_joints), so
  // the forearm bones (placed from the animated forearm) still blend smoothly into the wrist.
  out.twist = r.twist;
  return true;
}

// The 192 pose (cell-local world rows) as joints.
inline Joints pose_joints(const float (*pose)[16], unsigned hand, bool with_shoulder = true) {
  using skin_arm_ik::position;
  const ArmBones& b = kArm[hand & 1];
  Joints j;
  j.shoulder = with_shoulder ? pose[b.shoulder] : nullptr;
  j.upper = pose[b.upper]; j.lower = pose[b.lower]; j.hand = pose[b.hand];
  j.index = position(pose[b.index]); j.middle = position(pose[b.middle]); j.pinky = position(pose[b.pinky]);
  return j;
}
inline bool solve(const float (*pose)[16], unsigned hand, const float* grip, const float* basis,
                  const Tune& tune, Arm& out, Vec* prev_bend = nullptr) {
  if (hand > 1 || !pose) return false;
  return solve_joints(pose_joints(pose, hand), hand, grip, basis, tune, out, prev_bend);
}

// Apply a role delta to one bone matrix in place: m = m * delta.
inline bool apply(float* m, const float* delta) {
  float r[16];
  skin_arm_ik::multiply_affine(r, m, delta);
  for (float v : r) if (!std::isfinite(v)) return false;
  std::memcpy(m, r, sizeof(r));
  return true;
}

// Harness grab (2026-10-24 [AUTHOR]: "when we grip on the harness the arm takes the flat normal pose ... smooth"): the
// arm is blended from the IK result back to the game's riding pose. m = old blended towards old * delta by w (0 = the
// game's pose, 1 = full IK): rotation along the delta's axis by w * its angle, position linearly. Rows may carry
// scale (rotation keeps it).
inline bool blend(float* m, const float* delta, float w) {
  using namespace skin_arm_ik;
  if (!(w > 0.f)) return true;
  if (w >= 1.f) return apply(m, delta);
  float n[16];
  multiply_affine(n, m, delta);
  for (float v : n) if (!std::isfinite(v)) return false;
  // Row-vector rotation part M of the delta (v' = v M) is the column rotation R = M^T.
  const float tr = delta[0] + delta[5] + delta[10];
  const float angle = std::acos(std::clamp((tr - 1.f) * 0.5f, -1.f, 1.f));
  Vec axis{delta[6] - delta[9], delta[8] - delta[2], delta[1] - delta[4]};
  const float al = length(axis);
  float r[16];
  std::memcpy(r, m, sizeof(r));
  if (al > 1e-5f) {
    axis = scale(axis, 1.f / al);
    for (int i = 0; i < 3; ++i) {
      const Vec v = rotate_axis(row(m, i), axis, w * angle);
      r[i * 4] = v.x; r[i * 4 + 1] = v.y; r[i * 4 + 2] = v.z;
    }
  } else if (angle > 3.f && w >= 0.5f) {  // half-turn: no stable axis, switch at the midpoint
    for (int i = 0; i < 3; ++i) for (int k = 0; k < 3; ++k) r[i * 4 + k] = n[i * 4 + k];
  }
  for (int k = 0; k < 3; ++k) r[12 + k] = m[12 + k] + (n[12 + k] - m[12 + k]) * w;
  for (float v : r) if (!std::isfinite(v)) return false;
  std::memcpy(m, r, sizeof(r));
  return true;
}

// Rebase a shared solved segment into an accessory palette's cell, independent of its native animation/root.
struct AttachmentPose {
  float world[4][16]{};
  Arm arm;
  Vec body{};
  bool valid=false;
};
// Publish complete output frames, never a retained delta paired with newer native animation joints.
inline bool capture_attachment(AttachmentPose& out,const float* const (&native)[4],const Arm& solved,
                               Vec cell_origin,Vec body) {
  AttachmentPose next; next.body=body; next.arm=solved;
  for(int role=0;role<4;++role) {
    if(!native[role]) return false;
    skin_arm_ik::multiply_affine(next.world[role],native[role],solved.delta[role]);
    if(!quality(next.world[role]).valid) return false;
    for(int k=0;k<3;++k) next.world[role][12+k]+=k==0?cell_origin.x:k==1?cell_origin.y:cell_origin.z;
  }
  const Vec wrist=skin_arm_ik::add(solved.wrist,cell_origin);
  if(skin_arm_ik::length(skin_arm_ik::sub(skin_arm_ik::position(next.world[kHand]),wrist))>0.002f) return false;
  next.valid=true; out=next;
  return true;
}
inline bool attachment_frame(float* out,const AttachmentPose& pose,Role role,Vec body,Vec cell_origin) {
  if(!pose.valid) return false;
  std::memcpy(out,pose.world[role],64);
  const Vec shift=skin_arm_ik::sub(skin_arm_ik::sub(body,pose.body),cell_origin);
  out[12]+=shift.x; out[13]+=shift.y; out[14]+=shift.z;
  return quality(out).valid;
}
inline bool attachment_delta(float* out,const float* parent,const float* reference,const float* solved_delta,Vec cell_shift) {
  float target[16],inverse[16];
  skin_arm_ik::multiply_affine(target,reference,solved_delta);
  target[12]+=cell_shift.x; target[13]+=cell_shift.y; target[14]+=cell_shift.z;
  if(!quality(target).valid || !skin_arm_ik::inverse_affine(inverse,parent)) return false;
  skin_arm_ik::multiply_affine(out,inverse,target);
  return quality(out).valid;
}
inline bool fixed_helper(float* out,const float* animated,const float* local,const float* parent,
                         const Arm& arm,Role role,float along,bool is_twist,float weight,const Tune& tune) {
  float placed[16],target[16],inverse[16],delta[16];
  skin_arm_ik::multiply_affine(placed,local,parent);
  skin_arm_ik::multiply_affine(target,placed,arm.delta[role]);
  if(is_twist) twist(target,arm,role,along,tune);
  if(!quality(target).valid || !skin_arm_ik::inverse_affine(inverse,animated)) return false;
  skin_arm_ik::multiply_affine(delta,inverse,target);
  std::memcpy(out,animated,64);
  return blend(out,delta,weight) && preserves_geometry(animated,out);
}

// Commit a whole arm together; on any invalid bone leave the caller's source array intact.
inline bool transform_arm(float (*out)[16],const float (*source)[16],unsigned count,
                          const signed char* roles,const signed char* sides,const char* twists,
                          const float* along,unsigned hand,const Arm& arm,float weight,const Tune& tune,
                          unsigned* bad_bone=nullptr,Quality* rejected=nullptr) {
  bool valid=true;
  for(unsigned i=0;i<count;++i) {
    if(sides[i]!=static_cast<int>(hand) || roles[i]<0) continue;
    std::memcpy(out[i],source[i],64);
    const Role r=static_cast<Role>(roles[i]);
    bool ok=blend(out[i],arm.delta[r],weight);
    if(ok && twists[i]) twist(out[i],arm,r,along[i]*weight,tune);
    ok=ok && preserves_geometry(source[i],out[i]);
    if(!ok) {
      if(valid) {if(bad_bone) *bad_bone=i; if(rejected) *rejected=quality(out[i]);}
      valid=false;
    }
  }
  if(!valid) for(unsigned i=0;i<count;++i)
    if(sides[i]==static_cast<int>(hand) && roles[i]>=0) std::memcpy(out[i],source[i],64);
  return valid;
}

// Harness grab state for one hand. A grip press starts a grab only within kGrabRadius of that hand's harness point;
// while held it lasts until release (the hand may move). The arm's IK weight eases to 0 (game pose holding the harness)
// over kBlendSeconds and back to 1 on release, with a smoothstep so the hand neither jerks off nor onto the harness.
constexpr float kGrabRadius = 0.15f;
constexpr float kBlendSeconds = 0.25f;
struct Grab {
  bool held = false;
  bool was_pressed = false;
  float t = 1.f;  // linear: 1 = IK, 0 = game pose
  // Returns true on a grab (+1 edge) via `edge` = +1 grab, -1 release, 0 none.
  void step(bool pressed, bool in_reach, bool allowed, float dt, int& edge) {
    edge = 0;
    if (!allowed) { if (held) edge = -1; held = false; }
    else if (pressed && !was_pressed && in_reach) { held = true; edge = 1; }
    else if (!pressed && held) { held = false; edge = -1; }
    was_pressed = pressed;
    const float d = std::clamp(std::isfinite(dt) ? dt : 0.f, 0.f, 0.1f) / kBlendSeconds;
    t = held ? std::max(0.f, t - d) : std::min(1.f, t + d);
  }
  float ik_weight() const { return t * t * (3.f - 2.f * t); }
};

// Harness grip recorder (2026-10-24): the game's own riding pose (with a hands-on-harness animation installed) puts the
// hand bones on the harness. Their position and orientation are averaged in the rider's pelvis frame (192 bone 6; the
// pelvis sits on the saddle), so a later grab can snap the tracked hand to the same place. Samples are taken only
// while both hands are steady relative to the pelvis.
constexpr int kPelvis = 6;
constexpr int kPalm[2] = {63, 101};
struct GripStat {
  float mean[3] = {}, m2[3] = {};
  float axis_fwd[3] = {}, axis_up[3] = {};  // averaged hand rows 2 and 1 in pelvis axes (unnormalised sums)
  unsigned n = 0;
  void add(const float* p, const float* fwd, const float* up) {
    ++n;
    for (int k = 0; k < 3; ++k) {
      const float d = p[k] - mean[k];
      mean[k] += d / static_cast<float>(n);
      m2[k] += d * (p[k] - mean[k]);
      axis_fwd[k] += fwd[k];
      axis_up[k] += up[k];
    }
  }
  float spread() const {  // RMS distance from the mean, metres
    return n > 1 ? std::sqrt((m2[0] + m2[1] + m2[2]) / static_cast<float>(n - 1)) : 0.f;
  }
};
// Pelvis-local position / direction (pelvis rows: may carry scale; divided out like mount_anchor::to_local).
inline bool to_pelvis(const float* pelvis, const float* world_point, float* out) {
  for (int r = 0; r < 3; ++r) {
    const float* a = pelvis + 4 * r;
    const float n2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    if (!(n2 > 1e-6f) || !std::isfinite(n2)) return false;
    out[r] = (a[0] * (world_point[0] - pelvis[12]) + a[1] * (world_point[1] - pelvis[13]) +
              a[2] * (world_point[2] - pelvis[14])) / n2;
  }
  return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
}
inline bool dir_to_pelvis(const float* pelvis, const float* world_dir, float* out) {
  for (int r = 0; r < 3; ++r) {
    const float* a = pelvis + 4 * r;
    const float n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!(n > 1e-3f)) return false;
    out[r] = (a[0] * world_dir[0] + a[1] * world_dir[1] + a[2] * world_dir[2]) / n;
  }
  return true;
}
}  // namespace rider_ik
