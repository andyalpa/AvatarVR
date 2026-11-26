#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
namespace skin_arm_ik {
struct Vec {float x,y,z;};
inline Vec add(Vec a,Vec b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec sub(Vec a,Vec b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec scale(Vec a,float s){return {a.x*s,a.y*s,a.z*s};}
inline float dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec cross(Vec a,Vec b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline float length(Vec a){return std::sqrt(dot(a,a));}
inline bool finite(Vec a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline Vec position(const float* m){return {m[12],m[13],m[14]};}
struct Solution {Vec elbow,wrist;bool clamped;};
struct WristCalibration {float relative[3][3]{};bool valid=false;};
inline Vec row(const float* matrix,unsigned index){return {matrix[index*4],matrix[index*4+1],matrix[index*4+2]};}
inline bool grip_basis(const float* matrix){
  for(unsigned i=0;i<3;++i){const Vec axis=row(matrix,i);if(!finite(axis)||std::fabs(length(axis)-1.f)>0.03f) return false;}
  return std::fabs(dot(row(matrix,0),row(matrix,1)))<0.03f &&
    std::fabs(dot(row(matrix,0),row(matrix,2)))<0.03f && std::fabs(dot(row(matrix,1),row(matrix,2)))<0.03f;
}
inline bool calibrate(WristCalibration& out,const float* hand,const float* grip){
  if(!grip_basis(grip)) return false;
  WristCalibration next;
  for(unsigned i=0;i<3;++i){const Vec axis=row(hand,i);const float n=length(axis);
    if(!finite(axis)||!std::isfinite(n)||n<0.05f||n>4.f) return false;
    for(unsigned j=0;j<3;++j) next.relative[i][j]=dot(axis,row(grip,j));
  }
  next.valid=true;out=next;return true;
}
inline bool orient(float* hand,const WristCalibration& calibration,const float* grip){
  if(!calibration.valid||!grip_basis(grip)) return false;
  float basis[3][3]{};
  for(unsigned i=0;i<3;++i){Vec axis{0,0,0};
    for(unsigned j=0;j<3;++j){if(!std::isfinite(calibration.relative[i][j])) return false;
      axis=add(axis,scale(row(grip,j),calibration.relative[i][j]));}
    if(!finite(axis)) return false;
    basis[i][0]=axis.x;basis[i][1]=axis.y;basis[i][2]=axis.z;
  }
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) hand[i*4+j]=basis[i][j];
  return true;
}
inline bool solve(Vec shoulder,Vec target,Vec pole,float upper,float lower,Solution& out){
  if(!finite(shoulder)||!finite(target)||!finite(pole)||!std::isfinite(upper)||!std::isfinite(lower)||
     upper<0.01f||lower<0.01f||upper>2.f||lower>2.f) return false;
  const Vec delta=sub(target,shoulder);const float distance=length(delta);
  if(!std::isfinite(distance)||distance<1e-6f) return false;
  const Vec axis=scale(delta,1.f/distance);
  const float reach=std::clamp(distance,std::fabs(upper-lower)+1e-5f,upper+lower-1e-5f);
  Vec bend=sub(pole,scale(axis,dot(pole,axis)));
  if(length(bend)<1e-6f){const Vec fallback=std::fabs(axis.y)<0.9f?Vec{0,1,0}:Vec{1,0,0};bend=sub(fallback,scale(axis,dot(fallback,axis)));}
  const float along=(upper*upper-lower*lower+reach*reach)/(2*reach);
  const float height=std::sqrt(std::max(0.f,upper*upper-along*along));
  out.elbow=add(shoulder,add(scale(axis,along),scale(bend,height/length(bend))));
  out.wrist=add(shoulder,scale(axis,reach));out.clamped=std::fabs(reach-distance)>1e-5f;
  return finite(out.elbow)&&finite(out.wrist);
}
inline Vec rotate(Vec value,Vec from,Vec to){
  const Vec a=scale(from,1.f/length(from)),b=scale(to,1.f/length(to));
  const float cosine=std::clamp(dot(a,b),-1.f,1.f);
  if(cosine>1.f-1e-7f) return value;
  Vec axis=cross(a,b);
  if(cosine<-1.f+1e-7f) axis=cross(a,std::fabs(a.y)<0.9f?Vec{0,1,0}:Vec{1,0,0});
  axis=scale(axis,1.f/length(axis));
  const float sine=std::sqrt(std::max(0.f,1.f-cosine*cosine));
  return add(add(scale(value,cosine),scale(cross(axis,value),sine)),scale(axis,dot(axis,value)*(1.f-cosine)));
}
inline void matrix(float* out,const float* source,Vec from,Vec to,Vec pos){
  std::memcpy(out,source,16*sizeof(float));
  for(unsigned row=0;row<3;++row){Vec v=rotate({source[row*4],source[row*4+1],source[row*4+2]},from,to);out[row*4]=v.x;out[row*4+1]=v.y;out[row*4+2]=v.z;}
  out[12]=pos.x;out[13]=pos.y;out[14]=pos.z;
}
// Row-vector affine helpers (translation in row 3). Column 3 of the input is ignored and preserved by follow().
inline bool inverse_affine(float* out,const float* m){
  const float a=m[0],b=m[1],c=m[2],d=m[4],e=m[5],f=m[6],g=m[8],h=m[9],i=m[10];
  const float A=e*i-f*h,B=f*g-d*i,C=d*h-e*g,det=a*A+b*B+c*C;
  if(!std::isfinite(det)||std::fabs(det)<1e-8f) return false;
  const float s=1.f/det;
  float r[16]{A*s,(c*h-b*i)*s,(b*f-c*e)*s,0, B*s,(a*i-c*g)*s,(c*d-a*f)*s,0, C*s,(b*g-a*h)*s,(a*e-b*d)*s,0, 0,0,0,1};
  for(unsigned j=0;j<3;++j) r[12+j]=-(m[12]*r[j]+m[13]*r[4+j]+m[14]*r[8+j]);
  std::memcpy(out,r,sizeof(r));return true;
}
inline void multiply_affine(float* out,const float* a,const float* b){
  float r[16];std::memcpy(r,a,sizeof(r));
  for(unsigned row=0;row<4;++row) for(unsigned j=0;j<3;++j)
    r[row*4+j]=a[row*4]*b[j]+a[row*4+1]*b[4+j]+a[row*4+2]*b[8+j]+(row==3?b[12+j]:0.f);
  std::memcpy(out,r,sizeof(r));
}
// The palette input is flat world-space bones: children are not re-derived from moved parents. Assign same-side
// bones lying on a solved segment or around the hand (fingers, twist helpers, held props) to the joint they ride on.
// side[]: 0 left, 1 right, -1 neither (from L_/R_ name prefixes). owner[] receives upper/lower/wrist index or -1.
inline float segment_t(Vec p,Vec a,Vec b,float& distance){
  const Vec ab=sub(b,a);const float l2=dot(ab,ab);
  const float t=l2>1e-10f?dot(sub(p,a),ab)/l2:0.f;
  distance=length(sub(p,add(a,scale(ab,std::clamp(t,0.f,1.f)))));return t;
}
inline unsigned classify(int (&owner)[211],const float (&source)[211][16],unsigned hand,const signed char (&side)[211]){
  const unsigned upper=hand?113:14,lower=upper+1,wrist=upper+2;
  const Vec s=position(source[upper]),e=position(source[lower]),w=position(source[wrist]);
  const Vec forearm=sub(w,e);const float forearm_length=length(forearm);
  unsigned assigned=0;
  for(unsigned bone=0;bone<211;++bone){
    owner[bone]=-1;
    if(side[bone]!=static_cast<signed char>(hand)||bone==upper||bone==lower||bone==wrist) continue;
    const Vec p=position(source[bone]);if(!finite(p)) continue;
    float d_lower=0,d_upper=0;
    const float t_lower=segment_t(p,e,w,d_lower),t_upper=segment_t(p,s,e,d_upper);
    const float beyond=forearm_length>1e-5f?dot(sub(p,w),forearm)/forearm_length:-1.f;
    if(d_lower<0.05f&&t_lower>=0.f&&t_lower<0.97f) owner[bone]=static_cast<int>(lower);
    else if(d_upper<0.05f&&t_upper>0.08f&&t_upper<=1.f) owner[bone]=static_cast<int>(upper);
    else if(length(sub(p,w))<0.25f&&beyond>-0.02f) owner[bone]=static_cast<int>(wrist);
    if(owner[bone]>=0) ++assigned;
  }
  return assigned;
}
// Name rule from the live 211-name dump (P_BODY_01_FP_M): side from L_/R_; *_upperArm_* helpers ride the upper arm,
// *_lowerArm_* helpers the forearm, every other same-side bone (fingers, thumb, hand_*, handWeapon, attachment) the hand.
// Shoulder bones and the three solved joints themselves are not followers. Returns -1 none, 0 upper, 1 lower, 2 hand.
inline int name_role(const char* name){
  if(!name||(std::strncmp(name,"L_",2)&&std::strncmp(name,"R_",2))) return -1;
  const char* rest=name+2;
  if(std::strstr(rest,"shoulder")||!std::strcmp(rest,"upperArm")||!std::strcmp(rest,"lowerArm")||!std::strcmp(rest,"hand")) return -1;
  if(!std::strncmp(rest,"upperArm_",9)) return 0;
  if(!std::strncmp(rest,"lowerArm_",9)) return 1;
  return 2;
}
inline unsigned owners_from_roles(int (&owner)[211],const signed char (&side)[211],const signed char (&role)[211],unsigned hand){
  const int upper=hand?113:14;unsigned assigned=0;
  for(unsigned bone=0;bone<211;++bone){
    owner[bone]=(side[bone]==static_cast<signed char>(hand)&&role[bone]>=0&&role[bone]<=2)?upper+role[bone]:-1;
    if(owner[bone]==static_cast<int>(bone)) owner[bone]=-1;
    assigned+=owner[bone]>=0;
  }
  return assigned;
}
// out[bone] = source[bone] * inverse(source[owner]) * out[owner]: keeps each follower rigid relative to its joint.
inline unsigned follow(float (&out)[211][16],const float (&source)[211][16],const int (&owner)[211]){
  unsigned moved=0;
  for(unsigned bone=0;bone<211;++bone){
    const int o=owner[bone];if(o<0||o>=211||static_cast<unsigned>(o)==bone) continue;
    float inverse[16],delta[16],result[16];
    if(!inverse_affine(inverse,source[o])) continue;
    multiply_affine(delta,inverse,out[o]);multiply_affine(result,source[bone],delta);
    bool ok=true;for(float v:result) ok=ok&&std::isfinite(v);
    if(!ok) continue;
    std::memcpy(out[bone],result,sizeof(result));++moved;
  }
  return moved;
}
// Anatomical hand alignment (2026-10-15): replaces the activation-pose calibration, which baked whatever pose the user
// held at switch-on. Game-space grip rows: 0 right, 1 up, 2 forward (OpenXR -z, the pointing direction), 3 palm origin.
// Hand frame from the animated bones: forward = wrist -> middle knuckle, side = pinky knuckle -> index knuckle.
// Target: forward along the (tuned) grip forward, index side along the grip's up (index above pinky on the handle).
struct HandTune {
  float pitch = 0.f, yaw = 0.f, roll = 0.f;          // degrees, in grip space; +pitch raises the fingers
  float forward = 0.f, up = 0.f, side = 0.f;         // metres, in the body frame (body_offset), added to the wrist target
  float palm = 0.75f;                                // wrist sits this share of wrist->knuckle behind the grip origin
};
inline void tuned_grip(float* out, const float* grip, const HandTune& t) {
  const float d = 0.0174532925f, cy = std::cos(t.yaw * d), sy = std::sin(t.yaw * d), cp = std::cos(-t.pitch * d),
              sp = std::sin(-t.pitch * d), cr = std::cos(t.roll * d), sr = std::sin(t.roll * d);
  // Local rotation rows (yaw about up, then pitch about right, then roll about forward), row-vector convention.
  const float ry[3][3] = {{cy, 0, -sy}, {0, 1, 0}, {sy, 0, cy}};
  const float rx[3][3] = {{1, 0, 0}, {0, cp, sp}, {0, -sp, cp}};
  const float rz[3][3] = {{cr, sr, 0}, {-sr, cr, 0}, {0, 0, 1}};
  float a[3][3], r[3][3];
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) a[i][j] = rz[i][0] * rx[0][j] + rz[i][1] * rx[1][j] + rz[i][2] * rx[2][j];
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r[i][j] = a[i][0] * ry[0][j] + a[i][1] * ry[1][j] + a[i][2] * ry[2][j];
  std::memcpy(out, grip, 16 * sizeof(float));
  for (int i = 0; i < 3; ++i) for (int k = 0; k < 3; ++k)
    out[i * 4 + k] = r[i][0] * grip[k] + r[i][1] * grip[4 + k] + r[i][2] * grip[8 + k];
}
// Position offsets in the body frame (root rows: 0 right, 1 up, 2 forward), so they do not swing with the controller's
// rotation (2026-10-15: grip-space offsets moved the hand towards the face when the palm turned).
inline Vec body_offset(Vec target, const float* basis, const HandTune& t) {
  const Vec r = row(basis, 0), u = row(basis, 1), f = row(basis, 2);
  const float lr = length(r), lu = length(u), lf = length(f);
  if (lr < 1e-4f || lu < 1e-4f || lf < 1e-4f) return target;
  return add(target, add(scale(r, t.side / lr), add(scale(u, t.up / lu), scale(f, t.forward / lf))));
}
// Arm scaling (2026-10-15, [AUTHOR] "a small person inside a larger character"): the controller's offset from the
// user's real shoulder is replayed from the Na'vi shoulder, scaled by Na'vi reach / user reach, so a fully stretched
// real arm is a fully stretched Na'vi arm and motion keeps its proportions. The real shoulder is estimated from the
// headset centre in the body frame (basis rows: 0 right, 1 up, 2 forward).
struct ArmScale {
  bool enabled = true;
  float user_length = 0.62f;  // real shoulder -> controller grip with the arm straight, metres
  float drop = 0.24f;         // eyes -> shoulder joint, metres
  float width = 0.38f;        // shoulder joint to shoulder joint, metres
  float back = 0.06f;         // shoulder joints behind the eyes, metres
  float reach = 1.f;          // extra multiplier
};
inline Vec real_shoulder(Vec head, const float* basis, unsigned hand, const ArmScale& a) {
  const Vec r = row(basis, 0), u = row(basis, 1), f = row(basis, 2);
  const float lr = length(r), lu = length(u), lf = length(f);
  if (lr < 1e-4f || lu < 1e-4f || lf < 1e-4f) return head;
  const float side = hand ? 0.5f * a.width : -0.5f * a.width;
  return add(head, add(scale(r, side / lr), add(scale(u, -a.drop / lu), scale(f, -a.back / lf))));
}
// navi_length: Na'vi shoulder -> grip-equivalent point (upper + lower arm + wrist-to-palm). Returns the scaled point.
inline Vec scale_reach(Vec grip, Vec user_shoulder, Vec navi_shoulder, float navi_length, const ArmScale& a, float* used) {
  const float factor = navi_length / std::max(0.3f, a.user_length) * a.reach;
  if (used) *used = factor;
  if (!std::isfinite(factor) || factor <= 0.f) return grip;
  return add(navi_shoulder, scale(sub(grip, user_shoulder), factor));
}
// Model-space hand target (2026-10-16): the controller is taken
// relative to the headset (grip - head, both from the same sample, so the game camera's bob and position cancel),
// minus the user's calibrated shoulder offset from the headset (body axes of `basis`), scaled to the Na'vi arm and
// added to the Na'vi shoulder: target = navi_shoulder + (hmd_rel - user_shoulder_rel) * factor (factor 1 when off).
inline Vec model_hand_point(Vec hmd_rel, const float* basis, unsigned hand, const ArmScale& a, Vec navi_shoulder,
                            float navi_length, float* used) {
  const Vec user_shoulder = real_shoulder(Vec{0.f, 0.f, 0.f}, basis, hand, a);
  if (a.enabled) return scale_reach(hmd_rel, user_shoulder, navi_shoulder, navi_length, a, used);
  if (used) *used = 1.f;
  return add(navi_shoulder, sub(hmd_rel, user_shoulder));
}
// Hybrid reach (2026-11-13, [AUTHOR] "when I move the arms near my chest or head the hands go beyond"). [LIVE
// IK_MOTION_TIMING 2026-11-11] the uniform factor was ~1.8 while the eyes are 1:1, so a hand at the face drew ~17 cm above
// the eye. Near the body the hand is placed 1:1 where the eyes see the real hand (eye + hmd_rel); towards full extension the
// gain (Na'vi metres per real metre, radially from the shoulder) eases from 1 to a constant G over [near, far] and stays G
// beyond, with G chosen so a fully stretched real arm still lands where the Na'vi factor K put it. A constant outer gain
// is the lowest peak gain that reaches K (a smoothstep factor peaked ~4x the hand speed in the blend [unit test 121]).
// The anchor moves from the user's shoulder (seen 1:1) to the Na'vi shoulder over [near, 1]. The offset is scaled along
// its own direction, so moving the hand away from the shoulder always moves the target away (no fold-back).
struct HybridReach {
  bool enabled = false;
  float near_share = 0.40f;  // share of the user's arm length (shoulder -> grip) below which the hand is 1:1
  float far_share = 0.60f;   // share by which the gain has reached its outer value
};
inline float smooth01(float e0, float e1, float x) {
  if (!(e1 > e0)) return x >= e1 ? 1.f : 0.f;
  const float t = std::clamp((x - e0) / (e1 - e0), 0.f, 1.f);
  return t * t * (3.f - 2.f * t);
}
// Outer gain G for full factor k: k = a + s(1+G)/2 + G(1-a-s), s = far - near.
inline float hybrid_outer_gain(float k, float a, float s) {
  const float d = 1.f - a - 0.5f * s;
  if (!(d > 0.05f) || !std::isfinite(k)) return k;
  return std::max(1.f, (k - a - 0.5f * s) / d);
}
// Mapped radial distance (in user arm lengths) for real distance r: integral of the gain.
inline float hybrid_radial(float r, float a, float s, float g) {
  if (r <= a) return r;
  if (!(s > 1e-4f)) return a + g * (r - a);
  if (r < a + s) { const float u = (r - a) / s; return r + (g - 1.f) * s * (u * u * u - 0.5f * u * u * u * u); }
  return a + 0.5f * s * (1.f + g) + g * (r - a - s);
}
// weight 0 = 1:1 with the eyes, 1 = the model_hand_point anchor. eye: the drawn eye centre in the space of navi_shoulder.
inline Vec hybrid_hand_point(Vec hmd_rel, Vec eye, const float* basis, unsigned hand, const ArmScale& a, const HybridReach& h,
                             Vec navi_shoulder, float navi_length, float* used, float* weight) {
  const Vec scaled = model_hand_point(hmd_rel, basis, hand, a, navi_shoulder, navi_length, used);
  if (weight) *weight = 1.f;
  if (!h.enabled || !finite(eye) || !finite(scaled)) return scaled;
  float full = used ? *used : 1.f;
  if (!a.enabled) full = 1.f;
  if (!std::isfinite(full) || full <= 0.f) full = 1.f;
  const Vec user_shoulder = real_shoulder(Vec{0.f, 0.f, 0.f}, basis, hand, a);
  const Vec v = sub(hmd_rel, user_shoulder);
  const float user_length = std::max(0.3f, a.user_length);
  const float r = length(v) / user_length;
  const float near_share = std::clamp(h.near_share, 0.f, 0.95f);
  const float span = std::min(std::max(h.far_share - near_share, 0.1f), 1.f - near_share);
  const float g = hybrid_outer_gain(full, near_share, span);
  const float k = r > 1e-6f ? hybrid_radial(r, near_share, span, g) / r : 1.f;  // this point's factor
  const float w = smooth01(near_share, 1.f, r);
  if (weight) *weight = w;
  if (used) *used = k;
  const Vec seen_shoulder = add(eye, user_shoulder);  // the real shoulder where the eyes would see it
  const Vec anchor = add(seen_shoulder, scale(sub(navi_shoulder, seen_shoulder), w));
  return add(anchor, scale(v, k));
}
// Soft reach limit (2026-11-13): a target past ~97% of the arm is eased towards full length instead of clamped, so the
// elbow does not snap straight. Applied to the wrist target itself, so the gun (placed from the same target) stays on the hand.
inline float soft_reach(float distance, float limit, float knee_share = 0.03f) {
  if (!(limit > 0.f) || !std::isfinite(distance)) return distance;
  const float knee = knee_share * limit, start = limit - knee;
  if (distance <= start || knee <= 0.f) return distance;
  return start + knee * (1.f - std::exp(-(distance - start) / knee));
}
inline Vec soft_reach_point(Vec target, Vec shoulder, float limit, float knee_share = 0.03f) {
  const Vec d = sub(target, shoulder);
  const float l = length(d);
  if (!(l > 1e-6f)) return target;
  const float r = soft_reach(l, limit, knee_share);
  return add(shoulder, scale(d, r / l));
}
// Elbow-only swivel smoothing (2026-11-13). The pole direction is
// filtered with time constant tau; the wrist is not touched, so the hand gets no extra latency.
struct PoleFilter {
  Vec v{0.f, 0.f, 0.f};
  bool primed = false;
  Vec step(Vec pole, float dt, float tau) {
    const float lp = length(pole);
    if (!(lp > 1e-6f) || !finite(pole)) return pole;
    const Vec n = scale(pole, 1.f / lp);
    if (!primed || !(dt > 0.f) || dt > 0.2f || !(tau > 0.f)) { v = n; primed = true; return n; }
    const Vec m = add(v, scale(sub(n, v), 1.f - std::exp(-dt / tau)));
    const float lm = length(m);
    v = lm > 1e-3f ? scale(m, 1.f / lm) : n;
    return v;
  }
};
// Row-vector rotation part of a 4x4 applied to a direction, and to the axis rows of another matrix (translation kept).
inline Vec rotate_dir(Vec v, const float* m) {
  return {v.x * m[0] + v.y * m[4] + v.z * m[8], v.x * m[1] + v.y * m[5] + v.z * m[9], v.x * m[2] + v.y * m[6] + v.z * m[10]};
}
inline void rotate_rows(float* target, const float* m) {
  for (int r = 0; r < 3; ++r) {const Vec v = rotate_dir(row(target, r), m); target[r * 4] = v.x; target[r * 4 + 1] = v.y; target[r * 4 + 2] = v.z;}
}
// T-pose calibration (2026-10-15, [AUTHOR] "extend the arms to the sides and press a button"): with both arms straight
// out to the sides the grips are at shoulder height on the shoulder line. span = grip-to-grip; the shoulder joints sit
// 20% of the span apart (anthropometric shoulder-joint breadth); arm length = (span - width) / 2 (shoulder to grip);
// drop / back = headset centre above / in front of the grips' midpoint along the body axes.
struct TPose {bool ok = false; float span = 0.f, width = 0.f, drop = 0.f, back = 0.f, length = 0.f, lateral = 0.f;};
inline TPose tpose_calibration(Vec left, Vec right, Vec head, const float* basis) {
  TPose t;
  Vec r = row(basis, 0), u = row(basis, 1), f = row(basis, 2);
  const float lr = length(r), lu = length(u), lf = length(f);
  if (lr < 1e-4f || lu < 1e-4f || lf < 1e-4f || !finite(left) || !finite(right) || !finite(head)) return t;
  r = scale(r, 1.f / lr); u = scale(u, 1.f / lu); f = scale(f, 1.f / lf);
  const Vec across = sub(right, left);
  t.span = length(across);
  if (t.span < 1e-3f) return t;
  t.lateral = dot(across, r) / t.span;  // 1 = hands exactly left/right of each other
  const Vec mid = scale(add(left, right), 0.5f), d = sub(head, mid);
  t.drop = dot(d, u);
  t.back = dot(d, f);
  t.width = 0.20f * t.span;
  t.length = 0.5f * (t.span - t.width);
  t.ok = t.span > 1.0f && t.span < 2.4f && t.lateral > 0.85f && t.drop > 0.05f && t.drop < 0.45f && std::fabs(t.back) < 0.3f &&
         t.length > 0.35f && t.length < 1.0f;
  return t;
}
// Rotate a row-vector matrix (axes rows 0-2, position row 3) about the vertical axis through (cx, *, cz) by yaw delta d,
// using the game's yaw convention (forward = (sin yaw, 0, cos yaw)).
inline void rotate_about_up(float* m, float cx, float cz, float d) {
  const float c = std::cos(d), s = std::sin(d);
  for (int r = 0; r < 3; ++r) {
    const float x = m[r * 4], z = m[r * 4 + 2];
    m[r * 4] = x * c + z * s;
    m[r * 4 + 2] = -x * s + z * c;
  }
  const float x = m[12] - cx, z = m[14] - cz;
  m[12] = cx + x * c + z * s;
  m[14] = cz - x * s + z * c;
}
// One-Euro filter (Casiez et al. 2012), the usual VR hand-jitter filter: strong smoothing at low speed, little lag at
// high speed. Applied per component to the grip in the body frame (position and the three axis rows).
struct OneEuro {
  // [LIVE 2026-10-30] 1.5/3/1 delayed the hands 30-50 ms, then caught up as the slow speed estimate rose ("laggy and
  // snappy"). Replay of the recorded raw grips: this setting cuts the lag to about a third with the same still-hand step.
  float min_cutoff = 2.0f, beta = 60.0f, d_cutoff = 5.0f;  // Hz, (1/s per unit speed), Hz
  bool primed = false;
  float x = 0.f, dx = 0.f;
  static float alpha(float cutoff, float dt) {
    const float tau = 1.f / (6.2831853f * cutoff);
    return 1.f / (1.f + tau / dt);
  }
  float filter(float value, float dt) {
    if (!primed || !(dt > 0.f) || dt > 0.2f) { primed = true; x = value; dx = 0.f; return value; }
    const float raw_dx = (value - x) / dt;
    dx += alpha(d_cutoff, dt) * (raw_dx - dx);
    const float cutoff = min_cutoff + beta * std::fabs(dx);
    x += alpha(cutoff, dt) * (value - x);
    return x;
  }
};
// [SOURCE 2026-10-31] the 2/60/5 retune was replayed on positions only (metres/s). The same beta on axis components keeps a
// shaking wrist near 8-20 Hz, and a held gun is a lever: 0.14 deg/frame at the wrist is 1.2 mm/frame at a 50 cm muzzle
// [LIVE WEAPON_IDLE_PARTS 2026-10-31]. Rotation gets its own setting, in the shared stream, so hands and gun both take it.
inline void set_rotation_tuning(OneEuro& f) { f.min_cutoff = 1.6f; f.beta = 14.0f; f.d_cutoff = 2.5f; }
// Filters a row-vector grip matrix (axes rows 0-2 and position) in place; axes are re-orthonormalised (forward kept).
inline void smooth_grip(float* m, OneEuro (&f)[12], float dt, bool rotation_tuned = false) {
  const int idx[12] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
  for (int i = 0; i < 12; ++i) {
    if (i < 9) { if (rotation_tuned) set_rotation_tuning(f[i]); else { f[i].min_cutoff = 2.0f; f[i].beta = 60.0f; f[i].d_cutoff = 5.0f; } }
    m[idx[i]] = f[i].filter(m[idx[i]], dt);
  }
  Vec fw = row(m, 2), up = row(m, 1);
  const float lf = length(fw);
  if (lf < 1e-6f) return;
  fw = scale(fw, 1.f / lf);
  up = sub(up, scale(fw, dot(up, fw)));
  const float lu = length(up);
  if (lu < 1e-6f) return;
  up = scale(up, 1.f / lu);
  const Vec rt = cross(up, fw);
  m[0] = rt.x; m[1] = rt.y; m[2] = rt.z; m[4] = up.x; m[5] = up.y; m[6] = up.z; m[8] = fw.x; m[9] = fw.y; m[10] = fw.z;
}
// Tuning is authored for the right hand; the left hand mirrors the left-right components (yaw, roll, side).
inline HandTune for_hand(HandTune t, unsigned hand) {
  if (hand == 0) { t.yaw = -t.yaw; t.roll = -t.roll; t.side = -t.side; }
  return t;
}
inline Vec rotate_axis(Vec v, Vec axis, float angle) {
  const float c = std::cos(angle), s = std::sin(angle);
  return add(add(scale(v, c), scale(cross(axis, v), s)), scale(axis, dot(axis, v) * (1.f - c)));
}
// Secondary motion for gear-only bones (charms, cords): the game's dangle physics simulates the flat arm, so the mod runs
// its own damped spring on each anchored bone's tip. target = the rigidly anchored position, pivot = its anchor bone.
// The tip keeps its distance from the pivot (applied as a rotation about the pivot, nothing stretches) and stays within
// `clamp` of the target. A jump of the target (> 0.5 m: teleport, map change, IK toggle) resets the state.
struct Dangle {
  bool primed = false;
  Vec x{}, v{}, last{};
  Vec step(Vec pivot, Vec target, float dt, float hz, float zeta, float clamp) {
    if (!primed || !(dt > 0.f) || length(sub(target, last)) > 0.5f || !finite(x) || !finite(v)) {
      primed = true; x = target; v = {0.f, 0.f, 0.f}; last = target; return target;
    }
    last = target;
    dt = std::min(dt, 1.f / 30.f);
    const float w = 6.2831853f * hz;
    const Vec a = sub(scale(sub(target, x), w * w), scale(v, 2.f * zeta * w));
    v = add(v, scale(a, dt));
    x = add(x, scale(v, dt));
    const Vec off = sub(x, target);
    const float lo = length(off);
    if (lo > clamp) {
      x = add(target, scale(off, clamp / lo));
      const Vec n = scale(off, 1.f / lo); const float out = dot(v, n);
      if (out > 0.f) v = sub(v, scale(n, out));
    }
    const float r = length(sub(target, pivot)); const Vec d = sub(x, pivot); const float ld = length(d);
    if (r > 1e-5f && ld > 1e-5f) x = add(pivot, scale(d, r / ld));
    return x;
  }
};
// Rotates a row-vector affine bone matrix about `pivot` by the rotation taking direction `from` to `to`.
inline bool swing_about(float* m, Vec pivot, Vec from, Vec to) {
  const float lf = length(from), lt = length(to);
  if (lf < 1e-5f || lt < 1e-5f) return false;
  const Vec a = scale(from, 1.f / lf), b = scale(to, 1.f / lt);
  const Vec c = cross(a, b); const float s = length(c);
  if (s < 1e-6f) return false;
  const Vec axis = scale(c, 1.f / s); const float angle = std::atan2(s, dot(a, b));
  for (unsigned i = 0; i < 3; ++i) { const Vec r = rotate_axis(row(m, i), axis, angle); m[i * 4] = r.x; m[i * 4 + 1] = r.y; m[i * 4 + 2] = r.z; }
  const Vec p = add(pivot, rotate_axis(sub(position(m), pivot), axis, angle));
  m[12] = p.x; m[13] = p.y; m[14] = p.z;
  return true;
}
// Twist of `aligned` relative to `rigid` around unit `axis` (radians, right-handed about axis).
inline float twist_angle(const float* rigid, const float* aligned, Vec axis) {
  unsigned pick = 0; float least = 2.f;
  for (unsigned i = 0; i < 3; ++i) { const Vec r = row(rigid, i); const float l = length(r);
    if (l < 1e-6f) continue; const float d = std::fabs(dot(scale(r, 1.f / l), axis)); if (d < least) { least = d; pick = i; } }
  Vec a = row(rigid, pick), b = row(aligned, pick);
  a = sub(a, scale(axis, dot(a, axis))); b = sub(b, scale(axis, dot(b, axis)));
  if (length(a) < 1e-6f || length(b) < 1e-6f) return 0.f;
  return std::atan2(dot(axis, cross(a, b)), dot(a, b));
}
inline void twist_bone(float* m, Vec axis, float angle) {
  for (unsigned i = 0; i < 3; ++i) { const Vec r = rotate_axis(row(m, i), axis, angle); m[i * 4] = r.x; m[i * 4 + 1] = r.y; m[i * 4 + 2] = r.z; }
}
// hand: in = animated wrist matrix, out = axes re-oriented (translation untouched). target = where the wrist goes.
inline bool align_hand(float* hand, Vec& target, Vec middle, Vec index, Vec pinky, const float* grip, const HandTune& t) {
  if (!grip_basis(grip)) return false;
  const Vec w = position(hand);
  Vec f = sub(middle, w);
  const float reach = length(f);
  if (!finite(f) || reach < 1e-3f || reach > 1.f) return false;
  f = scale(f, 1.f / reach);
  Vec s = sub(index, pinky);
  s = sub(s, scale(f, dot(s, f)));
  const float sl = length(s);
  if (!finite(s) || sl < 1e-4f) return false;
  s = scale(s, 1.f / sl);
  const Vec n = cross(f, s);
  float g[16];
  tuned_grip(g, grip, t);
  const Vec ft = row(g, 2), st = row(g, 1), nt = cross(ft, st);
  float r[3][3];
  for (int i = 0; i < 3; ++i) {
    const float fi = i == 0 ? f.x : i == 1 ? f.y : f.z, si = i == 0 ? s.x : i == 1 ? s.y : s.z, ni = i == 0 ? n.x : i == 1 ? n.y : n.z;
    r[i][0] = fi * ft.x + si * st.x + ni * nt.x;
    r[i][1] = fi * ft.y + si * st.y + ni * nt.y;
    r[i][2] = fi * ft.z + si * st.z + ni * nt.z;
  }
  float out[12];
  for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j)
    out[k * 4 + j] = hand[k * 4] * r[0][j] + hand[k * 4 + 1] * r[1][j] + hand[k * 4 + 2] * r[2][j];
  for (float v : out) if (!std::isfinite(v)) return false;
  for (int k = 0; k < 3; ++k) for (int j = 0; j < 3; ++j) hand[k * 4 + j] = out[k * 4 + j];
  target = sub(position(g), scale(ft, t.palm * reach));
  return finite(target);
}
// World/cell-local positions are supplied by the caller. Root, other bones and source stay unchanged.
inline bool apply(float (&out)[211][16],const float (&source)[211][16],unsigned hand,Vec target,Vec pole){
  if(hand>1||&out==&source) return false;
  const unsigned upper=hand?113:14,lower=upper+1,wrist=upper+2;
  for(unsigned bone=upper;bone<=wrist;++bone) for(unsigned j=0;j<16;++j) if(!std::isfinite(source[bone][j])) return false;
  const Vec s=position(source[upper]),e=position(source[lower]),w=position(source[wrist]);
  const Vec a=sub(e,s),b=sub(w,e);Solution solved{};
  if(!solve(s,target,pole,length(a),length(b),solved)) return false;
  matrix(out[upper],source[upper],a,sub(solved.elbow,s),s);
  matrix(out[lower],source[lower],b,sub(solved.wrist,solved.elbow),solved.elbow);
  matrix(out[wrist],source[wrist],b,sub(solved.wrist,solved.elbow),solved.wrist);
  return true;
}
}
