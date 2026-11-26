#pragma once
#include "hand_gesture_policy.hpp"
#include "mounted_hand_pose.hpp"
#include "vr_arms_math.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace hand_gestures {
// A reference belongs to one actual mesh resource, rig, player and hand. Frozen
// bind axes/lengths are calibration only; open/curl directions are authored in
// anatomical space, never taken from a weapon/harness grip.
struct HandReference {
  int wrist=-1;
  std::array<int,mounted_hand_pose::count> core{};
  std::vector<int> family,anchor,helper;
  std::vector<std::array<float,16>> local;
  float palm_in_wrist[16]{};
  bool valid=false;
};
inline bool finite_matrix(const float* m) {
  for(int j=0;j<16;++j) if(!std::isfinite(m[j])) return false;
  return true;
}
inline bool rotation_scale(const float* m,float* rotation,float (&sizes)[3]) {
  using namespace skin_arm_ik;
  if(!finite_matrix(m)) return false;
  std::memcpy(rotation,m,64);
  for(int j=0;j<3;++j) {
    sizes[j]=length(row(m,j)); if(!(sizes[j]>.001f && sizes[j]<10.f)) return false;
    const auto v=scale(row(m,j),1.f/sizes[j]);
    rotation[4*j]=v.x;rotation[4*j+1]=v.y;rotation[4*j+2]=v.z;
  }
  return mounted_weapon_math::rigid(rotation);
}
inline bool blend_scaled(const float* a,const float* b,float weight,float* out,bool interpolate_scale=false) {
  float ra[16],rb[16],sa[3],sb[3];
  if(!rotation_scale(a,ra,sa) || !rotation_scale(b,rb,sb)) return false;
  vr_arms_math::blend_affine(ra,rb,weight,out);
  // Native bone scale belongs to the mesh, never the authored pose.
  for(int j=0;j<3;++j) for(int k=0;k<3;++k) out[4*j+k]*=interpolate_scale?sa[j]+weight*(sb[j]-sa[j]):sa[j];
  return finite_matrix(out);
}
inline bool capture_hand(HandReference& result,const float (*mats)[16],unsigned n,const char* const* names,int hand) {
  using namespace skin_arm_ik;
  if(hand<0 || hand>1 || !mats || !names || n>1024) return false;
  HandReference ref;ref.core.fill(-1);ref.family.assign(n,-1);ref.anchor.assign(n,-1);ref.helper.assign(n,-1);ref.local.resize(n);
  for(unsigned i=0;i<n;++i) {
    const char* name=names[i];
    if(!name || std::strlen(name)<3 || name[0]!=(hand?'R':'L') || name[1]!='_') continue;
    if(!std::strcmp(name+2,"hand")) ref.wrist=static_cast<int>(i);
    const int k=mounted_hand_pose::index(name);ref.family[i]=mounted_hand_pose::family(name);
    if(k>=0) {if(ref.core[k]>=0) return false;ref.core[k]=static_cast<int>(i);}
  }
  if(ref.wrist<0) return false;
  for(int k:ref.core) if(k<0) return false;
  float palm[16],wrist_inverse[16],inverse[16];
  if(!mounted_weapon_math::palm(mats[ref.wrist],position(mats[ref.core[10]]),position(mats[ref.core[5]]),
       position(mats[ref.core[15]]),palm) || !inverse_affine(wrist_inverse,mats[ref.wrist])) return false;
  multiply_affine(ref.palm_in_wrist,palm,wrist_inverse);
  for(unsigned i=0;i<n;++i) if(ref.family[i]>=0) {
    // Subtract the origin before rotation to avoid world/cell cancellation.
    if(!finite_matrix(mats[i])) return false;
    float local[16];std::memcpy(local,mats[i],64);
    for(int j=0;j<3;++j) local[12+j]-=palm[12+j];
    float rotation[16];std::memcpy(rotation,palm,64);rotation[12]=rotation[13]=rotation[14]=0;
    if(!inverse_affine(inverse,rotation)) return false;
    multiply_affine(ref.local[i].data(),local,inverse);
  }
  for(int k=0;k<mounted_hand_pose::count;++k) {
    float rotation[16],sizes[3];
    if(!rotation_scale(ref.local[ref.core[k]].data(),rotation,sizes)) return false;
    const int p=mounted_hand_pose::parent[k];
    if(p>=0 && length(sub(position(ref.local[ref.core[k]].data()),position(ref.local[ref.core[p]].data())))<=1e-6f &&
       mounted_hand_pose::child[k]>=0) return false;
  }
  for(unsigned i=0;i<n;++i) if(ref.family[i]>=0) {
    const int core=mounted_hand_pose::index(names[i]);
    if(core>=0) {ref.anchor[i]=core;continue;}
    ref.helper[i]=thumb_helper_index(names[i]);
    if(ref.helper[i]>=0) {
      // Recorded helper names explicitly identify 01/02/03. This is semantic
      // grouping, not a claim that unrecorded mesh parent tables were decoded.
      ref.anchor[i]=ref.helper[i]<4?0:(ref.helper[i]<7 || ref.helper[i]==10 || ref.helper[i]==11)?1:2;
      continue;
    }
    float best=1e9f;
    for(int k=0;k<mounted_hand_pose::count;++k) if(mounted_hand_pose::family(k)==ref.family[i]) {
      const float d=length(sub(position(ref.local[i].data()),position(ref.local[ref.core[k]].data())));
      if(d<best) {best=d;ref.anchor[i]=k;}
    }
    if(ref.anchor[i]<0) return false;
  }
  ref.valid=true;result=std::move(ref);return true;
}
// A reference frozen from an animation frame carries that frame's curl and twist,
// and pose_hand swings minimally away from it. It is saved per rig so slider values
// keep their meaning across game restarts; indices are re-derived from names.
inline std::uint64_t rig_hash(const char* const* names,unsigned n) {
  std::uint64_t h=14695981039346656037ull;
  for(unsigned i=0;i<n;++i) {
    for(const char* c=names[i]?names[i]:"";*c;++c) {h^=static_cast<unsigned char>(*c);h*=1099511628211ull;}
    h^=0xFF;h*=1099511628211ull;
  }
  return h;
}
inline bool hand_bones_present(const char* const* names,unsigned n,int hand) {
  bool wrist=false,core[mounted_hand_pose::count]{};
  for(unsigned i=0;i<n;++i) {
    const char* name=names[i];
    if(!name || std::strlen(name)<3 || name[0]!=(hand?'R':'L') || name[1]!='_') continue;
    wrist=wrist || !std::strcmp(name+2,"hand");
    const int k=mounted_hand_pose::index(name);if(k>=0) core[k]=true;
  }
  if(!wrist) return false;
  for(bool b:core) if(!b) return false;
  return true;
}
inline std::string reference_text(const HandReference& ref) {
  std::string out;char number[32];
  auto put=[&](float v) {std::snprintf(number,sizeof(number),"%.9g ",v);out+=number;};
  for(float v:ref.palm_in_wrist) put(v);
  for(std::size_t i=0;i<ref.family.size();++i) if(ref.family[i]>=0) {
    std::snprintf(number,sizeof(number),"%zu %d ",i,ref.anchor[i]);out+=number;
    for(float v:ref.local[i]) put(v);
  }
  return out;
}
// ref must already hold this rig's name-derived structure (capture_hand). All or nothing.
inline bool reference_from_text(const std::string& text,HandReference& ref) {
  if(!ref.valid) return false;
  HandReference next=ref;std::istringstream in(text);
  for(float& v:next.palm_in_wrist) if(!(in>>v) || !std::isfinite(v)) return false;
  std::size_t seen=0,index=0,expected=0;int anchor=0;
  for(int family:ref.family) if(family>=0) ++expected;
  while(in>>index>>anchor) {
    if(index>=ref.family.size() || ref.family[index]<0 || anchor<0 || anchor>=mounted_hand_pose::count) return false;
    next.anchor[index]=anchor;
    for(float& v:next.local[index]) if(!(in>>v) || !std::isfinite(v)) return false;
    ++seen;
  }
  if(seen!=expected) return false;
  ref=std::move(next);return true;
}
// Same mesh proportions? Animation does not stretch finger segments, another mesh does.
// Largest relative finger-segment length difference between a saved reference and the live hand (1e9 if unusable).
inline float reference_fit_error(const HandReference& ref,const float (*mats)[16],unsigned n) {
  using namespace skin_arm_ik;
  float palm[16],inverse[16];
  if(!ref.valid || ref.local.size()!=n) return 1e9f;
  multiply_affine(palm,ref.palm_in_wrist,mats[ref.wrist]);
  if(!inverse_affine(inverse,palm)) return 1e9f;
  float worst=0.f;
  for(int k=0;k<mounted_hand_pose::count;++k) {
    const int p=mounted_hand_pose::parent[k];if(p<0) continue;
    const float saved=length(sub(position(ref.local[ref.core[k]].data()),position(ref.local[ref.core[p]].data())));
    if(saved<=1e-5f) continue;
    float a[16],b[16];
    multiply_affine(a,mats[ref.core[k]],inverse);multiply_affine(b,mats[ref.core[p]],inverse);
    const float live=length(sub(position(a),position(b)));
    const float e=std::fabs(live/saved-1.f);
    if(!std::isfinite(e)) return 1e9f;
    worst=std::max(worst,e);
  }
  return worst;
}
inline bool reference_fits(const HandReference& ref,const float (*mats)[16],unsigned n) {return reference_fit_error(ref,mats,n)<.15f;}
// [AUTHOR 2026-11-18] tuned fingers looked different after some restarts. [STATIC] AvatarVR_hand_references.txt held two entries
// per rig and hand whose finger frames differ by a median 13 deg (up to 55) and whose segment lengths differ by up to 11%, so the
// old "first entry within 15%" rule could pick either one on a given load. The FIRST saved entry is now kept whenever it is
// plausibly the same mesh (within 25%); a later entry is used only if the first is not. Tunes are authored against it.
constexpr float kSavedReferenceTolerance=.25f;
inline int choose_saved_reference(const float* errors,int count) {
  for(int i=0;i<count;++i) if(errors[i]<kSavedReferenceTolerance) return i;
  return -1;
}
inline void euler_adjust(float x,float y,float z,float* out) {
  float a[16],b[16],c[16],ab[16];
  vr_arms_math::axis_angle_rows(1,0,0,x,a);
  vr_arms_math::axis_angle_rows(0,1,0,y,b);
  vr_arms_math::axis_angle_rows(0,0,1,z,c);
  skin_arm_ik::multiply_affine(ab,a,b);skin_arm_ik::multiply_affine(out,ab,c);
}
// skin_arm_ik::matrix divides by |from x to|; directions equal to float rounding
// (an authored/captured pose matching the reference) would yield NaN. Keep rows.
inline void swing_matrix(float* out,const float* source,skin_arm_ik::Vec from,skin_arm_ik::Vec to,skin_arm_ik::Vec pos) {
  using namespace skin_arm_ik;
  const float lf=length(from),lt=length(to);
  if(lf>1e-12f && lt>1e-12f && length(cross(from,to))<=1e-6f*lf*lt && dot(from,to)>0.f) {
    std::memcpy(out,source,64);out[12]=pos.x;out[13]=pos.y;out[14]=pos.z;return;
  }
  matrix(out,source,from,to,pos);
}
inline bool pose_hand(float (*mats)[16],unsigned n,int hand,const Pose& pose,const HandReference& ref) {
  using namespace skin_arm_ik;
  constexpr int count=mounted_hand_pose::count;
  if(!ref.valid || ref.local.size()!=n || !(pose.weight>0.f) || !std::isfinite(pose.weight)) return false;
  if(hand<0 || hand>1 || ref.wrist<0 || static_cast<unsigned>(ref.wrist)>=n) return false;
  for(int k:ref.core) if(k<0 || static_cast<unsigned>(k)>=n) return false;
  const float weight=std::clamp(pose.weight,0.f,1.f),sign=hand?-1.f:1.f,rad=.01745329252f;
  float palm[16],inverse[16],rotation[16],sizes[3];
  multiply_affine(palm,ref.palm_in_wrist,mats[ref.wrist]);
  if(!rotation_scale(palm,rotation,sizes) || !inverse_affine(inverse,palm)) return false;
  // Finger geometry scales uniformly with the wrist; nonuniform/sheared wrists
  // fail closed instead of producing a rig-dependent twist.
  if(std::fabs(sizes[0]-sizes[1])>.02f || std::fabs(sizes[0]-sizes[2])>.02f) return false;
  float native[count][16],target[count][16],next[count][16],root_rotation[4][16];
  for(int f=0;f<4;++f) {
    for(float v:pose.shape.finger[f].extra) if(!std::isfinite(v)) return false;
    const auto& e=pose.shape.finger[f].extra;
    euler_adjust(e[1]*rad,sign*e[0]*rad,sign*e[2]*rad,root_rotation[f]);
  }
  for(int k=0;k<count;++k) {
    if(!finite_matrix(mats[ref.core[k]])) return false;
    multiply_affine(native[k],mats[ref.core[k]],inverse);
    std::memcpy(target[k],ref.local[ref.core[k]].data(),64);
  }
  for(int k=0;k<count;++k) {
    const int p=mounted_hand_pose::parent[k],family=mounted_hand_pose::family(k);
    const int first=family==0?0:family==1?4:family==2?9:14;
    const int segment=family==0?k:k-first-1;
    if(p<0) continue; // frozen palm roots; do not inherit animated metacarpal offsets
    const Vec old=sub(position(ref.local[ref.core[k]].data()),position(ref.local[ref.core[p]].data()));
    const float len=length(old); Vec direction=old;
    if(segment>=1) {
      const auto& shape=pose.shape.finger[family];float bend=0;
      for(int j=0;j<segment && j<3;++j) bend+=shape.bend[j];
      float spread=shape.splay;
      for(int j=0;j<segment && j<3;++j) spread+=shape.extra[6+j];
      const float a=bend*rad,s=spread*rad;
      direction={sign*std::sin(a),std::sin(s)*std::cos(a),std::cos(s)*std::cos(a)};
    }
    const float dl=length(direction);
    const Vec offset=len>1e-6f && dl>1e-6f?scale(direction,len/dl):Vec{};
    const Vec point=add(position(target[p]),offset);
    target[k][12]=point.x;target[k][13]=point.y;target[k][14]=point.z;
  }
  // Root placement is independently adjustable, including the proximal thumb
  // and its webbing. Corrections are in this rig's anatomical palm, never FP axes.
  for(int k=0;k<count;++k) {
    const int f=mounted_hand_pose::family(k);const auto& e=pose.shape.finger[f].extra;
    const Vec point=add(rotate_dir(position(target[k]),root_rotation[f]),{sign*e[3]*.001f,e[4]*.001f,e[5]*.001f});
    target[k][12]=point.x;target[k][13]=point.y;target[k][14]=point.z;
  }
  for(int k=0;k<count;++k) {
    int node=k,c=mounted_hand_pose::child[k],p=mounted_hand_pose::parent[k];
    // Some native target bones coincide with their terminal phalanx.
    if(c<0 || length(sub(position(ref.local[ref.core[c]].data()),position(ref.local[ref.core[k]].data())))<=1e-6f) {
      if(p>=0) {c=k;node=p;} else return false;
    }
    Vec from=sub(position(ref.local[ref.core[c]].data()),position(ref.local[ref.core[node]].data()));
    Vec to=sub(position(target[c]),position(target[node]));
    if(length(from)<=1e-6f || length(to)<=1e-6f) {
      const int pp=mounted_hand_pose::parent[node];if(pp<0) return false;
      from=sub(position(ref.local[ref.core[node]].data()),position(ref.local[ref.core[pp]].data()));
      to=sub(position(target[node]),position(target[pp]));
    }
    if(length(from)<=1e-6f || length(to)<=1e-6f) return false;
    const int family=mounted_hand_pose::family(k);
    float reference_frame[16],oriented[16];
    multiply_affine(reference_frame,ref.local[ref.core[k]].data(),root_rotation[family]);
    const Vec rest=from;
    from=rotate_dir(from,root_rotation[family]);
    // Swing along the curl, not straight across: a fist curls a segment ~225 deg
    // from its rest direction, where the shortest swing is near-antiparallel and
    // its axis (so the finger's facing) swings sideways mid-curl [HEADSET 2026-11-07].
    const int c_first=family==0?0:family==1?4:family==2?9:14;
    const int c_segment=family==0?c:c-c_first-1;
    if(node!=c && c_segment>=1 && mounted_hand_pose::family(c)==family) {
      const auto& shape=pose.shape.finger[family];
      float bend=0,spread=shape.splay;
      for(int j=0;j<c_segment && j<3;++j) {bend+=shape.bend[j];spread+=shape.extra[6+j];}
      const float rl=length(rest);
      const float a0=std::asin(std::clamp(sign*rest.x/rl,-1.f,1.f))/rad,s0=std::atan2(rest.y,rest.z)/rad;
      const int steps=std::clamp(static_cast<int>(std::ceil(std::max(std::fabs(bend-a0),std::fabs(spread-s0))/45.f)),1,12);
      float frame[16],swung[16];std::memcpy(frame,reference_frame,64);Vec previous=from;
      for(int i=1;i<=steps;++i) {
        const float t=static_cast<float>(i)/steps,a=(a0+t*(bend-a0))*rad,s=(s0+t*(spread-s0))*rad;
        const Vec step=rotate_dir({sign*std::sin(a),std::sin(s)*std::cos(a),std::cos(s)*std::cos(a)},root_rotation[family]);
        swing_matrix(swung,frame,previous,step,position(target[k]));std::memcpy(frame,swung,64);previous=step;
      }
      swing_matrix(oriented,frame,previous,to,position(target[k]));
    } else swing_matrix(oriented,reference_frame,from,to,position(target[k]));
    const auto& shape=pose.shape.finger[mounted_hand_pose::family(k)];
    if(!std::isfinite(shape.roll)) return false;
    const auto axis=scale(to,1.f/length(to));
    const int first=family==0?0:family==1?4:family==2?9:14;
    const int joint=family==0?std::min(k,2):k>first?std::min(k-first-1,2):-1;
    const float joint_roll=joint>=0?shape.extra[9+joint]:0.f;
    float roll[16];vr_arms_math::axis_angle_rows(axis.x,axis.y,axis.z,sign*(shape.roll+joint_roll)*rad,roll);
    for(int j=0;j<3;++j) {
      const auto v=rotate_dir(row(oriented,j),roll);
      oriented[4*j]=v.x;oriented[4*j+1]=v.y;oriented[4*j+2]=v.z;
    }
    std::memcpy(target[k],oriented,64);
  }
  // Blend orientations, and interpolate segment DIRECTIONS on a sphere. Never
  // linearly lerp child positions: that shortens fingers in an interrupted blend.
  for(int k=0;k<count;++k) {
    if(!blend_scaled(native[k],target[k],weight,next[k])) return false;
    const int p=mounted_hand_pose::parent[k];
    if(p>=0) {
      const Vec a=sub(position(native[k]),position(native[p]));
      const Vec b=sub(position(target[k]),position(target[p]));
      const float len=length(a),bl=length(b);
      Vec offset{};
      if(len>1e-6f && bl>1e-6f) {
        float identity[16]{},swing[16],blended[16];identity[0]=identity[5]=identity[10]=identity[15]=1;
        swing_matrix(swing,identity,a,b,{});vr_arms_math::blend_affine(identity,swing,weight,blended);
        offset=rotate_dir(a,blended);
      } else if(len>1e-6f || bl>1e-6f) return false;
      const Vec point=add(position(next[p]),offset);
      next[k][12]=point.x;next[k][13]=point.y;next[k][14]=point.z;
    }
  }
  struct Change {unsigned index;std::array<float,16> matrix;};
  std::vector<Change> changes;changes.reserve(n);
  for(unsigned i=0;i<n;++i) if(ref.family[i]>=0) {
    const int k=ref.anchor[i];float local[16],out[16];
    const int exact=ref.core[k];
    if(static_cast<int>(i)==exact) std::memcpy(local,next[k],64);
    else {
      // Helpers keep their cached relation to the owning joint at full weight.
      // Native helper offsets contribute only to a native/gesture transition.
      float native_helper[16],inverse_live[16],inverse_ref[16],a[16],b[16],relative[16];
      multiply_affine(native_helper,mats[i],inverse);
      if(!inverse_affine(inverse_live,native[k]) || !inverse_affine(inverse_ref,ref.local[exact].data())) return false;
      multiply_affine(a,native_helper,inverse_live);multiply_affine(b,ref.local[i].data(),inverse_ref);
      const int helper=ref.helper[i];
      if(helper>=0) {
        const auto& trim=pose.shape.helper[helper];
        for(float v:trim) if(!std::isfinite(v)) return false;
        float delta[16],adjusted[16];
        euler_adjust(trim[0]*rad,trim[1]*rad,trim[2]*rad,delta);
        multiply_affine(adjusted,delta,b);
        float basis[16],bone_scale[3];
        if(!rotation_scale(b,basis,bone_scale)) {
          std::memset(basis,0,64);basis[0]=basis[5]=basis[10]=basis[15]=1;
        }
        const Vec shift=add(add(scale(row(basis,0),trim[3]*.001f),scale(row(basis,1),trim[4]*.001f)),
                            scale(row(basis,2),trim[5]*.001f));
        adjusted[12]+=shift.x;adjusted[13]+=shift.y;adjusted[14]+=shift.z;
        for(int axis=0;axis<3;++axis) for(int col=0;col<3;++col)
          adjusted[4*axis+col]*=1.f+trim[6+axis]*.01f;
        std::memcpy(b,adjusted,64);
      }
      if(!blend_scaled(a,b,weight,relative,true)) {
        // Native deformation helpers may legitimately have zero scale.
        if(!finite_matrix(a) || !finite_matrix(b)) return false;
        for(int j=0;j<16;++j) relative[j]=a[j]+weight*(b[j]-a[j]);
      }
      multiply_affine(local,relative,next[k]);
    }
    multiply_affine(out,local,palm);if(!finite_matrix(out)) return false;
    Change change;change.index=i;std::memcpy(change.matrix.data(),out,64);changes.push_back(change);
  }
  for(const auto& change:changes) std::memcpy(mats[change.index],change.matrix.data(),64);
  return !changes.empty();
}
// Test/one-shot convenience. Runtime always supplies a shared immutable reference.
inline bool pose_hand(float (*mats)[16],unsigned n,const char* const* names,int hand,const Pose& pose) {
  HandReference ref;return capture_hand(ref,mats,n,names,hand) && pose_hand(mats,n,hand,pose,ref);
}
// Inverse of pose_hand for the tuner's "capture native pose": slider values that
// reproduce the current palette (native animation) against the same reference.
// Root rotations stay 0 and finger metacarpal (segment 0) directions stay frozen,
// so the residual is measured by re-posing and reported, never assumed zero.
struct CaptureReport {float position_mm=0,angle_deg=0;int clamped=0,helpers=0,helpers_skipped=0;};
inline float wrap_degrees(float a) {
  while(a>180.f) a-=360.f;
  while(a<-180.f) a+=360.f;
  return a;
}
inline float clamp_count(float v,float lo,float hi,int& clamped) {
  if(v<lo || v>hi) ++clamped;
  return std::clamp(v,lo,hi);
}
inline bool palm_frame(const float (*mats)[16],unsigned n,const HandReference& ref,float* palm,float* inverse,float& size) {
  float rotation[16],sizes[3];
  if(!ref.valid || ref.local.size()!=n || ref.wrist<0 || static_cast<unsigned>(ref.wrist)>=n) return false;
  for(int k:ref.core) if(k<0 || static_cast<unsigned>(k)>=n) return false;
  skin_arm_ik::multiply_affine(palm,ref.palm_in_wrist,mats[ref.wrist]);
  if(!rotation_scale(palm,rotation,sizes) || !skin_arm_ik::inverse_affine(inverse,palm)) return false;
  size=sizes[0];return true;
}
// Signed rotation (degrees) about axis taking u onto v, both projected off the axis.
inline float twist_degrees(skin_arm_ik::Vec axis,skin_arm_ik::Vec u,skin_arm_ik::Vec v) {
  using namespace skin_arm_ik;
  u=sub(u,scale(axis,dot(u,axis)));v=sub(v,scale(axis,dot(v,axis)));
  if(length(u)<1e-4f || length(v)<1e-4f) return 0.f;
  return std::atan2(dot(axis,cross(u,v)),dot(u,v))*57.2957795f;
}
inline bool solve_shape(const float (*mats)[16],unsigned n,int hand,const HandReference& ref,Shape& out,CaptureReport& report) {
  using namespace skin_arm_ik;
  constexpr int count=mounted_hand_pose::count;
  if(hand<0 || hand>1 || n>1024) return false;
  float palm[16],inverse[16],size=1;
  if(!palm_frame(mats,n,ref,palm,inverse,size)) return false;
  const float sign=hand?-1.f:1.f,deg=57.2957795f;
  report=CaptureReport{};
  float native[count][16];
  for(int k=0;k<count;++k) {
    if(!finite_matrix(mats[ref.core[k]])) return false;
    multiply_affine(native[k],mats[ref.core[k]],inverse);
  }
  Shape shape;
  for(int f=0;f<4;++f) {
    const int first=f==0?0:f==1?4:f==2?9:14;
    auto& s=shape.finger[f];
    const Vec shift=sub(position(native[first]),position(ref.local[ref.core[first]].data()));
    s.extra[3]=clamp_count(sign*shift.x*1000.f,-20.f,20.f,report.clamped);
    s.extra[4]=clamp_count(shift.y*1000.f,-20.f,20.f,report.clamped);
    s.extra[5]=clamp_count(shift.z*1000.f,-20.f,20.f,report.clamped);
    // Cumulative curl A / spread S per segment; (A,S) and (180-A,S+180) give the
    // same direction, so follow the branch continuous with the previous segment.
    float a_prev=0,s_prev=0,a_seg[3]{},s_seg[3]{};
    for(int seg=1;seg<=3;++seg) {
      const int k=f==0?seg:first+1+seg,p=mounted_hand_pose::parent[k];
      const Vec d=sub(position(native[k]),position(native[p]));const float len=length(d);
      float a=a_prev,s_out=s_prev;
      if(len>1e-6f) {
        const Vec u=scale(d,1.f/len);
        const float a1=std::asin(std::clamp(sign*u.x,-1.f,1.f))*deg;
        const float side=std::sqrt(u.y*u.y+u.z*u.z);
        const float s1=side>.05f?std::atan2(u.y,u.z)*deg:s_prev;
        const float a2=180.f-a1,s2=side>.05f?wrap_degrees(s1+180.f):s_prev;
        const float c1=std::fabs(a1-a_prev)+std::fabs(wrap_degrees(s1-s_prev));
        const float c2=std::fabs(a2-a_prev)+std::fabs(wrap_degrees(s2-s_prev));
        a=c1<=c2?a1:a2;s_out=c1<=c2?s1:s2;
      }
      a_seg[seg-1]=a;s_seg[seg-1]=s_out;a_prev=a;s_prev=s_out;
    }
    for(int j=0;j<3;++j) s.bend[j]=clamp_count(a_seg[j]-(j?a_seg[j-1]:0.f),-30.f,120.f,report.clamped);
    s.splay=clamp_count(s_seg[0],-85.f,85.f,report.clamped);
    s.extra[7]=clamp_count(wrap_degrees(s_seg[1]-s_seg[0]),-60.f,60.f,report.clamped);
    s.extra[8]=clamp_count(wrap_degrees(s_seg[2]-s_seg[1]),-60.f,60.f,report.clamped);
  }
  // Roll: pose with zero roll, then measure each joint's native twist about the
  // same segment axis pose_hand rolls around.
  std::vector<std::array<float,16>> posed(n);
  auto run=[&](const Shape& s) {
    for(unsigned i=0;i<n;++i) std::memcpy(posed[i].data(),mats[i],64);
    Pose p;p.weight=1;p.shape=s;
    return pose_hand(reinterpret_cast<float(*)[16]>(posed.data()),n,hand,p,ref);
  };
  if(!run(shape)) return false;
  bool assigned[4][4]{};
  for(int k=0;k<count;++k) {
    int node=k,c=mounted_hand_pose::child[k];const int p=mounted_hand_pose::parent[k];
    if(c<0 || length(sub(position(ref.local[ref.core[c]].data()),position(ref.local[ref.core[k]].data())))<=1e-6f) {
      if(p<0) return false; c=k;node=p;
    }
    float local[16],child_local[16],node_local[16];
    multiply_affine(local,posed[ref.core[k]].data(),inverse);
    multiply_affine(child_local,posed[ref.core[c]].data(),inverse);
    multiply_affine(node_local,posed[ref.core[node]].data(),inverse);
    const Vec to=sub(position(child_local),position(node_local));
    if(length(to)<=1e-6f) continue;
    const Vec axis=scale(to,1.f/length(to));
    int best=0;float least=2;
    for(int j=0;j<3;++j) {
      const Vec r=row(local,j);const float l=length(r);
      const float along=l>1e-6f?std::fabs(dot(r,axis))/l:2.f;
      if(along<least) {least=along;best=j;}
    }
    const float total=sign*twist_degrees(axis,row(local,best),row(native[k],best));
    const int family=mounted_hand_pose::family(k);
    const int first=family==0?0:family==1?4:family==2?9:14;
    const int joint=family==0?std::min(k,2):k>first?std::min(k-first-1,2):-1;
    auto& s=shape.finger[family];
    if(joint<0) {s.roll=wrap_degrees(total);continue;}
    if(assigned[family][joint]) continue;
    assigned[family][joint]=true;
    s.extra[9+joint]=wrap_degrees(total-s.roll);
  }
  // Thumb webbing helpers: native helper relation to its owning joint, expressed
  // as the tuner's local rotation / offset / scale trims.
  for(unsigned i=0;i<n;++i) if(ref.family[i]>=0 && ref.helper[i]>=0) {
    const int k=ref.anchor[i],exact=ref.core[k];
    float helper[16],inverse_live[16],inverse_ref[16],a[16],b[16],ra[16],rb[16],sa[3],sb[3];
    if(!finite_matrix(mats[i])) {++report.helpers_skipped;continue;}
    multiply_affine(helper,mats[i],inverse);
    if(!inverse_affine(inverse_live,native[k]) || !inverse_affine(inverse_ref,ref.local[exact].data())) return false;
    multiply_affine(a,helper,inverse_live);multiply_affine(b,ref.local[i].data(),inverse_ref);
    if(!rotation_scale(a,ra,sa) || !rotation_scale(b,rb,sb)) {++report.helpers_skipped;continue;}
    float rb_inverse[16],delta[16],turned[16];
    rb[12]=rb[13]=rb[14]=ra[12]=ra[13]=ra[14]=0;
    if(!inverse_affine(rb_inverse,rb)) {++report.helpers_skipped;continue;}
    multiply_affine(delta,ra,rb_inverse);
    // euler_adjust(x,y,z) = Rx*Ry*Rz on row vectors (column form Rz*Ry*Rx).
    auto& trim=shape.helper[ref.helper[i]];
    trim[0]=clamp_count(std::atan2(delta[6],delta[10])*deg,-90.f,90.f,report.clamped);
    trim[1]=clamp_count(std::asin(std::clamp(-delta[2],-1.f,1.f))*deg,-90.f,90.f,report.clamped);
    trim[2]=clamp_count(std::atan2(delta[1],delta[0])*deg,-90.f,90.f,report.clamped);
    float applied[16];euler_adjust(trim[0]/deg,trim[1]/deg,trim[2]/deg,applied);
    multiply_affine(turned,applied,b);
    const Vec shift=sub(position(a),position(b));
    for(int j=0;j<3;++j) {
      trim[3+j]=clamp_count(dot(shift,row(rb,j))*1000.f,-20.f,20.f,report.clamped);
      const float l=length(row(turned,j));
      trim[6+j]=l>1e-6f?clamp_count((sa[j]/l-1.f)*100.f,-75.f,75.f,report.clamped):0.f;
    }
    ++report.helpers;
  }
  if(!run(shape)) return false;
  for(unsigned i=0;i<n;++i) if(ref.family[i]>=0) {
    float got[16],want[16];
    multiply_affine(got,posed[i].data(),inverse);multiply_affine(want,mats[i],inverse);
    report.position_mm=std::max(report.position_mm,length(sub(position(got),position(want)))*size*1000.f);
  }
  for(int k=0;k<count;++k) {
    float got[16],rg[16],rw[16],sg[3],sw[3];
    multiply_affine(got,posed[ref.core[k]].data(),inverse);
    if(!rotation_scale(got,rg,sg) || !rotation_scale(native[k],rw,sw)) continue;
    float trace=0;for(int j=0;j<3;++j) trace+=dot(row(rg,j),row(rw,j));
    report.angle_deg=std::max(report.angle_deg,std::acos(std::clamp((trace-1.f)*.5f,-1.f,1.f))*deg);
  }
  out=shape;return true;
}
} // namespace hand_gestures
