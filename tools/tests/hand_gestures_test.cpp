#include "../../src/vr/hand_gesture_pose.hpp"
#include "../../src/vr/hand_gesture_bindings.hpp"
#include "../../src/vr/hand_gesture_context_audit.hpp"
#include <map>
#include <cstdio>
#include <limits>
#include <string>

using namespace hand_gestures;
using namespace skin_arm_ik;
namespace {
int failures=0;
void check(bool ok,const char* why) {if(!ok) {std::printf("FAIL: %s\n",why); ++failures;}}
bool close(float a,float b,float e=1e-5f) {return std::fabs(a-b)<e;}
Signal signal(float v) {return {v,true,true};}
Input input() {
  Input in; in.focused=true; in.at=1000; in.sequence=1;
  for(auto& h:in.hand) {h.tracked=true; h.grip=signal(0); h.trigger=signal(0); h.index_touch=signal(0); h.thumb_touch=signal(0); h.thumb_press=signal(0);}
  return in;
}
Context context() {
  Context c; c.verified=c.gameplay=true; c.cinematic=false; c.at=1000; c.identity=1;
  c.owner[0]=c.owner[1]=Owner::Empty; return c;
}
Frame advance(Policy& p,Input& in,Context& c,std::uint64_t now) {
  in.at=c.at=now; ++in.sequence; return p.step(in,c,now,true);
}
void policy_tests() {
  auto in=input(); auto c=context(); Policy p;
  auto f=p.step(in,c,1000,true); f=advance(p,in,c,1100);
  check(f.hand[0].weight==0 && f.hand[1].weight==0,"native unarmed when untouched");
  auto& h=in.hand[0]; h.grip=signal(1); h.trigger=signal(1); h.index_touch=signal(1); h.thumb_touch=signal(1);
  auto t=target(h,false);
  for(float v:t.curl) check(v==1,"fist");
  h.trigger=signal(0); h.index_touch=signal(0); t=target(h,false);
  check(t.curl[1]==0 && t.curl[0]==1 && t.curl[2]==1,"point");
  h.thumb_touch=signal(0); t=target(h,false);
  check(t.curl[0]==0 && t.curl[1]==0 && t.curl[2]==1,"finger gun");
  h.trigger=signal(1); t=target(h,false);
  check(t.curl[0]==0 && t.curl[1]==1 && t.curl[3]==1,"thumbs up");
  h.grip=signal(.4f); h.trigger=signal(.3f); t=target(h,false);
  check(close(t.curl[2],.4f) && close(t.curl[1],.3f),"analog partial curl");
  h.trigger=signal(0); h.index_touch=signal(1); t=target(h,false);
  check(close(t.curl[1],.22f),"touch alone activates light index curl");
  h.index_touch={}; h.thumb_touch={}; h.thumb_press=signal(1); t=target(h,false);
  check(close(t.curl[1],.15f) && t.curl[0]==1,"unsupported index is relaxed, thumb press fallback");
  h.thumb_press={}; h.grip={}; t=target(h,false); check(t.weight==0,"no available input preserves native");
  t=target(h,true); check(t.weight==1 && close(t.curl[2],.2f),"contextual relaxed idle");

  in=input(); c=context(); p=Policy{}; in.hand[0].grip=signal(1);
  p.step(in,c,1000,true); f=advance(p,in,c,1120);
  check(f.hand[0].weight>.8f && f.hand[1].weight==0,"independent hands");
  const auto repeat=p.step(in,c,1200,true);
  check(repeat.sequence==f.sequence && repeat.hand[0].weight==f.hand[0].weight,"draws cannot advance filter");
  in.hand[0].grip=signal(0); f=advance(p,in,c,1140);
  check(f.hand[0].weight>0 && f.hand[0].weight<repeat.hand[0].weight,"native return eases out");
  in.hand[0].grip=signal(1); const float before=f.hand[0].weight; f=advance(p,in,c,1160);
  check(f.hand[0].weight>before && f.hand[0].weight<1,"interrupted reversal starts at displayed weight");
  for(auto owner:{Owner::Weapon,Owner::Item,Owner::Support,Owner::Harness}) {
    c.owner[0]=owner; const auto prev=f.hand[0].weight; f=advance(p,in,c,c.at+20);
    check(f.hand[0].weight<prev,"native occupancy wins with fade out");
  }
  c.owner[0]=Owner::Swap; f=advance(p,in,c,c.at+20); check(f.hand[0].weight==0,"swap immediate exclusion");
  c.owner[0]=Owner::Empty; c.mounted=true; in=input(); in.at=c.at; f=advance(p,in,c,c.at+120);
  check(f.hand[0].weight>0 && f.hand[1].weight>0,"free mounted hands idle");
  c.cinematic=true; f=advance(p,in,c,c.at+20); check(!f.at && f.hand[0].weight==0,"cinematic clears state immediately");
  c.cinematic=false; c.verified=false; f=advance(p,in,c,c.at+20); check(!f.at,"unknown context stays native");
  c.verified=true; c.weapon=true; f=advance(p,in,c,c.at+20); f=advance(p,in,c,c.at+120);
  check(f.hand[0].weight>0,"free weapon offhand idle");
  in.focused=false; f=advance(p,in,c,c.at+20); check(!f.at,"focus loss clears state");
  in.focused=true; p.step(in,c,c.at,true); f=p.step(in,c,c.at+251,true); check(!f.at,"stale sensor/context fails closed");
  f=p.step(in,c,c.at,false); check(!f.at,"toggle rollback");
  advance(p,in,c,c.at+10); f=advance(p,in,c,c.at+100); ++c.identity; f=advance(p,in,c,c.at+10);
  check(f.hand[0].weight==0,"new player clears old hand state");

  Policy p10,p20; auto a=input(),b=input(); auto ca=context(),cb=context(); a.hand[0].grip=b.hand[0].grip=signal(1);
  p10.step(a,ca,1000,true); p20.step(b,cb,1000,true);
  for(unsigned now=1010;now<=1180;now+=10) advance(p10,a,ca,now);
  for(unsigned now=1020;now<=1180;now+=20) advance(p20,b,cb,now);
  check(close(p10.frame().hand[0].weight,p20.frame().hand[0].weight),"frame-rate independent 60 ms filter");
  a.hand[0].grip.value=std::numeric_limits<float>::quiet_NaN();
  check(std::isfinite(target(a.hand[0],false).curl[2]),"non-finite sensor rejected");
}
struct Rig {
  static constexpr unsigned n=43;
  float m[n][16]{}; std::string text[n]; const char* names[n]{};
  Rig(float size=1.f) {
    for(unsigned i=0;i<n;++i) m[i][0]=m[i][5]=m[i][10]=m[i][15]=1;
    for(int h=0;h<2;++h) {
      const int base=h*20; const std::string prefix=h?"R_":"L_";
      text[base]=prefix+"hand"; m[base][12]=h?.3f:-.3f;
      for(int k=0;k<19;++k) {
        const int i=base+1+k,f=mounted_hand_pose::family(k);
        text[i]=prefix+mounted_hand_pose::names[k];
        const int first=f==0?0:f==1?4:f==2?9:14;
        m[i][12]=m[base][12]; m[i][13]=(f==0?.075f:f==1?.035f:f==3?-.035f:0.f)*size;
        m[i][14]=(.04f+.025f*(k-first))*size;
      }
    }
    text[40]="L_fingerIndex_expand"; m[40][12]=m[6][12]+.003f; m[40][13]=m[6][13]; m[40][14]=m[6][14];
    text[41]="L_weaponAttachmentHand"; text[42]="L_bracelet";
    for(unsigned i=0;i<n;++i) names[i]=text[i].c_str();
  }
};
void pose_tests() {
  for(float weight:{.01f,.5f,1.f}) for(float curl:{0.f,.35f,1.f}) {
    Rig rig; float before[Rig::n][16]; std::memcpy(before,rig.m,sizeof(before));
    Pose p; p.weight=weight; for(float& v:p.curl) v=curl;
    check(pose_hand(rig.m,Rig::n,rig.names,0,p),"complete left hand retargets");
    check(!std::memcmp(before[20],rig.m[20],20*64),"left-hand pose leaves the right hand unchanged");
    check(pose_hand(rig.m,Rig::n,rig.names,1,p),"complete right hand retargets");
    check(!std::memcmp(before[0],rig.m[0],64) && !std::memcmp(before[20],rig.m[20],64),"wrists unchanged");
    check(!std::memcmp(before[41],rig.m[41],128),"weapon attachment and bracelet unchanged");
    for(int h=0;h<2;++h) for(int k=0;k<19;++k) {
      const int i=h*20+1+k,parent=mounted_hand_pose::parent[k];
      check(mounted_weapon_math::rigid(rig.m[i]),"finite rigid finger matrix");
      if(parent>=0) {
        const int j=h*20+1+parent;
        check(close(length(sub(position(before[i]),position(before[j]))),length(sub(position(rig.m[i]),position(rig.m[j])))),"segment length preserved during blend");
      }
      check(close(rig.m[i][12]+.3f,-(rig.m[i+ (h? -20:20)][12]-.3f)) || h==1,"mirrored anatomical curl");
    }
    float native_inv[16],delta[16],helper[16];
    inverse_affine(native_inv,before[6]); multiply_affine(delta,native_inv,rig.m[6]); multiply_affine(helper,before[40],delta);
    check(length(sub(position(helper),position(rig.m[40])))<1e-5f,"finger helper follows owning joint delta");
  }
  Rig small(.8f),large(1.4f); Pose p; p.weight=1; for(float& v:p.curl) v=1;
  check(pose_hand(small.m,Rig::n,small.names,0,p) && pose_hand(large.m,Rig::n,large.names,0,p),"different FP/TP proportions");
  check(close(length(sub(position(small.m[8]),position(small.m[7])))*1.75f,
              length(sub(position(large.m[8]),position(large.m[7])))),"each resource keeps own geometry");
  Rig first,second;
  check(pose_hand(first.m,Rig::n,first.names,0,p) && pose_hand(second.m,Rig::n,second.names,0,p) &&
    !std::memcmp(first.m,second.m,sizeof(first.m)),"repeated draw copies produce deterministic poses");
  Rig shuffled;
  for(unsigned i=0;i<Rig::n/2;++i) {
    std::swap(shuffled.names[i],shuffled.names[Rig::n-1-i]);
    for(int j=0;j<16;++j) std::swap(shuffled.m[i][j],shuffled.m[Rig::n-1-i][j]);
  }
  check(pose_hand(shuffled.m,Rig::n,shuffled.names,0,p),"bone mapping independent of palette index order");
  Rig rig; float before[Rig::n][16]; std::memcpy(before,rig.m,sizeof(before)); rig.names[8]="missing";
  check(!pose_hand(rig.m,Rig::n,rig.names,0,p) && !std::memcmp(before,rig.m,sizeof(before)),"incomplete chain atomic fallback");
  rig.names[8]=rig.text[8].c_str(); rig.m[8][0]=std::numeric_limits<float>::quiet_NaN(); std::memcpy(before,rig.m,sizeof(before));
  check(!pose_hand(rig.m,Rig::n,rig.names,0,p) && !std::memcmp(before,rig.m,sizeof(before)),"invalid chain atomic fallback");
}
void stable_gesture_tests() {
  Rig baseline;HandReference reference;
  check(capture_hand(reference,baseline.m,Rig::n,baseline.names,0),"shared reference captures complete hand");
  Pose p;p.weight=1;p.shape=default_shape(11);
  Rig expected;
  check(pose_hand(expected.m,Rig::n,0,p,reference),"stable full fist");
  for(int frame=0;frame<15;++frame) {
    Rig animated;
    float spin[16];vr_arms_math::axis_angle_rows(0,1,0,frame*.13f,spin);
    for(unsigned i=1;i<20;++i) {
      float moved[16];const float pivot[3]={-.3f,0,0};
      vr_arms_math::rotate_about(animated.m[i],pivot,spin,moved);
      std::memcpy(animated.m[i],moved,64);
    }
    check(pose_hand(animated.m,Rig::n,0,p,reference),"animated native hand retargets against stable reference");
    for(unsigned i=1;i<20;++i) for(int j=0;j<16;++j)
      check(close(animated.m[i][j],expected.m[i][j],3e-5f),"full gesture excludes changing native roots and axial rotations");
  }
  Rig scaled;
  for(auto& bone:scaled.m) for(int r=0;r<3;++r) for(int c=0;c<3;++c) bone[4*r+c]*=1.3f;
  HandReference tp;
  check(capture_hand(tp,scaled.m,Rig::n,scaled.names,0) && pose_hand(scaled.m,Rig::n,0,p,tp),
    "scaled mounted skeleton is supported without copying FP axes");
  for(int i=1;i<20;++i) for(int axis=0;axis<3;++axis)
    check(close(length(row(scaled.m[i],axis)),1.3f),"native bone scale preserved");
  Rig rotated;float rig_rotation[16];vr_arms_math::axis_angle_rows(0,0,1,1.2f,rig_rotation);
  for(auto& bone:rotated.m) {float world[16];multiply_affine(world,bone,rig_rotation);std::memcpy(bone,world,64);}
  HandReference different_axes;
  check(capture_hand(different_axes,rotated.m,Rig::n,rotated.names,0) &&
    pose_hand(rotated.m,Rig::n,0,p,different_axes),"independent TP axes and palm orientation supported");
  for(int i=1;i<20;++i) {
    float world[16];multiply_affine(world,expected.m[i],rig_rotation);
    for(int j=0;j<16;++j) check(close(world[j],rotated.m[i][j],2e-5f),"anatomical gesture invariant under rig-space rotation");
  }
  Rig coincident;
  for(int k:{3,8,13,18}) {
    const int parent=mounted_hand_pose::parent[k];
    for(int j=12;j<15;++j) coincident.m[k+1][j]=coincident.m[parent+1][j];
  }
  HandReference terminal;
  check(capture_hand(terminal,coincident.m,Rig::n,coincident.names,0) &&
    pose_hand(coincident.m,Rig::n,0,p,terminal),"coincident terminal targets do not alternate to native");
  for(int combo=0;combo<combination_count;++combo) {
    Rig r;p.shape=default_shape(combo);
    check(pose_hand(r.m,Rig::n,0,p,reference),"all authored combination targets finite");
  }

  const auto settings=parse_gesture_tune(
    "tp.hand_pitch=90\n"
    "gesture.tp.left.fist.index.bend1=25\n"
    "gesture.fp.right.point.thumb.splay=-15\n"
    "gesture.tp.left.fist.index.bend2=999\n"
    "gesture.tp.left.fist.index.roll=nan\n"
    "gesture.tp.left.fist.index.bend3=5oops\n");
  check(settings.poses[1][0][11].finger[1].bend[0]==25 &&
    settings.poses[1][0][11].finger[1].bend[1]==120,"typed per-rig/hand/combination tune with clamped ranges");
  check(settings.poses[0][1][7].finger[0].splay==-15 &&
    settings.poses[0][0][11].finger[1].bend[0]==65,"tuning context and hand isolation");
  check(settings.poses[1][0][11].finger[1].roll==0 &&
    settings.poses[1][0][11].finger[1].bend[2]==60,"malformed and NaN tune values rejected");

  Tuning preview;preview.preview_context=0;preview.preview_hand=0;preview.preview_combo=11;preview.preview_until=102;
  auto in=input();auto c=context();Policy policy;
  policy.step(in,c,1000,true,&preview,100);
  in.at=c.at=1120;++in.sequence;
  auto out=policy.step(in,c,1120,true,&preview,100);
  check(out.hand[0].weight>.8f && out.hand[0].combination==11 && out.hand[1].weight==0,"preview targets only selected empty hand");
  in.at=c.at=1240;++in.sequence;const float previous=out.hand[0].weight;
  out=policy.step(in,c,1240,true,&preview,104);
  check(out.hand[0].weight<previous,"expired preview returns toward real input");
  c.owner[0]=Owner::Harness;in.at=c.at=1360;++in.sequence;
  out=policy.step(in,c,1360,true,&preview,100);
  check(out.hand[0].owner==Owner::Harness && out.hand[0].weight<previous,"preview cannot override harness ownership");
  c.cinematic=true;in.at=c.at=1380;++in.sequence;
  check(!policy.step(in,c,1380,true,&preview,100).at,"preview cannot bypass cinematic veto");
}

void thumb_control_tests() {
  Rig base;
  base.text[40]="L_thumb_01_expand01";base.names[40]=base.text[40].c_str();
  std::memcpy(base.m[40],base.m[2],64);base.m[40][12]+=.001f;
  HandReference ref;
  check(capture_hand(ref,base.m,Rig::n,base.names,0) && ref.anchor[40]==0,
    "named 01 webbing helper follows thumb01 even when another joint is closer");
  Pose p;p.weight=1;p.shape=default_shape(6);
  Rig posed=base;check(pose_hand(posed.m,Rig::n,0,p,ref),"baseline named thumb helper pose");
  Rig animated=base;
  for(int j=0;j<3;++j) animated.m[40][j]*=1.8f;
  check(pose_hand(animated.m,Rig::n,0,p,ref),"animated native helper scale is valid");
  for(int j=0;j<16;++j) check(close(animated.m[40][j],posed.m[40][j]),
    "full-weight helper scale excludes live native deformation");

  Rig root=base;p.shape.finger[0].extra[0]=25;p.shape.finger[0].extra[3]=3;
  check(pose_hand(root.m,Rig::n,0,p,ref),"thumb root pitch and normal offset applied");
  check(length(sub(position(root.m[1]),position(posed.m[1])))>.001f,"new base control changes previously frozen thumb01 placement");
  check(!std::memcmp(root.m[0],base.m[0],64),"thumb root control cannot move wrist");
  check(!std::memcmp(root.m[5],posed.m[5],15*64),"thumb root control leaves other fingers untouched");
  for(int k=1;k<4;++k) check(close(length(sub(position(root.m[k+1]),position(root.m[k]))),
    length(sub(position(base.m[k+1]),position(base.m[k])))),"root trim preserves thumb segment lengths");
  float inverse[16],delta[16],expected[16];
  inverse_affine(inverse,base.m[1]);multiply_affine(delta,inverse,root.m[1]);multiply_affine(expected,base.m[40],delta);
  for(int j=0;j<16;++j) check(close(expected[j],root.m[40][j]),"thumb base webbing follows corrected root transform");

  Rig adjusted=base;
  p.shape.helper[0][0]=15;p.shape.helper[0][3]=2;p.shape.helper[0][6]=20;
  check(pose_hand(adjusted.m,Rig::n,0,p,ref),"named helper rotation offset and scale are adjustable");
  for(int i=0;i<40;++i) check(!std::memcmp(adjusted.m[i],root.m[i],64),"helper trim never changes core joints");
  check(close(length(row(adjusted.m[40],0)),1.2f),"helper-only scale trim is applied");
  check(length(sub(position(adjusted.m[40]),position(root.m[40])))>.001f,"helper-only position trim is applied");
  Rig joint=base;Pose q;q.weight=1;q.shape=default_shape(6);
  q.shape.finger[0].extra[6]=15;q.shape.finger[0].extra[9]=20;
  check(pose_hand(joint.m,Rig::n,0,q,ref),"independent joint sideways and roll controls applied");
  check(length(sub(position(joint.m[2]),position(posed.m[2])))>.001f,"joint sideways changes thumb first phalanx direction");

  auto settings=parse_gesture_tune(
    "gesture.fp.right.finger_gun.thumb.root_pitch=18\n"
    "gesture.fp.right.finger_gun.thumb.root_normal=3\n"
    "gesture.fp.right.finger_gun.thumb.roll1=12\n"
    "gesture.fp.right.finger_gun.thumb_01_expand01.scale_x=22\n");
  check(settings.poses[0][1][6].finger[0].extra[0]==18 && settings.poses[0][1][6].finger[0].extra[3]==3 &&
    settings.poses[0][1][6].finger[0].extra[9]==12 && settings.poses[0][1][6].helper[0][6]==22,
    "runtime parses root joint and exact named helper keys");
  auto old=parse_gesture_tune("gesture.fp.right.finger_gun.thumb.bend1=10\n");
  check(old.poses[0][1][6].finger[0].bend[0]==10 && old.poses[0][1][6].finger[0].extra[0]==0 &&
    old.poses[0][1][6].helper[0][6]==0,"old preset keeps original fields and defaults new trims to zero");
}

void persistent_reference_tests() {
  // Reproduces the bug: same sliders, reference frozen in a frame with twisted fingers.
  Rig open,twisted;
  for(int i=1;i<20;++i) {
    float turn[16],moved[16];vr_arms_math::axis_angle_rows(0,0,1,.9f,turn);
    multiply_affine(moved,twisted.m[i],turn);std::memcpy(twisted.m[i],moved,48);
  }
  HandReference a,b;
  check(capture_hand(a,open.m,Rig::n,open.names,0) && capture_hand(b,twisted.m,Rig::n,twisted.names,0),"references from two frames");
  Pose p;p.weight=1;p.shape=default_shape(11);
  Rig out_a,out_b;
  check(pose_hand(out_a.m,Rig::n,0,p,a) && pose_hand(out_b.m,Rig::n,0,p,b),"both frames pose");
  float difference=0;
  for(int i=1;i<20;++i) difference=std::max(difference,length(sub(row(out_a.m[i],0),row(out_b.m[i],0))));
  check(difference>.5f,"different frozen frame twists identical sliders");

  // The fix: a session whose first frame differs reloads the saved reference.
  HandReference loaded;
  check(capture_hand(loaded,twisted.m,Rig::n,twisted.names,0),"new session structure");
  check(reference_from_text(reference_text(a),loaded),"saved reference parses");
  check(reference_fits(loaded,twisted.m,Rig::n),"saved reference fits same mesh in any animation frame");
  Rig out_c;
  check(pose_hand(out_c.m,Rig::n,0,p,loaded),"loaded reference poses");
  for(int i=1;i<20;++i) for(int j=0;j<16;++j) check(close(out_c.m[i][j],out_a.m[i][j],2e-4f),"restart reproduces the tuned pose");
  Rig big(1.5f);
  check(!reference_fits(a,big.m,Rig::n),"saved reference rejected on a mesh with other proportions");
  HandReference spoiled;capture_hand(spoiled,open.m,Rig::n,open.names,0);
  const HandReference untouched=spoiled;
  const auto text=reference_text(a);
  check(!reference_from_text(text.substr(0,text.size()/2),spoiled) && spoiled.local==untouched.local,"truncated file rejected atomically");
  check(!reference_from_text("1 2 nan",spoiled),"garbage file rejected");
  check(rig_hash(open.names,Rig::n)!=rig_hash(open.names,Rig::n-1),"rig hash depends on layout");
  const char* partial[]={"L_hand","L_fingerThumb_01"};
  check(hand_bones_present(open.names,Rig::n,0) && !hand_bones_present(partial,2,0),"hand bone presence");

  auto in=input();auto c=context();Policy policy;
  auto f=policy.step(in,c,1000,true);
  check(!f.hand[0].reference_ready,"calibration waits for a settled hand");
  f=advance(policy,in,c,1100);f=advance(policy,in,c,1600);
  check(f.hand[0].reference_ready && f.hand[1].reference_ready,"settled empty hands may calibrate");
  c.owner[1]=Owner::Weapon;f=advance(policy,in,c,1700);
  check(f.hand[0].reference_ready && !f.hand[1].reference_ready,"weapon hand never calibrates");
  c.moving=true;f=advance(policy,in,c,1800);
  check(!f.hand[0].reference_ready,"locomotion frames never calibrate");
}
void curl_path_tests() {
  // [HEADSET 2026-11-07] fingers turned sideways mid-curl. A naturally bent and
  // splayed rest finger plus a fist-like curl (~230 deg) made the shortest swing
  // near-antiparallel. Curling the tip joint 5 deg per step must turn the bone
  // by about 5 deg per step, never jump sideways.
  for(int hand=0;hand<2;++hand) {
    Rig rest;const int base=hand*20,first=4;const float sign=hand?-1.f:1.f;
    for(int k=first+1;k<=first+4;++k) {
      const int seg=k-first-1;const float a=(18.f*seg)*.01745329f,s=10.f*.01745329f;
      for(int j=0;j<3;++j) rest.m[base+1+k][12+j]=rest.m[base+1+k-1][12+j];
      rest.m[base+1+k][12]+=sign*.025f*std::sin(a);rest.m[base+1+k][13]+=.025f*std::sin(s)*std::cos(a);
      rest.m[base+1+k][14]+=.025f*std::cos(s)*std::cos(a);
    }
    HandReference reference;check(capture_hand(reference,rest.m,Rig::n,rest.names,hand),"bent rest reference captures");
    float previous[16]{};bool have=false,smooth=true;float worst=0;
    for(float bend3=0;bend3<=230.f;bend3+=5.f) {
      Rig rig;std::memcpy(rig.m,rest.m,sizeof(rig.m));
      Pose p;p.weight=1;p.shape=default_shape(12);
      for(auto& f:p.shape.finger) {f.bend[0]=f.bend[1]=0;f.bend[2]=0;f.splay=0;f.roll=0;}
      p.shape.finger[1].bend[2]=bend3;
      if(!pose_hand(rig.m,Rig::n,hand,p,reference)) {smooth=false;break;}
      const float* m=rig.m[base+1+first+3];
      if(have) {
        float trace=0;
        for(int r=0;r<3;++r) {
          const Vec u=row(previous,r),v=row(m,r);trace+=dot(u,v)/(length(u)*length(v));
        }
        const float turn=std::acos(std::clamp((trace-1.f)*.5f,-1.f,1.f))/.01745329f;
        worst=std::max(worst,turn);
      }
      std::memcpy(previous,m,64);have=true;
    }
    if(!(smooth && worst<9.f)) std::printf("  curl path hand %d worst step %.1f deg\n",hand,worst);
    check(smooth && worst<9.f,"curling the fingertip follows the curl without a sideways jump");
  }
}
void weapon_free_hand_tests() {
  auto in=input();Policy policy;
  // Resolved exactly as the live context does: right weapon, left free.
  Context c=context();c.weapon=true;c.owner[1]=Owner::Weapon;
  Frame f=policy.step(in,c,1000,true);f=advance(policy,in,c,1200);
  check(f.hand[0].weight>.9f && f.hand[0].combination==12,"free hand takes the rest pose with a weapon out");
  check(f.hand[1].weight==0,"weapon hand stays native");
  in.hand[0].grip=signal(1);in.hand[0].trigger=signal(1);in.hand[1].trigger=signal(1);f=advance(policy,in,c,1500);
  check(f.hand[0].combination!=12 && f.hand[0].weight>.9f,"free hand gestures with a weapon out");
  check(f.hand[1].weight==0,"firing cannot drive the weapon hand's fingers");
  f=advance(policy,in,c,1800);
  check(f.hand[0].reference_ready && !f.hand[0].reference_new,"weapon stance loads a saved reference but never freezes one");
  c.owner[0]=Owner::Support;f=advance(policy,in,c,2300);
  check(f.hand[0].weight<.01f,"two-hand hold returns to the native support pose");
  c.owner[0]=Owner::Empty;f=advance(policy,in,c,2600);
  check(f.hand[0].weight>.9f,"releasing the two-hand hold returns to the gesture");
  c.moving=true;in.hand[0].grip=signal(0);in.hand[0].trigger=signal(0);f=advance(policy,in,c,3000);
  check(f.hand[0].weight>.9f,"untouched free hand keeps the rest pose while walking (weapon out)");

  ContextAudit a;a.readable=true;a.mode=4;a.registry_count=0;
  PlayerOwnership p;p.equipment_readable=p.camera_readable=true;p.active=1;p.held=&a;p.archetype=6;p.camera_override=0;p.transition_state=0;
  Interactions i;i.stable=true;i.dominant=1;
  Context r=resolve_context(context(),a,p,i);
  check(r.owner[0]==Owner::Item && r.owner[1]==Owner::Item,"no weapon proof keeps both hands native");
  i.named_weapon=true;r=resolve_context(context(),a,p,i);
  check(r.weapon && r.owner[0]==Owner::Empty && r.owner[1]==Owner::Weapon,"proven weapon frees the other hand");
  i.support_held=true;i.support=0;r=resolve_context(context(),a,p,i);
  check(r.owner[0]==Owner::Support && r.owner[1]==Owner::Weapon,"two-hand hold owns the support hand");
}
void moving_tests() {
  Sensors s=input().hand[0];
  s.thumb_touch=signal(1);
  check(target(s,false,false).weight==1 && target(s,false,false).curl[0]==1,"thumb gesture while standing");
  check(target(s,false,true).weight==0,"moving: thumb-only touch leaves native locomotion");
  s.thumb_press=signal(1);
  check(target(s,true,true).weight==0,"moving: thumb press (sprint click) ignored");
  s.grip=signal(1);
  const auto grip=target(s,false,true);
  check(grip.weight==1 && grip.curl[0]==0 && grip.curl[2]==1,"moving: grip gesture kept, thumb treated as lifted");
  Sensors idle=input().hand[0];
  check(target(idle,true,false).weight==1 && target(idle,true,true).weight==0,"moving: relaxed idle yields to native");
  check(target(idle,true,true,true).weight==1,"weapon free hand keeps the rest pose while moving");
  {Sensors rest=idle;rest.thumb_touch=signal(1);
   check(target(rest,false,false).weight==1,"standing: thumb touch is a gesture");
   check(target(rest,false,true).weight==0,"moving: stick thumb alone plays native");
   rest.index_touch=signal(1);check(target(rest,false,true).weight==1,"moving: index touch is a gesture");
   check(target(idle,false,false).weight==0,"standing untouched bare hand plays native");}

  auto in=input();auto c=context();Policy p;
  in.hand[0].thumb_touch=signal(1);
  auto f=p.step(in,c,1000,true);f=advance(p,in,c,1300);
  check(f.hand[0].weight>.9f,"standing thumb gesture owns hand");
  c.moving=true;
  for(std::uint64_t t=1400;t<=1900;t+=100) f=advance(p,in,c,t);
  check(f.hand[0].weight<.01f,"starting to move eases thumb gesture out to native");

  MotionGate gate;const float origin[3]{0,0,0};
  check(!gate.step(origin,false,0,1000) && !gate.step(origin,false,0,1100),"still body is not moving");
  check(gate.step(origin,false,.5f,1200),"stick push is moving immediately");
  check(gate.step(origin,false,0,1500) && !gate.step(origin,false,0,1700),"moving held 400 ms after stick release");
  const float walk[3]{.2f,0,0},rise[3]{.2f,.3f,0},slow[3]{.22f,.3f,0},jump[3]{500,.3f,0};
  check(gate.step(walk,false,0,1800),"2 m/s horizontal body speed is moving");
  MotionGate foot,mount;
  foot.step(walk,false,0,1000);mount.step(walk,true,0,1000);
  check(!foot.step(rise,false,0,1100) && mount.step(rise,true,0,1100),"vertical speed counts only when mounted");
  MotionGate sway;sway.step(origin,false,0,1000);
  check(!sway.step(walk,false,0,1400),"0.5 m/s roomscale step is not moving");
  MotionGate teleport;teleport.step(slow,false,0,1000);
  check(!teleport.step(jump,false,0,1100),"teleport/fast travel jump ignored");
  MotionGate blind;
  check(!blind.step(nullptr,false,0,1000) && blind.step(nullptr,false,1,1050),"unreadable body: stick still works");
}
void capture_tests() {
  Rig base;
  base.text[40]="L_thumb_01_expand01";base.names[40]=base.text[40].c_str();
  std::memcpy(base.m[40],base.m[2],64);base.m[40][12]+=.001f;
  float rig_rotation[16];vr_arms_math::axis_angle_rows(0,.6f,.8f,.9f,rig_rotation);
  for(int rotated=0;rotated<2;++rotated) for(int hand=0;hand<2;++hand) for(int combo=0;combo<combination_count;++combo) {
    Rig rig=base;
    if(rotated) for(auto& bone:rig.m) {float world[16];multiply_affine(world,bone,rig_rotation);std::memcpy(bone,world,64);}
    HandReference ref;
    check(capture_hand(ref,rig.m,Rig::n,rig.names,hand),"capture reference");
    Pose known;known.weight=1;known.shape=default_shape(combo);
    known.shape.finger[1].roll=12;known.shape.finger[2].extra[10]=-9;known.shape.finger[0].extra[7]=6;
    known.shape.finger[3].extra[4]=2;
    if(!hand) {known.shape.helper[0][0]=10;known.shape.helper[0][2]=-6;known.shape.helper[0][4]=1.5f;known.shape.helper[0][6]=15;}
    Rig native=rig;
    check(pose_hand(native.m,Rig::n,hand,known,ref),"known authored pose for capture");
    Shape got;CaptureReport report;
    check(solve_shape(native.m,Rig::n,hand,ref,got,report),"native pose captured");
    check(report.position_mm<.05f && report.angle_deg<.2f,"captured sliders reproduce native palette");
    for(int f=0;f<4;++f) for(int j=0;j<3;++j)
      check(close(got.finger[f].bend[j],known.shape.finger[f].bend[j],.05f),"captured bend matches authored bend");
    check(close(got.finger[1].roll+got.finger[1].extra[9],12,.1f),"captured skin roll");
    check(close(got.finger[3].extra[4],2,.05f),"captured base sideways offset");
    if(!hand) check(report.helpers==1 && close(got.helper[0][0],10,.1f) && close(got.helper[0][6],15,.1f),
      "captured thumb webbing helper trims");
  }
  Rig rig;HandReference ref;Shape got;CaptureReport report;
  check(capture_hand(ref,rig.m,Rig::n,rig.names,0),"capture reference");
  Rig straight=rig;Pose flat;flat.weight=1;flat.shape=Shape{};
  check(pose_hand(straight.m,Rig::n,0,flat,ref),"pose equal to reference directions stays finite");
  rig.m[8][0]=std::numeric_limits<float>::quiet_NaN();
  check(!solve_shape(rig.m,Rig::n,0,ref,got,report),"invalid native palette is not captured");
  auto request=parse_gesture_tune("gesture.capture_hand=1\ngesture.capture_id=42\ngesture.capture_until=1000\n");
  check(request.capture_hand==1 && request.capture_id==42 && request.capture_until==1000,"capture request keys parsed");
}
void context_audit_tests() {
  // Regression for the live capture's short-circuit: a null/unreadable tool
  // component must not suppress camera mode or registry observations.
  std::map<const void*,std::vector<unsigned char>> memory;
  const void* body=reinterpret_cast<void*>(1),*holder=reinterpret_cast<void*>(2),*registry=reinterpret_cast<void*>(3),
    *tool=reinterpret_cast<void*>(4),*mode=reinterpret_cast<void*>(5);
  auto store=[&](const void* p,unsigned off,const auto& v) {
    auto& data=memory[p]; if(data.size()<off+sizeof(v)) data.resize(off+sizeof(v));
    std::memcpy(data.data()+off,&v,sizeof(v));
  };
  auto read=[&](const void* p,unsigned off,auto& v) {
    const auto it=memory.find(p); if(it==memory.end() || off+sizeof(v)>it->second.size()) return false;
    std::memcpy(&v,it->second.data()+off,sizeof(v)); return true;
  };
  const void* absent=nullptr;
  store(body,0x1F0,registry); store(registry,0x20,0u); store(body,0x258,absent);
  store(holder,0x40,mode); store(mode,0xC,4);
  auto out=sample_context(body,holder,read);
  check(out.readable && out.mode==4 && out.tool==0 && !out.tool_component && out.read_mask==55,
    "null optional tool still observes mode and registry");
  store(body,0x258,tool); out=sample_context(body,holder,read);
  check(!out.readable && out.mode==4 && out.tool==-1 && out.registry_count==0 && out.read_mask==55,
    "unreadable tool never becomes false or blocks independent fields");
  store(tool,0x10,static_cast<unsigned char>(1)); out=sample_context(body,holder,read);
  check(out.readable && out.tool==1 && out.read_mask==63,"all candidate fields readable");
  store(holder,0x40,absent); out=sample_context(body,holder,read);
  check(!out.readable && out.mode==-1 && out.tool==1,"missing mode retains tool evidence with unknown sentinel");
  store(holder,0x40,mode); store(body,0x1F0,absent); out=sample_context(body,holder,read);
  check(out.readable && !out.registry && out.registry_count==0 && out.mode==4,"null registry independently tagged");

  const void* equipment=reinterpret_cast<void*>(6),*definition=reinterpret_cast<void*>(7),*held=reinterpret_cast<void*>(8),
    *transition=reinterpret_cast<void*>(9),*animation=reinterpret_cast<void*>(10),*control=reinterpret_cast<void*>(11);
  store(body,0x250,equipment); store(equipment,0x3E40,absent); store(equipment,0x3E58,absent);
  store(equipment,0x3F64,static_cast<unsigned char>(0)); store(equipment,0x3E88,transition); store(transition,0x98,0);
  store(body,0x1A8,animation); store(animation,0x538,control); store(control,0x80,static_cast<unsigned char>(0));
  auto native=sample_player_ownership(body,read);
  check(native.equipment_readable && native.camera_readable && native.active==0 && !native.held && native.camera_override==0,
    "native empty equipment and camera ownership snapshot");
  auto c=context(); Interactions i; i.stable=true;
  auto resolved=resolve_context(c,out,native,i);
  check(resolved.verified && !resolved.cinematic && resolved.owner[0]==Owner::Empty && resolved.owner[1]==Owner::Empty,
    "known unarmed gameplay enables both independent hands");
  store(equipment,0x3E40,held); store(equipment,0x3E58,definition);
  store(equipment,0x3F64,static_cast<unsigned char>(1)); store(definition,0x3A8,5);
  native=sample_player_ownership(body,read); resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Item && resolved.owner[1]==Owner::Item,"unknown equipped item blocks both poses");
  i.named_weapon=true; resolved=resolve_context(c,out,native,i);
  check(resolved.weapon && resolved.owner[1]==Owner::Weapon && resolved.owner[0]==Owner::Empty,"right weapon free left idle");
  i.dominant=0; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Weapon && resolved.owner[1]==Owner::Empty,"left weapon free right idle");
  i.support_held=true; i.support=1; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[1]==Owner::Support,"two-hand support retains native fingers");
  i.support_only=true; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Empty && resolved.owner[1]==Owner::Weapon,"support-only hold frees former dominant hand");
  i.support_held=i.support_only=false; i.harness[1]=true; c.mounted=true; out.mode=7;
  resolved=resolve_context(c,out,native,i); check(!resolved.cinematic && resolved.owner[1]==Owner::Harness,"riding harness wins");
  i.swap=true; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Swap && resolved.owner[1]==Owner::Swap,"swap excludes both even with harness state");
  i.swap=false; i.harness[1]=false; i.busy[1]=true; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[1]==Owner::Item,"scan and equip gestures own native hand");
  i.busy[1]=false; native.archetype=14; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Item && resolved.owner[1]==Owner::Item,"feeding arm cannot reuse named weapon evidence");
  native.archetype=5; i.stable=false; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Item,"equipment transition eases to native until stable");
  i.stable=true; native.transition_state=1; resolved=resolve_context(c,out,native,i);
  check(resolved.owner[0]==Owner::Item,"native holstering retains fingers");
  native.transition_state=2; resolved=resolve_context(c,out,native,i); check(resolved.owner[1]==Owner::Item,"native unholstering retains fingers");
  native.transition_state=0; native.camera_override=1; resolved=resolve_context(c,out,native,i);
  check(resolved.cinematic,"native camera override cancels gestures immediately");
  native.camera_override=0; out.registry_count=1; resolved=resolve_context(c,out,native,i);
  check(resolved.cinematic,"all native sequence bindings conservatively cancel gestures");
  out.registry_count=0; out.mode=20; resolved=resolve_context(c,out,native,i); check(resolved.cinematic,"mount transition native");
  out.mode=999; resolved=resolve_context(c,out,native,i); check(resolved.cinematic,"unknown camera mode native");
  out.mode=7; native.camera_readable=false; resolved=resolve_context(c,out,native,i);
  check(!resolved.verified && resolved.cinematic,"missing camera ownership cannot enable override");
  store(equipment,0x3F64,static_cast<unsigned char>(2)); native=sample_player_ownership(body,read);
  check(!native.equipment_readable,"malformed equipment flag rejected");
}
}
int main(int argc,char** argv) {
  if(argc==2 && !std::strcmp(argv[1],"--defaults")) {
    for(int c=0;c<combination_count;++c) {
      const auto shape=default_shape(c);
      for(int f=0;f<4;++f) {
        for(int j=0;j<3;++j) std::printf("%d %d bend%d %.7g\n",c,f,j+1,shape.finger[f].bend[j]);
        std::printf("%d %d splay %.7g\n%d %d roll %.7g\n",c,f,shape.finger[f].splay,c,f,shape.finger[f].roll);
        for(int j=0;j<extra_count;++j) std::printf("%d %d %s %.7g\n",c,f,extra_name(j),shape.finger[f].extra[j]);
      }
      for(int b=0;b<thumb_helper_count;++b) for(int a=0;a<helper_axis_count;++a)
        std::printf("%d %d %s %.7g\n",c,4+b,helper_axis_name(a),shape.helper[b][a]);
    }
    return 0;
  }
  if(argc==3 && !std::strcmp(argv[1],"--bindings")) {
    for(const auto& b:sensor_bindings(argv[2])) std::printf("%d %s\n",b.id,b.path.c_str());
    return 0;
  }
  policy_tests(); pose_tests(); stable_gesture_tests(); thumb_control_tests(); persistent_reference_tests(); curl_path_tests(); weapon_free_hand_tests(); moving_tests(); capture_tests(); context_audit_tests(); if(failures) return 1; std::puts("hand_gestures_test passed"); return 0;
}
