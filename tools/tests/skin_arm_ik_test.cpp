#include "../../src/diagnostics/skin_arm_ik.hpp"
#include "../../src/vr/pitch_level.hpp"
#include <cstdio>
#include <limits>
using namespace skin_arm_ik;
bool near(float a,float b){return std::fabs(a-b)<1e-4f;}
int main(){
  for(Vec target:{Vec{0.5f,0.3f,0.2f},Vec{4,0,0},Vec{0.001f,0,0},Vec{-0.5f,0,0}}){
    Solution s{};if(!solve({0,0,0},target,{0,-1,0},0.45f,0.5f,s)) return 1;
    if(!near(length(s.elbow),0.45f)||!near(length(sub(s.wrist,s.elbow)),0.5f)) return 2;
    if(length(target)<0.95f && length(target)>0.05f && length(sub(s.wrist,target))>1e-4f) return 3;
  }
  Solution s{};if(solve({0,0,0},{0,0,0},{0,1,0},0.4f,0.4f,s)||
    solve({0,0,0},{1,0,0},{0,1,0},-1,1,s)) return 4;
  const Vec flipped=rotate({1,0,0},{1,0,0},{-1,0,0});if(!near(flipped.x,-1)||!near(length(flipped),1)) return 5;
  float source[211][16]{},copy[211][16]{};
  for(auto& m:source){m[0]=m[5]=m[10]=m[15]=1;}
  source[114][12]=0.45f;source[115][12]=0.95f;
  source[15][12]=-0.45f;source[16][12]=-0.95f;std::memcpy(copy,source,sizeof(copy));
  if(!apply(copy,source,1,{0.5f,0.3f,0},{0,-1,0})) return 6;
  if(!near(copy[115][12],0.5f)||!near(copy[115][13],0.3f)||source[115][12]!=0.95f) return 7;
  for(unsigned bone=0;bone<211;++bone) if(bone<113||bone>115)
    for(unsigned j=0;j<16;++j) if(copy[bone][j]!=source[bone][j]) return 8;
  if(!apply(copy,source,0,{-0.5f,0.3f,0},{0,-1,0}) || !near(copy[16][12],-0.5f) ||
     !near(copy[115][12],0.5f) || source[16][12]!=-0.95f || copy[0][12]!=source[0][12]) return 10;
  source[113][0]=std::numeric_limits<float>::quiet_NaN();if(apply(copy,source,1,{1,0,0},{0,1,0})) return 9;
  float grip[16]{},hand[16]{},rolled[16]{};
  grip[0]=grip[5]=grip[10]=grip[15]=1;
  hand[2]=-1;hand[5]=1;hand[8]=1;hand[12]=3;hand[13]=4;hand[14]=5;hand[15]=1;
  const float root_half=std::sqrt(0.5f);
  grip[0]=root_half;grip[2]=-root_half;grip[8]=root_half;grip[10]=root_half;
  WristCalibration calibration;
  if(!calibrate(calibration,hand,grip)) return 11;
  float oriented[16];std::memcpy(oriented,hand,sizeof(hand));
  if(!orient(oriented,calibration,grip)) return 12;
  for(unsigned i=0;i<16;++i) if(!near(oriented[i],hand[i])) return 13;
  std::memcpy(rolled,grip,sizeof(grip));
  for(unsigned i=0;i<3;++i){rolled[i*4]=-grip[i*4+1];rolled[i*4+1]=grip[i*4];}
  if(!orient(oriented,calibration,rolled)) return 14;
  for(unsigned i=0;i<3;++i){if(!near(oriented[i*4],-hand[i*4+1])||
    !near(oriented[i*4+1],hand[i*4])||!near(oriented[i*4+2],hand[i*4+2])) return 15;}
  for(unsigned i:{3u,7u,11u,12u,13u,14u,15u}) if(oriented[i]!=hand[i]) return 16;
  rolled[0]=std::numeric_limits<float>::quiet_NaN();
  float before[16];std::memcpy(before,oriented,sizeof(before));
  if(orient(oriented,calibration,rolled)||std::memcmp(before,oriented,sizeof(before))) return 17;
  { // Followers: finger beyond wrist, forearm twist, clavicle at shoulder, left bone near right hand, body bone.
    float base[211][16]{},out[211][16]{};
    for(auto& m:base){m[0]=m[5]=m[10]=m[15]=1;}
    base[113][12]=0.f;base[114][12]=0.3f;base[115][12]=0.6f;
    base[115][0]=0;base[115][1]=1;base[115][4]=-1;base[115][5]=0; // rotated wrist basis
    base[120][12]=0.68f;base[120][13]=0.02f; // R finger
    base[121][12]=0.45f;base[121][13]=0.01f; // R forearm twist
    base[122][12]=0.01f; // R clavicle at shoulder: must stay
    base[30][12]=0.65f; // L bone next to right hand: must stay
    base[0][12]=0.62f; // body bone, no side: must stay
    signed char side[211];for(auto& v:side) v=-1;
    side[113]=side[114]=side[115]=side[120]=side[121]=side[122]=1;side[30]=0;
    int owner[211];
    if(classify(owner,base,1,side)!=2||owner[120]!=115||owner[121]!=114||owner[122]!=-1||owner[30]!=-1||owner[0]!=-1) return 20;
    std::memcpy(out,base,sizeof(out));
    if(!apply(out,base,1,{0.2f,0.4f,0.1f},{0,-1,0})) return 21;
    // Emulate an oriented wrist: rows (0,0,1),(-1,0,0),(0,-1,0), a proper rotation.
    out[115][0]=0;out[115][1]=0;out[115][2]=1;out[115][4]=-1;out[115][5]=0;out[115][6]=0;
    out[115][8]=0;out[115][9]=-1;out[115][10]=0;
    if(follow(out,base,owner)!=2) return 22;
    for(unsigned bone:{120u,121u}){ // relative transform child*inv(joint) must be unchanged
      const unsigned joint=static_cast<unsigned>(owner[bone]);
      float inv_before[16],inv_after[16],rel_before[16],rel_after[16];
      if(!inverse_affine(inv_before,base[joint])||!inverse_affine(inv_after,out[joint])) return 23;
      multiply_affine(rel_before,base[bone],inv_before);multiply_affine(rel_after,out[bone],inv_after);
      for(unsigned i=0;i<15;++i) if(!near(rel_before[i],rel_after[i])) return 24;
    }
    for(unsigned bone:{0u,30u,122u}) for(unsigned j=0;j<16;++j) if(out[bone][j]!=base[bone][j]) return 25;
    float m[16]{2,0,0,0, 0,0.5f,0.3f,0, 0,-0.2f,1,0, 4,5,6,1},inv[16],id[16];
    if(!inverse_affine(inv,m)) return 26;
    multiply_affine(id,m,inv);
    for(unsigned i=0;i<16;++i) if(!near(id[i],(i%5==0)?1.f:0.f)) return 27;
  }
  { // Name roles from the live 211 dump.
    struct Case {const char* name;int role;};
    const Case cases[]={{"R_fingerIndex_03",2},{"R_fingerMiddle_Target",2},{"L_fingerThumb_03_expand01",2},{"R_thumb_01_expand01",2},
      {"R_handWeapon_Target",2},{"L_weaponAttachmentHand",2},{"R_hand_expand05",2},{"R_lowerArm_twist05",1},{"L_lowerArm_driver01",1},
      {"R_upperArm_bend01",0},{"L_upperArm_expand07",0},{"R_hand",-1},{"L_upperArm",-1},{"R_lowerArm",-1},{"L_shoulder",-1},
      {"R_shoulderRotateOffset",-1},{"C_head",-1},{"FP_Hands",-1},{"CameraMarker",-1},{nullptr,-1}};
    for(const Case& c:cases) if(name_role(c.name)!=c.role) return 30;
    signed char side[211],role[211];for(auto& v:side) v=-1;for(auto& v:role) v=-1;
    side[138]=1;role[138]=2;side[195]=1;role[195]=1;side[203]=1;role[203]=0;side[39]=0;role[39]=2;side[115]=1;role[115]=-1;
    int owner[211];
    if(owners_from_roles(owner,side,role,1)!=3||owner[138]!=115||owner[195]!=114||owner[203]!=113||owner[39]!=-1||owner[115]!=-1) return 31;
    if(owners_from_roles(owner,side,role,0)!=1||owner[39]!=16) return 32;
  }
  { // Anatomical alignment: hand forward = wrist->middle knuckle, side = pinky->index.
    float hand[16]{};hand[0]=hand[5]=hand[10]=hand[15]=1;  // animated wrist at origin, identity axes
    const Vec middle{0,0,0.1f},index{0,0.03f,0.09f},pinky{0,-0.03f,0.09f};
    float grip[16]{};grip[0]=grip[5]=grip[10]=grip[15]=1;grip[12]=1;grip[13]=2;grip[14]=3;
    HandTune t;Vec target{};float h[16];
    std::memcpy(h,hand,sizeof(h));
    if(!align_hand(h,target,middle,index,pinky,grip,t)) return 40;
    for(int i=0;i<16;++i) if(!near(h[i],hand[i])) return 41;  // frames already agree: axes unchanged
    if(!near(target.x,1)||!near(target.y,2)||!near(target.z,3-0.075f)) return 42;  // wrist behind the palm origin
    float yawed[16]{};yawed[2]=-1;yawed[5]=1;yawed[8]=1;yawed[15]=1;  // grip turned: forward now +x, right now -z
    std::memcpy(h,hand,sizeof(h));
    if(!align_hand(h,target,middle,index,pinky,yawed,t)) return 43;
    // hand row 2 (was forward +z) must now point +x; row 1 (up) stays up
    if(!near(h[8],1)||!near(h[9],0)||!near(h[10],0)||!near(h[5],1)) return 44;
    t.pitch=30;std::memcpy(h,hand,sizeof(h));
    if(!align_hand(h,target,middle,index,pinky,grip,t)) return 45;
    if(!(h[9]>0.49f&&h[9]<0.51f)||!near(h[10],std::cos(0.5235988f))) return 46;  // +pitch raises the fingers 30 deg
    t=HandTune{};t.forward=0.1f;t.up=0.05f;t.side=-0.02f;
    yawed[12]=1;yawed[13]=2;yawed[14]=3;std::memcpy(h,hand,sizeof(h));
    if(!align_hand(h,target,middle,index,pinky,yawed,t)) return 47;   // offsets no longer follow the grip's rotation
    if(!near(target.x,1-0.075f)||!near(target.y,2)||!near(target.z,3)) return 48;
    float basis[16]{};basis[0]=basis[5]=basis[10]=basis[15]=1;
    const Vec moved=body_offset(Vec{1,2,3},basis,t);  // body frame: right x, up y, forward z
    if(!near(moved.x,0.98f)||!near(moved.y,2.05f)||!near(moved.z,3.1f)) return 49;
    float bad[16];std::memcpy(bad,grip,sizeof(bad));bad[0]=std::numeric_limits<float>::quiet_NaN();
    std::memcpy(h,hand,sizeof(h));float before[16];std::memcpy(before,h,sizeof(h));
    if(align_hand(h,target,middle,index,pinky,bad,t)||std::memcmp(h,before,sizeof(h))) return 39;
  }
  { // Left hand mirrors the left-right tuning; twist measured and applied about the forearm axis.
    HandTune t;t.pitch=-65;t.yaw=-30;t.roll=-35;t.forward=-0.08f;t.up=-0.1f;t.side=-0.13f;
    const HandTune l=for_hand(t,0),r=for_hand(t,1);
    if(l.yaw!=30||l.roll!=35||!near(l.side,0.13f)||l.pitch!=-65||!near(l.up,-0.1f)||!near(l.forward,-0.08f)) return 50;
    if(r.yaw!=-30||r.roll!=-35||!near(r.side,-0.13f)) return 51;
    float rigid[16]{};rigid[0]=rigid[5]=rigid[10]=rigid[15]=1;
    float twisted[16];std::memcpy(twisted,rigid,sizeof(twisted));
    const Vec axis{1,0,0};
    twist_bone(twisted,axis,0.6f);
    if(!near(twist_angle(rigid,twisted,axis),0.6f)) return 52;
    if(!near(twisted[0],1)||!near(twisted[5],std::cos(0.6f))||!near(twisted[6],std::sin(0.6f))) return 53;
    twist_bone(twisted,axis,-1.0f);
    if(!near(twist_angle(rigid,twisted,axis),-0.4f)) return 54;
    // Swing about another axis does not read as twist.
    float swung[16];std::memcpy(swung,rigid,sizeof(swung));twist_bone(swung,Vec{0,1,0},0.5f);
    if(std::fabs(twist_angle(rigid,swung,axis))>1e-4f) return 55;
  }
  { // Pitch auto-level: a simulated game integrates the stick (rate 2 rad/s at full stick, 60 Hz) with either sign.
    for(int true_sign:{1,-1}) for(float start:{0.6f,-0.9f,0.1f}) {
      pitch_level::Controller c;float pitch=start;
      for(int i=0;i<600;++i){const float stick=c.step(pitch);
        const float eff=std::fabs(stick)<0.265f?0.f:stick;  // game deadzone
        pitch+=true_sign*eff*2.f/60.f;}
      if(std::fabs(pitch)>0.03f) return 60+(true_sign>0?0:1);  // converges within ~2 degrees
      if(true_sign<0 && c.sign!=-1 && std::fabs(start)>0.2f) return 62;  // inverted look learned
    }
    pitch_level::Controller c;
    if(c.step(0.005f)!=0.f) return 63;                   // inside the deadband: centred stick
    const float s1=c.step(0.2f);if(!(s1<=-0.35f&&s1>=-0.7f)) return 64;  // looking up: push down, above the deadzone
    if(c.step(std::numeric_limits<float>::quiet_NaN())!=0.f) return 65;
  }
  { // Non-zero default pitch: holds -25 degrees from above and below.
    for(float start:{0.3f,-0.9f}){pitch_level::Controller c;c.target=-0.4363f;float pitch=start;
      for(int i=0;i<600;++i){const float stick=c.step(pitch);const float eff=std::fabs(stick)<0.265f?0.f:stick;pitch+=eff*2.f/60.f;}
      if(std::fabs(pitch-c.target)>0.03f) return 66;}
  }
  { // Arm scaling: real shoulder from the head in the body frame; the offset from it is replayed scaled from the Na'vi shoulder.
    float basis[16]{};basis[0]=basis[5]=basis[10]=basis[15]=1;  // right x, up y, forward z
    ArmScale a;a.user_length=0.6f;a.drop=0.25f;a.width=0.4f;a.back=0.05f;a.reach=1.f;
    const Vec head{0,1.7f,0};
    const Vec ls=real_shoulder(head,basis,0,a),rs=real_shoulder(head,basis,1,a);
    if(!near(ls.x,-0.2f)||!near(rs.x,0.2f)||!near(ls.y,1.45f)||!near(rs.z,-0.05f)) return 90;
    const Vec navi{0.3f,2.2f,0.1f};float used=0;
    // Straight forward at full real reach -> full Na'vi reach (0.9 m).
    const Vec t=scale_reach(add(rs,Vec{0,0,0.6f}),rs,navi,0.9f,a,&used);
    if(!near(used,1.5f)||!near(t.z,0.1f+0.9f)||!near(t.x,0.3f)||!near(t.y,2.2f)) return 91;
    const Vec h=scale_reach(add(rs,Vec{0.1f,-0.2f,0.3f}),rs,navi,0.9f,a,&used);  // half reach keeps proportions
    if(!near(h.x,0.45f)||!near(h.y,1.9f)||!near(h.z,0.55f)) return 92;
    a.reach=1.1f;scale_reach(rs,rs,navi,0.9f,a,&used);if(!near(used,1.65f)) return 93;
  }
  { // T-pose calibration: span 1.70 m, hands 0.22 m below the eyes and 0.05 m behind them.
    float basis[16]{};basis[0]=basis[5]=basis[10]=basis[15]=1;
    const Vec head{0,1.70f,0},left{-0.85f,1.48f,-0.05f},right{0.85f,1.48f,-0.05f};
    const TPose t=tpose_calibration(left,right,head,basis);
    if(!t.ok||!near(t.span,1.7f)||!near(t.width,0.34f)||!near(t.length,0.68f)||!near(t.drop,0.22f)||!near(t.back,0.05f)) return 94;
    // The calibrated shoulder lands on the line between the hands, 0.17 m from the centre.
    ArmScale a;a.width=t.width;a.drop=t.drop;a.back=t.back;a.user_length=t.length;
    const Vec rs=real_shoulder(head,basis,1,a);
    if(!near(rs.x,0.17f)||!near(rs.y,1.48f)||!near(rs.z,-0.05f)||!near(length(sub(right,rs)),0.68f)) return 95;
    const TPose forward=tpose_calibration(Vec{-0.2f,1.48f,0.6f},Vec{0.2f,1.48f,0.6f},head,basis);  // arms forward
    if(forward.ok) return 96;
  }
  { // Turning compensation: positive yaw turns forward (+z) towards +x, about the given centre; axes rotate too.
    float m[16]{};m[0]=m[5]=m[10]=m[15]=1;m[12]=2;m[13]=1;m[14]=3;  // point 1 m in front of centre (2,*,2)
    rotate_about_up(m,2,2,1.5707963f);
    if(!near(m[12],3)||!near(m[13],1)||!near(m[14],2)) return 97;
    if(!near(m[8],1)||!near(m[10],0)||!near(m[0],0)||!near(m[2],-1)) return 98;  // forward row -> +x, right row -> -z
  }
  { // One-Euro: jitter around a still value is reduced; a fast move is followed with little lag.
    OneEuro f;float worst=0;
    for(int i=0;i<200;++i){const float noise=(i%2?1.f:-1.f)*0.003f;const float y=f.filter(0.5f+noise,1.f/72.f);if(i>20) worst=std::max(worst,std::fabs(y-0.5f));}
    if(worst>0.0015f) return 99;                         // at least halved
    OneEuro g;float y=0;for(int i=0;i<36;++i) y=g.filter(i/36.f,1.f/72.f);  // 2 m/s ramp
    if(std::fabs(y-35.f/36.f)>0.05f) return 100;          // lag under 5 cm at 2 m/s
    float m[16]{};m[0]=m[5]=m[10]=m[15]=1;OneEuro fs[12];smooth_grip(m,fs,1.f/72.f);
    if(!near(m[0],1)||!near(m[5],1)||!near(m[10],1)) return 101;
  }
  { // Accessory spring: rest stays on target, a step lags then settles, the clamp holds, the pivot distance is kept.
    Dangle d;const Vec pivot{0,0,0};Vec target{0,-0.1f,0};
    Vec x=d.step(pivot,target,1.f/72.f,2.f,0.35f,0.04f);
    for(int i=0;i<200;++i) x=d.step(pivot,target,1.f/72.f,2.f,0.35f,0.04f);
    if(length(sub(x,target))>1e-4f) return 102;
    target={0.1f,0,0};float lag_max=0;                     // the arm swings: target jumps 14 cm (under the reset limit)
    for(int i=0;i<300;++i){x=d.step(pivot,target,1.f/72.f,2.f,0.35f,0.04f);lag_max=std::max(lag_max,length(sub(x,target)));
      if(std::fabs(length(sub(x,pivot))-0.1f)>1e-4f) return 103;}  // length preserved
    if(lag_max>0.04f+1e-4f || lag_max<0.01f) return 104;  // lagged, within the clamp
    if(length(sub(x,target))>1e-3f) return 105;          // settled
    x=d.step(pivot,{5,0,0},1.f/72.f,2.f,0.35f,0.04f);    // teleport resets
    if(length(sub(x,Vec{5,0,0}))>1e-6f) return 106;
    float m[16]{};m[0]=m[5]=m[10]=m[15]=1;m[13]=-0.1f;   // bone hanging 10 cm below the pivot, swung to +x
    if(!swing_about(m,{0,0,0},{0,-0.1f,0},{0.1f,0,0})) return 107;
    if(!near(m[12],0.1f)||!near(m[13],0)||std::fabs(length(row(m,0))-1.f)>1e-4f) return 108;
  }
  { // Model-space hand target: grip at the user's shoulder -> Na'vi shoulder; displacement scaled; independent of head position.
    float basis[16]{}; basis[0]=basis[5]=basis[10]=basis[15]=1.f;  // right +x, up +y, forward +z
    ArmScale a; a.enabled=true; a.user_length=0.6f; a.width=0.4f; a.drop=0.2f; a.back=0.f; a.reach=1.f;
    const Vec navi{0.3f,1.5f,0.f};
    float used=0.f;
    const Vec at_shoulder=model_hand_point(Vec{0.2f,-0.2f,0.f},basis,1,a,navi,1.2f,&used);  // right shoulder = head + (0.2,-0.2,0)
    if(length(sub(at_shoulder,navi))>1e-5f || std::fabs(used-2.f)>1e-5f) return 109;
    const Vec forward=model_hand_point(Vec{0.2f,-0.2f,0.5f},basis,1,a,navi,1.2f,&used);  // 0.5 m forward -> 1.0 m (x2)
    if(std::fabs(forward.z-1.0f)>1e-5f || std::fabs(forward.x-0.3f)>1e-5f) return 110;
    a.enabled=false;
    const Vec plain=model_hand_point(Vec{0.2f,-0.2f,0.5f},basis,1,a,navi,1.2f,&used);
    if(std::fabs(plain.z-0.5f)>1e-5f) return 111;
    float rot[16]{}; rot[0]=0; rot[2]=-1; rot[5]=1; rot[8]=1; rot[10]=0; rot[15]=1;  // 90 deg about +y
    const Vec r=rotate_dir(Vec{0,0,1},rot); if(std::fabs(r.x-1.f)>1e-5f || std::fabs(r.z)>1e-5f) return 112;
  }
  std::puts("model-space hand: shoulder maps to Na'vi shoulder, reach scaled, unscaled 1:1, direction rotation pass");
  std::puts("accessory spring: rest, lag within clamp, settle, length kept, teleport reset, swing about pivot pass");
  std::puts("pitch level: converges from up/down with normal and inverted stick, deadband, minimum stick, NaN pass");
  std::puts("tuning mirror and forearm twist: left mirrors yaw/roll/side, twist angle/apply round-trip, swing excluded pass");
  std::puts("hand alignment: frame match, grip yaw, pitch sign, wrist offsets, invalid grip rejection pass");
  std::puts("arm IK: reach, lengths, antiparallel rotation, input rejection and source/body isolation pass");
  std::puts("name roles: fingertips/targets/props -> hand, lowerArm_* -> forearm, upperArm_* -> upper; shoulders/joints/body excluded");
  std::puts("followers: same-side finger/twist ride their joint rigidly; clavicle, other side and body untouched");
  std::puts("wrist calibration: nonidentity baseline, controller roll, translation preservation and invalid pose rejection pass");
}
