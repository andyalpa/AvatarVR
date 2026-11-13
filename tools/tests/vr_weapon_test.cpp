#include "../../src/vr/vr_weapon_math.hpp"
#include "../../src/vr/mounted_weapon_math.hpp"
#include "../../src/vr/mounted_hand_pose.hpp"
#include "../../src/vr/mounted_weapon_profiles.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
using namespace vr_weapon_math;
namespace {
bool near(float a, float b, float e = 1e-4f) { return std::fabs(a - b) < e; }
void identity(float* m) { for (int i = 0; i < 16; ++i) m[i] = (i % 5 == 0) ? 1.f : 0.f; }
}
int main() {
  {
    namespace mw=mounted_weapon_math;
    // A transient palette built from an older successful entity placement must
    // rebase once to the latest commit, including rotation, without resampling
    // an animated socket or changing the weapon's internal geometry.
    float old[16],latest[16],offset[16],actor[16],recovered[16],child_local[16],child[16],expected[16],mapped[16];
    identity(old); old[12]=100.f; old[13]=40.f;
    std::memcpy(latest,old,64); latest[12]+=.2f;
    latest[0]=0.f; latest[2]=-1.f; latest[8]=1.f; latest[10]=0.f;
    identity(offset); offset[14]=.12f; skin_arm_ik::twist_bone(offset,{0,0,1},.2f);
    skin_arm_ik::multiply_affine(actor,offset,old);
    mw::PlacedFrames history; history.push(old,100,actor);
    float latest_actor[16]; skin_arm_ik::multiply_affine(latest_actor,offset,latest);
    history.push(latest,116,latest_actor);
    if(!history.entity_offset(actor,120,recovered)) return 170;
    for(int i=0;i<16;++i) if(!near(recovered[i],offset[i])) return 171;
    identity(child_local); child_local[12]=.02f; child_local[13]=.03f; child_local[14]=.3f;
    skin_arm_ik::multiply_affine(child,child_local,actor);
    skin_arm_ik::multiply_affine(expected,child_local,latest_actor);
    float recovered_target[16]; skin_arm_ik::multiply_affine(recovered_target,recovered,latest);
    if(!mw::map(child,actor,recovered_target,mapped)) return 172;
    for(int i=0;i<16;++i) if(!near(mapped[i],expected[i],.0001f)) return 173;
    float unchanged[16];
    if(!mw::map(mapped,latest_actor,latest_actor,unchanged)) return 174;
    for(int i=0;i<16;++i) if(!near(unchanged[i],mapped[i],.0001f)) return 175;
    // Matching a target alone is insufficient: only an actual successful actor
    // frame may initialize the transient part, and expiry leaves output intact.
    std::memcpy(recovered,offset,64);
    if(history.entity_offset(old,120,recovered) || history.entity_offset(actor,351,recovered) ||
       history.entity_offset(actor,99,recovered)) return 176;
    for(int i=0;i<16;++i) if(!near(recovered[i],offset[i])) return 177;
    actor[0]=NAN; if(history.entity_offset(actor,120,recovered)) return 178;
    history=mw::PlacedFrames{};
    if(history.entity_offset(latest_actor,120,recovered)) return 179;
    std::puts("mounted transient draw: previous actor recovery, rotated rebase, local geometry, no double transform, target-only/stale/future/NaN/reset rejection PASS");
  }
  {
    namespace mw=mounted_weapon_math;
    // No mounted shot may borrow TP socket axes when its native on-foot ray
    // calibration is absent, even with an otherwise fresh visual placement.
    for(int flags=0;flags<8;++flags) {
      const bool ready=mw::shot_ready((flags&1)!=0,(flags&2)!=0,(flags&4)!=0,200,100,150);
      if(ready!=(flags==7)) return 160;
    }
    if(mw::shot_ready(true,true,true,401,100,350) || mw::shot_ready(true,true,true,401,350,100) ||
       mw::shot_ready(true,true,true,99,100,100)) return 161;
    // Before: once the engine skinned from our already-placed matrix, the
    // palette stopped matching its flat socket and ownership expired at 250 ms.
    // Renew only from fresh, committed frames; include the preceding placement
    // because the palette and entity query can run in different phases.
    mw::PlacedFrames history; float offset[16],old[16],next[16],native[16];
    identity(offset); identity(old); old[12]=100.f; std::memcpy(next,old,64); next[12]+=.15f;
    history.push(old,100); history.push(next,115);
    for(int i=0;i<12;++i) history.push(next,116+i); // repeated queries must not evict the older distinct frame
    if(!history.matches(old,offset,true,130) || !history.matches(next,offset,true,130)) return 162;
    offset[13]=.02f; skin_arm_ik::multiply_affine(native,offset,old);
    if(!history.matches(native,offset,false,130)) return 163;
    native[12]+=1.f; if(history.matches(native,offset,false,130)) return 164;
    if(history.matches(old,offset,true,400)) return 165;
    std::memcpy(native,next,64); native[14]+=.08f;
    history.push(next,140,native);
    if(!history.matches(native,offset,false,150)) return 168; // native actor root differs from the private palette offset
    history=mw::PlacedFrames{};
    if(history.matches(next,offset,true,130)) return 166; // player/mount/weapon proof reset
    std::uint64_t bound=100;
    for(std::uint64_t time=116;time<1500;time+=16) {
      history.push(next,time);
      if(history.matches(next,offset,true,time)) bound=time;
      if(!mw::fresh(time,bound)) return 167;
    }
    std::puts("mounted stability: reject uncalibrated shots; already-placed and previous-frame ownership renewal, duplicate queries, expiry/reset PASS");
  }
  {
    namespace store=mounted_weapon_profiles;
    store::Profile p; p.valid=true; p.aim_valid=true; p.forward={0,0,1}; p.muzzle={0,.02f,.3f};
    identity(p.relative); p.relative[14]=.35f; identity(p.trigger_root); p.trigger_root[13]=-.08f;
    identity(p.fp_root_socket); p.fp_root_socket[14]=.1f;
    for(auto& hand:p.fingers) {hand.valid=true; for(int k=0;k<mounted_hand_pose::count;++k) hand.point[k]={.01f*k,0,.02f+.005f*k};}
    const std::string key="BLUE/BAKED/CHARACTERART/WEAPON/PLAYER/RIFLE/FRAME.MMB";
    store::Cache cache{{key,p}},restored; const auto text=store::encode(cache);
    // All palette workers feed the same resource collector. Controller/world
    // movement is not a readiness input; native relative geometry must settle.
    int player=1,weapon=2,replacement=3;
    store::StableHold collector;
    for(std::uint64_t now=100;now<=700;now+=50)
      if(collector.observe(&player,&weapon,p,now)!=(now==700)) return 180;
    if(collector.observe(&player,&replacement,p,750)) return 181;
    auto moving=p; moving.trigger_root[12]+=.01f;
    if(collector.observe(&player,&replacement,moving,800) || collector.observe(&player,&replacement,moving,950)) return 182;
    if(collector.observe(&player,&replacement,moving,900)) return 183; // clock moved backward
    auto invalid=p; invalid.trigger_root[0]=NAN;
    if(collector.observe(&player,&replacement,invalid,950) || collector.last) return 184;
    if(collector.observe(nullptr,&weapon,p,1000) || collector.last) return 185;
    std::map<std::string,store::StableHold> windows;
    for(std::uint64_t now=100;now<=700;now+=50) {
      if(windows["rifle"].observe(&player,&weapon,p,now)!=(now==700) ||
         windows["shotgun"].observe(&player,&replacement,p,now)!=(now==700)) return 186;
    }
    std::puts("mounted native reference collection: shared/interleaved resources, 600 ms stability, identity/animation/gap/future/invalid resets PASS");
    if(text.empty() || !store::decode(text,restored) || store::encode(restored)!=text) return 140;
    // A new process starts with no native observation; a saved profile is enough.
    cache.clear(); if(!store::decode(text,cache) || !cache.at(key).aim_valid || !cache.at(key).fingers[1].valid) return 141;
    const auto saved=store::encode(cache);
    for(auto corrupt:{text.substr(0,text.size()/2),text+"junk",std::string(store::max_bytes+1,'x')})
      if(store::decode(corrupt,cache) || store::encode(cache)!=saved) return 142;
    auto bad=p; bad.trigger_root[0]=NAN; if(!store::encode({{key,bad}}).empty()) return 143;
    if(!store::encode({{"../WEAPON/PLAYER/BAD.MMB",p}}).empty()) return 144;
    bad=p; bad.fp_root_socket[5]=0; if(!store::valid(p) || store::valid(bad)) return 145;
    if(!store::same_hold(p,p)) return 146;
    bad=p; bad.trigger_root[12]+=.004f; if(store::same_hold(p,bad)) return 147;
    bad=p; bad.fingers[0].point[2].x+=.004f; if(store::same_hold(p,bad)) return 148;
    bad=p; skin_arm_ik::twist_bone(bad.trigger_root,{0,0,1},.03f); if(store::same_hold(p,bad)) return 155;
    std::string version="AVR_MOUNT_HOLDS 1 98588DA2 0\n";
    version+="CHECKSUM "+std::to_string(store::checksum(version))+"\n";
    if(store::decode(version,cache)) return 149;
    store::Cache too_many; for(unsigned k=0;k<=store::max_profiles;++k) too_many[key+std::to_string(k)]=p;
    if(!store::encode(too_many).empty()) return 150;
    std::puts("mounted profiles: restart restoration, exact float roundtrip, checksum/version/bounds/NaN rejection, stable native grip PASS");
  }
  {
    // Native on-foot barrel calibration follows the committed gun including
    // two-hand rotation; neither hand selection nor a guessed model axis enters.
    float root[16]; identity(root); root[12]=-7700.f; root[13]=2000.f;
    const float s[3]={-7700.f,2000.02f,.3f},t[3]={-7700.f,2000.02f,500.3f};
    skin_arm_ik::Vec muzzle,forward;
    if(!local_ray(s,t,root,muzzle,forward) || !near(muzzle.z,.3f) || !near(forward.z,1.f)) return 151;
    root[0]=0; root[2]=-1; root[8]=1; root[10]=0; root[12]+=.2f;
    float out_s[3],out_t[3];
    if(!calibrated_shot(s,t,root,muzzle,forward,3.f,out_s,out_t) ||
       !near(out_t[0]-out_s[0],500.f,.002f) || !near(out_t[2]-out_s[2],0.f,.002f)) return 152;
    root[12]+=5.f; if(calibrated_shot(s,t,root,muzzle,forward,3.f,out_s,out_t)) return 153;
    if(local_ray(s,s,root,muzzle,forward)) return 154;
    std::puts("mounted aim: on-foot native barrel measurement, committed two-hand rotation, range and teleport/degenerate guards PASS");
  }
  {
    namespace hp=mounted_hand_pose;
    using namespace skin_arm_ik;
    float reference[hp::count][16],out[hp::count][16]; hp::Pose pose; pose.valid=true;
    for(int i=0;i<hp::count;++i) {
      identity(reference[i]); twist_bone(reference[i],{0,0,1},i*.13f);
      const int p=hp::parent[i];
      const Vec previous=p<0?Vec{}:position(reference[p]);
      const Vec q=add(previous,p<0?Vec{.015f*(hp::family(i)+1),0,.04f}:Vec{0,0,.025f});
      reference[i][12]=q.x; reference[i][13]=q.y; reference[i][14]=q.z;
      const Vec source_parent=p<0?Vec{}:pose.point[p];
      pose.point[i]=add(source_parent,p<0?Vec{.02f,.01f,.05f}:Vec{0,-.04f,.015f});
    }
    for(bool mirror:{false,true}) {
      if(!hp::retarget(reference,pose,mirror,out)) return 130;
      for(int i=0;i<hp::count;++i) {
        const int p=hp::parent[i];
        const float old_length=length(p<0?position(reference[i]):sub(position(reference[i]),position(reference[p])));
        const float new_length=length(p<0?position(out[i]):sub(position(out[i]),position(out[p])));
        if(!near(old_length,new_length) || !mounted_weapon_math::rigid(out[i])) return 131;
        Vec want=p<0?pose.point[i]:sub(pose.point[i],pose.point[p]);
        if(mirror) want.x=-want.x;
        const Vec actual=p<0?position(out[i]):sub(position(out[i]),position(out[p]));
        if(dot(want,actual)/(length(want)*length(actual))<.9999f) return 132;
      }
      // Finger helpers keep their own rigid local attachment to the retargeted bone.
      float helper[16],inverse[16],delta[16],moved[16],local_before[16],local_after[16];
      std::memcpy(helper,reference[1],64); helper[12]+=.004f;
      inverse_affine(inverse,reference[1]); multiply_affine(delta,inverse,out[1]); multiply_affine(moved,helper,delta);
      if(!mounted_weapon_math::relative(helper,reference[1],local_before) ||
         !mounted_weapon_math::relative(moved,out[1],local_after) || !mounted_weapon_math::same_frame(local_before,local_after,.0001f)) return 133;
    }
    float saved[hp::count][16]; std::memcpy(saved,out,sizeof(out));
    pose.point[6].x=NAN;
    if(hp::retarget(reference,pose,false,out) || std::memcmp(saved,out,sizeof(out))) return 134;
    if(hp::index("L_fingerIndex_02")!=6 || hp::index("R_hand")!=-1 || hp::family("L_thumb_driver")!=0 || hp::family("L_bracelet")!=-1) return 135;
    // A slow native query still owns the gun at 60/200 ms: arm pins must share
    // that validity, rather than dropping to free IK at the old 50 ms boundary.
    if(!mounted_weapon_math::fresh(1060,1000) || !mounted_weapon_math::fresh(1200,1000) || mounted_weapon_math::fresh(1251,1000)) return 136;
    // FP and TP placement sockets can have different root offsets. Carry the
    // accepted hold through the actual weapon root, not between raw sockets.
    float fp_root_in_socket[16],tp_root_in_socket[16],native_hold[16],inverse[16],root_hold[16],mounted_hold[16];
    identity(fp_root_in_socket); fp_root_in_socket[14]=.12f;
    identity(tp_root_in_socket); tp_root_in_socket[13]=.02f;
    identity(native_hold); native_hold[12]=.04f; native_hold[13]=-.08f;
    inverse_affine(inverse,fp_root_in_socket); multiply_affine(root_hold,native_hold,inverse);
    multiply_affine(mounted_hold,root_hold,tp_root_in_socket);
    float gun[16],palm[16],mounted_socket[16],placed_root[16],recovered[16]; identity(gun); gun[12]=300.f;
    multiply_affine(palm,root_hold,gun); inverse_affine(inverse,mounted_hold); multiply_affine(mounted_socket,inverse,palm);
    multiply_affine(placed_root,tp_root_in_socket,mounted_socket); multiply_affine(recovered,root_hold,placed_root);
    if(!mounted_weapon_math::same_frame(placed_root,gun,.001f) || !mounted_weapon_math::same_frame(recovered,palm,.001f)) return 137;
    std::puts("mounted hand pose: both grips, own segment lengths/axes, helper attachment, atomic rejection, named coverage and shared expiry PASS");
  }
  {
    namespace mw=mounted_weapon_math;
    using namespace skin_arm_ik;
    float wrist[16],palm[16],hold[16],mirrored[16],again[16]; identity(wrist);
    if(!mw::palm(wrist,{0,0,.1f},{.04f,0,.1f},{-.04f,0,.1f},palm) || !mw::rigid(palm)) return 120;
    if(mw::palm(wrist,{0,0,0},{0,0,.1f},{0,0,.1f},again)) return 121;
    for(int i=0;i<24;++i) {
      std::memcpy(hold,palm,64); skin_arm_ik::twist_bone(hold,{0,0,1},i*.261799388f);
      hold[12]=.04f; hold[13]=-.12f; hold[14]=.06f;
      mw::mirror_palm_hold(hold,mirrored); mw::mirror_palm_hold(mirrored,again);
      if(!mw::rigid(mirrored)) return 122;
      for(int k=0;k<16;++k) if(!near(hold[k],again[k])) return 123;
      // Left accepted grip and mirrored right grip both reconstruct the same
      // gun from their own anatomical palm. Native right socket is irrelevant.
      float gun[16],left[16],right[16],inverse[16],out[16]; identity(gun); gun[12]=100.f; gun[13]=2.f;
      multiply_affine(left,hold,gun); multiply_affine(right,mirrored,gun);
      inverse_affine(inverse,mirrored); multiply_affine(out,inverse,right);
      if(!mw::same_frame(out,gun,.001f)) return 124;
      // Mapping both arm rigs through palm-local cancels arbitrary wrist axes.
      float bone[16],local[16],recovered[16]; identity(bone); skin_arm_ik::twist_bone(bone,{1,0,0},i*.2f);
      if(!mw::relative(left,bone,local)) return 125;
      multiply_affine(recovered,local,bone);
      if(!mw::same_frame(recovered,left,.001f)) return 126;
    }
    std::puts("mounted palm grips: handed reflection, wrist-axis independence, grip reconstruction and degenerate rejection PASS");
  }
  {
    namespace mw=mounted_weapon_math;
    float native[16],source[16],target[16],out[16];
    {
      float root[16],socket[16],bone[16],goal[16],mapped[16];
      identity(root); root[12]=-6400.f; root[13]=1120.f; root[14]=-5050.f; std::memcpy(socket,root,64);
      socket[12]+=.18f; // one flight tick: same_frame refuses, the bound object stays owned
      if(mw::same_frame(root,socket) || !mw::bound_carry(true,true,root,socket)) return 140;
      if(mw::bound_carry(true,false,root,socket) || mw::bound_carry(false,true,root,socket)) return 141;
      socket[12]+=1.f; if(mw::bound_carry(true,true,root,socket)) return 142; // holstered
      std::memcpy(bone,root,64); bone[0]=bone[5]=bone[10]=0.f; bone[13]+=.1f; // zero-scale firing part
      std::memcpy(goal,root,64); goal[12]+=1.f;
      if(mw::map(bone,root,goal,mapped)) return 143;
      if(!mw::map_part(bone,root,goal,mapped) || !near(mapped[12],goal[12],.002f) || !near(mapped[13],goal[13]+.1f,.002f) || !near(mapped[0],0.f)) return 144;
      bone[3]=std::nanf(""); if(mw::map_part(bone,root,goal,mapped)) return 145;
      float then[3]={10.f,0.f,0.f},now[3]={12.5f,0.f,0.f},frame[16];
      std::memcpy(frame,goal,64); const float x=frame[12];
      if(!mw::carry(frame,then,now) || !near(frame[12],x+2.5f,.01f)) return 146;
      now[0]=100.f; std::memcpy(frame,goal,64); if(mw::carry(frame,then,now) || frame[12]!=goal[12]) return 147;
      float palm_then[16],palm_now[16];
      std::memcpy(palm_then,root,64); std::memcpy(palm_now,root,64); palm_now[12]+=.4f; palm_now[13]-=.2f;
      std::memcpy(frame,palm_then,64); // one-hand commit: the gun hand becomes the free hand exactly
      if(!mw::follow_palm(frame,palm_then,palm_now) || !mw::same_frame(frame,palm_now,.001f)) return 148;
      std::memcpy(frame,goal,64); // kept relation (gun 1 m from the palm)
      if(!mw::follow_palm(frame,palm_then,palm_now) || !near(frame[12],palm_now[12]+1.f,.002f)) return 149;
      palm_now[0]=0.f; std::memcpy(frame,goal,64); if(mw::follow_palm(frame,palm_then,palm_now) || frame[12]!=goal[12]) return 150;
    }
    identity(native); native[12]=-10000.f; native[13]=1500.f; native[14]=400.f;
    std::memcpy(source,native,64);
    if(!mw::same_frame(native,source)) return 101;
    identity(out); if(mw::same_frame(native,out)) return 102; // inactive FP candidate is identity
    out[0]=-1.f; if(mw::rigid(out)) return 103; // reflected geometry is not a rigid weapon frame
    for(float x:{-.35f,.35f}) {
      std::memcpy(target,source,64); target[12]+=x; target[13]+=.2f;
      // Native source is always L; destination is whichever hand is selected. No source-hand swap.
      native[14]=source[14]+.01f;
      if(!mw::map(native,source,target,out) || !near(out[12],target[12],.002f) ||
         !near(out[14],target[14]+.01f,.002f)) return 104;
      if(!mw::same_frame(out,target)) return 105; // detects the already-placed draw before applying a second delta
      float moved_source[16],moved_native[16],moved_target[16],carried[16];
      std::memcpy(moved_source,source,64); std::memcpy(moved_native,native,64); std::memcpy(moved_target,target,64);
      moved_source[12]+=.7f; moved_native[12]+=.7f; moved_target[12]+=.7f;
      if(!mw::map(moved_native,moved_source,moved_target,carried) || !near(carried[12]-out[12],.7f,.003f)) return 106;
    }
    std::memcpy(target,source,64); target[0]=0.f; target[2]=-1.f; target[8]=1.f; target[10]=0.f;
    target[12]+=.4f; native[14]=source[14]+.1f;
    if(!mw::map(native,source,target,out) || !near(out[12],target[12]+.1f,.002f) ||
       !near(out[14],target[14],.002f) || !near(out[2],-1.f)) return 111;
    mw::Identity a; int owner=0,mount=0,candidate=0,other=0; a.owner=&owner; a.partner=&mount; a.candidate=&candidate; a.id[0]=3; a.id[1]=4;
    if(!mw::same_identity(a,a)) return 112;
    for(int field=0;field<5;++field) {
      auto b=a;
      if(field==0) b.owner=&other; if(field==1) b.partner=&other; if(field==2) b.candidate=&other;
      if(field==3) ++b.id[0]; if(field==4) ++b.id[1];
      if(mw::same_identity(a,b)) return 113;
    }
    std::memcpy(target,source,64); target[12]+=4.f; if(mw::map(native,source,target,out)) return 107;
    std::memcpy(target,source,64); target[0]=NAN; if(mw::map(native,source,target,out)) return 108;
    std::memcpy(target,source,64); target[5]=0.f; if(mw::map(native,source,target,out)) return 109;
    if(!mw::fresh(350,100) || mw::fresh(351,100) || mw::fresh(99,100) || mw::fresh(1,0)) return 110;
    std::puts("mounted weapon: left native source to either hand, large-world carry, offset preservation, identity/reflection/NaN/singular rejection, freshness PASS");
  }
  const float s[3] = {1.f, 2.f, 3.f}, t[3] = {1.f, 2.f, 13.f};  // aim +z, 10 m
  float flat[16], ik[16], s2[3], t2[3];
  // 1. IK == flat: unchanged.
  identity(flat); flat[12] = 0.5f; flat[13] = 1.8f; flat[14] = 2.9f;
  for (int i = 0; i < 16; ++i) ik[i] = flat[i];
  if (!map_shot(s, t, flat, ik, 1.5f, s2, t2)) return 1;
  if (!near(s2[0], 1) || !near(s2[1], 2) || !near(s2[2], 3) || !near(t2[2], 13)) return 2;
  // 2. IK socket moved 0.2 m left and yawed 90 deg about +y: start moves rigidly, aim turns with it, distance kept.
  identity(ik);
  ik[0] = 0; ik[2] = -1; ik[8] = 1; ik[10] = 0;  // row0 (1,0,0)->(0,0,-1), row2 (0,0,1)->(1,0,0)
  ik[12] = 0.3f; ik[13] = 1.8f; ik[14] = 2.9f;
  if (!map_shot(s, t, flat, ik, 1.5f, s2, t2)) return 3;
  // start relative to flat socket: (0.5,0.2,0.1) -> rotated (0.1,0.2,-0.5) + (0.3,1.8,2.9)
  if (!near(s2[0], 0.4f) || !near(s2[1], 2.0f) || !near(s2[2], 2.4f)) return 4;
  // direction +z -> +x, length 10
  if (!near(t2[0] - s2[0], 10.f) || !near(t2[1] - s2[1], 0.f) || !near(t2[2] - s2[2], 0.f)) return 5;
  // 3. Implausible shift rejected.
  ik[12] = 5.f;
  if (map_shot(s, t, flat, ik, 1.5f, s2, t2)) return 6;
  // 4. Degenerate aim (target == start) rejected.
  for (int i = 0; i < 16; ++i) ik[i] = flat[i];
  if (map_shot(s, s, flat, ik, 1.5f, s2, t2)) return 7;
  // 2026-10-31 audit. A weapon object's place on the socket is frozen while moving, so game motion never reaches the gun; on the
  // socket means exactly the socket; a new weapon may settle. The second entity keeps its own place through the commit history.
  {
    auto lerp=[](const float* a,const float* b,float t,float* out){for(int i=0;i<16;++i) out[i]=a[i]+(b[i]-a[i])*t;};
    float socket[16],entity[16]; identity(socket); identity(entity);
    socket[12]=-7284.f; socket[13]=2037.f; socket[14]=725.f;
    entity[12]=socket[12]+0.31f; entity[13]=socket[13]; entity[14]=socket[14];
    BoundOffset b; const void* gun=&b; int other=0;
    update_bound(b,gun,entity,socket,false,1000,lerp);
    if(!b.have || b.on_socket || !near(b.rel[12],0.31f,1e-2f)) return 60;
    float swayed[16]; std::memcpy(swayed,entity,64); swayed[13]+=0.12f;  // locomotion sway, after the weapon came up
    update_bound(b,gun,swayed,socket,false,2500,lerp);
    update_bound(b,gun,swayed,socket,false,2520,lerp);
    if(!near(b.rel[13],0.f,1e-3f)) return 61;
    update_bound(b,gun,swayed,socket,true,3520,lerp);  // standing still it may follow slowly
    if(!(b.rel[13]>0.05f)) return 62;
    BoundOffset onto; float close[16]; std::memcpy(close,socket,64); close[12]+=0.005f;
    update_bound(onto,gun,close,socket,false,1000,lerp);
    if(!onto.on_socket) return 63;
    update_bound(onto,&other,entity,socket,false,1100,lerp);  // another weapon: a new place
    if(onto.on_socket) return 64;
    // Second entity: a palette that follows it finds its fixed offset in the same history as the main gun.
    namespace mw=mounted_weapon_math;
    float target[16],second[16],found[16]; identity(target); identity(second);
    target[12]=10.f; second[12]=10.31f;
    mw::PlacedFrames history; history.push(target,100); history.push(target,100,second);
    if(!history.entity_offset(second,120,found) || !near(found[12],0.31f,1e-4f)) return 65;
    if(!history.entity_offset(target,120,found) || !near(found[12],0.f,1e-4f)) return 66;
    // Single precision at this location: a cell-space sum keeps sub-millimetre motion that a world-space sum rounds away.
    const float world=-7284.3f, cell=-1024.f*7.f, step=0.0002f;
    int world_steps=0,cell_steps=0; float last_world=world,last_cell=world-cell;
    for(int i=1;i<=10;++i) {
      const float w=world+step*i, c=(world-cell)+step*i;
      if(w!=last_world) {++world_steps; last_world=w;}
      if(c!=last_cell) {++cell_steps; last_cell=c;}
    }
    if(!(cell_steps>=10 && world_steps<=5)) return 67;
  }
  std::puts("vr_weapon: identity, rigid move with aim rotation and kept distance, shift limit, degenerate aim pass");
  return 0;
}
