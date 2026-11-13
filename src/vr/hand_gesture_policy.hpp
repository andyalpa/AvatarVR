#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "hand_gesture_tune.hpp"

// Portable policy. Unsupported is different from supported-and-not-touched.
namespace hand_gestures {
struct Signal { float value=0.f; bool supported=false,active=false; };
struct Sensors {
  Signal grip,trigger,index_touch,thumb_touch,thumb_press;
  bool tracked=false;
};
struct Input {
  Sensors hand[2];
  std::uint64_t at=0,sequence=0;
  bool focused=false;
};
enum class Owner { Unknown, Empty, Weapon, Item, Support, Harness, Swap };
struct Context {
  Owner owner[2]={Owner::Unknown,Owner::Unknown};
  bool verified=false,gameplay=false,cinematic=true,mounted=false,weapon=false;
  // Walking/running/riding [AUTHOR 2026-10-29]: thumb sensors ignored (thumb is on
  // the stick), untouched hands return to native locomotion animation.
  bool moving=false;
  std::uint64_t at=0,identity=0;
};
struct Pose {
  float curl[4]={.15f,.15f,.2f,.2f}; // thumb, index, middle, pinky
  float weight=0.f;
  Shape shape=default_shape(12);
  float mix[combination_count]{};
  int combination=12;
  Owner owner=Owner::Unknown;
  bool reference_ready=false; // settled empty hand: a saved HandReference may load
  bool reference_new=false; // and no weapon stance: its native frame may become a new one
};
struct Frame { bool mounted=false; Pose hand[2]; std::uint64_t at=0,sequence=0,identity=0; };
inline bool fresh(std::uint64_t now,std::uint64_t at) {return at && now>=at && now-at<=250;}
inline float value(const Signal& s) {
  return s.supported && s.active && std::isfinite(s.value)?std::clamp(s.value,0.f,1.f):0.f;
}
inline bool available(const Signal& s) {return s.supported && s.active;}
// [AUTHOR 2026-11-07] Bare hand: untouched = native; moving = thumb (stick) ignored, any other
// touch/press gestures. One-handed weapon out (rest_while_moving): the free hand keeps the
// rest pose all the time, also while moving, until a touch/press picks a gesture.
inline Pose target(const Sensors& s,bool idle,bool moving=false,bool rest_while_moving=false) {
  Pose out;
  const float grip=value(s.grip),trigger=value(s.trigger);
  const bool index=value(s.index_touch)>.5f;
  const bool thumb=!moving && (value(s.thumb_touch)>.5f || value(s.thumb_press)>.5f);
  const bool gesture=grip>.03f || trigger>.03f || index || thumb;
  out.weight=(gesture || (idle && (!moving || rest_while_moving)))?1.f:0.f;
  if(!gesture) {out.mix[12]=1.f; return out;}
  // An unavailable touch sensor never means "finger lifted". Keep a relaxed
  // finger unless a corresponding press/value can drive it.
  out.curl[0]=thumb?1.f:available(s.thumb_touch)?0.f:.15f;
  out.curl[1]=trigger>.03f?trigger:index?.22f:available(s.index_touch)?0.f:.15f;
  out.curl[2]=out.curl[3]=available(s.grip)?grip:.2f;
  float index_mix[3]={1.f-out.curl[1],0.f,out.curl[1]};
  if(index) {index_mix[0]=0;index_mix[1]=1.f-trigger;index_mix[2]=trigger;}
  for(int g=0;g<2;++g) for(int i=0;i<3;++i) for(int th=0;th<2;++th)
    out.mix[g*6+i*2+th]=(g?out.curl[2]:1.f-out.curl[2])*index_mix[i]*(th?out.curl[0]:1.f-out.curl[0]);
  out.combination=static_cast<int>(std::max_element(out.mix,out.mix+combination_count)-out.mix);
  return out;
}
// Locomotion from the left stick (intent, immediate) or body speed (also covers a
// mount that keeps moving without stick input). Horizontal on foot so jumps and
// roomscale head sway do not count; full 3D when mounted. Held 400 ms after stop.
struct MotionGate {
  static constexpr float stick_threshold=.25f,speed_threshold=1.2f; // m/s; slow roomscale steps stay below
  float last[3]{};
  std::uint64_t last_at=0,until=0;
  bool step(const float* position,bool mounted,float stick,std::uint64_t now) {
    bool fast=false;
    if(!position) last_at=0;
    else if(!last_at || now<last_at) {std::copy(position,position+3,last);last_at=now;}
    else if(now-last_at>=50) {
      const float dt=static_cast<float>(now-last_at)*.001f;
      const float dx=position[0]-last[0],dy=mounted?position[1]-last[1]:0.f,dz=position[2]-last[2];
      const float speed=std::sqrt(dx*dx+dy*dy+dz*dz)/dt;
      fast=std::isfinite(speed) && speed>speed_threshold && speed<100.f && dt<1.f; // teleports/stalls ignored
      std::copy(position,position+3,last);last_at=now;
      fast_=fast;
    } else fast=fast_;
    if(fast || (std::isfinite(stick) && stick>stick_threshold)) until=now+400;
    return now<until;
  }
 private:
  bool fast_=false;
};
class Policy {
  Frame frame_;
  std::uint64_t input_sequence_=0,context_at_=0,settled_since_[2]{};
 public:
  const Frame& frame() const {return frame_;}
  Frame step(const Input& in,const Context& c,std::uint64_t now,bool enabled,const Tuning* tuning=nullptr,double wall_seconds=0.) {
    const bool valid=enabled && c.verified && c.gameplay && !c.cinematic && in.focused &&
      fresh(now,in.at) && fresh(now,c.at);
    if(!valid || (frame_.identity && frame_.identity!=c.identity)) {
      frame_=Frame{}; input_sequence_=context_at_=0; settled_since_[0]=settled_since_[1]=0;
    }
    if(!valid) return frame_;
    if(in.sequence==input_sequence_ && c.at==context_at_) return frame_;
    const float dt=frame_.at && now>=frame_.at?static_cast<float>(now-frame_.at)*.001f:0.f;
    const float alpha=1.f-std::exp(-dt/.060f);
    for(int h=0;h<2;++h) {
      auto& p=frame_.hand[h]; p.owner=c.owner[h];
      if(!in.hand[h].tracked || c.owner[h]==Owner::Unknown || c.owner[h]==Owner::Swap) {
        p=Pose{}; p.owner=c.owner[h]; continue;
      }
      if(c.owner[h]!=Owner::Empty) {
        // Known native ownership eases the previous pose out. New gestures
        // cannot drive occupied fingers during this handoff.
        p.weight+=alpha*(0.f-p.weight); if(p.weight<.0001f) p.weight=0.f;
        continue;
      }
      Pose t=target(in.hand[h],c.mounted || c.weapon,c.moving,c.weapon);
      static const Tuning defaults;
      const Tuning& settings=tuning?*tuning:defaults;
      if(settings.preview_context==static_cast<int>(c.mounted) && settings.preview_hand==h &&
         settings.preview_combo>=0 && settings.preview_combo<combination_count &&
         wall_seconds>0 && settings.preview_until>=wall_seconds && settings.preview_until-wall_seconds<=3.) {
        std::fill(std::begin(t.mix),std::end(t.mix),0.f);
        t.combination=settings.preview_combo;t.mix[t.combination]=1.f;t.weight=1.f;
      }
      t.shape=mix_shape(t.mix,settings,c.mounted?settings.mounted_context:0,h);
      p.combination=t.combination;
      std::copy(std::begin(t.mix),std::end(t.mix),std::begin(p.mix));
      if(p.weight==0.f) p.shape=t.shape;
      if(t.weight>0.f) for(int f=0;f<4;++f) {
        for(int j=0;j<3;++j) p.shape.finger[f].bend[j]+=alpha*(t.shape.finger[f].bend[j]-p.shape.finger[f].bend[j]);
        p.shape.finger[f].splay+=alpha*(t.shape.finger[f].splay-p.shape.finger[f].splay);
        p.shape.finger[f].roll+=alpha*(t.shape.finger[f].roll-p.shape.finger[f].roll);
        for(int i=0;i<extra_count;++i) p.shape.finger[f].extra[i]+=alpha*(t.shape.finger[f].extra[i]-p.shape.finger[f].extra[i]);
      }
      if(t.weight>0.f) for(int b=0;b<thumb_helper_count;++b) for(int a=0;a<helper_axis_count;++a) {
        p.shape.helper[b][a]+=alpha*(t.shape.helper[b][a]-p.shape.helper[b][a]);
      }
      // At zero weight the curl is invisible: initialize it to the new target
      // and ease ownership in; otherwise reversals start at the displayed curl.
      if(p.weight==0.f) for(int f=0;f<4;++f) p.curl[f]=t.curl[f];
      if(t.weight>0.f) for(int f=0;f<4;++f) p.curl[f]+=alpha*(t.curl[f]-p.curl[f]);
      p.weight+=alpha*(t.weight-p.weight);
      if(p.weight<.0001f && t.weight==0.f) p.weight=0.f;
    }
    for(int h=0;h<2;++h) {
      // Never freeze a reference from a weapon grip, swap or locomotion frame.
      const bool settled=c.owner[h]==Owner::Empty && !c.moving;
      if(!settled) settled_since_[h]=0; else if(!settled_since_[h]) settled_since_[h]=now;
      frame_.hand[h].reference_ready=settled && now-settled_since_[h]>=500;
      frame_.hand[h].reference_new=frame_.hand[h].reference_ready && !c.weapon;
    }
    frame_.mounted=c.mounted; frame_.at=now; frame_.sequence++; frame_.identity=c.identity;
    input_sequence_=in.sequence; context_at_=c.at;
    return frame_;
  }
};
} // namespace hand_gestures
