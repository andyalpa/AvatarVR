#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>

namespace hand_gestures {
constexpr int combination_count=13; // grip(2) x index(lift/touch/press) x thumb(2), plus idle
constexpr int extra_count=12,thumb_helper_count=14,helper_axis_count=9;
inline const char* extra_name(int i) {
  static const char* names[extra_count]={"root_pitch","root_yaw","root_roll","root_normal","root_side","root_forward",
    "spread1","spread2","spread3","roll1","roll2","roll3"};
  return names[i];
}
inline const char* helper_name(int i) {
  static const char* names[thumb_helper_count]={"thumb_01_expand01","thumb_01_expand02","thumb_01_expand03","thumb_01_expand04",
    "fingerThumb_02_expand01","fingerThumb_02_expand02","fingerThumb_02_expand03",
    "fingerThumb_03_expand01","fingerThumb_03_expand02","fingerThumb_03_expand03",
    "fingerThumb_02_driver01","fingerThumb_02_driver02","fingerThumb_03_driver01","fingerThumb_03_driver02"};
  return names[i];
}
inline const char* helper_axis_name(int i) {
  static const char* names[helper_axis_count]={"pitch","yaw","roll","offset_x","offset_y","offset_z","scale_x","scale_y","scale_z"};
  return names[i];
}
inline int thumb_helper_index(const char* name) {
  if(!name || std::strlen(name)<3 || name[1]!='_' || (name[0]!='L' && name[0]!='R')) return -1;
  for(int i=0;i<thumb_helper_count;++i) if(!std::strcmp(name+2,helper_name(i))) return i;
  return -1;
}
struct FingerShape {float bend[3]{},splay=0.f,roll=0.f;float extra[extra_count]{};};
struct Shape {FingerShape finger[4];float helper[thumb_helper_count][helper_axis_count]{};};
inline const char* combination_name(int combo) {
  static const char* names[combination_count]={
    "open","thumb","index_touch","index_touch_thumb","index_press","index_press_thumb",
    "finger_gun","point","grip_index_touch","grip_index_touch_thumb","thumbs_up","fist","idle"};
  return combo>=0 && combo<combination_count?names[combo]:"unknown";
}
inline Shape default_shape(int combo) {
  Shape out;
  const bool idle=combo==12,grip=!idle && combo>=6,thumb=!idle && (combo%2);
  const int index=idle?0:(combo%6)/2;
  const float curls[4]={idle?.15f:thumb?1.f:0.f,idle?.15f:index==2?1.f:index==1?.22f:0.f,
                       idle?.2f:grip?1.f:0.f,idle?.2f:grip?1.f:0.f};
  const float open[4][3]={{0,8,5},{4,6,3},{7,8,5},{10,10,6}};
  const float closed[4][3]={{45,40,25},{65,85,60},{70,90,65},{75,90,65}};
  for(int f=0;f<4;++f) {
    for(int j=0;j<3;++j) out.finger[f].bend[j]=open[f][j]+curls[f]*(closed[f][j]-open[f][j]);
    out.finger[f].splay=f==0?60.f-25.f*curls[f]:f==1?4.f:f==3?-7.f:0.f;
  }
  return out;
}
struct Tuning {
  Shape poses[2][2][combination_count]; // on-foot/mounted, left/right; independent rig-specific edits
  int preview_context=-1,preview_hand=-1,preview_combo=-1;
  // [AUTHOR 2026-11-11] mounted gestures must be the on-foot ones: riding uses the fp shapes (0) unless gesture.mounted_context 1 asks
  // for the separate tp set ([LIVE] AvatarVR_ik_tune.txt: 728 fp lines per hand, 10 tp lines).
  int mounted_context=0;
  double preview_until=0;
  // Tuner capture request: copy one hand's native finger pose into slider values.
  int capture_hand=-1;
  double capture_id=0,capture_until=0;
  Tuning() {
    for(auto& context:poses) for(auto& hand:context) for(int c=0;c<combination_count;++c) hand[c]=default_shape(c);
  }
};
inline bool tune_key(Tuning& out,const std::string& key,double value) {
  if(!std::isfinite(value)) return false;
  if(key=="gesture.preview_until") {out.preview_until=value;return true;}
  if(key=="gesture.preview_context") {out.preview_context=static_cast<int>(std::clamp(value,-1.,1.));return true;}
  if(key=="gesture.mounted_context") {out.mounted_context=static_cast<int>(std::clamp(value,0.,1.));return true;}
  if(key=="gesture.preview_hand") {out.preview_hand=static_cast<int>(std::clamp(value,-1.,1.));return true;}
  if(key=="gesture.preview_combo") {out.preview_combo=static_cast<int>(std::clamp(value,-1.,12.));return true;}
  if(key=="gesture.capture_hand") {out.capture_hand=static_cast<int>(std::clamp(value,-1.,1.));return true;}
  if(key=="gesture.capture_id") {out.capture_id=value;return true;}
  if(key=="gesture.capture_until") {out.capture_until=value;return true;}
  char context[16]{},hand[16]{},name[40]{},finger[40]{},axis[24]{},extra=0;
  if(std::sscanf(key.c_str(),"gesture.%15[^.].%15[^.].%39[^.].%39[^.].%23[^.]%c",
    context,hand,name,finger,axis,&extra)!=5) return false;
  const int r=!std::strcmp(context,"fp")?0:!std::strcmp(context,"tp")?1:-1;
  const int h=!std::strcmp(hand,"left")?0:!std::strcmp(hand,"right")?1:-1;
  int c=-1,f=-1;
  for(int i=0;i<combination_count;++i) if(!std::strcmp(name,combination_name(i))) c=i;
  const char* fingers[]={"thumb","index","middle","pinky"};
  for(int i=0;i<4;++i) if(!std::strcmp(finger,fingers[i])) f=i;
  if(r<0 || h<0 || c<0) return false;
  if(f<0) {
    for(int b=0;b<thumb_helper_count;++b) if(!std::strcmp(finger,helper_name(b)))
      for(int a=0;a<helper_axis_count;++a) if(!std::strcmp(axis,helper_axis_name(a))) {
        const double limit=a<3?90.:a<6?20.:75.;
        out.poses[r][h][c].helper[b][a]=static_cast<float>(std::clamp(value,-limit,limit));return true;
      }
    return false;
  }
  auto& shape=out.poses[r][h][c].finger[f];
  for(int j=0;j<3;++j) if(axis==std::string("bend")+char('1'+j)) {
    shape.bend[j]=static_cast<float>(std::clamp(value,-30.,120.));return true;
  }
  if(!std::strcmp(axis,"splay")) {shape.splay=static_cast<float>(std::clamp(value,-85.,85.));return true;}
  if(!std::strcmp(axis,"roll")) {shape.roll=static_cast<float>(std::clamp(value,-180.,180.));return true;}
  for(int i=0;i<extra_count;++i) if(!std::strcmp(axis,extra_name(i))) {
    const double limit=i<3?90.:i<6?20.:i<9?60.:180.;
    shape.extra[i]=static_cast<float>(std::clamp(value,-limit,limit));return true;
  }
  return false;
}
inline Tuning parse_gesture_tune(const std::string& text) {
  Tuning out; std::istringstream lines(text);std::string line;
  while(std::getline(lines,line)) {
    if(line.empty() || line[0]=='#') continue;
    const auto at=line.find('=');if(at==std::string::npos) continue;
    std::string key=line.substr(0,at); const auto last=key.find_last_not_of(" \t\r");if(last==std::string::npos) continue;
    key.erase(last+1);key.erase(0,key.find_first_not_of(" \t"));
    double value=0;char tail=0;
    if(std::sscanf(line.c_str()+at+1," %lf %c",&value,&tail)==1) tune_key(out,key,value);
  }
  return out;
}
inline Shape mix_shape(const float (&mix)[combination_count],const Tuning& t,int context,int hand) {
  Shape out;
  for(int c=0;c<combination_count;++c) {
   for(int f=0;f<4;++f) {
    const auto& src=t.poses[context][hand][c].finger[f];auto& dst=out.finger[f];
    for(int j=0;j<3;++j) dst.bend[j]+=mix[c]*src.bend[j];
    dst.splay+=mix[c]*src.splay;dst.roll+=mix[c]*src.roll;
    for(int i=0;i<extra_count;++i) dst.extra[i]+=mix[c]*src.extra[i];
   }
   for(int b=0;b<thumb_helper_count;++b) for(int a=0;a<helper_axis_count;++a)
     out.helper[b][a]+=mix[c]*t.poses[context][hand][c].helper[b][a];
  }
  return out;
}
} // namespace hand_gestures
