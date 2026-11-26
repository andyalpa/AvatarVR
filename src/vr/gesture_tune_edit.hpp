#pragma once
// [AUTHOR 2026-11-17] Finger-pose editing from the in-VR menu (Hands tab). Reads and writes the same
// AvatarVR_ik_tune.txt as tools/ik_tuner (keys, ranges, defaults and file layout match gesture_tuning.py),
// so either editor can be used; never run both at once, the desktop tuner rewrites the file from its own state.
#include "hand_gesture_tune.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace gesture_edit {
using hand_gestures::combination_count;
constexpr int axis_count=17;
// Same order as gesture_tuning.py AXES.
inline const char* axis_key(int a) {
  static const char* keys[axis_count]={"root_pitch","root_yaw","root_roll","root_normal","root_side","root_forward",
    "bend1","spread1","roll1","bend2","spread2","roll2","bend3","spread3","roll3","splay","roll"};
  return a>=0 && a<axis_count?keys[a]:"";
}
inline const char* axis_label(int a) {
  static const char* labels[axis_count]={"Base pitch","Base oppose","Base roll","Base normal (mm)","Base sideways (mm)",
    "Base forward (mm)","01 bend","01 sideways","01 skin roll","02 bend","02 sideways","02 skin roll","03 bend",
    "03 sideways","03 skin roll","All spread","All skin roll"};
  return a>=0 && a<axis_count?labels[a]:"";
}
inline bool axis_is_position(int a) {return a>=3 && a<=5;}
// Same limits as tune_key() and gesture_tuning.py bounds().
inline void axis_range(int a,float& lo,float& hi,float& step) {
  const std::string k=axis_key(a);
  step=axis_is_position(a)?.25f:1.f;
  if(!k.compare(0,4,"bend")) {lo=-30;hi=120;}
  else if(axis_is_position(a)) {lo=-20;hi=20;}
  else if(!k.compare(0,5,"root_")) {lo=-90;hi=90;}
  else if(!k.compare(0,6,"spread")) {lo=-60;hi=60;}
  else if(k=="splay") {lo=-85;hi=85;}
  else {lo=-180;hi=180;}
}
inline const char* finger_key(int f) {static const char* k[4]={"thumb","index","middle","pinky"};return f>=0 && f<4?k[f]:"";}
inline float default_value(int combo,int finger,int a) {
  const auto s=hand_gestures::default_shape(combo).finger[finger];
  const std::string k=axis_key(a);
  if(k=="bend1") return s.bend[0];
  if(k=="bend2") return s.bend[1];
  if(k=="bend3") return s.bend[2];
  if(k=="splay") return s.splay;
  if(k=="roll") return s.roll;
  for(int i=0;i<hand_gestures::extra_count;++i) if(k==hand_gestures::extra_name(i)) return s.extra[i];
  return 0.f;
}
inline std::string key(int rig,int hand,int combo,int finger,int a) {
  return std::string("gesture.")+(rig?"tp":"fp")+"."+(hand?"right":"left")+"."+hand_gestures::combination_name(combo)+"."+
    finger_key(finger)+"."+axis_key(a);
}
inline std::string pose_prefix(int rig,int hand,int combo) {
  return std::string("gesture.")+(rig?"tp":"fp")+"."+(hand?"right":"left")+"."+hand_gestures::combination_name(combo)+".";
}

// Bones: 0-3 fingers (17 axes), 4-17 thumb webbing/deformation helpers (9 local axes each), as in the tuner.
constexpr int bone_count=4+hand_gestures::thumb_helper_count;
inline bool is_helper(int b) {return b>=4;}
inline int bone_axis_count(int b) {return is_helper(b)?hand_gestures::helper_axis_count:axis_count;}
inline std::string bone_name(int b) {return is_helper(b)?hand_gestures::helper_name(b-4):finger_key(b);}
inline const char* bone_axis_key(int b,int a) {return is_helper(b)?hand_gestures::helper_axis_name(a):axis_key(a);}
inline const char* bone_axis_label(int b,int a) {
  static const char* helper[hand_gestures::helper_axis_count]={"Rotate X","Rotate Y","Rotate Z","Offset X (mm)","Offset Y (mm)",
    "Offset Z (mm)","Scale X (%)","Scale Y (%)","Scale Z (%)"};
  return is_helper(b)?helper[a]:axis_label(a);
}
inline void bone_axis_range(int b,int a,float& lo,float& hi,float& step) {
  if(!is_helper(b)) {axis_range(a,lo,hi,step);return;}
  const float limit=a<3?90.f:a<6?20.f:75.f; lo=-limit;hi=limit;step=a>=3 && a<6?.25f:1.f;
}
inline float bone_default(int combo,int b,int a) {return is_helper(b)?0.f:default_value(combo,b,a);}
inline std::string bone_key(int rig,int hand,int combo,int b,int a) {return pose_prefix(rig,hand,combo)+bone_name(b)+"."+bone_axis_key(b,a);}
// "bone.axis" (capture reply and clipboard) to indices.
inline bool bone_axis_of(const std::string& name,int& b,int& a) {
  for(b=0;b<bone_count;++b) for(a=0;a<bone_axis_count(b);++a) if(name==bone_name(b)+"."+bone_axis_key(b,a)) return true;
  return false;
}

// Numeric key=value lines, kept in the desktop tuner's layout: header, non-gesture keys first, sorted, CRLF.
struct File {
  std::map<std::string,double> values;
  void parse(const std::string& text) {
    values.clear();
    std::size_t at=0;
    while(at<text.size()) {
      std::size_t end=text.find('\n',at); if(end==std::string::npos) end=text.size();
      std::string line=text.substr(at,end-at); at=end+1;
      while(!line.empty() && (line.back()=='\r' || line.back()==' ' || line.back()=='\t')) line.pop_back();
      if(line.empty() || line[0]=='#') continue;
      const auto eq=line.find('='); if(eq==std::string::npos) continue;
      std::string k=line.substr(0,eq);
      while(!k.empty() && (k.back()==' ' || k.back()=='\t')) k.pop_back();
      const auto first=k.find_first_not_of(" \t"); if(first==std::string::npos) continue; k.erase(0,first);
      double v=0; char tail=0;
      if(std::sscanf(line.c_str()+eq+1," %lf %c",&v,&tail)==1 && std::isfinite(v)) values[k]=v;
    }
  }
  std::string text() const {
    std::vector<std::pair<std::string,double>> rows(values.begin(),values.end());
    std::stable_sort(rows.begin(),rows.end(),[](const auto& a,const auto& b) {
      const bool ga=!a.first.compare(0,8,"gesture."),gb=!b.first.compare(0,8,"gesture.");
      return ga!=gb?gb:a.first<b.first;});
    std::string out="# AvatarVR live arm + finger tuning; angles in degrees. Preview expires unless the tuner renews it.\r\n";
    char line[256];
    for(const auto& r:rows) {std::snprintf(line,sizeof(line),"%s=%.17g\r\n",r.first.c_str(),r.second);out+=line;}
    return out;
  }
  // A value equal to the default is removed, as the desktop tuner does; absent keys mean the default.
  float get(int rig,int hand,int combo,int finger,int a) const {
    const auto it=values.find(key(rig,hand,combo,finger,a));
    return it!=values.end()?static_cast<float>(it->second):default_value(combo,finger,a);
  }
  void set(int rig,int hand,int combo,int finger,int a,float v) {
    float lo,hi,step; axis_range(a,lo,hi,step);
    if(!std::isfinite(v)) return;
    v=std::clamp(v,lo,hi);
    const std::string k=key(rig,hand,combo,finger,a);
    if(std::fabs(v-default_value(combo,finger,a))<1e-6f) values.erase(k); else values[k]=v;
  }
  float get_bone(int rig,int hand,int combo,int b,int a) const {
    const auto it=values.find(bone_key(rig,hand,combo,b,a));
    return it!=values.end()?static_cast<float>(it->second):bone_default(combo,b,a);
  }
  void set_bone(int rig,int hand,int combo,int b,int a,float v) {
    float lo,hi,step; bone_axis_range(b,a,lo,hi,step);
    if(!std::isfinite(v)) return;
    v=std::clamp(v,lo,hi);
    const std::string k=bone_key(rig,hand,combo,b,a);
    if(std::fabs(v-bone_default(combo,b,a))<1e-6f) values.erase(k); else values[k]=v;
  }
  void reset_pose(int rig,int hand,int combo) {
    const std::string p=pose_prefix(rig,hand,combo);
    for(auto it=values.lower_bound(p);it!=values.end() && !it->first.compare(0,p.size(),p);) it=values.erase(it);
  }
  // Whole combination (fingers and thumb helpers) onto another hand/rig, like the tuner's copy buttons.
  void copy_pose(int rig,int hand,int combo,int to_rig,int to_hand) {
    if(rig==to_rig && hand==to_hand) return;
    const std::string from=pose_prefix(rig,hand,combo);
    std::map<std::string,double> copied;
    for(auto it=values.lower_bound(from);it!=values.end() && !it->first.compare(0,from.size(),from);++it)
      copied[it->first.substr(from.size())]=it->second;
    reset_pose(to_rig,to_hand,combo);
    const std::string to=pose_prefix(to_rig,to_hand,combo);
    for(const auto& c:copied) values[to+c.first]=c.second;
  }
  void preview(bool on,int rig,int hand,int combo,double until) {
    values["gesture.preview_until"]=on?until:0.;
    values["gesture.preview_context"]=rig;
    values["gesture.preview_hand"]=hand;
    values["gesture.preview_combo"]=combo;
  }
};
} // namespace gesture_edit
