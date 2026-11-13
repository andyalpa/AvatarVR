#pragma once
#include "mounted_weapon_math.hpp"

// Retarget named finger chains geometrically. Never copy FP bone axes or scale
// into the TP mesh: its own matrices/segment lengths supply the bind geometry.
namespace mounted_hand_pose {
constexpr int count=19;
constexpr const char* names[count]={"fingerThumb_01","fingerThumb_02","fingerThumb_03","fingerThumb_Target",
  "fingerIndexBase_01","fingerIndex_01","fingerIndex_02","fingerIndex_03","fingerIndex_Target",
  "fingerMiddleBase_01","fingerMiddle_01","fingerMiddle_02","fingerMiddle_03","fingerMiddle_Target",
  "fingerPinkyBase_01","fingerPinky_01","fingerPinky_02","fingerPinky_03","fingerPinky_Target"};
constexpr int parent[count]={-1,0,1,2,-1,4,5,6,7,-1,9,10,11,12,-1,14,15,16,17};
constexpr int child[count]={1,2,3,-1,5,6,7,8,-1,10,11,12,13,-1,15,16,17,18,-1};
inline int index(const char* name) {
  if(!name || (std::strncmp(name,"L_",2) && std::strncmp(name,"R_",2))) return -1;
  for(int i=0;i<count;++i) if(!std::strcmp(name+2,names[i])) return i;
  return -1;
}
inline int family(const char* name) {
  if(!name) return -1;
  if(std::strstr(name,"fingerThumb") || std::strstr(name,"thumb")) return 0;
  if(std::strstr(name,"fingerIndex")) return 1;
  if(std::strstr(name,"fingerMiddle")) return 2;
  if(std::strstr(name,"fingerPinky")) return 3;
  return -1;
}
inline int family(int node) {return node<4?0:node<9?1:node<14?2:3;}
struct Pose {skin_arm_ik::Vec point[count]{}; bool valid=false;};
inline bool retarget(const float (&reference)[count][16],const Pose& pose,bool mirror,float (&out)[count][16]) {
  using namespace skin_arm_ik;
  if(!pose.valid) return false;
  Vec points[count]; float next[count][16];
  for(int i=0;i<count;++i) {
    if(!mounted_weapon_math::rigid(reference[i])) return false;
    const int p=parent[i];
    Vec direction=p<0?pose.point[i]:sub(pose.point[i],pose.point[p]);
    if(mirror) direction.x=-direction.x; // anatomical normal is the mirrored axial coordinate
    const Vec old=p<0?position(reference[i]):sub(position(reference[i]),position(reference[p]));
    const float from=length(old),to=length(direction);
    if(!finite(direction) || !(to>1e-6f) || !(from>1e-6f)) return false;
    points[i]=add(p<0?Vec{}:points[p],scale(direction,from/to));
  }
  for(int i=0;i<count;++i) {
    const int c=child[i],p=parent[i];
    const Vec from=c>=0?sub(position(reference[c]),position(reference[i])):
      sub(position(reference[i]),position(reference[p]));
    const Vec to=c>=0?sub(points[c],points[i]):sub(points[i],points[p]);
    matrix(next[i],reference[i],from,to,points[i]);
    if(!mounted_weapon_math::rigid(next[i])) return false;
  }
  std::memcpy(out,next,sizeof(next)); return true;
}
}
