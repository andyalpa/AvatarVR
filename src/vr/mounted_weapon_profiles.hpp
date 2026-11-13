#pragma once
#include "mounted_hand_pose.hpp"
#include <map>
#include <sstream>
#include <iomanip>
#include <limits>
#include <locale>

namespace mounted_weapon_profiles {
struct Profile {
  float relative[16]{},trigger_root[16]{},fp_root_socket[16]{};
  mounted_hand_pose::Pose fingers[2];
  skin_arm_ik::Vec muzzle{},forward{};
  bool valid=false,aim_valid=false;
};
using Cache=std::map<std::string,Profile>;
constexpr std::size_t max_profiles=32,max_bytes=256*1024;
inline bool path_valid(const std::string& path) {
  if(path.size()>127 || path.find("/WEAPON/PLAYER/")==std::string::npos || path.find("..")!=std::string::npos ||
     path.size()<4 || path.substr(path.size()-4)!=".MMB") return false;
  for(unsigned char c:path) if(!((c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='/'||c=='_'||c=='.'||c=='-')) return false;
  return true;
}
inline bool matrix_valid(const float* m,float radius) {
  return mounted_weapon_math::rigid(m) && std::fabs(m[3])+std::fabs(m[7])+std::fabs(m[11])+std::fabs(m[15]-1.f)<1e-4f &&
    skin_arm_ik::length(skin_arm_ik::position(m))<radius;
}
inline bool valid(const Profile& p) {
  using namespace skin_arm_ik;
  if(!p.valid || !matrix_valid(p.relative,1.2f) || !matrix_valid(p.trigger_root,.6f) || !matrix_valid(p.fp_root_socket,.6f)) return false;
  for(const auto& pose:p.fingers) for(auto point:pose.point) if(!finite(point) || length(point)>.6f) return false;
  if(!finite(p.muzzle)||!finite(p.forward)) return false;
  return !p.aim_valid || (length(p.muzzle)<1.5f && std::fabs(length(p.forward)-1.f)<.01f);
}
inline std::uint32_t checksum(const std::string& text) {
  std::uint32_t n=2166136261u; for(unsigned char c:text) {n^=c; n*=16777619u;} return n;
}
inline std::string encode(const Cache& cache) {
  if(cache.size()>max_profiles) return {};
  std::ostringstream s; s.imbue(std::locale::classic()); s<<std::setprecision(std::numeric_limits<float>::max_digits10);
  s<<"AVR_MOUNT_HOLDS 2 98588DA2 "<<cache.size()<<'\n';
  for(const auto& item:cache) {
    const auto& p=item.second; if(!path_valid(item.first)||!valid(p)) return {};
    s<<item.first;
    for(const float* m:{p.relative,p.trigger_root,p.fp_root_socket}) for(int k=0;k<16;++k) s<<' '<<m[k];
    for(const auto& pose:p.fingers) {s<<' '<<int(pose.valid); for(auto q:pose.point) s<<' '<<q.x<<' '<<q.y<<' '<<q.z;}
    s<<' '<<int(p.aim_valid)<<' '<<p.muzzle.x<<' '<<p.muzzle.y<<' '<<p.muzzle.z<<' '<<p.forward.x<<' '<<p.forward.y<<' '<<p.forward.z<<'\n';
  }
  const auto body=s.str(); return body+"CHECKSUM "+std::to_string(checksum(body))+"\n";
}
inline bool decode(const std::string& text,Cache& out) {
  if(text.size()>max_bytes) return false;
  const auto end=text.rfind("CHECKSUM "); if(end==std::string::npos) return false;
  std::istringstream check(text.substr(end+9)); check.imbue(std::locale::classic());
  std::uint32_t expected=0; std::string extra; if(!(check>>expected) || check>>extra || expected!=checksum(text.substr(0,end))) return false;
  std::istringstream s(text.substr(0,end)); s.imbue(std::locale::classic());
  std::string magic,build; unsigned version=0,count=0;
  if(!(s>>magic>>version>>build>>count)||magic!="AVR_MOUNT_HOLDS"||version!=2||build!="98588DA2"||count>max_profiles) return false;
  Cache next;
  for(unsigned i=0;i<count;++i) {
    std::string path; Profile p; int bit=0;
    if(!(s>>path)||!path_valid(path)||next.count(path)) return false;
    for(float* m:{p.relative,p.trigger_root,p.fp_root_socket}) for(int k=0;k<16;++k) if(!(s>>m[k])) return false;
    for(auto& pose:p.fingers) {
      if(!(s>>bit)||(bit!=0&&bit!=1)) return false; pose.valid=bit!=0;
      for(auto& q:pose.point) if(!(s>>q.x>>q.y>>q.z)) return false;
    }
    if(!(s>>bit)||(bit!=0&&bit!=1)) return false; p.aim_valid=bit!=0;
    if(!(s>>p.muzzle.x>>p.muzzle.y>>p.muzzle.z>>p.forward.x>>p.forward.y>>p.forward.z)) return false;
    p.valid=true; if(!valid(p)) return false; next.emplace(path,p);
  }
  if(s>>extra) return false;
  out.swap(next); return true;
}
// Equip/reload animation must settle in relative grip space, not just world space.
inline bool same_hold(const Profile& a,const Profile& b) {
  using namespace skin_arm_ik;
  auto settled=[](const float* x,const float* y) {
    float trace=0.f; for(int k=0;k<3;++k) trace+=dot(row(x,k),row(y,k));
    return mounted_weapon_math::same_frame(x,y,.003f) && trace>2.999695f; // one degree
  };
  if(!valid(a)||!valid(b)||!settled(a.trigger_root,b.trigger_root)) return false;
  for(int h=0;h<2;++h) {
    if(a.fingers[h].valid!=b.fingers[h].valid) return false;
    if(a.fingers[h].valid) for(int i=0;i<mounted_hand_pose::count;++i)
      if(length(sub(a.fingers[h].point[i],b.fingers[h].point[i]))>.003f) return false;
  }
  return settled(a.relative,b.relative) && settled(a.fp_root_socket,b.fp_root_socket);
}
// Native hold stability is measured in grip space, not from controller motion.
// The caller serializes this collector across palette workers for one resource.
struct StableHold {
  const void* owner=nullptr; const void* weapon=nullptr;
  Profile profile;
  std::uint64_t since=0,last=0;
  bool observe(const void* player,const void* equipped,const Profile& next,std::uint64_t now) {
    if(!player || !equipped || !now || !valid(next)) { *this=StableHold{}; return false; }
    if(owner!=player || weapon!=equipped || !last || now<last || now-last>100 || !same_hold(profile,next)) {
      owner=player; weapon=equipped; profile=next; since=last=now; return false;
    }
    last=now; return now-since>=600;
  }
};
}
