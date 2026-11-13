#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <string_view>

// Portable capture policy. No engine types, calls, or camera writes.
namespace mount_camera::capture {
constexpr std::uint64_t duration_ms = 180000;
constexpr std::uint64_t sample_ms = 33;
constexpr unsigned max_camera_samples = 12000;
constexpr unsigned max_meshes = 128;
constexpr unsigned max_weapon_meshes = 32;
constexpr unsigned max_palette_resources = 128;
// Read-only discovery admission. Player weapon paths have reserved slots; distance is never
// evidence of equipped ownership. Preserve the existing eight-metre observation neighbourhood.
inline bool admit_palette(bool weapon,unsigned general_count,unsigned weapon_count,float distance2) {
  if(!std::isfinite(distance2) || distance2<0.f || distance2>64.f) return false;
  return weapon ? weapon_count<max_weapon_meshes : general_count<max_meshes;
}
constexpr std::uint64_t weapon_sample_ms = 250;
constexpr unsigned max_weapon_sources = 64;

// Observer-only throttle. Resource/source identity is part of the key: recycled mesh pointers cannot
// inherit another resource's cadence. No eviction within a capture; floods fail closed at the bound.
struct Budget;
struct WeaponBudget {
  struct Key { std::uintptr_t object=0, resource=0, source=0; unsigned kind=0; };
  struct Entry { Key key; std::uint64_t last=0; };
  Entry entries[max_weapon_sources]{};
  unsigned count=0,capacity_drops=0;
  std::uintptr_t player=0, partner=0;
  std::uint64_t mount[2]{};
  bool identified=false;
  void identity(std::uintptr_t p,std::uintptr_t m,std::uint64_t a,std::uint64_t b) {
    if(!identified || player!=p || partner!=m || mount[0]!=a || mount[1]!=b) {
      count=0; player=p; partner=m; mount[0]=a; mount[1]=b; identified=true;
    }
  }
  bool take(const Budget& budget,std::uint64_t now,Key key);
};

struct Settings {
  bool first_person = true;
  bool mounted_arms = true;  // stage B: arms follow the controllers on the riding body
  bool auto_capture = false;
};

inline Settings parse_settings(std::string_view text) {
  Settings out;
  while (!text.empty()) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == text.npos) break;
    text.remove_prefix(begin);
    auto end = text.find_first_of(" \t\r\n");
    if (end == text.npos) end = text.size();
    const auto word = text.substr(0, end);
    if (word == "first_person_mounts=0") out.first_person = false;
    if (word == "first_person_mounts=1") out.first_person = true;
    if (word == "mounted_arms=0") out.mounted_arms = false;
    if (word == "mounted_arms=1") out.mounted_arms = true;
    if (word == "auto_capture=1") out.auto_capture = true;
    if (word == "auto_capture=0") out.auto_capture = false;
    text.remove_prefix(end);
  }
  return out;
}

// Only decode a complete, side-effect-free constant-return leaf. Never call an
// unknown controller virtual method, follow a jump, or scan executable pages.
inline bool constant_state(const unsigned char* code, std::size_t n, int& value) {
  if (n >= 3 && code[0] == 0xB0 && code[2] == 0xC3) {
    value = code[1];
    return true;
  }
  if (n >= 6 && code[0] == 0xB8 && code[5] == 0xC3 &&
      code[2] == 0 && code[3] == 0 && code[4] == 0) {
    value = code[1];
    return true;
  }
  if (n >= 3 && code[0] == 0x33 && code[1] == 0xC0 && code[2] == 0xC3) {
    value = 0;
    return true;
  }
  return false;
}

struct Budget {
  bool active = false;
  std::uint64_t started = 0;
  std::uint64_t next[2]{};
  unsigned samples = 0;

  void start(std::uint64_t now) {
    active = true;
    started = now;
    next[0] = next[1] = now;
    samples = 0;
  }
  bool expired(std::uint64_t now) const {
    return !active || now < started || now - started >= duration_ms || samples >= max_camera_samples;
  }
  bool take(std::uint64_t now, bool post) {
    if (expired(now)) { active = false; return false; }
    const unsigned phase = post ? 1u : 0u;
    if (now < next[phase]) return false;
    next[phase] = now + sample_ms;
    ++samples;
    return true;
  }
};
inline bool WeaponBudget::take(const Budget& budget,std::uint64_t now,Key key) {
  if(budget.expired(now)) return false;
  for(unsigned i=0;i<count;++i) {
    auto& e=entries[i];
    if(e.key.object==key.object && e.key.resource==key.resource && e.key.source==key.source && e.key.kind==key.kind) {
      if(now<e.last || now-e.last<weapon_sample_ms) return false;
      e.last=now; return true;
    }
  }
  if(count==max_weapon_sources) {++capacity_drops; return false;}
  entries[count++]={key,now}; return true;
}
// Explicit missing/backwards sentinel; zero age must never mean 'not observed'.
inline std::uint64_t age(std::uint64_t now,std::uint64_t stamp) {
  return stamp && now>=stamp ? now-stamp : UINT64_MAX;
}
}  // namespace mount_camera::capture
