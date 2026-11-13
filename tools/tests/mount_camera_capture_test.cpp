#include "../../src/camera/mount_camera_capture.hpp"
#include <cstdio>

using namespace mount_camera::capture;
int main() {
  // Missing settings enable the requested preference, but never arm diagnostics.
  if (!parse_settings("").first_person || parse_settings("").auto_capture) return 1;
  if (parse_settings("first_person_mounts=0\r\nauto_capture=1").first_person ||
      !parse_settings("first_person_mounts=0\r\nauto_capture=1").auto_capture) return 2;
  if (!parse_settings("first_person_mounts=0 first_person_mounts=1").first_person) return 3;
  if (!parse_settings("xfirst_person_mounts=0 first_person_mounts=00 first_person_mounts=garbage").first_person) return 4;
  if (parse_settings("xauto_capture=1 auto_capture=10").auto_capture) return 5;

  int state = -1;
  const unsigned char leaf8[]{0xB0,7,0xC3}, leaf32[]{0xB8,8,0,0,0,0xC3}, zero[]{0x33,0xC0,0xC3};
  const unsigned char jump[]{0xE9,7,0xC3}, writes[]{0xB0,7,0x90,0xC3}, wide[]{0xB8,8,1,0,0,0xC3};
  if (!constant_state(leaf8,sizeof(leaf8),state) || state!=7) return 6;
  if (!constant_state(leaf32,sizeof(leaf32),state) || state!=8) return 7;
  if (!constant_state(zero,sizeof(zero),state) || state!=0) return 8;
  if (constant_state(jump,sizeof(jump),state) || constant_state(writes,sizeof(writes),state) ||
      constant_state(wide,sizeof(wide),state)) return 9;
  for (unsigned n=0;n<sizeof(leaf8);++n) if (constant_state(leaf8,n,state)) return 10;
  for (unsigned n=0;n<sizeof(leaf32);++n) if (constant_state(leaf32,n,state)) return 11;

  Budget b;
  if (b.take(100,false)) return 12;
  b.start(100);
  if (!b.take(100,false) || !b.take(100,true)) return 13;
  if (b.take(101,false) || b.take(101,true)) return 14;
  if (!b.take(133,false) || !b.take(133,true) || b.samples!=4) return 15;
  if (b.take(99,false) || b.active) return 16; // backwards clock fails closed
  b.start(500);
  if (b.take(500+duration_ms,true) || b.active) return 17;
  b.start(500); b.samples=max_camera_samples;
  if (b.take(500,false) || b.active) return 18;
  b.start(900);
  if (!b.take(900,true) || b.samples!=1) return 19; // restart clears old window
  b.active=false;
  if (b.take(1000,true)) return 20;
  WeaponBudget w;
  b.start(1000); w.identity(1,2,3,4);
  const WeaponBudget::Key entity{10,0,0,1},palette{10,20,30,2};
  if(!w.take(b,1000,entity) || w.take(b,1249,entity) || !w.take(b,1250,entity)) return 21;
  if(!w.take(b,1250,palette) || w.take(b,1251,palette)) return 22;
  if(!w.take(b,1251,{10,21,30,2}) || !w.take(b,1251,{10,21,31,2})) return 23;
  // No pointer eviction on a flood. Existing entries continue to sample.
  for(unsigned i=w.count;i<max_weapon_sources;++i) if(!w.take(b,1300,{100+i,0,0,1})) return 24;
  if(w.take(b,1500,{999,0,0,1}) || w.capacity_drops!=1 || !w.take(b,1500,entity)) return 25;
  if(w.take(b,1499,entity)) return 26;
  w.identity(1,2,3,4); if(w.count!=max_weapon_sources) return 27;
  w.identity(1,2,3,5); if(w.count || !w.take(b,1501,entity)) return 28;
  w.identity(2,2,3,5); if(w.count) return 29;
  w.identity(2,0,0,0); if(w.count || !w.take(b,1501,entity)) return 30;
  b.active=false; if(w.take(b,1800,entity)) return 31; // cancelled capture
  b.start(2000); w=WeaponBudget{};
  if(w.take(b,2000+duration_ms,entity) || w.take(b,1999,entity)) return 32;
  if(age(2000,0)!=UINT64_MAX || age(2000,2001)!=UINT64_MAX || age(2000,1900)!=100) return 33;
  b.start(100); w=WeaponBudget{}; unsigned records=0;
  for(std::uint64_t now=100;now<100+duration_ms;++now) if(w.take(b,now,entity)) ++records;
  if(records!=duration_ms/weapon_sample_ms || w.take(b,100+duration_ms,entity)) return 34;
  // Regression: songcord/world mesh saturation must not consume the weapon discovery pool.
  if(!admit_palette(true,max_meshes,0,0.f) || admit_palette(true,max_meshes,0,1000000.f)) return 35;
  if(admit_palette(false,max_meshes,0,0.f) || admit_palette(true,0,max_weapon_meshes,0.f)) return 36;
  if(!admit_palette(false,0,max_weapon_meshes,1.f) || admit_palette(false,0,0,65.f)) return 37;
  if(admit_palette(true,0,0,NAN) || admit_palette(true,0,0,INFINITY) || admit_palette(true,0,0,-1.f)) return 38;
  b.start(1000); w=WeaponBudget{};
  const WeaponBudget::Key hands{1234,0,0,3}; // one paired sample per body resource, independent of weapon rows
  if(!w.take(b,1000,hands) || w.take(b,1249,hands) || !w.take(b,1250,hands) || !w.take(b,1250,entity)) return 39;
  std::puts("weapon capture policy: PASS (4 Hz, identities, capacity, cancellation, stale/missing ages)");
  std::puts("mount capture policy: PASS (settings, leaf-only decoding, phase cadence, bounds, restart)");
  return 0;
}
