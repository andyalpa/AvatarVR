// vr_weapon: the player's shot follows the tracked (IK) gun. Single choke point, replaces the muzzle probes (2026-10-15).
//
// Evidence (research/WEAPONS_AND_HANDS.md, 2026-10-15 entries):
// [STATIC] 0x14445C570(weapon, start, target, third, a5..a9), called every update from 0x14446C934 for each weapon:
//   weapon+0x5254 = *start (the shot origin), weapon+0x3E90 = *target (the default aim point), weapon+0x3E9C = *third;
//   it publishes the origin to *(weapon+0x3F20)+0x68 (dirty flag +0x60) for other readers.
// [STATIC] 0x143F8EBC0 (the player weapon ray) reads origin weapon+0x5254 and aims at weapon+0x3E90, unless the aim
//   object at owner+0x1A8 -> +0x60 is valid (then its point). Hits are applied from that origin.
// [LIVE] start = camera + 0.55 m (constant), ~0.33 m from the flat right weapon socket; target ~1.9 m along the aim.
// [LIVE/HEADSET] writing weapon+0x5254 only around 0x143F8EBC0 (then restoring it) did not move the tracer or smoke.
//
// With "Shoot from tracked gun (experimental)" on (and Script writes on), the producer is given copies of start and
// target carried rigidly from the flat weapon socket to the IK weapon socket (vr_weapon_math::map_shot): the stored and
// published origin and the default aim point become the tracked gun's muzzle and barrel direction for every reader.
// The caller's buffers are never modified; the producer runs exactly once with the original remaining arguments.
// Off (default, not persisted): read-only logging of start/target/third and the aim-object state, 1 line per second.
// Log: scripts\VR_WEAPON.log. Rollback: toggle off, or the previous DLL.
#include "../runtime_log.hpp"
#include "vr_weapon.hpp"
#include "vr_weapon_math.hpp"
#include "vr_arms_math.hpp"
#include "vr_recoil.hpp"
#include "bow_draw.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../camera/mount_camera.hpp"
#include "../game/player.hpp"
#include "../script/lua_host.hpp"
#include <windows.h>
#include <MinHook.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace {
constexpr std::uintptr_t kProducerRva=0x445C570;
constexpr unsigned char kProducerSig[]={0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x48,0x89,0x7C,0x24,0x18,
                                        0x41,0x56,0x48,0x83,0xEC,0x30};
using Producer=std::uintptr_t(__fastcall*)(void*,void*,void*,void*,void*,std::uintptr_t,void*,std::uintptr_t,void*);
Producer original=nullptr;
bool enabled=true;                        // menu "Shoot from tracked gun"; on by default (user request 2026-10-15)
std::atomic<bool> write_permission{false}; // enabled && Script writes, refreshed every Present
bool tried=false;
std::atomic<unsigned> calls{0},shifted{0},rejected{0},lines{0};
std::atomic<ULONGLONG> next_line{0};
std::wstring dir;

void log(const char* text) {
  if(dir.empty()) {
    HMODULE self=nullptr; wchar_t path[MAX_PATH]{};
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&log),&self);
    if(!GetModuleFileNameW(self,path,MAX_PATH)) return;
    dir=path; dir=dir.substr(0,dir.find_last_of(L"\\/"));
  }
  runtime_log::append(dir+L"\\VR_WEAPON.log", {text, "\r\n"});
}
const unsigned char* ptr(const void* base,unsigned offset) {
  const unsigned char* value=nullptr;
  if(base) game::mem_read(static_cast<const unsigned char*>(base)+offset,&value,sizeof(value));
  return value;
}
float dist(const float* a,const float* b) {
  const float x=a[0]-b[0],y=a[1]-b[1],z=a[2]-b[2]; return std::sqrt(x*x+y*y+z*z);
}

// ---- Stored copies of the unmoved muzzle point (2026-10-15, run 7) ----
// [HEADSET] the flash still appears ~camera + look*0.55 (the unmoved start S) although weapon+0x5254/+0x5248 hold the
// tracked muzzle. Discriminator: after the producer ran with the shifted start, search the weapon object, the player
// body and every object they point to for float triples within 1 cm of S. A hit is another stored copy the effects may
// read; no hit means the effects recompute camera + offset themselves. Read-only, 1 scan/s, 120 hit lines max.
std::atomic<ULONGLONG> scan_next{0};
std::atomic<unsigned> scan_lines{0};
bool user_ptr(std::uintptr_t v) { return v>=0x10000 && v<0x7FFFFFFFFFFFull && (v&7)==0; }
unsigned scan_block(const unsigned char* base,unsigned size,const float* target,const char* path,unsigned budget) {
  static thread_local unsigned char buf[0x6000];
  if(!base || size>sizeof(buf) || !game::mem_read(base,buf,size)) return 0;
  unsigned found=0;
  for(unsigned off=0;off+12<=size && found<budget;off+=4) {
    const float* v=reinterpret_cast<const float*>(buf+off);
    if(std::fabs(v[0]-target[0])<0.01f && std::fabs(v[1]-target[1])<0.01f && std::fabs(v[2]-target[2])<0.01f) {
      ++found;
      if(scan_lines.fetch_add(1)<120) {
        char line[200]; std::snprintf(line,sizeof(line),"  copy of unmoved S: %s(%p)+0x%X = (%.3f %.3f %.3f)",path,base,off,v[0],v[1],v[2]);
        log(line);
      }
    }
  }
  return found;
}
// Run 8: the copy scan found no stored copy of S (only weapon+0x5248 before our B write). The three untracked items (flash,
// visible bullet origin, muzzle ring) all sit at the flat gun's muzzle -> search for the flat weapon's own transform:
// float triples within 2 cm of the flat right socket or the flat weapon C_root (world), in the weapon object, the body and
// two levels of their pointees. A match whose preceding 3 rows are unit vectors is logged as a matrix.
unsigned scan_targets(const unsigned char* base,unsigned size,const float (*t)[3],unsigned nt,const char* path,unsigned budget) {
  static thread_local unsigned char buf[0x6000];
  if(!base || size>sizeof(buf) || !game::mem_read(base,buf,size)) return 0;
  unsigned found=0;
  for(unsigned off=0;off+12<=size && found<budget;off+=4) {
    const float* v=reinterpret_cast<const float*>(buf+off);
    for(unsigned k=0;k<nt;++k) if(std::fabs(v[0]-t[k][0])<0.02f && std::fabs(v[1]-t[k][1])<0.02f && std::fabs(v[2]-t[k][2])<0.02f) {
      ++found;
      bool matrix=off>=48;
      for(int r=0;matrix && r<3;++r) {const float* row=reinterpret_cast<const float*>(buf+off-48+16*r);
        const float l=std::sqrt(row[0]*row[0]+row[1]*row[1]+row[2]*row[2]); matrix=std::fabs(l-1.f)<0.02f;}
      if(scan_lines.fetch_add(1)<160) {
        char line[220]; std::snprintf(line,sizeof(line),"  flat %s: %s(%p)+0x%X = (%.3f %.3f %.3f)%s",k?"weapon root":"socket",path,base,off,
          v[0],v[1],v[2],matrix?" [4x4 row 3]":"");
        log(line);
      }
      break;
    }
  }
  return found;
}
void scan_for_weapon(const unsigned char* weapon,const unsigned char* body) {
  float flat[16],ik[16],rootm[16]; float t[2][3]; unsigned nt=0;
  // Riding: who else holds the native socket? [LIVE] weapon+0x3FB0 is at the origin, so it does not place the ring there. Read-only.
  const bool riding=mount_camera::rider_view();
  static bool riding_scan_started=false;
  if(riding && !riding_scan_started) {riding_scan_started=true; scan_lines.store(0);}
  if(riding?skin_palette_probe::mounted_placed_pair(flat,ik):skin_palette_probe::weapon_attach_matrices(skin_palette_probe::weapon_hand_index(),flat,ik)) {t[nt][0]=flat[12]; t[nt][1]=flat[13]; t[nt][2]=flat[14]; ++nt;}
  if(!riding && skin_palette_probe::held_weapon_root(rootm)) {t[nt][0]=rootm[12]; t[nt][1]=rootm[13]; t[nt][2]=rootm[14]; ++nt;}
  if(!nt) return;
  unsigned total=0;
  const unsigned char* roots[2]={weapon,body}; const unsigned sizes[2]={0x6000,0x2000}; const char* names[2]={"weapon","body"};
  for(int r=0;r<2;++r) {
    total+=scan_targets(roots[r],sizes[r],t,nt,names[r],8);
    std::uintptr_t ptrs[0x600];
    const unsigned count=std::min<unsigned>(sizes[r]/8,0x600);
    if(!game::mem_read(roots[r],ptrs,count*8)) continue;
    for(unsigned i=0;i<count;++i) {
      if(!user_ptr(ptrs[i])) continue;
      const unsigned char* child=reinterpret_cast<const unsigned char*>(ptrs[i]);
      char path[48]; std::snprintf(path,sizeof(path),"%s+0x%X>",names[r],i*8);
      total+=scan_targets(child,0x800,t,nt,path,4);
      std::uintptr_t p2[0x40];
      if(!game::mem_read(child,p2,sizeof(p2))) continue;
      for(unsigned j=0;j<0x40;++j) if(user_ptr(p2[j])) {
        char path2[64]; std::snprintf(path2,sizeof(path2),"%s+0x%X>+0x%X>",names[r],i*8,j*8);
        total+=scan_targets(reinterpret_cast<const unsigned char*>(p2[j]),0x200,t,nt,path2,2);
      }
    }
  }
  char line[200]; std::snprintf(line,sizeof(line),"scan for flat weapon transform: socket=(%.3f %.3f %.3f) targets=%u hits=%u",t[0][0],t[0][1],t[0][2],nt,total);
  log(line);
}
void scan_for_copies(const unsigned char* weapon,const unsigned char* body,const float* s) {
  unsigned total=0;
  const unsigned char* roots[2]={weapon,body}; const unsigned sizes[2]={0x6000,0x2000}; const char* names[2]={"weapon","body"};
  for(int r=0;r<2;++r) {
    total+=scan_block(roots[r],sizes[r],s,names[r],8);
    std::uintptr_t ptrs[0x600];
    const unsigned count=std::min<unsigned>(sizes[r]/8,0x600);
    if(!game::mem_read(roots[r],ptrs,count*8)) continue;
    for(unsigned i=0;i<count;++i) if(user_ptr(ptrs[i]) && reinterpret_cast<const unsigned char*>(ptrs[i])!=weapon) {
      char path[40]; std::snprintf(path,sizeof(path),"%s+0x%X>",names[r],i*8);
      total+=scan_block(reinterpret_cast<const unsigned char*>(ptrs[i]),0x400,s,path,4);
    }
  }
  char line[160]; std::snprintf(line,sizeof(line),"scan for unmoved S=(%.3f %.3f %.3f): %u copies (weapon, body, depth 1)",s[0],s[1],s[2],total);
  log(line);
}
// ---- Access watch on the weapon world matrix (2026-10-15, run 10) ----
// [LIVE/HEADSET run 9] moving weapon+0x3FB0 onto the IK socket persisted (d_ik ~0, ~60 writes/s) but the flash/ring
// did not move: either they read the matrix before our write each frame, or they use another source. A one-shot 3 s
// guard-page watch (PAGE_GUARD + single step re-arm, vectored handler; no game value is changed) records every
// instruction reading or writing weapon+0x3FB0..+0x3FEF, by RVA and access type. Armed by turning on "Weapon effects
// follow tracked gun" (once per launch). Removes itself when the window ends.
struct WatchHit { std::uintptr_t rva; unsigned reads,writes; };
WatchHit watch_hits[64];
std::atomic<unsigned> watch_hit_count{0};
std::atomic<bool> watch_active{false};
std::atomic<std::uintptr_t> watch_lo{0},watch_hi{0},watch_page{0};
std::atomic<ULONGLONG> watch_until{0};
std::atomic<unsigned> watch_faults{0},watch_other{0};
DWORD watch_protect=0;
thread_local bool watch_rearm=false;
void* watch_handler_handle=nullptr;
bool watch_done=true,watch_reported=false;  // retired after run 10 (set false to watch again)
std::atomic<long long> placement_last_us{0};
std::atomic<unsigned> placement_ticks{0};
// ---- Gun timing recorder (run 44, read-only) ----
struct TimeEvent { long long us; unsigned tid; char kind; unsigned rva; float x,y,z; };
TimeEvent timing_events[16384];
std::atomic<unsigned> timing_count{0};
std::atomic<bool> timing_active{false};
std::atomic<ULONGLONG> timing_until{0};
bool timing_started=false,timing_written=false;
std::atomic<ULONGLONG> timing_gun_since{0};
std::atomic<bool> timing_turning{false};
float timing_last_yaw=0.f; ULONGLONG timing_last_tick=0;  // Present thread
long long qpc_us() {static LARGE_INTEGER f{}; if(!f.QuadPart) QueryPerformanceFrequency(&f); LARGE_INTEGER n; QueryPerformanceCounter(&n);
  return static_cast<long long>(n.QuadPart/f.QuadPart*1000000+(n.QuadPart%f.QuadPart)*1000000/f.QuadPart);}
void timing_push(char kind,unsigned rva,const float* p) {
  const unsigned i=timing_count.fetch_add(1,std::memory_order_relaxed); if(i>=16384) return;
  TimeEvent& e=timing_events[i]; e.us=qpc_us(); e.tid=GetCurrentThreadId(); e.kind=kind; e.rva=rva;
  e.x=p?p[0]:0.f; e.y=p?p[1]:0.f; e.z=p?p[2]:0.f;
}
LONG CALLBACK watch_handler(EXCEPTION_POINTERS* e) {
  const DWORD code=e->ExceptionRecord->ExceptionCode;
  if(code==STATUS_GUARD_PAGE_VIOLATION && watch_active.load()) {
    const std::uintptr_t addr=e->ExceptionRecord->ExceptionInformation[1];
    const std::uintptr_t page=watch_page.load();
    if(addr<page || addr>=page+0x1000) return EXCEPTION_CONTINUE_SEARCH;  // not our page
    watch_faults.fetch_add(1,std::memory_order_relaxed);
    if(addr>=watch_lo.load() && addr<watch_hi.load()) {
      const std::uintptr_t base=reinterpret_cast<std::uintptr_t>(game::module_base());
      const std::uintptr_t rip=e->ContextRecord->Rip;
      const std::uintptr_t rva=(rip>=base && rip<base+0x16000000)?rip-base:0xFFFFFFFF;
      const bool write=e->ExceptionRecord->ExceptionInformation[0]==1;
      unsigned n=watch_hit_count.load();
      bool found=false;
      for(unsigned i=0;i<n && i<64;++i) if(watch_hits[i].rva==rva) {(write?watch_hits[i].writes:watch_hits[i].reads)++; found=true; break;}
      if(!found && n<64) {unsigned slot=watch_hit_count.fetch_add(1); if(slot<64) watch_hits[slot]={rva,write?0u:1u,write?1u:0u};}
      if(write && timing_active.load(std::memory_order_relaxed)) timing_push('W',static_cast<unsigned>(rva),nullptr);  // run 46: reads known (run 44), writes only
    } else watch_other.fetch_add(1,std::memory_order_relaxed);
    e->ContextRecord->EFlags|=0x100;  // single-step the access, then re-arm the guard
    watch_rearm=true;
    return EXCEPTION_CONTINUE_EXECUTION;
  }
  if(code==STATUS_SINGLE_STEP && watch_rearm) {
    watch_rearm=false;
    if(watch_active.load() && GetTickCount64()<watch_until.load()) {
      DWORD old=0; VirtualProtect(reinterpret_cast<void*>(watch_page.load()),0x1000,watch_protect|PAGE_GUARD,&old);
    }
    return EXCEPTION_CONTINUE_EXECUTION;
  }
  return EXCEPTION_CONTINUE_SEARCH;
}
void watch_start(unsigned char* weapon) {
  if(watch_done || !weapon) return;
  watch_done=true;
  const std::uintptr_t lo=reinterpret_cast<std::uintptr_t>(weapon)+0x3FB0;
  const std::uintptr_t page=lo&~static_cast<std::uintptr_t>(0xFFF);
  if(((lo+0x40-1)&~static_cast<std::uintptr_t>(0xFFF))!=page) {log("watch SKIP: matrix crosses a page"); return;}
  MEMORY_BASIC_INFORMATION mbi{};
  if(!VirtualQuery(reinterpret_cast<void*>(page),&mbi,sizeof(mbi)) || mbi.State!=MEM_COMMIT) {log("watch SKIP: query"); return;}
  watch_protect=mbi.Protect&~(PAGE_GUARD);
  watch_lo.store(lo); watch_hi.store(lo+0x40); watch_page.store(page);
  watch_handler_handle=AddVectoredExceptionHandler(1,watch_handler);
  if(!watch_handler_handle) {log("watch SKIP: handler"); return;}
  watch_until.store(GetTickCount64()+3000);
  watch_active.store(true);
  DWORD old=0;
  if(!VirtualProtect(reinterpret_cast<void*>(page),0x1000,watch_protect|PAGE_GUARD,&old)) {watch_active.store(false); log("watch SKIP: protect"); return;}
  char line[160]; std::snprintf(line,sizeof(line),"watch: guarding weapon+0x3FB0..+0x3FEF (page %p, protect 0x%lX) for 3 s",
    reinterpret_cast<void*>(page),watch_protect); log(line);
}
// Guard an arbitrary range (one page) for ms milliseconds; the hit report comes from watch_poll.
bool watch_range(std::uintptr_t lo,std::size_t len,ULONGLONG ms,const char* label) {
  if(watch_active.load()) return false;
  const std::uintptr_t page=lo&~static_cast<std::uintptr_t>(0xFFF);
  if(((lo+len-1)&~static_cast<std::uintptr_t>(0xFFF))!=page) {log("watch SKIP: range crosses a page"); return false;}
  MEMORY_BASIC_INFORMATION mbi{};
  if(!VirtualQuery(reinterpret_cast<void*>(page),&mbi,sizeof(mbi)) || mbi.State!=MEM_COMMIT) {log("watch SKIP: query"); return false;}
  watch_protect=mbi.Protect&~(PAGE_GUARD);
  watch_lo.store(lo); watch_hi.store(lo+len); watch_page.store(page);
  watch_hit_count.store(0); watch_faults.store(0); watch_other.store(0); watch_reported=false;
  if(!watch_handler_handle) watch_handler_handle=AddVectoredExceptionHandler(1,watch_handler);
  if(!watch_handler_handle) {log("watch SKIP: handler"); return false;}
  watch_until.store(GetTickCount64()+ms);
  watch_active.store(true);
  DWORD old=0;
  if(!VirtualProtect(reinterpret_cast<void*>(page),0x1000,watch_protect|PAGE_GUARD,&old)) {watch_active.store(false); log("watch SKIP: protect"); return false;}
  char line[200]; std::snprintf(line,sizeof(line),"watch: guarding %s %p..+0x%zX (page %p, protect 0x%lX) for %llu ms",label,
    reinterpret_cast<void*>(lo),len,reinterpret_cast<void*>(page),watch_protect,static_cast<unsigned long long>(ms)); log(line);
  return true;
}
void timing_poll() {
  {const game::Player pl=game::read(); const ULONGLONG t=GetTickCount64();  // turning detector: game look yaw rate
    if(pl.look_ok) {
      if(timing_last_tick && t>timing_last_tick) {float dy=pl.yaw-timing_last_yaw; while(dy>3.14159f) dy-=6.28318f; while(dy<-3.14159f) dy+=6.28318f;
        timing_turning.store(std::fabs(dy)*57.2958f*1000.f/static_cast<float>(t-timing_last_tick)>30.f);}
      timing_last_yaw=pl.yaw; timing_last_tick=t;}}
  if(timing_active.load()) timing_push('P',0,nullptr);
  if(!timing_active.load() || GetTickCount64()<timing_until.load() || timing_written) return;
  timing_active.store(false); timing_written=true;
  wchar_t module_path[MAX_PATH]{}; HMODULE self=nullptr;
  GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&timing_poll),&self);
  GetModuleFileNameW(self,module_path,MAX_PATH);
  std::wstring path(module_path); const auto slash=path.find_last_of(L"\\/"); path=path.substr(0,slash==std::wstring::npos?0:slash+1)+L"GUN_TIMING.csv";
  FILE* f=_wfopen(path.c_str(),L"w");
  const unsigned n=std::min(16384u,timing_count.load());
  if(f) {std::fputs("us,tid,kind,rva,x,y,z\n",f);
    for(unsigned i=0;i<n;++i) {const TimeEvent& e=timing_events[i];
      std::fprintf(f,"%lld,%u,%c,%X,%.4f,%.4f,%.4f\n",e.us,e.tid,e.kind,e.rva,e.x,e.y,e.z);}
    std::fclose(f);}
  char line[160]; std::snprintf(line,sizeof(line),"gun timing: wrote %u events to GUN_TIMING.csv (%s)",n,f?"ok":"open failed"); log(line);
}
void watch_poll() {
  if(!watch_active.load() || GetTickCount64()<watch_until.load() || watch_reported) return;
  watch_active.store(false);
  DWORD old=0; VirtualProtect(reinterpret_cast<void*>(watch_page.load()),0x1000,watch_protect,&old);  // drop any pending guard
  watch_reported=true;
  char line[200]; std::snprintf(line,sizeof(line),"watch report: faults=%u on matrix=%u other on page=%u distinct RIPs=%u",
    watch_faults.load(),watch_faults.load()-watch_other.load(),watch_other.load(),std::min(64u,watch_hit_count.load()));
  log(line);
  for(unsigned i=0;i<watch_hit_count.load() && i<64;++i) {
    std::snprintf(line,sizeof(line),"  matrix access rip=+0x%llX reads=%u writes=%u",static_cast<unsigned long long>(watch_hits[i].rva),
      watch_hits[i].reads,watch_hits[i].writes); log(line);
  }
  // The handler stays registered but inert (watch_active false) so a late single step on another thread is still handled.
}

// ---- Weapon component source matrix (2026-10-15, run 11) ----
// [LIVE run 10 watch] weapon+0x3FB0 is written by 0x14401D7BD..7D4 and read right after in the same function
// (0x14401DB14..35), plus readers 0x142E021A0..20F and 0x144016D45..DB8. [STATIC] 0x14401D420: component rdi =
// weapon+0x3FA0; matrix rdi+0x10 = 0x141859AD0(*(rdi+0xC8), out) (return +0x401D7B2), or via 0x143F966C0 (return
// +0x401D861). 0x141859AD0(obj, out) builds the world matrix of scene object obj from obj+0x2860 (0x14180E4E0), scale
// +0x2884..0x288C and cell +0x2890..0x2894 (x1024), returns out. So the component copies the weapon scene entity.
// With "Weapon effects follow tracked gun" + Script writes: for those two return addresses only, the returned matrix
// (within 0.25 m of the flat socket) gets the flat->IK socket change before the component and its readers use it.
// Logs the entity and its raw transform words (obj+0x2860..+0x2898) 1/s for the next step (moving the entity itself).
constexpr std::uintptr_t kEntityMatrixRva=0x1859AD0;
constexpr unsigned char kEntityMatrixSig[]={0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,
                                            0x48,0x89,0x7C,0x24,0x20,0x41,0x56,0x48,0x83,0xEC,0x60};
using EntityMatrix=float*(__fastcall*)(void*,float*);
EntityMatrix entity_original=nullptr;
bool effects_enabled=true;  // menu "Weapon follows tracked gun"; on by default (user request 2026-10-15)
std::atomic<unsigned> entity_moves{0},entity_lines{0};
std::atomic<unsigned> entity_direct_socket_copies{0};
std::atomic<ULONGLONG> entity_moved_tick{0};
float entity_last_raw[16]{},entity_last_written[16]{}; bool entity_have_last=false;  // game thread only
std::mutex socket_mutex;
float socket_used[16]{},socket_prev[16]{}; ULONGLONG socket_tick=0; float socket_motion=1.f;  // IK socket of the last write
float socket_ref[16]{}; ULONGLONG socket_ref_tick=0;  // reference >= 80 ms old for the stillness test
float socket_target[16]{}; ULONGLONG socket_target_tick=0;  // controller socket before the anti-sway correction
std::atomic<unsigned> entity_reused{0},entity_far_skips{0},entity_bound_moves{0};
float socket_local[3]{}; std::int16_t socket_cell[3]{}; bool socket_local_valid=false;  // socket_mutex: socket_used in its own cell
// [HEADSET run 18] "attach fully": placing the entity at m * inv(flat) * ik carried the body's walk (m is this game
// frame's entity, flat/ik are the drawn hand of the last solve), so the gun moved with the body while the drawn hands
// trail it. The entity is now placed at grip * ik: the drawn hand socket times a fixed grip offset (entity relative to
// the flat socket), measured once while body and hand are both still (< 3 mm over >= 80 ms), so no body travel enters.
// [LIVE run 19] grip=0 with 176 measures: two weapon entities (in hand and on the back) both pass here, so a single
// slot was reset on every call. One grip offset per weapon entity.
struct Grip { const void* obj=nullptr; float offset[16]; bool have=false; float correction[3]{}; ULONGLONG error_tick=0; vr_weapon_math::BoundOffset bound; };
// Closed loop (2026-10-15, run 20): the renderer redraws the skinned frame at this frame's drawn hand and reports the
// gap between that target root and where the game placed the weapon (rigid parts follow the game placement). Each new
// report moves the game placement by half the gap (clamped to 0.3 m), so the rigid parts converge onto the drawn hand,
// whatever the source of the offset (walking, thread timing).
constexpr bool kRenderFeedback=false;  // run 25: off (corrections stayed ~1 cm; fewer moving parts)
std::mutex error_mutex; float render_error[3]{}; ULONGLONG render_error_tick=0; std::atomic<unsigned> error_reports{0};
Grip grips_by_obj[4]; unsigned grip_next_slot=0;
float still_body_ref[3]{}; ULONGLONG still_body_tick=0; float body_motion=1.f;
std::atomic<unsigned> grip_measures{0};
std::atomic<ULONGLONG> entity_next{0};
float* __fastcall entity_detour(void* obj,float* out) {
  float* result=entity_original(obj,out);
  const std::uintptr_t ret=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(game::module_base());
  if(ret!=0x401D7B2 && ret!=0x401D861) return result;
  float m[16]{},flat[16],ik[16];
  if(!result || !game::mem_read(result,m,sizeof(m))) {
    mount_camera::observe_weapon_entity(obj,ret,false,m,"native_unreadable",-1.f,-1.f,false,false);
    return result;
  }
  // Preserve the native result before any mod re-use or placement. Observation never changes gating.
  const bool capture=mount_camera::capture_active(); float native[16]{};
  if(capture) std::memcpy(native,m,64);
  if(mount_camera::rider_view()) {
    // C1 uses a named TP palette binding and its selected-hand target. Do not overwrite its source
    // with FP90 bone 79, and do not move the candidate-matching identity/holstered entity.
    const char* reason="mounted_disabled"; float moved[16]{}; bool wrote=false;
    if(ret!=0x401D7B2) reason="mounted_unproven_caller";
    if(ret==0x401D7B2 && effects_enabled && write_permission.load(std::memory_order_relaxed) &&
       skin_palette_probe::mounted_weapon_entity_target(obj,m,moved,&reason)) {
      std::int16_t cell[3]{};
      if(game::mem_read(static_cast<unsigned char*>(obj)+0x2890,cell,sizeof(cell))) {
        float local[3]={moved[14]-1024.f*cell[2],moved[13]-1024.f*cell[1],moved[12]-1024.f*cell[0]};
        {float exact[3]; if(skin_palette_probe::mounted_entity_local(cell,exact)) {local[0]=exact[2]; local[1]=exact[1]; local[2]=exact[0];}}  // z,y,x
        bool fine=true; for(float v:local) fine=fine && std::isfinite(v) && std::fabs(v)<2048.f;
        if(fine && game::mem_write(static_cast<unsigned char*>(obj)+0x2864,local,sizeof(local))) {
          std::memcpy(result,moved,64); wrote=true;
          entity_moves.fetch_add(1,std::memory_order_relaxed); entity_moved_tick.store(GetTickCount64(),std::memory_order_relaxed);
        } else reason="mounted_position_rejected";
      } else reason="mounted_cell_unreadable";
      skin_palette_probe::mounted_weapon_entity_commit(obj,moved,wrote);
    }
    skin_palette_probe::note_mounted_weapon_attempt(reason,wrote);
    if(capture) mount_camera::observe_weapon_entity(obj,ret,true,native,reason,-1.f,-1.f,false,wrote);
    return result;
  }
  // [LIVE 2026-10-31] this hook also serves other characters' weapons (d_flat 850 m in VR_WEAPON.log). Each such call evaluated and
  // re-stamped the player's gun record, which the arm solve then takes its sample from; none may reach it.
  {float body_far[3]; const game::Pointers fp=game::pointers();
    if(skin_palette_probe::rig_coherent_on() && fp.body && game::mem_read(fp.body+0x94,body_far,sizeof(body_far)) && dist(m+12,body_far)>8.f) {
      entity_far_skips.fetch_add(1,std::memory_order_relaxed); return result;}}
  if(!skin_palette_probe::weapon_attach_place(skin_palette_probe::weapon_hand_index(),flat,ik)) {
    if(capture) mount_camera::observe_weapon_entity(obj,ret,true,native,"no_fresh_socket",-1.f,-1.f,false,false);
    return result;
  }
  // Our own entity position still in place (game did not re-attach it yet): start from the remembered flat matrix so the
  // hand change is never applied twice.
  if(ret==0x401D7B2 && entity_have_last && dist(m+12,entity_last_written+12)<0.001f) {
    std::memcpy(m,entity_last_raw,sizeof(m)); entity_reused.fetch_add(1,std::memory_order_relaxed);}
  const float d_flat=dist(m+12,flat+12);
  bool wrote=false;
  // [LIVE run 13] at 0.25 m the move was skipped while running (stale sockets, d_flat up to 0.33) -> gun jumped back.
  {float body[3]{}; const game::Pointers bp=game::pointers();
    if(bp.body && game::mem_read(bp.body+0x94,body,sizeof(body))) {
      const ULONGLONG t=GetTickCount64();
      if(!still_body_tick || t-still_body_tick>=80) {body_motion=still_body_tick?dist(body,still_body_ref):1.f;
        std::memcpy(still_body_ref,body,12); still_body_tick=t;}}}
  Grip* grip=nullptr;
  for(auto& g:grips_by_obj) if(g.obj==obj) {grip=&g; break;}
  if(!grip) {grip=&grips_by_obj[grip_next_slot++%4]; grip->obj=obj; grip->have=false;}
  bool& grip_have=grip->have; float (&grip_offset)[16]=grip->offset;
  float hand_motion=1.f; {std::lock_guard<std::mutex> lock(socket_mutex); if(socket_tick) hand_motion=socket_motion;}
  if(ret==0x401D7B2 && d_flat<0.5f && body_motion<0.003f && hand_motion<0.003f) {
    float inverse_flat[16];
    if(skin_arm_ik::inverse_affine(inverse_flat,flat)) {skin_arm_ik::multiply_affine(grip_offset,m,inverse_flat); grip_have=true;
      grip_measures.fetch_add(1,std::memory_order_relaxed);}
  }
  // [HEADSET run 25] static grip offsets (measured while still) gave a different hold after weapon switches and dropped
  // the game's per-weapon hold animation. The flat socket is now the game's own R_weaponAttachmentHand of THIS game
  // frame (90-bone FP pose, index 79; cell fixed by rounding against the entity): the gun moves by exactly
  // "game socket -> IK socket", keeping the game's grip and animations, with no frame mismatch.
  float game_socket[16]; bool have_game_socket=false; float d_game=-1.f;
  {const game::Pointers gp=game::pointers(); const unsigned char* data=nullptr; std::int32_t count=0;
    if(gp.pose && game::mem_read(gp.pose,&data,sizeof(data)) && game::mem_read(gp.pose+8,&count,sizeof(count)) && data && count==90 &&
       game::mem_read(data+79*64,game_socket,64)) {
      for(int k=0;k<3;++k) game_socket[12+k]+=std::round((m[12+k]-game_socket[12+k])/1024.f)*1024.f;
      d_game=dist(game_socket+12,m+12);
      // [LIVE run 57] moving/crouching animations hold the game's gun up to ~0.53 m from its own hand socket; the 0.5 m gate
      // then skipped our placement for that frame and the flat gun flashed. Holstered guns sit > 1.3 m away: gate 0.9 m.
      bool ok=d_game<0.9f; for(float v:game_socket) ok=ok&&std::isfinite(v);
      have_game_socket=ok;
    }}
  if(have_game_socket) std::memcpy(flat,game_socket,64);
  // [LIVE run 26] sprinting: the stale render-thread flat socket drifted 0.5-0.86 m from the entity, so this gate
  // rejected the move (wrote=0) and the gun flipped to the flat spot. With the game socket (d_game 0.000) gate on that.
  const float gate=have_game_socket?d_game:d_flat;
  if(effects_enabled && write_permission.load(std::memory_order_relaxed) && gate<0.9f) {
    float inverse[16],delta[16],moved[16];
    if(skin_arm_ik::inverse_affine(inverse,flat)) {
      skin_arm_ik::multiply_affine(delta,inverse,ik);
      float ik_corr[16]; std::memcpy(ik_corr,ik,64);
      {std::lock_guard<std::mutex> lock(socket_mutex); std::memcpy(socket_target,ik,64); socket_target_tick=GetTickCount64();}
      {float e[3]; ULONGLONG et=0;
        {std::lock_guard<std::mutex> lock(error_mutex); std::memcpy(e,render_error,12); et=render_error_tick;}
        if(kRenderFeedback && ret==0x401D7B2 && et && et!=grip->error_tick && GetTickCount64()-et<100) {
          grip->error_tick=et;
          for(int k=0;k<3;++k) grip->correction[k]+=0.5f*e[k];
          const float len=std::sqrt(grip->correction[0]*grip->correction[0]+grip->correction[1]*grip->correction[1]+grip->correction[2]*grip->correction[2]);
          if(!(len<0.3f)) for(float& v:grip->correction) v=std::isfinite(len)&&len>0.f?v*0.3f/len:0.f;
        }
        for(int k=0;k<3;++k) ik_corr[12+k]+=grip->correction[k];}
      std::memcpy(ik,ik_corr,64);  // the socket reported to the renderer includes the correction (its offset must not absorb it)
      if(have_game_socket || !grip_have) skin_arm_ik::multiply_affine(moved,m,delta);  // game socket -> IK socket (run 25)
      else skin_arm_ik::multiply_affine(moved,grip_offset,ik);                        // fallback: static grip offset
      // [AUTHOR 2026-10-31] the weapon is strictly the finished hand target. [LIVE] half of the moves are not on the socket (moves
      // 19342, direct 9703): the second entity, and any frame an animation holds the gun off its socket, re-entered the game's
      // motion through m * inverse(flat) while the hand stayed pinned to ik. Each object now keeps one offset from the socket.
      const bool bound_rig=skin_palette_probe::rig_coherent_on() && have_game_socket;
      if(bound_rig) {
        if(ret==0x401D7B2) {
          const game::Pointers wp=game::pointers(); const unsigned char* held=wp.body?ptr(wp.body,0x250):nullptr;
          vr_weapon_math::update_bound(grip->bound,held,m,flat,skin_palette_probe::player_standing_still(),GetTickCount64(),
            [](const float* a,const float* b,float t,float* out){vr_arms_math::blend_affine(a,b,t,out);});
        }
        if(grip->bound.have) {
          if(grip->bound.on_socket) std::memcpy(moved,ik,64);
          else skin_arm_ik::multiply_affine(moved,grip->bound.rel,ik);
          entity_bound_moves.fetch_add(1,std::memory_order_relaxed);
        }
      }
      // In the normal held pose the game's entity matrix is the game hand socket itself.  The three large
      // world-space float matrix products above can then add ~1 mm of roundoff to a gun whose IK hand is steady.
      // Preserve the multiplication for real per-weapon offsets and animated parts; copy the target only when
      // the two source frames already coincide in position and orientation.
      if(!bound_rig && have_game_socket && d_game<0.002f) {
        float rotation_match=0.f;
        for(int r=0;r<3;++r)
          for(int c=0;c<3;++c) rotation_match+=m[r*4+c]*flat[r*4+c];
        if(rotation_match>2.99999f) {
          std::memcpy(moved,ik,64);
          entity_direct_socket_copies.fetch_add(1,std::memory_order_relaxed);
        }
      }
      bool ok=dist(moved+12,m+12)<3.f; for(float v:moved) ok=ok&&std::isfinite(v);  // [HEADSET run 24] 1.5 m rejected arms out to the side
      if(ok) {std::memcpy(result,moved,64); wrote=true; entity_moves.fetch_add(1,std::memory_order_relaxed);
        entity_moved_tick.store(GetTickCount64(),std::memory_order_relaxed);
        {std::lock_guard<std::mutex> lock(socket_mutex);
          // [LIVE run 16] grip=91673/91932: the entity moves several times per frame, so "moved < 2 mm since the last
          // write" was almost always true and the grip offset was re-measured every frame, absorbing the hand motion.
          // Stillness is now measured against a reference at least 80 ms old (position and rotation).
          const ULONGLONG now_t=GetTickCount64();
          std::memcpy(socket_prev,socket_used,64); std::memcpy(socket_used,ik,64); socket_tick=now_t;
          socket_local_valid=skin_palette_probe::placed_socket_local(socket_local,socket_cell);
          if(!socket_ref_tick || now_t-socket_ref_tick>=80) {
            float turn=0.f; for(int r=0;r<3;++r) turn+=socket_used[r*4]*socket_ref[r*4]+socket_used[r*4+1]*socket_ref[r*4+1]+socket_used[r*4+2]*socket_ref[r*4+2];
            socket_motion=socket_ref_tick?std::max(dist(socket_used+12,socket_ref+12),(3.f-turn)):1.f;
            std::memcpy(socket_ref,socket_used,64); socket_ref_tick=now_t;
          }}
        // [HEADSET run 12] with only the matrix replaced, the gun took the IK rotation but stayed at the flat position:
        // after this call 0x14401D7D7 loads the position from the entity itself (x +0x286C, y +0x2868, z +0x2864,
        // cell-local; cells int16 x/y/z at +0x2890) and the cells. So the entity position gets the same IK position.
        if(ret==0x401D7B2) vr_weapon::timing_mark('E',ik+12);
        if(ret==0x401D7B2) {const long long us=qpc_us(); if(us-placement_last_us.exchange(us)>3000) placement_ticks.fetch_add(1);}
        if(ret==0x401D7B2) {
          std::int16_t cell[3]{};
          if(game::mem_read(static_cast<unsigned char*>(obj)+0x2890,cell,sizeof(cell))) {
            float local[3]={moved[14]-1024.f*cell[2],moved[13]-1024.f*cell[1],moved[12]-1024.f*cell[0]};  // z,y,x
            // On the socket: the cell-space position itself, never rounded through world floats.
            {float exact[3]; std::int16_t exact_cell[3];
              if(bound_rig && grip->bound.have && grip->bound.on_socket && skin_palette_probe::placed_socket_local(exact,exact_cell))
                for(int k=0;k<3;++k) local[2-k]=exact[k]+1024.f*(exact_cell[k]-cell[k]);}
            bool fine=true; for(float v:local) fine=fine&&std::isfinite(v)&&std::fabs(v)<2048.f;
            if(fine && game::mem_write(static_cast<unsigned char*>(obj)+0x2864,local,sizeof(local))) {
              std::memcpy(entity_last_raw,m,64); std::memcpy(entity_last_written,moved,64); entity_have_last=true;}
          }
        }}
    }
  }
  if(capture) {
    const char* reason=wrote?"placed":(!effects_enabled?"effects_disabled":
      (!write_permission.load(std::memory_order_relaxed)?"writes_disabled":
       (!(gate<0.9f)?"socket_gate_rejected":"transform_rejected")));
    mount_camera::observe_weapon_entity(obj,ret,true,native,reason,d_flat,d_game,have_game_socket,wrote);
  }
  const ULONGLONG now=GetTickCount64(); auto due=entity_next.load();
  if(entity_lines.load()<300 && now>=due && entity_next.compare_exchange_strong(due,now+1000)) {
    entity_lines.fetch_add(1);
    float raw[14]{}; game::mem_read(static_cast<unsigned char*>(obj)+0x2860,raw,sizeof(raw));
    std::int16_t cell[3]{}; game::mem_read(static_cast<unsigned char*>(obj)+0x2890,cell,sizeof(cell));
    char line[520]; std::snprintf(line,sizeof(line),"weapon entity: ret=+0x%llX obj=%p pos=(%.3f %.3f %.3f) d_flat=%.3f d_game_socket=%.3f d_ik=%.3f wrote=%d moves=%u reused=%u grip=%d/%u body_motion=%.4f corr=(%.3f %.3f %.3f) err_reports=%u "
      "direct=%u raw2860=(%.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f | %.3f %.3f) cell=(%d %d %d)",
      static_cast<unsigned long long>(ret),obj,m[12],m[13],m[14],d_flat,d_game,dist(m+12,ik+12),wrote?1:0,entity_moves.load(),entity_reused.load(),grip->have?1:0,grip_measures.load(),body_motion,grip->correction[0],grip->correction[1],grip->correction[2],error_reports.load(),
      entity_direct_socket_copies.load(std::memory_order_relaxed),
      raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],raw[8],raw[9],raw[10],raw[11],raw[12],raw[13],cell[0],cell[1],cell[2]);
    log(line);
  }
  return result;
}

// ---- Gameplay weapon world matrix (2026-10-15, run 9) ----
// [LIVE run 8] weapon (= *(body+0x250)) +0x3FB0..+0x3FEF is a 4x4 world matrix whose row 3 (+0x3FE0) matches the flat
// right weapon socket within 2 cm in 25/28 scans (rows 0-2 unit). [INFERENCE] the gameplay weapon transform that places
// the muzzle node -> flash, visible bullet origin and the muzzle ring (rigid mesh). Writer not found statically.
// "Weapon effects follow tracked gun (experimental)" (+ Script writes): replace it by M * inv(flat socket) * IK socket at
// the producer (weapon update), after the camera update and at Present. Our own value still in place is not re-applied.
float weapon_world_written[16]{}; bool weapon_world_have=false;
std::atomic<unsigned> weapon_world_writes{0},weapon_world_lines{0};
std::atomic<ULONGLONG> weapon_world_next{0};
void move_weapon_world(unsigned char* weapon,const char* phase) {
  if(!weapon) return;
  if(effects_enabled && write_permission.load(std::memory_order_relaxed) && !watch_done) watch_start(weapon);
  float m[16];
  if(!game::mem_read(weapon+0x3FB0,m,sizeof(m))) return;
  float flat[16],ik[16];
  // Riding: the same rule as on foot for the matrix that places the muzzle node, from the committed placement. [INFERENCE] the ring reads it.
  const bool riding=mount_camera::rider_view();
  if(riding?!skin_palette_probe::mounted_placed_pair(flat,ik):!skin_palette_probe::weapon_attach_matrices(skin_palette_probe::weapon_hand_index(),flat,ik)) return;
  const bool ours=weapon_world_have && !std::memcmp(m,weapon_world_written,sizeof(m));
  const float d_flat=dist(m+12,flat+12);
  float rot=0.f; for(int r=0;r<3;++r) rot+=m[r*4]*flat[r*4]+m[r*4+1]*flat[r*4+1]+m[r*4+2]*flat[r*4+2];  // 3 = same rotation
  bool wrote=false;
  if(!ours && write_permission.load(std::memory_order_relaxed) && effects_enabled && d_flat<(riding?0.9f:0.25f)) {
    float inverse[16],delta[16],out[16];
    if(skin_arm_ik::inverse_affine(inverse,flat)) {
      skin_arm_ik::multiply_affine(delta,inverse,ik);
      skin_arm_ik::multiply_affine(out,m,delta);
      bool ok=dist(out+12,m+12)<1.5f; for(float v:out) ok=ok&&std::isfinite(v);
      if(ok && game::mem_write(weapon+0x3FB0,out,sizeof(out))) {
        wrote=true; std::memcpy(weapon_world_written,out,64); weapon_world_have=true; weapon_world_writes.fetch_add(1);}
    }
  }
  const ULONGLONG now=GetTickCount64(); auto due=weapon_world_next.load();
  if(weapon_world_lines.load()<300 && now>=due && weapon_world_next.compare_exchange_strong(due,now+1000)) {
    weapon_world_lines.fetch_add(1);
    char line[300]; std::snprintf(line,sizeof(line),"weapon world%s [%s]: pos=(%.3f %.3f %.3f) d_flat=%.3f d_ik=%.3f rot_match=%.3f ours=%d wrote=%d writes=%u",
      riding?" riding":"",phase,m[12],m[13],m[14],d_flat,dist(m+12,ik+12),rot,ours?1:0,wrote?1:0,weapon_world_writes.load());
    log(line);
  }
}
// ---- The shot ray for the in-world reticle (FEATURE-14) ----
// [HEADSET, WEAPONS_AND_HANDS] with the mapped start and target, bullets go where the controller aims. [LIVE VR_WEAPON.log] the
// game's target is a point on its own camera ray, sometimes under 1.5 m away (near a surface), where the direction from the start
// means nothing. [HEADSET 2026-10-31] a ray taken fresh from every shot on the gun followed the flat recoil and reload. [SOURCE] the
// socket is the game's animated attach<-wrist relation times the wrist target. So the ray is held in the wrist's frame, which only the
// controller moves, and steadied (SteadyRay).
// [HEADSET 2026-11-07] after equipping, the mark sat off the gun for a few seconds, then snapped onto it. INFERENCE: the draw animation
// holds still long enough (500 ms) to be locked, and the right ray only replaces it after 2 s. So a weapon's settled ray is kept across
// equips, and a weapon with none waits 1.5 s after the change before it samples.
// Bows (2026-11-07 f): the steady ray learns from the drawn arrow only (see bow_ray), never from the game's aim.
// [HEADSET 2026-11-07 e] with the raw arrow read every frame, the mark followed the arrow at rest but went off while the trigger was
// held. [LIVE VR_WEAPON.log] bow_frames only grew in bursts (the arrow is seen mostly while drawing) and turned 32-111 deg/s then:
// the mark swapped between the learned ray and the swinging arrow of the draw animation. So the arrow is only a sample, kept when
// it is still and within 12 deg of the game's own aim (the drawn arrow is 2-7 deg from it, part-drawn ~32 deg).
// [HEADSET 2026-11-07 g] with that lock the mark sat on the resting arrow but left it while tensing. LIVE: the arrow is now seen every
// call (~60/s); at rest it lies 4-15 deg from the drawn arrow in the bow's frame (arrow_in_root 0.31/0.54 vs 0.90 m), so no one
// fixed ray fits both. So a bow follows this frame's arrow (wrist frame, half-step smoothing per sample), held while unseen.
struct ShotAim {const void* weapon=nullptr; int hand=-1; vr_weapon_math::SteadyRay steady; float distance=30.f; ULONGLONG at=0,since=0;
                bool bow=false; skin_arm_ik::Vec bow_origin{},bow_forward{}; ULONGLONG bow_at=0;
                bool bow_full=false;};  // FEATURE-07: the bow ray was learned at the game's full draw
// The ray the reticle and shots use, in the wrist frame: a bow's arrow, else the locked steady ray.
bool current_ray(const ShotAim& aim,ULONGLONG,skin_arm_ik::Vec& origin,skin_arm_ik::Vec& forward) {
  if(aim.bow) {if(!aim.bow_at) return false; origin=aim.bow_origin; forward=aim.bow_forward; return true;}
  if(!aim.steady.locked) return false;
  origin=aim.steady.origin; forward=aim.steady.forward; return true;
}
std::mutex shot_aim_mutex;
ShotAim shot_aim_state;
struct KnownRay {const void* weapon=nullptr; int hand=-1; vr_weapon_math::SteadyRay steady; bool bow=false;
                 bool bow_full=false; skin_arm_ik::Vec bow_origin{},bow_forward{};};
KnownRay known_rays[8];  // shot_aim_mutex; most recent first
std::atomic<unsigned> aim_casts{0},aim_hits{0},aim_locks{0},aim_restores{0},aim_changes{0},bow_samples{0},bow_rejects{0};
std::atomic<ULONGLONG> aim_next_line{0};
bool wrist_of_gun(int hand,const float* gun,float* wrist) {
  float rel[16],inverse[16];
  if(!skin_palette_probe::weapon_hold(hand,rel) || !skin_arm_ik::inverse_affine(inverse,rel)) return false;
  skin_arm_ik::multiply_affine(wrist,inverse,gun); return true;
}
void note_shot_ray(const void* weapon,const float* start,const float* target,const float* ik,const float* bow_start=nullptr,
                   const float* bow_dir=nullptr) {
  skin_arm_ik::Vec origin,forward,bow_o,bow_f; float wrist[16];
  const int hand=skin_palette_probe::weapon_hand_index();
  const float reach=dist(start,target);
  const bool wrist_ok=!skin_palette_probe::hand_swap_active() && wrist_of_gun(hand,ik,wrist);
  const bool good=reach>=1.5f && wrist_ok && vr_weapon_math::local_ray(start,target,wrist,origin,forward);
  bool bow_ok=false;
  if(bow_start && bow_dir && wrist_ok) {
    const float far_point[3]={bow_start[0]+bow_dir[0]*10.f,bow_start[1]+bow_dir[1]*10.f,bow_start[2]+bow_dir[2]*10.f};
    bow_ok=vr_weapon_math::local_ray(bow_start,far_point,wrist,bow_o,bow_f);
  }
  const ULONGLONG now=GetTickCount64();
  std::lock_guard<std::mutex> lock(shot_aim_mutex);
  auto& state=shot_aim_state;
  if(state.weapon!=weapon || state.hand!=hand) {
    int slot=7;
    if(state.steady.locked || state.bow) {  // keep the old one, newest first
      for(int i=0;i<7;++i) if(known_rays[i].weapon==state.weapon && known_rays[i].hand==state.hand) {slot=i; break;}
      for(int i=slot;i>0;--i) known_rays[i]=known_rays[i-1];
      known_rays[0]={state.weapon,state.hand,state.steady,state.bow,state.bow_full,state.bow_origin,state.bow_forward};
    }
    state.weapon=weapon; state.hand=hand; state.steady={}; state.bow=false; state.bow_at=0; state.bow_full=false; state.since=now; aim_changes.fetch_add(1,std::memory_order_relaxed);
    for(const auto& known:known_rays) if(known.weapon==weapon && known.hand==hand) {
      state.steady=known.steady; state.steady.try_last=0; state.bow=known.bow; aim_restores.fetch_add(1,std::memory_order_relaxed);
      if(known.bow_full) {state.bow_full=true; state.bow_origin=known.bow_origin; state.bow_forward=known.bow_forward; state.bow_at=now;}
      break;}
  }
  state.at=now;
  // A weapon that has shown an arrow is a bow from then on: it aims along its arrow, never the game's aim.
  if(bow_ok && !state.bow) {state.bow=true; state.bow_at=0;}
  if(state.bow) {
    if(!bow_ok) {bow_rejects.fetch_add(1,std::memory_order_relaxed); return;}
    bow_samples.fetch_add(1,std::memory_order_relaxed);
    // FEATURE-07 (HEADSET 2026-11-18): with the string pull, aim follows the bow hand only. The ray lives in the bow
    // hand's wrist frame; it is learned only when the game's draw is full and settled (part-drawn arrows run ~32 deg off,
    // resting ones 4-15 deg), and held through the manual pull and between draws.
    if(*bow_draw::enabled_flag() && state.bow_at) {
      const bool learn=bow_draw::pulling() && bow_draw::native_full();
      if(!learn && (bow_draw::pulling() || state.bow_full)) {state.bow_at=now; return;}
      if(learn) state.bow_full=true;
    }
    if(!state.bow_at || now-state.bow_at>250) {state.bow_origin=bow_o; state.bow_forward=bow_f;}
    else {
      using namespace skin_arm_ik;
      const Vec mixed=add(scale(state.bow_forward,.5f),scale(bow_f,.5f)); const float n=length(mixed);
      if(n>1e-4f) state.bow_forward=scale(mixed,1.f/n); else state.bow_forward=bow_f;
      state.bow_origin=add(scale(state.bow_origin,.5f),scale(bow_o,.5f));
    }
    state.bow_at=now;
    return;
  }
  // [HEADSET 2026-11-07 b] with the kept ray the mark showed at once but took about a second to settle onto the gun, every weapon.
  // INFERENCE: this equip's hold differs a little from the one the ray was kept from, and the slow refine (2%/sample, >=5 m) took ~1 s.
  // For 4 s after the change it refines 8x faster and down to 2.5 m.
  const bool settle=now-state.since<4000;
  if(good && (state.steady.locked || now-state.since>=1500) &&
     vr_weapon_math::steady_ray_sample(state.steady,origin,forward,reach>=(settle?2.5f:5.f),now,settle)) aim_locks.fetch_add(1,std::memory_order_relaxed);
}
// How far the ray runs. The camera collision filter misses what it ignores, so a target it lets through puts the reticle behind it.
void cast_shot_ray() {
  ShotAim aim; {std::lock_guard<std::mutex> lock(shot_aim_mutex); aim=shot_aim_state;}
  const ULONGLONG now=GetTickCount64();
  float frame[16],origin[3],forward[3],start[3],end[3];
  if(mount_camera::rider_view()) {
    // Mounted: the gun's calibrated ray. OPEN: the camera filter may stop on the mount itself when aiming across it.
    const game::Pointers p=game::pointers(); const void* weapon=p.body?ptr(p.body,0x250):nullptr;
    skin_palette_probe::MountedShotFrame gun;
    if(!weapon || !skin_palette_probe::mounted_weapon_shot_frame(weapon,gun) || !gun.calibrated) return;
    std::memcpy(frame,gun.root,64);
    origin[0]=gun.muzzle.x; origin[1]=gun.muzzle.y; origin[2]=gun.muzzle.z; forward[0]=gun.forward.x; forward[1]=gun.forward.y; forward[2]=gun.forward.z;
    std::lock_guard<std::mutex> lock(shot_aim_mutex); shot_aim_state.at=now;
  } else {
    float flat[16],ik[16];
    const int hand=skin_palette_probe::weapon_hand_index();
    skin_arm_ik::Vec o,f;
    if(!current_ray(aim,now,o,f) || now-aim.at>250 || !skin_palette_probe::weapon_attach_matrices(hand,flat,ik) || !wrist_of_gun(hand,ik,frame)) return;
    origin[0]=o.x; origin[1]=o.y; origin[2]=o.z; forward[0]=f.x; forward[1]=f.y; forward[2]=f.z;
    if(!aim.bow) vr_recoil::unkick_ray(hand,origin,forward);  // the wrist is kicked; the ray keeps the un-kicked aim
  }
  for(int k=0;k<3;++k) {
    start[k]=frame[12+k]; float way=0.f;
    for(int a=0;a<3;++a) {start[k]+=origin[a]*frame[a*4+k]; way+=forward[a]*frame[a*4+k];}
    end[k]=start[k]+way*99.f;
  }
  const auto cast=game::raycast_camera_filter({start[0],start[1],start[2]},{end[0],end[1],end[2]});
  if(!cast.executed) return;
  const float at[3]={cast.position.x,cast.position.y,cast.position.z};
  const float distance=cast.hit?dist(start,at):99.f;
  {std::lock_guard<std::mutex> lock(shot_aim_mutex); shot_aim_state.distance=distance;}
  aim_casts.fetch_add(1,std::memory_order_relaxed); if(cast.hit) aim_hits.fetch_add(1,std::memory_order_relaxed);
  auto due=aim_next_line.load();
  if(now>=due && aim_next_line.compare_exchange_strong(due,now+5000)) {
    char line[256]; std::snprintf(line,sizeof(line),"reticle ray: distance=%.2f hit=%d casts=%u hits=%u locks=%u equips=%u restored=%u bow=%d bow_samples=%u bow_rejects=%u",
      distance,cast.hit?1:0,aim_casts.load(),aim_hits.load(),aim_locks.load(),aim_changes.load(),aim_restores.load(),aim.bow?1:0,
      bow_samples.load(),bow_rejects.load());
    log(line);
  }
}
// ---- Bows: is the shot fixed to the drawn arrow? (read-only, 2026-11-07) ----
// [AUTHOR 2026-11-07] for bows the reticle and the shot should follow where the nocked arrow points, so a later manual draw
// shoots along the arrow. Discriminator: the mapped shot (start, direction) expressed in the drawn C_arrow_01 palette frame.
// Constant while aiming -> the shot has a fixed place on the arrow and can be carried by it. Also logged in the bow C_root
// frame, and the arrow's offset from the root (the draw animation). 1 line/s, 300 max.
std::atomic<ULONGLONG> arrow_next{0}; std::atomic<unsigned> arrow_lines{0};
float arrow_last_dir[3]{}; bool arrow_last_have=false;
void local_of(const float* inverse,const float* p,float* out,bool point) {
  for(int k=0;k<3;++k) {out[k]=point?inverse[12+k]:0.f; for(int a=0;a<3;++a) out[k]+=p[a]*inverse[a*4+k];}
}
void note_arrow_probe(const float* s2,const float* t2) {
  const ULONGLONG now=GetTickCount64(); auto due=arrow_next.load();
  if(arrow_lines.load()>=300 || now<due || !arrow_next.compare_exchange_strong(due,now+1000)) return;
  float arrow[16],bow_root[16],string[16]; bool have_string=false; char path[96]{};
  if(!skin_palette_probe::drawn_arrow(arrow,bow_root,string,&have_string,path)) return;
  float inv_arrow[16],inv_root[16];
  if(!skin_arm_ik::inverse_affine(inv_arrow,arrow) || !skin_arm_ik::inverse_affine(inv_root,bow_root)) return;
  float way[3]={t2[0]-s2[0],t2[1]-s2[1],t2[2]-s2[2]}; const float len=std::sqrt(way[0]*way[0]+way[1]*way[1]+way[2]*way[2]);
  if(!(len>1e-4f)) return;
  for(float& v:way) v/=len;
  float sa[3],da[3],sr[3],dr[3],rel[16]; local_of(inv_arrow,s2,sa,true); local_of(inv_arrow,way,da,false);
  local_of(inv_root,s2,sr,true); local_of(inv_root,way,dr,false);
  skin_arm_ik::multiply_affine(rel,arrow,inv_root);
  const float dn=std::sqrt(da[0]*da[0]+da[1]*da[1]+da[2]*da[2]);
  float turn=-1.f;
  if(dn>1e-4f) {
    if(arrow_last_have) {float d=0.f; for(int k=0;k<3;++k) d+=arrow_last_dir[k]*da[k]/dn; turn=std::acos(std::clamp(d,-1.f,1.f))*57.2958f;}
    for(int k=0;k<3;++k) arrow_last_dir[k]=da[k]/dn; arrow_last_have=true;
  }
  // world dot of each arrow palette axis with the shot direction: which axis is the arrow (if any)
  float axis_dot[3]; for(int r=0;r<3;++r) {
    const float n=std::sqrt(arrow[r*4]*arrow[r*4]+arrow[r*4+1]*arrow[r*4+1]+arrow[r*4+2]*arrow[r*4+2]);
    axis_dot[r]=n>1e-6f?(arrow[r*4]*way[0]+arrow[r*4+1]*way[1]+arrow[r*4+2]*way[2])/n:0.f;}
  const float d_string=have_string?dist(s2,string+12):-1.f;
  char line[600];
  std::snprintf(line,sizeof(line),"bow arrow probe [%s]: in_arrow start=(%.3f %.3f %.3f) dir=(%.3f %.3f %.3f) turn_1s=%.2f deg | "
    "in_root start=(%.3f %.3f %.3f) dir=(%.3f %.3f %.3f) | arrow_in_root t=(%.3f %.3f %.3f) | axis_dot=(%.3f %.3f %.3f) | "
    "d_start_arrow=%.3f d_start_root=%.3f d_start_string=%.3f",
    path,sa[0],sa[1],sa[2],da[0],da[1],da[2],turn,sr[0],sr[1],sr[2],dr[0],dr[1],dr[2],rel[12],rel[13],rel[14],
    axis_dot[0],axis_dot[1],axis_dot[2],dist(s2,arrow+12),dist(s2,bow_root+12),d_string);
  arrow_lines.fetch_add(1); log(line);
}
// [LIVE 2026-11-07 probe, still frames] with the arrow drawn, the game's own aim runs along the C_arrow_01 palette +Z axis: within
// ~2 deg (HUNTING_01, drawn 0.90 m) and ~7 deg (HUNTING_03, shortbow, drawn 0.54 m); part-drawn (0.31 m) it is ~32 deg off, the
// arrow not yet on target. [HEADSET 2026-11-07] with the learned ray the bow mark wandered and arrows left beside the bow.
// So a held bow's ray is the arrow: +Z of C_arrow_01 as drawn this frame, from the bow socket (the arrow rests at the grip).
std::atomic<unsigned> bow_frames{0};
bool bow_ray(const float* ik,float* start,float* dir) {
  float arrow[16],root[16],string[16]; char path[96];
  if(!skin_palette_probe::drawn_arrow(arrow,root,string,nullptr,path)) return false;
  const float n=std::sqrt(arrow[8]*arrow[8]+arrow[9]*arrow[9]+arrow[10]*arrow[10]);
  if(!(n>1e-4f) || !std::isfinite(n)) return false;
  for(int k=0;k<3;++k) {dir[k]=arrow[8+k]/n; start[k]=ik[12+k];}
  bow_frames.fetch_add(1,std::memory_order_relaxed); return true;
}
// The reticle's world point for this weapon: its ray (arrow or locked steady ray) carried by this frame's wrist, at the last cast distance.
std::atomic<unsigned> steered{0};
bool steady_aim_point(const void* weapon,int hand,const float* ik,float* frame,float* point,float* ray_start=nullptr,bool* is_bow=nullptr) {
  ShotAim aim; {std::lock_guard<std::mutex> lock(shot_aim_mutex); aim=shot_aim_state;}
  skin_arm_ik::Vec ro,rf;
  if(aim.weapon!=weapon || aim.hand!=hand || !current_ray(aim,GetTickCount64(),ro,rf) || !wrist_of_gun(hand,ik,frame)) return false;
  if(is_bow) *is_bow=aim.bow;
  float o[3]={ro.x,ro.y,ro.z};
  float f[3]={rf.x,rf.y,rf.z};
  if(!aim.bow) vr_recoil::unkick_ray(hand,o,f);  // shots and reticle keep the un-kicked aim (recoil is visual only)
  float start[3],way[3];
  for(int k=0;k<3;++k) {
    start[k]=frame[12+k]; way[k]=0.f;
    for(int a=0;a<3;++a) {start[k]+=o[a]*frame[a*4+k]; way[k]+=f[a]*frame[a*4+k];}
  }
  const float n=std::sqrt(way[0]*way[0]+way[1]*way[1]+way[2]*way[2]);
  if(!(n>1e-4f) || !std::isfinite(aim.distance)) return false;
  for(int k=0;k<3;++k) point[k]=start[k]+way[k]/n*aim.distance;
  if(ray_start) std::memcpy(ray_start,start,sizeof(start));
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]);
}
std::uintptr_t __fastcall detour(void* weapon,void* start,void* target,void* third,
                                 void* a5,std::uintptr_t a6,void* a7,std::uintptr_t a8,void* a9) {
  const game::Pointers p=game::pointers();
  const bool mine=p.body && weapon && ptr(p.body,0x250)==weapon;
  if(!mine) return original(weapon,start,target,third,a5,a6,a7,a8,a9);
  calls.fetch_add(1,std::memory_order_relaxed);
  move_weapon_world(static_cast<unsigned char*>(weapon),"producer");
  float s[3]{},t[3]{},v[3]{};
  const bool readable=game::mem_read(start,s,sizeof(s)) && game::mem_read(target,t,sizeof(t)) &&
                      (!third || game::mem_read(third,v,sizeof(v)));
  float flat[16],ik[16],s2[3]{},t2[3]{};
  const int hand=skin_palette_probe::weapon_hand_index();
  const bool riding=mount_camera::rider_view(); skin_palette_probe::MountedShotFrame mounted;
  const bool have_socket=readable && (riding?skin_palette_probe::mounted_weapon_shot_frame(weapon,mounted):
    skin_palette_probe::weapon_attach_matrices(hand,flat,ik));
  if(riding && have_socket) {std::memcpy(flat,mounted.flat,64); std::memcpy(ik,mounted.root,64);}
  bool mapped=have_socket && vr_weapon_math::map_shot(s,t,flat,ik,3.f,s2,t2);
  float baseline_start[3]; std::memcpy(baseline_start,s2,sizeof(s2)); bool calibrated=false;
  if(mapped && riding && mounted.calibrated) {
    mapped=vr_weapon_math::calibrated_shot(s,t,ik,mounted.muzzle,mounted.forward,3.f,s2,t2); calibrated=mapped;
  }
  if(readable && have_socket && !riding) skin_palette_probe::observe_onfoot_weapon_ray(weapon,flat,s,t);
  // [STATIC 2026-10-15] The caller 0x14446C760 (return +0x446C939) passes start = &aim.start of a stack struct at
  // start-0x18: +0x00 vec A -> weapon+0x56E8, +0x18 start -> +0x5254 (here), +0x24 vec B -> **weapon+0x5248** (copied
  // by the caller right after this call), +0x30 target -> +0x3E90, +0x3C third -> +0x3E9C. 0x14441C170 builds the shot
  // direction from origin +0x5254 only when (flag && weapon+0x3F64), otherwise from **+0x5248**, and toward +0x3E90 or
  // (global flag) +0x3E9C. [HEADSET run 1-3] aim followed the tracked gun but bullets kept the flat origin: B was never
  // moved. So B gets the same rigid change, written into the caller's struct just before its copy, and third is mapped
  // like target.
  const std::uintptr_t ret=reinterpret_cast<std::uintptr_t>(_ReturnAddress());
  const bool known_caller=ret==reinterpret_cast<std::uintptr_t>(game::module_base())+0x446C939;
  float a_vec[3]{},b_vec[3]{},b2[3]{},v2[3]{},unused[3];
  const bool have_ab=known_caller && game::mem_read(static_cast<unsigned char*>(start)-0x18,a_vec,sizeof(a_vec)) &&
                     game::mem_read(static_cast<unsigned char*>(start)+0x0C,b_vec,sizeof(b_vec));
  const float probe_dir[3]={b_vec[0]+1.f,b_vec[1],b_vec[2]};
  bool b_mapped=have_ab && have_socket && vr_weapon_math::map_shot(b_vec,probe_dir,flat,ik,3.f,b2,unused);
  if(b_mapped && calibrated) {
    for(int k=0;k<3;++k) b2[k]+=s2[k]-baseline_start[k];
    b_mapped=dist(b2,b_vec)<=3.f;
  }
  const bool v_mapped=third && mapped && (calibrated?
    vr_weapon_math::calibrated_shot(s,v,ik,mounted.muzzle,mounted.forward,3.f,unused,v2):
    vr_weapon_math::map_shot(s,v,flat,ik,3.f,unused,v2));
  const bool write=mapped && write_permission.load(std::memory_order_relaxed) &&
    (!riding || (known_caller && have_ab && b_mapped && (!third || v_mapped)));
  // A bow shoots along its learned arrow ray (note_shot_ray): start and B at the ray's start on the bow, target at the reticle (below).
  float game_s2[3]; std::memcpy(game_s2,s2,sizeof(s2));
  float bow_start[3],bow_dir[3],aim_wrist[16],aim_point[3],aim_start[3]; bool aim_bow=false;
  const bool arrow=write && !riding && bow_ray(ik,bow_start,bow_dir) && dist(bow_start,s)<=3.f;
  const bool aim_ok=write && !riding && steady_aim_point(weapon,hand,ik,aim_wrist,aim_point,aim_start,&aim_bow);
  if(aim_ok && aim_bow && dist(aim_start,s)<=3.f) {std::memcpy(s2,aim_start,sizeof(s2)); if(b_mapped) std::memcpy(b2,aim_start,sizeof(b2));}
  bool wrote_b=false;
  if(write && b_mapped) wrote_b=game::mem_write(static_cast<unsigned char*>(start)+0x0C,b2,sizeof(b2));
  if(write) shifted.fetch_add(1,std::memory_order_relaxed);
  if(write && !riding) {
    note_arrow_probe(game_s2,t2);
    note_shot_ray(weapon,game_s2,t2,ik,arrow?bow_start:nullptr,arrow?bow_dir:nullptr);
    // [HEADSET 2026-11-07 c] the reticle (steady ray) and the bullets (this frame's mapped game aim) part for a while after equip.
    // LIVE VR_WEAPON.log: locks=9 over equips=5, so the learned ray jumps more than 3 degrees at times. So shots aim at the reticle's
    // point: the steady ray in the wrist frame, at the reticle's cast distance. They leave the muzzle (s2) and meet the mark there.
    // Only while the ray is locked; before that the reticle is hidden and the mapped game aim stays.
    if(aim_ok && dist(s2,aim_point)>=1.f) {
      std::memcpy(t2,aim_point,sizeof(t2));
      if(v_mapped) std::memcpy(v2,aim_point,sizeof(v2));
      steered.fetch_add(1,std::memory_order_relaxed);
    }
  }
  else if(write_permission.load(std::memory_order_relaxed)) rejected.fetch_add(1,std::memory_order_relaxed);

  const ULONGLONG now=GetTickCount64(); auto due=next_line.load();
  if(lines.load()<600 && now>=due && next_line.compare_exchange_strong(due,now+1000)) {
    lines.fetch_add(1);
    const game::Player pl=game::read();
    const bool aim_object=ptr(ptr(p.body,0x1A8),0x60)!=nullptr;  // 0x143F8EBC0 prefers this aim point when valid
    char line[896];
    std::snprintf(line,sizeof(line),
      "shot hand=%c write=%d mapped=%d start=(%.3f %.3f %.3f) target=(%.3f %.3f %.3f) aim_len=%.2f third=(%.3f %.3f %.3f) "
      "d_cam=%.3f d_flat=%.3f d_ik=%.3f new_start=(%.3f %.3f %.3f) new_target=(%.3f %.3f %.3f) moved=%.3f aim_object=%d "
      "calls=%u shifted=%u rejected=%u steered=%u bow_frames=%u mounted=%d calibrated=%d commit_seq=%llu commit_age_ms=%llu",
      hand?'R':'L',write?1:0,mapped?1:0,s[0],s[1],s[2],t[0],t[1],t[2],dist(s,t),v[0],v[1],v[2],
      pl.valid?dist(s,pl.cam+12):-1.f,have_socket?dist(s,flat+12):-1.f,have_socket?dist(s,ik+12):-1.f,
      s2[0],s2[1],s2[2],t2[0],t2[1],t2[2],mapped?dist(s,s2):0.f,aim_object?1:0,calls.load(),shifted.load(),rejected.load(),steered.load(),bow_frames.load(),
      riding,calibrated,static_cast<unsigned long long>(mounted.sequence),static_cast<unsigned long long>(mounted.age_ms));
    log(line);
    if(!riding) {std::snprintf(line,sizeof(line),"  bow capture: %s",skin_palette_probe::arrow_report().c_str()); log(line);}
    if(have_ab) {
      std::snprintf(line,sizeof(line),"  aimdata A=(%.3f %.3f %.3f) d_cam=%.3f d_start=%.3f | B=(%.3f %.3f %.3f) d_cam=%.3f d_start=%.3f "
        "d_flat=%.3f d_ik=%.3f new_B=(%.3f %.3f %.3f) wrote_B=%d third_mapped=%d",
        a_vec[0],a_vec[1],a_vec[2],pl.valid?dist(a_vec,pl.cam+12):-1.f,dist(a_vec,s),b_vec[0],b_vec[1],b_vec[2],
        pl.valid?dist(b_vec,pl.cam+12):-1.f,dist(b_vec,s),have_socket?dist(b_vec,flat+12):-1.f,have_socket?dist(b_vec,ik+12):-1.f,
        b2[0],b2[1],b2[2],wrote_b?1:0,v_mapped?1:0);
      log(line);
    }
  }
  if(write) {
    const std::uintptr_t result=original(weapon,s2,t2,v_mapped?v2:third,a5,a6,a7,a8,a9);
    const ULONGLONG t_now=GetTickCount64(); auto t_due=scan_next.load();
    if(scan_lines.load()<160 && t_now>=t_due && scan_next.compare_exchange_strong(t_due,t_now+3000))
      scan_for_weapon(static_cast<const unsigned char*>(weapon),p.body);  // run 8 (S copies: none in run 7)
    return result;
  }
  return original(weapon,start,target,third,a5,a6,a7,a8,a9);
}
// ---- Read-only: does the published Muzzle_Position follow the shifted origin? (2026-10-15, run 3) ----
// [STATIC, retired weapon_origin notes] 0x143789B50(agent, out) builds the agent muzzle matrix; caller +3954CED publishes
// it as the graph property Muzzle_Position, +40068BD / +4007943 are other consumers. [LIVE earlier] mode 2 output ==
// weapon+0x5254. [HEADSET] with the producer shift on, flash/tracer stayed at the flat gun. Discriminator: output vs the
// current (shifted) origin and vs the camera, per caller, while firing. Original first, never modified.
constexpr std::uintptr_t kBuilderRva=0x3789B50;
constexpr unsigned char kBuilderSig[]={0x48,0x8B,0xC4,0x48,0x89,0x58,0x10,0x48,0x89,0x78,0x18,0x55,0x48,0x8D,0x68,0xD8};
using Builder=float*(__fastcall*)(void*,float*);
Builder builder_original=nullptr;
std::atomic<ULONGLONG> builder_next[4]{};
std::atomic<unsigned> builder_lines{0};
float* __fastcall builder_detour(void* agent,float* out) {
  float* result=builder_original(agent,out);
  const game::Pointers p=game::pointers();
  if(!p.body || agent!=p.body || builder_lines.load()>=400) return result;
  const std::uintptr_t caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(game::module_base());
  const unsigned slot=caller==0x3954CED?0:caller==0x40068BD?1:caller==0x4007943?2:3;
  const ULONGLONG now=GetTickCount64(); auto due=builder_next[slot].load();
  if(now<due || !builder_next[slot].compare_exchange_strong(due,now+1000)) return result;
  builder_lines.fetch_add(1);
  float m[16]{},origin[3]{}; unsigned mode=0;
  const bool readable=result && game::mem_read(result,m,sizeof(m));
  const unsigned char* weapon=ptr(p.body,0x250);
  const bool have_origin=weapon && game::mem_read(weapon+0x5254,origin,sizeof(origin));
  game::mem_read(p.body+0x514,&mode,sizeof(mode));
  const game::Player pl=game::read();
  char line[400];
  std::snprintf(line,sizeof(line),"muzzle_position caller=+%llX mode=%u pos=(%.3f %.3f %.3f) d_origin=%.3f d_cam=%.3f shifting=%d shifted=%u",
    static_cast<unsigned long long>(caller),mode,m[12],m[13],m[14],readable&&have_origin?dist(m+12,origin):-1.f,
    readable&&pl.valid?dist(m+12,pl.cam+12):-1.f,write_permission.load()?1:0,shifted.load());
  log(line);
  return result;
}
// ---- Player-view graph muzzle properties (2026-10-15) ----
// [STATIC] 0x143954A50(this, flag) updates player-view graph properties (names registered in avr::PlayerViewCtor:
// Muzzle_Position, Secondary_Muzzle_Position). agent = *(*(this+0x18)+0x28). If agent+0x258 -> +0x10 is set, it builds
// Muzzle_Position from 0x143789B50 (weapon origin, already moved by the producer hook) into binding this+0x140 and
// Secondary from 0x14378FD50 into this+0x150. Otherwise it copies source bindings with 0x143863630(dst, src):
// this+0x140 <- this+0xD0 (Muzzle_Position), +0x120 <- +0xB0, +0x130 <- +0xC0, +0x160 <- +0xE0, +0x170 <- +0xF0,
// +0x180 <- +0x100/+0xF0, +0x190 <- +0x110. 0x143863630 copies a reference to the source entry (entry type
// 0x148DB6490 = matrix, data at *(entry+0)), not the matrix. [LIVE run 3] the 0x143789B50 branch did not run in that
// session (publisher caller +3954CED never seen) -> the copy branch is the live path.
// [INFERENCE] the tracer (graph shape node "tracerEffect") and the flash read Muzzle_Position, i.e. the source matrix
// at this+0xD0, which the first-person animation fills at the flat gun.
// Read-only: log every matrix source of this publisher (1/s). With "Shoot from tracked gun" on (Script writes on), the
// Muzzle_Position source matrix (this+0xD0) and Secondary (this+0xE0) get the flat->IK socket change before the copy.
constexpr std::uintptr_t kViewPubRva=0x3954A50;
constexpr unsigned char kViewPubSig[]={0x48,0x8B,0xC4,0x48,0x89,0x58,0x10,0x48,0x89,0x68,0x18,0x48,0x89,0x70,0x20,0x57,
                                       0x48,0x81,0xEC,0xB0,0x00,0x00,0x00};
constexpr std::uintptr_t kMatrixTypeRva=0x8DB6490;
using ViewPub=std::uintptr_t(__fastcall*)(void*,std::uintptr_t,void*,void*);
ViewPub view_original=nullptr;
std::atomic<ULONGLONG> view_next{0};
std::atomic<unsigned> view_lines{0},view_moved{0};
float view_written[2][16]{}; bool view_have_written[2]{};
float* source_matrix(void* self,unsigned offset) {
  const unsigned char* entry=ptr(self,offset);
  if(!entry || ptr(entry,0x18)!=game::module_base()+kMatrixTypeRva) return nullptr;
  return reinterpret_cast<float*>(const_cast<unsigned char*>(ptr(entry,0)));
}
std::atomic<unsigned> view_calls{0},view_id_lines{0};
std::atomic<ULONGLONG> view_id_next{0};
std::uintptr_t __fastcall view_detour(void* self,std::uintptr_t flag,void* a3,void* a4) {
  const game::Pointers p=game::pointers();
  const unsigned char* agent=ptr(ptr(self,0x18),0x28);
  view_calls.fetch_add(1,std::memory_order_relaxed);
  // [LIVE run 5] no "view props" line at all: identify the agent this publisher belongs to (1/s, 60 lines).
  {const ULONGLONG now=GetTickCount64(); auto due=view_id_next.load();
    if(view_id_lines.load()<60 && now>=due && view_id_next.compare_exchange_strong(due,now+1000)) {
      view_id_lines.fetch_add(1);
      char line[300]; std::snprintf(line,sizeof(line),"view publisher id: calls=%u self=%p this18=%p agent=%p body=%p entity=%p handle=%p "
        "this20_28=%p",view_calls.load(),self,ptr(self,0x18),agent,p.body,p.entity,p.handle,ptr(ptr(self,0x20),0x28));
      log(line);
    }}
  if(!p.body || agent!=p.body) return view_original(self,flag,a3,a4);
  const bool builder_branch=[&]{const unsigned char* q=ptr(agent,0x258); unsigned char on=0;
    return q && game::mem_read(q+0x10,&on,1) && on!=0;}();
  float flat[16],ik[16];
  const bool have_socket=skin_palette_probe::weapon_attach_matrices(skin_palette_probe::weapon_hand_index(),flat,ik);
  float delta[16],inverse[16];
  const bool have_delta=have_socket && skin_arm_ik::inverse_affine(inverse,flat) && (skin_arm_ik::multiply_affine(delta,inverse,ik),true);
  const unsigned moved_offsets[2]={0xD0,0xE0};
  bool wrote[2]{};
  if(write_permission.load(std::memory_order_relaxed) && have_delta) {
    for(int k=0;k<2;++k) {
      float* data=source_matrix(self,moved_offsets[k]); float m[16];
      if(!data || !game::mem_read(data,m,sizeof(m))) continue;
      // Our own value still in place (the animation did not refresh it): do not apply the change twice.
      if(view_have_written[k] && !std::memcmp(m,view_written[k],sizeof(m))) continue;
      float out[16]; skin_arm_ik::multiply_affine(out,m,delta);
      bool ok=dist(out+12,m+12)<1.5f; for(float v:out) ok=ok&&std::isfinite(v);
      if(ok && game::mem_write(data,out,sizeof(out))) {wrote[k]=true; std::memcpy(view_written[k],out,64); view_have_written[k]=true;
        view_moved.fetch_add(1,std::memory_order_relaxed);}
    }
  }
  const ULONGLONG now=GetTickCount64(); auto due=view_next.load();
  if(view_lines.load()<400 && now>=due && view_next.compare_exchange_strong(due,now+1000)) {
    view_lines.fetch_add(1);
    const game::Player pl=game::read();
    char line[900]; int n=std::snprintf(line,sizeof(line),"view props builder_branch=%d moved=%u wrote=%d/%d:",builder_branch?1:0,
      view_moved.load(),wrote[0]?1:0,wrote[1]?1:0);
    const unsigned offsets[7]={0xB0,0xC0,0xD0,0xE0,0xF0,0x100,0x110};
    for(unsigned off:offsets) {
      float* data=source_matrix(self,off); float m[16];
      if(!data || !game::mem_read(data,m,sizeof(m))) {n+=std::snprintf(line+n,sizeof(line)-n," +%X=-",off); continue;}
      n+=std::snprintf(line+n,sizeof(line)-n," +%X=(%.3f %.3f %.3f) d_cam=%.3f d_flat=%.3f d_ik=%.3f",off,m[12],m[13],m[14],
        pl.valid?dist(m+12,pl.cam+12):-1.f,have_socket?dist(m+12,flat+12):-1.f,have_socket?dist(m+12,ik+12):-1.f);
      if(n>820) break;
    }
    log(line);
  }
  return view_original(self,flag,a3,a4);
}
}  // namespace

namespace vr_weapon {
bool* shoot_from_tracked_flag() { return &enabled; }
bool* effects_flag() { return &effects_enabled; }
void report_render_error(const float* world_error) {
  bool ok=true; for(int k=0;k<3;++k) ok=ok&&std::isfinite(world_error[k])&&std::fabs(world_error[k])<1.f;
  if(!ok) return;
  std::lock_guard<std::mutex> lock(error_mutex);
  std::memcpy(render_error,world_error,12); render_error_tick=GetTickCount64(); error_reports.fetch_add(1);
}
bool target_socket(float (&ik_world)[16]) {
  std::lock_guard<std::mutex> lock(socket_mutex);
  if(!socket_target_tick || GetTickCount64()-socket_target_tick>150) return false;
  std::memcpy(ik_world,socket_target,64); return true;
}
bool written_socket(float (&ik_world)[16],float* motion,unsigned long long* age_ms) {
  std::lock_guard<std::mutex> lock(socket_mutex);
  const ULONGLONG age=GetTickCount64()-socket_tick;
  if(!socket_tick || age>150) return false;
  if(age_ms) *age_ms=age;
  std::memcpy(ik_world,socket_used,64); if(motion) *motion=socket_motion; return true;
}
bool written_socket_in_cell(float (&m)[16],const std::int16_t* cell) {
  std::lock_guard<std::mutex> lock(socket_mutex);
  if(!socket_tick || !socket_local_valid || GetTickCount64()-socket_tick>150) return false;
  std::memcpy(m,socket_used,64);
  for(int k=0;k<3;++k) m[12+k]=socket_local[k]+1024.f*(socket_cell[k]-cell[k]);
  return true;
}
bool weapon_placed_by_game() { return GetTickCount64()-entity_moved_tick.load(std::memory_order_relaxed)<150; }
unsigned rig_far_skips() {return entity_far_skips.load(std::memory_order_relaxed);}
unsigned rig_bound_moves() {return entity_bound_moves.load(std::memory_order_relaxed);}
void on_camera_post() {
  const game::Pointers p=game::pointers();
  if(p.body) move_weapon_world(const_cast<unsigned char*>(ptr(p.body,0x250)),"camera_post");
  cast_shot_ray();
}
bool shot_distance(float* distance) {
  std::lock_guard<std::mutex> lock(shot_aim_mutex);
  if(GetTickCount64()-shot_aim_state.at>250) return false;
  *distance=shot_aim_state.distance; return true;
}
bool shot_ray_on() {return write_permission.load(std::memory_order_relaxed);}
bool held_is_bow() {std::lock_guard<std::mutex> lock(shot_aim_mutex); return shot_aim_state.bow;}
bool shot_aim(float* origin,float* forward,float* distance) {
  std::lock_guard<std::mutex> lock(shot_aim_mutex);
  const ULONGLONG now=GetTickCount64(); skin_arm_ik::Vec o,f;
  if(now-shot_aim_state.at>250 || !current_ray(shot_aim_state,now,o,f)) return false;
  origin[0]=o.x; origin[1]=o.y; origin[2]=o.z;
  forward[0]=f.x; forward[1]=f.y; forward[2]=f.z; *distance=shot_aim_state.distance;
  return true;
}
unsigned placement_tick() { return placement_ticks.load(); }
void timing_mark(char kind,const float* world_pos,unsigned value) {
  if(timing_active.load(std::memory_order_relaxed)) timing_push(kind,value,world_pos);
}
void timing_offer(const void* gun_bone0) {
  const ULONGLONG t=GetTickCount64();
  if(!timing_gun_since.load()) timing_gun_since.store(t);
  if(timing_started || !gun_bone0 || t-timing_gun_since.load()<5000 || !timing_turning.load()) return;
  timing_started=true;
  timing_count.store(0);
  timing_until.store(t+3000); timing_active.store(true);
  if(!watch_range(reinterpret_cast<std::uintptr_t>(gun_bone0),64,3000,"gun bone0 matrix")) log("gun timing: watch not armed (events only)");
  log("gun timing: 3 s window started (view turning, gun in hand)");
}
void on_present() {
  vr_recoil::on_present();
  watch_poll();
  timing_poll();
  {const game::Pointers p=game::pointers();
    if(p.body && tried) move_weapon_world(const_cast<unsigned char*>(ptr(p.body,0x250)),"present");}
  write_permission.store(enabled && *lua_host::writes_flag(),std::memory_order_relaxed);
  if(tried || !game::read().valid) return;
  tried=true;
  unsigned char* target=game::module_base()+kProducerRva;
  unsigned char bytes[sizeof(kProducerSig)]{};
  if(!game::mem_read(target,bytes,sizeof(bytes)) || std::memcmp(bytes,kProducerSig,sizeof(bytes))) {
    log("vr_weapon SKIP: guard mismatch at +0x445C570 (different build?)"); return;}
  if(MH_CreateHook(target,reinterpret_cast<void*>(&detour),reinterpret_cast<void**>(&original))!=MH_OK ||
     MH_EnableHook(target)!=MH_OK) {log("vr_weapon SKIP: MinHook"); return;}
  {unsigned char* b=game::module_base()+kBuilderRva; unsigned char got[sizeof(kBuilderSig)]{};
    if(game::mem_read(b,got,sizeof(got)) && !std::memcmp(got,kBuilderSig,sizeof(got)) &&
       MH_CreateHook(b,reinterpret_cast<void*>(&builder_detour),reinterpret_cast<void**>(&builder_original))==MH_OK &&
       MH_EnableHook(b)==MH_OK) log("vr_weapon: read-only Muzzle_Position builder observer on +0x3789B50");
    else log("vr_weapon: Muzzle_Position observer skipped (guard/hook)");}
  {unsigned char* b=game::module_base()+kViewPubRva; unsigned char got[sizeof(kViewPubSig)]{};
    if(game::mem_read(b,got,sizeof(got)) && !std::memcmp(got,kViewPubSig,sizeof(got)) &&
       MH_CreateHook(b,reinterpret_cast<void*>(&view_detour),reinterpret_cast<void**>(&view_original))==MH_OK &&
       MH_EnableHook(b)==MH_OK) log("vr_weapon: player-view muzzle property hook on +0x3954A50");
    else log("vr_weapon: player-view property hook skipped (guard/hook)");}
  {unsigned char* b=game::module_base()+kEntityMatrixRva; unsigned char got[sizeof(kEntityMatrixSig)]{};
    if(game::mem_read(b,got,sizeof(got)) && !std::memcmp(got,kEntityMatrixSig,sizeof(got)) &&
       MH_CreateHook(b,reinterpret_cast<void*>(&entity_detour),reinterpret_cast<void**>(&entity_original))==MH_OK &&
       MH_EnableHook(b)==MH_OK) log("vr_weapon: weapon entity matrix hook on +0x1859AD0 (returns into +0x401D7B2/+0x401D861 only)");
    else log("vr_weapon: weapon entity matrix hook skipped (guard/hook)");}
  log("vr_weapon build " __DATE__ " " __TIME__ ": hook on the shot producer 0x14445C570 (player weapon only). Moves the "
      "shot start/aim onto the IK gun only while \"Shoot from tracked gun (experimental)\" and Script writes are on.");
}
}  // namespace vr_weapon
