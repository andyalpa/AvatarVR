#include "../runtime_log.hpp"
#include "move_probe.hpp"
#include "../game/mount_state.hpp"
#include "../game/player.hpp"
#include "../script/lua_host.hpp"
#include "../camera/native_stereo.hpp"
#include <windows.h>
#include <MinHook.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include "../settings_file.hpp"

// Field map (STATIC, research/ROOMSCALE_BODY_20261022.md): P+0x38 body (pos +0x94), P+0x28 -> +0xA8 controller CC,
// P+0x18 motion state M (target +0x50, swimming int +0x124), CC+0x2B8 hknpCharacterProxy (pos +0x80, vel +0xB0),
// CC+0x298/+0x29C substep/accumulator, branch flags (P+0x28)+0x14 / +0xCC and CC+0x36A.
namespace {
constexpr std::uintptr_t kMoveUpdateRva=0x4271680;
constexpr unsigned char kMoveUpdateSig[]={0x40,0x55,0x56,0x57,0x48,0x8D,0x6C,0x24,0xC0,0x48,0x81,0xEC,0x40,0x01,0x00,0x00};
constexpr ULONGLONG kWindowMs=20000;
constexpr unsigned kMaxRecords=6000;
using Update=void(__fastcall*)(void*);
Update original_update=nullptr;

struct Snap {
  bool ok=false;
  float body[3]{},target[3]{},proxy[3]{},vel[3]{},cc_pos[3]{};
  float step=0,acc=0;
  unsigned char f14=0,fcc=0,f36a=0;
  int swim=0;
};
enum Kind : unsigned { Move, CamPre, CamPost };
struct Record {
  unsigned kind=Move; unsigned long long seq=0; DWORD tid=0; long long us=0; unsigned present=0;
  const void* p=nullptr; const void* cc=nullptr; const void* m=nullptr;
  Snap pre,post;
};

std::atomic<ULONGLONG> deadline{0};
std::atomic<const void*> local_body{nullptr};
std::atomic<unsigned long long> seq{0};
std::atomic<unsigned> present_index{0};
std::atomic<unsigned> calls_all{0},calls_local{0},dropped{0},camera_events{0};
std::mutex mutex;
Record records[1024];
unsigned count=0,total=0;
unsigned branches[5]{};  // 1 scripted, 2 teleport-to-target, 3 integrated, 4 swimming, 0 unread
DWORD threads[8]{}; unsigned thread_count=0;
std::wstring dir;
bool on=false,created=false,enabled=false,failed=false;
ULONGLONG last_poll=0;
LARGE_INTEGER qpf{},qpc0{};

const unsigned char* ptr(const void* base,unsigned off) {
  const unsigned char* v=nullptr;
  if (base) game::mem_read(static_cast<const unsigned char*>(base)+off,&v,sizeof(v));
  return v;
}
bool vec(const unsigned char* base,unsigned off,float out[3]) { return base && game::mem_read(base+off,out,12); }

Snap snap(const void* p,const unsigned char* cc,const unsigned char* m) {
  Snap s;
  const auto body=ptr(p,0x38), owner=ptr(p,0x28), proxy=ptr(cc,0x2B8);
  s.ok=vec(body,0x94,s.body) && vec(m,0x50,s.target) && vec(proxy,0x80,s.proxy) && vec(proxy,0xB0,s.vel) &&
       vec(cc,0x2A0,s.cc_pos) && game::mem_read(cc+0x298,&s.step,4) && game::mem_read(cc+0x29C,&s.acc,4) &&
       owner && game::mem_read(owner+0x14,&s.f14,1) && game::mem_read(owner+0xCC,&s.fcc,1) &&
       game::mem_read(cc+0x36A,&s.f36a,1) && game::mem_read(m+0x124,&s.swim,4);
  return s;
}
// The branch 0x144271680 takes after integrating, predicted from the flags it reads (STATIC).
unsigned branch(const Snap& s) {
  if (!s.ok) return 0;
  if (s.f14) return 1;
  if (!s.fcc || !s.f36a) return s.swim ? 4 : 2;
  return 3;
}
long long now_us() {
  LARGE_INTEGER t; QueryPerformanceCounter(&t);
  return qpf.QuadPart ? (t.QuadPart-qpc0.QuadPart)*1000000/qpf.QuadPart : 0;
}
bool armed() { const auto until=deadline.load(std::memory_order_relaxed); return until && GetTickCount64()<until; }

void push(const Record& r) {
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
  if (!lock.owns_lock() || count>=1024 || total>=kMaxRecords) { dropped.fetch_add(1,std::memory_order_relaxed); return; }
  records[count++]=r; ++total;
  if (r.kind==Move) {
    ++branches[branch(r.pre)];
    bool seen=false;
    for (unsigned i=0;i<thread_count;++i) seen|=threads[i]==r.tid;
    if (!seen && thread_count<8) threads[thread_count++]=r.tid;
  }
}

// Nudge test (write): "nudge" in the control file + Script writes + F11 queues one 10 cm horizontal offset along the
// game camera's forward. The 2026-10-23 07:58 run showed that adding it to M+0x50 becomes smoothed momentum (~39 cm
// coasting), so the offset is now applied at the Havok step instead: during the next local, integrated-branch
// movement call, the first hknpCharacterProxy::integrate (0x143451EA0) for that controller's proxy runs with
// velocity v+d/h (h = step delta time), and afterwards the injected part is taken out of the proxy velocity again so
// no momentum carries into the next substep. Into-wall bounce is prevented: the velocity component along d is never
// left below min(0, the game's own). The call and the next kFollow calls are logged to MOVE_NUDGE.log.
constexpr std::uintptr_t kIntegrateRva=0x3451EA0;
constexpr unsigned char kIntegrateSig[]={0x48,0x8B,0xC4,0x48,0x89,0x58,0x08,0x4C,0x89,0x48,0x20,0x48,0x89,0x50,0x10,0x55};
using Integrate=std::uintptr_t(__fastcall*)(void*,void*,void*,void*,void*,void*,std::uintptr_t);
Integrate original_integrate=nullptr;
bool integrate_created=false,integrate_enabled=false;
constexpr float kNudge=0.10f;
constexpr unsigned kFollow=6;
struct Inject {
  bool on=false,done=false,roomscale=false;
  const void* proxy=nullptr;
  float d[3]{},h=0,v_game[3]{},v_post[3]{},v_final[3]{},pos_pre[3]{},pos_post[3]{};
  float consumed[3]{};    // roomscale: proxy displacement in the injected step minus the game's own v_game*h
  unsigned integrates=0;  // local-proxy integrate calls seen during the shot movement call
};
// Roomscale prototype ("roomscale" in the control file + Script writes, default off): every local integrated-branch
// movement call requests the horizontal headset offset from the game camera (native_stereo::head_offset, minus moves
// already queued for the view) beyond kDeadZone, at most kMaxSpeed*h, through the same one-step Havok injection.
// What the capsule actually moved is queued with native_stereo::rebase_head, so the eyes stay where the headset is.
// Smooth follow (2026-10-23 headset history): v1 moved only the excess beyond 8 cm (smooth, but the head stayed 8 cm
// ahead and the body shoulders sat off the IK shoulders); v2 chased from 8 cm back to 1.5 cm at the speed cap (shoulders
// fixed, but "jumpy": ~3 steps of ~2.8 cm a second). v3: every call the body closes h/kFollowTau of the offset beyond
// kDeadZone (exponential, time constant kFollowTau), so it glides and settles kDeadZone under the head.
constexpr float kDeadZone=0.03f;    // metres of head offset left under the body at rest
constexpr float kFollowTau=0.2f;    // seconds: follow time constant
constexpr float kMaxSpeed=1.5f;     // m/s cap on the requested body speed
std::atomic<bool> roomscale_armed{false};
// Body follows head yaw (on by default, "nobodyturn" off; "turndz=<deg>" sets the dead zone, default 15). History: the first version
// wrote look +0x34 in camera_pre and FAILED in the headset (the view and the IK arms turned with the head): the game
// rebuilds +0x34 from its internal look state every frame (WEAPONS_AND_HANDS 2026-10-15 06:56 LIVE), so the write
// moved one frame's camera while the view rebase accumulated. This version steers the internal state the way
// pitch_level does: while the user's right stick X is idle and the head is more than the dead zone off the game
// camera's yaw, the virtual pad reports a right-stick X that turns the game toward the head (sign learned from the
// measured response), until the head is within the dead zone minus kTurnHyst. The view is rebased only by the yaw the
// game camera ACTUALLY turned (look +0x34 between consecutive camera-update post phases, the camera's own yaw), and
// only while the mod drove the stick (plus kDriveTail for the game's turn deceleration), so the eyes and hands stay
// world-fixed whatever the game's stick curve does, while the body, its shoulders and the arm rig turn.
constexpr float kTurnHyst=0.0524f;   // 3 degrees below the dead zone the turn stops
constexpr float kTurnGain=1.5f;      // stick per radian beyond the stop angle
constexpr float kTurnMinStick=0.3f;  // above typical right-stick deadzones
constexpr float kTurnMaxStick=0.8f;
constexpr float kUserIdle=0.15f;     // |user right stick X| below this counts as idle
constexpr long long kDriveTail=250000;  // us after the last driven stick during which measured turns are rebased
std::atomic<bool> body_turn{true};  // permanent since 2026-10-23 (user: "the body turn works ... make it permanent")
std::atomic<float> turn_dead_zone{0.2618f};  // 15 degrees
std::atomic<long long> drive_until_us{0};
std::atomic<int> user_rx_milli{0};
std::atomic<float> turn_measured{0.f};  // camera thread adds rebased yaw; input thread reads it for the sign check
std::atomic<int> turn_sign{1};          // +1: positive stick X increases the game yaw (turns right)
std::atomic<unsigned> turn_flips{0};
struct TurnStats { unsigned rebased=0,drive_calls=0; float total=0,max_rel=0,last_stick=0; };
TurnStats turn_stats;                 // room_mutex
struct RoomStats { unsigned calls=0,requests=0,steps=0,blocked=0; float requested=0,consumed=0,max_offset=0; };
std::mutex room_mutex;
RoomStats room_stats;
thread_local Inject tl_inject;
struct NudgeRec { unsigned call=0; long long us=0; DWORD tid=0; bool wrote=false; Inject inj; Snap pre,post; };
std::atomic<bool> nudge_armed{false};
std::atomic<bool> shot_pending{false};
float shot_d[3]{};             // set before shot_pending is released
std::atomic<unsigned> follow_left{0};
std::mutex nudge_mutex;
NudgeRec nudge_recs[64]; unsigned nudge_count=0;
unsigned shots=0;

std::uintptr_t __fastcall integrate_detour(void* proxy,void* step,void* a3,void* a4,void* a5,void* a6,std::uintptr_t a7) {
  Inject& j=tl_inject;
  if (!j.on || proxy!=j.proxy) return original_integrate(proxy,step,a3,a4,a5,a6,a7);
  ++j.integrates;
  if (j.done) return original_integrate(proxy,step,a3,a4,a5,a6,a7);
  auto* p=static_cast<unsigned char*>(proxy);
  float h=0;
  if (!game::mem_read(static_cast<unsigned char*>(step)+8,&h,4) || !std::isfinite(h) || h<1e-4f || h>0.2f ||
      !game::mem_read(p+0xB0,j.v_game,12) || !game::mem_read(p+0x80,j.pos_pre,12)) {
    j.done=true; return original_integrate(proxy,step,a3,a4,a5,a6,a7);
  }
  j.h=h; j.done=true;
  if (j.roomscale) {
    const float follow=std::fmin(1.f,h/kFollowTau);
    j.d[0]*=follow; j.d[2]*=follow;
    const float len=std::sqrt(j.d[0]*j.d[0]+j.d[2]*j.d[2]), cap=kMaxSpeed*h;
    if (len>cap) { j.d[0]*=cap/len; j.d[2]*=cap/len; }
  }
  const float inj[3]={j.d[0]/h,0.f,j.d[2]/h};
  const float v[3]={j.v_game[0]+inj[0],j.v_game[1],j.v_game[2]+inj[2]};
  const bool wrote=game::mem_write(p+0xB0,v,12);
  const std::uintptr_t r=original_integrate(proxy,step,a3,a4,a5,a6,a7);
  if (!wrote) return r;
  game::mem_read(p+0xB0,j.v_post,12); game::mem_read(p+0x80,j.pos_post,12);
  float f[3]={j.v_post[0]-inj[0],j.v_post[1],j.v_post[2]-inj[2]};
  const float len=std::sqrt(j.d[0]*j.d[0]+j.d[2]*j.d[2]);
  const float ux=j.d[0]/len, uz=j.d[2]/len;
  const float along=f[0]*ux+f[2]*uz, floor=std::fmin(0.f,j.v_game[0]*ux+j.v_game[2]*uz);
  if (along<floor) { f[0]+=(floor-along)*ux; f[2]+=(floor-along)*uz; }  // collision ate the injection: no bounce back
  std::memcpy(j.v_final,f,sizeof(f));
  game::mem_write(p+0xB0,f,12);
  if (j.roomscale) {
    // Our share of this step's horizontal motion; capped so a game-side collision correction is never credited as
    // a large head move (the remainder stays in the view offset and is requested again next call).
    float c[3]={(j.pos_post[0]-j.pos_pre[0])-j.v_game[0]*h,0.f,(j.pos_post[2]-j.pos_pre[2])-j.v_game[2]*h};
    const float clen=std::sqrt(c[0]*c[0]+c[2]*c[2]), cmax=1.25f*std::sqrt(j.d[0]*j.d[0]+j.d[2]*j.d[2])+0.005f;
    if (clen>cmax) { c[0]*=cmax/clen; c[2]*=cmax/clen; }
    if (std::isfinite(c[0]) && std::isfinite(c[2])) {
      std::memcpy(j.consumed,c,sizeof(c));
      native_stereo::rebase_head(c);
    }
  }
  return r;
}

void roomscale_call(void* p) {
  const auto cc=ptr(ptr(p,0x28),0xA8), m=ptr(p,0x18);
  const Snap pre=snap(p,cc,m);
  float off[3], pend[3]{};
  bool request=false; float len=0;
  if (integrate_enabled && branch(pre)==3 && lua_host::writes_flag() && *lua_host::writes_flag() &&
      !mount_state::mounted() && native_stereo::head_offset(off)) {  // never push the rider's capsule on a mount
    native_stereo::pending_rebase(pend);
    const float rx=off[0]-pend[0], rz=off[2]-pend[2];
    len=std::sqrt(rx*rx+rz*rz);
    if (std::isfinite(len) && len>kDeadZone && len<3.f) {  // >3 m: stale/teleport/recentre pending; leave it alone
      const float s=(len-kDeadZone)/len;
      tl_inject=Inject{};
      tl_inject.on=true; tl_inject.roomscale=true; tl_inject.proxy=ptr(cc,0x2B8);
      tl_inject.d[0]=rx*s; tl_inject.d[2]=rz*s;  // the excess; scaled by h/kFollowTau at the step
      request=true;
    }
  }
  original_update(p);
  Inject j{};
  if (request) { j=tl_inject; tl_inject.on=false; }
  std::unique_lock<std::mutex> lock(room_mutex,std::try_to_lock);
  if (!lock.owns_lock()) return;
  ++room_stats.calls;
  if (len>room_stats.max_offset) room_stats.max_offset=len;
  if (!request) return;
  ++room_stats.requests;
  if (j.h<=0) return;
  ++room_stats.steps;
  const float req=std::sqrt(j.d[0]*j.d[0]+j.d[2]*j.d[2]), got=std::sqrt(j.consumed[0]*j.consumed[0]+j.consumed[2]*j.consumed[2]);
  room_stats.requested+=req; room_stats.consumed+=got;
  if (got<0.5f*req) ++room_stats.blocked;
}

void nudge_push(const NudgeRec& r) {
  std::lock_guard<std::mutex> lock(nudge_mutex);
  if (nudge_count<64) nudge_recs[nudge_count++]=r;
}
void nudge_call(void* p) {
  const auto cc=ptr(ptr(p,0x28),0xA8), m=ptr(p,0x18);
  NudgeRec r; r.us=now_us(); r.tid=GetCurrentThreadId(); r.pre=snap(p,cc,m);
  if (shot_pending.exchange(false)) {
    if (integrate_enabled && branch(r.pre)==3 && lua_host::writes_flag() && *lua_host::writes_flag()) {
      tl_inject=Inject{};
      tl_inject.on=true; tl_inject.proxy=ptr(cc,0x2B8);
      tl_inject.d[0]=shot_d[0]; tl_inject.d[2]=shot_d[2];
    }
    r.call=0; follow_left.store(kFollow);
  } else {
    r.call=kFollow+1-follow_left.fetch_sub(1);
  }
  original_update(p);
  if (tl_inject.on) { r.inj=tl_inject; r.wrote=tl_inject.h>0; tl_inject.on=false; }
  r.post=snap(p,cc,m);
  nudge_push(r);
}

void __fastcall update_detour(void* p) {
  if ((shot_pending.load(std::memory_order_relaxed) || follow_left.load(std::memory_order_relaxed)) &&
      local_body.load(std::memory_order_relaxed) && ptr(p,0x38)==local_body.load(std::memory_order_relaxed)) {
    nudge_call(p); return;
  }
  if (roomscale_armed.load(std::memory_order_relaxed) && !armed() && local_body.load(std::memory_order_relaxed) &&
      ptr(p,0x38)==local_body.load(std::memory_order_relaxed)) {
    roomscale_call(p); return;
  }
  if (!armed()) { original_update(p); return; }
  calls_all.fetch_add(1,std::memory_order_relaxed);
  const void* body=local_body.load(std::memory_order_relaxed);
  if (!body || ptr(p,0x38)!=body) { original_update(p); return; }
  calls_local.fetch_add(1,std::memory_order_relaxed);
  Record r;
  r.kind=Move; r.seq=seq.fetch_add(1); r.tid=GetCurrentThreadId(); r.us=now_us(); r.present=present_index.load();
  r.p=p; r.cc=ptr(ptr(p,0x28),0xA8); r.m=ptr(p,0x18);
  const auto cc=static_cast<const unsigned char*>(r.cc), m=static_cast<const unsigned char*>(r.m);
  r.pre=snap(p,cc,m);
  original_update(p);
  r.post=snap(p,cc,m);
  push(r);
}

void log(const char* line) {
  runtime_log::append(dir+L"\\MOVE_PROBE.log", {line, "\r\n"});
}
void csv(const std::string& text,bool truncate) {
  HANDLE file=CreateFileW((dir+L"\\MOVE_PROBE.csv").c_str(),truncate?GENERIC_WRITE:FILE_APPEND_DATA,FILE_SHARE_READ,nullptr,
                          truncate?CREATE_ALWAYS:OPEN_ALWAYS,0,nullptr);
  if (file==INVALID_HANDLE_VALUE) return;
  DWORD n; WriteFile(file,text.data(),static_cast<DWORD>(text.size()),&n,nullptr); CloseHandle(file);
}
void append_snap(std::string& out,const Snap& s) {
  char b[512];
  std::snprintf(b,sizeof(b),",%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%u,%u,%u,%d",
    s.ok,s.body[0],s.body[1],s.body[2],s.target[0],s.target[1],s.target[2],s.proxy[0],s.proxy[1],s.proxy[2],
    s.vel[0],s.vel[1],s.vel[2],s.cc_pos[0],s.cc_pos[1],s.cc_pos[2],s.step,s.acc,s.f14,s.fcc,s.f36a,s.swim);
  out+=b;
}

bool install() {
  if (created) return true;
  if (failed) return false;
  unsigned char* target=game::module_base()+kMoveUpdateRva;
  unsigned char got[sizeof(kMoveUpdateSig)]{};
  char line[160];
  if (!game::mem_read(target,got,sizeof(got)) || std::memcmp(got,kMoveUpdateSig,sizeof(got))!=0) {
    log("movement update guard mismatch at +0x4271680 (different build?); probe not installed"); failed=true; return false;
  }
  const MH_STATUS c=MH_CreateHook(target,reinterpret_cast<void*>(&update_detour),reinterpret_cast<void**>(&original_update));
  std::snprintf(line,sizeof(line),"movement update hook create +0x4271680 status=%d",static_cast<int>(c)); log(line);
  if (c!=MH_OK) { failed=true; return false; }
  created=true; return true;
}
void set_enabled(bool want) {
  if (!created || enabled==want) return;
  void* target=game::module_base()+kMoveUpdateRva;
  const MH_STATUS s=want?MH_EnableHook(target):MH_DisableHook(target);
  if (s==MH_OK) enabled=want;
  char line[96]; std::snprintf(line,sizeof(line),"movement update hook %s status=%d",want?"enable":"disable",static_cast<int>(s)); log(line);
}
// The integrate hook exists only for the nudge test (every character's proxy runs through it; the detour passes
// through unless this thread is inside a local shot call).
void set_integrate(bool want,char* why,std::size_t n) {
  void* target=game::module_base()+kIntegrateRva;
  if (want && !integrate_created) {
    unsigned char got[sizeof(kIntegrateSig)]{};
    if (!game::mem_read(target,got,sizeof(got)) || std::memcmp(got,kIntegrateSig,sizeof(got))!=0) {
      std::snprintf(why,n,"integrate guard mismatch at +0x3451EA0; shots will not write"); return;
    }
    const MH_STATUS c=MH_CreateHook(target,reinterpret_cast<void*>(&integrate_detour),reinterpret_cast<void**>(&original_integrate));
    if (c!=MH_OK) { std::snprintf(why,n,"integrate hook create status=%d",static_cast<int>(c)); return; }
    integrate_created=true;
  }
  if (!integrate_created || integrate_enabled==want) return;
  const MH_STATUS s=want?MH_EnableHook(target):MH_DisableHook(target);
  if (s==MH_OK) integrate_enabled=want;
  std::snprintf(why,n,"integrate hook %s status=%d",want?"enable":"disable",static_cast<int>(s));
}

void drain() {
  static Record pending[1024];  // present thread only; too large for its stack
  unsigned n=0;
  {std::lock_guard<std::mutex> lock(mutex); n=count; std::memcpy(pending,records,n*sizeof(Record)); count=0;}
  if (!n) return;
  std::string out; out.reserve(n*420);
  static const char* kinds[]={"move","cam_pre","cam_post"};
  char b[256];
  for (unsigned i=0;i<n;++i) {
    const auto& r=pending[i];
    std::snprintf(b,sizeof(b),"%s,%llu,%lu,%lld,%u,%p,%p,%p,%u",kinds[r.kind],r.seq,r.tid,r.us,r.present,r.p,r.cc,r.m,
                  r.kind==Move?branch(r.pre):0u);
    out+=b;
    if (r.kind==Move) { append_snap(out,r.pre); append_snap(out,r.post); }
    out+="\r\n";
  }
  csv(out,false);
}

void finish() {
  deadline.store(0);
  if (!nudge_armed.load()) set_enabled(false);
  drain();
  char line[512];
  std::lock_guard<std::mutex> lock(mutex);
  std::snprintf(line,sizeof(line),"window complete records=%u calls_all=%u calls_local=%u camera_events=%u dropped=%u presents=%u "
                "branch unread=%u scripted=%u teleport=%u integrated=%u swimming=%u threads=%u",
                total,calls_all.load(),calls_local.load(),camera_events.load(),dropped.load(),present_index.load(),
                branches[0],branches[1],branches[2],branches[3],branches[4],thread_count);
  log(line);
  for (unsigned i=0;i<thread_count;++i) { std::snprintf(line,sizeof(line),"movement thread %lu",threads[i]); log(line); }
}

void nudge_log(const char* line) {
  runtime_log::append(dir+L"\\MOVE_NUDGE.log", {line, "\r\n"});
}
void nudge_drain() {
  static NudgeRec pending[64];  // present thread only
  unsigned n=0;
  {std::lock_guard<std::mutex> lock(nudge_mutex); n=nudge_count; std::memcpy(pending,nudge_recs,n*sizeof(NudgeRec)); nudge_count=0;}
  char line[1024];
  for (unsigned i=0;i<n;++i) {
    const auto& r=pending[i];
    const auto& j=r.inj;
    const float ax=r.post.target[0]-r.pre.target[0], az=r.post.target[2]-r.pre.target[2];
    const float along=(j.d[0]*ax+j.d[2]*az)/(kNudge*kNudge);  // achieved move along d, as a fraction of 10 cm
    std::snprintf(line,sizeof(line),
      "%s call=%u us=%lld tid=%lu branch=%u wrote=%d d=(%.4f,%.4f) h=%.5f integrates=%u body_pre=(%.4f,%.4f,%.4f) "
      "target_pre=(%.4f,%.4f,%.4f) target_post=(%.4f,%.4f,%.4f) body_post=(%.4f,%.4f,%.4f) moved_h=%.4f along_d=%.3f "
      "vel_post=(%.3f,%.3f,%.3f) v_game=(%.3f,%.3f,%.3f) v_after_step=(%.3f,%.3f,%.3f) v_restored=(%.3f,%.3f,%.3f) "
      "proxy_step=(%.4f,%.4f,%.4f)",
      r.call==0?"SHOT":"follow",r.call,r.us,r.tid,branch(r.pre),r.wrote,j.d[0],j.d[2],j.h,j.integrates,
      r.pre.body[0],r.pre.body[1],r.pre.body[2],r.pre.target[0],r.pre.target[1],r.pre.target[2],
      r.post.target[0],r.post.target[1],r.post.target[2],r.post.body[0],r.post.body[1],r.post.body[2],
      std::sqrt(ax*ax+az*az),r.call==0?along:0.f,r.post.vel[0],r.post.vel[1],r.post.vel[2],
      j.v_game[0],j.v_game[1],j.v_game[2],j.v_post[0],j.v_post[1],j.v_post[2],j.v_final[0],j.v_final[1],j.v_final[2],
      j.pos_post[0]-j.pos_pre[0],j.pos_post[1]-j.pos_pre[1],j.pos_post[2]-j.pos_pre[2]);
    nudge_log(line);
  }
}
void room_log(const char* line) {
  runtime_log::append(dir+L"\\MOVE_ROOMSCALE.log", {line, "\r\n"});
}
bool control_read=false;  // the control file has been polled at least once (present thread)
// Whole-word match ("on" must not match inside other words).
bool has_word(const char* text,const char* word) {
  const std::size_t n=std::strlen(word);
  const auto space=[](char c){ return c==0 || c==' ' || c=='\n' || c=='\r' || c=='\t'; };
  for (const char* s=std::strstr(text,word); s; s=std::strstr(s+1,word)) {
    if ((s==text || space(s[-1])) && space(s[n])) return true;
  }
  return false;
}
// VR menu options, persisted as words in AvatarVR_moveprobe.txt (2026-10-23): "nobodyturn" (body turn off; on by
// default, headset-accepted) and "headmove" (locomotion follows the head instead of the game camera / body). The menu
// writes the plain bools on the Present thread; other threads read the atomics (like virtual_pad's g_pitch_level).
bool menu_body_turn=true, menu_head_move=false;
bool applied_body_turn=true, applied_head_move=false, options_loaded=false;
std::atomic<bool> head_move{false};
void write_options(char* control,std::size_t size) {
  std::string out;
  const char* s=control;
  while (*s) {
    while (*s==' ' || *s=='\t' || *s=='\r' || *s=='\n') ++s;
    const char* e=s;
    while (*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n') ++e;
    const std::string w(s,e);
    if (!w.empty() && w!="bodyturn" && w!="nobodyturn" && w!="headmove") { if (!out.empty()) out+=' '; out+=w; }
    s=e;
  }
  if (!applied_body_turn) out+=out.empty()?"nobodyturn":" nobodyturn";
  if (applied_head_move) out+=out.empty()?"headmove":" headmove";
  if (out.size()>=size) return;
  if (!settings_file::write(dir+L"\\AvatarVR_moveprobe.txt",out)) return;
  std::memset(control,0,size); std::memcpy(control,out.data(),out.size());
}
// fresh: control was just read from the file. A menu change wins over the file and is written back.
void sync_options(char* control,std::size_t size,bool fresh) {
  const bool file_body=!has_word(control,"nobodyturn"), file_head=has_word(control,"headmove");
  if (!options_loaded) {
    if (!fresh) return;
    applied_body_turn=menu_body_turn=file_body; applied_head_move=menu_head_move=file_head; options_loaded=true;
  } else if (menu_body_turn!=applied_body_turn || menu_head_move!=applied_head_move) {
    applied_body_turn=menu_body_turn; applied_head_move=menu_head_move;
    write_options(control,size);
  } else if (fresh && (file_body!=applied_body_turn || file_head!=applied_head_move)) {
    applied_body_turn=menu_body_turn=file_body; applied_head_move=menu_head_move=file_head;
  }
  body_turn.store(applied_body_turn); head_move.store(applied_head_move);
}
void roomscale_present(const char* control,ULONGLONG now) {
  static ULONGLONG last_report=0;
  static std::uint64_t last_rebases=0;
  if (!control_read) return;
  // Permanent since 2026-10-23 (user: "it does work, let's make it permanent"): on unless the file says noroomscale.
  const bool want=std::strstr(control,"noroomscale")==nullptr;
  if (const char* t=std::strstr(control,"turndz=")) {
    const float deg=static_cast<float>(std::atof(t+7));
    if (deg>=0.f && deg<=90.f) turn_dead_zone.store(deg*0.0174532925f);
  }
  if (want!=roomscale_armed.load()) {
    char why[128]{};
    if (want) {
      if (!install()) return;
      set_enabled(true); set_integrate(true,why,sizeof(why));
      roomscale_armed.store(true);
      char line[256];
      std::snprintf(line,sizeof(line),"roomscale armed build " __DATE__ " " __TIME__ "; smooth follow beyond %.2f m, "
                    "tau %.2f s, max %.1f m/s; body turn %s, dead zone %.1f deg; Script writes required",kDeadZone,kFollowTau,kMaxSpeed,
                    body_turn.load()?"on":"off",turn_dead_zone.load()*57.2958f);
      room_log(line);
    } else {
      roomscale_armed.store(false);
      if (!nudge_armed.load()) { set_integrate(false,why,sizeof(why)); if (!deadline.load()) set_enabled(false); }
      room_log("roomscale disarmed");
    }
    if (why[0]) room_log(why);
    last_report=now;
  }
  if (!roomscale_armed.load()) return;
  local_body.store(game::pointers().body,std::memory_order_relaxed);
  if (now-last_report<1000) return;
  last_report=now;
  RoomStats s; TurnStats ts;
  {std::lock_guard<std::mutex> lock(room_mutex); s=room_stats; room_stats=RoomStats{}; ts=turn_stats; turn_stats=TurnStats{};}
  const std::uint64_t rebases=native_stereo::rebases_applied();
  float off[3]{}; const bool head=native_stereo::head_offset(off);
  float rel=0; const bool yaw_ok=native_stereo::head_yaw_relative(&rel);
  char line[480];
  std::snprintf(line,sizeof(line),"1s: calls=%u requests=%u steps=%u blocked=%u requested=%.3f m consumed=%.3f m "
                "max_offset=%.3f m view_rebases=%llu now_offset=%s(%.3f,%.3f) | turn %s: drive_calls=%u last_stick=%.2f "
                "rebased_frames=%u turned=%.1f deg max_head_yaw=%.1f deg now_head_yaw=%s%.1f deg sign=%+d flips=%u | writes=%d",
                s.calls,s.requests,s.steps,s.blocked,s.requested,s.consumed,s.max_offset,
                static_cast<unsigned long long>(rebases-last_rebases),head?"":"none",off[0],off[2],
                body_turn.load()?"on":"off",ts.drive_calls,ts.last_stick,ts.rebased,ts.total*57.2958f,ts.max_rel*57.2958f,
                yaw_ok?"":"none",rel*57.2958f,turn_sign.load(),turn_flips.load(),
                lua_host::writes_flag() && *lua_host::writes_flag());
  last_rebases=rebases;
  room_log(line);
}
bool game_focused() {
  DWORD pid=0; GetWindowThreadProcessId(GetForegroundWindow(),&pid);
  return pid==GetCurrentProcessId();
}
void nudge_present(const char* control,ULONGLONG now) {
  static bool key_was_down=false;
  static ULONGLONG last_shot=0;
  roomscale_present(control,now);
  const bool want=std::strstr(control,"nudge")!=nullptr;
  if (want!=nudge_armed.load()) {
    if (want && !install()) return;
    nudge_armed.store(want);
    char why[128]{};
    if (want) {
      set_enabled(true); set_integrate(true,why,sizeof(why));
      nudge_log("nudge armed build " __DATE__ " " __TIME__ "; F11 = one 10 cm shot along game camera forward, injected as "
                "velocity d/h at one Havok integrate and removed afterwards (Script writes required)");
    } else {
      shot_pending.store(false);
      if (!roomscale_armed.load()) { set_integrate(false,why,sizeof(why)); if (!deadline.load()) set_enabled(false); }
      nudge_log("nudge disarmed");
    }
    if (why[0]) nudge_log(why);
  }
  if (!nudge_armed.load()) return;
  local_body.store(game::pointers().body,std::memory_order_relaxed);
  const bool down=game_focused() && (GetAsyncKeyState(VK_F11)&0x8000)!=0;
  const bool edge=down && !key_was_down;
  key_was_down=down;
  if (edge) {
    const bool writes=lua_host::writes_flag() && *lua_host::writes_flag();
    const game::Player pl=game::read();
    const float fx=pl.cam[8], fz=pl.cam[10], len=std::sqrt(fx*fx+fz*fz);
    char line[200];
    if (!writes || !pl.valid || len<0.2f || shot_pending.load() || follow_left.load() || now-last_shot<1000) {
      std::snprintf(line,sizeof(line),"F11 ignored: writes=%d gameplay=%d fwd_len=%.2f busy=%d",writes,pl.valid,len,
                    shot_pending.load()||follow_left.load()||now-last_shot<1000);
      nudge_log(line); return;
    }
    shot_d[0]=fx/len*kNudge; shot_d[1]=0; shot_d[2]=fz/len*kNudge;
    last_shot=now; ++shots;
    shot_pending.store(true);
    std::snprintf(line,sizeof(line),"shot %u queued d=(%.4f,0,%.4f) body=(%.4f,%.4f,%.4f)",shots,shot_d[0],shot_d[2],pl.body.x,pl.body.y,pl.body.z);
    nudge_log(line);
  }
}
}  // namespace

namespace {
// Monotonic microseconds for the body turn (independent of the capture window's clock, which only starts when armed).
long long turn_clock_us() {
  static const long long freq=[]{ LARGE_INTEGER f; QueryPerformanceFrequency(&f); return f.QuadPart; }();
  LARGE_INTEGER t; QueryPerformanceCounter(&t);
  return t.QuadPart/freq*1000000+(t.QuadPart%freq)*1000000/freq;
}
// Camera update post phase (local holder): look +0x34 equals the camera's yaw this frame (CAMERA_FINDINGS, max error 0).
void body_turn_post() {
  static float prev=0.f; static bool have=false;
  const game::Player pl=game::read();
  if (!pl.valid || !pl.look_ok || !std::isfinite(pl.yaw)) { have=false; return; }
  float d=pl.yaw-prev; prev=pl.yaw;
  if (!have) { have=true; return; }
  if (d>3.14159265f) d-=6.2831853f; else if (d<-3.14159265f) d+=6.2831853f;
  if (!body_turn.load(std::memory_order_relaxed) || turn_clock_us()>=drive_until_us.load(std::memory_order_relaxed) ||
      std::abs(user_rx_milli.load(std::memory_order_relaxed))>static_cast<int>(kUserIdle*1000) ||
      std::fabs(d)>0.5f || d==0.f) return;
  native_stereo::rebase_yaw(d);
  turn_measured.fetch_add(d,std::memory_order_relaxed);
  std::unique_lock<std::mutex> lock(room_mutex,std::try_to_lock);
  if (lock.owns_lock()) { ++turn_stats.rebased; turn_stats.total+=std::fabs(d); }
}
}  // namespace

namespace move_probe {
bool* body_turn_flag() { return &menu_body_turn; }
bool* head_move_flag() { return &menu_head_move; }

// XInputGetState thread (virtual pad), VR gameplay. "Movement direction: Head": the game moves the body relative to
// its camera (body) yaw, so the left stick is rotated by the headset's yaw relative to that camera (engine convention,
// + = right; stick x right, y forward): forward on the stick goes where the head looks.
void head_move_stick(short* lx,short* ly) {
  if (!head_move.load(std::memory_order_relaxed) || lx==nullptr || ly==nullptr || (*lx==0 && *ly==0)) return;
  float rel=0;
  if (!native_stereo::head_yaw_relative(&rel) || !std::isfinite(rel) || std::fabs(rel)>2.5f) return;
  const float x=*lx/32767.f, y=*ly/32767.f, c=std::cos(rel), s=std::sin(rel);
  const float nx=std::clamp(x*c+y*s,-1.f,1.f), ny=std::clamp(-x*s+y*c,-1.f,1.f);
  *lx=static_cast<short>(nx*32767.f); *ly=static_cast<short>(ny*32767.f);
}

// XInputGetState thread (virtual pad), after pitch levelling.
float body_turn_stick(float user_rx) {
  static bool driving=false;
  static int calls=0;
  user_rx_milli.store(static_cast<int>(user_rx*1000.f),std::memory_order_relaxed);
  const auto stop=[&]{ driving=false; return user_rx; };
  // Mounted (FEATURE-06, 2026-10-24 [AUTHOR]): the head looks around freely; the stick steers the mount.
  if (!body_turn.load(std::memory_order_relaxed) || !roomscale_armed.load(std::memory_order_relaxed) ||
      !lua_host::writes_flag() || !*lua_host::writes_flag() || mount_state::mounted()) return stop();
  if (std::fabs(user_rx)>kUserIdle) { drive_until_us.store(0,std::memory_order_relaxed); return stop(); }  // user turns
  float rel=0;
  if (!native_stereo::head_yaw_relative(&rel) || !std::isfinite(rel)) return stop();
  const float dz=turn_dead_zone.load(std::memory_order_relaxed), halt=std::fmax(0.f,dz-kTurnHyst), a=std::fabs(rel);
  {std::unique_lock<std::mutex> lock(room_mutex,std::try_to_lock); if (lock.owns_lock() && a>turn_stats.max_rel) turn_stats.max_rel=a;}
  if (!driving && a>dz && a<2.5f) { driving=true; calls=0; turn_measured.store(0.f); }
  if (driving && (a<=halt || a>=2.5f)) return stop();
  if (!driving) return user_rx;
  const float toward=rel>0.f?1.f:-1.f;  // + = head right of the camera: the game must turn right
  if (++calls>=12) {
    // Driven for a whole window but the camera turned away from the head: the stick sign is the other way.
    if (turn_measured.exchange(0.f)*toward < -0.0087f) { turn_sign.store(-turn_sign.load()); turn_flips.fetch_add(1); }
    calls=0;
  }
  const float mag=std::clamp(kTurnGain*(a-halt),kTurnMinStick,kTurnMaxStick);
  const float stick=static_cast<float>(turn_sign.load(std::memory_order_relaxed))*toward*mag;
  drive_until_us.store(turn_clock_us()+kDriveTail,std::memory_order_relaxed);
  std::unique_lock<std::mutex> lock(room_mutex,std::try_to_lock);
  if (lock.owns_lock()) { ++turn_stats.drive_calls; turn_stats.last_stick=stick; }
  return stick;
}

void on_camera(bool post) {
  if (post) body_turn_post();
  if (!armed()) return;
  camera_events.fetch_add(1,std::memory_order_relaxed);
  Record r; r.kind=post?CamPost:CamPre; r.seq=seq.fetch_add(1); r.tid=GetCurrentThreadId(); r.us=now_us();
  r.present=present_index.load();
  push(r);
}

void on_present() {
  if (dir.empty()) {
    HMODULE self=nullptr; wchar_t path[MAX_PATH]{};
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&on_present),&self);
    if (!GetModuleFileNameW(self,path,MAX_PATH)) return;
    dir=path; dir=dir.substr(0,dir.find_last_of(L"\\/"));
  }
  if (deadline.load()) {
    present_index.fetch_add(1,std::memory_order_relaxed);
    local_body.store(game::pointers().body,std::memory_order_relaxed);
  }
  const auto now=GetTickCount64();
  static char control[128]{};
  sync_options(control,sizeof(control),false);  // menu changes
  nudge_present(control,now);  // every present: key edge
  nudge_drain();
  if (now-last_poll<1000) return;
  last_poll=now;
  std::memset(control,0,sizeof(control));
  settings_file::read(dir+L"\\AvatarVR_moveprobe.txt",control,sizeof(control));
  control_read=true;
  sync_options(control,sizeof(control),true);
  const bool requested=has_word(control,"on") || has_word(control,"capture");
  if (requested && !on && game::read().valid) {
    on=true;
    if (install()) {
      QueryPerformanceFrequency(&qpf); QueryPerformanceCounter(&qpc0);
      {std::lock_guard<std::mutex> lock(mutex); count=total=thread_count=0; std::memset(branches,0,sizeof(branches));}
      calls_all=calls_local=dropped=camera_events=0; present_index=0; seq=0;
      local_body.store(game::pointers().body);
      csv("kind,seq,tid,us,present,P,CC,M,branch"
          ",pre_ok,pre_body_x,pre_body_y,pre_body_z,pre_target_x,pre_target_y,pre_target_z,pre_proxy_x,pre_proxy_y,pre_proxy_z"
          ",pre_vel_x,pre_vel_y,pre_vel_z,pre_cc_x,pre_cc_y,pre_cc_z,pre_step,pre_acc,pre_f14,pre_fcc,pre_f36a,pre_swim"
          ",post_ok,post_body_x,post_body_y,post_body_z,post_target_x,post_target_y,post_target_z,post_proxy_x,post_proxy_y,post_proxy_z"
          ",post_vel_x,post_vel_y,post_vel_z,post_cc_x,post_cc_y,post_cc_z,post_step,post_acc,post_f14,post_fcc,post_f36a,post_swim\r\n",true);
      log("move probe build " __DATE__ " " __TIME__ "; 20s read-only window; local body only; max 6000 records");
      set_enabled(true);
      deadline.store(now+kWindowMs);
    }
  }
  if (!requested) { if (deadline.load()) finish(); on=false; }
  drain();
  const auto until=deadline.load();
  if (until && now>=until) finish();
}
}  // namespace move_probe
