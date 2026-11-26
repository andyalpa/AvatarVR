#include <sstream>
#include "../settings_file.hpp"
#include "ui_ownership.hpp"
#include "native_stereo.hpp"
#include "mount_camera.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../vr/wrist_hud.hpp"
#include "../vr/xr_runtime.hpp"
#include "../vr/vr_weapon.hpp"
#include "../vr/vr_recoil.hpp"
#include <windows.h>
#include <MinHook.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

// Supported build 98588DA2; derivation/guards: research/HUD_NATIVE_ATTACHMENT_20261028.md.
// No engine pointer is retained for later dereference. File I/O is exclusively on Present.
namespace ui_ownership {
namespace {
using Callback = void (*)(void*, void*, void*, void*);
using Command = void* (*)(void*, unsigned);
using Name = void (*)(void*, void*, const char*);
using Slot = unsigned (*)();
Callback original_callback = nullptr;
Command original_command = nullptr;
Name original_name = nullptr;
Slot slot_index = nullptr;
unsigned char* module = nullptr;
Log logger = nullptr;
std::atomic<bool> prepared{false}, installed{false}, allowed{true}, requested{false}, stopped{false};
// 0 idle, 1 waiting for a selected gameplay query, 2 recording, 3 draining.
std::atomic<unsigned> phase{0}, capture_id{0};
std::atomic<ULONGLONG> started{0};
std::atomic<std::uint64_t> present_epoch{0}, serial{0};
std::atomic<unsigned> contention{0}, overflow{0}, throttled{0}, stack_overflow{0}, repeats{0};
constexpr unsigned QueueSize = 512, MaxEvents = 14400, Depth = 16, Ancestry = 6, SeenSize = 2048;
constexpr ULONGLONG CaptureMs = 30000, WaitMs = 90000;
struct Event {
  char kind=0;
  unsigned session=0, burst=0, tid=0, depth=0, id=~0u, value=0, flags=0, tag=0;
  int eye=0, pass=0;
  bool menu=false, local_read=false, world_read=false, inherited_graph=false;
  std::uint64_t order=0, scope=0, parent_scope=0, epoch=0, qpc=0, elapsed=0;
  std::uintptr_t node=0, node_type=0, params=0, context=0, widget=0, record=0, caller=0;
  std::uintptr_t graph=0, graph_context=0, cache=0, ancestry[Ancestry]{};
  std::uintptr_t execution_resource=0, pin_descriptor=0, pin_payload=0;
  std::uint64_t scope_chain[Depth]{};
  unsigned scope_count=0, graph_epoch=0;
  unsigned ancestry_count=0, matrix_flags=0;
  float local[16]{}, world[16]{};
  char name[80]{};
};
Event queue[QueueSize];
unsigned head=0, count=0, total=0, burst=~0u, quotas[5]{};
// Per-burst first sightings, so busy widgets cannot use up the quota before later widgets in the walk are seen.
struct Seen { std::uintptr_t widget; unsigned char kinds; };
Seen seen[SeenSize]{};
std::mutex mutex;
FILE* output = nullptr; // Present only
ULONGLONG waiting_at=0;
unsigned summaries=0;
char status_label[96]="HUD ownership: idle"; // menu stores this pointer once; Present refreshes its contents
struct Scope {
  unsigned session=0;
  std::uint64_t id=0, parent=0;
  std::uintptr_t graph=0, resource=0;
  unsigned graph_epoch=0;
};
thread_local Scope scopes[Depth];
thread_local unsigned nesting=0;

bool read(const void* p, void* dst, size_t bytes) {
  if (!p) return false;
  __try { std::memcpy(dst,p,bytes); return true; }
  __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
}
template<class T> bool field(const void* p,size_t offset,T* dst) {
  return p && read(static_cast<const unsigned char*>(p)+offset,dst,sizeof(T));
}
std::uintptr_t address(const void* p) { return reinterpret_cast<std::uintptr_t>(p); }
bool selected(unsigned id) { return id==0 || id==7 || id==12 || id==45 || id==63; }
void* context() {
  if (!slot_index || !module) return nullptr;
  const unsigned slot=slot_index();
  void* ctx=nullptr;
  if (slot<256) field(module,0x96AB220+static_cast<size_t>(slot)*8,&ctx);
  return ctx;
}
bool recording() { return phase.load(std::memory_order_relaxed)==2; }
bool sample_window(ULONGLONG now) {
  const auto begin=started.load(std::memory_order_relaxed);
  return begin && now>=begin && now-begin<CaptureMs && (now-begin)%1000<50;
}
Scope current_scope() {
  if (!nesting || nesting>Depth) return {};
  const auto s=scopes[nesting-1];
  return s.session==capture_id.load(std::memory_order_relaxed)?s:Scope{};
}
// Mutex held. False only for a repeat; a full probe admits, leaving the global quota as the bound.
bool first_sighting(std::uintptr_t widget,unsigned kind) {
  const auto key=widget?widget:1;
  const unsigned char bit=static_cast<unsigned char>(1u<<kind);
  auto slot=static_cast<unsigned>((key>>4)^(key>>16));
  for (unsigned probe=0;probe<16;++probe,++slot) {
    auto& s=seen[slot%SeenSize];
    if (!s.widget) { s.widget=key; s.kinds=bit; return true; }
    if (s.widget!=key) continue;
    if (s.kinds&bit) return false;
    s.kinds|=bit; return true;
  }
  return true;
}
bool claim(unsigned kind,ULONGLONG now,std::uintptr_t owner=0) {
  if (!recording()) return false;
  const auto begin=started.load(std::memory_order_relaxed);
  // Short bursts permit graph evaluation outside OnDrawUI too. A burst/Present epoch is NOT an eye frame ID.
  if (!sample_window(now)) return false;
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
  if (!lock.owns_lock()) { ++contention; return false; }
  if (total>=MaxEvents) { phase.store(3); return false; }
  const unsigned b=static_cast<unsigned>((now-begin)/1000);
  if (burst!=~0u && b<burst) { ++throttled; return false; } // a delayed worker cannot reopen an old quota
  if (burst!=b) { burst=b; std::memset(quotas,0,sizeof(quotas)); std::memset(seen,0,sizeof(seen)); }
  const unsigned limits[]={64,128,128,128,32}; // queries, callback entry/exit, commands, names per burst
  if (quotas[kind]>=limits[kind]) { ++throttled; return false; }
  if (kind>=1 && kind<=3 && !first_sighting(owner,kind)) { ++repeats; return false; }
  ++quotas[kind];
  return true;
}
Event snapshot(char kind,void* node,void* params,void* ctx,void* widget) {
  Event e;
  e.kind=kind; e.session=capture_id.load(); e.tid=GetCurrentThreadId(); e.depth=nesting;
  e.order=serial.fetch_add(1)+1; e.epoch=present_epoch.load();
  LARGE_INTEGER stamp; QueryPerformanceCounter(&stamp); e.qpc=stamp.QuadPart;
  const auto begin=started.load(),now=GetTickCount64(); e.elapsed=now>=begin?now-begin:0;
  e.burst=static_cast<unsigned>(e.elapsed/1000);
  const auto scope=current_scope(); e.scope=scope.id; e.parent_scope=scope.parent;
  if (nesting<=Depth) for (unsigned i=0;i<nesting;++i)
    if (scopes[i].session==e.session) e.scope_chain[e.scope_count++]=scopes[i].id;
  if (nesting>Depth) e.flags|=16; // truncated stack has no trustworthy current scope
  e.eye=native_stereo::ui_pass_eye(); e.pass=native_stereo::ui_pass_kind();
  e.menu=native_stereo::ui_layer_active();
  e.node=address(node); e.params=address(params); e.context=address(ctx); e.widget=address(widget);
  field(node,0,&e.node_type);
  field(params,0x828,&e.graph); field(params,0x830,&e.graph_context); field(params,0x840,&e.cache);
  if (e.graph) {
    field(reinterpret_cast<void*>(e.graph),0x50,&e.execution_resource);
    field(reinterpret_cast<void*>(e.graph),0x4C,&e.graph_epoch);
  } else if (!params && scope.id) {
    e.graph=scope.graph; e.execution_resource=scope.resource; e.graph_epoch=scope.graph_epoch;
    e.inherited_graph=true; // callback's context, not a claim that the leaf command executed this graph
  }
  void* ancestor=widget;
  while (ancestor && e.ancestry_count<Ancestry) {
    bool cycle=false;
    for (unsigned i=0;i<e.ancestry_count;++i) if (e.ancestry[i]==address(ancestor)) cycle=true;
    if (cycle) { e.flags|=1; break; }
    e.ancestry[e.ancestry_count++]=address(ancestor);
    void* parent=nullptr;
    if (!field(ancestor,0x150,&parent)) { e.flags|=2; break; }
    ancestor=parent;
  }
  if (ancestor && e.ancestry_count==Ancestry) e.flags|=4;
  e.local_read=widget && read(static_cast<unsigned char*>(widget)+0x20,e.local,sizeof(e.local));
  e.world_read=widget && read(static_cast<unsigned char*>(widget)+0x60,e.world,sizeof(e.world));
  // Store cache flags as observed bytes; do not force recomputation or call the matrix getter.
  field(widget,0x17D,&e.matrix_flags);
  return e;
}
void enqueue(const Event& e) {
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
  if (!lock.owns_lock()) { ++contention; return; }
  if (!recording() || e.session!=capture_id.load()) return;
  if (count==QueueSize) { ++overflow; return; }
  if (total>=MaxEvents) { phase.store(3); return; }
  queue[(head+count)%QueueSize]=e; ++count; ++total;
  if (total==MaxEvents) phase.store(3);
}
void Callback_hook(void* node,void* params,void* ctx,void* record) {
  if (!recording()) { original_callback(node,params,ctx,record); return; }
  const unsigned old_depth=nesting;
  Scope s; s.session=capture_id.load(); s.parent=current_scope().id; s.id=serial.fetch_add(1)+1;
  // Extra graph reads stay inside the sampled window, even though scope tracking spans callbacks.
  if (sample_window(GetTickCount64())) field(params,0x828,&s.graph);
  if (s.graph) {
    field(reinterpret_cast<void*>(s.graph),0x50,&s.resource);
    field(reinterpret_cast<void*>(s.graph),0x4C,&s.graph_epoch);
  }
  if (nesting<Depth) scopes[nesting]=s; else ++stack_overflow;
  ++nesting;
  if (sample_window(GetTickCount64())) {
    void* widget=nullptr; field(ctx,0x68,&widget);
    if (claim(1,GetTickCount64(),address(widget))) {
      auto e=snapshot('W',node,params,ctx,widget); e.record=address(record); enqueue(e);
    }
  }
  original_callback(node,params,ctx,record);
  if (capture_id.load()==s.session && sample_window(GetTickCount64())) {
    void* widget=nullptr; field(ctx,0x68,&widget);
    if (claim(2,GetTickCount64(),address(widget))) {
      auto e=snapshot('X',node,params,ctx,widget); e.record=address(record); enqueue(e);
    }
  }
  nesting=old_depth;
}
void* Command_hook(void* widget,unsigned tag) {
  void* result=original_command(widget,tag);
  if (recording() && claim(3,GetTickCount64(),address(widget))) {
    auto e=snapshot('C',nullptr,nullptr,context(),widget); e.record=address(result); e.tag=tag;
    enqueue(e);
  }
  return result;
}
// Native widget test (2026-10-30, opt-in from F10, not saved). 0x527390 walks one widget subtree and 0x527430 emits each
// widget's commands with its 4x4 matrix, perspective included. Only arguments change: no widget memory is written.
using Walk = void (*)(void*, void*, void*, void*);
using Emit = void (*)(void*, void*, void*, float*, void*);
Walk original_walk = nullptr;
Emit original_emit = nullptr;
// 7 logs draws per UI layer; 8-14 also send the health group to one other layer, to find layers that can be drawn per eye.
constexpr unsigned TestModes = 15; // 0 off; health 1 move 2 tilt 3 hide; ammo 4 move 5 tilt 6 hide
constexpr unsigned RouteLayers[7] = {1,2,3,5,6,7,8};
const char* const test_labels[TestModes] = {"Native widget test: off", "Native widget test: 1 move health", "Native widget test: 2 tilt health",
  "Native widget test: 3 hide health", "Native widget test: 4 move ammo", "Native widget test: 5 tilt ammo", "Native widget test: 6 hide ammo",
  "Native widget test: 7 layer log", "Native widget test: 8 health layer 1", "Native widget test: 9 health layer 2",
  "Native widget test: 10 health layer 3", "Native widget test: 11 health layer 5", "Native widget test: 12 health layer 6",
  "Native widget test: 13 health layer 7", "Native widget test: 14 health layer 8"};
std::atomic<unsigned> test_mode{0}, test_requested{0};
std::atomic<unsigned> test_selected{0}, test_emitted{0}, test_hidden{0};
std::atomic<void*> test_health[8];  // widgets are recycled: newest names overwrite the oldest
std::atomic<void*> test_energy[8];
std::atomic<unsigned> name_cursor[2];
thread_local bool t_test_inside = false;
using Build = void (*)(void*);
using Layer = void (*)(void*, void*, int, char, char);
Build original_build = nullptr;
Layer original_layer = nullptr;
thread_local unsigned char* t_list_base = nullptr;  // first widget list of the running UI build
thread_local int t_layer = -1;
std::atomic<unsigned> layer_draws[12][3];
std::atomic<unsigned> test_routed{0};
// Native HUD on the arm (2026-10-30). The UI is flattened once per build, so each selected group is flattened twice: left-eye
// matrix into layer 8, right-eye into layer 7. Layer 8 draws in the UI Normal pass; in the right eye its slot draws layer 7
// instead (same target class, otherwise only a main-view caller that has no art of its own). camera.cpp stretches these two
// layers to the whole eye, so canvas pixels are the eye's frustum and the matrix is that eye's projection of an arm panel.
constexpr int ArmLayers[2]={8,7};
std::atomic<bool> arm_on{false};
std::atomic<unsigned> arm_builds{0},arm_groups{0},arm_fallback{0},arm_hidden{0},arm_age_us{0},arm_stale{0};
thread_local bool t_arm_ready=false,t_arm_swap=false;
thread_local const float* t_arm=nullptr;
float arm_probe[2][2][3]{};  // log only: group centre in canvas units per eye, and its distance; racy reads are acceptable
struct V3 { float x,y,z; };
V3 operator+(V3 a,V3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
V3 operator-(V3 a,V3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
V3 operator*(V3 a,float k) { return {a.x*k,a.y*k,a.z*k}; }
float dot(V3 a,V3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
V3 cross(V3 a,V3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
V3 unit(V3 a) { const float l=std::sqrt(dot(a,a)); return l>1e-5f?a*(1.f/l):V3{0.f,0.f,0.f}; }
V3 rotate(float x,float y,float z,float w,V3 v) { const V3 q{x,y,z}; const V3 t=cross(q,v)*2.f; return v+t*w+cross(q,t); }
// Elements: canvas selector from the ownership captures, and the panel centre that keeps each group's own layout. The four bottom
// groups share a centre so they stay arranged as the game lays them out. Placement comes from AvatarVR_hud_widgets.txt (IK tuner).
enum Attach { Flat=0, LeftArm, RightArm, LeftHand, RightHand, Weapon, Hidden, Bone, Head, Aim };
struct ElementDef { const char* key; int named; float x0,y0,x1,y1,cx,cy; };
constexpr ElementDef Elements[]={
  {"health",0,475,45,490,55,640,75},{"energy",1,495,10,580,42,640,75},{"heal",-1,430,25,445,36,640,75},{"eat",-1,585,3,595,13,640,75},
  {"ammo",-1,1015,8,1160,24,1130,65},{"status",-1,1100,615,1115,636,1185,650},{"tracker",-1,22,625,60,710,150,660},{"compass",-1,440,685,840,730,640,700},
  // Seen in captures, flat until chosen. Only "save" and "notify" have a known on-screen meaning: the others are named by where they sit.
  {"save",-1,10,4,24,14,40,15},{"notify",-1,655,135,700,160,640,150},{"centre",-1,655,298,680,312,666,304},
  {"top",-1,625,640,655,655,640,650},{"right",-1,780,320,1000,625,880,470},{"low",-1,784,135,1000,310,880,220},
  // The aim mark. HEADSET: it is the full-canvas square near (280,0), which moves a few units (LIVE 277-281, -1 to 1). The box must hold
  // all of that: a frame outside it draws on the flat HUD. INFERENCE: the square's centre is the point aimed at.
  {"reticle",-1,262,-18,298,18,640,360},
  // HEADSET 2026-10-31: a double stays on the flat HUD. LIVE (AvatarVR_fullview_seen.txt): a scale-1 widget at (635,355), 10 units around the
  // canvas centre, drew flat in that run. INFERENCE: it is that double. Same key, so it takes the reticle's placement and lands on the same point.
  {"reticle",-1,630,350,640,360,640,360}};
constexpr int ElementCount=sizeof(Elements)/sizeof(Elements[0]);
struct Placement { float attach,along,side,lift,yaw,pitch,roll,scale,bone,two_sided,shrink; };
const Placement PlacementDefaults[ElementCount]={
  {LeftArm,.12f,0,.015f,0,0,0,1,-1,0},{LeftArm,.12f,0,.015f,0,0,0,1,-1,0},{LeftArm,.12f,0,.015f,0,0,0,1,-1,0},{LeftArm,.12f,0,.015f,0,0,0,1,-1,0},
  {LeftArm,.26f,0,.015f,0,0,0,1,-1,0},{LeftHand,0,0,.02f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},
  {Flat,0,0,.015f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},{Flat,0,0,.015f,0,0,0,1,-1,0},
  {Aim,0,0,0,0,0,0,1,-1,0,.5f},{Aim,0,0,0,0,0,0,1,-1,0,.5f}};
std::atomic<int> arm_method{1};
std::atomic<int> arm_behind{0};
std::mutex placement_mutex;
Placement placements[ElementCount];
bool placements_ready=false;
ULONGLONG placements_read=0;
thread_local float t_arm_elements[ElementCount][2][16];
thread_local unsigned char t_arm_state[ElementCount];  // 0 flat, 1 placed, 2 hidden (behind the eye or no controller)
thread_local bool t_arm_aim[ElementCount];
void read_placements() {
  Placement next[ElementCount]; std::memcpy(next,PlacementDefaults,sizeof(next));
  wchar_t path[MAX_PATH]{}; HMODULE self=nullptr;
  if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&install),&self)) {
    const DWORD length=GetModuleFileNameW(self,path,MAX_PATH);
    std::wstring file(path,length); const auto slash=file.find_last_of(L"\\/");
    std::string text;
    if (length && length<MAX_PATH && slash!=std::wstring::npos &&
        settings_file::read(file.substr(0,slash+1)+L"AvatarVR_hud_widgets.txt", text)) {
      std::istringstream input(text);
      for (std::string line; std::getline(input, line);) {
        char element[24]{},field_name[16]{}; float value=0.f;
        if (std::sscanf(line.c_str(),"%23[^.].%15[^=]=%f",element,field_name,&value)!=3 || !std::isfinite(value)) continue;
        if (!std::strcmp(element,"settings") && !std::strcmp(field_name,"frame")) { skin_palette_probe::set_hud_frame_method(static_cast<int>(value)); arm_method.store(static_cast<int>(value)); continue; }
        for (int i=0;i<ElementCount;++i) if (!std::strcmp(element,Elements[i].key)) {
          auto& n=next[i];
          if (!std::strcmp(field_name,"attach")) n.attach=value; else if (!std::strcmp(field_name,"along")) n.along=value;
          else if (!std::strcmp(field_name,"side")) n.side=value; else if (!std::strcmp(field_name,"lift")) n.lift=value;
          else if (!std::strcmp(field_name,"yaw")) n.yaw=value; else if (!std::strcmp(field_name,"pitch")) n.pitch=value;
          else if (!std::strcmp(field_name,"roll")) n.roll=value; else if (!std::strcmp(field_name,"scale")) n.scale=value;
          else if (!std::strcmp(field_name,"bone")) n.bone=value; else if (!std::strcmp(field_name,"two_sided")) n.two_sided=value;
          else if (!std::strcmp(field_name,"shrink")) n.shrink=value;
        }
      }
    }
    // One bone list per process for the wizard, once the game's names are known.
    static bool names_written=false;
    if (!names_written && length && length<MAX_PATH && slash!=std::wstring::npos && !skin_palette_probe::hud_bone_name(0).empty()) {
      FILE* out=nullptr;
      if (_wfopen_s(&out,(file.substr(0,slash+1)+L"AvatarVR\\state\\AvatarVR_hud_bones.txt").c_str(),L"wb")==0 && out) {
        for (int i=0;i<211;++i) { const auto name=skin_palette_probe::hud_bone_name(i); if (!name.empty()) std::fprintf(out,"%d=%s\n",i,name.c_str()); }
        std::fclose(out); names_written=true;
      }
    }
  }
  for (int i=0;i<ElementCount;++i) skin_palette_probe::set_hud_bone(i,static_cast<int>(next[i].attach)==Bone?static_cast<int>(next[i].bone):-1);
  std::lock_guard<std::mutex> lock(placement_mutex);
  std::memcpy(placements,next,sizeof(placements)); placements_ready=true;
}
V3 about(V3 v,V3 axis,float degrees) {
  const float a=degrees*0.01745329f, c=std::cos(a), s=std::sin(a);
  return v*c+cross(axis,v)*s+axis*(dot(axis,v)*(1.f-c));
}
// The bone axis closest to a direction, signed. Bone axes have no fixed meaning, so sliders are matched to real directions this way.
V3 nearest_axis(const float* axes,const float* target,int skip,int* picked=nullptr,bool* positive=nullptr) {
  int best=-1; float best_dot=0.f;
  for (int a=0;a<3;++a) {
    if (a==skip) continue;
    const float d=axes[a*3]*target[0]+axes[a*3+1]*target[1]+axes[a*3+2]*target[2];
    if (best<0 || std::fabs(d)>std::fabs(best_dot)) { best=a; best_dot=d; }
  }
  if (picked) *picked=best;
  if (positive) *positive=best_dot>=0.f;
  const float sign=best_dot<0.f?-1.f:1.f;
  return unit(V3{axes[best*3],axes[best*3+1],-axes[best*3+2]}*sign);
}
// Weapon panel axes, chosen once per hand and rig and again when a weapon is drawn. [SOURCE] choosing every build let a hold near 45 degrees
// from the controller swap axes for a frame, which throws a panel with an offset to another place. Log: `weapon axis flips=`.
std::atomic<int> weapon_axis_latch[4];
std::atomic<unsigned> weapon_axis_flips{0};
std::atomic<bool> weapon_axis_drawn{false};
void weapon_axes(const float* axes,const float* grip_forward,const float* grip_up,int slot,V3* forward_out,V3* up_out) {
  const bool drawn=skin_palette_probe::held_weapon_drawn();
  if (weapon_axis_drawn.exchange(drawn)!=drawn) for (auto& latch:weapon_axis_latch) latch.store(0);
  int forward=-1,up=-1; bool forward_positive=true,up_positive=true;
  nearest_axis(axes,grip_forward,-1,&forward,&forward_positive);
  nearest_axis(axes,grip_up,forward,&up,&up_positive);
  const int code=1+forward+3*up+9*(forward_positive?1:0)+18*(up_positive?1:0);
  int latched=0;
  if (weapon_axis_latch[slot].compare_exchange_strong(latched,code)) latched=code;
  else if (latched!=code) weapon_axis_flips.fetch_add(1,std::memory_order_relaxed);
  const int c=latched-1;
  const auto axis=[&](int a,bool positive) { const float s=positive?1.f:-1.f; return unit(V3{axes[a*3],axes[a*3+1],-axes[a*3+2]}*s); };
  *forward_out=axis(c%3,(c/9)%2!=0); *up_out=axis((c/3)%3,(c/18)%2!=0);
}
bool arm_frames() {
  // Everything here is relative to the game camera that rendered the bones, so body, locomotion and head motion need no correction.
  const auto settings=wrist_hud::current();
  skin_palette_probe::HudCameraPose pose; head_pose::Snapshot snap;
  // The flat HUD comes back only where no arm exists to attach to: menus, mounted without solved arms, or a rig that has never drawn.
  if (!native_stereo::stereo_running() || native_stereo::ui_layer_active() || (mount_camera::rider_view() && !mount_camera::mounted_arms()) ||
      !skin_palette_probe::hud_camera_seen()) return false;
  Placement current[ElementCount];
  { std::lock_guard<std::mutex> lock(placement_mutex); if (!placements_ready) return false; std::memcpy(current,placements,sizeof(current)); }
  const bool pose_ok=skin_palette_probe::hud_camera_pose(&pose);
  if (!pose_ok) {
    // INFERENCE: the game stops drawing the arm mesh out of view. Its elements are out of view too: hide them, stay off the flat HUD.
    for (int element=0;element<ElementCount;++element) t_arm_state[element]=static_cast<int>(current[element].attach)==Flat?0:2;
    arm_stale.fetch_add(1,std::memory_order_relaxed);
    // Head elements need no bones and stay up.
    if (!native_stereo::views_for_present(0,&snap)) return true;
  } else {
    arm_age_us.store(static_cast<unsigned>(pose.age_us),std::memory_order_relaxed);
    // The eyes of the same applied pose the bones were measured against.
    if (!native_stereo::applied_pose(pose.frame_id,&snap) && !native_stereo::views_for_present(0,&snap)) return false;
    float newest[16]; std::uint64_t id=0; if (native_stereo::frame_camera(0,newest,&id)) arm_behind.store(static_cast<int>(id-pose.frame_id),std::memory_order_relaxed);
  }
  const int weapon_hand=skin_palette_probe::weapon_hand_index();
  // Head elements: the camera does not pitch with the headset, and the eye image is reprojected from the pose it rendered with. So place
  // them on the newest headset pose, seen from the eyes this frame renders with: they stay on the view through that reprojection.
  head_pose::Snapshot rendering=snap, newest_head;
  { float cam[16]; std::uint64_t id=0; if (native_stereo::frame_camera(1,cam,&id)) native_stereo::applied_pose(id,&rendering); }
  if (!xr_runtime::latest_views(&newest_head)) newest_head=rendering;
  const auto& h0=newest_head.eye[0]; const auto& h1=newest_head.eye[1];
  const float flip=h0.qx*h1.qx+h0.qy*h1.qy+h0.qz*h1.qz+h0.qw*h1.qw<0.f?-1.f:1.f;
  float head_q[4]={h0.qx+flip*h1.qx,h0.qy+flip*h1.qy,h0.qz+flip*h1.qz,h0.qw+flip*h1.qw};
  { const float l=std::sqrt(head_q[0]*head_q[0]+head_q[1]*head_q[1]+head_q[2]*head_q[2]+head_q[3]*head_q[3]); if (!(l>1e-5f)) return true; for (float& q:head_q) q/=l; }
  const V3 head_p{(h0.px+h1.px)*0.5f,(h0.py+h1.py)*0.5f,(h0.pz+h1.pz)*0.5f};
  // Camera axes are x right, y up, z forward; panel maths uses right-handed -z forward.
  const auto rh=[](const float* p) { return V3{p[0],p[1],-p[2]}; };
  float eye_m[2][9],eye_off[2][3];
  for (int e=0;e<2;++e) native_stereo::eye_in_camera(snap.eye[e],eye_m[e],eye_off[e]);
  for (int element=0;element<ElementCount;++element) {
    Placement place=current[element];
    // C_weaponAttachmentWorld (index 2, AvatarVR_hud_bones.txt) stays with the animation, not the IK gun: it means the weapon.
    if (static_cast<int>(place.attach)==Bone && static_cast<int>(place.bone)==2) place.attach=Weapon;
    const int kind=static_cast<int>(place.attach);
    t_arm_state[element]=2;
    t_arm_aim[element]=kind==Aim;
    // Saved with the gun in the right hand; a left-hand hold mirrors it across the gun.
    if (kind==Weapon && weapon_hand==0) { place.side=-place.side; place.yaw=-place.yaw; place.roll=-place.roll; }
    if (kind==Flat) { t_arm_state[element]=0; continue; }
    if (kind==Hidden || kind<0 || kind>Aim || (!pose_ok && kind!=Head)) continue;
    const int hand=kind==Weapon?weapon_hand:(kind==RightArm||kind==RightHand)?1:0;
    V3 origin,axis,normal,right;
    float reach=0.f;  // Aim: eyes to the point aimed at
    if (kind==Aim) {
      // On the steady ray where it hits, facing the eyes, at the flat HUD's angular size. On foot from the wrist, mounted from the gun's
      // calibrated ray. Flat only with shots not on the gun or no mounted gun; otherwise hidden without a ray: flat it would be a second mark.
      float shot_distance=0.f,start[3],way[3];
      if (!vr_weapon::shot_ray_on() || (pose.rider && pose.ray_flat)) { t_arm_state[element]=0; continue; }
      if (pose.rider) {
        if (!pose.ray_ok || !vr_weapon::shot_distance(&shot_distance)) continue;
        std::memcpy(start,pose.ray_start,sizeof(start)); std::memcpy(way,pose.ray_way,sizeof(way));
      } else {
        float shot_origin[3],shot_forward[3];
        if (!pose.hand_ok[weapon_hand] || !vr_weapon::shot_aim(shot_origin,shot_forward,&shot_distance)) continue;
        if (!vr_weapon::held_is_bow()) vr_recoil::unkick_ray(weapon_hand,shot_origin,shot_forward);  // the drawn wrist kicks, the mark does not
        const float* wrist=pose.wrist_axes[weapon_hand];
        for (int r=0;r<3;++r) {
          start[r]=pose.wrist[weapon_hand][r]; way[r]=0.f;
          for (int a=0;a<3;++a) { start[r]+=shot_origin[a]*wrist[a*3+r]; way[r]+=shot_forward[a]*wrist[a*3+r]; }
        }
      }
      origin=rh(start)+unit(rh(way))*shot_distance;
      const V3 eyes=(rh(eye_off[0])+rh(eye_off[1]))*0.5f, head_up{eye_m[0][3],eye_m[0][4],-eye_m[0][5]};
      const V3 back=eyes-origin;
      reach=std::sqrt(dot(back,back));
      if (!(reach>0.05f)) continue;
      normal=back*(1.f/reach); axis=normal; right=unit(cross(head_up,normal));
    } else if (kind==Head) {
      // The camera itself: along = forward, side = right, lift = up, facing the eyes.
      origin=V3{0.f,0.f,0.f}; axis=V3{0.f,0.f,-1.f}; normal=V3{0.f,0.f,1.f}; right=V3{1.f,0.f,0.f};
    } else if (kind==Bone) {
      if (!pose.bone_ok[element]) continue;
      origin=rh(pose.bone[element]); right=unit(rh(pose.bone_axes[element])); axis=unit(rh(pose.bone_axes[element]+3)); normal=unit(rh(pose.bone_axes[element]+6));
    } else if (kind==Weapon) {
      if (!pose.hand_ok[hand]) continue;
      // Along = gun forward, off the surface = gun up, whichever bone axes those are.
      origin=rh(pose.attach[hand]);
      weapon_axes(pose.attach_axes[hand],pose.grip_axes[hand]+6,pose.grip_axes[hand]+3,hand+(pose.rider?2:0),&axis,&normal);
      right=unit(cross(axis,normal));
    } else if (kind==LeftHand || kind==RightHand) {
      if (!pose.hand_ok[hand]) continue;
      // The wrist bone itself, so the panel bends and rolls with the hand. Which axis is which is chosen once, near the rest pose.
      static thread_local int along_axes[4]={-1,-1,-1,-1}, along_signs[4], normal_axes[4], normal_signs[4];
      const int rig=pose.rider?2:0;  // the two rigs have different bone axes
      int* along_axis=along_axes+rig; int* along_sign=along_signs+rig; int* normal_axis=normal_axes+rig; int* normal_sign=normal_signs+rig;
      const V3 w=rh(pose.wrist[hand]), d=unit(rh(pose.elbow[hand])-w);
      const float* a=pose.wrist_axes[hand];
      const auto bone_axis=[&](int i) { return unit(V3{a[i*3],a[i*3+1],-a[i*3+2]}); };
      if (along_axis[hand]<0) {
        for (int i=0;i<3;++i) if (std::fabs(dot(bone_axis(i),d))>0.7f) { along_axis[hand]=i; along_sign[hand]=dot(bone_axis(i),d)<0.f?-1:1; }
        if (along_axis[hand]<0) continue;
        const V3 outward=rh(pose.outward[hand])*(hand?1.f:-1.f);
        float best=0.f;
        for (int i=0;i<3;++i) if (i!=along_axis[hand] && std::fabs(dot(bone_axis(i),outward))>=std::fabs(best)) { best=dot(bone_axis(i),outward); normal_axis[hand]=i; }
        normal_sign[hand]=best<0.f?-1:1;
      }
      axis=bone_axis(along_axis[hand])*static_cast<float>(along_sign[hand]);
      normal=bone_axis(normal_axis[hand])*static_cast<float>(normal_sign[hand]);
      right=hand?axis:axis*-1.f;
      origin=w-axis*0.07f;
    } else {
      if (!pose.hand_ok[hand]) continue;
      const V3 w=rh(pose.wrist[hand]), d=unit(rh(pose.elbow[hand])-w);
      const V3 outward=rh(pose.outward[hand])*(hand?1.f:-1.f);
      normal=unit(outward-d*dot(outward,d)); right=hand?d:d*-1.f; axis=d;
      origin=w;
      if (dot(d,d)<0.5f || dot(normal,normal)<0.5f) continue;
    }
    V3 up=cross(normal,right);
    // Sideways is across the attachment, whichever way the panel's text runs: on an arm the panel's right is along the arm.
    const bool faces_view=kind==Head||kind==Aim;
    const V3 across=faces_view?right:unit(cross(axis,normal));
    // Aim: side and lift are angles (per metre of distance), to line the mark up with where shots land.
    const float slide=kind==Aim?reach:1.f;
    const V3 centre=origin+axis*place.along+across*(place.side*slide)+(faces_view?up:normal)*(place.lift*slide);
    if (place.yaw!=0.f) { right=about(right,up,place.yaw); normal=about(normal,up,place.yaw); }
    if (place.pitch!=0.f) { up=about(up,right,place.pitch); normal=about(normal,right,place.pitch); }
    if (place.roll!=0.f) { right=about(right,normal,place.roll); up=about(up,normal,place.roll); }
    const float flat_unit=(std::tan(snap.eye[0].fov[1])-std::tan(snap.eye[0].fov[0]))/1280.f;
    // Aim: the flat HUD's apparent size at 5 m. Shrink 0 keeps it at any distance, 1 scales it like a real object; limited to stay usable.
    const float apparent=kind==Aim?std::min(3.f,std::max(.3f,std::pow(5.f/reach,place.shrink))):1.f;
    const float scale=kind==Aim?flat_unit*reach*place.scale*apparent:0.0005f*place.scale*settings.size_m/0.1f;
    const float cx=Elements[element].cx, cy=Elements[element].cy;
    bool visible=true;
    float facing=0.f;
    for (int e=0;e<2 && visible;++e) {
      const auto& eye=snap.eye[e];
      const float tl=eye.rvalid?eye.rtan[0]:std::tan(eye.fov[0]), tr=eye.rvalid?eye.rtan[1]:std::tan(eye.fov[1]);
      const float tu=eye.rvalid?eye.rtan[2]:std::tan(eye.fov[2]), td=eye.rvalid?eye.rtan[3]:std::tan(eye.fov[3]);
      if (!(tr>tl) || !(tu>td)) return false;
      const float* m=eye_m[e];
      const auto& seen_from=rendering.eye[e];
      const auto head_to_eye=[&](V3 v) { return rotate(-seen_from.qx,-seen_from.qy,-seen_from.qz,seen_from.qw,v); };
      const auto to_eye=[&](V3 v) {  // eye axis i is column i of m, in camera axes
        if (kind==Head) return head_to_eye(rotate(head_q[0],head_q[1],head_q[2],head_q[3],v));
        const V3 c{v.x,v.y,-v.z};
        return V3{m[0]*c.x+m[3]*c.y+m[6]*c.z, m[1]*c.x+m[4]*c.y+m[7]*c.z, -(m[2]*c.x+m[5]*c.y+m[8]*c.z)};
      };
      const V3 er=to_eye(right)*scale, eu=to_eye(up)*scale, e0=kind==Head?head_to_eye(head_p+rotate(head_q[0],head_q[1],head_q[2],head_q[3],centre)-V3{seen_from.px,seen_from.py,seen_from.pz}):to_eye(centre-rh(eye_off[e]));
      const float w0=-e0.z;
      if (!(w0>0.05f)) { visible=false; break; }  // behind this eye: nothing to draw, and never back to the flat HUD
      facing-=dot(to_eye(normal),e0);  // normal toward the eye: the front of the panel
      const auto row=[&](V3 v,float* out) {
        const float depth=-v.z;
        out[0]=1280.f*(v.x-tl*depth)/(tr-tl)/w0; out[1]=720.f*(v.y-td*depth)/(tu-td)/w0; out[2]=0.f; out[3]=depth/w0;
      };
      float* h=t_arm_elements[element][e];
      row(er,h); row(eu,h+4); h[8]=0.f; h[9]=0.f; h[10]=1.f; h[11]=0.f;
      row(e0-er*cx-eu*cy,h+12);
      if (element==0 || element==4) {
        const int probe=element?1:0;
        arm_probe[probe][e][0]=(cx*h[0]+cy*h[4]+h[12])/(cx*h[3]+cy*h[7]+h[15]);
        arm_probe[probe][e][1]=(cx*h[1]+cy*h[5]+h[13])/(cx*h[3]+cy*h[7]+h[15]);
        arm_probe[probe][e][2]=w0;
      }
    }
    // Back-face hiding is for skin; a weapon or bone panel has no fixed front, and vanishing there reads as a missing widget.
    if (visible && (place.two_sided!=0.f || kind==Weapon || kind==Bone || faces_view || facing>0.f)) t_arm_state[element]=1;
  }
  return true;
}
std::atomic<int> arm_seen[ElementCount][4];  // log only: distinct matched origins, x*4096+y; racy updates are acceptable
void note_arm_match(int element,const float* world) {
  const int origin=static_cast<int>(world[12])*4096+static_cast<int>(world[13])+1;
  for (auto& seen:arm_seen[element]) {
    int expected=seen.load(std::memory_order_relaxed);
    if (expected==origin) return;
    if (!expected && seen.compare_exchange_strong(expected,origin,std::memory_order_relaxed)) return;
  }
}
int arm_element(void* widget) {
  // By origin only. A remembered name is not proof: the HUD is rebuilt around menus and a freed address can come back as another widget.
  float world[16]; void* parent=nullptr; float above[16];
  if (!read(static_cast<unsigned char*>(widget)+0x60,world,sizeof(world)) || !(world[0]>0.5f)) return -1;
  for (int i=0;i<ElementCount;++i) {
    const auto& def=Elements[i];
    if (!t_arm_state[i]) continue;  // named elements also match by origin, in case their name registration was missed
    const auto in_box=[&](const float* m) { return m[12]>=def.x0 && m[12]<=def.x1 && m[13]>=def.y0 && m[13]<=def.y1; };
    // The two ammo groups sit at each end of the box; short-lived widgets at equip and pickup appear between them.
    if (!std::strcmp(def.key,"ammo") && world[12]>1036.f && world[12]<1114.f) continue;
    if (!in_box(world) || !field(widget,0x150,&parent) || !parent) continue;
    if (!read(static_cast<unsigned char*>(parent)+0x60,above,sizeof(above)) || !in_box(above)) { note_arm_match(i,world); return i; }
  }
  return -1;
}
// Full-view effects (2026-10-30, saved flag, default off). As the scanner's decorative plane: the game's own flat draw, unmoved, in its
// own list (FxLayer, drawn through the arm layer's pass in each eye), and camera.cpp maps that layer onto one head-forward plane
// shared by both eyes. An earlier route moved it into the arm layers, where it fought the body widgets and the scanner and boost routes
// [HEADSET]. Chosen at the emit by origin (AvatarVR_fullview_fx.txt, IK tuner): only a widget's own commands move, so a parent composing
// to the same origin cannot take a HUD branch with it. Strength repeats the draw: coverage 1-(1-a)^n with the game's src-alpha blend.
struct FxEffect { float x,y,on,strength,size; };
constexpr int FxMax=6;
// Layer 6: no game draws in gameplay (LIVE 10:32). Layer 5 was drawn by the game in both eyes, but into a target that never reaches the
// eye: the red and the scanner art with it vanished [HEADSET 2026-11-07, while routed and layer draws were both above 0].
constexpr int FxLayer=6;
constexpr FxEffect FxDefault={0.f,0.f,1.f,3.f,1.02f};  // the red damage effect [HEADSET]
std::mutex fx_mutex;
FxEffect fx_effects[FxMax]={FxDefault};
std::atomic<float> fx_size{FxDefault.size};
std::atomic<unsigned> fx_routed{0};
// The scanner and boost draw full-screen art at the same origin, and moved into the effects layer it composites black [HEADSET].
// Widgets that first appear while either draws stay in their own list; the red is there long before (OPEN: red created during a scan).
std::atomic<ULONGLONG> fx_effect_at{0};
std::atomic<unsigned> fx_effect_in_layer{0}, fx_kept_out{0};
struct FxWidget { void* widget; ULONGLONG first,last; bool tainted; };
std::mutex fx_widget_mutex;
FxWidget fx_widgets[64];
// After a load or a stereo start every widget is new, and the 600 ms wait showed as the red arriving a second late [HEADSET 2026-11-08].
// Widgets present in the first 1.5 s of routing count as already settled, unless the scanner or boost is drawing.
std::atomic<ULONGLONG> fx_ready_last{0}, fx_ready_since{0};
bool fx_widget_eligible(void* widget) {
  const ULONGLONG now=GetTickCount64();
  const bool effect=now-fx_effect_at.load(std::memory_order_relaxed)<1000;
  const bool settling=now-fx_ready_since.load(std::memory_order_relaxed)<1500;
  std::lock_guard<std::mutex> lock(fx_widget_mutex);
  FxWidget* slot=nullptr;
  for (auto& w:fx_widgets) {
    if (w.widget==widget && now-w.last<2000) { slot=&w; break; }
    if (!slot && (!w.widget || now-w.last>=2000)) slot=&w;
  }
  if (!slot) return false;
  if (slot->widget!=widget || now-slot->last>=2000) *slot={widget,settling && !effect?now-600:now,now,effect};
  slot->last=now;
  if (now-slot->first<600) { slot->tainted=slot->tainted || effect; return false; }
  return !slot->tainted;
}
std::atomic<int> fx_seen[16];
std::atomic<bool> fx_seen_changed{false};
thread_local bool t_fx_ready=false;
thread_local FxEffect t_fx[FxMax];
std::wstring beside_dll(const wchar_t* name) {
  wchar_t path[MAX_PATH]{}; HMODULE self=nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&install),&self)) return {};
  const DWORD length=GetModuleFileNameW(self,path,MAX_PATH);
  std::wstring file(path,length); const auto slash=file.find_last_of(L"\\/");
  return !length || length>=MAX_PATH || slash==std::wstring::npos ? std::wstring{} : file.substr(0,slash+1)+name;
}
void read_fx() {
  FxEffect next[FxMax]={FxDefault};
  std::string text;
  const auto path=beside_dll(L"AvatarVR_fullview_fx.txt");
  if (!path.empty() && settings_file::read(path, text)) {
    // Fields a line leaves out keep the default (older tuners wrote other fields); entries with no line at all are off.
    FxEffect parsed[FxMax]; bool listed[FxMax]{}; bool any=false;
    for (auto& n:parsed) n=FxDefault;
    std::istringstream input(text);
    for (std::string line; std::getline(input, line);) {
      unsigned i=0; char field_name[16]{}; float value=0.f;
      if (std::sscanf(line.c_str(),"effect%u.%15[^=]=%f",&i,field_name,&value)!=3 || i>=FxMax || !std::isfinite(value)) continue;
      any=listed[i]=true;
      auto& n=parsed[i];
      if (!std::strcmp(field_name,"x")) n.x=value; else if (!std::strcmp(field_name,"y")) n.y=value;
      else if (!std::strcmp(field_name,"on")) n.on=value; else if (!std::strcmp(field_name,"strength")) n.strength=value;
      else if (!std::strcmp(field_name,"size")) n.size=value;
    }
    // An empty file (LIVE 2026-11-07: 0 bytes from the tuner) keeps the default instead of turning every effect off.
    if (any) { for (int i=0;i<FxMax;++i) if (!listed[i]) parsed[i].on=0.f; std::memcpy(next,parsed,sizeof(next)); }
  }
  for (auto& n:next) { n.strength=n.strength<1.f?1.f:n.strength>6.f?6.f:n.strength; n.size=n.size<.7f?.7f:n.size>2.f?2.f:n.size; }
  // One layer, one plane: the first effect that is on sets its size.
  for (const auto& n:next) if (n.on!=0.f) { fx_size.store(n.size,std::memory_order_relaxed); break; }
  std::lock_guard<std::mutex> lock(fx_mutex);
  std::memcpy(fx_effects,next,sizeof(next));
}
// Every full-size widget origin goes to AvatarVR_fullview_seen.txt, so a new flat effect can be picked in the tuner.
int fx_match(void* widget) {
  float world[16];
  if (!read(static_cast<unsigned char*>(widget)+0x60,world,sizeof(world)) || !(world[0]>0.99f) || !(world[5]>0.99f)) return -1;
  if (!(std::fabs(world[12]-640.f)<1000.f) || !(std::fabs(world[13]-360.f)<1000.f)) return -1;  // parked off screen
  const int code=(static_cast<int>(std::lround(world[12]))+4096)*16384+static_cast<int>(std::lround(world[13]))+4096;
  for (auto& seen:fx_seen) {
    int expected=seen.load(std::memory_order_relaxed);
    if (expected==code) break;
    if (!expected && seen.compare_exchange_strong(expected,code,std::memory_order_relaxed)) { fx_seen_changed.store(true); break; }
  }
  for (int i=0;i<FxMax;++i) if (t_fx[i].on!=0.f && std::fabs(world[12]-t_fx[i].x)<1.f && std::fabs(world[13]-t_fx[i].y)<1.f) {
    if (fx_widget_eligible(widget)) return i;
    fx_kept_out.fetch_add(1,std::memory_order_relaxed);
    return -1;
  }
  return -1;
}
void write_fx_seen() {
  FILE* f=nullptr;
  const auto path=beside_dll(L"AvatarVR\\state\\AvatarVR_fullview_seen.txt");
  if (path.empty() || _wfopen_s(&f,path.c_str(),L"wb")!=0 || !f) return;
  for (auto& seen:fx_seen) if (const int code=seen.load(std::memory_order_relaxed)) std::fprintf(f,"%d,%d\n",code/16384-4096,code%16384-4096);
  std::fclose(f);
}
// A list of 0x527430's shape, for glass nobody should see. It uses only +0x20 data, +0x28 count, +0x2C capacity (top bit: inline storage).
// Our own storage, never grown: growing goes through the engine's allocator, and the build that kept such storage across frames corrupted the
// scene (research/HUD_NATIVE_ATTACHMENT_20261028.md). Emit_hook empties it per widget and refuses any widget that would not fit.
constexpr unsigned GlassSinkRecords=8192;
alignas(16) thread_local unsigned char t_glass_sink[0x30]{};
void* glass_sink() {
  void* store=nullptr; std::memcpy(&store,t_glass_sink+0x20,sizeof(store));
  if (!store) {
    store=std::malloc(GlassSinkRecords*0x28);
    if (!store) return nullptr;
    std::memcpy(t_glass_sink+0x20,&store,sizeof(store));
    std::memcpy(t_glass_sink+0x2C,&GlassSinkRecords,sizeof(GlassSinkRecords));
  }
  return t_glass_sink;
}
void Build_hook(void* build) {
  t_list_base=nullptr;
  const bool arm=arm_on.load(std::memory_order_relaxed);
  t_arm_ready=arm && arm_frames();
  // Gameplay only: with stereo paused for a menu, its own full-screen art at the same origin would follow the head [LIVE, HEADSET].
  t_fx_ready=arm && wrist_hud::current().damage_fx_full && native_stereo::stereo_running() && !native_stereo::front_end();
  if (t_fx_ready) {
    const ULONGLONG now=GetTickCount64();
    if (now-fx_ready_last.exchange(now,std::memory_order_relaxed)>2000) fx_ready_since.store(now,std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(fx_mutex); std::memcpy(t_fx,fx_effects,sizeof(t_fx));
  }
  if (arm) { arm_builds.fetch_add(1,std::memory_order_relaxed); if (!t_arm_ready) arm_fallback.fetch_add(1,std::memory_order_relaxed); }
  if (t_arm_ready || t_fx_ready || test_mode.load(std::memory_order_relaxed)>=8) {
    unsigned char* owner=nullptr;
    if (field(build,0x28,&owner) && owner) t_list_base=owner+0x1F8;
  }
  original_build(build);
  t_list_base=nullptr;
}
// 0x586AA0 stores the running layer's pass in the owner table (+0x1F0, entry id*0x20+0x28) around its call; another layer's list draws
// through that pass when its entry is swapped in for the call. Layer 7 (right-eye arm list) and the full-view effects layer draw this way.
void draw_through(void* owner,void* view,unsigned char* targets,int from,int layer,char first,char second) {
  void* source=nullptr; void* saved=nullptr;
  const unsigned entry=static_cast<unsigned>(layer)*0x20+0x28;
  if (!field(targets,static_cast<unsigned>(from)*0x20+0x28,&source) || !field(targets,entry,&saved)) return;
  std::memcpy(targets+entry,&source,sizeof(source));
  const int outer=t_layer; t_layer=layer; t_arm_swap=true;
  original_layer(owner,view,layer,first,second);
  t_arm_swap=false; t_layer=outer;
  std::memcpy(targets+entry,&saved,sizeof(saved));
}
void Layer_hook(void* owner,void* view,int id,char first,char second) {
  // Widget glass goes to one list per build (layers 9 and 10, drawn by 0x55F150), so placed widgets now send it none (Walk_hook). Never skip those
  // layers here: 0x55F150 reads their target right after the call and crashed the game (research/HUD_NATIVE_ATTACHMENT_20261028.md).
  if (arm_on.load(std::memory_order_relaxed) && !t_arm_swap) {
    const int eye=native_stereo::ui_pass_eye();
    if (id==ArmLayers[1]) return;  // only ever our right-eye list, drawn below
    unsigned char* targets=nullptr;
    if (id==ArmLayers[0] && (eye==1 || eye==2) && field(owner,0x1F0,&targets) && targets) {
      if (eye==2) draw_through(owner,view,targets,id,ArmLayers[1],first,second);
      else { const int outer=t_layer; t_layer=id; original_layer(owner,view,id,first,second); t_layer=outer; }
      // The effects layer is drawn by nobody else (LIVE: no draws when routed there), so it shows only through this pass, in both eyes.
      if (wrist_hud::current().damage_fx_full) draw_through(owner,view,targets,id,FxLayer,first,second);
      return;
    }
  }
  const int outer=t_layer; t_layer=id;
  original_layer(owner,view,id,first,second);
  t_layer=outer;
}
bool test_is_ammo(void* widget) {
  float world[16]; void* parent = nullptr; float above[16];
  if (!read(static_cast<unsigned char*>(widget)+0x60,world,sizeof(world)) || !(world[0]>0.5f)) return false;
  const auto in_box=[](const float* m) { return m[12]>=1000.f && m[12]<=1280.f && m[13]>=0.f && m[13]<=144.f; };
  if (!in_box(world) || !field(widget,0x150,&parent) || !parent) return false;
  return !read(static_cast<unsigned char*>(parent)+0x60,above,sizeof(above)) || !in_box(above);
}
void Walk_hook(void* list,void* widget,void* tree,void* extra) {
  if (t_arm_ready && !t_arm && t_list_base && !t_test_inside) {
    const int element=arm_element(widget);
    if (element>=0 && t_arm_state[element]==2) { arm_hidden.fetch_add(1,std::memory_order_relaxed); return; }
    if (element>=0 && t_arm_state[element]==1) {
      auto* at=static_cast<unsigned char*>(list);
      if (at>=t_list_base && (at-t_list_base)%0x30==0 && (at-t_list_base)/0x30<18) {
        const unsigned slot=static_cast<unsigned>((at-t_list_base)/0x30)&1;
        arm_groups.fetch_add(1,std::memory_order_relaxed);
        void* const sink=glass_sink();
        if (!sink) { arm_hidden.fetch_add(1,std::memory_order_relaxed); return; }
        // Aim: the game moves its mark with sway and recoil (LIVE: origin x 277-281). The ray alone places it, so that motion is taken out.
        float shift[2]={0.f,0.f}, world[16], steady[16];
        if (t_arm_aim[element] && read(static_cast<unsigned char*>(widget)+0x60,world,sizeof(world))) {
          shift[0]=(Elements[element].x0+Elements[element].x1)*.5f-world[12]; shift[1]=(Elements[element].y0+Elements[element].y1)*.5f-world[13];
        }
        for (int e=0;e<2;++e) {
          const float* h=t_arm_elements[element][e];
          std::memcpy(steady,h,sizeof(steady));
          for (int c=0;c<4;++c) steady[12+c]+=shift[0]*h[c]+shift[1]*h[4+c];
          t_arm=steady;
          // One shared glass mask cannot line up with both eyes, and two placements doubled in it. So their glass goes to a list nothing draws.
          // Not null: 0x527430 then draws glass-only commands as ordinary art in the main list.
          original_walk(t_list_base+(slot+ArmLayers[e]*2)*0x30,widget,tree,sink);
        }
        t_arm=nullptr;
        return;
      }
    }
  }
  const unsigned mode=test_mode.load(std::memory_order_relaxed);
  if (!mode || t_test_inside) { original_walk(list,widget,tree,extra); return; }
  bool selected_group=false;
  if (mode<=3 || mode>=8) { for (auto& h:test_health) if (h.load(std::memory_order_relaxed)==widget) selected_group=true; }
  else if (mode!=7) selected_group=test_is_ammo(widget);
  if (!selected_group) { original_walk(list,widget,tree,extra); return; }
  test_selected.fetch_add(1,std::memory_order_relaxed);
  if (mode==3 || mode==6) { test_hidden.fetch_add(1,std::memory_order_relaxed); return; }
  if (mode>=8) {
    // Lists are 0x30 apart: index = hand-off slot + layer*2 (0x535770). Its extra list is 18 + slot, or 20 + slot for layer 4.
    auto* at=static_cast<unsigned char*>(list);
    const unsigned target=RouteLayers[mode-8];
    if (t_list_base && at>=t_list_base && (at-t_list_base)%0x30==0 && (at-t_list_base)/0x30<18) {
      const unsigned slot=static_cast<unsigned>((at-t_list_base)/0x30)&1;
      test_routed.fetch_add(1,std::memory_order_relaxed);
      list=t_list_base+(slot+target*2)*0x30; extra=t_list_base+(slot+18)*0x30;
    }
    original_walk(list,widget,tree,extra);
    return;
  }
  t_test_inside=true;
  original_walk(list,widget,tree,extra);
  t_test_inside=false;
}
// 0x527430 stores this pointer in its output records and reads it when the lists are flushed, after the call has returned, so a
// replacement matrix must outlive the call. Rows are 64 bytes; the ring outlasts any one UI build (about 200 widgets per eye).
struct alignas(16) MatrixSlot { float m[16]; };
MatrixSlot matrix_ring[8192];
std::atomic<unsigned> matrix_next{0};
float* next_matrix() { return matrix_ring[matrix_next.fetch_add(1,std::memory_order_relaxed)%8192].m; }
void Emit_hook(void* list,void* widget,void* commands,float* matrix,void* extra) {
  const int effect=t_fx_ready && !t_arm && t_list_base && !t_test_inside ? fx_match(widget) : -1;
  if (effect>=0) {
    auto* at=static_cast<unsigned char*>(list);
    if (at>=t_list_base && (at-t_list_base)%0x30==0 && (at-t_list_base)/0x30<18) {
      const unsigned slot=static_cast<unsigned>((at-t_list_base)/0x30)&1;
      fx_routed.fetch_add(1,std::memory_order_relaxed);
      // The game's own matrix and glass list: only the list changes.
      for (int n=static_cast<int>(t_fx[effect].strength);n>0;--n)
        original_emit(t_list_base+(slot+FxLayer*2)*0x30,widget,commands,matrix,extra);
      return;
    }
  }
  if (t_arm) {
    unsigned command_count=0;
    if (!read(static_cast<unsigned char*>(commands)+0x10,&command_count,sizeof(command_count)) || command_count>GlassSinkRecords) return;
    std::memset(t_glass_sink+0x28,0,sizeof(int));
    float* placed=next_matrix();
    for (int r=0;r<4;++r) for (int c=0;c<4;++c)
      placed[r*4+c]=matrix[r*4]*t_arm[c]+matrix[r*4+1]*t_arm[4+c]+matrix[r*4+2]*t_arm[8+c]+matrix[r*4+3]*t_arm[12+c];
    original_emit(list,widget,commands,placed,extra);
    return;
  }
  if (!t_test_inside) { original_emit(list,widget,commands,matrix,extra); return; }
  const unsigned mode=test_mode.load(std::memory_order_relaxed);
  float* moved=next_matrix(); std::memcpy(moved,matrix,16*sizeof(float));
  if (mode==1) moved[13]+=150.f;
  else if (mode==4) moved[12]-=300.f;
  else {
    // Rows are v * M, canvas units: turn 50 degrees about a vertical axis through the group, viewed from 300 units.
    // Strong enough that one end is plainly taller if the game keeps perspective in its emitted vertices.
    const float cx=mode==2?640.f:1130.f, c=0.64279f, s=0.76604f, d=300.f;
    for (int r=0;r<4;++r) {
      const float x=matrix[r*4]-matrix[r*4+3]*cx, y=matrix[r*4+1], w=matrix[r*4+3];
      const float nw=w-x*s/d;
      moved[r*4]=x*c+nw*cx; moved[r*4+1]=y; moved[r*4+3]=nw;
    }
  }
  test_emitted.fetch_add(1,std::memory_order_relaxed);
  original_emit(list,widget,commands,moved,extra);
}
void Name_hook(void* registry,void* widget,const char* name) {
  if ((test_mode.load(std::memory_order_relaxed) || arm_on.load(std::memory_order_relaxed)) && name && widget) {
    char text[20]{};
    if (read(name,text,sizeof(text)-1)) {
      const auto remember=[&](std::atomic<void*> (&list)[8],std::atomic<unsigned>& cursor) {
        bool known=false; for (auto& h:list) known=known||h.load(std::memory_order_relaxed)==widget;
        if (!known) list[cursor.fetch_add(1,std::memory_order_relaxed)%8].store(widget,std::memory_order_relaxed);
      };
      if (!std::strcmp(text,"HUD_HealthBar")) remember(test_health,name_cursor[0]);
      else if (!std::strcmp(text,"HUD_PlayerEnergy")) remember(test_energy,name_cursor[1]);
    }
  }
  if (recording() && claim(4,GetTickCount64())) {
    auto e=snapshot('N',nullptr,nullptr,context(),widget); e.record=address(registry);
    for (unsigned i=0;i<sizeof(e.name)-1;++i) {
      char c=0; if (!read(name?name+i:nullptr,&c,1) || !c) break;
      e.name[i]=(c>=32 && c<127 && c!='"' && c!='\\')?c:'?';
    }
    enqueue(e);
  }
  original_name(registry,widget,name);
}
void write_matrix(const float* values) {
  std::fputc('[',output);
  for (unsigned i=0;i<16;++i) {
    if (i) std::fputc(',',output);
    if (std::isfinite(values[i])) std::fprintf(output,"%.9g",values[i]); else std::fputs("null",output);
  }
  std::fputc(']',output);
}
void write_event(const Event& e) {
  std::fprintf(output,"{\"event\":\"%c\",\"session\":%u,\"order\":%llu,\"scope\":%llu,\"parent_scope\":%llu,"
    "\"qpc\":%llu,\"ms\":%llu,\"burst\":%u,\"present_epoch\":%llu,\"tid\":%u,\"depth\":%u,\"eye\":%d,\"pass\":%d,\"frontend_layer\":%d,"
    "\"node\":\"%llx\",\"node_type\":\"%llx\",\"params\":\"%llx\",\"context\":\"%llx\",\"widget\":\"%llx\","
    "\"record\":\"%llx\",\"caller\":\"%llx\",\"graph\":\"%llx\",\"graph_context\":\"%llx\",\"cache\":\"%llx\","
    "\"piece\":%u,\"value\":%u,\"tag\":%u,\"flags\":%u,\"matrix_flags\":%u,\"name\":\"%s\","
    "\"execution_resource\":\"%llx\",\"graph_epoch\":%u,\"inherited_graph\":%d,"
    "\"pin_descriptor\":\"%llx\",\"pin_payload\":\"%llx\",\"scope_chain\":[",
    e.kind,e.session,e.order,e.scope,e.parent_scope,e.qpc,e.elapsed,e.burst,e.epoch,e.tid,e.depth,e.eye,e.pass,e.menu,
    e.node,e.node_type,e.params,e.context,e.widget,e.record,e.caller,e.graph,e.graph_context,e.cache,
    e.id,e.value,e.tag,e.flags,e.matrix_flags,e.name,e.execution_resource,e.graph_epoch,e.inherited_graph,
    e.pin_descriptor,e.pin_payload);
  for (unsigned i=0;i<e.scope_count;++i) std::fprintf(output,"%s%llu",i?",":"",e.scope_chain[i]);
  std::fputs("],\"ancestry\":[",output);
  for (unsigned i=0;i<e.ancestry_count;++i) std::fprintf(output,"%s\"%llx\"",i?",":"",e.ancestry[i]);
  std::fprintf(output,"],\"local_read\":%d,\"world_read\":%d,\"local\":",e.local_read,e.world_read);
  write_matrix(e.local); std::fputs(",\"world\":",output); write_matrix(e.world); std::fputs("}\n",output);
}
bool open_output() {
  wchar_t path[MAX_PATH]{}; HMODULE self=nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
      reinterpret_cast<LPCWSTR>(&install),&self)) return false;
  const DWORD length=GetModuleFileNameW(self,path,MAX_PATH);
  if (!length || length>=MAX_PATH) return false;
  std::wstring filename(path,length); const auto slash=filename.find_last_of(L"\\/");
  if (slash==std::wstring::npos) return false;
  filename.resize(slash+1); SYSTEMTIME time; GetLocalTime(&time);
  wchar_t suffix[96]; swprintf_s(suffix,L"HUD_OWNERSHIP_%04u%02u%02u_%02u%02u%02u_%03u.jsonl",
    time.wYear,time.wMonth,time.wDay,time.wHour,time.wMinute,time.wSecond,time.wMilliseconds);
  filename+=suffix;
  if (_wfopen_s(&output,filename.c_str(),L"wb")!=0 || !output) return false;
  LARGE_INTEGER frequency; QueryPerformanceFrequency(&frequency);
  std::fprintf(output,"{\"header\":1,\"schema\":3,\"session\":%u,\"module_base\":\"%llx\",\"observer_compiled\":\"%s %s\","
    "\"required_exe_sha256\":\"98588DA283E8DCF4BB8BA420AC220DC70892222329DFCE115EB1DD6D42DB1E91\","
    "\"qpc_hz\":%llu,\"capture_ms\":%llu,"
    "\"wait_ms\":%llu,\"max_events\":%u,\"burst_period_ms\":1000,\"burst_width_ms\":50,"
    "\"eye_labels\":\"0=unknown/outside UI pass,1=primary,2=second\","
    "\"events\":\"Q=bool query,S=uint query,W=callback entry,X=exit,C=command,N=name\","
    "\"warning\":\"Scopes/ancestry are observations, not proven piece ownership; present_epoch is not rendered frame identity\"}\n",
    capture_id.load()+1,address(module),__DATE__,__TIME__,frequency.QuadPart,CaptureMs,WaitMs,MaxEvents);
  if (std::fflush(output)!=0) { std::fclose(output);output=nullptr;return false; }
  if (logger) {
    char name[96]{}; unsigned i=0; while (suffix[i] && i<sizeof(name)-1) {name[i]=static_cast<char>(suffix[i]);++i;}
    char line[160]; std::snprintf(line,sizeof(line),"hud ownership: file %s",name);logger(line);
  }
  return true;
}
}

void install(unsigned char* base,Log log) {
  module=base; logger=log; prepared.store(base!=nullptr);
  if (log) log("hud ownership: capture action ready; native observer hooks remain uninstalled until requested");
}
// First explicit request only. Once installed, hooks remain idle pass-through until another capture.
// We do not free native trampolines while worker callbacks might still be returning through them.
bool activate_hooks() {
  if (installed.load()) return true;
  auto* base=module; const auto log=logger;
  if (!base) return false;
  const unsigned char slot[]={0x41,0x56,0x48,0x83,0xEC,0x50,0x65,0x48,0x8B,0x04,0x25,0x58,0,0,0};
  // 32-byte guards each occur once across the supported artifact's readable code sections.
  const unsigned char callback[]={0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,0x57,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x20,0x4D,0x8B,0xF0,0x48,0x8B,0xF2,0x4D,0x8B,0x01,0x4D,0x8B,0xF9,0x48};
  const unsigned char command[]={0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0x8B,0xFA,0x48,0x81,0xC1,0x18,0x01,0x00,0x00,0xE8,0x20,0xD6,0xFF,0xFF};
  const unsigned char name[]={0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x6C,0x24,0x18,0x48,0x89,0x74,0x24,0x20,0x41,0x56,0x48,0x83,0xEC,0x20,0x49,0x8B,0xE8,0x4C,0x8B,0xF2,0x48,0x8B,0xD9,0xE8,0x0D};
  const unsigned char walk[]={0x48,0x89,0x6C,0x24,0x18,0x56,0x57,0x41,0x57,0x48,0x83,0xEC,0x30,0x48,0x8B,0xE9,0x49,0x8B,0xF1,0x48,0x8B,0xCA,0x4D,0x8B,0xF8,0x48,0x8B,0xFA,0xE8,0xAF,0x2A,0x03};
  const unsigned char build[]={0x48,0x89,0x4C,0x24,0x08,0x53,0x55,0x56,0x57,0x41,0x56,0x48,0x81,0xEC,0xB0,0x00,0x00,0x00,0x4C,0x89,0xA4,0x24,0xA8,0x00,0x00,0x00,0x33,0xED,0x4C,0x8B,0x61,0x28};
  const unsigned char layer[]={0x44,0x88,0x4C,0x24,0x20,0x48,0x89,0x54,0x24,0x10,0x55,0x53,0x41,0x54,0x41,0x55,0x41,0x57,0x48,0x8D,0xAC,0x24,0xA0,0xFE,0xFF,0xFF,0x48,0x81,0xEC,0x60,0x02,0x00};
  const unsigned char emit[]={0x4C,0x8B,0xDC,0x4D,0x89,0x4B,0x20,0x55,0x53,0x57,0x41,0x57,0x49,0x8D,0xAB,0x18,0xFF,0xFF,0xFF,0x48,0x81,0xEC,0xC8,0x01,0x00,0x00,0x49,0x8B,0x18,0x48,0x8B,0xF9};
  struct Hook { size_t rva; const unsigned char* bytes; size_t size; void* detour; void** trampoline; };
  Hook hooks[]={
    {0x532D60,callback,sizeof(callback),reinterpret_cast<void*>(&Callback_hook),reinterpret_cast<void**>(&original_callback)},
    {0x51BD00,command,sizeof(command),reinterpret_cast<void*>(&Command_hook),reinterpret_cast<void**>(&original_command)},
    {0x601F70,name,sizeof(name),reinterpret_cast<void*>(&Name_hook),reinterpret_cast<void**>(&original_name)},
    {0x527390,walk,sizeof(walk),reinterpret_cast<void*>(&Walk_hook),reinterpret_cast<void**>(&original_walk)},
    {0x527430,emit,sizeof(emit),reinterpret_cast<void*>(&Emit_hook),reinterpret_cast<void**>(&original_emit)},
    {0x535770,build,sizeof(build),reinterpret_cast<void*>(&Build_hook),reinterpret_cast<void**>(&original_build)},
    {0x586EC0,layer,sizeof(layer),reinterpret_cast<void*>(&Layer_hook),reinterpret_cast<void**>(&original_layer)}
  };
  constexpr unsigned HookCount=sizeof(hooks)/sizeof(hooks[0]);
  unsigned char got[32]{};
  if (!read(base+0x32C77A0,got,sizeof(slot)) || std::memcmp(got,slot,sizeof(slot))) {
    if (log) log("hud ownership: TLS guard mismatch; unavailable"); prepared.store(false); return false;
  }
  for (const auto& h:hooks) if (!read(base+h.rva,got,h.size) || std::memcmp(got,h.bytes,h.size)) {
    if (log) log("hud ownership: callback/command/name guard mismatch; unavailable"); prepared.store(false); return false;
  }
  unsigned made=0,enabled=0;
  for (;made<HookCount;++made) if (MH_CreateHook(base+hooks[made].rva,hooks[made].detour,hooks[made].trampoline)!=MH_OK) break;
  if (made==HookCount) for (;enabled<HookCount;++enabled) if (MH_EnableHook(base+hooks[enabled].rva)!=MH_OK) break;
  if (made!=HookCount || enabled!=HookCount) {
    for (unsigned i=0;i<enabled;++i) MH_DisableHook(base+hooks[i].rva);
    for (unsigned i=0;i<made;++i) MH_RemoveHook(base+hooks[i].rva);
    if (log) log("hud ownership: observer hook set rolled back; unavailable"); prepared.store(false); return false;
  }
  slot_index=reinterpret_cast<Slot>(base+0x32C77A0); installed.store(true);
  if (log) log("hud ownership: guarded read-only observer installed for explicit capture request");
  return true;
}
void allow(bool value) { allowed.store(value); if (!value) stopped.store(true); }
void request() { requested.store(true); }
void stop() { stopped.store(true); }
bool capture_active() { return recording(); }
const char* status() {
  const char* text="HUD ownership: unavailable / nouipiece";
  if (prepared.load() && allowed.load()) switch (phase.load()) {
    case 1:text="HUD ownership: waiting for gameplay";break;
    case 2:text="HUD ownership: recording (30 seconds)";break;
    case 3:text="HUD ownership: saving capture";break;
    default:text="HUD ownership: idle";break;
  }
  std::snprintf(status_label,sizeof(status_label),"%s",text);
  return status_label;
}
void piece(void* node,void* params,bool uint_output,std::uintptr_t caller) {
  if (!installed.load(std::memory_order_relaxed) || !allowed.load(std::memory_order_relaxed)) return;
  const auto p=phase.load(std::memory_order_relaxed);
  if (p!=1 && p!=2) return;
  void* input=nullptr; unsigned flags=0;
  if (!field(params,0x28,&input) || !field(input,0x1C,&flags)) return;
  const unsigned n=flags&0xFFFF; if (!n || n>64) return;
  if (flags&0x10000000) { void* inner=nullptr; if (!field(input,0,&inner)) return; input=inner; }
  bool relevant=false;
  for (unsigned i=0;i<n;++i) { unsigned id=~0u; if (field(input,i*0x18+8,&id) && selected(id)) relevant=true; }
  if (!relevant) return;
  const auto now=GetTickCount64();
  if (p==1) {
    if (!native_stereo::stereo_running() || native_stereo::ui_layer_active()) return;
    unsigned expected=1;
    if (phase.compare_exchange_strong(expected,2)) started.store(now);
  }
  if (!recording()) return;
  void* result=nullptr; unsigned out_flags=0; bool output_valid=field(params,0x20,&result) && field(result,0x1C,&out_flags);
  const auto descriptor=address(result);
  if (output_valid && (out_flags&0x10000000)) { void* inner=nullptr; output_valid=field(result,0,&inner); result=inner; }
  if ((out_flags&0xFFFF)<n) output_valid=false;
  void* ctx=context(); void* widget=nullptr; field(ctx,0x68,&widget);
  for (unsigned i=0;i<n;++i) {
    unsigned id=~0u;
    if (!field(input,i*0x18+8,&id) || !selected(id) || !claim(0,now)) continue;
    auto e=snapshot(uint_output?'S':'Q',node,params,ctx,widget); e.id=id; e.caller=caller;
    e.pin_descriptor=descriptor; e.pin_payload=address(result);
    bool valid=false;
    if (output_valid) {
      if (uint_output) valid=field(result,i*4,&e.value);
      else { unsigned char value=0;valid=field(result,i,&value);e.value=value; }
    }
    if (!valid) e.flags|=8;
    enqueue(e);
  }
}
bool arm_layer_active() {
  return arm_on.load(std::memory_order_relaxed) && (t_layer==ArmLayers[0] || t_layer==ArmLayers[1]);
}
bool fx_layer_active() {
  return t_layer==FxLayer && arm_on.load(std::memory_order_relaxed) && wrist_hud::current().damage_fx_full;
}
float fx_plane_size() { return fx_size.load(std::memory_order_relaxed); }
void fx_effect_drawing(bool in_layer) {
  fx_effect_at.store(GetTickCount64(),std::memory_order_relaxed);
  if (in_layer) fx_effect_in_layer.fetch_add(1,std::memory_order_relaxed);
}
void layer_draw(int eye) {
  if (t_layer<0 || t_layer>=12 || eye<0 || eye>2) return;
  layer_draws[t_layer][eye].fetch_add(1,std::memory_order_relaxed);
}
void widget_test_next() { test_requested.fetch_add(1); }
char test_status_label[64]="Native widget test: off"; // menu stores this pointer once; Present refreshes its contents
const char* widget_test_status() { return test_status_label; }
void report() {
  present_epoch.fetch_add(1,std::memory_order_relaxed);
  const auto now=GetTickCount64();
  if (test_requested.exchange(0)) {
    const unsigned next=(test_mode.load()+1)%TestModes;
    if (!next) test_mode.store(0);
    else if (prepared.load() && allowed.load() && activate_hooks()) test_mode.store(next);
    else { test_mode.store(0); if (logger) logger("hud widget test: unavailable (guards or nouipiece)"); }
    std::snprintf(test_status_label,sizeof(test_status_label),"%s",test_labels[test_mode.load()]);
    if (logger) logger(test_labels[test_mode.load()]);
  }
  const bool want_arm=wrist_hud::current().native_arm;
  if (want_arm && !arm_on.load(std::memory_order_relaxed)) {
    if (prepared.load() && allowed.load() && activate_hooks()) { arm_on.store(true); if (logger) logger("native arm hud: on"); }
  } else if (!want_arm && arm_on.load(std::memory_order_relaxed)) { arm_on.store(false); if (logger) logger("native arm hud: off"); }
  if (arm_on.load(std::memory_order_relaxed) && now-placements_read>=250) { placements_read=now; read_placements(); read_fx(); }
  static ULONGLONG arm_logged=0;
  if (arm_on.load(std::memory_order_relaxed) && now-arm_logged>=5000) {
    arm_logged=now; char line[420];
    float travel[3]; skin_palette_probe::hud_camera_range(travel);
    float gap_mm,gap_degrees; skin_palette_probe::hud_camera_gaps(&gap_mm,&gap_degrees);
    std::snprintf(line,sizeof(line),"native arm hud: method=%d frames behind=%d left wrist travel mm=%.0f,%.0f,%.0f rig vs render max=%.0fmm %.1fdeg weapon axis flips=%u pose age=%uus no arm drawn=%u builds=%u flat fallback=%u hidden=%u groups=%u draws L8=%u/%u L7=%u/%u health L=%.0f,%.0f R=%.0f,%.0f at %.2fm ammo L=%.0f,%.0f R=%.0f,%.0f",
      arm_method.load(),arm_behind.load(),travel[0],travel[1],travel[2],gap_mm,gap_degrees,weapon_axis_flips.exchange(0),arm_age_us.load(),arm_stale.exchange(0),arm_builds.exchange(0),arm_fallback.exchange(0),arm_hidden.exchange(0),arm_groups.exchange(0),layer_draws[8][1].exchange(0),layer_draws[8][2].exchange(0),layer_draws[7][1].exchange(0),layer_draws[7][2].exchange(0),
      arm_probe[0][0][0],arm_probe[0][0][1],arm_probe[0][1][0],arm_probe[0][1][1],arm_probe[0][0][2],arm_probe[1][0][0],arm_probe[1][0][1],arm_probe[1][1][0],arm_probe[1][1][1]);
    if (logger) logger(line);
    if (wrist_hud::current().damage_fx_full) {
      char fx_line[160]; std::snprintf(fx_line,sizeof(fx_line),"full view fx: routed=%u layer draws=%u/%u kept out=%u scanner/boost draws in layer=%u",fx_routed.exchange(0),layer_draws[FxLayer][1].exchange(0),layer_draws[FxLayer][2].exchange(0),fx_kept_out.exchange(0),fx_effect_in_layer.exchange(0));
      if (logger) logger(fx_line);
      if (fx_seen_changed.exchange(false)) write_fx_seen();
    }
    if (logger) {
      char seen_line[900]; int used=std::snprintf(seen_line,sizeof(seen_line),"native arm hud: matched origins");
      for (int i=0;i<ElementCount;++i) {
        used+=std::snprintf(seen_line+used,sizeof(seen_line)-used," %s=",Elements[i].key);
        for (auto& seen:arm_seen[i]) {
          const int origin=seen.exchange(0,std::memory_order_relaxed)-1;
          if (origin>=0) used+=std::snprintf(seen_line+used,sizeof(seen_line)-used,"%d,%d;",origin/4096,origin%4096);
        }
      }
      logger(seen_line);
    }
  }
  static ULONGLONG test_logged=0;
  if (test_mode.load(std::memory_order_relaxed) && now-test_logged>=1000) {
    test_logged=now; char line[128];
    std::snprintf(line,sizeof(line),"hud widget test: mode %u selected=%u emitted=%u hidden=%u",test_mode.load(),test_selected.exchange(0),test_emitted.exchange(0),test_hidden.exchange(0));
    if (logger) logger(line);
    if (test_mode.load(std::memory_order_relaxed)>=7) {
      char layers[256]; int used=std::snprintf(layers,sizeof(layers),"hud layers: routed=%u",test_routed.exchange(0));
      for (unsigned l=0;l<12;++l) {
        unsigned n[3]; for (unsigned e=0;e<3;++e) n[e]=layer_draws[l][e].exchange(0);
        if (n[0]||n[1]||n[2]) used+=std::snprintf(layers+used,sizeof(layers)-used," L%u=%u/%u/%u",l,n[0],n[1],n[2]);
        if (used>200) break;
      }
      if (logger) logger(layers);
    }
  }
  if (stopped.exchange(false) && phase.load()) phase.store(3);
  if (requested.exchange(false)) {
    if (!prepared.load() || !allowed.load()) {
      if (logger) logger("hud ownership: capture unavailable (guards or nouipiece)");
    } else if (phase.load()) {
      if (logger) logger("hud ownership: capture already pending/recording/saving");
    } else if (!activate_hooks()) {
      if (logger) logger("hud ownership: not armed; native observer installation rejected");
    } else if (open_output()) {
      { std::lock_guard<std::mutex> lock(mutex); head=count=total=0;burst=~0u;std::memset(quotas,0,sizeof(quotas));std::memset(seen,0,sizeof(seen)); }
      ++capture_id; started.store(0); waiting_at=now; summaries=0;
      contention=0;overflow=0;throttled=0;stack_overflow=0;repeats=0;
      phase.store(1);
      if (logger) logger("hud ownership: armed; close menu, equip weapon and play 30s; HUD_OWNERSHIP_*.jsonl beside DLL");
    } else if (logger) logger("hud ownership: could not create capture file; not armed");
  }
  const auto begin=started.load();
  if ((phase.load()==1 && now-waiting_at>=WaitMs) || (phase.load()==2 && begin && now-begin>=CaptureMs)) phase.store(3);
  status();
  if (!output) return;
  Event ready[32]; unsigned n=0,queued=0,recorded=0;
  { std::unique_lock<std::mutex> lock(mutex,std::try_to_lock); if (!lock.owns_lock()) return;
    while (count && n<32) {ready[n++]=queue[head];head=(head+1)%QueueSize;--count;}
    queued=count;recorded=total;
  }
  for (unsigned i=0;i<n;++i) write_event(ready[i]);
  if (phase.load()==3 && !queued) {
    std::fprintf(output,"{\"summary\":1,\"session\":%u,\"recorded\":%u,\"contention_drops\":%u,\"queue_drops\":%u,"
      "\"quota_skips\":%u,\"repeat_skips\":%u,\"stack_overflow\":%u,\"started\":%d}\n",
      capture_id.load(),recorded,contention.load(),overflow.load(),throttled.load(),repeats.load(),stack_overflow.load(),begin!=0);
    const bool failed=std::ferror(output)!=0; const int closed=std::fclose(output); output=nullptr;phase.store(0);
    if (logger) logger(failed || closed?"hud ownership: capture file write failed":"hud ownership: capture saved; ownership remains unverified until correlated");
  } else if (++summaries%60==0) std::fflush(output);
}
}
