#include "hand_gestures.hpp"
#include "hand_gesture_pose.hpp"
#include "hand_gesture_context_audit.hpp"
#include "xr_input.hpp"
#include "vr_weapon.hpp"
#include "vr_controls.hpp"
#include "../game/player.hpp"
#include "../game/mount_state.hpp"
#include "../camera/native_stereo.hpp"
#include "../camera/mount_camera.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../script/lua_host.hpp"
#include <windows.h>
#include <atomic>
#include <cstdio>
#include <mutex>
#include <string>

namespace hand_gestures {
namespace {
bool menu_enabled=true; // default on since 2026-10-28 [AUTHOR]; last state persists in AvatarVR_arms.txt
std::atomic<bool> enabled{false};
std::atomic<bool> write_permission{false};
std::mutex mutex;
std::mutex tune_mutex;
Tuning tuning;
std::wstring runtime_dir;
double wall_seconds() {
  FILETIME ft;GetSystemTimeAsFileTime(&ft);ULARGE_INTEGER v;v.LowPart=ft.dwLowDateTime;v.HighPart=ft.dwHighDateTime;
  return static_cast<double>(v.QuadPart)/10000000.-11644473600.;
}
Policy policy;
Frame published;
std::atomic<unsigned> applications{0},rejections{0},hand_applications[2]{},hand_rejections[2]{};
// Capture handshake: on_present arms a request from the tune file; palette
// workers solve; on_present writes the answer once for the tuner.
std::atomic<int> capture_request{-1};
std::mutex capture_mutex;
struct Captured {double id=0,wanted=0;bool ready=false,written=false,mounted=false;int hand=-1;Shape shape;CaptureReport report;};
Captured captured;
struct Audit {
  std::uint64_t at=0,sequence=0;
  const void* owner=nullptr;
  int mode=-1,tool=-1;
  unsigned registry_count=~0u;
  bool guards=false,readable=false;
  ContextAudit candidates;
  PlayerOwnership native;
  Context context;
  float weight[2]{};
  float peak[2][5]{}; // maximum observed per signal since this owner's audit began
  Input input;
} audit;

// User-authorized experimental pose path. Native readers have STATIC provenance
// and byte guards; visual/held-item/cinematic acceptance is still OPEN. Unknown
// reads, native camera/sequence ownership and unknown tools stay fully native.

template<class T> bool read(const void* p,unsigned off,T& out) {
  return p && game::mem_read(static_cast<const unsigned char*>(p)+off,&out,sizeof(out));
}
bool guards() {
  static const bool ok=[] {
    const unsigned char seq[]={0x48,0x8b,0x81,0xf0,0x01,0x00,0x00,0xc3};
    const unsigned char tool[]={0x48,0x89,0x5c,0x24,0x08,0x57,0x48,0x83,0xec,0x20,0x48,0x8b,0xca,0x48,0x8b,0xfa};
    const unsigned char held[]={0x40,0x57,0x48,0x83,0xec,0x20,0x48,0x8b,0x4a,0x20,0x48,0x8b,0xfa,0x48,0x85,0xc9};
    const unsigned char type[]={0x80,0xb9,0x64,0x3f,0x00,0x00,0x00,0x75,0x0a,0xc7,0x02,0x00,0x00,0x00,0x00,0x48};
    const unsigned char control[]={0x48,0x8b,0x81,0x38,0x05,0x00,0x00,0xc3};
    const unsigned char camera[]={0x0f,0xb6,0x81,0x80,0x00,0x00,0x00,0xc3};
    auto match=[](unsigned rva,const auto& bytes) {
      unsigned char got[sizeof(bytes)]{};
      return game::mem_read(game::module_base()+rva,got,sizeof(got)) && !std::memcmp(got,bytes,sizeof(got));
    };
    return match(0x1D5C440,seq) && match(0x1C721A0,tool) && match(0x3F4F630,held) &&
      match(0x27A91A0,type) && match(0x3DEDDE0,control) && match(0x3E3DD60,camera);
  }(); return ok;
}
Context observe(Audit& out,std::uint64_t now) {
  Context c; c.at=now;
  const auto p=game::pointers(); c.identity=reinterpret_cast<std::uint64_t>(p.body);
  c.gameplay=native_stereo::stereo_running() && !native_stereo::front_end() && game::read().valid;
  c.mounted=mount_state::mounted();
  out.owner=p.body; out.at=now; out.guards=guards();
  if(!out.guards || !p.body || !p.holder || !c.gameplay) return c;
  out.candidates=sample_context(p.body,p.holder,[](const void* base,unsigned off,auto& value) {return read(base,off,value);});
  out.tool=out.candidates.tool; out.mode=out.candidates.mode; out.registry_count=out.candidates.registry_count;
  out.readable=out.candidates.readable && game::pointers().body==p.body;
  out.native=sample_player_ownership(p.body,[](const void* base,unsigned off,auto& value) {return read(base,off,value);});
  const auto& native=out.native;
  // Wait out cached old weapon meshes before accepting named-render evidence.
  // A known descriptor can subsequently survive brief missing weapon draws.
  struct Stable {
    const void* owner=nullptr; const void* equipment=nullptr; const void* held=nullptr; const void* definition=nullptr;
    int active=-1,type=-1; std::uint64_t since=0; std::vector<const void*> weapons;
  };
  static Stable stable; // local camera thread only
  if(stable.owner!=p.body) {stable=Stable{}; stable.owner=p.body;}
  if(stable.equipment!=native.equipment || stable.held!=native.held || stable.definition!=native.definition ||
     stable.active!=native.active || stable.type!=native.archetype) {
    stable.equipment=native.equipment; stable.held=native.held; stable.definition=native.definition;
    stable.active=native.active; stable.type=native.archetype; stable.since=now;
  }
  Interactions i; i.stable=stable.since && now>=stable.since && now-stable.since>=300;
  const auto weapon=skin_palette_probe::weapon_capture_state();
  const bool mounted_proof=weapon.mounted_bound && weapon.mounted_owner==p.body && weapon.mounted_candidate==native.equipment;
  // held_weapon_root is only fresh when the game does NOT place the weapon, so it could
  // not prove a placed one: 12 of 13 audited weapon runs never left Item,Item (2026-10-30).
  const bool onfoot_proof=!c.mounted && vr_weapon::weapon_placed_by_game() && skin_palette_probe::held_weapon_drawn();
  if(i.stable && native.active==1 && native.held && native.definition && native.archetype!=14 &&
     (mounted_proof || onfoot_proof) && std::find(stable.weapons.begin(),stable.weapons.end(),native.definition)==stable.weapons.end()) {
    if(stable.weapons.size()>=64) stable.weapons.erase(stable.weapons.begin());
    stable.weapons.push_back(native.definition);
  }
  i.named_weapon=std::find(stable.weapons.begin(),stable.weapons.end(),native.definition)!=stable.weapons.end();
  i.dominant=skin_palette_probe::weapon_hand_index(); i.support=weapon.support_hand;
  i.support_held=weapon.two_hand_held; i.support_only=weapon.support_only; i.swap=weapon.swap_target>=0;
  for(int h=0;h<2;++h) {i.harness[h]=mount_camera::harness_held(h); i.busy[h]=vr_controls::hand_busy(h);}
  const void* equipment=nullptr; const void* held=nullptr;
  const bool same=game::pointers().body==p.body && read(p.body,0x250,equipment) && equipment==native.equipment &&
    equipment && read(equipment,0x3E40,held) && held==native.held;
  c=resolve_context(c,out.candidates,native,i);
  if(!same || !out.readable || mount_state::get().phase==mount_state::Phase::Mounting ||
     mount_state::get().phase==mount_state::Phase::Dismounting) {c.verified=false; c.cinematic=true;}
  // body+0x94 = world position (SDK_LEDGER 0x4271680, LIVE). Same thread as above.
  static MotionGate motion; static const void* motion_owner=nullptr;
  if(motion_owner!=p.body) {motion=MotionGate{};motion_owner=p.body;}
  float position[3]{};xr_input::PadState pad;
  const bool located=read(p.body,0x94,position);
  const float stick=xr_input::latest(&pad)?std::sqrt(pad.lx*pad.lx+pad.ly*pad.ly):0.f;
  c.moving=motion.step(located?position:nullptr,c.mounted,stick,now);
  return c;
}
void log_audit(const Audit& a) {
  static std::wstring path=[] {
    HMODULE self=nullptr; wchar_t file[MAX_PATH]{};
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
      reinterpret_cast<LPCWSTR>(&log_audit),&self);
    GetModuleFileNameW(self,file,MAX_PATH); std::wstring s=file;
    return s.substr(0,s.find_last_of(L"\\/"))+L"\\HAND_GESTURES.log";
  }();
  const HANDLE f=CreateFileW(path.c_str(),FILE_APPEND_DATA,FILE_SHARE_READ,nullptr,OPEN_ALWAYS,0,nullptr);
  if(f==INVALID_HANDLE_VALUE) return;
  char text[2200];
  auto masks=[](const Sensors& s,bool supported) {
    const Signal* fields[]={&s.grip,&s.trigger,&s.index_touch,&s.thumb_touch,&s.thumb_press};
    unsigned mask=0; for(unsigned i=0;i<5;++i) if(supported?fields[i]->supported:fields[i]->active) mask|=1u<<i;
    return mask;
  };
  const auto weapon=skin_palette_probe::weapon_capture_state();
  const int n=std::snprintf(text,sizeof(text),"audit_version=5 pid=%lu at=%llu experimental=1 owner=%p guards=%d readable=%d mode=%d tool=%d registry_count=%u sensor_seq=%llu focused=%d "
    "L=%.3f,%.3f,%.0f,%.0f,%.0f tracked=%d supported=%u active=%u R=%.3f,%.3f,%.0f,%.0f,%.0f tracked=%d supported=%u active=%u "
    "mounted=%d harness=%d,%d weapon_recent=%d weapon_hand=%d support=%d support_hand=%d support_only=%d swap=%d "
    "read_mask=%u registry=%p tool_component=%p camera_mode=%p peakL=%.3f,%.3f,%.0f,%.0f,%.0f peakR=%.3f,%.3f,%.0f,%.0f,%.0f "
    "context_ready=%d native_owned=%d active=%d archetype=%d holster=%d camera_override=%d equipment=%p held=%p owners=%d,%d weight=%.3f,%.3f applications=%u rejects=%u\r\n",
    GetCurrentProcessId(),a.at,a.owner,a.guards,a.readable,a.mode,a.tool,a.registry_count,a.input.sequence,a.input.focused,
    value(a.input.hand[0].grip),value(a.input.hand[0].trigger),value(a.input.hand[0].index_touch),value(a.input.hand[0].thumb_touch),value(a.input.hand[0].thumb_press),a.input.hand[0].tracked,masks(a.input.hand[0],true),masks(a.input.hand[0],false),
    value(a.input.hand[1].grip),value(a.input.hand[1].trigger),value(a.input.hand[1].index_touch),value(a.input.hand[1].thumb_touch),value(a.input.hand[1].thumb_press),a.input.hand[1].tracked,masks(a.input.hand[1],true),masks(a.input.hand[1],false),
    mount_state::mounted(),mount_camera::harness_held(0),mount_camera::harness_held(1),vr_weapon::weapon_placed_by_game(),
    skin_palette_probe::weapon_hand_index(),weapon.two_hand_held,weapon.support_hand,weapon.support_only,weapon.swap_target,
    a.candidates.read_mask,a.candidates.registry,a.candidates.tool_component,a.candidates.camera_mode,
    a.peak[0][0],a.peak[0][1],a.peak[0][2],a.peak[0][3],a.peak[0][4],
    a.peak[1][0],a.peak[1][1],a.peak[1][2],a.peak[1][3],a.peak[1][4],
    a.context.verified,a.context.cinematic,a.native.active,a.native.archetype,a.native.transition_state,a.native.camera_override,
    a.native.equipment,a.native.held,static_cast<int>(a.context.owner[0]),static_cast<int>(a.context.owner[1]),
    a.weight[0],a.weight[1],applications.load(std::memory_order_relaxed),rejections.load(std::memory_order_relaxed));
  DWORD wrote=0; if(n>0) WriteFile(f,text,static_cast<DWORD>(std::min<int>(n,sizeof(text)-1)),&wrote,nullptr); CloseHandle(f);
}
} // namespace
bool* enabled_flag() {return &menu_enabled;}
void on_present() {
  enabled.store(menu_enabled,std::memory_order_release);
  write_permission.store(*lua_host::writes_flag(),std::memory_order_release);
  static std::uint64_t tune_next=0;
  const auto tick=GetTickCount64();
  if(menu_enabled && tick>=tune_next) {
    tune_next=tick+250;
    if(runtime_dir.empty()) {
      HMODULE self=nullptr;wchar_t file[MAX_PATH]{};
      GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        reinterpret_cast<LPCWSTR>(&on_present),&self);
      GetModuleFileNameW(self,file,MAX_PATH);runtime_dir=file;
      runtime_dir=runtime_dir.substr(0,runtime_dir.find_last_of(L"\\/"));
    }
    FILE* f=nullptr;std::string text;
    if(_wfopen_s(&f,(runtime_dir+L"\\AvatarVR_ik_tune.txt").c_str(),L"rb")==0 && f) {
      char bytes[4096];std::size_t size=0;
      while((size=std::fread(bytes,1,sizeof(bytes),f))>0 && text.size()<1048576) text.append(bytes,size);
      std::fclose(f);
    }
    static std::string prior;static bool initialized=false;
    if(!initialized || text!=prior) {
      initialized=true;prior=text;Tuning next=parse_gesture_tune(text);
      std::lock_guard<std::mutex> lock(tune_mutex);tuning=next;
    }
    {
      Tuning settings;{std::lock_guard<std::mutex> lock(tune_mutex);settings=tuning;}
      std::lock_guard<std::mutex> lock(capture_mutex);
      const bool live=settings.capture_hand>=0 && settings.capture_id!=0 && wall_seconds()<settings.capture_until;
      if(live && settings.capture_id!=captured.wanted) {captured=Captured{};captured.wanted=settings.capture_id;}
      capture_request.store(live && !captured.ready?settings.capture_hand:-1,std::memory_order_release);
      if(captured.ready && !captured.written) {
        captured.written=true;
        const std::wstring path=runtime_dir+L"\\AvatarVR_gesture_capture.json";
        if(_wfopen_s(&f,(path+L".tmp").c_str(),L"wb")==0 && f) {
          static const char* fingers[]={"thumb","index","middle","pinky"};
          std::fprintf(f,"{\"id\":%.17g,\"at\":%.3f,\"hand\":\"%s\",\"context\":\"%s\",\"position_mm\":%.3f,\"angle_deg\":%.3f,"
            "\"clamped\":%d,\"helpers\":%d,\"helpers_skipped\":%d,\"values\":{",captured.id,wall_seconds(),
            captured.hand?"right":"left",captured.mounted?"tp":"fp",captured.report.position_mm,captured.report.angle_deg,
            captured.report.clamped,captured.report.helpers,captured.report.helpers_skipped);
          const char* comma="";
          for(int k=0;k<4;++k) {
            const auto& s=captured.shape.finger[k];
            for(int j=0;j<3;++j) {std::fprintf(f,"%s\"%s.bend%d\":%.4f",comma,fingers[k],j+1,s.bend[j]);comma=",";}
            std::fprintf(f,",\"%s.splay\":%.4f,\"%s.roll\":%.4f",fingers[k],s.splay,fingers[k],s.roll);
            for(int e=0;e<extra_count;++e) std::fprintf(f,",\"%s.%s\":%.4f",fingers[k],extra_name(e),s.extra[e]);
          }
          for(int b=0;b<thumb_helper_count;++b) for(int a=0;a<helper_axis_count;++a)
            std::fprintf(f,",\"%s.%s\":%.4f",helper_name(b),helper_axis_name(a),captured.shape.helper[b][a]);
          std::fprintf(f,"}}\n");std::fclose(f);
          MoveFileExW((path+L".tmp").c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING);
        }
      }
    }
    Audit copy;Frame pose;
    {std::lock_guard<std::mutex> lock(mutex);copy=audit;pose=published;}
    // Tuner reads status only; this file never drives gameplay or skeleton writes.
    if(_wfopen_s(&f,(runtime_dir+L"\\AvatarVR_gesture_status.json").c_str(),L"wb")==0 && f) {
      std::fprintf(f,"{\"at\":%.3f,\"moving\":%d,\"mounted\":%d,\"context_ready\":%d,\"native_owned\":%d,"
        "\"owners\":[%d,%d],\"weight\":[%.3f,%.3f],\"combo\":[%d,%d],"
        "\"grip\":[%.3f,%.3f],\"trigger\":[%.3f,%.3f],\"index_touch\":[%.0f,%.0f],"
        "\"thumb_touch\":[%.0f,%.0f],\"tracked\":[%d,%d],\"hand_applications\":[%u,%u],\"hand_rejects\":[%u,%u],"
        "\"applications\":%u,\"rejects\":%u}\n",
        wall_seconds(),copy.context.moving,copy.context.mounted,copy.context.verified,copy.context.cinematic,
        static_cast<int>(copy.context.owner[0]),static_cast<int>(copy.context.owner[1]),
        pose.hand[0].weight,pose.hand[1].weight,pose.hand[0].combination,pose.hand[1].combination,
        value(copy.input.hand[0].grip),value(copy.input.hand[1].grip),value(copy.input.hand[0].trigger),value(copy.input.hand[1].trigger),
        value(copy.input.hand[0].index_touch),value(copy.input.hand[1].index_touch),
        value(copy.input.hand[0].thumb_touch),value(copy.input.hand[1].thumb_touch),
        copy.input.hand[0].tracked,copy.input.hand[1].tracked,
        hand_applications[0].load(std::memory_order_relaxed),hand_applications[1].load(std::memory_order_relaxed),
        hand_rejections[0].load(std::memory_order_relaxed),hand_rejections[1].load(std::memory_order_relaxed),
        applications.load(std::memory_order_relaxed),rejections.load(std::memory_order_relaxed));
      std::fclose(f);
    }
  }
  // Bounded read-only audit: 120 changed camera snapshots, <=1/s per launch.
  static unsigned lines=0; static std::uint64_t next=0,last_sequence=0;
  const auto now=GetTickCount64();
  if(!menu_enabled || lines>=120 || now<next) return;
  Audit copy; {std::lock_guard<std::mutex> lock(mutex); copy=audit;}
  if(!fresh(now,copy.at) || copy.sequence==last_sequence) return;
  log_audit(copy); ++lines; next=now+1000; last_sequence=copy.sequence;
}
void on_camera_post() {
  if(!enabled.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lock(mutex); policy=Policy{}; published={}; return;
  }
  const auto now=GetTickCount64(); Audit next;
  const Context c=observe(next,now);
  xr_input::sensors(&next.input);
  Tuning settings;{std::lock_guard<std::mutex> lock(tune_mutex);settings=tuning;}
  std::lock_guard<std::mutex> lock(mutex);
  if(audit.owner==next.owner) std::memcpy(next.peak,audit.peak,sizeof(next.peak));
  if(next.input.focused) for(int h=0;h<2;++h) {
    const auto& s=next.input.hand[h];
    const Signal* signals[]={&s.grip,&s.trigger,&s.index_touch,&s.thumb_touch,&s.thumb_press};
    for(int f=0;f<5;++f) next.peak[h][f]=std::max(next.peak[h][f],value(*signals[f]));
  }
  next.sequence=audit.sequence+1; next.context=c;
  published=policy.step(next.input,c,now,write_permission.load(std::memory_order_acquire),&settings,wall_seconds());
  for(int h=0;h<2;++h) next.weight[h]=published.hand[h].weight;
  audit=next;
}
Frame latest() {
  if(!enabled.load(std::memory_order_acquire) || !write_permission.load(std::memory_order_acquire)) return {};
  std::lock_guard<std::mutex> lock(mutex);
  return fresh(GetTickCount64(),published.at)?published:Frame{};
}
bool apply(float (*mats)[16],unsigned n,int hand,const Pose& pose,const HandReference& reference) {
  const bool ok=pose_hand(mats,n,hand,pose,reference);
  (ok?applications:rejections).fetch_add(1,std::memory_order_relaxed);
  if(hand==0 || hand==1) (ok?hand_applications[hand]:hand_rejections[hand]).fetch_add(1,std::memory_order_relaxed);
  return ok;
}
bool calibrate(HandReference& out,const float (*mats)[16],unsigned n,const char* const* names,int hand,bool mounted,bool allow_new) {
  static std::mutex file_mutex;static bool loaded=false;static std::vector<std::string> lines;
  HandReference fresh_reference;
  if(!capture_hand(fresh_reference,mats,n,names,hand)) return false;
  HMODULE self=nullptr;wchar_t file[MAX_PATH]{};
  GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&calibrate),&self);
  GetModuleFileNameW(self,file,MAX_PATH);std::wstring path=file;
  path=path.substr(0,path.find_last_of(L"\\/"))+L"\\AvatarVR_hand_references.txt";
  char key[64];
  std::snprintf(key,sizeof(key),"%016llx %u %d %d ",static_cast<unsigned long long>(rig_hash(names,n)),n,mounted?1:0,hand);
  std::lock_guard<std::mutex> lock(file_mutex);
  if(!loaded) {
    loaded=true;std::string text;FILE* f=nullptr;
    if(_wfopen_s(&f,path.c_str(),L"rb")==0 && f) {
      char bytes[4096];std::size_t size=0;
      while((size=std::fread(bytes,1,sizeof(bytes),f))>0) text.append(bytes,size);
      std::fclose(f);
    }
    std::istringstream all(text);std::string line;
    while(std::getline(all,line)) if(!line.empty()) lines.push_back(line);
  }
  unsigned entries=0;
  for(const auto& line:lines) if(!line.compare(0,std::strlen(key),key)) {
    ++entries;HandReference saved=fresh_reference;
    if(reference_from_text(line.substr(std::strlen(key)),saved) && reference_fits(saved,mats,n)) {out=std::move(saved);return true;}
  }
  if(!allow_new) return false;
  out=fresh_reference;
  if(entries<8) { // distinct mesh proportions sharing one skeleton; bounded
    lines.push_back(key+reference_text(fresh_reference));
    FILE* f=nullptr;
    if(_wfopen_s(&f,path.c_str(),L"ab")==0 && f) {std::fprintf(f,"%s\n",lines.back().c_str());std::fclose(f);}
  }
  return true;
}
int capture_wanted() {return capture_request.load(std::memory_order_acquire);}
void capture(const float (*mats)[16],unsigned n,int hand,bool mounted,const HandReference& reference) {
  if(hand!=capture_request.load(std::memory_order_acquire)) return;
  Shape shape;CaptureReport report;
  if(!solve_shape(mats,n,hand,reference,shape,report)) return;
  std::lock_guard<std::mutex> lock(capture_mutex);
  if(captured.ready || captured.wanted==0) return;
  captured.id=captured.wanted;captured.ready=true;captured.hand=hand;captured.mounted=mounted;
  captured.shape=shape;captured.report=report;
  capture_request.store(-1,std::memory_order_release);
}
bool native_base_required() {return latest().at!=0;}
} // namespace hand_gestures
