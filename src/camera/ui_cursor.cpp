#include "ui_cursor.hpp"
#include "native_stereo.hpp"
#include "ui_cursor_math.hpp"
#include "../vr/menu_pointer.hpp"
#include <windows.h>
#include <MinHook.h>
#include <intrin.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <cstdint>

namespace {
using Local = void* (*)(void*, void*, const float*);
using Update = std::uintptr_t (*)(void*);
using Select = void* (*)(void*);
using Pick = void* (*)(void*,const float*);
using Node = void (*)(void*, void*);
using Slot = unsigned (*)();
Local g_local = nullptr;
Update g_update = nullptr;
Select g_select = nullptr;
Pick g_pick = nullptr;
std::atomic<bool> g_enabled{true};
std::atomic<unsigned long long> g_corrected{0};
Node g_radius = nullptr;
Slot g_slot = nullptr;
unsigned char* g_base = nullptr;
ui_cursor::Log g_log = nullptr;
std::atomic<ULONGLONG> g_start{0};
std::atomic<unsigned> g_budget{0};
std::atomic<unsigned long long> g_calls[4]{};
std::mutex g_mutex;
thread_local int t_selected_w=0, t_selected_h=0;

bool read(const void* p, void* out, size_t n) {
  if (!p) return false;
  __try { std::memcpy(out,p,n); return true; }
  __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
}
template<class T> bool field(void* p, size_t off, T* out) {
  return p && read(static_cast<unsigned char*>(p)+off,out,sizeof(T));
}
bool active() {
  if (!native_stereo::ui_layer_active()) return false;
  const auto now=GetTickCount64();
  const auto start=g_start.load(std::memory_order_relaxed);
  return start && now-start<90000 && g_budget.load(std::memory_order_relaxed)<192;
}
struct Sample {
  char kind=0;
  std::uintptr_t caller=0;
  void* object=nullptr;
  void* hover=nullptr;
  int w=0,h=0;
  float before[2]{},point[2]{},local[2]{},matrix[16]{},bounds[4]{},radius=0;
  unsigned char valid=0;
  bool have_matrix=false,have_bounds=false;
  int context[8]{};
};
Sample g_pending[32];
unsigned g_pending_count=0;
// Rate limits are per lane/widget; retain several buttons rather than sampling
// only the first widget in every hit-test traversal. Never wait on game input.
struct Seen { char kind=0; void* object=nullptr; ULONGLONG last=0; };
Seen g_seen[32];
bool claim(char kind,void* object) {
  std::unique_lock<std::mutex> lock(g_mutex,std::try_to_lock);
  if (!lock.owns_lock() || g_pending_count>=32 || g_budget.load()>=192) return false;
  const auto now=GetTickCount64();
  for (auto& x:g_seen) {
    if (x.kind==kind && x.object==object) {
      if (now-x.last<2000) return false;
      x.last=now; ++g_budget; return true;
    }
  }
  for (auto& x:g_seen) if (!x.kind) {
    x={kind,object,now}; ++g_budget; return true;
  }
  return false;
}
void queue(const Sample& s) {
  std::unique_lock<std::mutex> lock(g_mutex,std::try_to_lock);
  if (lock.owns_lock() && g_pending_count<32) g_pending[g_pending_count++]=s;
}
void* Local_hook(void* widget,void* out,const float* point) {
  const auto caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(g_base);
  // Static readable callers: polygon hit tests, cursor picking and graph queries.
  const bool known=caller==0x26E3BD || caller==0x22C058 || caller==0x235AB1 || caller==0x601F2D;
  const bool collect=known && active();
  if (collect) ++g_calls[0];
  Sample s;
  const bool capture=collect && claim('H',widget);
  if (capture) { s.kind='H'; s.object=widget; s.caller=caller; read(point,s.point,sizeof(s.point)); }
  void* result=g_local(widget,out,point);
  if (capture) {
    read(out,s.local,sizeof(s.local)); field(out,8,&s.valid);
    s.have_matrix=widget && read(static_cast<unsigned char*>(widget)+0x60,s.matrix,sizeof(s.matrix));
    void* shapes=nullptr; unsigned count=0;
    // STATIC5553F0: shape records are32 bytes; local bounding rectangle at+10.
    if (field(widget,0x100,&shapes) && shapes && field(widget,0x110,&count) && count>0 && count<1024)
      s.have_bounds=read(static_cast<unsigned char*>(shapes)+0x10,s.bounds,sizeof(s.bounds));
    queue(s);
  }
  return result;
}
std::atomic<int> g_view_w{0},g_view_h{0};  // the view the updater picks for its cursor range
// Controller laser (menu_pointer.hpp): place the native cursor before the updater picks from root+5E0. STATIC 2D7A2F/2D7B66:
// mode 2 picks at 5E0, copies it to 5C0, adds the stick, clamps to the view and stores it back. Pixels from the lower left,
// like the stick's own (LIVE 2026-10-13); the picker hook above still converts them when the UI layer is up.
void laser_cursor(void* root) {
  unsigned mode=0;
  if (!read(g_base+0x96A5CF0,&mode,sizeof(mode)) || mode!=2) return;
  const auto now=GetTickCount64();
  // Stamped with the laser off too: native_stereo's front-end tracker uses it to recognise the 3D main menu (2026-11-08).
  menu_pointer::g_native_cursor_at.store(now,std::memory_order_relaxed);
  if (!menu_pointer::g_enabled.load(std::memory_order_relaxed)) return;
  float u=0,v=0;
  const int w=g_view_w.load(std::memory_order_relaxed),h=g_view_h.load(std::memory_order_relaxed);
  if (w<=0 || h<=0 || !menu_pointer::cursor(&u,&v,now)) return;
  const float p[2]={u*static_cast<float>(w),v*static_cast<float>(h)};
  __try { std::memcpy(static_cast<unsigned char*>(root)+0x5E0,p,sizeof(p)); }
  __except(EXCEPTION_EXECUTE_HANDLER) {}
}
std::uintptr_t Update_hook(void* root) {
  laser_cursor(root);
  // Start on actual native stick motion, so the startup video cannot consume
  // the bounded recording interval before the user reaches the menu.
  const bool can_start=!g_start.load(std::memory_order_relaxed) && native_stereo::ui_layer_active();
  float before[2]{};
  if (can_start || active()) field(root,0x5E0,&before);
  const auto result=g_update(root);
  // STATIC 2D7AD7/2D7AFF: B8 takes the picked widget, or B0 (the root's fallback) when the pick finds nothing.
  void* hovered=nullptr; void* fallback=nullptr;
  if (field(root,0xB8,&hovered) && field(root,0xB0,&fallback) && hovered && hovered!=fallback)
    menu_pointer::g_hover_at.store(GetTickCount64(),std::memory_order_relaxed);
  if (can_start) {
    float velocity[2]{};
    // STATIC2D7D9F: updater writes the native cursor's stick displacement here.
    if (field(root,0x650,&velocity) && std::isfinite(velocity[0]) && std::isfinite(velocity[1]) &&
        (std::fabs(velocity[0])>.01f || std::fabs(velocity[1])>.01f)) {
      ULONGLONG zero=0;
      g_start.compare_exchange_strong(zero,GetTickCount64());
    }
  }
  if (active()) {
    ++g_calls[1];
    if (!claim('U',root)) return result;
    Sample s; s.kind='U'; s.object=root;
    std::memcpy(s.before,before,sizeof(before));
    field(root,0x5E0,&s.point); field(root,0x5C0,&s.local); field(root,0xB8,&s.hover);
    s.w=t_selected_w; s.h=t_selected_h; queue(s);
  }
  return result;
}
void* Select_hook(void* manager) {
  void* result=g_select(manager);
  const auto caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(g_base);
  if (caller==0x2D79C2) {
    void* camera=nullptr; int rect[4]{};
    if (field(result,0x10,&camera) && camera && read(static_cast<unsigned char*>(camera)+0x244,rect,sizeof(rect))) {
      g_view_w.store(rect[2]-rect[0],std::memory_order_relaxed); g_view_h.store(rect[3]-rect[1],std::memory_order_relaxed);
    }
  }
  if (caller==0x2D79C2 && active()) {
    ++g_calls[2];
    void* camera=nullptr; int rect[4]{};
    if (field(result,0x10,&camera) && camera && read(static_cast<unsigned char*>(camera)+0x244,rect,sizeof(rect))) {
      t_selected_w=rect[2]-rect[0]; t_selected_h=rect[3]-rect[1];
    } else { t_selected_w=t_selected_h=0; }
  }
  return result;
}
void Radius_hook(void* self,void* params) {
  g_radius(self,params);
  if (!active()) return;
  ++g_calls[3];
  const unsigned slot=g_slot();
  void* context=nullptr; void* widget=nullptr;
  if (slot>=256 || !read(g_base+0x96AB220+sizeof(void*)*slot,&context,sizeof(context)) || !context) return;
  field(context,0x68,&widget);
  if (!claim('R',widget ? widget : context)) return;
  Sample s; s.kind='R'; s.object=widget;
  read(g_base+0x8DB74F0,&s.radius,sizeof(s.radius));
  read(static_cast<unsigned char*>(context)+0x304,s.context,sizeof(s.context));
  if (widget) s.have_matrix=read(static_cast<unsigned char*>(widget)+0x60,s.matrix,sizeof(s.matrix));
  queue(s);
}

void* Pick_hook(void* root,const float* point) {
  const auto caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(g_base);
  unsigned mode=0; float w=0,h=0,input[2]{},mapped[2]{}; std::uint32_t ew=0,eh=0;
  // STATIC2D7A39: dedicated virtual-cursor pick branch, mode2. Mouse takes a
  // different branch (26E260); never change its inputs or native cursor state.
  if (caller!=0x2D7A3E || !g_enabled.load(std::memory_order_relaxed) ||
      !native_stereo::ui_layer_active() || !read(g_base+0x96A5CF0,&mode,sizeof(mode)) || mode!=2 ||
      !native_stereo::eye_size(&ew,&eh) || !native_stereo::ui_canvas_size(&w,&h) ||
      !read(point,input,sizeof(input)) || !ui_cursor::layout_point(input,static_cast<float>(ew),static_cast<float>(eh),w,h,mapped))
    return g_pick(root,point);
  ++g_corrected;
  // Original traversal consumes this two-float pointer synchronously. Its local
  // projection, clip-plane/polygon tests and hover arbitration all see one basis.
  return g_pick(root,mapped);
}
}

namespace {
// Read-only crash receipt, scripts\AvatarVR.log ([CRASH_TRACE]): every first-chance fault up to a limit, so the last line before a silent
// death names the code. Faults the game or the mod's own __try reads handle appear here too. An exit line = no fault killed it.
HANDLE g_trace=INVALID_HANDLE_VALUE;
std::atomic<unsigned> g_trace_lines{0};
void trace_text(const char* text,int n) {
  if (g_trace == INVALID_HANDLE_VALUE || n <= 0) return;
  constexpr char tag[] = "[CRASH_TRACE] ";
  char line[320];
  constexpr auto prefix = static_cast<int>(sizeof(tag) - 1);
  if (n > 255) n = 255;
  std::memcpy(line, tag, prefix);
  std::memcpy(line + prefix, text, n);
  DWORD done = 0;
  WriteFile(g_trace, line, static_cast<DWORD>(prefix + n), &done, nullptr);
}
LONG CALLBACK fault_trace(EXCEPTION_POINTERS* e) {
  const DWORD code=e->ExceptionRecord->ExceptionCode;
  if (code!=EXCEPTION_ACCESS_VIOLATION && code!=EXCEPTION_ILLEGAL_INSTRUCTION && code!=EXCEPTION_STACK_OVERFLOW &&
      code!=EXCEPTION_INT_DIVIDE_BY_ZERO && code!=EXCEPTION_PRIV_INSTRUCTION && code!=STATUS_HEAP_CORRUPTION &&
      code!=STATUS_STACK_BUFFER_OVERRUN) return EXCEPTION_CONTINUE_SEARCH;
  const auto rip=e->ContextRecord->Rip;
  // A repeating handled fault (a guarded read scan) must not use up the lines before the real one: 4 per address.
  static std::atomic<std::uint64_t> sites[64]{}; static std::atomic<unsigned> site_lines[64]{};
  for (auto i=(rip>>2)%64,probe=0ull;probe<64;++probe,i=(i+1)%64) {
    std::uint64_t seen=sites[i].load(std::memory_order_relaxed);
    if (!seen && sites[i].compare_exchange_strong(seen,rip)) seen=rip;
    if (seen!=rip) continue;
    if (site_lines[i].fetch_add(1,std::memory_order_relaxed)>=4) return EXCEPTION_CONTINUE_SEARCH;
    break;
  }
  if (g_trace_lines.fetch_add(1,std::memory_order_relaxed)>=512) return EXCEPTION_CONTINUE_SEARCH;
  const auto game=reinterpret_cast<std::uintptr_t>(g_base);
  HMODULE own=nullptr; GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCSTR>(&fault_trace),&own);
  const auto mod=reinterpret_cast<std::uintptr_t>(own);
  char line[256];
  int n=std::snprintf(line,sizeof(line),"%llu tid=%lu code=%08lX rip=%llX game+%llX mod+%llX access=%llu addr=%llX\r\n",
    static_cast<unsigned long long>(GetTickCount64()),GetCurrentThreadId(),code,static_cast<unsigned long long>(rip),
    static_cast<unsigned long long>(rip-game),static_cast<unsigned long long>(rip-mod),
    static_cast<unsigned long long>(e->ExceptionRecord->NumberParameters>0?e->ExceptionRecord->ExceptionInformation[0]:0),
    static_cast<unsigned long long>(e->ExceptionRecord->NumberParameters>1?e->ExceptionRecord->ExceptionInformation[1]:0));
  trace_text(line,n);
  // Likely callers: stack slots pointing into the mod or near the game's code.
  const auto* stack=reinterpret_cast<const std::uint64_t*>(e->ContextRecord->Rsp);
  int found=0;
  for (int i=0;i<256 && found<10;++i) {
    std::uint64_t v=0;
    if (!read(stack+i,&v,sizeof(v))) break;
    const bool in_game=v>=game && v<game+0x16000000;
    const bool in_mod=v>=mod && v<mod+0x1000000;
    if (!in_game && !in_mod) continue;
    n=std::snprintf(line,sizeof(line),"  stack+%X %s+%llX\r\n",i*8,in_mod?"mod":"game",static_cast<unsigned long long>(v-(in_mod?mod:game)));
    trace_text(line,n);
    ++found;
  }
  return EXCEPTION_CONTINUE_SEARCH;
}
void trace_exit() { const char t[]="process exit path ran (no unhandled fault ended the game)\r\n"; trace_text(t,sizeof(t)-1); }
void fault_trace_install() {
  char path[MAX_PATH]{};
  HMODULE own=nullptr;
  if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCSTR>(&fault_trace),&own) ||
      !GetModuleFileNameA(own,path,MAX_PATH)) return;
  char* slash=std::strrchr(path,'\\');
  if (!slash || static_cast<size_t>(slash-path)+20>=MAX_PATH) return;
  std::memcpy(slash + 1, "AvatarVR.log", sizeof("AvatarVR.log"));
  g_trace=CreateFileA(path, FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                      OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (g_trace==INVALID_HANDLE_VALUE) return;
  char head[96];
  trace_text(head,std::snprintf(head,sizeof(head),"game base %p, tick %llu\r\n",g_base,static_cast<unsigned long long>(GetTickCount64())));
  AddVectoredExceptionHandler(0,fault_trace);
  std::atexit(trace_exit);
}
}

namespace ui_cursor {
void set_enabled(bool enabled) { g_enabled.store(enabled,std::memory_order_relaxed); }
void install(unsigned char* base,Log log) {
  g_base=base; g_log=log;
  fault_trace_install();
  static const unsigned char local[]={0x40,0x57,0x48,0x83,0xEC,0x40,0x48,0x89,0x5C,0x24,0x50,0x48,0x8B,0xFA,0x49,0x8B,0xD8};
  static const unsigned char update[]={0x48,0x8B,0xC4,0x55,0x53,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57};
  static const unsigned char select[]={0x40,0x53,0x8B,0x59,0x38,0x45,0x33,0xD2,0x45,0x8B,0xDA,0x41,0x8B,0xD2};
  static const unsigned char radius[]={0x48,0x83,0xEC,0x38,0x48,0x8B,0x4A,0x20,0x48,0x85,0xC9,0x74,0x30};
  static const unsigned char slot[]={0x41,0x56,0x48,0x83,0xEC,0x50,0x65,0x48,0x8B,0x04,0x25,0x58,0,0,0};
  static const unsigned char pick[]={0x48,0x8B,0xC4,0x55,0x53,0x56,0x57,0x48,0x8D,0x6C,0x24,0x88,0x48,0x81,0xEC,0x78,0x01,0,0};
  struct Hook { size_t rva; const unsigned char* sig; size_t size; void* detour; void** original; };
  Hook hooks[]={
    {0x554E10,local,sizeof(local),reinterpret_cast<void*>(&Local_hook),reinterpret_cast<void**>(&g_local)},
    {0x2D7830,update,sizeof(update),reinterpret_cast<void*>(&Update_hook),reinterpret_cast<void**>(&g_update)},
    {0x54D150,select,sizeof(select),reinterpret_cast<void*>(&Select_hook),reinterpret_cast<void**>(&g_select)},
    {0x633640,radius,sizeof(radius),reinterpret_cast<void*>(&Radius_hook),reinterpret_cast<void**>(&g_radius)},
    {0x26E600,pick,sizeof(pick),reinterpret_cast<void*>(&Pick_hook),reinterpret_cast<void**>(&g_pick)}
  };
  unsigned char got[32];
  if (!read(base+0x32C77A0,got,sizeof(slot)) || std::memcmp(got,slot,sizeof(slot))) {
    if (log) log("ui cursor: TLS guard mismatch; read-only trace disabled"); return;
  }
  for (const auto& h:hooks) if (!read(base+h.rva,got,h.size) || std::memcmp(got,h.sig,h.size)) {
    if (log) log("ui cursor: code guard mismatch; read-only trace disabled"); return;
  }
  g_slot=reinterpret_cast<Slot>(base+0x32C77A0);
  unsigned made=0,enabled=0;
  for (;made<5;++made) if (MH_CreateHook(base+hooks[made].rva,hooks[made].detour,hooks[made].original)!=MH_OK) break;
  if (made==5) for (;enabled<5;++enabled) if (MH_EnableHook(base+hooks[enabled].rva)!=MH_OK) break;
  if (made!=5 || enabled!=5) {
    for (unsigned i=0;i<enabled;++i) MH_DisableHook(base+hooks[i].rva);
    for (unsigned i=0;i<made;++i) MH_RemoveHook(base+hooks[i].rva);
    if (log) log("ui cursor: hook set rolled back; trace disabled"); return;
  }
  if (log) log("ui cursor: stick-only eye-to-layout hit-test mapping enabled (nocursorfix disables); bounded trace awaits stick motion");
}
void report() {
  static ULONGLONG last=0;
  const auto now=GetTickCount64();
  if (!g_log || !g_start.load() || now-last<2000) return;
  last=now;
  static unsigned long long reported=0;
  const auto corrected=g_corrected.load();
  if (corrected!=reported) {
    reported=corrected;
    float w=0,h=0; std::uint32_t ew=0,eh=0;
    native_stereo::ui_canvas_size(&w,&h); native_stereo::eye_size(&ew,&eh);
    char status[200];
    std::snprintf(status,sizeof(status),"ui cursor mapping: eye=%ux%u logical=%.0fx%.0f picks=%llu enabled=%d (mouse unchanged)",ew,eh,w,h,corrected,g_enabled.load());
    g_log(status);
  }
  Sample batch[32]; unsigned n=0;
  {
    std::unique_lock<std::mutex> lock(g_mutex,std::try_to_lock);
    if (!lock.owns_lock()) return;
    n=g_pending_count; std::memcpy(batch,g_pending,n*sizeof(Sample)); g_pending_count=0;
  }
  char line[850];
  for (unsigned i=0;i<n;++i) {
    const auto& s=batch[i];
    std::snprintf(line,sizeof(line),"ui cursor %c: obj=%p caller=+%llX selected=%dx%d point=%.3f,%.3f before=%.3f,%.3f local=%.3f,%.3f valid=%u hover=%p radius=%.6g context window=%d,%d,%d,%d draw=%d,%d,%d,%d bounds=%d %.3f,%.3f,%.3f,%.3f",
      s.kind,s.object,static_cast<unsigned long long>(s.caller),s.w,s.h,s.point[0],s.point[1],s.before[0],s.before[1],s.local[0],s.local[1],s.valid,s.hover,s.radius,
      s.context[0],s.context[1],s.context[2],s.context[3],s.context[4],s.context[5],s.context[6],s.context[7],s.have_bounds,s.bounds[0],s.bounds[1],s.bounds[2],s.bounds[3]);
    g_log(line);
    if (s.have_matrix) {
      std::snprintf(line,sizeof(line),"ui cursor %c matrix: obj=%p +60 [%.6g %.6g %.6g %.6g] [%.6g %.6g %.6g %.6g] [%.6g %.6g %.6g %.6g] [%.6g %.6g %.6g %.6g]",
        s.kind,s.object,s.matrix[0],s.matrix[1],s.matrix[2],s.matrix[3],s.matrix[4],s.matrix[5],s.matrix[6],s.matrix[7],s.matrix[8],s.matrix[9],s.matrix[10],s.matrix[11],s.matrix[12],s.matrix[13],s.matrix[14],s.matrix[15]);
      g_log(line);
    }
  }
  static bool ended=false;
  if (!ended && (now-g_start.load()>=90000 || g_budget.load()>=192)) {
    ended=true;
    std::snprintf(line,sizeof(line),"ui cursor: trace finished, helper/update/select/radius calls=%llu/%llu/%llu/%llu, samples=%u; native cursor state preserved",
      g_calls[0].load(),g_calls[1].load(),g_calls[2].load(),g_calls[3].load(),g_budget.load());
    g_log(line);
  }
}
}
