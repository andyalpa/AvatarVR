#include "pose_phase_probe.hpp"
#include "../game/player.hpp"
#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace {
struct Pose { const void* header=nullptr; const void* array=nullptr; unsigned count=0; float y=0; bool valid=false; };
struct Record { ULONGLONG tick=0; DWORD thread=0; unsigned phase=0; const void* body=nullptr; const void* view=nullptr; Pose poses[2]; };
std::atomic<ULONGLONG> deadline{0};
std::atomic<unsigned> calls[4]{};
std::mutex mutex;
Record records[512]{};
unsigned count=0,total=0;
ULONGLONG sampled[4]{},start=0,last_poll=0;
bool on=false;
std::wstring dir;
const char* names[]={"camera_pre","camera_post","render_pre","present"};
const unsigned char* pointer(const unsigned char* base,unsigned offset) {
 const unsigned char* value=nullptr;
 if (base) game::mem_read(base+offset,&value,sizeof(value));
 return value;
}
Pose pose(const unsigned char* comp,unsigned offset,unsigned expected,unsigned wrist) {
 Pose result;const auto header=pointer(comp,offset);result.header=header;
 if (!header || !game::mem_read(header+8,&result.count,sizeof(result.count)) || result.count!=expected) return result;
 const auto array=pointer(header,0);result.array=array;
 if (!array || !game::mem_read(array+wrist*64+0x34,&result.y,sizeof(result.y)) || !std::isfinite(result.y)) return result;
 unsigned after=0;
 result.valid=pointer(comp,offset)==header && pointer(header,0)==array &&
  game::mem_read(header+8,&after,sizeof(after)) && after==expected;
 return result;
}
void log(const char* text) {
 HANDLE file=CreateFileW((dir+L"\\POSE_PHASE_PROBE.log").c_str(),FILE_APPEND_DATA,FILE_SHARE_READ,nullptr,OPEN_ALWAYS,0,nullptr);
 if(file==INVALID_HANDLE_VALUE)return;
 DWORD n;WriteFile(file,text,static_cast<DWORD>(std::strlen(text)),&n,nullptr);WriteFile(file,"\r\n",2,&n,nullptr);CloseHandle(file);
}
}
namespace pose_phase_probe {
void observe(Phase phase,const void* view) {
 const auto until=deadline.load(std::memory_order_relaxed);if(!until)return;
 const auto now=GetTickCount64();if(now>=until)return;
 const unsigned index=static_cast<unsigned>(phase);if(index>=4)return;
 calls[index].fetch_add(1,std::memory_order_relaxed);
 std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
 if(!lock.owns_lock() || total>=512 || now-sampled[index]<200)return;
 sampled[index]=now;
 const auto p=game::pointers();if(!p.body)return;
 const auto comp=pointer(p.body,0x1A8);if(!comp)return;
 Record record;record.tick=now;record.thread=GetCurrentThreadId();record.phase=index;record.body=p.body;record.view=view;
 record.poses[0]=pose(comp,0x4F8,192,77);record.poses[1]=pose(comp,0x500,90,56);
 if(game::pointers().body!=p.body || pointer(p.body,0x1A8)!=comp)return;
 records[count++]=record;++total;
}
void on_present() {
 if(dir.empty()) {
  HMODULE self=nullptr;wchar_t path[MAX_PATH]{};
  GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&on_present),&self);
  if(!GetModuleFileNameW(self,path,MAX_PATH))return;
  dir=path;dir=dir.substr(0,dir.find_last_of(L"\\/"));
 }
 observe(Phase::Present);
 const auto now=GetTickCount64();if(now-last_poll<1000)return;last_poll=now;
 char control[16]{};
 HANDLE file=CreateFileW((dir+L"\\AvatarVR_posetiming.txt").c_str(),GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,0,nullptr);
 if(file!=INVALID_HANDLE_VALUE){DWORD n;ReadFile(file,control,sizeof(control)-1,&n,nullptr);CloseHandle(file);}
 const bool requested=std::strncmp(control,"on",2)==0;
 if(requested && !on && game::read().valid) {
  std::lock_guard<std::mutex> lock(mutex);count=total=0;start=now;
  for(unsigned i=0;i<4;++i){sampled[i]=0;calls[i].store(0);}
  deadline.store(now+20000);
  log("pose phase build " __DATE__ " " __TIME__ "; 20s read-only window; 200ms per-phase sampling; max512; render_pre=tracked view0; Y is cell-local");
  on=true;
 }
 if(!requested){deadline.store(0);on=false;}
 Record pending[512];unsigned n=0,captured=0;
 {std::lock_guard<std::mutex> lock(mutex);n=count;std::memcpy(pending,records,n*sizeof(Record));count=0;captured=total;}
 char line[512];
 for(unsigned i=0;i<n;++i){const auto& r=pending[i];
  std::snprintf(line,sizeof(line),"age_ms=%llu phase=%s tid=%lu body=%p view=%p pose192=%p array192=%p valid192=%d wrist77_y=%.6f pose90=%p array90=%p valid90=%d wrist56_y=%.6f",
   r.tick-start,names[r.phase],r.thread,r.body,r.view,r.poses[0].header,r.poses[0].array,r.poses[0].valid,r.poses[0].y,
   r.poses[1].header,r.poses[1].array,r.poses[1].valid,r.poses[1].y);log(line);
 }
 const auto until=deadline.load();
 if(until && now>=until){deadline.store(0);
  std::snprintf(line,sizeof(line),"window complete samples=%u calls camera_pre=%u camera_post=%u render_pre=%u present=%u",captured,calls[0].load(),calls[1].load(),calls[2].load(),calls[3].load());log(line);
 }
}
}
