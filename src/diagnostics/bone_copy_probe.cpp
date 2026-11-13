#include "bone_copy_probe.hpp"
#include "../game/player.hpp"
#include <windows.h>
#include <MinHook.h>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace {
constexpr std::uintptr_t kBonePackRva=0x637220;
constexpr unsigned char kBonePackSig[]={0x48,0x89,0x5C,0x24,0x10,0x48,0x89,0x6C,0x24,0x18,0x56,0x57,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x50,0x48};
constexpr std::uintptr_t kBoneAllocateRva=0x42C9F0;
constexpr unsigned char kBoneAllocateSig[]={0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18,0x48,0x89,0x7C,0x24,0x20,0x41,0x56,0x48,0x83,0xEC,0x30,0x44,0x8B,0xF2,0x41,0x8B,0xF9,0x41,0x03,0xD1,0x49,0x8B,0xF0,0x48,0x8B,0xE9,0xE8,0x82};
using Pack=void(__fastcall*)(void*,void*);
using Allocate=std::uint32_t(__fastcall*)(void*,std::uint32_t,const void*,std::uint32_t,const void*);
Pack original_pack=nullptr;
Allocate original_allocate=nullptr;
thread_local void* packing_owner=nullptr;
std::atomic<ULONGLONG> deadline{0};
std::atomic<unsigned> pack_calls{0}, allocations{0};
struct Sample {
  const void* binding;
  void* packing; const void* first; const void* second; const void* pose; const void* cpu;
  std::uint32_t nfirst,nsecond; int bones;
  float first_head[4],second_head[4];
};
Sample samples[128]{};
unsigned count=0,total=0;
const void* seen_owners[128]{};
unsigned owner_count=0;
std::mutex sample_mutex;
std::wstring dir;
bool tried=false,on=false,installed=false;
ULONGLONG last_poll=0;

void log(const char* line) {
  HANDLE file=CreateFileW((dir+L"\\BONE_COPY_PROBE.log").c_str(),FILE_APPEND_DATA,FILE_SHARE_READ,nullptr,OPEN_ALWAYS,0,nullptr);
  if (file==INVALID_HANDLE_VALUE) return;
  DWORD n;
  WriteFile(file,line,static_cast<DWORD>(std::strlen(line)),&n,nullptr);
  WriteFile(file,"\r\n",2,&n,nullptr); CloseHandle(file);
}

void __fastcall pack_detour(void* owner,void* target) {
  void* previous=packing_owner;
  packing_owner=owner;
  if (GetTickCount64()<deadline.load()) {
    pack_calls.fetch_add(1);
    std::unique_lock<std::mutex> lock(sample_mutex,std::try_to_lock);
    if (lock.owns_lock() && owner_count<128 && total<128) {
      bool seen=false;
      for (unsigned i=0;i<owner_count;++i) seen|=seen_owners[i]==owner;
      if (!seen) {
        seen_owners[owner_count++]=owner;
        const auto player=game::pointers();
        auto bytes=static_cast<const unsigned char*>(owner);
        const unsigned char* groups=nullptr;
        std::uint32_t ngroups=0;
        // +48 is a candidate vector length, not a confirmed engine field.
        // Bounds and guarded reads make this a read-only layout discriminator.
        game::mem_read(bytes+0x40,&groups,sizeof(groups));
        game::mem_read(bytes+0x48,&ngroups,sizeof(ngroups));
        Sample root{}; root.packing=owner; root.second=groups; root.nsecond=ngroups;
        root.pose=player.pose; root.bones=game::bone_count(player);
        if (player.pose) game::mem_read(player.pose,&root.cpu,sizeof(root.cpu));
        samples[count++]=root; ++total;
        if (groups && ngroups>0 && ngroups<=1024) {
          for (unsigned g=0;g<ngroups && g<64 && total<128;++g) {
            const unsigned char* list=nullptr; std::uint32_t length=0;
            if (!game::mem_read(groups+g*16,&list,sizeof(list)) ||
                !game::mem_read(groups+g*16+8,&length,sizeof(length)) || !list || length>1024) continue;
            for (unsigned j=0;j<length && j<32 && total<128;++j) {
              const unsigned char* binding=nullptr; const unsigned char* asset=nullptr;
              if (!game::mem_read(list+j*8,&binding,sizeof(binding)) || !binding) continue;
              Sample s=root; s.binding=binding; s.first=nullptr; s.second=nullptr; s.nfirst=0; s.nsecond=0;
              if (!game::mem_read(binding,&s.second,sizeof(s.second)) ||
                  !game::mem_read(binding+8,&s.nsecond,sizeof(s.nsecond)) ||
                  !game::mem_read(binding+0x18,&asset,sizeof(asset)) || !asset ||
                  !game::mem_read(asset+0xD8,&s.first,sizeof(s.first))) continue;
              if (s.first) game::mem_read(static_cast<const unsigned char*>(s.first)-0x10,&s.nfirst,sizeof(s.nfirst));
              if (s.nfirst>65536 || s.nsecond>65536) continue;
              if (s.first && s.nfirst) game::mem_read(s.first,s.first_head,sizeof(s.first_head));
              if (s.second && s.nsecond) game::mem_read(s.second,s.second_head,sizeof(s.second_head));
              samples[count++]=s; ++total;
            }
          }
        }
      }
    }
  }
  original_pack(owner,target);
  packing_owner=previous;
}

std::uint32_t __fastcall allocate_detour(void* manager,std::uint32_t nfirst,const void* first,
                                       std::uint32_t nsecond,const void* second) {
  if (packing_owner && GetTickCount64()<deadline.load()) {
    allocations.fetch_add(1);
    std::unique_lock<std::mutex> lock(sample_mutex,std::try_to_lock);
    if (lock.owns_lock() && total<128 && count<128) {
      const auto player=game::pointers();
      const int bones=game::bone_count(player);
      if (bones>0) {
        Sample sample{};
        sample.packing=packing_owner; sample.first=first; sample.second=second;
        sample.nfirst=nfirst; sample.nsecond=nsecond; sample.pose=player.pose; sample.bones=bones;
        game::mem_read(player.pose,&sample.cpu,sizeof(sample.cpu));
        if (first && nfirst>0) game::mem_read(first,sample.first_head,sizeof(sample.first_head));
        if (second && nsecond>0) game::mem_read(second,sample.second_head,sizeof(sample.second_head));
        samples[count++]=sample; ++total;
      }
    }
  }
  return original_allocate(manager,nfirst,first,nsecond,second);
}

bool guard(std::uintptr_t rva,const unsigned char* signature,std::size_t n) {
  unsigned char bytes[48]{};
  return game::mem_read(game::module_base()+rva,bytes,n) && std::memcmp(bytes,signature,n)==0;
}
bool install() {
  auto base=game::module_base();
  if (!guard(kBonePackRva,kBonePackSig,sizeof(kBonePackSig)) ||
      !guard(kBoneAllocateRva,kBoneAllocateSig,sizeof(kBoneAllocateSig))) { log("guard mismatch; no hooks"); return false; }
  void* pack=base+kBonePackRva; void* allocate=base+kBoneAllocateRva;
  if (MH_CreateHook(pack,reinterpret_cast<void*>(&pack_detour),reinterpret_cast<void**>(&original_pack))!=MH_OK) return false;
  if (MH_CreateHook(allocate,reinterpret_cast<void*>(&allocate_detour),reinterpret_cast<void**>(&original_allocate))!=MH_OK) {
    MH_RemoveHook(pack); return false;
  }
  if (MH_EnableHook(allocate)!=MH_OK) { MH_RemoveHook(allocate); MH_RemoveHook(pack); return false; }
  if (MH_EnableHook(pack)!=MH_OK) {
    MH_DisableHook(allocate); MH_RemoveHook(allocate); MH_RemoveHook(pack); return false;
  }
  log("bone packing/allocation observation hooks installed; original arguments/results unchanged"); return true;
}
}

namespace bone_copy_probe {
void on_present() {
  if (dir.empty()) {
    HMODULE self=nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
      reinterpret_cast<LPCWSTR>(&on_present),&self);
    wchar_t path[MAX_PATH]{};
    if (!GetModuleFileNameW(self,path,MAX_PATH)) return;
    dir=path; dir=dir.substr(0,dir.find_last_of(L"\\/"));
  }
  const auto now=GetTickCount64();
  if (now-last_poll<1000) return;
  last_poll=now;
  char control[16]{};
  HANDLE file=CreateFileW((dir+L"\\AvatarVR_bonecopy.txt").c_str(),GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,0,nullptr);
  if (file!=INVALID_HANDLE_VALUE) { DWORD n; ReadFile(file,control,sizeof(control)-1,&n,nullptr); CloseHandle(file); }
  const bool wanted=std::strncmp(control,"on",2)==0 && game::read().valid;
  if (wanted && !on) {
    if (!tried) { tried=true; installed=install(); }
    if (installed) {
      std::lock_guard<std::mutex> lock(sample_mutex); total=0; count=0; owner_count=0;
      pack_calls.store(0); allocations.store(0); deadline.store(now+30000);
      log("30 second persistent-binding observation window started (128 samples maximum; root records use candidate +48 group count)");
    }
  }
  on=wanted;
  if (!on) deadline.store(0);
  Sample pending[128]; unsigned n=0;
  { std::lock_guard<std::mutex> lock(sample_mutex); n=count; std::memcpy(pending,samples,n*sizeof(Sample)); count=0; }
  char line[600];
  for (unsigned i=0;i<n;++i) {
    const auto& s=pending[i];
    std::snprintf(line,sizeof(line),"pack=%p binding=%p first=%p records=%u second=%p records=%u pose=%p cpu=%p bones=%d same_first=%d same_second=%d first4=(%.3g %.3g %.3g %.3g) second4=(%.3g %.3g %.3g %.3g)",
      s.packing,s.binding,s.first,s.nfirst,s.second,s.nsecond,s.pose,s.cpu,s.bones,s.first==s.cpu,s.second==s.cpu,
      s.first_head[0],s.first_head[1],s.first_head[2],s.first_head[3],s.second_head[0],s.second_head[1],s.second_head[2],s.second_head[3]);
    log(line);
  }
  const auto until=deadline.load();
  if (until && now>=until) {
    deadline.store(0);
    unsigned captured=0;
    { std::lock_guard<std::mutex> lock(sample_mutex); captured=total; }
    std::snprintf(line,sizeof(line),"window complete: packing_calls=%u allocation_calls=%u samples=%u",pack_calls.load(),allocations.load(),captured);
    log(line);
  }
}
}
