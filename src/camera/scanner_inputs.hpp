#pragma once
#include <d3d12.h>
#include <atomic>
#include <array>
// Read-only, opt-in descriptor provenance. Never inspect opaque descriptor bytes.
namespace scanner_inputs {
inline std::atomic<bool> enabled{false};
// bytes>0: CBV GPU address; bytes==0: SRV resource identity (never dereferenced).
struct View {
  UINT64 address=0; UINT bytes=0; UINT64 tag=0; UINT64 raw_va=0,raw_bytes=0;
  std::array<UINT,16> raw64{};
  bool raw_saved=false;
  UINT heap_type=0,map_result=0;
  UINT64 raw_present=0;
};
struct Slot {
  std::atomic<SIZE_T> key{0};
  std::atomic<unsigned> seq{0};
  std::atomic<UINT64> address{0};
  std::atomic<UINT> bytes{0};
  std::atomic<UINT64> tag{0};
  std::atomic<UINT64> raw_va{0},raw_bytes{0};
  std::atomic<UINT> raw64[16]{},raw_saved{0},heap_type{0},map_result{0};
  std::atomic<UINT64> raw_present{0};
};
inline Slot slots[1<<16];
inline unsigned index(SIZE_T key) {return static_cast<unsigned>(((key>>3)*0x9e3779b97f4a7c15ull)>>48);}
inline void note(SIZE_T key,View value) {
  if(!enabled.load(std::memory_order_relaxed)||!key) return;
  auto& s=slots[index(key)];
  if(!value.address&&s.key.load(std::memory_order_acquire)!=key) return;
  unsigned seq=s.seq.load(std::memory_order_acquire);
  if(seq&1||!s.seq.compare_exchange_strong(seq,seq+1)) return; // drop contended observation; never stall workers
  s.address.store(value.address,std::memory_order_relaxed);s.bytes.store(value.bytes,std::memory_order_relaxed);
  s.tag.store(value.tag,std::memory_order_relaxed);
  s.raw_va.store(value.raw_va,std::memory_order_relaxed);s.raw_bytes.store(value.raw_bytes,std::memory_order_relaxed);
  for(int i=0;i<16;++i) s.raw64[i].store(value.raw64[i],std::memory_order_relaxed);
  s.raw_saved.store(value.raw_saved,std::memory_order_relaxed);
  s.heap_type.store(value.heap_type,std::memory_order_relaxed);s.map_result.store(value.map_result,std::memory_order_relaxed);
  s.raw_present.store(value.raw_present,std::memory_order_relaxed);
  s.key.store(key,std::memory_order_relaxed);s.seq.store(seq+2,std::memory_order_release);
}
inline View get(SIZE_T key) {
  if(!key) return {};
  auto& s=slots[index(key)];
  const unsigned seq=s.seq.load(std::memory_order_acquire);
  if((seq&1)||s.key.load(std::memory_order_relaxed)!=key) return {};
  View v{s.address.load(std::memory_order_relaxed),s.bytes.load(std::memory_order_relaxed),s.tag.load(std::memory_order_relaxed),
    s.raw_va.load(std::memory_order_relaxed),s.raw_bytes.load(std::memory_order_relaxed)};
  for(int i=0;i<16;++i) v.raw64[i]=s.raw64[i].load(std::memory_order_relaxed);
  v.raw_saved=s.raw_saved.load(std::memory_order_relaxed)!=0;
  v.heap_type=s.heap_type.load(std::memory_order_relaxed);v.map_result=s.map_result.load(std::memory_order_relaxed);
  v.raw_present=s.raw_present.load(std::memory_order_relaxed);
  return seq==s.seq.load(std::memory_order_acquire)?v:View{};
}
inline void copy(SIZE_T dst,SIZE_T src) {if(enabled.load(std::memory_order_relaxed)) note(dst,get(src));}
}
