#pragma once
#include "scanner_inputs.hpp"
#include <mutex>
#include <unordered_map>
// Read-only IA bindings, reset by the existing command-list Reset observer.
// No retained resource pointers or command-list calls. Only valid during probe.
namespace scanner_geometry {
struct State {
  D3D12_VERTEX_BUFFER_VIEW vb[32]{};
  UINT valid=0;
  D3D12_INDEX_BUFFER_VIEW ib{};
  bool index=false,reset_seen=false;
};
inline std::mutex mutex;
inline std::unordered_map<void*,State> lists;
inline State* find(void* list) {
  auto it=lists.find(list);
  if(it!=lists.end()) return &it->second;
  if(lists.size()>=4096) return nullptr;
  return &lists[list];
}
inline void reset(void* list) {
  if(!scanner_inputs::enabled.load(std::memory_order_relaxed)) return;
  std::lock_guard<std::mutex> lock(mutex);
  if(auto* s=find(list)) {*s={};s->reset_seen=true;}
}
inline void vertices(void* list,UINT first,UINT n,const D3D12_VERTEX_BUFFER_VIEW* v) {
  if(!scanner_inputs::enabled.load(std::memory_order_relaxed)||first>=32) return;
  std::lock_guard<std::mutex> lock(mutex);
  if(auto* s=find(list)) for(UINT k=0;k<n&&k<32-first;++k) {
    s->vb[first+k]=v?v[k]:D3D12_VERTEX_BUFFER_VIEW{};
    s->valid|=UINT(1)<<(first+k);
  }
}
inline void indices(void* list,const D3D12_INDEX_BUFFER_VIEW* v) {
  if(!scanner_inputs::enabled.load(std::memory_order_relaxed)) return;
  std::lock_guard<std::mutex> lock(mutex);
  if(auto* s=find(list)) {s->ib=v?*v:D3D12_INDEX_BUFFER_VIEW{};s->index=v!=nullptr;}
}
inline State snapshot(void* list) {
  std::lock_guard<std::mutex> lock(mutex);
  auto it=lists.find(list);return it==lists.end()?State{}:it->second;
}
}
