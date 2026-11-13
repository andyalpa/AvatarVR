#pragma once

#include <d3d12.h>

namespace upload {

void install();
void watch_device(ID3D12Device* device);
// Full rate (every CreateConstantBufferView is resolved and recorded) or light (1 in 32 per thread: enough for the
// measured game frustum, without a process-wide lock on every CBV the engine creates). Default light.
void set_full_rate(bool full);
void sample();
// Called once per IDXGISwapChain::Present (before the original). Lock-free counter only.
void on_present();
// Bounded read-only copy for scanner draw-time diagnostics; unresolved/default-heap addresses fail closed.
bool copy_constants(UINT64 address, UINT bytes, void* out);
// Probe-only mapped upload snapshot. Never maps a resource; max 16 KiB.
bool copy_mapped(UINT64 address, UINT bytes, void* out);
// Resource is live in its CreateSRV callback. Small CPU-visible buffer read only;
// balanced nested Map/Unmap, empty written range, no GPU commands or resource retention.
bool copy_live_raw64(ID3D12Resource* resource,UINT64 offset,void* out,UINT* heap_type,UINT* map_result);
UINT64 present_sequence();

}
