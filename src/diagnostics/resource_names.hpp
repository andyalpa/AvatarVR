#pragma once

#include <d3d12.h>

// Read-only discovery of the engine's debug names on D3D12 resources (Snowdrop names its render targets
// "Gfx_RenderTargets/...", including the temporal histories myLastFrameColor, mySSAOHistory, ...).
// Hooks ID3D12Object::SetName and ::SetPrivateData(WKPDID_D3DDebugObjectName[W]) on the shared resource
// vtable, records name -> resource with its description, and logs each new texture name once to
// scripts\RESOURCE_NAMES.log. Nothing about the resources is changed.
namespace resource_names {

void install();                         // remember the log path
void arm(ID3D12Resource* any_resource);  // hook the resource vtable once (called from the creation hooks)
void poll();                            // flush a summary line (Present thread, once a second)
void* resource_vtable();                // the shared ID3D12Resource vtable once armed (nullptr before)

}  // namespace resource_names
