#pragma once

#include <windows.h>

// Per-eye XeSS history (the AFR "TAA/history problem", vrframework guide 10).
// The game's upscaler is Intel XeSS (libxess.dll 2.0.1; its internal InternalHistoryA/B, OutputHistRes,
// HiddenHistory and PrevMvec textures were seen by resource_names). With alternate-eye rendering one XeSS
// context blends each eye with the other eye's history. This module gives the second eye its own context:
//   - xessD3D12CreateContext: also create a twin context on the same device
//   - xessD3D12Init: also init the twin with the same parameters (but XeSS-owned temp heaps, so no
//     history memory is shared), and the per-context setters are mirrored onto the twin
//   - xessD3D12Execute: while eye_write alternates eyes and scripts\AvatarVR_xess.txt says "twin",
//     frames with odd present parity go to the twin (the first twin frame resets its history)
//   - xessDestroyContext: destroys the twin too
// Exports are hooked by name (the public XeSS API), no addresses. Default: pass-through.
namespace xess_twin {

void install();                // paths only
void hook(HMODULE libxess);    // hook the XeSS exports once the DLL is loaded
void poll();                   // control file + counters (Present thread, once a second)

}  // namespace xess_twin
