#pragma once

#include <windows.h>

// Per-eye DLSS / DLSS Ray Reconstruction history through NVIDIA Streamline viewports.
// AFoP upscales and denoises with DLSS-RR (resource names InternalHistoryA/B, OutputHistRes, HiddenHistory,
// PrevMvec1/2, DFN* all live in nvngx_dlssd.dll) through Streamline 2.8 (sl.interposer.dll). Streamline keeps a
// separate DLSS instance and history per ViewportHandle, so alternate-eye frames can use two viewports:
//   - every per-viewport setter (options, tags, per-frame tags, constants, allocate/free) is mirrored onto a
//     twin viewport (id + kTwinOffset), so the twin is always configured exactly like the game's viewport
//   - slEvaluateFeature for DLSS/DLSS-RR is redirected to the twin viewport on right-eye frames while
//     scripts\AvatarVR_dlss.txt says "twin" and eye_write is alternating
// Exports are hooked by name (public Streamline API). Default: pass-through apart from the mirroring.
namespace dlss_twin {

void install();              // paths only
void hook(HMODULE interposer);  // hook sl.interposer.dll exports once it is loaded
void poll();                 // control file + counters (Present thread, once a second)
// One bounded, read-only three-frame stereo motion diagnostic; false if another capture is pending.
bool request_mv_burst();
// "jitlog" in AvatarVR_dlss.txt: a burst of per-call jitter lines (DLSS_TWIN.log, QPC-stamped). native_stereo logs the
// rendered camera blocks' projection centre during the same burst, so the two can be matched frame by frame.
bool jit_logging();

}  // namespace dlss_twin
