#pragma once

#include <windows.h>

#include <d3d12.h>

#include <cstddef>
#include <cstdint>

namespace camera {

void install();
void note_backbuffer(UINT width, UINT height);
// Hooks the direct command-list methods: the UI probe always, the projection / view / viewport research observers
// only when `research` is set.
void watch_list(void* list, bool research);
// UI probe counters for eye 1 (main view) or 2 (second view) since the last call, as one text line; resets them.
void ui_probe_report(int eye, char* out, size_t size);
// UI placement: called when a UI pass returns, on its thread; restores the viewport / scissor the placement changed.
void ui_pass_end();
void ui_pass_begin();
// Hooks the device's PSO creation (graphics / stream desc, pipeline library loads) to record each PSO's blend state.
void watch_device(void* device);
// Scan mask shift for the right eye (native control "noscanshift" turns it off).
void set_scan_shift(bool on);
// Main menu panel: the UI pass draws with alpha-accumulating PSO clones into the panel layer ("nopanelcover" = off).
void set_panel_cover(bool on);
// Scan mask shift around the second view's scan composite (native control "noscanmask" turns it off).
void set_scan_mask(bool on);
// Experimental scan overlay redirect, explicitly opt-in with native control "scanoverlay".
void set_scan_overlay(bool on);
void set_fx_stretch(bool on);  // "fxstretch": full-screen effect draws of the UI pass cover the whole eye
void hudfx_next();             // damage vignette test: next candidate shader stretched to the whole eye, then off
const char* hudfx_status();    // stable label buffer
void hudfx_sweep();            // damage vignette test: start or stop stretching each HUD-pass draw in turn
void hudfx_draw_next();        // same test, one draw forward (from off: the first draw)
void hudfx_draw_previous();    // same test, one draw back; before the first draw it is off
// Rect probe: the sub-target viewport / scissor rects set outside UI passes, one line each through `line`.
void rect_probe_report(void (*line)(const char*));
// Main menu layer texture (AddRef'd; caller releases), or nullptr when no UI was drawn into it in the last 250 ms.
ID3D12Resource* ui_layer_acquire();
// Wrist HUD texture (main view HUD pass copied with the logical canvas fitted, 1920x1080 RGBA8, simultaneous-access);
// AddRef'd, or null when nothing was copied in the last 250 ms.
ID3D12Resource* hud_acquire();
void hud_report(char* out, size_t size);
// UI passes whose target got the placed viewport (eye 1 = main view, 2 = second view), since start.
std::uint64_t ui_placed(int eye);
void sample();

}
