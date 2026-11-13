#pragma once

namespace dx12 {

void install();
// Eye mode: true when the swapchain buffers were grown past the window (see grow_buffers); w x h = the window's
// client size, which is the top-left region the monitor is shown.
bool client_size(unsigned* w, unsigned* h);
// Eye mode: the game's own GetClientRect calls get the eye size (its UI lays out for the eye, as with an eye-shaped
// window), so the UI needs no canvas correction.
bool ui_window_is_eye();
// GPU profiler marks: name what a game command list carries, so the submission that executes it can be labelled.
// kind 'D' = DLSS evaluate (value = Streamline viewport id), 'U' = UI pass (value = eye, 1 main / 2 second).
// Cheap no-op while the profiler is off.
void mark_list(void* list, char kind, int value);
// The game's swapchain output window (nullptr before the swapchain exists).
void* game_window();

}
