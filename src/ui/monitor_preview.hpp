#pragma once

#include <windows.h>

// A scaled-down live copy of the game window on the monitor (DWM thumbnail, the same mechanism as the taskbar's
// window previews). The VR render shapes make the game window taller than the screen; this preview fits the whole
// picture on the monitor. The preview is click-through and never takes focus, so input still goes to the game.
namespace monitor_preview {

// Present thread, every frame. `want` = the user setting; the preview shows only while the game window does not fit
// the monitor's work area.
void update(HWND game, bool want);
// Counters for the menu log: thumbnail registrations, failures, current visibility.
void stats(unsigned long long* registered, unsigned long long* failed, bool* visible);

}  // namespace monitor_preview
