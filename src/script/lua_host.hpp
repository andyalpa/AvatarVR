#pragma once

#include <cstdint>
#include <string>
#include <vector>

// In-game Lua scripting for live gameplay experiments (docs/SCRIPTING.md). One Lua 5.4 state runs
// scripts\avr\autorun\*.lua; F7 or the menu's "Reload scripts" rebuilds the state while hooks and the game stay up.
// Events: "frame" (Present thread), "camera_pre" / "camera_post" (game thread, around the camera manager update
// 0x144271330), "unload". Game writes from scripts are no-ops until the menu's "Script writes" is on.
namespace lua_host {

// Present thread, once per frame, before vr_menu::on_present(): first call installs the hook and loads the scripts;
// then runs a requested reload and the "frame" event.
void on_present();
void request_reload();

bool* writes_flag();  // the menu's "Script writes" toggle (off at every launch)
const char* status();  // one line: scripts loaded, callbacks, last error

// Script tunables shown as rows in the menu's Scripts tab. Pointers stay valid until rows_version() changes.
struct Tunable {
  char name[48] = {};
  float value = 0.f, lo = 0.f, hi = 1.f, step = 0.1f;
  bool toggle = false;
  bool on = false;  // toggle value
  bool used = false;
  bool ready = false;  // set by the first avr.tunable / avr.toggle call for this name
};
int tunable_count();
Tunable* tunable(int index);
unsigned rows_version();

// Debug drawing recorded by scripts, already projected to the game camera: coordinates are fractions of the game
// image (0..1, origin top-left). The menu draws them on the monitor.
struct DrawPrim {
  enum Kind : std::uint8_t { Line, Point, Text } kind = Line;
  float x0 = 0.f, y0 = 0.f, x1 = 0.f, y1 = 0.f;
  float size = 2.f;
  std::uint32_t rgba = 0xFFFFFFFF;
  std::string text;
};
bool copy_draws(std::vector<DrawPrim>* out);  // false when there is nothing to draw

}  // namespace lua_host
