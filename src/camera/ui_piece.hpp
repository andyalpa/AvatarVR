#pragma once
namespace ui_piece {
using Log = void (*)(const char*);
// Read-only HUD piece discovery (Gfx_MovablePieceID settings, crosshair viewport node).
// Never writes engine memory; original node results pass through unchanged.
void install(unsigned char* base, Log log);
void report();
void set_enabled(bool enabled);
// The gameplay HUD graph asked for the health bar piece within `ms` (it runs while the HUD is up). Needs the node hooks.
bool hud_live(unsigned ms);
}
