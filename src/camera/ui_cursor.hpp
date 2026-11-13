#pragma once
namespace ui_cursor {
using Log = void (*)(const char*);
// Bounded read-only main-menu trace; original engine input/results pass through.
void install(unsigned char* base, Log log);
void report();
void set_enabled(bool enabled);
}
