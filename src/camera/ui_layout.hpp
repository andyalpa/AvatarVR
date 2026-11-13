#pragma once

namespace ui_layout {
using Log = void (*)(const char*);
void install(unsigned char* base, Log log);
void set_enabled(bool enabled);
void report();
}
