#pragma once
#include <cstdint>

// Bounded native HUD ownership observation; original engine calls and results are untouched. The opt-in widget test alone
// changes call arguments while it is on.
namespace ui_ownership {
using Log = void (*)(const char*);
void install(unsigned char* base, Log log); // prepare only; native hooks wait for the first explicit request
void allow(bool enabled);  // follows nouipiece; disabling cancels a pending/active observation
void request();            // Present/menu thread: request 30s after the next selected gameplay query
void stop();
bool arm_layer_active();           // draw hooks: this thread is drawing the per-eye arm HUD layers
bool fx_layer_active();            // draw hooks: this thread is drawing the full-view effects layer
float fx_plane_size();             // its plane, times the lens span (tuner Size)
void fx_effect_drawing(bool in_layer); // draw hooks: the scanner or boost is drawing (in the effects layer or not)
void layer_draw(int eye);          // draw hooks: counts this thread's draw under the running UI layer (widget test)
void widget_test_next();           // F10: next native widget test, off after the last. Not saved.
const char* widget_test_status();  // stable label buffer; refreshed on Present
bool capture_active();     // recording now; the HUD draw census in camera.cpp runs only during this
const char* status();      // stable label buffer; refreshed only on Present/menu thread
void report();             // Present only: epoch, file I/O, timeout, bounded queue drain
void piece(void* node, void* params, bool uint_output, std::uintptr_t caller);
}
