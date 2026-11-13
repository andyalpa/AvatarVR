#include "ui_layout.hpp"
#include "ui_layout_math.hpp"
#include "native_stereo.hpp"
#include "../dx12/dx12.hpp"
#include <windows.h>
#include <MinHook.h>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {
using Node = void (*)(void*, void*);
using Matrix = void* (*)(void*, float*, void*);
Node g_size = nullptr, g_aspect = nullptr, g_safe = nullptr, g_viewport = nullptr, g_game_size = nullptr;
Matrix g_to_pixel = nullptr, g_to_viewport = nullptr;
using Slot = unsigned (*)();
Slot g_slot = nullptr;
unsigned char* g_base = nullptr;
ui_layout::Log g_log = nullptr;
std::atomic<bool> g_enabled{false}, g_armed{false};
std::atomic<unsigned long long> g_hits[7], g_skips{0};
std::atomic<float> g_old_w{0}, g_old_h{0};
std::atomic<float> g_old_pixel_w{0}, g_old_pixel_h{0}, g_old_inverse_w{0}, g_old_inverse_h{0};
std::atomic<float> g_context_w{0}, g_context_h{0};

bool copy_read(const void* from, void* to, size_t bytes) {
  __try { std::memcpy(to, from, bytes); return true; }
  __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}
bool copy_write(void* to, const void* from, size_t bytes) {
  __try { std::memcpy(to, from, bytes); return true; }
  __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}

bool display_context(void* context, float w, float h, float cw, float ch) {
  std::int32_t rect[4];
  if (context == nullptr || !copy_read(static_cast<unsigned char*>(context)+0x304,rect,sizeof(rect))) return false;
  const float sw=static_cast<float>(rect[2])-static_cast<float>(rect[0]);
  const float sh=static_cast<float>(rect[3])-static_cast<float>(rect[1]);
  g_context_w.store(sw); g_context_h.store(sh);
  return (ui_layout::close(sw,cw) && ui_layout::close(sh,ch)) ||
         (ui_layout::close(sw,w) && ui_layout::close(sh,h));
}
bool current_display_context(float w, float h, float cw, float ch) {
  if (g_slot==nullptr || g_base==nullptr) return false;
  const unsigned slot=g_slot(); // same TLS slot accessor called by the original UI nodes
  if (slot>=256) return false;
  void* context=nullptr;
  return copy_read(g_base+0x96AB220+sizeof(void*)*slot,&context,sizeof(context)) &&
      display_context(context,w,h,cw,ch);
}

bool canvas(float* w, float* h, float* cw = nullptr, float* ch = nullptr) {
  if (!g_enabled.load(std::memory_order_relaxed) || !g_armed.load(std::memory_order_relaxed)) return false;
  std::uint32_t ew = 0, eh = 0;
  if (!native_stereo::stereo_running() || !native_stereo::eye_size(&ew, &eh)) return false; // excludes paused/flat cinema
  *w = static_cast<float>(ew); *h = static_cast<float>(eh);
  if (cw != nullptr && ch != nullptr) {
    unsigned sw = 0, sh = 0;
    if (!dx12::client_size(&sw, &sh)) return false;
    *cw = static_cast<float>(sw); *ch = static_cast<float>(sh);
  }
  return ui_layout::dimension(*w) && ui_layout::dimension(*h);
}

// Blue outputs contain inline payload at +0 and count/reference flags at +1C.
// STATIC allocator0x8E6E0: reference storage's data pointer is written into +0
// before returning writable payload. Original nodes have already allocated it.
// Decode that reference instead of overwriting its pointer with float pixels.
// Do not change the engine's global window/display object.
float* output(void* params, size_t slot, unsigned* count) {
  float* p = nullptr;
  std::uint32_t flags = 0;
  if (!copy_read(static_cast<unsigned char*>(params) + slot, &p, sizeof(p)) || p == nullptr ||
      !copy_read(reinterpret_cast<unsigned char*>(p) + 0x1c, &flags, sizeof(flags)) ||
      (flags & 0xffff) == 0 || (flags & 0xffff) > 1024) return nullptr;
  *count = flags & 0xffff;
  if ((flags & 0x10000000) != 0) {
    float* data = nullptr;
    if (!copy_read(p,&data,sizeof(data)) || data==nullptr) return nullptr;
    return data;
  }
  if (*count != 1) return nullptr; // multiple values require reference storage
  return p;
}

void Size_hook(void* self, void* params) {
  g_size(self, params);
  float w, h, cw, ch;
  if (!canvas(&w, &h, &cw, &ch) || !current_display_context(w,h,cw,ch)) return;
  unsigned n = 0;
  float* p = output(params, 0x20, &n);
  for (unsigned i = 0; p != nullptr && i < n; ++i) {
    float old[2];
    if (!copy_read(p + 2*i, old, sizeof(old)) || !ui_layout::dimension(old[0]) || !ui_layout::dimension(old[1])) continue;
    // Safe-area sizes can be smaller; retain their fractions of the canvas.
    // Already eye-sized contexts pass through, preventing double scaling.
    if (ui_layout::close(old[0], w) && ui_layout::close(old[1], h)) continue;
    if (old[0] > cw + 2.f || old[1] > ch + 2.f) { ++g_skips; continue; }
    const float next[2] = {old[0] * w/cw, old[1] * h/ch};
    if (copy_write(p + 2*i, next, sizeof(next))) {
      ++g_hits[0]; g_old_w.store(old[0]); g_old_h.store(old[1]);
    }
  }
}

void Aspect_hook(void* self, void* params) {
  g_aspect(self, params);
  float w, h, cw, ch;
  if (!canvas(&w, &h, &cw, &ch) || !current_display_context(w,h,cw,ch)) return;
  unsigned n = 0;
  float* p = output(params, 0x20, &n);
  for (unsigned i = 0; p != nullptr && i < n; ++i) {
    float old;
    if (!copy_read(p+i, &old, sizeof(old)) || !std::isfinite(old) || old < .25f || old > 4.f) continue;
    // Only change the physical-window aspect, not a custom graph's aspect.
    if (std::fabs(old - cw/ch) > .02f) { ++g_skips; continue; }
    const float next = w/h;
    if (copy_write(p+i, &next, sizeof(next))) ++g_hits[1];
  }
}

void Safe_hook(void* self, void* params) {
  g_safe(self, params);
  float w, h, cw, ch;
  if (!canvas(&w, &h, &cw, &ch) || !current_display_context(w,h,cw,ch)) return;
  float old[4]; float* p[4];
  unsigned n;
  for (unsigned i=0; i<4; ++i) {
    p[i] = output(params, 0x20+8*i, &n);
    if (p[i] == nullptr || n != 1 || !copy_read(p[i], old+i, sizeof(float)) || !std::isfinite(old[i])) return;
  }
  if (old[0] < 0 || old[1] < 0 || old[2] < old[0] || old[3] < old[1] || old[2] > cw+2 || old[3] > ch+2) { ++g_skips; return; }
  for (unsigned i=0; i<4; ++i) {
    const float next = old[i] * (i%2 == 0 ? w/cw : h/ch);
    copy_write(p[i], &next, sizeof(next));
  }
  ++g_hits[2];
}

void Viewport_hook(void* self, void* params) {
  g_viewport(self, params);
  float w, h, cw, ch;
  if (!canvas(&w, &h, &cw, &ch) || !current_display_context(w,h,cw,ch)) return;
  unsigned n;
  float* p = output(params, 0x28, &n);
  float old[2];
  if (p == nullptr || n != 1 || !copy_read(p, old, sizeof(old)) ||
      !ui_layout::dimension(old[0]) || !ui_layout::dimension(old[1])) return;
  const float next[2] = {w,h};
  if (copy_write(p, next, sizeof(next))) ++g_hits[3];
}

void GameSize_hook(void* self, void* params) {
  g_game_size(self, params);
  float w, h, cw, ch;
  if (!canvas(&w, &h, &cw, &ch) || !current_display_context(w,h,cw,ch)) return;
  unsigned n;
  float* x = output(params, 0x20, &n);
  if (x != nullptr && n != 1) x = nullptr;
  float* y = output(params, 0x28, &n);
  if (y != nullptr && n != 1) y = nullptr;
  float old;
  // Restrict this general Blue query to verified display-size values.
  if (x != nullptr && copy_read(x,&old,4) && ui_layout::close(old,cw)) { copy_write(x,&w,4); ++g_hits[4]; }
  if (y != nullptr && copy_read(y,&old,4) && ui_layout::close(old,ch)) { copy_write(y,&h,4); ++g_hits[4]; }
}

void* ToPixel_hook(void* self, float* out, void* context) {
  void* result = g_to_pixel(self,out,context);
  float w,h,cw,ch;
  float m[16];
  if (result == out && canvas(&w,&h,&cw,&ch) && display_context(context,w,h,cw,ch) && copy_read(out,m,sizeof(m))) {
    g_old_pixel_w.store(m[0]); g_old_pixel_h.store(m[5]);
    ui_layout::viewport_to_pixel(m,w,h);
    if (copy_write(out,m,sizeof(m))) ++g_hits[5];
  }
  return result;
}
void* ToViewport_hook(void* self, float* out, void* context) {
  void* result = g_to_viewport(self,out,context);
  float w,h,cw,ch;
  float m[16];
  if (result == out && canvas(&w,&h,&cw,&ch) && display_context(context,w,h,cw,ch) && copy_read(out,m,sizeof(m))) {
    g_old_inverse_w.store(m[0]); g_old_inverse_h.store(m[5]);
    ui_layout::pixel_to_viewport(m,w,h);
    if (copy_write(out,m,sizeof(m))) ++g_hits[6];
  }
  return result;
}

struct Hook { std::uintptr_t rva; const unsigned char* sig; size_t size; void* detour; void** original; };
}

namespace ui_layout {
void set_enabled(bool enabled) { g_enabled.store(enabled); }
void install(unsigned char* base, Log log) {
  g_log = log;
  // The 14:25 headset run rejected this query-only attempt. Keep it available
  // for research, but do not install or modify UI queries in normal play.
  if (!g_enabled.load()) {
    if (g_log) g_log("ui layout: query-only experiment retired; normal UI queries preserved");
    return;
  }
  // STATIC retail98588da2: exact instruction prefixes, not guessed entrypoints.
  static const unsigned char size[] = {0x40,0x53,0x55,0x41,0x54,0x41,0x57,0x48,0x83,0xEC,0x28};
  static const unsigned char aspect[] = {0x40,0x55,0x41,0x54,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x38};
  static const unsigned char safe[] = {0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x18,0x48,0x89,0x7C,0x24,0x20};
  static const unsigned char viewport[] = {0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x18,0x57,0x48,0x83,0xEC,0x20};
  static const unsigned char game[] = {0x40,0x53,0x48,0x83,0xEC,0x40,0x48,0x8B,0xDA};
  static const unsigned char to_pixel[] = {0x33,0xC0,0x48,0xC7,0x02,0x00,0x00,0x80,0x3F};
  static const unsigned char to_viewport[] = {0x40,0x53,0x48,0x83,0xEC,0x60,0x49,0x8D,0x88,0xC0,0x00,0x00,0x00};
  Hook hooks[] = {
    {0x6284A0,size,sizeof(size),reinterpret_cast<void*>(&Size_hook),reinterpret_cast<void**>(&g_size)},
    {0x628380,aspect,sizeof(aspect),reinterpret_cast<void*>(&Aspect_hook),reinterpret_cast<void**>(&g_aspect)},
    {0x62E090,safe,sizeof(safe),reinterpret_cast<void*>(&Safe_hook),reinterpret_cast<void**>(&g_safe)},
    {0x62DFB0,viewport,sizeof(viewport),reinterpret_cast<void*>(&Viewport_hook),reinterpret_cast<void**>(&g_viewport)},
    {0x430D620,game,sizeof(game),reinterpret_cast<void*>(&GameSize_hook),reinterpret_cast<void**>(&g_game_size)},
    {0x529570,to_pixel,sizeof(to_pixel),reinterpret_cast<void*>(&ToPixel_hook),reinterpret_cast<void**>(&g_to_pixel)},
    {0x529020,to_viewport,sizeof(to_viewport),reinterpret_cast<void*>(&ToViewport_hook),reinterpret_cast<void**>(&g_to_viewport)}
  };
  unsigned char got[16];
  static const unsigned char slot_sig[] = {0x41,0x56,0x48,0x83,0xEC,0x50,0x65,0x48,0x8B,0x04,0x25,0x58,0x00,0x00,0x00};
  if (!copy_read(base+0x32C77A0,got,sizeof(slot_sig)) || std::memcmp(got,slot_sig,sizeof(slot_sig))!=0) {
    if (g_log) g_log("ui layout: TLS accessor mismatch; overrides disabled");
    return;
  }
  g_base=base;
  g_slot=reinterpret_cast<Slot>(base+0x32C77A0);
  for (const auto& hook : hooks) {
    if (!copy_read(base+hook.rva,got,hook.size) || std::memcmp(got,hook.sig,hook.size) != 0) {
      if (g_log) g_log("ui layout: signature mismatch; entire logical-canvas override disabled");
      return;
    }
  }
  unsigned made=0, enabled=0;
  for (; made<7; ++made) if (MH_CreateHook(base+hooks[made].rva,hooks[made].detour,hooks[made].original) != MH_OK) break;
  if (made==7) for (; enabled<7; ++enabled) if (MH_EnableHook(base+hooks[enabled].rva) != MH_OK) break;
  if (made!=7 || enabled!=7) {
    for (unsigned i=0; i<enabled; ++i) MH_DisableHook(base+hooks[i].rva);
    for (unsigned i=0; i<made; ++i) MH_RemoveHook(base+hooks[i].rva);
    if (g_log) g_log("ui layout: incomplete hook set rolled back; overrides disabled");
    return;
  }
  g_armed.store(true);
  if (g_log) g_log("ui layout: 7 guarded engine UI size/conversion hooks installed; no global viewport writes");
}
void report() {
  if (!g_armed.load()) return;
  static ULONGLONG last=0;
  const auto now=GetTickCount64();
  if (now-last<5000 || g_log==nullptr) return;
  last=now;
  float w=0,h=0;
  const bool active=canvas(&w,&h);
  char line[500];
  std::snprintf(line,sizeof(line),"ui layout: active=%d eye=%.0fx%.0f last screen=%.0fx%.0f hits size/aspect/safe/viewport/game/toPixel/toViewport=%llu/%llu/%llu/%llu/%llu/%llu/%llu skips=%llu",
      active,w,h,g_old_w.load(),g_old_h.load(),g_hits[0].load(),g_hits[1].load(),g_hits[2].load(),g_hits[3].load(),g_hits[4].load(),g_hits[5].load(),g_hits[6].load(),g_skips.load());
  g_log(line);
  std::snprintf(line,sizeof(line),"ui layout original conversions: toPixel scale=%.6g,%.6g toViewport scale=%.6g,%.6g (read before override)",
      g_old_pixel_w.load(),g_old_pixel_h.load(),g_old_inverse_w.load(),g_old_inverse_h.load());
  g_log(line);
  std::snprintf(line,sizeof(line),"ui layout original context: last window rect %.0fx%.0f (custom rectangles pass through)",
      g_context_w.load(),g_context_h.load());
  g_log(line);
}
}
