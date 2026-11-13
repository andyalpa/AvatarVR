#include "ui_piece.hpp"
#include "ui_ownership.hpp"
#include <windows.h>
#include <MinHook.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <intrin.h>

// STATIC (sdk_decomp.c + AvrQuery 2026-10-18, build 98588da2):
// - UI settings manager *0x1496A28B8 (read by Set_UI_Config_Slot 0x62E560 and the other UI piece nodes).
//   +0xE0 piece defaults, 0x28 each: offset xy +0/+4, scale +0x20, configurable +0x24.
//   +0xF0 game override, 2 bytes each: value, active.  +0x108 active config slot (<3).
//   Slot table at mgr+(slot+0xB)*0x10, 0x1C each: offset xy +0/+4, custom offset +8, scale +0xC,
//   custom scale +0x10, enabled +0x14, custom enabled +0x18.
// - Piece ids are the enum type "Gfx_MovablePieceID" at 0x1496F8A60 (built by 0x140646CC0); its value
//   count is 0x143159840: +0x18, plus children +0x70/+0x78 (recursive), plus +0x80/+0x88 entries' +0x18.
// - Get_UI_Piece_Enabled(_State) 0x627650/0x627770: ids at *(p+0x28), count flags+0x1C &0xFFFF,
//   bit28 = indirect, id at +i*0x18+8.
// - GetCrosshairViewportPosition 0x3F5E970 writes a float2 to *(p+0x20) (scalar when flags&0x1000FFFF==1).
// All of the above are leads; this probe only reads them and logs what it sees.

namespace {
using Node = void (*)(void*, void*);
Node g_enabled_node = nullptr, g_state_node = nullptr, g_crosshair_node = nullptr;
unsigned char* g_base = nullptr;
ui_piece::Log g_log = nullptr;
std::atomic<bool> g_on{true};
bool g_ready = false;
constexpr unsigned kMaxPieces = 128;
std::atomic<unsigned long long> g_queries[kMaxPieces]{};
std::atomic<unsigned char> g_last_enabled[kMaxPieces]{};
std::atomic<unsigned long long> g_query_calls{0}, g_crosshair_calls{0};
std::atomic<std::uint64_t> g_crosshair_bits{0};
std::atomic<ULONGLONG> g_health_query_tick{0};  // piece #12 (health bar group), the most-queried live HUD piece

bool read(const void* p, void* out, size_t n) {
  if (!p) return false;
  __try { std::memcpy(out, p, n); return true; }
  __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}
template <class T> bool at(const void* p, size_t off, T* out) {
  return p && read(static_cast<const unsigned char*>(p) + off, out, sizeof(T));
}

// Same rule as 0x143159840, bounded and fault-safe.
bool enum_count(const unsigned char* type, int depth, unsigned long long* total) {
  if (!type || depth > 8) return false;
  unsigned long long own = 0; const unsigned char* kids = nullptr; unsigned nkids = 0;
  const unsigned char* extra = nullptr; unsigned nextra = 0;
  if (!at(type, 0x18, &own) || !at(type, 0x70, &kids) || !at(type, 0x78, &nkids) ||
      !at(type, 0x80, &extra) || !at(type, 0x88, &nextra) || nkids > 64 || nextra > 64) return false;
  *total += own;
  for (unsigned i = 0; i < nkids; ++i) {
    const unsigned char* k = nullptr;
    if (!at(kids, i * 8, &k) || !enum_count(k, depth + 1, total)) return false;
  }
  for (unsigned i = 0; i < nextra; ++i) {
    const unsigned char* e = nullptr; unsigned long long n = 0;
    if (!at(extra, i * 8, &e) || !at(e, 0x18, &n)) return false;
    *total += n;
  }
  return *total < 100000;
}

void count_ids(void* params) {
  if (!g_on.load(std::memory_order_relaxed)) return;
  ++g_query_calls;
  const unsigned char* ids = nullptr; unsigned flags = 0;
  if (!at(params, 0x28, &ids) || !at(ids, 0x1C, &flags)) return;
  if ((flags >> 28) & 1) { const unsigned char* inner = nullptr; if (!at(ids, 0, &inner)) return; ids = inner; }
  const unsigned n = flags & 0xFFFF;
  const unsigned char* out = nullptr; unsigned oflags = 0;
  const bool have_out = at(params, 0x20, &out) && at(out, 0x1C, &oflags);
  if (have_out && (oflags >> 28) & 1) { const unsigned char* inner = nullptr; if (!at(out, 0, &inner)) out = nullptr; else out = inner; }
  for (unsigned i = 0; i < n && i < 64; ++i) {
    unsigned id = 0;
    if (!at(ids, i * 0x18 + 8, &id) || id >= kMaxPieces) continue;
    g_queries[id].fetch_add(1, std::memory_order_relaxed);
    if (id == 12) g_health_query_tick.store(GetTickCount64(), std::memory_order_relaxed);
    unsigned char v = 0;
    if (have_out && out && at(out, i, &v)) g_last_enabled[id].store(static_cast<unsigned char>(v ? 2 : 1), std::memory_order_relaxed);
  }
}
void Enabled_hook(void* self, void* params) {
  const auto caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(g_base);
  g_enabled_node(self, params); count_ids(params);
  ui_ownership::piece(self,params,false,caller);
}
void State_hook(void* self, void* params) {
  const auto caller=reinterpret_cast<std::uintptr_t>(_ReturnAddress())-reinterpret_cast<std::uintptr_t>(g_base);
  g_state_node(self, params); count_ids(params);
  ui_ownership::piece(self,params,true,caller);
}
void Crosshair_hook(void* self, void* params) {
  g_crosshair_node(self, params);
  if (!g_on.load(std::memory_order_relaxed)) return;
  ++g_crosshair_calls;
  const unsigned char* out = nullptr; unsigned flags = 0; std::uint64_t bits = 0;
  if (at(params, 0x20, &out) && at(out, 0x1C, &flags) && (flags & 0x1000FFFF) == 1 && at(out, 0, &bits))
    g_crosshair_bits.store(bits, std::memory_order_relaxed);
}

struct Piece { float def[2]{}, def_scale = 0, off[2]{}, scale = 0; unsigned char configurable = 0, custom_off = 0,
               custom_scale = 0, custom_en = 0, ov_value = 0, ov_active = 0; unsigned enabled = 0; };
Piece g_last[kMaxPieces];
unsigned g_last_count = 0, g_last_slot = ~0u;
bool g_have_last = false;

const unsigned char* manager() {
  const unsigned char* m = nullptr;
  return at(g_base, 0x96A28B8, &m) ? m : nullptr;
}
bool snapshot(Piece* out, unsigned* count, unsigned* slot) {
  const unsigned char* m = manager();
  unsigned long long total = 0;
  if (!m || !enum_count(g_base + 0x96F8A60, 0, &total) || total == 0) return false;
  const unsigned n = total < kMaxPieces ? static_cast<unsigned>(total) : kMaxPieces;
  const unsigned char *defs = nullptr, *ov = nullptr, *rec = nullptr;
  if (!at(m, 0xE0, &defs) || !at(m, 0xF0, &ov) || !at(m, 0x108, slot) || *slot >= 3 ||
      !at(m, (static_cast<size_t>(*slot) + 0xB) * 0x10, &rec) || !defs || !rec) return false;
  for (unsigned i = 0; i < n; ++i) {
    Piece p; std::memset(&p, 0, sizeof(p));  // padding too: report() diffs with memcmp
    const unsigned char* d = defs + i * 0x28; const unsigned char* r = rec + i * 0x1C;
    if (!read(d, p.def, 8) || !at(d, 0x20, &p.def_scale) || !at(d, 0x24, &p.configurable) ||
        !read(r, p.off, 8) || !at(r, 8, &p.custom_off) || !at(r, 0xC, &p.scale) || !at(r, 0x10, &p.custom_scale) ||
        !at(r, 0x14, &p.enabled) || !at(r, 0x18, &p.custom_en)) return false;
    if (ov) { at(ov, i * 2, &p.ov_value); at(ov, i * 2 + 1, &p.ov_active); }
    out[i] = p;
  }
  *count = n;
  return true;
}
void log_piece(const char* tag, unsigned i, const Piece& p) {
  char line[400];
  std::snprintf(line, sizeof(line),
    "ui piece %s #%u: default off=%.4g,%.4g scale=%.4g configurable=%u | slot off=%.4g,%.4g custom=%u scale=%.4g custom=%u enabled=%u custom=%u | override value=%u active=%u | queried=%llu last=%u",
    tag, i, p.def[0], p.def[1], p.def_scale, p.configurable, p.off[0], p.off[1], p.custom_off, p.scale, p.custom_scale,
    p.enabled, p.custom_en, p.ov_value, p.ov_active, g_queries[i].load(), g_last_enabled[i].load());
  g_log(line);
}
void log_type_once() {
  // Layout of the enum type beyond the count fields is OPEN; dump raw words and any pointers to text.
  static bool done = false;
  if (done) return;
  done = true;
  const unsigned char* t = g_base + 0x96F8A60;
  std::uint64_t q[24]{};
  if (!read(t, q, sizeof(q))) { g_log("ui piece: enum type unreadable"); return; }
  char line[600]; int len = std::snprintf(line, sizeof(line), "ui piece type Gfx_MovablePieceID raw:");
  for (int i = 0; i < 24 && len < 560; ++i) len += std::snprintf(line + len, sizeof(line) - len, " %llX", static_cast<unsigned long long>(q[i]));
  g_log(line);
  for (int i = 0; i < 24; ++i) {
    char text[48] = {};
    if (q[i] > 0x10000 && read(reinterpret_cast<const void*>(q[i]), text, sizeof(text) - 1)) {
      bool printable = text[0] >= 0x20 && text[0] < 0x7F;
      for (int k = 0; printable && k < 4; ++k) printable = text[k] >= 0x20 && text[k] < 0x7F;
      for (int k = 0; k < 47; ++k) if (text[k] < 0x20 || text[k] >= 0x7F) { text[k] = 0; break; }
      if (printable) { std::snprintf(line, sizeof(line), "ui piece type +%X -> \"%s\"", i * 8, text); g_log(line); }
    }
  }
  // Ctor 0x143127250 zeroes +0x0..+0x38, so the heap blocks at +0x0/+0x8/+0x10 are filled at run time. Their layout
  // is OPEN: log every text reachable directly or one pointer deep, so member names can be matched to ids.
  unsigned logged = 0;
  for (int src = 0; src < 3; ++src) {
    const auto* block = reinterpret_cast<const unsigned char*>(q[src]);
    if (q[src] < 0x10000) continue;
    for (unsigned off = 0; off < 0x1000 && logged < 300; off += 8) {
      std::uint64_t v = 0;
      if (!at(block, off, &v)) break;
      for (int depth = 0; depth < 2 && v > 0x10000; ++depth) {
        char text[64] = {};
        if (!read(reinterpret_cast<const void*>(v), text, sizeof(text) - 1)) break;
        int k = 0;
        while (k < 63 && text[k] >= 0x20 && text[k] < 0x7F) ++k;
        if (k >= 3 && (text[k] == 0 || k == 63)) {
          text[k] = 0;
          std::snprintf(line, sizeof(line), "ui piece name src%d +%X depth%d -> \"%s\"", src, off, depth, text);
          g_log(line); ++logged;
          break;
        }
        std::uint64_t inner = 0;
        if (!at(reinterpret_cast<const void*>(v), 0, &inner)) break;
        v = inner;
      }
    }
  }
  std::snprintf(line, sizeof(line), "ui piece name harvest: %u texts", logged);
  g_log(line);
}
}

namespace ui_piece {
void set_enabled(bool enabled) { g_on.store(enabled, std::memory_order_relaxed); ui_ownership::allow(enabled); }
bool hud_live(unsigned ms) {
  const ULONGLONG t = g_health_query_tick.load(std::memory_order_relaxed);
  return t != 0 && GetTickCount64() - t <= ms;
}

void install(unsigned char* base, Log log) {
  g_base = base; g_log = log;
  // Set_UI_Config_Slot: mov rbx,[rip+disp] must resolve to the manager global (same build check).
  static const unsigned char slot_sig[] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0x1D};
  static const unsigned char type_sig[] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x65,0x48,0x8B,0x04,0x25,0x58,0,0,0};
  static const unsigned char en_sig[] = {0x40,0x55,0x41,0x54,0x41,0x57,0x48,0x83,0xEC,0x20,0x4C,0x8B,0x62,0x28,0x41,0x8B,0x44,0x24,0x1C,0x44,0x0F,0xB7,0xF8};
  static const unsigned char st_sig[] = {0x40,0x55,0x41,0x54,0x41,0x57,0x48,0x83,0xEC,0x20,0x4C,0x8B,0x62,0x28,0x41,0x8B,0x44,0x24,0x1C,0x0F,0xB7,0xE8};
  static const unsigned char ch_sig[] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0x5A,0x20,0x8B,0x43,0x1C,0x25,0xFF,0xFF,0x00,0x10};
  unsigned char got[32]; std::int32_t disp = 0;
  if (!read(base + 0x62E560, got, sizeof(slot_sig)) || std::memcmp(got, slot_sig, sizeof(slot_sig)) ||
      !read(base + 0x62E569, &disp, 4) || 0x62E56D + static_cast<std::int64_t>(disp) != 0x96A28B8 ||
      !read(base + 0x646CC0, got, sizeof(type_sig)) || std::memcmp(got, type_sig, sizeof(type_sig))) {
    if (log) log("ui piece: manager/type guard mismatch; probe disabled");
    return;
  }
  g_ready = true;
  struct Hook { size_t rva; const unsigned char* sig; size_t size; void* detour; void** original; };
  Hook hooks[] = {
    {0x627650, en_sig, sizeof(en_sig), reinterpret_cast<void*>(&Enabled_hook), reinterpret_cast<void**>(&g_enabled_node)},
    {0x627770, st_sig, sizeof(st_sig), reinterpret_cast<void*>(&State_hook), reinterpret_cast<void**>(&g_state_node)},
    {0x3F5E970, ch_sig, sizeof(ch_sig), reinterpret_cast<void*>(&Crosshair_hook), reinterpret_cast<void**>(&g_crosshair_node)},
  };
  for (const auto& h : hooks) if (!read(base + h.rva, got, h.size) || std::memcmp(got, h.sig, h.size)) {
    if (log) log("ui piece: node guard mismatch; memory dump only, no node hooks");
    return;
  }
  unsigned made = 0, enabled = 0;
  for (; made < 3; ++made) if (MH_CreateHook(base + hooks[made].rva, hooks[made].detour, hooks[made].original) != MH_OK) break;
  if (made == 3) for (; enabled < 3; ++enabled) if (MH_EnableHook(base + hooks[enabled].rva) != MH_OK) break;
  if (made != 3 || enabled != 3) {
    for (unsigned i = 0; i < enabled; ++i) MH_DisableHook(base + hooks[i].rva);
    for (unsigned i = 0; i < made; ++i) MH_RemoveHook(base + hooks[i].rva);
    if (log) log("ui piece: node hook set rolled back; memory dump only");
    return;
  }
  if (log) log("ui piece: read-only probe installed (piece table diff, enabled queries, crosshair node; nouipiece disables)");
  ui_ownership::install(base,log);
}

void report() {
  ui_ownership::report();
  if (!g_ready || !g_log || !g_on.load(std::memory_order_relaxed)) return;
  static ULONGLONG last = 0, last_full = 0, last_cross = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last < 1000) return;
  last = now;
  log_type_once();
  Piece cur[kMaxPieces]; unsigned n = 0, slot = 0;
  if (!snapshot(cur, &n, &slot)) return;
  const bool full = !g_have_last || n != g_last_count || slot != g_last_slot || now - last_full >= 60000;
  char line[200];
  if (full) {
    last_full = now;
    std::snprintf(line, sizeof(line), "ui piece: %u pieces, config slot %u, enabled-node calls %llu", n, slot, g_query_calls.load());
    g_log(line);
    for (unsigned i = 0; i < n; ++i) log_piece("all", i, cur[i]);
  } else {
    for (unsigned i = 0; i < n; ++i) if (std::memcmp(&cur[i], &g_last[i], sizeof(Piece))) log_piece("changed", i, cur[i]);
  }
  std::memcpy(g_last, cur, n * sizeof(Piece));
  g_last_count = n; g_last_slot = slot; g_have_last = true;
  static std::uint64_t reported_bits = ~0ull;
  const auto bits = g_crosshair_bits.load(std::memory_order_relaxed);
  if (bits != reported_bits && now - last_cross >= 1000) {
    last_cross = now; reported_bits = bits;
    float xy[2]; std::memcpy(xy, &bits, sizeof(xy));
    std::snprintf(line, sizeof(line), "ui piece crosshair viewport: %.4f,%.4f (node calls %llu)", xy[0], xy[1], g_crosshair_calls.load());
    g_log(line);
  }
}
}
