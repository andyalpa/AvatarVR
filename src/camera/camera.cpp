#include "../runtime_log.hpp"
#include "camera.hpp"
#include "../diagnostics/render_activity.hpp"
#include "../diagnostics/hitch_probe.hpp"

#include "native_stereo.hpp"
#include "mount_camera.hpp"
#include "ui_canvas.hpp"
#include "ui_piece.hpp"
#include "ui_ownership.hpp"
#include "scan_uv_shader.hpp"
#include "screen_fx_shaders.hpp"
#include "scanner_inputs.hpp"
#include "scanner_geometry.hpp"
#include "../diagnostics/upload_sampler.hpp"
#include "../vr/wrist_hud.hpp"
#include "../vr/frame_gen.hpp"
#include "../dx12/dx12.hpp"

#include <MinHook.h>

#include <d3d12.h>
#include <d3dcompiler.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <intrin.h>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <string>
#include "../settings_file.hpp"

namespace {

constexpr int kDrawInstanced = 12;
constexpr int kDrawIndexedInstanced = 13;
constexpr int kDispatch = 14;
constexpr int kRsSetViewports = 21;
constexpr int kRsSetScissorRects = 22;
constexpr int kSetPipelineState = 25;
constexpr int kSetGraphicsRoot32BitConstants = 36;
constexpr int kSetGraphicsRootConstantBufferView = 38;
constexpr int kOmSetRenderTargets = 46;
constexpr int kExecuteIndirect = 59;

using RsSetViewportsFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, const D3D12_VIEWPORT*);
using SetGraphicsRoot32BitConstantsFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, UINT, const void*, UINT);
using SetGraphicsRootConstantBufferViewFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, D3D12_GPU_VIRTUAL_ADDRESS);

using DrawInstancedFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, UINT, UINT, UINT);
using DrawIndexedInstancedFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, UINT, UINT, INT, UINT);
using DispatchFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, UINT, UINT);
using SetPipelineStateFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, ID3D12PipelineState*);
using RsSetScissorRectsFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, const D3D12_RECT*);
using OmSetRenderTargetsFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, const D3D12_CPU_DESCRIPTOR_HANDLE*, BOOL,
                                                     const D3D12_CPU_DESCRIPTOR_HANDLE*);
using ExecuteIndirectFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, ID3D12CommandSignature*, UINT, ID3D12Resource*, UINT64,
                                                  ID3D12Resource*, UINT64);

RsSetViewportsFn g_viewports = nullptr;
SetGraphicsRoot32BitConstantsFn g_root_constants = nullptr;
SetGraphicsRootConstantBufferViewFn g_root_cbv = nullptr;
DrawInstancedFn g_draw = nullptr;
DrawIndexedInstancedFn g_draw_indexed = nullptr;
DispatchFn g_dispatch = nullptr;
SetPipelineStateFn g_set_pso = nullptr;
RsSetScissorRectsFn g_scissors = nullptr;
OmSetRenderTargetsFn g_render_targets = nullptr;
ExecuteIndirectFn g_execute_indirect = nullptr;
std::atomic<UINT> g_backbuffer_w{0};
std::atomic<UINT> g_backbuffer_h{0};
bool g_research = false;  // research observers (projection / view / viewport sampling); set by watch_list

// UI probe (2026-10-08, read-only). The game draws its UI inside each view's render (UI pass record 0x14055EFE0,
// shared into the second view), at the same pixels in both eyes; with per-eye centering the eye images point apart and
// the UI cannot fuse. Question: are the UI's D3D12 commands recorded on the thread that runs the UI pass, while it runs?
// If so, the UI can be moved and scaled per eye through its viewport/scissor. native_stereo tags the thread (1 = main
// view, 2 = second view); these counters say what the tagged thread records. Index 0 is unused.
struct UiStats {
  std::atomic<std::uint64_t> viewports{0}, scissors{0}, targets{0}, draws{0}, indexed{0}, indirect{0}, consts{0}, cbvs{0};
  std::atomic<std::uint64_t> dispatches{0}, layer_binds{0}, other_targets{0};
  std::atomic<std::uint64_t> fullscreen{0}, idx3{0}, idx6{0}, idx_other{0};
  std::atomic<std::uint64_t> half{0}, world{0}, reproj{0};
};
UiStats g_ui[3];
std::mutex g_ui_mutex;
constexpr int kUiShapes = 4;
D3D12_VIEWPORT g_ui_vp[3][kUiShapes] = {};  // distinct viewports seen in the UI pass since the last report
int g_ui_vp_count[3] = {};
D3D12_RECT g_ui_sc[3] = {};
bool g_ui_sc_seen[3] = {};
UINT g_ui_rt_count[3] = {};
SIZE_T g_ui_rt0[3] = {};
ID3D12GraphicsCommandList* g_ui_list[3] = {};
std::atomic<std::uint64_t> g_ui_placed[3];  // UI passes whose viewport was moved/scaled for the eye

// UI placement (native_stereo::ui_map). [LIVE] probe 2026-10-08 06:28: the UI's draws are recorded on the thread that
// runs the UI pass, into a render target of its own per eye; the main view's pass sets a full-target viewport and
// scissor once per pass, the second view's sets neither (it inherits them). So: after the pass binds its target, and
// on every viewport / scissor it sets, the mapped values go to the list instead; when the pass returns, the values it
// set itself (or, if it set none, the last ones set on that list on this thread) are put back unmapped.
thread_local ID3D12GraphicsCommandList* t_vp_list = nullptr;  // last viewport set on this thread: list and value
thread_local D3D12_VIEWPORT t_vp = {};
thread_local ID3D12GraphicsCommandList* t_sc_list = nullptr;  // last scissor set on this thread
thread_local D3D12_RECT t_sc = {};
thread_local ID3D12GraphicsCommandList* t_placed_list = nullptr;  // list whose state the UI placement changed
thread_local bool t_placed_sc = false;
// The viewport now in force on the placed list is a placed (mapped) full-target one. [LIVE] run 2026-10-08 07:22-07:23
// (scan ability): the main UI pass also ran the effect's own chain -- ~38 viewports and ~5 DrawInstanced per pass at
// 1172x1271 / 996x1080 (the upscaler's input sizes) -- which the placement scaled too. Only viewports covering the
// whole eye target are the UI layer; everything else passes unmapped, and scissors / full-screen draws follow suit.
thread_local bool t_mapped_now = false;

// UI pass trace (2026-10-08, read-only). [LIVE] run 07:35-07:38 (scan ability): the picture still shrank by the HUD
// placement, and the scan highlights (3D silhouettes; idxother ~140 -> ~870 per pass) sat off their animals. To tell
// UI draws from the scene composite and the 3D highlights, the call sequence of one UI pass that sets a non-full-target
// viewport (scan active) is recorded on its thread and logged (at most one every 10 s; one plain pass every 30 s for
// comparison). Repeated identical draws are run-length merged.
struct TraceEv {
  char kind;     // R rt bind, V viewport, S scissor, D DrawInstanced, I DrawIndexedInstanced, C root CBV
  UINT a, b;     // R: count, has depth | V: w, h | S: w, h | D: vertices, instances | I: indices, instances
  int x, y;      // V/S: top-left; R: low bits of rt0
  bool mapped;   // V: placed; D/I: drawn under a placed viewport
  UINT n;        // run length
};
constexpr int kTraceMax = 160;
thread_local TraceEv t_trace[kTraceMax];
thread_local int t_trace_n = 0;
thread_local bool t_trace_odd = false;  // a non-full-target viewport was set in this pass
thread_local bool t_depth_bound = false;  // the pass's current targets include a DSV
std::atomic<ULONGLONG> g_trace_last_odd{0}, g_trace_last_plain{0};
// Screen effects (2026-10-23, research/SCREEN_EFFECTS_20261023.md): with UI placement off, flash / mount boost / scan
// draw the whole picture into the window-canvas region (1280x720) of the eye-size UI raster. A pass that sets a viewport
// of the window client size on a larger target is traced on its own (every 2 s), with render-target sizes.
thread_local bool t_trace_canvas = false;
std::atomic<ULONGLONG> g_trace_last_canvas{0};
std::atomic<std::uint64_t> g_draws_dsv_mapped{0}, g_draws_nodsv_mapped{0};

void trace_ev(char kind, UINT a, UINT b, int x, int y, bool mapped) {
  if (t_trace_n > 0) {
    TraceEv& last = t_trace[t_trace_n - 1];
    if (last.kind == kind && last.a == a && last.b == b && last.x == x && last.y == y && last.mapped == mapped) {
      ++last.n;
      return;
    }
  }
  if (t_trace_n < kTraceMax) {
    t_trace[t_trace_n++] = {kind, a, b, x, y, mapped, 1};
  }
}

// Scale of the placed target relative to the eye image: 1 = full size, 0.5 = the half-size target. [LIVE] trace
// 2026-10-08 07:46-07:48: the frosted glass is drawn by the neighbour passes as quads into a 996x1080 target (half of
// 1992x2160) with no DSV, then composited; with full-target-only placement it came back at the unplaced positions. The
// scan's 3D work uses 1172x1271 (the upscaler input) with a DSV and is left alone.
thread_local float t_map_scale = 1.f;
thread_local bool t_odd_vp = false;  // the pass's last viewport was neither full nor half size (left unplaced)
thread_local float t_panel_source_w = 0.f, t_panel_source_h = 0.f;
thread_local float t_panel_target_w = 0.f, t_panel_target_h = 0.f;

bool full_target(const D3D12_VIEWPORT& v, float w, float h) {
  return std::fabs(v.TopLeftX) <= 1.f && std::fabs(v.TopLeftY) <= 1.f && std::fabs(v.Width - w) <= 2.f && std::fabs(v.Height - h) <= 2.f;
}

// 1 for a full-target viewport, 0.5 for an exactly half-size one at the origin, 0 for anything else.
float target_scale(const D3D12_VIEWPORT& v, float w, float h) {
  if (full_target(v, w, h)) {
    return 1.f;
  }
  if (std::fabs(v.TopLeftX) <= 1.f && std::fabs(v.TopLeftY) <= 1.f && std::fabs(v.Width - 0.5f * w) <= 2.f &&
      std::fabs(v.Height - 0.5f * h) <= 2.f) {
    return 0.5f;
  }
  return 0.f;
}

// Scan highlights. [LIVE] same trace: during the scan the UI pass also draws the highlight outlines (index counts like
// 5580, 894, 528, interleaved with 6-index quads) under the full-target viewport; placed, they no longer sit on their
// animals. Index counts cannot tell them from UI (both are multiples of 6), the pipeline state can: PSOs used by
// placed draws while no scan runs are learned as UI; while a scan runs (a UI-tagged pass set a viewport that is neither
// full nor half size within the last 0.5 s), a placed draw with a PSO not in that set is drawn with the game's own
// viewport.
thread_local ID3D12PipelineState* t_pso = nullptr;
std::atomic<ULONGLONG> g_scan_tick{0};

// Scan signal candidates (2026-10-08, read-only). Two guessed signals fired outside the scan too (a non-full viewport,
// 07:51; a DSV bind, 08:26). Per tagged-pass kind (0 = UI pass, 1 = neighbour passes) these count, per second: target
// binds with RTs + DSV ("rd"), DSV-only binds ("d"), viewports at a size neither full nor half ("odd"), full-screen
// DrawInstanced at half size ("h3"), and indexed draws of more than 3000 indices ("big"). One "scansig" line a second in
// CAMERA_OBSERVE.log; a timed run (no scan / scan / no scan) shows which is zero without the scan.
std::atomic<std::uint64_t> g_sig_rd[2], g_sig_d[2], g_sig_odd[2], g_sig_h3[2], g_sig_big[2];
std::atomic<ULONGLONG> g_sig_last{0};

void sig_count(std::atomic<std::uint64_t>* arr) {
  const int k = native_stereo::ui_pass_kind();
  arr[k == 1 ? 1 : 0].fetch_add(1, std::memory_order_relaxed);
}

void sig_report();
constexpr int kPsoSlots = 1024;
std::atomic<ID3D12PipelineState*> g_ui_psos[kPsoSlots];
std::atomic<int> g_ui_pso_count{0};
// When each PSO was learned, and whether it was revoked. [AUTHOR] + captures 2026-10-08 12:1x: the first scan lined up,
// later ones shrank the picture and moved the highlights again: the scan signal (a neighbour pass's RT + DSV bind)
// arrives a frame or two after the scan starts, and the scan's own PSOs were learned as UI in those frames. At each
// scan start, PSOs learned in the 2 s before are revoked for good: unknown during scans, never learned again.
std::atomic<ULONGLONG> g_ui_pso_time[kPsoSlots];
std::atomic<bool> g_ui_pso_revoked[kPsoSlots];
std::atomic<int> g_ui_pso_revokes{0};

bool scan_active() { return GetTickCount64() - g_scan_tick.load(std::memory_order_relaxed) < 500; }

size_t pso_slot(ID3D12PipelineState* p) { return (reinterpret_cast<std::uintptr_t>(p) >> 4) * 0x9E3779B97F4A7C15ull >> 54; }

bool ui_pso_known(ID3D12PipelineState* p) {
  if (p == nullptr) {
    return true;
  }
  for (size_t i = pso_slot(p), n = 0; n < kPsoSlots; ++n, i = (i + 1) % kPsoSlots) {
    ID3D12PipelineState* v = g_ui_psos[i].load(std::memory_order_relaxed);
    if (v == p) {
      return !g_ui_pso_revoked[i].load(std::memory_order_relaxed);
    }
    if (v == nullptr) {
      return false;
    }
  }
  return false;
}

// PSOs drawn in a placed UI pass while the scan ran and not known as UI: the scan's own (its picture and highlights).
// [AUTHOR] 2026-10-08 12:3x: after closing the scanner the view stayed different for a moment -- its fade-out draws
// come after the signal stops. A PSO in this set is handled the scan way whatever the signal, and never learned as UI.
std::atomic<ID3D12PipelineState*> g_scan_psos[kPsoSlots];
std::atomic<ID3D12PipelineState*> g_picture_psos[kPsoSlots];
std::mutex g_picture_mutex;
unsigned g_picture_count=0;
bool g_picture_on=true; // scoped by fxstretch; startup no-picture opt-out
bool g_exact_fx_route=true; // startup legacy-route opt-out; captured shaders only
std::atomic<unsigned> g_exact_scan_draws{0},g_exact_scan_without_signal{0};
std::atomic<unsigned> g_picture_draws{0};

bool pso_set_has(std::atomic<ID3D12PipelineState*>* set, ID3D12PipelineState* p) {
  if (p == nullptr) {
    return false;
  }
  for (size_t i = pso_slot(p), n = 0; n < kPsoSlots; ++n, i = (i + 1) % kPsoSlots) {
    ID3D12PipelineState* v = set[i].load(std::memory_order_relaxed);
    if (v == p) {
      return true;
    }
    if (v == nullptr) {
      return false;
    }
  }
  return false;
}

void pso_set_add(std::atomic<ID3D12PipelineState*>* set, ID3D12PipelineState* p) {
  if (p == nullptr) {
    return;
  }
  for (size_t i = pso_slot(p), n = 0; n < kPsoSlots / 2; ++n, i = (i + 1) % kPsoSlots) {
    ID3D12PipelineState* v = set[i].load(std::memory_order_relaxed);
    if (v == p) {
      return;
    }
    if (v == nullptr && (set[i].compare_exchange_strong(v, p) || v == p)) {
      return;
    }
  }
}

// Right eye: whether the current scan draw goes through the left-to-right reprojection (draw_unplaced).
thread_local bool t_reproj_draw = false;
void srv_probe_draw(ID3D12GraphicsCommandList* list, ID3D12PipelineState* pso, UINT count);  // SRV probe, below
int mask_shift_pre(ID3D12GraphicsCommandList* list);   // scan mask shift around the second view's composite, below
bool ov_pre(ID3D12GraphicsCommandList* list);           // scan overlay redirect, below
void ov_post(ID3D12GraphicsCommandList* list);
void mask_shift_post(ID3D12GraphicsCommandList* list);
std::atomic<bool> g_scan_shift_on{false};  // scan mask shift (see scan_shift); off: 13:1x 0 shifts / 157 skips, SRV probe first

std::uint32_t blend_pack_of(ID3D12PipelineState* p);  // defined with the blend table below

// Scan draw table (right eye): distinct (PSO, index count, blend pack, reprojected) with counts, logged every 10 s.
struct ScanDraw {
  ID3D12PipelineState* pso;
  UINT count;
  std::uint32_t pack;
  bool reproj;
  std::uint64_t n;
};
ScanDraw g_scan_draws[32] = {};
int g_scan_draw_n = 0;
std::mutex g_scan_draw_mutex;
std::atomic<ULONGLONG> g_scan_draw_logged{0};

void scan_draw_note(ID3D12PipelineState* pso, UINT count, bool reproj) {
  std::unique_lock<std::mutex> lock(g_scan_draw_mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  for (int i = 0; i < g_scan_draw_n; ++i) {
    ScanDraw& d = g_scan_draws[i];
    if (d.pso == pso && d.count == count && d.reproj == reproj) {
      ++d.n;
      return;
    }
  }
  if (g_scan_draw_n < 32) {
    g_scan_draws[g_scan_draw_n++] = {pso, count, blend_pack_of(pso), reproj, 1};
  }
}

// Blend state per PSO (2026-10-08). [AUTHOR] + capture 12:4x: with only the meshes reprojected the right eye keeps its
// own picture, but its highlights sit at the left eye's pixels again: they are quads (6 indices) that composite the
// main view's highlight images, like the scan's opaque picture draw that must stay in place. What tells them apart is
// the blend state: the picture replaces pixels, the highlights blend over them. The device's PSO creation (graphics
// desc, stream desc, pipeline library loads) records whether render target 0 blends. 1 = blends, 2 = opaque.
constexpr int kBlendSlots = 1 << 15;
std::atomic<ID3D12PipelineState*> g_blend_key[kBlendSlots];
std::atomic<std::uint8_t> g_blend_val[kBlendSlots];
// Packed blend of render target 0: SrcBlend | DestBlend << 5 | BlendOp << 10 | SrcBlendAlpha << 13 | DestBlendAlpha << 18 |
// WriteMask << 23 | BlendEnable << 27. [AUTHOR] + capture 12:5x: with BlendEnable alone, the scan's picture draw also
// counted as blended (reprojected -> black bar again); its factors tell it apart.
std::atomic<std::uint32_t> g_blend_pack[kBlendSlots];

std::uint32_t blend_pack(const D3D12_RENDER_TARGET_BLEND_DESC& r) {
  return static_cast<std::uint32_t>(r.SrcBlend & 31) | (static_cast<std::uint32_t>(r.DestBlend & 31) << 5) |
         (static_cast<std::uint32_t>(r.BlendOp & 7) << 10) | (static_cast<std::uint32_t>(r.SrcBlendAlpha & 31) << 13) |
         (static_cast<std::uint32_t>(r.DestBlendAlpha & 31) << 18) | (static_cast<std::uint32_t>(r.RenderTargetWriteMask & 15) << 23) |
         (r.BlendEnable ? (1u << 27) : 0u);
}
thread_local std::uint32_t t_blend_pack_pending = 0;
std::atomic<std::uint64_t> g_pso_seen[4];  // graphics desc, stream, library graphics, library stream
std::atomic<std::uint64_t> g_pso_blended{0};

size_t blend_slot(ID3D12PipelineState* p) { return (reinterpret_cast<std::uintptr_t>(p) >> 4) * 0x9E3779B97F4A7C15ull >> 49; }

void blend_note(ID3D12PipelineState* p, bool blends) {
  if (p == nullptr) {
    return;
  }
  for (size_t i = blend_slot(p), n = 0; n < 64; ++n, i = (i + 1) % kBlendSlots) {
    ID3D12PipelineState* v = g_blend_key[i].load(std::memory_order_relaxed);
    if (v == p || (v == nullptr && (g_blend_key[i].compare_exchange_strong(v, p) || v == p))) {
      g_blend_val[i].store(blends ? 1 : 2, std::memory_order_relaxed);
      g_blend_pack[i].store(t_blend_pack_pending, std::memory_order_relaxed);
      if (blends) {
        g_pso_blended.fetch_add(1, std::memory_order_relaxed);
      }
      return;
    }
  }
}

std::uint32_t blend_pack_of(ID3D12PipelineState* p) {
  if (p == nullptr) {
    return 0;
  }
  for (size_t i = blend_slot(p), n = 0; n < 64; ++n, i = (i + 1) % kBlendSlots) {
    ID3D12PipelineState* v = g_blend_key[i].load(std::memory_order_relaxed);
    if (v == p) {
      return g_blend_pack[i].load(std::memory_order_relaxed);
    }
    if (v == nullptr) {
      return 0;
    }
  }
  return 0;
}

int blend_of(ID3D12PipelineState* p) {  // 0 unknown, 1 blends, 2 opaque
  if (p == nullptr) {
    return 0;
  }
  for (size_t i = blend_slot(p), n = 0; n < 64; ++n, i = (i + 1) % kBlendSlots) {
    ID3D12PipelineState* v = g_blend_key[i].load(std::memory_order_relaxed);
    if (v == p) {
      return g_blend_val[i].load(std::memory_order_relaxed);
    }
    if (v == nullptr) {
      return 0;
    }
  }
  return 0;
}

// Blends = keeps some of what is underneath: blending on and a destination factor other than ZERO (D3D12_BLEND_ZERO = 1).
// A draw that blends with DestBlend ZERO replaces the pixels like an opaque one.
bool blend_desc_blends(const D3D12_BLEND_DESC& b) {
  t_blend_pack_pending = blend_pack(b.RenderTarget[0]);
  return (b.RenderTarget[0].BlendEnable != FALSE && b.RenderTarget[0].DestBlend != D3D12_BLEND_ZERO) ||
         b.RenderTarget[0].LogicOpEnable != FALSE;
}

// Stream desc: subobjects are { type; inner } aligned to a pointer (CD3DX12_PIPELINE_STATE_STREAM_SUBOBJECT). Inner
// sizes from d3d12.h; an unknown type stops the walk (blend then unknown). No BLEND subobject = default (opaque).
int stream_blend(const void* data, SIZE_T size) {
  const unsigned char* p = static_cast<const unsigned char*>(data);
  SIZE_T off = 0;
  int result = 2;
  __try {
    while (off + sizeof(D3D12_PIPELINE_STATE_SUBOBJECT_TYPE) <= size) {
      const auto type = *reinterpret_cast<const D3D12_PIPELINE_STATE_SUBOBJECT_TYPE*>(p + off);
      size_t isz = 0, ial = 4;
      switch (type) {
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_ROOT_SIGNATURE: isz = sizeof(void*); ial = alignof(void*); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_VS:
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PS:
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DS:
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_HS:
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_GS:
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_AS:
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_MS: isz = sizeof(D3D12_SHADER_BYTECODE); ial = alignof(D3D12_SHADER_BYTECODE); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_STREAM_OUTPUT: isz = sizeof(D3D12_STREAM_OUTPUT_DESC); ial = alignof(D3D12_STREAM_OUTPUT_DESC); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_BLEND: isz = sizeof(D3D12_BLEND_DESC); ial = alignof(D3D12_BLEND_DESC); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_MASK: isz = sizeof(UINT); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RASTERIZER: isz = sizeof(D3D12_RASTERIZER_DESC); ial = alignof(D3D12_RASTERIZER_DESC); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL: isz = sizeof(D3D12_DEPTH_STENCIL_DESC); ial = alignof(D3D12_DEPTH_STENCIL_DESC); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_INPUT_LAYOUT: isz = sizeof(D3D12_INPUT_LAYOUT_DESC); ial = alignof(D3D12_INPUT_LAYOUT_DESC); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_IB_STRIP_CUT_VALUE: isz = sizeof(D3D12_INDEX_BUFFER_STRIP_CUT_VALUE); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_PRIMITIVE_TOPOLOGY: isz = sizeof(D3D12_PRIMITIVE_TOPOLOGY_TYPE); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_RENDER_TARGET_FORMATS: isz = sizeof(D3D12_RT_FORMAT_ARRAY); ial = alignof(D3D12_RT_FORMAT_ARRAY); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL_FORMAT: isz = sizeof(DXGI_FORMAT); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_SAMPLE_DESC: isz = sizeof(DXGI_SAMPLE_DESC); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_NODE_MASK: isz = sizeof(UINT); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_CACHED_PSO: isz = sizeof(D3D12_CACHED_PIPELINE_STATE); ial = alignof(D3D12_CACHED_PIPELINE_STATE); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_FLAGS: isz = sizeof(D3D12_PIPELINE_STATE_FLAGS); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_DEPTH_STENCIL1: isz = sizeof(D3D12_DEPTH_STENCIL_DESC1); ial = alignof(D3D12_DEPTH_STENCIL_DESC1); break;
        case D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_VIEW_INSTANCING: isz = sizeof(D3D12_VIEW_INSTANCING_DESC); ial = alignof(D3D12_VIEW_INSTANCING_DESC); break;
        default: return 0;  // unknown subobject: stop
      }
      const size_t ioff = (sizeof(D3D12_PIPELINE_STATE_SUBOBJECT_TYPE) + ial - 1) / ial * ial;
      if (type == D3D12_PIPELINE_STATE_SUBOBJECT_TYPE_BLEND && off + ioff + isz <= size) {
        result = blend_desc_blends(*reinterpret_cast<const D3D12_BLEND_DESC*>(p + off + ioff)) ? 1 : 2;  // also sets the pack
      }
      off += (ioff + isz + sizeof(void*) - 1) / sizeof(void*) * sizeof(void*);
    }
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return 0;
  }
  return result;
}

using CreateGraphicsPsoFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, const D3D12_GRAPHICS_PIPELINE_STATE_DESC*, REFIID, void**);
using CreateStreamPsoFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device2*, const D3D12_PIPELINE_STATE_STREAM_DESC*, REFIID, void**);
using CreateLibraryFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device1*, const void*, SIZE_T, REFIID, void**);
using LibLoadGraphicsFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12PipelineLibrary*, LPCWSTR, const D3D12_GRAPHICS_PIPELINE_STATE_DESC*, REFIID, void**);
using LibLoadStreamFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12PipelineLibrary1*, LPCWSTR, const D3D12_PIPELINE_STATE_STREAM_DESC*, REFIID, void**);
CreateGraphicsPsoFn g_create_gpso = nullptr;
CreateStreamPsoFn g_create_spso = nullptr;
CreateLibraryFn g_create_library = nullptr;
LibLoadGraphicsFn g_lib_load_g = nullptr;
LibLoadStreamFn g_lib_load_s = nullptr;

// Scanner-only evidence capture. No PSO, texture, descriptor or projection writes.
// Keep shader bytes while their creation arguments are live; draw-time inspection
// cannot recover bytecode from an ID3D12PipelineState.
std::atomic<bool> g_scan_trace{false};
bool g_fx_trace=false; // startup-only, observational effects capture
struct ScanShader {
  std::vector<unsigned char> vs, ps;
  const void* root = nullptr;  // identity only; never dereferenced
  DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
  unsigned seen = 0;
};
std::mutex g_scan_shader_mutex;
std::unordered_map<const void*, ScanShader> g_scan_shaders;
size_t g_scan_shader_bytes = 0;
void scan_shader_keep(ID3D12PipelineState* pso, const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d) {
  if (!g_scan_trace.load(std::memory_order_relaxed) || !pso || !d.PS.pShaderBytecode ||
      d.PS.BytecodeLength == 0 || d.PS.BytecodeLength > 1024 * 1024 || d.VS.BytecodeLength > 1024 * 1024) return;
  // The live scanner composite was excluded by the old RGBA8/RG8 filter.
  // Full-screen HDR effects need their original shader captured as well.
  if (d.NumRenderTargets != 1) return;
  // Capture both the depth-tested highlight producer and its screen composite.
  std::lock_guard<std::mutex> lock(g_scan_shader_mutex);
  const size_t bytes = d.VS.BytecodeLength + d.PS.BytecodeLength;
  if (g_scan_shaders.size() >= 4096 || g_scan_shader_bytes + bytes > 32 * 1024 * 1024) return;
  auto& s = g_scan_shaders[pso];
  g_scan_shader_bytes -= s.vs.size() + s.ps.size();
  s = {};
  s.root = d.pRootSignature;
  s.format = d.RTVFormats[0];
  const auto* ps = static_cast<const unsigned char*>(d.PS.pShaderBytecode);
  s.ps.assign(ps, ps + d.PS.BytecodeLength);
  if (d.VS.pShaderBytecode && d.VS.BytecodeLength) {
    const auto* vs = static_cast<const unsigned char*>(d.VS.pShaderBytecode);
    s.vs.assign(vs, vs + d.VS.BytecodeLength);
  }
  g_scan_shader_bytes += s.vs.size() + s.ps.size();
}
void scan_binding_trace(ID3D12GraphicsCommandList* list, UINT count, bool indexed);
void scan_inputs_draw(ID3D12GraphicsCommandList* list,UINT count,UINT instances,UINT start,INT base,UINT first_instance);
void scan_uv_keep(ID3D12PipelineState*, const D3D12_GRAPHICS_PIPELINE_STATE_DESC&);
ID3D12PipelineState* scan_uv_clone(ID3D12GraphicsCommandList*, ID3D12PipelineState*,int);
void logf(const char* fmt, ...);
// PS identity for the HUD draw census: part of the DXBC container checksum, stable across runs unlike the PSO pointer.
std::unordered_map<ID3D12PipelineState*,std::uint32_t> g_pso_ps_id;
void keep_picture_pso(ID3D12PipelineState* pso,const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d) {
  if(pso&&d.PS.pShaderBytecode&&d.PS.BytecodeLength>=20) {
    std::uint32_t id=0;
    std::memcpy(&id,static_cast<const char*>(d.PS.pShaderBytecode)+4,sizeof(id));
    std::lock_guard<std::mutex> lock(g_picture_mutex);
    g_pso_ps_id[pso]=id;
  }
  if(!pso||!screen_fx::radial_picture(d.PS.pShaderBytecode,d.PS.BytecodeLength)) return;
  std::lock_guard<std::mutex> lock(g_picture_mutex);
  if(g_picture_count>=32||pso_set_has(g_picture_psos,pso)) return;
  pso->AddRef(); // prevent stale pointer classification after engine cache eviction
  pso_set_add(g_picture_psos,pso);++g_picture_count;
  logf("fxpicture: retained captured HDR radial-blur PSO %p",pso);
}

ID3D12PipelineState* pso_out(REFIID riid, void** out) {
  if (out == nullptr || *out == nullptr) {
    return nullptr;
  }
  // The returned interface is an ID3D12PipelineState (or derived); command lists see the same pointer when the IID is
  // ID3D12PipelineState, which is what engines ask for.
  return riid == __uuidof(ID3D12PipelineState) ? static_cast<ID3D12PipelineState*>(*out) : nullptr;
}

// Read-only PSO depth census (2026-10-15, FP arm depth): graphics PSOs with a depth buffer and colour targets, by depth
// test state. Arms that always win could use depth test off or an always-pass comparison.
std::atomic<unsigned> g_pso_depth_off{0}, g_pso_depth_always{0}, g_pso_depth_normal{0}, g_pso_depth_nowrite{0};
// FP arm depth occlusion (2026-10-15). [LIVE] the draws consuming the FP arm camera block include a depth pre-pass draw
// (DSV, 0 RTs, eye size) with depth write + func ALWAYS, while scene pre-pass draws use GREATER_EQUAL and the G-buffer
// pass tests EQUAL without writing: the arms overwrite nearer scene depth and then win the EQUAL colour pass. Graphics
// PSOs with depth write + ALWAYS + DSV + no RTs are kept as deep copies so a GREATER_EQUAL clone can be made on demand
// and bound for FP arm draws only (menu "Arm depth occlusion" / "fpocclude").
struct StoredPso {
  D3D12_GRAPHICS_PIPELINE_STATE_DESC desc{};
  std::vector<unsigned char> code[5];  // VS PS DS HS GS
  std::vector<D3D12_INPUT_ELEMENT_DESC> elements;
  std::vector<std::string> names;
  ID3D12PipelineState* clone = nullptr;
  bool failed = false;
  bool mesh = false;  // has a vertex input layout (skinned/mesh draw, not a full-screen depth copy)
};
std::unordered_map<const void*, StoredPso*> g_always_psos;
std::mutex g_always_mutex;
std::atomic<unsigned> g_occlude_swaps{0}, g_occlude_clones{0}, g_occlude_failures{0};
void store_always_pso(ID3D12PipelineState* pso, const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d) {
  if (pso == nullptr || !d.DepthStencilState.DepthEnable || d.DepthStencilState.DepthWriteMask != D3D12_DEPTH_WRITE_MASK_ALL ||
      d.DepthStencilState.DepthFunc != D3D12_COMPARISON_FUNC_ALWAYS || d.DSVFormat == DXGI_FORMAT_UNKNOWN || d.NumRenderTargets != 0 ||
      d.StreamOutput.NumEntries != 0) return;
  auto* s = new StoredPso;
  s->desc = d;
  const D3D12_SHADER_BYTECODE* bc[5] = {&d.VS, &d.PS, &d.DS, &d.HS, &d.GS};
  D3D12_SHADER_BYTECODE* out[5] = {&s->desc.VS, &s->desc.PS, &s->desc.DS, &s->desc.HS, &s->desc.GS};
  for (int i = 0; i < 5; ++i) {
    if (bc[i]->pShaderBytecode != nullptr && bc[i]->BytecodeLength > 0) {
      const auto* p = static_cast<const unsigned char*>(bc[i]->pShaderBytecode);
      s->code[i].assign(p, p + bc[i]->BytecodeLength);
      out[i]->pShaderBytecode = s->code[i].data();
    } else {
      *out[i] = {};
    }
  }
  s->names.reserve(d.InputLayout.NumElements);
  for (UINT i = 0; i < d.InputLayout.NumElements; ++i) s->names.emplace_back(d.InputLayout.pInputElementDescs[i].SemanticName);
  for (UINT i = 0; i < d.InputLayout.NumElements; ++i) {
    D3D12_INPUT_ELEMENT_DESC e = d.InputLayout.pInputElementDescs[i];
    e.SemanticName = s->names[i].c_str();
    s->elements.push_back(e);
  }
  s->desc.InputLayout.pInputElementDescs = s->elements.empty() ? nullptr : s->elements.data();
  s->mesh = !s->elements.empty();
  s->desc.CachedPSO = {};
  if (s->desc.pRootSignature != nullptr) s->desc.pRootSignature->AddRef();
  std::lock_guard<std::mutex> lock(g_always_mutex);
  if (g_always_psos.size() >= 512 || g_always_psos.count(pso)) {
    if (s->desc.pRootSignature != nullptr) s->desc.pRootSignature->Release();
    delete s;
    return;
  }
  g_always_psos[pso] = s;
}

// Wrist HUD alpha (2026-10-30). [LIVE] the replayed HUD copy has RGB but alpha 0 everywhere (maxAlpha=0), so the flat
// originals could never be hidden. Blended single-target RGBA8 PSOs are kept so the replay alone can use a clone that also
// accumulates coverage: "over" alpha, or none for additive art (premultiplied alpha 0 stays additive on the panel).
std::unordered_map<const void*, StoredPso*> g_hud_alpha_psos;
std::mutex g_hud_alpha_mutex;
std::atomic<unsigned> g_hud_alpha_clones{0}, g_hud_alpha_failures{0};
void store_hud_alpha_pso(ID3D12PipelineState* pso, const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d) {
  if (pso == nullptr || d.NumRenderTargets != 1 || d.DSVFormat != DXGI_FORMAT_UNKNOWN || d.StreamOutput.NumEntries != 0 ||
      !d.BlendState.RenderTarget[0].BlendEnable || d.InputLayout.NumElements > 64 ||
      (d.RTVFormats[0] != DXGI_FORMAT_R8G8B8A8_UNORM && d.RTVFormats[0] != DXGI_FORMAT_R8G8B8A8_TYPELESS)) return;
  auto* s = new StoredPso;
  s->desc = d;
  s->desc.CachedPSO = {};
  const D3D12_SHADER_BYTECODE* bc[5] = {&d.VS, &d.PS, &d.DS, &d.HS, &d.GS};
  D3D12_SHADER_BYTECODE* out[5] = {&s->desc.VS, &s->desc.PS, &s->desc.DS, &s->desc.HS, &s->desc.GS};
  for (int i = 0; i < 5; ++i) {
    *out[i] = {};
    if (bc[i]->pShaderBytecode != nullptr && bc[i]->BytecodeLength > 0) {
      const auto* p = static_cast<const unsigned char*>(bc[i]->pShaderBytecode);
      s->code[i].assign(p, p + bc[i]->BytecodeLength);
      *out[i] = {s->code[i].data(), s->code[i].size()};
    }
  }
  s->names.reserve(d.InputLayout.NumElements);
  for (UINT i = 0; i < d.InputLayout.NumElements; ++i) s->names.emplace_back(d.InputLayout.pInputElementDescs[i].SemanticName);
  for (UINT i = 0; i < d.InputLayout.NumElements; ++i) {
    D3D12_INPUT_ELEMENT_DESC e = d.InputLayout.pInputElementDescs[i];
    e.SemanticName = s->names[i].c_str();
    s->elements.push_back(e);
  }
  s->desc.InputLayout.pInputElementDescs = s->elements.empty() ? nullptr : s->elements.data();
  if (s->desc.pRootSignature != nullptr) s->desc.pRootSignature->AddRef();
  std::lock_guard<std::mutex> lock(g_hud_alpha_mutex);
  if (g_hud_alpha_psos.size() >= 512 || g_hud_alpha_psos.count(pso)) {
    if (s->desc.pRootSignature != nullptr) s->desc.pRootSignature->Release();
    delete s;
    return;
  }
  g_hud_alpha_psos[pso] = s;
}
// Replay PSO for this draw, or nullptr (replay with the game's own). Per-thread cache like occlusion_clone.
ID3D12PipelineState* hud_alpha_clone(ID3D12GraphicsCommandList* list, ID3D12PipelineState* pso) {
  thread_local std::unordered_map<const void*, ID3D12PipelineState*> cache;
  const auto cached = cache.find(pso);
  if (cached != cache.end()) return cached->second;
  ID3D12PipelineState* result = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_hud_alpha_mutex);
    const auto it = g_hud_alpha_psos.find(pso);
    if (it != g_hud_alpha_psos.end()) {
      StoredPso* s = it->second;
      if (s->clone == nullptr && !s->failed) {
        ID3D12Device* dev = nullptr;
        if (SUCCEEDED(list->GetDevice(IID_PPV_ARGS(&dev))) && dev != nullptr) {
          D3D12_GRAPHICS_PIPELINE_STATE_DESC d = s->desc;
          D3D12_RENDER_TARGET_BLEND_DESC& r = d.BlendState.RenderTarget[0];
          const bool additive = r.DestBlend == D3D12_BLEND_ONE;
          logf("hud alpha: PSO %p game blend src=%d dst=%d alpha src=%d dst=%d mask=%u%s", pso, r.SrcBlend, r.DestBlend, r.SrcBlendAlpha,
               r.DestBlendAlpha, r.RenderTargetWriteMask, additive ? " additive" : "");
          r.SrcBlendAlpha = additive ? D3D12_BLEND_ZERO : D3D12_BLEND_ONE;
          r.DestBlendAlpha = additive ? D3D12_BLEND_ONE : D3D12_BLEND_INV_SRC_ALPHA;
          r.BlendOpAlpha = D3D12_BLEND_OP_ADD;
          r.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
          void* made = nullptr;
          const HRESULT hr = g_create_gpso(dev, &d, __uuidof(ID3D12PipelineState), &made);
          dev->Release();
          if (SUCCEEDED(hr) && made != nullptr) {
            s->clone = static_cast<ID3D12PipelineState*>(made);
            g_hud_alpha_clones.fetch_add(1, std::memory_order_relaxed);
          } else {
            s->failed = true;
            g_hud_alpha_failures.fetch_add(1, std::memory_order_relaxed);
            logf("hud alpha: clone failed hr=0x%08lX", static_cast<unsigned long>(hr));
          }
        }
      }
      result = s->clone;
    }
  }
  if (cache.size() < 4096) cache[pso] = result;
  return result;
}

std::atomic<bool> g_scan_uv_on{false};
bool g_scan_plane_wide=false; // startup-only, before device hooks are installed
bool g_scan_stable=false;
bool g_scan_screen=false;
float g_scan_plane_width=1.25f;
std::atomic<unsigned> g_scan_uv_draws{0};
std::mutex g_scan_uv_mutex;
struct ScanUvPso {
  ID3D12PipelineState* original = nullptr; // retained: key cannot be recycled
  StoredPso saved;
  struct Variant { float uv[10]; ID3D12PipelineState* pso; };
  std::vector<Variant> variants; // bounded; never release GPU-referenced PSOs mid-frame
};
std::unordered_map<ID3D12PipelineState*,ScanUvPso*> g_scan_uv_psos;
void scan_uv_keep(ID3D12PipelineState* pso, const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d) {
  if ((!g_scan_uv_on.load(std::memory_order_relaxed)&&!scanner_inputs::enabled.load(std::memory_order_relaxed)) || !pso || !scan_uv::matches(d.PS.pShaderBytecode,d.PS.BytecodeLength) ||
      d.StreamOutput.NumEntries || d.InputLayout.NumElements > 64) return;
  std::lock_guard<std::mutex> lock(g_scan_uv_mutex);
  if (g_scan_uv_psos.size()>=16 || g_scan_uv_psos.count(pso)) return;
  auto* s = new ScanUvPso;
  s->original=pso;pso->AddRef();s->saved.desc=d;s->saved.desc.CachedPSO={};
  if (d.pRootSignature) d.pRootSignature->AddRef();
  const D3D12_SHADER_BYTECODE* src[]={&d.VS,&d.PS,&d.DS,&d.HS,&d.GS};
  D3D12_SHADER_BYTECODE* dst[]={&s->saved.desc.VS,&s->saved.desc.PS,&s->saved.desc.DS,&s->saved.desc.HS,&s->saved.desc.GS};
  for(int i=0;i<5;++i) {
    if(src[i]->pShaderBytecode&&src[i]->BytecodeLength) {
      const auto* b=static_cast<const unsigned char*>(src[i]->pShaderBytecode);
      s->saved.code[i].assign(b,b+src[i]->BytecodeLength);
      *dst[i]={s->saved.code[i].data(),s->saved.code[i].size()};
    }
  }
  s->saved.names.reserve(d.InputLayout.NumElements);
  for(UINT i=0;i<d.InputLayout.NumElements;++i) s->saved.names.emplace_back(d.InputLayout.pInputElementDescs[i].SemanticName);
  for(UINT i=0;i<d.InputLayout.NumElements;++i) {
    auto e=d.InputLayout.pInputElementDescs[i];e.SemanticName=s->saved.names[i].c_str();s->saved.elements.push_back(e);
  }
  s->saved.desc.InputLayout.pInputElementDescs=s->saved.elements.empty()?nullptr:s->saved.elements.data();
  g_scan_uv_psos[pso]=s;
}
ID3D12PipelineState* scan_uv_clone(ID3D12GraphicsCommandList* list, ID3D12PipelineState* original,int eye) {
  if(!g_scan_uv_on.load(std::memory_order_relaxed)) return nullptr;
  float ox,oy,sx,sy;
  const float w=static_cast<float>(g_backbuffer_w.load()),h=static_cast<float>(g_backbuffer_h.load());
  if(!native_stereo::scan_offsets(w,h,&ox,&oy,&sx,&sy)||sx<0.1f||sy<0.1f) return nullptr;
  float uv[10]={1.f,1.f,0.f,0.f,0.f,0.f,0.f,0.f,w,h};
  if(eye==2) {uv[0]=1.f/sx;uv[1]=1.f/sy;uv[2]=-ox/w/sx;uv[3]=-oy/h/sy;}
  if(g_scan_plane_wide&&!native_stereo::scan_plane_uv(eye,g_scan_plane_width,uv+4)) return nullptr;
  // Suppress float noise in reported eye tangents (<0.025 px at this eye size)
  // rather than compiling variants for numerically equivalent projections.
  for(int i=0;i<8;++i) uv[i]=std::round(uv[i]*100000.f)/100000.f;
  std::lock_guard<std::mutex> lock(g_scan_uv_mutex);
  const auto found=g_scan_uv_psos.find(original);
  if(found==g_scan_uv_psos.end()) return nullptr;
  auto* s=found->second;
  for(const auto& v:s->variants) if(std::memcmp(v.uv,uv,sizeof(uv))==0) return v.pso;
  if(s->variants.size()>=8) return nullptr;
  auto bytes=scan_uv::patch(s->saved.code[1].data(),s->saved.code[1].size(),uv,g_scan_plane_wide?uv+4:nullptr,g_scan_stable&&g_scan_plane_wide,g_scan_screen?uv+8:nullptr);
  if(bytes.empty()) return nullptr;
  ID3D12Device* dev=nullptr;
  if(FAILED(list->GetDevice(IID_PPV_ARGS(&dev)))) return nullptr;
  auto desc=s->saved.desc;desc.PS={bytes.data(),bytes.size()};
  ID3D12PipelineState* made=nullptr;
  const HRESULT hr=g_create_gpso(dev,&desc,IID_PPV_ARGS(&made));dev->Release();
  ScanUvPso::Variant variant{};std::memcpy(variant.uv,uv,sizeof(uv));variant.pso=made;
  s->variants.push_back(variant); // also remember failure to avoid per-frame retry
  logf("scanstereo: clone hr=%08lX eye=%d original=%p clone=%p mask=(%.6f %.6f %.6f %.6f) plane=(%.6f %.6f %.6f %.6f)",
       static_cast<unsigned long>(hr),eye,original,made,uv[0],uv[1],uv[2],uv[3],uv[4],uv[5],uv[6],uv[7]);
  return made;
}

// Depth state per PSO (graphics desc only): bit0 depth enable, bit1 depth write, bits2-5 comparison func, bit6 has DSV.
std::unordered_map<const void*, unsigned char> g_pso_depth_state;
std::mutex g_pso_depth_mutex;
void note_pso_depth(const D3D12_GRAPHICS_PIPELINE_STATE_DESC& d) {
  if (d.DSVFormat == DXGI_FORMAT_UNKNOWN || d.NumRenderTargets == 0) return;
  if (!d.DepthStencilState.DepthEnable) g_pso_depth_off.fetch_add(1, std::memory_order_relaxed);
  else if (d.DepthStencilState.DepthFunc == D3D12_COMPARISON_FUNC_ALWAYS) g_pso_depth_always.fetch_add(1, std::memory_order_relaxed);
  else g_pso_depth_normal.fetch_add(1, std::memory_order_relaxed);
  if (d.DepthStencilState.DepthEnable && d.DepthStencilState.DepthWriteMask == D3D12_DEPTH_WRITE_MASK_ZERO)
    g_pso_depth_nowrite.fetch_add(1, std::memory_order_relaxed);
}

HRESULT STDMETHODCALLTYPE CreateGraphicsPso_hook(ID3D12Device* dev, const D3D12_GRAPHICS_PIPELINE_STATE_DESC* desc, REFIID riid, void** out) {
  const std::uint64_t pso_t0 = hitch_probe::enabled() ? hitch_probe::now_us() : 0;
  const HRESULT hr = g_create_gpso(dev, desc, riid, out);
  if (pso_t0 != 0) hitch_probe::note(hitch_probe::kGraphicsPso, hitch_probe::now_us() - pso_t0);
  if (SUCCEEDED(hr) && desc != nullptr) {
    g_pso_seen[0].fetch_add(1, std::memory_order_relaxed);
    note_pso_depth(*desc);
    if (ID3D12PipelineState* made = pso_out(riid, out)) {
      scan_shader_keep(made, *desc);
      keep_picture_pso(made,*desc);
      scan_uv_keep(made, *desc);
      store_always_pso(made, *desc);
      store_hud_alpha_pso(made, *desc);
      const unsigned char st = static_cast<unsigned char>((desc->DepthStencilState.DepthEnable ? 1 : 0) |
          (desc->DepthStencilState.DepthWriteMask == D3D12_DEPTH_WRITE_MASK_ALL ? 2 : 0) |
          ((static_cast<unsigned>(desc->DepthStencilState.DepthFunc) & 15u) << 2) | (desc->DSVFormat != DXGI_FORMAT_UNKNOWN ? 64 : 0));
      std::lock_guard<std::mutex> lock(g_pso_depth_mutex);
      g_pso_depth_state[made] = st;
    }
    blend_note(pso_out(riid, out), blend_desc_blends(desc->BlendState));
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE CreateStreamPso_hook(ID3D12Device2* dev, const D3D12_PIPELINE_STATE_STREAM_DESC* desc, REFIID riid, void** out) {
  const HRESULT hr = g_create_spso(dev, desc, riid, out);
  if (SUCCEEDED(hr) && desc != nullptr) {
    g_pso_seen[1].fetch_add(1, std::memory_order_relaxed);
    const int b = stream_blend(desc->pPipelineStateSubobjectStream, desc->SizeInBytes);
    if (b != 0) {
      blend_note(pso_out(riid, out), b == 1);
    }
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE LibLoadGraphics_hook(ID3D12PipelineLibrary* lib, LPCWSTR name, const D3D12_GRAPHICS_PIPELINE_STATE_DESC* desc, REFIID riid,
                                               void** out) {
  const HRESULT hr = g_lib_load_g(lib, name, desc, riid, out);
  if (SUCCEEDED(hr) && desc != nullptr) {
    g_pso_seen[2].fetch_add(1, std::memory_order_relaxed);
    scan_shader_keep(pso_out(riid, out), *desc);
    keep_picture_pso(pso_out(riid,out),*desc);
    scan_uv_keep(pso_out(riid, out), *desc);
    store_hud_alpha_pso(pso_out(riid, out), *desc);
    blend_note(pso_out(riid, out), blend_desc_blends(desc->BlendState));
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE LibLoadStream_hook(ID3D12PipelineLibrary1* lib, LPCWSTR name, const D3D12_PIPELINE_STATE_STREAM_DESC* desc, REFIID riid,
                                             void** out) {
  const HRESULT hr = g_lib_load_s(lib, name, desc, riid, out);
  if (SUCCEEDED(hr) && desc != nullptr) {
    g_pso_seen[3].fetch_add(1, std::memory_order_relaxed);
    const int b = stream_blend(desc->pPipelineStateSubobjectStream, desc->SizeInBytes);
    if (b != 0) {
      blend_note(pso_out(riid, out), b == 1);
    }
  }
  return hr;
}

bool hook_target(void* target, void* detour, void** original, const char* name);
void* vtable_slot(void* object, int index);

HRESULT STDMETHODCALLTYPE CreateLibrary_hook(ID3D12Device1* dev, const void* blob, SIZE_T size, REFIID riid, void** out) {
  const HRESULT hr = g_create_library(dev, blob, size, riid, out);
  if (SUCCEEDED(hr) && out != nullptr && *out != nullptr) {
    auto* unk = static_cast<IUnknown*>(*out);
    ID3D12PipelineLibrary* lib = nullptr;
    if (SUCCEEDED(unk->QueryInterface(IID_PPV_ARGS(&lib)))) {
      hook_target(vtable_slot(lib, 9), reinterpret_cast<void*>(&LibLoadGraphics_hook), reinterpret_cast<void**>(&g_lib_load_g),
                  "ID3D12PipelineLibrary::LoadGraphicsPipeline");
      lib->Release();
    }
    ID3D12PipelineLibrary1* lib1 = nullptr;
    if (SUCCEEDED(unk->QueryInterface(IID_PPV_ARGS(&lib1)))) {
      hook_target(vtable_slot(lib1, 13), reinterpret_cast<void*>(&LibLoadStream_hook), reinterpret_cast<void**>(&g_lib_load_s),
                  "ID3D12PipelineLibrary1::LoadPipeline");
      lib1->Release();
    }
  }
  return hr;
}

// Scan start: revoke what was learned in the last 2 s (the scan's first frames, before its signal).
void ui_pso_revoke_recent() {
  const ULONGLONG now = GetTickCount64();
  for (int i = 0; i < kPsoSlots; ++i) {
    if (g_ui_psos[i].load(std::memory_order_relaxed) != nullptr && !g_ui_pso_revoked[i].load(std::memory_order_relaxed) &&
        now - g_ui_pso_time[i].load(std::memory_order_relaxed) < 2000) {
      g_ui_pso_revoked[i].store(true, std::memory_order_relaxed);
      g_ui_pso_revokes.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void ui_pso_learn(ID3D12PipelineState* p) {
  if (p == nullptr || g_ui_pso_count.load(std::memory_order_relaxed) >= kPsoSlots / 2) {
    return;
  }
  for (size_t i = pso_slot(p), n = 0; n < kPsoSlots; ++n, i = (i + 1) % kPsoSlots) {
    ID3D12PipelineState* v = g_ui_psos[i].load(std::memory_order_relaxed);
    if (v == p) {
      return;
    }
    if (v == nullptr) {
      if (g_ui_psos[i].compare_exchange_strong(v, p)) {
        g_ui_pso_time[i].store(GetTickCount64(), std::memory_order_relaxed);
        g_ui_pso_count.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      if (v == p) {
        return;
      }
    }
  }
}

// Main menu layer (native_stereo::ui_layer_active): the main view's UI pass draws into this texture instead of the
// eye image; xr_runtime copies it into an OpenXR quad layer fixed in the room. RGBA8 like the backbuffer and the second
// view's output (both fmt 28, logs 2026-10-08), simultaneous-access so no barriers are needed in the game's list (it
// is promoted to RENDER_TARGET there and to COPY_SOURCE in ours, and decays to COMMON between submissions). Cleared to
// transparent at the first bind in each pass, so the layer holds only the UI, with the blend's alpha.
std::mutex g_layer_mutex;
ID3D12Resource* g_layer = nullptr;
ID3D12DescriptorHeap* g_layer_heap = nullptr;
D3D12_CPU_DESCRIPTOR_HANDLE g_layer_rtv = {};
UINT g_layer_w = 0, g_layer_h = 0;
std::atomic<ULONGLONG> g_layer_written{0};
std::atomic<std::uint64_t> g_layer_passes{0};
thread_local bool t_layer_cleared = false;
thread_local SIZE_T t_first_rt = 0;  // first RTV the pass bound (to count passes that bind other targets)
thread_local ID3D12GraphicsCommandList* t_layer_list = nullptr;  // list whose target was redirected; restored at pass end
// Panel coverage (2026-11-12; "nopanelcover" in [native] = off). LIVE panel capture 2026-11-12 13:59 (F9): the layer held the
// whole menu in RGB but alpha 0 everywhere (opaque 0.0%, empty 100.0%): the game's UI blends leave alpha untouched, so the
// headset quad added the menu on top of the world (AUTHOR: "opacity very low"; pictures and dark panels see-through). While
// the UI pass draws into the layer, each PSO is replaced by the wrist HUD's alpha-accumulating clone (hud_alpha_clone:
// "over" alpha for normal blends, unchanged for additive art), and the game's PSO is bound back at the pass end.
std::atomic<bool> g_panel_cover{true};
std::atomic<std::uint64_t> g_panel_cover_binds{0};
thread_local bool t_layer_clone = false;                      // a clone is bound on t_layer_list
thread_local ID3D12GraphicsCommandList* t_pso_list = nullptr;  // the list t_pso was last set on
thread_local D3D12_CPU_DESCRIPTOR_HANDLE t_orig_rt = {};
thread_local D3D12_CPU_DESCRIPTOR_HANDLE t_orig_depth = {};
thread_local bool t_orig_has_depth = false;

void logf(const char* fmt, ...);  // defined below

// Rect probe (2026-10-08, read-only). [AUTHOR] + capture 06:53: frosted rectangles (blurred scene) show in the left eye
// only, where the UI's panels sit in the unplaced layout, in the main menu and in gameplay. The UI pass records no
// compute (uiprobe dispatch=0), so they are drawn by other work that sets its own viewport / scissor to the panel
// rectangles. This table keeps the sub-target rectangles set OUTSIDE any UI pass (not at the origin, inside the render
// target), with how often and from which return address (the caller of the D3D12 method; a game or wrapper address).
struct RectHit {
  LONG l, t, r, b;
  char kind;  // 'v' viewport, 's' scissor
  std::uint64_t count;
  void* caller;
  DWORD thread;
};
constexpr int kRectSlots = 48;
RectHit g_rects[kRectSlots] = {};
int g_rect_count = 0;
std::uint64_t g_rect_dropped = 0;
std::mutex g_rect_mutex;

void note_rect(char kind, float l, float t, float r, float b, void* caller) {
  const float w = static_cast<float>(g_backbuffer_w.load(std::memory_order_relaxed));
  const float h = static_cast<float>(g_backbuffer_h.load(std::memory_order_relaxed));
  if (w <= 0.f || (l <= 0.5f && t <= 0.5f) || r - l < 8.f || b - t < 8.f || r > w + 0.5f || b > h + 0.5f ||
      (r - l > w - 2.f && b - t > h - 2.f)) {
    return;  // at the origin, tiny, outside the target, or the full target
  }
  std::unique_lock<std::mutex> lock(g_rect_mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  const LONG il = static_cast<LONG>(l), it = static_cast<LONG>(t), ir = static_cast<LONG>(r), ib = static_cast<LONG>(b);
  for (int i = 0; i < g_rect_count; ++i) {
    RectHit& e = g_rects[i];
    if (e.kind == kind && e.l == il && e.t == it && e.r == ir && e.b == ib && e.caller == caller) {
      ++e.count;
      return;
    }
  }
  if (g_rect_count < kRectSlots) {
    g_rects[g_rect_count++] = {il, it, ir, ib, kind, 1, caller, GetCurrentThreadId()};
  } else {
    ++g_rect_dropped;
  }
}

bool ensure_layer(ID3D12GraphicsCommandList* list, UINT w, UINT h) {
  std::lock_guard<std::mutex> lock(g_layer_mutex);
  if (g_layer != nullptr && g_layer_w == w && g_layer_h == h) {
    return true;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(list->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) {
    return false;
  }
  if (g_layer_heap == nullptr) {
    D3D12_DESCRIPTOR_HEAP_DESC hd = {};
    hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    hd.NumDescriptors = 1;
    if (FAILED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_layer_heap)))) {
      g_layer_heap = nullptr;
      device->Release();
      return false;
    }
    g_layer_rtv = g_layer_heap->GetCPUDescriptorHandleForHeapStart();
  }
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC rd = {};
  rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  rd.Width = w;
  rd.Height = h;
  rd.DepthOrArraySize = 1;
  rd.MipLevels = 1;
  rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  rd.SampleDesc.Count = 1;
  rd.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
  rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS;
  ID3D12Resource* tex = nullptr;
  if (FAILED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                             IID_PPV_ARGS(&tex)))) {
    device->Release();
    logf("ui layer: texture %ux%u creation failed", w, h);
    return false;
  }
  tex->SetName(L"AvatarVR_UiLayer");
  device->CreateRenderTargetView(tex, nullptr, g_layer_rtv);
  device->Release();
  // A previous texture may still be in flight on the GPU; it is left alive (a resize is rare).
  g_layer = tex;
  g_layer_w = w;
  g_layer_h = h;
  logf("ui layer: texture %ux%u RGBA8 simultaneous-access", w, h);
  return true;
}

bool ui_target(float* w, float* h) {
  if (t_layer_list != nullptr) {
    *w = t_panel_target_w;
    *h = t_panel_target_h;
    return true;
  }
  *w = static_cast<float>(g_backbuffer_w.load(std::memory_order_relaxed));
  *h = static_cast<float>(g_backbuffer_h.load(std::memory_order_relaxed));
  return *w > 0.f && *h > 0.f;
}

// ---- Wrist HUD texture (2026-10-18, FEATURE-13 Phase 3, research/HUD_PIECES.md) -------------------------------
// The game decides which HUD pieces are visible while it builds the HUD (Gfx_MovablePieceID settings), not while it
// draws, so a piece cannot be in one target and not in another by flipping its flag. Instead every placed draw of the
// main view's HUD pass is issued a second time into this texture, with the game's logical canvas fitted to it; the
// eye images are untouched. xr_runtime crops ammo / health out of it onto the wrist. Like the menu layer: RGBA8,
// simultaneous-access, cleared to transparent at the first copy of each pass. Only while wrist_hud is on.
std::mutex g_hud_mutex;
ID3D12Resource* g_hud = nullptr;
ID3D12DescriptorHeap* g_hud_heap = nullptr;
D3D12_CPU_DESCRIPTOR_HANDLE g_hud_rtv = {};
std::atomic<ULONGLONG> g_hud_written{0};
std::atomic<std::uint64_t> g_hud_copies{0}, g_hud_band_cuts{0};
std::atomic<std::uint64_t> g_hud_masks{0};
constexpr UINT kHudW = ui_canvas::panel_width, kHudH = ui_canvas::panel_height;
thread_local wrist_hud::Settings t_hud = {};
thread_local bool t_hud_pass = false;     // this UI pass copies into the HUD texture
thread_local bool t_hud_band = false;     // this UI pass cuts the bottom band off the in-view HUD
thread_local bool t_hud_cleared = false;
// What the game bound last on this thread (to put it back after a copy draw).
thread_local ID3D12GraphicsCommandList* t_rt_list = nullptr;
thread_local D3D12_CPU_DESCRIPTOR_HANDLE t_rts[8] = {};
thread_local UINT t_rt_count = 0;
thread_local D3D12_CPU_DESCRIPTOR_HANDLE t_rt_depth = {};
thread_local bool t_rt_has_depth = false;

bool ensure_hud(ID3D12GraphicsCommandList* list) {
  std::lock_guard<std::mutex> lock(g_hud_mutex);
  if (g_hud != nullptr) {
    return true;
  }
  static bool failed = false;
  if (failed) {
    return false;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(list->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) {
    return false;
  }
  D3D12_DESCRIPTOR_HEAP_DESC hd = {};
  hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
  hd.NumDescriptors = 1;
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC rd = {};
  rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  rd.Width = kHudW;
  rd.Height = kHudH;
  rd.DepthOrArraySize = 1;
  rd.MipLevels = 1;
  rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  rd.SampleDesc.Count = 1;
  rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS;
  ID3D12Resource* tex = nullptr;
  if (FAILED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_hud_heap))) ||
      FAILED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                             IID_PPV_ARGS(&tex)))) {
    device->Release();
    failed = true;
    logf("wrist hud: texture %ux%u creation failed; wrist HUD disabled", kHudW, kHudH);
    return false;
  }
  tex->SetName(L"AvatarVR_WristHud");
  g_hud_rtv = g_hud_heap->GetCPUDescriptorHandleForHeapStart();
  device->CreateRenderTargetView(tex, nullptr, g_hud_rtv);
  device->Release();
  g_hud = tex;
  logf("wrist hud: texture %ux%u RGBA8 simultaneous-access", kHudW, kHudH);
  return true;
}

// The game's logical HUD canvas inside the eye raster (same rule as native_stereo::ui_map): a window-sized canvas in
// the lower-left of an independent eye image, else the whole target.
bool hud_canvas(float w, float h, float* top, float* cw, float* ch) {
  float lw = 0.f, lh = 0.f;
  std::uint32_t ew = 0, eh = 0;
  const bool logical = native_stereo::eye_size(&ew, &eh) && native_stereo::ui_canvas_size(&lw, &lh) && lw >= 64.f &&
                       lh >= 64.f && lw <= w + 0.5f && lh <= h + 0.5f;
  *cw = logical ? lw : w;
  *ch = logical ? lh : h;
  *top = logical ? h - lh : 0.f;
  return *cw > 0.f && *ch > 0.f;
}

// The game's viewport / scissor (eye raster, unmapped) -> HUD texture.
bool hud_map(float w, float h, float* ox, float* oy, float* s) {
  float top = 0.f, cw = 0.f, ch = 0.f;
  if (!hud_canvas(w, h, &top, &cw, &ch)) {
    return false;
  }
  const auto fit = ui_canvas::place_rect(0.f, top, cw, ch, static_cast<float>(kHudW), static_cast<float>(kHudH),
                                         kHudW * 0.5f, kHudH * 0.5f, 1.f);
  *ox = fit.x;
  *oy = fit.y;
  *s = fit.scale;
  return true;
}

// Bottom of the in-view HUD kept (eye raster, unmapped, at viewport scale k): everything below is cut.
bool hud_band_bottom(float w, float h, float k, LONG* bottom) {
  float top = 0.f, cw = 0.f, ch = 0.f;
  if (!t_hud_band || scan_active() || !hud_canvas(w, h, &top, &cw, &ch)) {
    return false;
  }
  *bottom = static_cast<LONG>((top + ch * (1.f - t_hud.band)) * k);
  return true;
}

D3D12_RECT band_cut(D3D12_RECT r, float w, float h, float k) {
  LONG bottom = 0;
  if (hud_band_bottom(w, h, k, &bottom) && r.bottom > bottom) {
    r.bottom = bottom > r.top ? bottom : r.top;
    g_hud_band_cuts.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

D3D12_VIEWPORT map_viewport(const D3D12_VIEWPORT& v, float ox, float oy, float sx, float sy) {
  D3D12_VIEWPORT m = v;
  m.TopLeftX = ox + v.TopLeftX * sx;
  m.TopLeftY = oy + v.TopLeftY * sy;
  m.Width = v.Width * sx;
  m.Height = v.Height * sy;
  return m;
}

D3D12_RECT map_rect(const D3D12_RECT& r, float ox, float oy, float sx, float sy, float w, float h);

// Full-screen draws inside a placed UI pass keep the game's own viewport. [AUTHOR] + capture 2026-10-08 07:2x: with the
// scan ability (Na'vi senses) the whole picture shrank by the HUD size in both eyes, with black right/bottom borders
// and blocky garbage: the effect is drawn inside the UI pass as a full-screen draw [INFERENCE], and the placement
// scaled it and broke its pixel-addressed reads. A full-screen draw is taken as DrawInstanced with <= 4 vertices or
// DrawIndexedInstanced with 3 indices, one instance (the usual full-screen triangle / quad; UI quads use 6 indices,
// uiprobe idx3 / idx6 / idxother count them). The game's viewport and scissor are put back for that draw and the
// placed ones re-applied after it.
template <typename Draw>
void draw_unplaced(ID3D12GraphicsCommandList* list, int eye, Draw draw);
D3D12_RECT map_rect(const D3D12_RECT& r, float ox, float oy, float sx, float sy, float w, float h) {
  auto clampf = [](float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; };
  D3D12_RECT m;
  m.left = static_cast<LONG>(clampf(std::floor(ox + static_cast<float>(r.left) * sx), 0.f, w));
  m.top = static_cast<LONG>(clampf(std::floor(oy + static_cast<float>(r.top) * sy), 0.f, h));
  m.right = static_cast<LONG>(clampf(std::ceil(ox + static_cast<float>(r.right) * sx), 0.f, w));
  m.bottom = static_cast<LONG>(clampf(std::ceil(oy + static_cast<float>(r.bottom) * sy), 0.f, h));
  if (m.right < m.left) {
    m.right = m.left;
  }
  if (m.bottom < m.top) {
    m.bottom = m.top;
  }
  return m;
}

std::mutex g_log_mutex;
std::mutex g_hook_mutex;
std::mutex g_sample_mutex;
std::wstring g_log_path;
void* g_hooked[64] = {};
int g_hooked_count = 0;

std::atomic<std::uint64_t> g_cbv_count{0};

struct ProjectionHit {
  bool seen = false;
  float fov_y = 0;
  float near_z = 0;
  float far_z = 0;
  bool near_ok = false;
  bool far_ok = false;
  bool column_major = false;
  UINT root = 0;
};

struct ViewHit {
  bool seen = false;
  float x = 0;
  float y = 0;
  float z = 0;
  float pitch = 0;
  float yaw = 0;
  float roll = 0;
  bool column_major = false;
  UINT root = 0;
};

struct ViewportHit {
  bool seen = false;
  float width = 0;
  float height = 0;
  float min_depth = 0;
  float max_depth = 0;
};

ProjectionHit g_projection;
ViewHit g_view;
ViewportHit g_viewport;

std::wstring module_path(HMODULE module) {
  std::wstring buf(MAX_PATH, L'\0');
  for (;;) {
    const DWORD wrote = GetModuleFileNameW(module, buf.data(), static_cast<DWORD>(buf.size()));
    if (wrote == 0) {
      return {};
    }
    if (wrote < buf.size() - 1) {
      buf.resize(wrote);
      return buf;
    }
    buf.resize(buf.size() * 2);
  }
}

std::wstring parent_dir(const std::wstring& path) {
  const auto slash = path.find_last_of(L"\\/");
  if (slash == std::wstring::npos) {
    return {};
  }
  return path.substr(0, slash);
}

void log_line(const std::string& text) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty()) {
    return;
  }
  runtime_log::append(g_log_path, {text, "\r\n"});
}

void logf(const char* fmt, ...) {
  char buffer[900];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  log_line(buffer);
}

void* vtable_slot(void* object, int index) {
  if (object == nullptr) {
    return nullptr;
  }
  auto** table = *reinterpret_cast<void***>(object);
  if (table == nullptr) {
    return nullptr;
  }
  return table[index];
}

bool hook_target(void* target, void* detour, void** original, const char* name) {
  if (target == nullptr || detour == nullptr || original == nullptr) {
    logf("hook skip %s null", name);
    return false;
  }
  bool already = false;
  MH_STATUS created = MH_OK;
  MH_STATUS enabled = MH_OK;
  {
    std::lock_guard<std::mutex> lock(g_hook_mutex);
    for (int i = 0; i < g_hooked_count; ++i) {
      if (g_hooked[i] == target) {
        already = true;
        break;
      }
    }
    if (!already && g_hooked_count >= static_cast<int>(sizeof(g_hooked) / sizeof(g_hooked[0]))) {
      logf("hook failed %s: registry full (%d); hook NOT installed", name, g_hooked_count);
      return false;
    }
    if (!already) {
      created = MH_CreateHook(target, detour, original);
      if (created == MH_OK) {
        enabled = MH_EnableHook(target);
        if (enabled == MH_OK) {
          g_hooked[g_hooked_count++] = target;
        }
      }
    }
  }
  if (already) {
    return true;
  }
  if (created != MH_OK || enabled != MH_OK) {
    logf("hook failed %s create=%d enable=%d target=%p", name, static_cast<int>(created), static_cast<int>(enabled), target);
    return false;
  }
  logf("hook enabled %s target=%p thread=%lu", name, target, GetCurrentThreadId());
  return true;
}

bool finite4(float a, float b, float c, float d) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(d);
}

float at(const float* m, int row, int col, bool column_major) {
  return column_major ? m[col * 4 + row] : m[row * 4 + col];
}

bool perspective(const float* m, bool column_major, float* fov_y, float* near_z, float* far_z, bool* near_ok, bool* far_ok) {
  const float x_scale = at(m, 0, 0, column_major);
  const float y_scale = at(m, 1, 1, column_major);
  const float m22 = at(m, 2, 2, column_major);
  const float m23 = at(m, 2, 3, column_major);
  const float m32 = at(m, 3, 2, column_major);
  const float m33 = at(m, 3, 3, column_major);
  if (!finite4(x_scale, y_scale, m22, m23) || !std::isfinite(m32) || !std::isfinite(m33)) {
    return false;
  }
  if (std::fabs(std::fabs(m23) - 1.f) > 0.02f || std::fabs(m33) > 0.02f) {
    return false;
  }
  if (y_scale < 0.05f || y_scale > 20.f || x_scale < 0.05f || x_scale > 20.f) {
    return false;
  }
  *fov_y = 2.f * std::atan(1.f / y_scale) * (180.f / 3.14159265f);
  *near_ok = false;
  *far_ok = false;
  if (std::fabs(m22 - 1.f) < 0.001f) {
    const float near_candidate = -m32;
    if (near_candidate > 0.f && near_candidate < 100.f) {
      *near_z = near_candidate;
      *near_ok = true;
    }
    return true;
  }
  if (std::fabs(m22) < 1e-4f) {
    return true;
  }
  const float near_candidate = -m32 / m22;
  if (near_candidate > 1e-4f && near_candidate < 1000.f) {
    *near_z = near_candidate;
    *near_ok = true;
    const float far_candidate = (m22 * near_candidate) / (m22 - 1.f);
    if (far_candidate > near_candidate && far_candidate < 1.0e7f) {
      *far_z = far_candidate;
      *far_ok = true;
    }
  }
  return true;
}

bool unit_axis(float x, float y, float z) {
  const float length = x * x + y * y + z * z;
  return length > 0.90f && length < 1.10f;
}

bool axes_orthogonal(float ax, float ay, float az, float bx, float by, float bz) {
  return std::fabs(ax * bx + ay * by + az * bz) < 0.08f;
}

bool rigid_view(const float* m, bool column_major, float* x, float* y, float* z, float* pitch, float* yaw, float* roll) {
  const float r00 = at(m, 0, 0, column_major);
  const float r01 = at(m, 0, 1, column_major);
  const float r02 = at(m, 0, 2, column_major);
  const float r10 = at(m, 1, 0, column_major);
  const float r11 = at(m, 1, 1, column_major);
  const float r12 = at(m, 1, 2, column_major);
  const float r20 = at(m, 2, 0, column_major);
  const float r21 = at(m, 2, 1, column_major);
  const float r22 = at(m, 2, 2, column_major);
  if (!unit_axis(r00, r01, r02) || !unit_axis(r10, r11, r12) || !unit_axis(r20, r21, r22)) {
    return false;
  }
  if (!axes_orthogonal(r00, r01, r02, r10, r11, r12) || !axes_orthogonal(r00, r01, r02, r20, r21, r22) || !axes_orthogonal(r10, r11, r12, r20, r21, r22)) {
    return false;
  }
  *x = at(m, 3, 0, column_major);
  *y = at(m, 3, 1, column_major);
  *z = at(m, 3, 2, column_major);
  if (!finite4(*x, *y, *z, r21)) {
    return false;
  }
  const float forward_y = std::fmax(-1.f, std::fmin(1.f, r21));
  *pitch = std::asin(forward_y) * (180.f / 3.14159265f);
  *yaw = std::atan2(r20, r22) * (180.f / 3.14159265f);
  *roll = std::atan2(r01, r11) * (180.f / 3.14159265f);
  return true;
}

void consider_constants(UINT root, const float* src, UINT count) {
  ProjectionHit projection = {};
  ViewHit view = {};
  bool found_projection = false;
  bool found_view = false;
  for (UINT offset = 0; offset + 16 <= count; offset += 4) {
    const float* window = src + offset;
    float fov = 0;
    float near_z = 0;
    float far_z = 0;
    bool near_ok = false;
    bool far_ok = false;
    if (!found_projection && perspective(window, false, &fov, &near_z, &far_z, &near_ok, &far_ok)) {
      projection.seen = true;
      projection.fov_y = fov;
      projection.near_z = near_z;
      projection.far_z = far_z;
      projection.near_ok = near_ok;
      projection.far_ok = far_ok;
      projection.column_major = false;
      projection.root = root;
      found_projection = true;
    } else if (!found_projection && perspective(window, true, &fov, &near_z, &far_z, &near_ok, &far_ok)) {
      projection.seen = true;
      projection.fov_y = fov;
      projection.near_z = near_z;
      projection.far_z = far_z;
      projection.near_ok = near_ok;
      projection.far_ok = far_ok;
      projection.column_major = true;
      projection.root = root;
      found_projection = true;
    }
    float x = 0;
    float y = 0;
    float z = 0;
    float pitch = 0;
    float yaw = 0;
    float roll = 0;
    if (!found_view && rigid_view(window, false, &x, &y, &z, &pitch, &yaw, &roll)) {
      view = {true, x, y, z, pitch, yaw, roll, false, root};
      found_view = true;
    } else if (!found_view && rigid_view(window, true, &x, &y, &z, &pitch, &yaw, &roll)) {
      view = {true, x, y, z, pitch, yaw, roll, true, root};
      found_view = true;
    }
  }
  if (!found_projection && !found_view) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_sample_mutex);
  if (found_projection) {
    g_projection = projection;
  }
  if (found_view) {
    g_view = view;
  }
}

// Read-only depth-range census (2026-10-15, FP arm depth investigation): first-person arms often win the depth test via
// a viewport with a narrowed MinDepth/MaxDepth slice. Distinct non-0..1 ranges (with size, count) are logged every 10 s.
struct DepthRange {float min_depth, max_depth, width, height; unsigned long long count;};
DepthRange g_depth_ranges[32];
int g_depth_range_count = 0;
std::mutex g_depth_mutex;
std::atomic<unsigned long long> g_viewport_calls{0};
void note_depth_range(const D3D12_VIEWPORT& v) {
  g_viewport_calls.fetch_add(1, std::memory_order_relaxed);
  if (v.MinDepth == 0.f && v.MaxDepth == 1.f) return;
  std::lock_guard<std::mutex> lock(g_depth_mutex);
  for (int i = 0; i < g_depth_range_count; ++i) {
    DepthRange& d = g_depth_ranges[i];
    if (d.min_depth == v.MinDepth && d.max_depth == v.MaxDepth && d.width == v.Width && d.height == v.Height) {
      ++d.count;
      return;
    }
  }
  if (g_depth_range_count < 32) g_depth_ranges[g_depth_range_count++] = {v.MinDepth, v.MaxDepth, v.Width, v.Height, 1};
}
// Read-only depth-clear census (2026-10-15, FP arm depth): a world-then-clear-then-arms frame clears a depth buffer
// mid-frame. Distinct (depth value, flags, partial) combinations with counts and distinct DSV handles per period.
using ClearDsvFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_CLEAR_FLAGS, FLOAT, UINT8,
                                            UINT, const D3D12_RECT*);
ClearDsvFn g_clear_dsv = nullptr;
struct ClearKind {float depth; unsigned flags; bool partial; unsigned long long count;};
ClearKind g_clear_kinds[16];
int g_clear_kind_count = 0;
SIZE_T g_clear_dsvs[64];
int g_clear_dsv_count = 0;
std::atomic<unsigned long long> g_clear_calls{0};
thread_local ID3D12GraphicsCommandList* t_clear_list = nullptr;  // depth buffers cleared in the list recorded on this thread
thread_local SIZE_T t_cleared[8] = {};
thread_local int t_cleared_n = 0;
thread_local SIZE_T t_bound_dsv = 0;
thread_local ID3D12GraphicsCommandList* t_probe_vp_list = nullptr;  // FP depth probe: last viewport on this thread (any path)
thread_local float t_probe_vw = 0.f, t_probe_vh = 0.f;
thread_local UINT t_bound_rts = 0;
// FP arm draw samples: distinct (dsv, pso depth state, cleared in this list) with counts.
struct FpDraw {SIZE_T dsv; int state; bool cleared; UINT rts; float vw, vh; unsigned long long count;};
FpDraw g_fp_draws[16];
int g_fp_draw_count = 0;
bool note_fp_draw(ID3D12GraphicsCommandList* list) {
  if (!native_stereo::fp_draw_window()) return false;
  int state = -1;  // unknown (stream PSO or none)
  {
    std::lock_guard<std::mutex> lock(g_pso_depth_mutex);
    auto it = g_pso_depth_state.find(t_pso);
    if (it != g_pso_depth_state.end()) state = it->second;
  }
  bool cleared = false;
  if (t_clear_list == list) for (int i = 0; i < t_cleared_n; ++i) cleared = cleared || t_cleared[i] == t_bound_dsv;
  const float vw = t_probe_vp_list == list ? t_probe_vw : 0.f, vh = t_probe_vp_list == list ? t_probe_vh : 0.f;
  std::lock_guard<std::mutex> lock(g_depth_mutex);
  for (int i = 0; i < g_fp_draw_count; ++i) {
    FpDraw& d = g_fp_draws[i];
    if (d.dsv == t_bound_dsv && d.state == state && d.cleared == cleared && d.rts == t_bound_rts && d.vw == vw && d.vh == vh) {++d.count; return true;}
  }
  if (g_fp_draw_count < 16) g_fp_draws[g_fp_draw_count++] = {t_bound_dsv, state, cleared, t_bound_rts, vw, vh, 1};
  return true;
}
// Arm PSOs learned from FP-window draws (2026-10-15: one PSO, swapped only right after a new FP block, so frames where
// the block was reused or the draw order differed slipped through: "the hand flickers in front"). Every later draw with
// a learned PSO at eye resolution is swapped too.
std::atomic<const void*> g_arm_psos[8] = {};
bool learned_arm_pso(const void* pso) {
  for (const auto& p : g_arm_psos) if (p.load(std::memory_order_relaxed) == pso) return true;
  return false;
}
void learn_arm_pso(const void* pso) {
  if (learned_arm_pso(pso)) return;
  for (auto& p : g_arm_psos) {
    const void* empty = nullptr;
    if (p.compare_exchange_strong(empty, pso)) {
      logf("arm occlusion: learned arm depth PSO %p", pso);
      return;
    }
  }
}
// Returns the GREATER_EQUAL clone to bind for this FP arm draw, or nullptr (draw unchanged).
// 2026-10-15: arm skin, bracer and bracelets draw with their own ALWAYS PSOs, not only the hand's. Every mesh PSO
// (vertex input) with depth write + ALWAYS in the eye-resolution depth pre-pass is swapped; full-screen ALWAYS passes
// (depth copies, no vertex input) are left alone. Per-thread cache: the global lock is taken once per PSO per thread.
ID3D12PipelineState* clone_for(ID3D12GraphicsCommandList* list, const void* pso, bool fp_draw) {
  std::lock_guard<std::mutex> lock(g_always_mutex);
  auto it = g_always_psos.find(pso);
  if (it == g_always_psos.end()) return nullptr;
  StoredPso* s = it->second;
  if (!s->mesh && !fp_draw && !learned_arm_pso(pso)) return nullptr;
  if (s->clone == nullptr && !s->failed) {
    ID3D12Device* dev = nullptr;
    if (SUCCEEDED(list->GetDevice(IID_PPV_ARGS(&dev))) && dev != nullptr) {
      D3D12_GRAPHICS_PIPELINE_STATE_DESC d = s->desc;
      d.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_GREATER_EQUAL;  // the scene pre-pass's reverse-Z test
      void* made = nullptr;
      const HRESULT hr = g_create_gpso(dev, &d, __uuidof(ID3D12PipelineState), &made);
      dev->Release();
      if (SUCCEEDED(hr) && made != nullptr) {
        s->clone = static_cast<ID3D12PipelineState*>(made);
        g_occlude_clones.fetch_add(1, std::memory_order_relaxed);
        logf("arm occlusion: clone %u for ALWAYS depth PSO %p (mesh=%d, fp_window=%d)", g_occlude_clones.load(), pso, s->mesh ? 1 : 0,
             fp_draw ? 1 : 0);
      } else {
        s->failed = true;
        g_occlude_failures.fetch_add(1, std::memory_order_relaxed);
        logf("arm occlusion: clone PSO creation failed hr=0x%08lX", static_cast<unsigned long>(hr));
      }
    }
  }
  return s->clone;
}
// Returns the GREATER_EQUAL clone to bind for this draw, or nullptr (draw unchanged).
std::atomic<unsigned> g_occlude_rider_skips{0};
ID3D12PipelineState* occlusion_clone(ID3D12GraphicsCommandList* list, bool fp_draw) {
  if (t_pso == nullptr || !native_stereo::fp_occlude_enabled()) return nullptr;
  // 2026-11-08 [HEADSET] with this clone live (swaps 91k-214k), riding showed the arm in the game's harness pose while its gear followed
  // the controllers; it was written and checked on foot only. Riding keeps the game's arm depth (draw unchanged); on foot unchanged.
  if (mount_camera::rider_view()) {g_occlude_rider_skips.fetch_add(1, std::memory_order_relaxed); return nullptr;}
  // Eye-resolution depth pre-pass only: no colour targets, eye-sized viewport and not a power-of-two square (the same
  // ALWAYS arm PSO also draws into 512x512 shadow-like targets, whose depth convention is not verified).
  if (t_bound_rts != 0 || t_probe_vp_list != list) return nullptr;
  const unsigned w = static_cast<unsigned>(t_probe_vw), h = static_cast<unsigned>(t_probe_vh);
  if (w == h && (w & (w - 1)) == 0) return nullptr;
  // 2026-11-08 [LIVE]: the fixed ">= 1200 px" floor (written at 1600x1600 eyes) rejected every arm draw once the upscaler
  // rendered the 2320x2464 eye at 1183x1257 (CAMERA_OBSERVE: swaps=0). In stereo the viewport must instead match the
  // eye's aspect at >= 25% of its width (DLSS/XeSS internal scales); flat keeps the old floor.
  std::uint32_t ew = 0, eh = 0;
  if (native_stereo::eye_size(&ew, &eh)) {
    const float eye_aspect = static_cast<float>(ew) / static_cast<float>(eh), vp_aspect = t_probe_vw / t_probe_vh;
    if (std::fabs(vp_aspect - eye_aspect) > 0.02f * eye_aspect || w * 4 < ew || w > ew + 1 || h > eh + 1) return nullptr;
  } else if (t_probe_vw < 1200.f || t_probe_vh < 1200.f) {
    return nullptr;
  }
  if (fp_draw) learn_arm_pso(t_pso);
  thread_local std::unordered_map<const void*, ID3D12PipelineState*> cache;
  auto it = cache.find(t_pso);
  if (it != cache.end()) return it->second;
  ID3D12PipelineState* clone = clone_for(list, t_pso, fp_draw);
  if (cache.size() < 4096) cache[t_pso] = clone;
  return clone;
}

void STDMETHODCALLTYPE ClearDsv_hook(ID3D12GraphicsCommandList* list, D3D12_CPU_DESCRIPTOR_HANDLE dsv, D3D12_CLEAR_FLAGS flags, FLOAT depth,
                                     UINT8 stencil, UINT rects, const D3D12_RECT* rect) {
  g_clear_calls.fetch_add(1, std::memory_order_relaxed);
  if (t_clear_list != list) {t_clear_list = list; t_cleared_n = 0;}
  if (t_cleared_n < 8) t_cleared[t_cleared_n++] = dsv.ptr;
  {
    std::lock_guard<std::mutex> lock(g_depth_mutex);
    bool found = false;
    for (int i = 0; i < g_clear_kind_count; ++i) {
      ClearKind& k = g_clear_kinds[i];
      if (k.depth == depth && k.flags == static_cast<unsigned>(flags) && k.partial == (rects > 0)) {
        ++k.count;
        found = true;
        break;
      }
    }
    if (!found && g_clear_kind_count < 16) g_clear_kinds[g_clear_kind_count++] = {depth, static_cast<unsigned>(flags), rects > 0, 1};
    bool known = false;
    for (int i = 0; i < g_clear_dsv_count; ++i) known = known || g_clear_dsvs[i] == dsv.ptr;
    if (!known && g_clear_dsv_count < 64) g_clear_dsvs[g_clear_dsv_count++] = dsv.ptr;
  }
  g_clear_dsv(list, dsv, flags, depth, stencil, rects, rect);
}

void log_depth_ranges() {
  static ULONGLONG next = 0;
  const ULONGLONG now = GetTickCount64();
  if (now < next) return;
  next = now + 10000;
  char line[1024];
  int n = snprintf(line, sizeof(line), "depth ranges: viewports=%llu non-0..1:", g_viewport_calls.load());
  {
    std::lock_guard<std::mutex> lock(g_depth_mutex);
    for (int i = 0; i < g_depth_range_count && n > 0 && n < static_cast<int>(sizeof(line)) - 64; ++i) {
      const DepthRange& d = g_depth_ranges[i];
      n += snprintf(line + n, sizeof(line) - n, " [z%.4f-%.4f %.0fx%.0f x%llu]", d.min_depth, d.max_depth, d.width, d.height, d.count);
    }
  }
  if (g_depth_range_count == 0 && n > 0) snprintf(line + n, sizeof(line) - n, " none");
  logf("%s", line);
  // Depth clears since the last line (counts reset each period) and the PSO depth-state totals.
  static unsigned long long last_clears = 0, last_presents = 0;
  const unsigned long long clears = g_clear_calls.load();
  n = snprintf(line, sizeof(line), "depth clears: %llu in period, distinct dsv %d:", clears - last_clears, g_clear_dsv_count);
  last_clears = clears;
  {
    std::lock_guard<std::mutex> lock(g_depth_mutex);
    for (int i = 0; i < g_clear_kind_count && n > 0 && n < static_cast<int>(sizeof(line)) - 64; ++i) {
      ClearKind& k = g_clear_kinds[i];
      n += snprintf(line + n, sizeof(line) - n, " [depth %.3f flags %u %s x%llu]", k.depth, k.flags, k.partial ? "rects" : "full", k.count);
      k.count = 0;
    }
    g_clear_dsv_count = 0;
  }
  logf("%s", line);
  (void)last_presents;
  {
    char fp[1400];
    int m = snprintf(fp, sizeof(fp), "fp draws (first draw after an FP arm camera block on the same thread; func 8 = ALWAYS):");
    std::lock_guard<std::mutex> lock(g_depth_mutex);
    for (int i = 0; i < g_fp_draw_count && m > 0 && m < static_cast<int>(sizeof(fp)) - 120; ++i) {
      const FpDraw& d = g_fp_draws[i];
      if (d.state < 0) {
        m += snprintf(fp + m, sizeof(fp) - m, " [dsv %llx rts %u vp %.0fx%.0f pso unknown cleared_in_list %d x%llu]",
                      static_cast<unsigned long long>(d.dsv), d.rts, d.vw, d.vh, d.cleared ? 1 : 0, d.count);
      } else {
        m += snprintf(fp + m, sizeof(fp) - m, " [dsv %llx rts %u vp %.0fx%.0f depth %s write %d func %d psodsv %d cleared_in_list %d x%llu]",
                      static_cast<unsigned long long>(d.dsv), d.rts, d.vw, d.vh, (d.state & 1) ? "on" : "OFF", (d.state >> 1) & 1, (d.state >> 2) & 15,
                      (d.state >> 6) & 1, d.cleared ? 1 : 0, d.count);
      }
    }
    if (g_fp_draw_count == 0 && m > 0) snprintf(fp + m, sizeof(fp) - m, " none");
    g_fp_draw_count = 0;
    logf("%s", fp);
  }
  logf("arm occlusion: %s swaps=%u clones=%u failures=%u riding_skips=%u stored_always_psos=%u", native_stereo::fp_occlude_enabled() ? "ON" : "off",
       g_occlude_swaps.load(), g_occlude_clones.load(), g_occlude_failures.load(), g_occlude_rider_skips.load(),
       [] { std::lock_guard<std::mutex> lock(g_always_mutex); return static_cast<unsigned>(g_always_psos.size()); }());
  logf("pso depth: with DSV+RT normal=%u depth_off=%u always=%u (no-write %u)", g_pso_depth_normal.load(), g_pso_depth_off.load(),
       g_pso_depth_always.load(), g_pso_depth_nowrite.load());
}

void STDMETHODCALLTYPE Viewports_hook(ID3D12GraphicsCommandList* list, UINT count, const D3D12_VIEWPORT* viewports) {
  if (viewports != nullptr) for (UINT i = 0; i < count && i < 4; ++i) note_depth_range(viewports[i]);
  if (viewports != nullptr && count > 0) {t_probe_vp_list = list; t_probe_vw = viewports[0].Width; t_probe_vh = viewports[0].Height;}
  const int eye = native_stereo::ui_pass_eye();
  if (eye != 0) {
    dx12::mark_list(list, 'U', eye);  // GPU profiler: which submission holds this eye's UI pass
  }
  if (eye == 0 && viewports != nullptr && count > 0) {
    note_rect('v', viewports[0].TopLeftX, viewports[0].TopLeftY, viewports[0].TopLeftX + viewports[0].Width,
              viewports[0].TopLeftY + viewports[0].Height, _ReturnAddress());
  }
  float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  const bool panel = t_layer_list == list && count == 1 && viewports != nullptr &&
      viewports[0].TopLeftX == 0.f && viewports[0].TopLeftY == 0.f &&
      viewports[0].Width >= 16.f && viewports[0].Height >= 16.f;
  if (panel) {
    t_panel_source_w = viewports[0].Width;
    t_panel_source_h = viewports[0].Height;
    thread_local float last_w = 0.f, last_h = 0.f;
    if (last_w != t_panel_source_w || last_h != t_panel_source_h) {
      last_w = t_panel_source_w;
      last_h = t_panel_source_h;
      logf("ui canvas: source %.0fx%.0f -> panel %.0fx%.0f, centred uniform fit", last_w, last_h,
           t_panel_target_w, t_panel_target_h);
    }
  }
  if (eye != 0 && viewports != nullptr && count > 0 && count <= D3D12_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE &&
      ui_target(&w, &h) && native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy,
          panel ? t_panel_source_w : 0.f, panel ? t_panel_source_h : 0.f) &&
      (panel || target_scale(viewports[0], w, h) != 0.f) && !(scan_active() && native_stereo::ui_pass_kind() == 1)) {
    // [AUTHOR] 2026-10-08: the UI and the frosted glass were aligned with the 07:18 build, which placed every viewport
    // of a tagged pass at full scale; only the scan broke (its chain at the upscaler-input size, 1172x1271, was placed
    // too). Later rules (full-target only, half-size scaling, PSO / full-screen rules gated by a scan signal) split the
    // UI, and the scan signal (08:26: a DSV bound in a tagged pass) fired all through gameplay [LIVE 08:33-08:34], so
    // they were always on. Now: 07:18 placement for viewports of the full or exactly half eye size, nothing else.
    const float k = panel ? 1.f : target_scale(viewports[0], w, h);
    D3D12_VIEWPORT mapped[D3D12_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
    for (UINT i = 0; i < count; ++i) {
      mapped[i] = map_viewport(viewports[i], ox * k, oy * k, sx, sy);
    }
    g_viewports(list, count, mapped);
    t_placed_list = list;
    t_mapped_now = true;
    t_odd_vp = false;
    t_map_scale = k;
    if (k < 1.f) {
      g_ui[eye].half.fetch_add(1, std::memory_order_relaxed);
    }
  } else {
    g_viewports(list, count, viewports);
    if (eye != 0) {
      t_mapped_now = false;
      t_odd_vp = viewports != nullptr && count > 0;
      if (t_odd_vp) {
        sig_count(g_sig_odd);
      }
    }
  }
  if (viewports != nullptr && count > 0) {
    t_vp_list = list;  // unmapped: what the game asked for
    t_vp = viewports[0];
    if (eye != 0) {
      const bool full = ui_target(&w, &h) && full_target(viewports[0], w, h);
      t_trace_odd = t_trace_odd || !full;
      unsigned cw = 0, ch = 0;
      if (dx12::client_size(&cw, &ch) && std::fabs(viewports[0].Width - static_cast<float>(cw)) <= 2.f &&
          std::fabs(viewports[0].Height - static_cast<float>(ch)) <= 2.f && w > static_cast<float>(cw) + 8.f) {
        t_trace_canvas = true;
      }
      trace_ev('V', static_cast<UINT>(viewports[0].Width), static_cast<UINT>(viewports[0].Height),
               static_cast<int>(viewports[0].TopLeftX), static_cast<int>(viewports[0].TopLeftY), t_mapped_now);
    }
  }
  if (eye != 0 && viewports != nullptr && count > 0) {
    g_ui[eye].viewports.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(g_ui_mutex);
    g_ui_list[eye] = list;
    bool known = false;
    for (int i = 0; i < g_ui_vp_count[eye]; ++i) {
      const D3D12_VIEWPORT& v = g_ui_vp[eye][i];
      known = known || (v.TopLeftX == viewports[0].TopLeftX && v.TopLeftY == viewports[0].TopLeftY &&
                        v.Width == viewports[0].Width && v.Height == viewports[0].Height);
    }
    if (!known && g_ui_vp_count[eye] < kUiShapes) {
      g_ui_vp[eye][g_ui_vp_count[eye]++] = viewports[0];
    }
  }
  if (!g_research || viewports == nullptr || count == 0) {
    return;
  }
  const UINT target_w = g_backbuffer_w.load();
  const UINT target_h = g_backbuffer_h.load();
  const D3D12_VIEWPORT* chosen = nullptr;
  for (UINT i = 0; i < count; ++i) {
    if (target_w == 0 || target_h == 0) {
      chosen = &viewports[0];
      break;
    }
    const float dw = std::fabs(viewports[i].Width - static_cast<float>(target_w));
    const float dh = std::fabs(viewports[i].Height - static_cast<float>(target_h));
    if (dw <= 2.f && dh <= 2.f) {
      chosen = &viewports[i];
      break;
    }
  }
  if (chosen == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_sample_mutex);
  g_viewport.seen = true;
  g_viewport.width = chosen->Width;
  g_viewport.height = chosen->Height;
  g_viewport.min_depth = chosen->MinDepth;
  g_viewport.max_depth = chosen->MaxDepth;
}

thread_local ID3D12GraphicsCommandList* t_inputs_list=nullptr;
using InputVerticesFn=void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*,UINT,UINT,const D3D12_VERTEX_BUFFER_VIEW*);
using InputIndicesFn=void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*,const D3D12_INDEX_BUFFER_VIEW*);
InputVerticesFn g_input_vertices=nullptr;
InputIndicesFn g_input_indices=nullptr;
void STDMETHODCALLTYPE InputVertices_hook(ID3D12GraphicsCommandList* list,UINT first,UINT count,const D3D12_VERTEX_BUFFER_VIEW* views) {
  g_input_vertices(list,first,count,views);
  scanner_geometry::vertices(list,first,count,views);
}
void STDMETHODCALLTYPE InputIndices_hook(ID3D12GraphicsCommandList* list,const D3D12_INDEX_BUFFER_VIEW* view) {
  g_input_indices(list,view);
  scanner_geometry::indices(list,view);
}
thread_local UINT64 t_input_va[32]={},t_input_valid[32]={};
thread_local UINT t_input_words[32][64]={};
void input_list(ID3D12GraphicsCommandList* list) {
  if(t_inputs_list!=list) {t_inputs_list=list;std::memset(t_input_va,0,sizeof(t_input_va));std::memset(t_input_valid,0,sizeof(t_input_valid));}
}
void STDMETHODCALLTYPE RootConstants_hook(ID3D12GraphicsCommandList* list, UINT root, UINT count, const void* src, UINT dest_offset) {
  g_root_constants(list, root, count, src, dest_offset);
  if(scanner_inputs::enabled.load(std::memory_order_relaxed)&&root<32&&src&&dest_offset<64) {
    input_list(list);
    const UINT n=count<64-dest_offset?count:64-dest_offset;
    std::memcpy(t_input_words[root]+dest_offset,src,n*4);
    for(UINT i=0;i<n;++i) t_input_valid[root]|=UINT64(1)<<(dest_offset+i);
  }
  (void)dest_offset;
  if (const int eye = native_stereo::ui_pass_eye()) {
    g_ui[eye].consts.fetch_add(1, std::memory_order_relaxed);
  }
  if (!g_research || src == nullptr || count < 16) {
    return;
  }
  consider_constants(root, static_cast<const float*>(src), count);
}

void STDMETHODCALLTYPE RootCbv_hook(ID3D12GraphicsCommandList* list, UINT root, D3D12_GPU_VIRTUAL_ADDRESS address) {
  g_root_cbv(list, root, address);
  if(scanner_inputs::enabled.load(std::memory_order_relaxed)&&root<32) {input_list(list);t_input_va[root]=address;}
  if (const int eye = native_stereo::ui_pass_eye()) {
    g_ui[eye].cbvs.fetch_add(1, std::memory_order_relaxed);
  }
  if (!g_research) {
    return;
  }
  (void)root;
  (void)address;
  g_cbv_count.fetch_add(1);
}

// Screen effects (2026-10-23, research/SCREEN_EFFECTS_20261023.md). [LIVE] uitrace 18:32-18:34: in gameplay the UI pass
// draws on the full eye raster and the game lays its UI out in a window-sized logical canvas (1280x720) in the raster's
// lower-left; ui_map fits that canvas to the eye. [HEADSET] the X-ray scan, mount boost and flash grenade draw the whole
// picture as one canvas-filling draw in that pass: unplaced it lands in the lower-left 1280x720 patch (black elsewhere),
// placed it becomes the 16:9 HUD box. "fxstretch": such draws map the canvas onto the WHOLE eye raster (non-uniform: they
// sample the picture by UV) -- the scan's unplaced draws, and opaque (non-blended) draws of the placed HUD pass.
std::atomic<bool> g_fx_stretch{false};
std::atomic<std::uint64_t> g_fx_opaque_draws{0}, g_fx_stretched_scan{0}, g_fx_stretched_opaque{0};
// [LIVE] 18:43-18:45 with fxstretch on: 0 opaque HUD-pass draws, 0 scan draws during the mount boost -> the boost is
// not drawn in the HUD pass. uitrace 18:33:51 (effect period): a neighbour pass (kind 1, half-size 1160x1232 target,
// placed) draws D3/1 -- a full-screen triangle, the usual whole-screen effect draw; placed like HUD it fills only the
// HUD box of a target cleared to black. Full-screen triangles (Draw 3 vertices, 1 instance) in a placed UI-tagged pass
// therefore get the plain full-target viewport with "fxstretch". Counted per pass kind.
std::atomic<std::uint64_t> g_fx_tri[2], g_fx_tri_full[2];  // [kind 0 HUD pass, kind 1 neighbour pass]
bool canvas_stretch(float w, float h, const D3D12_VIEWPORT& vp, float k, D3D12_VIEWPORT* out) {
  float cw = 0.f, ch = 0.f;
  if (!g_fx_stretch.load(std::memory_order_relaxed) || !native_stereo::ui_canvas_size(&cw, &ch) || w < cw + 8.f || h < ch + 8.f) {
    return false;
  }
  const float sx = w / cw, sy = h / ch;
  *out = map_viewport(vp, 0.f, -(h - ch) * sy * k, sx, sy);
  return true;
}

// A full-screen triangle on the whole bound target (not the placed HUD box), then the placed viewport / scissor back.
template <typename Draw>
void draw_full_target(ID3D12GraphicsCommandList* list, int eye, Draw draw) {
  float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  ui_target(&w, &h);
  const float k = t_map_scale;
  const D3D12_VIEWPORT full = {0.f, 0.f, w * k, h * k, 0.f, 1.f};
  const D3D12_RECT all = {0, 0, static_cast<LONG>(w * k), static_cast<LONG>(h * k)};
  g_viewports(list, 1, &full);
  g_scissors(list, 1, &all);
  draw();
  if (native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy)) {
    const D3D12_VIEWPORT mvp = map_viewport(t_vp_list == list ? t_vp : full, ox * k, oy * k, sx, sy);
    g_viewports(list, 1, &mvp);
    const D3D12_RECT sc = t_sc_list == list ? t_sc : all;
    const D3D12_RECT msc = map_rect(sc, ox * k, oy * k, sx, sy, w * k, h * k);
    g_scissors(list, 1, &msc);
  }
}

template <typename Draw>
void draw_unplaced(ID3D12GraphicsCommandList* list, int eye, Draw draw) {
  float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  ui_target(&w, &h);
  const float k = t_map_scale;
  const D3D12_VIEWPORT full = {0.f, 0.f, w * k, h * k, 0.f, 1.f};
  const D3D12_VIEWPORT vp = t_vp_list == list ? t_vp : full;  // what the game asked for (unmapped)
  const D3D12_RECT all = {0, 0, static_cast<LONG>(w * k), static_cast<LONG>(h * k)};
  const D3D12_RECT sc = t_sc_list == list ? t_sc : all;
  // Right eye during the scan: these draws carry the main view's (left-eye) highlight images, so they go through the
  // left-image -> right-image map instead of the game's viewport (native_stereo::scan_reproject).
  float rox = 0.f, roy = 0.f, rsx = 1.f, rsy = 1.f;
  if (eye == 2 && t_reproj_draw && native_stereo::scan_reproject(w, h, &rox, &roy, &rsx, &rsy)) {
    const D3D12_VIEWPORT rvp = map_viewport(vp, rox * k, roy * k, rsx, rsy);
    g_viewports(list, 1, &rvp);
    g_ui[2].reproj.fetch_add(1, std::memory_order_relaxed);
  } else {
    D3D12_VIEWPORT svp;
    if (canvas_stretch(w, h, vp, k, &svp)) {
      g_viewports(list, 1, &svp);
      g_fx_stretched_scan.fetch_add(1, std::memory_order_relaxed);
    } else {
      g_viewports(list, 1, &vp);
    }
  }
  if (t_placed_sc) {
    g_scissors(list, 1, &sc);
  }
  draw();
  if (native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy)) {
    const D3D12_VIEWPORT mvp = map_viewport(vp, ox * k, oy * k, sx, sy);
    g_viewports(list, 1, &mvp);
    if (t_placed_sc) {
      const D3D12_RECT msc = map_rect(sc, ox * k, oy * k, sx, sy, w * k, h * k);
      g_scissors(list, 1, &msc);
    }
  }
}

// Full-view effects layer (ui_ownership): the canvas on the scanner's head-forward plane, one surface for both eyes. Falls back to the
// whole eye when the plane is unknown.
template <typename Draw>
void draw_fx_plane(ID3D12GraphicsCommandList* list, int eye, Draw draw) {
  float w = 0.f, h = 0.f, rect[4];
  ui_target(&w, &h);
  const float k = t_map_scale;
  const D3D12_VIEWPORT full = {0.f, 0.f, w * k, h * k, 0.f, 1.f};
  const D3D12_VIEWPORT vp = t_vp_list == list ? t_vp : full;
  D3D12_VIEWPORT svp;
  if (!canvas_stretch(w, h, vp, k, &svp) || !native_stereo::fx_plane_rect(eye, ui_ownership::fx_plane_size(), rect)) {
    draw_unplaced(list, eye, draw);
    return;
  }
  const D3D12_VIEWPORT pvp = map_viewport(svp, rect[0] * w * k, rect[1] * h * k, rect[2] - rect[0], rect[3] - rect[1]);
  // Once every 2 s per eye: the numbers behind the plane, since the bottom edge sat higher than computed [HEADSET 2026-11-07].
  static std::atomic<ULONGLONG> logged[3]{};
  const ULONGLONG now = GetTickCount64();
  if (now - logged[eye].load(std::memory_order_relaxed) >= 2000) {
    logged[eye].store(now, std::memory_order_relaxed);
    float cw = 0.f, ch = 0.f; native_stereo::ui_canvas_size(&cw, &ch);
    logf("fxplane: eye=%d target=%.0fx%.0f k=%.2f canvas=%.0fx%.0f vp=%.0f,%.0f %.0fx%.0f stretched=%.0f,%.0f %.0fx%.0f rect=%.3f,%.3f,%.3f,%.3f plane=%.0f,%.0f %.0fx%.0f",
         eye, w, h, k, cw, ch, vp.TopLeftX, vp.TopLeftY, vp.Width, vp.Height, svp.TopLeftX, svp.TopLeftY, svp.Width, svp.Height,
         rect[0], rect[1], rect[2], rect[3], pvp.TopLeftX, pvp.TopLeftY, pvp.Width, pvp.Height);
  }
  const D3D12_RECT all = {0, 0, static_cast<LONG>(w * k), static_cast<LONG>(h * k)};
  g_viewports(list, 1, &pvp);
  g_scissors(list, 1, &all);
  draw();
  float ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  if (native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy)) {
    const D3D12_VIEWPORT mvp = map_viewport(vp, ox * k, oy * k, sx, sy);
    g_viewports(list, 1, &mvp);
    const D3D12_RECT sc = t_sc_list == list ? t_sc : all;
    const D3D12_RECT msc = map_rect(sc, ox * k, oy * k, sx, sy, w * k, h * k);
    g_scissors(list, 1, &msc);
  }
}

// Wrist HUD: issue a placed HUD draw of the main view's pass again into the HUD texture, then put the game's target
// and the placed viewport / scissor back. Same PSO, root signature and bindings; only the target and raster change.
template <class Draw>
void hud_copy(ID3D12GraphicsCommandList* list, Draw draw) {
  float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, s = 1.f, eox = 0.f, eoy = 0.f, esx = 1.f, esy = 1.f;
  if (!ui_target(&w, &h) || !hud_map(w, h, &ox, &oy, &s) || !native_stereo::ui_map(1, w, h, &eox, &eoy, &esx, &esy) ||
      !ensure_hud(list)) {
    t_hud_pass = false;
    return;
  }
  const D3D12_VIEWPORT full = {0.f, 0.f, w, h, 0.f, 1.f};
  const D3D12_VIEWPORT vp = t_vp_list == list ? t_vp : full;
  const D3D12_RECT all = {0, 0, static_cast<LONG>(w), static_cast<LONG>(h)};
  const D3D12_RECT sc = t_sc_list == list ? t_sc : all;
  g_render_targets(list, 1, &g_hud_rtv, FALSE, nullptr);
  if (!t_hud_cleared) {
    const float clear[4] = {0.f, 0.f, 0.f, 0.f};
    list->ClearRenderTargetView(g_hud_rtv, clear, 0, nullptr);
    t_hud_cleared = true;
  }
  const D3D12_VIEWPORT hvp = map_viewport(vp, ox, oy, s, s);
  const D3D12_RECT hsc = map_rect(sc, ox, oy, s, s, static_cast<float>(kHudW), static_cast<float>(kHudH));
  g_viewports(list, 1, &hvp);
  g_scissors(list, 1, &hsc);
  ID3D12PipelineState* const alpha_pso = t_pso != nullptr ? hud_alpha_clone(list, t_pso) : nullptr;
  if (alpha_pso != nullptr) g_set_pso(list, alpha_pso);
  draw();
  if (alpha_pso != nullptr) g_set_pso(list, t_pso);
  g_render_targets(list, t_rt_count, t_rts, FALSE, t_rt_has_depth ? &t_rt_depth : nullptr);
  const D3D12_VIEWPORT evp = map_viewport(vp, eox, eoy, esx, esy);
  g_viewports(list, 1, &evp);
  if (t_placed_sc) {
    const D3D12_RECT esc = map_rect(band_cut(sc, w, h, 1.f), eox, eoy, esx, esy, w, h);
    g_scissors(list, 1, &esc);
  } else {
    g_scissors(list, 1, &sc);
  }
  g_hud_copies.fetch_add(1, std::memory_order_relaxed);
}

ID3D12Resource* rtv_resource(SIZE_T h);
bool hud_copy_wanted(ID3D12GraphicsCommandList* list, int eye, bool placed) {
  if (!(t_hud_pass && eye == 1 && placed && !t_depth_bound && !t_rt_has_depth && t_map_scale == 1.f &&
        t_rt_list == list && t_rt_count == 1 && t_layer_list == nullptr)) return false;
  // Replay only full-sized RGBA8 draws: smaller intermediate targets and other formats need their own PSO.
  ID3D12Resource* target = rtv_resource(t_rts[0].ptr);
  if (!target) return false;
  const auto d = target->GetDesc();
  float w = 0.f, h = 0.f;
  return ui_target(&w, &h) && d.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE2D && d.SampleDesc.Count == 1 &&
      (d.Format == DXGI_FORMAT_R8G8B8A8_UNORM || d.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS) &&
      d.Width == static_cast<UINT64>(w) && d.Height == static_cast<UINT>(h);
}


// Native-crop replacement: the two source rectangles are omitted from the eye images, while the replay
// into the wrist texture remains complete. Retain the eye HUD until RGB/alpha and XR submission are established.
// This is raster masking, so unrelated artwork overlapping either rectangle is also masked.
template <class Draw>
void hud_eye_draw(ID3D12GraphicsCommandList* list, int eye, bool placed, Draw draw) {
  float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f, hx = 0.f, hy = 0.f, hs = 1.f;
  if (!placed || !t_hud.on || !t_hud.hide_original || !wrist_hud::replacing() || t_depth_bound ||
      t_map_scale != 1.f || t_rt_list != list || t_rt_count != 1 || t_layer_list ||
      !ui_target(&w, &h) || !native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy) ||
      !hud_map(w, h, &hx, &hy, &hs)) { draw(); return; }
  const D3D12_RECT source = t_sc_list == list ? t_sc : D3D12_RECT{0, 0, static_cast<LONG>(w), static_cast<LONG>(h)};
  const D3D12_RECT base = map_rect(source, ox, oy, sx, sy, w, h);
  D3D12_RECT pieces[16] = {base};
  unsigned count = 1;
  const wrist_hud::Crop crops[] = {t_hud.ammo, t_hud.health};
  for (const auto& c : crops) {
    const D3D12_RECT crop = {static_cast<LONG>(std::floor(c.x * kHudW)), static_cast<LONG>(std::floor(c.y * kHudH)),
        static_cast<LONG>(std::ceil((c.x + c.w) * kHudW)), static_cast<LONG>(std::ceil((c.y + c.h) * kHudH))};
    const auto hole = map_rect(crop, ox - hx * sx / hs, oy - hy * sy / hs, sx / hs, sy / hs, w, h);
    D3D12_RECT next[16] = {};
    unsigned n = 0;
    for (unsigned i = 0; i < count; ++i) {
      const auto r = pieces[i];
      const LONG l = r.left > hole.left ? r.left : hole.left, t = r.top > hole.top ? r.top : hole.top;
      const LONG rr = r.right < hole.right ? r.right : hole.right, b = r.bottom < hole.bottom ? r.bottom : hole.bottom;
      if (l >= rr || t >= b) { next[n++] = r; continue; }
      if (r.top < t) next[n++] = {r.left, r.top, r.right, t};
      if (b < r.bottom) next[n++] = {r.left, b, r.right, r.bottom};
      if (r.left < l) next[n++] = {r.left, t, l, b};
      if (rr < r.right) next[n++] = {rr, t, r.right, b};
    }
    count = n;
    std::memcpy(pieces, next, sizeof(pieces));
  }
  for (unsigned i = 0; i < count; ++i) {
    if (pieces[i].left >= pieces[i].right || pieces[i].top >= pieces[i].bottom) continue;
    g_scissors(list, 1, &pieces[i]);
    draw();
  }
  g_hud_masks.fetch_add(1, std::memory_order_relaxed);
  g_scissors(list, 1, &base);
  t_placed_sc = true;
}

// HUD draw census (2026-10-30, read-only, only during an ownership capture): left-eye draws per second of the placed
// HUD pass by pixel shader and vertex/index count. Comparing seconds with and without the damage vignette gives the
// shader to route, the way the boost's radial blur is routed. research/HUD_NATIVE_ATTACHMENT_20261028.md.
struct CensusSlot { std::atomic<std::uint64_t> key{0}; std::atomic<std::uint32_t> n{0}; };
CensusSlot g_census[256];
void census_note(ID3D12PipelineState* pso, UINT count) {
  if (!pso || !ui_ownership::capture_active()) return;
  const std::uint64_t key = (reinterpret_cast<std::uint64_t>(pso) << 16) | (count & 0xFFFFu);
  std::uint64_t slot = (key >> 20) ^ count;
  for (int probe = 0; probe < 16; ++probe, ++slot) {
    CensusSlot& s = g_census[slot % 256];
    std::uint64_t seen = s.key.load(std::memory_order_relaxed);
    if (seen == 0 && s.key.compare_exchange_strong(seen, key)) seen = key;
    if (seen == key) { s.n.fetch_add(1, std::memory_order_relaxed); return; }
  }
}
// Damage vignette shader test (2026-10-30, opt-in from F10, not saved): the selected candidate's HUD-pass draws go to
// the whole eye by the boost's route. The census cannot pick the shader: the three full-canvas widgets draw constantly.
constexpr std::uint32_t kHudFxCandidates[] = {0xe1dc8d42, 0x25baebbf, 0xca50dd2c, 0x720bde7d, 0x0fac3625,
                                              0xb42431a9, 0x1f388e44, 0x6ae25306, 0x3fcfced2};
constexpr unsigned kHudFxCount = sizeof(kHudFxCandidates) / sizeof(kHudFxCandidates[0]);
std::atomic<unsigned> g_hudfx_pick{0};  // 0 off, else candidate index + 1
std::atomic<ID3D12PipelineState*> g_hudfx_psos[kPsoSlots];
std::atomic<std::uint64_t> g_hudfx_draws{0};
char g_hudfx_label[64] = "HUD shader test: off";
// All nine candidates left the red boxed [AUTHOR], so the draw sweep stretches one draw of the HUD pass by its position
// in the pass and logs its shader and index count. An ordinal moves as the HUD changes: this finds the draw, it is not
// a shipping selector.
std::atomic<unsigned> g_hudfx_draw{0}, g_hudfx_draw_max{0};  // 0 off, else 1-based draw of each eye's HUD pass
std::atomic<bool> g_hudfx_sweeping{false};
std::atomic<std::uint64_t> g_hudfx_draw_key{0};
thread_local unsigned t_hudfx_ordinal = 0;
unsigned g_hudfx_sweep_secs = 0;
void hudfx_draw_label() {
  const unsigned draw = g_hudfx_draw.load();
  if (draw == 0) snprintf(g_hudfx_label, sizeof(g_hudfx_label), "HUD draw test: off");
  else snprintf(g_hudfx_label, sizeof(g_hudfx_label), "HUD draw test: %u of %u%s", draw, g_hudfx_draw_max.load(), g_hudfx_sweeping.load() ? " (sweep)" : "");
}
void hudfx_sweep_step() {
  if (!g_hudfx_sweeping.load() || ++g_hudfx_sweep_secs < 3) return;
  g_hudfx_sweep_secs = 0;
  const unsigned draw = g_hudfx_draw.load() + 1;
  if (draw > g_hudfx_draw_max.load()) { g_hudfx_sweeping.store(false); g_hudfx_draw.store(0); }
  else g_hudfx_draw.store(draw);
  hudfx_draw_label();
}
void hudfx_refresh() {
  static unsigned applied = 0;
  const unsigned pick = g_hudfx_pick.load();
  if (pick != applied || pick != 0) {
    for (auto& slot : g_hudfx_psos) slot.store(nullptr, std::memory_order_relaxed);
    applied = pick;
  }
  if (pick == 0) return;
  std::lock_guard<std::mutex> lock(g_picture_mutex);
  for (const auto& entry : g_pso_ps_id) {
    if (entry.second == kHudFxCandidates[pick - 1]) pso_set_add(g_hudfx_psos, entry.first);
  }
}
void census_report(const SYSTEMTIME& st) {
  hudfx_refresh();
  if (const unsigned draw = g_hudfx_draw.load()) {
    const std::uint64_t key = g_hudfx_draw_key.load();
    auto* pso = reinterpret_cast<ID3D12PipelineState*>(key >> 16);
    std::uint32_t id = 0;
    {
      std::lock_guard<std::mutex> lock(g_picture_mutex);
      const auto found = g_pso_ps_id.find(pso);
      if (found != g_pso_ps_id.end()) id = found->second;
    }
    logf("%02u:%02u:%02u hudfx: draw %u of %u ps%08x/%u/b%d sweep=%d", st.wHour, st.wMinute, st.wSecond, draw, g_hudfx_draw_max.load(), id,
         static_cast<unsigned>(key & 0xFFFFu), blend_of(pso), g_hudfx_sweeping.load() ? 1 : 0);
  }
  hudfx_sweep_step();
  if (const unsigned pick = g_hudfx_pick.load()) {
    logf("%02u:%02u:%02u hudfx: candidate %u ps%08x draws=%llu", st.wHour, st.wMinute, st.wSecond, pick, kHudFxCandidates[pick - 1],
         static_cast<unsigned long long>(g_hudfx_draws.exchange(0)));
  }
  char line[512];
  int used = 0;
  const auto flush = [&] {
    if (used) logf("%02u:%02u:%02u hudcensus:%s", st.wHour, st.wMinute, st.wSecond, line);
    used = 0;
  };
  for (CensusSlot& s : g_census) {
    const std::uint32_t n = s.n.exchange(0);
    if (!n) continue;
    const std::uint64_t key = s.key.load();
    auto* pso = reinterpret_cast<ID3D12PipelineState*>(key >> 16);
    std::uint32_t id = 0;
    {
      std::lock_guard<std::mutex> lock(g_picture_mutex);
      const auto found = g_pso_ps_id.find(pso);
      if (found != g_pso_ps_id.end()) id = found->second;
    }
    used += snprintf(line + used, sizeof(line) - used, " ps%08x/%u/b%d x%u", id, static_cast<unsigned>(key & 0xFFFFu), blend_of(pso), n);
    if (used > 440) flush();
  }
  flush();
}

// One call per eligible HUD-pass indexed draw while the sweep is armed. The count stays per eye and per pass.
bool hud_draw_pick(ID3D12PipelineState* pso, UINT count) {
  const unsigned wanted = g_hudfx_draw.load(std::memory_order_relaxed);
  if (wanted == 0) return false;
  const unsigned ordinal = ++t_hudfx_ordinal;
  unsigned top = g_hudfx_draw_max.load(std::memory_order_relaxed);
  while (ordinal > top && !g_hudfx_draw_max.compare_exchange_weak(top, ordinal)) {}
  if (ordinal != wanted) return false;
  g_hudfx_draw_key.store((reinterpret_cast<std::uint64_t>(pso) << 16) | (count & 0xFFFFu), std::memory_order_relaxed);
  return true;
}

// In-mod frame generation, HUD kept steady (2026-11-13; frame_gen.hpp): at the first draw of each eye's gameplay UI pass
// the eye image (the scene without the HUD) is copied, so generated frames can warp the scene and lay the unwarped
// HUD back on top. At a draw the bound target is in RENDER_TARGET. Front-end panel layer and other passes excluded.
thread_local bool t_prehud_done = false;  // reset in ui_pass_begin
void prehud_note(ID3D12GraphicsCommandList* list) {
  if (t_prehud_done || !frame_gen::enabled()) return;
  const int eye = native_stereo::ui_pass_eye();
  if ((eye != 1 && eye != 2) || native_stereo::ui_pass_kind() != 0 || native_stereo::ui_layer_active() || t_layer_list != nullptr ||
      t_rt_list != list || t_rt_count < 1) {
    return;
  }
  t_prehud_done = true;
  if (ID3D12Resource* r = rtv_resource(t_rts[0].ptr)) {
    frame_gen::capture_prehud(eye - 1, list, r);
  }
}

void STDMETHODCALLTYPE Draw_hook(ID3D12GraphicsCommandList* list, UINT a, UINT b, UINT c, UINT d) {
  prehud_note(list);
  scan_binding_trace(list, a, false);
  render_activity::note(list,0);
  if (ID3D12PipelineState* clone = occlusion_clone(list, note_fp_draw(list))) {
    ID3D12PipelineState* original = t_pso;
    g_set_pso(list, clone);
    g_draw(list, a, b, c, d);
    g_set_pso(list, original);
    g_occlude_swaps.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const int eye = native_stereo::ui_pass_eye();
  const bool placed = eye != 0 && t_placed_list == list && t_mapped_now;
  // During the scan (reliable signal since 11:44) the UI pass keeps its placement, except full-screen draws and draws
  // with a pipeline state never seen in a placed UI pass outside the scan (the scan picture and its highlights).
  const bool kind0 = native_stereo::ui_pass_kind() == 0;
  if (placed && kind0 && eye == 1) census_note(t_pso, a);
  ui_ownership::layer_draw(eye);
  const bool scanning = placed && kind0 && scan_active();
  const bool known = ui_pso_known(t_pso);
  if (scanning && !known) {
    pso_set_add(g_scan_psos, t_pso);
  }
  if (placed && ui_ownership::arm_layer_active()) {
    t_reproj_draw = false;
    draw_unplaced(list, eye, [&] { g_draw(list, a, b, c, d); });
  } else if (placed && kind0 && ((scanning && ((a <= 4 && b == 1) || !known)) || pso_set_has(g_scan_psos, t_pso))) {
    // Scan draws in place; in the right eye blended ones are reprojected (see DrawIndexed_hook).
    g_ui[eye].fullscreen.fetch_add(1, std::memory_order_relaxed);
    t_reproj_draw = !g_scan_shift_on.load(std::memory_order_relaxed) && blend_of(t_pso) == 1;
    if (eye == 2) {
      scan_draw_note(t_pso, 0x80000000u | a, t_reproj_draw);  // high bit: DrawInstanced vertex count
    }
    draw_unplaced(list, eye, [&] { g_draw(list, a, b, c, d); });
    t_reproj_draw = false;
  } else if (placed && kind0 && ui_ownership::fx_layer_active()) {
    t_reproj_draw = false;
    draw_fx_plane(list, eye, [&] { g_draw(list, a, b, c, d); });
  } else if (placed && a == 3 && b == 1 && (g_fx_tri[kind0 ? 0 : 1].fetch_add(1, std::memory_order_relaxed), true) &&
             g_fx_stretch.load(std::memory_order_relaxed)) {
    g_fx_tri_full[kind0 ? 0 : 1].fetch_add(1, std::memory_order_relaxed);
    draw_full_target(list, eye, [&] { g_draw(list, a, b, c, d); });
  } else {
    if (kind0 && !scanning) hud_eye_draw(list, eye, placed, [&] { g_draw(list, a, b, c, d); });
    else g_draw(list, a, b, c, d);
    if (kind0 && !scanning && hud_copy_wanted(list, eye, placed)) {
      hud_copy(list, [&] { g_draw(list, a, b, c, d); });
    }
  }
  if (eye != 0) {
    if (a <= 4 && b == 1 && t_map_scale == 1.f && t_vp_list == list && t_vp.Width < static_cast<float>(g_backbuffer_w.load()) * 0.75f) {
      sig_count(g_sig_h3);
    }
    g_ui[eye].draws.fetch_add(1, std::memory_order_relaxed);
    trace_ev('D', a, b, 0, 0, t_mapped_now && t_placed_list == list);
  }
}

void STDMETHODCALLTYPE DrawIndexed_hook(ID3D12GraphicsCommandList* list, UINT a, UINT b, UINT c, INT d, UINT e) {
  prehud_note(list);
  scan_binding_trace(list, a, true);
  scan_inputs_draw(list,a,b,c,d,e);
  render_activity::note(list,0);
  if (ID3D12PipelineState* clone = occlusion_clone(list, note_fp_draw(list))) {
    ID3D12PipelineState* original = t_pso;
    g_set_pso(list, clone);
    g_draw_indexed(list, a, b, c, d, e);
    g_set_pso(list, original);
    g_occlude_swaps.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const int eye = native_stereo::ui_pass_eye();
  // Only the captured scanner composite, right eye, while scanning. Binding
  // and viewport state remain the game's; restore the original PSO after draw.
  bool exact_scanner=false;
  if(g_exact_fx_route&&g_fx_stretch.load(std::memory_order_relaxed)&&eye!=0&&native_stereo::ui_pass_kind()==0&&
     !t_depth_bound&&!native_stereo::front_end()&&t_layer_list==nullptr) {
    std::lock_guard<std::mutex> lock(g_scan_uv_mutex);
    exact_scanner=g_scan_uv_psos.count(t_pso)!=0;
  }
  ID3D12PipelineState* uv_clone = (eye == 2 || (eye==1&&g_scan_plane_wide)) && native_stereo::ui_pass_kind() == 0 && (exact_scanner||scan_active())
      ? scan_uv_clone(list,t_pso,eye) : nullptr;
  struct RestoreScannerPso {
    ID3D12GraphicsCommandList* list; ID3D12PipelineState* original; bool changed;
    ~RestoreScannerPso() { if(changed) g_set_pso(list,original); }
  } restore_scanner{list,t_pso,uv_clone!=nullptr};
  if(uv_clone) {g_set_pso(list,uv_clone);g_scan_uv_draws.fetch_add(1,std::memory_order_relaxed);}
  const bool placed = eye != 0 && t_placed_list == list && t_mapped_now;
  const bool kind0 = native_stereo::ui_pass_kind() == 0;
  if (placed && kind0 && eye == 1) census_note(t_pso, a);
  ui_ownership::layer_draw(eye);
  const bool scanning = placed && kind0 && scan_active();
  const bool known = ui_pso_known(t_pso);
  const bool scan_pso =placed && kind0 && pso_set_has(g_scan_psos, t_pso);
  if (scanning && !known) {
    pso_set_add(g_scan_psos, t_pso);
  }
  const bool exact_picture=g_picture_on&&pso_set_has(g_picture_psos,t_pso);
  if(exact_picture||exact_scanner||scanning) ui_ownership::fx_effect_drawing(ui_ownership::fx_layer_active());
  if (placed && kind0 && !scanning && !scan_pso && !exact_scanner && !(g_exact_fx_route&&exact_picture)) {
    ui_pso_learn(t_pso);
  }
  if(placed&&ui_ownership::arm_layer_active()) {
    t_reproj_draw=false;
    draw_unplaced(list,eye,[&] {g_draw_indexed(list,a,b,c,d,e);});
  } else if(placed&&exact_scanner) {
    // This shader composites scene colour as well as the highlight mask. It must
    // never become HUD art or receive the legacy whole-picture eye reprojection.
    // The scanner twin already maps its mask independently for the right eye.
    t_reproj_draw=false;
    draw_unplaced(list,eye,[&] {g_draw_indexed(list,a,b,c,d,e);});
    g_exact_scan_draws.fetch_add(1,std::memory_order_relaxed);
    if(!scan_active()) g_exact_scan_without_signal.fetch_add(1,std::memory_order_relaxed);
  } else if(placed&&kind0&&a==6&&b==1&&!t_depth_bound&&!native_stereo::front_end()&&t_layer_list==nullptr&&exact_picture&&g_fx_stretch.load(std::memory_order_relaxed)) {
    // Captured boost whole-picture radial blur uses a logical-canvas quad,
    // unlike normal HUD art. Stretch this exact shader, not every I6 draw.
    t_reproj_draw=false;
    draw_unplaced(list,eye,[&] {g_draw_indexed(list,a,b,c,d,e);});
    g_picture_draws.fetch_add(1,std::memory_order_relaxed);
  } else if(placed&&kind0&&!scan_pso&&!(scanning&&!known)&&ui_ownership::fx_layer_active()) {
    t_reproj_draw=false;
    draw_fx_plane(list,eye,[&] {g_draw_indexed(list,a,b,c,d,e);});
  } else if(placed&&kind0&&!scanning&&!scan_pso&&!t_depth_bound&&!native_stereo::front_end()&&t_layer_list==nullptr&&
            g_fx_stretch.load(std::memory_order_relaxed)&&
            (hud_draw_pick(t_pso,a)||(g_hudfx_pick.load(std::memory_order_relaxed)!=0&&pso_set_has(g_hudfx_psos,t_pso)))) {
    t_reproj_draw=false;
    draw_unplaced(list,eye,[&] {g_draw_indexed(list,a,b,c,d,e);});
    g_hudfx_draws.fetch_add(1,std::memory_order_relaxed);
  } else if ((scanning && (!known || (a == 3 && b == 1))) || scan_pso) {
    // Scan draw: in place. In the right eye only the highlight meshes (index counts other than 3 or 6; uitrace: 5580,
    // 894, 528 ... drawn with the main view's camera) go through the left-to-right reprojection. The scan's
    // full-screen / quad draws stay in place: reprojected, they carried the left eye's whole picture into the right
    // eye (capture 12:3x: right eye = left picture shifted ~430 px, black band at its edge).
    g_ui[eye].world.fetch_add(1, std::memory_order_relaxed);
    // Right eye: blended scan draws carry the main view's highlights -> reprojected; opaque ones (the scan picture)
    // stay. PSOs whose blend state was not seen at creation fall back to the mesh rule (index count not 3 or 6).
    const int blend = blend_of(t_pso);
    // With the scan targets shifted at the source (scan_shift) nothing is reprojected; the viewport rule stays as the
    // fallback when the shift is off.
    t_reproj_draw = !g_scan_shift_on.load(std::memory_order_relaxed) && (blend == 1 || (blend == 0 && a != 3 && a != 6));
    if (eye == 2) {
      scan_draw_note(t_pso, a, t_reproj_draw);
      srv_probe_draw(list, t_pso, a);
    }
    const int shifted = eye == 2 ? mask_shift_pre(list) : 0;
    const bool redirected = eye == 2 && shifted == 0 && ov_pre(list);
    draw_unplaced(list, eye, [&] { g_draw_indexed(list, a, b, c, d, e); });
    if (redirected) {
      ov_post(list);
    }
    if (shifted > 0) {
      mask_shift_post(list);
    }
    t_reproj_draw = false;
  } else if (placed && kind0 && blend_of(t_pso) == 2 && (g_fx_opaque_draws.fetch_add(1, std::memory_order_relaxed), true) &&
             g_fx_stretch.load(std::memory_order_relaxed)) {
    // Opaque draw in the placed HUD pass: the screen effect's whole picture (HUD draws blend) -> whole eye.
    g_fx_stretched_opaque.fetch_add(1, std::memory_order_relaxed);
    draw_unplaced(list, eye, [&] { g_draw_indexed(list, a, b, c, d, e); });
  } else {
    if (kind0 && !scanning && !scan_pso) hud_eye_draw(list, eye, placed, [&] { g_draw_indexed(list, a, b, c, d, e); });
    else g_draw_indexed(list, a, b, c, d, e);
    if (kind0 && !scanning && !scan_pso && hud_copy_wanted(list, eye, placed)) {
      hud_copy(list, [&] { g_draw_indexed(list, a, b, c, d, e); });
    }
  }
  if (eye != 0) {
    if (a > 3000) {
      sig_count(g_sig_big);
    }
    const bool under = t_mapped_now && t_placed_list == list;
    trace_ev('I', a, b, 0, 0, under);
    if (under) {
      (t_depth_bound ? g_draws_dsv_mapped : g_draws_nodsv_mapped).fetch_add(1, std::memory_order_relaxed);
    }
    g_ui[eye].indexed.fetch_add(1, std::memory_order_relaxed);
    (a == 3 ? g_ui[eye].idx3 : a == 6 ? g_ui[eye].idx6 : g_ui[eye].idx_other).fetch_add(1, std::memory_order_relaxed);
  }
}

// ---- Scan mask shift (2026-10-08) --------------------------------------------------------------------------------
// [AUTHOR] + captures / scandraw table 12:5x: in the right eye the scan's last step is ONE draw (PSO 9D30, 6 indices,
// premultiplied blend) that mixes the right eye's own picture with the main view's highlight images. Reprojecting the
// draw aligns the highlights but shifts the picture (black bar); leaving it misplaces the highlights. So the draw stays
// in place and the images it reads are shifted instead: the render targets the neighbour passes write during the scan
// (the highlight meshes' target, 1172x1271, and the half-size targets after it) are copied shifted by the
// left-to-right offset (scan_reproject; both eyes span the same tangents, so a horizontal shift at the target's own
// resolution) at the start of the second view's UI pass. States come from the game's own ResourceBarrier calls.
// OPEN: whether the main view's UI pass reads them before the second view's runs (else the LEFT eye would shift).
// "noscanshift" (native control) turns it off.
constexpr int kRtvSlots = 1 << 16;
std::atomic<SIZE_T> g_rtv_key[kRtvSlots];
std::atomic<ID3D12Resource*> g_rtv_res[kRtvSlots];

size_t rtv_slot(SIZE_T h) { return (static_cast<std::uint64_t>(h) >> 3) * 0x9E3779B97F4A7C15ull >> 48; }

void rtv_note(SIZE_T h, ID3D12Resource* r) {
  for (size_t i = rtv_slot(h), n = 0; n < 32; ++n, i = (i + 1) % kRtvSlots) {
    SIZE_T v = g_rtv_key[i].load(std::memory_order_relaxed);
    if (v == h || (v == 0 && (g_rtv_key[i].compare_exchange_strong(v, h) || v == h))) {
      g_rtv_res[i].store(r, std::memory_order_relaxed);
      return;
    }
  }
}

ID3D12Resource* rtv_resource(SIZE_T h) {
  for (size_t i = rtv_slot(h), n = 0; n < 32; ++n, i = (i + 1) % kRtvSlots) {
    SIZE_T v = g_rtv_key[i].load(std::memory_order_relaxed);
    if (v == h) {
      return g_rtv_res[i].load(std::memory_order_relaxed);
    }
    if (v == 0) {
      return nullptr;
    }
  }
  return nullptr;
}

// SRV probe (2026-10-08, read-only). [LIVE] 13:1x: the per-frame shift never ran (0 shifts, 157 skips) -- which image the
// second view's scan composite (PSO 9D30) reads, and when, is a guess on a multi-threaded engine. This finds out:
// CPU SRV handle -> resource (CreateShaderResourceView), copies into shader-visible heaps (CopyDescriptors[Simple]) for
// SRVs of scan targets only, shader-visible heap ranges (CreateDescriptorHeap), and the descriptor tables bound per
// list (SetGraphicsRootDescriptorTable). At the second view's scan draws the bound tables are searched for scan targets.
constexpr int kSrvSlots = 1 << 18;
std::atomic<SIZE_T> g_srv_key[kSrvSlots];
std::atomic<ID3D12Resource*> g_srv_res[kSrvSlots];
size_t srv_slot(SIZE_T h) { return (static_cast<std::uint64_t>(h) >> 3) * 0x9E3779B97F4A7C15ull >> 46; }

void srv_note(SIZE_T h, ID3D12Resource* r) {
  for (size_t i = srv_slot(h), n = 0; n < 32; ++n, i = (i + 1) % kSrvSlots) {
    SIZE_T v = g_srv_key[i].load(std::memory_order_relaxed);
    if (v == h || (v == 0 && (g_srv_key[i].compare_exchange_strong(v, h) || v == h))) {
      g_srv_res[i].store(r, std::memory_order_relaxed);
      return;
    }
  }
}

ID3D12Resource* srv_resource(SIZE_T h) {
  for (size_t i = srv_slot(h), n = 0; n < 32; ++n, i = (i + 1) % kSrvSlots) {
    SIZE_T v = g_srv_key[i].load(std::memory_order_relaxed);
    if (v == h) {
      return g_srv_res[i].load(std::memory_order_relaxed);
    }
    if (v == 0) {
      return nullptr;
    }
  }
  return nullptr;
}

struct VisHeap {
  SIZE_T cpu = 0;
  UINT64 gpu = 0;
  UINT count = 0;
};
VisHeap g_vis_heaps[16];
std::atomic<int> g_vis_heap_n{0};
UINT g_srv_stride = 0;
thread_local UINT64 t_tables[32] = {};  // matches RootInfo's 32 root parameters
thread_local ID3D12GraphicsCommandList* t_tables_list = nullptr;

SIZE_T gpu_to_cpu(UINT64 gpu) {
  const int n = g_vis_heap_n.load(std::memory_order_acquire);
  for (int i = 0; i < n; ++i) {
    const VisHeap& h = g_vis_heaps[i];
    if (gpu >= h.gpu && gpu < h.gpu + static_cast<UINT64>(h.count) * g_srv_stride) {
      return h.cpu + static_cast<SIZE_T>(gpu - h.gpu);
    }
  }
  return 0;
}

struct ScanTarget {
  ID3D12Resource* res = nullptr;
  ID3D12Resource* temp = nullptr;
  ID3D12Resource* zero = nullptr;  // never written: zero-initialised source for the strip a shift uncovers
  std::atomic<std::uint32_t> state{0xFFFFFFFFu};  // last StateAfter from the game's barriers (0xFFFFFFFF = unknown)
  UINT64 width = 0;
  UINT height = 0;
};
constexpr int kScanTargets = 8;
ScanTarget g_scan_targets[kScanTargets];
std::atomic<int> g_scan_target_n{0};
std::mutex g_scan_target_mutex;
std::atomic<std::uint64_t> g_scan_shifts{0}, g_scan_shift_skips{0};
// [LIVE] crash 2026-10-08 13:08 (3 s into the scan, 873 shifts): 8 targets were collected -- two alternating sets of
// pooled transients (996x1080 fmt 28/49/61, 1172x1271 fmt 28) -- and all were shifted every frame, also the set not in
// use that frame, whose memory the engine had handed to other work. Only the LAST target the neighbour passes bound
// (the chain's output, which the second view's composite reads) is shifted now, and only within 50 ms of that bind.
std::atomic<ID3D12Resource*> g_last_fx_target{nullptr};
std::atomic<ULONGLONG> g_last_fx_tick{0};
thread_local bool t_shift_done = false;  // per UI pass

void scan_target_add(ID3D12Resource* r) {
  if (r == nullptr) {
    return;
  }
  const int n = g_scan_target_n.load(std::memory_order_acquire);
  for (int i = 0; i < n; ++i) {
    if (g_scan_targets[i].res == r) {
      return;
    }
  }
  std::lock_guard<std::mutex> lock(g_scan_target_mutex);
  const int m = g_scan_target_n.load();
  for (int i = 0; i < m; ++i) {
    if (g_scan_targets[i].res == r) {
      return;
    }
  }
  if (m >= kScanTargets) {
    return;
  }
  const D3D12_RESOURCE_DESC d = r->GetDesc();
  if (d.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || d.SampleDesc.Count != 1 || d.DepthOrArraySize != 1) {
    return;
  }
  g_scan_targets[m].res = r;
  g_scan_targets[m].width = d.Width;
  g_scan_targets[m].height = d.Height;
  g_scan_target_n.store(m + 1, std::memory_order_release);
  logf("scan shift: target %d = %p %llux%u fmt %d flags 0x%X", m, static_cast<void*>(r), static_cast<unsigned long long>(d.Width), d.Height,
       static_cast<int>(d.Format), static_cast<unsigned>(d.Flags));
}

int scan_target_index(ID3D12Resource* r) {
  const int n = g_scan_target_n.load(std::memory_order_acquire);
  for (int i = 0; i < n; ++i) {
    if (g_scan_targets[i].res == r) {
      return i;
    }
  }
  return -1;
}

void scan_state_note(ID3D12Resource* r, D3D12_RESOURCE_STATES after) {
  const int n = g_scan_target_n.load(std::memory_order_acquire);
  for (int i = 0; i < n; ++i) {
    if (g_scan_targets[i].res == r) {
      g_scan_targets[i].state.store(static_cast<std::uint32_t>(after), std::memory_order_relaxed);
      return;
    }
  }
}

using ResourceBarrierFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, const D3D12_RESOURCE_BARRIER*);
ResourceBarrierFn g_resource_barrier = nullptr;

void barrier_on(ID3D12GraphicsCommandList* list, ID3D12Resource* r, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = r;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = before;
  b.Transition.StateAfter = after;
  if (g_resource_barrier != nullptr) {
    g_resource_barrier(list, 1, &b);
  } else {
    list->ResourceBarrier(1, &b);
  }
}

void STDMETHODCALLTYPE ResourceBarrier_hook(ID3D12GraphicsCommandList* list, UINT count, const D3D12_RESOURCE_BARRIER* barriers) {
  render_activity::note(list,6);
  g_resource_barrier(list, count, barriers);
  if (barriers != nullptr && g_scan_target_n.load(std::memory_order_relaxed) > 0) {
    for (UINT i = 0; i < count; ++i) {
      if (barriers[i].Type == D3D12_RESOURCE_BARRIER_TYPE_TRANSITION) {
        scan_state_note(barriers[i].Transition.pResource, barriers[i].Transition.StateAfter);
      }
    }
  }
}

// Our copy of a scan target: same size and format, simultaneous-access (decays to COMMON, promoted to COPY_DEST).
bool scan_temp(ID3D12GraphicsCommandList* list, ScanTarget& t) {
  if (t.temp != nullptr) {
    return true;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(list->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) {
    return false;
  }
  D3D12_RESOURCE_DESC d = t.res->GetDesc();
  d.Flags = static_cast<D3D12_RESOURCE_FLAGS>((d.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS);
  d.MipLevels = 1;
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_DEFAULT;
  const HRESULT hr = device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&t.temp));
  device->Release();
  if (FAILED(hr)) {
    t.temp = nullptr;
    logf("scan shift: temp for %p failed hr=0x%08lX", static_cast<void*>(t.res), static_cast<unsigned long>(hr));
    return false;
  }
  t.temp->SetName(L"AvatarVR_ScanShift");
  return true;
}

using CreateRtvFn = void(STDMETHODCALLTYPE*)(ID3D12Device*, ID3D12Resource*, const D3D12_RENDER_TARGET_VIEW_DESC*, D3D12_CPU_DESCRIPTOR_HANDLE);
using CopyDescSimpleFn = void(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_DESCRIPTOR_HEAP_TYPE);
CreateRtvFn g_create_rtv = nullptr;
CopyDescSimpleFn g_copy_desc_simple = nullptr;
UINT g_rtv_stride = 0;

using CreateSrvFn = void(STDMETHODCALLTYPE*)(ID3D12Device*, ID3D12Resource*, const D3D12_SHADER_RESOURCE_VIEW_DESC*, D3D12_CPU_DESCRIPTOR_HANDLE);
using CreateHeapFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, const D3D12_DESCRIPTOR_HEAP_DESC*, REFIID, void**);
using CopyDescFn = void(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, const D3D12_CPU_DESCRIPTOR_HANDLE*, const UINT*, UINT,
                                            const D3D12_CPU_DESCRIPTOR_HANDLE*, const UINT*, D3D12_DESCRIPTOR_HEAP_TYPE);
using SetTableFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, D3D12_GPU_DESCRIPTOR_HANDLE);
CreateSrvFn g_create_srv = nullptr;
CreateHeapFn g_create_heap = nullptr;
CopyDescFn g_copy_desc = nullptr;
SetTableFn g_set_table = nullptr;
std::atomic<std::uint64_t> g_srv_created{0}, g_srv_copied{0};

void STDMETHODCALLTYPE CreateSrv_hook(ID3D12Device* dev, ID3D12Resource* r, const D3D12_SHADER_RESOURCE_VIEW_DESC* desc, D3D12_CPU_DESCRIPTOR_HANDLE h) {
  g_create_srv(dev, r, desc, h);
  if(scanner_inputs::enabled.load(std::memory_order_relaxed)) {
    UINT64 tag=desc?(UINT64(desc->Format)|(UINT64(desc->ViewDimension)<<16)):0;
    if(desc&&desc->ViewDimension==D3D12_SRV_DIMENSION_TEXTURE2D)
      tag|=(UINT64(desc->Texture2D.MostDetailedMip&255)<<24)|(UINT64(desc->Texture2D.MipLevels&255)<<32);
    if(desc&&desc->ViewDimension==D3D12_SRV_DIMENSION_TEXTURE2DARRAY)
      tag|=(UINT64(desc->Texture2DArray.MostDetailedMip&255)<<24)|(UINT64(desc->Texture2DArray.FirstArraySlice&65535)<<32)|(UINT64(desc->Texture2DArray.ArraySize&65535)<<48);
    UINT64 raw_va=0,raw_bytes=0;
    if(r&&desc&&desc->ViewDimension==D3D12_SRV_DIMENSION_BUFFER&&
       (desc->Buffer.Flags&D3D12_BUFFER_SRV_FLAG_RAW)) {
      const auto rd=r->GetDesc();
      const UINT64 first=desc->Buffer.FirstElement;
      if(rd.Dimension==D3D12_RESOURCE_DIMENSION_BUFFER&&first<=rd.Width/4&&
         UINT64(desc->Buffer.NumElements)<=(rd.Width-first*4)/4) {
        const UINT64 va=r->GetGPUVirtualAddress();
        if(va&&va<=~UINT64(0)-first*4) {raw_va=va+first*4;raw_bytes=UINT64(desc->Buffer.NumElements)*4;}
      }
    }
    scanner_inputs::View observation{reinterpret_cast<UINT64>(r),0,tag,raw_va,raw_bytes};
    // Current capture: all scanner transform SRVs expose exactly one 64-byte
    // matrix. Snapshot it while r is live, including transiently unmapped uploads.
    if(raw_va&&raw_bytes==64) {
      observation.raw_present=upload::present_sequence();
      observation.raw_saved=upload::copy_live_raw64(r,desc->Buffer.FirstElement*4,observation.raw64.data(),&observation.heap_type,&observation.map_result);
    }
    scanner_inputs::note(h.ptr,observation);
  }
  if (r != nullptr) {
    srv_note(h.ptr, r);
    g_srv_created.fetch_add(1, std::memory_order_relaxed);
  }
}

HRESULT STDMETHODCALLTYPE CreateHeap_hook(ID3D12Device* dev, const D3D12_DESCRIPTOR_HEAP_DESC* desc, REFIID riid, void** out) {
  const HRESULT hr = g_create_heap(dev, desc, riid, out);
  if (SUCCEEDED(hr) && desc != nullptr && out != nullptr && *out != nullptr && desc->Type == D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV &&
      (desc->Flags & D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE) != 0) {
    ID3D12DescriptorHeap* heap = nullptr;
    if (SUCCEEDED(static_cast<IUnknown*>(*out)->QueryInterface(IID_PPV_ARGS(&heap)))) {
      const int n = g_vis_heap_n.load();
      if (n < 16) {
        g_vis_heaps[n].cpu = heap->GetCPUDescriptorHandleForHeapStart().ptr;
        g_vis_heaps[n].gpu = heap->GetGPUDescriptorHandleForHeapStart().ptr;
        g_vis_heaps[n].count = desc->NumDescriptors;
        g_vis_heap_n.store(n + 1, std::memory_order_release);
        logf("srv probe: shader-visible heap %d cpu=%llx gpu=%llx n=%u", n, static_cast<unsigned long long>(g_vis_heaps[n].cpu),
             static_cast<unsigned long long>(g_vis_heaps[n].gpu), desc->NumDescriptors);
      }
      heap->Release();
    }
  }
  return hr;
}

// Propagate only SRVs of scan targets (cheap: one lookup per copied descriptor).
void srv_copy_one(SIZE_T dst, SIZE_T src) {
  ID3D12Resource* r = srv_resource(src);
  if (r != nullptr && scan_target_index(r) >= 0) {
    srv_note(dst, r);
    g_srv_copied.fetch_add(1, std::memory_order_relaxed);
  }
}

void STDMETHODCALLTYPE CopyDesc_hook(ID3D12Device* dev, UINT ndst, const D3D12_CPU_DESCRIPTOR_HANDLE* dst, const UINT* dsz, UINT nsrc,
                                     const D3D12_CPU_DESCRIPTOR_HANDLE* src, const UINT* ssz, D3D12_DESCRIPTOR_HEAP_TYPE type) {
  g_copy_desc(dev, ndst, dst, dsz, nsrc, src, ssz, type);
  if (type != D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV || (!scanner_inputs::enabled.load(std::memory_order_relaxed)&&g_scan_target_n.load(std::memory_order_relaxed) == 0) || g_srv_stride == 0 ||
      dst == nullptr || src == nullptr) {
    return;
  }
  // Walk both range lists in step (descriptor by descriptor).
  UINT di = 0, si = 0, doff = 0, soff = 0;
  for (int guard = 0; di < ndst && si < nsrc && guard < 4096; ++guard) {
    const UINT dn = dsz != nullptr ? dsz[di] : 1, sn = ssz != nullptr ? ssz[si] : 1;
    if(!dn) {++di;doff=0;continue;}
    if(!sn) {++si;soff=0;continue;}
    srv_copy_one(dst[di].ptr + static_cast<SIZE_T>(doff) * g_srv_stride, src[si].ptr + static_cast<SIZE_T>(soff) * g_srv_stride);
    scanner_inputs::copy(dst[di].ptr+static_cast<SIZE_T>(doff)*g_srv_stride,src[si].ptr+static_cast<SIZE_T>(soff)*g_srv_stride);
    if (++doff >= dn) {
      ++di;
      doff = 0;
    }
    if (++soff >= sn) {
      ++si;
      soff = 0;
    }
  }
}

void STDMETHODCALLTYPE SetTable_hook(ID3D12GraphicsCommandList* list, UINT index, D3D12_GPU_DESCRIPTOR_HANDLE h) {
  g_set_table(list, index, h);
  if (t_tables_list != list) {
    t_tables_list = list;
    std::memset(t_tables, 0, sizeof(t_tables));
  }
  if (index < 32) {
    t_tables[index] = h.ptr;
  }
}

// At a second-view scan draw: which scan targets do the bound tables reference (first 32 descriptors of each)?
std::atomic<int> g_srv_probe_logs{0};
// Scan mask shift (2026-10-08). [LIVE] SRV probe 13:29-13:34: the second view's scan composite (PSO ...FBC0 this run,
// 6 indices, premultiplied) reads scan target T5 = 996x1080 fmt 49 (R8G8_UNORM, ALLOW_RENDER_TARGET only) -- the
// half-size two-channel mask of the main view's scan (highlights and the blue tint). In the second view's own list,
// right around that draw: back up the mask, write it back shifted by the left-to-right offset at its width (the
// uncovered strip from a never-written zero texture), draw, copy the backup back. Ordering is that list's, so the main
// view always reads the original; the mask is sampled by the draw, so its tracked state is a shader-resource state.
// Off by default: [AUTHOR] 2026-10-08 13:38 crash (device removed, DXGI 0x887A000B) when the scan opened with it on.
std::atomic<bool> g_scan_mask_on{false};
std::atomic<std::uint64_t> g_mask_shifts{0}, g_mask_skips{0};
thread_local ScanTarget* t_mask_shifted[4] = {};
thread_local int t_mask_shifted_n = 0;

bool mask_temps(ID3D12GraphicsCommandList* list, ScanTarget& t) {
  if (!scan_temp(list, t)) {
    return false;
  }
  if (t.zero != nullptr) {
    return true;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(list->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr) {
    return false;
  }
  D3D12_RESOURCE_DESC d = t.res->GetDesc();
  d.Flags = static_cast<D3D12_RESOURCE_FLAGS>((d.Flags & D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET) | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS);
  d.MipLevels = 1;
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_DEFAULT;
  const HRESULT hr = device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&t.zero));
  device->Release();
  if (FAILED(hr)) {
    t.zero = nullptr;
    return false;
  }
  t.zero->SetName(L"AvatarVR_ScanZero");
  return true;
}

int mask_shift_pre(ID3D12GraphicsCommandList* list) {
  t_mask_shifted_n = 0;
  if (!g_scan_mask_on.load(std::memory_order_relaxed) || t_tables_list != list) {
    return 0;
  }
  const float eye_w = static_cast<float>(g_backbuffer_w.load()), eye_h = static_cast<float>(g_backbuffer_h.load());
  float ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  if (!native_stereo::scan_offsets(eye_w, eye_h, &ox, &oy, &sx, &sy)) {
    return 0;
  }
  for (int p = 0; p < 16 && t_mask_shifted_n < 4; ++p) {
    if (t_tables[p] == 0) {
      continue;
    }
    const SIZE_T cpu = gpu_to_cpu(t_tables[p]);
    for (int k = 0; cpu != 0 && k < 32 && t_mask_shifted_n < 4; ++k) {
      ID3D12Resource* r = srv_resource(cpu + static_cast<SIZE_T>(k) * g_srv_stride);
      const int ti = r != nullptr ? scan_target_index(r) : -1;
      if (ti < 0) {
        continue;
      }
      ScanTarget& t = g_scan_targets[ti];
      bool dup = false;
      for (int j = 0; j < t_mask_shifted_n; ++j) {
        dup = dup || t_mask_shifted[j] == &t;
      }
      const std::uint32_t st = t.state.load(std::memory_order_relaxed);
      const bool readable = st != 0xFFFFFFFFu && (st & (D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)) != 0;
      if (dup || !readable || !mask_temps(list, t)) {
        if (!dup) {
          g_mask_skips.fetch_add(1, std::memory_order_relaxed);
        }
        continue;
      }
      const auto S = static_cast<D3D12_RESOURCE_STATES>(st);
      const LONG W = static_cast<LONG>(t.width), H = static_cast<LONG>(t.height);
      const LONG dx = static_cast<LONG>(std::lround(ox * static_cast<float>(t.width) / eye_w));
      if (dx == 0 || (dx < 0 ? -dx : dx) >= W) {
        continue;
      }
      D3D12_TEXTURE_COPY_LOCATION res = {t.res, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
      D3D12_TEXTURE_COPY_LOCATION bak = {t.temp, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
      D3D12_TEXTURE_COPY_LOCATION zer = {t.zero, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
      res.SubresourceIndex = bak.SubresourceIndex = zer.SubresourceIndex = 0;
      // 1) backup (mask -> temp)
      barrier_on(list, t.res, S, D3D12_RESOURCE_STATE_COPY_SOURCE);
      const D3D12_BOX all = {0, 0, 0, static_cast<UINT>(W), static_cast<UINT>(H), 1};
      list->CopyTextureRegion(&bak, 0, 0, 0, &res, &all);
      barrier_on(list, t.temp, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COPY_SOURCE);
      // 2) shifted back into the mask: content moves by dx; the uncovered strip from the zero texture
      barrier_on(list, t.res, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_COPY_DEST);
      const D3D12_BOX src_box = {static_cast<UINT>(dx < 0 ? -dx : 0), 0, 0, static_cast<UINT>(dx < 0 ? W : W - dx), static_cast<UINT>(H), 1};
      list->CopyTextureRegion(&res, static_cast<UINT>(dx < 0 ? 0 : dx), 0, 0, &bak, &src_box);
      const LONG strip = dx < 0 ? -dx : dx;
      const D3D12_BOX zero_box = {0, 0, 0, static_cast<UINT>(strip), static_cast<UINT>(H), 1};
      list->CopyTextureRegion(&res, static_cast<UINT>(dx < 0 ? W - strip : 0), 0, 0, &zer, &zero_box);
      barrier_on(list, t.res, D3D12_RESOURCE_STATE_COPY_DEST, S);
      t_mask_shifted[t_mask_shifted_n++] = &t;
      g_mask_shifts.fetch_add(1, std::memory_order_relaxed);
    }
  }
  return t_mask_shifted_n;
}

void mask_shift_post(ID3D12GraphicsCommandList* list) {
  for (int i = 0; i < t_mask_shifted_n; ++i) {
    ScanTarget& t = *t_mask_shifted[i];
    const auto S = static_cast<D3D12_RESOURCE_STATES>(t.state.load(std::memory_order_relaxed));
    barrier_on(list, t.res, S, D3D12_RESOURCE_STATE_COPY_DEST);
    list->CopyResource(t.res, t.temp);  // the original back for everything after this draw
    barrier_on(list, t.res, D3D12_RESOURCE_STATE_COPY_DEST, S);
    barrier_on(list, t.temp, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_COMMON);
  }
  t_mask_shifted_n = 0;
}

void srv_probe_draw(ID3D12GraphicsCommandList* list, ID3D12PipelineState* pso, UINT count) {
  if (g_srv_probe_logs.load(std::memory_order_relaxed) >= 12 || t_tables_list != list) {
    return;
  }
  char text[700];
  int n = snprintf(text, sizeof(text), "srv probe: R scan draw pso=%p I%u tables:", static_cast<void*>(pso), count);
  bool any = false;
  for (int p = 0; p < 16 && n < static_cast<int>(sizeof(text)) - 80; ++p) {
    if (t_tables[p] == 0) {
      continue;
    }
    const SIZE_T cpu = gpu_to_cpu(t_tables[p]);
    n += snprintf(text + n, sizeof(text) - n, " [%d]%s", p, cpu == 0 ? "?" : "");
    for (int k = 0; cpu != 0 && k < 32 && n < static_cast<int>(sizeof(text)) - 40; ++k) {
      ID3D12Resource* r = srv_resource(cpu + static_cast<SIZE_T>(k) * g_srv_stride);
      const int t = r != nullptr ? scan_target_index(r) : -1;
      if (t >= 0) {
        n += snprintf(text + n, sizeof(text) - n, " +%d=T%d", k, t);
        any = true;
      }
    }
  }
  if (any || g_srv_probe_logs.load() < 4) {
    g_srv_probe_logs.fetch_add(1, std::memory_order_relaxed);
    logf("%s", text);
  }
}


// ---- Scan overlay redirect (2026-10-08) ---------------------------------------------------------------------------
// The second view's scan composite reads the main view's (left-eye) scan mask (SRV probe 13:29-13:34: T5, 996x1080
// R8G8_UNORM; sometimes also the 1172x1271 target). Writing the game's textures crashed twice (13:08, 13:38: barrier
// states guessed on a multi-threaded engine). Here nothing of the game's is written or transitioned:
//   1. our compute pass reads each referenced scan target (already in a shader-readable state: the composite samples
//      it next) and writes it shifted by the left-to-right offset into a texture of ours;
//   2. the composite's descriptor tables are copied into our shader-visible heap with those entries pointing at our
//      textures, our heap is bound for that one draw (the game's sampler heap stays), and the tables are set to the
//      copies;
//   3. after the draw the game's heaps, tables and PSO are back.
// Table sizes come from the root signatures (deserialized at creation); a draw whose resource table is unbounded or
// whose root signature is unknown is left alone. "noscanoverlay" turns it off.
struct RootInfo {
  ID3D12RootSignature* rs = nullptr;
  std::vector<unsigned char> original_blob;  // optional CPU-only scanner evidence
  int n = 0;
  std::uint8_t kind[32] = {};  // 0 other, 1 CBV/SRV/UAV table, 2 sampler table
  UINT extent[32] = {};        // descriptors a table spans (0xFFFFFFFF = unbounded)
  struct Cb {UINT root,start,reg,count,space; bool direct;};
  struct Inline {UINT root,reg,space,words;};
  std::vector<Cb> cb;
  std::vector<Cb> srv;
  std::vector<Inline> inlines;
};
constexpr int kRootInfos = 4096;
RootInfo g_root_infos[kRootInfos];
std::atomic<int> g_root_info_n{0};
std::mutex g_root_info_mutex;

const RootInfo* root_info(ID3D12RootSignature* rs) {
  const int n = g_root_info_n.load(std::memory_order_acquire);
  for (int i = n - 1; i >= 0; --i) {
    if (g_root_infos[i].rs == rs) {
      return &g_root_infos[i];
    }
  }
  return nullptr;
}

void root_info_add(ID3D12RootSignature* rs, const void* blob, SIZE_T size) {
  ID3D12VersionedRootSignatureDeserializer* des = nullptr;
  if (FAILED(D3D12CreateVersionedRootSignatureDeserializer(blob, size, IID_PPV_ARGS(&des))) || des == nullptr) {
    return;
  }
  const D3D12_VERSIONED_ROOT_SIGNATURE_DESC* vd = nullptr;
  if (SUCCEEDED(des->GetRootSignatureDescAtVersion(D3D_ROOT_SIGNATURE_VERSION_1_1, &vd)) && vd != nullptr) {
    RootInfo info;
    info.rs = rs;
    if (g_scan_trace.load(std::memory_order_relaxed) && size <= 64 * 1024 && g_root_info_n.load() < 32) {
      const auto* bytes = static_cast<const unsigned char*>(blob);
      info.original_blob.assign(bytes, bytes + size);
    }
    const D3D12_ROOT_SIGNATURE_DESC1& d = vd->Desc_1_1;
    info.n = static_cast<int>(d.NumParameters < 32 ? d.NumParameters : 32);
    for (int p = 0; p < info.n; ++p) {
      const D3D12_ROOT_PARAMETER1& rp = d.pParameters[p];
      if(rp.ParameterType==D3D12_ROOT_PARAMETER_TYPE_CBV)
        info.cb.push_back({static_cast<UINT>(p),0,rp.Descriptor.ShaderRegister,1,rp.Descriptor.RegisterSpace,true});
      if(rp.ParameterType==D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS)
        info.inlines.push_back({static_cast<UINT>(p),rp.Constants.ShaderRegister,rp.Constants.RegisterSpace,rp.Constants.Num32BitValues});
      if (rp.ParameterType != D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE) {
        continue;
      }
      UINT end = 0, next = 0;
      bool sampler = false, unbounded = false;
      for (UINT r = 0; r < rp.DescriptorTable.NumDescriptorRanges; ++r) {
        const D3D12_DESCRIPTOR_RANGE1& range = rp.DescriptorTable.pDescriptorRanges[r];
        sampler = sampler || range.RangeType == D3D12_DESCRIPTOR_RANGE_TYPE_SAMPLER;
        const UINT start = range.OffsetInDescriptorsFromTableStart == D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND ? next : range.OffsetInDescriptorsFromTableStart;
        if(range.RangeType==D3D12_DESCRIPTOR_RANGE_TYPE_CBV)
          info.cb.push_back({static_cast<UINT>(p),start,range.BaseShaderRegister,range.NumDescriptors,range.RegisterSpace,false});
        if(range.RangeType==D3D12_DESCRIPTOR_RANGE_TYPE_SRV)
          info.srv.push_back({static_cast<UINT>(p),start,range.BaseShaderRegister,range.NumDescriptors,range.RegisterSpace,false});
        if (range.NumDescriptors == 0xFFFFFFFFu) {
          unbounded = true;
          next = start;
        } else {
          next = start + range.NumDescriptors;
          end = end > next ? end : next;
        }
      }
      info.kind[p] = sampler ? 2 : 1;
      info.extent[p] = unbounded ? 0xFFFFFFFFu : end;
    }
    std::lock_guard<std::mutex> lock(g_root_info_mutex);
    const int n = g_root_info_n.load();
    if (n < kRootInfos) {
      g_root_infos[n] = info;
      g_root_info_n.store(n + 1, std::memory_order_release);
    }
  }
  des->Release();
}

using CreateRootSigFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, UINT, const void*, SIZE_T, REFIID, void**);
CreateRootSigFn g_create_rootsig = nullptr;

HRESULT STDMETHODCALLTYPE CreateRootSig_hook(ID3D12Device* dev, UINT mask, const void* blob, SIZE_T size, REFIID riid, void** out) {
  const HRESULT hr = g_create_rootsig(dev, mask, blob, size, riid, out);
  if (SUCCEEDED(hr) && out != nullptr && *out != nullptr && riid == __uuidof(ID3D12RootSignature)) {
    root_info_add(static_cast<ID3D12RootSignature*>(*out), blob, size);
  }
  return hr;
}

// Per-thread bindings of the list being recorded: graphics root signature, descriptor heaps, compute root signature.
using SetHeapsFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, UINT, ID3D12DescriptorHeap* const*);
using SetRootSigFn = void(STDMETHODCALLTYPE*)(ID3D12GraphicsCommandList*, ID3D12RootSignature*);
SetHeapsFn g_set_heaps = nullptr;
SetRootSigFn g_set_gfx_rootsig = nullptr;
SetRootSigFn g_set_cs_rootsig = nullptr;
thread_local ID3D12GraphicsCommandList* t_bind_list = nullptr;
thread_local ID3D12DescriptorHeap* t_heaps[2] = {};
thread_local UINT t_heap_n = 0;
thread_local ID3D12RootSignature* t_gfx_rs = nullptr;
thread_local ID3D12RootSignature* t_cs_rs = nullptr;

void bind_list(ID3D12GraphicsCommandList* list) {
  if (t_bind_list != list) {
    t_bind_list = list;
    t_heap_n = 0;
    t_gfx_rs = nullptr;
    t_cs_rs = nullptr;
  }
}

void STDMETHODCALLTYPE SetHeaps_hook(ID3D12GraphicsCommandList* list, UINT n, ID3D12DescriptorHeap* const* heaps) {
  g_set_heaps(list, n, heaps);
  bind_list(list);
  t_heap_n = 0;
  for (UINT i = 0; heaps != nullptr && i < n && i < 2; ++i) {
    t_heaps[t_heap_n++] = heaps[i];
  }
}

void STDMETHODCALLTYPE SetGfxRootSig_hook(ID3D12GraphicsCommandList* list, ID3D12RootSignature* rs) {
  g_set_gfx_rootsig(list, rs);
  bind_list(list);
  if (rs != t_gfx_rs) {
    t_gfx_rs = rs;
    if (t_tables_list == list) {
      std::memset(t_tables, 0, sizeof(t_tables));  // a new root signature drops the old arguments
    }
    if(t_inputs_list==list) {std::memset(t_input_va,0,sizeof(t_input_va));std::memset(t_input_valid,0,sizeof(t_input_valid));}
  }
}

void STDMETHODCALLTYPE SetCsRootSig_hook(ID3D12GraphicsCommandList* list, ID3D12RootSignature* rs) {
  g_set_cs_rootsig(list, rs);
  bind_list(list);
  t_cs_rs = rs;
}

ULONGLONG g_inputs_armed=0;
std::atomic<ULONGLONG> g_inputs_started{0},g_inputs_bucket[2];
std::atomic<unsigned> g_inputs_records{0};
SIZE_T input_descriptor_cpu(ID3D12GraphicsCommandList* list,UINT64 gpu) {
  if(t_bind_list!=list||!g_srv_stride) return 0;
  for(UINT i=0;i<t_heap_n;++i) {
    if(!t_heaps[i]) continue;
    const auto d=t_heaps[i]->GetDesc();
    if(d.Type!=D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV) continue;
    const UINT64 start=t_heaps[i]->GetGPUDescriptorHandleForHeapStart().ptr;
    if(gpu>=start&&gpu-start<UINT64(d.NumDescriptors)*g_srv_stride&&(gpu-start)%g_srv_stride==0)
      return t_heaps[i]->GetCPUDescriptorHandleForHeapStart().ptr+static_cast<SIZE_T>(gpu-start);
  }
  return 0;
}
void scan_inputs_draw(ID3D12GraphicsCommandList* list,UINT count,UINT instances,UINT start,INT base,UINT first_instance) {
  if(!scanner_inputs::enabled.load(std::memory_order_relaxed)) return;
  const int eye=native_stereo::ui_pass_eye();
  if((eye!=1&&eye!=2)||native_stereo::ui_pass_kind()!=0) return;
  {std::lock_guard<std::mutex> lock(g_scan_uv_mutex);if(!g_scan_uv_psos.count(t_pso)) return;}
  const auto now=GetTickCount64();ULONGLONG zero=0;g_inputs_started.compare_exchange_strong(zero,now);
  if(now-g_inputs_started.load()>120000||g_inputs_records.load()>=120) {scanner_inputs::enabled.store(false);return;}
  const auto bucket=now/1000;
  if(g_inputs_bucket[eye-1].exchange(bucket)==bucket) return;
  const unsigned id=g_inputs_records.fetch_add(1);
  if(id>=120) return;
  const RootInfo* ri=t_bind_list==list?root_info(t_gfx_rs):nullptr;
  const auto frame=upload::present_sequence();SYSTEMTIME st;GetLocalTime(&st);
  logf("scaninputs: id=%u eye=%d present=%llu bucket=%llu at=%02u:%02u:%02u.%03u pso=%p list=%p root=%p known=%d",
       id,eye,frame,bucket,st.wHour,st.wMinute,st.wSecond,st.wMilliseconds,t_pso,list,t_gfx_rs,ri!=nullptr);
  if(!ri) return;
  const std::wstring dir=parent_dir(g_log_path)+L"\\scaninputs_"+std::to_wstring(GetCurrentProcessId());
  CreateDirectoryW(dir.c_str(),nullptr);
  const auto save=[&](UINT reg,UINT space,const void* data,UINT bytes,UINT64 va) {
    const std::wstring path=dir+L"\\s"+std::to_wstring(id)+L"_e"+std::to_wstring(eye)+L"_b"+std::to_wstring(reg)+L"_space"+std::to_wstring(space)+L".bin";
    HANDLE f=CreateFileW(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
    DWORD wrote=0;BOOL ok=FALSE;
    if(f!=INVALID_HANDLE_VALUE) {ok=WriteFile(f,data,bytes,&wrote,nullptr);CloseHandle(f);}
    logf("scaninputs: id=%u b%u space=%u va=%llx bytes=%u saved=%d file=%ls",id,reg,space,va,bytes,ok&&wrote==bytes,path.c_str());
  };
  const auto save_geometry=[&](const std::wstring& suffix,UINT64 va,UINT bytes) {
    unsigned char data[16384];
    const bool mapped=bytes&&bytes<=sizeof(data)&&upload::copy_mapped(va,bytes,data);
    const auto path=dir+L"\\s"+std::to_wstring(id)+L"_e"+std::to_wstring(eye)+L"_"+suffix+L".bin";
    DWORD wrote=0;BOOL ok=FALSE;
    if(mapped) {
      HANDLE f=CreateFileW(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
      if(f!=INVALID_HANDLE_VALUE) {ok=WriteFile(f,data,bytes,&wrote,nullptr);CloseHandle(f);}
    }
    logf("scangeometry: id=%u part=%ls va=%llx bytes=%u mapped=%d saved=%d file=%ls",id,suffix.c_str(),va,bytes,mapped,ok&&wrote==bytes,path.c_str());
    return ok&&wrote==bytes;
  };
  const auto ia=scanner_geometry::snapshot(list);
  logf("scangeometry: id=%u count=%u instances=%u start=%u base=%d first_instance=%u reset_seen=%d ib_known=%d ib_format=%u",
    id,count,instances,start,base,first_instance,ia.reset_seen,ia.index,UINT(ia.ib.Format));
  // Decode only a bounded prefix of this indexed draw. The explicit reset receipt
  // prevents stale binding reuse when recording restarts on the same list pointer.
  UINT used[16]={},used_n=0;
  const UINT index_bytes=ia.ib.Format==DXGI_FORMAT_R16_UINT?2:ia.ib.Format==DXGI_FORMAT_R32_UINT?4:0;
  const UINT take=count<48?count:48;
  unsigned char indices[48*4]={};
  const UINT64 index_offset=UINT64(start)*index_bytes;
  if(ia.reset_seen&&ia.index&&index_bytes&&take&&index_offset<=ia.ib.SizeInBytes&&
     UINT64(take)*index_bytes<=ia.ib.SizeInBytes-index_offset&&
     ia.ib.BufferLocation<=~UINT64(0)-index_offset&&
     upload::copy_mapped(ia.ib.BufferLocation+index_offset,take*index_bytes,indices)) {
    save_geometry(L"indices",ia.ib.BufferLocation+index_offset,take*index_bytes);
    for(UINT k=0;k<take;++k) {
      UINT value=0;std::memcpy(&value,indices+k*index_bytes,index_bytes);
      const INT64 vertex=INT64(value)+base;
      if(vertex<0||vertex>UINT_MAX) continue;
      bool seen=false;for(UINT j=0;j<used_n;++j) seen|=used[j]==UINT(vertex);
      if(!seen&&used_n<16) used[used_n++]=UINT(vertex);
    }
  }
  {
    std::lock_guard<std::mutex> lock(g_scan_uv_mutex);
    const auto it=g_scan_uv_psos.find(t_pso);
    if(it!=g_scan_uv_psos.end()) {
      UINT slots=0;
      for(const auto& el:it->second->saved.elements) {
        logf("scangeometry: id=%u layout=%s semantic=%u slot=%u offset=%u format=%u class=%u step=%u",id,
          el.SemanticName,el.SemanticIndex,el.InputSlot,el.AlignedByteOffset,UINT(el.Format),UINT(el.InputSlotClass),el.InstanceDataStepRate);
        if(el.InputSlot<32) slots|=UINT(1)<<el.InputSlot;
      }
      for(UINT slot=0;slot<32;++slot) if(slots&(UINT(1)<<slot)) {
        const auto& vb=ia.vb[slot];
        logf("scangeometry: id=%u slot=%u bound=%d stride=%u size=%u",id,slot,!!(ia.valid&(UINT(1)<<slot)),vb.StrideInBytes,vb.SizeInBytes);
        // Per-instance streams are reported but not interpreted as per-vertex.
        bool instance=false;for(const auto& el:it->second->saved.elements)
          if(el.InputSlot==slot&&el.InputSlotClass==D3D12_INPUT_CLASSIFICATION_PER_INSTANCE_DATA) instance=true;
        if(instance||!ia.reset_seen||!(ia.valid&(UINT(1)<<slot))||!vb.StrideInBytes||vb.StrideInBytes>256) continue;
        for(UINT k=0;k<used_n;++k) {
          const UINT64 offset=UINT64(used[k])*vb.StrideInBytes;
          if(offset<=vb.SizeInBytes&&vb.StrideInBytes<=vb.SizeInBytes-offset&&vb.BufferLocation<=~UINT64(0)-offset)
            save_geometry(L"v"+std::to_wstring(used[k])+L"_slot"+std::to_wstring(slot),vb.BufferLocation+offset,vb.StrideInBytes);
        }
      }
    }
  }
  // Union of captured VS/PS declarations (UI block includes VS row 22;
  // space-2 material block includes VS raw-transform index at row 7).
  const UINT regs[]={2200,2203,2240,2260,2380,0},spaces[]={0,0,0,0,0,2},sizes[]={368,80,1168,512,32,128};
  UINT material[32]={};bool material_valid=false;
  for(unsigned k=0;k<6;++k) {
    bool found=false;
    for(const auto& r:ri->inlines) {
      if(r.reg!=regs[k]||r.space!=spaces[k]||t_inputs_list!=list||r.words>64) continue;
      const UINT needed=sizes[k]/4;
      const UINT64 mask=needed==64?~UINT64(0):(needed<64?(UINT64(1)<<needed)-1:0);
      if(needed<=r.words&&mask&&(t_input_valid[r.root]&mask)==mask) {
        save(regs[k],spaces[k],t_input_words[r.root],sizes[k],0);found=true;
        if(k==5) {std::memcpy(material,t_input_words[r.root],128);material_valid=true;}
        break;
      }
    }
    if(found) continue;
    for(const auto& r:ri->cb) {
      if(r.space!=spaces[k]||regs[k]<r.reg||regs[k]-r.reg>=r.count) continue;
      scanner_inputs::View view{};
      if(r.direct) {if(t_inputs_list==list) view={t_input_va[r.root],sizes[k]};}
      else if(t_tables_list==list&&t_tables[r.root]) {
        const UINT64 offset=UINT64(r.start)+regs[k]-r.reg;
        const auto cpu=input_descriptor_cpu(list,t_tables[r.root]+offset*g_srv_stride);
        view=scanner_inputs::get(cpu);
      }
      unsigned char copy[1536]={};
      if(view.address&&view.bytes>=sizes[k]&&upload::copy_constants(view.address,sizes[k],copy)) {
        save(regs[k],spaces[k],copy,sizes[k],view.address);found=true;
        if(k==5) {std::memcpy(material,copy,128);material_valid=true;}
      } else logf("scaninputs: id=%u b%u space=%u unresolved va=%llx available=%u need=%u",id,regs[k],spaces[k],view.address,view.bytes,sizes[k]);
      break;
    }
    if(!found) logf("scaninputs: id=%u b%u space=%u NOT_CAPTURED",id,regs[k],spaces[k]);
  }
  const auto srv=[&](UINT reg,UINT space) {
    for(const auto& r:ri->srv) {
      if(r.space!=space||reg<r.reg||reg-r.reg>=r.count||t_tables_list!=list||!t_tables[r.root]) continue;
      const auto cpu=input_descriptor_cpu(list,t_tables[r.root]+(UINT64(r.start)+reg-r.reg)*g_srv_stride);
      const auto view=scanner_inputs::get(cpu);
      logf("scaninputs: id=%u t%u space=%u cpu=%llx resource_id=%llx view_tag=%llx known_srv=%d",id,reg,space,UINT64(cpu),view.address,view.tag,view.address&&view.bytes==0);
      if(space==2500&&view.address&&view.bytes==0) {
        logf("scangeometry: id=%u raw_heap=%u raw_map_hr=%08X raw_at_create=%d",id,view.heap_type,view.map_result,view.raw_saved);
        // VS byte-normalized index selects at most 256 matrices of 64 bytes.
        const UINT bytes=static_cast<UINT>(view.raw_bytes<16384?view.raw_bytes:16384);
        if(bytes&&save_geometry(L"transforms",view.raw_va,bytes)) return;
        if(view.raw_saved&&view.raw_bytes==64) {
          const auto path=dir+L"\\s"+std::to_wstring(id)+L"_e"+std::to_wstring(eye)+L"_transforms.bin";
          HANDLE f=CreateFileW(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
          DWORD wrote=0;BOOL ok=FALSE;
          if(f!=INVALID_HANDLE_VALUE) {ok=WriteFile(f,view.raw64.data(),64,&wrote,nullptr);CloseHandle(f);}
          logf("scangeometry: id=%u part=transforms source=CreateSRV capture_present=%llu bytes=64 saved=%d file=%ls",id,view.raw_present,ok&&wrote==64,path.c_str());
        }
      }
      return;
    }
    logf("scaninputs: id=%u t%u space=%u NOT_BOUND",id,reg,space);
  };
  for(UINT reg:{640,660,680,681,682,860}) srv(reg,0);
  if(material_valid) {srv(material[1],581);srv(material[2],582);srv(material[3],583);srv(material[30],2500);}
}

UINT vis_remaining(SIZE_T cpu);
// Bounded CPU-only snapshot of live bindings, before any of our scan routing.
// Shader files are the original creation bytes, never replacements.
void scan_binding_trace(ID3D12GraphicsCommandList* list, UINT count, bool indexed) {
  if (!g_scan_trace.load(std::memory_order_relaxed)) return;
  if(g_fx_trace) {
    if(native_stereo::front_end()||t_depth_bound||count>6) return;
  } else if(!scan_active()) return;
  const int eye = native_stereo::ui_pass_eye();
  if (eye != 1 && eye != 2) return;
  static std::atomic<ULONGLONG> started{0};
  ULONGLONG zero = 0;
  const ULONGLONG now = GetTickCount64();
  started.compare_exchange_strong(zero, now);
  if (now - started.load() > (g_fx_trace?600000:120000)) return;
  static unsigned records = 0;
  struct Key { const void* pso; int eye, kind; bool indexed; };
  static Key keys[128] = {};
  const int kind = native_stereo::ui_pass_kind();
  std::unique_lock<std::mutex> lock(g_scan_shader_mutex, std::try_to_lock);
  if (!lock || records >= (g_fx_trace?128u:64u)) return;
  for (unsigned i = 0; i < records; ++i)
    if (keys[i].pso == t_pso && keys[i].eye == eye && keys[i].kind == kind && keys[i].indexed == indexed) return;
  keys[records++] = {t_pso, eye, kind, indexed};
  const RootInfo* ri = t_bind_list == list ? root_info(t_gfx_rs) : nullptr;
  logf("scantrace: #%u eye=%d kind=%d list=%p pso=%p %c%u root=%p root_known=%d bind_list_match=%d tables_match=%d vp=%.0fx%.0f mapped=%d depth=%d",
       records, eye, kind, list, t_pso, indexed ? 'I' : 'D', count, t_gfx_rs, ri != nullptr,
       t_bind_list == list, t_tables_list == list, t_vp.Width, t_vp.Height, t_mapped_now, t_depth_bound);
  if (t_tables_list == list && t_bind_list == list) {
    for (unsigned p = 0; p < 32; ++p) {
      if (!t_tables[p]) continue;
      SIZE_T cpu = 0;
      UINT remaining = 0;
      // Use the currently bound heaps, not the historical 16-entry heap census
      // (the supplied run exhausted that census). These are live draw bindings.
      for (UINT hi = 0; hi < t_heap_n; ++hi) {
        if (!t_heaps[hi]) continue;
        const auto hd = t_heaps[hi]->GetDesc();
        if (hd.Type != D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV || !g_srv_stride) continue;
        const UINT64 start = t_heaps[hi]->GetGPUDescriptorHandleForHeapStart().ptr;
        if (t_tables[p] < start) continue;
        const UINT64 offset = t_tables[p] - start;
        if (offset >= static_cast<UINT64>(hd.NumDescriptors) * g_srv_stride || offset % g_srv_stride) continue;
        cpu = t_heaps[hi]->GetCPUDescriptorHandleForHeapStart().ptr + static_cast<SIZE_T>(offset);
        remaining = hd.NumDescriptors - static_cast<UINT>(offset / g_srv_stride);
        break;
      }
      const UINT extent = ri && p < static_cast<unsigned>(ri->n) ? ri->extent[p] : 0;
      logf("scantrace: table=%u gpu=%llx cpu=%llx remaining=%u declared=%u", p,
           static_cast<unsigned long long>(t_tables[p]), static_cast<unsigned long long>(cpu), remaining, extent);
      // Metadata only; no descriptor reads outside the observed heap. Unknown
      // signatures get no speculative descriptor-table traversal.
      if (!ri || p >= static_cast<unsigned>(ri->n) || ri->kind[p] != 1) continue;
      const UINT n = (extent < 32 ? extent : 32) < remaining ? (extent < 32 ? extent : 32) : remaining;
      for (UINT k = 0; k < n; ++k) {
        ID3D12Resource* r = srv_resource(cpu + static_cast<SIZE_T>(k) * g_srv_stride);
        if (!r) continue;
        const int target = scan_target_index(r);
        // Resource identity only: tracked pointers may describe recycled slots.
        // Do NOT call GetDesc on them or assume ownership from a pointer match.
        logf("scantrace: table=%u offset=%u resource=%p scan_target=%d", p, k, r, target);
      }
    }
  }
  const auto it = g_scan_shaders.find(t_pso);
  if (it == g_scan_shaders.end()) {
    logf("scantrace: shader bytes unavailable (not captured, unsupported format/path, or budget reached); retained=%zu bytes=%zu",
         g_scan_shaders.size(), g_scan_shader_bytes);
    return;
  }
  ScanShader& shader = it->second;
  logf("scantrace: pso_root=%p bound_root=%p root_equal=%d format=%u VS=%zu PS=%zu",
       shader.root, t_gfx_rs, shader.root == t_gfx_rs, static_cast<unsigned>(shader.format), shader.vs.size(), shader.ps.size());
  if (shader.seen++) return;
  const std::wstring dir = parent_dir(g_log_path) + L"\\scantrace_" + std::to_wstring(GetCurrentProcessId());
  CreateDirectoryW(dir.c_str(), nullptr);
  const auto save = [&](const wchar_t* suffix, const void* data, size_t size) {
    const std::wstring path = dir + L"\\pso_" + std::to_wstring(reinterpret_cast<uintptr_t>(t_pso)) + suffix;
    HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) { logf("scantrace: file open failed error=%lu", GetLastError()); return; }
    DWORD wrote = 0;
    const BOOL ok = WriteFile(file, data, static_cast<DWORD>(size), &wrote, nullptr);
    CloseHandle(file);
    logf("scantrace: saved %ls bytes=%lu ok=%d", path.c_str(), wrote, ok && wrote == size);
  };
  if (!shader.vs.empty()) save(L"_vs.cso", shader.vs.data(), shader.vs.size());
  save(L"_ps.cso", shader.ps.data(), shader.ps.size());
  if (ri && !ri->original_blob.empty()) save(L"_rootsig.blob", ri->original_blob.data(), ri->original_blob.size());
  ID3DBlob* assembly = nullptr;
  if (SUCCEEDED(D3DDisassemble(shader.ps.data(), shader.ps.size(), 0, nullptr, &assembly))) {
    save(L"_ps.asm", assembly->GetBufferPointer(), assembly->GetBufferSize());
    assembly->Release();
  }
}

struct OverlayTex {
  ID3D12Resource* src = nullptr;  // game scan target this one shadows
  ID3D12Resource* tex = nullptr;  // ours, same size / format, UAV + simultaneous access
  DXGI_FORMAT fmt = DXGI_FORMAT_UNKNOWN;
  UINT64 w = 0;
  UINT h = 0;
};
struct Overlay {
  std::mutex m;
  bool failed = false;
  ID3D12DescriptorHeap* heap = nullptr;
  UINT heap_n = 1 << 18;
  UINT stride = 0;
  ID3D12RootSignature* rs = nullptr;
  ID3D12PipelineState* cs = nullptr;
  OverlayTex tex[8];
  int tex_n = 0;
};
Overlay g_ov;
std::atomic<UINT> g_ov_ring{0};
std::atomic<bool> g_ov_on{false};  // dormant experiment: explicit scanoverlay only, never armed by a probe
std::atomic<std::uint64_t> g_ov_draws{0}, g_ov_skip_root{0}, g_ov_skip_unbounded{0}, g_ov_skip_other{0};

const char kOverlayCs[] = R"(
Texture2D<float4> src : register(t0);
RWTexture2D<float4> dst : register(u0);
cbuffer C : register(b0) { int dx; int w; int h; };
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if ((int)id.x >= w || (int)id.y >= h) return;
  const int sx = (int)id.x - dx;
  dst[id.xy] = (sx >= 0 && sx < w) ? src.Load(int3(sx, id.y, 0)) : float4(0, 0, 0, 0);
}
)";

bool ov_init(ID3D12Device* dev) {
  if (g_ov.heap != nullptr) {
    return true;
  }
  if (g_ov.failed) {
    return false;
  }
  g_ov.failed = true;  // until everything below succeeds
  D3D12_DESCRIPTOR_HEAP_DESC hd = {};
  hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
  hd.NumDescriptors = g_ov.heap_n;
  hd.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
  ID3D12DescriptorHeap* heap = nullptr;
  if (FAILED(g_create_heap != nullptr ? g_create_heap(dev, &hd, IID_PPV_ARGS(&heap)) : dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&heap)))) {
    logf("scan overlay: descriptor heap failed");
    return false;
  }
  ID3DBlob* code = nullptr;
  ID3DBlob* err = nullptr;
  if (FAILED(D3DCompile(kOverlayCs, sizeof(kOverlayCs) - 1, "scan_overlay", nullptr, nullptr, "main", "cs_5_0", 0, 0, &code, &err))) {
    logf("scan overlay: shader compile failed: %s", err != nullptr ? static_cast<const char*>(err->GetBufferPointer()) : "?");
    if (err != nullptr) err->Release();
    heap->Release();
    return false;
  }
  D3D12_DESCRIPTOR_RANGE ranges[2] = {};
  ranges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
  ranges[0].NumDescriptors = 1;
  ranges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
  ranges[1].NumDescriptors = 1;
  ranges[1].OffsetInDescriptorsFromTableStart = 1;
  D3D12_ROOT_PARAMETER params[2] = {};
  params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
  params[0].DescriptorTable.NumDescriptorRanges = 2;
  params[0].DescriptorTable.pDescriptorRanges = ranges;
  params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
  params[1].Constants.Num32BitValues = 3;
  D3D12_ROOT_SIGNATURE_DESC rd = {};
  rd.NumParameters = 2;
  rd.pParameters = params;
  ID3DBlob* sig = nullptr;
  if (FAILED(D3D12SerializeRootSignature(&rd, D3D_ROOT_SIGNATURE_VERSION_1, &sig, &err)) || sig == nullptr ||
      FAILED(dev->CreateRootSignature(0, sig->GetBufferPointer(), sig->GetBufferSize(), IID_PPV_ARGS(&g_ov.rs)))) {
    logf("scan overlay: root signature failed");
    if (sig != nullptr) sig->Release();
    if (err != nullptr) err->Release();
    code->Release();
    heap->Release();
    return false;
  }
  sig->Release();
  D3D12_COMPUTE_PIPELINE_STATE_DESC pd = {};
  pd.pRootSignature = g_ov.rs;
  pd.CS = {code->GetBufferPointer(), code->GetBufferSize()};
  const HRESULT hr = dev->CreateComputePipelineState(&pd, IID_PPV_ARGS(&g_ov.cs));
  code->Release();
  if (FAILED(hr)) {
    logf("scan overlay: compute PSO failed hr=0x%08lX", static_cast<unsigned long>(hr));
    heap->Release();
    return false;
  }
  heap->SetName(L"AvatarVR_ScanOverlayHeap");
  g_ov.stride = dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
  g_ov.heap = heap;
  g_ov.failed = false;
  logf("scan overlay: ready (heap %u descriptors)", g_ov.heap_n);
  return true;
}

OverlayTex* ov_tex(ID3D12Device* dev, ID3D12Resource* src) {
  for (int i = 0; i < g_ov.tex_n; ++i) {
    if (g_ov.tex[i].src == src) {
      return &g_ov.tex[i];
    }
  }
  if (g_ov.tex_n >= 8) {
    return nullptr;
  }
  const D3D12_RESOURCE_DESC sd = src->GetDesc();
  D3D12_FEATURE_DATA_FORMAT_SUPPORT fs = {sd.Format, {}, {}};
  if (sd.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || FAILED(dev->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &fs, sizeof(fs))) ||
      (fs.Support2 & D3D12_FORMAT_SUPPORT2_UAV_TYPED_STORE) == 0) {
    logf("scan overlay: target %p format %d cannot be a typed UAV; left alone", static_cast<void*>(src), static_cast<int>(sd.Format));
    return nullptr;
  }
  D3D12_RESOURCE_DESC d = {};
  d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  d.Width = sd.Width;
  d.Height = sd.Height;
  d.DepthOrArraySize = 1;
  d.MipLevels = 1;
  d.Format = sd.Format;
  d.SampleDesc.Count = 1;
  d.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS;
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_DEFAULT;
  OverlayTex& t = g_ov.tex[g_ov.tex_n];
  if (FAILED(dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&t.tex)))) {
    t.tex = nullptr;
    return nullptr;
  }
  t.tex->SetName(L"AvatarVR_ScanOverlay");
  t.src = src;
  t.fmt = sd.Format;
  t.w = sd.Width;
  t.h = sd.Height;
  ++g_ov.tex_n;
  logf("scan overlay: shadow texture for %p %llux%u fmt %d", static_cast<void*>(src), static_cast<unsigned long long>(t.w), t.h, static_cast<int>(t.fmt));
  return &t;
}

D3D12_CPU_DESCRIPTOR_HANDLE ov_cpu(UINT i) { return {g_ov.heap->GetCPUDescriptorHandleForHeapStart().ptr + static_cast<SIZE_T>(i) * g_ov.stride}; }
D3D12_GPU_DESCRIPTOR_HANDLE ov_gpu(UINT i) { return {g_ov.heap->GetGPUDescriptorHandleForHeapStart().ptr + static_cast<UINT64>(i) * g_ov.stride}; }

// Remaining descriptors from a CPU handle to the end of its shader-visible heap (0 = not in one).
UINT vis_remaining(SIZE_T cpu) {
  const int n = g_vis_heap_n.load(std::memory_order_acquire);
  for (int i = 0; i < n; ++i) {
    const VisHeap& h = g_vis_heaps[i];
    if (cpu >= h.cpu && cpu < h.cpu + static_cast<SIZE_T>(h.count) * g_srv_stride) {
      return h.count - static_cast<UINT>((cpu - h.cpu) / g_srv_stride);
    }
  }
  return 0;
}

thread_local bool t_ov_active = false;
thread_local OverlayTex* t_ov_used[4] = {};
thread_local int t_ov_used_n = 0;

// Before the second view's scan composite draw. True when the draw is redirected (ov_post must follow).
bool ov_pre(ID3D12GraphicsCommandList* list) {
  t_ov_active = false;
  t_ov_used_n = 0;
  if (!g_ov_on.load(std::memory_order_relaxed) || t_tables_list != list || t_bind_list != list || t_heap_n == 0 || g_srv_stride == 0) {
    return false;
  }
  const RootInfo* ri = t_gfx_rs != nullptr ? root_info(t_gfx_rs) : nullptr;
  if (ri == nullptr) {
    g_ov_skip_root.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Which tables reference scan targets, and are all resource tables bounded and relocatable?
  struct Hit {
    int p;
    UINT k;
    ID3D12Resource* res;
  } hits[8];
  int nh = 0;
  UINT need = 0;
  for (int p = 0; p < ri->n; ++p) {
    if (ri->kind[p] != 1 || t_tables[p] == 0) {
      continue;
    }
    if (ri->extent[p] == 0xFFFFFFFFu || ri->extent[p] > 4096) {
      g_ov_skip_unbounded.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    const SIZE_T cpu = gpu_to_cpu(t_tables[p]);
    if (cpu == 0 || vis_remaining(cpu) < ri->extent[p]) {
      g_ov_skip_other.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    need += ri->extent[p];
    for (UINT k = 0; k < ri->extent[p] && nh < 8; ++k) {
      ID3D12Resource* r = srv_resource(cpu + static_cast<SIZE_T>(k) * g_srv_stride);
      if (r != nullptr && scan_target_index(r) >= 0) {
        hits[nh++] = {p, k, r};
      }
    }
  }
  if (nh == 0) {
    return false;  // not a composite that reads the main view's scan targets
  }
  const float eye_w = static_cast<float>(g_backbuffer_w.load()), eye_h = static_cast<float>(g_backbuffer_h.load());
  float ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  if (!native_stereo::scan_offsets(eye_w, eye_h, &ox, &oy, &sx, &sy)) {
    return false;
  }
  ID3D12Device* dev = nullptr;
  if (FAILED(list->GetDevice(IID_PPV_ARGS(&dev))) || dev == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(g_ov.m);
  if (!ov_init(dev)) {
    dev->Release();
    return false;
  }
  OverlayTex* tex_for_hit[8] = {};
  for (int i = 0; i < nh; ++i) {
    tex_for_hit[i] = ov_tex(dev, hits[i].res);
    if (tex_for_hit[i] == nullptr) {
      dev->Release();
      g_ov_skip_other.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
  }
  // Ring: 2 descriptors per hit for the compute pass, then the relocated tables.
  need += 2 * static_cast<UINT>(nh);
  UINT base = g_ov_ring.fetch_add(need, std::memory_order_relaxed) % g_ov.heap_n;
  if (base + need > g_ov.heap_n) {
    base = 0;
    g_ov_ring.store(need, std::memory_order_relaxed);
  }
  // 1) compute: scan target -> our texture, shifted.
  ID3D12DescriptorHeap* ours = g_ov.heap;
  g_set_heaps(list, 1, &ours);
  if (g_set_cs_rootsig != nullptr) {
    g_set_cs_rootsig(list, g_ov.rs);
  } else {
    list->SetComputeRootSignature(g_ov.rs);
  }
  g_set_pso(list, g_ov.cs);
  for (int i = 0; i < nh; ++i) {
    OverlayTex& t = *tex_for_hit[i];
    bool done = false;
    for (int j = 0; j < t_ov_used_n; ++j) {
      done = done || t_ov_used[j] == &t;
    }
    if (done) {
      continue;
    }
    D3D12_SHADER_RESOURCE_VIEW_DESC sv = {};
    sv.Format = t.fmt;
    sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sv.Texture2D.MipLevels = 1;
    g_create_srv != nullptr ? g_create_srv(dev, hits[i].res, &sv, ov_cpu(base + 2 * i)) : dev->CreateShaderResourceView(hits[i].res, &sv, ov_cpu(base + 2 * i));
    D3D12_UNORDERED_ACCESS_VIEW_DESC uv = {};
    uv.Format = t.fmt;
    uv.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    dev->CreateUnorderedAccessView(t.tex, nullptr, &uv, ov_cpu(base + 2 * i + 1));
    barrier_on(list, t.tex, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    list->SetComputeRootDescriptorTable(0, ov_gpu(base + 2 * i));
    const int consts[3] = {static_cast<int>(std::lround(ox * static_cast<float>(t.w) / eye_w)), static_cast<int>(t.w), static_cast<int>(t.h)};
    list->SetComputeRoot32BitConstants(1, 3, consts, 0);
    (g_dispatch != nullptr ? g_dispatch : nullptr) != nullptr ? g_dispatch(list, static_cast<UINT>((t.w + 7) / 8), (t.h + 7) / 8, 1)
                                                               : list->Dispatch(static_cast<UINT>((t.w + 7) / 8), (t.h + 7) / 8, 1);
    barrier_on(list, t.tex, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    t_ov_used[t_ov_used_n++] = &t;
  }
  // 2) relocate the composite's resource tables into our heap, scan targets replaced by our textures.
  UINT at = base + 2 * static_cast<UINT>(nh);
  UINT64 newgpu[32] = {};
  for (int p = 0; p < ri->n; ++p) {
    if (ri->kind[p] != 1 || t_tables[p] == 0) {
      continue;
    }
    const SIZE_T cpu = gpu_to_cpu(t_tables[p]);
    g_copy_desc_simple != nullptr
        ? g_copy_desc_simple(dev, ri->extent[p], ov_cpu(at), {cpu}, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV)
        : dev->CopyDescriptorsSimple(ri->extent[p], ov_cpu(at), {cpu}, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    for (int i = 0; i < nh; ++i) {
      if (hits[i].p != p) {
        continue;
      }
      D3D12_SHADER_RESOURCE_VIEW_DESC sv = {};
      sv.Format = tex_for_hit[i]->fmt;
      sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
      sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
      sv.Texture2D.MipLevels = 1;
      dev->CreateShaderResourceView(tex_for_hit[i]->tex, &sv, ov_cpu(at + hits[i].k));
    }
    newgpu[p] = ov_gpu(at).ptr;
    at += ri->extent[p];
  }
  dev->Release();
  // 3) bind for the draw: our resource heap with the game's sampler heap, the game's PSO, tables re-set.
  ID3D12DescriptorHeap* heaps[2] = {ours, nullptr};
  UINT nheaps = 1;
  for (UINT i = 0; i < t_heap_n; ++i) {
    if (t_heaps[i] != nullptr && t_heaps[i]->GetDesc().Type == D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER) {
      heaps[nheaps++] = t_heaps[i];
    }
  }
  g_set_heaps(list, nheaps, heaps);
  g_set_pso(list, t_pso);
  for (int p = 0; p < ri->n; ++p) {
    if (ri->kind[p] == 1 && newgpu[p] != 0) {
      g_set_table(list, static_cast<UINT>(p), {newgpu[p]});
    } else if (ri->kind[p] == 2 && t_tables[p] != 0) {
      g_set_table(list, static_cast<UINT>(p), {t_tables[p]});  // sampler heap unchanged; re-set after the heap switch
    }
  }
  t_ov_active = true;
  g_ov_draws.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void ov_post(ID3D12GraphicsCommandList* list) {
  if (!t_ov_active) {
    return;
  }
  t_ov_active = false;
  g_set_heaps(list, t_heap_n, t_heaps);
  const RootInfo* ri = t_gfx_rs != nullptr ? root_info(t_gfx_rs) : nullptr;
  for (int p = 0; ri != nullptr && p < ri->n; ++p) {
    if (ri->kind[p] != 0 && t_tables[p] != 0) {
      g_set_table(list, static_cast<UINT>(p), {t_tables[p]});
    }
  }
  if (t_cs_rs != nullptr && g_set_cs_rootsig != nullptr) {
    g_set_cs_rootsig(list, t_cs_rs);  // the game's compute root signature (its arguments are not restored)
  }
  for (int i = 0; i < t_ov_used_n; ++i) {
    barrier_on(list, t_ov_used[i]->tex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COMMON);
  }
  t_ov_used_n = 0;
}

void STDMETHODCALLTYPE CreateRtv_hook(ID3D12Device* dev, ID3D12Resource* r, const D3D12_RENDER_TARGET_VIEW_DESC* desc, D3D12_CPU_DESCRIPTOR_HANDLE h) {
  g_create_rtv(dev, r, desc, h);
  rtv_note(h.ptr, r);
}

void STDMETHODCALLTYPE CopyDescSimple_hook(ID3D12Device* dev, UINT n, D3D12_CPU_DESCRIPTOR_HANDLE dst, D3D12_CPU_DESCRIPTOR_HANDLE src,
                                           D3D12_DESCRIPTOR_HEAP_TYPE type) {
  g_copy_desc_simple(dev, n, dst, src, type);
  if(scanner_inputs::enabled.load(std::memory_order_relaxed)&&type==D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV&&g_srv_stride)
    for(UINT i=0;i<n&&i<4096;++i) scanner_inputs::copy(dst.ptr+static_cast<SIZE_T>(i)*g_srv_stride,src.ptr+static_cast<SIZE_T>(i)*g_srv_stride);
  if (type == D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV && g_srv_stride != 0 && g_scan_target_n.load(std::memory_order_relaxed) > 0) {
    for (UINT i = 0; i < n && i < 4096; ++i) {
      srv_copy_one(dst.ptr + static_cast<SIZE_T>(i) * g_srv_stride, src.ptr + static_cast<SIZE_T>(i) * g_srv_stride);
    }
  }
  if (type == D3D12_DESCRIPTOR_HEAP_TYPE_RTV && g_rtv_stride != 0) {
    for (UINT i = 0; i < n && i < 64; ++i) {
      if (ID3D12Resource* r = rtv_resource(src.ptr + static_cast<SIZE_T>(i) * g_rtv_stride)) {
        rtv_note(dst.ptr + static_cast<SIZE_T>(i) * g_rtv_stride, r);
      }
    }
  }
}

void STDMETHODCALLTYPE SetPipelineState_hook(ID3D12GraphicsCommandList* list, ID3D12PipelineState* pso) {
  ID3D12PipelineState* bind = pso;
  if (t_layer_list == list && pso != nullptr && g_panel_cover.load(std::memory_order_relaxed)) {
    if (ID3D12PipelineState* clone = hud_alpha_clone(list, pso)) {
      bind = clone;  // panel layer: accumulate coverage in alpha
      g_panel_cover_binds.fetch_add(1, std::memory_order_relaxed);
    }
  }
  g_set_pso(list, bind);
  t_pso = pso;
  t_pso_list = list;
  t_layer_clone = t_layer_list == list && bind != pso;
  render_activity::note_pso(list, pso);
}

void STDMETHODCALLTYPE Dispatch_hook(ID3D12GraphicsCommandList* list, UINT x, UINT y, UINT z) {
  render_activity::note(list,1,x,y,z);
  const UINT stamp = render_activity::stamp_before(list, x, y, z, false);  // census only
  g_dispatch(list, x, y, z);
  render_activity::stamp_after(list, stamp);
  if (const int eye = native_stereo::ui_pass_eye()) {
    g_ui[eye].dispatches.fetch_add(1, std::memory_order_relaxed);
  }
}

void STDMETHODCALLTYPE Scissors_hook(ID3D12GraphicsCommandList* list, UINT count, const D3D12_RECT* rects) {
  const int eye = native_stereo::ui_pass_eye();
  if (eye == 0 && rects != nullptr && count > 0) {
    note_rect('s', static_cast<float>(rects[0].left), static_cast<float>(rects[0].top), static_cast<float>(rects[0].right),
              static_cast<float>(rects[0].bottom), _ReturnAddress());
  }
  float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  if (eye != 0 && !t_odd_vp && !(scan_active() && native_stereo::ui_pass_kind() == 1) && rects != nullptr && count > 0 &&
      count <= D3D12_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE && ui_target(&w, &h) &&
      native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy,
          t_layer_list == list ? t_panel_source_w : 0.f, t_layer_list == list ? t_panel_source_h : 0.f)) {
    D3D12_RECT mapped[D3D12_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
    const float k = t_map_scale;
    for (UINT i = 0; i < count; ++i) {
      mapped[i] = map_rect(t_layer_list == list ? rects[i] : band_cut(rects[i], w, h, k), ox * k, oy * k, sx, sy, w * k, h * k);
    }
    g_scissors(list, count, mapped);
    t_placed_list = list;
    t_placed_sc = true;
  } else {
    g_scissors(list, count, rects);
  }
  if (rects != nullptr && count > 0) {
    t_sc_list = list;
    t_sc = rects[0];
  }
  if (eye != 0) {
    g_ui[eye].scissors.fetch_add(1, std::memory_order_relaxed);
    if (rects != nullptr && count > 0) {
      std::lock_guard<std::mutex> lock(g_ui_mutex);
      g_ui_sc[eye] = rects[0];
      g_ui_sc_seen[eye] = true;
    }
  }
}

// Second view's UI pass, scan running, before its first target bind: shift every scan target by the right-eye offset.
void scan_shift(ID3D12GraphicsCommandList* list) {
  if (!g_scan_shift_on.load(std::memory_order_relaxed)) {
    return;
  }
  const float eye_w = static_cast<float>(g_backbuffer_w.load()), eye_h = static_cast<float>(g_backbuffer_h.load());
  float ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
  if (!native_stereo::scan_reproject(eye_w, eye_h, &ox, &oy, &sx, &sy)) {
    return;
  }
  ID3D12Resource* const live = g_last_fx_target.load(std::memory_order_relaxed);
  if (live == nullptr || GetTickCount64() - g_last_fx_tick.load(std::memory_order_relaxed) > 50) {
    g_scan_shift_skips.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const int n = g_scan_target_n.load(std::memory_order_acquire);
  for (int i = 0; i < n; ++i) {
    ScanTarget& t = g_scan_targets[i];
    if (t.res != live) {
      continue;  // only this frame's chain output
    }
    const std::uint32_t st = t.state.load(std::memory_order_relaxed);
    if (st == 0xFFFFFFFFu || !scan_temp(list, t)) {
      g_scan_shift_skips.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    const auto S = static_cast<D3D12_RESOURCE_STATES>(st);
    const LONG dx = static_cast<LONG>(std::lround(ox * static_cast<float>(t.width) / eye_w));
    const LONG W = static_cast<LONG>(t.width), H = static_cast<LONG>(t.height);
    if (dx == 0 || (dx < 0 ? -dx : dx) >= W) {
      continue;
    }
    // temp(x) = mask(x - dx): content moves by dx (negative = left).
    D3D12_TEXTURE_COPY_LOCATION src = {t.res, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
    D3D12_TEXTURE_COPY_LOCATION tmp = {t.temp, D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, {}};
    src.SubresourceIndex = 0;
    tmp.SubresourceIndex = 0;
    const D3D12_BOX box = {static_cast<UINT>(dx < 0 ? -dx : 0), 0, 0, static_cast<UINT>(dx < 0 ? W : W - dx), static_cast<UINT>(H), 1};
    if (S != D3D12_RESOURCE_STATE_COPY_SOURCE) {
      barrier_on(list, t.res, S, D3D12_RESOURCE_STATE_COPY_SOURCE);
    }
    list->CopyTextureRegion(&tmp, static_cast<UINT>(dx < 0 ? 0 : dx), 0, 0, &src, &box);
    barrier_on(list, t.temp, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COPY_SOURCE);
    barrier_on(list, t.res, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_COPY_DEST);
    list->CopyResource(t.res, t.temp);
    barrier_on(list, t.res, D3D12_RESOURCE_STATE_COPY_DEST, S);
    barrier_on(list, t.temp, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_COMMON);
    g_scan_shifts.fetch_add(1, std::memory_order_relaxed);
  }
}

void STDMETHODCALLTYPE RenderTargets_hook(ID3D12GraphicsCommandList* list, UINT count, const D3D12_CPU_DESCRIPTOR_HANDLE* rts, BOOL single,
                                          const D3D12_CPU_DESCRIPTOR_HANDLE* depth) {
  t_bound_dsv = depth != nullptr ? depth->ptr : 0;
  t_bound_rts = count;
  if (count <= 8 && (rts != nullptr || count == 0)) {
    t_rt_list = list;
    t_rt_count = count;
    for (UINT i = 0; i < count; ++i) {
      t_rts[i].ptr = single ? rts[0].ptr + static_cast<SIZE_T>(i) * g_rtv_stride : rts[i].ptr;
    }
    t_rt_has_depth = depth != nullptr;
    t_rt_depth = depth != nullptr ? *depth : D3D12_CPU_DESCRIPTOR_HANDLE{};
  } else {
    t_rt_list = nullptr;
  }
  const int layer_eye = native_stereo::ui_pass_eye();
  // Scan targets: what the neighbour passes render into while the scan runs (main view only; eyefx is off).
  if (layer_eye == 1 && native_stereo::ui_pass_kind() == 1 && scan_active() && rts != nullptr && count > 0) {
    for (UINT i = 0; i < count; ++i) {
      ID3D12Resource* r = rtv_resource(single ? rts[0].ptr + static_cast<SIZE_T>(i) * g_rtv_stride : rts[i].ptr);
      scan_target_add(r);
      if (i == 0 && r != nullptr) {
        g_last_fx_target.store(r, std::memory_order_relaxed);
        g_last_fx_tick.store(GetTickCount64(), std::memory_order_relaxed);
      }
    }
  }
  // Second view's UI pass during the scan: shift the scan targets once, before the pass binds its own target.
  if (layer_eye == 2 && native_stereo::ui_pass_kind() == 0 && scan_active() && !t_shift_done) {
    t_shift_done = true;
    scan_shift(list);
  }
  if (layer_eye != 0) {
    if (depth != nullptr) {
      sig_count(count > 0 ? g_sig_rd : g_sig_d);
    }
    t_depth_bound = depth != nullptr;
    // Scan running: a UI-tagged pass binds a depth target (the scan's highlight meshes, uitrace 07:46-07:48 "R0+d" /
    // "R1+d" at 1172x1271). [LIVE] 08:11-08:12: the first signal (any viewport that is neither full nor half size)
    // fired all through gameplay -- the neighbour passes use the upscaler-input size outside the scan too -- so no HUD
    // PSO was ever learned in gameplay (14, all from the main menu) and the compass was left unplaced.
    // [LIVE] scansig run 2026-10-08 11:41-11:43 (scan 11:42:17-11:42:48): a neighbour pass binding a render target
    // together with a DSV ("fx rd") was 0 every second outside the scan and ~65-74/s (once per frame) during it;
    // DSV-only binds and odd-size viewports run all through gameplay (~70/s), which is why the earlier signals failed.
    if (depth != nullptr && count > 0 && native_stereo::ui_pass_kind() == 1) {
      const ULONGLONG now = GetTickCount64();
      if (now - g_scan_tick.exchange(now, std::memory_order_relaxed) >= 500) {
        ui_pso_revoke_recent();  // scan starts
      }
    }
    {
      int rw = 0, rh = 0;  // render-target size when the RTV is known (the trace prints it instead of the handle bits)
      if (ID3D12Resource* r0 = rts != nullptr && count > 0 ? rtv_resource(rts[0].ptr) : nullptr) {
        const D3D12_RESOURCE_DESC rd = r0->GetDesc();
        rw = static_cast<int>(rd.Width);
        rh = static_cast<int>(rd.Height);
      }
      trace_ev('R', count, depth != nullptr ? 1u : 0u, rw, rh, false);
    }
  }
  if (layer_eye == 1 && count == 1 && rts != nullptr && native_stereo::ui_pass_kind() == 0 && native_stereo::ui_layer_active()) {
    float w = 0.f, h = 0.f;
    if (t_first_rt == 0) {
      t_first_rt = rts[0].ptr;
    } else if (rts[0].ptr != t_first_rt) {
      g_ui[1].other_targets.fetch_add(1, std::memory_order_relaxed);
    }
    if (ui_target(&w, &h) && ensure_layer(list, ui_canvas::panel_width, ui_canvas::panel_height)) {
      g_render_targets(list, 1, &g_layer_rtv, FALSE, depth);
      t_layer_list = list;
      t_orig_rt = rts[0];
      t_orig_has_depth = depth != nullptr;
      t_orig_depth = depth != nullptr ? *depth : D3D12_CPU_DESCRIPTOR_HANDLE{};
      t_panel_target_w = static_cast<float>(ui_canvas::panel_width);
      t_panel_target_h = static_cast<float>(ui_canvas::panel_height);
      // The raster basis is the eye canvas. ui_map separately fits the logical
      // window-sized content inside it, including its lower-left origin.
      t_panel_source_w = w;
      t_panel_source_h = h;
      if (!t_layer_clone && t_pso != nullptr && t_pso_list == list && g_panel_cover.load(std::memory_order_relaxed)) {
        if (ID3D12PipelineState* clone = hud_alpha_clone(list, t_pso)) {  // the PSO set before the redirect
          g_set_pso(list, clone);
          t_layer_clone = true;
          g_panel_cover_binds.fetch_add(1, std::memory_order_relaxed);
        }
      }
      float ox=0.f,oy=0.f,sx=1.f,sy=1.f;
      native_stereo::ui_map(1,t_panel_target_w,t_panel_target_h,&ox,&oy,&sx,&sy,t_panel_source_w,t_panel_source_h);
      const D3D12_VIEWPORT source = {0.f, 0.f, t_panel_source_w, t_panel_source_h, 0.f, 1.f};
      const auto placed = map_viewport(source, ox, oy, sx, sy);
      g_viewports(list, 1, &placed);
      const D3D12_RECT all = {0, 0, static_cast<LONG>(t_panel_target_w), static_cast<LONG>(t_panel_target_h)};
      g_scissors(list, 1, &all);
      t_placed_list = list;
      t_mapped_now = true;
      t_odd_vp = false;
      t_placed_sc = true;
      t_map_scale = 1.f;
      if (!t_layer_cleared) {
        const float clear[4] = {0.f, 0.f, 0.f, 0.f};
        list->ClearRenderTargetView(g_layer_rtv, clear, 0, nullptr);
        t_layer_cleared = true;
      }
      g_ui[1].layer_binds.fetch_add(1, std::memory_order_relaxed);
      g_ui[1].targets.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
  g_render_targets(list, count, rts, single, depth);
  if (const int eye = native_stereo::ui_pass_eye()) {
    float w = 0.f, h = 0.f, ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
    if (count > 0 && scan_active() && native_stereo::ui_pass_kind() == 1 && ui_target(&w, &h)) {
      // Scan running: no placement; the second view's pass still sets no viewport, so give it the plain full target
      // (not a placed one left over on the list).
      const D3D12_VIEWPORT full = {0.f, 0.f, w, h, 0.f, 1.f};
      g_viewports(list, 1, &full);
      t_mapped_now = false;
    } else if (count > 0 && !(scan_active() && native_stereo::ui_pass_kind() == 1) && ui_target(&w, &h) &&
               native_stereo::ui_map(eye, w, h, &ox, &oy, &sx, &sy)) {
      // The second view's pass sets no viewport of its own: give the bound target the mapped full-target viewport.
      // A viewport the pass sets afterwards is mapped by Viewports_hook.
      const D3D12_VIEWPORT full = {0.f, 0.f, w, h, 0.f, 1.f};
      const D3D12_VIEWPORT mapped = map_viewport(full, ox, oy, sx, sy);
      g_viewports(list, 1, &mapped);
      if (t_hud_band) {
        // Wrist HUD: the pass may never set a scissor of its own, so the cut is installed with the target.
        const D3D12_RECT all = band_cut({0, 0, static_cast<LONG>(w), static_cast<LONG>(h)}, w, h, 1.f);
        const D3D12_RECT msc = map_rect(all, ox, oy, sx, sy, w, h);
        g_scissors(list, 1, &msc);
        t_placed_sc = true;
      }
      if (t_placed_list != list) {
        g_ui_placed[eye].fetch_add(1, std::memory_order_relaxed);
      }
      t_placed_list = list;
      t_mapped_now = true;
      t_map_scale = 1.f;
    }
    g_ui[eye].targets.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(g_ui_mutex);
    g_ui_rt_count[eye] = count;
    g_ui_rt0[eye] = rts != nullptr && count > 0 ? rts[0].ptr : 0;
  }
}

void STDMETHODCALLTYPE ExecuteIndirect_hook(ID3D12GraphicsCommandList* list, ID3D12CommandSignature* sig, UINT max, ID3D12Resource* args,
                                            UINT64 args_off, ID3D12Resource* count, UINT64 count_off) {
  render_activity::note(list,3);
  const UINT stamp = render_activity::stamp_before(list, max, 0, 0, true);  // census only
  g_execute_indirect(list, sig, max, args, args_off, count, count_off);
  render_activity::stamp_after(list, stamp);
  if (const int eye = native_stereo::ui_pass_eye()) {
    g_ui[eye].indirect.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace

namespace camera {

void install() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&install), &self)) {
    return;
  }
  g_log_path = parent_dir(module_path(self)) + L"\\CAMERA_OBSERVE.log";
  logf("camera observer build %s %s", __DATE__, __TIME__);
  // Dedicated startup flag: menu saves cannot silently remove the probe token.
  const std::wstring trace_path = parent_dir(g_log_path) + L"\\AvatarVR_scantrace.txt";
  HANDLE trace = CreateFileW(trace_path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                           nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  char mode[64] = {};
  DWORD read = 0;
  if (trace != INVALID_HANDLE_VALUE) {
    ReadFile(trace, mode, sizeof(mode) - 1, &read, nullptr);
    CloseHandle(trace);
  }
  g_fx_trace=std::strstr(mode,"effects")!=nullptr;
  g_scan_trace.store((mode[0] == 'o' && mode[1] == 'n')||g_fx_trace);
  scanner_inputs::enabled.store(std::strstr(mode,"inputs")!=nullptr);
  g_inputs_armed=GetTickCount64();
  logf("scaninputs: startup=%d geometry=1 (read-only; 120 scanner draw snapshots max, 1/eye/second, 120 s after first scan)",scanner_inputs::enabled.load());
  logf("scantrace: startup=%d (CPU-only; max 64 binding snapshots, 120 s after scan starts, 32 MiB shader budget)", g_scan_trace.load());
  if(g_fx_trace) logf("scantrace: effects mode; max 128 non-depth UI draw identities within 600 s of gameplay");
  const std::wstring uv_path=parent_dir(g_log_path)+L"\\AvatarVR_scanstereo.txt";
  std::memset(mode,0,sizeof(mode));
  settings_file::read(uv_path,mode,sizeof(mode));
  g_scan_uv_on.store(mode[0]=='o'&&mode[1]=='n');
  g_scan_stable=std::strstr(mode,"stable")!=nullptr;
  g_scan_screen=std::strstr(mode,"screen")!=nullptr;
  g_picture_on=std::strstr(mode,"no-picture")==nullptr;
  g_exact_fx_route=std::strstr(mode,"legacy-route")==nullptr;
  if(const char* wide=std::strstr(mode,"wide")) {
    const float value=std::strtof(wide+4,nullptr);
    g_scan_plane_wide=true;
    g_scan_plane_width=value>=1.f&&value<=2.f?value:1.25f;
  }
  logf("scanstereo: startup=%d wide=%d width=%.2f stable=%d screen=%d (mask and decorative plane independent)",g_scan_uv_on.load(),g_scan_plane_wide,g_scan_plane_width,g_scan_stable,g_scan_screen);
  logf("fxpicture: startup=%d (captured radial-blur shader only, requires fxstretch)",g_picture_on);
  logf("fxroute: exact=%d (captured scanner/boost; legacy-route opts out)",g_exact_fx_route);
}

void note_backbuffer(UINT width, UINT height) {
  g_backbuffer_w.store(width);
  g_backbuffer_h.store(height);
}

void watch_list(void* object, bool research) {
  g_research = research;
  if (object == nullptr) {
    return;
  }
  auto* unknown = static_cast<IUnknown*>(object);
  ID3D12GraphicsCommandList* list = nullptr;
  if (FAILED(unknown->QueryInterface(IID_PPV_ARGS(&list)))) {
    logf("command list is not ID3D12GraphicsCommandList");
    return;
  }
  hook_target(vtable_slot(list, kRsSetViewports), reinterpret_cast<void*>(&Viewports_hook), reinterpret_cast<void**>(&g_viewports), "RSSetViewports");
  hook_target(vtable_slot(list, kSetGraphicsRoot32BitConstants), reinterpret_cast<void*>(&RootConstants_hook), reinterpret_cast<void**>(&g_root_constants), "SetGraphicsRoot32BitConstants");
  hook_target(vtable_slot(list, kSetGraphicsRootConstantBufferView), reinterpret_cast<void*>(&RootCbv_hook), reinterpret_cast<void**>(&g_root_cbv), "SetGraphicsRootConstantBufferView");
  hook_target(vtable_slot(list, kDrawInstanced), reinterpret_cast<void*>(&Draw_hook), reinterpret_cast<void**>(&g_draw), "DrawInstanced");
  hook_target(vtable_slot(list, kDrawIndexedInstanced), reinterpret_cast<void*>(&DrawIndexed_hook), reinterpret_cast<void**>(&g_draw_indexed), "DrawIndexedInstanced");
  hook_target(vtable_slot(list, kRsSetScissorRects), reinterpret_cast<void*>(&Scissors_hook), reinterpret_cast<void**>(&g_scissors), "RSSetScissorRects");
  hook_target(vtable_slot(list, kOmSetRenderTargets), reinterpret_cast<void*>(&RenderTargets_hook), reinterpret_cast<void**>(&g_render_targets), "OMSetRenderTargets");
  hook_target(vtable_slot(list, 47), reinterpret_cast<void*>(&ClearDsv_hook), reinterpret_cast<void**>(&g_clear_dsv), "ClearDepthStencilView");
  hook_target(vtable_slot(list, kExecuteIndirect), reinterpret_cast<void*>(&ExecuteIndirect_hook), reinterpret_cast<void**>(&g_execute_indirect), "ExecuteIndirect");
  hook_target(vtable_slot(list, kDispatch), reinterpret_cast<void*>(&Dispatch_hook), reinterpret_cast<void**>(&g_dispatch), "Dispatch");
  hook_target(vtable_slot(list, kSetPipelineState), reinterpret_cast<void*>(&SetPipelineState_hook), reinterpret_cast<void**>(&g_set_pso), "SetPipelineState");
  hook_target(vtable_slot(list, 26), reinterpret_cast<void*>(&ResourceBarrier_hook), reinterpret_cast<void**>(&g_resource_barrier), "ResourceBarrier");
  hook_target(vtable_slot(list, 32), reinterpret_cast<void*>(&SetTable_hook), reinterpret_cast<void**>(&g_set_table), "SetGraphicsRootDescriptorTable");
  hook_target(vtable_slot(list, 28), reinterpret_cast<void*>(&SetHeaps_hook), reinterpret_cast<void**>(&g_set_heaps), "SetDescriptorHeaps");
  hook_target(vtable_slot(list, 29), reinterpret_cast<void*>(&SetCsRootSig_hook), reinterpret_cast<void**>(&g_set_cs_rootsig), "SetComputeRootSignature");
  hook_target(vtable_slot(list, 30), reinterpret_cast<void*>(&SetGfxRootSig_hook), reinterpret_cast<void**>(&g_set_gfx_rootsig), "SetGraphicsRootSignature");
  if(scanner_inputs::enabled.load(std::memory_order_relaxed)) {
    // Outside profiler modes dx12 does not necessarily watch these lists.
    // Arm the existing Reset observer explicitly for geometry invalidation.
    render_activity::watch(list);
    hook_target(vtable_slot(list,43),reinterpret_cast<void*>(&InputIndices_hook),reinterpret_cast<void**>(&g_input_indices),"IASetIndexBuffer (scanner observation)");
    hook_target(vtable_slot(list,44),reinterpret_cast<void*>(&InputVertices_hook),reinterpret_cast<void**>(&g_input_vertices),"IASetVertexBuffers (scanner observation)");
  }
  list->Release();
}

void sig_report() {
  const ULONGLONG now = GetTickCount64();
  const auto first_input=g_inputs_started.load(std::memory_order_relaxed);
  if(scanner_inputs::enabled.load(std::memory_order_relaxed)&&
     (now-g_inputs_armed>600000||(first_input&&now-first_input>120000))) {
    scanner_inputs::enabled.store(false);logf("scaninputs: time budget reached; stopped");
  }
  ULONGLONG last = g_sig_last.load(std::memory_order_relaxed);
  if (now - last < 1000 || !g_sig_last.compare_exchange_strong(last, now)) {
    return;
  }
  unsigned long long v[2][5];
  logf("scanstereo: on=%d draws=%u",g_scan_uv_on.load(),g_scan_uv_draws.load());
  logf("fxpicture: whole-picture draws=%u",g_picture_draws.load());
  logf("fxroute: scanner=%u without_scan_signal=%u",g_exact_scan_draws.load(),g_exact_scan_without_signal.load());
  for (int k = 0; k < 2; ++k) {
    v[k][0] = g_sig_rd[k].exchange(0);
    v[k][1] = g_sig_d[k].exchange(0);
    v[k][2] = g_sig_odd[k].exchange(0);
    v[k][3] = g_sig_h3[k].exchange(0);
    v[k][4] = g_sig_big[k].exchange(0);
  }
  SYSTEMTIME st;
  GetLocalTime(&st);
  {
    ULONGLONG last_table = g_scan_draw_logged.load();
    if (now - last_table >= 10000 && g_scan_draw_logged.compare_exchange_strong(last_table, now)) {
      std::lock_guard<std::mutex> lock(g_scan_draw_mutex);
      for (int i = 0; i < g_scan_draw_n; ++i) {
        const ScanDraw& d = g_scan_draws[i];
        const std::uint32_t k = d.pack;
        logf("%02u:%02u:%02u scandraw R pso=%p %s%u n=%llu reproj=%d blend: on=%u src=%u dst=%u op=%u srcA=%u dstA=%u mask=%u", st.wHour,
             st.wMinute, st.wSecond, static_cast<void*>(d.pso), (d.count & 0x80000000u) ? "D" : "I", d.count & 0x7FFFFFFFu,
             static_cast<unsigned long long>(d.n), d.reproj ? 1 : 0, (k >> 27) & 1, k & 31, (k >> 5) & 31, (k >> 10) & 7, (k >> 13) & 31,
             (k >> 18) & 31, (k >> 23) & 15);
      }
      g_scan_draw_n = 0;
    }
  }
  census_report(st);
  logf("%02u:%02u:%02u fxstretch: on=%d opaque HUD-pass draws=%llu stretched scan=%llu opaque=%llu | full-screen tris HUD pass "
       "%llu (full %llu) neighbour pass %llu (full %llu)", st.wHour, st.wMinute,
       st.wSecond, g_fx_stretch.load() ? 1 : 0, static_cast<unsigned long long>(g_fx_opaque_draws.exchange(0)),
       static_cast<unsigned long long>(g_fx_stretched_scan.exchange(0)), static_cast<unsigned long long>(g_fx_stretched_opaque.exchange(0)),
       static_cast<unsigned long long>(g_fx_tri[0].exchange(0)), static_cast<unsigned long long>(g_fx_tri_full[0].exchange(0)),
       static_cast<unsigned long long>(g_fx_tri[1].exchange(0)), static_cast<unsigned long long>(g_fx_tri_full[1].exchange(0)));
  logf("%02u:%02u:%02u scanoverlay: on=%d draws=%llu skip root=%llu unbounded=%llu other=%llu rootsigs=%d", st.wHour, st.wMinute,
       st.wSecond, g_ov_on.load() ? 1 : 0, static_cast<unsigned long long>(g_ov_draws.load()),
       static_cast<unsigned long long>(g_ov_skip_root.load()), static_cast<unsigned long long>(g_ov_skip_unbounded.load()),
       static_cast<unsigned long long>(g_ov_skip_other.load()), g_root_info_n.load());
  logf("%02u:%02u:%02u scanmask: on=%d shifts=%llu skips=%llu", st.wHour, st.wMinute, st.wSecond, g_scan_mask_on.load() ? 1 : 0,
       static_cast<unsigned long long>(g_mask_shifts.load()), static_cast<unsigned long long>(g_mask_skips.load()));
  logf("%02u:%02u:%02u srvprobe: heaps=%d srvs=%llu copied=%llu", st.wHour, st.wMinute, st.wSecond, g_vis_heap_n.load(),
       static_cast<unsigned long long>(g_srv_created.load()), static_cast<unsigned long long>(g_srv_copied.load()));
  logf("%02u:%02u:%02u scanshift: on=%d targets=%d shifts=%llu skips=%llu", st.wHour, st.wMinute, st.wSecond,
       g_scan_shift_on.load() ? 1 : 0, g_scan_target_n.load(), static_cast<unsigned long long>(g_scan_shifts.load()),
       static_cast<unsigned long long>(g_scan_shift_skips.load()));
  logf("%02u:%02u:%02u psos: graphics=%llu stream=%llu libgraphics=%llu libstream=%llu blended=%llu", st.wHour, st.wMinute, st.wSecond,
       static_cast<unsigned long long>(g_pso_seen[0].load()), static_cast<unsigned long long>(g_pso_seen[1].load()),
       static_cast<unsigned long long>(g_pso_seen[2].load()), static_cast<unsigned long long>(g_pso_seen[3].load()),
       static_cast<unsigned long long>(g_pso_blended.load()));
  logf("%02u:%02u:%02u scansig ui: rd=%llu d=%llu odd=%llu h3=%llu big=%llu | fx: rd=%llu d=%llu odd=%llu h3=%llu big=%llu", st.wHour,
       st.wMinute, st.wSecond, v[0][0], v[0][1], v[0][2],
       v[0][3], v[0][4], v[1][0], v[1][1], v[1][2], v[1][3], v[1][4]);
}

void rect_probe_report(void (*line)(const char*)) {
  RectHit copy[kRectSlots];
  int n = 0;
  std::uint64_t dropped = 0;
  {
    std::lock_guard<std::mutex> lock(g_rect_mutex);
    n = g_rect_count;
    std::memcpy(copy, g_rects, sizeof(RectHit) * static_cast<size_t>(n));
    dropped = g_rect_dropped;
  }
  HMODULE game = GetModuleHandleW(nullptr);
  char text[200];
  snprintf(text, sizeof(text), "rects: %d sub-target viewport/scissor rects outside UI passes (dropped %llu); afop base %p", n,
           static_cast<unsigned long long>(dropped), static_cast<void*>(game));
  line(text);
  for (int i = 0; i < n; ++i) {
    const RectHit& e = copy[i];
    HMODULE owner = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       static_cast<LPCWSTR>(e.caller), &owner);
    wchar_t path[MAX_PATH] = {};
    if (owner != nullptr) {
      GetModuleFileNameW(owner, path, MAX_PATH);
    }
    const wchar_t* base = std::wcsrchr(path, L'\\');
    snprintf(text, sizeof(text), "  %c (%ld,%ld)-(%ld,%ld) %ldx%ld n=%llu caller %ls+0x%llX thread %lu", e.kind, e.l, e.t, e.r, e.b,
             e.r - e.l, e.b - e.t, static_cast<unsigned long long>(e.count), base != nullptr ? base + 1 : L"?",
             static_cast<unsigned long long>(static_cast<const char*>(e.caller) - reinterpret_cast<const char*>(owner)), e.thread);
    line(text);
  }
}

void watch_device(void* object) {
  if (object == nullptr) {
    return;
  }
  auto* unk = static_cast<IUnknown*>(object);
  ID3D12Device* dev = nullptr;
  if (SUCCEEDED(unk->QueryInterface(IID_PPV_ARGS(&dev)))) {
    hook_target(vtable_slot(dev, 10), reinterpret_cast<void*>(&CreateGraphicsPso_hook), reinterpret_cast<void**>(&g_create_gpso),
                "ID3D12Device::CreateGraphicsPipelineState");
    g_rtv_stride = dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    hook_target(vtable_slot(dev, 20), reinterpret_cast<void*>(&CreateRtv_hook), reinterpret_cast<void**>(&g_create_rtv),
                "ID3D12Device::CreateRenderTargetView");
    hook_target(vtable_slot(dev, 24), reinterpret_cast<void*>(&CopyDescSimple_hook), reinterpret_cast<void**>(&g_copy_desc_simple),
                "ID3D12Device::CopyDescriptorsSimple");
    g_srv_stride = dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    hook_target(vtable_slot(dev, 18), reinterpret_cast<void*>(&CreateSrv_hook), reinterpret_cast<void**>(&g_create_srv),
                "ID3D12Device::CreateShaderResourceView");
    hook_target(vtable_slot(dev, 14), reinterpret_cast<void*>(&CreateHeap_hook), reinterpret_cast<void**>(&g_create_heap),
                "ID3D12Device::CreateDescriptorHeap");
    hook_target(vtable_slot(dev, 16), reinterpret_cast<void*>(&CreateRootSig_hook), reinterpret_cast<void**>(&g_create_rootsig),
                "ID3D12Device::CreateRootSignature");
    hook_target(vtable_slot(dev, 23), reinterpret_cast<void*>(&CopyDesc_hook), reinterpret_cast<void**>(&g_copy_desc),
                "ID3D12Device::CopyDescriptors");
    dev->Release();
  }
  ID3D12Device1* dev1 = nullptr;
  if (SUCCEEDED(unk->QueryInterface(IID_PPV_ARGS(&dev1)))) {
    hook_target(vtable_slot(dev1, 44), reinterpret_cast<void*>(&CreateLibrary_hook), reinterpret_cast<void**>(&g_create_library),
                "ID3D12Device1::CreatePipelineLibrary");
    dev1->Release();
  }
  ID3D12Device2* dev2 = nullptr;
  if (SUCCEEDED(unk->QueryInterface(IID_PPV_ARGS(&dev2)))) {
    hook_target(vtable_slot(dev2, 47), reinterpret_cast<void*>(&CreateStreamPso_hook), reinterpret_cast<void**>(&g_create_spso),
                "ID3D12Device2::CreatePipelineState");
    dev2->Release();
  }
}

void set_scan_shift(bool on) { g_scan_shift_on.store(on); }
void set_panel_cover(bool on) { g_panel_cover.store(on, std::memory_order_relaxed); }
void set_scan_mask(bool on) { g_scan_mask_on.store(on); }
void set_scan_overlay(bool on) { g_ov_on.store(on); }
void set_fx_stretch(bool on) { g_fx_stretch.store(on); }

void hudfx_sweep() {
  if (g_hudfx_sweeping.exchange(false)) {
    // Opening the menu takes a moment, so stop on the draw before the one now shown.
    const unsigned draw = g_hudfx_draw.load();
    if (draw > 1) g_hudfx_draw.store(draw - 1);
    logf("hudfx: sweep stopped at draw %u", g_hudfx_draw.load());
  } else {
    g_hudfx_pick.store(0);
    g_hudfx_sweep_secs = 0;
    if (g_hudfx_draw.load() == 0) g_hudfx_draw.store(1);
    g_hudfx_sweeping.store(true);
  }
  hudfx_draw_label();
}
void hudfx_draw_step(int direction) {
  g_hudfx_pick.store(0);
  g_hudfx_sweeping.store(false);
  const int draw = static_cast<int>(g_hudfx_draw.load()) + direction;
  g_hudfx_draw.store(draw < 0 ? 0u : static_cast<unsigned>(draw));
  hudfx_draw_label();
}
void hudfx_draw_next() { hudfx_draw_step(1); }
void hudfx_draw_previous() { hudfx_draw_step(-1); }
void hudfx_next() {
  g_hudfx_sweeping.store(false);
  g_hudfx_draw.store(0);
  const unsigned pick = (g_hudfx_pick.load() + 1) % (kHudFxCount + 1);
  g_hudfx_pick.store(pick);
  if (pick == 0) snprintf(g_hudfx_label, sizeof(g_hudfx_label), "HUD shader test: off");
  else snprintf(g_hudfx_label, sizeof(g_hudfx_label), "HUD shader test: %u of %u", pick, kHudFxCount);
  logf("hudfx: selected %s", g_hudfx_label);
}
const char* hudfx_status() { return g_hudfx_label; }

void ui_pass_begin() {
  {
    // Wrist HUD for this pass: gameplay HUD only (not the main-menu layer, not the effect passes, not the scan), and
    // the band cut only while the HUD graph is live (it asks for the health piece), so pause menus keep their bottom.
    t_hud = wrist_hud::current();
    const bool hud = t_hud.on && native_stereo::ui_pass_kind() == 0 && !native_stereo::ui_layer_active() && !scan_active();
    t_hud_pass = hud && native_stereo::ui_pass_eye() == 1;
    t_hud_band = hud && t_hud.band > 0.f && ui_piece::hud_live(500);
    t_hud_cleared = false;
  }
  t_shift_done = false;
  t_map_scale = 1.f;
  t_odd_vp = false;
  t_hudfx_ordinal = 0;
  t_trace_n = 0;
  t_trace_odd = false;
  t_trace_canvas = false;
  t_depth_bound = false;
  t_mapped_now = false;
  t_layer_cleared = false;
  t_first_rt = 0;
  t_prehud_done = false;
}

ID3D12Resource* hud_acquire() {
  if (GetTickCount64() - g_hud_written.load(std::memory_order_relaxed) > 250) {
    return nullptr;  // no HUD copied lately (wrist HUD off, menus, loading, stereo paused)
  }
  std::lock_guard<std::mutex> lock(g_hud_mutex);
  if (g_hud != nullptr) {
    g_hud->AddRef();
  }
  return g_hud;
}

void hud_report(char* out, size_t size) {
  snprintf(out, size, "wrist hud: copies %llu, band cuts %llu, hud live %d, region masks %llu, replacing %d",
           static_cast<unsigned long long>(g_hud_copies.load()), static_cast<unsigned long long>(g_hud_band_cuts.load()),
           ui_piece::hud_live(500) ? 1 : 0, static_cast<unsigned long long>(g_hud_masks.load()),
           wrist_hud::replacing() ? 1 : 0);
}

ID3D12Resource* ui_layer_acquire() {
  if (GetTickCount64() - g_layer_written.load(std::memory_order_relaxed) > 250) {
    return nullptr;  // no UI drawn into it lately (front end over, or stereo paused)
  }
  std::lock_guard<std::mutex> lock(g_layer_mutex);
  if (g_layer != nullptr) {
    g_layer->AddRef();
  }
  return g_layer;
}

void ui_pass_end() {
  sig_report();
  {
    const ULONGLONG now = GetTickCount64();
    const int eye = native_stereo::ui_pass_eye();
    std::atomic<ULONGLONG>& last = t_trace_canvas ? g_trace_last_canvas : t_trace_odd ? g_trace_last_odd : g_trace_last_plain;
    const ULONGLONG gap = t_trace_canvas ? 2000 : t_trace_odd ? 10000 : 30000;
    ULONGLONG prev = last.load();
    if (t_trace_n > 0 && now - prev >= gap && last.compare_exchange_strong(prev, now)) {
      logf("uitrace eye %d kind %d %s: %d events (dsv-mapped draws so far %llu, no-dsv %llu)", eye, native_stereo::ui_pass_kind(),
           t_trace_canvas ? "with a window-canvas viewport (screen effect?)" : t_trace_odd ? "with non-full viewports (scan?)" : "plain", t_trace_n,
           static_cast<unsigned long long>(g_draws_dsv_mapped.load()), static_cast<unsigned long long>(g_draws_nodsv_mapped.load()));
      char line[900];
      int n = 0;
      for (int i = 0; i < t_trace_n; ++i) {
        const TraceEv& e = t_trace[i];
        char one[64];
        switch (e.kind) {
          case 'R': snprintf(one, sizeof(one), " R%u%s[%dx%d]", e.a, e.b ? "+d" : "", e.x, e.y); break;
          case 'V': snprintf(one, sizeof(one), " V%ux%u@%d,%d%s", e.a, e.b, e.x, e.y, e.mapped ? "*" : ""); break;
          case 'D': snprintf(one, sizeof(one), " D%u/%u%s", e.a, e.b, e.mapped ? "*" : ""); break;
          case 'I': snprintf(one, sizeof(one), " I%u/%u%s", e.a, e.b, e.mapped ? "*" : ""); break;
          default: snprintf(one, sizeof(one), " %c", e.kind); break;
        }
        char run[16] = "";
        if (e.n > 1) {
          snprintf(run, sizeof(run), "x%u", e.n);
        }
        if (n + 80 > static_cast<int>(sizeof(line))) {
          logf("  %s", line);
          n = 0;
        }
        n += snprintf(line + n, sizeof(line) - n, "%s%s", one, run);
      }
      if (n > 0) {
        logf("  %s", line);
      }
    }
  }
  if (t_hud_cleared) {
    g_hud_written.store(GetTickCount64(), std::memory_order_relaxed);
    t_hud_cleared = false;
  }
  t_hud_pass = false;
  t_hud_band = false;
  if (t_layer_cleared) {
    g_layer_written.store(GetTickCount64(), std::memory_order_relaxed);
    g_layer_passes.fetch_add(1, std::memory_order_relaxed);
    t_layer_cleared = false;
  }
  if (t_layer_list != nullptr) {
    // Bind back what the pass asked for, so later work on the list does not draw into the layer.
    g_render_targets(t_layer_list, 1, &t_orig_rt, FALSE, t_orig_has_depth ? &t_orig_depth : nullptr);
    if (t_layer_clone && t_pso != nullptr && t_pso_list == t_layer_list) {
      g_set_pso(t_layer_list, t_pso);  // the game's own PSO, not the panel's alpha clone
    }
    t_layer_clone = false;
    t_layer_list = nullptr;
  }
  ID3D12GraphicsCommandList* list = t_placed_list;
  if (list == nullptr) {
    return;
  }
  float w = 0.f, h = 0.f;
  ui_target(&w, &h);
  // Put back what the game set (unmapped); a list with nothing recorded on this thread gets the full target.
  const D3D12_VIEWPORT full = {0.f, 0.f, w, h, 0.f, 1.f};
  const D3D12_VIEWPORT vp = t_vp_list == list ? t_vp : full;
  if (vp.Width > 0.f && vp.Height > 0.f) {
    g_viewports(list, 1, &vp);
  }
  if (t_placed_sc) {
    const D3D12_RECT all = {0, 0, static_cast<LONG>(w), static_cast<LONG>(h)};
    const D3D12_RECT sc = t_sc_list == list ? t_sc : all;
    g_scissors(list, 1, &sc);
  }
  t_placed_list = nullptr;
  t_placed_sc = false;
}

std::uint64_t ui_placed(int eye) { return eye >= 1 && eye <= 2 ? g_ui_placed[eye].load() : 0; }

void ui_probe_report(int eye, char* out, size_t size) {
  if (eye < 1 || eye > 2 || out == nullptr || size == 0) {
    return;
  }
  UiStats& u = g_ui[eye];
  int n = snprintf(out, size, "reproj=%llu half=%llu world=%llu uipsos=%d scan=%d fullscreen=%llu idx3=%llu idx6=%llu idxother=%llu ",
                   static_cast<unsigned long long>(u.reproj.exchange(0)),
                   static_cast<unsigned long long>(u.half.exchange(0)), static_cast<unsigned long long>(u.world.exchange(0)),
                   g_ui_pso_count.load() - g_ui_pso_revokes.load(), scan_active() ? 1 : 0,
                   static_cast<unsigned long long>(u.fullscreen.exchange(0)), static_cast<unsigned long long>(u.idx3.exchange(0)),
                   static_cast<unsigned long long>(u.idx6.exchange(0)), static_cast<unsigned long long>(u.idx_other.exchange(0)));
  n += snprintf(out + n, size - n, "dispatch=%llu layerbind=%llu othert=%llu layerpasses=%llu panelcover=%llu ",
                   static_cast<unsigned long long>(u.dispatches.exchange(0)), static_cast<unsigned long long>(u.layer_binds.exchange(0)),
                   static_cast<unsigned long long>(u.other_targets.exchange(0)), static_cast<unsigned long long>(g_layer_passes.load()),
                   static_cast<unsigned long long>(eye == 1 ? g_panel_cover_binds.exchange(0) : 0));
  n += snprintf(out + n, size - n, "vp=%llu sc=%llu rt=%llu draw=%llu idx=%llu indirect=%llu consts=%llu cbv=%llu",
                   static_cast<unsigned long long>(u.viewports.exchange(0)), static_cast<unsigned long long>(u.scissors.exchange(0)),
                   static_cast<unsigned long long>(u.targets.exchange(0)), static_cast<unsigned long long>(u.draws.exchange(0)),
                   static_cast<unsigned long long>(u.indexed.exchange(0)), static_cast<unsigned long long>(u.indirect.exchange(0)),
                   static_cast<unsigned long long>(u.consts.exchange(0)), static_cast<unsigned long long>(u.cbvs.exchange(0)));
  std::lock_guard<std::mutex> lock(g_ui_mutex);
  for (int i = 0; i < g_ui_vp_count[eye] && n > 0 && n < static_cast<int>(size); ++i) {
    const D3D12_VIEWPORT& v = g_ui_vp[eye][i];
    n += snprintf(out + n, size - n, " vp%d=(%.0f,%.0f %.0fx%.0f z%.1f-%.1f)", i, v.TopLeftX, v.TopLeftY, v.Width, v.Height, v.MinDepth,
                  v.MaxDepth);
  }
  if (g_ui_sc_seen[eye] && n > 0 && n < static_cast<int>(size)) {
    const D3D12_RECT& r = g_ui_sc[eye];
    n += snprintf(out + n, size - n, " sc=(%ld,%ld-%ld,%ld)", r.left, r.top, r.right, r.bottom);
  }
  if (n > 0 && n < static_cast<int>(size)) {
    snprintf(out + n, size - n, " rts=%u rt0=%llx list=%p", g_ui_rt_count[eye], static_cast<unsigned long long>(g_ui_rt0[eye]),
             static_cast<void*>(g_ui_list[eye]));
  }
  g_ui_vp_count[eye] = 0;
  g_ui_sc_seen[eye] = false;
}

void sample() {
  log_depth_ranges();  // read-only census, every 10 s
  ProjectionHit projection;
  ViewHit view;
  ViewportHit viewport;
  {
    std::lock_guard<std::mutex> lock(g_sample_mutex);
    projection = g_projection;
    view = g_view;
    viewport = g_viewport;
    g_projection.seen = false;
    g_view.seen = false;
    g_viewport.seen = false;
  }
  const char* pos = "absent";
  const char* rot = "absent";
  const char* fov = "absent";
  const char* viewport_text = "absent";
  char pos_buf[80] = {};
  char rot_buf[96] = {};
  char fov_buf[160] = {};
  char viewport_buf[64] = {};
  if (view.seen) {
    snprintf(pos_buf, sizeof(pos_buf), "%.3f %.3f %.3f", view.x, view.y, view.z);
    snprintf(rot_buf, sizeof(rot_buf), "pitch=%.2f yaw=%.2f roll=%.2f", view.pitch, view.yaw, view.roll);
    pos = pos_buf;
    rot = rot_buf;
  }
  if (projection.seen) {
    char near_buf[32] = "absent";
    char far_buf[32] = "absent";
    if (projection.near_ok) {
      snprintf(near_buf, sizeof(near_buf), "%.4f", projection.near_z);
    }
    if (projection.far_ok) {
      snprintf(far_buf, sizeof(far_buf), "%.1f", projection.far_z);
    }
    snprintf(fov_buf, sizeof(fov_buf), "LIKELY %.2f near=%s far=%s layout=%s root=%u",
             projection.fov_y, near_buf, far_buf,
             projection.column_major ? "column" : "row", projection.root);
    fov = fov_buf;
  }
  if (viewport.seen) {
    snprintf(viewport_buf, sizeof(viewport_buf), "%.0fx%.0f", viewport.width, viewport.height);
    viewport_text = viewport_buf;
  }
  logf("sample thread=%lu position=%s rotation=%s fov=%s view=%s viewport=%s cbv=%llu",
       GetCurrentThreadId(),
       pos,
       rot,
       fov,
       view.seen ? (view.column_major ? "UNCONFIRMED column" : "UNCONFIRMED row") : "absent",
       viewport_text,
       static_cast<unsigned long long>(g_cbv_count.load()));
}

}  // namespace camera
