#include "../runtime_log.hpp"
#include "dlss_twin.hpp"

#include "../camera/eye_write.hpp"
#include "../camera/native_stereo.hpp"
#include "../diagnostics/frame_capture.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "vr_arms.hpp"
#include "vr_weapon.hpp"
#include "mv_repair.hpp"
#include "xr_depth.hpp"
#include "frame_gen.hpp"
#include "../dx12/dx12.hpp"

#include <MinHook.h>
#include <windows.h>
#include <d3d12.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include "../settings_file.hpp"

namespace {

// Minimal ABI-compatible declarations of Streamline 2.x core types (include/sl_struct.h, sl_core_types.h).
struct SlGuid {
  uint32_t data1;
  uint16_t data2, data3;
  uint8_t data4[8];
};
struct SlBase {
  SlBase* next;
  SlGuid structType;
  size_t structVersion;
};
struct SlViewport {  // sl::ViewportHandle
  SlBase base;
  uint32_t value;
};
static_assert(offsetof(SlViewport, value) == 32, "sl::ViewportHandle layout");
constexpr SlGuid kViewportGuid = {0x171b6435, 0x9b3c, 0x4fc8, {0x99, 0x94, 0xfb, 0xe5, 0x25, 0x69, 0xaa, 0xa4}};
using SlResult = int;    // sl::Result, eOk == 0
using SlFeature = uint32_t;
constexpr SlFeature kFeatureDLSS = 0;
constexpr SlFeature kFeatureDLSS_RR = 1001;
constexpr uint32_t kTwinOffset = 0x100;

using SetTagFn = SlResult (*)(const SlViewport&, const void*, uint32_t, void*);
using SetTagForFrameFn = SlResult (*)(const void*, const SlViewport&, const void*, uint32_t, void*);
using SetConstantsFn = SlResult (*)(const void*, const void*, const SlViewport&);
using AllocateFn = SlResult (*)(void*, SlFeature, const SlViewport&);
using FreeFn = SlResult (*)(SlFeature, const SlViewport&);
using EvaluateFn = SlResult (*)(SlFeature, const void*, const SlBase**, uint32_t, void*);
using GetFeatureFunctionFn = SlResult (*)(SlFeature, const char*, void*&);
using SetOptionsFn = SlResult (*)(const SlViewport&, const void*);  // slDLSSSetOptions / slDLSSDSetOptions

SetTagFn g_set_tag = nullptr;
SetTagForFrameFn g_set_tag_for_frame = nullptr;
SetConstantsFn g_set_constants = nullptr;
AllocateFn g_allocate = nullptr;
FreeFn g_free = nullptr;
EvaluateFn g_evaluate = nullptr;
GetFeatureFunctionFn g_get_feature_function = nullptr;
SetOptionsFn g_dlss_set_options = nullptr;
SetOptionsFn g_dlssd_set_options = nullptr;

std::atomic<bool> g_hooked{false};
std::atomic<bool> g_enabled{false};
std::wstring g_control_path;
std::wstring g_log_path;
std::atomic<uint64_t> g_mirrors{0};
std::atomic<uint64_t> g_eval_left{0};
std::atomic<uint64_t> g_eval_right_twin{0};
std::atomic<uint64_t> g_eval_mono{0};
std::atomic<uint64_t> g_eval_other{0};
std::atomic<uint64_t> g_eval_no_viewport{0};
std::atomic<uint32_t> g_last_viewport{0xFFFFFFFFu};
std::atomic<uint32_t> g_last_feature{0xFFFFFFFFu};

void log_line(const char* fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  if (g_log_path.empty()) {
    return;
  }
  runtime_log::append(g_log_path, {buffer, "\r\n"});
}

std::wstring self_dir() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&self_dir), &self)) {
    return {};
  }
  wchar_t path[MAX_PATH] = {};
  const DWORD n = GetModuleFileNameW(self, path, MAX_PATH);
  if (n == 0 || n >= MAX_PATH) {
    return {};
  }
  std::wstring dir(path, n);
  const auto slash = dir.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : dir.substr(0, slash);
}

bool is_viewport(const SlBase* s) {
  return s != nullptr && std::memcmp(&s->structType, &kViewportGuid, sizeof(SlGuid)) == 0;
}

SlViewport twin_of(const SlViewport& v) {
  SlViewport t = v;
  t.base.next = nullptr;
  t.value = v.value + kTwinOffset;
  return t;
}

bool is_twin(const SlViewport& v) {
  return v.value >= kTwinOffset && v.value != 0xFFFFFFFFu;
}

// ---- per-viewport input record (read-only; for comparing the main view with a second engine view) -----
// Streamline 2.x ABI: ResourceTag = base(32) + Resource* @32 + BufferType u32 @40 + lifecycle u32 @44 + Extent @48
// (top,left,width,height) -> 64 bytes. Resource = base(32) + type @32 + native @40 + memory @48 + view @56 +
// state @64 + width @68 + height @72 + nativeFormat @76. DLSS(D)Options = base(32) + mode @32 + outputWidth @36 +
// outputHeight @40.
// sl::Constants (Streamline 2.x sl_consts.h) = base(32) + cameraViewToClip @32, clipToCameraView @96, clipToLensClip
// @160, clipToPrevClip @224, prevClipToClip @288 (float4x4 each) + jitterOffset @352 + mvecScale @360 +
// cameraPinholeOffset @368 + cameraPos @376 + up/right/fwd @388/400/412 + near/far/fov/aspect/mvInvalid @424..440 +
// Boolean (1 byte) depthInverted @444, cameraMotionIncluded @445, motionVectors3D @446, reset @447.
constexpr size_t kConstsBytes = 456;
constexpr size_t kClipToPrevClip = 224, kJitter = 352, kMvecScale = 360, kReset = 447;
constexpr int kTagTypes = 64;
struct VpInfo {
  std::atomic<uint32_t> id{0xFFFFFFFFu};
  std::atomic<uint32_t> res_w[kTagTypes]{}, res_h[kTagTypes]{}, ext_w[kTagTypes]{}, ext_h[kTagTypes]{}, fmt[kTagTypes]{};
  std::atomic<uint32_t> ext_t[kTagTypes]{}, ext_l[kTagTypes]{};
  std::atomic<bool> seen[kTagTypes]{};
  std::atomic<uint32_t> opt_mode{0}, opt_w{0}, opt_h{0};
  unsigned char opt_raw[96] = {};  // diagnostic copy of the caller's DLSSOptions (torn reads are harmless)
  std::atomic<bool> opt_raw_seen{false};
  std::atomic<uint64_t> tag_calls{0};
  // Latest sl::Constants (diagnostic copy, torn reads are harmless) and how many of them asked for a history reset.
  unsigned char consts[kConstsBytes] = {};
  std::atomic<uint64_t> const_calls{0}, const_resets{0};
  // Depth (buffer type 0) and motion-vector (type 1) tags: the native resource and the state the engine declared.
  std::atomic<void*> tag_native[2] = {};
  std::atomic<uint32_t> tag_state[2] = {};
  // Tag freshness (2026-10-09): the tagged resources are recreated on level loads. [LIVE] 06:29:00: the first stereo
  // frame after a loading screen faulted the GPU with the repair on; the recorded pointers can be the freed ones until
  // the game tags again. tag_seq counts records per type; tag_used is its value at this viewport's last evaluate.
  std::atomic<uint64_t> tag_seq[2] = {};
  uint64_t tag_used[2] = {};
};
// View rebuilds allocate new Streamline viewport IDs. Stable map nodes keep
// metadata pointers valid across callbacks without a four-view lifetime limit.
std::map<uint32_t, VpInfo> g_info;
std::mutex g_info_mutex;
VpInfo* g_primary_info = nullptr;

VpInfo* info_for(uint32_t id) {
  std::lock_guard<std::mutex> lock(g_info_mutex);
  auto [it, inserted] = g_info.try_emplace(id);
  if (inserted) it->second.id.store(id, std::memory_order_relaxed);
  if (g_primary_info == nullptr) g_primary_info = &it->second;
  return &it->second;
}

bool read_mem(const void* src, void* dst, size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

void record_tags(uint32_t viewport, const void* tags, uint32_t count) {
  if (tags == nullptr || viewport >= kTwinOffset) {
    return;
  }
  VpInfo* info = info_for(viewport);
  if (info == nullptr) {
    return;
  }
  info->tag_calls.fetch_add(1, std::memory_order_relaxed);
  const auto* base = static_cast<const unsigned char*>(tags);
  for (uint32_t i = 0; i < count && i < 32; ++i) {
    unsigned char tag[64];
    if (!read_mem(base + static_cast<size_t>(i) * 64, tag, sizeof(tag))) {
      return;
    }
    const void* res = nullptr;
    uint32_t type = 0, ext[4] = {};
    std::memcpy(&res, tag + 32, sizeof(res));
    std::memcpy(&type, tag + 40, sizeof(type));
    std::memcpy(ext, tag + 48, sizeof(ext));
    if (type >= static_cast<uint32_t>(kTagTypes)) {
      continue;
    }
    uint32_t dims[3] = {};
    if ((type == 0 || type == 1) && res != nullptr) {
      void* native = nullptr;
      uint32_t state = 0;
      if (read_mem(static_cast<const unsigned char*>(res) + 40, &native, sizeof(native)) &&
          read_mem(static_cast<const unsigned char*>(res) + 64, &state, sizeof(state))) {
        info->tag_native[type].store(native, std::memory_order_relaxed);
        info->tag_state[type].store(state, std::memory_order_relaxed);
        info->tag_seq[type].fetch_add(1, std::memory_order_release);
      }
    }
    if (res != nullptr && read_mem(static_cast<const unsigned char*>(res) + 68, dims, sizeof(dims))) {
      info->res_w[type].store(dims[0], std::memory_order_relaxed);
      info->res_h[type].store(dims[1], std::memory_order_relaxed);
      info->fmt[type].store(dims[2], std::memory_order_relaxed);
    }
    info->ext_t[type].store(ext[0], std::memory_order_relaxed);
    info->ext_l[type].store(ext[1], std::memory_order_relaxed);
    info->ext_w[type].store(ext[2], std::memory_order_relaxed);
    info->ext_h[type].store(ext[3], std::memory_order_relaxed);
    info->seen[type].store(true, std::memory_order_relaxed);
  }
}

// "match" (AvatarVR_dlss.txt): a second engine view (any viewport other than the first one seen) gets the
// first viewport's DLSS mode and input extents. The second Gfx_View renders at the same dynamic render size as
// the main view but asks Streamline for DLAA over the full output, so DLSS-RR treats the whole target as input.
// Only private copies of the caller's structs are changed.
std::atomic<bool> g_match{false};
std::atomic<uint64_t> g_match_tags{0}, g_match_options{0};

VpInfo* primary_info() {
  std::lock_guard<std::mutex> lock(g_info_mutex);
  return g_primary_info;
}

bool is_secondary(uint32_t viewport) {
  VpInfo* p = primary_info();
  return g_match.load(std::memory_order_relaxed) && viewport < kTwinOffset && p != nullptr &&
         p->id.load(std::memory_order_relaxed) != viewport;
}

// Copy `count` tags and give each one the primary viewport's extent for its buffer type, when the primary uses a
// sub-rectangle for that type. Returns false if nothing could be copied.
bool matched_tags(const void* tags, uint32_t count, std::vector<unsigned char>* out) {
  VpInfo* p = primary_info();
  if (p == nullptr || tags == nullptr || count == 0 || count > 32) {
    return false;
  }
  out->resize(static_cast<size_t>(count) * 64);
  if (!read_mem(tags, out->data(), out->size())) {
    return false;
  }
  for (uint32_t i = 0; i < count; ++i) {
    unsigned char* tag = out->data() + static_cast<size_t>(i) * 64;
    uint32_t type = 0;
    std::memcpy(&type, tag + 40, sizeof(type));
    if (type >= static_cast<uint32_t>(kTagTypes) || !p->seen[type].load(std::memory_order_relaxed)) {
      continue;
    }
    const uint32_t ext[4] = {p->ext_t[type].load(), p->ext_l[type].load(), p->ext_w[type].load(), p->ext_h[type].load()};
    if (ext[2] == 0 || ext[3] == 0) {
      continue;  // the primary uses the whole resource for this type
    }
    std::memcpy(tag + 48, ext, sizeof(ext));
  }
  return true;
}

void record_options(uint32_t viewport, const void* options) {
  if (options == nullptr || viewport >= kTwinOffset) {
    return;
  }
  VpInfo* info = info_for(viewport);
  uint32_t v[3] = {};
  if (info != nullptr && read_mem(static_cast<const unsigned char*>(options) + 32, v, sizeof(v))) {
    info->opt_mode.store(v[0], std::memory_order_relaxed);
    info->opt_w.store(v[1], std::memory_order_relaxed);
    info->opt_h.store(v[2], std::memory_order_relaxed);
  }
  if (info != nullptr && read_mem(options, info->opt_raw, sizeof(info->opt_raw))) {
    info->opt_raw_seen.store(true, std::memory_order_relaxed);
  }
}

// ---- DLSS model preset (2026-11-11, ISSUE-05) -------------------------------------------------------------------------
// [LIVE] stage run 2026-11-11 17:02 at 100 % eyes (2904x3088 out, 1481x1575 in): Gfx_DLSS::UpscaleInternal ~1.7 ms per
// eye, the largest single GPU stage. DLSS 310.9 defaults to the transformer model unless the game picks a preset; NVIDIA
// documents the transformer as heavier than the CNN presets. "preset X" in [dlss] (X = A..O, or "default") rewrites the
// six per-mode preset fields of a private copy of the game's DLSSOptions for both eyes. Default: absent = the game's own
// choice, nothing written.
// [INFERENCE] layout, Streamline 2.x sl_dlss.h: base(32) + mode @32, outputWidth @36, outputHeight @40, sharpness @44,
// preExposure @48, exposureScale @52, three 1-byte Booleans @56..58, presets dlaa/quality/balanced/performance/
// ultraPerformance/ultraQuality @60..80 (uint32, eDefault 0, A 1 ... O 15), useAutoExposure @84, alphaUpscaling @85.
// mode/outputWidth/outputHeight are already confirmed by the logged "out=" sizes. The write is skipped unless the
// struct looks like that layout (finite exposure floats, Booleans 0/1, presets <= 31); the "options raw" log line
// shows the game's values so the layout can be checked from a run before the switch is used.
constexpr size_t kPresetFirst = 60, kPresetCount = 6;
std::atomic<int> g_preset{-1};  // -1 = the game's choice
std::atomic<uint64_t> g_preset_writes{0}, g_preset_rejects{0};

bool options_layout_ok(const unsigned char* o) {
  uint32_t mode = 0, w = 0, h = 0;
  float pre = 0.f, scale = 0.f;
  std::memcpy(&mode, o + 32, 4);
  std::memcpy(&w, o + 36, 4);
  std::memcpy(&h, o + 40, 4);
  std::memcpy(&pre, o + 48, 4);
  std::memcpy(&scale, o + 52, 4);
  if (mode > 6 || w == 0 || h == 0 || w > 16384 || h > 16384) return false;
  if (!std::isfinite(pre) || !std::isfinite(scale) || pre <= 0.f || scale <= 0.f || pre > 1000.f || scale > 1000.f) return false;
  for (int i = 56; i <= 58; ++i) if (o[i] > 1) return false;
  for (size_t i = 0; i < kPresetCount; ++i) {
    uint32_t p = 0;
    std::memcpy(&p, o + kPresetFirst + i * 4, 4);
    if (p > 31) return false;
  }
  return true;
}

// Applies the preset to `buffer` (a 256-byte copy). Returns false when nothing was written.
bool apply_preset(unsigned char* buffer) {
  const int preset = g_preset.load(std::memory_order_relaxed);
  if (preset < 0) return false;
  if (!options_layout_ok(buffer)) {
    g_preset_rejects.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  const uint32_t value = static_cast<uint32_t>(preset);
  for (size_t i = 0; i < kPresetCount; ++i) std::memcpy(buffer + kPresetFirst + i * 4, &value, 4);
  g_preset_writes.fetch_add(1, std::memory_order_relaxed);
  return true;
}

// Reads "preset X" from the [dlss] text: A..O -> 1..15, "default" -> 0, absent -> -1.
int parse_preset(const char* text) {
  const char* p = std::strstr(text, "preset ");
  if (p == nullptr) return -1;
  p += 7;
  if (std::strncmp(p, "default", 7) == 0) return 0;
  const char c = static_cast<char>(*p & ~0x20);  // upper case
  return c >= 'A' && c <= 'O' ? c - 'A' + 1 : -1;
}

// ---- mirrored setters --------------------------------------------------------------------------------

SlResult SetTag_hook(const SlViewport& viewport, const void* tags, uint32_t count, void* cmd) {
  record_tags(viewport.value, tags, count);
  std::vector<unsigned char> copy;
  if (is_secondary(viewport.value) && matched_tags(tags, count, &copy)) {
    tags = copy.data();
    g_match_tags.fetch_add(1, std::memory_order_relaxed);
  }
  const SlResult r = g_set_tag(viewport, tags, count, cmd);
  if (!is_twin(viewport)) {
    const SlViewport t = twin_of(viewport);
    g_set_tag(t, tags, count, cmd);
    g_mirrors.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

SlResult SetTagForFrame_hook(const void* frame, const SlViewport& viewport, const void* tags, uint32_t count, void* cmd) {
  record_tags(viewport.value, tags, count);
  std::vector<unsigned char> copy;
  if (is_secondary(viewport.value) && matched_tags(tags, count, &copy)) {
    tags = copy.data();
    g_match_tags.fetch_add(1, std::memory_order_relaxed);
  }
  const SlResult r = g_set_tag_for_frame(frame, viewport, tags, count, cmd);
  if (!is_twin(viewport)) {
    const SlViewport t = twin_of(viewport);
    g_set_tag_for_frame(frame, t, tags, count, cmd);
    g_mirrors.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

std::atomic<int> g_jitlog_left{0};

SlResult SetConstants_hook(const void* values, const void* frame, const SlViewport& viewport) {
  if (values != nullptr && viewport.value < kTwinOffset) {
    if (VpInfo* info = info_for(viewport.value)) {
      unsigned char c[kConstsBytes];
      if (read_mem(values, c, sizeof(c))) {
        if (g_jitlog_left.load(std::memory_order_relaxed) > 0) {
          g_jitlog_left.fetch_sub(1, std::memory_order_relaxed);
          float f[kConstsBytes / 4];
          std::memcpy(f, c, sizeof(f));
          LARGE_INTEGER q;
          QueryPerformanceCounter(&q);
          log_line("jit %lld vp%u j=(%.5f, %.5f) p8=%.7f p9=%.7f", static_cast<long long>(q.QuadPart), viewport.value,
                   f[kJitter / 4], f[kJitter / 4 + 1], f[8 + 8], f[8 + 9]);
        }
        std::memcpy(info->consts, c, sizeof(c));
        info->const_calls.fetch_add(1, std::memory_order_relaxed);
        if (c[kReset] != 0) {
          info->const_resets.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
  }
  const SlResult r = g_set_constants(values, frame, viewport);
  if (!is_twin(viewport)) {
    const SlViewport t = twin_of(viewport);
    g_set_constants(values, frame, t);
    g_mirrors.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

SlResult Allocate_hook(void* cmd, SlFeature feature, const SlViewport& viewport) {
  const SlResult r = g_allocate(cmd, feature, viewport);
  if (!is_twin(viewport)) {
    const SlViewport t = twin_of(viewport);
    const SlResult tr = g_allocate(cmd, feature, t);
    log_line("slAllocateResources feature=%u viewport=%u -> %d, twin %u -> %d", feature, viewport.value, r, t.value, tr);
  }
  return r;
}

SlResult Free_hook(SlFeature feature, const SlViewport& viewport) {
  if (!is_twin(viewport)) {
    const SlViewport t = twin_of(viewport);
    const SlResult tr = g_free(feature, t);
    log_line("slFreeResources feature=%u twin %u -> %d", feature, t.value, tr);
  }
  return g_free(feature, viewport);
}

// Options copy with the primary viewport's mode (struct is far smaller than 256 bytes; SL reads only its own size).
const void* matched_options(uint32_t viewport, const void* options, unsigned char* buffer) {
  VpInfo* p = primary_info();
  if (!is_secondary(viewport) || options == nullptr || p == nullptr || p->opt_mode.load() == 0) {
    return options;
  }
  if (!read_mem(options, buffer, 256)) {
    return options;
  }
  const uint32_t mode = p->opt_mode.load();
  std::memcpy(buffer + 32, &mode, sizeof(mode));
  g_match_options.fetch_add(1, std::memory_order_relaxed);
  return buffer;
}

SlResult DlssSetOptions_wrap(const SlViewport& viewport, const void* options) {
  record_options(viewport.value, options);
  alignas(16) unsigned char buffer[256];
  options = matched_options(viewport.value, options, buffer);
  if (options != nullptr && g_preset.load(std::memory_order_relaxed) >= 0) {
    // Preset override on a private copy (DLSS SR only; DLSS-D / ray reconstruction keeps the game's choice).
    alignas(16) static thread_local unsigned char copy[256];
    if (read_mem(options, copy, sizeof(copy)) && apply_preset(copy)) options = copy;
  }
  const SlResult r = g_dlss_set_options(viewport, options);
  if (!is_twin(viewport)) {
    g_dlss_set_options(twin_of(viewport), options);
    g_mirrors.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

SlResult DlssdSetOptions_wrap(const SlViewport& viewport, const void* options) {
  record_options(viewport.value, options);
  alignas(16) unsigned char buffer[256];
  options = matched_options(viewport.value, options, buffer);
  const SlResult r = g_dlssd_set_options(viewport, options);
  if (!is_twin(viewport)) {
    g_dlssd_set_options(twin_of(viewport), options);
    g_mirrors.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

SlResult GetFeatureFunction_hook(SlFeature feature, const char* name, void*& function) {
  const SlResult r = g_get_feature_function(feature, name, function);
  if (r == 0 && name != nullptr && function != nullptr) {
    if (std::strcmp(name, "slDLSSDSetOptions") == 0) {
      g_dlssd_set_options = reinterpret_cast<SetOptionsFn>(function);
      function = reinterpret_cast<void*>(&DlssdSetOptions_wrap);
      log_line("slGetFeatureFunction(%u, slDLSSDSetOptions): wrapped", feature);
    } else if (std::strcmp(name, "slDLSSSetOptions") == 0) {
      g_dlss_set_options = reinterpret_cast<SetOptionsFn>(function);
      function = reinterpret_cast<void*>(&DlssSetOptions_wrap);
      log_line("slGetFeatureFunction(%u, slDLSSSetOptions): wrapped", feature);
    } else {
      log_line("slGetFeatureFunction(%u, %s)", feature, name);
    }
  }
  return r;
}

// ---- evaluate: right-eye frames use the twin viewport ------------------------------------------------

// Per-viewport evaluate counts (distinct Streamline viewport ids, first 8 seen).
constexpr int kVpSlots = 8;
std::atomic<uint32_t> g_vp_id[kVpSlots];
std::atomic<uint64_t> g_vp_count[kVpSlots];
std::atomic<int> g_vp_used{0};

void count_viewport(uint32_t id) {
  const int used = g_vp_used.load(std::memory_order_acquire);
  for (int i = 0; i < used; ++i) {
    if (g_vp_id[i].load(std::memory_order_relaxed) == id) {
      g_vp_count[i].fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
  int slot = g_vp_used.load();
  while (slot < kVpSlots && !g_vp_used.compare_exchange_weak(slot, slot + 1)) {
  }
  if (slot < kVpSlots) {
    g_vp_id[slot].store(id, std::memory_order_relaxed);
    g_vp_count[slot].fetch_add(1, std::memory_order_relaxed);
  }
}

// ---- motion-vector capture ("mvcap" in AvatarVR_dlss.txt) -------------------------------------------------
// One-shot diagnostic: on the next evaluate of each viewport (up to kMvSlots), the motion-vector resource tagged for that
// viewport is copied on the evaluate command list into a readback buffer; poll() maps it about 1.5 s later and writes
// scripts\captures\mv_vp<N>_<time>.bmp (red/green = signed x/y motion, blue = magnitude, in pixels x 20) plus a .bin
// with the raw texels (header: width, height, DXGI format, row pitch). Used to check whether the second view's
// animated objects (hair, characters) carry object motion like the main view's.
// 2026-10-09: also the depth tag (kind 0, saved as dp_vp<N>_<time>.bin) and the viewport's sl::Constants
// (mv_vp<N>_<time>_consts.bin, kConstsBytes raw) for the offline check of the right-eye motion repair.
constexpr int kMvSlots = 18;  // three pairs: L motion/depth, R before motion/depth, R after motion
struct MvGrab {
  uint32_t vp = 0;
  int kind = 1;  // 1 motion vectors, 0 depth
  int pair = -1;
  char stage = 's';  // s=legacy sample, l=left, b=right before repair, a=right after repair
  ULONGLONG tick = 0;
  unsigned char consts[kConstsBytes] = {};
  ID3D12Resource* readback = nullptr;
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp = {};
  UINT w = 0, h = 0;
  DXGI_FORMAT fmt = DXGI_FORMAT_UNKNOWN;
  float scale[2] = {1.f, 1.f};
  ULONGLONG at = 0;
  UINT64 total = 0;  // readback size; the last row is not padded to the row pitch
};
MvGrab g_mv[kMvSlots];
std::atomic<int> g_mv_used{0};
std::atomic<bool> g_mv_armed{false};
std::atomic<bool> g_mv_burst{false};
std::atomic<int> g_mv_burst_pair{-1};
std::atomic<bool> g_mv_burst_wait_right{false};
std::atomic<ULONGLONG> g_mv_burst_start{0};
UINT64 g_mv_budget = 0;  // evaluate thread; at most 256 MiB of readback storage per burst
std::string g_capture_dir;

void mv_grab(uint32_t vp, int kind, void* cmd, int pair = -1, char stage = 's');

// A burst records this once per eye. The game has so far tagged only depth and
// motion through slSetTag; color may instead arrive as a local evaluate input.
// Log structure identities only, without retaining pointers across a frame.
void log_burst_evaluate_inputs(uint32_t vp, const SlBase** inputs, uint32_t count) {
  char line[420] = {};
  size_t used = static_cast<size_t>(snprintf(line, sizeof(line), "mvburst eval vp=%u inputs=%u", vp, count));
  for (uint32_t i = 0; inputs != nullptr && i < count && i < 12 && used < sizeof(line) - 55; ++i) {
    SlBase base = {};
    uint32_t field40 = 0;
    if (inputs[i] == nullptr || !read_mem(inputs[i], &base, sizeof(base))) {
      used += static_cast<size_t>(snprintf(line + used, sizeof(line) - used, " i%u=unread", i));
      continue;
    }
    if (i != 0) read_mem(reinterpret_cast<const unsigned char*>(inputs[i]) + 40, &field40, sizeof(field40));
    used += static_cast<size_t>(snprintf(line + used, sizeof(line) - used, " i%u=%08X/v%zu/f%u", i,
                                         base.structType.data1, base.structVersion, field40));
  }
  log_line("%s", line);
}

void mv_record(uint32_t vp, void* cmd) {
  if (!g_mv_armed.load(std::memory_order_relaxed) || cmd == nullptr || vp >= kTwinOffset) {
    return;
  }
  mv_grab(vp, 1, cmd);
  mv_grab(vp, 0, cmd);
}

void mv_grab(uint32_t vp, int kind, void* cmd, int pair, char stage) {
  const int used = g_mv_used.load();
  for (int i = 0; i < used; ++i) {
    if (g_mv[i].vp == vp && g_mv[i].kind == kind && g_mv[i].pair == pair && g_mv[i].stage == stage) {
      return;  // this viewport's buffer is already captured
    }
  }
  if (used >= kMvSlots) {
    return;
  }
  VpInfo* info = info_for(vp);
  auto* res = info != nullptr ? static_cast<ID3D12Resource*>(info->tag_native[kind].load()) : nullptr;
  if (res == nullptr) {
    return;
  }
  const D3D12_RESOURCE_DESC desc = res->GetDesc();
  ID3D12Device* device = nullptr;
  if (desc.Dimension != D3D12_RESOURCE_DIMENSION_TEXTURE2D || FAILED(res->GetDevice(IID_PPV_ARGS(&device)))) {
    return;
  }
  MvGrab& g = g_mv[used];
  UINT64 total = 0;
  device->GetCopyableFootprints(&desc, 0, 1, 0, &g.fp, nullptr, nullptr, &total);
  if (pair >= 0 && (total > 32ull * 1024 * 1024 || g_mv_budget + total > 256ull * 1024 * 1024)) {
    device->Release();
    log_line("mvburst: readback budget reached (next %llu bytes, used %llu); capture incomplete",
             static_cast<unsigned long long>(total), static_cast<unsigned long long>(g_mv_budget));
    return;
  }
  g.total = total;
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC bd = {};
  bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  bd.Width = total;
  bd.Height = 1;
  bd.DepthOrArraySize = 1;
  bd.MipLevels = 1;
  bd.SampleDesc.Count = 1;
  bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  const HRESULT hr = device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &bd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                     IID_PPV_ARGS(&g.readback));
  device->Release();
  if (FAILED(hr)) {
    log_line("mvcap vp%u: readback buffer failed 0x%08X", vp, static_cast<unsigned>(hr));
    g_mv_armed.store(false);
    return;
  }
  auto* list = static_cast<ID3D12GraphicsCommandList*>(cmd);
  const auto state = static_cast<D3D12_RESOURCE_STATES>(info->tag_state[kind].load());
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = res;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = state;
  b.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
  const bool transition = state != D3D12_RESOURCE_STATE_COPY_SOURCE;
  if (transition) {
    list->ResourceBarrier(1, &b);
  }
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = g.readback;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = g.fp;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = res;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  if (transition) {
    std::swap(b.Transition.StateBefore, b.Transition.StateAfter);
    list->ResourceBarrier(1, &b);
  }
  g.vp = vp;
  g.kind = kind;
  g.pair = pair;
  g.stage = stage;
  g.tick = GetTickCount64();
  g.w = static_cast<UINT>(desc.Width);
  g.h = desc.Height;
  g.fmt = desc.Format;
  std::memcpy(g.consts, info->consts, sizeof(g.consts));
  float c[kConstsBytes / 4];
  std::memcpy(c, info->consts, sizeof(c));
  g.scale[0] = c[kMvecScale / 4];
  g.scale[1] = c[kMvecScale / 4 + 1];
  g.at = GetTickCount64();
  if (pair >= 0) g_mv_budget += total;
  g_mv_used.store(used + 1);
  if (pair >= 0 && kind == 1) {
    vr_arms::Sample sample;
    const bool arm = vr_arms::latest(sample);
    float socket[16] = {};
    unsigned long long gun_age = 0;
    const bool gun = vr_weapon::written_socket(socket, nullptr, &gun_age);
    float skinned_root[16] = {};
    const bool skinned = skin_palette_probe::held_weapon_root(skinned_root);
    log_line("mvburst pair=%d stage=%c vp=%u tick=%llu arm_seq=%llu hand_valid=%d/%d gun=%d age=%llu place_tick=%u gun_xyz=(%.4f %.4f %.4f) skinned=%d root_xyz=(%.4f %.4f %.4f)",
             pair, stage, vp, static_cast<unsigned long long>(g.tick), arm ? sample.seq : 0,
             arm && sample.valid[0], arm && sample.valid[1], gun, gun_age, vr_weapon::placement_tick(),
             gun ? socket[12] : 0.f, gun ? socket[13] : 0.f, gun ? socket[14] : 0.f, skinned,
             skinned ? skinned_root[12] : 0.f, skinned ? skinned_root[13] : 0.f, skinned ? skinned_root[14] : 0.f);
  }
  log_line("mvcap vp%u %s pair=%d stage=%c: copied %ux%u fmt=%d state=0x%X", vp, kind == 1 ? "motion" : "depth", pair, stage, g.w, g.h,
           static_cast<int>(g.fmt), static_cast<unsigned>(state));
}

float half_to_float(uint16_t h) {
  const uint32_t sign = (h & 0x8000u) << 16, exp = (h >> 10) & 0x1Fu, mant = h & 0x3FFu;
  uint32_t bits;
  if (exp == 0) {
    if (mant == 0) {
      bits = sign;
    } else {  // subnormal
      int e = -1;
      uint32_t m = mant;
      do {
        ++e;
        m <<= 1;
      } while ((m & 0x400u) == 0);
      bits = sign | ((127 - 15 - e) << 23) | ((m & 0x3FFu) << 13);
    }
  } else if (exp == 31) {
    bits = sign | 0x7F800000u | (mant << 13);
  } else {
    bits = sign | ((exp + 127 - 15) << 23) | (mant << 13);
  }
  float f;
  std::memcpy(&f, &bits, 4);
  return f;
}

// Present thread (poll): save captures whose copy has had time to finish.
void mv_flush() {
  const int used = g_mv_used.load();
  const ULONGLONG now = GetTickCount64();
  bool all_done = used > 0;
  for (int i = 0; i < used; ++i) {
    MvGrab& g = g_mv[i];
    if (g.readback == nullptr) {
      continue;
    }
    if (now - g.at < 1500) {
      all_done = false;
      continue;
    }
    void* mapped = nullptr;
    const D3D12_RANGE range = {0, static_cast<SIZE_T>(g.total)};
    const HRESULT mhr = g.readback->Map(0, &range, &mapped);
    if (SUCCEEDED(mhr) && mapped != nullptr) {
      const auto* base = static_cast<const unsigned char*>(mapped) + g.fp.Offset;
      const UINT pitch = g.fp.Footprint.RowPitch;
      SYSTEMTIME t;
      GetLocalTime(&t);
      char stem[MAX_PATH];
      if (g.pair < 0) {
        snprintf(stem, sizeof(stem), "%s\\%s_vp%u_%02d%02d%02d", g_capture_dir.c_str(), g.kind == 1 ? "mv" : "dp", g.vp, t.wHour,
                 t.wMinute, t.wSecond);
      } else {
        snprintf(stem, sizeof(stem), "%s\\%s_mvburst_%llu_f%d_%c_vp%u", g_capture_dir.c_str(), g.kind == 1 ? "mv" : "dp",
                 static_cast<unsigned long long>(g_mv_burst_start.load()), g.pair, g.stage, g.vp);
      }
      // raw
      if (FILE* f = std::fopen((std::string(stem) + ".bin").c_str(), "wb")) {
        const uint32_t hdr[4] = {g.w, g.h, static_cast<uint32_t>(g.fmt), pitch};
        std::fwrite(hdr, sizeof(hdr), 1, f);
        std::fwrite(base, 1, static_cast<size_t>(g.total - g.fp.Offset), f);
        std::fclose(f);
      }
      if (g.kind == 1) {
        if (FILE* f = std::fopen((std::string(stem) + "_consts.bin").c_str(), "wb")) {
          std::fwrite(g.consts, 1, sizeof(g.consts), f);
          std::fclose(f);
        }
      }
      // visualisation, 24-bit bottom-up BMP
      const bool half = g.fmt == DXGI_FORMAT_R16G16_FLOAT || g.fmt == DXGI_FORMAT_R16G16B16A16_FLOAT;
      const bool full = g.fmt == DXGI_FORMAT_R32G32_FLOAT || g.fmt == DXGI_FORMAT_R32G32B32A32_FLOAT;
      const UINT texel = g.fmt == DXGI_FORMAT_R16G16_FLOAT ? 4 : g.fmt == DXGI_FORMAT_R16G16B16A16_FLOAT ? 8
                       : g.fmt == DXGI_FORMAT_R32G32_FLOAT ? 8 : 16;
      double sum = 0.;
      uint64_t moving = 0;
      if (half || full) {
        const UINT row = (g.w * 3 + 3) & ~3u;
        std::vector<unsigned char> img(static_cast<size_t>(row) * g.h);
        for (UINT y = 0; y < g.h; ++y) {
          const unsigned char* src = base + static_cast<size_t>(pitch) * y;
          unsigned char* dst = img.data() + static_cast<size_t>(row) * (g.h - 1 - y);
          for (UINT x = 0; x < g.w; ++x) {
            float mx, my;
            if (half) {
              uint16_t hv[2];
              std::memcpy(hv, src + static_cast<size_t>(x) * texel, 4);
              mx = half_to_float(hv[0]);
              my = half_to_float(hv[1]);
            } else {
              std::memcpy(&mx, src + static_cast<size_t>(x) * texel, 4);
              std::memcpy(&my, src + static_cast<size_t>(x) * texel + 4, 4);
            }
            const float px = std::isfinite(mx) ? mx * g.scale[0] * static_cast<float>(g.w) : 0.f;
            const float py = std::isfinite(my) ? my * g.scale[1] * static_cast<float>(g.h) : 0.f;
            const float mag = std::sqrt(px * px + py * py);
            sum += mag;
            moving += mag > 0.25f ? 1 : 0;
            auto clamp8 = [](float v) { return static_cast<unsigned char>(std::clamp(v, 0.f, 255.f)); };
            dst[x * 3 + 0] = clamp8(mag * 20.f);           // B
            dst[x * 3 + 1] = clamp8(128.f + py * 20.f);    // G
            dst[x * 3 + 2] = clamp8(128.f + px * 20.f);    // R
          }
        }
        if (FILE* f = std::fopen((std::string(stem) + ".bmp").c_str(), "wb")) {
          const uint32_t size = static_cast<uint32_t>(img.size());
          unsigned char hdr[54] = {'B', 'M'};
          const uint32_t file_size = 54 + size, off = 54, info_size = 40, w = g.w, h = g.h, img_size = size;
          const uint16_t planes = 1, bpp = 24;
          std::memcpy(hdr + 2, &file_size, 4);
          std::memcpy(hdr + 10, &off, 4);
          std::memcpy(hdr + 14, &info_size, 4);
          std::memcpy(hdr + 18, &w, 4);
          std::memcpy(hdr + 22, &h, 4);
          std::memcpy(hdr + 26, &planes, 2);
          std::memcpy(hdr + 28, &bpp, 2);
          std::memcpy(hdr + 34, &img_size, 4);
          std::fwrite(hdr, sizeof(hdr), 1, f);
          std::fwrite(img.data(), 1, img.size(), f);
          std::fclose(f);
        }
      }
      const D3D12_RANGE none = {0, 0};
      g.readback->Unmap(0, &none);
      const double n = static_cast<double>(g.w) * g.h;
      log_line("mvcap vp%u: saved %s.bmp/.bin (%ux%u fmt=%d scale=(%.4f, %.4f)) mean motion %.3f px, moving(>0.25 px) %.1f%%",
               g.vp, stem, g.w, g.h, static_cast<int>(g.fmt), g.scale[0], g.scale[1], n > 0 ? sum / n : 0.,
               n > 0 ? 100. * static_cast<double>(moving) / n : 0.);
    } else {
      log_line("mvcap vp%u: Map failed 0x%08X (size %llu)", g.vp, static_cast<unsigned>(mhr),
               static_cast<unsigned long long>(g.total));
    }
    g.readback->Release();
    g.readback = nullptr;
  }
  if (all_done && !g_mv_burst.load()) {
    // keep armed=false until the control file drops and re-adds "mvcap" (edge-triggered in poll)
    g_mv_armed.store(false);
    g_mv_used.store(0);
    g_mv_budget = 0;
  }
}

SlResult Evaluate_hook(SlFeature feature, const void* frame, const SlBase** inputs, uint32_t count, void* cmd) {
  g_last_feature.store(feature, std::memory_order_relaxed);
  if (feature != kFeatureDLSS && feature != kFeatureDLSS_RR) {
    g_eval_other.fetch_add(1, std::memory_order_relaxed);
    return g_evaluate(feature, frame, inputs, count, cmd);
  }
  int index = -1;
  for (uint32_t i = 0; inputs != nullptr && i < count && i < 32; ++i) {
    if (is_viewport(inputs[i])) {
      index = static_cast<int>(i);
      break;
    }
  }
  if (index < 0) {
    g_eval_no_viewport.fetch_add(1, std::memory_order_relaxed);
    return g_evaluate(feature, frame, inputs, count, cmd);
  }
  const auto* viewport = reinterpret_cast<const SlViewport*>(inputs[index]);
  g_last_viewport.store(viewport->value, std::memory_order_relaxed);
  count_viewport(viewport->value);
  VpInfo* tag_info = viewport->value < kTwinOffset ? info_for(viewport->value) : nullptr;
  bool tags_fresh = false;  // depth and motion vectors both re-tagged since this viewport's last evaluate
  if (tag_info != nullptr) {
    const uint64_t s0 = tag_info->tag_seq[0].load(std::memory_order_acquire), s1 = tag_info->tag_seq[1].load(std::memory_order_acquire);
    tags_fresh = s0 != tag_info->tag_used[0] && s1 != tag_info->tag_used[1];
    tag_info->tag_used[0] = s0;
    tag_info->tag_used[1] = s1;
  }
  if ((mv_repair::enabled() || g_mv_burst.load() || xr_depth::enabled() || frame_gen::enabled()) && cmd != nullptr &&
      tags_fresh) {
    unsigned main_id = 0, second_id = 0;
    VpInfo* info = tag_info;
    if (info != nullptr && native_stereo::stereo_viewport_ids(&main_id, &second_id) &&
        (viewport->value == main_id || viewport->value == second_id)) {
      float consts[kConstsBytes / 4];
      std::memcpy(consts, info->consts, sizeof(consts));
      mv_repair::EyeInput eye;
      eye.mv = static_cast<ID3D12Resource*>(info->tag_native[1].load());
      eye.mv_state = static_cast<D3D12_RESOURCE_STATES>(info->tag_state[1].load());
      eye.depth = static_cast<ID3D12Resource*>(info->tag_native[0].load());
      eye.depth_state = static_cast<D3D12_RESOURCE_STATES>(info->tag_state[0].load());
      eye.consts = consts;
      auto* list = static_cast<ID3D12GraphicsCommandList*>(cmd);
      // Optional per-eye depth for the headset layer (xr_depth.hpp, "xrdepth" in [xr]): copied before DLSS runs.
      if (xr_depth::enabled()) {
        xr_depth::capture(viewport->value == main_id ? 0 : 1, list, eye.depth, eye.depth_state, info->ext_l[0].load(),
                          info->ext_t[0].load(), info->ext_w[0].load(), info->ext_h[0].load());
      }
      if (viewport->value == main_id) {
        if (g_mv_burst.load() && !g_mv_burst_wait_right.exchange(true)) {
          const int pair = g_mv_burst_pair.fetch_add(1) + 1;
          if (pair < 3) {
            if (pair == 0) frame_capture::request_eyes_burst(g_mv_burst_start.load());
            if (pair == 0) log_burst_evaluate_inputs(viewport->value, inputs, count);
            mv_grab(viewport->value, 1, cmd, pair, 'l');
            mv_grab(viewport->value, 0, cmd, pair, 'l');
          }
        }
        if (mv_repair::enabled()) mv_repair::capture_left(list, eye);
        if (frame_gen::enabled()) {  // in-mod frame extrapolation (frame_gen.hpp): motion vectors + depth of the main view
          frame_gen::capture(0, list, eye.mv, eye.mv_state, eye.depth, eye.depth_state, consts + kMvecScale / 4,
                             {info->ext_l[1].load(), info->ext_t[1].load(), info->ext_w[1].load(), info->ext_h[1].load()},
                             {info->ext_l[0].load(), info->ext_t[0].load(), info->ext_w[0].load(), info->ext_h[0].load()});
        }
      } else {
        const int pair = g_mv_burst_pair.load();
        const bool burst_pair = g_mv_burst.load() && pair >= 0 && pair < 3;
        if (burst_pair) {
          if (pair == 0) log_burst_evaluate_inputs(viewport->value, inputs, count);
          mv_grab(viewport->value, 1, cmd, pair, 'b');
          mv_grab(viewport->value, 0, cmd, pair, 'b');
        }
        if (mv_repair::enabled()) mv_repair::repair_right(list, eye);  // before DLSS reads MVs
        if (frame_gen::enabled()) {  // after the repair, so the second view's copy has the repaired vectors
          frame_gen::capture(1, list, eye.mv, eye.mv_state, eye.depth, eye.depth_state, consts + kMvecScale / 4,
                             {info->ext_l[1].load(), info->ext_t[1].load(), info->ext_w[1].load(), info->ext_h[1].load()},
                             {info->ext_l[0].load(), info->ext_t[0].load(), info->ext_w[0].load(), info->ext_h[0].load()});
        }
        if (burst_pair) {
          mv_grab(viewport->value, 1, cmd, pair, 'a');
          g_mv_burst_wait_right.store(false);
          if (pair == 2) g_mv_burst.store(false);
        }
      }
    }
  }
  mv_record(viewport->value, cmd);  // after the repair: the capture shows what DLSS receives
  dx12::mark_list(cmd, 'D', static_cast<int>(viewport->value));  // GPU profiler: which submission holds this upscale
  const int eye = eye_write::parity_eye();  // 0 when not alternating
  if (!g_enabled.load(std::memory_order_relaxed) || eye == 0) {
    g_eval_mono.fetch_add(1, std::memory_order_relaxed);
    return g_evaluate(feature, frame, inputs, count, cmd);
  }
  if (eye < 0) {
    g_eval_left.fetch_add(1, std::memory_order_relaxed);
    return g_evaluate(feature, frame, inputs, count, cmd);
  }
  SlViewport twin = twin_of(*viewport);
  twin.base.next = viewport->base.next;  // keep the caller's chain intact
  std::vector<const SlBase*> copy(inputs, inputs + count);
  copy[static_cast<size_t>(index)] = &twin.base;
  g_eval_right_twin.fetch_add(1, std::memory_order_relaxed);
  return g_evaluate(feature, frame, copy.data(), count, cmd);
}

bool hook_export(HMODULE module, const char* name, void* detour, void** original) {
  void* target = reinterpret_cast<void*>(GetProcAddress(module, name));
  if (target == nullptr) {
    log_line("export %s missing", name);
    return false;
  }
  MH_STATUS s = MH_CreateHook(target, detour, original);
  if (s == MH_OK) {
    s = MH_EnableHook(target);
  }
  log_line("hook %s target=%p status=%d", name, target, static_cast<int>(s));
  return s == MH_OK;
}

}  // namespace

namespace dlss_twin {

bool request_mv_burst() {
  if (g_mv_burst.load() || g_mv_used.load() != 0 || g_mv_armed.load()) return false;
  g_mv_burst_pair.store(-1);
  g_mv_burst_wait_right.store(false);
  g_mv_budget = 0;
  g_mv_burst_start.store(GetTickCount64());
  g_mv_burst.store(true);
  log_line("mvburst: armed for three stereo pairs, L raw / R raw / R repaired; eyes follow first main evaluation");
  return true;
}

bool jit_logging() { return g_jitlog_left.load(std::memory_order_relaxed) > 0; }

void install() {
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_control_path = dir + L"\\AvatarVR_dlss.txt";
  g_log_path = dir + L"\\DLSS_TWIN.log";
  mv_repair::set_log(&log_line);
  xr_depth::set_log(&log_line);
  frame_gen::set_log(&log_line);
  {
    const std::wstring cap = dir + L"\\AvatarVR\\captures";
    CreateDirectoryW(cap.c_str(), nullptr);
    char narrow[MAX_PATH] = {};
    WideCharToMultiByte(CP_ACP, 0, cap.c_str(), -1, narrow, sizeof(narrow), nullptr, nullptr);
    g_capture_dir = narrow;
  }
  log_line("dlss twin build %s %s pid=%lu control=AvatarVR_dlss.txt (twin|off; missing = off) twin viewport = id + 0x%X",
           __DATE__, __TIME__, GetCurrentProcessId(), kTwinOffset);
}

void hook(HMODULE interposer) {
  if (interposer == nullptr || g_hooked.exchange(true)) {
    return;
  }
  hook_export(interposer, "slEvaluateFeature", reinterpret_cast<void*>(&Evaluate_hook), reinterpret_cast<void**>(&g_evaluate));
  hook_export(interposer, "slSetTag", reinterpret_cast<void*>(&SetTag_hook), reinterpret_cast<void**>(&g_set_tag));
  hook_export(interposer, "slSetTagForFrame", reinterpret_cast<void*>(&SetTagForFrame_hook), reinterpret_cast<void**>(&g_set_tag_for_frame));
  hook_export(interposer, "slSetConstants", reinterpret_cast<void*>(&SetConstants_hook), reinterpret_cast<void**>(&g_set_constants));
  hook_export(interposer, "slAllocateResources", reinterpret_cast<void*>(&Allocate_hook), reinterpret_cast<void**>(&g_allocate));
  hook_export(interposer, "slFreeResources", reinterpret_cast<void*>(&Free_hook), reinterpret_cast<void**>(&g_free));
  hook_export(interposer, "slGetFeatureFunction", reinterpret_cast<void*>(&GetFeatureFunction_hook), reinterpret_cast<void**>(&g_get_feature_function));
  log_line("sl.interposer hooked");
}

void poll() {
  bool want = false;
  if (g_mv_burst.load() && GetTickCount64() - g_mv_burst_start.load() > 10000) {
    g_mv_burst.store(false);
    log_line("mvburst: timed out after 10 s; captured %d pairs", g_mv_burst_pair.load() + 1);
  }
  if (!g_control_path.empty()) {
    char text[64] = {};
    if (settings_file::read(g_control_path, text, sizeof(text))) {
      want = std::strncmp(text, "twin", 4) == 0;
      const bool match = std::strstr(text, "match") != nullptr;
      if (match != g_match.exchange(match)) {
        log_line("MATCH -> %s", match ? "on (second viewports get the first viewport's mode and extents)" : "off");
      }
      // "mvcap": one motion-vector capture per viewport each time the keyword is added (edge-triggered).
      static bool last_mv = false;
      const bool mv = std::strstr(text, "mvcap") != nullptr;
      if (mv && !last_mv && !g_mv_armed.load()) {
        for (MvGrab& g : g_mv) {
          if (g.readback != nullptr) {
            g.readback->Release();
          }
          g = MvGrab{};
        }
        g_mv_used.store(0);
        g_mv_armed.store(true);
        log_line("mvcap: armed");
      }
      last_mv = mv;
      // One short, opt-in capture. The token is edge-triggered like mvcap; remove it before a repeat.
      static bool last_burst = false;
      const bool burst = std::strstr(text, "mvburst") != nullptr;
      if (burst && !last_burst) request_mv_burst();
      last_burst = burst;
      const int preset = parse_preset(text);
      if (preset != g_preset.exchange(preset)) {
        log_line("PRESET -> %s (takes effect when the game next sets its DLSS options; a mode or resolution change "
                 "in the game's menu forces it)", preset < 0 ? "game's choice" : preset == 0 ? "default" :
                 (std::string("preset ") + static_cast<char>('A' + preset - 1)).c_str());
      }
      mv_repair::set_enabled(std::strstr(text, "mvrepair") != nullptr);  // right-eye motion-vector repair (mv_repair.hpp)
      mv_repair::set_variant(std::strstr(text, "mvfast") != nullptr, std::strstr(text, "mvab") != nullptr);
      static bool last_jl = false;
      const bool jl = std::strstr(text, "jitlog") != nullptr;
      if (jl && !last_jl) {
        g_jitlog_left.store(120);
        log_line("jitlog: next 120 constants calls");
      }
      last_jl = jl;
    }
  }
  if (want != g_enabled.exchange(want)) {
    log_line("MODE -> %s", want ? "twin" : "off");
  }
  mv_flush();
  mv_repair::report();
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last >= 5000) {
    last = now;
    log_line("dlss mode=%s evaluate: left/game=%llu right/twin=%llu mono=%llu other=%llu no_viewport=%llu mirrors=%llu last feature=%u viewport=%u",
             want ? "twin" : "off", static_cast<unsigned long long>(g_eval_left.load()),
             static_cast<unsigned long long>(g_eval_right_twin.load()), static_cast<unsigned long long>(g_eval_mono.load()),
             static_cast<unsigned long long>(g_eval_other.load()), static_cast<unsigned long long>(g_eval_no_viewport.load()),
             static_cast<unsigned long long>(g_mirrors.load()), g_last_feature.load(), g_last_viewport.load());
    char per[256] = {};
    size_t used = 0;
    const int n = g_vp_used.load() < kVpSlots ? g_vp_used.load() : kVpSlots;
    for (int i = 0; i < n && used < sizeof(per) - 40; ++i) {
      used += static_cast<size_t>(snprintf(per + used, sizeof(per) - used, " vp%u=%llu", g_vp_id[i].load(),
                                           static_cast<unsigned long long>(g_vp_count[i].load())));
    }
    log_line("  evaluates per viewport:%s", n > 0 ? per : " none");
    log_line("  preset: %d (-1 = game's choice) writes=%llu rejected=%llu", g_preset.load(),
             static_cast<unsigned long long>(g_preset_writes.load()), static_cast<unsigned long long>(g_preset_rejects.load()));
    std::vector<VpInfo*> infos;
    {
      std::lock_guard<std::mutex> lock(g_info_mutex);
      for (auto& item : g_info) infos.push_back(&item.second);
    }
    for (VpInfo* info : infos) {
      VpInfo& v = *info;
      char tags[400] = {};
      size_t u = 0;
      for (int t = 0; t < kTagTypes && u < sizeof(tags) - 30; ++t) {
        if (v.seen[t].load()) {
          u += static_cast<size_t>(snprintf(tags + u, sizeof(tags) - u, " t%d=%ux%u", t, v.ext_w[t].load(), v.ext_h[t].load()));
        }
      }
      log_line("  vp%u tags=%llu options mode=%u out=%ux%u | ext:%s", v.id.load(), static_cast<unsigned long long>(v.tag_calls.load()),
               v.opt_mode.load(), v.opt_w.load(), v.opt_h.load(), tags);
      if (v.opt_raw_seen.load()) {
        unsigned char o[sizeof(v.opt_raw)];
        std::memcpy(o, v.opt_raw, sizeof(o));
        uint64_t ver = 0;
        float f[3] = {};
        uint32_t pr[kPresetCount] = {};
        std::memcpy(&ver, o + 24, 8);
        std::memcpy(f, o + 44, sizeof(f));
        std::memcpy(pr, o + kPresetFirst, sizeof(pr));
        log_line("  vp%u options raw: structVersion=%llu sharpness=%.3f preExposure=%.3f exposureScale=%.3f bools=%u/%u/%u "
                 "presets dlaa/q/bal/perf/ultraperf/ultraq=%u/%u/%u/%u/%u/%u autoExp=%u layout_ok=%d",
                 v.id.load(), static_cast<unsigned long long>(ver), f[0], f[1], f[2], o[56], o[57], o[58], pr[0], pr[1], pr[2],
                 pr[3], pr[4], pr[5], o[84], options_layout_ok(o) ? 1 : 0);
      }
      // Constants: the jitter DLSS is told vs the jitter in the projection it is given (row 2 of cameraViewToClip),
      // and how far clipToPrevClip is from identity (head still -> near 0; a per-eye error shows as a constant offset).
      float c[kConstsBytes / 4];
      std::memcpy(c, v.consts, sizeof(c));
      const float* proj = c + 32 / 4;
      const float* cpc = c + kClipToPrevClip / 4;
      float dev = 0.f;
      for (int k = 0; k < 16; ++k) {
        dev = std::max(dev, std::fabs(cpc[k] - ((k % 5) == 0 ? 1.f : 0.f)));
      }
      log_line("  vp%u consts=%llu resets=%llu jitter=(%.4f, %.4f) proj[8,9]=(%.6f, %.6f) mvecScale=(%.6f, %.6f) "
               "clipToPrevClip |M-I|max=%.5f row3=(%.5f %.5f %.5f %.5f) flags depthInv=%u camMotion=%u mv3d=%u",
               v.id.load(), static_cast<unsigned long long>(v.const_calls.load()),
               static_cast<unsigned long long>(v.const_resets.load()), c[kJitter / 4], c[kJitter / 4 + 1], proj[8], proj[9],
               c[kMvecScale / 4], c[kMvecScale / 4 + 1], dev, cpc[12], cpc[13], cpc[14], cpc[15], v.consts[444], v.consts[445],
               v.consts[446]);
      // Viewport-tagged Streamline projection is stronger evidence than the
      // generic CBV observer, which also sees shadow/UI/other cameras.
      if (proj[0] > 0.01f && proj[5] > 0.01f) {
        log_line("  vp%u projection: M00/M11 %.6f %.6f aspect %.6f rays %.6f %.6f %.6f %.6f (actual DLSS camera)",
                 v.id.load(), proj[0], proj[5], proj[5]/proj[0], (-1.f-proj[8])/proj[0], (1.f-proj[8])/proj[0],
                 (1.f-proj[9])/proj[5], (-1.f-proj[9])/proj[5]);
      }
    }
    if (g_match.load()) {
      log_line("  match: tag calls rewritten=%llu option calls rewritten=%llu", static_cast<unsigned long long>(g_match_tags.load()),
               static_cast<unsigned long long>(g_match_options.load()));
    }
  }
}

}  // namespace dlss_twin
