#include "../runtime_log.hpp"
#include "xr_runtime.hpp"

#include "../camera/camera.hpp"
#include "../camera/native_stereo.hpp"
#include "head_pose.hpp"
#include "../dx12/dx12.hpp"
#include "xr_input.hpp"
#include "menu_pointer.hpp"
#include "menu_transition.hpp"
#include "wrist_hud.hpp"
#include "xr_depth.hpp"
#include "xr_depth_math.hpp"
#include "frame_gen.hpp"
#include "frame_gen_math.hpp"
#include "../diagnostics/skin_palette_probe.hpp"

#include "../camera/eye_write.hpp"

#include <windows.h>
#include <d3d12.h>
#include <dxgi1_4.h>
#include <d3dcompiler.h>

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D12
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include "../settings_file.hpp"

namespace {

enum class Mode : int { Off = 0, Probe = 1, Session = 2, Cinema = 3 };

bool is_session(Mode m) { return m == Mode::Session || m == Mode::Cinema; }

std::wstring g_control_path;
std::wstring g_log_path;
std::mutex g_log_mutex;
std::mutex g_queue_mutex;
ID3D12CommandQueue* g_queue = nullptr;  // AddRef'd
std::atomic<int> g_want{static_cast<int>(Mode::Off)};
std::atomic<bool> g_worker_started{false};

// Cinema settings (control: "cinema [swap] [width_m] [distance_m]").
std::atomic<bool> g_swap_eyes{false};
std::atomic<bool> g_mirror_right{false};  // diagnostic: show the submitted second view on the desktop
std::atomic<bool> g_copy_perf_probe{false};  // opt-in "perfprobe", CPU waits only; stops after 60 seconds
// "cinema native ...": left eye = the game's backbuffer (main view), right eye = the engine second view's own
// output (native_stereo::acquire_view2_output), both copied every Present.
std::atomic<bool> g_native{false};
std::atomic<uint64_t> g_native_pairs{0};
std::atomic<uint64_t> g_native_missing{0};
std::atomic<uint64_t> g_native_mismatch{0};
// Head tracking: the worker publishes the latest per-eye poses every XR frame; "proj" shows the two eye images as
// a projection layer posed where they were rendered (instead of cinema quads).
std::atomic<bool> g_proj{false};
// Frame pacing (opt-in "halfrate"; worse in the headset 2026-10-07, superseded by game-rate submission): the game's Presents are released on every second headset frame boundary. [LIVE]
// 2026-10-07 05:46 (10339eab / ddf15d4e): stereo at 1992x2160 per eye ran at 38-44 fps, GPU-bound (gpu lag ~ frame
// interval, 24 ms), on a 72 Hz headset: 40 does not divide 72, so images are held for one or two headset frames
// unevenly (judder on top of reprojection). At exactly half the refresh every image is shown twice.
std::atomic<bool> g_half_rate{false};
// Frame submission at game rate (default; "repeat" = the old behaviour). [LIVE]/[AUTHOR] 2026-10-07 06:30 (9af0b879):
// the session loop ran xrWaitFrame/xrEndFrame on every headset refresh and resubmitted the last game image with its
// pose, so the runtime saw a 72 fps app (xr_fps 72 while the game rendered 40-60), Virtual Desktop showed 72 fps and
// never engaged its own frame synthesis (SSW); the user saw plain reprojection. Now the loop waits for a new game image
// before each xrWaitFrame, so every game frame is submitted once and the runtime sees the real frame rate. A 100 ms
// keep-alive submits anyway when the game stalls. [LIVE] 2026-10-07 06:40 (2511b048): with a second signal for the
// menu canvas (same Present, a moment after the eye copy) xr_fps stayed 72 while the game ran 63-68: the second signal
// landed during the submit of the first and re-sent the frame. One signal per Present now.
std::atomic<bool> g_game_rate{true};
// In-mod frame extrapolation (2026-11-12, "framegen" in [xr]; frame_gen.hpp). The worker runs at the headset rate;
// a refresh with no new game image gets a frame warped forward from the last real one. Guarded by g_xr.
std::atomic<bool> g_framegen{false};
std::atomic<bool> g_framegen_k0{false};  // "framegenk0" (test): generated frames are the last real frame, unmoved
bool g_real_new = false;            // the swapchains hold a real image that has not been submitted yet
bool g_content_generated = false;   // the swapchains hold a generated image (labelled with g_gen_views)
bool g_real_kept = false;           // frame_gen kept the last real image (colour + motion vectors + depth)
head_pose::Snapshot g_real_prev, g_real_last, g_gen_views;
bool g_real_prev_valid = false;
int64_t g_real_disp_prev = 0, g_real_disp_last = 0;  // predicted display times of the last two real submissions
// The game's own frame time (smoothed interval between real Presents, ns). The motion vectors span one game frame, so
// k is measured in it; the display-time gap between real submissions jumps between one and two refreshes and made the
// generated motion alternate between half and full steps (2026-11-12).
double g_game_frame_ns = 0.0;
LARGE_INTEGER g_last_real_qpc = {};
std::atomic<uint64_t> g_fg_real{0}, g_fg_generated{0}, g_fg_repeats{0}, g_fg_skip_old{0}, g_fg_skip_not_ready{0},
    g_fg_skip_acquire{0};
HANDLE g_new_image = nullptr;  // auto-reset: a new eye image or menu canvas is in the swapchains
std::atomic<std::uint64_t> g_keepalive_submits{0};
void signal_new_image() {
  if (g_new_image != nullptr) {
    SetEvent(g_new_image);
  }
}
std::atomic<LONGLONG> g_xr_wait_qpc{0};    // QPC when xrWaitFrame last returned (the XR thread)
std::atomic<LONGLONG> g_xr_period_qpc{0};  // headset frame period in QPC ticks
std::atomic<std::uint64_t> g_pace_frames{0}, g_pace_wait_us{0};
std::atomic<int> g_pose_lag{1};   // right eye (second view): StartFrames between the applied pose and the Present
std::atomic<int> g_pose_lag_l{2};  // left eye (backbuffer / main view), usually one frame older than the right
std::mutex g_pose_mutex;
head_pose::Snapshot g_latest;          // guarded by g_pose_mutex
head_pose::Snapshot g_layer_views;     // pose + fov of the images currently in the swapchains (guarded by g_xr)
bool g_layer_valid = false;            // guarded by g_xr
int g_native_absent = 0;               // consecutive native Presents without a second-view image (guarded by g_xr)
constexpr int kMonoAfterPresents = 6;
float g_layer_tan[4] = {};             // game frustum tangents (left, right, up, down) for those images
std::atomic<uint64_t> g_proj_layers{0}, g_proj_no_pose{0};
std::atomic<float> g_screen_width{3.0f};
std::atomic<float> g_screen_distance{2.5f};

// Session objects shared between the XR worker (frame loop) and the Present thread (image copies).
// g_xr guards everything below; the worker never holds it across xrWaitFrame.
struct EyeChain {
  XrSwapchain chain = XR_NULL_HANDLE;
  std::vector<ID3D12Resource*> images;  // owned by the runtime
  bool sampleable = false;  // sampled + mutable usage accepted at creation
  bool has_image = false;               // at least one image released since creation
};
std::mutex g_xr;
// Swapchain handle lifetime. The worker holds it (not g_xr) across xrEndFrame, which can block while the GPU works
// through the game's frame backlog; anything that destroys a swapchain takes it too. Order: g_xr, then g_chains.
// Recursive because destroy_chains_locked() calls destroy_overlay_locked().
std::recursive_mutex g_chains;
XrInstance g_instance = XR_NULL_HANDLE;
XrSession g_session = XR_NULL_HANDLE;


bool g_running = false;
EyeChain g_eyes[2];
uint32_t g_chain_width = 0;
uint32_t g_chain_height = 0;
int64_t g_chain_format = 0;
bool g_chain_failed = false;
// Copy plumbing on the game's swapchain queue.
ID3D12Device* g_copy_device = nullptr;
ID3D12CommandAllocator* g_allocators[3] = {};
UINT64 g_allocator_fence_value[3] = {};
ID3D12GraphicsCommandList* g_copy_list = nullptr;
ID3D12Fence* g_copy_fence = nullptr;
HANDLE g_copy_event = nullptr;
UINT64 g_copy_fence_next = 0;
int g_allocator_index = 0;
ID3D12QueryHeap* g_transport_queries = nullptr;
ID3D12Resource* g_transport_readback = nullptr;
UINT64* g_transport_values = nullptr;
UINT64 g_transport_frequency = 0;
bool g_transport_tried = false, g_transport_pending[3] = {};
double g_transport_eye_ms = 0.0, g_transport_mirror_ms = 0.0;
unsigned g_transport_samples = 0;
std::atomic<uint64_t> g_copies{0};
std::atomic<uint64_t> g_copies_left{0};
std::atomic<uint64_t> g_copies_right{0};
std::atomic<uint64_t> g_copies_mono{0};
std::atomic<uint64_t> g_parity_mismatch{0};
std::atomic<uint64_t> g_layers_submitted{0};
std::atomic<uint64_t> g_pairs{0};
std::atomic<uint64_t> g_orphans{0};
// Mod menu overlay (src/ui/vr_menu.cpp): one quad swapchain seen by both eyes, placed in LOCAL space in front of the
// head when the menu opens. The menu renders its canvas on the game queue and copies it in between
// overlay_acquire() and overlay_release() (Present thread); the worker adds the quad on top of the eye layers.
XrSwapchain g_ovl_chain = XR_NULL_HANDLE;    // guarded by g_xr
std::vector<ID3D12Resource*> g_ovl_images;   // owned by the runtime
uint32_t g_ovl_width = 0, g_ovl_height = 0;  // guarded by g_xr
bool g_ovl_has_image = false;                // guarded by g_xr
bool g_ovl_failed = false;                   // guarded by g_xr
bool g_ovl_acquired = false;                 // guarded by g_xr (held from overlay_acquire to overlay_release)
std::atomic<bool> g_ovl_visible{false};
std::atomic<bool> g_ovl_recentre{false};
std::atomic<float> g_ovl_distance{1.2f};
std::atomic<float> g_ovl_width_m{1.1f};
std::atomic<bool> g_session_running{false};
std::atomic<uint32_t> g_rec_w{0}, g_rec_h{0};  // runtime's recommended per-eye size (view 0), for the menu
XrPosef g_ovl_pose = {{0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, -1.2f}};  // worker only
bool g_ovl_placed = false;                                         // worker only
// Game UI panel (main menu, native_stereo::ui_layer_active): camera.cpp's layer texture is copied into this quad
// swapchain at each Present (copy_present) and shown fixed in LOCAL space, placed in front of the head (yaw only) when
// it appears and on Recenter. Below the mod menu quad.
XrSwapchain g_gui_chain = XR_NULL_HANDLE;    // guarded by g_xr
std::vector<ID3D12Resource*> g_gui_images;   // owned by the runtime
uint32_t g_gui_width = 0, g_gui_height = 0;  // guarded by g_xr
bool g_gui_has_image = false;                // guarded by g_xr
bool g_gui_failed = false;                   // guarded by g_xr
std::atomic<bool> g_gui_visible{false};
std::atomic<bool> g_gui_recentre{false};
std::atomic<float> g_gui_distance{2.f};
std::atomic<float> g_gui_width_m{1.8f};
std::atomic<float> g_gui_height_m{0.f};
std::atomic<float> g_panel_alpha{2.f};  // panel opacity exponent ("panelalpha" in [xr]; record_panel_opacity)
std::atomic<std::uint64_t> g_gui_copies{0}, g_gui_layers{0};
XrPosef g_gui_pose = {{0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, -2.f}};  // worker only
// Menu laser (menu_pointer.hpp): one small teal sprite, written once. A row of it is the beam, all of it the dot.
constexpr uint32_t kLaserTexels = 32;
XrSwapchain g_laser_chain = XR_NULL_HANDLE;  // guarded by g_xr
ID3D12Resource* g_laser_upload = nullptr;
bool g_laser_ready = false, g_laser_failed = false;
head_pose::Hand g_laser_aim[2];                // worker only
float g_laser_head[3] = {};                    // worker only
menu_pointer::Selector g_laser_select;         // worker only
bool g_gui_placed = false;                                         // worker only
// Wrist HUD overlay swapchain; source crops are submitted above the free wrist.
XrSwapchain g_hud_chain = XR_NULL_HANDLE;
std::vector<ID3D12Resource*> g_hud_images;
uint32_t g_hud_width = 0, g_hud_height = 0;
bool g_hud_has_image = false, g_hud_failed = false;
std::atomic<ULONGLONG> g_hud_copied_at{0};
std::atomic<std::uint64_t> g_hud_copies{0}, g_hud_layers{0}, g_hud_arm_frames{0};
uint32_t g_hud_max_layers = 0;
// Cinema screen pose. It used to sit at a fixed point of LOCAL space, (0, 0, -distance); in run 2026-10-05 16:01 the
// head was about 5 m from LOCAL's origin (pose log pos=(-3.64,0.13,-3.81)), so the mono start-up video and menus were
// out of view. It is now placed in front of the head (yaw only) when shown, every time stereo hands over to mono,
// and on recentre.
XrPosef g_cin_pose = {{0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, -2.5f}};  // worker only
int g_cin_follow = 0;                                               // worker only: frames left to keep following the head
std::atomic<bool> g_cin_recentre{true};
// Menu transitions (menu_transition.hpp, 2026-11-08). Instead of following the head for 90 frames, the screen is placed
// once at a steady head pose behind black, or kept where it is on a stereo -> flat handover while still in view; every
// flat <-> stereo switch goes through a black layer. "screenfollow" in [xr] = the old follow, "nofade" = no black layer.
std::atomic<bool> g_cin_lazy{false};      // the pending recentre may keep a screen that is still in view
std::atomic<bool> g_cin_startup{false};   // the pending recentre is the session start (first poses can be stale)
std::atomic<bool> g_screen_follow{false};
std::atomic<bool> g_fade_on{true};
menu_transition::Placer g_cin_placer;     // worker only
menu_transition::Fade g_fade;             // worker only
bool g_cin_has_pose = false;              // worker only: g_cin_pose was placed in this session
// Fade sprite: menu_transition::kLevels black levels of rising opacity, written once like the laser sprite.
XrSwapchain g_fade_chain = XR_NULL_HANDLE;  // guarded by g_xr
ID3D12Resource* g_fade_upload = nullptr;
bool g_fade_ready = false, g_fade_failed = false;
// Pair submission (BioshockVR XR_SubmitPair): the left-eye frame is stashed here; when the right-eye frame
// arrives both swapchains are filled and released together, so the headset never mixes eyes from two pairs.
ID3D12Resource* g_stage = nullptr;  // in COPY_DEST between uses
bool g_stage_valid = false;
// Same-frame pairing ("cinema ... pair"): two hold textures for the second view's image, alternately written and
// read one Present later (guarded by g_xr).
std::atomic<bool> g_pair{false};
std::atomic<uint64_t> g_paired_frames{0};
ID3D12Resource* g_holds[2] = {};
D3D12_RESOURCE_STATES g_hold_states[2] = {D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COPY_DEST};
int g_hold_read = 0;        // the slot holding the previous Present's image
bool g_hold_valid = false;  // g_holds[g_hold_read] holds an image not yet shown

const char* mode_name(Mode m) {
  return m == Mode::Probe ? "probe" : m == Mode::Session ? "session" : m == Mode::Cinema ? "cinema" : "off";
}

void log_line(const char* fmt, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  if (g_log_path.empty()) {
    return;
  }
  SYSTEMTIME t = {};
  GetLocalTime(&t);
  char stamp[32];
  snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u.%03u ", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds);
  std::lock_guard<std::mutex> lock(g_log_mutex);
  runtime_log::append(g_log_path, {stamp, buffer, "\r\n"});
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

// ---- optional per-eye depth for the headset layer (2026-11-12, "xrdepth" in [xr]; see xr_depth.hpp) ----------------
// XR_KHR_composition_layer_depth is enabled whenever the runtime offers it (enabling alone changes nothing); depth is
// written and chained to the projection views only with "xrdepth". Guarded by g_xr like the colour chains.
bool g_depth_ext = false;
std::atomic<bool> g_xr_depth{false};
struct DepthChain {
  XrSwapchain chain = XR_NULL_HANDLE;
  std::vector<ID3D12Resource*> images;  // owned by the runtime
};
DepthChain g_depth_chains[2];
uint32_t g_depth_w = 0, g_depth_h = 0;
bool g_depth_failed = false;
bool g_depth_valid[2] = {false, false};  // this eye's latest released colour image has a matching released depth image
std::atomic<uint64_t> g_depth_submitted{0}, g_depth_stale{0}, g_depth_pq_skips{0}, g_depth_acquire_fails{0},
    g_depth_not_eye_mode{0};

Mode read_control() {
  if (g_control_path.empty()) {
    return Mode::Off;
  }
  char text[192] = {};  // was 96; the [xr] line is ~60 characters and gains opt-out words
  if (!settings_file::read(g_control_path, text, sizeof(text))) {
    return Mode::Off;
  }
  if (std::strncmp(text, "probe", 5) == 0) {
    return Mode::Probe;
  }
  if (std::strncmp(text, "session", 7) == 0) {
    return Mode::Session;
  }
  if (std::strncmp(text, "cinema", 6) == 0) {
    bool swap = false;
    float numbers[2] = {3.0f, 2.5f};
    int found = 0;
    char* cursor = text + 6;
    while (*cursor != '\0') {
      while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') {
        ++cursor;
      }
      if (*cursor == '\0') {
        break;
      }
      if (std::strncmp(cursor, "swap", 4) == 0) {
        swap = true;
        cursor += 4;
        continue;
      }
      char* end = nullptr;
      const float value = std::strtof(cursor, &end);
      if (end == cursor) {
        ++cursor;
        continue;
      }
      if (found < 2 && std::isfinite(value)) {
        numbers[found++] = value;
      }
      cursor = end;
    }
    g_swap_eyes = swap;
    const bool mirror_right = std::strstr(text, "mirrorright") != nullptr;
    if (g_mirror_right.exchange(mirror_right) != mirror_right) {
      log_line("mirror: desktop source %s (headset eye submission unchanged)", mirror_right ? "second view" : "left view");
    }
    g_native = std::strstr(text, "native") != nullptr;
    {
      const bool follow = std::strstr(text, "screenfollow") != nullptr;
      const bool fade = std::strstr(text, "nofade") == nullptr;
      {
        const char* pa = std::strstr(text, "panelalpha");
        const float k = pa != nullptr ? std::strtof(pa + 10, nullptr) : 2.f;
        const float clamped = !(k >= 1.f) ? 1.f : k > 4.f ? 4.f : k;
        if (g_panel_alpha.exchange(clamped) != clamped) {
          log_line("gui: main menu panel opacity k=%.2f (alpha 0.5 -> %.2f)", clamped, 1.f - std::pow(0.5f, clamped));
        }
      }
      static bool logged = false;
      const bool follow_changed = g_screen_follow.exchange(follow) != follow;
      const bool fade_changed = g_fade_on.exchange(fade) != fade;
      if (!logged || follow_changed || fade_changed) {
        logged = true;
        log_line("transition: flat screen %s, black fades %s", follow ? "follows the head for 90 frames (old)" : "placed once",
                 fade ? "on" : "off");
      }
    }
    g_copy_perf_probe = std::strstr(text, "perfprobe") != nullptr;
    {
      const bool depth = std::strstr(text, "xrdepth") != nullptr;  // no other [xr] word contains it
      if (g_xr_depth.exchange(depth) != depth) {
        log_line("xrdepth: per-eye depth to the headset layer %s (runtime extension %s)", depth ? "ON" : "off",
                 g_depth_ext ? "enabled" : "not offered or not yet known");
      }
      xr_depth::set_enabled(depth);
      const bool fg = std::strstr(text, "framegen") != nullptr;  // no other [xr] word contains it
      if (g_framegen.exchange(fg) != fg) {
        log_line("framegen: in-mod frame extrapolation %s", fg ? "ON (headset-rate loop; generated frames between game frames)" : "off");
      }
      frame_gen::set_enabled(fg);
      const bool k0 = std::strstr(text, "framegenk0") != nullptr;
      if (g_framegen_k0.exchange(k0) != k0) log_line("framegen: test mode k0 %s (generated frames = last real frame, no motion)", k0 ? "ON" : "off");
    }
    g_proj = std::strstr(text, "proj") != nullptr;
    g_pair = std::strstr(text, "pair") != nullptr;
    g_half_rate = std::strstr(text, "halfrate") != nullptr;  // opt-in: made motion worse in the headset (2026-10-07)
    g_game_rate = std::strstr(text, "repeat") == nullptr;
    {
      const char* lagl = std::strstr(text, "lagl");
      const char* lag = std::strstr(text, "lag ");
      int right = 1;
      if (lag != nullptr) {
        const long n = std::strtol(lag + 3, nullptr, 10);
        right = (n >= 0 && n <= 6) ? static_cast<int>(n) : 1;
      }
      g_pose_lag = right;
      int left = right + 1;
      if (lagl != nullptr) {
        const long n = std::strtol(lagl + 4, nullptr, 10);
        left = (n >= 0 && n <= 6) ? static_cast<int>(n) : right + 1;
      }
      g_pose_lag_l = left;
    }
    g_screen_width = numbers[0] < 0.5f ? 0.5f : numbers[0] > 20.f ? 20.f : numbers[0];
    g_screen_distance = numbers[1] < 0.5f ? 0.5f : numbers[1] > 30.f ? 30.f : numbers[1];
    return Mode::Cinema;
  }
  return Mode::Off;
}

ID3D12CommandQueue* acquire_queue() {
  std::lock_guard<std::mutex> lock(g_queue_mutex);
  if (g_queue != nullptr) {
    g_queue->AddRef();
  }
  return g_queue;
}

// ---- small helpers ---------------------------------------------------------------------------------

std::string result_text(XrInstance instance, XrResult r) {
  char buffer[XR_MAX_RESULT_STRING_SIZE] = {};
  if (instance != XR_NULL_HANDLE && XR_SUCCEEDED(xrResultToString(instance, r, buffer))) {
    return buffer;
  }
  char fallback[32];
  snprintf(fallback, sizeof(fallback), "XrResult(%d)", static_cast<int>(r));
  return fallback;
}

std::string adapter_name(const LUID& luid) {
  IDXGIFactory4* factory = nullptr;
  if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory))) || factory == nullptr) {
    return "?";
  }
  std::string name = "?";
  IDXGIAdapter1* adapter = nullptr;
  if (SUCCEEDED(factory->EnumAdapterByLuid(luid, IID_PPV_ARGS(&adapter))) && adapter != nullptr) {
    DXGI_ADAPTER_DESC1 desc = {};
    if (SUCCEEDED(adapter->GetDesc1(&desc))) {
      char narrow[128] = {};
      WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, narrow, sizeof(narrow) - 1, nullptr, nullptr);
      name = narrow;
    }
    adapter->Release();
  }
  factory->Release();
  return name;
}

bool same_luid(const LUID& a, const LUID& b) {
  return a.LowPart == b.LowPart && a.HighPart == b.HighPart;
}

void rotate(const XrQuaternionf& q, const float v[3], float out[3]) {
  // v' = v + 2w(u x v) + 2 u x (u x v)
  const float ux = q.x, uy = q.y, uz = q.z, w = q.w;
  const float cx = uy * v[2] - uz * v[1];
  const float cy = uz * v[0] - ux * v[2];
  const float cz = ux * v[1] - uy * v[0];
  const float ccx = uy * cz - uz * cy;
  const float ccy = uz * cx - ux * cz;
  const float ccz = ux * cy - uy * cx;
  out[0] = v[0] + 2.f * (w * cx + ccx);
  out[1] = v[1] + 2.f * (w * cy + ccy);
  out[2] = v[2] + 2.f * (w * cz + ccz);
}

void yaw_pitch(const XrQuaternionf& q, float* yaw_deg, float* pitch_deg) {
  const float forward[3] = {0.f, 0.f, -1.f};  // OpenXR: -Z forward, +Y up
  float f[3];
  rotate(q, forward, f);
  const float rad = 57.2957795f;
  *yaw_deg = std::atan2(-f[0], -f[2]) * rad;
  const float y = f[1] < -1.f ? -1.f : f[1] > 1.f ? 1.f : f[1];
  *pitch_deg = std::asin(y) * rad;
}

// ---- instance / system -----------------------------------------------------------------------------

struct XrBase {
  XrInstance instance = XR_NULL_HANDLE;
  XrSystemId system = XR_NULL_SYSTEM_ID;
  bool has_d3d12 = false;
  PFN_xrGetD3D12GraphicsRequirementsKHR get_d3d12_reqs = nullptr;
  std::vector<std::string> enabled;  // instance extensions actually enabled
};

// ---- lens-edge (hidden-area) mask, step 1: measure (2026-10-23, read-only) ----------------------------------------
// XR_KHR_visibility_mask gives each view's hidden triangle mesh: 2D vertices on the view's z = -1 plane (tangent space).
// Queried once after xrBeginSession; every 10 s the share of each eye's RENDERED frustum it covers is logged, so the
// possible saving is known before anything is written into the game's depth buffer.
bool g_vismask_ext = false;
struct VisMask {
  std::vector<XrVector2f> v;
  std::vector<uint32_t> i;
};
VisMask g_vismask[2];
void query_vismask(XrInstance instance, XrSession session) {
  PFN_xrGetVisibilityMaskKHR get = nullptr;
  if (!g_vismask_ext ||
      XR_FAILED(xrGetInstanceProcAddr(instance, "xrGetVisibilityMaskKHR", reinterpret_cast<PFN_xrVoidFunction*>(&get))) ||
      get == nullptr) {
    log_line("vismask: XR_KHR_visibility_mask not enabled");
    return;
  }
  for (uint32_t e = 0; e < 2; ++e) {
    g_vismask[e] = {};  // no stale mesh across session/runtime recreation
    XrVisibilityMaskKHR m = {XR_TYPE_VISIBILITY_MASK_KHR};
    XrResult r = get(session, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, e, XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR, &m);
    if (XR_SUCCEEDED(r) && m.vertexCountOutput > 0 && m.indexCountOutput > 0) {
      g_vismask[e].v.resize(m.vertexCountOutput);
      g_vismask[e].i.resize(m.indexCountOutput);
      m.vertexCapacityInput = m.vertexCountOutput;
      m.vertices = g_vismask[e].v.data();
      m.indexCapacityInput = m.indexCountOutput;
      m.indices = g_vismask[e].i.data();
      r = get(session, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, e, XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR, &m);
      if (XR_SUCCEEDED(r) && m.vertexCountOutput <= m.vertexCapacityInput && m.indexCountOutput <= m.indexCapacityInput) {
        g_vismask[e].v.resize(m.vertexCountOutput);
        g_vismask[e].i.resize(m.indexCountOutput);
      } else {
        g_vismask[e] = {};
      }
    }
    if (XR_FAILED(r)) {
      g_vismask[e] = {};
    }
    // Some runtimes return one repeated vertex as a placeholder triangle.
    // A zero-area triangle passes all three old sign tests for EVERY pixel.
    // Reject it (and invalid indices/nonfinite data) before estimating coverage.
    auto& mesh = g_vismask[e];
    std::vector<uint32_t> valid;
    for (size_t k = 0; k + 2 < mesh.i.size(); k += 3) {
      const uint32_t a = mesh.i[k], b = mesh.i[k+1], c = mesh.i[k+2];
      if (a >= mesh.v.size() || b >= mesh.v.size() || c >= mesh.v.size()) continue;
      const XrVector2f& va = mesh.v[a];
      const XrVector2f& vb = mesh.v[b];
      const XrVector2f& vc = mesh.v[c];
      const double area = (static_cast<double>(vb.x)-va.x)*(static_cast<double>(vc.y)-va.y) -
                          (static_cast<double>(vb.y)-va.y)*(static_cast<double>(vc.x)-va.x);
      if (!std::isfinite(area) || std::abs(area) <= 1e-12) continue;
      valid.insert(valid.end(), {a, b, c});
    }
    mesh.i.swap(valid);
    if (mesh.i.empty()) log_line("vismask: view %u has no usable hidden triangles; coverage unavailable (not 100%%)", e);
    float lo[2] = {1e9f, 1e9f}, hi[2] = {-1e9f, -1e9f};
    for (const XrVector2f& v : g_vismask[e].v) {
      lo[0] = std::min(lo[0], v.x); lo[1] = std::min(lo[1], v.y);
      hi[0] = std::max(hi[0], v.x); hi[1] = std::max(hi[1], v.y);
    }
    log_line("vismask: view %u hidden mesh %s, %zu vertices, %zu triangles, x %.3f..%.3f y %.3f..%.3f (tangent space)", e,
             result_text(instance, r).c_str(), g_vismask[e].v.size(), g_vismask[e].i.size() / 3, lo[0], hi[0], lo[1], hi[1]);
  }
}
// Share of the rendered frustum (tan left/right/up/down as stored in rtan: l < 0, r > 0, u > 0, d < 0) inside the hidden
// mesh, on a 64 x 64 sample grid (cheap enough for the frame thread every 10 s).
double vismask_share(int e, const float* t) {
  const VisMask& m = g_vismask[e];
  if (m.i.size() < 3) return -1.0;
  int hidden = 0;
  constexpr int kN = 64;
  for (int yi = 0; yi < kN; ++yi) {
    const float y = t[2] + (t[3] - t[2]) * (yi + 0.5f) / kN;
    for (int xi = 0; xi < kN; ++xi) {
      const float x = t[0] + (t[1] - t[0]) * (xi + 0.5f) / kN;
      for (size_t k = 0; k + 2 < m.i.size(); k += 3) {
        const XrVector2f& a = m.v[m.i[k]];
        const XrVector2f& b = m.v[m.i[k + 1]];
        const XrVector2f& c = m.v[m.i[k + 2]];
        const float d1 = (x - b.x) * (a.y - b.y) - (a.x - b.x) * (y - b.y);
        const float d2 = (x - c.x) * (b.y - c.y) - (b.x - c.x) * (y - c.y);
        const float d3 = (x - a.x) * (c.y - a.y) - (c.x - a.x) * (y - a.y);
        const bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
        if (!(neg && pos)) {
          ++hidden;
          break;
        }
      }
    }
  }
  return 100.0 * hidden / (kN * kN);
}


// ---- frame generation status -------------------------------------------------------------------------------------
// Reports this mod's own rendered and generated frames per second.
char g_framegen_status[192] = "Frame generation (mod): off";
ULONGLONG g_framegen_checked = 0;

void update_framegen_status() {
  const ULONGLONG now = GetTickCount64();
  if (now - g_framegen_checked < 1000) return;
  const double seconds = g_framegen_checked == 0 ? 1.0 : static_cast<double>(now - g_framegen_checked) / 1000.0;
  g_framegen_checked = now;
  static uint64_t last_real = 0, last_gen = 0;
  const uint64_t real = g_fg_real.load(), gen = g_fg_generated.load();
  const double real_ps = static_cast<double>(real - last_real) / seconds, gen_ps = static_cast<double>(gen - last_gen) / seconds;
  last_real = real;
  last_gen = gen;
  char text[192];
  if (!g_framegen.load(std::memory_order_relaxed)) {
    snprintf(text, sizeof(text), "Frame generation (mod): off");
  } else {
    const frame_gen::Stats s = frame_gen::stats();
    snprintf(text, sizeof(text), "Frame generation (mod): on, game %.0f + generated %.0f per s, warp %.2f ms", real_ps, gen_ps,
             s.gpu_ms);
    static int reports = 0;
    if (++reports % 5 == 0) {
      log_line("framegen: real=%llu generated=%llu skipped not_ready=%llu old=%llu acquire=%llu | kept=%llu not_kept=%llu "
               "captures main=%llu second=%llu skipped_layout=%llu gen_failures=%llu warp_gpu=%.3f ms gen_luma=%.1f/%.1f%s "
               "| hud: copies=%llu steady_frames=%llu unusable=%llu",
               static_cast<unsigned long long>(real), static_cast<unsigned long long>(gen),
               static_cast<unsigned long long>(g_fg_skip_not_ready.load()), static_cast<unsigned long long>(g_fg_skip_old.load()),
               static_cast<unsigned long long>(g_fg_skip_acquire.load()), static_cast<unsigned long long>(s.kept),
               static_cast<unsigned long long>(s.not_kept), static_cast<unsigned long long>(s.captures[0]),
               static_cast<unsigned long long>(s.captures[1]), static_cast<unsigned long long>(s.skipped_layout),
               static_cast<unsigned long long>(s.gen_failures), s.gpu_ms, s.luma[0], s.luma[1],
               g_framegen_k0.load() ? " (k0 test)" : "", static_cast<unsigned long long>(s.prehud_captures),
               static_cast<unsigned long long>(s.hud_frames), static_cast<unsigned long long>(s.hud_mismatch));
    }
  }
  std::strncpy(g_framegen_status, text, sizeof(g_framegen_status) - 1);
}

// Which API layers the loader will put into this instance, and whether this process is elevated (the loader then
// skips per-user layers, e.g. OFXR's). Logged once at instance creation.
void log_api_layers() {
  uint32_t count = 0;
  std::string names;
  if (XR_SUCCEEDED(xrEnumerateApiLayerProperties(0, &count, nullptr)) && count > 0) {
    std::vector<XrApiLayerProperties> props(count, {XR_TYPE_API_LAYER_PROPERTIES});
    if (XR_SUCCEEDED(xrEnumerateApiLayerProperties(count, &count, props.data()))) {
      for (const auto& p : props) {
        names += p.layerName;
        names += ' ';
      }
    }
  }
  bool elevated = false;
  HANDLE token = nullptr;
  if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token)) {
    TOKEN_ELEVATION te = {};
    DWORD size = 0;
    if (GetTokenInformation(token, TokenElevation, &te, sizeof(te), &size)) elevated = te.TokenIsElevated != 0;
    CloseHandle(token);
  }
  log_line("api layers (%u): %s| process elevated=%d%s", count, names.empty() ? "none " : names.c_str(), elevated ? 1 : 0,
           elevated ? " (the loader skips per-user layers such as OFXR's)" : "");
}

bool create_instance(XrBase* base) {
  uint32_t count = 0;
  XrResult r = xrEnumerateInstanceExtensionProperties(nullptr, 0, &count, nullptr);
  if (XR_FAILED(r)) {
    log_line("xrEnumerateInstanceExtensionProperties failed %d (no OpenXR runtime installed or active?)", static_cast<int>(r));
    return false;
  }
  std::vector<XrExtensionProperties> exts(count, {XR_TYPE_EXTENSION_PROPERTIES});
  xrEnumerateInstanceExtensionProperties(nullptr, count, &count, exts.data());
  std::string list;
  for (const auto& e : exts) {
    if (std::strcmp(e.extensionName, XR_KHR_D3D12_ENABLE_EXTENSION_NAME) == 0) {
      base->has_d3d12 = true;
    }
    if (list.size() < 700) {
      list += e.extensionName;
      list += ' ';
    }
  }
  log_line("runtime extensions (%u): %s", count, list.c_str());
  if (!base->has_d3d12) {
    log_line("runtime does NOT offer %s; D3D12 sessions are impossible on this runtime", XR_KHR_D3D12_ENABLE_EXTENSION_NAME);
  }

  // D3D12 plus the controller-profile extensions src/vr/xr_input.cpp has bindings for, when offered.
  std::vector<const char*> enabled;
  if (base->has_d3d12) {
    enabled.push_back(XR_KHR_D3D12_ENABLE_EXTENSION_NAME);
  }
  for (const auto& e : exts) {
    if (std::strcmp(e.extensionName, XR_KHR_VISIBILITY_MASK_EXTENSION_NAME) == 0) {
      enabled.push_back(XR_KHR_VISIBILITY_MASK_EXTENSION_NAME);
      g_vismask_ext = true;
      break;
    }
  }
  for (const auto& e : exts) {
    if (std::strcmp(e.extensionName, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME) == 0) {
      enabled.push_back(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
      g_depth_ext = true;
      break;
    }
  }
  for (const char* want : xr_input::optional_extensions()) {
    for (const auto& e : exts) {
      if (std::strcmp(e.extensionName, want) == 0) {
        enabled.push_back(want);
        break;
      }
    }
  }
  base->enabled.assign(enabled.begin(), enabled.end());
  XrInstanceCreateInfo info = {XR_TYPE_INSTANCE_CREATE_INFO};
  std::strncpy(info.applicationInfo.applicationName, "AvatarVR", XR_MAX_APPLICATION_NAME_SIZE - 1);
  info.applicationInfo.applicationVersion = 1;
  std::strncpy(info.applicationInfo.engineName, "AvatarVR", XR_MAX_ENGINE_NAME_SIZE - 1);
  info.applicationInfo.engineVersion = 1;
  info.applicationInfo.apiVersion = XR_API_VERSION_1_0;
  info.enabledExtensionCount = static_cast<uint32_t>(enabled.size());
  info.enabledExtensionNames = enabled.empty() ? nullptr : enabled.data();
  log_api_layers();
  r = xrCreateInstance(&info, &base->instance);
  if (XR_FAILED(r)) {
    log_line("xrCreateInstance failed %d", static_cast<int>(r));
    base->instance = XR_NULL_HANDLE;
    return false;
  }
  XrInstanceProperties props = {XR_TYPE_INSTANCE_PROPERTIES};
  if (XR_SUCCEEDED(xrGetInstanceProperties(base->instance, &props))) {
    log_line("runtime: %s %u.%u.%u", props.runtimeName,
             static_cast<unsigned>(XR_VERSION_MAJOR(props.runtimeVersion)),
             static_cast<unsigned>(XR_VERSION_MINOR(props.runtimeVersion)),
             static_cast<unsigned>(XR_VERSION_PATCH(props.runtimeVersion)));
  }
  if (base->has_d3d12) {
    xrGetInstanceProcAddr(base->instance, "xrGetD3D12GraphicsRequirementsKHR",
                          reinterpret_cast<PFN_xrVoidFunction*>(&base->get_d3d12_reqs));
  }
  return true;
}

bool get_system(XrBase* base) {
  XrSystemGetInfo info = {XR_TYPE_SYSTEM_GET_INFO};
  info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  const XrResult r = xrGetSystem(base->instance, &info, &base->system);
  if (XR_FAILED(r)) {
    log_line("xrGetSystem(HMD) failed: %s (headset off, asleep, or not connected?)", result_text(base->instance, r).c_str());
    return false;
  }
  XrSystemProperties props = {XR_TYPE_SYSTEM_PROPERTIES};
  if (XR_SUCCEEDED(xrGetSystemProperties(base->instance, base->system, &props))) {
    g_hud_max_layers = props.graphicsProperties.maxLayerCount;
    log_line("system: %s vendor=0x%X maxSwapchain=%ux%u maxLayers=%u orientationTracking=%u positionTracking=%u",
             props.systemName, props.vendorId, props.graphicsProperties.maxSwapchainImageWidth,
             props.graphicsProperties.maxSwapchainImageHeight, props.graphicsProperties.maxLayerCount,
             static_cast<unsigned>(props.trackingProperties.orientationTracking),
             static_cast<unsigned>(props.trackingProperties.positionTracking));
  }
  uint32_t count = 0;
  xrEnumerateViewConfigurationViews(base->instance, base->system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &count, nullptr);
  std::vector<XrViewConfigurationView> views(count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
  if (count > 0 && XR_SUCCEEDED(xrEnumerateViewConfigurationViews(base->instance, base->system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, count, &count, views.data()))) {
    for (uint32_t i = 0; i < count; ++i) {
      log_line("view %u: recommended %ux%u (max %ux%u) samples %u", i, views[i].recommendedImageRectWidth,
               views[i].recommendedImageRectHeight, views[i].maxImageRectWidth, views[i].maxImageRectHeight,
               views[i].recommendedSwapchainSampleCount);
    }
    g_rec_w.store(views[0].recommendedImageRectWidth, std::memory_order_relaxed);
    g_rec_h.store(views[0].recommendedImageRectHeight, std::memory_order_relaxed);
  }
  uint32_t blend_count = 0;
  xrEnumerateEnvironmentBlendModes(base->instance, base->system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &blend_count, nullptr);
  std::vector<XrEnvironmentBlendMode> blends(blend_count);
  if (blend_count > 0) {
    xrEnumerateEnvironmentBlendModes(base->instance, base->system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, blend_count, &blend_count, blends.data());
    log_line("blend modes: first=%d count=%u", static_cast<int>(blends[0]), blend_count);
  }
  return true;
}

// Returns true when the headset can take frames from the given device (same adapter).
bool check_gpu(const XrBase& base, ID3D12Device* device, LUID* required) {
  if (base.get_d3d12_reqs == nullptr) {
    log_line("no xrGetD3D12GraphicsRequirementsKHR");
    return false;
  }
  XrGraphicsRequirementsD3D12KHR reqs = {XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR};
  const XrResult r = base.get_d3d12_reqs(base.instance, base.system, &reqs);
  if (XR_FAILED(r)) {
    log_line("xrGetD3D12GraphicsRequirementsKHR failed: %s", result_text(base.instance, r).c_str());
    return false;
  }
  *required = reqs.adapterLuid;
  log_line("headset requires GPU luid=%08lX:%08lX \"%s\" minFeatureLevel=0x%X",
           static_cast<unsigned long>(reqs.adapterLuid.HighPart), static_cast<unsigned long>(reqs.adapterLuid.LowPart),
           adapter_name(reqs.adapterLuid).c_str(), static_cast<unsigned>(reqs.minFeatureLevel));
  if (device == nullptr) {
    log_line("game device not known yet (no swapchain seen); GPU match not checked");
    return false;
  }
  const LUID game = device->GetAdapterLuid();
  const bool match = same_luid(game, reqs.adapterLuid);
  log_line("game renders on GPU luid=%08lX:%08lX \"%s\" -> %s", static_cast<unsigned long>(game.HighPart),
           static_cast<unsigned long>(game.LowPart), adapter_name(game).c_str(),
           match ? "MATCH" : "MISMATCH (headset is driven by a different GPU; a D3D12 session on the game device will fail)");
  return match;
}

ID3D12Device* device_of(ID3D12CommandQueue* queue) {
  if (queue == nullptr) {
    return nullptr;
  }
  ID3D12Device* device = nullptr;
  if (FAILED(queue->GetDevice(IID_PPV_ARGS(&device)))) {
    return nullptr;
  }
  return device;
}

// ---- probe -----------------------------------------------------------------------------------------

void run_probe() {
  log_line("probe: start");
  XrBase base;
  if (!create_instance(&base)) {
    return;
  }
  if (get_system(&base)) {
    ID3D12CommandQueue* queue = acquire_queue();
    ID3D12Device* device = device_of(queue);
    LUID required = {};
    check_gpu(base, device, &required);
    if (device != nullptr) {
      device->Release();
    }
    if (queue != nullptr) {
      queue->Release();
    }
  }
  xrDestroyInstance(base.instance);
  log_line("probe: done (instance destroyed)");
}

// ---- cinema: per-eye swapchains and the Present-side copy ---------------------------------------------

void destroy_overlay_locked() {
  std::lock_guard<std::recursive_mutex> chains(g_chains);
  if (g_ovl_chain != XR_NULL_HANDLE) {
    xrDestroySwapchain(g_ovl_chain);
  }
  g_ovl_chain = XR_NULL_HANDLE;
  g_ovl_images.clear();
  g_ovl_width = g_ovl_height = 0;
  g_ovl_has_image = false;
  g_ovl_failed = false;
  g_ovl_acquired = false;
}

bool ensure_overlay_locked(uint32_t width, uint32_t height) {
  if (g_ovl_chain != XR_NULL_HANDLE && g_ovl_width == width && g_ovl_height == height) {
    return true;
  }
  if (g_ovl_failed || g_session == XR_NULL_HANDLE) {
    return false;
  }
  destroy_overlay_locked();
  uint32_t count = 0;
  xrEnumerateSwapchainFormats(g_session, 0, &count, nullptr);
  std::vector<int64_t> formats(count);
  if (count == 0 || XR_FAILED(xrEnumerateSwapchainFormats(g_session, count, &count, formats.data()))) {
    g_ovl_failed = true;
    return false;
  }
  // The canvas is R8G8B8A8 (sRGB-encoded bytes, premultiplied alpha); the _SRGB twin makes the runtime decode it.
  int64_t chosen = 0;
  for (int64_t f : formats) {
    if (f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
      chosen = f;
    }
  }
  for (int64_t f : formats) {
    if (chosen == 0 && f == DXGI_FORMAT_R8G8B8A8_UNORM) {
      chosen = f;
    }
  }
  if (chosen == 0) {
    log_line("menu: runtime offers no R8G8B8A8 swapchain format; overlay disabled");
    g_ovl_failed = true;
    return false;
  }
  XrSwapchainCreateInfo info = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
  info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
  info.format = chosen;
  info.sampleCount = 1;
  info.width = width;
  info.height = height;
  info.faceCount = 1;
  info.arraySize = 1;
  info.mipCount = 1;
  XrResult r = xrCreateSwapchain(g_session, &info, &g_ovl_chain);
  if (XR_FAILED(r)) {
    log_line("menu: xrCreateSwapchain failed: %s", result_text(g_instance, r).c_str());
    g_ovl_chain = XR_NULL_HANDLE;
    g_ovl_failed = true;
    return false;
  }
  uint32_t images = 0;
  xrEnumerateSwapchainImages(g_ovl_chain, 0, &images, nullptr);
  std::vector<XrSwapchainImageD3D12KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
  r = xrEnumerateSwapchainImages(g_ovl_chain, images, &images, reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
  if (XR_FAILED(r) || images == 0) {
    log_line("menu: xrEnumerateSwapchainImages failed: %s", result_text(g_instance, r).c_str());
    destroy_overlay_locked();
    g_ovl_failed = true;
    return false;
  }
  for (const auto& image : list) {
    g_ovl_images.push_back(image.texture);
  }
  g_ovl_width = width;
  g_ovl_height = height;
  log_line("menu: overlay swapchain %ux%u format=%lld images=%u", width, height, static_cast<long long>(chosen), images);
  return true;
}

void destroy_gui_locked() {
  std::lock_guard<std::recursive_mutex> chains(g_chains);
  if (g_gui_chain != XR_NULL_HANDLE) {
    xrDestroySwapchain(g_gui_chain);
  }
  g_gui_chain = XR_NULL_HANDLE;
  g_gui_images.clear();
  g_gui_width = g_gui_height = 0;
  g_gui_has_image = false;
  g_gui_failed = false;
}

void destroy_laser_locked() {
  std::lock_guard<std::recursive_mutex> chains(g_chains);
  if (g_laser_chain != XR_NULL_HANDLE) {
    xrDestroySwapchain(g_laser_chain);
  }
  g_laser_chain = XR_NULL_HANDLE;
  if (g_laser_upload != nullptr) {
    g_laser_upload->Release();
    g_laser_upload = nullptr;
  }
  g_laser_ready = false;
  g_laser_failed = false;
}

void destroy_hud_locked() {
  std::lock_guard<std::recursive_mutex> chains(g_chains);
  if (g_hud_chain != XR_NULL_HANDLE) {
    xrDestroySwapchain(g_hud_chain);
  }
  g_hud_chain = XR_NULL_HANDLE;
  g_hud_images.clear();
  g_hud_width = g_hud_height = 0;
  g_hud_has_image = false;
  g_hud_failed = false;
}

bool ensure_hud_locked(uint32_t width, uint32_t height) {
  if (g_hud_chain != XR_NULL_HANDLE && g_hud_width == width && g_hud_height == height) {
    return true;
  }
  if (g_hud_failed || g_session == XR_NULL_HANDLE) {
    return false;
  }
  destroy_hud_locked();
  uint32_t count = 0;
  xrEnumerateSwapchainFormats(g_session, 0, &count, nullptr);
  std::vector<int64_t> formats(count);
  if (count == 0 || XR_FAILED(xrEnumerateSwapchainFormats(g_session, count, &count, formats.data()))) {
    g_hud_failed = true;
    return false;
  }
  int64_t chosen = 0;
  for (int64_t f : formats) {
    if (f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
      chosen = f;
    }
  }
  for (int64_t f : formats) {
    if (chosen == 0 && f == DXGI_FORMAT_R8G8B8A8_UNORM) {
      chosen = f;
    }
  }
  if (chosen == 0) {
    log_line("hud: runtime offers no R8G8B8A8 swapchain format; wrist HUD disabled");
    g_hud_failed = true;
    return false;
  }
  XrSwapchainCreateInfo info = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
  info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
  info.format = chosen;
  info.sampleCount = 1;
  info.width = width;
  info.height = height;
  info.faceCount = 1;
  info.arraySize = 1;
  info.mipCount = 1;
  XrResult r = xrCreateSwapchain(g_session, &info, &g_hud_chain);
  if (XR_FAILED(r)) {
    log_line("hud: xrCreateSwapchain failed: %s", result_text(g_instance, r).c_str());
    g_hud_chain = XR_NULL_HANDLE;
    g_hud_failed = true;
    return false;
  }
  uint32_t images = 0;
  xrEnumerateSwapchainImages(g_hud_chain, 0, &images, nullptr);
  std::vector<XrSwapchainImageD3D12KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
  r = xrEnumerateSwapchainImages(g_hud_chain, images, &images, reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
  if (XR_FAILED(r) || images == 0) {
    log_line("hud: xrEnumerateSwapchainImages failed: %s", result_text(g_instance, r).c_str());
    destroy_hud_locked();
    g_hud_failed = true;
    return false;
  }
  for (const auto& image : list) {
    g_hud_images.push_back(image.texture);
  }
  g_hud_width = width;
  g_hud_height = height;
  log_line("hud: wrist HUD swapchain %ux%u format=%lld images=%u", width, height, static_cast<long long>(chosen), images);
  return true;
}

XrQuaternionf quat_mul(const XrQuaternionf& a, const XrQuaternionf& b) {
  return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
          a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};
}
XrPosef wrist_pose(const head_pose::Hand& grip, const XrVector3f& offset) {
  const XrQuaternionf g={grip.qx,grip.qy,grip.qz,grip.qw};
  const auto v=quat_mul(quat_mul(g,{offset.x,offset.y,offset.z,0.f}),{-g.x,-g.y,-g.z,g.w});
  const XrQuaternionf face_up={-0.70710678f,0.f,0.f,0.70710678f};
  return {quat_mul(g,face_up),{grip.px+v.x,grip.py+v.y,grip.pz+v.z}};
}
// A point in grip axes (x right, y up, z forward) to LOCAL space through the grip pose (-z forward).
XrVector3f grip_point(const head_pose::Hand& grip, const float* p) {
  const XrQuaternionf g={grip.qx,grip.qy,grip.qz,grip.qw};
  const auto v=quat_mul(quat_mul(g,{p[0],p[1],-p[2],0.f}),{-g.x,-g.y,-g.z,g.w});
  return {grip.px+v.x,grip.py+v.y,grip.pz+v.z};
}
// On the drawn forearm: `along` metres from the wrist toward the elbow, `lift` metres toward the viewer.
XrPosef arm_pose(const head_pose::Hand& grip, const float* wrist, const float* elbow, float along, float lift,
                 const head_pose::Eye& viewer) {
  const XrVector3f w=grip_point(grip,wrist), e=grip_point(grip,elbow);
  const XrVector3f d={e.x-w.x,e.y-w.y,e.z-w.z};
  const float length=std::max(0.001f,std::sqrt(d.x*d.x+d.y*d.y+d.z*d.z));
  XrVector3f p={w.x+d.x*along/length,w.y+d.y*along/length,w.z+d.z*along/length};
  const XrVector3f t={viewer.px-p.x,viewer.py-p.y,viewer.pz-p.z};
  const float reach=std::max(0.001f,std::sqrt(t.x*t.x+t.y*t.y+t.z*t.z));
  p={p.x+t.x*lift/reach,p.y+t.y*lift/reach,p.z+t.z*lift/reach};
  const XrQuaternionf face_up={-0.70710678f,0.f,0.f,0.70710678f};
  return {quat_mul({grip.qx,grip.qy,grip.qz,grip.qw},face_up),p};
}

// One asynchronous readback per process, after a second of HUD copies. Establish whether the transported
// texture contains RGB and alpha before changing blend state or guessing new crop positions.
ID3D12Resource* g_hud_readback = nullptr;
ID3D12Fence* g_hud_readback_done = nullptr;  // retain the submission fence across XR session recreation
D3D12_PLACED_SUBRESOURCE_FOOTPRINT g_hud_footprint = {};
UINT64 g_hud_readback_bytes = 0, g_hud_readback_fence = 0;
bool g_hud_capture_attempted = false;

bool hud_capture_record(ID3D12Resource* source) {
  if (g_hud_capture_attempted || g_hud_copies.load() < 60 || !g_copy_device) return false;
  g_hud_capture_attempted = true;
  const auto desc = source->GetDesc();
  if (desc.Format != DXGI_FORMAT_R8G8B8A8_UNORM || desc.Width != 1920 || desc.Height != 1080) return false;
  g_copy_device->GetCopyableFootprints(&desc, 0, 1, 0, &g_hud_footprint, nullptr, nullptr, &g_hud_readback_bytes);
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC buffer = {};
  buffer.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  buffer.Width = g_hud_readback_bytes;
  buffer.Height = 1;
  buffer.DepthOrArraySize = buffer.MipLevels = 1;
  buffer.SampleDesc.Count = 1;
  buffer.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  if (FAILED(g_copy_device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &buffer,
      D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&g_hud_readback)))) {
    log_line("wrist hud capture: readback allocation failed");
    return false;
  }
  D3D12_TEXTURE_COPY_LOCATION dst = {}, src = {};
  dst.pResource = g_hud_readback;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = g_hud_footprint;
  src.pResource = source;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);  // source promotes from COMMON to COPY_SOURCE
  return true;
}

void hud_capture_collect() {
  if (!g_hud_readback || !g_hud_readback_fence || !g_hud_readback_done) return;
  const UINT64 completed = g_hud_readback_done->GetCompletedValue();
  if (completed == UINT64_MAX) {
    g_hud_readback->Release(); g_hud_readback = nullptr;
    g_hud_readback_done->Release(); g_hud_readback_done = nullptr;
    log_line("wrist hud capture: device removed");
    return;
  }
  if (completed < g_hud_readback_fence) return;
  g_hud_readback_done->Release(); g_hud_readback_done = nullptr;
  void* mapped = nullptr;
  D3D12_RANGE range = {0, static_cast<SIZE_T>(g_hud_readback_bytes)};
  if (FAILED(g_hud_readback->Map(0, &range, &mapped))) {
    log_line("wrist hud capture: readback map failed");
    g_hud_readback->Release(); g_hud_readback = nullptr;
    return;
  }
  const auto* bytes = static_cast<const unsigned char*>(mapped);
  const UINT w = g_hud_footprint.Footprint.Width, h = g_hud_footprint.Footprint.Height;
  std::vector<unsigned char> rgb(static_cast<size_t>(w) * h * 4), alpha(rgb.size());
  UINT rgb_count = 0, alpha_count = 0, max_a = 0;
  UINT left = w, top = h, right = 0, bottom = 0;
  for (UINT y = 0; y < h; ++y) {
    const auto* row = bytes + g_hud_footprint.Offset + static_cast<size_t>(y) * g_hud_footprint.Footprint.RowPitch;
    for (UINT x = 0; x < w; ++x) {
      const auto* p = row + x * 4;
      const size_t i = (static_cast<size_t>(y) * w + x) * 4;
      rgb[i] = p[2]; rgb[i + 1] = p[1]; rgb[i + 2] = p[0]; rgb[i + 3] = 255;
      alpha[i] = alpha[i + 1] = alpha[i + 2] = p[3]; alpha[i + 3] = 255;
      if (p[0] || p[1] || p[2]) {
        ++rgb_count; left = std::min(left, x); top = std::min(top, y);
        right = std::max(right, x); bottom = std::max(bottom, y);
      }
      if (p[3]) ++alpha_count;
      max_a = std::max(max_a, static_cast<UINT>(p[3]));
    }
  }
  D3D12_RANGE unchanged = {0, 0};
  g_hud_readback->Unmap(0, &unchanged);
  g_hud_readback->Release(); g_hud_readback = nullptr;
  const std::wstring dir = self_dir() + L"\\AvatarVR\\captures";
  CreateDirectoryW(dir.c_str(), nullptr);
  auto save = [&](const wchar_t* name, const std::vector<unsigned char>& pixels) {
    HANDLE file = CreateFileW((dir + name).c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) { log_line("wrist hud capture: file creation failed"); return; }
    BITMAPFILEHEADER fh = {};
    BITMAPINFOHEADER ih = {};
    fh.bfType = 0x4d42; fh.bfOffBits = sizeof(fh) + sizeof(ih); fh.bfSize = fh.bfOffBits + static_cast<DWORD>(pixels.size());
    ih.biSize = sizeof(ih); ih.biWidth = w; ih.biHeight = -static_cast<LONG>(h); ih.biPlanes = 1; ih.biBitCount = 32;
    ih.biCompression = BI_RGB; ih.biSizeImage = static_cast<DWORD>(pixels.size());
    DWORD wrote = 0;
    const bool ok = WriteFile(file, &fh, sizeof(fh), &wrote, nullptr) && wrote == sizeof(fh) &&
        WriteFile(file, &ih, sizeof(ih), &wrote, nullptr) && wrote == sizeof(ih) &&
        WriteFile(file, pixels.data(), ih.biSizeImage, &wrote, nullptr) && wrote == ih.biSizeImage;
    CloseHandle(file);
    if (!ok) log_line("wrist hud capture: incomplete file write");
  };
  save(L"\\wrist_hud_rgb.bmp", rgb);
  save(L"\\wrist_hud_alpha.bmp", alpha);
  log_line("wrist hud capture: %ux%u rgbPixels=%u alphaPixels=%u maxAlpha=%u rgbBounds=%u,%u..%u,%u (captures/wrist_hud_rgb.bmp + alpha.bmp)",
           w, h, rgb_count, alpha_count, max_a, left, top, right, bottom);
  wrist_hud::image_ready(rgb_count > 0 && alpha_count > 0);
}

// Same formats as the mod menu overlay: the layer holds display-encoded RGBA8 (like the backbuffer it replaces), so
// the _SRGB twin makes the runtime decode it.
bool ensure_gui_locked(uint32_t width, uint32_t height) {
  if (g_gui_chain != XR_NULL_HANDLE && g_gui_width == width && g_gui_height == height) {
    return true;
  }
  if (g_gui_failed || g_session == XR_NULL_HANDLE) {
    return false;
  }
  destroy_gui_locked();
  uint32_t count = 0;
  xrEnumerateSwapchainFormats(g_session, 0, &count, nullptr);
  std::vector<int64_t> formats(count);
  if (count == 0 || XR_FAILED(xrEnumerateSwapchainFormats(g_session, count, &count, formats.data()))) {
    g_gui_failed = true;
    return false;
  }
  int64_t chosen = 0;
  for (int64_t f : formats) {
    if (f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
      chosen = f;
    }
  }
  for (int64_t f : formats) {
    if (chosen == 0 && f == DXGI_FORMAT_R8G8B8A8_UNORM) {
      chosen = f;
    }
  }
  if (chosen == 0) {
    log_line("gui: runtime offers no R8G8B8A8 swapchain format; game UI panel disabled");
    g_gui_failed = true;
    return false;
  }
  XrSwapchainCreateInfo info = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
  info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
  info.format = chosen;
  info.sampleCount = 1;
  info.width = width;
  info.height = height;
  info.faceCount = 1;
  info.arraySize = 1;
  info.mipCount = 1;
  XrResult r = xrCreateSwapchain(g_session, &info, &g_gui_chain);
  if (XR_FAILED(r)) {
    log_line("gui: xrCreateSwapchain failed: %s", result_text(g_instance, r).c_str());
    g_gui_chain = XR_NULL_HANDLE;
    g_gui_failed = true;
    return false;
  }
  uint32_t images = 0;
  xrEnumerateSwapchainImages(g_gui_chain, 0, &images, nullptr);
  std::vector<XrSwapchainImageD3D12KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
  r = xrEnumerateSwapchainImages(g_gui_chain, images, &images, reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
  if (XR_FAILED(r) || images == 0) {
    log_line("gui: xrEnumerateSwapchainImages failed: %s", result_text(g_instance, r).c_str());
    destroy_gui_locked();
    g_gui_failed = true;
    return false;
  }
  for (const auto& image : list) {
    g_gui_images.push_back(image.texture);
  }
  g_gui_width = width;
  g_gui_height = height;
  log_line("gui: game UI panel swapchain %ux%u format=%lld images=%u", width, height, static_cast<long long>(chosen), images);
  return true;
}

void transition(ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after);

// A small sprite swapchain written once: picks an R8G8B8A8 format, fills an upload buffer with `fill` (premultiplied,
// display-encoded RGBA rows) and records the copy on the copy list; the image stays acquired until the list is
// submitted. The runtime keeps showing the last released image, so it is never acquired again. True = release it.
template <typename Fill>
bool record_sprite_locked(ID3D12Device* device, const char* name, uint32_t width, uint32_t height, XrSwapchain* chain,
                          ID3D12Resource** upload, Fill fill) {
  uint32_t count = 0;
  xrEnumerateSwapchainFormats(g_session, 0, &count, nullptr);
  std::vector<int64_t> formats(count);
  int64_t chosen = 0;
  if (count == 0 || XR_FAILED(xrEnumerateSwapchainFormats(g_session, count, &count, formats.data()))) {
    return false;
  }
  for (int64_t f : formats) {
    if (f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
      chosen = f;
    }
  }
  for (int64_t f : formats) {
    if (chosen == 0 && f == DXGI_FORMAT_R8G8B8A8_UNORM) {
      chosen = f;
    }
  }
  if (chosen == 0) {
    log_line("%s: runtime offers no R8G8B8A8 swapchain format; not drawn", name);
    return false;
  }
  XrSwapchainCreateInfo info = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
  info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
  info.format = chosen;
  info.sampleCount = 1;
  info.width = width;
  info.height = height;
  info.faceCount = info.arraySize = info.mipCount = 1;
  XrResult r = xrCreateSwapchain(g_session, &info, chain);
  if (XR_FAILED(r)) {
    log_line("%s: xrCreateSwapchain failed: %s", name, result_text(g_instance, r).c_str());
    *chain = XR_NULL_HANDLE;
    return false;
  }
  uint32_t images = 0;
  xrEnumerateSwapchainImages(*chain, 0, &images, nullptr);
  std::vector<XrSwapchainImageD3D12KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
  r = xrEnumerateSwapchainImages(*chain, images, &images, reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
  const uint32_t align = D3D12_TEXTURE_DATA_PITCH_ALIGNMENT;
  const uint32_t pitch = (width * 4 + align - 1) / align * align;
  D3D12_HEAP_PROPERTIES heap = {D3D12_HEAP_TYPE_UPLOAD};
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = static_cast<UINT64>(pitch) * height;
  desc.Height = desc.DepthOrArraySize = desc.MipLevels = 1;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  unsigned char* mapped = nullptr;
  if (XR_FAILED(r) || images == 0 ||
      FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
                                             IID_PPV_ARGS(upload))) ||
      FAILED((*upload)->Map(0, nullptr, reinterpret_cast<void**>(&mapped)))) {
    log_line("%s: sprite setup failed", name);
    if (*upload != nullptr) {
      (*upload)->Release();
      *upload = nullptr;
    }
    xrDestroySwapchain(*chain);
    *chain = XR_NULL_HANDLE;
    return false;
  }
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      fill(x, y, mapped + y * pitch + x * 4);
    }
  }
  (*upload)->Unmap(0, nullptr);
  uint32_t index = 0;
  XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
  XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
  wait_info.timeout = 50000000;  // 50 ms
  if (XR_FAILED(xrAcquireSwapchainImage(*chain, &acquire_info, &index)) || index >= images ||
      xrWaitSwapchainImage(*chain, &wait_info) != XR_SUCCESS) {
    log_line("%s: first image not available", name);
    return false;  // the caller destroys the chain and upload
  }
  ID3D12Resource* target = list[index].texture;
  transition(target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = target;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = *upload;
  src.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  // A swapchain of the _SRGB format copies from the same bytes: the sprite is already display-encoded.
  src.PlacedFootprint.Footprint = {static_cast<DXGI_FORMAT>(chosen), width, height, 1, pitch};
  g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  transition(target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
  log_line("%s: sprite swapchain %ux%u format=%lld images=%u", name, width, height, static_cast<long long>(chosen), images);
  return true;
}

// Records the laser sprite's one upload (see record_sprite_locked). True = release it after the list is submitted.
bool record_laser_locked(ID3D12Device* device) {
  if (g_laser_ready || g_laser_failed || g_session == XR_NULL_HANDLE) {
    return false;
  }
  g_laser_failed = true;  // cleared on success; no retry every frame
  const bool ok = record_sprite_locked(device, "laser", kLaserTexels, kLaserTexels, &g_laser_chain, &g_laser_upload,
                                       [](uint32_t x, uint32_t y, unsigned char* t) {
    const float dx = (static_cast<float>(x) + 0.5f) / (0.5f * kLaserTexels) - 1.f;
    const float dy = (static_cast<float>(y) + 0.5f) / (0.5f * kLaserTexels) - 1.f;
    const float d = std::sqrt(dx * dx + dy * dy);
    const float a = d >= 1.f ? 0.f : d <= 0.45f ? 1.f : (1.f - d) / 0.55f;
    t[0] = static_cast<unsigned char>(0x20 * a);  // premultiplied blue-teal
    t[1] = static_cast<unsigned char>(0xC8 * a);
    t[2] = static_cast<unsigned char>(0xE0 * a);
    t[3] = static_cast<unsigned char>(0xFF * a);
  });
  if (!ok && g_laser_chain != XR_NULL_HANDLE) {
    destroy_laser_locked();
    g_laser_failed = true;
  }
  return ok;
}

void destroy_fade_locked() {
  std::lock_guard<std::recursive_mutex> chains(g_chains);
  if (g_fade_chain != XR_NULL_HANDLE) {
    xrDestroySwapchain(g_fade_chain);
  }
  g_fade_chain = XR_NULL_HANDLE;
  if (g_fade_upload != nullptr) {
    g_fade_upload->Release();
    g_fade_upload = nullptr;
  }
  g_fade_ready = false;
  g_fade_failed = false;
}

// The black transition layer's sprite: kLevels blocks of premultiplied black, opacity 0 .. 1 left to right.
bool record_fade_locked(ID3D12Device* device) {
  if (g_fade_ready || g_fade_failed || g_session == XR_NULL_HANDLE) {
    return false;
  }
  g_fade_failed = true;
  constexpr uint32_t kWidth = menu_transition::kLevels * menu_transition::kLevelTexels;
  const bool ok = record_sprite_locked(device, "fade", kWidth, menu_transition::kLevelTexels, &g_fade_chain, &g_fade_upload,
                                       [](uint32_t x, uint32_t, unsigned char* t) {
    const uint32_t level = x / menu_transition::kLevelTexels;
    t[0] = t[1] = t[2] = 0;
    t[3] = static_cast<unsigned char>((255u * level + (menu_transition::kLevels - 1) / 2) / (menu_transition::kLevels - 1));
  });
  if (!ok && g_fade_chain != XR_NULL_HANDLE) {
    destroy_fade_locked();
    g_fade_failed = true;
  }
  return ok;
}

void destroy_chains_locked() {
  std::lock_guard<std::recursive_mutex> chains(g_chains);
  destroy_overlay_locked();
  destroy_gui_locked();
  destroy_laser_locked();
  destroy_fade_locked();
  destroy_hud_locked();
  for (EyeChain& eye : g_eyes) {
    if (eye.chain != XR_NULL_HANDLE) {
      xrDestroySwapchain(eye.chain);
    }
    eye = EyeChain{};
  }
  for (DepthChain& d : g_depth_chains) {
    if (d.chain != XR_NULL_HANDLE) {
      xrDestroySwapchain(d.chain);
    }
    d = DepthChain{};
  }
  g_depth_w = g_depth_h = 0;
  g_depth_valid[0] = g_depth_valid[1] = false;
  g_chain_width = 0;
  g_chain_height = 0;
  g_chain_format = 0;
}

void wait_copies_idle() {
  if (g_copy_fence != nullptr && g_copy_event != nullptr && g_copy_fence->GetCompletedValue() < g_copy_fence_next) {
    g_copy_fence->SetEventOnCompletion(g_copy_fence_next, g_copy_event);
    WaitForSingleObject(g_copy_event, 2000);
  }
}

void release_copy_objects() {
  wait_copies_idle();
  if (g_transport_readback != nullptr) {
    if (g_transport_values != nullptr) g_transport_readback->Unmap(0, nullptr);
    g_transport_readback->Release(); g_transport_readback = nullptr;
  }
  if (g_transport_queries != nullptr) { g_transport_queries->Release(); g_transport_queries = nullptr; }
  g_transport_values = nullptr; g_transport_frequency = 0; g_transport_tried = false;
  for (auto& pending : g_transport_pending) pending = false;
  for (auto*& allocator : g_allocators) {
    if (allocator != nullptr) {
      allocator->Release();
      allocator = nullptr;
    }
  }
  if (g_copy_list != nullptr) {
    g_copy_list->Release();
    g_copy_list = nullptr;
  }
  if (g_copy_fence != nullptr) {
    g_copy_fence->Release();
    g_copy_fence = nullptr;
  }
  if (g_copy_event != nullptr) {
    CloseHandle(g_copy_event);
    g_copy_event = nullptr;
  }
  if (g_stage != nullptr) {
    g_stage->Release();
    g_stage = nullptr;
  }
  g_stage_valid = false;
  for (auto*& hold : g_holds) {
    if (hold != nullptr) {
      hold->Release();
      hold = nullptr;
    }
  }
  g_hold_valid = false;
  if (g_copy_device != nullptr) {
    g_copy_device->Release();
    g_copy_device = nullptr;
  }
  for (UINT64& value : g_allocator_fence_value) {
    value = 0;
  }
  g_copy_fence_next = 0;
}

bool ensure_copy_objects(ID3D12Device* device) {
  if (g_copy_list != nullptr) {
    return true;
  }
  g_copy_device = device;
  g_copy_device->AddRef();
  for (auto*& allocator : g_allocators) {
    if (FAILED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)))) {
      log_line("cinema: CreateCommandAllocator failed");
      return false;
    }
  }
  if (FAILED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_allocators[0], nullptr, IID_PPV_ARGS(&g_copy_list))) ||
      FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_copy_fence)))) {
    log_line("cinema: command list / fence creation failed");
    return false;
  }
  g_copy_list->Close();
  g_copy_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  return g_copy_event != nullptr;
}

// ---- private present queue ("ownqueue", 2026-11-09) ---------------------------------------------------------------
// LIVE gpuprof timelines 2026-11-09 (43 sampled frames): the OpenXR runtime's swapchain synchronisation (fence "?25",
// two waits per frame, one per eye swapchain) stalled the GAME's direct queue > 2 ms in 37% of frames (max 9.4 ms),
// because the session was bound to the game's own queue. With "ownqueue" in scripts\AvatarVR_queue.txt (read when the
// session is created) the session gets a private direct queue instead. The game queue then only copies the eye images
// (and the menu panel / wrist HUD textures) into mod-owned transport textures (three sets) and signals g_copy_fence; the
// private queue waits for that value and writes every swapchain image (eyes incl. sharpening, panel opacity pass, HUD,
// laser / fade sprites). The runtime's waits land on the private queue. A private queue three Presents behind drops that
// Present's copy (the headset repeats the previous frame) instead of blocking the game. The mod menu (vr_menu, recorded
// on the game queue) bridges both queues while it is open: overlay_acquire / overlay_release.
ID3D12CommandQueue* g_pq = nullptr;  // guarded by g_xr; created with the session
bool g_pq_on = false;                // guarded by g_xr
ID3D12CommandAllocator* g_pq_alloc[3] = {};
ID3D12GraphicsCommandList* g_pq_list = nullptr;
ID3D12Fence* g_pq_fence = nullptr;
HANDLE g_pq_event = nullptr;
UINT64 g_pq_fence_next = 0;
UINT64 g_pq_slot_value[3] = {};
struct PqTexture {
  ID3D12Resource* tex = nullptr;
  UINT64 w = 0;
  UINT h = 0;
  DXGI_FORMAT f = DXGI_FORMAT_UNKNOWN;
};
PqTexture g_pq_eye[3][2], g_pq_gui[3], g_pq_hud[3];
std::atomic<std::uint64_t> g_pq_frames{0}, g_pq_drops{0}, g_pq_bridges{0};

bool own_queue_wanted() {
  char text[64] = {};
  const std::wstring dir = self_dir();
  return !dir.empty() && settings_file::read(dir + L"\\AvatarVR_queue.txt", text, sizeof(text)) &&
         std::strstr(text, "ownqueue") != nullptr;
}

void wait_pq_idle() {
  if (g_pq_fence != nullptr && g_pq_event != nullptr && g_pq_fence->GetCompletedValue() < g_pq_fence_next) {
    g_pq_fence->SetEventOnCompletion(g_pq_fence_next, g_pq_event);
    WaitForSingleObject(g_pq_event, 2000);
  }
}

bool ensure_pq_objects(ID3D12Device* device) {
  if (g_pq_list != nullptr) {
    return true;
  }
  if (g_pq == nullptr) {
    return false;
  }
  for (auto*& allocator : g_pq_alloc) {
    if (FAILED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&allocator)))) {
      log_line("ownqueue: CreateCommandAllocator failed");
      return false;
    }
  }
  if (FAILED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_pq_alloc[0], nullptr, IID_PPV_ARGS(&g_pq_list))) ||
      FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_pq_fence)))) {
    log_line("ownqueue: command list / fence creation failed");
    if (g_pq_list != nullptr) {
      g_pq_list->Release();
      g_pq_list = nullptr;
    }
    return false;
  }
  g_pq_list->Close();
  g_pq_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  log_line("ownqueue: private present queue %p ready (copies, list, fence)", static_cast<void*>(g_pq));
  return g_pq_event != nullptr;
}

// Transport texture: simultaneous-access, so it is written on the game queue and read on the private queue from COMMON
// without barriers. Recreated (after both queues are idle) when the size or format changes.
bool pq_texture(ID3D12Device* device, PqTexture& t, UINT64 w, UINT h, DXGI_FORMAT f) {
  if (t.tex != nullptr && t.w == w && t.h == h && t.f == f) {
    return true;
  }
  if (t.tex != nullptr) {
    wait_copies_idle();
    wait_pq_idle();
    t.tex->Release();
    t.tex = nullptr;
  }
  D3D12_HEAP_PROPERTIES heap = {D3D12_HEAP_TYPE_DEFAULT};
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  desc.Width = w;
  desc.Height = h;
  desc.DepthOrArraySize = desc.MipLevels = 1;
  desc.Format = f;
  desc.SampleDesc.Count = 1;
  desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
                                             IID_PPV_ARGS(&t.tex)))) {
    t.tex = nullptr;
    static ULONGLONG warned = 0;
    if (GetTickCount64() - warned > 5000) {
      warned = GetTickCount64();
      log_line("ownqueue: transport texture %llux%u format %d failed", static_cast<unsigned long long>(w), h, static_cast<int>(f));
    }
    return false;
  }
  t.w = w;
  t.h = h;
  t.f = f;
  return true;
}

void release_pq_locked() {
  wait_pq_idle();
  for (auto& slot : g_pq_eye) {
    for (PqTexture& t : slot) {
      if (t.tex != nullptr) t.tex->Release();
      t = PqTexture{};
    }
  }
  for (PqTexture* list : {g_pq_gui, g_pq_hud}) {
    for (int i = 0; i < 3; ++i) {
      if (list[i].tex != nullptr) list[i].tex->Release();
      list[i] = PqTexture{};
    }
  }
  for (auto*& allocator : g_pq_alloc) {
    if (allocator != nullptr) {
      allocator->Release();
      allocator = nullptr;
    }
  }
  if (g_pq_list != nullptr) {
    g_pq_list->Release();
    g_pq_list = nullptr;
  }
  if (g_pq_fence != nullptr) {
    g_pq_fence->Release();
    g_pq_fence = nullptr;
  }
  if (g_pq_event != nullptr) {
    CloseHandle(g_pq_event);
    g_pq_event = nullptr;
  }
  g_pq_on = false;  // g_pq itself is released by run_session after xrDestroySession
  g_pq_fence_next = 0;
  for (UINT64& v : g_pq_slot_value) v = 0;
}

// GPU duration of our copies and mirror, in their existing command list.
// No extra submissions and no readback wait: read only after the allocator fence.
bool transport_begin(ID3D12Device* device, ID3D12CommandQueue* queue, int slot) {
  if (!g_transport_tried) {
    g_transport_tried = true;
    D3D12_QUERY_HEAP_DESC q = {};
    q.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP; q.Count = 12;
    D3D12_HEAP_PROPERTIES hp = {}; hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd = {};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; rd.Width = 12*sizeof(UINT64);
    rd.Height = 1; rd.DepthOrArraySize = 1; rd.MipLevels = 1; rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    const D3D12_RANGE range = {0, 12*sizeof(UINT64)};
    void* data = nullptr;
    if (FAILED(queue->GetTimestampFrequency(&g_transport_frequency)) || g_transport_frequency == 0 ||
        FAILED(device->CreateQueryHeap(&q, IID_PPV_ARGS(&g_transport_queries))) ||
        FAILED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST,
                                               nullptr, IID_PPV_ARGS(&g_transport_readback))) ||
        FAILED(g_transport_readback->Map(0, &range, &data))) {
      log_line("transport GPU timing unavailable; copies still enabled");
      g_transport_frequency = 0;
    } else g_transport_values = static_cast<UINT64*>(data);
  }
  if (g_transport_values == nullptr || g_transport_frequency == 0) return false;
  if (g_transport_pending[slot]) {
    const UINT64* t = g_transport_values + slot*4;
    if (t[1] >= t[0] && t[3] >= t[2] && t[0] != 0) {
      g_transport_eye_ms += static_cast<double>(t[1]-t[0])*1000.0/g_transport_frequency;
      g_transport_mirror_ms += static_cast<double>(t[3]-t[2])*1000.0/g_transport_frequency;
      ++g_transport_samples;
    }
    g_transport_pending[slot] = false;
  }
  static ULONGLONG reported = 0;
  const ULONGLONG now = GetTickCount64();
  if (now-reported >= 5000 && g_transport_samples > 0) {
    log_line("transport GPU: eye pair copies %.3f ms, mirror %.3f ms (%u completed samples; not CPU timing)",
             g_transport_eye_ms/g_transport_samples, g_transport_mirror_ms/g_transport_samples, g_transport_samples);
    g_transport_eye_ms = g_transport_mirror_ms = 0.0; g_transport_samples = 0; reported = now;
  }
  return true;
}

bool ensure_chains_locked(uint32_t width, uint32_t height, DXGI_FORMAT backbuffer_format) {
  if (g_chain_width == width && g_chain_height == height && g_eyes[0].chain != XR_NULL_HANDLE && g_eyes[1].chain != XR_NULL_HANDLE) {
    return true;
  }
  if (g_chain_failed) {
    return false;
  }
  // Our mirror may sample an acquired eye image. Finish our previous copy/draw
  // before destroying that image's chain during a resolution change.
  wait_copies_idle();
  if (g_copy_fence != nullptr && g_copy_fence->GetCompletedValue() < g_copy_fence_next) {
    return false;  // GPU not idle: keep the old images alive and retry later
  }
  destroy_chains_locked();
  uint32_t count = 0;
  xrEnumerateSwapchainFormats(g_session, 0, &count, nullptr);
  std::vector<int64_t> formats(count);
  if (count == 0 || XR_FAILED(xrEnumerateSwapchainFormats(g_session, count, &count, formats.data()))) {
    log_line("cinema: no swapchain formats");
    g_chain_failed = true;
    return false;
  }
  // The game's backbuffer holds sRGB-encoded bytes in a UNORM format. Prefer the _SRGB twin (same typeless
  // family, so the copy is legal) so the runtime decodes it correctly; fall back to the identical format.
  int64_t srgb_twin = 0;
  if (backbuffer_format == DXGI_FORMAT_R8G8B8A8_UNORM) {
    srgb_twin = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
  } else if (backbuffer_format == DXGI_FORMAT_B8G8R8A8_UNORM) {
    srgb_twin = DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  }
  int64_t chosen = 0;
  for (int64_t f : formats) {
    if (srgb_twin != 0 && f == srgb_twin) {
      chosen = f;
    }
  }
  if (chosen == 0) {
    for (int64_t f : formats) {
      if (f == static_cast<int64_t>(backbuffer_format)) {
        chosen = f;
      }
    }
  }
  if (chosen == 0) {
    log_line("cinema: runtime offers no format compatible with backbuffer format %d (first offered %lld)",
             static_cast<int>(backbuffer_format), static_cast<long long>(formats[0]));
    g_chain_failed = true;
    return false;
  }
  for (int e = 0; e < 2; ++e) {
    XrSwapchainCreateInfo info = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
                      XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_MUTABLE_FORMAT_BIT;
    info.format = chosen;
    info.sampleCount = 1;
    info.width = width;
    info.height = height;
    info.faceCount = 1;
    info.arraySize = 1;
    info.mipCount = 1;
    XrResult r = xrCreateSwapchain(g_session, &info, &g_eyes[e].chain);
    g_eyes[e].sampleable = XR_SUCCEEDED(r);
    if (XR_FAILED(r)) {
      // Optional monitor optimization must never prevent headset operation.
      log_line("mirror: sampleable eye %d rejected (%s); retrying original usage", e, result_text(g_instance, r).c_str());
      info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
      r = xrCreateSwapchain(g_session, &info, &g_eyes[e].chain);
    }
    if (XR_FAILED(r)) {
      log_line("cinema: xrCreateSwapchain failed: %s", result_text(g_instance, r).c_str());
      g_eyes[e].chain = XR_NULL_HANDLE;
      destroy_chains_locked();
      g_chain_failed = true;
      return false;
    }
    uint32_t images = 0;
    xrEnumerateSwapchainImages(g_eyes[e].chain, 0, &images, nullptr);
    std::vector<XrSwapchainImageD3D12KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
    r = xrEnumerateSwapchainImages(g_eyes[e].chain, images, &images, reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
    if (XR_FAILED(r) || images == 0) {
      log_line("cinema: xrEnumerateSwapchainImages failed: %s", result_text(g_instance, r).c_str());
      destroy_chains_locked();
      g_chain_failed = true;
      return false;
    }
    for (const auto& image : list) {
      g_eyes[e].images.push_back(image.texture);
    }
  }
  g_chain_width = width;
  g_chain_height = height;
  g_chain_format = chosen;
  log_line("cinema: swapchains %ux%u format=%lld (backbuffer format %d) images=%zu per eye", width, height,
           static_cast<long long>(chosen), static_cast<int>(backbuffer_format), g_eyes[0].images.size());
  return true;
}

// Depth swapchains at the colour chain size (D32_FLOAT, only if the runtime lists it). A failure is final for the
// session: depth is optional and must never affect the colour path.
bool ensure_depth_chains_locked(uint32_t width, uint32_t height) {
  if (g_depth_w == width && g_depth_h == height && g_depth_chains[0].chain != XR_NULL_HANDLE &&
      g_depth_chains[1].chain != XR_NULL_HANDLE) {
    return true;
  }
  if (g_depth_failed || !g_depth_ext || g_session == XR_NULL_HANDLE) {
    return false;
  }
  for (DepthChain& d : g_depth_chains) {
    if (d.chain != XR_NULL_HANDLE) xrDestroySwapchain(d.chain);
    d = DepthChain{};
  }
  uint32_t count = 0;
  xrEnumerateSwapchainFormats(g_session, 0, &count, nullptr);
  std::vector<int64_t> formats(count);
  bool offered = false;
  if (count > 0 && XR_SUCCEEDED(xrEnumerateSwapchainFormats(g_session, count, &count, formats.data()))) {
    for (int64_t f : formats) offered = offered || f == static_cast<int64_t>(DXGI_FORMAT_D32_FLOAT);
  }
  if (!offered) {
    log_line("xrdepth: runtime offers no D32_FLOAT swapchain format; depth stays off");
    g_depth_failed = true;
    return false;
  }
  for (int e = 0; e < 2; ++e) {
    XrSwapchainCreateInfo info = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    info.format = DXGI_FORMAT_D32_FLOAT;
    info.sampleCount = 1;
    info.width = width;
    info.height = height;
    info.faceCount = 1;
    info.arraySize = 1;
    info.mipCount = 1;
    XrResult r = xrCreateSwapchain(g_session, &info, &g_depth_chains[e].chain);
    uint32_t images = 0;
    if (XR_SUCCEEDED(r)) r = xrEnumerateSwapchainImages(g_depth_chains[e].chain, 0, &images, nullptr);
    std::vector<XrSwapchainImageD3D12KHR> list(images, {XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR});
    if (XR_SUCCEEDED(r) && images > 0) {
      r = xrEnumerateSwapchainImages(g_depth_chains[e].chain, images, &images,
                                     reinterpret_cast<XrSwapchainImageBaseHeader*>(list.data()));
    }
    if (XR_FAILED(r) || images == 0) {
      log_line("xrdepth: depth swapchain eye %d failed: %s; depth stays off", e, result_text(g_instance, r).c_str());
      for (DepthChain& d : g_depth_chains) {
        if (d.chain != XR_NULL_HANDLE) xrDestroySwapchain(d.chain);
        d = DepthChain{};
      }
      g_depth_failed = true;
      return false;
    }
    for (const auto& image : list) g_depth_chains[e].images.push_back(image.texture);
  }
  g_depth_w = width;
  g_depth_h = height;
  log_line("xrdepth: depth swapchains %ux%u D32_FLOAT images=%zu per eye", width, height, g_depth_chains[0].images.size());
  return true;
}

bool ensure_stage(ID3D12Device* device, const D3D12_RESOURCE_DESC& desc) {
  if (g_stage != nullptr) {
    const D3D12_RESOURCE_DESC have = g_stage->GetDesc();
    if (have.Width == desc.Width && have.Height == desc.Height && have.Format == desc.Format) {
      return true;
    }
    g_stage->Release();
    g_stage = nullptr;
    g_stage_valid = false;
  }
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC stage_desc = desc;
  stage_desc.Flags = D3D12_RESOURCE_FLAG_NONE;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &stage_desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&g_stage)))) {
    log_line("cinema: stage texture creation failed");
    g_stage = nullptr;
    return false;
  }
  log_line("cinema: left-eye stage %llux%u", static_cast<unsigned long long>(desc.Width), desc.Height);
  return true;
}

void transition(ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after);

// Hold textures for same-frame pairing: backbuffer-sized, in the second view's format (same typeless family as the
// eye swapchains). Recreated (after the GPU is idle on our copies) when the size or format changes.
bool ensure_holds(ID3D12Device* device, const D3D12_RESOURCE_DESC& back_desc, DXGI_FORMAT format) {
  if (g_holds[0] != nullptr && g_holds[1] != nullptr) {
    const D3D12_RESOURCE_DESC have = g_holds[0]->GetDesc();
    if (have.Width == back_desc.Width && have.Height == back_desc.Height && have.Format == format) {
      return true;
    }
  }
  wait_copies_idle();
  for (auto*& hold : g_holds) {
    if (hold != nullptr) {
      hold->Release();
      hold = nullptr;
    }
  }
  g_hold_valid = false;
  g_hold_read = 0;
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC d = back_desc;
  d.Format = format;
  d.Flags = D3D12_RESOURCE_FLAG_NONE;
  d.MipLevels = 1;
  d.DepthOrArraySize = 1;
  d.SampleDesc.Count = 1;
  d.SampleDesc.Quality = 0;
  d.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
  for (int i = 0; i < 2; ++i) {
    if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                               IID_PPV_ARGS(&g_holds[i])))) {
      g_holds[i] = nullptr;
      log_line("pair: hold texture creation failed (%llux%u format %d); pairing off", static_cast<unsigned long long>(d.Width),
               d.Height, static_cast<int>(format));
      g_pair = false;
      return false;
    }
    g_hold_states[i] = D3D12_RESOURCE_STATE_COPY_DEST;
  }
  log_line("pair: hold textures %llux%u format %d", static_cast<unsigned long long>(d.Width), d.Height, static_cast<int>(format));
  return true;
}

void hold_state(int i, D3D12_RESOURCE_STATES want) {
  if (g_hold_states[i] != want) {
    transition(g_holds[i], g_hold_states[i], want);
    g_hold_states[i] = want;
  }
}

void transition(ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = resource;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = before;
  b.Transition.StateAfter = after;
  g_copy_list->ResourceBarrier(1, &b);
}

// ---- eye mode: selected submitted eye -> monitor mirror -------------------------------------------------------
// With "eyesize W H both" both views render offscreen at the eye size and nothing draws the backbuffer. The left
// eye is copied into our own texture (state-managed) and drawn over the whole backbuffer with a fullscreen triangle,
// cropped to the window's aspect (fill, centred) and with no blending, so a widescreen window shows the middle of the
// submitted eye image. The monitor copy takes one draw per Present.
ID3D12RootSignature* g_mir_rs = nullptr;
ID3D12PipelineState* g_mir_pso = nullptr;
ID3D12PipelineState* g_mir_pso_blend = nullptr;  // main menu panel over the mirror (premultiplied alpha)
constexpr UINT kMirSrvSlots = 11;                  // [0..2] eye SRVs (allocator fence ring), [3..10] panel ring (a slot may still be in flight)
UINT g_mir_srv_stride = 0;
UINT g_mir_panel_slot = 0;
DXGI_FORMAT g_mir_rtv_fmt = DXGI_FORMAT_UNKNOWN;
ID3D12DescriptorHeap* g_mir_srv_heap = nullptr;
ID3D12DescriptorHeap* g_mir_rtv_heap = nullptr;
ID3D12Resource* g_mir_tex = nullptr;
UINT g_mir_w = 0, g_mir_h = 0;
DXGI_FORMAT g_mir_fmt = DXGI_FORMAT_UNKNOWN;
ID3D12Resource* g_mir_retired[4] = {};
int g_mir_retired_next = 0;
bool g_mir_failed = false;
std::atomic<unsigned> g_mirror_refresh{30};
std::atomic<std::uint64_t> g_mirror_copies{0}, g_mirror_direct{0};
std::atomic<std::uint64_t> g_mirror_draws{0}, g_eye_mode_presents{0}, g_eye_mode_mismatch{0};

DXGI_FORMAT view_format(DXGI_FORMAT f) {
  switch (f) {
    case DXGI_FORMAT_R8G8B8A8_TYPELESS:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
      return DXGI_FORMAT_R8G8B8A8_UNORM;  // the bytes are already sRGB-encoded: pass them through unchanged
    case DXGI_FORMAT_B8G8R8A8_TYPELESS:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
      return DXGI_FORMAT_B8G8R8A8_UNORM;
    case DXGI_FORMAT_R10G10B10A2_TYPELESS:
      return DXGI_FORMAT_R10G10B10A2_UNORM;
    default:
      return f;
  }
}

bool ensure_mirror_pipeline(ID3D12Device* device, DXGI_FORMAT rtv_fmt) {
  if (g_mir_pso != nullptr && g_mir_rtv_fmt == rtv_fmt) {
    return true;
  }
  if (g_mir_failed) {
    return false;
  }
  if (g_mir_pso != nullptr) {
    g_mir_pso->Release();
    g_mir_pso = nullptr;
  }
  static const char kShader[] =
      "cbuffer C : register(b0) { float4 uvr; };\n"
      "Texture2D T : register(t0); SamplerState S : register(s0);\n"
      "struct V { float4 p : SV_Position; float2 uv : TEXCOORD0; };\n"
      "V vs(uint id : SV_VertexID) { V o; float2 q = float2((id << 1) & 2, id & 2);\n"
      "  o.p = float4(q * float2(2, -2) + float2(-1, 1), 0, 1); o.uv = uvr.xy + (uvr.zw - uvr.xy) * q; return o; }\n"
      "float4 ps(V i) : SV_Target { return float4(T.Sample(S, i.uv).rgb, 1); }\n"
      "float4 psa(V i) : SV_Target { return T.Sample(S, i.uv); }\n";
  ID3DBlob* psa = nullptr;
  ID3DBlob* vs = nullptr;
  ID3DBlob* ps = nullptr;
  ID3DBlob* err = nullptr;
  bool ok = SUCCEEDED(D3DCompile(kShader, sizeof(kShader) - 1, "mirror", nullptr, nullptr, "vs", "vs_5_0", 0, 0, &vs, &err)) &&
            SUCCEEDED(D3DCompile(kShader, sizeof(kShader) - 1, "mirror", nullptr, nullptr, "ps", "ps_5_0", 0, 0, &ps, &err)) &&
            SUCCEEDED(D3DCompile(kShader, sizeof(kShader) - 1, "mirror", nullptr, nullptr, "psa", "ps_5_0", 0, 0, &psa, &err));
  if (!ok && err != nullptr) {
    log_line("mirror: shader compile failed: %s", static_cast<const char*>(err->GetBufferPointer()));
  }
  if (ok && g_mir_rs == nullptr) {
    D3D12_DESCRIPTOR_RANGE range = {D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0, 0};
    D3D12_ROOT_PARAMETER params[2] = {};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.Num32BitValues = 4;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[1].DescriptorTable.NumDescriptorRanges = 1;
    params[1].DescriptorTable.pDescriptorRanges = &range;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    D3D12_STATIC_SAMPLER_DESC sampler = {};
    sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    sampler.AddressU = sampler.AddressV = sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.MaxLOD = D3D12_FLOAT32_MAX;
    sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    D3D12_ROOT_SIGNATURE_DESC rsd = {2, params, 1, &sampler, D3D12_ROOT_SIGNATURE_FLAG_NONE};
    ID3DBlob* blob = nullptr;
    ok = SUCCEEDED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &blob, nullptr)) && blob != nullptr &&
         SUCCEEDED(device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g_mir_rs)));
    if (blob != nullptr) {
      blob->Release();
    }
  }
  if (ok) {
    D3D12_GRAPHICS_PIPELINE_STATE_DESC pd = {};
    pd.pRootSignature = g_mir_rs;
    pd.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
    pd.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
    pd.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    pd.SampleMask = UINT_MAX;
    pd.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    pd.RasterizerState.DepthClipEnable = TRUE;
    pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pd.NumRenderTargets = 1;
    pd.RTVFormats[0] = rtv_fmt;
    pd.SampleDesc.Count = 1;
    ok = SUCCEEDED(device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&g_mir_pso)));
    if (ok) {
      if (g_mir_pso_blend != nullptr) {
        g_mir_pso_blend->Release();
        g_mir_pso_blend = nullptr;
      }
      pd.PS = {psa->GetBufferPointer(), psa->GetBufferSize()};
      D3D12_RENDER_TARGET_BLEND_DESC& b = pd.BlendState.RenderTarget[0];
      b.BlendEnable = TRUE;
      b.SrcBlend = D3D12_BLEND_ONE;
      b.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
      b.BlendOp = D3D12_BLEND_OP_ADD;
      b.SrcBlendAlpha = D3D12_BLEND_ONE;
      b.DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA;
      b.BlendOpAlpha = D3D12_BLEND_OP_ADD;
      ok = SUCCEEDED(device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&g_mir_pso_blend)));
    }
  }
  if (ok && g_mir_srv_heap == nullptr) {
    D3D12_DESCRIPTOR_HEAP_DESC hd = {D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, kMirSrvSlots, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE, 0};
    D3D12_DESCRIPTOR_HEAP_DESC rd = {D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 1, D3D12_DESCRIPTOR_HEAP_FLAG_NONE, 0};
    ok = SUCCEEDED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_mir_srv_heap))) &&
         SUCCEEDED(device->CreateDescriptorHeap(&rd, IID_PPV_ARGS(&g_mir_rtv_heap)));
  }
  if (ok) {
    g_mir_srv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
  }
  for (ID3DBlob* b : {vs, ps, psa, err}) {
    if (b != nullptr) {
      b->Release();
    }
  }
  if (!ok) {
    g_mir_failed = true;
    log_line("mirror: pipeline creation failed; the monitor shows the last backbuffer");
    return false;
  }
  g_mir_rtv_fmt = rtv_fmt;
  log_line("mirror: pipeline ready (target format %d)", static_cast<int>(rtv_fmt));
  return true;
}

bool ensure_mirror_tex(ID3D12Device* device, UINT w, UINT h, DXGI_FORMAT fmt) {
  if (g_mir_tex != nullptr && g_mir_w == w && g_mir_h == h && g_mir_fmt == fmt) {
    return true;
  }
  if (g_mir_tex != nullptr) {  // an earlier list may still read it on the GPU: released four replacements later
    if (g_mir_retired[g_mir_retired_next] != nullptr) {
      g_mir_retired[g_mir_retired_next]->Release();
    }
    g_mir_retired[g_mir_retired_next] = g_mir_tex;
    g_mir_retired_next = (g_mir_retired_next + 1) % 4;
    g_mir_tex = nullptr;
  }
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  desc.Width = w;
  desc.Height = h;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.Format = fmt;
  desc.SampleDesc.Count = 1;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                                             nullptr, IID_PPV_ARGS(&g_mir_tex)))) {
    g_mir_tex = nullptr;
    return false;
  }
  g_mir_w = w;
  g_mir_h = h;
  g_mir_fmt = fmt;
  D3D12_SHADER_RESOURCE_VIEW_DESC sd = {};
  sd.Format = view_format(fmt);
  sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
  sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
  sd.Texture2D.MipLevels = 1;
  // SRV written into the current allocator slot in mirror_draw, after its fence.
  (void)sd;
  log_line("mirror: eye texture %ux%u format %d", w, h, static_cast<int>(fmt));
  return true;
}

// ---- VR sharpness (2026-10-17): AMD FidelityFX RCAS on each eye image -----------------------------------------------
// [LIVE] The game's DLSS is 310.9 (DLSS 4); DLSS 2.5.1+ ignores the old in-game sharpening parameter, and Avatar has no
// sharpness option. View -> Sharpness (0-100 %, AvatarVR_monitor.txt "sharpen N") runs FSR 1's robust contrast-adaptive
// sharpening (RCAS: 5-tap cross, limiter against halos) when each eye is copied into its XR image, identically for both
// eyes. 0 % keeps the plain copy (no extra work, identical pixels). Runs on our own copy list after the game finished
// the frame: no game command-list state, descriptor heaps or Streamline internals are touched. The in-world HUD is part
// of the eye image and is sharpened too; the main-menu panel is a separate layer and is not.
// RCAS math from AMD FidelityFX FSR 1 (ffx_fsr1.h, Copyright (c) 2021 Advanced Micro Devices, Inc., MIT licence).
// It operates on the stored (sRGB-encoded, perceptual) values, as FSR intends. A typed _SRGB XR image re-encodes on
// write, so the shader decodes its result first in that case and the stored bytes match the UNORM path.
std::atomic<float> g_sharpness{0.f};  // 0..1 (RCAS "con": 1 = FSR's maximum, 0 stops)
ID3D12RootSignature* g_sharp_rs = nullptr;
ID3D12PipelineState* g_sharp_pso[2] = {};  // per RTV format (cached by format)
DXGI_FORMAT g_sharp_pso_fmt[2] = {DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_UNKNOWN};
ID3D12DescriptorHeap* g_sharp_srv_heap = nullptr;  // [allocator slot * 2 + eye]: read at execution, fence-ringed
ID3D12DescriptorHeap* g_sharp_rtv_heap = nullptr;  // 2 RTVs (read when recorded)
UINT g_sharp_srv_stride = 0, g_sharp_rtv_stride = 0;
ID3D12Resource* g_sharp_tex[2] = {};
UINT g_sharp_w[2] = {}, g_sharp_h[2] = {};
DXGI_FORMAT g_sharp_fmt[2] = {};
ID3D12Resource* g_sharp_retired[8] = {};
int g_sharp_retired_next = 0;
bool g_sharp_failed = false;
std::atomic<std::uint64_t> g_sharp_draws{0};
constexpr UINT kSharpSrvSlots = 8;  // 3 allocator slots x 2 eyes (+ spare)

const char kRcasShader[] =
    "cbuffer C : register(b0) { float con; uint W; uint H; uint srgbOut; };\n"
    "Texture2D<float4> T : register(t0);\n"
    "float4 vs(uint id : SV_VertexID) : SV_Position { float2 q = float2((id << 1) & 2, id & 2);\n"
    "  return float4(q * float2(2, -2) + float2(-1, 1), 0, 1); }\n"
    "float4 at(int2 p) { return T.Load(int3(clamp(p, int2(0, 0), int2(W - 1, H - 1)), 0)); }\n"
    "float3 to_linear(float3 c) { return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4); }\n"
    "float4 ps(float4 pos : SV_Position) : SV_Target {\n"
    "  int2 ip = int2(pos.xy);\n"
    "  float3 b = at(ip + int2(0, -1)).rgb, d = at(ip + int2(-1, 0)).rgb, f = at(ip + int2(1, 0)).rgb,\n"
    "         h = at(ip + int2(0, 1)).rgb;\n"
    "  float4 e4 = at(ip); float3 e = e4.rgb;\n"
    "  float3 mn4 = min(min(b, d), min(f, h)), mx4 = max(max(b, d), max(f, h));\n"
    "  float3 hitMin = min(mn4, e) / max(4.0 * mx4, 1e-5);\n"                      // FSR: min(mn4,e) * rcp(4*mx4)
    "  float3 hitMax = (1.0 - max(mx4, e)) / min(4.0 * mn4 - 4.0, -1e-5);\n"       // FSR: (peak - max) * rcp(4*mn4 - 4)
    "  float3 lobeC = max(-hitMin, hitMax);\n"
    "  float lobe = max(-(0.25 - 1.0 / 16.0), min(max(lobeC.r, max(lobeC.g, lobeC.b)), 0.0)) * con;\n"  // FSR_RCAS_LIMIT
    "  float3 c = saturate((lobe * (b + d + f + h) + e) / (4.0 * lobe + 1.0));\n"
    "  return float4(srgbOut != 0 ? to_linear(c) : c, e4.a); }\n";

bool sharp_format_ok(DXGI_FORMAT f) {
  switch (f) {
    case DXGI_FORMAT_R8G8B8A8_TYPELESS: case DXGI_FORMAT_R8G8B8A8_UNORM: case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
    case DXGI_FORMAT_B8G8R8A8_TYPELESS: case DXGI_FORMAT_B8G8R8A8_UNORM: case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
    case DXGI_FORMAT_R10G10B10A2_TYPELESS: case DXGI_FORMAT_R10G10B10A2_UNORM:
      return true;
    default:
      return false;
  }
}

void sharp_fail(const char* what) {
  if (!g_sharp_failed) log_line("sharpen: %s; plain eye copies for this session", what);
  g_sharp_failed = true;
}

ID3D12PipelineState* sharp_pipeline(ID3D12Device* device, DXGI_FORMAT rtv_fmt) {
  for (int i = 0; i < 2; ++i) {
    if (g_sharp_pso[i] != nullptr && g_sharp_pso_fmt[i] == rtv_fmt) return g_sharp_pso[i];
  }
  ID3DBlob* vs = nullptr;
  ID3DBlob* ps = nullptr;
  ID3DBlob* err = nullptr;
  bool ok = SUCCEEDED(D3DCompile(kRcasShader, sizeof(kRcasShader) - 1, "rcas", nullptr, nullptr, "vs", "vs_5_0", 0, 0, &vs, &err)) &&
            SUCCEEDED(D3DCompile(kRcasShader, sizeof(kRcasShader) - 1, "rcas", nullptr, nullptr, "ps", "ps_5_0", 0, 0, &ps, &err));
  if (!ok && err != nullptr) log_line("sharpen: shader compile failed: %s", static_cast<const char*>(err->GetBufferPointer()));
  if (ok && g_sharp_rs == nullptr) {
    D3D12_DESCRIPTOR_RANGE range = {D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0, 0};
    D3D12_ROOT_PARAMETER params[2] = {};
    params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
    params[0].Constants.Num32BitValues = 4;
    params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    params[1].DescriptorTable.NumDescriptorRanges = 1;
    params[1].DescriptorTable.pDescriptorRanges = &range;
    params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    D3D12_ROOT_SIGNATURE_DESC rsd = {2, params, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_NONE};
    ID3DBlob* blob = nullptr;
    ok = SUCCEEDED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &blob, nullptr)) && blob != nullptr &&
         SUCCEEDED(device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g_sharp_rs)));
    if (blob != nullptr) blob->Release();
  }
  if (ok && g_sharp_srv_heap == nullptr) {
    D3D12_DESCRIPTOR_HEAP_DESC hd = {D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, kSharpSrvSlots, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE, 0};
    D3D12_DESCRIPTOR_HEAP_DESC rd = {D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 2, D3D12_DESCRIPTOR_HEAP_FLAG_NONE, 0};
    ok = SUCCEEDED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_sharp_srv_heap))) &&
         SUCCEEDED(device->CreateDescriptorHeap(&rd, IID_PPV_ARGS(&g_sharp_rtv_heap)));
    g_sharp_srv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    g_sharp_rtv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
  }
  ID3D12PipelineState* pso = nullptr;
  if (ok) {
    D3D12_GRAPHICS_PIPELINE_STATE_DESC pd = {};
    pd.pRootSignature = g_sharp_rs;
    pd.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
    pd.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
    pd.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    pd.SampleMask = UINT_MAX;
    pd.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    pd.RasterizerState.DepthClipEnable = TRUE;
    pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pd.NumRenderTargets = 1;
    pd.RTVFormats[0] = rtv_fmt;
    pd.SampleDesc.Count = 1;
    ok = SUCCEEDED(device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&pso)));
  }
  for (ID3DBlob* b : {vs, ps, err}) {
    if (b != nullptr) b->Release();
  }
  if (!ok || pso == nullptr) {
    sharp_fail("pipeline creation failed");
    return nullptr;
  }
  const int i = g_sharp_pso[0] == nullptr ? 0 : 1;  // at most two RTV formats (UNORM / _SRGB views)
  if (g_sharp_pso[i] != nullptr) g_sharp_pso[i]->Release();  // only if a third format appeared; not in flight (new chain)
  g_sharp_pso[i] = pso;
  g_sharp_pso_fmt[i] = rtv_fmt;
  log_line("sharpen: RCAS pipeline ready (target format %d)", static_cast<int>(rtv_fmt));
  return pso;
}

bool sharp_texture(ID3D12Device* device, int eye, UINT w, UINT h, DXGI_FORMAT fmt) {
  if (g_sharp_tex[eye] != nullptr && g_sharp_w[eye] == w && g_sharp_h[eye] == h && g_sharp_fmt[eye] == fmt) return true;
  if (g_sharp_tex[eye] != nullptr) {  // an earlier list may still read it: released eight replacements later
    if (g_sharp_retired[g_sharp_retired_next] != nullptr) g_sharp_retired[g_sharp_retired_next]->Release();
    g_sharp_retired[g_sharp_retired_next] = g_sharp_tex[eye];
    g_sharp_retired_next = (g_sharp_retired_next + 1) % 8;
    g_sharp_tex[eye] = nullptr;
  }
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_DEFAULT;
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
  desc.Width = w;
  desc.Height = h;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.Format = fmt;  // the source's own format: a plain copy, then an encoded-value (UNORM) view
  desc.SampleDesc.Count = 1;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr,
                                             IID_PPV_ARGS(&g_sharp_tex[eye])))) {
    g_sharp_tex[eye] = nullptr;
    sharp_fail("eye texture creation failed");
    return false;
  }
  g_sharp_w[eye] = w;
  g_sharp_h[eye] = h;
  g_sharp_fmt[eye] = fmt;
  log_line("sharpen: eye %d texture %ux%u format %d", eye, w, h, static_cast<int>(fmt));
  return true;
}

// Records into g_copy_list: src -> our eye texture (copy) -> RCAS draw into `target` (already in RENDER_TARGET).
// False = nothing recorded; the caller does the plain copy.
bool sharpen_eye(ID3D12Device* device, int eye, ID3D12Resource* target, const D3D12_TEXTURE_COPY_LOCATION& src, UINT w, UINT h,
                 int slot, float strength) {
  if (g_sharp_failed || src.Type != D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX || src.pResource == nullptr || slot < 0 ||
      static_cast<UINT>(slot) * 2 + 1 >= kSharpSrvSlots) {
    return false;
  }
  const DXGI_FORMAT sfmt = src.pResource->GetDesc().Format;
  const D3D12_RESOURCE_DESC td = target->GetDesc();
  if (!sharp_format_ok(sfmt) || !sharp_format_ok(td.Format)) {
    static DXGI_FORMAT warned = DXGI_FORMAT_UNKNOWN;
    if (warned != sfmt) {
      warned = sfmt;
      log_line("sharpen: unsupported formats (source %d, eye image %d); plain copy", static_cast<int>(sfmt), static_cast<int>(td.Format));
    }
    return false;
  }
  // Typeless/UNORM storage: UNORM view, stored bytes pass through. Typed _SRGB: _SRGB view, shader decodes first.
  const bool srgb_out = td.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || td.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  const DXGI_FORMAT rtv_fmt = srgb_out ? td.Format : view_format(td.Format);
  ID3D12PipelineState* pso = sharp_pipeline(device, rtv_fmt);
  if (pso == nullptr || !sharp_texture(device, eye, w, h, sfmt)) return false;
  ID3D12Resource* tex = g_sharp_tex[eye];
  transition(tex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_DEST);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = tex;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  D3D12_BOX box = {0, 0, 0, w, h, 1};
  g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
  transition(tex, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
  const UINT srv_index = static_cast<UINT>(slot) * 2 + static_cast<UINT>(eye);
  D3D12_CPU_DESCRIPTOR_HANDLE srv_cpu = g_sharp_srv_heap->GetCPUDescriptorHandleForHeapStart();
  D3D12_GPU_DESCRIPTOR_HANDLE srv_gpu = g_sharp_srv_heap->GetGPUDescriptorHandleForHeapStart();
  srv_cpu.ptr += static_cast<SIZE_T>(srv_index) * g_sharp_srv_stride;
  srv_gpu.ptr += static_cast<UINT64>(srv_index) * g_sharp_srv_stride;
  D3D12_SHADER_RESOURCE_VIEW_DESC sd = {};
  sd.Format = view_format(sfmt);
  sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
  sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
  sd.Texture2D.MipLevels = 1;
  device->CreateShaderResourceView(tex, &sd, srv_cpu);
  D3D12_CPU_DESCRIPTOR_HANDLE rtv = g_sharp_rtv_heap->GetCPUDescriptorHandleForHeapStart();
  rtv.ptr += static_cast<SIZE_T>(eye) * g_sharp_rtv_stride;
  D3D12_RENDER_TARGET_VIEW_DESC rd = {};
  rd.Format = rtv_fmt;
  rd.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
  device->CreateRenderTargetView(target, &rd, rtv);
  const D3D12_VIEWPORT vp = {0.f, 0.f, static_cast<float>(w), static_cast<float>(h), 0.f, 1.f};
  const D3D12_RECT sc = {0, 0, static_cast<LONG>(w), static_cast<LONG>(h)};
  struct { float con; UINT w, h, srgb; } k = {strength, w, h, srgb_out ? 1u : 0u};
  g_copy_list->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
  g_copy_list->RSSetViewports(1, &vp);
  g_copy_list->RSSetScissorRects(1, &sc);
  g_copy_list->SetGraphicsRootSignature(g_sharp_rs);
  g_copy_list->SetPipelineState(pso);
  ID3D12DescriptorHeap* heaps[] = {g_sharp_srv_heap};
  g_copy_list->SetDescriptorHeaps(1, heaps);
  g_copy_list->SetGraphicsRoot32BitConstants(0, 4, &k, 0);
  g_copy_list->SetGraphicsRootDescriptorTable(1, srv_gpu);
  g_copy_list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  g_copy_list->DrawInstanced(3, 1, 0, 0);
  g_sharp_draws.fetch_add(1, std::memory_order_relaxed);
  return true;
}

// Records into g_copy_list. `back` is in COPY_SOURCE on entry and on exit. `source` is either the backbuffer
// (left) or the second view's simultaneous-access texture (right); `xr_eye` is its actual submitted XR image.
void mirror_draw(ID3D12Device* device, ID3D12Resource* source, UINT ew, UINT eh, ID3D12Resource* back,
                 const D3D12_RESOURCE_DESC& bd, ID3D12Resource* xr_eye, int allocator_slot) {
  const D3D12_RESOURCE_DESC source_desc = source->GetDesc();
  const DXGI_FORMAT rtv_fmt = view_format(bd.Format);
  const unsigned rate = g_mirror_refresh.load(std::memory_order_relaxed);
  const D3D12_RESOURCE_DESC xd = xr_eye != nullptr ? xr_eye->GetDesc() : D3D12_RESOURCE_DESC{};
  // Preserve encoded colour exactly. A typed SRGB image cannot be viewed as
  // UNORM in D3D12, so use the old staging path unless its storage is typeless
  // (mutable) or already UNORM. No shader, FOV or headset pixels are altered.
  const bool direct = (rate == 0 || g_mirror_right.load(std::memory_order_relaxed)) && xr_eye != nullptr &&
      !(xd.Flags & D3D12_RESOURCE_FLAG_DENY_SHADER_RESOURCE) &&
      (xd.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS || xd.Format == DXGI_FORMAT_B8G8R8A8_TYPELESS ||
       xd.Format == DXGI_FORMAT_R8G8B8A8_UNORM || xd.Format == DXGI_FORMAT_B8G8R8A8_UNORM);
  if (!ensure_mirror_pipeline(device, rtv_fmt)) return;
  if (!direct && !ensure_mirror_tex(device, ew, eh, source_desc.Format)) return;
  ID3D12Resource* sampled = direct ? xr_eye : g_mir_tex;
  if (direct) {
    transition(xr_eye, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    g_mirror_direct.fetch_add(1, std::memory_order_relaxed);
  } else {
  static ULONGLONG copied_at = 0;
  static UINT copied_w = 0, copied_h = 0;
  const bool resized = copied_w != ew || copied_h != eh;
  const ULONGLONG now = GetTickCount64();
  if (resized || rate == 0 || g_mirror_right.load(std::memory_order_relaxed) || now - copied_at >= (1000u + rate - 1) / rate) {
  transition(g_mir_tex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_DEST);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = g_mir_tex;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = source;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  D3D12_BOX box = {0, 0, 0, ew, eh, 1};
  g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
  transition(g_mir_tex, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
  copied_at = now;
  copied_w = ew; copied_h = eh;
  g_mirror_copies.fetch_add(1, std::memory_order_relaxed);
  }
  }
  D3D12_CPU_DESCRIPTOR_HANDLE eye_cpu = g_mir_srv_heap->GetCPUDescriptorHandleForHeapStart();
  D3D12_GPU_DESCRIPTOR_HANDLE eye_gpu = g_mir_srv_heap->GetGPUDescriptorHandleForHeapStart();
  eye_cpu.ptr += static_cast<SIZE_T>(allocator_slot) * g_mir_srv_stride;
  eye_gpu.ptr += static_cast<UINT64>(allocator_slot) * g_mir_srv_stride;
  D3D12_SHADER_RESOURCE_VIEW_DESC eye_sd = {};
  eye_sd.Format = view_format(sampled->GetDesc().Format);
  eye_sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
  eye_sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
  eye_sd.Texture2D.MipLevels = 1;
  device->CreateShaderResourceView(sampled, &eye_sd, eye_cpu);
  // The game overwrites the backbuffer with an eye each frame, so repaint the
  // cached mirror every time; only the full-eye staging copy is throttled.
  // Fill the window with the centre of the eye image, keeping its proportions.
  unsigned cw = 0, ch = 0;
  if (!dx12::client_size(&cw, &ch)) {
    cw = static_cast<unsigned>(bd.Width);
    ch = bd.Height;
  }
  const float bw = static_cast<float>(cw), bh = static_cast<float>(ch);  // the region the monitor is shown
  const float eye_aspect = static_cast<float>(ew) / static_cast<float>(eh), win_aspect = bw / bh;
  float uvr[4] = {0.f, 0.f, 1.f, 1.f};
  if (win_aspect >= eye_aspect) {
    const float f = eye_aspect / win_aspect;  // visible fraction of the eye height
    uvr[1] = 0.5f - 0.5f * f;
    uvr[3] = 0.5f + 0.5f * f;
  } else {
    const float f = win_aspect / eye_aspect;
    uvr[0] = 0.5f - 0.5f * f;
    uvr[2] = 0.5f + 0.5f * f;
  }
  transition(back, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
  D3D12_RENDER_TARGET_VIEW_DESC rd = {};
  rd.Format = rtv_fmt;
  rd.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
  const D3D12_CPU_DESCRIPTOR_HANDLE rtv = g_mir_rtv_heap->GetCPUDescriptorHandleForHeapStart();
  device->CreateRenderTargetView(back, &rd, rtv);
  const D3D12_VIEWPORT vp = {0.f, 0.f, bw, bh, 0.f, 1.f};
  const D3D12_RECT sc = {0, 0, static_cast<LONG>(cw), static_cast<LONG>(ch)};
  g_copy_list->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
  g_copy_list->RSSetViewports(1, &vp);
  g_copy_list->RSSetScissorRects(1, &sc);
  g_copy_list->SetGraphicsRootSignature(g_mir_rs);
  g_copy_list->SetPipelineState(g_mir_pso);
  ID3D12DescriptorHeap* heaps[] = {g_mir_srv_heap};
  g_copy_list->SetDescriptorHeaps(1, heaps);
  g_copy_list->SetGraphicsRoot32BitConstants(0, 4, uvr, 0);
  g_copy_list->SetGraphicsRootDescriptorTable(1, eye_gpu);
  g_copy_list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  g_copy_list->DrawInstanced(3, 1, 0, 0);
  // Main menu panel (3D main menu): the game's UI goes into camera.cpp's layer texture for the headset quad, so it
  // never reaches the backbuffer; blend it over the mirror too, fitted into the window and centred.
  ID3D12Resource* panel = g_gui_visible.load(std::memory_order_relaxed) ? camera::ui_layer_acquire() : nullptr;
  if (panel != nullptr && g_mir_pso_blend != nullptr) {
    const D3D12_RESOURCE_DESC pdsc = panel->GetDesc();
    const float pa = static_cast<float>(pdsc.Width) / static_cast<float>(pdsc.Height);
    float pw = bw, ph = bw / pa;
    if (ph > bh) {
      ph = bh;
      pw = bh * pa;
    }
    const D3D12_VIEWPORT pvp = {0.5f * (bw - pw), 0.5f * (bh - ph), pw, ph, 0.f, 1.f};
    g_mir_panel_slot = g_mir_panel_slot >= kMirSrvSlots - 1 ? 3 : std::max(3u, g_mir_panel_slot + 1);
    D3D12_CPU_DESCRIPTOR_HANDLE cpu = g_mir_srv_heap->GetCPUDescriptorHandleForHeapStart();
    D3D12_GPU_DESCRIPTOR_HANDLE gpu = g_mir_srv_heap->GetGPUDescriptorHandleForHeapStart();
    cpu.ptr += static_cast<SIZE_T>(g_mir_panel_slot) * g_mir_srv_stride;
    gpu.ptr += static_cast<UINT64>(g_mir_panel_slot) * g_mir_srv_stride;
    D3D12_SHADER_RESOURCE_VIEW_DESC sd = {};
    sd.Format = view_format(pdsc.Format);
    sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    sd.Texture2D.MipLevels = 1;
    device->CreateShaderResourceView(panel, &sd, cpu);
    const float full[4] = {0.f, 0.f, 1.f, 1.f};
    g_copy_list->RSSetViewports(1, &pvp);
    g_copy_list->SetPipelineState(g_mir_pso_blend);
    g_copy_list->SetGraphicsRoot32BitConstants(0, 4, full, 0);
    g_copy_list->SetGraphicsRootDescriptorTable(1, gpu);
    g_copy_list->DrawInstanced(3, 1, 0, 0);  // the layer is simultaneous-access: read from COMMON
  }
  if (panel != nullptr) {
    panel->Release();
  }
  if (direct) {
    transition(xr_eye, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
  }
  transition(back, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
  g_mirror_draws.fetch_add(1, std::memory_order_relaxed);
}

// ---- main menu panel opacity (2026-11-08) ---------------------------------------------------------------------------
// HEADSET/AUTHOR 2026-11-08: the 3D main menu's panel is too see-through. The game draws its menu art with partial alpha
// meant for blending onto the 3D scene, so on the quad layer the panels let the world through. Before the panel copy,
// a full-screen pass raises the coverage, a' = 1 - (1 - a)^k, keeping each pixel's colour (premultiplied rgb scaled by
// a'/a; alpha-0 additive art unchanged), into a texture of ours that is then copied into the swapchain as before.
// "panelalpha K" in [xr], 1..4; 1 = the plain copy. Default 2 (a = 0.5 -> 0.75). g_panel_alpha is with the panel state.
ID3D12RootSignature* g_gop_rs = nullptr;
ID3D12PipelineState* g_gop_pso = nullptr;
ID3D12DescriptorHeap* g_gop_srv_heap = nullptr;  // one SRV per copy allocator slot
ID3D12DescriptorHeap* g_gop_rtv_heap = nullptr;
ID3D12Resource* g_gop_tex = nullptr;  // R8G8B8A8_UNORM, COPY_SOURCE between uses
UINT g_gop_w = 0, g_gop_h = 0, g_gop_srv_stride = 0;
bool g_gop_failed = false;
std::atomic<std::uint64_t> g_gop_draws{0};

bool ensure_panel_opacity(ID3D12Device* device, UINT w, UINT h) {
  if (g_gop_failed) {
    return false;
  }
  if (g_gop_pso == nullptr) {
    static const char kShader[] =
        "cbuffer C : register(b0) { float4 k; };\n"
        "Texture2D T : register(t0);\n"
        "float4 vs(uint id : SV_VertexID) : SV_Position { float2 q = float2((id << 1) & 2, id & 2);\n"
        "  return float4(q * float2(2, -2) + float2(-1, 1), 0, 1); }\n"
        "float4 ps(float4 p : SV_Position) : SV_Target {\n"
        "  float4 c = T.Load(int3(p.xy, 0)); float a = saturate(c.a); float b = 1 - pow(1 - a, k.x);\n"
        "  float s = a > 0.002 ? b / a : 1; return float4(saturate(c.rgb * s), a > 0.002 ? b : a); }\n";
    ID3DBlob* vs = nullptr;
    ID3DBlob* ps = nullptr;
    ID3DBlob* err = nullptr;
    bool ok = SUCCEEDED(D3DCompile(kShader, sizeof(kShader) - 1, "panelalpha", nullptr, nullptr, "vs", "vs_5_0", 0, 0, &vs, &err)) &&
              SUCCEEDED(D3DCompile(kShader, sizeof(kShader) - 1, "panelalpha", nullptr, nullptr, "ps", "ps_5_0", 0, 0, &ps, &err));
    if (!ok && err != nullptr) {
      log_line("panel opacity: shader compile failed: %s", static_cast<const char*>(err->GetBufferPointer()));
    }
    if (ok) {
      D3D12_DESCRIPTOR_RANGE range = {D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0, 0};
      D3D12_ROOT_PARAMETER params[2] = {};
      params[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
      params[0].Constants.Num32BitValues = 4;
      params[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
      params[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
      params[1].DescriptorTable.NumDescriptorRanges = 1;
      params[1].DescriptorTable.pDescriptorRanges = &range;
      params[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
      D3D12_ROOT_SIGNATURE_DESC rsd = {2, params, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_NONE};
      ID3DBlob* blob = nullptr;
      ok = SUCCEEDED(D3D12SerializeRootSignature(&rsd, D3D_ROOT_SIGNATURE_VERSION_1, &blob, nullptr)) && blob != nullptr &&
           SUCCEEDED(device->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&g_gop_rs)));
      if (blob != nullptr) {
        blob->Release();
      }
    }
    if (ok) {
      D3D12_GRAPHICS_PIPELINE_STATE_DESC pd = {};
      pd.pRootSignature = g_gop_rs;
      pd.VS = {vs->GetBufferPointer(), vs->GetBufferSize()};
      pd.PS = {ps->GetBufferPointer(), ps->GetBufferSize()};
      pd.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
      pd.SampleMask = UINT_MAX;
      pd.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
      pd.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
      pd.RasterizerState.DepthClipEnable = TRUE;
      pd.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
      pd.NumRenderTargets = 1;
      pd.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
      pd.SampleDesc.Count = 1;
      ok = SUCCEEDED(device->CreateGraphicsPipelineState(&pd, IID_PPV_ARGS(&g_gop_pso)));
    }
    if (ok) {
      D3D12_DESCRIPTOR_HEAP_DESC hd = {D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, 3, D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE, 0};
      D3D12_DESCRIPTOR_HEAP_DESC rd = {D3D12_DESCRIPTOR_HEAP_TYPE_RTV, 1, D3D12_DESCRIPTOR_HEAP_FLAG_NONE, 0};
      ok = SUCCEEDED(device->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_gop_srv_heap))) &&
           SUCCEEDED(device->CreateDescriptorHeap(&rd, IID_PPV_ARGS(&g_gop_rtv_heap)));
      g_gop_srv_stride = ok ? device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV) : 0;
    }
    for (ID3DBlob* b : {vs, ps, err}) {
      if (b != nullptr) {
        b->Release();
      }
    }
    if (!ok) {
      g_gop_failed = true;
      log_line("panel opacity: pipeline creation failed; plain panel copy");
      return false;
    }
    log_line("panel opacity: pipeline ready");
  }
  if (g_gop_tex == nullptr || g_gop_w != w || g_gop_h != h) {
    if (g_gop_tex != nullptr) {  // panel resize: an earlier copy list may still read it, so retire it a few sizes later
      static ID3D12Resource* retired[4] = {};
      static int next = 0;
      if (retired[next] != nullptr) {
        retired[next]->Release();
      }
      retired[next] = g_gop_tex;
      next = (next + 1) % 4;
      g_gop_tex = nullptr;
    }
    D3D12_HEAP_PROPERTIES heap = {D3D12_HEAP_TYPE_DEFAULT};
    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = w;
    desc.Height = h;
    desc.DepthOrArraySize = desc.MipLevels = 1;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.SampleDesc.Count = 1;
    desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
    if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_SOURCE, nullptr,
                                               IID_PPV_ARGS(&g_gop_tex)))) {
      g_gop_tex = nullptr;
      g_gop_failed = true;
      log_line("panel opacity: %ux%u texture failed; plain panel copy", w, h);
      return false;
    }
    g_gop_w = w;
    g_gop_h = h;
    D3D12_RENDER_TARGET_VIEW_DESC rv = {};
    rv.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    rv.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    device->CreateRenderTargetView(g_gop_tex, &rv, g_gop_rtv_heap->GetCPUDescriptorHandleForHeapStart());
  }
  return true;
}

// Records the opacity pass from the panel layer (simultaneous-access, read from COMMON) into g_gop_tex. False = the
// caller copies the layer itself (k = 1, or the pass is unavailable).
bool record_panel_opacity(ID3D12Device* device, ID3D12Resource* layer, int allocator_slot) {
  const float k = g_panel_alpha.load(std::memory_order_relaxed);
  const D3D12_RESOURCE_DESC ld = layer->GetDesc();
  if (k <= 1.001f || ld.SampleDesc.Count != 1 ||
      !ensure_panel_opacity(device, static_cast<UINT>(ld.Width), ld.Height)) {
    return false;
  }
  D3D12_CPU_DESCRIPTOR_HANDLE cpu = g_gop_srv_heap->GetCPUDescriptorHandleForHeapStart();
  D3D12_GPU_DESCRIPTOR_HANDLE gpu = g_gop_srv_heap->GetGPUDescriptorHandleForHeapStart();
  cpu.ptr += static_cast<SIZE_T>(allocator_slot) * g_gop_srv_stride;
  gpu.ptr += static_cast<UINT64>(allocator_slot) * g_gop_srv_stride;
  D3D12_SHADER_RESOURCE_VIEW_DESC sd = {};
  sd.Format = view_format(ld.Format);
  sd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
  sd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
  sd.Texture2D.MipLevels = 1;
  device->CreateShaderResourceView(layer, &sd, cpu);
  transition(g_gop_tex, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
  const D3D12_CPU_DESCRIPTOR_HANDLE rtv = g_gop_rtv_heap->GetCPUDescriptorHandleForHeapStart();
  const D3D12_VIEWPORT vp = {0.f, 0.f, static_cast<float>(g_gop_w), static_cast<float>(g_gop_h), 0.f, 1.f};
  const D3D12_RECT sc = {0, 0, static_cast<LONG>(g_gop_w), static_cast<LONG>(g_gop_h)};
  const float constants[4] = {k, 0.f, 0.f, 0.f};
  g_copy_list->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
  g_copy_list->RSSetViewports(1, &vp);
  g_copy_list->RSSetScissorRects(1, &sc);
  g_copy_list->SetGraphicsRootSignature(g_gop_rs);
  g_copy_list->SetPipelineState(g_gop_pso);
  ID3D12DescriptorHeap* heaps[] = {g_gop_srv_heap};
  g_copy_list->SetDescriptorHeaps(1, heaps);
  g_copy_list->SetGraphicsRoot32BitConstants(0, 4, constants, 0);
  g_copy_list->SetGraphicsRootDescriptorTable(1, gpu);
  g_copy_list->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  g_copy_list->DrawInstanced(3, 1, 0, 0);
  transition(g_gop_tex, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
  g_gop_draws.fetch_add(1, std::memory_order_relaxed);
  return true;
}

// ---- dark right eye watchdog (2026-10-09) ---------------------------------------------------------------------
// OPEN fault (research/DEBUG_TESTERS.md): the right eye sometimes loses all ambient / indirect light and stays dark
// until a loading screen builds a new second view. Once a second a 128x128 centre crop of both eye images is copied
// into readback buffers on the eye-copy list (no wait); a later Present compares their mean brightness. Right below
// half of left for 3 samples in a row (and the left not simply dark) -> native_stereo rebuilds the second view, as a
// loading screen does, and the moment is logged so the trigger can be found. At most 5 rebuilds per session.
constexpr UINT kLumaCrop = 128;
ID3D12Resource* g_luma_rb[2] = {};
UINT64 g_luma_fence = 0;
bool g_luma_pending = false;
ULONGLONG g_luma_last = 0;
int g_dark_streak = 0;
int g_dark_rebuilds = 0;
ULONGLONG g_dark_cooldown_until = 0;
ULONGLONG g_luma_log_last = 0;

bool luma_format(DXGI_FORMAT f) {
  return f == DXGI_FORMAT_R8G8B8A8_UNORM || f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_R8G8B8A8_TYPELESS ||
         f == DXGI_FORMAT_B8G8R8A8_UNORM || f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_TYPELESS;
}

bool ensure_luma(ID3D12Device* device) {
  if (g_luma_rb[0] != nullptr) {
    return true;
  }
  D3D12_HEAP_PROPERTIES hp = {};
  hp.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC rd = {};
  rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  rd.Width = static_cast<UINT64>(kLumaCrop) * kLumaCrop * 4;
  rd.Height = 1;
  rd.DepthOrArraySize = 1;
  rd.MipLevels = 1;
  rd.SampleDesc.Count = 1;
  rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  for (auto*& b : g_luma_rb) {
    if (FAILED(device->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&b)))) {
      b = nullptr;
      return false;
    }
  }
  return true;
}

double luma_mean(ID3D12Resource* rb) {
  void* p = nullptr;
  D3D12_RANGE range = {0, static_cast<SIZE_T>(kLumaCrop) * kLumaCrop * 4};
  if (FAILED(rb->Map(0, &range, &p)) || p == nullptr) {
    return -1.0;
  }
  const unsigned char* px = static_cast<const unsigned char*>(p);
  std::uint64_t sum = 0;
  for (UINT i = 0; i < kLumaCrop * kLumaCrop; ++i) {
    sum += static_cast<std::uint64_t>(px[i * 4]) + px[i * 4 + 1] + px[i * 4 + 2];  // channel order does not matter
  }
  D3D12_RANGE none = {0, 0};
  rb->Unmap(0, &none);
  return static_cast<double>(sum) / (3.0 * kLumaCrop * kLumaCrop);
}

// Present thread: read the last sample once the GPU has finished it.
void luma_poll() {
  if (!g_luma_pending || g_copy_fence == nullptr || g_copy_fence->GetCompletedValue() < g_luma_fence) {
    return;
  }
  g_luma_pending = false;
  const double l = luma_mean(g_luma_rb[0]), r = luma_mean(g_luma_rb[1]);
  if (l < 0.0 || r < 0.0) {
    return;
  }
  const double ratio = l > 0.5 ? r / l : 1.0;
  const ULONGLONG now = GetTickCount64();
  const bool dark = l > 12.0 && ratio < 0.5;
  g_dark_streak = dark ? g_dark_streak + 1 : 0;
  if (now - g_luma_log_last >= 10000 || dark) {
    g_luma_log_last = now;
    log_line("eye brightness: left %.1f right %.1f (ratio %.2f)%s", l, r, ratio, dark ? " DARK RIGHT" : "");
  }
  // 10 s, not 3: a hand held in front of the face (arm IK) darkens one eye's centre crop for a few seconds, which
  // triggered rebuilds 5 times in 3 minutes (2026-10-15); the real fault stays dark until a load.
  if (g_dark_streak >= 10 && now >= g_dark_cooldown_until) {
    g_dark_streak = 0;
    g_dark_cooldown_until = now + 15000;
    if (g_dark_rebuilds < 5) {
      ++g_dark_rebuilds;
      log_line("dark right eye for 10 s (left %.1f right %.1f): rebuilding the second view (%d of 5 this session)", l, r, g_dark_rebuilds);
      native_stereo::request_rebuild("dark right eye");
    } else {
      log_line("dark right eye again; rebuild limit reached (load a save or fast travel)");
    }
  }
}

// Records the crops into g_copy_list (sources readable in their current state: backbuffer in COPY_SOURCE, second view
// simultaneous-access). True when recorded; the caller sets g_luma_fence after its Signal.
bool luma_record(ID3D12Device* device, const D3D12_TEXTURE_COPY_LOCATION& left, DXGI_FORMAT lf, const D3D12_TEXTURE_COPY_LOCATION& right,
                 DXGI_FORMAT rf, UINT w, UINT h) {
  const ULONGLONG now = GetTickCount64();
  if (g_luma_pending || now - g_luma_last < 1000 || w < kLumaCrop || h < kLumaCrop || !luma_format(lf) || !luma_format(rf) ||
      !ensure_luma(device)) {
    return false;
  }
  g_luma_last = now;
  const UINT x = (w - kLumaCrop) / 2, y = (h - kLumaCrop) / 2;
  const D3D12_BOX box = {x, y, 0, x + kLumaCrop, y + kLumaCrop, 1};
  const D3D12_TEXTURE_COPY_LOCATION* src[2] = {&left, &right};
  const DXGI_FORMAT fmt[2] = {lf, rf};
  for (int e = 0; e < 2; ++e) {
    D3D12_TEXTURE_COPY_LOCATION dst = {};
    dst.pResource = g_luma_rb[e];
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint.Offset = 0;
    dst.PlacedFootprint.Footprint = {fmt[e], kLumaCrop, kLumaCrop, 1, kLumaCrop * 4};
    g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, src[e], &box);
  }
  g_luma_pending = true;
  return true;
}

// Present thread, before the original Present. Copies the finished backbuffer into the eye it was rendered for.
// ---- headset burst capture (scripts\AvatarVR_burst.txt) -------------------------------------------------------
// Writing a new tag into AvatarVR_burst.txt records the next kBurstFrames stereo pairs exactly as they are copied into
// the eye swapchains, at full speed: a centre crop of the left source (backbuffer) and the right source (second-view
// output) is copied into readback buffers on the same command list, with no CPU wait. poll() saves them about 1 s
// later as scripts\captures\burst_<tag>_f<k>_L.bmp / _R.bmp and logs each frame's QPC time and right-eye texture.
// Purpose: see whether the right eye the headset receives repeats or lags frames (the F9 / quad captures stall the
// game and would hide a race between the second view's GPU work and this copy).
constexpr int kBurstFrames = 12;
constexpr UINT kBurstCrop = 1024;
struct BurstFrame {
  ID3D12Resource* rb[2] = {};
  long long qpc = 0;
  const void* right_tex = nullptr;
  bool paired = false, identity = false;
  DXGI_FORMAT fmt[2] = {};
};
BurstFrame g_burst[kBurstFrames];
int g_burst_next = 0, g_burst_want = 0;
ULONGLONG g_burst_done_at = 0;
std::string g_burst_tag, g_burst_last_tag;
bool g_burst_first_poll = true;

ID3D12Resource* burst_buffer(ID3D12Device* device) {
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC bd = {};
  bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  bd.Width = static_cast<UINT64>(kBurstCrop) * 4 * kBurstCrop;
  bd.Height = 1;
  bd.DepthOrArraySize = 1;
  bd.MipLevels = 1;
  bd.SampleDesc.Count = 1;
  bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  ID3D12Resource* r = nullptr;
  if (FAILED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &bd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                             IID_PPV_ARGS(&r)))) {
    return nullptr;
  }
  return r;
}

// Present thread, inside copy_present with the copy list open: both eye sources are readable (backbuffer in
// COPY_SOURCE, second-view output simultaneous-access).
void burst_record(ID3D12Device* device, ID3D12GraphicsCommandList* list, const D3D12_TEXTURE_COPY_LOCATION& left,
                  const D3D12_TEXTURE_COPY_LOCATION& right, UINT width, UINT height, bool paired, bool identity) {
  if (g_burst_next >= g_burst_want || width < kBurstCrop || height < kBurstCrop) {
    return;
  }
  BurstFrame& f = g_burst[g_burst_next];
  const UINT x0 = (width - kBurstCrop) / 2, y0 = (height - kBurstCrop) / 2;
  const D3D12_TEXTURE_COPY_LOCATION* src[2] = {&left, &right};
  for (int e = 0; e < 2; ++e) {
    if (f.rb[e] == nullptr) {
      f.rb[e] = burst_buffer(device);
    }
    if (f.rb[e] == nullptr) {
      return;
    }
    f.fmt[e] = src[e]->pResource->GetDesc().Format;
    D3D12_TEXTURE_COPY_LOCATION dst = {};
    dst.pResource = f.rb[e];
    dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    dst.PlacedFootprint.Offset = 0;
    dst.PlacedFootprint.Footprint = {f.fmt[e], kBurstCrop, kBurstCrop, 1, kBurstCrop * 4};
    const D3D12_BOX box = {x0, y0, 0, x0 + kBurstCrop, y0 + kBurstCrop, 1};
    list->CopyTextureRegion(&dst, 0, 0, 0, src[e], &box);
  }
  LARGE_INTEGER q;
  QueryPerformanceCounter(&q);
  f.qpc = q.QuadPart;
  f.right_tex = right.pResource;
  f.paired = paired;
  f.identity = identity;
  if (++g_burst_next == g_burst_want) {
    g_burst_done_at = GetTickCount64();
  }
}

void burst_save_bmp(const std::wstring& path, const unsigned char* rgba, DXGI_FORMAT fmt) {
  const bool bgra = fmt == DXGI_FORMAT_B8G8R8A8_UNORM || fmt == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  const UINT row = kBurstCrop * 3;
  std::vector<unsigned char> img(static_cast<size_t>(row) * kBurstCrop);
  for (UINT y = 0; y < kBurstCrop; ++y) {
    const unsigned char* s = rgba + static_cast<size_t>(y) * kBurstCrop * 4;
    unsigned char* d = img.data() + static_cast<size_t>(kBurstCrop - 1 - y) * row;
    for (UINT x = 0; x < kBurstCrop; ++x) {
      d[x * 3 + 0] = s[x * 4 + (bgra ? 0 : 2)];
      d[x * 3 + 1] = s[x * 4 + 1];
      d[x * 3 + 2] = s[x * 4 + (bgra ? 2 : 0)];
    }
  }
  HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  unsigned char hdr[54] = {'B', 'M'};
  const uint32_t size = static_cast<uint32_t>(img.size()), file_size = 54 + size, off = 54, info = 40, wh = kBurstCrop;
  const uint16_t planes = 1, bpp = 24;
  std::memcpy(hdr + 2, &file_size, 4);
  std::memcpy(hdr + 10, &off, 4);
  std::memcpy(hdr + 14, &info, 4);
  std::memcpy(hdr + 18, &wh, 4);
  std::memcpy(hdr + 22, &wh, 4);
  std::memcpy(hdr + 26, &planes, 2);
  std::memcpy(hdr + 28, &bpp, 2);
  std::memcpy(hdr + 34, &size, 4);
  DWORD written = 0;
  WriteFile(file, hdr, sizeof(hdr), &written, nullptr);
  WriteFile(file, img.data(), size, &written, nullptr);
  CloseHandle(file);
}

// Present thread (poll): trigger on a new tag, save a finished burst.
void burst_poll(const std::wstring& dir) {
  char text[64] = {};
  HANDLE file = CreateFileW((dir + L"\\AvatarVR_burst.txt").c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  std::string tag;
  if (file != INVALID_HANDLE_VALUE) {
    DWORD read = 0;
    ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
    CloseHandle(file);
    for (DWORD i = 0; i < read; ++i) {
      const char c = text[i];
      if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') {
        tag += c;
      }
    }
  }
  if (g_burst_first_poll) {  // a tag already present at launch is not a request
    g_burst_first_poll = false;
    g_burst_last_tag = tag;
    return;
  }
  std::lock_guard<std::mutex> lock(g_xr);
  if (!tag.empty() && tag != g_burst_last_tag && g_burst_want == 0) {
    g_burst_last_tag = tag;
    g_burst_tag = tag;
    g_burst_next = 0;
    g_burst_want = kBurstFrames;
    log_line("burst %s: armed (%d stereo pairs, centre %ux%u)", tag.c_str(), kBurstFrames, kBurstCrop, kBurstCrop);
    return;
  }
  if (g_burst_want == 0 || g_burst_next < g_burst_want || GetTickCount64() - g_burst_done_at < 1000) {
    return;
  }
  const std::wstring folder = dir + L"\\AvatarVR\\captures";
  CreateDirectoryW(folder.c_str(), nullptr);
  LARGE_INTEGER freq;
  QueryPerformanceFrequency(&freq);
  for (int k = 0; k < g_burst_want; ++k) {
    BurstFrame& f = g_burst[k];
    for (int e = 0; e < 2; ++e) {
      void* mapped = nullptr;
      const D3D12_RANGE range = {0, static_cast<SIZE_T>(kBurstCrop) * 4 * kBurstCrop};
      if (f.rb[e] != nullptr && SUCCEEDED(f.rb[e]->Map(0, &range, &mapped)) && mapped != nullptr) {
        const std::wstring name = folder + L"\\burst_" + std::wstring(g_burst_tag.begin(), g_burst_tag.end()) + L"_f" +
                                  std::to_wstring(k) + (e == 0 ? L"_L.bmp" : L"_R.bmp");
        burst_save_bmp(name, static_cast<const unsigned char*>(mapped), f.fmt[e]);
        const D3D12_RANGE none = {0, 0};
        f.rb[e]->Unmap(0, &none);
      }
    }
    const double ms = k == 0 ? 0.0 : 1000.0 * static_cast<double>(f.qpc - g_burst[k - 1].qpc) / static_cast<double>(freq.QuadPart);
    log_line("burst %s f%d: +%.2f ms right texture %p paired=%d identity=%d", g_burst_tag.c_str(), k, ms, f.right_tex,
             f.paired ? 1 : 0, f.identity ? 1 : 0);
  }
  g_burst_want = 0;
  g_burst_next = 0;
}

// Observer only: distinguish queue backlog from CPU waits inside eye transport.
// Per-thread statistics remain safe even if another Present thread is observed.
// No waits, queue operations, image ownership changes or GPU queries are added.
struct CopyPerfProbe {
  enum Phase { Total, Mutex, Allocator, Acquire, ImageWait, Submit, Count };
  bool active = false;
  LONGLONG started = 0;
  double values[Count] = {};
  static inline thread_local LONGLONG frequency = 0;
  static inline thread_local ULONGLONG first = 0, reported = 0;
  static inline thread_local bool done = false;
  static inline thread_local unsigned frames = 0;
  static inline thread_local double sums[Count] = {}, maxima[Count] = {};

  CopyPerfProbe() {
    if (done || !g_copy_perf_probe.load(std::memory_order_relaxed) || !native_stereo::stereo_running()) return;
    active = true;
    if (frequency == 0) {
      LARGE_INTEGER f; QueryPerformanceFrequency(&f); frequency = f.QuadPart;
    }
    if (first == 0) {
      first = reported = GetTickCount64();
      log_line("transport CPU probe: started; 60s bound, avg/max per observed Present; overlapping total/parts, not GPU time");
    }
    started = now();
  }
  LONGLONG now() const {
    if (!active) return 0;
    LARGE_INTEGER t; QueryPerformanceCounter(&t); return t.QuadPart;
  }
  void add(Phase phase, LONGLONG begin) {
    if (active) values[phase] += static_cast<double>(now() - begin) * 1000.0 / frequency;
  }
  ~CopyPerfProbe() {
    if (!active) return;
    add(Total, started);
    ++frames;
    for (int i = 0; i < Count; ++i) {
      sums[i] += values[i]; maxima[i] = std::max(maxima[i], values[i]);
    }
    const ULONGLONG tick = GetTickCount64();
    done = tick - first >= 60000;
    if (tick - reported < 5000 && !done) return;
    log_line("transport CPU: thread %lu %u presents | total %.3f/%.3f mutex %.3f/%.3f allocator %.3f/%.3f acquire %.3f/%.3f imagewait %.3f/%.3f submit %.3f/%.3f ms (avg/max; CPU wall time, nested parts; %s)",
             GetCurrentThreadId(), frames, sums[Total]/frames, maxima[Total], sums[Mutex]/frames, maxima[Mutex],
             sums[Allocator]/frames, maxima[Allocator], sums[Acquire]/frames, maxima[Acquire],
             sums[ImageWait]/frames, maxima[ImageWait], sums[Submit]/frames, maxima[Submit], done ? "stopped after 60s" : "sampling");
    frames = 0; reported = tick;
    for (int i = 0; i < Count; ++i) sums[i] = maxima[i] = 0;
  }
};

void copy_present(IDXGISwapChain* swapchain) {
  CopyPerfProbe perf;  // destroyed after lock: log formatting/I/O is outside g_xr
  const LONGLONG lock_begin = perf.now();
  std::lock_guard<std::mutex> lock(g_xr);
  perf.add(CopyPerfProbe::Mutex, lock_begin);
  if (!g_running || g_session == XR_NULL_HANDLE || swapchain == nullptr) {
    return;
  }
  ID3D12CommandQueue* queue = acquire_queue();
  if (queue == nullptr) {
    return;
  }
  IDXGISwapChain3* sc3 = nullptr;
  ID3D12Resource* back = nullptr;
  ID3D12Device* device = nullptr;
  ID3D12Resource* right_held = nullptr;
  ID3D12Resource* left_held = nullptr;  // eye mode: the main view's offscreen output (left eye)
  do {
    if (FAILED(swapchain->QueryInterface(IID_PPV_ARGS(&sc3))) || sc3 == nullptr) {
      break;
    }
    if (FAILED(sc3->GetBuffer(sc3->GetCurrentBackBufferIndex(), IID_PPV_ARGS(&back))) || back == nullptr) {
      break;
    }
    const D3D12_RESOURCE_DESC desc = back->GetDesc();
    // Eye mode ("eyesize W H both"): both eyes render offscreen at their own size; the chains follow it and the
    // backbuffer (any window size) only gets the mirror.
    // The main view renders the left eye into the top-left W x H of the (grown) backbuffer.
    std::uint32_t eye_w = 0, eye_h = 0;
    bool eye_mode = false;
    if (g_native.load() && native_stereo::eye_size(&eye_w, &eye_h)) {
      eye_mode = desc.Width >= eye_w && desc.Height >= eye_h;
      if (!eye_mode) {
        g_eye_mode_mismatch.fetch_add(1, std::memory_order_relaxed);  // buffers not grown yet (resize pending)
        // Do not submit a monitor-sized crop with a full-eye projection. Wait
        // for the engine-owned resize; preserve the last correctly sized pair.
        static ULONGLONG last_resize_warning = 0;
        const ULONGLONG now = GetTickCount64();
        if (now - last_resize_warning >= 1000) {
          last_resize_warning = now;
          log_line("eye resize pending: buffers %llux%u need %ux%u; cropped stereo copy skipped",
                   static_cast<unsigned long long>(desc.Width), desc.Height, eye_w, eye_h);
        }
        break;
      }
    }
    // Grown buffers outside gameplay (menus, loading): the game draws only the window-size top-left region, so the
    // cinema screen gets that region, not the whole buffer (run 18:4x: the menu sat in a corner of the screen).
    unsigned client_w = 0, client_h = 0;
    const bool grown = dx12::client_size(&client_w, &client_h);
    const uint32_t chain_w = eye_mode ? eye_w : grown ? client_w : static_cast<uint32_t>(desc.Width);
    const UINT chain_h = eye_mode ? eye_h : grown ? client_h : desc.Height;
    if (eye_mode) {
      g_eye_mode_presents.fetch_add(1, std::memory_order_relaxed);
    }
    if (FAILED(back->GetDevice(IID_PPV_ARGS(&device))) || device == nullptr || !ensure_copy_objects(device) ||
        !ensure_chains_locked(chain_w, chain_h, desc.Format)) {
      break;
    }
    int eye = eye_write::presented_eye();
    const int parity = eye_write::parity_eye();
    if (eye != parity) {
      g_parity_mismatch.fetch_add(1, std::memory_order_relaxed);
    }
    if (g_swap_eyes.load()) {
      eye = -eye;
    }
    ID3D12Resource* right = nullptr;
    if (g_native.load()) {
      right = native_stereo::acquire_view2_output();
      right_held = right;
      if (right == nullptr) {
        g_native_missing.fetch_add(1, std::memory_order_relaxed);
      } else {
        const D3D12_RESOURCE_DESC rd = right->GetDesc();
        // The engine's pooled output texture can be larger than the window (height rounded up); the image sits in
        // its top-left corner, so a texture at least as large is accepted and only that region is copied.
        if (rd.Width < chain_w || rd.Height < chain_h) {
          g_native_mismatch.fetch_add(1, std::memory_order_relaxed);
          right = nullptr;  // released below via right_held
        }
      }
      // Without a usable second-view image this Present goes to both eyes (mono); never alternate here.
      eye = 0;
      // native_stereo hands out no image while stereo is paused (start-up video, main menu, pause / inventory /
      // character menus) or before the second view exists. A few missing frames keep the head-tracked layer; a
      // longer gap drops it so the worker shows the mono image on the flat cinema screen instead of one eye.
      if (right == nullptr) {
        if (g_native_absent < kMonoAfterPresents) {
          ++g_native_absent;
        }
        if (g_native_absent == kMonoAfterPresents && g_layer_valid) {
          g_layer_valid = false;
          g_cin_lazy.store(true);      // keep the screen where it was if it is still in view
          g_cin_recentre.store(true);  // otherwise put it in front of the head
          log_line("native: no second-view image for %d presents; showing the mono cinema screen", kMonoAfterPresents);
        }
      } else {
        if (g_native_absent >= kMonoAfterPresents) {
          log_line("native: second-view image back; head-tracked stereo layer");
        }
        g_native_absent = 0;
      }
    }
    if (right == nullptr && !ensure_stage(device, desc)) {
      break;
    }
    // Allocator ring: never reset an allocator the GPU may still be executing.
    const int slot = g_allocator_index;
    g_allocator_index = (g_allocator_index + 1) % 3;
    if (g_copy_fence->GetCompletedValue() < g_allocator_fence_value[slot]) {
      const LONGLONG wait_begin = perf.now();
      g_copy_fence->SetEventOnCompletion(g_allocator_fence_value[slot], g_copy_event);
      const DWORD wait_result = WaitForSingleObject(g_copy_event, 100);
      perf.add(CopyPerfProbe::Allocator, wait_begin);
      if (wait_result != WAIT_OBJECT_0 ||
          g_copy_fence->GetCompletedValue() < g_allocator_fence_value[slot]) {
        break;  // never reset an allocator (or overwrite its SRV) while it is executing
      }
    }
    D3D12_TEXTURE_COPY_LOCATION back_loc = {};
    back_loc.pResource = back;
    back_loc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    back_loc.SubresourceIndex = 0;
    D3D12_TEXTURE_COPY_LOCATION stage_loc = {};
    stage_loc.pResource = g_stage;
    stage_loc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    stage_loc.SubresourceIndex = 0;

    if (eye < 0) {
      // Left eye: stash only. Nothing reaches the headset until its right eye arrives.
      g_allocators[slot]->Reset();
      g_copy_list->Reset(g_allocators[slot], nullptr);
      transition(back, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_SOURCE);
      g_copy_list->CopyTextureRegion(&stage_loc, 0, 0, 0, &back_loc, nullptr);
      transition(back, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_PRESENT);
      g_copy_list->Close();
      ID3D12CommandList* lists[] = {g_copy_list};
      queue->ExecuteCommandLists(1, lists);
      queue->Signal(g_copy_fence, ++g_copy_fence_next);
      g_allocator_fence_value[slot] = g_copy_fence_next;
      g_stage_valid = true;
      g_copies.fetch_add(1, std::memory_order_relaxed);
      g_copies_left.fetch_add(1, std::memory_order_relaxed);
      break;
    }
    if (eye > 0 && !g_stage_valid) {
      g_orphans.fetch_add(1, std::memory_order_relaxed);  // a right eye with no left to pair with
      break;
    }
    // Private present queue: never block the game. Three Presents behind = the runtime is holding it; drop this copy.
    const bool pq = g_pq_on && g_pq != nullptr;
    if (pq && (!ensure_pq_objects(device) || g_pq_fence->GetCompletedValue() < g_pq_slot_value[slot])) {
      g_pq_drops.fetch_add(1, std::memory_order_relaxed);
      break;
    }
    bool pq_src_ok[2] = {false, false};
    // Right eye (completes the pair) or mono (both eyes get this frame).
    uint32_t index[2] = {};
    bool acquired[2] = {};
    for (int e = 0; e < 2; ++e) {
      XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
      const LONGLONG acquire_begin = perf.now();
      const XrResult acquire_result = xrAcquireSwapchainImage(g_eyes[e].chain, &acquire_info, &index[e]);
      perf.add(CopyPerfProbe::Acquire, acquire_begin);
      if (XR_FAILED(acquire_result)) {
        continue;
      }
      XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
      wait_info.timeout = 100000000;  // 100 ms
      XrResult r = XR_TIMEOUT_EXPIRED;
      const LONGLONG image_wait_begin = perf.now();
      for (int attempt = 0; attempt < 5 && r == XR_TIMEOUT_EXPIRED; ++attempt) {
        r = xrWaitSwapchainImage(g_eyes[e].chain, &wait_info);
      }
      perf.add(CopyPerfProbe::ImageWait, image_wait_begin);
      acquired[e] = XR_SUCCEEDED(r) && r != XR_TIMEOUT_EXPIRED;
      if (!acquired[e]) {
        log_line("cinema: xrWaitSwapchainImage eye %d: %s", e, result_text(g_instance, r).c_str());
      }
    }
    if (!acquired[0] && !acquired[1]) {
      break;
    }
    g_allocators[slot]->Reset();
    g_copy_list->Reset(g_allocators[slot], nullptr);
    transition(back, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_SOURCE);
    const bool use_stage = eye > 0;
    if (use_stage) {
      transition(g_stage, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_COPY_SOURCE);
    }
    // Same-frame pairing ("pair"): the second view's image is one engine frame newer than the backbuffer at this
    // Present, so it is parked in a hold texture and shown with the NEXT backbuffer -- both eyes then come from
    // the same frame and the same pose snapshot instead of two consecutive ones (which reads as alternate-eye judder).
    bool paired = false;
    int hold_write = -1;
    // Pose identity: when native_stereo knows which applied pose the newest second-view image was built with, that image
    // and the backbuffer are the same engine frame (the game renders and presents each frame before starting the next;
    // trace 2026-10-05 13:01), so no hold is needed and both eyes are labelled with that pose.
    head_pose::Snapshot ident;
    bool identity = right != nullptr && native_stereo::presented_views(&ident);
    if (identity) {
      g_hold_valid = false;
    } else if (!eye_mode && right != nullptr && g_pair.load() && ensure_holds(device, desc, right->GetDesc().Format)) {
      const int read = g_hold_read;
      hold_write = 1 - read;
      if (g_hold_valid) {
        hold_state(read, D3D12_RESOURCE_STATE_COPY_SOURCE);
        paired = true;
      }
      hold_state(hold_write, D3D12_RESOURCE_STATE_COPY_DEST);
    } else if (right == nullptr || !g_pair.load()) {
      g_hold_valid = false;
    }
    D3D12_TEXTURE_COPY_LOCATION right_loc = {};
    right_loc.pResource = paired ? g_holds[g_hold_read] : right;
    right_loc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    right_loc.SubresourceIndex = 0;
    int right_eye = g_swap_eyes.load() ? 0 : 1;  // which XR eye gets the second view (AFW: the made eye)
    D3D12_TEXTURE_COPY_LOCATION left_loc = {};
    left_loc.pResource = back;
    left_loc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    left_loc.SubresourceIndex = 0;
    const bool timed_transport = eye_mode && right != nullptr && transport_begin(device, queue, slot);
    if (timed_transport) g_copy_list->EndQuery(g_transport_queries, D3D12_QUERY_TYPE_TIMESTAMP, slot*4);
    const float sharpness = g_sharpness.load(std::memory_order_relaxed);  // 0 = plain copies (View -> Sharpness)
    for (int e = 0; e < 2; ++e) {
      if (!acquired[e]) {
        continue;
      }
      ID3D12Resource* target = g_eyes[e].images[index[e]];
      // The second view's texture is simultaneous-access: copied from COMMON without barriers.
      const D3D12_TEXTURE_COPY_LOCATION& src = (right != nullptr && e == right_eye) ? right_loc
                                               : eye_mode                         ? left_loc
                                               : (use_stage && e == 0)            ? stage_loc
                                                                                  : back_loc;
      if (pq) {
        // Game queue: into this slot's transport texture only; the private queue writes the XR image below.
        PqTexture& t = g_pq_eye[slot][e];
        if (pq_texture(device, t, chain_w, chain_h, src.pResource->GetDesc().Format)) {
          D3D12_TEXTURE_COPY_LOCATION dst = {};
          dst.pResource = t.tex;
          dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
          dst.SubresourceIndex = 0;
          D3D12_BOX box = {0, 0, 0, chain_w, chain_h, 1};
          g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
          pq_src_ok[e] = true;
        }
        continue;
      }
      if (sharpness > 0.f && sharpen_eye(device, e, target, src, chain_w, chain_h, slot, sharpness)) {
        continue;  // sharpened copy drawn straight into the XR image, which stays in RENDER_TARGET
      }
      transition(target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
      D3D12_TEXTURE_COPY_LOCATION dst = {};
      dst.pResource = target;
      dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
      dst.SubresourceIndex = 0;
      D3D12_BOX box = {0, 0, 0, chain_w, chain_h, 1};
      g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);  // every source is at least chain-sized
      transition(target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
    }
    if (timed_transport) g_copy_list->EndQuery(g_transport_queries, D3D12_QUERY_TYPE_TIMESTAMP, slot*4+1);
    // Optional per-eye depth ("xrdepth"): each eye's depth comes from the same view as its colour (the second view's
    // eye is right_eye). Only in eye mode with the second view present, and not on the private queue.
    uint32_t depth_index[2] = {};
    bool depth_recorded[2] = {false, false};
    if (g_xr_depth.load(std::memory_order_relaxed)) {
      if (pq) {
        g_depth_pq_skips.fetch_add(1, std::memory_order_relaxed);
      } else if (!eye_mode || right == nullptr) {
        g_depth_not_eye_mode.fetch_add(1, std::memory_order_relaxed);
      } else if (!xr_depth::ready(100)) {
        g_depth_stale.fetch_add(1, std::memory_order_relaxed);
      } else if (ensure_depth_chains_locked(chain_w, chain_h)) {
        for (int e = 0; e < 2; ++e) {
          if (!acquired[e]) continue;
          XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
          if (XR_FAILED(xrAcquireSwapchainImage(g_depth_chains[e].chain, &acquire_info, &depth_index[e]))) {
            g_depth_acquire_fails.fetch_add(1, std::memory_order_relaxed);
            continue;
          }
          XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
          wait_info.timeout = 100000000;  // 100 ms
          const XrResult r = xrWaitSwapchainImage(g_depth_chains[e].chain, &wait_info);
          const bool waited = XR_SUCCEEDED(r) && r != XR_TIMEOUT_EXPIRED;
          const int view = e == right_eye ? 1 : 0;
          // The image arrives in DEPTH_WRITE (OpenXR D3D12 binding) and is left there.
          depth_recorded[e] = waited && xr_depth::record(device, g_copy_list, view,
                                                         g_depth_chains[e].images[depth_index[e]], chain_w, chain_h);
          if (!depth_recorded[e]) {
            g_depth_acquire_fails.fetch_add(1, std::memory_order_relaxed);
            if (waited) {
              // Waited but nothing written: release it so the chain stays usable (it is not submitted).
              XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
              xrReleaseSwapchainImage(g_depth_chains[e].chain, &release_info);
            }
          }
        }
      }
    }
    // In-mod frame generation: keep this real frame (both eye sources, its motion vectors and depth) for the worker.
    bool fg_kept = false;
    if (g_framegen.load(std::memory_order_relaxed) && !pq && eye_mode && right != nullptr && acquired[0] && acquired[1]) {
      ID3D12Resource* fg_src[2] = {};
      int fg_view[2] = {0, 0};
      for (int e = 0; e < 2; ++e) {
        fg_src[e] = e == right_eye ? right_loc.pResource : left_loc.pResource;
        fg_view[e] = e == right_eye ? 1 : 0;
      }
      fg_kept = frame_gen::keep_real(device, g_copy_list, fg_src, fg_view, chain_w, chain_h);
    }
    luma_poll();
    const bool luma_recorded =
        right != nullptr && !use_stage &&
        luma_record(device, eye_mode ? left_loc : back_loc, desc.Format, right_loc, right->GetDesc().Format, chain_w, chain_h);
    if (right != nullptr && !use_stage && !eye_mode) {
      burst_record(device, g_copy_list, back_loc, right_loc, static_cast<UINT>(desc.Width), desc.Height, paired, identity);
    }
    if (use_stage) {
      transition(g_stage, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_COPY_DEST);
    }
    if (hold_write >= 0) {
      // Park this Present's second-view image (simultaneous-access, read from COMMON) for the next Present.
      D3D12_TEXTURE_COPY_LOCATION src = {};
      src.pResource = right;
      src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
      src.SubresourceIndex = 0;
      D3D12_TEXTURE_COPY_LOCATION dst = {};
      dst.pResource = g_holds[hold_write];
      dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
      dst.SubresourceIndex = 0;
      D3D12_BOX box = {0, 0, 0, static_cast<UINT>(desc.Width), desc.Height, 1};
      g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
      g_hold_read = hold_write;
      g_hold_valid = true;
    }
    if (timed_transport) g_copy_list->EndQuery(g_transport_queries, D3D12_QUERY_TYPE_TIMESTAMP, slot*4+2);
    if (eye_mode) {
      const int left_eye = 1 - right_eye;
      const bool show_right = g_mirror_right.load(std::memory_order_relaxed) && right != nullptr && acquired[right_eye];
      const int mirror_index = show_right ? right_eye : left_eye;
      ID3D12Resource* mirror_eye = !pq && acquired[mirror_index] && g_eyes[mirror_index].sampleable
          ? g_eyes[mirror_index].images[index[mirror_index]] : nullptr;
      mirror_draw(device, show_right ? right_loc.pResource : back, chain_w, chain_h, back, desc, mirror_eye, slot);
    }
    if (timed_transport) {
      g_copy_list->EndQuery(g_transport_queries, D3D12_QUERY_TYPE_TIMESTAMP, slot*4+3);
      g_copy_list->ResolveQueryData(g_transport_queries, D3D12_QUERY_TYPE_TIMESTAMP, slot*4, 4,
                                    g_transport_readback, slot*4*sizeof(UINT64));
    }
    transition(back, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_PRESENT);
    // Game UI panel: the layer texture (simultaneous-access, read from COMMON) into the panel swapchain.
    bool gui_acquired = false;
    ID3D12Resource* gui_target = nullptr;  // pq: written by the private queue from g_pq_gui[slot]
    ID3D12Resource* gui = g_gui_visible.load(std::memory_order_relaxed) ? camera::ui_layer_acquire() : nullptr;
    if (gui != nullptr) {
      const D3D12_RESOURCE_DESC gd = gui->GetDesc();
      uint32_t gui_index = 0;
      XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
      if (ensure_gui_locked(static_cast<uint32_t>(gd.Width), gd.Height) &&
          XR_SUCCEEDED(xrAcquireSwapchainImage(g_gui_chain, &acquire_info, &gui_index)) && gui_index < g_gui_images.size()) {
        XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        wait_info.timeout = 50000000;  // 50 ms
        const XrResult wr = xrWaitSwapchainImage(g_gui_chain, &wait_info);
        if (XR_SUCCEEDED(wr) && wr != XR_TIMEOUT_EXPIRED) {
          ID3D12Resource* target = g_gui_images[gui_index];
          if (pq) {
            if (pq_texture(device, g_pq_gui[slot], gd.Width, gd.Height, gd.Format)) {
              g_copy_list->CopyResource(g_pq_gui[slot].tex, gui);  // both simultaneous-access: no barriers
              gui_target = target;
            }
          } else {
            const bool opaque = record_panel_opacity(device, gui, slot);  // more opaque menu art (panelalpha)
            transition(target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
            g_copy_list->CopyResource(target, opaque ? g_gop_tex : gui);
            transition(target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
          }
          gui_acquired = true;
        } else if (wr == XR_TIMEOUT_EXPIRED) {
          destroy_gui_locked();  // a timed-out image cannot be released; start over with a new chain
        } else {
          XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
          xrReleaseSwapchainImage(g_gui_chain, &release_info);
        }
      }
    }
    // Wrist HUD: the HUD texture (simultaneous-access, read from COMMON) into its swapchain.
    hud_capture_collect();
    bool hud_acquired = false;
    bool hud_captured = false;
    ID3D12Resource* hud_target = nullptr;  // pq: written by the private queue from g_pq_hud[slot]
    ID3D12Resource* hud = wrist_hud::current().on && native_stereo::stereo_running() &&
        !native_stereo::ui_layer_active() ? camera::hud_acquire() : nullptr;
    if (hud != nullptr) {
      const D3D12_RESOURCE_DESC hd = hud->GetDesc();
      uint32_t hud_index = 0;
      XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
      if (ensure_hud_locked(static_cast<uint32_t>(hd.Width), hd.Height) &&
          XR_SUCCEEDED(xrAcquireSwapchainImage(g_hud_chain, &acquire_info, &hud_index)) && hud_index < g_hud_images.size()) {
        XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        wait_info.timeout = 50000000;  // 50 ms
        const XrResult wr = xrWaitSwapchainImage(g_hud_chain, &wait_info);
        if (XR_SUCCEEDED(wr) && wr != XR_TIMEOUT_EXPIRED) {
          ID3D12Resource* target = g_hud_images[hud_index];
          if (pq) {
            if (pq_texture(device, g_pq_hud[slot], hd.Width, hd.Height, hd.Format)) {
              g_copy_list->CopyResource(g_pq_hud[slot].tex, hud);
              hud_target = target;
            }
            hud_captured = hud_capture_record(hud);
          } else {
            transition(target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
            g_copy_list->CopyResource(target, hud);
            hud_captured = hud_capture_record(hud);
            transition(target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
          }
          hud_acquired = true;
        } else if (wr == XR_TIMEOUT_EXPIRED) {
          destroy_hud_locked();
        } else {
          XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
          xrReleaseSwapchainImage(g_hud_chain, &release_info);
        }
      }
    }
    bool laser_recorded = !pq && menu_pointer::g_enabled.load(std::memory_order_relaxed) && record_laser_locked(device);
    bool fade_recorded = !pq && g_fade_on.load(std::memory_order_relaxed) && record_fade_locked(device);
    g_copy_list->Close();
    ID3D12CommandList* lists[] = {g_copy_list};
    const LONGLONG submit_begin = perf.now();
    if (fg_kept) frame_gen::gate(queue);  // the worker may still read the colour bank this list overwrites
    queue->ExecuteCommandLists(1, lists);
    queue->Signal(g_copy_fence, ++g_copy_fence_next);
    if (fg_kept) frame_gen::real_submitted(g_copy_fence, g_copy_fence_next);
    g_real_kept = fg_kept;
    perf.add(CopyPerfProbe::Submit, submit_begin);
    g_allocator_fence_value[slot] = g_copy_fence_next;
    if (pq) {
      // Private queue: wait for the game queue's transport copies, then write every acquired swapchain image. The
      // helpers below (transition, sharpen_eye, record_panel_opacity, sprite records) record into g_copy_list.
      const UINT64 handoff = g_copy_fence_next;
      ID3D12GraphicsCommandList* game_list = g_copy_list;
      g_pq_alloc[slot]->Reset();
      g_pq_list->Reset(g_pq_alloc[slot], nullptr);
      g_copy_list = g_pq_list;
      for (int e = 0; e < 2; ++e) {
        if (!acquired[e] || !pq_src_ok[e]) {
          continue;
        }
        ID3D12Resource* target = g_eyes[e].images[index[e]];
        D3D12_TEXTURE_COPY_LOCATION src = {};
        src.pResource = g_pq_eye[slot][e].tex;
        src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        src.SubresourceIndex = 0;
        if (sharpness > 0.f && sharpen_eye(device, e, target, src, chain_w, chain_h, slot, sharpness)) {
          continue;
        }
        transition(target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
        D3D12_TEXTURE_COPY_LOCATION dst = {};
        dst.pResource = target;
        dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        dst.SubresourceIndex = 0;
        D3D12_BOX box = {0, 0, 0, chain_w, chain_h, 1};
        g_copy_list->CopyTextureRegion(&dst, 0, 0, 0, &src, &box);
        transition(target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
      }
      if (gui_target != nullptr) {
        ID3D12Resource* staged = g_pq_gui[slot].tex;
        const bool opaque = record_panel_opacity(device, staged, slot);
        transition(gui_target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
        g_copy_list->CopyResource(gui_target, opaque ? g_gop_tex : staged);
        transition(gui_target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
      }
      if (hud_target != nullptr) {
        transition(hud_target, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
        g_copy_list->CopyResource(hud_target, g_pq_hud[slot].tex);
        transition(hud_target, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
      }
      laser_recorded = menu_pointer::g_enabled.load(std::memory_order_relaxed) && record_laser_locked(device);
      fade_recorded = g_fade_on.load(std::memory_order_relaxed) && record_fade_locked(device);
      g_pq_list->Close();
      g_copy_list = game_list;
      g_pq->Wait(g_copy_fence, handoff);
      ID3D12CommandList* pq_lists[] = {g_pq_list};
      g_pq->ExecuteCommandLists(1, pq_lists);
      g_pq->Signal(g_pq_fence, ++g_pq_fence_next);
      g_pq_slot_value[slot] = g_pq_fence_next;
      g_pq_frames.fetch_add(1, std::memory_order_relaxed);
      static ULONGLONG pq_logged = 0;
      if (GetTickCount64() - pq_logged >= 5000) {
        pq_logged = GetTickCount64();
        log_line("ownqueue: %llu frames written on the private queue, %llu dropped (runtime 3 behind), %llu menu bridges",
                 static_cast<unsigned long long>(g_pq_frames.load()), static_cast<unsigned long long>(g_pq_drops.load()),
                 static_cast<unsigned long long>(g_pq_bridges.load()));
      }
    }
    if (laser_recorded) {
      XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      g_laser_ready = XR_SUCCEEDED(xrReleaseSwapchainImage(g_laser_chain, &release_info));
      g_laser_failed = !g_laser_ready;
    }
    if (fade_recorded) {
      XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      g_fade_ready = XR_SUCCEEDED(xrReleaseSwapchainImage(g_fade_chain, &release_info));
      g_fade_failed = !g_fade_ready;
    }
    if (timed_transport) g_transport_pending[slot] = true;
    if (hud_captured) {
      g_hud_readback_fence = g_copy_fence_next;
      g_hud_readback_done = g_copy_fence;
      g_hud_readback_done->AddRef();
    }
    if (luma_recorded) {
      g_luma_fence = g_copy_fence_next;
    }
    if (gui_acquired) {
      XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      if (XR_SUCCEEDED(xrReleaseSwapchainImage(g_gui_chain, &release_info))) {
        g_gui_has_image = true;
        g_gui_copies.fetch_add(1, std::memory_order_relaxed);
      }
    }
    if (gui != nullptr) {
      gui->Release();
    }
    if (hud_acquired) {
      XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      if (XR_SUCCEEDED(xrReleaseSwapchainImage(g_hud_chain, &release_info))) {
        g_hud_has_image = true;
        g_hud_copied_at.store(GetTickCount64(), std::memory_order_relaxed);
        g_hud_copies.fetch_add(1, std::memory_order_relaxed);
      }
    }
    if (hud != nullptr) {
      hud->Release();
    }
    for (int e = 0; e < 2; ++e) {
      if (!acquired[e]) {
        continue;
      }
      XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      if (XR_SUCCEEDED(xrReleaseSwapchainImage(g_eyes[e].chain, &release_info))) {
        g_eyes[e].has_image = true;
      }
      bool depth_ok = false;
      if (depth_recorded[e]) {
        XrSwapchainImageReleaseInfo depth_release = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        depth_ok = XR_SUCCEEDED(xrReleaseSwapchainImage(g_depth_chains[e].chain, &depth_release));
      }
      g_depth_valid[e] = depth_ok;  // a colour image without matching depth is submitted without depth
    }
    {
      static ULONGLONG depth_reported = 0;
      const ULONGLONG now_ms = GetTickCount64();
      if (g_xr_depth.load(std::memory_order_relaxed) && now_ms - depth_reported >= 5000) {
        depth_reported = now_ms;
        const xr_depth::Stats s = xr_depth::stats();
        log_line("xrdepth: ext=%d chains=%ux%u%s submitted=%llu stale=%llu not_eye_mode=%llu ownqueue_skips=%llu "
                 "image_fails=%llu | captures main=%llu second=%llu skipped_layout=%llu skipped_alloc=%llu records=%llu "
                 "record_failures=%llu",
                 g_depth_ext ? 1 : 0, g_depth_w, g_depth_h, g_depth_failed ? " (failed)" : "",
                 static_cast<unsigned long long>(g_depth_submitted.load()), static_cast<unsigned long long>(g_depth_stale.load()),
                 static_cast<unsigned long long>(g_depth_not_eye_mode.load()),
                 static_cast<unsigned long long>(g_depth_pq_skips.load()),
                 static_cast<unsigned long long>(g_depth_acquire_fails.load()), static_cast<unsigned long long>(s.captures[0]),
                 static_cast<unsigned long long>(s.captures[1]), static_cast<unsigned long long>(s.skipped_layout),
                 static_cast<unsigned long long>(s.skipped_alloc), static_cast<unsigned long long>(s.records),
                 static_cast<unsigned long long>(s.record_failures));
      }
    }
    g_stage_valid = false;
    g_copies.fetch_add(1, std::memory_order_relaxed);
    signal_new_image();
    if (right != nullptr) {
      g_native_pairs.fetch_add(1, std::memory_order_relaxed);
      head_pose::Snapshot used;
      head_pose::Snapshot used_l;
      float tan4[4] = {};
      if (paired) {
        g_paired_frames.fetch_add(1, std::memory_order_relaxed);
      }
      // A held (paired) right image was rendered one StartFrame before the live one would have been.
      const int lag_r = g_pose_lag.load() + (paired ? 1 : 0);
      if (identity || (native_stereo::views_for_present(lag_r, &used) &&
                       native_stereo::views_for_present(g_pose_lag_l.load(), &used_l))) {
        if (identity) {
          g_layer_views = ident;  // both eyes: the pose this frame was rendered with
        } else {
          g_layer_views = used;
          g_layer_views.eye[0] = used_l.eye[0];  // the left image (backbuffer) was rendered from an older pose
        }
        // Labels: the per-eye computed frustum ("autofov") when both eyes carry one, else the measured game frustum.
        const bool computed = g_layer_views.eye[0].rvalid && g_layer_views.eye[1].rvalid;
        const bool measured = native_stereo::game_frustum(tan4);
        if (measured) {
          std::memcpy(g_layer_tan, tan4, sizeof(tan4));
        }
        g_layer_valid = computed || measured;
        if (g_layer_valid) {
          g_real_prev = g_real_last;
          g_real_prev_valid = g_real_last.valid;
          g_real_last = g_layer_views;
          g_real_last.valid = true;
        }
        g_real_new = true;
        g_content_generated = false;
        {
          LARGE_INTEGER now_qpc, qpf;
          QueryPerformanceCounter(&now_qpc);
          QueryPerformanceFrequency(&qpf);
          if (g_last_real_qpc.QuadPart != 0) {
            const double ns = static_cast<double>(now_qpc.QuadPart - g_last_real_qpc.QuadPart) * 1e9 / static_cast<double>(qpf.QuadPart);
            if (ns > 5e6 && ns < 6e7) g_game_frame_ns = g_game_frame_ns <= 0.0 ? ns : g_game_frame_ns * 0.9 + ns * 0.1;
          }
          g_last_real_qpc = now_qpc;
        }
      } else {
        g_layer_valid = false;
      }
    } else if (eye > 0) {
      g_copies_right.fetch_add(1, std::memory_order_relaxed);
      g_pairs.fetch_add(1, std::memory_order_relaxed);
    } else {
      g_copies_mono.fetch_add(1, std::memory_order_relaxed);
    }
  } while (false);
  if (left_held != nullptr) {
    left_held->Release();
  }
  if (right_held != nullptr) {
    // The copy was queued on the game queue; the held pool texture stays referenced by native_stereo until a newer
    // one replaces it, and the engine only reuses its memory frames later, so dropping our extra ref here is safe.
    right_held->Release();
  }
  if (device != nullptr) {
    device->Release();
  }
  if (back != nullptr) {
    back->Release();
  }
  if (sc3 != nullptr) {
    sc3->Release();
  }
  queue->Release();
}

// ---- session ---------------------------------------------------------------------------------------

// Returns when the control file leaves "session" (after a clean exit) or on a fatal error.
// Returns false on a fatal error so the worker does not retry until the mode is cycled.
bool run_session() {
  g_cin_lazy.store(false);
  g_cin_startup.store(true);
  g_cin_recentre.store(true);
  ID3D12CommandQueue* queue = acquire_queue();
  if (queue == nullptr) {
    log_line("session: waiting for the game's swapchain queue");
    return true;
  }
  ID3D12Device* device = device_of(queue);
  bool ok = false;
  XrBase base;
  XrSession session = XR_NULL_HANDLE;
  XrSpace local = XR_NULL_HANDLE;
  XrSpace view = XR_NULL_HANDLE;
  LUID required = {};
  do {
    if (device == nullptr) {
      log_line("session: queue has no device");
      break;
    }
    if (!create_instance(&base) || !base.has_d3d12 || !get_system(&base) || !check_gpu(base, device, &required)) {
      break;
    }
    {
      std::lock_guard<std::mutex> lock(g_xr);
      g_pq_on = false;
      if (g_pq == nullptr && own_queue_wanted()) {
        D3D12_COMMAND_QUEUE_DESC qd = {};
        qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        if (SUCCEEDED(device->CreateCommandQueue(&qd, IID_PPV_ARGS(&g_pq))) && g_pq != nullptr) {
          log_line("ownqueue: session bound to a private present queue %p (game queue %p no longer carries runtime waits)",
                   static_cast<void*>(g_pq), static_cast<void*>(queue));
        } else {
          g_pq = nullptr;
          log_line("ownqueue: CreateCommandQueue failed; session stays on the game queue");
        }
      }
      g_pq_on = g_pq != nullptr;
    }
    XrGraphicsBindingD3D12KHR binding = {XR_TYPE_GRAPHICS_BINDING_D3D12_KHR};
    binding.device = device;
    binding.queue = g_pq_on ? g_pq : queue;
    XrSessionCreateInfo info = {XR_TYPE_SESSION_CREATE_INFO};
    info.next = &binding;
    info.systemId = base.system;
    XrResult r = xrCreateSession(base.instance, &info, &session);
    if (XR_FAILED(r)) {
      log_line("xrCreateSession failed: %s", result_text(base.instance, r).c_str());
      session = XR_NULL_HANDLE;
      break;
    }
    XrReferenceSpaceCreateInfo space = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    space.poseInReferenceSpace.orientation.w = 1.f;
    space.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    r = xrCreateReferenceSpace(session, &space, &local);
    if (XR_FAILED(r)) {
      log_line("LOCAL space failed: %s", result_text(base.instance, r).c_str());
      break;
    }
    space.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    r = xrCreateReferenceSpace(session, &space, &view);
    if (XR_FAILED(r)) {
      log_line("VIEW space failed: %s", result_text(base.instance, r).c_str());
      break;
    }
    xr_input::create(base.instance, session, base.enabled);  // controller actions (XR_INPUT.log)
    log_line("session created on the game's device/queue; waiting for READY");
    {
      std::lock_guard<std::mutex> lock(g_xr);
      g_instance = base.instance;
      g_session = session;
      g_chain_failed = false;
    }
    ok = true;
  } while (false);

  bool fatal = !ok;
  if (ok) {
    bool running = false;
    bool exit_requested = false;
    bool done = false;
    XrSessionState state = XR_SESSION_STATE_UNKNOWN;
    ULONGLONG last_log = 0;
    uint64_t frames = 0;
    uint64_t frames_logged = 0;
    while (!done) {
      if (!is_session(static_cast<Mode>(g_want.load()))) {
        if (running && !exit_requested) {
          log_line("session: control left 'session'; requesting exit");
          xrRequestExitSession(session);
          exit_requested = true;
        } else if (!running) {
          break;
        }
      }
      XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
      while (xrPollEvent(base.instance, &event) == XR_SUCCESS) {
        if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
          const auto* changed = reinterpret_cast<const XrEventDataSessionStateChanged*>(&event);
          state = changed->state;
          log_line("session state -> %d", static_cast<int>(state));
          if (state == XR_SESSION_STATE_READY) {
            XrSessionBeginInfo begin = {XR_TYPE_SESSION_BEGIN_INFO};
            begin.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            const XrResult r = xrBeginSession(session, &begin);
            log_line("xrBeginSession: %s", result_text(base.instance, r).c_str());
            if (XR_SUCCEEDED(r)) query_vismask(base.instance, session);
            running = XR_SUCCEEDED(r);
            std::lock_guard<std::mutex> lock(g_xr);
            g_running = running;
          } else if (state == XR_SESSION_STATE_STOPPING) {
            {
              std::lock_guard<std::mutex> lock(g_xr);
              g_running = false;
              wait_copies_idle();
              destroy_chains_locked();
            }
            xrEndSession(session);
            running = false;
            log_line("session ended (STOPPING)");
            if (exit_requested) {
              done = true;
            }
          } else if (state == XR_SESSION_STATE_EXITING || state == XR_SESSION_STATE_LOSS_PENDING) {
            done = true;
          }
        } else if (event.type == XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED) {
          xr_input::on_profile_changed(session);
        } else if (event.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
          log_line("instance loss pending");
          done = true;
        }
        event = {XR_TYPE_EVENT_DATA_BUFFER};
      }
      if (done) {
        break;
      }
      if (!running) {
        Sleep(50);
        continue;
      }
      if (g_framegen.load(std::memory_order_relaxed)) {
        if (g_new_image != nullptr) WaitForSingleObject(g_new_image, 0);  // headset rate: drain, never block
      } else if (g_game_rate.load(std::memory_order_relaxed) && g_new_image != nullptr &&
          WaitForSingleObject(g_new_image, 100) == WAIT_TIMEOUT) {
        g_keepalive_submits.fetch_add(1, std::memory_order_relaxed);  // game stalled: keep the session fed
      }
      XrFrameWaitInfo wait_info = {XR_TYPE_FRAME_WAIT_INFO};
      XrFrameState frame = {XR_TYPE_FRAME_STATE};
      XrResult r = xrWaitFrame(session, &wait_info, &frame);
      if (XR_FAILED(r)) {
        log_line("xrWaitFrame: %s", result_text(base.instance, r).c_str());
        Sleep(50);
        continue;
      }
      {
        LARGE_INTEGER now, freq;
        QueryPerformanceCounter(&now);
        QueryPerformanceFrequency(&freq);
        g_xr_wait_qpc.store(now.QuadPart, std::memory_order_relaxed);
        if (frame.predictedDisplayPeriod > 0) {
          g_xr_period_qpc.store(static_cast<LONGLONG>(static_cast<double>(frame.predictedDisplayPeriod) * 1e-9 *
                                                      static_cast<double>(freq.QuadPart)),
                                std::memory_order_relaxed);
        }
      }
      XrFrameBeginInfo begin_info = {XR_TYPE_FRAME_BEGIN_INFO};
      r = xrBeginFrame(session, &begin_info);
      if (XR_FAILED(r)) {
        log_line("xrBeginFrame: %s", result_text(base.instance, r).c_str());
        continue;
      }
      ++frames;
      g_session_running.store(true, std::memory_order_relaxed);
      xr_input::sync(session);
      {
        XrViewLocateInfo locate_all = {XR_TYPE_VIEW_LOCATE_INFO};
        locate_all.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        locate_all.displayTime = frame.predictedDisplayTime;
        locate_all.space = local;
        XrViewState vs = {XR_TYPE_VIEW_STATE};
        XrView vv[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        uint32_t vc = 0;
        if (XR_SUCCEEDED(xrLocateViews(session, &locate_all, &vs, 2, &vc, vv)) && vc == 2 &&
            (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0) {
          std::lock_guard<std::mutex> pose_lock(g_pose_mutex);
          for (int e = 0; e < 2; ++e) {
            head_pose::Eye& d = g_latest.eye[e];
            d.qx = vv[e].pose.orientation.x;
            d.qy = vv[e].pose.orientation.y;
            d.qz = vv[e].pose.orientation.z;
            d.qw = vv[e].pose.orientation.w;
            d.px = vv[e].pose.position.x;
            d.py = vv[e].pose.position.y;
            d.pz = vv[e].pose.position.z;
            d.fov[0] = vv[e].fov.angleLeft;
            d.fov[1] = vv[e].fov.angleRight;
            d.fov[2] = vv[e].fov.angleUp;
            d.fov[3] = vv[e].fov.angleDown;
          }
          xr_input::locate_hands(local, frame.predictedDisplayTime, &g_latest);  // controllers, same space and time
          g_laser_aim[0] = g_latest.aim[0];
          g_laser_aim[1] = g_latest.aim[1];
          g_laser_head[0] = g_latest.eye[0].px; g_laser_head[1] = g_latest.eye[0].py; g_laser_head[2] = g_latest.eye[0].pz;
          g_latest.display_time = frame.predictedDisplayTime;
          ++g_latest.seq;
          g_latest.valid = true;
          if (g_cin_recentre.exchange(false)) {
            const bool lazy = g_cin_lazy.exchange(false);
            const bool startup = g_cin_startup.exchange(false);
            if (g_screen_follow.load(std::memory_order_relaxed)) {
              g_cin_follow = 90;  // old: follow the head for ~1.2 s (the first poses after start can be stale)
            } else {
              g_cin_follow = 0;
              g_cin_placer.request(lazy, startup);
            }
          }
          if (g_cin_follow > 0 || g_cin_placer.waiting()) {
            const XrQuaternionf& q = vv[0].pose.orientation;
            const float fwd[3] = {0.f, 0.f, -1.f};
            float f[3];
            rotate(q, fwd, f);
            const float yaw = std::atan2(-f[0], -f[2]);
            const float dist = g_screen_distance.load();
            const float hx = 0.5f * (vv[0].pose.position.x + vv[1].pose.position.x);
            const float hy = 0.5f * (vv[0].pose.position.y + vv[1].pose.position.y);
            const float hz = 0.5f * (vv[0].pose.position.z + vv[1].pose.position.z);
            bool place = false;
            if (g_cin_follow > 0) {
              --g_cin_follow;
              place = true;
            } else {
              // Placed once, behind the black layer, when the head is steady (or kept where it is while still in view).
              const float vx = g_cin_pose.position.x - hx, vz = g_cin_pose.position.z - hz;
              const bool tracked = (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_TRACKED_BIT) != 0;
              const menu_transition::Step step = g_cin_placer.step(
                  yaw * 57.2958f, tracked, g_cin_has_pose, std::atan2(-vx, -vz) * 57.2958f, std::sqrt(vx * vx + vz * vz),
                  g_cin_pose.position.y - hy, dist);
              if (step == menu_transition::Step::Place) {
                place = true;
              } else if (step == menu_transition::Step::Keep) {
                log_line("cinema: screen kept where it was (%.0f deg off the gaze)",
                         std::fabs(menu_transition::wrap_deg(std::atan2(-vx, -vz) * 57.2958f - yaw * 57.2958f)));
              }
            }
            if (place) {
              g_cin_pose.orientation = {0.f, std::sin(yaw * 0.5f), 0.f, std::cos(yaw * 0.5f)};
              g_cin_pose.position = {hx - std::sin(yaw) * dist, hy, hz - std::cos(yaw) * dist};
              g_cin_has_pose = true;
            }
            if (place && g_cin_follow == 0) {
              log_line("cinema: screen placed yaw=%.1f deg distance=%.2f m at (%.2f,%.2f,%.2f) %s %d frames", yaw * 57.2958f,
                       dist, g_cin_pose.position.x, g_cin_pose.position.y, g_cin_pose.position.z,
                       g_screen_follow.load(std::memory_order_relaxed) ? "after following for" : "once, waited",
                       g_screen_follow.load(std::memory_order_relaxed) ? 90 : g_cin_placer.frames());
            }
          }
          if (!g_gui_visible.load(std::memory_order_relaxed)) {
            g_gui_placed = false;  // placed again the next time the game UI panel shows
          } else if (!g_gui_placed || g_gui_recentre.exchange(false)) {
            const XrQuaternionf& q = vv[0].pose.orientation;
            const float fwd[3] = {0.f, 0.f, -1.f};
            float f[3];
            rotate(q, fwd, f);
            const float yaw = std::atan2(-f[0], -f[2]);
            const float dist = g_gui_distance.load();
            const float hx = 0.5f * (vv[0].pose.position.x + vv[1].pose.position.x);
            const float hy = 0.5f * (vv[0].pose.position.y + vv[1].pose.position.y);
            const float hz = 0.5f * (vv[0].pose.position.z + vv[1].pose.position.z);
            g_gui_pose.orientation = {0.f, std::sin(yaw * 0.5f), 0.f, std::cos(yaw * 0.5f)};
            g_gui_pose.position = {hx - std::sin(yaw) * dist, hy + g_gui_height_m.load(), hz - std::cos(yaw) * dist};
            g_gui_placed = true;
            log_line("gui: game UI panel placed yaw=%.1f deg distance=%.2f m height=%+.2f m", yaw * 57.2958f, dist,
                     g_gui_height_m.load());
          }
          if (!g_ovl_visible.load(std::memory_order_relaxed)) {
            g_ovl_placed = false;  // place it again the next time the menu opens
          } else if (!g_ovl_placed || g_ovl_recentre.exchange(false)) {
            // Yaw-only, at head height minus a little, `distance` metres along the gaze; faces the head (+z normal).
            const XrQuaternionf& q = vv[0].pose.orientation;
            const float fwd[3] = {0.f, 0.f, -1.f};
            float f[3];
            rotate(q, fwd, f);
            const float yaw = std::atan2(-f[0], -f[2]);
            const float dist = g_ovl_distance.load();
            const float hx = 0.5f * (vv[0].pose.position.x + vv[1].pose.position.x);
            const float hy = 0.5f * (vv[0].pose.position.y + vv[1].pose.position.y);
            const float hz = 0.5f * (vv[0].pose.position.z + vv[1].pose.position.z);
            g_ovl_pose.orientation = {0.f, std::sin(yaw * 0.5f), 0.f, std::cos(yaw * 0.5f)};
            g_ovl_pose.position = {hx - std::sin(yaw) * dist, hy - 0.08f, hz - std::cos(yaw) * dist};
            g_ovl_placed = true;
            log_line("menu: panel placed yaw=%.1f deg distance=%.2f m", yaw * 57.2958f, dist);
          }
        }
      }
      const ULONGLONG now = GetTickCount64();
      if (now - last_log >= 1000) {
        XrSpaceLocation head = {XR_TYPE_SPACE_LOCATION};
        xrLocateSpace(view, local, frame.predictedDisplayTime, &head);
        XrViewLocateInfo locate = {XR_TYPE_VIEW_LOCATE_INFO};
        locate.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        locate.displayTime = frame.predictedDisplayTime;
        locate.space = local;
        XrViewState view_state = {XR_TYPE_VIEW_STATE};
        XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        uint32_t view_count = 0;
        xrLocateViews(session, &locate, &view_state, 2, &view_count, views);
        float yaw = 0.f, pitch = 0.f;
        yaw_pitch(head.pose.orientation, &yaw, &pitch);
        const auto& p = head.pose.position;
        float ipd = 0.f;
        if (view_count == 2) {
          const float dx = views[1].pose.position.x - views[0].pose.position.x;
          const float dy = views[1].pose.position.y - views[0].pose.position.y;
          const float dz = views[1].pose.position.z - views[0].pose.position.z;
          ipd = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        const float rad = 57.2957795f;
        const double fps = last_log == 0 ? 0.0 : static_cast<double>(frames - frames_logged) * 1000.0 / static_cast<double>(now - last_log);
        log_line("pose flags=0x%llX pos=(%.3f,%.3f,%.3f) yaw=%.1f pitch=%.1f q=(%.3f,%.3f,%.3f,%.3f) ipd=%.4f "
                 "fovL=(%.1f,%.1f,%.1f,%.1f) fovR=(%.1f,%.1f,%.1f,%.1f) period=%.2fms xr_fps=%.1f shouldRender=%u "
                 "copies=%llu (L=%llu R=%llu mono=%llu) pairs=%llu orphans=%llu parity_mismatch=%llu layers=%llu "
                 "native=%d native_pairs=%llu missing=%llu size_mismatch=%llu proj=%d proj_layers=%llu no_pose=%llu lag=%d "
                 "tan=(%.3f,%.3f,%.3f,%.3f) pair=%d paired=%llu pace=%s wait=%.2fms/frame submit=%s keepalive=%llu",
                 static_cast<unsigned long long>(head.locationFlags), p.x, p.y, p.z, yaw, pitch,
                 head.pose.orientation.x, head.pose.orientation.y, head.pose.orientation.z, head.pose.orientation.w, ipd,
                 views[0].fov.angleLeft * rad, views[0].fov.angleRight * rad, views[0].fov.angleUp * rad, views[0].fov.angleDown * rad,
                 views[1].fov.angleLeft * rad, views[1].fov.angleRight * rad, views[1].fov.angleUp * rad, views[1].fov.angleDown * rad,
                 static_cast<double>(frame.predictedDisplayPeriod) / 1.0e6, fps, static_cast<unsigned>(frame.shouldRender),
                 static_cast<unsigned long long>(g_copies.load()), static_cast<unsigned long long>(g_copies_left.load()),
                 static_cast<unsigned long long>(g_copies_right.load()), static_cast<unsigned long long>(g_copies_mono.load()),
                 static_cast<unsigned long long>(g_pairs.load()), static_cast<unsigned long long>(g_orphans.load()),
                 static_cast<unsigned long long>(g_parity_mismatch.load()), static_cast<unsigned long long>(g_layers_submitted.load()),
                 g_native.load() ? 1 : 0, static_cast<unsigned long long>(g_native_pairs.load()),
                 static_cast<unsigned long long>(g_native_missing.load()), static_cast<unsigned long long>(g_native_mismatch.load()),
                 g_proj.load() ? 1 : 0, static_cast<unsigned long long>(g_proj_layers.load()),
                 static_cast<unsigned long long>(g_proj_no_pose.load()), g_pose_lag.load() + 100 * g_pose_lag_l.load(), g_layer_tan[0], g_layer_tan[1],
                 g_layer_tan[2], g_layer_tan[3], g_pair.load() ? 1 : 0, static_cast<unsigned long long>(g_paired_frames.load()),
                 g_half_rate.load() ? "half" : "free",
                 g_pace_frames.load() ? static_cast<double>(g_pace_wait_us.load()) * 1e-3 / static_cast<double>(g_pace_frames.load()) : 0.0,
                 g_game_rate.load() ? "game" : "repeat", static_cast<unsigned long long>(g_keepalive_submits.load()));
        log_line("mirror: refresh %u Hz, staging copies %llu, direct sampled frames %llu (desktop draws every frame)", g_mirror_refresh.load(),
                 static_cast<unsigned long long>(g_mirror_copies.load()), static_cast<unsigned long long>(g_mirror_direct.load()));
        log_line("eye mode: presents %llu, left image too small %llu, mirror draws %llu, chains %ux%u",
                 static_cast<unsigned long long>(g_eye_mode_presents.load()), static_cast<unsigned long long>(g_eye_mode_mismatch.load()),
                 static_cast<unsigned long long>(g_mirror_draws.load()), g_chain_width, g_chain_height);
        log_line("wrist hud: on=%d texture copies=%llu wrist frames=%llu on drawn arm=%llu", wrist_hud::current().on ? 1 : 0,
                 static_cast<unsigned long long>(g_hud_copies.load()), static_cast<unsigned long long>(g_hud_layers.load()),
                 static_cast<unsigned long long>(g_hud_arm_frames.load()));
        last_log = now;
        frames_logged = frames;
      }
      XrFrameEndInfo end_info = {XR_TYPE_FRAME_END_INFO};
      end_info.displayTime = frame.predictedDisplayTime;
      end_info.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
      end_info.layerCount = 0;
      XrCompositionLayerQuad quads[2] = {{XR_TYPE_COMPOSITION_LAYER_QUAD}, {XR_TYPE_COMPOSITION_LAYER_QUAD}};
      XrCompositionLayerProjection proj = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
      XrCompositionLayerProjectionView pviews[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
                                                    {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
      XrCompositionLayerDepthInfoKHR depth_info[2] = {{XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR},
                                                      {XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR}};
      const XrCompositionLayerBaseHeader* layers[12] = {};  // eyes 2 + panel + laser 2 + wrist 3 + fade + menu
      XrCompositionLayerQuad hud_quads[3] = {{XR_TYPE_COMPOSITION_LAYER_QUAD}, {XR_TYPE_COMPOSITION_LAYER_QUAD}, {XR_TYPE_COMPOSITION_LAYER_QUAD}};
      bool wrist_in_frame = false;
      XrCompositionLayerQuad menu_quad = {XR_TYPE_COMPOSITION_LAYER_QUAD};
      XrCompositionLayerQuad gui_quad = {XR_TYPE_COMPOSITION_LAYER_QUAD};
      std::unique_lock<std::mutex> lock(g_xr);
      const bool cinema_ready = static_cast<Mode>(g_want.load()) == Mode::Cinema && frame.shouldRender && g_chain_width > 0 &&
                                g_eyes[0].has_image && g_eyes[1].has_image;
      // In-mod frame generation: a refresh without a new game image gets the last real one warped forward.
      if (g_framegen.load(std::memory_order_relaxed) && cinema_ready && g_proj.load() && g_layer_valid) {
        if (g_real_new) {
          g_real_disp_prev = g_real_disp_last;
          g_real_disp_last = frame.predictedDisplayTime;
          g_real_new = false;
          g_fg_real.fetch_add(1, std::memory_order_relaxed);
        } else {
          const int64_t interval = static_cast<int64_t>(g_game_frame_ns);  // the game's frame time (smoothed)
          const int64_t since = frame.predictedDisplayTime - g_real_disp_last;
          const float k = frame_gen_math::extrapolation_k(frame.predictedDisplayTime, g_real_disp_last, interval, 1.0f);
          if (!g_real_kept || !g_real_prev_valid || g_real_disp_prev == 0 || k <= 0.f) {
            g_fg_skip_not_ready.fetch_add(1, std::memory_order_relaxed);
          } else if (since > 2 * interval) {
            g_fg_skip_old.fetch_add(1, std::memory_order_relaxed);  // the game stalled: keep showing what is there
          } else if (g_copy_device == nullptr || g_eyes[0].images.empty() ||
                     !frame_gen::can_generate(g_copy_device, g_eyes[0].images[0]->GetDesc().Format, g_chain_width, g_chain_height)) {
            g_fg_skip_not_ready.fetch_add(1, std::memory_order_relaxed);
          } else {
            uint32_t idx[2] = {};
            bool got[2] = {false, false};
            for (int e = 0; e < 2; ++e) {
              XrSwapchainImageAcquireInfo ai = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
              if (XR_FAILED(xrAcquireSwapchainImage(g_eyes[e].chain, &ai, &idx[e]))) continue;
              XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
              wi.timeout = 20000000;  // 20 ms
              const XrResult wr = xrWaitSwapchainImage(g_eyes[e].chain, &wi);
              got[e] = XR_SUCCEEDED(wr) && wr != XR_TIMEOUT_EXPIRED;
            }
            bool generated = false;
            if (got[0] && got[1]) {
              ID3D12Resource* targets[2] = {g_eyes[0].images[idx[0]], g_eyes[1].images[idx[1]]};
              generated = frame_gen::generate(g_copy_device, targets, g_chain_width, g_chain_height,
                                              g_framegen_k0.load(std::memory_order_relaxed) ? 0.f : k,
                                              g_sharpness.load(std::memory_order_relaxed));
              // The session is bound to `queue` (framegen is off with ownqueue): it must wait for the warp before the
              // runtime may read the released images.
              if (generated) frame_gen::sync_to(queue);
            }
            for (int e = 0; e < 2; ++e) {
              if (!got[e]) continue;
              XrSwapchainImageReleaseInfo ri = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
              xrReleaseSwapchainImage(g_eyes[e].chain, &ri);
            }
            if (generated) {
              g_gen_views = g_real_last;
              for (int e = 0; e < 2 && !g_framegen_k0.load(std::memory_order_relaxed); ++e) {
                const head_pose::Eye& a = g_real_prev.eye[e];
                const head_pose::Eye& b = g_real_last.eye[e];
                const frame_gen_math::Pose x = frame_gen_math::extrapolate({a.qx, a.qy, a.qz, a.qw, a.px, a.py, a.pz},
                                                                           {b.qx, b.qy, b.qz, b.qw, b.px, b.py, b.pz}, k);
                head_pose::Eye& o = g_gen_views.eye[e];
                o.qx = x.qx; o.qy = x.qy; o.qz = x.qz; o.qw = x.qw;
                o.px = x.px; o.py = x.py; o.pz = x.pz;
              }
              g_content_generated = true;
              g_fg_generated.fetch_add(1, std::memory_order_relaxed);
            } else {
              g_fg_skip_acquire.fetch_add(1, std::memory_order_relaxed);
            }
          }
        }
      }
      if (cinema_ready && g_proj.load() && !g_layer_valid) {
        g_proj_no_pose.fetch_add(1, std::memory_order_relaxed);
      }
      if (cinema_ready && g_proj.load() && g_layer_valid) {
        // Head-tracked: each eye image is shown where it was rendered from (its LOCAL-space eye pose) with the
        // frustum the game actually rendered; the runtime reprojects to the current head pose.
        for (int e = 0; e < 2; ++e) {
          const head_pose::Eye& ev = (g_content_generated ? g_gen_views : g_layer_views).eye[e];
          XrCompositionLayerProjectionView& v = pviews[e];
          v.pose.orientation = {ev.qx, ev.qy, ev.qz, ev.qw};
          v.pose.position = {ev.px, ev.py, ev.pz};
          const float* t = ev.rvalid ? ev.rtan : g_layer_tan;  // per-eye rendered frustum when native_stereo set one
          static ULONGLONG vismask_logged[2] = {0, 0};
          if (GetTickCount64() - vismask_logged[e] > 10000 && !g_vismask[e].i.empty()) {
            vismask_logged[e] = GetTickCount64();
            const double share = vismask_share(e, t);
            if (share >= 0.0) {
              log_line("vismask: eye %d rendered frustum tan l %.3f r %.3f u %.3f d %.3f, hidden by the lens mask %.1f%%", e, t[0], t[1],
                       t[2], t[3], share);
            } else log_line("vismask: eye %d hidden-area coverage unavailable", e);
          }
          v.fov.angleLeft = std::atan(t[0]);
          v.fov.angleRight = std::atan(t[1]);
          v.fov.angleUp = std::atan(t[2]);
          v.fov.angleDown = std::atan(t[3]);
          v.subImage.swapchain = g_eyes[e].chain;
          v.subImage.imageRect.offset = {0, 0};
          v.subImage.imageRect.extent = {static_cast<int32_t>(g_chain_width), static_cast<int32_t>(g_chain_height)};
          v.subImage.imageArrayIndex = 0;
          // Optional depth ("xrdepth"): only when this eye's latest colour image came with a released depth image.
          // Not while frame generation runs: depth on some frames only (generated ones have none, and depth is missed
          // when a capture is stale) made Virtual Desktop switch reprojection per frame (AUTHOR 2026-11-12: blinking).
          if (g_xr_depth.load(std::memory_order_relaxed) && !g_framegen.load(std::memory_order_relaxed) && g_depth_valid[e] && g_depth_chains[e].chain != XR_NULL_HANDLE &&
              g_depth_w == g_chain_width && g_depth_h == g_chain_height) {
            const xr_depth_math::DepthInfo di = xr_depth_math::reverse_z_info();
            XrCompositionLayerDepthInfoKHR& d = depth_info[e];
            d.subImage.swapchain = g_depth_chains[e].chain;
            d.subImage.imageRect = v.subImage.imageRect;
            d.subImage.imageArrayIndex = 0;
            d.minDepth = di.min_depth;
            d.maxDepth = di.max_depth;
            d.nearZ = di.near_z;
            d.farZ = di.far_z;
            v.next = &d;
            if (e == 1) g_depth_submitted.fetch_add(1, std::memory_order_relaxed);
          }
        }
        proj.space = local;
        proj.viewCount = 2;
        proj.views = pviews;
        layers[0] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&proj);
        end_info.layerCount = 1;
        end_info.layers = layers;
        g_layers_submitted.fetch_add(1, std::memory_order_relaxed);
        g_proj_layers.fetch_add(1, std::memory_order_relaxed);
      } else if (cinema_ready) {
        // A flat 3D screen in front of the session's starting head pose: the left eye sees only the left
        // swapchain, the right eye only the right one. Also the fallback for "proj" while no head-tracked stereo
        // pair is available (mono start-up screens and game menus): both swapchains then hold the same image.
        const float width = g_screen_width.load();
        const float height = width * static_cast<float>(g_chain_height) / static_cast<float>(g_chain_width);
        for (int e = 0; e < 2; ++e) {
          XrCompositionLayerQuad& q = quads[e];
          q.space = local;
          q.eyeVisibility = e == 0 ? XR_EYE_VISIBILITY_LEFT : XR_EYE_VISIBILITY_RIGHT;
          q.subImage.swapchain = g_eyes[e].chain;
          q.subImage.imageRect.offset = {0, 0};
          q.subImage.imageRect.extent = {static_cast<int32_t>(g_chain_width), static_cast<int32_t>(g_chain_height)};
          q.subImage.imageArrayIndex = 0;
          q.pose = g_cin_pose;
          q.size = {width, height};
          layers[e] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&q);
        }
        end_info.layerCount = 2;
        end_info.layers = layers;
        g_layers_submitted.fetch_add(1, std::memory_order_relaxed);
      }
      // What the eye layers show this frame, for the black transition layer (added below, under the mod menu).
      const menu_transition::Kind fade_kind = !cinema_ready || end_info.layerCount == 0 ? menu_transition::Kind::None
                                              : g_proj.load() && g_layer_valid             ? menu_transition::Kind::Stereo
                                                                                           : menu_transition::Kind::Flat;
      if (frame.shouldRender && end_info.layerCount > 0 && g_gui_visible.load(std::memory_order_relaxed) && g_gui_placed &&
          g_gui_has_image && g_gui_chain != XR_NULL_HANDLE && g_gui_width > 0) {
        // Game UI panel: the main menu's UI, premultiplied by the game's own blending onto a transparent target.
        const float w = g_gui_width_m.load();
        gui_quad.space = local;
        gui_quad.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        gui_quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        gui_quad.subImage.swapchain = g_gui_chain;
        gui_quad.subImage.imageRect.offset = {0, 0};
        gui_quad.subImage.imageRect.extent = {static_cast<int32_t>(g_gui_width), static_cast<int32_t>(g_gui_height)};
        gui_quad.subImage.imageArrayIndex = 0;
        gui_quad.pose = g_gui_pose;
        gui_quad.size = {w, w * static_cast<float>(g_gui_height) / static_cast<float>(g_gui_width)};
        layers[end_info.layerCount] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&gui_quad);
        end_info.layerCount += 1;
        end_info.layers = layers;
        g_gui_layers.fetch_add(1, std::memory_order_relaxed);
      }
      // Menu laser: the panel showing the game's menu is the UI quad on the main menu, the mono screen in the paused game's
      // menus. Only while the game's own virtual cursor runs, so loading screens and gameplay get nothing.
      XrCompositionLayerQuad laser_quads[2] = {{XR_TYPE_COMPOSITION_LAYER_QUAD}, {XR_TYPE_COMPOSITION_LAYER_QUAD}};
      {
        const XrCompositionLayerQuad* panel = gui_quad.subImage.swapchain != XR_NULL_HANDLE ? &gui_quad
            : quads[0].subImage.swapchain != XR_NULL_HANDLE && !native_stereo::stereo_running() ? &quads[0] : nullptr;
        const ULONGLONG now = GetTickCount64();
        bool hit[2] = {false, false};
        float u[2] = {}, v[2] = {}, dist[2] = {}, origin[2][3] = {}, dir[2][3] = {};
        const bool live = panel != nullptr && frame.shouldRender && menu_pointer::g_enabled.load(std::memory_order_relaxed) &&
                          !g_ovl_visible.load(std::memory_order_relaxed) &&
                          now - menu_pointer::g_native_cursor_at.load(std::memory_order_relaxed) < 250;
        if (live) {
          const float q[4] = {panel->pose.orientation.x, panel->pose.orientation.y, panel->pose.orientation.z, panel->pose.orientation.w};
          const float p[3] = {panel->pose.position.x, panel->pose.position.y, panel->pose.position.z};
          const float forward[3] = {0.f, 0.f, -1.f};
          for (int h = 0; h < 2; ++h) {
            const head_pose::Hand& a = g_laser_aim[h];
            if (!a.valid) continue;
            const float aq[4] = {a.qx, a.qy, a.qz, a.qw};
            origin[h][0] = a.px; origin[h][1] = a.py; origin[h][2] = a.pz;
            menu_pointer::rotate(aq, forward, dir[h]);
            hit[h] = menu_pointer::hit_quad(origin[h], dir[h], q, p, panel->size.width, panel->size.height, &u[h], &v[h], &dist[h]);
          }
        }
        // Nothing hoverable lately (a loading screen): no beam or trigger, but the cursor keeps following to find a widget.
        const bool shown = live && menu_pointer::interactive(now);
        static bool logged_live = false, logged_shown = false;
        if (live != logged_live || shown != logged_shown) {
          log_line("laser: game cursor mode %d, hoverable widget lately %d, stereo %d", live ? 1 : 0, shown ? 1 : 0,
                   native_stereo::stereo_running() ? 1 : 0);
          logged_live = live;
          logged_shown = shown;
        }
        const bool usable[2] = {shown && hit[0], shown && hit[1]};
        xr_input::PadState raw;
        const float trigger[2] = {live && xr_input::controls(&raw) ? raw.lt : 0.f, live ? raw.rt : 0.f};
        const menu_pointer::Selection sel = menu_pointer::step(g_laser_select, usable, trigger);
        if (sel.press) {
          xr_input::pulse(sel.hand, 0.02f, 0.3f);
          menu_pointer::g_hover_at.store(0, std::memory_order_relaxed);
        }
        const int follow = hit[sel.hand] ? sel.hand : 1 - sel.hand;
        menu_pointer::publish(live && hit[follow], u[follow], v[follow], live && sel.a, live && sel.suppress, now);
        if (shown && g_laser_aim[sel.hand].valid && g_laser_ready && g_laser_chain != XR_NULL_HANDLE &&
            end_info.layerCount + 2u <= 8u && (g_hud_max_layers == 0 || end_info.layerCount + 2u <= g_hud_max_layers)) {
          const float* o = origin[sel.hand];
          const float* d = dir[sel.hand];
          const float length = sel.hit ? dist[sel.hand] : 1.5f;
          // The beam: a quad along the ray, turned to face the eyes (its row of sprite gives a soft-edged line).
          const float to_head[3] = {g_laser_head[0] - o[0], g_laser_head[1] - o[1], g_laser_head[2] - o[2]};
          float x[3] = {d[1] * to_head[2] - d[2] * to_head[1], d[2] * to_head[0] - d[0] * to_head[2], d[0] * to_head[1] - d[1] * to_head[0]};
          const float xl = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
          if (xl > 1e-4f) {
            x[0] /= xl; x[1] /= xl; x[2] /= xl;
            const float z[3] = {x[1] * d[2] - x[2] * d[1], x[2] * d[0] - x[0] * d[2], x[0] * d[1] - x[1] * d[0]};
            // Rotation with columns x, d, z -> quaternion.
            const float m00 = x[0], m11 = d[1], m22 = z[2];
            float qq[4];
            const float tr = m00 + m11 + m22;
            if (tr > 0.f) {
              const float s = std::sqrt(tr + 1.f) * 2.f;
              qq[3] = 0.25f * s; qq[0] = (d[2] - z[1]) / s; qq[1] = (z[0] - x[2]) / s; qq[2] = (x[1] - d[0]) / s;
            } else if (m00 > m11 && m00 > m22) {
              const float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
              qq[3] = (d[2] - z[1]) / s; qq[0] = 0.25f * s; qq[1] = (d[0] + x[1]) / s; qq[2] = (z[0] + x[2]) / s;
            } else if (m11 > m22) {
              const float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
              qq[3] = (z[0] - x[2]) / s; qq[0] = (d[0] + x[1]) / s; qq[1] = 0.25f * s; qq[2] = (z[1] + d[2]) / s;
            } else {
              const float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
              qq[3] = (x[1] - d[0]) / s; qq[0] = (z[0] + x[2]) / s; qq[1] = (z[1] + d[2]) / s; qq[2] = 0.25f * s;
            }
            XrCompositionLayerQuad& beam = laser_quads[0];
            beam.space = local;
            beam.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
            beam.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
            beam.subImage.swapchain = g_laser_chain;
            beam.subImage.imageRect.offset = {0, static_cast<int32_t>(kLaserTexels / 2)};
            beam.subImage.imageRect.extent = {static_cast<int32_t>(kLaserTexels), 1};
            beam.pose.orientation = {qq[0], qq[1], qq[2], qq[3]};
            beam.pose.position = {o[0] + d[0] * length * 0.5f, o[1] + d[1] * length * 0.5f, o[2] + d[2] * length * 0.5f};
            beam.size = {0.004f, length};
            layers[end_info.layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&beam);
          }
          if (sel.hit) {
            // The dot, on the panel, keeping about the same apparent size at any distance.
            XrCompositionLayerQuad& dot = laser_quads[1];
            dot.space = local;
            dot.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
            dot.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
            dot.subImage.swapchain = g_laser_chain;
            dot.subImage.imageRect.extent = {static_cast<int32_t>(kLaserTexels), static_cast<int32_t>(kLaserTexels)};
            dot.pose.orientation = panel->pose.orientation;
            const float l = dist[sel.hand];
            dot.pose.position = {o[0] + d[0] * l, o[1] + d[1] * l, o[2] + d[2] * l};
            const float size = 0.012f * l;
            dot.size = {size, size};
            layers[end_info.layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&dot);
          }
          end_info.layers = layers;
        }
      }
      const wrist_hud::Settings wrist = wrist_hud::current();
      if (cinema_ready && g_proj.load() && native_stereo::stereo_running() && !native_stereo::ui_layer_active() &&
          end_info.layerCount > 0 && wrist.on && g_hud_has_image && g_hud_chain != XR_NULL_HANDLE &&
          end_info.layerCount + (wrist.full ? 3u : 2u) + 1u <= g_hud_max_layers &&
          g_hud_width > 0 && GetTickCount64() - g_hud_copied_at.load(std::memory_order_relaxed) < 300) {
        // Wrist HUD: crops of the HUD texture on the forearm of the hand without the weapon, same display time.
        head_pose::Hand grip;
        head_pose::Eye viewer;
        {
          std::lock_guard<std::mutex> pose_lock(g_pose_mutex);
          grip = g_latest.grip[1 - skin_palette_probe::weapon_hand_index()];
          viewer = g_latest.eye[0];
        }
        // The anchor is relative to a grip sample, so apply it to the grip of the images being shown: the arm in them.
        const int free_hand = 1 - skin_palette_probe::weapon_hand_index();
        float arm_wrist[3], arm_elbow[3];
        bool anchored = wrist.arm_anchor && skin_palette_probe::hud_arm_anchor(free_hand, arm_wrist, arm_elbow);
        if (anchored) {
          head_pose::Snapshot shown;
          if (native_stereo::presented_views(&shown) && shown.grip[free_hand].valid) grip = shown.grip[free_hand];
          anchored = grip.valid;
        }
        if (grip.valid) {
          if (anchored) g_hud_arm_frames.fetch_add(1, std::memory_order_relaxed);
          const float W = static_cast<float>(g_hud_width), H = static_cast<float>(g_hud_height);
          auto add_quad = [&](XrCompositionLayerQuad& q, const wrist_hud::Crop& c, float width_m, const XrVector3f& off) {
            q.space = local;
            q.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;  // premultiplied by the game's blending
            q.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
            q.subImage.swapchain = g_hud_chain;
            q.subImage.imageRect.offset = {static_cast<int32_t>(c.x * W), static_cast<int32_t>(c.y * H)};
            q.subImage.imageRect.extent = {std::max(1, static_cast<int32_t>(c.w * W)), std::max(1, static_cast<int32_t>(c.h * H))};
            q.subImage.imageArrayIndex = 0;
            q.pose = anchored ? arm_pose(grip, arm_wrist, arm_elbow, off.z, 0.02f, viewer) : wrist_pose(grip, off);
            if (wrist.face_viewer) q.pose.orientation = {viewer.qx, viewer.qy, viewer.qz, viewer.qw};
            q.size = {width_m, width_m * (c.h * H) / (c.w * W)};
            layers[end_info.layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&q);
          };
          // Both crops at the same pixel scale: metres per unit of texture width.
          const float per_unit = wrist.size_m / wrist.health.w;
          const float health_h = per_unit * wrist.health.h * H / W, ammo_h = per_unit * wrist.ammo.h * H / W;
          const float z0 = 0.07f;  // health panel centre, toward the elbow from the palm
          add_quad(hud_quads[0], wrist.health, wrist.size_m, {0.f, 0.035f, z0});
          add_quad(hud_quads[1], wrist.ammo, per_unit * wrist.ammo.w, {0.f, 0.035f, z0 + 0.5f * (health_h + ammo_h) + 0.01f});
          if (wrist.full) {
            add_quad(hud_quads[2], {0.f, 0.f, 1.f, 1.f}, 0.32f, {0.f, 0.16f, 0.10f});
            // Diagnostic: opaque full RGB makes a zero-alpha HUD visible; face the viewer regardless of wrist angle.
            hud_quads[2].layerFlags = 0;
            hud_quads[2].pose.orientation = {viewer.qx, viewer.qy, viewer.qz, viewer.qw};
            hud_quads[2].pose.position = {grip.px, grip.py + 0.14f, grip.pz};
          }
          end_info.layers = layers;
          g_hud_layers.fetch_add(1, std::memory_order_relaxed);
          wrist_in_frame = true;
        }
      }
      // Black transition layer (menu_transition.hpp): a head-locked quad of premultiplied black in front of everything but
      // the mod menu. Cuts to black on every flat <-> stereo switch, stays black while the flat screen waits for its
      // placement or stereo is about to start, then fades in.
      XrCompositionLayerQuad fade_quad = {XR_TYPE_COMPOSITION_LAYER_QUAD};
      {
        static const char* const kKindName[] = {"none", "flat screen", "stereo"};
        LARGE_INTEGER qpc, qpf;
        QueryPerformanceCounter(&qpc);
        QueryPerformanceFrequency(&qpf);
        const double t = static_cast<double>(qpc.QuadPart) / static_cast<double>(qpf.QuadPart);
        menu_transition::FadeInput in;
        in.kind = fade_kind;
        in.screen_placed = !g_cin_placer.waiting();
        in.stereo_pending = native_stereo::stereo_pending();
        in.menu_pause = native_stereo::stereo_paused_by_menu();
        const menu_transition::Kind before = g_fade.kind();
        const float alpha = g_fade.update(in, t);
        static double began = 0.0, black_end = 0.0;  // this transition's start, and when its black hold ended
        static bool was_clear = false, prev_black = true;
        if (g_fade.kind_changed()) {
          began = black_end = t;
          was_clear = false;
          if (fade_kind != menu_transition::Kind::None) {
            log_line("transition: %s -> %s, black%s", kKindName[static_cast<int>(before)], kKindName[static_cast<int>(fade_kind)],
                     g_fade_on.load(std::memory_order_relaxed) ? "" : " (fades off: hard switch)");
          }
        }
        if (prev_black && !g_fade.black_wanted()) {
          black_end = t;
        }
        prev_black = g_fade.black_wanted();
        if (alpha <= 0.f && !was_clear && fade_kind != menu_transition::Kind::None) {
          was_clear = true;
          log_line("transition: %s clear after %.0f ms (black %.0f ms)", kKindName[static_cast<int>(fade_kind)],
                   (t - began) * 1000.0, (black_end - began) * 1000.0);
        } else if (alpha > 0.f && was_clear) {
          was_clear = false;  // fading out without a switch: stereo announced on the flat screen
          began = black_end = t;
          log_line("transition: %s fades to black (stereo about to start)", kKindName[static_cast<int>(fade_kind)]);
        }
        if (frame.shouldRender && end_info.layerCount > 0 && alpha > 0.f && g_fade_on.load(std::memory_order_relaxed) &&
            g_fade_ready && g_fade_chain != XR_NULL_HANDLE && fade_kind != menu_transition::Kind::None) {
          const int level = menu_transition::level_for(alpha);
          fade_quad.space = view;
          fade_quad.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;  // premultiplied black
          fade_quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
          fade_quad.subImage.swapchain = g_fade_chain;
          fade_quad.subImage.imageRect.offset = {level * menu_transition::kLevelTexels + 1, 1};
          fade_quad.subImage.imageRect.extent = {menu_transition::kLevelTexels - 2, menu_transition::kLevelTexels - 2};
          fade_quad.subImage.imageArrayIndex = 0;
          fade_quad.pose.orientation = {0.f, 0.f, 0.f, 1.f};
          fade_quad.pose.position = {0.f, 0.f, -0.15f};
          fade_quad.size = {3.f, 3.f};  // +-84 deg at 0.15 m: past any headset's field of view
          layers[end_info.layerCount] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&fade_quad);
          end_info.layerCount += 1;
          end_info.layers = layers;
        }
      }
      if (frame.shouldRender && g_ovl_visible.load(std::memory_order_relaxed) && g_ovl_placed && g_ovl_has_image &&
          g_ovl_chain != XR_NULL_HANDLE) {
        const float w = g_ovl_width_m.load();
        menu_quad.space = local;
        menu_quad.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;  // premultiplied canvas
        menu_quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        menu_quad.subImage.swapchain = g_ovl_chain;
        menu_quad.subImage.imageRect.offset = {0, 0};
        menu_quad.subImage.imageRect.extent = {static_cast<int32_t>(g_ovl_width), static_cast<int32_t>(g_ovl_height)};
        menu_quad.subImage.imageArrayIndex = 0;
        menu_quad.pose = g_ovl_pose;
        menu_quad.size = {w, w * static_cast<float>(g_ovl_height) / static_cast<float>(g_ovl_width)};
        layers[end_info.layerCount] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&menu_quad);
        end_info.layerCount += 1;
        end_info.layers = layers;
      }
      // Hand over to g_chains before xrEndFrame: the runtime may wait there for our copies, which sit behind the game's
      // GPU work. Holding g_xr through that stalled copy_present (the game's Present thread) and dropped XR frames
      // (run 2026-10-05 12:28: Present-thread "xr" stage 10 ms avg / 33 ms max, xr_fps 37-45).
      std::unique_lock<std::recursive_mutex> chains(g_chains);
      lock.unlock();
      r = xrEndFrame(session, &end_info);
      wrist_hud::submitted(XR_SUCCEEDED(r) && wrist_in_frame);
      chains.unlock();
      if (XR_FAILED(r)) {
        log_line("xrEndFrame: %s", result_text(base.instance, r).c_str());
      }
    }
    log_line("session loop finished (frames=%llu)", static_cast<unsigned long long>(frames));
  }

  {
    std::lock_guard<std::mutex> lock(g_xr);
    g_running = false;
    g_session_running.store(false);
    g_ovl_placed = false;
    wait_pq_idle();       // the private queue may still be writing swapchain images
    destroy_chains_locked();
    release_pq_locked();  // private-queue objects; the queue itself is released after the session
    release_copy_objects();
    g_session = XR_NULL_HANDLE;
    g_instance = XR_NULL_HANDLE;
  }
  xr_input::destroy();
  if (view != XR_NULL_HANDLE) {
    xrDestroySpace(view);
  }
  if (local != XR_NULL_HANDLE) {
    xrDestroySpace(local);
  }
  if (session != XR_NULL_HANDLE) {
    xrDestroySession(session);
  }
  {
    std::lock_guard<std::mutex> lock(g_xr);
    if (g_pq != nullptr) {  // bound to the session: released only after it is destroyed
      g_pq->Release();
      g_pq = nullptr;
    }
  }
  if (base.instance != XR_NULL_HANDLE) {
    xrDestroyInstance(base.instance);
  }
  if (device != nullptr) {
    device->Release();
  }
  queue->Release();
  log_line("session torn down");
  return !fatal;
}

// ---- worker ----------------------------------------------------------------------------------------

DWORD WINAPI worker(void*) {
  Mode done_for = Mode::Off;  // a probe or failed session is not repeated until the mode changes
  Mode last = Mode::Off;
  for (;;) {
    const Mode want = static_cast<Mode>(g_want.load());
    if (want != last) {
      done_for = Mode::Off;
      last = want;
    }
    if (want == Mode::Probe && done_for != Mode::Probe) {
      run_probe();
      done_for = Mode::Probe;
    } else if (is_session(want) && done_for != want) {
      if (!run_session()) {
        log_line("session: fatal; not retrying until the control file changes");
        done_for = want;
      } else if (is_session(static_cast<Mode>(g_want.load()))) {
        Sleep(2000);  // queue not ready yet, or the runtime ended the session: retry slowly
      }
    }
    Sleep(200);
  }
}

}  // namespace

namespace xr_runtime {

ID3D12CommandQueue* acquire_game_queue() { return acquire_queue(); }

bool session_running() { return g_session_running.load(std::memory_order_relaxed); }

double display_period_ms() {
  const LONGLONG p = g_xr_period_qpc.load(std::memory_order_relaxed);
  LARGE_INTEGER f;
  QueryPerformanceFrequency(&f);
  return p > 0 && f.QuadPart > 0 ? static_cast<double>(p) * 1000.0 / static_cast<double>(f.QuadPart) : 0.0;
}

const char* framegen_status() { return g_framegen_status; }

bool recommended_size(uint32_t* w, uint32_t* h) {
  *w = g_rec_w.load(std::memory_order_relaxed);
  *h = g_rec_h.load(std::memory_order_relaxed);
  return *w > 0 && *h > 0;
}

// AFW full-rate pacing (2026-11-15). [LIVE 16:12-16:31, pace=free] With AFW the game ran 90-98 fps against the 72 Hz
// headset: about one frame in four was never shown and the rest reached the compositor at uneven times (AUTHOR: "both
// eyes feel reprojected even though it shows high FPS"; the right eye, whose images alternate sources, wobbles most).
// While AFW is on, each Present is released on a headset frame boundary at most once per refresh (game <= 72 fps, in
// step with the display). "halfrate" keeps priority.
// [HEADSET/AUTHOR + LIVE 16:34-16:36] Removed again: no visible change, and at 90 Hz it waited 9.3 ms per frame and held
// the game at 72 fps (a frame ready just after a refresh waits almost a whole period). Kept as a stub for the A/B record.
bool pace_full_rate() { return false; }

void pace_present() {
  static LONGLONG last_release = 0;
  const bool half = g_half_rate.load(std::memory_order_relaxed);
  const bool full = !half && pace_full_rate();
  if (!(half || full) || !g_session_running.load(std::memory_order_relaxed) || !native_stereo::stereo_running()) {
    last_release = 0;
    return;
  }
  const LONGLONG base = g_xr_wait_qpc.load(std::memory_order_relaxed);
  const LONGLONG period = g_xr_period_qpc.load(std::memory_order_relaxed);
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  if (base == 0 || period <= 0 || now.QuadPart - base > 8 * period) {
    return;  // no live headset clock
  }
  // Release on the first headset frame boundary at least two frames (half rate) or one frame (AFW full rate), less a
  // quarter for jitter, after the last one.
  LONGLONG earliest = last_release == 0 ? now.QuadPart : last_release + (half ? 2 : 1) * period - period / 4;
  if (earliest < now.QuadPart) {
    earliest = now.QuadPart;
  }
  const LONGLONG k = (earliest - base + period - 1) / period;
  const LONGLONG target = base + (k > 0 ? k : 0) * period;
  if (target - now.QuadPart > 3 * period) {
    last_release = now.QuadPart;
    return;
  }
  LARGE_INTEGER freq;
  QueryPerformanceFrequency(&freq);
  const LONGLONG spin = freq.QuadPart / 667;  // last ~1.5 ms spun, the rest slept
  LARGE_INTEGER t = now;
  while (target - t.QuadPart > spin) {
    Sleep(1);
    QueryPerformanceCounter(&t);
  }
  while (t.QuadPart < target) {
    YieldProcessor();
    QueryPerformanceCounter(&t);
  }
  g_pace_frames.fetch_add(1, std::memory_order_relaxed);
  g_pace_wait_us.fetch_add(static_cast<std::uint64_t>((t.QuadPart - now.QuadPart) * 1000000 / freq.QuadPart),
                           std::memory_order_relaxed);
  last_release = t.QuadPart;
}

void set_overlay(bool visible, float distance_m, float width_m) {
  g_ovl_distance.store(distance_m < 0.4f ? 0.4f : distance_m > 5.f ? 5.f : distance_m);
  g_ovl_width_m.store(width_m < 0.3f ? 0.3f : width_m > 4.f ? 4.f : width_m);
  g_ovl_visible.store(visible);
}

void recentre_overlay() {
  g_ovl_recentre.store(true);
  g_cin_lazy.store(false);  // Keep the panel in front of the headset.
  g_cin_recentre.store(true);
}

void set_gui_panel(bool visible, float distance_m, float width_m, float height_m) {
  const float d = distance_m < 0.4f ? 0.4f : distance_m > 10.f ? 10.f : distance_m;
  const float h = height_m < -2.f ? -2.f : height_m > 2.f ? 2.f : height_m;
  if (d != g_gui_distance.load(std::memory_order_relaxed) || h != g_gui_height_m.load(std::memory_order_relaxed)) {
    g_gui_recentre.store(true);
  }
  g_gui_distance.store(d, std::memory_order_relaxed);
  g_gui_height_m.store(h, std::memory_order_relaxed);
  g_gui_width_m.store(width_m < 0.3f ? 0.3f : width_m > 10.f ? 10.f : width_m, std::memory_order_relaxed);
  if (g_gui_visible.exchange(visible) != visible) {
    log_line("gui: game UI panel %s (copies=%llu layers=%llu)", visible ? "on" : "off",
             static_cast<unsigned long long>(g_gui_copies.load()), static_cast<unsigned long long>(g_gui_layers.load()));
  }
}

void recentre_gui_panel() { g_gui_recentre.store(true); }

ID3D12Resource* overlay_acquire(uint32_t width, uint32_t height) {
  g_xr.lock();
  if (!g_running || g_session == XR_NULL_HANDLE || g_ovl_acquired || !ensure_overlay_locked(width, height)) {
    g_xr.unlock();
    return nullptr;
  }
  uint32_t index = 0;
  XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
  if (XR_FAILED(xrAcquireSwapchainImage(g_ovl_chain, &acquire_info, &index)) || index >= g_ovl_images.size()) {
    g_xr.unlock();
    return nullptr;
  }
  XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
  wait_info.timeout = 50000000;  // 50 ms
  const XrResult r = xrWaitSwapchainImage(g_ovl_chain, &wait_info);
  if (XR_FAILED(r) || r == XR_TIMEOUT_EXPIRED) {
    // Not waited: the image must still be released before the next acquire; release it empty.
    XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    if (r == XR_TIMEOUT_EXPIRED) {
      // A timed-out image is still acquired but cannot be released before a successful wait; drop the chain.
      destroy_overlay_locked();
    } else {
      xrReleaseSwapchainImage(g_ovl_chain, &release_info);
    }
    g_xr.unlock();
    return nullptr;
  }
  g_ovl_acquired = true;
  if (g_pq_on && g_pq != nullptr && g_pq_fence != nullptr) {
    // vr_menu writes this image on the game queue: hold the game queue until the private queue has passed the
    // runtime's wait for it (a stall only while the mod menu is open).
    if (ID3D12CommandQueue* game = acquire_queue()) {
      g_pq->Signal(g_pq_fence, ++g_pq_fence_next);
      game->Wait(g_pq_fence, g_pq_fence_next);
      game->Release();
      g_pq_bridges.fetch_add(1, std::memory_order_relaxed);
    }
  }
  return g_ovl_images[index];  // g_xr stays locked until overlay_release()
}

void overlay_release(bool written) {
  if (!g_ovl_acquired) {
    return;  // overlay_acquire returned nullptr and already unlocked
  }
  if (g_pq_on && g_pq != nullptr && g_copy_fence != nullptr) {
    // The runtime's release signal goes on the private queue: order it after vr_menu's copy on the game queue.
    if (ID3D12CommandQueue* game = acquire_queue()) {
      const UINT64 v = ++g_copy_fence_next;
      game->Signal(g_copy_fence, v);
      g_pq->Wait(g_copy_fence, v);
      game->Release();
    }
  }
  XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
  if (XR_SUCCEEDED(xrReleaseSwapchainImage(g_ovl_chain, &release_info)) && written) {
    g_ovl_has_image = true;  // no signal: the canvas is drawn in the same Present as the eye copy, which signals
  }
  g_ovl_acquired = false;
  g_xr.unlock();
}

bool latest_views(head_pose::Snapshot* out) {
  std::lock_guard<std::mutex> lock(g_pose_mutex);
  if (!g_latest.valid || !g_running) {
    return false;
  }
  *out = g_latest;
  return true;
}

void install() {
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_control_path = dir + L"\\AvatarVR_xr.txt";
  if (g_new_image == nullptr) {
    g_new_image = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  }
  g_log_path = dir + L"\\XR_RUNTIME.log";
  log_line("xr runtime build %s %s pid=%lu control=AvatarVR_xr.txt (missing file = off) loader=static Khronos %u.%u.%u",
           __DATE__, __TIME__, GetCurrentProcessId(),
           static_cast<unsigned>(XR_VERSION_MAJOR(XR_CURRENT_API_VERSION)),
           static_cast<unsigned>(XR_VERSION_MINOR(XR_CURRENT_API_VERSION)),
           static_cast<unsigned>(XR_VERSION_PATCH(XR_CURRENT_API_VERSION)));
}

void set_queue(IUnknown* queue_or_device) {
  if (queue_or_device == nullptr) {
    return;
  }
  ID3D12CommandQueue* queue = nullptr;
  if (FAILED(queue_or_device->QueryInterface(IID_PPV_ARGS(&queue))) || queue == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_queue_mutex);
  if (g_queue != nullptr) {
    g_queue->Release();
  }
  g_queue = queue;  // keep the reference
  log_line("swapchain queue recorded %p", queue);
}

void on_present(IDXGISwapChain* swapchain) {
  update_framegen_status();
  if (static_cast<Mode>(g_want.load(std::memory_order_relaxed)) == Mode::Cinema) {
    copy_present(swapchain);
  }
}

void set_sharpness(float percent) {
  const float v = percent <= 0.f ? 0.f : percent >= 100.f ? 1.f : percent / 100.f;
  if (v != g_sharpness.exchange(v)) {
    log_line("sharpen: strength %.0f%% (%s)", v * 100.f, v > 0.f ? "RCAS on both eyes" : "off, plain eye copies");
  }
}

void set_mirror_refresh(unsigned hz) {
  g_mirror_refresh.store(hz <= 120 ? hz : 120, std::memory_order_relaxed);
}

void poll() {
  if (!g_control_path.empty()) {
    burst_poll(g_control_path.substr(0, g_control_path.find_last_of(L"\\/")));
  }
  const Mode want = read_control();
  const Mode previous = static_cast<Mode>(g_want.exchange(static_cast<int>(want)));
  if (want != previous) {
    log_line("MODE -> %s", mode_name(want));
  }
  if (want == Mode::Cinema) {
    static float last_w = 0.f, last_d = 0.f;
    static bool last_swap = false;
    if (last_w != g_screen_width.load() || last_d != g_screen_distance.load() || last_swap != g_swap_eyes.load()) {
      last_w = g_screen_width.load();
      last_d = g_screen_distance.load();
      last_swap = g_swap_eyes.load();
      log_line("cinema: screen width=%.2fm distance=%.2fm swap=%d", last_w, last_d, last_swap ? 1 : 0);
      g_cin_lazy.store(false);
      g_cin_recentre.store(true);
    }
  }
  if (want != Mode::Off && !g_worker_started.exchange(true)) {
    HANDLE thread = CreateThread(nullptr, 0, &worker, nullptr, 0, nullptr);
    if (thread != nullptr) {
      CloseHandle(thread);
    } else {
      g_worker_started = false;
      log_line("worker thread creation failed");
    }
  }
}

}  // namespace xr_runtime
