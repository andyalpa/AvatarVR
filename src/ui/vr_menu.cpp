#include "vr_menu.hpp"

#include "../camera/native_stereo.hpp"
#include "../camera/mount_camera.hpp"
#include "../vr/xr_runtime.hpp"
#include "../dx12/dx12.hpp"
#include "../vr/xr_input.hpp"
#include "../vr/virtual_pad.hpp"
#include "monitor_preview.hpp"
#include "../diagnostics/frame_capture.hpp"
#include "../script/lua_host.hpp"
#include "../vr/vr_weapon.hpp"
#include "../vr/vr_controls.hpp"
#include "../vr/hand_gestures.hpp"
#include "../vr/dlss_twin.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../diagnostics/move_probe.hpp"

#include <windows.h>
#include <d3d12.h>
#include <dxgi1_4.h>
#include <Xinput.h>

#include "imgui.h"
#include "imgui_impl_dx12.h"

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../vr/wrist_hud.hpp"
#include "../camera/ui_ownership.hpp"
#include "../camera/camera.hpp"
#include "../settings_file.hpp"

namespace {

constexpr UINT kCanvasW = 1280;
constexpr UINT kCanvasH = 960;
constexpr int kFramesInFlight = 6;
constexpr int kShapeFlatEyes = 3;  // Render shape "Flat + VR eyes"

// ---- log -------------------------------------------------------------------------------------------------

std::wstring g_dir;
std::wstring g_log_path;

void log_line(const char* fmt, ...) {
  if (g_log_path.empty()) {
    return;
  }
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  SYSTEMTIME t = {};
  GetLocalTime(&t);
  char line[600];
  const int n = snprintf(line, sizeof(line), "%02u:%02u:%02u.%03u %s\r\n", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds, buffer);
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, line, static_cast<DWORD>(n > 0 && n < static_cast<int>(sizeof(line)) ? n : 0), &written, nullptr);
  CloseHandle(file);
}

std::wstring module_dir() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&module_dir), &self)) {
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

// ---- settings (mirrors of the two control files) ---------------------------------------------------------

struct Settings {
  // scripts\AvatarVR_native.txt
  bool stereo = false;     // "create": the engine second view for the right eye
  bool autostart = false;  // "autostart": keep native stereo armed across launches (run_cycle keeps the file)
  bool head = true;        // "head": head-tracked cameras
  float fov = 128.f;       // " fov D" (0 = the game's own)
  float offx = 0.f;        // "offx X" projection centre
  float offy = 0.17f;      // "offy Y"
  float cullfov = 0.f;     // "cullfov D" (0 = off)
  float sep = 0.03265f;    // "create SEP": camera offset when head tracking is off
  bool autofov = true;     // "autofov M": projection computed from the headset FOV and the render aspect ("Matched")
  bool eyeshift = false;   // "eyeshift": each eye centred on its own canted frustum (UI placement follows; menu "Per-eye lens projection")
  float ui_depth = 2.f;     // "uidepth M": distance the game UI appears at without eyeshift (0 = infinity)
  float margin = 0.f;      // autofov edge margin, percent
  float narrow = 100.f;    // "narrow N": matched projection covers N % of the headset FOV (sharper, black lens edges)
  float outer_edge = 0.f;  // "outeredge B": percent of the horizontal cut taken from each eye's inner (nasal) side
                           // (0 = both edges scaled evenly, 100 = outer edge kept); needs per-eye centering
  float narrow_v = 100.f;  // "narrowv R": vertical narrowing relative to the horizontal, percent (100 = same; the
                           // vertical share is capped at the full headset FOV)
  float narrow_vbias = 0.f; // "narrowvbias B": vertical cut from the top (+) or the bottom (-), percent
  float vcrop = 100.f;      // "vcrop P": keep P % of the vertical FOV with the eye height cut to match (2026-11-11)
  bool lodfov = false;     // "lodfov": object/texture detail no longer follows the VR FOV (ISSUE-04)
  bool fxstretch = false;  // "fxstretch": scan / boost / flash full-screen pictures cover the whole eye (UI pass)
  bool outsizerender = false;  // "outsizerender": screen effects inside an eye render get the eye size (not the window)
  bool spotprobe = false;   // "spotprobe": read-only spot-shadow atlas slot probe (ISSUE-04 test run)
  bool qtprobe = false;     // "qtprobe": read-only terrain/vegetation tile collapse probe (ISSUE-04 test run)
  bool lodprobe = false;    // "lodprobe": read-only per-eye LOD decision probe (on for the ISSUE-04 test run)
  bool lodtexgame = true;  // "lodtexgame": with the detail fix, texture streaming keeps the game's own value (performance)
  float lodkeep = 0.f;     // "lodkeep N": glow test, one detail consumer group back on the original value
  float lodref = 0.f;      // "lodref D": reference FOV for detail, degrees (0 = the game's own FOV)
  bool shadowunion = false; // "shadowunion": both eyes choose the same shadow-casting lights (ISSUE-04)
  bool flatfollow = false; // "flatfollow": test, the flat game camera turns with the headset (ISSUE-04)
  bool shadowfix = false;  // "shadowfix": each eye resolves sun shadows from its own shadow history (ISSUE-04)
  bool symproj = false;    // "symproj": test, each eye renders an on-axis frustum covering its window (ISSUE-04)
  // menu only: the game window's shape, which is the render resolution in windowed mode
  // Defaults are the settings that rendered best in the headset (2026-10-05): VR eye, Matched, 2160 px. The shape is
  // applied once at start-up when VR output is on, before gameplay builds the second view.
  int shape = 1;           // 0 = as the game set it, 1 = VR per-eye shape, 2 = VR wide shape
  float render_h = 2160.f; // client height in pixels for the VR shapes (0 = fit the screen); may exceed the screen
  float eye_scale = 0.f;   // 0 = legacy height/aspect sizing; otherwise percent of OpenXR's recommended eye pixels
  bool menus_mono = true;  // stereo only in gameplay; mono cinema screen on the start-up screens and in menus
  bool menu_stereo = true; // "nomenustereo": the 3D main menu (front end) stays mono on the flat screen (2026-11-09)
  bool zerosep = false;    // "zerosep": test, both eyes rendered from the same point (no parallax)
  bool vegsync = true;     // off = "novegsync": the second view's animation clock follows the main view's
  bool level = true;       // off = "nolevel": mouse / gamepad turn left-right only, the head looks up and down
  bool eye_ui = true;      // off = "noeyeui": the game UI in both eyes (main-view slot +0x130 shared)
  bool eye_fx = false;     // "eyefx": the UI's effect passes (scan highlights, frosted glass) in the right eye too (experimental)
  bool ui_place = true;    // "uiplace SIZE HEIGHT" / "nouiplace": the UI moved and scaled per eye (straight ahead at UI distance)
  float ui_size = 85.f;    // percent of the eye image
  float ui_height = 0.f;   // degrees, + = up
  bool mm_panel = true;    // "menupanel D W H" / "nomenupanel": the main menu on a panel fixed in the room
  float mm_dist = 2.f;     // m
  float mm_width = 1.8f;   // m
  float mm_height = 0.f;   // m above head height
  float share_slot = 0.f;  // "sharehooks N": experiment, main-view object slot N (1..11, 12 = all) copied into the second view
  // Off: run 2026-10-05 16:01 showed the game keeps its own render width across these switches (window 2400x1352 ->
  // swapchain 2856x1352, back to 1992x2160 -> swapchain 2856x2160 for the rest of the run), which breaks the VR shape.
  bool wide_menus = false; // menu only: a wide 16:9 game window while stereo is paused (menus), the VR shape in gameplay
  bool pair = true;        // xr "pair": both eyes from the same engine frame
  bool gamerate = true;    // off = xr "repeat": submit to the headset once per game frame (the runtime sees the real rate)
  // scripts\AvatarVR_xr.txt
  bool xr = true;          // "cinema ..." vs "off"
  bool proj = true;        // projection layer (head-tracked) vs flat cinema quads
  bool swap = false;
  bool mirror_right = false; // diagnostic "mirrorright": desktop shows the second view
  bool xr_depth = false;     // "xrdepth": per-eye depth to the headset layer (frame generation; 2026-11-12)
  bool framegen = false;     // "framegen": in-mod frame extrapolation (frame_gen.hpp; 2026-11-12)
  bool framegen_k0 = false;  // "framegenk0": test, generated frames without motion (dark-flash diagnosis)
  int lag = 1;             // right eye pose lag (StartFrames)
  int lagl = 2;            // left eye
  float screen_w = 3.f;    // cinema quad width (m)
  float screen_d = 2.5f;   // cinema quad distance (m)
  // menu only
  float menu_dist = 1.2f;
  float menu_width = 1.1f;
  float sharpness = 0.f;    // View -> Sharpness, 0-100 %: RCAS on the eye images (xr_runtime), saved as "sharpen N"
  float monitor_hz = 30.f;  // mirror copy refresh only; headset and desktop Present remain uncapped
  bool monitor_keep = true;
  bool square_eyes = false;
  float view_scale = 100.f;
  bool mirror = true;      // also draw on the monitor
  bool keep_active = true; // hide focus loss from the game (it drops to ~10 fps in the background)
  bool preview = false;    // scaled live copy of the game window on the monitor while the window does not fit
};

Settings g_s;
// Monitor placement is independent of eye resolution. Window-thread state is
// atomic; persistence and correction run on Present, outside the window proc.
std::atomic<int> g_monitor_w{0}, g_monitor_h{0}, g_monitor_x{0}, g_monitor_y{0};
std::atomic<DWORD> g_monitor_frame{0};
std::atomic<bool> g_monitor_drag{false}, g_monitor_capture{false}, g_monitor_keep{false};
std::atomic<ULONGLONG> g_monitor_bypass_until{0};
constexpr DWORD kMonitorFrameMask = WS_CAPTION | WS_THICKFRAME | WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX;

bool read_text(const std::wstring& path, char* text, size_t size) {
  return settings_file::read(path, text, size);
}

bool write_text(const std::wstring& path, const char* text) {
  return settings_file::write(path, text);
}

double number_after(const char* text, const char* key, double fallback) {
  const char* k = std::strstr(text, key);
  return k != nullptr ? std::strtod(k + std::strlen(key), nullptr) : fallback;
}

// Pull the live state back in from the files (they may have been edited by hand or reset by run_cycle).
void load_settings() {
  char text[512];
  if (read_text(g_dir + L"\\AvatarVR_monitor.txt", text, sizeof(text))) {
    g_s.monitor_hz = static_cast<float>(number_after(text, "refresh", 30.0));
    g_s.monitor_hz = std::clamp(g_s.monitor_hz, 0.f, 120.f);
    g_s.monitor_keep = number_after(text, "keep", 1) != 0;
    g_s.sharpness = std::clamp(static_cast<float>(number_after(text, "sharpen", 0.0)), 0.f, 100.f);
    const int mw = static_cast<int>(number_after(text, "windoww", 0));
    const int mh = static_cast<int>(number_after(text, "windowh", 0));
    if (mw >= 320 && mw <= 8192 && mh >= 240 && mh <= 8192) {
      g_monitor_w.store(mw); g_monitor_h.store(mh);
      g_monitor_x.store(static_cast<int>(number_after(text, "windowx", 0)));
      g_monitor_y.store(static_cast<int>(number_after(text, "windowy", 0)));
      g_monitor_frame.store(static_cast<DWORD>(number_after(text, "frame", 0)) & kMonitorFrameMask);
    }
  }
  if (read_text(g_dir + L"\\AvatarVR_native.txt", text, sizeof(text))) {
    const char* c = std::strstr(text, "create");
    g_s.stereo = c != nullptr;
    g_s.autostart = c != nullptr && std::strstr(text, "autostart") != nullptr;
    g_s.menus_mono = std::strstr(text, "anyscene") == nullptr;
    g_s.menu_stereo = std::strstr(text, "nomenustereo") == nullptr;
    {
      const char* sh = std::strstr(text, "sharehooks");
      g_s.share_slot = sh == nullptr ? 0.f : static_cast<float>(number_after(text, "sharehooks", 12.0));
    }
    g_s.eye_ui = std::strstr(text, "noeyeui") == nullptr;
    g_s.eye_fx = std::strstr(text, "eyefx") != nullptr && std::strstr(text, "noeyefx") == nullptr;
    g_s.ui_place = std::strstr(text, "nouiplace") == nullptr;
    g_s.mm_panel = std::strstr(text, "nomenupanel") == nullptr;
    if (const char* mp = g_s.mm_panel ? std::strstr(text, "menupanel") : nullptr) {
      float* dst[3] = {&g_s.mm_dist, &g_s.mm_width, &g_s.mm_height};
      const char* cur = mp + 9;
      for (float* f : dst) {
        char* end = nullptr;
        const double v = std::strtod(cur, &end);
        if (end == cur) {
          break;
        }
        *f = static_cast<float>(v);
        cur = end;
      }
    }
    if (const char* up = g_s.ui_place ? std::strstr(text, "uiplace") : nullptr) {
      char* end = nullptr;
      const double size = std::strtod(up + 7, &end);
      if (end != up + 7) {
        g_s.ui_size = static_cast<float>(size);
        g_s.ui_height = static_cast<float>(std::strtod(end, nullptr));
      }
    }
    g_s.level = std::strstr(text, "nolevel") == nullptr;
    // Animation sync is always on (it fixed the wind desync, 2026-10-07); the test settings below have no menu rows
    // any more, so values left in the file by earlier tests are dropped.
    g_s.vegsync = true;
    g_s.zerosep = false;
    g_s.share_slot = 0.f;
    g_s.shadowfix = std::strstr(text, "shadowfix") != nullptr;
    g_s.flatfollow = std::strstr(text, "flatfollow") != nullptr;
    g_s.shadowunion = std::strstr(text, "shadowunion") != nullptr;
    g_s.lodfov = std::strstr(text, "lodfov") != nullptr;
    g_s.outsizerender = std::strstr(text, "outsizerender") != nullptr;
    g_s.fxstretch = std::strstr(text, "fxstretch") != nullptr;
    g_s.lodref = static_cast<float>(number_after(text, "lodref", 0.0));
    g_s.lodkeep = static_cast<float>(std::clamp(number_after(text, "lodkeep", 0.0), 0.0, 15.0));
    g_s.lodtexgame = std::strstr(text, "nolodtexgame") == nullptr;
    // research probes are opt-in (performance pass 2026-10-22)
    g_s.lodprobe = std::strstr(text, "lodprobe") != nullptr && std::strstr(text, "nolodprobe") == nullptr;
    g_s.qtprobe = std::strstr(text, "qtprobe") != nullptr && std::strstr(text, "noqtprobe") == nullptr;
    g_s.spotprobe = std::strstr(text, "spotprobe") != nullptr && std::strstr(text, "nospotprobe") == nullptr;
    if (c != nullptr) {
      g_s.head = std::strstr(text, "head") != nullptr;
      g_s.fov = static_cast<float>(number_after(text, " fov", 0.0));
      g_s.offx = static_cast<float>(number_after(text, "offx", 0.0));
      g_s.offy = static_cast<float>(number_after(text, "offy", 0.0));
      g_s.cullfov = static_cast<float>(number_after(text, "cullfov", 0.0));
      g_s.autofov = std::strstr(text, "autofov") != nullptr;
    g_s.view_scale = static_cast<float>(std::clamp(number_after(text, "viewscale", 100.0), 70.0, 130.0));
      if (g_s.autofov) {
        g_s.margin = static_cast<float>(number_after(text, "autofov", 0.0));
        g_s.narrow = static_cast<float>(number_after(text, "narrow", 100.0));
        if (const char* oe = std::strstr(text, "outeredge")) {
          char* end = nullptr;
          const double b = std::strtod(oe + 9, &end);
          g_s.outer_edge = end == oe + 9 ? 100.f : static_cast<float>(std::clamp(b, 0.0, 100.0));  // bare = older files
        } else {
          g_s.outer_edge = 0.f;
        }
        g_s.narrow_v = static_cast<float>(std::clamp(number_after(text, "narrowv ", 100.0), 50.0, 150.0));
        g_s.narrow_vbias = static_cast<float>(number_after(text, "narrowvbias", 0.0));
        g_s.vcrop = static_cast<float>(std::clamp(number_after(text, " vcrop ", 100.0), 60.0, 100.0));
        g_s.eyeshift = std::strstr(text, "eyeshift") != nullptr;
        g_s.symproj = std::strstr(text, "symproj") != nullptr;
        g_s.ui_depth = static_cast<float>(number_after(text, "uidepth", 2.0));
      }
      char* end = nullptr;
      const double m = std::strtod(c + 6, &end);
      if (end != c + 6 && m > -0.5 && m < 0.5) {
        g_s.sep = static_cast<float>(m);
      }
    }
  }
  // scripts\AvatarVR_shape.txt "shape S height H": the render shape and height kept across launches (written by
  // apply_shape). Missing file = the defaults above.
  if (read_text(g_dir + L"\\AvatarVR_shape.txt", text, sizeof(text))) {
    g_s.square_eyes = number_after(text, "square", 0) != 0;
    const int shape = static_cast<int>(number_after(text, "shape", g_s.shape));
    const float h = static_cast<float>(number_after(text, "height", g_s.render_h));
    const float scale = static_cast<float>(number_after(text, "scale", 0));
    if (shape >= 0 && shape <= kShapeFlatEyes) {
      g_s.shape = shape;
    }
    if (h == 0.f || (h >= 1200.f && h <= 4096.f)) {
      g_s.render_h = h;
    }
    if (scale == 0.f || (scale >= 70.f && scale <= 140.f)) {
      g_s.eye_scale = scale;
    }
  }
  if (read_text(g_dir + L"\\AvatarVR_xr.txt", text, sizeof(text))) {
    g_s.xr = std::strncmp(text, "cinema", 6) == 0;
    if (g_s.xr) {
      g_s.proj = std::strstr(text, "proj") != nullptr;
      g_s.swap = std::strstr(text, "swap") != nullptr;
      g_s.mirror_right = std::strstr(text, "mirrorright") != nullptr;
      g_s.xr_depth = std::strstr(text, "xrdepth") != nullptr;
      g_s.framegen = std::strstr(text, "framegen") != nullptr;
      g_s.framegen_k0 = std::strstr(text, "framegenk0") != nullptr;
      g_s.pair = std::strstr(text, "pair") != nullptr;
      g_s.gamerate = std::strstr(text, "repeat") == nullptr;
      const char* lagl = std::strstr(text, "lagl");
      const char* lag = std::strstr(text, "lag ");
      g_s.lag = lag != nullptr ? static_cast<int>(std::strtol(lag + 3, nullptr, 10)) : 1;
      g_s.lagl = lagl != nullptr ? static_cast<int>(std::strtol(lagl + 4, nullptr, 10)) : g_s.lag + 1;
      // "cinema W D ..." when the two numbers come first (how the menu writes it)
      char* end = nullptr;
      const double w = std::strtod(text + 6, &end);
      if (end != text + 6) {
        g_s.screen_w = static_cast<float>(w);
        const char* start = end;
        const double d = std::strtod(start, &end);
        if (end != start) {
          g_s.screen_d = static_cast<float>(d);
        }
      }
    }
  }
}

float shape_aspect(int shape);  // below

// Render shape 3 "Flat + VR eyes" (2026-10-09): the game window keeps its own size (widescreen, any size) and each
// eye renders at Render height x the headset eye's aspect, independent of it (native_stereo "eyesize W H both";
// dx12.cpp grows the swapchain buffers and presents the window-size region). Shapes 1/2 size the window instead.
bool forced_shape(int shape) { return shape == 1 || shape == 2; }

bool eye_render_size(int* w, int* h) {
  if (g_s.shape != kShapeFlatEyes) {
    return false;
  }
  std::uint32_t recommended_w = 0, recommended_h = 0;
  if (g_s.eye_scale >= 70.f && xr_runtime::recommended_size(&recommended_w, &recommended_h)) {
    const float scale = std::min(g_s.eye_scale * 0.01f,
                                 std::min(4096.f / recommended_w, 4096.f / recommended_h));
    *w = static_cast<int>(recommended_w * scale) & ~7;
    *h = static_cast<int>(recommended_h * scale) & ~7;
    if (g_s.square_eyes) *w = *h;
  } else {
    if (g_s.render_h < 1200.f) return false;
    *h = static_cast<int>(g_s.render_h) & ~7;
    *w = g_s.square_eyes ? *h : static_cast<int>(static_cast<float>(*h) * shape_aspect(1)) & ~7;
  }
  // Vertical FOV (2026-11-11): fewer rows for the same share of the vertical FOV that native_stereo drops ("vcrop"),
  // so pixel density stays the same. Square eyes keep their own rule.
  if (!g_s.square_eyes && g_s.autofov && g_s.vcrop >= 60.f && g_s.vcrop < 99.5f) {
    *h = static_cast<int>(static_cast<float>(*h) * g_s.vcrop * 0.01f) & ~7;
  }
  return *w >= 256 && *h >= 256;
}

void apply_native() {
  char text[512];
  if (!g_s.stereo) {
    snprintf(text, sizeof(text), "observe");
  } else {
    int n = snprintf(text, sizeof(text), "create %.5f%s offscreen%s%s%s", g_s.sep, g_s.head ? " head" : "",
                     g_s.autostart ? " autostart" : "", g_s.menus_mono ? "" : " anyscene",
                     g_s.eye_ui ? "" : " noeyeui");
    if (g_s.eye_fx) {
      n += snprintf(text + n, sizeof(text) - n, " eyefx");
    }
    if (!g_s.menu_stereo) {
      n += snprintf(text + n, sizeof(text) - n, " nomenustereo");
    }
    if (!g_s.level) {
      n += snprintf(text + n, sizeof(text) - n, " nolevel");
    }
    if (!g_s.vegsync) {
      n += snprintf(text + n, sizeof(text) - n, " novegsync");
    }
    if (g_s.zerosep) {
      n += snprintf(text + n, sizeof(text) - n, " zerosep");
    }
    if (g_s.shadowfix) {
      n += snprintf(text + n, sizeof(text) - n, " shadowfix");
    }
    if (g_s.flatfollow) {
      n += snprintf(text + n, sizeof(text) - n, " flatfollow");
    }
    if (g_s.shadowunion) {
      n += snprintf(text + n, sizeof(text) - n, " shadowunion");
    }
    if (g_s.lodfov) {
      n += snprintf(text + n, sizeof(text) - n, " lodfov");
    }
    if (g_s.outsizerender) {
      n += snprintf(text + n, sizeof(text) - n, " outsizerender");
    }
    if (g_s.fxstretch) {
      n += snprintf(text + n, sizeof(text) - n, " fxstretch");
    }
    if (g_s.lodref >= 30.f) {
      n += snprintf(text + n, sizeof(text) - n, " lodref %.0f", g_s.lodref);
    }
    if (g_s.lodprobe) n += snprintf(text + n, sizeof(text) - n, " lodprobe");
    if (g_s.qtprobe) n += snprintf(text + n, sizeof(text) - n, " qtprobe");
    if (g_s.spotprobe) n += snprintf(text + n, sizeof(text) - n, " spotprobe");
    n += snprintf(text + n, sizeof(text) - n, "%s", g_s.lodtexgame ? " lodtexgame" : " nolodtexgame");
    if (g_s.lodkeep >= 0.5f) {
      n += snprintf(text + n, sizeof(text) - n, " lodkeep %.0f", g_s.lodkeep);
    }
    if (g_s.share_slot >= 1.f) {
      n += snprintf(text + n, sizeof(text) - n, " sharehooks %d", static_cast<int>(g_s.share_slot + 0.5f));
    }
    if (g_s.fov > 0.f) {
      n += snprintf(text + n, sizeof(text) - n, " fov %.1f", g_s.fov);
    }
    if (g_s.offx != 0.f || g_s.offy != 0.f) {
      n += snprintf(text + n, sizeof(text) - n, " offx %.3f offy %.3f", g_s.offx, g_s.offy);
    }
    if (g_s.cullfov > 0.f) {
      n += snprintf(text + n, sizeof(text) - n, " cullfov %.0f", g_s.cullfov);
    }
    if (g_s.autofov) {
      n += snprintf(text + n, sizeof(text) - n, " autofov %.0f%s", g_s.margin, g_s.eyeshift ? " eyeshift" : "");
      if (std::fabs(g_s.view_scale - 100.f) > 0.01f)
        n += snprintf(text + n, sizeof(text) - n, " viewscale %.0f", g_s.view_scale);
      if (g_s.narrow < 99.5f) {
        n += snprintf(text + n, sizeof(text) - n, " narrow %.0f", g_s.narrow);
        if (g_s.outer_edge > 0.5f) {
          n += snprintf(text + n, sizeof(text) - n, " outeredge %.0f", g_s.outer_edge);
        }
      }
      if (std::fabs(g_s.narrow_v - 100.f) > 0.5f) {
        n += snprintf(text + n, sizeof(text) - n, " narrowv %.0f", g_s.narrow_v);
      }
      if (std::fabs(g_s.narrow_vbias) > 0.5f) {
        n += snprintf(text + n, sizeof(text) - n, " narrowvbias %.0f", g_s.narrow_vbias);
      }
      if (g_s.vcrop < 99.5f) {
        n += snprintf(text + n, sizeof(text) - n, " vcrop %.0f", g_s.vcrop);
      }
      if (g_s.symproj) {
        n += snprintf(text + n, sizeof(text) - n, " symproj");
      }
    }
    n += snprintf(text + n, sizeof(text) - n, " uidepth %.1f", g_s.ui_depth);
    if (g_s.ui_place) {
      n += snprintf(text + n, sizeof(text) - n, " uiplace %.0f %.1f", g_s.ui_size, g_s.ui_height);
    } else {
      n += snprintf(text + n, sizeof(text) - n, " nouiplace");
    }
    if (g_s.mm_panel) {
      n += snprintf(text + n, sizeof(text) - n, " menupanel %.2f %.2f %.2f", g_s.mm_dist, g_s.mm_width, g_s.mm_height);
    } else {
      n += snprintf(text + n, sizeof(text) - n, " nomenupanel");
    }
    int ew = 0, eh = 0;
    if (eye_render_size(&ew, &eh) && n > 0 && n < static_cast<int>(sizeof(text))) {
      snprintf(text + n, sizeof(text) - n, " eyesize %d %d both", ew, eh);
    }
  }
  if (write_text(g_dir + L"\\AvatarVR_native.txt", text)) {
    native_stereo::poll();
    log_line("native <- %s", text);
  }
}

void apply_xr() {
  char text[192];
  if (!g_s.xr) {
    snprintf(text, sizeof(text), "off");
  } else {
    snprintf(text, sizeof(text), "cinema %.2f %.2f%s native%s%s%s lag %d lagl %d%s%s%s", g_s.screen_w, g_s.screen_d,
             g_s.swap ? " swap" : "", g_s.proj ? " proj" : "", g_s.pair ? " pair" : "", g_s.gamerate ? "" : " repeat",
             g_s.lag, g_s.lagl, g_s.mirror_right ? " mirrorright" : "", g_s.xr_depth ? " xrdepth" : "",
             g_s.framegen ? (g_s.framegen_k0 ? " framegenk0" : " framegen") : "");
  }
  if (write_text(g_dir + L"\\AvatarVR_xr.txt", text)) {
    xr_runtime::poll();
    log_line("xr <- %s", text);
  }
}

// ---- menu model -----------------------------------------------------------------------------------------

enum class Kind { Toggle, Choice, Float, Int, Action };
enum class Target { None, Native, Xr, Menu, Shape };

struct Row {
  const char* label;
  const char* help;
  Kind kind;
  Target target;
  bool* b = nullptr;
  float* f = nullptr;
  int* i = nullptr;
  float lo = 0.f, hi = 1.f, step = 0.1f;
  float zero_below = -FLT_MAX;  // Float: values below this read as 0 ("Game"/"Off"), stepping jumps over the gap
  const char* zero_label = nullptr;
  const char* fmt = "%.2f";
  float display_scale = 1.f;
  const char* choices[4] = {};  // Toggle: {off, on}; Choice uses i with these labels
  int choice_count = 0;
  void (*action)() = nullptr;
};

void act_recentre_view();
void act_recentre_menu();
void act_recentre_main_menu();
void act_preset();
void act_close();
void act_capture();
void act_mv_burst();
void act_small_window();

constexpr int kMaxRows = 64;  // rows per tab; the panel shows kVisibleRows at a time and scrolls with the selection
constexpr int kVisibleRows = 10;

struct Tab {
  const char* name;
  Row rows[kMaxRows];
  int count;
};

constexpr int kTabCount = 6;  // 5 = Scripts (lua_host, rebuilt when its rows change)
Tab g_tabs[kTabCount];
int g_tab = 0;
int g_sel = 0;  // -1 = the tab strip

Row toggle(const char* label, const char* help, Target t, bool* b, const char* off = "Off", const char* on = "On") {
  Row r{label, help, Kind::Toggle, t};
  r.b = b;
  r.choices[0] = off;
  r.choices[1] = on;
  r.choice_count = 2;
  return r;
}

Row slider(const char* label, const char* help, Target t, float* f, float lo, float hi, float step, const char* fmt,
           float scale = 1.f) {
  Row r{label, help, Kind::Float, t};
  r.f = f;
  r.lo = lo;
  r.hi = hi;
  r.step = step;
  r.fmt = fmt;
  r.display_scale = scale;
  return r;
}

Row islider(const char* label, const char* help, Target t, int* i, int lo, int hi, const char* fmt) {
  Row r{label, help, Kind::Int, t};
  r.i = i;
  r.lo = static_cast<float>(lo);
  r.hi = static_cast<float>(hi);
  r.step = 1.f;
  r.fmt = fmt;
  return r;
}

Row choice(const char* label, const char* help, Target t, int* i, const char* a, const char* b, const char* c) {
  Row r{label, help, Kind::Choice, t};
  r.i = i;
  r.choices[0] = a;
  r.choices[1] = b;
  r.choices[2] = c;
  r.choice_count = 3;
  return r;
}

Row choice(const char* label, const char* help, Target t, int* i, const char* a, const char* b, const char* c, const char* d) {
  Row r = choice(label, help, t, i, a, b, c);
  r.choices[3] = d;
  r.choice_count = 4;
  return r;
}

Row action(const char* label, const char* help, void (*fn)()) {
  Row r{label, help, Kind::Action, Target::None};
  r.action = fn;
  return r;
}

// Appends a row. A row past kMaxRows is dropped: writing it would land on the tab's count (rows[] is followed by it)
// and the menu would then draw and edit rows that do not exist.
void add(Tab& tab, const Row& r) {
  if (tab.count < kMaxRows) {
    tab.rows[tab.count++] = r;
  }
}

void build_tabs() {
  Tab& stereo = g_tabs[0];
  stereo.name = "Stereo";
  stereo.count = 0;
  add(stereo, toggle("Native stereo",
         "Renders a real second engine camera for the right eye. It is built when gameplay starts; the "
         "start-up screens and the game's menus stay mono (see Mono in menus).",
         Target::Native, &g_s.stereo));
  add(stereo, toggle("Auto start",
                     "Keeps native stereo on from game launch. Stereo then starts by itself when gameplay starts, "
                     "so it no longer needs to be switched on by hand.",
                     Target::Native, &g_s.autostart));
  add(stereo, toggle("Mono in menus",
                     "On: the start-up video, main menu and in-game menus are shown flat on the cinema screen and "
                     "stereo runs only in gameplay. Off: stereo wherever the game renders its world.",
                     Target::Native, &g_s.menus_mono));
  add(stereo, toggle("3D main menu",
                     "On: the game's 3D main menu runs in stereo (on the main menu panel). Off: the start-up screens "
                     "and the main menu stay flat on the cinema screen, and stereo starts with gameplay. Test switch "
                     "for the performance question of 2026-11-09.",
                     Target::Native, &g_s.menu_stereo));
  add(stereo, toggle("Level horizon",
                     "On: the mouse / gamepad only turns you left and right and the horizon stays level; look up and "
                     "down with your head. Off: the game's up/down look tilts the view as on a monitor.",
                     Target::Native, &g_s.level));
  add(stereo, toggle("Keep game pitch level",
                     "On (VR gameplay): the right stick's up/down no longer tilts the game's own head; the mod steers it "
                     "back to level, so the first-person shoulders, body and camera stop shifting. You look up and down "
                     "with your head. Right-stick up/down in in-game menus (map, inventory) is also blocked while on.",
                     Target::None, virtual_pad::pitch_level_flag()));
  add(stereo, slider("Default game pitch",
                     "The up/down angle the game's own head is held at (degrees, + up). Some angles show more of the "
                     "body/armour and bring the arms closer. After changing it, use Scripts > Recapture rest pose.",
                     Target::None, virtual_pad::pitch_target_value(), -60.f, 40.f, 1.f, "%.0f"));
  // Removed 2026-10-07 (answered, kept in code): "Sync animation" (now always on), "Share UI hook (test)",
  // "Test UI hooks (auto)", "Test right-eye effects (auto)".
  add(stereo, toggle("Head tracking", "Both eye cameras follow the headset (rotation and position).",
         Target::Native, &g_s.head));
  add(stereo, action("Recenter view", "Takes where you are looking now as straight ahead.", &act_recentre_view));
  add(stereo, slider("Camera offset",
         "Right-eye offset used only when head tracking is off (half the eye distance).",
         Target::Native, &g_s.sep, 0.f, 0.06f, 0.0005f, "%.1f mm", 1000.f));

  Tab& view = g_tabs[1];
  view.name = "View";
  view.count = 0;
  add(view, slider("Eye render height",
         "Manual eye height. Used when Eye resolution scale is Manual. Higher improves detail and costs GPU time; monitor size is independent.",
         Target::Shape, &g_s.render_h, 1200.f, 4096.f, 80.f, "%.0f px"));
  {
    Row r = slider("Eye resolution scale",
                   "Manual keeps the prior eye sizing. Otherwise both eye dimensions follow the OpenXR recommended size at this percentage (capped at 4096 px). Projection remains matched to the rendered aspect.",
                   Target::Shape, &g_s.eye_scale, 70.f, 140.f, 10.f, "%.0f%%");
    r.zero_below = 70.f;
    r.zero_label = "Manual";
    add(view, r);
  }
  add(view, toggle("Square eye view", "Forces eye width equal to height, including with Eye resolution scale. Changes field of view, never stretches the image; vertical edge coverage can be narrower.",
                   Target::Shape, &g_s.square_eyes));
  add(view, slider("VR FOV", "100% keeps the current matched FOV. Lower narrows the view; higher widens it. Camera and headset projection change together. Resolution and square shape stay unchanged.",
                   Target::Native, &g_s.view_scale, 70.f, 130.f, 1.f, "%.0f%%"));
  add(view, slider("Vertical FOV",
                   "Share of the headset's top-to-bottom view rendered; 100% = all. Lower drops the top and bottom edges the "
                   "lens shows least (more from the bottom) and renders fewer rows, so detail per degree stays the same and "
                   "GPU time falls. Raise Eye resolution scale with the saving to get a sharper image. Needs Matched FOV.",
                   Target::Shape, &g_s.vcrop, 60.f, 100.f, 1.f, "%.0f%%"));
  add(view, toggle("Centred eye projection (test)",
                   "Test for light/shadow and shape shifts with head turns: each eye renders a frustum centred on where it "
                   "looks (covering the same lens area), instead of the off-centre one. Slightly softer image. Off = current.",
                   Target::Native, &g_s.symproj));
  add(view, toggle("Per-eye sun shadows (test)",
                   "Fix test for sun shadows that show in one eye only: each eye resolves its shadows from its own "
                   "shadow history instead of the one both eyes share. Off = current.",
                   Target::Native, &g_s.shadowfix));
  add(view, toggle("Flat camera follows head (test)",
                   "Test for shapes, lights and shadows that change with head turns: the game's flat camera turns with "
                   "the headset and covers both eye views, so systems that still use it match what you see. The "
                   "crosshair and scan targeting follow your head while on. Off = current.",
                   Target::Native, &g_s.flatfollow));
  add(view, toggle("Same shadow lights both eyes (fix)",
                   "Fix for shadows that show in one eye only: the game picks a limited set of shadow-casting lights per eye "
                   "from what that eye sees, so the eyes disagree near the edges. On: both eyes pick from their combined view "
                   "and get the same set. Off = current.",
                   Target::Native, &g_s.shadowunion));
  add(view, toggle("Full-screen effects fill the eye (fix)",
                   "Fix for the X-ray scan, mount boost and flash grenade showing as a 16:9 box (or a corner patch) with black "
                   "around it: the game draws those effects as one picture in its UI layer, which the HUD placement shrinks. "
                   "On: that picture covers the whole eye; the HUD keeps its placement. Off = current.",
                   Target::Native, &g_s.fxstretch));
  add(view, toggle("Screen effects at eye size (fix)",
                   "Fix for flash grenade, mount boost and similar full-screen effects drawing a washed-out 16:9 box inside "
                   "each eye: the game sizes those effects from the flat window (1280x720). On: inside each eye's render "
                   "they get the eye size instead. Off = current.",
                   Target::Native, &g_s.outsizerender));
  add(view, toggle("Per-eye lens projection",
                   "Each eye renders exactly its own lens field of view (off-centre, like the Cyberpunk VR port) instead of "
                   "the symmetric union of both: the same pixels cover only what that lens shows, so more pixels per degree "
                   "and the outer edge is covered. HUD placement follows each eye's frustum. Off = symmetric union.",
                   Target::Native, &g_s.eyeshift));
  add(view, toggle("Detail independent of VR FOV",
                   "Fix for objects and textures changing with the VR FOV and on head turns: the game picks model and "
                   "texture detail from the rendered FOV, so a wide VR view drops detail. On: detail is chosen as for the "
                   "reference FOV below, the same in both eyes. Costs some GPU. Off = current.",
                   Target::Native, &g_s.lodfov));
  {
    Row r = slider("Detail reference FOV",
                   "Detail is chosen as if the view were this wide. Lower = more detail (more GPU). Game = the game's own "
                   "FOV setting.", Target::Native, &g_s.lodref, 50.f, 110.f, 5.f, "%.0f deg");
    r.zero_below = 50.f;
    r.zero_label = "Game";
    add(view, r);
  }
  add(view, toggle("Texture detail follows game",
                   "With Detail independent of VR FOV on: model and terrain detail use the reference FOV, but texture "
                   "streaming keeps the game's own level. Saves GPU memory and bandwidth. Off = textures also use the reference.",
                   Target::Native, &g_s.lodtexgame));
  add(view, toggle("LOD probe (read-only)", "Logs how the two eyes decide near/far detail for objects: eye disagreements "
                   "and flips while the head is still vs turning. Changes nothing visible.", Target::Native, &g_s.lodprobe));
  add(view, toggle("Vegetation tile probe (read-only)", "Logs how terrain/vegetation detail tiles collapse when they leave each "
                   "eye's view and rebuild when they return, head still vs turning. Changes nothing visible.", Target::Native, &g_s.qtprobe));
  add(view, toggle("Spot-shadow slot probe (read-only)", "Logs which atlas slot each lamp shadow uses in each eye, to find lamps "
                   "whose shadow shows in one eye only. Changes nothing visible.", Target::Native, &g_s.spotprobe));
  add(view, slider("Glow test (detail group)",
                   "Diagnostic with Detail independent of VR FOV on: puts ONE group back on the game's original detail to "
                   "find what switches the fauna glow off. 0 off, 1 model detail, 2 culling, 3 texture streaming, "
                   "4 unknown system, 5 all four, 6 culling first half only, 7 culling second half only, 8 light culling only, "
                   "9 particle culling only, 10 and 11 two other systems, 12 lights+particles, 13 = 10+11, 14 = 8+10+11, "
                   "15 = 9+10+11. Hold each about 10 s: the glow fades in slowly.", Target::Native, &g_s.lodkeep, 0.f, 15.f, 1.f, "%.0f"));
  add(view, slider("Sharpness",
         "Sharpens both eye images after DLSS (AMD FidelityFX RCAS, same on both eyes). The game's DLSS has no sharpness "
         "option of its own. 0 = off. Also sharpens the in-world HUD; costs a little GPU time.",
         Target::Menu, &g_s.sharpness, 0.f, 100.f, 5.f, "%.0f%%"));
  add(view, slider("Mirror copy refresh",
         "How often the left-eye copy refreshes. Lower saves copy bandwidth; 0 refreshes every frame. Does not limit headset rendering or desktop Present.",
         Target::Menu, &g_s.monitor_hz, 0.f, 120.f, 10.f, "%.0f Hz"));
  add(view, action("Small monitor window", "Sets the monitor window to 1280 x 720. Eye resolution stays unchanged.", &act_small_window));
  add(view, toggle("Keep monitor window", "Remembers the monitor size, position and border. Drag or resize it to save a new choice. Eye resolution stays independent.",
                   Target::Menu, &g_s.monitor_keep));
  add(view, action("Recenter view", "Sets your current direction as straight ahead without changing lens coverage.", &act_recentre_view));

  // UI tab (2026-10-08): the game's UI in stereo. In-game HUD = placed straight ahead, follows the head; main menu =
  // a panel fixed in the room (native_stereo: front end until the first load after the 3D main menu).
  Tab& ui = g_tabs[2];
  ui.name = "UI";
  ui.count = 0;
  add(ui, toggle("UI in both eyes",
                 "Draws the game's UI (menus, compass, prompts, subtitles) into the right eye too.",
                 Target::Native, &g_s.eye_ui));
  add(ui, toggle("UI effects in both eyes",
                 "The right eye renders the UI's own effects too: scan highlights and frosted-glass backdrops (weapon "
                 "wheel, panels). Experimental: both eyes share the game's effect images, so the eye drawn last wins and "
                 "the other eye's highlights move. Off (default): the right eye shows the left eye's highlights.",
                 Target::Native, &g_s.eye_fx));
  add(ui, toggle("UI placement",
                 "On: the game's UI is moved inside each eye's picture so both eyes see one UI: the HUD straight ahead "
                 "at HUD distance, the main menu on a panel fixed in the room. Works with Per-eye centering on or off. "
                 "Off: the UI stays where the game draws it (double with Per-eye centering on).",
                 Target::Native, &g_s.ui_place));
  {
    Row r = slider("HUD distance",
                   "In-game HUD (compass, prompts, subtitles, markers): how far away it appears. It follows your head. "
                   "The scene is not affected.",
                   Target::Native, &g_s.ui_depth, 0.5f, 10.f, 0.25f, "%.2f m");
    r.zero_below = 0.5f;
    r.zero_label = "Infinity";
    add(ui, r);
  }
  add(ui, slider("HUD size", "In-game HUD: size as a share of the eye's picture. Smaller keeps the edges (compass, "
                 "subtitles) closer to the middle of your view.",
                 Target::Native, &g_s.ui_size, 40.f, 110.f, 5.f, "%.0f %%"));
  add(ui, slider("HUD height", "In-game HUD: moves it up (+) or down (-).", Target::Native, &g_s.ui_height, -20.f, 20.f,
                 1.f, "%.0f deg"));
  add(ui, toggle("Main menu panel",
                 "On: the game's main menu stays on a panel fixed in the room, placed in front of you when it opens. "
                 "Off: the main menu is placed like the HUD and follows your head.",
                 Target::Native, &g_s.mm_panel, "Follows head", "Fixed in room"));
  add(ui, slider("Main menu distance", "How far in front of you the main menu panel is placed.", Target::Native,
                 &g_s.mm_dist, 0.75f, 6.f, 0.25f, "%.2f m"));
  add(ui, slider("Main menu width", "Width of the main menu panel.", Target::Native, &g_s.mm_width, 0.5f, 6.f, 0.1f,
                 "%.1f m"));
  add(ui, slider("Main menu height", "Moves the main menu panel up (+) or down (-) from your head height.",
                 Target::Native, &g_s.mm_height, -1.5f, 1.5f, 0.05f, "%.2f m"));
  add(ui, action("Recenter main menu", "Places the main menu panel in front of where you are looking.",
                 &act_recentre_main_menu));
  {
    // Wrist HUD (FEATURE-13 Phase 3): its values live in wrist_hud (scripts\AvatarVR_hud.txt), not in the native file.
    wrist_hud::Settings* w = wrist_hud::settings();
    add(ui, action("Capture native HUD ownership (30s)",
                   "Read-only research capture. Close the menu, equip a weapon and play for 30 seconds. Saves widget ancestry and drawing commands to HUD_OWNERSHIP beside the mod DLL.",
                   &ui_ownership::request));
    add(ui, action(ui_ownership::status(), "Capture status; this does not change HUD placement or visibility.", nullptr));
    add(ui, action("Stop HUD ownership capture", "Stops a pending or active capture and saves observations already collected.", &ui_ownership::stop));
    add(ui, toggle("Native HUD on arm (experimental)",
                   "Draws the game's own health and ammo widgets on the forearm of the hand without the weapon, separately for each eye. No screen crop. Turns the cropped Wrist HUD panels off. Prompts and the rest of the HUD stay on the flat HUD.",
                   Target::None, &w->native_arm));
    add(ui, action("Next native widget test",
                   "Experimental, not saved. Moves, tilts or hides only the health bar, then only the ammo widgets, by changing the matrix the game draws them with. Other HUD art must stay where it is. Keep the shader test off.",
                   &ui_ownership::widget_test_next));
    add(ui, action(ui_ownership::widget_test_status(), "Current native widget test. Selecting Next past the last one turns it off.", nullptr));
    add(ui, toggle("Damage effect fills your view (experimental)",
                   "Draws the red damage effect, and any effect added in the IK tuner's Full-view effects tab, across your view instead of the HUD window. Needs Native HUD on arm. Saved.",
                   Target::None, &w->damage_fx_full));
    add(ui, action("Next HUD shader test",
                   "Damage effect test while the screen is red. Stretches one candidate shader to the whole eye, then off after the last. Other HUD art may look wrong during the test. Not saved.",
                   &camera::hudfx_next));
    add(ui, action("Start or stop HUD draw sweep",
                   "Damage effect test while the screen is red. Stretches one HUD draw to the whole eye, moving on every 3 seconds. Stop it when the red fills your view. Other HUD art may look wrong during the test. Not saved.",
                   &camera::hudfx_sweep));
    add(ui, action("Next HUD draw", "Stops the sweep and moves one draw forward.", &camera::hudfx_draw_next));
    add(ui, action("Previous HUD draw", "Stops the sweep and moves one draw back. Before the first draw the test is off.", &camera::hudfx_draw_previous));
    add(ui, action(camera::hudfx_status(), "Report the number at which the red fills your whole view without other HUD art blowing up.", nullptr));
    add(ui, toggle("Wrist HUD (ammo + health)",
                   "Experimental. Shows the game's ammo counter and health bar on the forearm of the hand without the "
                   "weapon. Hide original ammo + health replaces the two source regions after valid image capture and XR submission.",
                   Target::None, &w->on));
    add(ui, slider("Wrist HUD size", "Width of the health panel on the wrist; the ammo panel keeps the same scale.",
                   Target::None, &w->size_m, 0.04f, 0.25f, 0.01f, "%.2f m"));
    add(ui, toggle("Panels follow the drawn arm", "Places the panels on the character's forearm instead of the controller. Falls back to the controller while mounted or when no arm was drawn recently.",
                   Target::None, &w->arm_anchor));
    add(ui, toggle("Wrist panels face you", "Keeps the panels facing the headset while their position follows the free wrist. Off: watch-face orientation.",
                   Target::None, &w->face_viewer));
    add(ui, toggle("Hide original ammo + health", "Removes the two crop regions from the flat HUD after nonempty texture capture and wrist submission. Any other art overlapping those regions is also removed.",
                   Target::None, &w->hide_original));
    add(ui, slider("Bottom band cut",
                   "Share of the HUD's height removed from the bottom of the HUD in front of you while the wrist HUD is "
                   "on (ammo and health sit there). 0 = keep everything. Only while the gameplay HUD is up.",
                   Target::None, &w->band, 0.f, 0.4f, 0.01f, "%.2f"));
    add(ui, toggle("Wrist HUD: show whole HUD",
                   "Diagnostic: the complete HUD picture on an opaque panel above the wrist, facing you. Reveals artwork even when source alpha is zero.",
                   Target::None, &w->full));
    add(ui, slider("Ammo crop left", "Left edge of the ammo crop (share of the HUD width).", Target::None,
                   &w->ammo.x, 0.f, 0.95f, 0.01f, "%.2f"));
    add(ui, slider("Ammo crop top", "Top edge of the ammo crop (share of the HUD height).", Target::None,
                   &w->ammo.y, 0.f, 0.95f, 0.01f, "%.2f"));
    add(ui, slider("Ammo crop width", "Width of the ammo crop.", Target::None, &w->ammo.w, 0.05f, 1.f, 0.01f, "%.2f"));
    add(ui, slider("Ammo crop height", "Height of the ammo crop.", Target::None, &w->ammo.h, 0.05f, 1.f, 0.01f, "%.2f"));
    add(ui, slider("Health crop left", "Left edge of the health crop.", Target::None, &w->health.x, 0.f, 0.95f, 0.01f,
                   "%.2f"));
    add(ui, slider("Health crop top", "Top edge of the health crop.", Target::None, &w->health.y, 0.f, 0.95f, 0.01f,
                   "%.2f"));
    add(ui, slider("Health crop width", "Width of the health crop.", Target::None, &w->health.w, 0.05f, 1.f, 0.01f,
                   "%.2f"));
    add(ui, slider("Health crop height", "Height of the health crop.", Target::None, &w->health.h, 0.05f, 1.f, 0.01f,
                   "%.2f"));
  }

  Tab& headset = g_tabs[3];
  headset.name = "Headset";
  headset.count = 0;
  add(headset, toggle("VR output", "Sends the game to the headset (OpenXR). Off closes the VR session.",
         Target::Xr, &g_s.xr));
  add(headset, toggle("Display mode", "Head-tracked: the eyes fill your view. Cinema: a 3D screen in front of you.",
         Target::Xr, &g_s.proj, "Cinema", "Head-tracked"));
  add(headset, toggle("Swap eyes", "Swaps the left and right images.", Target::Xr, &g_s.swap));
  add(headset, toggle("Frame submission",
         "Game rate (recommended): each game frame is sent to the headset once, so the headset software sees the "
         "real frame rate and can use its own frame synthesis (Virtual Desktop SSW). Every refresh: the last image "
         "is re-sent on every headset refresh (the headset then always reports full rate).",
         Target::Xr, &g_s.gamerate, "Every refresh", "Game rate"));
  add(headset, toggle("Frame generation (mod)",
         "Extra frames between game frames, made by the mod from the last frame and the game's motion vectors, so the "
         "headset gets its full refresh rate while the game renders fewer frames (best: headset 90 Hz, game 45-60 fps, "
         "Virtual Desktop SSW off). No added delay. The HUD and the edges of fast-moving objects can smear. Off = as before.",
         Target::Xr, &g_s.framegen));
  add(headset, toggle("Frame generation test: no motion",
         "Test for the dark flashes: generated frames repeat the last game frame without moving it forward. If the "
         "flashes stay with this on, the motion prediction is not their cause. Needs Frame generation (mod) on.",
         Target::Xr, &g_s.framegen_k0));
  add(headset, action(xr_runtime::framegen_status(),
         "Live state of the mod's frame generation: game frames and generated frames per second, and the GPU time of "
         "one generated frame pair. Read-only.", nullptr));
  add(headset, toggle("Send depth to headset",
         "For frame generation layers (VectorXR) and the headset runtime: each eye's depth is sent with its image, which "
         "keeps hands, gun and nearby objects steadier in generated frames. Small GPU cost. Off = as before.",
         Target::Xr, &g_s.xr_depth));
  // Removed 2026-10-07: "Eye timing", "Right/Left eye delay" (fallback only: each frame's pose is known exactly).
  add(headset, slider("Cinema screen width", "Width of the 3D screen in cinema mode.", Target::Xr, &g_s.screen_w,
         1.f, 10.f, 0.25f, "%.2f m"));
  add(headset, slider("Cinema screen distance", "Distance of the 3D screen in cinema mode.", Target::Xr,
         &g_s.screen_d, 1.f, 10.f, 0.25f, "%.2f m"));

  Tab& menu = g_tabs[4];
  menu.name = "Mod";
  menu.count = 0;
  add(menu, slider("Menu distance", "How far this panel floats in front of you.", Target::Menu, &g_s.menu_dist, 0.6f,
         3.f, 0.05f, "%.2f m"));
  add(menu, slider("Menu size", "Width of this panel.", Target::Menu, &g_s.menu_width, 0.6f, 2.f, 0.05f, "%.2f m"));
  add(menu, action("Recenter menu", "Moves this panel in front of where you are looking.", &act_recentre_menu));
  add(menu, toggle("Desktop mirror", "Also shows this menu on the monitor.", Target::Menu, &g_s.mirror));
  // Removed 2026-10-07 (answered, kept in code): "Zero eye distance (test)", "Frame lag test", "Foliage sweep",
  // "Plant constants probe", "Wind search".
  add(menu, toggle("Monitor preview",
                   "While the game window is taller or wider than the screen (VR render shapes), shows a scaled live "
                   "copy of the whole picture on the monitor. Click-through; the game keeps the input.",
                   Target::Menu, &g_s.preview));
  add(menu, toggle("Run in background",
                   "Keeps the game at full speed when its window loses focus (it otherwise drops to about 10 fps, "
                   "e.g. after clicking another window). Input still goes only to the focused window.",
                   Target::Menu, &g_s.keep_active));
  add(menu, action("Load working preset",
         "Native stereo + head tracking, matched projection, UI distance 2 m, head-tracked display, game-rate "
         "frame submission, VR eye shape at 2160 px.",
         &act_preset));
  add(menu, action("Capture eyes (F9)",
                   "Saves the left and right eye images of the next frame side by side to scripts\\captures\\eyes_<time>.jpg. "
                   "F9 does the same in game without opening the menu.",
                   &act_capture));
  add(menu, action("Capture motion (3 frames)",
                   "Closes this menu and saves three stereo eye pairs plus raw and repaired motion vectors and depth. "
                   "Use while moving a hand or gun near the view. Files go to scripts\\captures.", &act_mv_burst));
  // Removed 2026-10-09 (answered; code in research/debug_archive/2026-10-09_testers, notes in research/DEBUG_TESTERS.md):
  // "Character sweep: slots", "Same-point eye test", "VFX test: fire / smoke" (+ F8), "Capture motion vectors",
  // "Character buffer probe", "Frame counter search".
  add(menu, action("Close menu", "Same as F10.", &act_close));
}

void act_reload_scripts() { lua_host::request_reload(); }

// Scripts tab: fixed rows plus one row per script tunable (docs/SCRIPTING.md). Rebuilt when lua_host's rows change.
unsigned g_scripts_rows = 0;
void build_scripts_tab() {
  Tab& t = g_tabs[5];
  t.name = "Scripts";
  t.count = 0;
  add(t, action("Reload scripts (F7)",
                "Runs scripts\\avr\\autorun\\*.lua again from disk. Hooks and the game stay up; script errors show "
                "below and in scripts\\SCRIPTS.log.",
                &act_reload_scripts));
  add(t, toggle("Script writes",
                "Off: scripts can only read the game (avr.set_look, avr.set_bone, avr.write_* do nothing). On at launch "
                "(the arm IK and the tracked gun need it).",
                Target::None, lua_host::writes_flag()));
  // Arm IK rows, most used first (2026-10-15: fewer sliders; T-pose calibration sets the user measurements).
  add(t, toggle("First-person mounts",
                "On by default, saved. While mounting, riding and dismounting, the headset view stays on your head "
                "instead of the game's mount camera behind you; head tracking and controllers keep working. Off: the "
                "game's mount camera. The first-person arms are not drawn while mounted yet.", Target::None,
                mount_camera::first_person_flag()));
  add(t, toggle("Arms follow controllers while mounted",
                "On by default, saved. While mounted, the riding body's arms reach for the controllers (needs Arm IK "
                "and Script writes on). Off: the game's riding pose.", Target::None, mount_camera::mounted_arms_flag()));
  add(t, toggle("Weapons follow controllers while mounted",
                "Saved, on by default. Moves proven mounted weapon parts to the selected VR hand. "
                "Needs mounted arms, Script writes and Weapon follows tracked gun. Two-hand aiming and firing "
                "alignment still need mounted integration. Off restores the native mounted weapon placement.",
                Target::None, skin_palette_probe::mounted_weapon_flag()));
  add(t, action("Capture mount transitions (3 min)",
                "Read-only capture. Starts in stereo gameplay; mount, ride, aim and dismount, then repeat with the "
                "other mount if available. Saves camera pre/post, mount-state candidates, poses and nearby mesh "
                "palettes to scripts/MOUNT_CAMERA_*.trace. Stops automatically after three minutes.",
                &mount_camera::request_capture));
  add(t, action("Stop mount capture", "Stops a queued or active mount capture and keeps the samples already saved.",
                &mount_camera::cancel_capture));
  add(t, toggle("Arm IK (controllers)",
                "First-person arms follow the controllers. Needs Script writes on and gameplay. On by default, saved.",
                Target::None, skin_palette_probe::ik_menu_flag()));
  add(t, action("Calibrate arms (T-pose)",
                "Stand straight, arms straight out to the sides at shoulder height, then select. Measures your arm "
                "length and shoulder position so your reach maps onto the Na'vi arms; resets hand position offsets.",
                &skin_palette_probe::calibrate_arms));
  add(t, toggle("Model-space hands",
                "On (new): hands are placed relative to your headset and a body-fixed shoulder, like the Cyberpunk VR port, "
                "so locomotion, bob and turning cannot pull them. Off: previous world-space hands. Saved.",
                Target::None, skin_palette_probe::model_hands_flag()));
  add(t, toggle("Stabilize hands while moving",
                "On: removes the game's running/jumping head bob from your hands and gun so they stay steady for aiming. "
                "Saved.", Target::None, skin_palette_probe::stabilize_flag()));
  add(t, toggle("Arms follow every game tick",
                "On: when the game updates the gun twice between two displayed frames (often while turning), the arms are "
                "re-solved for the newer update so the gun hand stays on the grip. Saved.", Target::None,
                skin_palette_probe::tick_resolve_flag()));
  add(t, toggle("VR controls",
                "On: VR gestures and buttons. Reload = the gun hand's upper button (B right, Y left); flick the right stick "
                "up to jump, down to crouch; hand over a shoulder + grip tap = equip into that hand / holster, hold = "
                "weapon wheel (left stick selects); empty hand at your head + grip = scan; the free trigger does nothing. "
                "A, X and the free hand's upper button keep their game use. Game menus always get the default "
                "controls. Off: plain gamepad mapping. Saved.", Target::None, vr_controls::enabled_flag()));
  add(t, toggle("Two-hand aiming",
                "On: grab the gun's front grip with your free hand's grip button and aim with both hands; rifles follow the "
                "line between your hands, pistols are steadied. While both hands hold it, the trigger hand's grip lets go (the gun stays in the "
                "support hand, no firing) and grips the handle again to re-grab. Grabbing near the trigger hand still swaps hands. Saved.",
                Target::None, skin_palette_probe::two_hand_flag()));
  add(t, toggle("Gesture haptics", "A short vibration when a hand enters a gesture zone (shoulder, head). Saved.",
                Target::None, vr_controls::haptics_flag()));
  add(t, toggle("Movement direction",
                "Which way the left stick walks. Game: relative to your body (the game's own facing). Head: forward is "
                "where you look with the headset. Saved.", Target::None, move_probe::head_move_flag(), "Game", "Head"));
  add(t, toggle("Body turns with head",
                "On: when you turn your head more than 15 degrees away from your body, the body and arms turn to follow "
                "while the view stays still. Off: the body only turns with the stick. Saved.", Target::None,
                move_probe::body_turn_flag()));
  add(t, toggle("Weapon hand",
                "Which hand holds the gun. Left: the gun sits in the left controller, shots and aim follow it and the "
                "triggers swap (left trigger fires). Saved.", Target::None, skin_palette_probe::left_handed_flag(),
                "Right", "Left"));
  add(t, toggle("Clean arm rig",
                "On: both hands and the gun come from one controller sample and one target per game frame, and the arms "
                "never drop to the game's pose for a frame. Off: previous arm code (for comparison). Saved.",
                Target::None, skin_palette_probe::clean_rig_flag()));
  add(t, toggle("Shoulders follow view height",
                "On: the shoulders (and the hands and gun) move with the view's height, so crouching and running keep "
                "the arms with you. Saved.", Target::None, skin_palette_probe::view_height_flag()));
  add(t, toggle("Freeze hand animation while moving",
                "On: while you move, jump or turn, the game's hand animation cannot steer your wrists, fingers, gun or "
                "two-hand hold; it resumes when you stand still. Off: the hand keeps following it. Saved.",
                Target::None, skin_palette_probe::hand_freeze_flag()));
  add(t, toggle("Lock arms to view",
                "On: the shoulders, hands and gun use the same camera position as your eyes, with or without a weapon, so "
                "walking, running and jumping do not sway them in your view. Off: previous smoothed anchor. Saved.",
                Target::None, skin_palette_probe::arms_view_lock_flag()));
  add(t, toggle("Gun hand follows shoulder and controller",
                "On: while holding a gun, the gun (and the hand on it) moves with this frame's shoulder and your latest "
                "controller motion, like the free hand, so it does not lag while moving. Saved.", Target::None,
                skin_palette_probe::shoulder_socket_flag()));
  add(t, toggle("Body turn in step",
                "On: when you turn, the arms, gear (bracelets, gloves) and the gun all use the body's current turn, so the "
                "hand stays on the grip. Saved.", Target::None, skin_palette_probe::root_carry_flag()));
  add(t, toggle("Arms in step with view",
                "On: your controllers are placed in the camera of the frame being drawn, so hands, shoulders and the gun "
                "stay together while moving. Off: previous behaviour. Saved.", Target::None, skin_palette_probe::rig_lock_flag()));
  add(t, slider("Weapon hand trail fix", "How rigidly the gun and hand follow your view while moving, running and "
                "turning (0 = old lag, 1 = locked to the view). Saved.", Target::None, skin_palette_probe::weapon_trail_value(), 0.f, 1.f, 0.05f, "%.2f"));
  add(t, slider("Reach multiplier", "Fine-tunes how far the Na'vi arms reach for your reach (1.00 = calibrated).", Target::None,
                skin_palette_probe::reach_multiplier_value(), 0.70f, 1.50f, 0.01f, "%.2f"));
  add(t, slider("Shoulder height", "Raises (+) or lowers (-) the Na'vi shoulders, metres.", Target::None,
                skin_palette_probe::shoulder_height_value(), -0.4f, 0.4f, 0.01f, "%.2f"));
  add(t, slider("Shoulder forward", "Moves the Na'vi shoulders forward (+) or back (-), metres.", Target::None,
                skin_palette_probe::shoulder_forward_value(), -0.4f, 0.4f, 0.01f, "%.2f"));
  add(t, slider("Hand separation", "Moves both hands apart (+) or together (-), metres. Fixes crossing hands.", Target::None,
                skin_palette_probe::hand_separation_value(), -0.3f, 0.3f, 0.01f, "%.2f"));
  add(t, slider("Shoulder separation", "Moves the Na'vi arms' roots apart (+) or together (-) at the shoulders, metres.",
                Target::None, skin_palette_probe::shoulder_spread_value(), -0.3f, 0.3f, 0.01f, "%.2f"));
  add(t, slider("Hand pitch", "Tilts the hands up (+) or down (-) relative to the controllers, degrees.", Target::None,
                &skin_palette_probe::tune()->pitch, -90.f, 90.f, 5.f, "%.0f"));
  add(t, slider("Hand yaw", "Turns the hands right (+) or left (-) relative to the controllers, degrees.", Target::None,
                &skin_palette_probe::tune()->yaw, -90.f, 90.f, 5.f, "%.0f"));
  add(t, slider("Hand roll", "Rolls the hands around the controller's forward axis, degrees.", Target::None,
                &skin_palette_probe::tune()->roll, -180.f, 180.f, 5.f, "%.0f"));
  add(t, action("Reset hand tuning", "Hand pitch/yaw/roll back to the tuned defaults (left hand mirrored).",
                &skin_palette_probe::reset_tune));
  add(t, action("Recapture rest pose", "Stand still: captures a new rest pose for the shoulders within half a second.",
                &skin_palette_probe::recapture_rest));
  add(t, toggle("Arm scaling (Na'vi reach)",
                "On: your reach from your shoulder is scaled onto the Na'vi arm (calibrate with the T-pose).",
                Target::None, skin_palette_probe::arm_scale_flag()));
  add(t, toggle("Stable shoulders (rest pose)",
                "On: the arms hang from a rest pose captured while you stand still, so running and weapon animation "
                "no longer move the shoulders against your view.",
                Target::None, skin_palette_probe::stable_flag()));
  add(t, toggle("Empty-hand gestures",
                "On by default; your last choice is remembered. Touch/press controls curl empty-hand fingers. Free weapon offhand and "
                "released mounted hands use relaxed idle; held/support/harness hands keep native poses. Needs Arm IK "
                "and Script writes. Unknown items/camera states retain native animation. Turn off to restore native hands.",
                Target::None, hand_gestures::enabled_flag()));
  add(t, toggle("Native hand poses",
                "On: the hands play the game's own animation (weapon grips, reload, gestures). Off: the hands keep the "
                "pose captured with the rest pose (frozen; for future controller finger gestures).",
                Target::None, skin_palette_probe::native_hands_flag()));
  add(t, toggle("Shoulders follow headset",
                "On: the arms and shoulders move with your real head position (leaning, crouching).",
                Target::None, skin_palette_probe::shoulders_flag()));
  add(t, toggle("Forearm twist",
                "On: the wrist's roll is spread along the forearm instead of twisting the skin at the wrist.",
                Target::None, skin_palette_probe::twist_flag()));
  add(t, toggle("Shoot from tracked gun",
                "On: shots and aim leave the gun in your IK hand and go where its barrel points. Needs Script writes; "
                "keep \"aim: independent view\" off. On by default.",
                Target::None, vr_weapon::shoot_from_tracked_flag()));
  add(t, toggle("Weapon follows tracked gun",
                "On: the game's own weapon (all parts, muzzle flash, tracer, muzzle ring) is placed in your IK hand. "
                "Needs Script writes. On by default.",
                Target::None, vr_weapon::effects_flag()));
  add(t, toggle("Accessory physics",
                "On: charms and cords on the arms swing a little behind your arm movement (a spring). Off: they stay "
                "rigidly attached.",
                Target::None, skin_palette_probe::accessory_physics_flag()));
  add(t, toggle("Hand smoothing",
                "On: removes small hand jitter (One-Euro filter: smooths slow movements, almost no lag on fast ones).",
                Target::None, skin_palette_probe::smoothing_flag()));
  add(t, toggle("Steady hands while moving",
                "On: the hand targets follow the body's movement within the frame, removing jitter while walking.",
                Target::None, skin_palette_probe::anchor_flag()));
  add(t, toggle("Arm projection fix",
                "On: the first-person arms are drawn with each eye's headset projection instead of the game's fixed "
                "64 degree first-person projection. Off: game's own (for comparison).",
                Target::None, native_stereo::fp_proj_menu_flag()));
  add(t, toggle("Arm depth occlusion",
                "On: plants, rocks and creatures in front of your arms hide them (the game normally draws the "
                "first-person arms over everything). Off: game's own behaviour.",
                Target::None, native_stereo::fp_occlude_menu_flag()));
  add(t, action(lua_host::status(), "Script status: loaded scripts, running callbacks, the last error.", nullptr));
  for (int i = 0; i < lua_host::tunable_count(); ++i) {
    lua_host::Tunable* tu = lua_host::tunable(i);
    if (tu == nullptr) {
      break;
    }
    if (tu->toggle) {
      add(t, toggle(tu->name, "Script tunable (avr.toggle). Kept in scripts\\avr\\tunables.txt.", Target::None,
                    &tu->on));
    } else {
      add(t, slider(tu->name, "Script tunable (avr.tunable). Kept in scripts\\avr\\tunables.txt.", Target::None,
                    &tu->value, tu->lo, tu->hi, tu->step > 0.f ? tu->step : 0.01f, "%.3f"));
    }
  }
  g_scripts_rows = lua_host::rows_version();
  if (g_tab == 5 && g_sel >= t.count) {
    g_sel = t.count - 1;
  }
}

bool g_visible = false;
unsigned g_buf_w = 0, g_buf_h = 0;  // game swapchain buffer size = each eye's render size (Present thread)
bool g_dirty_native = false, g_dirty_xr = false, g_dirty_shape = false;
ULONGLONG g_shape_changed_at = 0;
RECT g_eye_resize_window = {};
ULONGLONG g_eye_resize_restore_at = 0, g_eye_resize_check_at = 0;
unsigned g_eye_resize_attempts = 0;
bool g_eye_resize_zoomed = false;

void act_recentre_view() {
  native_stereo::recentre();
  xr_runtime::recentre_overlay();
  log_line("recentre view");
}

void act_recentre_menu() { xr_runtime::recentre_overlay(); }

void act_recentre_main_menu() {
  native_stereo::recentre_menu_panel();
  log_line("recentre main menu panel");
}

void act_preset() {
  g_s.stereo = true;
  g_s.autostart = true;
  g_s.head = true;
  g_s.fov = 128.f;
  g_s.offx = 0.f;
  g_s.offy = 0.17f;
  g_s.cullfov = 0.f;
  g_s.autofov = true;
  g_s.eyeshift = false;
  g_s.ui_depth = 2.f;
  g_s.margin = 0.f;
  g_s.menus_mono = true;
  g_s.menu_stereo = true;
  g_s.eye_ui = true;
  g_s.ui_place = true;
  g_s.level = true;
  g_s.vegsync = true;
  if (g_s.shape != kShapeFlatEyes || g_s.render_h != 2160.f || g_s.eye_scale != 0.f) {
    g_s.shape = kShapeFlatEyes;
    g_s.render_h = 2160.f;
    g_s.eye_scale = 0.f;
    g_dirty_shape = true;
    g_shape_changed_at = GetTickCount64();
  }
  g_s.pair = true;
  g_s.gamerate = true;
  g_s.xr = true;
  g_s.proj = true;
  g_s.swap = false;
  g_s.lag = 1;
  g_s.lagl = 2;
  g_dirty_native = g_dirty_xr = true;
  log_line("preset: working config");
}

void act_close() { g_visible = false; }

void act_capture() {
  g_visible = false;  // the capture is taken at the next Present; close the menu so it is not in the way
  frame_capture::request_eyes();
  log_line("menu: eye capture requested");
}

void act_mv_burst() {
  g_visible = false;
  log_line("menu: motion burst %s", dlss_twin::request_mv_burst() ? "requested" : "busy");
}

// One left/right (dir = -1/+1) or Enter (dir = 0) on a row.
void change(Row& r, int dir) {
  switch (r.kind) {
    case Kind::Toggle:
      *r.b = dir == 0 ? !*r.b : dir > 0;
      break;
    case Kind::Choice:
      *r.i = (*r.i + (dir == 0 ? 1 : dir) + r.choice_count) % r.choice_count;
      break;
    case Kind::Int: {
      const int v = *r.i + dir;
      *r.i = std::clamp(v, static_cast<int>(r.lo), static_cast<int>(r.hi));
      break;
    }
    case Kind::Float: {
      if (dir == 0) {
        return;
      }
      float v = *r.f;
      if (v < r.zero_below) {
        v = dir > 0 ? r.zero_below : 0.f;  // from "Game"/"Off" the first step right lands on the lowest real value
      } else {
        v = std::round((v + dir * r.step) / r.step) * r.step;
        if (v < r.zero_below - 1e-4f) {
          v = 0.f;
        } else {
          v = std::clamp(v, r.lo, r.hi);
        }
      }
      if (std::fabs(v) < 1e-6f) {
        v = 0.f;
      }
      *r.f = v;
      break;
    }
    case Kind::Action:
      if (dir == 0 && r.action != nullptr) {
        r.action();
      }
      return;
  }
  if (r.target == Target::Native) {
    g_dirty_native = true;
    if ((r.f == &g_s.narrow || r.f == &g_s.narrow_v || r.b == &g_s.autofov) && g_s.shape != 0) {
      g_dirty_shape = true;  // the narrowed frustum's aspect is the VR shape's aspect
      g_shape_changed_at = GetTickCount64();
    }
  } else if (r.target == Target::Xr) {
    g_dirty_xr = true;
  } else if (r.target == Target::Shape) {
    g_dirty_shape = true;
    g_dirty_native = true;  // Flat + VR eyes: the eye size lives in AvatarVR_native.txt
    g_shape_changed_at = GetTickCount64();  // applied once the value has rested (a held key would resize every step)
  }
}

// ---- render shape (game window size) --------------------------------------------------------------------

std::atomic<bool> g_allow_big{false};  // WM_GETMINMAXINFO: let the window be larger than the screen
// The window size a VR render shape asked for (outer size, 0 = none). The game widens its window on its own after some
// size changes (run 2026-10-05 13:49-13:51: 2160 high came out 1992, 2750 or 2963 wide; 3708x2240 once), which renders
// up to 50% more pixels per eye at the same height. WM_WINDOWPOSCHANGING holds the window at this size.
std::atomic<int> g_hold_w{0}, g_hold_h{0};
std::atomic<std::uint64_t> g_hold_fixes{0};
HWND g_shape_hwnd = nullptr;
RECT g_orig_rect = {};
bool g_orig_saved = false, g_orig_zoomed = false;
int g_applied_shape = 0;
bool g_applied_square = false;
float g_applied_h = 0.f;
float g_applied_aspect = 0.f;  // VR shape aspect last placed (follows the narrowed matched projection)
float g_applied_scale = 0.f;

// Target aspect (width / height) of one headset eye, or of both eyes' union, from the runtime's per-eye FOV.
float shape_aspect(int shape) {
  head_pose::Snapshot snap;
  float fallback = shape == 1 ? 0.93f : 1.15f;
  if (!xr_runtime::latest_views(&snap)) {
    return fallback;
  }
  float hl = 0.f, hr = 0.f, vu = 0.f, vd = 0.f, w1 = 0.f;
  for (int e = 0; e < 2; ++e) {
    const float* f = snap.eye[e].fov;
    hl = std::max(hl, std::tan(-f[0]));
    hr = std::max(hr, std::tan(f[1]));
    vu = std::max(vu, std::tan(f[2]));
    vd = std::max(vd, std::tan(-f[3]));
    w1 = std::max(w1, std::tan(-f[0]) + std::tan(f[1]));
  }
  const float v = vu + vd;
  if (v < 0.2f) {
    return fallback;
  }
  float a = shape == 1 ? w1 / v : 2.f * std::max(hl, hr) / v;
  if (g_s.autofov) {
    // narrowed matched projection (native_stereo auto_projection): the width scales by kh, the height by kv
    const float kh = g_s.narrow >= 40.f && g_s.narrow < 99.5f ? g_s.narrow * 0.01f : 1.f;
    const float kv = std::min(1.f, kh * std::clamp(g_s.narrow_v, 50.f, 150.f) * 0.01f);
    a *= kh / kv;
  }
  return a > 0.4f && a < 3.f ? a : fallback;
}

// Resize the game window so its client area is cw x ch (may exceed the monitor; the hold keeps the game from
// shrinking it back). Returns false when the monitor could not be read.
bool place_client(HWND hwnd, int cw, int ch, const char* why) {
  MONITORINFO mi = {sizeof(mi)};
  if (!GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &mi) || cw < 320 || ch < 240) {
    return false;
  }
  const DWORD style = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_STYLE));
  const DWORD exstyle = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_EXSTYLE));
  RECT frame = {0, 0, 0, 0};
  AdjustWindowRectEx(&frame, style, FALSE, exstyle);  // frame.left/top negative, right/bottom positive
  const int work_w = mi.rcWork.right - mi.rcWork.left;
  const int work_h = mi.rcWork.bottom - mi.rcWork.top;
  const int ww = cw + (frame.right - frame.left), wh = ch + (frame.bottom - frame.top);
  g_allow_big.store(ww > work_w || wh > work_h);
  if (IsZoomed(hwnd)) {
    ShowWindowAsync(hwnd, SW_RESTORE);
  }
  const int x = mi.rcWork.left + (ww <= work_w ? (work_w - ww) / 2 : 0);
  const int y = mi.rcWork.top + (wh <= work_h ? (work_h - wh) / 2 : 0);
  g_hold_h.store(wh);
  g_hold_w.store(ww);
  SetWindowPos(hwnd, nullptr, x, y, ww, wh, SWP_ASYNCWINDOWPOS | SWP_NOZORDER | SWP_NOACTIVATE);
  log_line("render shape (%s): client %dx%d (window %dx%d at %d,%d)", why, cw, ch, ww, wh, x, y);
  return true;
}

// Client size for a VR shape (1 = one eye, 2 = both eyes) or, for shape -1, the wide 16:9 menu shape that fits the
// work area. False when the monitor could not be read.
bool shape_client(HWND hwnd, int shape, int* cw, int* ch) {
  MONITORINFO mi = {sizeof(mi)};
  if (!GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &mi)) {
    return false;
  }
  const DWORD style = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_STYLE));
  const DWORD exstyle = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_EXSTYLE));
  RECT frame = {0, 0, 0, 0};
  AdjustWindowRectEx(&frame, style, FALSE, exstyle);
  const int avail_w = (mi.rcWork.right - mi.rcWork.left) - (frame.right - frame.left);
  const int avail_h = (mi.rcWork.bottom - mi.rcWork.top) - (frame.bottom - frame.top);
  const float aspect = shape < 0 ? 16.f / 9.f : shape_aspect(shape);
  if (shape > 0 && g_s.render_h >= 1200.f) {
    *ch = static_cast<int>(g_s.render_h) & ~7;
    *cw = static_cast<int>(static_cast<float>(*ch) * aspect) & ~7;
    return true;
  }
  *ch = avail_h & ~7;
  *cw = static_cast<int>(static_cast<float>(*ch) * aspect) & ~7;
  if (*cw > avail_w) {
    *cw = avail_w & ~7;
    *ch = static_cast<int>(static_cast<float>(*cw) / aspect) & ~7;
  }
  return true;
}

bool g_menu_shape = false;  // the wide menu shape is on the window instead of the VR shape (stereo paused)

// [LIVE] run 2026-10-07 06:23 (d400f87c / 3d843878): Render height 2400 placed a 2216x2400 client, yet the game's next
// ResizeBuffers came with client 2856x2400 and later 3176x2400, with no WM_WINDOWPOSCHANGING held at first (the hold
// count was 0). The game rendered 29-43% more width than the eye shape; the projection widens to the render aspect, so
// those pixels fall outside the lens and only cost frame time. Backstop: when the swapchain stays off the held VR
// shape for 1 s in a VR shape, the window is placed again (at most every 3 s).
std::atomic<std::uint64_t> g_shape_restores{0};
void enforce_shape(HWND hwnd, UINT bw, UINT bh) {
  static ULONGLONG off_since = 0, last_fix = 0;
  static int tries = 0, tries_for_w = 0, tries_for_h = 0;  // re-placements for the current held shape
  const int hw = g_hold_w.load(std::memory_order_relaxed), hh = g_hold_h.load(std::memory_order_relaxed);
  if (hw != tries_for_w || hh != tries_for_h) {
    tries = 0;
    tries_for_w = hw;
    tries_for_h = hh;
  }
  if (hwnd == nullptr || hwnd != g_shape_hwnd || hw <= 0 || g_menu_shape || !forced_shape(g_applied_shape) || IsIconic(hwnd)) {
    off_since = 0;
    return;
  }
  const DWORD style = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_STYLE));
  const DWORD exstyle = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_EXSTYLE));
  RECT frame = {0, 0, 0, 0};
  AdjustWindowRectEx(&frame, style, FALSE, exstyle);
  const int cw = hw - (frame.right - frame.left), ch = hh - (frame.bottom - frame.top);
  const bool off = std::abs(static_cast<int>(bw) - cw) > 16 || std::abs(static_cast<int>(bh) - ch) > 16;
  const ULONGLONG now = GetTickCount64();
  if (!off) {
    off_since = 0;
    tries = 0;
    return;
  }
  if (tries >= 3) {
    return;  // each re-placement pauses stereo for the resize; give up rather than loop (logged once below)
  }
  if (off_since == 0) {
    off_since = now;
    return;
  }
  if (now - off_since < 1000 || now - last_fix < 3000) {
    return;
  }
  RECT wr = {};
  GetWindowRect(hwnd, &wr);
  native_stereo::hold_for_resize(3000);  // the swapchain resize fails while stereo holds its frames
  SetWindowPos(hwnd, nullptr, wr.left, wr.top, hw, hh, SWP_ASYNCWINDOWPOS | SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOMOVE);
  g_shape_restores.fetch_add(1, std::memory_order_relaxed);
  log_line("render shape: buffers %ux%u drifted off the VR shape %dx%d; window placed again (%llu so far)", bw, bh, cw, ch,
           static_cast<unsigned long long>(g_shape_restores.load()));
  last_fix = now;
  off_since = 0;
  if (++tries == 3) {
    log_line("render shape: the game keeps the window off the VR shape at %dx%d; no more re-placements for this size", cw, ch);
  }
}

void act_small_window() {
  HWND hwnd = g_shape_hwnd;
  if (hwnd == nullptr) return;
  native_stereo::hold_for_resize(3000);
  g_monitor_bypass_until.store(GetTickCount64() + 3000);
  g_hold_w.store(0); g_hold_h.store(0); g_allow_big.store(false);
  ShowWindowAsync(hwnd, SW_RESTORE);
  RECT r = {0, 0, 1280, 720};
  AdjustWindowRectEx(&r, static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_STYLE)), FALSE,
                    static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_EXSTYLE)));
  SetWindowPos(hwnd, nullptr, 0, 0, r.right-r.left, r.bottom-r.top, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
  g_s.monitor_keep = true;
  g_monitor_capture.store(true);
  log_line("monitor: small window requested (1280x720), eye size unchanged");
}

// Keep the changed window size across real game frames. Two SetWindowPos calls
// in one Present can be coalesced, leaving the old buffers and a cropped eye.
void start_eye_resize() {
  HWND hwnd = g_shape_hwnd;
  if (g_eye_resize_restore_at != 0) return;  // retain the original size across rapid slider changes
  g_eye_resize_zoomed = hwnd != nullptr && IsZoomed(hwnd);
  if (g_eye_resize_zoomed) ShowWindow(hwnd, SW_RESTORE);
  if (hwnd == nullptr || !GetWindowRect(hwnd, &g_eye_resize_window)) return;
  native_stereo::hold_for_resize(3000);
  g_monitor_bypass_until.store(GetTickCount64() + 3000);
  SetWindowPos(hwnd, nullptr, 0, 0, g_eye_resize_window.right - g_eye_resize_window.left,
               g_eye_resize_window.bottom - g_eye_resize_window.top - 8,
               SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
  g_eye_resize_restore_at = GetTickCount64() + 800;
  g_eye_resize_check_at = 0;
  log_line("eye resize: window nudge started, attempt %u", g_eye_resize_attempts);
}

void update_eye_resize() {
  const ULONGLONG now = GetTickCount64();
  if (g_eye_resize_restore_at != 0 && now >= g_eye_resize_restore_at) {
    SetWindowPos(g_shape_hwnd, nullptr, 0, 0, g_eye_resize_window.right - g_eye_resize_window.left,
                 g_eye_resize_window.bottom - g_eye_resize_window.top,
                 SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
    if (g_eye_resize_zoomed) ShowWindow(g_shape_hwnd, SW_MAXIMIZE);
    g_eye_resize_restore_at = 0;
    g_eye_resize_check_at = now + 1000;
  }
  if (g_eye_resize_check_at == 0 || now < g_eye_resize_check_at) return;
  int ew = 0, eh = 0;
  if (g_s.shape != kShapeFlatEyes || !eye_render_size(&ew, &eh)) {
    g_eye_resize_check_at = 0;
    return;
  }
  if (g_buf_w >= static_cast<unsigned>(ew) && g_buf_h >= static_cast<unsigned>(eh)) {
    log_line("eye resize: verified buffers %ux%u cover eyes %dx%d", g_buf_w, g_buf_h, ew, eh);
    g_eye_resize_check_at = 0;
  } else if (g_eye_resize_attempts < 3) {
    ++g_eye_resize_attempts;
    start_eye_resize();
  } else {
    log_line("eye resize: FAILED buffers %ux%u need %dx%d; cropped stereo submission blocked", g_buf_w, g_buf_h, ew, eh);
    g_eye_resize_check_at = 0;
  }
}

void update_monitor_window() {
  HWND hwnd = g_shape_hwnd;
  if (hwnd == nullptr) return;
  g_monitor_keep.store(g_s.monitor_keep && g_s.shape == kShapeFlatEyes);
  if (g_monitor_capture.exchange(false) && !IsIconic(hwnd) && !IsZoomed(hwnd)) {
    RECT r = {};
    if (GetWindowRect(hwnd, &r)) {
      g_monitor_w.store(r.right-r.left); g_monitor_h.store(r.bottom-r.top);
      g_monitor_x.store(r.left); g_monitor_y.store(r.top);
      g_monitor_frame.store(static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_STYLE)) & kMonitorFrameMask);
      log_line("monitor: saved window %dx%d at %d,%d frame=0x%lX", r.right-r.left, r.bottom-r.top, r.left, r.top,
               static_cast<unsigned long>(g_monitor_frame.load()));
    }
  }
  static ULONGLONG last_restore = 0;
  const ULONGLONG now = GetTickCount64();
  if (!g_monitor_keep.load() || g_monitor_w.load() == 0 || g_monitor_drag.load() || IsIconic(hwnd) ||
      now < g_monitor_bypass_until.load() || now-last_restore < 1000) return;
  RECT r = {};
  if (!GetWindowRect(hwnd, &r)) return;
  const DWORD style = static_cast<DWORD>(GetWindowLongPtrW(hwnd, GWL_STYLE));
  const bool frame_changed = (style & kMonitorFrameMask) != g_monitor_frame.load();
  const bool moved = r.left != g_monitor_x.load() || r.top != g_monitor_y.load();
  const bool resized = r.right-r.left != g_monitor_w.load() || r.bottom-r.top != g_monitor_h.load() || IsZoomed(hwnd);
  if (!moved && !resized && !frame_changed) return;
  last_restore = now;
  if (resized || frame_changed) native_stereo::hold_for_resize(3000);
  g_monitor_bypass_until.store(now+3000);
  if (IsZoomed(hwnd)) ShowWindow(hwnd, SW_RESTORE);
  if (frame_changed) SetWindowLongPtrW(hwnd, GWL_STYLE, (style & ~kMonitorFrameMask) | g_monitor_frame.load());
  SetWindowPos(hwnd, nullptr, g_monitor_x.load(), g_monitor_y.load(), g_monitor_w.load(), g_monitor_h.load(),
               SWP_NOZORDER | SWP_NOACTIVATE | (frame_changed ? SWP_FRAMECHANGED : 0));
  log_line("monitor: restored saved placement and border");
}

void apply_shape() {
  HWND hwnd = g_shape_hwnd;
  int eye_w = 0, eye_h = 0;
  const bool have_eye_size = eye_render_size(&eye_w, &eye_h);
  const float aspect = g_s.shape == kShapeFlatEyes && have_eye_size ? static_cast<float>(eye_w) / eye_h :
                       g_s.shape > 0 ? shape_aspect(g_s.shape == kShapeFlatEyes ? 1 : g_s.shape) : 0.f;
  const float applied_h = g_s.shape == kShapeFlatEyes && have_eye_size ? static_cast<float>(eye_h) : g_s.render_h;
  if (hwnd == nullptr || (g_s.shape == g_applied_shape && g_s.square_eyes == g_applied_square &&
                          g_s.eye_scale == g_applied_scale &&
                          (g_s.shape == 0 || (applied_h == g_applied_h && std::fabs(aspect - g_applied_aspect) < 0.005f)))) {
    return;
  }
  if (g_s.shape == kShapeFlatEyes) {
    apply_native();  // the eye size
    if (forced_shape(g_applied_shape)) {
      // leaving a VR window shape: give the game its own window back first
      g_hold_w.store(0);
      g_hold_h.store(0);
      g_allow_big.store(false);
      g_menu_shape = false;
      if (g_orig_saved) {
        SetWindowPos(hwnd, nullptr, g_orig_rect.left, g_orig_rect.top, g_orig_rect.right - g_orig_rect.left,
                     g_orig_rect.bottom - g_orig_rect.top, SWP_ASYNCWINDOWPOS | SWP_NOZORDER | SWP_NOACTIVATE);
      }
      if (g_orig_zoomed) {
        ShowWindowAsync(hwnd, SW_MAXIMIZE);
      }
    } else {
      g_eye_resize_attempts = 1;
      start_eye_resize();
    }
    int ew = 0, eh = 0;
    eye_render_size(&ew, &eh);
    log_line("render shape (Flat + VR eyes): window left to the game, eyes %dx%d", ew, eh);
  } else if (forced_shape(g_s.shape) && !forced_shape(g_applied_shape)) {
    g_orig_zoomed = IsZoomed(hwnd) != FALSE;
    g_orig_saved = GetWindowRect(hwnd, &g_orig_rect) != FALSE;
  }
  if (g_s.shape == kShapeFlatEyes) {
  } else if (g_s.shape == 0) {
    g_hold_w.store(0);  // release the hold before restoring the game's own size
    g_hold_h.store(0);
    g_allow_big.store(false);
    g_menu_shape = false;
    if (g_orig_saved) {
      SetWindowPos(hwnd, nullptr, g_orig_rect.left, g_orig_rect.top, g_orig_rect.right - g_orig_rect.left,
                   g_orig_rect.bottom - g_orig_rect.top, SWP_ASYNCWINDOWPOS | SWP_NOZORDER | SWP_NOACTIVATE);
    }
    if (g_orig_zoomed) {
      ShowWindowAsync(hwnd, SW_MAXIMIZE);
    }
    log_line("render shape: restored the game window");
  } else if (!g_menu_shape) {
    // The second view's output follows the backbuffer size without a rebuild (run 2026-10-05 13:48: same view, output
    // texture 2564x1421 -> 1248x1352 -> 2072x2240 -> 2963x2160 ... within about 2 s of each resize), so native stereo
    // is no longer switched off and on around a resize.
    int cw = 0, ch = 0;
    if (!shape_client(hwnd, g_s.shape, &cw, &ch) || !place_client(hwnd, cw, ch, g_s.shape == 1 ? "VR eye" : "VR wide")) {
      return;
    }
  }
  g_applied_shape = g_s.shape;
  g_applied_square = g_s.square_eyes;
  g_applied_h = applied_h;
  g_applied_aspect = aspect;
  g_applied_scale = g_s.eye_scale;
  char text[64];
  snprintf(text, sizeof(text), "shape %d height %.0f square %d scale %.0f", g_s.shape, g_s.render_h,
           g_s.square_eyes ? 1 : 0, g_s.eye_scale);
  if (write_text(g_dir + L"\\AvatarVR_shape.txt", text)) {
    log_line("shape <- %s", text);
  }
}

// Present thread: while stereo is paused (start-up video, pause / inventory / character menus) the game window takes a
// wide 16:9 shape so menus fill the cinema screen; in gameplay it goes back to the VR shape. The second view follows
// the size by itself, and the headset shows the mono cinema screen until both eyes match again.
void update_menu_shape(ULONGLONG now) {
  static ULONGLONG paused_since = 0, last_switch = 0, resize_until = 0;
  HWND hwnd = g_shape_hwnd;
  const bool active = hwnd != nullptr && g_s.wide_menus && g_s.stereo && g_s.xr && forced_shape(g_applied_shape);
  // Our own resize pause (hold_for_resize) is not a menu.
  const bool running = native_stereo::stereo_running() || now < resize_until;
  if (running || !active) {
    paused_since = 0;
  } else if (paused_since == 0) {
    paused_since = now;
  }
  // Debounced: wide only after 0.6 s paused (menus and loading flicker), at most one switch per 1.5 s.
  const bool want_menu = active && paused_since != 0 && now - paused_since > 600;
  if (want_menu == g_menu_shape || now - last_switch < 1500) {
    return;
  }
  int cw = 0, ch = 0;
  native_stereo::hold_for_resize(3000);  // the swapchain resize fails while stereo holds its frames
  resize_until = now + 3500;
  if (hwnd == nullptr || !shape_client(hwnd, want_menu ? -1 : g_applied_shape, &cw, &ch) ||
      !place_client(hwnd, cw, ch, want_menu ? "menu, stereo paused" : "gameplay, stereo")) {
    return;
  }
  g_menu_shape = want_menu;
  last_switch = now;
}

// ---- input ----------------------------------------------------------------------------------------------

HWND g_hwnd = nullptr;
WNDPROC g_prev_wndproc = nullptr;
std::atomic<bool> g_swallow{false};
std::atomic<bool> g_keep_active{true};
std::atomic<uint64_t> g_hidden_deactivations{0};

bool is_nav_key(WPARAM vk) {
  return vk == VK_UP || vk == VK_DOWN || vk == VK_LEFT || vk == VK_RIGHT || vk == VK_RETURN || vk == VK_PRIOR || vk == VK_NEXT;
}

LRESULT CALLBACK menu_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  if (msg == WM_ENTERSIZEMOVE) g_monitor_drag.store(true);
  if (msg == WM_EXITSIZEMOVE) {
    const LRESULT result = CallWindowProcW(g_prev_wndproc, hwnd, msg, wp, lp);
    g_monitor_drag.store(false);
    g_monitor_capture.store(true);
    return result;
  }
  if (msg == WM_WINDOWPOSCHANGING && g_monitor_keep.load() && g_monitor_w.load() > 0 &&
      !g_monitor_drag.load() && GetTickCount64() >= g_monitor_bypass_until.load() && !IsIconic(hwnd)) {
    const LRESULT result = CallWindowProcW(g_prev_wndproc, hwnd, msg, wp, lp);
    WINDOWPOS* pos = reinterpret_cast<WINDOWPOS*>(lp);
    if (pos != nullptr && !(pos->flags & SWP_NOSIZE)) {
      pos->cx = g_monitor_w.load(); pos->cy = g_monitor_h.load();
    }
    return result;
  }
  if (msg == WM_WINDOWPOSCHANGING && g_hold_w.load(std::memory_order_relaxed) > 0) {
    // The game's own handler runs FIRST and the hold is applied to what it leaves. [LIVE] run 2026-10-07 06:42
    // (d028ec52): with the hold applied before the game's handler, every client of 2400 px or taller came out wider
    // (2288x2480 -> 3069, 3282; 2808x3040 -> 3814, 4027) with the hold count at 0: the request reaching us was
    // already the VR shape and the game widened it after us.
    const LRESULT r = CallWindowProcW(g_prev_wndproc, hwnd, msg, wp, lp);
    WINDOWPOS* pos = reinterpret_cast<WINDOWPOS*>(lp);
    const int hw = g_hold_w.load(std::memory_order_relaxed), hh = g_hold_h.load(std::memory_order_relaxed);
    if (pos != nullptr && !(pos->flags & SWP_NOSIZE) && (pos->cx != hw || pos->cy != hh) && !IsIconic(hwnd)) {
      pos->cx = hw;  // keep the VR render shape
      pos->cy = hh;
      g_hold_fixes.fetch_add(1, std::memory_order_relaxed);
    }
    return r;
  }
  if (msg == WM_DPICHANGED && g_hold_w.load(std::memory_order_relaxed) > 0) {
    return 0;  // a tall VR window spans monitors; a DPI-suggested rect would rescale it off the VR shape
  }
  if (msg == WM_GETMINMAXINFO && g_allow_big.load(std::memory_order_relaxed)) {
    // A VR render height taller than the screen: lift the default "no larger than the desktop" track limit.
    const LRESULT r = CallWindowProcW(g_prev_wndproc, hwnd, msg, wp, lp);
    MINMAXINFO* mm = reinterpret_cast<MINMAXINFO*>(lp);
    mm->ptMaxTrackSize.x = 16384;
    mm->ptMaxTrackSize.y = 16384;
    return r;
  }
  if (g_keep_active.load(std::memory_order_relaxed)) {
    // The game throttles itself to ~10.5 fps while inactive (run 2026-10-05: 94 ms per frame unfocused vs 15-38 ms).
    // Hide the deactivation so it keeps its frame rate; Windows still moves keyboard focus normally.
    if ((msg == WM_ACTIVATEAPP && wp == FALSE) || (msg == WM_ACTIVATE && LOWORD(wp) == WA_INACTIVE) || msg == WM_KILLFOCUS) {
      g_hidden_deactivations.fetch_add(1, std::memory_order_relaxed);
      return 0;
    }
  }
  if (msg == WM_KEYDOWN || msg == WM_KEYUP || msg == WM_SYSKEYDOWN || msg == WM_SYSKEYUP) {
    if (wp == VK_F10 || wp == VK_F9 || wp == VK_F8 || wp == VK_F7 || (g_swallow.load(std::memory_order_relaxed) && is_nav_key(wp))) {
      return 0;  // ours (F10 would otherwise also start the system menu-bar mode)
    }
  }
  return CallWindowProcW(g_prev_wndproc, hwnd, msg, wp, lp);
}

void ensure_subclass(HWND hwnd) {
  if (hwnd == nullptr || g_hwnd == hwnd) {
    return;
  }
  g_hwnd = hwnd;
  // Publish the previous procedure BEFORE installing ours: the window thread may call ours immediately.
  const LONG_PTR prev = GetWindowLongPtrW(hwnd, GWLP_WNDPROC);
  if (prev == 0) {
    log_line("window procedure not readable (%lu); keys still work but the game also sees them", GetLastError());
    return;
  }
  g_prev_wndproc = reinterpret_cast<WNDPROC>(prev);
  if (SetWindowLongPtrW(hwnd, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&menu_wndproc)) == 0) {
    log_line("window subclass failed (%lu); keys still work but the game also sees them", GetLastError());
    return;
  }
  log_line("window %p subclassed", static_cast<void*>(hwnd));
}

bool game_focused() {
  HWND fg = GetForegroundWindow();
  if (fg == nullptr) {
    return false;
  }
  DWORD pid = 0;
  GetWindowThreadProcessId(fg, &pid);
  return pid == GetCurrentProcessId();
}

struct KeyState {
  int vk;
  bool down = false;
  ULONGLONG next_repeat = 0;
  ULONGLONG pressed_at = 0;
};

KeyState g_keys[] = {{VK_F10}, {VK_UP}, {VK_DOWN}, {VK_LEFT}, {VK_RIGHT}, {VK_RETURN}, {VK_PRIOR}, {VK_NEXT}, {VK_F9}, {VK_F7}};
enum KeyIndex { kF10, kUp, kDown, kLeft, kRight, kEnter, kPgUp, kPgDn, kF9, kF7, kKeyCount };

// Returns the number of "presses" this frame (edge + repeat for the navigation inputs) for any held/not-held input.
int poll_down(KeyState& k, bool down, bool repeat, ULONGLONG now) {
  int presses = 0;
  if (down && !k.down) {
    presses = 1;
    k.pressed_at = now;
    k.next_repeat = now + 380;
  } else if (down && repeat && now >= k.next_repeat) {
    presses = 1;
    k.next_repeat = now + (now - k.pressed_at > 1500 ? 30 : 75);
  }
  k.down = down;
  return presses;
}

int poll_key(KeyState& k, bool focused, bool repeat, ULONGLONG now) {
  return poll_down(k, focused && (GetAsyncKeyState(k.vk) & 0x8000) != 0, repeat, now);
}

// Motion controllers (src/vr/xr_input.cpp pad controls, so any controller and any rebinding works): either stick or
// the D-pad selects / changes, A activates, LB/RB switch tabs, B closes. Works without window focus (headset use).
KeyState g_pad_keys[8] = {};
enum PadKey { kPadUp, kPadDown, kPadLeft, kPadRight, kPadA, kPadLB, kPadRB, kPadB };
constexpr float kStickPress = 0.6f;

// ---- D3D12 + ImGui --------------------------------------------------------------------------------------

bool g_init_failed = false;
bool g_imgui_ready = false;
ID3D12Device* g_device = nullptr;
ID3D12CommandQueue* g_queue = nullptr;
ID3D12DescriptorHeap* g_srv_heap = nullptr;
ID3D12DescriptorHeap* g_rtv_heap = nullptr;
UINT g_rtv_stride = 0;
ID3D12Resource* g_canvas = nullptr;  // R8G8B8A8_TYPELESS, rests in RENDER_TARGET
ID3D12CommandAllocator* g_alloc[kFramesInFlight] = {};
UINT64 g_alloc_value[kFramesInFlight] = {};
int g_alloc_index = 0;
ID3D12GraphicsCommandList* g_list = nullptr;
ID3D12Fence* g_fence = nullptr;
UINT64 g_fence_next = 0;
HANDLE g_event = nullptr;
ImFont* g_font = nullptr;
ImFont* g_font_big = nullptr;
LARGE_INTEGER g_qpf = {}, g_last_qpc = {};

void srv_alloc(ImGui_ImplDX12_InitInfo* info, D3D12_CPU_DESCRIPTOR_HANDLE* cpu, D3D12_GPU_DESCRIPTOR_HANDLE* gpu) {
  *cpu = info->SrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart();
  *gpu = info->SrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart();
}

void srv_free(ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE, D3D12_GPU_DESCRIPTOR_HANDLE) {}

ImFont* load_font(float size) {
  ImGuiIO& io = ImGui::GetIO();
  char windir[MAX_PATH] = {};
  GetWindowsDirectoryA(windir, MAX_PATH);
  // Semibold italic first: the game's menus use a white italic sans.
  const char* names[] = {"seguisbi.ttf", "segoeuii.ttf", "segoeui.ttf", "arial.ttf"};
  for (const char* name : names) {
    char path[MAX_PATH * 2];
    snprintf(path, sizeof(path), "%s\\Fonts\\%s", windir, name);
    if (GetFileAttributesA(path) == INVALID_FILE_ATTRIBUTES) {
      continue;
    }
    ImFontConfig cfg;
    cfg.OversampleH = 2;
    cfg.OversampleV = 2;
    ImFont* font = io.Fonts->AddFontFromFileTTF(path, size, &cfg);
    if (font != nullptr) {
      return font;
    }
  }
  ImFontConfig cfg;
  cfg.SizePixels = size;
  return io.Fonts->AddFontDefault(&cfg);
}

bool init_d3d(ID3D12Device* device) {
  g_queue = xr_runtime::acquire_game_queue();
  if (g_queue == nullptr) {
    return false;  // not yet; retried next Present
  }
  g_device = device;
  g_device->AddRef();
  bool ok = true;
  D3D12_DESCRIPTOR_HEAP_DESC srv = {};
  srv.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
  srv.NumDescriptors = 1;
  srv.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
  ok = ok && SUCCEEDED(device->CreateDescriptorHeap(&srv, IID_PPV_ARGS(&g_srv_heap)));
  D3D12_DESCRIPTOR_HEAP_DESC rtv = {};
  rtv.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
  rtv.NumDescriptors = 2;  // [0] canvas, [1] the current backbuffer
  ok = ok && SUCCEEDED(device->CreateDescriptorHeap(&rtv, IID_PPV_ARGS(&g_rtv_heap)));
  if (ok) {
    g_rtv_stride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    D3D12_HEAP_PROPERTIES heap = {};
    heap.Type = D3D12_HEAP_TYPE_DEFAULT;
    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    desc.Width = kCanvasW;
    desc.Height = kCanvasH;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = DXGI_FORMAT_R8G8B8A8_TYPELESS;  // copyable into an _SRGB or _UNORM XR swapchain image
    desc.SampleDesc.Count = 1;
    desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
    D3D12_CLEAR_VALUE clear = {};
    clear.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    ok = SUCCEEDED(device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_RENDER_TARGET, &clear,
                                                   IID_PPV_ARGS(&g_canvas)));
  }
  if (ok) {
    D3D12_RENDER_TARGET_VIEW_DESC rd = {};
    rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    rd.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    device->CreateRenderTargetView(g_canvas, &rd, g_rtv_heap->GetCPUDescriptorHandleForHeapStart());
    for (auto*& a : g_alloc) {
      ok = ok && SUCCEEDED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&a)));
    }
  }
  ok = ok && SUCCEEDED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_alloc[0], nullptr, IID_PPV_ARGS(&g_list)));
  ok = ok && SUCCEEDED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_fence)));
  if (ok) {
    g_list->Close();
    g_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    ok = g_event != nullptr;
  }
  if (!ok) {
    log_line("D3D12 object creation failed; menu disabled");
    g_init_failed = true;
    return false;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.LogFilename = nullptr;
  io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
  g_font = load_font(34.f);
  g_font_big = load_font(64.f);

  ImGui_ImplDX12_InitInfo info;
  info.Device = device;
  info.CommandQueue = g_queue;
  info.NumFramesInFlight = 2 * kFramesInFlight + 2;  // two RenderDrawData per Present (canvas + monitor)
  info.RTVFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
  info.DSVFormat = DXGI_FORMAT_UNKNOWN;
  info.SrvDescriptorHeap = g_srv_heap;
  info.SrvDescriptorAllocFn = &srv_alloc;
  info.SrvDescriptorFreeFn = &srv_free;
  if (!ImGui_ImplDX12_Init(&info) || !ImGui_ImplDX12_CreateDeviceObjects() || io.Fonts->TexID == 0) {
    log_line("ImGui D3D12 backend init failed (shader compile or font upload); menu disabled");
    g_init_failed = true;
    return false;
  }
  QueryPerformanceFrequency(&g_qpf);
  QueryPerformanceCounter(&g_last_qpc);
  g_imgui_ready = true;
  log_line("ImGui %s ready: canvas %ux%u, fonts %s", IMGUI_VERSION, kCanvasW, kCanvasH,
           g_font != nullptr ? g_font->GetDebugName() : "?");
  return true;
}

void barrier(ID3D12Resource* r, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = r;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = before;
  b.Transition.StateAfter = after;
  g_list->ResourceBarrier(1, &b);
}

// ---- drawing (design space 1280x960, mapped by origin + scale) ------------------------------------------

struct Painter {
  ImDrawList* d;
  ImVec2 o;
  float s;
  ImVec2 p(float x, float y) const { return ImVec2(o.x + x * s, o.y + y * s); }
  void text(ImFont* font, float size, float x, float y, ImU32 col, const char* t) const {
    d->AddText(font, size * s, p(x, y), col, t);
  }
  float width(ImFont* font, float size, const char* t) const { return font->CalcTextSizeA(size, FLT_MAX, 0.f, t).x; }
};

ImU32 rgba(int r, int g, int b, int a) { return IM_COL32(r, g, b, a); }

void glow_pill(const Painter& P, float x0, float y0, float x1, float y1, ImU32 core, int strength) {
  const float r = (y1 - y0) * 0.5f;
  for (int k = 6; k >= 1; --k) {
    const float g = 2.2f * static_cast<float>(k);
    const int a = strength / (k + 1);
    P.d->AddRect(P.p(x0 - g, y0 - g), P.p(x1 + g, y1 + g), rgba(70, 150, 255, a), (r + g) * P.s, 0, 2.6f * P.s);
  }
  P.d->AddRect(P.p(x0, y0), P.p(x1, y1), core, r * P.s, 0, 2.4f * P.s);
}

void value_text(const Row& r, char* out, size_t n) {
  switch (r.kind) {
    case Kind::Toggle:
      snprintf(out, n, "%s", r.choices[*r.b ? 1 : 0]);
      break;
    case Kind::Choice:
      snprintf(out, n, "%s", r.choices[std::clamp(*r.i, 0, r.choice_count - 1)]);
      break;
    case Kind::Int:
      snprintf(out, n, r.fmt, *r.i);
      break;
    case Kind::Float:
      if (*r.f < r.zero_below && r.zero_label != nullptr) {
        snprintf(out, n, "%s", r.zero_label);
      } else {
        snprintf(out, n, r.fmt, *r.f * r.display_scale);
      }
      break;
    case Kind::Action:
      snprintf(out, n, "Select");
      break;
  }
}

void draw_row(const Painter& P, const Row& r, float y, bool selected, float alpha, float h) {
  const float x0 = 80.f, x1 = 1200.f;
  const float y1 = y + h;
  const float rad = h * 0.5f;
  const int A = static_cast<int>(255.f * alpha);
  auto fade = [&](int a) { return a * A / 255; };
  // body: translucent blue-violet pill, a lighter band on the left like the game's rows
  P.d->AddRectFilled(P.p(x0, y), P.p(x1, y1), selected ? rgba(40, 62, 150, fade(205)) : rgba(26, 30, 78, fade(150)), rad * P.s);
  P.d->AddRectFilledMultiColor(P.p(x0 + rad, y + 3), P.p(x0 + 420, y1 - 3), rgba(120, 150, 255, fade(selected ? 45 : 22)),
                               rgba(120, 150, 255, 0), rgba(120, 150, 255, 0), rgba(120, 150, 255, fade(selected ? 45 : 22)));
  if (selected) {
    glow_pill(P, x0, y, x1, y1, rgba(150, 210, 255, fade(235)), fade(150));
  } else {
    P.d->AddRect(P.p(x0, y), P.p(x1, y1), rgba(120, 135, 230, fade(60)), rad * P.s, 0, 1.5f * P.s);
  }
  const ImU32 white = rgba(255, 255, 255, fade(selected ? 255 : 215));
  const float fs = std::min(34.f, h * 0.66f);
  P.text(g_font, fs, x0 + 46.f, y + (h - fs) * 0.5f - 2.f, white, r.label);

  char value[64];
  value_text(r, value, sizeof(value));
  const float vw = P.width(g_font, fs, value);
  const float vx1 = x1 - 64.f;  // right edge of the value text
  const float vx0 = vx1 - vw;
  const bool has_indicator = r.kind != Kind::Action;
  const float vy = y + (h - fs) * 0.5f - (has_indicator ? 7.f : 2.f);
  P.text(g_font, fs, vx0, vy, white, value);
  if (selected && r.kind != Kind::Action) {
    // < value >
    const float cy = vy + fs * 0.55f;
    const ImU32 arrow = rgba(170, 220, 255, fade(240));
    const float lx = vx0 - 26.f, rx = x1 - 38.f;
    P.d->AddTriangleFilled(P.p(lx, cy), P.p(lx + 11.f, cy - 9.f), P.p(lx + 11.f, cy + 9.f), arrow);
    P.d->AddTriangleFilled(P.p(rx, cy), P.p(rx - 11.f, cy - 9.f), P.p(rx - 11.f, cy + 9.f), arrow);
  }
  if (!has_indicator) {
    return;
  }
  const float iy = y1 - 13.f;
  const float icx = (vx0 + vx1) * 0.5f;
  if (r.kind == Kind::Toggle || r.kind == Kind::Choice) {
    const int n = r.choice_count;
    const int cur = r.kind == Kind::Toggle ? (*r.b ? 1 : 0) : *r.i;
    const float gap = 15.f;
    const float start = icx - gap * static_cast<float>(n - 1) * 0.5f;
    for (int k = 0; k < n; ++k) {
      const ImVec2 c = P.p(start + gap * static_cast<float>(k), iy);
      if (k == cur) {
        P.d->AddCircleFilled(c, 4.5f * P.s, rgba(200, 235, 255, fade(255)));
      } else {
        P.d->AddCircle(c, 4.f * P.s, rgba(200, 220, 255, fade(140)), 0, 1.5f * P.s);
      }
    }
  } else {
    float t = 0.f;
    if (r.kind == Kind::Int) {
      t = (static_cast<float>(*r.i) - r.lo) / (r.hi - r.lo);
    } else if (!(*r.f < r.zero_below)) {
      t = (*r.f - r.lo) / (r.hi - r.lo);
    }
    t = std::clamp(t, 0.f, 1.f);
    const float bw = 150.f;
    const float bx0 = vx1 - bw;
    P.d->AddRectFilled(P.p(bx0, iy - 2.f), P.p(vx1, iy + 2.f), rgba(255, 255, 255, fade(50)), 2.f * P.s);
    P.d->AddRectFilled(P.p(bx0, iy - 2.f), P.p(bx0 + bw * t, iy + 2.f), rgba(130, 200, 255, fade(235)), 2.f * P.s);
    P.d->AddCircleFilled(P.p(bx0 + bw * t, iy), 5.f * P.s, rgba(220, 240, 255, fade(255)));
  }
}

void draw_wrapped(const Painter& P, float size, float x, float y, float wrap, ImU32 col, const char* t) {
  P.d->AddText(g_font, size * P.s, P.p(x, y), col, t, nullptr, wrap * P.s);
}

void draw_menu(const Painter& P) {
  // backdrop
  P.d->AddRectFilled(P.p(0, 0), P.p(kCanvasW, kCanvasH), rgba(8, 10, 32, 205), 40.f * P.s);
  P.d->AddRectFilledMultiColor(P.p(20, 20), P.p(kCanvasW - 20, 300), rgba(60, 80, 190, 40), rgba(60, 80, 190, 40), rgba(0, 0, 0, 0),
                               rgba(0, 0, 0, 0));
  P.d->AddRect(P.p(0, 0), P.p(kCanvasW, kCanvasH), rgba(110, 160, 255, 110), 40.f * P.s, 0, 2.f * P.s);

  // tab strip
  P.text(g_font, 24.f, 80.f, 36.f, rgba(150, 180, 255, 200), "AVATAR VR  |  MOD MENU");
  {
    const char* status = xr_runtime::session_running() ? "Headset connected" : "No headset session";
    const float w = P.width(g_font, 24.f, status);
    P.text(g_font, 24.f, 1200.f - w, 36.f, xr_runtime::session_running() ? rgba(140, 230, 190, 220) : rgba(255, 180, 140, 220), status);
  }
  float tx = 80.f;
  for (int t = 0; t < kTabCount; ++t) {
    const bool cur = t == g_tab;
    const float size = 30.f;
    const char* name = g_tabs[t].name;
    const float w = P.width(g_font, size, name);
    P.text(g_font, size, tx, 78.f, cur ? rgba(255, 255, 255, 255) : rgba(200, 210, 255, 120), name);
    if (cur) {
      const ImU32 bar = g_sel < 0 ? rgba(150, 215, 255, 255) : rgba(120, 180, 255, 200);
      P.d->AddRectFilled(P.p(tx, 118.f), P.p(tx + w, 122.f), bar, 2.f * P.s);
      if (g_sel < 0) {
        for (int k = 1; k <= 3; ++k) {
          P.d->AddRectFilled(P.p(tx - k * 2.f, 118.f - k * 2.f), P.p(tx + w + k * 2.f, 122.f + k * 2.f), rgba(70, 150, 255, 40 / k),
                             4.f * P.s);
        }
      }
    }
    tx += w + 46.f;
  }
  P.text(g_font, 22.f, 1200.f - P.width(g_font, 22.f, "PgUp / PgDn"), 84.f, rgba(170, 190, 240, 150), "PgUp / PgDn");

  // big italic header, like the game's "Video"
  P.text(g_font_big, 64.f, 76.f, 140.f, rgba(255, 255, 255, 255), g_tabs[g_tab].name);

  // rows
  const Tab& tab = g_tabs[g_tab];
  float y = 236.f;
  // rows fill 236..806 at most (the help footer starts at 818); long tabs scroll so the selected row stays visible
  static int first_row[kTabCount] = {};
  const int visible = std::min(tab.count, kVisibleRows);
  int& first = first_row[g_tab];
  if (g_sel >= 0 && g_sel < first) first = g_sel;
  if (g_sel >= first + visible) first = g_sel - visible + 1;
  first = std::clamp(first, 0, std::max(0, tab.count - visible));
  const float step = std::min(80.f, 570.f / static_cast<float>(std::max(visible, 1)));
  if (first > 0) {
    P.text(g_font, 20.f, 1200.f - P.width(g_font, 20.f, "more above"), 210.f, rgba(170, 190, 240, 170), "more above");
  }
  if (first + visible < tab.count) {
    P.text(g_font, 20.f, 1200.f - P.width(g_font, 20.f, "more below"), 800.f, rgba(170, 190, 240, 170), "more below");
  }
  for (int k = first; k < first + visible; ++k) {
    const Row& r = tab.rows[k];
    float alpha = 1.f;
    if ((r.b == &g_s.head || r.f == &g_s.fov || r.f == &g_s.offx || r.f == &g_s.offy || r.f == &g_s.cullfov ||
         r.b == &g_s.autofov) && !g_s.stereo) {
      alpha = 0.55f;  // only meaningful while native stereo is on
    }
    if ((r.f == &g_s.fov || r.f == &g_s.offx || r.f == &g_s.offy) && g_s.autofov) {
      alpha = 0.55f;  // replaced by the matched projection
    }
    if ((r.b == &g_s.eyeshift || r.f == &g_s.margin) && !g_s.autofov) {
      alpha = 0.55f;
    }
    if ((r.f == &g_s.ui_size || r.f == &g_s.ui_height || r.b == &g_s.mm_panel) && !g_s.ui_place) {
      alpha = 0.55f;
    }
    if ((r.f == &g_s.mm_dist || r.f == &g_s.mm_width || r.f == &g_s.mm_height) && (!g_s.ui_place || !g_s.mm_panel)) {
      alpha = 0.55f;
    }
    if (r.f == &g_s.sep && g_s.head) {
      alpha = 0.55f;
    }
    if ((r.f == &g_s.screen_w || r.f == &g_s.screen_d) && g_s.proj) {
      alpha = 0.55f;
    }
    draw_row(P, r, y, k == g_sel, alpha, step - 14.f);
    y += step;
  }

  // footer: help for the selected row and the key hints
  P.d->AddLine(P.p(80.f, 818.f), P.p(1200.f, 818.f), rgba(120, 150, 255, 70), 1.5f * P.s);
  const char* help = g_sel >= 0 && g_sel < tab.count ? tab.rows[g_sel].help : "Left / Right to switch between the tabs.";
  char budget[512];
  if (g_sel >= 0 && g_sel < tab.count && (tab.rows[g_sel].f == &g_s.render_h || tab.rows[g_sel].f == &g_s.eye_scale || tab.rows[g_sel].f == &g_s.margin ||
                                          tab.rows[g_sel].f == &g_s.narrow)) {
    // Show the requested eye size separately from the pooled swapchain buffer, which may be larger during a resize.
    const unsigned rw = g_buf_w, rh = g_buf_h;
    uint32_t qw = 0, qh = 0;
    int ew = 0, eh = 0;
    if (eye_render_size(&ew, &eh) && xr_runtime::recommended_size(&qw, &qh)) {
      snprintf(budget, sizeof(budget), "%s  Selected: %dx%d per eye (%.0f%% of recommended %ux%u pixels). Buffer: %ux%u. "
               "Projection coverage depends on the current FOV settings.", help, ew, eh,
               100.f * static_cast<float>(ew) * eh / (static_cast<float>(qw) * qh), qw, qh, rw, rh);
      help = budget;
    }
  }
  draw_wrapped(P, 26.f, 84.f, 830.f, 1110.f, rgba(200, 212, 245, 225), help);
  P.text(g_font, 22.f, 84.f, 905.f, rgba(160, 180, 235, 170),
         "Up/Down  Select      Left/Right  Change      Enter  Activate      PgUp/PgDn  Tab      F10  Close");
}

std::vector<lua_host::DrawPrim> g_draws;  // script debug drawing for this frame (fractions of the game image)

ImU32 prim_color(std::uint32_t rgba) {
  return IM_COL32((rgba >> 24) & 0xFF, (rgba >> 16) & 0xFF, (rgba >> 8) & 0xFF, rgba & 0xFF);
}

void draw_script_prims(ImDrawList* d, float w, float h) {
  for (const auto& p : g_draws) {
    const ImU32 c = prim_color(p.rgba);
    switch (p.kind) {
      case lua_host::DrawPrim::Line:
        d->AddLine(ImVec2(p.x0 * w, p.y0 * h), ImVec2(p.x1 * w, p.y1 * h), c, p.size);
        break;
      case lua_host::DrawPrim::Point:
        d->AddCircleFilled(ImVec2(p.x0 * w, p.y0 * h), p.size, c);
        break;
      case lua_host::DrawPrim::Text:
        d->AddText(g_font, 22.f, ImVec2(p.x0 * w + 1.f, p.y0 * h + 1.f), IM_COL32(0, 0, 0, 200), p.text.c_str());
        d->AddText(g_font, 22.f, ImVec2(p.x0 * w, p.y0 * h), c, p.text.c_str());
        break;
    }
  }
}

void build_frame(float display_w, float display_h, const ImVec2& origin, float scale, float dt, bool menu = true,
                 bool script_draws = false) {
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(display_w, display_h);
  io.DeltaTime = dt > 0.f ? dt : 1e-4f;
  ImGui_ImplDX12_NewFrame();
  ImGui::NewFrame();
  if (script_draws) {
    draw_script_prims(ImGui::GetBackgroundDrawList(), display_w, display_h);  // under the menu
  }
  Painter P{ImGui::GetForegroundDrawList(), origin, scale};
  if (menu) {
    draw_menu(P);
  }
  ImGui::Render();
}

// ---- per Present ----------------------------------------------------------------------------------------

void handle_input(bool focused, ULONGLONG now) {
  xr_input::PadState c;
  const bool pad = xr_input::controls(&c);
  auto held = [&](std::uint16_t bit) { return pad && (c.buttons & bit) != 0; };
  const bool p_up = pad && (c.ly > kStickPress || c.ry > kStickPress || held(XINPUT_GAMEPAD_DPAD_UP));
  const bool p_down = pad && (c.ly < -kStickPress || c.ry < -kStickPress || held(XINPUT_GAMEPAD_DPAD_DOWN));
  const bool p_left = pad && (c.lx < -kStickPress || c.rx < -kStickPress || held(XINPUT_GAMEPAD_DPAD_LEFT));
  const bool p_right = pad && (c.lx > kStickPress || c.rx > kStickPress || held(XINPUT_GAMEPAD_DPAD_RIGHT));
  const int up = poll_key(g_keys[kUp], focused, true, now) + poll_down(g_pad_keys[kPadUp], p_up, true, now);
  const int down = poll_key(g_keys[kDown], focused, true, now) + poll_down(g_pad_keys[kPadDown], p_down, true, now);
  const int left = poll_key(g_keys[kLeft], focused, true, now) + poll_down(g_pad_keys[kPadLeft], p_left, true, now);
  const int right = poll_key(g_keys[kRight], focused, true, now) + poll_down(g_pad_keys[kPadRight], p_right, true, now);
  const int enter = poll_key(g_keys[kEnter], focused, false, now) +
                    poll_down(g_pad_keys[kPadA], held(XINPUT_GAMEPAD_A), false, now);
  const int pgup = poll_key(g_keys[kPgUp], focused, false, now) +
                   poll_down(g_pad_keys[kPadLB], held(XINPUT_GAMEPAD_LEFT_SHOULDER), false, now);
  const int pgdn = poll_key(g_keys[kPgDn], focused, false, now) +
                   poll_down(g_pad_keys[kPadRB], held(XINPUT_GAMEPAD_RIGHT_SHOULDER), false, now);
  const int back = poll_down(g_pad_keys[kPadB], held(XINPUT_GAMEPAD_B), false, now);
  if (!g_visible) {
    return;
  }
  if (back) {
    g_visible = false;
    log_line("menu closed (controller B)");
    return;
  }
  auto switch_tab = [&](int dir) {
    g_tab = (g_tab + dir + kTabCount) % kTabCount;
    if (g_sel >= g_tabs[g_tab].count) {
      g_sel = g_tabs[g_tab].count - 1;
    }
    log_line("tab %s (%d rows)", g_tabs[g_tab].name, g_tabs[g_tab].count);
  };
  if (pgup) {
    switch_tab(-1);
  }
  if (pgdn) {
    switch_tab(1);
  }
  const int count = g_tabs[g_tab].count;
  if (up) {
    g_sel = g_sel <= -1 ? count - 1 : g_sel - 1;
  }
  if (down) {
    g_sel = g_sel >= count - 1 ? -1 : g_sel + 1;
  }
  if (g_sel < 0) {
    if (left) {
      switch_tab(-1);
    }
    if (right) {
      switch_tab(1);
    }
    return;
  }
  Row& r = g_tabs[g_tab].rows[g_sel];
  if (left) {
    change(r, -1);
  }
  if (right) {
    change(r, 1);
  }
  if (enter) {
    change(r, 0);
  }
}

// A skipped menu frame leaves the headset panel showing its last image and the monitor without the menu.
void skipped(const char* why) {
  static ULONGLONG last = 0;
  static unsigned count = 0;
  ++count;
  const ULONGLONG now = GetTickCount64();
  if (now - last >= 2000) {
    log_line("menu frame skipped (%s), %u since last report", why, count);
    last = now;
    count = 0;
  }
}

void render(IDXGISwapChain* swapchain, ID3D12Resource* back, const D3D12_RESOURCE_DESC& bdesc) {
  LARGE_INTEGER now_qpc;
  QueryPerformanceCounter(&now_qpc);
  const float dt = static_cast<float>(static_cast<double>(now_qpc.QuadPart - g_last_qpc.QuadPart) / static_cast<double>(g_qpf.QuadPart));
  g_last_qpc = now_qpc;
  (void)swapchain;

  const int slot = g_alloc_index;
  g_alloc_index = (g_alloc_index + 1) % kFramesInFlight;
  if (g_fence->GetCompletedValue() < g_alloc_value[slot]) {
    g_fence->SetEventOnCompletion(g_alloc_value[slot], g_event);
    if (WaitForSingleObject(g_event, 200) != WAIT_OBJECT_0) {
      skipped("GPU wait");  // GPU far behind; skip this frame's menu rather than reuse a busy allocator
      return;
    }
  }
  if (FAILED(g_alloc[slot]->Reset()) || FAILED(g_list->Reset(g_alloc[slot], nullptr))) {
    skipped("command list reset");
    return;
  }
  ID3D12DescriptorHeap* heaps[] = {g_srv_heap};

  // 1) canvas (headset): only while the menu is open (script drawing goes to the monitor only)
  const bool xr = xr_runtime::session_running() && g_visible;
  if (xr) {
    build_frame(static_cast<float>(kCanvasW), static_cast<float>(kCanvasH), ImVec2(0.f, 0.f), 1.f, dt);
    D3D12_CPU_DESCRIPTOR_HANDLE rtv = g_rtv_heap->GetCPUDescriptorHandleForHeapStart();
    const float clear[4] = {0.f, 0.f, 0.f, 0.f};
    g_list->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
    g_list->ClearRenderTargetView(rtv, clear, 0, nullptr);
    g_list->SetDescriptorHeaps(1, heaps);
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), g_list);
  }

  // 2) monitor: the same layout scaled into the backbuffer (already copied to the eyes by xr_runtime)
  const bool flat_ok = back != nullptr && (bdesc.Format == DXGI_FORMAT_R8G8B8A8_UNORM || bdesc.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS);
  const bool flat = flat_ok && ((g_s.mirror && g_visible) || !g_draws.empty());
  if (flat) {
    unsigned cw = 0, ch = 0;
    const bool grown = dx12::client_size(&cw, &ch);  // eye mode: the monitor shows only the client-size top-left
    const float bw = static_cast<float>(grown ? cw : bdesc.Width), bh = static_cast<float>(grown ? ch : bdesc.Height);
    const float s = std::min(bw / kCanvasW, bh / kCanvasH) * 0.92f;
    build_frame(bw, bh, ImVec2((bw - kCanvasW * s) * 0.5f, (bh - kCanvasH * s) * 0.5f), s, xr ? 1e-4f : dt,
                g_visible && g_s.mirror, true);
    D3D12_CPU_DESCRIPTOR_HANDLE rtv = g_rtv_heap->GetCPUDescriptorHandleForHeapStart();
    rtv.ptr += g_rtv_stride;
    D3D12_RENDER_TARGET_VIEW_DESC rd = {};
    rd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    rd.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    g_device->CreateRenderTargetView(back, &rd, rtv);
    barrier(back, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
    g_list->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
    g_list->SetDescriptorHeaps(1, heaps);
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), g_list);
    barrier(back, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
  }

  // 3) canvas -> XR overlay swapchain image (the XR lock is held from acquire to release)
  ID3D12Resource* image = xr ? xr_runtime::overlay_acquire(kCanvasW, kCanvasH) : nullptr;
  if (image != nullptr) {
    barrier(g_canvas, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE);
    barrier(image, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
    g_list->CopyResource(image, g_canvas);
    barrier(image, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET);
    barrier(g_canvas, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
  }
  const bool recorded = xr || flat;
  const bool closed = SUCCEEDED(g_list->Close());
  if (closed && recorded) {
    ID3D12CommandList* lists[] = {g_list};
    g_queue->ExecuteCommandLists(1, lists);
    g_queue->Signal(g_fence, ++g_fence_next);
    g_alloc_value[slot] = g_fence_next;
  }
  if (image != nullptr) {
    xr_runtime::overlay_release(closed && recorded);
  }
}

}  // namespace

namespace vr_menu {

void on_present(IDXGISwapChain* swapchain) {
  if (swapchain == nullptr || g_init_failed) {
    return;
  }
  if (g_dir.empty()) {
    g_dir = module_dir();
    if (g_dir.empty()) {
      g_init_failed = true;
      return;
    }
    g_log_path = g_dir + L"\\VR_MENU.log";
    build_tabs();
    build_scripts_tab();
    load_settings();
    wrist_hud::load(g_dir.c_str());
    log_line("vr menu build %s %s: F10 toggles", __DATE__, __TIME__);
    log_line("tabs: %s %d, %s %d, %s %d, %s %d, %s %d (max %d)", g_tabs[0].name, g_tabs[0].count, g_tabs[1].name,
             g_tabs[1].count, g_tabs[2].name, g_tabs[2].count, g_tabs[3].name, g_tabs[3].count, g_tabs[4].name,
             g_tabs[4].count, kMaxRows);
  }
  DXGI_SWAP_CHAIN_DESC scd = {};
  if (SUCCEEDED(swapchain->GetDesc(&scd))) {
    ensure_subclass(scd.OutputWindow);
  }
  native_stereo::set_render_size(scd.BufferDesc.Width, scd.BufferDesc.Height);
  xr_runtime::set_mirror_refresh(static_cast<unsigned>(g_s.monitor_hz));
  xr_runtime::set_sharpness(g_s.sharpness);
  static std::string saved_monitor;
  char monitor[256];
  snprintf(monitor, sizeof(monitor), "refresh %u keep %d windoww %d windowh %d windowx %d windowy %d frame %lu sharpen %u",
           static_cast<unsigned>(g_s.monitor_hz), g_s.monitor_keep ? 1 : 0, g_monitor_w.load(), g_monitor_h.load(),
           g_monitor_x.load(), g_monitor_y.load(), static_cast<unsigned long>(g_monitor_frame.load()),
           static_cast<unsigned>(g_s.sharpness + 0.5f));
  if (saved_monitor != monitor && write_text(g_dir + L"\\AvatarVR_monitor.txt", monitor)) saved_monitor = monitor;
  wrist_hud::poll();
  g_buf_w = scd.BufferDesc.Width;
  g_buf_h = scd.BufferDesc.Height;
  {
    static UINT last_w = 0, last_h = 0;
    if (scd.BufferDesc.Width != last_w || scd.BufferDesc.Height != last_h) {
      last_w = scd.BufferDesc.Width;
      last_h = scd.BufferDesc.Height;
      log_line("swapchain buffers %ux%u (window resizes held to the VR shape so far: %llu)", last_w, last_h,
               static_cast<unsigned long long>(g_hold_fixes.load()));
    }
  }
  enforce_shape(scd.OutputWindow, scd.BufferDesc.Width, scd.BufferDesc.Height);

  const ULONGLONG now = GetTickCount64();
  const bool focused = game_focused();
  if (poll_key(g_keys[kF7], focused, false, now)) {
    lua_host::request_reload();
    log_line("F7: script reload requested");
  }
  if (g_scripts_rows != lua_host::rows_version()) {
    build_scripts_tab();
  }
  if (poll_key(g_keys[kF9], focused, false, now)) {
    frame_capture::request_eyes();
    log_line("F9: eye capture requested (scripts\\captures\\eyes_*.jpg)");
  }
  const int xr_events = xr_input::take_menu_events();
  const bool xr_toggle = ((xr_events & xr_input::kMenuOpen) != 0 && !g_visible) ||
                         ((xr_events & xr_input::kMenuClose) != 0 && g_visible);
  if (poll_key(g_keys[kF10], focused, false, now) || xr_toggle) {
    g_visible = !g_visible;
    if (g_visible) {
      load_settings();
      xr_runtime::recentre_overlay();
      log_line("menu opened (stereo=%d head=%d fov=%.1f offy=%.2f xr=%d proj=%d)", g_s.stereo ? 1 : 0, g_s.head ? 1 : 0, g_s.fov,
               g_s.offy, g_s.xr ? 1 : 0, g_s.proj ? 1 : 0);
    } else {
      log_line("menu closed");
    }
  }
  handle_input(focused, now);
  xr_input::set_menu_open(g_visible);  // the game gets a neutral pad while the menu has the controllers
  if (g_dirty_native) {
    g_dirty_native = false;
    apply_native();
  }
  if (g_dirty_xr) {
    g_dirty_xr = false;
    apply_xr();
  }
  if (scd.OutputWindow != nullptr) {
    g_shape_hwnd = scd.OutputWindow;
  }
  {
    // Start-up render shape: apply the default VR shape once, after the headset's FOV is known (or 20 s without it).
    static ULONGLONG first = 0;
    static bool done = false;
    if (!done && g_shape_hwnd != nullptr) {
      if (first == 0) {
        first = GetTickCount64();
      }
      head_pose::Snapshot snap;
      const ULONGLONG age = GetTickCount64() - first;
      if (!g_s.xr || g_s.shape == 0) {  // Flat + VR eyes also waits: its eye aspect needs the runtime FOV
        done = true;
      } else if (age > 3000 && (xr_runtime::latest_views(&snap) || age > 20000)) {
        done = true;
        g_dirty_shape = true;
        g_shape_changed_at = 0;
        log_line("render shape: start-up %s %.0f px (AvatarVR_shape.txt or default)", g_s.shape == 1 ? "VR eye" : "VR wide", g_s.render_h);
      }
    }
  }
  update_eye_resize();
  update_monitor_window();
  update_menu_shape(GetTickCount64());
  monitor_preview::update(g_shape_hwnd, g_s.preview);
  {
    static int last = -1;
    unsigned long long reg = 0, fail = 0;
    bool vis = false;
    monitor_preview::stats(&reg, &fail, &vis);
    if (static_cast<int>(vis) != last) {
      last = vis ? 1 : 0;
      log_line("monitor preview %s (thumbnails registered %llu, failed %llu)", vis ? "shown" : "hidden", reg, fail);
    }
  }
  if (g_dirty_shape && GetTickCount64() - g_shape_changed_at > 800) {
    std::uint32_t recommended_w = 0, recommended_h = 0;
    if (g_s.shape == kShapeFlatEyes && g_s.eye_scale >= 70.f &&
        !xr_runtime::recommended_size(&recommended_w, &recommended_h)) {
      // Wait for OpenXR's size rather than briefly writing a height-derived eye pair.
    } else if (native_stereo::stereo_running()) {
      // Pause stereo first and resize once it has let go of its frames: with stereo running the game's ResizeBuffers
      // failed (0x887A0001, run 2026-10-06 07:26), leaving a warped picture and then an XR runtime failure.
      native_stereo::hold_for_resize(3000);
      g_shape_changed_at = GetTickCount64();
      log_line("render shape: pausing stereo for the resize");
    } else {
      g_dirty_shape = false;
      native_stereo::hold_for_resize(3000);  // stays paused while the game resizes its buffers
      apply_shape();
    }
  }
  g_swallow.store(g_visible, std::memory_order_relaxed);
  g_keep_active.store(g_s.keep_active, std::memory_order_relaxed);
  {
    static uint64_t logged = 0;
    const uint64_t hidden = g_hidden_deactivations.load(std::memory_order_relaxed);
    if (hidden != logged) {
      logged = hidden;
      log_line("focus loss hidden from the game (%llu messages so far)", static_cast<unsigned long long>(hidden));
    }
  }
  xr_runtime::set_overlay(g_visible, g_s.menu_dist, g_s.menu_width);
  const bool draws = lua_host::copy_draws(&g_draws);
  if (!g_visible && !draws) {
    return;
  }

  IDXGISwapChain3* sc3 = nullptr;
  ID3D12Resource* back = nullptr;
  if (FAILED(swapchain->QueryInterface(IID_PPV_ARGS(&sc3))) || sc3 == nullptr) {
    return;
  }
  if (FAILED(sc3->GetBuffer(sc3->GetCurrentBackBufferIndex(), IID_PPV_ARGS(&back)))) {
    back = nullptr;
  }
  if (back != nullptr) {
    if (!g_imgui_ready) {
      ID3D12Device* device = nullptr;
      if (SUCCEEDED(back->GetDevice(IID_PPV_ARGS(&device))) && device != nullptr) {
        init_d3d(device);
        device->Release();
      }
    }
    if (g_imgui_ready) {
      render(swapchain, back, back->GetDesc());
    }
    back->Release();
  }
  sc3->Release();
}

}  // namespace vr_menu
