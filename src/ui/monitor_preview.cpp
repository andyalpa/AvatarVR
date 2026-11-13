#include "monitor_preview.hpp"

#include <dwmapi.h>

#include <atomic>
#include <thread>

namespace monitor_preview {
namespace {

std::atomic<HWND> g_game{nullptr};
std::atomic<bool> g_want{false};
std::atomic<bool> g_started{false};
std::atomic<bool> g_visible{false};
std::atomic<unsigned long long> g_registered{0}, g_failed{0};

constexpr UINT_PTR kTimer = 1;
constexpr wchar_t kClass[] = L"AvatarVR_MonitorPreview";

struct State {
  HWND game = nullptr;
  HTHUMBNAIL thumb = nullptr;
  RECT placed = {};
};
State g_state;  // preview thread only

void drop_thumb() {
  if (g_state.thumb != nullptr) {
    DwmUnregisterThumbnail(g_state.thumb);
    g_state.thumb = nullptr;
  }
  g_state.game = nullptr;
}

// Preview thread, on a timer: follow the game window's size and the monitor it is on.
void refresh(HWND self) {
  HWND game = g_game.load();
  RECT client = {};
  MONITORINFO mi = {sizeof(mi)};
  RECT wr = {};
  bool show = g_want.load() && game != nullptr && IsWindow(game) && !IsIconic(game) && GetClientRect(game, &client) &&
              GetWindowRect(game, &wr) && GetMonitorInfoW(MonitorFromWindow(game, MONITOR_DEFAULTTONEAREST), &mi);
  const int cw = client.right - client.left;
  const int ch = client.bottom - client.top;
  const int work_w = mi.rcWork.right - mi.rcWork.left;
  const int work_h = mi.rcWork.bottom - mi.rcWork.top;
  // Only needed while the game window runs off the work area.
  show = show && cw > 0 && ch > 0 && work_w > 0 && work_h > 0 &&
         (wr.right - wr.left > work_w || wr.bottom - wr.top > work_h);
  if (!show) {
    if (g_visible.exchange(false)) {
      ShowWindow(self, SW_HIDE);
    }
    if (game != g_state.game) {
      drop_thumb();
    }
    return;
  }
  if (game != g_state.game || g_state.thumb == nullptr) {
    drop_thumb();
    if (SUCCEEDED(DwmRegisterThumbnail(self, game, &g_state.thumb))) {
      g_state.game = game;
      g_registered.fetch_add(1);
      // Keep the preview above the game window (owned windows stay above their owner) but not above other apps.
      SetWindowLongPtrW(self, GWLP_HWNDPARENT, reinterpret_cast<LONG_PTR>(game));
    } else {
      g_state.thumb = nullptr;
      g_failed.fetch_add(1);
      return;
    }
  }
  // Fit the client area into the work area, keeping its aspect, centred.
  int w = work_w;
  int h = static_cast<int>(static_cast<long long>(w) * ch / cw);
  if (h > work_h) {
    h = work_h;
    w = static_cast<int>(static_cast<long long>(h) * cw / ch);
  }
  const RECT dest = {mi.rcWork.left + (work_w - w) / 2, mi.rcWork.top + (work_h - h) / 2, 0, 0};
  RECT placed = {dest.left, dest.top, dest.left + w, dest.top + h};
  if (!EqualRect(&placed, &g_state.placed) || !g_visible.load()) {
    g_state.placed = placed;
    SetWindowPos(self, nullptr, placed.left, placed.top, w, h, SWP_NOACTIVATE | SWP_NOZORDER | SWP_SHOWWINDOW);
  }
  DWM_THUMBNAIL_PROPERTIES p = {};
  p.dwFlags = DWM_TNP_RECTDESTINATION | DWM_TNP_VISIBLE | DWM_TNP_SOURCECLIENTAREAONLY | DWM_TNP_OPACITY;
  p.rcDestination = {0, 0, w, h};
  p.fVisible = TRUE;
  p.fSourceClientAreaOnly = TRUE;
  p.opacity = 255;
  DwmUpdateThumbnailProperties(g_state.thumb, &p);
  if (!g_visible.exchange(true)) {
    ShowWindow(self, SW_SHOWNOACTIVATE);
  }
}

LRESULT CALLBACK wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  switch (msg) {
    case WM_TIMER:
      if (wp == kTimer) {
        refresh(hwnd);
      }
      return 0;
    case WM_MOUSEACTIVATE:
      return MA_NOACTIVATE;
    case WM_NCHITTEST:
      return HTTRANSPARENT;
    case WM_ERASEBKGND: {
      RECT r = {};
      GetClientRect(hwnd, &r);
      FillRect(reinterpret_cast<HDC>(wp), &r, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));
      return 1;
    }
    default:
      return DefWindowProcW(hwnd, msg, wp, lp);
  }
}

void thread_main() {
  HINSTANCE inst = GetModuleHandleW(nullptr);
  WNDCLASSEXW wc = {sizeof(wc)};
  wc.lpfnWndProc = wndproc;
  wc.hInstance = inst;
  wc.lpszClassName = kClass;
  wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
  RegisterClassExW(&wc);
  // Layered + transparent: clicks fall through to the game window underneath, and the preview never takes focus.
  HWND self = CreateWindowExW(WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_NOACTIVATE | WS_EX_TOOLWINDOW, kClass,
                              L"AvatarVR preview", WS_POPUP, 0, 0, 16, 16, nullptr, nullptr, inst, nullptr);
  if (self == nullptr) {
    g_failed.fetch_add(1);
    return;
  }
  SetLayeredWindowAttributes(self, 0, 255, LWA_ALPHA);
  SetTimer(self, kTimer, 250, nullptr);
  MSG msg;
  while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
    TranslateMessage(&msg);
    DispatchMessageW(&msg);
  }
}

}  // namespace

void update(HWND game, bool want) {
  if (game != nullptr) {
    g_game.store(game);
  }
  g_want.store(want);
  if (want && !g_started.exchange(true)) {
    std::thread(thread_main).detach();
  }
}

void stats(unsigned long long* registered, unsigned long long* failed, bool* visible) {
  *registered = g_registered.load();
  *failed = g_failed.load();
  *visible = g_visible.load();
}

}  // namespace monitor_preview
