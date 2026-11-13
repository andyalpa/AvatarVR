#include "virtual_pad.hpp"
#include "vr_controls.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../diagnostics/move_probe.hpp"

#include "xr_input.hpp"
#include "menu_pointer.hpp"
#include "pitch_level.hpp"
#include "../camera/native_stereo.hpp"
#include "../game/player.hpp"

#include <windows.h>
#include <Xinput.h>

#include <MinHook.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cwchar>
#include <mutex>
#include <string>
#include "../settings_file.hpp"

// Evidence: research/INPUT_OWNERSHIP.md "Gamepad backends" ([STATIC] + [LIVE] INPUT_PROBE runs 2026-10-07):
// afop.exe+0x3294C50 calls the Windows.Gaming.Input init first and LoadLibraryA's XInput only when it fails; with
// WGI up (init_ok=1) xinput is never loaded and the game creates wgi_pad#0..3. This module never touches game
// memory: it hooks combase!RoGetActivationFactory and the xinput* exports. Only XInput's documented ABI is used.

namespace {

enum class Mode { kOff, kTest, kXr };

using RoGetActivationFactoryFn = HRESULT(WINAPI*)(void* hstring_class, REFIID iid, void** factory);
using WindowsGetStringRawBufferFn = PCWSTR(WINAPI*)(void* hstring, UINT32* length);
using XInputGetStateFn = DWORD(WINAPI*)(DWORD, XINPUT_STATE*);
using XInputSetStateFn = DWORD(WINAPI*)(DWORD, XINPUT_VIBRATION*);
using XInputGetCapabilitiesFn = DWORD(WINAPI*)(DWORD, DWORD, XINPUT_CAPABILITIES*);

constexpr const wchar_t* kBlockedClass = L"Windows.Gaming.Input.Gamepad";
constexpr const wchar_t* kXinputDlls[] = {L"xinput1_4.dll", L"xinput1_3.dll", L"xinput9_1_0.dll"};
constexpr int kMaxXinputDlls = 3;

Mode g_mode = Mode::kOff;
std::atomic<bool> g_install_attempted{false};

RoGetActivationFactoryFn g_ro_original = nullptr;
WindowsGetStringRawBufferFn g_raw_buffer = nullptr;
XInputGetStateFn g_get_state_original[kMaxXinputDlls] = {};
XInputSetStateFn g_set_state_original[kMaxXinputDlls] = {};
XInputGetCapabilitiesFn g_get_caps_original[kMaxXinputDlls] = {};

std::atomic<std::uint64_t> g_blocked{0};
std::atomic<std::uint64_t> g_get_state_calls[4] = {};
std::atomic<std::uint64_t> g_get_caps_calls{0};
std::atomic<std::uint64_t> g_set_state_calls{0};
std::atomic<std::uint64_t> g_xr_reads{0};  // slot-0 reads that had a focused controller state
std::atomic<std::uint32_t> g_packet{1};
ULONGLONG g_start_tick = 0;
ULONGLONG g_last_log = 0;

std::mutex g_log_mutex;
std::wstring g_log_path;

void write_line(const char* text) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty()) {
    return;
  }
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, text, static_cast<DWORD>(std::strlen(text)), &written, nullptr);
  WriteFile(file, "\r\n", 2, &written, nullptr);
  CloseHandle(file);
}

void logf(const char* fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  write_line(buffer);
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

Mode read_mode(const std::wstring& dir) {
  char text[64] = {};
  if (!settings_file::read(dir + L"\\AvatarVR_pad.txt", text, sizeof(text))) {
    return Mode::kOff;
  }
  if (std::strncmp(text, "test", 4) == 0) {
    return Mode::kTest;
  }
  return std::strncmp(text, "xr", 2) == 0 ? Mode::kXr : Mode::kOff;
}

const char* mode_name(Mode m) {
  return m == Mode::kTest ? "test" : m == Mode::kXr ? "xr" : "off";
}

// The scripted test pad: connected, left stick forward for 2 s of every 10 s, nothing else pressed.
void fill_test_state(XINPUT_GAMEPAD* pad) {
  const ULONGLONG ms = (GetTickCount64() - g_start_tick) % 10000;
  pad->sThumbLY = ms < 2000 ? 32767 : 0;
}

SHORT to_axis(float v) {
  const float c = v < -1.f ? -1.f : v > 1.f ? 1.f : v;
  return static_cast<SHORT>(c * 32767.f);
}

BYTE to_trigger(float v) {
  return static_cast<BYTE>((v < 0.f ? 0.f : v > 1.f ? 1.f : v) * 255.f + 0.5f);
}

SHORT stronger(SHORT a, SHORT b) {
  return std::abs(static_cast<int>(a)) >= std::abs(static_cast<int>(b)) ? a : b;
}

// "xr": controllers merged with a real pad on slot 0 (buttons OR'ed; per axis/trigger the larger deflection wins).
void merge_xr(XINPUT_GAMEPAD* pad, const xr_input::PadState& s) {
  pad->wButtons |= s.buttons;
  pad->sThumbLX = stronger(pad->sThumbLX, to_axis(s.lx));
  pad->sThumbLY = stronger(pad->sThumbLY, to_axis(s.ly));
  pad->sThumbRX = stronger(pad->sThumbRX, to_axis(s.rx));
  pad->sThumbRY = stronger(pad->sThumbRY, to_axis(s.ry));
  pad->bLeftTrigger = std::max(pad->bLeftTrigger, to_trigger(s.lt));
  pad->bRightTrigger = std::max(pad->bRightTrigger, to_trigger(s.rt));
}

// Keep the game's pitch level in VR gameplay (pitch_level.hpp): the right stick's up/down is replaced by a correcting
// value computed from the game's current pitch (look +0x38, which the game rewrites from its internal state each frame).
std::atomic<int> g_last_rx_milli{0};             // right stick X the game would have seen (-1000..1000)
std::atomic<ULONGLONG> g_turn_block_until{0};     // while in the future, right stick X is withheld from the game
bool g_pitch_level = true;  // mod menu "Keep game pitch level" (Present thread writes, input thread reads)
float g_pitch_target_deg = 0.f;  // mod menu "Default game pitch" (degrees, + up)
pitch_level::Controller g_pitch;
std::mutex g_pitch_mutex;
std::atomic<unsigned> g_pitch_steps{0}, g_pitch_active{0};
std::atomic<int> g_pitch_last_mdeg{0}, g_pitch_last_stick{0};

void level_pitch(XINPUT_GAMEPAD* pad) {
  if (!g_pitch_level || !native_stereo::stereo_running() || native_stereo::front_end()) return;
  const game::Player p = game::read();
  if (!p.look_ok) return;
  float stick = 0.f;
  {
    std::lock_guard<std::mutex> lock(g_pitch_mutex);
    g_pitch.target = std::clamp(g_pitch_target_deg, -70.f, 70.f) * 0.0174532925f;
    stick = g_pitch.step(p.pitch);
  }
  pad->sThumbRY = to_axis(stick);
  g_pitch_steps.fetch_add(1, std::memory_order_relaxed);
  if (stick != 0.f) g_pitch_active.fetch_add(1, std::memory_order_relaxed);
  g_pitch_last_mdeg.store(static_cast<int>(p.pitch * 57295.78f), std::memory_order_relaxed);
  g_pitch_last_stick.store(static_cast<int>(stick * 1000.f), std::memory_order_relaxed);
}

// The packet number must change whenever the reported state changes.
DWORD stamp(XINPUT_STATE* state) {
  static XINPUT_GAMEPAD last = {};
  static std::mutex m;
  std::lock_guard<std::mutex> lock(m);
  if (std::memcmp(&last, &state->Gamepad, sizeof(last)) != 0) {
    last = state->Gamepad;
    g_packet.fetch_add(1, std::memory_order_relaxed);
  }
  state->dwPacketNumber = g_packet.load(std::memory_order_relaxed);
  return ERROR_SUCCESS;
}

template <int I>
DWORD WINAPI get_state_detour(DWORD user, XINPUT_STATE* state) {
  if (user < 4) {
    g_get_state_calls[user].fetch_add(1, std::memory_order_relaxed);
  }
  if (user != 0 || state == nullptr) {
    return g_get_state_original[I](user, state);
  }
  if (g_mode == Mode::kTest) {
    std::memset(state, 0, sizeof(*state));
    fill_test_state(&state->Gamepad);
    return stamp(state);
  }
  // kXr: a real pad on slot 0 (if any) plus the controllers (if a session is focused).
  XINPUT_STATE real = {};
  const bool have_real = g_get_state_original[I](0, &real) == ERROR_SUCCESS;
  xr_input::PadState s;
  const bool have_xr = xr_input::latest(&s);
  if (have_xr) {
    g_xr_reads.fetch_add(1, std::memory_order_relaxed);
  }
  if (!have_real && !have_xr) {
    return ERROR_DEVICE_NOT_CONNECTED;
  }
  std::memset(state, 0, sizeof(*state));
  if (have_real) {
    state->Gamepad = real.Gamepad;
  }
  if (have_xr) {
    menu_pointer::apply(&s, GetTickCount64());  // the controller laser's trigger is A while it points at a game menu
    // Run 64: the VR control scheme (gestures, weapon-hand buttons) when on; otherwise the plain mapping, with the
    // left-handed trigger swap (left trigger fires).
    XINPUT_GAMEPAD vr = {};
    const bool gameplay = native_stereo::stereo_running() && !native_stereo::front_end() && game::read().valid;
    if (vr_controls::compose(s, &vr, gameplay)) {
      state->Gamepad.wButtons |= vr.wButtons;
      state->Gamepad.sThumbLX = stronger(state->Gamepad.sThumbLX, vr.sThumbLX);
      state->Gamepad.sThumbLY = stronger(state->Gamepad.sThumbLY, vr.sThumbLY);
      state->Gamepad.sThumbRX = stronger(state->Gamepad.sThumbRX, vr.sThumbRX);
      state->Gamepad.sThumbRY = stronger(state->Gamepad.sThumbRY, vr.sThumbRY);
      state->Gamepad.bLeftTrigger = std::max(state->Gamepad.bLeftTrigger, vr.bLeftTrigger);
      state->Gamepad.bRightTrigger = std::max(state->Gamepad.bRightTrigger, vr.bRightTrigger);
    } else {
      xr_input::PadState t = s;  // plain mapping; the left-handed trigger swap only in gameplay (menus keep LT/RT tabs)
      if (gameplay && skin_palette_probe::weapon_hand_index() == 0) std::swap(t.lt, t.rt);
      merge_xr(&state->Gamepad, t);
    }
  }
  const bool wheel = vr_controls::wheel_active();  // the right stick carries the wheel selection
  if (!wheel) level_pitch(&state->Gamepad);
  // Movement direction: Head (move_probe, VR menu): the left stick is rotated so forward is where the headset looks.
  if (!wheel && native_stereo::stereo_running() && !native_stereo::front_end())
    move_probe::head_move_stick(&state->Gamepad.sThumbLX, &state->Gamepad.sThumbLY);
  // Mod-owned turning (aim: independent view): the game never sees right-stick X while a script holds the turn.
  g_last_rx_milli.store(static_cast<int>(state->Gamepad.sThumbRX * 1000 / 32767), std::memory_order_relaxed);
  if (!wheel && GetTickCount64() < g_turn_block_until.load(std::memory_order_relaxed)) state->Gamepad.sThumbRX = 0;
  // Roomscale body turn (move_probe, default on, "nobodyturn" off): while the user's right stick X is idle, a value steering the
  // game's yaw toward the headset beyond a dead zone; the view is rebased by the measured turn. User X passes through.
  else if (!wheel && native_stereo::stereo_running() && !native_stereo::front_end())
    state->Gamepad.sThumbRX = to_axis(move_probe::body_turn_stick(state->Gamepad.sThumbRX / 32767.f));
  return stamp(state);
}

template <int I>
DWORD WINAPI set_state_detour(DWORD user, XINPUT_VIBRATION* vibration) {
  g_set_state_calls.fetch_add(1, std::memory_order_relaxed);
  if (user != 0) {
    return g_set_state_original[I](user, vibration);
  }
  if (g_mode == Mode::kXr && vibration != nullptr) {
    xr_input::rumble(vibration->wLeftMotorSpeed, vibration->wRightMotorSpeed);
    g_set_state_original[I](0, vibration);  // a real pad on slot 0 rumbles too
  }
  return ERROR_SUCCESS;
}

template <int I>
DWORD WINAPI get_caps_detour(DWORD user, DWORD flags, XINPUT_CAPABILITIES* caps) {
  g_get_caps_calls.fetch_add(1, std::memory_order_relaxed);
  if (user == 0 && caps != nullptr && g_mode == Mode::kXr) {
    const DWORD real = g_get_caps_original[I](0, flags, caps);
    xr_input::PadState s;
    if (real == ERROR_SUCCESS || !xr_input::latest(&s)) {
      return real;  // a real pad describes itself; with neither source slot 0 stays disconnected
    }
  }
  if (user == 0 && caps != nullptr) {
    std::memset(caps, 0, sizeof(*caps));
    caps->Type = XINPUT_DEVTYPE_GAMEPAD;
    caps->SubType = XINPUT_DEVSUBTYPE_GAMEPAD;
    caps->Gamepad.wButtons = 0xF3FF;  // every standard button
    caps->Gamepad.bLeftTrigger = caps->Gamepad.bRightTrigger = 0xFF;
    caps->Gamepad.sThumbLX = caps->Gamepad.sThumbLY = caps->Gamepad.sThumbRX = caps->Gamepad.sThumbRY = static_cast<SHORT>(0xFFC0);
    caps->Vibration.wLeftMotorSpeed = caps->Vibration.wRightMotorSpeed = 0xFF;
    return ERROR_SUCCESS;
  }
  return g_get_caps_original[I](user, flags, caps);
}

HRESULT WINAPI ro_get_activation_factory_detour(void* hstring_class, REFIID iid, void** factory) {
  if (g_raw_buffer != nullptr && hstring_class != nullptr) {
    UINT32 len = 0;
    PCWSTR name = g_raw_buffer(hstring_class, &len);
    if (name != nullptr && len == std::wcslen(kBlockedClass) && std::wcsncmp(name, kBlockedClass, len) == 0) {
      const std::uint64_t n = g_blocked.fetch_add(1, std::memory_order_relaxed) + 1;
      if (n <= 4) {
        logf("blocked RoGetActivationFactory(%ls) #%llu -> REGDB_E_CLASSNOTREG (game falls back to XInput)", kBlockedClass,
             static_cast<unsigned long long>(n));
      }
      if (factory != nullptr) {
        *factory = nullptr;
      }
      return REGDB_E_CLASSNOTREG;
    }
  }
  return g_ro_original(hstring_class, iid, factory);
}

bool hook(void* target, void* detour, void** original, const char* what) {
  if (target == nullptr) {
    logf("hook %s: not found", what);
    return false;
  }
  const MH_STATUS created = MH_CreateHook(target, detour, original);
  const MH_STATUS enabled = created == MH_OK ? MH_EnableHook(target) : created;
  logf("hook %s at %p: %s", what, target, enabled == MH_OK ? "ok" : "FAILED");
  return enabled == MH_OK;
}

template <int I>
void hook_xinput(const wchar_t* dll) {
  HMODULE m = LoadLibraryW(dll);  // the game's own LoadLibraryA then returns this module and its hooked exports
  if (m == nullptr) {
    logf("%ls: not present", dll);
    return;
  }
  char name[64] = {};
  WideCharToMultiByte(CP_UTF8, 0, dll, -1, name, sizeof(name) - 1, nullptr, nullptr);
  std::string label = std::string(name) + "!";
  hook(reinterpret_cast<void*>(GetProcAddress(m, "XInputGetState")), reinterpret_cast<void*>(&get_state_detour<I>),
       reinterpret_cast<void**>(&g_get_state_original[I]), (label + "XInputGetState").c_str());
  hook(reinterpret_cast<void*>(GetProcAddress(m, "XInputSetState")), reinterpret_cast<void*>(&set_state_detour<I>),
       reinterpret_cast<void**>(&g_set_state_original[I]), (label + "XInputSetState").c_str());
  hook(reinterpret_cast<void*>(GetProcAddress(m, "XInputGetCapabilities")), reinterpret_cast<void*>(&get_caps_detour<I>),
       reinterpret_cast<void**>(&g_get_caps_original[I]), (label + "XInputGetCapabilities").c_str());
}

}  // namespace

namespace virtual_pad {

bool* pitch_level_flag() { return &g_pitch_level; }
float take_turn_input() {
  g_turn_block_until.store(GetTickCount64() + 250, std::memory_order_relaxed);
  return g_last_rx_milli.load(std::memory_order_relaxed) * 0.001f;
}
float* pitch_target_value() { return &g_pitch_target_deg; }

void install() {
  if (g_install_attempted.exchange(true)) {
    return;
  }
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_log_path = dir + L"\\VIRTUAL_PAD.log";
  MoveFileExW(g_log_path.c_str(), (dir + L"\\VIRTUAL_PAD.prev.log").c_str(), MOVEFILE_REPLACE_EXISTING);
  g_mode = read_mode(dir);
  logf("virtual pad build %s %s pid=%lu mode=%s (scripts\\AvatarVR_pad.txt: off|test|xr)", __DATE__, __TIME__,
       GetCurrentProcessId(), mode_name(g_mode));
  if (g_mode == Mode::kOff) {
    return;
  }
  g_start_tick = GetTickCount64();

  HMODULE combase = LoadLibraryW(L"combase.dll");
  if (combase == nullptr) {
    logf("combase.dll not loadable; WGI cannot be blocked, virtual pad SKIPPED");
    return;
  }
  g_raw_buffer = reinterpret_cast<WindowsGetStringRawBufferFn>(GetProcAddress(combase, "WindowsGetStringRawBuffer"));
  if (!hook(reinterpret_cast<void*>(GetProcAddress(combase, "RoGetActivationFactory")),
            reinterpret_cast<void*>(&ro_get_activation_factory_detour), reinterpret_cast<void**>(&g_ro_original),
            "combase!RoGetActivationFactory") ||
      g_raw_buffer == nullptr) {
    logf("WGI block not armed; virtual pad SKIPPED");
    return;
  }
  hook_xinput<0>(kXinputDlls[0]);
  hook_xinput<1>(kXinputDlls[1]);
  hook_xinput<2>(kXinputDlls[2]);
  logf(g_mode == Mode::kTest ? "armed: slot 0 = scripted pad (left stick forward 2 s of every 10 s), slots 1-3 pass through"
                             : "armed: slot 0 = OpenXR controllers (XR_INPUT.log) merged with a real pad on slot 0, rumble to both; slots 1-3 pass through");
}

void on_present() {
  {
    static ULONGLONG next = 0;
    const ULONGLONG now = GetTickCount64();
    if (g_pitch_steps.load() && now >= next) {
      next = now + 5000;
      int sign = 0, flips = 0;
      {
        std::lock_guard<std::mutex> lock(g_pitch_mutex);
        sign = g_pitch.sign;
        flips = g_pitch.flips;
      }
      logf("pitch level: %s target=%.1f game pitch=%.1f deg stick=%.2f sign=%+d flips=%d steps=%u correcting=%u", g_pitch_level ? "on" : "off", g_pitch_target_deg,
           g_pitch_last_mdeg.load() * 0.001, g_pitch_last_stick.load() * 0.001, sign, flips, g_pitch_steps.load(),
           g_pitch_active.load());
    }
  }
  if (g_mode == Mode::kOff) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (now - g_last_log < 5000) {
    return;
  }
  g_last_log = now;
  logf("t=%llus wgi_blocked=%llu get_state[0..3]=%llu/%llu/%llu/%llu get_caps=%llu set_state=%llu packet=%u xr_reads=%llu",
       static_cast<unsigned long long>((now - g_start_tick) / 1000),
       static_cast<unsigned long long>(g_blocked.load()), static_cast<unsigned long long>(g_get_state_calls[0].load()),
       static_cast<unsigned long long>(g_get_state_calls[1].load()), static_cast<unsigned long long>(g_get_state_calls[2].load()),
       static_cast<unsigned long long>(g_get_state_calls[3].load()), static_cast<unsigned long long>(g_get_caps_calls.load()),
       static_cast<unsigned long long>(g_set_state_calls.load()), g_packet.load(),
       static_cast<unsigned long long>(g_xr_reads.load()));
}

}  // namespace virtual_pad
