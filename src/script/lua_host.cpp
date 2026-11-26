#include "../runtime_log.hpp"
#include <sstream>
#include "../settings_file.hpp"
#include "lua_host.hpp"

#include "../camera/native_stereo.hpp"
#include "../camera/mount_camera.hpp"
#include "../game/mount_state.hpp"
#include "../game/player.hpp"
#include "../diagnostics/bone_copy_probe.hpp"
#include "../diagnostics/pose_phase_probe.hpp"
#include "../diagnostics/move_probe.hpp"
#include "../vr/vr_weapon.hpp"
#include "../vr/hand_gestures.hpp"
#include "../diagnostics/skin_palette_probe.hpp"
#include "../vr/head_pose.hpp"
#include "../vr/xr_runtime.hpp"
#include "../vr/virtual_pad.hpp"

#include <windows.h>

#include <MinHook.h>

extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

// Design and API: docs/SCRIPTING.md. The DLL owns every hook; a reload only replaces the Lua state.
// Threads: "frame" runs on the Present thread, "camera_pre"/"camera_post" on the game thread that runs the camera
// manager update. One mutex guards the state; the game thread only try-locks, so a busy or slow script makes the
// game skip that event instead of waiting.

namespace {

// Camera manager update (research/CAMERA_FINDINGS.md "Camera manager update"): this = camera holder (handle+0x418).
// Guard: tools/re/make_sig.py, unique in readable code on 98588DA2...
constexpr std::uintptr_t kCamUpdateRva = 0x4271330;
constexpr unsigned char kCamUpdateSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x20, 0x57,
                                           0x48, 0x81, 0xEC, 0xD0, 0x00, 0x00, 0x00, 0x48, 0x8B, 0xD9};

enum Event { kFrame, kCameraPre, kCameraPost, kUnload, kEventCount };
thread_local int g_active_script_event=-1;
constexpr const char* kEventNames[kEventCount] = {"frame", "camera_pre", "camera_post", "unload"};
constexpr int kMaxTunables = 16;
constexpr float kNear = 0.05f;

struct Callback {
  int event = 0;
  int ref = LUA_NOREF;
  bool enabled = true;
  std::string where;
};

std::recursive_mutex g_mu;  // the Lua state, callbacks, tunables, building draw list
lua_State* g_L = nullptr;
std::vector<Callback> g_callbacks;
std::atomic<int> g_event_counts[kEventCount] = {};
std::atomic<bool> g_reload{false};
bool g_installed = false;
bool g_writes = true;  // on at launch since 2026-10-15 (user request: arm IK and tracked gun on by default)
std::wstring g_dir;   // ...\scripts (logs, tunables)
std::wstring g_root;  // script root with autorun\ and lib\: scripts\avr, or the folder named in scripts\avr\root.txt
std::wstring g_log_path;
char g_status[320] = "scripts not loaded";
char g_error[256] = "";
int g_loaded = 0;
std::uint64_t g_frame = 0;
LARGE_INTEGER g_qpf{}, g_qpc0{};

lua_host::Tunable g_tunables[kMaxTunables];
std::atomic<unsigned> g_rows_version{1};
std::map<std::string, float> g_saved;  // tunable values by name, kept across reloads, in scripts\avr\tunables.txt
std::string g_saved_text;

// Projection context for this frame (set before "frame" runs).
struct View {
  bool ok = false;
  float cam[16] = {};
  float tan[4] = {-1.f, 1.f, 0.5625f, -0.5625f};
};
View g_view;
std::vector<lua_host::DrawPrim> g_building;
std::mutex g_draw_mu;
std::vector<lua_host::DrawPrim> g_ready;

using CamUpdateFn = void(__fastcall*)(void*);
CamUpdateFn g_cam_update_orig = nullptr;
std::atomic<void*> g_holder{nullptr};
std::atomic<std::uint64_t> g_cam_skipped{0};

// ---- logging ---------------------------------------------------------------------------------------------

void write_line(const char* text) {
  if (g_log_path.empty()) {
    return;
  }
  SYSTEMTIME st;
  GetLocalTime(&st);
  char stamp[32];
  snprintf(stamp, sizeof(stamp), "%02d:%02d:%02d.%03d ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
  runtime_log::append(g_log_path, {stamp, text, "\r\n"});
}

void logf(const char* fmt, ...) {
  char buf[1024];
  va_list a;
  va_start(a, fmt);
  vsnprintf(buf, sizeof(buf), fmt, a);
  va_end(a);
  buf[sizeof(buf) - 1] = '\0';
  write_line(buf);
}

void set_error(const char* msg) {
  snprintf(g_error, sizeof(g_error), "%s", msg);
  logf("ERROR %s", msg);
}

std::string narrow(const std::wstring& w) {
  if (w.empty()) {
    return {};
  }
  const int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<size_t>(n), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.c_str(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
  return s;
}

std::wstring module_dir() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&module_dir), &self)) {
    return {};
  }
  wchar_t path[MAX_PATH] = {};
  const DWORD n = GetModuleFileNameW(self, path, MAX_PATH);
  std::wstring dir(path, n);
  const auto slash = dir.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : dir.substr(0, slash);
}

// ---- tunables file ---------------------------------------------------------------------------------------

std::wstring tunables_path() { return g_dir + L"\\avr\\tunables.txt"; }

void load_saved() {
  std::string text;
  if (!settings_file::read(tunables_path(), text)) return;
  std::istringstream input(text);
  for (std::string line; std::getline(input, line);) {
    const auto eq = line.find('=');
    if (eq != std::string::npos)
      g_saved[line.substr(0, eq)] = static_cast<float>(std::atof(line.c_str() + eq + 1));
  }
}

// Called once a second: menu edits land in g_tunables; keep g_saved and the file in step.
void save_tunables() {
  for (const auto& t : g_tunables) {
    if (t.used) {
      g_saved[t.name] = t.toggle ? (t.on ? 1.f : 0.f) : t.value;
    }
  }
  std::string text;
  char line[160];
  for (const auto& kv : g_saved) {
    snprintf(line, sizeof(line), "%s=%g\n", kv.first.c_str(), kv.second);
    text += line;
  }
  if (text == g_saved_text) {
    return;
  }
  if (settings_file::write(tunables_path(), text)) g_saved_text = text;
}

lua_host::Tunable* find_tunable(const char* name, bool create) {
  for (auto& t : g_tunables) {
    if (t.used && std::strcmp(t.name, name) == 0) {
      return &t;
    }
  }
  if (!create) {
    return nullptr;
  }
  for (auto& t : g_tunables) {
    if (!t.used) {
      t = lua_host::Tunable{};
      snprintf(t.name, sizeof(t.name), "%s", name);
      t.used = true;
      g_rows_version.fetch_add(1);
      return &t;
    }
  }
  return nullptr;
}

// ---- projection ------------------------------------------------------------------------------------------

// World point -> camera view space (x right, y up, z forward).
void to_view(const float p[3], float v[3]) {
  const float* m = g_view.cam;
  const float d[3] = {p[0] - m[12], p[1] - m[13], p[2] - m[14]};
  for (int r = 0; r < 3; ++r) {
    v[r] = d[0] * m[4 * r] + d[1] * m[4 * r + 1] + d[2] * m[4 * r + 2];
  }
}

void view_to_screen(const float v[3], float* x, float* y) {
  const float tx = v[0] / v[2], ty = v[1] / v[2];
  *x = (tx - g_view.tan[0]) / (g_view.tan[1] - g_view.tan[0]);
  *y = (g_view.tan[2] - ty) / (g_view.tan[2] - g_view.tan[3]);
}

void add_line(const float a[3], const float b[3], std::uint32_t rgba, float width) {
  if (!g_view.ok) {
    return;
  }
  float va[3], vb[3];
  to_view(a, va);
  to_view(b, vb);
  if (va[2] < kNear && vb[2] < kNear) {
    return;
  }
  auto clip = [](float* behind, const float* front) {  // move the behind end onto the near plane
    const float t = (kNear - behind[2]) / (front[2] - behind[2]);
    for (int k = 0; k < 3; ++k) {
      behind[k] += (front[k] - behind[k]) * t;
    }
  };
  if (va[2] < kNear) {
    clip(va, vb);
  } else if (vb[2] < kNear) {
    clip(vb, va);
  }
  lua_host::DrawPrim p;
  p.kind = lua_host::DrawPrim::Line;
  view_to_screen(va, &p.x0, &p.y0);
  view_to_screen(vb, &p.x1, &p.y1);
  p.rgba = rgba;
  p.size = width;
  g_building.push_back(std::move(p));
}

bool project(const float w[3], float* x, float* y) {
  if (!g_view.ok) {
    return false;
  }
  float v[3];
  to_view(w, v);
  if (v[2] < kNear) {
    return false;
  }
  view_to_screen(v, x, y);
  return true;
}

// ---- Lua helpers -----------------------------------------------------------------------------------------

void push_vec(lua_State* L, float x, float y, float z) {
  lua_createtable(L, 0, 3);
  lua_pushnumber(L, x);
  lua_setfield(L, -2, "x");
  lua_pushnumber(L, y);
  lua_setfield(L, -2, "y");
  lua_pushnumber(L, z);
  lua_setfield(L, -2, "z");
}

void push_vec(lua_State* L, const float* v) { push_vec(L, v[0], v[1], v[2]); }

// {x=,y=,z=} or {1,2,3}
bool get_vec(lua_State* L, int idx, float out[3]) {
  if (!lua_istable(L, idx)) {
    return false;
  }
  idx = lua_absindex(L, idx);
  static const char* names[3] = {"x", "y", "z"};
  for (int k = 0; k < 3; ++k) {
    lua_getfield(L, idx, names[k]);
    if (lua_isnumber(L, -1)) {
      out[k] = static_cast<float>(lua_tonumber(L, -1));
      lua_pop(L, 1);
      continue;
    }
    lua_pop(L, 1);
    lua_rawgeti(L, idx, k + 1);
    if (!lua_isnumber(L, -1)) {
      lua_pop(L, 1);
      return false;
    }
    out[k] = static_cast<float>(lua_tonumber(L, -1));
    lua_pop(L, 1);
  }
  return true;
}

void check_vec(lua_State* L, int idx, float out[3]) {
  if (!get_vec(L, idx, out)) {
    luaL_error(L, "argument %d: expected a vector {x=,y=,z=}", idx);
  }
}

// Matrix rows -> {pos=, right=, up=, fwd=}
void push_xform(lua_State* L, const float m[16]) {
  lua_createtable(L, 0, 4);
  push_vec(L, m + 12);
  lua_setfield(L, -2, "pos");
  push_vec(L, m + 0);
  lua_setfield(L, -2, "right");
  push_vec(L, m + 4);
  lua_setfield(L, -2, "up");
  push_vec(L, m + 8);
  lua_setfield(L, -2, "fwd");
}

std::uint32_t opt_color(lua_State* L, int idx, std::uint32_t def) {
  return lua_isinteger(L, idx) ? static_cast<std::uint32_t>(lua_tointeger(L, idx)) : def;
}

std::string concat_args(lua_State* L) {
  std::string s;
  const int n = lua_gettop(L);
  for (int i = 1; i <= n; ++i) {
    if (i > 1) {
      s += ' ';
    }
    s += luaL_tolstring(L, i, nullptr);
    lua_pop(L, 1);
  }
  return s;
}

// ---- API: core and memory --------------------------------------------------------------------------------

int l_log(lua_State* L) {
  const std::string s = concat_args(L);
  logf("[lua] %s", s.c_str());
  return 0;
}

int l_time(lua_State* L) {
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  lua_pushnumber(L, static_cast<double>(now.QuadPart - g_qpc0.QuadPart) / static_cast<double>(g_qpf.QuadPart));
  return 1;
}

int l_frame(lua_State* L) {
  lua_pushinteger(L, static_cast<lua_Integer>(g_frame));
  return 1;
}

int l_base(lua_State* L) {
  lua_pushinteger(L, static_cast<lua_Integer>(reinterpret_cast<std::uintptr_t>(game::module_base())));
  return 1;
}

int l_writes(lua_State* L) {
  lua_pushboolean(L, g_writes);
  return 1;
}

template <typename T>
int read_impl(lua_State* L) {
  const auto addr = static_cast<std::uintptr_t>(luaL_checkinteger(L, 1));
  T v{};
  if (addr == 0 || !game::mem_read(reinterpret_cast<const void*>(addr), &v, sizeof(v))) {
    lua_pushnil(L);
    return 1;
  }
  if constexpr (std::is_floating_point_v<T>) {
    lua_pushnumber(L, static_cast<lua_Number>(v));
  } else {
    lua_pushinteger(L, static_cast<lua_Integer>(v));
  }
  return 1;
}

template <typename T>
int write_impl(lua_State* L) {
  const auto addr = static_cast<std::uintptr_t>(luaL_checkinteger(L, 1));
  T v;
  if constexpr (std::is_floating_point_v<T>) {
    v = static_cast<T>(luaL_checknumber(L, 2));
  } else {
    v = static_cast<T>(luaL_checkinteger(L, 2));
  }
  lua_pushboolean(L, g_writes && addr != 0 && game::mem_write(reinterpret_cast<void*>(addr), &v, sizeof(v)));
  return 1;
}

// ---- API: game -------------------------------------------------------------------------------------------

int l_player(lua_State* L) {
  const game::Player p = game::read();
  if (!p.valid) {
    lua_pushnil(L);
    return 1;
  }
  push_xform(L, p.cam);
  push_vec(L, p.body.x, p.body.y, p.body.z);
  lua_setfield(L, -2, "body");
  lua_pushboolean(L, p.look_ok);
  lua_setfield(L, -2, "look_ok");
  lua_pushnumber(L, p.yaw);
  lua_setfield(L, -2, "yaw");
  lua_pushnumber(L, p.pitch);
  lua_setfield(L, -2, "pitch");
  return 1;
}

// FEATURE-06: {mounted, phase, since, camera_mode, movement_state, mount_type, partner, transitions}; phase is
// "none" / "mounting" / "riding" / "dismounting", since = seconds in this phase.
int l_mount(lua_State* L) {
  const mount_state::State m = mount_state::get();
  lua_createtable(L, 0, 8);
  lua_pushboolean(L, m.phase != mount_state::Phase::None);
  lua_setfield(L, -2, "mounted");
  lua_pushstring(L, mount_state::name(m.phase));
  lua_setfield(L, -2, "phase");
  lua_pushnumber(L, m.since_ms ? static_cast<lua_Number>(GetTickCount64() - m.since_ms) * 1e-3 : 0.0);
  lua_setfield(L, -2, "since");
  lua_pushinteger(L, m.camera_mode);
  lua_setfield(L, -2, "camera_mode");
  lua_pushinteger(L, m.movement_state);
  lua_setfield(L, -2, "movement_state");
  lua_pushinteger(L, m.mount_type);
  lua_setfield(L, -2, "mount_type");
  lua_pushinteger(L, static_cast<lua_Integer>(reinterpret_cast<std::uintptr_t>(m.partner)));
  lua_setfield(L, -2, "partner");
  lua_pushinteger(L, static_cast<lua_Integer>(m.transitions));
  lua_setfield(L, -2, "transitions");
  return 1;
}

int l_pointers(lua_State* L) {
  const game::Pointers p = game::pointers();
  lua_createtable(L, 0, 7);
  auto put = [&](const char* k, const void* v) {
    lua_pushinteger(L, static_cast<lua_Integer>(reinterpret_cast<std::uintptr_t>(v)));
    lua_setfield(L, -2, k);
  };
  put("handle", p.handle);
  put("entity", p.entity);
  put("body", p.body);
  put("holder", p.holder);
  put("camera", p.camera);
  put("look", p.look);
  put("pose", p.pose);
  return 1;
}

int l_collision_context(lua_State* L) {
  const auto context=game::collision_context();
  lua_createtable(L,0,6);
  const void* addresses[]={context.world,context.owner,context.query,context.ray};
  const char* names[]={"world","owner","query","ray"};
  for (int i=0;i<4;++i) {
    lua_pushinteger(L,static_cast<lua_Integer>(reinterpret_cast<std::uintptr_t>(addresses[i])));
    lua_setfield(L,-2,names[i]);
  }
  lua_pushboolean(L,context.known_interface); lua_setfield(L,-2,"known_interface");
  lua_pushboolean(L,context.thread_ready); lua_setfield(L,-2,"thread_ready");
  return 1;
}

int l_raycast_camera(lua_State* L) {
  if (!g_writes || (g_active_script_event!=kCameraPre && g_active_script_event!=kCameraPost)) {
    lua_pushnil(L); lua_pushliteral(L,"requires Script writes and camera_pre/post"); return 2;
  }
  float start[3],end[3]; check_vec(L,1,start); check_vec(L,2,end);
  const auto result=game::raycast_camera_filter({start[0],start[1],start[2]},{end[0],end[1],end[2]});
  lua_createtable(L,0,5);
  lua_pushboolean(L,result.executed); lua_setfield(L,-2,"executed");
  lua_pushboolean(L,result.hit); lua_setfield(L,-2,"hit");
  if (result.hit) {
    push_vec(L,result.position.x,result.position.y,result.position.z); lua_setfield(L,-2,"pos");
    push_vec(L,result.normal.x,result.normal.y,result.normal.z); lua_setfield(L,-2,"normal");
    lua_pushnumber(L,result.distance); lua_setfield(L,-2,"distance");
  }
  return 1;
}

int l_bone_count(lua_State* L) {
  lua_pushinteger(L, game::bone_count(game::pointers()));
  return 1;
}

int l_bone(lua_State* L) {
  float m[16];
  if (!game::bone_world(game::pointers(), static_cast<int>(luaL_checkinteger(L, 1)), m)) {
    lua_pushnil(L);
    return 1;
  }
  push_xform(L, m);
  return 1;
}

// All bone positions as {[0]=pos, [1]=pos, ...} (one pointer walk for the whole pose).
int l_bones(lua_State* L) {
  const game::Pointers p = game::pointers();
  const int n = game::bone_count(p);
  lua_createtable(L, n, 0);
  float m[16];
  for (int i = 0; i < n; ++i) {
    if (game::bone_world(p, i, m)) {
      push_vec(L, m + 12);
      lua_rawseti(L, -2, i);
    }
  }
  return 1;
}

int l_set_look(lua_State* L) {
  const float yaw = static_cast<float>(luaL_checknumber(L, 1));
  const float pitch = static_cast<float>(luaL_checknumber(L, 2));
  lua_pushboolean(L, g_writes && game::set_look(game::pointers(), yaw, pitch));
  return 1;
}

// avr.set_bone(i, {pos=, right=, up=, fwd=}); missing parts keep the bone's current value.
int l_set_bone(lua_State* L) {
  const int i = static_cast<int>(luaL_checkinteger(L, 1));
  luaL_checktype(L, 2, LUA_TTABLE);
  const game::Pointers p = game::pointers();
  float m[16];
  if (!g_writes || !game::bone_world(p, i, m)) {
    lua_pushboolean(L, 0);
    return 1;
  }
  static const char* parts[4] = {"right", "up", "fwd", "pos"};
  for (int r = 0; r < 4; ++r) {
    lua_getfield(L, 2, parts[r]);
    float v[3];
    if (get_vec(L, -1, v)) {
      std::memcpy(m + 4 * r, v, sizeof(v));
    }
    lua_pop(L, 1);
  }
  lua_pushboolean(L, game::set_bone_world(p, i, m));
  return 1;
}

// ---- API: XR and stereo ----------------------------------------------------------------------------------

void push_pose(lua_State* L, float px, float py, float pz, float qx, float qy, float qz, float qw, bool valid) {
  lua_createtable(L, 0, 3);
  push_vec(L, px, py, pz);
  lua_setfield(L, -2, "pos");
  lua_createtable(L, 0, 4);
  lua_pushnumber(L, qx);
  lua_setfield(L, -2, "x");
  lua_pushnumber(L, qy);
  lua_setfield(L, -2, "y");
  lua_pushnumber(L, qz);
  lua_setfield(L, -2, "z");
  lua_pushnumber(L, qw);
  lua_setfield(L, -2, "w");
  lua_setfield(L, -2, "rot");
  lua_pushboolean(L, valid);
  lua_setfield(L, -2, "valid");
}

// OpenXR LOCAL space (x right, y up, -z forward, metres): {head=, eyes={[0],[1]}, aim={[0],[1]}, grip={[0],[1]}}
int l_xr_views(lua_State* L) {
  head_pose::Snapshot s;
  if (!xr_runtime::latest_views(&s) || !s.valid) {
    lua_pushnil(L);
    return 1;
  }
  lua_createtable(L, 0, 4);
  const auto& e0 = s.eye[0];
  const auto& e1 = s.eye[1];
  push_pose(L, (e0.px + e1.px) * 0.5f, (e0.py + e1.py) * 0.5f, (e0.pz + e1.pz) * 0.5f, e0.qx, e0.qy, e0.qz, e0.qw, true);
  lua_setfield(L, -2, "head");
  lua_createtable(L, 2, 0);
  for (int i = 0; i < 2; ++i) {
    push_pose(L, s.eye[i].px, s.eye[i].py, s.eye[i].pz, s.eye[i].qx, s.eye[i].qy, s.eye[i].qz, s.eye[i].qw, true);
    lua_rawseti(L, -2, i);
  }
  lua_setfield(L, -2, "eyes");
  const head_pose::Hand* sets[2] = {s.aim, s.grip};
  const char* names[2] = {"aim", "grip"};
  for (int k = 0; k < 2; ++k) {
    lua_createtable(L, 2, 0);
    for (int i = 0; i < 2; ++i) {
      const auto& h = sets[k][i];
      push_pose(L, h.px, h.py, h.pz, h.qx, h.qy, h.qz, h.qw, h.valid);
      lua_rawseti(L, -2, i);
    }
    lua_setfield(L, -2, names[k]);
  }
  return 1;
}

// Controller ray on the game camera basis (world), as native_stereo builds it: origin, direction (or nil).
int l_aim_ray(lua_State* L) {
  float org[3], dir[3];
  if (!native_stereo::aim_ray(static_cast<int>(luaL_checkinteger(L, 1)), org, dir)) {
    lua_pushnil(L);
    return 1;
  }
  push_vec(L, org);
  push_vec(L, dir);
  return 2;
}

int l_stereo_running(lua_State* L) {
  lua_pushboolean(L, native_stereo::stereo_running());
  return 1;
}

int l_grip_transform(lua_State* L) {
  float matrix[16];
  if (!native_stereo::grip_transform(static_cast<int>(luaL_checkinteger(L, 1)), matrix)) {
    lua_pushnil(L);
    return 1;
  }
  lua_createtable(L, 0, 4);
  const char* fields[] = {"right", "up", "fwd", "pos"};
  for (int i = 0; i < 4; ++i) {
    push_vec(L, matrix + i * 4);
    lua_setfield(L, -2, fields[i]);
  }
  return 1;
}

int l_turn_input(lua_State* L) {
  lua_pushnumber(L, virtual_pad::take_turn_input());
  return 1;
}

int l_body_yaw(lua_State* L) {
  if (lua_isnoneornil(L, 1)) {
    native_stereo::clear_body_yaw();
  } else if (g_writes) {
    const float yaw = static_cast<float>(luaL_checknumber(L, 1));
    if (!std::isfinite(yaw)) return luaL_error(L, "body yaw must be finite");
    native_stereo::set_body_yaw(yaw);
  }
  return 0;
}

// ---- API: events and tunables ----------------------------------------------------------------------------

int l_on(lua_State* L) {
  const char* name = luaL_checkstring(L, 1);
  luaL_checktype(L, 2, LUA_TFUNCTION);
  int ev = -1;
  for (int i = 0; i < kEventCount; ++i) {
    if (std::strcmp(name, kEventNames[i]) == 0) {
      ev = i;
    }
  }
  if (ev < 0) {
    return luaL_error(L, "unknown event '%s' (frame, camera_pre, camera_post, unload)", name);
  }
  lua_Debug ar{};
  std::string where = "?";
  if (lua_getstack(L, 1, &ar) && lua_getinfo(L, "Sl", &ar)) {
    where = std::string(ar.short_src) + ":" + std::to_string(ar.currentline);
  }
  lua_pushvalue(L, 2);
  Callback cb;
  cb.event = ev;
  cb.ref = luaL_ref(L, LUA_REGISTRYINDEX);
  cb.where = where;
  g_callbacks.push_back(cb);
  g_event_counts[ev].fetch_add(1);
  return 0;
}

// avr.tunable(name, default, lo, hi, step) -> current value (menu row; value kept across reloads)
int l_tunable(lua_State* L) {
  const char* name = luaL_checkstring(L, 1);
  const float def = static_cast<float>(luaL_checknumber(L, 2));
  lua_host::Tunable* t = find_tunable(name, true);
  if (t == nullptr) {
    lua_pushnumber(L, def);  // more than kMaxTunables: works, but no menu row
    return 1;
  }
  if (!t->ready || t->toggle) {
    t->ready = true;
    t->toggle = false;
    t->lo = static_cast<float>(luaL_optnumber(L, 3, def < 0 ? def * 2 : 0.0));
    t->hi = static_cast<float>(luaL_optnumber(L, 4, def == 0 ? 1.0 : std::fabs(def) * 2));
    t->step = static_cast<float>(luaL_optnumber(L, 5, (t->hi - t->lo) / 50.0));
    const auto it = g_saved.find(name);
    t->value = std::clamp(it != g_saved.end() ? it->second : def, t->lo, t->hi);
  }
  lua_pushnumber(L, t->value);
  return 1;
}

// avr.toggle(name, default) -> bool (menu row)
int l_toggle(lua_State* L) {
  const char* name = luaL_checkstring(L, 1);
  const bool def = lua_toboolean(L, 2) != 0;
  lua_host::Tunable* t = find_tunable(name, true);
  if (t == nullptr) {
    lua_pushboolean(L, def);
    return 1;
  }
  if (!t->ready || !t->toggle) {
    t->ready = true;
    t->toggle = true;
    const auto it = g_saved.find(name);
    t->on = it != g_saved.end() ? it->second > 0.5f : def;
  }
  lua_pushboolean(L, t->on);
  return 1;
}

// avr.value(name) -> current tunable value (number or bool), or nil
// avr.weapon_hand(): 1 right (default), 0 left ("Weapon hand" menu, run 59).
int l_weapon_hand(lua_State* L) {
  lua_pushinteger(L, skin_palette_probe::weapon_hand_index());
  return 1;
}

int l_value(lua_State* L) {
  const lua_host::Tunable* t = find_tunable(luaL_checkstring(L, 1), false);
  if (t == nullptr) {
    lua_pushnil(L);
  } else if (t->toggle) {
    lua_pushboolean(L, t->on);
  } else {
    lua_pushnumber(L, t->value);
  }
  return 1;
}

// ---- API: drawing ----------------------------------------------------------------------------------------

int l_draw_line(lua_State* L) {
  float a[3], b[3];
  check_vec(L, 1, a);
  check_vec(L, 2, b);
  add_line(a, b, opt_color(L, 3, 0xFFFFFFFF), static_cast<float>(luaL_optnumber(L, 4, 2.0)));
  return 0;
}

int l_draw_point(lua_State* L) {
  float p[3], x, y;
  check_vec(L, 1, p);
  if (project(p, &x, &y)) {
    lua_host::DrawPrim d;
    d.kind = lua_host::DrawPrim::Point;
    d.x0 = x;
    d.y0 = y;
    d.size = static_cast<float>(luaL_optnumber(L, 2, 5.0));
    d.rgba = opt_color(L, 3, 0xFFFFFFFF);
    g_building.push_back(std::move(d));
  }
  return 0;
}

int l_draw_text(lua_State* L) {
  float p[3], x, y;
  check_vec(L, 1, p);
  const char* s = luaL_checkstring(L, 2);
  if (project(p, &x, &y)) {
    lua_host::DrawPrim d;
    d.kind = lua_host::DrawPrim::Text;
    d.x0 = x;
    d.y0 = y;
    d.rgba = opt_color(L, 3, 0xFFFFFFFF);
    d.text = s;
    g_building.push_back(std::move(d));
  }
  return 0;
}

// avr.draw.screen_text(x, y, text [, rgba]) with x, y as fractions of the image (0..1)
int l_draw_screen_text(lua_State* L) {
  lua_host::DrawPrim d;
  d.kind = lua_host::DrawPrim::Text;
  d.x0 = static_cast<float>(luaL_checknumber(L, 1));
  d.y0 = static_cast<float>(luaL_checknumber(L, 2));
  d.text = luaL_checkstring(L, 3);
  d.rgba = opt_color(L, 4, 0xFFFFFFFF);
  g_building.push_back(std::move(d));
  return 0;
}

// ---- state -----------------------------------------------------------------------------------------------

void set_fn(lua_State* L, const char* name, lua_CFunction fn) {
  lua_pushcfunction(L, fn);
  lua_setfield(L, -2, name);
}

void register_api(lua_State* L) {
  lua_newtable(L);  // avr
  set_fn(L, "log", l_log);
  set_fn(L, "time", l_time);
  set_fn(L, "frame", l_frame);
  set_fn(L, "base", l_base);
  set_fn(L, "writes", l_writes);
  set_fn(L, "read_f32", read_impl<float>);
  set_fn(L, "read_u32", read_impl<std::uint32_t>);
  set_fn(L, "read_i32", read_impl<std::int32_t>);
  set_fn(L, "read_u64", read_impl<std::uint64_t>);
  set_fn(L, "read_ptr", read_impl<std::uint64_t>);
  set_fn(L, "write_f32", write_impl<float>);
  set_fn(L, "write_u32", write_impl<std::uint32_t>);
  set_fn(L, "player", l_player);
  set_fn(L, "pointers", l_pointers);
  set_fn(L, "mount", l_mount);
  set_fn(L, "bone_count", l_bone_count);
  set_fn(L, "bone", l_bone);
  set_fn(L, "bones", l_bones);
  set_fn(L, "set_look", l_set_look);
  set_fn(L, "collision_context", l_collision_context);
  set_fn(L, "raycast_camera", l_raycast_camera);
  set_fn(L, "set_bone", l_set_bone);
  set_fn(L, "on", l_on);
  set_fn(L, "tunable", l_tunable);
  set_fn(L, "toggle", l_toggle);
  set_fn(L, "value", l_value);
  set_fn(L, "weapon_hand", l_weapon_hand);
  lua_newtable(L);
  set_fn(L, "views", l_xr_views);
  lua_setfield(L, -2, "xr");
  lua_newtable(L);
  set_fn(L, "aim_ray", l_aim_ray);
  set_fn(L, "grip_transform", l_grip_transform);
  set_fn(L, "body_yaw", l_body_yaw);
  set_fn(L, "turn_input", l_turn_input);
  set_fn(L, "running", l_stereo_running);
  lua_setfield(L, -2, "stereo");
  lua_newtable(L);
  set_fn(L, "line", l_draw_line);
  set_fn(L, "point", l_draw_point);
  set_fn(L, "text", l_draw_text);
  set_fn(L, "screen_text", l_draw_screen_text);
  lua_setfield(L, -2, "draw");
  lua_pushstring(L, narrow(g_root).c_str());
  lua_setfield(L, -2, "dir");
  lua_setglobal(L, "avr");
  // require() finds scripts\avr\lib\*.lua
  lua_getglobal(L, "package");
  const std::string path = narrow(g_root) + "\\lib\\?.lua";
  lua_pushstring(L, path.c_str());
  lua_setfield(L, -2, "path");
  lua_pop(L, 1);
}

int traceback(lua_State* L) {
  luaL_traceback(L, L, lua_tostring(L, 1), 1);
  return 1;
}

// Runs the function on top of the stack with no arguments under a traceback handler. Pops it.
bool pcall0(lua_State* L, std::string* err) {
  const int fn = lua_gettop(L);
  lua_pushcfunction(L, traceback);
  lua_insert(L, fn);
  const int rc = lua_pcall(L, 0, 0, fn);
  if (rc != LUA_OK) {
    *err = lua_tostring(L, -1) ? lua_tostring(L, -1) : "unknown error";
    lua_pop(L, 1);
  }
  lua_remove(L, fn);
  return rc == LUA_OK;
}

void fire_locked(int ev) {
  if (g_L == nullptr) {
    return;
  }
  const int previous_event=g_active_script_event;
  g_active_script_event=ev;
  for (auto& cb : g_callbacks) {
    if (cb.event != ev || !cb.enabled) {
      continue;
    }
    lua_rawgeti(g_L, LUA_REGISTRYINDEX, cb.ref);
    std::string err;
    if (!pcall0(g_L, &err)) {
      cb.enabled = false;
      const std::string msg = std::string(kEventNames[ev]) + " callback (" + cb.where + ") disabled: " + err;
      set_error(msg.c_str());
    }
  }
  g_active_script_event=previous_event;
}

void close_state() {
  if (g_L != nullptr) {
    fire_locked(kUnload);
    lua_close(g_L);
    g_L = nullptr;
  }
  g_callbacks.clear();
  for (auto& c : g_event_counts) {
    c.store(0);
  }
  for (auto& t : g_tunables) {
    t = lua_host::Tunable{};
  }
  g_rows_version.fetch_add(1);
  g_building.clear();
}

void load_all() {
  native_stereo::clear_body_yaw();
  std::lock_guard<std::recursive_mutex> lock(g_mu);
  save_tunables();
  close_state();
  g_error[0] = '\0';
  g_loaded = 0;
  g_L = luaL_newstate();
  if (g_L == nullptr) {
    set_error("luaL_newstate failed");
    return;
  }
  luaL_openlibs(g_L);
  register_api(g_L);
  // autorun\*.lua in name order
  std::vector<std::wstring> files;
  WIN32_FIND_DATAW fd;
  const std::wstring autorun = g_root + L"\\autorun";
  HANDLE h = FindFirstFileW((autorun + L"\\*.lua").c_str(), &fd);
  if (h != INVALID_HANDLE_VALUE) {
    do {
      if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0) {
        files.push_back(fd.cFileName);
      }
    } while (FindNextFileW(h, &fd));
    FindClose(h);
  }
  std::sort(files.begin(), files.end());
  for (const auto& f : files) {
    const std::string path = narrow(autorun + L"\\" + f);
    std::string err;
    if (luaL_loadfile(g_L, path.c_str()) != LUA_OK) {
      err = lua_tostring(g_L, -1) ? lua_tostring(g_L, -1) : "load error";
      lua_pop(g_L, 1);
      set_error(err.c_str());
      continue;
    }
    if (!pcall0(g_L, &err)) {
      set_error(err.c_str());
      continue;
    }
    ++g_loaded;
    logf("loaded %s", narrow(f).c_str());
  }
  logf("reload: %d of %zu scripts loaded, %zu callbacks (frame %d, camera_pre %d, camera_post %d)", g_loaded,
       files.size(), g_callbacks.size(), g_event_counts[kFrame].load(), g_event_counts[kCameraPre].load(),
       g_event_counts[kCameraPost].load());
}

// ---- hook ------------------------------------------------------------------------------------------------

void fire_game_thread(int ev) {
  if (g_event_counts[ev].load(std::memory_order_relaxed) == 0) {
    return;
  }
  std::unique_lock<std::recursive_mutex> lock(g_mu, std::try_to_lock);
  if (!lock.owns_lock()) {
    g_cam_skipped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  fire_locked(ev);
}

void __fastcall cam_update_detour(void* holder) {
  const bool mine = holder != nullptr && holder == g_holder.load(std::memory_order_relaxed);
  if (mine) {
    mount_camera::on_camera(holder, false);
    pose_phase_probe::observe(pose_phase_probe::Phase::CameraPre);
    move_probe::on_camera(false);
    fire_game_thread(kCameraPre);
  }
  g_cam_update_orig(holder);
  if (mine) {
    mount_camera::on_camera(holder, true);
    skin_palette_probe::on_camera_post();
    vr_weapon::on_camera_post();
    hand_gestures::on_camera_post();
    fire_game_thread(kCameraPost);
    pose_phase_probe::observe(pose_phase_probe::Phase::CameraPost);
    move_probe::on_camera(true);
  }
}

void install_hook() {
  unsigned char* target = game::module_base() + kCamUpdateRva;
  unsigned char got[sizeof(kCamUpdateSig)] = {};
  if (!game::mem_read(target, got, sizeof(got)) || std::memcmp(got, kCamUpdateSig, sizeof(got)) != 0) {
    logf("camera hook SKIP: guard mismatch at +0x%llX (different build?); camera_pre/post will not fire",
         static_cast<unsigned long long>(kCamUpdateRva));
    return;
  }
  const MH_STATUS c = MH_CreateHook(target, reinterpret_cast<void*>(&cam_update_detour),
                                    reinterpret_cast<void**>(&g_cam_update_orig));
  const MH_STATUS e = c == MH_OK ? MH_EnableHook(target) : c;
  logf("camera hook +0x%llX %s (%d)", static_cast<unsigned long long>(kCamUpdateRva), e == MH_OK ? "on" : "FAILED",
       static_cast<int>(e));
}

void install() {
  g_installed = true;
  QueryPerformanceFrequency(&g_qpf);
  QueryPerformanceCounter(&g_qpc0);
  g_dir = module_dir();
  if (g_dir.empty()) {
    return;
  }
  g_log_path = g_dir + L"\\SCRIPTS.log";
  CreateDirectoryW((g_dir + L"\\avr").c_str(), nullptr);
  CreateDirectoryW((g_dir + L"\\avr\\autorun").c_str(), nullptr);
  CreateDirectoryW((g_dir + L"\\avr\\lib").c_str(), nullptr);
  logf("lua host build %s %s, %s, scripts in scripts\\avr\\autorun (F7 or menu Scripts tab reloads)", __DATE__,
       __TIME__, LUA_RELEASE);
  // scripts\avr\root.txt may name another script root (e.g. the repo's lua\ folder, so edits stay versioned).
  g_root = g_dir + L"\\avr";
  std::string root_setting;
  if (settings_file::read(g_root + L"\\root.txt", root_setting)) {
    const auto newline = root_setting.find_first_of("\r\n");
    std::string root = root_setting.substr(0, newline);
    while (!root.empty() && (root.back() == ' ' || root.back() == '\\')) root.pop_back();
    wchar_t path[MAX_PATH] = {};
    if (!root.empty() && MultiByteToWideChar(CP_UTF8, 0, root.c_str(), -1, path, MAX_PATH) > 0 &&
        GetFileAttributesW(path) != INVALID_FILE_ATTRIBUTES) g_root = path;
  }
  logf("script root %s", narrow(g_root).c_str());
  load_saved();
  install_hook();
  load_all();
}

}  // namespace

namespace lua_host {

void on_present() {
  mount_camera::on_present();
  pose_phase_probe::on_present();
  move_probe::on_present();
  vr_weapon::on_present();
  bone_copy_probe::on_present();
  skin_palette_probe::on_present();
  if (!g_installed) {
    install();
  }
  if (g_dir.empty()) {
    return;
  }
  ++g_frame;
  g_holder.store(game::pointers().holder, std::memory_order_relaxed);
  if (g_reload.exchange(false)) {
    load_all();
  }
  {
    std::lock_guard<std::recursive_mutex> lock(g_mu);
    View v;
    const game::Player p = game::read();
    v.ok = p.valid;
    std::memcpy(v.cam, p.cam, sizeof(v.cam));
    float t[4];
    if (native_stereo::game_frustum(t)) {
      std::memcpy(v.tan, t, sizeof(t));
    }
    g_view = v;
    g_building.clear();
    fire_locked(kFrame);
    std::lock_guard<std::mutex> dl(g_draw_mu);
    g_ready.swap(g_building);
    g_building.clear();
  }
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last >= 1000) {
    last = now;
    std::lock_guard<std::recursive_mutex> lock(g_mu);
    save_tunables();
    int enabled = 0;
    for (const auto& cb : g_callbacks) {
      enabled += cb.enabled ? 1 : 0;
    }
    snprintf(g_status, sizeof(g_status), "%d scripts, %d/%zu callbacks running, writes %s%s%s", g_loaded, enabled,
             g_callbacks.size(), g_writes ? "ON" : "off", g_error[0] ? " | " : "", g_error);
  }
}

void request_reload() { g_reload.store(true); }

bool* writes_flag() { return &g_writes; }

const char* status() { return g_status; }

int tunable_count() {
  int n = 0;
  for (const auto& t : g_tunables) {
    n += t.used ? 1 : 0;
  }
  return n;
}

Tunable* tunable(int index) {
  for (auto& t : g_tunables) {
    if (t.used && index-- == 0) {
      return &t;
    }
  }
  return nullptr;
}

unsigned rows_version() { return g_rows_version.load(); }

bool copy_draws(std::vector<DrawPrim>* out) {
  std::lock_guard<std::mutex> lock(g_draw_mu);
  *out = g_ready;
  return !out->empty();
}

}  // namespace lua_host
