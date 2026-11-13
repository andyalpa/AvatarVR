#include "wrist_hud.hpp"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <mutex>
#include <string>
#include "../settings_file.hpp"

namespace wrist_hud {
namespace {
Settings g_s;          // edited by the menu on the Present thread
Settings g_published;  // copy handed to other threads
std::mutex g_mutex;
Settings g_saved;
std::wstring g_path;
std::atomic<bool> g_image_ready{false};
std::atomic<ULONGLONG> g_submitted_at{0};

void clamp_crop(Crop& c) {
  c.x = std::clamp(c.x, 0.f, 0.95f);
  c.y = std::clamp(c.y, 0.f, 0.95f);
  c.w = std::clamp(c.w, 0.05f, 1.f - c.x);
  c.h = std::clamp(c.h, 0.05f, 1.f - c.y);
}

void sanitize(Settings& s) {
  if (s.native_arm) s.on = false;
  s.band = std::clamp(s.band, 0.f, 0.5f);
  s.size_m = std::clamp(s.size_m, 0.03f, 0.4f);
  clamp_crop(s.ammo);
  clamp_crop(s.health);
}

bool same(const Settings& a, const Settings& b) { return std::memcmp(&a, &b, sizeof(Settings)) == 0; }

float number_after(const char* text, const char* key, float fallback) {
  const char* p = std::strstr(text, key);
  if (p == nullptr) return fallback;
  float v = fallback;
  return std::sscanf(p + std::strlen(key), "%f", &v) == 1 && std::isfinite(v) ? v : fallback;
}
}  // namespace

Settings* settings() { return &g_s; }

void load(const wchar_t* dir) {
  g_path = std::wstring(dir) + L"\\AvatarVR_hud.txt";
  std::memset(&g_s, 0, sizeof(g_s));  // padding too: changes are detected with memcmp
  const Settings defaults;
  g_s.on = defaults.on; g_s.full = defaults.full; g_s.band = defaults.band; g_s.size_m = defaults.size_m;
  g_s.native_arm = defaults.native_arm; g_s.damage_fx_full = defaults.damage_fx_full; g_s.arm_anchor = defaults.arm_anchor; g_s.face_viewer = defaults.face_viewer; g_s.hide_original = defaults.hide_original;
  g_s.ammo = defaults.ammo; g_s.health = defaults.health;
  char text[512] = {};
  if (settings_file::read(g_path, text, sizeof(text))) {
    g_s.on = number_after(text, "wrist_hud", 0.f) != 0.f;
    g_s.full = number_after(text, "show_full", 0.f) != 0.f;
    g_s.native_arm = number_after(text, "native_arm", 0.f) != 0.f;
    g_s.damage_fx_full = number_after(text, "damage_fx_full", 0.f) != 0.f;
    g_s.arm_anchor = number_after(text, "arm_anchor", 1.f) != 0.f;
    g_s.face_viewer = number_after(text, "face_viewer", 1.f) != 0.f;
    g_s.hide_original = number_after(text, "hide_original", 1.f) != 0.f;
    g_s.band = number_after(text, "band", g_s.band);
    g_s.size_m = number_after(text, "size_m", g_s.size_m);
    g_s.ammo = {number_after(text, "ammo_x", g_s.ammo.x), number_after(text, "ammo_y", g_s.ammo.y),
                number_after(text, "ammo_w", g_s.ammo.w), number_after(text, "ammo_h", g_s.ammo.h)};
    g_s.health = {number_after(text, "health_x", g_s.health.x), number_after(text, "health_y", g_s.health.y),
                  number_after(text, "health_w", g_s.health.w), number_after(text, "health_h", g_s.health.h)};
  }
  sanitize(g_s);
  g_saved = g_s;
  std::lock_guard<std::mutex> lock(g_mutex);
  g_published = g_s;
}

void poll() {
  sanitize(g_s);
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_published = g_s;
  }
  if (same(g_s, g_saved) || g_path.empty()) return;
  char text[512];
  int n = std::snprintf(text, sizeof(text), "wrist_hud %d\nshow_full %d\nband %.3f\nsize_m %.3f\n", g_s.on ? 1 : 0, g_s.full ? 1 : 0, g_s.band,
                        g_s.size_m);
  n += std::snprintf(text + n, sizeof(text) - n, "ammo_x %.3f\nammo_y %.3f\nammo_w %.3f\nammo_h %.3f\n", g_s.ammo.x, g_s.ammo.y, g_s.ammo.w, g_s.ammo.h);
  n += std::snprintf(text + n, sizeof(text) - n, "health_x %.3f\nhealth_y %.3f\nhealth_w %.3f\nhealth_h %.3f\n", g_s.health.x, g_s.health.y,
                     g_s.health.w, g_s.health.h);
  n += std::snprintf(text + n, sizeof(text) - n, "face_viewer %d\nhide_original %d\narm_anchor %d\nnative_arm %d\ndamage_fx_full %d\n", g_s.face_viewer ? 1 : 0, g_s.hide_original ? 1 : 0,
                     g_s.arm_anchor ? 1 : 0, g_s.native_arm ? 1 : 0, g_s.damage_fx_full ? 1 : 0);
  if (settings_file::write(g_path, std::string_view(text, n))) {
    g_saved = g_s;
  }
}

Settings current() {
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_published;
}

void image_ready(bool ready) { g_image_ready.store(ready, std::memory_order_relaxed); }
void submitted(bool visible) {
  g_submitted_at.store(visible ? GetTickCount64() : 0, std::memory_order_relaxed);
}
bool replacing() {
  const auto tick = g_submitted_at.load(std::memory_order_relaxed);
  return g_image_ready.load(std::memory_order_relaxed) && tick && GetTickCount64() - tick < 250;
}

}  // namespace wrist_hud
