#pragma once
// [AUTHOR 2026-11-17] Models for the in-VR Tuner tab pages that mirror tools/ik_tuner: mounted arms (tp.* keys in
// AvatarVR_ik_tune.txt), HUD on arm (AvatarVR_hud_widgets.txt), full-view effects (AvatarVR_fullview_fx.txt) and the
// finger capture reply (AvatarVR_gesture_capture.json). Keys, ranges, defaults and file layouts follow the Python
// tuner (ik_tuner.py, hud_tuning.py, fullview_tuning.py); the mod's parsers are rider_ik.hpp and ui_ownership.cpp.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace tuner_model {

inline std::vector<std::string> lines_of(const std::string& text) {
  std::vector<std::string> out;
  std::size_t at = 0;
  while (at < text.size()) {
    std::size_t end = text.find('\n', at);
    if (end == std::string::npos) end = text.size();
    std::string line = text.substr(at, end - at);
    at = end + 1;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
    const auto first = line.find_first_not_of(" \t");
    out.push_back(first == std::string::npos ? std::string() : line.substr(first));
  }
  return out;
}
inline bool number(const std::string& text, double& out) {
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (end == text.c_str()) return false;
  while (*end == ' ' || *end == '\t') ++end;
  if (*end || !std::isfinite(v)) return false;
  out = v;
  return true;
}

// ---- Mounted arms -----------------------------------------------------------------------------------------
struct ArmSpec {
  std::string key, label;
  float lo, hi, step, def;
  const char* inherit;  // [arms] key in AvatarVR_settings.ini an absent value inherits, or nullptr
  const char* unit;
  const char* help;
};
// ik_tuner.py TP_GROUPS, in order.
inline const std::vector<ArmSpec>& shared_arm_specs() {
  static const std::vector<ArmSpec> specs = {
      {"tp.mount_correction", "Symmetry + hand helpers", 0, 1, 1, 1, nullptr, "",
       "Mirror right shoulder placement onto the left and correct thumb helpers. 0 restores the previous mounted solve."},
      {"tp.hand_pitch", "Hand pitch", -180, 180, 1, 0, "hand_pitch", "deg", "Tilts the hand up/down around the knuckles."},
      {"tp.hand_yaw", "Hand yaw", -180, 180, 1, 0, "hand_yaw", "deg", "Turns the hand left/right."},
      {"tp.hand_roll", "Hand roll", -180, 180, 1, 0, "hand_roll", "deg", "Rolls the hand (palm in / palm down)."},
      {"tp.hand_forward", "Hand forward", -0.3f, 0.3f, 0.005f, 0, nullptr, "m", "Moves both mounted hand targets forward in the body frame."},
      {"tp.hand_up", "Hand up", -0.3f, 0.3f, 0.005f, 0, nullptr, "m", "Raises both mounted hand targets equally."},
      {"tp.hand_side", "Hand side (mirrored)", -0.3f, 0.3f, 0.005f, 0, nullptr, "m",
       "Right-authored lateral offset; the left hand receives the opposite sign."},
      {"tp.hand_palm", "Palm depth", 0, 1.5f, 0.01f, 0.75f, nullptr, "", "Wrist distance behind the grip, in knuckle lengths."},
      {"tp.shoulder_out", "Shoulder out", -0.3f, 0.3f, 0.005f, 0, nullptr, "m", "+ moves both shoulders away from the body centre."},
      {"tp.shoulder_up", "Shoulder up", -0.3f, 0.3f, 0.005f, 0, nullptr, "m", "+ raises both shoulders."},
      {"tp.shoulder_forward", "Shoulder forward", -0.3f, 0.3f, 0.005f, 0, nullptr, "m", "+ moves both shoulders forward."},
      {"tp.reach_scale", "Reach scale", 0.5f, 1.5f, 0.01f, 1, nullptr, "x", "Below 1 keeps the elbows bent (less stretched arms)."},
      {"tp.max_reach", "Clavicle reach", 0, 0.3f, 0.005f, 0.12f, nullptr, "m", "How far the shoulder may roll toward a far target."},
      {"tp.elbow_out", "Elbow outward", -2, 2, 0.05f, 1, nullptr, "", "Weight pushing the elbows outward."},
      {"tp.elbow_down", "Elbow down", -2, 2, 0.05f, 1, nullptr, "", "Weight pushing the elbows down."},
      {"tp.elbow_back", "Elbow back", -2, 2, 0.05f, 0.2f, nullptr, "", "Weight pushing the elbows back."},
      {"tp.twist_fore", "Forearm twist share", 0, 1.5f, 0.05f, 1, nullptr, "", "Share of the hand roll the forearm takes."},
      {"tp.twist_upper", "Upper-arm twist share", 0, 1, 0.05f, 0.3f, nullptr, "", "Share of the hand roll the upper arm takes."},
  };
  return specs;
}
// ik_tuner.py arm_groups(side): additive trims, zero keeps the shared setup.
inline const std::vector<ArmSpec>& side_arm_specs(int side) {
  static std::vector<ArmSpec> specs[2];
  auto& out = specs[side ? 1 : 0];
  if (!out.empty()) return out;
  const std::string name = side ? "right" : "left";
  for (const auto& s : shared_arm_specs()) {
    if (s.key == "tp.mount_correction") continue;
    ArmSpec t = s;
    const std::string shortkey = s.key.substr(3);
    t.key = "tp." + name + "." + shortkey;
    t.label = s.label + " trim";
    t.def = 0;
    t.inherit = nullptr;
    if (shortkey == "hand_palm") { t.lo = -1.5f; t.hi = 1.5f; }
    if (shortkey == "reach_scale") { t.lo = -1; t.hi = 1; }
    if (shortkey == "max_reach" || shortkey == "twist_fore" || shortkey == "twist_upper") t.lo = -t.hi;
    t.help = side ? "Right arm only: added to the shared value. Zero keeps the current setup."
                  : "Left arm only: added to the shared value. Zero keeps the current setup.";
    out.push_back(t);
    if (shortkey == "twist_upper") {
      out.push_back({"tp." + name + ".forearm_roll", "Forearm skin roll", -180, 180, 1, 0, nullptr, "deg",
                     "Extra skin roll on this arm only. Start with small changes; zero adds no roll."});
      out.push_back({"tp." + name + ".upper_roll", "Upper-arm skin roll", -180, 180, 1, 0, nullptr, "deg",
                     "Extra skin roll on this arm only. Start with small changes; zero adds no roll."});
    }
  }
  return out;
}
// Numeric "key value" lines of the [arms] section (the values an absent hand angle inherits).
inline std::map<std::string, double> settings_section(const std::string& text, const char* section) {
  std::map<std::string, double> out;
  bool inside = false;
  const std::string head = std::string("[") + section + "]";
  for (const auto& line : lines_of(text)) {
    if (!line.empty() && line[0] == '[') { inside = line == head; continue; }
    if (!inside) continue;
    const auto gap = line.find_first_of(" \t");
    if (gap == std::string::npos) continue;
    double v = 0;
    std::string value = line.substr(gap + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    if (number(value, v)) out[line.substr(0, gap)] = v;
  }
  return out;
}

// ---- HUD on arm -------------------------------------------------------------------------------------------
struct HudElement { const char* key; const char* label; int attach; float along, side, lift; };
constexpr HudElement kHudElements[] = {
    {"health", "Health bar", 1, .12f, 0, .015f}, {"energy", "Energy arcs", 1, .12f, 0, .015f},
    {"heal", "Heal prompt", 1, .12f, 0, .015f}, {"eat", "Eat food prompt", 1, .12f, 0, .015f},
    {"ammo", "Ammo and weapon slots", 1, .26f, 0, .015f}, {"status", "Status icons (top right)", 3, 0, 0, .02f},
    {"tracker", "Objective tracker", 0, 0, 0, .015f}, {"compass", "Compass", 0, 0, 0, .015f},
    {"save", "Autosave dots", 0, 0, 0, .015f}, {"notify", "Centre notifications", 0, 0, 0, .015f},
    {"centre", "Centre widget (unidentified)", 0, 0, 0, .015f}, {"top", "Top centre widget (unidentified)", 0, 0, 0, .015f},
    {"right", "Right of centre prompts (unidentified)", 0, 0, 0, .015f}, {"low", "Lower right of centre (unidentified)", 0, 0, 0, .015f},
    {"reticle", "Reticle", 9, 0, 0, 0}};
constexpr int kHudCount = sizeof(kHudElements) / sizeof(kHudElements[0]);
constexpr int kHudFieldCount = 11;
enum HudField { HAttach, HAlong, HSide, HLift, HYaw, HPitch, HRoll, HScale, HBone, HTwoSided, HShrink };
inline const char* hud_field_name(int f) {
  static const char* n[kHudFieldCount] = {"attach", "along", "side", "lift", "yaw", "pitch", "roll", "scale", "bone", "two_sided", "shrink"};
  return n[f];
}
constexpr int kHudAttachCount = 10, kHudBoneAttach = 7, kHudHeadAttach = 8;
inline const char* const* hud_attach_labels() {
  static const char* l[kHudAttachCount] = {"Flat HUD (not moved)", "Left arm", "Right arm", "Left hand", "Right hand", "Weapon",
                                           "Hidden", "Specific bone", "Head (follows headset)", "Weapon aim (reticle)"};
  return l;
}
inline std::array<float, kHudFieldCount> hud_defaults(int e) {
  const auto& d = kHudElements[e];
  return {static_cast<float>(d.attach), d.along, d.side, d.lift, 0, 0, 0, 1, -1, 0,
          std::strcmp(d.key, "reticle") ? 0.f : .5f};
}
inline bool hud_additive(int f) { return f >= HAlong && f <= HRoll; }
struct Hud {
  std::array<std::array<float, kHudFieldCount>, kHudCount> v{};
  int frame = 1;
  Hud() { for (int e = 0; e < kHudCount; ++e) v[e] = hud_defaults(e); }
  void parse(const std::string& text) {
    *this = Hud();
    for (const auto& line : lines_of(text)) {
      const auto eq = line.find('='), dot = line.find('.');
      double value = 0;
      if (eq == std::string::npos || dot == std::string::npos || dot > eq || !number(line.substr(eq + 1), value)) continue;
      const std::string element = line.substr(0, dot), field = line.substr(dot + 1, eq - dot - 1);
      if (element == "settings" && field == "frame") { frame = static_cast<int>(value); continue; }
      for (int e = 0; e < kHudCount; ++e)
        if (element == kHudElements[e].key)
          for (int f = 0; f < kHudFieldCount; ++f)
            if (field == hud_field_name(f)) v[e][f] = static_cast<float>(value);
    }
  }
  std::string text() const {  // hud_tuning.py render()
    std::string out;
    char line[96];
    for (int e = 0; e < kHudCount; ++e)
      for (int f = 0; f < kHudFieldCount; ++f) {
        std::snprintf(line, sizeof(line), "%s.%s=%g\n", kHudElements[e].key, hud_field_name(f), v[e][f]);
        out += line;
      }
    std::snprintf(line, sizeof(line), "settings.frame=%d\n", frame);
    return out + line;
  }
  // hud_tuning.py set_value: selected elements move by the same amount, size scales, other fields are set.
  void set(const std::vector<int>& selected, int current, int f, float value) {
    const float old = v[current][f];
    for (int e : selected) {
      if (e == current) continue;
      if (hud_additive(f)) v[e][f] += value - old;
      else if (f == HScale && old != 0.f) v[e][f] = std::max(.05f, v[e][f] * value / old);
      else v[e][f] = value;
    }
    v[current][f] = value;
  }
  void attach(const std::vector<int>& selected, int current, int value) {
    const bool to_head = value == kHudHeadAttach && static_cast<int>(v[current][HAttach]) != kHudHeadAttach;
    set(selected, current, HAttach, static_cast<float>(value));
    if (to_head) for (int e : selected) { v[e][HAlong] = .3f; v[e][HSide] = 0; v[e][HLift] = 0; }
  }
  void reset(const std::vector<int>& selected) { for (int e : selected) v[e] = hud_defaults(e); }
  void line_up(const std::vector<int>& selected, int current) { for (int e : selected) v[e] = v[current]; }
};
inline int hud_index(const std::string& key) {
  for (int e = 0; e < kHudCount; ++e) if (key == kHudElements[e].key) return e;
  return -1;
}
// AvatarVR_hud_groups.json: {"name": ["health", ...], ...}. The default group is always there.
inline std::vector<std::pair<std::string, std::vector<int>>> hud_groups(const std::string& json) {
  std::vector<std::pair<std::string, std::vector<int>>> out = {{"Bottom HUD (health, energy, prompts)", {0, 1, 2, 3}}};
  std::size_t at = 0;
  auto string_at = [&](std::size_t& p, std::string& s) {
    p = json.find('"', p);
    if (p == std::string::npos) return false;
    const std::size_t end = json.find('"', p + 1);
    if (end == std::string::npos) return false;
    s = json.substr(p + 1, end - p - 1);
    p = end + 1;
    return true;
  };
  while (true) {
    std::string name;
    if (!string_at(at, name)) break;
    const std::size_t open = json.find('[', at), close = json.find(']', at);
    if (open == std::string::npos || close == std::string::npos || close < open) break;
    std::vector<int> members;
    std::size_t p = open;
    std::string key;
    while (p < close && string_at(p, key) && p <= close) {
      const int e = hud_index(key);
      if (e >= 0) members.push_back(e);
    }
    at = close + 1;
    if (members.empty()) continue;
    bool replaced = false;
    for (auto& g : out) if (g.first == name) { g.second = members; replaced = true; }
    if (!replaced) out.push_back({name, members});
  }
  return out;
}
// AvatarVR_hud_bones.txt "index=name" lines written by the mod.
inline std::vector<std::pair<int, std::string>> bone_list(const std::string& text) {
  std::vector<std::pair<int, std::string>> out;
  for (const auto& line : lines_of(text)) {
    const auto eq = line.find('=');
    double index = 0;
    if (eq != std::string::npos && eq + 1 < line.size() && number(line.substr(0, eq), index)) out.push_back({static_cast<int>(index), line.substr(eq + 1)});
  }
  return out;
}

// ---- Full-view effects ------------------------------------------------------------------------------------
constexpr int kFxMax = 6;  // FxMax in ui_ownership.cpp
struct Effect { float x = 0, y = 0, on = 1, strength = 3, size = 1.02f; };  // FxDefault
inline const char* effect_name(int x, int y) {
  if (x == 0 && y == 0) return "red damage effect";
  if (x == 280 && y == 0) return "aim dot, probably: leave off";
  return "new";
}
// fullview_tuning.py read_effects: the saved list, or the default when there is none.
inline std::vector<Effect> parse_effects(const std::string& text) {
  std::map<int, Effect> found;
  for (const auto& line : lines_of(text)) {
    unsigned i = 0;
    char field[16]{};
    float value = 0;
    if (std::sscanf(line.c_str(), "effect%u.%15[^=]=%f", &i, field, &value) != 3 || i >= kFxMax || !std::isfinite(value)) continue;
    auto& e = found[static_cast<int>(i)];
    if (!std::strcmp(field, "x")) e.x = value; else if (!std::strcmp(field, "y")) e.y = value;
    else if (!std::strcmp(field, "on")) e.on = value; else if (!std::strcmp(field, "strength")) e.strength = value;
    else if (!std::strcmp(field, "size")) e.size = value;
  }
  std::vector<Effect> out;
  for (const auto& f : found) out.push_back(f.second);
  if (out.empty()) out.push_back(Effect{});
  return out;
}
inline std::string effects_text(const std::vector<Effect>& effects) {
  std::string out;
  char line[80];
  for (std::size_t i = 0; i < effects.size() && i < static_cast<std::size_t>(kFxMax); ++i) {
    const Effect& e = effects[i];
    const float fields[5] = {e.x, e.y, e.on, e.strength, e.size};
    const char* names[5] = {"x", "y", "on", "strength", "size"};
    for (int f = 0; f < 5; ++f) {
      std::snprintf(line, sizeof(line), "effect%zu.%s=%.10g\n", i, names[f], fields[f]);
      out += line;
    }
  }
  return out;
}
inline std::vector<std::pair<int, int>> seen_widgets(const std::string& text) {
  std::vector<std::pair<int, int>> out;
  for (const auto& line : lines_of(text)) {
    int x = 0, y = 0;
    char tail = 0;
    if (std::sscanf(line.c_str(), "%d,%d%c", &x, &y, &tail) == 2) out.push_back({x, y});
  }
  return out;
}

// ---- Flat JSON fields (AvatarVR_gesture_status.json) ---------------------------------------------------------
inline bool json_number(const std::string& json, const char* key, double& out) {
  const std::string k = std::string("\"") + key + "\":";
  const auto p = json.find(k);
  if (p == std::string::npos) return false;
  char* end = nullptr;
  const double v = std::strtod(json.c_str() + p + k.size(), &end);
  if (end == json.c_str() + p + k.size() || !std::isfinite(v)) return false;
  out = v;
  return true;
}
inline bool json_pair(const std::string& json, const char* key, double out[2]) {
  const std::string k = std::string("\"") + key + "\":[";
  const auto p = json.find(k);
  if (p == std::string::npos) return false;
  const char* at = json.c_str() + p + k.size();
  char* end = nullptr;
  const double a = std::strtod(at, &end);
  if (end == at || *end != ',') return false;
  at = end + 1;
  const double b = std::strtod(at, &end);
  if (end == at || !std::isfinite(a) || !std::isfinite(b)) return false;
  out[0] = a;
  out[1] = b;
  return true;
}

// ---- Finger capture reply ---------------------------------------------------------------------------------
struct Capture {
  long long id = 0;
  bool mounted = false;
  double position_mm = 0, angle_deg = 0;
  int clamped = 0;
  std::map<std::string, double> values;  // "finger.axis" / "helper.axis"
};
inline bool parse_capture(const std::string& json, Capture& out) {
  out = Capture{};
  auto field = [&](const char* name, double& value) {
    const std::string key = std::string("\"") + name + "\":";
    const auto p = json.find(key);
    if (p == std::string::npos) return false;
    char* end = nullptr;
    value = std::strtod(json.c_str() + p + key.size(), &end);
    return end != json.c_str() + p + key.size() && std::isfinite(value);
  };
  double id = 0, clamped = 0;
  if (!field("id", id)) return false;
  out.id = static_cast<long long>(id);
  field("position_mm", out.position_mm);
  field("angle_deg", out.angle_deg);
  if (field("clamped", clamped)) out.clamped = static_cast<int>(clamped);
  out.mounted = json.find("\"context\":\"tp\"") != std::string::npos;
  const auto start = json.find("\"values\":{");
  if (start == std::string::npos) return false;
  std::size_t p = start + 10;
  const std::size_t stop = json.find('}', p);
  while (p < stop) {
    const auto q0 = json.find('"', p);
    if (q0 == std::string::npos || q0 >= stop) break;
    const auto q1 = json.find('"', q0 + 1);
    if (q1 == std::string::npos || q1 >= stop || json[q1 + 1] != ':') break;
    char* end = nullptr;
    const double v = std::strtod(json.c_str() + q1 + 2, &end);
    if (end == json.c_str() + q1 + 2) break;
    if (std::isfinite(v)) out.values[json.substr(q0 + 1, q1 - q0 - 1)] = v;
    p = static_cast<std::size_t>(end - json.c_str());
  }
  return !out.values.empty();
}
}  // namespace tuner_model
