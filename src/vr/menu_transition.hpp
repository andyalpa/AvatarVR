#pragma once
#include <cmath>

// Flat-screen placement and black fades for the switches between the flat "cinema" screen (start-up videos, loading
// screens, pause / inventory menus) and head-tracked stereo. Pure logic, used by the XR worker (xr_runtime.cpp) and
// tested by tools/tests/menu_transition_test.cpp.
//
// Before (2026-11-08 HEADSET/AUTHOR): every time the flat screen appeared it followed the head for 90 XR frames (~1.3 s)
// and then stopped; after a load the headset showed flat gameplay, then stereo, flat again and stereo (LIVE
// XR_RUNTIME/NATIVE_STEREO 18:26:24-26). Now the screen is placed once, at a steady head pose (or kept where it is when
// still in view), behind black, and every flat <-> stereo switch goes through black.
// "screenfollow" in [xr] restores the old follow; "nofade" turns the black layer off.
namespace menu_transition {

inline float wrap_deg(float a) {
  while (a > 180.f) a -= 360.f;
  while (a < -180.f) a += 360.f;
  return a;
}

// ---- placement ----------------------------------------------------------------------------------------------------

struct PlaceConfig {
  int start_frames = 45;    // at session start the first poses can be stale: never place before this many frames
  int min_frames = 2;       // otherwise
  int stable_frames = 8;    // consecutive frames with the head nearly still in yaw
  int max_frames = 90;      // place anyway after this many frames (the old follow length)
  float stable_deg = 0.6f;  // yaw change per frame that still counts as still (~43 deg/s at 72 Hz)
  float keep_deg = 35.f;    // lazy request: the screen stays where it is while its centre is this close to the gaze
  float keep_rise_m = 0.4f; // ... and its height is this close to the head's
};

enum class Step { Idle, Wait, Place, Keep };

class Placer {
 public:
  PlaceConfig cfg;

  // lazy: a flat screen that is still in view stays put (stereo -> flat handovers). startup: session start.
  void request(bool lazy, bool startup) {
    waiting_ = true;
    lazy_ = lazy;
    min_ = startup ? cfg.start_frames : cfg.min_frames;
    frames_ = 0;
    stable_ = 0;
    have_prev_ = false;
  }

  // One XR frame. head_yaw_deg: gaze yaw; tracked: orientation valid and tracked. bearing_deg / dist_m / rise_m: the
  // current screen centre seen from the head (yaw, horizontal distance, height above the head); want_dist_m: the distance
  // a new placement would use. Place = put the screen in front of the head now; Keep = leave it where it is.
  Step step(float head_yaw_deg, bool tracked, bool has_screen, float bearing_deg, float dist_m, float rise_m,
            float want_dist_m) {
    if (!waiting_) return Step::Idle;
    if (lazy_ && frames_ == 0 && has_screen && placed_once_ && tracked &&
        std::fabs(wrap_deg(bearing_deg - head_yaw_deg)) < cfg.keep_deg && std::fabs(rise_m) < cfg.keep_rise_m &&
        dist_m > 0.5f * want_dist_m && dist_m < 1.5f * want_dist_m) {
      waiting_ = false;
      return Step::Keep;
    }
    ++frames_;
    const float d = have_prev_ ? std::fabs(wrap_deg(head_yaw_deg - prev_yaw_)) : 1e9f;
    stable_ = tracked && d < cfg.stable_deg ? stable_ + 1 : 0;
    prev_yaw_ = head_yaw_deg;
    have_prev_ = true;
    if ((frames_ >= min_ && stable_ >= cfg.stable_frames) || frames_ >= cfg.max_frames) {
      waiting_ = false;
      placed_once_ = true;
      return Step::Place;
    }
    return Step::Wait;
  }

  bool waiting() const { return waiting_; }
  int frames() const { return frames_; }

 private:
  bool waiting_ = false, lazy_ = false, placed_once_ = false, have_prev_ = false;
  int min_ = 0, frames_ = 0, stable_ = 0;
  float prev_yaw_ = 0.f;
};

// ---- black fade ---------------------------------------------------------------------------------------------------

enum class Kind { None, Flat, Stereo };

struct FadeConfig {
  float out_s = 0.15f;           // to black (only where the old picture is still there to fade)
  float in_s = 0.30f;            // from black
  float flat_hold_s = 0.10f;     // black after a cut to the flat screen
  float flap_stint_s = 2.0f;     // a stereo stint shorter than this probably comes back (post-load dip) ...
  float flap_hold_s = 2.0f;      // ... so the flat screen after it stays black this long (not for pause menus)
  float stereo_settle_s = 0.5f;  // stereo is shown only after running this long
  float menu_settle_s = 0.2f;    // ... or this long when it comes back from a pause menu (same second view, no rebuild)
  float pending_max_s = 5.0f;    // the flat screen stays black at most this long while stereo is announced
};

struct FadeInput {
  Kind kind = Kind::None;
  bool screen_placed = true;    // flat: the screen has its final pose
  bool stereo_pending = false;  // native_stereo: gameplay seen, stereo about to start (the flat picture is gameplay)
  bool menu_pause = false;      // native_stereo: stereo stopped (or is coming back) because of a pause menu
};

class Fade {
 public:
  FadeConfig cfg;

  // t: monotonic seconds. Returns the black layer's opacity, 0 (clear) .. 1 (black).
  float update(const FadeInput& in, double t) {
    const double dt = have_t_ ? std::fmin(std::fmax(t - last_t_, 0.0), 0.1) : 0.0;
    last_t_ = t;
    have_t_ = true;
    changed_ = false;
    if (in.kind != kind_) {
      const Kind prev = kind_;
      const double stint = prev == Kind::Stereo ? t - since_ : 1e9;
      kind_ = in.kind;
      since_ = t;
      changed_ = true;
      pending_since_ = -1.0;
      // Both eye swapchains already hold the new picture, so the old one cannot fade out: cut to black. A first picture
      // also starts from black.
      if (in.kind != Kind::None) alpha_ = 1.f;
      // HEADSET 2026-11-08: pausing within 2 s of unpausing gave 2 s of black, so the flap hold is for loads only.
      const bool flap = prev == Kind::Stereo && stint < cfg.flap_stint_s && !in.menu_pause;
      hold_until_ = in.kind == Kind::Flat && prev != Kind::None ? t + (flap ? cfg.flap_hold_s : cfg.flat_hold_s) : t;
      settle_s_ = in.menu_pause ? cfg.menu_settle_s : cfg.stereo_settle_s;
    }
    bool black = false;
    if (kind_ == Kind::Flat) {
      black = !in.screen_placed || t < hold_until_;
      if (in.stereo_pending) {
        if (pending_since_ < 0.0) pending_since_ = t;
        black = black || t - pending_since_ < cfg.pending_max_s;
      } else {
        pending_since_ = -1.0;
      }
    } else if (kind_ == Kind::Stereo) {
      black = t - since_ < settle_s_;
    }
    black_ = black;
    const float target = black ? 1.f : 0.f;
    if (alpha_ < target) {
      alpha_ = std::fmin(target, alpha_ + static_cast<float>(dt / cfg.out_s));
    } else if (alpha_ > target) {
      alpha_ = std::fmax(target, alpha_ - static_cast<float>(dt / cfg.in_s));
    }
    return alpha_;
  }

  float alpha() const { return alpha_; }
  bool black_wanted() const { return black_; }
  bool kind_changed() const { return changed_; }
  Kind kind() const { return kind_; }

 private:
  Kind kind_ = Kind::None;
  float alpha_ = 1.f;
  bool black_ = true, have_t_ = false, changed_ = false;
  double since_ = 0.0, last_t_ = 0.0, hold_until_ = 0.0, pending_since_ = -1.0;
  float settle_s_ = 0.5f;
};

// Fade sprite: kLevels opacity levels side by side, each kLevelTexels wide; a quad shows one level through a sub-rect
// that stays inside its block, so filtering never mixes levels.
constexpr int kLevels = 32;
constexpr int kLevelTexels = 4;
inline int level_for(float alpha) {
  const float a = alpha < 0.f ? 0.f : alpha > 1.f ? 1.f : alpha;
  return static_cast<int>(a * static_cast<float>(kLevels - 1) + 0.5f);
}

}  // namespace menu_transition
