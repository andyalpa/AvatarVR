#pragma once
// FEATURE-07 bow string pull (2026-11-18): pure state machine, no game or OpenXR state (tested by bow_draw_test).
// The off hand (the hand without the bow) presses its grip near the bow string, pulls back along the arrow, and opens
// the grip to shoot. The bow hand's trigger does nothing while this owns the fire input.
//   Idle    -> Drawing : nock: the off hand's grip took the native two-hand hold at the arrow (decided by the caller,
//                         skin_palette_probe::grab_two_hand == 1); a grip in the air never nocks (HEADSET 2026-11-18)
//   Drawing, started    : the hand has pulled back start_pull: only now is the trigger pressed (the game's draw begins
//                         with the hand's own motion, not at the grab; HEADSET run 152157: a trigger at the grab made
//                         the game draw by itself)
//   Drawing -> Idle    : grip opened -> shot if started (a short pull = a weak shot), else dropped without a shot
//   Drawing -> Idle    : bow no longer active (lost, holstered, menu, hand swap) -> cancel
// Pull = the hand's travel since the grab, along the draw direction (backwards along the arrow). It is measured from
// the grab point, so the controller-to-pinch offset cancels.
// Fire output (pad RT, 0..1), Mode:
//   kAnalog : RT = max(floor, pull fraction): pull distance sets the trigger, so the draw if the game reads RT as analog
//   kHold   : RT = 1 while drawing (the game charges over time, as with a held trigger)
// press_fire (bows the game fires on the trigger press): RT 0 while drawing, a tap_ms pulse on release.
// Which mode the game honours, and which bows fire on press, are OPEN (Stage 0 probe, research/WEAPONS_AND_HANDS.md).
#include <algorithm>
#include <cmath>

namespace bow_draw_math {
enum class Mode { kAnalog = 0, kHold = 1 };
enum class Phase { kIdle, kDrawing };
enum class Event { kNone, kNock, kRelease, kCancel, kDrop, kCancelShot };  // kDrop: let go before pulling;
// kCancelShot: pushed back to the bow after drawing (the game's X Cancel Shot is tapped, the grip still holds the string)

struct Params {
  float draw_length = 0.60f;  // m of pull for a full draw
  float rt_floor = 0.20f;     // analog: the least trigger sent once the draw has started
  float start_pull = 0.025f;  // m of pull before the trigger is pressed (HEADSET run 153032: 6 cm felt like a threshold)
  Mode mode = Mode::kAnalog;
  bool press_fire = false;
  unsigned tap_ms = 120;
  float cancel_peak = 0.06f;   // m drawn before pushing forward can cancel
  float cancel_pull = 0.015f;  // m: back within this of the grab point = cancel (HEADSET 2026-11-18 run 203601)
};

struct Input {
  bool active = false;     // a bow is held in the weapon hand and the feature may own the trigger
  bool grip = false;       // the off hand's grip, this poll
  bool nock = false;       // the off hand's grip press took the native two-hand hold this poll
  bool have_hand = false;  // hand/string/direction below are fresh
  float hand[3]{};         // off-hand wrist, world
  float string_pt[3]{};    // C_bowString, world
  float draw_dir[3]{};     // unit, backwards along the arrow (-Z of C_arrow_01), world
  unsigned long long now_ms = 0;
};

struct State {
  Phase phase = Phase::kIdle;
  bool prev_grip = false;
  float grab[3]{};
  float pull = 0.f;      // m
  float fraction = 0.f;  // 0..1
  bool started = false;  // pulled past start_pull since the nock
  unsigned long long tap_until = 0;
  float peak = 0.f;                    // most pull since the draw started (m)
  unsigned long long cancel_until = 0;  // X held (with RT) until then
};

struct Output {
  bool owns_trigger = false;  // true: send rt, ignore the bow hand's trigger
  float rt = 0.f;
  Event event = Event::kNone;
  bool cancel_button = false;  // press the game's Cancel Shot (pad X) this poll
  float fraction = 0.f;
  float grab_distance = -1.f;  // at a nock: off hand to the string (m, log), -1 otherwise
};

inline float distance3(const float* a, const float* b) {
  float s = 0.f; for (int k = 0; k < 3; ++k) {const float d = a[k] - b[k]; s += d * d;} return std::sqrt(s);
}

inline float trigger_for(const State& st, const Params& p) {
  if (st.phase != Phase::kDrawing || p.press_fire || !st.started) return 0.f;
  if (p.mode == Mode::kHold) return 1.f;
  return std::clamp(std::max(p.rt_floor, st.fraction), 0.f, 1.f);
}

inline Output step(State& st, const Input& in, const Params& p) {
  Output out;
  const bool press = in.grip && !st.prev_grip;
  const bool release = !in.grip && st.prev_grip;
  st.prev_grip = in.grip;
  if (!in.active) {
    if (st.phase == Phase::kDrawing) out.event = Event::kCancel;
    st.phase = Phase::kIdle; st.pull = st.fraction = 0.f; st.tap_until = 0; st.started = false; st.peak = 0.f; st.cancel_until = 0;
    return out;  // owns_trigger false: the normal trigger mapping applies
  }
  out.owns_trigger = true;
  (void)press;
  if (st.phase == Phase::kIdle && in.nock && in.grip && in.have_hand) {
    out.grab_distance = distance3(in.hand, in.string_pt);
    st.phase = Phase::kDrawing; st.pull = st.fraction = 0.f; st.started = false; st.peak = 0.f;
    for (int k = 0; k < 3; ++k) st.grab[k] = in.hand[k];
    out.event = Event::kNock;
  } else if (st.phase == Phase::kDrawing) {
    if (in.have_hand) {
      float d = 0.f; for (int k = 0; k < 3; ++k) d += (in.hand[k] - st.grab[k]) * in.draw_dir[k];
      st.pull = std::max(0.f, d);
      st.fraction = p.draw_length > 1e-3f ? std::clamp(st.pull / p.draw_length, 0.f, 1.f) : 1.f;
      if (st.pull >= p.start_pull) st.started = true;
      if (st.started) st.peak = std::max(st.peak, st.pull);
      if (st.started && st.peak >= p.cancel_peak && st.pull <= p.cancel_pull && !release) {
        st.started = false; st.peak = 0.f; st.cancel_until = in.now_ms + p.tap_ms;
        out.event = Event::kCancelShot;
      }
    }  // else keep the last pull (a tracking gap does not let go)
    if (release) {
      st.phase = Phase::kIdle; out.event = st.started ? Event::kRelease : Event::kDrop;
      if (p.press_fire && st.started) st.tap_until = in.now_ms + p.tap_ms;
      st.started = false;
    }
  }
  out.fraction = st.fraction;
  out.rt = trigger_for(st, p);
  if (in.now_ms < st.tap_until) out.rt = 1.f;
  if (in.now_ms < st.cancel_until) {out.rt = 1.f; out.cancel_button = true;}  // X with the trigger still held, then let go
  if (out.event == Event::kRelease) {st.pull = 0.f;}  // fraction kept in out for the log/haptics
  return out;
}
}  // namespace bow_draw_math
