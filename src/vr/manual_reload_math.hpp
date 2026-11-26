#pragma once
// FEATURE-05 manual rifle reload, Stage 1 (2026-11-19): pure state machine, unit-tested (tools/tests/manual_reload_test.cpp).
// Driven by manual_reload.cpp; no engine access here.
//
// [LIVE 2026-11-19 VR_RELOAD.log, 8 rifle reloads, all 3300 ms] the game's own reload: the old magazine leaves 500-760 ms
// (down ~70 cm, back ~45 cm in the gun frame), the same C_magazine bone comes back as the new one and seats at ~1400 ms;
// +0x3F68 sets at ~1460 ms (refilled, INFERENCE); C_reloadLevel goes back 17 cm (-Z) at 1950-2530 ms (rack); +0x3F67 sets
// at ~2650 ms (ready, INFERENCE). The empty flag never set: auto reloads at 0 rounds look the same and also rack.
//
// The player's sequence (off hand = the hand without the gun):
//   grab the magazine (grip within grab_radius of its rest point, and along the barrel nearer the magazine than the
//   two-hand support point; HEADSET 2026-11-19: grips at the magazine read 0.19-0.39 m from the C_magazine origin and
//   always nearer the support point, so the first rule, 0.16 m and nearer than the support point, never grabbed)
//   -> pull it out (beyond pull_out: the game's reload is started with a pad X tap unless it is already running)
//   -> let go (dropped, hidden) -> grip at the off-side hip (a new magazine in the hand)
//   -> bring it to the well (within seat_radius: seated)
//   -> only after an empty magazine (the game reloaded by itself): grip the charging handle and pull it back rack_pull.
// Firing is blocked from the grab to the end, and while the game's automatic reload waits for the hands.
#include <cmath>

namespace manual_reload_math {

enum class Phase { kIdle, kHolding, kOut, kEmptyHand, kNewMag, kNeedRack, kRacking };
enum class Event { kNone, kGrab, kReseat, kEjected, kDropped, kFetched, kDroppedNew, kSeated, kRackGrab, kRacked, kRackLetGo,
                   kDone, kAutoReload, kCancel };
enum class MagDraw { kGame, kRest, kHand, kHidden };

struct Params {
  float grab_radius = 0.30f;   // wrist to the magazine's rest point (m); the wrist sits ~0.1 m from the palm
  float pull_out = 0.10f;      // the magazine counts as out this far from its rest point
  float seat_radius = 0.07f;   // the new magazine snaps in this close to the rest point
  float handle_keep = 0.15f;   // within this of the trigger handle (and nearer it than the magazine): a hand swap
  float rack_radius = 0.30f;   // wrist to the charging handle's rest point
  float rack_pull = 0.09f;     // pull back this far to rack
  float lever_max = 0.17f;     // the handle's travel (LIVE: 17 cm)
  unsigned button_window_ms = 700;  // a game reload starting this soon after a button or our X belongs to it
};

struct Input {
  bool active = false;         // enabled, a rifle in the weapon hand, on foot, no hand swap
  unsigned long long now_ms = 0;
  bool grip = false;           // the off hand's grip
  bool have_hand = false;      // hand / mag_rest / lever_rest / axes valid
  float hand[3]{};             // off-hand wrist, world
  float mag_rest[3]{};         // C_magazine rest point, world (gun as held now)
  float lever_rest[3]{};       // C_reloadLevel rest point, world
  float axes[9]{};             // gun root rows, unit: x, y, z (z forward along the barrel)
  float support_d = 1e9f;      // wrist to the native two-hand support point (m), 1e9 unknown
  float handle_d = 1e9f;       // wrist to the trigger handle (the hand-swap grab), 1e9 unknown
  bool have_support = false;   // support_pt valid
  float support_pt[3]{};       // where the support wrist sits on the front grip, world
  bool at_hip = false;         // the off hand is in its hip zone
  bool game_reloading = false; // the game's reload timer runs (comp+0x3EA8 state 0)
  bool button = false;         // the player's reload button is down (fallback: the game's own reload)
  bool have_held = false;      // the caller tracks the held magazine rigidly on the wrist (rotation included):
  float held[3]{};             // its point, world; otherwise it follows the wrist position in the gun's axes
};

struct Output {
  Event event = Event::kNone;
  bool press_x = false;        // tap the game's reload (pad X)
  bool block_fire = false;
  bool owns_grip = false;      // this press/hold is the reload's; the controls skip their own grip gestures
  MagDraw mag = MagDraw::kGame;
  float mag_world[3]{};        // with kHand
  bool lever_override = false; // draw C_reloadLevel at rest + lever_pull (instead of the game's animation)
  float lever_pull = 0.f;
};

struct State {
  Phase phase = Phase::kIdle;
  bool empty = false;          // the game reloaded by itself: hands must swap the magazine and rack
  bool grip_was = false;
  bool hold_until_release = false;   // the grip that seated / racked is still held: ignore it until released
  bool game_was = false;
  bool ours = false;           // the current game reload was started by us or the automatic reload (we draw it)
  unsigned long long button_ms = 0, x_ms = 0;
  float offset_local[3]{};     // magazine minus wrist at the grab, in gun axes
  float rack_start = 0.f;      // wrist along -Z from the handle's rest point at the rack grab
  unsigned long long rack_ms = 0;  // the rack grab
  float lever_pull = 0.f;
  bool racked = false;
  unsigned long long seat_ms = 0;  // the new magazine went in (pacing: the game's insert, then ready)
  bool rack_done = false;          // the last reload ended with a rack
};

inline float dist3(const float* a, const float* b) {
  float s = 0.f; for (int k = 0; k < 3; ++k) {const float d = a[k] - b[k]; s += d * d;} return std::sqrt(s);
}
inline void to_local(const float* axes, const float* v, float* out) {
  for (int r = 0; r < 3; ++r) out[r] = v[0] * axes[r * 3] + v[1] * axes[r * 3 + 1] + v[2] * axes[r * 3 + 2];
}
inline void to_world(const float* axes, const float* v, float* out) {
  for (int k = 0; k < 3; ++k) out[k] = v[0] * axes[k] + v[1] * axes[3 + k] + v[2] * axes[6 + k];
}
inline void held_mag(const State& s, const Input& in, float* out) {
  if (in.have_held) {for (int k = 0; k < 3; ++k) out[k] = in.held[k]; return;}
  float w[3]; to_world(in.axes, s.offset_local, w);
  for (int k = 0; k < 3; ++k) out[k] = in.hand[k] + w[k];
}

// The magazine grab: near enough, and along the barrel (gun z) nearer the magazine than the front grip. Without the
// support point: nearer the magazine than the support point.
inline bool magazine_grab(const Input& in, const Params& p) {
  const float d = dist3(in.hand, in.mag_rest);
  if (d >= p.grab_radius || (in.handle_d < p.handle_keep && in.handle_d < d)) return false;  // the hand swap keeps its grab
  if (!in.have_support) return d < in.support_d;
  const float* z = in.axes + 6;
  float to_mag = 0.f, to_sup = 0.f;
  for (int k = 0; k < 3; ++k) {to_mag += (in.hand[k] - in.mag_rest[k]) * z[k]; to_sup += (in.hand[k] - in.support_pt[k]) * z[k];}
  return std::fabs(to_mag) < std::fabs(to_sup);
}

inline Output step(State& s, const Input& in, const Params& p = Params{}) {
  Output o;
  const bool press = in.grip && !s.grip_was;
  const bool release = !in.grip && s.grip_was;
  s.grip_was = in.grip;
  if (in.button) s.button_ms = in.now_ms;

  if (!in.active) {
    if (s.phase != Phase::kIdle || s.empty) o.event = Event::kCancel;
    const bool grip = s.grip_was;
    s = State{}; s.grip_was = grip; s.game_was = in.game_reloading;
    return o;
  }

  // The game's reload starting: ours (X tap), the player's button (native fallback, not drawn by us) or automatic.
  if (in.game_reloading && !s.game_was) {
    const bool by_x = s.x_ms && in.now_ms - s.x_ms <= p.button_window_ms;
    const bool by_button = s.button_ms && in.now_ms - s.button_ms <= p.button_window_ms;
    if (by_x) s.ours = true;
    else if (by_button) s.ours = false;
    else if (s.phase == Phase::kIdle) {s.ours = true; s.empty = true; o.event = Event::kAutoReload;}
    else s.ours = true;  // started while the hands are mid-sequence (e.g. the magazine ran dry): keep drawing it
  }
  if (!in.game_reloading && s.game_was && s.phase == Phase::kIdle && !s.empty) s.ours = false;
  s.game_was = in.game_reloading;

  if (release) s.hold_until_release = false;
  const bool usable_press = press && in.have_hand && !s.hold_until_release;

  switch (s.phase) {
    case Phase::kIdle:
      if (usable_press) {
        if (magazine_grab(in, p)) {
          float v[3]; for (int k = 0; k < 3; ++k) v[k] = in.mag_rest[k] - in.hand[k];
          to_local(in.axes, v, s.offset_local);
          s.phase = Phase::kHolding; o.event = Event::kGrab;
          // The game's reload starts at the grab, so its own left hand grips the magazine (the fingers show the game's
          // hold) while pacing holds it there. Letting go before pulling out then finishes it (pacing).
          if (!in.game_reloading) {o.press_x = true; s.x_ms = in.now_ms;}
          else s.ours = true;
        }
      }
      break;
    case Phase::kHolding: {
      float m[3]; held_mag(s, in, m);
      if (release) {s.phase = Phase::kIdle; o.event = Event::kReseat; break;}
      if (in.have_hand && dist3(m, in.mag_rest) > p.pull_out) {
        s.phase = Phase::kOut; o.event = Event::kEjected;
        if (!in.game_reloading && !(s.x_ms && in.now_ms - s.x_ms <= p.button_window_ms)) {o.press_x = true; s.x_ms = in.now_ms;}
        else if (in.game_reloading) s.ours = true;
      }
      break;
    }
    case Phase::kOut:
      if (release) {s.phase = Phase::kEmptyHand; o.event = Event::kDropped;}
      break;
    case Phase::kEmptyHand:
      if (usable_press && in.at_hip) {s.phase = Phase::kNewMag; o.event = Event::kFetched;}
      break;
    case Phase::kNewMag: {
      if (release) {s.phase = Phase::kEmptyHand; o.event = Event::kDroppedNew; break;}
      float m[3]; held_mag(s, in, m);
      if (in.have_hand && dist3(m, in.mag_rest) < p.seat_radius) {
        o.event = Event::kSeated; s.hold_until_release = true; s.seat_ms = in.now_ms; s.rack_done = false;
        if (s.empty) s.phase = Phase::kNeedRack;
        else {s.phase = Phase::kIdle; o.event = Event::kDone;}
      }
      break;
    }
    case Phase::kNeedRack:
      if (usable_press && dist3(in.hand, in.lever_rest) < p.rack_radius) {
        // Measured against the handle's rest point (moves with the gun): LIVE 2026-11-19/20 the rack registered 14-32 ms
        // after the grip on every run, the gun or body moving forward counting as a pull when measured in world.
        const float* z = in.axes + 6;
        s.rack_start = -((in.hand[0] - in.lever_rest[0]) * z[0] + (in.hand[1] - in.lever_rest[1]) * z[1] + (in.hand[2] - in.lever_rest[2]) * z[2]);
        s.rack_ms = in.now_ms;
        s.lever_pull = 0.f; s.racked = false; s.phase = Phase::kRacking; o.event = Event::kRackGrab;
      }
      break;
    case Phase::kRacking: {
      if (in.have_hand) {
        const float* z = in.axes + 6;
        const float along = -((in.hand[0] - in.lever_rest[0]) * z[0] + (in.hand[1] - in.lever_rest[1]) * z[1] +
                              (in.hand[2] - in.lever_rest[2]) * z[2]) - s.rack_start;
        s.lever_pull = std::fmax(0.f, std::fmin(p.lever_max, along));
        // a rack takes a real pull: not within the first 60 ms of the grip (the pin's ease-in)
        if (!s.racked && s.lever_pull >= p.rack_pull && in.now_ms - s.rack_ms >= 60) {s.racked = true; o.event = Event::kRacked;}
      }
      if (release) {
        s.lever_pull = 0.f;
        if (s.racked) {s.phase = Phase::kIdle; s.empty = false; s.rack_done = true; o.event = Event::kDone;}
        else {s.phase = Phase::kNeedRack; o.event = Event::kRackLetGo;}
      }
      break;
    }
  }

  o.block_fire = s.phase != Phase::kIdle || s.empty;
  o.owns_grip = s.phase != Phase::kIdle || o.event == Event::kDone || s.hold_until_release;
  const bool draw = s.phase != Phase::kIdle || s.empty || (s.ours && in.game_reloading);
  if (draw) {
    o.lever_override = true;
    o.lever_pull = s.phase == Phase::kRacking ? s.lever_pull : 0.f;
    switch (s.phase) {
      case Phase::kHolding: case Phase::kOut: case Phase::kNewMag:
        o.mag = MagDraw::kHand; held_mag(s, in, o.mag_world); break;
      case Phase::kEmptyHand: o.mag = MagDraw::kHidden; break;
      default: o.mag = MagDraw::kRest; break;
    }
  }
  return o;
}

// Pacing (Stage 2): the game's reload progress (0..1 of its 3.3 s) is kept between jump and hold while the hands work,
// from the LIVE 2026-11-19 timeline: magazine out 0.15-0.23, new one seated 0.42, refilled 0.445, rack 0.59-0.77,
// ready 0.80. Only while the game's reload is ours.
struct Pace { float jump = 0.f, hold = 1.f; bool active = false; };
inline Pace pace(const State& s, const Input& in) {
  Pace pc;
  if (!in.active || !in.game_reloading || !s.ours) return pc;
  pc.active = true;
  switch (s.phase) {
    case Phase::kHolding: pc.hold = 0.15f; break;           // the game's hand on the magazine
    case Phase::kOut: case Phase::kEmptyHand: pc.hold = 0.22f; break;  // out, before the new one comes
    case Phase::kNewMag: pc.jump = 0.22f; pc.hold = 0.36f; break;      // the new one on its way, before it seats
    case Phase::kNeedRack: pc.jump = 0.40f; pc.hold = 0.55f; break;    // seated and refilled, before the rack
    case Phase::kRacking: if (s.racked) {pc.jump = 0.60f;} else {pc.jump = 0.40f; pc.hold = 0.55f;} break;
    case Phase::kIdle:
      if (s.empty) {pc.hold = 0.15f; break;}                // automatic reload waiting for the hands
      if (s.rack_done) {pc.jump = 0.60f; break;}             // racked: the rest plays to ready
      if (s.seat_ms && in.now_ms - s.seat_ms < 250) pc.jump = 0.40f;  // the game's insert (sound, refill at 0.445)
      else pc.jump = 0.81f;                                  // then ready at once (a magazine with rounds: no rack)
      break;
  }
  return pc;
}
inline float paced(float progress, const Pace& pc) {
  return std::fmin(pc.hold, std::fmax(pc.jump, progress));
}

inline const char* event_name(Event e) {
  switch (e) {
    case Event::kGrab: return "grabs the magazine";
    case Event::kReseat: return "lets go before pulling it out: stays in";
    case Event::kEjected: return "pulls the magazine out";
    case Event::kDropped: return "drops the old magazine";
    case Event::kFetched: return "takes a new magazine from the hip";
    case Event::kDroppedNew: return "drops the new magazine";
    case Event::kSeated: return "seats the new magazine (rack needed)";
    case Event::kRackGrab: return "grips the charging handle";
    case Event::kRacked: return "racks the charging handle";
    case Event::kRackLetGo: return "lets go of the handle before racking";
    case Event::kDone: return "reload done";
    case Event::kAutoReload: return "game reloaded by itself (empty): swap the magazine and rack";
    case Event::kCancel: return "reload cancelled";
    default: return "";
  }
}

// Off-hand hip zone in the head's yaw frame (x right, y up, z forward, m from the headset): at least 0.40 m below the
// headset, not across the body (toward the hand's own side beyond -0.10 m), between 0.35 m behind and 0.30 m in front.
// Leaving needs 5 cm more on every bound.
inline bool in_hip(int hand, const float* p, bool was) {
  const float m = was ? 0.05f : 0.f;
  const float side = hand ? p[0] : -p[0];
  return p[1] < -0.40f + m && side > -0.10f - m && p[2] > -0.35f - m && p[2] < 0.30f + m;
}

}  // namespace manual_reload_math
