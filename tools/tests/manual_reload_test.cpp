// manual_reload_math: FEATURE-05 rifle reload by hand (grab, pull out, drop, hip fetch, seat, rack after an automatic
// reload), fire block, X tap, button fallback, support-point priority, cancel, hip zone handedness.
#include "../../src/vr/manual_reload_math.hpp"
#include <cmath>
#include <cstdio>
using namespace manual_reload_math;
namespace {
int failures = 0;
void check(bool ok, const char* what) { if (!ok) { std::printf("FAIL: %s\n", what); ++failures; } }
bool near(float a, float b, float e = 1e-3f) { return std::fabs(a - b) < e; }

// Gun along world +Z (identity axes) at the origin: magazine rest at (0,-0.1,0.1), charging handle at (0,0.05,-0.05).
struct Rig {
  State s; Params p; unsigned long long t = 1000; bool reloading = false; bool button = false;
  bool support = true;  // front grip at (0,-0.05,0.35): the support wrist
  Output go(bool grip, float x, float y, float z, bool hip = false, float support_d = 1e9f) {
    Input in; in.active = true; in.now_ms = (t += 16); in.grip = grip; in.have_hand = true;
    in.hand[0] = x; in.hand[1] = y; in.hand[2] = z;
    in.mag_rest[1] = -0.1f; in.mag_rest[2] = 0.1f;
    in.lever_rest[1] = 0.05f; in.lever_rest[2] = -0.05f;
    const float ax[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; for (int i = 0; i < 9; ++i) in.axes[i] = ax[i];
    in.support_d = support_d; in.at_hip = hip;
    in.have_support = support; in.support_pt[1] = -0.05f; in.support_pt[2] = 0.35f; in.game_reloading = reloading; in.button = button;
    return step(s, in, p);
  }
};

void tactical() {
  Rig r;
  Output o = r.go(false, 0.f, -0.15f, 0.1f);
  check(!o.block_fire && o.mag == MagDraw::kGame, "idle: fire and the game's own drawing");
  o = r.go(true, 0.f, -0.15f, 0.1f);
  check(o.event == Event::kGrab && o.block_fire && o.owns_grip && o.press_x, "grip at the magazine grabs it and starts the game's reload");
  check(o.mag == MagDraw::kHand && near(o.mag_world[1], -0.1f), "the grabbed magazine keeps its place in the hand");
  o = r.go(true, 0.f, -0.22f, 0.1f);
  check(o.event == Event::kNone && !o.press_x, "7 cm is not out yet");
  o = r.go(true, 0.f, -0.27f, 0.1f);
  check(o.event == Event::kEjected && !o.press_x, "past 10 cm it is out (no second X right after the grab's)");
  check(near(o.mag_world[1], -0.22f), "the magazine follows the hand");
  r.reloading = true;
  o = r.go(false, 0.f, -0.5f, 0.1f);
  check(o.event == Event::kDropped && o.mag == MagDraw::kHidden && o.block_fire, "letting go drops it (hidden), no fire");
  o = r.go(true, 0.3f, -0.6f, 0.f, false);
  check(o.event == Event::kNone, "a grip away from the hip takes nothing");
  o = r.go(false, 0.3f, -0.6f, 0.f, true);
  o = r.go(true, 0.3f, -0.6f, 0.f, true);
  check(o.event == Event::kFetched && o.mag == MagDraw::kHand, "a grip at the hip takes a new magazine");
  o = r.go(true, 0.f, -0.25f, 0.1f);
  check(o.event == Event::kNone && o.block_fire, "not at the well yet");
  o = r.go(true, 0.f, -0.14f, 0.1f);
  check(o.event == Event::kDone && !o.block_fire, "seated: a reload with rounds left needs no rack");
  check(o.owns_grip && o.mag == MagDraw::kRest, "the seating grip stays the reload's; drawn seated while the game reloads");
  o = r.go(true, 0.f, -0.14f, 0.1f);
  check(o.owns_grip && o.event == Event::kNone, "holding on after seating does not grab it again");
  o = r.go(false, 0.f, -0.14f, 0.1f);
  check(!o.owns_grip && o.mag == MagDraw::kRest, "released; still drawn seated until the game's reload ends");
  r.reloading = false;
  o = r.go(false, 0.f, -0.14f, 0.1f);
  check(o.mag == MagDraw::kGame && !o.lever_override, "after the game's reload: the game draws again");
}

void automatic_needs_rack() {
  Rig r;
  r.go(false, 0.5f, 0.f, 0.f);
  r.reloading = true;
  Output o = r.go(false, 0.5f, 0.f, 0.f);
  check(o.event == Event::kAutoReload && o.block_fire && o.mag == MagDraw::kRest, "automatic reload: magazine drawn in, no fire");
  r.reloading = false;
  o = r.go(false, 0.5f, 0.f, 0.f);
  check(o.block_fire && o.mag == MagDraw::kRest, "still waiting for the hands after the game's timer");
  o = r.go(true, 0.f, -0.15f, 0.1f);
  check(o.event == Event::kGrab && o.press_x, "grab (game reload already over: X again, refused when full)");
  o = r.go(true, 0.f, -0.3f, 0.1f);
  check(o.event == Event::kEjected, "out");
  r.go(false, 0.f, -0.5f, 0.1f);
  r.go(true, 0.3f, -0.6f, 0.f, true);
  o = r.go(true, 0.f, -0.15f, 0.1f);
  check(o.event == Event::kSeated && o.block_fire, "seated after an empty magazine: rack still needed");
  o = r.go(true, 0.f, 0.05f, -0.05f);
  check(o.event == Event::kNone, "the seating grip held up to the handle does not rack");
  r.go(false, 0.f, 0.05f, -0.05f);
  o = r.go(true, 0.f, 0.05f, -0.05f);
  check(o.event == Event::kRackGrab && o.lever_override && near(o.lever_pull, 0.f), "grip at the handle");
  o = r.go(true, 0.f, 0.05f, -0.10f);
  check(near(o.lever_pull, 0.05f) && o.event == Event::kNone, "the handle follows the pull");
  o = r.go(false, 0.f, 0.05f, -0.10f);
  check(o.event == Event::kRackLetGo && o.block_fire, "a short pull does not rack");
  r.go(true, 0.f, 0.05f, -0.05f);
  o = r.go(true, 0.f, 0.05f, -0.30f);
  check(o.event == Event::kNone, "a pull within 60 ms of the grip is not a rack yet");
  r.t += 60; o = r.go(true, 0.f, 0.05f, -0.30f);
  check(o.event == Event::kRacked && near(o.lever_pull, 0.17f), "racked; the handle stops at its travel");
  o = r.go(false, 0.f, 0.05f, -0.30f);
  check(o.event == Event::kDone && !o.block_fire && o.mag == MagDraw::kGame, "released: done, fire allowed");
}

void button_fallback_and_reseat() {
  Rig r;
  r.button = true; r.go(false, 0.5f, 0.f, 0.f); r.button = false;
  r.reloading = true;
  Output o = r.go(false, 0.5f, 0.f, 0.f);
  check(o.event == Event::kNone && !o.block_fire && o.mag == MagDraw::kGame, "button reload: the game's own, untouched");
  r.reloading = false; r.go(false, 0.5f, 0.f, 0.f);
  o = r.go(true, 0.f, -0.15f, 0.1f);
  o = r.go(true, 0.f, -0.18f, 0.1f);
  o = r.go(false, 0.f, -0.18f, 0.1f);
  check(o.event == Event::kReseat && !o.block_fire && !o.press_x, "let go before out: stays in");
  r.go(false, 0.f, -0.15f, 0.1f);
  o = r.go(true, 0.f, -0.08f, 0.30f);
  check(o.event == Event::kNone && !o.owns_grip, "at the front grip: left to the two-hand grab");
  r.go(false, 0.f, -0.08f, 0.30f);
  o = r.go(true, 0.05f, -0.30f, 0.12f);
  check(o.event == Event::kGrab, "25 cm below the magazine origin, behind the front grip: grabs (HEADSET 2026-11-19 distances)");
  r.go(false, 0.05f, -0.30f, 0.12f);  // reseat
  {Input in; in.active = true; in.now_ms = (r.t += 16); in.grip = true; in.have_hand = true;
   in.hand[1] = -0.2f; in.mag_rest[1] = -0.1f; in.mag_rest[2] = 0.1f; in.axes[0] = in.axes[4] = in.axes[8] = 1.f;
   in.handle_d = 0.08f;
   o = step(r.s, in, r.p);
   check(o.event == Event::kNone, "at the trigger handle: left to the hand swap");
   in.grip = false; in.now_ms = (r.t += 16); step(r.s, in, r.p);}
  r.support = false;
  o = r.go(true, 0.f, -0.15f, 0.1f, false, 0.03f);
  check(o.event == Event::kNone && !o.owns_grip, "no support point known: nearer the support distance loses");
}

void pacing() {
  Rig r;
  Input in; in.active = true; in.game_reloading = true;
  check(!pace(r.s, in).active, "not our reload: no pacing");
  r.go(false, 0.f, -0.15f, 0.1f);
  r.go(true, 0.f, -0.15f, 0.1f);           // grab, X
  r.reloading = true; r.go(true, 0.f, -0.15f, 0.1f);  // the game's reload starts: ours
  in.now_ms = r.t;
  Pace pc = pace(r.s, in);
  check(pc.active && near(pc.hold, 0.15f), "holding the magazine: the game holds at its hand on the magazine");
  check(near(paced(0.5f, pc), 0.15f) && near(paced(0.05f, pc), 0.05f), "progress clamps at the hold, runs below it");
  r.go(true, 0.f, -0.3f, 0.1f); r.go(false, 0.f, -0.5f, 0.1f);
  pc = pace(r.s, in);
  check(near(pc.hold, 0.22f), "magazine out: held before the new one");
  r.go(true, 0.3f, -0.6f, 0.f, true);
  pc = pace(r.s, in);
  check(near(pc.jump, 0.22f) && near(pc.hold, 0.36f), "new magazine in the hand: up to just before the seat");
  Output o = r.go(true, 0.f, -0.15f, 0.1f);
  check(o.event == Event::kDone, "seated");
  in.now_ms = r.t; pc = pace(r.s, in);
  check(near(pc.jump, 0.40f) && near(pc.hold, 1.f), "seated: the game's insert plays (refill at 0.445)");
  in.now_ms = r.t + 300; pc = pace(r.s, in);
  check(near(pc.jump, 0.81f), "then straight to ready: no rack with rounds left");
}

void cancel_and_hip() {
  Rig r;
  r.go(true, 0.f, -0.15f, 0.1f);
  Input in; in.active = false; in.now_ms = 5000;
  Output o = step(r.s, in, r.p);
  check(o.event == Event::kCancel && r.s.phase == Phase::kIdle && !o.block_fire, "rifle gone: cancelled");
  const float right_hip[3] = {0.2f, -0.6f, 0.f}, left_hip[3] = {-0.2f, -0.6f, 0.f}, chest[3] = {0.f, -0.25f, 0.2f};
  check(in_hip(1, right_hip, false) && !in_hip(1, left_hip, false), "right hand: right hip only");
  check(in_hip(0, left_hip, false) && !in_hip(0, right_hip, false), "left hand: left hip only");
  check(!in_hip(0, chest, false), "chest is not the hip");
}
}  // namespace

int main() {
  tactical();
  automatic_needs_rack();
  button_fallback_and_reseat();
  pacing();
  cancel_and_hip();
  if (failures) {std::printf("%d failure(s)\n", failures); return 1;}
  std::printf("manual_reload_test: all passed\n");
  return 0;
}
