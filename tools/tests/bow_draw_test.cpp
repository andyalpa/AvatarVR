// bow_draw_math: FEATURE-07 string pull state machine (nock via the two-hand hold, pull fraction, weak release, cancel, handedness).
#include "../../src/vr/bow_draw_math.hpp"
#include <cmath>
#include <cstdio>
using namespace bow_draw_math;
namespace {
int failures = 0;
void check(bool ok, const char* what) { if (!ok) { std::printf("FAIL: %s\n", what); ++failures; } }
bool near(float a, float b, float e = 1e-3f) { return std::fabs(a - b) < e; }
// A bow aiming along +X (world), string at the origin: drawing pulls toward -X.
Input bow_input(bool grip, float hand_x, float hand_y = 0.f, unsigned long long t = 0) {
  Input in; in.active = true; in.grip = grip; in.have_hand = true; in.now_ms = t;
  in.hand[0] = hand_x; in.hand[1] = hand_y; in.hand[2] = 0.f;
  in.draw_dir[0] = -1.f;
  return in;
}
// The grip press that took the native two-hand hold at the arrow (the caller's grab_two_hand == 1).
Input nock_input(float hand_x, float hand_y = 0.f, unsigned long long t = 0) {
  Input in = bow_input(true, hand_x, hand_y, t); in.nock = true; return in;
}
}  // namespace
int main() {
  Params p;  // length 0.60, floor 0.20, start 0.025, analog
  {  // the bow owns the trigger while active; idle sends 0 (the bow hand's trigger is ignored)
    State st; const Output o = step(st, bow_input(false, 0.5f), p);
    check(o.owns_trigger && o.rt == 0.f && o.event == Event::kNone, "idle owns trigger, rt 0");
  }
  {  // a grip in the air (no two-hand hold) never nocks, even right at the string
    State st; const Output o = step(st, bow_input(true, 0.f), p);
    check(o.event == Event::kNone && st.phase == Phase::kIdle && o.rt == 0.f, "air grip is no nock");
    const Output o2 = step(st, bow_input(false, -0.3f), p);
    check(o2.event == Event::kNone && o2.rt == 0.f, "air grip released: nothing fires");
  }
  {  // nock, pull, release: analog rt follows the pull, release fires
    State st;
    Output o = step(st, nock_input(0.05f), p);
    check(o.event == Event::kNock && st.phase == Phase::kDrawing, "two-hand hold nocks");
    check(o.rt == 0.f, "nocked, no pull: no trigger yet (the game must not draw by itself)");
    o = step(st, bow_input(true, 0.05f - 0.015f), p);
    check(o.rt == 0.f, "under start_pull: still no trigger");
    o = step(st, bow_input(true, 0.05f - 0.30f), p);
    check(near(o.fraction, 0.5f) && near(o.rt, 0.5f), "half pull = half trigger");
    o = step(st, bow_input(true, 0.05f - 0.90f), p);
    check(near(o.fraction, 1.f) && near(o.rt, 1.f), "overpull clamps to full");
    o = step(st, bow_input(true, 0.05f - 0.30f, 0.20f), p);
    check(near(o.fraction, 0.5f), "sideways motion does not add pull");
    o = step(st, bow_input(true, 0.40f), p);
    check(near(o.fraction, 0.f) && o.event == Event::kCancelShot, "pushing forward after a draw cancels");
    o = step(st, bow_input(true, 0.05f - 0.12f, 0.f, 1000), p);
    o = step(st, bow_input(false, 0.05f - 0.12f, 0.f, 1000), p);
    check(o.event == Event::kRelease && o.rt == 0.f && st.phase == Phase::kIdle, "short release fires (weak shot)");
    check(near(o.fraction, 0.2f), "release reports the draw it fired at");
  }
  {  // tracking gap keeps the pull
    State st; step(st, nock_input(0.f), p); step(st, bow_input(true, -0.36f), p);
    Input gap = bow_input(true, 0.f); gap.have_hand = false;
    const Output o = step(st, gap, p);
    check(near(o.fraction, 0.6f) && st.phase == Phase::kDrawing, "tracking gap keeps the draw");
  }
  {  // bow lost mid-draw: cancel, trigger handed back
    State st; step(st, nock_input(0.f), p); step(st, bow_input(true, -0.3f), p);
    Input lost = bow_input(true, -0.3f); lost.active = false;
    const Output o = step(st, lost, p);
    check(o.event == Event::kCancel && !o.owns_trigger && st.phase == Phase::kIdle, "bow lost cancels");
    // regaining the bow with the grip still held does not nock again until a new press
    const Output o2 = step(st, bow_input(true, 0.f), p);
    check(o2.event == Event::kNone, "no nock without a new press after a cancel");
  }
  {  // hold mode: full trigger while drawing regardless of pull
    Params h = p; h.mode = Mode::kHold; State st;
    Output o = step(st, nock_input(0.f), h);
    check(o.rt == 0.f, "hold mode: no trigger at the nock");
    o = step(st, bow_input(true, -0.1f), h);
    check(near(o.rt, 1.f), "hold mode: full trigger once pulling");
    o = step(st, bow_input(false, -0.1f), h);
    check(o.event == Event::kRelease && o.rt == 0.f, "hold mode: release drops the trigger");
  }
  {  // press-fire bows: nothing while drawing, a tap on release
    Params pf = p; pf.press_fire = true; State st;
    Output o = step(st, nock_input(0.f, 0.f, 1000), pf);
    check(o.rt == 0.f, "press-fire: no trigger at the nock");
    o = step(st, bow_input(true, -0.3f, 0.f, 1100), pf);
    check(o.rt == 0.f, "press-fire: no trigger while drawing");
    o = step(st, bow_input(false, -0.3f, 0.f, 1200), pf);
    check(o.rt == 1.f && o.event == Event::kRelease, "press-fire: tap on release");
    o = step(st, bow_input(false, -0.3f, 0.f, 1200 + pf.tap_ms - 1), pf);
    check(o.rt == 1.f, "press-fire: tap lasts tap_ms");
    o = step(st, bow_input(false, -0.3f, 0.f, 1200 + pf.tap_ms), pf);
    check(o.rt == 0.f, "press-fire: tap ends");
  }
  {  // handedness: the math is world-space and hand-agnostic; a bow aiming the other way (mirror) pulls the other way
    State st; Input a = nock_input(0.f); a.draw_dir[0] = 1.f;
    step(st, a, p);
    Input b = bow_input(true, 0.30f); b.draw_dir[0] = 1.f;
    const Output o = step(st, b, p);
    check(near(o.fraction, 0.5f), "mirrored bow: pull along its own draw direction");
  }
  {  // grab and let go without pulling: no shot, the trigger never pressed
    State st; step(st, nock_input(0.f), p); step(st, bow_input(true, -0.02f), p);
    const Output o = step(st, bow_input(false, -0.02f), p);
    check(o.event == Event::kDrop && o.rt == 0.f && st.phase == Phase::kIdle, "release before pulling drops, no shot");
  }
  {  // push forward after drawing: cancel (X with RT held for tap_ms), then no trigger; pulling again redraws; letting go = no shot
    State st; step(st, nock_input(0.f, 0.f, 0), p);
    step(st, bow_input(true, -0.20f, 0.f, 100), p);
    Output o = step(st, bow_input(true, -0.01f, 0.f, 200), p);
    check(o.event == Event::kCancelShot && o.cancel_button && o.rt == 1.f, "push forward cancels with X held and RT kept");
    o = step(st, bow_input(true, -0.01f, 0.f, 200 + p.tap_ms), p);
    check(!o.cancel_button && o.rt == 0.f && st.phase == Phase::kDrawing, "after the cancel tap: string still held, no trigger");
    o = step(st, bow_input(true, -0.10f, 0.f, 400), p);
    check(o.rt > 0.f, "pulling again redraws");
    step(st, bow_input(true, -0.005f, 0.f, 500), p);
    o = step(st, bow_input(false, -0.005f, 0.f, 700), p);
    check(o.event == Event::kDrop, "let go after a cancel: no shot");
  }
  {  // a small pull (under cancel_peak) brought back is no cancel
    State st; step(st, nock_input(0.f), p); step(st, bow_input(true, -0.04f), p);
    const Output o = step(st, bow_input(true, -0.005f), p);
    check(o.event != Event::kCancelShot, "small pull back is no cancel");
  }
  if (failures == 0) std::printf("bow_draw_test: all passed\n");
  return failures == 0 ? 0 : 1;
}
