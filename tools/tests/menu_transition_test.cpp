#include "../../src/vr/menu_transition.hpp"
#include <cstdio>
#include <cstdlib>
using namespace menu_transition;
void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    std::abort();
  }
}
const double kFrame = 1.0 / 72.0;

int main() {
  // Placement: a steady head is placed after start_frames at session start, never on the first stale frames.
  {
    Placer p;
    p.request(false, true);
    int placed_at = -1;
    for (int i = 0; i < 200 && placed_at < 0; ++i) {
      if (p.step(10.f, true, false, 0, 0, 0, 2.5f) == Step::Place) placed_at = i + 1;
    }
    check(placed_at == p.cfg.start_frames, "startup placement waits start_frames");
  }
  // A turning head is not placed until it settles, and the max wait still places it.
  {
    Placer p;
    p.request(false, false);
    float yaw = 0.f;
    int placed_at = -1;
    for (int i = 0; i < 200 && placed_at < 0; ++i) {
      yaw += i < 30 ? 2.f : 0.f;  // turning ~144 deg/s, then still
      if (p.step(yaw, true, false, 0, 0, 0, 2.5f) == Step::Place) placed_at = i + 1;
    }
    check(placed_at > 30 && placed_at <= 30 + p.cfg.stable_frames + 1, "placed once the head settles");
    p.request(false, false);
    int forced = -1;
    for (int i = 0; i < 200 && forced < 0; ++i) {
      if (p.step(static_cast<float>(i * 2), true, true, 0, 2.5f, 0, 2.5f) == Step::Place) forced = i + 1;
    }
    check(forced == p.cfg.max_frames, "a never-still head is placed after max_frames");
  }
  // Lazy: a screen still in view is kept at once; one behind the user is placed again; untracked heads never keep.
  {
    Placer p;
    p.request(false, false);
    while (p.step(0.f, true, false, 0, 0, 0, 2.5f) != Step::Place) {}
    p.request(true, false);
    check(p.step(20.f, true, true, 0.f, 2.5f, 0.f, 2.5f) == Step::Keep, "screen 20 deg off gaze kept");
    p.request(true, false);
    check(p.step(120.f, true, true, 0.f, 2.5f, 0.f, 2.5f) == Step::Wait, "screen 120 deg off gaze placed again");
    p.request(true, false);
    check(p.step(0.f, true, true, 0.f, 6.f, 0.f, 2.5f) == Step::Wait, "screen walked away from is placed again");
    p.request(true, false);
    check(p.step(0.f, true, true, 0.f, 2.5f, 0.8f, 2.5f) == Step::Wait, "screen far above the head is placed again");
    p.request(true, false);
    check(p.step(0.f, false, true, 0.f, 2.5f, 0.f, 2.5f) == Step::Wait, "untracked head never keeps");
    Placer fresh;
    fresh.request(true, false);
    check(fresh.step(0.f, true, true, 0.f, 2.5f, 0.f, 2.5f) == Step::Wait, "never-placed screen is not kept");
  }
  // Fade: session start begins black, the flat screen fades in once placed.
  {
    Fade f;
    double t = 0;
    check(f.update({Kind::Flat, false, false}, t) == 1.f, "first picture black");
    for (int i = 0; i < 20; ++i) check(f.update({Kind::Flat, false, false}, t += kFrame) == 1.f, "black until placed");
    float a = 1.f;
    for (int i = 0; i < 72; ++i) a = f.update({Kind::Flat, true, false}, t += kFrame);
    check(a == 0.f, "fully clear 1 s after placement");
  }
  // Pause: stereo -> flat cuts to black, clears within ~0.45 s (hold 0.1 + fade 0.3).
  {
    Fade f;
    double t = 0;
    f.update({Kind::Stereo, true, false}, t);
    for (int i = 0; i < 400; ++i) f.update({Kind::Stereo, true, false}, t += kFrame);
    check(f.alpha() == 0.f, "stereo clear after settling");
    check(f.update({Kind::Flat, true, false}, t += kFrame) == 1.f, "cut to black on stereo -> flat");
    int clear_at = -1;
    for (int i = 0; i < 200 && clear_at < 0; ++i) {
      if (f.update({Kind::Flat, true, false}, t += kFrame) == 0.f) clear_at = i + 1;
    }
    check(clear_at > 0 && clear_at * kFrame < 0.5, "pause menu clear within 0.5 s");
  }
  // Load -> gameplay: the flat picture fades to black while stereo is pending, stereo fades in after settling.
  {
    Fade f;
    double t = 0;
    for (int i = 0; i < 200; ++i) f.update({Kind::Flat, true, false}, t += kFrame);
    check(f.alpha() == 0.f, "loading screen visible");
    for (int i = 0; i < 20; ++i) f.update({Kind::Flat, true, true}, t += kFrame);
    check(f.alpha() == 1.f, "black within 20 frames of stereo pending");
    for (int i = 0; i < 60; ++i) check(f.update({Kind::Flat, true, true}, t += kFrame) == 1.f, "black while pending");
    check(f.update({Kind::Stereo, true, false}, t += kFrame) == 1.f, "stereo starts black");
    for (int i = 0; i < 30; ++i) check(f.update({Kind::Stereo, true, false}, t += kFrame) == 1.f, "black while stereo settles");
    for (int i = 0; i < 72; ++i) f.update({Kind::Stereo, true, false}, t += kFrame);
    check(f.alpha() == 0.f, "stereo visible");
  }
  // Pending that never turns into stereo gives the flat picture back after pending_max_s.
  {
    Fade f;
    double t = 0;
    for (int i = 0; i < 100; ++i) f.update({Kind::Flat, true, false}, t += kFrame);
    for (int i = 0; i < 72 * 7; ++i) f.update({Kind::Flat, true, true}, t += kFrame);
    check(f.alpha() == 0.f, "pending timeout shows the flat screen again");
  }
  // A short stereo stint (post-load dip) keeps the following flat screen black, so stereo returns without a flat flash.
  {
    Fade f;
    double t = 0;
    f.update({Kind::Flat, true, false}, t);
    f.update({Kind::Stereo, true, false}, t += kFrame);
    for (int i = 0; i < 30; ++i) f.update({Kind::Stereo, true, false}, t += kFrame);
    f.update({Kind::Flat, true, false}, t += kFrame);
    for (int i = 0; i < 72; ++i) check(f.update({Kind::Flat, true, false}, t += kFrame) == 1.f, "flat after a dip stays black");
  }
  // Pause menus: a pause soon after unpausing is not a post-load dip (HEADSET 2026-11-08: 2 s of black), and stereo comes
  // back from a pause menu after the short settle.
  {
    Fade f;
    double t = 0;
    FadeInput flat{Kind::Flat, true, false, true}, stereo{Kind::Stereo, true, false, true};
    f.update(flat, t);
    for (int i = 0; i < 100; ++i) f.update(flat, t += kFrame);
    f.update(stereo, t += kFrame);
    int clear_at = -1;
    for (int i = 0; i < 200 && clear_at < 0; ++i) {
      if (f.update(stereo, t += kFrame) == 0.f) clear_at = i + 1;
    }
    check(clear_at > 0 && clear_at * kFrame < 0.6, "unpause clear within 0.6 s");
    for (int i = 0; i < 50; ++i) f.update(stereo, t += kFrame);  // ~1.4 s of stereo, then pause again
    f.update(flat, t += kFrame);
    clear_at = -1;
    for (int i = 0; i < 200 && clear_at < 0; ++i) {
      if (f.update(flat, t += kFrame) == 0.f) clear_at = i + 1;
    }
    check(clear_at > 0 && clear_at * kFrame < 0.5, "quick re-pause clear within 0.5 s");
  }
  // Level sprite indices.
  check(level_for(0.f) == 0 && level_for(1.f) == kLevels - 1 && level_for(2.f) == kLevels - 1 && level_for(-1.f) == 0,
        "level_for clamps");
  std::printf("menu_transition_test: OK\n");
  return 0;
}
