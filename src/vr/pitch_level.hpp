#pragma once
// Keep the game's internal look pitch level in VR (2026-10-15). [LIVE] look +0x38 is rewritten by the game every frame
// from its internal look state (writes to it do not persist: the 06:57 run accumulated thousands of degrees of
// "discarded" pitch), and that internal pitch bends/shifts the first-person rig (shoulders up to 0.45 m), slides the
// camera bone forward (up to 0.29 m) and hides the body. The only reliable lever is the input: in VR gameplay the right
// stick's up/down never reaches the game; instead this controller feeds a correcting stick value that steers the
// game's own pitch back to level. The stick sign is learned from the response (inverted look settings work).
#include <algorithm>
#include <cmath>

namespace pitch_level {

struct Controller {
  float deadband = 0.0175f;  // 1 degree: below this, no stick (the game sees the stick centred)
  float gain = 2.5f;         // stick per radian of pitch error
  float min_stick = 0.35f;   // above typical right-stick deadzones (XInput 8689/32767 = 0.265)
  float max_stick = 0.7f;
  int sign = 1;              // +1: positive stick raises pitch (normal look); learned
  int window = 12;           // calls between sign checks
  int calls = 0;
  float window_start = 0.f;  // |pitch| at the start of the current correcting window
  bool correcting = false;
  int flips = 0;
  float target = 0.f;        // pitch to hold, radians (menu "Default game pitch"); 0 = level

  // game_pitch: game pitch in radians (+ up). Returns the right-stick Y (-1..1) to report to the game.
  float step(float game_pitch) {
    const float pitch = game_pitch - target;  // error relative to the chosen default pitch
    const float error = std::fabs(pitch);
    if (!std::isfinite(pitch) || error < deadband) {
      correcting = false;
      calls = 0;
      return 0.f;
    }
    if (!correcting) {
      correcting = true;
      calls = 0;
      window_start = error;
    } else if (++calls >= window) {
      // Commanded towards level for a whole window but the error grew noticeably: the stick sign is the other way.
      if (error > window_start + 0.0087f) {
        sign = -sign;
        ++flips;
      }
      calls = 0;
      window_start = error;
    }
    float stick = std::clamp(gain * error, min_stick, max_stick);
    return pitch > 0.f ? -sign * stick : sign * stick;
  }
};

}  // namespace pitch_level
