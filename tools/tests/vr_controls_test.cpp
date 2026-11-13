#include "../../src/vr/vr_controls_math.hpp"
#include <cmath>
#include <cstdio>
using namespace vr_controls_math;
namespace {
bool close(float a, float b, float e = 1e-4f) { return std::fabs(a - b) < e; }
}
int main() {
  // 1. head frame: looking straight ahead keeps the axes; looking 90 deg right turns them.
  float ahead[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // rows (m[r], m[3+r], m[6+r]) = identity
  const float d[3] = {0.1f, -0.2f, 0.3f};
  float p[3];
  if (!head_frame(ahead, d, p) || !close(p[0], 0.1f) || !close(p[1], -0.2f) || !close(p[2], 0.3f)) return 1;
  // head forward = +x (row 2 = (m[2], m[5], m[8]) = (1, 0, 0)): a hand at +x is straight ahead
  float right90[9] = {0, 0, 1, 0, 1, 0, -1, 0, 0};
  const float dx[3] = {0.4f, 0.f, 0.f};
  if (!head_frame(right90, dx, p) || !close(p[2], 0.4f) || !close(p[0], 0.f)) return 2;
  // looking straight up: no yaw
  float up[9] = {1, 0, 0, 0, 0, 1, 0, -1, 0};
  if (head_frame(up, d, p)) return 3;

  // 2. zones with hysteresis
  const float near_head[3] = {0.05f, -0.05f, 0.12f}, mid[3] = {0.f, 0.f, 0.23f};
  if (!in_head(near_head, false) || in_head(mid, false) || !in_head(mid, true)) return 4;
  const float right_shoulder[3] = {0.18f, -0.12f, -0.10f}, left_shoulder[3] = {-0.18f, -0.12f, -0.10f};
  if (!in_shoulder(1, right_shoulder, false) || in_shoulder(0, right_shoulder, false)) return 5;
  if (!in_shoulder(0, left_shoulder, false) || in_shoulder(1, left_shoulder, false)) return 6;
  const float in_front[3] = {0.18f, -0.12f, 0.25f}, low[3] = {0.18f, -0.5f, -0.1f};
  if (in_shoulder(1, in_front, false) || in_shoulder(1, low, false)) return 7;
  const float edge[3] = {0.18f, -0.12f, 0.05f};  // 3 cm past the face plane: in only while already inside
  if (in_shoulder(1, edge, false) || !in_shoulder(1, edge, true)) return 8;

  // 3. flicks: one event per push, re-arm under 0.30
  Flick f;
  if (flick(f, 0.5f) != 0 || flick(f, 0.8f) != 1 || flick(f, 0.9f) != 0 || flick(f, -0.9f) != 0) return 9;
  if (flick(f, 0.1f) != 0 || flick(f, -0.75f) != -1) return 10;

  // 4. shoulder tap decisions
  if (equip_decision(0, false, 1) != Equip::kEquipToHand) return 11;
  if (equip_decision(1, true, 1) != Equip::kHolster) return 12;
  if (equip_decision(0, true, 1) != Equip::kSwitchHand) return 13;

  std::puts("vr_controls: head yaw frame, head/shoulder zones with hysteresis, flicks, equip decisions pass");
  return 0;
}
