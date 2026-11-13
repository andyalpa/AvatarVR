#pragma once

#include <cstdint>

// Per-eye headset pose snapshot shared between the OpenXR worker (producer), the camera code (consumer, game
// thread) and the Present-side layer submission. OpenXR LOCAL space: x right, y up, -z forward, metres.
namespace head_pose {

struct Eye {
  float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;  // orientation
  float px = 0.f, py = 0.f, pz = 0.f;            // position
  float fov[4] = {};                             // angleLeft, angleRight, angleUp, angleDown (radians)
  // Set by native_stereo when it rendered this eye with a computed projection ("autofov"): the frustum tangents
  // actually rendered (left < 0, right, up, down < 0), which the projection layer must then be labelled with.
  float rtan[4] = {};
  bool rvalid = false;
};

// Controller pose (OpenXR grip or aim pose) in the same LOCAL space and for the same display time as the eyes.
// Aim pose: -z along the pointing ray. Grip pose: origin in the palm, -z along the held object.
struct Hand {
  float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;
  float px = 0.f, py = 0.f, pz = 0.f;
  bool valid = false;  // orientation and position valid this frame
};

struct Snapshot {
  Eye eye[2];
  Hand grip[2];  // 0 = left hand, 1 = right hand
  Hand aim[2];
  std::int64_t display_time = 0;  // XrTime the views were predicted for
  std::uint64_t seq = 0;          // increments per published snapshot
  bool valid = false;
};

}  // namespace head_pose
