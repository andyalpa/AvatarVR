#pragma once
// Pure math for in-mod frame extrapolation (frame_gen.cpp, 2026-11-12). No D3D12 or OpenXR, so
// tools/tests/frame_gen_test.cpp checks it alone.
#include <cmath>
#include <cstdint>

namespace frame_gen_math {

// How far to push the last real frame: elapsed display time since it was shown, in units of the game's frame
// interval. 0 = the real frame itself, 1 = where the next game frame would be. Clamped to [0, max_k]; an interval
// outside 5..60 ms (pause, load, first frame) gives 0 (no extrapolation).
inline float extrapolation_k(int64_t gen_display_ns, int64_t real_display_ns, int64_t real_interval_ns, float max_k = 1.5f) {
  if (real_interval_ns < 5000000 || real_interval_ns > 60000000 || gen_display_ns <= real_display_ns) return 0.f;
  const float k = static_cast<float>(static_cast<double>(gen_display_ns - real_display_ns) / static_cast<double>(real_interval_ns));
  return k > max_k ? max_k : k;
}

// DLSS motion vectors are NDC deltas from the current to the previous frame; Streamline's mvecScale turns them into
// UV deltas (live: 0.5, -0.5, the Y flip from NDC up to UV down). A pixel at uv moved to uv - mv_uv per game frame, so
// k frames ahead it is at uv - k * mv_uv, and the extrapolated image is gathered from uv + k * mv_uv.
inline void gather_uv(float u, float v, float mv_x, float mv_y, float scale_x, float scale_y, float k, float* su, float* sv) {
  *su = u + k * mv_x * scale_x;
  *sv = v + k * mv_y * scale_y;
}

// Pose extrapolation between the previous and the last real frame's labels: position linear, orientation slerp
// (shortest arc), both by 1 + k (k frames past the last one). Quaternions are (x, y, z, w).
struct Pose {
  float qx, qy, qz, qw, px, py, pz;
};
inline Pose extrapolate(const Pose& prev, const Pose& last, float k) {
  Pose out = last;
  out.px = last.px + (last.px - prev.px) * k;
  out.py = last.py + (last.py - prev.py) * k;
  out.pz = last.pz + (last.pz - prev.pz) * k;
  // delta = last * conj(prev); out = delta^k * last
  float ax = prev.qx, ay = prev.qy, az = prev.qz, aw = prev.qw;
  float dot = ax * last.qx + ay * last.qy + az * last.qz + aw * last.qw;
  if (dot < 0.f) { ax = -ax; ay = -ay; az = -az; aw = -aw; dot = -dot; }
  // conj(prev)
  const float cx = -ax, cy = -ay, cz = -az, cw = aw;
  // d = last * conj(prev)
  float dx = last.qw * cx + last.qx * cw + last.qy * cz - last.qz * cy;
  float dy = last.qw * cy - last.qx * cz + last.qy * cw + last.qz * cx;
  float dz = last.qw * cz + last.qx * cy - last.qy * cx + last.qz * cw;
  float dw = last.qw * cw - last.qx * cx - last.qy * cy - last.qz * cz;
  if (dw < 0.f) { dx = -dx; dy = -dy; dz = -dz; dw = -dw; }
  const float s = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (s < 1e-7f) return out;  // no rotation between the frames
  const float angle = 2.f * std::atan2(s, dw) * k;
  const float h = 0.5f * angle, sh = std::sin(h) / s;
  const float ex = dx * sh, ey = dy * sh, ez = dz * sh, ew = std::cos(h);
  // out.q = e * last.q
  out.qx = ew * last.qx + ex * last.qw + ey * last.qz - ez * last.qy;
  out.qy = ew * last.qy - ex * last.qz + ey * last.qw + ez * last.qx;
  out.qz = ew * last.qz + ex * last.qy - ey * last.qx + ez * last.qw;
  out.qw = ew * last.qw - ex * last.qx - ey * last.qy - ez * last.qz;
  const float n = std::sqrt(out.qx * out.qx + out.qy * out.qy + out.qz * out.qz + out.qw * out.qw);
  if (n > 0.f) { out.qx /= n; out.qy /= n; out.qz /= n; out.qw /= n; }
  return out;
}

// Ring of capture banks: writers use `write`, the published frame is `ready`. A bank becomes ready only when both
// views were captured into it since the last publish.
struct Banks {
  int write = 0, ready = -1;
  unsigned captured[3] = {0, 0, 0};  // bit 0 = main view, bit 1 = second view
  void mark(int view) { captured[write] |= 1u << view; }
  bool publish() {  // at a real Present; returns whether a complete bank became ready
    const bool complete = captured[write] == 3u;
    if (complete) ready = write;
    write = (write + 1) % 3;
    captured[write] = 0;
    return complete;
  }
};

}  // namespace frame_gen_math
