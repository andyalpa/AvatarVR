#include "matrix_detector.hpp"

#include <cmath>
#include <cstring>

namespace matrix_detector {
namespace {

float at(const float* m, int row, int col, bool column_major) {
  return column_major ? m[col * 4 + row] : m[row * 4 + col];
}

bool finite4(float a, float b, float c, float d) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(d);
}

bool perspective(const float* m, bool column_major, float* fov_y, float* near_z, float* far_z, bool* near_ok, bool* far_ok) {
  const float x_scale = at(m, 0, 0, column_major);
  const float y_scale = at(m, 1, 1, column_major);
  const float m22 = at(m, 2, 2, column_major);
  const float m23 = at(m, 2, 3, column_major);
  const float m32 = at(m, 3, 2, column_major);
  const float m33 = at(m, 3, 3, column_major);
  if (!finite4(x_scale, y_scale, m22, m23) || !std::isfinite(m32) || !std::isfinite(m33)) {
    return false;
  }
  if (std::fabs(std::fabs(m23) - 1.f) > 0.02f || std::fabs(m33) > 0.02f) {
    return false;
  }
  if (y_scale < 0.05f || y_scale > 20.f || x_scale < 0.05f || x_scale > 20.f) {
    return false;
  }
  *fov_y = 2.f * std::atan(1.f / y_scale) * (180.f / 3.14159265f);
  *near_ok = false;
  *far_ok = false;
  if (std::fabs(m22 - 1.f) < 0.001f) {
    const float near_candidate = -m32;
    if (near_candidate > 0.f && near_candidate < 100.f) {
      *near_z = near_candidate;
      *near_ok = true;
    }
    return true;
  }
  if (std::fabs(m22) < 1e-4f) {
    return true;
  }
  const float near_candidate = -m32 / m22;
  if (near_candidate > 1e-4f && near_candidate < 1000.f) {
    *near_z = near_candidate;
    *near_ok = true;
    const float far_candidate = (m22 * near_candidate) / (m22 - 1.f);
    if (far_candidate > near_candidate && far_candidate < 1.0e7f) {
      *far_z = far_candidate;
      *far_ok = true;
    }
  }
  return true;
}

bool unit_axis(float x, float y, float z) {
  const float length = x * x + y * y + z * z;
  return length > 0.90f && length < 1.10f;
}

bool perpendicular(float ax, float ay, float az, float bx, float by, float bz) {
  return std::fabs(ax * bx + ay * by + az * bz) < 0.08f;
}

float determinant(float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22) {
  return r00 * (r11 * r22 - r12 * r21) - r01 * (r10 * r22 - r12 * r20) + r02 * (r10 * r21 - r11 * r20);
}

bool rigid(const float* m, bool column_major, float* x, float* y, float* z) {
  const float r00 = at(m, 0, 0, column_major);
  const float r01 = at(m, 0, 1, column_major);
  const float r02 = at(m, 0, 2, column_major);
  const float r10 = at(m, 1, 0, column_major);
  const float r11 = at(m, 1, 1, column_major);
  const float r12 = at(m, 1, 2, column_major);
  const float r20 = at(m, 2, 0, column_major);
  const float r21 = at(m, 2, 1, column_major);
  const float r22 = at(m, 2, 2, column_major);
  if (!unit_axis(r00, r01, r02) || !unit_axis(r10, r11, r12) || !unit_axis(r20, r21, r22)) {
    return false;
  }
  if (!perpendicular(r00, r01, r02, r10, r11, r12) || !perpendicular(r00, r01, r02, r20, r21, r22) || !perpendicular(r10, r11, r12, r20, r21, r22)) {
    return false;
  }
  const float det = determinant(r00, r01, r02, r10, r11, r12, r20, r21, r22);
  if (!std::isfinite(det) || std::fabs(std::fabs(det) - 1.f) > 0.08f) {
    return false;
  }
  *x = at(m, 3, 0, column_major);
  *y = at(m, 3, 1, column_major);
  *z = at(m, 3, 2, column_major);
  if (!finite4(*x, *y, *z, det)) {
    return false;
  }
  if (std::fabs(*x) > 1.0e7f || std::fabs(*y) > 1.0e7f || std::fabs(*z) > 1.0e7f) {
    return false;
  }
  if (std::fabs(*x) < 1e-3f && std::fabs(*y) < 1e-3f && std::fabs(*z) < 1e-3f) {
    return false;
  }
  return true;
}

void copy16(float* dst, const float* src) {
  std::memcpy(dst, src, sizeof(float) * 16);
}

}  // namespace

Match scan(const void* bytes, std::size_t size) {
  Match match;
  if (bytes == nullptr || size < 64) {
    return match;
  }
  const auto* data = static_cast<const std::uint8_t*>(bytes);
  const std::size_t windows = (size - 63) / 16;
  for (std::size_t window = 0; window < windows; ++window) {
    const float* floats = reinterpret_cast<const float*>(data + window * 16);
    float fov = 0;
    float near_z = 0;
    float far_z = 0;
    bool near_ok = false;
    bool far_ok = false;
    bool found_perspective = false;
    bool column_perspective = false;
    if (perspective(floats, false, &fov, &near_z, &far_z, &near_ok, &far_ok)) {
      found_perspective = true;
    } else if (perspective(floats, true, &fov, &near_z, &far_z, &near_ok, &far_ok)) {
      found_perspective = true;
      column_perspective = true;
    }
    if (found_perspective) {
      ++match.perspective_matches;
      if (match.perspective_matches == 1) {
        copy16(match.perspective, floats);
        match.perspective_offset = window * 16;
        match.perspective_layout = column_perspective ? Layout::ColumnMajor : Layout::RowMajor;
        match.fov_y = fov;
        match.near_ok = near_ok;
        match.far_ok = far_ok;
        match.near_z = near_z;
        match.far_z = far_z;
      }
    }

    float x = 0;
    float y = 0;
    float z = 0;
    bool found_rigid = false;
    bool column_rigid = false;
    if (rigid(floats, false, &x, &y, &z)) {
      found_rigid = true;
    } else if (rigid(floats, true, &x, &y, &z)) {
      found_rigid = true;
      column_rigid = true;
    }
    if (found_rigid) {
      ++match.rigid_matches;
      if (match.rigid_matches == 1) {
        copy16(match.rigid, floats);
        match.rigid_offset = window * 16;
        match.rigid_layout = column_rigid ? Layout::ColumnMajor : Layout::RowMajor;
        match.translation[0] = x;
        match.translation[1] = y;
        match.translation[2] = z;
      }
    }
  }

  const bool has_perspective = match.perspective_matches > 0;
  const bool has_rigid = match.rigid_matches > 0;
  if (has_perspective && has_rigid) {
    match.kind = Kind::CoLocated;
  } else if (has_perspective) {
    match.kind = Kind::Perspective;
  } else if (has_rigid) {
    match.kind = Kind::Rigid;
  }
  return match;
}

}  // namespace matrix_detector
