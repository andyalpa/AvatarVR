#pragma once

#include <cstddef>
#include <cstdint>

namespace matrix_detector {

enum class Layout {
  RowMajor,
  ColumnMajor,
};

enum class Kind {
  None,
  Perspective,
  Rigid,
  CoLocated,
};

struct Match {
  Kind kind = Kind::None;
  Layout perspective_layout = Layout::RowMajor;
  Layout rigid_layout = Layout::RowMajor;
  float perspective[16] = {};
  float rigid[16] = {};
  std::size_t perspective_offset = 0;
  std::size_t rigid_offset = 0;
  float fov_y = 0;
  bool near_ok = false;
  bool far_ok = false;
  float near_z = 0;
  float far_z = 0;
  float translation[3] = {};
  int perspective_matches = 0;
  int rigid_matches = 0;
};

Match scan(const void* bytes, std::size_t size);

}
