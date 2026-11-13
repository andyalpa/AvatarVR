#include "../../src/game/collision_records.hpp"
#include <cassert>
#include <type_traits>

int main() {
  game::collision::InlineResult first, second;
  assert(first.header.hit == 0);
  assert(first.header.count == 0);
  assert(first.header.nearest.entity_reference == 0);
  assert(first.header.hits == first.storage);
  assert(second.header.hits == second.storage);
  assert(first.header.hits != second.header.hits);
  assert(first.header.capacity_flags == 0x80000004);
  assert(first.header.inline_capacity == 4);
  static_assert(!std::is_copy_constructible_v<game::collision::InlineResult>);
  static_assert(!std::is_move_constructible_v<game::collision::InlineResult>);
}
