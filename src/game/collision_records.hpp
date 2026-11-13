#pragma once

#include <cstddef>
#include <cstdint>

// STATIC reconstruction for retail 98588DA2, NOT a callable physics API.
// Query/reset/merge paths: 142C7F630, 142C7F800, 1433F5E00.
// Unknown fields retain neutral names. Entity references are engine-owned:
// never free/release them through C++ delete or a fabricated COM interface.
namespace game::collision {

struct Hit {
  float position[3];                 // +00
  float normal[3];                   // +0C
  std::uintptr_t entity_reference;   // +18, engine refcounted
  float distance;                    // +20
  std::uint32_t unknown24;
  std::uint64_t unknown28;
  std::uint64_t unknown30;
};

struct ResultHeader {
  std::uint8_t hit;
  std::uint8_t padding01[7];
  Hit nearest;                      // +08
  Hit* hits;                        // +40
  std::uint32_t count;               // +48
  std::uint32_t capacity_flags;      // +4C
  std::uint32_t inline_capacity;     // +50
  std::uint32_t padding54;
};

static_assert(sizeof(Hit) == 0x38);
static_assert(offsetof(Hit, entity_reference) == 0x18);
static_assert(offsetof(Hit, distance) == 0x20);
static_assert(offsetof(ResultHeader, nearest) == 8);
static_assert(offsetof(ResultHeader, hits) == 0x40);
static_assert(offsetof(ResultHeader, count) == 0x48);
static_assert(offsetof(ResultHeader, capacity_flags) == 0x4C);
static_assert(sizeof(ResultHeader) == 0x58);

// Mirrors the observed camera caller's four-element inline storage. This wrapper
// only prepares storage; a future query owner must call the engine cleanup before
// its lifetime ends, including on no-hit paths. No calls are made here.
struct InlineResult {
  ResultHeader header{};
  Hit storage[4]{};

  InlineResult() {
    header.hits = storage;
    header.capacity_flags = 0x80000004;
    header.inline_capacity = 4;
  }
  InlineResult(const InlineResult&) = delete;
  InlineResult& operator=(const InlineResult&) = delete;
  InlineResult(InlineResult&&) = delete;
  InlineResult& operator=(InlineResult&&) = delete;
};

static_assert(offsetof(InlineResult, storage) == 0x58);
}  // namespace game::collision
