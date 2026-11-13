#pragma once

#include <cstddef>
#include <cstdint>

// Local player, game camera, look angles and first-person pose (research/CAMERA_FINDINGS.md "Local player and camera
// roots", "Look and pose"; research/WEAPONS_AND_HANDS.md "First-person bone map"; [LIVE] on build 98588DA2...).
// Every read is SEH-guarded: the roots sit in .xtext and the objects come and go with loads and menus. Writes go
// through the same guard and are the caller's decision (lua_host gates them behind the menu's "Script writes").
// World positions are large-world metres, +y up; camera rows: 0 right, 1 up, 2 forward, 3 position.
namespace game {

struct Vec3 {
  float x = 0.f, y = 0.f, z = 0.f;
};

struct Pointers {
  unsigned char* handle = nullptr;    // *(*(*(afop+0x9ADB058)+0x128)+0x28)
  unsigned char* entity = nullptr;    // handle+0x3E8
  unsigned char* body = nullptr;      // entity+0x10 -> +0x28 (position at +0x94)
  unsigned char* holder = nullptr;    // handle+0x418: camera manager ("this" of 0x144271330)
  unsigned char* camera = nullptr;    // holder+0x48: player camera (transform at +0x20)
  unsigned char* look = nullptr;      // body+0x1A8 -> +0x30 (yaw +0x34, -pitch +0x38)
  unsigned char* pose = nullptr;      // body+0x1A8 -> +0x500: first-person pose (null outside gameplay)
};

struct Player {
  bool valid = false;  // body and camera readable (gameplay)
  Vec3 body;           // feet / root
  float cam[16] = {};  // camera transform rows (4 x 4 floats)
  bool look_ok = false;
  float yaw = 0.f;     // radians, = atan2(fwd.x, fwd.z)
  float pitch = 0.f;   // radians, + up (stored negated in the game)
};

// Raw SEH-guarded memory access (false on a fault). Plain-C frames inside.
bool mem_read(const void* src, void* dst, std::size_t n);
bool mem_write(void* dst, const void* src, std::size_t n);

unsigned char* module_base();
Pointers pointers();
Player read();

struct CollisionContext {
  const void* world = nullptr;
  const void* owner = nullptr;
  const void* query = nullptr;
  const void* ray = nullptr;
  bool known_interface = false;
  bool thread_ready = false;  // current thread's engine TLS flag; not proof of all lock requirements
};
// Read-only, no virtual calls. Live owner chain confirmed 2026-10-13.
CollisionContext collision_context();
struct RayResult {
  bool executed = false;
  bool hit = false;
  Vec3 position, normal;
  float distance = 0.f;
};
// Experimental read query, using the engine's existing camera filter, not a final weapon filter.
RayResult raycast_camera_filter(const Vec3& start, const Vec3& end);

// First-person pose: bone count, and one bone's world matrix (rows; cell offset added from the camera bone).
int bone_count(const Pointers& p);
bool bone_world(const Pointers& p, int index, float out[16]);
bool set_bone_world(const Pointers& p, int index, const float m[16]);

// Look angles in the camera convention (pitch + up). The game re-reads them in the camera manager update.
bool set_look(const Pointers& p, float yaw, float pitch);

}  // namespace game
