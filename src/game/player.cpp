#include "player.hpp"
#include "collision_records.hpp"

#include <windows.h>
#include <intrin.h>

#include <cmath>
#include <atomic>
#include <cstring>

namespace game {
namespace {

constexpr std::uintptr_t kRootRva = 0x9ADB058;
constexpr std::uint32_t kClientOff = 0x128;
constexpr std::uint32_t kHandleOff = 0x28;
constexpr std::uint32_t kEntityOff = 0x3E8;
constexpr std::uint32_t kCamHolderOff = 0x418;
constexpr std::uint32_t kCamObjOff = 0x48;
constexpr std::uint32_t kCamXformOff = 0x20;  // camera vtable +0x30 getter = lea rax,[rcx+0x20] ([LIVE] 2026-10-10)
constexpr std::uint32_t kBodyPosOff = 0x94;
constexpr std::uint32_t kCompOff = 0x1A8;
constexpr std::uint32_t kLookOff = 0x30;
constexpr std::uint32_t kLookYawOff = 0x34;
constexpr std::uint32_t kLookPitchOff = 0x38;  // -pitch
constexpr std::uint32_t kPoseOff = 0x500;
constexpr std::uint32_t kPoseCountOff = 0x8;
constexpr int kMaxBones = 512;
constexpr float kCell = 1024.f;

unsigned char* ptr_at(const unsigned char* p, std::uint32_t off) {
  unsigned char* v = nullptr;
  return p != nullptr && mem_read(p + off, &v, sizeof(v)) ? v : nullptr;
}

// Cell offset between the pose (cell-local) and the world: camera position minus bone 0 (the camera bone), snapped
// to whole 1024-unit cells.
bool cell_offset(const Pointers& p, float out[3]) {
  unsigned char* bones = ptr_at(p.pose, 0);
  float cam[16], b0[16];
  if (bones == nullptr || p.camera == nullptr || !mem_read(p.camera + kCamXformOff, cam, sizeof(cam)) ||
      !mem_read(bones, b0, sizeof(b0))) {
    return false;
  }
  for (int k = 0; k < 3; ++k) {
    out[k] = std::round((cam[12 + k] - b0[12 + k]) / kCell) * kCell;
  }
  return true;
}

}  // namespace

bool mem_read(const void* src, void* dst, std::size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

bool mem_write(void* dst, const void* src, std::size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

unsigned char* module_base() {
  static unsigned char* base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
  return base;
}

Pointers pointers() {
  Pointers p;
  unsigned char* root = ptr_at(module_base(), static_cast<std::uint32_t>(kRootRva));
  p.handle = ptr_at(ptr_at(root, kClientOff), kHandleOff);
  if (p.handle == nullptr) {
    return p;
  }
  p.entity = ptr_at(p.handle, kEntityOff);
  p.body = ptr_at(ptr_at(p.entity, 0x10), 0x28);
  p.holder = ptr_at(p.handle, kCamHolderOff);
  p.camera = ptr_at(p.holder, kCamObjOff);
  unsigned char* comp = ptr_at(p.body, kCompOff);
  p.look = ptr_at(comp, kLookOff);
  p.pose = ptr_at(comp, kPoseOff);
  return p;
}

// Keep the SEH frame free of C++ objects requiring unwinding.
bool query_thread_flag() {
  __try {
    const auto base = reinterpret_cast<std::uintptr_t>(module_base());
    const auto index = *reinterpret_cast<const std::uint32_t*>(base + 0x9C48740);
    if (index > 4095) return false;
    const auto slots = reinterpret_cast<const std::uintptr_t*>(__readgsqword(0x58));
    if (slots == nullptr || slots[index] == 0) return false;
    return *reinterpret_cast<const unsigned char*>(slots[index] + 0xEEC) != 0;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

CollisionContext collision_context() {
  CollisionContext out;
  const auto p = pointers();
  auto base = module_base();
  auto vt = ptr_at(p.body, 0);
  auto getter = ptr_at(vt, 0x108);
  const unsigned char expected[] = {0x48,0x8B,0x81,0xD8,0,0,0,0xC3};
  unsigned char actual[sizeof(expected)] = {};
  if (getter != base + 0x9630D0 || !mem_read(getter,actual,sizeof(actual)) ||
      std::memcmp(actual,expected,sizeof(actual)) != 0) return out;
  auto world = ptr_at(p.body,0xD8);
  auto owner = ptr_at(ptr_at(world,0xD0),0x18);
  auto query = ptr_at(owner,8);
  auto query_vt = ptr_at(query,0);
  auto ray = ptr_at(query_vt,0x10);
  out.world=world; out.owner=owner; out.query=query; out.ray=ray;
  out.known_interface=query_vt==base+0x5FBD350 && ray==base+0x2C7F630;
  out.thread_ready=out.known_interface && query_thread_flag();
  return out;
}

Player read() {
  Player out;
  const Pointers p = pointers();
  float pos[3];
  if (p.body == nullptr || p.camera == nullptr || !mem_read(p.body + kBodyPosOff, pos, sizeof(pos)) ||
      !mem_read(p.camera + kCamXformOff, out.cam, sizeof(out.cam))) {
    return out;
  }
  out.body = {pos[0], pos[1], pos[2]};
  out.valid = !(out.cam[0] == 1.f && out.cam[5] == 1.f && out.cam[10] == 1.f && out.cam[12] == 0.f);  // not identity
  float look[2];
  if (p.look != nullptr && mem_read(p.look + kLookYawOff, look, sizeof(look))) {
    out.look_ok = true;
    out.yaw = look[0];
    out.pitch = -look[1];
  }
  return out;
}

int bone_count(const Pointers& p) {
  std::int32_t n = 0;
  return p.pose != nullptr && mem_read(p.pose + kPoseCountOff, &n, sizeof(n)) && n > 0 && n <= kMaxBones ? n : 0;
}

bool bone_world(const Pointers& p, int index, float out[16]) {
  unsigned char* bones = ptr_at(p.pose, 0);
  float cell[3];
  if (bones == nullptr || index < 0 || index >= bone_count(p) || !cell_offset(p, cell) ||
      !mem_read(bones + 0x40 * index, out, sizeof(float) * 16)) {
    return false;
  }
  for (int k = 0; k < 3; ++k) {
    out[12 + k] += cell[k];
  }
  return true;
}

bool set_bone_world(const Pointers& p, int index, const float m[16]) {
  if (m == nullptr) return false;
  for (int i = 0; i < 16; ++i) {
    if (!std::isfinite(m[i])) return false;
  }
  unsigned char* bones = ptr_at(p.pose, 0);
  float cell[3];
  if (bones == nullptr || index < 0 || index >= bone_count(p) || !cell_offset(p, cell)) {
    return false;
  }
  float local[16];
  std::memcpy(local, m, sizeof(local));
  for (int k = 0; k < 3; ++k) {
    local[12 + k] -= cell[k];
  }
  return mem_write(bones + 0x40 * index, local, sizeof(local));
}

bool set_look(const Pointers& p, float yaw, float pitch) {
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) return false;
  const float v[2] = {yaw, -pitch};
  return p.look != nullptr && mem_write(p.look + kLookYawOff, v, sizeof(v));
}

namespace {
constexpr std::uintptr_t kCollisionRayRva=0x2C7F630;
constexpr unsigned char kCollisionRaySig[]={0x48,0x89,0x6C,0x24,0x18,0x56,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x40,0x4C};
constexpr std::uintptr_t kHitClearRva=0x33F5DB0;
constexpr unsigned char kHitClearSig[]={0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x33,0xFF,0x48,0x8B,0xD9,0x48,0x89,0x39,0x48,0x89,0x79,0x08,0x48,0x89,0x79,0x10,0x48,0x8B,0x49};
constexpr std::uintptr_t kHitsClearRva=0x1833E0;
constexpr unsigned char kHitsClearSig[]={0x40,0x56,0x48,0x83,0xEC,0x20,0x48,0x8B,0xF1,0x48,0x89,0x5C,0x24,0x30,0x48,0x8B,0x09,0x8B,0x46,0x08,0x83,0xE8,0x01,0x48,0x63,0xD8,0x78,0x33,0x48,0x89,0x7C,0x24,0x38,0x48,0x8D,0x79,0x18};
std::atomic<bool> g_query_faulted{false};
bool collision_guard(std::uintptr_t rva,const unsigned char* expected,std::size_t length) {
  unsigned char actual[48]={};
  return length<=sizeof(actual) && mem_read(module_base()+rva,actual,length) && std::memcmp(actual,expected,length)==0;
}
// SEH leaf: engine-owned references are released through the original cleanup methods.
bool cast_and_cleanup(const void* query,const float* start,const float* end,
                      collision::ResultHeader* result,collision::Hit* hit,bool* did_hit) {
  using Cast=unsigned char(__fastcall*)(const void*,const float*,const float*,const void*,collision::ResultHeader*);
  using Clear=void(__fastcall*)(void*);
  const auto base=module_base();
  __try {
    reinterpret_cast<Cast>(base+kCollisionRayRva)(query,start,end,base+0x8E07310,result);
    *did_hit=result->hit!=0;
    *hit=result->nearest; // return values only; no engine reference escapes this function
    reinterpret_cast<Clear>(base+kHitClearRva)(&result->nearest);
    reinterpret_cast<Clear>(base+kHitsClearRva)(&result->hits);
    return true;
  } __except(EXCEPTION_EXECUTE_HANDLER) {
    return false; // caller permanently disables further queries for this process
  }
}
}

RayResult raycast_camera_filter(const Vec3& start,const Vec3& end) {
  RayResult out;
  if (g_query_faulted.load()) return out;
  const float a[]={start.x,start.y,start.z}, b[]={end.x,end.y,end.z};
  float length2=0.f;
  for (int i=0;i<3;++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i])) return out;
    length2+=(a[i]-b[i])*(a[i]-b[i]);
  }
  if (!std::isfinite(length2) || length2<1e-8f || length2>10000.f) return out;
  auto context=collision_context();
  if (!context.thread_ready || !collision_guard(kCollisionRayRva,kCollisionRaySig,sizeof(kCollisionRaySig)) ||
      !collision_guard(kHitClearRva,kHitClearSig,sizeof(kHitClearSig)) ||
      !collision_guard(kHitsClearRva,kHitsClearSig,sizeof(kHitsClearSig))) return out;
  std::uint32_t filter[4];
  if (!mem_read(module_base()+0x8E07310,filter,sizeof(filter))) return out;
  collision::InlineResult storage;
  collision::Hit hit{};
  if (!cast_and_cleanup(context.query,a,b,&storage.header,&hit,&out.hit)) {
    g_query_faulted.store(true); return out;
  }
  out.executed=true;
  if (out.hit) {
    for (int i=0;i<3;++i) if (!std::isfinite(hit.position[i]) || !std::isfinite(hit.normal[i])) { out.hit=false; return out; }
    out.position={hit.position[0],hit.position[1],hit.position[2]};
    out.normal={hit.normal[0],hit.normal[1],hit.normal[2]};
    out.distance=hit.distance;
  }
  return out;
}

}  // namespace game
