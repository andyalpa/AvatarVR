#include "../runtime_log.hpp"
#include "mount_camera.hpp"
#include "mount_camera_capture.hpp"
#include "mount_anchor.hpp"
#include "../diagnostics/rider_ik.hpp"
#include "native_stereo.hpp"
#include "../game/player.hpp"
#include "../game/mount_state.hpp"
#include "../vr/vr_arms.hpp"
#include "../vr/vr_weapon.hpp"
#include "../vr/xr_runtime.hpp"
#include "../vr/xr_input.hpp"
#include "../diagnostics/skin_palette_probe.hpp"

#include <windows.h>
#include <Xinput.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cwchar>
#include <mutex>
#include <string>
#include <vector>
#include <utility>
#include "../settings_file.hpp"

namespace mount_camera {
namespace {
// STATIC leads, not mount write anchors. See CAMERA_FINDINGS.md "FEATURE-06
// mount ownership" and research/sdk/98588da2/mount_camera_static_20261023.txt.
constexpr std::uintptr_t kOnMountRva = 0x3A52D90;
constexpr unsigned char kOnMountSig[] = {
  0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x48,0x8B,0x5A,0x20,0x8B,0x43,
  0x1C,0x25,0xFF,0xFF,0x00,0x10,0x83,0xF8,0x01,0x74,0x15,0xBA,0x01,0x00,0x00,0x00,
  0x48,0x8B,0xCB,0xE8,0x08,0xCB,0x63,0xFC};
constexpr unsigned kQueueLimit = 512;

bool first_person = true;  // menu toggle (Present/menu thread); mirrored into fp_enabled for the game thread
bool arms_menu = true, saved_arms = true;  // stage B menu toggle (Present/menu thread)
std::atomic<bool> arms_enabled{true};
// Harness recorder (game thread writes under harness_mutex; Present thread logs/saves).
std::mutex harness_mutex;
rider_ik::GripStat harness[2];
float harness_prev[2][3]{};
bool harness_prev_ok = false;
bool harness_was_riding = false;
std::atomic<bool> harness_report{false};
// Harness grab (game thread state; atomics for the palette and input threads).
rider_ik::Grab grab[2];
std::atomic<bool> grab_near[2], grab_held[2];
std::atomic<float> grab_weight[2] = {1.f, 1.f};
std::atomic<float> grab_distance[2];
std::atomic<int> grab_events{0};  // + grabs, logged by the Present thread
struct GrabLog { int hand, edge; float distance; bool learned; };
GrabLog grab_log[8]; std::atomic<unsigned> grab_log_n{0};
std::atomic<bool> fp_enabled{true};
// [HEADSET 2026-11-13] riding with a bow, holding the trigger to tense makes the arms jiggle as riding did before. [LIVE
// RIDER_TRACE_36848500] in bursts the eye (C_head of the 192 pose + calibrated eye) moved unevenly in world (off-trend per step p50 2.2,
// p90 9.4 cm; steps 13-25 cm) while the body (+0x94) stayed even (0.5 cm; 15-20 cm); calm: eye 0.4/0.7 cm. The hands are built on this
// eye, the meshes on the body ticks, so the arms jiggled. INFERENCE: the pose is sometimes a tick older than the body read. The body is
// read in the pose's own pelvis frame (bone 6): that offset is near constant while riding and jumps when the pose lags; its median of 5
// steps, smoothed as on foot smooths camera sway (0.25 s, height 0.10 s), shifts the eye by (body - pelvis frame at the smoothed offset).
// Head-to-pelvis and the pelvis turn come from one pose, so tilt is not lagged. eye_pose_steady=0 in AvatarVR_mounts.txt turns it off.
std::atomic<bool> eye_pose_steady{true};
std::atomic<unsigned> eye_steady_mm{0},eye_steady_steps{0};
mount_anchor::Anchor anchor;  // game thread (head step)
long long anchor_us = 0;      // game thread
std::atomic<bool> anchor_on{false};
std::atomic<unsigned> anchor_rejects{0};
std::atomic<float> anchor_len{0.f};
std::atomic<int> anchor_source{0};  // 0 first-person bone 0 (fallback), 1 C_head joint, 2 C_head + calibrated eye
mount_anchor::EyeCalib eye_calib;   // game thread
std::atomic<bool> calib_ready{false};
std::atomic<float> calib_now[3];  // eye offset in head axes, for other threads (eye_point)
std::atomic<float> calib_logged[3];
bool loaded = false, saved_first = true;
bool auto_capture = false;
std::wstring dir;
std::atomic<bool> requested{false}, cancelled{false}, observing{false};
std::atomic<std::uint64_t> present{0}, camera_step{0};
std::atomic<unsigned> dropped{0}, fp_palettes{0};
std::atomic<ULONGLONG> last_fp{0};
std::mutex mutex;
capture::Budget budget;
capture::WeaponBudget weapon_budget;
struct Row { std::string text; };
std::vector<Row> pending;
struct Mesh { const void* mesh; const void* root; const void* resource; ULONGLONG next; bool fp; bool weapon; };
std::vector<Mesh> meshes;
std::vector<Mesh> weapon_meshes;
struct PaletteResource { const void* identity=nullptr; char path[192]{}; bool fp=false,weapon=false; };
std::vector<PaletteResource> palette_resources; // classify a resource once, even after general mesh saturation
HANDLE output = INVALID_HANDLE_VALUE;
bool waiting = false;
unsigned capture_id = 0;
ULONGLONG next_poll = 0;

template<class T> bool read(const void* p, std::size_t off, T& out) {
  return p && game::mem_read(static_cast<const unsigned char*>(p) + off, &out, sizeof(out));
}
const unsigned char* ptr(const void* p, std::size_t off) {
  const unsigned char* value = nullptr;
  read(p, off, value);
  return value;
}
std::uintptr_t rva(const void* p) {
  const auto address = reinterpret_cast<std::uintptr_t>(p);
  const auto base = reinterpret_cast<std::uintptr_t>(game::module_base());
  // The image range comes from the loaded PE, never from an assumed address span.
  IMAGE_DOS_HEADER dos{}; IMAGE_NT_HEADERS64 nt{};
  if (!read(game::module_base(), 0, dos) || dos.e_magic != IMAGE_DOS_SIGNATURE ||
      dos.e_lfanew <= 0 || dos.e_lfanew > 0x1000 ||
      !read(game::module_base(), static_cast<unsigned>(dos.e_lfanew), nt) || nt.Signature != IMAGE_NT_SIGNATURE)
    return 0;
  return address >= base && address - base < nt.OptionalHeader.SizeOfImage ? address - base : 0;
}
bool readable_code(const void* p, std::size_t n) {
  const auto offset = rva(p);
  if (!offset) return false;
  IMAGE_DOS_HEADER dos{}; IMAGE_NT_HEADERS64 nt{};
  if (!read(game::module_base(),0,dos) || !read(game::module_base(),static_cast<unsigned>(dos.e_lfanew),nt) ||
      nt.FileHeader.NumberOfSections > 96) return false;
  const auto section_start = static_cast<unsigned>(dos.e_lfanew) + sizeof(DWORD) + sizeof(IMAGE_FILE_HEADER) + nt.FileHeader.SizeOfOptionalHeader;
  for (unsigned i=0;i<nt.FileHeader.NumberOfSections;++i) {
    IMAGE_SECTION_HEADER section{};
    if (!read(game::module_base(),section_start+i*sizeof(section),section)) return false;
    const bool allowed = std::memcmp(section.Name,".text\0\0\0",8)==0 || std::memcmp(section.Name,".xpdata\0",8)==0;
    if (allowed && offset >= section.VirtualAddress && offset-section.VirtualAddress <= section.Misc.VirtualSize &&
        n <= section.Misc.VirtualSize-(offset-section.VirtualAddress)) return true;
  }
  return false;  // unknown/protected sections stay unread and uncalled
}
long long microseconds() {
  LARGE_INTEGER now{}, frequency{};
  QueryPerformanceCounter(&now); QueryPerformanceFrequency(&frequency);
  return now.QuadPart / frequency.QuadPart * 1000000 +
         now.QuadPart % frequency.QuadPart * 1000000 / frequency.QuadPart;
}
bool write(HANDLE file, const std::string& s) {
  DWORD n = 0;
  return file != INVALID_HANDLE_VALUE && WriteFile(file, s.data(), static_cast<DWORD>(s.size()), &n, nullptr) && n == s.size();
}
void log(const std::string& s) {
  runtime_log::append(dir + L"\\MOUNT_CAMERA.log", {s});
}
void queue(std::string s) {  // mutex held; never write files on an engine hook thread
  if (pending.size() < kQueueLimit) pending.push_back({std::move(s)});
  else dropped.fetch_add(1, std::memory_order_relaxed);
}
void append_matrix(std::string& out, const char* name, bool valid, const float* m) {
  char b[512];
  std::snprintf(b, sizeof(b), " %s=%d[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
    name, valid, m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8],m[9],m[10],m[11],m[12],m[13],m[14],m[15]);
  out += b;
}
void append_pose(std::string& out, const char* name, const void* pose) {
  int count = -1, camera_bone = -1;
  short cell[3]{};
  float m[16]{};
  const auto data = ptr(pose, 0);
  // Cell comes from the pose itself (+74), as consumed by mode 4. Do not
  // infer it by rounding a third-person chase camera against FP bone zero.
  const bool header = read(pose, 8, count) && read(pose, 0x7C, camera_bone) && read(pose, 0x74, cell);
  const bool valid = header && count > 0 && count <= 1024 && camera_bone >= 0 && camera_bone < count &&
                     read(data, static_cast<std::size_t>(camera_bone) * 64, m);
  char b[192];
  std::snprintf(b, sizeof(b), " %s_ptr=%p data=%p count=%d cam_bone=%d cell=%d,%d,%d",
                name, pose, data, count, camera_bone, cell[0],cell[1],cell[2]);
  out += b;
  append_matrix(out, name, valid, m);  // raw cell-local matrix; validity is explicit
}
// Third-person pose (component +0x4F8, 192 bones, WEAPONS_AND_HANDS.md "LIVE named skeletons"): C_head = 15. World
// matrix = cell-local matrix + 1024 * the pose's own cell (+0x74), as for the first-person pose.
constexpr int kTpHead = 15;
bool third_person_head(const game::Pointers& p, float* out16) {
  const auto pose = ptr(ptr(p.body, 0x1A8), 0x4F8);
  int count = 0;
  short cell[3]{};
  if (!read(pose, 8, count) || count != 192 || !read(pose, 0x74, cell) ||
      !read(ptr(pose, 0), static_cast<std::size_t>(kTpHead) * 64, *reinterpret_cast<float(*)[16]>(out16)))
    return false;
  for (int k = 0; k < 3; ++k) out16[12 + k] += 1024.f * cell[k];
  for (int i = 0; i < 16; ++i) if (!std::isfinite(out16[i])) return false;
  return true;
}
// One 192-pose bone, world (cell offset added).
bool tp_bone(const game::Pointers& p, int index, float* out16) {
  const auto pose = ptr(ptr(p.body, 0x1A8), 0x4F8);
  int count = 0;
  short cell[3]{};
  if (index < 0 || index >= 192 || !read(pose, 8, count) || count != 192 || !read(pose, 0x74, cell) ||
      !game::mem_read(ptr(pose, 0) + static_cast<std::size_t>(index) * 64, out16, 64))
    return false;
  for (int k = 0; k < 3; ++k) out16[12 + k] += 1024.f * cell[k];
  for (int i = 0; i < 16; ++i) if (!std::isfinite(out16[i])) return false;
  return true;
}
// Same pose header/cell for both sides. This is an observation, not an atomic engine pose snapshot.
void append_socket_pair(std::string& line,const char* label,const void* pose,int expected,int left,int right) {
  int count=0; short cell[3]{}; float m[2][16]{};
  const auto data=ptr(pose,0);
  const bool header=data && read(pose,8,count) && count==expected && read(pose,0x74,cell);
  bool valid[2]{}; const int index[2]={left,right};
  for(int h=0;h<2;++h) {
    valid[h]=header && read(data,index[h]*64,m[h]);
    if(valid[h]) for(int k=0;k<3;++k) m[h][12+k]+=1024.f*cell[k];
    for(float v:m[h]) valid[h]=valid[h] && std::isfinite(v);
  }
  short after_cell[3]{};
  const bool same=data==ptr(pose,0) && read(pose,8,count) && count==expected &&
    read(pose,0x74,after_cell) && std::memcmp(cell,after_cell,sizeof(cell))==0;
  char name[64];
  for(int h=0;h<2;++h) {
    std::snprintf(name,sizeof(name),"%s_%c_world",label,h?'R':'L');
    append_matrix(line,name,valid[h] && same,m[h]);
  }
}
bool weapon_identity(const game::Pointers& p,const mount_state::State& state) { // capture mutex held
  if(!p.body) return false;
  weapon_budget.identity(reinterpret_cast<std::uintptr_t>(p.body),reinterpret_cast<std::uintptr_t>(state.partner),
                         state.partner_id[0],state.partner_id[1]);
  return true;
}
bool append_weapon_context(std::string& line,const game::Pointers& p,const mount_state::State& state,ULONGLONG now,bool landmarks=false) {
  native_stereo::GripFrame frame{};
  const bool grips=native_stereo::grip_frame(&frame);
  const auto ws=skin_palette_probe::weapon_capture_state();
  const auto equipped=ptr(p.body,0x250);
  // Existing weapon component source: *(weapon+3FA0+C8), vr_weapon.cpp / SDK weapon entity matrix receipt.
  const auto entity=ptr(equipped,0x3FA0+0xC8);
  char b[1200];
  std::snprintf(b,sizeof(b)," player=%p equipped_candidate=%p equipped_entity=%p mount=%p mount_id=%llX:%llX phase=%d "
    "mount_age_ms=%llu hand=%d seq=%llu grips=%d fp_age_ms=%llu support_age_us=%llu measure_age_us=%llu "
    "support_valid=%d two_held=%d support_hand=%d support_only=%d swap_to=%d harness=%d,%d near=%d,%d "
    "support_dist=%.4f,%.4f handle_dist=%.4f,%.4f",
    p.body,equipped,entity,state.partner,static_cast<unsigned long long>(state.partner_id[0]),
    static_cast<unsigned long long>(state.partner_id[1]),static_cast<int>(state.phase),
    static_cast<unsigned long long>(capture::age(now,state.updated_ms)),skin_palette_probe::weapon_hand_index(),
    static_cast<unsigned long long>(frame.sequence),grips,
    static_cast<unsigned long long>(ws.fp_age_ms),static_cast<unsigned long long>(ws.support_age_us),
    static_cast<unsigned long long>(ws.measure_age_us),ws.support_valid,ws.two_hand_held,ws.support_hand,ws.support_only,ws.swap_target,
    harness_held(0),harness_held(1),harness_near(0),harness_near(1),
    ws.support_distance[0],ws.support_distance[1],ws.handle_distance[0],ws.handle_distance[1]);
  line+=b;
  char mounted[1408];
  std::snprintf(mounted,sizeof(mounted)," mounted_owner=%p mounted_candidate=%p mounted_resource=%p mounted_object=%p "
    "mounted_bound=%d mounted_committed=%d mounted_source=%u mounted_hand=%u mounted_bind_age_ms=%llu mounted_commit_age_ms=%llu "
    "mounted_frame_seq=%llu mounted_commit_seq=%llu mounted_grip_profile=%d mounted_support_profile=%d mounted_pin=%.3f,%.3f "
    "mounted_onfoot_hold=%d mounted_finger_profile=%d,%d mounted_finger_apps=%u mounted_finger_rejects=%u mounted_pin_expired=%u mounted_pin_distant=%u "
    "mounted_profiles=%u mounted_disk_profiles=%u mounted_aim_profile=%d mounted_commit_failures=%u "
    "mounted_palette_adopted=%u mounted_palette_unbound=%u mounted_palette_not_ready=%u mounted_palette_invalid=%u",
    ws.mounted_owner,ws.mounted_candidate,ws.mounted_resource,ws.mounted_object,ws.mounted_bound,ws.mounted_committed,
    ws.mounted_source,ws.mounted_hand,static_cast<unsigned long long>(ws.mounted_bind_age_ms),
    static_cast<unsigned long long>(ws.mounted_commit_age_ms),static_cast<unsigned long long>(ws.mounted_frame_sequence),
    static_cast<unsigned long long>(ws.mounted_commit_sequence),ws.mounted_grip_profile,ws.mounted_support_profile,
    ws.mounted_pin[0],ws.mounted_pin[1],ws.mounted_onfoot_hold,ws.mounted_finger_profile[0],ws.mounted_finger_profile[1],
    ws.mounted_finger_apps,ws.mounted_finger_rejects,ws.mounted_pin_expired,ws.mounted_pin_distant,
    ws.mounted_profiles,ws.mounted_disk_profiles,ws.mounted_aim_profile,ws.mounted_commit_failures,
    ws.mounted_palette_adopted,ws.mounted_palette_unbound,ws.mounted_palette_not_ready,ws.mounted_palette_invalid);
  line+=mounted;
  xr_input::PadState pad{}; const bool controls=xr_input::controls(&pad);
  char input[160]; std::snprintf(input,sizeof(input)," controls_valid=%d controls_buttons=%u controls_trigger=%.4f,%.4f",
    controls,static_cast<unsigned>(pad.buttons),pad.lt,pad.rt); line+=input;
  append_matrix(line,"mounted_native_source_world",ws.mounted_bound,ws.mounted_flat);
  append_matrix(line,"mounted_committed_socket_world",ws.mounted_committed,ws.mounted_target);
  append_matrix(line,"mounted_committed_entity_world",ws.mounted_committed,ws.mounted_placed);
  append_matrix(line,"grip_L_world",grips && frame.valid[0],frame.grip[0]);
  append_matrix(line,"grip_R_world",grips && frame.valid[1],frame.grip[1]);
  append_matrix(line,"grip_camera_world",grips,frame.camera);
  const auto comp=ptr(p.body,0x1A8);
  append_socket_pair(line,"fp_socket",ptr(comp,0x500),90,42,79);
  append_socket_pair(line,"tp_socket",ptr(comp,0x4F8),192,62,100);
  append_socket_pair(line,"tp_wrist",ptr(comp,0x4F8),192,39,77);
  if(landmarks) { // only periodic WC rows, not every entity/palette row
    append_matrix(line,"fp_cached_draw_L_world",ws.fp_draw_valid,ws.fp_draw[0]);
    append_matrix(line,"fp_cached_draw_R_world",ws.fp_draw_valid,ws.fp_draw[1]);
    append_socket_pair(line,"fp_wrist",ptr(comp,0x500),90,19,56);
    append_socket_pair(line,"fp_index01",ptr(comp,0x500),90,26,63);
    append_socket_pair(line,"fp_middle01",ptr(comp,0x500),90,31,68);
    append_socket_pair(line,"fp_pinky01",ptr(comp,0x500),90,36,73);
    append_socket_pair(line,"tp_index01",ptr(comp,0x4F8),192,46,84);
    append_socket_pair(line,"tp_middle01",ptr(comp,0x4F8),192,51,89);
    append_socket_pair(line,"tp_pinky01",ptr(comp,0x4F8),192,56,94);
  }
  append_matrix(line,"support_wrist_in_socket",ws.support_valid,ws.support);
  append_matrix(line,"dominant_wrist_in_socket",ws.support_valid,ws.dominant);
  const auto after=mount_state::get();
  return game::pointers().body==p.body && after.partner==state.partner &&
    after.partner_id[0]==state.partner_id[0] && after.partner_id[1]==state.partner_id[1] &&
    ptr(p.body,0x250)==equipped;
}
// Game thread, each head step: while riding, add steady samples of the game's own palm placement (both hands within
// 1 cm of the previous step in the pelvis frame). With a hands-on-harness riding animation this is the harness grip.
void record_harness(const game::Pointers& p, bool riding) {
  if (!riding) {
    harness_prev_ok = false;
    if (harness_was_riding) harness_report.store(true);
    harness_was_riding = false;
    return;
  }
  harness_was_riding = true;
  float pelvis[16], palm[16], hand[16];
  float local[2][3], fwd[2][3], up[2][3];
  if (!tp_bone(p, rider_ik::kPelvis, pelvis)) { harness_prev_ok = false; return; }
  for (int h = 0; h < 2; ++h) {
    if (!tp_bone(p, rider_ik::kPalm[h], palm) || !tp_bone(p, rider_ik::kArm[h].hand, hand) ||
        !rider_ik::to_pelvis(pelvis, palm + 12, local[h]) || !rider_ik::dir_to_pelvis(pelvis, hand + 8, fwd[h]) ||
        !rider_ik::dir_to_pelvis(pelvis, hand + 4, up[h])) { harness_prev_ok = false; return; }
  }
  bool steady = harness_prev_ok;
  for (int h = 0; h < 2 && steady; ++h) {
    float d2 = 0.f;
    for (int k = 0; k < 3; ++k) d2 += (local[h][k] - harness_prev[h][k]) * (local[h][k] - harness_prev[h][k]);
    steady = d2 < 0.01f * 0.01f;
  }
  std::memcpy(harness_prev, local, sizeof(local));
  harness_prev_ok = true;
  if (!steady) return;
  std::lock_guard<std::mutex> lock(harness_mutex);
  for (int h = 0; h < 2; ++h) harness[h].add(local[h], fwd[h], up[h]);
}
// Game thread, each head step: grab / release the harness. The harness point is the learned grip (pelvis frame, once
// about a second of steady samples with a spread under 5 cm), else the game's palm right now (with a hands-on-harness
// riding animation that is the harness). A grip press within rider_ik::kGrabRadius of it grabs; release lets go.
void update_grab(const game::Pointers& p, bool allowed, float dt) {
  xr_input::PadState c{};
  const bool pad = xr_input::controls(&c);
  float pelvis[16];
  const bool have_pelvis = allowed && tp_bone(p, rider_ik::kPelvis, pelvis);
  for (int h = 0; h < 2; ++h) {
    bool close = false, learned = false;
    float distance = -1.f;
    float harness_point[3]{}, grip[16];
    if (have_pelvis) {
      float local[3], spread = 1.f;
      if (harness_grip(h, local, &spread) && spread < 0.05f) {
        mount_anchor::to_world(pelvis, local, harness_point);  // pelvis rows: right, up, forward
        learned = true;
      } else {
        float palm[16];
        if (tp_bone(p, rider_ik::kPalm[h], palm)) for (int k = 0; k < 3; ++k) harness_point[k] = palm[12 + k];
        else distance = -2.f;
      }
      if (distance > -2.f && native_stereo::grip_transform(h, grip)) {
        float d2 = 0.f;
        for (int k = 0; k < 3; ++k) d2 += (grip[12 + k] - harness_point[k]) * (grip[12 + k] - harness_point[k]);
        distance = std::sqrt(d2);
        close = distance < rider_ik::kGrabRadius;
      }
    }
    const bool pressed = pad && (c.buttons & (h ? XINPUT_GAMEPAD_RIGHT_SHOULDER : XINPUT_GAMEPAD_LEFT_SHOULDER)) != 0;
    int edge = 0;
    grab[h].step(pressed, close, allowed, dt, edge);
    grab_near[h].store(close, std::memory_order_relaxed);
    grab_held[h].store(grab[h].held, std::memory_order_relaxed);
    grab_weight[h].store(grab[h].ik_weight(), std::memory_order_relaxed);
    grab_distance[h].store(distance, std::memory_order_relaxed);
    if (edge) {
      xr_input::pulse(h, edge > 0 ? 0.05f : 0.03f, edge > 0 ? 0.6f : 0.3f);
      const unsigned i = grab_log_n.load(std::memory_order_relaxed);
      if (i < 8) { grab_log[i] = {h, edge, distance, learned}; grab_log_n.store(i + 1, std::memory_order_release); }
    }
  }
}
bool guards_match() {
  unsigned char got[sizeof(kOnMountSig)]{};
  return game::mem_read(game::module_base() + kOnMountRva, got, sizeof(got)) &&
         std::memcmp(got, kOnMountSig, sizeof(got)) == 0;
}
void load() {
  wchar_t path[MAX_PATH]{}; HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                         reinterpret_cast<LPCWSTR>(&on_present), &self) || !GetModuleFileNameW(self, path, MAX_PATH)) return;
  dir = path;
  const auto slash = dir.find_last_of(L"\\/");
  if (slash == dir.npos) { dir.clear(); return; }
  dir.resize(slash);
  loaded = true;
  char text[1024]{}; DWORD n = 0;
  settings_file::read(dir + L"\\AvatarVR_mounts.txt", text, sizeof(text), &n);
  const auto settings = capture::parse_settings(std::string_view(text, n));
  first_person = saved_first = settings.first_person;
  arms_menu = saved_arms = settings.mounted_arms;
  arms_enabled.store(arms_menu);
  fp_enabled.store(first_person);
  eye_pose_steady.store(std::string_view(text, n).find("eye_pose_steady=0") == std::string_view::npos);
  auto_capture = settings.auto_capture;
  if (auto_capture) requested.store(true);
  log(std::string("build " __DATE__ " " __TIME__ "; first-person mounts (stage A eye anchor) ") +
      (first_person ? "ON" : "OFF") + "; capture read-only");
}
void save() {
  if (saved_first == first_person && saved_arms == arms_menu) return;
  const std::wstring path = dir + L"\\AvatarVR_mounts.txt";
  const std::string text = std::string("first_person_mounts=") + (first_person?"1":"0") + "\r\nmounted_arms=" +
                           (arms_menu?"1":"0") + "\r\nauto_capture=" + (auto_capture?"1":"0") + "\r\n";
  if (settings_file::write(path, text)) { saved_first = first_person; saved_arms = arms_menu; }
}
bool start() {
  if (!guards_match()) { log("capture SKIP: on-mount query guard mismatch; no engine reads or writes enabled"); return false; }
  SYSTEMTIME t{}; GetLocalTime(&t);
  wchar_t name[128];
  std::swprintf(name, 128, L"\\MOUNT_CAMERA_%04u%02u%02u_%02u%02u%02u_%u.trace", t.wYear,t.wMonth,t.wDay,t.wHour,t.wMinute,t.wSecond,++capture_id);
  const auto path = dir + name;
  output = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (output == INVALID_HANDLE_VALUE) { log("capture SKIP: cannot create unique trace"); return false; }
  char header[384];
  std::snprintf(header, sizeof(header), "MOUNT_CAMERA v1 build=" __DATE__ " " __TIME__ " pid=%lu image=%p preference=%d override=0 duration_ms=%llu weapon_observer=1 mounted_rig_capture=1 weapon_interval_ms=250 weapon_sources=64 entity_callers=401D7B2,401D861\r\n",
                GetCurrentProcessId(), game::module_base(), first_person,
                static_cast<unsigned long long>(capture::duration_ms));
  if (!write(output, header)) { CloseHandle(output); output = INVALID_HANDLE_VALUE; log("capture SKIP: header write failed"); return false; }
  {
    std::lock_guard<std::mutex> lock(mutex);
    pending.clear(); meshes.clear(); weapon_meshes.clear(); palette_resources.clear();
    pending.reserve(kQueueLimit); meshes.reserve(capture::max_meshes);
    weapon_meshes.reserve(capture::max_weapon_meshes); palette_resources.reserve(capture::max_palette_resources);
    weapon_budget=capture::WeaponBudget{};
    dropped.store(0); fp_palettes.store(0); last_fp.store(0);
    budget.start(GetTickCount64());
  }
  observing.store(true, std::memory_order_release);
  std::string filename;
  for (const wchar_t* c=name;*c;++c) filename += static_cast<char>(*c); // generated ASCII filename
  log("capture file: " + filename);
  log("capture started: 180 seconds, camera pre/post at <=30 Hz each; nearby palette observation; no camera, pose, gameplay or locomotion writes");
  return true;
}
void finish(const char* reason) {
  observing.store(false, std::memory_order_release);
  std::vector<Row> rows;
  unsigned samples = 0,weapon_capacity_drops=0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    budget.active = false; samples = budget.samples; weapon_capacity_drops=weapon_budget.capacity_drops; rows.swap(pending);
  }
  bool ok = true;
  for (const auto& row : rows) ok = write(output, row.text + "\r\n") && ok;
  char footer[256];
  std::snprintf(footer, sizeof(footer), "END reason=%s camera_samples=%u dropped=%u fp_palette_calls=%u write_ok=%d weapon_capacity_drops=%u\r\n",
                reason, samples, dropped.load(), fp_palettes.load(), ok,weapon_capacity_drops);
  ok = write(output, footer) && ok;
  if (output != INVALID_HANDLE_VALUE) { CloseHandle(output); output = INVALID_HANDLE_VALUE; }
  log(std::string(footer) + (ok ? "Capture ready for analysis; feature remains unverified." : "Capture incomplete: file write failure."));
}
}  // namespace

bool* first_person_flag() { return &first_person; }
bool* mounted_arms_flag() { return &arms_menu; }
void eye_point(const float* head16, float* eye3) {
  const float local[3] = {calib_now[0].load(std::memory_order_relaxed), calib_now[1].load(std::memory_order_relaxed),
                          calib_now[2].load(std::memory_order_relaxed)};
  mount_anchor::to_world(head16, local, eye3);
}
bool harness_near(int hand) { return hand >= 0 && hand < 2 && grab_near[hand].load(std::memory_order_relaxed); }
bool harness_held(int hand) { return hand >= 0 && hand < 2 && grab_held[hand].load(std::memory_order_relaxed); }
float arm_ik_weight(int hand) { return hand >= 0 && hand < 2 ? grab_weight[hand].load(std::memory_order_relaxed) : 1.f; }
bool mounted_arms() { return arms_enabled.load(std::memory_order_relaxed); }
bool harness_grip(int hand, float* pelvis_local3, float* spread_m) {
  if (hand < 0 || hand > 1 || !pelvis_local3) return false;
  std::lock_guard<std::mutex> lock(harness_mutex);
  const auto& g = harness[hand];
  if (g.n < 90) return false;  // about a second of steady riding
  for (int k = 0; k < 3; ++k) pelvis_local3[k] = g.mean[k];
  if (spread_m) *spread_m = g.spread();
  return true;
}
bool rider_view() { return anchor_on.load(std::memory_order_relaxed); }
bool capture_active() { return observing.load(std::memory_order_acquire); }
void observe_weapon_entity(const void* object,std::uintptr_t caller,bool valid,const float* native,
                           const char* reason,float flat_gap,float game_gap,bool game_socket,bool wrote) {
  if(!capture_active()) return;
  const auto now=GetTickCount64();
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
  if(!lock.owns_lock()) {++dropped; return;}
  if(!capture_active()) return;
  const auto p=game::pointers(); const auto state=mount_state::get();
  if(!weapon_identity(p,state) || !weapon_budget.take(budget,now,{reinterpret_cast<std::uintptr_t>(object),0,0,1})) return;
  const auto equipped=ptr(p.body,0x250);
  const auto entity=ptr(equipped,0x3FA0+0xC8);
  char b[560];
  std::snprintf(b,sizeof(b),"WE us=%lld tick=%llu present=%llu step=%llu tid=%lu object=%p caller=%llX "
    "entity_match=%d classification=%s reason=%s flat_gap=%.5f game_gap=%.5f game_socket=%d wrote=%d",
    microseconds(),now,present.load(),camera_step.load(),GetCurrentThreadId(),object,
    static_cast<unsigned long long>(caller),entity && entity==object,
    entity && entity==object?"equipped_entity_match":"unclassified",reason,flat_gap,game_gap,game_socket,wrote);
  if(!native) return;
  for(int i=0;i<16;++i) valid=valid && std::isfinite(native[i]);
  std::string line=b; append_matrix(line,"native_world",valid,native);
  if(append_weapon_context(line,p,state,now)) queue(std::move(line)); else ++dropped;
}
void observe_rider_hands(const void* mesh,const void* resource,const void* source,const void* owner,const short* cell,
                         const bool* valid,const float (*native)[16],const float (*rendered)[16],
                         const float* weight,std::uint64_t sequence) {
  if(!capture_active()) return;
  const auto now=GetTickCount64();
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
  if(!lock.owns_lock()) {++dropped; return;}
  if(!capture_active()) return;
  const auto p=game::pointers(); const auto state=mount_state::get();
  if(p.body!=owner) {++dropped; return;}
  if(!weapon_identity(p,state) || !weapon_budget.take(budget,now,
     {reinterpret_cast<std::uintptr_t>(resource),0,0,3})) return;
  char b[320];
  std::snprintf(b,sizeof(b),"WA us=%lld tick=%llu present=%llu step=%llu tid=%lu mesh=%p resource=%p source=%p "
    "rider_seq=%llu weight=%.4f,%.4f",microseconds(),now,present.load(),camera_step.load(),GetCurrentThreadId(),
    mesh,resource,source,static_cast<unsigned long long>(sequence),weight[0],weight[1]);
  std::string line=b;
  for(int h=0;h<2;++h) {
    float before[16],after[16]; std::memcpy(before,native[h],64); std::memcpy(after,rendered[h],64);
    for(int k=0;k<3;++k) {before[12+k]+=1024.f*cell[k]; after[12+k]+=1024.f*cell[k];}
    bool finite=valid[h]; for(int i=0;i<16;++i) finite=finite && std::isfinite(before[i]) && std::isfinite(after[i]);
    append_matrix(line,h?"palette_native_R_world":"palette_native_L_world",finite,before);
    append_matrix(line,h?"palette_rendered_R_world":"palette_rendered_L_world",finite,after);
  }
  if(append_weapon_context(line,p,state,now)) queue(std::move(line)); else ++dropped;
}
void request_capture() { requested.store(true); }
void cancel_capture() { cancelled.store(true); }

void on_present() {
  ++present;
  if (!loaded) load();
  if (dir.empty()) return;
  fp_enabled.store(first_person, std::memory_order_relaxed);
  arms_enabled.store(arms_menu, std::memory_order_relaxed);
  if (const unsigned n = grab_log_n.load(std::memory_order_acquire)) {
    for (unsigned i = 0; i < n; ++i) {
      char b[160];
      std::snprintf(b, sizeof(b), "harness %s %s (controller %.2f m from the %s harness point)",
                    grab_log[i].hand ? "right" : "left", grab_log[i].edge > 0 ? "GRAB" : "release", grab_log[i].distance,
                    grab_log[i].learned ? "learned" : "live palm");
      log(b);
    }
    grab_log_n.store(0, std::memory_order_release);
  }
  if (harness_report.exchange(false)) {
    // Ride ended: the grip learned so far (pelvis axes: right, up, forward) goes to the runtime log.
    std::string text;
    {
      std::lock_guard<std::mutex> lock(harness_mutex);
      for (int h = 0; h < 2; ++h) {
        const auto& g = harness[h];
        char b[256];
        std::snprintf(b, sizeof(b), "harness_%s=%.4f,%.4f,%.4f spread=%.4f samples=%u fwd=%.3f,%.3f,%.3f up=%.3f,%.3f,%.3f\r\n",
                      h ? "right" : "left", g.mean[0], g.mean[1], g.mean[2], g.spread(), g.n,
                      g.axis_fwd[0], g.axis_fwd[1], g.axis_fwd[2], g.axis_up[0], g.axis_up[1], g.axis_up[2]);
        text += b;
      }
    }
    log("harness grip (game riding pose, pelvis frame):\r\n" + text.substr(0, text.size() - 2));
    const std::string frames = skin_palette_probe::rider_frame_report();
    log("harness frame census (palms in each nearby mesh's frame; smallest spread first):\r\n" + frames.substr(0, frames.size() - 2));
  }
  {
    mount_state::Transition t[8];
    const auto n = mount_state::drain_transitions(t, 8);
    for (std::size_t i = 0; i < n; ++i) {
      char b[200];
      std::snprintf(b, sizeof(b), "mount: %s -> %s tick=%llu camera_mode=%d movement_state=%d mount_type=%d%s",
                    mount_state::name(t[i].from), mount_state::name(t[i].to), static_cast<unsigned long long>(t[i].ms),
                    t[i].camera_mode, t[i].movement_state, t[i].mount_type,
                    t[i].to == mount_state::Phase::Riding && t[i].camera_mode != 7 && t[i].camera_mode != 4
                        ? " (camera mode not seen before)" : "");
      log(b);
    }
    if (calib_ready.exchange(false, std::memory_order_acquire)) {
      char b[160];
      std::snprintf(b, sizeof(b), "eye calibrated on foot: eye = C_head + (%.3f, %.3f, %.3f) in head axes (right, up, fwd)",
                    calib_logged[0].load(), calib_logged[1].load(), calib_logged[2].load());
      log(b);
    }
    static bool logged_on = false;
    static unsigned logged_rejects = 0;
    const bool on = anchor_on.load(std::memory_order_relaxed);
    const unsigned rejects = anchor_rejects.load(std::memory_order_relaxed);
    if (on != logged_on || rejects != logged_rejects) {
      static const char* const sources[] = {"first-person bone 0 (fallback)", "C_head joint (eye not calibrated)",
                                            "C_head + calibrated eye"};
      char b[256];
      std::snprintf(b, sizeof(b), "eye anchor %s; source %s; correction %.2f m; rejected steps %u; toggle %d; "
                    "head/hair builds hidden so far %u", on ? "ON" : "off",
                    sources[anchor_source.load(std::memory_order_relaxed) % 3],
                    anchor_len.load(std::memory_order_relaxed), rejects, first_person ? 1 : 0,
                    skin_palette_probe::rider_hidden_builds());
      log(b);
      logged_on = on; logged_rejects = rejects;
    }
    static ULONGLONG next_rider_log = 0;
    if (on && GetTickCount64() >= next_rider_log) {
      next_rider_log = GetTickCount64() + 5000;
      const auto c = skin_palette_probe::rider_counters();
      {char e[160]; std::snprintf(e, sizeof(e), "eye pose steady: on=%d steps=%u correction_max=%u mm",
         eye_pose_steady.load(std::memory_order_relaxed) ? 1 : 0, eye_steady_steps.load(std::memory_order_relaxed),
         eye_steady_mm.exchange(0, std::memory_order_relaxed)); log(e);}
      char b[256];
      std::snprintf(b, sizeof(b), "rider: arm_apps=%u rest_solves=%u (rest captures %u) self_solves=%u ref_solves=%u "
                    "mismatch=%u (max %d mm, last joint %d) no_grip=%u anchored_bones=%u head_hidden=%u arms_toggle=%d "
                    "ik_weight=%.2f,%.2f", c.arm_apps, c.rest_solves, c.rest_captures, c.self_solves, c.solves,
                    c.mismatch, c.mismatch_mm, c.mismatch_bone, c.no_grip, c.anchored, c.hidden, arms_menu ? 1 : 0,
                    arm_ik_weight(0), arm_ik_weight(1));
      log(b);
    }
  }
  const auto now = GetTickCount64();
  if (now >= next_poll) { next_poll = now + 1000; save(); }
  if (cancelled.exchange(false)) {
    waiting = false; requested.store(false);
    if (observing.load()) finish("cancelled");
  }
  if (requested.exchange(false) && !observing.load()) {
    waiting = true;
    log("capture queued; starts automatically in stereo gameplay, not in loading/menu screens");
  }
  if (waiting && native_stereo::stereo_running() && !native_stereo::front_end() && game::read().valid) {
    waiting = false; start();
  }
  if (!observing.load()) return;
  bool expired = false;
  std::vector<Row> rows;
  {
    std::lock_guard<std::mutex> lock(mutex);
    expired = budget.expired(now); rows.swap(pending);
  }
  bool ok = true;
  for (const auto& row : rows) ok = write(output, row.text + "\r\n") && ok;
  if (!ok || expired) finish(ok ? "bounded_window" : "write_failed");
}

bool eye_anchor(const float* cam_pos, float* delta3) {
  const long long now = microseconds();
  mount_anchor::Input in;
  in.dt = anchor_us ? static_cast<float>(now - anchor_us) * 1e-6f : 0.f;
  anchor_us = now;
  const auto s = mount_state::get();
  // Camera mode 4 is the game's own first-person head camera: nothing to correct.
  in.wanted = fp_enabled.load(std::memory_order_relaxed) && s.phase != mount_state::Phase::None && s.camera_mode != 4;
  const auto p = game::pointers();
  in.body_ok = read(p.body, 0x94, in.body) &&
               std::isfinite(in.body[0]) && std::isfinite(in.body[1]) && std::isfinite(in.body[2]);
  for (int k = 0; k < 3; ++k) in.cam[k] = cam_pos[k];
  float head[16]{};
  const bool head_ok = third_person_head(p, head);
  if (head_ok && s.phase == mount_state::Phase::None && s.camera_mode == 4) {
    float obs[3];  // on foot the game camera is the eye: learn it in the head bone's frame
    if (mount_anchor::to_local(head, cam_pos, obs)) eye_calib.add(obs, in.dt);
    for (int k = 0; k < 3; ++k) calib_now[k].store(eye_calib.local[k], std::memory_order_relaxed);
    if (eye_calib.samples == mount_anchor::kCalibMin) {
      for (int k = 0; k < 3; ++k) calib_logged[k].store(eye_calib.local[k], std::memory_order_relaxed);
      calib_ready.store(true, std::memory_order_release);
    }
  }
  if (head_ok) {
    mount_anchor::to_world(head, eye_calib.local, in.bone);  // uncalibrated: the head joint itself
    in.bone_ok = std::isfinite(in.bone[0]) && std::isfinite(in.bone[1]) && std::isfinite(in.bone[2]);
    anchor_source.store(eye_calib.ready() ? 2 : 1, std::memory_order_relaxed);
  } else {
    float bone[16]{};  // fallback: first-person camera bone (not animated while mounted; stands behind the rider)
    in.bone_ok = game::bone_world(p, 0, bone) &&
                 std::isfinite(bone[12]) && std::isfinite(bone[13]) && std::isfinite(bone[14]);
    for (int k = 0; k < 3; ++k) in.bone[k] = bone[12 + k];
    anchor_source.store(0, std::memory_order_relaxed);
  }
  {  // eye_pose_steady (see its declaration)
    static float history[5][3]{}; static unsigned history_n = 0; static float steady[3]{}; static bool steady_have = false;
    float pelvis[16]{}, q[3]{};
    const bool use = in.wanted && in.body_ok && in.bone_ok && head_ok && eye_pose_steady.load(std::memory_order_relaxed) &&
                     tp_bone(p, 6, pelvis) && mount_anchor::to_local(pelvis, in.body, q);
    if (!use) { steady_have = false; history_n = 0; }
    else {
      std::memcpy(history[history_n++ % 5], q, sizeof(q));
      const unsigned count = std::min(history_n, 5u);
      float median[3];
      for (int k = 0; k < 3; ++k) {
        float v[5]; for (unsigned i = 0; i < count; ++i) v[i] = history[i][k];
        std::sort(v, v + count); median[k] = v[count / 2];
      }
      float jump = 0.f; for (int k = 0; k < 3; ++k) jump += (median[k] - steady[k]) * (median[k] - steady[k]);
      if (!steady_have || !(jump < 0.5f * 0.5f)) { std::memcpy(steady, median, sizeof(steady)); steady_have = true; }
      else for (int k = 0; k < 3; ++k) {
        const float tau = k == 1 ? 0.10f : 0.25f;
        steady[k] += (1.f - std::exp(-in.dt / tau)) * (median[k] - steady[k]);
      }
      float at[3]; mount_anchor::to_world(pelvis, steady, at);
      float c[3], len = 0.f; for (int k = 0; k < 3; ++k) { c[k] = in.body[k] - at[k]; len += c[k] * c[k]; }
      len = std::sqrt(len);
      if (std::isfinite(len) && len < 0.5f) {
        for (int k = 0; k < 3; ++k) in.bone[k] += c[k];
        eye_steady_steps.fetch_add(1, std::memory_order_relaxed);
        const unsigned mm = static_cast<unsigned>(len * 1000.f); unsigned seen = eye_steady_mm.load(std::memory_order_relaxed);
        while (mm > seen && !eye_steady_mm.compare_exchange_weak(seen, mm, std::memory_order_relaxed)) {}
      }
    }
  }
  record_harness(p, s.phase == mount_state::Phase::Riding && s.camera_mode != 4);
  update_grab(p, s.phase != mount_state::Phase::None && s.camera_mode != 4 && anchor_on.load(std::memory_order_relaxed) &&
                 arms_enabled.load(std::memory_order_relaxed), in.dt);
  const unsigned rejects = anchor.rejects;
  const float* d = anchor.step(in);
  for (int k = 0; k < 3; ++k) delta3[k] = d[k];
  if (anchor.rejects != rejects) anchor_rejects.fetch_add(1, std::memory_order_relaxed);
  const bool on = anchor.weight > 0.f;
  anchor_on.store(on, std::memory_order_relaxed);
  if (on) anchor_len.store(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), std::memory_order_relaxed);
  return on;
}

void on_camera(void* holder, bool post) {
  if (post) mount_state::update_camera_post(holder);
  if (!post) ++camera_step;
  if (!observing.load(std::memory_order_acquire)) return;
  const auto now = GetTickCount64();
  std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
  if (!lock.owns_lock()) { ++dropped; return; }
  if (!observing.load() || !budget.take(now, post)) return;
  const auto p = game::pointers();
  if (!holder || holder != p.holder || !p.body) return;
  if(post) {
    const auto state=mount_state::get();
    if(weapon_identity(p,state) && weapon_budget.take(budget,now,{0,0,0,0})) {
      char prefix[160]; std::snprintf(prefix,sizeof(prefix),"WC us=%lld tick=%llu present=%llu step=%llu tid=%lu",
        microseconds(),now,present.load(),camera_step.load(),GetCurrentThreadId());
      std::string context=prefix;
      if(append_weapon_context(context,p,state,now,true)) queue(std::move(context)); else ++dropped;
    }
  }
  const auto mode = ptr(holder, 0x40);
  const auto movement = ptr(p.body, 0x388);
  const auto state = ptr(movement, 0x1C0);
  const auto state_vt = ptr(state, 0);
  const auto getter = ptr(state_vt, 0x10);
  const auto partner = ptr(p.body, 0x278);
  const auto comp = ptr(p.body, 0x1A8);
  const auto pose_fp = ptr(comp, 0x500), pose_body = ptr(comp, 0x4F8);
  unsigned char code[16]{};
  int state_type = -1, mode_id = -1, mount_type = -1, partner_status = -2, requested_mode = -1;
  unsigned long long partner_id[2]{};
  bool partner_read = read(partner, 0x18, partner_id) && read(partner, 0x30, mount_type) && read(partner, 0x34, partner_status);
  const bool leaf = readable_code(getter,sizeof(code)) && read(getter, 0, code) && capture::constant_state(code, sizeof(code), state_type);
  read(mode, 0xC, mode_id); read(holder, 0x18, requested_mode);
  float body[3]{}, cam[16]{}, mode_matrix[16]{}, look[3]{};
  const bool body_ok = read(p.body, 0x94, body), cam_ok = read(p.camera, 0x20, cam);
  const bool mode_ok = read(mode, 0x20, mode_matrix), look_ok = read(p.look, 0x34, look);
  head_pose::Snapshot xr{}; const bool xr_ok = xr_runtime::latest_views(&xr);
  vr_arms::Sample arms{}; const bool arms_ok = vr_arms::latest(arms);
  float gun[16]{}; const bool gun_ok = skin_palette_probe::held_weapon_root(gun);
  const bool identity_ok = game::pointers().body == p.body && game::pointers().holder == holder;
  char b[1400];
  const auto fp_tick = last_fp.load();
  std::snprintf(b, sizeof(b),
    "C us=%lld tick=%llu present=%llu step=%llu phase=%s tid=%lu identity_ok=%d handle=%p body=%p holder=%p camera=%p "
    "mode=%p mode_vt=%llX mode_id=%d requested_mode=%d state=%p state_vt=%llX getter=%llX leaf=%d type=%d "
    "partner=%p partner_ok=%d partner_id=%llX,%llX mount_type=%d partner_status=%d "
    "body_ok=%d body_pos=%.6f,%.6f,%.6f look_ok=%d look=%.6f,%.6f,%.6f "
    "xr_ok=%d xr_seq=%llu xr_time=%lld xr_valid=%d grips=%d,%d arms_ok=%d arms_seq=%llu arms_us=%lld arms_valid=%d,%d "
    "fp_calls=%u fp_age_ms=%llu weapon_candidate=%p",
    microseconds(),now,present.load(),camera_step.load(),post?"post":"pre",GetCurrentThreadId(),identity_ok,p.handle,p.body,holder,p.camera,
    mode,rva(ptr(mode,0)),mode_id,requested_mode,state,rva(state_vt),rva(getter),leaf,state_type,
    partner,partner_read,partner_id[0],partner_id[1],mount_type,partner_status,
    body_ok,body[0],body[1],body[2],look_ok,look[0],look[1],look[2],
    xr_ok,xr.seq,xr.display_time,xr.valid,xr.grip[0].valid,xr.grip[1].valid,
    arms_ok,arms.seq,arms.sampled_qpc_us,arms.valid[0],arms.valid[1],fp_palettes.load(),fp_tick?now-fp_tick:~0ull,ptr(p.body,0x250));
  std::string line = b;
  append_matrix(line, "camera", cam_ok, cam);
  char code_text[96];
  std::snprintf(code_text, sizeof(code_text), " state_getter_bytes=%02X%02X%02X%02X%02X%02X%02X%02X",
                code[0],code[1],code[2],code[3],code[4],code[5],code[6],code[7]);
  line += code_text;
  append_matrix(line, "mode_raw", mode_ok, mode_matrix);  // raw, not a proven mount transform
  append_pose(line, "pose_fp", pose_fp); append_pose(line, "pose_body", pose_body);
  append_matrix(line, "arms_basis", arms_ok, arms.basis);
  append_matrix(line, "left_grip", arms_ok && arms.valid[0], arms.local[0]);
  append_matrix(line, "right_grip", arms_ok && arms.valid[1], arms.local[1]);
  append_matrix(line, "gun_root", gun_ok, gun);
  const auto counts = skin_palette_probe::counters();
  char count_text[160];
  std::snprintf(count_text,sizeof(count_text)," ik_apps=%u accessory_apps=%u weapon_apps=%u clean_solves=%u gun_places=%u",
                counts.fp_body,counts.accessories,counts.weapon,counts.solves,counts.gun_places);
  line += count_text;
  queue(std::move(line));
}

void observe_palette(const void* mesh, const void* root, const void* bones, const void* cells,
                     std::uintptr_t caller_rva) {
  if (!observing.load(std::memory_order_acquire)) return;
  const auto now = GetTickCount64();
  std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
  if (!lock.owns_lock()) { ++dropped; return; }
  if (!observing.load() || budget.expired(now)) return;
  const auto p = game::pointers();
  float body[3]{}, root_matrix[16]{}; short cell[3]{};
  if (!read(p.body,0x94,body) || !read(root,0,root_matrix) || !read(cells,0,cell)) return;
  float distance2 = 0.f;
  for (int k=0;k<3;++k) {
    const float d = root_matrix[12+k] + 1024.f * cell[k] - body[k];
    distance2 += d*d;
  }
  if (!std::isfinite(distance2) || distance2>64.f) return;
  const auto resource = ptr(root,0x58);
  Mesh* entry = nullptr;
  for (auto& seen : meshes) if (seen.mesh == mesh && seen.root == root && seen.resource == resource) { entry=&seen; break; }
  if(!entry) for(auto& seen:weapon_meshes)
    if(seen.mesh==mesh && seen.resource==resource) {entry=&seen; break;}
  if (!entry) {
    PaletteResource* description=nullptr;
    for(auto& seen:palette_resources) if(seen.identity==resource) {description=&seen; break;}
    if(!description) {
      if(!resource || palette_resources.size()>=capture::max_palette_resources) {++dropped; return;}
      PaletteResource fresh; fresh.identity=resource; auto& path=fresh.path;
      unsigned char name[16]{};
      if (ptr(resource,0) == game::module_base()+0x57C6D70 && read(resource,0x18,name)) {
        if (name[15]) {
          const unsigned char* heap = nullptr; std::memcpy(&heap,name,sizeof(heap));
          for (unsigned i=0;i<sizeof(path)-1;++i) {
            if (!read(heap,i,path[i])) { strcpy_s(path,"<unreadable>"); break; }
            if (path[i]==0) break;
            if (static_cast<unsigned char>(path[i])<32 || static_cast<unsigned char>(path[i])>126) { strcpy_s(path,"<non-ascii>"); break; }
          }
        } else std::memcpy(path,name,15);
      } else strcpy_s(path,"<unknown-resource>");
      fresh.fp=_stricmp(path,"BLUE/BAKED/CHARACTERART/PLAYER/BODY/MALE/P_BODY_01_M/P_BODY_01_FP_M.MMB")==0;
      fresh.weapon=std::strstr(path,"/WEAPON/PLAYER/")!=nullptr;
      palette_resources.push_back(fresh); description=&palette_resources.back();
      unsigned palette_count=0; read(bones,8,palette_count);
      char record[400];
      std::snprintf(record,sizeof(record),"R us=%lld tid=%lu resource=%p path=%s count=%u",microseconds(),
        GetCurrentThreadId(),resource,description->path,palette_count);
      queue(std::string(record)+skin_palette_probe::capture_palette_names(root));
    }
    if(!capture::admit_palette(description->weapon,static_cast<unsigned>(meshes.size()),
                              static_cast<unsigned>(weapon_meshes.size()),distance2)) {
      if(description->weapon || distance2<=64.f) ++dropped;
      return;
    }
    auto& pool=description->weapon?weapon_meshes:meshes;
    pool.push_back({mesh,root,resource,0,description->fp,description->weapon}); entry=&pool.back();
    char b[640];
    std::snprintf(b,sizeof(b),"M us=%lld tid=%lu mesh=%p root=%p resource=%p resource_vt=%llX path=%s fp=%d",microseconds(),GetCurrentThreadId(),mesh,root,resource,rva(ptr(resource,0)),description->path,description->fp);
    queue(b);
  }
  if (entry->fp) { ++fp_palettes; last_fp.store(now); }
  if (entry->weapon) {
    const auto state=mount_state::get();
    if(!weapon_identity(p,state) || !weapon_budget.take(budget,now,
       {reinterpret_cast<std::uintptr_t>(mesh),reinterpret_cast<std::uintptr_t>(resource),0,2})) return;
  } else {
    if (now < entry->next) return;
    entry->next = now + 100;
  }
  unsigned count = 0; float first[16]{};
  const auto source = ptr(bones,0);
  const bool count_ok = read(bones,8,count);
  const bool first_ok = count_ok && count>0 && count<=1024 && read(source,0,first);
  char b[480];
  std::snprintf(b,sizeof(b),"P us=%lld tick=%llu present=%llu step=%llu tid=%lu mesh=%p root=%p source=%p count_ok=%d count=%u caller=%llX fp=%d cell=%d,%d,%d dist=%.4f",
                microseconds(),now,present.load(),camera_step.load(),GetCurrentThreadId(),mesh,root,source,count_ok,count,caller_rva,entry->fp,cell[0],cell[1],cell[2],std::sqrt(distance2));
  std::string line=b;
  append_matrix(line,"root",true,root_matrix); append_matrix(line,"bone0",first_ok,first);
  if(entry->weapon) {
    char identity[128]; std::snprintf(identity,sizeof(identity)," resource=%p weapon_palette=1 classification=unclassified native_before_mod=1",resource);
    line+=identity;
    const auto state=mount_state::get();
    if(!append_weapon_context(line,p,state,now)) {++dropped; return;}
  }
  queue(std::move(line));
}
}  // namespace mount_camera
