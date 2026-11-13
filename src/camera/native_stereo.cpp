#include "native_stereo.hpp"
#include "scan_uv_shader.hpp"
#include "ui_canvas.hpp"
#include "ui_layout.hpp"
#include "ui_cursor.hpp"
#include "../vr/menu_pointer.hpp"
#include "ui_piece.hpp"
#include "../vr/lens_projection.hpp"
#include "../dx12/dx12.hpp"
#include "camera.hpp"
#include "time_sync.hpp"

#include "../diagnostics/eye_probe.hpp"
#include "../diagnostics/pose_phase_probe.hpp"
#include "../diagnostics/resource_names.hpp"
#include "../vr/dlss_twin.hpp"
#include "../vr/xr_runtime.hpp"
#include "../game/player.hpp"
#include "../vr/vr_weapon.hpp"
#include "../vr/vr_arms.hpp"
#include "../game/mount_state.hpp"
#include "mount_camera.hpp"

#include <d3d12.h>

#include <windows.h>

#include <MinHook.h>

#include <algorithm>
#include <vector>
#include <atomic>
#include <cmath>
#include <intrin.h>
#include <unordered_map>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include "../settings_file.hpp"

namespace {

// afop.exe 1.0.2.2 (SHA-256 98588DA2...1E91), readable .xpdata code. Bytes are checked before hooking.
constexpr std::uintptr_t kRenderRva = 0x57CEC0;
constexpr unsigned char kRenderSig[] = {0x40, 0x56, 0x57, 0x48, 0x81, 0xEC, 0x98, 0x00, 0x00, 0x00, 0x33, 0xFF, 0x48, 0x8B,
                                        0xF1, 0x89, 0xBC, 0x24, 0xB0, 0x00, 0x00, 0x00, 0x40, 0x38, 0x79, 0x24, 0x0F, 0x84,
                                        0x19, 0x03, 0x00, 0x00, 0x39, 0xB9, 0x58, 0x14, 0x00, 0x00};
constexpr std::uintptr_t kCaptureRva = 0x594F40;
constexpr unsigned char kCaptureSig[] = {0x4C, 0x8B, 0xDC, 0x55, 0x56, 0x49, 0x8D, 0x6B, 0xA1, 0x48, 0x81, 0xEC, 0xD8, 0x00,
                                         0x00, 0x00, 0x8B, 0x81, 0x58, 0x14, 0x00, 0x00, 0x48, 0x8B, 0xF1, 0x83, 0xE8, 0x02,
                                         0x83, 0xF8, 0x02, 0x0F, 0x87, 0xD5, 0x01, 0x00, 0x00};
constexpr std::uintptr_t kStereoRva = 0x58B270;
constexpr unsigned char kStereoSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56,
                                        0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24, 0x20, 0xF9, 0xFF, 0xFF, 0x48, 0x81, 0xEC, 0xE0,
                                        0x07, 0x00, 0x00, 0xC7, 0x81, 0x00, 0x15, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
// Gfx_View task wait ("* Wait for Gfx_View tasks"), called from both the mono and stereo paths.
constexpr std::uintptr_t kWaitRva = 0x5BE8E0;
constexpr unsigned char kWaitSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57,
                                      0x48, 0x83, 0xEC, 0x20, 0x0F, 0xB6, 0xEA, 0x48, 0x8B, 0xF9, 0x44, 0x0F, 0xB6, 0xCA, 0x48, 0x81,
                                      0xC1, 0x30, 0x4A, 0x00, 0x00};
// Stereo composite (picks leftright/redgreen/topdown from view[+0x1210] by mode).
constexpr std::uintptr_t kCompositeRva = 0x52E340;
constexpr unsigned char kCompositeSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x48, 0x89, 0x7C, 0x24, 0x20,
                                           0x41, 0x56, 0x48, 0x83, 0xEC, 0x60, 0x65, 0x48, 0x8B, 0x04, 0x25, 0x58, 0x00, 0x00, 0x00,
                                           0x48, 0x8B, 0xF1};
// Frame task-graph scheduler (see research/STEREO_ROUTE.md):
//   add view task  +0x51B860 (ctx=view+0x49F0, out, kind, job): appends the job to phase list `kind` and creates
//                  the per-frame gate task for that kind unless ctx[+0x5478+kind*8] already holds one
//   gate task      +0x5931E0 (?, graph, kind): runs every job in phase list `kind`
//   gate submitter +0x57FD20 (registered by Gfx_View::Render outside the left-eye pass): submits the frame's gates
constexpr std::uintptr_t kAddTaskRva = 0x51B860;
constexpr unsigned char kAddTaskSig[] = {0x48, 0x8B, 0xC4, 0x55, 0x41, 0x56, 0x48, 0x8D, 0x68, 0xA1, 0x48, 0x81, 0xEC, 0xF8, 0x00, 0x00, 0x00,
                                         0x48, 0x89, 0x58, 0x10, 0x4C, 0x8B, 0xF2, 0x48, 0x89, 0x78, 0x20, 0x49, 0x8B, 0xD1, 0x48, 0x8B, 0xF9};
constexpr std::uintptr_t kGateRva = 0x5931E0;
constexpr unsigned char kGateSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x48, 0x89, 0x7C, 0x24, 0x20, 0x41, 0x56, 0x48,
                                      0x83, 0xEC, 0x20, 0x48, 0x8D, 0x8A, 0x90, 0x00, 0x00, 0x00, 0x45, 0x0F, 0xB6, 0xF0, 0x48, 0x8B, 0xFA};
constexpr std::uintptr_t kSubmitRva = 0x57FD20;
constexpr unsigned char kSubmitSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C, 0x24, 0x18, 0x48, 0x89, 0x74, 0x24, 0x20, 0x57,
                                        0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC, 0xD0, 0x00, 0x00, 0x00};
constexpr std::uint32_t kModeOffset = 0x1458;
constexpr std::uint32_t kGuardOffset = 0x1500;

// Second-view experiment (user-approved 2026-10-02): per-eye camera build 0x1405BCC40(view, sched), called from
// Gfx_ViewManager::StartFrame on the game thread. It copies the view's camera ([view+0x10], 0x8B0-byte object) into
// the frame-data eye slots (frame + 0xEEF0 + eye*0x4590, five sub-cameras 0x8B0 apart). After it returns we rebuild
// slot 0 of one chosen view from a sideways-shifted copy, with the same engine calls its own stereo eye code uses.
constexpr std::uintptr_t kCamBuildRva = 0x5BCC40;
constexpr unsigned char kCamBuildSig[] = {0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x57,
                                          0x48, 0x8D, 0xAC, 0x24, 0xC8, 0xE1, 0xFF, 0xFF, 0xB8, 0x38, 0x1F, 0x00};
// Render size source (2026-10-09 [STATIC], Ghidra dump D:\re\dump\98588da2): the view update 0x1405B8420
// ("ourViewWaitForRenderTasksInUpdateView") gives the view's camera [view+0x10] its output size through
// 0x143082750(camera, size) (camera +0x264/+0x268, dirty +0x270). The size comes from 0x1415D63C0(render, out):
// the largest registered output (swapchain) size, cached at [0x149892000]+0x490F0/+0x490F4. The per-eye camera build
// 0x1405BCC40 derives the render rect and DLSS extents from the camera size, and the offscreen target
// "Gfx_RenderTargets/myMidComposite" (0x1401B73E0) is sized from it too. Returning another size from the getter while
// a chosen view updates gives that view its own render size, upstream of DLSS (SpidyVR's initBuffers idea).
constexpr std::uintptr_t kUpdateViewRva = 0x5B8420;
constexpr unsigned char kUpdateViewSig[] = {0x40, 0x55, 0x53, 0x56, 0x57, 0x48, 0x8D, 0xAC, 0x24, 0x58, 0xFF, 0xFF,
                                            0xFF, 0x48, 0x81, 0xEC, 0xA8, 0x01, 0x00, 0x00};
constexpr std::uintptr_t kOutSizeRva = 0x15D63C0;
constexpr unsigned char kOutSizeSig[] = {0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x83, 0xB9,
                                         0xF0, 0x90, 0x04, 0x00, 0xFF};
// UI pass record function (main-view slot +0x130, shared into the second view; STEREO_ROUTE.md 2026-10-06 [STATIC]).
// Prologue from CODEDUMP 2026-10-06 (runs\20261006_062319_after): push of rbx/rbp/rsi/rdi/r12-r15, sub rsp,0x20.
constexpr std::uintptr_t kUiPassRva = 0x55EFE0;
constexpr unsigned char kUiPassSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54,
                                        0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x20};
// The UI pass's neighbours, main-view slots only (the second view has no such slots): +0x138 -> 0x55EE10 and
// +0x128 -> 0x55F150 (STEREO_ROUTE.md 2026-10-05 viewslot). [STATIC] CODEDUMP 2026-10-06 (runs\20261006_063017_after):
// both are (owner, view, ...) like the UI pass (0x55EE10: mov r12,rdx / mov r14,rcx; 0x55F150: mov r15,rdx /
// mov rbx,rcx) and call 0x586EC0(owner, view, id, 1), the helper the present pass calls with id 2: 0x55EE10 with
// id 7, 0x55F150 with ids 9 and 10. [AUTHOR] + capture 2026-10-08: frosted-glass rectangles in the LEFT eye only at
// the unplaced UI panel positions; the UI pass records no compute and no sub-target viewport/scissor is set outside
// it (rect probe 07:15-07:16: 0 rects), so the glass is drawn with full-target viewports by main-only work
// [INFERENCE: these passes]. They get the UI placement too ("uimapx MASK": bit 0 = 0x55EE10, bit 1 = 0x55F150; default 3).
constexpr std::uintptr_t kUiExtra7Rva = 0x55EE10;
constexpr unsigned char kUiExtra7Sig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24,
                                          0x18, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x30};
constexpr std::uintptr_t kUiExtra9Rva = 0x55F150;
constexpr unsigned char kUiExtra9Sig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x4C, 0x89, 0x4C, 0x24, 0x20, 0x55, 0x56, 0x57, 0x41,
                                          0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC, 0x70};
struct Helper {
  std::uintptr_t rva;
  unsigned char sig[10];
  void* fn;
  const char* what;
};
Helper g_helpers[] = {
    {0x3065310, {0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20}, nullptr, "camera copy-construct"},
    {0x0BB800, {0x0F, 0x10, 0x02, 0x4C, 0x8B, 0xC1, 0x48, 0x8D, 0x82, 0x80}, nullptr, "camera desc copy"},
    {0x3307750, {0x48, 0x83, 0xEC, 0x28, 0xF3, 0x0F, 0x10, 0x22, 0x4C, 0x8B}, nullptr, "world-position add"},
    {0x30825C0, {0x40, 0x53, 0x48, 0x83, 0xEC, 0x40, 0x0F, 0x10, 0x02, 0x48}, nullptr, "desc set matrix"},
    {0x3082920, {0x8B, 0x02, 0x89, 0x81, 0xE4, 0x00, 0x00, 0x00, 0x8B, 0x42}, nullptr, "desc set world position"},
    {0x3082400, {0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20}, nullptr, "camera build from desc"},
    {0x3067130, {0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20}, nullptr, "camera assign"},
    {0x3082690, {0xF3, 0x0F, 0x10, 0x0A, 0x0F, 0x2E, 0x89, 0x08, 0x01, 0x00}, nullptr, "desc set projection offset"},
};
enum { kHCopyCtor, kHDescCopy, kHWpAdd, kHSetMatrix, kHSetWp, kHBuild, kHAssign, kHProjOffset, kHCount };
using Fn2 = void* (*)(void*, const void*);
using Fn3 = void* (*)(void*, const void*, void*);
constexpr std::uint32_t kCameraOffset = 0x10;
constexpr std::uint32_t kWorldOffset = 0x1248;
constexpr std::uint32_t kFrameRing = 0x9EB0;
constexpr std::uint32_t kFrameIdxGame = 0x9EC4;
constexpr std::uint32_t kFrameStride = 0x1C070;
constexpr std::uint32_t kSubCams[5] = {0xEEF0, 0xF7A0, 0x10050, 0x10900, 0x111B0};
bool g_helpers_ok = false;
std::atomic<int> g_second_view{-1};       // view index to shift, -1 = off
std::atomic<int> g_second_sep_um{0};      // sideways shift in micrometres (+ = right)
std::atomic<bool> g_second_world{false};  // also point the view at view #g_second_src's world
std::atomic<int> g_second_src{0};
std::atomic<std::uint64_t> g_second_applied{0}, g_second_failed{0}, g_cam_builds{0};
void* g_world_view = nullptr;   // view whose world pointer we replaced (game thread only, plus rescue)
void* g_world_saved = nullptr;  // its original world
std::atomic<bool> g_world_restore_now{false};
std::atomic<bool> g_shift_prev{true};  // also shift the previous-frame camera description (motion vectors)
// Private camera for the created view (default): a copy of the main view's camera object, both descriptions
// (this frame and previous frame) shifted, written before StartFrame so the engine builds every per-frame camera
// of that view itself. "slotshift" in the control file uses the older after-the-fact slot rewrite instead.
std::atomic<bool> g_priv_mode{true};
unsigned char* g_priv_cam = nullptr;  // engine-allocated 0x900 bytes, never freed
void* g_created_main_cam = nullptr;    // main view camera object when the second view was built
std::atomic<std::uint64_t> g_priv_updates{0}, g_priv_failed{0};

// Head tracking ("head" in the control file, needs a running OpenXR session): both eyes are rendered from the game
// camera plus that eye's headset pose relative to a recentre reference. Left eye = main view (private camera too),
// right eye = created view. The previous-frame description uses the previous frame's head pose (motion vectors).
std::atomic<bool> g_head{false};
unsigned char* g_priv_left = nullptr;  // main view's private camera in head mode (engine-allocated, never freed)
bool g_head_ref_valid = false;          // game thread
std::atomic<bool> g_recentre_request{false};  // set by the mod menu (any thread), consumed on the game thread
float g_ref_q[4] = {0, 0, 0, 1};        // yaw-only reference orientation (x,y,z,w)
float g_ref_p[3] = {};
// Roomscale rebase (move_probe "roomscale", default off): world-unit body moves queued by the movement update (any
// thread); the next head step shifts g_ref_p by the same amount so the eyes stay where the headset is while the body
// moves under them. Empty queue = no effect.
std::mutex g_rebase_mutex;
float g_rebase[3] = {};
float g_rebase_yaw = 0.f;  // engine-convention radians the game look yaw was turned by (body follows head yaw)
std::atomic<std::uint64_t> g_rebase_applied{0};
std::atomic<float> g_head_yaw_rel{0.f};      // headset yaw minus game camera yaw, engine convention (+ = right)
std::atomic<ULONGLONG> g_head_yaw_tick{0};
std::atomic<std::uint64_t> g_head_yaw_seq{0};  // head steps that published g_head_yaw_rel
float g_last_m[2][9];                   // last applied engine-space rotation per eye (row-major 3x3)
float g_last_o[2][3];                    // last applied engine-space offset per eye
bool g_last_valid[2] = {false, false};
std::atomic<std::uint64_t> g_head_frames{0}, g_head_nopose{0};
// Field-of-view experiment (head mode): "fov <deg>" writes camera-description +0x100 (radians), "offx <v>" / "offy <v>"
// write the projection centre offsets +0x108 / +0x10C, in both eyes' private cameras (this and previous frame).
std::atomic<int> g_fov_mdeg{0};      // 0 = leave the game's value
std::atomic<int> g_offx_milli{0}, g_offy_milli{0};
std::atomic<bool> g_off_set{false};
// Culling experiment: "cullfov <deg>" widens the GAME camera object's field of view (both descriptions) every frame
// before StartFrame, to test whether culling follows the game camera instead of the head-tracked eye cameras.
std::atomic<int> g_cullfov_mdeg{0};
// "autofov [margin%]": each eye camera gets the projection that exactly covers the headset's FOV at the game's
// render aspect (camera desc +0x104): horizontal FOV (+0x100) and the off-axis centre (+0x108/+0x10C via the
// engine's setter). "eyeshift": each eye is centred on its own (canted) frustum; without it both eyes use the
// symmetric union of the two. The rendered tangents travel with the pose snapshot to the projection layer.
std::atomic<bool> g_auto_fov{false};
std::atomic<bool> g_eye_shift{false};
std::atomic<bool> g_zero_sep{false};  // "zerosep": test, both eye cameras at the centre point
std::atomic<bool> g_zero_sep_forced{false};  // set by the foliage sweep for its duration
std::atomic<int> g_auto_margin_pct{0};
// Matched projection narrowed to a share of the headset FOV ("narrow N", percent, 100 = full): the same render pixels
// cover fewer degrees (higher pixels per degree, sharper), the lens edges go black. "outeredge" (with per-eye
// centering): each eye keeps its outer (temporal) edge and gives up its inner (nasal) side, which the other eye still
// sees in the binocular overlap. Both cameras still look straight ahead -- the frustum is shifted, not turned -- so
// view-dependent effects (reflections, shadows) stay consistent between the eyes. [AUTHOR] 2026-10-07 idea from a
// Cyberpunk 2077 VR user who turned the cameras outward instead (toe-out) and lost reflection/shadow matching.
std::atomic<int> g_narrow_pct{100};
std::atomic<bool> g_outer_edge{false};
// "vcrop P" (2026-11-11, ISSUE-05): keep P % of the headset's vertical FOV (tangents, top and bottom scaled alike, so
// the larger lower half gives up more), 100 = off. The menu lowers the eye height by the same share, so the image
// aspect and the frustum stay matched: horizontal coverage and pixel density are unchanged, the rows the lens shows
// least are not rendered. [INFERENCE] With the eye height unchanged it does nothing (matched zoom keeps the aspect).
std::atomic<int> g_vcrop_pct{100};
// "symproj" (ISSUE-04 test, 2026-10-21): each eye renders an on-axis frustum (m8 = m9 = 0) that covers its own
// off-centre window, and the XR layer is labelled with that wider frustum. Discriminates screen-space/lighting passes
// that assume the image centre is the view direction (the Cyberpunk VR port needed shader twins for those) from
// per-view state differences. Costs pixel density: the frustum grows to the larger tangent on each axis.
std::atomic<bool> g_sym_proj{false};
// UI distance ("uidepth M", metres; 0 = infinity). The game UI is drawn at the same pixels in both eyes, so its apparent
// depth is set by where each eye's image centre points. Per-eye centering (eyeshift) uses each eye's own canted
// frustum, whose centres point apart (Quest 3: tan -0.255 / +0.255): the UI diverges and reads double. The symmetric
// union frustum puts it at infinity. Here, without eyeshift, each eye's frustum centre is turned inward by
// atan(ipd/2 / M) (and widened by the same amount so the eye's FOV stays covered); the layer labels carry the real
// frustum, so the scene stays correct and only the UI moves to distance M.
std::atomic<int> g_ui_depth_mm{2000};
// UI placement per eye ("uiplace SIZE HEIGHT": size in percent, height in degrees; "nouiplace" = off). The UI is drawn
// inside each eye's image at the same pixels; instead of turning the frusta (above, needs per-eye centering off),
// camera.cpp moves and scales the UI's viewport/scissor in each eye's UI pass so it sits straight ahead at UI distance.
// The frusta are then left as they are (no turn-in), so per-eye centering can stay on.
std::atomic<bool> g_ui_place{true};
std::atomic<int> g_ui_size_pct{85};
std::atomic<int> g_ui_height_cdeg{0};  // centidegrees, + = up
// Latest rendered frustum per eye (left, right, up, down tangents) and the eye distance, from head_step.
std::mutex g_ui_tan_mutex;
float g_ui_tan[2][4] = {};
bool g_ui_tan_valid[2] = {};
float g_ui_ipd = 0.064f;
// Main menu panel ("menupanel DIST WIDTH HEIGHT" in metres; "nomenupanel" = off). [AUTHOR] 2026-10-08: the 3D main
// menu's UI should stay at a fixed place in the room, like the mod menu, not follow the head; tuned apart from the HUD.
// First try (billboard: the UI moved per eye toward a fixed point) [AUTHOR]: "fixed on the same position, but it does
// pitch and rotates following the headset" -- a viewport can move and scale the UI but not tilt it. Now, while the
// front end runs (see update_front_end), the main view's UI pass draws into a texture of ours (camera.cpp), the
// second view's UI is drawn off its target, and xr_runtime shows the texture as a real OpenXR quad layer fixed in the
// room (placed DIST in front of the head, HEIGHT above head height, when the front end starts or on Recenter).
std::atomic<bool> g_menu_panel{true};
std::atomic<int> g_menu_dist_mm{2000};
std::atomic<int> g_menu_width_mm{1800};
std::atomic<int> g_menu_height_mm{0};
// Front end = everything before the first load after the 3D main menu (update_front_end, game thread).
std::atomic<bool> g_front_end{true};
std::atomic<bool> g_lens_projection{false};  // full runtime FOV; no window or resolution geometry
std::atomic<int> g_view_scale_pct{100};
// First-person arm projection (FEATURE-03, 2026-10-14). [LIVE] In stereo a 1536-byte camera block at the eye position
// keeps a fixed 64.23 deg projection (the flat FP block: 38.89 deg vertical at 16:9 = 64.23 deg horizontal) while the eye
// blocks follow the headset (91.75 deg); [AUTHOR] only the FP arms look camera-attached/FOV-scaled, the full body does
// not. Each eye's expected world projection (m0, m5 from the requested tangents) identifies its world block; a narrower
// perspective block with the same view rows on the same eye's Render thread gets that eye's projection-dependent rows.
// Mod menu "Arm projection fix" (default on) or "nofpproj" in the native control turns the write off (logging stays).
std::atomic<float> g_expect_m0[2], g_expect_m5[2];
bool g_fp_proj_menu = true;  // mod menu "Arm projection fix" (Present thread); combined with "nofpproj" in poll
std::atomic<bool> g_fp_proj_nofile{true};  // false while AvatarVR_native.txt contains "nofpproj"
bool fp_proj_enabled() { return g_fp_proj_menu && g_fp_proj_nofile.load(std::memory_order_relaxed); }
// Experiment (2026-10-15, FP arm depth priority): [LIVE] no viewport uses a narrowed depth range (867k viewports), but
// the FP block holds (0, 1) at +0x4F0/+0x4F4 where the eye block holds (0.5, -0.25). INFERENCE: a depth scale/bias that
// puts every arm pixel at the nearest reverse-Z depth. Menu "Arm depth test (experimental)" also copies the eye's pair.
bool g_fp_depth_menu = false;  // [LIVE] 10:09 applied (written 2420) with no visible effect: not the depth mechanism
std::atomic<bool> g_fp_depth_file{false};  // "fpdepth" in AvatarVR_native.txt
bool g_fp_occlude_menu = true;               // menu "Arm depth occlusion" (camera.cpp PSO depth-func swap for FP arm draws)
std::atomic<bool> g_fp_occlude_file{false};  // "fpocclude" in AvatarVR_native.txt forces it on; "nofpocclude" off
std::atomic<bool> g_fp_occlude_off{false};
std::atomic<bool> g_pose_write{false};  // "posewrite" in AvatarVR_native.txt: IK hands into the game's own poses (experiment)
bool occlude_enabled_impl() {
  return !g_fp_occlude_off.load(std::memory_order_relaxed) && (g_fp_occlude_menu || g_fp_occlude_file.load(std::memory_order_relaxed));
}
std::atomic<unsigned> g_fp_depth_written{0};
struct AutoProj {
  float aspect = 0.f;  // optical aspect in lens mode; legacy mode uses the target aspect
  float fov_rad;  // horizontal FOV for desc +0x100
  float m8, m9;   // projection centre (NDC) for the offset setter
  float tan4[4];  // resulting frustum: left (<0), right, up, down (<0)
};
// Projection for `eye` of `s` at render aspect `aspect` (width / height). The engine derives tanV = tanH / aspect.
bool auto_projection(const head_pose::Snapshot& s, int eye, float aspect, AutoProj* out) {
  if (g_lens_projection.load(std::memory_order_relaxed)) {
    lens_projection::Projection p = {};
    if (!lens_projection::from_fov(s.eye[eye].fov, &p)) return false;
    out->aspect = p.aspect;
    out->fov_rad = p.fov_rad;
    out->m8 = p.m8; out->m9 = p.m9;
    std::memcpy(out->tan4, p.tan4, sizeof(p.tan4));
    return true;
  }
  if (!(aspect > 0.3f && aspect < 4.f)) {
    return false;
  }
  float hl = 0.f, hr = 0.f, vu = 0.f, vd = 0.f;
  for (int e = 0; e < 2; ++e) {
    const float* f = s.eye[e].fov;
    vu = std::max(vu, std::tan(f[2]));
    vd = std::max(vd, std::tan(-f[3]));
  }
  const bool test_same = g_zero_sep.load(std::memory_order_relaxed) || g_zero_sep_forced.load(std::memory_order_relaxed);  // test: identical frustum in both eyes
  const int narrow = g_narrow_pct.load(std::memory_order_relaxed);
  const float k = !test_same && narrow >= 40 && narrow < 100 ? static_cast<float>(narrow) * 0.01f : 1.f;
  vu *= k;
  vd *= k;
  if (const int vc = g_vcrop_pct.load(std::memory_order_relaxed); !test_same && vc >= 60 && vc < 100) {
    vu *= static_cast<float>(vc) * 0.01f;
    vd *= static_cast<float>(vc) * 0.01f;
  }
  if (g_eye_shift.load(std::memory_order_relaxed) && !test_same) {
    hl = std::tan(-s.eye[eye].fov[0]);
    hr = std::tan(s.eye[eye].fov[1]);
    if (k < 1.f) {
      if (g_outer_edge.load(std::memory_order_relaxed)) {
        // keep the outer edge, take the whole cut from the nasal side (left eye: right side; right eye: left side)
        const float total = (hl + hr) * k;
        float& outer = eye == 0 ? hl : hr;
        float& inner = eye == 0 ? hr : hl;
        outer = std::min(outer, total - 0.05f);
        inner = total - outer;
      } else {
        hl *= k;
        hr *= k;
      }
    }
  } else {
    for (int e = 0; e < 2; ++e) {
      hl = std::max(hl, std::max(std::tan(-s.eye[e].fov[0]), std::tan(s.eye[e].fov[1])));
    }
    hl *= k;
    hr = hl;
    const int depth_mm = g_ui_depth_mm.load(std::memory_order_relaxed);
    if (depth_mm > 0 && !test_same && !g_ui_place.load(std::memory_order_relaxed)) {
      const float dx = s.eye[1].px - s.eye[0].px, dy = s.eye[1].py - s.eye[0].py, dz = s.eye[1].pz - s.eye[0].pz;
      float ipd = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (!(ipd > 0.04f && ipd < 0.09f)) {
        ipd = 0.064f;
      }
      const float c = 0.5f * ipd / (static_cast<float>(depth_mm) * 1e-3f);  // inward: left eye right (+), right eye left (-)
      const float sc = eye == 0 ? c : -c;
      // tangents [sc - (U + c), sc + (U + c)] still cover [-U, U]
      hl = hl + c - sc;
      hr = hr + c + sc;
    }
  }
  if (g_sym_proj.load(std::memory_order_relaxed)) {
    hl = hr = std::max(hl, hr);
    vu = vd = std::max(vu, vd);
  }
  if (!(hl + hr > 0.2f && vu + vd > 0.2f && hl + hr < 20.f && vu + vd < 20.f)) {
    return false;
  }
  const float margin = 1.f + static_cast<float>(g_auto_margin_pct.load(std::memory_order_relaxed)) * 0.01f;
  const float scale = static_cast<float>(g_view_scale_pct.load(std::memory_order_relaxed)) * 0.01f;
  const float hh = 0.5f * (hl + hr), cx = 0.5f * (hr - hl) * scale;
  const float hv = 0.5f * (vu + vd), cy = 0.5f * (vu - vd) * scale;
  const float th = std::max(hh, aspect * hv) * margin * scale;  // matched zoom; no pixel stretching
  const float tv = th / aspect;
  out->fov_rad = 2.f * std::atan(th);
  out->m8 = -cx / th;  // frustum centre = -m8 * tanH
  out->m9 = -cy / tv;
  out->tan4[0] = (-1.f - out->m8) * th;
  out->tan4[1] = (1.f - out->m8) * th;
  out->tan4[2] = (1.f - out->m9) * tv;
  out->tan4[3] = (-1.f - out->m9) * tv;
  return out->fov_rad > 0.2f && out->fov_rad < 3.1f;
}
bool read_aspect(const void* cam, float* aspect) {
  __try {
    std::memcpy(aspect, static_cast<const unsigned char*>(cam) + 0x104, sizeof(float));
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
std::atomic<float> g_render_aspect{0.f};  // backbuffer width / height (0 = unknown)
std::atomic<unsigned> g_render_w{0}, g_render_h{0};  // backbuffer size (log only)
// Per-eye render size ("eyesize W H [both]"; 0 = follow the window). See OutSize_hook / apply_eye_rect.
std::atomic<std::uint32_t> g_eyesize_w{0}, g_eyesize_h{0};
std::atomic<bool> g_eyesize_both{false};
std::atomic<std::uint64_t> g_auto_applied{0}, g_auto_failed{0};
std::atomic<int> g_auto_last_fov_mdeg{0}, g_auto_last_aspect_milli{0};
std::atomic<std::uint64_t> g_cullfov_writes{0};

bool widen_camera(void* cam, float rad) {
  __try {
    unsigned char* c = static_cast<unsigned char*>(cam);
    const std::uint32_t dirty = 1;
    for (std::uint32_t d : {0u, 0x120u}) {
      std::memcpy(c + d + 0x100, &rad, sizeof(rad));
      std::memcpy(c + d + 0xE0, &dirty, sizeof(dirty));
    }
    std::memcpy(c + 0x270, &dirty, sizeof(dirty));
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
// Ring of the snapshots applied at each StartFrame (for the layer poses at Present).
constexpr int kPoseRing = 8;
head_pose::Snapshot g_applied[kPoseRing];
std::uint64_t g_applied_count = 0;
float g_applied_cam[kPoseRing][16] = {};  // base game camera of each applied pose; g_applied_mutex
float g_step_cam[16] = {};                // game thread, set by compute_aim during the head step
std::mutex g_applied_mutex;
// Pose identity (trace 2026-10-05 13:01): view #0's per-eye camera build runs BEFORE Gfx_ViewManager::StartFrame on the
// game thread, the created view's inside it. Head tracking used to run in the StartFrame pre-hook, so the left eye
// (view #0) rendered the previous frame's pose and the right eye the current one. Now the head step runs at the first
// of the two builds (g_head_prestepped tells StartFrame it is done), and each view's frame slot remembers the applied
// pose it was built with; the view-2 output callback picks up the pose of the slot it outputs.
bool g_head_prestepped = false;                       // game thread only
std::uint64_t g_slot_pose[2][8] = {};                 // [main, created][slot] -> applied index + 1 (0 = none); g_applied_mutex
std::atomic<std::uint64_t> g_out_pose{0};             // applied index + 1 of the latest view-2 output
std::atomic<std::uint64_t> g_identity_used{0}, g_identity_missing{0}, g_identity_split{0};
// Game frustum tangents from the latest main-camera projection seen in a 1536-byte camera block.
std::atomic<float> g_proj_m0{0.f}, g_proj_m5{0.f}, g_proj_m8{0.f}, g_proj_m9{0.f};
// Read-only projection candidates from the existing CBV observer, attributed
// by its Render thread. Not proof of a unique camera until compared live.
std::atomic<float> g_observed_projection[2][4];
std::atomic<std::uint64_t> g_observed_projection_count[2];

// Real second view (user-approved 2026-10-02 20:39): a Gfx_View built by the engine's own constructor for the main
// world and camera, registered with Gfx_ViewManager::AddView, created on the game thread at the start of StartFrame.
constexpr std::uintptr_t kStartFrameRva = 0x5A2520;
constexpr unsigned char kStartFrameSig[] = {0x40, 0x53, 0x57, 0x41, 0x55, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x28, 0x48};
constexpr std::uintptr_t kAllocRva = 0x7C270;  // engine allocator(size): xor edx,edx; jmp ...
constexpr unsigned char kAllocSig[] = {0x33, 0xD2, 0xE9};
constexpr std::uintptr_t kCtorRva = 0x4FBC20;  // Gfx_View::Gfx_View(this, world, camera, params)
constexpr unsigned char kCtorSig[] = {0x48, 0x89, 0x5C, 0x24, 0x18, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55};
constexpr std::uintptr_t kAddViewRva = 0x51ECD0;  // Gfx_ViewManager::AddView(mgr, view)
constexpr unsigned char kAddViewSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x54, 0x24, 0x10, 0x57, 0x48};
constexpr std::uint32_t kViewSize = 0x9F50;
constexpr std::uint32_t kParamsOffset = 0x13B0;
constexpr std::uint32_t kParamsSize = 0x150;
using AllocFn = void* (*)(std::size_t);
using CtorFn = void* (*)(void*, void*, void*, void*);
AllocFn g_alloc = nullptr;
CtorFn g_ctor = nullptr;
Fn2 g_add_view = nullptr;
std::atomic<bool> g_want_create{false};
std::atomic<int> g_create_sep_um{32650};
void* g_created = nullptr;  // the engine view we built (never freed)
void* g_created_main = nullptr;
std::atomic<int> g_created_index{-1};
std::atomic<std::uint64_t> g_created_frames{0};
bool g_create_failed = false;
// Offscreen output for the created view: frame byte +0xE0A9 makes the output selector 0x14054EC20 pick the engine's
// offscreen target set ([0x1496A1B80]+0xC00) instead of the display; 0x1404EDAB0 sizes it from the eye camera and
// 0x140587CC0 uses its texture ([+0xC38]). The frame-reset 0x140524920 clears the byte, so it is set every frame.
constexpr std::uint32_t kOffscreenFlag = 0xE0A9;
constexpr std::uintptr_t kTargetsGlobalRva = 0x96A1B80;  // afop+0x96A1B80 -> render-target globals
std::atomic<bool> g_want_offscreen{false};
std::atomic<std::uint64_t> g_offscreen_sets{0};

// Offscreen output probe: 0x1404EDAB0(out, ?, ctx) is the per-view output callback of the render job 0x140587860.
// It ensures myMidComposite when frame[+0xE0A9] is set and fills `out` (0x70 bytes) through the output selector;
// out+0x28 is the engine texture object (width +0x40, height +0x44, depth +0x48, format byte +0x56), the same field
// the engine's eye capture copies from. Read-only: we look for the ID3D12Resource inside that object.
constexpr std::uintptr_t kOutputCbRva = 0x4EDAB0;
constexpr unsigned char kOutputCbSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18,
                                          0x57, 0x48, 0x83, 0xEC, 0x30, 0x48, 0x8B, 0xF9};
std::atomic<std::uint64_t> g_output_calls{0}, g_output_created{0};
std::atomic<void*> g_out_texture{nullptr};   // engine texture object seen for the created view
std::atomic<void*> g_out_resource{nullptr};  // ID3D12Resource found inside it
std::atomic<int> g_out_res_offset{-1};       // where: offset in the texture (or 0x1000 + inner offset)
std::atomic<std::uint32_t> g_out_w{0}, g_out_h{0}, g_out_d{0}, g_out_fmt{0};
std::atomic<void*> g_main_texture{nullptr};  // same field for the main view, for comparison
std::mutex g_out_mutex;                      // guards g_out_held
ID3D12Resource* g_out_held = nullptr;        // one reference on the latest created-view output resource
std::atomic<std::uint64_t> g_out_changes{0};

bool add_ref(void* resource) {
  __try {
    static_cast<ID3D12Resource*>(resource)->AddRef();
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

// The main view's output IS the swapchain backbuffer (display output path). Holding a reference to it made every
// ResizeBuffers fail with DXGI_ERROR_INVALID_CALL (run 2026-10-06 07:38: stereo already paused, still 0x887A0001),
// so it is only taken for one frame when an eye capture asks for it, and handed over to the capture.
ID3D12Resource* g_main_held = nullptr;  // guarded by g_out_mutex
std::atomic<bool> g_main_wanted{false};

void hold_main_output(void* resource) {
  ID3D12Resource* old = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_out_mutex);
    if (g_main_held == resource || !add_ref(resource)) {
      return;
    }
    old = g_main_held;
    g_main_held = static_cast<ID3D12Resource*>(resource);
  }
  if (old != nullptr) {
    old->Release();
  }
}

// Eye mode ("eyesize W H both"): the main view renders offscreen at the eye size too, so its output is an engine pool
// texture, not the backbuffer, and is held every frame like the second view's (released when eye mode ends).
// Off: runs 18:16-18:24 showed the main view's final image goes to the display whatever +0xE0A9 says (the code is
// kept for reference); eye mode grows the swapchain buffers instead (dx12.cpp grow_buffers).
constexpr bool kMainOffscreen = false;
ID3D12Resource* g_main_eye_held = nullptr;  // guarded by g_out_mutex
std::atomic<std::uint64_t> g_main_eye_changes{0}, g_main_eye_same_as_second{0};

void hold_main_eye(void* resource) {
  ID3D12Resource* old = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_out_mutex);
    if (g_main_eye_held == resource || !add_ref(resource)) {
      return;
    }
    old = g_main_eye_held;
    g_main_eye_held = static_cast<ID3D12Resource*>(resource);
  }
  g_main_eye_changes.fetch_add(1, std::memory_order_relaxed);
  if (old != nullptr) {
    old->Release();
  }
}

void drop_main_eye() {
  ID3D12Resource* old = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_out_mutex);
    old = g_main_eye_held;
    g_main_eye_held = nullptr;
  }
  if (old != nullptr) {
    old->Release();
  }
}

void hold_output(void* resource) {
  ID3D12Resource* old = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_out_mutex);
    if (g_out_held == resource) {
      return;
    }
    if (!add_ref(resource)) {
      return;
    }
    old = g_out_held;
    g_out_held = static_cast<ID3D12Resource*>(resource);
  }
  g_out_changes.fetch_add(1, std::memory_order_relaxed);
  if (old != nullptr) {
    old->Release();
  }
}

using RenderFn = std::uintptr_t (*)(void*, void*, void*, void*);
using CaptureFn = std::uintptr_t (*)(void*, void*, void*, void*);
RenderFn g_render = nullptr;
CaptureFn g_capture = nullptr;
RenderFn g_stereo = nullptr;
RenderFn g_wait = nullptr;
RenderFn g_composite = nullptr;
RenderFn g_cam_build = nullptr;
RenderFn g_start_frame = nullptr;
RenderFn g_output_cb = nullptr;
RenderFn g_ui_pass = nullptr;
RenderFn g_ui_extra7 = nullptr;
RenderFn g_ui_extra9 = nullptr;
std::atomic<std::uint64_t> g_wait_in{0}, g_wait_out{0};
// Waits currently in progress: who called them (return address), on which view, with which flag, since when.
constexpr int kPendingMax = 32;
struct PendingWait {
  std::atomic<DWORD> tid{0};
  void* caller = nullptr;
  void* view = nullptr;
  void* flag = nullptr;
  ULONGLONG since = 0;
};
PendingWait g_pending[kPendingMax];
unsigned char* g_module_base = nullptr;
std::atomic<std::uint64_t> g_composite_in{0}, g_composite_out{0};
RenderFn g_add_task = nullptr;
RenderFn g_gate = nullptr;
RenderFn g_submit = nullptr;
std::atomic<std::uint64_t> g_add_new[4], g_add_reuse[4];  // per kind: gate created vs job appended to an existing gate
std::atomic<std::uint64_t> g_gate_in[4], g_gate_out[4];
std::atomic<std::uint64_t> g_submit_in{0}, g_submit_out{0};
std::atomic<bool> g_want_kick{false};
std::atomic<std::uint64_t> g_kicks{0};
// Progress counters for hang diagnosis: entered/exited per function; Render split by the eye-pass guard (0/1/2).
std::atomic<std::uint64_t> g_render_in[3], g_render_out[3];
std::atomic<std::uint64_t> g_stereo_in{0}, g_stereo_out{0};
std::atomic<std::uint64_t> g_capture_out{0};
std::atomic<std::uint64_t> g_presents{0};
std::atomic<bool> g_heartbeat_started{false};
std::atomic<bool> g_rescued{false};
std::atomic<bool> g_installed{false};

constexpr int kMaxViews = 16;
struct ViewInfo {
  void* view = nullptr;
  std::atomic<std::uint64_t> renders{0};
  std::atomic<std::uint64_t> stereo_entries{0};  // renders that saw the mode set and the guard clear
  bool written = false;
  std::uint32_t saved_mode = 0;
  // Camera blocks (1536-byte CBVs with a perspective projection) created while this view's Render runs.
  std::atomic<std::uint64_t> cams{0};
  std::atomic<std::int32_t> fovy_milli{0};
  std::atomic<std::int32_t> aspect_milli{0};
  std::atomic<std::int32_t> pos_x{0}, pos_y{0}, pos_z{0};  // centimetres
  std::atomic<DWORD> render_tid{0};
};
thread_local int t_view_index = -1;  // view whose Render is running on this thread
// Gfx_View::Render calls in progress on any thread, by kind (0 main view #0, 1 the created second view, 2 any other
// view: cube views, start-up views). Read by the GPU submission profiler (dx12.cpp) to tag each ExecuteCommandLists.
std::atomic<int> g_rendering[3];
std::atomic<std::uint64_t> g_cams_unattributed{0};
ViewInfo g_views[kMaxViews];
std::atomic<int> g_view_count{0};
std::mutex g_mutex;  // view registration and writes

// ---- per-view cost census (read-only; for sharing eye-independent work between the two views) ----------------
// Gfx_View::Render time per view (its own thread), phase-gate time per view and kind (worker threads; the gate's
// graph argument is matched to the view whose 0x9F50-byte object contains it, the offset is logged), and the jobs
// each view queues through "add view task" (ctx = view+0x49F0), identified by the first pointer into afop.exe found
// in the job's first 0x40 bytes (job layout OPEN; the identity is a census key, not a claimed function).
constexpr std::uintptr_t kViewObjectSize = 0x9F50;
LARGE_INTEGER g_qpf = {};
std::atomic<std::uint64_t> g_render_ticks[kMaxViews];
std::atomic<std::uint64_t> g_render_calls[kMaxViews];
std::atomic<std::uint64_t> g_gate_ticks[kMaxViews][4];
std::atomic<std::uint64_t> g_gate_calls[kMaxViews][4];
std::atomic<std::uint64_t> g_gate_unmatched{0};
std::atomic<std::uint64_t> g_gate_kind_ticks[4];   // all views: worker CPU time inside phase gates
std::atomic<std::uint64_t> g_gate_kind_calls[4];
std::atomic<std::uint64_t> g_wait_ticks[kMaxViews];  // time blocked in the Gfx_View task wait, by view
std::atomic<std::uint64_t> g_wait_calls[kMaxViews];
std::atomic<std::uint64_t> g_wait_other_ticks{0};
std::atomic<std::int64_t> g_gate_offset[kMaxViews];  // graph - view, first seen
std::atomic<std::uint64_t> g_task_calls[kMaxViews][4];
std::size_t g_image_size = 0;
struct JobKey {
  std::uint32_t rva;
  std::uint16_t at;  // byte offset of that pointer inside the job
  std::uint16_t kind;
  std::uint64_t count[kMaxViews];
  std::uintptr_t sample[8];  // the first job seen with this key (first 0x40 bytes)
};
constexpr int kMaxJobKeys = 96;
JobKey g_job_keys[kMaxJobKeys];
int g_job_key_count = 0;
std::mutex g_job_mutex;

// ---- frame pipeline trace (read-only): order of StartFrame, per-eye camera build, Render, view-2 output and Present
// across threads, with each view's frame-ring slots (+0x9EC4 = slot the game thread fills, +0x9EC8 = OPEN, logged
// to identify the slot a Render consumes). Purpose: tie each presented image to the pose it was rendered from.
struct TraceEvent {
  LONGLONG qpc;
  char type;   // S startframe (a = applied pose count), C cam build, R render begin, E render end, O view-2 output, P present
  std::int8_t view;
  std::uint16_t tid;
  std::uint32_t a, b;
};
constexpr int kTraceSize = 512;
TraceEvent g_trace[kTraceSize];
std::atomic<std::uint32_t> g_trace_next{0};

void trace(char type, int view, std::uint32_t a, std::uint32_t b) {
  const std::uint32_t i = g_trace_next.fetch_add(1, std::memory_order_relaxed) % kTraceSize;
  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);
  g_trace[i] = TraceEvent{now.QuadPart, type, static_cast<std::int8_t>(view), static_cast<std::uint16_t>(GetCurrentThreadId()), a, b};
}

bool read_u32(const void* address, std::uint32_t* out);
void view_slots(void* view, std::uint32_t* game, std::uint32_t* other) {
  *game = 0xFFFFFFFF;
  *other = 0xFFFFFFFF;
  if (view != nullptr) {
    read_u32(static_cast<unsigned char*>(view) + kFrameIdxGame, game);
    read_u32(static_cast<unsigned char*>(view) + kFrameIdxGame + 4, other);
  }
}

int view_containing(const void* p) {
  const auto a = reinterpret_cast<std::uintptr_t>(p);
  const int count = g_view_count.load(std::memory_order_acquire);
  for (int i = 0; i < count; ++i) {
    const auto v = reinterpret_cast<std::uintptr_t>(g_views[i].view);
    if (v != 0 && a >= v && a < v + kViewObjectSize) {
      return i;
    }
  }
  return -1;
}

std::atomic<int> g_want_mode{0};  // 0 = observe
std::atomic<int> g_want_view{0};
std::atomic<std::uint64_t> g_captures{0};
std::atomic<std::uint64_t> g_faults{0};

std::wstring g_control_path;
std::wstring g_log_path;
std::wstring g_dump_path, g_dump_request;  // CODEDUMP.log and its request file

void log_line(const char* fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  if (g_log_path.empty()) {
    return;
  }
  SYSTEMTIME t = {};
  GetLocalTime(&t);
  char stamp[24];
  snprintf(stamp, sizeof(stamp), "%02u:%02u:%02u ", t.wHour, t.wMinute, t.wSecond);
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, stamp, static_cast<DWORD>(std::strlen(stamp)), &written, nullptr);
  WriteFile(file, buffer, static_cast<DWORD>(std::strlen(buffer)), &written, nullptr);
  WriteFile(file, "\r\n", 2, &written, nullptr);
  CloseHandle(file);
}

std::wstring self_dir() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&self_dir), &self)) {
    return {};
  }
  wchar_t path[MAX_PATH] = {};
  const DWORD n = GetModuleFileNameW(self, path, MAX_PATH);
  if (n == 0 || n >= MAX_PATH) {
    return {};
  }
  std::wstring dir(path, n);
  const auto slash = dir.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : dir.substr(0, slash);
}

// SEH helpers (plain C types only in these frames).
bool read_u32(const void* address, std::uint32_t* out) {
  __try {
    *out = *static_cast<const volatile std::uint32_t*>(address);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
bool read_u8(const void* address, std::uint8_t* out) {
  __try {
    *out = *static_cast<const volatile std::uint8_t*>(address);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
bool read_bytes(const void* src, void* dst, size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
bool write_bytes(void* dst, const void* src, size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
bool write_u32(void* address, std::uint32_t value) {
  __try {
    *static_cast<volatile std::uint32_t*>(address) = value;
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

int view_index(void* view) {
  const int count = g_view_count.load(std::memory_order_acquire);
  for (int i = 0; i < count; ++i) {
    if (g_views[i].view == view) {
      return i;
    }
  }
  std::lock_guard<std::mutex> lock(g_mutex);
  const int now = g_view_count.load();
  for (int i = 0; i < now; ++i) {
    if (g_views[i].view == view) {
      return i;
    }
  }
  if (now >= kMaxViews) {
    return -1;
  }
  g_views[now].view = view;
  g_view_count.store(now + 1, std::memory_order_release);
  std::uint32_t mode = 0;
  read_u32(static_cast<unsigned char*>(view) + kModeOffset, &mode);
  log_line("new Gfx_View #%d at %p (mode field +0x1458 = %u)", now, view, mode);
  return now;
}

DWORD WINAPI heartbeat(void*);

bool __try_write_u8(void* address, std::uint8_t value) {
  __try {
    *static_cast<volatile std::uint8_t*>(address) = value;
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

void note_job(int view, int kind, const void* job) {
  if (job == nullptr || g_module_base == nullptr || g_image_size == 0) {
    return;
  }
  std::uintptr_t words[8] = {};
  if (!read_bytes(job, words, sizeof(words))) {
    return;
  }
  const auto base = reinterpret_cast<std::uintptr_t>(g_module_base);
  std::uint32_t rva = 0;
  std::uint16_t at = 0xFFFF;
  for (int k = 0; k < 8; ++k) {
    if (words[k] >= base && words[k] < base + g_image_size) {
      rva = static_cast<std::uint32_t>(words[k] - base);
      at = static_cast<std::uint16_t>(k * 8);
      break;
    }
  }
  std::lock_guard<std::mutex> lock(g_job_mutex);
  for (int i = 0; i < g_job_key_count; ++i) {
    JobKey& key = g_job_keys[i];
    if (key.rva == rva && key.at == at && key.kind == kind) {
      ++key.count[view];
      return;
    }
  }
  if (g_job_key_count < kMaxJobKeys) {
    JobKey& key = g_job_keys[g_job_key_count++];
    key = JobKey{rva, at, static_cast<std::uint16_t>(kind), {}, {}};
    std::memcpy(key.sample, words, sizeof(words));
    ++key.count[view];
  }
}

int find_view(void* view) {
  const int count = g_view_count.load(std::memory_order_acquire);
  for (int i = 0; i < count; ++i) {
    if (g_views[i].view == view) {
      return i;
    }
  }
  return -1;
}

void start_heartbeat() {
  if (!g_heartbeat_started.exchange(true)) {
    HANDLE thread = CreateThread(nullptr, 0, &heartbeat, nullptr, 0, nullptr);
    if (thread != nullptr) {
      CloseHandle(thread);
    }
  }
}

bool read_ptr(const void* address, void** out) {
  __try {
    *out = *static_cast<void* const volatile*>(address);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
bool write_ptr(void* address, void* value) {
  __try {
    *static_cast<void* volatile*>(address) = value;
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

// Rebuild eye slot 0 of `view` from its camera moved `sep` metres along the camera's right axis. Same sequence as
// the engine's own stereo eye code in 0x1405BCC40 (copy camera, copy desc, move matrix row 3 and the large-world
// position, set both back, rebuild), minus the convergence shift (parallel eyes). POD buffers only (SEH frame).
// Shift one camera description (0x120 bytes: matrix rows 0..3, right = row 0, position = row 3, large-world
// position at +0xE4) sideways by `sep` metres along its own right axis, the way the engine's eye code does.
void shift_desc(unsigned char* desc, float sep) {
  alignas(16) float mat[16];
  alignas(16) unsigned char wp[0x20];
  alignas(16) float off[4];
  std::memset(wp, 0, sizeof(wp));
  std::memcpy(mat, desc, sizeof(mat));
  off[0] = mat[0] * sep;
  off[1] = mat[1] * sep;
  off[2] = mat[2] * sep;
  off[3] = 0.f;
  mat[12] += off[0];
  mat[13] += off[1];
  mat[14] += off[2];
  std::memcpy(wp, desc + 0xE4, 0x12);
  reinterpret_cast<Fn2>(g_helpers[kHWpAdd].fn)(wp, off);
  reinterpret_cast<Fn2>(g_helpers[kHSetMatrix].fn)(desc, mat);
  reinterpret_cast<Fn2>(g_helpers[kHSetWp].fn)(desc, wp);
}

bool apply_shift(void* view, float sep) {
  // Each of the five sub-cameras of eye slot 0 is shifted from its OWN current contents: the engine gives them
  // different viewports (sub-cameras +0x1160/+0x1A10 carry the dynamic render-resolution rect that DLSS and the
  // render targets use), so rebuilding them all from [view+0x10] (full output rect) broke render size and DLSS mode.
  // Both descriptions are shifted: +0x0 (this frame) and +0x120 (previous frame, used for motion vectors). Shifting
  // only the current one made every frame look like a 3 cm camera jump to DLSS/TAA (streaky, sandy vegetation).
  alignas(16) unsigned char cam[0x900];
  alignas(16) unsigned char desc[0x140];
  alignas(16) unsigned char prev[0x140];
  __try {
    unsigned char* v = static_cast<unsigned char*>(view);
    unsigned char* ring = *reinterpret_cast<unsigned char* const*>(v + kFrameRing);
    const std::uint32_t idx = *reinterpret_cast<const std::uint32_t*>(v + kFrameIdxGame);
    if (ring == nullptr || idx > 7) {
      return false;
    }
    unsigned char* frame = ring + static_cast<std::uintptr_t>(idx) * kFrameStride;
    for (std::uint32_t sub : kSubCams) {
      unsigned char* slot = frame + sub;
      std::memset(cam, 0, sizeof(cam));
      std::memset(desc, 0, sizeof(desc));
      std::memset(prev, 0, sizeof(prev));
      reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(cam, slot);
      reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(desc, cam);
      reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(prev, cam + 0x120);
      shift_desc(desc, sep);
      if (g_shift_prev.load(std::memory_order_relaxed)) {
        shift_desc(prev, sep);
      }
      reinterpret_cast<Fn3>(g_helpers[kHBuild].fn)(cam, desc, prev);
      reinterpret_cast<Fn2>(g_helpers[kHAssign].fn)(slot, cam);
    }
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

// Game thread (inside StartFrame, before the per-world view lists are built): world-pointer swap and restore.
void world_step(void* view, int index) {
  const bool want = g_second_world.load() && !g_rescued.load() && index == g_second_view.load();
  if (g_world_view != nullptr && (g_world_restore_now.exchange(false) || !g_second_world.load() || g_rescued.load() ||
                                  g_views[g_second_view.load() < 0 ? 0 : g_second_view.load()].view != g_world_view)) {
    write_ptr(static_cast<unsigned char*>(g_world_view) + kWorldOffset, g_world_saved);
    log_line("second: restored world of %p to %p", g_world_view, g_world_saved);
    g_world_view = nullptr;
    g_world_saved = nullptr;
  }
  if (want && g_world_view == nullptr) {
    const int src_index = g_second_src.load();
    void* src_world = nullptr;
    void* own_world = nullptr;
    if (src_index < 0 || src_index >= g_view_count.load() || src_index == index ||
        !read_ptr(static_cast<unsigned char*>(g_views[src_index].view) + kWorldOffset, &src_world) ||
        !read_ptr(static_cast<unsigned char*>(view) + kWorldOffset, &own_world) || src_world == nullptr) {
      return;
    }
    if (write_ptr(static_cast<unsigned char*>(view) + kWorldOffset, src_world)) {
      g_world_view = view;
      g_world_saved = own_world;
      log_line("second: view #%d world %p -> %p (view #%d's world)", index, own_world, src_world, src_index);
      start_heartbeat();
    }
  }
}

// SEH frame: build the view (POD buffers only).
void* build_view(void* main) {
  alignas(16) unsigned char params[kParamsSize + 0x10];
  __try {
    unsigned char* m = static_cast<unsigned char*>(main);
    void* world = *reinterpret_cast<void* const*>(m + kWorldOffset);
    void* camera = *reinterpret_cast<void* const*>(m + kCameraOffset);
    if (world == nullptr || camera == nullptr) {
      return nullptr;
    }
    std::memcpy(params, m + kParamsOffset, kParamsSize);
    *reinterpret_cast<std::uint32_t*>(params + 0xA8) = 0;  // stereo mode off
    void* mem = g_alloc(kViewSize);
    if (mem == nullptr) {
      return nullptr;
    }
    std::memset(mem, 0, kViewSize);
    void* view = g_ctor(mem, world, camera, params);
    unsigned char* v = static_cast<unsigned char*>(view);
    *reinterpret_cast<void**>(v + 0x18) = *reinterpret_cast<void* const*>(m + 0x18);  // owner, as the factory sets it
    v[0x27] = m[0x27];
    *reinterpret_cast<std::uint32_t*>(v + 0x1250) = *reinterpret_cast<const std::uint32_t*>(m + 0x1250);
    return view;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}

bool add_view(void* mgr, void* view) {
  __try {
    g_add_view(mgr, view);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

void sync_flags(void* view, bool on) {
  __try {
    unsigned char* v = static_cast<unsigned char*>(view);
    v[0x24] = on ? 1 : 0;
    v[0x25] = on ? 1 : 0;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
}

// Main view live = its Render count moved within the last few StartFrames (false while paused).
int g_live_streak = 0;  // consecutive StartFrames with the main view rendering (game thread only)
bool main_live() {
  static std::uint64_t last = 0;
  static int idle = 100;
  const std::uint64_t now = g_views[0].renders.load(std::memory_order_relaxed);
  idle = (now != last) ? 0 : (idle < 100 ? idle + 1 : idle);
  last = now;
  const bool live = idle < 4;
  g_live_streak = live ? (g_live_streak < 100000 ? g_live_streak + 1 : g_live_streak) : 0;
  return live;
}
constexpr int kCreateAfterLiveFrames = 120;  // ~4 s of continuous main-view rendering (skips loading screens)
std::atomic<std::uint64_t> g_retired{0}, g_rebuilds{0};
std::atomic<bool> g_rebuild_requested{false};
std::atomic<std::uint64_t> g_start_frames{0};

void quat_mul(const float* a, const float* b, float* o) {  // o = a*b, (x,y,z,w)
  const float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  const float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  const float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  const float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
  o[0] = x;
  o[1] = y;
  o[2] = z;
  o[3] = w;
}
void quat_rot(const float* q, const float* v, float* o) {  // rotate v by unit quaternion q
  const float qv[4] = {v[0], v[1], v[2], 0.f};
  const float qc[4] = {-q[0], -q[1], -q[2], q[3]};
  float t[4], r[4];
  quat_mul(q, qv, t);
  quat_mul(t, qc, r);
  o[0] = r[0];
  o[1] = r[1];
  o[2] = r[2];
}
void quat_to_m(const float* q, float* m) {  // column-vector rotation matrix, row-major storage m[r*3+c]
  const float x = q[0], y = q[1], z = q[2], w = q[3];
  m[0] = 1 - 2 * (y * y + z * z);
  m[1] = 2 * (x * y - z * w);
  m[2] = 2 * (x * z + y * w);
  m[3] = 2 * (x * y + z * w);
  m[4] = 1 - 2 * (x * x + z * z);
  m[5] = 2 * (y * z - x * w);
  m[6] = 2 * (x * z - y * w);
  m[7] = 2 * (y * z + x * w);
  m[8] = 1 - 2 * (x * x + y * y);
}

// Engine-space eye transform for one OpenXR eye pose relative to the reference. Engine camera-local axes are
// x right, y up, z forward (row-vector, left-handed); OpenXR is x right, y up, -z forward: M_eng = S*M_xr*S,
// offset_eng = S*p_rel with S = diag(1,1,-1).
void eye_transform(const head_pose::Eye& e, float* m_eng, float* off_eng) {
  const float qe[4] = {e.qx, e.qy, e.qz, e.qw};
  const float qrc[4] = {-g_ref_q[0], -g_ref_q[1], -g_ref_q[2], g_ref_q[3]};
  float qrel[4];
  quat_mul(qrc, qe, qrel);
  const float dp[3] = {e.px - g_ref_p[0], e.py - g_ref_p[1], e.pz - g_ref_p[2]};
  float prel[3];
  quat_rot(qrc, dp, prel);
  float mx[9];
  quat_to_m(qrel, mx);
  static const float sgn[3] = {1.f, 1.f, -1.f};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      m_eng[r * 3 + c] = sgn[r] * sgn[c] * mx[r * 3 + c];
    }
  }
  off_eng[0] = prel[0];
  off_eng[1] = prel[1];
  off_eng[2] = -prel[2];
}

// Apply an eye transform to a camera description: new axis rows = M^T * rows, position += offset expressed in the
// camera's own axes; the large-world position gets the same world delta.
void transform_desc(unsigned char* desc, const float* m, const float* off) {
  alignas(16) float mat[16];
  alignas(16) unsigned char wp[0x20];
  alignas(16) float delta[4];
  std::memset(wp, 0, sizeof(wp));
  std::memcpy(mat, desc, sizeof(mat));
  float rows[3][3];
  for (int j = 0; j < 3; ++j) {
    rows[j][0] = mat[j * 4 + 0];
    rows[j][1] = mat[j * 4 + 1];
    rows[j][2] = mat[j * 4 + 2];
  }
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) {
      mat[i * 4 + k] = m[0 * 3 + i] * rows[0][k] + m[1 * 3 + i] * rows[1][k] + m[2 * 3 + i] * rows[2][k];
    }
  }
  for (int k = 0; k < 3; ++k) {
    delta[k] = off[0] * rows[0][k] + off[1] * rows[1][k] + off[2] * rows[2][k];
  }
  delta[3] = 0.f;
  mat[12] += delta[0];
  mat[13] += delta[1];
  mat[14] += delta[2];
  std::memcpy(wp, desc + 0xE4, 0x12);
  reinterpret_cast<Fn2>(g_helpers[kHWpAdd].fn)(wp, delta);
  reinterpret_cast<Fn2>(g_helpers[kHSetMatrix].fn)(desc, mat);
  reinterpret_cast<Fn2>(g_helpers[kHSetWp].fn)(desc, wp);
}

// FEATURE-06 stage A: move a camera description by a world delta (position and large-world position, as
// transform_desc does for its offset); rotation is untouched. Our private eye cameras only.
void translate_desc(unsigned char* desc, const float* d) {
  if (d[0] == 0.f && d[1] == 0.f && d[2] == 0.f) {
    return;
  }
  alignas(16) float mat[16];
  alignas(16) unsigned char wp[0x20];
  alignas(16) float delta[4] = {d[0], d[1], d[2], 0.f};
  std::memset(wp, 0, sizeof(wp));
  std::memcpy(mat, desc, sizeof(mat));
  mat[12] += d[0];
  mat[13] += d[1];
  mat[14] += d[2];
  std::memcpy(wp, desc + 0xE4, 0x12);
  reinterpret_cast<Fn2>(g_helpers[kHWpAdd].fn)(wp, delta);
  reinterpret_cast<Fn2>(g_helpers[kHSetMatrix].fn)(desc, mat);
  reinterpret_cast<Fn2>(g_helpers[kHSetWp].fn)(desc, wp);
}
// Game thread, set once per head step: this step's and the previous step's mount anchor delta (the previous-frame
// description gets the previous delta, so motion vectors stay consistent).
float g_mount_delta[3] = {}, g_mount_delta_prev[3] = {};
float g_arm_view_delta[3] = {}, g_arm_view_delta_prev[3] = {};

// Level horizon ("pitch decoupling", on unless "nolevel"): the game camera's pitch and roll are removed before the
// head pose is applied, so the mouse / gamepad only turns left-right and the head alone looks up and down. World up is
// detected from the game camera itself: the axis on which its right row has no component (no roll) and its up row is
// largest; it must agree for 30 frames before it is used. Position is left as the game sets it.
std::atomic<bool> g_level{true};
int g_up_axis = -1;           // 0..2, or -1 while unknown (game thread)
float g_up_sign = 1.f;
int g_up_votes = 0, g_up_candidate = -1;
float g_up_candidate_sign = 1.f;
std::atomic<std::uint64_t> g_level_applied{0}, g_level_skipped{0}, g_level_guarded{0};
std::mutex g_body_yaw_mutex;
float g_body_yaw = 0.f;
ULONGLONG g_body_yaw_until = 0;

bool read_body_yaw(float* yaw) {
  std::lock_guard<std::mutex> lock(g_body_yaw_mutex);
  if (GetTickCount64() >= g_body_yaw_until) return false;
  *yaw = g_body_yaw;
  return true;
}

void apply_body_yaw(unsigned char* desc, float yaw) {
  float mat[16];
  std::memcpy(mat, desc, sizeof(mat));
  const float c = std::cos(yaw), s = std::sin(yaw);
  mat[0] = c; mat[1] = 0.f; mat[2] = -s;
  mat[4] = 0.f; mat[5] = 1.f; mat[6] = 0.f;
  mat[8] = s; mat[9] = 0.f; mat[10] = c;
  reinterpret_cast<Fn2>(g_helpers[kHSetMatrix].fn)(desc, mat);
}

void detect_up(const float* r, const float* u) {
  int best = -1;
  float best_u = 0.5f;
  for (int a = 0; a < 3; ++a) {
    if (std::fabs(r[a]) < 0.08f && std::fabs(u[a]) > best_u) {
      best = a;
      best_u = std::fabs(u[a]);
    }
  }
  if (best < 0) {
    return;  // rolled, or pitched past ~60 deg: no vote
  }
  const float sign = u[best] > 0.f ? 1.f : -1.f;
  if (best == g_up_candidate && sign == g_up_candidate_sign) {
    if (++g_up_votes >= 30 && (g_up_axis != best || g_up_sign != sign)) {
      g_up_axis = best;
      g_up_sign = sign;
      log_line("level: world up = %c%c (from the game camera's right/up rows)", sign > 0 ? '+' : '-', "xyz"[best]);
    }
  } else {
    g_up_candidate = best;
    g_up_candidate_sign = sign;
    g_up_votes = 1;
  }
}

float dot3(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
bool normalize3(float* v) {
  const float l = std::sqrt(dot3(v, v));
  if (l < 1e-4f) {
    return false;
  }
  v[0] /= l;
  v[1] /= l;
  v[2] /= l;
  return true;
}

// Replace the description's axis rows with the yaw-only basis: up = world up, right and forward horizontal.
bool level_desc(unsigned char* desc, bool vote) {
  alignas(16) float mat[16];
  std::memcpy(mat, desc, sizeof(mat));
  float r[3] = {mat[0], mat[1], mat[2]}, u[3] = {mat[4], mat[5], mat[6]}, f[3] = {mat[8], mat[9], mat[10]};
  if (vote) {
    detect_up(r, u);
  }
  if (g_up_axis < 0) {
    return false;
  }
  float w[3] = {0.f, 0.f, 0.f};
  w[g_up_axis] = g_up_sign;
  // ISSUE-01 guard: near-vertical game pitch (stick fully up/down) makes the horizontal axes unreliable and the old
  // up-row fallback put the view above the body facing the floor. Within ~10 deg of vertical, keep the last level basis.
  static float last_rh[3] = {}, last_fh[3] = {};
  static bool have_last = false;
  {
    const float fl = std::sqrt(dot3(f, f));
    if (have_last && fl > 1e-6f && std::fabs(dot3(f, w)) / fl > 0.985f) {
      for (int k = 0; k < 3; ++k) {
        mat[k] = last_rh[k];
        mat[4 + k] = w[k];
        mat[8 + k] = last_fh[k];
      }
      g_level_guarded.fetch_add(1, std::memory_order_relaxed);
      reinterpret_cast<Fn2>(g_helpers[kHSetMatrix].fn)(desc, mat);
      return true;
    }
  }
  float rh[3] = {r[0] - dot3(r, w) * w[0], r[1] - dot3(r, w) * w[1], r[2] - dot3(r, w) * w[2]};
  if (!normalize3(rh)) {
    return false;  // right axis vertical: leave this frame as it is
  }
  // forward = horizontal and perpendicular to right, on the side the camera faces
  float fh[3] = {f[0] - dot3(f, w) * w[0], f[1] - dot3(f, w) * w[1], f[2] - dot3(f, w) * w[2]};
  const float d = dot3(fh, rh);
  fh[0] -= d * rh[0];
  fh[1] -= d * rh[1];
  fh[2] -= d * rh[2];
  if (!normalize3(fh)) {
    // looking straight up or down: the up row points along the horizontal facing (backwards when looking up)
    const float s = dot3(f, w) > 0.f ? -1.f : 1.f;
    fh[0] = s * (u[0] - dot3(u, w) * w[0]);
    fh[1] = s * (u[1] - dot3(u, w) * w[1]);
    fh[2] = s * (u[2] - dot3(u, w) * w[2]);
    if (!normalize3(fh)) {
      return false;
    }
  }
  for (int k = 0; k < 3; ++k) {
    mat[k] = rh[k];
    mat[4 + k] = w[k];
    mat[8 + k] = fh[k];
  }
  if (vote) {  // the current frame's description (not the previous-frame copy) refreshes the guard basis
    for (int k = 0; k < 3; ++k) {
      last_rh[k] = rh[k];
      last_fh[k] = fh[k];
    }
    have_last = true;
  }
  reinterpret_cast<Fn2>(g_helpers[kHSetMatrix].fn)(desc, mat);
  return true;
}

// Controller rays in engine world space (PORT-11 step 0, read-only; research/WEAPONS_AND_HANDS.md). Each hand's
// OpenXR aim pose is taken relative to the head recentre reference (eye_transform) and placed on the same game-camera
// basis the eye cameras are built on (after level horizon), so a ray lines up with what the headset shows. Logged
// with the head lines ("aim:"); nothing is written to the game.
const head_pose::Snapshot* g_step_snap = nullptr;  // game thread, set for the duration of head_step
struct AimRays {
  float grip[2][16] = {};
  float ride_grip[2][16] = {};  // grip with the shared on-foot hand filter ("Hand smoothing"), for the rider solve
  bool grip_valid[2] = {false, false};
  ULONGLONG sampled_at = 0;
  float org[2][3] = {};
  float dir[2][3] = {};
  bool valid[2] = {false, false};
  float cam_pos[3] = {};  // game camera description position (+0x30..), the frame the rays are expressed in
  float cam_mat[16] = {}; // the full game camera description matrix the grips were built in (run 28)
  float cam_fwd[3] = {};  // game camera forward before level horizon (what the game aims along, INFERENCE)
  float head_offset[3] = {};  // headset centre (mean of both eyes) minus the game camera position, world units
  bool head_valid = false;
  float body[3] = {};  // player body position (+0x94) read in the same head step as the grips
  bool body_valid = false;
  float yaw = 0.f;     // game look yaw at the same step (turning compensation)
  bool yaw_valid = false;
  std::uint64_t frames = 0;
};
std::mutex g_aim_mutex;
AimRays g_aim;  // g_aim_mutex

void compute_aim(const unsigned char* desc, const float* game_fwd) {
  if (g_step_snap == nullptr) {
    return;
  }
  float mat[16];
  std::memcpy(mat, desc, sizeof(mat));
  std::memcpy(g_step_cam, mat, sizeof(mat));
  AimRays a;
  // Re-rendering a frozen XR snapshot must not make an old pose appear fresh.
  static std::uint64_t last_pose_seq = 0;
  static std::int64_t last_pose_time = 0;
  static ULONGLONG last_pose_tick = 0;
  if (g_step_snap->seq != last_pose_seq || g_step_snap->display_time != last_pose_time || last_pose_tick == 0) {
    last_pose_seq = g_step_snap->seq;
    last_pose_time = g_step_snap->display_time;
    last_pose_tick = GetTickCount64();
  }
  a.sampled_at = last_pose_tick;
  float arm_rotation[2][9] = {}, arm_offset[2][3] = {};  // run 54: clean arm rig sample (vr_arms)
  bool arm_valid[2] = {false, false};
  for (int h = 0; h < 2; ++h) {
    const head_pose::Hand& hp = g_step_snap->grip[h];
    if (!hp.valid) continue;
    head_pose::Eye e;
    e.qx = hp.qx; e.qy = hp.qy; e.qz = hp.qz; e.qw = hp.qw;
    e.px = hp.px; e.py = hp.py; e.pz = hp.pz;
    float rotation[9], offset[3];
    eye_transform(e, rotation, offset);
    std::memcpy(arm_rotation[h], rotation, sizeof(rotation)); std::memcpy(arm_offset[h], offset, sizeof(offset));
    arm_valid[h] = true;
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) {
        a.grip[h][r * 4 + k] = rotation[r] * mat[k] + rotation[3 + r] * mat[4 + k] +
                                 rotation[6 + r] * mat[8 + k];
      }
    }
    for (int k = 0; k < 3; ++k) {
      a.grip[h][12 + k] = mat[12 + k] + offset[0] * mat[k] + offset[1] * mat[4 + k] + offset[2] * mat[8 + k];
    }
    a.grip[h][15] = 1.f;
    a.grip_valid[h] = true;
  }
  for (int h = 0; h < 2; ++h) {
    const head_pose::Hand& hp = g_step_snap->aim[h];
    if (!hp.valid) {
      continue;
    }
    head_pose::Eye e;
    e.qx = hp.qx, e.qy = hp.qy, e.qz = hp.qz, e.qw = hp.qw;
    e.px = hp.px, e.py = hp.py, e.pz = hp.pz;
    float m[9], off[3];
    eye_transform(e, m, off);
    for (int k = 0; k < 3; ++k) {
      // origin = camera position + offset in the camera's axes; direction = the hand's local +z (forward) in world
      a.org[h][k] = mat[12 + k] + off[0] * mat[k] + off[1] * mat[4 + k] + off[2] * mat[8 + k];
      a.dir[h][k] = m[0 * 3 + 2] * mat[k] + m[1 * 3 + 2] * mat[4 + k] + m[2 * 3 + 2] * mat[8 + k];
    }
    a.valid[h] = normalize3(a.dir[h]);
  }
  for (int k = 0; k < 3; ++k) {
    a.cam_pos[k] = mat[12 + k];
    a.cam_fwd[k] = game_fwd[k];
  }
  std::memcpy(a.cam_mat, mat, sizeof(a.cam_mat));
  {
    // Same basis as the eye cameras and grips: the headset's 6DoF offset from the game camera (0 at recentre).
    float sum[3] = {0.f, 0.f, 0.f}, local_head[3] = {0.f, 0.f, 0.f}, head_rot[9] = {};
    for (int e = 0; e < 2; ++e) {
      float m[9], off[3];
      eye_transform(g_step_snap->eye[e], m, off);
      if (e == 0) std::memcpy(head_rot, m, sizeof(head_rot));
      for (int k = 0; k < 3; ++k) sum[k] += off[0] * mat[k] + off[1] * mat[4 + k] + off[2] * mat[8 + k];
      for (int k = 0; k < 3; ++k) local_head[k] += 0.5f * off[k];
    }
    vr_arms::publish(arm_rotation, arm_offset, arm_valid, local_head, mat, head_rot);
    std::memcpy(a.ride_grip, a.grip, sizeof(a.ride_grip));
    vr_arms::Sample filtered;
    if (vr_arms::latest(filtered)) for (int h = 0; h < 2; ++h) if (a.grip_valid[h] && filtered.valid[h]) {
      const float* f = filtered.local[h];
      for (int r = 0; r < 3; ++r) for (int k = 0; k < 3; ++k)
        a.ride_grip[h][r * 4 + k] = f[r * 4] * mat[k] + f[r * 4 + 1] * mat[4 + k] + f[r * 4 + 2] * mat[8 + k];
      for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k)
        a.ride_grip[h][12 + k] += (f[12 + j] - filtered.raw_local[h][12 + j]) * mat[j * 4 + k];
    }
    for (int k = 0; k < 3; ++k) a.head_offset[k] = 0.5f * sum[k];

    a.head_valid = std::isfinite(a.head_offset[0]) && std::isfinite(a.head_offset[1]) && std::isfinite(a.head_offset[2]);
  }
  {
    // Consumers that pose the body later in the frame shift the grips by how far the body moved since this step.
    const game::Pointers p = game::pointers();
    a.body_valid = p.body != nullptr && game::mem_read(p.body + 0x94, a.body, sizeof(a.body)) &&
                   std::isfinite(a.body[0]) && std::isfinite(a.body[1]) && std::isfinite(a.body[2]);
    const game::Player look = game::read();
    a.yaw_valid = look.look_ok && std::isfinite(look.yaw);
    a.yaw = look.yaw;
  }
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  a.frames = g_aim.frames + 1;
  g_aim = a;
}

// Log helper: elevation of v above the detected world-up plane, degrees (NaN while world up is unknown).
float elevation_deg(const float* v) {
  if (g_up_axis < 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return std::asin(std::clamp(v[g_up_axis] * g_up_sign, -1.f, 1.f)) * 57.2958f;
}

// Log helper: angle from b to a around world up, degrees, sign as the engine's handedness gives it (OPEN).
float heading_delta_deg(const float* a, const float* b) {
  if (g_up_axis < 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const int i = (g_up_axis + 1) % 3, j = (g_up_axis + 2) % 3;
  const float d = std::atan2(a[j], a[i]) - std::atan2(b[j], b[i]);
  return std::remainder(d, 6.2831853f) * 57.2958f * g_up_sign;
}

// Private camera = game camera with this frame's description transformed by (m, off) and the previous-frame
// description by (pm, poff).
// Eye render size for the second view (second = true) or the main view, when "eyesize" is set.
bool eyesize_for(bool second, std::uint32_t* w, std::uint32_t* h) {
  *w = g_eyesize_w.load(std::memory_order_relaxed);
  *h = g_eyesize_h.load(std::memory_order_relaxed);
  return *w != 0 && *h != 0 && (second || g_eyesize_both.load(std::memory_order_relaxed));
}


// The view camera's viewport rect is the render rect source: the per-eye camera build 0x1405BCC40 copies it into the
// eye slots (frame +0xEEF0 + 0x244 = +0xF134) and derives the DLSS extents from it. The game camera gets it from the
// window (0x143082990: rect at +0x244 and +0x254, dirty +0x270); the private eye cameras are refreshed from the game
// camera every head step, so the eye size is written into them after that copy, before the build (2026-10-09 [STATIC]:
// run 18:00 showed the output size alone resized the offscreen target but not the DLSS extents). Our own objects only.
std::atomic<std::uint64_t> g_eye_rect_writes[2];
void apply_eye_rect(unsigned char* priv, bool second) {
  std::uint32_t w = 0, h = 0;
  if (priv == nullptr || !eyesize_for(second, &w, &h)) {
    return;
  }
  const std::int32_t rect[4] = {0, 0, static_cast<std::int32_t>(w), static_cast<std::int32_t>(h)};
  const std::int32_t size[2] = {static_cast<std::int32_t>(w), static_cast<std::int32_t>(h)};
  const std::uint32_t dirty = 1;
  if (write_bytes(priv + 0x244, rect, sizeof(rect)) && write_bytes(priv + 0x254, rect, sizeof(rect)) &&
      write_bytes(priv + 0x264, size, sizeof(size)) && write_bytes(priv + 0x270, &dirty, sizeof(dirty))) {
    g_eye_rect_writes[second ? 1 : 0].fetch_add(1, std::memory_order_relaxed);
  }
}

bool update_priv_camera_xf(unsigned char* priv, void* game_cam, const float* m, const float* off, const float* pm,
                           const float* poff, const AutoProj* ap) {
  alignas(16) unsigned char tmp[0x900];
  alignas(16) unsigned char desc[0x140];
  alignas(16) unsigned char prev[0x140];
  __try {
    if (game_cam == nullptr || priv == nullptr) {
      return false;
    }
    std::memset(tmp, 0, sizeof(tmp));
    std::memset(desc, 0, sizeof(desc));
    std::memset(prev, 0, sizeof(prev));
    reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(tmp, game_cam);
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(desc, tmp);
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(prev, tmp + 0x120);
    float game_fwd[3];
    std::memcpy(game_fwd, desc + 8 * sizeof(float), sizeof(game_fwd));
    if (g_level.load(std::memory_order_relaxed)) {
      // Both descriptions get the same treatment, so the previous-frame camera (motion vectors) stays consistent.
      const bool a = level_desc(desc, priv == g_priv_left);
      const bool b = level_desc(prev, false);
      (a && b ? g_level_applied : g_level_skipped).fetch_add(1, std::memory_order_relaxed);
    }
    {
      float body_yaw = 0.f;
      if (read_body_yaw(&body_yaw)) {
        // Both eyes and the controller transforms use this basis before HMD offsets.
        apply_body_yaw(desc, body_yaw);
        apply_body_yaw(prev, body_yaw);
      }
    }
    // Mounted: build on the rider's head instead of the mount chase camera (before compute_aim: grips, aim rays and
    // the head offset share the anchored basis).
    translate_desc(desc, g_mount_delta);
    translate_desc(prev, g_mount_delta_prev);
    if (priv == g_priv_left) {
      compute_aim(desc, game_fwd);  // once per head step, on the basis the eyes are built on
      // run 48 timing: C = the camera the eyes are built on (x = yaw deg of its forward, y/z = position x/z).
      float cf[3]; std::memcpy(cf, desc + 8 * sizeof(float), sizeof(cf));
      float cp[3]; std::memcpy(cp, desc + 12 * sizeof(float), sizeof(cp));
      const float c3[3] = {std::atan2(cf[0], cf[2]) * 57.2958f, cp[0], cp[2]};
      vr_weapon::timing_mark('C', c3);
      // "Lock arms to view": the eyes take the camera offset the arms and gun of this frame use, in every hand state.
      // Left eye first, so both eyes of a step share it. Not while mounted (the rider anchor owns the view).
      std::memcpy(g_arm_view_delta_prev, g_arm_view_delta, sizeof(g_arm_view_delta));
      if (mount_state::mounted() && !vr_arms::view_stable()) std::memset(g_arm_view_delta, 0, sizeof(g_arm_view_delta));
      else vr_arms::eye_view_delta(g_arm_view_delta);
      // The camera the eyes really render from: the native arm HUD places widgets against it.
      for (int k = 0; k < 3; ++k) g_step_cam[12 + k] += g_arm_view_delta[k];
    }
    translate_desc(desc, g_arm_view_delta);
    translate_desc(prev, g_arm_view_delta_prev);
    transform_desc(desc, m, off);
    transform_desc(prev, pm, poff);
    float render_aspect = g_render_aspect.load(std::memory_order_relaxed);
    std::uint32_t ew = 0, eh = 0;
    if (eyesize_for(priv != g_priv_left, &ew, &eh)) {
      render_aspect = static_cast<float>(ew) / static_cast<float>(eh);  // this eye renders at its own size
    }
    if (ap != nullptr && ap->aspect > 0.f) {
      render_aspect = ap->aspect;  // rounding the image dimensions must not change optical geometry
    }
    if (render_aspect > 0.f) {
      // Projection aspect = the render target's, not the game setting's (see set_render_size).
      const std::uint32_t dirty = 1;
      std::memcpy(desc + 0x104, &render_aspect, sizeof(float));
      std::memcpy(prev + 0x104, &render_aspect, sizeof(float));
      std::memcpy(desc + 0xE0, &dirty, sizeof(dirty));
      std::memcpy(prev + 0xE0, &dirty, sizeof(dirty));
    }
    const int fov_mdeg = g_fov_mdeg.load(std::memory_order_relaxed);
    if (ap != nullptr) {
      const std::uint32_t dirty = 1;
      std::memcpy(desc + 0x100, &ap->fov_rad, sizeof(float));
      std::memcpy(prev + 0x100, &ap->fov_rad, sizeof(float));
      std::memcpy(desc + 0xE0, &dirty, sizeof(dirty));
      std::memcpy(prev + 0xE0, &dirty, sizeof(dirty));
      const float o2[2] = {ap->m8, ap->m9};
      reinterpret_cast<Fn2>(g_helpers[kHProjOffset].fn)(desc, o2);
      reinterpret_cast<Fn2>(g_helpers[kHProjOffset].fn)(prev, o2);
    } else if (fov_mdeg > 0) {
      const float rad = static_cast<float>(fov_mdeg) * 1e-3f * 0.0174532925f;
      std::memcpy(desc + 0x100, &rad, sizeof(rad));
      std::memcpy(prev + 0x100, &rad, sizeof(rad));
      const std::uint32_t dirty = 1;  // same flag the engine's own projection setters raise (+0xE0)
      std::memcpy(desc + 0xE0, &dirty, sizeof(dirty));
      std::memcpy(prev + 0xE0, &dirty, sizeof(dirty));
    }
    if (ap == nullptr && g_off_set.load(std::memory_order_relaxed)) {
      const float o2[2] = {static_cast<float>(g_offx_milli.load()) * 1e-3f, static_cast<float>(g_offy_milli.load()) * 1e-3f};
      reinterpret_cast<Fn2>(g_helpers[kHProjOffset].fn)(desc, o2);
      reinterpret_cast<Fn2>(g_helpers[kHProjOffset].fn)(prev, o2);
    }
    reinterpret_cast<Fn3>(g_helpers[kHBuild].fn)(tmp, desc, prev);
    reinterpret_cast<Fn2>(g_helpers[kHAssign].fn)(priv, tmp);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

bool alloc_camera(unsigned char** slot, void* from) {
  __try {
    if (*slot == nullptr) {
      unsigned char* c = static_cast<unsigned char*>(g_alloc(0x900));
      if (c == nullptr) {
        return false;
      }
      std::memset(c, 0, 0x900);
      reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(c, from);
      *slot = c;
    }
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

// ---- "flatfollow" (ISSUE-04 test, 2026-10-21) -------------------------------------------------------------------------
// The eye cameras are private copies built FROM the game camera (levelled, body yaw, then the HMD offset); the game
// camera object itself stays the flat 16:9 camera facing the body. [AUTHOR] the VR image still behaves as if it were
// centred on the flat screen (shapes and some shadows change with head turns and with the FOV). Engine systems that read
// the game camera instead of a view's camera (light/shadow selection, LOD, streaming, cascade fit: owners UNKNOWN) would
// do exactly that; the 10-06 "cullfov" test only widened that camera, it never turned it with the head. With
// "flatfollow", after the eyes are built from the genuine game camera, the game camera itself becomes the mid-eye
// camera with an on-axis FOV covering both eye frusta. Next head step: if the camera still holds our matrix (the
// controller did not rebuild it), the genuine copy is assigned back first, so the head offset never compounds.
// Side effect while on: gameplay code that reads the game camera (crosshair, scan targeting) follows the head.
std::atomic<bool> g_flat_follow{false};
alignas(16) unsigned char g_ff_backup[0x900];
bool g_ff_written_valid = false;
float g_ff_written[16] = {};
float g_ff_last_m[9] = {}, g_ff_last_o[3] = {};
bool g_ff_last_valid = false;
std::atomic<std::uint64_t> g_ff_writes{0}, g_ff_restores{0}, g_ff_fail{0};
std::atomic<int> g_ff_fov_mdeg{0};

bool ff_read_matrix(void* cam, float* out16) {
  alignas(16) unsigned char desc[0x140];
  __try {
    std::memset(desc, 0, sizeof(desc));
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(desc, cam);
    std::memcpy(out16, desc, 16 * sizeof(float));
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

// Start of a head step: undo last step's write when the game did not rebuild its camera.
void flat_follow_restore(void* game_cam) {
  if (!g_ff_written_valid || game_cam == nullptr) {
    return;
  }
  float cur[16];
  if (ff_read_matrix(game_cam, cur) && std::memcmp(cur, g_ff_written, sizeof(cur)) == 0) {
    __try {
      reinterpret_cast<Fn2>(g_helpers[kHAssign].fn)(game_cam, g_ff_backup);
      g_ff_restores.fetch_add(1, std::memory_order_relaxed);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
      g_ff_fail.fetch_add(1, std::memory_order_relaxed);
    }
  }
  g_ff_written_valid = false;
}

bool flat_follow_write_impl(void* game_cam, const float* m, const float* off, const float* pm, const float* poff, float fov) {
  alignas(16) unsigned char tmp[0x900];
  alignas(16) unsigned char desc[0x140];
  alignas(16) unsigned char prev[0x140];
  __try {
    std::memset(g_ff_backup, 0, sizeof(g_ff_backup));
    reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(g_ff_backup, game_cam);
    std::memset(tmp, 0, sizeof(tmp));
    std::memset(desc, 0, sizeof(desc));
    std::memset(prev, 0, sizeof(prev));
    reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(tmp, game_cam);
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(desc, tmp);
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(prev, tmp + 0x120);
    if (g_level.load(std::memory_order_relaxed)) {  // same basis as the eyes (update_priv_camera_xf)
      level_desc(desc, false);
      level_desc(prev, false);
    }
    float body_yaw = 0.f;
    if (read_body_yaw(&body_yaw)) {
      apply_body_yaw(desc, body_yaw);
      apply_body_yaw(prev, body_yaw);
    }
    transform_desc(desc, m, off);
    transform_desc(prev, pm, poff);
    const std::uint32_t dirty = 1;
    std::memcpy(desc + 0x100, &fov, sizeof(float));
    std::memcpy(prev + 0x100, &fov, sizeof(float));
    std::memcpy(desc + 0xE0, &dirty, sizeof(dirty));
    std::memcpy(prev + 0xE0, &dirty, sizeof(dirty));
    const float o2[2] = {0.f, 0.f};
    reinterpret_cast<Fn2>(g_helpers[kHProjOffset].fn)(desc, o2);
    reinterpret_cast<Fn2>(g_helpers[kHProjOffset].fn)(prev, o2);
    reinterpret_cast<Fn3>(g_helpers[kHBuild].fn)(tmp, desc, prev);
    reinterpret_cast<Fn2>(g_helpers[kHAssign].fn)(game_cam, tmp);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

// End of a successful head step: the game camera becomes the mid-eye camera.
void flat_follow_write(void* game_cam, const head_pose::Snapshot& snap) {
  if (!g_flat_follow.load(std::memory_order_relaxed) || game_cam == nullptr || !snap.eye[0].rvalid || !snap.eye[1].rvalid) {
    g_ff_last_valid = false;
    return;
  }
  head_pose::Eye mid = snap.eye[0];
  mid.px = 0.5f * (snap.eye[0].px + snap.eye[1].px);
  mid.py = 0.5f * (snap.eye[0].py + snap.eye[1].py);
  mid.pz = 0.5f * (snap.eye[0].pz + snap.eye[1].pz);
  float m[9], off[3];
  eye_transform(mid, m, off);
  // On-axis frustum covering both eyes' rendered frusta at the game camera's own aspect.
  float th = 0.f, tv = 0.f;
  for (int e = 0; e < 2; ++e) {
    const float* t = snap.eye[e].rtan;
    th = std::max(th, std::max(-t[0], t[1]));
    tv = std::max(tv, std::max(t[2], -t[3]));
  }
  float aspect = 0.f;
  if (!read_aspect(game_cam, &aspect) || !(aspect > 0.3f && aspect < 4.f)) {
    aspect = 16.f / 9.f;
  }
  const float fov = 2.f * std::atan(std::max(th, aspect * tv));
  if (!(fov > 0.2f && fov < 3.05f)) {
    return;
  }
  const float* pm = g_ff_last_valid ? g_ff_last_m : m;
  const float* poff = g_ff_last_valid ? g_ff_last_o : off;
  if (!flat_follow_write_impl(game_cam, m, off, pm, poff, fov)) {
    g_ff_fail.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  std::memcpy(g_ff_last_m, m, sizeof(m));
  std::memcpy(g_ff_last_o, off, sizeof(off));
  g_ff_last_valid = true;
  g_ff_written_valid = ff_read_matrix(game_cam, g_ff_written);
  g_ff_writes.fetch_add(1, std::memory_order_relaxed);
  g_ff_fov_mdeg.store(static_cast<int>(fov * 57295.78f), std::memory_order_relaxed);
}

// Game thread: head-tracked cameras for both eyes. Returns false when no pose is available (caller falls back).
bool head_step(void* game_cam) {
  head_pose::Snapshot snap;
  if (!xr_runtime::latest_views(&snap)) {
    return false;
  }
  flat_follow_restore(game_cam);  // eyes are always built from the genuine game camera
  {
    std::memcpy(g_mount_delta_prev, g_mount_delta, sizeof(g_mount_delta));
    float cam[16];
    if (!ff_read_matrix(game_cam, cam) || !mount_camera::eye_anchor(cam + 12, g_mount_delta)) {
      std::memset(g_mount_delta, 0, sizeof(g_mount_delta));
    }
  }
  float rebase[3], rebase_yaw;
  {
    std::lock_guard<std::mutex> lock(g_rebase_mutex);
    std::memcpy(rebase, g_rebase, sizeof(rebase));
    std::memset(g_rebase, 0, sizeof(g_rebase));
    rebase_yaw = g_rebase_yaw;
    g_rebase_yaw = 0.f;
  }
  if (g_recentre_request.exchange(false)) {
    g_head_ref_valid = false;
  }
  if (g_head_ref_valid && (rebase[0] != 0.f || rebase[1] != 0.f || rebase[2] != 0.f)) {
    // World delta -> engine camera-local offset on the last aim basis (rows right/up/forward), -> XR LOCAL (S flips z),
    // -> reference frame. off_eng = S*R^-1*(p - ref), so ref += R*S*d_eng lowers the head offset by exactly the delta.
    float mat[16];
    {
      std::lock_guard<std::mutex> lock(g_aim_mutex);
      std::memcpy(mat, g_aim.cam_mat, sizeof(mat));
    }
    const float d_eng[3] = {dot3(rebase, mat), dot3(rebase, mat + 4), dot3(rebase, mat + 8)};
    if (std::isfinite(d_eng[0]) && std::isfinite(d_eng[1]) && std::isfinite(d_eng[2]) &&
        dot3(mat, mat) > 0.5f) {  // no basis before the first aim sample
      const float d_xr[3] = {d_eng[0], d_eng[1], -d_eng[2]};
      float w[3];
      quat_rot(g_ref_q, d_xr, w);
      for (int k = 0; k < 3; ++k) g_ref_p[k] += w[k];
      g_rebase_applied.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (g_head_ref_valid && rebase_yaw != 0.f && std::isfinite(rebase_yaw)) {
    // The game look yaw was turned by rebase_yaw (engine convention, + = right). An XR yaw of t about +y is an engine
    // yaw of -t (S*Ry(t)*S = Ry(-t)), so the head's engine yaw relative to the camera is ref_yaw - head_yaw (XR):
    // lowering the reference yaw by the same amount keeps every eye and grip where it was in the world.
    const float ang = 2.f * std::atan2(g_ref_q[1], g_ref_q[3]) - rebase_yaw;
    g_ref_q[0] = 0.f;
    g_ref_q[1] = std::sin(ang * 0.5f);
    g_ref_q[2] = 0.f;
    g_ref_q[3] = std::cos(ang * 0.5f);
  }
  {
    float hm[9], ho[3];
    eye_transform(snap.eye[0], hm, ho);  // head forward in camera-local axes = column 2 (x right, z forward)
    const float yaw_rel = std::atan2(hm[2], hm[8]);
    if (std::isfinite(yaw_rel) && hm[2] * hm[2] + hm[8] * hm[8] > 0.04f) {  // not looking straight up or down
      g_head_yaw_rel.store(yaw_rel, std::memory_order_relaxed);
      g_head_yaw_tick.store(GetTickCount64(), std::memory_order_relaxed);
      g_head_yaw_seq.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (!g_head_ref_valid) {
    // Recentre: yaw-only orientation of the left eye, position between the eyes.
    const head_pose::Eye& l = snap.eye[0];
    const float fwd_in[3] = {0.f, 0.f, -1.f};
    const float ql[4] = {l.qx, l.qy, l.qz, l.qw};
    float f[3];
    quat_rot(ql, fwd_in, f);
    const float yaw = std::atan2(-f[0], -f[2]);  // rotation about +y that maps -z to f (projected)
    g_ref_q[0] = 0.f;
    g_ref_q[1] = std::sin(yaw * 0.5f);
    g_ref_q[2] = 0.f;
    g_ref_q[3] = std::cos(yaw * 0.5f);
    g_ref_p[0] = 0.5f * (snap.eye[0].px + snap.eye[1].px);
    g_ref_p[1] = 0.5f * (snap.eye[0].py + snap.eye[1].py);
    g_ref_p[2] = 0.5f * (snap.eye[0].pz + snap.eye[1].pz);
    g_head_ref_valid = true;
    g_last_valid[0] = g_last_valid[1] = false;
    log_line("head: recentred yaw=%.1f deg ref=(%.3f,%.3f,%.3f)", yaw * 57.2958f, g_ref_p[0], g_ref_p[1], g_ref_p[2]);
  }
  unsigned char* targets[2] = {g_priv_left, g_priv_cam};
  bool ok = true;
  g_step_snap = &snap;
  float aspect = 0.f;
  const float render_aspect = g_render_aspect.load(std::memory_order_relaxed);
  const bool have_aspect = render_aspect > 0.f ? (aspect = render_aspect, true) : read_aspect(game_cam, &aspect);
  const bool want_auto = g_auto_fov.load(std::memory_order_relaxed) && have_aspect;
  for (int e = 0; e < 2; ++e) {
    float m[9], off[3];
    head_pose::Eye pose = snap.eye[e];
    if (g_zero_sep.load(std::memory_order_relaxed) || g_zero_sep_forced.load(std::memory_order_relaxed)) {
      // Test: both eyes rendered from the point between them. Any left/right difference that remains is not parallax
      // (or camera-facing foliage turning to each eye) but something that differs in time between the two views.
      pose.px = 0.5f * (snap.eye[0].px + snap.eye[1].px);
      pose.py = 0.5f * (snap.eye[0].py + snap.eye[1].py);
      pose.pz = 0.5f * (snap.eye[0].pz + snap.eye[1].pz);
    }
    eye_transform(pose, m, off);
    AutoProj ap = {};
    float eye_aspect = aspect;
    {
      std::uint32_t ew = 0, eh = 0;
      if (eyesize_for(e == 1, &ew, &eh)) {
        eye_aspect = static_cast<float>(ew) / static_cast<float>(eh);
      }
    }
    const bool use_auto = want_auto && auto_projection(snap, e, eye_aspect, &ap);
    snap.eye[e].rvalid = use_auto;
    if (use_auto) {
      std::memcpy(snap.eye[e].rtan, ap.tan4, sizeof(ap.tan4));
      if (ap.tan4[1] - ap.tan4[0] > 1e-4f && ap.tan4[2] - ap.tan4[3] > 1e-4f) {
        g_expect_m0[e].store(2.f / (ap.tan4[1] - ap.tan4[0]), std::memory_order_relaxed);
        g_expect_m5[e].store(2.f / (ap.tan4[2] - ap.tan4[3]), std::memory_order_relaxed);
      }
      g_auto_applied.fetch_add(1, std::memory_order_relaxed);
      if (e == 0) {
        g_auto_last_fov_mdeg.store(static_cast<int>(ap.fov_rad * 57295.78f), std::memory_order_relaxed);
        g_auto_last_aspect_milli.store(static_cast<int>((ap.aspect > 0.f ? ap.aspect : aspect) * 1000.f), std::memory_order_relaxed);
      }
    } else if (want_auto || g_auto_fov.load(std::memory_order_relaxed)) {
      g_auto_failed.fetch_add(1, std::memory_order_relaxed);
    }
    const float* pm = g_last_valid[e] ? g_last_m[e] : m;
    const float* poff = g_last_valid[e] ? g_last_o[e] : off;
    if (!alloc_camera(e == 0 ? &g_priv_left : &g_priv_cam, game_cam)) {
      ok = false;
      continue;
    }
    targets[e] = e == 0 ? g_priv_left : g_priv_cam;
    if (!update_priv_camera_xf(targets[e], game_cam, m, off, pm, poff, use_auto ? &ap : nullptr)) {
      ok = false;
      continue;
    }
    apply_eye_rect(targets[e], e == 1);
    std::memcpy(g_last_m[e], m, sizeof(m));
    std::memcpy(g_last_o[e], off, sizeof(off));
    g_last_valid[e] = true;
  }
  g_step_snap = nullptr;
  {
    const float dx = snap.eye[1].px - snap.eye[0].px, dy = snap.eye[1].py - snap.eye[0].py, dz = snap.eye[1].pz - snap.eye[0].pz;
    const float ipd = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::lock_guard<std::mutex> lock(g_ui_tan_mutex);
    for (int e = 0; e < 2; ++e) {
      g_ui_tan_valid[e] = snap.eye[e].rvalid;
      if (snap.eye[e].rvalid) {
        std::memcpy(g_ui_tan[e], snap.eye[e].rtan, sizeof(g_ui_tan[e]));
      }
    }
    g_ui_ipd = ipd > 0.04f && ipd < 0.09f ? ipd : 0.064f;
  }
  if (ok) {
    flat_follow_write(game_cam, snap);
  }
  if (ok) {
    std::lock_guard<std::mutex> lock(g_applied_mutex);
    g_applied[g_applied_count % kPoseRing] = snap;
    std::memcpy(g_applied_cam[g_applied_count % kPoseRing], g_step_cam, sizeof(g_step_cam));
    ++g_applied_count;
  }
  return ok;
}

bool update_priv_camera(void* main_cam, float sep) {
  alignas(16) unsigned char tmp[0x900];
  alignas(16) unsigned char desc[0x140];
  alignas(16) unsigned char prev[0x140];
  __try {
    if (main_cam == nullptr || g_priv_cam == nullptr) {
      return false;
    }
    std::memset(tmp, 0, sizeof(tmp));
    std::memset(desc, 0, sizeof(desc));
    std::memset(prev, 0, sizeof(prev));
    reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(tmp, main_cam);
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(desc, tmp);
    reinterpret_cast<Fn2>(g_helpers[kHDescCopy].fn)(prev, tmp + 0x120);
    shift_desc(desc, sep);
    shift_desc(prev, sep);
    reinterpret_cast<Fn3>(g_helpers[kHBuild].fn)(tmp, desc, prev);
    reinterpret_cast<Fn2>(g_helpers[kHAssign].fn)(g_priv_cam, tmp);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

bool init_priv_camera(void* main_cam) {
  __try {
    if (g_priv_cam == nullptr) {
      g_priv_cam = static_cast<unsigned char*>(g_alloc(0x900));
      if (g_priv_cam == nullptr) {
        return false;
      }
      std::memset(g_priv_cam, 0, 0x900);
      reinterpret_cast<Fn2>(g_helpers[kHCopyCtor].fn)(g_priv_cam, main_cam);
    }
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    g_priv_cam = nullptr;
    return false;
  }
}

// Gfx_ViewManager::StartFrame (game thread), before the engine walks its view list.
// A second view built on the start-up screens must not carry anything into gameplay: the private eye cameras were
// copy-constructed from the start-up camera, the slot poses and the held output belong to the old view, and the head
// reference was taken there. Stereo switched on from launch therefore came up corrupted after the start-up -> gameplay
// switch, while switching it on in gameplay worked. Everything is started fresh for the rebuilt view.
// Scene-state log (read-only), for telling gameplay from the start-up screens and from the pause / inventory /
// character menus: logged on every change, at most a few lines per second. Main view world (+0x1248), camera (+0x10;
// our private left camera is reported as the game camera it stands in for), owner (+0x18), which views are active
// (+0x24) and the GlobalGameSpeed float/flag (time_sync).
// Gameplay gate. The second view built on the start-up screens came up without a cube helper (view +0x1218 = 0, run
// 2026-10-05 14:59), while every view built in gameplay got one (runs 12:17, 12:24, 12:37, 13:29, 13:48). The view
// constructor creates that helper from params +0x46 (STEREO_ROUTE.md, static), and the params are copied from the main
// view's +0x13B0 block, so the main view's params +0x46 is 0 on the start-up screens and set in gameplay [INFERENCE from
// STATIC + LIVE]. Stereo runs only while it is set; otherwise the second view stays inactive, the game camera stays
// on the main view and the headset gets the mono picture on the cinema screen. Whether the pause / inventory /
// character menus clear it too is OPEN: the scene log reports the byte and any other params byte that changes.
constexpr std::uint32_t kSceneFlagOffset = kParamsOffset + 0x46;
constexpr int kSceneOnFrames = 60;   // StartFrames with the flag set before stereo (re)starts (~1 s)
constexpr int kSceneOffFrames = 3;   // StartFrames with the flag clear before stereo pauses
std::atomic<ULONGLONG> g_resize_hold_until{0};
std::atomic<bool> g_scene_gate_on{true};    // "anyscene" in the control file: stereo wherever the main view renders
std::atomic<bool> g_scene_gameplay{false};  // gate state (game thread writes)
std::atomic<bool> g_stereo_running{false};
// Eye mode: both views render offscreen at the eye size; the backbuffer is only a mirror (xr_runtime). Stereo paused
// (menus, loading, start-up) = off, so the game renders those at the window size as usual.
bool eye_mode_on() {
  std::uint32_t w = 0, h = 0;
  return eyesize_for(false, &w, &h) && g_created != nullptr && g_stereo_running.load(std::memory_order_relaxed) &&
         !g_rescued.load(std::memory_order_relaxed);
}  // second view active and its image wanted (read on the Present thread)
std::atomic<ULONGLONG> g_stereo_started_at{0};  // when stereo last (re)started; stereo_viewport_ids waits 1 s after it
int g_scene_streak = 0;                      // consecutive StartFrames in the current flag state
std::atomic<std::uint64_t> g_stereo_pauses{0}, g_stereo_resumes{0};

bool scene_flag(void* main, std::uint8_t* out) {
  return main != nullptr && read_u8(static_cast<unsigned char*>(main) + kSceneFlagOffset, out);
}

// Front end (3D main menu) vs gameplay, for the UI placement only (stereo runs in both). [LIVE] runs 2026-10-05 15:12
// and 2026-10-08 06:04: no params byte is known to tell the 3D main menu from gameplay (both set +0x46; OPEN), but the
// order is fixed: start-up (+0x46 clear) -> main menu (set) -> load (clear while the main view keeps rendering,
// 15:12:26-49, 06:04:50-06:05:15) -> gameplay (set). So everything up to the first load after the main menu is front
// end. Pause / inventory menus stop the main view instead (not live) and never count as a load. OPEN: quitting back to
// the main menu from gameplay is not detected (the HUD placement stays).
constexpr ULONGLONG kLoadClearMs = 2000;  // +0x46 clear with the main view rendering for this long = a load
constexpr int kFrontMenuFrames = 280;     // ~4 s set without a virtual cursor also counts as the main menu
constexpr ULONGLONG kBlipMs = 2500;       // a clear starting this soon after stereo started is the activation blip
bool g_front_clear_blip = false;          // game thread: the current clear began right after a stereo start
ULONGLONG g_front_seen_at = 0;            // game thread: when the main menu was last recognised
// "nomenustereo" in [native] (mod menu "3D main menu", 2026-11-09): the front end stays mono on the flat screen and the
// second view is not built before gameplay. AUTHOR test: does running the 3D main menu in stereo cost gameplay fps?
std::atomic<bool> g_menu_stereo{true};
bool g_front_menu_seen = false;           // game thread
int g_front_menu_streak = 0;
ULONGLONG g_front_clear_since = 0;

void update_front_end(bool live, bool flagged) {
  if (!g_front_end.load(std::memory_order_relaxed)) {
    return;
  }
  if (live && flagged) {
    g_front_clear_since = 0;
    // 2026-11-08: LIVE 18:50:36 the flag was set for ~1 s before the 3D main menu, cleared for 7 s with the main view
    // rendering, and that counted as "the load after the main menu": the real menu (18:50:44) then got the gameplay HUD
    // placement instead of the panel (HEADSET/AUTHOR: cursor and description windows wrong). The real menu shows the
    // game's virtual cursor (ui_cursor.cpp, cursor mode 2) or at least stays up for kFrontMenuFrames.
    const ULONGLONG cursor_age = GetTickCount64() - menu_pointer::g_native_cursor_at.load(std::memory_order_relaxed);
    ++g_front_menu_streak;
    if (!g_front_menu_seen && g_front_menu_streak >= kSceneOnFrames &&
        (cursor_age < 500 || g_front_menu_streak >= kFrontMenuFrames)) {
      g_front_menu_seen = true;
      g_front_seen_at = GetTickCount64();
      log_line("front end: 3D main menu (params +0x46 set, %s); main menu panel until the first load",
               cursor_age < 500 ? "virtual cursor on" : "set for 4 s");
    }
    return;
  }
  g_front_menu_streak = 0;
  if (!g_front_menu_seen || !live) {
    g_front_clear_since = 0;
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (g_front_clear_since == 0) {
    g_front_clear_since = now;
    // LIVE 18:50:36 and 19:27:15 (2026-11-08): the clear that ended the front end too early began within ~1 s of the
    // first second view becoming active (main params +0x21..+0x39 and +0x46 cleared together), and the virtual cursor was
    // already on, so the cursor test alone did not catch it (HEADSET/AUTHOR 19:27: main menu followed the head). The real
    // load in the 18:25 run began 6 s after stereo started. A clear that begins within kBlipMs of a stereo start is not
    // taken as the load: the main menu is waited for again. [INFERENCE: the blip is tied to the first activation.]
    // Also within kBlipMs of the menu being recognised, so the rule holds with the 3D main menu off (no stereo start).
    g_front_clear_blip = (g_stereo_running.load(std::memory_order_relaxed) &&
                          now - g_stereo_started_at.load(std::memory_order_relaxed) < kBlipMs) ||
                         now - g_front_seen_at < kBlipMs;
  } else if (now - g_front_clear_since >= kLoadClearMs && g_front_clear_blip) {
    g_front_menu_seen = false;
    g_front_clear_since = 0;
    log_line("front end: params +0x46 clear %llu ms, but it began right after stereo started or the menu appeared; not the "
             "load, waiting for the main menu again", static_cast<unsigned long long>(kLoadClearMs));
  } else if (now - g_front_clear_since >= kLoadClearMs) {
    g_front_end.store(false, std::memory_order_relaxed);
    log_line("front end: load after the main menu (params +0x46 clear %llu ms, main view rendering); HUD placement from here",
             static_cast<unsigned long long>(now - g_front_clear_since));
  }
}

// Game thread, once per StartFrame. Returns the gate state with hysteresis.
// Run 2026-10-09 19:55-19:56: a second view kept (inactive) through a loading screen came back with its ambient /
// indirect light lost (right/left brightness 0.04), while a freshly built one was correct. So when gameplay resumes
// after a load (gate off with the main view rendering and params +0x46 cleared; pause menus stop the main view instead),
// the second view is rebuilt rather than resumed.
bool g_load_seen = false;
// Quick resume after pause menus (2026-11-08, "nosettle" in [native] = off): stereo comes back after kResumeOnFrames
// (not kSceneOnFrames) when the gate closed because the main view stopped (pause menus), not for a load or the front end.
// The post-load dip (LIVE 18:26:24-26 and 18:50:36: the main params +0x21..+0x39 clear together just as a freshly built
// second view first becomes active, and in the 18:26 run came back as soon as it was paused) keeps the original handling:
// pause, rebuild, resume, now hidden behind black by xr_runtime. Keeping the second view through it (first try) was
// withdrawn before any headset test because the rebuild may be what ends the dip [INFERENCE]; the dip's cause is OPEN.
std::atomic<bool> g_load_settle{true};
std::atomic<bool> g_stereo_pending{false};
std::atomic<bool> g_last_pause_menu{false};  // the last stereo pause was a pause menu (main view idle)
constexpr int kResumeOnFrames = 10;
bool g_scene_set_last = false;           // game thread: this StartFrame's gate input
bool g_off_by_pause = false;             // game thread: the gate closed because the main view stopped, and no load followed
std::atomic<std::uint64_t> g_quick_resumes{0};
const char* g_rebuild_reason = "rebuild requested";

bool update_scene_gate(bool live) {
  if (g_view_count.load() == 0) {
    return false;
  }
  std::uint8_t flag = 0;
  const bool flagged = scene_flag(g_views[0].view, &flag) && flag != 0;
  update_front_end(live, flagged);
  const bool set = live && (flagged || !g_scene_gate_on.load(std::memory_order_relaxed));
  g_scene_set_last = set;
  const bool on = g_scene_gameplay.load(std::memory_order_relaxed);
  const bool settle = g_load_settle.load(std::memory_order_relaxed);
  if (!on && live && !flagged) {
    g_off_by_pause = false;  // a loading screen or the front end since the gate closed: no quick resume
  }
  if (set == on) {
    g_scene_streak = 0;
    return on;
  }
  int need = set ? kSceneOnFrames : kSceneOffFrames;
  if (settle && set && g_off_by_pause) {
    need = kResumeOnFrames;
  }
  if (++g_scene_streak >= need) {
    g_scene_streak = 0;
    g_scene_gameplay.store(set, std::memory_order_relaxed);
    log_line("scene gate: %s (main params +0x46 = %u, main view %s, after %d frames)",
             set ? "GAMEPLAY -> stereo allowed" : "menu / start-up -> mono", flag, live ? "rendering" : "idle", need);
    if (!set) {
      g_off_by_pause = !live;
    } else if (need == kResumeOnFrames) {
      g_quick_resumes.fetch_add(1, std::memory_order_relaxed);
    }
    if (!set && live && !flagged) {
      g_load_seen = true;  // loading screen (not a pause menu)
    } else if (set && g_load_seen) {
      g_load_seen = false;
      if (g_created != nullptr) {
        g_rebuild_reason = "loading screen (a kept second view comes back without ambient light)";
        g_rebuild_requested.store(true);
      }
    }
    return set;
  }
  return on;
}

// Params bytes of the main view that change (read-only), to find the pause / inventory / character menu signal.
// Bytes that keep changing are dropped after a few reports.
void log_params_changes() {
  static unsigned char last[kParamsSize];
  static unsigned char reports[kParamsSize];
  static bool have = false;
  static ULONGLONG window = 0;
  static int lines = 0;
  if (g_view_count.load() == 0) {
    return;
  }
  unsigned char cur[kParamsSize];
  if (!read_bytes(static_cast<unsigned char*>(g_views[0].view) + kParamsOffset, cur, sizeof(cur))) {
    return;
  }
  if (!have) {
    std::memcpy(last, cur, sizeof(cur));
    have = true;
    return;
  }
  char text[256];
  int n = 0;
  int shown = 0;
  for (std::uint32_t i = 0; i < kParamsSize; ++i) {
    if (cur[i] == last[i]) {
      continue;
    }
    if (reports[i] < 12) {
      ++reports[i];
      if (shown < 10 && n < static_cast<int>(sizeof(text)) - 24) {
        n += snprintf(text + n, sizeof(text) - n, " +%02X:%02X->%02X", i, last[i], cur[i]);
        ++shown;
      }
    }
    last[i] = cur[i];
  }
  if (shown == 0) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (now - window >= 1000) {
    window = now;
    lines = 0;
  }
  if (++lines <= 4) {
    log_line("scene params (main +0x13B0) changed:%s", text);
  }
}

void log_scene_state(const char* when) {
  static void* last[3] = {};
  static std::uint32_t last_active = 0xFFFFFFFF;
  static int last_speed_milli = -1, last_flag = -1;
  static ULONGLONG window = 0;
  static int lines = 0;
  if (g_view_count.load() == 0) {
    return;
  }
  void* main = g_views[0].view;
  void* cur[3] = {};
  read_ptr(static_cast<unsigned char*>(main) + kWorldOffset, &cur[0]);
  read_ptr(static_cast<unsigned char*>(main) + kCameraOffset, &cur[1]);
  read_ptr(static_cast<unsigned char*>(main) + 0x18, &cur[2]);
  if (cur[1] != nullptr && cur[1] == g_priv_left && g_created_main_cam != nullptr) {
    cur[1] = g_created_main_cam;
  }
  std::uint32_t active = 0;
  const int count = g_view_count.load();
  for (int i = 0; i < count && i < 32; ++i) {
    std::uint8_t a = 0;
    if (g_views[i].view != nullptr && read_u8(static_cast<unsigned char*>(g_views[i].view) + 0x24, &a) && a != 0) {
      active |= 1u << i;
    }
  }
  std::uint8_t cube = 0xFF;
  scene_flag(main, &cube);
  const int gate = g_scene_gameplay.load(std::memory_order_relaxed) ? 1 : 0;
  float speed = -1.f;
  unsigned char flag = 0xFF;
  const bool speed_ok = time_sync::read(&speed, &flag);
  const int speed_milli = speed_ok ? static_cast<int>(speed * 1000.f + 0.5f) : -1;
  static int last_cube = -1, last_gate = -1;
  const bool changed = cur[0] != last[0] || cur[1] != last[1] || cur[2] != last[2] || (active & 1u) != (last_active & 1u) ||
                       speed_milli != last_speed_milli || static_cast<int>(flag) != last_flag || cube != last_cube ||
                       gate != last_gate || (last_active == 0xFFFFFFFF);
  if (!changed) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (now - window >= 1000) {
    window = now;
    lines = 0;
  }
  if (++lines > 6) {
    return;  // a flickering field: report it again next second
  }
  log_line("scene (%s): world %p camera %p owner %p active views 0x%X | game speed %.3f flag %d | params+46 %d gate %s | second view %s",
           when, cur[0], cur[1], cur[2], active, speed, speed_ok ? static_cast<int>(flag) : -1, cube,
           gate ? "gameplay" : "menu", g_created != nullptr ? (g_stereo_running.load() ? "running" : "paused") : "none");
  last_cube = cube;
  last_gate = gate;
  last[0] = cur[0];
  last[1] = cur[1];
  last[2] = cur[2];
  last_active = active;
  last_speed_milli = speed_milli;
  last_flag = flag;
}

void reset_for_rebuild() {
  if (g_created_main != nullptr && g_priv_left != nullptr) {
    void* cur = nullptr;
    if (read_ptr(static_cast<unsigned char*>(g_created_main) + kCameraOffset, &cur) && cur == g_priv_left &&
        g_created_main_cam != nullptr) {
      write_ptr(static_cast<unsigned char*>(g_created_main) + kCameraOffset, g_created_main_cam);  // game camera back first
    }
  }
  g_priv_left = nullptr;  // engine-allocated; the old ones stay with the retired view (never freed)
  g_priv_cam = nullptr;
  // [LIVE] 2026-10-15: invalidating the head reference here recentred the headset (yaw/position) on every rebuild,
  // e.g. 5 dark-eye watchdog rebuilds in 3 min, -68.9 deg and 12 cm once: "the camera shifts on the body". The
  // reference lives in OpenXR space and does not depend on the view objects, so it is kept.
  g_last_valid[0] = g_last_valid[1] = false;
  g_head_prestepped = false;
  {
    std::lock_guard<std::mutex> lock(g_applied_mutex);
    std::memset(g_slot_pose, 0, sizeof(g_slot_pose));
  }
  g_out_pose.store(0);
  {
    std::lock_guard<std::mutex> lock(g_out_mutex);
    if (g_out_held != nullptr) {
      g_out_held->Release();
      g_out_held = nullptr;
    }
  }
  drop_main_eye();
  g_rebuilds.fetch_add(1, std::memory_order_relaxed);
}

// MSVC RTTI: the class name of the object whose vtable is `vt` (afop module only), or false.
bool rtti_name(std::uint64_t vt, char* out, size_t n) {
  const auto base = reinterpret_cast<std::uintptr_t>(g_module_base);
  auto in_module = [&](std::uint64_t a) { return base != 0 && a >= base && a < base + g_image_size; };
  std::uint64_t col = 0;
  if (!in_module(vt) || !read_bytes(reinterpret_cast<void*>(vt - 8), &col, 8) || !in_module(col)) {
    return false;
  }
  std::uint32_t sig = 0, td_rva = 0;
  if (!read_bytes(reinterpret_cast<void*>(col), &sig, 4) || sig != 1 ||
      !read_bytes(reinterpret_cast<void*>(col + 0xC), &td_rva, 4) || td_rva == 0 || td_rva >= g_image_size) {
    return false;
  }
  char name[96] = {};
  if (!read_bytes(reinterpret_cast<void*>(base + td_rva + 0x10), name, sizeof(name) - 1) || name[0] != '.' || name[1] != '?') {
    return false;
  }
  snprintf(out, n, "%s", name);
  return true;
}

// Per-view object slots (+0x40..+0x200) that the main view has and the second view lacks (found by log_view_diff).
// "sharehooks" in the control file copies them into the second view while stereo runs, to test whether the UI and the
// left-only effects are attached through them [experiment; the originals (null) are put back when it stops].
constexpr int kMaxShare = 32;
std::uint32_t g_share_off[kMaxShare] = {};
std::atomic<int> g_share_count{0};
std::atomic<bool> g_share_hooks{false};
std::atomic<int> g_share_pick{0};  // 99 = all slots, 1..11 = only that slot
std::atomic<int> g_share_override{-1};  // automatic slot sweep (frame_capture): -1 none, 0 off, 1..N / 99
int share_pick_now() {
  const int o = g_share_override.load(std::memory_order_relaxed);
  return o >= 0 ? o : (g_share_hooks.load(std::memory_order_relaxed) ? g_share_pick.load(std::memory_order_relaxed) : 0);
}
int g_share_applied_pick = 0;      // game thread
bool g_share_applied = false;  // game thread
std::atomic<std::uint64_t> g_share_writes{0};

// UI in both eyes (sweep 2026-10-05 16:54-16:56 [LIVE]): main-view slot +0x130 (record starting afop+0x55EFE0) shared
// with the second view puts the game UI into the right eye while the eyes stay different. Slot +0x108 (afop+0x563CC0)
// turns the right eye into a copy of the left and must never be shared. On unless "noeyeui" is in the control file.
constexpr std::uint32_t kUiSlot = 0x130;
std::atomic<bool> g_eye_ui{false};
bool g_eye_ui_applied = false;  // game thread

void eye_ui_step(bool on) {
  if (g_created == nullptr || g_view_count.load() == 0) {
    g_eye_ui_applied = false;
    return;
  }
  unsigned char* ours = static_cast<unsigned char*>(g_created);
  if (on) {
    void* m = nullptr;
    void* o = nullptr;
    if (read_ptr(static_cast<unsigned char*>(g_views[0].view) + kUiSlot, &m) && m != nullptr &&
        read_ptr(ours + kUiSlot, &o) && o != m) {
      write_ptr(ours + kUiSlot, m);
    }
    if (!g_eye_ui_applied) {
      log_line("eyeui: main view UI slot +0x%X shared with the second view (%p)", kUiSlot, m);
    }
    g_eye_ui_applied = true;
  } else if (g_eye_ui_applied) {
    write_ptr(ours + kUiSlot, nullptr);
    g_eye_ui_applied = false;
    log_line("eyeui: second view UI slot cleared");
  }
}

// UI effects in both eyes ("noeyefx" turns it off). [AUTHOR] + captures 2026-10-08 07:5x: the scan highlights sit on
// their animals in the left eye but at the LEFT eye's screen positions in the right eye, and the weapon wheel's frosted
// backdrop shows in the left eye only. Both come from the main-only neighbour passes of the UI pass (+0x128 ->
// 0x55F150, +0x138 -> 0x55EE10; uitrace 07:46-07:48: scan meshes into a 1172x1271 target, glass quads into the half
// target), which the second view does not have, so its UI pass composites the main view's results. The slot sweep of
// 2026-10-05 shared each of these slots alone without harm (no visible change outside the scan). Shared like +0x130,
// the second view runs them with its own camera; the UI hooks then place their glass per eye. OPEN: whether their
// targets are per view (if they are per owner, the two views overwrite each other's within a frame). [LIVE] 08:12: they
// are shared; off by default.
constexpr std::uint32_t kFxSlots[] = {0x128, 0x138};
std::atomic<bool> g_eye_fx{false};
bool g_eye_fx_applied = false;  // game thread

void eye_fx_step(bool on) {
  if (g_created == nullptr || g_view_count.load() == 0) {
    g_eye_fx_applied = false;
    return;
  }
  unsigned char* ours = static_cast<unsigned char*>(g_created);
  if (on) {
    void* got[2] = {};
    for (int i = 0; i < 2; ++i) {
      void* m = nullptr;
      void* o = nullptr;
      if (read_ptr(static_cast<unsigned char*>(g_views[0].view) + kFxSlots[i], &m) && m != nullptr &&
          read_ptr(ours + kFxSlots[i], &o) && o != m) {
        write_ptr(ours + kFxSlots[i], m);
      }
      got[i] = m;
    }
    if (!g_eye_fx_applied) {
      log_line("eyefx: main view slots +0x128 (%p) and +0x138 (%p) shared with the second view", got[0], got[1]);
    }
    g_eye_fx_applied = true;
  } else if (g_eye_fx_applied) {
    for (std::uint32_t off : kFxSlots) {
      write_ptr(ours + off, nullptr);
    }
    g_eye_fx_applied = false;
    log_line("eyefx: second view slots +0x128 / +0x138 cleared");
  }
}

// Field experiment (left-only fauna / waterfall / effects): the view diff 2026-10-05 16:36 showed a few small fields
// that differ between the main view and the second view. Variant k copies the main view's value into the second view
// every StartFrame (after sync_flags); 0 = none. Driven by the automatic sweep (frame_capture) or "fieldtest N".
struct FieldCopy {
  std::uint32_t off;
  std::uint32_t size;
};
constexpr FieldCopy kFieldCopies[] = {{0x48, 4}, {0x25, 1}, {0x20, 4}, {0x1184, 4}};
constexpr int kFieldCount = static_cast<int>(sizeof(kFieldCopies) / sizeof(kFieldCopies[0]));
constexpr int kFieldVariants = kFieldCount + 1;  // one per field, then all of them
std::atomic<int> g_field_variant{0};
int g_field_applied = 0;                 // game thread
std::uint32_t g_field_saved[kFieldCount] = {};
void* g_field_saved_for = nullptr;

bool field_in(int variant, int i) { return variant == kFieldVariants ? true : variant == i + 1; }

void field_step(bool run) {
  const int want = run && g_created != nullptr ? g_field_variant.load(std::memory_order_relaxed) : 0;
  unsigned char* ours = static_cast<unsigned char*>(g_created);
  if (g_created == nullptr || g_view_count.load() == 0) {
    g_field_applied = 0;
    return;
  }
  if (g_field_saved_for != g_created) {
    for (int i = 0; i < kFieldCount; ++i) {
      g_field_saved[i] = 0;
      read_bytes(ours + kFieldCopies[i].off, &g_field_saved[i], kFieldCopies[i].size);
    }
    g_field_saved_for = g_created;
  }
  if (g_field_applied != 0 && g_field_applied != want) {
    for (int i = 0; i < kFieldCount; ++i) {
      if (field_in(g_field_applied, i) && kFieldCopies[i].off != 0x25) {  // +0x25 is rewritten by sync_flags
        write_bytes(ours + kFieldCopies[i].off, &g_field_saved[i], kFieldCopies[i].size);
      }
    }
    log_line("fieldtest: variant %d undone", g_field_applied);
    g_field_applied = 0;
  }
  if (want == 0) {
    return;
  }
  unsigned char* main = static_cast<unsigned char*>(g_views[0].view);
  for (int i = 0; i < kFieldCount; ++i) {
    if (field_in(want, i)) {
      std::uint32_t v = 0;
      if (read_bytes(main + kFieldCopies[i].off, &v, kFieldCopies[i].size)) {
        write_bytes(ours + kFieldCopies[i].off, &v, kFieldCopies[i].size);
      }
    }
  }
  if (g_field_applied != want) {
    log_line("fieldtest: variant %d applied (%s)", want, want == kFieldVariants ? "all fields" : "one field");
  }
  g_field_applied = want;
}

// Game thread (StartFrame): apply or undo the shared slots.
void share_slots(bool on) {
  const int count = g_share_count.load();
  if (g_created == nullptr || g_view_count.load() == 0 || count == 0) {
    g_share_applied = false;
    return;
  }
  unsigned char* ours = static_cast<unsigned char*>(g_created);
  unsigned char* main = static_cast<unsigned char*>(g_views[0].view);
  const int pick = share_pick_now();
  if (on && g_share_applied && pick != g_share_applied_pick) {
    for (int i = 0; i < count; ++i) {
      write_ptr(ours + g_share_off[i], nullptr);  // the selection changed: start clean
    }
    g_share_applied = false;
  }
  if (on) {
    for (int i = 0; i < count; ++i) {
      if (pick != 99 && pick != i + 1) {
        continue;
      }
      void* m = nullptr;
      void* o = nullptr;
      if (read_ptr(main + g_share_off[i], &m) && read_ptr(ours + g_share_off[i], &o) && m != o) {
        write_ptr(ours + g_share_off[i], m);
        g_share_writes.fetch_add(1, std::memory_order_relaxed);
      }
    }
    if (!g_share_applied) {
      if (pick == 99) {
        log_line("sharehooks: all %d main-view slots copied into the second view", count);
      } else if (pick >= 1 && pick <= count) {
        log_line("sharehooks: slot %d (+0x%X) copied into the second view", pick, g_share_off[pick - 1]);
      }
    }
    g_share_applied = true;
    g_share_applied_pick = pick;
  } else if (g_share_applied) {
    for (int i = 0; i < count; ++i) {
      write_ptr(ours + g_share_off[i], nullptr);
    }
    g_share_applied = false;
    log_line("sharehooks: second-view slots cleared again");
  }
}

// Stereo paused: drop the held second-view image and its pose labels so nothing stale reaches the headset, and the
// next resume starts from fresh outputs. The head reference is kept (no recentre on resume).
void pause_output() {
  g_head_prestepped = false;
  {
    std::lock_guard<std::mutex> lock(g_out_mutex);
    if (g_main_held != nullptr) {
      g_main_held->Release();  // never keep the backbuffer referenced (ResizeBuffers)
      g_main_held = nullptr;
    }
  }
  g_last_valid[0] = g_last_valid[1] = false;
  {
    std::lock_guard<std::mutex> lock(g_applied_mutex);
    std::memset(g_slot_pose, 0, sizeof(g_slot_pose));
  }
  g_out_pose.store(0);
  drop_main_eye();
  std::lock_guard<std::mutex> lock(g_out_mutex);
  if (g_out_held != nullptr) {
    g_out_held->Release();
    g_out_held = nullptr;
  }
}

std::uintptr_t StartFrame_hook(void* mgr, void* a2, void* a3, void* a4) {
  g_start_frames.fetch_add(1, std::memory_order_relaxed);
  trace('S', -1, static_cast<std::uint32_t>(g_applied_count), 0);
  const bool want = g_want_create.load(std::memory_order_relaxed) && !g_rescued.load(std::memory_order_relaxed);
  const bool live = g_view_count.load() > 0 && main_live();
  const bool gameplay = update_scene_gate(live);
  xr_runtime::set_gui_panel(native_stereo::ui_layer_active(), static_cast<float>(g_menu_dist_mm.load()) * 1e-3f,
                            static_cast<float>(g_menu_width_mm.load()) * 1e-3f, static_cast<float>(g_menu_height_mm.load()) * 1e-3f);
  log_scene_state(live ? "live" : "idle");
  log_params_changes();
  // The main view's world changes when a level/region loads (start-up menu -> gameplay). A second view built for
  // the old world renders nothing useful, so retire it (inactive forever; never freed) and build a fresh one.
  // Same for the camera object (+0x10) and owner (+0x18): the game swaps the main view's camera when gameplay
  // starts (start-up screens -> gameplay), and a second view still bound to the old camera renders another place.
  if (g_created != nullptr && g_created_main != nullptr) {
    static const std::uint32_t kBound[] = {kWorldOffset, kCameraOffset, 0x18};
    const char* changed = nullptr;
    void* was = nullptr;
    void* now = nullptr;
    for (std::uint32_t off : kBound) {
      void* m = nullptr;
      void* o = nullptr;
      const bool read_own = off == kCameraOffset ? (o = g_created_main_cam, true)
                                                 : read_ptr(static_cast<unsigned char*>(g_created) + off, &o);
      if (read_ptr(static_cast<unsigned char*>(g_created_main) + off, &m) && off == kCameraOffset && m != nullptr && m == g_priv_left) {
        m = g_created_main_cam;  // our head-tracked left camera, not a game change
      }
      if (m != nullptr && read_own && m != o) {
        changed = off == kWorldOffset ? "world" : off == kCameraOffset ? "camera" : "owner";
        was = o;
        now = m;
        break;
      }
    }
    if (changed == nullptr && g_rebuild_requested.exchange(false)) {
      changed = g_rebuild_reason;
      g_rebuild_reason = "rebuild requested";
      was = g_created;
      now = g_created;
    }
    if (changed != nullptr) {
      sync_flags(g_created, false);
      log_line("create: main view %s changed %p -> %p; retiring second view %p (kept inactive) and rebuilding", changed, was, now,
               g_created);
      reset_for_rebuild();
      g_created = nullptr;
      g_created_index.store(-1);
      g_create_failed = false;
      g_retired.fetch_add(1, std::memory_order_relaxed);
    }
  }
  // Stereo runs only in gameplay (see update_scene_gate); on the start-up screens and in the game's menus the second
  // view is kept inactive and the headset gets the mono picture.
  // Window resize in progress (vr_menu::apply_shape): stereo pauses so the swapchain can be resized. Run 2026-10-06
  // 07:26: two ResizeBuffers with stereo running failed with DXGI_ERROR_INVALID_CALL (0x887A0001, references to the
  // buffers still held); the window and the render then disagreed (warp) and the XR runtime failed shortly after.
  const bool resizing = GetTickCount64() < g_resize_hold_until.load(std::memory_order_relaxed);
  // 3D main menu off: no stereo (and no second view) until the front end is over.
  const bool menu_blocked = !g_menu_stereo.load(std::memory_order_relaxed) && g_front_end.load(std::memory_order_relaxed);
  const bool run = want && live && gameplay && !resizing && !menu_blocked;
  {
    const bool was = g_stereo_running.load(std::memory_order_relaxed);
    const bool now_running = run && g_created != nullptr;
    if (was != now_running) {
      if (!now_running) {
        g_last_pause_menu.store(want && !live && !resizing, std::memory_order_relaxed);
        pause_output();
        g_stereo_pauses.fetch_add(1, std::memory_order_relaxed);
        if (g_created != nullptr) {
          log_line("stereo paused (%s); second view %p inactive, mono picture",
                   !want ? "switched off" : !live ? "main view idle" : resizing ? "window resize" : "menu / start-up", g_created);
        }
      } else {
        pause_output();  // an output that arrived after the pause belongs to the old frames
        g_stereo_resumes.fetch_add(1, std::memory_order_relaxed);
        g_stereo_started_at.store(GetTickCount64(), std::memory_order_relaxed);
        log_line("stereo running: second view %p active", g_created);
      }
      g_stereo_running.store(now_running, std::memory_order_relaxed);
    }
  }
  g_stereo_pending.store(want && live && g_scene_set_last && !resizing && !menu_blocked &&
                             !g_stereo_running.load(std::memory_order_relaxed) && !g_create_failed,
                         std::memory_order_relaxed);
  if (run && g_live_streak >= kCreateAfterLiveFrames && g_created == nullptr && !g_create_failed && g_alloc && g_ctor &&
      g_add_view) {
    void* main = g_views[0].view;
    std::uint8_t active = 0;
    read_u8(static_cast<unsigned char*>(main) + 0x24, &active);
    if (active != 0) {  // only while the main view is active and rendering (not paused)
      log_line("create: building second view for main view #0 %p (bare Gfx_View)", main);
      void* view = build_view(main);
      if (view == nullptr) {
        g_create_failed = true;
        log_line("create: construction FAILED; not retrying");
      } else if (!add_view(mgr, view)) {
        g_create_failed = true;
        log_line("create: AddView FAILED for %p; not retrying", view);
      } else {
        g_created = view;
        g_created_main = main;
        read_ptr(static_cast<unsigned char*>(main) + kCameraOffset, &g_created_main_cam);
        const int index = view_index(view);
        g_created_index.store(index);
        void* p10 = nullptr;
        void* p1248 = nullptr;
        void* p1218 = nullptr;
        read_ptr(static_cast<unsigned char*>(view) + 0x10, &p10);
        read_ptr(static_cast<unsigned char*>(view) + 0x1248, &p1248);
        read_ptr(static_cast<unsigned char*>(view) + 0x1218, &p1218);
        log_line("create: second view %p registered as #%d (camera %p world %p cubehelper %p)", view, index, p10, p1248, p1218);
        start_heartbeat();
      }
    }
  }
  bool head_done = false;
  {
    const int cull = g_cullfov_mdeg.load(std::memory_order_relaxed);
    if (cull > 0 && g_created != nullptr && run && g_created_main_cam != nullptr &&
        widen_camera(g_created_main_cam, static_cast<float>(cull) * 1e-3f * 0.0174532925f)) {
      g_cullfov_writes.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (g_created != nullptr && run && g_head.load(std::memory_order_relaxed) && g_helpers_ok && g_created_main_cam != nullptr) {
    const bool stepped = g_head_prestepped || head_step(g_created_main_cam);
    g_head_prestepped = false;
    if (stepped) {
      write_ptr(static_cast<unsigned char*>(g_created_main) + kCameraOffset, g_priv_left);
      write_ptr(static_cast<unsigned char*>(g_created) + kCameraOffset, g_priv_cam);
      g_head_frames.fetch_add(1, std::memory_order_relaxed);
      head_done = true;
    } else {
      g_head_nopose.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (!head_done && g_created_main != nullptr && g_created_main_cam != nullptr && g_priv_left != nullptr) {
    void* cur = nullptr;
    if (read_ptr(static_cast<unsigned char*>(g_created_main) + kCameraOffset, &cur) && cur == g_priv_left) {
      write_ptr(static_cast<unsigned char*>(g_created_main) + kCameraOffset, g_created_main_cam);  // give the game camera back
    }
  }
  if (!g_head.load(std::memory_order_relaxed)) {
    g_head_ref_valid = false;  // recentre next time head tracking is switched on
  }
  if (head_done) {
    // both eyes already set
  } else if (g_created != nullptr && run && g_priv_mode.load(std::memory_order_relaxed) && g_helpers_ok) {
    void* main_cam = nullptr;
    main_cam = g_created_main_cam;
    const float sep = static_cast<float>(g_create_sep_um.load(std::memory_order_relaxed)) * 1e-6f;
    if (main_cam != nullptr && init_priv_camera(main_cam) && update_priv_camera(main_cam, sep)) {
      write_ptr(static_cast<unsigned char*>(g_created) + kCameraOffset, g_priv_cam);
      g_priv_updates.fetch_add(1, std::memory_order_relaxed);
    } else {
      g_priv_failed.fetch_add(1, std::memory_order_relaxed);
    }
  } else if (g_created != nullptr && g_created_main_cam != nullptr) {
    write_ptr(static_cast<unsigned char*>(g_created) + kCameraOffset, g_created_main_cam);  // slot-shift mode
  }
  share_slots(run && share_pick_now() != 0);
  eye_ui_step(run && g_eye_ui.load(std::memory_order_relaxed));
  eye_fx_step(run && g_eye_fx.load(std::memory_order_relaxed));
  if (g_created != nullptr) {
    sync_flags(g_created, run);
    field_step(run);
    if (run) {
      g_created_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
  return g_start_frame(mgr, a2, a3, a4);
}

// SEH frame: find an ID3D12Resource pointer inside an engine texture object (one level deep, then two).
void* find_resource(void* texture, int* where) {
  void* vt = resource_names::resource_vtable();
  if (texture == nullptr || vt == nullptr) {
    return nullptr;
  }
  __try {
    auto* t = static_cast<unsigned char*>(texture);
    for (int off = 0; off < 0x200; off += 8) {
      void* p = *reinterpret_cast<void**>(t + off);
      if (p != nullptr && (reinterpret_cast<std::uintptr_t>(p) & 7) == 0 && *reinterpret_cast<void**>(p) == vt) {
        *where = off;
        return p;
      }
    }
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
  __try {
    auto* t = static_cast<unsigned char*>(texture);
    for (int off = 0; off < 0x100; off += 8) {
      auto* inner = *reinterpret_cast<unsigned char**>(t + off);
      if (inner == nullptr || (reinterpret_cast<std::uintptr_t>(inner) & 7) != 0 ||
          reinterpret_cast<std::uintptr_t>(inner) < 0x10000) {
        continue;
      }
      for (int j = 0; j < 0x100; j += 8) {
        void* p = nullptr;
        if (!read_ptr(inner + j, &p) || p == nullptr || (reinterpret_cast<std::uintptr_t>(p) & 7) != 0) {
          continue;
        }
        void* pv = nullptr;
        if (read_ptr(p, &pv) && pv == vt) {
          *where = 0x1000 + off * 0x100 + j;
          return p;
        }
      }
    }
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
  return nullptr;
}

bool describe(void* resource, D3D12_RESOURCE_DESC* out) {
  __try {
    *out = static_cast<ID3D12Resource*>(resource)->GetDesc();
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

void probe_output(void* out, void* view) {
  void* tex = nullptr;
  if (!read_ptr(static_cast<unsigned char*>(out) + 0x28, &tex) || tex == nullptr) {
    return;
  }
  if (view != g_created) {
    if (g_view_count.load() > 0 && view == g_views[0].view) {
      g_main_texture.store(tex, std::memory_order_relaxed);
      if (kMainOffscreen && eye_mode_on()) {
        int where = -1;
        void* res = find_resource(tex, &where);
        if (res != nullptr) {
          hold_main_eye(res);
          if (res == g_out_resource.load(std::memory_order_relaxed)) {
            g_main_eye_same_as_second.fetch_add(1, std::memory_order_relaxed);  // the pool gave both eyes one texture
          }
        }
        return;  // not the backbuffer: never mixed with the capture hold
      }
      if (g_main_wanted.load(std::memory_order_relaxed)) {
        int where = -1;
        void* res = find_resource(tex, &where);
        if (res != nullptr) {
          hold_main_output(res);
        }
      }
    }
    return;
  }
  g_out_texture.store(tex, std::memory_order_relaxed);
  std::uint32_t w = 0, h = 0, d = 0;
  std::uint8_t f = 0;
  read_u32(static_cast<unsigned char*>(tex) + 0x40, &w);
  read_u32(static_cast<unsigned char*>(tex) + 0x44, &h);
  read_u32(static_cast<unsigned char*>(tex) + 0x48, &d);
  read_u8(static_cast<unsigned char*>(tex) + 0x56, &f);
  g_out_w.store(w);
  g_out_h.store(h);
  g_out_d.store(d);
  g_out_fmt.store(f);
  int where = -1;
  void* res = find_resource(tex, &where);
  if (res != nullptr) {
    g_out_resource.store(res, std::memory_order_relaxed);
    g_out_res_offset.store(where);
    hold_output(res);
  }
}

std::uintptr_t OutputCb_hook(void* out, void* a2, void* ctx, void* a4) {
  const std::uintptr_t r = g_output_cb(out, a2, ctx, a4);
  g_output_calls.fetch_add(1, std::memory_order_relaxed);
  void* c0 = nullptr;
  void* view = nullptr;
  if (g_created != nullptr && ctx != nullptr && read_ptr(ctx, &c0) && c0 != nullptr &&
      read_ptr(static_cast<unsigned char*>(c0) + 8, &view) && view != nullptr) {
    if (view == g_created) {
      g_output_created.fetch_add(1, std::memory_order_relaxed);
      std::uint32_t game = 0, other = 0;
      view_slots(view, &game, &other);
      trace('O', find_view(view), game, other);
      // +0x9EC8 is the slot of the frame this output belongs to (trace 2026-10-05 13:29-13:33: the callback can run
      // after the next frame's camera build has already advanced +0x9EC4; +0x9EC8 still names the output's frame).
      if (other < 8) {
        std::lock_guard<std::mutex> lock(g_applied_mutex);
        g_out_pose.store(g_slot_pose[1][other], std::memory_order_relaxed);
      }
    }
    probe_output(out, view);
  }
  return r;
}

// ---- per-view render size ("eyesize W H [both]" in AvatarVR_native.txt; off by default) -------------------------
// Step 1 (2026-10-09): the second view only (it already renders offscreen, so the flat window is untouched). "both"
// also overrides the main view; only safe once the main view renders offscreen too (next step), otherwise the engine
// would draw a frame of another size into the backbuffer.
using UpdateViewFn = void (*)(void*);
using OutSizeFn = std::uint32_t* (*)(void*, std::uint32_t*);
UpdateViewFn g_update_view = nullptr;
OutSizeFn g_out_size = nullptr;
thread_local void* t_updating_view = nullptr;
std::atomic<std::uint64_t> g_eyesize_applied[2];  // [0] main, [1] second
std::atomic<std::uint64_t> g_outsize_calls{0}, g_outsize_in_update{0}, g_outsize_client{0};
std::atomic<std::uint32_t> g_outsize_last_w{0}, g_outsize_last_h{0};  // what the engine itself returned
// Screen effects (2026-10-23, ISSUE: flash grenade / mount boost / X-ray scan draw a 16:9 letterbox inside the eye).
// [LIVE] eyesize lines: ~17,900 getter calls per 5 s, only ~740 inside a view update (overridden); the rest return the
// window client size (1280x720). [LIVE] F9 14:48-14:50: during the effects the game's final image is a washed-out 16:9
// box inside each eye, i.e. a full-screen pass sized from a non-update call. Probe: callers (return RVA) of the calls
// outside a view update, split by whether an eye view's Render runs on the thread. "outsizerender": those calls made
// inside the main or second view's Render also get the eye size (stereo running only).
std::atomic<bool> g_outsize_render{false};
std::atomic<std::uint64_t> g_outsize_render_applied{0};
struct OutSizeCaller {
  std::uintptr_t rva = 0;
  std::uint64_t calls = 0, in_eye_render = 0;
};
OutSizeCaller g_outsize_callers[48];
std::mutex g_outsize_callers_mutex;
std::atomic<std::uint64_t> g_outsize_callers_overflow{0};

// Eye mode: the main view's offscreen byte. Run 2026-10-09 18:16-18:18: written only after the per-eye camera build,
// the main view still drew into the backbuffer (held output = backbuffer size). The view update resets the frame
// (0x140524920 clears +0xE0A9), so it is also written right after the update. Render_hook counts what the byte holds
// in the game slot (+0x9EC4) and the render slot (+0x9EC8) when the main view renders.
std::atomic<std::uint64_t> g_main_off_writes{0}, g_main_off_seen[2][2];  // [slot game/render][byte 0/1]

bool frame_byte(void* view, std::uint32_t slot_field, std::uint32_t offset, unsigned char** where) {
  void* ring = nullptr;
  std::uint32_t idx = 0;
  unsigned char* v = static_cast<unsigned char*>(view);
  if (!read_ptr(v + kFrameRing, &ring) || ring == nullptr || !read_u32(v + slot_field, &idx) || idx >= 8) {
    return false;
  }
  *where = static_cast<unsigned char*>(ring) + static_cast<std::uintptr_t>(idx) * kFrameStride + offset;
  return true;
}

void UpdateView_hook(void* view) {
  void* const outer = t_updating_view;
  t_updating_view = view;
  g_update_view(view);
  t_updating_view = outer;
  unsigned char* flag = nullptr;
  if (kMainOffscreen && view != nullptr && view == g_created_main && eye_mode_on() && frame_byte(view, kFrameIdxGame, kOffscreenFlag, &flag)) {
    __try_write_u8(flag, 1);
    g_main_off_writes.fetch_add(1, std::memory_order_relaxed);
  }
}

std::uint32_t* OutSize_hook(void* render, std::uint32_t* out) {
  std::uint32_t* const r = g_out_size(render, out);
  g_outsize_calls.fetch_add(1, std::memory_order_relaxed);
  void* const view = t_updating_view;
  if (view == nullptr && out != nullptr) {
    // Outside a view update: record the caller; optionally give eye renders the eye size.
    const int vi = t_view_index;
    void* const rendering = vi >= 0 && vi < kMaxViews ? g_views[vi].view : nullptr;
    const bool eye_render = rendering != nullptr && (vi == 0 || rendering == g_created);
    static const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(GetModuleHandleW(nullptr));
    const std::uintptr_t rva = reinterpret_cast<std::uintptr_t>(_ReturnAddress()) - base;
    {
      std::lock_guard<std::mutex> lock(g_outsize_callers_mutex);
      OutSizeCaller* slot = nullptr;
      for (OutSizeCaller& c : g_outsize_callers) {
        if (c.rva == rva || c.rva == 0) {
          slot = &c;
          break;
        }
      }
      if (slot != nullptr) {
        slot->rva = rva;
        ++slot->calls;
        if (eye_render) ++slot->in_eye_render;
      } else {
        g_outsize_callers_overflow.fetch_add(1, std::memory_order_relaxed);
      }
    }
    const std::uint32_t ew = g_eyesize_w.load(std::memory_order_relaxed), eh = g_eyesize_h.load(std::memory_order_relaxed);
    if (eye_render && g_outsize_render.load(std::memory_order_relaxed) && ew != 0 && eh != 0 &&
        g_stereo_running.load(std::memory_order_relaxed) && !g_rescued.load(std::memory_order_relaxed) &&
        (rendering == g_created || g_eyesize_both.load(std::memory_order_relaxed))) {
      out[0] = ew;
      out[1] = eh;
      g_outsize_render_applied.fetch_add(1, std::memory_order_relaxed);
    }
    return r;
  }
  if (view == nullptr || out == nullptr) {
    return r;
  }
  g_outsize_in_update.fetch_add(1, std::memory_order_relaxed);
  g_outsize_last_w.store(out[0], std::memory_order_relaxed);
  g_outsize_last_h.store(out[1], std::memory_order_relaxed);
  const std::uint32_t w = g_eyesize_w.load(std::memory_order_relaxed), h = g_eyesize_h.load(std::memory_order_relaxed);
  const bool stereo = w != 0 && h != 0 && g_stereo_running.load(std::memory_order_relaxed) && !g_rescued.load(std::memory_order_relaxed);
  const bool second = g_created != nullptr && view == g_created;
  const bool main = !second && g_eyesize_both.load(std::memory_order_relaxed) && g_view_count.load() > 0 && view == g_views[0].view;
  // The second view keeps the eye size even while stereo is paused: each size change reallocates its targets, and the
  // dark-right-eye fault (lost ambient / indirect light until a new second view, DEBUG_TESTERS.md) is suspected to
  // start at such a reallocation [INFERENCE]; pause / menu switches would otherwise resize it every time.
  if ((stereo && (second || main)) || (second && w != 0 && h != 0 && g_eyesize_both.load(std::memory_order_relaxed))) {
    out[0] = w;
    out[1] = h;
    g_eyesize_applied[second ? 1 : 0].fetch_add(1, std::memory_order_relaxed);
    return r;
  }
  // Eye mode, buffers grown past the window: every other view (and the main view in menus) renders at the window's
  // client size, the region the monitor is shown, instead of the larger buffer size.
  unsigned cw = 0, ch = 0;
  if (dx12::client_size(&cw, &ch)) {
    out[0] = cw;
    out[1] = ch;
    g_outsize_client.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

void log_eyesize() {
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last < 5000 || g_out_size == nullptr) {
    return;
  }
  last = now;
  {
    OutSizeCaller copy[48];
    {
      std::lock_guard<std::mutex> lock(g_outsize_callers_mutex);
      std::memcpy(copy, g_outsize_callers, sizeof(copy));
      for (OutSizeCaller& c : g_outsize_callers) c.calls = c.in_eye_render = 0;
    }
    std::sort(std::begin(copy), std::end(copy), [](const OutSizeCaller& a, const OutSizeCaller& b) { return a.calls > b.calls; });
    char text[900];
    int n = std::snprintf(text, sizeof(text), "outsize callers (outside view update, 5 s; rva calls/in eye render): %s applied %llu |",
                          g_outsize_render.load() ? "outsizerender ON" : "outsizerender off",
                          static_cast<unsigned long long>(g_outsize_render_applied.exchange(0)));
    for (const OutSizeCaller& c : copy) {
      if (c.calls == 0 || n <= 0 || n >= static_cast<int>(sizeof(text)) - 40) break;
      n += std::snprintf(text + n, sizeof(text) - n, " %llX %llu/%llu", static_cast<unsigned long long>(c.rva),
                         static_cast<unsigned long long>(c.calls), static_cast<unsigned long long>(c.in_eye_render));
    }
    log_line("%s", text);
  }
  log_line("eyesize: want %ux%u%s | engine output size %ux%u | getter calls %llu (in view update %llu) | applied main %llu "
           "second %llu | second output texture %ux%u | backbuffer %ux%u",
           g_eyesize_w.load(), g_eyesize_h.load(), g_eyesize_both.load() ? " both" : " second only", g_outsize_last_w.load(),
           g_outsize_last_h.load(), static_cast<unsigned long long>(g_outsize_calls.exchange(0)),
           static_cast<unsigned long long>(g_outsize_in_update.exchange(0)),
           static_cast<unsigned long long>(g_eyesize_applied[0].exchange(0)),
           static_cast<unsigned long long>(g_eyesize_applied[1].exchange(0)), g_out_w.load(), g_out_h.load(), g_render_w.load(),
           g_render_h.load());
  log_line("eyesize: eye mode %s | main eye output changes %llu, same texture as second %llu", eye_mode_on() ? "ON" : "off",
           static_cast<unsigned long long>(g_main_eye_changes.exchange(0)),
           static_cast<unsigned long long>(g_main_eye_same_as_second.exchange(0)));
  log_line("eyesize: views given the window client size %llu", static_cast<unsigned long long>(g_outsize_client.exchange(0)));
  log_line("eyesize: main offscreen byte writes after update %llu | at main Render: game slot set %llu clear %llu, render slot "
           "set %llu clear %llu",
           static_cast<unsigned long long>(g_main_off_writes.exchange(0)), static_cast<unsigned long long>(g_main_off_seen[0][1].exchange(0)),
           static_cast<unsigned long long>(g_main_off_seen[0][0].exchange(0)), static_cast<unsigned long long>(g_main_off_seen[1][1].exchange(0)),
           static_cast<unsigned long long>(g_main_off_seen[1][0].exchange(0)));
  head_pose::Snapshot optics;
  if (xr_runtime::latest_views(&optics)) {
    std::lock_guard<std::mutex> lock(g_ui_tan_mutex);
    for (int e = 0; e < 2; ++e) {
      if (!g_ui_tan_valid[e]) continue;
      const float* t = g_ui_tan[e];
      const float* f = optics.eye[e].fov;
      log_line("optics eye %d: requested rendered rays %.6f %.6f %.6f %.6f | runtime lens rays %.6f %.6f %.6f %.6f",
               e,t[0],t[1],t[2],t[3],std::tan(f[0]),std::tan(f[1]),std::tan(f[2]),std::tan(f[3]));
    }
  }
  for (int e = 0; e < 2; ++e) {
    const auto samples = g_observed_projection_count[e].exchange(0);
    const float m0 = g_observed_projection[e][0].load(), m5 = g_observed_projection[e][1].load();
    const float m8 = g_observed_projection[e][2].load(), m9 = g_observed_projection[e][3].load();
    if (samples > 0 && m0 > 0.01f && m5 > 0.01f) {
      log_line("projection candidate eye %d: samples %llu M %.6f %.6f %.6f %.6f rays %.6f %.6f %.6f %.6f",
               e, static_cast<unsigned long long>(samples), m0, m5, m8, m9,
               (-1.f-m8)/m0, (1.f-m8)/m0, (1.f-m9)/m5, (-1.f-m9)/m5);
    }
  }
  if (g_lens_projection.load()) {
    head_pose::Snapshot snap;
    if (xr_runtime::latest_views(&snap)) {
      for (int e = 0; e < 2; ++e) {
        lens_projection::Projection optical = {};
        if (lens_projection::from_fov(snap.eye[e].fov, &optical)) {
          log_line("lens eye %d: optical aspect %.6f rays %.6f %.6f %.6f %.6f | image %ux%u",
                   e, optical.aspect, optical.tan4[0], optical.tan4[1], optical.tan4[2], optical.tan4[3],
                   g_eyesize_w.load(), g_eyesize_h.load());
        }
      }
    }
  }
  log_line("eyesize: camera rect writes main %llu second %llu", static_cast<unsigned long long>(g_eye_rect_writes[0].exchange(0)),
           static_cast<unsigned long long>(g_eye_rect_writes[1].exchange(0)));
}

// ---- vegetation / animation clock sync -------------------------------------------------------------------------
// [AUTHOR] 2026-10-06: wind-animated vegetation is out of phase between the eyes (main menu capture: the red plant's
// leaves differ), while everything static matches. [INFERENCE] the wind animation reads a clock kept per view (in the
// view or in its per-frame data), and the second view's clock started when it was built, so it runs at a constant
// offset. Discovery: once a second (game thread, after the second view's camera build) the main view's and the second
// view's frame data and view object are compared with the previous sample; a float (or double) that advances by the
// elapsed wall time in BOTH views but holds a different value in each is a per-view clock. One that keeps the same
// offset for 3 samples is locked, and from then on the main view's value is copied into the second view every frame
// ("novegsync" turns the copy off; discovery keeps logging).
std::atomic<bool> g_vegsync{true};
// Auto-lock disabled after run 2026-10-06 08:45: it locked 8 fields that were camera-dependent vectors (their advance
// rate changed every sample), not clocks. Discovery keeps logging; nothing is copied.
constexpr bool g_clock_autolock = false;
constexpr bool kClockDiscovery = false;  // 2026-10-07: wind found (wind_sync); discovery logging off
struct ClockSlot {
  std::uint8_t region;  // see kRegionName
  std::uint8_t dbl;     // 1 = double
  std::uint32_t off;
};
constexpr int kMaxClocks = 8;
ClockSlot g_clocks[kMaxClocks];
std::atomic<int> g_clock_count{0};
std::atomic<std::uint64_t> g_clock_copies{0};

unsigned char* frame_of(void* view) {
  void* ring = nullptr;
  std::uint32_t idx = 0;
  unsigned char* v = static_cast<unsigned char*>(view);
  if (view == nullptr || !read_ptr(v + kFrameRing, &ring) || ring == nullptr || !read_u32(v + kFrameIdxGame, &idx) || idx >= 8) {
    return nullptr;
  }
  return static_cast<unsigned char*>(ring) + static_cast<std::uintptr_t>(idx) * kFrameStride;
}

// Regions compared between the main view and the second view: 0 = current frame data, 1 = view object, 2/3 = the
// per-view objects the views point to at +0x1190 / +0x49A8 (each view has its own; first 0x2000 bytes).
constexpr int kClockRegions = 4;
const std::size_t kRegionSize[kClockRegions] = {kFrameStride, kFrameRing, 0x2000, 0x2000};
const char* const kRegionName[kClockRegions] = {"frame", "view", "view+1190", "view+49A8"};

unsigned char* region_of(void* view, int r) {
  if (view == nullptr) {
    return nullptr;
  }
  if (r == 0) {
    return frame_of(view);
  }
  if (r == 1) {
    return static_cast<unsigned char*>(view);
  }
  void* p = nullptr;
  read_ptr(static_cast<unsigned char*>(view) + (r == 2 ? 0x1190 : 0x49A8), &p);
  return static_cast<unsigned char*>(p);
}

// Lag test 2026-10-06 08:32 (DLSS off, zero separation): the right eye matches the same left frame best, yet differs by
// about two frames of motion, only in foliage: the animation runs at a different phase per view. The first detector
// only accepted fields advancing at exactly 1/s; this one accepts any field that advances by the same nonzero amount in
// both views between samples (an accumulator at any rate) while holding a different value in each. Stand still while it
// samples: a moving camera also advances position fields in both views.
void clock_discover() {
  static std::vector<unsigned char> pm[kClockRegions], pc[kClockRegions];
  static ULONGLONG prev_t = 0;
  static std::vector<std::uint32_t> cand_off[kClockRegions][2];
  static std::vector<double> cand_diff[kClockRegions][2];
  static std::vector<int> cand_hits[kClockRegions][2];
  static void* sampled_for = nullptr;
  const ULONGLONG now = GetTickCount64();
  const ULONGLONG every = g_clock_count.load() > 0 ? 10000 : 1000;
  if (now - prev_t < every && sampled_for == g_created) {
    return;
  }
  void* mainv = g_created_main;
  void* sec = g_created;
  std::vector<unsigned char> cm[kClockRegions], cc[kClockRegions];
  for (int r = 0; r < kClockRegions; ++r) {
    unsigned char* a = region_of(mainv, r);
    unsigned char* b = region_of(sec, r);
    if (a == nullptr || b == nullptr || a == b) {
      continue;  // missing or shared: nothing per view here
    }
    cm[r].resize(kRegionSize[r]);
    cc[r].resize(kRegionSize[r]);
    if (!read_bytes(a, cm[r].data(), kRegionSize[r]) || !read_bytes(b, cc[r].data(), kRegionSize[r])) {
      cm[r].clear();
      cc[r].clear();
    }
  }
  const bool have_prev = sampled_for == sec;
  const double dt = static_cast<double>(now - prev_t) * 1e-3;
  if (have_prev && dt > 0.5 && dt < 12.0) {
    int logged = 0;
    for (int r = 0; r < kClockRegions; ++r) {
      if (cm[r].empty() || pm[r].size() != cm[r].size()) {
        continue;
      }
      for (int d = 0; d < 2; ++d) {
        std::vector<std::uint32_t> offs;
        std::vector<double> diffs;
        std::vector<int> hits;
        const std::size_t step = d ? 8 : 4;
        for (std::size_t o = 0; o + step <= cm[r].size(); o += step) {
          double m, c, m0, c0;
          if (d) {
            std::memcpy(&m, cm[r].data() + o, 8);
            std::memcpy(&c, cc[r].data() + o, 8);
            std::memcpy(&m0, pm[r].data() + o, 8);
            std::memcpy(&c0, pc[r].data() + o, 8);
          } else {
            float f[4];
            std::memcpy(&f[0], cm[r].data() + o, 4);
            std::memcpy(&f[1], cc[r].data() + o, 4);
            std::memcpy(&f[2], pm[r].data() + o, 4);
            std::memcpy(&f[3], pc[r].data() + o, 4);
            m = f[0];
            c = f[1];
            m0 = f[2];
            c0 = f[3];
          }
          if (!std::isfinite(m) || !std::isfinite(c) || !std::isfinite(m0) || !std::isfinite(c0) || std::fabs(m) > 1e7) {
            continue;
          }
          const double dm = m - m0, dc = c - c0;
          // same nonzero advance in both views (any rate), different value; 0.1 excludes eye-separation offsets
          if (std::fabs(dm) > 1e-3 && std::fabs(dm - dc) < 0.02 * std::fabs(dm) + 1e-5 && std::fabs(m - c) > 0.1) {
            const double diff = m - c;
            int h = 1;
            for (std::size_t k = 0; k < cand_off[r][d].size(); ++k) {
              if (cand_off[r][d][k] == o && std::fabs(cand_diff[r][d][k] - diff) < 0.02 * std::fabs(diff) + 1e-3) {
                h = cand_hits[r][d][k] + 1;
              }
            }
            offs.push_back(static_cast<std::uint32_t>(o));
            diffs.push_back(diff);
            hits.push_back(h);
            if (logged < 20) {
              log_line("clock: %s +0x%05X %s main %.4f second %.4f (offset %.4f, advance %.4f/s, stable %d)", kRegionName[r],
                       static_cast<unsigned>(o), d ? "double" : "float ", m, c, diff, dm / dt, h);
              ++logged;
            }
            if (h >= 3 && g_clock_autolock) {
              bool known = false;
              const int n = g_clock_count.load();
              for (int i = 0; i < n; ++i) {
                known = known || (g_clocks[i].region == r && g_clocks[i].dbl == d && g_clocks[i].off == o);
              }
              if (!known && n < kMaxClocks) {
                g_clocks[n] = {static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(d), static_cast<std::uint32_t>(o)};
                g_clock_count.store(n + 1);
                log_line("clock: LOCKED %s +0x%05X (%s); main value copied into the second view every frame", kRegionName[r],
                         static_cast<unsigned>(o), d ? "double" : "float");
              }
            }
          }
        }
        cand_off[r][d] = offs;
        cand_diff[r][d] = diffs;
        cand_hits[r][d] = hits;
      }
    }
    if (logged == 0) {
      static int quiet = 0;
      if (++quiet % 10 == 1) {
        log_line("clock: no per-view accumulator candidates this sample (dt %.2f s)", dt);
      }
    }
  }
  for (int r = 0; r < kClockRegions; ++r) {
    pm[r].swap(cm[r]);
    pc[r].swap(cc[r]);
  }
  prev_t = now;
  sampled_for = sec;
}

// Game thread, after the second view's camera build (the main view's ran earlier this frame).
void clock_sync() {
  const int n = g_clock_count.load(std::memory_order_relaxed);
  if (n == 0 || !g_vegsync.load(std::memory_order_relaxed)) {
    return;
  }
  for (int i = 0; i < n; ++i) {
    const ClockSlot& c = g_clocks[i];
    unsigned char* src = region_of(g_created_main, c.region);
    unsigned char* dst = region_of(g_created, c.region);
    if (src == nullptr || dst == nullptr || src == dst) {
      continue;
    }
    unsigned char v[8];
    const std::size_t size = c.dbl ? 8 : 4;
    if (read_bytes(src + c.off, v, size) && write_bytes(dst + c.off, v, size)) {
      g_clock_copies.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

// ---- TAA / DLSS jitter sync ----------------------------------------------------------------------------------
// [LIVE] run 2026-10-06 09:05 (7094283f): frame data +0x0 / +0x4 step through Halton(2,3) - 0.5 values (0.125, 0.0556,
// -0.3889) and differ between the main and the second view; the eyes' camera constant blocks differ at +0A0/+0A4 by
// sub-pixel amounts on top of the per-eye projection offset (09:06:39: 0.040024 | -0.039648). [INFERENCE] each view runs
// its own jitter sequence at a different phase, so temporal reconstruction (TAA/DLSS) resolves moving foliage
// differently per eye. The main view's jitter pair is copied into the second view's frame data after the second
// view's camera build, under the "Sync animation" toggle. OPEN: whether the constant block is built from +0x0/+0x4
// after this point; discriminator: the "jitter:" log line (cb y offsets equal = applied).
// Per-eye camera constant blocks: see note_eye_cb.
alignas(16) unsigned char g_eye_cb[2][1536];
std::atomic<ULONGLONG> g_eye_cb_at[2];
std::mutex g_eye_cb_mutex;
constexpr std::size_t kJitterOffset = 0x0;
std::atomic<std::uint64_t> g_jitter_copies{0};
// "jittersync": copy the main view's jitter into the second view (off by default). [LIVE] 2026-10-08 14:16-14:18, same
// spot, head still: right-eye detail (mean |Laplacian|, centre 60%) was 66% of the left eye with the copy on and 93%
// with it off. The copy lands after the second view's camera build, so DLSS is told a jitter the view did not render
// with and resolves its samples at the wrong offsets.
std::atomic<bool> g_jitter_sync{false};

void jitter_sync() {
  if (!g_vegsync.load(std::memory_order_relaxed) || !g_jitter_sync.load(std::memory_order_relaxed)) {
    return;
  }
  unsigned char* src = frame_of(g_created_main);
  unsigned char* dst = frame_of(g_created);
  if (src == nullptr || dst == nullptr || src == dst) {
    return;
  }
  float j[2];
  if (!read_bytes(src + kJitterOffset, j, sizeof(j)) || !std::isfinite(j[0]) || !std::isfinite(j[1]) ||
      std::fabs(j[0]) > 1.f || std::fabs(j[1]) > 1.f) {
    return;  // not a sub-pixel pair: leave the second view alone
  }
  if (write_bytes(dst + kJitterOffset, j, sizeof(j))) {
    g_jitter_copies.fetch_add(1, std::memory_order_relaxed);
  }
}

void log_jitter() {
  float m[2] = {}, c[2] = {};
  unsigned char* a = frame_of(g_created_main);
  unsigned char* b = frame_of(g_created);
  const bool ok = a != nullptr && b != nullptr && read_bytes(a + kJitterOffset, m, sizeof(m)) &&
                  read_bytes(b + kJitterOffset, c, sizeof(c));
  float ya = 0.f, yb = 0.f;
  {
    std::lock_guard<std::mutex> lock(g_eye_cb_mutex);
    std::memcpy(&ya, g_eye_cb[0] + 0xA4, 4);
    std::memcpy(&yb, g_eye_cb[1] + 0xA4, 4);
  }
  log_line("jitter: sync %s, %llu copies; frame+0 main (%.4f, %.4f) second (%.4f, %.4f)%s; cb +0A4 %.6g | %.6g (%s)",
           g_vegsync.load() && g_jitter_sync.load() ? "on" : "off", static_cast<unsigned long long>(g_jitter_copies.load()), m[0], m[1], c[0], c[1],
           ok ? "" : " (unread)", ya, yb, ya == yb ? "equal" : "differ");
}

// ---- wind search (Mod -> "Wind search") ------------------------------------------------------------------------
// [AUTHOR] 2026-10-06: in the headset both eyes' plants are sharp but sway differently -> a per-view wind state, not
// temporal reconstruction. [LIVE] it is in no constant block (cbprobe), no pass slot (foliage sweep) and not in the
// frame data / view+0x1190 / view+0x49A8 as an accumulator (clock discovery). Snowdrop ships without RTTI names, so the
// objects are found by behaviour: with both eyes at one point and the head still, camera-derived fields are equal in
// both views, and what keeps changing while differing between the views is animation state. Every heap pointer the
// two view objects hold with different values (the second view's own object vs the main view's) is followed one level
// and its first 0x400 bytes sampled 6 times, 0.4 s apart. Read-only; results as "wind:" lines.
struct WindPair {
  std::uint32_t off;
  std::uintptr_t a, b;
};
std::atomic<int> g_wind_phase{0};  // 0 idle, 1 settling, 2 sampling, 3 done (waiting for zero separation release)
ULONGLONG g_wind_t = 0;
int g_wind_sample = 0;
constexpr int kWindSamples = 6;
constexpr std::size_t kWindBytes = 0x400;
constexpr std::size_t kWindMaxPairs = 900;
std::vector<WindPair> g_wind_pairs;
std::vector<float> g_wind_data[kWindSamples];  // per sample: pair-major, main then second, kWindBytes/4 floats each

void wind_collect_pairs() {
  g_wind_pairs.clear();
  auto* a = static_cast<unsigned char*>(g_created_main);
  auto* b = static_cast<unsigned char*>(g_created);
  std::vector<std::uint64_t> qa(kViewObjectSize / 8), qb(kViewObjectSize / 8);
  if (a == nullptr || b == nullptr || !read_bytes(a, qa.data(), qa.size() * 8) || !read_bytes(b, qb.data(), qb.size() * 8)) {
    return;
  }
  const auto mod_lo = reinterpret_cast<std::uintptr_t>(g_module_base);
  const auto mod_hi = mod_lo + g_image_size;
  auto heap = [&](std::uint64_t v) {
    return v >= 0x10000 && v < 0x00007FFFFFFFFFFFull && (v & 7) == 0 && !(v >= mod_lo && v < mod_hi);
  };
  const auto va = reinterpret_cast<std::uintptr_t>(a), vb = reinterpret_cast<std::uintptr_t>(b);
  for (std::size_t i = 0; i < qa.size() && g_wind_pairs.size() < kWindMaxPairs; ++i) {
    const std::uint64_t x = qa[i], y = qb[i];
    if (x == y || !heap(x) || !heap(y)) {
      continue;
    }
    // pointers into the view objects themselves (self links) say nothing about separate state
    if ((x >= va && x < va + kViewObjectSize) || (y >= vb && y < vb + kViewObjectSize)) {
      continue;
    }
    g_wind_pairs.push_back({static_cast<std::uint32_t>(i * 8), static_cast<std::uintptr_t>(x), static_cast<std::uintptr_t>(y)});
  }
}

void wind_take_sample(int k) {
  const std::size_t floats = kWindBytes / 4;
  std::vector<float>& d = g_wind_data[k];
  d.assign(g_wind_pairs.size() * 2 * floats, std::numeric_limits<float>::quiet_NaN());
  for (std::size_t p = 0; p < g_wind_pairs.size(); ++p) {
    float* m = d.data() + p * 2 * floats;
    if (!read_bytes(reinterpret_cast<void*>(g_wind_pairs[p].a), m, kWindBytes) ||
        !read_bytes(reinterpret_cast<void*>(g_wind_pairs[p].b), m + floats, kWindBytes)) {
      std::fill(m, m + 2 * floats, std::numeric_limits<float>::quiet_NaN());
    }
  }
}

void wind_analyse() {
  const std::size_t floats = kWindBytes / 4;
  int found = 0, accum = 0, logged = 0;
  for (int pass = 0; pass < 2; ++pass) {  // accumulators first, then any other moving field
    for (std::size_t p = 0; p < g_wind_pairs.size(); ++p) {
      for (std::size_t f = 0; f < floats; ++f) {
        float m[kWindSamples], c[kWindSamples];
        bool ok = true;
        for (int k = 0; k < kWindSamples && ok; ++k) {
          m[k] = g_wind_data[k][p * 2 * floats + f];
          c[k] = g_wind_data[k][p * 2 * floats + floats + f];
          ok = std::isfinite(m[k]) && std::isfinite(c[k]) && std::fabs(m[k]) < 1e6f && std::fabs(c[k]) < 1e6f &&
               (m[k] == 0.f || std::isnormal(m[k])) && (c[k] == 0.f || std::isnormal(c[k])) && m[k] != c[k];
        }
        if (!ok) {
          continue;
        }
        int cm = 0, cc = 0, up_m = 0, up_c = 0;
        for (int k = 1; k < kWindSamples; ++k) {
          cm += m[k] != m[k - 1];
          cc += c[k] != c[k - 1];
          up_m += m[k] > m[k - 1];
          up_c += c[k] > c[k - 1];
        }
        if (cm < kWindSamples - 2 || cc < kWindSamples - 2) {
          continue;  // must keep changing in both views
        }
        const bool acc = (up_m == kWindSamples - 1 && up_c == kWindSamples - 1) || (up_m == 0 && up_c == 0);
        if ((pass == 0) != acc) {
          continue;
        }
        ++found;
        accum += acc ? 1 : 0;
        if (logged < 70) {
          ++logged;
          log_line("wind: view+0x%04X -> +0x%03X %s main %.5g %.5g %.5g %.5g %.5g %.5g | second %.5g %.5g %.5g %.5g %.5g %.5g",
                   g_wind_pairs[p].off, static_cast<unsigned>(f * 4), acc ? "ACCUM " : "moving", m[0], m[1], m[2], m[3], m[4],
                   m[5], c[0], c[1], c[2], c[3], c[4], c[5]);
        }
      }
    }
  }
  log_line("wind: done; %zu per-view objects sampled, %d fields keep changing and differ between the views (%d accumulators)",
           g_wind_pairs.size(), found, accum);
  for (auto& v : g_wind_data) {
    v.clear();
    v.shrink_to_fit();
  }
}

// Game thread, after the second view's camera build.
void wind_step() {
  const int phase = g_wind_phase.load(std::memory_order_relaxed);
  if (phase != 1 && phase != 2) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (phase == 1) {
    if (now - g_wind_t < 1000) {
      return;
    }
    wind_collect_pairs();
    log_line("wind: sampling %zu per-view objects (pointers that differ between the main and the second view)", g_wind_pairs.size());
    if (g_wind_pairs.empty()) {
      g_zero_sep_forced.store(false);
      g_wind_phase.store(0);
      return;
    }
    g_wind_sample = 0;
    wind_take_sample(g_wind_sample++);
    g_wind_t = now;
    g_wind_phase.store(2);
    return;
  }
  if (now - g_wind_t < 400) {
    return;
  }
  g_wind_t = now;
  wind_take_sample(g_wind_sample++);
  if (g_wind_sample == kWindSamples) {
    g_zero_sep_forced.store(false);
    wind_analyse();
    g_wind_phase.store(0);
  }
}

// ---- particle visibility bit for the second view ("fxbits", 2026-10-09) --------------------------------------------
// [LIVE] viewindex 11:46-11:47: in the Present-thread samples the views hold +0x1254 = 1000000 / 1000001 and +0x26 = 0.
// [STATIC] that is the end-of-frame state: each view takes a render-order index in its pre-render
// (0x140568850 / 0x14058B270: index = manager +0x50 counter, then counter + 1; main view 0, second view 1), and the
// end-of-frame pass (0x140535480) clears +0x26 and adds 1000000. The particle cull job 0x1404A4AA0 (rcx, rdx = per-
// job view entries, r8 = particle systems, 0x28 bytes each, system at +0x10; r9d = count) includes a system when its
// mask (+0xE4) has bit (view index & 31). Only the main view's bit gets set, so the second view gets none.
// "fxbits" (on by default since 2026-10-09; "nofxbits" turns it off): before each second-view call, every system whose
// mask has the main view's bit also gets the second view's bit. Testers used to find this: research/DEBUG_TESTERS.md.
constexpr std::uintptr_t kParticleCullRva = 0x4A4AA0;
constexpr unsigned char kParticleCullSig[] = {0x48, 0x8B, 0xC4, 0x44, 0x89, 0x48, 0x20, 0x4C, 0x89, 0x40, 0x18};
using ParticleCullFn = void (*)(void*, void*, void*, void*);
ParticleCullFn g_particle_cull = nullptr;
std::atomic<bool> g_fx_bits{true};
std::atomic<std::uint64_t> g_fx_bits_calls{0}, g_fx_bits_added{0}, g_fx_bits_seen{0};
std::atomic<std::int32_t> g_fx_bits_main{-1}, g_fx_bits_second{-1};

// [LIVE] 11:52:47 fxbits on: fire, flames and smoke appear in the right eye (fxon115245 captures), then the game stops
// presenting about 6 s later (process gone). [STATIC] the job writes per-system state (bounds +0xEC..+0x148, flags +0x11C,
// a component scratch block holding the view) and FUN_143211e60 evaluates the system. A system in both views' lists is
// then written by a main-view chunk and a second-view chunk on two worker threads at once, and the persistent extra bit
// also lets later views (any view that takes that index) pick it up.
// So: the second view's chunks run exclusive of every main/second chunk (main chunks stay parallel among themselves), the
// bit is added only for that chunk's systems and removed right after the call.
SRWLOCK g_fx_lock = SRWLOCK_INIT;

void* particle_cull_view(void* entries) {
  void* list = nullptr;
  void* view = nullptr;
  if (entries == nullptr || !read_ptr(entries, &list) || list == nullptr || !read_ptr(list, &view)) {
    return nullptr;
  }
  return view;
}

void ParticleCull_hook(void* a1, void* entries, void* systems, void* count_arg) {
  if (!g_fx_bits.load(std::memory_order_relaxed) || systems == nullptr || g_created == nullptr || g_created_main == nullptr ||
      !g_stereo_running.load(std::memory_order_relaxed)) {
    g_particle_cull(a1, entries, systems, count_arg);
    return;
  }
  void* view = particle_cull_view(entries);
  if (view == g_created_main) {
    AcquireSRWLockShared(&g_fx_lock);
    g_particle_cull(a1, entries, systems, count_arg);
    ReleaseSRWLockShared(&g_fx_lock);
    return;
  }
  if (view != g_created) {
    g_particle_cull(a1, entries, systems, count_arg);
    return;
  }
  AcquireSRWLockExclusive(&g_fx_lock);
  static std::vector<unsigned char*> added_list;
  added_list.clear();
  std::uint32_t sb = 0;
  {
    std::int32_t mi = -1, si = -1;
    read_bytes(static_cast<unsigned char*>(g_created_main) + 0x1254, &mi, 4);
    read_bytes(static_cast<unsigned char*>(g_created) + 0x1254, &si, 4);
    g_fx_bits_main.store(mi, std::memory_order_relaxed);
    g_fx_bits_second.store(si, std::memory_order_relaxed);
    g_fx_bits_calls.fetch_add(1, std::memory_order_relaxed);
    const std::uint32_t count = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(count_arg));
    if (mi >= 0 && si >= 0 && mi < 1000000 && si < 1000000 && (mi & 31) != (si & 31) && count < 100000) {
      const std::uint32_t mb = 1u << (mi & 31);
      sb = 1u << (si & 31);
      std::uint64_t added = 0, seen = 0;
      for (std::uint32_t i = 0; i < count; ++i) {
        void* sys = nullptr;
        std::uint32_t m = 0;
        if (!read_ptr(static_cast<unsigned char*>(systems) + static_cast<std::uintptr_t>(i) * 0x28 + 0x10, &sys) || sys == nullptr ||
            !read_u32(static_cast<unsigned char*>(sys) + 0xE4, &m)) {
          continue;
        }
        if ((m & mb) != 0) {
          ++seen;
          if ((m & sb) == 0) {
            InterlockedOr(reinterpret_cast<volatile LONG*>(static_cast<unsigned char*>(sys) + 0xE4), static_cast<LONG>(sb));
            ++added;
            added_list.push_back(static_cast<unsigned char*>(sys));
          }
        }
      }
      g_fx_bits_seen.fetch_add(seen, std::memory_order_relaxed);
      g_fx_bits_added.fetch_add(added, std::memory_order_relaxed);
    }
  }
  g_particle_cull(a1, entries, systems, count_arg);
  for (unsigned char* sys : added_list) {
    InterlockedAnd(reinterpret_cast<volatile LONG*>(sys + 0xE4), static_cast<LONG>(~sb));
  }
  ReleaseSRWLockExclusive(&g_fx_lock);
}

void log_fx_bits() {
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (g_particle_cull == nullptr || now - last < 20000) {
    return;
  }
  last = now;
  log_line("fxbits: %s | cull jobs %llu | systems visible to the main view %llu | second-view bit added %llu | last index main %d second %d",
           g_fx_bits.load() ? "on" : "off", static_cast<unsigned long long>(g_fx_bits_calls.exchange(0)),
           static_cast<unsigned long long>(g_fx_bits_seen.exchange(0)), static_cast<unsigned long long>(g_fx_bits_added.exchange(0)),
           g_fx_bits_main.load(), g_fx_bits_second.load());
}

// ---- wind sync ---------------------------------------------------------------------------------------------------
// [LIVE] wind search 2026-10-07 05:40 (93187511): the object at view+0x49A8 holds a 3-vector at +0x2C (and the previous
// frame's at +0x58, one frame behind) that advances at the same rate in both views, ~(-1.9, +0.7, +2.0)/s, with a
// large constant gap: main (-67.6, 32.5, 112.0), second (-35.9, 13.5, 38.4). [INFERENCE] the wind scroll accumulator:
// each view integrates wind since it was created, so the second view's foliage sways at another phase. The same
// values appear behind other view pointers (+0x4A98, +0x4AE8 ... +0x7DD8), as copies or the same object; all slots of
// the second view whose pointee's +0x2C equals its own accumulator get the main view's current and previous vectors,
// before and after the second view's camera build. Under Stereo -> "Sync animation".
constexpr std::uint32_t kWindObj = 0x49A8, kWindCur = 0x2C, kWindPrev = 0x58;
std::vector<std::uint32_t> g_wind_slots;
void* g_wind_for = nullptr;
ULONGLONG g_wind_scanned = 0;
std::atomic<std::uint64_t> g_wind_copies{0};
std::atomic<int> g_wind_slot_count{0};

bool read_vec3(std::uintptr_t p, float* v) {
  return p != 0 && read_bytes(reinterpret_cast<void*>(p), v, 12) && std::isfinite(v[0]) && std::isfinite(v[1]) &&
         std::isfinite(v[2]) && std::fabs(v[0]) < 1e7f && std::fabs(v[1]) < 1e7f && std::fabs(v[2]) < 1e7f;
}

void wind_sync() {
  if (!g_vegsync.load(std::memory_order_relaxed) || g_created == nullptr || g_created_main == nullptr) {
    return;
  }
  auto* mv = static_cast<unsigned char*>(g_created_main);
  auto* sv = static_cast<unsigned char*>(g_created);
  void* pm = nullptr;
  void* ps = nullptr;
  if (!read_ptr(mv + kWindObj, &pm) || !read_ptr(sv + kWindObj, &ps) || pm == nullptr || ps == nullptr || pm == ps) {
    return;
  }
  const auto om = reinterpret_cast<std::uintptr_t>(pm), os = reinterpret_cast<std::uintptr_t>(ps);
  float cur[3], prev[3], own[3];
  if (!read_vec3(om + kWindCur, cur) || !read_vec3(om + kWindPrev, prev) || !read_vec3(os + kWindCur, own)) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (g_wind_for != g_created || now - g_wind_scanned > 10000) {
    // Slots of the second view whose object carries the second view's own accumulator (bitwise equal).
    g_wind_slots.clear();
    for (std::uint32_t off = 0; off + 8 <= kViewObjectSize; off += 8) {
      void* q = nullptr;
      float v[3];
      if (read_ptr(sv + off, &q) && q != nullptr && q != pm && read_vec3(reinterpret_cast<std::uintptr_t>(q) + kWindCur, v) &&
          std::memcmp(v, own, 12) == 0) {
        g_wind_slots.push_back(off);
      }
    }
    if (g_wind_for != g_created) {
      char line[256];
      int n = 0;
      for (std::size_t i = 0; i < g_wind_slots.size() && i < 16; ++i) {
        n += snprintf(line + n, sizeof(line) - n, " +0x%04X", g_wind_slots[i]);
      }
      line[n] = '\0';
      log_line("windsync: %zu second-view slots carry its wind accumulator:%s", g_wind_slots.size(), n ? line : " -");
    }
    g_wind_for = g_created;
    g_wind_scanned = now;
    g_wind_slot_count.store(static_cast<int>(g_wind_slots.size()));
  }
  for (std::uint32_t off : g_wind_slots) {
    void* q = nullptr;
    float qc[3], qp[3];
    if (!read_ptr(sv + off, &q) || q == nullptr || q == pm) {
      continue;
    }
    const auto oq = reinterpret_cast<std::uintptr_t>(q);
    if (!read_vec3(oq + kWindCur, qc)) {
      continue;
    }
    write_bytes(reinterpret_cast<void*>(oq + kWindCur), cur, 12);
    // the previous-frame vector sits one frame behind the current one; leave it alone if this object differs
    if (read_vec3(oq + kWindPrev, qp) && std::fabs(qp[0] - qc[0]) < 5.f && std::fabs(qp[1] - qc[1]) < 5.f &&
        std::fabs(qp[2] - qc[2]) < 5.f) {
      write_bytes(reinterpret_cast<void*>(oq + kWindPrev), prev, 12);
    }
  }
  g_wind_copies.fetch_add(1, std::memory_order_relaxed);
}

void log_wind_sync() {
  float m[3] = {}, s[3] = {};
  void* pm = nullptr;
  void* ps = nullptr;
  const bool ok = g_created_main != nullptr && g_created != nullptr &&
                  read_ptr(static_cast<unsigned char*>(g_created_main) + kWindObj, &pm) &&
                  read_ptr(static_cast<unsigned char*>(g_created) + kWindObj, &ps) &&
                  read_vec3(reinterpret_cast<std::uintptr_t>(pm) + kWindCur, m) &&
                  read_vec3(reinterpret_cast<std::uintptr_t>(ps) + kWindCur, s);
  log_line("windsync: %s, %d slots, %llu frames; main (%.3f, %.3f, %.3f) second (%.3f, %.3f, %.3f)%s",
           g_vegsync.load() ? "on" : "off", g_wind_slot_count.load(), static_cast<unsigned long long>(g_wind_copies.load()),
           m[0], m[1], m[2], s[0], s[1], s[2], ok ? "" : " (unread)");
}

// UI pass probe (2026-10-08, read-only). [AUTHOR] the UI reads double in stereo with per-eye centering on (the eye
// images point apart, tan -0.255 / +0.255 on Quest 3), and turning centering off costs image quality. Candidate fix:
// move and scale the UI per eye inside each image. First question: are the UI's D3D12 commands recorded on this thread
// while the pass runs? The pass function is hooked and the thread tagged (1 = main view, 2 = second view);
// camera.cpp's command-list hooks count what the tagged thread records. "uiprobe" lines every 5 s.
thread_local int t_ui_eye = 0;
thread_local int t_ui_kind = 0;  // 0 = the UI pass, 1 = a neighbour pass (0x55EE10 / 0x55F150)
std::atomic<std::uint64_t> g_ui_calls[3], g_ui_ticks[3], g_ui_nested{0};
std::atomic<int> g_ui_extra_mask{3};
std::atomic<std::uint64_t> g_ui_extra_calls[2][3];  // [pass][eye]

std::uintptr_t UiPass_hook(void* owner, void* view, void* a3, void* a4) {
  const int eye = view == nullptr ? 0 : view == g_views[0].view ? 1 : view == g_created ? 2 : 0;
  const int outer = t_ui_eye;
  if (outer != 0) {
    g_ui_nested.fetch_add(1, std::memory_order_relaxed);
  }
  t_ui_eye = eye;
  if (eye != 0) {
    camera::ui_pass_begin();
    static std::atomic<ULONGLONG> last[3];
    const auto now=GetTickCount64();
    auto prev=last[eye].load();
    if (now-prev>=30000 && last[eye].compare_exchange_strong(prev,now)) {
      float cb[0x170/4]={};
      // STATIC: render-side UI setup copies owner+10 as its 170-byte view
      // constants block. Read only; rows remain unclassified until inspected.
      if (owner != nullptr && read_bytes(static_cast<unsigned char*>(owner)+0x10,cb,sizeof(cb))) {
        log_line("ui render constants eye %d owner=%p (read-only owner+10, row offsets below)",eye,owner);
        for (unsigned i=0;i<sizeof(cb)/sizeof(float);i+=4)
          log_line(" ui cb +%03X: %.6g %.6g %.6g %.6g",i*4,cb[i],cb[i+1],cb[i+2],cb[i+3]);
      }
    }
  }
  LARGE_INTEGER t0, t1;
  QueryPerformanceCounter(&t0);
  const std::uintptr_t r = g_ui_pass(owner, view, a3, a4);
  QueryPerformanceCounter(&t1);
  if (eye != 0) {
    camera::ui_pass_end();  // restore the viewport / scissor that UI placement changed
  }
  t_ui_eye = outer;
  if (eye != 0) {
    g_ui_calls[eye].fetch_add(1, std::memory_order_relaxed);
    g_ui_ticks[eye].fetch_add(static_cast<std::uint64_t>(t1.QuadPart - t0.QuadPart), std::memory_order_relaxed);
  }
  return r;
}

void log_rect_line(const char* text) { log_line("%s", text); }

std::uintptr_t ui_extra(int which, RenderFn original, void* owner, void* view, void* a3, void* a4) {
  const int eye = view == nullptr ? 0 : view == g_views[0].view ? 1 : view == g_created ? 2 : 0;
  g_ui_extra_calls[which][eye].fetch_add(1, std::memory_order_relaxed);
  if (eye == 0 || (g_ui_extra_mask.load(std::memory_order_relaxed) & (1 << which)) == 0 || t_ui_eye != 0) {
    return original(owner, view, a3, a4);
  }
  t_ui_eye = eye;
  t_ui_kind = 1;
  camera::ui_pass_begin();
  const std::uintptr_t r = original(owner, view, a3, a4);
  camera::ui_pass_end();  // restore the viewport / scissor the placement changed
  t_ui_kind = 0;
  t_ui_eye = 0;
  return r;
}

std::uintptr_t UiExtra7_hook(void* owner, void* view, void* a3, void* a4) { return ui_extra(0, g_ui_extra7, owner, view, a3, a4); }
std::uintptr_t UiExtra9_hook(void* owner, void* view, void* a3, void* a4) { return ui_extra(1, g_ui_extra9, owner, view, a3, a4); }

void log_ui_probe() {
  static ULONGLONG last = 0;
  static ULONGLONG last_rects = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last_rects >= 20000) {
    last_rects = now;
    camera::rect_probe_report(&log_rect_line);
  }
  if (last == 0) {
    last = now;
    return;
  }
  if (now - last < 5000 || g_ui_pass == nullptr) {
    return;
  }
  const double secs = static_cast<double>(now - last) * 1e-3;
  last = now;
  LARGE_INTEGER freq;
  QueryPerformanceFrequency(&freq);
  log_line("uiextra: mask=%d 0x55EE10 main=%llu second=%llu other=%llu | 0x55F150 main=%llu second=%llu other=%llu (calls since last)",
           g_ui_extra_mask.load(), static_cast<unsigned long long>(g_ui_extra_calls[0][1].exchange(0)),
           static_cast<unsigned long long>(g_ui_extra_calls[0][2].exchange(0)), static_cast<unsigned long long>(g_ui_extra_calls[0][0].exchange(0)),
           static_cast<unsigned long long>(g_ui_extra_calls[1][1].exchange(0)), static_cast<unsigned long long>(g_ui_extra_calls[1][2].exchange(0)),
           static_cast<unsigned long long>(g_ui_extra_calls[1][0].exchange(0)));
  for (int eye = 1; eye <= 2; ++eye) {
    const std::uint64_t calls = g_ui_calls[eye].exchange(0);
    const std::uint64_t ticks = g_ui_ticks[eye].exchange(0);
    char text[400] = {};
    camera::ui_probe_report(eye, text, sizeof(text));
    if (calls == 0 && eye == 2) {
      continue;  // no second view (menus, start-up)
    }
    float ox = 0.f, oy = 0.f, sx = 1.f, sy = 1.f;
    std::uint32_t ew = 0, eh = 0;
    const bool eyes = eyesize_for(eye == 2, &ew, &eh) && g_stereo_running.load(std::memory_order_relaxed);
    const bool mapped = native_stereo::ui_map(eye, static_cast<float>(eyes ? ew : g_render_w.load()),
                                              static_cast<float>(eyes ? eh : g_render_h.load()), &ox, &oy, &sx, &sy);
    log_line("uiprobe %s: passes=%.1f/s %.3f ms/pass nested=%llu placed=%llu map=%s%s(%.0f,%.0f x%.2f) | %s", eye == 1 ? "main" : "second",
             static_cast<double>(calls) / secs,
             calls ? static_cast<double>(ticks) * 1000.0 / static_cast<double>(freq.QuadPart) / static_cast<double>(calls) : 0.0,
             static_cast<unsigned long long>(g_ui_nested.load()), static_cast<unsigned long long>(camera::ui_placed(eye)),
             mapped ? "on" : "off", native_stereo::ui_layer_active() ? " layer" : "", ox, oy, sx, text);
  }
}

std::uintptr_t CamBuild_hook(void* view, void* a2, void* a3, void* a4) {
  g_cam_builds.fetch_add(1, std::memory_order_relaxed);
  const int index = view == nullptr ? -1 : find_view(view);
  if (index == 0 || (view != nullptr && view == g_created)) {
    std::uint32_t game = 0, other = 0;
    view_slots(view, &game, &other);
    trace('C', index, game, other);
  }
  if (index >= 0 || g_world_view != nullptr) {
    world_step(view, index);
  }
  if (view != nullptr && view == g_created_main && g_created != nullptr && !g_head_prestepped && g_helpers_ok &&
      g_head.load(std::memory_order_relaxed) && g_want_create.load(std::memory_order_relaxed) &&
      !g_rescued.load(std::memory_order_relaxed) && g_created_main_cam != nullptr && g_priv_left != nullptr) {
    void* cur = nullptr;
    if (read_ptr(static_cast<unsigned char*>(view) + kCameraOffset, &cur) && cur == g_priv_left && head_step(g_created_main_cam)) {
      g_head_prestepped = true;  // this frame's pose is in both private cameras before either view copies its camera
    }
  }
  const bool wind_view = view != nullptr && view == g_created && g_created_main != nullptr &&
                         g_stereo_running.load(std::memory_order_relaxed);
  if (wind_view) {
    wind_sync();  // before: the second view's camera build copies from these objects
  }
  const std::uintptr_t r = g_cam_build(view, a2, a3, a4);
  if (wind_view) {
    wind_sync();  // after: in case the build advanced the second view's own accumulator
  }
  if (view != nullptr && (view == g_created_main || view == g_created)) {
    std::uint32_t slot = 0;
    if (read_u32(static_cast<unsigned char*>(view) + kFrameIdxGame, &slot) && slot < 8) {
      std::lock_guard<std::mutex> lock(g_applied_mutex);
      g_slot_pose[view == g_created ? 1 : 0][slot] = g_applied_count;  // index + 1 of the newest applied pose
    }
  }
  if (view != nullptr && view == g_created && g_created_main != nullptr && g_stereo_running.load(std::memory_order_relaxed)) {
    if (kClockDiscovery) {
      clock_discover();  // diagnostic, off since the wind accumulator was found (wind_sync)
    }
    clock_sync();
    jitter_sync();
    wind_step();
  }
  if (kMainOffscreen && view != nullptr && view == g_created_main && g_created != nullptr) {  // eye mode: main offscreen
    static bool was_on = false;
    const bool on = eye_mode_on();
    void* ring = nullptr;
    std::uint32_t idx = 0;
    unsigned char* v = static_cast<unsigned char*>(view);
    if ((on || was_on) && read_ptr(v + kFrameRing, &ring) && ring != nullptr && read_u32(v + kFrameIdxGame, &idx) && idx < 8) {
      unsigned char* frame = static_cast<unsigned char*>(ring) + static_cast<std::uintptr_t>(idx) * kFrameStride;
      __try_write_u8(frame + kOffscreenFlag, on ? 1 : 0);
    }
    if (was_on && !on) {
      drop_main_eye();
    }
    was_on = on;
  }
  if (view != nullptr && view == g_created) {  // written every frame: 1 while wanted, 0 otherwise
    const bool off_on = g_want_create.load(std::memory_order_relaxed) && g_want_offscreen.load(std::memory_order_relaxed) &&
                        !g_rescued.load(std::memory_order_relaxed);
    void* ring = nullptr;
    std::uint32_t idx = 0;
    unsigned char* v = static_cast<unsigned char*>(view);
    if (read_ptr(v + kFrameRing, &ring) && ring != nullptr && read_u32(v + kFrameIdxGame, &idx) && idx < 8) {
      unsigned char* frame = static_cast<unsigned char*>(ring) + static_cast<std::uintptr_t>(idx) * kFrameStride;
      __try_write_u8(frame + kOffscreenFlag, off_on ? 1 : 0);
      if (off_on) {
        g_offscreen_sets.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
  const int target = g_second_view.load(std::memory_order_relaxed);
  const bool created = view != nullptr && view == g_created && g_want_create.load(std::memory_order_relaxed) &&
                       !g_priv_mode.load(std::memory_order_relaxed);
  if (g_helpers_ok && index >= 0 && (index == target || created) && !g_rescued.load(std::memory_order_relaxed)) {
    const float sep = static_cast<float>((created ? g_create_sep_um : g_second_sep_um).load(std::memory_order_relaxed)) * 1e-6f;
    (apply_shift(view, sep) ? g_second_applied : g_second_failed).fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

std::uintptr_t Render_hook(void* view, void* a2, void* a3, void* a4) {
  const int index = view == nullptr ? -1 : view_index(view);
  if (index == 0) pose_phase_probe::observe(pose_phase_probe::Phase::RenderPre, view);
  if (kMainOffscreen && index == 0 && view == g_created_main && eye_mode_on()) {
    for (int s = 0; s < 2; ++s) {
      unsigned char* flag = nullptr;
      std::uint8_t b = 0;
      if (frame_byte(view, s == 0 ? kFrameIdxGame : kFrameIdxGame + 4, kOffscreenFlag, &flag) && read_u8(flag, &b)) {
        g_main_off_seen[s][b != 0 ? 1 : 0].fetch_add(1, std::memory_order_relaxed);
        // Run 18:20-18:22: the game slot held 1 every time, the render slot was clear in ~70% of renders and the
        // main view kept drawing into the backbuffer. Set both before the render reads its frame.
        if (b == 0) {
          __try_write_u8(flag, 1);
        }
      }
    }
  }
  if (index >= 0) {
    ViewInfo& info = g_views[index];
    info.renders.fetch_add(1, std::memory_order_relaxed);
    unsigned char* base = static_cast<unsigned char*>(view);
    std::uint32_t guard = 1;
    read_u32(base + kGuardOffset, &guard);
    const int want = g_want_mode.load(std::memory_order_relaxed);
    if (want != 0 && !g_rescued.load(std::memory_order_relaxed) && index == g_want_view.load(std::memory_order_relaxed) && guard == 0) {
      std::lock_guard<std::mutex> lock(g_mutex);
      if (!info.written) {
        read_u32(base + kModeOffset, &info.saved_mode);
        info.written = true;
        log_line("view #%d: writing mode %d (saved %u)", index, want, info.saved_mode);
        start_heartbeat();
      }
      if (!write_u32(base + kModeOffset, static_cast<std::uint32_t>(want))) {
        g_faults.fetch_add(1, std::memory_order_relaxed);
      }
    }
    std::uint32_t mode = 0;
    if (guard == 0 && read_u32(base + kModeOffset, &mode) && mode != 0) {
      info.stereo_entries.fetch_add(1, std::memory_order_relaxed);
    }
  }
  std::uint32_t pass = 0;
  if (view != nullptr) {
    read_u32(static_cast<unsigned char*>(view) + kGuardOffset, &pass);
  }
  const int slot = pass > 2 ? 2 : static_cast<int>(pass);
  g_render_in[slot].fetch_add(1, std::memory_order_relaxed);
  const int saved_view = t_view_index;
  t_view_index = index;
  if (index >= 0) {
    g_views[index].render_tid.store(GetCurrentThreadId(), std::memory_order_relaxed);
  }
  const bool traced = index == 0 || (view != nullptr && view == g_created);
  if (traced) {
    std::uint32_t game = 0, other = 0;
    view_slots(view, &game, &other);
    trace('R', index, game, other);
  }
  const int kind = index == 0 ? 0 : (view != nullptr && view == g_created) ? 1 : 2;
  g_rendering[kind].fetch_add(1, std::memory_order_relaxed);
  LARGE_INTEGER t0, t1;
  QueryPerformanceCounter(&t0);
  const std::uintptr_t r = g_render(view, a2, a3, a4);
  QueryPerformanceCounter(&t1);
  g_rendering[kind].fetch_sub(1, std::memory_order_relaxed);
  if (traced) {
    std::uint32_t game = 0, other = 0;
    view_slots(view, &game, &other);
    trace('E', index, game, other);
  }
  if (index >= 0) {
    g_render_ticks[index].fetch_add(static_cast<std::uint64_t>(t1.QuadPart - t0.QuadPart), std::memory_order_relaxed);
    g_render_calls[index].fetch_add(1, std::memory_order_relaxed);
  }
  t_view_index = saved_view;
  g_render_out[slot].fetch_add(1, std::memory_order_relaxed);
  return r;
}

std::uintptr_t Stereo_hook(void* view, void* a2, void* a3, void* a4) {
  g_stereo_in.fetch_add(1, std::memory_order_relaxed);
  const std::uintptr_t r = g_stereo(view, a2, a3, a4);
  g_stereo_out.fetch_add(1, std::memory_order_relaxed);
  // Experiment ("kick", user-approved): the stereo frame's own gate-submitter task never runs, so its phase gates
  // (eye stepper, capture, composite) are never handed to the workers. Run the engine's submitter once for this
  // view: the same function Gfx_View::Render's submitter task runs in every mono frame.
  if (g_want_kick.load(std::memory_order_relaxed) && g_submit != nullptr) {
    g_kicks.fetch_add(1, std::memory_order_relaxed);
    g_submit(view, nullptr, nullptr, nullptr);
  }
  return r;
}

std::uintptr_t Wait_hook(void* view, void* a2, void* a3, void* a4) {
  g_wait_in.fetch_add(1, std::memory_order_relaxed);
  const DWORD tid = GetCurrentThreadId();
  int slot = -1;
  for (int i = 0; i < kPendingMax; ++i) {
    DWORD expected = 0;
    if (g_pending[i].tid.compare_exchange_strong(expected, tid)) {
      g_pending[i].caller = _ReturnAddress();
      g_pending[i].view = view;
      g_pending[i].flag = a2;
      g_pending[i].since = GetTickCount64();
      slot = i;
      break;
    }
  }
  LARGE_INTEGER t0, t1;
  QueryPerformanceCounter(&t0);
  const std::uintptr_t r = g_wait(view, a2, a3, a4);
  QueryPerformanceCounter(&t1);
  {
    const auto dt = static_cast<std::uint64_t>(t1.QuadPart - t0.QuadPart);
    const int index = view != nullptr ? find_view(view) : -1;
    if (index >= 0) {
      g_wait_ticks[index].fetch_add(dt, std::memory_order_relaxed);
      g_wait_calls[index].fetch_add(1, std::memory_order_relaxed);
    } else {
      g_wait_other_ticks.fetch_add(dt, std::memory_order_relaxed);
    }
  }
  if (slot >= 0) {
    g_pending[slot].tid.store(0);
  }
  g_wait_out.fetch_add(1, std::memory_order_relaxed);
  return r;
}

std::uintptr_t Composite_hook(void* view, void* a2, void* a3, void* a4) {
  g_composite_in.fetch_add(1, std::memory_order_relaxed);
  const std::uintptr_t r = g_composite(view, a2, a3, a4);
  g_composite_out.fetch_add(1, std::memory_order_relaxed);
  return r;
}

std::uintptr_t AddTask_hook(void* ctx, void* out, void* kind_arg, void* job) {
  const int kind = static_cast<int>(reinterpret_cast<std::uintptr_t>(kind_arg) & 0xFF);
  const int k = kind > 3 ? 3 : kind;
  std::uint32_t lo = 0, hi = 0;
  bool existing = false;
  if (ctx != nullptr) {
    unsigned char* slot = static_cast<unsigned char*>(ctx) + 0x5478 + static_cast<std::uintptr_t>(kind) * 8;
    existing = read_u32(slot, &lo) && read_u32(slot + 4, &hi) && (lo != 0 || hi != 0);
  }
  (existing ? g_add_reuse[k] : g_add_new[k]).fetch_add(1, std::memory_order_relaxed);
  if (ctx != nullptr) {
    const int view = find_view(static_cast<unsigned char*>(ctx) - 0x49F0);
    if (view >= 0) {
      g_task_calls[view][k].fetch_add(1, std::memory_order_relaxed);
      note_job(view, k, job);
    }
  }
  return g_add_task(ctx, out, kind_arg, job);
}

std::uintptr_t Gate_hook(void* a1, void* graph, void* kind_arg, void* a4) {
  const int kind = static_cast<int>(reinterpret_cast<std::uintptr_t>(kind_arg) & 0xFF);
  const int k = kind > 3 ? 3 : kind;
  g_gate_in[k].fetch_add(1, std::memory_order_relaxed);
  const int view = view_containing(graph);
  LARGE_INTEGER t0, t1;
  QueryPerformanceCounter(&t0);
  const std::uintptr_t r = g_gate(a1, graph, kind_arg, a4);
  QueryPerformanceCounter(&t1);
  g_gate_out[k].fetch_add(1, std::memory_order_relaxed);
  g_gate_kind_ticks[k].fetch_add(static_cast<std::uint64_t>(t1.QuadPart - t0.QuadPart), std::memory_order_relaxed);
  g_gate_kind_calls[k].fetch_add(1, std::memory_order_relaxed);
  if (view >= 0) {
    g_gate_ticks[view][k].fetch_add(static_cast<std::uint64_t>(t1.QuadPart - t0.QuadPart), std::memory_order_relaxed);
    g_gate_calls[view][k].fetch_add(1, std::memory_order_relaxed);
    std::int64_t expected = 0;
    g_gate_offset[view].compare_exchange_strong(
        expected, static_cast<std::int64_t>(static_cast<unsigned char*>(graph) - static_cast<unsigned char*>(g_views[view].view)));
  } else {
    g_gate_unmatched.fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

std::uintptr_t Submit_hook(void* a1, void* a2, void* a3, void* a4) {
  g_submit_in.fetch_add(1, std::memory_order_relaxed);
  const std::uintptr_t r = g_submit(a1, a2, a3, a4);
  g_submit_out.fetch_add(1, std::memory_order_relaxed);
  return r;
}

std::uintptr_t Capture_hook(void* view, void* a2, void* a3, void* a4) {
  g_captures.fetch_add(1, std::memory_order_relaxed);
  const std::uintptr_t r = g_capture(view, a2, a3, a4);
  g_capture_out.fetch_add(1, std::memory_order_relaxed);
  return r;
}

void restore_all() {
  std::lock_guard<std::mutex> lock(g_mutex);
  const int count = g_view_count.load();
  for (int i = 0; i < count; ++i) {
    ViewInfo& info = g_views[i];
    if (info.written) {
      write_u32(static_cast<unsigned char*>(info.view) + kModeOffset, info.saved_mode);
      info.written = false;
      log_line("view #%d: restored mode %u", i, info.saved_mode);
    }
  }
}

void restore_all();

// Runs on its own thread so it keeps logging when the game's Present thread stops.
// Every 5 s: per view, Render and phase-gate CPU time per rendered frame, and queued tasks per frame; every 30 s
// the job census (jobs per frame for the main view #0 and the second view, by identity key).
// Text at module pointers inside a sample job: Snowdrop jobs may carry a profiler name. Printable ASCII only.
void describe_job(const std::uintptr_t* words, char* out, size_t size) {
  out[0] = '\0';
  const auto base = reinterpret_cast<std::uintptr_t>(g_module_base);
  int n = 0;
  for (int k = 0; k < 8; ++k) {
    if (words[k] < base || words[k] >= base + g_image_size) {
      continue;
    }
    char text[49] = {};
    if (!read_bytes(reinterpret_cast<const void*>(words[k]), text, 48)) {
      continue;
    }
    int len = 0;
    while (len < 48 && text[len] >= 0x20 && text[len] < 0x7F) {
      ++len;
    }
    if (len >= 4 && (len == 48 || text[len] == '\0')) {
      text[len] = '\0';
      n += snprintf(out + n, size - n, " +0x%X=\"%s\"", k * 8, text);
    } else {
      n += snprintf(out + n, size - n, " +0x%X=afop+0x%llX", k * 8, static_cast<unsigned long long>(words[k] - base));
    }
    if (n <= 0 || n >= static_cast<int>(size)) {
      break;
    }
  }
}

void report_census() {
  static std::uint64_t last_render_ticks[kMaxViews], last_render_calls[kMaxViews], last_wait_ticks[kMaxViews], last_wait_calls[kMaxViews];
  static std::uint64_t last_tasks[kMaxViews][4], last_kind_ticks[4], last_kind_calls[4], last_presents = 0, last_wait_other = 0;
  static int reports = 0;
  if (g_qpf.QuadPart == 0) {
    QueryPerformanceFrequency(&g_qpf);
  }
  const double ms = 1000.0 / static_cast<double>(g_qpf.QuadPart);
  const std::uint64_t presents = g_presents.load();
  const std::uint64_t frames_all = presents - last_presents;
  last_presents = presents;
  if (frames_all == 0) {
    return;
  }
  {  // phase gates are frame-level (their graph argument is not inside a Gfx_View): worker CPU per presented frame
    char line[256];
    int n = snprintf(line, sizeof(line), "cost frame: %llu presents | gate worker ms/frame (calls/frame)",
                     static_cast<unsigned long long>(frames_all));
    for (int k = 0; k < 4 && n > 0 && n < static_cast<int>(sizeof(line)); ++k) {
      const std::uint64_t t = g_gate_kind_ticks[k].load(), c = g_gate_kind_calls[k].load();
      n += snprintf(line + n, sizeof(line) - n, " k%d %.2f (%.1f)", k, static_cast<double>(t - last_kind_ticks[k]) * ms / frames_all,
                    static_cast<double>(c - last_kind_calls[k]) / frames_all);
      last_kind_ticks[k] = t;
      last_kind_calls[k] = c;
    }
    const std::uint64_t wo = g_wait_other_ticks.load();
    log_line("%s | task wait outside known views %.2f ms/frame | pose identity used %llu missing %llu eyes-a-frame-apart %llu", line,
             static_cast<double>(wo - last_wait_other) * ms / frames_all, static_cast<unsigned long long>(g_identity_used.load()),
             static_cast<unsigned long long>(g_identity_missing.load()), static_cast<unsigned long long>(g_identity_split.load()));
    last_wait_other = wo;
  }
  const int count = g_view_count.load();
  for (int v = 0; v < count && v < kMaxViews; ++v) {
    const std::uint64_t rt = g_render_ticks[v].load(), rc = g_render_calls[v].load();
    const std::uint64_t wt = g_wait_ticks[v].load(), wc = g_wait_calls[v].load();
    const std::uint64_t frames = rc - last_render_calls[v];
    if (frames >= 20) {  // skip the auxiliary views that render a few times per second
      std::uint64_t tasks[4];
      for (int k = 0; k < 4; ++k) {
        tasks[k] = g_task_calls[v][k].load();
      }
      log_line("cost view #%d: %llu renders | Render %.2f ms/frame | task wait %.2f ms/frame (%.1f waits) | tasks/frame k1 %.1f k2 %.1f k3 %.1f",
               v, static_cast<unsigned long long>(frames), static_cast<double>(rt - last_render_ticks[v]) * ms / frames,
               static_cast<double>(wt - last_wait_ticks[v]) * ms / frames, static_cast<double>(wc - last_wait_calls[v]) / frames,
               static_cast<double>(tasks[1] - last_tasks[v][1]) / frames, static_cast<double>(tasks[2] - last_tasks[v][2]) / frames,
               static_cast<double>(tasks[3] - last_tasks[v][3]) / frames);
    }
    for (int k = 0; k < 4; ++k) {
      last_tasks[v][k] = g_task_calls[v][k].load();
    }
    last_render_ticks[v] = rt;
    last_render_calls[v] = rc;
    last_wait_ticks[v] = wt;
    last_wait_calls[v] = wc;
  }
  if (++reports % 6 != 0) {
    return;
  }
  {  // the last 40 pipeline events, oldest first; times in ms relative to the first
    const std::uint32_t end = g_trace_next.load();
    const std::uint32_t n = end < 40 ? end : 40;
    LONGLONG t0 = 0;
    char line[1600];
    int len = snprintf(line, sizeof(line), "trace (type view tid slotGame/slot+4 @ms):");
    for (std::uint32_t k = end - n; k != end && len > 0 && len < static_cast<int>(sizeof(line)) - 48; ++k) {
      const TraceEvent e = g_trace[k % kTraceSize];
      if (t0 == 0) {
        t0 = e.qpc;
      }
      len += snprintf(line + len, sizeof(line) - len, " %c%d/%u %u/%u@%.1f", e.type, e.view, e.tid, e.a, e.b,
                      static_cast<double>(e.qpc - t0) * ms);
    }
    log_line("%s", line);
  }
  // Job census: the main view and the other view with the most renders (the native second view when it exists).
  int second = -1;
  std::uint64_t best = 0;
  for (int v = 1; v < count && v < kMaxViews; ++v) {
    const std::uint64_t r = g_render_calls[v].load();
    if (r > best) {
      best = r;
      second = v;
    }
  }
  std::lock_guard<std::mutex> lock(g_job_mutex);
  const std::uint64_t r0 = g_render_calls[0].load();
  log_line("jobs (per render since start): key = first afop pointer in the job [offset], kind; view #0 %llu renders, view #%d %llu renders",
           static_cast<unsigned long long>(r0), second, static_cast<unsigned long long>(best));
  for (int i = 0; i < g_job_key_count; ++i) {
    const JobKey& key = g_job_keys[i];
    const double a = r0 != 0 ? static_cast<double>(key.count[0]) / r0 : 0.0;
    const double b = second >= 0 && best != 0 ? static_cast<double>(key.count[second]) / best : 0.0;
    if (a >= 0.05 || b >= 0.05) {
      char text[400];
      describe_job(key.sample, text, sizeof(text));
      log_line("  job afop+0x%X [+0x%X] k%u: #0 %.2f  #%d %.2f |%s", key.rva, key.at, key.kind, a, second, b, text);
    }
  }
}

DWORD WINAPI heartbeat(void*) {
  std::uint64_t last_presents = g_presents.load();
  ULONGLONG last_change = GetTickCount64();
  for (;;) {
    Sleep(250);
    {
      static int tick = 0;
      if (++tick % 20 == 0) {
        report_census();
      }
    }
    const std::uint64_t p = g_presents.load();
    const ULONGLONG now = GetTickCount64();
    if (p != last_presents) {
      last_presents = p;
      last_change = now;
    }
    const bool stalled = now - last_change > 2000;
    log_line("hb presents=%llu%s stereo in/out=%llu/%llu render[g0] %llu/%llu [g1] %llu/%llu [g2] %llu/%llu capture %llu/%llu "
             "wait %llu/%llu composite %llu/%llu stepper %llu kicks %llu",
             static_cast<unsigned long long>(p), stalled ? " STALLED" : "",
             static_cast<unsigned long long>(g_stereo_in.load()), static_cast<unsigned long long>(g_stereo_out.load()),
             static_cast<unsigned long long>(g_render_in[0].load()), static_cast<unsigned long long>(g_render_out[0].load()),
             static_cast<unsigned long long>(g_render_in[1].load()), static_cast<unsigned long long>(g_render_out[1].load()),
             static_cast<unsigned long long>(g_render_in[2].load()), static_cast<unsigned long long>(g_render_out[2].load()),
             static_cast<unsigned long long>(g_captures.load()), static_cast<unsigned long long>(g_capture_out.load()),
             static_cast<unsigned long long>(g_wait_in.load()), static_cast<unsigned long long>(g_wait_out.load()),
             static_cast<unsigned long long>(g_composite_in.load()), static_cast<unsigned long long>(g_composite_out.load()),
             eye_probe::stepper_calls(), static_cast<unsigned long long>(g_kicks.load()));
    {  // every heartbeat tick (the heartbeat only runs once a mode has been written)
      log_line("  sched add new/reuse k0 %llu/%llu k1 %llu/%llu k2 %llu/%llu k3 %llu/%llu | gate in/out k0 %llu/%llu k1 %llu/%llu k2 %llu/%llu k3 %llu/%llu | submit %llu/%llu",
               static_cast<unsigned long long>(g_add_new[0].load()), static_cast<unsigned long long>(g_add_reuse[0].load()),
               static_cast<unsigned long long>(g_add_new[1].load()), static_cast<unsigned long long>(g_add_reuse[1].load()),
               static_cast<unsigned long long>(g_add_new[2].load()), static_cast<unsigned long long>(g_add_reuse[2].load()),
               static_cast<unsigned long long>(g_add_new[3].load()), static_cast<unsigned long long>(g_add_reuse[3].load()),
               static_cast<unsigned long long>(g_gate_in[0].load()), static_cast<unsigned long long>(g_gate_out[0].load()),
               static_cast<unsigned long long>(g_gate_in[1].load()), static_cast<unsigned long long>(g_gate_out[1].load()),
               static_cast<unsigned long long>(g_gate_in[2].load()), static_cast<unsigned long long>(g_gate_out[2].load()),
               static_cast<unsigned long long>(g_gate_in[3].load()), static_cast<unsigned long long>(g_gate_out[3].load()),
               static_cast<unsigned long long>(g_submit_in.load()), static_cast<unsigned long long>(g_submit_out.load()));
    }
    if (stalled) {
      const ULONGLONG t = GetTickCount64();
      for (int i = 0; i < kPendingMax; ++i) {
        const DWORD tid = g_pending[i].tid.load();
        if (tid != 0 && t - g_pending[i].since > 1000) {
          const auto caller = reinterpret_cast<unsigned char*>(g_pending[i].caller);
          log_line("  pending wait tid=%lu caller=afop+0x%llX view=%p flag=%llu age=%llums", tid,
                   static_cast<unsigned long long>(caller - g_module_base), g_pending[i].view,
                   static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(g_pending[i].flag) & 0xFF),
                   static_cast<unsigned long long>(t - g_pending[i].since));
        }
      }
    }
    if (stalled && !g_rescued.exchange(true)) {
      log_line("RESCUE: no Present for 2 s with a mode written; restoring the mode field and stopping writes");
      g_want_mode.store(0);
      restore_all();
      if (g_world_view != nullptr) {  // game thread is not making progress; put the world pointer back from here
        write_ptr(static_cast<unsigned char*>(g_world_view) + kWorldOffset, g_world_saved);
        log_line("RESCUE: restored world of %p to %p", g_world_view, g_world_saved);
        g_world_view = nullptr;
      }
    }
    if (stalled && now - last_change > 30000) {
      log_line("heartbeat: still stalled after 30 s; stopping heartbeat");
      return 0;
    }
  }
}

// ---- named pass census ---------------------------------------------------------------------------------
// 0x1405537D0 takes a pass name in rdx ("offscreen particles", "offscreen particles composited", ... code dump round 3:
// lea rdx,[name]; call 0x1405537D0). Counting calls per name per Present, separately while stereo runs and while it is
// paused, shows which named passes run once per frame (main view only) and which run once per view. A pass whose
// rate does not double when stereo runs is one the second view never gets: the left-only effects are in that list.
constexpr std::uintptr_t kPassNameRva = 0x5537D0;
using PassNameFn = std::uintptr_t (*)(void*, void*, void*, void*);
PassNameFn g_pass_name = nullptr;
constexpr int kPassSlots = 1024;
std::atomic<const void*> g_pass_key[kPassSlots];
std::atomic<std::uint32_t> g_pass_count[kPassSlots][2];  // [running, paused]
std::atomic<std::uint64_t> g_pass_presents[2];
std::atomic<std::uint64_t> g_pass_overflow{0};
// Per-view attribution while stereo runs: [slot][0 main, 1 second view, 2 other view, 3 no Render on this thread].
std::atomic<std::uint32_t> g_pass_view[kPassSlots][4];
// The first argument (render context) of the first 4 distinct contexts seen per name, with counts.
std::atomic<const void*> g_pass_ctx[kPassSlots][4];
std::atomic<std::uint32_t> g_pass_ctx_n[kPassSlots][4];

// A pass name: at least 4 printable characters. The stub is probably shared by other empty functions (identical
// code folding), so rdx is not always a string.
bool is_name(const void* p) {
  unsigned char c[4] = {};
  if (!read_bytes(p, c, sizeof(c))) {
    return false;
  }
  for (unsigned char ch : c) {
    if (ch < 0x20 || ch > 0x7E) {
      return false;
    }
  }
  return true;
}

void note_pass_view(std::size_t h, const void* ctx) {
  const int vi = t_view_index;
  int bucket = 3;
  if (vi >= 0) {
    void* v = g_views[vi].view;
    bucket = v == g_created_main ? 0 : (v != nullptr && v == g_created) ? 1 : 2;
  }
  g_pass_view[h][bucket].fetch_add(1, std::memory_order_relaxed);
  for (int i = 0; i < 4; ++i) {
    const void* c = g_pass_ctx[h][i].load(std::memory_order_relaxed);
    if (c == ctx) {
      g_pass_ctx_n[h][i].fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (c == nullptr) {
      const void* expected = nullptr;
      if (g_pass_ctx[h][i].compare_exchange_strong(expected, ctx) || expected == ctx) {
        g_pass_ctx_n[h][i].fetch_add(1, std::memory_order_relaxed);
        return;
      }
    }
  }
}

std::uintptr_t PassName_hook(void* a1, void* name, void* a3, void* a4) {
  const auto a = reinterpret_cast<std::uintptr_t>(name);
  const auto base = reinterpret_cast<std::uintptr_t>(g_module_base);
  if (a >= base && a + 4 < base + g_image_size && (a & 7) == 0 && is_name(name)) {  // names: string literals in the module
    const int state = g_stereo_running.load(std::memory_order_relaxed) ? 0 : 1;
    std::size_t h = (a >> 3) * 0x9E3779B97F4A7C15ull >> 54;  // 10 bits
    for (int probe = 0; probe < 32; ++probe, h = (h + 1) & (kPassSlots - 1)) {
      const void* k = g_pass_key[h].load(std::memory_order_relaxed);
      if (k == name) {
        g_pass_count[h][state].fetch_add(1, std::memory_order_relaxed);
        if (state == 0) {
          note_pass_view(h, a1);
        }
        break;
      }
      if (k == nullptr) {
        const void* expected = nullptr;
        if (g_pass_key[h].compare_exchange_strong(expected, name) || expected == name) {
          g_pass_count[h][state].fetch_add(1, std::memory_order_relaxed);
          if (state == 0) {
            note_pass_view(h, a1);
          }
          break;
        }
      }
      if (probe == 31) {
        g_pass_overflow.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
  return g_pass_name(a1, name, a3, a4);
}

// Present thread, every 15 s: names with their calls per Present while running and while paused.
void log_pass_census() {
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (g_pass_name == nullptr || now - last < 15000) {
    return;
  }
  last = now;
  const double pr = static_cast<double>(g_pass_presents[0].load()), pp = static_cast<double>(g_pass_presents[1].load());
  if (pr < 100) {
    return;  // wait for some stereo frames
  }
  log_line("passes: calls per Present, stereo running (%0.f presents) vs paused (%0.f); \"main only\" = not doubled when running",
           pr, pp);
  for (int i = 0; i < kPassSlots; ++i) {
    const void* k = g_pass_key[i].load();
    if (k == nullptr) {
      continue;
    }
    char name[72] = {};
    read_bytes(k, name, sizeof(name) - 1);
    for (char& c : name) {
      if (c != 0 && (c < 0x20 || c > 0x7E)) c = '?';
    }
    const double r = g_pass_count[i][0].load() / pr;
    const double p = pp > 0 ? g_pass_count[i][1].load() / pp : -1.0;
    const char* tag = p > 0.05 && r < p * 1.5 ? "  <- main only?" : "";
    log_line("  pass %-40s run %6.2f  paused %6.2f%s | while running: main %u second %u other %u unknown %u | ctx %p:%u %p:%u %p:%u %p:%u",
             name, r, p, tag, g_pass_view[i][0].load(), g_pass_view[i][1].load(), g_pass_view[i][2].load(), g_pass_view[i][3].load(),
             g_pass_ctx[i][0].load(), g_pass_ctx_n[i][0].load(), g_pass_ctx[i][1].load(), g_pass_ctx_n[i][1].load(),
             g_pass_ctx[i][2].load(), g_pass_ctx_n[i][2].load(), g_pass_ctx[i][3].load(), g_pass_ctx_n[i][3].load());
  }
  if (g_pass_overflow.load() != 0) {
    log_line("  (table full: %llu calls not counted)", static_cast<unsigned long long>(g_pass_overflow.load()));
  }
}

// ---- per-view sun-shadow history slot ("shadowfix", ISSUE-04, 2026-10-21) -------------------------------------------
// [STATIC] RenderShadows' helper 0x1401A1130(a1, view) stores the view's projected-CSM target (ShadowHistory from the
// view's per-eye pool, or a transient ProjectedCsmQuarter) into ONE global slot, render-target global +0xE90 (ref-counted
// pointer, refcount int at +8) with four descriptor qwords at +0xE98..+0xEB0. ComputeResolvedShadows' worker
// 0x140186630(a1, view) reads that slot later (penumbra / screen-space sun shadow resolve). Both views share the slot, so
// if the second view's writer runs between the main view's writer and reader, one eye resolves its sun shadows from the
// other eye's history. The probe (always on, read-only) counts reads whose slot differs from what the same view wrote.
// "shadowfix" swaps the view's own entry in for its read and restores it afterwards, holding its own reference so the
// entry outlives the slot. Writer and swap/restore are serialized by g_shadow_lock (short sections only).
constexpr std::uintptr_t kShadowWriteRva = 0x1A1130;
constexpr unsigned char kShadowWriteSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x20, 0x55, 0x57, 0x41, 0x56};
constexpr std::uintptr_t kShadowReadRva = 0x186630;
constexpr unsigned char kShadowReadSig[] = {0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x56, 0x57, 0x41, 0x54};
constexpr std::uintptr_t kRefReleaseRva = 0x3261190;  // release(obj): decrements +8, frees on the last reference
constexpr unsigned char kRefReleaseSig[] = {0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x8B, 0x41, 0x08, 0x48, 0x8D, 0x51, 0x08};
constexpr std::size_t kShadowSlot = 0xE90;
using ShadowFn = void (*)(void*, void*);
using RefReleaseFn = void (*)(void*);
ShadowFn g_shadow_write = nullptr, g_shadow_read = nullptr;
RefReleaseFn g_ref_release = nullptr;
unsigned char* g_targets_global_ptr = nullptr;  // address of the global pointer (afop+0x96A1B80)
std::atomic<bool> g_shadow_fix{false};
// Performance pass 2026-10-22: the always-on research probes (shadow slot, light queue, shadow-pick recording) run only
// with "diagprobes" in the native control (or their fix toggle); otherwise their hooks pass straight through.
std::atomic<bool> g_diag_probes{false};
SRWLOCK g_shadow_lock = SRWLOCK_INIT;
struct ShadowView {
  void* view = nullptr;
  std::uint64_t entry[5] = {};  // slot pointer + descriptor, as this view's writer left it
  void* held = nullptr;         // our reference on entry[0]
  std::atomic<std::uint64_t> writes{0}, reads{0}, mismatched{0}, swapped{0}, overwritten{0}, unknown{0};
};
ShadowView g_shadow_views[2];  // 0 = main view (left eye), 1 = created view (right eye)
std::atomic<std::uint64_t> g_shadow_other{0};

bool ref_add(void* obj) {
  __try {
    InterlockedIncrement(reinterpret_cast<volatile LONG*>(static_cast<unsigned char*>(obj) + 8));
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}
void ref_release(void* obj) {
  if (obj == nullptr || g_ref_release == nullptr) return;
  __try {
    g_ref_release(obj);
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
}
int shadow_view_index(void* view) {
  if (view == nullptr || !g_stereo_running.load(std::memory_order_relaxed)) return -1;
  if (view == g_created_main) return 0;
  if (view == g_created) return 1;
  return -1;
}
unsigned char* shadow_slot() {
  void* global = nullptr;
  if (g_targets_global_ptr == nullptr || !read_ptr(g_targets_global_ptr, &global) || global == nullptr) return nullptr;
  return static_cast<unsigned char*>(global) + kShadowSlot;
}

void ShadowWrite_hook(void* a1, void* view) {
  if (!g_diag_probes.load(std::memory_order_relaxed) && !g_shadow_fix.load(std::memory_order_relaxed)) {
    g_shadow_write(a1, view);
    return;
  }
  const int vi = shadow_view_index(view);
  unsigned char* slot = vi >= 0 ? shadow_slot() : nullptr;
  if (slot == nullptr) {
    g_shadow_write(a1, view);
    if (vi < 0) g_shadow_other.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  ShadowView& v = g_shadow_views[vi];
  void* drop = nullptr;
  AcquireSRWLockExclusive(&g_shadow_lock);
  g_shadow_write(a1, view);
  std::uint64_t e[5] = {};
  if (read_bytes(slot, e, sizeof(e))) {
    void* p = reinterpret_cast<void*>(e[0]);
    if (p != v.held) {
      drop = v.held;
      v.held = p != nullptr && ref_add(p) ? p : nullptr;
    }
    std::memcpy(v.entry, e, sizeof(e));
    v.view = view;
  }
  ReleaseSRWLockExclusive(&g_shadow_lock);
  ref_release(drop);
  v.writes.fetch_add(1, std::memory_order_relaxed);
}

void ShadowRead_hook(void* a1, void* view) {
  if (!g_diag_probes.load(std::memory_order_relaxed) && !g_shadow_fix.load(std::memory_order_relaxed)) {
    g_shadow_read(a1, view);
    return;
  }
  const int vi = shadow_view_index(view);
  unsigned char* slot = vi >= 0 ? shadow_slot() : nullptr;
  if (slot == nullptr) {
    g_shadow_read(a1, view);
    return;
  }
  ShadowView& v = g_shadow_views[vi];
  v.reads.fetch_add(1, std::memory_order_relaxed);
  std::uint64_t old[5] = {}, mine[5] = {};
  bool swapped = false;
  AcquireSRWLockExclusive(&g_shadow_lock);
  if (v.view != view || v.held == nullptr) {
    v.unknown.fetch_add(1, std::memory_order_relaxed);
  } else if (read_bytes(slot, old, sizeof(old)) && old[0] != v.entry[0]) {
    v.mismatched.fetch_add(1, std::memory_order_relaxed);
    std::memcpy(mine, v.entry, sizeof(mine));
    if (g_shadow_fix.load(std::memory_order_relaxed) && ref_add(reinterpret_cast<void*>(mine[0]))) {
      // the slot now owns the added reference on ours; we hold the slot's former reference on the old entry
      swapped = write_bytes(slot, mine, sizeof(mine));
      if (!swapped) ref_release(reinterpret_cast<void*>(mine[0]));
    }
  }
  ReleaseSRWLockExclusive(&g_shadow_lock);
  g_shadow_read(a1, view);
  if (!swapped) return;
  v.swapped.fetch_add(1, std::memory_order_relaxed);
  void* drop = nullptr;
  AcquireSRWLockExclusive(&g_shadow_lock);
  std::uint64_t cur = 0;
  if (read_bytes(slot, &cur, sizeof(cur)) && cur == mine[0] && write_bytes(slot, old, sizeof(old))) {
    drop = reinterpret_cast<void*>(mine[0]);  // the slot owns the old entry again
  } else {
    drop = reinterpret_cast<void*>(old[0]);  // a writer replaced ours (and released it) meanwhile
    v.overwritten.fetch_add(1, std::memory_order_relaxed);
  }
  ReleaseSRWLockExclusive(&g_shadow_lock);
  ref_release(drop);
}

void log_shadow_slot() {
  if (!g_diag_probes.load()) return;
  for (int i = 0; i < 2; ++i) {
    const ShadowView& v = g_shadow_views[i];
    log_line("shadowslot %s: fix=%d writes=%llu reads=%llu mismatched=%llu swapped=%llu overwritten=%llu unknown=%llu other=%llu",
             i == 0 ? "main" : "second", g_shadow_fix.load() ? 1 : 0, static_cast<unsigned long long>(v.writes.load()),
             static_cast<unsigned long long>(v.reads.load()), static_cast<unsigned long long>(v.mismatched.load()),
             static_cast<unsigned long long>(v.swapped.load()), static_cast<unsigned long long>(v.overwritten.load()),
             static_cast<unsigned long long>(v.unknown.load()), static_cast<unsigned long long>(g_shadow_other.load()));
  }
}

// ---- per-view light queue probe ("lightq", ISSUE-04, 2026-10-21; read-only) ------------------------------------------
// [STATIC] 0x14022D950(owner, view) is the light job each view queues (registration 0x1402A7CA0, view job list +0x49F0).
// It walks the owner's pending light list (owner+0x18 pointers, owner+0x20 count): every active light (light+0xE0 bit 0)
// is frustum-tested against THIS view and survivors are appended to the view frame's list (frame+0x7D48, count +0x7D50;
// frame = [view+0x9EB0] + [view+0x9EC4]*0x1C070). Afterwards ALL processed active lights are removed from the owner list.
// Before that it stamps light+0x55 = index into this view's shadow-light list (view+0x1088, stride 0x138, light pointer
// at +0, count +0x1090) under a shared lock only. If both eyes share one owner, the first eye consumes lights the second
// never sees, and concurrent calls race on light+0x55. This probe measures exactly that, without changing anything.
constexpr std::uintptr_t kLightJobRva = 0x22D950;
constexpr unsigned char kLightJobSig[] = {0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x41, 0x55, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24};
using LightJobFn = void (*)(void*, void*);
LightJobFn g_light_job = nullptr;
struct LightQView {
  std::atomic<std::uint64_t> calls{0}, pending_in{0}, consumed{0}, added{0}, starved{0}, after_other{0}, empty_in{0};
  std::atomic<std::uint64_t> shadow_lists{0}, shadow_only_mine{0}, shadow_diff_calls{0}, shadow_total{0};
  void* owner = nullptr;      // g_lightq_mutex
  void* shadow[64] = {};      // latest shadow-light list (light pointers), g_lightq_mutex
  int shadow_n = 0;
  std::uint32_t last_pending_in = 0;
};
LightQView g_lightq[2];  // 0 = main view (left eye), 1 = created view (right eye)
std::mutex g_lightq_mutex;
std::atomic<int> g_lightq_inside{0};
std::atomic<std::uint64_t> g_lightq_overlap{0}, g_lightq_owner_same{0}, g_lightq_owner_diff{0}, g_lightq_other{0};
void* g_lightq_last_owner_caller_owner = nullptr;  // g_lightq_mutex: owner of the previous call
int g_lightq_last_caller = -1;                      // g_lightq_mutex: view index of the previous call

unsigned char* lightq_frame(void* view) {
  unsigned char* ring = nullptr;
  std::uint32_t slot = 0;
  if (!read_ptr(static_cast<unsigned char*>(view) + 0x9EB0, reinterpret_cast<void**>(&ring)) || ring == nullptr ||
      !read_u32(static_cast<unsigned char*>(view) + 0x9EC4, &slot) || slot > 7) {
    return nullptr;
  }
  return ring + static_cast<std::uintptr_t>(slot) * 0x1C070;
}

void LightJob_hook(void* owner, void* view) {
  if (!g_diag_probes.load(std::memory_order_relaxed)) {
    g_light_job(owner, view);
    return;
  }
  const int vi = shadow_view_index(view);  // main / created view while stereo runs (shadow-slot probe helper)
  if (vi < 0 || owner == nullptr) {
    if (vi < 0) g_lightq_other.fetch_add(1, std::memory_order_relaxed);
    g_light_job(owner, view);
    return;
  }
  LightQView& v = g_lightq[vi];
  std::uint32_t pin = 0, pout = 0, fin = 0, fout = 0, shadow_n = 0;
  unsigned char* frame = lightq_frame(view);
  read_u32(static_cast<unsigned char*>(owner) + 0x20, &pin);
  if (frame != nullptr) read_u32(frame + 0x7D50, &fin);
  // this view's shadow-light list as the job will use it
  void* shadow[64] = {};
  {
    void* list = nullptr;
    if (read_u32(static_cast<unsigned char*>(view) + 0x1090, &shadow_n) && shadow_n <= 64 &&
        read_ptr(static_cast<unsigned char*>(view) + 0x1088, &list) && list != nullptr) {
      for (std::uint32_t i = 0; i < shadow_n; ++i) {
        read_ptr(static_cast<unsigned char*>(list) + static_cast<std::uintptr_t>(i) * 0x138, &shadow[i]);
      }
    } else {
      shadow_n = 0;
    }
  }
  if (g_lightq_inside.fetch_add(1, std::memory_order_acq_rel) > 0) g_lightq_overlap.fetch_add(1, std::memory_order_relaxed);
  g_light_job(owner, view);
  g_lightq_inside.fetch_sub(1, std::memory_order_acq_rel);
  read_u32(static_cast<unsigned char*>(owner) + 0x20, &pout);
  if (frame != nullptr) read_u32(frame + 0x7D50, &fout);
  v.calls.fetch_add(1, std::memory_order_relaxed);
  v.pending_in.fetch_add(pin, std::memory_order_relaxed);
  if (pin == 0) v.empty_in.fetch_add(1, std::memory_order_relaxed);
  if (pin >= pout) v.consumed.fetch_add(pin - pout, std::memory_order_relaxed);
  if (fout >= fin) v.added.fetch_add(fout - fin, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(g_lightq_mutex);
  v.owner = owner;
  v.last_pending_in = pin;
  void* other_owner = g_lightq[vi ^ 1].owner;
  if (other_owner != nullptr) (other_owner == owner ? g_lightq_owner_same : g_lightq_owner_diff).fetch_add(1, std::memory_order_relaxed);
  // called right after the other eye on the same owner: did that eye leave fewer pending lights for this one?
  if (g_lightq_last_caller == (vi ^ 1) && g_lightq_last_owner_caller_owner == owner) {
    v.after_other.fetch_add(1, std::memory_order_relaxed);
    if (pin < g_lightq[vi ^ 1].last_pending_in) v.starved.fetch_add(1, std::memory_order_relaxed);
  }
  g_lightq_last_caller = vi;
  g_lightq_last_owner_caller_owner = owner;
  // shadow-light sets: compare with the other eye's latest list
  std::memcpy(v.shadow, shadow, sizeof(v.shadow));
  v.shadow_n = static_cast<int>(shadow_n);
  v.shadow_lists.fetch_add(1, std::memory_order_relaxed);
  v.shadow_total.fetch_add(shadow_n, std::memory_order_relaxed);
  const LightQView& o = g_lightq[vi ^ 1];
  if (o.shadow_n > 0 || shadow_n > 0) {
    std::uint64_t only = 0;
    for (std::uint32_t i = 0; i < shadow_n; ++i) {
      bool found = false;
      for (int j = 0; j < o.shadow_n && !found; ++j) found = o.shadow[j] == shadow[i];
      if (!found) ++only;
    }
    v.shadow_only_mine.fetch_add(only, std::memory_order_relaxed);
    if (only > 0 || static_cast<int>(shadow_n) != o.shadow_n) v.shadow_diff_calls.fetch_add(1, std::memory_order_relaxed);
  }
}

void log_light_queue() {
  if (!g_diag_probes.load()) return;
  if (g_light_job == nullptr) return;
  for (int i = 0; i < 2; ++i) {
    const LightQView& v = g_lightq[i];
    log_line("lightq %s: calls=%llu pending_in=%llu empty_in=%llu consumed=%llu added=%llu after_other=%llu starved=%llu | "
             "shadow lists=%llu avg_size=%.2f differ=%llu only_this_eye=%llu",
             i == 0 ? "main" : "second", static_cast<unsigned long long>(v.calls.load()),
             static_cast<unsigned long long>(v.pending_in.load()), static_cast<unsigned long long>(v.empty_in.load()),
             static_cast<unsigned long long>(v.consumed.load()), static_cast<unsigned long long>(v.added.load()),
             static_cast<unsigned long long>(v.after_other.load()), static_cast<unsigned long long>(v.starved.load()),
             static_cast<unsigned long long>(v.shadow_lists.load()),
             v.shadow_lists.load() ? static_cast<double>(v.shadow_total.load()) / static_cast<double>(v.shadow_lists.load()) : 0.0,
             static_cast<unsigned long long>(v.shadow_diff_calls.load()),
             static_cast<unsigned long long>(v.shadow_only_mine.load()));
  }
  log_line("lightq: owner same=%llu diff=%llu overlap=%llu other_views=%llu", static_cast<unsigned long long>(g_lightq_owner_same.load()),
           static_cast<unsigned long long>(g_lightq_owner_diff.load()), static_cast<unsigned long long>(g_lightq_overlap.load()),
           static_cast<unsigned long long>(g_lightq_other.load()));
}

// ---- shared spot-shadow light choice ("shadowunion", ISSUE-04, 2026-10-21) --------------------------------------------
// [STATIC] 0x1405B0F30(renderer = view+0x1C0) chooses each view's shadow-casting lights from the world list: score =
// radius / (distance from renderer+0x8D0..0x8D8 + 11), halved when outside the 6 planes at [renderer+0x980] (16 bytes
// each: nx ny nz d, inside >= 0), x1.2 when the light was already in this view's previous list (view+0x1088), then the
// top N under a slot budget get shadow maps. [LIVE] lightq 12:36: the eyes' lists differ in ~96% of frames, ~4.5 lights
// each only in one eye -- a shadow in one eye only. "shadowunion": for each eye's call the planes pointer is pointed at a
// merged frustum (per plane the looser of the two eyes' planes, judged at a point 10 m ahead) and the position at the
// mid-eye point, so both eyes score every light identically. The engine's plane memory is never written; the pointer and
// position are restored right after the call. Measured by the lightq "shadow differ" counter.
constexpr std::uintptr_t kShadowPickRva = 0x5B0F30;
constexpr unsigned char kShadowPickSig[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x48, 0x08, 0x55, 0x41, 0x57, 0x48, 0x8D, 0xA8};
using ShadowPickFn = void (*)(void*);
ShadowPickFn g_shadow_pick = nullptr;
std::atomic<bool> g_shadow_union{false};
std::mutex g_pick_mutex;
struct PickEye {
  float planes[24] = {};
  float pos[3] = {};
  bool valid = false;
};
PickEye g_pick_eye[2];  // g_pick_mutex: latest planes and position each eye selected with
thread_local float t_pick_planes[24];
std::atomic<std::uint64_t> g_pick_calls[2], g_pick_merged{0}, g_pick_skipped{0};

void ShadowPick_hook(void* renderer) {
  if (!g_shadow_union.load(std::memory_order_relaxed)) {
    g_shadow_pick(renderer);
    return;
  }
  unsigned char* r = static_cast<unsigned char*>(renderer);
  const int vi = renderer != nullptr ? shadow_view_index(r - 0x1C0) : -1;
  if (vi < 0) {
    g_shadow_pick(renderer);
    return;
  }
  g_pick_calls[vi].fetch_add(1, std::memory_order_relaxed);
  void* planes_ptr = nullptr;
  float own[24], pos[3];
  if (!read_ptr(r + 0x980, &planes_ptr) || planes_ptr == nullptr || !read_bytes(planes_ptr, own, sizeof(own)) ||
      !read_bytes(r + 0x8D0, pos, sizeof(pos))) {
    g_pick_skipped.fetch_add(1, std::memory_order_relaxed);
    g_shadow_pick(renderer);
    return;
  }
  PickEye other;
  {
    std::lock_guard<std::mutex> lock(g_pick_mutex);
    std::memcpy(g_pick_eye[vi].planes, own, sizeof(own));
    std::memcpy(g_pick_eye[vi].pos, pos, sizeof(pos));
    g_pick_eye[vi].valid = true;
    other = g_pick_eye[vi ^ 1];
  }
  if (!g_shadow_union.load(std::memory_order_relaxed) || !other.valid) {
    g_shadow_pick(renderer);
    return;
  }
  // mid-eye position; reference point 10 m along the inward normal sum of the near/far pair is unknown, so use the
  // centroid direction of all six normals (they point inward), which lies inside both frusta
  float mid[3], dir[3] = {0.f, 0.f, 0.f};
  for (int k = 0; k < 3; ++k) mid[k] = 0.5f * (pos[k] + other.pos[k]);
  for (int i = 0; i < 6; ++i) for (int k = 0; k < 3; ++k) dir[k] += own[i * 4 + k];
  const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
  if (!(len > 1e-3f)) {
    g_pick_skipped.fetch_add(1, std::memory_order_relaxed);
    g_shadow_pick(renderer);
    return;
  }
  float ref[3];
  for (int k = 0; k < 3; ++k) ref[k] = mid[k] + dir[k] / len * 10.f;
  float* merged = t_pick_planes;
  for (int i = 0; i < 6; ++i) {
    const float* a = own + i * 4;
    const float* b = other.planes + i * 4;
    const float da = a[0] * ref[0] + a[1] * ref[1] + a[2] * ref[2] + a[3];
    const float db = b[0] * ref[0] + b[1] * ref[1] + b[2] * ref[2] + b[3];
    std::memcpy(merged + i * 4, db > da ? b : a, 4 * sizeof(float));  // looser plane: the reference point is deeper inside
  }
  void* merged_ptr = merged;
  if (!write_bytes(r + 0x980, &merged_ptr, sizeof(merged_ptr))) {
    g_pick_skipped.fetch_add(1, std::memory_order_relaxed);
    g_shadow_pick(renderer);
    return;
  }
  write_bytes(r + 0x8D0, mid, sizeof(mid));
  g_shadow_pick(renderer);
  write_bytes(r + 0x980, &planes_ptr, sizeof(planes_ptr));
  write_bytes(r + 0x8D0, pos, sizeof(pos));
  g_pick_merged.fetch_add(1, std::memory_order_relaxed);
}

void log_shadow_pick() {
  if (g_shadow_pick == nullptr) return;
  log_line("shadowunion: %s calls main=%llu second=%llu merged=%llu skipped=%llu", g_shadow_union.load() ? "on" : "off",
           static_cast<unsigned long long>(g_pick_calls[0].load()), static_cast<unsigned long long>(g_pick_calls[1].load()),
           static_cast<unsigned long long>(g_pick_merged.load()), static_cast<unsigned long long>(g_pick_skipped.load()));
}

// ---- detail independent of the VR FOV ("lodfov", ISSUE-04, 2026-10-21) ------------------------------------------------
// [STATIC] camera update 0x143073370(cam) writes cam+0x79C = (viewport width * 0.5) / tan(cam+0x220 * 0.5), the LOD focal
// length in pixels (decomp 143000000.c:78757-78766; ortho cameras, cam+0x23C != 0, use another formula). Mesh LOD
// (0x14036EC90, cull view setup 0x1402F9080), terrain/scatter quadtree LOD and texture-streaming mip requests
// (0x143085AB0 callers) all multiply by it, so at a fixed eye width a wider VR FOV makes every detail decision act as if
// objects were further away. [AUTHOR] lower VR FOV = better object/texture detail, wider = shapes change on head turns.
// Same bug the Cyberpunk VR port fixed (src/Hooks/LodFov.cpp: LOD threshold clamped to the flat-screen value). "lodfov"
// rewrites only +0x79C after each update of OUR eye cameras -- the private eye cameras and their per-frame sub-cameras
// (view frame ring +0xEEF0 + j*0x8B0, the kSubCams) whose FOV equals the eye FOV -- to (width/2)/tan(ref/2), ref = the
// game camera's own FOV or "lodref <deg>". The FOV fields, projection and culling planes are untouched.
constexpr std::uintptr_t kCamUpdateRva = 0x3073370;
constexpr unsigned char kCamUpdateSig[] = {0x40, 0x55, 0x56, 0x48, 0x8D, 0xAC, 0x24, 0x88, 0xFA, 0xFF, 0xFF, 0x48, 0x81, 0xEC};
using CamUpdateFn = void* (*)(void*);
CamUpdateFn g_cam_update = nullptr;
std::atomic<bool> g_lod_fov{false};
std::atomic<int> g_lod_ref_mdeg{0};  // 0 = the game camera's own FOV
std::atomic<std::uint64_t> g_lod_hits_priv{0}, g_lod_hits_slot{0}, g_lod_written{0};
std::atomic<int> g_lod_old_focal[2] = {}, g_lod_new_focal[2] = {};  // x1000, per eye (log)
std::atomic<int> g_lod_ref_used_mdeg{0};
std::atomic<std::uint32_t> g_lod_ratio_bits{0x3F800000u};  // orig/new focal of the last overridden eye camera

// Eye index (0 left/main, 1 right/created) when cam is one of our eye cameras, else -1.
int lod_eye_camera(const unsigned char* cam, bool* slot) {
  *slot = false;
  if (cam == nullptr) return -1;
  if (cam == g_priv_left) return 0;
  if (cam == g_priv_cam) return 1;
  void* views[2] = {g_created_main, g_created};
  for (int e = 0; e < 2; ++e) {
    unsigned char* ring = nullptr;
    if (views[e] == nullptr || !read_ptr(static_cast<unsigned char*>(views[e]) + kFrameRing, reinterpret_cast<void**>(&ring)) ||
        ring == nullptr || cam < ring) {
      continue;
    }
    const std::uintptr_t off = static_cast<std::uintptr_t>(cam - ring);
    if (off >= 8u * kFrameStride) continue;
    const std::uintptr_t r = off % kFrameStride;
    for (std::uint32_t sub : kSubCams) {
      if (r == sub) {
        *slot = true;
        return e;
      }
    }
  }
  return -1;
}

void* CamUpdate_hook(void* cam_v) {
  void* ret = g_cam_update(cam_v);
  if (!g_lod_fov.load(std::memory_order_relaxed) || !g_stereo_running.load(std::memory_order_relaxed)) return ret;
  unsigned char* cam = static_cast<unsigned char*>(cam_v);
  bool slot = false;
  const int eye = lod_eye_camera(cam, &slot);
  if (eye < 0) return ret;
  std::uint8_t ortho = 1;
  float fov = 0.f, old_focal = 0.f;
  std::int32_t x0 = 0, x1 = 0;
  if (!read_u8(cam + 0x23C, &ortho) || ortho != 0 || !read_bytes(cam + 0x220, &fov, sizeof(fov)) ||
      !read_bytes(cam + 0x254, &x0, sizeof(x0)) || !read_bytes(cam + 0x25C, &x1, sizeof(x1)) ||
      !read_bytes(cam + 0x79C, &old_focal, sizeof(old_focal))) {
    return ret;
  }
  if (slot) {  // only the sub-camera that carries the eye's own projection
    const float eye_fov = static_cast<float>(g_auto_last_fov_mdeg.load(std::memory_order_relaxed)) * 1e-3f * 0.0174532925f;
    if (!(eye_fov > 0.2f) || std::fabs(fov - eye_fov) > 2e-3f) return ret;
  }
  float ref = static_cast<float>(g_lod_ref_mdeg.load(std::memory_order_relaxed)) * 1e-3f * 0.0174532925f;
  if (!(ref > 0.3f) && g_created_main_cam != nullptr) {
    read_bytes(static_cast<unsigned char*>(g_created_main_cam) + 0x100, &ref, sizeof(ref));  // the game's own FOV
  }
  const float width = static_cast<float>(x1 - x0);
  if (!(ref > 0.3f && ref < 2.8f) || !(width > 16.f)) return ret;
  const float focal = width * 0.5f / std::tan(ref * 0.5f);
  if (write_bytes(cam + 0x79C, &focal, sizeof(focal))) {
    (slot ? g_lod_hits_slot : g_lod_hits_priv).fetch_add(1, std::memory_order_relaxed);
    g_lod_written.fetch_add(1, std::memory_order_relaxed);
    g_lod_old_focal[eye].store(static_cast<int>(old_focal * 1000.f), std::memory_order_relaxed);
    g_lod_new_focal[eye].store(static_cast<int>(focal * 1000.f), std::memory_order_relaxed);
    g_lod_ref_used_mdeg.store(static_cast<int>(ref * 57295.78f), std::memory_order_relaxed);
    const float ratio = std::tan(ref * 0.5f) / std::tan(fov * 0.5f);  // original / new focal (width cancels)
    std::uint32_t bits;
    std::memcpy(&bits, &ratio, sizeof(bits));
    g_lod_ratio_bits.store(bits, std::memory_order_relaxed);
  }
  return ret;
}

void log_lod_fov() {
  if (g_cam_update == nullptr) return;
  log_line("lodfov: %s ref=%.1f deg hits priv=%llu slot=%llu | focal px old L/R %.0f/%.0f -> new %.0f/%.0f",
           g_lod_fov.load() ? "on" : "off", g_lod_ref_used_mdeg.load() * 1e-3,
           static_cast<unsigned long long>(g_lod_hits_priv.load()), static_cast<unsigned long long>(g_lod_hits_slot.load()),
           g_lod_old_focal[0].load() * 1e-3, g_lod_old_focal[1].load() * 1e-3, g_lod_new_focal[0].load() * 1e-3,
           g_lod_new_focal[1].load() * 1e-3);
}

// ---- lodfov consumer isolation ("lodkeep N", ISSUE-04, 2026-10-21) ----------------------------------------------------
// [HEADSET/AUTHOR] lodfov on at ref 50 deg: objects render consistently, but bioluminescent fauna never glows, even up
// close. Some consumer of the LOD focal gates the glow and breaks with the higher value. "lodkeep N" gives ONE consumer
// group the engine's original focal again (orig/new = tan(ref/2)/tan(eyeFov/2)) while all others keep the reference:
// 1 mesh LOD scale 0x14036EC90 (return), 2 cull view 0x1402F9080 (+0x20/+0x24), 3 texture streaming 0x143085AB0 /
// 0x143085AD0 (pixels-per-metre argument, all callers), 4 0x14037E9D0 (+0x86C, unidentified system), 5 = 1-4 together.
// [HEADSET/AUTHOR 16:29-16:31] group 2 (and 5) restore the fauna glow but bring back the head-turn culling/shape changes.
// The cull view holds two scales: +0x20 = k * settings[+0x8C] * focal and +0x24 = k * settings[+0x90] * focal
// (decomp 0x1402F9080). 6 restores only +0x20, 7 only +0x24, to see whether glow and instability ride on different ones.
// The group that brings the glow back owns it. Read-only on shared camera memory: only returns/outputs/arguments change.
constexpr std::uintptr_t kLodScaleRva = 0x36EC90;
constexpr unsigned char kLodScaleSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x30, 0x8B, 0x81, 0x50, 0x02};
constexpr std::uintptr_t kCullViewRva = 0x2F9080;
constexpr unsigned char kCullViewSig[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x08, 0x48, 0x89, 0x68, 0x10, 0x48, 0x89, 0x70};
constexpr std::uintptr_t kStreamMipRva = 0x3085AB0;
constexpr unsigned char kStreamMipSig[] = {0x48, 0x85, 0xD2, 0x74, 0x15, 0x48, 0x8B, 0x52, 0x48, 0x48, 0x85, 0xD2};
constexpr std::uintptr_t kStreamMip2Rva = 0x3085AD0;
constexpr unsigned char kStreamMip2Sig[] = {0x48, 0x83, 0xEC, 0x48, 0x49, 0x8B, 0x01, 0x48, 0x89, 0x5C, 0x24, 0x50};
constexpr std::uintptr_t kFocalUser4Rva = 0x37E9D0;
constexpr unsigned char kFocalUser4Sig[] = {0x4C, 0x89, 0x44, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x55, 0x53};
using LodScaleFn = float (*)(void*, void*, char);
using CullViewFn = void* (*)(void*, void*, unsigned char);
using StreamMipFn = void (*)(void*, void*, float);
using StreamMip2Fn = void (*)(void*, void*, float, void*, float);
using FocalUser4Fn = void (*)(void*, void*, void*);
LodScaleFn g_lod_scale = nullptr;
CullViewFn g_cull_view = nullptr;
StreamMipFn g_stream_mip = nullptr;
StreamMip2Fn g_stream_mip2 = nullptr;
FocalUser4Fn g_focal_user4 = nullptr;
std::atomic<int> g_lod_keep{0};
// Performance pass 2026-10-22: "lodtexgame" keeps texture-streaming requests on the engine's own focal while lodfov
// raises mesh/terrain detail (same as lodkeep group 3, which kept shapes stable in the 10-21 glow test). Cuts texture
// bandwidth/VRAM that the 2.2-2.7x focal would otherwise request.
std::atomic<bool> g_lod_tex_game{false};
std::atomic<std::uint64_t> g_keep_hits[5];
thread_local float t_lod_scale_seen = 0.f;  // last mesh LOD scale returned on this thread (lodprobe)
// [HEADSET/AUTHOR 16:42] 6 (+0x20) = glow + head-turn instability, 7 (+0x24) = stable, no glow: both ride on +0x20.
// [STATIC] 0x1402F9080 is called by separate systems: 0x1402277E0 (from the light-setup job 0x140291EE0), 0x1402F8DD0
// (particle cull jobs / avr::ParticleQueue), 0x14391A3F0 and 0x143CB8820 (unidentified). 8-11 restore +0x20 only when
// called from that one caller (return address), so e.g. light culling can keep the original while meshes keep 50 deg.
std::uintptr_t g_game_base = 0;
struct CullCaller { std::uintptr_t rva, size; const char* name; };
constexpr CullCaller kCullCallers[4] = {{0x2277E0, 2616, "lights"}, {0x2F8DD0, 674, "particles"}, {0x391A3F0, 3483, "c391A3F0"},
                                        {0x3CB8820, 807, "c3CB8820"}};
std::atomic<std::uint64_t> g_cull_caller_hits[5];  // eye-view calls per caller (index 4 = other)

float lod_ratio() {
  const std::uint32_t b = g_lod_ratio_bits.load(std::memory_order_relaxed);
  float r;
  std::memcpy(&r, &b, sizeof(r));
  return r > 0.05f && r < 20.f ? r : 1.f;
}
bool lod_keep(int group) {
  if (!g_lod_fov.load(std::memory_order_relaxed)) return false;
  const int k = g_lod_keep.load(std::memory_order_relaxed);
  return k == group || k == 5;
}

float LodScale_hook(void* cam, void* settings, char alt) {
  const float v = g_lod_scale(cam, settings, alt);
  bool slot = false;
  if (!lod_keep(1) || lod_eye_camera(static_cast<unsigned char*>(cam), &slot) < 0) {
    t_lod_scale_seen = v;
    return v;
  }
  g_keep_hits[1].fetch_add(1, std::memory_order_relaxed);
  t_lod_scale_seen = v * lod_ratio();
  return t_lod_scale_seen;
}

void* CullView_hook(void* out, void* view, unsigned char idx) {
  const std::uintptr_t ret = reinterpret_cast<std::uintptr_t>(_ReturnAddress()) - g_game_base;
  void* r = g_cull_view(out, view, idx);
  int caller = 4;
  for (int c = 0; c < 4; ++c) {
    if (ret >= kCullCallers[c].rva && ret < kCullCallers[c].rva + kCullCallers[c].size) caller = c;
  }
  if (shadow_view_index(view) >= 0) g_cull_caller_hits[caller].fetch_add(1, std::memory_order_relaxed);
  const int keep = g_lod_keep.load(std::memory_order_relaxed);
  // callers to restore: 6 all, 8-11 one each, 12 lights+particles, 13 the two unidentified, 14 lights+unidentified,
  // 15 particles+unidentified ([HEADSET/AUTHOR 16:53] 8-11 alone: stable, no glow; 6: glow + instability)
  static constexpr unsigned kMask[16] = {0, 0, 0, 0, 0, 0, 0xF, 0, 1, 2, 4, 8, 3, 12, 13, 14};
  const unsigned mask = keep >= 0 && keep < 16 ? kMask[keep] : 0;
  const bool first = lod_keep(2) || (g_lod_fov.load(std::memory_order_relaxed) && caller < 4 && ((mask >> caller) & 1u));
  const bool second = lod_keep(2) || (g_lod_fov.load(std::memory_order_relaxed) && keep == 7);
  if (!(first || second) || shadow_view_index(view) < 0 || out == nullptr) return r;
  float v[2];
  unsigned char* o = static_cast<unsigned char*>(out);
  if (read_bytes(o + 0x20, v, sizeof(v))) {
    const float k = lod_ratio();
    if (first) v[0] *= k;
    if (second) v[1] *= k;
    if (write_bytes(o + 0x20, v, sizeof(v))) g_keep_hits[2].fetch_add(1, std::memory_order_relaxed);
  }
  return r;
}

void StreamMip_hook(void* streamer, void* tex, float ppm) {
  if (lod_keep(3) || (g_lod_tex_game.load(std::memory_order_relaxed) && g_lod_fov.load(std::memory_order_relaxed))) {
    ppm *= lod_ratio();  // ~280k calls/s: no shared counter here
  }
  g_stream_mip(streamer, tex, ppm);
}

void StreamMip2_hook(void* streamer, void* lock, float ppm, void* out, float extra) {
  if (lod_keep(3) || (g_lod_tex_game.load(std::memory_order_relaxed) && g_lod_fov.load(std::memory_order_relaxed))) {
    ppm *= lod_ratio();
  }
  g_stream_mip2(streamer, lock, ppm, out, extra);
}

void FocalUser4_hook(void* out, void* view, void* settings) {
  g_focal_user4(out, view, settings);
  if (!lod_keep(4) || shadow_view_index(view) < 0 || out == nullptr) return;
  float v = 0.f;
  unsigned char* o = static_cast<unsigned char*>(out);
  if (read_bytes(o + 0x86C, &v, sizeof(v))) {
    v *= lod_ratio();
    if (write_bytes(o + 0x86C, &v, sizeof(v))) g_keep_hits[4].fetch_add(1, std::memory_order_relaxed);
  }
}

void log_lod_keep() {
  if (g_cam_update == nullptr) return;
  log_line("lodkeep: cull-view eye calls lights=%llu particles=%llu c391A3F0=%llu c3CB8820=%llu other=%llu",
           static_cast<unsigned long long>(g_cull_caller_hits[0].load()), static_cast<unsigned long long>(g_cull_caller_hits[1].load()),
           static_cast<unsigned long long>(g_cull_caller_hits[2].load()), static_cast<unsigned long long>(g_cull_caller_hits[3].load()),
           static_cast<unsigned long long>(g_cull_caller_hits[4].load()));
  log_line("lodkeep: group=%d ratio=%.3f hits mesh=%llu cull=%llu stream=%llu g4=%llu", g_lod_keep.load(), lod_ratio(),
           static_cast<unsigned long long>(g_keep_hits[1].load()), static_cast<unsigned long long>(g_keep_hits[2].load()),
           static_cast<unsigned long long>(g_keep_hits[3].load()), static_cast<unsigned long long>(g_keep_hits[4].load()));
}

// ---- per-eye LOD near/far decision probe ("lodprobe", ISSUE-04, 2026-10-21; read-only) --------------------------------
// [HEADSET/AUTHOR] across every lodkeep combination, distant fauna glow and head-turn shape changes come and go together:
// one near/far switch. [STATIC] 0x140295E70(owner = world+0x6300, view, ctx) walks owner+0x18 (count +0x20); for an object
// whose +0xCC/+0xD0 match +0x38/+0x3C and whose +0xB0/+0xC0 slots are empty, it skips it as NEAR when
//   sqrt(farthest bbox corner offset from eye pos subblock+0x150..0x158) < lodScale * obj+0x60 - k   (k = [0x148DB732C])
// after a plane test |n . corner + d| <= [0x148DB6F8C] against the eye camera's first culling plane (cam+0x7A0).
// lodScale = 0x14036EC90(subblock, frame+0x20, 0), captured per thread by LodScale_hook during the call.
// The probe re-evaluates that decision per object after each eye's call and counts eye disagreements in the same frame,
// per-eye frame-to-frame flips (head still vs turning) and objects within 2% of the threshold. Nothing is written.
constexpr std::uintptr_t kNearFarRva = 0x295E70;
constexpr unsigned char kNearFarSig[] = {0x4C, 0x89, 0x44, 0x24, 0x18, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x53, 0x55, 0x56, 0x57};
constexpr std::uintptr_t kNearFarKRva = 0x8DB732C, kNearFarPlaneRva = 0x8DB6F8C;
using NearFarFn = void (*)(void*, void*, void*);
NearFarFn g_near_far = nullptr;
std::atomic<bool> g_lod_probe{false};
std::mutex g_lodp_mutex;
struct LodpEye {
  std::unordered_map<const void*, std::uint8_t> cur, prev;  // object -> 1 near / 0 far (g_lodp_mutex)
  std::uint64_t frame = 0;
  std::uint64_t calls = 0, objects = 0, near_n = 0, edge = 0, flips_still = 0, flips_turn = 0, frames_still = 0, frames_turn = 0;
  float last_scale = 0.f;
};
LodpEye g_lodp[2];
std::uint64_t g_lodp_pairs = 0, g_lodp_disagree = 0, g_lodp_compared = 0;  // g_lodp_mutex
float g_lodp_last_yaw = 0.f;
bool g_lodp_have_yaw = false;
bool g_lodp_turning = false;  // g_lodp_mutex: head yaw changed > 0.15 deg since the previous main-eye call
float g_lodp_yaw_sum = 0.f;

void NearFar_hook(void* owner, void* view, void* ctx) {
  const int vi = g_lod_probe.load(std::memory_order_relaxed) ? shadow_view_index(view) : -1;
  if (vi < 0 || owner == nullptr) {
    g_near_far(owner, view, ctx);
    return;
  }
  t_lod_scale_seen = 0.f;
  g_near_far(owner, view, ctx);
  const float scale = t_lod_scale_seen;
  unsigned char* frame = lightq_frame(view);
  if (frame == nullptr || !(scale > 0.f)) return;
  std::int32_t eye_slot = 0;
  read_bytes(frame + 0x1BFA0, &eye_slot, sizeof(eye_slot));
  if (eye_slot < 0 || eye_slot > 2) eye_slot = 0;
  unsigned char* sub = frame + 0xEEF0 + static_cast<std::uintptr_t>(eye_slot) * 0x4590;
  float pos[3], plane[4], k = 10.f, plane_max = 900.f;
  if (!read_bytes(sub + 0x150, pos, sizeof(pos)) || !read_bytes(sub + 0x7A0, plane, sizeof(plane))) return;
  read_bytes(reinterpret_cast<unsigned char*>(g_game_base) + kNearFarKRva, &k, sizeof(k));
  read_bytes(reinterpret_cast<unsigned char*>(g_game_base) + kNearFarPlaneRva, &plane_max, sizeof(plane_max));
  void* list = nullptr;
  std::uint32_t count = 0;
  if (!read_ptr(static_cast<unsigned char*>(owner) + 0x18, &list) || list == nullptr ||
      !read_u32(static_cast<unsigned char*>(owner) + 0x20, &count) || count > 50000) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_lodp_mutex);
  LodpEye& e = g_lodp[vi];
  if (vi == 0) {  // head motion between main-eye calls (one per frame)
    head_pose::Snapshot s;
    if (xr_runtime::latest_views(&s)) {
      const float f[3] = {0.f, 0.f, -1.f};
      const float q[4] = {s.eye[0].qx, s.eye[0].qy, s.eye[0].qz, s.eye[0].qw};
      float d[3];
      quat_rot(q, f, d);
      const float yaw = std::atan2(-d[0], -d[2]) * 57.2958f;
      float dy = g_lodp_have_yaw ? yaw - g_lodp_last_yaw : 0.f;
      while (dy > 180.f) dy -= 360.f;
      while (dy < -180.f) dy += 360.f;
      g_lodp_turning = std::fabs(dy) > 0.15f;
      g_lodp_yaw_sum += std::fabs(dy);
      g_lodp_last_yaw = yaw;
      g_lodp_have_yaw = true;
    }
  }
  e.prev.swap(e.cur);
  e.cur.clear();
  e.calls++;
  e.frame++;
  e.last_scale = scale;
  (g_lodp_turning ? e.frames_turn : e.frames_still)++;
  for (std::uint32_t i = 0; i < count; ++i) {
    void* obj = nullptr;
    if (!read_ptr(static_cast<unsigned char*>(list) + static_cast<std::uintptr_t>(i) * 8, &obj) || obj == nullptr) continue;
    const unsigned char* o = static_cast<const unsigned char*>(obj);
    std::int32_t a[2] = {}, b[2] = {}, s0 = 0, s1 = 0;
    float bb[6], size = 0.f;
    if (!read_bytes(o + 0x38, a, sizeof(a)) || (a[0] == 0 && a[1] == 0) || !read_bytes(o + 0xCC, b, sizeof(b)) ||
        b[1] != a[1] || b[0] != a[0] || !read_bytes(o + 0xB0, &s0, 4) || !read_bytes(o + 0xC0, &s1, 4) || s0 != 0 || s1 != 0 ||
        !read_bytes(o + 0x48, bb, sizeof(bb)) || !read_bytes(o + 0x60, &size, 4)) {
      continue;
    }
    const float cx = plane[0] <= -0.f && plane[0] != -0.f ? bb[3] : bb[0];
    const float cy = plane[1] < -0.f ? bb[4] : bb[1];
    const float cz = plane[2] < -0.f ? bb[5] : bb[2];
    if (std::fabs(cx * plane[0] + cy * plane[1] + cz * plane[2] + plane[3]) > plane_max) continue;
    const float dx = std::max(std::fabs(pos[0] - bb[0]), std::fabs(pos[0] - bb[3]));
    const float dyy = std::max(std::fabs(pos[1] - bb[1]), std::fabs(pos[1] - bb[4]));
    const float dz = std::max(std::fabs(pos[2] - bb[2]), std::fabs(pos[2] - bb[5]));
    const float dist = std::sqrt(dx * dx + dyy * dyy + dz * dz);
    const float thr = scale * size - k;
    const std::uint8_t is_near = dist < thr ? 1 : 0;
    e.cur[obj] = is_near;
    e.objects++;
    e.near_n += is_near;
    if (std::fabs(dist - thr) < 0.02f * std::max(1.f, std::fabs(thr))) e.edge++;
    auto it = e.prev.find(obj);
    if (it != e.prev.end() && it->second != is_near) (g_lodp_turning ? e.flips_turn : e.flips_still)++;
  }
  // same-frame comparison with the other eye (its latest evaluation, at most one frame apart)
  const LodpEye& other = g_lodp[vi ^ 1];
  if (vi == 1 && !other.cur.empty() && (other.frame == e.frame || other.frame + 1 == e.frame || other.frame == e.frame + 1)) {
    g_lodp_pairs++;
    for (const auto& kv : e.cur) {
      auto it = other.cur.find(kv.first);
      if (it == other.cur.end()) continue;
      g_lodp_compared++;
      if (it->second != kv.second) g_lodp_disagree++;
    }
  }
}

void log_lod_probe() {
  if (g_near_far == nullptr || !g_lod_probe.load()) return;
  std::lock_guard<std::mutex> lock(g_lodp_mutex);
  for (int i = 0; i < 2; ++i) {
    const LodpEye& e = g_lodp[i];
    log_line("lodprobe %s: calls=%llu objects/call=%.1f near/call=%.1f edge2%%/call=%.2f flips still=%llu over %llu frames, "
             "turning=%llu over %llu frames, scale=%.1f",
             i == 0 ? "main" : "second", static_cast<unsigned long long>(e.calls),
             e.calls ? static_cast<double>(e.objects) / e.calls : 0.0, e.calls ? static_cast<double>(e.near_n) / e.calls : 0.0,
             e.calls ? static_cast<double>(e.edge) / e.calls : 0.0, static_cast<unsigned long long>(e.flips_still),
             static_cast<unsigned long long>(e.frames_still), static_cast<unsigned long long>(e.flips_turn),
             static_cast<unsigned long long>(e.frames_turn), e.last_scale);
  }
  log_line("lodprobe eyes: pairs=%llu compared=%llu disagree=%llu (%.3f%%) head yaw total=%.1f deg",
           static_cast<unsigned long long>(g_lodp_pairs), static_cast<unsigned long long>(g_lodp_compared),
           static_cast<unsigned long long>(g_lodp_disagree),
           g_lodp_compared ? 100.0 * static_cast<double>(g_lodp_disagree) / static_cast<double>(g_lodp_compared) : 0.0,
           g_lodp_yaw_sum);
}

// ---- terrain/scatter quadtree collapse probe ("qtprobe", ISSUE-04, 2026-10-21; read-only) -----------------------------
// [STATIC] 0x140685390(owner, uint* xy, level, frustumCtx, lodCtx, x) visits one quadtree tile (recursing into children;
// root from 0x140682C90). It frustum-tests the tile box (0x14037A390 / 0x14037BF00) BEFORE any distance test; a tile outside
// the view goes to 0x14069B020(owner, int* xy, level) (sole caller: this function), which collapses it. The LOD position is
// [*lodCtx]+0x30 (or the shared owner+0x14F0 when DAT_1496f75e0). [LIVE 19:10-19:14] the far-object near/far switch is
// eye-identical and does not flip with head turns, so this frustum-gated tile collapse is the next suspect for "shapes and
// detail change when I move my head". The probe counts visits and collapses per eye, and per tile the visible<->collapsed
// transitions between frames with the head still vs turning, plus tiles the eyes treat differently. Nothing is written.
constexpr std::uintptr_t kQtVisitRva = 0x685390;
constexpr unsigned char kQtVisitSig[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x10, 0x48, 0x89, 0x70, 0x18, 0x55, 0x57, 0x41, 0x54};
constexpr std::uintptr_t kQtCollapseRva = 0x69B020;
constexpr unsigned char kQtCollapseSig[] = {0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x20};
using QtVisitFn = void (*)(void*, unsigned*, unsigned, void*, void*, void*);
using QtCollapseFn = void (*)(void*, int*, int);
QtVisitFn g_qt_visit = nullptr;
QtCollapseFn g_qt_collapse = nullptr;
std::atomic<bool> g_qt_probe{false};
thread_local int t_qt_depth = 0, t_qt_eye = -1;
std::mutex g_qt_mutex;
struct QtTile {
  std::uint8_t state[2] = {2, 2};  // per eye: 1 visible (visited, not collapsed), 0 collapsed, 2 unknown
  std::uint64_t frame[2] = {0, 0};
};
std::unordered_map<std::uint64_t, QtTile> g_qt_tiles;  // g_qt_mutex
struct QtEye {
  std::uint64_t roots = 0, visits = 0, collapses = 0;
  std::uint64_t to_collapsed_still = 0, to_collapsed_turn = 0, to_visible_still = 0, to_visible_turn = 0;
};
QtEye g_qt_eye[2];
std::uint64_t g_qt_shared = 0, g_qt_other = 0, g_qt_frames_still = 0, g_qt_frames_turn = 0;  // g_qt_mutex
std::uint64_t g_qt_turn_frame = 0;
bool g_qt_turning = false;
float g_qt_last_yaw = 0.f;
bool g_qt_have_yaw = false;

std::uint64_t qt_key(const void* owner, int level, int x, int y) {
  return (static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(owner) >> 4) * 0x9E3779B97F4A7C15ull) ^
         (static_cast<std::uint64_t>(level & 0xFF) << 56) ^ (static_cast<std::uint64_t>(x & 0xFFFFFFF) << 28) ^
         static_cast<std::uint64_t>(y & 0xFFFFFFF);
}

// Once per presented frame: is the head turning (yaw changed > 0.15 deg since the previous frame)? g_qt_mutex held.
bool qt_turning_locked() {
  const std::uint64_t f = g_presents.load(std::memory_order_relaxed);
  if (f == g_qt_turn_frame) return g_qt_turning;
  g_qt_turn_frame = f;
  head_pose::Snapshot s;
  if (xr_runtime::latest_views(&s)) {
    const float fw[3] = {0.f, 0.f, -1.f};
    const float q[4] = {s.eye[0].qx, s.eye[0].qy, s.eye[0].qz, s.eye[0].qw};
    float d[3];
    quat_rot(q, fw, d);
    const float yaw = std::atan2(-d[0], -d[2]) * 57.2958f;
    float dy = g_qt_have_yaw ? yaw - g_qt_last_yaw : 0.f;
    while (dy > 180.f) dy -= 360.f;
    while (dy < -180.f) dy += 360.f;
    g_qt_turning = std::fabs(dy) > 0.15f;
    g_qt_last_yaw = yaw;
    g_qt_have_yaw = true;
  }
  (g_qt_turning ? g_qt_frames_turn : g_qt_frames_still)++;
  return g_qt_turning;
}

// Eye whose camera sits at the visit's LOD position: 0 left/main, 1 right/created, 2 shared midpoint, -1 other.
int qt_eye_for(void* lod_ctx) {
  void* p = nullptr;
  float pos[3];
  if (lod_ctx == nullptr || !read_ptr(lod_ctx, &p) || p == nullptr || !read_bytes(static_cast<unsigned char*>(p) + 0x30, pos, sizeof(pos))) {
    return -1;
  }
  float eye[2][3];
  void* views[2] = {g_created_main, g_created};
  for (int e = 0; e < 2; ++e) {
    unsigned char* frame = views[e] != nullptr ? lightq_frame(views[e]) : nullptr;
    std::int32_t slot = 0;
    if (frame == nullptr) return -1;
    read_bytes(frame + 0x1BFA0, &slot, sizeof(slot));
    if (slot < 0 || slot > 2) slot = 0;
    if (!read_bytes(frame + 0xEEF0 + static_cast<std::uintptr_t>(slot) * 0x4590 + 0x150, eye[e], sizeof(eye[e]))) return -1;
  }
  float d[2];
  for (int e = 0; e < 2; ++e) {
    const float x = pos[0] - eye[e][0], y = pos[1] - eye[e][1], z = pos[2] - eye[e][2];
    d[e] = std::sqrt(x * x + y * y + z * z);
  }
  if (std::fabs(d[0] - d[1]) < 0.002f && d[0] < 0.5f) return 2;
  if (d[0] < d[1] && d[0] < 0.01f) return 0;
  if (d[1] < d[0] && d[1] < 0.01f) return 1;
  return -1;
}

void qt_note_locked(int eye, std::uint64_t key, std::uint8_t state) {
  const std::uint64_t f = g_presents.load(std::memory_order_relaxed);
  QtTile& t = g_qt_tiles[key];
  const std::uint8_t old = t.state[eye];
  const bool recent = t.frame[eye] + 2 >= f && t.frame[eye] != f;
  if (recent && old != 2 && old != state) {
    const bool turning = qt_turning_locked();
    QtEye& e = g_qt_eye[eye];
    if (state == 0) (turning ? e.to_collapsed_turn : e.to_collapsed_still)++;
    else (turning ? e.to_visible_turn : e.to_visible_still)++;
  }
  t.state[eye] = state;
  t.frame[eye] = f;
}

void QtVisit_hook(void* owner, unsigned* xy, unsigned level, void* frustum, void* lod_ctx, void* x6) {
  if (!g_qt_probe.load(std::memory_order_relaxed) || !g_stereo_running.load(std::memory_order_relaxed)) {
    g_qt_visit(owner, xy, level, frustum, lod_ctx, x6);
    return;
  }
  const bool root = t_qt_depth == 0;
  if (root) t_qt_eye = qt_eye_for(lod_ctx);
  const int eye = t_qt_eye;
  unsigned c[2] = {0, 0};
  const bool have_xy = xy != nullptr && read_bytes(xy, c, sizeof(c));
  if (eye == 0 || eye == 1) {
    std::lock_guard<std::mutex> lock(g_qt_mutex);
    qt_turning_locked();
    g_qt_eye[eye].visits++;
    if (root) g_qt_eye[eye].roots++;
    if (have_xy) qt_note_locked(eye, qt_key(owner, static_cast<int>(level), static_cast<int>(c[0]), static_cast<int>(c[1])), 1);
  } else if (root) {
    std::lock_guard<std::mutex> lock(g_qt_mutex);
    (eye == 2 ? g_qt_shared : g_qt_other)++;
  }
  ++t_qt_depth;
  g_qt_visit(owner, xy, level, frustum, lod_ctx, x6);
  --t_qt_depth;
  if (root) t_qt_eye = -1;
}

void QtCollapse_hook(void* owner, int* xy, int level) {
  const int eye = t_qt_depth > 0 ? t_qt_eye : -1;
  if (eye == 0 || eye == 1) {
    int c[2] = {0, 0};
    if (xy != nullptr && read_bytes(xy, c, sizeof(c))) {
      std::lock_guard<std::mutex> lock(g_qt_mutex);
      g_qt_eye[eye].collapses++;
      qt_note_locked(eye, qt_key(owner, level, c[0], c[1]), 0);
    }
  }
  g_qt_collapse(owner, xy, level);
}

void log_qt_probe() {
  if (g_qt_visit == nullptr || !g_qt_probe.load()) return;
  std::lock_guard<std::mutex> lock(g_qt_mutex);
  const std::uint64_t f = g_presents.load(std::memory_order_relaxed);
  std::uint64_t both = 0, differ = 0;
  for (auto it = g_qt_tiles.begin(); it != g_qt_tiles.end();) {
    const QtTile& t = it->second;
    if (t.frame[0] + 600 < f && t.frame[1] + 600 < f) {  // not seen for ~10 s: drop
      it = g_qt_tiles.erase(it);
      continue;
    }
    if (t.state[0] != 2 && t.state[1] != 2 && t.frame[0] + 2 >= f && t.frame[1] + 2 >= f) {
      both++;
      if (t.state[0] != t.state[1]) differ++;
    }
    ++it;
  }
  for (int i = 0; i < 2; ++i) {
    const QtEye& e = g_qt_eye[i];
    log_line("qtprobe %s: roots=%llu visits=%llu collapses=%llu | to-collapsed still=%llu turning=%llu | to-visible still=%llu "
             "turning=%llu",
             i == 0 ? "main" : "second", static_cast<unsigned long long>(e.roots), static_cast<unsigned long long>(e.visits),
             static_cast<unsigned long long>(e.collapses), static_cast<unsigned long long>(e.to_collapsed_still),
             static_cast<unsigned long long>(e.to_collapsed_turn), static_cast<unsigned long long>(e.to_visible_still),
             static_cast<unsigned long long>(e.to_visible_turn));
  }
  log_line("qtprobe: frames still=%llu turning=%llu | tiles now both eyes=%llu differ=%llu | roots shared=%llu other=%llu tiles=%zu",
           static_cast<unsigned long long>(g_qt_frames_still), static_cast<unsigned long long>(g_qt_frames_turn),
           static_cast<unsigned long long>(both), static_cast<unsigned long long>(differ),
           static_cast<unsigned long long>(g_qt_shared), static_cast<unsigned long long>(g_qt_other), g_qt_tiles.size());
}

// ---- spot-shadow atlas slot probe ("spotprobe", ISSUE-04, 2026-10-21; read-only) ---------------------------------------
// [STATIC] Gfx_ViewGeometryRenderer::RenderSpotShadows 0x14058A6C0(renderer): view = [renderer+0x18], frame =
// 0x14054D710(view) (ring slot +0x9EC4 or +0x9EC8 by a thread flag, so it is called on the same thread), spot shadow count
// frame+0xCD28, entries frame+0xCD18 (stride 0x1140; each a shadow camera, atlas slot +0x8B0, skip flag +0x8B4). Matrices
// go into the constants BY SLOT and the atlas [0x1496A1B60] is shared by both eyes, while each eye assigns slots with its
// own sticky history (0x1405B0F30). [AUTHOR] some lamp shadows show in one eye only. The probe identifies each lamp by its
// shadow camera's view-projection (lamp-defined, so equal in both eyes) and compares the eyes' slot maps per frame.
constexpr std::uintptr_t kSpotShadowsRva = 0x58A6C0;
constexpr unsigned char kSpotShadowsSig[] = {0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56};
constexpr std::uintptr_t kViewFrameRva = 0x54D710;
constexpr unsigned char kViewFrameSig[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x65, 0x48, 0x8B, 0x04};
using SpotShadowsFn = void (*)(void*);
using ViewFrameFn = unsigned char* (*)(void*);
SpotShadowsFn g_spot_shadows = nullptr;
ViewFrameFn g_view_frame = nullptr;
std::atomic<bool> g_spot_probe{false};
struct SpotEntry {
  std::uint64_t id;
  std::uint32_t slot;
  std::uint8_t skip;
};
struct SpotEye {
  std::vector<SpotEntry> cur;
  std::uint64_t frame = 0;
  std::uint64_t calls = 0, entries = 0, skips = 0;
};
SpotEye g_spot[2];  // g_spot_mutex
std::mutex g_spot_mutex;
std::uint64_t g_spot_pairs = 0, g_spot_both_same = 0, g_spot_both_diffslot = 0, g_spot_only_one = 0, g_spot_slot_clash = 0,
              g_spot_skip_mismatch = 0;

std::uint64_t spot_id(const float* m) {
  std::uint64_t h = 1469598103934665603ull;
  for (int i = 0; i < 16; ++i) {
    const std::int64_t q = static_cast<std::int64_t>(std::llround(static_cast<double>(m[i]) * 1000.0));
    h = (h ^ static_cast<std::uint64_t>(q)) * 1099511628211ull;
  }
  return h;
}

void spot_compare_locked() {
  const SpotEye& a = g_spot[0];
  const SpotEye& b = g_spot[1];
  if (a.cur.empty() && b.cur.empty()) return;
  if (!(a.frame == b.frame || a.frame + 1 == b.frame || b.frame + 1 == a.frame)) return;
  g_spot_pairs++;
  for (const SpotEntry& ea : a.cur) {
    const SpotEntry* match = nullptr;
    for (const SpotEntry& eb : b.cur) {
      if (eb.id == ea.id) {
        match = &eb;
        break;
      }
    }
    if (match == nullptr) {
      g_spot_only_one++;
      continue;
    }
    (match->slot == ea.slot ? g_spot_both_same : g_spot_both_diffslot)++;
    if (match->skip != ea.skip) g_spot_skip_mismatch++;
  }
  for (const SpotEntry& eb : b.cur) {
    bool found = false;
    for (const SpotEntry& ea : a.cur) found = found || ea.id == eb.id;
    if (!found) g_spot_only_one++;
    for (const SpotEntry& ea : a.cur) {
      if (ea.slot == eb.slot && ea.id != eb.id) g_spot_slot_clash++;  // same atlas slot, different lamp
    }
  }
}

unsigned char* spot_view_frame(void* view) {  // engine accessor on this (render) thread; SEH-isolated
  __try {
    return g_view_frame(view);
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return nullptr;
  }
}

void SpotShadows_hook(void* renderer) {
  g_spot_shadows(renderer);
  if (!g_spot_probe.load(std::memory_order_relaxed) || renderer == nullptr || g_view_frame == nullptr) return;
  void* view = nullptr;
  if (!read_ptr(static_cast<unsigned char*>(renderer) + 0x18, &view)) return;
  const int vi = shadow_view_index(view);
  if (vi < 0) return;
  unsigned char* frame = spot_view_frame(view);
  std::uint32_t count = 0;
  void* entries = nullptr;
  if (frame == nullptr || !read_u32(frame + 0xCD28, &count) || count > 64 || !read_ptr(frame + 0xCD18, &entries) ||
      (count > 0 && entries == nullptr)) {
    return;
  }
  std::vector<SpotEntry> list;
  list.reserve(count);
  std::uint64_t skips = 0;
  for (std::uint32_t i = 0; i < count; ++i) {
    unsigned char* e = static_cast<unsigned char*>(entries) + static_cast<std::uintptr_t>(i) * 0x1140;
    float vp[16];
    std::uint32_t slot = 0;
    std::uint8_t skip = 0;
    if (!read_bytes(e + 0x3A0 + 0xC0, vp, sizeof(vp)) || !read_u32(e + 0x8B0, &slot) || !read_u8(e + 0x8B4, &skip)) continue;
    list.push_back({spot_id(vp), slot, skip});
    skips += skip != 0;
  }
  std::lock_guard<std::mutex> lock(g_spot_mutex);
  SpotEye& s = g_spot[vi];
  s.cur.swap(list);
  s.frame = g_presents.load(std::memory_order_relaxed);
  s.calls++;
  s.entries += s.cur.size();
  s.skips += skips;
  if (vi == 1) spot_compare_locked();
}

void log_spot_probe() {
  if (g_spot_shadows == nullptr || !g_spot_probe.load()) return;
  std::lock_guard<std::mutex> lock(g_spot_mutex);
  for (int i = 0; i < 2; ++i) {
    const SpotEye& s = g_spot[i];
    log_line("spotprobe %s: calls=%llu shadows/call=%.2f skips/call=%.2f", i == 0 ? "main" : "second",
             static_cast<unsigned long long>(s.calls), s.calls ? static_cast<double>(s.entries) / s.calls : 0.0,
             s.calls ? static_cast<double>(s.skips) / s.calls : 0.0);
  }
  log_line("spotprobe eyes: pairs=%llu lamp in both same slot=%llu DIFFERENT slot=%llu | lamp in one eye only=%llu | "
           "slot holding different lamps=%llu | skip flag differs=%llu",
           static_cast<unsigned long long>(g_spot_pairs), static_cast<unsigned long long>(g_spot_both_same),
           static_cast<unsigned long long>(g_spot_both_diffslot), static_cast<unsigned long long>(g_spot_only_one),
           static_cast<unsigned long long>(g_spot_slot_clash), static_cast<unsigned long long>(g_spot_skip_mismatch));
}

// ---- frame-rate cap diagnosis (performance pass 2026-10-22; always on, ~1 line / 2 s) ---------------------------------
// [AUTHOR] outdoors ~30 fps; [LIVE 14:02-14:07] a flat 29.5-30 fps during play that rose to 66-75 with the mod menu open.
// A flat 30 suggests a cap (e.g. the driver's background-application frame limit when the game is not the foreground
// window, or a 90 Hz headset at every third refresh), not GPU load. Logs fps, whether the game window is the Windows
// foreground window (else which process is) and the headset display period.
void log_frame_status() {
  static ULONGLONG last_t = 0;
  static std::uint64_t last_p = 0;
  const ULONGLONG now = GetTickCount64();
  const std::uint64_t p = g_presents.load(std::memory_order_relaxed);
  const double fps = last_t != 0 && now > last_t ? static_cast<double>(p - last_p) * 1000.0 / static_cast<double>(now - last_t) : 0.0;
  last_t = now;
  last_p = p;
  HWND fg = GetForegroundWindow();
  HWND game = static_cast<HWND>(dx12::game_window());
  char who[96] = "?";
  if (fg != nullptr && fg != game) {
    DWORD pid = 0;
    GetWindowThreadProcessId(fg, &pid);
    if (HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid)) {
      wchar_t path[MAX_PATH] = {};
      DWORD n = MAX_PATH;
      if (QueryFullProcessImageNameW(h, 0, path, &n)) {
        const wchar_t* base = wcsrchr(path, L'\\');
        WideCharToMultiByte(CP_UTF8, 0, base != nullptr ? base + 1 : path, -1, who, sizeof(who), nullptr, nullptr);
      }
      CloseHandle(h);
    }
  }
  const double period = xr_runtime::display_period_ms();
  log_line("frame: fps=%.1f stereo=%d foreground=%s headset period=%.2f ms (%.0f Hz)", fps, g_stereo_running.load() ? 1 : 0,
           fg == nullptr ? "none" : fg == game ? "GAME" : who, period, period > 0.0 ? 1000.0 / period : 0.0);
}

bool hook_checked(unsigned char* base, std::uintptr_t rva, const unsigned char* sig, size_t sig_size, void* detour, void** original, const char* what) {
  unsigned char* target = base + rva;
  unsigned char got[64] = {};
  if (sig_size > sizeof(got) || !read_bytes(target, got, sig_size)) {
    log_line("%s: unreadable at +0x%llX", what, static_cast<unsigned long long>(rva));
    return false;
  }
  if (std::memcmp(got, sig, sig_size) != 0) {
    log_line("%s: signature mismatch at +0x%llX; not hooked", what, static_cast<unsigned long long>(rva));
    return false;
  }
  MH_STATUS s = MH_CreateHook(target, detour, original);
  if (s == MH_OK) {
    s = MH_EnableHook(target);
  }
  log_line("%s: hook at +0x%llX status=%d", what, static_cast<unsigned long long>(rva), static_cast<int>(s));
  return s == MH_OK;
}

}  // namespace

namespace native_stereo {

void set_render_size(unsigned width, unsigned height) {
  {  // eye mode: the views render at the eye size (gameplay) or the client size (menus), not the grown buffer size
    std::uint32_t ew = 0, eh = 0;
    unsigned cw = 0, ch = 0;
    if (eye_mode_on() && eyesize_for(false, &ew, &eh)) {
      width = ew;
      height = eh;
    } else if (dx12::client_size(&cw, &ch)) {
      width = cw;
      height = ch;
    }
  }
  g_render_w.store(width, std::memory_order_relaxed);
  g_render_h.store(height, std::memory_order_relaxed);
  // [LIVE] capture 2026-10-08 06:37: camera.cpp only learned the size at swapchain creation (1992x1421, before the VR
  // shape resize to 1992x2160), so the second view's UI got a viewport 1421 high and came out squashed to 0.66. Fed
  // here every Present (vr_menu), after resizes too.
  if (width > 0 && height > 0) {
    camera::note_backbuffer(width, height);
  }
  const float a = width > 0 && height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 0.f;
  if (std::fabs(a - g_render_aspect.load(std::memory_order_relaxed)) > 1e-4f) {
    g_render_aspect.store(a, std::memory_order_relaxed);
    log_line("render size %ux%u: eye cameras use aspect %.3f", width, height, a);
  }
}

bool aim_ray(int hand, float* org, float* dir) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (hand < 0 || hand > 1 || !g_aim.valid[hand] || GetTickCount64() - g_aim.sampled_at > 250) {
    return false;
  }
  std::memcpy(org, g_aim.org[hand], sizeof(g_aim.org[hand]));
  std::memcpy(dir, g_aim.dir[hand], sizeof(g_aim.dir[hand]));
  return true;
}

bool grip_transform(int hand, float* matrix16) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (hand < 0 || hand > 1 || matrix16 == nullptr || !g_aim.grip_valid[hand] ||
      GetTickCount64() - g_aim.sampled_at > 250) return false;
  std::memcpy(matrix16, g_aim.grip[hand], sizeof(g_aim.grip[hand]));
  return true;
}

bool grip_yaw(float* yaw) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (yaw == nullptr || !g_aim.yaw_valid || GetTickCount64() - g_aim.sampled_at > 250) return false;
  *yaw = g_aim.yaw;
  return true;
}

bool grip_frame(GripFrame* out) {
  if(!out) return false;
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  *out=GripFrame{};
  if(!g_aim.head_valid || !g_aim.body_valid || GetTickCount64()-g_aim.sampled_at>250) return false;
  std::memcpy(out->camera,g_aim.cam_mat,sizeof(out->camera));
  std::memcpy(out->body,g_aim.body,sizeof(out->body));
  for(int h=0;h<2;++h) {
    out->valid[h]=g_aim.grip_valid[h];
    std::memcpy(out->grip[h],g_aim.ride_grip[h],sizeof(out->grip[h]));
  }
  out->sequence=g_aim.frames;
  return true;
}

bool grip_camera(float* matrix16) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (matrix16 == nullptr || !g_aim.head_valid || GetTickCount64() - g_aim.sampled_at > 250) return false;
  std::memcpy(matrix16, g_aim.cam_mat, sizeof(g_aim.cam_mat));
  return true;
}

bool grip_body(float* body3) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (body3 == nullptr || !g_aim.body_valid || GetTickCount64() - g_aim.sampled_at > 250) return false;
  std::memcpy(body3, g_aim.body, sizeof(g_aim.body));
  return true;
}

bool head_position(float* pos3) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (pos3 == nullptr || !g_aim.head_valid || GetTickCount64() - g_aim.sampled_at > 250) return false;
  for (int k = 0; k < 3; ++k) pos3[k] = g_aim.cam_pos[k] + g_aim.head_offset[k];
  return true;
}

bool head_offset(float* offset3) {
  std::lock_guard<std::mutex> lock(g_aim_mutex);
  if (offset3 == nullptr || !g_aim.head_valid || GetTickCount64() - g_aim.sampled_at > 250) return false;
  std::memcpy(offset3, g_aim.head_offset, sizeof(g_aim.head_offset));
  return true;
}

void rebase_head(const float* world_delta3) {
  if (world_delta3 == nullptr || !std::isfinite(world_delta3[0]) || !std::isfinite(world_delta3[1]) ||
      !std::isfinite(world_delta3[2])) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_rebase_mutex);
  for (int k = 0; k < 3; ++k) g_rebase[k] += world_delta3[k];
}

void pending_rebase(float* world_delta3) {
  std::lock_guard<std::mutex> lock(g_rebase_mutex);
  std::memcpy(world_delta3, g_rebase, sizeof(g_rebase));
}

std::uint64_t rebases_applied() { return g_rebase_applied.load(std::memory_order_relaxed); }

void rebase_yaw(float engine_yaw) {
  if (!std::isfinite(engine_yaw)) return;
  std::lock_guard<std::mutex> lock(g_rebase_mutex);
  g_rebase_yaw += engine_yaw;
}

bool head_yaw_relative(float* yaw, std::uint64_t* seq) {
  if (seq != nullptr) *seq = g_head_yaw_seq.load(std::memory_order_relaxed);
  if (yaw == nullptr || GetTickCount64() - g_head_yaw_tick.load(std::memory_order_relaxed) > 250) return false;
  float pending;
  {
    std::lock_guard<std::mutex> lock(g_rebase_mutex);
    pending = g_rebase_yaw;
  }
  *yaw = g_head_yaw_rel.load(std::memory_order_relaxed) - pending;  // a queued turn already lowers it
  return true;
}

void set_body_yaw(float yaw) {
  if (!std::isfinite(yaw)) return;
  std::lock_guard<std::mutex> lock(g_body_yaw_mutex);
  g_body_yaw = yaw;
  g_body_yaw_until = GetTickCount64() + 250;
}

void clear_body_yaw() {
  std::lock_guard<std::mutex> lock(g_body_yaw_mutex);
  g_body_yaw_until = 0;
}

void eye_in_camera(const head_pose::Eye& eye, float* m9, float* off3) { eye_transform(eye, m9, off3); }
bool frame_camera(int source, float* cam16, std::uint64_t* id) {
  std::uint32_t slot = 0xFFFFFFFF;
  if (source == 1 && g_created_main != nullptr) read_u32(static_cast<unsigned char*>(g_created_main) + kFrameIdxGame + 4, &slot);
  std::lock_guard<std::mutex> lock(g_applied_mutex);
  std::uint64_t pick = slot < 8 ? g_slot_pose[0][slot] : 0;
  if (pick == 0 || pick > g_applied_count || g_applied_count - pick >= static_cast<std::uint64_t>(kPoseRing)) pick = g_applied_count;
  if (pick == 0) return false;
  std::memcpy(cam16, g_applied_cam[(pick - 1) % kPoseRing], 16 * sizeof(float));
  *id = pick;
  return true;
}
bool applied_pose(std::uint64_t id, head_pose::Snapshot* out) {
  std::lock_guard<std::mutex> lock(g_applied_mutex);
  if (id == 0 || id > g_applied_count || g_applied_count - id >= static_cast<std::uint64_t>(kPoseRing)) return false;
  *out = g_applied[(id - 1) % kPoseRing];
  return true;
}
bool views_for_present(int lag, head_pose::Snapshot* out) {
  std::lock_guard<std::mutex> lock(g_applied_mutex);
  if (g_applied_count == 0 || !g_head.load()) {
    return false;
  }
  const std::uint64_t back = static_cast<std::uint64_t>(lag < 0 ? 0 : lag);
  const std::uint64_t newest = g_applied_count - 1;
  const std::uint64_t idx = back > newest ? 0 : newest - back;
  if (g_applied_count - idx > static_cast<std::uint64_t>(kPoseRing)) {
    return false;
  }
  *out = g_applied[idx % kPoseRing];
  return true;
}

bool presented_views(head_pose::Snapshot* out) {
  // Right eye: the pose of the frame the newest view-2 output belongs to. Left eye (backbuffer): the pose of the frame
  // view #0 rendered last, i.e. its +0x9EC8 slot at Present time. The two can be one frame apart; each eye is labelled
  // with the pose it was actually rendered from, so the compositor's reprojection lines both up.
  const std::uint64_t right = g_out_pose.load(std::memory_order_relaxed);
  std::uint32_t slot0 = 0xFFFFFFFF;
  if (g_created_main != nullptr) {
    read_u32(static_cast<unsigned char*>(g_created_main) + kFrameIdxGame + 4, &slot0);
  }
  std::lock_guard<std::mutex> lock(g_applied_mutex);
  const std::uint64_t left = slot0 < 8 ? g_slot_pose[0][slot0] : 0;
  auto ok = [](std::uint64_t id) {
    return id != 0 && g_applied_count >= id && g_applied_count - id < static_cast<std::uint64_t>(kPoseRing);
  };
  if (!g_head.load() || !ok(right) || !ok(left)) {
    g_identity_missing.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  *out = g_applied[(right - 1) % kPoseRing];
  out->eye[0] = g_applied[(left - 1) % kPoseRing].eye[0];
  g_identity_used.fetch_add(1, std::memory_order_relaxed);
  if (left != right) {
    g_identity_split.fetch_add(1, std::memory_order_relaxed);
  }
  return true;
}

bool game_frustum(float* tan4) {
  const float m0 = g_proj_m0.load(), m5 = g_proj_m5.load(), m8 = g_proj_m8.load(), m9 = g_proj_m9.load();
  if (m0 <= 0.01f || m5 <= 0.01f) {
    return false;
  }
  // Row-vector projection, w = z: ndc.x = (x/z)*m0 + m8, ndc.y = (y/z)*m5 + m9.
  tan4[0] = (-1.f - m8) / m0;  // left (negative)
  tan4[1] = (1.f - m8) / m0;   // right
  tan4[2] = (1.f - m9) / m5;   // up
  tan4[3] = (-1.f - m9) / m5;  // down (negative)
  return true;
}

ID3D12Resource* acquire_main_output() {
  std::lock_guard<std::mutex> lock(g_out_mutex);
  ID3D12Resource* r = g_main_held;  // ownership moves to the caller; nothing is kept between captures
  g_main_held = nullptr;
  g_main_wanted.store(false);
  return r;
}

void want_main_output() { g_main_wanted.store(true); }

void set_share_override(int pick) { g_share_override.store(pick); }
void set_field_variant(int variant) { g_field_variant.store(variant >= 0 && variant <= kFieldVariants ? variant : 0); }
int field_variant_count() { return kFieldVariants; }
void force_zero_sep(bool on) { g_zero_sep_forced.store(on); }


int share_slot_count() { return g_share_count.load(); }
int share_slot_offset(int pick) {
  return pick >= 1 && pick <= g_share_count.load() ? static_cast<int>(g_share_off[pick - 1]) : -1;
}

void hold_for_resize(unsigned ms) { g_resize_hold_until.store(GetTickCount64() + ms); }

bool stereo_running() { return g_stereo_running.load(std::memory_order_relaxed); }
bool stereo_pending() { return g_stereo_pending.load(std::memory_order_relaxed); }
bool stereo_paused_by_menu() { return g_last_pause_menu.load(std::memory_order_relaxed); }

bool stereo_viewport_ids(unsigned* main_id, unsigned* second_id) {
  // [LIVE] 2026-10-09 06:15 / 06:16: with the motion-vector repair on from launch it ran in the 3D main menu (front end,
  // stereo running there too) and the game then hit a GPU fault ("GPU Crash Reporter", afop+0x162B730) while loading.
  // Gameplay only: not the front end.
  // [LIVE] 2026-10-09 06:29:00: GPU fault in the first second after stereo resumed from a loading screen (new second
  // view). Wait 1 s after every (re)start so the game has re-tagged its rebuilt buffers.
  if (!g_stereo_running.load(std::memory_order_relaxed) || g_front_end.load(std::memory_order_relaxed) || g_created == nullptr ||
      g_created_main == nullptr || GetTickCount64() - g_stereo_started_at.load(std::memory_order_relaxed) < 1000) {
    return false;
  }
  std::uint32_t m = 0, s = 0;
  if (!read_u32(static_cast<unsigned char*>(g_created_main) + 0x20, &m) || !read_u32(static_cast<unsigned char*>(g_created) + 0x20, &s) ||
      m == s) {
    return false;
  }
  *main_id = m;
  *second_id = s;
  return true;
}

unsigned rendering_kinds() {
  unsigned mask = 0;
  for (int k = 0; k < 3; ++k) {
    if (g_rendering[k].load(std::memory_order_relaxed) > 0) {
      mask |= 1u << k;
    }
  }
  return mask;
}

void request_rebuild(const char* why) {
  log_line("rebuild requested: %s", why != nullptr ? why : "?");
  g_rebuild_reason = "lighting (dark right eye watchdog)";
  g_rebuild_requested.store(true);
}

bool eye_size(std::uint32_t* w, std::uint32_t* h) {
  return eye_mode_on() && eyesize_for(false, w, h);
}

ID3D12Resource* acquire_main_eye_output() {
  if (!eye_mode_on()) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lock(g_out_mutex);
  if (g_main_eye_held != nullptr) {
    g_main_eye_held->AddRef();
  }
  return g_main_eye_held;
}

ID3D12Resource* acquire_view2_output() {
  if (!g_stereo_running.load(std::memory_order_relaxed)) {
    return nullptr;  // paused (menus / start-up): mono
  }
  std::lock_guard<std::mutex> lock(g_out_mutex);
  if (g_out_held != nullptr) {
    g_out_held->AddRef();
  }
  return g_out_held;
}

void install() {
  if (g_installed.exchange(true)) {
    return;
  }
  const std::wstring dir = self_dir();
  if (!dir.empty()) {
    g_control_path = dir + L"\\AvatarVR_native.txt";
    g_log_path = dir + L"\\NATIVE_STEREO.log";
    g_dump_path = dir + L"\\CODEDUMP.log";
    g_dump_request = dir + L"\\AvatarVR_dump.txt";
    MoveFileExW(g_log_path.c_str(), (dir + L"\\NATIVE_STEREO.prev.log").c_str(), MOVEFILE_REPLACE_EXISTING);
  }
  log_line("native stereo probe build %s %s pid=%lu control=AvatarVR_native.txt (off|observe|mode N [view K] [kick]|second K SEP [world [src S]]|create [SEP] [offscreen])", __DATE__, __TIME__, GetCurrentProcessId());
  auto* base = reinterpret_cast<unsigned char*>(GetModuleHandleW(nullptr));
  if (base == nullptr) {
    return;
  }
  g_module_base = base;
  {
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    g_image_size = nt->OptionalHeader.SizeOfImage;
  }
  hook_checked(base, kRenderRva, kRenderSig, sizeof(kRenderSig), reinterpret_cast<void*>(&Render_hook), reinterpret_cast<void**>(&g_render), "Gfx_View::Render");
  hook_checked(base, kCaptureRva, kCaptureSig, sizeof(kCaptureSig), reinterpret_cast<void*>(&Capture_hook), reinterpret_cast<void**>(&g_capture), "eye capture task");
  hook_checked(base, kStereoRva, kStereoSig, sizeof(kStereoSig), reinterpret_cast<void*>(&Stereo_hook), reinterpret_cast<void**>(&g_stereo), "stereo render path");
  hook_checked(base, kWaitRva, kWaitSig, sizeof(kWaitSig), reinterpret_cast<void*>(&Wait_hook), reinterpret_cast<void**>(&g_wait), "Gfx_View task wait");
  hook_checked(base, kCompositeRva, kCompositeSig, sizeof(kCompositeSig), reinterpret_cast<void*>(&Composite_hook), reinterpret_cast<void**>(&g_composite), "stereo composite");
  hook_checked(base, kAddTaskRva, kAddTaskSig, sizeof(kAddTaskSig), reinterpret_cast<void*>(&AddTask_hook), reinterpret_cast<void**>(&g_add_task), "add view task");
  hook_checked(base, kGateRva, kGateSig, sizeof(kGateSig), reinterpret_cast<void*>(&Gate_hook), reinterpret_cast<void**>(&g_gate), "phase gate task");
  // Particle visibility bit for the second view (fxbits, on by default; see ParticleCull_hook).
  hook_checked(base, kParticleCullRva, kParticleCullSig, sizeof(kParticleCullSig), reinterpret_cast<void*>(&ParticleCull_hook),
               reinterpret_cast<void**>(&g_particle_cull), "particle cull job (fxbits)");
  {  // per-view sun-shadow slot probe / shadowfix: needs the engine's release function first
    unsigned char got[sizeof(kRefReleaseSig)] = {};
    if (read_bytes(base + kRefReleaseRva, got, sizeof(got)) && std::memcmp(got, kRefReleaseSig, sizeof(got)) == 0) {
      g_ref_release = reinterpret_cast<RefReleaseFn>(base + kRefReleaseRva);
      g_targets_global_ptr = base + kTargetsGlobalRva;
      hook_checked(base, kShadowWriteRva, kShadowWriteSig, sizeof(kShadowWriteSig), reinterpret_cast<void*>(&ShadowWrite_hook),
                   reinterpret_cast<void**>(&g_shadow_write), "shadow slot writer (shadowslot)");
      hook_checked(base, kShadowReadRva, kShadowReadSig, sizeof(kShadowReadSig), reinterpret_cast<void*>(&ShadowRead_hook),
                   reinterpret_cast<void**>(&g_shadow_read), "shadow slot reader (shadowslot)");
      hook_checked(base, kLightJobRva, kLightJobSig, sizeof(kLightJobSig), reinterpret_cast<void*>(&LightJob_hook),
                   reinterpret_cast<void**>(&g_light_job), "light queue job (lightq probe)");
      hook_checked(base, kShadowPickRva, kShadowPickSig, sizeof(kShadowPickSig), reinterpret_cast<void*>(&ShadowPick_hook),
                   reinterpret_cast<void**>(&g_shadow_pick), "spot-shadow light choice (shadowunion)");
      hook_checked(base, kCamUpdateRva, kCamUpdateSig, sizeof(kCamUpdateSig), reinterpret_cast<void*>(&CamUpdate_hook),
                   reinterpret_cast<void**>(&g_cam_update), "camera update, LOD focal (lodfov)");
      hook_checked(base, kLodScaleRva, kLodScaleSig, sizeof(kLodScaleSig), reinterpret_cast<void*>(&LodScale_hook),
                   reinterpret_cast<void**>(&g_lod_scale), "mesh LOD scale (lodkeep 1)");
      g_game_base = reinterpret_cast<std::uintptr_t>(base);
      hook_checked(base, kCullViewRva, kCullViewSig, sizeof(kCullViewSig), reinterpret_cast<void*>(&CullView_hook),
                   reinterpret_cast<void**>(&g_cull_view), "cull view setup (lodkeep 2)");
      hook_checked(base, kStreamMipRva, kStreamMipSig, sizeof(kStreamMipSig), reinterpret_cast<void*>(&StreamMip_hook),
                   reinterpret_cast<void**>(&g_stream_mip), "texture streaming mip (lodkeep 3)");
      hook_checked(base, kStreamMip2Rva, kStreamMip2Sig, sizeof(kStreamMip2Sig), reinterpret_cast<void*>(&StreamMip2_hook),
                   reinterpret_cast<void**>(&g_stream_mip2), "texture streaming mip 2 (lodkeep 3)");
      hook_checked(base, kFocalUser4Rva, kFocalUser4Sig, sizeof(kFocalUser4Sig), reinterpret_cast<void*>(&FocalUser4_hook),
                   reinterpret_cast<void**>(&g_focal_user4), "focal consumer 0x37E9D0 (lodkeep 4)");
      hook_checked(base, kNearFarRva, kNearFarSig, sizeof(kNearFarSig), reinterpret_cast<void*>(&NearFar_hook),
                   reinterpret_cast<void**>(&g_near_far), "LOD near/far decision (lodprobe)");
      hook_checked(base, kQtVisitRva, kQtVisitSig, sizeof(kQtVisitSig), reinterpret_cast<void*>(&QtVisit_hook),
                   reinterpret_cast<void**>(&g_qt_visit), "quadtree tile visit (qtprobe)");
      hook_checked(base, kQtCollapseRva, kQtCollapseSig, sizeof(kQtCollapseSig), reinterpret_cast<void*>(&QtCollapse_hook),
                   reinterpret_cast<void**>(&g_qt_collapse), "quadtree tile collapse (qtprobe)");
      {
        unsigned char got[sizeof(kViewFrameSig)] = {};
        if (read_bytes(base + kViewFrameRva, got, sizeof(got)) && std::memcmp(got, kViewFrameSig, sizeof(got)) == 0) {
          g_view_frame = reinterpret_cast<ViewFrameFn>(base + kViewFrameRva);
          hook_checked(base, kSpotShadowsRva, kSpotShadowsSig, sizeof(kSpotShadowsSig), reinterpret_cast<void*>(&SpotShadows_hook),
                       reinterpret_cast<void**>(&g_spot_shadows), "RenderSpotShadows (spotprobe)");
        } else {
          log_line("spotprobe: view frame accessor signature mismatch; not installed");
        }
      }
    } else {
      log_line("shadow slot: release function signature mismatch; probe not installed");
    }
  }
  // Removed 2026-10-09 (see research/DEBUG_TESTERS.md): RenderColor / fxcopy, particle draw worker and particle queue probes,
  // facial animation line id and cloth mapping probes.
  hook_checked(base, kSubmitRva, kSubmitSig, sizeof(kSubmitSig), reinterpret_cast<void*>(&Submit_hook), reinterpret_cast<void**>(&g_submit), "gate submitter");
  bool ok = true;
  for (Helper& h : g_helpers) {
    unsigned char got[10] = {};
    if (!read_bytes(base + h.rva, got, sizeof(got)) || std::memcmp(got, h.sig, sizeof(got)) != 0) {
      log_line("%s: signature mismatch at +0x%llX", h.what, static_cast<unsigned long long>(h.rva));
      ok = false;
    } else {
      h.fn = base + h.rva;
    }
  }
  g_helpers_ok = ok;
  log_line("camera helpers %s", ok ? "verified" : "NOT verified (second-view shift disabled)");
  {
    unsigned char got[12] = {};
    const bool a = read_bytes(base + kAllocRva, got, sizeof(kAllocSig)) && std::memcmp(got, kAllocSig, sizeof(kAllocSig)) == 0;
    const bool c = read_bytes(base + kCtorRva, got, sizeof(kCtorSig)) && std::memcmp(got, kCtorSig, sizeof(kCtorSig)) == 0;
    const bool v = read_bytes(base + kAddViewRva, got, sizeof(kAddViewSig)) && std::memcmp(got, kAddViewSig, sizeof(kAddViewSig)) == 0;
    if (a && c && v) {
      g_alloc = reinterpret_cast<AllocFn>(base + kAllocRva);
      g_ctor = reinterpret_cast<CtorFn>(base + kCtorRva);
      g_add_view = reinterpret_cast<Fn2>(base + kAddViewRva);
    }
    log_line("view creation functions %s (alloc %d ctor %d addview %d)", (a && c && v) ? "verified" : "NOT verified", a, c, v);
    hook_checked(base, kStartFrameRva, kStartFrameSig, sizeof(kStartFrameSig), reinterpret_cast<void*>(&StartFrame_hook),
                 reinterpret_cast<void**>(&g_start_frame), "Gfx_ViewManager::StartFrame");
    hook_checked(base, kOutputCbRva, kOutputCbSig, sizeof(kOutputCbSig), reinterpret_cast<void*>(&OutputCb_hook),
                 reinterpret_cast<void**>(&g_output_cb), "view output callback");
  }
  hook_checked(base, kCamBuildRva, kCamBuildSig, sizeof(kCamBuildSig), reinterpret_cast<void*>(&CamBuild_hook), reinterpret_cast<void**>(&g_cam_build), "per-eye camera build");
  hook_checked(base, kUpdateViewRva, kUpdateViewSig, sizeof(kUpdateViewSig), reinterpret_cast<void*>(&UpdateView_hook),
               reinterpret_cast<void**>(&g_update_view), "Gfx_View update (render size)");
  hook_checked(base, kOutSizeRva, kOutSizeSig, sizeof(kOutSizeSig), reinterpret_cast<void*>(&OutSize_hook),
               reinterpret_cast<void**>(&g_out_size), "output size getter (render size)");
  hook_checked(base, kUiPassRva, kUiPassSig, sizeof(kUiPassSig), reinterpret_cast<void*>(&UiPass_hook), reinterpret_cast<void**>(&g_ui_pass), "UI pass (probe)");
  ui_cursor::install(base, &log_rect_line);
  ui_piece::install(base, &log_rect_line);
  ui_layout::install(base, &log_rect_line);
  hook_checked(base, kUiExtra7Rva, kUiExtra7Sig, sizeof(kUiExtra7Sig), reinterpret_cast<void*>(&UiExtra7_hook), reinterpret_cast<void**>(&g_ui_extra7), "UI neighbour pass 0x55EE10");
  hook_checked(base, kUiExtra9Rva, kUiExtra9Sig, sizeof(kUiExtra9Sig), reinterpret_cast<void*>(&UiExtra9_hook), reinterpret_cast<void**>(&g_ui_extra9), "UI neighbour pass 0x55F150");
  {
    // Pass-name census: the prologue is not in the code dumps yet, so it is read here and the hook is only placed on a
    // standard MSVC x64 prologue (logged either way).
    unsigned char got[16] = {};
    read_bytes(base + kPassNameRva, got, sizeof(got));
    char hex[64] = {};
    for (int i = 0; i < 16; ++i) snprintf(hex + i * 3, 4, "%02X ", got[i]);
    const bool standard = (got[0] == 0x48 && got[1] == 0x89 && (got[2] == 0x5C || got[2] == 0x54 || got[2] == 0x4C) && got[3] == 0x24) ||
                          (got[0] == 0x48 && got[1] == 0x8B && got[2] == 0xC4) || (got[0] == 0x40 && (got[1] >= 0x53 && got[1] <= 0x57)) ||
                          (got[0] == 0x48 && got[1] == 0x83 && got[2] == 0xEC) || (got[0] == 0x4C && got[1] == 0x89 && got[3] == 0x24) ||
                          // Live 2026-10-06: "ret 0" followed by int3 padding, i.e. the profiling marker is an empty stub in the
                          // retail build (probably shared by other empty functions). The 5-byte patch fits into the padding.
                          (got[0] == 0xC2 && got[1] == 0x00 && got[2] == 0x00 && got[3] == 0xCC && got[4] == 0xCC) ||
                          (got[0] == 0xC3 && got[1] == 0xCC && got[2] == 0xCC && got[3] == 0xCC && got[4] == 0xCC);
    if (standard) {
      hook_checked(base, kPassNameRva, got, sizeof(got), reinterpret_cast<void*>(&PassName_hook), reinterpret_cast<void**>(&g_pass_name),
                   "pass name census");
    }
    log_line("pass name function +0x%llX prologue %s-> %s", static_cast<unsigned long long>(kPassNameRva), hex,
             standard ? "hooked" : "not a standard prologue, not hooked");
  }
}

// Per-eye camera constant blocks (1536 bytes), told apart by the sign of the projection centre (proj[8]): with per-eye
// centering off and a UI distance set, the eyes' centres are turned inward in opposite directions. Comparing the two
// blocks shows which constants differ between the eyes besides the camera itself, e.g. an animation time.
// (The buffers are declared with the jitter sync, which reads them.)

void note_eye_cb(const unsigned char* b, float m8) {
  if (std::fabs(m8) < 1e-4f) {
    return;
  }
  const int e = m8 > 0.f ? 0 : 1;
  if (dlss_twin::jit_logging()) {
    float p9 = 0.f;
    read_bytes(b + 0xA4, &p9, 4);
    LARGE_INTEGER q;
    QueryPerformanceCounter(&q);
    log_line("jitcb %lld eye%d p8=%.7f p9=%.7f", static_cast<long long>(q.QuadPart), e, m8, p9);
  }
  std::lock_guard<std::mutex> lock(g_eye_cb_mutex);
  if (read_bytes(b, g_eye_cb[e], sizeof(g_eye_cb[e]))) {
    g_eye_cb_at[e].store(GetTickCount64());
  }
}

// Present thread, every 20 s: list the floats that differ between the two eyes' blocks.
void log_eye_cb_diff() {
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last < 20000) {
    return;
  }
  const ULONGLONG a = g_eye_cb_at[0].load(), b = g_eye_cb_at[1].load();
  if (a == 0 || b == 0 || now - a > 2000 || now - b > 2000) {
    return;
  }
  last = now;
  unsigned char c[2][1536];
  {
    std::lock_guard<std::mutex> lock(g_eye_cb_mutex);
    std::memcpy(c, g_eye_cb, sizeof(c));
  }
  log_line("eyecb: camera constant block, floats that differ between the eyes (m8>0 | m8<0), %lld ms apart:",
           static_cast<long long>(a) - static_cast<long long>(b));
  char line[256];
  int n = 0, shown = 0, total = 0;
  for (int o = 0; o < 1536; o += 4) {
    float x, y;
    std::memcpy(&x, c[0] + o, 4);
    std::memcpy(&y, c[1] + o, 4);
    if (x == y || !(std::isfinite(x) && std::isfinite(y))) {
      continue;
    }
    ++total;
    if (shown >= 96) {
      continue;
    }
    n += snprintf(line + n, sizeof(line) - n, " +%03X %.5g|%.5g", o, x, y);
    ++shown;
    if (shown % 6 == 0) {
      log_line("eyecb:%s", line);
      n = 0;
    }
  }
  if (n > 0) {
    log_line("eyecb:%s", line);
  }
  log_line("eyecb: %d differing floats", total);
}

// ---- plant constants probe (Mod -> "Plant constants probe") -----------------------------------------------------
// [LIVE] 2026-10-06 09:24 foliage sweep: sharing no main-view pass slot changes the plant mismatch (best 5.17 vs 5.73),
// so the per-view plant state is not a pass slot. This probe looks at the per-draw constant blocks instead: both eyes
// rendered from one point with one frustum (and the jitter synced), the CBV observer at full rate for a short window,
// and every block of 64..8192 bytes recorded with the Present count it was created in. Blocks of the same size from
// the same frame that differ in only 1..4 floats are the same draw seen by the two views (or two instances of one mesh);
// an offset that is the only difference in many such pairs is a per-view constant, e.g. a wind time. Logged as
// "cbprobe:" lines in NATIVE_STEREO.log.
struct ProbeBlock {
  std::uint64_t frame;
  std::uint32_t size;
  std::uint32_t at;  // byte offset into g_probe_bytes
};
std::atomic<int> g_probe_phase{0};  // 0 idle, 1 settling (zero separation applied), 2 recording, 3 analysing
std::atomic<ULONGLONG> g_probe_t{0};
std::mutex g_probe_mutex;
std::vector<ProbeBlock> g_probe_blocks;
std::vector<unsigned char> g_probe_bytes;
constexpr std::size_t kProbeBytes = 24u << 20;

void probe_note(const void* cpu, std::size_t size) {
  if (g_probe_phase.load(std::memory_order_relaxed) != 2 || size < 64 || size > 8192 || (size & 15) != 0) {
    return;
  }
  unsigned char tmp[8192];
  if (!read_bytes(cpu, tmp, size)) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_probe_mutex);
  if (g_probe_bytes.size() + size > kProbeBytes) {
    return;
  }
  const auto at = static_cast<std::uint32_t>(g_probe_bytes.size());
  g_probe_bytes.insert(g_probe_bytes.end(), tmp, tmp + size);
  g_probe_blocks.push_back({g_presents.load(std::memory_order_relaxed), static_cast<std::uint32_t>(size), at});
}

void probe_analyse(std::vector<ProbeBlock> blocks, std::vector<unsigned char> bytes) {
  std::sort(blocks.begin(), blocks.end(), [](const ProbeBlock& a, const ProbeBlock& b) {
    return a.frame != b.frame ? a.frame < b.frame : a.size < b.size;
  });
  struct Hit {
    std::uint32_t size, off;
    int count;
    float a, b;
  };
  std::vector<Hit> hits, nearly;  // nearly: both values ordinary floats within 50% of each other (a time or phase looks so)
  std::size_t groups = 0, pairs = 0, identical = 0;
  std::uint64_t ops = 0;
  for (std::size_t i = 0; i < blocks.size();) {
    std::size_t j = i;
    while (j < blocks.size() && blocks[j].frame == blocks[i].frame && blocks[j].size == blocks[i].size) {
      ++j;
    }
    const std::size_t n = std::min<std::size_t>(j - i, 600);
    const std::uint32_t floats = blocks[i].size / 4;
    ++groups;
    for (std::size_t x = 0; x < n && ops < 400000000ull; ++x) {
      const float* fa = reinterpret_cast<const float*>(bytes.data() + blocks[i + x].at);
      for (std::size_t y = x + 1; y < n; ++y) {
        const float* fb = reinterpret_cast<const float*>(bytes.data() + blocks[i + y].at);
        std::uint32_t diff[4];
        int d = 0;
        for (std::uint32_t k = 0; k < floats && d <= 4; ++k) {
          if (std::memcmp(fa + k, fb + k, 4) != 0) {
            if (d < 4) diff[d] = k;
            ++d;
          }
        }
        ops += floats;
        if (d == 0) {
          ++identical;
        } else if (d <= 4) {
          ++pairs;
          for (int q = 0; q < d; ++q) {
            const std::uint32_t off = diff[q] * 4;
            bool found = false;
            for (Hit& h : hits) {
              if (h.size == blocks[i].size && h.off == off) {
                ++h.count;
                found = true;
                break;
              }
            }
            if (!found && hits.size() < 4096) {
              hits.push_back({blocks[i].size, off, 1, fa[diff[q]], fb[diff[q]]});
            }
            const float va = fa[diff[q]], vb = fb[diff[q]];
            const float big = std::max(std::fabs(va), std::fabs(vb));
            if (std::isnormal(va) && std::isnormal(vb) && big > 1e-4f && big < 1e7f && std::fabs(va - vb) < 0.5f * big) {
              bool seen = false;
              for (Hit& h : nearly) {
                if (h.size == blocks[i].size && h.off == off) {
                  ++h.count;
                  seen = true;
                  break;
                }
              }
              if (!seen && nearly.size() < 4096) {
                nearly.push_back({blocks[i].size, off, 1, va, vb});
              }
            }
          }
        }
      }
    }
    i = j;
  }
  std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.count > b.count; });
  std::sort(nearly.begin(), nearly.end(), [](const Hit& a, const Hit& b) { return a.count > b.count; });
  log_line("cbprobe: %zu blocks in %zu frame/size groups; %zu same-frame pairs differ in 1..4 floats, %zu identical",
           blocks.size(), groups, pairs, identical);
  {
    std::uint32_t sizes[64];
    int counts[64], ns = 0;
    for (const ProbeBlock& b : blocks) {
      int k = 0;
      while (k < ns && sizes[k] != b.size) ++k;
      if (k == ns && ns < 64) {
        sizes[ns] = b.size;
        counts[ns++] = 0;
      }
      if (k < ns) ++counts[k];
    }
    char line[400];
    int n = 0;
    for (int k = 0; k < ns && n < 360; ++k) {
      n += snprintf(line + n, sizeof(line) - n, " %u:%d", sizes[k], counts[k]);
    }
    log_line("cbprobe: blocks per size (bytes:count)%s", line);
  }
  for (std::size_t k = 0; k < nearly.size() && k < 30; ++k) {
    const Hit& h = nearly[k];
    log_line("cbprobe: NEAR size %4u +0x%03X in %5d pairs, e.g. %.7g | %.7g", h.size, h.off, h.count,
             static_cast<double>(h.a), static_cast<double>(h.b));
  }
  for (std::size_t k = 0; k < hits.size() && k < 25; ++k) {
    const Hit& h = hits[k];
    log_line("cbprobe: size %3u +0x%03X differs in %5d pairs, e.g. %.6g | %.6g", h.size, h.off, h.count,
             static_cast<double>(h.a), static_cast<double>(h.b));
  }
}

// Present thread: drives the probe's phases.
void probe_step() {
  const int phase = g_probe_phase.load();
  if (phase == 0 || phase == 3) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (phase == 1 && now - g_probe_t.load() > 800) {
    {
      std::lock_guard<std::mutex> lock(g_probe_mutex);
      g_probe_blocks.clear();
      g_probe_bytes.clear();
      g_probe_bytes.reserve(kProbeBytes);
    }
    g_probe_t.store(now);
    g_probe_phase.store(2);
  } else if (phase == 2 && now - g_probe_t.load() > 250) {
    g_probe_phase.store(3);
    g_zero_sep_forced.store(false);
    std::vector<ProbeBlock> blocks;
    std::vector<unsigned char> bytes;
    {
      std::lock_guard<std::mutex> lock(g_probe_mutex);
      blocks.swap(g_probe_blocks);
      bytes.swap(g_probe_bytes);
    }
    std::thread([b = std::move(blocks), y = std::move(bytes)]() mutable {
      probe_analyse(std::move(b), std::move(y));
      g_probe_phase.store(0);
    }).detach();
  }
}

bool request_cb_probe() {
  if (!g_stereo_running.load() || g_probe_phase.load() != 0) {
    log_line("cbprobe: not started (stereo %s, probe %s)", g_stereo_running.load() ? "running" : "paused",
             g_probe_phase.load() != 0 ? "busy" : "idle");
    return false;
  }
  g_zero_sep_forced.store(true);
  g_probe_t.store(GetTickCount64());
  g_probe_phase.store(1);
  log_line("cbprobe: start; both eyes at one point for about a second");
  return true;
}

bool request_wind_search() {
  if (!g_stereo_running.load() || g_wind_phase.load() != 0) {
    log_line("wind: not started (stereo %s, search %s)", g_stereo_running.load() ? "running" : "paused",
             g_wind_phase.load() != 0 ? "busy" : "idle");
    return false;
  }
  g_zero_sep_forced.store(true);
  g_wind_t = GetTickCount64();
  g_wind_phase.store(1);
  log_line("wind: start; both eyes at one point for about 3 s, keep the head still");
  return true;
}

bool cb_probe_recording() { return g_probe_phase.load(std::memory_order_relaxed) == 2; }

// ---- first-person arm projection (see g_fp_proj) ----
// Called for EVERY 1536-byte CBV (the sampler's 1-in-32 thinning skips this size), on whichever thread creates it:
// the FP block is not necessarily created on an eye's Render thread, so eyes are matched by view rows, not by thread.
struct FpWorld {
  alignas(16) unsigned char block[1536];
  ULONGLONG at = 0;
  int eye = -1;
};
FpWorld g_fp_world[4];  // recent world (eye) blocks, g_fp_mutex
unsigned g_fp_next = 0;
std::mutex g_fp_mutex;
std::atomic<unsigned> g_fp_worlds{0}, g_fp_seen{0}, g_fp_same_view{0}, g_fp_other_view{0}, g_fp_no_world{0}, g_fp_written{0},
    g_fp_failed{0}, g_fp_calls{0};
std::atomic<ULONGLONG> g_fp_log_next{0};
// Projection-dependent rows of the 1536-byte block (CAMERA_FINDINGS field map): projection, worldToClip, cellToClip,
// inverse projection, clipToCell, clipToWorld, previous worldToClip/cellToClip, projection duplicate, six frustum planes
// and near/far. View, inverse view, previous view and positions are left as the block has them.
constexpr std::size_t kFpRows[][2] = {{0x080, 0x0C0}, {0x0C0, 0x100}, {0x100, 0x140}, {0x1C0, 0x200}, {0x200, 0x240},
                                      {0x240, 0x280}, {0x280, 0x2C0}, {0x2C0, 0x300}, {0x340, 0x380}, {0x380, 0x3F0}};
bool near_rel(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance * std::fabs(b); }
float view_error(const unsigned char* a, const unsigned char* b) {
  const float* va = reinterpret_cast<const float*>(a);
  const float* vb = reinterpret_cast<const float*>(b);
  float e = 0.f;  // worldToCamera (0x000) and cellToCamera (0x040)
  for (int i = 0; i < 32; ++i) e = std::max(e, std::fabs(va[i] - vb[i]) / (1.f + std::fabs(vb[i])));
  return e;
}

thread_local int t_fp_draws = 0;
bool fp_draw_window() {
  if (t_fp_draws <= 0) return false;
  --t_fp_draws;
  return true;
}

void note_camera_cbv(const void* cpu, std::size_t size) {
  if (size != 1536 || cpu == nullptr || !g_stereo_running.load(std::memory_order_relaxed)) return;
  g_fp_calls.fetch_add(1, std::memory_order_relaxed);
  unsigned char* b = static_cast<unsigned char*>(const_cast<void*>(cpu));
  const ULONGLONG now = GetTickCount64();
  float proj[16];
  if (!read_bytes(b + 0x080, proj, sizeof(proj))) return;
  const bool perspective = std::fabs(std::fabs(proj[11]) - 1.f) < 0.01f && std::fabs(proj[15]) < 0.01f && proj[5] > 0.01f &&
                           proj[0] > 0.01f;
  const float m0[2] = {g_expect_m0[0].load(std::memory_order_relaxed), g_expect_m0[1].load(std::memory_order_relaxed)};
  const float m5[2] = {g_expect_m5[0].load(std::memory_order_relaxed), g_expect_m5[1].load(std::memory_order_relaxed)};
  if (ULONGLONG due = g_fp_log_next.load(); now >= due && g_fp_log_next.compare_exchange_strong(due, now + 5000)) {
    log_line("fpproj depth experiment: %s written=%u", (g_fp_depth_menu || g_fp_depth_file.load()) ? "ON" : "off", g_fp_depth_written.load());
    log_line("fpproj status: write=%d calls=%u worlds=%u seen=%u same=%u other=%u noworld=%u written=%u failed=%u expect m0=%.4f/%.4f m5=%.4f/%.4f",
             fp_proj_enabled() ? 1 : 0, g_fp_calls.load(), g_fp_worlds.load(), g_fp_seen.load(), g_fp_same_view.load(),
             g_fp_other_view.load(), g_fp_no_world.load(), g_fp_written.load(), g_fp_failed.load(), m0[0], m0[1], m5[0], m5[1]);
  }
  if (!perspective) return;
  for (int e = 0; e < 2; ++e) {
    if (m0[e] > 0.f && m5[e] > 0.f && near_rel(proj[0], m0[e], 0.02f) && near_rel(proj[5], m5[e], 0.02f)) {
      std::lock_guard<std::mutex> lock(g_fp_mutex);
      FpWorld& w = g_fp_world[g_fp_next++ % 4];
      if (read_bytes(b, w.block, sizeof(w.block))) {
        w.at = now;
        w.eye = e;
        g_fp_worlds.fetch_add(1, std::memory_order_relaxed);
      }
      return;
    }
  }
  // Candidate: clearly narrower than both eyes (larger m5).
  if (!(m5[0] > 0.f && m5[1] > 0.f) || proj[5] < std::max(m5[0], m5[1]) * 1.15f) return;
  g_fp_seen.fetch_add(1, std::memory_order_relaxed);
  t_fp_draws = 1;  // the next draw recorded on this thread consumes this block (camera.cpp depth probe)
  alignas(16) unsigned char block[1536];
  if (!read_bytes(b, block, sizeof(block))) return;
  alignas(16) unsigned char world[1536];
  float best = 1e30f;
  int eye = -1;
  {
    std::lock_guard<std::mutex> lock(g_fp_mutex);
    for (const FpWorld& w : g_fp_world) {
      if (w.at == 0 || now - w.at > 100) continue;
      const float err = view_error(block, w.block);
      if (err < best) {
        best = err;
        eye = w.eye;
        std::memcpy(world, w.block, sizeof(world));
      }
    }
  }
  if (eye < 0) {
    g_fp_no_world.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const bool same_view = best < 1e-4f;
  (same_view ? g_fp_same_view : g_fp_other_view).fetch_add(1, std::memory_order_relaxed);
  static std::atomic<ULONGLONG> detail_next{0};
  if (ULONGLONG due = detail_next.load(); now >= due && detail_next.compare_exchange_strong(due, now + 5000)) {
    unsigned diff = 0;  // bit per 64-byte row (24 rows) that differs between the FP block and the eye block
    for (int row = 0; row < 24; ++row)
      if (std::memcmp(block + row * 64, world + row * 64, 64) != 0) diff |= 1u << row;
    const float* wp = reinterpret_cast<const float*>(world + 0x080);
    log_line("fpproj eye%d: fp fovy=%.2f m0=%.4f m5=%.4f m8=%.4f m9=%.4f near=%.4f | eye fovy=%.2f m0=%.4f m5=%.4f m8=%.4f m9=%.4f near=%.4f | view_err=%.2g rows_differ=0x%06X",
             eye, 2.f * std::atan(1.f / proj[5]) * 57.2958f, proj[0], proj[5], proj[8], proj[9], proj[14],
             2.f * std::atan(1.f / wp[5]) * 57.2958f, wp[0], wp[5], wp[8], wp[9], wp[14], best, diff);
    // Floats that still differ outside the rows the override copies (0x3F0..0x5FF: positions, render-target and other
    // per-view constants). A first-person FOV correction (e.g. a tan(half FOV) ratio) would show up here.
    const float* fa = reinterpret_cast<const float*>(block);
    const float* fb = reinterpret_cast<const float*>(world);
    char line[480];
    int used = 0, listed = 0;
    for (int i = 0x3F0 / 4; i < 1536 / 4 && listed < 40; ++i) {
      if (fa[i] == fb[i]) continue;
      used += std::snprintf(line + used, sizeof(line) - used, " +%03X fp=%.5g eye=%.5g", i * 4, fa[i], fb[i]);
      ++listed;
      if (used > 400 || listed == 40) {
        log_line("fpproj eye%d rest:%s", eye, line);
        used = 0;
      }
    }
    if (used > 0) log_line("fpproj eye%d rest:%s", eye, line);
  }
  if (!same_view || !fp_proj_enabled()) return;
  bool ok = true;
  for (const auto& r : kFpRows) ok = write_bytes(b + r[0], world + r[0], r[1] - r[0]) && ok;
  if ((g_fp_depth_menu || g_fp_depth_file.load(std::memory_order_relaxed)) && write_bytes(b + 0x4F0, world + 0x4F0, 8)) g_fp_depth_written.fetch_add(1, std::memory_order_relaxed);
  (ok ? g_fp_written : g_fp_failed).fetch_add(1, std::memory_order_relaxed);
}

void note_cbv(const void* cpu, std::size_t size) {
  probe_note(cpu, size);
  if (size != 1536 || cpu == nullptr) {
    return;
  }
  float proj[16];
  float pos[3];
  const unsigned char* b = static_cast<const unsigned char*>(cpu);
  if (!read_bytes(b + 0x080, proj, sizeof(proj)) || !read_bytes(b + 0x3F0, pos, sizeof(pos))) {
    return;
  }
  // Perspective with the engine's row-vector convention: w comes from row 2 (|P[11]| == 1), P[15] == 0.
  if (!(std::fabs(std::fabs(proj[11]) - 1.f) < 0.01f && std::fabs(proj[15]) < 0.01f && proj[5] > 0.01f && proj[0] > 0.01f)) {
    return;
  }
  // The main view's camera. Once the second view exists that is identified by view (a VR render shape can be
  // taller than wide, so aspect alone no longer tells it from other views); before, by its wide window aspect.
  note_eye_cb(b, proj[8]);
  const int vi = t_view_index;
  if (vi >= 0 && (g_views[vi].view == g_created_main || g_views[vi].view == g_created)) {
    const int e = g_views[vi].view == g_created ? 1 : 0;
    g_observed_projection[e][0].store(proj[0], std::memory_order_relaxed);
    g_observed_projection[e][1].store(proj[5], std::memory_order_relaxed);
    g_observed_projection[e][2].store(proj[8], std::memory_order_relaxed);
    g_observed_projection[e][3].store(proj[9], std::memory_order_relaxed);
    g_observed_projection_count[e].fetch_add(1, std::memory_order_relaxed);
  }
  const bool wide = proj[0] > 0.01f && proj[5] / proj[0] > 1.2f;
  const bool main_view = g_created_main != nullptr && vi >= 0 ? g_views[vi].view == g_created_main : wide;
  if (main_view) {
    g_proj_m0.store(proj[0], std::memory_order_relaxed);
    g_proj_m5.store(proj[5], std::memory_order_relaxed);
    g_proj_m8.store(proj[8], std::memory_order_relaxed);
    g_proj_m9.store(proj[9], std::memory_order_relaxed);
  }
  const int index = t_view_index;
  if (index < 0) {
    g_cams_unattributed.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  ViewInfo& v = g_views[index];
  v.cams.fetch_add(1, std::memory_order_relaxed);
  v.fovy_milli.store(static_cast<std::int32_t>(2.f * std::atan(1.f / proj[5]) * 57295.78f), std::memory_order_relaxed);
  v.aspect_milli.store(static_cast<std::int32_t>(proj[5] / proj[0] * 1000.f), std::memory_order_relaxed);
  v.pos_x.store(static_cast<std::int32_t>(pos[0] * 100.f), std::memory_order_relaxed);
  v.pos_y.store(static_cast<std::int32_t>(pos[1] * 100.f), std::memory_order_relaxed);
  v.pos_z.store(static_cast<std::int32_t>(pos[2] * 100.f), std::memory_order_relaxed);
}

bool* fp_proj_menu_flag() { return &g_fp_proj_menu; }
bool* fp_depth_menu_flag() { return &g_fp_depth_menu; }
bool* fp_occlude_menu_flag() { return &g_fp_occlude_menu; }
bool fp_occlude_enabled() { return occlude_enabled_impl(); }
bool pose_write_enabled() { return g_pose_write.load(std::memory_order_relaxed); }

void recentre() {
  g_recentre_request.store(true);
  xr_runtime::recentre_gui_panel();
}

void recentre_menu_panel() { xr_runtime::recentre_gui_panel(); }

bool front_end() { return g_front_end.load(std::memory_order_relaxed); }

// ---- the game's UI viewport size (eye mode, 2026-10-10) ----------------------------------------------------------
// [AUTHOR] 2026-10-09 20:2x-20:5x: with the window widescreen the HUD and the 3D main menu change size and position
// with Render height. GetClientRect is not what the game reads (0 calls from afop.exe for the game window). [STATIC]
// Blue::GetGameViewportSize::Execute (0x14430D620) reads the game viewport size at
// [[[0x149ADB058]+0x128]+0x28]+0x418 -> +0x314 / +0x318 (via 0x1435A4B30). This logs it every 5 s and, while eye mode
// runs, writes the eye size there (the original is put back when stereo pauses). "nouiviewport" turns the write off.
constexpr std::uintptr_t kGameRootRva = 0x9ADB058;
std::atomic<bool> g_ui_viewport_write{true};
std::atomic<float> g_ui_canvas_w{0.f}, g_ui_canvas_h{0.f};
std::uint32_t g_ui_vp_saved[2] = {0, 0};
bool g_ui_vp_written = false;

unsigned char* game_viewport_size() {
  if (g_module_base == nullptr) {
    return nullptr;
  }
  void* a = nullptr;
  void* b = nullptr;
  void* c = nullptr;
  void* v = nullptr;
  if (!read_ptr(g_module_base + kGameRootRva, &a) || a == nullptr || !read_ptr(static_cast<unsigned char*>(a) + 0x128, &b) ||
      b == nullptr || !read_ptr(static_cast<unsigned char*>(b) + 0x28, &c) || c == nullptr ||
      !read_ptr(static_cast<unsigned char*>(c) + 0x418, &v) || v == nullptr) {
    return nullptr;
  }
  return static_cast<unsigned char*>(v) + 0x314;
}

void ui_viewport_step() {
  unsigned char* p = game_viewport_size();
  std::uint32_t cur[2] = {0, 0};
  const bool readable = p != nullptr && read_u32(p, &cur[0]) && read_u32(p + 4, &cur[1]);
  float logical[2] = {};
  std::memcpy(logical,cur,sizeof(logical));
  if (readable && std::isfinite(logical[0]) && std::isfinite(logical[1]) &&
      logical[0]>=64.f && logical[1]>=64.f && logical[0]<=16384.f && logical[1]<=16384.f) {
    g_ui_canvas_w.store(logical[0],std::memory_order_relaxed);
    g_ui_canvas_h.store(logical[1],std::memory_order_relaxed);
  } else {
    g_ui_canvas_w.store(0.f,std::memory_order_relaxed);
    g_ui_canvas_h.store(0.f,std::memory_order_relaxed);
  }
  std::uint32_t ew = 0, eh = 0;
  const bool want = readable && g_ui_viewport_write.load(std::memory_order_relaxed) && eye_mode_on() && eyesize_for(false, &ew, &eh);
  if (want) {
    if (!g_ui_vp_written || cur[0] != ew || cur[1] != eh) {
      if (!g_ui_vp_written || !(cur[0] == ew && cur[1] == eh)) {
        if (!(cur[0] == ew && cur[1] == eh)) {
          g_ui_vp_saved[0] = cur[0];  // the game's own value (it may also have changed after a window resize)
          g_ui_vp_saved[1] = cur[1];
        }
      }
      write_u32(p, ew);
      write_u32(p + 4, eh);
      if (!g_ui_vp_written) {
        log_line("ui viewport: game %ux%u -> eye %ux%u while the eyes run", g_ui_vp_saved[0], g_ui_vp_saved[1], ew, eh);
      }
      g_ui_vp_written = true;
    }
  } else if (g_ui_vp_written) {
    if (readable && g_ui_vp_saved[0] != 0) {
      write_u32(p, g_ui_vp_saved[0]);
      write_u32(p + 4, g_ui_vp_saved[1]);
      log_line("ui viewport: restored %ux%u (stereo paused)", g_ui_vp_saved[0], g_ui_vp_saved[1]);
    }
    g_ui_vp_written = false;
  }
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last >= 5000) {
    last = now;
    log_line("ui viewport: logical canvas %.0fx%.0f (raw %u/%u, read-only)",
             g_ui_canvas_w.load(),g_ui_canvas_h.load(),cur[0],cur[1]);
    char hud[160];
    camera::hud_report(hud, sizeof(hud));
    log_line("%s", hud);
  }
}

void on_present() {
  ui_cursor::report();
  ui_piece::report();
  ui_layout::report();
  g_presents.fetch_add(1, std::memory_order_relaxed);
  ui_viewport_step();
  g_pass_presents[g_stereo_running.load(std::memory_order_relaxed) ? 0 : 1].fetch_add(1, std::memory_order_relaxed);
  log_pass_census();
  log_eye_cb_diff();
  probe_step();
  trace('P', -1, static_cast<std::uint32_t>(g_presents.load(std::memory_order_relaxed)), 0);
}

// Read-only diff of the main view against the created second view (Present thread, once per second view, a few
// seconds into stereo). Eye captures 2026-10-05 16:31 showed the UI inside the main view's own output texture, so the
// UI (and probably the effects that only the left eye shows) is per-view work; this looks for the per-view field that
// selects it. Pointer pairs and the frame ring are skipped; everything else that differs is logged.
void log_view_diff() {
  static void* done_for = nullptr;
  static ULONGLONG running_since = 0;
  void* created = g_created;
  if (created == nullptr || g_view_count.load() == 0 || !g_stereo_running.load()) {
    running_since = 0;
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (running_since == 0) {
    running_since = now;
  }
  if (done_for == created || now - running_since < 3000) {
    return;
  }
  done_for = created;
  void* main = g_views[0].view;
  static unsigned char a[kFrameRing], b[kFrameRing];
  if (!read_bytes(main, a, sizeof(a)) || !read_bytes(created, b, sizeof(b))) {
    log_line("viewdiff: read failed");
    return;
  }
  const auto base = reinterpret_cast<std::uintptr_t>(g_module_base);
  auto is_ptr = [](std::uint64_t v) { return v >= 0x10000 && v < 0x00007FFFFFFFFFFFull && (v & 3) == 0; };
  auto describe = [&](std::uint64_t v, char* out, size_t n) {
    if (base != 0 && v >= base && v < base + g_image_size) {
      snprintf(out, n, "afop+0x%llX", static_cast<unsigned long long>(v - base));
    } else {
      snprintf(out, n, "%016llX", static_cast<unsigned long long>(v));
    }
  };
  {
    std::uint64_t wrap_vt = 0;
    char d[40];
    read_bytes(static_cast<unsigned char*>(main) - 0x60, &wrap_vt, sizeof(wrap_vt));
    describe(wrap_vt, d, sizeof(d));
    log_line("viewdiff: main %p created %p | main-0x60 (factory wrapper vtable?) = %s", main, created, d);
  }
  int logged = 0, same_ptr = 0;
  char line[512];
  int n = 0;
  for (std::uint32_t off = 0; off < kFrameRing; off += 8) {
    std::uint64_t va = 0, vb = 0;
    std::memcpy(&va, a + off, 8);
    std::memcpy(&vb, b + off, 8);
    if (va == vb) {
      continue;
    }
    if (is_ptr(va) && is_ptr(vb) && !(base != 0 && ((va >= base && va < base + g_image_size) || (vb >= base && vb < base + g_image_size)))) {
      ++same_ptr;  // two different heap objects: expected (own buffers)
      continue;
    }
    if (logged >= 160) {
      continue;
    }
    char da[40], db[40];
    describe(va, da, sizeof(da));
    describe(vb, db, sizeof(db));
    n += snprintf(line + n, sizeof(line) - n, " +%04X %s/%s", off, da, db);
    ++logged;
    if (logged % 4 == 0) {
      log_line("viewdiff (main/created):%s", line);
      n = 0;
    }
  }
  if (n > 0) {
    log_line("viewdiff (main/created):%s", line);
  }
  log_line("viewdiff: %d differing qwords logged, %d heap-pointer pairs skipped", logged, same_ptr);
  // Object slots the main view has and the second view lacks: what they point at (RTTI class names where present).
  int shares = 0;
  for (std::uint32_t off = 0x40; off < 0x200; off += 8) {
    std::uint64_t va = 0, vb = 0;
    std::memcpy(&va, a + off, 8);
    std::memcpy(&vb, b + off, 8);
    if (!is_ptr(va) || vb != 0) {
      continue;
    }
    std::uint64_t obj[6] = {};
    read_bytes(reinterpret_cast<void*>(va), obj, sizeof(obj));
    char cls[96] = "-", inner[96] = "-", d[6][40];
    rtti_name(obj[0], cls, sizeof(cls));  // the slot points at an object with a vtable
    std::uint64_t inner_vt = 0;
    if (is_ptr(obj[0]) && read_bytes(reinterpret_cast<void*>(obj[0]), &inner_vt, 8)) {
      rtti_name(inner_vt, inner, sizeof(inner));  // or at a handle/node whose first field is the object
    }
    for (int k = 0; k < 6; ++k) {
      describe(obj[k], d[k], sizeof(d[k]));
    }
    log_line("viewslot +%03X -> %016llX class %s | [0] class %s | %s %s %s %s %s %s", off, static_cast<unsigned long long>(va), cls,
             inner, d[0], d[1], d[2], d[3], d[4], d[5]);
    if (shares < kMaxShare) {
      g_share_off[shares++] = off;
    }
  }
  g_share_count.store(shares);
  log_line("viewslot: %d slots set on the main view and empty on the second view (\"sharehooks\" copies them)", shares);
}

// Read-only code dump for static reading (CODEDUMP.log next to the DLL). Only the readable code section (see
// text_section) is dumped; anything else (the protected regions) is refused. Requests come from scripts\AvatarVR_dump.txt
// (lines "RVA LEN", hex), else a default list of the main-view-only job entries and the slot records found so far.

// Allowed: ranges inside the executable section that holds Gfx_View::Render (the readable code the hooks already
// signature-check). In this build that section is named ".xpdata" (CODEDUMP 2026-10-06: every request fell into it).
// Every other section, including the protected ".sbss" region, is refused.
bool text_section(std::uint32_t rva, std::uint32_t len, char* name) {
  const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_module_base);
  const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(g_module_base + dos->e_lfanew);
  const IMAGE_SECTION_HEADER* sec = IMAGE_FIRST_SECTION(nt);
  int code = -1, hit = -1;
  for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; ++i) {
    const std::uint32_t va = sec[i].VirtualAddress, size = sec[i].Misc.VirtualSize;
    if (kRenderRva >= va && kRenderRva < va + size) {
      code = static_cast<int>(i);
    }
    if (rva >= va && rva + len <= va + size) {
      hit = static_cast<int>(i);
    }
  }
  if (hit < 0) {
    std::strcpy(name, "?");
    return false;
  }
  std::memcpy(name, sec[hit].Name, 8);
  name[8] = '\0';
  return hit == code && (sec[hit].Characteristics & IMAGE_SCN_MEM_EXECUTE) != 0 && std::strncmp(name, ".sbss", 5) != 0;
}

// Background thread: for each word, ASCII strings in the readable data sections (never .sbss) that contain it, and the
// code sites in Render's section that load their address with a RIP-relative LEA. Appended to CODEDUMP.log.
void find_xrefs(std::vector<std::string> words) {
  const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_module_base);
  const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(g_module_base + dos->e_lfanew);
  const IMAGE_SECTION_HEADER* sec = IMAGE_FIRST_SECTION(nt);
  std::uint32_t code_va = 0, code_size = 0;
  struct Hit {
    std::uint32_t rva;
    std::string text;
    int word;
  };
  std::vector<Hit> hits;
  for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; ++i) {
    char nm[9] = {};
    std::memcpy(nm, sec[i].Name, 8);
    const std::uint32_t va = sec[i].VirtualAddress, size = sec[i].Misc.VirtualSize;
    if (kRenderRva >= va && kRenderRva < va + size) {
      code_va = va;
      code_size = size;
      continue;
    }
    if (std::strncmp(nm, ".sbss", 5) == 0 || (sec[i].Characteristics & IMAGE_SCN_MEM_READ) == 0 ||
        (sec[i].Characteristics & IMAGE_SCN_MEM_EXECUTE) != 0) {
      continue;
    }
    const unsigned char* base = g_module_base + va;
    std::vector<unsigned char> buf(size);
    if (!read_bytes(base, buf.data(), size)) {
      continue;
    }
    for (std::size_t w = 0; w < words.size(); ++w) {
      const std::string& word = words[w];
      int per_word = 0;
      for (std::size_t o = 0; o + word.size() <= size && per_word < 40; ++o) {
        if (std::memcmp(buf.data() + o, word.data(), word.size()) != 0) {
          continue;
        }
        std::size_t a = o, b = o + word.size();  // widen to the printable run
        while (a > 0 && buf[a - 1] >= 0x20 && buf[a - 1] < 0x7F && o - a < 96) --a;
        while (b < size && buf[b] >= 0x20 && buf[b] < 0x7F && b - o < 96) ++b;
        if (a > 0 && buf[a - 1] != 0) {
          o = b;
          continue;  // not the start of a C string: keep only strings that begin at a NUL boundary
        }
        hits.push_back({va + static_cast<std::uint32_t>(a), std::string(reinterpret_cast<const char*>(buf.data() + a), b - a),
                        static_cast<int>(w)});
        ++per_word;
        o = b;
      }
    }
  }
  std::vector<std::vector<std::uint32_t>> refs(hits.size());
  if (code_size != 0 && !hits.empty()) {
    std::vector<unsigned char> code(code_size);
    if (read_bytes(g_module_base + code_va, code.data(), code_size)) {
      for (std::uint32_t o = 0; o + 7 <= code_size; ++o) {
        // lea r64, [rip+disp32]: REX.W (48/4C) 8D modrm with mod=00 rm=101
        if ((code[o] != 0x48 && code[o] != 0x4C) || code[o + 1] != 0x8D || (code[o + 2] & 0xC7) != 0x05) {
          continue;
        }
        std::int32_t disp = 0;
        std::memcpy(&disp, code.data() + o + 3, 4);
        const std::uint32_t target = code_va + o + 7 + static_cast<std::uint32_t>(disp);
        for (std::size_t h = 0; h < hits.size(); ++h) {
          if (hits[h].rva == target && refs[h].size() < 8) {
            refs[h].push_back(code_va + o);
          }
        }
      }
    }
  }
  HANDLE out = CreateFileW(g_dump_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (out == INVALID_HANDLE_VALUE) {
    return;
  }
  auto put = [&](const std::string& line) {
    DWORD w = 0;
    WriteFile(out, line.data(), static_cast<DWORD>(line.size()), &w, nullptr);
  };
  put("== strings and code references (lea rip-relative)\r\n");
  for (std::size_t h = 0; h < hits.size(); ++h) {
    char head[64];
    snprintf(head, sizeof(head), "str %08X [%s] refs", hits[h].rva, words[hits[h].word].c_str());
    std::string line = head;
    for (std::uint32_t r : refs[h]) {
      char ref[16];
      snprintf(ref, sizeof(ref), " %08X", r);
      line += ref;
    }
    line += " | " + hits[h].text + "\r\n";
    put(line);
  }
  CloseHandle(out);
  log_line("codedump: %zu strings for %zu words appended to CODEDUMP.log", hits.size(), words.size());
}

void dump_code() {
  static bool done = false;
  static ULONGLONG first = 0;
  if (done || g_module_base == nullptr || g_dump_path.empty()) {
    return;
  }
  const ULONGLONG now = GetTickCount64();
  if (first == 0) {
    first = now;
  }
  if (now - first < 10000) {
    return;  // after start-up, once the image is fully mapped
  }
  done = true;
  struct Req {
    std::uint32_t rva, len;
  };
  std::vector<Req> reqs;
  std::vector<std::string> words;
  {
    HANDLE file = CreateFileW(g_dump_request.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file != INVALID_HANDLE_VALUE) {
      char text[4096] = {};
      DWORD read = 0;
      ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
      CloseHandle(file);
      for (char* line_start = text; *line_start != '\0';) {
        char* nl = std::strchr(line_start, '\n');
        if (std::strncmp(line_start, "find ", 5) == 0) {
          std::string w(line_start + 5, nl != nullptr ? nl : line_start + std::strlen(line_start));
          while (!w.empty() && (w.back() == '\r' || w.back() == ' ')) w.pop_back();
          if (!w.empty()) words.push_back(w);
          std::memset(line_start, ' ', (nl != nullptr ? nl : line_start + std::strlen(line_start)) - line_start);
        }
        if (nl == nullptr) break;
        line_start = nl + 1;
      }
      for (char* p = text; *p != '\0';) {
        char* end = nullptr;
        const unsigned long rva = std::strtoul(p, &end, 16);
        if (end == p) {
          ++p;
          continue;
        }
        char* end2 = nullptr;
        const unsigned long len = std::strtoul(end, &end2, 16);
        if (end2 == end) {
          break;
        }
        reqs.push_back({static_cast<std::uint32_t>(rva), static_cast<std::uint32_t>(std::min<unsigned long>(len, 0x2000))});
        p = end2;
      }
    }
  }
  if (reqs.empty()) {
    // Round 2 (2026-10-06): the passes the UI record (+0x130, 0x55EFE0) calls, and the other per-view pass records.
    reqs = {{0x586AA0, 0x400}, {0x58C580, 0x300}, {0x590400, 0x300}, {0x5998C0, 0x200},
            {0x563EF0, 0x200}, {0x55E8E0, 0x300}, {0x55F150, 0x300}, {0x55EE10, 0x200}};
  }
  HANDLE out = CreateFileW(g_dump_path.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (out == INVALID_HANDLE_VALUE) {
    return;
  }
  auto put = [&](const char* line) {
    DWORD w = 0;
    WriteFile(out, line, static_cast<DWORD>(std::strlen(line)), &w, nullptr);
  };
  char line[160];
  snprintf(line, sizeof(line), "codedump afop.exe image 0x%zX bytes, %zu requests (read-only, Render's code section only)\r\n", g_image_size, reqs.size());
  put(line);
  {
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_module_base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(g_module_base + dos->e_lfanew);
    const IMAGE_SECTION_HEADER* sec = IMAGE_FIRST_SECTION(nt);
    for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; ++i) {
      char nm[9] = {};
      std::memcpy(nm, sec[i].Name, 8);
      snprintf(line, sizeof(line), "section %-8s rva %08X size %08X flags %08X\r\n", nm, static_cast<unsigned>(sec[i].VirtualAddress),
               static_cast<unsigned>(sec[i].Misc.VirtualSize), static_cast<unsigned>(sec[i].Characteristics));
      put(line);
    }
  }
  int dumped = 0;
  for (const Req& r : reqs) {
    char sec[9];
    if (r.len == 0 || !text_section(r.rva, r.len, sec)) {
      snprintf(line, sizeof(line), "== %08X +%X: refused (section %s)\r\n", r.rva, r.len, sec);
      put(line);
      continue;
    }
    snprintf(line, sizeof(line), "== %08X +%X (%s)\r\n", r.rva, r.len, sec);
    put(line);
    for (std::uint32_t o = 0; o < r.len; o += 32) {
      unsigned char b[32] = {};
      const std::uint32_t n = std::min<std::uint32_t>(32, r.len - o);
      if (!read_bytes(g_module_base + r.rva + o, b, n)) {
        put("   (unreadable)\r\n");
        break;
      }
      int k = snprintf(line, sizeof(line), "%08X ", r.rva + o);
      for (std::uint32_t i = 0; i < n; ++i) {
        k += snprintf(line + k, sizeof(line) - k, "%02X", b[i]);
      }
      snprintf(line + k, sizeof(line) - k, "\r\n");
      put(line);
    }
    ++dumped;
  }
  CloseHandle(out);
  log_line("codedump: %d of %zu ranges written to CODEDUMP.log", dumped, reqs.size());
  if (words.empty()) {
    words = {"particle", "Particle", "fauna", "Fauna", "waterfall", "Waterfall", "VFX", "vfx", "critter", "Critter"};
  }
  std::thread(find_xrefs, words).detach();  // ~100 MB scan: off the Present thread
}

int ui_pass_eye() { return t_ui_eye; }

int ui_pass_kind() { return t_ui_kind; }

bool ui_layer_active() {
  return g_ui_place.load(std::memory_order_relaxed) && g_menu_panel.load(std::memory_order_relaxed) &&
         g_front_end.load(std::memory_order_relaxed) && g_stereo_running.load(std::memory_order_relaxed);
}

bool ui_canvas_size(float* w, float* h) {
  *w=g_ui_canvas_w.load(std::memory_order_relaxed);
  *h=g_ui_canvas_h.load(std::memory_order_relaxed);
  return std::isfinite(*w) && std::isfinite(*h) && *w>=64.f && *h>=64.f && *w<=16384.f && *h<=16384.f;
}

// UI pixel (x, y) of the game's layout -> (ox + x * sx, oy + y * sy) in this eye's image. The layout's centre goes
// straight ahead at UI distance D (eye-relative tangent +-ipd/2/D, toward the nose), its size is SIZE % of the eye's
// image span, and it is raised by HEIGHT degrees. The eye's image covers tangents [l, r] x [down, up] (rendered
// frustum). Identity at SIZE 100 with a centred frustum and D = infinity.
// Scan highlights in the right eye. [AUTHOR] + capture 2026-10-08 12:0x: with the 11:57 build the left eye's highlights
// sit on their animals, while the right eye's sit at the same screen pixels as the left eye's (the main view's highlight
// images are shared; the second view has no neighbour passes) and seem to move with the head. Both eyes look the same
// way, so a direction at left-image pixel x lies at right-image pixel ox + x * sx, from the two rendered frusta:
// xR = W (lL - lR) / (rR - lR) + xL (rL - lL) / (rR - lR); yR = H (uR - uL) / (uR - dR) + yL (uL - dL) / (uR - dR).
// The eye separation (parallax, ipd / depth) is left out: exact far away, off by a few pixels for things a few metres
// away. "noreproj" turns it off.
std::atomic<bool> g_scan_reproj{true};

bool scan_offsets(float rt_w, float rt_h, float* ox, float* oy, float* sx, float* sy) {
  if (rt_w < 16.f || rt_h < 16.f) {
    return false;
  }
  float L[4], R[4];
  {
    std::lock_guard<std::mutex> lock(g_ui_tan_mutex);
    if (!g_ui_tan_valid[0] || !g_ui_tan_valid[1]) {
      return false;
    }
    std::memcpy(L, g_ui_tan[0], sizeof(L));
    std::memcpy(R, g_ui_tan[1], sizeof(R));
  }
  const float rw = R[1] - R[0], rh = R[2] - R[3];
  if (!(rw > 0.1f && rh > 0.1f)) {
    return false;
  }
  *ox = rt_w * (L[0] - R[0]) / rw;
  *sx = (L[1] - L[0]) / rw;
  *oy = rt_h * (R[2] - L[2]) / rh;
  *sy = (L[2] - L[3]) / rh;
  return true;
}

bool scan_reproject(float rt_w, float rt_h, float* ox, float* oy, float* sx, float* sy) {
  return g_scan_reproj.load(std::memory_order_relaxed) && scan_offsets(rt_w, rt_h, ox, oy, sx, sy);
}

bool scan_plane_uv(int eye, float width, float* uv) {
  std::lock_guard<std::mutex> lock(g_ui_tan_mutex);
  return g_ui_tan_valid[0] && g_ui_tan_valid[1] &&
      scan_uv::plane_map(g_ui_tan[0],g_ui_tan[1],eye,width,uv);
}

bool fx_plane_rect(int eye, float size, float* rect) {
  if ((eye != 1 && eye != 2) || !std::isfinite(size) || size < 0.5f || size > 3.f) return false;
  float L[4], R[4];
  {
    std::lock_guard<std::mutex> lock(g_ui_tan_mutex);
    if (!g_ui_tan_valid[0] || !g_ui_tan_valid[1]) return false;
    std::memcpy(L, g_ui_tan[0], sizeof(L));
    std::memcpy(R, g_ui_tan[1], sizeof(R));
  }
  // As scan_uv::plane_map, in tangents, so both eyes see each point at one direction and it reads as one surface far ahead. Centred
  // on head forward, and at Size 1 reaching the farthest edge of either eye: the lenses are off-centre, and a plane only as wide as one
  // lens left its border inside the outer edge [HEADSET 2026-11-07, left border at 11% of the left eye, as computed].
  const float* e = eye == 1 ? L : R;
  const float ew = e[1] - e[0], eh = e[2] - e[3];
  if (ew < 0.1f || eh < 0.1f) return false;
  const float middle_y = std::fmax(L[2] - L[3], R[2] - R[3]) * 0.05f;
  const float half_x = std::fmax(std::fmax(-L[0], L[1]), std::fmax(-R[0], R[1])) * size;
  const float half_y = std::fmax(std::fmax(L[2], R[2]) - middle_y, middle_y - std::fmin(L[3], R[3])) * size;
  rect[0] = (-half_x - e[0]) / ew;
  rect[2] = (half_x - e[0]) / ew;
  rect[1] = (e[2] - (middle_y + half_y)) / eh;
  rect[3] = (e[2] - (middle_y - half_y)) / eh;
  return true;
}

bool ui_map(int eye, float rt_w, float rt_h, float* ox, float* oy, float* sx, float* sy,
            float source_w, float source_h) {
  if ((eye != 1 && eye != 2) || rt_w < 16.f || rt_h < 16.f || !g_ui_place.load(std::memory_order_relaxed) ||
      !g_stereo_running.load(std::memory_order_relaxed)) {
    return false;
  }
  float logical_w=g_ui_canvas_w.load(std::memory_order_relaxed);
  float logical_h=g_ui_canvas_h.load(std::memory_order_relaxed);
  unsigned ew=0, eh=0;
  const bool independent=eye_size(&ew,&eh);
  if (independent && (logical_w<64.f || logical_h<64.f)) {
    unsigned cw=0,ch=0;
    if (dx12::client_size(&cw,&ch)) { logical_w=static_cast<float>(cw); logical_h=static_cast<float>(ch); }
  }
  const bool logical=independent && logical_w>=64.f && logical_h>=64.f;
  // Front-end raster canvas is independent of the scene cameras. GetClientRect
  // being hooked is not evidence that the engine uses its spoofed dimensions:
  // LIVE traces show 1280x720 menu viewports alongside 2480x2480 eye targets.
  if (ui_layer_active()) {
    if (eye == 1 && t_ui_kind == 0) {
      unsigned cw = 0, ch = 0;
      if (!(source_w > 0.f && source_h > 0.f)) {
        if (eye_size(&cw, &ch)) {
          source_w = static_cast<float>(cw);
          source_h = static_cast<float>(ch);
        } else {
          source_w = rt_w;
          source_h = rt_h;
        }
      }
      // An early window-sized viewport uses the whole source. Once its raster
      // grows to eye size, the unchanged window UI occupies its lower-left.
      const bool embedded=logical && std::fabs(source_w-static_cast<float>(ew))<=2.f &&
          std::fabs(source_h-static_cast<float>(eh))<=2.f;
      const auto fitted = ui_canvas::place_rect(0.f,embedded ? source_h-logical_h : 0.f,
          embedded ? logical_w : source_w,embedded ? logical_h : source_h,
          rt_w,rt_h,rt_w*.5f,rt_h*.5f,1.f);
      if (fitted.scale*source_w>32767.f || fitted.scale*source_h>32767.f ||
          fitted.x<-32768.f || fitted.y<-32768.f || fitted.x>32767.f || fitted.y>32767.f) return false;
      *sx = *sy = fitted.scale;
      *ox = fitted.x;
      *oy = fitted.y;
      return true;
    }
    *sx = *sy = 0.01f;
    *ox = -4.f * rt_w;
    *oy = -4.f * rt_h;
    return true;
  }
  const int e = eye - 1;  // main view = left eye, second view = right eye
  float t[4];
  float ipd = 0.064f;
  {
    std::lock_guard<std::mutex> lock(g_ui_tan_mutex);
    if (!g_ui_tan_valid[e]) {
      return false;
    }
    std::memcpy(t, g_ui_tan[e], sizeof(t));
    ipd = g_ui_ipd;
  }
  const float l = t[0], r = t[1], up = t[2], dn = t[3];
  if (!(r - l > 0.1f && up - dn > 0.1f)) {
    return false;
  }
  // The 14:25 capture and offsets(437,-14 x.85) locate a window-sized UI in
  // the lower-left of the eye raster. Fit that logical rectangle, rather than
  // the entire eye viewport. Origin and size must be corrected together.
  const float canvas_w=logical ? logical_w : rt_w, canvas_h=logical ? logical_h : rt_h;
  const float s = static_cast<float>(g_ui_size_pct.load(std::memory_order_relaxed)) * 0.01f;
  const int depth_mm = g_ui_depth_mm.load(std::memory_order_relaxed);
  const float c = depth_mm > 0 ? 0.5f * ipd / (static_cast<float>(depth_mm) * 1e-3f) : 0.f;
  const float cx = e == 0 ? c : -c;  // left eye looks right toward the UI, right eye left
  const float cy = std::tan(static_cast<float>(g_ui_height_cdeg.load(std::memory_order_relaxed)) * 0.01f * 0.0174533f);
  // Centre of the UI = the forward direction (with the convergence and height offsets) in target pixels; the UI
  // region is s x the canvas fitted to the eye (s x the eye itself when the canvas is the eye).
  const float centre_x = rt_w * (cx - l) / (r - l);
  const float centre_y = rt_h * (up - cy) / (up - dn);
  const auto mapped=ui_canvas::place_rect(0.f,logical ? rt_h-canvas_h : 0.f,canvas_w,canvas_h,
      rt_w,rt_h,centre_x,centre_y,s);
  if (mapped.scale*rt_w>32767.f || mapped.scale*rt_h>32767.f ||
      mapped.x<-32768.f || mapped.y<-32768.f || mapped.x>32767.f || mapped.y>32767.f) return false;
  *sx = *sy = mapped.scale;
  *ox = mapped.x;
  *oy = mapped.y;
  return true;
}

void poll() {
  dump_code();
  log_view_diff();
  log_ui_probe();
  log_eyesize();
  int mode = 0;
  int view = 0;
  if (!g_control_path.empty()) {
    char text[256] = {};
    if (settings_file::read(g_control_path, text, sizeof(text))) {
      g_want_kick.store(std::strstr(text, "kick") != nullptr);
      ui_layout::set_enabled(std::strstr(text, "uilayoutquery") != nullptr && std::strstr(text, "nouilayout") == nullptr);
      ui_cursor::set_enabled(std::strstr(text, "nocursorfix") == nullptr);
      menu_pointer::g_enabled.store(std::strstr(text, "nolaser") == nullptr, std::memory_order_relaxed);
      ui_piece::set_enabled(std::strstr(text, "nouipiece") == nullptr);
      g_fp_proj_nofile.store(std::strstr(text, "nofpproj") == nullptr);
      g_fp_depth_file.store(std::strstr(text, "fpdepth") != nullptr);
      g_fp_occlude_off.store(std::strstr(text, "nofpocclude") != nullptr);
      g_pose_write.store(std::strstr(text, "posewrite") != nullptr);
      g_fp_occlude_file.store(std::strstr(text, "fpocclude") != nullptr && std::strstr(text, "nofpocclude") == nullptr);
      {
        const char* c = std::strstr(text, "create");
        g_want_create.store(c != nullptr);
        g_want_offscreen.store(c != nullptr && std::strstr(text, "offscreen") != nullptr);
        g_shift_prev.store(std::strstr(text, "noprev") == nullptr);
        g_priv_mode.store(std::strstr(text, "slotshift") == nullptr);
        g_head.store(c != nullptr && std::strstr(text, "head") != nullptr);
        g_scene_gate_on.store(std::strstr(text, "anyscene") == nullptr);
        g_load_settle.store(std::strstr(text, "nosettle") == nullptr);  // on by default (settle after loads)
        {
          const bool menu_stereo = std::strstr(text, "nomenustereo") == nullptr;
          if (g_menu_stereo.exchange(menu_stereo) != menu_stereo) {
            log_line("front end: 3D main menu %s", menu_stereo ? "in stereo" : "OFF (flat screen until gameplay)");
          }
        }
        {
          const char* sh = std::strstr(text, "sharehooks");
          char* end = nullptr;
          const long k = sh != nullptr ? std::strtol(sh + 10, &end, 10) : 0;
          // "sharehooks" alone or >= 12: all slots; 1..11: that slot only (in viewslot order)
          g_share_pick.store(sh == nullptr ? 0 : (end == sh + 10 || k >= 12 || k < 1) ? 99 : static_cast<int>(k));
          g_share_hooks.store(sh != nullptr);
        g_eye_ui.store(std::strstr(text, "noeyeui") == nullptr);  // on by default
        // Off by default ("eyefx" turns it on). [AUTHOR] + capture 2026-10-08 08:12: shared, the right eye's highlights
        // were right and the LEFT eye's moved to the right eye's positions -- the passes' targets are shared between
        // the views, so the view that runs last wins (per-owner targets, OPEN which).
        g_eye_fx.store(std::strstr(text, "eyefx") != nullptr && std::strstr(text, "noeyefx") == nullptr);
        g_ui_viewport_write.store(false);
        {  // "eyesize W H [both]": per-view render size (see OutSize_hook); absent = engine size
          const char* es = std::strstr(text, "eyesize");
          unsigned long w = 0, hh = 0;
          if (es != nullptr) {
            char* e1 = nullptr;
            char* e2 = nullptr;
            w = std::strtoul(es + 7, &e1, 10);
            hh = e1 != nullptr ? std::strtoul(e1, &e2, 10) : 0;
          }
          const bool ok = w >= 256 && w <= 8192 && hh >= 256 && hh <= 8192;
          g_eyesize_w.store(ok ? static_cast<std::uint32_t>(w & ~7ul) : 0);
          g_eyesize_h.store(ok ? static_cast<std::uint32_t>(hh & ~7ul) : 0);
          g_eyesize_both.store(ok && es != nullptr && std::strstr(es, "both") != nullptr);
          g_outsize_render.store(std::strstr(text, "outsizerender") != nullptr);
        }
        }
        {
          const char* af = std::strstr(text, "autofov");
          g_lens_projection.store(false);  // rejected in headset; pure math tests did not validate Snowdrop
          g_auto_fov.store(af != nullptr || g_lens_projection.load());
          const long pct = af != nullptr ? std::strtol(af + 7, nullptr, 10) : 0;
          g_auto_margin_pct.store(pct >= 0 && pct <= 40 ? static_cast<int>(pct) : 0);
          g_eye_shift.store(std::strstr(text, "eyeshift") != nullptr);
          {
            const char* nw = std::strstr(text, "narrow");
            const long n = nw != nullptr ? std::strtol(nw + 6, nullptr, 10) : 100;
            g_narrow_pct.store(n >= 40 && n <= 100 ? static_cast<int>(n) : 100);
            g_outer_edge.store(std::strstr(text, "outeredge") != nullptr);
            const char* vc = std::strstr(text, " vcrop ");
            const long v = vc != nullptr ? std::strtol(vc + 7, nullptr, 10) : 100;
            g_vcrop_pct.store(v >= 60 && v <= 100 ? static_cast<int>(v) : 100);
          }
          g_sym_proj.store(std::strstr(text, "symproj") != nullptr);
          g_shadow_fix.store(std::strstr(text, "shadowfix") != nullptr);
          g_diag_probes.store(std::strstr(text, "diagprobes") != nullptr);
          g_flat_follow.store(std::strstr(text, "flatfollow") != nullptr);
          g_shadow_union.store(std::strstr(text, "shadowunion") != nullptr);
          g_lod_fov.store(std::strstr(text, "lodfov") != nullptr);
          g_lod_tex_game.store(std::strstr(text, "lodtexgame") != nullptr);
          g_lod_probe.store(std::strstr(text, "lodprobe") != nullptr && std::strstr(text, "nolodprobe") == nullptr);
          g_qt_probe.store(std::strstr(text, "qtprobe") != nullptr && std::strstr(text, "noqtprobe") == nullptr);
          g_spot_probe.store(std::strstr(text, "spotprobe") != nullptr && std::strstr(text, "nospotprobe") == nullptr);
          {
            const char* lk = std::strstr(text, "lodkeep");
            const long k = lk != nullptr ? std::strtol(lk + 7, nullptr, 10) : 0;
            g_lod_keep.store(k >= 0 && k <= 15 ? static_cast<int>(k) : 0);
          }
          {
            const char* lr = std::strstr(text, "lodref");
            const double d = lr != nullptr ? std::strtod(lr + 6, nullptr) : 0.0;
            g_lod_ref_mdeg.store(d >= 30.0 && d <= 150.0 ? static_cast<int>(d * 1000.0) : 0);
          }
          g_level.store(std::strstr(text, "nolevel") == nullptr);  // on by default
          g_vegsync.store(std::strstr(text, "novegsync") == nullptr);  // on by default
          g_fx_bits.store(std::strstr(text, "nofxbits") == nullptr);  // on by default: VFX in the right eye (ParticleCull_hook)
          {
            const char* js = std::strstr(text, "jittersync");  // "nojittersync" (older files) also reads as off
            g_jitter_sync.store(js != nullptr && !(js - text >= 2 && js[-2] == 'n' && js[-1] == 'o'));
          }
          g_zero_sep.store(std::strstr(text, "zerosep") != nullptr);
          {
            const char* ud = std::strstr(text, "uidepth");
            const double m = ud != nullptr ? std::strtod(ud + 7, nullptr) : 2.0;
            g_ui_depth_mm.store(m > 0.2 && m < 100.0 ? static_cast<int>(m * 1000.0) : 0);
          }
          {
            g_ui_place.store(std::strstr(text, "nouiplace") == nullptr);  // on by default
            // Off by default since 13:2x: reprojecting the second view's scan composite shifts its own picture too
            // (black bar); "reproj" turns it back on.
            g_scan_reproj.store(std::strstr(text, " reproj") != nullptr);
            camera::set_scan_shift(std::strstr(text, "scanshift") != nullptr && std::strstr(text, "noscanshift") == nullptr);
            camera::set_panel_cover(std::strstr(text, "nopanelcover") == nullptr);  // on by default (2026-11-12)
            camera::set_scan_mask(std::strstr(text, "scanmask") != nullptr && std::strstr(text, "noscanmask") == nullptr);  // opt-in
            // The hook-registry repair makes this dormant experiment reachable.
            // Do not silently arm texture replacement while collecting scanner evidence.
            camera::set_scan_overlay(std::strstr(text, " scanoverlay") != nullptr && std::strstr(text, "noscanoverlay") == nullptr);
            camera::set_fx_stretch(std::strstr(text, "fxstretch") != nullptr);
            const char* ux = std::strstr(text, "uimapx");
            g_ui_extra_mask.store(ux != nullptr ? static_cast<int>(std::strtol(ux + 6, nullptr, 10)) & 3 : 3);
            const char* up = std::strstr(text, "uiplace");
            char* end = nullptr;
            const double size = up != nullptr ? std::strtod(up + 7, &end) : 85.0;
            const double height = up != nullptr && end != nullptr ? std::strtod(end, nullptr) : 0.0;
            g_ui_size_pct.store(size >= 30.0 && size <= 120.0 ? static_cast<int>(size + 0.5) : 85);
            g_ui_height_cdeg.store(height >= -30.0 && height <= 30.0 ? static_cast<int>(height * 100.0) : 0);
          }
          {
            const bool off = std::strstr(text, "nomenupanel") != nullptr;
            g_menu_panel.store(!off);
            const char* mp = off ? nullptr : std::strstr(text, "menupanel");
            double v[3] = {2.0, 1.8, 0.0};
            if (mp != nullptr) {
              const char* cur = mp + 9;
              for (double& x : v) {
                char* end = nullptr;
                const double n = std::strtod(cur, &end);
                if (end == cur) {
                  break;
                }
                x = n;
                cur = end;
              }
            }
            const int dist = static_cast<int>(std::clamp(v[0], 0.5, 10.0) * 1000.0);
            const int width = static_cast<int>(std::clamp(v[1], 0.3, 10.0) * 1000.0);
            const int height = static_cast<int>(std::clamp(v[2], -2.0, 2.0) * 1000.0);
            if (dist != g_menu_dist_mm.load() || height != g_menu_height_mm.load()) {
              xr_runtime::recentre_gui_panel();  // a new distance / height places the panel again in front of the head
            }
            g_menu_dist_mm.store(dist);
            g_menu_width_mm.store(width);
            g_menu_height_mm.store(height);
          }
        }
        {
          const char* cf = std::strstr(text, "cullfov");
          const double cdeg = cf != nullptr ? std::strtod(cf + 7, nullptr) : 0.0;
          g_cullfov_mdeg.store(cdeg > 20.0 && cdeg < 179.0 ? static_cast<int>(cdeg * 1000.0) : 0);
          const char* f = std::strstr(text, " fov");
          const double deg = f != nullptr ? std::strtod(f + 4, nullptr) : 0.0;  // past " fov"
          const char* vs = std::strstr(text, "viewscale");
          const double scale = vs != nullptr ? std::strtod(vs + 9, nullptr) : 100.0;
          g_view_scale_pct.store(static_cast<int>(std::clamp(scale, 70.0, 130.0)));
          g_fov_mdeg.store(deg > 20.0 && deg < 170.0 ? static_cast<int>(deg * 1000.0) : 0);
          const char* ox = std::strstr(text, "offx");
          const char* oy = std::strstr(text, "offy");
          g_offx_milli.store(ox != nullptr ? static_cast<int>(std::strtod(ox + 4, nullptr) * 1000.0) : 0);
          g_offy_milli.store(oy != nullptr ? static_cast<int>(std::strtod(oy + 4, nullptr) * 1000.0) : 0);
          g_off_set.store(ox != nullptr || oy != nullptr);
        }
        if (c != nullptr) {
          char* end = nullptr;
          const double m = std::strtod(c + 6, &end);
          if (end != c + 6 && m > -0.5 && m < 0.5) {
            g_create_sep_um.store(static_cast<int>(m * 1e6));
          }
        }
      }
      int sv = -1;
      int sep_um = 0;
      bool sw = false;
      int src = 0;
      if (std::strncmp(text, "second", 6) == 0) {
        char* end = nullptr;
        const long k = std::strtol(text + 6, &end, 10);
        if (end != text + 6 && k >= 0 && k < kMaxViews) {
          sv = static_cast<int>(k);
          const double m = std::strtod(end, nullptr);
          if (m > -0.5 && m < 0.5) {
            sep_um = static_cast<int>(m * 1e6);
          }
          sw = std::strstr(text, "world") != nullptr;
          const char* sp = std::strstr(text, "src");
          if (sp != nullptr) {
            src = static_cast<int>(std::strtol(sp + 3, nullptr, 10));
          }
        }
      }
      const int prev_sv = g_second_view.exchange(sv);
      const int prev_um = g_second_sep_um.exchange(sep_um);
      const bool prev_sw = g_second_world.exchange(sw);
      g_second_src.store(src);
      if (prev_sv != sv || prev_um != sep_um || prev_sw != sw) {
        log_line("SECOND -> view %d sep %.4f m world %d src %d", sv, sep_um * 1e-6, sw ? 1 : 0, src);
        if (sv >= 0) {
          start_heartbeat();
        }
      }
      if (std::strncmp(text, "mode", 4) == 0) {
        char* end = nullptr;
        const long n = std::strtol(text + 4, &end, 10);
        if (n >= 1 && n <= 4) {
          mode = static_cast<int>(n);
          const char* v = std::strstr(text, "view");
          if (v != nullptr) {
            const long k = std::strtol(v + 4, nullptr, 10);
            view = (k >= 0 && k < kMaxViews) ? static_cast<int>(k) : 0;
          }
        }
      }
    }
  }
  const int previous = g_want_mode.exchange(mode);
  g_want_view.store(view);
  if (previous != mode) {
    log_line("MODE -> %s%d view %d", mode == 0 ? "observe " : "mode ", mode, view);
    if (mode == 0) {
      restore_all();
    }
  }
  static ULONGLONG last = 0;
  const ULONGLONG now = GetTickCount64();
  if (now - last < 2000) {
    return;
  }
  last = now;
  const int count = g_view_count.load();
  char line[480] = {};
  size_t used = 0;
  for (int i = 0; i < count && used < sizeof(line) - 80; ++i) {
    std::uint32_t m = 0;
    std::uint32_t guard = 0;
    std::uint8_t active = 0;
    unsigned char* base = static_cast<unsigned char*>(g_views[i].view);
    read_u32(base + kModeOffset, &m);
    read_u32(base + kGuardOffset, &guard);
    read_u8(base + 0x24, &active);
    used += static_cast<size_t>(snprintf(line + used, sizeof(line) - used, " #%d renders=%llu stereo=%llu mode=%u guard=%u act=%u |", i,
                                         static_cast<unsigned long long>(g_views[i].renders.load()),
                                         static_cast<unsigned long long>(g_views[i].stereo_entries.load()), m, guard, active));
  }
  static ULONGLONG last_cam_log = 0;
  if (now - last_cam_log >= 10000) {
    last_cam_log = now;
    for (int i = 0; i < count; ++i) {
      const ViewInfo& v = g_views[i];
      {
        // Identity fields (pointers only, read-only): which scene/world and helper objects this view uses.
        static const std::uint32_t kFields[] = {0x10, 0x18, 0x1190, 0x1210, 0x1218, 0x1248, 0x49A8};
        char ptrs[256] = {};
        size_t used_p = 0;
        for (std::uint32_t off : kFields) {
          std::uint32_t lo = 0, hi = 0;
          unsigned char* b = static_cast<unsigned char*>(v.view) + off;
          read_u32(b, &lo);
          read_u32(b + 4, &hi);
          used_p += static_cast<size_t>(snprintf(ptrs + used_p, sizeof(ptrs) - used_p, " +%X=%08X%08X", off, hi, lo));
        }
        log_line("view #%d fields:%s", i, ptrs);
      }
      log_line("view #%d %p renders=%llu tid=%lu cams=%llu fovY=%.2f aspect=%.3f pos=(%.1f,%.1f,%.1f) act=%u",
               i, v.view, static_cast<unsigned long long>(v.renders.load()), static_cast<unsigned long>(v.render_tid.load()),
               static_cast<unsigned long long>(v.cams.load()), v.fovy_milli.load() / 1000.0, v.aspect_milli.load() / 1000.0,
               v.pos_x.load() / 100.0, v.pos_y.load() / 100.0, v.pos_z.load() / 100.0,
               [&]() { std::uint8_t a = 0; read_u8(static_cast<unsigned char*>(v.view) + 0x24, &a); return static_cast<unsigned>(a); }());
    }
    log_line("camera blocks created outside any Gfx_View::Render: %llu", static_cast<unsigned long long>(g_cams_unattributed.load()));
  }
  if (g_created != nullptr || g_want_create.load()) {
    std::uint8_t a24 = 0, a25 = 0, a26 = 0;
    if (g_created != nullptr) {
      read_u8(static_cast<unsigned char*>(g_created) + 0x24, &a24);
      read_u8(static_cast<unsigned char*>(g_created) + 0x25, &a25);
      read_u8(static_cast<unsigned char*>(g_created) + 0x26, &a26);
    }
    const int ci = g_created_index.load();
    log_line("head: fov=%.1f offx=%.3f offy=%.3f cullfov=%.1f (writes %llu) (game desc fov/aspect/offsets logged below)",
             g_fov_mdeg.load() * 1e-3, g_offx_milli.load() * 1e-3, g_offy_milli.load() * 1e-3, g_cullfov_mdeg.load() * 1e-3,
             static_cast<unsigned long long>(g_cullfov_writes.load()));
    if (g_created_main_cam != nullptr) {
      float f[8] = {};
      read_bytes(static_cast<unsigned char*>(g_created_main_cam) + 0xF8, f, sizeof(f));
      log_line("head: game camera desc +F8..+114 = %.4f %.4f %.4f(%.1fdeg) %.4f %.4f %.4f %.4f %.4f", f[0], f[1], f[2], f[2] * 57.2958f,
               f[3], f[4], f[5], f[6], f[7]);
    }
    log_line("head: on=%d frames=%llu nopose=%llu ref=%d applied=%llu proj m0=%.4f m5=%.4f m8=%.4f m9=%.4f", g_head.load() ? 1 : 0,
             static_cast<unsigned long long>(g_head_frames.load()), static_cast<unsigned long long>(g_head_nopose.load()),
             g_head_ref_valid ? 1 : 0, static_cast<unsigned long long>(g_applied_count), g_proj_m0.load(), g_proj_m5.load(),
             g_proj_m8.load(), g_proj_m9.load());
    log_line("head: autofov=%d eyeshift=%d margin=%d%% narrow=%d%% vcrop=%d%%%s%s applied=%llu failed=%llu last fov=%.1f deg aspect=%.3f",
             g_auto_fov.load() ? 1 : 0, g_eye_shift.load() ? 1 : 0, g_auto_margin_pct.load(), g_narrow_pct.load(), g_vcrop_pct.load(),
             g_outer_edge.load() ? " outeredge" : "", g_sym_proj.load() ? " symproj" : "",
             static_cast<unsigned long long>(g_auto_applied.load()), static_cast<unsigned long long>(g_auto_failed.load()),
             g_auto_last_fov_mdeg.load() * 1e-3, g_auto_last_aspect_milli.load() * 1e-3);
    log_line("vegsync: %s, %d clock field(s) locked, %llu copies", g_vegsync.load() ? "on" : "off", g_clock_count.load(),
             static_cast<unsigned long long>(g_clock_copies.load()));
    log_jitter();
    log_shadow_slot();
    log_light_queue();
    log_shadow_pick();
    log_lod_fov();
    log_frame_status();
    log_lod_keep();
    log_lod_probe();
    log_qt_probe();
    log_spot_probe();
    log_line("flatfollow: %s writes=%llu restores=%llu failed=%llu fov=%.1f deg", g_flat_follow.load() ? "on" : "off",
             static_cast<unsigned long long>(g_ff_writes.load()), static_cast<unsigned long long>(g_ff_restores.load()),
             static_cast<unsigned long long>(g_ff_fail.load()), g_ff_fov_mdeg.load() * 1e-3);
    log_wind_sync();
    log_fx_bits();
    log_line("level: %s, world up %c%c, applied %llu skipped %llu", g_level.load() ? "on" : "off",
             g_up_axis < 0 ? '?' : (g_up_sign > 0 ? '+' : '-'), g_up_axis < 0 ? '?' : "xyz"[g_up_axis],
             static_cast<unsigned long long>(g_level_applied.load()), static_cast<unsigned long long>(g_level_skipped.load()));
    {
      AimRays a;
      {
        std::lock_guard<std::mutex> lock(g_aim_mutex);
        a = g_aim;
      }
      log_line("aim: frames=%llu cam pos=(%.2f,%.2f,%.2f) game fwd=(%.3f,%.3f,%.3f) elev=%.1f",
               static_cast<unsigned long long>(a.frames), a.cam_pos[0], a.cam_pos[1], a.cam_pos[2], a.cam_fwd[0],
               a.cam_fwd[1], a.cam_fwd[2], elevation_deg(a.cam_fwd));
      for (int h = 0; h < 2; ++h) {
        if (!a.valid[h]) {
          log_line("aim: %s --", h == 0 ? "left" : "right");
          continue;
        }
        const float c = std::clamp(dot3(a.dir[h], a.cam_fwd), -1.f, 1.f);
        log_line("aim: %s org=(%.2f,%.2f,%.2f) dir=(%.3f,%.3f,%.3f) elev=%.1f | vs game fwd: %.1f deg (heading %.1f, "
                 "elev %.1f) | org-cam %.2f m",
                 h == 0 ? "left" : "right", a.org[h][0], a.org[h][1], a.org[h][2], a.dir[h][0], a.dir[h][1], a.dir[h][2],
                 elevation_deg(a.dir[h]), std::acos(c) * 57.2958f, heading_delta_deg(a.dir[h], a.cam_fwd),
                 elevation_deg(a.dir[h]) - elevation_deg(a.cam_fwd),
                 std::sqrt((a.org[h][0] - a.cam_pos[0]) * (a.org[h][0] - a.cam_pos[0]) +
                           (a.org[h][1] - a.cam_pos[1]) * (a.org[h][1] - a.cam_pos[1]) +
                           (a.org[h][2] - a.cam_pos[2]) * (a.org[h][2] - a.cam_pos[2])));
      }
    }
    log_line("create: privcam=%d updates=%llu failed=%llu", g_priv_mode.load() ? 1 : 0,
             static_cast<unsigned long long>(g_priv_updates.load()), static_cast<unsigned long long>(g_priv_failed.load()));
    log_line("create: want=%d startframes=%llu view=%p #%d renders=%llu frames=%llu flags %u/%u/%u sep=%.4f applied=%llu failed=%llu",
             g_want_create.load() ? 1 : 0, static_cast<unsigned long long>(g_start_frames.load()), g_created, ci,
             static_cast<unsigned long long>(ci >= 0 ? g_views[ci].renders.load() : 0),
             static_cast<unsigned long long>(g_created_frames.load()), a24, a25, a26,
             g_create_sep_um.load() * 1e-6, static_cast<unsigned long long>(g_second_applied.load()),
             static_cast<unsigned long long>(g_second_failed.load()));
    log_line("create: scene gate %s (%s) stereo %s pauses=%llu resumes=%llu", g_scene_gameplay.load() ? "gameplay" : "menu",
             g_scene_gate_on.load() ? "params +0x46" : "anyscene", g_stereo_running.load() ? "running" : "paused",
             static_cast<unsigned long long>(g_stereo_pauses.load()), static_cast<unsigned long long>(g_stereo_resumes.load()));
    if (g_want_offscreen.load() && g_module_base != nullptr) {
      void* globals = nullptr;
      if (read_ptr(g_module_base + kTargetsGlobalRva, &globals) && globals != nullptr) {
        char ptrs[200] = {};
        size_t used_p = 0;
        for (std::uint32_t off = 0xC00; off <= 0xC40; off += 8) {
          void* q = nullptr;
          read_ptr(static_cast<unsigned char*>(globals) + off, &q);
          used_p += static_cast<size_t>(snprintf(ptrs + used_p, sizeof(ptrs) - used_p, " +%X=%p", off, q));
        }
        log_line("offscreen: sets=%llu globals=%p%s", static_cast<unsigned long long>(g_offscreen_sets.load()), globals, ptrs);
      }
    }
    {
      void* res = g_out_resource.load();
      char rdesc[160] = "none";
      if (res != nullptr) {
        D3D12_RESOURCE_DESC desc = {};
        if (describe(res, &desc)) {
          snprintf(rdesc, sizeof(rdesc), "%p dim=%d %llux%u depth=%u mips=%u fmt=%d flags=0x%X", res, static_cast<int>(desc.Dimension),
                   static_cast<unsigned long long>(desc.Width), desc.Height, desc.DepthOrArraySize, desc.MipLevels,
                   static_cast<int>(desc.Format), static_cast<unsigned>(desc.Flags));
        }
      }
      log_line("output: changes=%llu callbacks=%llu created=%llu texture=%p %ux%ux%u fmt=%u res@%X=%s | main texture=%p resvtable=%p",
               static_cast<unsigned long long>(g_out_changes.load()),
               static_cast<unsigned long long>(g_output_calls.load()), static_cast<unsigned long long>(g_output_created.load()),
               g_out_texture.load(), g_out_w.load(), g_out_h.load(), g_out_d.load(), g_out_fmt.load(), g_out_res_offset.load(), rdesc,
               g_main_texture.load(), resource_names::resource_vtable());
    }
  }
  const int sv = g_second_view.load();
  if (sv >= 0) {
    char cams[400] = {};
    size_t used_c = 0;
    const int show[2] = {g_second_src.load(), sv};
    for (int which : show) {
      if (which < 0 || which >= count) {
        continue;
      }
      unsigned char* v = static_cast<unsigned char*>(g_views[which].view);
      void* ring = nullptr;
      std::uint32_t idx = 0;
      float pos[3] = {};
      float wpf[3] = {};
      std::uint16_t cell[3] = {};
      void* world = nullptr;
      if (read_ptr(v + kFrameRing, &ring) && ring != nullptr && read_u32(v + kFrameIdxGame, &idx) && idx < 8) {
        unsigned char* sub = static_cast<unsigned char*>(ring) + static_cast<std::uintptr_t>(idx) * kFrameStride + kSubCams[0];
        read_bytes(sub + 0x30, pos, sizeof(pos));
        read_bytes(sub + 0xE4, wpf, sizeof(wpf));
        read_bytes(sub + 0xF0, cell, sizeof(cell));
      }
      read_ptr(v + kWorldOffset, &world);
      used_c += static_cast<size_t>(snprintf(cams + used_c, sizeof(cams) - used_c,
                                             " #%d world=%p slot0 pos=(%.3f,%.3f,%.3f) wp=(%.3f,%.3f,%.3f|%u,%u,%u) |", which, world,
                                             pos[0], pos[1], pos[2], wpf[0], wpf[1], wpf[2], cell[0], cell[1], cell[2]));
    }
    log_line("second: builds=%llu applied=%llu failed=%llu%s", static_cast<unsigned long long>(g_cam_builds.load()),
             static_cast<unsigned long long>(g_second_applied.load()), static_cast<unsigned long long>(g_second_failed.load()), cams);
  }
  log_line("views=%d captures=%llu faults=%llu:%s", count, static_cast<unsigned long long>(g_captures.load()),
           static_cast<unsigned long long>(g_faults.load()), line);
}

}  // namespace native_stereo
