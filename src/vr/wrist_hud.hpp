#pragma once

// Wrist HUD (FEATURE-13 Phase 3, research/HUD_PIECES.md): the game's HUD is also drawn into an offscreen texture
// (camera.cpp), and rectangles of it (ammo, health) are shown as OpenXR quads on the hand without the weapon
// (xr_runtime.cpp). The bottom band of the in-view HUD can be cut away so those items are not shown twice.
// Settings live in scripts\AvatarVR_hud.txt; the menu edits the values through the pointers below (Present thread).
namespace wrist_hud {

struct Crop {
  float x, y, w, h;  // HUD texture fractions, origin top-left
};

struct Settings {
  bool on = false;           // master flag: HUD texture + wrist quads + band cut
  bool full = false;         // tuning: also show the whole HUD texture above the wrist
  bool native_arm = false;   // the game's own health and ammo widgets on the forearm, per eye; replaces these quads
  bool damage_fx_full = false;  // listed full-screen HUD widgets (AvatarVR_fullview_fx.txt) cover the lens view; needs native_arm
  bool arm_anchor = true;    // follow the drawn forearm; the controller is only the fallback (mounted, stale rig)
  bool face_viewer = true;   // wrist position with a readable face toward the headset
  bool hide_original = true;// remove the two source crop regions once valid wrist imagery is submitted
  float band = 0.f;          // opt-in bottom strip cut; keep originals until the wrist crops are checked
  float size_m = 0.10f;      // width of the health panel on the wrist, metres; ammo is scaled by its crop
  Crop ammo = {0.78f, 0.80f, 0.22f, 0.20f};    // piece #0 anchored bottom right
  Crop health = {0.30f, 0.84f, 0.40f, 0.16f};  // pieces #7/#12/#45/#63 anchored bottom centre
};

Settings* settings();  // menu rows bind to its fields
void load(const wchar_t* dir);
void poll();           // Present thread: saves AvatarVR_hud.txt when a value changed

// Snapshot for render threads and the OpenXR worker; briefly locks the published settings.
Settings current();
void image_ready(bool ready);  // bounded texture readback established nonempty RGB + alpha
void submitted(bool visible); // worker: successful xrEndFrame included wrist layers
bool replacing();             // valid image and recent wrist submission

}  // namespace wrist_hud
