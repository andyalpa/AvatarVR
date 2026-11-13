#pragma once
#include "hand_gesture_policy.hpp"

namespace hand_gestures {
bool* enabled_flag(); // Present/menu thread; default off, not persisted
void on_present();   // publishes the menu setting; no OpenXR calls
void on_camera_post(); // local game thread: observe context, advance policy once
Frame latest();      // palette callbacks: immutable, freshness-checked copy
bool native_base_required(); // native animator is the base while gestures own a valid context
// Private palette only. Names must be the actual mesh names, never FP indices
// applied to a mounted rig. On any incomplete chain the whole hand is unchanged.
struct HandReference;
bool apply(float (*matrices)[16],unsigned count,int hand,const Pose& pose,const HandReference& reference);
// Palette workers, under the caller's rig lock: saved reference for this rig if one fits,
// otherwise, when allow_new (Pose::reference_new), freeze this frame and save it.
bool calibrate(HandReference& out,const float (*matrices)[16],unsigned count,const char* const* names,int hand,bool mounted,bool allow_new);
// Tuner "capture native pose": hand index requested (-1 none). capture() must see
// the palette BEFORE apply(); the first successful solve per request wins.
int capture_wanted();
void capture(const float (*matrices)[16],unsigned count,int hand,bool mounted,const HandReference& reference);
} // namespace hand_gestures
