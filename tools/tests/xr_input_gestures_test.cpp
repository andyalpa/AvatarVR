// Exercise the production XR adapter against a fake runtime: pad isolation,
// optional-binding failure, unsupported sensors and lifecycle invalidation.
#include "../../src/vr/xr_input.hpp"
#include <windows.h>
#include <array>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <cstdio>

namespace fake {
std::map<XrPath,std::string> paths;
std::map<XrAction,std::string> actions;
std::map<XrPath,std::vector<XrActionSuggestedBinding>> bindings;
std::map<XrSpace,int> spaces;
std::map<std::string,std::array<float,2>> values;
std::uintptr_t next=10;
bool focused=true,tracking=true,sensor_active=true,reject_sensors=false,fail_sensor_create=false;
unsigned retries=0;
XrPath path(const char* s) {
  for(const auto& p:paths) if(p.second==s) return p.first;
  const auto id=++next; paths[id]=s; return id;
}
bool sensor(XrAction a) {return actions[a].find("finger_")==0;}
float value(const XrActionStateGetInfo* i) {
  const int h=paths[i->subactionPath]=="/user/hand/right"?1:0;
  return values[actions[i->action]][h];
}
}
extern "C" {
XrResult XRAPI_CALL xrCreateActionSet(XrInstance,const XrActionSetCreateInfo*,XrActionSet* out) {*out=reinterpret_cast<XrActionSet>(++fake::next); return XR_SUCCESS;}
XrResult XRAPI_CALL xrDestroyActionSet(XrActionSet) {return XR_SUCCESS;}
XrResult XRAPI_CALL xrCreateAction(XrActionSet,const XrActionCreateInfo* i,XrAction* out) {
  if(fake::fail_sensor_create && std::strncmp(i->actionName,"finger_",7)==0) return XR_ERROR_RUNTIME_FAILURE;
  *out=reinterpret_cast<XrAction>(++fake::next); fake::actions[*out]=i->actionName; return XR_SUCCESS;
}
XrResult XRAPI_CALL xrCreateActionSpace(XrSession,const XrActionSpaceCreateInfo* i,XrSpace* out) {
  *out=reinterpret_cast<XrSpace>(++fake::next); fake::spaces[*out]=fake::paths[i->subactionPath]=="/user/hand/right"?1:0; return XR_SUCCESS;
}
XrResult XRAPI_CALL xrDestroySpace(XrSpace) {return XR_SUCCESS;}
XrResult XRAPI_CALL xrLocateSpace(XrSpace,XrSpace,XrTime,XrSpaceLocation* out) {
  out->pose.orientation.w=1; out->locationFlags=fake::tracking?(XR_SPACE_LOCATION_ORIENTATION_VALID_BIT|XR_SPACE_LOCATION_POSITION_VALID_BIT):0; return XR_SUCCESS;
}
XrResult XRAPI_CALL xrStringToPath(XrInstance,const char* s,XrPath* out) {*out=fake::path(s); return XR_SUCCESS;}
XrResult XRAPI_CALL xrPathToString(XrInstance,XrPath p,uint32_t size,uint32_t* count,char* out) {
  *count=static_cast<uint32_t>(fake::paths[p].size()+1);
  if(size<*count) return XR_ERROR_SIZE_INSUFFICIENT;
  std::memcpy(out,fake::paths[p].c_str(),*count); return XR_SUCCESS;
}
XrResult XRAPI_CALL xrResultToString(XrInstance,XrResult,char* out) {std::strcpy(out,"fake result"); return XR_SUCCESS;}
XrResult XRAPI_CALL xrSuggestInteractionProfileBindings(XrInstance,const XrInteractionProfileSuggestedBinding* i) {
  for(unsigned j=0;j<i->countSuggestedBindings;++j) if(fake::reject_sensors && fake::sensor(i->suggestedBindings[j].action)) {++fake::retries; return XR_ERROR_PATH_UNSUPPORTED;}
  fake::bindings[i->interactionProfile]={i->suggestedBindings,i->suggestedBindings+i->countSuggestedBindings}; return XR_SUCCESS;
}
XrResult XRAPI_CALL xrAttachSessionActionSets(XrSession,const XrSessionActionSetsAttachInfo*) {return XR_SUCCESS;}
XrResult XRAPI_CALL xrSyncActions(XrSession,const XrActionsSyncInfo*) {return fake::focused?XR_SUCCESS:XR_SESSION_NOT_FOCUSED;}
XrResult XRAPI_CALL xrGetCurrentInteractionProfile(XrSession,XrPath,XrInteractionProfileState* out) {
  out->interactionProfile=fake::path("/interaction_profiles/oculus/touch_controller"); return XR_SUCCESS;
}
XrResult XRAPI_CALL xrEnumerateBoundSourcesForAction(XrSession,const XrBoundSourcesForActionEnumerateInfo* i,uint32_t size,uint32_t* count,XrPath* out) {
  std::vector<XrPath> paths;
  for(const auto& b:fake::bindings[fake::path("/interaction_profiles/oculus/touch_controller")]) if(b.action==i->action) paths.push_back(b.binding);
  *count=static_cast<uint32_t>(paths.size()); if(!size) return XR_SUCCESS;
  if(size<*count) return XR_ERROR_SIZE_INSUFFICIENT;
  std::copy(paths.begin(),paths.end(),out); return XR_SUCCESS;
}
XrResult XRAPI_CALL xrGetActionStateFloat(XrSession,const XrActionStateGetInfo* i,XrActionStateFloat* out) {
  out->isActive=!fake::sensor(i->action) || fake::sensor_active; out->currentState=fake::value(i); return XR_SUCCESS;
}
XrResult XRAPI_CALL xrGetActionStateBoolean(XrSession,const XrActionStateGetInfo* i,XrActionStateBoolean* out) {
  out->isActive=!fake::sensor(i->action) || fake::sensor_active; out->currentState=fake::value(i)>.5f; return XR_SUCCESS;
}
XrResult XRAPI_CALL xrGetActionStateVector2f(XrSession,const XrActionStateGetInfo*,XrActionStateVector2f* out) {out->isActive=true; return XR_SUCCESS;}
XrResult XRAPI_CALL xrApplyHapticFeedback(XrSession,const XrHapticActionInfo*,const XrHapticBaseHeader*) {return XR_SUCCESS;}
XrResult XRAPI_CALL xrStopHapticFeedback(XrSession,const XrHapticActionInfo*) {return XR_SUCCESS;}
}
#include "../../src/vr/xr_input.cpp"

int main() {
  int errors=0;
  auto check=[&](bool ok,const char* why) {if(!ok) {++errors; std::printf("FAIL: %s\n",why);}};
  const auto instance=reinterpret_cast<XrInstance>(1); const auto session=reinterpret_cast<XrSession>(2);
  auto start=[&] {
    xr_input::create(instance,session,{}); xr_input::set_menu_open(false); xr_input::on_profile_changed(session);
  };
  auto locate=[&] {head_pose::Snapshot pose; xr_input::locate_hands(XR_NULL_HANDLE,1,&pose);};
  fake::values["pad_left_trigger"]={.7f,.7f}; fake::values["pad_right_trigger"]={.8f,.8f};
  fake::values["pad_left_bumper"]={1,1}; fake::values["pad_a"]={1,1};
  fake::values["finger_squeeze"]={.35f,.65f}; fake::values["finger_trigger"]={.2f,.4f};
  fake::values["finger_index_touch"]={1,0}; fake::values["finger_thumb_touch"]={0,1};
  start(); xr_input::sync(session);
  hand_gestures::Input input; xr_input::PadState raw;
  check(!xr_input::sensors(&input),"sensor frame waits for matching pose location"); locate();
  check(xr_input::sensors(&input) && input.hand[0].tracked && input.hand[1].tracked,"matched tracked sensor snapshot");
  check(input.hand[0].grip.value==.35f && input.hand[1].grip.value==.65f,"per-hand analog sensor values");
  check(input.hand[0].index_touch.value==1 && input.hand[1].thumb_touch.value==1,"touch signals independent by hand");
  check(xr_input::controls(&raw) && raw.lt==.7f && raw.rt==.8f && (raw.buttons&XINPUT_GAMEPAD_A),"sensor actions do not consume/change pad state");
  xr_input::on_profile_changed(session); check(!xr_input::sensors(&input),"profile change invalidates prior sensor frame");
  xr_input::sync(session); locate();
  fake::tracking=false; xr_input::sync(session); locate(); check(xr_input::sensors(&input) && !input.hand[0].tracked,"tracking loss visible to policy");
  fake::tracking=true; fake::focused=false; xr_input::sync(session); check(!xr_input::sensors(&input),"focus loss invalidates immediately");
  fake::focused=true; xr_input::set_menu_open(true); xr_input::sync(session); locate();
  check(!xr_input::sensors(&input) && xr_input::controls(&raw) && raw.lt==.7f,"menu blocks gestures while raw pad remains available");
  xr_input::set_menu_open(false); xr_input::sync(session); locate(); Sleep(270);
  check(!xr_input::sensors(&input),"stale sensor frame rejected");
  xr_input::destroy(); check(!xr_input::sensors(&input),"session destroy clears frame");
  fake::reject_sensors=true; fake::sensor_active=false; start(); xr_input::sync(session); locate();
  check(fake::retries>0,"rejected optional paths trigger pad-only retry");
  check(xr_input::sensors(&input) && input.hand[0].grip.value==1 && input.hand[0].trigger.value==.7f,"active pad press/value fallback after rejected sensors");
  check(!input.hand[0].index_touch.active && !input.hand[0].thumb_touch.active,"fallback never invents lifted/touch sensors");
  check(xr_input::controls(&raw) && raw.lt==.7f,"existing pad survives sensor binding rejection");
  xr_input::destroy(); fake::fail_sensor_create=true; start(); xr_input::sync(session); locate();
  check(xr_input::sensors(&input) && input.hand[0].trigger.value==.7f,"pad fallback survives optional action creation failure");
  xr_input::destroy(); if(errors) return 1;
  std::puts("xr_input_gestures_test passed"); return 0;
}
