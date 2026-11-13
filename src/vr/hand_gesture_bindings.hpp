#pragma once
#include <string>
#include <vector>

namespace hand_gestures {
enum SensorId { kSenseGrip,kSenseTrigger,kSenseIndexTouch,kSenseThumbTouch,kSenseThumbPress,kSensorCount };
struct SensorBinding { SensorId id; std::string path; };
inline std::vector<SensorBinding> sensor_bindings(const std::string& name) {
  const bool touch=name=="/interaction_profiles/oculus/touch_controller" ||
    name=="/interaction_profiles/facebook/touch_controller_pro" || name=="/interaction_profiles/meta/touch_controller_plus";
  const bool index=name=="/interaction_profiles/valve/index_controller";
  const bool pico=name=="/interaction_profiles/bytedance/pico4_controller";
  const bool simple=name=="/interaction_profiles/khr/simple_controller";
  const bool vive=name=="/interaction_profiles/htc/vive_controller";
  const bool wmr=name=="/interaction_profiles/microsoft/motion_controller";
  const bool hp=name=="/interaction_profiles/hp/mixed_reality_controller";
  std::vector<SensorBinding> out;
  if(!(touch || index || pico || simple || vive || wmr || hp)) return out;
  for(int h=0;h<2;++h) {
    const std::string prefix=h?"/user/hand/right/input/":"/user/hand/left/input/";
    auto bind=[&](SensorId id,const char* component) {out.push_back({id,prefix+component});};
    bind(kSenseTrigger,simple?"select/click":"trigger/value");
    if(!simple) bind(kSenseGrip,vive || wmr?"squeeze/click":"squeeze/value");
    if(touch || index || pico) bind(kSenseIndexTouch,"trigger/touch");
    if(touch || index || pico) bind(kSenseThumbTouch,"thumbstick/touch");
    if(touch) bind(kSenseThumbTouch,"thumbrest/touch");
    if(index || vive || wmr) bind(kSenseThumbTouch,"trackpad/touch");
    if(!simple) bind(kSenseThumbPress,vive?"trackpad/click":"thumbstick/click");
    if(touch || pico) {
      bind(kSenseThumbTouch,h?"a/touch":"x/touch"); bind(kSenseThumbTouch,h?"b/touch":"y/touch");
      bind(kSenseThumbPress,h?"a/click":"x/click"); bind(kSenseThumbPress,h?"b/click":"y/click");
    } else if(index) {
      bind(kSenseThumbTouch,"a/touch"); bind(kSenseThumbTouch,"b/touch");
      bind(kSenseThumbPress,"a/click"); bind(kSenseThumbPress,"b/click");
    } else if(hp) {
      bind(kSenseThumbPress,h?"a/click":"x/click"); bind(kSenseThumbPress,h?"b/click":"y/click");
    }
  }
  return out;
}
} // namespace hand_gestures
