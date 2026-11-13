#include "../../src/camera/ui_layout_math.hpp"
#include <cstdio>

int main() {
  // Monitor pixels must not survive in a logical->pixel->logical conversion.
  const float monitors[][2]={{1280,720},{1920,1080},{2560,1440},{800,600}};
  const float eyes[][2]={{1760,1760},{2480,2480},{3200,3200},{2032,2640}};
  for (const auto& monitor:monitors) for (const auto& eye:eyes) {
    float to_pixel[16]={}, to_viewport[16]={};
    // Reproduce the mismatch observed statically before applying the override.
    to_pixel[0]=monitor[0]; to_pixel[5]=monitor[1];
    to_viewport[0]=1/eye[0]; to_viewport[5]=1/eye[1];
    ui_layout::viewport_to_pixel(to_pixel,eye[0],eye[1]);
    ui_layout::pixel_to_viewport(to_viewport,eye[0],eye[1]);
    const float points[]={0.f,.1f,.5f,.9f,1.f};
    for (const float point: points) {
      if (std::fabs(point*to_pixel[0]*to_viewport[0]-point)>.00001f ||
          std::fabs(point*to_pixel[5]*to_viewport[5]-point)>.00001f) return 1;
    }
    // UI size queried in eye pixels maps back to full coverage, independently
    // of the monitor resolution and the eye resolution.
    if (std::fabs(eye[0]*to_viewport[0]-1)>.00001f ||
        std::fabs(eye[1]*to_viewport[5]-1)>.00001f) return 2;
  }
  if (ui_layout::dimension(0.f) || ui_layout::dimension(INFINITY) ||
      ui_layout::dimension(NAN) || ui_layout::dimension(20000.f)) return 3;
  std::puts("UI layout: reciprocal conversion, full coverage and resize invariance passed");
  return 0;
}
