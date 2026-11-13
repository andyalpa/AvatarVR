#include "../../src/camera/ui_canvas.hpp"
#include <cstdio>

int main() {
  // Compare monitor resolutions/aspects, including a non-wide window. The
  // source centre must land at the same panel centre and no edge may be cut.
  const float sizes[][2] = {{1280,720}, {1920,1080}, {2560,1440}, {800,600}, {720,1280}, {2480,2480}};
  for (const auto& size : sizes) {
    const auto f = ui_canvas::fit(size[0], size[1], 1920.f, 1080.f);
    if (std::fabs(f.x + size[0] * f.scale * 0.5f - 960.f) > .001f ||
        std::fabs(f.y + size[1] * f.scale * 0.5f - 540.f) > .001f ||
        f.x < -.001f || f.y < -.001f ||
        f.x + size[0] * f.scale > 1920.001f ||
        f.y + size[1] * f.scale > 1080.001f) return 1;
  }
  // The same normalized 16:9 layout remains identical after monitor resizing.
  const auto a = ui_canvas::fit(1280,720,1920,1080);
  const auto b = ui_canvas::fit(2560,1440,1920,1080);
  if (std::fabs(a.x + 1280 * .25f * a.scale - (b.x + 2560 * .25f * b.scale)) > .001f ||
      std::fabs(a.y + 720 * .8f * a.scale - (b.y + 1440 * .8f * b.scale)) > .001f) return 2;
  // Actual failing path: window pixels embedded at the bottom of a square
  // eye raster. Test centres and corners at different monitor/eye resolutions,
  // not just an ordinary origin-zero texture fit.
  const float windows[][2]={{1280,720},{1920,1080},{2560,1440}};
  const float eye_sizes[]={1760.f,2480.f,3200.f};
  for (const auto& window:windows) for (const float eye: eye_sizes) {
    const float origin_y=eye-window[1];
    const auto mapped=ui_canvas::place_rect(0,origin_y,window[0],window[1],eye,eye,eye*.6f,eye*.42f,.85f);
    if (std::fabs(mapped.x+window[0]*.5f*mapped.scale-eye*.6f)>.001f ||
        std::fabs(mapped.y+(origin_y+window[1]*.5f)*mapped.scale-eye*.42f)>.001f) return 3;
    const auto panel=ui_canvas::place_rect(0,origin_y,window[0],window[1],1920,1080,960,540,1);
    if (std::fabs(panel.x)> .001f || std::fabs(panel.y+origin_y*panel.scale)>.001f ||
        std::fabs(window[0]*panel.scale-1920)>.001f ||
        std::fabs(window[1]*panel.scale-1080)>.001f) return 4;
    // Half-resolution UI/blur targets must use half offsets, preserving their
    // normalized alignment with the full-resolution HUD.
    if (std::fabs((mapped.y*.5f+(origin_y+window[1]*.5f)*.5f*mapped.scale)/(eye*.5f)-.42f)>.00001f) return 5;
  }
  std::puts("UI canvas: centre, edge coverage and resize invariance passed");
  return 0;
}
