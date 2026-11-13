#include "../../src/camera/ui_cursor_math.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>
#include <initializer_list>
void check(bool ok) { if (!ok) std::abort(); }
bool near(float a,float b) { return std::fabs(a-b)<.002f; }
int main() {
  for (float eye:{1600.f,2480.f,2560.f,3200.f}) {
    for (auto size:{std::pair<float,float>{1280,720},{1920,1080},{960,960}}) {
      for (auto uv:{std::pair<float,float>{0,0},{.5f,.5f},{1,1},{.1f,.2f}}) {
        float p[]={uv.first*eye,uv.second*eye}, q[2]{};
        check(ui_cursor::layout_point(p,eye,eye,size.first,size.second,q));
        check(near(q[0],uv.first*size.first) && near(q[1],uv.second*size.second));
        // Same normalized cursor location remains invariant with eye resolution.
        check(near(q[0]/size.first,p[0]/eye) && near(q[1]/size.second,p[1]/eye));
      }
    }
  }
  float raw[]={278.584f,516.970f},q[2]{};
  check(ui_cursor::layout_point(raw,2560,2560,1280,720,q));
  check(near(q[0],139.292f) && near(720-q[1],574.602f));
  float bad[]={std::numeric_limits<float>::quiet_NaN(),0};
  check(!ui_cursor::layout_point(bad,2560,2560,1280,720,q));
  check(!ui_cursor::layout_point(raw,0,2560,1280,720,q));
  std::puts("Stick cursor: normalized position, resize invariance and invalid-input guards passed");
}
