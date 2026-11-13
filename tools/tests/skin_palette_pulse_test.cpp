#include "../../src/diagnostics/skin_palette_pulse_math.hpp"
#include <cstdio>
#include <limits>
int main() {
  float source[211][16]{},output[211][16]{};
  for(unsigned i=0;i<211;++i) for(unsigned j=0;j<16;++j) source[i][j]=static_cast<float>(i*16+j)/100;
  float saved[211][16]; std::memcpy(saved,source,sizeof(saved));
  for(float delta:{0.f,0.25f,-0.25f}) {
    if(!skin_palette_pulse::copy_right(output,source,delta)) return 1;
    for(unsigned i=0;i<211;++i) for(unsigned j=0;j<16;++j) {
      const float expected=source[i][j]+((i>=113 && i<=115 && j==13)?delta:0.f);
      if(output[i][j]!=expected || source[i][j]!=saved[i][j]) return 2;
    }
  }
  if(skin_palette_pulse::offset(2999)!=0 || skin_palette_pulse::offset(3000)!=0.25f ||
     skin_palette_pulse::offset(6000)!=0 || skin_palette_pulse::offset(9000)!=-0.25f ||
     skin_palette_pulse::offset(12000)!=0 || skin_palette_pulse::offset(20000)!=0) return 3;
  if(skin_palette_pulse::copy_right(source,source,0.25f) ||
     skin_palette_pulse::copy_right(output,source,0.251f)) return 4;
  source[114][0]=std::numeric_limits<float>::quiet_NaN();
  if(skin_palette_pulse::copy_right(output,source,0.25f)) return 5;
  std::puts("palette pulse: source immutable, only right-arm Y changes, phases and invalid inputs pass");
}
