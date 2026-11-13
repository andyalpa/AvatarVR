#pragma once
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

// Captured scanner PS only. See SCREEN_EFFECTS_20261023 and scan_shader_twin.py.
// DXBC checksum layout reference: AMD GPUOpen common-src-ShaderUtils/DX10/DXBCChecksum.cpp.
namespace scan_uv {
using U = std::uint32_t;
// Decorative plane only: centre on head-forward, widen its angular span.
// World highlights retain their separately computed source-mask coordinates.
inline bool plane_map(const float* l,const float* r,int eye,float width,float* out) {
  if((eye!=1&&eye!=2)||!std::isfinite(width)||width<1.f||width>2.f) return false;
  for(int i=0;i<4;++i) if(!std::isfinite(l[i])||!std::isfinite(r[i])) return false;
  const float xspan=std::fmax(l[1]-l[0],r[1]-r[0])*width;
  const float yspan=std::fmax(l[2]-l[3],r[2]-r[3]);
  if(xspan<0.1f||yspan<0.1f) return false;
  const float* e=eye==1?l:r;
  out[0]=(e[1]-e[0])/xspan;out[1]=(e[2]-e[3])/yspan;
  out[2]=0.5f+e[0]/xspan;out[3]=0.55f-e[2]/yspan;
  return true;
}
inline std::array<U,4> checksum(const unsigned char* bytes, size_t size) {
  std::array<U,4> state{0x67452301,0xefcdab89,0x98badcfe,0x10325476};
  const auto block = [&](const unsigned char* p) {
    U w[16]; std::memcpy(w,p,64);
    U a=state[0],b=state[1],c=state[2],d=state[3];
    constexpr int shifts[4][4]={{7,12,17,22},{5,9,14,20},{4,11,16,23},{6,10,15,21}};
    for (int i=0;i<64;++i) {
      U f=0; int g=0;
      if(i<16) {f=(b&c)|(~b&d);g=i;}
      else if(i<32) {f=(d&b)|(~d&c);g=(5*i+1)%16;}
      else if(i<48) {f=b^c^d;g=(3*i+5)%16;}
      else {f=c^(b|~d);g=(7*i)%16;}
      const U k=static_cast<U>(std::abs(std::sin(double(i+1)))*4294967296.0);
      const U n=b+std::rotl(a+f+w[g]+k,shifts[i/16][i%4]);
      a=d;d=c;c=b;b=n;
    }
    state[0]+=a;state[1]+=b;state[2]+=c;state[3]+=d;
  };
  const U bits=static_cast<U>(size*8);
  while(size>=64) {block(bytes);bytes+=64;size-=64;}
  std::array<unsigned char,64> end{};
  if(size>=56) {
    std::memcpy(end.data(),bytes,size);end[size]=128;block(end.data());end.fill(0);
    std::memcpy(end.data(),&bits,4);
  } else {
    std::memcpy(end.data(),&bits,4);std::memcpy(end.data()+4,bytes,size);end[4+size]=128;
  }
  const U last=(bits>>2)|1;std::memcpy(end.data()+60,&last,4);block(end.data());
  return state;
}
inline bool matches(const void* ptr,size_t size) {
  constexpr U digest[4]={0xb2945a0c,0xa04b4e4a,0x650799e7,0xb6f9de63};
  if(!ptr||size!=15176) return false;
  const auto* b=static_cast<const unsigned char*>(ptr);
  return std::memcmp(b,"DXBC",4)==0&&std::memcmp(b+4,digest,16)==0;
}
inline std::vector<unsigned char> patch(const void* ptr,size_t size,const float* uv,const float* plane=nullptr,bool stable=false,const float* pixels=nullptr) {
  if(stable&&!plane) return {};
  if(!matches(ptr,size)) return {};
  const auto* b=static_cast<const unsigned char*>(ptr);
  const auto original_hash=checksum(b+20,size-20);
  if(std::memcmp(b+4,original_hash.data(),16)) return {};
  for(int i=0;i<4;++i) if(!std::isfinite(uv[i])) return {};
  if(plane) for(int i=0;i<4;++i) if(!std::isfinite(plane[i])) return {};
  if(pixels&&(!std::isfinite(pixels[0])||!std::isfinite(pixels[1])||pixels[0]<16.f||pixels[1]<16.f)) return {};
  U words[3654];std::memcpy(words,b+560,sizeof(words));
  if((words[138]&2047)!=104||words[139]!=12) return {};
  words[139]=stable?15:plane?14:13;
  for(int k:{388,1127,1426,1928}) {
    if(((words[k]>>12)&255)!=1||words[k+1]!=4) return {};
    words[k]&=~(255u<<12);words[k+1]=(plane&&k!=388)?13:12;
  }
  if(words[1486]!=0x80101ea6||words[1488]!=4) return {};
  words[1486]&=~(255u<<12);words[1488]=plane?13:12;
  if(plane) {
    // Later radial-distortion centre reconstructs UV from SV_Position, rather
    // than v4. Give it the same head-centred plane UV; keep depth/scene reads.
    constexpr U radial[13]={0x0d000032,0x001000c2,1,0x00101406,8,
      0x00308ea6,2,0x8c0,71,0x00308406,2,0x8c0,72};
    if(std::memcmp(words+3133,radial,sizeof(radial))) return {};
    const U move[5]={0x05000036,0x001000c2,1,0x00100ea6,13}; // mov r1.zw,r13.zzzw
    std::memcpy(words+3133,move,sizeof(move));
    for(int i=3138;i<3146;++i) words[i]=0x0100003a; // nop, retain all instruction offsets
  }
  if(stable) {
    // Isolate scene refraction/blur from the scanner's tint and highlight mask.
    // Cache unwarped scene UV before any colour sample overwrites r2.
    for(int k:{347,447,469,545,567,663,685,1380,2044,3322,3344,3417}) {
      if(words[k]!=0x0d000048 || !((words[k+6]==3&&words[k+7]==681)||(words[k+6]==2&&words[k+7]==680))) return {};
      words[k+3]=0x00100046;words[k+4]=14; // scene colour only, r14.xyxx
    }
  }
  const float px=pixels?1.f/pixels[0]:1.f,py=pixels?1.f/pixels[1]:1.f;
  const U mad[15]={0x0f000032,0x001000c2,12,pixels?0x00101406u:0x00101ea6u,pixels?8u:4u,0x4002,0,0,
    std::bit_cast<U>(uv[0]*px),std::bit_cast<U>(uv[1]*py),0x4002,0,0,
    std::bit_cast<U>(uv[2]),std::bit_cast<U>(uv[3])};
  U plane_mad[15]={0x0f000032,0x001000c2,13,0x00101ea6,4,0x4002,0,0,0,0,0x4002,0,0,0,0};
  if(plane) {
    plane_mad[3]=pixels?0x00101406u:0x00101ea6u;plane_mad[4]=pixels?8u:4u;
    plane_mad[8]=std::bit_cast<U>(plane[0]*px);plane_mad[9]=std::bit_cast<U>(plane[1]*py);
    plane_mad[13]=std::bit_cast<U>(plane[2]);plane_mad[14]=std::bit_cast<U>(plane[3]);
  }
  const U prefix=plane?30:15,added=prefix+(stable?5:0);
  std::vector<unsigned char> out(size+added*4);
  std::memcpy(out.data(),b,560);
  words[1]=3654+added;
  std::memcpy(out.data()+560,words,140*4);
  std::memcpy(out.data()+560+140*4,mad,sizeof(mad));
  if(plane) std::memcpy(out.data()+560+155*4,plane_mad,sizeof(plane_mad));
  if(stable) {
    std::memcpy(out.data()+560+(140+prefix)*4,words+140,(347-140)*4);
    const U cache[5]={0x05000036,0x00100032,14,0x00100046,2}; // mov r14.xy,r2.xyxx
    std::memcpy(out.data()+560+(347+prefix)*4,cache,sizeof(cache));
    std::memcpy(out.data()+560+(347+added)*4,words+347,(3654-347)*4);
  } else std::memcpy(out.data()+560+(140+added)*4,words+140,(3654-140)*4);
  const U total=static_cast<U>(out.size()),chunk=(3654+added)*4;
  std::memcpy(out.data()+24,&total,4);std::memcpy(out.data()+556,&chunk,4);
  const auto hash=checksum(out.data()+20,out.size()-20);std::memcpy(out.data()+4,hash.data(),16);
  return out;
}
}
