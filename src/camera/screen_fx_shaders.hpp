#pragma once
#include "scan_uv_shader.hpp"
namespace screen_fx {
// Captured HDR radial-blur whole-picture composite, not a general UI quad rule.
// scantrace_27076/pso_1890699050496_ps.cso; SHA256 bcccf802...557a530.
inline bool radial_picture(const void* ptr,size_t size) {
  constexpr std::uint32_t digest[]={0x4944b9ad,0x5f4289bc,0xeea92b0d,0x1ff37d54};
  if(!ptr||size!=2400) return false;
  const auto* b=static_cast<const unsigned char*>(ptr);
  if(std::memcmp(b,"DXBC",4)||std::memcmp(b+4,digest,16)) return false;
  const auto hash=scan_uv::checksum(b+20,size-20);
  return std::memcmp(b+4,hash.data(),16)==0;
}
}
