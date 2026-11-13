#include "../../src/camera/scan_uv_shader.hpp"
#include "../../src/camera/screen_fx_shaders.hpp"
#include "../../src/camera/scanner_inputs.hpp"
#include "../../src/camera/scanner_geometry.hpp"
#include <d3d12.h>
#include <d3d12shader.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
using Microsoft::WRL::ComPtr;
std::vector<unsigned char> read(const char* path) {
  std::ifstream f(path,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};
}
int main(int argc,char** argv) {
  scanner_inputs::enabled.store(true);
  int geometry_key=0;
  scanner_geometry::reset(&geometry_key);
  D3D12_VERTEX_BUFFER_VIEW geometry_vb{0x123400,128,32};
  D3D12_INDEX_BUFFER_VIEW geometry_ib{0x567800,12,DXGI_FORMAT_R16_UINT};
  scanner_geometry::vertices(&geometry_key,0,1,&geometry_vb);
  scanner_geometry::indices(&geometry_key,&geometry_ib);
  auto geometry=scanner_geometry::snapshot(&geometry_key);
  if(!geometry.reset_seen||geometry.valid!=1||!geometry.index||geometry.vb[0].StrideInBytes!=32) return 40;
  scanner_geometry::reset(&geometry_key);
  geometry=scanner_geometry::snapshot(&geometry_key);
  if(!geometry.reset_seen||geometry.valid||geometry.index) return 41;
  scanner_geometry::vertices(&geometry_key,32,1,&geometry_vb);
  if(scanner_geometry::snapshot(&geometry_key).valid) return 42;
  scanner_inputs::note(0x5000,{0x234567,0,0,0x100000,4096});
  scanner_inputs::copy(0x6000,0x5000);
  if(scanner_inputs::get(0x6000).raw_va!=0x100000||scanner_inputs::get(0x6000).raw_bytes!=4096) return 43;
  scanner_inputs::View raw_observation{0x345678,0,0,0x100000,64};
  raw_observation.raw64[15]=0x3f800000;raw_observation.raw_saved=true;raw_observation.raw_present=123;
  raw_observation.heap_type=D3D12_HEAP_TYPE_UPLOAD;
  scanner_inputs::note(0x5000,raw_observation);scanner_inputs::copy(0x6000,0x5000);
  const auto raw_copied=scanner_inputs::get(0x6000);
  if(!raw_copied.raw_saved||raw_copied.raw64[15]!=0x3f800000||raw_copied.raw_present!=123||raw_copied.heap_type!=D3D12_HEAP_TYPE_UPLOAD) return 44;
  scanner_inputs::note(0x1000,{0x20000,256});scanner_inputs::copy(0x2000,0x1000);
  if(scanner_inputs::get(0x2000).address!=0x20000) return 29;
  scanner_inputs::note(0x1000,{});scanner_inputs::copy(0x2000,0x1000);
  if(scanner_inputs::get(0x2000).address) return 30;
  scanner_inputs::note(0x1000,{0x40000,512});
  scanner_inputs::note(0x3000,{0x123456,0,0x202001c});scanner_inputs::copy(0x4000,0x3000);
  if(scanner_inputs::get(0x4000).address!=0x123456||scanner_inputs::get(0x4000).bytes||scanner_inputs::get(0x4000).tag!=0x202001c) return 32;
  scanner_inputs::enabled.store(false);scanner_inputs::note(0x1000,{0x80000,256});
  if(scanner_inputs::get(0x1000).address!=0x40000||scanner_inputs::get(0x1000).bytes!=512) return 31;
  if(argc!=4&&argc!=5) {std::cerr<<"usage: scan_uv_shader_test PS.cso VS.cso rootsig.blob [boost_PS.cso]\n";return 2;}
  auto ps=read(argv[1]),vs=read(argv[2]),rs=read(argv[3]);
  const float uv[]={1.f,1.f,-0.187f,0.017f};
  auto twin=scan_uv::patch(ps.data(),ps.size(),uv);
  if(twin.empty()) return 3;
  const float l[]={-1.3f,0.9f,1.1f,-1.f},r[]={-0.9f,1.3f,1.1f,-1.f};
  float plane[4],right_plane[4];
  if(!scan_uv::plane_map(l,r,1,1.25f,plane)||!scan_uv::plane_map(l,r,2,1.25f,right_plane)) return 12;
  for(int e=0;e<2;++e) {
    const float* f=e?r:l;const float* m=e?right_plane:plane;
    const float cx=-f[0]/(f[1]-f[0]),cy=f[2]/(f[2]-f[3]);
    if(std::abs(cx*m[0]+m[2]-.5f)>1e-6f||std::abs(cy*m[1]+m[3]-.55f)>1e-6f) return 13;
    if(std::abs(m[0]-.8f)>1e-6f) return 14; // 25% wider at unchanged projection
  }
  auto wide=scan_uv::patch(ps.data(),ps.size(),uv,plane);
  if(wide.empty()||std::memcmp(twin.data()+1120,wide.data()+1120,60)) return 15; // mask MAD identical
  auto stable=scan_uv::patch(ps.data(),ps.size(),uv,plane,true);
  if(stable.empty()||std::memcmp(wide.data()+1120,stable.data()+1120,60)) return 19;
  const float pixels[]={2320.f,2464.f};
  auto screen=scan_uv::patch(ps.data(),ps.size(),uv,plane,false,pixels);
  if(screen.empty()) return 23;
  if(argc==5) {
    auto boost=read(argv[4]);
    if(!screen_fx::radial_picture(boost.data(),boost.size())||screen_fx::radial_picture(ps.data(),ps.size())) return 24;
    boost.back()^=1;
    if(screen_fx::radial_picture(boost.data(),boost.size())) return 25;
  }
  // Original corruption/unknown shader must never be patched.
  auto corrupt=ps;corrupt.back()^=1;
  if(!scan_uv::patch(corrupt.data(),corrupt.size(),uv).empty()) return 4;
  ComPtr<ID3DBlob> assembly;
  if(FAILED(D3DDisassemble(twin.data(),twin.size(),0,nullptr,&assembly))) return 5;
  std::string text(static_cast<char*>(assembly->GetBufferPointer()),assembly->GetBufferSize());
  if(text.find("sample_l r4.xyzw, r12.zwzz")==std::string::npos ||
     text.find("add r7.zw, -r12.zzzw")==std::string::npos ||
     text.find("sample_l r0.yzw, r2.xyxx, T3[681]")==std::string::npos) return 6;
  ComPtr<ID3DBlob> wide_assembly;
  if(FAILED(D3DDisassemble(wide.data(),wide.size(),0,nullptr,&wide_assembly))) return 16;
  std::string wide_text(static_cast<char*>(wide_assembly->GetBufferPointer()),wide_assembly->GetBufferSize());
  if(wide_text.find("sample_l r4.xyzw, r12.zwzz")==std::string::npos||
     wide_text.find("add r7.zw, -r13.zzzw")==std::string::npos||
     wide_text.find("mov r1.zw, r13.zzzw")==std::string::npos||
     wide_text.find("mad r1.zw, v8.xxxy")!=std::string::npos) return 17;
  std::ofstream out("scan_uv_test.twin.cso",std::ios::binary);out.write(reinterpret_cast<char*>(twin.data()),twin.size());out.close();
  ComPtr<ID3D12Device> dev;
  HRESULT hr=D3D12CreateDevice(nullptr,D3D_FEATURE_LEVEL_11_0,IID_PPV_ARGS(&dev));
  if(FAILED(hr)) {std::cerr<<"device "<<std::hex<<hr;return 7;}
  ComPtr<ID3D12RootSignature> root;
  hr=dev->CreateRootSignature(0,rs.data(),rs.size(),IID_PPV_ARGS(&root));
  if(FAILED(hr)) {std::cerr<<"root "<<std::hex<<hr;return 8;}
  ComPtr<ID3D12ShaderReflection> reflect;
  hr=D3DReflect(vs.data(),vs.size(),IID_PPV_ARGS(&reflect));
  if(FAILED(hr)) return 9;
  D3D12_SHADER_DESC sd{};reflect->GetDesc(&sd);
  std::vector<D3D12_INPUT_ELEMENT_DESC> inputs;
  for(UINT i=0;i<sd.InputParameters;++i) {
    D3D12_SIGNATURE_PARAMETER_DESC p{};reflect->GetInputParameterDesc(i,&p);
    if(p.SystemValueType!=D3D_NAME_UNDEFINED) continue;
    const UINT n=p.Mask>7?4:p.Mask>3?3:p.Mask>1?2:1;
    const DXGI_FORMAT ff[]={DXGI_FORMAT_R32_FLOAT,DXGI_FORMAT_R32G32_FLOAT,DXGI_FORMAT_R32G32B32_FLOAT,DXGI_FORMAT_R32G32B32A32_FLOAT};
    const DXGI_FORMAT uu[]={DXGI_FORMAT_R32_UINT,DXGI_FORMAT_R32G32_UINT,DXGI_FORMAT_R32G32B32_UINT,DXGI_FORMAT_R32G32B32A32_UINT};
    const DXGI_FORMAT ii[]={DXGI_FORMAT_R32_SINT,DXGI_FORMAT_R32G32_SINT,DXGI_FORMAT_R32G32B32_SINT,DXGI_FORMAT_R32G32B32A32_SINT};
    inputs.push_back({p.SemanticName,p.SemanticIndex,p.ComponentType==D3D_REGISTER_COMPONENT_UINT32?uu[n-1]:p.ComponentType==D3D_REGISTER_COMPONENT_SINT32?ii[n-1]:ff[n-1],0,D3D12_APPEND_ALIGNED_ELEMENT,D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,0});
  }
  D3D12_GRAPHICS_PIPELINE_STATE_DESC desc{};
  desc.pRootSignature=root.Get();desc.VS={vs.data(),vs.size()};
  desc.PS={ps.data(),ps.size()};desc.InputLayout={inputs.data(),static_cast<UINT>(inputs.size())};
  desc.SampleMask=UINT_MAX;desc.SampleDesc.Count=1;desc.PrimitiveTopologyType=D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
  desc.NumRenderTargets=1;desc.RTVFormats[0]=DXGI_FORMAT_R11G11B10_FLOAT;
  desc.RasterizerState.FillMode=D3D12_FILL_MODE_SOLID;desc.RasterizerState.CullMode=D3D12_CULL_MODE_NONE;desc.RasterizerState.DepthClipEnable=TRUE;
  desc.DepthStencilState.DepthFunc=D3D12_COMPARISON_FUNC_ALWAYS;
  desc.DepthStencilState.FrontFace.StencilFunc=desc.DepthStencilState.BackFace.StencilFunc=D3D12_COMPARISON_FUNC_ALWAYS;
  desc.DepthStencilState.FrontFace.StencilFailOp=desc.DepthStencilState.FrontFace.StencilDepthFailOp=desc.DepthStencilState.FrontFace.StencilPassOp=D3D12_STENCIL_OP_KEEP;
  desc.DepthStencilState.BackFace=desc.DepthStencilState.FrontFace;
  auto& blend=desc.BlendState.RenderTarget[0];blend.RenderTargetWriteMask=15;
  blend.SrcBlend=blend.SrcBlendAlpha=D3D12_BLEND_ONE;blend.DestBlend=blend.DestBlendAlpha=D3D12_BLEND_INV_SRC_ALPHA;
  blend.BlendOp=blend.BlendOpAlpha=D3D12_BLEND_OP_ADD;blend.LogicOp=D3D12_LOGIC_OP_NOOP;
  ComPtr<ID3D12PipelineState> original,patched;
  hr=dev->CreateGraphicsPipelineState(&desc,IID_PPV_ARGS(&original));
  if(FAILED(hr)) {std::cerr<<"original PSO "<<std::hex<<hr;return 10;}
  desc.PS={twin.data(),twin.size()};
  hr=dev->CreateGraphicsPipelineState(&desc,IID_PPV_ARGS(&patched));
  if(FAILED(hr)) {std::cerr<<"twin PSO "<<std::hex<<hr;return 11;}
  ComPtr<ID3D12PipelineState> widened;
  desc.PS={wide.data(),wide.size()};
  hr=dev->CreateGraphicsPipelineState(&desc,IID_PPV_ARGS(&widened));
  if(FAILED(hr)) {std::cerr<<"wide PSO "<<std::hex<<hr;return 18;}
  ComPtr<ID3D12PipelineState> stable_pso;
  desc.PS={stable.data(),stable.size()};
  hr=dev->CreateGraphicsPipelineState(&desc,IID_PPV_ARGS(&stable_pso));
  if(FAILED(hr)) {std::cerr<<"stable PSO "<<std::hex<<hr;return 20;}
  ComPtr<ID3DBlob> stable_asm;
  if(FAILED(D3DDisassemble(stable.data(),stable.size(),0,nullptr,&stable_asm))) return 21;
  std::string stable_text(static_cast<char*>(stable_asm->GetBufferPointer()),stable_asm->GetBufferSize());
  if(stable_text.find("mov r14.xy, r2.xyxx")==std::string::npos||
     stable_text.find("sample_l r4.xyzw, r12.zwzz")==std::string::npos||
     stable_text.find("ld r7.x, r7.xyzw, T0[640]")==std::string::npos) return 22;
  std::ofstream stable_out("scan_uv_test.stable.cso",std::ios::binary);
  stable_out.write(reinterpret_cast<char*>(stable.data()),stable.size());
  std::ofstream wide_out("scan_uv_test.wide.cso",std::ios::binary);
  wide_out.write(reinterpret_cast<char*>(wide.data()),wide.size());
  ComPtr<ID3DBlob> screen_asm;
  if(FAILED(D3DDisassemble(screen.data(),screen.size(),0,nullptr,&screen_asm))) return 26;
  std::string screen_text(static_cast<char*>(screen_asm->GetBufferPointer()),screen_asm->GetBufferSize());
  if(screen_text.find("mad r12.zw, v8.xxxy")==std::string::npos||screen_text.find("mad r13.zw, v8.xxxy")==std::string::npos||
     screen_text.find("sample_l r4.xyzw, r12.zwzz")==std::string::npos) return 27;
  ComPtr<ID3D12PipelineState> screen_pso;
  desc.PS={screen.data(),screen.size()};
  hr=dev->CreateGraphicsPipelineState(&desc,IID_PPV_ARGS(&screen_pso));
  if(FAILED(hr)) {std::cerr<<"screen PSO "<<std::hex<<hr;return 28;}
  std::ofstream screen_out("scan_uv_test.screen.cso",std::ios::binary);
  screen_out.write(reinterpret_cast<char*>(screen.data()),screen.size());
  std::cout<<"PASS: scanner original/UV/wide/stable/screen hardware PSOs; exact boost classifier and corruption guards\n";
}
