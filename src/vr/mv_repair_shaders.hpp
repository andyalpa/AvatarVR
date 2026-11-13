#pragma once
// Right-eye motion-vector repair shaders, shared by the plugin (mv_repair.cpp) and the offline equivalence/timing test
// (tools/tests/mv_repair_test.cpp). Both shaders take the same root constants and buffers and must write identical
// output; the reference is the shipped 2026-10-09 shader, unchanged.
#include <windows.h>

namespace mv_repair_shaders {

// Right-eye pixel -> its view position (depth, right projection) -> left view (rotation m, translation t, both from the
// two cameras' sl::Constants) -> left pixel. Where the left depth agrees within 3% + 2 cm, the left eye's motion vector
// is written; otherwise the right eye keeps its own. Depth is reversed infinite: view z = near / depth, near = 0.01.
inline constexpr char kShaderReference[] = R"(
cbuffer C : register(b0) {
  uint W; uint H; uint mvPitch; uint dPitch;
  float rP00; float rP11; float rP20; float rP21;
  float lP00; float lP11; float lP20; float lP21;
  float m00; float m01; float m02; float m10; float m11; float m12; float m20; float m21; float m22;
  float tx; float ty; float tz;
};
ByteAddressBuffer mvL : register(t0);
ByteAddressBuffer dL : register(t1);
ByteAddressBuffer dR : register(t2);
ByteAddressBuffer mvR : register(t3);
RWByteAddressBuffer outMv : register(u0);

// Right-eye pixel (x, y) with depth d -> the left eye's motion vector at the matching pixel, if the left depth agrees.
bool match(int x, int y, float d, out uint mv) {
  mv = 0;
  float vz = 0.01 / d;
  float nx = (x + 0.5) / W * 2 - 1;
  float ny = 1 - (y + 0.5) / H * 2;
  float3 v = float3((nx - rP20) * vz / rP00, (ny - rP21) * vz / rP11, vz);
  float3 l = float3(m00 * v.x + m01 * v.y + m02 * v.z + tx,
                    m10 * v.x + m11 * v.y + m12 * v.z + ty,
                    m20 * v.x + m21 * v.y + m22 * v.z + tz);
  if (!(l.z > 0.01)) return false;
  float lnx = (lP00 * l.x + lP20 * l.z) / l.z;
  float lny = (lP11 * l.y + lP21 * l.z) / l.z;
  float fx = (lnx + 1) * 0.5 * W - 0.5, fy = (1 - lny) * 0.5 * H - 0.5;
  if (!(fx > -0.5 && fy > -0.5 && fx < W - 0.5 && fy < H - 0.5)) return false;  // also rejects NaN
  uint lx = (uint)round(fx), ly = (uint)round(fy);
  float dl = asfloat(dL.Load(ly * dPitch + lx * 4));
  if (!(dl > 0) || abs(0.01 / dl - l.z) > 0.03 * l.z + 0.02) return false;
  mv = mvL.Load(ly * mvPitch + lx * 4);
  return true;
}

// Single pass (2026-10-09). A pixel the left eye cannot see (the far side of a character from the left eye) has no
// match; it kept the right eye's camera-only vector, which DLSS dragged as a ghost along one side of moving characters.
// It now takes the vector of the neighbour (within 12 px, step 3) at the closest depth (within 5%) that does match:
// the same surface. Background uncovered next to a character finds matching background and keeps camera motion.
// (A separate second dispatch for this faulted the GPU, 2026-10-09 06:20; this keeps the one-dispatch layout.)
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= W || id.y >= H) return;
  uint addr = id.y * mvPitch + id.x * 4;
  uint result = mvR.Load(addr);
  float d = asfloat(dR.Load(id.y * dPitch + id.x * 4));
  if (d > 0) {
    uint mv;
    if (match((int)id.x, (int)id.y, d, mv)) {
      result = mv;
    } else {
      float z = 0.01 / d;
      float best = 0.05;
      for (int dy = -12; dy <= 12; dy += 3) {
        for (int dx = -12; dx <= 12; dx += 3) {
          int nx = (int)id.x + dx, ny = (int)id.y + dy;
          if (nx < 0 || ny < 0 || nx >= (int)W || ny >= (int)H) continue;
          float dn = asfloat(dR.Load(ny * dPitch + nx * 4));
          if (!(dn > 0)) continue;
          float err = abs(0.01 / dn - z) / z;
          if (err >= best) continue;
          uint nmv;
          if (match(nx, ny, dn, nmv)) {
            best = err; result = nmv;
            // Absolute depth error cannot improve on zero. The original scan
            // rejects every later tie (err >= best), so the output is identical.
            if (best == 0) break;
          }
        }
        if (best == 0) break;
      }
    }
  }
  outMv.Store(addr, result);
}
)";

// Tiled variant (2026-10-17, "mvfast" in AvatarVR_dlss.txt; off by default). Same inputs, constants and output as the
// reference. [LIVE] census 2026-10-17: the reference dispatch is the most expensive single compute dispatch of the
// frame (~0.46 ms attributed). Cost model: a pixel without a direct match runs up to 81 serial neighbour matches
// while the other lanes of its wave idle. Here each 8x8 tile first matches its own pixels; only a tile with an
// unmatched pixel then matches its 32x32 apron (radius 12) once, cooperatively, into groupshared memory, and the
// unmatched pixels scan that table in the reference order with the reference error test and tie rule. match() is the
// reference function text, a pure function of (x, y, depth), so precomputing it does not change which neighbour wins.
// Compiled with IEEE strictness: fxc otherwise reassociates float math differently at each inlined match() call site
// (the reference's centre and neighbour sites already disagree by an ulp at rounding boundaries). [LIVE] mv_repair_test
// on captures 060033/062628, 4 eye offsets each: tiled+strict is bit-identical to the reference compiled strict; the
// shipped (non-strict) reference differs on 60-168 of ~1.4M pixels by <= 3.3e-4 motion-vector units.
inline constexpr unsigned kTiledExtraFlags = 1u << 13;  // D3DCOMPILE_IEEE_STRICTNESS
inline constexpr char kShaderTiled[] = R"(
cbuffer C : register(b0) {
  uint W; uint H; uint mvPitch; uint dPitch;
  float rP00; float rP11; float rP20; float rP21;
  float lP00; float lP11; float lP20; float lP21;
  float m00; float m01; float m02; float m10; float m11; float m12; float m20; float m21; float m22;
  float tx; float ty; float tz;
};
ByteAddressBuffer mvL : register(t0);
ByteAddressBuffer dL : register(t1);
ByteAddressBuffer dR : register(t2);
ByteAddressBuffer mvR : register(t3);
RWByteAddressBuffer outMv : register(u0);

bool match(int x, int y, float d, out uint mv) {
  mv = 0;
  float vz = 0.01 / d;
  float nx = (x + 0.5) / W * 2 - 1;
  float ny = 1 - (y + 0.5) / H * 2;
  float3 v = float3((nx - rP20) * vz / rP00, (ny - rP21) * vz / rP11, vz);
  float3 l = float3(m00 * v.x + m01 * v.y + m02 * v.z + tx,
                    m10 * v.x + m11 * v.y + m12 * v.z + ty,
                    m20 * v.x + m21 * v.y + m22 * v.z + tz);
  if (!(l.z > 0.01)) return false;
  float lnx = (lP00 * l.x + lP20 * l.z) / l.z;
  float lny = (lP11 * l.y + lP21 * l.z) / l.z;
  float fx = (lnx + 1) * 0.5 * W - 0.5, fy = (1 - lny) * 0.5 * H - 0.5;
  if (!(fx > -0.5 && fy > -0.5 && fx < W - 0.5 && fy < H - 0.5)) return false;  // also rejects NaN
  uint lx = (uint)round(fx), ly = (uint)round(fy);
  float dl = asfloat(dL.Load(ly * dPitch + lx * 4));
  if (!(dl > 0) || abs(0.01 / dl - l.z) > 0.03 * l.z + 0.02) return false;
  mv = mvL.Load(ly * mvPitch + lx * 4);
  return true;
}

#define TILE 8
#define REACH 12
#define APRON 32  // TILE + 2 * REACH
groupshared float sD[APRON * APRON];  // right depth; 0 = outside the image or no depth (reference: skipped)
groupshared uint sMv[APRON * APRON];  // left vector when matched
groupshared uint sOk[APRON * APRON];  // match() result
groupshared uint sSearch;             // any pixel of this tile needs the neighbour search

[numthreads(8, 8, 1)]
void main(uint3 gid : SV_GroupID, uint3 tid : SV_GroupThreadID, uint gi : SV_GroupIndex) {
  if (gi == 0) sSearch = 0;
  GroupMemoryBarrierWithGroupSync();
  int ox = (int)gid.x * TILE - REACH, oy = (int)gid.y * TILE - REACH;
  int x = (int)gid.x * TILE + (int)tid.x, y = (int)gid.y * TILE + (int)tid.y;
  bool inside = x < (int)W && y < (int)H;
  float d = 0;
  uint own = 0;
  bool ok = false;
  if (inside) {
    d = asfloat(dR.Load(y * dPitch + x * 4));
    if (d > 0) {
      ok = match(x, y, d, own);
      if (!ok) InterlockedOr(sSearch, 1u);
    }
  }
  int c = ((int)tid.y + REACH) * APRON + (int)tid.x + REACH;
  sD[c] = inside ? d : 0;
  sMv[c] = own;
  sOk[c] = ok ? 1u : 0u;
  GroupMemoryBarrierWithGroupSync();
  if (sSearch != 0) {  // uniform across the group
    for (uint i = gi; i < APRON * APRON; i += TILE * TILE) {
      uint cy = i / APRON, cx = i % APRON;
      if (cx >= REACH && cx < REACH + TILE && cy >= REACH && cy < REACH + TILE) continue;  // own pixels, done above
      int nx = ox + (int)cx, ny = oy + (int)cy;
      float dn = 0;
      uint nmv = 0;
      bool nok = false;
      if (nx >= 0 && ny >= 0 && nx < (int)W && ny < (int)H) {
        dn = asfloat(dR.Load(ny * dPitch + nx * 4));
        if (dn > 0) nok = match(nx, ny, dn, nmv);
      }
      sD[i] = dn;
      sMv[i] = nmv;
      sOk[i] = nok ? 1u : 0u;
    }
  }
  GroupMemoryBarrierWithGroupSync();
  if (!inside) return;
  uint addr = y * mvPitch + x * 4;
  uint result = mvR.Load(addr);
  if (d > 0) {
    if (ok) {
      result = own;
    } else {
      float z = 0.01 / d;
      float best = 0.05;
      for (int dy = -12; dy <= 12; dy += 3) {
        for (int dx = -12; dx <= 12; dx += 3) {
          int n = c + dy * APRON + dx;
          float dn = sD[n];
          if (!(dn > 0)) continue;
          float err = abs(0.01 / dn - z) / z;
          if (err >= best) continue;
          if (sOk[n] != 0) {
            best = err; result = sMv[n];
            if (best == 0) break;
          }
        }
        if (best == 0) break;
      }
    }
  }
  outMv.Store(addr, result);
}
)";

// Root constants (28 x 32-bit): sizes/pitches, both projections, right-camera -> left-camera rotation and translation.
struct Constants {
  UINT w, h, mv_pitch, d_pitch;
  float rp[4], lp[4];
  float m[9], t[3];
};

// r, l: the two eyes' sl::Constants as floats (projection @8, camera position @94, up @97, right @100, fwd @103).
// Double precision on the CPU: world positions are large, the eye offset is centimetres. False = unusable projection.
inline bool build_constants(const float* r, const float* l, UINT w, UINT h, UINT mv_pitch, UINT d_pitch, Constants* out) {
  auto axis = [](const float* c, int k, int i) { return static_cast<double>(c[(k == 0 ? 100 : k == 1 ? 97 : 103) + i]); };  // right, up, fwd
  Constants k = {};
  k.w = w;
  k.h = h;
  k.mv_pitch = mv_pitch;
  k.d_pitch = d_pitch;
  k.rp[0] = r[8];
  k.rp[1] = r[13];
  k.rp[2] = r[16];
  k.rp[3] = r[17];
  k.lp[0] = l[8];
  k.lp[1] = l[13];
  k.lp[2] = l[16];
  k.lp[3] = l[17];
  const double d[3] = {static_cast<double>(r[94]) - l[94], static_cast<double>(r[95]) - l[95], static_cast<double>(r[96]) - l[96]};
  for (int i = 0; i < 3; ++i) {      // left axis i
    for (int j = 0; j < 3; ++j) {    // right axis j
      double s = 0;
      for (int c = 0; c < 3; ++c) s += axis(l, i, c) * axis(r, j, c);
      k.m[i * 3 + j] = static_cast<float>(s);
    }
    double s = 0;
    for (int c = 0; c < 3; ++c) s += axis(l, i, c) * d[c];
    k.t[i] = static_cast<float>(s);
  }
  *out = k;
  return k.rp[0] > 0.01f && k.rp[1] > 0.01f && k.lp[0] > 0.01f && k.lp[1] > 0.01f;
}

}  // namespace mv_repair_shaders
