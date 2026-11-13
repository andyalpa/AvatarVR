#include "vr_arms.hpp"
#include "../diagnostics/skin_arm_ik.hpp"
#include "vr_arms_math.hpp"
#include "../game/player.hpp"
#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

namespace vr_arms {
namespace {
long long qpc_us(LARGE_INTEGER now, LARGE_INTEGER freq) {
  return static_cast<long long>(now.QuadPart / freq.QuadPart * 1000000 +
                                (now.QuadPart % freq.QuadPart) * 1000000 / freq.QuadPart);
}
std::mutex g_mutex;
Sample g_sample;  // g_mutex
Sample g_prev_sample;  // g_mutex: the sample before g_sample
std::atomic<unsigned> g_published{0};
skin_arm_ik::OneEuro g_filters[2][12];
LARGE_INTEGER g_last{};
std::atomic<bool> g_smoothing{true};
std::atomic<bool> g_view_lock{true};
std::atomic<bool> g_rotation_tuned{true};
std::atomic<unsigned> g_present_gen{1},g_basis_rejects{0};
float g_good_basis[16]{}; bool g_have_good_basis=false;  // g_mutex
std::atomic<bool> g_view_stable{true},g_view_track{true};
float g_raw_history[3][3]{}; int g_raw_count=0;  // g_mutex: raw offsets, newest first
vr_arms_math::ViewTracker g_tracker;              // g_mutex
long long g_anchor_us=0;                          // g_mutex
bool anchor_offset_locked(const float* body,float* out3) {
  if(!g_raw_count) return false;
  LARGE_INTEGER now,freq; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&freq);
  const long long us=qpc_us(now,freq);
  if(us-g_anchor_us>5000) {  // gun placement and eye step of one tick are under a millisecond apart, ticks over 10 ms
    double m[3];
    for(int k=0;k<3;++k) {
      const float o=g_raw_count>=3 && g_view_track.load(std::memory_order_relaxed)?vr_arms_math::median3(g_raw_history[0][k],g_raw_history[1][k],g_raw_history[2][k]):g_raw_history[0][k];
      m[k]=static_cast<double>(body[k])+o;
    }
    g_tracker.step(m,g_view_track.load(std::memory_order_relaxed));
    g_anchor_us=us;
  }
  for(int k=0;k<3;++k) out3[k]=static_cast<float>(g_tracker.g[k]-static_cast<double>(body[k]));
  return true;
}
float g_previous_raw[3]{}; bool g_previous_raw_valid=false;  // g_mutex
float g_eye_delta[3]{};  // g_mutex
float g_view_offset[3]{};
LARGE_INTEGER g_view_last{};
bool g_view_primed=false;
bool stable_view_offset_locked(const float* raw3, float* out3, LARGE_INTEGER now, LARGE_INTEGER freq) {
  float change2=0.f;
  for(int k=0;k<3;++k) {
    if(!std::isfinite(raw3[k])) return false;
    change2+=(raw3[k]-g_view_offset[k])*(raw3[k]-g_view_offset[k]);
  }
  if(std::fabs(raw3[0])+std::fabs(raw3[1])+std::fabs(raw3[2])>=6.f) return false;
  const float dt=g_view_last.QuadPart?static_cast<float>(now.QuadPart-g_view_last.QuadPart)/static_cast<float>(freq.QuadPart):0.f;
  g_view_last=now;
  if(!g_view_primed || change2>0.5f*0.5f || !(dt>0.f) || dt>0.5f) {
    std::memcpy(g_view_offset,raw3,sizeof(g_view_offset));
    g_view_primed=true;
  } else {
    // Horizontal camera sway is animation, while the body root already supplies locomotion.
    // Follow crouch height sooner than the horizontal sway; both hands and gun use this state.
    for(int k=0;k<3;++k) {
      const float tau=k==1?0.10f:0.25f;
      g_view_offset[k]+=(1.f-std::exp(-dt/tau))*(raw3[k]-g_view_offset[k]);
    }
  }
  std::memcpy(out3,g_view_offset,sizeof(g_view_offset));
  return true;
}
}

void set_smoothing(bool on) { g_smoothing.store(on, std::memory_order_relaxed); }
void set_view_lock(bool on) { g_view_lock.store(on, std::memory_order_relaxed); }
void set_rotation_tuned(bool on) { g_rotation_tuned.store(on, std::memory_order_relaxed); }
unsigned basis_rejects() { return g_basis_rejects.load(std::memory_order_relaxed); }
void set_view_stable(bool on, bool track) { g_view_stable.store(on,std::memory_order_relaxed); g_view_track.store(track,std::memory_order_relaxed); }
bool view_stable() { return g_view_stable.load(std::memory_order_relaxed); }
bool stable_view_anchor(const float* body3, float* offset3) {
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_view_stable.load(std::memory_order_relaxed) && anchor_offset_locked(body3,offset3);
}
unsigned published_steps() { return g_published.load(std::memory_order_relaxed); }
bool armed_sample(Sample& out, float* anchor3) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if(!g_sample.seq) return false;
  const bool this_frame=g_sample.view_locked && g_sample.view_gen==g_present_gen.load(std::memory_order_relaxed);
  if(this_frame && g_prev_sample.seq+1==g_sample.seq && g_sample.sampled_qpc_us-g_prev_sample.sampled_qpc_us<50000) {
    out=g_prev_sample; std::memcpy(anchor3,g_sample.view_offset,12); return true;
  }
  out=g_sample; std::memcpy(anchor3,anchor_view_offset(g_sample),12); return true;
}
bool frame_eye_shift(float* out3) {
  std::lock_guard<std::mutex> lock(g_mutex);
  const bool now=g_sample.view_locked && g_sample.view_gen==g_present_gen.load(std::memory_order_relaxed);
  for(int k=0;k<3;++k) out3[k]=now?g_eye_delta[k]:0.f;
  return now;
}
void note_present() { g_present_gen.fetch_add(1, std::memory_order_relaxed); }
const float* anchor_view_offset(const Sample& s) {
  if(!s.view_locked) return s.view_offset;
  return vr_arms_math::shared_view_offset(s.view_gen,g_present_gen.load(std::memory_order_relaxed),s.view_offset,s.previous_view_offset);
}
void eye_view_delta(float* out3) {
  std::lock_guard<std::mutex> lock(g_mutex);
  std::memcpy(out3,g_eye_delta,sizeof(g_eye_delta));
}
bool stable_view_offset(const float* raw3, float* out3) {
  LARGE_INTEGER now,freq; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&freq);
  std::lock_guard<std::mutex> lock(g_mutex);
  return stable_view_offset_locked(raw3,out3,now,freq);
}

void publish(const float (&rotation)[2][9], const float (&offset)[2][3], const bool (&valid)[2], const float* head_offset3,
             const float* basis16, const float* head_rot9) {
  Sample s;
  // Rows of the last good basis with this step's position: see vr_arms_math::basis_good.
  float checked_basis[16];
  std::memcpy(checked_basis, basis16, sizeof(checked_basis));
  {std::lock_guard<std::mutex> lock(g_mutex);
    if(vr_arms_math::basis_good(basis16)) {std::memcpy(g_good_basis,basis16,sizeof(g_good_basis)); g_have_good_basis=true;}
    else if(g_have_good_basis) {std::memcpy(checked_basis,g_good_basis,48); g_basis_rejects.fetch_add(1,std::memory_order_relaxed);}}
  basis16=checked_basis;
  std::memcpy(s.basis, basis16, sizeof(s.basis));
  if(head_offset3) {
    for(int k=0;k<3;++k)
      s.head_world[k]=head_offset3[0]*basis16[k]+head_offset3[1]*basis16[4+k]+head_offset3[2]*basis16[8+k];
    s.head_world_valid=std::isfinite(s.head_world[0]) && std::isfinite(s.head_world[1]) &&
                       std::isfinite(s.head_world[2]);
  }
  if (head_rot9) {std::memcpy(s.head_rot, head_rot9, sizeof(s.head_rot)); s.head_valid = std::isfinite(head_rot9[2]) && std::isfinite(head_rot9[8]);}
  float body_at[3] = {};
  {float body[3]; const game::Pointers p = game::pointers();  // the view's height relative to the body, same instant
    if (p.body && game::mem_read(p.body + 0x94, body, sizeof(body))) {
      std::memcpy(body_at, body, sizeof(body_at));
      for (int k = 0; k < 3; ++k) s.view_offset[k] = basis16[12 + k] - body[k];
      s.view_valid = std::isfinite(s.view_offset[0]) && std::isfinite(s.view_offset[1]) && std::isfinite(s.view_offset[2]) &&
                     std::fabs(s.view_offset[0]) + std::fabs(s.view_offset[1]) + std::fabs(s.view_offset[2]) < 6.f;
    }}
  LARGE_INTEGER now, freq;
  QueryPerformanceCounter(&now); QueryPerformanceFrequency(&freq);
  s.sampled_qpc_us = qpc_us(now, freq);
  std::lock_guard<std::mutex> lock(g_mutex);
  const float dt = g_last.QuadPart ? static_cast<float>(now.QuadPart - g_last.QuadPart) / static_cast<float>(freq.QuadPart) : 0.f;
  g_last = now;
  std::memcpy(s.raw_view_offset, s.view_offset, sizeof(s.raw_view_offset));
  if(s.view_valid) s.view_valid=stable_view_offset_locked(s.view_offset,s.view_offset,now,freq);
  // [LIVE 2026-10-30] the eyes use the raw camera, so the filter's residual moved the arms against the view (max 38 cm in
  // sprint jumps). [LIVE, same day, armed] a gun placed before the camera step used a sample one frame older than the eyes
  // (solve sample age 15 ms, bare hands 0.16 ms), so the sway came back with a weapon. Locked: arms, gun and eyes all use
  // the previous sample's raw offset (anchor_view_offset, eye_view_delta); the filter keeps running for the off path.
  std::memset(g_eye_delta,0,sizeof(g_eye_delta));
  if(s.view_valid && g_view_lock.load(std::memory_order_relaxed)) {
    std::memcpy(s.view_offset,s.raw_view_offset,sizeof(s.view_offset));
    float change2=0.f;
    for(int k=0;k<3;++k) change2+=(s.raw_view_offset[k]-g_previous_raw[k])*(s.raw_view_offset[k]-g_previous_raw[k]);
    if(!g_previous_raw_valid || change2>0.5f*0.5f) {std::memcpy(g_previous_raw,s.raw_view_offset,sizeof(g_previous_raw)); std::memcpy(g_raw_history[0],s.raw_view_offset,12); g_raw_count=1; g_anchor_us=0;}  // teleport, stance snap
    std::memcpy(s.previous_view_offset,g_previous_raw,sizeof(g_previous_raw));
    if(g_view_stable.load(std::memory_order_relaxed) && anchor_offset_locked(body_at,s.previous_view_offset)) std::memcpy(s.view_offset,s.previous_view_offset,sizeof(s.view_offset));
    for(int k=0;k<3;++k) g_eye_delta[k]=s.previous_view_offset[k]-s.raw_view_offset[k];
    s.view_gen=g_present_gen.load(std::memory_order_relaxed); s.view_locked=true;
  }
  if(s.view_valid) {
    std::memcpy(g_previous_raw,s.raw_view_offset,sizeof(g_previous_raw)); g_previous_raw_valid=true;
    std::memmove(g_raw_history[1],g_raw_history[0],24); std::memcpy(g_raw_history[0],s.raw_view_offset,12); if(g_raw_count<3) ++g_raw_count;
  }
  else {g_previous_raw_valid=false; g_raw_count=0;}
  for (int h = 0; h < 2; ++h) {
    s.valid[h] = valid[h];
    if (!valid[h]) {g_filters[h][0].primed = false; for (auto& f : g_filters[h]) f.primed = false; continue;}
    float* m = s.local[h];
    for (int r = 0; r < 3; ++r) {
      m[r * 4] = rotation[h][r]; m[r * 4 + 1] = rotation[h][3 + r]; m[r * 4 + 2] = rotation[h][6 + r]; m[r * 4 + 3] = 0.f;
    }
    for (int k = 0; k < 3; ++k) m[12 + k] = offset[h][k] - head_offset3[k];
    m[15] = 1.f;
    bool finite = true;
    for (int i = 0; i < 16; ++i) finite = finite && std::isfinite(m[i]);
    if (!finite) {s.valid[h] = false; continue;}
    std::memcpy(s.raw_local[h], m, sizeof(s.raw_local[h]));
    if (g_smoothing.load(std::memory_order_relaxed) && dt > 0.f && dt < 0.25f) skin_arm_ik::smooth_grip(m, g_filters[h], dt, g_rotation_tuned.load(std::memory_order_relaxed));
    else for (auto& f : g_filters[h]) f.primed = false;
  }
  s.seq = g_sample.seq + 1;
  g_prev_sample = g_sample;
  g_published.fetch_add(1, std::memory_order_relaxed);
  g_sample = s;
}

bool latest(Sample& out) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_sample.seq) return false;
  out = g_sample;
  return true;
}
}  // namespace vr_arms
