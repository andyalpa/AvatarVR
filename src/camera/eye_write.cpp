#include "eye_write.hpp"

#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

// Field map (row-vector convention, clip = pos * M, matrices stored row-major), [LIVE] runs 7 and 9:
//   0x000 View (world->view)            0x040 View duplicate
//   0x080 Proj (jittered, off-axis)     0x0C0 ViewProj      0x100 ViewProj duplicate
//   0x140 InvView (cell-local)          0x180 InvView duplicate
//   0x1C0 InvProj                       0x200 InvViewProj   0x240 InvViewProj duplicate
//   0x280 prev ViewProj                 0x2C0 prev ViewProj duplicate
//   0x300 prev View                     0x340 Proj duplicate
//   0x3F0, 0x500 absolute camera position (xyz)
// An eye offset d along the camera's right axis is a view-space translation T(-d) applied after the
// view: V' = V*T(-d), which only changes V[12] by -d. Every dependent field changes by a delta:
//   ViewProj'[12+j]  = ViewProj[12+j] - d*P[j]          (row 3 of V*P)
//   InvView' row 3  += d*right_world, right_world = (V[0], V[4], V[8])
//   InvViewProj'[i][j] += Pinv[i][3] * d*right_world[j]  (j < 3), from (V^-1 P^-1 form) P^-1 * V'^-1
//   abs position    += d*right_world
// Frustum planes (0x380..0x3D0) are left alone in this test; culling is off by at most |d|.

namespace {

constexpr std::size_t kBlockSize = 1536;
constexpr int kHashRing = 64;

std::mutex g_mutex;
std::wstring g_control_path;
std::wstring g_log_path;

enum class Mode { Off, Zero, Offset, Alt };
// Present counter for alternate-eye mode: even presents = left eye (-h), odd = right eye (+h).
std::atomic<std::uint64_t> g_presents{0};
std::atomic<int> g_alt_period{1};  // presents per eye in alt mode (1 = every frame)
std::atomic<std::uint64_t> g_written_left{0};
std::atomic<std::uint64_t> g_written_right{0};
std::atomic<int> g_mode{static_cast<int>(Mode::Off)};
std::atomic<float> g_offset{0.f};
std::atomic<int> g_last_eye{0};
// alt mode: previous-frame fields carry the OTHER eye's offset (default, one shared temporal history) or the SAME
// eye's offset ("same": for per-eye upscaler history, where each eye's previous image is its own).
std::atomic<bool> g_prev_same{false};
// Inter-eye travel (BioshockVR's clamp check): how far the ORIGINAL camera moved between consecutive
// presents, split by transition. With the world clock frozen between the two eyes of a pair, the
// in-pair transition is ~0 while walking and the between-pair one carries all the motion.
double g_travel_lr = 0.0, g_travel_rl = 0.0;  // guarded by g_mutex
std::uint64_t g_travel_lr_n = 0, g_travel_rl_n = 0;
std::uint64_t g_travel_last_present = ~0ull;
float g_travel_last_pos[3] = {};
bool g_travel_last_right = false;  // alt mode: eye of the most recent camera write (-1 left, +1 right)

std::atomic<std::uint64_t> g_seen{0};
std::atomic<std::uint64_t> g_guard_fail{0};
std::atomic<std::uint64_t> g_already{0};
std::atomic<std::uint64_t> g_written{0};
std::atomic<std::uint64_t> g_faults{0};

struct HashEntry {
  const void* address;
  std::uint64_t hash;
};
HashEntry g_ring[kHashRing] = {};
int g_ring_next = 0;

void log_line(const char* fmt, ...) {
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  if (g_log_path.empty()) {
    return;
  }
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, buffer, static_cast<DWORD>(std::strlen(buffer)), &written, nullptr);
  WriteFile(file, "\r\n", 2, &written, nullptr);
  CloseHandle(file);
}

std::uint64_t fnv(const unsigned char* p, std::size_t n) {
  std::uint64_t h = 14695981039346656037ull;
  for (std::size_t i = 0; i < n; ++i) {
    h = (h ^ p[i]) * 1099511628211ull;
  }
  return h;
}

bool finite16(const float* m) {
  for (int i = 0; i < 16; ++i) {
    if (!std::isfinite(m[i])) {
      return false;
    }
  }
  return true;
}

bool rigid_row(const float* m) {
  if (!finite16(m) || std::fabs(m[3]) > 1e-3f || std::fabs(m[7]) > 1e-3f || std::fabs(m[11]) > 1e-3f || std::fabs(m[15] - 1.f) > 1e-3f) {
    return false;
  }
  for (int r = 0; r < 3; ++r) {
    const float len = m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2];
    if (len < 0.98f || len > 1.02f) {
      return false;
    }
  }
  return true;
}

bool perspective_row(const float* m) {
  return finite16(m) && std::fabs(m[11] - 1.f) < 1e-3f && std::fabs(m[15]) < 1e-3f && m[5] > 0.1f && m[0] > 0.1f;
}

// V*P == VP within a relative tolerance (row-vector product).
bool product_matches(const float* v, const float* p, const float* vp) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      float s = 0;
      for (int k = 0; k < 4; ++k) {
        s += v[r * 4 + k] * p[k * 4 + c];
      }
      const float ref = vp[r * 4 + c];
      if (std::fabs(s - ref) > 1e-3f + 1e-4f * std::fabs(ref)) {
        return false;
      }
    }
  }
  return true;
}

// Works on a private copy; returns false if the layout guard fails.
// d = eye offset for the current-frame fields; d_prev = the offset the PREVIOUS presented frame was
// rendered with. In alternate-eye mode the previous frame is the other eye (d_prev = -d), so the engine's
// motion vectors describe the real eye swap and temporal history is reprojected instead of averaged.
bool transform_copy(unsigned char* b, float d, float d_prev) {
  float* f = reinterpret_cast<float*>(b);
  const float* v = f + 0x000 / 4;
  const float* p = f + 0x080 / 4;
  const float* vp = f + 0x0C0 / 4;
  const float* prev_v = f + 0x300 / 4;
  if (!rigid_row(v) || !perspective_row(p) || !product_matches(v, p, vp) || !rigid_row(prev_v)) {
    return false;
  }
  // Skip square projections (environment-probe cube faces share this layout; they must not move).
  if (std::fabs(p[5] / p[0] - 1.f) < 0.05f) {
    return false;
  }
  const float rx = v[0], ry = v[4], rz = v[8];  // camera right axis in world (column 0 of R)
  const float dw[3] = {d * rx, d * ry, d * rz};
  const float* pinv = f + 0x1C0 / 4;

  // Views: current (+dup) and previous.
  f[0x000 / 4 + 12] -= d;
  f[0x040 / 4 + 12] -= d;
  f[0x300 / 4 + 12] -= d_prev;
  // ViewProj (+dup) and previous ViewProj (+dup): row 3 -= d * P row 0.
  for (int j = 0; j < 4; ++j) {
    const float delta = d * p[j];
    f[0x0C0 / 4 + 12 + j] -= delta;
    f[0x100 / 4 + 12 + j] -= delta;
    const float delta_prev = d_prev * p[j];
    f[0x280 / 4 + 12 + j] -= delta_prev;
    f[0x2C0 / 4 + 12 + j] -= delta_prev;
  }
  // Inverse view (cell-local, +dup): camera moves by dw.
  for (int j = 0; j < 3; ++j) {
    f[0x140 / 4 + 12 + j] += dw[j];
    f[0x180 / 4 + 12 + j] += dw[j];
  }
  // Inverse ViewProj (+dup): (P^-1 * V'^-1)[i][j] += Pinv[i][3] * dw[j].
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      const float delta = pinv[i * 4 + 3] * dw[j];
      f[0x200 / 4 + i * 4 + j] += delta;
      f[0x240 / 4 + i * 4 + j] += delta;
    }
  }
  // Absolute camera position copies.
  for (int j = 0; j < 3; ++j) {
    f[0x3F0 / 4 + j] += dw[j];
    f[0x500 / 4 + j] += dw[j];
  }
  return true;
}

// SEH-guarded copies to and from the live upload slot. Plain C types only in these frames.
bool copy_in_seh(unsigned char* dst, const void* src, std::size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

bool copy_out_seh(void* dst, const unsigned char* src, std::size_t n) {
  __try {
    std::memcpy(dst, src, n);
    return true;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return false;
  }
}

std::wstring self_dir() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<LPCWSTR>(&self_dir), &self)) {
    return {};
  }
  wchar_t path[MAX_PATH] = {};
  const DWORD n = GetModuleFileNameW(self, path, MAX_PATH);
  if (n == 0 || n >= MAX_PATH) {
    return {};
  }
  std::wstring dir(path, n);
  const auto slash = dir.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : dir.substr(0, slash);
}

}  // namespace

namespace eye_write {

void install() {
  const std::wstring dir = self_dir();
  if (dir.empty()) {
    return;
  }
  g_control_path = dir + L"\\AvatarVR_eye.txt";
  g_log_path = dir + L"\\EYE_WRITE.log";
  MoveFileExW(g_log_path.c_str(), (dir + L"\\EYE_WRITE.prev.log").c_str(), MOVEFILE_REPLACE_EXISTING);
  log_line("eye write build %s %s pid=%lu control=AvatarVR_eye.txt (missing file = off)", __DATE__, __TIME__, GetCurrentProcessId());
}

void poll() {
  Mode mode = Mode::Off;
  float offset = 0.f;
  char text[64] = {};
  HANDLE file = g_control_path.empty() ? INVALID_HANDLE_VALUE
                                       : CreateFileW(g_control_path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file != INVALID_HANDLE_VALUE) {
    DWORD read = 0;
    ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
    CloseHandle(file);
    text[read < sizeof(text) ? read : sizeof(text) - 1] = '\0';
    if (std::strncmp(text, "zero", 4) == 0) {
      mode = Mode::Zero;
    } else if (std::strncmp(text, "alt", 3) == 0) {
      char* end = nullptr;
      const float value = std::strtof(text + 3, &end);
      long period = (end != nullptr) ? std::strtol(end, nullptr, 10) : 1;
      if (period < 1 || period > 600) {
        period = 1;
      }
      g_alt_period.store(static_cast<int>(period));
      g_prev_same.store(std::strstr(text, "same") != nullptr);
      // Half-IPD in metres; hard clamp to 0.5 m (exaggerated test values allowed, real IPD is ~0.032).
      if (std::isfinite(value) && std::fabs(value) <= 0.5f) {
        mode = Mode::Alt;
        offset = value;
      }
    } else if (std::strncmp(text, "offset", 6) == 0) {
      const float value = static_cast<float>(std::atof(text + 6));
      // Hard clamp: never more than 2 m in this experiment.
      if (std::isfinite(value) && std::fabs(value) <= 2.f) {
        mode = Mode::Offset;
        offset = value;
      }
    }
  }
  if (mode == Mode::Off && text[0] != '\0' && std::strncmp(text, "off", 3) != 0) {
    static char last_rejected[64] = {};
    if (std::strncmp(last_rejected, text, sizeof(last_rejected)) != 0) {
      std::strncpy(last_rejected, text, sizeof(last_rejected) - 1);
      log_line("eye write REJECTED control text '%s' (out of range or unknown) -> off", text);
    }
  }
  const int previous = g_mode.exchange(static_cast<int>(mode));
  g_offset.store((mode == Mode::Offset || mode == Mode::Alt) ? offset : 0.f);
  if (previous != static_cast<int>(mode)) {
    log_line("eye write MODE -> %s offset=%.4f period=%d prev=%s", mode == Mode::Off ? "off" : mode == Mode::Zero ? "zero" : mode == Mode::Alt ? "alt" : "offset", offset, g_alt_period.load(), g_prev_same.load() ? "same-eye" : "other-eye");
  }
  log_line("eye write mode=%s offset=%.4f seen=%llu written=%llu (L=%llu R=%llu) already=%llu guard_fail=%llu faults=%llu presents=%llu",
           mode == Mode::Off ? "off" : mode == Mode::Zero ? "zero" : mode == Mode::Alt ? "alt" : "offset", g_offset.load(),
           static_cast<unsigned long long>(g_seen.load()), static_cast<unsigned long long>(g_written.load()),
           static_cast<unsigned long long>(g_written_left.load()), static_cast<unsigned long long>(g_written_right.load()),
           static_cast<unsigned long long>(g_already.load()), static_cast<unsigned long long>(g_guard_fail.load()),
           static_cast<unsigned long long>(g_faults.load()), static_cast<unsigned long long>(g_presents.load()));
  if (mode == Mode::Alt) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_travel_lr_n + g_travel_rl_n > 0) {
      log_line("inter-eye travel (original camera, per present): L->R avg=%.2f mm (n=%llu)  R->L avg=%.2f mm (n=%llu)",
               g_travel_lr_n ? 1000.0 * g_travel_lr / static_cast<double>(g_travel_lr_n) : 0.0,
               static_cast<unsigned long long>(g_travel_lr_n),
               g_travel_rl_n ? 1000.0 * g_travel_rl / static_cast<double>(g_travel_rl_n) : 0.0,
               static_cast<unsigned long long>(g_travel_rl_n));
    }
    g_travel_lr = g_travel_rl = 0.0;
    g_travel_lr_n = g_travel_rl_n = 0;
  }
}

void on_present() {
  g_presents.fetch_add(1, std::memory_order_relaxed);
}

void apply(void* block, std::size_t size) {
  const Mode mode = static_cast<Mode>(g_mode.load(std::memory_order_relaxed));
  if (mode == Mode::Off || block == nullptr || size != kBlockSize) {
    return;
  }
  g_seen.fetch_add(1, std::memory_order_relaxed);
  unsigned char copy[kBlockSize];
  if (!copy_in_seh(copy, block, kBlockSize)) {
    g_faults.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  std::lock_guard<std::mutex> lock(g_mutex);
  const std::uint64_t before = fnv(copy, kBlockSize);
  for (const HashEntry& e : g_ring) {
    if (e.address == block && e.hash == before) {
      g_already.fetch_add(1, std::memory_order_relaxed);
      return;  // this refill was already transformed
    }
  }
  float d = 0.f;
  bool right_eye = false;
  if (mode == Mode::Offset) {
    d = g_offset.load(std::memory_order_relaxed);
  } else if (mode == Mode::Alt) {
    right_eye = ((g_presents.load(std::memory_order_relaxed) / static_cast<std::uint64_t>(g_alt_period.load(std::memory_order_relaxed))) & 1) != 0;
    const float h = g_offset.load(std::memory_order_relaxed);
    d = right_eye ? h : -h;
  }
  float d_prev = d;
  if (mode == Mode::Alt) {
    const std::uint64_t period = static_cast<std::uint64_t>(g_alt_period.load(std::memory_order_relaxed));
    const bool switched = (g_presents.load(std::memory_order_relaxed) % period) == 0;
    d_prev = (switched && !g_prev_same.load(std::memory_order_relaxed)) ? -d : d;
  }
  float original_pos[3];
  std::memcpy(original_pos, copy + 0x3F0, sizeof(original_pos));
  if (!transform_copy(copy, d, d_prev)) {
    g_guard_fail.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (!copy_out_seh(block, copy, kBlockSize)) {
    g_faults.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  g_ring[g_ring_next] = HashEntry{block, fnv(copy, kBlockSize)};
  g_ring_next = (g_ring_next + 1) % kHashRing;
  g_written.fetch_add(1, std::memory_order_relaxed);
  if (mode == Mode::Alt) {
    (right_eye ? g_written_right : g_written_left).fetch_add(1, std::memory_order_relaxed);
    g_last_eye.store(right_eye ? 1 : -1, std::memory_order_relaxed);
    const std::uint64_t now = g_presents.load(std::memory_order_relaxed);
    if (now != g_travel_last_present) {  // first camera block of a new present
      if (g_travel_last_present != ~0ull && now == g_travel_last_present + 1 && right_eye != g_travel_last_right) {
        const double dx = original_pos[0] - g_travel_last_pos[0];
        const double dy = original_pos[1] - g_travel_last_pos[1];
        const double dz = original_pos[2] - g_travel_last_pos[2];
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (right_eye) {
          g_travel_lr += dist;
          ++g_travel_lr_n;
        } else {
          g_travel_rl += dist;
          ++g_travel_rl_n;
        }
      }
      g_travel_last_present = now;
      std::memcpy(g_travel_last_pos, original_pos, sizeof(original_pos));
      g_travel_last_right = right_eye;
    }
  }
}

int presented_eye() {
  if (static_cast<Mode>(g_mode.load(std::memory_order_relaxed)) != Mode::Alt) {
    return 0;
  }
  return g_last_eye.load(std::memory_order_relaxed);
}

int parity_eye() {
  if (static_cast<Mode>(g_mode.load(std::memory_order_relaxed)) != Mode::Alt) {
    return 0;
  }
  const std::uint64_t period = static_cast<std::uint64_t>(g_alt_period.load(std::memory_order_relaxed));
  return ((g_presents.load(std::memory_order_relaxed) / period) & 1) != 0 ? 1 : -1;
}

}  // namespace eye_write
