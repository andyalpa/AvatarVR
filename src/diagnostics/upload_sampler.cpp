#include "upload_sampler.hpp"

#include "resource_names.hpp"
#include "../camera/native_stereo.hpp"
#include "../camera/scanner_inputs.hpp"

#include "matrix_detector.hpp"
#include "../camera/eye_write.hpp"

#include <MinHook.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace {

constexpr int kCreateCommittedResource = 27;
constexpr int kCreatePlacedResource = 29;
constexpr int kCreateCBV = 17;
constexpr int kResourceMap = 8;
constexpr int kResourceUnmap = 9;
constexpr UINT64 kMinBytes = 256;
constexpr UINT64 kMaxBytes = 65536;
constexpr UINT64 kModestMax = 1048576;
constexpr UINT64 kModestCopy = 4096;
constexpr int kTrackedMax = 256;
constexpr int kHistoryMax = 64;
constexpr int kQueueMax = 64;
constexpr int kLogMax = 12;

// Safety/clarity switches (2026-10-01, run-6 analysis).
// The legacy Phase-4 background scan copies 4 KB/s out of a mapped upload buffer with a plain memcpy.
// Slots are never retired when the engine releases a resource, so that pointer can dangle. The CBV path
// superseded this scan (the camera was found there), so it is off by default.
constexpr bool kEnableModestWorker = false;
// The 16-slot per-CBV table is saturated every sample (144 distinct size/kind/FOV buckets compete for 16
// slots in the run-6 log), and logging all of it every second is ~150 file writes on the Present thread.
// Dump it every Nth sample only; camera tracking uses the dedicated latch below instead.
constexpr int kCbvTableEvery = 10;
// Main-camera latch: co-located blocks with a gameplay-plausible projection, keyed by (size, FOV*10).
constexpr int kCamBucketMax = 12;
constexpr UINT64 kMainCamSize = 1536;  // [LIVE] run 2..6: the main view lives in 1536-byte CBVs
// Read-only field dump of the latched main-camera block (and each sibling once), for the RE-08/RE-09
// field map. Bounded: at most kFieldDumpMax main dumps, one every kFieldDumpEvery samples.
constexpr int kFieldDumpEvery = 10;
constexpr int kFieldDumpMax = 6;
constexpr int kSiblingDumpMax = 6;
constexpr std::size_t kFieldDumpBytes = 4096;
// A dump set holds up to kDumpCopies copies of the main block: every CBV of that bucket in two
// consecutive frames (present N and N+1). Same-frame copies show whether the per-frame blocks agree;
// the N/N+1 pair identifies previous-frame (TAA/motion-vector) fields.
constexpr int kDumpCopies = 8;
// Only dump the main bucket while it looks like gameplay (camera far from the menu/UI scenes, which sit
// within a few hundred units of the origin in run 7). Position-dependent on purpose; diagnostics only.
constexpr float kGameplayTmagMin = 1000.f;

using CreateCommittedResourceFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, const D3D12_HEAP_PROPERTIES*, D3D12_HEAP_FLAGS, const D3D12_RESOURCE_DESC*, D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*, REFIID, void**);
using CreatePlacedResourceFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Device*, ID3D12Heap*, UINT64, const D3D12_RESOURCE_DESC*, D3D12_RESOURCE_STATES, const D3D12_CLEAR_VALUE*, REFIID, void**);
using CreateCBVFn = void(STDMETHODCALLTYPE*)(ID3D12Device*, const D3D12_CONSTANT_BUFFER_VIEW_DESC*, D3D12_CPU_DESCRIPTOR_HANDLE);
using MapFn = HRESULT(STDMETHODCALLTYPE*)(ID3D12Resource*, UINT, const D3D12_RANGE*, void**);
using UnmapFn = void(STDMETHODCALLTYPE*)(ID3D12Resource*, UINT, const D3D12_RANGE*);

CreateCommittedResourceFn g_create_committed = nullptr;
CreatePlacedResourceFn g_create_placed = nullptr;
CreateCBVFn g_create_cbv = nullptr;
MapFn g_map = nullptr;
UnmapFn g_unmap = nullptr;

std::mutex g_log_mutex;
std::mutex g_hook_mutex;
std::mutex g_state_mutex;
std::wstring g_log_path;
void* g_hooked[8] = {};
int g_hooked_count = 0;
bool g_map_armed = false;
bool g_logged_small = false;

std::atomic<std::uint64_t> g_small_count{0};
std::atomic<std::uint64_t> g_large_count{0};
std::atomic<std::uint64_t> g_small_skip_count{0};
std::atomic<std::uint64_t> g_readback_count{0};
std::atomic<std::uint64_t> g_scan_count{0};
std::atomic<std::uint64_t> g_map_without_pointer{0};
std::atomic<std::uint64_t> g_cbv_total{0};
std::atomic<std::uint64_t> g_cbv_resolved{0};
std::atomic<std::uint64_t> g_cbv_matrix{0};

// Bounded per-CBV observation log: one line per second, newest-wins per size bucket.
struct CbvLog {
  UINT64 size = 0;
  UINT64 va = 0;
  void* cpu = nullptr;        // address of the first matched matrix inside the block
  void* block_cpu = nullptr;  // start of the CBV range (only dereferenced inside the CBV hook)
  void* resource = nullptr;
  UINT64 width = 0;
  UINT64 offset = 0;
  matrix_detector::Kind kind = matrix_detector::Kind::None;
  matrix_detector::Layout layout = matrix_detector::Layout::RowMajor;
  float matrix[16] = {};
  float fov_y = 0;
  bool near_ok = false;
  bool far_ok = false;
  float near_z = 0;
  float far_z = 0;
  float translation[3] = {};
  bool has_translation = false;
  int rigid_matches = 0;
  int perspective_matches = 0;
  // Co-located buffers carry a projection AND a view. Log both so the camera owner can be
  // correlated: the projection is usually constant (stable FOV), the view carries position+rotation.
  bool has_proj = false;
  matrix_detector::Layout proj_layout = matrix_detector::Layout::RowMajor;
  float proj_matrix[16] = {};
  float proj_fov_y = 0;
  bool has_view = false;
  matrix_detector::Layout view_layout = matrix_detector::Layout::RowMajor;
  float view_matrix[16] = {};
  float view_translation[3] = {};
  ULONGLONG stamp = 0;
};
constexpr int kCbvLogMax = 16;
CbvLog g_cbv_log[kCbvLogMax] = {};
int g_cbv_log_count = 0;

// Per-frame camera-delta tracker. Persists across sample() calls so a deliberate look or walk
// shows up as a delta between two consecutive one-second samples. drot uses the rotation-matrix
// angle (convention-independent); yaw/pitch are logged for human interpretability only.
struct CamTrack {
  bool have_prev = false;
  float prev_translation[3] = {};
  float prev_matrix[16] = {};
  float prev_yaw = 0;
  float prev_pitch = 0;
  ULONGLONG prev_stamp = 0;
  float max_dtrans = 0;
  float max_drot = 0;
  int moved_samples = 0;
  int looked_samples = 0;
  // Identity history: the ring holds MANY camera-like co-located blocks (several 1536 blocks at
  // different FOVs, plus 256/512/768/2304 blocks). Runs 3-6 tried size-sticky, |t|-sticky and
  // |t|>4000 selectors over the shared 16-slot table; all bounced.
  //
  // Run-6 correction: the "world-absent" frames were an observer artifact. The 16-slot newest-wins table
  // was full on 174 of 175 samples while 144 distinct buckets competed for it, so the main camera's slot
  // was evicted on roughly half the samples. Tracking now reads a dedicated latch filled on the CBV path
  // (see CamBucket), keyed by (size, FOV), so eviction and |t| heuristics are no longer involved. The |t|
  // heuristic was also position-dependent (it would fail near the world origin).
  float prev_fov = 0;
  float prev_tmag = 0;
  int prev_key = -1;           // bucket identity (size, FOV*10) of the previous tracked sample
  UINT64 prev_size = 0;
  std::uint64_t prev_seq = 0;  // latch sequence number of the previous tracked sample (stale detection)
  // Mode FOV tracking: histogram of FOVs for world-present 1536-byte buckets.
  int fov_histogram[121] = {}; // index 0..120, we use 30..120
  int fov_histogram_count = 0;
};
CamTrack g_cam_track = {};

// Dedicated camera latch, filled in record_cbv under g_state_mutex. One bucket per (size, FOV*10) of a
// co-located block whose projection FOV is 30..120 deg. Hit counts accumulate for the whole session so
// the main camera is the most-created 1536-byte bucket, not whichever block survived table eviction.
struct CamBucket {
  bool used = false;
  UINT64 size = 0;
  int fov_key = 0;  // round(fov * 10)
  std::uint64_t hits = 0;
  std::uint64_t seq = 0;  // global CBV sequence number of the latest update
  float fov = 0;
  float proj[16] = {};
  float view[16] = {};
  float t[3] = {};
  UINT64 va = 0;
  ULONGLONG stamp = 0;
  bool dumped = false;
};
CamBucket g_cam_buckets[kCamBucketMax] = {};
std::uint64_t g_cam_seq = 0;

// Field-dump handshake between sample() (requests) and the CBV hook (copies while the slot is live).
// Both sides run under g_state_mutex.
int g_dump_request = -1;  // bucket index to copy on its next update, or -1
int g_dump_request_sample = 0;  // sample index when the request was queued (requests expire)
bool g_dump_ready = false;
struct DumpCopy {
  std::uint8_t bytes[kFieldDumpBytes];
  std::size_t size;
  UINT64 va;
  std::uint64_t frame;
};
DumpCopy g_dump_copies[kDumpCopies] = {};
int g_dump_copy_count = 0;
std::uint64_t g_dump_base_frame = 0;
float g_dump_fov = 0;
std::uint64_t g_dump_hits = 0;
std::atomic<std::uint64_t> g_present_seq{0};
int g_dump_count_main = 0;
int g_dump_count_sibling = 0;
int g_sample_index = 0;

// Batched logging: sample() collects its lines and writes them with one file open, so the Present
// thread does one write per second instead of one per line.
thread_local std::string* t_log_batch = nullptr;

void* g_large_example = nullptr;
UINT64 g_large_width = 0;

struct Slot {
  void* resource = nullptr;
  UINT64 width = 0;
  UINT64 gpu_va = 0;
  void* mapped = nullptr;
  ULONGLONG last_scan = 0;
  ULONGLONG last_write = 0;
  UINT64 window = 0;
  bool large = false;
};

struct History {
  void* address = nullptr;
  float values[16] = {};
  bool used = false;
};

struct Queued {
  void* cpu = nullptr;
  void* resource = nullptr;
  UINT64 width = 0;
  matrix_detector::Kind kind = matrix_detector::Kind::None;
  matrix_detector::Layout layout = matrix_detector::Layout::RowMajor;
  float matrix[16] = {};
  float translation[3] = {};
  bool has_translation = false;
  float fov_y = 0;
  bool near_ok = false;
  bool far_ok = false;
  float near_z = 0;
  float far_z = 0;
  int rigid_matches = 0;
  UINT64 window = 0;
  bool changed = false;
  bool medium = false;
  int score = 0;
};

Slot g_slots[kTrackedMax] = {};
int g_slot_count = 0;
History g_history[kHistoryMax] = {};
int g_history_next = 0;
Queued g_queue[kQueueMax] = {};
int g_queue_count = 0;
int g_queue_dropped = 0;
int g_large_cursor = 0;
void* g_modest_resource = nullptr;
UINT64 g_modest_window = 0;
std::atomic<bool> g_worker_started{false};

std::wstring module_path(HMODULE module) {
  std::wstring buf(MAX_PATH, L'\0');
  for (;;) {
    const DWORD wrote = GetModuleFileNameW(module, buf.data(), static_cast<DWORD>(buf.size()));
    if (wrote == 0) {
      return {};
    }
    if (wrote < buf.size() - 1) {
      buf.resize(wrote);
      return buf;
    }
    buf.resize(buf.size() * 2);
  }
}

std::wstring parent_dir(const std::wstring& path) {
  const auto slash = path.find_last_of(L"\\/");
  if (slash == std::wstring::npos) {
    return {};
  }
  return path.substr(0, slash);
}

void write_raw(const std::string& text) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_path.empty() || text.empty()) {
    return;
  }
  HANDLE file = CreateFileW(g_log_path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return;
  }
  DWORD written = 0;
  WriteFile(file, text.data(), static_cast<DWORD>(text.size()), &written, nullptr);
  CloseHandle(file);
}

void log_line(const std::string& text) {
  if (t_log_batch != nullptr) {
    t_log_batch->append(text);
    t_log_batch->append("\r\n");
    return;
  }
  write_raw(text + "\r\n");
}

// Collects every log line written on this thread during its lifetime and flushes them in one write.
struct LogBatch {
  std::string text;
  std::string* previous = nullptr;
  LogBatch() {
    text.reserve(64 * 1024);
    previous = t_log_batch;
    t_log_batch = &text;
  }
  ~LogBatch() {
    t_log_batch = previous;
    write_raw(text);
  }
  LogBatch(const LogBatch&) = delete;
  LogBatch& operator=(const LogBatch&) = delete;
};

void logf(const char* fmt, ...) {
  char buffer[1400];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  buffer[sizeof(buffer) - 1] = '\0';
  log_line(buffer);
}

void rotate_log() {
  if (g_log_path.empty() || GetFileAttributesW(g_log_path.c_str()) == INVALID_FILE_ATTRIBUTES) {
    return;
  }
  const std::wstring previous = parent_dir(g_log_path) + L"\\UPLOAD_OBSERVE.prev.log";
  MoveFileExW(g_log_path.c_str(), previous.c_str(), MOVEFILE_REPLACE_EXISTING);
}

void* vtable_slot(void* object, int index) {
  if (object == nullptr) {
    return nullptr;
  }
  auto** table = *reinterpret_cast<void***>(object);
  if (table == nullptr) {
    return nullptr;
  }
  return table[index];
}

bool hook_target(void* target, void* detour, void** original, const char* name) {
  if (target == nullptr || detour == nullptr || original == nullptr) {
    logf("hook skip %s null", name);
    return false;
  }
  bool already = false;
  MH_STATUS created = MH_OK;
  MH_STATUS enabled = MH_OK;
  {
    std::lock_guard<std::mutex> lock(g_hook_mutex);
    for (int i = 0; i < g_hooked_count; ++i) {
      if (g_hooked[i] == target) {
        already = true;
        break;
      }
    }
    if (!already && g_hooked_count < static_cast<int>(sizeof(g_hooked) / sizeof(g_hooked[0]))) {
      created = MH_CreateHook(target, detour, original);
      if (created == MH_OK) {
        enabled = MH_EnableHook(target);
        if (enabled == MH_OK) {
          g_hooked[g_hooked_count++] = target;
        }
      }
    }
  }
  if (already) {
    return true;
  }
  if (created != MH_OK || enabled != MH_OK) {
    logf("hook failed %s create=%d enable=%d target=%p", name, static_cast<int>(created), static_cast<int>(enabled), target);
    return false;
  }
  logf("hook enabled %s target=%p thread=%lu", name, target, GetCurrentThreadId());
  return true;
}

bool cpu_upload(const D3D12_HEAP_PROPERTIES& props) {
  if (props.Type == D3D12_HEAP_TYPE_UPLOAD) {
    return true;
  }
  if (props.Type != D3D12_HEAP_TYPE_CUSTOM) {
    return false;
  }
  return props.CPUPageProperty == D3D12_CPU_PAGE_PROPERTY_WRITE_COMBINE || props.CPUPageProperty == D3D12_CPU_PAGE_PROPERTY_WRITE_BACK;
}

const char* layout_name(matrix_detector::Layout layout) {
  return layout == matrix_detector::Layout::ColumnMajor ? "column" : "row";
}

const char* kind_name(matrix_detector::Kind kind) {
  switch (kind) {
    case matrix_detector::Kind::Perspective:
      return "perspective";
    case matrix_detector::Kind::Rigid:
      return "rigid";
    case matrix_detector::Kind::CoLocated:
      return "co-located";
    case matrix_detector::Kind::None:
      return "none";
  }
  return "none";
}

Slot* find_slot(void* resource) {
  for (int i = 0; i < g_slot_count; ++i) {
    if (g_slots[i].resource == resource) {
      return &g_slots[i];
    }
  }
  return nullptr;
}

bool remember_values(void* address, const float* values, bool* changed) {
  *changed = false;
  for (const History& row : g_history) {
    if (!row.used || row.address != address) {
      continue;
    }
    for (int i = 0; i < 16; ++i) {
      if (std::fabs(row.values[i] - values[i]) > 0.001f) {
        *changed = true;
        break;
      }
    }
    return true;
  }
  return false;
}

void store_values(void* address, const float* values) {
  for (History& row : g_history) {
    if (row.used && row.address == address) {
      std::memcpy(row.values, values, sizeof(row.values));
      return;
    }
  }
  History& row = g_history[g_history_next];
  g_history_next = (g_history_next + 1) % kHistoryMax;
  row.used = true;
  row.address = address;
  std::memcpy(row.values, values, sizeof(row.values));
}

void queue_hit(const Queued& hit) {
  if (g_queue_count >= kQueueMax) {
    ++g_queue_dropped;
    return;
  }
  g_queue[g_queue_count++] = hit;
}

void consider(void* resource, UINT64 resource_width, void* region, std::size_t region_bytes, UINT64 window) {
  if (region == nullptr || region_bytes < 64 || region_bytes > kModestCopy) {
    return;
  }
  const matrix_detector::Match match = matrix_detector::scan(region, region_bytes);
  g_scan_count.fetch_add(1);
  if (match.kind == matrix_detector::Kind::None) {
    return;
  }
  Queued hit;
  hit.resource = resource;
  hit.width = resource_width;
  hit.window = window;
  hit.kind = match.kind;
  hit.rigid_matches = match.rigid_matches;
  const bool use_perspective = match.kind != matrix_detector::Kind::Rigid;
  const float* values = use_perspective ? match.perspective : match.rigid;
  const std::size_t offset = use_perspective ? match.perspective_offset : match.rigid_offset;
  hit.cpu = static_cast<std::uint8_t*>(region) + offset;
  hit.layout = use_perspective ? match.perspective_layout : match.rigid_layout;
  std::memcpy(hit.matrix, values, sizeof(hit.matrix));
  if (match.rigid_matches > 0) {
    hit.has_translation = true;
    hit.translation[0] = match.translation[0];
    hit.translation[1] = match.translation[1];
    hit.translation[2] = match.translation[2];
  }
  if (match.perspective_matches > 0) {
    hit.fov_y = match.fov_y;
    hit.near_ok = match.near_ok;
    hit.far_ok = match.far_ok;
    hit.near_z = match.near_z;
    hit.far_z = match.far_z;
  }
  bool changed = false;
  const bool seen = remember_values(hit.cpu, hit.matrix, &changed);
  hit.changed = changed;
  hit.medium = seen;
  store_values(hit.cpu, hit.matrix);
  hit.score = (match.kind == matrix_detector::Kind::CoLocated ? 4 : use_perspective ? 2 : 1) + (hit.medium ? 1 : 0) + (hit.changed ? 1 : 0);
  queue_hit(hit);
}

HRESULT STDMETHODCALLTYPE Map_hook(ID3D12Resource* resource, UINT subresource, const D3D12_RANGE* read_range, void** data);
void STDMETHODCALLTYPE Unmap_hook(ID3D12Resource* resource, UINT subresource, const D3D12_RANGE* written_range);

void scan_slot(Slot& slot, ULONGLONG now) {
  if (slot.large || slot.mapped == nullptr || slot.width < kMinBytes || slot.width > kMaxBytes) {
    return;
  }
  if (slot.last_scan != 0 && now - slot.last_scan < 1000) {
    return;
  }
  slot.last_scan = now;
  consider(slot.resource, slot.width, slot.mapped, static_cast<std::size_t>(slot.width), 0);
}

bool copy_large(Slot& slot, UINT64 length, ULONGLONG now, std::vector<std::uint8_t>* bytes, UINT64* used_offset) {
  if (slot.mapped == nullptr || slot.width < kMinBytes || bytes == nullptr) {
    return false;
  }
  if (slot.last_scan != 0 && now - slot.last_scan < 1000) {
    return false;
  }
  UINT64 offset = slot.window;
  if (offset >= slot.width || slot.width - offset < 64) {
    offset = 0;
  }
  UINT64 available = slot.width - offset;
  if (length > available) {
    length = available;
  }
  if (length > kModestCopy) {
    length = kModestCopy;
  }
  if (length < 64) {
    return false;
  }
  bytes->resize(static_cast<std::size_t>(length));
  std::memcpy(bytes->data(), static_cast<const std::uint8_t*>(slot.mapped) + offset, static_cast<std::size_t>(length));
  slot.last_scan = now;
  slot.window = offset + length;
  if (slot.window >= slot.width) {
    slot.window = 0;
  }
  if (used_offset != nullptr) {
    *used_offset = offset;
  }
  return true;
}

void arm_map(ID3D12Resource* resource) {
  if (g_map_armed || resource == nullptr) {
    return;
  }
  const bool map_ok = hook_target(vtable_slot(resource, kResourceMap), reinterpret_cast<void*>(&Map_hook), reinterpret_cast<void**>(&g_map), "ID3D12Resource::Map");
  const bool unmap_ok = hook_target(vtable_slot(resource, kResourceUnmap), reinterpret_cast<void*>(&Unmap_hook), reinterpret_cast<void**>(&g_unmap), "ID3D12Resource::Unmap");
  g_map_armed = map_ok && unmap_ok;
}

void note_buffer(const D3D12_HEAP_PROPERTIES& props, const D3D12_RESOURCE_DESC* desc, void* created) {
  if (desc == nullptr || created == nullptr || desc->Dimension != D3D12_RESOURCE_DIMENSION_BUFFER) {
    return;
  }
  if (props.Type == D3D12_HEAP_TYPE_READBACK) {
    g_readback_count.fetch_add(1);
    return;
  }
  if (!cpu_upload(props)) {
    return;
  }
  const UINT64 width = desc->Width;
  if (width < kMinBytes) {
    if (cpu_upload(props)) {
      g_small_skip_count.fetch_add(1);
    }
    return;
  }
  const bool large = width > kMaxBytes;
  if (large) {
    g_large_count.fetch_add(1);
  } else {
    g_small_count.fetch_add(1);
  }
  ID3D12Resource* resource = nullptr;
  if (FAILED(static_cast<IUnknown*>(created)->QueryInterface(IID_PPV_ARGS(&resource)))) {
    return;
  }
  const UINT64 gpu_va = resource->GetGPUVirtualAddress();
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (find_slot(resource) == nullptr && g_slot_count < kTrackedMax) {
      g_slots[g_slot_count].resource = resource;
      g_slots[g_slot_count].width = width;
      g_slots[g_slot_count].gpu_va = gpu_va;
      g_slots[g_slot_count].large = large;
      ++g_slot_count;
    }
    if (large && (g_large_example == nullptr || width > g_large_width)) {
      g_large_example = resource;
      g_large_width = width;
    }
  }
  if (large) {
    logf("large_upload resource=%p width=%llu", resource, static_cast<unsigned long long>(width));
  } else if (!g_logged_small) {
    g_logged_small = true;
    logf("track resource=%p width=%llu", resource, static_cast<unsigned long long>(width));
  }
  arm_map(resource);
  resource->Release();
}

HRESULT STDMETHODCALLTYPE Map_hook(ID3D12Resource* resource, UINT subresource, const D3D12_RANGE* read_range, void** data) {
  const HRESULT hr = g_map(resource, subresource, read_range, data);
  if (FAILED(hr)) {
    return hr;
  }
  std::lock_guard<std::mutex> lock(g_state_mutex);
  Slot* slot = find_slot(resource);
  if (slot == nullptr) {
    return hr;
  }
  if (data == nullptr || *data == nullptr) {
    g_map_without_pointer.fetch_add(1);
    return hr;
  }
  slot->mapped = *data;
  return hr;
}

void STDMETHODCALLTYPE Unmap_hook(ID3D12Resource* resource, UINT subresource, const D3D12_RANGE* written_range) {
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    Slot* slot = find_slot(resource);
    if (slot != nullptr) {
      slot->mapped = nullptr;
    }
  }
  g_unmap(resource, subresource, written_range);
  (void)written_range;
}

void arm_names(HRESULT hr, void** resource) {
  static std::atomic<bool> armed{false};
  if (FAILED(hr) || resource == nullptr || *resource == nullptr || armed.load()) {
    return;
  }
  ID3D12Resource* r = nullptr;
  if (SUCCEEDED(static_cast<IUnknown*>(*resource)->QueryInterface(IID_PPV_ARGS(&r))) && r != nullptr) {
    armed = true;
    resource_names::arm(r);
    r->Release();
  }
}

HRESULT STDMETHODCALLTYPE CreateCommittedResource_hook(ID3D12Device* device, const D3D12_HEAP_PROPERTIES* props, D3D12_HEAP_FLAGS flags, const D3D12_RESOURCE_DESC* desc, D3D12_RESOURCE_STATES state, const D3D12_CLEAR_VALUE* clear_value, REFIID riid, void** resource) {
  const HRESULT hr = g_create_committed(device, props, flags, desc, state, clear_value, riid, resource);
  arm_names(hr, resource);
  if (SUCCEEDED(hr) && props != nullptr && resource != nullptr && *resource != nullptr) {
    note_buffer(*props, desc, *resource);
  }
  return hr;
}

HRESULT STDMETHODCALLTYPE CreatePlacedResource_hook(ID3D12Device* device, ID3D12Heap* heap, UINT64 offset, const D3D12_RESOURCE_DESC* desc, D3D12_RESOURCE_STATES state, const D3D12_CLEAR_VALUE* clear_value, REFIID riid, void** resource) {
  const HRESULT hr = g_create_placed(device, heap, offset, desc, state, clear_value, riid, resource);
  arm_names(hr, resource);
  if (SUCCEEDED(hr) && heap != nullptr && resource != nullptr && *resource != nullptr) {
    const D3D12_HEAP_DESC heap_desc = heap->GetDesc();
    note_buffer(heap_desc.Properties, desc, *resource);
  }
  return hr;
}

// Resolve a CBV's GPU virtual address to a tracked CPU buffer. Returns the CPU pointer for the
// range [va, va+size) and fills slot/offset. Only buffers currently mapped by the engine are usable.
bool resolve_va(UINT64 va, UINT64 size, void** cpu_out, Slot** slot_out, UINT64* offset_out) {
  if (va == 0 || size == 0) {
    return false;
  }
  for (int i = 0; i < g_slot_count; ++i) {
    Slot& slot = g_slots[i];
    if (slot.gpu_va == 0 || slot.mapped == nullptr) {
      continue;
    }
    if (va < slot.gpu_va || size > slot.width) {
      continue;
    }
    const UINT64 offset = va - slot.gpu_va;
    if (offset + size <= slot.width) {
      *cpu_out = static_cast<std::uint8_t*>(slot.mapped) + offset;
      *slot_out = &slot;
      *offset_out = offset;
      return true;
    }
  }
  return false;
}

// SEH-guarded content scan of a resolved CBV range. The mapped pointer can dangle if the engine
// frees the ring, so the read is isolated in __try. No C++ objects with destructors live in this frame.
void cbv_scan_seh(UINT64 size, void* cpu, void* resource, UINT64 width, UINT64 offset, CbvLog* out) {
  __try {
    const std::size_t scan_bytes = size < kModestCopy ? static_cast<std::size_t>(size) : kModestCopy;
    if (scan_bytes < 64) {
      return;
    }
    const matrix_detector::Match match = matrix_detector::scan(cpu, scan_bytes);
    if (match.kind == matrix_detector::Kind::None) {
      return;
    }
    out->kind = match.kind;
    out->resource = resource;
    out->width = width;
    out->offset = offset;
    out->rigid_matches = match.rigid_matches;
    out->perspective_matches = match.perspective_matches;
    const bool use_perspective = match.kind != matrix_detector::Kind::Rigid;
    const float* values = use_perspective ? match.perspective : match.rigid;
    const std::size_t moff = use_perspective ? match.perspective_offset : match.rigid_offset;
    out->cpu = static_cast<std::uint8_t*>(cpu) + moff;
    out->layout = use_perspective ? match.perspective_layout : match.rigid_layout;
    std::memcpy(out->matrix, values, sizeof(out->matrix));
    if (match.rigid_matches > 0) {
      out->has_translation = true;
      out->translation[0] = match.translation[0];
      out->translation[1] = match.translation[1];
      out->translation[2] = match.translation[2];
    }
    if (match.perspective_matches > 0) {
      out->fov_y = match.fov_y;
      out->near_ok = match.near_ok;
      out->far_ok = match.far_ok;
      out->near_z = match.near_z;
      out->far_z = match.far_z;
    }
    // For co-located buffers, capture BOTH the projection and the view explicitly so the camera
    // owner can be correlated (constant projection + moving view).
    if (match.kind == matrix_detector::Kind::CoLocated) {
      if (match.perspective_matches > 0) {
        out->has_proj = true;
        out->proj_layout = match.perspective_layout;
        out->proj_fov_y = match.fov_y;
        std::memcpy(out->proj_matrix, match.perspective, sizeof(out->proj_matrix));
      }
      if (match.rigid_matches > 0) {
        out->has_view = true;
        out->view_layout = match.rigid_layout;
        std::memcpy(out->view_matrix, match.rigid, sizeof(out->view_matrix));
        out->view_translation[0] = match.translation[0];
        out->view_translation[1] = match.translation[1];
        out->view_translation[2] = match.translation[2];
      }
    }
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
}

// SEH-guarded byte copy out of a live mapped slot. Plain C types only in this frame.
std::size_t copy_seh(void* dst, const void* src, std::size_t bytes) {
  __try {
    std::memcpy(dst, src, bytes);
    return bytes;
  } __except (EXCEPTION_EXECUTE_HANDLER) {
    return 0;
  }
}

bool matrices_equal(const float* a, const float* b, float tolerance) {
  for (int i = 0; i < 16; ++i) {
    if (std::fabs(a[i] - b[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

// Called with g_state_mutex held, while the CBV's slot is live (inside the CreateConstantBufferView hook).
void latch_camera(const CbvLog& entry) {
  if (entry.kind != matrix_detector::Kind::CoLocated || !entry.has_proj || !entry.has_view) {
    return;
  }
  if (!(entry.proj_fov_y >= 30.f && entry.proj_fov_y <= 120.f)) {
    return;
  }
  const int key = static_cast<int>(std::floor(entry.proj_fov_y * 10.f + 0.5f));
  int index = -1;
  int free_index = -1;
  int weakest = -1;
  for (int i = 0; i < kCamBucketMax; ++i) {
    const CamBucket& b = g_cam_buckets[i];
    if (!b.used) {
      if (free_index < 0) {
        free_index = i;
      }
      continue;
    }
    if (b.size == entry.size && b.fov_key == key) {
      index = i;
      break;
    }
    if (weakest < 0 || b.hits < g_cam_buckets[weakest].hits) {
      weakest = i;
    }
  }
  if (index < 0) {
    if (free_index >= 0) {
      index = free_index;
    } else if (weakest >= 0 && g_cam_buckets[weakest].hits < 16 && g_dump_request != weakest) {
      // Only displace a bucket that never became a recurring camera.
      index = weakest;
    } else {
      return;
    }
    g_cam_buckets[index] = CamBucket{};
    g_cam_buckets[index].used = true;
    g_cam_buckets[index].size = entry.size;
    g_cam_buckets[index].fov_key = key;
  }
  CamBucket& b = g_cam_buckets[index];
  ++b.hits;
  b.seq = ++g_cam_seq;
  b.fov = entry.proj_fov_y;
  std::memcpy(b.proj, entry.proj_matrix, sizeof(b.proj));
  std::memcpy(b.view, entry.view_matrix, sizeof(b.view));
  b.t[0] = entry.view_translation[0];
  b.t[1] = entry.view_translation[1];
  b.t[2] = entry.view_translation[2];
  b.va = entry.va;
  b.stamp = entry.stamp;
  if (g_dump_request == index && !g_dump_ready && entry.block_cpu != nullptr) {
    const std::uint64_t frame = g_present_seq.load(std::memory_order_relaxed);
    if (g_dump_copy_count == 0) {
      g_dump_base_frame = frame;
    }
    if (frame > g_dump_base_frame + 1 || g_dump_copy_count >= kDumpCopies) {
      g_dump_ready = g_dump_copy_count > 0;
      g_dump_request = -1;
    } else {
      DumpCopy& d = g_dump_copies[g_dump_copy_count];
      const std::size_t want = static_cast<std::size_t>(std::min<UINT64>(entry.size, kFieldDumpBytes));
      if (copy_seh(d.bytes, entry.block_cpu, want) == want) {
        d.size = want;
        d.va = entry.va;
        d.frame = frame - g_dump_base_frame;
        g_dump_fov = entry.proj_fov_y;
        g_dump_hits = b.hits;
        ++g_dump_copy_count;
      }
    }
  }
}

void record_cbv(const D3D12_CONSTANT_BUFFER_VIEW_DESC& desc, void* cpu, Slot* slot, UINT64 offset) {
  CbvLog entry;
  entry.size = desc.SizeInBytes;
  entry.va = desc.BufferLocation;
  entry.cpu = cpu;
  entry.block_cpu = cpu;
  entry.resource = slot != nullptr ? slot->resource : nullptr;
  entry.width = slot != nullptr ? slot->width : 0;
  entry.offset = offset;
  entry.stamp = GetTickCount64();
  cbv_scan_seh(desc.SizeInBytes, cpu, entry.resource, entry.width, offset, &entry);
  if (entry.kind != matrix_detector::Kind::None) {
    g_cbv_matrix.fetch_add(1);
  }
  std::lock_guard<std::mutex> lock(g_state_mutex);
  latch_camera(entry);
  // Newest-wins per (size, kind, rounded FOV). Deduping by size alone makes the 50.73 main camera
  // and the 90 aux camera (both 1536) collide in one slot, so only the last-created survives and
  // the snapshot's FOV bounces. Including the rounded FOV gives each camera its own slot so both
  // are present in every snapshot and the sticky selector can lock onto the main camera.
  const int entry_fov_bucket = static_cast<int>(std::floor(entry.fov_y + 0.5f));
  for (int i = 0; i < g_cbv_log_count; ++i) {
    const int slot_fov_bucket = static_cast<int>(std::floor(g_cbv_log[i].fov_y + 0.5f));
    if (g_cbv_log[i].size == entry.size && g_cbv_log[i].kind == entry.kind && slot_fov_bucket == entry_fov_bucket) {
      g_cbv_log[i] = entry;
      return;
    }
  }
  if (g_cbv_log_count < kCbvLogMax) {
    g_cbv_log[g_cbv_log_count++] = entry;
    return;
  }
  // Full: replace the oldest.
  int oldest = 0;
  for (int i = 1; i < g_cbv_log_count; ++i) {
    if (g_cbv_log[i].stamp < g_cbv_log[oldest].stamp) {
      oldest = i;
    }
  }
  g_cbv_log[oldest] = entry;
}

std::atomic<bool> g_full_rate{false};

void STDMETHODCALLTYPE CreateCBV_hook(ID3D12Device* device, const D3D12_CONSTANT_BUFFER_VIEW_DESC* desc, D3D12_CPU_DESCRIPTOR_HANDLE handle) {
  g_create_cbv(device, desc, handle);
  if(desc) scanner_inputs::note(handle.ptr,{desc->BufferLocation,desc->SizeInBytes});
  else scanner_inputs::note(handle.ptr,{});
  if (desc == nullptr || desc->SizeInBytes == 0) {
    return;
  }
  if (!g_full_rate.load(std::memory_order_relaxed) && !native_stereo::cb_probe_recording()) {
    // Snowdrop creates CBVs per draw on every worker thread; the observer below takes one process-wide lock per call,
    // which serialised the workers (perf run 2026-10-05: GPU idle, engine-bound). Sample 1 in 32 per thread.
    thread_local unsigned calls = 0;
    if ((++calls & 31u) != 0) {
      // Camera blocks (1536 bytes) still reach the first-person projection override on every call.
      if (desc->SizeInBytes == 1536) {
        void* camera_cpu = nullptr;
        Slot* camera_slot = nullptr;
        UINT64 camera_offset = 0;
        bool resolved = false;
        {
          std::lock_guard<std::mutex> lock(g_state_mutex);
          resolved = resolve_va(desc->BufferLocation, desc->SizeInBytes, &camera_cpu, &camera_slot, &camera_offset);
        }
        if (resolved) native_stereo::note_camera_cbv(camera_cpu, desc->SizeInBytes);
      }
      return;
    }
  }
  g_cbv_total.fetch_add(1);
  void* cpu = nullptr;
  Slot* slot = nullptr;
  UINT64 offset = 0;
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!resolve_va(desc->BufferLocation, desc->SizeInBytes, &cpu, &slot, &offset)) {
      return;
    }
  }
  g_cbv_resolved.fetch_add(1);
  // RE-08: optional per-eye write (default off; see src/camera/eye_write.cpp). Runs before the observer
  // records the block so the logs show what the GPU will read.
  native_stereo::note_camera_cbv(cpu, desc->SizeInBytes);
  native_stereo::note_cbv(cpu, desc->SizeInBytes);
  eye_write::apply(cpu, desc->SizeInBytes);
  record_cbv(*desc, cpu, slot, offset);
}

DWORD WINAPI modest_worker(LPVOID) {
  for (;;) {
    Sleep(1000);
    void* resource = nullptr;
    UINT64 width = 0;
    UINT64 window = 0;
    std::vector<std::uint8_t> copied;
    {
      std::lock_guard<std::mutex> lock(g_state_mutex);
      if (g_slot_count == 0) {
        continue;
      }
      for (int n = 0; n < g_slot_count; ++n) {
        const int index = (g_large_cursor + n) % g_slot_count;
        Slot& slot = g_slots[index];
        const bool modest = slot.mapped != nullptr && slot.width > kMaxBytes && slot.width <= kModestMax;
        if (!modest) {
          continue;
        }
        UINT64 offset = slot.window;
        if (offset >= slot.width || slot.width - offset < 64) {
          offset = 0;
        }
        UINT64 length = kModestCopy;
        if (length > slot.width - offset) {
          length = slot.width - offset;
        }
        if (length < 64) {
          continue;
        }
        copied.resize(static_cast<std::size_t>(length));
        std::memcpy(copied.data(), static_cast<const std::uint8_t*>(slot.mapped) + offset, static_cast<std::size_t>(length));
        slot.window = offset + length;
        if (slot.window >= slot.width) {
          slot.window = 0;
        }
        resource = slot.resource;
        width = slot.width;
        window = offset;
        g_modest_resource = resource;
        g_modest_window = window;
        g_large_cursor = index + 1;
        break;
      }
    }
    if (copied.empty()) {
      continue;
    }
    std::lock_guard<std::mutex> lock(g_state_mutex);
    consider(resource, width, copied.data(), copied.size(), window);
  }
  return 0;
}

void log_matrix(const char* label, const float* values) {
  logf("%s %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g %.5g",
       label,
       values[0], values[1], values[2], values[3],
       values[4], values[5], values[6], values[7],
       values[8], values[9], values[10], values[11],
       values[12], values[13], values[14], values[15]);
}

// Yaw/pitch from a row-major world->view matrix. The third row (m[8..10]) is -forward_world,
// so forward_world = (-m[8], -m[9], -m[10]). Degrees out.
void view_angles(const float* m, float* yaw_deg, float* pitch_deg) {
  const float fx = -m[8];
  const float fy = -m[9];
  const float fz = -m[10];
  const float pitch = std::asin(std::max(-1.f, std::min(1.f, fy)));
  const float yaw = std::atan2(fx, fz);
  *pitch_deg = pitch * 57.29578f;
  *yaw_deg = yaw * 57.29578f;
}

// Angle between two rotation parts (the 3x3 of two row-major 4x4 matrices), in degrees.
// Convention-independent: uses trace(a * b^T) = 1 + 2*cos(theta).
float view_rot_delta(const float* a, const float* b) {
  float trace = 0;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float dot = 0;
      for (int k = 0; k < 3; ++k) {
        dot += a[r * 4 + k] * b[c * 4 + k];
      }
      if (r == c) {
        trace += dot;
      }
    }
  }
  const float cos_theta = std::max(-1.f, std::min(1.f, (trace - 1.f) * 0.5f));
  return std::acos(cos_theta) * 57.29578f;
}

// Signed angle delta wrapped to [-180, 180].
float angle_delta(float from, float to) {
  float d = to - from;
  while (d > 180.f) {
    d -= 360.f;
  }
  while (d < -180.f) {
    d += 360.f;
  }
  return d;
}

}  // namespace

namespace upload {

void install() {
  HMODULE self = nullptr;
  if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&install), &self)) {
    return;
  }
  g_log_path = parent_dir(module_path(self)) + L"\\UPLOAD_OBSERVE.log";
  rotate_log();
  eye_write::install();
  logf("upload observer build %s %s modest_worker=%s cbv_table_every=%d field_dump_every=%d field_dump_max=%d dump_copies=%d gameplay_tmag_min=%.0f",
       __DATE__, __TIME__, kEnableModestWorker ? "on" : "off", kCbvTableEvery, kFieldDumpEvery, kFieldDumpMax, kDumpCopies, kGameplayTmagMin);
  if (kEnableModestWorker && !g_worker_started.exchange(true)) {
    HANDLE thread = CreateThread(nullptr, 0, modest_worker, nullptr, 0, nullptr);
    if (thread != nullptr) {
      CloseHandle(thread);
    } else {
      g_worker_started.store(false);
    }
  }
}

void on_present() {
  g_present_seq.fetch_add(1, std::memory_order_relaxed);
}

void watch_device(ID3D12Device* device) {
  if (device == nullptr) {
    return;
  }
  hook_target(vtable_slot(device, kCreateCommittedResource), reinterpret_cast<void*>(&CreateCommittedResource_hook), reinterpret_cast<void**>(&g_create_committed), "ID3D12Device::CreateCommittedResource");
  hook_target(vtable_slot(device, kCreatePlacedResource), reinterpret_cast<void*>(&CreatePlacedResource_hook), reinterpret_cast<void**>(&g_create_placed), "ID3D12Device::CreatePlacedResource");
  hook_target(vtable_slot(device, kCreateCBV), reinterpret_cast<void*>(&CreateCBV_hook), reinterpret_cast<void**>(&g_create_cbv), "ID3D12Device::CreateConstantBufferView");
}

void sample() {
  eye_write::poll();
  LogBatch batch;
  ++g_sample_index;
  const bool table_sample = (g_sample_index % kCbvTableEvery) == 1;
  int mapped_now = 0;
  int tracked = 0;
  int large_mapped = 0;
  int modest_mapped = 0;
  void* scan_resource = nullptr;
  UINT64 scan_window = 0;
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    tracked = g_slot_count;
    for (int i = 0; i < g_slot_count; ++i) {
      if (g_slots[i].mapped == nullptr) {
        continue;
      }
      ++mapped_now;
      if (g_slots[i].width > kMaxBytes) {
        ++large_mapped;
      }
      if (g_slots[i].width > kMaxBytes && g_slots[i].width <= kModestMax) {
        ++modest_mapped;
      }
    }
    scan_resource = g_modest_resource;
    scan_window = g_modest_window;
  }

  Queued logged[kLogMax] = {};
  int logged_count = 0;
  int dropped = 0;
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    dropped = g_queue_dropped;
    g_queue_dropped = 0;
    while (g_queue_count > 0 && logged_count < kLogMax) {
      int best = 0;
      for (int i = 1; i < g_queue_count; ++i) {
        if (g_queue[i].score > g_queue[best].score) {
          best = i;
        }
      }
      logged[logged_count++] = g_queue[best];
      g_queue[best] = g_queue[g_queue_count - 1];
      --g_queue_count;
    }
    g_queue_count = 0;
  }

  logf("summary tracked=%d small=%llu large=%llu large_example=%p large_width=%llu large_mapped=%d modest_mapped=%d modest_scan=%p modest_window=%llu read=modest-4k below_256=%llu readback=%llu mapped_now=%d scans=%llu logged=%d dropped=%d map_without_pointer=%llu",
       tracked,
       static_cast<unsigned long long>(g_small_count.load()),
       static_cast<unsigned long long>(g_large_count.load()),
       g_large_example,
       static_cast<unsigned long long>(g_large_width),
       large_mapped,
       modest_mapped,
       scan_resource,
       static_cast<unsigned long long>(scan_window),
       static_cast<unsigned long long>(g_small_skip_count.load()),
       static_cast<unsigned long long>(g_readback_count.load()),
       mapped_now,
       static_cast<unsigned long long>(g_scan_count.load()),
       logged_count,
       dropped,
       static_cast<unsigned long long>(g_map_without_pointer.load()));

  // Constant-buffer-view observation: per-frame CBV creation is the per-frame trigger that names the
  // exact buffer (GPU-VA + size) the engine binds. Resolved entries are scanned for matrix content.
  {
    CbvLog snapshot[kCbvLogMax] = {};
    int snapshot_count = 0;
    {
      std::lock_guard<std::mutex> lock(g_state_mutex);
      snapshot_count = g_cbv_log_count;
      for (int i = 0; i < g_cbv_log_count; ++i) {
        snapshot[i] = g_cbv_log[i];
      }
    }
    logf("cbv total=%llu resolved=%llu matrix=%llu logged=%d",
         static_cast<unsigned long long>(g_cbv_total.load()),
         static_cast<unsigned long long>(g_cbv_resolved.load()),
         static_cast<unsigned long long>(g_cbv_matrix.load()),
         snapshot_count);
    for (int i = 0; table_sample && i < snapshot_count; ++i) {
      const CbvLog& c = snapshot[i];
      const char* kind_text = kind_name(c.kind);
      char fov_buf[32] = {};
      char near_buf[32] = {};
      char far_buf[32] = {};
      const char* fov_text = "absent";
      const char* near_text = "absent";
      const char* far_text = "absent";
      if (c.kind != matrix_detector::Kind::Rigid && c.perspective_matches > 0) {
        snprintf(fov_buf, sizeof(fov_buf), "%.2f", c.fov_y);
        fov_text = fov_buf;
      }
      if (c.near_ok) {
        snprintf(near_buf, sizeof(near_buf), "%.4f", c.near_z);
        near_text = near_buf;
      }
      if (c.far_ok) {
        snprintf(far_buf, sizeof(far_buf), "%.1f", c.far_z);
        far_text = far_buf;
      }
      const char* translation = "absent";
      char translation_buf[80] = {};
      if (c.has_translation) {
        snprintf(translation_buf, sizeof(translation_buf), "%.3f %.3f %.3f", c.translation[0], c.translation[1], c.translation[2]);
        translation = translation_buf;
      }
      // Camera-candidate heuristic: a co-located buffer whose projection has a gameplay-plausible
      // FOV (40..110 deg) AND whose view carries a world-scale translation (magnitude > 20). Batch
      // transform buffers have small/permuted translations and are excluded.
      bool camlike = false;
      if (c.kind == matrix_detector::Kind::CoLocated && c.has_proj && c.has_view) {
        const float tx = c.view_translation[0], ty = c.view_translation[1], tz = c.view_translation[2];
        const float mag = std::sqrt(tx * tx + ty * ty + tz * tz);
        camlike = c.proj_fov_y > 40.f && c.proj_fov_y < 110.f && mag > 20.f;
      }
      const char* camlike_text = camlike ? "YES" : "no";
      logf("cbv size=%llu va=0x%llx cpu=%p resource=%p width=%llu offset=%llu kind=%s layout=%s fov=%s near=%s far=%s translation=%s rigid=%d persp=%d camlike=%s",
           static_cast<unsigned long long>(c.size),
           static_cast<unsigned long long>(c.va),
           c.cpu,
           c.resource,
           static_cast<unsigned long long>(c.width),
           static_cast<unsigned long long>(c.offset),
           kind_text,
           layout_name(c.layout),
           fov_text,
           near_text,
           far_text,
           translation,
           c.rigid_matches,
           c.perspective_matches,
           camlike_text);
      if (c.kind != matrix_detector::Kind::None) {
        log_matrix("cbvmatrix", c.matrix);
      }
      if (c.has_proj) {
        logf("cbvproj layout=%s fov=%.2f", layout_name(c.proj_layout), c.proj_fov_y);
        log_matrix("cbvprojm", c.proj_matrix);
      }
      if (c.has_view) {
        char vt_buf[80] = {};
        snprintf(vt_buf, sizeof(vt_buf), "%.3f %.3f %.3f", c.view_translation[0], c.view_translation[1], c.view_translation[2]);
        logf("cbvview layout=%s translation=%s", layout_name(c.view_layout), vt_buf);
        log_matrix("cbvviewm", c.view_matrix);
      }
    }

    // Camera latch report. One compact line per latched (size, FOV) bucket, with a flag that says
    // whether that bucket carries the same view as the main camera. Blocks sharing the main view are the
    // set a per-eye write would have to treat consistently (RE-08/RE-09 side-effect ledger).
    CamBucket buckets[kCamBucketMax] = {};
    {
      std::lock_guard<std::mutex> lock(g_state_mutex);
      for (int i = 0; i < kCamBucketMax; ++i) {
        buckets[i] = g_cam_buckets[i];
      }
    }
    // Main camera: among world-present 1536-byte buckets, pick the one matching the mode FOV.
    // If mode not established, pick the one with max |t|.
    // If no world-present 1536-byte bucket, then no main camera (world-absent).
    int main_index = -1;
    bool found_world_present = false;
    float max_tmag = 0;
    int max_tmag_index = -1;

    // First, update the FOV histogram for world-present 1536-byte buckets and find the max tmag among them.
    for (int i = 0; i < kCamBucketMax; ++i) {
      const CamBucket& b = buckets[i];
      if (!b.used || b.size != kMainCamSize) {
        continue;
      }
      float tx = b.t[0], ty = b.t[1], tz = b.t[2];
      float tmag = std::sqrt(tx * tx + ty * ty + tz * tz);
      if (tmag > 4000.0f) {
        found_world_present = true;
        // Update histogram
        int fov_bucket = static_cast<int>(std::floor(b.fov + 0.5f));
        if (fov_bucket >= 30 && fov_bucket <= 120) {
          g_cam_track.fov_histogram[fov_bucket]++;
          g_cam_track.fov_histogram_count++;
        }
        // Track max tmag for fallback
        if (tmag > max_tmag) {
          max_tmag = tmag;
          max_tmag_index = i;
        }
      }
    }

    // If we found any world-present 1536-byte bucket, compute the mode FOV.
    int mode_fov = -1;
    if (found_world_present && g_cam_track.fov_histogram_count > 0) {
      int mode_count = 0;
      for (int fov = 30; fov <= 120; ++fov) {
        if (g_cam_track.fov_histogram[fov] > mode_count) {
          mode_count = g_cam_track.fov_histogram[fov];
          mode_fov = fov;
        }
      }
    }

    // Now, select the main camera:
    if (found_world_present) {
      if (mode_fov >= 0) {
        // Look for the first world-present 1536-byte bucket that matches the mode FOV.
        for (int i = 0; i < kCamBucketMax; ++i) {
          const CamBucket& b = buckets[i];
          if (!b.used || b.size != kMainCamSize) {
            continue;
          }
          float tx = b.t[0], ty = b.t[1], tz = b.t[2];
          float tmag = std::sqrt(tx * tx + ty * ty + tz * tz);
          if (tmag > 4000.0f) {
            int fov_bucket = static_cast<int>(std::floor(b.fov + 0.5f));
            if (fov_bucket == mode_fov) {
              main_index = i;
              break;
            }
          }
        }
      }
      // If we didn't find a mode-matched bucket (shouldn't happen if we have a mode_fov, but just in case) or mode not established,
      // fall back to the max tmag we found.
      if (main_index < 0) {
        main_index = max_tmag_index;
      }
    }
    // If not found_world_present, main_index remains -1.
    const ULONGLONG now_tick = GetTickCount64();
    {
      std::string line = "camset";
      char item[160] = {};
      for (int i = 0; i < kCamBucketMax; ++i) {
        const CamBucket& b = buckets[i];
        if (!b.used) {
          continue;
        }
        const float bt = std::sqrt(b.t[0] * b.t[0] + b.t[1] * b.t[1] + b.t[2] * b.t[2]);
        const bool same_view = main_index >= 0 && i != main_index && matrices_equal(b.view, buckets[main_index].view, 1e-3f);
        snprintf(item, sizeof(item), " [%s%llu/%.2f hits=%llu age=%llums |t|=%.1f%s]",
                 i == main_index ? "MAIN " : "",
                 static_cast<unsigned long long>(b.size), b.fov,
                 static_cast<unsigned long long>(b.hits),
                 static_cast<unsigned long long>(now_tick - b.stamp),
                 bt,
                 same_view ? " sameview" : "");
        line += item;
      }
      logf("%s", line.c_str());
    }

    if (main_index < 0) {
      logf("camdelta no-camera (no co-located 30..120 deg block latched yet)");
    } else {
      const CamBucket& cam = buckets[main_index];
      const int key = static_cast<int>(cam.size) * 10000 + cam.fov_key;
      float yaw = 0;
      float pitch = 0;
      view_angles(cam.view, &yaw, &pitch);
      const float tx = cam.t[0];
      const float ty = cam.t[1];
      const float tz = cam.t[2];
      const float tmag = std::sqrt(tx * tx + ty * ty + tz * tz);
      char delta_buf[160] = {};
      const char* verdict = "IDLE";
      const bool camera_switch = g_cam_track.have_prev && g_cam_track.prev_key != key;
      const bool stale = g_cam_track.have_prev && !camera_switch && cam.seq == g_cam_track.prev_seq;
      if (stale) {
        // The main bucket was not refreshed since the previous sample (menu, loading, pause). No delta.
        logf("camdelta stale size=%llu fov=%.2f age=%llums (main camera not updated since last sample)",
             static_cast<unsigned long long>(cam.size), cam.fov,
             static_cast<unsigned long long>(now_tick - cam.stamp));
      } else {
        if (g_cam_track.have_prev && !camera_switch) {
          const float dtx = tx - g_cam_track.prev_translation[0];
          const float dty = ty - g_cam_track.prev_translation[1];
          const float dtz = tz - g_cam_track.prev_translation[2];
          const float dtrans = std::sqrt(dtx * dtx + dty * dty + dtz * dtz);
          const float drot = view_rot_delta(g_cam_track.prev_matrix, cam.view);
          const float dyaw = angle_delta(g_cam_track.prev_yaw, yaw);
          const float dpitch = angle_delta(g_cam_track.prev_pitch, pitch);
          const bool moved = dtrans > 0.1f;
          const bool looked = drot > 1.0f;
          if (moved && looked) {
            verdict = "BOTH";
          } else if (moved) {
            verdict = "MOVED";
          } else if (looked) {
            verdict = "LOOKED";
          }
          if (moved) {
            ++g_cam_track.moved_samples;
          }
          if (looked) {
            ++g_cam_track.looked_samples;
          }
          g_cam_track.max_dtrans = std::max(g_cam_track.max_dtrans, dtrans);
          g_cam_track.max_drot = std::max(g_cam_track.max_drot, drot);
          snprintf(delta_buf, sizeof(delta_buf), "dtrans=%.4f drot=%.2f dyaw=%.2f dpitch=%.2f", dtrans, drot, dyaw, dpitch);
        } else if (camera_switch) {
          // Main bucket identity changed (e.g. hit counts overtook). Re-baseline without a delta.
          snprintf(delta_buf, sizeof(delta_buf), "camera-switch prevsize=%llu prevfov=%.2f newsize=%llu newfov=%.2f",
                   static_cast<unsigned long long>(g_cam_track.prev_size), g_cam_track.prev_fov,
                   static_cast<unsigned long long>(cam.size), cam.fov);
        } else {
          snprintf(delta_buf, sizeof(delta_buf), "first-sample");
        }
        logf("camdelta size=%llu fov=%.2f hits=%llu trans=%.3f %.3f %.3f |t|=%.4f yaw=%.2f pitch=%.2f %s verdict=%s moved=%d looked=%d maxdt=%.4f maxdr=%.2f",
             static_cast<unsigned long long>(cam.size),
             cam.fov,
             static_cast<unsigned long long>(cam.hits),
             tx, ty, tz,
             tmag,
             yaw, pitch,
             delta_buf,
             verdict,
             g_cam_track.moved_samples,
             g_cam_track.looked_samples,
             g_cam_track.max_dtrans,
             g_cam_track.max_drot);
        g_cam_track.have_prev = true;
        std::memcpy(g_cam_track.prev_translation, cam.t, sizeof(g_cam_track.prev_translation));
        std::memcpy(g_cam_track.prev_matrix, cam.view, sizeof(g_cam_track.prev_matrix));
        g_cam_track.prev_yaw = yaw;
        g_cam_track.prev_pitch = pitch;
        g_cam_track.prev_stamp = now_tick;
        g_cam_track.prev_fov = cam.fov;
        g_cam_track.prev_tmag = tmag;
        g_cam_track.prev_key = key;
        g_cam_track.prev_size = cam.size;
        g_cam_track.prev_seq = cam.seq;
      }
    }

    // Field dump: print a dump set copied on the CBV path (if one is ready), then queue the next request.
    // Main block: one set every kFieldDumpEvery samples while in gameplay, up to kFieldDumpMax sets.
    // Each recurring sibling bucket: one set. A set = every copy of that bucket in frames N and N+1.
    {
      static DumpCopy copies[kDumpCopies];
      int copy_count = 0;
      float dump_fov = 0;
      std::uint64_t dump_hits = 0;
      {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        if (g_dump_request >= 0 && g_sample_index - g_dump_request_sample > 3) {
          // The bucket stopped updating. Keep what was collected; never block later dumps.
          g_dump_ready = g_dump_copy_count > 0;
          g_dump_request = -1;
        }
        if (g_dump_ready) {
          copy_count = g_dump_copy_count;
          for (int i = 0; i < copy_count; ++i) {
            copies[i] = g_dump_copies[i];
          }
          dump_fov = g_dump_fov;
          dump_hits = g_dump_hits;
          g_dump_ready = false;
          g_dump_copy_count = 0;
        }
        if (!g_dump_ready && g_dump_request < 0 && g_dump_copy_count == 0 && main_index >= 0) {
          const CamBucket& mb = g_cam_buckets[main_index];
          const float mt = std::sqrt(mb.t[0] * mb.t[0] + mb.t[1] * mb.t[1] + mb.t[2] * mb.t[2]);
          const bool gameplay = mt > kGameplayTmagMin && (now_tick - mb.stamp) < 500;
          const bool main_due = gameplay && g_dump_count_main < kFieldDumpMax && (g_sample_index % kFieldDumpEvery) == 5;
          if (main_due && mb.used) {
            g_dump_request = main_index;
            g_dump_request_sample = g_sample_index;
            ++g_dump_count_main;
          } else if (gameplay && g_dump_count_sibling < kSiblingDumpMax && (g_sample_index % kFieldDumpEvery) == 0) {
            for (int i = 0; i < kCamBucketMax; ++i) {
              if (i != main_index && g_cam_buckets[i].used && !g_cam_buckets[i].dumped && g_cam_buckets[i].hits >= 16 &&
                  (now_tick - g_cam_buckets[i].stamp) < 500) {
                g_cam_buckets[i].dumped = true;
                g_dump_request = i;
                g_dump_request_sample = g_sample_index;
                ++g_dump_count_sibling;
                break;
              }
            }
          }
        }
      }
      for (int c = 0; c < copy_count; ++c) {
        const DumpCopy& d = copies[c];
        logf("camfield begin size=%llu fov=%.2f va=0x%llx hits=%llu bytes=%llu copy=%d/%d frame=+%llu (read-only copy taken inside CreateConstantBufferView)",
             static_cast<unsigned long long>(d.size), dump_fov,
             static_cast<unsigned long long>(d.va),
             static_cast<unsigned long long>(dump_hits),
             static_cast<unsigned long long>(d.size),
             c + 1, copy_count,
             static_cast<unsigned long long>(d.frame));
        for (std::size_t off = 0; off + 16 <= d.size; off += 16) {
          float f[4] = {};
          std::memcpy(f, d.bytes + off, sizeof(f));
          logf("camfield +0x%03llx %.9g %.9g %.9g %.9g", static_cast<unsigned long long>(off), f[0], f[1], f[2], f[3]);
        }
        logf("camfield end");
      }
    }
  }

  if (logged_count == 0) {
    if constexpr (kEnableModestWorker) {
      logf("candidates=absent");
    }
    return;
  }
  for (int i = 0; i < logged_count; ++i) {
    const Queued& hit = logged[i];
    const char* fov_text = "absent";
    const char* near_text = "absent";
    const char* far_text = "absent";
    char fov_buf[32] = {};
    char near_buf[32] = {};
    char far_buf[32] = {};
    if (hit.kind != matrix_detector::Kind::Rigid) {
      snprintf(fov_buf, sizeof(fov_buf), "%.2f", hit.fov_y);
      fov_text = fov_buf;
    }
    if (hit.near_ok) {
      snprintf(near_buf, sizeof(near_buf), "%.4f", hit.near_z);
      near_text = near_buf;
    }
    if (hit.far_ok) {
      snprintf(far_buf, sizeof(far_buf), "%.1f", hit.far_z);
      far_text = far_buf;
    }
    const char* translation = "absent";
    char translation_buf[80] = {};
    if (hit.has_translation) {
      snprintf(translation_buf, sizeof(translation_buf), "%.3f %.3f %.3f", hit.translation[0], hit.translation[1], hit.translation[2]);
      translation = translation_buf;
    }
    logf("candidate n=%d cpu=%p resource=%p width=%llu window=%llu type=%s confidence=%s changed=%s layout=%s fov=%s near=%s far=%s translation=%s rigid_matches=%d%s",
         i + 1,
         hit.cpu,
         hit.resource,
         static_cast<unsigned long long>(hit.width),
         static_cast<unsigned long long>(hit.window),
         kind_name(hit.kind),
         hit.medium ? "MEDIUM" : "LOW",
         hit.changed ? "YES" : "NO",
         layout_name(hit.layout),
         fov_text,
         near_text,
         far_text,
         translation,
         hit.rigid_matches,
         hit.rigid_matches > 3 ? " rejected_rest=skinning-like" : "");
    log_matrix("matrix", hit.matrix);
  }
}

void set_full_rate(bool full) { g_full_rate.store(full); }

bool scanner_copy_safe(void* out,const void* src,UINT bytes) {
  __try {std::memcpy(out,src,bytes);return true;}
  __except(EXCEPTION_EXECUTE_HANDLER) {return false;}
}
bool copy_constants(UINT64 address,UINT bytes,void* out) {
  if(!out||!bytes||bytes>1536) return false;
  return copy_mapped(address,bytes,out);
}
bool copy_mapped(UINT64 address,UINT bytes,void* out) {
  if(!out||!bytes||bytes>16384) return false;
  std::lock_guard<std::mutex> lock(g_state_mutex);
  void* cpu=nullptr;Slot* slot=nullptr;UINT64 offset=0;
  return resolve_va(address,bytes,&cpu,&slot,&offset)&&scanner_copy_safe(out,cpu,bytes);
}
UINT64 present_sequence() {return g_present_seq.load(std::memory_order_relaxed);}
bool copy_live_raw64(ID3D12Resource* resource,UINT64 offset,void* out,UINT* heap_type,UINT* map_result) {
  if(!resource||!out||!heap_type||!map_result) return false;
  *heap_type=0;*map_result=UINT(E_NOTIMPL);
  D3D12_HEAP_PROPERTIES props{};D3D12_HEAP_FLAGS flags{};
  HRESULT hr=resource->GetHeapProperties(&props,&flags);
  *map_result=UINT(hr);if(FAILED(hr)) return false;
  *heap_type=UINT(props.Type);
  // DEFAULT resources cannot be read by Map; never attempt it on those heaps.
  if(!cpu_upload(props)&&props.Type!=D3D12_HEAP_TYPE_GPU_UPLOAD) {*map_result=UINT(E_ACCESSDENIED);return false;}
  const auto desc=resource->GetDesc();
  if(desc.Dimension!=D3D12_RESOURCE_DIMENSION_BUFFER||offset>desc.Width||64>desc.Width-offset||offset>SIZE_MAX-64) {
    *map_result=UINT(E_INVALIDARG);return false;
  }
  if(!g_map||!g_unmap) {*map_result=UINT(E_NOTIMPL);return false;}
  D3D12_RANGE read{SIZE_T(offset),SIZE_T(offset+64)};void* cpu=nullptr;
  hr=g_map(resource,0,&read,&cpu);*map_result=UINT(hr);
  if(FAILED(hr)) return false;
  const bool copied=cpu&&scanner_copy_safe(out,static_cast<unsigned char*>(cpu)+offset,64);
  D3D12_RANGE written{0,0};g_unmap(resource,0,&written);
  return copied;
}

}  // namespace upload
