#include "frame_capture.hpp"

#include "../camera/camera.hpp"
#include "../camera/native_stereo.hpp"

#include <windows.h>
#include <wincodec.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

namespace {

std::wstring g_dir;          // scripts folder (directory of AvatarVR.dll)
std::wstring g_trigger_path;
std::wstring g_log_path;
std::string g_last_tag;
std::string g_pending_tag;
int g_pending_frames = 0;  // a tag ending in "_pair" captures two consecutive presents
int g_pending_index = 0;
bool g_first_poll = true;

ID3D12CommandQueue* g_queue = nullptr;
ID3D12Device* g_device = nullptr;
ID3D12CommandAllocator* g_allocator = nullptr;
ID3D12GraphicsCommandList* g_list = nullptr;
ID3D12Fence* g_fence = nullptr;
HANDLE g_event = nullptr;
UINT64 g_fence_value = 0;
ID3D12Resource* g_readback = nullptr;
UINT64 g_readback_size = 0;
ID3D12Resource* g_readback2 = nullptr;  // eye capture: the right eye
UINT64 g_readback2_size = 0;
ID3D12Resource* g_readback3 = nullptr;  // eye capture: the main view's own output (before the UI pass?)
UINT64 g_readback3_size = 0;
std::atomic<bool> g_eyes_pending{false};
std::atomic<int> g_eyes_burst{0};
std::atomic<unsigned long long> g_eyes_burst_id{0};

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

std::string sanitize(const char* text) {
  std::string out;
  for (const char* p = text; *p != '\0' && out.size() < 40; ++p) {
    const char c = *p;
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-') {
      out.push_back(c);
    } else if (c == '\r' || c == '\n') {
      break;
    }
  }
  return out;
}

bool ensure_objects(ID3D12Device* device) {
  if (g_device == device && g_list != nullptr) {
    return true;
  }
  if (g_device != nullptr) {
    return false;  // device changed; keep it simple and refuse
  }
  g_device = device;
  g_device->AddRef();
  if (FAILED(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g_allocator))) ||
      FAILED(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_allocator, nullptr, IID_PPV_ARGS(&g_list))) ||
      FAILED(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_fence)))) {
    log_line("capture: object creation failed");
    return false;
  }
  g_list->Close();
  g_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  return g_event != nullptr;
}

bool ensure_readback(UINT64 size) {
  if (g_readback != nullptr && g_readback_size >= size) {
    return true;
  }
  if (g_readback != nullptr) {
    g_readback->Release();
    g_readback = nullptr;
  }
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = size;
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.Format = DXGI_FORMAT_UNKNOWN;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  if (FAILED(g_device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&g_readback)))) {
    log_line("capture: readback buffer creation failed size=%llu", static_cast<unsigned long long>(size));
    return false;
  }
  g_readback_size = size;
  return true;
}

void barrier(ID3D12Resource* resource, D3D12_RESOURCE_STATES before, D3D12_RESOURCE_STATES after) {
  D3D12_RESOURCE_BARRIER b = {};
  b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
  b.Transition.pResource = resource;
  b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
  b.Transition.StateBefore = before;
  b.Transition.StateAfter = after;
  g_list->ResourceBarrier(1, &b);
}

bool write_bmp(const std::wstring& path, const unsigned char* rgba, UINT width, UINT height, UINT row_pitch, bool bgra) {
  const UINT out_stride = (width * 3 + 3) & ~3u;
  std::vector<unsigned char> image(static_cast<std::size_t>(out_stride) * height);
  for (UINT y = 0; y < height; ++y) {
    const unsigned char* src = rgba + static_cast<std::size_t>(y) * row_pitch;
    unsigned char* dst = image.data() + static_cast<std::size_t>(height - 1 - y) * out_stride;  // bottom-up
    for (UINT x = 0; x < width; ++x) {
      const unsigned char r = bgra ? src[x * 4 + 2] : src[x * 4 + 0];
      const unsigned char g = src[x * 4 + 1];
      const unsigned char b = bgra ? src[x * 4 + 0] : src[x * 4 + 2];
      dst[x * 3 + 0] = b;
      dst[x * 3 + 1] = g;
      dst[x * 3 + 2] = r;
    }
  }
  BITMAPFILEHEADER fh = {};
  BITMAPINFOHEADER ih = {};
  fh.bfType = 0x4D42;
  fh.bfOffBits = sizeof(fh) + sizeof(ih);
  fh.bfSize = fh.bfOffBits + static_cast<DWORD>(image.size());
  ih.biSize = sizeof(ih);
  ih.biWidth = static_cast<LONG>(width);
  ih.biHeight = static_cast<LONG>(height);
  ih.biPlanes = 1;
  ih.biBitCount = 24;
  ih.biCompression = BI_RGB;
  ih.biSizeImage = static_cast<DWORD>(image.size());
  HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return false;
  }
  DWORD written = 0;
  WriteFile(file, &fh, sizeof(fh), &written, nullptr);
  WriteFile(file, &ih, sizeof(ih), &written, nullptr);
  WriteFile(file, image.data(), static_cast<DWORD>(image.size()), &written, nullptr);
  CloseHandle(file);
  return true;
}

// Main menu panel capture (2026-11-11): with F9 (or the mod menu's capture) while the 3D main menu's panel layer is live,
// the layer texture (camera.cpp; premultiplied RGBA, simultaneous-access, read from COMMON) is also saved as
// captures\panel_<date>_<time>.bmp (composited over a grey checkerboard, so see-through areas show the squares) and
// ..._alpha.bmp (opacity: white = opaque, black = transparent). AUTHOR 2026-11-11: the panel is too see-through, the
// sub-windows shown on hover are wrong and the cursor is not round; this shows what actually reaches the headset panel.
void capture_panel() {
  ID3D12Resource* layer = camera::ui_layer_acquire();
  if (layer == nullptr) {
    return;  // not on the main menu panel
  }
  const D3D12_RESOURCE_DESC desc = layer->GetDesc();
  const bool bgra = desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM || desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB ||
                    desc.Format == DXGI_FORMAT_B8G8R8A8_TYPELESS;
  const bool rgba = desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM || desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ||
                    desc.Format == DXGI_FORMAT_R8G8B8A8_TYPELESS;
  ID3D12Device* device = nullptr;
  layer->GetDevice(IID_PPV_ARGS(&device));
  bool ok = g_queue != nullptr && (rgba || bgra) && device != nullptr && ensure_objects(device);
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint = {};
  UINT64 total = 0;
  if (ok) {
    device->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &total);
    ok = ensure_readback(total);
  }
  if (device != nullptr) {
    device->Release();
  }
  if (!ok) {
    log_line("panel capture: skipped (format=%d)", static_cast<int>(desc.Format));
    layer->Release();
    return;
  }
  g_allocator->Reset();
  g_list->Reset(g_allocator, nullptr);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = g_readback;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = footprint;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = layer;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  src.SubresourceIndex = 0;
  g_list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);  // simultaneous-access: no barriers
  g_list->Close();
  ID3D12CommandList* lists[] = {g_list};
  g_queue->ExecuteCommandLists(1, lists);
  g_queue->Signal(g_fence, ++g_fence_value);
  g_fence->SetEventOnCompletion(g_fence_value, g_event);
  const DWORD wait = WaitForSingleObject(g_event, 2000);
  layer->Release();
  if (wait != WAIT_OBJECT_0) {
    log_line("panel capture: copy did not finish within 2 s");
    return;
  }
  void* mapped = nullptr;
  D3D12_RANGE read_range = {0, static_cast<SIZE_T>(total)};
  if (FAILED(g_readback->Map(0, &read_range, &mapped)) || mapped == nullptr) {
    log_line("panel capture: Map failed");
    return;
  }
  const UINT w = static_cast<UINT>(desc.Width), h = desc.Height;
  std::vector<unsigned char> over(static_cast<std::size_t>(w) * h * 4), alpha(static_cast<std::size_t>(w) * h * 4);
  unsigned long long opaque = 0, partial = 0, clear = 0;
  for (UINT y = 0; y < h; ++y) {
    const unsigned char* row = static_cast<const unsigned char*>(mapped) + footprint.Offset +
                               static_cast<std::size_t>(y) * footprint.Footprint.RowPitch;
    for (UINT x = 0; x < w; ++x) {
      const unsigned char* px = row + x * 4;
      const int r = bgra ? px[2] : px[0], g = px[1], b = bgra ? px[0] : px[2], a = px[3];
      const int checker = ((x / 32) + (y / 32)) % 2 ? 150 : 90;  // grey squares where the panel lets the world through
      unsigned char* o = over.data() + (static_cast<std::size_t>(y) * w + x) * 4;
      o[0] = static_cast<unsigned char>(std::min(255, r + checker * (255 - a) / 255));  // premultiplied "over"
      o[1] = static_cast<unsigned char>(std::min(255, g + checker * (255 - a) / 255));
      o[2] = static_cast<unsigned char>(std::min(255, b + checker * (255 - a) / 255));
      o[3] = 255;
      unsigned char* m = alpha.data() + (static_cast<std::size_t>(y) * w + x) * 4;
      m[0] = m[1] = m[2] = static_cast<unsigned char>(a);
      m[3] = 255;
      if (a >= 250) {
        ++opaque;
      } else if (a > 5) {
        ++partial;
      } else {
        ++clear;
      }
    }
  }
  D3D12_RANGE no_write = {0, 0};
  g_readback->Unmap(0, &no_write);
  SYSTEMTIME t = {};
  GetLocalTime(&t);
  wchar_t stamp[32];
  swprintf(stamp, 32, L"%04u%02u%02u_%02u%02u%02u", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
  const std::wstring folder = g_dir + L"\\captures";
  CreateDirectoryW(folder.c_str(), nullptr);
  const std::wstring base = folder + L"\\panel_" + stamp;
  const bool saved = write_bmp(base + L".bmp", over.data(), w, h, w * 4, false) &&
                     write_bmp(base + L"_alpha.bmp", alpha.data(), w, h, w * 4, false);
  const double n = static_cast<double>(w) * h;
  log_line("panel capture: %s %ux%u format=%d | opaque %.1f%%, partly transparent %.1f%%, empty %.1f%%",
           saved ? "saved" : "write failed", w, h, static_cast<int>(desc.Format), 100.0 * opaque / n, 100.0 * partial / n,
           100.0 * clear / n);
}

void capture(IDXGISwapChain* swapchain, const std::string& tag) {
  if (g_queue == nullptr) {
    log_line("capture %s: no swapchain queue recorded", tag.c_str());
    return;
  }
  IDXGISwapChain3* sc3 = nullptr;
  if (FAILED(swapchain->QueryInterface(IID_PPV_ARGS(&sc3)))) {
    log_line("capture %s: IDXGISwapChain3 unavailable", tag.c_str());
    return;
  }
  const UINT index = sc3->GetCurrentBackBufferIndex();
  sc3->Release();
  ID3D12Resource* back = nullptr;
  // "_v2" (anywhere in the tag): the created second view's output texture. It is a simultaneous-access texture,
  // so it is copied from COMMON without barriers (implicit promotion to COPY_SOURCE, decays after execution).
  // "_ui": the game-UI layer texture (main-menu panel / HUD quad, camera.cpp), also simultaneous-access; it never
  // reaches the backbuffer while the main-menu panel is on (unattended menu navigation, 2026-10-22).
  const bool ui = tag.find("_ui") != std::string::npos;
  const bool view2 = ui || tag.find("_v2") != std::string::npos;
  if (ui) {
    back = camera::ui_layer_acquire();
    if (back == nullptr) {
      log_line("capture %s: no UI layer", tag.c_str());
      return;
    }
  } else if (view2) {
    back = native_stereo::acquire_view2_output();
    if (back == nullptr) {
      log_line("capture %s: no second-view output held", tag.c_str());
      return;
    }
  } else if (FAILED(swapchain->GetBuffer(index, IID_PPV_ARGS(&back)))) {
    log_line("capture %s: GetBuffer failed", tag.c_str());
    return;
  }
  const D3D12_RESOURCE_DESC desc = back->GetDesc();
  const bool rgba = desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM || desc.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
  const bool bgra = desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM || desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  ID3D12Device* device = nullptr;
  back->GetDevice(IID_PPV_ARGS(&device));
  bool ok = (rgba || bgra) && device != nullptr && ensure_objects(device);
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint = {};
  UINT64 total = 0;
  if (ok) {
    device->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &total);
    ok = ensure_readback(total);
  }
  if (device != nullptr) {
    device->Release();
  }
  if (!ok) {
    log_line("capture %s: skipped (format=%d)", tag.c_str(), static_cast<int>(desc.Format));
    back->Release();
    return;
  }
  g_allocator->Reset();
  g_list->Reset(g_allocator, nullptr);
  if (!view2) {
    barrier(back, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_SOURCE);
  }
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = g_readback;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = footprint;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = back;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  src.SubresourceIndex = 0;
  g_list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  if (!view2) {
    barrier(back, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_PRESENT);
  }
  g_list->Close();
  ID3D12CommandList* lists[] = {g_list};
  g_queue->ExecuteCommandLists(1, lists);
  g_queue->Signal(g_fence, ++g_fence_value);
  g_fence->SetEventOnCompletion(g_fence_value, g_event);
  const DWORD wait = WaitForSingleObject(g_event, 2000);
  back->Release();
  if (wait != WAIT_OBJECT_0) {
    log_line("capture %s: copy did not finish within 2 s", tag.c_str());
    return;
  }
  void* mapped = nullptr;
  D3D12_RANGE read_range = {0, static_cast<SIZE_T>(total)};
  if (FAILED(g_readback->Map(0, &read_range, &mapped)) || mapped == nullptr) {
    log_line("capture %s: Map failed", tag.c_str());
    return;
  }
  const std::wstring folder = g_dir + L"\\captures";
  CreateDirectoryW(folder.c_str(), nullptr);
  const std::wstring path = folder + L"\\cap_" + std::wstring(tag.begin(), tag.end()) + L".bmp";
  const bool saved = write_bmp(path, static_cast<const unsigned char*>(mapped) + footprint.Offset,
                               static_cast<UINT>(desc.Width), desc.Height, footprint.Footprint.RowPitch, bgra);
  D3D12_RANGE no_write = {0, 0};
  g_readback->Unmap(0, &no_write);
  log_line("capture %s: %s %llux%u format=%d", tag.c_str(), saved ? "saved" : "write failed",
           static_cast<unsigned long long>(desc.Width), desc.Height, static_cast<int>(desc.Format));
}

bool ensure_buffer(ID3D12Resource** buffer, UINT64* have, UINT64 size) {
  if (*buffer != nullptr && *have >= size) {
    return true;
  }
  if (*buffer != nullptr) {
    (*buffer)->Release();
    *buffer = nullptr;
  }
  D3D12_HEAP_PROPERTIES heap = {};
  heap.Type = D3D12_HEAP_TYPE_READBACK;
  D3D12_RESOURCE_DESC desc = {};
  desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
  desc.Width = size;
  desc.Height = 1;
  desc.DepthOrArraySize = 1;
  desc.MipLevels = 1;
  desc.Format = DXGI_FORMAT_UNKNOWN;
  desc.SampleDesc.Count = 1;
  desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
  if (FAILED(g_device->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                               IID_PPV_ARGS(buffer)))) {
    *buffer = nullptr;
    return false;
  }
  *have = size;
  return true;
}

// Worker thread: encode a BGRA image (top-down, tightly packed) to JPEG with WIC.
void save_jpeg(std::wstring path, std::vector<unsigned char> bgra, UINT width, UINT height) {
  const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
  IWICImagingFactory* factory = nullptr;
  IWICStream* stream = nullptr;
  IWICBitmapEncoder* encoder = nullptr;
  IWICBitmapFrameEncode* frame = nullptr;
  IPropertyBag2* props = nullptr;
  bool ok = SUCCEEDED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory))) &&
            SUCCEEDED(factory->CreateStream(&stream)) && SUCCEEDED(stream->InitializeFromFilename(path.c_str(), GENERIC_WRITE)) &&
            SUCCEEDED(factory->CreateEncoder(GUID_ContainerFormatJpeg, nullptr, &encoder)) &&
            SUCCEEDED(encoder->Initialize(stream, WICBitmapEncoderNoCache)) &&
            SUCCEEDED(encoder->CreateNewFrame(&frame, &props));
  if (ok && props != nullptr) {
    PROPBAG2 option = {};
    option.pstrName = const_cast<LPOLESTR>(L"ImageQuality");
    VARIANT value;
    VariantInit(&value);
    value.vt = VT_R4;
    value.fltVal = 0.92f;
    props->Write(1, &option, &value);
  }
  WICPixelFormatGUID format = GUID_WICPixelFormat24bppBGR;
  std::vector<unsigned char> bgr;
  if (ok) {
    ok = SUCCEEDED(frame->Initialize(props)) && SUCCEEDED(frame->SetSize(width, height)) &&
         SUCCEEDED(frame->SetPixelFormat(&format));
  }
  if (ok) {
    bgr.resize(static_cast<std::size_t>(width) * height * 3);
    for (std::size_t i = 0, n = static_cast<std::size_t>(width) * height; i < n; ++i) {
      bgr[i * 3 + 0] = bgra[i * 4 + 0];
      bgr[i * 3 + 1] = bgra[i * 4 + 1];
      bgr[i * 3 + 2] = bgra[i * 4 + 2];
    }
    ok = SUCCEEDED(frame->WritePixels(height, width * 3, static_cast<UINT>(bgr.size()), bgr.data())) &&
         SUCCEEDED(frame->Commit()) && SUCCEEDED(encoder->Commit());
  }
  if (props != nullptr) props->Release();
  if (frame != nullptr) frame->Release();
  if (encoder != nullptr) encoder->Release();
  if (stream != nullptr) stream->Release();
  if (factory != nullptr) factory->Release();
  if (SUCCEEDED(co)) {
    CoUninitialize();
  }
  const std::string name(path.begin(), path.end());
  log_line("eyes: %s %s (%ux%u)", ok ? "saved" : "JPEG encode FAILED", name.c_str(), width, height);
}

// Present thread, before the original Present: the backbuffer is the left eye of this Present, the held second-view
// output the right eye (pose identity: the same engine frame). Both are copied in one list, then placed side by side.
// Eye pair kept in memory instead of saved (automatic slot sweep).
struct Grab {
  std::vector<unsigned char> pair;  // BGRA, left | right (or left only), top-down
  UINT w = 0, h = 0;                // one eye
  bool has_right = false;
};

void capture_eyes(IDXGISwapChain* swapchain, Grab* grab = nullptr, int burst_index = -1, unsigned long long burst_id = 0) {
  if (g_queue == nullptr) {
    log_line("eyes: no swapchain queue recorded");
    return;
  }
  IDXGISwapChain3* sc3 = nullptr;
  if (FAILED(swapchain->QueryInterface(IID_PPV_ARGS(&sc3)))) {
    return;
  }
  ID3D12Resource* left = nullptr;
  const HRESULT got = sc3->GetBuffer(sc3->GetCurrentBackBufferIndex(), IID_PPV_ARGS(&left));
  sc3->Release();
  if (FAILED(got) || left == nullptr) {
    log_line("eyes: GetBuffer failed");
    return;
  }
  ID3D12Resource* right = native_stereo::acquire_view2_output();  // may be null (stereo paused): left only
  ID3D12Resource* main_out = native_stereo::acquire_main_output();  // the left eye's scene image without the UI pass?
  D3D12_RESOURCE_DESC ld = left->GetDesc();
  // Eye mode (Flat + VR eyes): the left eye is the top-left W x H of the grown backbuffer, not the whole buffer.
  std::uint32_t eye_w = 0, eye_h = 0;
  const bool eye_region = native_stereo::eye_size(&eye_w, &eye_h) && ld.Width >= eye_w && ld.Height >= eye_h;
  if (eye_region) {
    ld.Width = eye_w;
    ld.Height = eye_h;
  }
  D3D12_RESOURCE_DESC rd = right != nullptr ? right->GetDesc() : ld;
  auto readable = [](DXGI_FORMAT f) {
    return f == DXGI_FORMAT_R8G8B8A8_UNORM || f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_UNORM ||
           f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  };
  auto is_bgra = [](DXGI_FORMAT f) { return f == DXGI_FORMAT_B8G8R8A8_UNORM || f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB; };
  if (right != nullptr && (!readable(rd.Format) || rd.Width < ld.Width || rd.Height < ld.Height)) {
    log_line("eyes: right image unusable (format %d, %llux%u vs %llux%u); left only", static_cast<int>(rd.Format),
             static_cast<unsigned long long>(rd.Width), rd.Height, static_cast<unsigned long long>(ld.Width), ld.Height);
    right->Release();
    right = nullptr;
  }
  D3D12_RESOURCE_DESC md = main_out != nullptr ? main_out->GetDesc() : ld;
  if (main_out != nullptr && (!readable(md.Format) || md.Width < ld.Width || md.Height < ld.Height)) {
    log_line("eyes: main view output unusable (format %d, %llux%u)", static_cast<int>(md.Format),
             static_cast<unsigned long long>(md.Width), md.Height);
    main_out->Release();
    main_out = nullptr;
  }
  ID3D12Device* device = nullptr;
  left->GetDevice(IID_PPV_ARGS(&device));
  bool ok = readable(ld.Format) && device != nullptr && ensure_objects(device);
  D3D12_PLACED_SUBRESOURCE_FOOTPRINT lf = {}, rf = {}, mf = {};
  UINT64 ltotal = 0, rtotal = 0, mtotal = 0;
  if (ok) {
    device->GetCopyableFootprints(&ld, 0, 1, 0, &lf, nullptr, nullptr, &ltotal);
    ok = ensure_buffer(&g_readback, &g_readback_size, ltotal);
    if (ok && right != nullptr) {
      device->GetCopyableFootprints(&rd, 0, 1, 0, &rf, nullptr, nullptr, &rtotal);
      ok = ensure_buffer(&g_readback2, &g_readback2_size, rtotal);
    }
    if (ok && main_out != nullptr) {
      device->GetCopyableFootprints(&md, 0, 1, 0, &mf, nullptr, nullptr, &mtotal);
      ok = ensure_buffer(&g_readback3, &g_readback3_size, mtotal);
    }
  }
  if (device != nullptr) {
    device->Release();
  }
  if (!ok) {
    log_line("eyes: skipped (left format %d)", static_cast<int>(ld.Format));
    left->Release();
    if (right != nullptr) right->Release();
    if (main_out != nullptr) main_out->Release();
    return;
  }
  g_allocator->Reset();
  g_list->Reset(g_allocator, nullptr);
  barrier(left, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_SOURCE);
  D3D12_TEXTURE_COPY_LOCATION dst = {};
  dst.pResource = g_readback;
  dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
  dst.PlacedFootprint = lf;
  D3D12_TEXTURE_COPY_LOCATION src = {};
  src.pResource = left;
  src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
  const D3D12_BOX left_box = {0, 0, 0, static_cast<UINT>(ld.Width), ld.Height, 1};
  g_list->CopyTextureRegion(&dst, 0, 0, 0, &src, eye_region ? &left_box : nullptr);
  barrier(left, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_PRESENT);
  if (right != nullptr) {  // simultaneous-access texture: copied from COMMON without barriers
    dst.pResource = g_readback2;
    dst.PlacedFootprint = rf;
    src.pResource = right;
    g_list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  }
  if (main_out != nullptr) {  // also simultaneous-access (same kind of view output texture)
    dst.pResource = g_readback3;
    dst.PlacedFootprint = mf;
    src.pResource = main_out;
    g_list->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
  }
  g_list->Close();
  ID3D12CommandList* lists[] = {g_list};
  g_queue->ExecuteCommandLists(1, lists);
  g_queue->Signal(g_fence, ++g_fence_value);
  g_fence->SetEventOnCompletion(g_fence_value, g_event);
  const DWORD wait = WaitForSingleObject(g_event, 2000);
  left->Release();
  if (right != nullptr) right->Release();
  if (main_out != nullptr) main_out->Release();
  if (wait != WAIT_OBJECT_0) {
    log_line("eyes: copy did not finish within 2 s");
    return;
  }
  const UINT w = static_cast<UINT>(ld.Width), h = ld.Height;
  const UINT out_w = right != nullptr ? w * 2 : w;
  std::vector<unsigned char> image(static_cast<std::size_t>(out_w) * h * 4);
  auto blit_into = [&](std::vector<unsigned char>& image, UINT out_w, ID3D12Resource* buffer, UINT64 total,
                       const D3D12_PLACED_SUBRESOURCE_FOOTPRINT& f, bool bgra, UINT x0) {
    void* mapped = nullptr;
    D3D12_RANGE range = {0, static_cast<SIZE_T>(total)};
    if (FAILED(buffer->Map(0, &range, &mapped)) || mapped == nullptr) {
      return false;
    }
    const unsigned char* base = static_cast<const unsigned char*>(mapped) + f.Offset;
    for (UINT y = 0; y < h; ++y) {
      const unsigned char* s = base + static_cast<std::size_t>(y) * f.Footprint.RowPitch;
      unsigned char* d = image.data() + (static_cast<std::size_t>(y) * out_w + x0) * 4;
      for (UINT x = 0; x < w; ++x) {
        d[x * 4 + 0] = bgra ? s[x * 4 + 0] : s[x * 4 + 2];
        d[x * 4 + 1] = s[x * 4 + 1];
        d[x * 4 + 2] = bgra ? s[x * 4 + 2] : s[x * 4 + 0];
        d[x * 4 + 3] = 255;
      }
    }
    D3D12_RANGE none = {0, 0};
    buffer->Unmap(0, &none);
    return true;
  };
  if (!blit_into(image, out_w, g_readback, ltotal, lf, is_bgra(ld.Format), 0) ||
      (right != nullptr && !blit_into(image, out_w, g_readback2, rtotal, rf, is_bgra(rd.Format), w))) {
    log_line("eyes: Map failed");
    return;
  }
  if (grab != nullptr) {
    grab->pair = std::move(image);
    grab->w = w;
    grab->h = h;
    grab->has_right = right != nullptr;
    return;
  }
  // Third file: |left - right| amplified 4x, and the mean difference in CAPTURE.log. With "Zero eye distance" both
  // eyes see from one point, so anything bright here differs between the views in time, not in viewpoint.
  if (right != nullptr) {
    std::vector<unsigned char> diff(static_cast<std::size_t>(w) * h * 4);
    unsigned long long sum = 0, strong = 0;
    for (UINT y = 0; y < h; ++y) {
      const unsigned char* l = image.data() + static_cast<std::size_t>(y) * out_w * 4;
      const unsigned char* r = l + static_cast<std::size_t>(w) * 4;
      unsigned char* d = diff.data() + static_cast<std::size_t>(y) * w * 4;
      for (UINT x = 0; x < w; ++x) {
        int t = 0;
        for (int c = 0; c < 3; ++c) {
          const int v = std::abs(static_cast<int>(l[x * 4 + c]) - static_cast<int>(r[x * 4 + c]));
          t += v;
          d[x * 4 + c] = static_cast<unsigned char>(std::min(255, v * 4));
        }
        d[x * 4 + 3] = 255;
        sum += t;
        strong += t > 60 ? 1 : 0;
      }
    }
    log_line("eyes: mean |L-R| %.2f, %.1f%% of pixels differ strongly", static_cast<double>(sum) / (3.0 * w * h),
             100.0 * static_cast<double>(strong) / (static_cast<double>(w) * h));
    if (grab == nullptr) {
      SYSTEMTIME t;
      GetLocalTime(&t);
      wchar_t dname[96];
      wchar_t burst[40] = {};
      if (burst_index >= 0) swprintf(burst, 40, L"_mvburst_%llu_f%d", burst_id, burst_index);
      swprintf(dname, 96, L"\\eyes_%04u%02u%02u_%02u%02u%02u%s_diff.jpg", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond, burst);
      const std::wstring folder = g_dir + L"\\captures";
      CreateDirectoryW(folder.c_str(), nullptr);
      std::thread(save_jpeg, folder + dname, std::move(diff), w, h).detach();
    }
  }
  // Second file: the left eye as presented | the main view's own output. Where they differ is what the game adds
  // after the view renders (UI, and anything else composited only into the presented frame).
  std::vector<unsigned char> ui;
  if (main_out != nullptr) {
    ui.resize(static_cast<std::size_t>(w) * 2 * h * 4);
    if (!blit_into(ui, w * 2, g_readback, ltotal, lf, is_bgra(ld.Format), 0) ||
        !blit_into(ui, w * 2, g_readback3, mtotal, mf, is_bgra(md.Format), w)) {
      ui.clear();
    }
  }
  const std::wstring folder = g_dir + L"\\captures";
  CreateDirectoryW(folder.c_str(), nullptr);
  SYSTEMTIME t;
  GetLocalTime(&t);
  wchar_t name[96];
  wchar_t burst[40] = {};
  if (burst_index >= 0) swprintf(burst, 40, L"_mvburst_%llu_f%d", burst_id, burst_index);
  swprintf(name, 96, L"\\eyes_%04u%02u%02u_%02u%02u%02u%s%s.jpg", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond,
           burst, right != nullptr ? L"" : L"_leftonly");
  log_line("eyes: copied %ux%u %s; encoding", w, h, right != nullptr ? "left | right" : "left only (no right image)");
  std::thread(save_jpeg, folder + name, std::move(image), out_w, h).detach();
  if (!ui.empty()) {
    std::wstring ui_name = name;
    ui_name.replace(ui_name.size() - 4, 4, L"_mainview.jpg");
    std::thread(save_jpeg, folder + ui_name, std::move(ui), w * 2, h).detach();
  } else {
    log_line("eyes: no main view output held (second file skipped)");
  }
}


// ---- automatic slot sweep -----------------------------------------------------------------------------------
// Steps through the main-view slots (off, 1..N) by itself, grabs the eye pair after each change, and writes ONE small
// contact sheet of the right eye (cell 0 = slot sharing off, cell k = slot k) plus numbers in CAPTURE.log:
//   mono   = mean |left - right| (near 0: the right eye became a copy of the left)
//   dR, dL = share of pixels that changed against the off cell in the right / left eye (head motion moves both;
//            something new drawn only into the right eye, like the UI, raises dR alone).
constexpr int kSweepSettle = 45;  // Presents after a slot change before the grab
std::atomic<int> g_sweep_request{0};  // 1 = slots, 2 = fields, 3 = slots with zero eye distance (foliage),
                                      // 4 = fields with zero eye distance
int g_sweep_mode = 0;                 // as above (Present thread)
double g_foliage_off = 0, g_foliage_best = 0;
int g_foliage_best_step = -1;
int g_cols = 4;
int g_sweep_step = -1;  // -1 idle, 0 = off, k = slot k
int g_sweep_wait = 0;
int g_sweep_slots = 0;
std::vector<unsigned char> g_sweep_l0, g_sweep_r0;  // the off cell, full size
std::vector<unsigned char> g_sweep_sheet;
UINT g_sheet_w = 0, g_sheet_h = 0, g_cell_w = 0, g_cell_h = 0;

// 3x5 digits for the cell labels.
const unsigned short kDigits[11] = {0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9, 0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF,
                                    0x4927 /* L */};

void set_step(int step) {
  if (g_sweep_mode == 1 || g_sweep_mode == 3) {
    native_stereo::set_share_override(step);
  } else {
    native_stereo::set_field_variant(step);
  }
}

void end_sweep() {
  native_stereo::force_zero_sep(false);
  native_stereo::set_share_override(-1);
  native_stereo::set_field_variant(0);
  g_sweep_step = -1;
}

void draw_label(int cell, int value) {
  const int cols = g_cols;
  const int x0 = (cell % cols) * static_cast<int>(g_cell_w) + 6, y0 = (cell / cols) * static_cast<int>(g_cell_h) + 6;
  const int scale = 6;
  const int digits = value >= 10 ? 2 : 1;  // value 10 is drawn as "10"; -1 as "L"
  const bool letter = value < 0;
  for (int y = -3; y < 5 * scale + 3; ++y) {
    for (int x = -3; x < digits * 4 * scale + 3; ++x) {
      const int px = x0 + x, py = y0 + y;
      if (px < 0 || py < 0 || px >= static_cast<int>(g_sheet_w) || py >= static_cast<int>(g_sheet_h)) continue;
      bool on = false;
      if (x >= 0 && y >= 0 && y < 5 * scale) {
        const int di = x / (4 * scale), cx = (x % (4 * scale)) / scale;
        const int d = letter ? 10 : digits == 2 ? (di == 0 ? value / 10 : value % 10) : value;
        on = di < digits && cx < 3 && ((kDigits[d] >> (14 - ((y / scale) * 3 + cx))) & 1);
      }
      unsigned char* p = g_sweep_sheet.data() + (static_cast<std::size_t>(py) * g_sheet_w + px) * 4;
      p[0] = p[1] = p[2] = on ? 255 : 0;
    }
  }
}

void sweep_cell(int cell, const Grab& g, bool left_eye = false, int label = -2) {
  const int cols = g_cols;
  const UINT stride = g.w * (g.has_right ? 2 : 1);
  const UINT xoff = g.has_right && !left_eye ? g.w : 0;
  for (UINT y = 0; y < g_cell_h; ++y) {
    for (UINT x = 0; x < g_cell_w; ++x) {
      const UINT sx = x * g.w / g_cell_w, sy = y * g.h / g_cell_h;
      const unsigned char* src = g.pair.data() + (static_cast<std::size_t>(sy) * stride + xoff + sx) * 4;
      unsigned char* d = g_sweep_sheet.data() +
                         ((static_cast<std::size_t>((cell / cols) * g_cell_h + y)) * g_sheet_w + (cell % cols) * g_cell_w + x) * 4;
      std::memcpy(d, src, 4);
    }
  }
  draw_label(cell, label == -2 ? cell : label);
}

// Mean |a - b| per channel, and the share of pixels whose summed channel difference exceeds 60.
void compare(const unsigned char* a, const unsigned char* b, std::size_t pixels, double* mean, double* changed) {
  unsigned long long sum = 0, count = 0;
  for (std::size_t i = 0; i < pixels; ++i) {
    const int d = std::abs(a[i * 4] - b[i * 4]) + std::abs(a[i * 4 + 1] - b[i * 4 + 1]) + std::abs(a[i * 4 + 2] - b[i * 4 + 2]);
    sum += d;
    count += d > 60 ? 1 : 0;
  }
  *mean = pixels ? static_cast<double>(sum) / (3.0 * pixels) : 0.0;
  *changed = pixels ? static_cast<double>(count) / pixels : 0.0;
}

// ---- frame lag test ------------------------------------------------------------------------------------------
// Vegetation still differs between the eyes with both eyes at one point and one frustum (2026-10-06 08:29), static
// geometry cancels. Is the right eye simply a frame early or late? Grab 4 consecutive Presents and compare the right
// image of each with the left images of the previous, same and next Present.
std::atomic<bool> g_lag_request{false};
int g_lag_step = -1;
std::vector<unsigned char> g_lag_l[4], g_lag_r[4];
UINT g_lag_w = 0, g_lag_h = 0;

double mean_diff(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
  if (a.size() != b.size() || a.empty()) {
    return -1.0;
  }
  unsigned long long sum = 0;
  for (std::size_t i = 0; i < a.size(); i += 4) {
    sum += std::abs(a[i] - b[i]) + std::abs(a[i + 1] - b[i + 1]) + std::abs(a[i + 2] - b[i + 2]);
  }
  return static_cast<double>(sum) / (3.0 * (a.size() / 4));
}

void lag_present(IDXGISwapChain* swapchain) {
  if (g_lag_step < 0 && g_lag_request.exchange(false)) {
    g_lag_step = 0;
    log_line("lagtest: grabbing 4 consecutive frames");
  }
  if (g_lag_step < 0) {
    return;
  }
  Grab g;
  capture_eyes(swapchain, &g);
  if (g.pair.empty() || !g.has_right) {
    log_line("lagtest: no eye pair; stopped");
    g_lag_step = -1;
    return;
  }
  const std::size_t row = static_cast<std::size_t>(g.w) * 4;
  g_lag_l[g_lag_step].resize(row * g.h);
  g_lag_r[g_lag_step].resize(row * g.h);
  for (UINT y = 0; y < g.h; ++y) {
    std::memcpy(g_lag_l[g_lag_step].data() + y * row, g.pair.data() + y * row * 2, row);
    std::memcpy(g_lag_r[g_lag_step].data() + y * row, g.pair.data() + y * row * 2 + row, row);
  }
  g_lag_w = g.w;
  g_lag_h = g.h;
  if (++g_lag_step < 4) {
    return;
  }
  for (int k = 1; k <= 2; ++k) {
    log_line("lagtest: R%d vs L%d (same) %.2f | vs L%d (previous) %.2f | vs L%d (next) %.2f | L%d vs L%d (one frame of motion) %.2f",
             k, k, mean_diff(g_lag_r[k], g_lag_l[k]), k - 1, mean_diff(g_lag_r[k], g_lag_l[k - 1]), k + 1,
             mean_diff(g_lag_r[k], g_lag_l[k + 1]), k, k - 1, mean_diff(g_lag_l[k], g_lag_l[k - 1]));
  }
  log_line("lagtest: done. The smallest of same/previous/next tells which left frame the right eye matches; "
           "all large means the right eye's animation is not just shifted in time.");
  for (auto& v : g_lag_l) v.clear();
  for (auto& v : g_lag_r) v.clear();
  g_lag_step = -1;
}

void sweep_present(IDXGISwapChain* swapchain) {
  const int request = g_sweep_request.exchange(0);
  if (request != 0 && g_sweep_step < 0) {
    g_sweep_mode = request;
    g_sweep_slots = request == 2 || request == 4 ? native_stereo::field_variant_count() : native_stereo::share_slot_count();
    if (!native_stereo::stereo_running() || g_sweep_slots == 0) {
      log_line("sweep: not started (stereo %s, %d steps known; the slots are found 3 s into stereo)",
               native_stereo::stereo_running() ? "running" : "paused", g_sweep_slots);
      return;
    }
    g_sweep_slots = std::min(g_sweep_slots, 11);
    g_cols = request == 2 || request == 4 ? 3 : 4;
    g_sweep_step = 0;
    g_sweep_wait = kSweepSettle;
    g_foliage_best_step = -1;
    native_stereo::force_zero_sep(request == 3 || request == 4);
    set_step(0);
    log_line("sweep: start (%s), %d steps; keep the head still",
             request == 1 ? "view slots" : request == 3 ? "view slots, zero eye distance" :
             request == 4 ? "view fields, zero eye distance" : "view fields", g_sweep_slots);
  }
  if (g_sweep_step < 0 || --g_sweep_wait > 0) {
    return;
  }
  Grab g;
  capture_eyes(swapchain, &g);
  if (g.pair.empty() || !g.has_right) {
    log_line("sweep: step %d: no eye pair (stereo paused?); stopped", g_sweep_step);
    end_sweep();
    return;
  }
  const std::size_t pixels = static_cast<std::size_t>(g.w) * g.h;
  std::vector<unsigned char> l(pixels * 4), r(pixels * 4);
  for (UINT y = 0; y < g.h; ++y) {
    std::memcpy(l.data() + static_cast<std::size_t>(y) * g.w * 4, g.pair.data() + static_cast<std::size_t>(y) * g.w * 8, g.w * 4);
    std::memcpy(r.data() + static_cast<std::size_t>(y) * g.w * 4, g.pair.data() + static_cast<std::size_t>(y) * g.w * 8 + g.w * 4, g.w * 4);
  }
  if (g_sweep_mode == 3 || g_sweep_mode == 4) {
    // Zero eye distance: keep each step's full eye pair for region analysis (face, hair).
    SYSTEMTIME t;
    GetLocalTime(&t);
    wchar_t step_name[96];
    swprintf(step_name, 96, L"\\captures\\sweepstep_%d_%02d_%02u%02u%02u.jpg", g_sweep_mode, g_sweep_step, t.wHour, t.wMinute,
             t.wSecond);
    std::thread(save_jpeg, g_dir + step_name, g.pair, g.w * 2, g.h).detach();
  }
  if (g_sweep_step == 0) {
    const int cells = g_sweep_slots + 1 + (g_sweep_mode == 2 || g_sweep_mode == 4 ? 1 : 0);  // fields: plus the left eye
    g_cell_w = 1920 / g_cols;
    g_cell_h = static_cast<UINT>(static_cast<double>(g_cell_w) * g.h / g.w);
    g_sheet_w = g_cell_w * g_cols;
    g_sheet_h = g_cell_h * ((cells + g_cols - 1) / g_cols);
    g_sweep_sheet.assign(static_cast<std::size_t>(g_sheet_w) * g_sheet_h * 4, 0);
    g_sweep_l0 = l;
    g_sweep_r0 = r;
  }
  double mono = 0, mono_c = 0, dr = 0, dr_c = 0, dl = 0, dl_c = 0;
  compare(l.data(), r.data(), pixels, &mono, &mono_c);
  compare(r.data(), g_sweep_r0.data(), pixels, &dr, &dr_c);
  compare(l.data(), g_sweep_l0.data(), pixels, &dl, &dl_c);
  (void)dr;
  (void)dl;
  if (g_sweep_mode == 3) {
    // Both eyes at one point with one frustum: static geometry cancels, so mono is the foliage mismatch alone.
    const int off = native_stereo::share_slot_offset(g_sweep_step);
    if (g_sweep_step == 0) {
      g_foliage_off = mono;
    } else if (off != 0x108 && (g_foliage_best_step < 0 || mono < g_foliage_best)) {
      g_foliage_best = mono;  // +0x108 (present pass) makes the right eye a copy of the left: not a fix
      g_foliage_best_step = g_sweep_step;
    }
    log_line("sweep: foliage cell %2d slot %s%03X | |L-R| %.2f (%.0f%% of slot off)%s", g_sweep_step,
             g_sweep_step == 0 ? "off  " : "+0x", g_sweep_step == 0 ? 0 : off, mono,
             g_foliage_off > 0 ? 100.0 * mono / g_foliage_off : 100.0, off == 0x108 ? "  (present pass: copy, ignore)" : "");
  } else if (g_sweep_mode == 1) {
    const int off = native_stereo::share_slot_offset(g_sweep_step);
    log_line("sweep: cell %2d slot %s%03X | mono %.2f%s", g_sweep_step, g_sweep_step == 0 ? "off  " : "+0x",
             g_sweep_step == 0 ? 0 : off, mono, mono < 1.0 ? "  <- right eye = copy of left" : "");
  } else {
    log_line("sweep: cell %2d field variant %d | mono %.2f | right eye changed vs cell 0: %.1f%% of pixels (left: %.1f%%)%s",
             g_sweep_step, g_sweep_step, mono, dr_c * 100.0, dl_c * 100.0, mono < 1.0 ? "  <- right eye = copy of left" : "");
  }
  sweep_cell(g_sweep_step, g);
  ++g_sweep_step;
  if (g_sweep_step <= g_sweep_slots) {
    set_step(g_sweep_step);
    g_sweep_wait = kSweepSettle;
    return;
  }
  if (g_sweep_mode == 2 || g_sweep_mode == 4) {
    sweep_cell(g_sweep_step, g, true, -1);  // reference: the left eye, labelled L
  }
  if (g_sweep_mode == 3 && g_foliage_best_step > 0) {
    log_line("sweep: foliage best slot %d (+0x%03X): |L-R| %.2f vs %.2f with sharing off (%s)", g_foliage_best_step,
             native_stereo::share_slot_offset(g_foliage_best_step), g_foliage_best, g_foliage_off,
             g_foliage_best < 0.5 * g_foliage_off ? "candidate: this slot carries the per-view foliage work"
                                                  : "no slot halves the mismatch");
  }
  end_sweep();
  const std::wstring folder = g_dir + L"\\captures";
  CreateDirectoryW(folder.c_str(), nullptr);
  SYSTEMTIME t;
  GetLocalTime(&t);
  wchar_t name[64];
  swprintf(name, 64, L"\\sweep_%04u%02u%02u_%02u%02u%02u.jpg", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
  log_line("sweep: done; contact sheet of the right eye (cell 0 = off, cell k = %s k%s)",
           g_sweep_mode == 2 || g_sweep_mode == 4 ? "field variant" : "slot",
           g_sweep_mode == 2 || g_sweep_mode == 4 ? ", L = left eye" : "");
  std::thread(save_jpeg, folder + name, std::move(g_sweep_sheet), g_sheet_w, g_sheet_h).detach();
  g_sweep_sheet.clear();
  g_sweep_l0.clear();
  g_sweep_r0.clear();
}

}  // namespace

namespace frame_capture {

void install() {
  g_dir = self_dir();
  if (g_dir.empty()) {
    return;
  }
  g_trigger_path = g_dir + L"\\AvatarVR_capture.txt";
  g_log_path = g_dir + L"\\CAPTURE.log";
  MoveFileExW(g_log_path.c_str(), (g_dir + L"\\CAPTURE.prev.log").c_str(), MOVEFILE_REPLACE_EXISTING);
  log_line("frame capture build %s %s pid=%lu trigger=AvatarVR_capture.txt", __DATE__, __TIME__, GetCurrentProcessId());
}

void set_queue(IUnknown* device_or_queue) {
  if (device_or_queue == nullptr) {
    return;
  }
  ID3D12CommandQueue* queue = nullptr;
  if (SUCCEEDED(device_or_queue->QueryInterface(IID_PPV_ARGS(&queue))) && queue != nullptr) {
    if (g_queue != nullptr) {
      g_queue->Release();
    }
    g_queue = queue;  // keep the reference
    log_line("capture: swapchain queue recorded %p", queue);
  }
}

void poll() {
  if (g_trigger_path.empty()) {
    return;
  }
  char text[64] = {};
  HANDLE file = CreateFileW(g_trigger_path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    g_first_poll = false;
    return;
  }
  DWORD read = 0;
  ReadFile(file, text, sizeof(text) - 1, &read, nullptr);
  CloseHandle(file);
  text[read < sizeof(text) ? read : sizeof(text) - 1] = '\0';
  const std::string tag = sanitize(text);
  if (g_first_poll) {
    // A tag already present at launch is not a request.
    g_first_poll = false;
    g_last_tag = tag;
    return;
  }
  if (!tag.empty() && tag != g_last_tag) {
    g_last_tag = tag;
    g_pending_tag = tag;
    // "_pair" captures two consecutive presents, "_quad" four (enough to see both phases of a 2-cycle).
    const bool pair = tag.size() > 5 && tag.compare(tag.size() - 5, 5, "_pair") == 0;
    const bool quad = tag.size() > 5 && tag.compare(tag.size() - 5, 5, "_quad") == 0;
    g_pending_frames = quad ? 4 : pair ? 2 : 1;
    g_pending_index = 0;
  }
}

void request_eyes() {
  native_stereo::want_main_output();  // held from the next frame's output callback, released after the capture
  g_eyes_pending.store(true);
}
void request_eyes_burst(unsigned long long id) {
  native_stereo::want_main_output();
  g_eyes_burst_id.store(id);
  g_eyes_burst.store(3);
}
void request_sweep() { g_sweep_request.store(1); }
void request_field_sweep() { g_sweep_request.store(2); }
void request_lag_test() { g_lag_request.store(true); }
void request_foliage_sweep() { g_sweep_request.store(3); }
void request_zero_field_sweep() { g_sweep_request.store(4); }

void on_present(IDXGISwapChain* swapchain) {
  if (swapchain != nullptr) {
    sweep_present(swapchain);
    lag_present(swapchain);
  }
  if (swapchain != nullptr && g_eyes_pending.exchange(false)) {
    capture_eyes(swapchain);
    capture_panel();  // main menu panel layer too, when it is live
  }
  if (swapchain != nullptr) {
    int remaining = g_eyes_burst.load();
    if (remaining > 0 && g_eyes_burst.compare_exchange_strong(remaining, remaining - 1)) {
      capture_eyes(swapchain, nullptr, 3 - remaining, g_eyes_burst_id.load());
      if (remaining > 1) native_stereo::want_main_output();
    }
  }
  if (g_pending_tag.empty() || swapchain == nullptr) {
    return;
  }
  std::string tag = g_pending_tag;
  if (g_pending_frames > 1) {
    tag += "_f" + std::to_string(g_pending_index);
  }
  ++g_pending_index;
  if (g_pending_index >= g_pending_frames) {
    g_pending_tag.clear();
  }
  if (tag.find("_both") != std::string::npos) {
    // Both eyes of the same Present: the backbuffer (left) and the held second-view output (right).
    capture(swapchain, tag + "_L");
    capture(swapchain, tag + "_v2R");
    return;
  }
  capture(swapchain, tag);
}

}  // namespace frame_capture
