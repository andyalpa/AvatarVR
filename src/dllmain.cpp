#include "camera/camera.hpp"
#include "diagnostics/upload_sampler.hpp"
#include "dx12/dx12.hpp"
#include "settings_file.hpp"

#include <windows.h>
#include <bcrypt.h>

#include <memory>
#include <string>
#include <vector>

namespace {

std::wstring module_path(HMODULE module) {
  std::wstring buf(MAX_PATH, L'\0');
  for (;;) {
    DWORD wrote = GetModuleFileNameW(module, buf.data(), static_cast<DWORD>(buf.size()));
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

std::wstring join_path(const std::wstring& dir, const wchar_t* leaf) {
  if (dir.empty()) {
    return leaf;
  }
  if (dir.back() == L'\\' || dir.back() == L'/') {
    return dir + leaf;
  }
  return dir + L'\\' + leaf;
}

std::string wide_to_utf8(const std::wstring& text) {
  if (text.empty()) {
    return {};
  }
  const int bytes = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
  if (bytes <= 0) {
    return {};
  }
  std::string out(static_cast<size_t>(bytes), '\0');
  WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), out.data(), bytes, nullptr, nullptr);
  return out;
}

bool write_bytes(const std::wstring& path, const std::string& text, bool append) {
  const DWORD access = append ? FILE_APPEND_DATA : GENERIC_WRITE;
  const DWORD disposition = append ? OPEN_ALWAYS : CREATE_ALWAYS;
  HANDLE file = CreateFileW(path.c_str(), access, FILE_SHARE_READ, nullptr, disposition, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return false;
  }
  DWORD written = 0;
  const BOOL ok = WriteFile(file, text.data(), static_cast<DWORD>(text.size()), &written, nullptr);
  CloseHandle(file);
  return ok && written == text.size();
}

void rotate_log(const std::wstring& log_path) {
  const std::wstring previous = join_path(parent_dir(log_path), L"LOAD_PROOF.prev.log");
  if (GetFileAttributesW(log_path.c_str()) == INVALID_FILE_ATTRIBUTES) {
    return;
  }
  MoveFileExW(log_path.c_str(), previous.c_str(), MOVEFILE_REPLACE_EXISTING);
}

std::string sha256_file(const std::wstring& path) {
  HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return "ERROR open";
  }

  BCRYPT_ALG_HANDLE algorithm = nullptr;
  NTSTATUS status = BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0);
  if (status < 0) {
    CloseHandle(file);
    return "ERROR algorithm";
  }

  DWORD object_size = 0;
  DWORD ignored = 0;
  status = BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&object_size), sizeof(object_size), &ignored, 0);
  if (status < 0) {
    BCryptCloseAlgorithmProvider(algorithm, 0);
    CloseHandle(file);
    return "ERROR object";
  }

  std::vector<UCHAR> object(object_size);
  BCRYPT_HASH_HANDLE hash = nullptr;
  status = BCryptCreateHash(algorithm, &hash, object.data(), object_size, nullptr, 0, 0);
  if (status < 0) {
    BCryptCloseAlgorithmProvider(algorithm, 0);
    CloseHandle(file);
    return "ERROR hash";
  }

  std::vector<UCHAR> buffer(1 << 20);
  DWORD read = 0;
  while (ReadFile(file, buffer.data(), static_cast<DWORD>(buffer.size()), &read, nullptr) && read > 0) {
    status = BCryptHashData(hash, buffer.data(), read, 0);
    if (status < 0) {
      BCryptDestroyHash(hash);
      BCryptCloseAlgorithmProvider(algorithm, 0);
      CloseHandle(file);
      return "ERROR data";
    }
  }
  CloseHandle(file);

  DWORD hash_size = 0;
  status = BCryptGetProperty(algorithm, BCRYPT_HASH_LENGTH, reinterpret_cast<PUCHAR>(&hash_size), sizeof(hash_size), &ignored, 0);
  if (status < 0) {
    BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(algorithm, 0);
    return "ERROR length";
  }

  std::vector<UCHAR> digest(hash_size);
  status = BCryptFinishHash(hash, digest.data(), hash_size, 0);
  BCryptDestroyHash(hash);
  BCryptCloseAlgorithmProvider(algorithm, 0);
  if (status < 0) {
    return "ERROR finish";
  }

  static const char* hex = "0123456789ABCDEF";
  std::string out;
  out.resize(digest.size() * 2);
  for (size_t i = 0; i < digest.size(); ++i) {
    out[i * 2] = hex[digest[i] >> 4];
    out[i * 2 + 1] = hex[digest[i] & 0x0F];
  }
  return out;
}

std::string module_line(const wchar_t* name) {
  std::string line = "module ";
  line += wide_to_utf8(name);
  line += ": ";
  HMODULE module = GetModuleHandleW(name);
  if (module == nullptr) {
    line += "not loaded\r\n";
    return line;
  }
  const std::wstring path = module_path(module);
  line += "loaded";
  if (!path.empty()) {
    line += " ";
    line += wide_to_utf8(path);
  }
  line += "\r\n";
  return line;
}

struct DetailWork {
  std::wstring log_path;
  std::wstring host_path;
};

DWORD WINAPI detail_thread(LPVOID param) {
  std::unique_ptr<DetailWork> work(static_cast<DetailWork*>(param));
  std::string extra;
  extra += "architecture: x64\r\n";
  extra += "host_sha256: ";
  extra += work->host_path.empty() ? "ERROR host path" : sha256_file(work->host_path);
  extra += "\r\n";
  extra += module_line(L"dxgi.dll");
  extra += module_line(L"d3d12.dll");
  extra += module_line(L"d3d12core.dll");
  extra += "detail_complete\r\n";
  write_bytes(work->log_path, extra, true);
  return 0;
}

std::string header_text(const std::wstring& dll_path, const std::wstring& config_path, const std::wstring& host_path) {
  std::string text;
  text += "Hello from AvatarVR\r\n";
  text += "build: ";
  text += __DATE__;
  text += " ";
  text += __TIME__;
  text += "\r\n";
  text += "dll: ";
  text += wide_to_utf8(dll_path);
  text += "\r\n";
  text += "config: ";
  text += wide_to_utf8(config_path);
  text += "\r\n";
  text += "pid: ";
  text += std::to_string(GetCurrentProcessId());
  text += "\r\n";
  text += "host: ";
  text += wide_to_utf8(host_path);
  text += "\r\n";
  text += "route: RE-03\r\n";
  text += "CANARY Hello from AvatarVR\r\n";
  return text;
}

}  // namespace

BOOL WINAPI DllMain(HINSTANCE instance, DWORD reason, LPVOID) {
  if (reason != DLL_PROCESS_ATTACH) {
    return TRUE;
  }
  DisableThreadLibraryCalls(instance);
  settings_file::migrate(parent_dir(module_path(instance)));  // before any module reads its settings
  camera::install();
  upload::install();
  dx12::install();

  const std::wstring dll_path = module_path(instance);
  const std::wstring host_path = module_path(nullptr);
  const std::wstring config_path = join_path(parent_dir(host_path), L"version.ini");
  const std::wstring log_path = join_path(parent_dir(dll_path), L"LOAD_PROOF.log");

  rotate_log(log_path);
  write_bytes(log_path, header_text(dll_path, config_path, host_path), false);

  auto work = std::make_unique<DetailWork>();
  work->log_path = log_path;
  work->host_path = host_path;
  HANDLE thread = CreateThread(nullptr, 0, detail_thread, work.get(), 0, nullptr);
  if (thread == nullptr) {
    write_bytes(log_path, "detail_error thread\r\n", true);
    return TRUE;
  }
  work.release();
  CloseHandle(thread);
  return TRUE;
}
