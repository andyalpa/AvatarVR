#include "runtime_log.hpp"
#include <windows.h>

#include <string>

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

std::string read_file(const std::wstring& path) {
  HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    return {};
  }
  LARGE_INTEGER size{};
  if (!GetFileSizeEx(file, &size) || size.QuadPart <= 0 || size.QuadPart > 32 * 1024 * 1024) {
    CloseHandle(file);
    return {};
  }
  std::string text(static_cast<size_t>(size.QuadPart), '\0');
  DWORD read = 0;
  const BOOL ok = ReadFile(file, text.data(), static_cast<DWORD>(text.size()), &read, nullptr);
  CloseHandle(file);
  if (!ok) {
    return {};
  }
  text.resize(read);
  return text;
}

}  // namespace

int wmain() {
  const std::wstring host_path = module_path(nullptr);
  const std::wstring dll_path = join_path(parent_dir(host_path), L"AvatarVR.dll");
  const std::wstring log_path = join_path(parent_dir(dll_path), runtime_log::kFile);

  HMODULE loaded = LoadLibraryW(dll_path.c_str());
  if (loaded == nullptr) {
    const DWORD error = GetLastError();
    fwprintf(stderr, L"LoadLibrary failed %lu %ls\n", error, dll_path.c_str());
    return 1;
  }

  const auto complete = "detail_complete pid=" + std::to_string(GetCurrentProcessId()) + "\r\n";
  const auto failed = "detail_error thread pid=" + std::to_string(GetCurrentProcessId()) + "\r\n";
  std::string log;
  for (int attempt = 0; attempt < 200; ++attempt) {
    log = read_file(log_path);
    if (log.find(complete) != std::string::npos || log.find(failed) != std::string::npos) {
      break;
    }
    Sleep(50);
  }

  const std::string host_utf8 = wide_to_utf8(host_path);
  const bool canary = log.find("CANARY Hello from AvatarVR") != std::string::npos;
  const bool host_named = !host_utf8.empty() && log.find(host_utf8) != std::string::npos;
  const bool finished = log.find(complete) != std::string::npos;
  fwrite(log.data(), 1, log.size(), stdout);
  if (!canary || !host_named || !finished) {
    fwprintf(stderr, L"load proof failed canary=%d host=%d finished=%d\n", canary ? 1 : 0, host_named ? 1 : 0, finished ? 1 : 0);
    return 2;
  }
  return 0;
}
