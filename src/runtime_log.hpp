#pragma once

#include <windows.h>

#include <initializer_list>
#include <string>
#include <string_view>

namespace runtime_log {

inline constexpr wchar_t kFile[] = L"AvatarVR.log";

inline void rotate_if_large(const std::wstring& scripts) {
  const auto path = scripts + L"\\" + kFile;
  WIN32_FILE_ATTRIBUTE_DATA info{};
  if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &info)) return;
  constexpr DWORD kRotateBytes = 8 * 1024 * 1024;
  if (info.nFileSizeHigh == 0 && info.nFileSizeLow < kRotateBytes) return;
  MoveFileExW(path.c_str(), (scripts + L"\\AvatarVR\\state\\previous.log").c_str(),
              MOVEFILE_REPLACE_EXISTING);
}

// The legacy path supplies the channel label and the directory. Each batch
// is appended in one write so multiple render and input threads can share it.
inline bool append(const std::wstring& legacy_path, std::initializer_list<std::string_view> fragments) {
  const auto separator = legacy_path.find_last_of(L"\\/");
  if (separator == legacy_path.npos) return false;
  auto name = std::wstring_view(legacy_path).substr(separator + 1);
  if (name.ends_with(L".log")) name.remove_suffix(4);
  std::string prefix = "[";
  for (wchar_t character : name) prefix += static_cast<char>(character);
  prefix += "] ";
  std::string text;
  bool at_line_start = true;
  for (auto fragment : fragments) {
    while (!fragment.empty()) {
      if (at_line_start) text += prefix;
      const auto newline = fragment.find('\n');
      at_line_start = newline != fragment.npos;
      if (!at_line_start) {
        text += fragment;
        break;
      }
      text += fragment.substr(0, newline + 1);
      fragment.remove_prefix(newline + 1);
    }
  }
  if (text.empty()) return true;
  if (!at_line_start) text += "\r\n";
  const auto path = legacy_path.substr(0, separator + 1) + kFile;
  HANDLE file = CreateFileW(path.c_str(), FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                            nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) return false;
  DWORD written = 0;
  const bool ok = WriteFile(file, text.data(), static_cast<DWORD>(text.size()), &written, nullptr) && written == text.size();
  CloseHandle(file);
  return ok;
}

}  // namespace runtime_log
