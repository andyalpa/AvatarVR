#pragma once
// One settings file: scripts\AvatarVR_settings.ini holds a [section] with the verbatim text of each former
// scripts\AvatarVR_<section>.txt, so every module keeps its own format and only changes where the text lives.
// Callers still pass the old path; only names in kSections are redirected. Debug switches such as
// AvatarVR_hitch.txt stay separate files that exist only while in use, and tuner data files are not touched.
// Writes are atomic (temp file + rename) and readers need no lock.
#include <windows.h>

#include <cstring>
#include <string>
#include <string_view>

namespace settings_file {

inline constexpr const char* kSections[] = {"native", "xr",    "shape",     "monitor", "arms",   "hud",    "moveprobe",
                                            "mounts", "pad",   "dlss",      "scanstereo", "skinik", "iktrace"};
inline constexpr const char kHeader[] =
    "; AvatarVR settings. The menu keeps this file up to date; edit by hand only while the game is closed.\r\n";

namespace detail {

inline bool is_section(std::string_view name) {
  for (const char* s : kSections) {
    if (name == s) return true;
  }
  return false;
}

inline std::wstring dir_of(const std::wstring& path) {
  const auto slash = path.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : path.substr(0, slash);
}

// "...\AvatarVR_native.txt" -> "native"; empty for any file this store does not own.
inline std::string section_of(const std::wstring& path) {
  const auto slash = path.find_last_of(L"\\/");
  const std::wstring leaf = slash == std::wstring::npos ? path : path.substr(slash + 1);
  constexpr std::wstring_view prefix = L"AvatarVR_", suffix = L".txt";
  if (leaf.size() <= prefix.size() + suffix.size() || leaf.compare(0, prefix.size(), prefix) != 0 ||
      leaf.compare(leaf.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return {};
  }
  std::string name;
  for (size_t i = prefix.size(); i < leaf.size() - suffix.size(); ++i) {
    if (leaf[i] > 127) return {};
    name += static_cast<char>(leaf[i]);
  }
  return is_section(name) ? name : std::string();
}

inline std::wstring store_path(const std::wstring& dir) { return dir + L"\\AvatarVR_settings.ini"; }

inline bool read_all(const std::wstring& path, std::string& out) {
  out.clear();
  const HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                  OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) return false;
  LARGE_INTEGER size{};
  bool ok = GetFileSizeEx(file, &size) && size.QuadPart < (1 << 24);
  if (ok && size.QuadPart > 0) {
    out.resize(static_cast<size_t>(size.QuadPart));
    DWORD got = 0;
    ok = ReadFile(file, out.data(), static_cast<DWORD>(out.size()), &got, nullptr) != FALSE;
    out.resize(got);
  }
  CloseHandle(file);
  return ok;
}

// A section starts at a line that is exactly [name] for a known name and runs to the next such line, so a value
// line can never be mistaken for a header unless it repeats a section name in brackets.
inline bool header_at(std::string_view text, size_t line, size_t line_end, std::string_view* name) {
  std::string_view s = text.substr(line, line_end - line);
  if (!s.empty() && s.back() == '\r') s.remove_suffix(1);
  if (s.size() < 3 || s.front() != '[' || s.back() != ']') return false;
  s = s.substr(1, s.size() - 2);
  if (!is_section(s)) return false;
  *name = s;
  return true;
}

// Byte range of a section's whole block, header included. False when it is absent.
inline bool find_block(std::string_view text, std::string_view wanted, size_t* begin, size_t* body, size_t* end) {
  bool inside = false;
  for (size_t line = 0; line < text.size();) {
    size_t next = text.find('\n', line);
    const size_t line_end = next == std::string_view::npos ? text.size() : next;
    next = next == std::string_view::npos ? text.size() : next + 1;
    std::string_view name;
    if (header_at(text, line, line_end, &name)) {
      if (inside) {
        *end = line;
        return true;
      }
      if (name == wanted) {
        inside = true;
        *begin = line;
        *body = next;
      }
    }
    line = next;
  }
  if (inside) *end = text.size();
  return inside;
}

inline bool replace_file(const std::wstring& path, const std::string& text) {
  const std::wstring tmp = path + L".tmp";
  const HANDLE file = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) return false;
  DWORD wrote = 0;
  const bool ok = WriteFile(file, text.data(), static_cast<DWORD>(text.size()), &wrote, nullptr) && wrote == text.size();
  CloseHandle(file);
  if (ok) {
    for (int attempt = 0; attempt < 5; ++attempt) {  // a reader or a virus scanner may hold the target for a moment
      if (MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING)) return true;
      Sleep(2);
    }
  }
  DeleteFileW(tmp.c_str());
  return false;
}

inline std::string with_section(std::string text, std::string_view section, std::string_view value) {
  if (text.empty()) text = kHeader;
  if (text.back() != '\n') text += "\r\n";
  std::string block = "[" + std::string(section) + "]\r\n" + std::string(value);
  if (!block.empty() && block.back() != '\n') block += "\r\n";
  size_t begin = 0, body = 0, end = 0;
  if (find_block(text, section, &begin, &body, &end)) {
    text.replace(begin, end - begin, block);
  } else {
    text += block;
  }
  return text;
}

inline SRWLOCK& write_lock() {
  static SRWLOCK lock = SRWLOCK_INIT;
  return lock;
}

}  // namespace detail

// True when the path is one of the settings now kept in the shared file.
inline bool owns(const std::wstring& path) { return !detail::section_of(path).empty(); }

// Same contract as reading the old file into a zeroed buffer: false when it does not exist.
inline bool read(const std::wstring& path, char* text, size_t size, DWORD* got = nullptr) {
  text[0] = '\0';
  if (got != nullptr) *got = 0;
  const std::string section = detail::section_of(path);
  std::string all, value;
  bool found = false;
  if (!section.empty() && detail::read_all(detail::store_path(detail::dir_of(path)), all)) {
    size_t begin = 0, body = 0, end = 0;
    if (detail::find_block(all, section, &begin, &body, &end)) {
      value = all.substr(body, end - body);
      found = true;
    }
  }
  if (!found && !detail::read_all(path, value)) return false;  // not migrated yet, or dropped in by hand this session
  const size_t n = value.size() < size - 1 ? value.size() : size - 1;
  std::memcpy(text, value.data(), n);
  text[n] = '\0';
  if (got != nullptr) *got = static_cast<DWORD>(n);
  return true;
}

inline bool write(const std::wstring& path, std::string_view value) {
  const std::string section = detail::section_of(path);
  if (section.empty()) return false;
  const std::wstring store = detail::store_path(detail::dir_of(path));
  AcquireSRWLockExclusive(&detail::write_lock());
  std::string all;
  detail::read_all(store, all);
  const std::string next = detail::with_section(all, section, value);
  const bool ok = next == all || detail::replace_file(store, next);
  ReleaseSRWLockExclusive(&detail::write_lock());
  return ok;
}

// At load, before any module reads settings: import every old file, then move it to settings_migrated\ as a backup.
// A file placed by hand between launches therefore replaces its section once. Does nothing in a folder that has none.
inline void migrate(const std::wstring& dir) {
  std::string all;
  detail::read_all(detail::store_path(dir), all);
  const std::string before = all;
  std::wstring moved[sizeof(kSections) / sizeof(kSections[0])];
  size_t count = 0;
  for (const char* s : kSections) {
    const std::wstring old = dir + L"\\AvatarVR_" + std::wstring(s, s + std::strlen(s)) + L".txt";
    std::string value;
    if (detail::read_all(old, value)) {
      all = detail::with_section(all, s, value);
      moved[count++] = std::wstring(s, s + std::strlen(s));
    }
  }
  if (count == 0 || (all != before && !detail::replace_file(detail::store_path(dir), all))) return;
  const std::wstring backup = dir + L"\\settings_migrated";
  CreateDirectoryW(backup.c_str(), nullptr);
  for (size_t i = 0; i < count; ++i) {
    const std::wstring leaf = L"\\AvatarVR_" + moved[i] + L".txt";
    MoveFileExW((dir + leaf).c_str(), (backup + leaf).c_str(), MOVEFILE_REPLACE_EXISTING);
  }
}

}  // namespace settings_file
