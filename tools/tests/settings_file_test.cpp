#include "../../src/settings_file.hpp"

#include <cstdio>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++failures;
  }
}

void put(const std::wstring& path, const char* text) {
  const HANDLE f = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  DWORD n = 0;
  WriteFile(f, text, static_cast<DWORD>(std::strlen(text)), &n, nullptr);
  CloseHandle(f);
}

bool exists(const std::wstring& path) { return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES; }

}  // namespace

int main() {
  wchar_t temp[MAX_PATH]{};
  GetTempPathW(MAX_PATH, temp);
  const std::wstring dir = std::wstring(temp) + L"avr_settings_test_" + std::to_wstring(GetCurrentProcessId());
  CreateDirectoryW(dir.c_str(), nullptr);
  const std::wstring native = dir + L"\\AvatarVR_native.txt", arms = dir + L"\\AvatarVR_arms.txt";
  const std::wstring hud = dir + L"\\AvatarVR_hud.txt", other = dir + L"\\AvatarVR_hitch.txt";
  char text[256];
  DWORD got = 0;

  check(settings_file::owns(native) && !settings_file::owns(other), "only listed names are owned");
  check(!settings_file::read(native, text, sizeof(text)), "missing section and file reads as missing");
  check(!settings_file::write(other, "on"), "an unowned name is refused");

  // Old files move in verbatim, including multi-line ones and a value that looks like a header.
  put(native, "create 0.03265 head offscreen autostart");
  put(arms, "arm_ik 1\nhand_pitch -65.0000\n[not_a_section]\nclean_arm_rig 1\n");
  settings_file::migrate(dir);
  check(!exists(native) && exists(dir + L"\\settings_migrated\\AvatarVR_native.txt"), "old file kept as a backup");
  check(settings_file::read(native, text, sizeof(text), &got) && std::string(text).find("autostart") != std::string::npos,
        "native survives migration");
  check(settings_file::read(arms, text, sizeof(text)) && std::string(text).find("clean_arm_rig 1") != std::string::npos &&
            std::string(text).find("[not_a_section]") != std::string::npos, "arms survive migration whole");

  // Writing one section leaves its neighbours alone.
  check(settings_file::write(hud, "wrist_hud 1\nband 0.100\n"), "new section written");
  check(settings_file::write(native, "observe"), "section replaced");
  check(settings_file::read(native, text, sizeof(text)) && std::strncmp(text, "observe", 7) == 0, "replaced value read back");
  check(std::string(text).find("autostart") == std::string::npos, "old value is gone");
  check(settings_file::read(arms, text, sizeof(text)) && std::string(text).find("hand_pitch -65.0000") != std::string::npos,
        "neighbour unchanged");
  check(settings_file::write(hud, ""), "empty value");
  check(settings_file::read(hud, text, sizeof(text), &got) && got == 0, "empty section still exists, like an empty file");
  check(settings_file::read(arms, text, sizeof(text)) && std::string(text).find("clean_arm_rig 1") != std::string::npos,
        "section after an emptied one unchanged");

  // A long value is cut to the caller's buffer and stays terminated, as ReadFile with size-1 did.
  char small[8];
  check(settings_file::read(arms, small, sizeof(small), &got) && got == 7 && small[7] == '\0', "short buffer");

  // A file dropped in by hand during a session is ignored until the next load imports it.
  put(native, "second 3");
  check(settings_file::read(native, text, sizeof(text)) && std::strncmp(text, "observe", 7) == 0, "shared file wins while a section exists");
  settings_file::migrate(dir);
  check(settings_file::read(native, text, sizeof(text)) && std::strncmp(text, "second 3", 8) == 0, "next load imports the dropped-in file");

  std::printf(failures ? "settings_file_test: %d FAILED\n" : "settings_file_test: all passed\n", failures);
  return failures ? 1 : 0;
}
