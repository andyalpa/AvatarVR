"""Read and update one AvatarVR INI section without replacing other sections."""

from contextlib import contextmanager
import ctypes
from ctypes import wintypes
import os
from pathlib import Path
import re
import tempfile
import time

SECTIONS = {
    "AvatarVR_ik_tune.txt": "ik_tune",
    "AvatarVR_hud_widgets.txt": "hud_widgets",
    "AvatarVR_fullview_fx.txt": "fullview_fx",
    "AvatarVR_tuner_prefs.json": "tuner_prefs",
    "AvatarVR_hud_groups.json": "hud_groups",
}
HEADER = re.compile(
    r"^\[(" + "|".join((*SECTIONS.values(), "native", "xr", "shape", "monitor", "arms", "hud",
                       "moveprobe", "mounts", "pad", "dlss", "scanstereo", "skinik", "iktrace",
                       "lua_root", "lua_tunables")) + r")\]\r?$",
    re.MULTILINE,
)


def block(text, section):
    current = None
    for match in HEADER.finditer(text):
        if current is not None:
            return current.start(), current.end() + 1, match.start()
        if match[1] == section:
            current = match
    if current is not None:
        return current.start(), min(current.end() + 1, len(text)), len(text)
    return None


def read(path):
    legacy = Path(path)
    section = SECTIONS.get(legacy.name)
    config = legacy.with_name("AvatarVR_settings.ini")
    if section is not None and config.is_file():
        text = config.read_bytes().decode("utf-8")
        found = block(text, section)
        if found:
            return text[found[1]:found[2]]
    return legacy.read_text(encoding="utf-8")


@contextmanager
def write_lock():
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.CreateMutexW.argtypes = (wintypes.LPVOID, wintypes.BOOL, wintypes.LPCWSTR)
    kernel.CreateMutexW.restype = wintypes.HANDLE
    kernel.WaitForSingleObject.argtypes = (wintypes.HANDLE, wintypes.DWORD)
    kernel.WaitForSingleObject.restype = wintypes.DWORD
    kernel.ReleaseMutex.argtypes = (wintypes.HANDLE,)
    kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
    handle = kernel.CreateMutexW(None, False, r"Local\AvatarVR.Settings")
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        if kernel.WaitForSingleObject(handle, 2000) not in (0, 0x80):
            raise OSError("AvatarVR settings are busy")
        try:
            yield
        finally:
            kernel.ReleaseMutex(handle)
    finally:
        kernel.CloseHandle(handle)


def write(path, value):
    legacy = Path(path)
    section = SECTIONS[legacy.name]
    config = legacy.with_name("AvatarVR_settings.ini")
    with write_lock():
        try:
            text = config.read_bytes().decode("utf-8")
        except FileNotFoundError:
            text = "; AvatarVR settings.\r\n"
        replacement = f"[{section}]\r\n{value}"
        if not replacement.endswith("\n"):
            replacement += "\r\n"
        found = block(text, section)
        if found:
            updated = text[:found[0]] + replacement + text[found[2]:]
        else:
            updated = text + ("" if text.endswith("\n") else "\r\n") + replacement
        if updated == text:
            return
        fd, name = tempfile.mkstemp(prefix="AvatarVR_settings.", suffix=".tmp", dir=config.parent)
        try:
            with os.fdopen(fd, "wb") as stream:
                stream.write(updated.encode("utf-8"))
            for attempt in range(5):
                try:
                    os.replace(name, config)
                    break
                except PermissionError:
                    if attempt == 4:
                        raise
                    time.sleep(0.01)
        finally:
            Path(name).unlink(missing_ok=True)
