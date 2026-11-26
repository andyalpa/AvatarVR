"""Read a tagged AvatarVR session or a historical per-module log."""

from pathlib import Path


def channel_text(text, channel):
    prefix = f"[{channel}] "
    return "".join(line[len(prefix):] for line in text.splitlines(keepends=True) if line.startswith(prefix))


def read_channel(path, channel=None):
    path = Path(path)
    channel = channel or path.stem
    combined = path.with_name("AvatarVR.log")
    if combined.is_file() and path.suffix == ".log":
        text = channel_text(combined.read_text(encoding="utf-8", errors="replace"), channel)
        if text or path == combined or not path.is_file():
            return text
    return path.read_text(encoding="utf-8", errors="replace")
