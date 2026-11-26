"""Read-only table of archived runs: stereo frame rate next to what differed on the PC.

    python tools\\perf\\runs_report.py [runs_dir]        (default: the game's scripts\\runs)

Per run folder it reads NATIVE_STEREO.log (`frame:` lines with stereo=1), DX12_OBSERVE.log (build, `perf` lines),
GPU_MONITOR.log (utilisation, encoder sessions, other programs on the GPU) and, from 2026-11-11, RUN_FINGERPRINT.log
(game settings, power plan). Nothing is launched or written. Frame rates are 2 s averages logged by the mod, not
individual frames. A run needs at least 5 stereo samples to be listed.
"""
from session_log import read_channel
import re
import statistics as st
import sys
from pathlib import Path

FRAME = re.compile(r"^(\d\d:\d\d:\d\d) frame: fps=([\d.]+) stereo=(\d) foreground=(\S+)")
BUILD = re.compile(r"dx12 observer build (\w+ +\d+ \d+ [\d:]+)")
PERF = re.compile(r"perf fps=([\d.]+) interval avg=([\d.]+).*?\bgame ([\d.]+)/.*?gpu lag ([\d.]+)/")
VIEW0 = re.compile(r"cost view #0: \d+ renders \| Render ([\d.]+) ms/frame")
GPU = re.compile(r"gpu +(\d+)% \|.*\| enc (\d+)/(\d+) fps")
PROC = re.compile(r"processes on the GPU: (.*)")
OWNER = re.compile(r"encoder session owner: pid \d+ = (\S+) \((\S+) (\d+x\d+)\)")
KEYS = ("reduceLatency", "maxFramerate", "customFPSLimit", "nvReflex")


def read(path):
    try:
        return read_channel(path).splitlines()
    except OSError:
        return []


def med(values):
    return st.median(values) if values else float("nan")


def summarize(folder):
    frames = [(m.group(1), float(m.group(2)), m.group(4)) for m in map(FRAME.match, read(folder / "NATIVE_STEREO.log"))
              if m and m.group(3) == "1"]
    if len(frames) < 5:
        return None
    dx = read(folder / "DX12_OBSERVE.log")
    build = next((m.group(1) for m in map(BUILD.search, dx) if m), "?")
    perf = [tuple(map(float, m.groups())) for m in map(PERF.search, dx) if m]
    perf = [p for p in perf if p[0] >= 45]          # gameplay-rate windows only
    render0 = [float(m.group(1)) for m in map(VIEW0.search, read(folder / "NATIVE_STEREO.log")) if m]
    render0 = render0[len(render0) // 3:]            # skip start-up
    util, sessions, others, owners = [], [], {}, set()
    for line in read(folder / "GPU_MONITOR.log"):
        m = GPU.search(line)
        if m and int(m.group(1)) > 60:               # samples with the game loaded
            util.append(int(m.group(1)))
            sessions.append(int(m.group(2)))
        m = PROC.search(line)
        if m and "afop" in m.group(1):
            for part in m.group(1).split("; "):
                name = part.split("(pid")[0].strip()
                share = re.search(r"sm (\d+)", part)
                if share and not name.startswith(("afop", "VirtualDesktop")):
                    others[name] = max(others.get(name, 0), int(share.group(1)))
        m = OWNER.search(line)
        if m:
            owners.add(f"{m.group(1)} {m.group(3)}")
    cfg, plan = {}, ""
    for line in read(folder / "RUN_FINGERPRINT.log"):
        if line.startswith("cfg ") and "=" in line:
            key, value = line[4:].split("=", 1)
            if key in KEYS:
                cfg[key] = value
        elif line.startswith("power plan="):
            plan = line.split("=", 1)[1]
    fps = [f[1] for f in frames]
    foreground = sorted({f[2] for f in frames if f[2] != "GAME"})
    return {
        "name": folder.name, "build": build, "start": frames[0][0], "end": frames[-1][0], "n": len(fps),
        "fps": st.mean(fps), "median": med(fps), "locked": 100.0 * sum(1 for f in fps if f >= 70) / len(fps),
        "game_ms": med([p[2] for p in perf]), "lag_ms": med([p[3] for p in perf]), "render0": med(render0),
        "util": st.mean(util) if util else None, "sessions": st.mean(sessions) if sessions else None,
        "others": others, "owners": owners, "cfg": cfg, "plan": plan, "foreground": foreground,
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Usage: runs_report.py <runs-directory>")
    root = Path(sys.argv[1])
    folders = sorted(p for p in root.iterdir() if p.is_dir())
    folders.append(root.parent)                      # the live logs of the latest run
    seen, rows = set(), []
    for folder in folders:
        row = summarize(folder)
        if row and (row["build"], row["start"], row["n"]) not in seen:
            seen.add((row["build"], row["start"], row["n"]))
            rows.append(row)
    print("run folder                 start    n   fps  med >=70% | game ms  gpu lag  view0 Render | GPU%  enc | other GPU programs / notes")
    for r in rows:
        gpu = f"{r['util']:4.0f}  {r['sessions']:3.1f}" if r["util"] is not None else "   -    -"
        notes = [f"{k} {v}%" for k, v in sorted(r["others"].items())]
        notes += sorted(r["owners"])
        notes += [f"{k}={v}" for k, v in r["cfg"].items()]
        if r["plan"]:
            notes.append(r["plan"])
        if r["foreground"]:
            notes.append("foreground: " + ",".join(r["foreground"]))
        print(f"{r['name']:26s} {r['start']} {r['n']:3d} {r['fps']:5.1f} {r['median']:4.0f} {r['locked']:4.0f}% |"
              f" {r['game_ms']:7.2f} {r['lag_ms']:8.2f} {r['render0']:13.2f} | {gpu} | {'; '.join(notes)}")
    print("\ngame ms = Present-to-Present time outside the mod's Present work (median of gameplay-rate windows);"
          " gpu lag = fence latency, not GPU execution time; view0 Render = CPU time inside the main view's Render"
          " (includes waiting); enc = video encoder sessions while the GPU was loaded.")


if __name__ == "__main__":
    main()
