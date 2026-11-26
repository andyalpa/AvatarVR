"""Compare the GPU submission profiler ("gpuprof" lines in DX12_OBSERVE.log) between two runs.

Usage:  python tools/perf/gpuprof_compare.py <slow run dir or DX12_OBSERVE.log> <fast run dir or DX12_OBSERVE.log>

The profiler is switched on by scripts/AvatarVR_gpuprof.txt ("on 120" = 120 s from gameplay stereo start; see dx12.cpp
"GPU submission profiler"). Only blocks reported while stereo was running are used. For each queue (direct, compute)
it prints, per frame: GPU busy time, span, the per-view split (main / second / none / other / mixed), the idle gaps
(starved = GPU waited for the game's CPU; wait = cross-queue fence; other), and the submission positions whose GPU time
differs most. Added 2026-11-09 for the main-menu-panel performance question (dev notes "Panel vs follow-head").
"""
from session_log import read_channel
import os
import re
import sys

HEAD = re.compile(r"gpuprof stereo (running|off)")
MAIN = re.compile(r"gpuprof (direct|compute): (\d+) frames .*?\| busy ([\d.]+) span ([\d.]+) ms/frame \|(.*)$")
TAG = re.compile(r"(none|main|second|other|mixed) ([\d.]+) \(([\d.]+) subs\)")
GAPS = re.compile(r"gpuprof (direct|compute) gaps per frame: starved ([\d.]+) ms \(([\d.]+)\), after queue Wait ([\d.]+) ms "
                  r"\(([\d.]+)\), other ([\d.]+) ms \(([\d.]+)\) \| queue latency avg ([\d.]+)")
SUB = re.compile(r"gpuprof (direct|compute)\s+sub\s+(\d+): gpu ([\d.]+) ms, cpu at ([\d.]+) ms after Present, ([\d.]+) lists, "
                 r"tag (\w+)")


def log_path(arg):
    return os.path.join(arg, "DX12_OBSERVE.log") if os.path.isdir(arg) else arg


def parse(path):
    """Frame-weighted averages over the blocks reported while stereo was running."""
    q = {}
    running = False
    for line in read_channel(path, "DX12_OBSERVE").splitlines():
        line = line.rstrip("\n")
        m = HEAD.search(line)
        if m:
            running = m.group(1) == "running"
            continue
        if not running:
            continue
        m = MAIN.search(line)
        if m:
            name, frames = m.group(1), int(m.group(2))
            s = q.setdefault(name, {"frames": 0, "busy": 0.0, "span": 0.0, "tags": {}, "gaps": [0.0] * 4,
                                    "gap_frames": 0, "subs": {}, "last_frames": 0})
            if frames == 0:
                continue
            s["frames"] += frames
            s["last_frames"] = frames
            s["busy"] += float(m.group(3)) * frames
            s["span"] += float(m.group(4)) * frames
            for t, ms, _n in TAG.findall(m.group(5)):
                s["tags"][t] = s["tags"].get(t, 0.0) + float(ms) * frames
            continue
        m = GAPS.search(line)
        if m and m.group(1) in q:
            s = q[m.group(1)]
            f = s["last_frames"]
            for i, v in enumerate((m.group(2), m.group(4), m.group(6), m.group(8))):
                s["gaps"][i] += float(v) * f
            s["gap_frames"] += f
            continue
        m = SUB.search(line)
        if m and m.group(1) in q:
            s = q[m.group(1)]
            f = s["last_frames"]
            pos = int(m.group(2))
            acc = s["subs"].setdefault(pos, [0.0, 0, m.group(6), 0.0])
            acc[0] += float(m.group(3)) * f
            acc[1] += f
            acc[2] = m.group(6)
            acc[3] += float(m.group(4)) * f
    out = {}
    for name, s in q.items():
        n = max(s["frames"], 1)
        g = max(s["gap_frames"], 1)
        out[name] = {
            "frames": s["frames"],
            "busy": s["busy"] / n,
            "span": s["span"] / n,
            "tags": {t: v / n for t, v in s["tags"].items()},
            "gaps": [v / g for v in s["gaps"]],
            "subs": {p: (a[0] / max(a[1], 1), a[2], a[3] / max(a[1], 1)) for p, a in s["subs"].items()},
        }
    return out


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    a, b = parse(log_path(sys.argv[1])), parse(log_path(sys.argv[2]))
    print("A = %s\nB = %s\n(values are ms per frame, averaged over frames profiled while stereo was running)\n"
          % (sys.argv[1], sys.argv[2]))
    for name in ("direct", "compute"):
        if name not in a and name not in b:
            continue
        x, y = a.get(name), b.get(name)
        if not x or not y:
            print("%s: only in %s" % (name, "A" if x else "B"))
            continue
        print("== %s queue: A %d frames, B %d frames" % (name, x["frames"], y["frames"]))
        print("  %-28s %9s %9s %9s" % ("", "A", "B", "A-B"))
        print("  %-28s %9.2f %9.2f %+9.2f" % ("GPU busy", x["busy"], y["busy"], x["busy"] - y["busy"]))
        print("  %-28s %9.2f %9.2f %+9.2f" % ("span (first to last sub)", x["span"], y["span"], x["span"] - y["span"]))
        for t in ("main", "second", "none", "other", "mixed"):
            xa, yb = x["tags"].get(t, 0.0), y["tags"].get(t, 0.0)
            if xa or yb:
                print("  %-28s %9.2f %9.2f %+9.2f" % ("  busy during " + t, xa, yb, xa - yb))
        for i, label in enumerate(("idle: starved (waits for CPU)", "idle: after queue Wait", "idle: other", "queue latency avg")):
            print("  %-28s %9.2f %9.2f %+9.2f" % (label, x["gaps"][i], y["gaps"][i], x["gaps"][i] - y["gaps"][i]))
        diffs = []
        for p in set(x["subs"]) | set(y["subs"]):
            ga = x["subs"].get(p, (0.0, "-", 0.0))
            gb = y["subs"].get(p, (0.0, "-", 0.0))
            diffs.append((ga[0] - gb[0], p, ga, gb))
        diffs.sort(key=lambda d: -abs(d[0]))
        print("  submissions with the largest GPU-time difference (position: A ms [tag, cpu ms after Present] | B):")
        for d, p, ga, gb in diffs[:12]:
            print("    sub %3d: %+7.3f  A %.3f [%s, %.1f] | B %.3f [%s, %.1f]" % (p, d, ga[0], ga[1], ga[2], gb[0], gb[1], gb[2]))
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
