"""Read-only: where the game's direct queue sits idle, grouped by what each submission waited on.

    python tools\\perf\\idle_by_wait.py <run dir or DX12_OBSERVE.log>

Reads the `gputimeline D ...` rows the GPU profiler logs (one sampled frame every 10 s while AvatarVR_gpuprof.txt
says "on N"). Fence labels: c = the game's compute queue, x = a copy queue, ? = a fence no profiled queue signals
(e.g. DXGI / the headset runtime), d = the direct queue. Idle is the GPU gap before a submission.
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

ROW = re.compile(r"gputimeline D s(\d+)\s+(-?[\d.]+)-\s*(-?[\d.]+) \(([\d.]+) ms, idle before ([\d.]+)\)(.*)")
FRAME = re.compile(r"gputimeline frame (\d+)")
WAIT = re.compile(r"wait\[([^\]]*)\]")


def main():
    p = Path(sys.argv[1])
    if p.is_dir():
        p = p / "DX12_OBSERVE.log"
    frames = 0
    idle = defaultdict(float)
    count = defaultdict(int)
    busy = 0.0
    for line in p.read_text(errors="replace").splitlines():
        if FRAME.search(line):
            frames += 1
            continue
        m = ROW.search(line)
        if not m:
            continue
        busy += float(m.group(4))
        w = WAIT.search(m.group(6))
        key = "none"
        if w:
            fences = sorted({f.split(":")[0] for f in w.group(1).split()})
            key = " ".join(fences) if fences else "none"
        idle[key] += float(m.group(5))
        count[key] += 1
    if not frames:
        print("no gputimeline frames in", p)
        return
    print(f"{frames} sampled frames; direct busy {busy / frames:.2f} ms/frame; idle {sum(idle.values()) / frames:.2f} ms/frame")
    print(f"{'idle ms/frame':>13s} {'subs/frame':>10s}  waited on")
    for key in sorted(idle, key=lambda k: -idle[k]):
        print(f"{idle[key] / frames:13.2f} {count[key] / frames:10.1f}  {key}")


if __name__ == "__main__":
    main()
