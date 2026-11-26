"""Read-only stage-timing summary of one run (AvatarVR_gpuprof.txt = "stages N").

    python tools\\perf\\stages_report.py <run dir or DX12_OBSERVE.log> [--split LINE] [--top N]

Averages the `gpustages <queue> <stage>` lines over the stereo-running windows (direct busy > 6 ms). With --split,
windows before that log line and after it are averaged separately and compared (e.g. eye scale 80 % vs 100 %):
the delta column shows which stages grow with resolution. "runs/frame ~2" usually means the stage runs once per eye.
Stage time is GPU time between a run's timestamps: it includes overlap with earlier unfinished work (attribution).
"""
from session_log import read_channel
import argparse
import re
from collections import defaultdict
from pathlib import Path

BLOCK = re.compile(r"gpuprof direct: \d+ frames .*\| busy ([\d.]+) span ([\d.]+)")
STAGE = re.compile(r"gpustages (\w+) (.+?): ([\d.]+) ms/frame \(main view ([\d.]+) \| second view ([\d.]+) \| after ([\d.]+)\), "
                   r"([\d.]+) runs/frame")


def windows(lines):
    """Yields (line number, busy, span, {(queue, stage): (ms, main, second, after, runs)}) per 5 s window."""
    current = None
    for number, line in enumerate(lines, 1):
        m = BLOCK.search(line)
        if m:
            if current:
                yield current
            current = [number, float(m.group(1)), float(m.group(2)), {}]
            continue
        m = STAGE.search(line)
        if m and current:
            q, name = m.group(1), m.group(2)
            current[3][(q, name)] = tuple(float(x) for x in m.groups()[2:])
    if current:
        yield current


def average(ws):
    total = defaultdict(lambda: [0.0] * 5)
    for _, _, _, stages in ws:
        for key, values in stages.items():
            for i, v in enumerate(values):
                total[key][i] += v
    n = len(ws) or 1
    return {k: [v / n for v in vals] for k, vals in total.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--split", type=int, default=0)
    ap.add_argument("--top", type=int, default=40)
    ap.add_argument("--series", nargs="*", default=None,
                    help="print one line per 5 s window: busy, span and the ms of stages whose name contains each text")
    a = ap.parse_args()
    p = Path(a.path)
    if p.is_dir():
        p = p / "DX12_OBSERVE.log"
    ws = [w for w in windows(read_channel(p, "DX12_OBSERVE").splitlines()) if w[1] > 6.0]
    if a.series is not None:
        # Settings sweep: a setting change shows as a step in its stages (and in busy) between two windows.
        print("window  line   busy   span  " + "  ".join(f"{s[:14]:>14s}" for s in a.series))
        for i, (line, busy, span, stages) in enumerate(ws):
            cols = [sum(v[0] for (q, n), v in stages.items() if s.lower() in n.lower()) for s in a.series]
            print(f"{i:6d} {line:5d} {busy:6.2f} {span:6.2f}  " + "  ".join(f"{c:14.2f}" for c in cols))
        return
    groups = [("all", ws)] if not a.split else [("before", [w for w in ws if w[0] < a.split]),
                                                  ("after", [w for w in ws if w[0] >= a.split])]
    avgs = []
    for name, g in groups:
        busy = sum(w[1] for w in g) / max(len(g), 1)
        span = sum(w[2] for w in g) / max(len(g), 1)
        print(f"{name}: {len(g)} windows, direct busy {busy:.2f} ms, span {span:.2f} ms")
        avgs.append(average(g))
    ref = avgs[-1]
    keys = sorted(ref, key=lambda k: -ref[k][0])[:a.top]
    print(f"\n{'queue':7s} {'ms':>6s} {'main':>6s} {'second':>6s} {'after':>6s} {'runs':>5s}" +
          ("  before  delta" if a.split else "") + "  stage")
    for k in keys:
        ms, main_, second, after, runs = ref[k]
        extra = ""
        if a.split:
            b = avgs[0].get(k, [0.0] * 5)[0]
            extra = f"  {b:6.2f} {ms - b:+6.2f}"
        print(f"{k[0]:7s} {ms:6.2f} {main_:6.2f} {second:6.2f} {after:6.2f} {runs:5.1f}{extra}  {k[1]}")


if __name__ == "__main__":
    main()
