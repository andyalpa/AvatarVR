"""Read-only AvatarVR timing summary. No game launch, settings writes or deployment.

Percentiles describe logged reporting-window averages, NOT individual frames.
GPU fence latency and Present-to-Present residuals are not execution timings.
"""
from session_log import read_channel
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

NUMBER = r"(\d+(?:\.\d+)?)"
PERF = re.compile(r"perf fps=" + NUMBER + r" interval avg=" + NUMBER + r" max=" + NUMBER + r" ms \|")
STAGE = re.compile(r"\b(upload|eyeprobe|capture|xr|menu|native|eye|time|present|game) " + NUMBER + r"/" + NUMBER)
LAG = re.compile(r"gpu lag " + NUMBER + r"/" + NUMBER)
TRANSPORT = re.compile(r"transport GPU: eye pair copies " + NUMBER + r" ms, mirror " + NUMBER + r" ms \((\d+) completed samples")
QUEUE = re.compile(r"gpuprof (\w+): (\d+) frames \(skipped (\d+), dropped subs (\d+)\) \| busy " + NUMBER + r" span " + NUMBER)
GAPS = re.compile(r"gpuprof (\w+) gaps per frame: starved " + NUMBER + r" ms \([^)]*\), after queue Wait " + NUMBER + r" ms \([^)]*\), other " + NUMBER + r" ms \([^)]*\) \| queue latency avg " + NUMBER)
MEMORY = re.compile(r"video memory: process local " + NUMBER + r" MiB / budget " + NUMBER)
CPU_PROBE = re.compile(r"transport CPU: thread (\d+) (\d+) presents \| (.*?) ms \(avg/max;")
TIMELINE_FRAME = re.compile(r"gputimeline frame (\d+): (\d+) submissions")
SIGNED_NUMBER = r"(-?\d+(?:\.\d+)?)"
TIMELINE_ROW = re.compile(r"gputimeline ([DC]) s(\d+)\s+" + SIGNED_NUMBER + r"-\s*" + SIGNED_NUMBER + r" \(" + NUMBER + r" ms, idle before " + NUMBER + r"\) (.*)")


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {"count": len(values), "median": statistics.median(values),
            "p95": values[max(0, math.ceil(len(values) * .95) - 1)], "max": values[-1]}


def parse_dx12(text):
    windows, queues, memory = [], [], []
    session = 0
    for line_number, line in enumerate(text.splitlines(), 1):
        if "dx12 observer build " in line:
            session += 1
        match = PERF.search(line)
        if match:
            fps, interval, worst = map(float, match.groups())
            row = {"line": line_number, "session": session, "fps": fps, "interval_ms": interval, "max_interval_ms": worst}
            for stage, average, maximum in STAGE.findall(line):
                row[stage + "_ms"] = float(average)
                row[stage + "_max_ms"] = float(maximum)
            lag = LAG.search(line)
            if lag:
                row["fence_latency_ms"], row["fence_latency_max_ms"] = map(float, lag.groups())
            windows.append(row)
        match = QUEUE.search(line)
        if match:
            name, frames, skipped, dropped, busy, span = match.groups()
            queues.append({"line": line_number, "session": session, "queue": name, "frames": int(frames),
                           "skipped": int(skipped), "dropped_submissions": int(dropped),
                           "busy_ms": float(busy), "span_ms": float(span)})
        match = MEMORY.search(line)
        if match:
            used, budget = map(float, match.groups())
            memory.append({"line": line_number, "session": session, "local_mib": used, "budget_mib": budget,
                           "usage_fraction": used / budget if budget else None})
        match = GAPS.search(line)
        if match:
            name, starved, wait, other, latency = match.groups()
            for report in reversed(queues):
                if report["session"] == session and report["queue"] == name:
                    report.update(starved_ms=float(starved), after_queue_wait_ms=float(wait),
                                  other_gap_ms=float(other), submission_latency_ms=float(latency))
                    break
    # Rotated/concatenated logs must not silently mix builds.
    return tuple([r for r in rows if r["session"] == session] for rows in (windows, queues, memory))


def parse_transport(text):
    # XR_RUNTIME is normally per launch; reset on its explicit launch marker.
    rows = []
    for line_number, line in enumerate(text.splitlines(), 1):
        if "xr runtime build " in line.lower():
            rows = []
        match = TRANSPORT.search(line)
        if match:
            copies, mirror, count = match.groups()
            rows.append({"line": line_number, "eye_copies_ms": float(copies), "mirror_ms": float(mirror), "samples": int(count)})
    return rows


def parse_cpu_probe(text):
    reports = []
    for line in text.splitlines():
        if "xr runtime build " in line.lower():
            reports = []
        match = CPU_PROBE.search(line)
        if match:
            thread, presents, parts = match.groups()
            row = {"thread": int(thread), "presents": int(presents)}
            for name, average, maximum in re.findall(r"(total|mutex|allocator|acquire|imagewait|submit) " + NUMBER + "/" + NUMBER, parts):
                row[name + "_avg_ms"] = float(average)
                row[name + "_max_ms"] = float(maximum)
            reports.append(row)
    return reports


def parse_timelines(text):
    """Single-queue gaps remain meaningful for old logs with different origins.

    Wait labels are correlations, not causal attribution. Never add gaps across
    queues or treat a direct gap as globally idle/recoverable GPU time.
    """
    frames = []
    current = None
    for line in text.splitlines():
        if "dx12 observer build " in line:
            frames, current = [], None
        match = TIMELINE_FRAME.search(line)
        if match:
            current = {"frame": int(match[1]), "expected_submissions": int(match[2]),
                       "common_origin": "common Present QPC origin" in line, "rows": []}
            frames.append(current)
            continue
        match = TIMELINE_ROW.search(line)
        if match and current is not None:
            queue, sub, begin, end, duration, gap, detail = match.groups()
            current["rows"].append({"queue": queue, "sub": int(sub), "begin_ms": float(begin), "end_ms": float(end),
                                    "duration_ms": float(duration), "gap_ms": float(gap), "detail": detail})
    for frame in frames:
        frame["complete"] = len(frame["rows"]) == frame["expected_submissions"]
        frame["eye_ui_markers"] = sorted({int(eye) for row in frame["rows"] for eye in re.findall(r"UI eye([12])", row["detail"])})
        # Zero/short rows are included: tiny submissions can carry important waits.
        direct = sorted((r for r in frame["rows"] if r["queue"] == "D"), key=lambda r: r["sub"])
        gaps = {"compute_only": 0.0, "copy_only": 0.0, "compute_and_copy": 0.0, "other_or_no_wait": 0.0}
        for row in direct[1:]:
            compute = bool(re.search(r"wait\[[ck]\d+:", row["detail"]))
            copy = bool(re.search(r"wait\[x\d+:", row["detail"]))
            kind = "compute_and_copy" if compute and copy else "compute_only" if compute else "copy_only" if copy else "other_or_no_wait"
            gaps[kind] += row["gap_ms"]
        frame["direct_gap_ms_by_wait_labels"] = gaps
        frame["direct_span_ms"] = max((r["end_ms"] for r in direct), default=0) - min((r["begin_ms"] for r in direct), default=0)
        frame["direct_wrapped_submission_ms"] = sum(r["duration_ms"] for r in direct)
        # Keep the bounded frame aggregates, not hundreds of raw event rows.
        del frame["rows"]
    return frames


def summarize(windows, hz):
    budget = 1000 / hz
    fields = sorted({k for row in windows for k in row if k.endswith("_ms")})
    return {"windows": len(windows), "budget_ms": budget,
            "window_average_distributions_ms": {key: distribution([r[key] for r in windows if key in r]) for key in fields if "_max_" not in key and not key.startswith("max_")},
            "windows_average_over_budget": sum(r["interval_ms"] > budget for r in windows),
            "windows_containing_interval_over_40ms": sum(r["max_interval_ms"] > 40 for r in windows),
            "worst_logged_interval_ms": max((r["max_interval_ms"] for r in windows), default=None),
            "worst_residual_game_interval_ms": max((r.get("game_max_ms", 0) for r in windows), default=None)}


def analyze(folder, hz=72, tail=120):
    if hz <= 0 or tail <= 0:
        raise ValueError("hz and tail must be positive")
    files = {}
    texts = {}
    combined = folder / "AvatarVR.log"
    if combined.is_file():
        raw = combined.read_bytes()
        files[combined.name] = {"path": str(combined.resolve()), "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest(),
                                "mtime_unix": combined.stat().st_mtime}
    for name in ("DX12_OBSERVE.log", "XR_RUNTIME.log", "DLSS_TWIN.log", "NATIVE_STEREO.log", "AvatarVR_settings.ini", "AvatarVR.dll"):
        path = folder / name
        if name.endswith(".log"):
            try:
                texts[name] = read_channel(path)
            except FileNotFoundError:
                pass
        if path.is_file():
            raw = path.read_bytes()
            files[name] = {"path": str(path.resolve()), "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest(),
                           "mtime_unix": path.stat().st_mtime}
            if not name.endswith(".log") and path.suffix != ".dll":
                texts[name] = raw.decode("utf-8", errors="replace")
    windows, queues, memory = parse_dx12(texts.get("DX12_OBSERVE.log", ""))
    transport = parse_transport(texts.get("XR_RUNTIME.log", ""))
    total_samples = sum(r["samples"] for r in transport)
    xr = texts.get("XR_RUNTIME.log", "")
    dlss = texts.get("DLSS_TWIN.log", "")
    return {"sources": files, "target_hz": hz, "scope": "Latest DX12 session; tail is a reporting-window selection, not a verified gameplay filter.",
            "limitations": ["GPU fence latency includes backlog, queue waits and observer scheduling; it is not GPU execution time.",
                            "game is the residual between Presents, including engine work/waits; it is not measured CPU execution.",
                            "p95 values are percentiles of reporting-window averages, not frame-time p95.",
                            "Transport and DX12 logs are summarized separately; no cross-log frame/time alignment is claimed.",
                            "On-disk DLL/settings hashes identify the current files, not proof of the historical loaded DLL/settings."],
            "all_windows": summarize(windows, hz), "tail_windows": summarize(windows[-tail:], hz),
            "tail_selection": {"requested": tail, "first_dx12_line": windows[-tail:][0]["line"] if windows else None,
                               "last_dx12_line": windows[-1]["line"] if windows else None},
            "transport": {"report_windows": len(transport), "completed_samples": total_samples,
                          "sample_weighted_eye_copies_ms": sum(r["eye_copies_ms"] * r["samples"] for r in transport) / total_samples if total_samples else None,
                          "sample_weighted_mirror_ms": sum(r["mirror_ms"] * r["samples"] for r in transport) / total_samples if total_samples else None},
            "queue_reports": queues, "memory": {"reports": len(memory), "peak_local_mib": max((r["local_mib"] for r in memory), default=None),
                                                  "peak_budget_fraction": max((r["usage_fraction"] for r in memory if r["usage_fraction"] is not None), default=None)},
            "transport_cpu_probe_reports": parse_cpu_probe(xr),
            "timeline_frames": parse_timelines(texts.get("DX12_OBSERVE.log", "")),
            "runtime_identity_lines": [line for line in xr.splitlines() if "runtime: " in line or ": recommended " in line][:6],
            "visibility_mesh_lines": [line for line in xr.splitlines() if "vismask: view " in line],
            "last_dlss_extent_lines": [line for line in dlss.splitlines() if "options mode=" in line][-5:]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scripts", type=Path)
    parser.add_argument("--hz", type=float, default=72)
    parser.add_argument("--tail", type=int, default=120)
    parser.add_argument("--output", type=Path, help="Optional JSON output; never edits the source logs/settings")
    args = parser.parse_args()
    if args.hz <= 0 or args.tail <= 0:
        parser.error("--hz and --tail must be positive")
    if not any((args.scripts / name).is_file() for name in ("AvatarVR.log", "DX12_OBSERVE.log")):
        parser.error("scripts folder must contain AvatarVR.log or DX12_OBSERVE.log")
    result = analyze(args.scripts, args.hz, args.tail)
    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        # Refuse to overwrite the inputs even if a caller supplies that path.
        if args.output.resolve() in {Path(item["path"]) for item in result["sources"].values()}:
            parser.error("output must not overwrite a source file")
        args.output.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")


if __name__ == "__main__":
    main()
