import unittest
from pathlib import Path
import tempfile
from analyze_logs import analyze, parse_dx12, parse_cpu_probe, parse_timelines, summarize


class TimingTests(unittest.TestCase):
    def test_wait_heavy_window_is_not_labelled_cpu_bound(self):
        windows, _, _ = parse_dx12("perf fps=30.0 interval avg=33.3 max=120.0 ms | xr 20.00/90.0 game 13.00/30.0 | gpu lag 2.0/5.0")
        self.assertEqual(windows[0]["xr_max_ms"], 90)
        self.assertEqual(windows[0]["fence_latency_ms"], 2)
        result = summarize(windows, 72)
        self.assertEqual(result["windows_containing_interval_over_40ms"], 1)
        self.assertNotIn("bottleneck", result)

    def test_latest_build_only(self):
        line = "perf fps=72.0 interval avg=13.9 max=20.0 ms | game 5.0/8.0"
        windows, _, _ = parse_dx12("dx12 observer build old\n" + line + "\ndx12 observer build new\n" + line)
        self.assertEqual(len(windows), 1)
        self.assertNotIn("fence_latency_ms", windows[0])

    def test_queue_execution_is_separate(self):
        _, queues, _ = parse_dx12("gpuprof direct: 100 frames (skipped 2, dropped subs 3) | busy 12.50 span 16.00 ms/frame | other 0.1")
        self.assertEqual(queues[0]["busy_ms"], 12.5)
        self.assertEqual(queues[0]["dropped_submissions"], 3)

    def test_transport_sample_weighting(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / "XR_RUNTIME.log").write_text("transport GPU: eye pair copies 1.000 ms, mirror 0.100 ms (1 completed samples)\ntransport GPU: eye pair copies 3.000 ms, mirror 0.300 ms (3 completed samples)")
            result = analyze(folder)
            self.assertEqual(result["transport"]["sample_weighted_eye_copies_ms"], 2.5)
            self.assertIsNone(result["all_windows"]["worst_logged_interval_ms"])

    def test_bad_parameters(self):
        with self.assertRaises(ValueError):
            analyze(Path("."), hz=0)

    def test_gaps_belong_to_their_own_queue(self):
        _, queues, _ = parse_dx12("gpuprof direct: 100 frames (skipped 0, dropped subs 0) | busy 10.00 span 14.00 ms/frame |\n"
                                 "gpuprof direct gaps per frame: starved 0.50 ms (1.0), after queue Wait 3.00 ms (2.0), other 0.50 ms (1.0) | queue latency avg 8.00 ms (GPU begin - CPU submit)")
        self.assertEqual(queues[0]["after_queue_wait_ms"], 3)
        self.assertEqual(queues[0]["busy_ms"], 10)

    def test_cpu_probe_parts_are_separate_from_gpu(self):
        rows = parse_cpu_probe("transport CPU: thread 12 300 presents | total 1.500/8.000 mutex 0.100/1.000 allocator 0.000/0.000 acquire 0.050/0.500 imagewait 1.200/6.000 submit 0.100/0.900 ms (avg/max; CPU wall time, nested parts; sampling)")
        self.assertEqual(rows[0]["imagewait_avg_ms"], 1.2)
        self.assertEqual(rows[0]["presents"], 300)

    def test_legacy_timeline_keeps_gaps_separate(self):
        rows = parse_timelines("gputimeline frame 64: 3 submissions (D direct, C compute);\n"
                               "gputimeline D s00   7.000-  8.000 (1.000 ms, idle before 0.000) cpu 0.0 UI eye1\n"
                               "gputimeline C s00   7.500-  9.000 (1.500 ms, idle before 0.000) cpu 0.0\n"
                               "gputimeline D s01   9.500- 10.000 (0.500 ms, idle before 1.500) cpu 0.0 wait[c7:3] wait[x8:2] UI eye2")
        self.assertTrue(rows[0]["complete"])
        self.assertFalse(rows[0]["common_origin"])
        self.assertEqual(rows[0]["direct_gap_ms_by_wait_labels"]["compute_and_copy"], 1.5)
        self.assertEqual(rows[0]["direct_span_ms"], 3)
        self.assertEqual(rows[0]["eye_ui_markers"], [1, 2])

    def test_incomplete_timeline_is_flagged(self):
        rows = parse_timelines("gputimeline frame 64: 20 submissions; common Present QPC origin;\n"
                               "gputimeline D s00 0.000-1.000 (1.000 ms, idle before 0.000) cpu 0.0")
        self.assertFalse(rows[0]["complete"])
        self.assertTrue(rows[0]["common_origin"])

if __name__ == "__main__":
    unittest.main()
