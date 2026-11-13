import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location('scanner_plane_report', Path(__file__).parents[1] / 're/scanner_plane_report.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class ScannerPlacement(unittest.TestCase):
    def setUp(self):
        self.canvas = [0.] * 92
        self.canvas[76:80] = [2., -2., -1., 1.]
        self.matrix = struct.pack('<16f', 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)

    def test_clip_w_and_canvas_translation(self):
        matrix = struct.pack('<16f', 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 2)
        result = replay.placement((0., 0., 0.), 0., matrix, self.canvas)
        self.assertEqual(result['clip'], [0., -0., 0., 2.])
        self.canvas[82] = .25
        result = replay.placement((0., 0., 0.), 0., matrix, self.canvas)
        self.assertEqual(result['ndc'][0], .5)

    def test_byte_normalized_transform_selector(self):
        raw = self.matrix * 256
        for index in range(256):
            selector = replay.decode(bytes([index, 0, 0, 0]), 0, 28)[0]
            result = replay.placement((.5, .5, 0), selector, raw, self.canvas)
            self.assertEqual(result['transform_index'], index)

    def test_missing_transform_fails_closed(self):
        with self.assertRaises(ValueError):
            replay.placement((0., 0., 0.), 1., self.matrix, self.canvas)


if __name__ == '__main__':
    unittest.main()
