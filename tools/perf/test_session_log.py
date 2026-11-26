import tempfile
from pathlib import Path
import unittest

from session_log import read_channel


class SessionLogTests(unittest.TestCase):
    def test_extracts_a_channel_and_reads_legacy_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            legacy = root / "XR_RUNTIME.log"
            legacy.write_text("legacy example\n", encoding="utf-8")
            self.assertEqual(read_channel(legacy), "legacy example\n")
            (root / "AvatarVR.log").write_text(
                "[LOAD_PROOF] startup\n[XR_RUNTIME] first\n"
                "[DX12_OBSERVE] present\n[XR_RUNTIME] second\n", encoding="utf-8"
            )
            self.assertEqual(read_channel(legacy), "first\nsecond\n")
            self.assertEqual(read_channel(root / "AvatarVR.log", "DX12_OBSERVE"), "present\n")
