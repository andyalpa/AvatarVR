"""Pose editor model, persistence, runtime default parity and isolated Tk smoke tests."""
from pathlib import Path
import json
import math
import subprocess
import sys
import tempfile
import time
import unittest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools/ik_tuner"))
from gesture_tuning import GestureModel, defaults, prefix, COMBINATIONS, FINGERS, HELPERS, AXES, HELPER_AXES
import ik_tuner


class GestureTunerTests(unittest.TestCase):
    def test_hand_context_isolation_and_copy(self):
        model = GestureModel()
        pose = model.pose("fp", "left", 11);pose["index.bend1"] = 22
        model.store("fp", "left", 11, pose)
        self.assertEqual(model.pose("fp", "left", 11)["index.bend1"], 22)
        self.assertEqual(model.pose("tp", "left", 11)["index.bend1"], 65)
        self.assertEqual(model.pose("fp", "right", 11)["index.bend1"], 65)
        model.copy("fp", "left", "tp", "right", 11)
        self.assertEqual(model.pose("tp", "right", 11)["index.bend1"], 22)
        model.reset("fp", "left", 11)
        self.assertEqual(model.pose("tp", "right", 11)["index.bend1"], 22)

    def test_invalid_values_and_stale_preview_are_not_imported(self):
        model = GestureModel({"gesture.fp.left.fist.index.roll": math.nan, "gesture.preview_until": time.time()+1000})
        self.assertEqual(model.values, {})
        pose = defaults(11);pose["index.bend1"] = 999;pose["thumb.splay"] = -999
        model.store("fp", "left", 11, pose)
        self.assertEqual(model.pose("fp", "left", 11)["index.bend1"], 120)
        self.assertEqual(model.pose("fp", "left", 11)["thumb.splay"], -85)

    def test_bone_clipboard_model(self):
        from gesture_tuning import bone_values, paste_bone
        pose = defaults(11);pose["index.bend1"] = 50;pose["thumb_01_expand01.scale_x"] = 30
        clip = bone_values(pose, "index")
        self.assertEqual(set(clip), set(AXES))
        target = paste_bone(defaults(0), "pinky", dict(clip, bend1=999))
        self.assertEqual(target["pinky.bend1"], 120)
        self.assertEqual(target["index.bend1"], defaults(0)["index.bend1"])
        helper = paste_bone(defaults(0), HELPERS[5], bone_values(pose, HELPERS[0]))
        self.assertEqual(helper[HELPERS[5] + ".scale_x"], 30)

    def test_runtime_default_parity(self):
        binary = REPO / "build/hand_gestures/bin/hand_gestures_test.exe"
        output = subprocess.check_output([str(binary), "--defaults"], text=True)
        count = 0
        for line in output.splitlines():
            c, f, axis, value = line.split()
            expected = defaults(int(c))[(FINGERS + HELPERS)[int(f)] + "." + axis]
            self.assertAlmostEqual(float(value), expected, places=4)
            count += 1
        self.assertEqual(count, 13*(len(FINGERS)*len(AXES)+len(HELPERS)*len(HELPER_AXES)))

    def test_window_persistence_and_preview_in_isolated_folder(self):
        import tkinter as tk
        with tempfile.TemporaryDirectory() as directory:
            tune = Path(directory) / ik_tuner.TUNE_FILE
            original = "# existing\ntp.hand_yaw=15\ntp.left.forearm_roll=-9\nother_stream.value=37\n"
            tune.write_text(original, encoding="utf-8")
            root = tk.Tk();root.withdraw()
            try:
                app = ik_tuner.App(root, directory)
                root.update()
                self.assertEqual(tune.read_text(), original, "opening tuner must not write")
                row = next(r for r in app.gestures.rows if r.key == "index.bend1")
                row.set(25)
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.fp.left.fist.index.bend1"], 25)
                self.assertEqual(saved["tp.hand_yaw"], 15)
                self.assertEqual(saved["tp.left.forearm_roll"], -9)
                self.assertEqual(saved["other_stream.value"], 37)
                serialized = app.text()
                self.assertLess(serialized.index("tp.hand_yaw="), serialized.index("gesture."), "arm keys precede large pose banks for bounded legacy readers")
                self.assertEqual(saved["gesture.preview_until"], 0)
                self.assertTrue(any(r.key == "thumb.root_normal" for r in app.gestures.rows))
                self.assertTrue(any(r.key == "thumb.roll1" for r in app.gestures.rows))
                app.gestures.helper_rows[0].set(12)
                app.gestures.helper.set(HELPERS[1])
                app.gestures.select_helper()
                app.gestures.helper_rows[3].set(3)
                app.gestures.helper.set(HELPERS[0])
                app.gestures.select_helper()
                self.assertEqual(app.gestures.helper_rows[0].value(), 12, "hidden helper adjustments survive selection")
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.fp.left.fist.thumb_01_expand02.offset_x"], 3)
                app.gestures.preview.set(True)
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertGreater(saved["gesture.preview_until"], time.time())
                self.assertLessEqual(saved["gesture.preview_until"], time.time()+2.1)
                # Serialized presets never arm preview.
                self.assertIn("gesture.preview_until=0", app.text(preview=False))
                app.gestures.context.set("Mounted")
                app.gestures.select()
                self.assertEqual(next(r for r in app.gestures.rows if r.key=="index.bend1").value(), 65)
                status = {"at":time.time(),"mounted":1,"context_ready":1,"native_owned":0,
                          "owners":[1,5],"weight":[1,0],"combo":[11,12],"grip":[1,0],"trigger":[1,0],
                          "index_touch":[1,0],"thumb_touch":[1,0],"applications":1,"rejects":0}
                (Path(directory)/"AvatarVR_gesture_status.json").write_text(json.dumps(status))
                app.gestures.poll()
                self.assertIn("Mounted", app.gestures.live.get())
                self.assertIn("harness", app.gestures.live.get())
                status["tracked"] = [0, 1]
                (Path(directory)/"AvatarVR_gesture_status.json").write_text(json.dumps(status))
                app.gestures.poll()
                self.assertIn("Left controller NOT TRACKED", app.gestures.live.get())
                # Capture: request keys go out, matching answer lands in the selected combination.
                app.gestures.context.set("On foot");app.gestures.hand.set("Right")
                app.gestures.combo.set(ik_tuner.LABELS[12]);app.gestures.select()
                app.gestures.capture()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.capture_hand"], 1)
                self.assertEqual(saved["gesture.capture_id"], app.gestures.capture_id)
                self.assertGreater(saved["gesture.capture_until"], time.time())
                self.assertIn("gesture.capture_until=0", app.text(preview=False))
                answer = {"id": app.gestures.capture_id, "hand": "right", "context": "tp", "position_mm": 1.5,
                          "angle_deg": 2, "clamped": 0, "values": {"index.bend1": 33, "thumb.roll1": 7,
                                                                    "thumb_01_expand01.scale_x": 500, "bogus.key": 1}}
                (Path(directory)/"AvatarVR_gesture_capture.json").write_text(json.dumps(answer))
                app.gestures.poll()
                self.assertEqual(app.gestures.current, ("tp", "right", 12))
                self.assertEqual(app.gestures.context.get(), "Mounted")
                self.assertEqual(next(r for r in app.gestures.rows if r.key=="index.bend1").value(), 33)
                self.assertIn("Captured right hand", app.gestures.capture_text.get())
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.tp.right.idle.index.bend1"], 33)
                self.assertEqual(saved["gesture.tp.right.idle.thumb.roll1"], 7)
                self.assertEqual(saved["gesture.tp.right.idle.thumb_01_expand01.scale_x"], 75)
                self.assertEqual(saved["gesture.capture_hand"], -1)
                # Bone copy/paste across hand, rig and combination; finger<->finger and helper<->helper only.
                g = app.gestures
                g.context.set("On foot");g.hand.set("Left");g.combo.set(ik_tuner.LABELS[11]);g.select()
                rows = {r.key: r for r in g.rows}
                rows["index.bend2"].set(77);rows["index.roll3"].set(-12)
                g.copy_bone("index")
                g.helper_rows[0].set(9);g.copy_bone(HELPERS[0])
                self.assertIn("thumb_01_expand01 (left on foot, fist)", g.clipboard_text.get())
                g.copy_bone("index")
                g.context.set("Mounted");g.hand.set("Right");g.combo.set(ik_tuner.LABELS[7]);g.select()
                g.paste_bone("middle")
                g.paste_bone(HELPERS[2])
                self.assertIn("nothing to paste onto a helper", g.clipboard_text.get())
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.tp.right.point.middle.bend2"], 77)
                self.assertEqual(saved["gesture.tp.right.point.middle.roll3"], -12)
                self.assertNotIn("gesture.tp.right.point.index.bend2", saved, "source bone name not pasted")
                self.assertEqual(saved["gesture.fp.left.fist.index.bend2"], 77, "source pose unchanged")
                # Whole-pose copy/paste between combinations.
                g.context.set("On foot");g.hand.set("Left");g.combo.set(ik_tuner.LABELS[11]);g.select()
                g.copy_pose()
                g.combo.set(ik_tuner.LABELS[0]);g.select()
                g.paste_pose()
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.fp.left.open.index.bend2"], 77)
                self.assertEqual(saved["gesture.fp.left.open.thumb_01_expand01.pitch"], 9)
                self.assertEqual(saved["gesture.fp.left.open.middle.bend1"], 70, "fist defaults carried, not open defaults")
                self.assertNotIn("gesture.fp.right.open.index.bend2", saved, "mirror off: other hand untouched")
                # Mirror toggle: edits on one hand land on the other, and the choice is remembered.
                g.mirror.set(True);g.mirror_toggled()
                rows = {r.key: r for r in g.rows}
                rows["pinky.splay"].set(-20)
                app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.fp.left.open.pinky.splay"], -20)
                self.assertEqual(saved["gesture.fp.right.open.pinky.splay"], -20)
                self.assertEqual(saved["gesture.fp.right.open.index.bend2"], 77)
                self.assertNotIn("gesture.tp.left.open.pinky.splay", saved, "mirror never crosses rigs")
                g.hand.set("Right");g.select()
                rows["pinky.splay"].set(-30);app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.fp.left.open.pinky.splay"], -30, "mirror works from either hand")
                g.reset();app.write()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertFalse(any(k.startswith("gesture.fp.left.open.") for k in saved), "reset mirrors too")
                self.assertTrue(json.loads((Path(directory)/"AvatarVR_tuner_prefs.json").read_text())["mirror_hands"])
                self.assertTrue(ik_tuner.GestureEditor.prefs(g)["mirror_hands"])
                app.close()
                saved = ik_tuner.read_kv(tune, "=")
                self.assertEqual(saved["gesture.preview_until"], 0)
            finally:
                try:
                    root.destroy()
                except tk.TclError:
                    pass


if __name__ == "__main__":
    unittest.main()
