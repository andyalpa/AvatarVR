#!/usr/bin/env python3
"""AvatarVR IK tuner: mounted arm adjustments and live per-combination finger poses.

Writes <game>/scripts/AvatarVR_ik_tune.txt on every change; the mod re-reads it about 4 times a second, so the
headset shows the change almost immediately. Keys are prefixed "tp." (third-person rider rig); "fp." is reserved
for the first-person rig. Rotation values left on "inherit" use the first-person hand tune. Mounted position offsets
start at zero and are authored here; side is mirrored automatically. This tool writes the tune file and saved presets.
"""
import argparse
import json
import math
from fullview_tuning import FullViewTab
from hud_tuning import HudWizard
from gesture_tuning import (COMBINATIONS, LABELS, FINGERS, AXES, defaults, prefix, GestureModel, HELPERS, HELPER_AXES, bounds,
                            capture_values, bone_values, paste_bone)
import os
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SCRIPTS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "scripts"))
TUNE_FILE = "AvatarVR_ik_tune.txt"
SETTINGS_FILE = "AvatarVR_settings.ini"  # its [arms] section: first-person settings (hand tune the third person inherits)
ARMS_FILE = "AvatarVR_arms.txt"  # the same settings before the first launch moves them there

# key, label, minimum, maximum, step, default, unit, inheritable-from (first-person key) or None, help
TP_GROUPS = [
    ("Mounted correction", [
        ("tp.mount_correction", "Symmetry + hand helpers", 0, 1, 1, 1, "0 off / 1 on", None,
         "Mirror right shoulder placement onto the left and correct thumb helpers. 0 restores the previous mounted solve."),
    ]),
    ("Hand", [
        ("tp.hand_pitch", "Pitch", -180, 180, 1, 0.0, "deg", "hand_pitch", "Tilts the hand up/down around the knuckles."),
        ("tp.hand_yaw", "Yaw", -180, 180, 1, 0.0, "deg", "hand_yaw", "Turns the hand left/right."),
        ("tp.hand_roll", "Roll", -180, 180, 1, 0.0, "deg", "hand_roll", "Rolls the hand (palm in / palm down)."),
        ("tp.hand_forward", "Forward", -0.3, 0.3, 0.005, 0.0, "m", None, "Moves both mounted hand targets forward in the body frame."),
        ("tp.hand_up", "Up", -0.3, 0.3, 0.005, 0.0, "m", None, "Raises both mounted hand targets equally."),
        ("tp.hand_side", "Side (mirrored)", -0.3, 0.3, 0.005, 0.0, "m", None, "Right-authored lateral offset; the left hand receives the opposite sign."),
        ("tp.hand_palm", "Palm depth", 0.0, 1.5, 0.01, 0.75, "", None, "Wrist distance behind the grip, in knuckle lengths."),
    ]),
    ("Shoulder (rest position, relative to your eyes)", [
        ("tp.shoulder_out", "Out", -0.3, 0.3, 0.005, 0.0, "m", None, "+ moves both shoulders away from the body centre."),
        ("tp.shoulder_up", "Up", -0.3, 0.3, 0.005, 0.0, "m", None, "+ raises both shoulders."),
        ("tp.shoulder_forward", "Forward", -0.3, 0.3, 0.005, 0.0, "m", None, "+ moves both shoulders forward."),
    ]),
    ("Reach", [
        ("tp.reach_scale", "Reach scale", 0.5, 1.5, 0.01, 1.0, "x", None, "Below 1 keeps the elbows bent (less stretched arms)."),
        ("tp.max_reach", "Clavicle reach", 0.0, 0.3, 0.005, 0.12, "m", None, "How far the shoulder may roll toward a far target."),
    ]),
    ("Elbow direction", [
        ("tp.elbow_out", "Outward", -2.0, 2.0, 0.05, 1.0, "", None, "Weight pushing the elbows outward."),
        ("tp.elbow_down", "Down", -2.0, 2.0, 0.05, 1.0, "", None, "Weight pushing the elbows down."),
        ("tp.elbow_back", "Back", -2.0, 2.0, 0.05, 0.2, "", None, "Weight pushing the elbows back."),
    ]),
    ("Twist (wrist roll shared along the arm)", [
        ("tp.twist_fore", "Forearm", 0.0, 1.5, 0.05, 1.0, "", None, "Share of the hand roll the forearm takes."),
        ("tp.twist_upper", "Upper arm", 0.0, 1.0, 0.05, 0.3, "", None, "Share of the hand roll the upper arm takes."),
    ]),
]

def arm_groups(side):
    """Independent additive trims. Zero leaves the shared/current setup exactly as it is."""
    groups = []
    for title, specs in TP_GROUPS:
        if title == "Mounted correction":
            continue
        rows = []
        for key, label, lo, hi, step, default, unit, inherited, tip in specs:
            short = key[3:]
            if short == "hand_palm": lo, hi = -1.5, 1.5
            if short == "reach_scale": lo, hi = -1.0, 1.0
            if short in ("max_reach", "twist_fore", "twist_upper"): lo = -hi
            rows.append(("tp." + side + "." + short, label + " trim", lo, hi, step, 0.0, unit, None,
                         side.title() + " only: added to the shared value. Zero keeps the current setup."))
        if title.startswith("Twist"):
            for key, label in (("forearm_roll", "Forearm skin roll"), ("upper_roll", "Upper-arm skin roll")):
                rows.append(("tp." + side + "." + key, label, -180, 180, 1, 0.0, "deg", None,
                             "Extra skin roll on this arm only. Start with small changes; zero adds no roll."))
        groups.append((title, rows))
    return groups


def read_arms(scripts):
    """Numeric values of [arms] in the mod's settings file, or of the old file if it has not moved yet."""
    old = os.path.join(scripts, ARMS_FILE)
    if os.path.exists(old):
        return read_kv(old, " ")
    out, inside = {}, False
    try:
        with open(os.path.join(scripts, SETTINGS_FILE), encoding="utf-8", errors="replace") as f:
            for line in f:
                parts = line.split()
                if line.startswith("[") and line.rstrip().endswith("]"):
                    inside = line.strip() == "[arms]"
                elif inside and len(parts) == 2:
                    try:
                        if math.isfinite(float(parts[1])):
                            out[parts[0]] = float(parts[1])
                    except ValueError:
                        pass
    except OSError:
        pass
    return out


def read_kv(path, sep):
    out = {}
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if sep == "=" and "=" in line:
                    k, v = line.split("=", 1)
                elif sep == " " and " " in line:
                    k, v = line.split(None, 1)
                else:
                    continue
                try:
                    value = float(v.strip())
                    if math.isfinite(value):
                        out[k.strip()] = value
                except ValueError:
                    pass
    except OSError:
        pass
    return out


class Row:
    def __init__(self, parent, spec, on_change, inherited):
        key, label, lo, hi, step, default, unit, inherit_key, tip = spec
        self.key, self.lo, self.hi, self.step, self.default, self.inherit_key = key, lo, hi, step, default, inherit_key
        self.on_change = on_change
        self.inherited = inherited  # value the mod uses while inheriting
        self.var = tk.DoubleVar(value=default)
        self.inherit = tk.BooleanVar(value=inherit_key is not None)
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=1)
        ttk.Label(frame, text=label, width=14).pack(side="left")
        self.scale = ttk.Scale(frame, from_=lo, to=hi, variable=self.var, command=lambda _v: self._slid())
        self.scale.pack(side="left", fill="x", expand=True, padx=4)
        self.entry = ttk.Entry(frame, width=8)
        self.entry.pack(side="left")
        self.entry.bind("<Return>", lambda _e: self._typed())
        self.entry.bind("<FocusOut>", lambda _e: self._typed())
        ttk.Label(frame, text=unit, width=4).pack(side="left")
        ttk.Button(frame, text="-", width=2, command=lambda: self.nudge(-1)).pack(side="left")
        ttk.Button(frame, text="+", width=2, command=lambda: self.nudge(1)).pack(side="left")
        ttk.Button(frame, text="reset", width=6, command=self.reset).pack(side="left", padx=(4, 0))
        if inherit_key:
            ttk.Checkbutton(frame, text="inherit on-foot", variable=self.inherit, command=self._inherit_toggled).pack(side="left", padx=4)
        else:
            ttk.Label(frame, text="", width=15).pack(side="left", padx=4)
        frame.bind("<Enter>", lambda _e: on_change.status(tip))
        self._show()

    def _clamp(self, v):
        v = min(max(v, self.lo), self.hi)
        return round(round(v / self.step) * self.step, 6)

    def _show(self):
        self.entry.delete(0, "end")
        self.entry.insert(0, ("%g" % self.value()))
        state = "disabled" if self.is_inheriting() else "normal"
        self.scale.state(["disabled"] if state == "disabled" else ["!disabled"])

    def is_inheriting(self):
        return self.inherit_key is not None and self.inherit.get()

    def value(self):
        return self.inherited if self.is_inheriting() else self._clamp(self.var.get())

    def set(self, v, inherit=False):
        if self.inherit_key is not None:
            self.inherit.set(inherit)
        self.var.set(self._clamp(v))
        self._show()

    def _slid(self):
        self.var.set(self._clamp(self.var.get()))
        self._show()
        self.on_change()

    def _typed(self):
        try:
            v = float(self.entry.get())
        except ValueError:
            self._show()
            return
        if self.inherit_key is not None:
            self.inherit.set(False)
        self.var.set(self._clamp(v))
        self._show()
        self.on_change()

    def nudge(self, sign):
        if self.is_inheriting():
            self.inherit.set(False)
            self.var.set(self.inherited)
        self.var.set(self._clamp(self.var.get() + sign * self.step))
        self._show()
        self.on_change()

    def reset(self):
        self.set(self.default, inherit=self.inherit_key is not None)
        self.on_change()

    def _inherit_toggled(self):
        if not self.inherit.get():
            self.var.set(self._clamp(self.inherited))
        self._show()
        self.on_change()

    def line(self):
        if self.is_inheriting():
            return None
        return "%s=%g" % (self.key, self._clamp(self.var.get()))


class Changed:
    """Debounced writer: every change schedules one write 100 ms later."""

    def __init__(self, app):
        self.app = app
        self.pending = None

    def __call__(self):
        if self.pending is not None:
            self.app.root.after_cancel(self.pending)
        self.pending = self.app.root.after(100, self._fire)

    def _fire(self):
        self.pending = None
        self.app.write()

    def status(self, text):
        self.app.status.set(text)


class GestureEditor:
    def __init__(self, parent, app, changed, saved):
        self.app, self.changed = app, changed
        self.model = GestureModel(saved)
        self.context = tk.StringVar(value="On foot")
        self.hand = tk.StringVar(value="Left")
        self.combo = tk.StringVar(value=LABELS[11])
        self.preview = tk.BooleanVar(value=False)
        self.current = ("fp", "left", 11)
        self.rows = []
        self.live = tk.StringVar(value="Start gameplay with Empty-hand gestures enabled to see live inputs.")
        ttk.Label(parent, text="Finger poses only. Arm / wrist / weapon settings stay separate. Adjust one hand and rig at a time.",
                  wraplength=850).pack(anchor="w")
        selectors = ttk.Frame(parent); selectors.pack(fill="x", pady=6)
        for label, var, values, width in (
                ("Rig", self.context, ("On foot", "Mounted"), 12),
                ("Hand", self.hand, ("Left", "Right"), 8),
                ("Combination", self.combo, LABELS, 56)):
            ttk.Label(selectors, text=label).pack(side="left", padx=(0, 4))
            box = ttk.Combobox(selectors, textvariable=var, values=values, state="readonly", width=width)
            box.pack(side="left", padx=(0, 10)); box.bind("<<ComboboxSelected>>", self.select)
        ttk.Checkbutton(parent, text="Preview selected pose on the selected empty hand (while this tuner is open)",
                        variable=self.preview, command=changed).pack(anchor="w")
        self.mirror = tk.BooleanVar(value=bool(self.prefs().get("mirror_hands", False)))
        ttk.Checkbutton(parent, text="Mirror edits to the other hand (same rig and combination; the hand you edit overwrites the other)",
                        variable=self.mirror, command=self.mirror_toggled).pack(anchor="w")
        poses = ttk.Frame(parent); poses.pack(fill="x", pady=(4, 0))
        ttk.Button(poses, text="Copy pose", command=self.copy_pose).pack(side="left")
        ttk.Button(poses, text="Paste pose", command=self.paste_pose).pack(side="left", padx=4)
        ttk.Label(poses, text="Whole combination (all fingers + thumb helpers): copy, switch hand / rig / combination, paste.",
                  foreground="#555").pack(side="left", padx=6)
        ttk.Label(parent, text="Preview obeys cinematic, item, grip and harness ownership. Stop preview to test real touch/press inputs.",
                  wraplength=850, foreground="#555").pack(anchor="w")
        self.capture_id, self.capture_deadline = 0, 0
        self.capture_text = tk.StringVar(value="")
        capture = ttk.Frame(parent); capture.pack(fill="x", pady=(4, 0))
        ttk.Button(capture, text="Capture native pose", command=self.capture).pack(side="left")
        ttk.Label(capture, text="Copies the game's current finger animation on the selected hand into this combination; then edit.",
                  foreground="#555").pack(side="left", padx=6)
        ttk.Label(parent, textvariable=self.capture_text, wraplength=870).pack(anchor="w")
        self.clipboard = None  # (kind, values, description); survives hand/rig/combination changes
        self.clipboard_text = tk.StringVar(value="Bone clipboard: empty. Use Copy bone, switch hand/rig/combination, then Paste bone.")
        ttk.Label(parent, textvariable=self.clipboard_text, wraplength=870, foreground="#555").pack(anchor="w")
        ttk.Label(parent, textvariable=self.live, wraplength=870).pack(anchor="w", pady=5)
        canvas = tk.Canvas(parent, highlightthickness=0)
        bar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y");canvas.pack(fill="both", expand=True)
        inner = ttk.Frame(canvas); window = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
        canvas.bind("<MouseWheel>", lambda e: canvas.yview_scroll(-int(e.delta/120), "units"))
        self.canvas = canvas
        values = self.model.pose(*self.current)
        self.helper = tk.StringVar(value=HELPERS[0])
        self.helper_rows = []
        for finger in FINGERS:
            box = ttk.LabelFrame(inner, text={"middle": "Lower finger 1", "pinky": "Lower finger 2"}.get(finger, finger.title()) +
                                 " / base + joints 01 / 02 / 03", padding=4)
            box.pack(fill="x", pady=3)
            self.bone_buttons(box, lambda finger=finger: finger)
            for axis in AXES:
                key = finger + "." + axis
                labels = {"root_pitch": "Base pitch", "root_yaw": "Base oppose", "root_roll": "Base roll",
                          "root_normal": "Base normal", "root_side": "Base sideways", "root_forward": "Base forward",
                          "splay": "All spread", "roll": "All skin roll"}
                label = labels.get(axis)
                if label is None:
                    label = "0%s %s" % (axis[-1], "bend" if axis.startswith("bend") else
                                        "sideways" if axis.startswith("spread") else "skin roll")
                lo, hi = bounds(key)
                position = axis in ("root_normal", "root_side", "root_forward")
                row = Row(box, (key, label, lo, hi, .25 if position else 1, defaults(self.current[2])[key],
                                "mm" if position else "deg", None,
                                "Base controls move the whole finger in its anatomical palm. Joint 01/02/03 controls are independent. Zero new trims preserves saved poses."),
                          changed, values[key])
                row.set(values[key])
                self.rows.append(row)
            if finger == "thumb":
                helpers = ttk.LabelFrame(inner, text="Thumb webbing / deformation bones", padding=4)
                helpers.pack(fill="x", pady=3)
                chooser = ttk.Combobox(helpers, textvariable=self.helper, values=HELPERS, state="readonly", width=35)
                chooser.pack(anchor="w");chooser.bind("<<ComboboxSelected>>", self.select_helper)
                self.bone_buttons(helpers, self.helper.get)
                ttk.Label(helpers, text="Advanced: local bone axes. Start at 0. Missing bones in a mesh have no effect.",
                          wraplength=800).pack(anchor="w")
                for axis in HELPER_AXES:
                    key = self.helper.get() + "." + axis
                    label = {"pitch": "Rotate X", "yaw": "Rotate Y", "roll": "Rotate Z"}.get(axis, axis.replace("_", " ").title())
                    lo, hi = bounds(key)
                    unit = "mm" if axis.startswith("offset") else "%" if axis.startswith("scale") else "deg"
                    row = Row(helpers, (key, label, lo, hi, .25 if unit=="mm" else 1, 0, unit, None,
                                       "Adjust only this named thumb helper bone; does not move the finger chain or the wrist."),
                              changed, values[key])
                    row.set(values[key]);self.rows.append(row);self.helper_rows.append(row)
        buttons = ttk.Frame(inner);buttons.pack(fill="x", pady=6)
        ttk.Button(buttons, text="Reset this pose", command=self.reset).pack(side="left")
        ttk.Button(buttons, text="Copy pose to other hand", command=self.copy_hand).pack(side="left", padx=5)
        ttk.Button(buttons, text="Copy pose to other rig", command=self.copy_context).pack(side="left")
        self.app.root.after(500, self.poll)

    def bone_buttons(self, parent, bone):
        bar = ttk.Frame(parent); bar.pack(anchor="w", pady=(0, 2))
        ttk.Button(bar, text="Copy bone", command=lambda: self.copy_bone(bone())).pack(side="left")
        ttk.Button(bar, text="Paste bone", command=lambda: self.paste_bone(bone())).pack(side="left", padx=4)

    def describe(self, bone):
        context, hand, combo = self.current
        return "%s (%s %s, %s)" % (bone, hand, "mounted" if context == "tp" else "on foot", COMBINATIONS[combo])

    def copy_bone(self, bone):
        self.remember()
        kind = "helper" if bone in HELPERS else "finger"
        self.clipboard = (kind, bone_values(self.model.pose(*self.current), bone), self.describe(bone))
        self.clipboard_text.set("Bone clipboard: " + self.clipboard[2] + ". Paste onto any " +
                                ("thumb helper." if kind == "helper" else "finger."))

    def paste_bone(self, bone):
        kind = "helper" if bone in HELPERS else "finger"
        if self.clipboard is None or self.clipboard[0] != kind:
            self.clipboard_text.set("Bone clipboard: nothing to paste onto a %s. Copy a %s first." % (kind, kind))
            return
        self.remember()
        self.model.store(*self.current, paste_bone(self.model.pose(*self.current), bone, self.clipboard[1]))
        values = self.model.pose(*self.current)
        for row in self.rows:
            row.set(values[row.key])
        self.clipboard_text.set("Pasted %s onto %s." % (self.clipboard[2], self.describe(bone)))
        self.changed()

    def prefs_path(self):
        return os.path.join(self.app.scripts, "AvatarVR_tuner_prefs.json")

    def prefs(self):
        try:
            with open(self.prefs_path(), encoding="utf-8") as file:
                data = json.load(file)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def mirror_toggled(self):
        data = self.prefs(); data["mirror_hands"] = bool(self.mirror.get())
        try:
            with open(self.prefs_path(), "w", encoding="utf-8") as file:
                json.dump(data, file)
        except OSError:
            pass
        if self.mirror.get():
            self.changed()  # mirror the pose on screen right away

    def copy_pose(self):
        self.remember()
        context, hand, combo = self.current
        self.pose_clipboard = (self.model.pose(*self.current),
                               "%s (%s %s)" % (COMBINATIONS[combo], hand, "mounted" if context == "tp" else "on foot"))
        self.clipboard_text.set("Pose clipboard: " + self.pose_clipboard[1] + ". Select a combination and press Paste pose.")

    def paste_pose(self):
        clip = getattr(self, "pose_clipboard", None)
        if clip is None:
            self.clipboard_text.set("Pose clipboard: empty. Press Copy pose first.")
            return
        self.model.store(*self.current, clip[0])
        values = self.model.pose(*self.current)
        for row in self.rows:
            row.set(values[row.key])
        context, hand, combo = self.current
        self.clipboard_text.set("Pasted pose %s onto %s (%s %s)." % (clip[1], COMBINATIONS[combo], hand,
                                "mounted" if context == "tp" else "on foot"))
        self.changed()

    def remember(self):
        values = self.model.pose(*self.current)
        values.update({r.key: r.value() for r in self.rows})
        self.model.store(*self.current, values)
        if self.mirror.get():
            # Values are anatomical; the runtime applies the hand sign, so the mirror is an exact copy.
            context, hand, combo = self.current
            self.model.store(context, "right" if hand == "left" else "left", combo, values)

    def select_helper(self, _event=None):
        self.remember()
        values = self.model.pose(*self.current)
        for row, axis in zip(self.helper_rows, HELPER_AXES):
            row.key = self.helper.get() + "." + axis
            row.set(values[row.key])

    def select(self, _event=None):
        self.remember()
        self.current = ("tp" if self.context.get() == "Mounted" else "fp",
                        self.hand.get().lower(), LABELS.index(self.combo.get()))
        values = self.model.pose(*self.current)
        base = defaults(self.current[2])
        for row in self.rows:
            row.default = base[row.key]; row.set(values[row.key])
        if self.preview.get():
            self.changed()

    def reset(self):
        self.model.reset(*self.current)
        values = defaults(self.current[2])
        for row in self.rows:
            row.set(values[row.key])
        self.changed()

    def copy_hand(self):
        self.remember()
        context, hand, combo = self.current
        self.model.copy(context, hand, context, "right" if hand == "left" else "left", combo)
        self.changed()

    def copy_context(self):
        self.remember()
        context, hand, combo = self.current
        self.model.copy(context, hand, "tp" if context == "fp" else "fp", hand, combo)
        self.changed()

    def values(self, preview=True):
        self.remember()
        values = dict(self.model.values)
        context, hand, combo = self.current
        values.update({"gesture.preview_until": time.time() + 2 if preview and self.preview.get() else 0,
                       "gesture.preview_context": int(context == "tp"),
                       "gesture.preview_hand": int(hand == "right"), "gesture.preview_combo": combo})
        waiting = preview and time.time() < self.capture_deadline
        values.update({"gesture.capture_hand": int(hand == "right") if waiting else -1,
                       "gesture.capture_id": self.capture_id if waiting else 0,
                       "gesture.capture_until": self.capture_deadline if waiting else 0})
        return values

    def capture(self):
        self.capture_id = int(time.time() * 1000) % 1000000000
        self.capture_deadline = time.time() + 5
        self.capture_text.set("Capturing %s hand... keep the hand visible in game." % self.hand.get().lower())
        self.app.write()

    def capture_poll(self):
        if not self.capture_id:
            return
        if time.time() >= self.capture_deadline:
            self.capture_id = 0
            self.capture_text.set("Capture timed out. Needs gameplay with Empty-hand gestures + Arm IK + Script writes, hand in view.")
            self.app.write()
            return
        try:
            with open(os.path.join(self.app.scripts, "AvatarVR_gesture_capture.json"), encoding="utf-8") as file:
                payload = json.load(file)
        except (OSError, ValueError):
            return
        if payload.get("id") != self.capture_id:
            return
        self.capture_id, self.capture_deadline = 0, 0
        captured = capture_values(payload)
        self.remember()
        context = "tp" if payload.get("context") == "tp" else "fp"
        self.context.set("Mounted" if context == "tp" else "On foot")
        self.current = (context, self.current[1], self.current[2])
        values = self.model.pose(*self.current)
        values.update(captured)
        self.model.store(*self.current, values)
        values = self.model.pose(*self.current)
        base = defaults(self.current[2])
        for row in self.rows:
            row.default = base[row.key]; row.set(values[row.key])
        self.capture_text.set("Captured %s hand (%s) into '%s'. Fit error %.1f mm / %.1f deg%s. Edit sliders as needed." % (
            payload.get("hand", "?"), "mounted" if context == "tp" else "on foot", COMBINATIONS[self.current[2]],
            payload.get("position_mm", 0), payload.get("angle_deg", 0),
            ", %d values hit slider limits" % payload["clamped"] if payload.get("clamped") else ""))
        self.changed()

    def poll(self):
        try:
            with open(os.path.join(self.app.scripts, "AvatarVR_gesture_status.json"), encoding="utf-8") as file:
                status = json.load(file)
            if time.time() - status["at"] > 3:
                raise ValueError("stale")
            owners = ("unknown", "empty", "weapon", "item/action", "support", "harness", "swapping")
            messages = []
            for h, label in enumerate(("L", "R")):
                owner = status["owners"][h]
                combo = status["combo"][h]
                messages.append("%s %s | %s | weight %.2f | grip %.2f trigger %.2f touch %d/%d" %
                                (label, owners[owner] if 0 <= owner < len(owners) else "unknown",
                                 COMBINATIONS[combo] if 0 <= combo < len(COMBINATIONS) else "unknown",
                                 status["weight"][h], status["grip"][h], status["trigger"][h],
                                 status["index_touch"][h], status["thumb_touch"][h]))
            lost = [side for h, side in enumerate(("Left", "Right")) if not status.get("tracked", [1, 1])[h]]
            self.live.set(("Mounted" if status["mounted"] else "On foot") +
                          (" | moving: thumb ignored, untouched hands native" if status.get("moving") else "") +
                          (" | native ownership veto" if status["native_owned"] else "") +
                          "".join(" | %s controller NOT TRACKED (OpenXR runtime)" % side for side in lost) +
                          "\n" + "\n".join(messages))
        except (OSError, ValueError, KeyError, IndexError, TypeError):
            self.live.set("No fresh gesture status. Enable Empty-hand gestures + Arm IK + Script writes in gameplay.")
        self.capture_poll()
        if self.preview.get() or self.capture_id:
            self.app.write()
        self.app.root.after(750, self.poll)


class App:
    def __init__(self, root, scripts):
        self.root, self.scripts = root, scripts
        self.path = os.path.join(scripts, TUNE_FILE)
        root.title("AvatarVR IK tuner")
        root.minsize(760, 0)
        root.geometry("940x%d" % min(880, max(600, root.winfo_screenheight() - 120)))
        self.status = tk.StringVar(value="")
        changed = Changed(self)
        fp = read_arms(scripts)
        saved = read_kv(self.path, "=")

        top = ttk.Frame(root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Writes: " + self.path).pack(side="left")

        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, padx=6)
        tp = ttk.Frame(notebook, padding=6)
        notebook.add(tp, text="Third person (mounted)")
        fpf = ttk.Frame(notebook, padding=12)
        notebook.add(fpf, text="First person (on foot)")
        ttk.Label(fpf, text="First-person tuning comes next. For now the on-foot hand tune is in the VR menu;\n"
                            "the third-person hand rows can inherit it.", justify="left").pack(anchor="w")

        ttk.Label(tp, text="Shared setup stays intact. Left / Right adjustments add to it; zero keeps the current offsets.",
                  foreground="#555").pack(anchor="w", pady=(0, 6))
        arms = ttk.Notebook(tp)
        arms.pack(fill="both", expand=True)
        canvases = {}
        def page(label):
            outer = ttk.Frame(arms)
            arms.add(outer, text=label)
            canvas = tk.Canvas(outer, highlightthickness=0)
            bar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=bar.set)
            bar.pack(side="right", fill="y"); canvas.pack(side="left", fill="both", expand=True)
            inner = ttk.Frame(canvas, padding=4)
            window = canvas.create_window((0, 0), window=inner, anchor="nw")
            inner.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
            canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
            canvases[str(outer)] = canvas
            return inner
        shared = page("Shared offsets")
        left, right = page("Left arm adjustments"), page("Right arm adjustments")
        root.bind_all("<MouseWheel>", lambda e: canvases[arms.select()].yview_scroll(-int(e.delta/120), "units")
                      if notebook.select() == str(tp) and arms.select() in canvases else None)
        self.rows = []
        for parent, groups, side in ((shared, TP_GROUPS, None), (left, arm_groups("left"), "left"),
                                     (right, arm_groups("right"), "right")):
            if side:
                ttk.Label(parent, text="Changes affect this arm only. Side/out is outward; angle trims apply after mirroring.").pack(anchor="w")
                ttk.Button(parent, text="Reset %s adjustments to zero" % side,
                           command=lambda side=side: self.reset_arm(side)).pack(anchor="w", pady=4)
            for title, specs in groups:
                box = ttk.LabelFrame(parent, text=title, padding=6)
                box.pack(fill="x", pady=3)
                for spec in specs:
                    inherited = fp.get(spec[7], spec[5]) if spec[7] else spec[5]
                    row = Row(box, spec, changed, inherited)
                    if spec[0] in saved:
                        row.set(saved[spec[0]], inherit=False)
                    elif spec[7]:
                        row.set(inherited, inherit=True)
                    self.rows.append(row)

        fingers = ttk.Frame(notebook, padding=6)
        notebook.add(fingers, text="Finger gestures")
        self.gestures = GestureEditor(fingers, self, changed, saved)
        hud = ttk.Frame(notebook, padding=8)
        notebook.add(hud, text="HUD on arm")
        self.hud = HudWizard(hud, root, scripts)
        fullview = ttk.Frame(notebook, padding=8)
        notebook.add(fullview, text="Full-view effects")
        self.fullview = FullViewTab(fullview, root, scripts)
        root.bind_all("<MouseWheel>", lambda e:
                      self.gestures.canvas.yview_scroll(-int(e.delta/120), "units")
                      if notebook.select() == str(fingers) else
                      canvases[arms.select()].yview_scroll(-int(e.delta/120), "units")
                      if notebook.select() == str(tp) and arms.select() in canvases else None)
        self.extra = {k: v for k, v in saved.items() if k not in {r.key for r in self.rows} and not k.startswith("gesture.")}
        root.protocol("WM_DELETE_WINDOW", self.close)

        bottom = ttk.Frame(root, padding=6)
        bottom.pack(fill="x")
        ttk.Button(bottom, text="Reset all", command=self.reset_all).pack(side="left")
        ttk.Button(bottom, text="Save preset...", command=self.save_preset).pack(side="left", padx=4)
        ttk.Button(bottom, text="Load preset...", command=self.load_preset).pack(side="left")
        ttk.Label(bottom, textvariable=self.status, foreground="#555").pack(side="left", padx=10)
        if not os.path.isdir(scripts):
            messagebox.showwarning("AvatarVR IK tuner", "Game scripts folder not found:\n" + scripts +
                                   "\nStart with --scripts <game>\\scripts")

    def text(self, preview=True):
        # Preserve unknown/current keys so finger tuning never erases other tuners.
        values = dict(self.extra)
        current = read_kv(self.path, "=")
        known = {r.key for r in self.rows}
        values.update({k: v for k, v in current.items() if k not in known and not k.startswith("gesture.")})
        for row in self.rows:
            if not row.is_inheriting():
                values[row.key] = row.value()
        values.update(self.gestures.values(preview))
        lines = ["# AvatarVR live arm + finger tuning; angles in degrees. Preview expires unless the tuner renews it."]
        lines += ["%s=%.17g" % (key, value) for key, value in sorted(values.items(), key=lambda item: (item[0].startswith("gesture."), item[0]))]
        return "\r\n".join(lines) + "\r\n"

    def close(self):
        if self.gestures.preview.get():
            self.gestures.preview.set(False)
            self.write()
        self.root.destroy()

    def write(self):
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8", newline="") as f:
                f.write(self.text())
            os.replace(tmp, self.path)
            self.status.set("Live: written " + time.strftime("%H:%M:%S"))
        except OSError as e:
            self.status.set("Write failed: %s" % e)

    def reset_all(self):
        self.gestures.model = GestureModel()
        self.gestures.reset()
        for r in self.rows:
            r.set(r.default, inherit=r.inherit_key is not None)
        self.write()

    def reset_arm(self, side):
        for r in self.rows:
            if r.key.startswith("tp." + side + "."):
                r.set(0.0, inherit=False)
        self.write()

    def save_preset(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialdir=self.scripts,
                                            filetypes=[("IK tune", "*.txt")], title="Save IK preset")
        if path:
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(self.text(preview=False))
            self.status.set("Preset saved: " + os.path.basename(path))

    def load_preset(self):
        path = filedialog.askopenfilename(initialdir=self.scripts, filetypes=[("IK tune", "*.txt")], title="Load IK preset")
        if not path:
            return
        values = read_kv(path, "=")
        self.gestures.remember()
        self.gestures.model = GestureModel(values)
        self.gestures.preview.set(False)
        loaded = self.gestures.model.pose(*self.gestures.current)
        for row in self.gestures.rows:
            row.set(loaded[row.key])
        self.extra.update({k: v for k, v in values.items() if not k.startswith("gesture.") and k not in {r.key for r in self.rows}})
        for r in self.rows:
            if r.key in values:
                r.set(values[r.key], inherit=False)
            else:
                r.set(r.inherited if r.inherit_key else r.default, inherit=r.inherit_key is not None)
        self.write()
        self.status.set("Preset loaded: " + os.path.basename(path))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scripts", default=DEFAULT_SCRIPTS, help="game scripts folder (default: %(default)s)")
    args = ap.parse_args()
    root = tk.Tk()
    App(root, os.path.abspath(args.scripts))
    root.mainloop()


if __name__ == "__main__":
    main()
