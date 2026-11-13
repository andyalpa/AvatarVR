"""Finger pose editor model. Keys match hand_gesture_tune.hpp; no game writes on import."""
import math

COMBINATIONS = (
    "open", "thumb", "index_touch", "index_touch_thumb", "index_press", "index_press_thumb",
    "finger_gun", "point", "grip_index_touch", "grip_index_touch_thumb", "thumbs_up", "fist", "idle")
LABELS = (
    "Open: no squeeze, index lifted, thumb lifted",
    "Thumb folded: no squeeze, index lifted",
    "Index touch: no squeeze, thumb lifted",
    "Index + thumb touch: no squeeze",
    "Index press: no squeeze, thumb lifted",
    "Index press + thumb folded: no squeeze",
    "Finger gun: squeeze, index lifted, thumb lifted",
    "Point: squeeze, index lifted, thumb folded",
    "Squeeze + index touch: thumb lifted",
    "Squeeze + index + thumb touch",
    "Thumbs up: squeeze + index press, thumb lifted",
    "Fist: squeeze + index press + thumb folded",
    "Relaxed idle: free weapon offhand / released mounted hand")
FINGERS = ("thumb", "index", "middle", "pinky")
EXTRAS = ("root_pitch", "root_yaw", "root_roll", "root_normal", "root_side", "root_forward",
          "spread1", "spread2", "spread3", "roll1", "roll2", "roll3")
AXES = ("root_pitch", "root_yaw", "root_roll", "root_normal", "root_side", "root_forward",
        "bend1", "spread1", "roll1", "bend2", "spread2", "roll2", "bend3", "spread3", "roll3", "splay", "roll")
HELPERS = ("thumb_01_expand01", "thumb_01_expand02", "thumb_01_expand03", "thumb_01_expand04",
           "fingerThumb_02_expand01", "fingerThumb_02_expand02", "fingerThumb_02_expand03",
           "fingerThumb_03_expand01", "fingerThumb_03_expand02", "fingerThumb_03_expand03",
           "fingerThumb_02_driver01", "fingerThumb_02_driver02", "fingerThumb_03_driver01", "fingerThumb_03_driver02")
HELPER_AXES = ("pitch", "yaw", "roll", "offset_x", "offset_y", "offset_z", "scale_x", "scale_y", "scale_z")


def bounds(key):
    bone, axis = key.split(".")
    if bone in HELPERS:
        return (-90, 90) if axis in ("pitch", "yaw", "roll") else (-20, 20) if axis.startswith("offset") else (-75, 75)
    if axis.startswith("bend"):
        return -30, 120
    if axis in ("root_normal", "root_side", "root_forward"):
        return -20, 20
    if axis.startswith("root_"):
        return -90, 90
    if axis.startswith("spread"):
        return -60, 60
    return (-85, 85) if axis == "splay" else (-180, 180)


def defaults(combo):
    idle = combo == 12
    grip, thumb = not idle and combo >= 6, not idle and combo % 2
    index = 0 if idle else combo % 6 // 2
    curls = (.15 if idle else 1 if thumb else 0,
             .15 if idle else 1 if index == 2 else .22 if index == 1 else 0,
             .2 if idle else 1 if grip else 0, .2 if idle else 1 if grip else 0)
    opened = ((0, 8, 5), (4, 6, 3), (7, 8, 5), (10, 10, 6))
    closed = ((45, 40, 25), (65, 85, 60), (70, 90, 65), (75, 90, 65))
    out = {}
    for f, finger in enumerate(FINGERS):
        for j in range(3):
            out[finger + ".bend" + str(j + 1)] = opened[f][j] + curls[f] * (closed[f][j] - opened[f][j])
        out[finger + ".splay"] = 60 - 25 * curls[f] if f == 0 else 4 if f == 1 else -7 if f == 3 else 0
        out[finger + ".roll"] = 0
        for axis in EXTRAS:
            out[finger + "." + axis] = 0
    for bone in HELPERS:
        for axis in HELPER_AXES:
            out[bone + "." + axis] = 0
    return out


def capture_values(payload):
    """Slider values from the DLL's AvatarVR_gesture_capture.json; unknown keys dropped, ranges clamped."""
    known = defaults(0)
    out = {}
    for key, value in payload.get("values", {}).items():
        if key in known and isinstance(value, (int, float)) and math.isfinite(value):
            lo, hi = bounds(key)
            out[key] = min(hi, max(lo, float(value)))
    return out


def bone_values(pose, bone):
    """One finger's (or thumb helper's) slider values, without the bone name: a copy/paste clipboard."""
    axes = HELPER_AXES if bone in HELPERS else AXES
    return {axis: pose[bone + "." + axis] for axis in axes}


def paste_bone(pose, bone, values):
    """Pose with `values` applied to `bone`. Fingers share axes, helpers share axes; other axes are ignored."""
    out = dict(pose)
    for axis, value in values.items():
        key = bone + "." + axis
        if key in out and math.isfinite(value):
            lo, hi = bounds(key)
            out[key] = min(hi, max(lo, value))
    return out


def prefix(context, hand, combo):
    return "gesture.%s.%s.%s." % (context, hand, COMBINATIONS[combo])


class GestureModel:
    def __init__(self, values=None):
        self.values = {k: v for k, v in (values or {}).items()
                       if k.startswith("gesture.") and not k.startswith("gesture.preview_") and math.isfinite(v)}

    def pose(self, context, hand, combo):
        out = defaults(combo)
        stem = prefix(context, hand, combo)
        return {k: self.values.get(stem + k, v) for k, v in out.items()}

    def store(self, context, hand, combo, values):
        stem = prefix(context, hand, combo)
        for key, default in defaults(combo).items():
            value = values.get(key, default)
            if not math.isfinite(value):
                continue
            lo, hi = bounds(key)
            value = min(hi, max(lo, value))
            if value != default:
                self.values[stem + key] = value
            else:
                self.values.pop(stem + key, None)

    def reset(self, context, hand, combo):
        stem = prefix(context, hand, combo)
        self.values = {k: v for k, v in self.values.items() if not k.startswith(stem)}

    def copy(self, source_context, source_hand, context, hand, combo):
        self.store(context, hand, combo, self.pose(source_context, source_hand, combo))
