"""Wizard for placing the game's HUD widgets on the arms, hands or weapon. Selected elements move together.

Writes <game>/scripts/AvatarVR_hud_widgets.txt on every change; the mod re-reads it about 4 times a second while
F10 -> UI -> Native HUD on arm is on. Defaults here must match PlacementDefaults in src/camera/ui_ownership.cpp.
"""
import json
import os
import tkinter as tk
from tkinter import ttk

FILE = "AvatarVR_hud_widgets.txt"
GROUPS_FILE = "AvatarVR_hud_groups.json"
ADDITIVE = ("along", "side", "lift", "yaw", "pitch", "roll")  # moved by the same amount; attach, bone and back side are set; size scales
ATTACH = ["Flat HUD (not moved)", "Left arm", "Right arm", "Left hand", "Right hand", "Weapon", "Hidden", "Specific bone", "Head (follows the headset)", "Weapon aim (in the world, for the reticle)"]
BONE_ATTACH = 7
HEAD_ATTACH = 8
HEAD_START = dict(along=.3, side=0, lift=0)  # along 0 would sit inside the eyes, where nothing can draw
BONES_FILE = "AvatarVR_hud_bones.txt"  # written by the mod once it has read the skeleton
# key, label, attach, along, side, lift
ELEMENTS = [
    ("health", "Health bar", 1, .12, 0, .015), ("energy", "Energy arcs", 1, .12, 0, .015),
    ("heal", "Heal prompt", 1, .12, 0, .015), ("eat", "Eat food prompt", 1, .12, 0, .015),
    ("ammo", "Ammo and weapon slots", 1, .26, 0, .015), ("status", "Status icons (top right)", 3, 0, 0, .02),
    ("tracker", "Objective tracker", 0, 0, 0, .015), ("compass", "Compass", 0, 0, 0, .015),
    ("save", "Autosave dots", 0, 0, 0, .015), ("notify", "Centre notifications", 0, 0, 0, .015),
    ("centre", "Centre widget (unidentified, may be the reticle)", 0, 0, 0, .015), ("top", "Top centre widget (unidentified)", 0, 0, 0, .015),
    ("right", "Right of centre prompts (unidentified)", 0, 0, 0, .015), ("low", "Lower right of centre (unidentified)", 0, 0, 0, .015),
    ("reticle", "Reticle (size is on step 4)", 9, 0, 0, 0),
]
BOTTOM = ("health", "energy", "heal", "eat")  # share one layout centre, so equal values keep the game's arrangement
DEFAULT_GROUPS = {"Bottom HUD (health, energy, prompts)": list(BOTTOM)}
# key, label, minimum, maximum, step
POSITION = [("along", "Along (arm, hand: toward elbow; weapon, head: forward)", -.6, .6, .005),
            ("side", "Sideways (across the arm or gun; reticle: angle)", -.3, .3, .005), ("lift", "Off the surface (weapon, head: up; reticle: angle)", -.1, .2, .005)]
ANGLE = [("yaw", "Yaw", -180, 180, 1), ("pitch", "Pitch", -180, 180, 1), ("roll", "Roll", -180, 180, 1),
         ("scale", "Size", .2, 4, .05),
         ("shrink", "Reticle: smaller when far (0 same size, 1 like a real object)", 0, 1, .05)]
STEPS = ["1. Pick elements", "2. Choose where it attaches", "3. Move it", "4. Turn and size it"]


def defaults(element):
    key, _label, attach, along, side, lift = element
    return dict(attach=attach, along=along, side=side, lift=lift, yaw=0, pitch=0, roll=0, scale=1, bone=-1, two_sided=0,
                shrink=.5 if key == "reticle" else 0)


def read_bones(path):
    bones = []
    try:
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                index, _, name = line.strip().partition("=")
                if name:
                    bones.append((int(index), name))
    except (OSError, ValueError):
        pass
    return bones


def read_values(path):
    values = {e[0]: defaults(e) for e in ELEMENTS}
    try:
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                name, _, value = line.strip().partition("=")
                element, _, field = name.partition(".")
                if element in values and field in values[element]:
                    values[element][field] = float(value)
    except (OSError, ValueError):
        pass
    return values


def read_frame(path):
    try:
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                if line.startswith("settings.frame="):
                    return int(float(line.split("=")[1]))
    except (OSError, ValueError):
        pass
    return 1


def render(values, frame=1):
    lines = ["%s.%s=%g" % (e[0], field, values[e[0]][field]) for e in ELEMENTS for field in values[e[0]]]
    lines.append("settings.frame=%d" % frame)
    return "\n".join(lines) + "\n"


class HudWizard:
    def __init__(self, parent, root, scripts):
        self.root, self.path = root, os.path.join(scripts, FILE)
        self.values = read_values(self.path)
        self.frame = tk.IntVar(value=read_frame(self.path))
        self.groups_path = os.path.join(scripts, GROUPS_FILE)
        self.groups = dict(DEFAULT_GROUPS)
        try:
            with open(self.groups_path, encoding="utf-8") as stream:
                self.groups.update(json.load(stream))
        except (OSError, ValueError):
            pass
        self.bones_path = os.path.join(scripts, BONES_FILE)
        self.bones = read_bones(self.bones_path)
        self.pending = None
        self.loading = False
        self.status = tk.StringVar(value="Turn on F10 > UI > Native HUD on arm. Changes show in the headset within a moment.")
        self.step = 0
        self.title = tk.StringVar()
        ttk.Label(parent, textvariable=self.title, font=("", 11, "bold")).pack(anchor="w")
        self.body = ttk.Frame(parent)
        self.body.pack(fill="both", expand=True, pady=6)
        self.pages = [ttk.Frame(self.body) for _ in STEPS]
        self.vars = {}
        self.build_pick(self.pages[0])
        self.build_attach(self.pages[1])
        self.build_sliders(self.pages[2], POSITION)
        self.build_sliders(self.pages[3], ANGLE)
        self.two_sided = tk.IntVar()
        ttk.Checkbutton(self.pages[3], text="Show from behind too (off: hidden when you are not facing it)", variable=self.two_sided,
                        command=lambda: self.set_value("two_sided", self.two_sided.get())).pack(anchor="w", pady=6)
        nav = ttk.Frame(parent)
        nav.pack(fill="x")
        self.back = ttk.Button(nav, text="< Back", command=lambda: self.go(-1))
        self.back.pack(side="left")
        self.next = ttk.Button(nav, text="Next >", command=lambda: self.go(1))
        self.next.pack(side="left", padx=4)
        ttk.Button(nav, text="Reset selected", command=self.reset).pack(side="left", padx=12)
        ttk.Button(nav, text="Line up selected", command=self.line_up).pack(side="left")
        frames = ttk.LabelFrame(parent, text="Stability method: walk and stick-turn, keep whichever stays stuck to the bone", padding=4)
        frames.pack(fill="x", pady=(8, 0))
        for value, label in ((1, "A: rendering frame (default)"), (0, "B: newest frame"), (2, "C: arm rig frame")):
            ttk.Radiobutton(frames, text=label, value=value, variable=self.frame, command=self.schedule).pack(side="left", padx=8)
        ttk.Label(parent, textvariable=self.status, foreground="#555", wraplength=820).pack(anchor="w", pady=(8, 0))
        self.select(0)
        self.go(0)

    def build_pick(self, page):
        ttk.Label(page, text="Choose one element, or several with Ctrl+click or Shift+click. Selected elements move together and keep their spacing.").pack(anchor="w")
        row = ttk.Frame(page)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="Group:").pack(side="left")
        self.group = tk.StringVar()
        self.group_box = ttk.Combobox(row, textvariable=self.group, values=sorted(self.groups), width=36)
        self.group_box.pack(side="left", padx=4)
        self.group_box.bind("<<ComboboxSelected>>", lambda _e: self.select_group())
        ttk.Button(row, text="Save selection as this group", command=self.save_group).pack(side="left")
        ttk.Button(row, text="Delete group", command=self.delete_group).pack(side="left", padx=4)
        self.listbox = tk.Listbox(page, height=len(ELEMENTS), exportselection=False, selectmode="extended")
        self.listbox.pack(fill="x", pady=6)
        for element in ELEMENTS:
            self.listbox.insert("end", element[1])
        self.listbox.bind("<<ListboxSelect>>", lambda _e: self.selection_changed())
        ttk.Label(page, text="Health, energy and the two prompts share one layout, so the Bottom HUD group keeps the game's arrangement "
                             "when they start from the same values. Line up selected gives them the first element's values.",
                  foreground="#555", wraplength=820).pack(anchor="w")

    def build_attach(self, page):
        ttk.Label(page, text="Where should this element live?").pack(anchor="w")
        self.attach = tk.IntVar()
        for index, label in enumerate(ATTACH):
            ttk.Radiobutton(page, text=label, value=index, variable=self.attach,
                            command=self.pick_attach).pack(anchor="w", pady=1)
        row = ttk.Frame(page)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="Bone:").pack(side="left")
        self.bone_filter = tk.StringVar()
        entry = ttk.Entry(row, textvariable=self.bone_filter, width=16)
        entry.pack(side="left", padx=4)
        entry.bind("<KeyRelease>", lambda _e: self.fill_bones())
        ttk.Label(row, text="filter").pack(side="left")
        self.bone_list = tk.Listbox(page, height=9, exportselection=False)
        self.bone_list.pack(fill="x", pady=4)
        self.bone_list.bind("<<ListboxSelect>>", lambda _e: self.pick_bone())
        ttk.Button(page, text="Reload bone list", command=self.reload_bones).pack(anchor="w")
        self.fill_bones()
        ttk.Label(page, text="On a specific bone, the three sliders follow that bone's own axes, which vary by bone.", foreground="#555").pack(anchor="w")
        ttk.Label(page, text="Elements out of view stay hidden. While mounted, arms and hands still work, but a specific bone is hidden. A weapon element uses the hand instead. Head starts 30 cm in front of your eyes; Along brings it closer.",
                  foreground="#555", wraplength=820).pack(anchor="w", pady=(6, 0))

    def pick_attach(self):
        attach = self.attach.get()
        moved = attach == HEAD_ATTACH and self.current()["attach"] != HEAD_ATTACH
        self.set_value("attach", attach)
        if moved:
            for index in self.selected:
                self.values[ELEMENTS[index][0]].update(HEAD_START)
            self.show()

    def reload_bones(self):
        self.bones = read_bones(self.bones_path)
        self.fill_bones()
        if not self.bones:
            self.status.set("No bone list yet: turn on Native HUD on arm in the game, stand in gameplay, then reload.")

    def fill_bones(self):
        text = self.bone_filter.get().lower()
        self.shown_bones = [b for b in self.bones if text in b[1].lower()]
        self.bone_list.delete(0, "end")
        for _index, name in self.shown_bones:
            self.bone_list.insert("end", name)

    def pick_bone(self):
        if self.bone_list.curselection():
            self.attach.set(BONE_ATTACH)
            self.set_value("attach", BONE_ATTACH)
            self.set_value("bone", self.shown_bones[self.bone_list.curselection()[0]][0])

    def build_sliders(self, page, specs):
        for key, label, low, high, step in specs:
            row = ttk.Frame(page)
            row.pack(fill="x", pady=4)
            ttk.Label(row, text=label, width=38).pack(side="left")
            var = tk.DoubleVar()
            self.vars[key] = var
            ttk.Scale(row, from_=low, to=high, variable=var,
                      command=lambda _v, k=key, v=var, s=step: self.set_value(k, round(v.get() / s) * s)).pack(side="left", fill="x", expand=True)
            ttk.Spinbox(row, from_=low, to=high, increment=step, textvariable=var, width=8,
                        command=lambda k=key, v=var: self.set_value(k, v.get())).pack(side="left", padx=4)

    def go(self, delta):
        self.step = max(0, min(len(STEPS) - 1, self.step + delta))
        for index, page in enumerate(self.pages):
            if index == self.step:
                page.pack(fill="both", expand=True)
            else:
                page.pack_forget()
        self.title.set(STEPS[self.step])
        self.back.state(["disabled" if self.step == 0 else "!disabled"])
        self.next.state(["disabled" if self.step == len(STEPS) - 1 else "!disabled"])

    def select(self, index):
        self.index = index
        self.selected = [index]
        self.listbox.selection_clear(0, "end")
        self.listbox.selection_set(index)
        self.show()

    def selection_changed(self):
        chosen = list(self.listbox.curselection())
        if chosen:
            self.selected = chosen
            self.index = chosen[0]
            self.show()

    def select_group(self):
        keys = self.groups.get(self.group.get(), [])
        chosen = [i for i, e in enumerate(ELEMENTS) if e[0] in keys]
        if chosen:
            self.listbox.selection_clear(0, "end")
            for index in chosen:
                self.listbox.selection_set(index)
            self.selection_changed()

    def save_groups(self):
        self.group_box["values"] = sorted(self.groups)
        try:
            with open(self.groups_path, "w", encoding="utf-8") as stream:
                json.dump(self.groups, stream, indent=2)
        except OSError as error:
            self.status.set("Could not save groups: %s" % error)

    def save_group(self):
        name = self.group.get().strip()
        if not name:
            self.status.set("Type a name in the Group box first.")
            return
        self.groups[name] = [ELEMENTS[i][0] for i in self.selected]
        self.save_groups()
        self.status.set("Saved group %s." % name)

    def delete_group(self):
        if self.groups.pop(self.group.get(), None) is not None:
            self.group.set("")
            self.save_groups()

    def current(self):
        return self.values[ELEMENTS[self.index][0]]

    def show(self):
        self.loading = True
        values = self.current()
        self.attach.set(int(values["attach"]))
        self.two_sided.set(int(values["two_sided"]))
        for key, var in self.vars.items():
            var.set(values[key])
        self.loading = False

    def set_value(self, key, value):
        if self.loading:
            return
        value = float(value)
        old = self.current()[key]
        for index in self.selected:
            values = self.values[ELEMENTS[index][0]]
            if key in ADDITIVE:
                values[key] += value - old
            elif key == "scale" and old:
                values[key] = max(.05, values[key] * value / old)
            else:
                values[key] = value
        self.current()[key] = value
        self.schedule()

    def reset(self):
        for index in self.selected:
            self.values[ELEMENTS[index][0]] = defaults(ELEMENTS[index])
        self.show()
        self.schedule()

    def line_up(self):
        for index in self.selected:
            self.values[ELEMENTS[index][0]] = dict(self.current())
        self.schedule()
        self.status.set("Selected elements now use the values of %s." % ELEMENTS[self.index][1])

    def schedule(self):
        if self.pending:
            self.root.after_cancel(self.pending)
        self.pending = self.root.after(80, self.write)

    def write(self):
        self.pending = None
        try:
            temporary = self.path + ".tmp"
            with open(temporary, "w", encoding="utf-8", newline="\n") as stream:
                stream.write(render(self.values, self.frame.get()))
            os.replace(temporary, self.path)
            self.status.set("Saved " + self.path)
        except OSError as error:
            self.status.set("Could not save: %s" % error)
