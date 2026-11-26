"""Full-view effects: flat HUD effects (the red damage effect and the like) drawn across the view instead of the HUD window.

Writes <game>/scripts/AvatarVR_fullview_fx.txt on every change; the mod re-reads it about 4 times a second while
F10 -> UI -> Native HUD on arm and Damage effect fills your view are on. Defaults must match FxDefault in src/camera/ui_ownership.cpp.
The mod lists every full-size widget origin it has seen in AvatarVR_fullview_seen.txt. Effects are drawn on the scanner's
head-forward plane, one surface for both eyes.
"""
import io
import settings_store
import os
import tkinter as tk
from tkinter import ttk

FILE = "AvatarVR_fullview_fx.txt"
SEEN_FILE = "AvatarVR_fullview_seen.txt"
MAX_EFFECTS = 6  # FxMax in the mod
DEFAULT = dict(x=0, y=0, on=1, strength=3, size=1.02)
KNOWN = {(0, 0): "red damage effect", (280, 0): "aim dot, probably: leave off"}


def label(effect):
    origin = (int(effect["x"]), int(effect["y"]))
    return "%d, %d  %s%s" % (origin[0], origin[1], KNOWN.get(origin, "new"), "" if effect["on"] else "  (off)")


def read_effects(path):
    """The saved list, or the default when there is none. Fields from older builds are ignored."""
    found = {}
    try:
        with io.StringIO(settings_store.read(path)) as stream:
            for line in stream:
                name, _, value = line.strip().partition("=")
                effect, _, field = name.partition(".")
                if effect.startswith("effect") and effect[6:].isdigit() and field in DEFAULT and int(effect[6:]) < MAX_EFFECTS:
                    found.setdefault(int(effect[6:]), dict(DEFAULT))[field] = float(value)
    except (OSError, ValueError):
        pass
    return [found[index] for index in sorted(found)] or [dict(DEFAULT)]


def read_seen(path):
    seen = []
    try:
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                fields = line.strip().split(",")
                if len(fields) == 2:
                    seen.append((int(fields[0]), int(fields[1])))
    except (OSError, ValueError):
        pass
    return seen


def render(effects):
    return "".join("effect%d.%s=%.10g\n" % (index, field, effect[field]) for index, effect in enumerate(effects) for field in DEFAULT)


class FullViewTab:
    def __init__(self, parent, root, scripts):
        self.root, self.path = root, os.path.join(scripts, FILE)
        self.seen_path = os.path.join(scripts, "AvatarVR", "state", SEEN_FILE)
        self.effects = read_effects(self.path)
        self.pending = None
        self.loading = False
        self.status = tk.StringVar(value="Turn on F10 > UI > Native HUD on arm and Damage effect fills your view. Changes show within a moment.")
        ttk.Label(parent, text="Effects drawn across your view", font=("", 11, "bold")).pack(anchor="w")
        self.listbox = tk.Listbox(parent, height=MAX_EFFECTS, exportselection=False)
        self.listbox.pack(fill="x", pady=4)
        self.listbox.bind("<<ListboxSelect>>", lambda _e: self.show())
        self.on = tk.IntVar()
        ttk.Checkbutton(parent, text="On", variable=self.on, command=lambda: self.set_value("on", self.on.get())).pack(anchor="w")
        self.strength = tk.IntVar()
        self.size = tk.DoubleVar()
        self.slider(parent, "Strength (more also costs GPU time)", self.strength, 1, 6, 1, "strength")
        self.slider(parent, "Size of the plane, times your lenses' view (smaller: further into view; below 1 a hard edge can show)", self.size, .7, 2, .01, "size")
        ttk.Button(parent, text="Remove effect", command=self.remove).pack(anchor="w", pady=(4, 10))
        ttk.Label(parent, text="Full-size widgets the game has drawn", font=("", 11, "bold")).pack(anchor="w")
        ttk.Label(parent, text="When a new effect shows up boxed, add its line and check it fills your view. "
                               "If the aim dot doubles or other HUD art stretches, turn that line off.", foreground="#555", wraplength=820).pack(anchor="w")
        self.seen_list = tk.Listbox(parent, height=8, exportselection=False)
        self.seen_list.pack(fill="x", pady=4)
        row = ttk.Frame(parent)
        row.pack(fill="x")
        ttk.Button(row, text="Add selected", command=self.add_seen).pack(side="left")
        ttk.Label(parent, textvariable=self.status, foreground="#555", wraplength=820).pack(anchor="w", pady=(8, 0))
        self.seen = []
        self.fill()
        self.refresh_seen()

    def slider(self, parent, text, var, low, high, step, key):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text=text, width=72).pack(side="left")
        ttk.Scale(row, from_=low, to=high, variable=var,
                  command=lambda _v: self.set_value(key, round(var.get() / step) * step)).pack(side="left", fill="x", expand=True)
        ttk.Spinbox(row, from_=low, to=high, increment=step, textvariable=var, width=6,
                    command=lambda: self.set_value(key, var.get())).pack(side="left", padx=4)

    def fill(self, index=0):
        self.listbox.delete(0, "end")
        for effect in self.effects:
            self.listbox.insert("end", label(effect))
        if self.effects:
            self.listbox.selection_set(min(index, len(self.effects) - 1))
        self.show()

    def current(self):
        chosen = self.listbox.curselection()
        return self.effects[chosen[0]] if chosen else None

    def show(self):
        effect = self.current()
        if effect:
            self.loading = True
            self.on.set(int(effect["on"]))
            self.strength.set(int(effect["strength"]))
            self.size.set(effect["size"])
            self.loading = False

    def set_value(self, key, value):
        effect = self.current()
        if self.loading or not effect:
            return
        effect[key] = float(value)
        self.fill(self.listbox.curselection()[0])
        self.schedule()

    def remove(self):
        if self.current():
            del self.effects[self.listbox.curselection()[0]]
            self.fill()
            self.schedule()

    def refresh_seen(self):
        """Follows the mod's list; the selection stays on the same origin."""
        chosen = self.seen_list.curselection()
        kept = self.seen[chosen[0]] if chosen and chosen[0] < len(self.seen) else None
        self.seen = read_seen(self.seen_path)
        self.seen_list.delete(0, "end")
        for index, origin in enumerate(self.seen):
            used = "  IN LIST" if any((int(e["x"]), int(e["y"])) == origin for e in self.effects) else ""
            self.seen_list.insert("end", "%d, %d  %s%s" % (origin[0], origin[1], KNOWN.get(origin, "new"), used))
            if origin == kept:
                self.seen_list.selection_set(index)
        self.root.after(2000, self.refresh_seen)

    def add_seen(self):
        chosen = self.seen_list.curselection()
        if not chosen or chosen[0] >= len(self.seen):
            return
        x, y = self.seen[chosen[0]]
        if any(int(e["x"]) == x and int(e["y"]) == y for e in self.effects):
            self.status.set("That widget is already in the list.")
        elif len(self.effects) >= MAX_EFFECTS:
            self.status.set("The list is full: remove one first.")
        else:
            self.effects.append(dict(DEFAULT, x=x, y=y))
            self.fill(len(self.effects) - 1)
            self.schedule()

    def schedule(self):
        if self.pending:
            self.root.after_cancel(self.pending)
        self.pending = self.root.after(80, self.write)

    def write(self):
        self.pending = None
        try:
            settings_store.write(self.path, render(self.effects))
            self.status.set("Saved " + self.path)
        except OSError as error:
            self.status.set("Could not save: %s" % error)
