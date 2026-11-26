# Live mounted-arm tuning

After rebuilding through the usual run cycle, double-click `ik_tuner.bat`. Restart the tuner if an older window is open.
Changes are saved to `scripts/AvatarVR_ik_tune.txt` and picked up by the mod about four times a second.

The mounted page has three tabs:

- **Shared offsets**: your existing settings. These continue to provide the base for both arms.
- **Left arm adjustments**: extra corrections for the left arm only.
- **Right arm adjustments**: extra corrections for the right arm only.

All new adjustments start at zero. Zero means keep the current shared setup; it does not reset the arm to an untuned pose.
The tool does not write anything merely by opening its window. Existing settings are retained when you save.

For the left-arm twist, start with **Left arm adjustments â†’ Twist â†’ Forearm skin roll**. This adds roll to the skin's
twist helpers; the tracked hand's orientation is unchanged. **Upper-arm skin roll** does the same for upper-arm helpers.
Use small changes in either direction and compare the arm in hover and boost. Hand **Roll trim** changes hand alignment
instead. The arm's own twist-sharing trims can reduce or increase how much controller roll reaches its skin.

Each arm also has hand position/angle, shoulder, elbow, palm-depth, reach and clavicle trims. Angle trims apply after
the shared mirroring. Side/out trims are positive away from the body on the selected side. Up/forward are the same
directions on both arms. Reach and twist shares are clamped to the supported effective ranges after adding the trim.

**Reset left/right adjustments to zero** affects only that arm. **Reset all** resets shared settings too.
Presets include shared settings and both sets of adjustments. Scroll within each tab to reach all controls.

The file uses `tp.left.*` and `tp.right.*` for new trims, for example `tp.left.forearm_roll=-10`. These add to the existing
`tp.*` settings; absent new keys mean zero. Clearing both adjustment tabs restores the prior tuning behavior.

First-person/on-foot tuning remains separate. The tuner does not change skeletons, mesh geometry or accessory ownership.


## Finger gestures

Rebuild through your normal manual run cycle, then restart **ik_tuner.bat**.
Enable **Arm IK**, **Script writes**, and **Empty-hand gestures** in the VR menu.
The tuner does not deploy a DLL or start/stop the game.

Open **Finger gestures**, then select **Rig**, **Hand**, and **Combination**.
On-foot and mounted poses are independent because their native bone axes differ.
Each of the four fingers has **Base bend**, **Middle bend**, **Tip bend**,
**Spread / opposition**, and **Skin roll**, in degrees. The two lower fingers are
labelled Lower finger 1/2. Bend curls toward the palm; spread opens sideways.
Use skin roll sparingly to correct axial twisting. Adjust thumb opposition with
its spread and base bend. Joint lengths and wrist placement stay on the existing rig.

There are twelve input combinations plus **Relaxed idle**. Index touch has its
own poses, separate from index travel/press. Squeeze and trigger travel blend
continuously between the edited combinations; partial input is not a hard pose switch.
The page shows the current left/right input, resolved ownership, selected combination
and blend weight. Native ownership means the pose is intentionally suppressed.

Use **Preview selected pose** to edit without physically holding its combination.
Only the selected empty hand and matching rig can preview. Weapon/item/support/
harness ownership and cinematic/unknown-state vetoes still win. Leave the tuner open
while previewing. Clear Preview to test real touch/press input. Preview expires within
two seconds if the tuner closes or stops, and is never armed by loading a preset.
The button does not send controller actions or grab/release anything.

**Copy pose to other hand** mirrors the same anatomical settings through each hand's
own axes. **Copy pose to other rig** copies angle values only; it never copies FP
bone matrices into the mounted skeleton. Both buttons affect the selected combination.
**Reset this pose** leaves every other hand/rig/combination and all arm tuning intact.
**Reset all** deliberately resets arm and finger settings.

Finger keys share AvatarVR_ik_tune.txt with existing arm keys, for example:
gesture.tp.left.fist.index.bend1=65
gesture.fp.right.point.thumb.splay=60

Pose edits persist in the tune file and saved presets. Unknown numeric keys from
other tuning work are preserved. New settings are read about four times per second.
The runtime status file is AvatarVR_gesture_status.json; it is diagnostic only.
Turning Empty-hand gestures OFF restores the previous pose path. Default poses
and the new flicker correction still require headset acceptance.

## Thumb base, individual joints and webbing controls (2026-10-28)

The old shared Spread/Skin roll controls could not edit thumb-root placement or
the separate deformation bones around its base. Restart the tuner after your
usual rebuild/run cycle to get the added controls. Existing presets retain their
old values; every new adjustment starts at zero.

In **Thumb / base + joints 01 / 02 / 03**, start with:
- **Base pitch / Base oppose / Base roll**: rotate the whole thumb in the anatomical
  palm frame, including its base.
- **Base normal / Base sideways / Base forward**: move the thumb root in millimetres.
  Normal is perpendicular to the palm, sideways points toward the index, and forward
  points from wrist toward knuckles. Small changes are easier to judge.
- **01 / 02 / 03 bend, sideways, skin roll**: independent flexion, side angle and
  axial roll. The old **All spread / All skin roll** values still work as shared
  adjustments. The other three fingers expose the same added root/joint controls.

If the thumb skeleton looks right but its skin/webbing remains pinched, use
**Thumb webbing / deformation bones**. Start with the four **thumb_01_expand01**
through **thumb_01_expand04** entries for the base area. The dropdown also exposes
the recorded thumb02/03 expansion and driver bones. The selected hand/rig/combination
still applies. Each named bone has rotation, offset in mm, and scale trim in percent.
Zero means keep its reference relation. These are advanced mesh-local bone axes;
their orientation can differ between hands and rigs. Copying these numeric trims
to the other hand/rig may need further adjustment. Absent named helpers have no
effect in a mesh. Helper selection preserves edits on the other helper bones.

Preview the same combination while editing. First correct the whole-thumb base,
then the individual joints, then any residual skin deformation. Root edits keep
phalanx lengths; helper edits do not move the core finger joints or wrist. The
runtime now uses the named 01/02/03 helper group rather than whichever thumb joint
happens to be nearest, and native helper scale no longer leaks through at full
gesture weight. This grouping and the final appearance still need visual acceptance.

The new keys include thumb.root_pitch, thumb.root_normal, thumb.spread1,
thumb.roll1 and thumb_01_expand01.offset_x / scale_x, under the same
gesture.<fp|tp>.<left|right>.<combination>. prefix. New values blend through the
existing 60 ms policy and save/copy/reset with the selected pose.

## Capture native pose

On the Finger gestures page pick the hand and combination, then press
**Capture native pose** during gameplay (Empty-hand gestures + Arm IK + Script
writes on, hand in view). The DLL fits the game's current finger animation to
the sliders and writes `AvatarVR_gesture_capture.json`; the tuner loads it into
that combination (switching Rig to on foot/mounted to match) and shows the fit
error. Edit from there and save as usual. Request keys `gesture.capture_*` expire
after 5 s and are never written into presets.

## Copy / paste a bone

Each finger box and the thumb webbing box has **Copy bone** / **Paste bone**.
Copy takes every slider of that bone (finger: base, joints, spread, roll; helper:
rotation, offset, scale) from the pose on screen. Switch hand, rig or
combination (or pick another finger / helper) and press Paste bone on the target.
Fingers paste onto fingers and helpers onto helpers; the clipboard line shows the
source. Values are anatomical, so left and right need no mirroring. The source
pose is not changed.

## Copy / paste a whole pose, and mirror hands

**Copy pose** / **Paste pose** (top of the Finger gestures page) copy every finger
and thumb-helper slider of the selected combination; switch hand, rig or
combination and paste. **Mirror edits to the other hand** makes every edit, paste,
capture or reset on one hand land on the same rig + combination of the other hand
(the hand you are editing wins). Values are anatomical, so the mirror is an exact
copy. The toggle is remembered in `AvatarVR_tuner_prefs.json`.


## HUD on arm

Turn on **F10 > UI > Native HUD on arm**, then open the **HUD on arm** tab. Pick an element, choose where it attaches (arm, hand, weapon, or a specific bone), then move, turn and size it. Changes save to `scripts/AvatarVR_hud_widgets.txt` and show in the headset within a moment. The bone list appears after the mod has run once in gameplay; use **Reload bone list**.
Select several elements with Ctrl+click or Shift+click to move them together, or save the selection as a group. **Bottom HUD** is built in; **Line up selected** gives the health bar, energy and prompts the same values, which restores the game's arrangement.


## Tuning inside VR (Tuner tab)

The VR mod menu has a **Tuner** tab with this tuner's pages: **Fingers** (poses, thumb helpers, capture, mirror,
copy/paste), **Mounted arms**, **HUD on arm** and **Full-view effects**. Pick the page in the first row; values change
with left/right and apply live, in the same files as this tuner. Close this desktop tuner while tuning in VR: it
rewrites the files from its own state and would undo the menu's edits. HUD group save/delete and presets are still
only here.
