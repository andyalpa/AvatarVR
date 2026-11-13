# In-game Lua scripting

Gameplay features (raycast aim, two-hand aim, camera adjustments, arm IK) and stereo debugging are first tried as Lua scripts, which reload in the running game. The C++ code stays the shipping path:

1. Try the idea in Lua. Edit a script, then press F7 in game.
2. Repeat until the result looks right in game.
3. Port the script to C++ (see [Moving a script into C++](#moving-a-script-into-c)).
4. Delete the script or switch it off.

Lua is the test bench, not the product.

Host: [src/script/lua_host.cpp](../src/script/lua_host.cpp). Game access: [src/game/player.cpp](../src/game/player.cpp). Lua 5.4.7 is vendored in `third_party/lua` (MIT).

## Where scripts live

| Path | Content |
|---|---|
| `lua/autorun/*.lua` (this repo) | Scripts. They run in name order at load and on each reload. Empty in normal play since 2026-10-30, so no script runs every frame |
| `lua/bench/*.lua` | Finished test scripts, not loaded. Copy one into `lua/autorun` and press F7 to use it again |
| `lua/lib/*.lua` | Modules for `require`, e.g. `avrmath` |
| `<game>\scripts\avr\root.txt` | One line naming the script root. Here it names this repo's `lua` folder, so live edits stay versioned. Without the file the root is `scripts\avr` |
| `<game>\scripts\avr\tunables.txt` | Menu values of script tunables, kept across reloads and launches |
| `<game>\scripts\SCRIPTS.log` | Load messages, `avr.log` output, errors with traceback (`SCRIPTS.prev.log` = last run) |

## Workflow

1. Start the game as usual. The scripts load at the first frame.
2. Edit a file under `lua/autorun`, save it, and press **F7**, or use menu (F10) → **Scripts** → **Reload scripts**. Nothing else restarts: hooks, stereo and the game stay up.
3. A script with an error loads nothing. A callback that errors is switched off until the next reload. The error appears in the Scripts tab and in `SCRIPTS.log`, and the game keeps running.
4. Script tunables (`avr.tunable`, `avr.toggle`) appear as rows in the Scripts tab. Change them in game, including from the headset menu.
5. **Writes are off at every launch.** `avr.set_look`, `avr.set_bone` and `avr.write_*` do nothing until menu → Scripts → **Script writes** is on.
   - Scripts re-apply writes every frame. After a reload or an error, the game's own values come back on the next frame.
6. Debug drawing (`avr.draw.*`) shows on the monitor over the game image, with the menu open or closed. It does not go into the headset.

## Events and threads

| Event | Thread | When | Use for |
|---|---|---|---|
| `frame` | Present | once per presented frame | reading, logic, debug drawing, registering tunables |
| `camera_pre` | game | before the camera manager update `0x144271330` (`research/CAMERA_FINDINGS.md`) | look writes: the camera mode reads the look angles inside that update |
| `camera_post` | game | after it | camera or bone adjustments after the game's camera is built |
| `unload` | Present | before a reload | restoring anything a script changed outside the per-frame writes |

- One mutex guards the Lua state.
- The game thread only tries the lock. When a `frame` callback is still running, that frame's `camera_*` event is skipped instead of waited for, so keep callbacks short.
- `camera_pre` / `camera_post` fire only for the local player's camera holder.

## API (`avr`)

Vectors are `{x=, y=, z=}`. Positions are world, large-world metres, `+y` up. Camera rows: `right`, `up`, `fwd`, `pos`. Look convention: `yaw = atan2(fwd.x, fwd.z)`, pitch positive up, both in radians.

| Call | Returns / does |
|---|---|
| `avr.on(event, fn)` | register a callback |
| `avr.log(...)` | line in `SCRIPTS.log` |
| `avr.time()`, `avr.frame()` | seconds since load, Present count |
| `avr.player()` | `nil` outside gameplay, else `{pos, right, up, fwd, body, yaw, pitch, look_ok}` |
| `avr.mount()` | `{mounted, phase, since, camera_mode, movement_state, mount_type, partner, transitions}`; phase `none` / `mounting` / `riding` / `dismounting`, `since` = seconds in this phase (FEATURE-06) |
| `avr.pointers()` | raw object addresses (`handle, entity, body, holder, camera, look, pose`) for exploration |
| `avr.bone_count()`, `avr.bone(i)`, `avr.bones()` | first-person pose: count; one bone `{pos, right, up, fwd}`; all positions `{[0]=…}`. Bone map: `research/WEAPONS_AND_HANDS.md` |
| `avr.set_look(yaw, pitch)` | write the look angles (gated) |
| `avr.set_bone(i, {pos=, right=, up=, fwd=})` | write one bone's world matrix; omitted parts are kept (gated). Whether the arm mesh follows is `OPEN` |
| `avr.xr.views()` | `nil` without a headset session, else `{head, eyes, aim, grip}` poses `{pos, rot={x,y,z,w}, valid}` in OpenXR LOCAL space |
| `avr.stereo.aim_ray(hand)` | controller ray on the game camera basis (world): `org, dir`, or `nil` (needs head mode) |
| `avr.stereo.running()` | the second view renders |
| `avr.tunable(name, default, lo, hi, step)`, `avr.toggle(name, default)`, `avr.value(name)` | menu rows (at most 12); `value` reads without creating, safe on the game thread |
| `avr.draw.line(a, b, rgba, width)`, `.point(p, size, rgba)`, `.text(p, s, rgba)`, `.screen_text(x, y, s, rgba)` | debug drawing; `rgba` = `0xRRGGBBAA`, screen x/y as fractions |
| `avr.read_f32/u32/i32/u64/ptr(addr)`, `avr.write_f32/u32(addr, v)`, `avr.base()` | guarded memory access (writes gated) |
| `avr.writes()` | state of the menu's write gate |

`lua/lib/avrmath.lua` provides `v`, `add`, `sub`, `scale`, `dot`, `cross`, `norm`, `dist`, `lerp`, `yaw_pitch(dir)`, `dir(yaw, pitch)`, `angle_deg`, `qrot(q, v)` and `rgba`.

## Starter scripts (in `lua/bench`)

| Script | Shows |
|---|---|
| `00_player_info.lua` | player, camera and look values; markers 3 m ahead and at the feet (projection check) |
| `10_aim_debug.lua` | controller rays (headset) and a flat stand-in from right hand bone 56 along the camera forward |
| `20_bones_debug.lua` | arm chains 16–20 and 53–57, prop bone 89, optional bone indices |
| `90_look_write_test.lua` | PORT-06 write test: "look: hold" (flat) or "look: follow right controller" (headset). Needs Script writes on |

## Moving a script into C++

### Gameplay work started 2026-10-13

Validation checkpoint: Release DLL built in `build/player_probe/bin` (2,184,704
bytes), not deployed over `scripts/AvatarVR.dll` (2,181,632 bytes). Gameplay
math, controller-aim ownership, arm IK constraint/write-gate, and collision
record initialization tests pass. These are offline tests, not headset or
retail physics proofs. The latest inspected SCRIPTS.log still lists only the
four original scripts; the collision-owner probe has not run. No game process
was present at that check. Resume collision discovery after loading gameplay
and F7 on the existing Lua bench, then inspecting the owner-probe output.
The new grip/view bridge requires the prepared DLL before aim/IK live tests.
The broader goal is incomplete pending these live tests and collision-aware
roomscale movement; do not mark the offline scaffold as playable gameplay.

`lua/lib/vrgameplay.lua` contains tested two-hand direction, convergence target,
analytic elbow IK and physical roomscale offset math. Run its tests from the repo
root with a Lua 5.4 interpreter: `lua lua/tests/vrgameplay_test.lua`.

`30_gameplay_preview.lua` is disabled by default (`enabled = false`). Enabling it
and pressing F7 draws a range ray and a right-arm IK solution without writes.
The yellow endpoint is explicitly a range fallback, not a physics collision hit.
Roomscale output is a displacement preview; existing stereo already applies HMD
translation, so adding this offset again would double physical movement.

The native bridge adds `avr.stereo.grip_transform(hand)` returning world
`{right, up, fwd, pos}` on the exact eye recentre/camera basis. Aim and grip
samples older than 250 ms are rejected. This bridge requires the new DLL;
the script tolerates older DLLs by omitting its grip/IK preview.

Current evidence: controller-directed shots were confirmed by the author;
camera-filter raycast hits were verified live; both 90- and 192-bone skeleton
names and parent indices were recovered from the running game. The maps in
`lua/lib/vr_skeletons.lua` contain those named parent relationships. Neither
camera-post pulse test produced visible hand movement despite successful
immediate readback. The current 20-second pulse follow-up measures persistence
at Present and the next camera events; its logs do not prove mesh consumption.

Remaining engine work: the final animation/skinning dev notes, controller wrist
orientation and weapon calibration, collision-aware roomscale body movement,
weapon-specific collision filtering, and independent body turning beyond the
experimental captured-yaw view. HMD translation already moves the stereo eyes;
the missing roomscale work is body/collision integration, not another eye offset.
Math tests and a successful build do not prove headset alignment or skinning.

`40_controller_aim.lua` now offers an experimental `aim: independent view`
toggle plus `aim: two hands`. Both require Script writes and the new native
`avr.stereo.body_yaw(yaw)` bridge. Turn the old `look: hold` and `look: follow
right controller` tests off first. The script holds view yaw fixed at activation
while game look aims toward a 20 m range target, using grip-to-grip direction
when requested. This is view-basis separation, not a new player-body movement
or collision implementation. Reload or disable to release it; the native
override expires after 250 ms without renewal. Headset behaviour is unverified.
The fixed heading currently has no stick-turn integration; disable/re-enable
to capture a different heading. Existing stereo HMD translation remains active.

`50_arm_ik.lua` is an opt-in camera-post write experiment (`IK: controller arms`
off by default, plus Script writes). It solves both inferred arm chains toward world grip positions,
rotates the segment matrices, and transforms wrist/finger and twist bones from
one captured source pose. It deliberately omits weapon bone 89 and camera bone
0. Controller roll, wrist attachment calibration, actual skeleton parent-table
recovery, and skinning-stage validation remain OPEN. It currently relies on the
game regenerating its animation pose to remove a write; that restoration must
be verified in game, rather than assumed. Keep it disabled in normal play.
This is not yet a shipping IK implementation.

`55_right_grip_validation.lua` is a20s attachment-path test, armed on reload
but requiring Script writes, tracked right grip and a90bone gameplay pose.
It translates candidate right-hand/finger/helpers and prop89 together toward
the grip, clamped to0.15m. It changes no orientations, camera, left hand or aim
angles. It refuses concurrent IK/two-hand mode. A saved matrix is restored
only if the same pose still contains that test-owned matrix, including before
the next write to prevent accumulation. Actual visible hand/weapon consumption
and restoration remain user-test questions. The test auto-stops; disable its
local enabled flag after validation. It is not a calibrated weapon attachment.

Current attachment-path test mode is `pulse`: baseline0–3s,25cm up3–6s,
baseline6–9s,25cm down9–12s, then baseline until the20s stop. It logs immediate
readback error. Hold head and controllers still and do not fire; visual motion
is unconfirmed by the previous grip-follow test. Follow mode remains available
in the script for later calibration once mesh consumption is established.

`avr.raycast_camera(start, end)` is an experimental camera-filter query. It
requires Script writes and a camera_pre/post callback; Present calls return
nil plus an explanation. Result: `{executed, hit, pos, normal, distance}`;
position/normal/distance are present only on a hit. `executed=false` means
rejected/unavailable, not a miss. Segments are bounded to100m and finite inputs;
world getter/interface identity, query and cleanup prefixes and current engine
TLS readiness are checked. The existing filter at afop+8E07310 is reused
without mutation. It is NOT yet a weapon/self-ignore filter. A caught query or
cleanup fault disables subsequent casts for the process; thread readiness does
not by itself prove all engine lock requirements. Native runtime validation is
pending, so no production script invokes this by default.

`60_raycast_validation.lua` is disabled (`enabled=false`), calls only at10Hz on
the camera thread and displays query availability/hit data. Its debug drawing
is monitor-only. Validate on a flat view first: world-space overlay projection
currently uses the retail camera, so independent-view/headset display alignment
must not be judged solely by this marker. No query hit has been live-validated.

Every `avr` call is a thin wrapper over a C++ function. A finished script ports line for line:

| Lua | C++ |
|---|---|
| `avr.player()` | `game::read()` |
| `avr.pointers()` | `game::pointers()` |
| `avr.mount()` | `mount_state::get()` |
| `avr.bone(i)` / `avr.set_bone(i, …)` | `game::bone_world()` / `game::set_bone_world()` |
| `avr.set_look(y, p)` | `game::set_look()` |
| `avr.stereo.aim_ray(h)` | `native_stereo::aim_ray()` |
| `avr.xr.views()` | `xr_runtime::latest_views()` |
| `camera_pre` / `camera_post` | the camera manager detour in `lua_host.cpp`. A shipping feature gets its own call there, or its own hook module, not the script dispatcher |
| `avr.tunable` | a field in the mod menu (`src/ui/vr_menu.cpp`) and its control file |

When porting, record the fact the script proved in the matching research file with its evidence grade, and move any address it used into `research/ADDRESS_REGISTRY.md` with a guard.

## 2026-10-14 visible first-person arm test

The earlier CPU-pose/attachment tests above did not prove visible hand following.
The current demonstrated seam is a separate211-matrix player first-person body
mesh at the native palette dev notes. Exact named entries: left upperArm/lowerArm/
hand14/15/16; right113/114/115. Parent links and finger/weapon descendants remain
unresolved; do not apply these indices to a90/192 or other-count pose.

The native observer uses DLL-adjacent `AvatarVR_skinpalette.txt` with `on`/`off`:
20seconds, at most64distinct sources, bounded name scans, read-only. It logs to
`SKIN_PALETTE_PROBE.log`. An explicit off/on transition rearms it. Loading a new
native implementation requires the normal run cycle; control-file rearming does
not require a restart or Lua reload.

`AvatarVR_skinpulse.txt on` additionally requires valid gameplay and the menu's
Script writes toggle. It runs20seconds, right-arm Y+0.25 at3..6seconds and-0.25
at9..12seconds, baseline otherwise. Only a callback-local copy of the validated
male first-person211 source is altered; original animation arrays are untouched.
Other matrices/root/remap are preserved. Turning Script writes off or the control
off stops substitutions; timeout does not automatically rearm. Watch the right
arm/wrist while testing. Application counters establish scope execution, not
visible movement. This pulse received explicit user visual acceptance on2026-10-14;
controller following and complete arm IK are not yet implemented at this seam.
