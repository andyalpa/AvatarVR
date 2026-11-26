# Development status

AvatarVR targets the Windows x64 PC build of Avatar: Frontiers of Pandora 1.0.2.2. The main path renders both eye views in the same game frame. The mod also provides OpenXR tracking and controls, tracked arms, roomscale movement, and a headset menu. The built-in frame generator adds extra frames between rendered stereo pairs. For supported controls, available features, and planned work, see the [player guide](../README.md).

The build, tests, and DLL load check run without the game. Use the headset for these remaining checks before preparing a playtest release.

- **Rendering** — Look for right-eye motion artifacts and uneven light or shadows. Check the scanner while looking toward the horizon, pause and resume, and fast travel to the cave.
- **Tracking and input** — Compare equipped, bare-hand, and mounted arm tracking. Check near-object occlusion, menu pointing, recentering, and movement after transitions. A native ikran harness grip still needs a verified saddle attachment point. The mount capture compares rider palms with the saddle draw for this work.
- **Weapons** — Check automatic recoil across a full burst. Compare bow draw and aim at partial and full pull. Run every manual reload stage, then equip another weapon and repeat.
- **Performance** — Record game CPU and GPU frame time, headset frame pacing, and memory use at the same eye resolution in matched indoor, outdoor, and mounted runs.

Start with `src/dllmain.cpp` for initialization, `src/camera/native_stereo.cpp` for view creation and timing, `src/vr/xr_runtime.cpp` for OpenXR presentation, and `src/ui/vr_menu.cpp` for controls and tuning. Arm and weapon skinning currently lives in `src/diagnostics/skin_palette_probe.cpp`.

Build from the [README](../README.md). The source export includes a manifest with each file’s hash. The mod ZIP installs `scripts/AvatarVR.dll`. Keep a backup of the installed DLL and `scripts/AvatarVR_settings.ini` when comparing builds.
