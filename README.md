# AvatarVR

A native stereo VR mod for **Avatar: Frontiers of Pandora** on PC (Snowdrop engine).

- Both eyes rendered every frame (true stereo, not alternate-eye)
- 6DoF head tracking with decoupled head pitch
- OpenXR motion controllers, tracked arms, roomscale body follow
- VR mod menu, HUD placed in 3D, comfort options

This repository holds the mod's source code, tests and build scripts. It does **not** contain the game or any game files; you need your own copy. Current status: [docs/ROADMAP.md](docs/ROADMAP.md).

---

## 1. Installing the mod (players and testers)

### Requirements

| | |
|---|---|
| Game | Avatar: Frontiers of Pandora, PC, Ubisoft Connect, version **1.0.2.2**. Other versions are not supported: the mod hooks addresses specific to this build. |
| OS / GPU | Windows 10/11 x64, DirectX 12 GPU. Stereo doubles the rendering cost; development uses an RTX 4090. |
| Headset | Any headset with an **OpenXR** runtime: SteamVR, Meta/Oculus, Virtual Desktop, … Tested on Quest 3 over Virtual Desktop. |

### Steps

1. **Mod loader.** Download the Universal Snowdrop Modloader [`version.zip` (USM v1.0.0)](https://github.com/patrickmollohan/Universal-Snowdrop-Modloader-Old/releases/download/USM-v1.0.0/version.zip). Extract `version.dll` and `version.ini` into the game folder, next to `afop.exe`. Check that `version.ini` has `EnableScripts=true`.
2. **Mod.** Create a `scripts` folder in the game folder if there isn't one, then put `AvatarVR.dll` in it:
   ```
   <game folder>\
     afop.exe
     version.dll
     version.ini
     scripts\AvatarVR.dll
   ```
   Get `AvatarVR.dll` from a release, or build it yourself (section 2).
3. **Game settings** (see the table below): windowed display mode, DLSS on.
4. Start your OpenXR runtime (SteamVR / Virtual Desktop / Meta link), then launch the game from Ubisoft Connect.
5. Press **F10** for the mod menu, then **Mod → Load working preset**.

### Keys

| Key | Action |
|---|---|
| F10 | Open / close the mod menu (arrow keys, Enter, PgUp/PgDn to navigate; controllers work too) |
| F9 | Save a screenshot of both eyes to `scripts\captures\` |
| F7 | Reload Lua scripts (developers, see [docs/SCRIPTING.md](docs/SCRIPTING.md)) |

### Files the mod writes

All in `<game folder>\scripts\`:

- `AvatarVR_settings.ini`: every mod setting. Delete it to reset to defaults.
- `*.log`: diagnostic logs. Attach them to bug reports.

### Recommended game settings

| Setting | Value | Why |
|---|---|---|
| Display mode | Windowed | The mod sizes the window to the headset's eye shape; render resolution follows the window. |
| Upscaler | DLSS Quality or Balanced | Two views per frame; DLSS gives back frame rate. |
| Object detail | Maximum | Each eye has a much wider view than a monitor, so lower settings make objects pop in and out. |
| Vegetation, scatter density, streaming distance | As high as frame rate allows | Less pop-in. |
| `stereo3d` in `graphic settings.cfg` | `0` | The mod renders the second eye itself. |
| Dynamic resolution | Off, resolution scale 100 % | Otherwise it stays at its lowest scale in VR. |
| Film grain, chromatic aberration, motion blur, depth of field | Off | They blur and smear in a headset. |

Reference: RTX 4090, Quest 3 via Virtual Desktop at 72 Hz, 1992×2160 per eye: about 60 fps with DLSS, about 40 fps without.

### Uninstall

Delete `version.dll`, `version.ini` and `scripts\AvatarVR.dll` (and `scripts\AvatarVR_settings.ini` and `scripts\*.log` if you like). The game's own files are never modified.

### Reporting a problem

Open an issue with: what you were doing (on foot, weapon drawn, riding, menu, …), headset and OpenXR runtime, GPU, and the `scripts\*.log` files from that session.

---

## 2. Building from source (developers)

### Tools

- **Visual Studio 2022** with the *Desktop development with C++* workload
- **CMake 3.20+** (the copy bundled with Visual Studio is fine)
- **Git**
- Optional: **Python 3.11+** for the log tools and the arm tuner; **Lua 5.4** for the Lua tests

All other dependencies are in `third_party/`: MinHook, Dear ImGui, Lua 5.4 and the OpenXR SDK.

### Build

```powershell
git clone https://github.com/andyalpa/AvatarVR.git
cd AvatarVR
cmake -S . -B build
cmake --build build --config Release --parallel
```

Output in `build\bin\`: `AvatarVR.dll` (the mod), `load_proof_host.exe` (loads the DLL outside the game as a smoke test) and the `*_test.exe` unit tests.

### Build, install and launch in one step

Point the scripts at your game folder once per terminal (or set it as a user environment variable):

```powershell
$env:AFOP_GAME_ROOT = 'D:\Games\Avatar Frontiers of Pandora'
tools\run_cycle.bat
```

`run_cycle` builds the mod, runs the load test, copies `AvatarVR.dll` into `<game>\scripts\`, moves the previous logs into `<game>\scripts\runs\<time>\`, launches the game and waits for you to close it. Progress: `build\cycle_status.txt`. Full build log: `build\cycle_build.log`. It never touches `afop.exe`, `version.dll` or `version.ini`. The mod loader must already be installed (section 1, step 1).

`tools\pull_and_cycle.bat` does a `git pull` first, then the same cycle.

### Tests

| What | Command |
|---|---|
| All C++ unit tests | see the loop below |
| DLL load smoke test | `build\bin\load_proof_host.exe` (exit code 0 = pass) |
| Log analyzer tests | `python -I -m unittest discover -s tools/perf -p "test_*.py"` |
| Lua math tests | `lua lua/tests/vrgameplay_test.lua` (and the other files in `lua/tests/`) |

```powershell
Get-ChildItem build\bin\*_test.exe | Where-Object Name -ne 'mv_repair_test.exe' |
  ForEach-Object { & $_.FullName > $null; '{0,-32} exit {1}' -f $_.Name, $LASTEXITCODE }
```

Every test should exit with 0. `mv_repair_test.exe` is an offline tool for captured frames, not a pass/fail test.

Unit tests don't replace a headset test. Any change to rendering, camera, input or arms has to be checked in game. See [CONTRIBUTING.md](CONTRIBUTING.md#testing-in-the-headset).

### Developer tools

| Tool | Use |
|---|---|
| `python tools\perf\runs_report.py <game>\scripts\runs` | Frame rate and frame time per play session |
| `python tools\perf\analyze_logs.py` | Summarize the renderer / CPU logs of a session |
| `tools\ik_tuner\ik_tuner.bat` | Live arm-IK tuning window (writes `scripts\AvatarVR_ik_tune.txt`; the mod reloads it while running) |
| Lua scripting, F7 to reload | Try gameplay ideas in the running game: [docs/SCRIPTING.md](docs/SCRIPTING.md) |

### Source layout

| Path | Contents |
|---|---|
| `src/dllmain.cpp` | Entry point: starts hooks, menu, VR runtime |
| `src/camera/` | Second-eye rendering, per-eye camera and projection, UI placement, mounted camera |
| `src/dx12/` | Direct3D 12 / DXGI hooks (Present, queues, swapchain) |
| `src/vr/` | OpenXR session and frame submission, controller input, virtual gamepad, arms/IK, weapons, hand gestures |
| `src/ui/` | Mod menu (ImGui) and HUD panels |
| `src/game/` | Game-side hooks and state reads |
| `src/sdk/` | Engine names and addresses for game version 1.0.2.2 |
| `src/diagnostics/` | Probes and logs, all off unless enabled in the menu or settings |
| `src/script/` | Embedded Lua host |
| `src/settings_file.hpp` | `AvatarVR_settings.ini` reader/writer |
| `tools/tests/` | C++ unit tests (each is registered in `CMakeLists.txt`) |
| `tools/perf/` | Performance logging and analysis |
| `tools/ik_tuner/` | Arm tuning app |
| `lua/` | `lib/` shared Lua modules, `bench/` ready-made test scripts, `tests/` offline Lua tests |
| `third_party/` | Vendored libraries, each under its own license |

---

## Legal

Unofficial fan project, not affiliated with or endorsed by Ubisoft, Massive Entertainment, Lightstorm or Disney. The mod doesn't modify game files and doesn't bypass DRM, anti-cheat or online services. Do not commit game files, game executables or extracted game data to this repository.
