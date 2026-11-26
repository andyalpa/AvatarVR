# Contributing

Thanks for helping. The mod hooks a closed engine, so wrong guesses usually mean crashes. The rules below exist to catch those early.

## Getting started

1. Build the mod and run the tests (see [Building from source](README.md#building-from-source)).
2. Check [docs/ROADMAP.md](docs/ROADMAP.md) for open issues and features. Open an issue before starting anything large, so work isn't duplicated.

## Branches and pull requests

- Branch from `master`: `feature/<short-name>` or `fix/<short-name>`.
- One feature or fix per pull request.
- New behavior goes behind a setting (menu row or `AvatarVR_settings.ini` key). With the setting off, the game must behave as before.
- Describe in the PR:
  - **Goal**: what changes for the player
  - **Evidence**: why you believe the engine works the way your change assumes (log lines, capture, disassembly)
  - **Testing**: unit tests run, and what you checked in the headset
  - **Rollback**: which setting turns it off
  - **Open questions**: anything still unverified
- Add or update a unit test in `tools/tests/` for any math, layout, parsing or settings change, and register it in `CMakeLists.txt`.

## Working with engine addresses

- Addresses are for game version **1.0.2.2** only. Every hook checks the bytes it expects before patching (a guard); keep that pattern.
- Start read-only: log or probe a value and confirm it in game before writing to game memory.
- Mark how sure you are when a comment states an engine fact: `[LIVE]` (seen in the running game), `[STATIC]` (seen in the binary), `[HEADSET]` (checked in VR), `[INFERENCE]` (reasoned, unverified). Unknowns are written as `OPEN` with the test that would settle them.
- Don't analyze or patch the game's protected/obfuscated code. No DRM, anti-cheat or online-service bypasses.

## Testing in the headset

1. Set `AFOP_GAME_ROOT` and run `tools\run_cycle.bat`.
2. Play the scenarios your change touches. Write down the time of anything odd.
3. Close the game. Logs: `<game>\scripts\*.log`. Earlier sessions: `<game>\scripts\runs\`.
4. In the PR, list what you tested (on foot, weapon drawn, bow, riding, menus, loading screens, …), with which headset, runtime and GPU, and paste the relevant log lines.
5. Check the flat-screen game with the mod's stereo off as well.

`python tools\perf\runs_report.py <game>\scripts\runs` compares frame rate across sessions.

## Code style

- C++20, MSVC. The `AvatarVR` target builds at `/W4`; don't add warnings.
- Match the file you're editing: 2-space indent, `snake_case`, `g_` prefix for globals.
- Comments explain *why*, and where a number came from.
- Don't edit `third_party/`.

## Never commit

Game files or executables, extracted game data or disassembly dumps, build output, logs, captures, or your personal `AvatarVR_settings.ini`. Run `git status` before every commit.
