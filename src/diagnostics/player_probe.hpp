#pragma once

// Read-only probe of the local player's body position and game camera transform (research/CAMERA_FINDINGS.md
// "Local player and camera roots from script nodes", [STATIC] on build 98588DA2...). No hooks, no writes: SEH-guarded
// reads of game memory on the Present thread. Off unless scripts\AvatarVR_player.txt says "on". Logs to
// PLAYER_PROBE.log (about 10 lines a second while on).
namespace player_probe {

// Called once per IDXGISwapChain::Present.
void on_present();

}  // namespace player_probe
