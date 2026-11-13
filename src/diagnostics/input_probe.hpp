#pragma once

// Read-only probe of the game's gamepad backends (research/INPUT_OWNERSHIP.md "Gamepad backends").
// One MinHook detour on the device enumeration function afop.exe+0x32B62B0, installed only if its bytes match the
// build 98588DA2... guard; the detour records the device manager pointer and calls the original unchanged.
// Everything else is SEH-guarded reads of game globals. Logs to INPUT_PROBE.log.
namespace input_probe {

// Idempotent. Call after MH_Initialize, as early as possible (enumeration runs at start-up and on pad hot-plug).
void install();
// Called once per IDXGISwapChain::Present; logs at most once a second (on change, plus every 10 s).
void on_present();

}  // namespace input_probe
