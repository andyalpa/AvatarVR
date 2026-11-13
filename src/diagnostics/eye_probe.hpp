#pragma once

// Read-only probe of Snowdrop's built-in eye stepper (research/ADDRESS_REGISTRY.md LEAD-01).
// Installs one MinHook detour on afop.exe+0x5A7530 only if the 0x36 bytes there match the
// build 98588DA2... signature exactly. The detour calls the original unchanged and only counts.
namespace eye_probe {

// Idempotent. Call after MH_Initialize (from the D3D12 device path). Logs to EYE_PROBE.log.
void install();
// Called once per IDXGISwapChain::Present (lock-free counters only; logs at most once a second).
void on_present();
// Total calls of the engine eye stepper (afop.exe+0x5A7530) seen so far (for the native-stereo heartbeat).
unsigned long long stepper_calls();

}  // namespace eye_probe
