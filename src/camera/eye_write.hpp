#pragma once

#include <cstddef>

// RE-08: one reversible per-eye camera write on the 1536-byte camera block (field map in
// research/CAMERA_FINDINGS.md). Default OFF. Controlled by scripts\AvatarVR_eye.txt:
//   off                 -> pass-through (stock)
//   zero                -> full read/guard/write path with a 0 offset (bytes written back unchanged)
//   offset <metres>     -> shift the eye along the camera's right axis by <metres> (view space)
//   alt <half_ipd>      -> alternate eyes per Present: even = left (-h), odd = right (+h)
namespace eye_write {

// Remember where the control file lives (directory of AvatarVR.dll). Call once at load.
void install();
// Re-read the control file and log counters. Call at most once a second (Present thread).
void poll();
// Called from the CreateConstantBufferView hook with the live CPU pointer of the bound range.
void apply(void* block, std::size_t size);
// Called once per IDXGISwapChain::Present, before the original Present (eye parity for alt mode).
void on_present();
// Alt mode only (0 otherwise): -1 left / +1 right. presented_eye() is the eye of the most recent camera
// write; parity_eye() is the eye the present counter selects right now. Call before on_present().
int presented_eye();
int parity_eye();

}  // namespace eye_write
