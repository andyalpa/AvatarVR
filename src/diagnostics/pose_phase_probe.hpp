#pragma once
namespace pose_phase_probe {
enum class Phase { CameraPre, CameraPost, RenderPre, Present };
// Read-only; disabled unless AvatarVR_posetiming.txt explicitly says on.
void observe(Phase phase, const void* view = nullptr);
void on_present();
}
