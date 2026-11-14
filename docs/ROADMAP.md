# Roadmap

## Goal

Native same-frame stereo, 6DoF head tracking, stable decoupled head pitch, OpenXR motion controllers, comfortable and correctly placed UI, and usable performance. The longer-term goal is physical VR gameplay: roomscale movement, controller-based weapon aiming, visible tracked arms, natural weapon/item handling and selected HUD elements attached to the body. Preserve the working flat-screen fallback.

## Known issues

- [ ] Wavy, shadow-like artifact follows moving IK body, weapon and NPCs in the right eye only
- [ ] IK hands draw in front of nearer objects/body (arm depth occlusion regressed)
- [ ] Decoupled pitch occasionally snaps the view fully up or down
- [x] Main menu and gameplay HUD centering
- [ ] X-ray scan disrupts the 3D view; animal/plant/environment highlights are off-center
- [ ] Distant shadows differ between the eyes
- [x] Sharpness slider
- [ ] Performance is currently too heavy
- [x] Changing VR resolution in the pause/main menu prevents returning to 3D VR
- [ ] Native stick cursor appearance and hover alignment
- [ ] Flat screen follows the head; flat flashes between loading and stereo gameplay
- [ ] Right eye dark (ambient light lost) after fast travel to the cave
- [ ] Motion-controller menu pointer

## Planned features

- [ ] Roomscale
- [ ] Raycast weapon aiming
- [ ] IK arms
- [ ] Hand gestures and finger movement
- [ ] Manual reload
- [ ] First-person mounts
- [ ] Physical bow shooting and arrow tension
- [ ] World-scale slider
- [ ] Holsters
- [ ] Grenade throwing
- [ ] Two-handed weapon handling
- [ ] Physical melee
- [ ] Selected HUD elements on the body
- [ ] In-world reticle on the weapon raycast

`[x]` = done and checked in the headset. `[ ]` = open (may already have partial code behind a menu toggle).
