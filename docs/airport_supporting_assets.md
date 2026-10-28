# Airport supporting assets v0.1.0

This adds independent assets only. Airport scene v0.3.0 and its existing screening controller are unchanged.

## Six new Gazebo Fuel bags

OpenRobotics: Backpack, JanSport Backpack Red, Suitcase1H, Suitcase2H (CC0); Small Dry Bag and Small Dry Bag Handle (CC-BY-4.0). The last item is a complete bag variant, not a loose handle. Same platform as the existing Google scans, different contributor as authorized. Two backpacks, two pull-handle suitcases, two dry bags; no briefcase or duffel is claimed. Exact URLs/archive hashes in each provenance.json. USD/MJCF/GLB, metric dimensions, color/available PBR maps. MuJoCo floor-drop checked; actual Isaac Sim render checked. These remain rigid props; wheels, handles and straps are not articulated. Mass/friction are estimates. Dry bags use source compound collisions; others use convex approximations.

## Three passenger combinations

Two fitted backpack poses and one small hand-carry pose. Microsoft Rocketbox MIT + Google GSO CC-BY-4.0. Bone-local attachment and short pose animation in USD; static skinned snapshot in web/MJCF. Visual straps/handle, no cloth straps, force-validated grip, walking or passenger-flow logic.

## Sampling sheet and box

Native MuJoCo 2D flex thin sampling paper/nonwoven, not a rigid swab. Edge grip, bending, curved contact, release tested. Box has hollow collision, finger access, driven lid and a top-sheet removal demo. Lower stock sheets are static; single-sheet separation is not validated. USD sheet is visual only; no Isaac native cloth claim. Dimensions and material parameters need physical consumable calibration. See per-asset reports and the video in validation/previews.

## Viewer clarification

The airport page describes its own batch-screening layout. Its belt controller only retracts/extends rigid proxies; endpoint lifting, unlatching/latching and flexible webbing are not implemented. The airport still uses the legacy rigid swab. The separate flexible sampling-sheet page defaults to a recorded native MuJoCo physics demonstration, with an explicit static 3D appearance switch. This playback is not a browser physics engine or an integration into the airport scene. Model versions are unchanged by these UI/documentation corrections.

## Sampling and collision correction v0.1.1

The independent rigid swab entry and its asset directory are removed. Only the minimal fixture required to replay the historical airport v0.3.0 operation is retained under examples/airport_operations, explicitly documented as legacy. It is not a current library consumable. The airport operation has not yet migrated to flexible-sheet grasp/insertion.

The flexible sheet remains native MuJoCo flex, with a stiffer numerical material candidate and finite-width grip; recordings and validation reports are refreshed. No physical-material calibration claim.

All three passenger/bag assemblies retain external bag collision. USD bags are kinematic rigid bodies following the authored bone pose; wearer navigation proxy collisions are filtered. MuJoCo supports collision in the static fitted pose. MuJoCo probe contact and Isaac Sim 4.5 PhysX guided-drop tests pass; disabling bag collision provides a negative control. Straps remain visual and bags do not freely swing.

## Next level of human modeling

Keep appearance, movement and contact as separate representations. Existing Rocketbox meshes and 80-joint rigs supply appearance and posing. For the airport task, the next useful step is roughly 15–20 bone-following collision parts (torso, pelvis, upper/lower limbs, hands and feet), plus sampleable regions on clothes and bags. Log surface region, contact location/normal, force estimate and wipe path. Whole-body soft tissue simulation is not required for that first contact model.

Use detailed local surfaces where the swab actually touches; calibrate their contact/friction against the real clothing or bag material. Add shoulder strap deformation and compliant bag attachment only when validating strap forces or bag sway. Validate self-intersections, reachability, normal contact, pressure limits, body-shape/pose variation and avoidance of the head/face. Visual mesh detail alone does not establish physical or human-contact accuracy. These finer human models are planned, not implemented in this release.

## Retractable belt update (2026-10-28)

The earlier rigid-belt viewer clarification is historical. The current airport catalog references `belt_preview.xml` / `belt_preview.usda` and exposes `belt_interaction.xml` for native MuJoCo flexible belt operations. The new `retractable_belt_kit` asset includes a checked grasp/lift/unlatch/reel/seat/latch cycle and defaults to a native physics video. USD and web 3D remain static snapshots; no PhysX cloth or browser physics is claimed. Reel feed is prescribed, and full passenger/robot workflow integration remains pending.

## Preview correction (2026-10-28)

The airport scene page now shows only its static 3D model; operation videos remain on the belt kit page. The kit has an explicit catalog thumbnail, and static previews include a webbing collider approximation. Native MuJoCo flex dynamics are unchanged. The office thumbnail shows the entire modeled layout with ceiling hidden for the render only.
