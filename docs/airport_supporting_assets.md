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
