# 男性旅客 + 双肩背包

Human: third_party:characters/rocketbox_male_adult_06 (Microsoft MIT). Bag: third_party:bags/gso_jansport_school_backpack_blue_streak (Google CC-BY-4.0). Both source licenses are included.

assembly.usdc preserves UsdSkel bones and attaches the bag to Bip01 Spine2. Frames 0–60 are a small pose-following example, not a walking motion. preview.xml is a static skinned view. Custom ribbons/handle fit the pose visually; no cloth, gripping force, walking controller or detailed contact validation. The bag is kinematic in the assembly; the standalone source bag retains its own rigid physics.

## Collision update v0.1.1

The carried bag now retains external collision. USD uses a kinematic rigid body with convex collision and filters only the wearer collision prims. MuJoCo retains the fitted static pose with bag collision (type 2, affinity 1); the coarse human navigation collider uses type 4, affinity 1. External default-type-1 objects collide with both. The bag does not dynamically swing and straps remain visual. Detailed skin/garment contact is not yet modeled.
