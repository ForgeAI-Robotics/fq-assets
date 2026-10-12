# 0.1.1 — 2026-10-12

Correct the estimated right door hinge axis by +12 mm along cabinet X, to X=0.354 m. Preserve closed geometry, collision shapes, masks, and the left hinge. Synchronize MJCF, USD, Blender source, and export scripts.

Validated with a 0–90 degree MuJoCo collision sweep. A separate fixture-supported native G1+O6 experiment reached approximately 55 degrees with continued left-palm pushing after right-hand release. This is not free-standing robot or Isaac/PhysX runtime validation. Hinge placement and physical parameters remain uncalibrated against hardware.
