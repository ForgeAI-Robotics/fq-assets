# Baggage assets v0.1.0

Eight Google Scanned Objects scans, CC-BY-4.0: five backpacks (one JanSport adult/daypack and four Olive Kids packs) and three small lunch/carry bags (digital camo, US Army, butterfly). These are eight separate scans, not procedural recolors. This batch does not cover wheeled luggage, business briefcases or large duffels.

Each directory under assets/third_party/bags includes textured OBJ, USD rigid body, MuJoCo free body, attribution and provenance with archive/file hashes. Source metric scale is retained, XY centered and Z grounded. Heights range from approximately 22 to 44 cm. Texture resolution retained.

MuJoCo load and dynamic floor-drop checks passed for every model; USD dependencies and actual Isaac Sim rendering checked. PhysX dynamic tests have not been performed. Convex hull collision fills strap and handle gaps. Mass and friction are estimates. These are rigid visual/contact props, not deformable cloth bags. No fitted human attachment, opening, internal contents, explosive detection or alarm logic is included.

Original scans: https://fuel.gazebosim.org/1.0/GoogleResearch/models ; exact per-model URLs and modifications in provenance.json. Copyright 2020 Google LLC; https://creativecommons.org/licenses/by/4.0/ .
