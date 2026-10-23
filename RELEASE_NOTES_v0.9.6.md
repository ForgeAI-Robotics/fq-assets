# v0.9.6 — round table rim and regional PBR

Round table 0.1.2 regularizes the visual rim with a 1 mm bevel and analytic normals, and separates tabletop, coated pillar and coated base PBR materials. Preserves the photo-derived tabletop color texture. Material values and bevels are estimates, not physical calibration.

Overall dimensions remain 0.70 x 0.70 x 0.75 m. All three collision mesh files, collision parameters, body placements and all other scene physics are unchanged. Baseline 0.1.21 uses the same new visual surfaces in MJCF and USD. The viewer supports original, soft and strong lighting and uses adjusted round-table shadow bias.

Validation: standalone and baseline MJCF compile; zero reversed face/vertex normals; dimension tolerance 2 micrometers; collision mesh byte equality; all existing USD collision attributes unchanged; three USD material bindings. CPU-only validation and browser previews; no Isaac RTX result claimed and no running SIMPLE task or version lock changed.
