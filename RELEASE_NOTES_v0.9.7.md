# v0.9.7 — USD round table scale correction

Isaac validation revealed the legacy Meshes dimensions scale was applied twice to v0.1.2 visual geometry. Round table 0.1.3 compensates this scale only on its three visual meshes in standalone USD and baseline 0.1.22. World visual dimensions are 0.70 x 0.70 x 0.75 m again. Collision meshes, transforms and parameters remain unchanged. PBR and MJCF geometry are unchanged.
