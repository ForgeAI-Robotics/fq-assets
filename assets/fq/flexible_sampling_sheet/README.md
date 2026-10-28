# Flexible sampling sheet v0.1.0

Thin sampling paper/nonwoven prototype: 170 x 30 x 0.4 mm, 0.6 g, 90 vertices. **model.xml is actual native MuJoCo 3.3.6 2D flex**, with bending elasticity and edge constraints. bend_contact_demo.xml demonstrates edge attachment, gravity sag, curved contact and release. See validation.json and material_parameters.json. Use timestep 0.00005 s; contact parameters on BOTH the sheet and contacting geoms matter (mixed default contacts are too soft).

preview.xml and preview.usda are static visual representations only. No native PhysX cloth is claimed. Material parameters are uncalibrated assumptions; no tearing, plastic creasing, chemical uptake or contamination transfer. The existing airport rigid swab and scene v0.3.0 are unchanged.

Reference: https://mujoco.readthedocs.io/en/3.3.6/XMLreference.html#flex-elasticity . Implementation uses the native flex API, not copied third-party example code.

## Stiffer candidate v0.1.1

Effective Young modulus increased from 2 MPa to 200 MPa; bending thickness and sheet mass unchanged. Damping is 0.00002 and timestep 0.00005 s; edge solref is 0.0001 s. The demo fixes two vertex rows (10 mm grip width), preventing free hinging about a single line. Numerical cantilever comparison under the same two-row grip gives about 115 mm tip drop for 2 MPa versus 59 mm for 200 MPa after 1 s (see stiffness_comparison.json). This is a numerical material candidate, not measured paper properties. Actual bending, curved contact and controlled release remain deformable.
