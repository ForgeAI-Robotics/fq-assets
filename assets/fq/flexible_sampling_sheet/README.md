# Flexible sampling sheet v0.1.0

Thin sampling paper/nonwoven prototype: 170 x 30 x 0.4 mm, 0.6 g, 90 vertices. **model.xml is actual native MuJoCo 3.3.6 2D flex**, with bending elasticity and edge constraints. bend_contact_demo.xml demonstrates edge attachment, gravity sag, curved contact and release. See validation.json and material_parameters.json. Use timestep 0.0002 s; contact parameters on BOTH the sheet and contacting geoms matter (mixed default contacts are too soft).

preview.xml and preview.usda are static visual representations only. No native PhysX cloth is claimed. Material parameters are uncalibrated assumptions; no tearing, plastic creasing, chemical uptake or contamination transfer. The existing airport rigid swab and scene v0.3.0 are unchanged.

Reference: https://mujoco.readthedocs.io/en/3.3.6/XMLreference.html#flex-elasticity . Implementation uses the native flex API, not copied third-party example code.
