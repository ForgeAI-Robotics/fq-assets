# BDO-6550 basket — 0.1.2

Body nominal envelope: **280 × 190 × 165 mm**, excluding handles. Source: user label photo and explicit body-height confirmation. Bottom **240 × 150 mm**, rim corner radius **28 mm** and handle geometry are estimates, not measured factory geometry.

PP body, open-top segmented collision; independent passive handles. Estimated total mass remains 0.41 kg. Inertia is rescaled from prior estimates; friction is not calibrated. See `dimensions_update.json` and `parameters.json`.

Generate geometry with Blender `--background --threads 1 --python build_basket.py -- <directory>`; optional `--render` uses CPU. Then run `export_assets.py` and `build_usd.py`. Current validation is recorded separately from historical Isaac results.

MuJoCo floor and wall contact: priority 2, solref 0.002/1, solimp 0.98/0.999/0.0005, margin 1 mm, gap 0. Use a 0.5 ms physics timestep for the verified drop behavior. This overrides priority-1 can softness. The 1 mm numerical contact layer is not measured material calibration. Geometry, mass and friction are unchanged. USD contact offset remains 1 mm; PhysX has not been dynamically retested.

SIMPLE must preserve the asset MULTICCD flag when composing mesh contacts.
