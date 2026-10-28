# Supporting asset validation

Run from a checkout with hydrated Git LFS files. validate_soft_supply.py and validate_fuel_bags.py need MuJoCo 3.3.6, numpy, Pillow; soft supply also needs imageio-ffmpeg and DejaVu Sans. MUJOCO_GL=egl supports headless rendering. check_combo_bindings.py needs OpenUSD Python bindings and numpy. render_new_assets_isaac.py runs with Isaac Sim 4.5 python.sh on an RTX GPU. Output goes to /tmp/fq-airport-validation. Validators update per-asset validation reports (and generate the dispensing example), so use a writable checkout.

MuJoCo flex is native physics. USD sheet and browser are visual snapshots. Bag/character Isaac images validate rendering, not PhysX dynamics. The clamps are equality attachments, not frictional fingers. Lower stock sheets are fixed spacers, not multilayer cloth. See docs/airport_supporting_assets.md.
