# Photo basket v1

SIMPLE asset ID: `basket:photo_basket_v1`. Canonical location: `data/assets/basket/photo_basket_v1`.

```python
from simple.assets import AssetManager
asset = AssetManager.create("basket:photo_basket_v1")
print(asset.mjcf_path, asset.usd_path, asset.articulate_init_joint_qpos)
```

The asset has a free basket body and two passive handle hinges. Estimated total mass: **0.41 kg** (body 0.32, each handle 0.045). Estimated rim size: 0.36 x 0.24 m; body height: 0.23 m. Parameters are not measurements. Z is up; origin is at the bottom center. Real visual perforations; segmented convex walls keep the collision interior open.

Load `basket_asset.xml` for the reusable asset (no ground or lights, origin at the bottom center); `basket.xml` remains the original standalone preview or `test_mujoco.xml` for the drop test. Apply keyframe `photo_pose` to use lowered handles. Load `basket.usd` in Isaac (keep `basket.usdc` alongside it); its lowered handle pose is already authored. URDF zero pose differs: explicitly set both hinge angles to -0.65 rad after import, and do not replace segmented collisions with one hull.

This registration exposes a reusable articulated asset, not a robot grasp task. Existing SIMPLE generic articulated actor adapters have additional assumptions about joint names and USD hierarchy; use the explicit engine entry files and supplied integration validation when composing a new task. Robot adapter integration and grasp-policy regression are separate work.

`validation_history/` and `SOURCE_README.md` are historical evidence. Current remote installation checks are in `validation/`. The model was authored locally; no remote Blender installation is implied.
