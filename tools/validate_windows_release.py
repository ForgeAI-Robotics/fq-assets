from pathlib import Path
import json,hashlib
import numpy as np,mujoco
from pxr import Usd,UsdGeom
r=Path('/assets');old=Path('/old');rel='scenes/fq/office_baseline/'
a=mujoco.MjModel.from_xml_path(str(old/rel/'scene.xml'));b=mujoco.MjModel.from_xml_path(str(r/rel/'scene.xml'))
assert a.nq==b.nq and a.nv==b.nv
checked=0
for i in range(a.ngeom):
    if not (a.geom_contype[i] or a.geom_conaffinity[i]):continue
    name=mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_GEOM,i);j=mujoco.mj_name2id(b,mujoco.mjtObj.mjOBJ_GEOM,name)
    assert j>=0
    for key in ('geom_type','geom_pos','geom_quat','geom_size','geom_friction','geom_solref','geom_solimp','geom_contype','geom_conaffinity'):
        np.testing.assert_array_equal(getattr(a,key)[i],getattr(b,key)[j])
    checked+=1
da=mujoco.MjData(a);db=mujoco.MjData(b)
for _ in range(1000):
    mujoco.mj_step(a,da);mujoco.mj_step(b,db)
np.testing.assert_array_equal(da.qpos,db.qpos);np.testing.assert_array_equal(da.qvel,db.qvel)
stage=Usd.Stage.Open(str(r/rel/'scene.usda'))
assert UsdGeom.Imageable(stage.GetPrimAtPath('/World/east')).ComputeVisibility()=='invisible'
assert len(list(stage.GetPrimAtPath('/World/DaylightWindows').GetChildren()))==11
report=dict(collision_geoms_unchanged=checked,physics_steps=1000,qpos_max_difference=float(abs(da.qpos-db.qpos).max()),window_visual_prims=11,baseline_version='0.1.13')
(r/'validation/windows_release.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
