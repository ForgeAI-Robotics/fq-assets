from pathlib import Path
import json,sys,hashlib
import mujoco,numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
r=Path(sys.argv[1]);old=Path(sys.argv[2]);report={}
for rel in ['assets/fq/conference_table/table.xml','scenes/fq/office_baseline/scene.xml']:
 m=mujoco.MjModel.from_xml_path(str(r/rel));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) for i in range(m.ngeom)]
 ids=[i for i,n in enumerate(names) if n and n.startswith('meeting_table')]
 cols=[i for i in ids if m.geom_contype[i]];assert len(cols)==5
 assert not any(n and n.startswith('meeting_table_leg_') for n in names)
 for i in cols:
  assert m.geom_type[i]==mujoco.mjtGeom.mjGEOM_BOX
 # Grounded frame outer extents, and center underside is free.
 beams=[i for i in cols if '_x_' in names[i]]
 for i in beams:
  half=abs(d.geom_xmat[i].reshape(3,3))@m.geom_size[i]
  assert np.allclose(half,[.03,.555,.3510881960392],atol=1e-6),half
  assert abs(d.geom_xpos[i,2]-half[2])<1e-6
 for z in [.1,.35,.65]:
  point=np.array([2.25568199,10.34090948,z])
  assert all(not np.all(np.abs(d.geom_xmat[i].reshape(3,3).T@(point-d.geom_xpos[i]))<m.geom_size[i]) for i in cols)
 oldm=mujoco.MjModel.from_xml_path(str(old/rel))
 # Unrelated scene physics stays unchanged.
 for i,n in enumerate(names):
  if n and not n.startswith('meeting_table'):
   j=mujoco.mj_name2id(oldm,mujoco.mjtObj.mjOBJ_GEOM,n)
   for attr in ['geom_pos','geom_quat','geom_size','geom_friction','geom_contype','geom_conaffinity']:
    np.testing.assert_array_equal(getattr(m,attr)[i],getattr(oldm,attr)[j])
 report[rel]={'collision_boxes':5,'x_beams':4,'center_under_table_clear':True,'other_geoms_unchanged':True}
for rel,root in [('assets/fq/conference_table/table.usda','/World/meeting_table_v7'),('scenes/fq/office_baseline/scene.usda','/World/meeting_table_v7')]:
 s=Usd.Stage.Open(str(r/rel));p=s.GetPrimAtPath(root)
 bbox=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(p).ComputeAlignedRange();size=list(bbox.GetSize())
 assert np.allclose(size,[3.51,1.2,.75],atol=2e-6),size
 assert len([x for x in Usd.PrimRange(p) if x.HasAPI(UsdPhysics.CollisionAPI)])==5
 assert not s.GetPrimAtPath('/World/meeting_table_collision').IsActive() if 'scene' in rel else True
 for x in Usd.PrimRange(p):
  if x.GetName().startswith('meeting_table_x_') and not x.GetName().endswith('collision'):
   mat,_=UsdShade.MaterialBindingAPI(x).ComputeBoundMaterial();assert mat
 report[rel]={'size_m':size,'collision_boxes':5}
for name in ['basecolor_v4.png','roughness_v4.png']:
 p='assets/fq/conference_table/textures/'+name;assert (r/p).read_bytes()==(old/p).read_bytes()
report['pbr_texture_bytes_unchanged']=True
(r/'validation/table_x351.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
