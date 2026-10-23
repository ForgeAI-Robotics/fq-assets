from pathlib import Path
import json,sys,hashlib,xml.etree.ElementTree as ET
import numpy as np,mujoco
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
r=Path(sys.argv[1]);old=Path(sys.argv[2]);report={}
for p in (old/'assets/fq/tabletop/common/meshes').glob('round_table_collision*'):
 assert p.read_bytes()==(r/p.relative_to(old)).read_bytes()
report['collision_mesh_bytes_unchanged']=True
for rel in ['assets/fq/tabletop/mujoco/round_table.xml','scenes/fq/office_baseline/scene.xml']:
 a=ET.parse(old/rel);b=ET.parse(r/rel)
 for kind in ['geom','body','joint']:
  before={x.get('name'):x.attrib for x in a.iter(kind) if x.get('name')!='round_table_visual'}
  after={x.get('name'):x.attrib for x in b.iter(kind) if x.get('name') not in ['round_table_'+part+'_visual' for part in ['top','pillar','base']]}
  assert before==after,(rel,kind)
 m=mujoco.MjModel.from_xml_path(str(r/rel));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 vs=[];negative=0;min_dot=1
 for part in ['top','pillar','base']:
  g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'round_table_'+part+'_visual');mid=m.geom_dataid[g]
  v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
  vs.append(v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g])
  fa=m.mesh_faceadr[mid];faces=m.mesh_face[fa:fa+m.mesh_facenum[mid]];tris=v[faces]
  fn=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]);fn/=np.linalg.norm(fn,axis=1)[:,None]
  n=m.mesh_normal[m.mesh_normaladr[mid]+m.mesh_facenormal[fa:fa+m.mesh_facenum[mid]]]
  dot=(fn[:,None,:]*n).sum(2);negative+=int((dot<0).sum());min_dot=min(min_dot,float(dot.min()))
  np.testing.assert_allclose(np.linalg.norm(n,axis=2),1,atol=1e-6)
 v=np.concatenate(vs);size=np.ptp(v,axis=0);np.testing.assert_allclose(size,[.7,.7,.75],atol=2e-6);assert negative==0
 report[rel]={'dimensions_m':size.tolist(),'negative_normal_count':negative,'minimum_face_normal_dot':min_dot,'all_original_physics_and_body_placements_unchanged':True,'compiled':True}
for rel,base in [('assets/fq/tabletop/isaac/round_table.usd','/World'),('scenes/fq/office_baseline/scene.usda','/World/round_table')]:
 a=Usd.Stage.Open(str(old/rel));b=Usd.Stage.Open(str(r/rel))
 for prim in a.Traverse():
  if prim.HasAPI(UsdPhysics.CollisionAPI):
   new=b.GetPrimAtPath(prim.GetPath());assert new
   for attr in prim.GetAttributes():assert str(attr.Get())==str(new.GetAttribute(attr.GetName()).Get()),str(attr.GetPath())
 for part in ['top','pillar','base']:
  prim=b.GetPrimAtPath(base+'/Meshes/'+part+'_visual');assert not prim.HasAPI(UsdPhysics.CollisionAPI)
  mat,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial();assert mat
  bounds=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(prim).ComputeAlignedRange()
 report[rel]={'all_existing_collision_attributes_unchanged':True,'pbr_bound_regions':3}
report['appearance_status']='estimated, not measured; browser lighting is not Isaac RTX validation'
(r/'validation/round_table_pbr.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
