from pathlib import Path
import json,sys
import numpy as np,mujoco as mj
from pxr import Usd,UsdGeom,Gf
r=Path(sys.argv[1]);s=Usd.Stage.Open(str(r/'scenes/fq/office_baseline/scene.usda'));m=mj.MjModel.from_xml_path(str(r/'scenes/fq/office_baseline/scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d);cache=UsdGeom.XformCache();max_error=0;count=0
for k in range(1,7):
 root=s.GetPrimAtPath('/World/office_chair_'+str(k));assert root
 for p in Usd.PrimRange(root):
  if not p.IsA(UsdGeom.Mesh) or '/Visuals/' not in str(p.GetPath()):continue
  n=p.GetName().replace('neg1','-1');gid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'office_chair_'+str(k)+'__'+n);assert gid>=0,n
  pts=np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get());mat=np.array(cache.GetLocalToWorldTransform(p));usd=pts@mat[:3,:3]+mat[3,:3]
  mid=m.geom_dataid[gid];v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]];actual=v@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
  err=max(np.max(np.abs(usd.min(0)-actual.min(0))),np.max(np.abs(usd.max(0)-actual.max(0))));max_error=max(max_error,float(err));count+=1
assert count==726 and max_error<1e-5,(count,max_error)
(r/'validation/baseline_chairs_usd.json').write_text(json.dumps({'passed':True,'visual_meshes_compared':count,'max_world_bounds_error_m':max_error,'scope':'USD vs MJCF posed visual bounds; USD chair colliders authored from the same MuJoCo primitives; static placement only'},indent=2));print(count,max_error)
