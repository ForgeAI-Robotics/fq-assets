from pathlib import Path
import json,shutil
import numpy as np,mujoco
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Vt
root=Path('/mnt/simple');assets=Path('/assets');out=Path('/out');report=[]
def mesh(stage,path,m,g,collision=False):
 mi=m.geom_dataid[g];vertices=m.mesh_vert[m.mesh_vertadr[mi]:m.mesh_vertadr[mi]+m.mesh_vertnum[mi]];R=np.zeros(9);mujoco.mju_quat2Mat(R,m.geom_quat[g]);vertices=vertices@R.reshape(3,3).T+m.geom_pos[g];faces=m.mesh_face[m.mesh_faceadr[mi]:m.mesh_faceadr[mi]+m.mesh_facenum[mi]]
 x=UsdGeom.Mesh.Define(stage,path);x.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)));x.CreateFaceVertexCountsAttr([3]*len(faces));x.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist());x.CreateSubdivisionSchemeAttr('none');x.CreateDoubleSidedAttr(True);x.CreateDisplayColorAttr([Gf.Vec3f(*map(float,m.geom_rgba[g,:3]))])
 if collision:
  x.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(x.GetPrim()).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(x.GetPrim()).CreateApproximationAttr('convexHull')
 return x
for family,folder in [('o6','g1_o6'),('o7','g1_o7_left')]:
 m=mujoco.MjModel.from_xml_path(str(assets/f'assets/fq/robots/g1_{family}/robot.xml'))
 for file in (root/'data/robots'/folder).glob('*.usd'):
  stage=Usd.Stage.Open(str(file));torso=next(p for p in stage.Traverse() if p.GetName()=='torso_link');base=str(torso.GetPath());backup=out/'usd-backup'/folder/file.name;backup.parent.mkdir(parents=True,exist_ok=True)
  if not backup.exists():shutil.copy2(file,backup)
  visual=stage.GetPrimAtPath(base+'/visuals/head_link');assert visual
  # De-instance only the ancestor chains of the edited visual/collision prims.
  for path in [base+'/visuals',base+'/collisions']:
   p=stage.GetPrimAtPath(path)
   if p and p.IsInstance():p.SetInstanceable(False)
  visual=stage.GetPrimAtPath(base+'/visuals/head_link');old=stage.GetPrimAtPath(str(visual.GetPath())+'/mesh');assert old;old.SetActive(False)
  g=next(g for g in range(m.ngeom) if m.geom_bodyid[g]==m.body('head_link').id and m.geom_group[g]==1);mesh(stage,str(visual.GetPath())+'/fq_slot',m,g)
  collision=stage.GetPrimAtPath(base+'/collisions/head_link');assert collision
  transform=UsdGeom.Xformable(collision).GetLocalTransformation();collision.SetActive(False)
  for g in range(m.ngeom):
   if m.geom_bodyid[g]==m.body('head_link').id and m.geom_group[g]==3:
    parent=UsdGeom.Xform.Define(stage,base+'/collisions/'+m.geom(g).name);parent.AddTransformOp().Set(transform);mesh(stage,str(parent.GetPath())+'/mesh',m,g,True)
  housing=stage.GetPrimAtPath(base+'/d435i_head_housing');assert housing
  for child in list(housing.GetChildren()):stage.RemovePrim(child.GetPath())
  for g in range(m.ngeom):
   if m.geom_bodyid[g]==m.body('d435i_head_housing').id and m.geom_rgba[g,3]>0:mesh(stage,str(housing.GetPath())+'/optical_part_'+str(g),m,g)
  torso.SetCustomDataByKey('fq:head_geometry_status','provisional_camera_slot_not_physically_measured');stage.GetRootLayer().Save()
  # Serialized file is readable; this is not a live Isaac/PhysX runtime test.
  check=Usd.Stage.Open(str(file));assert check.GetPrimAtPath(str(visual.GetPath())+'/fq_slot')
  report.append({'path':str(file),'head_visual_and_four_convex_collision_parts_updated':True,'head_optical_channels_updated':True,'physics_runtime_tested':False})
(out/'head-slot-usd-sync.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
