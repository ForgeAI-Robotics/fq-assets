from pathlib import Path
import sys,json
sys.path.insert(0,'/home/fangqi/.local/share/fq-assets-validation-deps')
from pxr import Usd,UsdGeom,UsdPhysics,Gf
r=Path(sys.argv[1]);sd=r/'scenes/fq/office_baseline';stage=Usd.Stage.Open(str(sd/'scene.usda'))
for it in json.loads((sd/'chair_placements.json').read_text())['instances']:
 path='/World/'+it['name'];stage.RemovePrim(path);inst=UsdGeom.Xform.Define(stage,path);inst.GetPrim().GetReferences().AddReference('../../../assets/fq/office_chair/chair.usda');inst.AddTranslateOp().Set(Gf.Vec3d(*it['position_m']));inst.AddRotateZOp().Set(it['yaw_deg'])
 stage.GetPrimAtPath(path+'/Collision').SetActive(False)
 for name,p in it['mesh_body_poses'].items():
  prim=stage.GetPrimAtPath(path+'/Visuals/'+name.replace('-','neg'))
  if not prim.IsValid():continue
  xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(*p['body_pos']));q=p['body_quat'];xf.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
 for c in it['collisions']:
  cp=path+'/PhysicsCollision/'+c['name'].replace('-','neg');sz=c['size'];t=c['type']
  if t==6:sh=UsdGeom.Cube.Define(stage,cp);sh.CreateSizeAttr(2)
  elif t==4:sh=UsdGeom.Sphere.Define(stage,cp);sh.CreateRadiusAttr(1)
  elif t==2:sh=UsdGeom.Sphere.Define(stage,cp);sh.CreateRadiusAttr(sz[0])
  elif t==5:sh=UsdGeom.Cylinder.Define(stage,cp);sh.CreateRadiusAttr(sz[0]);sh.CreateHeightAttr(2*sz[1])
  elif t==3:sh=UsdGeom.Capsule.Define(stage,cp);sh.CreateRadiusAttr(sz[0]);sh.CreateHeightAttr(2*sz[1])
  else:raise ValueError(t)
  xf=UsdGeom.Xformable(sh);xf.AddTranslateOp().Set(Gf.Vec3d(*c['pos']));q=c['quat'];xf.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
  if t in (4,6):xf.AddScaleOp().Set(Gf.Vec3f(*sz))
  sh.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(sh.GetPrim())
stage.GetRootLayer().Save();print('USD fixed posed chairs with matching collision shapes saved')
