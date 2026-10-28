from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'disable_viewport_updates':True})
from pathlib import Path
import json,numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from pxr import Usd,UsdGeom,UsdPhysics,Gf
R=Path(__file__).resolve().parents[2];world=World(physics_dt=1/120,stage_units_in_meters=1,backend='numpy',device='cpu');stage=world.stage;world.scene.add_default_ground_plane();records=[]
for i,D in enumerate(sorted((R/'assets/fq/passenger_combinations').iterdir())):
 root=UsdGeom.Xform.Define(stage,'/World/Combo'+str(i));root.GetPrim().GetReferences().AddReference(str(D/'assembly.usdc'));root.AddTranslateOp().Set(Gf.Vec3d((i-1)*2,0,0));bag=stage.GetPrimAtPath(str(root.GetPath())+'/Bag');cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']);bounds=cache.ComputeWorldBound(bag).ComputeAlignedRange();lo=np.array(bounds.GetMin());hi=np.array(bounds.GetMax());pos=(lo+hi)/2;pos[2]=hi[2]+.15
 probe=world.scene.add(DynamicCuboid(prim_path='/World/Probe'+str(i),name='probe'+str(i),position=pos,size=.04,mass=.1,color=np.array([1.,.3,.1])))
 # A vertical guide avoids confusing sliding off a curved backpack with missing collision.
 guide=UsdPhysics.PrismaticJoint.Define(stage,'/World/ProbeGuide'+str(i));guide.CreateBody1Rel().SetTargets([probe.prim.GetPath()]);guide.CreateLocalPos0Attr(Gf.Vec3f(*pos));guide.CreateLocalPos1Attr(Gf.Vec3f(0));guide.CreateAxisAttr('Z');guide.CreateLowerLimitAttr(-3);guide.CreateUpperLimitAttr(.2);records.append((D,bag,probe,pos))
world.reset()
for _ in range(480):world.step(render=False)
enabled=[]
for D,bag,probe,pos in records:
 p,_=probe.get_world_pose();enabled.append(float(p[2]));assert p[2]>.45,(D,'probe fell through enabled bag',p)
 for prim in Usd.PrimRange(bag):
  if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
 probe.set_world_pose(position=pos);probe.set_linear_velocity(np.zeros(3));probe.set_angular_velocity(np.zeros(3))
for _ in range(480):world.step(render=False)
for i,(D,bag,probe,pos) in enumerate(records):
 p,_=probe.get_world_pose();assert p[2]<.1,(D,'disabled control did not fall',p)
 v=json.loads((D/'validation.json').read_text());v['physx_dynamic_test']=True;v['physx_external_bag_contact']={'enabled_probe_final_z_m':enabled[i],'disabled_probe_final_z_m':float(p[2]),'engine':'Isaac Sim 4.5 / PhysX CPU','test':'4 second vertically guided dynamic cube drop onto kinematic carried bag; disabled-collider negative control'};(D/'validation.json').write_text(json.dumps(v,indent=2)+'\n');print('PHYSX_BAG_CONTACT_PASS',D.name,enabled[i],float(p[2]),flush=True)
app.close()
