from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'disable_viewport_updates':True})
from pathlib import Path
import json
import omni.usd
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from isaacsim.core.api import SimulationContext
R=Path(__file__).resolve().parents[1];D=R/'scenes/fq/airport_preview'
omni.usd.get_context().open_stage(str(D/'scene.usdc'));app.update()
stage=omni.usd.get_context().get_stage()
tests=[('floor',[4,-3,1],[0,0,0],.05),('worktop',[-1,.65,1.4],[0,0,0],.7925),('detector',[.19,1.38,1.7],[0,0,0],1.16),('belt',[0,3.25,.98],[0,1,0],None),('partition',[1.7,3.25,.6],[0,1,0],None),('open_passage',[0,6.4,.6],[0,1,0],None)]
for name,pos,vel,z in tests:
    sphere=UsdGeom.Sphere.Define(stage,'/World/Probe_'+name);sphere.CreateRadiusAttr(.05)
    sphere.AddTranslateOp().Set(Gf.Vec3d(*pos));p=sphere.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    rigid=UsdPhysics.RigidBodyAPI.Apply(p);rigid.CreateVelocityAttr(Gf.Vec3f(*vel))
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.1)
    api=PhysxSchema.PhysxRigidBodyAPI.Apply(p);api.CreateDisableGravityAttr(z is None);api.CreateEnableCCDAttr(True)
sim=SimulationContext(physics_dt=1/240,rendering_dt=1/60,stage_units_in_meters=1)
sim.initialize_physics();sim.play()
for _ in range(480):sim.step(render=False)
results=[]
for name,pos,vel,z in tests:
    p=stage.GetPrimAtPath('/World/Probe_'+name)
    xyz=list(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
    if z:assert abs(xyz[2]-z)<.025,(name,xyz)
    elif name=='open_passage':assert xyz[1]>8.2,(name,xyz)
    else:assert 3.25<xyz[1]<3.65,(name,xyz)
    results.append({'test':name,'passed':True,'final_xyz':xyz})
p=R/'validation/airport_preview_v011.json';r=json.loads(p.read_text());r['isaac_physx_dynamic_probes']=results;p.write_text(json.dumps(r,indent=2)+'\n');print('COLLISION_RESULTS '+json.dumps(results),flush=True)
sim.stop();app.close()
