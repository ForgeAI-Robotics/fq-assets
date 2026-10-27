from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'disable_viewport_updates':True})
from pathlib import Path
import json
import omni.usd
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from isaacsim.core.api import SimulationContext
R=Path(__file__).resolve().parents[1];D=R/'scenes/fq/airport_preview'
omni.usd.get_context().open_stage(str(D/'scene.usdc'));app.update();stage=omni.usd.get_context().get_stage()
tests=[('floor',[4,-3,1],[0,0,0],.05),('worktop',[1.65,.75,1.4],[0,0,0],.7925),('detector',[1.65,1.75,1.7],[0,0,0],1.1325),('exit',[0,3.25,.98],[0,1,0],None),('entry',[0,-2.2,.98],[0,1,0],None),('left',[-2,.7,.98],[-1,0,0],None),('right',[2,-.6,.98],[1,0,0],None),('front_flank',[-1.6,-2.2,.98],[0,1,0],None),('rear_flank',[-1.6,3.25,.98],[0,1,0],None)]
for name,pos,vel,z in tests:
    sphere=UsdGeom.Sphere.Define(stage,'/World/Probe_'+name);sphere.CreateRadiusAttr(.05);sphere.AddTranslateOp().Set(Gf.Vec3d(*pos));p=sphere.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p);rigid=UsdPhysics.RigidBodyAPI.Apply(p);rigid.CreateVelocityAttr(Gf.Vec3f(*vel));UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.1)
    api=PhysxSchema.PhysxRigidBodyAPI.Apply(p);api.CreateDisableGravityAttr(z is None);api.CreateEnableCCDAttr(True)
sim=SimulationContext(physics_dt=1/240,rendering_dt=1/60,stage_units_in_meters=1);sim.initialize_physics();sim.play()
for _ in range(480):sim.step(render=False)
results=[]
for name,pos,vel,z in tests:
    p=stage.GetPrimAtPath('/World/Probe_'+name);xyz=list(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
    if z:assert abs(xyz[2]-z)<.025,(name,xyz)
    else:
        distance=sum((a-b)*v for a,b,v in zip(xyz,pos,vel));assert .1<distance<.4,(name,xyz)
    results.append({'test':name,'passed':True,'final_xyz':xyz})
p=R/'validation/airport_preview_v020.json';r=json.loads(p.read_text());r['isaac_physx_dynamic_probes']=results;p.write_text(json.dumps(r,indent=2)+'\n');print('ENCLOSURE_RESULTS '+json.dumps(results),flush=True)
sim.stop();app.close()
