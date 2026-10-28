from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1920,'height':1080,'renderer':'RayTracedLighting','disable_viewport_updates':True})
from pathlib import Path
import json
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdLux,UsdPhysics,Gf
R=Path(__file__).resolve().parents[2];out=Path('/tmp/fq-airport-validation/soft_assets/isaac');out.mkdir(parents=True,exist_ok=True)
omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1)
bagdirs=sorted((R/'assets/third_party/bags').glob('fuel_openrobotics_*'));combodirs=sorted((R/'assets/fq/passenger_combinations').iterdir())
for i,D in enumerate(bagdirs):
 x=UsdGeom.Xform.Define(stage,'/World/Bag'+str(i));x.GetPrim().GetReferences().AddReference(str(D/'model.usdc'));x.AddTranslateOp().Set(Gf.Vec3d((i-2.5)*.7,-1.5,0));UsdPhysics.RigidBodyAPI(x.GetPrim()).GetRigidBodyEnabledAttr().Set(False)
for i,D in enumerate(combodirs):
 x=UsdGeom.Xform.Define(stage,'/World/Person'+str(i));x.GetPrim().GetReferences().AddReference(str(D/'assembly.usdc'));x.AddTranslateOp().Set(Gf.Vec3d((i-1)*1.5,.3,0))
floor=UsdGeom.Cube.Define(stage,'/World/Floor');floor.AddTranslateOp().Set(Gf.Vec3d(0,0,-.025));floor.AddScaleOp().Set(Gf.Vec3d(4,3,.025));floor.CreateDisplayColorAttr([Gf.Vec3f(.72,.74,.77)])
light=UsdLux.DomeLight.Define(stage,'/World/Dome');light.CreateIntensityAttr(650)
key=UsdLux.DistantLight.Define(stage,'/World/Sun');key.CreateIntensityAttr(1700);key.AddRotateXYZOp().Set(Gf.Vec3f(-35,-20,-20));key.CreateAngleAttr(12)
rep.orchestrator.set_capture_on_play(False)
camera=rep.create.camera(position=(0,-10,4.3),look_at=(0,-.3,.75),focal_length=36);product=rep.create.render_product(camera,(1920,1080))
for name,pos,look,focal in [('overview',(0,-10,4.3),(0,-.3,.75),36),('back',(0,7,2.3),(0,.3,.9),36),('bags',(0,-5.8,2.5),(0,-1.5,.3),36)]:
 with camera:rep.modify.pose(position=pos,look_at=look)
 for _ in range(15):app.update()
 writer=rep.WriterRegistry.get('BasicWriter');writer.initialize(output_dir=str(out/name),rgb=True);writer.attach([product]);rep.orchestrator.step(rt_subframes=32);rep.orchestrator.wait_until_complete();writer.detach()
for D in bagdirs+combodirs:
 r=json.loads((D/'validation.json').read_text());r.update(isaac_stage_load=True,isaac_render=True,physx_dynamic_test=False);(D/'validation.json').write_text(json.dumps(r,indent=2)+'\n')
print('ISAAC_NEW_ASSETS_RENDERED',flush=True);app.close()
