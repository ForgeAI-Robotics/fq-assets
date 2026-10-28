from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'disable_viewport_updates':True})
from pathlib import Path
import json,sys
sys.path.insert(0,str(Path(__file__).parent));from airport_interaction import usd_belts
import omni.usd
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils
from isaacsim.core.api import SimulationContext
R=Path(__file__).resolve().parents[1];D=R/'scenes/fq/airport_preview';config=json.loads((D/'interaction.json').read_text());results=[]
for gate,y in [('entry',-1.8),('exit',3.65)]:
 for state in ['closed','open','reclosed']:
  omni.usd.get_context().open_stage(str(D/'scene.usdc'));app.update();st=omni.usd.get_context().get_stage();ctrl=usd_belts(st,config)
  ball=UsdGeom.Sphere.Define(st,'/World/TestProbe');ball.CreateRadiusAttr(.05);ball.AddTranslateOp().Set(Gf.Vec3d(0,y-.4,.98));p=ball.GetPrim();UsdPhysics.CollisionAPI.Apply(p);rb=UsdPhysics.RigidBodyAPI.Apply(p);rb.CreateVelocityAttr(Gf.Vec3f(0,1,0));UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.1);PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)
  sim=SimulationContext(physics_dt=1/240,rendering_dt=1/60,stage_units_in_meters=1);sim.initialize_physics();sim.play()
  if state!='closed':ctrl.command(gate,True,release_authorized=True);ctrl.step(1.2)
  if state=='reclosed':ctrl.command(gate,False);ctrl.step(1.2)
  for _ in range(240):sim.step(render=False)
  pos=list(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation())
  assert (pos[1]>y+.5 if state=='open' else pos[1]<y),(gate,state,pos)
  results.append({'test':gate+'_'+state,'passed':True,'position':pos});sim.stop();SimulationContext.clear_instance()
  print('PHYSX_GATE',gate,state,pos,flush=True)
omni.usd.get_context().open_stage(str(D/'operable.usda'));app.update();st=omni.usd.get_context().get_stage();p=st.GetPrimAtPath('/World/MovableSwab');assert p
sim=SimulationContext(physics_dt=1/240,rendering_dt=1/60,stage_units_in_meters=1);sim.initialize_physics();sim.play()
for _ in range(480):sim.step(render=False)
pos=list(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation());assert abs(pos[2]-.7925)<.01,pos
results.append({'test':'rigid_swab_supported_on_tray','passed':True,'position':pos});sim.stop()
_,_,missing=UsdUtils.ComputeAllDependencies(str(D/'operable.usda'));assert not missing,missing
rp=R/'validation/airport_operable_v030.json';r=json.loads(rp.read_text());r['physx_tests']=results;r['usd_unresolved_dependencies']=list(missing);rp.write_text(json.dumps(r,indent=2)+'\n');print('PHYSX_COMPLETE',flush=True);app.close()
