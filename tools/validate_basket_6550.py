from pathlib import Path
import json, numpy as np, mujoco
from PIL import Image
from pxr import Usd,UsdGeom,UsdPhysics
r=Path(__file__).resolve().parents[1];b=r/'assets/fq/photo_basket';out=r/'validation';p=json.loads((b/'parameters.json').read_text());g=json.loads((b/'geometry.json').read_text())
v=np.array(g['visuals']['basket_body']['vertices']);extent=np.ptp(v,axis=0)
assert np.allclose(extent,[.28,.19,.165],atol=.0002),extent
m=mujoco.MjModel.from_xml_path(str(b/'test_mujoco.xml'));d=mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m,d,0);maxpen=0.;warnings=np.zeros(len(d.warning),int)
for i in range(5000):
 mujoco.mj_step(m,d);assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
 if d.ncon:maxpen=max(maxpen,max(-c.dist for c in d.contact))
warnings+=np.array([w.number for w in d.warning])
bid=m.body('basket').id;pid=m.body('payload').id
local=d.xmat[bid].reshape(3,3).T@(d.xpos[pid]-d.xpos[bid]);speed=np.linalg.norm(d.qvel[-6:-3])
assert abs(local[0])<.10 and abs(local[1])<.06 and .02<local[2]<.06 and speed<.03,(local,speed)
settled=d.qpos.copy();sweeps=[]
for name in ['handle_right','handle_left']:
 mujoco.mj_resetDataKeyframe(m,d,0);j=m.joint(name+'_hinge');q=j.qposadr[0];vel=j.dofadr[0];seen=[]
 for i in range(3600):
  target=p['joint_initial']+(2.35-p['joint_initial'])*min(i/1900,1)
  d.qfrc_applied[:]=0;d.qfrc_applied[vel]=np.clip(.16*(target-d.qpos[q])-.012*d.qvel[vel],-.12,.12)
  mujoco.mj_step(m,d);assert np.isfinite(d.qpos).all();seen.append(float(d.qpos[q]))
 warnings+=np.array([w.number for w in d.warning])
 assert max(seen)-min(seen)>1 and min(seen)>-.73 and max(seen)<2.9,seen[-1]
 sweeps.append({'handle':name,'minimum_rad':min(seen),'maximum_rad':max(seen),'torque_driven':True})
assert not warnings.any(),warnings
assert maxpen<.005,maxpen
stage=Usd.Stage.Open(str(b/'basket.usd'));assert len([x for x in stage.Traverse() if x.IsA(UsdPhysics.RevoluteJoint)])==2
uv=np.array(UsdGeom.Mesh(stage.GetPrimAtPath('/Basket/basket_body/Visual')).GetPointsAttr().Get());assert np.allclose(np.ptp(uv,axis=0),extent,atol=1e-6)
renderer=mujoco.Renderer(m,height=600,width=800);opt=mujoco.MjvOption();opt.geomgroup[3]=0
cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.12];cam.distance=.65;cam.azimuth=125;cam.elevation=-27
d.qpos[:]=settled;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(renderer.render()).save(out/'basket_6550_settled.png')
mujoco.mj_resetDataKeyframe(m,d,0);d.qpos[7:9]=1.55;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(renderer.render()).save(out/'basket_6550_handles.png');renderer.close()
report={'passed':True,'body_dimensions_m':extent.tolist(),'nominal_dimensions_m':[.28,.19,.165],'height_excludes_handles':True,'bottom_estimated_m':[.24,.15],'corner_radius_estimated_m':.028,'payload_local_position_m':local.tolist(),'payload_speed_m_s':float(speed),'maximum_drop_penetration_m':maxpen,'joint_sweeps':sweeps,'warnings':warnings.tolist(),'usd_matches_mujoco_geometry':True,'isaac_physx_runtime_tested':False,'cpu_only':True,'mass_friction_measured':False}
(out/'basket_6550_20261014.json').write_text(json.dumps(report,indent=2)+'\n')
manifest=json.loads((b/'manifest.json').read_text());manifest['validation']['mujoco_dynamics_passed']=True;manifest['validation']['usd_structure_passed']=True;(b/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(report,indent=2))
