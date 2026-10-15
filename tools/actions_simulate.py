from pathlib import Path
import json,subprocess,hashlib
from collections import deque
import numpy as np,mujoco,onnxruntime as ort
SRC=Path('/source');R=Path('/assets');OUT=Path('/out')
cpp='#include <vector>\n#include <iostream>\n#include "/source/policy_parameters.hpp"\nint main(){std::cout.precision(17);'
fields=['isaaclab_to_mujoco','mujoco_to_isaaclab','default_angles','g1_action_scale','kps','kds']
for n in fields:cpp+='for(auto x:'+n+')std::cout<<x<<" ";std::cout<<"\\n";'
Path('/tmp/constants.cpp').write_text(cpp+'}');subprocess.run(['g++','/tmp/constants.cpp','-o','/tmp/constants'],check=True)
vals=[np.fromstring(x,sep=' ') for x in subprocess.check_output(['/tmp/constants'],text=True).splitlines()]
constants={k:v.tolist() for k,v in zip(fields,vals)};constants.update(control_dt_s=.02,initial_ramp_s=3.,runtime_defaults_only=True)
(OUT/'stand_constants.json').write_text(json.dumps(constants,indent=2))
to_mj,to_lab=vals[0].astype(int),vals[1].astype(int);default,scale,kp,kd=vals[2:]
token=np.fromstring((SRC/'safe_stand_token.txt').read_text(),sep=' ');assert token.shape==(64,)
policy=Path('/policy/model_decoder.onnx');assert hashlib.sha256(policy.read_bytes()).hexdigest()=='34bae8570d4a4421a5391a5c2befd745d4a02d182ec539e5f9da44c091c67509'
opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1;dec=ort.InferenceSession(str(policy),opts,providers=['CPUExecutionProvider'])
names=json.loads((R/'profiles/poses/g1_beyondmimic_initial.json').read_text())['joint_order']
spec=mujoco.MjSpec.from_file(str(R/'assets/fq/robots/g1_o6/robot.xml'));spec.worldbody.add_geom(name='validation_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[4,4,.1],rgba=[.8,.8,.8,1]);m=spec.compile();m.opt.timestep=.002
qa=np.array([m.joint(n).qposadr[0] for n in names]);va=np.array([m.joint(n).dofadr[0] for n in names]);acts=np.array([m.actuator(n).id for n in names])
def run():
 d=mujoco.MjData(m);d.qpos[qa]=default;d.qpos[2]=.80;mujoco.mj_forward(m,d)
 # Place feet close to the plane using collision geom bounds.
 feet=[i for i in range(m.ngeom) if 'ankle_roll' in m.body(int(m.geom_bodyid[i])).name and m.geom_contype[i]]
 bottom=min(d.geom_xpos[i,2]-m.geom_rbound[i] for i in feet);d.qpos[2]-=bottom;d.qpos[2]+=.005
 hist=deque(maxlen=10);last=np.zeros(29);frames=[];stop='completed'
 for i in range(500):
  mujoco.mj_forward(m,d);b=d.body('pelvis');rot=b.xmat.reshape(3,3);entry=(rot.T@b.cvel[:3],(d.qpos[qa]-default)[to_lab],d.qvel[va][to_lab],last.copy(),rot.T@np.array([0,0,-1]))
  if not hist:hist.extend([entry]*10)
  else:hist.append(entry)
  obs=np.concatenate([token]+[np.array([h[k] for h in hist]).reshape(-1) for k in range(5)]).astype(np.float32);assert obs.shape==(994,)
  raw=dec.run(None,{'obs_dict':obs[None]})[0][0];target=default+raw[to_mj]*scale;last=raw.copy()
  for _ in range(10):
   d.ctrl[acts]=np.clip(kp*(target-d.qpos[qa])-kd*d.qvel[va],m.actuator_ctrlrange[acts,0],m.actuator_ctrlrange[acts,1]);mujoco.mj_step(m,d)
  mujoco.mj_forward(m,d);torso=d.body('torso_link');tr=torso.xmat.reshape(3,3)
  wrists={s:(tr.T@(d.body(s+'_wrist_yaw_link').xpos-torso.xpos)).tolist() for s in ['left','right']}
  tilt=float(np.arccos(np.clip(d.body('pelvis').xmat[8],-1,1)));frames.append({'t':round(float(d.time),6),'position':d.qpos[qa].tolist(),'wrists_in_torso_m':wrists,'base_pose':d.qpos[:7].tolist(),'tilt_rad':tilt})
  if not np.isfinite(d.qpos).all() or d.qpos[2]<.45 or tilt>.8:stop='unstable';break
 return frames,stop
frames,stop=run();again,stop2=run();err=float(np.max(np.abs(np.array([f['position'] for f in frames])-np.array([f['position'] for f in again]))))
(OUT/'safe_stand_trace.json').write_text(json.dumps({'joint_order':names,'frames':frames}))
report={'scope':'CPU MuJoCo free-base G1+O6, matched SONIC decoder, constant safe-stand token; no real robot control','frames':len(frames),'duration_s':frames[-1]['t'],'stop':stop,'max_tilt_rad':max(f['tilt_rad'] for f in frames),'replay_max_joint_error_rad':err,'final_wrists_in_torso_m':frames[-1]['wrists_in_torso_m'],'policy_sha256':hashlib.sha256(policy.read_bytes()).hexdigest(),'initialization':'29-joint default pose, feet placed on isolated plane; not full real-robot INIT state machine'}
(OUT/'safe_stand_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
