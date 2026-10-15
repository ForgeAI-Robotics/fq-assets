from pathlib import Path
import sys,json,hashlib
import numpy as np,mujoco
sys.path.insert(0,'/source');import o7_startup
R=Path('/assets');S=Path('/source');O=Path('/out');O.mkdir(exist_ok=True)
spec=mujoco.MjSpec.from_file(str(R/'assets/fq/robots/g1_o6/robot.xml'));spec.worldbody.add_geom(name='token_validation_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[30,30,.1]);m=spec.compile();m.opt.timestep=.002
profile=json.loads((S/'o7_startup.json').read_text());names=profile['body_joint_names'];qa=np.array([m.joint(n).qposadr[0] for n in names]);results=[];cache={}
for version in ['v3','v4']:
 manifest=json.loads((S/version/'dataset_manifest.json').read_text())
 for key,entry in manifest['built_entries'].items():
  p=S/version/entry['directory']/'frames.npz';digest=hashlib.sha256(p.read_bytes()).hexdigest();slug='sonic_'+version+'_'+key
  if digest in cache:
   report=dict(cache[digest],slug=slug,reused_identical_source=True);results.append(report);print(slug,'reused',flush=True);continue
  with np.load(p,allow_pickle=False) as z:tokens=z['motion_token'].copy();times=z['timestamp_unix'].copy()
  assert tokens.shape==(entry['frame_count'],64) and np.isfinite(tokens).all();assert np.all(np.diff(times)>0)
  d=mujoco.MjData(m);controller,warmup=o7_startup.initialize(m,d,'/policy/model_decoder.onnx');start_token=controller.token.copy()
  # Match navigation's one-second handover, then execute every recorded token once at 50 Hz.
  for i in range(50):
   controller.token=((1-(i+1)/50)*start_token+(i+1)/50*tokens[0]).astype(np.float32)
   for _ in range(10):controller.apply();mujoco.mj_step(m,d)
  frames=[];stop='completed';minimum_z=10;max_tilt=0
  for i,token in enumerate(tokens):
   controller.token=token.astype(np.float32)
   for _ in range(10):controller.apply();mujoco.mj_step(m,d)
   mujoco.mj_forward(m,d);tilt=float(np.degrees(np.arccos(np.clip(d.body('pelvis').xmat[8],-1,1))));minimum_z=min(minimum_z,float(d.qpos[2]));max_tilt=max(max_tilt,tilt)
   frames.append({'t':round(i*.02,6),'position':d.qpos[qa].tolist(),'base_pose':d.qpos[:7].tolist()})
   if not np.isfinite(d.qpos).all() or d.qpos[2]<.45 or tilt>45:stop='unstable';break
  trace={'schema_version':1,'joint_order':names,'units':'radian','root_frame':'pelvis_world_wxyz','frames':frames};raw=json.dumps(trace,separators=(',',':'));(O/(digest+'.json')).write_text(raw)
  report={'slug':slug,'source_sha256':digest,'trace_file':digest+'.json','trace_sha256':hashlib.sha256(raw.encode()).hexdigest(),'frames':len(frames),'expected_frames':len(tokens),'duration_s':frames[-1]['t'],'stop':stop,'min_pelvis_z_m':minimum_z,'max_tilt_deg':max_tilt,'displacement_m':(np.array(frames[-1]['base_pose'][:3])-frames[0]['base_pose'][:3]).tolist(),'warmup':warmup,'source_max_gap_s':float(np.max(np.diff(times))),'hardware_validated':False,'robot':'g1_o6','physics_hz':500,'policy_hz':50,'token_playback':'full recorded sequence, one frame per policy step; not loop sequencer navigation','handover_s':1}
  results.append(report);cache[digest]=report;(O/'simulation.json').write_text(json.dumps(results,indent=2));print(slug,stop,round(minimum_z,3),round(max_tilt,2),flush=True)
(O/'simulation.json').write_text(json.dumps(results,indent=2))
