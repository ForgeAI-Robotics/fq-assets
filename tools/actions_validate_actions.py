from pathlib import Path
import json, numpy as np,mujoco
R=Path('/assets');results=[]
models={f:mujoco.MjModel.from_xml_path(str(R/f'assets/fq/robots/{f}/robot.xml')) for f in ['g1_o6','g1_o7']}
for e in json.loads((R/'actions/index.json').read_text())['actions']:
 meta=json.loads((R/e['manifest']).read_text());t=json.loads((R/meta['trajectory']).read_text());q=np.array([f['position'] for f in t['frames']]);times=np.array([f['t'] for f in t['frames']]);assert np.isfinite(q).all() and np.all(np.diff(times)>0);assert q.shape[1]==len(t['joint_order'])
 for robot,m in models.items():
  d=mujoco.MjData(m);violations=[]
  for name,angle in meta['initial_pose'].items():d.joint(name).qpos[0]=angle
  for i,name in enumerate(t['joint_order']):
   j=m.joint(name)
   if m.jnt_limited[j.id] and (q[:,i].min()<j.range[0]-1e-6 or q[:,i].max()>j.range[1]+1e-6):violations.append({'joint':name,'trajectory_range':[float(q[:,i].min()),float(q[:,i].max())],'model_range':j.range.tolist()})
  frames=[]
  for f in t['frames']:
   for name,v in zip(t['joint_order'],f['position']):d.joint(name).qpos[0]=v
   mujoco.mj_forward(m,d);assert np.isfinite(d.xpos).all();frames.append(d.xpos.copy())
  # A fresh data instance must reconstruct the same final transform.
  d2=mujoco.MjData(m);d2.qpos[:]=d.qpos;mujoco.mj_forward(m,d2);err=float(abs(d2.xpos-d.xpos).max())
  results.append({'id':e['id'],'robot':robot,'frames':len(q),'duration_s':float(times[-1]),'limit_violations':violations,'kinematic_replay_error_m':err})
report={'results':results,'passed':not any(r['limit_violations'] for r in results),'hardware_validated':False};Path('/out/actions_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
