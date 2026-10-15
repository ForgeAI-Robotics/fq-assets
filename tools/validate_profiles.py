from pathlib import Path
import json, numpy as np, mujoco
root=Path('/assets');report=[]
index=json.loads((root/'profiles/index.json').read_text())
assets={x['id']:json.loads((root/x['manifest']).read_text()) for x in json.loads((root/'catalog/index.json').read_text())['assets']}
for item in index['profiles']:
 p=json.loads((root/item['path']).read_text());assert p['id']==item['id'];assert len(set(p['joint_order']))==len(p['joint_order'])
 for pose in p['poses'].values():
  q=np.array(pose['position']);assert len(q)==len(p['joint_order']);assert np.isfinite(q).all()
  if p['kind']=='hand_command_presets':assert ((q>=0)&(q<=255)&(q==q.astype(int))).all()
 if p['kind']=='joint_positions':
  for asset in p['compatible_assets']:
   m=mujoco.MjModel.from_xml_path(str(root/assets[asset]['representations']['mujoco']));d=mujoco.MjData(m)
   for name,q in zip(p['joint_order'],p['poses']['initial']['position']):
    j=m.joint(name);assert not m.jnt_limited[j.id] or j.range[0]<=q<=j.range[1],(name,q,j.range)
    d.qpos[j.qposadr[0]]=q
   mujoco.mj_forward(m,d);assert np.isfinite(d.xpos).all()
   saved=d.qpos.copy();d2=mujoco.MjData(m);d2.qpos[:]=saved;mujoco.mj_forward(m,d2)
   assert np.array_equal(d.xpos,d2.xpos)
   report.append({'asset':asset,'joint_names_and_limits':'pass','kinematic_pose_replay_max_error':float(abs(d.xpos-d2.xpos).max()),'balance_or_dynamics_validated':False})
 report.append({'profile':p['id'],'schema_and_units':'pass','hardware_validated':False})
out={'passed':True,'results':report,'scope':'Protocol value validation and G1 named-joint kinematics; no protocol-to-angle calibration, dynamic balance or hardware control'}
Path('/out/profiles_validation.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
