import mujoco as mj,numpy as np,json,sys
from pathlib import Path
r=Path(sys.argv[1]);m=mj.MjModel.from_xml_path(str(r/'chair_dynamic_test.xml'))
names=[mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,i) for i in range(1,m.nbody)]
upper=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'chair_upper');jid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'seat_swivel');qadr=m.jnt_qposadr[jid]
results={};replays={}
def run(kind,record=True):
 d=mj.MjData(m)
 for _ in range(2000):mj.mj_step(m,d)
 start=d.qpos.copy();frames=[];states=[];max_pen=0.;max_tilt=0.;forces=[]
 for step in range(7000):
  t=step*.001;d.qfrc_applied[:]=0;force=np.zeros(3);torque=np.zeros(3)
  if .5<=t<2.5:
   if kind in ['push','combined']:force[0]=4.
   if kind=='side_push':force[1]=-4.
   if kind in ['swivel','combined']:torque[2]=.6
  mj.mj_applyFT(m,d,force,torque,d.xipos[upper],upper,d.qfrc_applied)
  mj.mj_step(m,d)
  if not np.isfinite(d.qpos).all():raise RuntimeError('Non-finite state')
  max_pen=max(max_pen,max([-c.dist for c in d.contact],default=0));max_tilt=max(max_tilt,float(np.arccos(np.clip(d.xmat[1].reshape(3,3)[2,2],-1,1))))
  if step%33==0:
   mj.mj_forward(m,d);frames.append(np.concatenate([d.xpos[1:],d.xquat[1:]],axis=1).round(7).tolist());states.append(d.qpos.copy().tolist())
 end=d.qpos.copy();wheel=[i for i in range(m.njnt) if (mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,i) or '').startswith('wheel_roll')];caster=[i for i in range(m.njnt) if (mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,i) or '').startswith('caster_swivel')]
 info={'displacement_m':float(np.linalg.norm(end[:2]-start[:2])),'seat_rotation_deg':float(np.degrees(end[qadr]-start[qadr])),'max_penetration_mm':max_pen*1000,'max_base_tilt_deg':float(np.degrees(max_tilt)),'max_wheel_rotation_deg':max(abs(float(np.degrees(end[m.jnt_qposadr[i]]-start[m.jnt_qposadr[i]]))) for i in wheel),'max_caster_rotation_deg':max(abs(float(np.degrees(end[m.jnt_qposadr[i]]-start[m.jnt_qposadr[i]]))) for i in caster),'final_speed_m_s':float(np.linalg.norm(d.qvel[:3])),'force_N':4 if kind in ['push','side_push','combined'] else 0,'torque_Nm':.6 if kind in ['swivel','combined'] else 0}
 # State playback must recreate every exported body transform.
 replay_error=0.
 if record:
  dd=mj.MjData(m)
  for q,frame in zip(states,frames):
   dd.qpos[:]=q;mj.mj_forward(m,dd);replay_error=max(replay_error,float(np.max(np.abs(dd.xpos[1:]-np.asarray(frame)[:,:3]))))
 info['state_replay_max_position_error_m']=replay_error
 return info,{'dt':.033,'bodies':names,'frames':frames},end
for kind in ['rest','swivel','push','side_push','combined']:
 info,clip,end=run(kind);again,_,end2=run(kind,False);info['repeat_max_qpos_error']=float(np.max(np.abs(end-end2)));results[kind]=info;replays[kind]=clip;print(kind,json.dumps(info),flush=True)
assert all(x['max_base_tilt_deg']<10 and x['max_penetration_mm']<2 and x['repeat_max_qpos_error']<1e-8 for x in results.values()),results
assert results['rest']['displacement_m']<.005
assert abs(results['swivel']['seat_rotation_deg'])>30
assert results['push']['displacement_m']>.1 and results['push']['max_wheel_rotation_deg']>90
(r/'dynamics_validation.json').write_text(json.dumps({'engine':'MuJoCo '+mj.__version__,'nq':m.nq,'nv':m.nv,'total_mass_kg':float(m.body_mass.sum()),'tests':results,'passed':True,'scope':'Unoccupied chair on flat plane; simulated force/torque; not robot contact or PhysX validation'},indent=2))
(r/'dynamics_replays.json').write_text(json.dumps(replays,separators=(',',':')))
print('ALL_CHECKS_PASS',flush=True)

