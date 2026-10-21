"""CPU contact validation; cans are never position-clamped or attached to the door."""
from pathlib import Path
import argparse,json,mujoco,numpy as np

def run_case(m,d,seconds=4.,names=('right_door_coke_1','right_door_coke_2'),prefix=''):
 ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)for n in names];assert min(ids)>=0
 door=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,prefix+'refrigerator_door_right');joints=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,prefix+'refrigerator_'+n+'_hinge')for n in ['left','mullion','right']];assert min(joints)>=0
 for b in ids:
  j=m.body_jntadr[b];assert m.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE;assert abs(m.body_mass[b]-.35)<1e-6
 m.opt.timestep=.001;m.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST;m.opt.iterations=100
 duration=2+seconds+5;rows=[];worst=0;max_tilt=0;escaped=False
 for step in range(round(duration/.001)):
  time=step*.001;s=np.clip((time-2)/seconds,0,1);target=np.pi/2*(3*s*s-2*s*s*s)
  for j in joints:
   adr=m.jnt_qposadr[j];v=m.jnt_dofadr[j];goal=target if j==joints[-1]else 0
   d.qfrc_applied[v]=np.clip(1500*(goal-d.qpos[adr])-100*d.qvel[v],-80,80)
  mujoco.mj_step(m,d)
  if step%20==0:
   local=[(d.xpos[b]-d.xpos[door])@d.xmat[door].reshape(3,3)for b in ids];tilts=[float(np.degrees(np.arccos(np.clip(d.xmat[b].reshape(3,3)[2,2],-1,1))))for b in ids];max_tilt=max(max_tilt,*tilts)
   escaped|=any(not(-.293<p[0]<-.051 and .028<p[1]<.140 and .07<p[2]<.20)for p in local)
   rows.append([time,float(d.qpos[m.jnt_qposadr[joints[-1]]]),*np.concatenate(local).tolist(),*tilts])
  for con in d.contact:
   if m.geom_bodyid[con.geom1]in ids or m.geom_bodyid[con.geom2]in ids:worst=max(worst,-float(con.dist))
 assert np.isfinite(d.qpos).all()
 velocities=[float(np.linalg.norm(d.qvel[m.jnt_dofadr[m.body_jntadr[b]]:m.jnt_dofadr[m.body_jntadr[b]]+3]))for b in ids]
 report={'opening_seconds':seconds,'hold_seconds':5,'retained':not escaped,'max_tilt_deg':max_tilt,'max_penetration_m':worst,'final_linear_speeds_m_s':velocities,'final_local_centers_m':rows[-1][2:8],'final_door_deg':float(np.degrees(d.qpos[m.jnt_qposadr[joints[-1]]]))}
 return report,np.array(rows)

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(exist_ok=True,parents=True);reports=[]
 for seconds in (4.,8.):
  m=mujoco.MjModel.from_xml_path(str(a.root/'scenes/fq/office_baseline/scene.xml'));d=mujoco.MjData(m);report,rows=run_case(m,d,seconds);reports.append(report);np.save(a.out/f'opening_{seconds}.npy',rows);print(report,flush=True)
 (a.out/'dynamic_door_cokes.json').write_text(json.dumps({'engine':mujoco.__version__,'cases':reports},indent=2))
 assert all(x['retained'] and x['max_tilt_deg']<15 and x['max_penetration_m']<.002 and max(x['final_linear_speeds_m_s'])<.01 for x in reports),reports
