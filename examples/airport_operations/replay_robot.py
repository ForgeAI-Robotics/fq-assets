"""Replay the recorded kinematic robot example; this does not run robot dynamics."""
from pathlib import Path
import argparse,json,time
import mujoco,numpy as np
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--headless',action='store_true');a=ap.parse_args();folder=Path(__file__).resolve().parent
 m=mujoco.MjModel.from_xml_path(str(folder/'robot_demo.xml'));d=mujoco.MjData(m);t=json.loads((folder/'robot_trajectory.json').read_text());rows=t['samples']
 def jid(n):return mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)
 ids=[m.jnt_qposadr[jid(n)] for n in t['joint_names']];sw=m.jnt_qposadr[jid('swab_free')];base=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'base_link');tool=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'gripper_interface');mid=m.body_mocapid[tool]
 for n,v in t['fixed_posture'].items():d.qpos[m.jnt_qposadr[jid(n)]]=v
 viewer=None
 if not a.headless:
  from mujoco import viewer as mv
  viewer=mv.launch_passive(m,d);viewer.cam.lookat[:]=[1.05,1.35,.75];viewer.cam.distance=3.2;viewer.cam.azimuth=35;viewer.cam.elevation=-28
 try:
  for i,r in enumerate(rows):
   start=time.monotonic();d.qpos[ids]=r['arm_q'];d.qpos[sw:sw+7]=r['swab'];d.mocap_pos[mid]=r['tool'];m.body_pos[base]=r['base_position'];mujoco.mj_forward(m,d)
   assert np.isfinite(d.xpos).all()
   if viewer:
    if not viewer.is_running():break
    viewer.sync();dt=rows[i+1]['t']-r['t'] if i+1<len(rows) else 0;time.sleep(max(0,dt-(time.monotonic()-start)))
  print('Kinematic replay verified:',len(rows),'frames; no robot dynamics claim')
 finally:
  if viewer:viewer.close()
if __name__=='__main__':main()
