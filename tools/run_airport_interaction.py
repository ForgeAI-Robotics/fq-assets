"""Run a live rigid-swab cycle and belt controls. Optional MuJoCo viewer."""
from pathlib import Path
import argparse,time,json
import mujoco,numpy as np
from airport_interaction import mujoco_belts,SwabControl
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);ap.add_argument('--viewer',action='store_true');args=ap.parse_args()
 m=mujoco.MjModel.from_xml_path(str(args.root/'scenes/fq/airport_preview/operable.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);g=mujoco_belts(m);s=SwabControl(m,d);mid=m.body_mocapid[s.tool];dt=m.opt.timestep;viewer=None
 if args.viewer:
  from mujoco import viewer as mv
  viewer=mv.launch_passive(m,d);viewer.cam.lookat[:]=[1.2,1.3,.8];viewer.cam.distance=3.;viewer.cam.azimuth=30;viewer.cam.elevation=-30
 def tick():
  t=time.monotonic();g.step(dt);mujoco.mj_step(m,d)
  if viewer:
   if not viewer.is_running():raise KeyboardInterrupt
   viewer.sync();time.sleep(max(0,dt-(time.monotonic()-t)))
 def wait(seconds):
  for _ in range(round(seconds/dt)):tick()
 def move(p,seconds):
  start=d.mocap_pos[mid].copy();n=round(seconds/dt)
  for i in range(n):
   u=(i+1)/n;u=u*u*(3-2*u);d.mocap_pos[mid]=start+(np.array(p)-start)*u;tick()
 try:
  g.command('entry',True);wait(1.5);g.command('entry',False,occupied=False);wait(1.5)
  move([1.24,.95,.7925],1.5);mujoco.mj_forward(m,d);s.grasp();move([1.24,.95,1.15],1.6);move([1.15,1.58,1.15],2);move([1.15,1.58,.8825],1.5);move([1.365,1.58,.8825],2);wait(.5);s.accept_insertion()
  move([1.15,1.58,.8825],1.5);move([1.24,1.22,1.15],2);move([1.24,1.22,.7925],1.5);s.release();move([1.24,1.22,1.15],1.5)
  # Explicit synthetic authorization: no chemical measurement is performed.
  g.command('exit',True,release_authorized=True);wait(1.5);g.command('exit',False,occupied=False);wait(1.5)
  print(json.dumps({'entry':g.state('entry'),'exit':g.state('exit'),'sample':s.state,'sample_used':s.used,'authorization':'synthetic demo input','grasp':'bilateral contact followed by rigid constraint'}))
 finally:
  if viewer:viewer.close()
if __name__=='__main__':main()
