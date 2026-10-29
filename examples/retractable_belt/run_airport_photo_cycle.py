"""Contact-checked native simulation of the complete belt operation cycle."""
from pathlib import Path
import json,sys
import numpy as np,mujoco
R=Path(__file__).resolve().parents[2];sys.path.insert(0,str(R/'tools'))
from retractable_belt import RetractableBelt
import argparse
p=argparse.ArgumentParser();p.add_argument('--output',default='outputs/airport_photo_cycle');p.add_argument('--gate',choices=['entry','exit'],required=True);args=p.parse_args();gate=args.gate
O=Path(args.output)/gate;O.mkdir(parents=True,exist_ok=True)
config=json.loads((R/'scenes/fq/airport_preview/photo_interaction.json').read_text())
m=mujoco.MjModel.from_xml_path(str(R/'scenes/fq/airport_preview/photo_interaction.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);belt=RetractableBelt(m,d,gate+'_',config[gate]['origin']);records=[];rejections=[];done=set();maxstrain=0;maxgrip=0
def reject(name,call):
 try:call()
 except (RuntimeError,PermissionError,ValueError) as e:rejections.append({'test':name,'reason':str(e)})
 else:raise AssertionError('Expected rejection: '+name)
reject('alarm_blocks_open',lambda:belt.begin_open(release_authorized=True,alarm=True))
reject('authorization_required',lambda:belt.begin_open(release_authorized=False))
reject('occupied_blocks_close',lambda:belt.begin_close(occupied=True))
othergate='exit' if gate=='entry' else 'entry';other=RetractableBelt(m,d,othergate+'_',config[othergate]['origin']);d.eq_active[other.latch_eq]=0;reject('other_gate_open_blocks_open',lambda:belt.begin_open(release_authorized=True,other=other));d.eq_active[other.latch_eq]=1
belt.begin_open(release_authorized=True,other=other)
reject('unheld_handle_cannot_unlatch',belt.unlatch)
original=d.mocap_pos[belt.gripper].copy();d.mocap_pos[belt.gripper]+=[0,.1,0];mujoco.mj_forward(m,d);reject('unaligned_gripper_cannot_grasp',belt.grasp);d.mocap_pos[belt.gripper]=original;mujoco.mj_forward(m,d)
def once(name,call):
 if name not in done:call();done.add(name)
def smooth(u):return .5-.5*np.cos(np.pi*np.clip(u,0,1))
def check_locked_feed():
 reject('latched_retraction_rejected',lambda:belt.feed_to(.05,m.opt.timestep))
 d.eq_active[belt.latch_eq]=0;d.eq_active[belt.dock_eq]=1
 try:reject('docked_feed_rejected',lambda:belt.feed_to(.05,m.opt.timestep))
 finally:d.eq_active[belt.latch_eq]=1;d.eq_active[belt.dock_eq]=0
for k in range(int(15.2/m.opt.timestep)):
 t=k*m.opt.timestep;p=belt.seat.copy();slack=0
 if t<.5:phase='LATCHED'
 elif t<1.5:
  phase='GRASP + LIFT';once('grasp',belt.grasp);once('locked_feed',check_locked_feed);once('early_unlatch',lambda:reject('seated_handle_cannot_unlatch',belt.unlatch));u=smooth(t-.5);p[2]+=.05*u;slack=.012*u
 elif t<6.5:
  phase='UNLATCH + REWIND';once('unlatch',belt.unlatch);once('unsafe_release',lambda:reject('unsecured_handle_cannot_release',belt.release));u=smooth((t-1.5)/5);p=belt.seat+[-1.45*u,0,.05*(1-u)];slack=.012
 elif t<7:
  phase='PARK AT REEL';p=belt.park.copy();slack=.012
  if t>=6.7:once('dock',belt.dock);once('release_dock',belt.release)
 elif t<7.6:
  phase='PASSAGE OPEN';p=belt.park+[0,0,.12*smooth((t-7)/.3)];slack=.012
 elif t<8.2:
  phase='REGRASP END';p=belt.park+[0,0,.12*(1-smooth((t-7.6)/.4))];slack=.012
 elif t<13.2:
  phase='PULL OUT';once('close_authorization',lambda:belt.begin_close(occupied=False));once('close_jaws',belt.close_gripper);once('regrasp',belt.grasp);once('undock',belt.undock);once('unaligned_latch',lambda:reject('unseated_handle_cannot_latch',belt.latch));u=smooth((t-8.2)/5);p=belt.park+[1.45*u,0,.05*u];slack=.012
 elif t<14.4:
  phase='ALIGN + LOWER';u=smooth((t-13.2));p=belt.seat+[0,0,.05*(1-u)];slack=.012*(1-u)
 elif t<14.7:
  phase='LATCH VERIFIED';p=belt.seat.copy();once('latch',belt.latch);once('release_latch',belt.release)
 else:phase='CLOSED';p=belt.seat+[0,0,.12*smooth((t-14.7)/.4)]
 d.mocap_pos[belt.gripper]=p
 if d.eq_active[belt.grasp_eq]:belt.feed_to(float(np.clip(belt.LENGTH-np.linalg.norm(p-belt.outlet)-slack,0,1.48)),m.opt.timestep)
 mujoco.mj_step(m,d)
 if d.warning.number.sum() or not np.isfinite(d.qpos).all():raise RuntimeError('Physics instability')
 strain=float(np.max(abs(d.flexedge_length/m.flexedge_length0-1)));maxstrain=max(maxstrain,strain)
 if d.eq_active[belt.grasp_eq]:maxgrip=max(maxgrip,float(np.linalg.norm(d.xpos[belt.handle]-d.mocap_pos[belt.gripper])))
 if k%100==0:records.append({'t':float(d.time),'phase':phase,'stored':belt.stored,'qpos':d.qpos.tolist(),'mocap_pos':d.mocap_pos.tolist(),'eq_active':d.eq_active.tolist(),'gripper_pad_y':float(m.geom_pos[belt.id('geom','gripper_pad_1'),1]),'strain':strain,'handle':d.xpos[belt.handle].tolist(),'contacts':belt.contacts()})
 if k%10000==0:print('CYCLE',round(t,2),phase,'strain',round(strain,5),'handle',d.xpos[belt.handle],flush=True)
assert bool(d.eq_active[belt.latch_eq]) and not d.eq_active[belt.grasp_eq] and not d.eq_active[belt.dock_eq]
assert maxstrain<.03,maxstrain
assert maxgrip<.008,maxgrip
assert np.linalg.norm(d.xpos[belt.handle]-belt.seat)<.005
report={'engine':'MuJoCo '+mujoco.__version__,'cycle_complete':True,'duration_s':float(d.time),'flex_vertices':m.nflexvert,'max_edge_strain':maxstrain,'max_grasp_position_error_m':maxgrip,'warnings':int(d.warning.number.sum()),'negative_tests':rejections,'events':belt.events,'final_latched':True,'scope':'native deformable webbing; prescribed reel guide boundary; explicit checked grasp/latch constraints; not calibrated reel mechanics'}
(O/'cycle_frames.json').write_text(json.dumps({'records':records,'report':report}));(O/'validation.json').write_text(json.dumps(report,indent=2));print('CYCLE_PASS',maxstrain,maxgrip,len(rejections),flush=True)
