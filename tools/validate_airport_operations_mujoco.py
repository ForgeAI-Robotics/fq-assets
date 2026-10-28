from pathlib import Path
import xml.etree.ElementTree as E,json,sys
import mujoco,numpy as np
sys.path.insert(0,str(Path(__file__).parent));from airport_interaction import mujoco_belts,SwabControl
R=Path(__file__).resolve().parents[1];D=R/'scenes/fq/airport_preview';out=R/'validation/operation_run';out.mkdir(parents=True,exist_ok=True);results=[]
def absolute(root):
 for e in root.findall('.//*[@file]'):e.set('file',str(D/e.get('file')))
def check(label,condition,**kwargs):
 assert condition,(label,kwargs)
 results.append(dict(test=label,passed=True,**kwargs))
def rejects(label,fn):
 try:fn()
 except (RuntimeError,PermissionError):check(label,True);return
 raise AssertionError(label)
for gate,y in [('entry',-1.8),('exit',3.65)]:
 for state in ['closed','open','reclosed']:
  root=E.parse(D/'scene.xml').getroot();absolute(root);root.find('option').set('gravity','0 0 0');root.find('option').set('timestep','.002');w=root.find('worldbody');b=E.SubElement(w,'body',name='probe',pos=f'0 {y-.4} .98');E.SubElement(b,'freejoint');E.SubElement(b,'geom',name='probe_geom',type='sphere',size='.05',mass='.1')
  m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m);ctrl=mujoco_belts(m)
  if state!='closed':ctrl.command(gate,True,release_authorized=True);ctrl.step(1.2)
  if state=='reclosed':ctrl.command(gate,False);ctrl.step(1.2)
  d.qvel[1]=1.;touch=False
  for _ in range(500):
   mujoco.mj_step(m,d)
   touch|=any(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,c.geom1)==('EntryBelt' if gate=='entry' else 'ReleaseBelt') or mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,c.geom2)==('EntryBelt' if gate=='entry' else 'ReleaseBelt') for c in d.contact)
  check(gate+'_'+state, (d.qpos[1]>y+.5 and not touch) if state=='open' else (d.qpos[1]<y and touch),final_y=float(d.qpos[1]),belt_contact=touch)
m=mujoco.MjModel.from_xml_path(str(D/'operable.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);ctrl=mujoco_belts(m)
rejects('exit_requires_authorization',lambda:ctrl.command('exit',True));rejects('alarm_blocks_open',lambda:ctrl.command('entry',True,alarm=True))
ctrl.command('entry',True);ctrl.step(1.2);rejects('both_gates_open_rejected',lambda:ctrl.command('exit',True,release_authorized=True));rejects('occupied_close_rejected',lambda:ctrl.command('entry',False,occupied=True));ctrl.command('entry',False);ctrl.step(.3);f=ctrl.fraction['entry'];ctrl.step(.1,{'entry':True});check('occupancy_during_closure_reverses',ctrl.fraction['entry']>f)
swab=SwabControl(m,d);rejects('remote_grasp_rejected',swab.grasp);rejects('insertion_without_hold_rejected',swab.accept_insertion)
mid=int(m.body_mocapid[swab.tool]);qadr=int(m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'swab_free')]);dt=float(m.opt.timestep)
for _ in range(250):mujoco.mj_step(m,d)
check('swab_rests_on_tray',abs(d.xpos[swab.body,2]-.7925)<.005,z=float(d.xpos[swab.body,2]))
samples=[];max_error=0.;contacts=set()
def move(target,seconds,phase):
 global max_error
 start=d.mocap_pos[mid].copy();n=max(1,round(seconds/dt))
 for i in range(n):
  u=(i+1)/n;u=u*u*(3-2*u);d.mocap_pos[mid]=start+(np.array(target)-start)*u;mujoco.mj_step(m,d)
  if swab.state in ['held','inserted']:max_error=max(max_error,float(np.linalg.norm(d.site_xpos[swab.grip]-d.xpos[swab.tool])))
  for c in d.contact:
   a=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,c.geom1);b=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,c.geom2)
   if (a or '').startswith('swab_') or (b or '').startswith('swab_'):contacts.add(tuple(sorted([a,b])))
  if i%max(1,round(1/30/dt))==0:samples.append({'t':float(d.time),'phase':phase,'tool':d.mocap_pos[mid].tolist(),'swab':d.qpos[qadr:qadr+7].tolist()})
 print('PHASE',phase,'max_error',max_error,'contacts',sorted(contacts),flush=True)
move([1.24,.95,.7925],1.5,'approach');mujoco.mj_forward(m,d);swab.grasp();check('bilateral_contact_then_grasp',bool(d.eq_active[swab.eq]))
move([1.24,.95,1.15],1.6,'lift');check('lifted_off_tray',d.xpos[swab.body,2]>1.1)
move([1.15,1.58,1.15],2,'transfer');move([1.15,1.58,.8825],1.5,'align');rejects('premature_insertion_rejected',swab.accept_insertion)
move([1.365,1.58,.8825],2,'insert');move([1.365,1.58,.8825],.5,'hold');swab.accept_insertion();check('sample_inserted',swab.state=='inserted',tip_xyz=d.site_xpos[swab.tip].tolist());check('grasp_tracking_error',max_error<.004,max_error_m=max_error)
move([1.15,1.58,.8825],1.5,'withdraw');move([1.24,1.22,1.15],2,'transfer_to_used_tray');move([1.24,1.22,.7925],1.5,'lower_to_used_tray');swab.release();move([1.24,1.22,1.15],1.5,'release_used_swab');check('released_sample_on_separate_used_tray',abs(d.xpos[swab.body,2]-.7925)<.008 and abs(d.xpos[swab.body,1]-1.22)<.008);rejects('used_swab_cannot_be_reused',swab.grasp)
check('full_cycle_grasp_error',max_error<.005,max_error_m=max_error)
bad=[pair for pair in contacts if any(n.startswith('Detector') for n in pair)]
check('no_detector_shell_collision_on_nominal_path',not bad,contacts=bad)
(out/'trajectory.json').write_text(json.dumps({'samples':samples,'physics_dt':dt,'grasp_mode':'bilateral contact checked + explicit weld constraint','robot_dynamics':False},indent=2))
(R/'validation/airport_operable_v030.json').write_text(json.dumps({'tests':results,'swab_contacts':sorted(contacts),'soft_body':False,'chemical_detection':False},indent=2)+'\n')
print(json.dumps(results),flush=True)
