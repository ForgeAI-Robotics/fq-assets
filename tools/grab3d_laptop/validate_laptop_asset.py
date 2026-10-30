from pathlib import Path
import json,math,copy,xml.etree.ElementTree as E,numpy as np,mujoco,imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
R=Path('/assets');D=R/'assets/third_party/office/grab3d_laptop';V=R/'validation/previews';report=json.loads((D/'validation.json').read_text());r=E.parse(D/'model.xml').getroot()
for e in r.findall('.//*[@file]'):e.set('file',str(D/e.get('file')))
def fixed():
 t=copy.deepcopy(r);b=t.find('worldbody/body');b.remove(b.find('freejoint'));t.remove(t.find('keyframe'));return t
m=mujoco.MjModel.from_xml_string(E.tostring(r,encoding='unicode'));d=mujoco.MjData(m);jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'lid_hinge');qa=m.jnt_qposadr[jid];dof=m.jnt_dofadr[jid];lid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'lid');assert np.allclose(m.jnt_range[jid],[0,math.radians(140)])
depth=[];heights=[]
for deg in np.linspace(0,140,141):
 d.qpos[qa]=math.radians(deg);mujoco.mj_forward(m,d);depth.extend(float(c.dist) for c in d.contact)
assert not depth or min(depth)>-1e-6, min(depth)
report['angle_sweep']=dict(samples=141,min_contact_distance_m=min(depth) if depth else None,no_penetration=True)
u=mujoco.MjModel.from_xml_path(str(D/'model.urdf'));ud=mujoco.MjData(u);ulid=mujoco.mj_name2id(u,mujoco.mjtObj.mjOBJ_BODY,'lid');assert u.njnt==1
for deg in [0,55,110,140]:
 d.qpos[qa]=math.radians(deg);ud.qpos[0]=math.radians(deg);mujoco.mj_forward(m,d);mujoco.mj_forward(u,ud);assert np.allclose(d.xpos[lid],ud.xpos[ulid],atol=1e-6) and abs(np.dot(d.xquat[lid],ud.xquat[ulid]))>1-1e-6
report['urdf_mjcf_pose_equivalence']=dict(angles_degrees=[0,55,110,140],passed=True)
for sign in [-1,1]:
 t=fixed();fm=mujoco.MjModel.from_xml_string(E.tostring(t,encoding='unicode'));fd=mujoco.MjData(fm);fd.qpos[0]=.05 if sign<0 else math.radians(140)-.05;fd.qfrc_applied[0]=sign*1.
 for _ in range(1000):mujoco.mj_step(fm,fd)
 assert -.02<fd.qpos[0]<math.radians(140)+.02 and np.isfinite(fd.qpos).all(),fd.qpos
report['forced_joint_limits']=dict(applied_torque_Nm=1.,duration_s=1.,passed=True)
t=copy.deepcopy(r);E.SubElement(t.find('worldbody'),'geom',name='floor',type='plane',size='1 1 .01');t.find('worldbody/body').set('pos','0 0 .03');dm=mujoco.MjModel.from_xml_string(E.tostring(t,encoding='unicode'));dd=mujoco.MjData(dm)
for _ in range(2000):mujoco.mj_step(dm,dd)
assert np.isfinite(dd.qpos).all() and dd.ncon>0 and -.005<dd.qpos[2]<.02
report['free_body_drop']=dict(start_height_m=.03,duration_s=2.,final_z_m=float(dd.qpos[2]),contact_count=dd.ncon,passed=True)
# A physical probe hits the front of the fixed screen and creates a contact response.
t=fixed();w=t.find('worldbody');b=E.SubElement(w,'body',name='probe',pos='0 -.02 .13');E.SubElement(b,'freejoint');E.SubElement(b,'geom',name='probe',type='sphere',size='.012',mass='.02');t.find('option').set('gravity','0 0 0');pm=mujoco.MjModel.from_xml_string(E.tostring(t,encoding='unicode'));pd=mujoco.MjData(pm);pd.qvel[2]=.5;found=False
for _ in range(600):
 mujoco.mj_step(pm,pd)
 for c in pd.contact:
  names=[mujoco.mj_id2name(pm,mujoco.mjtObj.mjOBJ_GEOM,int(i)) for i in [c.geom1,c.geom2]]
  if 'probe' in names and 'lid_collision' in names:found=True
assert found,'Probe did not touch screen';report['screen_probe_contact']=dict(passed=True,probe_radius_m=.012)
# Position actuator exists only in this fixed-base demonstration rig.
t=fixed();act=E.SubElement(t,'actuator');E.SubElement(act,'position',name='demo_servo',joint='lid_hinge',kp='20',kv='.8',ctrlrange='0 '+str(math.radians(140)),forcerange='-2 2');vis=E.SubElement(t,'visual');E.SubElement(vis,'global',offwidth='1920',offheight='1080');E.SubElement(vis,'headlight',ambient='.4 .4 .4',diffuse='.4 .4 .4');E.SubElement(vis,'quality',shadowsize='2048',offsamples='4');w=t.find('worldbody');E.SubElement(w,'geom',type='plane',size='2 2 .01',pos='0 0 -.001',rgba='.18 .23 .30 1');E.SubElement(w,'light',pos='-.3 -.5 1',dir='.1 .3 -1',diffuse='.5 .5 .5');mm=mujoco.MjModel.from_xml_string(E.tostring(t,encoding='unicode'));dd=mujoco.MjData(mm);dd.qpos[0]=0;dd.ctrl[0]=0;ren=mujoco.Renderer(mm,height=1080,width=1920);cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.08];cam.distance=.65;cam.azimuth=55;cam.elevation=-27
font=ImageFont.truetype('/project/assets/fonts/NotoSansCJK-Regular.ttc',37);small=ImageFont.truetype('/project/assets/fonts/NotoSansCJK-Regular.ttc',26);writer=imageio_ffmpeg.write_frames(str(V/'grab3d_laptop_open_close.mp4'),(1920,1080),fps=30,codec='libx264',macro_block_size=1,output_params=['-crf','20','-movflags','+faststart']);writer.send(None);maxerr=0.
for frame in range(240):
 tm=frame/30
 if tm<1:target=0
 elif tm<3:target=110*(.5-.5*math.cos(math.pi*(tm-1)/2))
 elif tm<4:target=110
 elif tm<6:target=110*(.5+.5*math.cos(math.pi*(tm-4)/2))
 else:target=0
 dd.ctrl[0]=math.radians(target)
 for _ in range(33 if frame%3 else 34):mujoco.mj_step(mm,dd)
 assert np.isfinite(dd.qpos).all();err=abs(math.degrees(dd.qpos[0])-target)
 if tm>3.5 and tm<4:maxerr=max(maxerr,err)
 ren.update_scene(dd,camera=cam);im=Image.fromarray(ren.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1920,126),fill='#111b27');draw.text((30,15),'可开合笔记本电脑  |  MuJoCo 物理关节演示',font=font,fill='white');draw.text((30,69),f'当前角度 {math.degrees(dd.qpos[0]):.1f}°  ·  0–140° 限位  ·  固定底座 + 临时伺服，非机器人抓取',font=small,fill='#adc4d5');writer.send(np.asarray(im))
 if frame==119:im.save(V/'third_party_office_grab3d_laptop.png')
writer.close();ren.close();assert maxerr<3.,maxerr;report['driven_open_close']=dict(duration_s=8.,hold_error_degrees=maxerr,passed=True,scope='temporary position servo on fixed base; asset remains passive')
(D/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print('LAPTOP_VALIDATED',report,flush=True)
