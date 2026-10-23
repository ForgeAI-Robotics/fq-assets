"""Isolated armrest adjustment and physical table insertion test."""
from pathlib import Path
import xml.etree.ElementTree as E,copy,json,sys,subprocess
import numpy as np
import mujoco as mj
r=Path(sys.argv[1]);root=E.parse(r/'chair_adjustable.xml').getroot()
for a in root.findall('./asset/mesh'):a.set('file',str(r/a.get('file')))
world=root.find('worldbody');E.SubElement(world,'geom',name='floor',type='plane',size='8 8 .1',contype='2',conaffinity='1',friction='.9 .00002 .0001',rgba='.45 .47 .45 1')
td=Path(sys.argv[2]);table=E.parse(td/'table.xml').getroot()
for a in table.find('asset'):
 if a.get('file'):a.set('file',str(td/a.get('file')))
 root.find('asset').append(copy.deepcopy(a))
tb=E.SubElement(world,'body',name='conference_table',pos='-2.25568199 -11.64090948 0')
for g in table.find('worldbody'):
 if g.get('contype','1')!='0':g.set('contype','2');g.set('conaffinity','1')
 tb.append(copy.deepcopy(g))
E.SubElement(world,'light',pos='-2 1 4',dir='.2 -.3 -1',diffuse='.9 .9 .9');E.SubElement(world,'light',pos='2 -3 3',dir='-.2 .2 -1',diffuse='.5 .5 .5',castshadow='false')
visual=E.SubElement(root,'visual');E.SubElement(visual,'global',offwidth='800',offheight='600');E.SubElement(visual,'headlight',ambient='.35 .35 .35')
E.indent(root);E.ElementTree(root).write(r/'chair_table_insert_scene.xml',encoding='unicode')
m=mj.MjModel.from_xml_path(str(r/'chair_table_insert_scene.xml'));d=mj.MjData(m)
uid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'chair_upper');jids=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'armrest_'+s+'_lift') for s in ['left','right']]
qads=m.jnt_qposadr[jids];sid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'seat_swivel');sd=m.jnt_dofadr[sid];sq=m.jnt_qposadr[sid]
render='--render' in sys.argv
if render:
 from PIL import Image,ImageDraw,ImageFont
 font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
 renderer=mj.Renderer(m,480,640);cam=mj.MjvCamera();cam.lookat[:]=[0,-.7,.57];cam.distance=3.6;cam.azimuth=-35;cam.elevation=-14;opt=mj.MjvOption();opt.geomgroup[3]=0
 pipe=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','640x480','-r','15','-i','-','-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(r/'chair_table_insert.mp4')],stdin=subprocess.PIPE)
frames=[];recorded=[];contacts=[];max_floor_pen=0.;maxforce=0.;max_tilt=0.
for step in range(12000):
 t=step*.001;alpha=np.clip((t-.6)/1.6,0,1);alpha=alpha*alpha*(3-2*alpha);d.ctrl[:]=np.clip(-.04*alpha+.5*9.81/8000,-.06,0)
 d.qfrc_applied[:]=0;target=-.78*np.clip((t-3)/5,0,1);vy=-.156 if 3<t<8 else 0
 # Force-controlled push/braking at base axis, no prescribed chair pose.
 f=np.array([np.clip(-45*d.qpos[0]-20*d.qvel[0],-8,8),np.clip(70*(target-d.qpos[1])+25*(vy-d.qvel[1]),-12,12),0.])
 if t<3:f[:]=0
 maxforce=max(maxforce,float(np.linalg.norm(f)));mj.mj_applyFT(m,d,f,np.zeros(3),np.array([d.qpos[0],d.qpos[1],.65]),uid,d.qfrc_applied)
 # Straight insertion fixture: small yaw stabilizing torque, explicitly reported.
 d.qfrc_applied[sd]+=np.clip(-2*d.qpos[sq]-.4*d.qvel[sd],-.5,.5)
 mj.mj_step(m,d)
 max_tilt=max(max_tilt,float(np.degrees(np.arccos(np.clip(d.xmat[1].reshape(3,3)[2,2],-1,1)))))
 for c in d.contact:
  ns=[mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in [c.geom1,c.geom2]]
  if any('meeting_table' in n for n in ns):contacts.append({'time':t,'geoms':ns,'penetration_m':max(0,-c.dist)})
  if 'floor' in ns:max_floor_pen=max(max_floor_pen,-c.dist)
 if step%67==0:
  mj.mj_forward(m,d);frames.append(d.qpos.tolist());recorded.append(d.xpos.copy())
  if render:
   renderer.update_scene(d,camera=cam,scene_option=opt);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,78),fill=(24,32,40));draw.text((12,8),'ARMREST LOWERING 4 cm -> TABLE INSERTION',font=font,fill='white');draw.text((12,31),'Armrest: %.1f cm | chair travel: %.1f cm'%(-d.qpos[qads].mean()*100,-d.qpos[1]*100),font=font,fill='#d4e0e4');draw.text((12,54),'MuJoCo | force-guided straight push | estimated dimensions',font=font,fill='#ffc36a');pipe.stdin.write(np.asarray(im).tobytes())
   if step in [4020,11926]:im.save(r/('table_insert_'+str(step)+'.png'))
if render:pipe.stdin.close();assert pipe.wait()==0;renderer.close()
report={'armrest_slide_range_m':[-.06,0],'range_status':'simulation_design_not_measured','target_lowering_m':.04,'actual_lowering_m':(-d.qpos[qads]).tolist(),'chair_y_m':float(d.qpos[1]),'table_near_edge_y_m':-.7,'collision_armrest_top_m':float(.725+d.qpos[2]+d.qpos[qads].max()),'table_underside_m':.7021763920784,'table_contact_count':len(contacts),'table_contact_examples':contacts[:5],'max_floor_penetration_mm':max_floor_pen*1000,'max_tilt_deg':max_tilt,'max_push_force_N':maxforce,'method':'Force-guided straight insertion with upper yaw stabilization, not robot contact manipulation','chair_mass_kg':float(m.body_mass.sum()-m.body_mass[mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'conference_table')]),'final_base_speed_m_s':float(np.linalg.norm(d.qvel[:3]))}
report['vertical_clearance_m']=report['table_underside_m']-report['collision_armrest_top_m'];report['passed']=len(contacts)==0 and d.qpos[1]<-.75 and report['vertical_clearance_m']>.01 and max_tilt<5
(r/'table_insert_validation.json').write_text(json.dumps(report,indent=2));(r/'table_insert_qpos.json').write_text(json.dumps(frames));print(json.dumps(report),flush=True)
assert report['passed'],report
replay=mj.MjData(m);max_error=0.
for q,expected in zip(frames,recorded):
 replay.qpos[:]=q;mj.mj_forward(m,replay)
 assert np.isfinite(replay.xpos).all()
 max_error=max(max_error,float(np.max(np.abs(replay.xpos-expected))))
report['replay_frame_count']=len(frames)
report['replay_finite']=True
report['replay_max_position_error_m']=max_error
assert max_error<1e-10
(r/'table_insert_validation.json').write_text(json.dumps(report,indent=2))


