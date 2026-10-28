from pathlib import Path
import json,xml.etree.ElementTree as E,copy,os
import mujoco,numpy as np,imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[2];F=R/'assets/fq/flexible_sampling_sheet';B=R/'assets/fq/swab_supply_box';O=Path('/tmp/fq-airport-validation/soft_assets');O.mkdir(parents=True,exist_ok=True)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
def record(path,mode):
 m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);mujoco.mj_forward(m,d);initial=d.flexvert_xpos.copy();edge=m.flex_edge.reshape(-1,2);rest=m.flexedge_length0.copy();eqids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,'clamp_'+str(j)) for j in range(5)];maxstrain=0.;contacts=set();maxbend=0.;minz=1.;frames=[];capture=[]
 m.vis.global_.offwidth=1280;m.vis.global_.offheight=720;m.vis.headlight.ambient[:]=.4;m.vis.headlight.diffuse[:]=.6;ren=mujoco.Renderer(m,width=1280,height=720);cam=mujoco.MjvCamera();cam.lookat[:]=[.015,0,.065] if mode=='bend' else [.07,0,.145];cam.distance=.48 if mode=='bend' else .85;cam.azimuth=115;cam.elevation=-30
 opt=mujoco.MjvOption();opt.flags[mujoco.mjtVisFlag.mjVIS_FLEXFACE]=1;opt.flags[mujoco.mjtVisFlag.mjVIS_FLEXEDGE]=0;opt.flags[mujoco.mjtVisFlag.mjVIS_FLEXVERT]=0
 dt=m.opt.timestep;duration=4. if mode=='bend' else 5.;stride=round(1/30/dt)
 for i in range(round(duration/dt)):
  t=i*dt
  if mode=='bend':
   if t<1:phase='Clamp one edge: gravity sag'
   elif t<2:phase='Move onto curved target';d.mocap_pos[0]=[-.085+.095*(t-1),0,.12-.03*(t-1)]
   elif t<2.8:phase='Flexible sheet in contact'
   else:phase='Release and settle';d.eq_active[eqids]=0;d.mocap_pos[0,2]=.17
  else:
   d.ctrl[0]=1.65
   if t<1:phase='Open lid; top sheet remains in box'
   elif t<1.01:
    phase='Grip top sheet edge';assert np.linalg.norm(d.flexvert_xpos[:5].mean(0)-d.mocap_pos[0])<.004;d.eq_active[eqids]=1
   elif t<2:
    phase='Lift flexible top sheet';u=(t-1.01)/.99;d.mocap_pos[0,2]=.0252+.28*(u*u*(3-2*u))
   elif t<3:
    phase='Transfer sheet out of supply box';u=t-2;d.mocap_pos[0,0]=-.085+.26*(u*u*(3-2*u))
   elif t<3.5:phase='Hold: sheet bends under gravity'
   else:phase='Release over separate collection area';d.eq_active[eqids]=0;d.mocap_pos[0,2]=.22
  mujoco.mj_step(m,d);assert np.isfinite(d.qpos).all()
  for c in d.contact:
   for gid in [c.geom1,c.geom2]:
    if gid>=0:contacts.add(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,gid))
  if i%20==0:
   pts=d.flexvert_xpos.copy();lens=np.linalg.norm(pts[edge[:,1]]-pts[edge[:,0]],axis=1);strain=float(np.max(np.abs(lens/rest-1)))
   if strain>maxstrain:maxstrain=strain;worst={'time':t,'phase':phase,'zmin':float(pts[:,2].min()),'lid':float(d.qpos[0]),'contacts':d.ncon}
   minz=min(minz,float(pts[:,2].min()))
   # Rigid Procrustes residual: nonzero residual proves non-rigid deformation, not whole-object motion.
   a=initial-initial.mean(0);b=pts-pts.mean(0);u,_,vh=np.linalg.svd(a.T@b);fit=a@(u@vh);maxbend=max(maxbend,float(np.sqrt(np.mean(np.sum((fit-b)**2,axis=1)))))
  if i%stride==0:
   ren.update_scene(d,camera=cam,scene_option=opt);im=Image.fromarray(ren.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,68),fill=(22,30,38));draw.text((16,8),phase,font=font,fill='white');draw.text((16,37),'Native MuJoCo 2D flex | 90 vertices | 0.4 mm sheet | Material parameters not calibrated',font=font,fill=(210,220,230));frames.append(np.array(im));capture.append({'time':float(d.time),'points':d.flexvert_xpos.tolist(),'clamp':d.mocap_pos[0].tolist()})
   if i in range(round(2.5/dt),round(2.5/dt)+stride):im.save(O/(mode+'_preview.png'))
 ren.close();print('WORST',mode,maxstrain,worst,flush=True);assert maxbend>.002,(mode,maxbend);assert maxstrain<.02,(mode,maxstrain);assert not any(d.eq_active[eqids]);assert minz>-.003,(mode,minz)
 if mode=='bend':assert 'curved_contact_target' in contacts,contacts
 else:assert d.qpos[0]>1.5;assert d.flexvert_xpos[:,0].mean()>.1
 report={'native_flex_count':m.nflex,'vertices':m.nflexvert,'degrees_of_freedom':m.nv,'duration_s':duration,'max_edge_strain':maxstrain,'nonrigid_deformation_rms_m':maxbend,'min_world_z_m':minz,'contact_geometries':sorted(contacts),'released':True,'finite':True};print(mode,report,flush=True)
 return frames,capture,report
root=E.parse(F/'bend_contact_demo.xml').getroot();root.find('worldbody/geom[@name="curved_contact_target"]').set('pos','.035 0 .035');E.ElementTree(root).write(F/'bend_contact_demo.xml',encoding='utf8',xml_declaration=True)
box=E.parse(B/'model.xml').getroot();box.find('option').set('timestep','.0002');box.find('option').set('integrator','implicitfast');box.find('option').set('iterations','100');w=box.find('worldbody');E.SubElement(w,'geom',name='collection_table',type='plane',size='.5 .3 .01',rgba='.2 .26 .32 1')
default=E.SubElement(box,'default');E.SubElement(default,'geom',solref='.0004 1',solimp='.98 .999 .0001')
flexroot=E.parse(F/'bend_contact_demo.xml').getroot();f=copy.deepcopy(flexroot.find('worldbody/flexcomp'));f.set('pos','0 0 .0252');w.append(f);clamp=copy.deepcopy(flexroot.find('worldbody/body'));clamp.set('pos','-.085 0 .0252');w.append(clamp);eq=copy.deepcopy(flexroot.find('equality'))
for x in eq:x.set('active','false')
box.append(eq)
for a in flexroot.find('asset'):
 a=copy.deepcopy(a)
 if a.get('file'):a.set('file',os.path.relpath(F/a.get('file'),B))
 box.find('asset').append(a)
for i in range(9):E.SubElement(w,'geom',name='stock_sheet_'+str(i),type='box',pos=f'0 0 { .0202+i*.00055}',size='.085 .015 .0002',rgba=f'{.88+i*.01} {.88+i*.01} {.84+i*.01} 1')
E.indent(box);E.ElementTree(box).write(B/'dispensing_demo.xml',encoding='utf8',xml_declaration=True)
frames1,cap1,r1=record(F/'bend_contact_demo.xml','bend');frames2,cap2,r2=record(B/'dispensing_demo.xml','dispense')
writer=imageio_ffmpeg.write_frames(str(O/'flexible_sheet_and_dispenser.mp4'),(1280,720),fps=20,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',output_params=['-crf','18','-preset','fast','-movflags','+faststart']);writer.send(None)
for im in frames1+frames2:writer.send(im)
writer.close();(F/'validation.json').write_text(json.dumps(r1,indent=2)+'\n');(B/'validation.json').write_text(json.dumps(r2,indent=2)+'\n');(O/'soft_frames.json').write_text(json.dumps({'bend':cap1,'dispense':cap2},indent=2));print('SOFT_SUPPLY_VALIDATED',flush=True)
