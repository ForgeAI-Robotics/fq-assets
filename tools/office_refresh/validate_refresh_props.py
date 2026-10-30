from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
R=Path('/assets');O=R/'assets/office_refresh';V=R/'validation/previews';font=ImageFont.truetype('/project/assets/fonts/NotoSansCJK-Regular.ttc',23);tiles=[];results=[]
def scene(D):
 root=E.parse(D/'model.xml').getroot()
 for a in root.findall('.//*[@file]'):a.set('file',str((D/a.get('file')).resolve()))
 E.SubElement(root,'option',timestep='.002',gravity='0 0 -9.81');vis=E.SubElement(root,'visual');E.SubElement(vis,'global',offwidth='512',offheight='512');E.SubElement(vis,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6');E.SubElement(vis,'quality',shadowsize='2048',offsamples='4');w=root.find('worldbody');E.SubElement(w,'geom',name='floor',type='plane',size='2 2 .01',rgba='.22 .25 .29 1');E.SubElement(w,'light',pos='-.3 -.4 1',dir='.3 .4 -1',diffuse='.6 .6 .6')
 return root
for D in sorted(O.iterdir()):
 a=json.loads((D/'asset.json').read_text());root=scene(D);b=root.find('worldbody/body');b.set('pos','0 0 .10');m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
 for _ in range(1000):mujoco.mj_step(m,d)
 assert np.isfinite(d.qpos).all() and -.02<d.qpos[2]<.2 and d.ncon>0,(D.name,d.qpos[:3],d.ncon)
 validation=json.loads((D/'validation.json').read_text());validation['drop_test']=dict(duration_s=2.,start_height_m=.10,final_position_m=d.qpos[:3].tolist(),contact_count=d.ncon,passed=True);(D/'validation.json').write_text(json.dumps(validation,indent=2));results.append(dict(slug=D.name,**validation['drop_test']))
 # Thumbnail shows the grounded source pose, independent from drop-test motion.
 b.set('pos','0 0 .001');m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);ren=mujoco.Renderer(m,height=512,width=512);cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,a['dimensions_m'][2]/2];cam.distance=max(a['dimensions_m'])*2.25;cam.azimuth=125;cam.elevation=-35;ren.update_scene(d,camera=cam);im=Image.fromarray(ren.render());ren.close();dr=ImageDraw.Draw(im);dr.rectangle((0,462,512,512),fill=(22,28,34));dr.text((15,474),a['name'],font=font,fill='white');im.save(V/(a['id'].replace(':','_').replace('/','_')+'.png'));tiles.append(im);print('OFFICE_DROP_PASS',D.name,flush=True)

sheet=Image.new('RGB',(1536,1536),(22,28,34))
for i,im in enumerate(tiles):sheet.paste(im,((i%3)*512,(i//3)*512))
sheet.save(V/'office_refresh_contact_sheet.png')
