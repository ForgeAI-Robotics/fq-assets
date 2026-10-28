from pathlib import Path
import json,xml.etree.ElementTree as E
import mujoco,numpy as np
from PIL import Image,ImageDraw
R=Path(__file__).resolve().parents[2];out=Path('/tmp/fq-airport-validation/fuel_bags');out.mkdir(parents=True,exist_ok=True);tiles=[]
for D in sorted((R/'assets/third_party/bags').iterdir()):
 if not D.name.startswith('fuel_openrobotics_'):continue
 path=D/'model.xml';root=E.parse(path).getroot()
 for texture in root.findall('asset/texture'):
  f=D/texture.get('file')
  if f.suffix.lower()!='.png':
   dest=f.with_suffix('.png');Image.open(f).convert('RGB').save(dest);texture.set('file',dest.name)
 E.ElementTree(root).write(path,encoding='utf8',xml_declaration=True)
 for e in root.findall('.//*[@file]'):e.set('file',str(D/e.get('file')))
 E.SubElement(root.find('worldbody'),'geom',name='test_floor',type='plane',size='2 2 .05',rgba='.85 .86 .88 1')
 m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m);d.qpos[2]=.3
 for _ in range(1500):mujoco.mj_step(m,d)
 assert np.isfinite(d.qpos).all() and d.ncon>0 and d.qpos[2]>-.15 and d.qpos[2]<1,(D.name,d.qpos,d.ncon)
 report=json.loads((D/'validation.json').read_text());report.update(mujoco_load=True,drop_test={'steps':1500,'contacts':d.ncon,'root_position_m':d.qpos[:3].tolist(),'finite':True},soft_body_validated=False)
 (D/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
 mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d);m.vis.headlight.ambient[:]=.5;m.vis.headlight.diffuse[:]=.7
 cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,report['dimensions_m'][2]/2];cam.distance=max(report['dimensions_m'])*2.4;cam.azimuth=110;cam.elevation=-15
 ren=mujoco.Renderer(m,height=480,width=500);ren.update_scene(d,camera=cam);im=Image.fromarray(ren.render());ren.close()
 aid=json.loads((D/'asset.json').read_text())['id'];dest=R/'validation/previews'/(aid.replace(':','_').replace('/','_')+'.png');im.save(dest)
 tile=Image.new('RGB',(500,530),'white');tile.paste(im,(0,0));ImageDraw.Draw(tile).text((10,490),D.name.replace('fuel_openrobotics_','')[:55],fill='black');tiles.append(tile)
 print('VALIDATED',D.name,flush=True)
grid=Image.new('RGB',(1500,1060),'white')
for i,t in enumerate(tiles):grid.paste(t,((i%3)*500,(i//3)*530))
grid.save(out/'bags_mujoco.png')
