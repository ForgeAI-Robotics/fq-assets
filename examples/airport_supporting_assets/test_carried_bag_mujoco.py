from pathlib import Path
import xml.etree.ElementTree as E,json
import mujoco,numpy as np
R=Path(__file__).resolve().parents[2]
for D in sorted((R/'assets/fq/passenger_combinations').iterdir()):
 x=E.parse(D/'preview.xml').getroot()
 for e in x.findall('.//*[@file]'):e.set('file',str((D/e.get('file')).resolve()))
 g=x.find('.//geom[@name="carried_bag"]');pos=np.array(list(map(float,g.get('pos').split())));a=json.loads((D/'assembly.json').read_text());bag=R/'assets/third_party/bags'/a['bag_asset'].split('/')[-1];height=json.loads((bag/'asset.json').read_text())['dimensions_m'][2]*a['bag_scale']
 w=x.find('worldbody');E.SubElement(w,'geom',name='test_floor',type='plane',size='3 3 .01');b=E.SubElement(w,'body',name='test_probe',pos=' '.join(map(str,pos+[0,0,height+.12])));E.SubElement(b,'freejoint');E.SubElement(b,'geom',name='test_probe',type='sphere',size='.025',mass='.1');m=mujoco.MjModel.from_xml_string(E.tostring(x,encoding='unicode'));d=mujoco.MjData(m);bagid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'carried_bag');probe=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'test_probe');contacts=0
 for _ in range(1000):
  mujoco.mj_step(m,d);contacts+=sum({c.geom1,c.geom2}=={bagid,probe} for c in d.contact)
 assert contacts>0,(D,'no bag/probe contact');assert np.isfinite(d.qpos).all();v=json.loads((D/'validation.json').read_text());v['mujoco_external_probe_bag_contact']={'contact_steps':contacts,'finite':True};(D/'validation.json').write_text(json.dumps(v,indent=2)+'\n');print('MUJOCO_CONTACT',D.name,contacts,flush=True)
