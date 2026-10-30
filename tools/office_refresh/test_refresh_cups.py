from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np,mujoco
O=Path('/assets/assets/office_refresh')
for slug in ['gso_blue_mug','gso_white_mug']:
 D=O/slug;a=json.loads((D/'asset.json').read_text());r=E.parse(D/'model.xml').getroot()
 for e in r.findall('.//*[@file]'):e.set('file',str(D/e.get('file')))
 E.SubElement(r,'option',timestep='.001');w=r.find('worldbody');cup=w.find('body');cup.remove(cup.find('freejoint'));E.SubElement(w,'geom',type='plane',size='1 1 .01')
 b=E.SubElement(w,'body',name='probe',pos='0 0 '+str(a['dimensions_m'][2]+.05));E.SubElement(b,'freejoint');E.SubElement(b,'geom',type='sphere',size='.005',mass='.002')
 m=mujoco.MjModel.from_xml_string(E.tostring(r,encoding='unicode'));d=mujoco.MjData(m)
 for _ in range(2000):mujoco.mj_step(m,d)
 assert .004<d.qpos[2]<a['dimensions_m'][2]*.5 and np.linalg.norm(d.qpos[:2])<.04,(slug,d.qpos[:3])
 v=json.loads((D/'validation.json').read_text());v['cup_cavity_test']=dict(passed=True,probe_radius_m=.005,final_position_m=d.qpos[:3].tolist(),duration_s=2);(D/'validation.json').write_text(json.dumps(v,indent=2));print('CUP_CAVITY_PASS',slug,d.qpos[:3],flush=True)
