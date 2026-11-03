from pathlib import Path
import json,shutil,hashlib,struct,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco
R=Path('/assets');P=Path('/project')
def st(v):return ' '.join(map(str,v))
def tf(e):
 o=e.find('origin');return (np.zeros(3),[1,0,0,0]) if o is None else (list(map(float,o.get('xyz','0 0 0').split())),Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_quat()[[3,0,1,2]])
for kind,slug in [('tianji','tianji_robot'),('rhino','rhino_v6_gripper')]:
 D=R/'assets/user_supplied'/slug;r=E.parse(D/'model.urdf').getroot();root=E.Element('mujoco',model=slug+'_static_preview');E.SubElement(root,'compiler',angle='radian');a=E.SubElement(root,'asset');w=E.SubElement(root,'worldbody');links={l.get('name'):l for l in r.findall('link')};joints=r.findall('joint');children={j.find('child').get('link') for j in joints};count=0
 def visit(n,parent,j=None):
  global count
  pos,q=tf(j) if j is not None else ([0,0,0],[1,0,0,0]);b=E.SubElement(parent,'body',name=n,pos=st(pos),quat=st(q))
  for v in links[n].findall('visual'):
   mesh=v.find('geometry/mesh')
   if mesh is None:continue
   f=D/mesh.get('filename');assert f.exists()
   if f.suffix.lower()=='.dae':
    dest=D/'derived';dest.mkdir(exist_ok=True);obj=P/(kind+'_inspection_20261103/source/inspection_converted')/(f.stem+'.obj');shutil.copyfile(obj,dest/obj.name);f=dest/obj.name
   name='mesh_'+str(count);count+=1;E.SubElement(a,'mesh',name=name,file=str(f.relative_to(D)),scale=mesh.get('scale','1 1 1'),inertia='shell');p,q=tf(v);c=v.find('material/color');rgba=c.get('rgba') if c is not None else '.6 .65 .7 1';E.SubElement(b,'geom',name=name,type='mesh',mesh=name,pos=st(p),quat=st(q),rgba=rgba,contype='0',conaffinity='0')
  for child in joints:
   if child.find('parent').get('link')==n:visit(child.find('child').get('link'),b,child)
 visit(next(iter(set(links)-children)),w);E.indent(root);E.ElementTree(root).write(D/'preview.xml',encoding='utf-8',xml_declaration=True)
 m=mujoco.MjModel.from_xml_path(str(D/'preview.xml'));assert m.njnt==0 and m.nu==0 and m.ngeom==count
 # Verify original payloads retained exactly.
 prov=json.loads((D/'provenance.json').read_text())
 for path,sha in prov['source_files'].items():assert hashlib.sha256((D/'source'/path).read_bytes()).hexdigest()==sha
 for mesh in r.findall('.//mesh'):assert (D/mesh.get('filename')).is_file()
 data=(D/'model.glb').read_bytes();assert data[:4]==b'glTF' and struct.unpack_from('<I',data,8)[0]==len(data)
 jslen=struct.unpack_from('<I',data,12)[0];gl=json.loads(data[20:20+jslen]);assert len(gl['meshes'])>0 and all('uri' not in b for b in gl.get('buffers',[]))
 meta=json.loads((D/'asset.json').read_text());meta['representations']['mujoco']=str((D/'preview.xml').relative_to(R));meta['limitations'].append('preview.xml is a fixed, non-colliding appearance model only; use source URDF for future physical adaptation');(D/'asset.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
 report=json.loads((D/'validation.json').read_text());report['publication_checks']=dict(source_hashes_verified=len(prov['source_files']),mesh_references_resolve=True,glb_embedded_meshes=len(gl['meshes']),static_preview_geoms=m.ngeom,static_preview_joints=0,physical_operation_validated=False);(D/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 with (D/'README.md').open('a') as f:f.write('\n网页使用 `preview.xml`：固定零位、关闭接触，只用于外观展示；它不是可执行动力学版本。源 URDF 保留原有碰撞定义。\n')
 print(slug,report['publication_checks'],flush=True)
shutil.copyfile(P/'validate_publish_robots.py',R/'tools/tianji_rhino/validate_publish_robots.py')
