"""Install head geometry only into an isolated SIMPLE checkout."""
from pathlib import Path
import argparse,copy,json,os,shutil,hashlib,xml.etree.ElementTree as E
ap=argparse.ArgumentParser();ap.add_argument('--simple',type=Path,required=True);ap.add_argument('--assets',type=Path,required=True);ap.add_argument('--backup',type=Path,required=True);a=ap.parse_args();changed=[]
def backup(p):
 dest=a.backup/p.relative_to(a.simple)
 if p.exists() and not dest.exists():dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 changed.append(str(p.relative_to(a.simple)))
def save(t,p):backup(p);E.indent(t);t.write(p,encoding='utf-8',xml_declaration=True)
for family,folder,stem in [('o6','g1_o6','g1_29dof_wholebody_o6'),('o7','g1_o7_left','g1_29dof_wholebody_o7_left')]:
 source=a.assets/f'assets/fq/robots/g1_{family}';dest=a.simple/'data/robots'/folder
 if not dest.exists():continue
 model=E.parse(source/'robot.xml').getroot();names={g.get('mesh') for n in ['head_link','d435i_head_housing'] for g in model.findall(f'.//body[@name="{n}"]/geom')}
 meshes={el.get('name'):el for el in model.findall('asset/mesh') if el.get('name') in names}
 installed={}
 for name,mesh in meshes.items():
  src=(source/'meshes'/mesh.get('file')).resolve();target=dest/'meshes/fq_head_slot'/src.name;backup(target);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target);installed[name]=target
 paths=[dest/(stem+'.xml')]
 if family=='o6':paths.extend([dest/'d435i/g1_o6_mp.xml',dest/'d435i/g1_o6_mp_capture.xml'])
 for p in paths:
  if not p.exists():continue
  t=E.parse(p);r=t.getroot();assets=r.find('asset');meshdir=dest/'meshes'
  r.find('compiler').set('meshdir',os.path.relpath(meshdir,p.parent).replace('\\','/'))
  for name,target in installed.items():
   mesh=assets.find(f'mesh[@name="{name}"]')
   if mesh is None:mesh=E.SubElement(assets,'mesh',name=name)
   mesh.set('file',os.path.relpath(target,meshdir).replace('\\','/'))
  for n in ['head_link','d435i_head_housing']:
   body=r.find(f'.//body[@name="{n}"]');assert body is not None
   for el in list(body.findall('geom')):
    # Preserve SIMPLE's pre-existing physical head-camera box/mass.
    if n=='d435i_head_housing' and (el.get('name')=='d435i_head_collision' or el.get('type')=='box'):continue
    body.remove(el)
   for geom in model.findall(f'.//body[@name="{n}"]/geom'):body.append(copy.deepcopy(geom))
  save(t,p)
 urdf=E.parse(source/'robot.urdf').getroot()
 for p in [dest/(stem+'.urdf'),dest/(stem+'_isaac.urdf')]:
  if not p.exists():continue
  t=E.parse(p);r=t.getroot()
  for n in ['head_link','d435i_head_housing']:
   link=r.find(f'link[@name="{n}"]');assert link is not None
   for tag in ['visual','collision']:
    if n=='d435i_head_housing' and tag=='collision':continue
    for el in list(link.findall(tag)):link.remove(el)
    for el in urdf.findall(f'link[@name="{n}"]/{tag}'):
     el=copy.deepcopy(el)
     for mesh in el.findall('geometry/mesh'):mesh.set('filename',os.path.relpath(dest/'meshes/fq_head_slot'/Path(mesh.get('filename')).name,p.parent).replace('\\','/'))
     link.append(el)
  save(t,p)
 meta=dest/'head_camera_slot.json';backup(meta);shutil.copy2(source/'head_camera_slot.json',meta)
record={'status':'provisional_head_slot_geometry','changed_files':changed,'backup':str(a.backup),'running_tasks_modified':False}
p=a.simple/'assets/calibration/g1_d435i/head_slot_update_manifest.json';backup(p);p.write_text(json.dumps(record,indent=2)+'\n');print(json.dumps({'updated':len(changed),'simple':str(a.simple)}))
