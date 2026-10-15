"""Refresh baseline refrigerator body from canonical standalone MJCF."""
from pathlib import Path
import copy,math,xml.etree.ElementTree as E
r=Path(__file__).resolve().parents[1];p=r/'scenes/fq/office_baseline/scene.xml';scene=E.parse(p);source=E.parse(r/'assets/fq/haier_bcd402/refrigerator.xml');world=scene.find('worldbody');old=world.find('body[@name="refrigerator_base"]');index=list(world).index(old);body=copy.deepcopy(source.find('worldbody/body'));body.set('pos',old.get('pos'))
for geom in body.iter('geom'):
 if geom.get('mesh'):geom.set('mesh','baseline_fridge_'+geom.get('mesh'))
 if geom.get('material'):geom.set('material','baseline_fridge_'+geom.get('material'))
if scene.find('compiler') is None or scene.find('compiler').get('angle','degree')!='radian':
 for j in body.iter('joint'):
  if j.get('range'):j.set('range',' '.join(str(math.degrees(float(a))) for a in j.get('range').split()));j.set('limited','true')
import os
assets=scene.find('asset')
for node in list(assets):
 if node.get('name','').startswith('baseline_fridge_'):assets.remove(node)
for node in source.find('asset'):
 node=copy.deepcopy(node);node.set('name','baseline_fridge_'+node.get('name'))
 if node.get('file'):node.set('file',os.path.relpath(r/'assets/fq/haier_bcd402/meshes'/node.get('file'),p.parent))
 assets.append(node)
world.remove(old);world.insert(index,body);E.indent(scene);scene.write(p,encoding='utf-8',xml_declaration=True)
print('Baseline refrigerator refreshed at unchanged placement')
