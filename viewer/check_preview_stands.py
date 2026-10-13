from pathlib import Path
import json,copy,xml.etree.ElementTree as E
import numpy as np,mujoco
r=Path('/assets');out=Path('/out')
def canonical(path,label):
 m=mujoco.MjModel.from_xml_path(str(path));tmp=out/(label+'.xml');mujoco.mj_saveLastXML(str(tmp),m);tree=E.parse(tmp);compiler=tree.getroot().find('compiler')
 # Saved XML keeps file paths relative to the original compiler directories.
 source=E.parse(path).getroot();meshdir=path.parent/source.find('compiler').get('meshdir','') if source.find('compiler') is not None else path.parent
 texturedir=path.parent/(source.find('compiler').get('texturedir','') if source.find('compiler') is not None else '')
 for asset in tree.getroot().findall('asset/*'):
  if asset.get('file'):asset.set('file',str((meshdir if asset.tag=='mesh' else texturedir)/asset.get('file')))
 return tree.getroot()
scene=r/'scenes/fq/office_baseline/scene.xml';results=[]
for family in ['o6','o7']:
 robot=r/f'assets/fq/robots/g1_{family}/robot.xml';bot=canonical(robot,'robot_'+family)
 for preset,(x,y,stand) in {'table':(10.8,11.55,.65),'fridge':(16.5,11.9,1.6)}.items():
  root=canonical(scene,'scene');body=copy.deepcopy(bot.find('worldbody/body'));body.set('pos',f'{x} {y-stand} .7942728964');body.set('quat','0.7071067811865476 0 0 0.7071067811865476');root.find('worldbody').append(body)
  for el in bot.findall('asset/*'):root.find('asset').append(copy.deepcopy(el))
  for tag in ['contact','equality','actuator']:
   if bot.find(tag) is not None:
    dest=root.find(tag)
    if dest is None:dest=E.SubElement(root,tag)
    for el in bot.find(tag):dest.append(copy.deepcopy(el))
  m=mujoco.MjModel.from_xml_string(E.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
  robotids={m.body(b.get('name')).id for b in body.iter('body')};tests=[]
  for opening in ([0] if preset=='table' else [0,np.pi/4,np.pi/2]):
   for n in ['refrigerator_left_hinge','refrigerator_right_hinge']:d.qpos[m.joint(n).qposadr[0]]=opening
   mujoco.mj_forward(m,d);contacts=[]
   for c in d.contact:
    a=int(m.geom_bodyid[c.geom1]);b=int(m.geom_bodyid[c.geom2])
    if (a in robotids)!=(b in robotids) and c.dist<-.001:contacts.append({'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'distance':float(c.dist)})
   tests.append({'door_angle':opening,'penetrations_over_1mm':contacts})
  results.append({'family':family,'preset':preset,'tests':tests})
print(json.dumps(results,indent=2))
