from pathlib import Path
import copy,json,math,sys,os
import numpy as np
import mujoco as mj
import xml.etree.ElementTree as E
r=Path(sys.argv[1]);sd=r/'scenes/fq/office_baseline';cr=r/'assets/fq/office_chair'
src=E.parse(cr/'chair_adjustable.xml').getroot();m=mj.MjModel.from_xml_path(str(cr/'chair_adjustable.xml'))
scene=E.parse(sd/'scene.xml');root=scene.getroot();assets=root.find('asset');world=root.find('worldbody');prefix='officechair__'
for a in list(assets):
 if (a.get('name') or '').startswith(prefix):assets.remove(a)
for b in list(world):
 if (b.get('name') or '').startswith('office_chair_'):world.remove(b)
for a in src.find('asset'):
 a=copy.deepcopy(a)
 if a.get('name'):a.set('name',prefix+a.get('name'))
 if a.get('file'):a.set('file',os.path.relpath(cr/a.get('file'),sd))
 assets.append(a)
placements=[]
for k,(x,y,yaw,swivel) in enumerate([(1.18,9.12,168,-28),(2.40,9.30,185,18),(3.61,9.03,173,-8),(1.22,11.53,-12,30),(2.43,11.38,9,-32),(3.64,11.60,-7,12)],1):
 name=f'office_chair_{k}';d=mj.MjData(m)
 d.qpos[m.joint('seat_swivel').qposadr[0]]=math.radians(swivel)
 for side in ('left','right'):d.qpos[m.joint('armrest_'+side+'_lift').qposadr[0]]=-.04
 for j in range(m.njnt):
  n=mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,j) or ''
  if n.startswith('caster_swivel'):d.qpos[m.jnt_qposadr[j]]=math.radians((k*37+j*19)%150-75)
 mj.mj_forward(m,d);q=[math.cos(math.radians(yaw)/2),0,0,math.sin(math.radians(yaw)/2)]
 inst=E.SubElement(world,'body',name=name,pos=f'{x} {y} 0',quat=' '.join(map(str,q)))
 poses={}
 for body in src.findall('.//body'):
  bid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,body.get('name'))
  b=E.SubElement(inst,'body',name=name+'__'+body.get('name'),pos=' '.join(map(str,d.xpos[bid])),quat=' '.join(map(str,d.xquat[bid])))
  for g in body.findall('geom'):
   g=copy.deepcopy(g);gn=g.get('name');g.set('name',name+'__'+gn)
   for key in ('mesh','material'):
    if g.get(key):g.set(key,prefix+g.get(key))
   if g.get('contype','1')!='0' or g.get('conaffinity','1')!='0':g.set('contype','1');g.set('conaffinity','1')
   b.append(g);poses[gn]={'body_pos':d.xpos[bid].tolist(),'body_quat':d.xquat[bid].tolist()}
 collisions=[]
 for gi in range(m.ngeom):
  if not (m.geom_contype[gi] or m.geom_conaffinity[gi]):continue
  gq=np.empty(4);mj.mju_mat2Quat(gq,d.geom_xmat[gi]);collisions.append({'name':mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,gi),'type':int(m.geom_type[gi]),'size':m.geom_size[gi].tolist(),'pos':d.geom_xpos[gi].tolist(),'quat':gq.tolist()})
 placements.append({'collisions':collisions,'name':name,'asset_id':'fq:articulated/office_chair','version':'0.1.0','position_m':[x,y,0],'yaw_deg':yaw,'seat_swivel_deg':swivel,'armrest_lowering_m':.04,'quaternion_wxyz':q,'fixed_base':True,'joints_baked':True,'mesh_body_poses':poses})
E.indent(scene);scene.write(sd/'scene.xml',encoding='unicode')
(sd/'chair_placements.json').write_text(json.dumps({'status':'authored_varied_layout_not_measured','method':'Six deliberately varied poses, saved deterministically; no startup randomization','instances':placements},indent=2))
print('placed',[(p['name'],p['position_m'],p['yaw_deg'],p['seat_swivel_deg']) for p in placements])
