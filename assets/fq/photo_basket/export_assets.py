"""Generate platform-neutral MJCF and URDF from Blender geometry.json."""
import json,math,sys,xml.etree.ElementTree as E
from pathlib import Path
R=Path(__file__).resolve().parent
G=json.loads((R/'geometry.json').read_text());P=G['parameters']
def fmt(v): return ' '.join(f'{x:.9g}' for x in v)
def sub(p,t,**a):return E.SubElement(p,t,{k:str(v) for k,v in a.items()})
def save(root,path):E.indent(root);E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
root=E.Element('mujoco',model='photo_reference_basket')
sub(root,'compiler',angle='radian',meshdir='meshes',autolimits='true')
option=sub(root,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast',iterations='80')
sub(option,'flag',multiccd='enable')
default=sub(root,'default');sub(default,'geom',friction=f"{P['friction']} .005 .0001",solref='.003 1',solimp='.98 .999 .0005',condim='4')
sub(root,'size',njmax='4000',nconmax='2000')
visual_settings=sub(root,'visual');sub(visual_settings,'global',offwidth='960',offheight='720')
asset=sub(root,'asset')
sub(asset,'material',name='white',rgba='.83 .85 .79 1',shininess='.25',specular='.3')
sub(asset,'material',name='gray',rgba='.15 .17 .16 1',shininess='.25')
for name in G['visuals']:sub(asset,'mesh',name=name,file=f'{name}.obj')
for c in G['body_collision']:
    if 'vertices' in c:sub(asset,'mesh',name=c['name'],file=f"collision_{c['name']}.obj")
world=sub(root,'worldbody');sub(world,'light',pos='0 -.5 1.3',dir='0 .2 -1',diffuse='.9 .9 .9')
sub(world,'light',pos='.6 .3 .8',dir='-.3 -.2 -1',diffuse='.5 .5 .5')
sub(world,'geom',name='ground',type='plane',size='2 2 .1',rgba='.17 .20 .23 1')
body=sub(world,'body',name='basket',pos='0 0 .035');sub(body,'freejoint',name='basket_free')
sub(body,'inertial',pos='0 0 .108',mass=P['body_mass'],diaginertia='.0032 .0047 .0054')
sub(body,'geom',name='basket_visual',type='mesh',mesh='basket_body',material='white',contype='0',conaffinity='0',group='2',mass='0')
for c in G['body_collision']:
    args=dict(name='basket_'+c['name'],group='3',rgba='0 0 0 0',mass='0')
    if 'vertices' in c:args.update(type='mesh',mesh=c['name'])
    else:args.update(type='box',pos=fmt(c['center']),size=fmt([s/2 for s in c['size']]))
    sub(body,'geom',**args)
for sign,name in [(1,'handle_right'),(-1,'handle_left')]:
    h=sub(body,'body',name=name,pos=fmt([sign*P['handle_anchor_x'],0,P['handle_pivot_z']]))
    sub(h,'joint',name=name+'_hinge',type='hinge',axis=fmt([0,-sign,0]),range=fmt(P['joint_limits']),damping=P['joint_damping'],armature='.00001',frictionloss='.0001',solreflimit='.003 1',solimplimit='.99 .999 .001')
    sub(h,'inertial',pos=fmt([sign*.108,0,0]),mass=P['handle_mass'],diaginertia='.0007 .0002 .00085')
    sub(h,'geom',name=name+'_visual',type='mesh',mesh=name,material='gray',contype='0',conaffinity='0',group='2',mass='0')
    for i,c in enumerate(G['handle_collision'][name]):sub(h,'geom',name=f'{name}_collision_{i}',type='box',pos=fmt(c['center']),size=fmt([s/2 for s in c['size']]),euler=fmt(c['rpy']),group='3',rgba='0 0 0 0',mass='0')
key=sub(root,'keyframe');sub(key,'key',name='photo_pose',qpos=fmt([0,0,.035,1,0,0,0,P['joint_initial'],P['joint_initial']]))
save(root,R/'basket.xml')
# Standalone drop test: payload is distinct from the reusable basket asset.
test=E.fromstring(E.tostring(root));w=test.find('worldbody');payload=sub(w,'body',name='payload',pos='0 0 .42')
sub(payload,'freejoint',name='payload_free');sub(payload,'geom',name='payload_geom',type='sphere',size='.025',mass='.06',rgba='.65 .12 .055 1')
test.remove(test.find('keyframe'));key=sub(test,'keyframe');sub(key,'key',name='photo_pose',qpos=fmt([0,0,.035,1,0,0,0,P['joint_initial'],P['joint_initial'],0,0,.42,1,0,0,0]))
save(test,R/'test_mujoco.xml')

robot=E.Element('robot',name='photo_reference_basket')
for name,rgba in [('white','.83 .85 .79 1'),('gray','.15 .17 .16 1')]: sub(sub(robot,'material',name=name),'color',rgba=rgba)
def inertial(link,mass,center,diag):
    i=sub(link,'inertial');sub(i,'origin',xyz=fmt(center),rpy='0 0 0');sub(i,'mass',value=mass)
    sub(i,'inertia',ixx=diag[0],iyy=diag[1],izz=diag[2],ixy='0',ixz='0',iyz='0')
def visual(link,name,mat):
    v=sub(link,'visual');sub(v,'origin',xyz='0 0 0',rpy='0 0 0');sub(sub(v,'geometry'),'mesh',filename=f'meshes/{name}.obj');sub(v,'material',name=mat)
def collision(link,c):
    v=sub(link,'collision',name=c.get('name','box'))
    sub(v,'origin',xyz=fmt(c.get('center',[0,0,0])),rpy=fmt(c.get('rpy',[0,0,0])))
    geo=sub(v,'geometry')
    if 'vertices' in c:sub(geo,'mesh',filename=f"meshes/collision_{c['name']}.obj")
    else:sub(geo,'box',size=fmt(c['size']))
b=sub(robot,'link',name='basket_body');inertial(b,P['body_mass'],[0,0,.108],[.0032,.0047,.0054]);visual(b,'basket_body','white')
for c in G['body_collision']:collision(b,c)
for sign,name in [(1,'handle_right'),(-1,'handle_left')]:
    h=sub(robot,'link',name=name);inertial(h,P['handle_mass'],[sign*.108,0,0],[.0007,.0002,.00085]);visual(h,name,'gray')
    for c in G['handle_collision'][name]:collision(h,c)
    j=sub(robot,'joint',name=name+'_hinge',type='revolute');sub(j,'parent',link='basket_body');sub(j,'child',link=name)
    sub(j,'origin',xyz=fmt([sign*P['handle_anchor_x'],0,P['handle_pivot_z']]),rpy='0 0 0');sub(j,'axis',xyz=fmt([0,-sign,0]))
    sub(j,'limit',lower=P['joint_limits'][0],upper=P['joint_limits'][1],effort='.3',velocity='4');sub(j,'dynamics',damping=P['joint_damping'],friction='.0001')
save(robot,R/'basket.urdf')
print('EXPORTED MJCF, test MJCF, URDF',flush=True)
