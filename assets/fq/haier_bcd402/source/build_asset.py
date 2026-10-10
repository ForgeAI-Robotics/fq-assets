"""Build portable MJCF and USD from evaluated Blender geometry. Physics is provisional."""
from pathlib import Path
import json,math,xml.etree.ElementTree as E
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
ROOT=Path('/mnt/simple');R=ROOT/'data/assets/refrigerator/haier_bcd402_v1'
G=json.loads((ROOT/'.runtime/refrigerator-integration/geometry.json').read_text())
R.mkdir(parents=True,exist_ok=True);(R/'meshes').mkdir(exist_ok=True)
# Recess shadow meshes are visual fills, not solid material.
G['collisions']=[c for c in G['collisions'] if 'handle_shadow' not in c['source']]
anchors=G['anchors'];parents={'door_left':'cabinet','door_right':'cabinet','mullion':'door_left'}
joints={'door_left':'refrigerator_left_hinge','door_right':'refrigerator_right_hinge','mullion':'refrigerator_mullion_hinge'}
masses={'cabinet':70.,'door_left':7.,'door_right':7.,'mullion':.3}
coms={'cabinet':[0,0,.9],'door_left':[.165,-.022,.598],'door_right':[-.165,-.022,.598],'mullion':[.018,.007,.554]}
inertias={'cabinet':[20,20,5],'door_left':[.85,.92,.08],'door_right':[.85,.92,.08],'mullion':[.031,.031,.0001]}
def text(v):return ' '.join(f'{float(x):.9g}' for x in v)
def rel(a,b):return [a[i]-b[i] for i in range(3)]
x=E.Element('mujoco',model='haier_bcd402_v1');E.SubElement(x,'compiler',angle='radian',meshdir='meshes',autolimits='true')
E.SubElement(x,'option',timestep='.002',gravity='0 0 -9.81')
assets=E.SubElement(x,'asset');world=E.SubElement(x,'worldbody');bs={}
for link in anchors:
    if link=='cabinet':b=E.SubElement(world,'body',name='refrigerator_base')
    else:
        parent=parents[link];b=E.SubElement(bs[parent],'body',name='refrigerator_'+link,pos=text(rel(anchors[link],anchors[parent])))
        E.SubElement(b,'joint',name=joints[link],type='hinge',axis='0 0 -1' if link=='door_left' else '0 0 1',range='0 1.57079632679',damping='.15' if link!='mullion' else '.01',frictionloss='0',armature='.002')
        E.SubElement(b,'inertial',pos=text(coms[link]),mass=str(masses[link]),diaginertia=text(inertias[link]))
    bs[link]=b
for v in G['visuals']:
    name=v['name'];fp=R/'meshes'/f'{name}.obj'
    with fp.open('w') as f:
        for p in v['vertices']:f.write('v '+text(p)+'\n')
        for face in v['faces']:f.write('f '+' '.join(str(i+1) for i in face)+'\n')
    E.SubElement(assets,'mesh',name=name,file=name+'.obj')
    E.SubElement(bs[v['link']],'geom',name=name,type='mesh',mesh=name,rgba=text(v['rgba']),contype='0',conaffinity='0',group='2',density='0')
for c in G['collisions']:
    E.SubElement(bs[c['link']],'geom',name=c['name'],type='box',pos=text(c['center']),size=text([a/2 for a in c['size']]),rgba='0 0 0 0',group='3',density='0',friction='.5 .005 .0001',contype='0' if c['link']=='mullion' else '1',conaffinity='0' if c['link']=='mullion' else '1')
contact=E.SubElement(x,'contact')
E.SubElement(contact,'exclude',body1='refrigerator_base',body2='refrigerator_mullion')
E.indent(x);E.ElementTree(x).write(R/'refrigerator.xml',encoding='unicode')

stage=Usd.Stage.CreateNew(str(R/'refrigerator.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
root=UsdGeom.Xform.Define(stage,'/Refrigerator');stage.SetDefaultPrim(root.GetPrim());UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
root.GetPrim().SetCustomDataByKey('refrigerator:physicsCalibrated',False)
root.GetPrim().SetCustomDataByKey('refrigerator:source','Photo-estimated Blender body, no contents')
bodypaths={}
for link in anchors:
    path='/Refrigerator/'+link;bodypaths[link]=path
    o=UsdGeom.Xform.Define(stage,path);o.AddTranslateOp().Set(Gf.Vec3d(*anchors[link]))
    UsdPhysics.RigidBodyAPI.Apply(o.GetPrim());m=UsdPhysics.MassAPI.Apply(o.GetPrim());m.CreateMassAttr(masses[link]);m.CreateCenterOfMassAttr(Gf.Vec3f(*coms[link]));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertias[link]))
fixed=UsdPhysics.FixedJoint.Define(stage,'/Refrigerator/Joints/fixed_base');fixed.CreateBody1Rel().SetTargets([bodypaths['cabinet']]);fixed.CreateLocalPos0Attr(Gf.Vec3f(0));fixed.CreateLocalPos1Attr(Gf.Vec3f(0))
UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(bodypaths['mullion'])).CreateFilteredPairsRel().SetTargets([bodypaths['cabinet']])
for link,parent in parents.items():
    j=UsdPhysics.RevoluteJoint.Define(stage,'/Refrigerator/Joints/'+joints[link]);j.CreateBody0Rel().SetTargets([bodypaths[parent]]);j.CreateBody1Rel().SetTargets([bodypaths[link]])
    j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(*rel(anchors[link],anchors[parent])));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateLowerLimitAttr(0);j.CreateUpperLimitAttr(90);j.CreateCollisionEnabledAttr(False)
    if link=='door_left':
        flip=Gf.Quatf(0,Gf.Vec3f(1,0,0));j.CreateLocalRot0Attr(flip);j.CreateLocalRot1Attr(flip)
for v in G['visuals']:
    path=bodypaths[v['link']]+'/Visual/'+v['name'];m=UsdGeom.Mesh.Define(stage,path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in v['vertices']]);m.CreateFaceVertexCountsAttr([3]*len(v['faces']));m.CreateFaceVertexIndicesAttr([i for f in v['faces'] for i in f]);m.CreateSubdivisionSchemeAttr('none')
    m.CreateDisplayColorAttr([Gf.Vec3f(*v['rgba'][:3])]);m.CreateDisplayOpacityAttr([v['rgba'][3]])
    mat=UsdShade.Material.Define(stage,'/Refrigerator/Materials/'+v['name']);shader=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild('Shader'));shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*v['rgba'][:3]));shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(v['rgba'][3]);shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.35);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
for c in G['collisions']:
    o=UsdGeom.Cube.Define(stage,bodypaths[c['link']]+'/Collision/'+c['name']);o.CreateSizeAttr(1);o.AddTranslateOp().Set(Gf.Vec3d(*c['center']));o.AddScaleOp().Set(Gf.Vec3f(*c['size']));o.CreatePurposeAttr('guide');UsdPhysics.CollisionAPI.Apply(o.GetPrim()).CreateCollisionEnabledAttr(c['link']!='mullion')
stage.GetRootLayer().Save()
meta={'asset_id':'refrigerator:haier_bcd402_v1','entries':{'mujoco':'refrigerator.xml','isaac':'refrigerator.usda','blender':'source/haier_bcd402_body_v1.blend'},'initial_joint_positions':{j:0. for j in joints.values()},'joint_limits_rad':{j:[0,math.pi/2] for j in joints.values()},'units':'meter','up_axis':'Z','front_axis':'-Y','fixed_base':True,'contents_included':False,'physics_calibrated':False,'estimated_mass_kg':masses,'limitations':['Component dimensions are estimated from photos','Mass, inertia, friction and damping are provisional engineering placeholders','No seal adhesion or pressure differential model','Mullion is independent and has no automatic folding constraint','Freezer is static','MuJoCo parent-child contact filtering applies; full self-collision not calibrated','USD structural export does not imply Isaac runtime validation'],'preferred_robot':'g1_o7_left_wholebody','visual_meshes':len(G['visuals']),'collision_boxes':len(G['collisions'])}
meta['limitations'].append('Mullion contact response disabled pending guide-linkage calibration; not suitable for seal/contact policy evaluation')
(R/'manifest.json').write_text(json.dumps(meta,indent=2))
print('BUILT',R)

