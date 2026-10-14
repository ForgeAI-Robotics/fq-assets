"""Portable USD Physics asset, generated from the same Blender geometry as MJCF.
Authoring and structural validation do not substitute for Isaac runtime tests.
"""
from pathlib import Path
import json,math
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
R=Path(__file__).resolve().parent;G=json.loads((R/'geometry.json').read_text());P=G['parameters']
stage=Usd.Stage.CreateNew(str(R/'basket.usdc'));UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
root=UsdGeom.Xform.Define(stage,'/Basket');stage.SetDefaultPrim(root.GetPrim());UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
root.GetPrim().AddAppliedSchema('PhysxArticulationAPI')
root.GetPrim().CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(True)
root.GetPrim().CreateAttribute('physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(32)
root.GetPrim().CreateAttribute('physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int).Set(8)
root.GetPrim().SetCustomDataByKey('basket:estimatedDimensions',False)
root.GetPrim().SetCustomDataByKey('basket:estimatedMassAndFriction',True)
root.GetPrim().SetCustomDataByKey('basket:source','BDO-6550 label body dimensions; details estimated; see parameters.json')
def mat(name,color):
    m=UsdShade.Material.Define(stage,'/Basket/Materials/'+name);s=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('Shader'));s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.3)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m
white=mat('WarmWhite',(.83,.85,.79));gray=mat('Grey',(.15,.17,.16))
pm=UsdShade.Material.Define(stage,'/Basket/Materials/Contact');ph=UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
ph.CreateStaticFrictionAttr(P['friction']);ph.CreateDynamicFrictionAttr(P['friction']);ph.CreateRestitutionAttr(0.)
def collider(prim):
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema('PhysxCollisionAPI')
    UsdGeom.Imageable(prim).CreatePurposeAttr('guide')
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(pm,materialPurpose='physics')
    prim.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.001)
    prim.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
def solid_mesh(path,vertices,faces):
    m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(vertices);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr([x for f in faces for x in f]);m.CreateSubdivisionSchemeAttr('none');return m
def body(name,mass,com,inertia,pos,angle=0):
    p='/Basket/'+name;x=UsdGeom.Xform.Define(stage,p);x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if angle:x.AddOrientOp().Set(Gf.Quatf(math.cos(angle/2),Gf.Vec3f(0,math.sin(angle/2),0)))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());m=UsdPhysics.MassAPI.Apply(x.GetPrim());m.CreateMassAttr(mass);m.CreateCenterOfMassAttr(Gf.Vec3f(*com));m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    g=G['visuals'][name];v=solid_mesh(p+'/Visual',g['vertices'],g['triangles'])
    if 'triangle_normals' in g:v.CreateNormalsAttr(g['triangle_normals']);v.SetNormalsInterpolation('faceVarying')
    v.CreateDoubleSidedAttr(False);UsdShade.MaterialBindingAPI.Apply(v.GetPrim()).Bind(white if name=='basket_body' else gray)
    return x
b=body('basket_body',P['body_mass'],[0,0,P['body_com_z']],P['body_inertia'],[0,0,0])
for i,c in enumerate(G['body_collision']):
    m=solid_mesh('/Basket/basket_body/Collision/'+c['name'],c['vertices'],c['faces']);collider(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
for sign,name in [(1,'handle_right'),(-1,'handle_left')]:
    anchor=[sign*P['handle_anchor_x'],0,P['handle_pivot_z']]
    h=body(name,P['handle_mass'],[sign*P['handle_com_x'],0,0],P['handle_inertia'],anchor,-sign*P['joint_initial'])
    for i,c in enumerate(G['handle_collision'][name]):
        box=UsdGeom.Cube.Define(stage,'/Basket/'+name+f'/Collision/segment_{i:02d}');box.CreateSizeAttr(1)
        box.AddTranslateOp().Set(Gf.Vec3d(*c['center']));box.AddRotateZOp().Set(math.degrees(c['rpy'][2]));box.AddScaleOp().Set(Gf.Vec3f(*c['size']));collider(box.GetPrim())
    j=UsdPhysics.RevoluteJoint.Define(stage,'/Basket/Joints/'+name+'_hinge');j.CreateBody0Rel().SetTargets([b.GetPath()]);j.CreateBody1Rel().SetTargets([h.GetPath()]);j.CreateAxisAttr('Y')
    j.CreateLocalPos0Attr(Gf.Vec3f(*anchor));j.CreateLocalPos1Attr(Gf.Vec3f(0));rotation=Gf.Quatf(0,Gf.Vec3f(0,0,1)) if sign==1 else Gf.Quatf(1)
    j.CreateLocalRot0Attr(rotation);j.CreateLocalRot1Attr(rotation);j.CreateLowerLimitAttr(math.degrees(P['joint_limits'][0]));j.CreateUpperLimitAttr(math.degrees(P['joint_limits'][1]));j.CreateCollisionEnabledAttr(False)
    drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(P['joint_damping']*math.pi/180)
stage.GetRootLayer().Save()
wrapper=Usd.Stage.CreateNew(str(R/'basket.usd'));wr=UsdGeom.Xform.Define(wrapper,'/Basket');wr.GetPrim().GetReferences().AddReference('./basket.usdc');wrapper.SetDefaultPrim(wr.GetPrim());UsdGeom.SetStageUpAxis(wrapper,'Z');UsdGeom.SetStageMetersPerUnit(wrapper,1.);wrapper.GetRootLayer().Save()
# Verify the exported reference resolves and contains the expected free articulation.
check=Usd.Stage.Open(str(R/'basket.usd'));joints=[p for p in check.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)];rigids=[p for p in check.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)];collisions=[p for p in check.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
assert len(joints)==2 and len(rigids)==3 and len(collisions)>50
report={'status':'structural_validation_passed','isaac_runtime_validated':False,'usd_version':'.'.join(map(str,Usd.GetVersion())),'rigid_bodies':len(rigids),'revolute_joints':len(joints),'collision_primitives':len(collisions),'meters_per_unit':UsdGeom.GetStageMetersPerUnit(check),'up_axis':str(UsdGeom.GetStageUpAxis(check))}
(R/'evidence/usd_structure_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
