from pathlib import Path
import sys,json,xml.etree.ElementTree as ET,math
sys.path.insert(0,'/home/fangqi/.local/share/fq-assets-validation-deps')
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Sdf,Gf
r=Path(sys.argv[1]);s=Usd.Stage.CreateNew(str(r/'chair.usda'));world=UsdGeom.Xform.Define(s,'/World');s.SetDefaultPrim(world.GetPrim());UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1)
pbr=json.loads((r/'pbr.json').read_text());materials={}
for name,p in pbr.items():
 m=UsdShade.Material.Define(s,'/World/Materials/'+name);sh=UsdShade.Shader.Define(s,str(m.GetPath())+'/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(tuple(p['base_color']));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(p['roughness']);sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(p['metallic']);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');materials[name]=m
 if name=='GreyWovenFabric':
  uv=UsdShade.Shader.Define(s,str(m.GetPath())+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
  for channel,socket,typ in [('basecolor','diffuseColor',Sdf.ValueTypeNames.Color3f),('roughness','roughness',Sdf.ValueTypeNames.Float),('normal','normal',Sdf.ValueTypeNames.Normal3f)]:
   tx=UsdShade.Shader.Define(s,str(m.GetPath())+'/'+channel);tx.CreateIdAttr('UsdUVTexture');tx.CreateInput('file',Sdf.ValueTypeNames.Asset).Set('textures/fabric_'+channel+'.png');tx.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB' if channel=='basecolor' else 'raw');tx.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tx.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat');tx.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
   if channel=='normal':tx.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.5,.5,2,1));tx.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-.25,-.25,-1,0))
   out='r' if channel=='roughness' else 'rgb';tx.CreateOutput(out,Sdf.ValueTypeNames.Float if out=='r' else Sdf.ValueTypeNames.Float3);sh.CreateInput(socket,typ).ConnectToSource(tx.ConnectableAPI(),out)
doc=ET.parse(r/'chair.xml');meshes={m.get('name'):m.get('file') for m in doc.findall('.//asset/mesh')}
for g in doc.findall('.//geom'):
 name=g.get('name').replace('-','neg')
 if g.get('group')!='2':continue
 v=[];uv=[];norm=[];faces=[];fuv=[];fn=[]
 for line in (r/meshes[g.get('mesh')]).read_text().splitlines():
  a=line.split()
  if not a:continue
  if a[0]=='v':v.append(tuple(map(float,a[1:4])))
  if a[0]=='vt':uv.append(tuple(map(float,a[1:3])))
  if a[0]=='vn':norm.append(tuple(map(float,a[1:4])))
  if a[0]=='f':
   for token in a[1:]:
    vi,ti,ni=map(int,token.split('/'));faces.append(vi-1);fuv.append(uv[ti-1]);fn.append(norm[ni-1])
 mesh=UsdGeom.Mesh.Define(s,'/World/Visuals/'+name);mesh.CreatePointsAttr(v);mesh.CreateFaceVertexCountsAttr([3]*(len(faces)//3));mesh.CreateFaceVertexIndicesAttr(faces);mesh.CreateNormalsAttr(fn);mesh.SetNormalsInterpolation('faceVarying');mesh.CreateSubdivisionSchemeAttr('none');UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying').Set(fuv);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[g.get('material')])
for c in json.loads((r/'collision_geometry.json').read_text()):
 path='/World/Collision/'+c['name'].replace('-','neg');kind=c['type'];size=c['size']
 if kind=='box':
  shape=UsdGeom.Cube.Define(s,path);shape.CreateSizeAttr(2);shape.AddScaleOp().Set(Gf.Vec3f(*size))
 elif kind=='sphere':shape=UsdGeom.Sphere.Define(s,path);shape.CreateRadiusAttr(size[0])
 elif kind=='cylinder':shape=UsdGeom.Cylinder.Define(s,path);shape.CreateRadiusAttr(size[0]);shape.CreateHeightAttr(2*size[1])
 elif kind=='capsule':
  shape=UsdGeom.Capsule.Define(s,path);shape.CreateRadiusAttr(size[0]);a=Gf.Vec3d(*c['fromto'][:3]);b=Gf.Vec3d(*c['fromto'][3:]);d=b-a;shape.CreateHeightAttr(d.GetLength());c['pos']=list((a+b)/2);shape.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),d).GetQuat()))
 xf=UsdGeom.Xformable(shape);ops=xf.GetOrderedXformOps();tr=xf.AddTranslateOp();tr.Set(Gf.Vec3d(*c.get('pos',[0,0,0])));xf.SetXformOpOrder([tr]+ops)
 if 'euler' in c:
  rot=shape.AddRotateXYZOp();rot.Set(Gf.Vec3f(*(x*180/math.pi for x in c['euler'])));xf.SetXformOpOrder([tr,rot]+ops)
 shape.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
s.GetRootLayer().Save();print('USD exported',len(list(s.Traverse())))
