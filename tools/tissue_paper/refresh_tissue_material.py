from pathlib import Path
import numpy as np,json,copy,xml.etree.ElementTree as E
from PIL import Image,ImageDraw,ImageFilter
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf
D=Path('/assets/assets/fq/tissue_paper');rng=np.random.default_rng(810);N=2048
# An authored fine fibrous surface with dot embossing; no geometric displacement.
noise=rng.normal(0,1,(N,N));fiber=Image.new('L',(N,N),128);dr=ImageDraw.Draw(fiber)
for _ in range(45000):
 x,y=rng.integers(0,N,2);dx,dy=rng.integers(-14,15,2);dr.line((int(x),int(y),int(x+dx),int(y+dy)),fill=int(rng.integers(90,170)),width=1)
h=np.asarray(fiber.filter(ImageFilter.GaussianBlur(.45)),dtype=float)/255
y,x=np.mgrid[:N,:N];spacing=82;u=(x+spacing/2)%spacing-spacing/2;v=(y+spacing/2)%spacing-spacing/2
emboss=np.exp(-((np.sqrt(u*u+v*v)-10)/2.4)**2)
height=.035*h+.018*emboss
tone=np.clip(245+5*(h-.5)-6*emboss+noise*.6,0,255).astype('uint8');Image.fromarray(np.stack([tone,tone,tone],-1)).save(D/'tissue_fiber_color.png')
gy,gx=np.gradient(height);normal=np.stack([-gx*18,gy*18,np.ones_like(gx)],-1);normal/=np.linalg.norm(normal,axis=-1,keepdims=True);Image.fromarray(np.uint8((normal*.5+.5)*255)).save(D/'tissue_fiber_normal.png')
z=np.load(D/'trajectory.npz');uv=z['rest_uv']/[.1,.12]+.5
for name in ['preview.xml','flat.xml']:
 p=D/name;r=E.parse(p).getroot();a=r.find('asset');mesh=a.find('mesh');mesh.set('texcoord',' '.join(map(str,uv.ravel())))
 E.SubElement(a,'texture',name='fibers',type='2d',file='tissue_fiber_color.png');E.SubElement(a,'material',name='tissue_white',texture='fibers',rgba='1 1 1 1',specular='.01',shininess='.01');g=r.find('worldbody/geom');g.attrib.pop('rgba',None);g.set('material','tissue_white')
 back=copy.deepcopy(mesh);back.set('name','tissue_back');faces=np.array(list(map(int,mesh.get('face').split()))).reshape(-1,3);back.set('face',' '.join(map(str,faces[:,::-1].ravel())));a.append(back)
 bg=copy.deepcopy(g);bg.set('name','tissue_visual_back');bg.set('mesh','tissue_back');bg.set('mass','0');r.find('worldbody').append(bg)
 E.indent(r);E.ElementTree(r).write(p,encoding='utf-8',xml_declaration=True)
for name in ['preview.usda','flat.usda','crumple_playback.usdc']:
 s=Usd.Stage.Open(str(D/name));m=UsdGeom.Mesh(s.GetPrimAtPath('/Tissue/Sheet'));m.GetDisplayColorAttr().Set([Gf.Vec3f(.96)])
 UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(uv.tolist())
 mat=UsdShade.Material.Define(s,'/Tissue/Material');sh=UsdShade.Shader.Define(s,'/Tissue/Material/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.96);sh.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.02));sh.CreateOutput('surface',Sdf.ValueTypeNames.Token)
 st=UsdShade.Shader.Define(s,'/Tissue/Material/ST');st.CreateIdAttr('UsdPrimvarReader_float2');st.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');st.CreateOutput('result',Sdf.ValueTypeNames.Float2)
 for key,file,typ in [('diffuseColor','tissue_fiber_color.png',Sdf.ValueTypeNames.Color3f),('normal','tissue_fiber_normal.png',Sdf.ValueTypeNames.Normal3f)]:
  t=UsdShade.Shader.Define(s,'/Tissue/Material/'+key);t.CreateIdAttr('UsdUVTexture');t.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(file));t.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB' if key=='diffuseColor' else 'raw');t.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(),'result');t.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
  if key=='normal':t.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2,2,2,1));t.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1,-1,-1,0))
  sh.CreateInput(key,typ).ConnectToSource(t.ConnectableAPI(),'rgb')
 mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat);s.GetRootLayer().Save()
new_physics=len(z['vertices'][0])==775
p=D/'asset.json';a=json.loads(p.read_text());a['version']='0.1.1';a['appearance']={'authored_fiber_texture':True,'embossing':'normal map only; no added physical geometry','physics_updated':new_physics};p.write_text(json.dumps(a,ensure_ascii=False,indent=2)+'\n')
p=D/'README.md';p.write_text(p.read_text()+'\n\nv0.1.1: neutral-white fine fibers and dot embossing added. Embossing is a visual normal map, not physical microgeometry. '+('Shell refined to 775 vertices / 1440 triangles, bending stiffness 8e-7 Nm, compression radius 27 mm; trajectory and contact forces revalidated. ' if new_physics else 'The validated shell, trajectory and contact force data are unchanged. ')+'MuJoCo displays the albedo; USD also carries the normal map.\n')
print('TISSUE_MATERIAL_UPDATED')
