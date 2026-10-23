"""Deterministic visual-only round table cleanup; leave all collisions untouched."""
from pathlib import Path
import sys, math, json, xml.etree.ElementTree as ET
sys.path.insert(0,'/home/fangqi/.local/share/fq-assets-validation-deps')
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
root=Path(sys.argv[1])
parts={
 'top':(.35,.35,.7275972962379428,.75,0,0,.001),
 'pillar':(.07,.07018544565,.02629869617521766,.7275972962379428,.00458909021,.00480801572,.0006),
 'base':(.2625,.263195419337,0,.02629869617521766,.00458909093,.00480801461,.0012),
}
materials={
 'round_table_top_pbr':{'roughness':.48,'metalness':0,'base_color':[.815,.797,.756]},
 'round_table_pillar_pbr':{'roughness':.32,'metalness':0,'base_color':[.76,.75,.71]},
 'round_table_base_pbr':{'roughness':.38,'metalness':0,'base_color':[.74,.73,.69]},
}
meshes={}
for part,(rx,ry,z0,z1,cx,cy,b) in parts.items():
 rings=[]
 for j in range(5):
  a=-math.pi/2+j*math.pi/8;rings.append((rx-b+b*math.cos(a),z0+b+b*math.sin(a),math.cos(a),math.sin(a)))
 for j in range(5):
  a=j*math.pi/8;rings.append((rx-b+b*math.cos(a),z1-b+b*math.sin(a),math.cos(a),math.sin(a)))
 v=[];n=[];uv=[];f=[];N=256
 for r,z,nr,nz in rings:
  for j in range(N):
   a=2*math.pi*j/N;x,y=math.cos(a),math.sin(a)
   v.append((cx+r*x,cy+r*y*ry/rx,z));nn=(nr*x,nr*y*rx/ry,nz);l=math.sqrt(sum(q*q for q in nn));n.append(tuple(q/l for q in nn));uv.append((.5+r*x/(2*rx),.5+r*y/(2*rx)))
 for k in range(len(rings)-1):
  for j in range(N):
   a=k*N+j;bidx=k*N+(j+1)%N;c=(k+1)*N+j;d=(k+1)*N+(j+1)%N
   f.extend([(a,bidx,c),(bidx,d,c)])
 for k,sign in [(0,-1),(len(rings)-1,1)]:
  center=len(v);v.append((cx,cy,rings[k][1]));n.append((0,0,sign));uv.append((.5,.5))
  for j in range(N):
   a=k*N+j;bb=k*N+(j+1)%N;f.append((center,bb,a) if sign<0 else (center,a,bb))
 path=root/f'assets/fq/tabletop/common/meshes/round_table_{part}_pbr.obj'
 lines=['# FQ visual-only regularized surface, explicit analytic normals, meters']
 lines += ['v '+' '.join(f'{x:.12g}' for x in row) for row in v]
 lines += ['vt '+' '.join(f'{x:.12g}' for x in row) for row in uv]
 lines += ['vn '+' '.join(f'{x:.12g}' for x in row) for row in n]
 lines += ['f '+' '.join(f'{i+1}/{i+1}/{i+1}' for i in row) for row in f]
 path.write_text('\n'.join(lines)+'\n');meshes[part]=(v,n,uv,f)
for rel in ['assets/fq/tabletop/mujoco/round_table.xml','scenes/fq/office_baseline/scene.xml']:
 p=root/rel;t=ET.parse(p);a=t.getroot().find('asset')
 old=next(x for x in a.findall('mesh') if x.get('name')=='round_table_visual');prefix=str(Path(old.get('file')).parent);a.remove(old)
 old=next(x for x in a.findall('material') if x.get('name')=='round_table_mat');a.remove(old)
 for part in parts:
  ET.SubElement(a,'mesh',name=f'round_table_{part}_pbr',file=f'{prefix}/round_table_{part}_pbr.obj')
  props=materials[f'round_table_{part}_pbr'];attrs={'name':f'round_table_{part}_pbr','rgba':' '.join(map(str,props['base_color']+[1])),'specular':'.25','shininess':str(1-props['roughness'])}
  if part=='top':attrs.update(texture='surface_table_tex',rgba='1 1 1 1')
  ET.SubElement(a,'material',attrs)
 body=next(x for x in t.iter('body') if x.get('name')=='round_table');body.remove(next(x for x in body.findall('geom') if x.get('name')=='round_table_visual'))
 for part in parts:ET.SubElement(body,'geom',name=f'round_table_{part}_visual',mesh=f'round_table_{part}_pbr',material=f'round_table_{part}_pbr',type='mesh',contype='0',conaffinity='0',group='2',mass='0')
 ET.indent(t,space='  ');t.write(p,encoding='unicode');p.write_text(p.read_text()+'\n')
for rel,base in [('assets/fq/tabletop/isaac/round_table.usd','/World'),('scenes/fq/office_baseline/scene.usda','/World/round_table')]:
 s=Usd.Stage.Open(str(root/rel));s.RemovePrim(base+'/Meshes/visual')
 for part,(v,n,uv,f) in meshes.items():
  m=UsdGeom.Mesh.Define(s,base+'/Meshes/'+part+'_visual');m.CreatePointsAttr(v);m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr([i for face in f for i in face]);m.CreateNormalsAttr(n);m.SetNormalsInterpolation('vertex');m.CreateSubdivisionSchemeAttr('none')
  UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set(uv)
  mat=UsdShade.Material.Define(s,base+'/RoundTablePBR/'+part);shader=UsdShade.Shader.Define(s,str(mat.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');pr=materials[f'round_table_{part}_pbr']
  # USD inputs are linear; web stores the same swatch in sRGB.
  linear=lambda x:x/12.92 if x<=.04045 else ((x+.055)/1.055)**2.4
  shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(tuple(linear(x) for x in pr['base_color']));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(pr['roughness']);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(pr['metalness']);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
  if part=='top':
   tex=UsdShade.Shader.Define(s,str(mat.GetPath())+'/Color');tex.CreateIdAttr('UsdUVTexture')
   import os
   tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(os.path.relpath(root/'assets/fq/tabletop/common/textures/surfaces_20260921/table_basecolor.png',(root/rel).parent));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
   st=UsdShade.Shader.Define(s,str(mat.GetPath())+'/UV');st.CreateIdAttr('UsdPrimvarReader_float2');st.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');st.CreateOutput('result',Sdf.ValueTypeNames.Float2);tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(),'result');shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
  UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
 s.GetRootLayer().Save()
p=root/'assets/fq/tabletop/round_table/pbr.json';p.write_text(json.dumps({'schema_version':1,'status':'estimated_appearance_not_measured','materials':materials,'regions':['top','pillar','base'],'edge_bevel_m':{'top':.001,'pillar':.0006,'base':.0012},'source':'FQ cleanup of existing table silhouette; original photo-derived tabletop base color retained','collision_changed':False},indent=2)+'\n')
p=root/'viewer/export_assets.py';text=p.read_text();text=text.replace("table_pbr=json.loads((root/'assets/fq/conference_table/pbr.json').read_text())","table_pbr=json.loads((root/'assets/fq/conference_table/pbr.json').read_text())\n    round_pbr=root/'assets/fq/tabletop/round_table/pbr.json'\n    if round_pbr.exists():table_pbr['materials'].update(json.loads(round_pbr.read_text())['materials'])")
p.write_text(text)
print('Built visual surfaces and PBR; collision definitions untouched')
