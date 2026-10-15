"""Reproducible post-build visual/PBR and convex collision modifier."""
from pathlib import Path
import json,itertools,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial import ConvexHull
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Sdf,Gf
from door_arch import apply_curve

def author(root,geometry):
 root=Path(root);g=geometry;stage=Usd.Stage.Open(str(root/'refrigerator.usda'))
 report=apply_curve(stage,'/Refrigerator')
 tree=E.parse(root/'refrigerator.xml');asset=tree.find('asset')
 def numbers(a):return ' '.join(f'{float(x):.9g}' for x in a)
 def obj(path,points,faces,normals=None):
  with path.open('w') as f:
   for v in points:f.write('v '+numbers(v)+'\n')
   if normals is not None:
    for n in normals:f.write('vn '+numbers(n)+'\n')
   for t in faces:f.write('f '+' '.join(f'{i+1}//{i+1}' if normals is not None else str(i+1) for i in t)+'\n')
 bounds={}
 for v in g['visuals']:
  if v['name'] in ('part_049','part_076'):
   p=np.asarray(v['vertices']);bounds[v['link']]=(p.min(0),p.max(0))
 for rec in report:
  name=rec['part'];v=next(v for v in g['visuals'] if v['name']==name)
  mesh=UsdGeom.Mesh(stage.GetPrimAtPath(f"/Refrigerator/{v['link']}/Visual/{name}"))
  obj(root/'meshes'/f'{name}.obj',mesh.GetPointsAttr().Get(),np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3),mesh.GetNormalsAttr().Get())
 presets=json.loads((root/'appearance.json').read_text())
 for v in g['visuals']:
  name=v['name'];p=presets['presets'][presets['region_assignment'][name]];color=p.get('color',v['rgba'][:3]);rgba=[*color,v['rgba'][3]]
  s=UsdShade.Shader(stage.GetPrimAtPath(f'/Refrigerator/Materials/{name}/Shader'))
  s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(p['roughness']);s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(p['metallic'])
  E.SubElement(asset,'material',name='pbr_'+name,rgba=numbers(rgba),shininess=str(1-p['roughness']),specular=str(.5+.5*p['metallic']),reflectance=str(.15*p['metallic']))
  geom=tree.find(f".//geom[@name='{name}']");geom.set('material','pbr_'+name);geom.attrib.pop('rgba',None)
 def bend(points,link):
  p=np.array(points,dtype=float);lo,hi=bounds[link];x=p[:,0]+g['anchors'][link][0]
  z=np.clip(np.minimum(p[:,2]-lo[2],hi[2]-p[:,2])/.03,0,1)
  d=.01*np.clip(1-(x/.35)**2,0,1)*(z*z*(3-2*z))*np.clip((hi[1]-p[:,1])/(hi[1]-lo[1]),0,1)
  p[:,1]-=d;return p
 count=0;maxerr=0
 for c in g['collisions']:
  if c['source'] not in ('Door_LEFT_metal','Door_RIGHT_metal'):continue
  link=c['link'];center=np.array(c['center']);size=np.array(c['size']);lo=center-size/2;hi=center+size/2
  old=tree.find(f".//geom[@name='{c['name']}']");parent=next(b for b in tree.iter('body') if old in list(b));parent.remove(old)
  stage.RemovePrim(f"/Refrigerator/{link}/Collision/{c['name']}")
  xs=np.linspace(lo[0],hi[0],max(1,int(np.ceil(size[0]/.035)))+1)
  blo,bhi=bounds[link]
  zs=sorted(set([lo[2],hi[2]]+[float(z) for z in list(np.linspace(blo[2],blo[2]+.03,7))+list(np.linspace(bhi[2]-.03,bhi[2],7)) if lo[2]+1e-6<z<hi[2]-1e-6]))
  for ix,(x0,x1) in enumerate(zip(xs,xs[1:])):
   for iz,(z0,z1) in enumerate(zip(zs,zs[1:])):
    points=bend(list(itertools.product([x0,x1],[lo[1],hi[1]],[z0,z1])),link);h=ConvexHull(points);faces=h.simplices.copy()
    for i,t in enumerate(faces):
     if np.dot(np.cross(points[t[1]]-points[t[0]],points[t[2]]-points[t[0]]),h.equations[i,:3])<0:faces[i]=t[::-1]
    name=f"{c['name']}_arch_{ix}_{iz}";obj(root/'meshes'/f'{name}.obj',points,faces)
    E.SubElement(asset,'mesh',name=name,file=name+'.obj');attrs=dict(old.attrib);attrs.update(name=name,type='mesh',mesh=name);attrs.pop('pos',None);attrs.pop('size',None);E.SubElement(parent,'geom',**attrs)
    mesh=UsdGeom.Mesh.Define(stage,f'/Refrigerator/{link}/Collision/{name}');mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in points]);mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.flatten().tolist());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreatePurposeAttr('guide');UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    # Analytic-vs-piecewise surface error at 5x5 samples on the outer plane.
    for tx,tz in itertools.product(np.linspace(0,1,5),repeat=2):
     x=x0+(x1-x0)*tx;z=z0+(z1-z0)*tz;actual=bend([[x,lo[1],z]],link)[0,1]
     corners=bend([[x0,lo[1],z0],[x1,lo[1],z0],[x0,lo[1],z1],[x1,lo[1],z1]],link)[:,1]
     planes=h.equations[h.equations[:,1]<-1e-8];approx=max((-eq[3]-eq[0]*x-eq[2]*z)/eq[1] for eq in planes);maxerr=max(maxerr,abs(actual-approx))
    count+=1
 assert maxerr<.0003,maxerr
 E.indent(tree);tree.write(root/'refrigerator.xml',encoding='utf-8',xml_declaration=True);stage.GetRootLayer().Save()
 report={'status':'simulation_geometry_not_hardware_calibration','shared_arch_peak_m':.01,'visuals':report,'convex_collision_parts':count,'sampled_convex_surface_error_m':maxerr,'mass_kg':91,'hinges_and_resistance_unchanged':True}
 (root/'arch_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
 meta=json.loads((root/'manifest.json').read_text());meta.update(collision_boxes=sum(c.get('type')=='box' for c in tree.iter('geom') if c.get('group')=='3'),collision_convex_meshes=count,geometry_modifier='source/author_arch.py',appearance='appearance.json');(root/'manifest.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
 print('ARCH_AUTHORED',count,maxerr)

