from pathlib import Path
import bpy,json,math,shutil,hashlib
from mathutils import Vector
S=Path('/project/assets/external/laptop_candidates/grab3d');D=Path('/assets/assets/third_party/office/grab3d_laptop');D.mkdir(parents=True,exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True);bpy.ops.import_scene.gltf(filepath=str(S/'laptop.glb'));lid=bpy.data.objects['laptop_lid'];lid.rotation_mode='XYZ';lid.rotation_euler.x+=math.radians(110);bpy.context.view_layer.update();pivot=list(lid.matrix_world.translation);parts=[]
for i,name in enumerate(['laptop_body','laptop_lid']):
 o=bpy.data.objects[name];o.data.calc_loop_triangles();o.data.calc_normals_split();offset=Vector(pivot) if i else Vector((0,0,0));pts=[list(o.matrix_world@v.co-offset) for v in o.data.vertices];faces=[];uv=[];ns=[];nm=o.matrix_world.to_3x3().inverted().transposed()
 for tri in o.data.loop_triangles:
  faces.append(list(tri.vertices))
  for k in tri.loops:uv.append(list(o.data.uv_layers.active.data[k].uv));ns.append(list((nm@o.data.loops[k].normal).normalized()))
 slug='lid' if i else 'base'
 with (D/(slug+'.obj')).open('w') as f:
  f.write('mtllib '+slug+'.mtl\nusemtl '+slug+'\n')
  for p in pts:f.write('v '+' '.join(map(str,p))+'\n')
  for p in uv:f.write('vt '+' '.join(map(str,p))+'\n')
  for p in ns:f.write('vn '+' '.join(map(str,p))+'\n')
  for j,face in enumerate(faces):f.write('f '+' '.join(f'{v+1}/{j*3+k+1}/{j*3+k+1}' for k,v in enumerate(face))+'\n')
 shutil.copyfile(S/'inspection/closed_0'/('part'+str(i)+'.png'),D/(slug+'.png'));(D/(slug+'.mtl')).write_text('newmtl '+slug+'\nKd 1 1 1\nmap_Kd '+slug+'.png\n')
 lo=[min(p[j] for p in pts) for j in range(3)];hi=[max(p[j] for p in pts) for j in range(3)];parts.append(dict(name=slug,points=pts,faces=faces,uvs=uv,normals=ns,bounds=[lo,hi],mass_kg=.35 if i else 1.1));print('PART',slug,lo,hi,flush=True)
for name,target in [('laptop.glb','model.glb'),('LICENSE.txt','LICENSE.txt'),('manifest.json','upstream_manifest.json')]:shutil.copyfile(S/name,D/target)
(D/'geometry.json').write_text(json.dumps(dict(pivot_m=pivot,axis=[-1,0,0],parts=parts)))
(D/'provenance.json').write_text(json.dumps(dict(source_url='https://grab3d.com/models/electronics/laptop/',download_url='https://grab3d.com/dl/laptop/laptop.glb',sha256=hashlib.sha256((S/'laptop.glb').read_bytes()).hexdigest(),license='CC0-1.0',changes=['Closed-pose geometry localized to original hinge pivot','FQ added articulated physics; estimated mass, friction, damping and limits','Visual GLB retained unmodified; MJCF/USD/URDF use explicit revolute joint'],upstream_origin=json.loads((S/'manifest.json').read_text())['origin']),indent=2))
print('PIVOT',pivot,flush=True)
