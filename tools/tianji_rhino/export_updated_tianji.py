from pathlib import Path
import bpy,xml.etree.ElementTree as E
from mathutils import Matrix,Euler,Vector
D=Path('/assets/assets/user_supplied/tianji_robot');r=E.parse(D/'model.urdf').getroot()
bpy.ops.wm.read_factory_settings(use_empty=True)
links={l.get('name'):l for l in r.findall('link')};parents={};joints={j.find('child').get('link'):j for j in r.findall('joint')}
for n in links:
 ob=bpy.data.objects.new(n,None);bpy.context.collection.objects.link(ob);parents[n]=ob
def transform(o):
 if o is None:return Matrix.Identity(4)
 pos=Vector(map(float,o.get('xyz','0 0 0').split()));rot=Euler(tuple(map(float,o.get('rpy','0 0 0').split())),'XYZ').to_matrix().to_4x4();return Matrix.Translation(pos)@rot
for n,ob in parents.items():
 if n in joints:
  j=joints[n];ob.parent=parents[j.find('parent').get('link')];ob.matrix_local=transform(j.find('origin'));ob['joint_name']=j.get('name');ob['joint_type']=j.get('type')
for n,l in links.items():
 for idx,v in enumerate(l.findall('visual')):
  mesh=v.find('geometry/mesh')
  if mesh is None:continue
  f=D/mesh.get('filename');before=set(bpy.data.objects)
  if f.suffix.lower()=='.stl':bpy.ops.import_mesh.stl(filepath=str(f))
  elif f.suffix.lower()=='.dae':bpy.ops.wm.collada_import(filepath=str(f))
  else:raise ValueError(f)
  objs=set(bpy.data.objects)-before;sc=list(map(float,mesh.get('scale','1 1 1').split()));T=transform(v.find('origin'))@Matrix.Diagonal(Vector(sc+[1]))
  color=v.find('material/color');rgba=tuple(map(float,color.get('rgba').split())) if color is not None else (.6,.65,.7,1)
  mat=bpy.data.materials.new(n+'_material');mat.diffuse_color=rgba;mat.use_nodes=True;mat.node_tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value=rgba
  for ob in objs:
   if ob.parent not in objs:ob.parent=parents[n];ob.matrix_local=T@ob.matrix_local
   if ob.type=='MESH' and f.suffix.lower()=='.stl':ob.data.materials.clear();ob.data.materials.append(mat)
bpy.ops.export_scene.gltf(filepath=str(D/'model.glb'),export_format='GLB',export_yup=True,export_animations=False)
