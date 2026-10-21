import bpy, math, random, json, os
from mathutils import Vector
from pathlib import Path
R=Path(__file__).resolve().parent;(R/'textures').mkdir(exist_ok=True)
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
# Top OD and height user measured; bottom OD provisional: metre units, Z up.
bpy.context.scene.unit_settings.system='METRIC'
N=96; steps=72; z0=.006; z1=.274; radius0=.106; radius1=.126; slope=(2*math.pi/N)/.012
verts=[];faces=[]
for sign in [-1,1]:
 for i in range(N):
  start=len(verts)
  for j in range(steps+1):
   z=z0+(z1-z0)*j/steps;r=radius0+(radius1-radius0)*j/steps;th=2*math.pi*i/N+sign*slope*(z-z0)
   radial=Vector((math.cos(th),math.sin(th),0));tan=Vector((-math.sin(th),math.cos(th),0));normal=(radial-Vector((0,0,(radius1-radius0)/(z1-z0)))).normalized()
   along=(radial*((radius1-radius0)/(z1-z0))+tan*(r*sign*slope)+Vector((0,0,1))).normalized();cross=normal.cross(along).normalized();p=radial*r+Vector((0,0,z))
   for u,v in [(-1,-1),(1,-1),(1,1),(-1,1)]:verts.append(tuple(p+cross*(u*.00036)+normal*(v*.00024)))
  for j in range(steps):
   for k in range(4):a=start+4*j+k;b=start+4*j+(k+1)%4;faces.append((a,b,b+4,a+4))
  faces.extend([(start+3,start+2,start+1,start),(start+steps*4,start+steps*4+1,start+steps*4+2,start+steps*4+3)])
mesh=bpy.data.meshes.new('diamond_lattice_real_openings');mesh.from_pydata(verts,[],faces);mesh.update();obj=bpy.data.objects.new('PerforatedSteel',mesh);bpy.context.collection.objects.link(obj)
# Cylindrical UVs shared by all actual lattice faces.
uv=mesh.uv_layers.new(name='UVMap')
for poly in mesh.polygons:
 for li in poly.loop_indices:
  p=mesh.vertices[mesh.loops[li].vertex_index].co;uv.data[li].uv=((math.atan2(p.y,p.x)/(2*math.pi))%1,p.z/.28)
# Exportable PBR images: subtle powder-coat variation, no lighting baked in.
random.seed(21)
for name in ['basecolor','roughness','normal']:
 if (R/'textures'/f'powdercoat_{name}.png').exists():continue
 im=bpy.data.images.new(name,width=256,height=256);pixels=[]
 for j in range(256*256):
  noise=random.random()-.5
  if name=='basecolor':v=.035+noise*.008;rgba=(v*.85,v*.92,v,1)
  elif name=='roughness':v=.48+noise*.08;rgba=(v,v,v,1)
  else:rgba=(.5+noise*.025,.5+noise*.025,1,1)
  pixels.extend(rgba)
 im.pixels=pixels;im.filepath_raw=str(R/'textures'/f'powdercoat_{name}.png');im.file_format='PNG';im.save()
mat=bpy.data.materials.new('BlackPowderCoatedSteel');mat.use_nodes=True;n=mat.node_tree.nodes;links=mat.node_tree.links;bs=n.get('Principled BSDF');bs.inputs['Metallic'].default_value=.2
for name,socket in [('basecolor','Base Color'),('roughness','Roughness')]:
 tx=n.new('ShaderNodeTexImage');tx.image=bpy.data.images.load(str(R/'textures'/f'powdercoat_{name}.png'))
 if name!='basecolor':tx.image.colorspace_settings.name='Linear'
 links.new(tx.outputs['Color'],bs.inputs[socket])
tx=n.new('ShaderNodeTexImage');tx.image=bpy.data.images.load(str(R/'textures/powdercoat_normal.png'));tx.image.colorspace_settings.name='Linear';nm=n.new('ShaderNodeNormalMap');nm.inputs['Strength'].default_value=.2;links.new(tx.outputs['Color'],nm.inputs['Color']);links.new(nm.outputs['Normal'],bs.inputs['Normal'])
obj.data.materials.append(mat)
for name,z,major,minor in [('RolledRim',.276,.126,.004),('BottomRing',.003,.107,.003)]:
 bpy.ops.mesh.primitive_torus_add(major_radius=major,minor_radius=minor,major_segments=128,minor_segments=12,location=(0,0,z));o=bpy.context.object;o.name=name;o.data.materials.append(mat)
 for f in o.data.polygons:f.use_smooth=True
bpy.ops.mesh.primitive_cylinder_add(vertices=128,radius=.107,depth=.003,location=(0,0,.004));bpy.context.object.name='ClosedBottom';bpy.context.object.data.materials.append(mat)
asset_objects=list(bpy.context.scene.objects)
bpy.ops.object.select_all(action='DESELECT')
for o in asset_objects:o.select_set(True)
bpy.context.view_layer.objects.active=obj
bpy.ops.export_scene.obj(filepath=str(R/'trash_bin.obj'),use_selection=True,use_materials=True,axis_forward='Y',axis_up='Z',use_triangles=True)
# Set up an isolated CPU studio preview, not a scene deployment.
bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,0));floor=bpy.context.object;floor.name='PreviewFloor';fm=bpy.data.materials.new('StudioFloor');fm.diffuse_color=(.45,.46,.48,1);floor.data.materials.append(fm)
world=bpy.context.scene.world;world.use_nodes=True;world.node_tree.nodes['Background'].inputs[0].default_value=(.65,.68,.72,1);world.node_tree.nodes['Background'].inputs[1].default_value=.5
for pos,power,size in [((.5,-.4,.8),70,.6),((-.4,.1,.5),45,.4)]:
 bpy.ops.object.light_add(type='AREA',location=pos);o=bpy.context.object;o.data.energy=power;o.data.size=size;o.rotation_euler=(Vector((0,0,.15))-o.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(.48,-.63,.49));cam=bpy.context.object;cam.rotation_euler=(Vector((0,0,.145))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.lens=55;bpy.context.scene.camera=cam
scene=bpy.context.scene;scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=32;scene.render.threads_mode='FIXED';scene.render.threads=3;scene.render.resolution_x=900;scene.render.resolution_y=900;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.filepath=str(R/'preview.png')
# Keep source inspectable; no trash bag, no contents.
bpy.ops.wm.save_as_mainfile(filepath=str(R/'trash_bin.blend'))
print('BIN_GEOMETRY_DONE')
