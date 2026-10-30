from pathlib import Path
import bpy,json,math,shutil
from mathutils import Vector
S=Path('/project/office_assets/refresh');R=Path('/assets/assets/office_refresh');R.mkdir(parents=True,exist_ok=True)
def mat(name,color,rough=.5,metal=0):
 m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True;p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*color,1);p.inputs['Roughness'].default_value=rough;p.inputs['Metallic'].default_value=metal;return m
def finish(o,m):
 o.data.materials.append(m)
 for p in o.data.polygons:p.use_smooth=True
 return o
def lathe(name,profile,m):
 pts=[];faces=[];N=96
 for r,z in profile:
  for i in range(N):a=2*math.pi*i/N;pts.append((r*math.cos(a),r*math.sin(a),z))
 for j in range(len(profile)-1):
  for i in range(N):a=j*N+i;b=j*N+(i+1)%N;faces.append((a,b,b+N,a+N))
 me=bpy.data.meshes.new(name);me.from_pydata(pts,[],faces);me.update();o=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(o);return finish(o,m)
def box(name,loc,size,m,bevel=.001):
 bpy.ops.mesh.primitive_cube_add(size=1,location=loc);o=bpy.context.object;o.name=name;o.dimensions=size;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);finish(o,m)
 mod=o.modifiers.new('Rounded edges','BEVEL');mod.width=bevel;mod.segments=3;bpy.context.view_layer.objects.active=o;bpy.ops.object.modifier_apply(modifier=mod.name)
 return o
def text(label,loc,size,m,rot=(0,0,0)):
 bpy.ops.object.text_add(location=loc,rotation=rot);o=bpy.context.object;o.data.body=label;o.data.align_x='CENTER';o.data.size=size;o.data.extrude=.00008;o.data.materials.append(m);bpy.ops.object.convert(target='MESH')
def cyl(r,h,z):return dict(type='cylinder',radius=r,length=h,pos=[0,0,z],euler=[0,0,0])
def bx(size,pos):return dict(type='box',size=size,pos=pos,euler=[0,0,0])
spec=[('khronos_water_bottle','金属运动水瓶',.15),('gso_white_mug','白色浮点陶瓷杯',.30),('gso_blue_mug','蓝色陶瓷马克杯',.30),('takeaway_coffee','外带热咖啡杯 · 纸套',.035),('iced_coffee','冷咖啡杯 · 吸管',.04),('pizza_box','披萨外卖盒 · 闭合',.16),('pet_water_bottle','瓶装水 · 密封瓶',.52)]
for slug,title,mass in spec:
 bpy.ops.wm.read_factory_settings(use_empty=True);D=R/slug;D.mkdir(exist_ok=True);source={};collision=[]
 if slug.startswith('gso_'):
  src=S/slug;bpy.ops.import_scene.obj(filepath=str(src/'meshes/model.obj'),axis_forward='Y',axis_up='Z');source=json.loads((src/'source.json').read_text())
  for image in bpy.data.images:
   if image.source=='FILE':image.filepath=str(src/'materials/textures/texture.png');image.reload()
 elif slug.startswith('khronos'):
  bpy.ops.import_scene.gltf(filepath=str(S/slug/'source.glb'));source=json.loads((S/slug/'source.json').read_text());shutil.copyfile(S/slug/'LICENSE_source.md',D/'LICENSE_source.md')
 else:
  source=dict(url='FQ authored procedural asset',license='FQ authored',author='FQ');white=mat('Paper ivory',(.92,.90,.85),.86);black=mat('Lid charcoal',(.025,.03,.028),.35);kraft=mat('Corrugated kraft',(.46,.29,.14),.93);green=mat('Cafe green',(.025,.18,.10),.7);coffee=mat('Iced coffee',(.38,.20,.09),.38)
  if slug=='takeaway_coffee':
   lathe('Tapered paper cup',[(0,0),(.028,0),(.029,.003),(.044,.11),(.043,.112),(.042,.11),(.027,.004),(0,.004)],white)
   lathe('Paper sleeve',[(.034,.033),(.040,.073),(.041,.073),(.035,.033),(.034,.033)],kraft)
   lathe('Lid',[(0,.111),(.045,.111),(.046,.114),(.043,.118),(.038,.119),(.037,.124),(0,.124)],black)
   text('COFFEE',(0,-.0385,.053),.008,green,(math.pi/2,0,0));collision=[cyl(.037,.11,.055),cyl(.046,.013,.1175)]
  elif slug=='iced_coffee':
   cream=mat('Cup rim',(.86,.83,.73),.3);lathe('Cold drink cup',[(0,0),(.028,0),(.03,.006),(.044,.125),(.043,.127),(.041,.124),(.027,.005),(0,.005)],coffee)
   lathe('Lid clear rim proxy',[(0,.126),(.045,.126),(.046,.13),(.043,.133),(0,.133)],cream)
   lathe('Green straw',[(.003,.129),(.003,.205),(.002,.205),(.002,.129)],green)
   text('ICED',(0,-.040,.080),.012,white,(math.pi/2,0,0));collision=[cyl(.038,.127,.0635),cyl(.046,.008,.130),cyl(.003,.076,.167)]
  elif slug=='pizza_box':
   box('Box base',(0,0,.018),(.32,.32,.036),kraft,.0015);box('Folding lid',(0,0,.038),(.326,.326,.004),white,.001)
   text('PIZZA',(0,.015,.0403),.047,green);text('FRESH FROM THE OVEN',(0,-.025,.0403),.009,green)
   for x in [-.11,.11]:box('Fold seam',(x,-.1603,.018),(.001,.0005,.025),black,.0001)
   collision=[bx([.326,.326,.040],[0,0,.020])]
  else:
   plastic=mat('Blue tinted PET',(.48,.70,.80),.22);cap=mat('Blue screw cap',(.025,.20,.55),.35);label=mat('White water label',(.9,.94,.96),.62)
   profile=[(0,0),(.027,0),(.032,.008)]
   for i in range(18):profile.append((.032-(.0017 if i%2 else 0),.012+i*.007))
   profile +=[(.032,.14),(.027,.155),(.014,.17),(.014,.19),(0,.19)]
   lathe('Ribbed PET bottle',profile,plastic);lathe('Printed label',[(.0324,.05),(.0324,.10)],label);lathe('Screw cap',[(0,.183),(.016,.183),(.016,.202),(0,.202)],cap)
   text('WATER',(0,-.0328,.071),.008,cap,(math.pi/2,0,0));collision=[cyl(.032,.14,.07),cyl(.025,.03,.155),cyl(.016,.032,.186)]
 # Center and ground visual geometry; preserve source scale except WaterBottle's documented demo scale.
 objs=[o for o in bpy.context.scene.objects if o.type=='MESH'];bpy.context.view_layer.update();pts=[o.matrix_world@v.co for o in objs for v in o.data.vertices]
 lo=Vector([min(p[i] for p in pts) for i in range(3)]);hi=Vector([max(p[i] for p in pts) for i in range(3)])
 scale=.235/(hi.z-lo.z) if slug.startswith('khronos') else 1.;offset=Vector(((lo.x+hi.x)/2,(lo.y+hi.y)/2,lo.z))
 for o in objs:
  mw=o.matrix_world.copy()
  for v in o.data.vertices:v.co=(mw@v.co-offset)*scale
  o.matrix_world.identity()
 bpy.context.view_layer.update();dims=list((hi-lo)*scale)
 if slug.startswith('khronos'):collision=[cyl(min(dims[:2])*.48,dims[2]*.85,dims[2]*.425),cyl(min(dims[:2])*.31,dims[2]*.15,dims[2]*.925)]
 if slug.startswith('gso_'):
  # Separate scan/handle collisions into spatial cells; each cell is a convex mesh.
  # Preserve the mug cavity and handle opening rather than applying a single enclosing hull.
  pass
 parts=[]
 for oi,o in enumerate(objs):
  o.data.calc_loop_triangles();o.data.calc_normals_split()
  for mi in sorted(set(p.material_index for p in o.data.polygons)):
   tris=[t for t in o.data.loop_triangles if t.material_index==mi]
   if not tris:continue
   mid=len(parts);name='part_'+str(mid);m=o.data.materials[mi] if o.data.materials else None;color=list(m.diffuse_color[:3]) if m else [.7]*3;rough=.6;tex=None
   if m and m.use_nodes:
    bs=next((n for n in m.node_tree.nodes if n.type=='BSDF_PRINCIPLED'),None)
    if bs:
     color=list(bs.inputs['Base Color'].default_value[:3]);rough=bs.inputs['Roughness'].default_value
     if bs.inputs['Base Color'].links:
      node=bs.inputs['Base Color'].links[0].from_node
      if node.type=='TEX_IMAGE' and node.image:
       im=node.image.copy();im.scale(min(im.size[0],2048),min(im.size[1],2048));tex=name+'.png';im.filepath_raw=str(D/tex);im.file_format='PNG';im.save()
   pp=[list(v.co) for v in o.data.vertices];ff=[list(t.vertices) for t in tris];uv=[];nn=[]
   for t in tris:
    for li in t.loops:uv.append(list(o.data.uv_layers.active.data[li].uv) if o.data.uv_layers.active else [0,0]);nn.append(list(o.data.loops[li].normal))
   with (D/(name+'.obj')).open('w') as f:
    for p in pp:f.write('v '+' '.join(map(str,p))+'\n')
    for p in uv:f.write('vt '+' '.join(map(str,p))+'\n')
    for p in nn:f.write('vn '+' '.join(map(str,p))+'\n')
    for ti,t in enumerate(ff):f.write('f '+' '.join(f'{v+1}/{ti*3+j+1}/{ti*3+j+1}' for j,v in enumerate(t))+'\n')
   parts.append(dict(file=name+'.obj',points=pp,faces=ff,uvs=uv,normals=nn,color=color,roughness=rough,texture=tex))
 if slug.startswith('gso_'):
  # Assign triangles to small 3D cells and keep original triangles within each hull.
  import numpy as np
  p=parts[0];xyz=np.array(p['points']);faces=np.array(p['faces']);cent=xyz[faces].mean(1);cells=np.floor(cent/.012).astype(int)
  for cell in np.unique(cells,axis=0):
   fs=faces[np.all(cells==cell,axis=1)];ids=np.unique(fs);vv=xyz[ids]
   if len(ids)<4 or np.linalg.matrix_rank(vv-vv.mean(0))<3:continue
   idx={int(v):i for i,v in enumerate(ids)};ff=[[idx[int(v)] for v in face] for face in fs];name='collision_'+str(len(collision))+'.obj'
   collision.append(dict(type='mesh',file=name,points=vv.tolist(),faces=ff,pos=[0,0,0],euler=[0,0,0]))
 bpy.ops.object.select_all(action='DESELECT')
 for o in objs:o.select_set(True)
 bpy.ops.export_scene.gltf(filepath=str(D/'model.glb'),export_format='GLB',use_selection=True)
 data=dict(slug=slug,title=title,mass_kg=mass,parts=parts,collisions=collision,dimensions_m=dims,source=source,scale_applied=scale)
 (D/'geometry.json').write_text(json.dumps(data));print('BUILT',slug,dims,len(collision),flush=True)
