"""Photo-guided office chair candidate. All dimensions/materials are estimates."""
import bpy, math, json, sys
from pathlib import Path
from mathutils import Vector
R=Path(sys.argv[sys.argv.index('--')+1]);R.mkdir(parents=True,exist_ok=True)
(R/'textures').mkdir(exist_ok=True);(R/'meshes').mkdir(exist_ok=True);(R/'previews').mkdir(exist_ok=True)
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
def mat(name,c,rough,metal=0):
 m=bpy.data.materials.new(name);m.diffuse_color=(*c,1);m.use_nodes=True;p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*c,1);p.inputs['Roughness'].default_value=rough;p.inputs['Metallic'].default_value=metal;return m
fabric=mat('GreyWovenFabric',(.32,.335,.325),.86)
rear=mat('DarkRearTextile',(.035,.043,.040),.9)
frame=mat('WarmGreyCoatedFrame',(.70,.72,.67),.48)
pad=mat('ArmrestSoftPad',(.57,.61,.55),.57)
black=mat('BlackMechanism',(.018,.022,.021),.36)
wheel=mat('BlackWheelTread',(.025,.028,.027),.7)
chrome=mat('GasLiftSteel',(.52,.56,.58),.19,1)
seam=mat('FabricSeams',(.39,.42,.39),.9)
import numpy as np
N=512;y,x=np.mgrid[0:N,0:N];a=x*2*math.pi/16;b=y*2*math.pi/16
weave=.5+.25*np.sin(a)+.25*np.sin(b)
for kind in ['basecolor','roughness','normal']:
 if kind=='basecolor':rgb=np.stack([(.32+.028*(weave-.5)),(.335+.028*(weave-.5)),(.325+.028*(weave-.5))],axis=2)
 elif kind=='roughness':rgb=np.repeat((.82+.1*weave)[:,:,None],3,axis=2)
 else:
  nx=.17*np.cos(a);ny=.17*np.cos(b);nz=np.ones_like(nx);norm=np.sqrt(nx*nx+ny*ny+nz*nz);rgb=np.stack([.5+.5*nx/norm,.5+.5*ny/norm,.5+.5*nz/norm],axis=2)
 # Write PNG bytes directly: this Blender build lacks OpenColorIO.
 import zlib, struct
 if kind=='basecolor':rgb=np.where(rgb<=.0031308,12.92*rgb,1.055*rgb**(1/2.4)-.055)
 pixels=np.uint8(np.clip(rgb,0,1)*255)
 def chunk(t,d):return struct.pack('>I',len(d))+t+d+struct.pack('>I',zlib.crc32(t+d)&0xffffffff)
 path=R/'textures'/('fabric_'+kind+'.png')
 path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>2I5B',N,N,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(b''.join(b'\0'+row.tobytes() for row in pixels)))+chunk(b'IEND',b''))
 im=bpy.data.images.load(str(path));tx=fabric.node_tree.nodes.new('ShaderNodeTexImage');tx.image=im
 if kind!='basecolor':im.colorspace_settings.name='Linear'
 p=fabric.node_tree.nodes.get('Principled BSDF')
 if kind=='normal':
  nm=fabric.node_tree.nodes.new('ShaderNodeNormalMap');nm.inputs['Strength'].default_value=.25;fabric.node_tree.links.new(tx.outputs['Color'],nm.inputs['Color']);fabric.node_tree.links.new(nm.outputs['Normal'],p.inputs['Normal'])
 else:fabric.node_tree.links.new(tx.outputs['Color'],p.inputs['Base Color' if kind=='basecolor' else 'Roughness'])
objects=[];collisions=[]
def finish(o,name,m):
 o.name=name;o.data.materials.append(m);objects.append(o)
 if o.type=='MESH':
  o.data.use_auto_smooth=True
  for p in o.data.polygons:p.use_smooth=True
 return o
def box(name,pos,size,m,bevel=.008):
 bpy.ops.mesh.primitive_cube_add(size=1,location=pos);o=bpy.context.object;o.dimensions=size;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
 if bevel:
  mod=o.modifiers.new('Soft manufactured edges','BEVEL');mod.width=bevel;mod.segments=4
  mod=o.modifiers.new('Face normals','WEIGHTED_NORMAL');mod.keep_sharp=True
 return finish(o,name,m)
def cyl(name,pos,r,depth,m,axis=(0,0,1)):
 bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=r,depth=depth,location=pos);o=bpy.context.object;o.rotation_euler=Vector(axis).to_track_quat('Z','Y').to_euler();mod=o.modifiers.new('Rim bevel','BEVEL');mod.width=.002;mod.segments=3;o.modifiers.new('Normals','WEIGHTED_NORMAL');return finish(o,name,m)
def rod(name,pts,r,m):
 cu=bpy.data.curves.new(name,'CURVE');cu.dimensions='3D';cu.resolution_u=16;cu.bevel_depth=r;cu.bevel_resolution=4;s=cu.splines.new('BEZIER');s.bezier_points.add(len(pts)-1)
 for bp,p in zip(s.bezier_points,pts):bp.co=p;bp.handle_left_type='AUTO';bp.handle_right_type='AUTO'
 o=bpy.data.objects.new(name,cu);bpy.context.collection.objects.link(o);return finish(o,name,m)
def beam(name,p1,p2,width,depth,m):
 delta=Vector(p2)-Vector(p1);o=box(name,(Vector(p1)+Vector(p2))/2,(width,depth,delta.length),m,min(width,depth)*.24);o.rotation_euler=delta.to_track_quat('Z','Y').to_euler();return o

def strap(name,pts,widths,depth,m):
 # Rounded rectangular loft along a smooth molded frame centerline.
 pts=[Vector(p) for p in pts];samples=[];ww=[]
 for i in range(len(pts)-1):
  a=pts[max(0,i-1)];b=pts[i];c=pts[i+1];d=pts[min(len(pts)-1,i+2)]
  for j in range(12):
   t=j/12;samples.append(.5*((2*b)+(-a+c)*t+(2*a-5*b+4*c-d)*t*t+(-a+3*b-3*c+d)*t*t*t));ww.append(widths[i]*(1-t)+widths[i+1]*t)
 samples.append(pts[-1]);ww.append(widths[-1]);v=[];f=[];n=24
 for i,center in enumerate(samples):
  tangent=samples[min(i+1,len(samples)-1)]-samples[max(0,i-1)];across=Vector((tangent.z,0,-tangent.x)).normalized()
  for j in range(n):
   ang=j*2*math.pi/n;aa=math.copysign(abs(math.cos(ang))**.5,math.cos(ang));bb=math.copysign(abs(math.sin(ang))**.5,math.sin(ang));v.append(tuple(center+across*(ww[i]*.5*aa)+Vector((0,depth*.5*bb,0))))
 for i in range(len(samples)-1):
  for j in range(n):f.append((i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j))
 f.extend([tuple(range(n))[::-1],tuple((len(samples)-1)*n+j for j in range(n))]);me=bpy.data.meshes.new(name);me.from_pydata(v,[],f);me.update();o=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(o);finish(o,name,m)
 bpy.ops.object.select_all(action='DESELECT');o.select_set(True);bpy.context.view_layer.objects.active=o;bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT');bpy.ops.mesh.normals_make_consistent(inside=False);bpy.ops.object.mode_set(mode='OBJECT');o.select_set(False)
 return o

def cushion(name,w,h,thickness,mapper,m):
 # Two rounded-rectangle surface patches joined at a soft perimeter.
 v=[];f=[];seg=96;rings=9
 for side in [1,-1]:
  for k in range(rings):
   rr=(k+.04)/(rings-1+.04)
   for j in range(seg):
    a=2*math.pi*j/seg;u=math.copysign(abs(math.cos(a))**.5,math.cos(a))*rr;vv=math.copysign(abs(math.sin(a))**.5,math.sin(a))*rr
    depth=side*thickness*.5*(.7+.3*math.sqrt(max(.0,1-rr**8)) if name=='SeatCushion' else math.sqrt(max(.0,1-rr**8)))
    v.append(mapper(u*w/2,vv*h/2,depth,rr))
  base=(0 if side==1 else rings*seg)
  for k in range(rings-1):
   for j in range(seg):
    ids=(base+k*seg+j,base+k*seg+(j+1)%seg,base+(k+1)*seg+(j+1)%seg,base+(k+1)*seg+j);f.append(ids if side==1 else ids[::-1])
  f.append(tuple(base+j for j in range(seg))[::-1] if side==1 else tuple(base+j for j in range(seg)))
 for j in range(seg):f.append(((rings-1)*seg+j,(rings-1)*seg+(j+1)%seg,(2*rings-1)*seg+(j+1)%seg,(2*rings-1)*seg+j))
 me=bpy.data.meshes.new(name);me.from_pydata(v,[],f);me.update();o=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(o);finish(o,name,m)
 # Recalculate outward normals, then UV unwrap. Separate geometry remains editable.
 bpy.context.view_layer.objects.active=o;o.select_set(True);bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT');bpy.ops.mesh.remove_doubles(threshold=.000001);bpy.ops.mesh.normals_make_consistent(inside=False);bpy.ops.uv.smart_project(island_margin=.02);bpy.ops.object.mode_set(mode='OBJECT');o.select_set(False)
 for uv in me.uv_layers.active.data:uv.uv*=4
 return o
# Front is -Y; floor origin at the gas lift axis. Fixed reference seat height ~0.49 m.
cushion('SeatCushion',.49,.47,.085,lambda x,y,d,r:(x,y-.015,.445+d+.009*r*r),fabric)
cushion('UpperBackCushion',.465,.46,.040,lambda x,z,d,r:(x,.225+.21*(z+.23)-.030*(x/.2325)**2-d,z+.935),fabric)
cushion('LumbarCushion',.45,.245,.055,lambda x,z,d,r:(x,.173+.045*(z/.1225)**2-.024*(x/.225)**2-d,z+.65),fabric)
cushion('RearLumbarTextile',.411,.214,.010,lambda x,z,d,r:(x,.218+.025*(z/.107)**2-d,z+.64),rear)
# The dark back layer is visible through the frame openings, not a solid shell.
cushion('RearTextile',.444,.445,.009,lambda x,z,d,r:(x,.249+.21*(z+.2225)-.030*(x/.222)**2-d,z+.935),rear)
rod('SeatStitch',[(-.22,-.20,.449),(-.12,-.249,.455),(.12,-.249,.455),(.22,-.20,.449)],.0008,seam)
for side in [-1,1]:
 strap('BackOuterFrame_'+str(side),[(side*.17,.19,.53),(side*.218,.22,.67),(side*.23,.245,.83),(side*.235,.285,1.08),(side*.19,.295,1.157)],[.038,.034,.028,.027,.025],.024,frame)
 strap('LumbarLoop_'+str(side),[(side*.15,.205,.525),(side*.215,.18,.58),(side*.224,.17,.68),(side*.195,.20,.755)],[.038,.038,.035,.032],.026,frame)
 strap('YBranch_'+str(side),[(0,.324,.88),(side*.065,.326,.965),(side*.155,.323,1.045),(side*.213,.313,1.095)],[.08,.065,.045,.036],.028,frame)
 strap('LowerBackCross_'+str(side),[(0,.29,.62),(side*.10,.267,.602),(side*.19,.217,.61)],[.036,.036,.032],.027,frame)
 strap('ArmUpright_'+str(side),[(side*.289,.015,.395),(side*.289,.015,.57),(side*.289,.015,.64),(side*.289,-.006,.674)],[.053,.048,.061,.088],.065,frame)
 beam('ArmUnderSeatMount_'+str(side),(side*.12,.04,.39),(side*.288,.04,.41),.045,.043,black)
 box('ArmrestBase_'+str(side),(side*.289,-.035,.69),(.089,.266,.035),frame,.016)
 box('ArmrestPad_'+str(side),(side*.289,-.035,.711),(.083,.256,.020),pad,.009)
 box('ArmHeightButton_'+str(side),(side*.317,-.006,.613),(.003,.014,.060),black,.003)
rod('BackTopFrame',[(-.19,.295,1.157),(0,.31,1.175),(.19,.295,1.157)],.015,frame)
strap('YSpineLower',[(0,.19,.39),(0,.27,.48),(0,.312,.62),(0,.325,.78)],[.10,.108,.092,.079],.034,frame)
strap('YSpineUpper',[(0,.325,.72),(0,.327,.84),(0,.324,.925)],[.079,.088,.127],.030,frame)
# Subtle rear honeycomb emboss, not open holes.
for row in range(13):
 for col in range(4):
  z=.67+row*.018;x=(col-1.5)*.015+(row%2)*.0075
  pts=[(x+.009*math.cos(k*math.pi/3),.344,z+.009*math.sin(k*math.pi/3)) for k in range(7)];rod('RearHex_%d_%d'%(row,col),pts,.00055,frame)
box('UnderSeatMechanism',(0,.04,.377),(.205,.235,.065),black,.012)
rod('HeightLever',[(.08,.0,.37),(.20,-.035,.34),(.255,-.07,.34)],.007,black)
box('HeightLeverPaddle',(.26,-.075,.342),(.075,.048,.012),black,.006)
cyl('GasLiftChrome',(0,0,.29),.022,.145,chrome)
cyl('GasLiftSleeve',(0,0,.177),.032,.15,black)
cyl('GasLiftBottom',(0,0,.086),.026,.070,black)
cyl('FiveStarHub',(0,0,.147),.065,.05,frame)
for k in range(5):
 a=math.radians(18)+k*2*math.pi/5;u=Vector((math.cos(a),math.sin(a),0));t=Vector((-u.y,u.x,0));p=u*.315
 # Tapered solid five-star spoke.
 v=[]
 for rad,w,z,hh in [(.035,.047,.149,.030),(.315,.020,.092,.016)]:
  for sy,sz in [(-1,-1),(1,-1),(1,1),(-1,1)]:v.append(tuple(u*rad+t*(sy*w)+Vector((0,0,z+sz*hh))))
 f=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)];me=bpy.data.meshes.new('Spoke');me.from_pydata(v,[],f);me.update();o=bpy.data.objects.new('StarSpoke_'+str(k),me);bpy.context.collection.objects.link(o);finish(o,o.name,frame);mod=o.modifiers.new('Cast edge','BEVEL');mod.width=.009;mod.segments=4;o.modifiers.new('Normals','WEIGHTED_NORMAL')
 cyl('CasterStem_'+str(k),(*p[:2],.078),.010,.035,chrome)
 cp=p+u*.012
 box('CasterFork_'+str(k),(*cp[:2],.053),(.043,.037,.034),black,.008)
 for side in [-1,1]:
  center=cp+t*(side*.015)+Vector((0,0,.032));cyl('CasterWheel_%d_%d'%(k,side),center,.032,.024,wheel,t)
  cyl('CasterHub_%d_%d'%(k,side),center+t*(side*.0125),.020,.0015,black,t)
 # Collision spokes and casters are coarse but leave the star gaps open.
 collisions.append({'name':'spoke_'+str(k),'type':'capsule','fromto':[*(u*.045+Vector((0,0,.15))),*(u*.31+Vector((0,0,.092)))],'size':[.025]})
 collisions.append({'name':'caster_'+str(k),'type':'sphere','pos':[float(cp.x),float(cp.y),.032],'size':[.032]})
collisions += [dict(name='seat',type='box',pos=[0,-.015,.446],size=[.235,.225,.038]),dict(name='back',type='box',pos=[0,.25,.935],size=[.22,.024,.22],euler=[-.205,0,0]),dict(name='lumbar',type='box',pos=[0,.155,.67],size=[.19,.035,.095]),dict(name='gaslift',type='cylinder',pos=[0,0,.205],size=[.033,.15])]
for s in [-1,1]:
 collisions += [dict(name='arm_'+str(s),type='box',pos=[s*.289,-.035,.70],size=[.044,.132,.025]),dict(name='arm_post_'+str(s),type='box',pos=[s*.289,.015,.535],size=[.027,.034,.14])]
# Convert all visual pieces, apply transforms and modifiers; keep them separately named.
bpy.ops.object.select_all(action='DESELECT')
for o in objects:o.select_set(True)
bpy.context.view_layer.objects.active=objects[0];bpy.ops.object.convert(target='MESH');objects=list(bpy.context.selected_objects)
bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
for o in objects:
 if not o.data.uv_layers:
  bpy.ops.object.select_all(action='DESELECT');o.select_set(True);bpy.context.view_layer.objects.active=o;bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT');bpy.ops.uv.smart_project(island_margin=.02);bpy.ops.object.mode_set(mode='OBJECT')
materials=list({o.data.materials[0].name:o.data.materials[0] for o in objects}.values())
(R/'pbr.json').write_text(json.dumps({m.name:{'base_color':list(m.diffuse_color[:3]),'roughness':m.node_tree.nodes.get('Principled BSDF').inputs['Roughness'].default_value,'metallic':m.node_tree.nodes.get('Principled BSDF').inputs['Metallic'].default_value} for m in materials},indent=2))
(R/'pbr.json').write_text(json.dumps({m.name:{'base_color':list(m.diffuse_color[:3]),'roughness':float(m.node_tree.nodes.get('Principled BSDF').inputs['Roughness'].default_value),'metallic':float(m.node_tree.nodes.get('Principled BSDF').inputs['Metallic'].default_value)} for m in materials},indent=2))
import xml.etree.ElementTree as ET
mj=ET.Element('mujoco',model='office_chair_candidate');ET.SubElement(mj,'compiler',angle='radian');ast=ET.SubElement(mj,'asset');wb=ET.SubElement(mj,'worldbody');body=ET.SubElement(wb,'body',name='office_chair')
for m in materials:ET.SubElement(ast,'material',name=m.name,rgba=' '.join(str(v) for v in m.diffuse_color))
allcoords=[];triangles=0
for idx,o in enumerate(objects):
 me=o.data;me.calc_loop_triangles();fn='part_%03d.obj'%idx;lines=['o '+o.name]
 allcoords.extend([list(v.co) for v in me.vertices]);lines+=['v '+' '.join('%.9g'%x for x in v.co) for v in me.vertices]
 # Face-corner UVs and normals avoid ambiguity at seams.
 for tri in me.loop_triangles:
  for li in tri.loops:lines.append('vt %.9g %.9g'%tuple(me.uv_layers.active.data[li].uv))
 for tri in me.loop_triangles:
  for vi in tri.vertices:lines.append('vn '+' '.join('%.9g'%x for x in me.vertices[vi].normal))
 for j,tri in enumerate(me.loop_triangles):lines.append('f '+' '.join('%d/%d/%d'%(vi+1,j*3+k+1,j*3+k+1) for k,vi in enumerate(tri.vertices)))
 (R/'meshes'/fn).write_text('\n'.join(lines)+'\n');triangles+=len(me.loop_triangles)
 name='part_%03d'%idx;ET.SubElement(ast,'mesh',name=name,file='meshes/'+fn);ET.SubElement(body,'geom',name=o.name,mesh=name,type='mesh',material=o.data.materials[0].name,contype='0',conaffinity='0',group='2',mass='0')
for c in collisions:ET.SubElement(body,'geom',**{k:(' '.join(map(str,v)) if isinstance(v,list) else v) for k,v in c.items()},group='3',rgba='0 0 0 0',friction='.7 .005 .0001',mass='0')
ET.indent(mj);ET.ElementTree(mj).write(R/'chair.xml',encoding='unicode')
bpy.ops.object.select_all(action='DESELECT')
for o in objects:o.select_set(True)
bpy.ops.export_scene.gltf(filepath=str(R/'chair.glb'),export_format='GLB',use_selection=True)
# Bounds and uncertainty travel with the source, not just in the chat.
coords=np.asarray(allcoords);dims=(coords.max(0)-coords.min(0)).tolist()
report={'schema_version':1,'status':'photo_estimate_pending_dimension_calibration','units':'meter','front':'-Y','up':'Z','dimensions_m':dims,'seat_height_estimate_m':.49,'caster_diameter_estimate_m':.064,'parts':len(objects),'triangles':triangles,'collision_parts':len(collisions),'fixed_pose':True,'mass_and_joint_parameters':'not_calibrated','visual_revision':'v2_molded_frame_cushion_refinement','reference':'Six user photos supplied 2026-10-23; foreground chair only','limitations':['No measured dimensions yet','Hidden mechanism is simplified','Fabric is rigid geometry, not a deformable simulation','No rolling or swivel dynamics enabled']}
(R/'candidate.json').write_text(json.dumps(report,indent=2)+'\n');(R/'collision_geometry.json').write_text(json.dumps(collisions,indent=2)+'\n')
# Neutral studio: CPU-only Cycles, no production-scene modification.
bpy.ops.mesh.primitive_plane_add(size=200);o=bpy.context.object;o.name='PREVIEW_GROUND';o.data.materials.append(mat('Studio',(.29,.31,.32),.8))
scene.world.use_nodes=True;scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.60,.66,.72,1);scene.world.node_tree.nodes['Background'].inputs[1].default_value=.3
for loc,power,size in [((1.6,-2.1,2.8),400,2.0),((-1.7,-.5,1.8),220,1.8),((.8,1.7,2.5),330,1.5)]:
 bpy.ops.object.light_add(type='AREA',location=loc);o=bpy.context.object;o.data.energy=power;o.data.shape='DISK';o.data.size=size;o.rotation_euler=(Vector((0,0,.65))-o.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add();cam=bpy.context.object;scene.camera=cam;cam.data.type='ORTHO';cam.data.ortho_scale=1.42
scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=24;scene.cycles.use_denoising=True;scene.render.threads_mode='FIXED';scene.render.threads=2;scene.render.resolution_x=800;scene.render.resolution_y=900;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG'
scene.view_settings.view_transform='Standard';scene.view_settings.exposure=0;scene.view_settings.gamma=1
for name,loc in [('front_threequarter',(1.6,-2.5,1.5)),('rear_threequarter',(-1.5,2.4,1.6)),('side',(2.7,0,1.05))]:
 cam.location=loc;cam.rotation_euler=(Vector((0,0,.60))-cam.location).to_track_quat('-Z','Y').to_euler();scene.render.filepath=str(R/'previews'/f'{name}.png');pass # Preview images captured from isolated Three.js viewer; this Blender build lacks OpenColorIO.
cam.location=(1.6,-2.5,1.5);cam.rotation_euler=(Vector((0,0,.60))-cam.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(R/'chair.blend'))
print('CHAIR_CANDIDATE_COMPLETE',json.dumps(report),flush=True)
