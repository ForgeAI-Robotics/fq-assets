"""Run with Blender --background --python build_basket.py -- /absolute/output.
Geometry is generated in metres. Simulation collisions are exported separately.
"""
import bpy, bmesh, math, json, sys
from pathlib import Path
from mathutils import Vector,kdtree
from math import sin, cos, pi, sqrt

ROOT=Path(sys.argv[sys.argv.index('--')+1] if '--' in sys.argv else __file__).resolve()
if ROOT.is_file(): ROOT=ROOT.parent
P=json.loads((ROOT/'parameters.json').read_text())
for folder in ['meshes','previews','evidence']: (ROOT/folder).mkdir(exist_ok=True,parents=True)
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene
scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1

def material(name,color,rough=.29):
    m=bpy.data.materials.new(name); m.diffuse_color=(*color,1); m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(*color,1)
    bs.inputs['Roughness'].default_value=rough
    return m
white=material('Warm white polypropylene',(0.83,.85,.79),.26)
gray=material('Grey polypropylene handles',(.15,.17,.16),.3)
dark=material('Printed dark grey mark',(.045,.048,.043),.5)

def mesh(name,verts,faces,mat,smooth=True):
    me=bpy.data.meshes.new(name); me.from_pydata(verts,[],faces); me.update()
    ob=bpy.data.objects.new(name,me); scene.collection.objects.link(ob)
    if mat: me.materials.append(mat)
    bm=bmesh.new(); bm.from_mesh(me); bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-7)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces)); bm.to_mesh(me); bm.free()
    for f in me.polygons: f.use_smooth=smooth
    return ob

def bevel(ob,width,segments=2):
    mod=ob.modifiers.new('Soft injection-moulded edges','BEVEL'); mod.width=width; mod.segments=segments
    bpy.context.view_layer.objects.active=ob
    bpy.ops.object.modifier_apply(modifier=mod.name)

def join(obs,name):
    bpy.ops.object.select_all(action='DESELECT')
    for ob in obs: ob.select_set(True)
    bpy.context.view_layer.objects.active=obs[0]; bpy.ops.object.join()
    obs[0].name=name; return obs[0]

# Counter-clockwise rounded rectangle parameterized by true perimeter fraction.
def rr(s,L,W,r):
    a=L/2-r; b=W/2-r
    lengths=[2*a,pi*r/2,2*b,pi*r/2,2*a,pi*r/2,2*b,pi*r/2]
    d=(s%1)*sum(lengths)
    for i,q in enumerate(lengths):
        if d<=q or i==7: break
        d-=q
    t=d/q
    if i==0: return (-a+2*a*t,-W/2,0,-1)
    if i==2: return (L/2,-b+2*b*t,1,0)
    if i==4: return (a-2*a*t,W/2,0,1)
    if i==6: return (-L/2,b-2*b*t,-1,0)
    centers={1:(a,-b,-pi/2),3:(a,b,0),5:(-a,b,pi/2),7:(-a,-b,pi)}
    cx,cy,angle=centers[i]; angle+=t*pi/2
    return (cx+r*cos(angle),cy+r*sin(angle),cos(angle),sin(angle))

def dims(z):
    t=z/P['height']
    return (P['bottom_length']+(P['length']-.007-P['bottom_length'])*t,
            P['bottom_width']+(P['width']-.007-P['bottom_width'])*t,
            P['corner_radius']*(.79+.21*t))

def wall(s,z,inside=False,ribs=True):
    L,W,r=dims(z); x,y,nx,ny=rr(s,L,W,r)
    offset=-P['wall_thickness'] if inside else 0
    if not inside and ribs:
        fade=min(1,max(0,(z-.006)/.012),max(0,(.198-z)/.012))
        phase=2*pi*(s*P['rib_count']-z*P['rib_count']*.52/(2*(P['length']+P['width'])))
        offset+=P['rib_amplitude']*(.5+.5*cos(phase))*fade
    return (x+nx*offset,y+ny*offset,z)

V=[];F=[];wall_normals={}
def add(v): V.append(v); return len(V)-1
def add_wall(s,z,inside=False):
    p=wall(s,z,inside);eps=1e-6
    ds=Vector(wall(s+eps,z,inside))-Vector(wall(s-eps,z,inside))
    dz=Vector(wall(s,z+eps,inside))-Vector(wall(s,z-eps,inside))
    normal=ds.cross(dz).normalized()*(-1 if inside else 1)
    wall_normals[tuple(round(x,6) for x in p)]=(normal,inside)
    return add(p)
cols=P['hole_columns_perimeter']; rows=P['hole_rows']; zlo=.019; zhi=.192
# Each cell is a perforated patch, tessellated radially to resolve the raised ribs.
outer=[]
corners=[(-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5)]
for a,b in zip(corners,corners[1:]+corners[:1]):
    for k in range(6):
        t=k/6; outer.append((a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t))
n=len(outer); band=(zhi-zlo)/rows; perimeter=2*(P['length']+P['width'])-8*P['corner_radius']+2*pi*P['corner_radius']
for row in range(rows):
    zc=zlo+(row+.5)*band
    for col in range(cols):
        sc=(col+.5)/cols
        stagger=(.25 if row%2==0 else -.25)/cols
        layers=[]
        for inside in [False,True]:
            rings=[]
            for ir in range(5):
                t=ir/4; ring=[]
                for u,v in outer:
                    ox=u/cols; oz=v*band
                    ratio=1/(abs(ox)/(P['hole_width']/2/perimeter)+abs(oz)/(P['hole_height']/2))
                    hx=ox*ratio; hz=oz*ratio
                    ring.append(add_wall(sc+(hx+stagger)*(1-t)+ox*t,zc+hz*(1-t)+oz*t,inside))
                rings.append(ring)
            layers.append(rings)
            for ra,rb in zip(rings,rings[1:]):
                for k in range(n):
                    j=(k+1)%n; F.append([ra[k],ra[j],rb[j],rb[k]])
        for k in range(n):
            j=(k+1)%n; F.append([layers[0][0][k],layers[1][0][k],layers[1][0][j],layers[0][0][j]])

def wall_band(a,b,steps=2):
    N=cols*6; shells=[]
    for inside in [False,True]:
        rings=[[add_wall(i/N,a+(b-a)*j/steps,inside) for i in range(N)] for j in range(steps+1)]
        shells.append(rings)
        for ra,rb in zip(rings,rings[1:]):
            for i in range(N): k=(i+1)%N;F.append([ra[i],ra[k],rb[k],rb[i]])
    for j in [0,-1]:
        for i in range(N): k=(i+1)%N;F.append([shells[0][j][i],shells[1][j][i],shells[1][j][k],shells[0][j][k]])
wall_band(.003,zlo); wall_band(zhi,.202)
shell=mesh('Perforated tapered shell',V,F,white)
normal_entries=list(wall_normals.items());tree=kdtree.KDTree(len(normal_entries))
for i,(co,entry) in enumerate(normal_entries):tree.insert(co,i)
tree.balance()
vertex_entries=[normal_entries[tree.find(v.co)[1]][1] for v in shell.data.vertices]
split_normals=[]
for poly in shell.data.polygons:
    entries=[vertex_entries[k] for k in poly.vertices]
    flat=len(set(e[1] for e in entries))>1
    split_normals.extend([tuple(poly.normal) if flat else tuple(e[0]) for e in entries])
shell.data.normals_split_custom_set(split_normals)
body_parts=[shell]

def ring_solid(name,profiles,mat,N=312,cap=False):
    verts=[];faces=[]
    for L,W,r,z in profiles:
        verts.extend([(rr(i/N,L,W,r)[0],rr(i/N,L,W,r)[1],z) for i in range(N)])
    for j in range(len(profiles) if not cap else len(profiles)-1):
        j2=(j+1)%len(profiles)
        for i in range(N): k=(i+1)%N;faces.append([j*N+i,j*N+k,j2*N+k,j2*N+i])
    if cap:
        faces.append(list(reversed(range(N)))); faces.append([(len(profiles)-1)*N+i for i in range(N)])
    return mesh(name,verts,faces,mat)
L=P['length'];W=P['width'];r=P['corner_radius'];H=P['height'];t=P['wall_thickness']
shoulder_L,shoulder_W,shoulder_r=dims(.202)
rim=ring_solid('Thick upper rim',[(L-.005,W-.005,r,.198),(L,W,r,H-.001),(L-.001,W-.001,r,H),(L-2*t,W-2*t,r-t,H),(shoulder_L-2*t,shoulder_W-2*t,shoulder_r-t,.202)],white)
bevel(rim,.00065,2);body_parts.append(rim)
base=ring_solid('Solid basket floor',[(P['bottom_length']-.001,P['bottom_width']-.001,r*.79,.001),(P['bottom_length'],P['bottom_width'],r*.79,.0055)],white,cap=True)
bevel(base,.0011,3);body_parts.append(base)

def cube(name,center,size,mat,bev=0):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center);ob=bpy.context.object;ob.name=name;ob.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    if mat: ob.data.materials.append(mat)
    if bev: bevel(ob,bev,3)
    return ob

# Handle path surrounds each short end, with pivots on the two long walls.
# Zero angle is horizontal/outward; positive angles raise the grip.
def handle_path():
    R=.028; e=P['handle_extension']; y=W/2+.003
    pts=[]
    for i in range(14): pts.append(((e-R)*i/13,-y))
    for i in range(1,13):
        a=-pi/2+(pi/2)*i/12;pts.append((e-R+R*cos(a),-y+R+R*sin(a)))
    for i in range(1,19): pts.append((e,(-y+R)+2*(y-R)*i/18))
    for i in range(1,13):
        a=(pi/2)*i/12;pts.append((e-R+R*cos(a),y-R+R*sin(a)))
    for i in range(1,14):pts.append(((e-R)*(1-i/13),y))
    return pts
path=handle_path();handles=[];handle_coll=[]
for sign,name in [(1,'handle_right'),(-1,'handle_left')]:
    vs=[];fs=[];hw=P['handle_thickness']/2;ht=P['handle_width']/2
    for i,(x,y) in enumerate(path):
        p0=Vector(path[max(0,i-1)]);p1=Vector(path[min(len(path)-1,i+1)]);d=(p1-p0).normalized();nx=-d.y;ny=d.x
        for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]: vs.append((sign*(x+nx*hw*a),y+ny*hw*a,b*ht))
    for i in range(len(path)-1):
        for k in range(4):fs.append([i*4+k,i*4+(k+1)%4,(i+1)*4+(k+1)%4,(i+1)*4+k])
    fs.extend([[3,2,1,0],[(len(path)-1)*4+k for k in range(4)]])
    ob=mesh(name,vs,fs,gray);bevel(ob,.0012,3)
    ob.location=(sign*P['handle_anchor_x'],0,P['handle_pivot_z']);handles.append(ob)
    # Thin rectangular links preserve the open handle and its broad flat profile.
    pieces=[]
    # Use fewer segments than the visual mesh, preserving curved corners.
    ids=sorted(set([0,6,13,16,19,22,25,31,37,43,46,49,52,55,len(path)-1]))
    ids=[i for i in ids if i<len(path)]
    for ia,ib in zip(ids,ids[1:]):
        a=path[ia];b=path[ib];dx=sign*(b[0]-a[0]);dy=b[1]-a[1]
        pieces.append({'center':[sign*(a[0]+b[0])/2,(a[1]+b[1])/2,0],'size':[sqrt(dx*dx+dy*dy)+.001,P['handle_thickness'],P['handle_width']],'rpy':[0,0,math.atan2(dy,dx)]})
    handle_coll.append(pieces)
    for sy in [-1,1]:
        bpy.ops.mesh.primitive_cylinder_add(vertices=24,radius=.005,depth=.004,location=(sign*P['handle_anchor_x'],sy*(W/2-.001),P['handle_pivot_z']),rotation=(pi/2,0,0))
        pin=bpy.context.object;pin.name='Pivot pin';pin.data.materials.append(gray);bevel(pin,.0006,2);body_parts.append(pin)

body=join(body_parts,'basket_body')
# Discrete mark, as seen in the front reference. Kept independently editable.
bpy.ops.object.text_add(location=(0,-W/2-.001,H-.023),rotation=(pi/2,0,0))
logo=bpy.context.object;logo.name='Front printed bdo mark';logo.data.body='bdo';logo.data.align_x='CENTER';logo.data.size=.013;logo.data.extrude=.00002;logo.data.materials.append(dark)

# Segmented convex trapezoids follow the tapered rounded perimeter; never use a
# single convex hull of the entire basket, which would close the opening.
collision=[];N=48
for i in range(N):
    sv=[i/N,(i+1)/N];verts=[]
    for z in [.003,.229]:
        for inside in [False,True]:
            for s in sv:
                x,y,nx,ny=rr(s,*dims(z)); off=(-.0040 if inside else .0016)
                verts.append([x+off*nx,y+off*ny,z])
    collision.append({'name':f'wall_{i:02d}','vertices':verts,'faces':[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]})
floor_n=48
floor_vertices=[]
for z in [0,.006]:
    floor_vertices.extend([[*rr(i/floor_n,P['bottom_length']-.002,P['bottom_width']-.002,r*.79)[:2],z] for i in range(floor_n)])
floor_faces=[list(reversed(range(floor_n))),[floor_n+i for i in range(floor_n)]]
floor_faces.extend([[i,(i+1)%floor_n,(i+1)%floor_n+floor_n,i+floor_n] for i in range(floor_n)])
collision.insert(0,{'name':'floor','vertices':floor_vertices,'faces':floor_faces})

def export_obj(ob,path):
    bpy.ops.object.select_all(action='DESELECT');ob.select_set(True);bpy.context.view_layer.objects.active=ob
    bpy.ops.wm.obj_export(filepath=str(path),export_selected_objects=True,apply_modifiers=True,export_materials=False,forward_axis='Y',up_axis='Z',export_triangulated_mesh=True)

geometry={'parameters':P,'body_collision':collision,'handle_collision':dict(zip(['handle_right','handle_left'],handle_coll)),'visuals':{}}
for ob in [body,*handles]:
    old=ob.location.copy();ob.location=(0,0,0)
    export_obj(ob,ROOT/'meshes'/f'{ob.name}.obj')
    me=ob.data;me.calc_loop_triangles()
    geometry['visuals'][ob.name]={'vertices':[list(v.co) for v in me.vertices],'triangles':[list(f.vertices) for f in me.loop_triangles], 'triangle_normals':[list(me.corner_normals[i].vector) for f in me.loop_triangles for i in f.loops]}
    ob.location=old
for col in collision:
    if 'vertices' in col:
        ob=mesh(col['name'],col['vertices'],col['faces'],None,False);export_obj(ob,ROOT/'meshes'/f"collision_{col['name']}.obj");bpy.data.objects.remove(ob,do_unlink=True)
(ROOT/'geometry.json').write_text(json.dumps(geometry,separators=(',',':')))
for ob,sign in zip(handles,[1,-1]):
    ob.rotation_euler.y=-sign*P['joint_initial']
    ob['joint_axis_local']=[0,-sign,0];ob['joint_limits_radians']=P['joint_limits'];ob['estimated_mass_kg']=P['handle_mass']
body['dimensions_are_estimated']=True;body['estimated_mass_kg']=P['body_mass']

# Studio scene and three inspection cameras.
ground=cube('Studio floor',(0,0,-.009),(200,200,.016),material('Backdrop',(.16,.195,.22),.72))
def point_at(ob,target): ob.rotation_euler=(Vector(target)-ob.location).to_track_quat('-Z','Y').to_euler()
for name,loc,power,size in [('Large softbox',(.1,-.5,.8),12,.65),('Rim softbox',(-.45,.25,.5),8,.45),('Fill',(.45,.4,.3),5,.4)]:
    da=bpy.data.lights.new(name,'AREA');da.energy=power;da.shape='DISK';da.size=size;ob=bpy.data.objects.new(name,da);scene.collection.objects.link(ob);ob.location=loc;point_at(ob,(0,0,.1))
bpy.ops.object.camera_add(location=(.50,-.64,.49));cam=bpy.context.object;cam.name='Product camera';cam.data.type='ORTHO';cam.data.ortho_scale=.56;point_at(cam,(0,0,.12));scene.camera=cam
scene.render.engine='CYCLES';scene.cycles.samples=48;scene.cycles.use_denoising=True
try:
    prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
    for device in prefs.devices: device.use=device.type!='CPU'
    scene.cycles.device='GPU' if any(d.type!='CPU' for d in prefs.devices) else 'CPU'
except Exception: scene.cycles.device='CPU'
scene.world.color=(.08,.08,.08);scene.render.resolution_x=1400;scene.render.resolution_y=1200;scene.render.resolution_percentage=100
scene.view_settings.view_transform='AgX'
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'basket.blend'))
for name,loc,scale in [('perspective',(.50,-.64,.49),.56),('front',(0,-.8,.31),.53),('top',(.22,-.28,.85),.54)]:
    cam.location=loc;cam.data.ortho_scale=scale;point_at(cam,(0,0,.12));scene.render.filepath=str(ROOT/'previews'/f'{name}.png');bpy.ops.render.render(write_still=True)
cam.location=(.50,-.64,.49);cam.data.ortho_scale=.56;point_at(cam,(0,0,.12))
bpy.ops.object.select_all(action='DESELECT');body.select_set(True);bpy.context.view_layer.objects.active=body
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'basket.blend'))
print('BASKET_BUILD_COMPLETE',len(body.data.vertices),'body vertices',flush=True)
