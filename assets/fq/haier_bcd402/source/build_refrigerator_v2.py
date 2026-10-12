import bpy, math, json, os
from mathutils import Vector
ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, 'output')
os.makedirs(OUT, exist_ok=True)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.length_unit = 'METERS'
asset = bpy.data.collections.new('REFRIGERATOR_BODY_ONLY')
scene.collection.children.link(asset)
studio = bpy.data.collections.new('PREVIEW_STUDIO')
scene.collection.children.link(studio)

def mat(name, color, metal=0, rough=.3, transmission=0):
    m=bpy.data.materials.new(name); m.diffuse_color=(*color,1); m.use_nodes=True
    p=m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Metallic'].default_value=metal
    p.inputs['Roughness'].default_value=rough
    p.inputs['Transmission'].default_value=transmission
    p.inputs['IOR'].default_value=1.47
    return m
steel=mat('Brushed silver metal',(.53,.56,.58),.82,.29)
side=mat('Textured gray cabinet',(.30,.33,.35),.42,.42)
white=mat('Warm white liner',(.84,.86,.84),0,.24)
rubber=mat('Gray gasket',(.16,.18,.18),0,.65)
dark=mat('Dark recessed handles',(.023,.03,.032),.1,.34)
glass=mat('Clear shelf glass',(.78,.89,.87),0,.08,.90)
plastic=mat('Clear door bins',(.84,.90,.93),0,.12,.92)
chrome=mat('Shelf bright trim',(.65,.68,.70),.85,.18)
lightmat=mat('LED diffuser',(.94,1,1),0,.22)
p=lightmat.node_tree.nodes.get('Principled BSDF')
p.inputs['Emission'].default_value=(.8,.94,1,1); p.inputs['Emission Strength'].default_value=2
# Fine directional roughness only; no imaginary contents or photo textures.
n=steel.node_tree.nodes; l=steel.node_tree.links
tex=n.new('ShaderNodeTexNoise'); tex.inputs['Scale'].default_value=170
mapping=n.new('ShaderNodeVectorMath'); mapping.operation='MULTIPLY'; mapping.inputs[1].default_value=(1,1,.035)
coord=n.new('ShaderNodeTexCoord'); l.new(coord.outputs['Generated'],mapping.inputs[0]); l.new(mapping.outputs[0],tex.inputs['Vector'])
bump=n.new('ShaderNodeBump'); bump.inputs['Strength'].default_value=.12; bump.inputs['Distance'].default_value=.00015
l.new(tex.outputs['Fac'],bump.inputs['Height']); l.new(bump.outputs['Normal'],n.get('Principled BSDF').inputs['Normal'])

def move_collection(obj, coll):
    for c in list(obj.users_collection): c.objects.unlink(obj)
    coll.objects.link(obj)

def Z(z): return 1.798+(z-1.798)*(.950/1.200)
def box(name, loc, size, material, bevel=.003, parent=None, coll=asset):
    if coll==asset and not name.startswith(('Cabinet','Freezer','temporary_pocket','Handle_pocket')):
        loc=(loc[0],loc[1],Z(loc[2]))
        size=(size[0],size[1],size[2]*(.950/1.200))
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o=bpy.context.object; o.name=name; o.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    move_collection(o,coll)
    if material: o.data.materials.append(material)
    if bevel:
        o.data.use_auto_smooth=True
        mod=o.modifiers.new('Manufactured edge radius','BEVEL'); mod.width=bevel; mod.segments=3
        mod=o.modifiers.new('Weighted surface normals','WEIGHTED_NORMAL')
    if parent:
        world=o.matrix_world.copy(); o.parent=parent; o.matrix_world=world
    return o

def pivot(name,loc,parent=None):
    o=bpy.data.objects.new(name,None); asset.objects.link(o); o.location=(loc[0],loc[1],Z(loc[2])) if name!='BODY_FIXED' else loc; o.empty_display_size=.055
    o.empty_display_type='ARROWS'
    bpy.context.view_layer.update()
    if parent:
        world=o.matrix_world.copy(); o.parent=parent; o.matrix_world=world
    return o

body=pivot('BODY_FIXED',(0,0,0))
body['overall_width_m']=.708; body['overall_depth_m']=.708; body['overall_height_m']=1.804
body['source']='Haier manual overall dimensions; upper door 950mm; middle drawer 320+30mm; pocket length220/depth15/front lip5/end15mm measured; pocket short opening36mm provisional'
body['physics_status']='KINEMATIC PROTOTYPE: forces, masses and contact not calibrated'
for s in [-1,1]:
    box('Cabinet_side_'+str(s),(s*.332, .039,.907),(.044,.630,1.774),side,.008,body)
box('Cabinet_roof',(0,.039,1.786),(.664,.630,.036),side,.006,body)
box('Cabinet_back',(0,.331,.907),(.664,.046,1.774),side,.004,body)
box('Cabinet_base',(0,.035,.035),(.65,.60,.04),dark,.004,body)
for x in [-.29,.29]:
    for y in [-.24,.27]: box('Cabinet_foot',(x,y,.01),(.04,.04,.02),dark,.004,body)
for s in [-1,1]:
    box('White_inner_side_'+str(s),(s*.305,.030,1.182),(.015,.570,1.16),white,.012,body)
box('White_back',(0,.305,1.18),(.60,.020,1.17),white,.01,body)
box('White_ceiling',(0,.030,1.758),(.60,.57,.025),white,.009,body)
box('Cold_compartment_floor',(0,.035,.636),(.61,.57,.055),white,.012,body)
box('Rear_air_duct',(0,.283,1.235),(.155,.021,.91),white,.006,body)
for z in [1.03,1.06,1.09,1.37,1.40,1.43,1.58,1.61]:
    box('Air_vent',(0,.270,z),(.097,.003,.004),rubber,.001,body)
box('LED_ceiling',(0,-.005,1.738),(.40,.095,.008),lightmat,.005,body)
for i,z in enumerate([.858,1.137,1.425]):
    box('Glass_plane_'+str(i),(0,.085,z),(.584,.394,.007),glass,.002,body)
    box('Glass_front_trim_'+str(i),(0,-.114,z),(.589,.009,.011),chrome,.002,body)
    for s in [-1,1]: box('Shelf_support',(s*.292,.08,z-.009),(.014,.36,.013),white,.003,body)

def tray(name,cx,cy,z,w,d,h,material,parent):
    t=.004
    box(name+'_bottom',(cx,cy,z),(w,d,t),material,.002,parent)
    for s in [-1,1]:
        box(name+'_side',(cx+s*(w-t)/2,cy,z+h/2),(t,d,h),material,.002,parent)
        box(name+'_wall',(cx,cy+s*(d-t)/2,z+h/2),(w,t,h),material,.002,parent)
tray('Empty_crisper',0,.087,.674,.565,.380,.171,plastic,body)
box('Crisper_front_trim',(0,-.105,.686),(.566,.009,.023),chrome,.003,body)
# Simplified freezer: intentionally static, closed, separate face objects.
box('Freezer_core',(0,.030,.439),(.645,.570,.785),white,.006,body)
for name,z,h in [('Freezer_bottom',.255,.450),('Freezer_upper',.650,.320)]:
    box(name,(0,-.316,z),(.698,.076,h),steel,.008,body)
    box(name+'_handle_shadow',(0,-.300,z+h/2+.015),(.674,.064,.030),dark,.005,body)

doors={}
for label,sign in [('LEFT',-1),('RIGHT',1)]:
    # Preserve all closed geometry while correcting only the estimated right rotation axis.
    hinge_x=.354 if label=='RIGHT' else sign*.342
    hinge=pivot('HINGE_'+label,(hinge_x,-.275,.600)); doors[label]=hinge
    hinge['axis']='local Z'; hinge['estimated_axis']=True
    hinge['opening_sign']=sign; hinge['resistance_Nm']='UNMEASURED; independently calibrate'
    cx=sign*.177
    panel=box('Door_'+label+'_metal',(cx,-.317,1.198),(.346,.074,1.200),steel,.008,hinge)
    # Underside pocket: actual open cavity for contact, not a painted rectangle.
    cutter=box('temporary_pocket',(sign*.225,-.331,.853),(.220,.036,.020),None,0)
    bpy.context.view_layer.objects.active=panel
    cut=panel.modifiers.new('Underside_grasp_cavity','BOOLEAN'); cut.operation='DIFFERENCE'; cut.solver='FAST'; cut.object=cutter
    bpy.ops.object.modifier_move_up(modifier=cut.name); bpy.ops.object.modifier_move_up(modifier=cut.name)
    bpy.ops.object.modifier_apply(modifier=cut.name)
    bpy.data.objects.remove(cutter,do_unlink=True)
    cutter=box('temporary_top_pocket',(cx,-.314,1.801),(.255,.039,.020),None,0)
    bpy.context.view_layer.objects.active=panel
    cut=panel.modifiers.new('Top_recess_cavity','BOOLEAN'); cut.operation='DIFFERENCE'; cut.solver='FAST'; cut.object=cutter
    bpy.ops.object.modifier_move_up(modifier=cut.name); bpy.ops.object.modifier_move_up(modifier=cut.name)
    bpy.ops.object.modifier_apply(modifier=cut.name)
    bpy.data.objects.remove(cutter,do_unlink=True)
    box('Handle_pocket_roof_'+label,(sign*.225,-.331,.8645),(.220,.036,.003),dark,.001,hinge)
    box('Handle_pocket_back_'+label,(sign*.225,-.3115,.8555),(.220,.003,.015),dark,.001,hinge)
    box('Door_inner_liner_'+label,(cx,-.270,1.20),(.320,.024,1.160),white,.016,hinge)
    for sx in [-1,1]:
        box('Gasket_vertical_'+label,(cx+sx*.161,-.251,1.20),(.008,.011,1.158),rubber,.003,hinge)
    for z in [.623,1.777]: box('Gasket_horizontal_'+label,(cx,-.251,z),(.326,.011,.008),rubber,.003,hinge)
    for i,z in enumerate([.682,1.077,1.553]):
        tray('Door_'+label+'_empty_bin_'+str(i),sign*.170,-.191,z,.250,.119,.097 if i==0 else .115,plastic,hinge)
    for z in [.613,1.794]: box('Hinge_cap_'+label,(sign*.325,-.245,z),(.055,.104,.018),side,.005,body)
    # Upper recess visible in top reference image; an open rim with inset surface.
    box('Top_recess_'+label,(cx,-.314,1.792),(.251,.035,.002),dark,.001,hinge)

mullion=pivot('MULLION_LEFT_FOLD',(-.014,-.243,.65),doors['LEFT'])
mullion['motion_status']='Estimated folding preview; requires video calibration, not a physical linkage'
box('Center_mullion',(0.004,-.236,1.204),(.040,.024,1.103),white,.004,mullion)
for z in [.752,1.185,1.68]: box('Mullion_hinge',( -.016,-.238,z),(.021,.034,.042),white,.003,doors['LEFT'])
box('Mullion_top_guide',(0,-.217,1.742),(.043,.061,.029),white,.005,body)
box('Control_panel',( .078,-.356,1.535),(.056,.006,.213),dark,.005,doors['RIGHT'])
for z in [1.465,1.515,1.578]: box('Control_marker',(.078,-.360,z),(.023,.001,.002),chrome,0,doors['RIGHT'])
bpy.ops.object.text_add(location=(.193,-.358,Z(1.713)),rotation=(math.pi/2,0,0))
o=bpy.context.object; o.name='Haier_wordmark'; o.data.body='Haier'; o.data.size=.026; o.data.extrude=.00015
o.data.materials.append(chrome); move_collection(o,asset)
world=o.matrix_world.copy(); o.parent=doors['RIGHT']; o.matrix_world=world

states=[(1,'CLOSED',0,0),(40,'LEFT_OPEN',90,0),(80,'RIGHT_OPEN',0,90),(120,'BOTH_OPEN',90,90)]
for frame,name,a,b in states:
    for label,ang in [('LEFT',a),('RIGHT',b)]:
        doors[label].rotation_euler.z=math.radians(ang*(-1 if label=='LEFT' else 1))
        doors[label].keyframe_insert(data_path='rotation_euler',frame=frame)
    mullion.rotation_euler.z=math.radians(85 if a else 0)
    mullion.keyframe_insert(data_path='rotation_euler',frame=frame)
    scene.timeline_markers.new(name,frame=frame)
for o in list(doors.values())+[mullion]:
    for fc in o.animation_data.action.fcurves:
        for k in fc.keyframe_points: k.interpolation='LINEAR'
scene.frame_start=1; scene.frame_end=120

floor=mat('Studio floor',(.16,.19,.22),0,.65)
box('STUDIO_ground',(0,0,-.032),(200,200,.04),floor,0,coll=studio)
def aim(obj,point): obj.rotation_euler=(Vector(point)-obj.location).to_track_quat('-Z','Y').to_euler()
def area(name,loc,power,size):
    data=bpy.data.lights.new(name,'AREA'); data.energy=power; data.shape='DISK'; data.size=size
    o=bpy.data.objects.new(name,data); studio.objects.link(o); o.location=loc; aim(o,(0,0,1))
area('Key',(-3,-4,5),450,4); area('Fill',(3,-2,3),300,3); area('Rim',(0,3,4),450,3)
area('Interior_LED_light',(0,-.15,1.71),9,.35)
aim(bpy.data.objects['Interior_LED_light'],(0,.18,1.12))
data=bpy.data.cameras.new('Preview_camera'); cam=bpy.data.objects.new('Preview_camera',data); studio.objects.link(cam)
cam.location=(1.3,-5,2.4); aim(cam,(0,-.12,.92)); data.type='ORTHO'; data.ortho_scale=2.30; scene.camera=cam
scene.world.color=(.25,.25,.25)
scene.render.engine='BLENDER_EEVEE'; scene.eevee.use_gtao=True; scene.eevee.gtao_distance=.12
scene.eevee.use_ssr=True; scene.eevee.use_ssr_refraction=True; scene.eevee.taa_render_samples=128
for material in [glass,plastic]:
    material.use_screen_refraction=True; material.refraction_depth=.004
scene.render.resolution_x=1000; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
scene.view_settings.view_transform='Standard'; scene.view_settings.look='None'; scene.view_settings.exposure=-.35
scene.render.image_settings.file_format='PNG'
scene.frame_set(1)
bpy.ops.object.select_all(action='DESELECT'); doors['LEFT'].select_set(True); bpy.context.view_layer.objects.active=doors['LEFT']
for screen in bpy.data.screens:
    for ar in screen.areas:
        if ar.type=='VIEW_3D':
            ar.spaces.active.region_3d.view_distance=3.5
            ar.spaces.active.region_3d.view_location=(0,0,.9)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT,'haier_bcd402_body_v1.blend'))
audit={'overall_m':[.708,.708,1.804],'asset_objects':len(asset.objects),'states':states,'contents':False,'physics_calibrated':False,'estimated':['all component sizes','hinge axes','mullion folding'],'frames':{name:f for f,name,a,b in states}}
with open(os.path.join(OUT,'model_manifest.json'),'w') as f: json.dump(audit,f,indent=2)
for f,name,a,b in states[:1]:
    scene.frame_set(f); scene.render.filepath=os.path.join(OUT,name.lower()+'.png'); bpy.ops.render.render(write_still=True)
print('REFRIGERATOR_BUILD_COMPLETE',flush=True)

