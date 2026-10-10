"""Isaac RTX third-person replay of recorded, free-base MuJoCo dynamics."""
import json,os,subprocess,time
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp
view=os.environ.get('OFFICE_VIEW','third')
width,height=(1920,1080) if view=='third' else (640,360)
app=SimulationApp({'headless':True,'width':width,'height':height,'renderer':'RayTracedLighting','anti_aliasing':4 if os.environ.get('OFFICE_CLEAN','1')=='1' else 3})
root=Path(os.environ.get('OFFICE_ROOT','/tmp/office_video'));source=root/'reports'
out=root/os.environ.get('OFFICE_OUT','reports_hq_'+view);out.mkdir(exist_ok=True)
clean=os.environ.get('OFFICE_CLEAN','1')=='1'
pathtrace=os.environ.get('OFFICE_PATHTRACE','0')=='1'
subframes=int(os.environ.get('OFFICE_SUBFRAMES','6')) if clean else 1
limit=int(os.environ.get('OFFICE_FRAME_LIMIT','0'))
preview=os.environ.get('OFFICE_PREVIEW')=='1'
try:
 import omni.usd
 import carb
 import omni.replicator.core as rep
 from pxr import UsdGeom,UsdShade,UsdLux,UsdPhysics,Gf,Sdf,Vt
 from PIL import Image,ImageDraw,ImageFont
 config=json.loads((root/'mission_config.json').read_text())
 geometry=json.loads((root/'scene/geometry.json').read_text())
 replay=np.load(source/'replay.npz');positions=replay['poses'];xy=replay['root_xy'];times=replay['time']
 if limit:positions=positions[:limit];xy=xy[:limit];times=times[:limit]
 robot=json.loads((source/'robot_meshes.json').read_text())
 ctx=omni.usd.get_context();assert ctx.open_stage(str(root/'scene/scene.usda'))
 for _ in range(20):app.update()
 stage=ctx.get_stage()
 settings=carb.settings.get_settings()
 keys=['/rtx/post/aa/op','/rtx/post/motionblur/enabled','/rtx/post/dlss/execMode','/rtx-transient/dlssg/enabled']
 original_settings={k:settings.get(k) for k in keys}
 (out/'rtx_settings_snapshot.json').write_text(json.dumps(settings.get('/rtx'),default=str,indent=2))
 if clean:
  # Installed RTX backend accepts DLSS/DLAA; use native-resolution DLAA.
  settings.set_int('/rtx/post/aa/op',4)
  settings.set_bool('/rtx/post/motionblur/enabled',False)
  settings.set_bool('/rtx-transient/dlssg/enabled',False)
  # Settings discovered in this installed 4.5 renderer, rather than guessed names.
  history_settings={
   '/rtx/lightspeed/NRD_ReLAX/diffuseHistoryFrames':1,
   '/rtx/lightspeed/NRD_ReLAX/specularHistoryFrames':1,
   '/rtx/lightspeed/NRD_ReLAX/diffuseFastHistoryFrames':1,
   '/rtx/lightspeed/NRD_ReLAX/specularFastHistoryFrames':1,
   '/rtx/lightspeed/NRD_ReblurDiffuse/maxAccumulatedFrameNum':1,
   '/rtx/indirectDiffuse/denoiser/temporal/maxHistory':1,
   '/rtx/directLighting/sampledLighting/samplesPerPixel':2,
   '/rtx/shadows/sampleCount':2,
  }
  for k,v in history_settings.items():settings.set_int(k,v)
  for k in ['diffuseAlpha','specularAlpha','diffuseResponsiveAlpha','specularResponsiveAlpha']:
   settings.set_float('/rtx/lightspeed/ReLAX/'+k,1.0)
  settings.set_bool('/rtx/raytracing/autoToggleDenoiserSettings',False)
  keys+=list(history_settings)
 if pathtrace:
  settings.set_string('/rtx/rendermode','PathTracing')
  settings.set_int('/rtx/pathtracing/spp',32)
  settings.set_int('/rtx/pathtracing/totalSpp',32)
  settings.set_bool('/rtx/pathtracing/optixDenoiser/enabled',True)
  settings.set_bool('/rtx/pathtracing/optixDenoiser/temporalMode/enabled',False)
  subframes=1
  keys+=['/rtx/rendermode','/rtx/pathtracing/spp','/rtx/pathtracing/totalSpp','/rtx/pathtracing/optixDenoiser/temporalMode/enabled']
 (out/'render_settings.json').write_text(json.dumps(dict(before=original_settings,after={k:settings.get(k) for k in keys},rt_subframes=subframes),indent=2))
 for p in list(stage.Traverse()):
  if p.IsA(UsdGeom.Mesh) and p.GetName().startswith('ceiling_'):UsdGeom.Imageable(p).MakeInvisible()
 # Render-only cutaway: physics is unchanged and full-height scene is preserved.
 # Main tracking camera sees through ceiling; walls remain at their modeled height.
 for name,key in [('round_table','table'),('coke','coke')]:
  prim=stage.DefinePrim('/World/'+name,'Xform')
  prim.GetReferences().AddReference(f'../props/isaac/{name}.usd')
  UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*config[key]))
  for p in list(stage.Traverse()):
   if str(p.GetPath()).startswith('/World/'+name) and p.HasAPI(UsdPhysics.RigidBodyAPI):
    UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
 panel=UsdGeom.Cube.Define(stage,'/World/meeting_panel')
 panel.CreateSizeAttr(1);xf=UsdGeom.Xformable(panel)
 xf.AddTranslateOp().Set(Gf.Vec3d(2.7,10.4,2.62));xf.AddScaleOp().Set(Gf.Vec3f(1.3,.5,.036))
 UsdShade.MaterialBindingAPI.Apply(panel.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/Looks/light')))
 light=UsdLux.RectLight.Define(stage,'/World/MeetingDeskLight')
 light.CreateIntensityAttr(6000);light.CreateWidthAttr(1.3);light.CreateHeightAttr(.5)
 UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(2.7,10.4,2.55))
 for p in stage.Traverse():
  if p.IsA(UsdLux.DiskLight):
   op=UsdGeom.Xformable(p).GetOrderedXformOps()[0];v=op.Get();op.Set(Gf.Vec3d(v[0],v[1],v[2]-.04))
 # Refine the visible meeting table; retain its conservative collision box.
 UsdGeom.Imageable(stage.GetPrimAtPath('/World/meeting_table')).MakeInvisible()
 def box(name,pos,size,material):
  b=UsdGeom.Cube.Define(stage,'/World/'+name);b.CreateSizeAttr(1)
  x=UsdGeom.Xformable(b);x.AddTranslateOp().Set(Gf.Vec3d(*pos));x.AddScaleOp().Set(Gf.Vec3f(*size))
  UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/Looks/'+material)))
 table_spec=next(b for b in geometry['boxes'] if b['name']=='meeting_table')
 box('meeting_tabletop',[*table_spec['center'][:2],.72],[2*table_spec['half'][0],2*table_spec['half'][1],.06],'furniture')
 for i,x in enumerate([.55,3.96]):
  for j,y in enumerate([9.88,10.80]):box(f'meeting_table_leg_{i}_{j}',[x,y,.345],[.075,.075,.69],'frame')
 transforms=[]
 for i,g in enumerate(robot['geoms']):
  mesh=UsdGeom.Mesh.Define(stage,f'/World/G1/geom_{i}')
  src=robot['meshes'][g['mesh']]
  mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(src['vertices'],np.float32)))
  mesh.CreateFaceVertexCountsAttr([3]*len(src['faces']))
  mesh.CreateFaceVertexIndicesAttr(np.asarray(src['faces'],np.int32).ravel().tolist())
  mesh.CreateSubdivisionSchemeAttr('none')
  mat=UsdShade.Material.Define(stage,f'/World/G1/Materials/m_{i}')
  sh=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Surface');sh.CreateIdAttr('UsdPreviewSurface')
  sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*g['rgba'][:3]))
  sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.38)
  sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(.2)
  mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
  UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
  transforms.append(UsdGeom.Xformable(mesh).AddTransformOp())
 camera=UsdGeom.Camera.Define(stage,'/World/VideoCamera')
 camera.CreateFocalLengthAttr(27 if view=='third' else 18)
 camera.CreateHorizontalApertureAttr(36)
 camera.CreateClippingRangeAttr(Gf.Vec2f(.04,1000))
 cameraop=UsdGeom.Xformable(camera).AddTransformOp()
 product=rep.create.render_product(str(camera.GetPath()),(width,height))
 rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
 head=np.load(source/'head_camera.npz')
 for p in stage.Traverse():
  if p.IsA(UsdGeom.Mesh) and p.GetName().startswith('ceiling_'):
   (UsdGeom.Imageable(p).MakeVisible if view=='first' else UsdGeom.Imageable(p).MakeInvisible)()
  if view=='third' and (p.GetName().startswith('downlight_fixture') or p.GetName()=='meeting_panel'):
   UsdGeom.Imageable(p).MakeInvisible()
 if view=='third':
  for b in geometry['boxes']:
   if b['kind'] in ('light','ceiling'):UsdGeom.Imageable(stage.GetPrimAtPath('/World/'+b['name'])).MakeInvisible()
 sample=os.environ.get('OFFICE_SAMPLE','')
 indices=[int(x) for x in sample.split(',')] if sample else list(range(len(positions)))
 ff=subprocess.Popen(['/tmp/office_ffmpeg','-y','-f','rawvideo','-vcodec','rawvideo','-s',f'{width}x{height}','-pix_fmt','rgb24','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','16','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(view+'.mp4'))],stdin=subprocess.PIPE,stderr=open(out/'encoding.log','w'))
 started=time.monotonic();smooth=xy[0].copy()
 try:
  for count,index in enumerate(indices):
   for op,state in zip(transforms,positions[index]):
    mat=np.eye(4);mat[:3,:3]=state[3:].reshape(3,3).T;mat[3,:3]=state[:3]
    op.Set(Gf.Matrix4d(*mat.ravel().tolist()))
   if view=='third':
    smooth=xy[index].copy() if sample else .92*smooth+.08*xy[index]
    eye=[smooth[0]+3,smooth[1]-4,7.3];target=[smooth[0],smooth[1],.65];up=[0,0,1]
   else:
    eye=head['eye'][index];target=eye+head['forward'][index];up=head['up'][index]
   cameraop.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,target)),Gf.Vec3d(*map(float,up))).GetInverse())
   settings.set_int('/rtx/post/aa/op',4)
   rep.orchestrator.step(rt_subframes=max(12,subframes) if count==0 or sample else subframes,delta_time=.05,pause_timeline=True)
   assert settings.get('/rtx/post/aa/op')==4,'Renderer overrode native DLAA'
   pixels=rgb.get_data();assert pixels.shape[:2]==(height,width)
   im=Image.fromarray(pixels[:,:,:3]);ff.stdin.write(im.tobytes())
   if count==0:(out/'effective_settings.json').write_text(json.dumps({k:settings.get(k) for k in keys},indent=2))
   if sample or index in [0,180,240,300,600,900,1200,1500,1700,len(positions)-1]:im.save(out/f'frame_{index:05d}.png')
   if count%100==0:print('RENDER',view,count,'/',len(indices),'elapsed',round(time.monotonic()-started,1),flush=True)
 finally:
  ff.stdin.close();assert ff.wait()==0
 (out/'render_report.json').write_text(json.dumps(dict(view=view,width=width,height=height,fps=20,route_frames=len(indices),complete_route=not bool(limit or sample),rt_subframes=subframes,antialiasing='native DLAA',motion_blur=False,interpolated=False,elapsed_seconds=time.monotonic()-started),indent=2))
 print('VIDEO_COMPLETE',view,flush=True)
finally:app.close()
