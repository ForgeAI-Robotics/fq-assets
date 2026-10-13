from pathlib import Path
import json,xml.etree.ElementTree as E
import numpy as np,mujoco
from PIL import Image,ImageDraw
root=Path('/assets');out=Path('/out');report=[]
canvas=Image.new('RGB',(1600,1000),(245,247,250));pen=ImageDraw.Draw(canvas)
for family in ['o6','o7']:
 folder=root/f'assets/fq/robots/g1_{family}';m=mujoco.MjModel.from_xml_path(str(folder/'robot.xml'))
 tree=E.parse(out/f'original_{family}.xml');tree.getroot().find('compiler').set('meshdir',str(folder/'meshes'));before=mujoco.MjModel.from_xml_string(E.tostring(tree.getroot(),encoding='unicode'))
 for key in ['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_range','jnt_axis','dof_damping','dof_frictionloss','actuator_gainprm','actuator_biasprm','cam_pos','cam_quat','cam_intrinsic','site_pos','site_quat']:
  np.testing.assert_array_equal(getattr(m,key),getattr(before,key),err_msg=key)
 d=mujoco.MjData(m);k=json.loads((folder/'head_camera.json').read_text());samples=[]
 for pose in [0,.25,-.25]:
  if m.jnt_type[0]==0:d.qpos[:7]=[0,0,.8,1,0,0,0]
  for name in ['waist_yaw_joint','waist_pitch_joint','waist_roll_joint']:d.qpos[m.joint(name).qposadr[0]]=pose
  mujoco.mj_forward(m,d);head=m.body('head_link').id;housing=m.body('d435i_head_housing').id
  originalgroups=m.geom_group.copy()
  for camera,intrinsic in [('rgb','intrinsics'),('depth','depth_intrinsics')]:
   s=d.site(f'd435i_head_{camera}_optical');R=s.xmat.reshape(3,3);K=k[intrinsic]
   for scope in ['head_visual','head_collision','camera_housing_visual']:
    selected=[g for g in range(m.ngeom) if (m.geom_bodyid[g]==(housing if scope=='camera_housing_visual' else head)) and ((m.geom_contype[g]!=0 or m.geom_conaffinity[g]!=0) if scope=='head_collision' else originalgroups[g]==1 and m.geom_rgba[g,3]>0)]
    m.geom_group[:]=5;m.geom_group[selected]=4;hits=[]
    # alpha=0 collision proxies are ignored by ray routines; enable temporarily.
    savedalpha=m.geom_rgba[:,3].copy();m.geom_rgba[selected,3]=1
    for v in np.linspace(0,479,25):
     for u in np.linspace(0,639,33):
      ray=R@np.array([(u-K['cx'])/K['fx'],(v-K['cy'])/K['fy'],1.]);ray/=np.linalg.norm(ray);gid=np.array([-1],np.int32)
      distance=mujoco.mj_ray(m,d,s.xpos,ray,np.array([0,0,0,0,1,0],np.uint8),1,-1,gid)
      if 0<=distance<.30:hits.append(float(distance))
    m.geom_rgba[:,3]=savedalpha
    result={'waist_pose_rad':pose,'camera':camera,'scope':scope,'samples':825,'hits_within_30cm':len(hits),'nearest_m':min(hits) if hits else None}
    samples.append(result)
    if scope=='camera_housing_visual' or (scope=='head_visual' and camera=='rgb'):assert not hits,result
    if scope=='head_visual' and camera=='depth':assert len(hits)<=5,result
  m.geom_group[:]=originalgroups
 entry={'robot':family,'dynamics_and_camera_configuration_unchanged':True,'rays':samples};report.append(entry)
 if family=='o7':continue
 # Same geometry and camera rig in O6/O7; compare old/new O6 for visibility.
 for row,model in enumerate([before,m]):
  data=mujoco.MjData(model);data.qpos[:7]=[0,0,.8,1,0,0,0];mujoco.mj_forward(model,data)
  model.vis.global_.offwidth=800;model.vis.global_.offheight=460;model.vis.map.znear=.0001/model.stat.extent
  model.vis.headlight.ambient[:]=[.6,.6,.6];model.vis.headlight.diffuse[:]=[.6,.6,.6]
  opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.geomgroup[2]=0
  for g in range(model.ngeom):
   if model.geom_contype[g] or model.geom_conaffinity[g]:model.geom_rgba[g,3]=0
  with mujoco.Renderer(model,460,800) as renderer:
   for col in [0,1]:
    cam=mujoco.MjvCamera();cam.lookat[:]=data.body('head_link').xpos+[0,0,.44];cam.distance=.39;cam.azimuth=20 if col==0 else 90;cam.elevation=-10
    renderer.update_scene(data,camera=cam,scene_option=opt);canvas.paste(Image.fromarray(renderer.render()),(col*800,row*500+40))
    pen.text((col*800+15,row*500+12),('BEFORE' if row==0 else 'AFTER')+' | '+('Front quarter' if col==0 else 'Side')+' | near clip = 0.1 mm',fill=(25,30,40))
canvas.save(out/'head-slot-before-after.png');(out/'head-slot-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
