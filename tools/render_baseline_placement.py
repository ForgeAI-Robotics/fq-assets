import os
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
import mujoco
from PIL import Image
r=Path(__file__).resolve().parents[1];m=mujoco.MjModel.from_xml_path(str(r/'scenes/fq/office_baseline/scene.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
for i in range(m.ngeom):
 name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or ''
 if name.startswith(('ceiling_','downlight_fixture')) or name=='meeting_panel':m.geom_rgba[i,3]=0
cam=mujoco.MjvCamera();cam.lookat[:]=[8.5,6.2,.3];cam.distance=24;cam.azimuth=90;cam.elevation=-70
m.vis.global_.offwidth=960;m.vis.global_.offheight=720;m.vis.headlight.ambient[:]=[.5,.5,.5];m.vis.headlight.diffuse[:]=[.7,.7,.7]
opt=mujoco.MjvOption();opt.geomgroup[3]=0
with mujoco.Renderer(m,height=720,width=960) as renderer:
 renderer.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(renderer.render()).save(r/'validation/previews/fq_scenes_office_baseline.png')
print('Rendered updated baseline preview')
