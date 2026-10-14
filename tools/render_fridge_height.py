import os
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
import mujoco
from PIL import Image
r=Path(__file__).resolve().parents[1];m=mujoco.MjModel.from_xml_path(str(r/'assets/fq/haier_bcd402/refrigerator.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.9];cam.distance=3.2;cam.azimuth=100;cam.elevation=-10
m.vis.global_.offwidth=800;m.vis.global_.offheight=800;m.vis.headlight.ambient[:]=[.6,.6,.6];m.vis.headlight.diffuse[:]=[.7,.7,.7]
opt=mujoco.MjvOption();opt.geomgroup[3]=0
with mujoco.Renderer(m,height=800,width=800) as rr:
 rr.update_scene(d,camera=cam,scene_option=opt);im=Image.fromarray(rr.render());im.save(r/'validation/previews/fq_articulated_haier_bcd402.png')
print('Rendered corrected geometry')
