from pathlib import Path
import argparse,json,xml.etree.ElementTree as E
import numpy as np,mujoco,imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
p=argparse.ArgumentParser();p.add_argument('--mode',default='fold');a=p.parse_args();O=Path('/project/paper_lab/outputs')/a.mode
data=np.load(O/'trajectory.npz');frames=data['vertices'];faces=data['triangles'];meta=json.loads((O/'frames.json').read_text());n=len(frames[0])
vec=lambda v:' '.join(map(str,np.asarray(v).ravel()))
r=E.Element('mujoco');E.SubElement(r,'compiler',angle='radian');vis=E.SubElement(r,'visual');E.SubElement(vis,'global',offwidth='1920',offheight='1080');E.SubElement(vis,'headlight',ambient='.25 .25 .25',diffuse='.25 .25 .25');E.SubElement(vis,'quality',shadowsize='4096',offsamples='4');E.SubElement(vis,'map',znear='.001')
asset=E.SubElement(r,'asset');E.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.2 .25 .32',rgb2='.55 .6 .68',width='512',height='3072')
uv=data['rest_uv']/[.1,.12]+.5
E.SubElement(asset,'texture',name='fibers',type='2d',file='/assets/assets/fq/tissue_paper/tissue_fiber_color.png')
E.SubElement(asset,'mesh',name='paper',vertex=vec(frames[0]),face=vec(faces),texcoord=vec(uv),normal=vec(np.tile([0,0,1.],(n,1))),inertia='shell');E.SubElement(asset,'material',name='paper',texture='fibers',rgba='1 1 1 1',specular='.01',shininess='.01')
E.SubElement(asset,'mesh',name='paper_back',vertex=vec(frames[0]),face=vec(faces[:,::-1]),texcoord=vec(uv),normal=vec(np.tile([0,0,-1.],(n,1))),inertia='shell')
w=E.SubElement(r,'worldbody');E.SubElement(w,'light',pos='.2 -.3 .8',dir='-.1 .2 -1',diffuse='.45 .45 .45');E.SubElement(w,'geom',name='paper',type='mesh',mesh='paper',material='paper',contype='0',conaffinity='0');E.SubElement(w,'geom',type='plane',pos='0 0 -.0005',size='.5 .5 .01',rgba='.17 .22 .28 1')
E.SubElement(w,'geom',name='paper_back',type='mesh',mesh='paper_back',material='paper',contype='0',conaffinity='0')
for i in range(8):E.SubElement(w,'geom',name='tool'+str(i),type='capsule',size='.004 .13',rgba='.15 .4 .68 0',pos='0 0 1',contype='0',conaffinity='0')
m=mujoco.MjModel.from_xml_string(E.tostring(r,encoding='unicode'));d=mujoco.MjData(m);ren=mujoco.Renderer(m,height=1080,width=1920);cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.10];cam.distance=.24;cam.azimuth=110;cam.elevation=-40
R=np.zeros(9);mujoco.mju_quat2Mat(R,m.mesh_quat[0]);R=R.reshape(3,3)
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',25);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
writer=imageio_ffmpeg.write_frames(str(O/(a.mode+'.mp4')),(1920,1080),fps=meta['fps'],codec='libx264',macro_block_size=1,output_params=['-crf','18','-movflags','+faststart']);writer.send(None)
for k,x in enumerate(frames):
 cam.lookat[:]=x.mean(0);cam.azimuth=110+180*max(0,(k-400)/200)
 normal=np.zeros_like(x);tri=x[faces];fn=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
 for j in range(3):np.add.at(normal,faces[:,j],fn)
 normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
 for mid,sign in [(0,1),(1,-1)]:
  matrix=np.zeros(9);mujoco.mju_quat2Mat(matrix,m.mesh_quat[mid]);matrix=matrix.reshape(3,3);va=m.mesh_vertadr[mid];na=m.mesh_normaladr[mid]
  m.mesh_vert[va:va+n]=(x-m.mesh_pos[mid])@matrix;m.mesh_normal[na:na+n]=sign*normal@matrix;mujoco.mjr_uploadMesh(m,ren._mjr_context,mid)
 for i in range(8):m.geom_rgba[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'tool'+str(i)),3]=0
 for i,t in enumerate(meta['tools'][k]):
  gid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'tool'+str(i));pa=np.array(t['a']);pb=np.array(t['b']);delta=pb-pa;m.geom_pos[gid]=(pa+pb)/2;m.geom_size[gid,:2]=[t['radius'],np.linalg.norm(delta)/2];q=np.array([1.,0,0,0]);
  if np.linalg.norm(delta)>1e-9:mujoco.mju_quatZ2Vec(q,delta)
  m.geom_quat[gid]=q;m.geom_rgba[gid,3]=.08
 mujoco.mj_forward(m,d);ren.update_scene(d,camera=cam);ren.scene.flags[mujoco.mjtRndFlag.mjRND_CULL_FACE]=1;im=Image.fromarray(ren.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,1920,96),fill=(18,26,34));dr.text((20,10),'TISSUE SHEET | '+meta['phases'][k],font=font,fill='white');dr.text((20,47),'100 x 120 mm | FQ shell + IPC physics; MuJoCo visualization | 1440 triangles | Uncalibrated',font=small,fill='white');dr.text((20,70),'Soft tissue prototype | Spherical fixture; no hand, no tearing or wet-strength model.',font=small,fill=(180,200,220));writer.send(np.asarray(im))
 if k in [0,40,len(frames)//2,len(frames)-1]:im.save(O/('frame_%04d.png'%k))
 if k%50==0:print('PAPER_RENDER',a.mode,k,flush=True)
writer.close();ren.close();print('PAPER_RENDER_COMPLETE',a.mode,flush=True)
