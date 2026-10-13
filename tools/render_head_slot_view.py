from pathlib import Path
import xml.etree.ElementTree as E,json
import numpy as np,mujoco
from PIL import Image,ImageDraw
root=Path('/assets');out=Path('/out');folder=root/'assets/fq/robots/g1_o6'
canvas=Image.new('RGB',(1280,1040),(245,247,250));draw=ImageDraw.Draw(canvas);report=[]
for row,path in enumerate([out/'original_o6.xml',folder/'robot.xml']):
 tree=E.parse(path);r=tree.getroot();r.find('compiler').set('meshdir',str(folder/'meshes'));asset=r.find('asset');world=r.find('worldbody')
 E.SubElement(asset,'texture',name='diagnostic_checker',type='2d',builtin='checker',rgb1='.8 .8 .8',rgb2='.3 .35 .4',width='256',height='256')
 E.SubElement(asset,'material',name='diagnostic_floor',texture='diagnostic_checker',texrepeat='8 8')
 E.SubElement(world,'geom',name='diagnostic_floor',type='plane',size='3 3 .01',material='diagnostic_floor')
 for i,(y,color) in enumerate([(-.3,'.9 .15 .1 1'),(0,'.1 .65 .2 1'),(.3,'.1 .3 .9 1')]):
  E.SubElement(world,'geom',name=f'diagnostic_target_{i}',type='box',pos=f'.9 {y} .22',size='.1 .1 .22',rgba=color)
 m=mujoco.MjModel.from_xml_string(E.tostring(r,encoding='unicode'));d=mujoco.MjData(m);d.qpos[:7]=[0,0,.8,1,0,0,0];mujoco.mj_forward(m,d)
 m.vis.global_.offwidth=640;m.vis.global_.offheight=480;m.vis.map.znear=.0001/m.stat.extent;m.vis.headlight.ambient[:]=[.7,.7,.7]
 opt=mujoco.MjvOption();opt.geomgroup[3]=0
 # Hide duplicate robot collision proxies, never head visual or optical parts.
 for g in range(m.ngeom):
  if m.geom_bodyid[g]!=0 and (m.geom_contype[g] or m.geom_conaffinity[g]):m.geom_rgba[g,3]=0
 with mujoco.Renderer(m,480,640) as renderer:
  for col,name in enumerate(['d435i_head_rgb','d435i_head_depth']):
   renderer.update_scene(d,camera=name,scene_option=opt);rgb=renderer.render().copy();canvas.paste(Image.fromarray(rgb),(640*col,520*row+40));draw.text((640*col+12,520*row+12),('BEFORE' if row==0 else 'AFTER')+f' | {name} | near = 0.1 mm',fill=(25,30,40))
   renderer.enable_depth_rendering();z=renderer.render().copy();renderer.disable_depth_rendering()
   report.append({'version':'before' if row==0 else 'after','camera':name,'near_m':.0001,'depth_center_m':float(z[240,320]),'near_pixels_under_1cm':int((z<.01).sum()),'total_pixels':z.size})
canvas.save(out/'head-slot-camera-view.png');(out/'head-slot-render-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
