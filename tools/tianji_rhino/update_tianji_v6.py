from pathlib import Path
import copy,json,math,hashlib,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco
R=Path('/assets');D=R/'assets/user_supplied/tianji_robot';G=R/'assets/user_supplied/rhino_v6_gripper';C=R/'assets/fq/cameras/realsense_d435i'
def save(p,o):p.write_text(json.dumps(o,ensure_ascii=False,indent=2)+'\n')
def st(v):return ' '.join(map(str,v))
def tf(e):
 o=e.find('origin');return (np.zeros(3),[1,0,0,0]) if o is None else (list(map(float,o.get('xyz','0 0 0').split())),Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_quat()[[3,0,1,2]])
r=E.parse(D/'source/urdf/robot.urdf').getroot()
for m in r.findall('.//mesh'):m.set('filename','source/'+m.get('filename').split('/',3)[-1])
removed={l.get('name') for l in r.findall('link') if 'robotiq' in l.get('name')}
for e in list(r):
 if e.tag=='link' and e.get('name') in removed:r.remove(e)
 elif e.tag=='joint' and (e.find('child').get('link') in removed or e.find('parent').get('link') in removed):r.remove(e)
mounts=[]
rh=E.parse(G/'model.urdf').getroot()
for side,zrot in [('left',-math.pi/2),('right',math.pi/2)]:
 pre=side+'_rhino_v6_'
 for l in rh.findall('link'):
  l=copy.deepcopy(l);l.set('name',pre+l.get('name'))
  for m in l.findall('.//mesh'):m.set('filename','../rhino_v6_gripper/'+m.get('filename'))
  r.append(l)
 for j in rh.findall('joint'):
  j=copy.deepcopy(j);j.set('name',pre+j.get('name'))
  for tag in ['parent','child']:j.find(tag).set('link',pre+j.find(tag).get('link'))
  r.append(j)
 j=E.SubElement(r,'joint',name=pre+'mount',type='fixed');E.SubElement(j,'parent',link=side+'_adapter_flange');E.SubElement(j,'child',link=pre+'base_link');E.SubElement(j,'origin',xyz='0 0 0.011',rpy=st([0,0,zrot]))
 mounts.append(dict(component=side+'_rhino_v6',parent=side+'_adapter_flange',xyz_m=[0,0,.011],rpy_rad=[0,0,zrot],status='provisional: original flange offset retained; opening plane aligned to original gripper; bolt-hole fit not calibrated'))
cr=E.parse(C/'camera.xml').getroot();cm={m.get('name'):m for m in cr.findall('asset/mesh')};camera_links=['cam_left_wrist_link','cam_right_wrist_link','cam_high_link'];camera_info=[]
for name in camera_links:
 l=next(l for l in r.findall('link') if l.get('name')==name);oldvis=l.find('visual');visorigin=copy.deepcopy(oldvis.find('origin'))
 for v in list(l):
  if v.tag=='visual':l.remove(v)
 for g in cr.findall('worldbody/body/geom'):
  if g.get('type')!='mesh':continue
  v=E.SubElement(l,'visual',name=name+'_'+g.get('name'));v.append(copy.deepcopy(visorigin));m=cm[g.get('mesh')];E.SubElement(E.SubElement(v,'geometry'),'mesh',filename='../../fq/cameras/realsense_d435i/meshes/'+m.get('file'));E.SubElement(E.SubElement(v,'material',name=name+'_housing'),'color',rgba=g.get('rgba'))
 # Keep existing collision envelope, replace implausibly large source inertia with a labelled box estimate.
 inert=l.find('inertial');col=l.find('collision');size=np.array(list(map(float,col.find('geometry/box').get('size').split())));mass=.072;I=mass/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2]);inert.find('origin').set('xyz',col.find('origin').get('xyz'));inert.find('inertia').attrib.update(ixx=str(I[0]),iyy=str(I[1]),izz=str(I[2]),ixy='0',ixz='0',iyz='0')
 camera_info.append(dict(link=name,visual_origin=visorigin.attrib,collision='existing camera box retained',inertia='72 g uniform-box estimate, not calibrated',extrinsics='existing source mount and optical frames retained',sensor='D435i reference RGB/depth rendering; no IMU or stereo algorithm simulation'))
# Preserve existing optical names; add high depth optical frame missing in source.
for suffix in ['depth_frame','depth_optical_frame']:
 n='cam_high_'+suffix
 if not any(l.get('name')==n for l in r.findall('link')):
  E.SubElement(r,'link',name=n);j=E.SubElement(r,'joint',name=n+'_joint',type='fixed');E.SubElement(j,'parent',link='cam_high_link' if suffix=='depth_frame' else 'cam_high_depth_frame');E.SubElement(j,'child',link=n);E.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0' if suffix=='depth_frame' else '-1.5707963267948966 0 -1.5707963267948966')
E.indent(r);E.ElementTree(r).write(D/'model.urdf',encoding='utf-8',xml_declaration=True)
root=E.Element('mujoco',model='tianji_v6_three_d435i_static');E.SubElement(root,'compiler',angle='radian');a=E.SubElement(root,'asset');w=E.SubElement(root,'worldbody');links={l.get('name'):l for l in r.findall('link')};joints=r.findall('joint');children={j.find('child').get('link') for j in joints};count=0;assetnames={}
def visit(n,parent,j=None):
 global count
 p,q=tf(j) if j is not None else ([0,0,0],[1,0,0,0]);b=E.SubElement(parent,'body',name=n,pos=st(p),quat=st(q))
 for v in links[n].findall('visual'):
  mesh=v.find('geometry/mesh')
  if mesh is None:continue
  path=mesh.get('filename');assert (D/path).is_file()
  key=(path,mesh.get('scale','1 1 1'))
  if key not in assetnames:
   assetnames[key]='mesh_'+str(len(assetnames));E.SubElement(a,'mesh',name=assetnames[key],file=path,scale=key[1],inertia='shell')
  p,q=tf(v);c=v.find('material/color');E.SubElement(b,'geom',name=n+'_visual_'+str(count),type='mesh',mesh=assetnames[key],pos=st(p),quat=st(q),rgba=c.get('rgba') if c is not None else '.6 .65 .7 1',contype='0',conaffinity='0');count+=1
 if n.endswith('_optical_frame') and n in [x.replace('_link','')+'_'+k+'_optical_frame' for x in camera_links for k in ['color','depth']]:
  k='rgb' if '_color_' in n else 'depth';prof=json.loads((C/'asset.json').read_text())['simulation'][k+'_profile'];E.SubElement(b,'camera',name=n.replace('_optical_frame',''),quat='0 1 0 0',resolution=st([prof['width'],prof['height']]),sensorsize='640 480',focalpixel=st([prof['fx'],prof['fy']]),principalpixel=st([prof['cx']-320,prof['cy']-240]))
 for j in joints:
  if j.find('parent').get('link')==n:visit(j.find('child').get('link'),b,j)
visit(next(iter(set(links)-children)),w);E.indent(root);E.ElementTree(root).write(D/'preview.xml',encoding='utf-8',xml_declaration=True)
m=mujoco.MjModel.from_xml_path(str(D/'preview.xml'));assert m.ncam==6 and m.njnt==0
assert len(links)==len(set(links)) and len(joints)==len(links)-1
assert not any('robotiq' in l for l in links)
for mesh in r.findall('.//mesh'):assert (D/mesh.get('filename')).is_file()
prov=json.loads((D/'provenance.json').read_text())
for path,sha in prov['source_files'].items():assert hashlib.sha256((D/'source'/path).read_bytes()).hexdigest()==sha
save(D/'assembly_v6.json',dict(grippers=mounts,cameras=camera_info,source_archive_unchanged=True,physical_grasp_validated=False,bolt_fit_verified=False))
meta=json.loads((D/'asset.json').read_text());meta.update(version='0.2.0',name='Tianji 双臂机器人 · RhinoV6 + 三台 D435i',description='两侧 RhinoV6 夹爪；左腕、右腕、头部三台 D435i。静态装配及参考相机，安装位姿待实机校准。',dependencies=['user_supplied:grippers/rhino_v6_gripper','fq:cameras/realsense_d435i'],dependency_versions={'user_supplied:grippers/rhino_v6_gripper':'0.1.0','fq:cameras/realsense_d435i':'0.1.0'},viewer_status=[['夹爪','左右 RhinoV6，零位装配'],['相机','左腕 / 右腕 / 头部 D435i'],['状态','装配预览；无驱动，外参和安装孔位未实测']]);meta['limitations']=['RhinoV6 original zero joint limits retained; no actuator or linkage closure implemented','Gripper mounting retains source 11 mm flange offset with opening-plane alignment; physical bolt fit unverified','Camera source mounting and optical frames retained; no per-device extrinsic calibration','Camera mass and box inertia are estimates; no simulated IMU or RealSense stereo pipeline','preview.xml fixes all joints and disables contact; source model.urdf retains collision definitions','Original source package still contains old Robotiq meshes for provenance; active model excludes them'];save(D/'asset.json',meta)
for filename in ['catalog/index.json','assets.lock.json']:
 x=json.loads((R/filename).read_text())
 if filename.startswith('catalog'):
  for entry in x['assets']:
   if entry['id']==meta['id']:entry['version']='0.2.0'
 else:x['assets'][meta['id']]='0.2.0'
 save(R/filename,x)
save(D/'validation.json',dict(version='0.2.0',links=len(links),joints=len(joints),active_robotiq_links=0,rhino_v6_instances=2,d435i_instances=3,camera_render_channels=m.ncam,source_hashes_verified=len(prov['source_files']),mesh_references_resolve=True,physics_validated=False,mounting_calibrated=False))
(D/'README.md').write_text('# Tianji · RhinoV6 + 三台 D435i（0.2.0）\n\n活动 model.urdf 已更换左右 RhinoV6 夹爪，并在原左腕、右腕、头部相机位使用库内 D435i。源 source/ 原样保存，里面旧夹爪仅用于溯源。\n\n夹爪沿用法兰 11 mm 偏移，绕安装轴旋转使开合平面对应原夹爪；属于待标定安装变换，未验证孔位与实物紧固。V6 原始关节零限位保留，未提供开合驱动或闭环连杆约束。\n\n三个相机沿用原装配外参和光学坐标系，补全头部深度坐标系；相机外壳与参考内参来自 fq:cameras/realsense_d435i。72 g 质量和箱体惯量为估计，不是实测。preview.xml 包含三路 RGB 与三路 depth 渲染视角，未模拟 IMU 或双目深度算法。\n\npreview.xml 为全部关节固定、接触关闭的装配展示；model.urdf 保留碰撞描述，未完成动态抓取与全工作空间避碰验证。model.glb 为分层静态三维预览。安装变换详见 assembly_v6.json。\n',encoding='utf-8')
print('ASSEMBLED',len(links),m.ngeom,m.ncam,flush=True)
