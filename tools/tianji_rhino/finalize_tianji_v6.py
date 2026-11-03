from pathlib import Path
import json,struct,shutil,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
R=Path('/assets');D=R/'assets/user_supplied/tianji_robot';r=E.parse(D/'model.urdf').getroot();joints=r.findall('joint');links=r.findall('link');assert len({l.get('name') for l in links})==len(links);roots=set(l.get('name') for l in links)-{j.find('child').get('link') for j in joints};assert len(roots)==1
expected={}
def walk(n,M):
 assert n not in expected;expected[n]=M
 for j in joints:
  if j.find('parent').get('link')==n:
   o=j.find('origin');T=np.eye(4)
   if o is not None:T[:3,3]=list(map(float,o.get('xyz','0 0 0').split()));T[:3,:3]=Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_matrix()
   walk(j.find('child').get('link'),M@T)
walk(next(iter(roots)),np.eye(4));assert len(expected)==len(links)
data=(D/'model.glb').read_bytes();assert data[:4]==b'glTF' and len(data)==struct.unpack_from('<I',data,8)[0];n=struct.unpack_from('<I',data,12)[0];g=json.loads(data[20:20+n]);actual={}
def gwalk(i,M):
 node=g['nodes'][i]
 if 'matrix' in node:T=np.array(node['matrix']).reshape(4,4).T
 else:
  T=np.eye(4);T[:3,:3]=Rotation.from_quat(node.get('rotation',[0,0,0,1])).as_matrix()@np.diag(node.get('scale',[1,1,1]));T[:3,3]=node.get('translation',[0,0,0])
 M=M@T
 if node.get('name') in expected:actual[node['name']]=M
 for j in node.get('children',[]):gwalk(j,M)
for i in g['scenes'][g.get('scene',0)]['nodes']:gwalk(i,np.eye(4))
C=Rotation.from_euler('x',90,degrees=True).as_matrix();errors={n:float(np.linalg.norm(C@actual[n][:3,3]-M[:3,3])) for n,M in expected.items()};assert max(errors.values())<1e-5,errors
p=D/'validation.json';v=json.loads(p.read_text());v['glb_urdf_link_position_max_error_m']=max(errors.values());v['glb_link_frames_checked']=len(errors);v['connected_tree_verified']=True
assert all(x['min_per_mesh_aabb_separation_m']>0 for x in v['camera_gripper_convex_screen']);p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
p=D/'provenance.json';prov=json.loads(p.read_text());prov['changes']=['Original source package preserved byte-for-byte','Active model.urdf now replaces both grippers with referenced RhinoV6 and all three camera visuals with referenced D435i','Camera mounting and optical frames retained; camera inertia replaced with labelled box estimate','GLB and fixed non-contact MJCF rebuilt from active assembly'];p.write_text(json.dumps(prov,ensure_ascii=False,indent=2)+'\n')
T=R/'tools/tianji_rhino'
for name in ['update_tianji_v6.py','export_updated_tianji.py','render_updated_tianji.py','finalize_tianji_v6.py']:shutil.copyfile(Path('/project')/name,T/name)
print('VERIFIED',len(expected),'GLB/URDF link positions; max error',max(errors.values()),flush=True)
