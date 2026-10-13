"""Head-only visual optical channels, preserving the standalone D435i CAD.
Triangle clipping avoids opaque CAD glass/internal parts blocking a pinhole
camera. This is an idealized rendering model, not a model of physical optics.
"""
from pathlib import Path
import json,xml.etree.ElementTree as E
import numpy as np,mujoco,trimesh
root=Path('/assets')
def clip(poly,n,b,inside=True):
 if not len(poly):return []
 result=[]
 for a,c in zip(poly,poly[1:]+poly[:1]):
  da=np.dot(n,a)-b;dc=np.dot(n,c)-b;ia=da<=1e-10 if inside else da>=-1e-10;ic=dc<=1e-10 if inside else dc>=-1e-10
  if ia:result.append(a)
  if ia!=ic:result.append(a+(c-a)*(da/(da-dc)))
 return result
def subtract(poly,planes):
 outside=[]
 for n,b in planes:
  piece=clip(poly,n,b,False)
  if len(piece)>=3:outside.append(piece)
  poly=clip(poly,n,b,True)
  if len(poly)<3:break
 return outside
for f in ['o6','o7']:
 folder=root/f'assets/fq/robots/g1_{f}';p=json.loads((folder/'head_camera.json').read_text());m=mujoco.MjModel.from_xml_path(str(folder/'robot.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 h=d.body('d435i_head_housing');H=h.xmat.reshape(3,3);channels=[]
 for camera,kn in [('rgb','intrinsics'),('depth','depth_intrinsics')]:
  s=d.site(f'd435i_head_{camera}_optical');R=H.T@s.xmat.reshape(3,3);o=H.T@(s.xpos-h.xpos);k=p[kn]
  slopes=[-k['cx']/k['fx'],(k['width']-k['cx'])/k['fx'],-k['cy']/k['fy'],(k['height']-k['cy'])/k['fy']]
  planes=[]
  for n,b in [(np.array([-1.,0,slopes[0]]),.001),(np.array([1.,0,-slopes[1]]),.001),(np.array([0,-1.,slopes[2]]),.001),(np.array([0,1.,-slopes[3]]),.001),(np.array([0,0,-1.]),.0001),(np.array([0,0,1.]),.04)]:
   n=R@n;planes.append((n,b+np.dot(n,o)))
  channels.append(planes)
 tree=E.parse(folder/'robot.xml');r=tree.getroot();body=r.find('.//body[@name="d435i_head_housing"]');urdf=E.parse(folder/'robot.urdf');link=urdf.getroot().find('link[@name="d435i_head_housing"]')
 for el in list(link.findall('visual')):link.remove(el)
 output=[]
 for index,g in enumerate(np.where(m.geom_bodyid==h.id)[0]):
  mi=int(m.geom_dataid[g]);vertices=m.mesh_vert[m.mesh_vertadr[mi]:m.mesh_vertadr[mi]+m.mesh_vertnum[mi]];Q=np.zeros(9);mujoco.mju_quat2Mat(Q,m.geom_quat[g]);vertices=vertices@Q.reshape(3,3).T+m.geom_pos[g]
  triangles=vertices[m.mesh_face[m.mesh_faceadr[mi]:m.mesh_faceadr[mi]+m.mesh_facenum[mi]]];polygons=[list(t) for t in triangles]
  for planes in channels:polygons=[piece for poly in polygons for piece in subtract(poly,planes)]
  tris=[np.array([poly[0],poly[i],poly[i+1]]) for poly in polygons for i in range(1,len(poly)-1)]
  geom=body.findall('geom')[index]
  if not tris:
   geom.set('rgba','0 0 0 0');continue
  verts=np.array(tris).reshape(-1,3);result=trimesh.Trimesh(vertices=verts,faces=np.arange(len(verts)).reshape(-1,3),process=False)
  name=f'head_d435i_optics_{index:02}';result.export(folder/f'meshes/{name}.STL');E.SubElement(r.find('asset'),'mesh',name=name,file=name+'.STL');geom.set('mesh',name);geom.set('pos','0 0 0');geom.set('quat','1 0 0 0')
  visual=E.SubElement(link,'visual');E.SubElement(visual,'origin',xyz='0 0 0',rpy='0 0 0');geometry=E.SubElement(visual,'geometry');E.SubElement(geometry,'mesh',filename=f'meshes/{name}.STL');material=E.SubElement(visual,'material',name=name);E.SubElement(material,'color',rgba=' '.join(map(str,m.geom_rgba[g])))
  output.append({'mesh':name,'original_triangles':len(triangles),'result_triangles':len(tris)})
 for t,path in [(tree,folder/'robot.xml'),(urdf,folder/'robot.urdf')]:E.indent(t);t.write(path,encoding='utf-8',xml_declaration=True)
 meta=json.loads((folder/'head_camera_slot.json').read_text());meta['idealized_optical_channels']={'status':'simulation_pinhole_window_not_physical_lens_model','margin_m':.001,'near_m':-.0001,'far_m':.04,'cameras':['rgb','native_depth'],'parts':output,'scope':'Only head instance; standalone D435i and wrist cameras unchanged.'};(folder/'head_camera_slot.json').write_text(json.dumps(meta,indent=2)+'\n');print(f,len(output),'optical mesh parts')
