"""Reproducible provisional camera pocket; original vendor STL is retained.
Run with numpy, trimesh==4.4.9, manifold3d==3.5.4, mujoco==3.3.6.
"""
from pathlib import Path
import argparse, json, hashlib, copy, xml.etree.ElementTree as E
import numpy as np, trimesh, mujoco

ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,default=Path('/assets'));a=ap.parse_args();root=a.root
def fmt(v): return ' '.join(f'{x:.16g}' for x in v)
def save(tree,path): E.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
for family in ['o6','o7']:
 folder=root/f'assets/fq/robots/g1_{family}';xmlpath=folder/'robot.xml'
 m=mujoco.MjModel.from_xml_path(str(xmlpath));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 h=d.body('head_link');c=d.body('d435i_head_housing');T=np.eye(4)
 T[:3,:3]=h.xmat.reshape(3,3).T@c.xmat.reshape(3,3);T[:3,3]=h.xmat.reshape(3,3).T@(c.xpos-h.xpos)
 original=folder/'meshes/head_link.STL';source=trimesh.load(original,force='mesh');assert source.is_volume
 # 90 x 25 x 25.055 mm CAD camera envelope, plus 2 mm nominal clearance.
 # The opening continues forward (+housing Z); these are design dimensions,
 # not a physical measurement of the manufacturer's slot.
 lo=np.array([-.047,-.0145,-.0271]);hi=np.array([.047,.0145,.20])
 local=source.copy();local.apply_transform(np.linalg.inv(T))
 def box(low,high):
  mat=np.eye(4);mat[:3,3]=(low+high)/2
  return trimesh.creation.box(extents=high-low,transform=mat)
 cut=box(lo,hi);opened=trimesh.boolean.difference([local,cut],engine='manifold');assert opened.is_volume
 opened.apply_transform(T);opened.export(folder/'meshes/head_camera_slot.STL')
 # Partition the previous convex collision hull into five convex regions
 # outside the pocket. Unlike one convex hull of the cut mesh, these cannot
 # bridge/fill the camera opening. Explicit body inertia is left untouched.
 hull=local.convex_hull;low=hull.bounds[0]-.001;high=hull.bounds[1]+.001;parts=[]
 regions=[]
 for axis in [0,1]:
  l=low.copy();u=high.copy();u[axis]=lo[axis];regions.append((l,u))
  l=low.copy();u=high.copy();l[axis]=hi[axis];regions.append((l,u))
  low[axis]=lo[axis];high[axis]=hi[axis]
 l=low.copy();u=high.copy();u[2]=lo[2];regions.append((l,u))
 for i,(l,u) in enumerate(regions):
  if np.any(u<=l):continue
  part=trimesh.boolean.intersection([hull,box(l,u)],engine='manifold')
  if part.is_empty:continue
  assert part.is_volume and part.is_convex
  part.apply_transform(T);name=f'head_camera_slot_collision_{i}';part.export(folder/f'meshes/{name}.STL');parts.append(name)
 tree=E.parse(xmlpath);r=tree.getroot();assets=r.find('asset');head=r.find('.//body[@name="head_link"]')
 for el in list(assets):
  if el.get('name','').startswith('head_camera_slot'):assets.remove(el)
 E.SubElement(assets,'mesh',name='head_camera_slot',file='head_camera_slot.STL')
 for name in parts:E.SubElement(assets,'mesh',name=name,file=name+'.STL')
 geoms=head.findall('geom');visual=next(x for x in geoms if x.get('group')=='1');collision=next(x for x in geoms if x.get('contype')!='0')
 template=copy.deepcopy(collision)
 for el in geoms:head.remove(el)
 visual.set('mesh','head_camera_slot');head.append(visual)
 for name in parts:
  el=copy.deepcopy(template);el.set('name',name);el.set('mesh',name);el.set('group','3');el.set('rgba','0.2 0.2 0.2 0');head.append(el)
 save(tree,xmlpath)
 tree=E.parse(folder/'robot.urdf');link=tree.getroot().find('link[@name="head_link"]')
 mesh=link.find('visual/geometry/mesh');base=str(Path(mesh.get('filename')).parent);mesh.set('filename',base+'/head_camera_slot.STL')
 collision=copy.deepcopy(link.find('collision'))
 for el in list(link.findall('collision')):link.remove(el)
 for name in parts:
  el=copy.deepcopy(collision);el.set('name',name);el.find('geometry/mesh').set('filename',base+'/'+name+'.STL');link.append(el)
 save(tree,folder/'robot.urdf')
 meta={'status':'provisional_geometry_not_measured_on_hardware','original_mesh':'meshes/head_link.STL','original_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),
 'visual_mesh':'meshes/head_camera_slot.STL','collision_meshes':[f'meshes/{x}.STL' for x in parts],
 'source':'Operator confirms physical head has camera slot; dimensions inferred from current D435i CAD envelope, not official slot CAD.',
 'slot_frame':'d435i_head_housing','head_T_slot':T.tolist(),'cut_box_min_m':lo.tolist(),'cut_box_max_m':hi.tolist(),
 'nominal_camera_clearance_m':.002,'removed_mesh_volume_fraction':float(1-opened.volume/source.volume),
 'original_triangles':len(source.faces),'opened_triangles':len(opened.faces),'visual_watertight':bool(opened.is_watertight),
 'collision_strategy':'Five convex regions of original collision hull outside rectangular pocket; group 3 and alpha 0 keep collision proxies out of RGB.',
 'physical_parameters':'Explicit mass, inertia, joints, camera extrinsics and intrinsics unchanged; head contact geometry intentionally changed.',
 'limitations':['Slot dimensions and housing CAD offset are approximate.','Hand-eye candidate still awaits independent physical validation.','Opening geometry does not validate camera optics, depth accuracy or infrared emitter occlusion.']}
 (folder/'head_camera_slot.json').write_text(json.dumps(meta,indent=2)+'\n')
 print(family,json.dumps(meta))
