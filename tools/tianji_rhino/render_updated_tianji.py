from pathlib import Path
import json,xml.etree.ElementTree as E, numpy as np
import mujoco
from PIL import Image,ImageDraw
D=Path('/assets/assets/user_supplied/tianji_robot');r=E.parse(D/'preview.xml').getroot()
for mesh in r.findall('asset/mesh'):mesh.set('file',str((D/mesh.get('file')).resolve()))
vis=E.SubElement(r,'visual');E.SubElement(vis,'global',offwidth='1920',offheight='1440');E.SubElement(vis,'headlight',ambient='.3 .3 .3',diffuse='.5 .5 .5',specular='.1 .1 .1')
def compile():
 m=mujoco.MjModel.from_xml_string(E.tostring(r,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d);return m,d
m,d=compile();bounds=[]
for i in range(m.ngeom):
 mi=m.geom_dataid[i];adr=m.mesh_vertadr[mi];n=m.mesh_vertnum[mi];pts=m.mesh_vert[adr:adr+n]@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i];bounds.append([pts.min(0),pts.max(0)])
bounds=np.array(bounds);lo=bounds[:,0].min(0);hi=bounds[:,1].max(0)
# Convex visual geometry screening only: not a precise triangle-mesh collision certification.
screen=[]
for side in ['left','right']:
 cams=[i for i in range(m.ngeom) if mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i).startswith('cam_'+side+'_wrist_link_')]
 grips=[i for i in range(m.ngeom) if mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i).startswith(side+'_rhino_v6_')]
 distances=[(mujoco.mj_geomDistance(m,d,i,j,1,None),i,j) for i in cams for j in grips];distance,i,j=min(distances)
 gaps=[float(np.linalg.norm(np.maximum(0,np.maximum(bounds[i,0]-bounds[j,1],bounds[j,0]-bounds[i,1])))) for i in cams for j in grips]
 screen.append(dict(side=side,min_convex_query_distance_m=float(distance),min_per_mesh_aabb_separation_m=min(gaps),pair=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,k) for k in [i,j]],scope='static zero pose; positive AABB separation proves visual meshes disjoint at this pose. Convex query may return zero on tiny degenerate CAD parts. Not full-motion or bolt-fit verification'))
w=r.find('worldbody');E.SubElement(w,'geom',name='inspection_floor',type='plane',size='5 5 .01',pos='0 0 -.004',rgba='.18 .21 .25 1');E.SubElement(w,'light',pos='2 -2 3',dir='-1 1 -2',diffuse='.5 .5 .5')
m,d=compile();renderer=mujoco.Renderer(m,1440,1920);out=Path('/project/tianji_v6_updated');out.mkdir(exist_ok=True);panels=[]
views=[('Overall: 2 x RhinoV6 + 3 x D435i',None,3.1,15,-15),('Left wrist: RhinoV6 + D435i','left_rhino_v6_base_link',.55,40,-20),('Right wrist: RhinoV6 + D435i','right_rhino_v6_base_link',.55,-40,-20),('Head: D435i','cam_high_link',.5,0,-12)]
for idx,(title,body,dist,az,el) in enumerate(views):
 cam=mujoco.MjvCamera();cam.lookat[:]=(lo+hi)/2 if body is None else d.xpos[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,body)];cam.distance=dist;cam.azimuth=az;cam.elevation=el
 renderer.update_scene(d,cam);im=Image.fromarray(renderer.render());im.save(out/('view_'+str(idx)+'.png'));panel=im.resize((960,720));draw=ImageDraw.Draw(panel);draw.rectangle((0,0,960,38),fill=(20,24,30));draw.text((16,13),title,fill='white');panels.append(panel)
 if idx==0:im.save(Path('/assets/validation/previews/user_supplied_robots_tianji_robot.png'))
sheet=Image.new('RGB',(1920,1440));
for idx,panel in enumerate(panels):sheet.paste(panel,((idx%2)*960,(idx//2)*720))
sheet.save(out/'assembly_overview.png');sheet.save(D/'assembly_overview.png');renderer.close()
report=json.loads((D/'validation.json').read_text());report.update(bounds_m=[lo.tolist(),hi.tolist()],camera_gripper_convex_screen=screen,preview_images=['assembly_overview.png']);(D/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(screen),flush=True)
