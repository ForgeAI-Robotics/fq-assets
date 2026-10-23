import sys,json,math
from pathlib import Path
import mujoco as mj,numpy as np
from PIL import Image
r=Path(sys.argv[1]);m=mj.MjModel.from_xml_path(str(r/'scenes/fq/office_baseline/scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d)
chairs=[];others=[]
for i in range(m.ngeom):
 if not (m.geom_contype[i] or m.geom_conaffinity[i]):continue
 n=mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,i) or ''
 (chairs if n.startswith('office_chair_') else others).append(i)
violations=[];min_dist=1e9;checked=0
for a in chairs:
 for b in others+chairs:
  if b==a:continue
  na=m.geom(a).name;nb=m.geom(b).name
  if nb.startswith(na.split('__')[0]+'__'):continue
  if nb in ('floor','floor_carpet'):continue
  if np.linalg.norm(d.geom_xpos[a]-d.geom_xpos[b])>m.geom_rbound[a]+m.geom_rbound[b]+.1:continue
  dis=mj.mj_geomDistance(m,d,a,b,.15,None);checked+=1;min_dist=min(min_dist,dis)
  if dis<-.002:violations.append([na,nb,float(dis)])
assert not violations,violations[:20]
# Chairs are fixed scenery: preserve all body world poses across integration.
ids=[i for i in range(m.nbody) if (m.body(i).name or '').startswith('office_chair_')];before=d.xpos[ids].copy();mj.mj_step(m,d,nstep=500);mj.mj_forward(m,d);delta=float(np.max(np.abs(d.xpos[ids]-before)));assert delta==0
report={'passed':True,'chair_count':6,'collision_shapes':len(chairs),'pairs_checked':checked,'penetrations_over_2mm':violations,'fixed_pose_drift_m':delta,'method':'MuJoCo geomDistance against neighboring scene and chairs; fixed furniture, no chair dynamics','placement_status':'authored varied placement; not measured','baseline_joints_unchanged':m.njnt}
(r/'validation/baseline_chairs.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
# Detail render: omit ceiling and walls only in this diagnostic image.
for i in range(m.ngeom):
 n=m.geom(i).name or ''
 if not (n.startswith('office_chair_') or n.startswith('meeting_table') or n=='floor') :m.geom_rgba[i,3]=0;m.geom_group[i]=5
ren=mj.Renderer(m,720,960);cam=mj.MjvCamera();cam.lookat[:]=[2.4,10.35,.45];cam.distance=6.0;cam.azimuth=-70;cam.elevation=-42;opt=mj.MjvOption();opt.geomgroup[3]=0;opt.geomgroup[5]=0;opt.sitegroup[:]=0
ren.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(ren.render()).save(r/'validation/previews/baseline_chairs_detail.png');ren.close()
