import mujoco,numpy as np,json
from pathlib import Path
r=Path(__file__).resolve().parents[1];m=mujoco.MjModel.from_xml_path(str(r/'scenes/fq/office_baseline/scene.xml'));d=mujoco.MjData(m)
name=lambda typ,i:mujoco.mj_id2name(m,typ,i) or ''
cans=[i for i in range(m.ngeom)if name(mujoco.mjtObj.mjOBJ_GEOM,i).startswith('right_door_coke_') and m.geom_contype[i]]
assert len(cans)==2
j=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'refrigerator_right_hinge');q=m.jnt_qposadr[j];door=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'refrigerator_door_right');results=[];worst=0;hits=[]
for angle in np.linspace(*m.jnt_range[j],31):
 d.qpos[q]=angle;mujoco.mj_forward(m,d)
 for c in cans:
  for g in range(m.ngeom):
   if g==c or not m.geom_contype[g]:continue
   dist=mujoco.mj_geomDistance(m,d,c,g,.002,None)
   if dist<-.0005:hits.append([float(angle),name(mujoco.mjtObj.mjOBJ_GEOM,c),name(mujoco.mjtObj.mjOBJ_GEOM,g),dist])
   worst=min(worst,dist)
 results.append({'angle_rad':float(angle),'can_centers_m':[d.geom_xpos[c].tolist()for c in cans]})
report={'sample_count':31,'range_rad':m.jnt_range[j].tolist(),'worst_penetration_m':-worst,'intersections':hits[:20],'fixed_kinematic_placements':True,'samples':results}
(r/'validation/door_cokes_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items()if k!='samples'}));assert not hits
