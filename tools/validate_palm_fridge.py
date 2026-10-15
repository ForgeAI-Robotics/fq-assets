from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np,mujoco
from pxr import Usd,UsdPhysics
R=Path('/assets');old=Path('/old');out=Path('/out');report={'passed':True,'hardware_validated':False,'palm':[]}
poses=json.loads((R/'validation/palm_source/palm_poses.json').read_text())
for file in ['assets/fq/end_effectors/o6_right/hand.xml','assets/fq/robots/g1_o6/robot.xml']:
 a=mujoco.MjModel.from_xml_path(str(old/file));b=mujoco.MjModel.from_xml_path(str(R/file));invariants=[]
 for key in ['body_mass','body_inertia','body_pos','body_quat','jnt_range','jnt_pos','jnt_axis','dof_frictionloss','dof_damping','actuator_ctrlrange','actuator_gear','eq_data','exclude_signature']:
  assert np.array_equal(getattr(a,key),getattr(b,key)),(file,key);invariants.append(key)
 ids=[g for g in range(b.ngeom) if b.geom(g).name.startswith('right_palm_convex_')];assert len(ids)==64;assert all(b.geom_contype[g]==1 and b.geom_conaffinity[g]==1 for g in ids)
 checks=[]
 for name,pose in poses.items():
  row={'pose':name}
  for label,m in [('old',a),('new',b)]:
   d=mujoco.MjData(m)
   for j,q in pose.items():d.joint(j).qpos[0]=q
   mujoco.mj_forward(m,d);contacts=[c for c in d.contact if {m.body(int(m.geom_bodyid[g])).name for g in c.geom}=={'right_o6_hand_base_link','right_o6_thumb_metacarpals'}];row[label+'_contacts']=len(contacts);row[label+'_penetration_mm']=max([-float(c.dist)*1000 for c in contacts]+[0])
  assert row['old_contacts']>0 and row['new_contacts']==0,row
  checks.append(row)
 report['palm'].append({'file':file,'unchanged':invariants,'parts':len(ids),'checks':checks})
tree=E.parse(R/'assets/fq/robots/g1_o6/robot.urdf');cs=tree.find("link[@name='right_o6_hand_base_link']").findall('collision');assert len(cs)==64
for c in cs:assert (R/'assets/fq/robots/g1_o6'/c.find('geometry/mesh').get('filename')).exists()
rows=[]
for file in ['assets/fq/haier_bcd402/refrigerator.xml','scenes/fq/office_baseline/scene.xml']:
 a=mujoco.MjModel.from_xml_path(str(old/file));m=mujoco.MjModel.from_xml_path(str(R/file));d=mujoco.MjData(m);v=m.joint('refrigerator_right_hinge').dofadr[0]
 for key in ['body_mass','body_inertia','body_pos','body_quat','jnt_pos','jnt_axis','jnt_range','geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction']:
  assert np.array_equal(getattr(a,key),getattr(m,key)),(file,key)
 for key in ['dof_frictionloss','dof_damping']:
  expected=getattr(a,key).copy();expected[v]=0;assert np.array_equal(expected,getattr(m,key))
 m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT);worst=0.
 for left,right in [(x,0) for x in range(91)]+[(0,x) for x in range(91)]+[(x,x) for x in range(91)]:
  d.joint('refrigerator_left_hinge').qpos[0]=np.deg2rad(left);d.joint('refrigerator_right_hinge').qpos[0]=np.deg2rad(right);mujoco.mj_forward(m,d)
  for c in d.contact:
   if any('refrigerator' in m.geom(int(g)).name for g in c.geom):worst=max(worst,-float(c.dist))
 assert worst<1e-5,worst
 rows.append({'file':file,'frictionloss_Nm':float(m.dof_frictionloss[v]),'damping_Nm_s_rad':float(m.dof_damping[v]),'other_physical_parameters_unchanged':True,'sweep_max_penetration_m':worst})
for file,root in [('assets/fq/haier_bcd402/refrigerator.usda','/Refrigerator'),('scenes/fq/office_baseline/scene.usda','/World/refrigerator')]:
 s=Usd.Stage.Open(str(R/file));a=Usd.Stage.Open(str(old/file));j=s.GetPrimAtPath(root+'/Joints/refrigerator_right_hinge');drive=UsdPhysics.DriveAPI(j,'angular');assert drive.GetMaxForceAttr().Get()==0 and drive.GetDampingAttr().Get()==0
 for side in ['left','mullion']:
  p=root+'/Joints/refrigerator_'+side+'_hinge'
  for attr in ['drive:angular:physics:damping','drive:angular:physics:maxForce','physxJoint:jointFriction']:
   assert s.GetPrimAtPath(p).GetAttribute(attr).Get()==a.GetPrimAtPath(p).GetAttribute(attr).Get()
report['fridge']=rows;report['usd_standalone_and_baseline_zero']=True;report['urdf_parts']=len(cs);(out/'palm_fridge_structure.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
