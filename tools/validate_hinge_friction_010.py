"""Validate the small hinge-friction setting without changing scene geometry."""
from pathlib import Path
import sys,json,math,copy,importlib.util,xml.etree.ElementTree as ET
import numpy as np,mujoco
from pxr import Usd,UsdPhysics
r=Path(sys.argv[1]);report={'setting_not_hardware_calibration':True,'mujoco':[],'usd':[]}
profile=json.loads((r/'assets/fq/haier_bcd402/simulation_resistance.json').read_text())
assert profile['frictionloss_Nm']==.1 and profile['damping_Nm_s_rad']==0 and profile['randomization']['bounds_Nm']==[.1,.1]
for rel in ['assets/fq/haier_bcd402/refrigerator.xml','scenes/fq/office_baseline/scene.xml']:
 p=r/rel;m=mujoco.MjModel.from_xml_path(str(p));j=m.joint('refrigerator_right_hinge');v=int(j.dofadr[0]);q=int(j.qposadr[0]);assert m.dof_frictionloss[v]==.1 and m.dof_damping[v]==0
 # Model-only baseline clone: remove exactly the new friction setting, then
 # compare all tested solver forces at identical non-contact poses.
 base=copy.copy(m);base.dof_frictionloss[v]=0.;cases=[]
 for speed in [-.1,.1]:
  forces=[]
  for model in [base,m]:
   d=mujoco.MjData(model);d.qpos[q]=math.radians(30);d.qvel[v]=speed;mujoco.mj_forward(model,d);forces.append(float(d.qfrc_constraint[v]))
  diff=forces[1]-forces[0];assert abs(diff+np.sign(speed)*.1)<1e-6,(rel,speed,diff);cases.append({'velocity_rad_s':speed,'added_opposing_torque_Nm':diff})
 report['mujoco'].append(dict(path=rel,frictionloss_Nm=float(m.dof_frictionloss[v]),damping=float(m.dof_damping[v]),solver_cases=cases))
for rel,root in [('assets/fq/haier_bcd402/refrigerator.usda','/Refrigerator'),('scenes/fq/office_baseline/scene.usda','/World/refrigerator')]:
 stage=Usd.Stage.Open(str(r/rel));joint=stage.GetPrimAtPath(root+'/Joints/refrigerator_right_hinge');assert joint
 drive=UsdPhysics.DriveAPI(joint,'angular');cap=drive.GetMaxForceAttr().Get();slope=drive.GetDampingAttr().Get()
 assert abs(cap-.1)<1e-7 and abs(slope-10000*math.pi/180)<1e-4 and drive.GetStiffnessAttr().Get()==0 and drive.GetTargetVelocityAttr().Get()==0
 assert joint.GetCustomDataByKey('fq:simulationFrictionlossNm')==.1 and joint.GetAttribute('physxJoint:jointFriction').Get()==0
 report['usd'].append(dict(path=rel,cap_Nm=cap,drive_slope_Nm_s_degree=slope,dimensionless_joint_friction=0,validation='Composed USD attributes; not a new PhysX dynamics run'))
spec=importlib.util.spec_from_file_location('fq_resistance',r/'assets/fq/haier_bcd402/source/physx_resistance.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
stage=Usd.Stage.Open(str(r/'assets/fq/haier_bcd402/refrigerator.usda'));stage.SetEditTarget(stage.GetSessionLayer());module.author(stage)
assert abs(UsdPhysics.DriveAPI(stage.GetPrimAtPath('/Refrigerator/Joints/refrigerator_right_hinge'),'angular').GetMaxForceAttr().Get()-.1)<1e-7
report['generator_default_reads_canonical_profile']=True
(r/'validation/fridge_hinge_010_20261019.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
