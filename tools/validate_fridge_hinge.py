"""Regression check for the estimated right refrigerator hinge correction."""
from pathlib import Path
import argparse,json,math
import mujoco
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True);p.add_argument('--baseline',type=Path);p.add_argument('--generated',type=Path);a=p.parse_args()
f=a.root/'assets/fq/haier_bcd402';m=mujoco.MjModel.from_xml_path(str(f/'refrigerator.xml'));m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT);d=mujoco.MjData(m)
j=m.joint('refrigerator_right_hinge');assert np.allclose(j.pos,[.012,0,0],atol=1e-7)
mujoco.mj_forward(m,d);assert abs(d.xanchor[j.id,0]-.354)<1e-6
report={'asset_version':'0.1.1','right_axis_x_m':float(d.xanchor[j.id,0]),'sweep_degrees':[0,90],'step_degrees':.5,'physics_calibrated':False,'isaac_physx_runtime_tested':False}
if a.baseline:
 b=mujoco.MjModel.from_xml_path(str(a.baseline/'assets/fq/haier_bcd402/refrigerator.xml'));bd=mujoco.MjData(b);mujoco.mj_forward(b,bd)
 for field in ['geom_size','geom_pos','geom_quat','geom_type','geom_contype','geom_conaffinity','mesh_vert','mesh_face']:
  assert np.array_equal(getattr(m,field),getattr(b,field)),field
 assert np.allclose(d.geom_xpos,bd.geom_xpos,atol=1e-9)
 assert np.allclose(d.geom_xmat,bd.geom_xmat,atol=1e-9)
 report['baseline_geometry_and_masks_unchanged']=True
 b.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
 baseline_worst=0.
 for angle in np.arange(0,90.01,.5):
  bd.qpos[b.joint('refrigerator_right_hinge').qposadr[0]]=math.radians(angle);mujoco.mj_forward(b,bd)
  baseline_worst=max(baseline_worst,max((-float(c.dist) for c in bd.contact),default=0.))
 assert baseline_worst>1e-3,baseline_worst
 report['baseline_maximum_internal_penetration_m']=baseline_worst
worst=0.
for angle in np.arange(0,90.01,.5):
 d.qpos[j.qposadr[0]]=math.radians(angle);mujoco.mj_forward(m,d)
 worst=max(worst,max((-float(c.dist) for c in d.contact),default=0.))
assert worst<1e-6,worst
report['maximum_internal_penetration_m']=worst
report['parent_child_collision_filter_disabled_for_sweep']=True
s=Usd.Stage.Open(str(f/'refrigerator.usda'));cache=UsdGeom.XformCache();uj=UsdPhysics.RevoluteJoint.Get(s,'/Refrigerator/Joints/refrigerator_right_hinge');anchors=[]
for rel,pos in [(uj.GetBody0Rel(),uj.GetLocalPos0Attr()),(uj.GetBody1Rel(),uj.GetLocalPos1Attr())]:
 prim=s.GetPrimAtPath(rel.GetTargets()[0]);anchors.append(np.array(cache.GetLocalToWorldTransform(prim).Transform(Gf.Vec3d(*pos.Get()))))
assert np.max(np.abs(anchors[0]-anchors[1]))<1e-6;assert abs(anchors[0][0]-.354)<1e-6
report['usd_joint_anchor_error_m']=float(np.max(np.abs(anchors[0]-anchors[1])))
if a.generated:
 g=mujoco.MjModel.from_xml_path(str(a.generated/'refrigerator.xml'));gd=mujoco.MjData(g);error=0.
 for angle in [0,55,90]:
  d.qpos[j.qposadr[0]]=math.radians(angle);gd.qpos[g.joint('refrigerator_right_hinge').qposadr[0]]=math.radians(angle)
  mujoco.mj_forward(m,d);mujoco.mj_forward(g,gd)
  error=max(error,float(np.max(np.abs(d.geom_xpos-gd.geom_xpos))),float(np.max(np.abs(d.geom_xmat-gd.geom_xmat))))
 assert error<1e-6,error
 report['regenerated_geometry_max_error']=error
 assert json.loads((a.generated/'manifest.json').read_text())['version']=='0.1.1'
report['passed']=True;a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
