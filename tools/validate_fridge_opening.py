from pathlib import Path
import json,math,argparse
import numpy as np,mujoco
from pxr import Usd,UsdGeom,UsdPhysics,Gf
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--before',type=Path,required=True);p.add_argument('--generated',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=args.root;f=root/'assets/fq/haier_bcd402';m=mujoco.MjModel.from_xml_path(str(f/'refrigerator.xml'));d=mujoco.MjData(m)
b=mujoco.MjModel.from_xml_path(str(args.before/'assets/fq/haier_bcd402/refrigerator.xml'));bd=mujoco.MjData(b)
mujoco.mj_forward(m,d);mujoco.mj_forward(b,bd)
for field in ['geom_size','geom_pos','geom_quat','geom_type','geom_contype','geom_conaffinity','mesh_vert','mesh_face','body_mass','body_inertia']:
 np.testing.assert_array_equal(getattr(m,field),getattr(b,field))
np.testing.assert_allclose(d.geom_xpos,bd.geom_xpos,atol=1e-9)
for model in [m,b]:model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
names=['refrigerator_left_hinge','refrigerator_right_hinge'];qs=[m.joint(n).qposadr[0] for n in names]
worst=0;oldworst=0;count=0
for a in np.arange(0,90.01,.5):
 bd.qpos[b.joint(names[0]).qposadr[0]]=math.radians(a);mujoco.mj_forward(b,bd);oldworst=max(oldworst,max((-c.dist for c in bd.contact),default=0))
for left,right in [(a,0) for a in np.arange(0,90.01,.5)]+[(0,a) for a in np.arange(0,90.01,.5)]+[(a,b) for a in np.arange(0,90.01,5) for b in np.arange(0,90.01,5)]:
 d.qpos[qs]=np.deg2rad([left,right]);mujoco.mj_forward(m,d);count+=1
 pen=max((-float(c.dist) for c in d.contact),default=0);worst=max(worst,pen)
 if pen>1e-6:raise AssertionError((left,right,[(m.geom(c.geom[0]).name,m.geom(c.geom[1]).name,c.dist) for c in d.contact if c.dist < -1e-6]))
assert oldworst>.001
s=Usd.Stage.Open(str(f/'refrigerator.usda'));cache=UsdGeom.XformCache();errors=[]
mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d)
for n in names:
 j=UsdPhysics.RevoluteJoint.Get(s,'/Refrigerator/Joints/'+n);anchors=[]
 for rel,pos in [(j.GetBody0Rel(),j.GetLocalPos0Attr()),(j.GetBody1Rel(),j.GetLocalPos1Attr())]:
  anchors.append(np.array(cache.GetLocalToWorldTransform(s.GetPrimAtPath(rel.GetTargets()[0])).Transform(Gf.Vec3d(*pos.Get()))))
 errors.append(float(np.max(abs(anchors[0]-anchors[1]))));np.testing.assert_allclose(anchors[0],d.xanchor[m.joint(n).id],atol=1e-6)
assert max(errors)<1e-6
report=dict(passed=True,closed_geometry_collision_masks_masses_inertia_unchanged=True,parent_child_filter_disabled=True,sweep_configurations=count,max_penetration_m=worst,previous_left_max_penetration_m=float(oldworst),joint_anchors_m=[d.xanchor[m.joint(n).id].tolist() for n in names],usd_anchor_max_error_m=max(errors),physics_calibrated=False,isaac_runtime_tested=False)
if args.generated:
 g=mujoco.MjModel.from_xml_path(str(args.generated/'refrigerator.xml'));gd=mujoco.MjData(g);error=0.
 for angles in [(0,0),(45,0),(0,45),(90,90)]:
  d.qpos[qs]=np.deg2rad(angles)
  for name,angle in zip(names,angles):gd.qpos[g.joint(name).qposadr[0]]=math.radians(angle)
  mujoco.mj_forward(m,d);mujoco.mj_forward(g,gd)
  error=max(error,float(np.max(abs(d.geom_xpos-gd.geom_xpos))),float(np.max(abs(d.geom_xmat-gd.geom_xmat))))
 assert error<1e-6,error
 report['regenerated_geometry_max_error']=error
 np.testing.assert_allclose(g.body_mass,m.body_mass,atol=1e-7)
 assert json.loads((args.generated/'manifest.json').read_text())['version']=='0.1.3'
args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
