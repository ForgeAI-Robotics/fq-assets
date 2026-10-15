from pathlib import Path
import json,math
import numpy as np,mujoco
from pxr import Usd,UsdGeom,UsdPhysics,Gf
R=Path('/assets');F=R/'assets/fq/haier_bcd402';V=Path('/out')
m=mujoco.MjModel.from_xml_path(str(F/'refrigerator.xml'));d=mujoco.MjData(m)
old=mujoco.MjModel.from_xml_path('/before/assets/fq/haier_bcd402/refrigerator.xml')
for k in ['body_mass','body_inertia','body_ipos','body_pos','jnt_pos','jnt_axis','jnt_range','dof_frictionloss','dof_damping']:
 np.testing.assert_allclose(getattr(m,k),getattr(old,k),atol=1e-7)
m.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
q=[m.joint('refrigerator_'+side+'_hinge').qposadr[0] for side in ['left','right']]
worst=0.;count=0
for left,right in [(a,0) for a in np.arange(0,90.01,.5)]+[(0,a) for a in np.arange(0,90.01,.5)]+[(a,b) for a in np.arange(0,90.01,5) for b in np.arange(0,90.01,5)]:
 d.qpos[q]=np.deg2rad([left,right]);mujoco.mj_forward(m,d);count+=1
 pen=max((-float(c.dist) for c in d.contact),default=0);worst=max(worst,pen)
 assert pen<1e-5,(left,right,[(m.geom(c.geom[0]).name,m.geom(c.geom[1]).name,c.dist) for c in d.contact if c.dist< -1e-5])
s=Usd.Stage.Open(str(F/'refrigerator.usda'));cache=UsdGeom.XformCache();errors=[]
mujoco.mj_resetData(m,d);mujoco.mj_forward(m,d)
for side in ['left','right']:
 name='refrigerator_'+side+'_hinge';j=UsdPhysics.RevoluteJoint.Get(s,'/Refrigerator/Joints/'+name);points=[]
 for rel,pos in [(j.GetBody0Rel(),j.GetLocalPos0Attr()),(j.GetBody1Rel(),j.GetLocalPos1Attr())]:points.append(np.array(cache.GetLocalToWorldTransform(s.GetPrimAtPath(rel.GetTargets()[0])).Transform(Gf.Vec3d(*pos.Get()))))
 np.testing.assert_allclose(points[0],points[1],atol=1e-6);np.testing.assert_allclose(points[0],d.xanchor[m.joint(name).id],atol=1e-6)
errors=[float(np.max(abs(points[0]-points[1])))]
arch=json.loads((F/'arch_geometry.json').read_text());usd=[p for p in s.Traverse() if '/Collision/' in str(p.GetPath()) and '_arch_' in p.GetName()]
assert len(usd)==arch['convex_collision_parts']
assert all(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()=='convexHull' for p in usd)
for p in usd:assert UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
baseline=mujoco.MjModel.from_xml_path(str(R/'scenes/fq/office_baseline/scene.xml'))
assert len([i for i in range(baseline.ngeom) if '_arch_' in baseline.geom(i).name])==len(usd)
report={'passed':True,'configurations':count,'max_penetration_m':worst,'parent_child_filter_disabled':True,'mass_inertia_hinges_resistance_unchanged':True,'convex_parts_mjcf_usd_baseline':len(usd),'max_usd_anchor_error_m':max(errors),'geometry':arch,'hardware_calibrated':False}
(V/'geometry-validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
