"""Validate translated baseline against the previous content-locked snapshot."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np,mujoco
from pxr import Usd,UsdGeom,UsdPhysics
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--before',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=a.root
for f in (r/'assets').rglob('*'):
 if f.is_file():assert f.read_bytes()==(a.before/f.relative_to(r)).read_bytes(),f
m=mujoco.MjModel.from_xml_path(str(r/'scenes/fq/office_baseline/scene.xml'));d=mujoco.MjData(m);b=mujoco.MjModel.from_xml_path(str(a.before/'scenes/fq/office_baseline/scene.xml'));bd=mujoco.MjData(b);mujoco.mj_forward(m,d);mujoco.mj_forward(b,bd)
root=m.body('refrigerator_base').id
for field in ['body_mass','body_inertia','body_quat','geom_pos','geom_size','geom_quat','mesh_vert','mesh_face','dof_frictionloss','dof_damping']:np.testing.assert_array_equal(getattr(m,field),getattr(b,field))
expected=b.body_pos.copy();expected[root,0]-=1;np.testing.assert_allclose(m.body_pos,expected,atol=1e-12)
def inside(body):
 while body:
  if body==root:return True
  body=m.body_parentid[body]
 return False
fridge=[i for i in range(m.ngeom) if inside(m.geom_bodyid[i])]
others=[i for i in range(m.ngeom) if i not in fridge]
np.testing.assert_allclose(d.geom_xpos[fridge],bd.geom_xpos[fridge]+[-1,0,0],atol=1e-9);np.testing.assert_array_equal(d.geom_xpos[others],bd.geom_xpos[others])
s=Usd.Stage.Open(str(r/'scenes/fq/office_baseline/scene.usda'));cache=UsdGeom.XformCache();np.testing.assert_allclose(cache.GetLocalToWorldTransform(s.GetPrimAtPath('/World/refrigerator')).ExtractTranslation(),[15.5,11.9,0],atol=1e-9);np.testing.assert_allclose(UsdPhysics.FixedJoint(s.GetPrimAtPath('/World/refrigerator/Joints/fixed_base')).GetLocalPos0Attr().Get(),[15.5,11.9,0],atol=1e-6)
# Closed geometry clearances, exact collision surface distance for walls.
coll=[i for i in fridge if m.geom_contype[i] or m.geom_conaffinity[i]]
assert all(m.geom_type[i]==mujoco.mjtGeom.mjGEOM_BOX for i in coll)
for i in coll:np.testing.assert_allclose(abs(d.geom_xmat[i].reshape(3,3)),np.eye(3),atol=1e-8)
closed_max=np.max([d.geom_xpos[i]+m.geom_size[i] for i in coll],axis=0)
wall_clearance={name:float(d.geom_xpos[m.geom(name).id,axis]-m.geom_size[m.geom(name).id,axis]-closed_max[axis]) for name,axis in [('east',0),('north_east',1)]}
assert min(wall_clearance.values())>.02,wall_clearance
# Horizontal visual envelope: fridge to round table, useful plan-view dimension.
def bounds(indices):
 points=[]
 for i in indices:
  if m.geom_type[i]!=mujoco.mjtGeom.mjGEOM_MESH:continue
  mesh=m.geom_dataid[i];v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]];points.extend(v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i])
 v=np.asarray(points);return v.min(0),v.max(0)
table=[i for i in range(m.ngeom) if 'round_table' in (m.geom(i).name or '') or 'round_table' in (m.body(m.geom_bodyid[i]).name or '')]
lo,hi=bounds(fridge);tl,th=bounds(table);table_x_gap=float(lo[0]-th[0]);assert table_x_gap>0
m.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT);q=[m.joint('refrigerator_'+x+'_hinge').qposadr[0] for x in ['left','right']];worst=0
for left,right in [(x,0) for x in np.arange(0,90.01,.5)]+[(0,x) for x in np.arange(0,90.01,.5)]+[(x,y) for x in np.arange(0,90.01,5) for y in np.arange(0,90.01,5)]:
 d.qpos[q]=np.deg2rad([left,right]);mujoco.mj_forward(m,d)
 for c in d.contact:
  if int(c.geom[0]) in fridge or int(c.geom[1]) in fridge:worst=max(worst,-float(c.dist))
assert worst<1e-6,worst
report={'passed':True,'old_position_m':[16.5,11.9,0],'new_position_m':[15.5,11.9,0],'displacement_m':[-1,0,0],'orientation_and_other_geometry_unchanged':True,'standalone_assets_byte_identical':True,'usd_world_anchor_matches':True,'wall_clearance_m':wall_clearance,'round_table_horizontal_gap_m':table_x_gap,'sweep_configurations':723,'max_fridge_penetration_m':worst,'measurement_status':'User-reported approximate 1m plan-left displacement'}
a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
