"""Check published geometry, collision closure, mass and full door sweep."""
from pathlib import Path
import argparse,json,math
import numpy as np,mujoco
from pxr import Usd,UsdGeom,UsdPhysics
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=a.root;f=r/'assets/fq/haier_bcd402'
m=mujoco.MjModel.from_xml_path(str(f/'refrigerator.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
def visual_bounds(model,data,body):
 verts=[]
 for k in range(model.ngeom):
  if model.geom_bodyid[k]!=model.body(body).id or model.geom_type[k]!=mujoco.mjtGeom.mjGEOM_MESH:continue
  mesh=model.geom_dataid[k];v=model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]]
  verts.extend(v@data.geom_xmat[k].reshape(3,3).T+data.geom_xpos[k])
 return np.array(verts).min(0),np.array(verts).max(0)
s=Usd.Stage.Open(str(f/'refrigerator.usda'));bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','guide']);result={}
for side in ['left','right']:
 body='refrigerator_door_'+side;lo,hi=visual_bounds(m,d,body);assert abs(lo[2]-.82)<2e-6 and abs(hi[2]-1.798)<2e-6,(side,lo,hi)
 c=[k for k in range(m.ngeom) if m.geom_bodyid[k]==m.body(body).id and m.geom_type[k]==mujoco.mjtGeom.mjGEOM_BOX];bottom=min(d.geom_xpos[k,2]-m.geom_size[k,2] for k in c);assert abs(bottom-.82)<2e-6
 bound=bbox.ComputeWorldBound(s.GetPrimAtPath('/Refrigerator/door_'+side)).ComputeAlignedRange();assert abs(bound.GetMin()[2]-.82)<2e-6 and abs(bound.GetMax()[2]-1.798)<2e-6
 j=m.joint('refrigerator_'+side+'_hinge');assert abs(d.xanchor[j.id,0]-(-.354 if side=='left' else .354))<1e-6
 result[side]={'visual_bottom_cm':float(lo[2]*100),'collision_bottom_cm':float(bottom*100),'usd_bottom_cm':float(bound.GetMin()[2]*100),'top_cm':float(hi[2]*100)}
assert abs(sum(m.body_mass)-91)<1e-9
assert m.dof_frictionloss[m.joint('refrigerator_right_hinge').dofadr[0]]==1.6
assert abs(UsdPhysics.DriveAPI(s.GetPrimAtPath('/Refrigerator/Joints/refrigerator_right_hinge'),'angular').GetMaxForceAttr().Get()-1.6)<1e-6
m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT);qi=[int(m.joint('refrigerator_'+side+'_hinge').qposadr[0]) for side in ['left','right']];worst=0.;count=0
for l,rr in [(x,0) for x in np.arange(0,90.01,.5)]+[(0,x) for x in np.arange(0,90.01,.5)]+[(x,y) for x in np.arange(0,90.01,5) for y in np.arange(0,90.01,5)]:
 d.qpos[qi]=np.deg2rad([l,rr]);mujoco.mj_forward(m,d);count+=1;pen=max((-float(c.dist) for c in d.contact),default=0);worst=max(worst,pen)
 assert pen<1e-6,(l,rr,[(m.geom(c.geom[0]).name,m.geom(c.geom[1]).name,c.dist) for c in d.contact if c.dist < -1e-6])
bm=mujoco.MjModel.from_xml_path(str(r/'scenes/fq/office_baseline/scene.xml'));bd=mujoco.MjData(bm);mujoco.mj_forward(bm,bd)
for side in ['left','right']:
 lo,hi=visual_bounds(bm,bd,'refrigerator_door_'+side);assert abs(lo[2]-.82)<2e-6
np.testing.assert_allclose(bd.xpos[bm.body('refrigerator_base').id],[16.5,11.9,0],atol=1e-8)
g=json.loads((f/'source/geometry.json').read_text());shadow=next(v for v in g['visuals'] if v['source']=='Freezer_upper_handle_shadow');top=max(v[2] for v in shadow['vertices']);assert abs(top-.812)<1e-6
report={'passed':True,'asset_version':'0.1.6','doors':result,'inter_compartment_gap_cm':float((.82-top)*100),'mass_kg':float(sum(m.body_mass)),'static_sweep_configurations':count,'max_penetration_m':worst,'baseline_placement_unchanged':True,'resistance_unchanged':True,'physics_calibrated':False}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
