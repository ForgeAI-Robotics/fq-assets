"""Reconstruct the solver's endpoint forces, preserving force type and lagged-state semantics."""
from pathlib import Path
import argparse,json,numpy as np,ipctk
p=argparse.ArgumentParser();p.add_argument('--mode',required=True);a=p.parse_args();O=Path('outputs')/a.mode
z=np.load(O/'trajectory.npz');x=z['vertices'];f=z['triangles'];uv=z['rest_uv'];r=json.loads((O/'validation.json').read_text());dt=r['dt_s']
edges=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0);mesh=ipctk.CollisionMesh(np.column_stack([uv,np.zeros(len(uv))]),edges,f)
B=ipctk.BarrierPotential(.001,stiffness=1e6);D=ipctk.FrictionPotential(.001)
node_load_scale=np.ones(len(uv))
if r.get('node_load_scaling')=='lumped_area_reference_315':
 rest=np.column_stack([uv,np.zeros(len(uv))]);tri=rest[f];areas=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/2;mass=np.zeros(len(uv))
 for j in range(3):np.add.at(mass,f[:,j],areas*.080/3)
 node_load_scale=mass/(mass.sum()/315)
sn=[];sf=[];floor=[];dissipation=[]
for old,new in zip(x[:-1],x[1:]):
 c=ipctk.NormalCollisions();c.build(mesh,old,.001,.0001);t=ipctk.TangentialCollisions();t.build(mesh,old,c,B,r['mu_self'])
 sf.append(-D.gradient(t,mesh,(new-old)/dt).reshape(-1,3))
 dissipation.append(float(-np.sum(sf[-1]*(new-old))))
 c.build(mesh,new,.001,.0001);sn.append(-B.gradient(c,mesh,new).reshape(-1,3))
 load=2000*node_load_scale*np.maximum(.0001-old[:,2],0);slip=new[:,:2]-old[:,:2];force=np.zeros_like(new)
 force[:,:2]=-r['mu_floor']*load[:,None]*slip/np.sqrt(np.sum(slip**2,axis=1)[:,None]+(.001*dt)**2)
 force[:,2]=2000*node_load_scale*np.maximum(.0001-new[:,2],0);floor.append(force)
sn=np.array(sn);sf=np.array(sf);floor=np.array(floor)
np.savez_compressed(O/'contact_forces.npz',time_s=np.arange(1,len(x))*dt,self_normal_N=sn,self_friction_N=sf,floor_contact_N=floor)
report=dict(units='N',shape=list(sn.shape),vertex_index_matches='trajectory.npz vertices',max_self_force_balance_error_N=float(np.linalg.norm((sn+sf).sum(1),axis=1).max()),max_floor_normal_force_N=float(floor[:,:,2].sum(1).max()),final_floor_normal_force_N=float(floor[-1,:,2].sum()),weight_N=r['mass_kg']*9.81,scope='Computed model forces, not measured sensor readings; normal contact at endpoint, friction uses lagged previous-step normal loads; excludes cage and internal elastic forces')
assert report['max_self_force_balance_error_N']<1e-6
(O/'contact_force_validation.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
# Dissipation is minus force dot displacement, not the regularized potential's value.
metrics=json.loads((O/'metrics.json').read_text())
for m,work in zip(metrics,dissipation):m['self_friction_dissipation_J']=work
r['total_self_friction_dissipation_J']=sum(dissipation);r['final']=metrics[-1]
(O/'metrics.json').write_text(json.dumps(metrics));(O/'validation.json').write_text(json.dumps(r,indent=2))
