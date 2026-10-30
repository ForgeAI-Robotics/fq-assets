"""Experimental discrete elastic-plastic shell + IPC. SI units, no robot coupling."""
from pathlib import Path
import argparse,json,time
import numpy as np,torch,scipy.sparse as sp,scipy.sparse.linalg as sla,ipctk
torch.set_num_threads(4)
p=argparse.ArgumentParser();p.add_argument('--nx',type=int,default=17);p.add_argument('--ny',type=int,default=21);p.add_argument('--steps',type=int,default=360);p.add_argument('--settle',type=int,default=200);p.add_argument('--radius',type=float,default=.027);p.add_argument('--out',default='outputs/tissue_ipc');p.add_argument('--elastic',action='store_true');p.add_argument('--dt',type=float,default=.0125);p.add_argument('--mu-self',type=float,default=.3);p.add_argument('--mu-floor',type=float,default=.4);p.add_argument('--resume-dir');a=p.parse_args();O=Path(a.out);O.mkdir(parents=True,exist_ok=True)
assert a.dt>0 and a.mu_self>=0 and a.mu_floor>=0
nx,ny=a.nx,a.ny;uv=np.array([(x,y) for y in np.linspace(-.060,.060,ny) for x in np.linspace(-.050,.050,nx)])
rest=np.column_stack([uv,np.full(len(uv),.20)]);faces=[]
for j in range(ny-1):
 for i in range(nx-1):
  v=j*nx+i
  faces.extend([[v,v+1,v+nx+1],[v,v+nx+1,v+nx]] if (i+j)%2==0 else [[v,v+1,v+nx],[v+1,v+nx+1,v+nx]])
faces=np.array(faces,dtype=np.int32);emap={};hinges=[]
for f in faces:
 for k in range(3):
  aa,bb,cc=f[k],f[(k+1)%3],f[(k+2)%3];key=tuple(sorted([aa,bb]))
  if key in emap:
   u,v,c=emap[key];hinges.append([u,v,c,cc])
  else:emap[key]=(aa,bb,cc)
edges=np.array(list(emap),dtype=np.int32);hinges=np.array(hinges,dtype=np.int32);n=len(rest);ne=len(edges);nh=len(hinges);l0=np.linalg.norm(rest[edges[:,1]]-rest[edges[:,0]],axis=1)
tri=rest[faces];areas=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/2
mass=np.zeros(n)
for j in range(3):np.add.at(mass,faces[:,j],areas*.030/3)
node_load_scale=mass/(mass.sum()/315)
linear_device='cuda' if torch.cuda.is_available() else 'cpu'
kb=np.full(nh,4e-6);ks=np.full(ne,3e4);weights=np.r_[ks,kb];plastic=np.zeros(nh)
mesh=ipctk.CollisionMesh(rest,edges,faces);dhat=.001;dmin=.0001;barrier=ipctk.BarrierPotential(dhat,stiffness=1e6)
friction=ipctk.FrictionPotential(1e-3);tangential=ipctk.TangentialCollisions()
floor_load=np.zeros(n)
rows=np.r_[np.repeat(np.arange(ne),6),np.repeat(ne+np.arange(nh),12)]
cols=np.r_[(edges[:,:,None]*3+np.arange(3)).ravel(),(hinges[:,:,None]*3+np.arange(3)).ravel()]
def angles(x,gradient=True):
 loc=torch.tensor(x[hinges],dtype=torch.float64,requires_grad=gradient);u,v,c,d=loc.unbind(1);edge=v-u;e=edge/torch.linalg.vector_norm(edge,dim=1,keepdim=True)
 n1=torch.linalg.cross(edge,c-u);n2=torch.linalg.cross(-edge,d-v);n1=n1/torch.linalg.vector_norm(n1,dim=1,keepdim=True);n2=n2/torch.linalg.vector_norm(n2,dim=1,keepdim=True)
 theta=torch.atan2((e*torch.linalg.cross(n1,n2)).sum(1),(n1*n2).sum(1))
 grad=torch.autograd.grad(theta.sum(),loc)[0].numpy() if gradient else None
 return theta.detach().numpy(),grad
def terms(x,pred,radius,gravity,full=True):
 delta=x[edges[:,1]]-x[edges[:,0]];length=np.linalg.norm(delta,axis=1);theta,jb=angles(x,full);db=np.arctan2(np.sin(theta-plastic),np.cos(theta-plastic));res=np.r_[length-l0,db]
 inertia=np.repeat(mass/dt**2,3);E=.5*np.dot(weights,res*res)+.5*np.dot(inertia,(x-pred).ravel()**2)+np.dot(mass*gravity,x[:,2])
 if full:
  je=np.stack([-delta/length[:,None],delta/length[:,None]],axis=1);J=sp.csr_matrix((np.r_[je.ravel(),jb.ravel()],(rows,cols)),shape=(ne+nh,3*n));g=np.asarray(J.T@(weights*res)).ravel()+inertia*(x-pred).ravel();g[2::3]+=mass*gravity;H=J.T@sp.diags(weights)@J+sp.diags(inertia+1e-7)
 # A soft spherical compression fixture and a support plane, not prescribed paper vertices.
 q=x-np.array([0,0,.20]);rad=np.linalg.norm(q,axis=1);penetration=np.maximum(rad-radius,0) if radius else np.zeros(n)
 floor=np.minimum(x[:,2]-.0001,0);kc=10.*node_load_scale;kf=500.*node_load_scale;E+=.5*np.dot(kc,penetration*penetration)+.5*np.dot(kf,floor*floor)
 if full:
  direction=q/np.maximum(rad[:,None],1e-12);g+=((kc*penetration)[:,None]*direction).ravel();g[2::3]+=kf*floor
  ids=np.where(penetration>0)[0];rr=(ids[:,None,None]*3+np.arange(3)[None,:,None]+np.zeros((1,1,3),int)).ravel();cc=(ids[:,None,None]*3+np.arange(3)[None,None,:]+np.zeros((1,3,1),int)).ravel();h=(kc[ids,None,None]*direction[ids,:,None]*direction[ids,None,:]).ravel();H+=sp.csr_matrix((h,(rr,cc)),shape=(3*n,3*n));diag=np.zeros(3*n);diag[2::3]=kf*(floor<0);H+=sp.diags(diag)
 collisions=ipctk.NormalCollisions();collisions.build(mesh,x,dhat,dmin);E+=barrier(collisions,mesh,x)
 # Lagged contact frames and normal forces are frozen at the beginning of each step.
 # D is a velocity potential: dt*D((x-start)/dt) has position gradient D' and Hessian D''/dt.
 velocity=(x-start)/dt
 E+=dt*friction(tangential,mesh,velocity)
 slip=x[:,:2]-start[:,:2];eps=1e-3*dt;norm=np.sqrt(np.sum(slip*slip,axis=1)+eps**2);load=a.mu_floor*floor_load
 E+=float(load@(norm-eps))
 if full:
  g+=barrier.gradient(collisions,mesh,x)+friction.gradient(tangential,mesh,velocity)
  H+=barrier.hessian(collisions,mesh,x,ipctk.PSDProjectionMethod.CLAMP)+friction.hessian(tangential,mesh,velocity,ipctk.PSDProjectionMethod.CLAMP)/dt
  fg=np.zeros_like(x);fg[:,:2]=load[:,None]*slip/norm[:,None];g+=fg.ravel()
  block=load[:,None,None]*(np.eye(2)[None,:,:]/norm[:,None,None]-slip[:,:,None]*slip[:,None,:]/norm[:,None,None]**3)
  ids=np.arange(n);rr=np.broadcast_to(ids[:,None,None]*3+np.arange(2)[None,:,None],(n,2,2)).ravel();cc=np.broadcast_to(ids[:,None,None]*3+np.arange(2)[None,None,:],(n,2,2)).ravel();H+=sp.csr_matrix((block.ravel(),(rr,cc)),shape=(3*n,3*n))
  return E,g,H,theta
 return E
dt=a.dt;plastic_rate=-np.log(.7)/.025;damping_rate=-np.log(.7)/.025;x=rest.copy();x[:,2]+=.0003*np.sin(uv[:,0]*60)*np.cos(uv[:,1]*43);vel=np.zeros_like(x);trajectory=[x.copy()];path_positions=[x.copy()];path_times=[0.];records=[];t0=time.time();ccd_count=0;min_alpha=1.;peak=0
start_step=0
if a.resume_dir:
 previous=Path(a.resume_dir);saved=np.load(previous/'trajectory.npz');saved_path=np.load(previous/'ccd_path.npz');old_report=json.loads((previous/'validation.json').read_text())
 assert old_report['grid']==[nx,ny] and old_report['dt_s']==dt and old_report['mu_self']==a.mu_self and old_report['mu_floor']==a.mu_floor
 assert np.array_equal(saved['triangles'],faces) and old_report['final']['phase']=='FIXTURE RELEASED'
 trajectory=list(saved['vertices']);x=trajectory[-1].copy();vel=(trajectory[-1]-trajectory[-2])/dt;plastic=saved['plastic_rest'].copy();path_positions=list(saved_path['vertices']);path_times=list(saved_path['times']);records=json.loads((previous/'metrics.json').read_text());start_step=len(records)
 assert start_step>=a.steps*.75 and a.steps+a.settle>start_step
 ccd_count=old_report['ccd_queries'];min_alpha=old_report['min_ccd_step'];peak=old_report['max_edge_strain'];t0-=old_report['elapsed_s']
assert not ipctk.has_intersections(mesh,x)
# Verify the dihedral Jacobian against finite differences before using it in Newton solves.
theta,jac=angles(x);idx=nh//2;node=hinges[idx,2];xp=x.copy();xm=x.copy();xp[node,2]+=1e-7;xm[node,2]-=1e-7
fd=(angles(xp,False)[0][idx]-angles(xm,False)[0][idx])/2e-7;assert abs(fd-jac[idx,2,2])<1e-4
for step in range(start_step,a.steps+a.settle):
 u=min(step/(a.steps*.65),1);radius=.080-(.080-a.radius)*(.5-.5*np.cos(np.pi*u));gravity=0.
 phase='SPHERICAL FIXTURE COMPRESSION'
 if step>=a.steps*.75:radius=None;gravity=9.81;phase='FIXTURE RELEASED'
 pred=x+dt*vel*np.exp(-damping_rate*dt);start=x.copy();stepsizes=[];accepted_updates=[]
 lagged=ipctk.NormalCollisions();lagged.build(mesh,start,dhat,dmin)
 tangential=ipctk.TangentialCollisions();tangential.build(mesh,start,lagged,barrier,a.mu_self)
 floor_load=500*node_load_scale*np.maximum(.0001-start[:,2],0)
 for it in range(60):
  energy,g,H,theta=terms(x,pred,radius,gravity);dense=H.toarray();assert np.isfinite(dense).all() and np.isfinite(g).all();matrix=torch.from_numpy((dense+dense.T)*.5).to(linear_device);rhs=torch.from_numpy(-g).to(linear_device);L=torch.linalg.cholesky(matrix);dx=torch.cholesky_solve(rhs[:,None],L).squeeze(1).cpu().numpy().reshape(n,3)
  if np.max(np.linalg.norm(dx,axis=1))<2e-6:break
  alpha=float(ipctk.compute_collision_free_stepsize(mesh,x,x+dx,dmin));ccd_count+=1;min_alpha=min(min_alpha,alpha)
  if alpha<1:alpha*=.9
  slope=float(g@dx.ravel())
  for _ in range(18):
   candidate=x+alpha*dx
   if np.isfinite(candidate).all() and terms(candidate,pred,radius,gravity,False)<=energy+1e-4*alpha*slope:break
   alpha*=.5
  else:break
  x=candidate;stepsizes.append(alpha);accepted_updates.append(x.copy())
  if alpha*np.max(np.linalg.norm(dx,axis=1))<1e-7:break
 vel=(x-start)/dt
 if not accepted_updates:accepted_updates=[x.copy()]
 path_positions.extend(accepted_updates);path_times.extend((step+np.arange(1,len(accepted_updates)+1)/len(accepted_updates))*dt)
 theta,_=angles(x,False);difference=np.arctan2(np.sin(theta-plastic),np.cos(theta-plastic));yield_angle=.065*np.sqrt(320/((nx-1)*(ny-1)))
 if not a.elastic:plastic+=np.sign(difference)*np.maximum(abs(difference)-yield_angle,0)*(1-np.exp(-plastic_rate*dt))
 strain=float(np.max(np.abs(np.linalg.norm(x[edges[:,1]]-x[edges[:,0]],axis=1)/l0-1)));peak=max(peak,strain)
 intersect=bool(ipctk.has_intersections(mesh,x));assert not intersect,'IPC accepted an intersecting state'
 rec=dict(max_cage_penetration_m=float(max(0,np.linalg.norm(x-[0,0,.20],axis=1).max()-radius)) if radius else 0,step=step,time_s=(step+1)*dt,phase=phase,radius_m=radius,edge_strain=strain,extent=np.ptp(x,axis=0).tolist(),plastic_hinges=int(np.sum(abs(plastic)>.03)),newton_iterations=it+1,min_step=min(stepsizes,default=1),intersections=intersect)
 records.append(rec);trajectory.append(x.copy())
 rec.update(self_contact_count=len(tangential),self_friction_dissipation_J=float(np.dot(friction.gradient(tangential,mesh,(x-start)/dt),(x-start).ravel())),floor_normal_force_N=float(floor_load.sum()),floor_friction_force_N=float(np.linalg.norm(a.mu_floor*floor_load[:,None]*(x[:,:2]-start[:,:2])/np.sqrt(np.sum((x[:,:2]-start[:,:2])**2,axis=1)[:,None]+(1e-3*dt)**2),axis=1).sum()))
 if step%10==0:
  print('IPC_PAPER',step,'r',radius,'strain',round(strain,5),'plastic',rec['plastic_hinges'],'iter',it+1,'wall',round(time.time()-t0,1),flush=True)
  np.savez_compressed(O/'trajectory.npz',vertices=np.array(trajectory),triangles=faces,rest_uv=uv,plastic_rest=plastic)
report=dict(engine='FQ discrete shell + IPC Toolkit 1.6.1',width_m=.100,length_m=.120,mass_kg=float(mass.sum()),vertices=n,triangles=len(faces),dt_s=dt,steps=a.steps+a.settle,ccd_queries=ccd_count,min_ccd_step=min_alpha,max_edge_strain=peak,all_accepted_states_intersection_free=True,final=records[-1],elapsed_s=time.time()-t0,limitations=['Uncalibrated elastic-plastic shell parameters','Frictionless self contact; no tearing or fiber anisotropy','Spherical penalty fixture, not human fingers or robot grasp','Finite tolerance CCD with 0.1 mm clearance, not an exact arithmetic proof'])
report.update(mu_self=a.mu_self,mu_floor=a.mu_floor,friction_model='IPC lagged regularized Coulomb self-contact; lagged smooth Coulomb floor contact',plastic_rate_per_s=float(plastic_rate),damping_rate_per_s=float(damping_rate),total_self_friction_dissipation_J=sum(r['self_friction_dissipation_J'] for r in records),max_self_contacts=max(r['self_contact_count'] for r in records))
report.update(material_preset='uncalibrated tissue sheet',areal_density_kg_m2=.030,floor_stiffness_reference_N_m=500,cage_stiffness_reference_N_m=10)
report['limitations'][1]='Uncalibrated friction; lagged once per step; no tearing or fiber anisotropy; no mesh-convergence claim'
report.update(node_load_scaling='lumped_area_reference_315',yield_angle_rad=float(.065*np.sqrt(320/((nx-1)*(ny-1)))),linear_solver='float64 Cholesky '+linear_device,grid=[nx,ny],resolution_variant='tissue_experimental')
np.savez_compressed(O/'ccd_path.npz',vertices=np.array(path_positions),times=np.array(path_times),triangles=faces,rest_uv=uv);np.savez_compressed(O/'trajectory.npz',vertices=np.array(trajectory),triangles=faces,rest_uv=uv,plastic_rest=plastic);(O/'validation.json').write_text(json.dumps(report,indent=2));(O/'metrics.json').write_text(json.dumps(records));(O/'frames.json').write_text(json.dumps(dict(fps=1/dt,phases=['INITIAL A4']+[r['phase'] for r in records],tools=[[] for _ in trajectory])));print('IPC_PAPER_COMPLETE',report,flush=True)
