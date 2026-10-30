from pathlib import Path
import argparse,json,time,math,sys
import numpy as np,torch
sys.path.insert(0,str(Path(__file__).resolve().parent))
import kami
p=argparse.ArgumentParser();p.add_argument('--output',default='outputs/a4_paper');p.add_argument('--mode',choices=['fold','crumple','flat'],default='fold');p.add_argument('--elastic',action='store_true');p.add_argument('--N',type=int,default=25);a=p.parse_args()
O=Path(a.output)/(a.mode+'_elastic' if a.elastic else a.mode);O.mkdir(parents=True,exist_ok=True)
s=kami.Sheet(kami.Paper.printer_80gsm(),kami.Sim(N=a.N,size=.297,aspect_x=.210/.297,device='cuda:0',substeps=36 if a.mode=='crumple' else 24,frame_dt=1/60,layer_spacing=.001 if a.mode=='crumple' else .0006,damping=2,contact_items_per_triangle=128),kami.CreaseLaw.classic(yield0=.12,yield_floor=.04,rate=0 if a.elastic else 14),batch=1)
frames=[];log=[];peak=0;overflow=0;t0=time.time();tools=[]
def step(count,phase):
 global peak,overflow
 for _ in range(count):
  s.step();st,sc=kami.metrics.max_strain(s);strain=float(torch.maximum(st,sc)[0]);peak=max(peak,strain);overflow=max(overflow,int(s.T['overflow'].max()))
  pos=s.positions[0].detach().cpu().numpy().copy();assert np.isfinite(pos).all()
  if len(log)%3==0:frames.append(pos);snap_tools.append(list(tools));phases.append(phase)
  log.append(dict(t=len(log)/60,phase=phase,strain=strain,plastic_hinges=int((s.T['rest'].abs()>.03).sum()),extent=np.ptp(pos,axis=0).tolist(),overflow=int(s.T['overflow'].max())))
  if len(log)%60==0:print(a.mode,len(log),phase,'strain',round(strain,4),'plastic',log[-1]['plastic_hinges'],'wall',round(time.time()-t0,1),flush=True)
snap_tools=[];phases=[]
uv=s.rest_uv();initial=s.positions[0].detach().cpu().numpy().copy();assert np.allclose(np.ptp(initial,axis=0)[:2],[.210,.297],atol=1e-6)
step(30,'FLAT A4')
if a.mode=='fold':
 near=uv[:,1]<-.1485+1e-6;far=uv[:,1]>.1485-1e-6;s.pin(near|far);p0=s.positions.clone()
 for i in range(180):
  phi=math.pi*(1-math.cos(math.pi*(i+1)/180))/2;target=p0.clone();target[:,far,1]=.1485*math.cos(phi);target[:,far,2]=s.z_floor+.1485*math.sin(phi)+s.t*phi/math.pi;s.move_pins(target,far);step(1,'EDGE CLAMPS FOLD')
 step(30,'FOLD HOLD')
 y=float(s.positions[0,:,1].max())-.024;r=.004
 for i in range(180):
  z=s.z_floor+1.5*s.t+r+.04*max(0,1-i/45)
  if i>135:z+=.04*(i-135)/45
  ends=[[-.13,y,z],[.13,y,z]];s.set_tool(0,*ends,radius=r,half_width=.02,mu=.5,force=10,contact_length=.210);tools[:]=[dict(a=ends[0],b=ends[1],radius=r,half_width=.02)];step(1,'PRESS CREASE' if i<=135 else 'LIFT PRESS')
 s.remove_tool();tools.clear();s.unpin();step(180,'RELEASED: RETAINED FOLD')
elif a.mode=='crumple':
 # External rounded tools gather and squeeze; initial slight curl breaks perfect planar symmetry.
 s.reset(curl=1.0,curl_axis=0)
 for i in range(540):
  u=.5-.5*math.cos(math.pi*min(1,(i+1)/360));rx=.18-(.18-.105)*u;ry=.225-(.225-.105)*u
  tools.clear()
  for j,(x,y) in enumerate([(rx,0),(-rx,0),(0,ry),(0,-ry)]):
   pa=[x,y,-.02];pb=[x,y,.20];s.set_tool(j,pa,pb,radius=.065,mu=.65);tools.append(dict(a=pa,b=pb,radius=.065,half_width=0))
  top=.18-.11*max(0,min(1,(i-240)/180));s.set_tool(4,[0,0,top+.10],radius=.10,mu=.65,force=10)
  tools.append(dict(a=[0,0,top+.10],b=[0,0,top+.10],radius=.10,half_width=0))
  step(1,'ROUNDED TOOLS GATHER + SQUEEZE')
 for i in range(90):
  u=(i+1)/90;tools.clear()
  for j,(x,y) in enumerate([(.105+.1*u,0),(-.105-.1*u,0),(0,.105+.1*u),(0,-.105-.1*u)]):
   pa=[x,y,-.02];pb=[x,y,.20];s.set_tool(j,pa,pb,radius=.065,mu=.65);tools.append(dict(a=pa,b=pb,radius=.065,half_width=0))
  s.set_tool(4,[0,0,.17+.2*u],radius=.10,mu=.65);tools.append(dict(a=[0,0,.17+.2*u],b=[0,0,.17+.2*u],radius=.10,half_width=0));step(1,'TOOLS RELEASE')
 s.remove_tool();tools.clear();step(180,'RELEASED: CRUMPLED SHEET')
else:step(90,'FLAT STABILITY')
np.savez_compressed(O/'trajectory.npz',vertices=np.asarray(frames),triangles=s.topo.tris,rest_uv=uv.cpu().numpy(),plastic_rest=s.T['rest'].cpu().numpy())
(O/'frames.json').write_text(json.dumps({'phases':phases,'tools':snap_tools,'fps':20}))
report=dict(mode=a.mode,engine='kami / NVIDIA Warp 1.16, FQ rectangular-grid prototype',width_m=.210,length_m=.297,grammage_g_m2=80,mass_kg=.210*.297*.080,physical_thickness_m=.0001,numerical_layer_spacing_m=s.t,vertices=s.n,triangles=s.topo.n_tri,frames=len(log),max_edge_strain=peak,contact_hash_overflow=overflow,final=log[-1],elapsed_s=time.time()-t0,scope='Uncalibrated rectangular adaptation; kinematic clamps; discrete vertex-triangle contact, not CCD; no robot coupling')
(O/'validation.json').write_text(json.dumps(report,indent=2));(O/'metrics.json').write_text(json.dumps(log));print('A4_RUN_COMPLETE',report,flush=True)
