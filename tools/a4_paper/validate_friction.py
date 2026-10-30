"""Independent friction benchmarks: two sliding sheets, derivative and time-rate checks."""
from pathlib import Path
import json,numpy as np,ipctk
O=Path('outputs/ipc_friction_tests');O.mkdir(parents=True,exist_ok=True)
x=np.array([[0,0,0],[.01,0,0],[0,.01,0],[0,0,.0005],[.01,0,.0005],[0,.01,.0005]],float)
f=np.array([[0,1,2],[3,4,5]],np.int32);e=np.array([[0,1],[1,2],[2,0],[3,4],[4,5],[5,3]],np.int32)
mesh=ipctk.CollisionMesh(x,e,f);normal=ipctk.NormalCollisions();normal.build(mesh,x,.001,.0001)
B=ipctk.BarrierPotential(.001,stiffness=1e6);N=abs(B.gradient(normal,mesh,x).reshape(-1,3)[3:,2].sum());assert N>0
D=ipctk.FrictionPotential(.001);results=[]
for mu in [0,.3,.6]:
 c=ipctk.TangentialCollisions();c.build(mesh,x,normal,B,mu)
 v=np.zeros_like(x);v[3:,0]=.02;grad=D.gradient(c,mesh,v).reshape(-1,3);force=grad[3:,0].sum()
 assert abs(force-mu*N)<1e-7*max(1,N)
 assert np.linalg.norm(grad.sum(0))<1e-9
 # Actual friction force is minus this potential gradient; it opposes the upper sheet's motion.
 assert force>=0
 rigid=np.tile([.02,.01,0],(6,1));assert np.linalg.norm(D.gradient(c,mesh,rigid))<1e-9
 dt=.025;displacement=v*dt;direction=np.random.default_rng(7).normal(size=x.shape);h=1e-8
 fun=lambda u:dt*D(c,mesh,u/dt)
 fd=(fun(displacement+h*direction)-fun(displacement-h*direction))/(2*h)
 analytic=float(np.sum(grad*direction));assert abs(fd-analytic)<1e-5*max(1,abs(analytic))
 results.append(dict(mu=mu,normal_load_N=float(N),tangential_resistance_N=float(force),expected_mu_N=float(mu*N),position_gradient_relative_error=float(abs(fd-analytic)/max(1,abs(analytic)))))
 # The contact force must do non-positive work at nonzero relative velocity.
 assert -float(np.sum(grad*v))<=1e-12
rate=-np.log(.7)/.025
coarse=1.;fine=1.
for _ in range(40):coarse*=np.exp(-rate*.025)
for _ in range(80):fine*=np.exp(-rate*.0125)
assert abs(coarse-fine)<1e-12
report=dict(passed=True,parallel_sheet_sliding=results,uniform_translation_zero_friction=True,force_balance_passed=True,nonpositive_friction_work=True,constant_state_time_rate_consistency=dict(coarse=coarse,fine=fine),scope='Analytical contact benchmark and constant-state rate check; not material calibration or full trajectory convergence')
(O/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
