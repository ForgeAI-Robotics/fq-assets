from pathlib import Path
import argparse,json,numpy as np,ipctk
p=argparse.ArgumentParser();p.add_argument('--mode',default='ipc_refined');a=p.parse_args();O=Path('outputs')/a.mode;z=np.load(O/'trajectory.npz');x=z['vertices'];f=z['triangles'];uv=z['rest_uv'];e=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0);rest=np.column_stack([uv,np.zeros(len(uv))]);mesh=ipctk.CollisionMesh(rest,e,f)
assert np.allclose(np.ptp(uv,axis=0),[.210,.297]);l0=np.linalg.norm(rest[e[:,1]]-rest[e[:,0]],axis=1);peak=float(np.max(abs(np.linalg.norm(x[:,e[:,1]]-x[:,e[:,0]],axis=2)/l0-1)))
crossings=[i for i,xx in enumerate(x) if ipctk.has_intersections(mesh,xx)];linear_bad=[]
for i in range(len(x)-1):
 if not ipctk.is_step_collision_free(mesh,x[i],x[i+1]):linear_bad.append(i)
report=json.loads((O/'validation.json').read_text());window=min(len(x)-1,round(.5/report['dt_s']));last_motion=float(np.max(np.linalg.norm(x[-1]-x[-1-window],axis=1)));held=[r for r in json.loads((O/'metrics.json').read_text()) if r['radius_m'] is not None]
final_extent=np.ptp(x[-1],axis=0);accepted=not crossings and peak<.01 and max(final_extent)<.18 and min(final_extent)>.05 and last_motion<.001
audit={'accepted_loose_paper_ball':bool(accepted),'max_edge_strain_independent':peak,'discrete_intersections_frames':crossings,'linear_export_interpolation_crossing_intervals':linear_bad,'last_half_second_max_vertex_motion_m':last_motion,'final_extent_m':final_extent.tolist(),'last_held_cage_penetration_m':held[-1]['max_cage_penetration_m'],'release_observation_s':sum(r['radius_m'] is None for r in json.loads((O/'metrics.json').read_text()))*report['dt_s'],'continuous_solver_scope':'CCD applied to each accepted Newton update; straight interpolation between exported time-step endpoints is separately audited','limitations':['No physical material calibration','Spherical compression fixture, not hands','No self-friction']}
if 'friction_model' in report:audit['limitations']=['No physical material calibration','Spherical compression fixture, not hands','Uncalibrated lagged regularized friction; no mesh convergence claim']
if (O/'ccd_path.npz').exists():
 path=np.load(O/'ccd_path.npz');xx=path['vertices'];bad=[]
 for j in range(len(xx)-1):
  if not ipctk.is_step_collision_free(mesh,xx[j],xx[j+1]):bad.append(j)
 audit['ccd_path_updates']=len(xx)-1;audit['ccd_path_interpolation_crossings']=bad
 audit['accepted_loose_paper_ball']=bool(accepted and not bad)
(O/'acceptance.json').write_text(json.dumps(audit,indent=2));print(a.mode,audit,flush=True)
# Add a transparent cage to the renderer metadata; this is the external fixture, not paper geometry.
meta=json.loads((O/'frames.json').read_text());meta['tools']=[[]]+[[dict(a=[0,0,.20],b=[0,0,.20],radius=r['radius_m'],half_width=0)] if r['radius_m'] else [] for r in json.loads((O/'metrics.json').read_text())];(O/'frames.json').write_text(json.dumps(meta))
