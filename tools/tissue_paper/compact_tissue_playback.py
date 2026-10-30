from pathlib import Path
import numpy as np,json,ipctk,shutil
from pxr import Usd,UsdGeom,Vt
R=Path('/assets');D=R/'assets/fq/tissue_paper';O=Path('/project/paper_lab/outputs/tissue_v2')
z=np.load(O/'ccd_path.npz');x=z['vertices'];times=z['times'];f=z['triangles'];uv=z['rest_uv'];traj=np.load(O/'trajectory.npz')['vertices'];dt=.0125
e=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0);mesh=ipctk.CollisionMesh(np.column_stack([uv,np.zeros(len(uv))]),e,f)
anchors=[]
for i,frame in enumerate(traj):
 j=int(np.argmin(abs(times-i*dt)));assert abs(times[j]-i*dt)<1e-10 and np.max(abs(x[j]-frame))<1e-12;anchors.append(j)
keep=[anchors[0]];expanded=[]
for i,(a,b) in enumerate(zip(anchors[:-1],anchors[1:])):
 if ipctk.is_step_collision_free(mesh,x[a].astype('float32').astype(float),x[b].astype('float32').astype(float)):keep.append(b)
 else:keep.extend(range(a+1,b+1));expanded.append(i)
xx=x[keep];tt=times[keep];xf=xx.astype('float32').astype(float)
bad=[i for i in range(len(xf)-1) if not ipctk.is_step_collision_free(mesh,xf[i],xf[i+1])];assert not bad
np.savez_compressed(D/'ccd_path.npz',vertices=xx,times=tt,triangles=f,rest_uv=uv,original_indices=np.array(keep))
s=Usd.Stage.Open(str(D/'crumple_playback.usdc'));attr=UsdGeom.Mesh(s.GetPrimAtPath('/Tissue/Sheet')).GetPointsAttr();attr.Clear()
for t,frame in zip(tt,xx):attr.Set(Vt.Vec3fArray.FromNumpy(frame.astype('float32')),float(t*80))
temp=D/'crumple_compact.usdc';s.GetRootLayer().Export(str(temp));s=None;temp.replace(D/'crumple_playback.usdc')
report=dict(original_solver_samples=len(x),published_samples=len(xx),preserved_physical_endpoints=len(anchors),expanded_intervals=expanded,float32_crossing_intervals=bad,physical_endpoint_change_max_m=0.,method='Retain all 80 Hz physical endpoints; retain full internal updates only where endpoint interpolation fails CCD; revalidate every published segment after float32 conversion',passed=True)
(D/'playback_compaction.json').write_text(json.dumps(report,indent=2)+'\n')
p=D/'acceptance.json';a=json.loads(p.read_text());a['solver_full_path_updates']=a['ccd_path_updates'];a['published_path_updates']=len(xx)-1;a['published_path_crossings']=bad;p.write_text(json.dumps(a,indent=2)+'\n')
p=D/'README.md';text=p.read_text();start=text.index('### Playback sampling');text=text[:start]+'''### Playback sampling
The 80 Hz trajectory.npz contains physical-step endpoints for analysis and force indexing. Straight interpolation at release interval 270 crosses. Use ccd_path.npz or crumple_playback.usdc for interpolation: the release export preserves all physical endpoints plus necessary intermediate updates, with every published segment checked again after float32 conversion. Original solver audit: 14807 internal update segments, no crossings. See playback_compaction.json for retained sample counts and verification; full solver outputs can be regenerated with run_demo.py. MP4 shows physical endpoint frames without geometry interpolation.
''';p.write_text(text)
shutil.copyfile('/project/compact_tissue_playback.py',R/'tools/tissue_paper/compact_tissue_playback.py')
print('PLAYBACK_COMPACTED',report,flush=True)
