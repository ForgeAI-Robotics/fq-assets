from pathlib import Path
import json,numpy as np,ipctk
from pxr import Usd,UsdGeom
D=Path('/assets/assets/fq/tissue_paper')
z=np.load(D/'trajectory.npz'); f=z['triangles']; uv=z['rest_uv']
e=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0)
mesh=ipctk.CollisionMesh(np.column_stack([uv,np.zeros(len(uv))]),e,f)
s=Usd.Stage.Open(str(D/'crumple_playback.usdc')); a=UsdGeom.Mesh(s.GetPrimAtPath('/Tissue/Sheet')).GetPointsAttr(); times=a.GetTimeSamples()
bad=[]; previous=np.asarray(a.Get(times[0]),dtype=np.float64)
for i,t in enumerate(times[1:]):
 current=np.asarray(a.Get(t),dtype=np.float64)
 if not ipctk.is_step_collision_free(mesh,previous,current):bad.append(i)
 previous=current
report=dict(usd_time_samples=len(times),float32_export_crossing_intervals=bad,passed=not bad)
(D/'ipc_usd_export_check.json').write_text(json.dumps(report,indent=2)+'\n')
print(report,flush=True)
assert not bad
