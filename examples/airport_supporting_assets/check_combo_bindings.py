from pathlib import Path
import json,numpy as np
from pxr import Usd,UsdGeom,UsdSkel
R=Path(__file__).resolve().parents[2]
for D in sorted((R/'assets/fq/passenger_combinations').iterdir()):
 s=Usd.Stage.Open(str(D/'assembly.usdc'));root=UsdSkel.Root(s.GetPrimAtPath('/Assembly/Person'));assert root
 cache=UsdSkel.Cache();cache.Populate(root,Usd.PrimDefaultPredicate);q=cache.GetSkelQuery(UsdSkel.Skeleton(s.GetPrimAtPath('/Assembly/Person/Skeleton')));assert q
 recipe=json.loads((D/'assembly.json').read_text());joints=list(q.GetJointOrder());i=joints.index(recipe['usd_joint']);offset=np.array(recipe['bone_local_matrix_column_major']);err=0.
 for t in [0,15,30,45,60]:
  bone=np.array(q.ComputeJointSkelTransforms(t)[i]).T;bag=np.array(UsdGeom.Xformable(s.GetPrimAtPath('/Assembly/Bag')).GetLocalTransformation(t)).T;err=max(err,float(np.max(np.abs(bone@offset-bag))))
 assert err<1e-3,(D,err)
 v=json.loads((D/'validation.json').read_text());v.update(usd_skeleton_binding_valid=True,bone_attachment_max_matrix_error=err,checked_frames=[0,15,30,45,60],matrix_tolerance=0.001,precision_note='USD animation stores float rotations and half-precision scales');(D/'validation.json').write_text(json.dumps(v,indent=2)+'\n');print('BINDING_VALID',D.name,err,flush=True)
