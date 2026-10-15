import numpy as np
from pxr import UsdGeom, Gf

def apply_curve(stage, root):
    """Visual-only shared 10 mm crown across both doors; outer edges fixed."""
    reports=[]
    targets=[('door_left','part_049'),('door_right','part_076')]+[('door_right',f'part_{i}') for i in range(108,113)]
    bounds={}
    for body,part in targets[:2]:
        pts=np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(f'{root}/{body}/Visual/{part}')).GetPointsAttr().Get(),dtype=float)
        bounds[body]=(pts.min(0),pts.max(0))
    for body,part in targets:
        mesh=UsdGeom.Mesh(stage.GetPrimAtPath(f'{root}/{body}/Visual/{part}'))
        assert mesh
        p=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
        f=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
        assert all(n==3 for n in mesh.GetFaceVertexCountsAttr().Get())
        lo,hi=bounds[body]
        t=p[f]
        area=np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1);t=t[area>1e-14]
        normals=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
        # Uniform triangle splitting retains boundaries without T junctions.
        for _ in range(4 if part in ('part_049','part_076') else 1):
            a,b,c=t[:,0],t[:,1],t[:,2];ab=(a+b)/2;bc=(b+c)/2;ca=(c+a)/2
            t=np.concatenate([np.stack(x,axis=1) for x in [(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)]])
            normals=np.tile(normals,(4,1))
        p=t.reshape(-1,3);n=np.repeat(normals,3,axis=0)
        offset=-.34200000762939453 if body=='door_left' else .34200000762939453
        u=np.clip((p[:,0]+offset+.35)/.70,0,1);v=(p[:,2]-lo[2])/(hi[2]-lo[2])
        # Smooth 30 mm blend at top/bottom, full crown through door height.
        edge=.03/(hi[2]-lo[2]);z=np.minimum(v,1-v)/edge;z=np.clip(z,0,1)
        blend=z*z*(3-2*z)
        dz=np.where((v<edge),1,np.where(v>1-edge,-1,0))*6*z*(1-z)/.03
        front=np.clip((hi[1]-p[:,1])/(hi[1]-lo[1]),0,1)
        crown=4*u*(1-u);d=.010*crown*blend*front
        dx=.010*4*(1-2*u)/.70*blend*front
        dy=-.010*crown*blend/(hi[1]-lo[1]);dz=.010*crown*dz*front
        # Inverse-transpose Jacobian maps original normals to bent surface.
        ny=n[:,1]/(1-dy);n[:,0]+=dx*ny;n[:,2]+=dz*ny;n[:,1]=ny
        n/=np.linalg.norm(n,axis=1)[:,None]
        p[:,1]-=d
        mesh.GetPointsAttr().Set([Gf.Vec3f(*x) for x in p])
        mesh.GetFaceVertexCountsAttr().Set([3]*len(t));mesh.GetFaceVertexIndicesAttr().Set(list(range(len(p))))
        mesh.CreateNormalsAttr([Gf.Vec3f(*x) for x in n]);mesh.SetNormalsInterpolation('vertex')
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateExtentAttr([Gf.Vec3f(*p.min(0)),Gf.Vec3f(*p.max(0))])
        assert np.isfinite(p).all() and np.isfinite(n).all() and d.max()<=.01000001
        boundary=(u<1e-7)|(u>1-1e-7)|(v<1e-7)|(v>1-1e-7)
        assert not boundary.any() or np.max(d[boundary])<1e-8
        reports.append(dict(part=part,triangles=len(t),maximum_bulge_m=float(d.max()),outer_top_bottom_edge_displacement_m=float(d[boundary].max()) if boundary.any() else 0,width_m=float(hi[0]-lo[0]),height_m=float(hi[2]-lo[2])))
    return reports



