from pathlib import Path
import json, os
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, UsdUtils

root=Path(__file__).resolve().parents[1]
props=root/'assets/fq/tabletop'
scene=root/'scenes/fq/office_baseline'

# Rewrite authored resource paths, including binary USD, without flattening joint structure.
for p in sorted((root/'assets').rglob('*')):
    if p.suffix not in ('.usd','.usdc','.usda'):
        continue
    layer=Sdf.Layer.FindOrOpen(str(p))
    def remap(ref):
        if '/common/' in ref:
            target=props/'common'/ref.split('/common/',1)[1]
            if not target.exists():
                raise ValueError(f'{p}: missing dependency {ref}')
            return os.path.relpath(target,p.parent)
        return ref
    UsdUtils.ModifyAssetPaths(layer,remap)
    layer.Save()
    s=Usd.Stage.Open(str(p))
    print(p.relative_to(root), 'default=',s.GetDefaultPrim().GetPath(),
          'rigid=',[str(x.GetPath()) for x in s.Traverse() if x.HasAPI(UsdPhysics.RigidBodyAPI)])

out=scene/'scene.usda'
stage=Usd.Stage.CreateNew(str(out))
stage.GetRootLayer().subLayerPaths=['architecture.usda']
stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage,1)
layout=json.loads((scene/'reference_layout.json').read_text())
for name,pose in [('round_table',layout['table']),('coke',layout['coke'])]:
    path=props/f'isaac/{name}.usd'
    source=Usd.Stage.Open(str(path))
    prim=stage.DefinePrim('/World/'+name,'Xform')
    prim.GetReferences().AddReference(os.path.relpath(path,scene))
    # Put a wrapper around the referenced default prim so authored geometry transforms survive.
    xf=UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pose))
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(child).CreateRigidBodyEnabledAttr(False)
        if child.IsA(UsdPhysics.Scene):
            child.SetActive(False)
stage.GetRootLayer().Save()

# Reusable conference table keeps its authored baseline frame.
out=root/'assets/fq/conference_table/table.usda'
s=Usd.Stage.CreateNew(str(out))
s.DefinePrim('/World','Xform')
s.SetDefaultPrim(s.GetPrimAtPath('/World'))
UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(s,1)
arch=Usd.Stage.Open(str(scene/'architecture.usda'))
matches=[p for p in arch.Traverse() if p.GetName().startswith('meeting_table')]
assert matches, 'No conference table USD prims found'
# Reference architecture once, selectively deactivate unrelated top-level prims, preserving material bindings.
s.GetPrimAtPath('/World').GetReferences().AddReference(os.path.relpath(scene/'architecture.usda',out.parent))
for p in s.GetPrimAtPath('/World').GetChildren():
    if not (p.GetName().startswith('meeting_table') or p.GetName()=='Looks'):
        p.SetActive(False)
s.GetRootLayer().Save()
print('Composed portable baseline and conference table')
