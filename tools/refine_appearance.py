"""Persist the table appearance previously authored by navigate's render_hq.py.
The conservative conference-table collision box stays unchanged.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
from pxr import Usd,UsdGeom,UsdShade,UsdLux,Gf
root=Path(__file__).resolve().parents[1]
scene=root/'scenes/fq/office_baseline'
geo=json.loads((scene/'geometry.json').read_text())
table=next(b for b in geo['boxes'] if b['name']=='meeting_table')
parts=[('meeting_tabletop',[*table['center'][:2],.72],[2*table['half'][0],2*table['half'][1],.06],'furniture')]
for i,x in enumerate([.55,3.96]):
    for j,y in enumerate([9.88,10.80]):
        parts.append((f'meeting_table_leg_{i}_{j}',[x,y,.345],[.075,.075,.69],'frame'))
stage=Usd.Stage.Open(str(scene/'architecture.usda'))
UsdGeom.Imageable(stage.GetPrimAtPath('/World/meeting_table')).MakeInvisible()
for name,pos,size,mat in parts+[('meeting_panel',[2.7,10.4,2.62],[1.3,.5,.036],'light')]:
    b=UsdGeom.Cube.Define(stage,'/World/'+name);b.CreateSizeAttr(1)
    xf=UsdGeom.Xformable(b);xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos));xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/World/Looks/'+mat)))
light=UsdLux.RectLight.Define(stage,'/World/MeetingDeskLight')
light.CreateIntensityAttr(6000);light.CreateWidthAttr(1.3);light.CreateHeightAttr(.5)
xf=UsdGeom.Xformable(light);xf.ClearXformOpOrder();xf.AddTranslateOp().Set(Gf.Vec3d(2.7,10.4,2.55))
stage.GetRootLayer().Save()
for path in [scene/'scene.xml',root/'assets/fq/conference_table/table.xml']:
    tree=ET.parse(path);world=tree.getroot().find('worldbody')
    g=world.find("geom[@name='meeting_table']")
    g.set('rgba','0 0 0 0');g.set('group','3')
    for name,pos,size,mat in parts:
        if world.find(f"geom[@name='{name}']") is None:
            ET.SubElement(world,'geom',name=name,type='box',pos=' '.join(map(str,pos)),
                size=' '.join(str(v/2) for v in size),material=mat+'_material',contype='0',conaffinity='0',mass='0')
    if path.name=='table.xml':
        asset=tree.getroot().find('asset')
        if asset.find("material[@name='frame_material']") is None:
            source=ET.parse(scene/'scene.xml').getroot().find("asset/material[@name='frame_material']")
            asset.append(source)
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
print('Persisted navigation table visuals; collision geometry unchanged')
