"""Validate the compiled collision aperture against user-provided dimensions."""
import json,os
from pathlib import Path
import mujoco
root=Path(os.environ.get('OFFICE_ROOT','/project/office_video_v2'))
m=mujoco.MjModel.from_xml_path(str(root/'scene/mission.xml'))
upper=m.geom('meeting_door_upper_jamb');lower=m.geom('meeting_door_lower_frame')
clear=(upper.pos[1]-upper.size[1])-(lower.pos[1]+lower.size[1])
table=m.geom('meeting_table');width=2*table.size[1]
assert abs(clear-.8)<3e-5,clear
assert abs(width-1.2)<1e-5,width
for name in ['meeting_south_wall','meeting_east_wall']:
 g=m.geom(name);assert g.contype[0] and m.mat_rgba[g.matid[0],3]==1
for name in ['temporary_unreachable_southwest','temporary_unreachable_southeast']:
 g=m.geom(name);assert g.contype[0] and g.rgba[3]==0
report=dict(door_clear_width_m=float(clear),conference_table_width_m=float(width),meeting_walls_opaque_and_collidable=True,glass_zones_still_unreachable=True)
(root/'reports/dimension_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
