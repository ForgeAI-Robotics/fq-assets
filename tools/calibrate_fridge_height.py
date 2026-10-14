"""Apply the user's floor-to-upper-door-bottom measurement to canonical geometry.

Run once, then source/build_asset.py. The source Blender file is provenance;
geometry.json plus this calibration is the authoritative exported geometry.
"""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
f=ROOT/'assets/fq/haier_bcd402';p=f/'source/geometry.json';g=json.loads(p.read_text())
if g.get('door_bottom_height_calibration',{}).get('target_m')==.82:
 print('Already calibrated to 82 cm');raise SystemExit(0)
original=hashlib.sha256(p.read_bytes()).hexdigest()
OLD=.848;NEW=.82;TOP=1.798;S=(TOP-NEW)/(TOP-OLD)
# Upper compartment stretch leaves the top fixed. Preserve the 15 mm handle pockets.
def mapper(item):
 name=item['source'];link=item['link']
 if link in ('door_left','door_right','mullion'):
  if name.startswith('Handle_pocket'):return lambda z:z+(NEW-OLD)
  return lambda z:TOP+(z-TOP)*S
 if name=='Freezer_upper':return lambda z:.49+(z-.49)*(.782-.49)/(.81-.49)
 if name=='Freezer_upper_handle_shadow':return lambda z:z+(NEW-OLD)
 if name=='Freezer_core':return lambda z:.0465+(z-.0465)*(.8035-.0465)/(.8315-.0465)
 if name.startswith('Hinge_cap'):
  return (lambda z:z+(NEW-OLD)) if not name.endswith('.001') else (lambda z:z)
 if name.startswith(('White_','Cold_compartment','Rear_air','Air_vent','LED_','Glass_','Shelf_','Empty_crisper','Crisper_','Mullion_top')):
  return lambda z:TOP+(z-TOP)*S
 return lambda z:z
for v in g['visuals']:
 fn=mapper(v);anchor=g['anchors'][v['link']][2]
 for vertex in v['vertices']:vertex[2]=fn(vertex[2]+anchor)-anchor
for c in g['collisions']:
 fn=mapper(c);anchor=g['anchors'][c['link']][2];lo=fn(c['center'][2]+anchor-c['size'][2]/2);hi=fn(c['center'][2]+anchor+c['size'][2]/2)
 c['center'][2]=(lo+hi)/2-anchor;c['size'][2]=hi-lo
# Preserve body frames and vertical hinge lines. Update provisional COM/inertia
# for the upper assemblies under fixed-mass vertical stretch; Izz is unchanged.
x=E.parse(f/'refrigerator.xml');physics={}
for link in g['anchors']:
 name='refrigerator_base' if link=='cabinet' else 'refrigerator_'+link;b=x.find('.//body[@name="'+name+'"]');i=b.find('inertial');com=list(map(float,i.get('pos').split()));inertia=list(map(float,i.get('diaginertia').split()))
 if link!='cabinet':
  anchor=g['anchors'][link][2];com[2]=TOP+(com[2]+anchor-TOP)*S-anchor
  czz=(inertia[0]+inertia[1]-inertia[2])/2
  inertia[0]+=(S*S-1)*czz;inertia[1]+=(S*S-1)*czz
 physics[link]={'mass_kg':float(i.get('mass')),'com_m':com,'diagonal_inertia_kg_m2':inertia}
g['physics']=physics
g['door_bottom_height_calibration']={'target_m':NEW,'previous_m':OLD,'top_m':TOP,'upper_vertical_scale':S,'source_geometry_sha256':original,'basis':'User measured floor to lowest edge of both upper doors, confirmed 2026-10-14','unmeasured_adjustments':'Upper compartment affine vertical stretch, handle pocket depth retained; upper freezer panel shortened by 28 mm, 8 mm inter-compartment gap retained; cabinet exterior and lower freezer unchanged','physics':'Fixed component masses. Upper COM/inertia adjusted as approximate vertical stretch; not mass-property calibration.'}
p.write_text(json.dumps(g,separators=(',',':'))+'\n')
record={'revision':'v3-user-floor-to-door-bottom','upper_door_bottom_z_m':NEW,'upper_door_top_z_m':TOP,'upper_door_height_m':TOP-NEW,'previous_upper_door_height_m':.95,'supersedes':'calibration_v2.json upper door height/bottom and upper drawer height; other measurements retained','upper_freezer_panel_height_m':.292,'upper_freezer_recess_height_m':.03,'upper_freezer_top_z_m':.812,'door_gap_m':.008,'geometry_transform':g['door_bottom_height_calibration'],'source_blend_status':'Original provenance snapshot, not regenerated; use geometry.json and build_asset.py for calibrated exports','physics_calibrated':False}
(f/'calibration_v3.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(record,indent=2))
