"""Create the first standalone FQ baseline from explicit, read-only source paths."""
from pathlib import Path
import copy
import hashlib
import json
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path('/home/fangqi/code/fq-sim-assets')
NAV = Path('/home/fangqi/g1-navigation-sim/office_video_v2')
SIMPLE = Path('/home/fangqi/code/SIMPLE-o7-verify')
VERSION = '0.1.0'
records = []

def write(path, data):
    path = ROOT / path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

def cp(src, rel):
    src, dst = Path(src), ROOT / rel
    if src.is_dir():
        for p in sorted(src.rglob('*')):
            if p.is_file():
                cp(p, Path(rel) / p.relative_to(src))
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    records.append(dict(source=str(src), path=dst.relative_to(ROOT).as_posix(),
                        source_sha256=hashlib.sha256(src.read_bytes()).hexdigest()))

def xml_write(root, path):
    path = ROOT / path
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)

def entry(uid, kind, path, mj, usd, origin, upstream, **extra):
    data = dict(schema_version=1, id=uid, version=VERSION, kind=kind,
                units='meter', up_axis='Z', quaternion_order='wxyz',
                origin=origin, upstream_origin=upstream,
                provenance='provenance/source_files.json',
                license_status='See NOTICE.md; no blanket relicensing of upstream assets',
                representations={'mujoco': mj, 'usd': usd}, **extra)
    write(Path(path) / 'asset.json', data)
    return dict(id=uid, version=VERSION, manifest=str(Path(path) / 'asset.json'))

if ROOT.exists():
    raise SystemExit(f'Refusing to overwrite existing destination: {ROOT}')
ROOT.mkdir(parents=True)
props = Path('assets/fq/tabletop')
for folder in ('meshes', 'materials', 'textures'):
    for p in (NAV / 'props/common' / folder).glob('*'):
        if p.is_file() and p.name.startswith(('coke', 'round_table')):
            cp(p, props / 'common' / folder / p.name)
cp(NAV/'props/common/textures/surfaces_20260921/table_basecolor.png', props/'common/textures/surfaces_20260921/table_basecolor.png')
for name in ('coke', 'round_table'):
    cp(NAV/f'props/isaac/{name}.usd', props/f'isaac/{name}.usd')
    cp(SIMPLE/f'production_scenes/grasp_coke/mujoco/{name}.xml', props/f'mujoco/{name}.xml')
cp(SIMPLE/'data/scenes/simfoundry/tabletop_v1/source/source_manifest.json', 'provenance/simfoundry_source_manifest.json')
cp(SIMPLE/'production_scenes/grasp_coke/manifest.json', 'provenance/tabletop_source_manifest.json')

scene = Path('scenes/fq/office_baseline')
for name in ('scene.usda', 'geometry.json', 'photo_materials.json', 'textures'):
    cp(NAV/'scene'/name, scene / ('architecture.usda' if name == 'scene.usda' else name))
cp(NAV/'scene/mission.xml', 'provenance/office_mission.original.xml')
cp(NAV/'mission_config.json', scene/'reference_layout.json')
cp(NAV/'scene/README.md', 'provenance/office_source_README.md')
for name in ('prepare_remote.py', 'validate_dimensions.py'):
    cp(NAV/name, Path('provenance/source_scripts')/name)
for name in ('build_test_meeting_scene.py', 'test_meeting_appearance.py'):
    cp(NAV.parent/'workspace/simulation/tools'/name, Path('provenance/source_scripts')/name)
for name in ('dimension_validation.json', 'contact_audit.json', 'navigation_mission.json'):
    cp(NAV/'reports'/name, Path('provenance/historical_validation')/name)
cp(NAV/'reports_hq/frame_00000.jpg', 'previews/office_navigation_historical.jpg')

r = ET.parse(NAV/'scene/mission.xml').getroot()
r.set('model', 'fq_office_baseline_v0_1_0')
for el in list(r.findall('include')):
    r.remove(el)
for el in r.iter():
    ref = el.get('file')
    if not ref:
        continue
    if ref.startswith('/project/office_video_v2/scene/'):
        dst = ROOT/scene/ref.split('/scene/', 1)[1]
    elif ref.startswith('/project/office_video_v2/props/'):
        dst = ROOT/props/ref.split('/props/', 1)[1]
    else:
        raise ValueError(f'Unexpected scene reference: {ref}')
    assert dst.is_file(), dst
    el.set('file', os.path.relpath(dst, ROOT/scene))
xml_write(r, scene/'scene.xml')

catalog = []
for name, uid in [('coke','fq:objects/coke_cn'), ('round_table','fq:objects/round_table')]:
    catalog.append(entry(uid, 'rigid' if name=='coke' else 'fixture', props/name,
        f'{props}/mujoco/{name}.xml', f'{props}/isaac/{name}.usd', 'fq', 'SimFoundry (via FQ SIMPLE integration)',
        derived_from='provenance/tabletop_source_manifest.json',
        modifications=['FQ collision/scale/material adaptation', 'Chinese can label and UV repair' if name=='coke' else 'Photo-reference table material'],
        physics_calibrated=False))

for name, source, mj, usd in [
    ('photo_basket', SIMPLE/'data/assets/basket/photo_basket_v1', 'basket_asset.xml', 'basket.usd'),
    ('haier_bcd402', SIMPLE/'data/assets/refrigerator/haier_bcd402_v1', 'refrigerator.xml', 'refrigerator.usda')]:
    dest = Path('assets/fq')/name
    for p in source.iterdir():
        if p.name not in ('validation_history','validation','SHA256SUMS','references','previews'):
            cp(p, dest/p.name)
    original = json.loads((source/'manifest.json').read_text())
    catalog.append(entry('fq:articulated/'+name, 'articulated', dest,
        str(dest/mj), str(dest/usd), 'fq', 'FQ authored from user-supplied photos and dimensions',
        imported_from='FQ additions to SIMPLE; not SIMPLE-native stock assets',
        original_manifest=str(dest/'manifest.json'),
        initial_joint_positions=original['initial_joint_positions'],
        joint_limits_rad=original['joint_limits_rad'], physics_calibrated=False,
        limitations=original.get('limitations', ['Estimated dimensions and mass; passive handle hinges']),
        calibration_record=str(dest/'calibration_v2.json') if name=='haier_bcd402' else None))

# Export the conference table as a reusable static MJCF; preserve exact world-space geometry.
meeting = ET.Element('mujoco', model='fq_conference_table')
meeting.append(copy.deepcopy(r.find('asset')))
world = ET.SubElement(meeting,'worldbody')
selected = [g for g in r.find('worldbody') if g.get('name','').startswith('meeting_table')]
assert selected
for g in selected:
    world.append(copy.deepcopy(g))
# Only materials and textures used by these geoms are needed.
used = {g.get('material') for g in selected}
asset = meeting.find('asset')
textures = {a.get('texture') for a in asset if a.tag=='material' and a.get('name') in used}
for a in list(asset):
    if not ((a.tag=='material' and a.get('name') in used) or (a.tag=='texture' and a.get('name') in textures)):
        asset.remove(a)
meeting_path = Path('assets/fq/conference_table')
for a in asset:
    if a.get('file'):
        resolved=(ROOT/scene/a.get('file')).resolve()
        a.set('file',os.path.relpath(resolved,ROOT/meeting_path))
xml_write(meeting,meeting_path/'table.xml')
catalog.append(entry('fq:objects/conference_table','fixture',meeting_path,str(meeting_path/'table.xml'),str(meeting_path/'table.usda'),
    'fq','FQ floor-plan reconstruction', coordinates='Retains baseline world coordinates; not origin-normalized',
    measured_width_m=1.2, physics_calibrated=False))
catalog.append(entry('fq:scenes/office_baseline','scene',scene,str(scene/'scene.xml'),str(scene/'scene.usda'),
    'fq','FQ navigate office_video_v2; user floor plan and reference photos',
    baseline=True, randomization_enabled=False, robot_included=False,
    dependencies=['fq:objects/coke_cn','fq:objects/round_table','fq:objects/conference_table'],
    standalone_assets_not_placed=['fq:articulated/photo_basket','fq:articulated/haier_bcd402'],
    door_clear_width_m=0.8, conference_table_width_m=1.2,
    limitations=['Partial office reconstruction, not a surveyed complete building',
                 'Door fixed open; no door-opening dynamics',
                 'Invisible restricted-zone colliders intentionally preserved',
                 'Coke and round table are fixed in the navigation baseline; standalone coke is dynamic']))
write('catalog/index.json',dict(schema_version=1,release=VERSION,assets=catalog))
write('assets.lock.json',dict(schema_version=1,release=VERSION,assets={a['id']:a['version'] for a in catalog}))
write('provenance/source_files.json',dict(source_host='fangqi-4080s',captured_date='2026-10-10',files=records))
print(json.dumps(dict(root=str(ROOT),assets=len(catalog),source_files=len(records))))
