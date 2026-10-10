"""Prepare an isolated office mission; leave previous acceptance assets intact."""
from pathlib import Path
import shutil,json,xml.etree.ElementTree as E

project=Path('/home/fangqi/g1-navigation-sim')
run=project/'office_video_v2'
run.mkdir(exist_ok=True)
(run/'reports').mkdir(exist_ok=True)
src=project/'workspace/simulation/simulation/test_meeting'
shutil.copytree(src,run/'scene',dirs_exist_ok=True)
simple=Path('/home/fangqi/code/SIMPLE-o7-verify/production_scenes/grasp_coke')
shutil.copytree(simple/'common',run/'props/common',dirs_exist_ok=True)
shutil.copytree(simple/'isaac',run/'props/isaac',dirs_exist_ok=True)
root=E.parse(src/'scene.xml').getroot()
root.insert(0,E.Element('include',file='/project/sonic/assets/sonic_robot.xml'))
root.find('option').set('integrator','implicitfast')
asset=root.find('asset');world=root.find('worldbody')
for t in asset.findall('texture'):
    if t.get('file'):t.set('file','/project/office_video_v2/scene/'+t.get('file'))

def obj_bounds(path):
    vv=[list(map(float,l.split()[1:4])) for l in path.read_text().splitlines() if l.startswith('v ')]
    return [[min(v[i] for v in vv),max(v[i] for v in vv)] for i in range(3)]
tb=obj_bounds(simple/'common/meshes/round_table_textured.obj')
cb=obj_bounds(simple/'common/meshes/coke_cn_side.obj')
table=[10.8,11.55,-tb[2][0]]
coke=[10.8,11.55,table[2]+tb[2][1]-cb[2][0]+.002]
for name,pos in [('round_table',table),('coke',coke)]:
    r=E.parse(simple/'mujoco'/f'{name}.xml').getroot()
    for a in r.find('asset'):
        if a.get('name')=='studio_background':continue
        if a.get('file'):
            rel=a.get('file').replace('../common/','')
            a.set('file','/project/office_video_v2/props/common/'+rel)
        asset.append(a)
    b=r.find('worldbody/body')
    for j in b.findall('freejoint'):b.remove(j)
    b.set('pos',' '.join(map(str,pos)));world.append(b)
# Existing two meeting spotlights are retained; a visible panel adds desk lighting.
E.SubElement(world,'geom',name='meeting_panel',type='box',pos='2.7 10.4 2.62',size='.65 .25 .018',material='light_material',contype='0',conaffinity='0')
for light in world.findall('light'):
    if light.get('name') in ('ceiling_light_0','ceiling_light_1'):light.set('diffuse','1 1 .95')
E.indent(root)
E.ElementTree(root).write(run/'scene/mission.xml',encoding='unicode')
geo=json.loads((src/'geometry.json').read_text())
geo['rectangles'].append([table[0],table[1],max(abs(tb[0][0]),abs(tb[0][1])),max(abs(tb[1][0]),abs(tb[1][1]))])
(run/'scene/geometry.json').write_text(json.dumps(geo))
(run/'mission_config.json').write_text(json.dumps(dict(spawn=[11.3,5.5],table=table,coke=coke,table_bounds=tb,coke_bounds=cb,goals=[[11.2,10.45],[9.4,11.7],[6.25,geo['door_center'][1]],[4.65,geo['door_center'][1]],[3.6,11.7]],labels=['2','via','door approach','door passed','3']),indent=2))
sonic=project/'sonic'
p=(sonic/'physics.py').read_text().replace("ROOT/'assets/house.xml'","Path('/project/office_video_v2/scene/mission.xml')").replace('[-4.,-3.]','[11.3,5.5]')
p=p.replace("o7_startup.initialize(self.model,self.data,ROOT/'assets/model_decoder.onnx')", "o7_startup.initialize(self.model,self.data,ROOT/'assets/model_decoder.onnx',ground_z=0.)")
p=p.replace('self.controller,self.warmup=', 'self.data.qpos[3:7]=[np.cos(np.pi/4),0,0,np.sin(np.pi/4)]\n        self.controller,self.warmup=')
(run/'physics.py').write_text(p)
p=(sonic/'navigation_node.py').read_text().replace('from physics import SonicPhysics, ROOT',"from physics import SonicPhysics\nROOT=Path('/project/office_video_v2')")
p=p.replace('/project/workspace/simulation/simulation/aws_small_house_geometry.json','/project/office_video_v2/scene/geometry.json')
a=p.index('        for name,hx,hy in');b=p.index('        rect=np.asarray',a);p=p[:a]+p[b:]
a=p.index('        # Prescribed obstacle');b=p.index('        sim.step(',a);p=p[:a]+p[b:]
p=p.replace('sim.steps%50','sim.steps%25').replace('start<200','start<900')
(run/'navigation_node.py').write_text(p)
p=(sonic/'navigation.launch.py').read_text().replace('/project/workspace/simulation/simulation/aws_small_house_geometry.json','/project/office_video_v2/scene/geometry.json').replace('width=500,height=500','width=800,height=800').replace("'inflation_radius_m':.6","'inflation_radius_m':.30").replace("'lookahead_distance':.7","'lookahead_distance':.35").replace("'emergency_distance':.45","'emergency_distance':.28").replace("'avoid_radius':.85","'avoid_radius':.4")
(run/'navigation.launch.py').write_text(p)
p=p.replace("'execute_final_yaw':False", "'terminal_control_enabled':False,'execute_final_yaw':False")
p=p.replace("'turn_in_place_threshold':.44", "'turn_in_place_threshold':.22").replace("'turn_resume_threshold':.21", "'turn_resume_threshold':.10")
(run/'navigation.launch.py').write_text(p)
p=(sonic/'mission.py').read_text().replace('for goal in [[-4.,-4.5],[-5.4,-4.5]]:',"for goal in json.loads(Path('/project/office_video_v2/mission_config.json').read_text())['goals']:").replace(' < 85',' < 180').replace('/project/sonic/reports/','/project/office_video_v2/reports/')
(run/'mission.py').write_text(p)
(run/'run_demo.sh').write_text((sonic/'run_demo.sh').read_text().replace('/project/sonic/','/project/office_video_v2/'))
print((run/'mission_config.json').read_text())

