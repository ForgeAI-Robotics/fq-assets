from pathlib import Path
import json,xml.etree.ElementTree as ET
import mujoco,numpy as np
from PIL import Image
R=Path(__file__).resolve().parents[1];D=R/'scenes/fq/airport_preview'
tests=[('floor',[4,-3,1],[0,0,0],.05,'Floor'),('worktop',[1.65,.75,1.4],[0,0,0],.7925,'Worktop'),('detector',[1.65,1.75,1.7],[0,0,0],1.1325,'Detector'),('exit',[0,3.25,.98],[0,1,0],None,'ReleaseBelt'),('entry',[0,-2.2,.98],[0,1,0],None,'EntryBelt'),('left',[-2,.7,.98],[-1,0,0],None,'BoundaryLBelt1'),('right',[2,-.6,.98],[1,0,0],None,'BoundaryRBelt0'),('front_flank',[-1.6,-2.2,.98],[0,1,0],None,'EntryFlankL'),('rear_flank',[-1.6,3.25,.98],[0,1,0],None,'ExitFlankL'),('central_aisle',[.3,-1.1,.98],[0,1,0],None,None)]
results=[]
for name,pos,vel,z,target in tests+[('gates_open',[0,-2.2,.98],[0,1,0],None,None)]:
    root=ET.parse(D/'scene.xml').getroot()
    for a in root.findall('.//asset/*'):
        if a.get('file'):a.set('file',str(D/a.get('file')))
    if name=='gates_open':
        for g in list(root.find('worldbody').findall('geom')):
            if g.get('name') in ('EntryBelt','ReleaseBelt'):root.find('worldbody').remove(g)
    opt=root.find('option');opt.set('gravity','0 0 -9.81' if z else '0 0 0');opt.set('timestep','.002')
    b=ET.SubElement(root.find('worldbody'),'body',name='probe',pos=' '.join(map(str,pos)));ET.SubElement(b,'freejoint');ET.SubElement(b,'geom',name='probe_geom',type='sphere',size='.05',mass='.1',contype='1',conaffinity='1')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);d.qvel[:3]=vel;touched=set()
    for _ in range(3500 if name=='gates_open' else 1000):
        mujoco.mj_step(m,d)
        for c in d.contact:touched.update(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,k) for k in (c.geom1,c.geom2))
    if z:assert abs(d.qpos[2]-z)<.015,(name,d.qpos)
    if target:assert target in touched,(name,touched)
    if name=='central_aisle':assert abs(d.qpos[1]-.9)<.02 and not touched
    if name=='gates_open':assert d.qpos[1]>4.7 and not touched
    if target and not z:
        assert -.1<np.dot(d.qpos[:3]-pos,np.array(vel))<.4,(name,d.qpos)
    results.append({'test':name,'passed':True,'final_xyz':d.qpos[:3].tolist(),'contacts':sorted(touched)})
m=mujoco.MjModel.from_xml_path(str(D/'scene.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
assert m.nbody==1 and m.njnt==0
ren=mujoco.Renderer(m,height=1080,width=1920);cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);cam.lookat[:]=[0,-.5,.5];cam.distance=10;cam.azimuth=115;cam.elevation=-48
opt=mujoco.MjvOption();mujoco.mjv_defaultOption(opt);opt.geomgroup[:]=1
ren.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(ren.render()).save(R/'validation/previews/fq_scenes_airport_preview.png')
cam.lookat[:]=[0,-.9,0];cam.distance=14;cam.azimuth=90;cam.elevation=-89
ren.update_scene(d,camera=cam,scene_option=opt);Image.fromarray(ren.render()).save(R/'validation/previews/airport_enclosure_v020_top.png');ren.close()
p=R/'validation/airport_preview_v020.json';r=json.loads(p.read_text());r['mujoco_dynamic_probes']=results;r['preview_rendered']=True;p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(results))
