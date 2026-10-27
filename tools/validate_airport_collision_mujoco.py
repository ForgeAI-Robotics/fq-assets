from pathlib import Path
import json,xml.etree.ElementTree as ET
import mujoco,numpy as np
R=Path(__file__).resolve().parents[1];D=R/'scenes/fq/airport_preview'
tests=[('floor',[4,-3,1],[0,0,0],0.05,None),('worktop',[-1,0.65,1.4],[0,0,0],0.7925,'Worktop'),('detector',[0.19,1.38,1.7],[0,0,0],1.16,'Detector'),('belt',[0,3.25,0.98],[0,1,0],None,'ReleaseBelt'),('partition',[1.7,3.25,0.6],[0,1,0],None,'GatePartition1.7'),('open_passage',[0,6.4,0.6],[0,1,0],None,None)]
results=[]
for name,pos,vel,z,target in tests:
    root=ET.parse(D/'scene.xml').getroot()
    for a in root.findall('.//asset/*'):
        if a.get('file'):a.set('file',str(D/a.get('file')))
    option=root.find('option')
    if option is None:option=ET.SubElement(root,'option')
    option.set('gravity','0 0 -9.81' if z else '0 0 0');option.set('timestep','0.002')
    b=ET.SubElement(root.find('worldbody'),'body',name='probe',pos=' '.join(map(str,pos)))
    ET.SubElement(b,'freejoint');ET.SubElement(b,'geom',name='probe_geom',type='sphere',size='.05',mass='.1',contype='1',conaffinity='1')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);d.qvel[:3]=vel
    touched=set()
    for step in range(1000):
        mujoco.mj_step(m,d)
        for c in d.contact:
            touched.update(mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,k) for k in (c.geom1,c.geom2))
    if z:assert abs(d.qpos[2]-z)<.015,(name,d.qpos)
    elif target:assert d.qpos[1]<3.65 and target in touched,(name,d.qpos,touched)
    else:assert d.qpos[1]>8.2,(name,d.qpos)
    if target:assert target in touched
    results.append({'test':name,'passed':True,'final_xyz':d.qpos[:3].tolist(),'contacts':sorted(touched)})
p=R/'validation/airport_preview_v011.json';r=json.loads(p.read_text());r['mujoco_dynamic_probes']=results;p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(results))
