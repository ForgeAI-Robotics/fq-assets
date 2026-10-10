from pathlib import Path
import json,xml.etree.ElementTree as E,hashlib
import numpy as np
import mujoco
R=Path('/assets');report=[]
def compile(path):return mujoco.MjModel.from_xml_path(str(path))
for family,folder,filename in [('o6','g1_o6','g1_29dof_wholebody_o6'),('o7','g1_o7_left','g1_29dof_wholebody_o7_left')]:
    m=compile(R/f'assets/fq/robots/g1_{family}/robot.xml')
    original=compile(Path('/mnt/simple/data/robots')/folder/(filename+'.xml'))
    for i in range(original.nbody):
        before=original.body(i);after=m.body(before.name)
        for field in ('mass','inertia','pos','quat'):
            np.testing.assert_allclose(getattr(before,field),getattr(after,field),atol=1e-12)
    for field in ('jnt_range','dof_damping','dof_frictionloss','actuator_ctrlrange','eq_data'):
        np.testing.assert_array_equal(getattr(m,field),getattr(original,field),err_msg=family+':'+field)
    assert abs(m.body_mass.sum()-original.body_mass.sum())<1e-12
    report.append({'id':'fq:robots/g1_'+family,'source_robot_physics_equivalence':True,'mass_kg':float(m.body_mass.sum()),'hinge_joints':int(sum(m.jnt_type==3)),'actuators':m.nu})
    for side in ('left','right'):
        hm=compile(R/f'assets/fq/end_effectors/{family}_{side}/hand.xml');root=f'{side}_{family}_hand_base_link'
        hdata=mujoco.MjData(hm);data=mujoco.MjData(m)
        for i in range(1,hm.nbody):
            orig=m.body(hm.body(i).name)
            np.testing.assert_array_equal(hm.body_mass[i],orig.mass[0]);np.testing.assert_array_equal(hm.body_inertia[i],orig.inertia)
        for scale in (0,.1,.3,.5,.8):
            for j in range(hm.njnt):
                q=float(hm.jnt_range[j,0]*(1-scale)+hm.jnt_range[j,1]*scale)
                hdata.qpos[hm.jnt_qposadr[j]]=q;data.qpos[m.joint(hm.joint(j).name).qposadr[0]]=q
            mujoco.mj_forward(hm,hdata);mujoco.mj_forward(m,data)
            rid=m.body(root).id;rot=data.xmat[rid].reshape(3,3)
            for i in range(1,hm.nbody):
                oid=m.body(hm.body(i).name).id
                np.testing.assert_allclose(hdata.xpos[i],rot.T@(data.xpos[oid]-data.xpos[rid]),atol=1e-12)
                np.testing.assert_allclose(hdata.xmat[i].reshape(3,3),rot.T@data.xmat[oid].reshape(3,3),atol=1e-12)
        report.append({'id':f'fq:end_effectors/{family}_{side}','extracted_subtree_equivalence':True,'poses_checked':5,'mass_kg':float(hm.body_mass.sum()),'hinge_joints':hm.njnt,'actuators':hm.nu})
for item in json.loads((R/'catalog/index.json').read_text())['assets']:
    meta=json.loads((R/item['manifest']).read_text())
    if meta['kind'] not in ('robot','end_effector'):continue
    model=E.parse(R/meta['representations']['mujoco']).getroot()
    compiler=model.find('compiler');meshdir=compiler.get('meshdir','')
    for mesh in model.findall('./asset/mesh'):
        p=((R/meta['representations']['mujoco']).parent/meshdir/mesh.get('file')).resolve()
        assert p.is_relative_to(R) and p.is_file()
output={'passed':True,'mujoco_version':mujoco.__version__,'assets':report,'gpu_or_hardware_control_tested':False}
Path('/out/robots_validation.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))
