"""Check the interlock correction against a pre-update asset snapshot."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

p=argparse.ArgumentParser();p.add_argument('--reference',required=True);p.add_argument('--assets',default=str(Path(__file__).resolve().parents[1]));p.add_argument('--output',required=True);a=p.parse_args()
rel=Path('assets/fq/robots/g1_o6');old=Path(a.reference)/rel;new=Path(a.assets)/rel
m0=mujoco.MjModel.from_xml_path(str(old/'robot.xml'));m=mujoco.MjModel.from_xml_path(str(new/'robot.xml'))
assert (m.nq,m.nv,m.nu,m.nbody)==(m0.nq,m0.nv,m0.nu,m0.nbody)
for k in ['body_mass','body_inertia','jnt_type','jnt_axis','jnt_range','dof_damping','dof_frictionloss','actuator_ctrlrange','actuator_gainprm','eq_data','geom_contype','geom_conaffinity']:
    np.testing.assert_array_equal(getattr(m,k),getattr(m0,k),err_msg=k)
np.testing.assert_array_equal(m.body_ipos[m.body_mass>0],m0.body_ipos[m.body_mass>0])
u=ET.parse(new/'robot.urdf');meta=json.loads((new/'asset.json').read_text(encoding='utf-8'));profile=json.loads((new/'wrist_camera_visual_alignment_v1.json').read_text(encoding='utf-8'))
for side in ['left','right']:
    body=m.body(side+'_o6_hand_base_link')
    np.testing.assert_allclose(body.pos,[.065,0,0],atol=1e-12)
    np.testing.assert_allclose(np.fromstring(u.find(f"./joint[@name='{side}_o6_mount_joint']/origin").get('xyz'),sep=' '),body.pos,atol=1e-12)
    np.testing.assert_allclose(np.fromstring(meta['mounts'][side]['position'],sep=' '),body.pos,atol=1e-12)
    for name in [f'd435i_{side}_housing',f'd435i_{side}_rgb_optical',f'd435i_{side}_depth_optical']:
        o=u.find(f"./joint[@name='{name}_fixed']/origin");node=m.body(name) if name.endswith('housing') else m.site(name)
        if o is None:
            assert ET.parse(old/'robot.urdf').find(f"./joint[@name='{name}_fixed']/origin") is None
            continue
        np.testing.assert_allclose(np.fromstring(o.get('xyz'),sep=' '),node.pos,atol=1e-12)
    np.testing.assert_allclose(profile['cameras'][side]['housing_position_m'],m.body(f'd435i_{side}_housing').pos,atol=1e-12)
    for part in ['camera_bracket','hand_transfer_flange']:
        bid=m.body(f'{side}_o6_{part}_link').id
        ids=np.where(m.geom_bodyid==bid)[0];assert np.all(m.geom_contype[ids]==0) and np.all(m.geom_conaffinity[ids]==0)
d=mujoco.MjData(m);d0=mujoco.MjData(m0);worst=0
for amount in [0,.1,-.1]:
    d.qpos[:]=m.qpos0;d0.qpos[:]=m0.qpos0
    for j in range(m.njnt):
        if m.jnt_type[j]==3:d.qpos[m.jnt_qposadr[j]]+=amount;d0.qpos[m0.jnt_qposadr[j]]+=amount
    mujoco.mj_forward(m,d);mujoco.mj_forward(m0,d0)
    for side in ['left','right']:
        wid=m.body(side+'_wrist_yaw_link').id;rot=d.xmat[wid].reshape(3,3)
        np.testing.assert_allclose(d.xpos[wid],d0.xpos[wid],atol=1e-12)
        for i in range(m.nbody):
            name=m.body(i).name
            if not name.startswith(side+'_o6_') or any(s in name for s in ['camera','flange']):continue
            delta=rot.T@(d.xpos[i]-d0.xpos[i]);err=np.max(abs(delta-[-.01542,0,0]));worst=max(worst,float(err));assert err<1e-10,(name,delta)
            np.testing.assert_allclose(d.xmat[i],d0.xmat[i],atol=1e-12)
        for kind in ['rgb','depth']:
            i=m.camera(f'd435i_{side}_{kind}').id
            np.testing.assert_allclose(rot.T@(d.cam_xpos[i]-d0.cam_xpos[i]),[-.0085,0,0],atol=1e-12)
            np.testing.assert_allclose(d.cam_xmat[i],d0.cam_xmat[i],atol=1e-12)
result={'passed':True,'mujoco_version':mujoco.__version__,'poses_checked':3,'hand_translation_wrist_mm':[-15.42,0,0],'camera_translation_wrist_mm':[-8.5,0,0],'max_hand_translation_error_m':worst,'joint_actuator_mass_parameters_unchanged':True,'urdf_mjcf_positions_agree':True,'mount_collision_enabled':False,'hardware_calibrated':False}
Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
