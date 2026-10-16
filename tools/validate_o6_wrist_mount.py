"""Validate fixed wrist geometry, optical conventions and unchanged robot physics."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--reference',required=True);p.add_argument('--assets',required=True);p.add_argument('--output',required=True);a=p.parse_args()
old=Path(a.reference);new=Path(a.assets);rel=Path('assets/fq/robots/g1_o6');m0=mujoco.MjModel.from_xml_path(str(old/rel/'robot.xml'));m=mujoco.MjModel.from_xml_path(str(new/rel/'robot.xml'))
profile=json.loads((new/rel/'wrist_camera_visual_alignment_v1.json').read_text());c=profile['cameras']['right'];hid=m.body('d435i_right_housing').id
for key in ['body_mass','body_inertia','body_ipos','jnt_type','jnt_pos','jnt_axis','jnt_range','dof_damping','dof_frictionloss','actuator_ctrlrange','actuator_gainprm','actuator_biasprm','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction','eq_data']:
    np.testing.assert_array_equal(getattr(m,key),getattr(m0,key),err_msg=key)
keep=[i for i in range(m.nbody) if i!=hid]
assert m.body_mass[hid]==0 and np.all(m.body_inertia[hid]==0),'Camera must remain massless visual-only'
np.testing.assert_array_equal(m.body_iquat[keep],m0.body_iquat[keep])
for key in ['body_pos','body_quat']:np.testing.assert_array_equal(getattr(m,key)[keep],getattr(m0,key)[keep])
for i in range(m.ncam):
    if not m.camera(i).name.startswith('d435i_right_'):
        for key in ['cam_pos','cam_quat','cam_intrinsic']:np.testing.assert_array_equal(getattr(m,key)[i],getattr(m0,key)[i])
np.testing.assert_allclose(m.camera('d435i_right_rgb').pos,c['camera_position_m'],atol=1e-12)
u=ET.parse(new/rel/'robot.urdf');worst=0.
for name in ['d435i_right_housing','d435i_right_rgb_optical','d435i_right_depth_optical']:
    j=u.find(f'./joint[@name="{name}_fixed"]');origin=j.find('origin');ur=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy'),sep=' ')).as_matrix();up=np.fromstring(origin.get('xyz'),sep=' ')
    node=m.body(name) if name.endswith('housing') else m.site(name);q=node.quat;mr=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();np.testing.assert_allclose(up,node.pos,atol=1e-12);np.testing.assert_allclose(ur,mr,atol=1e-12)
d=mujoco.MjData(m);d0=mujoco.MjData(m0)
for scale in [0,.1,-.1]:
    d.qpos[:]=m.qpos0;d0.qpos[:]=m0.qpos0
    for j in range(m.njnt):
        if m.jnt_type[j]==3:d.qpos[m.jnt_qposadr[j]]+=scale;d0.qpos[m0.jnt_qposadr[j]]+=scale
    mujoco.mj_forward(m,d);mujoco.mj_forward(m0,d0)
    np.testing.assert_allclose(d.xpos[keep],d0.xpos[keep],atol=1e-12);np.testing.assert_allclose(d.xmat[keep],d0.xmat[keep],atol=1e-12)
    for kind in ['rgb','depth']:
        cam=m.camera(f'd435i_right_{kind}').id;site=m.site(f'd435i_right_{kind}_optical').id
        err=np.max(abs(d.cam_xmat[cam].reshape(3,3)@np.diag([1,-1,-1])-d.site_xmat[site].reshape(3,3)));worst=max(worst,float(err));assert err<1e-12
        np.testing.assert_allclose(d.cam_xpos[cam],d.site_xpos[site],atol=1e-12)
result={'passed':True,'poses_checked':3,'mujoco_version':mujoco.__version__,'physics_unchanged':True,'left_head_default_unchanged':True,'urdf_mjcf_agree':True,'max_optical_basis_error':worst,'real_hardware_calibration':False,'render_profile_status':profile['status']}
Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
