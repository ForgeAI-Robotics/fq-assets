from pathlib import Path
import json,numpy as np,mujoco
root=Path('/assets');snap=Path('/out');results=[]
for family,count in [('o6',7),('o7',3)]:
 p=root/f'assets/fq/robots/g1_{family}';model=mujoco.MjModel.from_xml_path(str(p/'robot.xml'));data=mujoco.MjData(model);export=json.loads((snap/f'g1_{family}/model.json').read_text());assert len(export['cameras'])==count
 profile=json.loads((p/'head_camera.json').read_text())
 for angle in [0,.2,-.2]:
  for n in ['waist_pitch_joint','waist_yaw_joint','left_wrist_yaw_joint']:data.qpos[model.joint(n).qposadr[0]]=angle
  mujoco.mj_forward(model,data);transforms={}
  for body in export['bodies']:
   T=np.eye(4);R=np.zeros(9);mujoco.mju_quat2Mat(R,np.array(body['quat']));T[:3,:3]=R.reshape(3,3);T[:3,3]=body['pos']
   for name in body['joints']:
    j=next(j for j in export['joints'] if j['name']==name);q=data.qpos[model.joint(name).qposadr[0]]-j['reference'];qq=np.r_[np.cos(q/2),np.array(j['axis'])*np.sin(q/2)];rr=np.zeros(9);mujoco.mju_quat2Mat(rr,qq);J=np.eye(4);J[:3,:3]=rr.reshape(3,3);pivot=np.array(j['pivot']);J[:3,3]=pivot-J[:3,:3]@pivot;T=T@J
   transforms[body['id']]=T if body['id']==0 else transforms[body['parent']]@T
  for s in export['cameras']:
   T=np.eye(4);R=np.zeros(9);mujoco.mju_quat2Mat(R,np.array(s['quaternion_wxyz']));T[:3,:3]=R.reshape(3,3);T[:3,3]=s['position'];T=transforms[s['body']]@T;cam=data.camera(s['name']);np.testing.assert_allclose(T[:3,3],cam.xpos,atol=1e-9);np.testing.assert_allclose(T[:3,:3],cam.xmat.reshape(3,3),atol=1e-9)
   k=s['intrinsics']
   if 'head' in s['name']:
    expected=profile['depth_intrinsics' if s['name'].endswith('_depth') else 'intrinsics']
    for key in ['width','height','fx','fy','cx','cy']:np.testing.assert_allclose(k[key],expected[key],atol=5e-5)
   # Off-center projection must return exact pixel corners and principal point.
   n=.0001;f=50;l=-k['cx']/k['fx']*n;r=(k['width']-k['cx'])/k['fx']*n;t=k['cy']/k['fy']*n;b=-(k['height']-k['cy'])/k['fy']*n
   P=np.array([[2*n/(r-l),0,(r+l)/(r-l),0],[0,2*n/(t-b),(t+b)/(t-b),0],[0,0,-(f+n)/(f-n),-2*f*n/(f-n)],[0,0,-1,0]])
   for u,v in [(0,0),(639,479),(k['cx'],k['cy'])]:
    point=np.array([(u-k['cx'])/k['fx'],-(v-k['cy'])/k['fy'],-1,1]);ndc=P@point;ndc/=ndc[3];np.testing.assert_allclose([(ndc[0]+1)*k['width']/2,(1-ndc[1])*k['height']/2],[u,v],atol=1e-8)
 results.append({'family':family,'exported_cameras':count,'fk_poses_verified':3,'intrinsics_and_off_axis_projection':True})
scene=json.loads((snap/'office_baseline/model.json').read_text());assert all(any(b['name']==n for b in scene['bodies']) for n in ['round_table','refrigerator_base']);assert all(any(j['name']==n for j in scene['joints']) for n in ['refrigerator_left_hinge','refrigerator_right_hinge'])
print(json.dumps({'passed':True,'robots':results,'baseline_anchors_and_door_joints':True},indent=2))
