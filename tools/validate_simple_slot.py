from pathlib import Path
import json,xml.etree.ElementTree as E
import numpy as np,mujoco
root=Path('/mnt/simple');out=Path('/out');report=[]
for family,folder,stem in [('o6','g1_o6','g1_29dof_wholebody_o6'),('o7','g1_o7_left','g1_29dof_wholebody_o7_left')]:
 paths=[root/'data/robots'/folder/(stem+'.xml')]
 if family=='o6':paths.extend([root/'data/robots'/folder/'d435i/g1_o6_mp.xml',root/'data/robots'/folder/'d435i/g1_o6_mp_capture.xml'])
 for path in paths:
  m=mujoco.MjModel.from_xml_path(str(path));tree=E.parse(out/'simple-backup'/path.relative_to(root));compiler=tree.getroot().find('compiler');compiler.set('meshdir',str(path.parent/compiler.get('meshdir','')));old=mujoco.MjModel.from_xml_string(E.tostring(tree.getroot(),encoding='unicode'))
  for key in ['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat','jnt_range','jnt_axis','dof_damping','dof_frictionloss','actuator_gainprm','actuator_biasprm','cam_pos','cam_quat','cam_intrinsic','site_pos','site_quat']:
   np.testing.assert_array_equal(getattr(m,key),getattr(old,key),err_msg=str(path)+key)
  d=mujoco.MjData(m);mujoco.mj_forward(m,d);record={'model':str(path.relative_to(root)),'compiled':True,'mass_inertia_joints_camera_unchanged':True}
  if family=='o6' and '/d435i/' in str(path):
   c=m.geom('d435i_head_collision').id;distances=[];gaps=[]
   for g in range(m.ngeom):
    if m.geom_bodyid[g]!=m.body('head_link').id or m.geom_group[g]!=3:continue
    distances.append(float(mujoco.mj_geomDistance(m,d,c,g,1.,None)))
    mi=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[mi]:m.mesh_vertadr[mi]+m.mesh_vertnum[mi]];world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];boxlocal=(world-d.geom_xpos[c])@d.geom_xmat[c].reshape(3,3)
    gap=float(max(np.max(boxlocal.min(0)-m.geom_size[c]),np.max(-m.geom_size[c]-boxlocal.max(0))));gaps.append(gap);assert gap>.0019
   record['camera_box_separating_axis_clearance_m']=min(gaps)
   record['mujoco_unfiltered_distances_m']=distances
   record['distance_note']='Analytic separating planes show >=1.9 mm clearance; mj_geomDistance reports zero for the rear hull. Same-weld self-contact is filtered. No claim of hardware clearance.'
  # Compiled visual meshes equal FQ's local-link geometry; copied original
  # source data is not silently selected by an old head mesh name.
  names=[m.mesh(int(m.geom_dataid[g])).name for g in range(m.ngeom) if m.geom_bodyid[g]==m.body('head_link').id];assert 'head_camera_slot' in names and 'head_link' not in names
  report.append(record)
(out/'simple-slot-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
