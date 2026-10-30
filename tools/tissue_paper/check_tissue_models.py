import mujoco
for name in ['preview.xml','flat.xml']:
 m=mujoco.MjModel.from_xml_path('/assets/assets/fq/tissue_paper/'+name);assert m.ngeom==2 and m.nmesh==2;print('TISSUE_MODEL_OK',name,int(m.mesh_facenum[0]))
