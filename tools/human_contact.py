"""Drive exported kinematic contact bodies and report model contact estimates."""
from pathlib import Path
import json,numpy as np,mujoco
class HumanContactRuntime:
 def __init__(self,model,data,asset_dir):
  self.model=model;self.data=data;root=Path(asset_dir);self.config=json.loads((root/'contact_regions.json').read_text());self.motion=json.loads((root/'contact_motion.json').read_text());self.regions={mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,r['mujoco_geom']):r for r in self.config['regions']};assert min(self.regions)>=0
  self.mocaps={r['mujoco_body']:int(model.body_mocapid[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,r['mujoco_body'])]) for r in self.config['regions']};assert min(self.mocaps.values())>=0
  if 'carried_bag_contact' in self.motion['frames'][0]['bodies']:
   bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'carried_bag_contact');self.mocaps['carried_bag_contact']=int(model.body_mocapid[bid]);gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'carried_bag');self.regions[gid]={'id':'bag','sampling_tag':'bag_surface'}
 def set_time(self,t):
  frames=self.motion['frames'];x=np.clip(t*self.motion['fps'],0,len(frames)-1);i=int(x);j=min(i+1,len(frames)-1);u=x-i
  for name,mid in self.mocaps.items():
   a=frames[i]['bodies'][name];b=frames[j]['bodies'][name];self.data.mocap_pos[mid]=(1-u)*np.array(a['position'])+u*np.array(b['position']);qa=np.array(a['quaternion']);qb=np.array(b['quaternion']);qb=qb if np.dot(qa,qb)>=0 else -qb;q=(1-u)*qa+u*qb;self.data.mocap_quat[mid]=q/np.linalg.norm(q)
  mujoco.mj_forward(self.model,self.data)
 def contacts(self,passenger_id='passenger'):
  result=[]
  for i,c in enumerate(self.data.contact):
   gid=c.geom1 if c.geom1 in self.regions else c.geom2 if c.geom2 in self.regions else None
   if gid is None:continue
   r=self.regions[gid];other=c.geom2 if gid==c.geom1 else c.geom1;force=np.zeros(6);mujoco.mj_contactForce(self.model,self.data,i,force);normal=np.array(c.frame).reshape(3,3)[0]*(1 if gid==c.geom1 else -1)
   result.append({'time_s':float(self.data.time),'passenger_id':passenger_id,'region':r['id'],'sampling_tag':r['sampling_tag'],'other_geom':mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,other),'position_m':c.pos.tolist(),'surface_normal':normal.tolist(),'normal_force_N':float(abs(force[0])),'force_is_model_estimate':True})
  return result
