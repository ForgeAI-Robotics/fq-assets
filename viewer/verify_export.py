"""Compare browser body transforms against MuJoCo at multiple joint poses."""
import argparse,json
from pathlib import Path
import numpy as np
import mujoco

def transform(pos,quat):
    out=np.eye(4); rot=np.empty(9)
    mujoco.mju_quat2Mat(rot,np.asarray(quat,dtype=float))
    out[:3,:3]=rot.reshape(3,3);out[:3,3]=pos
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path);p.add_argument('--snapshot',type=Path);a=p.parse_args()
    cat=json.loads((a.snapshot/'catalog.json').read_text());checks=0
    for asset in cat['assets']:
        meta=json.loads((a.root/asset['manifest_path']).read_text())
        assert asset['version']==meta['version']
        model=mujoco.MjModel.from_xml_path(str(a.root/meta['representations']['mujoco']))
        exported=json.loads((a.snapshot/asset['slug']/'model.json').read_text())
        assert abs((asset['mass_kg'] or 0)-model.body_mass.sum())<1e-8
        for fraction in (0,.37,1):
            data=mujoco.MjData(model); angles={}
            for j in exported['joints']:
                value=j['range'][0]+fraction*(j['range'][1]-j['range'][0]);angles[j['name']]=value
                jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,j['name'])
                data.qpos[model.jnt_qposadr[jid]]=value
            mujoco.mj_forward(model,data);world=[]
            for b in exported['bodies']:
                matrix=transform(b['pos'],b['quat'])
                for name in b['joints']:
                    j=next(j for j in exported['joints'] if j['name']==name)
                    angle=angles[name]-j['reference'];q=[np.cos(angle/2),*(np.asarray(j['axis'])*np.sin(angle/2))]
                    pivot=np.asarray(j['pivot']);rot=transform([0,0,0],q);rot[:3,3]=pivot-rot[:3,:3]@pivot
                    matrix=matrix@rot
                if b['id']:matrix=world[b['parent']]@matrix
                world.append(matrix)
                np.testing.assert_allclose(matrix[:3,3],data.xpos[b['id']],atol=1e-7)
                np.testing.assert_allclose(matrix[:3,:3],data.xmat[b['id']].reshape(3,3),atol=1e-7)
                checks+=1
        print(asset['slug'],'OK')
    print(checks,'body-pose comparisons passed')
if __name__=='__main__':main()
