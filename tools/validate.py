"""Offline release checks. Run against an isolated copy to prove resource closure."""
from pathlib import Path
import argparse, hashlib, json, math, os, re
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, Sdf

p=argparse.ArgumentParser()
p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
p.add_argument('--output',type=Path,required=True)
p.add_argument('--render',action='store_true')
args=p.parse_args()
root=args.root.resolve()
report={'mujoco_version':mujoco.__version__,'usd_version':list(Usd.GetVersion()),
        'root':str(root),'assets':[],'isaac_physx_runtime_tested':False}
catalog=json.loads((root/'catalog/index.json').read_text())
lock=json.loads((root/'assets.lock.json').read_text())
assert len({a['id'] for a in catalog['assets']})==len(catalog['assets'])

for item in catalog['assets']:
    meta=json.loads((root/item['manifest']).read_text())
    assert lock['assets'][meta['id']]==meta['version']==item['version']
    assert meta['origin'] in ('fq','simple') and meta['upstream_origin']
    entry={'id':meta['id']}
    xml=root/meta['representations']['mujoco']
    model=mujoco.MjModel.from_xml_path(str(xml))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    assert np.isfinite(data.qpos).all()
    entry.update(ngeom=model.ngeom,njoint=model.njnt)
    entry['joints']=[]
    for i in range(model.njnt):
        if model.jnt_type[i]!=mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i)
        limits=model.jnt_range[i].tolist()
        expected=meta.get('joint_limits_rad')
        if expected:
            exp=expected[name] if isinstance(expected,dict) else expected
            assert np.allclose(limits,exp,atol=1e-6),(name,limits,exp)
        for angle in np.linspace(*limits,5):
            data.qpos[model.jnt_qposadr[i]]=angle
            mujoco.mj_forward(model,data)
            assert np.isfinite(data.xpos).all()
        entry['joints'].append(dict(name=name,limits_rad=limits,kinematic_sweep_samples=5))
    if meta['id'] in ('fq:objects/coke_cn','fq:articulated/photo_basket'):
        spec=mujoco.MjSpec.from_file(str(xml))
        spec.worldbody.add_geom(name='release_test_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[3,3,.1])
        m=spec.compile();d=mujoco.MjData(m)
        d.qpos[2]+=.3
        for name,value in meta.get('initial_joint_positions',{}).items():
            d.qpos[m.joint(name).qposadr[0]]=value
        for _ in range(int(2/m.opt.timestep)):
            mujoco.mj_step(m,d)
        assert np.isfinite(d.qpos).all() and d.qpos[2]>-.02,(meta['id'],d.qpos)
        assert np.linalg.norm(d.qvel[:3])<.1,(meta['id'],d.qvel)
        entry['drop_test']={'seconds':2,'final_base_z':float(d.qpos[2]),'linear_speed':float(np.linalg.norm(d.qvel[:3]))}
    usd=root/meta['representations']['usd']
    layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(usd))
    assert not unresolved,(usd,list(unresolved))
    for dep in [Path(x.realPath) for x in layers]+[Path(x) for x in assets]:
        assert dep.resolve().is_relative_to(root),(usd,'external dependency',str(dep))
        assert dep.is_file(),dep
    stage=Usd.Stage.Open(str(usd))
    assert stage and stage.GetDefaultPrim()
    assert UsdGeom.GetStageUpAxis(stage)=='Z'
    assert abs(UsdGeom.GetStageMetersPerUnit(stage)-1)<1e-9
    entry['usd_dependency_files']=len(layers)+len(assets)
    entry['usd_revolute_joints']=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            j=UsdPhysics.RevoluteJoint(prim)
            lo,hi=j.GetLowerLimitAttr().Get(),j.GetUpperLimitAttr().Get()
            entry['usd_revolute_joints'].append(dict(path=str(prim.GetPath()),limits_deg=[lo,hi]))
    if meta['kind']=='articulated':
        assert len(entry['usd_revolute_joints'])==len(entry['joints'])
        usd_joints={j['path'].split('/')[-1]:j for j in entry['usd_revolute_joints']}
        for joint in entry['joints']:
            assert np.allclose(np.deg2rad(usd_joints[joint['name']]['limits_deg']),joint['limits_rad'],atol=1e-5)
    if meta['kind']=='scene':
        upper=model.geom('meeting_door_upper_jamb');lower=model.geom('meeting_door_lower_frame')
        width=(upper.pos[1]-upper.size[1])-(lower.pos[1]+lower.size[1])
        assert abs(width-.8)<3e-5,width
        assert abs(2*model.geom('meeting_table').size[1]-1.2)<1e-5
        for name in ('temporary_unreachable_southwest','temporary_unreachable_southeast'):
            g=model.geom(name)
            assert g.contype[0] and g.rgba[3]==0
        assert model.njnt==0,'Baseline must remain fixed and robot-free'
        for name in ('round_table','coke'):
            assert stage.GetPrimAtPath('/World/'+name)
            pos=UsdGeom.Xformable(stage.GetPrimAtPath('/World/'+name)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            assert np.allclose(pos,model.body(name).pos,atol=1e-6)
        entry['door_clear_width_m']=float(width)
        entry['fixed_layout_verified']=True
    if args.render:
        preview=args.output.parent/'previews'/ (meta['id'].replace(':','_').replace('/','_')+'.png')
        preview.parent.mkdir(parents=True,exist_ok=True)
        d=mujoco.MjData(model)
        for name,value in meta.get('initial_joint_positions',{}).items():
            d.qpos[model.joint(name).qposadr[0]]=value
        mujoco.mj_forward(model,d)
        cam=mujoco.MjvCamera()
        cam.lookat[:]=model.stat.center
        cam.distance=max(1.,float(model.stat.extent)*1.5)
        cam.azimuth=125;cam.elevation=-30
        if meta['kind']=='scene':
            cam.lookat[:]=[8.5,6.2,.3];cam.distance=24;cam.elevation=-70;cam.azimuth=90
            for i in range(model.ngeom):
                name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or ''
                if name.startswith(('ceiling_','downlight_fixture')) or name=='meeting_panel':
                    model.geom_rgba[i,3]=0
            entry['preview_is_cutaway']=True
        model.vis.headlight.ambient[:]=[.5,.5,.5]
        model.vis.headlight.diffuse[:]=[.7,.7,.7]
        opt=mujoco.MjvOption();opt.geomgroup[3]=0
        model.vis.global_.offwidth=960
        model.vis.global_.offheight=720
        with mujoco.Renderer(model,height=720,width=960) as renderer:
            renderer.update_scene(d,camera=cam,scene_option=opt)
            from PIL import Image
            Image.fromarray(renderer.render()).save(preview)
        entry['preview']=str(preview)
    report['assets'].append(entry)

report['passed']=True
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
