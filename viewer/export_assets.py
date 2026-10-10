"""Compile MJCF into compact browser geometry and factual physics metadata."""
from pathlib import Path
import argparse, gzip, hashlib, json, math, shutil, subprocess
import numpy as np
import mujoco
from PIL import Image

NAMES={'coke_cn':('中文可乐','刚体物体','饮料 / 可抓取'),
 'round_table':('圆桌','固定家具','桌面 / 工作区'),
 'conference_table':('会议桌','固定家具','会议室 / 长桌'),
 'photo_basket':('提手篮子','关节物体','容器 / 双提手'),
 'haier_bcd402':('双门冰箱','关节物体','海尔 BCD-402 / 三转轴'),
 'office_baseline':('办公基准场景','场景','测试区 → 走道 → 会议室')}

def dump(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')))
    with gzip.open(str(path)+'.gz','wb',compresslevel=6) as f:f.write(path.read_bytes())

def name(m,kind,i):return mujoco.mj_id2name(m,kind,i) or f'item_{i}'

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--commit',required=True)
    args=ap.parse_args();root=args.root.resolve();out=args.out;out.mkdir(parents=True,exist_ok=True)
    index=json.loads((root/'catalog/index.json').read_text());catalog=[]
    for item in index['assets']:
        meta=json.loads((root/item['manifest']).read_text())
        slug=meta['id'].split('/')[-1];dest=out/slug;dest.mkdir(exist_ok=True)
        if meta['kind']=='material':
            material_source=(root/meta['representations']['base_color']).resolve()
            if not material_source.is_relative_to(root):raise ValueError('External texture')
            preview_xml=dest/'swatch.xml'
            import xml.sax.saxutils
            filename=xml.sax.saxutils.quoteattr(str(material_source))
            preview_xml.write_text(f'<mujoco><asset><texture name="t" type="2d" file={filename}/><material name="m" texture="t" texrepeat="1 1"/></asset><worldbody><geom type="box" size=".75 .75 .015" pos="0 0 .015" material="m"/><geom type="sphere" size=".38" pos="0 0 .41" material="m"/></worldbody></mujoco>')
            entry_path=preview_xml.resolve()
        else:
            entry_path=(root/meta['representations']['mujoco']).resolve()
        if meta['kind']!='material' and not entry_path.is_relative_to(root):raise ValueError('External entry')
        m=mujoco.MjModel.from_xml_path(str(entry_path));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        binary=bytearray();meshes={};materials=[];used_meshes=set(int(i) for i in m.geom_dataid[m.geom_type==mujoco.mjtGeom.mjGEOM_MESH])
        for i in sorted(used_meshes):
            va,na,fa,ta=map(int,(m.mesh_vertadr[i],m.mesh_normaladr[i],m.mesh_faceadr[i],m.mesh_texcoordadr[i]))
            faces=m.mesh_face[fa:fa+m.mesh_facenum[i]]
            pos=m.mesh_vert[va+faces.reshape(-1)]
            normals=m.mesh_normal[na+m.mesh_facenormal[fa:fa+m.mesh_facenum[i]].reshape(-1)]
            uv=np.zeros((len(pos),2),dtype=np.float32)
            if ta>=0:
                ti=m.mesh_facetexcoord[fa:fa+m.mesh_facenum[i]].reshape(-1)
                valid=ti>=0;uv[valid]=m.mesh_texcoord[ta+ti[valid]]
            packed=np.concatenate([pos,normals,uv],axis=1).astype('<f4')
            meshes[str(i)]={'offset':len(binary),'count':len(pos)};binary.extend(packed.tobytes())
        (dest/'geometry.bin').write_bytes(binary)
        with gzip.open(dest/'geometry.bin.gz','wb',compresslevel=6) as f:f.write(binary)
        tex_cache={}
        for i in range(m.nmat):
            tid=int(m.mat_texid[i,1]);texture=None
            if tid>=0:
                texture=f'texture-{tid}.png'
                if tid not in tex_cache:
                    w,h,c=map(int,(m.tex_width[tid],m.tex_height[tid],m.tex_nchannel[tid]));start=int(m.tex_adr[tid])
                    pixels=m.tex_data[start:start+w*h*c].reshape(h,w,c)
                    im=Image.fromarray(pixels)
                    if max(im.size)>2048:im.thumbnail((2048,2048))
                    im.save(dest/texture,optimize=True);tex_cache[tid]=texture
            materials.append({'rgba':m.mat_rgba[i].tolist(),'texture':texture,'repeat':m.mat_texrepeat[i].tolist(),
                              'shininess':float(m.mat_shininess[i]),'specular':float(m.mat_specular[i])})
        bodies=[];joints=[];geoms=[];bounds=[]
        for i in range(m.nbody):
            b={'id':i,'name':name(m,mujoco.mjtObj.mjOBJ_BODY,i),'parent':int(m.body_parentid[i]),
               'pos':m.body_pos[i].tolist(),'quat':m.body_quat[i].tolist(),'mass':float(m.body_mass[i]),'joints':[]}
            for j in range(int(m.body_jntadr[i]),int(m.body_jntadr[i]+m.body_jntnum[i])):
                if m.jnt_type[j]!=mujoco.mjtJoint.mjJNT_HINGE:continue
                jname=name(m,mujoco.mjtObj.mjOBJ_JOINT,j);qadr=int(m.jnt_qposadr[j]);dof=int(m.jnt_dofadr[j])
                limits=m.jnt_range[j].tolist() if m.jnt_limited[j] else [-math.pi,math.pi]
                joint={'name':jname,'body':i,'pivot':m.jnt_pos[j].tolist(),'axis':m.jnt_axis[j].tolist(),
                       'range':limits,'reference':float(m.qpos0[qadr]),
                       'initial':float(meta.get('initial_joint_positions',{}).get(jname,m.qpos0[qadr])),
                       'damping':float(m.dof_damping[dof]),'friction_loss':float(m.dof_frictionloss[dof]),
                       'stiffness':float(m.jnt_stiffness[j])}
                joints.append(joint);b['joints'].append(jname)
            bodies.append(b)
        for i in range(m.ngeom):
            mat=int(m.geom_matid[i]);rgba=m.geom_rgba[i].tolist()
            # MuJoCo's default geom color defers to its material, explicit transparent colliders do not.
            if mat>=0 and np.allclose(rgba,[.5,.5,.5,1],atol=1e-6):rgba=m.mat_rgba[mat].tolist()
            gname=name(m,mujoco.mjtObj.mjOBJ_GEOM,i)
            g={'name':gname,'body':int(m.geom_bodyid[i]),'type':int(m.geom_type[i]),'mesh':int(m.geom_dataid[i]),
               'pos':m.geom_pos[i].tolist(),'quat':m.geom_quat[i].tolist(),'size':m.geom_size[i].tolist(),
               'rgba':rgba,'material':mat,'collision':bool(m.geom_contype[i] or m.geom_conaffinity[i]),
               'group':int(m.geom_group[i]),'friction':m.geom_friction[i].tolist(),
               'cutaway':gname.startswith(('ceiling_','downlight_fixture')) or gname=='meeting_panel'}
            geoms.append(g)
            if rgba[3]>.01 and g['type']!=mujoco.mjtGeom.mjGEOM_PLANE:
                if g['type']==mujoco.mjtGeom.mjGEOM_MESH:
                    mid=g['mesh'];v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
                    points=v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
                    bounds.extend([points.min(axis=0),points.max(axis=0)])
                else:
                    size=m.geom_size[i].copy()
                    if g['type']==mujoco.mjtGeom.mjGEOM_SPHERE:size[:]=size[0]
                    elif g['type'] in (mujoco.mjtGeom.mjGEOM_CYLINDER,mujoco.mjtGeom.mjGEOM_CAPSULE):size=np.array([size[0],size[0],size[1]+(size[0] if g['type']==3 else 0)])
                    ext=np.abs(d.geom_xmat[i].reshape(3,3))@size
                    bounds.extend([d.geom_xpos[i]-ext,d.geom_xpos[i]+ext])
        bbox=[np.min(bounds,axis=0).tolist(),np.max(bounds,axis=0).tolist()] if bounds else [[-.5]*3,[.5]*3]
        friction=np.array([g['friction'] for g in geoms if g['collision']])
        total_mass=float(m.body_mass.sum());fixed=not any(t==mujoco.mjtJoint.mjJNT_FREE for t in m.jnt_type)
        title,kind_label,subtitle=NAMES.get(slug,(slug,meta['kind'],meta.get('description','')))
        info={**meta,'slug':slug,'name':title,'kind_label':kind_label,'subtitle':subtitle,'manifest_path':item['manifest'],
              'main_commit':args.commit,'dimension_m':(np.array(bbox[1])-bbox[0]).tolist(),
              'mass_kg':total_mass if total_mass>0 else None,'fixed_base':fixed,'bodies':[{k:b[k] for k in ('name','mass')} for b in bodies if b['mass']>0],
              'friction_range':{'min':friction.min(axis=0).tolist(),'max':friction.max(axis=0).tolist()} if len(friction) else None,
              'joints':joints,'opening_force':{'status':'uncalibrated','value':None},
              'property_source':'MuJoCo 3.3.6 编译后的模型参数','physics_status':'模型设定 · 未完成实测标定',
              'model_url':f'/data/{args.commit}/{slug}/model.json','thumbnail':f'/data/{args.commit}/{slug}/preview.png'}
        if meta['kind']=='material':
            with Image.open(material_source) as im:
                info['resolution']=list(im.size)
                im.thumbnail((256,256));im.save(dest/'preview.png')
            info.update(name=meta['name'],kind_label='材质/纹理',subtitle='表面外观 / '+('当前场景使用' if meta['usage_status']=='in_use' else '备用纹理'),mass_kg=None)
        if meta.get('calibration_record'):
            info['calibration']=json.loads((root/meta['calibration_record']).read_text())
        preview=root/'validation/previews'/f"{meta['id'].replace(':','_').replace('/','_')}.png"
        if preview.is_file():shutil.copyfile(preview,dest/'preview.png')
        model={'bodies':bodies,'joints':joints,'geoms':geoms,'meshes':meshes,'materials':materials,'bounds':bbox,
               'buffer':'geometry.bin','mesh_vertices':sum(x['count'] for x in meshes.values()),'up_axis':'Z'}
        dump(dest/'model.json',model);dump(dest/'properties.json',info)
        catalog.append(info)
        print(slug,len(binary),len(joints),flush=True)
    dump(out/'catalog.json',{'main_commit':args.commit,'assets':catalog})

if __name__=='__main__':main()
