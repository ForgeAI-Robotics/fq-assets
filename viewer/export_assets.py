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
    appearance=json.loads((root/'assets/fq/haier_bcd402/appearance.json').read_text())
    table_pbr=json.loads((root/'assets/fq/conference_table/pbr.json').read_text())
    round_pbr=root/'assets/fq/tabletop/round_table/pbr.json'
    if round_pbr.exists():table_pbr['materials'].update(json.loads(round_pbr.read_text())['materials'])
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
        for i,mat in enumerate(materials):
            matname=name(m,mujoco.mjtObj.mjOBJ_MATERIAL,i)
            part=matname.split('pbr_',1)[-1] if 'pbr_' in matname else None
            if part in appearance['region_assignment']:
                props=appearance['presets'][appearance['region_assignment'][part]]
                mat.update(roughness=props['roughness'],metalness=props['metallic'])
        for i,mat in enumerate(materials):
            matname=name(m,mujoco.mjtObj.mjOBJ_MATERIAL,i)
            props=table_pbr['materials'].get(matname)
            if props:
                mat.update(roughness=props['roughness'],metalness=props['metalness'])
                if props.get('roughness_texture'):
                    im=Image.open(root/props['roughness_texture']).convert('RGB')
                    im=im.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
                    if max(im.size)>2048:im.thumbnail((2048,2048))
                    im.save(dest/'roughness-table.png');mat['roughness_texture']='roughness-table.png'
                    mat['roughness']=1.0 # texture contains absolute roughness
        if slug=='mesh_trash_bin':
            for mat in materials:
                mat.update(roughness=1.0,metalness=.2)
                for channel in ('roughness','normal'):
                    im=Image.open(root/f'assets/fq/mesh_trash_bin/textures/powdercoat_{channel}.png').convert('RGB').transpose(Image.Transpose.FLIP_TOP_BOTTOM)
                    im.save(dest/f'{channel}-bin.png');mat[f'{channel}_texture']=f'{channel}-bin.png'
                mat['normal_scale']=.2
        if slug=='office_chair':
            props=json.loads((root/meta['appearance']).read_text())
            for i,mat in enumerate(materials):
                key=name(m,mujoco.mjtObj.mjOBJ_MATERIAL,i)
                if key not in props:continue
                mat.update(roughness=props[key]['roughness'],metalness=props[key]['metallic'])
                if key=='GreyWovenFabric':
                    im=Image.open(root/'assets/fq/office_chair/textures/fabric_basecolor.png').convert('RGB').transpose(Image.Transpose.FLIP_TOP_BOTTOM)
                    im.save(dest/'fabric-chair.png');mat['texture']='fabric-chair.png';mat['rgba']=[1,1,1,1]
                    for channel in ('roughness','normal'):
                        im=Image.open(root/f'assets/fq/office_chair/textures/fabric_{channel}.png').convert('RGB').transpose(Image.Transpose.FLIP_TOP_BOTTOM)
                        im.save(dest/f'{channel}-chair.png');mat[f'{channel}_texture']=f'{channel}-chair.png'
                    mat['normal_scale']=.25;mat['roughness']=1.0
        pbr_enabled=any('roughness' in mat for mat in materials)
        bodies=[];joints=[];geoms=[];bounds=[]
        for i in range(m.nbody):
            b={'id':i,'name':name(m,mujoco.mjtObj.mjOBJ_BODY,i),'parent':int(m.body_parentid[i]),
               'pos':m.body_pos[i].tolist(),'quat':m.body_quat[i].tolist(),'mass':float(m.body_mass[i]),'joints':[]}
            for j in range(int(m.body_jntadr[i]),int(m.body_jntadr[i]+m.body_jntnum[i])):
                if m.jnt_type[j] not in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE):continue
                jname=name(m,mujoco.mjtObj.mjOBJ_JOINT,j);qadr=int(m.jnt_qposadr[j]);dof=int(m.jnt_dofadr[j])
                limits=m.jnt_range[j].tolist() if m.jnt_limited[j] else [-math.pi,math.pi]
                joint={'type':'slide' if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else 'hinge','name':jname,'body':i,'pivot':m.jnt_pos[j].tolist(),'axis':m.jnt_axis[j].tolist(),
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
            if slug=='office_chair' and mat>=0 and rgba[3]>.01:
                # Blender/MuJoCo parameters are linear; viewer geom colors are sRGB.
                if name(m,mujoco.mjtObj.mjOBJ_MATERIAL,mat)=='GreyWovenFabric':rgba=[1,1,1,rgba[3]]
                else:rgba=[12.92*x if x<=.0031308 else 1.055*x**(1/2.4)-.055 for x in rgba[:3]]+[rgba[3]]
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
        couplings=[]
        for i in range(m.neq):
            if m.eq_type[i]!=mujoco.mjtEq.mjEQ_JOINT or m.eq_obj2id[i]<0:continue
            couplings.append({'joint':name(m,mujoco.mjtObj.mjOBJ_JOINT,int(m.eq_obj1id[i])),
                              'driver':name(m,mujoco.mjtObj.mjOBJ_JOINT,int(m.eq_obj2id[i])),
                              'polycoef':m.eq_data[i,:5].tolist()})
        actuators=[{'name':name(m,mujoco.mjtObj.mjOBJ_ACTUATOR,i),'joint':name(m,mujoco.mjtObj.mjOBJ_JOINT,int(m.actuator_trnid[i,0])),
                    'control_range':m.actuator_ctrlrange[i].tolist() if m.actuator_ctrllimited[i] else None,
                    'gear':m.actuator_gear[i].tolist()} for i in range(m.nu) if m.actuator_trntype[i]==mujoco.mjtTrn.mjTRN_JOINT]
        for j in joints:
            match=next((c for c in couplings if c['joint']==j['name']),None)
            if match:j['coupling']=match
        joint_by_name={j['name']:j for j in joints}
        for c in couplings:
            dependent=joint_by_name[c['joint']];driver=joint_by_name[c['driver']];co=c['polycoef']
            if any(abs(x)>1e-12 for x in co[2:]) or abs(co[1])<1e-12:
                raise ValueError('Unsupported nonlinear joint coupling')
            allowed=sorted((x-dependent['reference']-co[0])/co[1]+driver['reference'] for x in dependent['range'])
            limits=driver.get('control_range',driver['range'])
            driver['control_range']=[max(limits[0],allowed[0]),min(limits[1],allowed[1])]
            if driver['control_range'][0]>driver['control_range'][1]:raise ValueError('Incompatible coupled joint limits')
        title,kind_label,subtitle=NAMES.get(slug,(meta.get('name',slug),{'camera':'相机','robot':'机器人','end_effector':'灵巧手 / 末端执行器'}.get(meta['kind'],meta['kind']),meta.get('description','')))
        if meta['kind']=='camera':subtitle='RGB-D / 双目红外 / 六轴 IMU · 设备规格'
        if meta['kind'] in ('robot','end_effector'):subtitle='FQ 适配 / '+str(len(joints))+' 个转轴 · '+str(m.nu)+' 个驱动'

        info={**meta,'slug':slug,'name':title,'kind_label':kind_label,'subtitle':subtitle,'manifest_path':item['manifest'],
              'main_commit':args.commit,'dimension_m':(np.array(bbox[1])-bbox[0]).tolist(),
              'mass_kg':total_mass if total_mass>0 else None,'fixed_base':fixed,'bodies':[{k:b[k] for k in ('name','mass')} for b in bodies if b['mass']>0],
              'friction_range':{'min':friction.min(axis=0).tolist(),'max':friction.max(axis=0).tolist()} if len(friction) else None,
              'joints':joints,'couplings':couplings,'actuators':actuators,'opening_force':{'status':'uncalibrated','value':None},
              'property_source':'MuJoCo 3.3.6 编译后的模型参数','physics_status':'模型设定 · 未完成实测标定',
              'model_url':f'/data/{args.commit}/{slug}/model.json','thumbnail':f'/data/{args.commit}/{slug}/preview.png'}
        if meta['kind']=='material':
            with Image.open(material_source) as im:
                info['resolution']=list(im.size)
                im.thumbnail((256,256));im.save(dest/'preview.png')
            info.update(name=meta['name'],kind_label='材质/纹理',subtitle='表面外观 / '+('当前场景使用' if meta['usage_status']=='in_use' else '备用纹理'),mass_kg=None)
        info['pose_profiles']=[json.loads((root/p['path']).read_text()) for p in meta.get('profiles',[])]
        if meta.get('calibration_record'):
            info['calibration']=json.loads((root/meta['calibration_record']).read_text())
        preview=root/'validation/previews'/f"{meta['id'].replace(':','_').replace('/','_')}.png"
        if preview.is_file():shutil.copyfile(preview,dest/'preview.png')
        elif meta['kind'] in ('robot','end_effector','camera'):
            from PIL import ImageDraw
            im=Image.new('RGB',(256,256),(239,239,231));draw=ImageDraw.Draw(im)
            dots=[]
            for i in range(m.ngeom):
                if m.geom_type[i]!=mujoco.mjtGeom.mjGEOM_MESH or m.geom_rgba[i,3]<.01:continue
                mid=int(m.geom_dataid[i]);vertices=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
                vertices=vertices[::max(1,len(vertices)//1500)]@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
                dots.extend(np.column_stack((vertices[:,1]+.3*vertices[:,0],-vertices[:,2]+.15*vertices[:,0])))
            xy=np.asarray(dots);lo=xy.min(axis=0);hi=xy.max(axis=0);factor=220/max(hi-lo)
            xy=(xy-(lo+hi)/2)*factor+128
            for x,y in xy:draw.ellipse((x-1,y-1,x+1,y+1),fill=(95,105,98))
            im.save(dest/'preview.png')
        info['pbr_preview']={'supported':pbr_enabled,'scope':'部分材质' if meta['kind']=='scene' else '分区域材质','roughness_map':any(mat.get('roughness_texture') for mat in materials),'status':'仿真材质 · 非实测标定'}
        model={'couplings':couplings,'bodies':bodies,'joints':joints,'geoms':geoms,'meshes':meshes,'materials':materials,'bounds':bbox,
               'buffer':'geometry.bin','mesh_vertices':sum(x['count'] for x in meshes.values()),'up_axis':'Z'}
        dump(dest/'model.json',model);dump(dest/'properties.json',info)
        catalog.append(info)
        print(slug,len(binary),len(joints),flush=True)
    if (root/'actions/index.json').exists():
        for item in json.loads((root/'actions/index.json').read_text())['actions']:
            meta=json.loads((root/item['manifest']).read_text())
            robot=next(a for a in catalog if a['id']==meta['default_robot'])
            slug=meta['id'].split('/')[-1];dest=out/slug;dest.mkdir(exist_ok=True)
            trajectory=json.loads((root/meta['trajectory']).read_text());dump(dest/'trajectory.json',trajectory)
            info={**robot,**meta,'slug':slug,'manifest_path':item['manifest'],'kind_label':'动作','subtitle':meta['validation_status'],'model_url':robot['model_url'],'thumbnail':robot['thumbnail'],'trajectory_url':f'/data/{args.commit}/{slug}/trajectory.json','pose_profiles':[]}
            dump(dest/'properties.json',info);catalog.append(info)
    controllers=json.loads((root/'profiles/controllers/locomotion/index.json').read_text()) if (root/'profiles/controllers/locomotion/index.json').exists() else None
    dump(out/'catalog.json',{'main_commit':args.commit,'assets':catalog,'locomotion_controllers':controllers})

if __name__=='__main__':main()
