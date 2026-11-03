from pathlib import Path
import bpy,json,shutil,hashlib,xml.etree.ElementTree as E
from mathutils import Matrix,Euler,Vector
R=Path('/assets');P=Path('/project')
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
catalog=json.loads((R/'catalog/index.json').read_text());lock=json.loads((R/'assets.lock.json').read_text())
for kind,slug,name in [('tianji','tianji_robot','Tianji 轮式双臂机器人（原始夹爪）'),('rhino','rhino_v6_gripper','RhinoV6 实装打印夹爪')]:
 src=P/(kind+'_inspection_20261103');D=R/'assets/user_supplied'/slug;assert not D.exists(),D
 D.mkdir();shutil.copytree(src/'source',D/'source',ignore=shutil.ignore_patterns('inspection_converted'))
 urdf=next((D/'source/urdf').glob('*.urdf'));r=E.parse(urdf).getroot()
 for mesh in r.findall('.//mesh'):
  suffix=mesh.get('filename').split('/',3)[-1];f=D/'source'/suffix;assert f.exists(),f
  mesh.set('filename',str(f.relative_to(D)))
 E.ElementTree(r).write(D/'model.urdf',encoding='utf-8',xml_declaration=True)
 bpy.ops.wm.read_factory_settings(use_empty=True)
 links={l.get('name'):l for l in r.findall('link')};parents={};joints={j.find('child').get('link'):j for j in r.findall('joint')}
 for n in links:
  ob=bpy.data.objects.new(n,None);bpy.context.collection.objects.link(ob);parents[n]=ob
 def transform(o):
  if o is None:return Matrix.Identity(4)
  pos=Vector(map(float,o.get('xyz','0 0 0').split()));rot=Euler(tuple(map(float,o.get('rpy','0 0 0').split())),'XYZ').to_matrix().to_4x4();return Matrix.Translation(pos)@rot
 for n,ob in parents.items():
  if n in joints:
   j=joints[n];ob.parent=parents[j.find('parent').get('link')];ob.matrix_local=transform(j.find('origin'));ob['joint_name']=j.get('name');ob['joint_type']=j.get('type')
 for n,l in links.items():
  for idx,v in enumerate(l.findall('visual')):
   mesh=v.find('geometry/mesh')
   if mesh is None:continue
   f=D/mesh.get('filename');before=set(bpy.data.objects)
   if f.suffix.lower()=='.stl':bpy.ops.import_mesh.stl(filepath=str(f))
   elif f.suffix.lower()=='.dae':bpy.ops.wm.collada_import(filepath=str(f))
   else:raise ValueError(f)
   objs=set(bpy.data.objects)-before;sc=list(map(float,mesh.get('scale','1 1 1').split()));T=transform(v.find('origin'))@Matrix.Diagonal(Vector(sc+[1]))
   color=v.find('material/color');rgba=tuple(map(float,color.get('rgba').split())) if color is not None else (.6,.65,.7,1)
   mat=bpy.data.materials.new(n+'_material');mat.diffuse_color=rgba;mat.use_nodes=True;mat.node_tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value=rgba
   for ob in objs:
    if ob.parent not in objs:ob.parent=parents[n];ob.matrix_local=T@ob.matrix_local
    if ob.type=='MESH' and f.suffix.lower()=='.stl':ob.data.materials.clear();ob.data.materials.append(mat)
 bpy.ops.export_scene.gltf(filepath=str(D/'model.glb'),export_format='GLB',export_yup=True,export_animations=False)
 folder=str(D.relative_to(R));aid='user_supplied:'+('robots/' if kind=='tianji' else 'grippers/')+slug
 limitations=(['Source model still carries Robotiq-named grippers; does not match the user-confirmed installed RhinoV6','Eight wheel joints declare zero effort/velocity','URDF mimic declarations require simulator-specific constraints; no actuators supplied'] if kind=='tianji' else ['Eight revolute joints have zero range, effort and velocity','No linkage closure, mimic constraints, spring forces or actuators supplied','Print tolerances and installation transform to Tianji are unverified'])
 limitations+=['GLB is a static zero-pose visual preview, not a physics articulation','Source collision meshes retained; contact behavior has not been validated','No claim that all STL links are individual fabrication-ready parts']
 meta=dict(schema_version=1,id=aid,version='0.1.0',kind='robot' if kind=='tianji' else 'articulated',name=name,description='用户原始装配与网格；静态预览，未配置驱动或完成实机标定',origin='user_supplied',upstream_origin='用户提供的 Tianji / RhinoV6 文件',license='Proprietary' if kind=='tianji' else 'LicenseRef-User-Supplied',license_status='Source package.xml retained; Rhino BSD field is unverified exporter metadata. Published to organization private repository at owner request.',units='meter',up_axis='Z',quaternion_order='wxyz',representations=dict(urdf=folder+'/model.urdf',glb=folder+'/model.glb'),dependencies=[],physics_calibrated=False,release_status='source_and_static_preview',limitations=limitations,viewer_status=[['状态','源模型与静态装配预览'],['驱动','未配置；不是可操作仿真版本'],['实机','RhinoV6 是用户确认的实装夹爪；Tianji 原包尚未替换']])
 save(D/'asset.json',meta)
 parts=[]
 for n,l in links.items():
  if kind=='rhino' or any(s in n.lower() for s in ['stand','mount','adapter','head']):
   meshes=[v.get('filename') for v in l.findall('visual/geometry/mesh')]
   if meshes:parts.append(dict(link=n,meshes=meshes,joint=joints[n].attrib if n in joints else None))
 save(D/'mounting_parts.json',parts)
 save(D/'provenance.json',dict(archive_name='tianji_latest_urdf_20261021.zip' if kind=='tianji' else 'RhinoV6.zip',archive_sha256=hashlib.sha256((src/'source.zip').read_bytes()).hexdigest(),source_files={str(f.relative_to(D/'source')):hashlib.sha256(f.read_bytes()).hexdigest() for f in (D/'source').rglob('*') if f.is_file()},changes=['Original package preserved byte-for-byte','Root model.urdf changes mesh references to repository-relative paths only','GLB visual conversion preserves link hierarchy at zero joint pose']))
 report=json.loads((src/'inspection.json').read_text());report['errors']=[e for e in report.get('errors',[]) if e[0]!='nonunit_axis'];report['native_import_note']=(src/'compatibility_result.txt').read_text();report['native_import_scope']='Inspection copy only; DAE converted to OBJ for Tianji. No calibrated controller, contact or coupling validation.';save(D/'validation.json',report)
 (D/'README.md').write_text('# '+name+'\n\n原包完整保留在 `source/`；`model.urdf` 仅改为可移植网格路径；`model.glb` 为零位静态装配预览。\n\n'+('\n'.join('- '+s for s in limitations))+'\n\n## 安装结构件\n\n见 `mounting_parts.json`，列出源 URDF 的 link、网格和连接关节。Tianji 的 `upper_adapter_flange.stl` 为左右共用法兰；`Link_Stand` 下含立柱/头部结构网格，`assembly_lower_mount` 下含 cad_11/12。空坐标系不作为实体支架。RhinoV6 的 base_link.STL 含基座，其他网格为夹爪连杆；不等于独立支架打印套件。CAD 编号件的具体制造名称未确认。\n\nTianji 源夹爪与 RhinoV6 几何不同。用户确认实机安装 RhinoV6；本次分别入库，未替换机器人末端或虚构安装校准。\n',encoding='utf-8')
 thumb=R/'validation/previews'/('user_supplied_'+('robots_' if kind=='tianji' else 'grippers_')+slug+'.png');shutil.copyfile(src/('assembly_preview.png' if kind=='tianji' else 'gripper_preview.png'),thumb)
 catalog['assets'].append(dict(id=aid,version='0.1.0',manifest=folder+'/asset.json'));lock['assets'][aid]='0.1.0'
 print('PACKAGED',slug,flush=True)
save(R/'catalog/index.json',catalog);save(R/'assets.lock.json',lock)
T=R/'tools/tianji_rhino';T.mkdir();shutil.copyfile(P/'publish_tianji_rhino.py',T/'publish_tianji_rhino.py')
shutil.copyfile(P/'tianji_rhino_gripper_comparison.png',T/'gripper_comparison.png')
with (R/'NOTICE.md').open('a') as f:f.write('\n## Tianji and RhinoV6 user-supplied source packages\n\nPublished to the private organization repository on user request. Original package metadata is preserved. Tianji declares Proprietary; RhinoV6 BSD metadata is not independently verified. No new open-source licensing grant is made. Static GLB previews and portable mesh paths were added; no physical calibration or gripper replacement was performed.\n')
