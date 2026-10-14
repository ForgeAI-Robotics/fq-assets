import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $=id=>document.getElementById(id);
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const deg=r=>r*180/Math.PI, rad=d=>d*Math.PI/180;
const jointLabels={handle_right_hinge:'右提手',handle_left_hinge:'左提手',refrigerator_left_hinge:'左门',refrigerator_right_hinge:'右门',refrigerator_mullion_hinge:'中间竖梁'};
const label=j=>jointLabels[j]||j.replace(/^left_/, '左 ').replace(/^right_/, '右 ').replace(/o[67]_/, '').replace(/thumb/g,'拇指').replace(/index/g,'食指').replace(/middle/g,'中指').replace(/ring/g,'无名指').replace(/pinky/g,'小指').replace(/hip/g,'髋').replace(/knee/g,'膝').replace(/ankle/g,'踝').replace(/waist/g,'腰').replace(/shoulder/g,'肩').replace(/elbow/g,'肘').replace(/wrist/g,'腕').replace(/_joint$/,'').replace(/_/g,' ');
const isRobot=a=>['robot','end_effector'].includes(a?.kind);
const independentDof=a=>a.joints.filter(j=>!j.coupling).length;
let catalog=null,selected=null,category='object',loadId=0,activeModel=null,modelRoot=null;
let bodyGroups=new Map(),jointMap=new Map(),angles=new Map(),pickables=[],colliders=[],cutaways=[],jointMode=false;
let showCollision=false,cutaway=true,ground=null,grid=null,drag=null,toastTimer;
const scene=new THREE.Scene();scene.background=new THREE.Color('#eeeee7');
const camera=new THREE.PerspectiveCamera(40,1,.01,1000);camera.up.set(0,0,1);
let renderer;
try{renderer=new THREE.WebGLRenderer({antialias:true,alpha:false});}
catch(e){$('loading').hidden=true;$('viewer-error').hidden=false;$('viewer-error').textContent='当前浏览器无法启动 WebGL。请开启硬件加速后重试。';throw e;}
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.3;
renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
$('canvas-host').append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.075;
controls.minPolarAngle=.07;controls.maxPolarAngle=Math.PI*.9;controls.screenSpacePanning=true;
scene.add(new THREE.HemisphereLight(0xfaf9f3,0xa9b4a1,2.3));
const keyLight=new THREE.DirectionalLight(0xfff9e9,3.2);keyLight.castShadow=true;keyLight.shadow.mapSize.set(2048,2048);keyLight.shadow.bias=-.00015;scene.add(keyLight);scene.add(keyLight.target);
const fill=new THREE.DirectionalLight(0xffffff,1.6);scene.add(fill);scene.add(fill.target);
new ResizeObserver(()=>{const w=$('canvas-host').clientWidth,h=$('canvas-host').clientHeight;if(w&&h){camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h);}}).observe($('canvas-host'));
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});

function toast(text){$('toast').textContent=text;$('toast').classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('show'),2800);}
function fmt(n,d=3){return n==null?'—':Number(n).toLocaleString('zh-CN',{maximumFractionDigits:d});}
function row(key,value,cls=''){return `<div class="prop-row"><span>${esc(key)}</span><strong class="${cls}">${value}</strong></div>`;}
function section(title,body){return `<section class="prop-section"><h3>${esc(title)}</h3>${body}</section>`;}
function provenance(a){if(a.kind==='camera')return 'RealSense 上游 · FQ 适配';if(isRobot(a))return 'FQ 适配 · '+(a.kind==='robot'?'Unitree / LinkerHand':'LinkerHand');if(a.origin==='simple')return 'SIMPLE 原生';return (a.upstream_origin||'').includes('SimFoundry')?'FQ 派生 · SimFoundry':'FQ 自建';}
function showProperties(a){
  if(a.kind==='material'){
    $('properties-content').innerHTML=section('材质/纹理',row('名称',esc(a.name))+row('来源',esc(provenance(a)))+row('分辨率',a.resolution.join(' × '))+row('色彩空间',esc(a.color_space))+row('贴图通道','基础颜色 · 法线/粗糙度贴图暂缺')+row('实际重复尺寸',a.tile_size_m?`${a.tile_size_m} m（估计）`:'未记录')+row('用途',a.usage_status==='in_use'?'基准场景使用中':'备用，未绑定基准场景')+'<p class="property-note">视觉外观不包含质量或接触摩擦。预览光照为近似效果。</p>')+section('版本与来源',row('物体版本',`v${esc(a.version)}`)+row('内容分支','main')+row('当前提交',esc(catalog.main_commit.slice(0,8)))+row('最新库发布',esc(catalog.latest_release.tag))+`<p class="property-note">${esc(a.upstream_origin)}</p><div class="asset-id">${esc(a.id)}</div><p class="property-note">引用：${esc(a.used_by.join(', ')||'暂无')}</p><a class="text-button" target="_blank" rel="noopener" href="${catalog.repository}/blob/${catalog.main_commit}/${a.manifest_path}">查看材质清单 ↗</a>`);
    return;
  }
  let html=section('基础信息',row('资产类型',esc(a.kind_label))+row('来源',esc(provenance(a)))+row('坐标约定','米制 · Z 向上')+row('基座',a.fixed_base?'固定':'自由运动')+`<div class="asset-id">${esc(a.id)}</div>`);
  const mass=a.mass_properties; const displayMass=mass?mass.total_mass_kg:a.mass_kg;
  html+=section('尺寸与质量',`<div class="value-large">${displayMass==null?'—':fmt(displayMass)}<small>${displayMass==null?'未定义动态质量':'kg'}</small></div><p class="value-caption">${displayMass==null?'固定场景几何不以 0 kg 表示':mass?'整机参考质量 · 部件分配为估计':a.kind==='scene'?'已定义物体质量合计 · 非场景总重':a.kind==='camera'?'近似仿真质量 · 非实测':a.kind==='end_effector'?'手模型质量 · 未完成实测标定':a.fixed_base?'已定义部件质量合计 · 非整机重量':'模型总质量 · 未完成实测标定'}</p>`+
    (!mass&&a.fixed_base&&!['scene','end_effector','camera'].includes(a.kind)&&displayMass!=null?'<p class="property-note">固定基座未定义完整质量，整机重量暂缺。</p>':'')+
    (a.dimensions_body_m?row('篮身尺寸（不含提手）',a.dimensions_body_m.map(v=>fmt(v)).join(' × ')+' m','value-small')+row('底部尺寸（估计）','0.240 × 0.150 m','value-small'):'')+
    (mass?row('可动部件质量（估计）',`${fmt(mass.movable_mass_kg)} kg`)+row('固定柜体总成（估计）',`${fmt(mass.fixed_assembly_mass_kg)} kg`)+`<a class="text-button" target="_blank" rel="noopener" href="${esc(mass.total_source_url)}">${esc(mass.model)} 官方整机重量 ↗</a>`:'')+
    row('外观包围尺寸',a.dimension_m.map(v=>fmt(v)).join(' × ')+' m','value-small')+
    (a.bodies.length>1?`<details class="property-details"><summary>查看 ${a.bodies.length} 个部件质量</summary>${a.bodies.map(b=>row(esc(b.name),`${fmt(b.mass)} kg`,'value-small')).join('')}</details>`:''));
  if(a.kind==='camera'){
    const rgb=a.simulation.rgb_profile,depth=a.simulation.depth_profile;
    html+=section('设备规格',row('型号','RealSense D435i')+row('标称外壳尺寸',a.device_spec.nominal_size_mm.join(' × ')+' mm')+row('硬件能力',a.device_spec.channels.map(esc).join(' / '))+`<a class="text-button" target="_blank" rel="noopener" href="${esc(a.device_spec.spec_url)}">厂商规格 ↗</a>`);
    html+=section('仿真参考配置',row('RGB',`${rgb.width} × ${rgb.height}`)+row('深度',`${depth.width} × ${depth.height}`)+row('IMU 仿真','未接入')+`<details class="property-details"><summary>查看参考内参</summary>${row('RGB fx / fy',`${fmt(rgb.fx)} / ${fmt(rgb.fy)}`)}${row('RGB cx / cy',`${fmt(rgb.cx)} / ${fmt(rgb.cy)}`)}${row('Depth fx / fy',`${fmt(depth.fx)} / ${fmt(depth.fy)}`)}${row('Depth cx / cy',`${fmt(depth.cx)} / ${fmt(depth.cy)}`)}</details><p class="property-note">${esc(a.simulation.calibration_status)}</p>`);
    html+=section('使用状态',`<p class="property-note">${a.limitations.map(esc).join('<br>')}</p><a class="text-button" href="/?asset=g1_o6">查看 G1 + O6 装配 ↗</a>`);
  }
  if(a.camera_mounts)html+=section('相机装配',row('型号','D435i × 3')+row('位置','头部 / 左腕 / 右腕')+row('装配状态','现有参考位姿 · 未重新标定')+'<p class="property-note">仅视觉替换，未新增相机质量或碰撞。</p><a class="text-button" href="/?asset=realsense_d435i">查看相机资产 ↗</a>');
  if(a.slug==='haier_bcd402'&&a.calibration?.revision==='v3-user-floor-to-door-bottom')html+=section('门体尺寸',row('地面到上部门下沿',`${fmt(a.calibration.upper_door_bottom_z_m*100,1)} cm`)+row('模型上门高度',`${fmt(a.calibration.upper_door_height_m*100,1)} cm`)+ '<p class="property-note">82 cm 为用户实测。门顶沿用现有模型位置，下方抽屉交界为对应调整。</p>');
  if(a.friction_range){
    const fr=i=>{let lo=a.friction_range.min[i],hi=a.friction_range.max[i];return Math.abs(lo-hi)<1e-8?fmt(lo,5):`${fmt(lo,5)} – ${fmt(hi,5)}`;};
    html+=section('接触参数 / MUJOCO',row('滑动摩擦',fr(0))+row('扭转摩擦',`${fr(1)} m`)+row('滚动摩擦',`${fr(2)} m`)+`<p class="property-note">模型设定值；有多个碰撞面时显示范围。滑动摩擦为无量纲，扭转与滚动参数使用长度单位。</p>`);
  }
  if(isRobot(a)){
    html+=section('关节与驱动',row('独立关节自由度',independentDof(a))+row('模型转动关节数',a.joints.length)+row('驱动数',a.actuators.length)+row('从动关节数',a.couplings.length)+(a.kind==='robot'?row('本体＋双手',`29 + ${a.family==='o6'?'6 + 6 = 41':'7 + 7 = 43'}`)+row('浮动基座','另有 6 自由度，不计入关节自由度'):'')+'<p class="property-note">滑块控制独立关节，手指从动关节按模型耦合关系联动。驱动范围为模型控制输入，不等于实测力矩。</p>'+a.actuators.map(v=>`<details class="property-details"><summary>${esc(label(v.joint))}</summary>${row('关节范围',a.joints.find(j=>j.name===v.joint)?.range.map(x=>fmt(deg(x),1)+'°').join(' ～ ')||'—')}${row('驱动输入范围',v.control_range? v.control_range.map(x=>fmt(x)).join(' ～ '):'未限定')}${row('传动系数',v.gear.map(x=>fmt(x)).join(', '))}</details>`).join(''));
    html+=section('模型状态',`<p class="property-note">${(a.limitations||[]).map(esc).join('<br>')}</p>`+row('上游手模型版本',esc(a.vendor_revision.slice(0,8)))+row('模型类型','现有仿真模型快照')+(a.family==='o7'?row('手模型代次','O7 v1（非 v3）'):''));
    if(a.dependencies?.length)html+=section('组合依赖',a.dependencies.map(id=>row(id,`v${esc(a.dependency_versions[id])}`)).join(''));
  }
  if(a.joints.length&&!isRobot(a)){
    html+=section('关节与阻力',row('可动关节',`${a.joints.length} 个`)+
      (a.slug==='haier_bcd402'?(a.simulation_resistance?row('右门等效摩擦阻力',`${a.simulation_resistance.frictionloss_Nm} N·m`)+row('每条轨迹随机范围','1.0–2.3 N·m · 均匀采样')+row('右门粘性阻尼',`${a.simulation_resistance.damping_Nm_s_rad} N·m·s/rad`)+row('参数状态','仿真参数 · 非实测标定','status-label')+'<p class="property-note">随机模式每条轨迹采样一次并保持不变，只作用于右门。浏览器仅展示姿态；USD/PhysX 已接入 SIMPLE 阻力运行时（CPU 验证），支持固定 / 随机模式，回放须使用同一引擎。门封启动吸力未标定。</p>':row('开门力度','未标定','status-label')):'<p class="property-note">两个被动提手转轴，角度受模型限位约束。浏览器不模拟抓取或接触力。</p>')+
      a.joints.map(j=>`<details class="property-details"><summary>${esc(label(j.name))} · ${fmt(deg(j.range[0]),1)}° ～ ${fmt(deg(j.range[1]),1)}°</summary>${row('阻尼',`${fmt(j.damping,5)} N·m·s/rad`)}${row('关节摩擦力矩',`${fmt(j.friction_loss,5)} N·m`)}${row('刚度',`${fmt(j.stiffness,5)} N·m/rad`)}<p class="property-note">以上为模型参数，非实测开门/提手作用力。</p></details>`).join(''));
  }
  html+=section('版本与来源',row(a.kind==='scene'?'场景版本':'物体版本',`v${esc(a.version)}`)+row('内容分支','main')+row('当前提交',esc(catalog.main_commit.slice(0,8)),'mono')+
    row('最新库发布',esc(catalog.latest_release.tag))+
    `<p class="property-note">本页展示 main 上的当前资产。整体发布版本与单个物体版本分别记录。</p>`+
    `<details class="property-details"><summary>来源与模型依据</summary><p class="property-note">${esc(a.upstream_origin)}</p><p class="property-note">属性取自 ${esc(a.property_source)}。${a.dimensions_body_m?'篮身尺寸依据产品标签；底部、圆角和提手细节为估计值。':a.calibration?'保留用户尺寸或标定记录；部分参数仍为估算。':''}</p><a class="text-button" target="_blank" rel="noopener" href="${catalog.repository}/blob/${catalog.main_commit}/${a.manifest_path}">查看资产清单 ↗</a></details>`);
  $('properties-content').innerHTML=html;
}
function renderList(){
  if(!catalog)return;
  const q=$('search').value.trim().toLowerCase(),origin=$('origin-filter').value,type=$('type-filter').value;
  const items=catalog.assets.filter(a=>(category==='scene'?a.kind==='scene':a.kind!=='scene')&&(!q||`${a.name} ${a.id} ${a.subtitle}`.toLowerCase().includes(q))&&(origin==='all'||a.origin===origin)&&(category==='scene'||type==='all'||a.kind===type));
  $('asset-list').innerHTML=items.length?items.map(a=>`<button class="asset-card ${selected?.id===a.id?'active':''}" data-slug="${esc(a.slug)}" aria-pressed="${selected?.id===a.id}" aria-label="查看${esc(a.name)}"><span class="asset-thumb"><img src="${a.thumbnail}" alt="" loading="lazy"></span><span class="asset-card-info"><div class="asset-card-name">${esc(a.name)}</div><div class="asset-card-meta"><span>v${esc(a.version)}</span><span class="source-badge">${esc(a.origin.toUpperCase())}</span></div></span></button>`).join(''):'<div class="empty-note">没有匹配的资产。<br>尝试其他关键词或筛选条件。</div>';
  document.querySelectorAll('.asset-card').forEach(b=>b.onclick=()=>selectAsset(b.dataset.slug));
}
function setCategory(kind){category=kind;document.querySelectorAll('.tabs button').forEach(b=>{b.classList.toggle('active',b.dataset.kind===kind);b.setAttribute('aria-selected',b.dataset.kind===kind);});$('type-filter').disabled=kind==='scene';renderList();}
async function refresh(){
  try{
    const res=await fetch('/api/catalog',{cache:'no-store'});if(!res.ok)throw Error('连接失败');const next=await res.json();
    const changed=!!catalog&&catalog.main_commit!==next.main_commit;catalog=next;
    $('release').textContent=next.latest_release.tag==='unknown'?'读取中':next.latest_release.tag;
    if(next.latest_release.url?.startsWith('https://github.com/'))$('release').href=next.latest_release.url;
    $('sync-dot').className='dot'+(next.sync.error?' error':next.sync.syncing?' pending':'');
    $('sync-text').textContent=next.sync.error?'同步暂不可用':next.sync.syncing?'main 同步中':'main 已同步';
    $('sync-text').title=next.sync.error||`每 ${next.sync_interval_seconds} 秒同步 main`;
    $('checked-at').textContent=next.sync.last_success?`已同步 ${new Date(next.sync.last_success).toLocaleTimeString('zh-CN',{hour:'2-digit',minute:'2-digit'})} · 每分钟更新`:'正在准备首次快照';
    $('object-count').textContent=next.assets.filter(a=>a.kind!=='scene').length;$('scene-count').textContent=next.assets.filter(a=>a.kind==='scene').length;$('total').textContent=next.assets.length;
    renderList();
    if(next.assets.length&&(!selected||changed)){
      const requested=new URL(location.href).searchParams.get('asset');
      const a=next.assets.find(a=>a.slug===(selected?.slug||requested))||next.assets[0];
      await selectAsset(a.slug);if(changed)toast('main 已更新，正在展示最新资产内容');
    }
    if(!next.assets.length){$('sync-text').textContent=next.sync.error?'首次同步失败':'首次资产转换中';setTimeout(refresh,3000);}
  }catch(e){$('sync-text').textContent='连接暂时中断';$('sync-dot').className='dot error';if(!selected){$('loading').querySelector('strong').textContent='正在等待服务连接';setTimeout(refresh,5000);}}
}
function clearModel(){
  if(modelRoot){const gs=new Set(),ms=new Set(),ts=new Set();modelRoot.traverse(o=>{if(o.geometry)gs.add(o.geometry);if(o.material){ms.add(o.material);if(o.material.map)ts.add(o.material.map);}});gs.forEach(g=>g.dispose());ms.forEach(m=>m.dispose());ts.forEach(t=>t.dispose());scene.remove(modelRoot);}
  if(ground){scene.remove(ground);ground.geometry.dispose();ground.material.dispose();}
  if(grid){scene.remove(grid);grid.geometry.dispose();grid.material.dispose();}
  ground=null;grid=null;bodyGroups.clear();jointMap.clear();angles.clear();pickables=[];colliders=[];cutaways=[];drag=null;controls.enabled=true;
}
function primitive(g){
  const s=g.size;switch(g.type){
    case 0:return new THREE.PlaneGeometry(Math.max(s[0]*2,30),Math.max(s[1]*2,30));
    case 2:return new THREE.SphereGeometry(s[0],28,18);
    case 3:{const v=new THREE.CapsuleGeometry(s[0],s[1]*2,8,24);v.rotateX(Math.PI/2);return v;}
    case 4:{const v=new THREE.SphereGeometry(1,28,18);v.scale(...s);return v;}
    case 5:{const v=new THREE.CylinderGeometry(s[0],s[0],s[1]*2,40);v.rotateX(Math.PI/2);return v;}
    case 6:return new THREE.BoxGeometry(s[0]*2,s[1]*2,s[2]*2);
    default:return new THREE.BoxGeometry(.02,.02,.02);
  }
}
function nearestJoint(body){let b=body;while(b>0){const item=activeModel.bodies[b];if(item.joints.length)return item.joints[0];b=item.parent;}return null;}
async function buildModel(data,url,token){
  const bufferURL=new URL(data.buffer,url),buffer=await fetch(bufferURL).then(r=>{if(!r.ok)throw Error('几何文件加载失败');return r.arrayBuffer();});
  const loader=new THREE.TextureLoader(),textures=new Map();
  await Promise.all(data.materials.filter(m=>m.texture).map(async m=>{const u=new URL(m.texture,url).href;if(textures.has(u))return;const tex=await loader.loadAsync(u);tex.flipY=false;tex.colorSpace=THREE.SRGBColorSpace;tex.wrapS=tex.wrapT=THREE.RepeatWrapping;tex.anisotropy=Math.min(8,renderer.capabilities.getMaxAnisotropy());textures.set(u,tex);}));
  if(token!==loadId){textures.forEach(t=>t.dispose());return;}
  clearModel();activeModel=data;modelRoot=new THREE.Group();scene.add(modelRoot);
  const geometries={};
  for(const [id,entry] of Object.entries(data.meshes)){
    const array=new Float32Array(buffer,entry.offset,entry.count*8),interleaved=new THREE.InterleavedBuffer(array,8),geo=new THREE.BufferGeometry();
    geo.setAttribute('position',new THREE.InterleavedBufferAttribute(interleaved,3,0));geo.setAttribute('normal',new THREE.InterleavedBufferAttribute(interleaved,3,3));geo.setAttribute('uv',new THREE.InterleavedBufferAttribute(interleaved,2,6));geo.computeBoundingSphere();geometries[id]=geo;
  }
  for(const b of data.bodies){const group=new THREE.Group();group.name=b.name;bodyGroups.set(b.id,group);if(b.id===0)modelRoot.add(group);else bodyGroups.get(b.parent).add(group);group.matrixAutoUpdate=false;}
  data.joints.forEach(j=>{jointMap.set(j.name,j);angles.set(j.name,j.initial);});
  for(const g of data.geoms){
    const geometry=g.type===7?geometries[g.mesh]:primitive(g);if(!geometry)continue;
    const materialDef=data.materials[g.material],color=new THREE.Color().setRGB(g.rgba[0],g.rgba[1],g.rgba[2],THREE.SRGBColorSpace);
    if(g.rgba[3]>.01&&g.group!==3&&!(isRobot(selected)&&g.collision&&g.group!==1&&data.geoms.some(v=>v.body===g.body&&v.group===1))){
      const texture=materialDef?.texture?textures.get(new URL(materialDef.texture,url).href):null;
      if(texture&&g.type!==7)texture.repeat.set(...materialDef.repeat);
      const mat=new THREE.MeshStandardMaterial({color,map:texture,roughness:Math.max(.3,1-(materialDef?.shininess||0)*.65),metalness:(g.name.includes('metal')||g.name.includes('aluminium')) ? .65 : .05,transparent:g.rgba[3]<.99,opacity:g.rgba[3],side:THREE.DoubleSide,depthWrite:g.rgba[3]>.5});
      const mesh=new THREE.Mesh(geometry,mat);mesh.position.fromArray(g.pos);mesh.quaternion.set(g.quat[1],g.quat[2],g.quat[3],g.quat[0]);mesh.castShadow=true;mesh.receiveShadow=true;mesh.userData.joint=nearestJoint(g.body);mesh.name=g.name;
      bodyGroups.get(g.body).add(mesh);pickables.push(mesh);if(g.cutaway)cutaways.push(mesh);
    }
    if(g.collision){const cm=new THREE.Mesh(geometry,new THREE.MeshBasicMaterial({color:0x69957e,wireframe:true,transparent:true,opacity:.35,depthWrite:false}));cm.position.fromArray(g.pos);cm.quaternion.set(g.quat[1],g.quat[2],g.quat[3],g.quat[0]);cm.visible=showCollision;bodyGroups.get(g.body).add(cm);colliders.push(cm);}
  }
  applyCouplings();updateBodies();applyCutaway();frameModel();renderJoints();
  $('loading').hidden=true;$('viewport').dataset.ready='true';
}
function updateBodies(){
  if(!activeModel)return;
  for(const b of activeModel.bodies){
    const group=bodyGroups.get(b.id),base=new THREE.Matrix4().compose(new THREE.Vector3(...b.pos),new THREE.Quaternion(b.quat[1],b.quat[2],b.quat[3],b.quat[0]),new THREE.Vector3(1,1,1));
    for(const jname of b.joints){const j=jointMap.get(jname),p=j.pivot,q=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...j.axis),angles.get(jname)-j.reference);base.multiply(new THREE.Matrix4().makeTranslation(...p)).multiply(new THREE.Matrix4().makeRotationFromQuaternion(q)).multiply(new THREE.Matrix4().makeTranslation(-p[0],-p[1],-p[2]));}
    group.matrix.copy(base);group.matrixWorldNeedsUpdate=true;
  }
  modelRoot.updateMatrixWorld(true);
}
function frameModel(){
  if(!activeModel)return;
  const [lo,hi]=activeModel.bounds,center=new THREE.Vector3(...lo).add(new THREE.Vector3(...hi)).multiplyScalar(.5),size=new THREE.Vector3(...hi).sub(new THREE.Vector3(...lo)),extent=Math.max(...size.toArray(),.05);
  const dir=new THREE.Vector3(.9,-1.8,selected.kind==='scene'?2.1:.85).normalize();camera.position.copy(center).addScaledVector(dir,extent*1.85);controls.target.copy(center);camera.near=extent/1000;camera.far=extent*100;camera.updateProjectionMatrix();controls.minDistance=extent*.18;controls.maxDistance=extent*8;controls.update();
  keyLight.position.copy(center).add(new THREE.Vector3(extent,-extent,extent*2));keyLight.target.position.copy(center);fill.position.copy(center).add(new THREE.Vector3(-extent,extent,extent));fill.target.position.copy(center);
  Object.assign(keyLight.shadow.camera,{left:-extent,right:extent,top:extent,bottom:-extent,near:.001,far:extent*8});keyLight.shadow.camera.updateProjectionMatrix();keyLight.shadow.normalBias=extent*.001;
  if(!ground&&selected.kind!=='scene'){
    ground=new THREE.Mesh(new THREE.PlaneGeometry(extent*30,extent*30),new THREE.MeshStandardMaterial({color:0xeeeee7,roughness:1,metalness:0}));ground.position.set(center.x,center.y,lo[2]-.002*extent);ground.receiveShadow=true;scene.add(ground);
    grid=new THREE.GridHelper(extent*8,32,0xd5dacd,0xe1e4d9);grid.rotation.x=Math.PI/2;grid.position.set(center.x,center.y,lo[2]-.001*extent);grid.material.transparent=true;grid.material.opacity=.55;scene.add(grid);
  }
}
function applyCutaway(){cutaways.forEach(m=>m.visible=!(selected?.kind==='scene'&&cutaway));$('cutaway-toggle').setAttribute('aria-pressed',cutaway);}
function applyCouplings(){
  for(const c of activeModel?.couplings||[]){const driver=jointMap.get(c.driver),dependent=jointMap.get(c.joint);if(!driver||!dependent)continue;const x=angles.get(c.driver)-driver.reference;angles.set(c.joint,dependent.reference+c.polycoef.reduce((sum,k,i)=>sum+k*x**i,0));}
}
function setJoint(jname,value){const j=jointMap.get(jname);if(!j)return;angles.set(jname,THREE.MathUtils.clamp(value,...(j.control_range||j.range)));applyCouplings();updateBodies();updateJointUI();}
function updateJointUI(){for(const [name,value] of angles){const row=document.querySelector(`[data-joint="${CSS.escape(name)}"]`);if(row){row.querySelector('input').value=deg(value);row.querySelector('output').textContent=`${fmt(deg(value),1)}°`;row.classList.toggle('active',drag?.name===name);}}}
function renderJoints(){
  $('joint-panel').hidden=!selected.joints.length;$('joint-count').textContent=isRobot(selected)?`${independentDof(selected)} 个独立 / ${selected.joints.length} 个转动关节`:`${selected.joints.length} 个`;
  const group=$('joint-group').value;
  const controls=selected.joints.filter(j=>!j.coupling&&(!isRobot(selected)||group==='all'||(group==='left'?j.name.startsWith('left_'):group==='right'?j.name.startsWith('right_'):!j.name.startsWith('left_')&&!j.name.startsWith('right_'))));
  $('joint-controls').innerHTML=controls.map(j=>`<div class="joint-row" data-joint="${esc(j.name)}"><label for="joint-${esc(j.name)}">${esc(label(j.name))}</label><input id="joint-${esc(j.name)}" type="range" aria-label="${esc(label(j.name))}角度" min="${deg((j.control_range||j.range)[0])}" max="${deg((j.control_range||j.range)[1])}" step="0.1" value="${deg(angles.get(j.name))}"><output>${fmt(deg(angles.get(j.name)),1)}°</output></div>`).join('');
  document.querySelectorAll('.joint-row input').forEach(input=>input.oninput=()=>setJoint(input.closest('.joint-row').dataset.joint,rad(Number(input.value))));
}
function setMode(enabled){jointMode=enabled&&!!selected?.joints.length;$('joint-mode').classList.toggle('active',jointMode);$('orbit-mode').classList.toggle('active',!jointMode);$('joint-mode').setAttribute('aria-pressed',jointMode);$('orbit-mode').setAttribute('aria-pressed',!jointMode);$('interaction-hint').textContent=jointMode?'拖动门 / 提手调整 · 拖动空白旋转':'拖动旋转 · 滚轮缩放 · 右键平移';}
async function selectAsset(slug){
  const a=catalog.assets.find(x=>x.slug===slug);if(!a)return;selected=a;const token=++loadId;setCategory(a.kind==='scene'?'scene':'object');
  const u=new URL(location.href);u.searchParams.set('asset',slug);history.replaceState(null,'',u);
  $('asset-name').textContent=a.name;$('asset-subtitle').textContent=isRobot(a)?`FQ 适配 / ${independentDof(a)} 独立自由度 · ${a.joints.length} 个模型转动关节${a.family==='o7'?' / O7 v1':''}`:a.subtitle;$('asset-version').textContent=`main · v${a.version}`;$('breadcrumb').textContent=`资产库 / ${a.kind==='scene'?'场景':'物体'} / ${a.kind_label}`;
  $('commit-label').textContent=`main · ${catalog.main_commit.slice(0,8)}`;$('pose-label').textContent=a.joints.length?'关节可动':a.kind==='scene'?'固定基准':'静态姿态';
  $('mode-toggle').hidden=!a.joints.length||isRobot(a);$('joint-group').hidden=!isRobot(a);$('joint-group').value=isRobot(a)?(a.handedness==='right'?'right':'left'):'all';document.querySelector('.joint-note').textContent=isRobot(a)?'滑块控制独立关节；从动手指按模型关系联动。仅姿态预览，不运行机器人控制。':'在模型上拖动门 / 提手，或使用滑块。角度受模型限位约束，操作不会写回资产库。';$('cutaway-toggle').hidden=a.kind!=='scene';$('joint-panel').hidden=true;$('scene-info').hidden=a.kind!=='scene';
  if(a.kind==='scene'){$('scene-info').innerHTML='<strong>固定基准 · 办公场地</strong><p>测试区、走道、圆桌与会议室。门净宽约 0.80 m；会议桌宽 1.20 m。</p><p>保留禁入区碰撞体。篮子与冰箱暂未指定正式摆放位置。</p><div class="scene-chips">'+(a.dependencies||[]).map(id=>{const dep=catalog.assets.find(x=>x.id===id);return dep?`<button data-asset="${esc(dep.slug)}">${esc(dep.name)} · v${esc(dep.version)} ↗</button>`:'';}).join('')+'</div>';document.querySelectorAll('.scene-chips button').forEach(b=>b.onclick=()=>selectAsset(b.dataset.asset));}
  $('material-controls').hidden=a.kind!=='material';$('collision-toggle').hidden=a.kind==='material';$('material-repeat').value=1;$('material-repeat-value').textContent='1 ×';
  showProperties(a);renderList();setMode(!!a.joints.length&&!isRobot(a));$('viewport').dataset.ready='false';$('loading').hidden=false;$('loading').querySelector('strong').textContent=`正在加载${a.name}`;$('viewer-error').hidden=true;
  try{const url=new URL(a.model_url,location.origin);const res=await fetch(url);if(!res.ok)throw Error('模型文件暂不可用');const data=await res.json();if(token!==loadId)return;await buildModel(data,url,token);}
  catch(e){if(token!==loadId)return;$('loading').hidden=true;$('viewer-error').hidden=false;$('viewer-error').textContent=`模型加载失败：${e.message}。请刷新后重试。`;console.error(e);}
}

const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();
function ray(e){const r=renderer.domElement.getBoundingClientRect();pointer.set((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1);raycaster.setFromCamera(pointer,camera);}
renderer.domElement.addEventListener('pointerdown',e=>{
  if(!jointMode||e.button!==0||!modelRoot||$('viewport').dataset.ready!=='true')return;ray(e);const hit=raycaster.intersectObjects(pickables,false).find(h=>h.object.visible);if(!hit?.object.userData.joint)return;
  const name=hit.object.userData.joint,j=jointMap.get(name),group=bodyGroups.get(j.body),pivot=group.localToWorld(new THREE.Vector3(...j.pivot)),axis=new THREE.Vector3(...j.axis).transformDirection(group.matrixWorld),plane=new THREE.Plane().setFromNormalAndCoplanarPoint(axis,pivot),pt=raycaster.ray.intersectPlane(plane,new THREE.Vector3());
  drag={name,start:angles.get(name),x:e.clientX,y:e.clientY,plane,pivot,axis,vector:pt&&Math.abs(raycaster.ray.direction.dot(axis))>.08?pt.clone().sub(pivot).normalize():null};
  controls.enabled=false;renderer.domElement.setPointerCapture(e.pointerId);e.preventDefault();e.stopImmediatePropagation();updateJointUI();
},true);
renderer.domElement.addEventListener('pointermove',e=>{
  if(!drag)return;ray(e);let delta=(e.clientX-drag.x)*.01-(e.clientY-drag.y)*.004;
  if(drag.vector){const pt=raycaster.ray.intersectPlane(drag.plane,new THREE.Vector3());if(pt){const v=pt.sub(drag.pivot).normalize();delta=Math.atan2(drag.axis.dot(new THREE.Vector3().crossVectors(drag.vector,v)),drag.vector.dot(v));}}
  setJoint(drag.name,drag.start+delta);e.preventDefault();e.stopImmediatePropagation();
},true);
function endDrag(e){if(!drag)return;drag=null;controls.enabled=true;updateJointUI();if(renderer.domElement.hasPointerCapture(e.pointerId))renderer.domElement.releasePointerCapture(e.pointerId);e.stopImmediatePropagation();}
renderer.domElement.addEventListener('pointerup',endDrag,true);renderer.domElement.addEventListener('pointercancel',endDrag,true);
$('orbit-mode').onclick=()=>setMode(false);$('joint-mode').onclick=()=>setMode(true);
$('reset-view').onclick=frameModel;$('reset-joints').onclick=()=>{selected.joints.forEach(j=>angles.set(j.name,j.initial));applyCouplings();updateBodies();updateJointUI();};
$('collision-toggle').onclick=()=>{showCollision=!showCollision;colliders.forEach(m=>m.visible=showCollision);$('collision-toggle').setAttribute('aria-pressed',showCollision);};
$('cutaway-toggle').onclick=()=>{cutaway=!cutaway;applyCutaway();};
$('copy-link').onclick=async()=>{try{await navigator.clipboard.writeText(location.href);toast('资产链接已复制');}catch{const input=document.createElement('input');input.value=location.href;document.body.append(input);input.select();const ok=document.execCommand('copy');input.remove();toast(ok?'资产链接已复制':'请复制浏览器地址栏中的链接');}};
$('refresh').onclick=()=>{refresh();toast('正在获取最新同步结果');};
document.querySelectorAll('.tabs button').forEach(b=>b.onclick=()=>{setCategory(b.dataset.kind);const first=catalog?.assets.find(a=>category==='scene'?a.kind==='scene':a.kind!=='scene');if(first&&(selected?.kind==='scene')!==(category==='scene'))selectAsset(first.slug);});
$('joint-group').onchange=renderJoints;
$('search').oninput=renderList;$('origin-filter').onchange=renderList;$('type-filter').onchange=renderList;
$('material-repeat').oninput=()=>{const n=Number($('material-repeat').value);$('material-repeat-value').textContent=`${n} ×`;modelRoot?.traverse(o=>{if(o.material?.map)o.material.map.repeat.set(n,n);});};
await refresh();setInterval(refresh,20000);
