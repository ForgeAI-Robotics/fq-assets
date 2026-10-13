import * as THREE from 'three';
const $=id=>document.getElementById(id),degree=v=>v*180/Math.PI;
const names={d435i_head_rgb:'头部 · 彩色',d435i_head_depth:'头部 · 原生深度视角',d435i_head_depth_aligned_to_color:'头部 · 对齐彩色深度视角',d435i_left_rgb:'左腕 · 彩色',d435i_right_rgb:'右腕 · 彩色',d435i_left_depth:'左腕 · 深度视角',d435i_right_depth:'右腕 · 深度视角'};
const jointNames={refrigerator_left_hinge:'左门',refrigerator_right_hinge:'右门'};
const quaternion=q=>new THREE.Quaternion(q[1],q[2],q[3],q[0]);
const pose=(p,q)=>new THREE.Matrix4().compose(new THREE.Vector3(...p),quaternion(q),new THREE.Vector3(1,1,1));
function dispose(root){const geos=new Set(),mats=new Set(),textures=new Set();root.traverse(o=>{if(o.geometry)geos.add(o.geometry);for(const m of Array.isArray(o.material)?o.material:o.material?[o.material]:[]){mats.add(m);if(m.map)textures.add(m.map);}});geos.forEach(g=>g.dispose());mats.forEach(m=>m.dispose());textures.forEach(t=>t.dispose());root.removeFromParent();}
async function environment(asset,primitive){
 const url=new URL(asset.model_url,location.origin),res=await fetch(url);if(!res.ok)throw Error('基准场景暂不可用');const data=await res.json();
 const response=await fetch(new URL(data.buffer,url));if(!response.ok)throw Error('基准场景几何加载失败');const buffer=await response.arrayBuffer();
 const root=new THREE.Group(),groups=new Map(),angles=new Map(data.joints.map(j=>[j.name,j.initial])),joints=new Map(data.joints.map(j=>[j.name,j])),pickables=[],cutaways=[],geos=new Map(),textures=new Map();
 try{
  const loader=new THREE.TextureLoader();await Promise.all([...new Set(data.materials.filter(m=>m.texture).map(m=>m.texture))].map(async name=>{const t=await loader.loadAsync(new URL(name,url).href);t.flipY=false;t.colorSpace=THREE.SRGBColorSpace;t.wrapS=t.wrapT=THREE.RepeatWrapping;textures.set(name,t);}));
  for(const b of data.bodies){const g=new THREE.Group();g.name=b.name;g.matrixAutoUpdate=false;groups.set(b.id,g);(b.id===0?root:groups.get(b.parent)).add(g);}
  function jointOf(id){while(id>0){const b=data.bodies[id];if(b.joints.length)return b.joints[0];id=b.parent;}return null;}
  for(const g of data.geoms){
   if(g.rgba[3]<=.01||g.group===3)continue;
   let geo;if(g.type===7){geo=geos.get(g.mesh);if(!geo){const e=data.meshes[g.mesh],arr=new Float32Array(buffer,e.offset,e.count*8),b=new THREE.InterleavedBuffer(arr,8);geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.InterleavedBufferAttribute(b,3,0));geo.setAttribute('normal',new THREE.InterleavedBufferAttribute(b,3,3));geo.setAttribute('uv',new THREE.InterleavedBufferAttribute(b,2,6));geo.computeBoundingSphere();geos.set(g.mesh,geo);}}else geo=primitive(g);
   const md=data.materials[g.material],texture=textures.get(md?.texture);if(texture&&g.type!==7)texture.repeat.set(...md.repeat);
   const material=new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(...g.rgba.slice(0,3),THREE.SRGBColorSpace),map:texture||null,roughness:Math.max(.3,1-(md?.shininess||0)*.65),metalness:.05,side:THREE.DoubleSide,transparent:g.rgba[3]<.99,opacity:g.rgba[3],depthWrite:g.rgba[3]>.5});
   const mesh=new THREE.Mesh(geo,material);mesh.position.fromArray(g.pos);mesh.quaternion.copy(quaternion(g.quat));mesh.receiveShadow=true;mesh.userData.sceneJoint=jointOf(g.body);groups.get(g.body).add(mesh);pickables.push(mesh);if(g.cutaway)cutaways.push(mesh);
  }
  const update=()=>{for(const b of data.bodies){const mat=pose(b.pos,b.quat);for(const name of b.joints){const j=joints.get(name),p=j.pivot,q=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...j.axis),angles.get(name)-j.reference);mat.multiply(new THREE.Matrix4().makeTranslation(...p)).multiply(new THREE.Matrix4().makeRotationFromQuaternion(q)).multiply(new THREE.Matrix4().makeTranslation(-p[0],-p[1],-p[2]));}groups.get(b.id).matrix.copy(mat);groups.get(b.id).matrixWorldNeedsUpdate=true;}root.updateMatrixWorld(true);};update();
  return {root,groups,angles,joints,pickables,cutaways,update,data,dispose:()=>dispose(root)};
 }catch(e){dispose(root);textures.forEach(t=>t.dispose());throw e;}
}
export class SensingPreview{
 constructor(context){
  this.context=context;this.active=false;this.token=0;this.env=null;this.lastRender=0;this.sensorCamera=new THREE.PerspectiveCamera();this.sensorCamera.matrixAutoUpdate=false;this.raycaster=new THREE.Raycaster();
  $('sensor-view').onclick=()=>this.enter();$('model-view').onclick=()=>this.stop();$('sensor-preset').onchange=()=>this.preset();$('sensor-camera').onchange=()=>this.chooseCamera();$('sensor-range').oninput=()=>{$('sensor-range-label').textContent=`${Number($('sensor-range').value).toFixed(1)} m`;this.makeFrustum();};$('sensor-frustum').onchange=()=>{if(this.helper)this.helper.visible=$('sensor-frustum').checked;};$('sensor-cutaway').onchange=()=>this.cutaway();$('sensor-reset').onclick=()=>{this.context().resetJoints();this.env?.angles.forEach((_,n)=>this.env.angles.set(n,this.env.joints.get(n).initial));this.env?.update();this.preset();this.doorUI();};
  $('sensor-enlarge').onclick=()=>{const on=$('sensor-window').classList.toggle('enlarged');$('sensor-enlarge').textContent=on?'缩小 ↙':'放大 ↗';$('sensor-enlarge').setAttribute('aria-pressed',on);};
 }
 configure(){const c=this.context();$('sensing-tabs').hidden=!['g1_o6','g1_o7'].includes(c.selected?.slug);$('sensor-view').disabled=!c.activeModel?.cameras?.length;$('sensor-view').title=$('sensor-view').disabled?'等待服务同步相机定义':'';}
 async enter(){
  if(this.active)return;const c=this.context();if(!c.activeModel?.cameras?.length)return;const asset=c.catalog.assets.find(a=>a.id==='fq:scenes/office_baseline');if(!asset){c.toast('当前版本中没有基准场景');return;}
  const token=++this.token;this.active=true;$('sensor-view').classList.add('active');$('model-view').classList.remove('active');$('sensor-view').setAttribute('aria-pressed',true);$('model-view').setAttribute('aria-pressed',false);$('sensing-controls').hidden=false;$('sensor-status').textContent='正在加载 baseline，首次加载可能需要片刻…';$('viewport').classList.add('sensing-active');$('sensor-preset').value='table';
  try{
   const env=await environment(asset,c.primitive);if(token!==this.token){env.dispose();return;}this.env=env;c.scene.add(env.root);this.sceneAsset=asset;
   this.savedRoot={position:c.modelRoot.position.clone(),quaternion:c.modelRoot.quaternion.clone()};if(c.ground)c.ground.visible=false;if(c.grid)c.grid.visible=false;
   if(!this.renderer){this.renderer=new THREE.WebGLRenderer({antialias:true});this.renderer.setSize(640,480,false);this.renderer.outputColorSpace=THREE.SRGBColorSpace;this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=1.3;$('sensor-canvas').append(this.renderer.domElement);}
   $('sensor-camera').replaceChildren(...c.activeModel.cameras.map(s=>new Option(names[s.name]||s.name,s.name)));$('sensor-camera').value='d435i_head_rgb';$('sensor-window').hidden=false;$('sensor-details').hidden=false;
   c.setMode(false);$('mode-toggle').hidden=true;$('cutaway-toggle').hidden=true;$('model-note').textContent='临时预览站位 · 不写回 baseline · 不运行机器人控制';$('interaction-hint').textContent='拖动旋转 · 冰箱门可拖动 · 下方可调机器人关节';
   this.preset();this.chooseCamera();this.doorUI();this.cutaway();
  }catch(e){if(token!==this.token)return;this.stop();c.toast(`感测预览加载失败：${e.message}`);console.error(e);}
 }
 stop(reframe=true){
  ++this.token;const c=this.context(),was=this.active;this.active=false;this.drag=null;c.controls.enabled=true;
  if(this.savedRoot&&c.modelRoot){c.modelRoot.position.copy(this.savedRoot.position);c.modelRoot.quaternion.copy(this.savedRoot.quaternion);c.modelRoot.updateMatrixWorld(true);}this.savedRoot=null;
  this.env?.dispose();this.env=null;if(this.helper)dispose(this.helper);this.helper=null;
  if(c.ground)c.ground.visible=true;if(c.grid)c.grid.visible=true;
  for(const id of ['sensing-controls','sensor-window','sensor-details'])$(id).hidden=true;
  $('viewport').classList.remove('sensing-active');$('sensor-window').classList.remove('enlarged');$('sensor-enlarge').textContent='放大 ↗';$('sensor-enlarge').setAttribute('aria-pressed',false);$('model-view').classList.add('active');$('sensor-view').classList.remove('active');$('model-view').setAttribute('aria-pressed',true);$('sensor-view').setAttribute('aria-pressed',false);
  $('model-note').textContent='浏览器姿态预览，不运行接触物理仿真。';if(was&&reframe){$('pose-label').textContent=c.selected?.joints.length?'关节可动':'静态姿态';c.frameModel();c.setMode(false);}
 }
 preset(){
  if(!this.env)return;const c=this.context(),fridge=$('sensor-preset').value==='fridge',name=fridge?'refrigerator_base':'round_table',body=this.env.data.bodies.find(b=>b.name===name);if(!body)throw Error(`场景缺少 ${name}`);
  const target=this.env.groups.get(body.id).getWorldPosition(new THREE.Vector3());
  // Both presets stand south of the target and face +Y. Fridge stand-off
  // leaves space for a 90-degree door sweep; values are preview-only.
  const stand=fridge?1.6:.65;c.modelRoot.position.set(target.x,target.y-stand,-c.activeModel.bounds[0][2]+.002);c.modelRoot.quaternion.setFromAxisAngle(new THREE.Vector3(0,0,1),Math.PI/2);c.modelRoot.updateMatrixWorld(true);
  const center=new THREE.Vector3(target.x,target.y-stand*.45,.85);c.camera.position.copy(center).add(new THREE.Vector3(fridge?-1.8:1.9,-1.2,1.45));c.camera.near=.01;c.camera.far=100;c.camera.updateProjectionMatrix();c.controls.target.copy(center);c.controls.minDistance=.25;c.controls.maxDistance=18;c.controls.update();
  c.keyLight.position.copy(center).add(new THREE.Vector3(-2,-2,4));c.keyLight.target.position.copy(center);c.fill.position.copy(center).add(new THREE.Vector3(2,1,3));c.fill.target.position.copy(center);
  $('viewport').dataset.sensorPreset=fridge?'fridge':'table';$('sensor-door-controls').hidden=!fridge;$('pose-label').textContent=fridge?'感测 / 冰箱前':'感测 / 圆桌前';this.lastRender=0;
 }
 chooseCamera(){
  const c=this.context();this.sensor=c.activeModel.cameras.find(s=>s.name===$('sensor-camera').value);if(!this.sensor)return;const s=this.sensor,k=s.intrinsics;
  const n=.0001,f=50;this.sensorCamera.near=n;this.sensorCamera.far=f;this.sensorCamera.projectionMatrix.makePerspective(-k.cx/k.fx*n,(k.width-k.cx)/k.fx*n,k.cy/k.fy*n,-(k.height-k.cy)/k.fy*n,n,f);this.sensorCamera.projectionMatrixInverse.copy(this.sensorCamera.projectionMatrix).invert();
  $('sensor-title').textContent=names[s.name]||s.name;$('sensor-status').textContent=s.status+(s.name.includes('head_depth')&&!s.name.includes('aligned')?'；头壳边缘仍有少量遮挡。':s.name.includes('head')?'。':'；腕部光学通道尚未校验，展示网格可能遮挡。');
  const fovx=degree(Math.atan(k.cx/k.fx)+Math.atan((k.width-k.cx)/k.fx)),fovy=degree(Math.atan(k.cy/k.fy)+Math.atan((k.height-k.cy)/k.fy));
  $('sensor-parameters').replaceChildren();for(const text of [`${names[s.name]} · ${k.width} × ${k.height} · 水平 ${fovx.toFixed(1)}° / 垂直 ${fovy.toFixed(1)}°`,`fx ${k.fx.toFixed(3)} / fy ${k.fy.toFixed(3)} · cx ${k.cx.toFixed(3)} / cy ${k.cy.toFixed(3)}`,`安装参考：${s.parent} · ${s.position.map(v=>v.toFixed(4)).join(', ')} m`,`${s.status}。${s.note}。`,`机器人 v${c.selected.version} · baseline v${this.sceneAsset.version} · main ${c.catalog.main_commit.slice(0,8)}`,'视锥距离只用于显示，不代表 D435i 的有效测距范围；第一视角保留顶棚与正常遮挡。']){const p=document.createElement('p');p.textContent=text;$('sensor-parameters').append(p);}
  this.makeFrustum();this.lastRender=0;
 }
 makeFrustum(){
  if(!this.sensor)return;if(this.helper)dispose(this.helper);const k=this.sensor.intrinsics,far=Number($('sensor-range').value),points=[],corners=[[0,0],[k.width,0],[k.width,k.height],[0,k.height]].map(([u,v])=>new THREE.Vector3((u-k.cx)/k.fx*far,-(v-k.cy)/k.fy*far,-far));
  for(let i=0;i<4;i++)points.push(new THREE.Vector3(),corners[i],corners[i],corners[(i+1)%4]);points.push(new THREE.Vector3(),new THREE.Vector3(0,0,-far));
  this.helper=new THREE.Group();this.helper.matrixAutoUpdate=false;this.helper.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:0x11a88b,transparent:true,opacity:.85})));
  const marker=new THREE.Mesh(new THREE.SphereGeometry(.018,12,8),new THREE.MeshBasicMaterial({color:0xf5a442}));this.helper.add(marker);this.helper.visible=$('sensor-frustum').checked;this.context().scene.add(this.helper);
 }
 cutaway(){if(this.env)this.env.cutaways.forEach(m=>m.visible=!$('sensor-cutaway').checked);}
 doorUI(){
  $('sensor-door-controls').replaceChildren();if(!this.env)return;
  for(const [name,title] of Object.entries(jointNames)){const j=this.env.joints.get(name);if(!j)continue;const label=document.createElement('label');label.textContent=title;const input=document.createElement('input');input.type='range';input.min=degree(j.range[0]);input.max=degree(j.range[1]);input.step='.5';input.value=degree(this.env.angles.get(name));input.setAttribute('aria-label',`预览冰箱${title}角度`);input.dataset.sceneJoint=name;const output=document.createElement('output');output.textContent=`${Number(input.value).toFixed(0)}°`;input.oninput=()=>this.setDoor(name,Number(input.value)*Math.PI/180);label.append(input,output);$('sensor-door-controls').append(label);}
 }
 setDoor(name,value){const j=this.env.joints.get(name);this.env.angles.set(name,THREE.MathUtils.clamp(value,...j.range));this.env.update();const input=$('sensor-door-controls').querySelector(`[data-scene-joint="${name}"]`);if(input){input.value=degree(this.env.angles.get(name));input.nextSibling.textContent=`${Number(input.value).toFixed(0)}°`;}this.lastRender=0;}
 render(now){
  if(!this.active||!this.env||!this.sensor||!this.renderer)return;const c=this.context(),body=c.bodyGroups.get(this.sensor.body);if(!body)return;
  body.updateWorldMatrix(true,false);const world=body.matrixWorld.clone().multiply(pose(this.sensor.position,this.sensor.quaternion_wxyz));this.sensorCamera.matrix.copy(world);this.sensorCamera.updateMatrixWorld(true);this.helper.matrix.copy(world);this.helper.updateMatrixWorld(true);
  if(now-this.lastRender<100)return;this.lastRender=now;
  const helperVisible=this.helper.visible,collisionVisibility=c.colliders.map(m=>m.visible),cutawayVisibility=this.env.cutaways.map(m=>m.visible);this.helper.visible=false;c.colliders.forEach(m=>m.visible=false);this.env.cutaways.forEach(m=>m.visible=true);
  try{this.renderer.render(c.scene,this.sensorCamera);}finally{this.helper.visible=helperVisible;c.colliders.forEach((m,i)=>m.visible=collisionVisibility[i]);this.env.cutaways.forEach((m,i)=>m.visible=cutawayVisibility[i]);}
 }
 cast(e){const c=this.context(),r=c.canvas.getBoundingClientRect();this.raycaster.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),c.camera);}
 pointerDown(e){
  if(!this.active||!this.env||e.button!==0)return false;this.cast(e);const c=this.context(),hit=this.raycaster.intersectObjects([...this.env.pickables,...c.pickables],false).find(h=>h.object.visible);const name=hit?.object.userData.sceneJoint;if(!jointNames[name])return false;
  const j=this.env.joints.get(name),group=this.env.groups.get(j.body),pivot=group.localToWorld(new THREE.Vector3(...j.pivot)),axis=new THREE.Vector3(...j.axis).transformDirection(group.matrixWorld),plane=new THREE.Plane().setFromNormalAndCoplanarPoint(axis,pivot),p=this.raycaster.ray.intersectPlane(plane,new THREE.Vector3());this.drag={name,start:this.env.angles.get(name),x:e.clientX,y:e.clientY,pivot,axis,plane,vector:p&&Math.abs(this.raycaster.ray.direction.dot(axis))>.08?p.sub(pivot).normalize():null};c.controls.enabled=false;c.canvas.setPointerCapture(e.pointerId);e.preventDefault();e.stopImmediatePropagation();return true;
 }
 pointerMove(e){if(!this.drag)return false;this.cast(e);const d=this.drag;let delta=(e.clientX-d.x)*.01-(e.clientY-d.y)*.004;if(d.vector){const p=this.raycaster.ray.intersectPlane(d.plane,new THREE.Vector3());if(p){const v=p.sub(d.pivot).normalize();delta=Math.atan2(d.axis.dot(new THREE.Vector3().crossVectors(d.vector,v)),d.vector.dot(v));}}this.setDoor(d.name,d.start+delta);e.preventDefault();e.stopImmediatePropagation();return true;}
 pointerUp(e){if(!this.drag)return false;this.drag=null;const c=this.context();c.controls.enabled=true;if(c.canvas.hasPointerCapture(e.pointerId))c.canvas.releasePointerCapture(e.pointerId);e.stopImmediatePropagation();return true;}
}
