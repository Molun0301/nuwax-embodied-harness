
import * as THREE from '/static/js/three.module.min.js';
import {OrbitControls} from '/static/js/OrbitControls.js';
const $=s=>document.querySelector(s);let transferring=false,referenceBounds,revision=0;const views=[];
function createView(id){const el=$(id),renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));el.appendChild(renderer.domElement);const scene=new THREE.Scene();scene.background=new THREE.Color('#0b1424');const camera=new THREE.PerspectiveCamera(52,1,.03,200000);camera.up.set(0,0,1);const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;const view={el,renderer,scene,camera,controls,cloud:null,line:null,markers:[],bounds:null};views.push(view);controls.addEventListener('change',()=>{if(transferring||!$('#sync').checked)return;transferring=true;for(const other of views)if(other!==view){other.camera.position.copy(camera.position);other.camera.quaternion.copy(camera.quaternion);other.controls.target.copy(controls.target);other.controls.update()}transferring=false});return view}
const left=createView('#left'),right=createView('#right');
function resize(){for(const v of views){const w=v.el.clientWidth,h=v.el.clientHeight;v.renderer.setSize(w,h,false);v.camera.aspect=w/h;v.camera.updateProjectionMatrix()}}new ResizeObserver(resize).observe($('.stage'));resize();
function fit(box=referenceBounds,mode='full'){if(!box)return;const center=box.getCenter(new THREE.Vector3()),extent=box.getSize(new THREE.Vector3()),span=Math.max(extent.length(),6);transferring=true;for(const v of views){v.controls.target.copy(center);const distance=span/(2*Math.tan(v.camera.fov*Math.PI/360))*Math.max(1,1/v.camera.aspect)*1.1;const d=mode==='top'?new THREE.Vector3(0,-.001,1):mode==='side'?new THREE.Vector3(0,-1,.02):new THREE.Vector3(.12,-.9,.8).normalize();v.camera.far=Math.max(2000,distance+span*4);v.camera.updateProjectionMatrix();v.camera.position.copy(center).addScaledVector(d,distance);v.camera.lookAt(center);v.controls.update()}transferring=false}
function updateColors(v){if(!v.cloud)return;const byTime=$('#color').value==='time',duration=+$('#end').max;const color=new THREE.Color(),colors=v.cloud.geometry.attributes.color.array,pos=v.cloud.geometry.attributes.position.array,times=v.cloud.geometry.attributes.sourceTime.array;for(let i=0;i<times.length;i++){const f=byTime?Math.max(0,Math.min(1,times[i]/duration)):Math.max(0,Math.min(1,(pos[i*3+2]+.5)/4));color.setHSL(.62-f*.6,.68,.62);colors[i*3]=color.r;colors[i*3+1]=color.g;colors[i*3+2]=color.b}v.cloud.geometry.attributes.color.needsUpdate=true}
function clear(v){for(const o of [v.cloud,v.line,...v.markers])if(o){v.scene.remove(o);o.geometry.dispose();o.material.dispose()}v.markers=[]}
async function load(v,variant){const mark=(v.loadMark||0)+1;v.loadMark=mark;const [response,pathResponse]=await Promise.all([fetch(variant.point_file),fetch(variant.trajectory_file)]);if(!response.ok||!pathResponse.ok)throw Error('地图文件读取失败');const [buffer,path]=await Promise.all([response.arrayBuffer(),pathResponse.json()]);if(v.loadMark!==mark)return;const all=new Float32Array(buffer);if(all.length%4)throw Error('点云长度不合法');const n=all.length/4,pos=new Float32Array(n*3),time=new Float32Array(n);for(let i=0;i<n;i++){pos.set(all.subarray(i*4,i*4+3),i*3);time[i]=all[i*4+3]}clear(v);const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(pos,3));g.setAttribute('color',new THREE.BufferAttribute(new Float32Array(n*3),3));g.setAttribute('sourceTime',new THREE.BufferAttribute(time,1));g.computeBoundingBox();v.bounds=g.boundingBox.clone();
const material=new THREE.PointsMaterial({vertexColors:true,size:+$('#size').value,sizeAttenuation:false});v.timeUniform={value:+$('#end').value};material.onBeforeCompile=shader=>{shader.uniforms.maximumTime=v.timeUniform;shader.vertexShader='attribute float sourceTime; varying float pointTime;\n'+shader.vertexShader;shader.vertexShader=shader.vertexShader.replace('#include <begin_vertex>','#include <begin_vertex>\npointTime=sourceTime;');shader.fragmentShader='uniform float maximumTime; varying float pointTime;\n'+shader.fragmentShader;shader.fragmentShader=shader.fragmentShader.replace('#include <clipping_planes_fragment>','#include <clipping_planes_fragment>\nif(pointTime>maximumTime) discard;')};v.cloud=new THREE.Points(g,material);v.scene.add(v.cloud);updateColors(v);
v.pathTimes=path.map(p=>(p.timestamp_ns-variant.source_base_ns)/1e9);v.line=new THREE.Line(new THREE.BufferGeometry().setFromPoints(path.map(p=>new THREE.Vector3(...p.p))),new THREE.LineBasicMaterial({color:'#e79aef',depthTest:false}));v.line.renderOrder=1;v.line.visible=$('#trajectory').checked;v.scene.add(v.line);for(const [point,color]of (variant.hide_markers?[]:[[variant.start,'#58e6a1'],[variant.end,'#ffb06c']])){const m=new THREE.Mesh(new THREE.SphereGeometry(.22,12,10),new THREE.MeshBasicMaterial({color,depthTest:false}));m.position.set(...point);m.renderOrder=2;v.markers.push(m);v.scene.add(m)}v.el.querySelector('.caption p').textContent=variant.static_map?`已载入 ${n.toLocaleString('zh-CN')} 个体素点 · 最终保存地图`:`已载入 ${n.toLocaleString('zh-CN')} 个点 · ${variant.frame_count} 帧`;updateTime();if(v===left){referenceBounds=v.bounds.clone();fit()}else if(!referenceBounds){fit(v.bounds)}}
function updateTime(){const end=+$('#end').value;$('#time-value').textContent=end;for(const v of views){if(v.timeUniform)v.timeUniform.value=end;if(v.line){let count=v.pathTimes.findIndex(t=>t>end);v.line.geometry.setDrawRange(0,count<0?v.pathTimes.length:count)}if(v.markers[1])v.markers[1].visible=end>=(+$('#end').max-5)}}

let meta;
const metric = value => Number.isFinite(value.closure_xy_m) ? value.closure_xy_m.toFixed(3) + ' 米' : '不可用';
async function selectVersion() {
  const variant = meta.variants.find(v => v.id === $('#variant').value);
  const reference = meta.variants.find(v => v.id === $('#reference').value);
  if (!variant || !reference) throw Error('参考或候选结果不存在');
  const selection = ++revision;
  $('#error').textContent = '';
  const duration = Math.max(variant.duration_seconds || 1, reference.duration_seconds || 1);
  const finalMap = !!variant.static_map || !!reference.static_map;
  $('#end').disabled = finalMap;
  $('#color').querySelector('option[value=time]').disabled = finalMap;
  if (finalMap) $('#color').value = 'height';
  $('#end').max = duration; $('#end').value = duration; $('#time-max').textContent = duration;
  $('#left-title').textContent = reference.title; $('#right-title').textContent = variant.title;
  $('#left .overlay').textContent = reference.cloud_source_note || '导入点云';
  $('#right .overlay').textContent = variant.cloud_source_note || '导入点云';
  $('header>small').textContent = '左：' + reference.title + ' · 右：' + variant.title;
  $('.source').textContent = reference.dataset_label === variant.dataset_label
    ? variant.dataset_label : '来源说明：' + reference.dataset_label + ' / ' + variant.dataset_label;
  $('#reference-metric-label').textContent = '参考首末水平距离';
  $('#candidate-metric-label').textContent = '候选首末水平距离';
  $('#fast-metric').textContent = metric(reference); $('#point-metric').textContent = metric(variant);
  $('#metric-note').textContent = variant.metric_note;
  $('#badge').textContent = variant.status || '待对比';
  $('#change').textContent = variant.change || '';
  $('#location').textContent = variant.location || '';
  $('#identity').textContent = `点数 ${variant.point_count} · 轨迹 ${variant.frame_count} 帧 · 高度跨度 ${Number.isFinite(variant.height_span_m) ? variant.height_span_m.toFixed(3) + ' 米' : '不可用'}`;
  $('#assessment').textContent = variant.assessment || '';
  history.replaceState(null, '', '?version=' + encodeURIComponent(variant.id) + '&reference=' + encodeURIComponent(reference.id));
  await Promise.all([load(left, reference), load(right, variant)]);
  if (selection === revision) updateTime();
}
const refresh = () => selectVersion().catch(error => { $('#error').textContent = error.message; });
$('#fit').onclick = () => fit();
$('#candidate-fit').onclick = () => fit(right.bounds);
$('#top').onclick = () => fit(referenceBounds, 'top');
$('#side').onclick = () => fit(referenceBounds, 'side');
$('#end').oninput = updateTime;
$('#size').oninput = () => { for (const view of views) if (view.cloud) view.cloud.material.size = +$('#size').value; };
$('#trajectory').onchange = () => { for (const view of views) if (view.line) view.line.visible = $('#trajectory').checked; };
$('#color').onchange = () => views.forEach(updateColors);
function animate() { requestAnimationFrame(animate); for (const view of views) { view.controls.update(); view.renderer.render(view.scene, view.camera); } }
animate();
try {
  const response = await fetch('版本清单.json', { cache: 'no-store' });
  if (!response.ok) throw Error('版本清单读取失败');
  meta = await response.json();
  if (!meta.variants.length) throw Error('请先使用lio add或lio import导入对比结果');
  $('#variant').replaceChildren(); $('#reference').replaceChildren();
  for (const variant of meta.variants) {
    $('#variant').add(new Option(variant.title, variant.id));
    $('#reference').add(new Option(variant.title, variant.id));
  }
  const query = new URLSearchParams(location.search);
  const valid = value => meta.variants.some(variant => variant.id === value);
  $('#variant').value = valid(query.get('version')) ? query.get('version') : meta.selected;
  const candidate = meta.variants.find(v => v.id === $('#variant').value) || meta.variants[0];
  $('#variant').value = candidate.id;
  $('#reference').value = valid(query.get('reference')) ? query.get('reference') : candidate.reference_id || meta.reference;
  $('#variant').onchange = () => {
    const next = meta.variants.find(v => v.id === $('#variant').value);
    if (valid(next.reference_id)) $('#reference').value = next.reference_id;
    refresh();
  };
  $('#reference').onchange = refresh;
  await selectVersion();
} catch (error) { $('#error').textContent = error.message; console.error(error); }
