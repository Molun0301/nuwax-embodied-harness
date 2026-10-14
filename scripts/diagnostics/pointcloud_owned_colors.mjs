import fs from 'node:fs';import path from 'node:path';import {fileURLToPath} from 'node:url';import assert from 'node:assert/strict';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..'),js=path.join(root,'gateways/http/static/js');
const url=s=>`data:text/javascript;base64,${Buffer.from(s).toString('base64')}`;
const THREE=await import(url(fs.readFileSync(path.join(js,'three.module.min.js'),'utf8')));
const {PointCloudProcessor}=await import(url(fs.readFileSync(path.join(js,'pointcloud-processing.js'),'utf8')));
const finalSource=fs.readFileSync(path.join(js,'pointcloud3d.js'),'utf8'),old='geo.setAttribute(\'color\', new THREE.Uint8BufferAttribute(',replacement='geo.setAttribute(\'color\', new THREE.BufferAttribute(';
const ownershipComment='    // 颜色缓冲由预处理结果一次性交给几何；私有调用方不再复用或修改。\n';
assert.equal(finalSource.split(replacement).length,2);assert.equal(finalSource.split(ownershipComment).length,2);
const source=finalSource.replace(ownershipComment,'').replace(replacement,old);
assert.equal(source.replace('    '+old,ownershipComment+'    '+replacement),finalSource);
const workerSource=fs.readFileSync(path.join(js,'pointcloud-worker.js'),'utf8').replace(/^import .*;\n/gm,'');
const viewerSource=source.slice(0,source.indexOf('window.pointcloud3d = new PointCloud3DViewer();')).replace(/^import .*;\n/gm,'').replaceAll('export function ','function ').replaceAll('import.meta.url',"'file:///fixture/pointcloud3d.js'");
const classes=[viewerSource,viewerSource.replace(old,replacement)].map(s=>Function('THREE','PointCloudProcessor',s+'\nreturn PointCloud3DViewer;')(THREE,PointCloudProcessor));
const originalWindow=globalThis.window,originalWorker=globalThis.Worker;let events=[];
globalThis.window={dispatchEvent:e=>{events.push(e);},devicePixelRatio:1};
class Worker {constructor(){this.sent=[];}postMessage(data){this.sent.push(structuredClone(data));}terminate(){this.terminated=true;}}
globalThis.Worker=Worker;
let checks=0,unsupported=[];
function points(count){const p=new Float32Array(count*3);for(let i=0;i<count;i++){p[i*3]=i%97;p[i*3+1]=Math.floor(i/97)%73;p[i*3+2]=(i%11)*.25;}return p;}
function viewer(arm,input,style='height'){const v=new classes[arm]();v._scene=new THREE.Scene();v._allPositions=input;v._pointStyle=style;v._hasFramedPointCloud=true;v._cloudSourceKey='fixture';return v;}
function job(v,id=1){return {id,epoch:v._cloudEpoch,positions:v._allPositions,revision:String(id),mode:'fixture',options:{style:v._pointStyle,lod:v._lod,live:false,robotZ:undefined}};}
function publicState(v){const geo=v._pointsMesh?.geometry,color=geo?.getAttribute('color');return {positions:geo?.getAttribute('position')?.array,colors:color?.array,itemSize:color?.itemSize,count:color?.count,normalized:color?.normalized,usage:color?.usage,gpuType:color?.gpuType,version:color?.version,bounds:v._pointCloudBounds,displayed:v._displayedPointCount,activeFloor:v._activeFloorZ,renderedRevision:v._renderedCloudMapRevision,visible:v._pointsMesh?.visible};}
function prepare(v){return v._preparePointCloud();}
function workerResult(v,j){const self={postMessage(value,buffers){this.original=value;this.delivered=structuredClone(value,{transfer:buffers});}};Function('PointCloudProcessor','self',workerSource)(PointCloudProcessor,self);const input=structuredClone({id:j.id,config:v._processingConfig,options:j.options,positions:j.positions});self.onmessage({data:input});assert.equal(self.original.result.colors.byteLength,0);assert.equal(self.original.result.positions.byteLength,0);return self.delivered;}
function contains(root,target,seen=new Set()){if(root===target)return true;if(!root||typeof root!=='object'||seen.has(root))return false;seen.add(root);return Object.values(root).some(value=>contains(value,target,seen));}
function contracts(){
 for(const count of [0,1,16,4096])for(const style of ['height','mono','ground','structure']){
  const input=points(count),vs=[0,1].map(a=>viewer(a,input,style)),results=vs.map(prepare);vs.forEach((v,a)=>v._applyPreparedPointCloud(job(v),results[a]));assert.deepEqual(publicState(vs[0]),publicState(vs[1]));
  if(count&&results[0].colors.length){const attrs=vs.map(v=>v._pointsMesh.geometry.getAttribute('color'));assert.notEqual(attrs[0].array,results[0].colors);assert.equal(attrs[1].array,results[1].colors);assert(attrs.every(a=>a.array instanceof Uint8Array));assert(attrs[0] instanceof THREE.Uint8BufferAttribute);assert(!(attrs[1] instanceof THREE.Uint8BufferAttribute));assert(attrs[1] instanceof THREE.BufferAttribute);assert.deepEqual(attrs[0].toJSON(),attrs[1].toJSON());for(const a of attrs){const clone=a.clone();assert.notEqual(clone.array,a.array);assert.deepEqual(clone.toJSON(),a.toJSON());}}
  vs.forEach(v=>v.dispose());checks++;
 }
 for(const arm of [0,1]){
  const input=points(16),v=viewer(arm,input),j=job(v),worker=v._ensurePointCloudWorker();v._cloudRunning=j;
  const data=workerResult(v,j),colors=data.result.colors,originalColors=colors.slice();worker.onmessage({data});assert.deepEqual(v._pointsMesh.geometry.getAttribute('color').array,originalColors);assert.equal(v._cloudRunning,null);assert(!contains(v,data.result));assert(!contains(events,data.result));assert(!contains(events,colors));assert(!contains(j,colors));
  // Main source remains independent of transferred worker buffers; future preprocessing is fresh.
  input.fill(-99);assert.deepEqual(v._pointsMesh.geometry.getAttribute('color').array,originalColors);const next=prepare(v);assert.notEqual(next.colors,colors);assert.notEqual(next.colors.buffer,colors.buffer);checks++;
  const geometry=v._pointsMesh.geometry;let disposed=0;geometry.addEventListener('dispose',()=>disposed++);
  // Stale result is fenced before geometry takes ownership.
  const stale=workerResult(v,{...j,id:2});v._cloudRunning={...j,id:2,epoch:v._cloudEpoch-1};worker.onmessage({data:stale});stale.result.colors.fill(0);assert.equal(v._pointsMesh.geometry,geometry);assert.deepEqual(geometry.getAttribute('color').array,originalColors);checks++;
  // A duplicate response with no in-flight job is rejected, not installed a second time.
  worker.onmessage({data});assert.equal(v._pointsMesh.geometry,geometry);assert.equal(v._cloudWorkerFailed,true);checks++;
  v._applyPreparedPointCloud(job(v,3),next);assert.equal(disposed,1);const latest=v._pointsMesh.geometry;let latestDisposed=0;latest.addEventListener('dispose',()=>latestDisposed++);v.dispose();v.dispose();assert.equal(latestDisposed,1);assert.equal(disposed,1);checks++;
 }
 // Show explicitly why the candidate requires private ownership handoff, not arbitrary borrowed arrays.
 for(const arm of [0,1]){const input=points(4),v=viewer(arm,input),result=prepare(v);v._applyPreparedPointCloud(job(v),result);const before=v._pointsMesh.geometry.getAttribute('color').array[0];result.colors[0]=before^255;const observed=v._pointsMesh.geometry.getAttribute('color').array[0];unsupported.push({arm,case:'caller_mutates_retained_result_colors',geometry_changes:observed!==before});assert.equal(observed!==before,arm===1);v.dispose();}
 for(const arm of [0,1]){const input=points(4),a=viewer(arm,input),b=viewer(arm,input),result=prepare(a);a._applyPreparedPointCloud(job(a),result);b._applyPreparedPointCloud(job(b),result);const ac=a._pointsMesh.geometry.getAttribute('color').array,bc=b._pointsMesh.geometry.getAttribute('color').array;unsupported.push({arm,case:'one_result_shared_between_viewers',shares_color_buffer:ac.buffer===bc.buffer});assert.equal(ac.buffer===bc.buffer,arm===1);a.dispose();b.dispose();}
 // Synchronous ordinary producer colors are separate from source/positions buffers.
 for(const arm of [0,1])for(const style of ['height','mono','ground']){const input=points(128),v=viewer(arm,input,style),r=prepare(v);assert.notEqual(r.colors.buffer,input.buffer);assert.notEqual(r.colors.buffer,r.positions.buffer);const original=r.colors.slice();v._applyPreparedPointCloud(job(v),r);input.fill(13);assert.deepEqual(v._pointsMesh.geometry.getAttribute('color').array,original);v.dispose();checks++;}
}
function median(values,q=.5){return [...values].sort((a,b)=>a-b)[Math.floor((values.length-1)*q)];}
function benchmark(){const cohorts=[];
 for(const count of [0,4096,100000,1000000])for(const mode of ['initial','replacement']){
  const input=points(count),samples=[[],[]],vs=mode==='replacement'?[0,1].map(a=>viewer(a,input)):null;
  if(vs)vs.forEach(v=>v._applyPreparedPointCloud(job(v),prepare(v)));
  const operation=(arm,record)=>{const v=vs?vs[arm]:viewer(arm,input),j=job(v),result=prepare(v);events=[];const start=process.hrtime.bigint();v._applyPreparedPointCloud(j,result);const elapsed=Number(process.hrtime.bigint()-start)/1e6;if(record)samples[arm].push(elapsed);if(!vs)v.dispose();};
  for(let i=0;i<10;i++)for(const arm of [0,1])operation(arm,false);for(let i=0;i<30;i++)for(const arm of (i%2?[1,0]:[0,1]))operation(arm,true);
  vs?.forEach(v=>v.dispose());cohorts.push({points:count,mode,color_bytes:count*3,median_ms:samples.map(s=>median(s)),p95_ms:samples.map(s=>median(s,.95)),samples_ms:samples});
 }
 return cohorts;}
try{contracts();const output={checks,node:process.version,source_window:'Final production candidate; one attribute constructor and ownership comment reversed for baseline',scope:'Actual private viewer prepare/apply/render methods, actual bundled Three attributes and Node structuredClone transfer; no browser worker or GPU',unsupported_shared_input_differences:unsupported,cohorts:process.argv.includes('--contracts-only')?[]:benchmark()};console.log(JSON.stringify(output,null,2));}finally{globalThis.window=originalWindow;globalThis.Worker=originalWorker;}
