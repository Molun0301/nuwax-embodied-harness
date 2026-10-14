import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..'),js=path.join(root,'gateways/http/static/js');
const source=fs.readFileSync(path.join(js,'pointcloud-pages.js'),'utf8');
const before='if (![x, y, z].every(Number.isFinite))';
const after='if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z))';
assert.equal(source.split(before).length,2);
const url=text=>`data:text/javascript;base64,${Buffer.from(text).toString('base64')}`;
const three=url(fs.readFileSync(path.join(js,'three.module.min.js'),'utf8'));
const classes=[];
for(const text of [source,source.replace(before,after)])classes.push((await import(url(text.replace("'./three.module.min.js'",JSON.stringify(three))))).IncrementalPointCloudLayer);
const PAGE=1024;
function points(count){const p=new Float32Array(count*3);for(let i=0;i<count;i++){p[i*3]=(i%511)*.07;p[i*3+1]=(Math.floor(i/511)%257)*.07;p[i*3+2]=(i%10)<6?0:(i%10)<9?2.8:(i%17)*.125;}return p;}
function tx(data,ids=null,reset=true,count=data.length/3){return {header:{point_count:count,chunk_size_points:PAGE,reset},chunks:(ids||Array.from({length:Math.ceil(count/PAGE)},(_,i)=>i)).map(i=>({chunk_index:i,xyz:data.subarray(i*PAGE*3,Math.min(data.length,(i+1)*PAGE*3))}))};}
function snapshot(layer){return {pointCount:layer.pointCount,displayed:layer.displayedPointCount,sourceMeasurement:layer.sourceMeasurement,measurement:layer.measurement,floor:layer.floorContext,revision:layer._revision,source:layer.pages.map(p=>p.array.slice(0,p.count*3)),metadata:layer.pages.map(p=>p.metadata),derived:layer.derivedPages.map(p=>p.array.slice(0,p.count*3)),stats:JSON.parse(JSON.stringify(layer.stats)),sourceSlabs:layer.sourceSlabs.length,derivedPages:layer.derivedPages.length};}
let checks=0;
function contracts(){
 for(const style of ['height','mono','ground','singleFloor','structure'])for(const count of [0,1,1023,1024,1025,4096]){
  const data=points(count),initial=data.slice(),layers=classes.map(C=>new C({style}));
  const result=layers.map(l=>l.apply(tx(data)));assert.deepEqual(result[0],result[1]);assert.deepEqual(snapshot(layers[0]),snapshot(layers[1]));assert.deepEqual(data,initial);
  data.fill(99);assert.deepEqual(snapshot(layers[0]),snapshot(layers[1]));if(count)assert.notEqual(layers[0].pages[0].array[0],99);layers.forEach(l=>{l.dispose();l.dispose();});checks++;
 }
 // Same finite predicate, order and stopping point within the first invalid point.
 for(const withVoxels of [false,true])for(const invalid of [NaN,Infinity,-Infinity])for(const index of [0,1,2,3,4,5]){
  const data=points(2);data[index]=invalid;const observations=[];
  for(const C of classes){const layer=new C(),seen=[],original=Number.isFinite;let error;Number.isFinite=value=>{seen.push(value);return original(value);};try{layer._metadata(data,2,withVoxels);}catch(e){error=e.message;}finally{Number.isFinite=original;layer.dispose();}observations.push({seen,error});}
  assert.deepEqual(observations[0],observations[1]);assert.equal(observations[0].error,'Canonical pointcloud contains non-finite XYZ');assert.equal(observations[0].seen.length,index+1);checks++;
 }
 for(const count of [0,1,2,3])for(const length of [0,1,2,3,5,6,9]){const outcomes=classes.map(C=>{const layer=new C();try{return {value:layer._metadata(new Float32Array(length),count,false)};}catch(e){return {error:e.message};}finally{layer.dispose();}});assert.deepEqual(outcomes[0],outcomes[1]);checks++;}
 for(const invalidIndex of [0,PAGE*3+1,(PAGE*2+2)*3-1])for(const invalid of [NaN,Infinity,-Infinity]){
  const input=points(PAGE*2+2),layers=classes.map(C=>new C());layers.forEach(l=>l.apply(tx(input)));const old=layers.map(snapshot);const changed=input.slice();changed[invalidIndex]=invalid;
  for(let a=0;a<2;a++){assert.throws(()=>layers[a].apply(tx(changed,null,false)),/non-finite XYZ/);assert.deepEqual(snapshot(layers[a]),old[a]);layers[a].dispose();}checks++;
 }
 // Existing page-length validation rejects partial arrays before any visible state is mutated.
 for(const length of [1,2,4,5])for(const C of classes){const l=new C();l.apply(tx(points(1)));const old=snapshot(l);assert.throws(()=>l.apply({header:{point_count:1,chunk_size_points:PAGE,reset:false},chunks:[{chunk_index:0,xyz:new Float32Array(length)}]}),/length/);assert.deepEqual(snapshot(l),old);l.dispose();checks++;}
 // Real prepare/apply and count-only tail truncation preserve copied/scanned coordinates and bounds.
 const data=points(PAGE*3+17),layers=classes.map(C=>new C());layers.forEach(l=>l.apply(tx(data)));data[PAGE*3+1]=-50;
 for(const l of layers){const transaction=tx(data,[1],false),stage=l.prepare(transaction);l.apply(transaction,stage);}
 assert.deepEqual(snapshot(layers[0]),snapshot(layers[1]));checks++;
 const short=tx(data,[],false,PAGE*2+5);layers.forEach(l=>l.apply(short));assert.deepEqual(snapshot(layers[0]),snapshot(layers[1]));checks++;layers.forEach(l=>l.dispose());
}
function median(a,q=.5){return [...a].sort((a,b)=>a-b)[Math.floor((a.length-1)*q)];}
contracts();const output={node:process.version,checks,production_modified:false,scope:'Real IncrementalPointCloudLayer prepare/apply using actual Three objects, no browser or GPU; no allocation counter claim',cohorts:[]};
if(!process.argv.includes('--contracts-only')){
 const cohorts=[{count:1,style:'height',mode:'reset'},{count:1024,style:'height',mode:'reset'},{count:100000,style:'height',mode:'reset'},{count:1000000,style:'height',mode:'reset'},
 {count:100000,style:'height',mode:'steady',pages:1},{count:100000,style:'height',mode:'steady',pages:32},{count:8192,style:'ground',mode:'reset'},{count:8192,style:'structure',mode:'steady',pages:1}];
 for(const c of cohorts){const data=points(c.count),transaction=tx(data),steady=c.mode==='steady'?classes.map(C=>{const l=new C({style:c.style});l.apply(transaction);return l;}):null;
  const changes=[data.slice(),data.slice()];changes[1][0]=-.125;const deltas=changes.map(x=>tx(x,Array.from({length:c.pages||1},(_,i)=>i),false));const samples=[[],[]];
  const run=(arm,iteration,measure)=>{const layer=steady?steady[arm]:new classes[arm]({style:c.style}),transaction=steady?deltas[iteration%2]:tx(data);const start=process.hrtime.bigint();layer.apply(transaction);const ms=Number(process.hrtime.bigint()-start)/1e6;if(measure)samples[arm].push(ms);if(!steady)layer.dispose();};
  for(let i=0;i<8;i++)for(const arm of [0,1])run(arm,i,false);for(let i=0;i<30;i++)for(const arm of (i%2?[1,0]:[0,1]))run(arm,i,true);
  const statistics=steady?steady.map(l=>l.stats.lastApply):null;if(steady){assert.deepEqual(snapshot(steady[0]),snapshot(steady[1]));steady.forEach(l=>l.dispose());}
  output.cohorts.push({...c,median_ms:samples.map(s=>median(s)),p95_ms:samples.map(s=>median(s,.95)),samples_ms:samples,steady_last_stats:statistics});
 }
}
console.log(JSON.stringify(output,null,2));
