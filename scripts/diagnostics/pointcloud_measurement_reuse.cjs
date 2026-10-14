'use strict';
const fs=require('fs'),assert=require('assert'),path=require('path');
const root=path.resolve(__dirname,'../..');
const finalSource=fs.readFileSync(path.join(root,'gateways/http/static/js/pointcloud-processing.js'),'utf8');
const viewer=fs.readFileSync(path.join(root,'gateways/http/static/js/pointcloud3d.js'),'utf8');
const old='const measurement = visibleSource.length ? this._measurePointCloud(visibleSource) : null;';
const replacement='const measurement = visibleSource === src ? sourceMeasurement\n      : visibleSource.length ? this._measurePointCloud(visibleSource) : null;';
assert.equal(finalSource.split(replacement).length,2);
const source=finalSource.replace(replacement,old);
assert.equal(source.replace(old,replacement),finalSource);
const classes=[source,source.replace(old,replacement)].map(text=>Function(text.replace('export class PointCloudProcessor','class PointCloudProcessor')+'\nreturn PointCloudProcessor;')());
const configSource=viewer.slice(viewer.indexOf('const DEFAULT_POINTCLOUD3D_CONFIG ='),viewer.indexOf('const MAX_BUFFER_BYTES ='));
const config=Function(configSource+'\nreturn { stylePresets: POINT_STYLE_PRESETS, heightColorStops: HEIGHT_COLOR_STOPS, floorColors: FLOOR_COLORS };')();
const configSnapshot=JSON.stringify(config);
function make(count,shape='rooms'){const points=new Float32Array(count*3);for(let i=0;i<count;i++){points[i*3]=(i%317)*.07-10;points[i*3+1]=(Math.floor(i/317)%173)*.07-5;points[i*3+2]=shape==='flat'?0:((i%10)<6?0:(i%10)<9?2.8:((i%53)/53)*2.5);}return points;}
function instance(arm,input,style='height',live=false,lod='preview'){const p=new classes[arm](config);Object.assign(p,{_allPositions:input,_pointStyle:style,_lod:lod,_isLiveCloudMapSnapshot:live,_robotPose:{z:.4}});return p;}
function result(p){try{return {value:p._preparePointCloud(),activeFloor:p._activeFloorZ};}catch(e){return {error:{name:e.name,message:e.message}};}}
function instrument(p){const calls=[];const method=p._measurePointCloud;p._measurePointCloud=function(input){calls.push(input);return method.call(this,input);};return calls;}
let checks=0;
function equivalent(input,style='height',live=false,lod='preview'){
 const inputBefore=input.slice(),processors=classes.map((_,a)=>instance(a,input,style,live,lod)),calls=processors.map(instrument),values=processors.map(result);
 assert.deepStrictEqual(values[0],values[1]);assert.deepStrictEqual(input,inputBefore);assert.equal(JSON.stringify(config),configSnapshot);
 if(input.length&&['height','mono','unknown'].includes(style)){assert.equal(calls[0].length,2);assert.equal(calls[1].length,1);assert.equal(calls[0][0],input);assert.equal(calls[0][1],input);}
 if(style==='full'||!input.length){assert.equal(calls[0].length,0);assert.equal(calls[1].length,0);}
 if(style==='height'&&input.length&&values[0].value){assert.equal(values[0].value.positions,input);assert.equal(values[1].value.positions,input);}
 checks++;return {processors,calls,values};
}
function contracts(){
 for(const count of [0,1,16,4095,4096,4097])for(const style of ['height','mono','singleFloor','ground','structure','unknown','full'])for(const live of [false,true])equivalent(make(count),style,live);
 // Distinct filtered array must be remeasured even when every input point survives.
 for(const style of ['singleFloor','ground','structure']){const {calls}=equivalent(make(100,'flat'),style);for(const c of calls){assert.equal(c.length,2);assert.equal(c[0].length,c[1].length);assert.notEqual(c[0],c[1]);}checks++;}
 for(const values of [[0,-0,0],[1e30,-1e30,1e-30],[NaN,1,0],[1,2,NaN],[Infinity,1,0],[-Infinity,2,0]])for(const style of ['height','mono','singleFloor'])equivalent(new Float32Array(values),style);
 for(const arm of [0,1]){
  const input=make(16),p=instance(arm,input),first=p._preparePointCloud();first.measurement.bounds.center.x=999;first.measurement.xMin=999;
  input[0]=-50;const second=p._preparePointCloud();assert.equal(second.measurement.xMin,-50);assert.notEqual(second.measurement,first.measurement);assert.notEqual(second.measurement.bounds,first.measurement.bounds);assert.notEqual(second.measurement.bounds.center,first.measurement.bounds.center);assert.equal(second.positions,input);checks++;
  const floor=instance(arm,make(100),'ground');floor._preparePointCloud();const previous=floor._floorLevelSource;floor._allPositions=make(100,'flat');floor._preparePointCloud();assert.notEqual(floor._floorLevelSource,previous);assert.equal(floor._floorLevelSource,floor._allPositions);checks++;
 }
 // Independent basic coordinate bounds, not just A/B equality.
 for(const arm of [0,1]){const p=instance(arm,new Float32Array([-2,1,4,6,-3,0]),'height');assert.deepStrictEqual(p._preparePointCloud().measurement,{xMin:-2,xMax:6,yMin:-3,yMax:1,zMin:0,zMax:4,bounds:{center:{x:2,y:-1,z:2},radius:Math.hypot(8,4,4)/2}});checks++;}
}
function quantile(a,q){return [...a].sort((a,b)=>a-b)[Math.floor((a.length-1)*q)];}
function counting(input,style,live){return [0,1].map(a=>{const p=instance(a,input,style,live),calls=instrument(p),r=p._preparePointCloud();return {calls:calls.length,scanned_coordinates:calls.reduce((n,x)=>n+x.length,0),output_points:r.positions.length/3};});}
function main(){contracts();const output={node:process.version,checks,source_window:'Final production candidate; baseline reconstructed by reversing exactly one measurement expression',config_source:'Actual pointcloud3d.js default normalized presets/colors',scope:'Full synchronous _preparePointCloud with new processor per call, excluding worker serialization, viewer admission, GPU, network and browser',cohorts:[]};if(process.argv.includes('--contracts-only')){console.log(JSON.stringify(output,null,2));return;}
 const cohorts=[];for(const count of [4096,100000,1000000])for(const style of ['height','mono'])cohorts.push({count,style,live:false});for(const style of ['singleFloor','ground','structure'])cohorts.push({count:100000,style,live:false});cohorts.push({count:0,style:'height',live:false});
 for(const cohort of cohorts){const input=make(cohort.count),calls=counting(input,cohort.style,cohort.live),samples=[[],[]];const operation=a=>instance(a,input,cohort.style,cohort.live)._preparePointCloud();for(let i=0;i<10;i++){operation(0);operation(1);}for(let i=0;i<30;i++)for(const a of (i%2?[1,0]:[0,1])){const t=process.hrtime.bigint(),value=operation(a);const duration=Number(process.hrtime.bigint()-t)/1e6;assert.equal(value.colors.length,value.positions.length);samples[a].push(duration);}output.cohorts.push({...cohort,input_bytes:input.byteLength,calls,median_ms:samples.map(x=>quantile(x,.5)),p95_ms:samples.map(x=>quantile(x,.95)),samples_ms:samples});}
 console.log(JSON.stringify(output,null,2));}
main();
