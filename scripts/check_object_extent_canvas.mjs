#!/usr/bin/env node
// Console Canvas2D behavior with deterministic drawing recorder; no browser/GPU.
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
const source = fs.readFileSync(new URL('../gateways/http/static/js/console.js', import.meta.url), 'utf8');
const names = ['semanticExtent', 'semanticExtentDescription', 'semanticFootprint', 'drawSemanticFootprints', 'hitTestSemanticMarkers',
  'semanticInstanceName', 'fitSemanticMarkerLabel', 'semanticMarkerMetrics', 'semanticClusterMarkerMetrics',
  'drawCanvasRoundedRectPath', 'drawSingleMarker', 'drawClusterMarker', 'closeSemanticOverlay', 'publishSemanticMapInstances'];
const functions = names.map(name => {
  const start = source.indexOf(`      function ${name}(`);
  assert.ok(start >= 0);
  const end = source.indexOf('\n      function ', start + 1);
  return source.slice(start, end);
}).join('\n');
const calls = [];
const paints = [];
const ctx = Object.fromEntries(['beginPath','moveTo','lineTo','closePath','quadraticCurveTo','setLineDash','save','restore']
  .map(name => [name, (...args) => calls.push([name, ...args])]));
Object.assign(ctx, {
  measureText: text => ({ width: text.length * 7 }),
  fill() { calls.push(['fill']); paints.push({ kind: 'fill', color: this.fillStyle }); },
  stroke() { calls.push(['stroke']); paints.push({ kind: 'stroke', color: this.strokeStyle, width: this.lineWidth }); },
  fillText(text) { calls.push(['text', text]); },
});
const item = { instance_id: 'cabinet',frame_id:'map',pose:{x:2,y:3,z:-2},object_extent:{frame_id:'map',
  source:'detection3d',size_m:{x:4,y:.5,z:2},
  footprint_xy:[{x:1.75,y:1},{x:2.25,y:1},{x:2.25,y:5},{x:1.75,y:5}] } };
let items=[item];
const context = { state:{showSemanticInstances:true,semanticInstances:items},els:{mapCanvas:{getContext:()=>ctx}},
  cameraAnchor:()=>({x:0,y:0}),cameraScale:()=>10,clusterInstances:()=>[],resolveHighlightCluster:()=>null,
  semanticMapInstances:()=>items,mapToCanvas:(p,anchor,scale)=>({x:(p.x-anchor.x)*scale,y:-(p.y-anchor.y)*scale}) };
const metrics = source.slice(source.indexOf('      const SEMANTIC_MARKER_HEIGHT'), source.indexOf('      function semanticInstanceName'));
vm.createContext(context);vm.runInContext(metrics + functions,context);
context.drawSemanticFootprints(ctx,items,{x:0,y:0},10);
assert.deepEqual(calls.filter(c=>c[0]==='moveTo'||c[0]==='lineTo'),
  [['moveTo',17.5,-10],['lineTo',22.5,-10],['lineTo',22.5,-50],['lineTo',17.5,-50]]);
assert.equal(calls.filter(c=>c[0]==='fill').length,1);
assert.equal(paints.find(p=>p.kind==='fill').color,'rgba(138,167,184,.10)');
context.state.semanticSelectedId='cabinet';paints.length=0;
context.drawSemanticFootprints(ctx,items,{x:0,y:0},10);
assert.equal(paints.find(p=>p.kind==='stroke').width,2.5,'selected footprint has a stronger outline');
context.state.semanticSelectedId=null;
assert.equal(context.hitTestSemanticMarkers(20,-48).instance,item,'large footprint selectable outside label');
assert.equal(context.hitTestSemanticMarkers(30,-48),null);
assert.match(context.semanticExtentDescription(item),/4.00 × 0.50 × 2.00 米/);
calls.length=0;context.drawSemanticFootprints(ctx,items,{x:0,y:0},20);
assert.deepEqual(calls.find(c=>c[0]==='moveTo'),['moveTo',35,-20],'metric footprint zooms with map');
item.object_extent.source='observed_points';
item.object_extent.size_m={x:4,y:0,z:2};item.object_extent.footprint_xy=[{x:0,y:3},{x:4,y:3}];
calls.length=0;context.drawSemanticFootprints(ctx,items,{x:0,y:0},10);
assert.equal(calls.filter(c=>c[0]==='fill').length,0,'zero span remains line, no fabricated footprint');
assert.ok(calls.some(c=>c[0]==='setLineDash'&&c[1].join(',')==='5,3'));
assert.match(context.semanticExtentDescription(item),/遮挡部分未知/);
for(const change of [null,{...item.object_extent,frame_id:'camera'},{...item.object_extent,size_m:{x:NaN,y:1,z:1}}]){
  const unknown={...item,object_extent:change};calls.length=0;
  context.drawSemanticFootprints(ctx,[unknown],{x:0,y:0},10);
  assert.equal(calls.filter(c=>c[0]==='stroke').length,0);
  assert.match(context.semanticExtentDescription(unknown),/尺寸未知/);
}
assert.ok(source.indexOf('drawSemanticFootprints(ctx, instances, anchor, scale);') < source.indexOf('let drawX = cl.cx;'));
const named={...item,display_name:'cabinet'};
for(const highlighted of [false,true])for(const fallback of [false,true]){
  calls.length=0;paints.length=0;
  context.drawSingleMarker(ctx,50,50,named,fallback,false,highlighted);
  assert.equal(calls.find(c=>c[0]==='text')[1],'cabinet','map label contains only the name');
  assert.ok(paints.filter(p=>p.kind==='fill').every(p=>/^rgba\(/.test(p.color)),'name backgrounds stay translucent');
  calls.length=0;paints.length=0;
  context.drawClusterMarker(ctx,50,50,{members:[named,named]},fallback,fallback?2:0,highlighted);
  assert.equal(calls.find(c=>c[0]==='text')[1],'2 个物品');
  assert.ok(paints.filter(p=>p.kind==='fill').every(p=>/^rgba\(/.test(p.color)),'cluster backgrounds stay translucent');
}
const selectedIds=[],events=[];
let dialog=null,mapName='map-a';
Object.assign(context,{
  document:{getElementById:()=>dialog},requestMapDraw:()=>{},activeMapName:()=>mapName,
  window:{pointcloud3d:{setSemanticSelectedId:id=>selectedIds.push(id)},dispatchEvent:event=>events.push(event)},
  CustomEvent:class{constructor(type,options){this.type=type;this.detail=options.detail;}},
});
function select(){
  context.state.semanticSelectedId='cabinet';context.state.highlightTarget={record_id:'cabinet'};
  dialog={dataset:{instanceId:'cabinet'},close(){},remove(){dialog=null;}};
}
select();context.closeSemanticOverlay();context.closeSemanticOverlay();
assert.equal(context.state.semanticSelectedId,null);assert.equal(context.state.highlightTarget,null);
assert.equal(selectedIds.at(-1),null,'dismissal clears the 3D selection as well');
assert.match(source,/dialog\.addEventListener\("cancel", \(event\) => \{ event\.preventDefault\(\); closeSemanticOverlay\(\); \}\)/,'Escape uses the same dismissal');
select();context.state.highlightTarget.target_pose={x:1,y:2,z:0};context.closeSemanticOverlay();
assert.ok(context.state.highlightTarget.target_pose,'dismissal preserves an active navigation target');
select();context.state.semanticPublishedKey={snapshot:'s',mapName:'map-a'};
context.state.semanticSnapshotKey='s';mapName='map-b';context.publishSemanticMapInstances();
assert.equal(dialog,null,'switching maps dismisses the old object');
assert.equal(events.at(-1).detail.selectedId,null,'map-switch publication cannot restore the old selection');
console.log('PASS object extent Canvas2D: name-only translucent labels/clusters, selection/dismissal/map-switch, metric/rotated footprint, zoom, footprint hit-test, partial bounds and detail dimensions (recorded Canvas calls only)');
