#!/usr/bin/env node
// Console Canvas2D behavior with deterministic drawing recorder; no browser/GPU.
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
const source = fs.readFileSync(new URL('../gateways/http/static/js/console.js', import.meta.url), 'utf8');
const names = ['semanticExtent', 'semanticExtentDescription', 'semanticFootprint', 'drawSemanticFootprints', 'hitTestSemanticMarkers'];
const functions = names.map(name => {
  const start = source.indexOf(`      function ${name}(`);
  assert.ok(start >= 0);
  const end = source.indexOf('\n      function ', start + 1);
  return source.slice(start, end);
}).join('\n');
const calls = [];
const ctx = Object.fromEntries(['beginPath','moveTo','lineTo','closePath','fill','stroke','setLineDash','save','restore']
  .map(name => [name, (...args) => calls.push([name, ...args])]));
const item = { instance_id: 'cabinet',frame_id:'map',pose:{x:2,y:3,z:-2},object_extent:{frame_id:'map',
  source:'detection3d',size_m:{x:4,y:.5,z:2},
  footprint_xy:[{x:1.75,y:1},{x:2.25,y:1},{x:2.25,y:5},{x:1.75,y:5}] } };
let items=[item];
const context = { state:{showSemanticInstances:true,semanticInstances:items},els:{mapCanvas:{getContext:()=>ctx}},
  cameraAnchor:()=>({x:0,y:0}),cameraScale:()=>10,clusterInstances:()=>[],resolveHighlightCluster:()=>null,
  semanticMapInstances:()=>items,mapToCanvas:(p,anchor,scale)=>({x:(p.x-anchor.x)*scale,y:-(p.y-anchor.y)*scale}) };
vm.createContext(context);vm.runInContext(functions,context);
context.drawSemanticFootprints(ctx,items,{x:0,y:0},10);
assert.deepEqual(calls.filter(c=>c[0]==='moveTo'||c[0]==='lineTo'),
  [['moveTo',17.5,-10],['lineTo',22.5,-10],['lineTo',22.5,-50],['lineTo',17.5,-50]]);
assert.equal(calls.filter(c=>c[0]==='fill').length,1);
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
console.log('PASS object extent Canvas2D: metric/rotated footprint, zoom, footprint hit-test, partial bounds and unknown dimensions (recorded Canvas calls only)');
