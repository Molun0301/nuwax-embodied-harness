#!/usr/bin/env node
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';
const staging = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const repository = path.resolve(process.argv[2] || process.env.NEH_REPOSITORY || staging);
const liveJs = path.join(repository, 'gateways/http/static/js');
const moduleSource = fs.readFileSync(path.join(staging, 'gateways/http/static/js/pointcloud-pages.js'), 'utf8');
const threeUrl = `data:text/javascript;base64,${fs.readFileSync(path.join(liveJs, 'three.module.min.js')).toString('base64')}`;
const moduleText = moduleSource.replace("'./three.module.min.js'", JSON.stringify(threeUrl));
const THREE = await import(threeUrl);
const { IncrementalPointCloudLayer, POINT_CLOUD_PAGE_POINTS: PAGE, POINT_CLOUD_RENDER_POINTS: RENDER } = await import(`data:text/javascript;base64,${Buffer.from(moduleText).toString('base64')}`);
const processorText = fs.readFileSync(path.join(liveJs, 'pointcloud-processing.js'), 'utf8');
const { PointCloudProcessor } = await import(`data:text/javascript;base64,${Buffer.from(processorText).toString('base64')}`);
const config = {
  stylePresets: {
    height: { preserveSource: true, colorMode: 'height', size: 1.35 },
    mono: { colorMode: 'mono', size: 1.1 },
    singleFloor: { colorMode: 'singleFloor', floorFilter: 'story', liveSpacingM: 0.08, size: 1.35 },
    ground: { colorMode: 'ground', floorFilter: 'ground', liveSpacingM: 0.08, size: 1.55 },
    structure: { colorMode: 'structure', floorFilter: 'story', liveSpacingM: 0.08, size: 1.35 },
    full: { denseSource: true },
  },
  heightColorStops: [[56,189,248],[34,211,238],[74,222,128],[250,204,21],[251,113,133]],
  floorColors: { ground: [56,189,248], context: [203,213,225], structure: [248,250,252], object: [251,191,36] },
};
const freshConfig = () => structuredClone(config);
const make = count => new Float32Array(Array.from({length:count * 3}, (_, i) => i % 3 === 2 ? (Math.floor(i / 3) % 31) / 10 : ((i * 19) % 1709) / 10));
function transaction(points, indices = null, reset = false) {
  const count = points.length / 3;
  const ids = indices || Array.from({length:Math.ceil(count / PAGE)}, (_, i) => i);
  return {header:{point_count:count,chunk_size_points:PAGE,reset},chunks:ids.map(chunk_index => ({chunk_index,xyz:points.subarray(chunk_index * PAGE * 3,Math.min(points.length,(chunk_index + 1) * PAGE * 3))}))};
}
function source(layer) { return new Float32Array(layer.pages.flatMap(page => Array.from(page.array.subarray(0,page.count * 3)))); }
function displayed(layer) { return new Float32Array(layer.derivedPages.flatMap(page => Array.from(page.array.subarray(0,page.count * 3)))); }
function clearRanges(layer) { for (const page of [...layer.pages,...layer.derivedPages]) for (const attr of Object.values(page.geometry.attributes)) attr.clearUpdateRanges(); }
function orderedRows(positions, colors = null) {
  const rows = [];
  for(let i=0;i<positions.length;i+=3) rows.push([...positions.subarray(i,i+3),...(colors ? colors.subarray(i,i+3) : [])]);
  return rows.sort((a,b) => {for(let j=0;j<a.length;j++) if(a[j]!==b[j]) return a[j]-b[j];return 0;});
}
function cpuShaderColors(layer) {
  const positions = displayed(layer), colors = [];
  for(let i=0;i<positions.length/3;i++) {
    const page = layer.derivedPages[Math.floor(i/RENDER)], offset=i%RENDER;
    const z=positions[i*3+2], preset=layer.config.stylePresets[layer.style];
    let color=layer.config.floorColors.context;
    if (z <= layer.floorContext.groundMax) color=layer.config.floorColors.ground;
    else if(preset.colorMode === 'structure') color=layer.config.floorColors[page.geometry.getAttribute('pcStructure').array[offset] ? 'structure' : 'object'];
    colors.push(...color);
  }
  return new Uint8Array(colors);
}
function oracle(points, style, cfg=config, robotZ=undefined) {
  const processor=new PointCloudProcessor(cfg);
  Object.assign(processor,{_allPositions:points,_pointStyle:style,_lod:'preview',_isLiveCloudMapSnapshot:true,_robotPose:{z:robotZ}});
  return processor._preparePointCloud();
}
function equivalent(layer, points, style=layer.style, robotZ=undefined) {
  const expected=oracle(points,style,layer.config,robotZ);
  assert.deepEqual(layer.floorContext,expected.floorContext);
  assert.deepEqual(JSON.parse(JSON.stringify(layer.measurement)),JSON.parse(JSON.stringify(expected.measurement)));
  if(layer.config.stylePresets[style].floorFilter) assert.deepEqual(orderedRows(displayed(layer),cpuShaderColors(layer)),orderedRows(expected.positions,expected.colors));
  else assert.deepEqual(source(layer),expected.positions);
}
let checks=0;
function check(name,run) {run();checks++;console.log(`PASS ${name}`);}
check('persistent source pages copy and dirty only replaced page',()=>{
  const layer=new IncrementalPointCloudLayer({config:freshConfig()}), raw=make(PAGE*33+25);
  layer.apply(transaction(raw,null,true));
  assert.equal(layer.stats.bootstrap.copiedCoordinates,raw.length);
  const identities=layer.pages.map(p=>[p,p.array,p.geometry,p.mesh,p.geometry.getAttribute('position').version]);
  clearRanges(layer);raw[PAGE*3+2]=20;
  layer.apply(transaction(raw,[1]));
  layer.pages.forEach((p,i)=>{
    assert.equal(p,identities[i][0]);assert.equal(p.array,identities[i][1]);assert.equal(p.geometry,identities[i][2]);assert.equal(p.mesh,identities[i][3]);
    assert.equal(p.geometry.getAttribute('position').version,identities[i][4]+(p.slabIndex===0?1:0));
    assert.equal(p.geometry.getAttribute('position').updateRanges.length,p.slabIndex===0?1:0);
  });
  assert.equal(layer.stats.lastApply.copiedCoordinates,PAGE*3);
  assert.equal(layer.stats.lastApply.scannedCoordinates,PAGE*3);
  assert.equal(layer.stats.lastApply.gpuDirtyBytes,PAGE*12);
  assert.equal(layer.stats.lastApply.pageAllocations,0);assert.equal(layer.stats.lastApply.derivedRebuilds,0);
  assert.equal(layer.uniforms.pcZMax.value,20);equivalent(layer,raw);
  layer.dispose();
});
check('point_count-only truncation uses drawRange without coordinate upload',()=>{
  const layer=new IncrementalPointCloudLayer(),raw=make(PAGE*4+25);layer.apply(transaction(raw,null,true));
  clearRanges(layer);const surviving=layer.pages[1],trimmed=raw.subarray(0,(PAGE+1)*3);
  layer.apply(transaction(trimmed,[]));
  assert.equal(layer.pages.length,2);assert.equal(layer.pages[1],surviving);assert.equal(surviving.geometry.drawRange.count,PAGE+1);
  assert.equal(layer.stats.lastApply.copiedCoordinates,0);assert.equal(layer.stats.lastApply.gpuDirtyBytes,0);
  assert.equal(layer.stats.lastApply.scannedCoordinates,3);assert.equal(layer.stats.lastApply.pageDisposals,3);equivalent(layer,trimmed);
});
check('deleting sole global z extrema changes bounds/uniforms without touching unchanged slabs',()=>{
  const raw=make(PAGE*48+2),layer=new IncrementalPointCloudLayer();
  raw[2]=-100;raw[PAGE*16*3+2]=100;layer.apply(transaction(raw,null,true));clearRanges(layer);
  assert.equal(layer.uniforms.pcZMin.value,-100);assert.equal(layer.uniforms.pcZMax.value,100);
  const unchanged=layer.sourceSlabs[2],saved=unchanged.array.slice(),version=unchanged.geometry.getAttribute('position').version;
  const oldBounds=unchanged.geometry.boundingBox.clone();
  raw.set(raw.subarray(raw.length-3),0);raw.set(raw.subarray(raw.length-6,raw.length-3),PAGE*16*3);
  const trimmed=raw.subarray(0,raw.length-6);layer.apply(transaction(trimmed,[0,16]));
  equivalent(layer,trimmed);assert.equal(layer.uniforms.pcZMin.value,0);assert.equal(layer.uniforms.pcZMax.value,3);
  assert.equal(layer.sourceSlabs[2],unchanged);assert.deepEqual(unchanged.array,saved);
  assert.equal(unchanged.geometry.getAttribute('position').version,version);
  assert.deepEqual(unchanged.geometry.getAttribute('position').updateRanges,[]);
  assert.deepEqual(unchanged.geometry.boundingBox,oldBounds);
  assert.equal(layer.stats.lastApply.copiedCoordinates,PAGE*6);assert.equal(layer.stats.lastApply.gpuDirtyBytes,PAGE*24);
  assert.equal(layer.stats.lastApply.derivedCopiedCoordinates,0);assert.equal(layer.stats.lastApply.pageAllocations,0);
  assert.match(layer.stats.gpuMeasurement,/scheduled/);
});
check('append and producer dense-slot swap removal preserve exact source',()=>{
  const layer=new IncrementalPointCloudLayer(),initial=make(PAGE+3);layer.apply(transaction(initial,null,true));
  const raw=make(PAGE*3+1);layer.apply(transaction(raw,[1,2,3]));assert.deepEqual(source(layer),raw);
  raw.set(raw.subarray(raw.length-3),15);const removed=raw.subarray(0,raw.length-3);
  layer.apply(transaction(removed,[0]));assert.deepEqual(source(layer),removed);
  assert.equal(layer.pages.length,3);assert.equal(layer.pages[2].count,PAGE);
});
check('all validation is atomic including missing growth, duplicates and bad last coordinate',()=>{
  const layer=new IncrementalPointCloudLayer(),raw=make(PAGE+5);layer.apply(transaction(raw,null,true));
  const pages=layer.pages.slice(),before=source(layer),stats=JSON.stringify(layer.stats),count=layer.pointCount;
  const invalid=[transaction(make(PAGE*3),[2]),transaction(raw,[0,0]),transaction(raw,[0])];
  invalid[2].chunks[0].xyz=invalid[2].chunks[0].xyz.slice();invalid[2].chunks[0].xyz[PAGE*3-1]=NaN;
  for(const tx of invalid) assert.throws(()=>layer.apply(tx));
  assert.deepEqual(layer.pages,pages);assert.deepEqual(source(layer),before);assert.equal(layer.pointCount,count);assert.equal(JSON.stringify(layer.stats),stats);
  assert.throws(()=>new IncrementalPointCloudLayer().apply(transaction(raw,[1],true)),/Incomplete/);
});
check('prepared transactions commit once and stale preparations are revalidated',()=>{
  const layer=new IncrementalPointCloudLayer(),raw=make(PAGE*2),tx=transaction(raw,null,true),prepared=layer.prepare(tx);
  assert.equal(layer.pointCount,0);assert.equal(layer.pages.length,0);layer.apply(tx,prepared);assert.equal(layer.pointCount,PAGE*2);
  const change=transaction(raw,[0]),stage=layer.prepare(change);layer.clear();assert.throws(()=>layer.apply(change,stage),/reset baseline/);
});
check('pending GPU update ranges survive multiple transactions before rendering',()=>{
  const layer=new IncrementalPointCloudLayer(),raw=make(PAGE*3);layer.apply(transaction(raw,null,true));clearRanges(layer);
  raw[0]=100;layer.apply(transaction(raw,[0]));raw[PAGE*6]=200;layer.apply(transaction(raw,[2]));
  assert.deepEqual(layer.sourceSlabs[0].geometry.getAttribute('position').updateRanges,[{start:0,count:PAGE*3},{start:PAGE*6,count:PAGE*3}]);
  assert.equal(layer.stats.lastApply.gpuUploadCount,1);
});
function floors() {
  const points=[];
  for(let floor=0;floor<3;floor++)for(let y=0;y<20;y++)for(let x=0;x<24;x++)points.push(x*.11-1.3,y*.11-1.1,floor*3);
  for(let floor=0;floor<3;floor++)for(let z=1;z<25;z++)for(let y=0;y<8;y++)points.push(2.4,y*.15,floor*3+z*.08);
  for(let i=0;i<30;i++)points.push(-.2+(i%4)*.02,-.3,0.5+(i%2)*.05);
  return new Float32Array(points);
}
check('floor/ground/structure golden matches existing live transforms and RGB',()=>{
  const raw=floors(),layer=new IncrementalPointCloudLayer({config:freshConfig()});layer.apply(transaction(raw,null,true));
  for(const style of ['singleFloor','ground','structure','height','mono']) {layer.setStyle(style);equivalent(layer,raw,style);}
  assert.equal(layer.stats.explicit.derivedRebuilds,1);
  layer.setStyle('structure');for(const z of [0,3.15,6.2,-2]) {layer.setRobotPose({z});equivalent(layer,raw,'structure',z);}
});
check('floor constructor, negative voxels and huge tuple coordinates match legacy',()=>{
  const values=[];for(let i=0;i<30;i++)values.push(i*.08-.8,0,0);
  values.push(-1,-0,0,-.01,0,0,2**32,0,0,0,2**32,0,1.1,2,0.3);
  const raw=new Float32Array(values),layer=new IncrementalPointCloudLayer({config:freshConfig(),style:'structure'});
  layer.apply(transaction(raw,null,true));equivalent(layer,raw);assert.equal(layer._voxels.size,[...layer._voxels.keys()].length);
  assert.ok([...layer._voxels.keys()].every(key=>typeof key==='string'&&key.split(',').length===3));
});
check('coarse cells use exact pre-sampling eligibility rather than centroid or min/max guesses',()=>{
  const cfg=freshConfig();for(const p of Object.values(cfg.stylePresets))if(p.floorFilter)p.liveSpacingM=1.6;
  const raw=[];for(let i=0;i<30;i++)raw.push(i*.1,0,0);raw.push(10.1,0,.3,10.2,0,1.5,20,0,-.2,20,0,-.01);
  const points=new Float32Array(raw),layer=new IncrementalPointCloudLayer({config:cfg});layer.apply(transaction(points,null,true));
  for(const style of ['ground','singleFloor','structure']){layer.setStyle(style);equivalent(layer,points,style);}
});
check('dirty floor pages update voxel and structure metadata without source rebuild',()=>{
  const raw=floors(),layer=new IncrementalPointCloudLayer({config:freshConfig()});layer.apply(transaction(raw,null,true));layer.setStyle('structure');
  clearRanges(layer);const sourceIdentities=layer.pages.map(p=>p.array),derivedIdentity=layer.derivedPages[0].geometry;
  for(let step=0;step<8;step++){
    const point=step*17;raw[point*3]=4+step*.2;raw[point*3+2]=.64;
    layer.apply(transaction(raw,[0]));equivalent(layer,raw);
    assert.equal(layer.stats.lastApply.derivedRebuilds,0);assert.equal(layer.stats.lastApply.rebuiltSourcePoints,0);
    assert.equal(layer.stats.lastApply.scannedCoordinates,PAGE*3);assert.equal(layer.stats.lastApply.pageAllocations,0);
    assert.equal(layer.derivedPages[0].geometry,derivedIdentity);layer.pages.forEach((p,i)=>assert.equal(p.array,sourceIdentities[i]));
  }
});
check('height peak changes reclassify metadata boundaries without scanning unmodified source XYZ',()=>{
  const points=[];for(let i=0;i<PAGE;i++)points.push(i*.01,0,0);for(let i=0;i<PAGE;i++)points.push(i*.01,1,3);
  const raw=new Float32Array(points),layer=new IncrementalPointCloudLayer({config:freshConfig()});layer.apply(transaction(raw,null,true));layer.setStyle('ground');
  for(let i=0;i<PAGE;i++)raw[i*3+2]=.08;
  layer.apply(transaction(raw,[0]));equivalent(layer,raw);assert.equal(layer.stats.lastApply.scannedCoordinates,PAGE*3);assert.equal(layer.stats.lastApply.derivedRebuilds,0);
});
check('explicit spacing rebuilds once, colors and height/mono changes upload no coordinates',()=>{
  const layer=new IncrementalPointCloudLayer({config:freshConfig()}),raw=floors();layer.apply(transaction(raw,null,true));
  layer.setStyle('mono');assert.equal(layer.stats.lastApply.copiedCoordinates,0);assert.equal(layer.stats.lastApply.gpuDirtyBytes,0);
  const cfg=freshConfig();cfg.floorColors.context=[1,2,3];layer.configure(cfg);assert.equal(layer.stats.lastApply.derivedCopiedCoordinates,0);
  layer.setStyle('structure');assert.equal(layer.stats.lastApply.derivedRebuilds,1);assert.equal(layer.stats.lastApply.rebuiltSourcePoints,raw.length/3);
  cfg.stylePresets.structure.liveSpacingM=.16;layer.configure(cfg);assert.equal(layer.stats.lastApply.derivedRebuilds,1);equivalent(layer,raw);
  cfg.floorColors.structure=[2,4,6];layer.configure(cfg);assert.equal(layer.stats.lastApply.derivedRebuilds,0);assert.equal(layer.stats.lastApply.derivedGpuDirtyBytes,0);equivalent(layer,raw);
});
check('all-point histogram remains exact beyond old 200000 analysis sampling cap',()=>{
  const raw=new Float32Array(200001*3);for(let i=0;i<200001;i++){raw[i*3]=i*.001;raw[i*3+2]=i%2?3:0;}
  const layer=new IncrementalPointCloudLayer();layer.apply(transaction(raw,null,true));
  assert.equal(layer._histogram.get(0).count,100001);assert.equal(layer._histogram.get(Math.round(3/.08)).count,100000);
  const last=Math.floor(200000/PAGE);raw[200000*3+2]=3;layer.apply(transaction(raw,[last]));
  assert.equal(layer._histogram.get(0).count,100000);assert.equal(layer._histogram.get(Math.round(3/.08)).count,100001);
  assert.equal(layer.stats.lastApply.copiedCoordinates,(200001-last*PAGE)*3);assert.equal(layer.stats.lastApply.pageAllocations,0);
});
check('shader uses global height/colors and structure flags instead of RGB attributes',()=>{
  const layer=new IncrementalPointCloudLayer({config:freshConfig()});layer.apply(transaction(make(4),null,true));
  assert.equal(layer.pages[0].geometry.getAttribute('color'),undefined);
  const shader={uniforms:{},vertexShader:'#include <begin_vertex>',fragmentShader:'#include <color_fragment>'};layer.material.onBeforeCompile(shader);
  assert.equal(shader.uniforms.pcZMin,layer.uniforms.pcZMin);assert.match(shader.vertexShader,/uniform vec3 pcStops\[5\]/);
  assert.match(shader.vertexShader,/floor\(pcColor \* 255\.0 \+ 0\.5\)/);assert.doesNotMatch(shader.vertexShader,/attribute float pcStructure/);
  layer.setStyle('structure');const derived={uniforms:{},vertexShader:'#include <begin_vertex>',fragmentShader:'#include <color_fragment>'};layer.derivedMaterial.onBeforeCompile(derived);
  assert.match(derived.vertexShader,/attribute float pcStructure/);assert.ok(layer.derivedPages.every(p=>p.geometry.getAttribute('color')===undefined));
});
check('actual Three r170 shader chunks preserve legacy normalized-byte and output-sRGB semantics',()=>{
  assert.equal(THREE.REVISION,'170');assert.match(THREE.ShaderChunk.color_fragment,/diffuseColor\.rgb \*= vColor/);
  assert.match(THREE.ShaderChunk.color_vertex,/vColor \*= color/);
  assert.match(THREE.ShaderChunk.colorspace_fragment,/linearToOutputTexel/);
  const layer=new IncrementalPointCloudLayer({config:freshConfig()});
  const shader={uniforms:{},vertexShader:THREE.ShaderLib.points.vertexShader,fragmentShader:THREE.ShaderLib.points.fragmentShader};
  layer.material.onBeforeCompile(shader);
  assert.match(shader.fragmentShader,/diffuseColor\.rgb \*= pcColor/);
  assert.ok(shader.fragmentShader.indexOf('diffuseColor.rgb *= pcColor')<shader.fragmentShader.indexOf('#include <colorspace_fragment>'));
  assert.match(shader.fragmentShader,/#include <tonemapping_fragment>/);assert.match(shader.fragmentShader,/#include <colorspace_fragment>/);
  assert.equal(shader.uniforms.pcContextColor,layer.uniforms.pcContextColor);
  assert.deepEqual(layer.uniforms.pcContextColor.value.toArray(),config.floorColors.context.map(v=>v/255));
  const legacy=new THREE.Uint8BufferAttribute(config.floorColors.context,3,true);
  assert.deepEqual([legacy.getX(0),legacy.getY(0),legacy.getZ(0)],layer.uniforms.pcContextColor.value.toArray());
  assert.equal(layer.material.color.r,1);assert.equal(layer.material.color.g,1);assert.equal(layer.material.color.b,1);
});
check('picking rejects hidden source pages and disposed layers release geometry/material',()=>{
  const layer=new IncrementalPointCloudLayer({config:freshConfig()}),raw=floors();layer.apply(transaction(raw,null,true));
  const sourceHit={object:layer.pages[0].mesh,index:0};assert.equal(layer.filterIntersection(sourceHit),true);layer.setStyle('structure');assert.equal(layer.filterIntersection(sourceHit),false);
  const hit={object:layer.derivedPages[0].mesh,index:0};assert.equal(layer.filterIntersection(hit),true);layer.setVisible(false);assert.equal(layer.filterIntersection(hit),false);
  layer.setVisible(true);layer.setStyle('full');assert.equal(layer.filterIntersection(hit),false);layer.setStyle('height');
  let geometries=0,materials=0;for(const p of [...layer.sourceSlabs,...layer.derivedPages])p.geometry.addEventListener('dispose',()=>geometries++);
  layer.material.addEventListener('dispose',()=>materials++);layer.derivedMaterial.addEventListener('dispose',()=>materials++);
  const expected=layer.sourceSlabs.length+layer.derivedPages.length;layer.dispose();layer.dispose();assert.equal(geometries,expected);assert.equal(materials,2);assert.equal(layer.group.children.length,0);
  assert.throws(()=>layer.apply(transaction(raw,null,true)),/disposed/);
});
check('empty reset and truncation dispose pages and clear bounds/derived state',()=>{
  const layer=new IncrementalPointCloudLayer({config:freshConfig()}),raw=floors();layer.apply(transaction(raw,null,true));layer.setStyle('structure');
  layer.apply(transaction(new Float32Array(),[]));assert.equal(layer.pointCount,0);assert.equal(layer.displayedPointCount,0);assert.equal(layer.bounds,null);assert.equal(layer.pages.length,0);assert.equal(layer.derivedPages.length,0);
  layer.apply(transaction(raw,null,true));equivalent(layer,raw);layer.apply(transaction(new Float32Array(),[],true));assert.equal(layer._histogram.size,0);assert.equal(layer._voxels.size,0);
});
check('randomized append, replacement, swap-remove and floor switches match fresh transforms',()=>{
  let seed=73129;const random=()=>{seed=(seed*16807)%2147483647;return seed/2147483647;};
  let raw=floors();const layer=new IncrementalPointCloudLayer({config:freshConfig()});layer.apply(transaction(raw,null,true));layer.setStyle('structure');
  for(let step=0;step<60;step++) {
    const operation=step%3, oldCount=raw.length/3, dirty=new Set();
    if(operation===0) {
      const next=new Float32Array(raw.length+3*(1+Math.floor(random()*120)));next.set(raw);raw=next;
      for(let i=oldCount;i<raw.length/3;i++){raw[i*3]=random()*5-2;raw[i*3+1]=random()*3;raw[i*3+2]=Math.floor(random()*3)*3+(random()<.6?0:random()*2);dirty.add(Math.floor(i/PAGE));}
      dirty.add(Math.floor((oldCount-1)/PAGE));
    } else if(operation===1) {
      for(let i=0;i<50;i++){const point=Math.floor(random()*oldCount);raw[point*3]=random()*5-2;raw[point*3+2]=Math.floor(random()*3)*3;dirty.add(Math.floor(point/PAGE));}
    } else {
      const remove=1+Math.floor(random()*100);const target=Math.floor(random()*(oldCount-remove));
      for(let i=0;i<remove;i++){raw.set(raw.subarray((oldCount-1-i)*3,(oldCount-i)*3),(target+i)*3);dirty.add(Math.floor((target+i)/PAGE));}
      raw=raw.slice(0,(oldCount-remove)*3);
      for(const id of [...dirty])if(id*PAGE>=raw.length/3)dirty.delete(id);
    }
    layer.apply(transaction(raw,[...dirty]));
    if(step%7===0)layer.setStyle(['singleFloor','ground','structure'][Math.floor(step/7)%3]);
    equivalent(layer,raw);
    assert.equal(layer.stats.steady.derivedRebuilds,0);
  }
});
check('dense single-voxel updates query aggregate bounds instead of rescanning raw XYZ',()=>{
  const cfg=freshConfig();for(const preset of Object.values(cfg.stylePresets))if(preset.floorFilter)preset.liveSpacingM=1.6;
  const raw=new Float32Array(50000*3);for(let i=0;i<50000;i++){raw[i*3]=i/100000;raw[i*3+1]=(i%250)/1000;}
  const layer=new IncrementalPointCloudLayer({config:cfg});layer.apply(transaction(raw,null,true));layer.setStyle('ground');
  assert.equal(layer._voxels.size,1);assert.equal(layer.displayedPointCount,1);
  raw[0]=.75;layer.apply(transaction(raw,[0]));equivalent(layer,raw);
  assert.equal(layer.stats.lastApply.voxelBoundsQueryNodes,1);assert.equal(layer.stats.lastApply.scannedCoordinates,PAGE*3);
  assert.equal(layer.stats.lastApply.derivedCopiedCoordinates,0);assert.equal(layer.stats.lastApply.derivedGpuDirtyBytes,0);
});
check('large canonical map stays in 86 stable slabs and one dirty page schedules 12KiB',()=>{
  const count=1398101,raw=make(count),layer=new IncrementalPointCloudLayer();
  const bootstrap=performance.now();layer.apply(transaction(raw,null,true));const bootstrapMs=performance.now()-bootstrap;
  assert.equal(layer.sourceSlabs.length,86);assert.equal(layer.stats.gauges.sourceDrawCalls,86);
  assert.equal(layer.stats.gauges.allocatedSourcePoints,86*RENDER);assert.equal(layer.stats.bootstrap.gpuUploadCount,86);
  const slabs=layer.sourceSlabs.map(s=>[s,s.array,s.geometry]);const timings=[];
  for(let step=0;step<30;step++) {
    clearRanges(layer);const index=(step*41)%Math.floor(count/PAGE);raw[index*PAGE*3]=step+1000;
    const start=performance.now();layer.apply(transaction(raw,[index]));timings.push(performance.now()-start);
    assert.equal(layer.stats.lastApply.copiedCoordinates,PAGE*3);assert.equal(layer.stats.lastApply.scannedCoordinates,PAGE*3);
    assert.equal(layer.stats.lastApply.gpuDirtyBytes,12288);assert.equal(layer.stats.lastApply.gpuUploadCount,1);
    assert.equal(layer.stats.lastApply.gpuAllocationBytes,0);assert.equal(layer.stats.lastApply.sourceSlabAllocations,0);
    layer.sourceSlabs.forEach((slab,i)=>{assert.equal(slab,slabs[i][0]);assert.equal(slab.array,slabs[i][1]);assert.equal(slab.geometry,slabs[i][2]);});
  }
  timings.sort((a,b)=>a-b);
  console.log(JSON.stringify({points:count,sourcePages:layer.pages.length,sourceDrawCalls:86,allocatedSourcePoints:86*RENDER,
    bootstrapMs:Number(bootstrapMs.toFixed(3)),dirtyPageMedianMs:Number(timings[Math.floor(timings.length/2)].toFixed(3)),
    dirtyPageP95Ms:Number(timings[Math.floor(timings.length*.95)].toFixed(3)),steadyCopiedCoordinates:layer.stats.steady.copiedCoordinates,
    steadyGpuDirtyBytes:layer.stats.steady.gpuDirtyBytes,steadyGpuUploadCount:layer.stats.steady.gpuUploadCount,
    gpuMeasurement:'scheduled BufferAttribute bytes/ranges; no WebGL driver timing'}));
  layer.dispose();
});
console.log(`PASS ${checks} incremental page-store/transform checks (Three r170, CPU attribute inspection; no GPU render)`);
