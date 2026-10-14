#!/usr/bin/env node
// Allocation counts only; no timing or GPU measurement. Optional argument: baseline module path.
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL, fileURLToPath } from 'node:url';
import os from 'node:os';
const repo=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const dir=fs.mkdtempSync(path.join(os.tmpdir(),'neh-region-count-'));
for(const file of ['three.module.min.js','semantic-regions3d.js'])fs.copyFileSync(path.join(repo,'gateways/http/static/js',file),path.join(dir,file));
if(process.argv[2])fs.copyFileSync(path.resolve(process.argv[2]),path.join(dir,'semantic-regions3d.js'));
fs.writeFileSync(path.join(dir,'package.json'),'{"type":"module"}');
let canvases=0;
globalThis.document={createElement(name){if(name!=='canvas')throw Error(name);canvases++;const canvas={width:300,height:150};canvas.getContext=()=>({measureText:text=>({width:text.length*12}),fillRect(){},fillText(text){canvas.text=text}});return canvas;}};
const THREE=await import(pathToFileURL(path.join(dir,'three.module.min.js')));
const {SemanticRegionLayer}=await import(pathToFileURL(path.join(dir,'semantic-regions3d.js')));
const layer=new SemanticRegionLayer(new THREE.Scene());
const regions=Array.from({length:3},(_,i)=>({scene_space_id:`r${i}`,frame_id:'map',metadata:{scene_revision:1},primary_label:`Region ${i}`,display_color:'#648fb6',boundary:{frame_id:'map',polygon_points:[{x:i*3,y:0,z:0},{x:i*3+2,y:0,z:0},{x:i*3+2,y:2,z:0},{x:i*3,y:2,z:0}]}}));
const detail={visible:true,binding:{frame_id:'map',source_epoch:'epoch-a'},regions};
const resources=()=>layer.entries.flatMap(e=>[e.mesh.geometry,e.mesh.material,...e.lines.flatMap(l=>[l.geometry,l.material]),e.label.material.map,e.label.material]);
let watched=new Set(),disposed=0;
function watch(){for(const r of resources())if(!watched.has(r)){watched.add(r);r.addEventListener('dispose',()=>disposed++);}}
function step(name,change){const before=resources(),canvasBefore=canvases,disposeBefore=disposed;change();layer.update(detail);watch();const after=resources();console.log(JSON.stringify({name,canvasesCreated:canvases-canvasBefore,resourcesDisposed:disposed-disposeBefore,priorResourcesRetained:after.filter(r=>before.includes(r)).length,priorResourceCount:before.length,currentResourceCount:after.length}));}
step('bootstrap',()=>{});
step('selection-only',()=>{detail.selectedId='r1'});
step('one-region-revision-only',()=>{regions[1].metadata.scene_revision++});
step('one-region-label',()=>{regions[1].primary_label='Edited'});
step('one-region-polygon',()=>{regions[1].boundary.polygon_points[1].x+=.1});
step('binding-only-revision',()=>{detail.binding.update_sequence=2});
layer.dispose();
console.log(JSON.stringify({scope:'Actual SemanticRegionLayer + bundled Three CPU objects; synthetic regions, stub Canvas2D, no browser or GPU measurement',allResourcesDisposedExactlyOnce:disposed===watched.size,canvasesTotal:canvases,totalResources:watched.size}));

fs.rmSync(dir,{recursive:true,force:true});
