import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';
import {fileURLToPath,pathToFileURL} from 'node:url';
const stage=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..'),repo=path.resolve(process.argv[2]||stage);
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'neh-cloud-viewer-'));
const modules=['pointcloud3d.js','pointcloud-incremental.js','pointcloud-pages.js','three.module.min.js','semantic-regions3d.js','scene-graph3d.js','OrbitControls.js','pointcloud-processing.js','dense-pointcloud.js'];
for(const name of modules){const staged=path.join(stage,'gateways/http/static/js',name);fs.copyFileSync(fs.existsSync(staged)?staged:path.join(repo,'gateways/http/static/js',name),path.join(temporary,name));}
fs.writeFileSync(path.join(temporary,'package.json'),'{"type":"module"}');
class Socket extends EventTarget{static OPEN=1;static all=[];constructor(){super();this.readyState=0;this.sent=[];Socket.all.push(this);}send(text){this.sent.push(JSON.parse(text));}emit(type,data){if(type==='open')this.readyState=1;const event=new Event(type);if(data!==undefined)Object.defineProperty(event,'data',{value:data});this.dispatchEvent(event);}close(){this.readyState=3;this.emit('close');}}
globalThis.WebSocket=Socket;globalThis.window=new EventTarget();Object.assign(window,{location:{href:'https://robot.example/'},setTimeout,clearTimeout});globalThis.document={getElementById:()=>null};
let viewer;
try{
 await import(pathToFileURL(path.join(temporary,'pointcloud3d.js')));
 viewer=window.pointcloud3d;assert.ok(viewer);const socket=viewer._incrementalConnection.socket;
 viewer.setMapContext({source_epoch:JSON.stringify(['map-view-v2','mapping-actual:7','display'])});socket.emit('open');socket.emit('message',JSON.stringify({type:'pointcloud.ready',version:2}));
 assert.equal(socket.sent[0].mapping_source_epoch,'mapping-actual:7');
 const input=JSON.parse(fs.readFileSync(path.join(process.env.NEH_POINTCLOUD_FIXTURE_DIR||path.join(os.tmpdir(),'neh-pointcloud-fixtures'),'pointcloud_native_wire_fixtures.json'),'utf8'));
 for(const step of input){for(const encoded of step.packets){const b=Buffer.from(encoded,'base64');socket.emit('message',b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength));}assert.equal(viewer._pointCloudSourcePoints,step.point_count);}
 assert.equal(viewer._incrementalActive,true);assert.equal(viewer._allPositions.length,0);assert.equal(viewer._lastCloudMapRevision,'cloud:40');
 const first=viewer._incrementalCloud.pages[0].array;viewer.setVisible(false);viewer.setVisible(true);assert.equal(viewer._incrementalCloud.pages[0].array,first);
 viewer.setPointStyle('ground');viewer.setRobotPose(0,0,3,0);viewer.clearRobotPose();assert.equal(viewer._incrementalCloud._robotZ,undefined);
 const header={...viewer._pointCloudMapWatermark.header,source:'saved',source_epoch:'saved-a',stream_generation:'2',mapping_source_epoch:'mapping-actual:7',point_count:1,chunk_size_points:1024,reset:true,revision:'asset:one',revision_ordinal:'1'};
 const saved={header,chunks:[{chunk_index:0,xyz:new Float32Array([1,2,3])}]};viewer._acceptCloudPageBatch(saved);viewer._incrementalConnection.assembler.accept(saved);
 assert.equal(viewer._incrementalActive,false);assert.deepEqual([...viewer._allPositions],[1,2,3]);
 const priorEpoch=viewer._cloudEpoch,priorLoad=viewer._cloudLoadToken;viewer._pointsMesh={visible:true,geometry:{dispose(){}},material:{dispose(){}}};
 viewer.setMapContext({source_epoch:'mapping-new:1'});assert.equal(viewer._pointsMesh.visible,false);assert.equal(viewer._allPositions.length,0);assert.equal(viewer._pointCloudMapWatermark,null);assert.ok(viewer._cloudEpoch>priorEpoch);assert.ok(viewer._cloudLoadToken>priorLoad);
 viewer.dispose();assert.equal(viewer._incrementalConnection,null);assert.equal(viewer._incrementalCloud,null);assert.equal(socket.readyState,3);
 console.log('PASS actual viewer module: cloud connection, live pages, hidden/reveal, floor pose clear, saved snapshot, map-change stale-worker/resource fences, dispose (CPU only; no WebGL)');
}finally{viewer?.dispose();fs.rmSync(temporary,{recursive:true,force:true});}
