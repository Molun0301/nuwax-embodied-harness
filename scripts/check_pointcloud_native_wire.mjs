import os from 'node:os';
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {fileURLToPath} from 'node:url';
const stage=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const repo=path.resolve(process.argv[2]||stage),js=path.join(stage,'gateways/http/static/js');
const moduleUrl=text=>'data:text/javascript;base64,'+Buffer.from(text).toString('base64');
const protocol=await import(moduleUrl(fs.readFileSync(path.join(js,'pointcloud-incremental.js'),'utf8')));
const three=moduleUrl(fs.readFileSync(path.join(repo,'gateways/http/static/js/three.module.min.js'),'utf8'));
const {IncrementalPointCloudLayer}=await import(moduleUrl(fs.readFileSync(path.join(js,'pointcloud-pages.js'),'utf8').replace("'./three.module.min.js'",JSON.stringify(three))));
const inputs=JSON.parse(fs.readFileSync(path.join(process.env.NEH_POINTCLOUD_FIXTURE_DIR||path.join(os.tmpdir(),'neh-pointcloud-fixtures'),'pointcloud_native_wire_fixtures.json'),'utf8'));
const assembler=new protocol.PointCloudPageAssembler(),layer=new IncrementalPointCloudLayer();
assembler.setMapEpoch('mapping-actual:7');
const stats=[];let firstArray=null,firstGeometry=null;
for(const input of inputs){
 let batch=null;
 for(const packet of input.packets){const bytes=Buffer.from(packet,'base64');const buffer=bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength);const candidate=assembler.push(buffer);if(candidate)batch=candidate;}
 assert.ok(batch);assert.equal(batch.header.mapping_source_epoch,'mapping-actual:7');
 layer.apply(batch);assembler.accept(batch);
 const actual=layer.pages.flatMap(page=>Array.from(page.array.subarray(0,page.count*3)));
 assert.deepEqual(actual,input.expected_xyz);assert.equal(layer.pointCount,input.point_count);
 if(!firstArray){firstArray=layer.pages[0].array;firstGeometry=layer.pages[0].geometry;}
 else{assert.equal(layer.pages[0].array,firstArray);assert.equal(layer.pages[0].geometry,firstGeometry);}
 if(input.action==='edit'){assert.equal(layer.stats.lastApply.copiedCoordinates,3072);assert.equal(layer.stats.lastApply.gpuDirtyBytes,12288);}
 stats.push({action:input.action,revision:assembler.base.revision_ordinal,points:layer.pointCount,changedPages:batch.chunks.length,stats:layer.stats.lastApply});
}
layer.dispose();console.log(JSON.stringify({checks:inputs.length,path:'staged v3 mmap -> actual LivePointCloudMapSource -> WS packets -> decoder/assembler -> actual Three attributes',gpu:'no actual WebGL context',actions:stats},null,2));
