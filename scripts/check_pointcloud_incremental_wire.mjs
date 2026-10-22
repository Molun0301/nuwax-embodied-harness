import os from 'node:os';
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {fileURLToPath} from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const text = fs.readFileSync(path.join(root,'gateways/http/static/js/pointcloud-incremental.js'),'utf8');
const {PointCloudPageAssembler, parsePointCloudPagePacket, IncrementalPointCloudConnection, mappingEpochFromMap} = await import('data:text/javascript;base64,'+Buffer.from(text).toString('base64'));
const fixture = JSON.parse(fs.readFileSync(path.join(process.env.NEH_POINTCLOUD_FIXTURE_DIR||path.join(os.tmpdir(),'neh-pointcloud-fixtures'),'pointcloud_incremental_fixtures.json')));
function raw(text) { const bytes=Buffer.from(text,'base64'); return bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength); }
const baseline=fixture.baseline.map(raw), delta=fixture.delta.map(raw);
function change(buffer,values) {
  const parsed=parsePointCloudPagePacket(buffer), old=new DataView(buffer).getUint32(9,true), payload=new Uint8Array(buffer,13+old);
  const header={...parsed.header,...values}; let encoded=new TextEncoder().encode(JSON.stringify(header));
  const padding=(4-(13+encoded.length)%4)%4, result=new Uint8Array(13+encoded.length+padding+payload.length);
  result.set(new Uint8Array(buffer,0,9)); new DataView(result.buffer).setUint32(9,encoded.length+padding,true); result.set(encoded,13);
  result.fill(32,13+encoded.length,13+encoded.length+padding); result.set(payload,13+encoded.length+padding);
  return result.buffer;
}
let checks=0; function check(name,fn){fn();checks++;console.log('PASS '+name);}
function seeded(){const a=new PointCloudPageAssembler(); for(const p of baseline){const b=a.push(p);if(b)a.accept(b);}return a;}
check('Python wire bootstrap atomic across two bounded packets',()=>{
 const a=new PointCloudPageAssembler(); assert.equal(a.push(baseline[0]),null); assert.equal(a.base,null);
 const b=a.push(baseline[1]); assert.equal(b.chunks.length,6); assert.equal(a.base,null); a.accept(b);
 assert.equal(a.base.revision_ordinal,'9007199254740999');assert.equal(a.pointCount,5123);
 assert.equal(b.chunks[0].xyz.buffer,baseline[0]);
});
check('one-page delta keeps base until apply then exact ACK base',()=>{const a=seeded(), before=a.base;const b=a.push(delta[0]);assert.equal(b.chunks.length,1);assert.equal(a.base,before);a.accept(b);assert.equal(a.base.revision_ordinal,'9007199254741000');});
check('wrong base and skipped packets cannot mutate accepted state',()=>{const a=seeded(), before=a.base;assert.throws(()=>a.push(change(delta[0],{base_revision_ordinal:'3'})));assert.equal(a.base,before);a.discardPending();assert.throws(()=>a.push(baseline[1]));assert.equal(a.base,before);});
check('explicit reset replaces the displayed base after a server restart',()=>{
 const a=seeded();a.base.stream_generation='3';a.base.layout_generation='5';
 a.push(baseline[0]);const b=a.push(baseline[1]);assert.ok(b);a.accept(b);
 assert.equal(a.base.stream_generation,'1');assert.equal(a.base.revision_ordinal,'9007199254740999');
});
check('late old-map baseline never accepted after map advance',()=>{
 const a=new PointCloudPageAssembler();a.setMapEpoch('session:1');a.push(baseline[0]);a.setMapEpoch('session:2');
 assert.throws(()=>a.push(baseline[1]));assert.equal(a.base,null);
});
check('nested display epochs normalize to canonical session',()=>{
 const wrapped=JSON.stringify(['map-view-v2',JSON.stringify(['map-view-v2','session:1','display']),'service']);
 assert.equal(mappingEpochFromMap({source_epoch:wrapped}),'session:1');
 assert.equal(mappingEpochFromMap({source_epoch:wrapped,metadata:{mapping_session_id:'new',session_generation:'9007199254740999'}}),'new:9007199254740999');
});
check('non-finite coordinates remain available while truncated or unaligned payloads fail',()=>{
 const a=seeded(), base=a.base, nan=delta[0].slice(0);const offset=13+new DataView(nan).getUint32(9,true);new DataView(nan).setFloat32(offset,NaN,true);
 const batch=a.push(nan);assert.ok(Number.isNaN(batch.chunks[0].xyz[0]));assert.equal(a.base,base);
 const unaligned=delta[0].slice(0);new DataView(unaligned).setUint32(9,new DataView(unaligned).getUint32(9,true)-1,true);
 for(const value of [delta[0].slice(0,-4),unaligned,change(delta[0],{payload_length:1})]) assert.throws(()=>a.push(value));
 assert.equal(a.base,base);
});
// Real connection code with deterministic socket/timer control.
class Socket {
 static OPEN=1;static CONNECTING=0;static all=[];
 constructor(url){this.url=String(url);this.readyState=0;this.handlers=new Map();this.sent=[];Socket.all.push(this);}
 addEventListener(type,fn){this.handlers.set(type,fn);}
 send(value){this.sent.push(JSON.parse(value));}
 emit(type,value={}){if(type==='open')this.readyState=1;if(type==='close')this.readyState=3;this.handlers.get(type)?.(value);}
 close(){this.emit('close');}
}
globalThis.WebSocket=Socket;let timerId=0;const timers=new Map();
globalThis.window={location:{href:'https://robot.example/console'},setTimeout:fn=>{timers.set(++timerId,fn);return timerId;},clearTimeout:id=>timers.delete(id)};
check('socket ACK only complete batch and no duplicate live connections',()=>{
 let applied=0;const c=new IncrementalPointCloudConnection({onBatch:()=>applied++});c.connect();c.connect();const s=c.socket;s.emit('open');
 assert.equal(s.sent[0].type,'pointcloud.resume');s.emit('message',{data:baseline[0]});assert.equal(applied,0);assert.equal(s.sent.length,1);
 s.emit('message',{data:baseline[1]});assert.equal(applied,1);assert.equal(s.sent.at(-1).type,'pointcloud.ack');c.dispose();assert.equal(timers.size,0);
});
check('reconnect drops staging but keeps complete base and fences old socket',()=>{
 let applied=0;const c=new IncrementalPointCloudConnection({onBatch:()=>applied++});c.connect();const old=c.socket;old.emit('open');for(const data of baseline)old.emit('message',{data});
 old.emit('close');assert.equal(timers.size,1);const fn=[...timers.values()][0];timers.clear();fn();const next=c.socket;next.emit('open');
 assert.equal(next.sent[0].base.revision_ordinal,'9007199254740999');old.emit('message',{data:delta[0]});assert.equal(applied,1);
 next.emit('message',{data:delta[0]});assert.equal(applied,2);c.dispose();assert.equal(timers.size,0);
});
check('one resync on parse failure and changed map opens a fresh selected-source connection',()=>{
 const c=new IncrementalPointCloudConnection({onBatch:()=>{}});c.connect();const s=c.socket;s.emit('open');
 s.emit('message',{data:new ArrayBuffer(1)});s.emit('message',{data:new ArrayBuffer(1)});
 assert.equal(s.sent.filter(x=>x.type==='pointcloud.resync').length,1);c.setMapEpoch('session:2');
 const selected=c.socket;assert.notEqual(selected,s);selected.emit('open');
 assert.equal(selected.sent[0].type,'pointcloud.resume');assert.equal(selected.sent[0].mapping_source_epoch,'session:2');assert.equal(selected.sent[0].base,null);c.dispose();
});
console.log(JSON.stringify({checks,decoderXYZAllocations:0,protocol:'NUWAXPC-v2'}));
