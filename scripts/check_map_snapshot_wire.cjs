#!/usr/bin/env node
// 实际控制台二进制解码、身份精度和新旧网页协商的离线回归。
'use strict';
const fs=require('node:fs'), path=require('node:path'), vm=require('node:vm'), assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
const source=fs.readFileSync(path.join(root,'gateways/http/static/js/console.js'),'utf8');
function fn(name){const start=source.indexOf(`      function ${name}(`),end=source.indexOf('\n      function ',start+20);assert.ok(start>=0&&end>start);return source.slice(start,end);}
const window={location:{protocol:'http:',host:'example.invalid'}};
const context=vm.createContext({window,JSON,Number,Uint8Array,Int8Array,ArrayBuffer,DataView,TextDecoder,URL,
  STREAM_CURSOR_MAX_U64:'18446744073709551615',STREAM_TOPICS:['map.frame'],state:{}});
vm.runInContext(fn('normalizedStreamCursor'),context);
window.__nuwaxMapRevision=value=>{const n=context.normalizedStreamCursor(value);return n&&n!=='0'?n:null;};
vm.runInContext(fn('decodeMapBinaryFrame')+'\n'+fn('buildStreamUrl'),context);
function payload(){return {schema_version:2,map_name:'地图',source_name:'owner',source_epoch:'run',frame_id:'map',update_sequence:'9007199254740993',
  occupancy_grid:{map_id:'地图',frame_id:'map',width:3,height:1,resolution_m:.1,
    origin:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}}};}
function bytes(version=3,body=payload(),cells=[0,255,100]){const json=Buffer.from(JSON.stringify({type:'stream.frame',frame:{topic:'map.frame',payload:body}}));
  const out=Buffer.alloc(13+json.length+cells.length);out.write('NUWAXMAP');out[8]=version;out.writeUInt32LE(json.length,9);json.copy(out,13);Buffer.from(cells).copy(out,13+json.length);
  return out.buffer.slice(out.byteOffset,out.byteOffset+out.byteLength);}
let checks=0;function check(name,run){run();checks++;console.log('PASS '+name);}
check('v3 keeps exact owner sequence and packed signed cells',()=>{const f=context.decodeMapBinaryFrame(bytes());assert.equal(f.payload.update_sequence,'9007199254740993');assert.deepEqual(Array.from(f.payload.occupancy_grid.data),[0,-1,100]);});
check('v2 compatibility ignores malformed historical digest without reconstructing it',()=>{const p=payload();delete p.schema_version;p.content_sha256='legacy untrusted';const f=context.decodeMapBinaryFrame(bytes(2,p));assert.equal(f.payload.schema_version,2);assert.ok(!('content_sha256' in f.payload));});
check('future and obsolete binary versions reject',()=>{for(const v of [0,1,4,255])assert.equal(context.decodeMapBinaryFrame(bytes(v)),null);});
check('v3 requires its new snapshot schema',()=>{for(const v of [undefined,1,3]){const p=payload();p.schema_version=v;assert.equal(context.decodeMapBinaryFrame(bytes(3,p)),null);}});
check('full source identity is required independently of legacy checksum',()=>{for(const key of ['source_name','source_epoch','frame_id','update_sequence']){const p=payload();delete p[key];assert.equal(context.decodeMapBinaryFrame(bytes(3,p)),null);}});
check('map and frame identity cannot contradict their grid',()=>{for(const key of ['map_id','frame_id']){const p=payload();p.occupancy_grid[key]='other';assert.equal(context.decodeMapBinaryFrame(bytes(3,p)),null);}});
check('zero negative unsafe numeric and over-u64 revisions reject',()=>{for(const n of [0,'0',-1,'-1',9007199254740992,'18446744073709551616',true]){const p=payload();p.update_sequence=n;assert.equal(context.decodeMapBinaryFrame(bytes(3,p)),null);}});
check('safe integers and decimal text share canonical identity',()=>{assert.equal(window.__nuwaxMapRevision(12),window.__nuwaxMapRevision('12'));assert.equal(window.__nuwaxMapRevision('00012'),null);assert.equal(window.__nuwaxMapRevision('18446744073709551615'),'18446744073709551615');});
check('length and occupancy range constraints survive identity migration',()=>{assert.equal(context.decodeMapBinaryFrame(bytes(3,payload(),[0,100])),null);assert.equal(context.decodeMapBinaryFrame(bytes(3,payload(),[0,101,100])),null);});
check('new client opts into v3 without changing identity-only or cursor parameters',()=>{const url=new URL(context.buildStreamUrl({topics:['localization.pose','map.frame'],mapPayload:'identity',latestStreamCursor:'7'}));assert.equal(url.searchParams.get('map_binary_version'),'3');assert.equal(url.searchParams.get('map_payload'),'identity');assert.equal(url.searchParams.get('after_cursor'),'7');});
const binding={map_name:'地图',source_name:'owner',source_epoch:'run',frame_id:'map',update_sequence:'9007199254740993',available:true};
window.__nuwaxMapBinding=()=>binding;
const semantic=fs.readFileSync(path.join(root,'gateways/http/static/js/semantic-map.js'),'utf8');
const start=semantic.indexOf('  function matchesCurrent()'),end=semantic.indexOf('\n  function drawable',start);
const semanticState={data:{map_binding:{...binding}}};
const semanticContext=vm.createContext({window,state:semanticState});vm.runInContext(semantic.slice(start,end),semanticContext);
check('semantic response identity compares exact revisions and changed owners',()=>{assert.equal(semanticContext.matchesCurrent(),true);semanticState.data.map_binding.update_sequence='9007199254740992';assert.equal(semanticContext.matchesCurrent(),false);semanticState.data.map_binding={...binding,source_epoch:'other'};assert.equal(semanticContext.matchesCurrent(),false);});
check('legacy semantic response without revision cannot authorize current overlay',()=>{semanticState.data.map_binding={...binding};delete semanticState.data.map_binding.update_sequence;assert.equal(semanticContext.matchesCurrent(),false);});
console.log(JSON.stringify({checks,status:'passed',scope:'offline production decoder and binding functions'}));
