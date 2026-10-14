'use strict';
const fs = require('fs'), vm = require('vm'), assert = require('assert');
const path = require('path');
const sourcePath = path.resolve(__dirname, '../../gateways/http/static/js/console.js');
const finalSource = fs.readFileSync(sourcePath, 'utf8');
assert.equal(finalSource.split('const remaining = buffer.subarray(eoi + 2);').length, 2);
assert.equal(finalSource.split('            if (parsedAny) buffer = buffer.slice();\n').length, 2);
// Reconstruct the stage-one baseline; the candidate must match final production source exactly.
const source = finalSource.replace('const remaining = buffer.subarray(eoi + 2);', 'const remaining = buffer.slice(eoi + 2);')
 .replace('            if (parsedAny) buffer = buffer.slice();\n', '');
assert.equal(source.replace('const remaining = buffer.slice(eoi + 2);', 'const remaining = buffer.subarray(eoi + 2);')
 .replace('            if (latestFrameBytes) {', '            if (parsedAny) buffer = buffer.slice();\n            if (latestFrameBytes) {'), finalSource);
const section = (start,end) => source.slice(source.indexOf(start), source.indexOf(end, source.indexOf(start)));
const consume = section('      async function consumeMjpegStream(', '      function appendBytes(');
const parser = section('      function appendBytes(', '      async function renderDecodedVideoFrame(');
const renderer = section('      async function renderDecodedVideoFrame(', '      async function displayVideoBlob(');
const sourceTime = section('      function videoSourceTime(', '      function videoFrameIdentity(');
assert(consume && parser && sourceTime);
assert.equal(parser.split('const frame = buffer.subarray(soi, eoi + 2);').length, 2);
assert.equal(consume.split('renderDecodedVideoFrame(latestFrameBytes.slice(), generation,').length, 2);
function makeArm(candidate) {
  const sandbox = {Uint8Array, TextDecoder, Number, Date, Error, decodeURIComponent,
    MJPEG_MAX_BUFFER_BYTES: 16*1024*1024, VIDEO_STREAM_INITIAL_RECONNECT_MS: 250};
  vm.createContext(sandbox);
  vm.runInContext(sourceTime + (candidate ? parser.replace('const remaining = buffer.slice(eoi + 2);','const remaining = buffer.subarray(eoi + 2);') : parser)
    + (candidate ? consume.replace('            if (latestFrameBytes) {','            if (parsedAny) buffer = buffer.slice();\n            if (latestFrameBytes) {') : consume), sandbox);
  return sandbox;
}
const arms = [makeArm(false),makeArm(true)];
function jpeg(size, tag=85) { const b=new Uint8Array(size).fill(tag); b[0]=255;b[1]=216;b[size-2]=255;b[size-1]=217;return b; }
function packet(data, seq, opts={}) {
 const header = `--frame\r\nContent-Type: image/jpeg\r\n${opts.noLength?'':`Content-Length: ${opts.length ?? data.length}\r\n`}X-Camera-Id: ${opts.camera ?? '%E5%89%8D%2Fcamera'}\r\nX-Frame-Sequence: ${seq}\r\nX-Frame-Timestamp: 2026-10-14T01:00:00.123456+00:00\r\nX-Frame-Timestamp-Source: video_relay.write_time\r\n\r\n`;
 return concat([Buffer.from(header), data,Buffer.from('\r\n')]);
}
function concat(parts) {const out=new Uint8Array(parts.reduce((n,x)=>n+x.length,0));let at=0;for(const x of parts){out.set(x,at);at+=x.length;}return out;}
function split(b,sizes) {const out=[];let at=0,i=0;while(at<b.length){const n=sizes[i++%sizes.length];out.push(b.slice(at,at+n));at+=n;}return out;}
async function run(arm,chunks,options={}) {
 const shown=[], captured=[], lifecycle={cancel:0,release:0,reconnect:0};let pos=0;
 arm.state={videoStreamGeneration:1,videoStreamBytes:0,videoFrames:0,videoDecodeErrors:0};
 arm.fetch=async()=>({ok:true,body:{getReader:()=>({read:async()=>{if(options.onRead)options.onRead(pos,arm);return pos<chunks.length?{value:chunks[pos++],done:false}:{done:true};},cancel:async()=>{lifecycle.cancel++;},releaseLock:()=>{lifecycle.release++;}})}});
 arm.setVideoStreamStatus=()=>{};arm.scheduleVideoStreamReconnect=()=>{lifecycle.reconnect++;};
 arm.renderDecodedVideoFrame=async(frame,generation,identity)=>{captured.push(frame);shown.push({bytes:Array.from(frame),identity:JSON.parse(JSON.stringify(identity)),generation});arm.state.videoFrames++;if(options.onRender)await options.onRender(frame,arm);};
 let error=null;try{await arm.consumeMjpegStream('fixture://mjpeg',{aborted:!!options.aborted},1);}catch(e){error=e.message;}
 return {shown,captured,lifecycle,error,decodeErrors:arm.state.videoDecodeErrors};
}
function publicResult(r){return {shown:r.shown,lifecycle:r.lifecycle,error:r.error,decodeErrors:r.decodeErrors};}
let checks=0;
async function parity(name,chunks,options={}) {const a=await run(arms[0],chunks,options),b=await run(arms[1],chunks,options);assert.deepStrictEqual(publicResult(a),publicResult(b),name);checks++;return b;}
async function renderContracts() {
 // Execute the actual rendering admission/generation function with a fake display boundary.
 // Node Blob gives a real byte snapshot; this does not exercise browser image decoding.
 for (const stale of ['none','entry','display']) {
   const a=makeArm(true), revoked=[], shown=[];
   a.Blob=Blob;a.URL={revokeObjectURL:url=>revoked.push(url)};
   a.state={videoStreamGeneration:stale==='entry'?2:1,videoFrames:0,videoDecodedBytes:0};
   a.els={videoFrame:{naturalWidth:2,naturalHeight:2,hidden:true},videoEmpty:{hidden:false}};
   a.recordVideoFrameMetric=()=>{};a.scheduleVideoDetectionDraw=()=>{};
   a.displayVideoBlob=async(blob)=>{shown.push(Array.from(new Uint8Array(await blob.arrayBuffer())));if(stale==='display')a.state.videoStreamGeneration=2;return 'blob:fixture';};
   vm.runInContext(renderer,a);
   const frame=jpeg(64);await a.renderDecodedVideoFrame(frame,1,{sequence:'7'});
   assert.equal(a.state.videoFrames,stale==='none'?1:0);
   assert.equal(shown.length,stale==='entry'?0:1);
   assert.deepStrictEqual(revoked,stale==='display'?['blob:fixture']:[]);
   if(stale==='none'){assert.deepStrictEqual(shown[0],Array.from(frame));assert.equal(a.state.videoDisplayedIdentity.sequence,'7');}
   checks++;
 }
}
async function contracts(){
 const j=jpeg(4096), embedded=jpeg(1024);embedded[32]=255;embedded[33]=217;
 const multi=concat([packet(j,1),packet(embedded,2),packet(j,3),packet(j,4).slice(0,500)]);
 for(const sizes of [[multi.length],[1],[2,3,7,31],[16383,1,2048],[4096]])await parity('fragmentation '+sizes,split(multi,sizes));
 for(const n of [1,4,8]){const r=await parity('complete '+n,[concat(Array.from({length:n},(_,i)=>packet(j,i+1)))]);assert.equal(r.shown.length,1);assert.equal(r.shown[0].identity.sequence,String(n));}
 await parity('empty',[]);await parity('no complete',[packet(j,1).slice(0,500)]);
 await parity('no SOI prefix trim',[new Uint8Array(17000).fill(65)]);
 await parity('no length legacy',[packet(j,1,{noLength:true})]);
 await parity('invalid camera',[packet(j,1,{camera:'%GG'})]);
 for(const length of ['3','16777217','9007199254740992']){const r=await parity('bad length '+length,[packet(j,1,{length})]);assert(r.error);}
 // Both arms retain independent selected bytes even when source storage is changed at async boundary.
 for(const arm of arms){const input=concat([packet(j,1),packet(embedded,2)]);const r=await run(arm,[input],{onRender:async(frame)=>{input.fill(0);await Promise.resolve();assert.deepStrictEqual(Array.from(frame),Array.from(embedded));}});assert.equal(r.captured.length,1);assert.deepStrictEqual(Array.from(r.captured[0]),Array.from(embedded));checks++;}
 await parity('generation before read',[packet(j,1)],{onRead:(i,a)=>{if(i===0)a.state.videoStreamGeneration=2;}});
 await parity('generation during render',[packet(j,1),packet(j,2)],{onRender:async(f,a)=>{a.state.videoStreamGeneration=2;}});
 await parity('decode error',[packet(j,1)],{onRender:async()=>{throw new Error('fixture decode failure');}});
 await parity('aborted decode',[packet(j,1)],{aborted:true,onRender:async()=>{throw new Error('fixture abort');}});
 // Direct parser outputs/remaining match, including bytes never selected for display.
 for(const input of [multi,j,new Uint8Array(0),packet(j,1).slice(0,100)]){const results=arms.map(a=>{const p=a.extractNextJpeg(input);return p&&{frame:p.frame&&Array.from(p.frame),remaining:Array.from(p.remaining),identity:p.identity&&JSON.parse(JSON.stringify(p.identity))};});assert.deepStrictEqual(results[0],results[1]);checks++;}
}
// Timing stays synchronous: same production parser/append loop and selected-frame ownership boundary,
// excluding fetch, async/DOM/Blob decode, network, fixture construction and copy instrumentation.
function parseBatch(arm,input,candidate) {let buffer=arm.appendBytes(new Uint8Array(0),input),last=null,identity=null;while(true){const p=arm.extractNextJpeg(buffer);if(!p)break;if(!p.frame){buffer=p.remaining;break;}last=p.frame;identity=p.identity;buffer=p.remaining;}if(candidate&&last)buffer=buffer.slice();if(last)last=last.slice();return {last,identity,buffer};}
function countCopies(arm,input,candidate){const orig=Uint8Array.prototype.slice;let bytes=0,calls=0;Uint8Array.prototype.slice=function(...args){const out=orig.apply(this,args);bytes+=out.byteLength;calls++;return out;};try{parseBatch(arm,input,candidate);}finally{Uint8Array.prototype.slice=orig;}return {bytes,calls};}
function quantile(v,q){const s=[...v].sort((a,b)=>a-b);return s[Math.floor((s.length-1)*q)];}

async function tailContracts() {
 const a=jpeg(4096,65),b=jpeg(2048,66),c=jpeg(8192,67);
 for(const cut of [32,200,800])for(const arm of arms){
  const next=packet(c,3),head=concat([packet(a,1),packet(b,2),next.slice(0,cut)]);
  const follow=split(next.slice(cut),[1,7,113,2048]);let once=false;
  const r=await run(arm,[head,...follow],{onRender:async(frame)=>{if(!once){once=true;head.fill(0);await Promise.resolve();assert.deepStrictEqual(Array.from(frame),Array.from(b));}}});
  assert.equal(r.error,null);assert.equal(r.shown.length,2);assert.deepStrictEqual(r.shown[1].bytes,Array.from(c));
  assert.deepStrictEqual(r.shown.map(x=>x.identity.sequence),['2','3']);assert.equal(r.lifecycle.cancel,1);assert.equal(r.lifecycle.release,1);checks++;
 }
 // Mutate after selection, while the next read is awaited rather than during display.
 for(const arm of arms){const next=packet(c,3),head=concat([packet(a,1),packet(b,2),next.slice(0,500)]);const r=await run(arm,[head,next.slice(500)],{onRead:(index)=>{if(index===1)head.fill(0);}});assert.deepStrictEqual(r.shown[1].bytes,Array.from(c));checks++;}
 for(const tail of [new Uint8Array(0),new Uint8Array(20000).fill(65),concat([new Uint8Array(20000).fill(65),packet(c,3).slice(0,500)])]){
  const r=await parity('empty or trimmed tail',[concat([a,tail])]);assert.equal(r.error,null);
 }
 // Trimming an incomplete frame must still preserve its header across mutation/fragmented read.
 for(const arm of arms){const next=packet(c,3),head=concat([packet(a,1),new Uint8Array(20000).fill(65),next.slice(0,500)]);const r=await run(arm,[head,...split(next.slice(500),[3,31,900])],{onRender:async()=>{head.fill(0);}});assert.equal(r.shown.length,2);assert.deepStrictEqual(r.shown[1].bytes,Array.from(c));assert.equal(r.shown[1].identity.sequence,'3');checks++;}
 // No complete frame: retain historical borrowing for small prefixes, historical detached trimming for large ones.
 for(const prefix of [32,20000]){const next=packet(c,3),chunks=[concat([new Uint8Array(prefix).fill(65),next.slice(0,500)]),next.slice(500)];await parity('no complete then fragmented '+prefix,chunks);}
 for(const arm of arms){const image=jpeg(512*1024,68),next=packet(image,3),cut=480*1024,head=concat([packet(a,1),new Uint8Array(20000).fill(65),next.slice(0,cut)]);const r=await run(arm,[head,next.slice(cut)],{onRender:async()=>{head.fill(0);}});assert.equal(r.shown.length,2);assert.deepStrictEqual(r.shown[1].bytes,Array.from(image));assert.equal(r.shown[1].identity.sequence,'3');checks++;}
 await parity('huge prefix bounded trim',[concat([new Uint8Array(16*1024*1024+1).fill(65),packet(a,1)])]);
 const overflow=await parity('huge prefix no SOI',[new Uint8Array(16*1024*1024+1).fill(65)]);assert(overflow.error);
}
function controlInputs(){const a=jpeg(4096),next=packet(jpeg(8192),2);const inputs={
 'empty':new Uint8Array(0),'no_complete_small':next.slice(0,500),
 'no_complete_prefix_trim':concat([new Uint8Array(20000).fill(65),next.slice(0,500)]),
 'single_empty_tail':a,'single_partial_tail':concat([packet(a,1),next.slice(0,500)]),
 'single_garbage_trim':concat([packet(a,1),new Uint8Array(20000).fill(65)]),
 'single_incomplete_prefix_trim':concat([packet(a,1),new Uint8Array(20000).fill(65),next.slice(0,500)]),
 'single_large_incomplete_prefix_trim':concat([packet(a,1),new Uint8Array(20000).fill(65),packet(jpeg(8*1024*1024),2).slice(0,4*1024*1024)])};
 return inputs;}
function controlCounts(){return Object.entries(controlInputs()).map(([name,input])=>({name,input_bytes:input.length,copies:arms.map((a,i)=>countCopies(a,input,!!i))}));}

async function main(){await contracts();await renderContracts();await tailContracts();const output={source:sourcePath,source_window:'Final production tail candidate; stage-one b97bdd9 baseline reconstructed by reversing exactly two tail changes',checks,node:process.version,scope:'Real-source parser and mocked consumer contracts; timing excludes actual browser/Blob/GPU/network and async consumer',cohorts:[],controls:controlCounts()};if(process.argv.includes('--contracts-only')){console.log(JSON.stringify(output,null,2));return;}
 for(const size of [4096,65536,262144])for(const frames of [1,4,8]){const input=concat(Array.from({length:frames},(_,i)=>packet(jpeg(size,65+i),i+1)));const copies=arms.map((a,i)=>countCopies(a,input,!!i));assert(copies[1].bytes<=copies[0].bytes);const samples=[[],[]];for(let i=0;i<20;i++)arms.forEach((a,n)=>parseBatch(a,input,!!n));for(let i=0;i<60;i++){const order=i%2?[1,0]:[0,1];for(const n of order){const start=process.hrtime.bigint();const r=parseBatch(arms[n],input,!!n);const elapsed=Number(process.hrtime.bigint()-start)/1e6;assert.equal(r.last.length,size);samples[n].push(elapsed);}}output.cohorts.push({jpeg_bytes:size,frames,input_bytes:input.length,copies,median_ms:samples.map(s=>quantile(s,.5)),p95_ms:samples.map(s=>quantile(s,.95)),samples_ms:samples});}
 for(const [name,input] of Object.entries(controlInputs())){const c=output.controls.find(c=>c.name===name),samples=[[],[]];for(let i=0;i<20;i++)arms.forEach((a,n)=>parseBatch(a,input,!!n));for(let i=0;i<60;i++)for(const n of (i%2?[1,0]:[0,1])){const start=process.hrtime.bigint();parseBatch(arms[n],input,!!n);samples[n].push(Number(process.hrtime.bigint()-start)/1e6);}c.median_ms=samples.map(s=>quantile(s,.5));c.p95_ms=samples.map(s=>quantile(s,.95));c.samples_ms=samples;}
 console.log(JSON.stringify(output,null,2));}
main().catch(e=>{console.error(e);process.exitCode=1;});
