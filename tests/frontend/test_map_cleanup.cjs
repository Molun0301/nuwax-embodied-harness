const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const code=fs.readFileSync('gateways/http/static/js/map-cleanup.js','utf8');
const settle=()=>new Promise(r=>setImmediate(r));
function app(){
  class Element {
    constructor(id){this.id=id;this.value='';this.textContent='';this.disabled=false;this.hidden=id==='mapCleanupPanel';this.listeners={};this.children=[];this.clientWidth=600;}
    addEventListener(name,fn){this.listeners[name]=fn;}
    async fire(name,event={}){await this.listeners[name]?.(event);await settle();await settle();}
    append(e){this.children.push(e);if(!this.value)this.value=e.value;}
    replaceChildren(){this.children=[];this.value='';}
    getContext(){return {fillRect(){},fillStyle:''};}
  }
  const ids=['mapCleanupPanel','mapCleanupSource','mapCleanupStrength','mapCleanupPercent','mapCleanupPreview','mapCleanupCancel','mapCleanupApply','mapCleanupNewName','mapCleanupStatus','mapCleanupDetails','mapCleanupCanvas','mapCleanupToggle'];
  const els=Object.fromEntries(ids.map(id=>[id,new Element(id)]));els.mapCleanupStrength.value='50';
  const calls=[];let counter=0,confirm=true,previewGate=null,cancelFail=false;
  const report={preview:{candidate_points:42},dense_before:1000,dense_after:958,offline_frames:190,recovered_voxels:0,offline_note:'全量原始扫描重判'};
  const fetch=async(url,options={})=>{
    const body=options.body?JSON.parse(options.body):null;calls.push({url,body});
    let result;
    if(url==='/api/maps/catalog')result={maps:[{map_name:'original',finalized:true},{map_name:'live',finalized:false}]};
    else if(url==='/api/maps/cleanup'){
      const job={id:'preview-'+(++counter),state:'queued',map_name:body.map_name,strength_percent:body.strength_percent};
      if(previewGate)return await previewGate(job);result=job;
    } else if(url.endsWith('/cancel')){
      if(cancelFail)return {ok:false,json:async()=>({error:{message:'取消请求未成功'}})};
      result={state:'cancelled'};
    } else if(url.endsWith('/apply')) result={id:'done',state:'applied',result_map:body.new_map_name};
    else if(url.includes('/points/'))return {ok:true,arrayBuffer:async()=>new Float32Array([0,0,0,1,1,1]).buffer};
    else result={id:url.split('/').at(-1),state:'ready',map_name:'original',strength_percent:Number(els.mapCleanupStrength.value),report};
    return {ok:true,json:async()=>result};
  };
  const window={confirm:()=>confirm,addEventListener(){},__nuwaxMapBinding:()=>({map_name:'original'}),nuwaxMapPanels:{activate(){},close(){els.mapCleanupPanel.hidden=true;}}};
  vm.runInNewContext(code,{document:{getElementById:id=>els[id],createElement:()=>new Element('option')},window,fetch,Float32Array,Math,Number,Promise,Error,console,
    setTimeout:()=>1,clearTimeout(){}});
  return {els,calls,setConfirm:v=>confirm=v,setGate:v=>previewGate=v,setCancelFail:v=>cancelFail=v};
}
test('only finalized maps; actual counts and continuous percent reach the worker',async()=>{
 const a=app();await a.els.mapCleanupToggle.fire('click');
 assert.deepEqual(a.els.mapCleanupSource.children.map(x=>x.value),['original']);
 a.els.mapCleanupStrength.value='73';await a.els.mapCleanupStrength.fire('input');
 assert.equal(a.els.mapCleanupPercent.textContent,'73%');await a.els.mapCleanupPreview.fire('click');
 assert.equal(a.calls.find(x=>x.url==='/api/maps/cleanup').body.strength_percent,73);
 assert.match(a.els.mapCleanupStatus.textContent,/42/);assert.match(a.els.mapCleanupStatus.textContent,/1,000 → 958/);
 assert.equal(a.els.mapCleanupApply.disabled,false);
 a.els.mapCleanupStrength.value='74';await a.els.mapCleanupStrength.fire('input');
 assert.equal(a.els.mapCleanupApply.disabled,true);assert.equal(a.els.mapCleanupCanvas.hidden,true);assert.ok(a.calls.some(x=>x.url.endsWith('/cancel')));
});
test('confirmation is explicit; apply saves a differently named copy without activation',async()=>{
 const a=app();await a.els.mapCleanupToggle.fire('click');await a.els.mapCleanupPreview.fire('click');
 a.setConfirm(false);await a.els.mapCleanupApply.fire('click');assert.ok(!a.calls.some(x=>x.url.endsWith('/apply')));
 a.setConfirm(true);a.els.mapCleanupNewName.value='cleaned';await a.els.mapCleanupApply.fire('click');
 const apply=a.calls.find(x=>x.url.endsWith('/apply'));assert.equal(apply.body.confirmed,true);assert.equal(apply.body.new_map_name,'cleaned');
 assert.match(a.els.mapCleanupStatus.textContent,/原图 original 保留/);assert.ok(!a.calls.some(x=>x.url.includes('/activate')));
});
test('a stale preview response cannot re-enable apply after slider changes',async()=>{
 const a=app();await a.els.mapCleanupToggle.fire('click');let resolve;
 a.setGate(job=>new Promise(r=>{resolve=()=>r({ok:true,json:async()=>job});}));
 const pending=a.els.mapCleanupPreview.fire('click');await settle();a.els.mapCleanupStrength.value='100';await a.els.mapCleanupStrength.fire('input');
 resolve();await pending;assert.equal(a.els.mapCleanupApply.disabled,true);assert.ok(a.calls.some(x=>x.url.endsWith('/cancel')));
});
test('failed cancellation is not reported as a completed cancellation',async()=>{
 const a=app();await a.els.mapCleanupToggle.fire('click');await a.els.mapCleanupPreview.fire('click');a.setCancelFail(true);
 await a.els.mapCleanupCancel.fire('click');assert.match(a.els.mapCleanupStatus.textContent,/取消请求未成功/);
 assert.equal(a.els.mapCleanupApply.disabled,true);assert.equal(a.els.mapCleanupCancel.disabled,false);
 a.setCancelFail(false);await a.els.mapCleanupCancel.fire('click');assert.match(a.els.mapCleanupStatus.textContent,/已请求取消/);
 assert.equal(a.calls.filter(x=>x.url.endsWith('/cancel')).length,2);
});
