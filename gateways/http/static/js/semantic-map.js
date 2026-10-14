// 语义是当前地图的一种视图和可选图层，复用控制台坐标、主题与公共能力。
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const panel = $('semanticRegionPanel');
  if (!panel) return;
  const labels = { candidate: '候选', auto_labeled: '自动标签', reviewed: '已审核', corrected: '已修正', rejected: '已否定', stale: '已过期' };
  const terminal = new Set(['succeeded', 'failed', 'cancelled', 'timeout']);
  const palette = ['#4b9c86', '#648fb6', '#a18aba', '#b79461', '#719783', '#828eb4'];
  const state = { data: null, service: null, selected: null, overlays: { '2d': false, '3d': false },
    paths: [], context: null, contextKey: '', request: null, generation: 0, streamVersion: 0,
    task: null, taskTimer: null, statusTimer: null, reloadTimer: null, disposed: false, busy: false, error: '', renderKey: '', detailKey: '' };
  const controllers = new Set();
  const mode = () => window.__nuwaxMapViewMode?.() || '2d';
  // 区域工具打开期间始终显示区域，便于点选与核对；图层开关只决定常态显示。
  const visible = () => mode() === 'semantic' || state.overlays[mode()] === true || !panel.hidden;
  const interested = () => visible() || !panel.hidden;
  const node = (tag, text, cls) => { const n = document.createElement(tag); if (text !== undefined) n.textContent = String(text); if (cls) n.className = cls; return n; };
  const bindingKey = b => JSON.stringify([b?.map_name, b?.source_name, b?.source_epoch, b?.frame_id, window.__nuwaxMapRevision?.(b?.update_sequence)]);
  const scopeKey = b => JSON.stringify([b?.map_name, b?.source_epoch, b?.frame_id]);
  const regions = () => Array.isArray(state.data?.regions) ? state.data.regions : [];
  const name = (r, index) => r.primary_label || '区域 ' + (index + 1);
  const statusName = r => r.metadata?.label_locked ? '已命名' : labels[r.review_status] || '未知';
  function matchesCurrent() {
    const current = window.__nuwaxMapBinding?.();
    // 请求暂时失败时保留同一地图的已读标注，错误仍在面板显示。
    if (!current?.available || !state.data?.map_binding) return false;
    const bound = state.data.map_binding;
    return ['map_name', 'frame_id', 'source_name', 'source_epoch'].every(k => current[k] != null && current[k] === bound[k])
      && window.__nuwaxMapRevision?.(current.update_sequence) != null
      && window.__nuwaxMapRevision?.(current.update_sequence) === window.__nuwaxMapRevision?.(bound.update_sequence);
  }
  function drawable() { return matchesCurrent() ? regions().filter(r => r.review_status !== 'stale' && r.review_status !== 'rejected' && r.frame_id === state.data.map_binding.frame_id && r.boundary?.frame_id === r.frame_id) : []; }
  function color(region) {
    return ({office:'#4b9c86',meeting_room:'#9580b0',corridor:'#648fb6',kitchen:'#b79461',storage:'#89916b'})[region.metadata?.category] || palette[Math.max(0, regions().indexOf(region)) % palette.length];
  }
  async function invoke(capability, args = {}) {
    const controller = new AbortController(); controllers.add(controller);
    const timer = setTimeout(() => controller.abort(), 20000);
    try {
      const response = await fetch('/api/capabilities/' + encodeURIComponent(capability) + '/invoke', {
        method: 'POST', credentials: 'same-origin', signal: controller.signal,
        headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ arguments: args })
      });
      const body = await response.json();
      if (!response.ok) { const error = new Error(body.error?.message || '请求未完成，请重新读取。'); error.status = response.status; throw error; }
      return body.result ?? body;
    } finally { clearTimeout(timer); controllers.delete(controller); }
  }
  function message(value, error = false) { $('semanticRegionStatus').textContent = value; $('semanticRegionStatus').dataset.error = String(error); }
  function ui() {
    const semantic = mode() === 'semantic';
    if ($('semanticRegionLegend')) $('semanticRegionLegend').hidden = !visible();
    panel.classList.toggle('has-selection', Boolean(state.selected));
    // 区域详情以检查卡显示在地图右上角，仅在区域工具打开且已选中区域时出现。
    $('semanticRegionInspector').hidden = panel.hidden || !regions().some(r => r.scene_space_id === state.selected);
    $('semanticRegionLayerControl').hidden = false;
    $('showSemanticRegions').checked = semantic || state.overlays[mode()] === true;
    $('showSemanticRegions').disabled = semantic;
    $('semanticRegionPanelToggle').setAttribute('aria-expanded', String(!panel.hidden));
    $('semanticRegionPanelToggle').classList.toggle('active', !panel.hidden);
    $('semanticRegionCount').textContent = state.data ? String(regions().length) : '—';
    const running = state.service?.running || state.service?.busy;
    $('semanticRegionStart').hidden = Boolean(running);
    $('semanticRegionStop').hidden = !running;
    $('semanticRegionRefresh').disabled = state.busy || Boolean(state.task) || state.service?.enabled === false;
    $('semanticRegionStart').disabled = state.busy || state.service?.enabled === false;
    $('semanticRegionStop').disabled = state.busy;
    $('semanticRegionApply').disabled = state.busy || Boolean(state.task);
  }
  function emitLayer() {
    state.paths = [];
    const list = visible() ? drawable().map(region => ({ ...region, display_color: color(region), display_index: regions().indexOf(region) + 1 })) : [];
    window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-regions', { detail: {
      regions: list, binding: state.data?.map_binding || null, visible: mode() === '3d' && visible(), selectedId: state.selected,
      theme: {surface:getComputedStyle(document.documentElement).getPropertyValue('--surface').trim(),ink:getComputedStyle(document.documentElement).getPropertyValue('--ink').trim()}
    }}));
    window.__nuwaxMapRequestDraw?.();
  }
  function openPanel(open) {
    if (open) window.nuwaxMapPanels?.activate('semanticRegionPanel');
    panel.hidden = !open;
    ui(); emitLayer();
    if (open) { window.dispatchEvent(new CustomEvent('map:semantic-panel-opened')); load(); }
  }
  function select(id) {
    if (!regions().some(r => r.scene_space_id === id)) return;
    state.selected = id;
    if (panel.hidden) openPanel(true);
    renderList(); renderDetail(); ui(); emitLayer();
    $('semanticRegionList').querySelector('[aria-pressed="true"]')?.scrollIntoView({block:'nearest',inline:'nearest'});
  }
  function renderList() {
    const list = $('semanticRegionList'), query = $('semanticRegionSearch').value.trim().toLowerCase();
    const key = JSON.stringify([regions().map(r => [r.scene_space_id,r.primary_label,r.review_status]),state.selected,query]);
    if (key === state.renderKey) return;
    state.renderKey = key;
    const top = list.scrollTop;
    list.replaceChildren();
    regions().forEach((r,index) => {
      if (query && !name(r,index).toLowerCase().includes(query)) return;
      const button = node('button', undefined, 'map-semantic-row'); button.type = 'button';
      button.dataset.regionId = r.scene_space_id; button.setAttribute('aria-pressed', String(r.scene_space_id === state.selected));
      button.append(node('span', r.primary_label ? (index + 1) + ' · ' + r.primary_label : name(r,index), 'label'), node('span',statusName(r),'tag'));
      button.addEventListener('click',() => select(r.scene_space_id)); list.append(button);
    });
    if (!list.children.length) list.append(node('p',query ? '没有匹配的区域。' : '当前地图尚无语义区域。','muted'));
    list.scrollTop = top;
  }
  function renderDetail() {
    const target = $('semanticRegionDetail'), r = regions().find(r => r.scene_space_id === state.selected);
    const key = JSON.stringify([r?.scene_space_id,r?.metadata?.scene_revision,r?.primary_label,r?.review_status,state.busy,matchesCurrent()]);
    if (key === state.detailKey) return;
    state.detailKey = key;
    const draft = target.querySelector('input')?.value;
    const oldId = target.dataset.regionId, oldRevision = target.dataset.revision;
    const focus = target.contains(document.activeElement) && document.activeElement?.tagName === 'INPUT';
    const scroll = target.scrollTop;
    target.replaceChildren();
    if (!r) { target.append(node('p','点选地图中的区域查看名称与证据。','muted')); return; }
    target.dataset.regionId = r.scene_space_id; target.dataset.revision = String(r.metadata?.scene_revision);
    target.append(node('h3',name(r,regions().indexOf(r))));
    const fields = node('dl');
    [['状态',statusName(r)],['面积',r.boundary?.area_m2 == null ? '未知' : r.boundary.area_m2.toFixed(2) + ' 平方米'],['区域修订',r.metadata?.scene_revision || '—']].forEach(([k,v]) => fields.append(node('dt',k),node('dd',v)));
    target.append(fields);
    const focusButton = node('button','聚焦此区域','tool'); focusButton.type='button';
    focusButton.addEventListener('click',() => window.__nuwaxMapFocusRegion?.(r)); target.append(focusButton);
    const form=node('form'),input=node('input'),save=node('button','保存名称','tool');
    input.type='text'; input.maxLength=120; input.required=true; input.setAttribute('aria-label','区域名称');
    input.placeholder='为区域命名'; input.value=oldId===r.scene_space_id && oldRevision===String(r.metadata?.scene_revision) && draft!==undefined ? draft : r.primary_label || '';
    save.type='submit'; save.disabled=state.busy || !matchesCurrent() || ['stale','rejected'].includes(r.review_status);
    form.append(input,save); target.append(form);
    form.addEventListener('submit',event => { event.preventDefault(); const value=input.value.trim(); if (!value) return;
      action(async() => {
        try { await invoke('update_scene_space_label',{scene_space_id:r.scene_space_id,map_name:r.map_name,expected_revision:r.metadata.scene_revision,label:value,request_id:'semantic_'+(crypto.randomUUID?.() || Date.now()+'_'+Math.random().toString(36).slice(2))}); await load(); message('名称已保存，三个视图共用此标注。'); }
        catch(error) { if(error.status===409){await load();throw new Error('地图或区域已改变，已重新读取。请核对后再次保存。');} throw error; }
      });
    });
    const relationButton=node('button','查看区域关系','tool');relationButton.type='button';
    relationButton.addEventListener('click',()=>window.dispatchEvent(new CustomEvent('scenegraph:open',{detail:{node_id:r.scene_space_id}})));
    target.append(relationButton);
    target.append(node('h4','识别结果'));
    const candidates=r.candidate_labels || [];
    if(candidates.length){
      target.append(node('p','模型分数尚未校准为房间识别准确率。','muted small'));
      const table=node('table'); candidates.forEach(c=>{const tr=node('tr');tr.append(node('td',c.label),node('td',Number(c.confidence).toFixed(3)));table.append(tr);});target.append(table);
      target.append(node('p','模型：'+[...new Set(candidates.map(c=>c.source_model).filter(Boolean))].join('、'),'muted small'));
    } else target.append(node('p','缺少识别证据，当前仅为几何候选。','muted small'));
    target.append(node('h4','观察证据'));
    const samples=r.observation_samples || [];
    if(!samples.length)target.append(node('p','暂无已关联图像和观察。','muted small'));
    samples.forEach(sample=>{const box=node('div',undefined,'map-semantic-evidence');box.append(node('span',sample.camera_id || '来源相机未提供'));
      (sample.artifact_ids || []).forEach(id=>{const link=node('a','查看图像');link.href='/artifacts/'+encodeURIComponent(id);link.target='_blank';link.rel='noopener noreferrer';box.append(link);});
      box.append(node('code',sample.sample_id));target.append(box);
    });
    const details=node('details');details.append(node('summary','来源与版本'),node('pre',JSON.stringify({map_binding:r.metadata?.map_binding,scene_space_id:r.scene_space_id,decision:r.metadata?.last_decision},null,2)));target.append(details);
    target.scrollTop=scroll;if(focus && oldId===r.scene_space_id)input.focus({preventScroll:true});
  }
  function accept(data) {
    const current=window.__nuwaxMapBinding?.();
    if(!data?.map_binding || !Array.isArray(data.regions))return false;
    if(current?.map_name && data.map_binding.map_name!==current.map_name)return false;
    if(state.data && scopeKey(state.data.map_binding)!==scopeKey(data.map_binding))state.selected=null;
    state.data=data;state.error='';
    if(!regions().some(r=>r.scene_space_id===state.selected))state.selected=null;
    renderList();renderDetail();ui();emitLayer();return true;
  }
  async function load() {
    if(state.disposed)return;
    if(state.request)return state.request;
    const generation=state.generation,version=state.streamVersion;
    state.request=(async()=>{
      const results=await Promise.allSettled([invoke('get_scene_mapping_status'),invoke('get_semantic_map')]);
      if(state.disposed || generation!==state.generation)return;
      if(results[0].status==='fulfilled')state.service=results[0].value;
      if(results[1].status==='fulfilled' && version===state.streamVersion){
        if(!accept(results[1].value)){state.error='地图已切换，等待对应语义记录。';emitLayer();}
      }else if(results[1].status==='rejected'){
        state.error=results[1].reason.message;emitLayer();
      }
      ui();renderDetail();
      if(state.error)message(state.error,true);
      else if(state.service?.enabled===false)message('可查看已有标注。自动识别需在设置中启用语义地图。');
      else if(state.service?.last_error)message(state.service.last_error.message || state.service.last_error.code || '最近一次识别未完成。',true);
      else if(!state.task)message(regions().length+' 个区域 · '+(state.service?.running ? ('持续识别中 · ' + (state.service.auto_apply ? '自动采用标签' : '保留候选')) : '点选查看，或识别当前地图'));
    })().finally(()=>{
      state.request=null;scheduleStatus();
      if(!state.disposed && generation!==state.generation && interested()){
        clearTimeout(state.reloadTimer);state.reloadTimer=setTimeout(load,100);
      }
    });
    return state.request;
  }
  function scheduleStatus(){clearTimeout(state.statusTimer);if(!state.disposed && interested() && (state.service?.running || state.service?.busy))state.statusTimer=setTimeout(()=>{if(document.visibilityState!=='hidden')load();else scheduleStatus();},10000);}
  async function action(fn){if(state.busy)return;state.busy=true;ui();renderDetail();try{await fn();}catch(e){message(e.message || '操作未完成。',true);}finally{state.busy=false;ui();renderDetail();}}
  function contextChanged(){
    const current=window.__nuwaxMapBinding?.();const key=bindingKey(current)+':'+Boolean(current?.available);
    if(key===state.contextKey)return;
    if(state.context && scopeKey(state.context)!==scopeKey(current)){state.selected=null;state.data=null;state.renderKey='';state.detailKey='';}
    state.context=current;state.contextKey=key;state.generation+=1;emitLayer();renderList();renderDetail();ui();
    clearTimeout(state.reloadTimer);
    if(interested())state.reloadTimer=setTimeout(load,600);
  }
  function modeChanged(){ui();emitLayer();if(interested())load();}
  function draw2d(ctx,project,currentMode){
    state.paths=[];if(!visible() || !matchesCurrent())return;
    const list=drawable(),ratio=ctx.canvas.width/Math.max(ctx.canvas.getBoundingClientRect().width,1);
    const labelRects=[];
    ctx.save();ctx.setTransform(1,0,0,1,0,0);
    const labelItems=[];
    for(const r of list){
      const polygon=r.boundary?.polygon_points;if(!polygon?.length)continue;
      const path=new Path2D();
      const ring=points=>{points.forEach((p,i)=>{const q=project({x:p.x??p[0],y:p.y??p[1]});if(i===0)path.moveTo(q.x,q.y);else path.lineTo(q.x,q.y);});path.closePath();};
      ring(polygon);for(const hole of r.boundary.metadata?.holes || [])ring(hole);
      ctx.fillStyle=color(r);ctx.globalAlpha=currentMode==='semantic'?.34:.14;ctx.fill(path,'evenodd');
      ctx.globalAlpha=1;ctx.strokeStyle=r.scene_space_id===state.selected ? getComputedStyle(document.documentElement).getPropertyValue('--ink').trim() : color(r);
      ctx.lineWidth=(r.scene_space_id===state.selected?2.6:1.2)*ratio;ctx.stroke(path);state.paths.push({path,region:r});
      const center=r.boundary.centroid?.position;if(center)labelItems.push({r,center:project(center),selected:r.scene_space_id===state.selected});
    }
    labelItems.sort((a,b)=>Number(b.selected)-Number(a.selected));
    const style=getComputedStyle(document.documentElement);ctx.font=`600 ${11.5*ratio}px ${style.getPropertyValue('--sans')}`;
    for(const item of labelItems){
      const index=regions().indexOf(item.r)+1;
      let text=item.r.primary_label || String(index);
      if(!item.selected && currentMode!=='semantic' && !item.r.primary_label)text=String(index);
      if(text.length>14)text=text.slice(0,13)+'…';
      const w=ctx.measureText(text).width+10*ratio,h=19*ratio,x=item.center.x-w/2,y=item.center.y-h/2;
      if(x+w<0 || y+h<0 || x>ctx.canvas.width || y>ctx.canvas.height)continue;
      if(!item.selected && labelRects.some(b=>x<b.x+b.w+5*ratio && x+w+5*ratio>b.x && y<b.y+b.h+4*ratio && y+h+4*ratio>b.y))continue;
      labelRects.push({x,y,w,h});ctx.fillStyle=style.getPropertyValue('--surface').trim();ctx.globalAlpha=.88;ctx.fillRect(x,y,w,h);ctx.globalAlpha=1;
      ctx.fillStyle=style.getPropertyValue('--ink').trim();ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,item.center.x,item.center.y);
    }
    ctx.restore();
  }
  function selectAt(x,y){if(!visible() || !matchesCurrent())return;const ctx=$('mapCanvas').getContext('2d');const hit=state.paths.find(p=>ctx.isPointInPath(p.path,x,y,'evenodd'));if(hit)select(hit.region.scene_space_id);}
  function pollTask(){if(!state.task || state.disposed)return;state.taskTimer=setTimeout(async()=>{
    if(document.visibilityState==='hidden'){pollTask();return;}
    try{const task=await invoke('get_task_status',{task_id:state.task});const status=task.status?.state || task.state;
      if(terminal.has(status)){state.task=null;await load();const failed=task.result?.failures?.length || 0;message(status==='succeeded'?(failed?`${failed} 个区域未完成识别，请查看来源与证据。`:'语义地图已更新。'):(task.error_payload?.message || task.error || '任务未完成。'),status!=='succeeded'||failed>0);ui();}
      else{message(task.status?.message || '正在更新识别…');pollTask();}
    }catch(e){message('暂未读到任务结果，将继续查询同一任务。',true);pollTask();}
  },3000);}
  window.nuwaxMapPanels?.register('semanticRegionPanel',()=>openPanel(false));
  $('semanticRegionPanelToggle').addEventListener('click',()=>openPanel(panel.hidden));
  $('semanticRegionDeselect').addEventListener('click',()=>{state.selected=null;renderList();renderDetail();ui();emitLayer();$('semanticRegionSearch').focus({preventScroll:true});});
  $('semanticRegionClose').addEventListener('click',()=>window.nuwaxMapPanels.close('semanticRegionPanel',true));
  $('showSemanticRegions').addEventListener('change',()=>{state.overlays[mode()]=$('showSemanticRegions').checked;ui();emitLayer();if(visible())load();});
  $('semanticRegionSearch').addEventListener('input',renderList);
  $('semanticRegionRefresh').addEventListener('click',()=>action(async()=>{const result=await invoke('refresh_semantic_map',{apply_labels:$('semanticRegionApply').checked});state.task=result.task_id || result.task?.task_id;if(!state.task)throw new Error('服务未返回任务标识。');message('更新请求已受理，等待识别结果。');pollTask();}));
  $('semanticRegionStart').addEventListener('click',()=>action(async()=>{state.service=await invoke('start_scene_mapping');message('持续识别已启动，采用策略沿当前设置。');scheduleStatus();}));
  $('semanticRegionStop').addEventListener('click',()=>action(async()=>{state.service=await invoke('stop_scene_mapping');message(state.service.stop_pending?'正在停止，等待在途识别退出。':'持续识别已停止。');scheduleStatus();}));
  $('semanticMemoryToggle')?.addEventListener('click',()=>openPanel(false));
  window.addEventListener('map:view-changed',modeChanged);
  window.addEventListener('map:context-changed',contextChanged);
  window.addEventListener('map:editing-changed',()=>{if(window.__nuwaxMapToolV2?.active())openPanel(false);});
  window.addEventListener('stream:memory.scene_spaces',event=>{state.streamVersion+=1;if(accept(event.detail))message(regions().length+' 个区域 · 标注已同步');});
  window.addEventListener('pointcloud3d:ready',emitLayer);
  window.addEventListener('pointcloud3d:semantic-region-select',event=>select(event.detail?.scene_space_id));
  window.addEventListener('scenegraph:region-select',event=>select(event.detail?.node_id));
  window.addEventListener('scenegraph:opened',()=>openPanel(false));
  new MutationObserver(()=>{emitLayer();}).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});
  window.addEventListener('pagehide',()=>{state.disposed=true;clearTimeout(state.taskTimer);clearTimeout(state.statusTimer);clearTimeout(state.reloadTimer);controllers.forEach(c=>c.abort());});
  window.addEventListener('pageshow',event=>{if(event.persisted){state.disposed=false;load();pollTask();}});
  window.nuwaxSemanticMap={draw2d,selectAt,count:()=>regions().length,selected:()=>state.selected,visible,refresh:load};
  ui();contextChanged();if(interested())load();
})();
