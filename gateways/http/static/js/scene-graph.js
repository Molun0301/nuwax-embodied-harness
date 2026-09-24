// 空间关系图复用当前地图与公共能力；布局坐标仅用于展示。
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const panel = $('sceneGraphPanel'), svg = $('sceneGraphCanvas');
  if (!panel || !svg) return;
  const NS = 'http://www.w3.org/2000/svg';
  const kinds = {region:'区域',passage:'连接口',object:'物品',observation:'观察'};
  const state = {graph:null,selected:null,detail:null,from:null,route:null,request:null,generation:0,detailGeneration:0,
    positions:new Map(),layoutKey:'',contextKey:'',view:null,drag:null,timer:null,error:'',disposed:false};
  const controllers = new Set();
  const element = (tag,text,cls) => {const n=document.createElement(tag);if(text!==undefined)n.textContent=String(text);if(cls)n.className=cls;return n;};
  const svgElement = (tag,attrs={}) => {const n=document.createElementNS(NS,tag);for(const [key,value] of Object.entries(attrs))n.setAttribute(key,String(value));return n;};
  const interested = () => !panel.hidden || $('showSceneConnections').checked;
  const bindingKey = b => JSON.stringify([b?.map_name,b?.source_name,b?.source_epoch,b?.frame_id,b?.content_sha256]);
  const current = () => {
    const b=window.__nuwaxMapBinding?.(),g=state.graph?.map_binding;
    // 同一地图的连接缓存不因一次刷新超时而消失。
    return b?.available && g && ['map_name','source_name','source_epoch','frame_id','content_sha256'].every(key=>b[key]===g[key]);
  };
  const message = (text,error=false) => {$('sceneGraphStatus').textContent=text;$('sceneGraphStatus').dataset.error=String(error);};
  async function invoke(name,args={}){
    const controller=new AbortController();controllers.add(controller);const timer=setTimeout(()=>controller.abort(),20000);
    try{const response=await fetch('/api/capabilities/'+name+'/invoke',{method:'POST',credentials:'same-origin',signal:controller.signal,
      headers:{'Content-Type':'application/json'},body:JSON.stringify({arguments:args})});const body=await response.json();
      if(!response.ok){const error=new Error(body.error?.message || '读取未完成。');error.status=response.status;throw error;}
      return body.result ?? body;
    }finally{clearTimeout(timer);controllers.delete(controller);}
  }
  function notifyLayer(){
    const visible=Boolean(current() && $('showSceneConnections').checked);
    window.dispatchEvent(new CustomEvent('pointcloud3d:scene-graph',{detail:{graph:visible?state.graph:null,visible,
      selectedId:state.selected,route:state.route}}));
    window.__nuwaxMapRequestDraw?.();
  }
  async function load(){
    if(state.disposed || !interested())return;
    if(state.request)return state.request;
    const generation=state.generation;
    state.request=(async()=>{
      try{
        const graph=await invoke('get_scene_graph',{limit:500,kinds:['region','passage','object']});
        if(state.disposed || generation!==state.generation)return;
        const binding=window.__nuwaxMapBinding?.();
        if(!binding?.available || bindingKey(binding)!==bindingKey(graph.map_binding))throw new Error('地图已更新，等待对应关系图。');
        if(state.graph?.graph_revision!==graph.graph_revision){state.route=null;state.detail=null;state.detailGeneration+=1;}
        state.graph=graph;state.error='';
        if(!graph.nodes.some(n=>n.node_id===state.selected))state.selected=null;
        if(!graph.nodes.some(n=>n.node_id===state.from))state.from=null;
        const coverage=graph.coverage;
        $('sceneGraphSummary').textContent=`${coverage.region_count} 个区域 · ${coverage.passage_count} 个连接口`;
        message(graph.truncated?'显示前500个节点；路径查询使用完整区域图。':'点选节点查看关系，或选择起点与终点查询候选路径。');
        render();notifyLayer();if(state.selected && !panel.hidden)select(state.selected);else $('sceneGraphDetail').replaceChildren(element('p','选择节点查看位置、连接和观察证据。','muted'));
      }catch(error){if(generation!==state.generation || state.disposed)return;state.error=error.message;message(state.error,true);notifyLayer();}
    })().finally(()=>{state.request=null;if(generation!==state.generation && interested())schedule();});
    return state.request;
  }
  function schedule(){clearTimeout(state.timer);if(!state.disposed && interested())state.timer=setTimeout(load,300);}
  function open(opened,nodeId){
    if(opened)window.nuwaxMapPanels?.activate('sceneGraphPanel');
    panel.hidden=!opened;$('sceneGraphToggle').setAttribute('aria-expanded',String(opened));$('sceneGraphToggle').classList.toggle('active',opened);
    if(opened){window.dispatchEvent(new CustomEvent('scenegraph:opened'));if(nodeId)state.selected=nodeId;load();render();}
  }
  function layout(nodes,edges){
    const key=JSON.stringify([$('sceneGraphLayout').value,nodes.map(n=>[n.node_id,n.position]),edges.map(e=>[e.source,e.target])]);
    if(key===state.layoutKey)return;
    state.layoutKey=key;state.positions=new Map();state.view=null;
    if($('sceneGraphLayout').value==='spatial'){
      for(const n of nodes){if(n.position)state.positions.set(n.node_id,{x:n.position.x*60,y:-n.position.y*60});}
      return;
    }
    // 确定性的力导向排布让高连接度区域展开；相同节点/边保持同一布局。
    const count=nodes.length,points=nodes.map((node,i)=>({id:node.node_id,x:280*Math.cos(2*Math.PI*i/Math.max(count,1)),y:220*Math.sin(2*Math.PI*i/Math.max(count,1))}));
    const byId=new Map(points.map((p,i)=>[p.id,i]));
    const links=edges.map(e=>[byId.get(e.source),byId.get(e.target)]).filter(([a,b])=>a!==undefined&&b!==undefined);
    const ideal=Math.sqrt(800*650/Math.max(count,1));
    for(let iteration=0;iteration<150;iteration++){
      const forces=points.map(()=>({x:0,y:0}));
      for(let i=0;i<count;i++)for(let j=i+1;j<count;j++){
        let dx=points[i].x-points[j].x,dy=points[i].y-points[j].y;const distance=Math.max(1,Math.hypot(dx,dy));
        if(dx===0&&dy===0){dx=.1;dy=.1;}
        const magnitude=ideal*ideal/(distance*distance);forces[i].x+=dx*magnitude;forces[i].y+=dy*magnitude;forces[j].x-=dx*magnitude;forces[j].y-=dy*magnitude;
      }
      for(const [a,b] of links){const dx=points[b].x-points[a].x,dy=points[b].y-points[a].y,distance=Math.max(1,Math.hypot(dx,dy)),magnitude=distance/ideal;
        forces[a].x+=dx*magnitude;forces[a].y+=dy*magnitude;forces[b].x-=dx*magnitude;forces[b].y-=dy*magnitude;}
      const temperature=14*(1-iteration/150)+.1;
      for(let i=0;i<count;i++){forces[i].x-=points[i].x*.03;forces[i].y-=points[i].y*.03;const length=Math.hypot(forces[i].x,forces[i].y)||1;
        points[i].x+=forces[i].x/length*Math.min(length,temperature);points[i].y+=forces[i].y/length*Math.min(length,temperature);}
    }
    for(const p of points)state.positions.set(p.id,p);
  }

  function fit(){
    const points=[...state.positions.values()];if(!points.length){state.view={x:0,y:0,w:800,h:600};}
    else{const xs=points.map(p=>p.x),ys=points.map(p=>p.y);const x=Math.min(...xs)-70,y=Math.min(...ys)-60;
      state.view={x,y,w:Math.max(160,Math.max(...xs)-x+90),h:Math.max(160,Math.max(...ys)-y+90)};}
    applyView();
  }
  function applyView(){
    const v=state.view;if(!v)return;svg.setAttribute('viewBox',`${v.x} ${v.y} ${v.w} ${v.h}`);
    const scale=Math.max(.05,Math.min(svg.clientWidth/v.w,svg.clientHeight/v.h));
    for(const group of svg.querySelectorAll('.scene-graph-node')){
      const passage=group.dataset.kind==='passage',radius=(passage?3.5:group.dataset.kind==='region'?6:4)/scale;
      const shape=group.querySelector('.node-shape'),text=group.querySelector('text'),hit=group.querySelector('.node-hit');
      if(hit){hit.setAttribute('x',-12/scale);hit.setAttribute('y',-12/scale);hit.setAttribute('width',24/scale);hit.setAttribute('height',36/scale);}
      if(passage)shape.setAttribute('d',`M 0 ${-radius} L ${radius} 0 L 0 ${radius} L ${-radius} 0 Z`);else shape.setAttribute('r',radius);
      text.style.fontSize=(passage?10:11)/scale+'px';text.style.strokeWidth=3/scale+'px';text.setAttribute('y',passage?-8/scale:18/scale);
    }
  }
  function render(){
    if(panel.hidden)return;
    const graph=state.graph;if(!graph){svg.replaceChildren();return;}
    layout(graph.nodes,graph.edges);const fragment=document.createDocumentFragment(),query=$('sceneGraphSearch').value.trim().toLowerCase();
    const matched=new Set(graph.nodes.filter(n=>!query || (n.name+' '+(n.aliases||[]).join(' ')+' '+(n.properties.display_code||'')).toLowerCase().includes(query)).map(n=>n.node_id));
    const routeNodes=new Set([...(state.route?.region_ids||[]),...(state.route?.passages||[]).map(p=>p.node_id)]);
    for(const e of graph.edges){const a=state.positions.get(e.source),b=state.positions.get(e.target);if(!a||!b)continue;
      fragment.append(svgElement('line',{x1:a.x,y1:a.y,x2:b.x,y2:b.y,class:'scene-graph-edge','data-route':routeNodes.has(e.source)&&routeNodes.has(e.target)}));}
    for(const n of graph.nodes){const p=state.positions.get(n.node_id);if(!p)continue;
      const group=svgElement('g',{transform:`translate(${p.x},${p.y})`,class:'scene-graph-node',role:'button',tabindex:0,
        'aria-label':`${kinds[n.kind]}：${n.name}`,'aria-pressed':String(state.selected===n.node_id),'data-kind':n.kind,'data-node-id':n.node_id,'data-dimmed':query&&!matched.has(n.node_id)});
      const radius=n.kind==='region'?17:n.kind==='passage'?6:9;
      group.append(svgElement('rect',{class:'node-hit',x:-12,y:-12,width:24,height:36,fill:'transparent'}));
      group.append(n.kind==='passage'?svgElement('path',{d:'M 0 -7 L 7 0 L 0 7 L -7 0 Z',class:'node-shape'}):svgElement('circle',{r:radius,class:'node-shape'}));
      const text=svgElement('text',{x:0,y:n.kind==='passage'?-11:radius+17,'text-anchor':'middle'});
      const code=n.properties.display_code;const label=n.kind==='passage'?(state.selected===n.node_id?code:''):n.kind==='region'?(code+(n.properties.has_label?' '+n.name:'')):n.name;
      text.textContent=label.length>18?label.slice(0,17)+'…':label;group.append(text);
      const title=svgElement('title');title.textContent=n.name;group.append(title);
      group.addEventListener('click',()=>select(n.node_id));group.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();select(n.node_id);}});fragment.append(group);
    }
    svg.replaceChildren(fragment);if(!state.view)fit();else applyView();
  }
  async function select(id){
    const graph=state.graph;if(!graph || !graph.nodes.some(n=>n.node_id===id))return;
    state.selected=id;state.detail=null;const token=++state.detailGeneration;render();notifyLayer();
    const target=$('sceneGraphDetail');target.replaceChildren(element('p','正在读取节点…','muted'));
    try{const detail=await invoke('get_scene_graph_node',{node_id:id,expected_graph_revision:graph.graph_revision});
      if(state.disposed||token!==state.detailGeneration||state.graph?.graph_revision!==detail.graph_revision)return;
      state.detail=detail;renderDetail();
    }catch(error){if(token!==state.detailGeneration)return;target.replaceChildren(element('p',error.message,'muted'));if(error.status===409)schedule();}
  }
  function button(text,action){const b=element('button',text,'tool');b.type='button';b.addEventListener('click',action);return b;}
  function renderDetail(){
    const detail=state.detail;if(!detail)return;const target=$('sceneGraphDetail'),n=detail.node,p=n.properties;
    target.replaceChildren(element('h3',n.name),element('p',kinds[n.kind]+(p.display_code?' · '+p.display_code:''),'muted'));
    if(n.position)target.append(element('p',`位置：${n.position.x.toFixed(2)}, ${n.position.y.toFixed(2)}, ${n.position.z.toFixed(2)} 米`));
    if(p.boundary?.area_m2!=null)target.append(element('p',`面积：${p.boundary.area_m2.toFixed(2)} 平方米`));
    if(n.kind==='passage')target.append(element('p','当前地图的自由区域在此相接；门类型与实际通行尚未确认。'));
    if(p.summary)target.append(element('p',p.summary));
    if(p.observed_at||p.last_seen_ts)target.append(element('p','观察时间：'+(p.observed_at||p.last_seen_ts),'muted'));
    const actions=element('div',undefined,'graph-detail-actions');
    if(n.kind==='region')actions.append(button('在地图中查看',()=>{open(false);window.dispatchEvent(new CustomEvent('scenegraph:region-select',{detail:{node_id:n.node_id}}));window.__nuwaxMapFocusRegion?.({scene_space_id:n.node_id,boundary:n.properties.boundary});}));
    if(n.kind==='region'||p.region_id){actions.append(button(state.from===n.node_id?'已设为起点':'设为起点',()=>{state.from=n.node_id;state.route=null;message('起点：'+n.name+'。选择终点后查询候选路径。');renderDetail();notifyLayer();}));
      const routeButton=button('查询到此的路径',()=>route(n.node_id));routeButton.disabled=!state.from;actions.append(routeButton);}
    actions.append(button('查看局部图像',()=>exportImage(n.node_id)));target.append(actions);
    target.append(element('h4','关联节点'));
    if(!detail.neighbors.length)target.append(element('p','暂无已建立的关系。','muted'));
    for(const neighbor of detail.neighbors){const b=element('button',`${kinds[neighbor.kind]} · ${neighbor.name}`,'map-semantic-row');b.type='button';b.addEventListener('click',()=>{
      if(state.graph.nodes.some(x=>x.node_id===neighbor.node_id))select(neighbor.node_id);else inspectRelated(neighbor.node_id);
    });target.append(b);}
    target.append(element('h4','图像证据'));
    const images=detail.images.filter(i=>i.available);if(!images.length)target.append(element('p','暂无可用图像。','muted'));
    for(const [i,image] of images.entries()){const a=element('a','查看图像 '+(i+1));a.href=image.url;a.target='_blank';a.rel='noopener noreferrer';target.append(a);}
    const source=element('details');source.append(element('summary','身份与来源'),element('pre',JSON.stringify({node_id:n.node_id,map_binding:detail.map_binding,graph_revision:detail.graph_revision,properties:p},null,2)));target.append(source);
  }
  async function inspectRelated(id){
    const version=state.graph?.graph_revision,token=++state.detailGeneration;
    try{const detail=await invoke('get_scene_graph_node',{node_id:id,expected_graph_revision:version});if(token!==state.detailGeneration||version!==state.graph?.graph_revision)return;state.detail=detail;renderDetail();}
    catch(error){message(error.message,true);}
  }
  async function route(to){
    const version=state.graph?.graph_revision,from=state.from;if(!from||!version)return;
    try{const result=await invoke('plan_scene_graph_route',{from_node_id:from,to_node_id:to,expected_graph_revision:version});
      if(version!==state.graph?.graph_revision||from!==state.from)return;state.route=result;render();notifyLayer();
      message(result.status==='candidate_route'?`候选路径：${result.region_names.join(' → ')} · ${result.passage_count} 个连接口，待导航检查。`:'当前关系图没有已知连接路径。');
    }catch(error){message(error.message,true);if(error.status===409)schedule();}
  }
  async function exportImage(id){
    if(!state.graph)return;const version=state.graph.graph_revision;
    try{const result=await invoke('render_scene_graph_image',{...(id?{node_id:id}:{}),expected_graph_revision:version,width:1280});
      if(version!==state.graph?.graph_revision)return;message('地图图像已生成。');const link=element('a','打开地图图像');link.href='/artifacts/'+encodeURIComponent(result.artifact.artifact_id);link.target='_blank';link.rel='noopener noreferrer';$('sceneGraphStatus').append(' ',link);
    }catch(error){message(error.message,true);if(error.status===409)schedule();}
  }
  function contextChanged(){
    const binding=window.__nuwaxMapBinding?.(),key=bindingKey(binding)+':'+Boolean(binding?.available);if(key===state.contextKey)return;
    state.contextKey=key;state.generation+=1;state.detailGeneration+=1;state.graph=null;state.route=null;state.detail=null;state.selected=null;state.from=null;state.layoutKey='';state.error='';
    $('sceneGraphDetail').replaceChildren(element('p','地图已更新，正在读取对应关系。','muted'));render();notifyLayer();schedule();
  }
  function draw2d(ctx,project){
    if(!$('showSceneConnections').checked||!current())return;
    const ratio=ctx.canvas.width/Math.max(ctx.canvas.getBoundingClientRect().width,1),nodes=new Map(state.graph.nodes.map(n=>[n.node_id,n]));
    const selected=new Set([...(state.route?.region_ids||[]),...(state.route?.passages||[]).map(p=>p.node_id)]);
    ctx.save();ctx.setTransform(1,0,0,1,0,0);ctx.lineWidth=1.1*ratio;ctx.setLineDash([4*ratio,4*ratio]);
    for(const edge of state.graph.edges){if(edge.kind!=='access')continue;const a=nodes.get(edge.source)?.position,b=nodes.get(edge.target)?.position;if(!a||!b)continue;
      const p=project(a),q=project(b);ctx.strokeStyle=selected.has(edge.source)&&selected.has(edge.target)?'#4f8ccc':'#707d87';ctx.globalAlpha=.7;
      ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(q.x,q.y);ctx.stroke();}
    ctx.setLineDash([]);ctx.globalAlpha=1;
    for(const n of nodes.values()){if(n.kind!=='passage'||!n.position)continue;const p=project(n.position);ctx.fillStyle='#c68a3b';ctx.beginPath();ctx.arc(p.x,p.y,3.3*ratio,0,2*Math.PI);ctx.fill();}
    ctx.restore();
  }
  window.nuwaxMapPanels?.register('sceneGraphPanel',()=>open(false));
  $('sceneGraphToggle').addEventListener('click',()=>open(panel.hidden));$('sceneGraphClose').addEventListener('click',()=>window.nuwaxMapPanels.close('sceneGraphPanel',true));
  $('showSceneConnections').addEventListener('change',()=>{notifyLayer();if(interested())load();});
  $('sceneGraphLayout').addEventListener('change',render);$('sceneGraphSearch').addEventListener('input',render);
  $('sceneGraphFit').addEventListener('click',fit);$('sceneGraphImage').addEventListener('click',()=>exportImage());
  svg.addEventListener('wheel',event=>{if(!state.view)return;event.preventDefault();const factor=Math.exp(Math.max(-.4,Math.min(.4,event.deltaY*.0015))),v=state.view;
    if(v.w*factor<40||v.w*factor>100000)return;state.view={x:v.x+v.w*(1-factor)/2,y:v.y+v.h*(1-factor)/2,w:v.w*factor,h:v.h*factor};applyView();},{passive:false});
  svg.addEventListener('pointerdown',event=>{if(event.target.closest('.scene-graph-node')||!state.view)return;state.drag={x:event.clientX,y:event.clientY,view:{...state.view}};svg.setPointerCapture(event.pointerId);});
  svg.addEventListener('pointermove',event=>{if(!state.drag)return;const rect=svg.getBoundingClientRect(),v=state.drag.view,factor=Math.max(v.w/rect.width,v.h/rect.height);state.view={...v,x:v.x-(event.clientX-state.drag.x)*factor,y:v.y-(event.clientY-state.drag.y)*factor};applyView();});
  svg.addEventListener('pointerup',()=>{state.drag=null;});svg.addEventListener('pointercancel',()=>{state.drag=null;});
  window.addEventListener('scenegraph:open',event=>open(true,event.detail?.node_id));
  window.addEventListener('map:context-changed',contextChanged);
  window.addEventListener('map:view-changed',notifyLayer);window.addEventListener('pointcloud3d:ready',notifyLayer);
  window.addEventListener('pointcloud3d:scene-graph-select',event=>open(true,event.detail?.node_id));
  window.addEventListener('stream:memory.scene_spaces',()=>{state.generation+=1;state.detailGeneration+=1;state.route=null;state.error='';schedule();});
  window.addEventListener('stream:memory.instances',()=>{state.generation+=1;schedule();});
  window.addEventListener('map:editing-changed',()=>{if(window.__nuwaxMapToolV2?.active())open(false);});
  $('semanticRegionPanelToggle')?.addEventListener('click',()=>open(false));
  const resizeObserver=new ResizeObserver(()=>{if(!panel.hidden)applyView();});resizeObserver.observe(svg);
  window.addEventListener('pagehide',()=>{state.disposed=true;resizeObserver.disconnect();clearTimeout(state.timer);controllers.forEach(c=>c.abort());});
  window.addEventListener('pageshow',event=>{if(event.persisted){state.disposed=false;resizeObserver.observe(svg);schedule();}});
  window.nuwaxSceneGraph={draw2d,refresh:load};contextChanged();
})();
