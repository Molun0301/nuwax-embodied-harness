/* Saved-map cleanup. The slider controls evidence tolerance, not a quota. */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const panel = $('mapCleanupPanel');
  if (!panel) return;
  const state = {job: null, epoch: 0, timer: null, source: null, candidates: null, angle: -.6, tilt: .65, zoom: 1};
  const say = text => { $('mapCleanupStatus').textContent = text; };
  async function api(path, body) {
    const response = await fetch('/api/maps/cleanup' + path, {credentials: 'same-origin',
      ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)})});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error?.message || value.detail || '地图清理请求失败');
    return value.result ?? value;
  }
  function buttons() {
    const ready = state.job?.state === 'ready';
    $('mapCleanupApply').disabled = !ready;
    $('mapCleanupCancel').disabled = !state.job || ['applied','cancelled','failed'].includes(state.job.state);
  }
  async function invalidate() {
    const old = state.job;
    state.epoch++; state.job = null; clearTimeout(state.timer); buttons();
    state.source = null; state.candidates = null; $('mapCleanupCanvas').hidden = true; $('mapCleanupDetails').textContent = '';
    if (old && ['queued','running','ready','cancel_error'].includes(old.state)) {
      try { await api('/' + old.id + '/cancel', {}); } catch (error) { state.job = {...old, state: 'cancel_error'}; buttons(); say(error.message); return false; }
    }
    return true;
  }
  async function loadPoints(job, epoch) {
    const points = async kind => {
      const response = await fetch('/api/maps/cleanup/' + job.id + '/points/' + kind, {credentials: 'same-origin'});
      if (!response.ok) throw new Error('预览点云读取失败');
      return new Float32Array(await response.arrayBuffer());
    };
    const [source, candidates] = await Promise.all([points('source'), points('candidates')]);
    if (epoch !== state.epoch) return;
    state.source = source; state.candidates = candidates; state.zoom = 1;
    $('mapCleanupCanvas').hidden = false; draw();
  }
  async function poll(id, epoch) {
    try {
      const job = await api('/' + id);
      if (epoch !== state.epoch) return;
      state.job = job; buttons();
      if (job.state === 'ready') {
        const r = job.report;
        say(`实际候选删除 ${r.preview.candidate_points.toLocaleString()} 个细点云体素；累计扫描点 ${r.dense_before.toLocaleString()} → ${r.dense_after.toLocaleString()}。${r.offline_note} 原图未改变。`);
        $('mapCleanupDetails').textContent = `原图灰色，候选红色。力度 ${job.strength_percent}%。人体重判 ${r.offline_frames} 帧；回补 ${r.recovered_voxels} 个体素。拖动旋转，滚轮缩放；大图仅显示抽样，候选计数为全量。`;
        await loadPoints(job, epoch);
      } else if (['failed','cancelled','applied'].includes(job.state)) {
        say(job.error || (job.state === 'cancelled' ? '已取消，原图未改变。' : `已另存为 ${job.result_map}，原图保留。`));
      } else {
        say('正在用保存的累计地图计算实际候选，原图未改变…');
        state.timer = setTimeout(() => poll(id, epoch), 1000);
      }
    } catch (error) { if (epoch === state.epoch) { say(error.message); state.job = null; buttons(); } }
  }
  $('mapCleanupPreview').addEventListener('click', async () => {
    if (!await invalidate()) return; const epoch = state.epoch;
    const name = $('mapCleanupSource').value;
    if (!name) { say('请选择已完成的地图。'); return; }
    say('正在准备预览…');
    try {
      const job = await api('', {map_name: name, strength_percent: Number($('mapCleanupStrength').value)});
      if (epoch !== state.epoch) { await api('/' + job.id + '/cancel', {}); return; }
      state.job = job; buttons(); poll(job.id, epoch);
    } catch (error) { if (epoch === state.epoch) say(error.message); }
  });
  $('mapCleanupStrength').addEventListener('input', () => {
    $('mapCleanupPercent').textContent = $('mapCleanupStrength').value + '%';
    invalidate(); say('力度已改变，请重新预览实际候选。');
  });
  $('mapCleanupSource').addEventListener('change', () => {
    invalidate(); $('mapCleanupNewName').value = $('mapCleanupSource').value + '-clean';
    say('请选择力度并预览，原图始终保留。');
  });
  $('mapCleanupCancel').addEventListener('click', async () => { if (await invalidate()) say('已请求取消预览，原图未改变。'); });
  $('mapCleanupApply').addEventListener('click', async () => {
    const job = state.job, name = $('mapCleanupNewName').value.trim();
    if (!job || job.state !== 'ready') return;
    if (!name || name === job.map_name) { say('请填写不同的新地图名称。'); return; }
    if (!window.confirm(`确认按当前 ${job.strength_percent}% 力度删除预览中的 ${job.report.preview.candidate_points} 个体素，并另存为“${name}”？原图“${job.map_name}”保留。`)) return;
    $('mapCleanupApply').disabled = true; $('mapCleanupCancel').disabled = true;
    for (const id of ['mapCleanupSource','mapCleanupStrength','mapCleanupPreview']) $(id).disabled = true;
    try {
      state.job = await api('/' + job.id + '/apply', {new_map_name: name, confirmed: true});
      buttons(); say(`已另存为 ${state.job.result_map}，原图 ${job.map_name} 保留；未切换当前地图。`);
    } catch (error) { say(error.message); state.job = null; buttons(); }
    finally { for (const id of ['mapCleanupSource','mapCleanupStrength','mapCleanupPreview']) $(id).disabled = false; }
  });
  async function maps() {
    try {
      const response = await fetch('/api/maps/catalog', {credentials: 'same-origin'});
      const body = await response.json(); if (!response.ok) throw new Error(body.error?.message || '地图列表读取失败');
      const items = (body.result ?? body).maps || [];
      const selected = state.job?.map_name || $('mapCleanupSource').value || window.__nuwaxMapBinding?.()?.map_name;
      $('mapCleanupSource').replaceChildren();
      for (const map of items.filter(m => m.finalized)) {
        const option = document.createElement('option'); option.value = map.map_name; option.textContent = map.display_name || map.map_name;
        $('mapCleanupSource').append(option);
      }
      if (items.some(m => m.finalized && m.map_name === selected)) $('mapCleanupSource').value = selected;
      if (!$('mapCleanupNewName').value) $('mapCleanupNewName').value = $('mapCleanupSource').value + '-clean';
      if (!$('mapCleanupSource').value) say('暂无已完成地图。请先保存地图并停止建图。');
    } catch (error) { say(error.message); }
  }
  $('mapCleanupToggle').addEventListener('click', () => {
    if (!panel.hidden) { window.nuwaxMapPanels?.close(panel.id, true); return; }
    window.nuwaxMapPanels?.activate(panel.id); panel.hidden = false; maps(); draw();
  });
  function draw() {
    const canvas = $('mapCleanupCanvas'), ctx = canvas.getContext('2d');
    if (!ctx || !state.source?.length) return;
    canvas.width = Math.max(320, canvas.clientWidth || 640); canvas.height = 240;
    ctx.fillStyle = '#111b24'; ctx.fillRect(0,0,canvas.width,canvas.height);
    let lo = [Infinity,Infinity,Infinity], hi = [-Infinity,-Infinity,-Infinity];
    for(let i=0;i<state.source.length;i+=3) for(let a=0;a<3;a++){lo[a]=Math.min(lo[a],state.source[i+a]);hi[a]=Math.max(hi[a],state.source[i+a]);}
    const center=lo.map((v,a)=>(v+hi[a])/2), span=Math.max(...hi.map((v,a)=>v-lo[a]),.1);
    const scale=Math.min(canvas.width,canvas.height)*.8/span*state.zoom;
    const ca=Math.cos(state.angle),sa=Math.sin(state.angle),ct=Math.cos(state.tilt),st=Math.sin(state.tilt);
    for (const [points,color,size] of [[state.source,'#748997',1],[state.candidates,'#ff615c',2]]) {
      ctx.fillStyle=color; const stride=Math.max(1,Math.ceil(points.length/3/200000))*3;
      for(let i=0;i<points.length;i+=stride){const x=points[i]-center[0],y=points[i+1]-center[1],z=points[i+2]-center[2];
        const px=x*ca-y*sa,py=(x*sa+y*ca)*st-z*ct;
        ctx.fillRect(canvas.width/2+px*scale,canvas.height/2+py*scale,size,size);}
    }
  }
  let drag=null;
  $('mapCleanupCanvas').addEventListener('pointerdown', e=>{drag=[e.clientX,e.clientY];e.target.setPointerCapture?.(e.pointerId);});
  $('mapCleanupCanvas').addEventListener('pointermove', e=>{if(!drag)return;state.angle+=(e.clientX-drag[0])*.01;state.tilt=Math.max(-1.5,Math.min(1.5,state.tilt+(e.clientY-drag[1])*.01));drag=[e.clientX,e.clientY];draw();});
  $('mapCleanupCanvas').addEventListener('pointerup',()=>{drag=null;});
  $('mapCleanupCanvas').addEventListener('pointercancel',()=>{drag=null;});
  $('mapCleanupCanvas').addEventListener('wheel',e=>{e.preventDefault();state.zoom=Math.max(.1,Math.min(20,state.zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
  window.addEventListener('resize',draw);
  buttons();
})();
