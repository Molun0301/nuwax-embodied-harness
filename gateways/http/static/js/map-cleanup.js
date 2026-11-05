/* 已保存地图清理：统计与点集来自工具，另存完成以任务真实结果为准。 */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const panel = $('mapCleanupPanel');
  if (!panel) return;
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  const layers = [
    {kind: 'source', label: '原图', color: '#748997', size: 1, checked: true},
    {kind: 'candidates', label: '删除候选', color: '#ff615c', size: 2, checked: true},
    {kind: 'result', label: '清理结果', color: '#66d6a5', size: 1, checked: false},
  ];
  const state = {job: null, task: null, epoch: 0, timer: null, points: {}, previewReady: false,
    cancelling: null, submitting: false, angle: -.6, tilt: .65, zoom: 1};
  const say = text => { $('mapCleanupStatus').textContent = text; };
  const legend = document.createElement('div');
  legend.className = 'msb-row'; legend.hidden = true;
  for (const layer of layers) {
    const label = document.createElement('label');
    const checkbox = document.createElement('input');
    checkbox.type = 'checkbox'; checkbox.checked = layer.checked;
    checkbox.id = 'mapCleanupLayer-' + layer.kind;
    checkbox.addEventListener('change', draw);
    label.style.color = layer.color;
    label.append(checkbox, document.createTextNode(layer.label));
    legend.append(label);
  }
  $('mapCleanupCanvas').before(legend);
  $('mapCleanupCanvas').setAttribute('aria-label', '原图、删除候选和清理结果三层点云预览');
  $('mapCleanupDetails').style.whiteSpace = 'pre-wrap';

  async function request(url, body) {
    const response = await fetch(url, {credentials: 'same-origin',
      ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)})});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error?.message || value.detail || '地图清理请求失败');
    return value;
  }
  const api = (path, body) => request('/api/maps/cleanup' + path, body);
  const taskActive = () => state.task !== null && !terminal.includes(state.task.state);
  function buttons() {
    const saving = state.submitting || taskActive();
    $('mapCleanupApply').disabled = saving || !state.previewReady || state.job?.state !== 'ready' || !!state.cancelling;
    $('mapCleanupPreview').disabled = saving || !!state.cancelling;
    $('mapCleanupSource').disabled = $('mapCleanupStrength').disabled = saving;
    $('mapCleanupCancel').disabled = saving || !!state.cancelling || !state.job || ['applied', 'cancelled'].includes(state.job.state);
  }
  function clearPreview() {
    state.job = state.task = null; state.points = {}; state.previewReady = false;
    $('mapCleanupCanvas').hidden = legend.hidden = true;
    $('mapCleanupDetails').textContent = '';
    buttons();
  }
  async function invalidate() {
    if (state.cancelling) return state.cancelling;
    const old = state.job, visible = state.previewReady;
    const epoch = ++state.epoch;
    clearTimeout(state.timer); state.previewReady = false;
    if (!old || ['applied', 'cancelled'].includes(old.state)) {
      clearPreview(); return true;
    }
    state.cancelling = (async () => {
      try {
        await api('/' + encodeURIComponent(old.id) + '/cancel', {});
        clearPreview();
        return true;
      } catch (error) {
        state.previewReady = visible;
        say(error.message);
        if (['queued', 'running'].includes(old.state)) schedule(old.id, epoch);
        return false;
      } finally {
        state.cancelling = null; buttons();
      }
    })();
    buttons();
    return state.cancelling;
  }
  function schedule(id, epoch) {
    clearTimeout(state.timer);
    state.timer = setTimeout(() => poll(id, epoch), 1000);
  }
  async function loadPoints(job, epoch) {
    const values = await Promise.all(layers.map(async layer => {
      const response = await fetch('/api/maps/cleanup/' + encodeURIComponent(job.id) + '/points/' + layer.kind, {credentials: 'same-origin'});
      if (!response.ok) throw new Error(layer.label + '点云读取失败');
      return [layer.kind, new Float32Array(await response.arrayBuffer())];
    }));
    if (epoch !== state.epoch) return;
    state.points = Object.fromEntries(values); state.zoom = 1;
    $('mapCleanupCanvas').hidden = legend.hidden = false;
    state.previewReady = true; draw(); buttons();
  }
  function showSummary(job) {
    $('mapCleanupDetails').textContent = `力度 ${job.strength_percent}%。可切换原图、删除候选和清理结果；拖动旋转，滚轮缩放。大图按显示抽样。\n工具统计摘要：\n` + JSON.stringify(job.report, null, 2);
  }
  async function poll(id, epoch) {
    try {
      const taskId = state.task?.task_id;
      const [job, value] = await Promise.all([
        api('/' + encodeURIComponent(id)),
        taskId ? request('/api/tasks/' + encodeURIComponent(taskId)) : Promise.resolve(null),
      ]);
      if (epoch !== state.epoch) return;
      state.job = job;
      if (value !== null) state.task = value.task;
      if (state.task !== null) {
        if (state.task.state === 'succeeded') {
          state.job = state.task.result;
          say(`已另存为 ${state.job.result_map}，原图 ${state.job.map_name} 保留。`);
        } else if (terminal.includes(state.task.state)) {
          const message = state.task.message;
          if (job.state === 'cancelled') clearPreview();
          say(message);
        } else {
          say(state.task.message || '另存任务正在执行。');
          schedule(id, epoch);
        }
      } else if (job.state === 'ready') {
        showSummary(job);
        if (!state.previewReady) await loadPoints(job, epoch);
        if (epoch === state.epoch) say('实际清理预览已就绪，请查看三层点云与工具统计摘要，再确认另存。');
      } else if (job.state === 'applied') {
        say(`已另存为 ${job.result_map}，原图 ${job.map_name} 保留。`);
      } else if (job.state === 'cancelled') {
        clearPreview(); say('预览已取消。');
      } else if (job.state === 'failed') {
        say(job.error);
      } else {
        say('正在计算清理预览。'); schedule(id, epoch);
      }
      buttons();
    } catch (error) {
      if (epoch === state.epoch) {
        say(error.message); buttons();
        if (taskActive() || ['queued', 'running', 'applying'].includes(state.job?.state)) schedule(id, epoch);
      }
    }
  }
  $('mapCleanupPreview').addEventListener('click', async () => {
    if (!await invalidate()) return;
    const epoch = state.epoch, name = $('mapCleanupSource').value;
    if (!name) { say('请选择已保存地图。'); return; }
    say('正在准备预览。');
    try {
      const job = await api('', {map_name: name, strength_percent: Number($('mapCleanupStrength').value)});
      if (epoch !== state.epoch) { await api('/' + encodeURIComponent(job.id) + '/cancel', {}); return; }
      state.job = job; buttons(); await poll(job.id, epoch);
    } catch (error) { if (epoch === state.epoch) say(error.message); }
  });
  $('mapCleanupStrength').addEventListener('input', async () => {
    $('mapCleanupPercent').textContent = $('mapCleanupStrength').value + '%';
    if (await invalidate()) say('力度已改变，请重新计算实际预览。');
  });
  $('mapCleanupSource').addEventListener('change', async () => {
    $('mapCleanupNewName').value = $('mapCleanupSource').value + '-clean';
    if (await invalidate()) say('请选择力度并计算预览。');
  });
  $('mapCleanupCancel').addEventListener('click', async () => {
    if (await invalidate()) say('预览已取消，工作目录已删除。');
  });
  $('mapCleanupApply').addEventListener('click', async () => {
    const job = state.job, name = $('mapCleanupNewName').value.trim(), epoch = state.epoch;
    if (!job || job.state !== 'ready' || !state.previewReady) return;
    if (!name) { say('请填写新地图名称。'); return; }
    if (!window.confirm(`确认按当前 ${job.strength_percent}% 力度，将查看过的清理结果另存为“${name}”？源地图“${job.map_name}”保留。`)) return;
    state.submitting = true; buttons(); say('正在提交另存任务。');
    try {
      const submitted = await api('/' + encodeURIComponent(job.id) + '/apply', {new_map_name: name, confirmed: true});
      state.task = submitted.task;
      say(state.task.message || '另存任务已提交，等待执行结果。');
      await poll(job.id, epoch);
    } catch (error) {
      say(error.message);
    } finally {
      state.submitting = false; buttons();
    }
  });
  async function maps() {
    try {
      const items = (await request('/api/maps/catalog')).maps;
      const selected = state.job?.map_name || $('mapCleanupSource').value;
      $('mapCleanupSource').replaceChildren();
      for (const map of items.filter(item => item.saved)) {
        const option = document.createElement('option'); option.value = map.name; option.textContent = map.display_name;
        $('mapCleanupSource').append(option);
      }
      if (items.some(map => map.saved && map.name === selected)) $('mapCleanupSource').value = selected;
      if (!$('mapCleanupNewName').value) $('mapCleanupNewName').value = $('mapCleanupSource').value + '-clean';
      if (!$('mapCleanupSource').value) say('暂无已保存地图，请先保存地图。');
    } catch (error) { say(error.message); }
  }
  $('mapCleanupToggle').addEventListener('click', async () => {
    if (!panel.hidden) { window.nuwaxMapPanels?.close(panel.id, true); return; }
    window.nuwaxMapPanels?.activate(panel.id); panel.hidden = false;
    await maps(); draw();
  });
  function draw() {
    const canvas = $('mapCleanupCanvas'), ctx = canvas.getContext('2d');
    if (!ctx) return;
    canvas.width = Math.max(320, canvas.clientWidth || 640); canvas.height = 240;
    ctx.fillStyle = '#111b24'; ctx.fillRect(0, 0, canvas.width, canvas.height);
    const reference = layers.map(layer => state.points[layer.kind]).find(points => points?.length);
    if (!reference) return;
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < reference.length; i += 3) for (let a = 0; a < 3; a++) {
      lo[a] = Math.min(lo[a], reference[i + a]); hi[a] = Math.max(hi[a], reference[i + a]);
    }
    const center = lo.map((value, axis) => (value + hi[axis]) / 2);
    const span = Math.max(...hi.map((value, axis) => value - lo[axis]), .1);
    const scale = Math.min(canvas.width, canvas.height) * .8 / span * state.zoom;
    const ca = Math.cos(state.angle), sa = Math.sin(state.angle), ct = Math.cos(state.tilt), st = Math.sin(state.tilt);
    for (const layer of layers) {
      if (!$('mapCleanupLayer-' + layer.kind).checked) continue;
      const points = state.points[layer.kind];
      if (!points) continue;
      ctx.fillStyle = layer.color;
      const stride = Math.max(1, Math.ceil(points.length / 3 / 200000)) * 3;
      for (let i = 0; i < points.length; i += stride) {
        const x = points[i] - center[0], y = points[i + 1] - center[1], z = points[i + 2] - center[2];
        const px = x * ca - y * sa, py = (x * sa + y * ca) * st - z * ct;
        ctx.fillRect(canvas.width / 2 + px * scale, canvas.height / 2 + py * scale, layer.size, layer.size);
      }
    }
  }
  let drag = null;
  $('mapCleanupCanvas').addEventListener('pointerdown', event => { drag = [event.clientX, event.clientY]; event.target.setPointerCapture?.(event.pointerId); });
  $('mapCleanupCanvas').addEventListener('pointermove', event => {
    if (!drag) return;
    state.angle += (event.clientX - drag[0]) * .01;
    state.tilt = Math.max(-1.5, Math.min(1.5, state.tilt + (event.clientY - drag[1]) * .01));
    drag = [event.clientX, event.clientY]; draw();
  });
  $('mapCleanupCanvas').addEventListener('pointerup', () => { drag = null; });
  $('mapCleanupCanvas').addEventListener('pointercancel', () => { drag = null; });
  $('mapCleanupCanvas').addEventListener('wheel', event => {
    event.preventDefault(); state.zoom = Math.max(.1, Math.min(20, state.zoom * Math.exp(-event.deltaY * .001))); draw();
  }, {passive: false});
  window.addEventListener('resize', draw);
  buttons();
})();
