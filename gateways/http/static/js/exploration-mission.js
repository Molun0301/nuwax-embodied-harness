// 单层探索使用原任务和已保存地图，页面只显示后端确认的进度。
(function () {
  'use strict';
  let panel, el, busy = false, requestId = null, taskId = null, active = false, timer, loadedCheckpoint = null;
  const terminal = ['succeeded', 'failed', 'cancelled', 'timeout', 'timed_out'];
  async function request(url, args) {
    const response = await fetch(url, args === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ arguments: args }) });
    const payload = await response.json();
    if (!response.ok || payload.ok === false) {
      const error = new Error(payload.error?.message || payload.detail || '探索请求失败');
      error.status = response.status;
      throw error;
    }
    return payload;
  }
  async function invoke(name, args = {}) { const value = await request('/api/capabilities/' + name + '/invoke', args); return value.result || value; }
  function status(message, failed = false) { el.status.textContent = message; el.status.dataset.level = failed ? 'error' : ''; }
  function buttons() {
    el.start.disabled = busy || active;
    el.pause.disabled = el.stop.disabled = busy || !active || !requestId;
    el.resume.disabled = busy || active || !el.checkpoint.value;
  }
  async function checkpoints() {
    const selected = el.checkpoint.value;
    const data = await invoke('list_exploration_checkpoints');
    el.checkpoint.replaceChildren(new Option('选择已保存的探索', ''));
    for (const item of data.checkpoints || []) {
      const option = new Option(item.map_name + ' · ' + new Date(item.created_at).toLocaleString() + (item.available ? '' : ' · 地图已改变'), item.checkpoint_id);
      option.disabled = !item.available;
      option.title = item.unavailable_reason || '';
      el.checkpoint.append(option);
    }
    if ([...el.checkpoint.options].some(option => option.value === selected && !option.disabled)) el.checkpoint.value = selected;
    buttons();
  }
  async function refresh() {
    const data = await invoke('get_navigation_context');
    const context = data.exploration_context || {};
    const state = context.exploration_state || {};
    const metadata = state.metadata || {};
    const currentRequest = context.current_request || context.exploration_request || {};
    const contextRequestId = currentRequest.request_id || state.current_request_id || metadata.request_id;
    active = ['accepted', 'running', 'active', 'pending', 'stopping'].includes(state.status);
    let task = null;
    if (taskId) {
      try { task = await request('/api/tasks/' + encodeURIComponent(taskId)); }
      catch (error) { if (error.status !== 404) throw error; taskId = null; }
    }
    // 新任务准备期间，几何上下文可能仍属于上一请求，不能覆盖定向停车身份。
    if (task) {
      const taskActive = !terminal.includes(task.status?.state);
      if (!taskActive && active && contextRequestId && contextRequestId !== requestId) {
        taskId = null; task = null;
      } else active = taskActive;
    }
    if (task) {
      const checkpoint = task.exploration_checkpoint;
      if (checkpoint?.status === 'succeeded') {
        status('已停车并保存探索，可从检查点继续。');
        if (loadedCheckpoint !== checkpoint.checkpoint_id) { await checkpoints(); loadedCheckpoint = checkpoint.checkpoint_id; }
      }
      else if (checkpoint?.status === 'failed' || checkpoint?.status === 'rejected') status('检查点保存未完成：' + checkpoint.message, true);
      else if (checkpoint?.status === 'pending') status('正在停车并保存探索进度。');
      else status(task.error_payload?.message || task.error || task.status?.message || '等待任务状态。', ['failed', 'timeout', 'timed_out'].includes(task.status?.state));
    } else {
      requestId = contextRequestId || null;
      if (state.message) status(state.message, state.status === 'failed');
    }
    const progress = contextRequestId === requestId ? metadata.mission_progress : null;
    el.progress.textContent = progress ? '采样行程：' + Number(progress.travelled_m).toFixed(2) + '米' + (progress.returning && active ? ' · 正在返航' : '') : '';
    buttons();
  }
  async function run(action) {
    if (busy) return;
    busy = true; buttons();
    try { await action(); } catch (error) { status(error.message, true); }
    finally { busy = false; buttons(); }
  }
  function track(result) {
    taskId = result.task?.task_id || result.task_id;
    requestId = result.task?.input_payload?.request_id || result.request_id || null;
    loadedCheckpoint = null;
    active = !!taskId;
    status('请求已受理，等待任务准备完成。');
  }
  function mount() {
    panel = document.querySelector('[data-role="exploration-mission"]');
    if (!panel) return;
    el = Object.fromEntries(['map', 'timeout', 'distance', 'reserve', 'return', 'start', 'pause', 'stop', 'checkpoint', 'refresh', 'resume', 'status', 'progress'].map(name => [name, panel.querySelector('[data-role="explore-' + name + '"]')]));
    el.start.addEventListener('click', () => run(async () => {
      const args = { timeout_sec: Number(el.timeout.value) };
      if (!Number.isFinite(args.timeout_sec) || args.timeout_sec <= 0) throw new Error('最长运行时间需要大于零。');
      if (el.map.value.trim()) args.map_name = el.map.value.trim();
      const reserve = Number(el.reserve.value);
      if (!Number.isFinite(reserve) || reserve < 0) throw new Error('返航余量需要是非负米数。');
      if (el.distance.value !== '' || el.return.checked) {
        args.return_policy = { reserve_distance_m: reserve, return_on_completion: el.return.checked };
        if (el.distance.value !== '') {
          const maximum = Number(el.distance.value);
          if (!Number.isFinite(maximum) || maximum <= reserve) throw new Error('总路程预算需要大于返航余量。');
          args.return_policy.maximum_distance_m = maximum;
        } else if (reserve !== 0) throw new Error('填写返航余量时需要同时填写总路程预算。');
      } else if (reserve !== 0) throw new Error('填写返航余量时需要同时填写总路程预算。');
      track(await invoke('explore_area_v2', args));
    }));
    el.pause.addEventListener('click', () => run(async () => { const result = await invoke('pause_exploration', { request_id: requestId }); taskId = result.task_id; status('暂停已受理，等待停车和检查点保存。'); }));
    el.stop.addEventListener('click', () => run(async () => { await invoke('stop_exploration', { request_id: requestId }); status('已请求停止探索，等待原任务完成停车。'); }));
    el.resume.addEventListener('click', () => run(async () => { track(await invoke('resume_exploration', { checkpoint_id: el.checkpoint.value, timeout_sec: Number(el.timeout.value) })); }));
    el.refresh.addEventListener('click', () => run(checkpoints));
    el.checkpoint.addEventListener('change', buttons);
    panel.addEventListener('toggle', () => { if (panel.open) run(async () => { await checkpoints(); await refresh(); }); });
    const poll = async () => { if (panel.open && !document.hidden && !busy) await run(refresh); timer = setTimeout(poll, 2000); };
    timer = setTimeout(poll, 2000);
    window.addEventListener('pagehide', () => clearTimeout(timer));
    buttons();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
