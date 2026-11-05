// 单层探索只读取真实任务、Geometry 状态和已保存的进度。
(function () {
  'use strict';
  let panel, el, taskId = null, explorationId = null, active = false, busy = false, loadedCheckpoint = null;
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  async function request(url, args) {
    const response = await fetch(url, args === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({arguments: args})});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error?.message || '探索请求失败');
    return value;
  }
  function invoke(name, args = {}) { return request('/api/capabilities/' + name + '/invoke', args); }
  function status(message, failed = false) { el.status.textContent = message; el.status.dataset.level = failed ? 'error' : ''; }
  function buttons() {
    el.start.disabled = busy || active;
    el.pause.disabled = el.stop.disabled = busy || !active || !explorationId;
    el.resume.disabled = busy || active || !el.checkpoint.value;
  }
  async function checkpoints() {
    const selected = el.checkpoint.value;
    const data = await invoke('list_exploration_checkpoints');
    el.checkpoint.replaceChildren(new Option('选择已保存的探索', ''));
    for (const item of data.checkpoints) {
      const option = new Option(item.map_name + ' · ' + new Date(item.created_at).toLocaleString() + (item.available ? '' : ' · 地图已删除'), item.checkpoint_id);
      option.disabled = !item.available; el.checkpoint.append(option);
    }
    if ([...el.checkpoint.options].some(option => option.value === selected && !option.disabled)) el.checkpoint.value = selected;
    buttons();
  }
  function applyTask(task) {
    if (task.task_id !== taskId) return;
    active = !terminal.includes(task.state);
    if (task.data.exploration_id) explorationId = task.data.exploration_id;
    const saved = task.result;
    if (saved?.paused) {
      status('已停止探索并保存地图和进度。');
      if (loadedCheckpoint !== saved.checkpoint_id) { loadedCheckpoint = saved.checkpoint_id; checkpoints().catch(error => status(error.message, true)); }
    } else status(task.message, ['failed', 'timed_out'].includes(task.state));
    const progress = task.data.exploration;
    el.progress.textContent = progress ? '行程：' + Number(progress.traveled_distance_m).toFixed(2) + ' 米' + (progress.returning ? ' · 返航中' : '') : '';
    buttons();
  }
  async function refresh() {
    const state = await invoke('get_navigation_state');
    explorationId = state.exploration?.exploration_id || null;
    const tasks = await request('/api/tasks');
    if (!taskId) taskId = tasks.active_tasks.find(task => ['explore_area_v2', 'resume_exploration', 'explore_building'].includes(task.capability))?.task_id || null;
    if (taskId) applyTask((await request('/api/tasks/' + encodeURIComponent(taskId))).task);
    else { active = state.exploration?.phase === 'running'; status(state.exploration?.message || '等待操作。'); buttons(); }
  }
  async function track(reply) { taskId = reply.task.task_id; loadedCheckpoint = null; applyTask(reply.task); await refresh(); }
  async function run(action) {
    if (busy) return;
    busy = true; buttons();
    try { await action(); } catch (error) { status(error.message, true); }
    finally { busy = false; buttons(); }
  }
  function mount() {
    panel = document.querySelector('[data-role="exploration-mission"]');
    if (!panel) return;
    el = Object.fromEntries(['map', 'timeout', 'distance', 'reserve', 'return', 'start', 'pause', 'stop', 'checkpoint', 'refresh', 'resume', 'status', 'progress'].map(name => [name, panel.querySelector('[data-role="explore-' + name + '"]')]));
    el.start.addEventListener('click', () => run(async () => {
      const args = {timeout_sec: Number(el.timeout.value)};
      if (el.map.value.trim()) args.map_name = el.map.value.trim();
      if (el.distance.value !== '' || el.return.checked) {
        args.return_policy = {reserve_distance_m: Number(el.reserve.value), return_on_completion: el.return.checked};
        if (el.distance.value !== '') args.return_policy.maximum_distance_m = Number(el.distance.value);
      }
      await track(await invoke('explore_area_v2', args));
    }));
    el.pause.addEventListener('click', () => run(async () => { const reply = await invoke('pause_exploration', {exploration_id: explorationId}); taskId = reply.task_id; status('暂停已受理，正在停止探索并保存。'); }));
    el.stop.addEventListener('click', () => run(async () => { await invoke('stop_exploration', {exploration_id: explorationId}); status('已发送停止探索命令，等待任务退出。'); }));
    el.resume.addEventListener('click', () => run(async () => { await track(await invoke('resume_exploration', {checkpoint_id: el.checkpoint.value, timeout_sec: Number(el.timeout.value)})); }));
    el.refresh.addEventListener('click', () => run(async () => { await checkpoints(); await refresh(); }));
    el.checkpoint.addEventListener('change', buttons);
    panel.addEventListener('toggle', () => { if (panel.open) run(async () => { await checkpoints(); await refresh(); }); });
    window.addEventListener('stream:tasks.state', event => applyTask(event.detail));
    window.addEventListener('stream:exploration.state', event => { if (event.detail) explorationId = event.detail.exploration_id; buttons(); });
    buttons();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
