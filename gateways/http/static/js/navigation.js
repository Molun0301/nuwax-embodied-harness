// 选点导航共用后端任务与实时状态，不保存导航回执副本。
(function () {
  'use strict';
  const state = {container: null, active: false, taskId: null, pendingPose: null, submittedPose: null, submitting: false};
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  function setStatus(text, level = '') { state.statusEl.textContent = text; state.statusEl.dataset.level = level; }
  function requestJson(url, options) {
    return fetch(url, options).then(async response => {
      const value = await response.json();
      if (!response.ok) throw new Error(value.error?.message || '导航请求失败');
      return value;
    });
  }
  function invoke(name, args = {}) {
    return requestJson('/api/capabilities/' + name + '/invoke', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({arguments: args})});
  }
  function targetHeight(frameId, point) {
    if (state.heightInput.value.trim()) return Number(state.heightInput.value);
    if (Number.isFinite(point.surface_z)) return point.surface_z - window.pointcloud3d._robotGeometry.z_min_m;
    return null;
  }
  function selectedPoseLabel(pose) { return 'x=' + pose.x.toFixed(2) + ' y=' + pose.y.toFixed(2); }
  function selectMapPoint(x, y, frameId, selection) {
    state.pendingPose = {...selection, x, y, frame_id: frameId || 'map'};
    window.pointcloud3d?.previewNavigationPoint(state.pendingPose);
    setStatus('已选点：' + selectedPoseLabel(state.pendingPose), 'pending');
  }
  function applyTask(task) {
    if (task.task_id !== state.taskId) return;
    state.detailEl.textContent = task.message;
    setStatus(task.message, task.state === 'failed' || task.state === 'timed_out' ? 'error' : task.state);
    if (terminal.includes(task.state)) {
      state.taskId = null;
      if (state.pendingPose === state.submittedPose) { state.pendingPose = null; window.pointcloud3d?.previewNavigationPoint(null); }
    }
  }
  async function refreshTask() { if (state.taskId) applyTask((await requestJson('/api/tasks/' + encodeURIComponent(state.taskId))).task); }
  function captureClickFromMap() {
    window.addEventListener('pointcloud3d:navigation-pick', function (event) {
      if (!window.__nuwaxNavigationActive || state.container.hidden) return;
      const point = event.detail;
      if (!point) {
        state.pendingPose = null;
        window.pointcloud3d?.previewNavigationPoint(null);
        setStatus('未命中地图点，请放大并点击可见地面；拖动可调整视角', 'warn'); return;
      }
      selectMapPoint(point.x, point.y, point.frame_id, point);
    });
    const mapEl = document.getElementById('mapCanvas') || document.getElementById('map') || document.getElementById('map-canvas');
    if (!mapEl) return;
    mapEl.addEventListener('nuwax:map-click', function (ev) {
      const d = ev && ev.detail;
      if (!d) return;
      selectMapPoint(d.x, d.y, d.frame_id);
    });
    let pointerStart = null;
    mapEl.addEventListener('pointerdown', function (ev) {
      if (!state.container || state.container.hidden || ev.button !== 0 || ev.isPrimary === false) return;
      pointerStart = { x: ev.clientX, y: ev.clientY, id: ev.pointerId };
    });
    mapEl.addEventListener('pointercancel', function () { pointerStart = null; });
    mapEl.addEventListener('pointerup', function (ev) {
      const start = pointerStart;
      pointerStart = null;
      if (!start || state.container.hidden || ev.button !== 0 || start.id !== ev.pointerId) return;
      if (Math.hypot(ev.clientX - start.x, ev.clientY - start.y) > 6) return;
      const point = typeof window.__nuwaxMapUnproject === 'function' ? window.__nuwaxMapUnproject(ev.clientX, ev.clientY) : null;
      if (!point) { setStatus('等待可用的二维地图后选点', 'warn'); return; }
      selectMapPoint(point.x, point.y, point.frame_id);
    });
  }

  async function submitNavigation() {
    if (!state.pendingPose || state.submitting) return;
    const pose = state.pendingPose;
    const args = {frame_id: pose.frame_id, x: pose.x, y: pose.y};
    const z = targetHeight(pose.frame_id, pose);
    if (z !== null) args.z = z;
    state.submitting = true;
    try {
      const reply = await invoke('navigate_to_pose', args);
      state.taskId = reply.task.task_id; state.submittedPose = pose;
      applyTask(reply.task); await refreshTask();
    } catch (error) { setStatus(error.message, 'error'); }
    finally { state.submitting = false; }
  }
  async function cancelNavigation() {
    try {
      if (state.taskId) await requestJson('/api/tasks/' + encodeURIComponent(state.taskId) + '/cancel', {method: 'POST'});
      else await invoke('cancel_navigation');
      setStatus('已发送取消，等待任务退出。', 'pending');
    } catch (error) { setStatus(error.message, 'error'); }
  }
  function mount() {
    state.container = document.getElementById('navigation-container');
    if (!state.container) return;
    state.statusEl = state.container.querySelector('[data-role="nav-status"]');
    state.detailEl = state.container.querySelector('[data-role="nav-detail"]');
    state.heightInput = state.container.querySelector('[data-role="nav-height"]');
    state.container.querySelector('[data-role="nav-submit"]').addEventListener('click', submitNavigation);
    state.container.querySelector('[data-role="nav-cancel"]').addEventListener('click', cancelNavigation);
    captureClickFromMap();
    window.addEventListener('stream:tasks.state', event => applyTask(event.detail));
    window.addEventListener('stream:navigation.state', event => {
      const value = event.detail;
      if (state.active && value) state.detailEl.textContent = value.message || value.status || '';
    });
    setStatus('未选点', 'idle');
  }
  window.__nuwaxNavigationModule = {
    setActive(active) { state.active = Boolean(active); state.pendingPose = null; window.pointcloud3d?.previewNavigationPoint(null); if (active) refreshTask().catch(error => setStatus(error.message, 'error')); },
    submit: submitNavigation, cancel: cancelNavigation, getState() { return state; }
  };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
