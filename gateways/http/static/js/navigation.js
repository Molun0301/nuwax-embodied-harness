// 选点导航共用后端任务与实时状态，不保存导航回执副本。
(function () {
  'use strict';
  const state = {container: null, active: false, taskId: null, pendingPose: null, submittedPose: null, submittedGoalId: null, submitting: false};
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  const PICK_HINT = '未选点：点击地图选点，按住拖动设定朝向';
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
  function selectedPoseLabel(pose) {
    const position = 'x=' + pose.x.toFixed(2) + ' y=' + pose.y.toFixed(2);
    return Number.isFinite(pose.yaw) ? position + ' 朝向' + Math.round(pose.yaw * 180 / Math.PI) + '°'
      : position + '，未定朝向（按住拖动设定朝向）';
  }
  // 选点标记同时画在三维与二维地图上。
  function showPick(pose) {
    window.pointcloud3d?.previewNavigationPoint(pose);
    window.__nuwaxMapRequestDraw();
  }
  function selectMapPoint(x, y, frameId, selection) {
    state.pendingPose = {...selection, x, y, frame_id: frameId || 'map'};
    showPick(state.pendingPose);
    setStatus('已选点：' + selectedPoseLabel(state.pendingPose), 'pending');
  }
  function applyTask(task) {
    if (task.task_id !== state.taskId) return;
    // 后端回传的目标不区分“未指定朝向”，按任务目标号记住本页提交时的选择，并让地图按选择重画目标。
    if (task.data?.goal_id && task.data.goal_id !== state.submittedGoalId) {
      state.submittedGoalId = task.data.goal_id;
      window.dispatchEvent(new CustomEvent('navigation:submitted-goal', {detail: {goal_id: state.submittedGoalId}}));
    }
    state.detailEl.textContent = task.message;
    setStatus(task.message, task.state === 'failed' || task.state === 'timed_out' ? 'error' : task.state);
    if (terminal.includes(task.state)) {
      state.taskId = null;
      if (state.pendingPose === state.submittedPose) { state.pendingPose = null; showPick(null); }
    }
  }
  async function refreshTask() { if (state.taskId) applyTask((await requestJson('/api/tasks/' + encodeURIComponent(state.taskId))).task); }
  function captureClickFromMap() {
    window.addEventListener('pointcloud3d:navigation-pick', function (event) {
      if (!window.__nuwaxNavigationActive || state.container.hidden) return;
      const point = event.detail;
      if (!point) {
        state.pendingPose = null;
        showPick(null);
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
    // 二维地图：按下处为位置，拖动方向为朝向；导航选点时二维地图本身不旋转不平移。
    let drag = null;
    function unproject(ev) {
      return typeof window.__nuwaxMapUnproject === 'function' ? window.__nuwaxMapUnproject(ev.clientX, ev.clientY) : null;
    }
    mapEl.addEventListener('pointerdown', function (ev) {
      if (!window.__nuwaxNavigationActive || ev.button !== 0 || ev.isPrimary === false) return;
      const point = unproject(ev);
      if (!point) { setStatus('等待可用的二维地图后选点', 'warn'); return; }
      drag = {id: ev.pointerId, x: ev.clientX, y: ev.clientY, point, dragged: false};
      mapEl.setPointerCapture(ev.pointerId);
      selectMapPoint(point.x, point.y, point.frame_id);
    });
    mapEl.addEventListener('pointermove', function (ev) {
      if (!drag || drag.id !== ev.pointerId) return;
      if (!drag.dragged && Math.hypot(ev.clientX - drag.x, ev.clientY - drag.y) <= 6) return;
      const point = unproject(ev);
      if (!point) return;
      drag.dragged = true;
      state.pendingPose = {...state.pendingPose, yaw: Math.atan2(point.y - drag.point.y, point.x - drag.point.x)};
      showPick(state.pendingPose);
    });
    function finishDrag(ev) {
      if (!drag || drag.id !== ev.pointerId) return;
      drag = null;
      setStatus('已选点：' + selectedPoseLabel(state.pendingPose), 'pending');
    }
    mapEl.addEventListener('pointerup', finishDrag);
    mapEl.addEventListener('pointercancel', finishDrag);
  }

  async function submitNavigation() {
    if (!state.pendingPose || state.submitting) return;
    const pose = state.pendingPose;
    const args = {frame_id: pose.frame_id, x: pose.x, y: pose.y};
    const z = targetHeight(pose.frame_id, pose);
    if (z !== null) args.z = z;
    if (Number.isFinite(pose.yaw)) args.yaw_rad = pose.yaw;
    state.submitting = true;
    try {
      const reply = await invoke('navigate_to_pose', args);
      state.taskId = reply.task.task_id; state.submittedPose = pose; state.submittedGoalId = null;
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
    setStatus(PICK_HINT, 'idle');
  }
  window.__nuwaxNavigationModule = {
    setActive(active) { state.active = Boolean(active); state.pendingPose = null; showPick(null); if (active) refreshTask().catch(error => setStatus(error.message, 'error')); },
    // 本页提交时没选朝向的目标，地图按不带方向显示。
    goalOrientationUnset(goalId) {
      return Boolean(goalId && goalId === state.submittedGoalId && !Number.isFinite(state.submittedPose?.yaw));
    },
    submit: submitNavigation, cancel: cancelNavigation, getState() { return state; }
  };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
