// navigation.js — V2 navigation-and-exploration-boundary §9.1
// 导航到目标位姿前端模块：
//   - 选点 click handler（地图 canvas / 地图图层 mouseup）
//   - navigate_to_pose 按钮
//   - 状态轮询 2Hz
//   - WS 订阅 navigation.state 流
//   - 状态显示区
// 控制台导航交互模块，通过统一网关接口提交任务。

(function () {
  'use strict';

  const NAV_API = '/api/capabilities/navigate_to_pose/invoke';
  const CANCEL_API = '/api/capabilities/cancel_navigation/invoke';
  const POLL_INTERVAL_MS = 500; // 2Hz
  const WS_STREAM_NAME = 'navigation.state';

  const state = {
    container: null,
    statusEl: null,
    detailEl: null,
    btnNav: null,
    btnCancel: null,
    heightInput: null,
    pendingPose: null,
    inflightRequestId: null,
    submittedPose: null,
    submitting: false,
    pollTimer: null,
    wsToken: null,
    mapClickHandler: null,
  };

  function setStatus(text, level) {
    if (!state.statusEl) return;
    state.statusEl.textContent = text || '--';
    state.statusEl.dataset.level = level || 'idle';
  }

  function renderDetail(payload) {
    if (!state.detailEl) return;
    if (!payload) {
      state.detailEl.textContent = '';
      return;
    }
    const lines = [];
    if (payload.status) lines.push('状态: ' + payload.status);
    if (typeof payload.remaining_distance_m === 'number') {
      lines.push('剩余距离: ' + payload.remaining_distance_m.toFixed(2) + ' m');
    }
    if (payload.current_pose) {
      const cp = payload.current_pose.position || payload.current_pose;
      const coordinates = ['x', 'y', 'z'].filter(function (axis) { return typeof cp[axis] === 'number' && Number.isFinite(cp[axis]); });
      if (coordinates.length) lines.push('当前位姿: ' + coordinates.map(function (axis) { return axis + '=' + cp[axis].toFixed(2); }).join(' '));
    }
    if (payload.message) lines.push('说明: ' + payload.message);
    if (payload.failure && payload.failure.code) {
      lines.push('错误: [' + payload.failure.code + '] ' + (payload.failure.message || ''));
    }
    // 详情与状态同处二级栏的一行说明中，用分隔符代替换行，避免栏高随导航反馈跳动。
    state.detailEl.textContent = lines.join(' · ');
  }

  function invokeCapability(url, args) {
    return requestJson(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ arguments: args || {} }),
    });
  }

  function requestJson(url, options) {
    return fetch(url, options).then(function (response) {
      return response.json().then(function (payload) {
        if (!response.ok || payload.ok === false) {
          const error = payload.error || payload;
          throw new Error(error.message || payload.detail || '请求失败：HTTP ' + response.status);
        }
        return payload;
      });
    });
  }

  function selectedPoseLabel(pose) {
    const height = typeof pose.z === 'number' && Number.isFinite(pose.z) ? pose.z.toFixed(2) + ' 米' : '待填写';
    return 'x=' + pose.x.toFixed(2) + ' y=' + pose.y.toFixed(2) + ' · 机体高度 ' + height;
  }

  function targetHeight(frameId, selection) {
    if (state.heightInput?.validity?.badInput) throw new Error('目标机体高度必须是有限数值（米）。');
    if (selection && Number.isFinite(selection.surface_z) && !String(state.heightInput?.value || '').trim()) {
      const distance = window.__NUWAX_NAV_SUPPORT_DISTANCE__;
      if (typeof distance !== 'number' || !Number.isFinite(distance) || distance <= 0) {
        throw new Error('请先在平台设置中填写机体到支撑面距离，保存并重启服务后刷新页面。');
      }
      // 选取的是地面点；机体目标沿地图正Z轴抬高标定距离，最终朝向不作要求。
      return selection.surface_z + distance;
    }
    if (typeof window.__nuwaxResolveNavigationHeight !== 'function') {
      throw new Error('定位高度入口不可用，请刷新控制台。');
    }
    return window.__nuwaxResolveNavigationHeight(state.heightInput, frameId);
  }

  function selectMapPoint(x, y, frameId, selection) {
    if (![x, y].every(function (value) { return typeof value === 'number' && Number.isFinite(value); })) return;
    const pose = Object.assign({}, selection, { x: x, y: y, z: null, frame_id: frameId || 'map', map_name: window.__nuwaxActiveMapName?.() });
    try { pose.z = targetHeight(pose.frame_id, pose); } catch (_) { /* 没有真实高度时保留待填，不猜楼层。 */ }
    state.pendingPose = pose;
    window.pointcloud3d?.previewNavigationPoint(pose);
    const surface = Number.isFinite(pose.surface_z) ? ' · 地面高度 ' + pose.surface_z.toFixed(2) + ' 米' : '';
    setStatus('已选点：' + selectedPoseLabel(pose) + surface, 'pending');
  }

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

  function submitNavigation() {
    if (state.submitting || state.inflightRequestId) {
      setStatus('上一条导航尚未结束，请先取消或等待完成', 'warn');
      return;
    }
    if (!state.pendingPose) {
      setStatus('请先在地图上选点', 'warn');
      return;
    }
    let pose;
    try {
      const pending = state.pendingPose;
      const mapName = window.__nuwaxActiveMapName?.();
      if (!mapName || pending.map_name !== mapName || (pending.source_key && !window.pointcloud3d?.navigationPointIsCurrent(pending))) {
        throw new Error('地图或点云来源已切换，请重新选点。');
      }
      pose = Object.assign({}, pending, { z: targetHeight(pending.frame_id, pending) });
    } catch (err) {
      setStatus(err.message, 'warn');
      return;
    }
    state.pendingPose = pose;
    state.submitting = true;
    setStatus('提交导航中：' + selectedPoseLabel(pose), 'pending');
    invokeCapability(NAV_API, {
      map_name: pose.map_name,
      frame_id: pose.frame_id,
      x: pose.x,
      y: pose.y,
      z: pose.z,
    }).then(function (resp) {
      const taskId = resp && resp.task_id;
      if (!taskId) throw new Error('网关响应缺少导航任务标识，无法确认受理。');
      state.inflightRequestId = taskId;
      state.submittedPose = pose;
      setStatus('已提交：' + taskId + ' · ' + selectedPoseLabel(pose), 'active');
      startPolling();
    }).catch(function (err) {
      setStatus('导航提交失败：' + err.message, 'error');
    }).finally(function () {
      state.submitting = false;
    });
  }

  function cancelNavigation() {
    if (state.submitting) { setStatus('正在确认导航受理，请稍候再取消', 'warn'); return; }
    const taskId = state.inflightRequestId;
    // 本面板持有的是平台任务标识，不能把它当作后端导航目标标识。
    const cancellation = taskId
      ? requestJson('/api/tasks/' + encodeURIComponent(taskId) + '/cancel', { method: 'POST' })
      : invokeCapability(CANCEL_API, {});
    cancellation
      .then(function (resp) {
        const result = taskId ? resp : resp.result;
        if (!result) throw new Error('网关响应缺少取消结果');
        if (taskId && result.cancelled !== true) setStatus('该任务未接受取消请求，正在读取当前状态', 'warn');
        else setStatus(result.completed === true ? '导航已结束' : '取消请求已发送，等待导航结束', 'pending');
        if (taskId) startPolling();
      })
      .catch(function (err) { setStatus('取消失败：' + err.message, 'error'); });
  }

  function pollOnce() {
    const taskId = state.inflightRequestId;
    if (!taskId) return;
    requestJson('/api/tasks/' + encodeURIComponent(taskId)).then(function (snapshot) {
      if (state.inflightRequestId !== taskId) return;
      const status = snapshot.status || {};
      const result = snapshot.result || {};
      const nav = result.navigation_context && result.navigation_context.navigation_state;
      renderDetail(nav || { status: status.state, message: status.message || snapshot.error });
      const selected = state.submittedPose ? ' · ' + selectedPoseLabel(state.submittedPose) : '';
      if (['succeeded', 'failed', 'cancelled', 'timeout'].includes(status.state)) {
        stopPolling();
        state.inflightRequestId = null;
        if (status.state === 'succeeded') setStatus('已到达' + selected, 'success');
        else if (status.state === 'cancelled') setStatus('已取消' + selected, 'idle');
        else setStatus((status.state === 'timeout' ? '导航超时' : '导航失败') + selected, 'error');
      } else {
        setStatus((status.message || '导航执行中') + selected, 'active');
      }
    }).catch(function (err) { setStatus('导航状态读取失败：' + err.message, 'warn'); });
  }

  function startPolling() {
    stopPolling();
    state.pollTimer = setInterval(pollOnce, POLL_INTERVAL_MS);
  }

  function stopPolling() {
    if (state.pollTimer) {
      clearInterval(state.pollTimer);
      state.pollTimer = null;
    }
  }

  function subscribeWs() {
    if (typeof window.__nuwaxWsSubscribe === 'function') {
      state.wsToken = window.__nuwaxWsSubscribe(WS_STREAM_NAME, function (msg) {
        if (!msg) return;
        if (msg.data && msg.data.state) {
          renderDetail(msg.data.state);
        }
      });
    }
  }

  function mount() {
    state.container = document.getElementById('navigation-container');
    if (!state.container) return;
    state.statusEl = state.container.querySelector('[data-role="nav-status"]');
    state.detailEl = state.container.querySelector('[data-role="nav-detail"]');
    state.btnNav = state.container.querySelector('[data-role="nav-submit"]');
    state.btnCancel = state.container.querySelector('[data-role="nav-cancel"]');
    state.heightInput = state.container.querySelector('[data-role="nav-height"]');
    if (state.heightInput) state.heightInput.addEventListener('input', function () {
      const pose = state.pendingPose;
      if (pose) {
        try { pose.z = targetHeight(pose.frame_id, pose); } catch (_) { pose.z = null; }
        window.pointcloud3d?.previewNavigationPoint(pose);
        setStatus('已选点：' + selectedPoseLabel(pose), 'pending');
      }
    });
    if (state.btnNav) state.btnNav.addEventListener('click', submitNavigation);
    if (state.btnCancel) state.btnCancel.addEventListener('click', cancelNavigation);
    captureClickFromMap();
    subscribeWs();
    setStatus('未选点', 'idle');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', mount);
  } else {
    mount();
  }

  window.__nuwaxNavigationModule = {
    setActive: function (active) {
      state.pendingPose = null;
      window.pointcloud3d?.previewNavigationPoint(null);
      if (state.heightInput) state.heightInput.placeholder = window.__nuwaxMapViewMode?.() === '3d'
        ? '留空使用地面高度＋机体支撑距离' : '留空使用当前定位高度';
      if (active && !state.inflightRequestId) setStatus('请在地图上点击可通行地面选择目标', 'idle');
    },
    submit: submitNavigation,
    cancel: cancelNavigation,
    getState: function () { return state; },
  };
})();
