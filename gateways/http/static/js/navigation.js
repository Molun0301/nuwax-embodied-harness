// navigation.js — V2 navigation-and-exploration-boundary §9.1
// 导航到目标位姿前端模块：
//   - 选点 click handler（地图 canvas / 地图图层 mouseup）
//   - navigate_to_pose 按钮
//   - 状态轮询 2Hz
//   - WS 订阅 navigation.state 流
//   - 状态显示区
// 本模块是 debug_panel.html 的拆模块；不堆业务到主 html。

(function () {
  'use strict';

  const NAV_API = '/api/capabilities/navigate_to_pose/invoke';
  const CANCEL_API = '/api/capabilities/cancel_navigation/invoke';
  const STATE_API = '/api/capabilities/get_navigation_state/invoke';
  const POLL_INTERVAL_MS = 500; // 2Hz
  const WS_STREAM_NAME = 'navigation.state';

  const state = {
    container: null,
    statusEl: null,
    detailEl: null,
    btnNav: null,
    btnCancel: null,
    pendingPose: null,
    inflightRequestId: null,
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
      const cp = payload.current_pose;
      lines.push('当前位姿: x=' + (cp.x || 0).toFixed(2) + ' y=' + (cp.y || 0).toFixed(2));
    }
    if (payload.message) lines.push('说明: ' + payload.message);
    if (payload.failure && payload.failure.code) {
      lines.push('错误: [' + payload.failure.code + '] ' + (payload.failure.message || ''));
    }
    state.detailEl.textContent = lines.join('\n');
  }

  function invokeCapability(url, args) {
    return fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ arguments: args || {} }),
    }).then(function (r) { return r.json(); });
  }

  function captureClickFromMap() {
    // 监听地图 canvas / 已知地图 div 的 mouseup，把最后点击的世界坐标缓存到 state.pendingPose。
    // 主 html 暴露的钩子：window.__nuwaxMapClick 提供 (x, y, frame_id) 回调。
    if (typeof window.__nuwaxMapClick === 'function') {
      const original = window.__nuwaxMapClick;
      window.__nuwaxMapClick = function (x, y, frameId) {
        state.pendingPose = { x: x, y: y, yaw: 0.0, frame_id: frameId || 'map' };
        setStatus('已选点：x=' + x.toFixed(2) + ' y=' + y.toFixed(2), 'pending');
        return original.apply(this, arguments);
      };
      return;
    }
    // 兜底：监听 #map 容器 click 事件（不影响主流程；主 html 仍可由 SVG/canvas 接管）。
    const mapEl = document.getElementById('map') || document.getElementById('map-canvas');
    if (!mapEl) return;
    mapEl.addEventListener('nuwax:map-click', function (ev) {
      const d = ev && ev.detail;
      if (!d) return;
      state.pendingPose = { x: d.x, y: d.y, yaw: 0.0, frame_id: d.frame_id || 'map' };
      setStatus('已选点：x=' + d.x.toFixed(2) + ' y=' + d.y.toFixed(2), 'pending');
    });
  }

  function submitNavigation() {
    if (!state.pendingPose) {
      setStatus('请先在地图上选点', 'warn');
      return;
    }
    setStatus('提交导航中…', 'pending');
    invokeCapability(NAV_API, {
      target_pose: state.pendingPose,
      timeout_sec: 60,
    }).then(function (resp) {
      if (resp && resp.ok === false) {
        setStatus('导航失败：' + (resp.error && resp.error.message || JSON.stringify(resp)), 'error');
        renderDetail({ status: 'failed', failure: resp.error });
        return;
      }
      const taskId = resp && (resp.task_id || resp.id);
      state.inflightRequestId = taskId || null;
      setStatus('已提交：' + (taskId || 'n/a'), 'active');
      startPolling();
    }).catch(function (err) {
      setStatus('网络错误：' + err, 'error');
    });
  }

  function cancelNavigation() {
    invokeCapability(CANCEL_API, { request_id: state.inflightRequestId || '' })
      .then(function (resp) {
        if (resp && resp.ok) {
          setStatus('已发送取消', 'idle');
        } else {
          setStatus('取消失败：' + JSON.stringify(resp), 'warn');
        }
      })
      .catch(function (err) { setStatus('网络错误：' + err, 'error'); });
  }

  function pollOnce() {
    invokeCapability(STATE_API, {}).then(function (resp) {
      if (!resp) return;
      const ns = resp.navigation_state || resp;
      renderDetail(ns);
      if (ns && (ns.status === 'succeeded' || ns.status === 'failed' || ns.status === 'canceled')) {
        stopPolling();
        if (ns.status === 'succeeded') setStatus('已到达', 'success');
        else if (ns.status === 'failed') setStatus('导航失败', 'error');
        else setStatus('已取消', 'idle');
      } else if (ns && ns.status) {
        setStatus('导航中：' + ns.status, 'active');
      }
    }).catch(function () { /* 静默轮询错误 */ });
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
    submit: submitNavigation,
    cancel: cancelNavigation,
    getState: function () { return state; },
  };
})();
