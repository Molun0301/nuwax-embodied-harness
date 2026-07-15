// boundary_draw.js — V2 navigation-and-exploration-boundary §9.2
// 手划多段线段前端模块（探索过程中实时划线）。
//
// 关键设计：
// 1. SVG viewBox 跟随相机当前可见的世界米范围——线段用世界坐标画，
//    探索中机器人移动 / 缩放 / 平移时线段自然跟随。
// 2. SVG 默认 Y 朝下、世界坐标 Y 朝上——绘制前对线段端点做 y' = cy - y 镜像。
// 3. 主 html 地图 IIFE 内部暴露 window.__nuwaxMapWorldViewport() 与
//    window.__nuwaxMapDrawTick；本模块在 mount 时把 syncViewBox 注册为 tick 回调。
// 4. 依赖主 html 暴露的钩子（必须在 boundary_draw.js 之前定义）：
//    window.__nuwaxMapUnproject(screenX, screenY) -> {x, y, frame_id} | null
//    window.__nuwaxMapWorldViewport() -> {cx, cy, halfX, halfY, scale, pixelWidth, pixelHeight} | null
//    window.__nuwaxMapDrawTick（被本模块挂回调）
//    window.__nuwaxMapToolV2.active() -> bool
// 5. 激活时设 window.__nuwaxBoundaryActive=true，让 canvas pointerdown/move/up 提前 return。
//
// 探索过程中画线：
// - canvas 每帧 requestAnimationFrame 重绘，主循环 drawMap() 末尾调 __nuwaxMapDrawTick()
// - 本模块的 syncViewBox 被调用：拉最新 viewport → 算 viewBox 字符串 → SVG 自动重排
// - 线段端点是世界米，跟 canvas 当前帧的世界坐标完全一致
// - 用户拖拽时 onPointerDown/Move/Up 都调 __nuwaxMapUnproject 拿世界米，存入 segments
// - segments 持久跨帧，redrawSegments() 在每帧 tick 里用最新 viewport 镜像 + 重画

(function () {
  'use strict';

  const EXPLORATION_API = '/api/capabilities/explore_area_v2/invoke';
  const FRAME_ID = 'map';
  const SVG_NS = 'http://www.w3.org/2000/svg';
  const OVERLAY_HOST_SELECTOR = '.canvas-wrap';

  const state = {
    container: null,
    btnToggle: null,
    btnSubmit: null,
    btnClear: null,
    listEl: null,
    overlaySvg: null,
    overlayHost: null,
    // 当前相机 viewport（世界米坐标系）；drawTick 每帧刷新
    viewport: null, // {cx, cy, halfX, halfY, scale, pixelWidth, pixelHeight}
    drawing: false,
    activePointerId: null,
    currentStart: null, // world 坐标 {x, y}（Y 朝上）
    currentEnd: null,   // 实时预览端点 {x, y}
    segments: [],       // [{a:{x,y}, b:{x,y}}] 世界米
  };

  function setV2ActiveFlag(on) {
    try { window.__nuwaxBoundaryActive = !!on; } catch (_) {}
  }

  function worldFromEvent(ev) {
    if (typeof window.__nuwaxMapUnproject !== 'function') return null;
    return window.__nuwaxMapUnproject(ev.clientX, ev.clientY);
  }

  function getViewport() {
    if (typeof window.__nuwaxMapWorldViewport !== 'function') return null;
    return window.__nuwaxMapWorldViewport();
  }

  // 世界坐标 (Y-up) → SVG 用户坐标 (Y-down)：y' = cy - y
  function mirrorY(point) {
    if (!state.viewport || !point) return point;
    return { x: point.x, y: state.viewport.cy - point.y };
  }

  function makePoint(x, y) {
    const el = document.createElementNS(SVG_NS, 'circle');
    el.setAttribute('cx', String(x));
    el.setAttribute('cy', String(y));
    el.setAttribute('r', '3');
    el.setAttribute('fill', '#f59e0b');
    return el;
  }

  function makeLine(a, b) {
    const el = document.createElementNS(SVG_NS, 'line');
    el.setAttribute('x1', String(a.x));
    el.setAttribute('y1', String(a.y));
    el.setAttribute('x2', String(b.x));
    el.setAttribute('y2', String(b.y));
    el.setAttribute('stroke', '#f59e0b');
    el.setAttribute('stroke-width', '0.15'); // 世界米单位，0.15m 视觉粗细
    el.setAttribute('stroke-linecap', 'round');
    return el;
  }

  function clearOverlay() {
    if (!state.overlaySvg) return;
    while (state.overlaySvg.firstChild) state.overlaySvg.removeChild(state.overlaySvg.firstChild);
  }

  function redrawSegments() {
    if (!state.overlaySvg || !state.viewport) return;
    clearOverlay();
    for (const seg of state.segments) {
      const a = mirrorY(seg.a);
      const b = mirrorY(seg.b);
      state.overlaySvg.appendChild(makeLine(a, b));
      state.overlaySvg.appendChild(makePoint(a.x, a.y));
      state.overlaySvg.appendChild(makePoint(b.x, b.y));
    }
    // 实时预览（按下鼠标到松开前的临时线段）
    if (state.drawing && state.currentStart && state.currentEnd) {
      const a = mirrorY(state.currentStart);
      const b = mirrorY(state.currentEnd);
      state.overlaySvg.appendChild(makeLine(a, b));
      state.overlaySvg.appendChild(makePoint(a.x, a.y));
      state.overlaySvg.appendChild(makePoint(b.x, b.y));
    }
  }

  function renderList() {
    if (!state.listEl) return;
    state.listEl.innerHTML = '';
    if (!state.segments.length) {
      const li = document.createElement('li');
      li.textContent = '（暂无线段）';
      li.style.opacity = '0.6';
      state.listEl.appendChild(li);
      return;
    }
    state.segments.forEach(function (seg, idx) {
      const li = document.createElement('li');
      li.textContent = '#' + (idx + 1) + ' (' +
        seg.a.x.toFixed(2) + ',' + seg.a.y.toFixed(2) + ') -> (' +
        seg.b.x.toFixed(2) + ',' + seg.b.y.toFixed(2) + ')';
      state.listEl.appendChild(li);
    });
  }

  function findOverlayHost() {
    if (state.overlayHost && document.contains(state.overlayHost)) return state.overlayHost;
    state.overlayHost = document.querySelector(OVERLAY_HOST_SELECTOR);
    return state.overlayHost;
  }

  function ensureOverlay() {
    if (state.overlaySvg && state.overlaySvg.isConnected) return state.overlaySvg;
    const host = findOverlayHost();
    if (!host) return null;
    state.overlaySvg = document.createElementNS(SVG_NS, 'svg');
    state.overlaySvg.id = 'boundary-draw-overlay';
    state.overlaySvg.setAttribute('width', '100%');
    state.overlaySvg.setAttribute('height', '100%');
    // z-index:2 让 toolbar/legend（HUD z-index:5）仍在 SVG 上方可点
    state.overlaySvg.style.cssText = [
      'position:absolute',
      'left:0',
      'top:0',
      'width:100%',
      'height:100%',
      'pointer-events:auto',
      'z-index:2',
      'cursor:crosshair',
      'touch-action:none',
      'overflow:visible',
    ].join(';');
    state.overlaySvg.style.display = state.drawing ? 'block' : 'none';
    host.appendChild(state.overlaySvg);
    return state.overlaySvg;
  }

  // drawTick 回调：每帧拉最新 viewport，更新 viewBox，重绘所有线段
  function syncViewBox() {
    const vp = getViewport();
    if (!vp) {
      state.viewport = null;
      return;
    }
    state.viewport = vp;
    if (!state.overlaySvg) return;
    // viewBox 用世界米：cx-halfX, cy-halfY, 2*halfX, 2*halfY
    const vb = [
      vp.cx - vp.halfX,
      vp.cy - vp.halfY,
      vp.halfX * 2,
      vp.halfY * 2,
    ].join(' ');
    state.overlaySvg.setAttribute('viewBox', vb);
    // preserveAspectRatio 默认 xMidYMid meet，跟 canvas 的居中/等比一致
    redrawSegments();
  }

  function setDrawing(on) {
    state.drawing = !!on;
    if (state.drawing) {
      const overlay = ensureOverlay();
      if (!overlay) {
        state.drawing = false;
        if (state.btnToggle) state.btnToggle.dataset.active = 'false';
        if (state.listEl) state.listEl.dataset.lastError = '找不到地图容器 .canvas-wrap';
        return;
      }
      overlay.style.display = 'block';
      setV2ActiveFlag(true);
      // V2 冻结相机：写入冻结中心 + 开关。cameraAnchor() 入口会优先判断此开关，
      // 强制走 mapFixed 路径——canvas 不再 robotLocked 跟随 pose 旋转/平移。
      // 冻结中心取当前 anchor.point：此刻世界米坐标系中心。
      const anchor = (typeof window.__nuwaxCameraAnchor === 'function') ? window.__nuwaxCameraAnchor() : null;
      if (anchor && anchor.point) {
        window.__nuwaxMapFreezeCenter = { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
      } else {
        // 兜底：第一次划线 / 还没收到 pose 时，中心设 (0,0)
        window.__nuwaxMapFreezeCenter = { x: 0, y: 0 };
      }
      window.__nuwaxMapFreeze = true;
      // 立即拉一次 viewport，不然首次画线前 SVG 还没 viewBox
      syncViewBox();
    } else {
      if (state.overlaySvg) state.overlaySvg.style.display = 'none';
      state.currentStart = null;
      state.currentEnd = null;
      state.activePointerId = null;
      setV2ActiveFlag(false);
      // V2 解冻：清掉开关与冻结中心，cameraAnchor() 下帧恢复 pose-locked 跟随
      try { window.__nuwaxMapFreeze = false; } catch (_) {}
      try { delete window.__nuwaxMapFreezeCenter; } catch (_) {}
    }
    if (state.btnToggle) state.btnToggle.dataset.active = state.drawing ? 'true' : 'false';
    if (state.btnToggle) state.btnToggle.textContent = state.drawing ? '停止划线' : '开始划线';
  }

  function onPointerDown(ev) {
    if (!state.drawing) return;
    if (ev.pointerType === 'mouse' && ev.button !== 0) return;
    ev.preventDefault();
    ev.stopPropagation();
    const w = worldFromEvent(ev);
    if (!w) return;
    state.currentStart = { x: w.x, y: w.y };
    state.currentEnd = { x: w.x, y: w.y };
    state.activePointerId = ev.pointerId;
    if (state.overlaySvg && state.overlaySvg.setPointerCapture) {
      try { state.overlaySvg.setPointerCapture(ev.pointerId); } catch (_) {}
    }
    redrawSegments();
  }

  function onPointerMove(ev) {
    if (!state.drawing || !state.currentStart) return;
    if (state.activePointerId !== null && ev.pointerId !== state.activePointerId) return;
    const w = worldFromEvent(ev);
    if (!w) return;
    state.currentEnd = { x: w.x, y: w.y };
    redrawSegments();
  }

  function onPointerUp(ev) {
    if (!state.drawing || !state.currentStart) return;
    if (state.activePointerId !== null && ev.pointerId !== state.activePointerId) return;
    const w = worldFromEvent(ev);
    if (!w) return;
    // 拖拽距离 < 5cm 视为误点，不入段
    const dx = w.x - state.currentStart.x;
    const dy = w.y - state.currentStart.y;
    if (Math.hypot(dx, dy) >= 0.05) {
      state.segments.push({
        a: { x: state.currentStart.x, y: state.currentStart.y },
        b: { x: w.x, y: w.y },
      });
      renderList();
    }
    state.currentStart = null;
    state.currentEnd = null;
    state.activePointerId = null;
    if (state.overlaySvg && state.overlaySvg.releasePointerCapture) {
      try { state.overlaySvg.releasePointerCapture(ev.pointerId); } catch (_) {}
    }
    redrawSegments();
  }

  function onPointerCancel(ev) {
    if (!state.drawing) return;
    state.currentStart = null;
    state.currentEnd = null;
    state.activePointerId = null;
    redrawSegments();
  }

  function clearAll() {
    state.segments = [];
    state.currentStart = null;
    state.currentEnd = null;
    state.activePointerId = null;
    redrawSegments();
    renderList();
  }

  function submitBoundary() {
    if (state.segments.length < 1) {
      if (state.listEl) state.listEl.dataset.lastError = '至少需要 1 段线段';
      return;
    }
    const payload = {
      boundary_line_segments: state.segments.map(function (seg) {
        return [
          { x: seg.a.x, y: seg.a.y, yaw: 0.0, frame_id: FRAME_ID },
          { x: seg.b.x, y: seg.b.y, yaw: 0.0, frame_id: FRAME_ID },
        ];
      }),
    };
    fetch(EXPLORATION_API, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ arguments: payload }),
    }).then(function (r) { return r.json(); }).then(function (resp) {
      if (state.listEl) {
        state.listEl.dataset.lastSubmit = JSON.stringify({
          at: new Date().toISOString(),
          ok: !(resp && resp.ok === false),
          task_id: (resp && (resp.task_id || resp.id)) || null,
        });
      }
    }).catch(function (err) {
      if (state.listEl) state.listEl.dataset.lastError = String(err);
    });
  }

  function bindOverlay(overlay) {
    overlay.addEventListener('pointerdown', onPointerDown);
    overlay.addEventListener('pointermove', onPointerMove);
    overlay.addEventListener('pointerup', onPointerUp);
    overlay.addEventListener('pointercancel', onPointerCancel);
  }

  function mount() {
    state.container = document.getElementById('boundary-draw-container');
    if (!state.container) return;
    state.btnToggle = state.container.querySelector('[data-role="boundary-toggle"]');
    state.btnSubmit = state.container.querySelector('[data-role="boundary-submit"]');
    state.btnClear = state.container.querySelector('[data-role="boundary-clear"]');
    state.listEl = state.container.querySelector('[data-role="boundary-list"]');
    if (state.btnToggle) {
      state.btnToggle.addEventListener('click', function () { setDrawing(!state.drawing); });
      state.btnToggle.textContent = '开始划线';
      state.btnToggle.dataset.active = 'false';
    }
    if (state.btnSubmit) state.btnSubmit.addEventListener('click', submitBoundary);
    if (state.btnClear) state.btnClear.addEventListener('click', clearAll);
    const overlay = ensureOverlay();
    if (overlay) bindOverlay(overlay);
    // 注册为 draw tick 回调：每帧同步 viewBox + 重绘
    window.__nuwaxMapDrawTick = syncViewBox;
    renderList();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', mount);
  } else {
    mount();
  }

  window.__nuwaxBoundaryDrawModule = {
    getSegments: function () { return state.segments.slice(); },
    setDrawing: setDrawing,
    clear: clearAll,
  };
})();
