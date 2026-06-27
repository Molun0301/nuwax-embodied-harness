// boundary_draw.js — V2 navigation-and-exploration-boundary §9.2
// 手划多段线段前端模块：
//   - 划线模式按钮
//   - SVG overlay（多线段拖拽）—— 挂在 .canvas-wrap，覆盖在 #mapCanvas 上方
//   - pointerdown/pointermove/pointerup 事件（含 setPointerCapture，避免丢帧）
//   - 多段线段列表 UI
//   - 提交 POST /api/capabilities/explore_area_v2/invoke
// 本模块是 debug_panel.html 的拆模块；不堆业务到主 html。
//
// 依赖主 html 暴露的钩子（必须在 boundary_draw.js 之前定义）：
//   window.__nuwaxMapUnproject(screenX, screenY) -> {x, y, frame_id} | null
//   window.__nuwaxMapToolV2.active() -> bool   // 主循环用此判断让事件
//
// 激活时设 window.__nuwaxBoundaryActive=true，让 canvas pointerdown/move/up 提前 return；
// 否则主循环会 setPointerCapture + preventDefault，SVG 收不到任何合成 mouse 事件。

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
    drawing: false,
    activePointerId: null,
    currentStart: null, // world 坐标 {x, y}
    segments: [], // [{a:{x,y}, b:{x,y}}]
  };

  function setV2ActiveFlag(on) {
    try { window.__nuwaxBoundaryActive = !!on; } catch (_) { /* ignore */ }
  }

  function worldFromEvent(ev) {
    if (typeof window.__nuwaxMapUnproject !== 'function') return null;
    return window.__nuwaxMapUnproject(ev.clientX, ev.clientY);
  }

  function makeSvgPoint(x, y) {
    const el = document.createElementNS(SVG_NS, 'circle');
    el.setAttribute('cx', String(x));
    el.setAttribute('cy', String(y));
    el.setAttribute('r', '3');
    el.setAttribute('fill', '#f59e0b');
    return el;
  }

  function makeSvgLine(a, b) {
    const el = document.createElementNS(SVG_NS, 'line');
    el.setAttribute('x1', String(a.x));
    el.setAttribute('y1', String(a.y));
    el.setAttribute('x2', String(b.x));
    el.setAttribute('y2', String(b.y));
    el.setAttribute('stroke', '#f59e0b');
    el.setAttribute('stroke-width', '2');
    return el;
  }

  function redrawOverlay() {
    if (!state.overlaySvg) return;
    while (state.overlaySvg.firstChild) state.overlaySvg.removeChild(state.overlaySvg.firstChild);
    state.segments.forEach(function (seg) {
      state.overlaySvg.appendChild(makeSvgLine(seg.a, seg.b));
      state.overlaySvg.appendChild(makeSvgPoint(seg.a.x, seg.a.y));
      state.overlaySvg.appendChild(makeSvgPoint(seg.b.x, seg.b.y));
    });
  }

  function renderPreview(endWorld) {
    if (!state.overlaySvg || !state.currentStart) return;
    redrawOverlay();
    state.overlaySvg.appendChild(makeSvgLine(state.currentStart, endWorld));
    state.overlaySvg.appendChild(makeSvgPoint(state.currentStart.x, state.currentStart.y));
    state.overlaySvg.appendChild(makeSvgPoint(endWorld.x, endWorld.y));
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
    if (!host) {
      // 兜底：退回到原容器（虽然位置不对，但不至于让模块彻底不工作）
      if (state.container) return null;
      return null;
    }
    state.overlaySvg = document.createElementNS(SVG_NS, 'svg');
    state.overlaySvg.id = 'boundary-draw-overlay';
    state.overlaySvg.setAttribute('width', '100%');
    state.overlaySvg.setAttribute('height', '100%');
    // 必须盖在 #mapCanvas 上方但保留工具栏/HUD 可点；用 z-index 低于 HUD (3) 不行，
    // 我们需要它高于 canvas 但不阻挡 toolbar（HUD 用 pointer-events:auto 单独覆盖）。
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
    ].join(';');
    state.overlaySvg.style.display = state.drawing ? 'block' : 'none';
    host.appendChild(state.overlaySvg);
    return state.overlaySvg;
  }

  function setDrawing(on) {
    state.drawing = !!on;
    if (state.drawing) {
      const overlay = ensureOverlay();
      if (!overlay) {
        // 找不到 host 时回退：保持按钮 active 让用户至少看到提示
        state.drawing = false;
        if (state.btnToggle) state.btnToggle.dataset.active = 'false';
        if (state.listEl) state.listEl.dataset.lastError = '找不到地图容器 .canvas-wrap';
        return;
      }
      overlay.style.display = 'block';
      setV2ActiveFlag(true);
    } else {
      if (state.overlaySvg) state.overlaySvg.style.display = 'none';
      state.currentStart = null;
      state.activePointerId = null;
      setV2ActiveFlag(false);
    }
    if (state.btnToggle) state.btnToggle.dataset.active = state.drawing ? 'true' : 'false';
    if (state.btnToggle) state.btnToggle.textContent = state.drawing ? '停止划线' : '开始划线';
  }

  function onPointerDown(ev) {
    if (!state.drawing) return;
    if (ev.button !== 0 && ev.pointerType === 'mouse') return; // 鼠标只接受左键
    ev.preventDefault();
    ev.stopPropagation();
    const w = worldFromEvent(ev);
    if (!w) return;
    state.currentStart = { x: w.x, y: w.y };
    state.activePointerId = ev.pointerId;
    if (state.overlaySvg && state.overlaySvg.setPointerCapture) {
      try { state.overlaySvg.setPointerCapture(ev.pointerId); } catch (_) {}
    }
  }

  function onPointerMove(ev) {
    if (!state.drawing || !state.currentStart) return;
    if (state.activePointerId !== null && ev.pointerId !== state.activePointerId) return;
    const w = worldFromEvent(ev);
    if (!w) return;
    renderPreview({ x: w.x, y: w.y });
  }

  function onPointerUp(ev) {
    if (!state.drawing || !state.currentStart) return;
    if (state.activePointerId !== null && ev.pointerId !== state.activePointerId) return;
    const w = worldFromEvent(ev);
    if (!w) return;
    state.segments.push({
      a: state.currentStart,
      b: { x: w.x, y: w.y },
    });
    state.currentStart = null;
    state.activePointerId = null;
    if (state.overlaySvg && state.overlaySvg.releasePointerCapture) {
      try { state.overlaySvg.releasePointerCapture(ev.pointerId); } catch (_) {}
    }
    redrawOverlay();
    renderList();
  }

  function onPointerCancel(ev) {
    if (!state.drawing) return;
    state.currentStart = null;
    state.activePointerId = null;
    if (state.overlaySvg) redrawOverlay();
  }

  function clearAll() {
    state.segments = [];
    state.currentStart = null;
    state.activePointerId = null;
    redrawOverlay();
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
      auto_loop_close: true,
      extend_radius_m: 3.0,
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