// boundary_draw.js — V2 navigation-and-exploration-boundary §9.2
// 手划多段线段前端模块：
//   - 划线模式按钮
//   - SVG overlay（多线段拖拽）
//   - mousedown/mousemove/mouseup 事件
//   - 多段线段列表 UI
//   - 提交 POST /api/exploration/area（即 explore_area_v2 capability）
// 本模块是 debug_panel.html 的拆模块；不堆业务到主 html。

(function () {
  'use strict';

  const EXPLORATION_API = '/api/capabilities/explore_area_v2/invoke';
  const FRAME_ID = 'map';
  const SVG_NS = 'http://www.w3.org/2000/svg';

  const state = {
    container: null,
    btnToggle: null,
    btnSubmit: null,
    btnClear: null,
    listEl: null,
    overlaySvg: null,
    drawing: false,
    currentStart: null, // world 坐标 {x, y}
    segments: [], // [{a:{x,y}, b:{x,y}}]
  };

  function worldFromEvent(ev) {
    // 主 html 暴露 window.__nuwaxMapUnproject(screenX, screenY) -> {x, y, frame_id}
    if (typeof window.__nuwaxMapUnproject !== 'function') return null;
    const rect = (state.overlaySvg && state.overlaySvg.getBoundingClientRect()) || null;
    const sx = ev.clientX;
    const sy = ev.clientY;
    return window.__nuwaxMapUnproject(sx, sy, rect);
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

  function ensureOverlay() {
    if (state.overlaySvg) return state.overlaySvg;
    // 主 html 已留 #boundary-draw-overlay 容器；动态创建 SVG。
    state.overlaySvg = document.createElementNS(SVG_NS, 'svg');
    state.overlaySvg.id = 'boundary-draw-overlay';
    state.overlaySvg.style.cssText = 'position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:auto;z-index:9000;cursor:crosshair';
    state.overlaySvg.setAttribute('width', '100%');
    state.overlaySvg.setAttribute('height', '100%');
    state.overlaySvg.style.display = state.drawing ? 'block' : 'none';
    if (state.container) state.container.appendChild(state.overlaySvg);
    return state.overlaySvg;
  }

  function setDrawing(on) {
    state.drawing = !!on;
    ensureOverlay();
    if (state.overlaySvg) state.overlaySvg.style.display = state.drawing ? 'block' : 'none';
    if (state.btnToggle) state.btnToggle.dataset.active = state.drawing ? 'true' : 'false';
  }

  function onMouseDown(ev) {
    if (!state.drawing) return;
    if (ev.button !== 0) return;
    const w = worldFromEvent(ev);
    if (!w) return;
    state.currentStart = { x: w.x, y: w.y };
    ev.preventDefault();
  }

  function onMouseMove(ev) {
    if (!state.drawing || !state.currentStart) return;
    // 实时预览
    const w = worldFromEvent(ev);
    if (!w) return;
    redrawOverlay();
    state.overlaySvg.appendChild(makeSvgLine(state.currentStart, { x: w.x, y: w.y }));
    state.overlaySvg.appendChild(makeSvgPoint(state.currentStart.x, state.currentStart.y));
    state.overlaySvg.appendChild(makeSvgPoint(w.x, w.y));
  }

  function onMouseUp(ev) {
    if (!state.drawing || !state.currentStart) return;
    const w = worldFromEvent(ev);
    if (!w) return;
    state.segments.push({
      a: state.currentStart,
      b: { x: w.x, y: w.y },
    });
    state.currentStart = null;
    redrawOverlay();
    renderList();
  }

  function clearAll() {
    state.segments = [];
    state.currentStart = null;
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

  function mount() {
    state.container = document.getElementById('boundary-draw-container');
    if (!state.container) return;
    state.btnToggle = state.container.querySelector('[data-role="boundary-toggle"]');
    state.btnSubmit = state.container.querySelector('[data-role="boundary-submit"]');
    state.btnClear = state.container.querySelector('[data-role="boundary-clear"]');
    state.listEl = state.container.querySelector('[data-role="boundary-list"]');
    if (state.btnToggle) state.btnToggle.addEventListener('click', function () { setDrawing(!state.drawing); });
    if (state.btnSubmit) state.btnSubmit.addEventListener('click', submitBoundary);
    if (state.btnClear) state.btnClear.addEventListener('click', clearAll);
    const overlay = ensureOverlay();
    overlay.addEventListener('mousedown', onMouseDown);
    overlay.addEventListener('mousemove', onMouseMove);
    overlay.addEventListener('mouseup', onMouseUp);
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
