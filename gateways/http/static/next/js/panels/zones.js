/* 禁行区面板：从 boundary_draw.js 迁出，几何、校验、绘制与保存规则不变。
   - 读写 /api/navigation/forbidden-zones（按地图名）；PUT 整体替换，409 时重新读取；随 navigation.forbidden_zones 更新；
   - 二维：覆盖层 SVG 画已保存的禁行区与草稿（外部禁行用反向填充）；三维：已保存的由三维场景画，覆盖层只画草稿；
   - 绘制时冻结二维相机；直线、矩形、圆形按住拖出，多边形逐点单击、Enter 闭合、Esc 取消，不能自相交、面积不能为零；
   - 三维草稿依附点云来源，点云来源变化时作废。
   新页面另加：每个禁行区有启用开关；删除不确认，轻提示里可撤销；清空全部先确认。 */
import { h, icon, toast, confirmDialog } from '/static/shared/neh-ui.js';

const API_URL = '/api/navigation/forbidden-zones';
const FRAME_ID = 'map';
const SVG_NS = 'http://www.w3.org/2000/svg';
const MIN_GEOMETRY_METERS = 0.05;
const CIRCLE_SEGMENTS = 64;
const SHAPE_LABELS = { line: '直线', rectangle: '矩形', circle: '圆形', polygon: '多边形' };
const MODE_LABELS = { inside: '内部禁行', outside: '外部禁行' };

const numberOrNull = value => (Number.isFinite(Number(value)) ? Number(value) : null);
const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));
function normalizedPoint(value) {
  if (!value || typeof value !== 'object') return null;
  const x = numberOrNull(value.x), y = numberOrNull(value.y);
  return x === null || y === null ? null : { x, y };
}
const normalizedVertices = values => (Array.isArray(values) ? values.map(normalizedPoint).filter(Boolean) : []);
const newZoneId = () => (window.crypto?.randomUUID ? window.crypto.randomUUID() : 'zone-' + Date.now().toString(36) + '-' + Math.random().toString(36).slice(2, 10));
function normalizeZone(raw) {
  if (!raw || typeof raw !== 'object') return null;
  let shape = String(raw.shape || '').trim().toLowerCase();
  if (shape === 'custom') shape = 'polygon';
  if (!Object.prototype.hasOwnProperty.call(SHAPE_LABELS, shape)) return null;
  let vertices = normalizedVertices(raw.vertices || raw.points);
  if (shape === 'rectangle' && vertices.length > 2) {
    const xs = vertices.map(p => p.x), ys = vertices.map(p => p.y);
    vertices = [{ x: Math.min(...xs), y: Math.min(...ys) }, { x: Math.max(...xs), y: Math.max(...ys) }];
  }
  const center = normalizedPoint(raw.center), radius = numberOrNull(raw.radius_m);
  const strokeWidth = numberOrNull(raw.stroke_width_m ?? raw.line_width_m);
  if (shape === 'line' && vertices.length !== 2) return null;
  if (shape === 'rectangle' && vertices.length !== 2) return null;
  if (shape === 'polygon' && vertices.length < 3) return null;
  if (shape === 'circle' && (!center || radius === null || radius <= 0)) return null;
  const metadata = raw.metadata && typeof raw.metadata === 'object' ? { ...raw.metadata } : {};
  if (!metadata.name && raw.name) metadata.name = String(raw.name);
  return { id: String(raw.id || raw.zone_id || newZoneId()), shape, mode: shape === 'line' ? 'inside' : raw.mode === 'outside' ? 'outside' : 'inside',
    frame_id: String(raw.frame_id || FRAME_ID), vertices: shape === 'circle' ? [] : vertices, center: shape === 'circle' ? center : null,
    radius_m: shape === 'circle' ? radius : null, stroke_width_m: strokeWidth !== null && strokeWidth > 0 ? strokeWidth : 0.25, enabled: raw.enabled !== false, metadata };
}
function canonicalZone(zone) {
  return { id: zone.id, shape: zone.shape, mode: zone.shape === 'line' ? 'inside' : zone.mode, frame_id: zone.frame_id || FRAME_ID,
    vertices: (zone.vertices || []).map(p => ({ x: p.x, y: p.y })), center: zone.shape === 'circle' && zone.center ? { x: zone.center.x, y: zone.center.y } : null,
    radius_m: zone.shape === 'circle' ? zone.radius_m : null, stroke_width_m: zone.stroke_width_m, enabled: zone.enabled !== false, metadata: { ...(zone.metadata || {}) } };
}
const entryFromEnvelope = raw => ({ mapName: raw.map_name, zones: raw.zones.map(normalizeZone).filter(Boolean) });
function polygonAreaTwice(vertices) {
  let sum = 0;
  for (let i = 0; i < vertices.length; i += 1) { const a = vertices[i], b = vertices[(i + 1) % vertices.length]; sum += a.x * b.y - b.x * a.y; }
  return Math.abs(sum);
}
const cross = (a, b, c) => (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
function pointOnSegment(point, start, end) {
  const e = 1e-10;
  return Math.abs(cross(start, end, point)) <= e && point.x >= Math.min(start.x, end.x) - e && point.x <= Math.max(start.x, end.x) + e
    && point.y >= Math.min(start.y, end.y) - e && point.y <= Math.max(start.y, end.y) + e;
}
function segmentsIntersect(a, b, c, d) {
  const abC = cross(a, b, c), abD = cross(a, b, d), cdA = cross(c, d, a), cdB = cross(c, d, b);
  if (abC * abD < 0 && cdA * cdB < 0) return true;
  return pointOnSegment(c, a, b) || pointOnSegment(d, a, b) || pointOnSegment(a, c, d) || pointOnSegment(b, c, d);
}
function polygonSelfIntersects(vertices) {
  for (let left = 0; left < vertices.length; left += 1) {
    const leftNext = (left + 1) % vertices.length;
    for (let right = left + 1; right < vertices.length; right += 1) {
      const rightNext = (right + 1) % vertices.length;
      if (left === right || leftNext === right || rightNext === left) continue;
      if (segmentsIntersect(vertices[left], vertices[leftNext], vertices[right], vertices[rightNext])) return true;
    }
  }
  return false;
}

export function mountZones(root, ctx) {
  const { stage, stream } = ctx;
  const state = { overlaySvg: null, overlayWidth: 0, overlayHeight: 0, overlayScale: 1, editing: false, draftAnchor: null, busy: false,
    activePointerId: null, dragStart: null, dragCurrent: null, customVertices: [], customHover: null, draft: null, zones: [], mapName: null,
    observedMapName: null, mapNameInitialized: false, requestSerial: 0, shape: 'rectangle', mode: 'inside', width: 0.25, creating: false, shown: false };

  /* ---------- 界面 ---------- */
  const newBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('plus', 'n-icon--sm'), onclick: () => startCreate() }, '新建禁行区');
  const draftBox = h('div', { hidden: true });
  const list = h('div', { class: 'n-list' });
  const status = h('p', { class: 'c-status-line', role: 'status' });
  const clearAll = h('button', { class: 'n-btn n-btn--quiet n-btn--sm c-danger-text', type: 'button', hidden: true, onclick: () => clearAllZones() }, '清空全部禁行区…');
  root.append(h('div', { class: 'c-pane-head' }, h('h2', {}, '禁行区'), h('div', { class: 'c-pane-actions' }, newBtn)), draftBox, list, status, clearAll);
  const nameInput = h('input', { class: 'n-input', maxlength: 80, autocomplete: 'off' });
  const widthInput = h('input', { class: 'n-input n-num', type: 'number', min: 0.02, max: 5, step: 0.01, value: 0.25 });
  const shapeSeg = h('div', { class: 'n-seg n-seg--block', role: 'group', 'aria-label': '形状' });
  const modeSeg = h('div', { class: 'n-seg n-seg--block', role: 'group', 'aria-label': '禁止通行的一侧' });
  const draftHint = h('div', { class: 'n-callout n-callout--accent', html: icon('info') }, h('div', { class: 'n-callout-body' }));
  const finishBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => finishPolygon() }, '闭合多边形');
  const undoBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => undoPolygonVertex() }, '撤销上一点');
  const saveBtn = h('button', { class: 'n-btn n-btn--sm n-btn--primary', type: 'button', onclick: () => void saveDraft() }, '保存禁行区');
  const cancelBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => endCreate() }, '放弃');
  draftBox.append(h('div', { class: 'c-card c-card--active c-gap-b' },
    h('div', { class: 'c-card-title c-gap-b' }, '新禁行区'),
    h('div', { class: 'n-field c-field-gap' }, h('span', { class: 'n-label' }, '形状'), shapeSeg),
    h('div', { class: 'n-field c-field-gap' }, h('span', { class: 'n-label' }, '禁止通行的一侧'), modeSeg),
    h('div', { class: 'c-form-2 c-field-gap' }, h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '名称'), nameInput),
      h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '边界宽度'), h('div', { class: 'n-input-group' }, widthInput, h('span', { class: 'n-suffix' }, '米')))),
    draftHint,
    h('div', { class: 'c-card-actions' }, saveBtn, finishBtn, undoBtn, cancelBtn)));
  function setStatus(message, level = '') { status.textContent = message; status.dataset.level = level; }
  function refreshStatus() {
    if (state.busy) return;
    setStatus(state.mapName ? '「' + state.mapName + '」上有 ' + state.zones.length + ' 个禁行区，导航规划会绕开启用的禁行区。' : '请先加载地图，再查看或编辑禁行区。', state.mapName ? '' : 'warn');
  }
  function renderSegs() {
    shapeSeg.replaceChildren(...Object.entries(SHAPE_LABELS).map(([id, label]) => h('button', { type: 'button', 'aria-pressed': String(state.shape === id),
      onclick: () => { state.shape = id; if (id === 'line') state.mode = 'inside'; clearDraft(); renderSegs(); } }, label)));
    modeSeg.replaceChildren(...Object.entries(MODE_LABELS).map(([id, label]) => h('button', { type: 'button', 'aria-pressed': String(state.mode === id),
      disabled: state.shape === 'line', title: state.shape === 'line' ? '直线只挡住经过它的路线' : '',
      onclick: () => { state.mode = id; syncDraftOptions(); renderSegs(); } }, label)));
    draftHint.lastChild.textContent = state.shape === 'polygon' ? '在地图上逐点单击，Enter 闭合，Esc 取消。' : '在地图上按住左键拖出' + SHAPE_LABELS[state.shape] + '。';
    syncDraftButtons();
  }
  function renderList() {
    newBtn.disabled = state.busy || !state.mapName;
    clearAll.hidden = !state.zones.length;
    clearAll.disabled = state.busy;
    if (!state.zones.length) {
      list.replaceChildren(h('div', { class: 'n-empty', html: icon('ban') }, h('div', { class: 'n-empty-title' }, state.mapName ? '这张地图还没有禁行区' : '还没有加载地图'),
        h('div', {}, state.mapName ? '画出机器人不能进入的区域，导航规划会绕开。' : '在“地图”页激活地图后再编辑禁行区。')));
      return;
    }
    list.replaceChildren(...state.zones.map(zone => {
      const enabled = h('input', { type: 'checkbox', checked: zone.enabled, disabled: state.busy });
      enabled.addEventListener('change', () => void toggleZone(zone.id, enabled.checked));
      return h('div', { class: 'n-row n-row--hover' }, h('span', { class: 'c-swatch c-zone-swatch', dataset: { mode: zone.mode } }),
        h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, zoneName(zone)), h('div', { class: 'n-row-sub' }, SHAPE_LABELS[zone.shape] + '，' + MODE_LABELS[zone.mode] + (zone.enabled ? '' : '，已停用'))),
        h('label', { class: 'n-switch', title: zone.enabled ? '停用这个禁行区' : '启用这个禁行区' }, enabled, h('span', { class: 'n-switch-track' })),
        h('button', { class: 'n-btn n-btn--quiet n-btn--icon n-btn--sm', type: 'button', title: '删除这个禁行区', 'aria-label': '删除' + zoneName(zone), disabled: state.busy,
          html: icon('trash-2', 'n-icon--sm'), onclick: () => void deleteZone(zone.id) }));
    }));
  }
  const zoneName = zone => String(zone?.metadata?.name || zone?.id || '未命名禁行区');

  /* ---------- 读写 ---------- */
  async function fetchJson(url, options) {
    const response = await fetch(url, options);
    let body = null;
    try { body = await response.json(); } catch (_) { /* 无响应体 */ }
    if (!response.ok) {
      const detail = body && (body.detail || body.error || body.message);
      const error = new Error(typeof detail === 'string' ? detail : detail?.message || 'HTTP ' + response.status);
      error.status = response.status;
      throw error;
    }
    return body || {};
  }
  function emitZonesUpdated() {
    window.dispatchEvent(new CustomEvent('forbidden-zones:updated', { detail: { map_name: state.mapName, zones: state.zones.map(canonicalZone) } }));
  }
  function applyEntry(entry) {
    state.mapName = entry.mapName || state.mapName;
    state.zones = entry.zones.slice();
    renderList();
    redrawOverlay();
    refreshStatus();
    emitZonesUpdated();
  }
  function setBusy(busy, message) {
    state.busy = Boolean(busy);
    if (state.busy && message) setStatus(message);
    renderList();
    syncDraftButtons();
  }
  async function loadZones(mapName) {
    const serial = ++state.requestSerial;
    const requested = String(mapName || '').trim() || null;
    if (!requested) {
      state.mapName = null; state.zones = [];
      endCreate(); setBusy(false); redrawOverlay(); emitZonesUpdated(); refreshStatus();
      return;
    }
    const preserving = requested === state.mapName;
    state.mapName = requested;
    if (!preserving) state.zones = [];
    clearDraft();
    renderList();
    redrawOverlay();
    if (!preserving) emitZonesUpdated();
    setBusy(true, '正在读取「' + requested + '」的禁行区');
    try {
      const raw = await fetchJson(API_URL + '?map_name=' + encodeURIComponent(requested), { headers: { Accept: 'application/json' } });
      if (serial !== state.requestSerial) return;
      const entry = entryFromEnvelope(raw);
      if (entry.mapName !== requested) throw new Error('服务端返回了错误的 map_name');
      state.busy = false;
      applyEntry(entry);
    } catch (error) {
      if (serial !== state.requestSerial) return;
      setStatus((error.message || '禁行区加载失败') + (preserving ? '；继续显示上次成功读取的禁行区' : '；当前地图禁行区状态未知'), 'error');
      state.busy = false;
      renderList();
      return;
    } finally {
      if (serial === state.requestSerial) { state.busy = false; renderList(); syncDraftButtons(); }
    }
  }
  async function replaceZones(nextZones, successMessage) {
    if (!state.mapName) { setStatus('当前没有可写入的地图', 'error'); return false; }
    const serial = ++state.requestSerial;
    const mapName = state.mapName;
    setBusy(true, '正在保存 ' + nextZones.length + ' 个禁行区');
    try {
      const raw = await fetchJson(API_URL, { method: 'PUT', headers: { 'Content-Type': 'application/json', Accept: 'application/json' },
        body: JSON.stringify({ map_name: mapName, zones: nextZones.map(canonicalZone) }) });
      if (serial !== state.requestSerial || state.mapName !== mapName) return false;
      const entry = entryFromEnvelope(raw);
      if (entry.mapName !== mapName) throw new Error('服务端保存响应缺少当前地图的禁行区条目');
      state.busy = false;
      applyEntry(entry);
      if (successMessage) setStatus(successMessage);
      return true;
    } catch (error) {
      if (serial === state.requestSerial) {
        state.busy = false;
        if (error.status === 409 && state.mapName === mapName) { setStatus('禁行区已被其他编辑器更新，正在重新读取', 'error'); await loadZones(mapName); }
        else { setStatus(error.message || '禁行区保存失败', 'error'); renderList(); }
      }
      return false;
    } finally {
      if (serial === state.requestSerial) { state.busy = false; renderList(); syncDraftButtons(); }
    }
  }
  function syncMapName(force) {
    const resolved = stage.map.activeMapName() || null;
    if (!force && state.mapNameInitialized && resolved === state.observedMapName) return;
    state.mapNameInitialized = true;
    state.observedMapName = resolved;
    if (!force && resolved === state.mapName) return;
    void loadZones(resolved);
  }
  async function toggleZone(zoneId, enabled) {
    await replaceZones(state.zones.map(zone => (zone.id === zoneId ? { ...zone, enabled } : zone)), enabled ? '已启用，下次规划路线时生效' : '已停用，下次规划路线时生效');
  }
  async function deleteZone(zoneId) {
    const index = state.zones.findIndex(zone => zone.id === zoneId);
    if (index < 0) return;
    const removed = state.zones[index];
    if (!await replaceZones(state.zones.filter(zone => zone.id !== zoneId), '禁行区已删除')) return;
    toast('已删除「' + zoneName(removed) + '」', { action: { label: '撤销', run: () => {
      const next = state.zones.slice();
      next.splice(Math.min(index, next.length), 0, removed);
      void replaceZones(next, '已恢复「' + zoneName(removed) + '」');
    } } });
  }
  async function clearAllZones() {
    if (!state.zones.length) return;
    if (!await confirmDialog({ title: '清空全部禁行区', text: '会删除「' + state.mapName + '」上的 ' + state.zones.length + ' 个禁行区，下次规划路线时不再绕开。', confirmText: '全部删除', danger: true })) return;
    if (await replaceZones([], '已清空当前地图的禁行区')) clearDraft();
  }

  /* ---------- 绘制 ---------- */
  const is2dMapVisible = () => stage.map.viewMode !== '3d';
  function worldFromEvent(event) {
    if (!is2dMapVisible()) {
      if (!state.draftAnchor && event.type !== 'pointerdown') return null;
      const point = window.pointcloud3d?.boundaryPointAt(event.clientX, event.clientY, state.draftAnchor);
      if (!point) { if (event.type === 'pointerdown') setStatus('请在可见点云地面上开始绘制，空白处无法确定位置。', 'warn'); return null; }
      if (!state.draftAnchor) state.draftAnchor = point;
      return normalizedPoint(point);
    }
    return normalizedPoint(window.__nuwaxMapUnproject?.(event.clientX, event.clientY));
  }
  function projectPoint(point) {
    if (!point) return null;
    const projected = is2dMapVisible() ? window.__nuwaxMapProject?.(point.x, point.y)
      : window.pointcloud3d?.projectBoundaryPoint(point.x, point.y, state.draftAnchor?.surface_z ?? 0);
    if (!projected || !Number.isFinite(Number(projected.x)) || !Number.isFinite(Number(projected.y))) return null;
    if (Number(projected.scale) > 0) state.overlayScale = Number(projected.scale);
    if (Number.isFinite(Number(projected.pixelWidth))) state.overlayWidth = Number(projected.pixelWidth);
    if (Number.isFinite(Number(projected.pixelHeight))) state.overlayHeight = Number(projected.pixelHeight);
    return { x: Number(projected.x), y: Number(projected.y) };
  }
  function displayVertices(zone) {
    if (zone.shape === 'rectangle') {
      const [a, b] = zone.vertices;
      return a && b ? [{ x: a.x, y: a.y }, { x: b.x, y: a.y }, { x: b.x, y: b.y }, { x: a.x, y: b.y }] : [];
    }
    if (zone.shape === 'circle') {
      const points = [];
      for (let i = 0; i < CIRCLE_SEGMENTS; i += 1) { const angle = Math.PI * 2 * i / CIRCLE_SEGMENTS; points.push({ x: zone.center.x + Math.cos(angle) * zone.radius_m, y: zone.center.y + Math.sin(angle) * zone.radius_m }); }
      return points;
    }
    return (zone.vertices || []).slice();
  }
  function svg(tag, attributes) {
    const element = document.createElementNS(SVG_NS, tag);
    for (const [name, value] of Object.entries(attributes)) if (value !== null && value !== undefined) element.setAttribute(name, String(value));
    return element;
  }
  function appendZone(zone, opts = {}) {
    const closed = zone.shape !== 'line';
    const projected = displayVertices(zone).map(projectPoint).filter(Boolean);
    if (projected.length < (closed ? 3 : 2)) return;
    const path = projected.map((p, i) => (i ? 'L' : 'M') + p.x + ' ' + p.y).join(' ') + (closed ? ' Z' : '');
    const enabled = zone.enabled !== false, outside = zone.mode === 'outside' && closed;
    const stroke = outside ? '#a855f7' : '#ef4444', fill = outside ? '#7e22ce' : '#dc2626';
    if (enabled && closed) {
      if (outside && state.overlayWidth > 0 && state.overlayHeight > 0) {
        state.overlaySvg.append(svg('path', { d: 'M0 0 H' + state.overlayWidth + ' V' + state.overlayHeight + ' H0 Z ' + path, fill, 'fill-opacity': opts.draft ? 0.18 : 0.14, 'fill-rule': 'evenodd', 'pointer-events': 'none' }));
      } else state.overlaySvg.append(svg('path', { d: path, fill, 'fill-opacity': opts.draft ? 0.36 : 0.22, 'pointer-events': 'none' }));
    }
    state.overlaySvg.append(svg('path', { d: path, fill: 'none', stroke: enabled ? stroke : '#94a3b8', 'stroke-width': clamp(Math.max(2, Number(zone.stroke_width_m || 0.25) * state.overlayScale), 2, 34),
      'stroke-linecap': 'round', 'stroke-linejoin': 'round', 'stroke-dasharray': enabled ? (opts.draft ? '10 7' : null) : '7 7',
      'stroke-opacity': opts.draft ? 0.95 : enabled ? 0.92 : 0.65, 'pointer-events': 'none' }));
    if (opts.vertices) for (const p of projected) state.overlaySvg.append(svg('circle', { cx: p.x, cy: p.y, r: 4.5, fill: '#fef2f2', stroke, 'stroke-width': 2, 'pointer-events': 'none' }));
  }
  function geometryFromDrag(shape, start, end) {
    const distance = Math.hypot(end.x - start.x, end.y - start.y);
    if (distance < MIN_GEOMETRY_METERS) return null;
    const common = { shape, mode: shape === 'line' ? 'inside' : state.mode, frame_id: FRAME_ID, vertices: [], center: null, radius_m: null, stroke_width_m: selectedStrokeWidth(), enabled: true };
    if (shape === 'line' || shape === 'rectangle') {
      common.vertices = [{ ...start }, { ...end }];
      if (shape === 'rectangle' && (Math.abs(end.x - start.x) < MIN_GEOMETRY_METERS || Math.abs(end.y - start.y) < MIN_GEOMETRY_METERS)) return null;
      return common;
    }
    if (shape === 'circle') { common.center = { ...start }; common.radius_m = distance; return common; }
    return null;
  }
  const selectedStrokeWidth = () => { const value = numberOrNull(widthInput.value); return value === null ? 0.25 : clamp(value, 0.02, 5); };
  function draftPreview() {
    if (state.draft) return state.draft;
    if (state.shape === 'polygon') {
      const vertices = state.customVertices.slice();
      if (state.customHover && vertices.length) vertices.push(state.customHover);
      if (vertices.length < 2) return null;
      return { shape: vertices.length >= 3 ? 'polygon' : 'line', mode: state.mode, frame_id: FRAME_ID, vertices, center: null, radius_m: null, stroke_width_m: selectedStrokeWidth(), enabled: true };
    }
    return state.dragStart && state.dragCurrent ? geometryFromDrag(state.shape, state.dragStart, state.dragCurrent) : null;
  }
  function syncOverlayMetrics() {
    if (!is2dMapVisible()) {
      const rect = stage.mapLayer.getBoundingClientRect();
      state.overlayWidth = rect.width || 1; state.overlayHeight = rect.height || 1; state.overlayScale = 1;
    } else {
      const projected = window.__nuwaxMapProject?.(0, 0);
      state.overlayWidth = Number(projected?.pixelWidth) || stage.canvas.width || 1;
      state.overlayHeight = Number(projected?.pixelHeight) || stage.canvas.height || 1;
      state.overlayScale = Number(projected?.scale) || state.overlayScale || 1;
    }
    state.overlaySvg.setAttribute('viewBox', '0 0 ' + state.overlayWidth + ' ' + state.overlayHeight);
  }
  function redrawOverlay() {
    const overlay = ensureOverlay();
    syncOverlayMetrics();
    overlay.replaceChildren();
    // 已保存的禁行区在三维里由三维场景绘制，覆盖层只承担三维草稿与指针输入；关系图视图不画。
    overlay.style.display = stage.map.viewMode === 'graph' ? 'none' : 'block';
    if (is2dMapVisible()) {
      for (const zone of state.zones.filter(z => z.mode === 'outside')) appendZone(zone);
      for (const zone of state.zones.filter(z => z.mode !== 'outside')) appendZone(zone);
    }
    const preview = draftPreview();
    if (preview) appendZone(preview, { draft: true, vertices: true });
  }
  function ensureOverlay() {
    if (state.overlaySvg) return state.overlaySvg;
    const overlay = svg('svg', { class: 'c-zones-overlay', 'aria-label': '导航禁行区覆盖层' });
    stage.mapLayer.append(overlay);
    overlay.addEventListener('pointerdown', onPointerDown);
    overlay.addEventListener('pointermove', onPointerMove);
    overlay.addEventListener('pointerup', onPointerUp);
    overlay.addEventListener('pointercancel', onPointerCancel);
    new ResizeObserver(redrawOverlay).observe(stage.mapLayer);
    state.overlaySvg = overlay;
    return overlay;
  }
  function finishPolygon() {
    const vertices = state.customVertices.slice();
    if (vertices.length < 3) { setStatus('多边形至少需要 3 个顶点', 'error'); return false; }
    if (polygonAreaTwice(vertices) <= 1e-8) { setStatus('多边形面积不能为零', 'error'); return false; }
    if (polygonSelfIntersects(vertices)) { setStatus('多边形不能自相交', 'error'); return false; }
    state.draft = { shape: 'polygon', mode: state.mode, frame_id: FRAME_ID, vertices, center: null, radius_m: null, stroke_width_m: selectedStrokeWidth(), enabled: true };
    state.customHover = null;
    redrawOverlay();
    syncDraftButtons();
    setStatus('多边形已闭合，填写名称后保存');
    return true;
  }
  function clearDraft() {
    state.draftAnchor = null; state.activePointerId = null; state.dragStart = null; state.dragCurrent = null;
    state.customVertices = []; state.customHover = null; state.draft = null;
    syncDraftButtons();
    if (state.overlaySvg) redrawOverlay();
  }
  function syncDraftButtons() {
    const polygon = state.shape === 'polygon';
    finishBtn.hidden = undoBtn.hidden = !polygon;
    finishBtn.disabled = state.busy || state.customVertices.length < 3;
    undoBtn.disabled = state.busy || state.customVertices.length < 1;
    saveBtn.disabled = state.busy || !(state.draft || (polygon && state.customVertices.length >= 3));
    cancelBtn.disabled = state.busy;
  }
  function syncDraftOptions() {
    if (state.draft) { state.draft.mode = state.draft.shape === 'line' ? 'inside' : state.mode; state.draft.stroke_width_m = selectedStrokeWidth(); }
    redrawOverlay();
  }
  widthInput.addEventListener('input', syncDraftOptions);
  function setEditing(active) {
    const next = Boolean(active);
    const overlay = ensureOverlay();
    state.editing = next;
    overlay.style.pointerEvents = next ? 'auto' : 'none';
    overlay.style.cursor = next ? 'crosshair' : 'default';
    window.__nuwaxBoundaryActive = next;
    window.pointcloud3d?.setBoundaryEditing(next && !is2dMapVisible());
    window.dispatchEvent(new CustomEvent('map:editing-changed'));
    if (next && is2dMapVisible()) {
      const point = window.__nuwaxCameraAnchor?.()?.point || { x: 0, y: 0 };
      window.__nuwaxMapFreezeCenter = { x: Number(point.x) || 0, y: Number(point.y) || 0 };
      window.__nuwaxMapFreeze = true;
    } else {
      state.activePointerId = null; state.dragStart = null; state.dragCurrent = null;
      window.__nuwaxMapFreeze = false;
      delete window.__nuwaxMapFreezeCenter;
    }
    stage.map.requestDraw();
    redrawOverlay();
    stage.hint(next ? h('div', { class: 'n-float c-hint', html: icon('pencil', 'n-icon--sm') },
      state.shape === 'polygon' ? '逐点单击画多边形，Enter 闭合，Esc 取消' : '按住左键拖出' + SHAPE_LABELS[state.shape],
      h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => endCreate() }, '放弃')) : null);
  }
  function onPointerDown(event) {
    if (!state.editing || (event.pointerType === 'mouse' && event.button !== 0)) return;
    event.preventDefault();
    event.stopPropagation();
    if (state.shape !== 'polygon' || !state.customVertices.length) state.draftAnchor = null;
    const point = worldFromEvent(event);
    if (!point) return;
    if (state.shape === 'polygon') {
      const previous = state.customVertices[state.customVertices.length - 1];
      if (!previous || Math.hypot(point.x - previous.x, point.y - previous.y) >= MIN_GEOMETRY_METERS) { state.customVertices.push(point); state.draft = null; }
      state.customHover = point;
      syncDraftButtons();
      redrawOverlay();
      return;
    }
    state.activePointerId = event.pointerId;
    state.dragStart = point;
    state.dragCurrent = point;
    state.draft = null;
    try { state.overlaySvg.setPointerCapture(event.pointerId); } catch (_) { /* 指针已释放 */ }
    redrawOverlay();
  }
  function onPointerMove(event) {
    if (!state.editing) return;
    const point = worldFromEvent(event);
    if (!point) return;
    if (state.shape === 'polygon') { if (state.customVertices.length) { state.customHover = point; redrawOverlay(); } return; }
    if (!state.dragStart || (state.activePointerId !== null && event.pointerId !== state.activePointerId)) return;
    state.dragCurrent = point;
    redrawOverlay();
  }
  function onPointerUp(event) {
    if (!state.editing || state.shape === 'polygon' || !state.dragStart) return;
    if (state.activePointerId !== null && event.pointerId !== state.activePointerId) return;
    event.preventDefault();
    const point = worldFromEvent(event) || state.dragCurrent;
    state.draft = point ? geometryFromDrag(state.shape, state.dragStart, point) : null;
    state.dragStart = null; state.dragCurrent = null; state.activePointerId = null;
    try { state.overlaySvg.releasePointerCapture(event.pointerId); } catch (_) { /* 指针已释放 */ }
    setStatus(state.draft ? '已画好，填写名称后保存' : '尺寸过小，请重新绘制', state.draft ? '' : 'error');
    syncDraftButtons();
    redrawOverlay();
  }
  function onPointerCancel(event) {
    if (!state.editing) return;
    state.activePointerId = null; state.dragStart = null; state.dragCurrent = null;
    try { state.overlaySvg.releasePointerCapture(event.pointerId); } catch (_) { /* 指针已释放 */ }
    redrawOverlay();
  }
  function undoPolygonVertex() {
    state.draft = null;
    state.customVertices.pop();
    state.customHover = null;
    syncDraftButtons();
    redrawOverlay();
  }
  async function saveDraft() {
    if (state.draftAnchor && !window.pointcloud3d?.navigationPointIsCurrent(state.draftAnchor)) {
      endCreate();
      setStatus('点云来源已变化，请重新绘制禁行区。', 'error');
      return;
    }
    if (state.shape === 'polygon' && !state.draft && !finishPolygon()) return;
    if (!state.draft) { setStatus('请先在地图上画出禁行区', 'error'); return; }
    const name = nameInput.value.trim();
    if (!name) { setStatus('禁行区名称不能为空', 'error'); nameInput.focus(); return; }
    const zone = normalizeZone({ ...state.draft, id: newZoneId(), mode: state.draft.shape === 'line' ? 'inside' : state.mode, stroke_width_m: selectedStrokeWidth(),
      enabled: true, metadata: { name, created_by: 'console' } });
    if (!await replaceZones(state.zones.concat(zone), '禁行区已保存，下次规划路线时生效')) return;
    endCreate();
  }
  function startCreate() {
    state.creating = true;
    draftBox.hidden = false;
    newBtn.hidden = true;
    nameInput.value = '禁行区 ' + (state.zones.length + 1);
    renderSegs();
    clearDraft();
    setEditing(true);
  }
  function endCreate() {
    state.creating = false;
    draftBox.hidden = true;
    newBtn.hidden = false;
    clearDraft();
    if (state.editing) setEditing(false);
  }
  document.addEventListener('keydown', event => {
    if (!state.editing) return;
    if (event.key === 'Escape') { clearDraft(); event.preventDefault(); }
    else if (event.key === 'Enter' && state.shape === 'polygon' && event.target === document.body) { finishPolygon(); event.preventDefault(); }
  });
  window.addEventListener('pointcloud3d:camera-state', () => { if (!is2dMapVisible() && state.overlaySvg) redrawOverlay(); });
  window.addEventListener('pointcloud3d:map-frame', () => {
    if (state.draftAnchor && !window.pointcloud3d?.navigationPointIsCurrent(state.draftAnchor)) {
      clearDraft();
      setStatus('点云来源已变化，请重新绘制禁行区。', 'warn');
    }
  });
  window.addEventListener('map:view-changed', () => { if (state.overlaySvg) redrawOverlay(); });
  // 二维地图每画一帧同步覆盖层视口（地图核心在绘制末尾调用这个钩子）。
  const previousTick = window.__nuwaxMapDrawTick;
  window.__nuwaxMapDrawTick = () => { if (typeof previousTick === 'function') previousTick(); if (state.overlaySvg) redrawOverlay(); };
  window.__nuwaxBoundaryDrawModule = { clearDraft, setDrawing: value => { if (!value) endCreate(); }, getZones: () => state.zones.map(canonicalZone), reload: () => syncMapName(true) };
  stream.on('navigation.forbidden_zones', frame => {
    const preferred = state.mapName || state.observedMapName;
    const entry = entryFromEnvelope(frame.payload);
    if (preferred && entry.mapName && entry.mapName !== preferred) return;
    applyEntry(entry);
  });
  for (const topic of ['map.frame', 'map.heartbeat', 'pointcloud_map.meta', 'geometry.state']) stream.on(topic, () => syncMapName(false));
  ensureOverlay();
  renderSegs();
  renderList();
  syncMapName(true);

  return {
    show() { state.shown = true; },
    // 离开禁行区页结束绘制，草稿不保留（与原控制台收起禁行区工具相同）。
    hide() { state.shown = false; if (state.creating) endCreate(); stage.hint(null); },
  };
}
