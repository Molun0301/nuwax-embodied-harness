// Navigation forbidden-zone editor and persistent 2D overlay.
// The backend owns the canonical document; this module only replaces the zone
// list for the active map through /api/navigation/forbidden-zones.

(function () {
  'use strict';

  const API_URL = '/api/navigation/forbidden-zones';
  const FRAME_ID = 'map';
  const SVG_NS = 'http://www.w3.org/2000/svg';
  const OVERLAY_HOST_SELECTOR = '.canvas-wrap';
  const MIN_GEOMETRY_METERS = 0.05;
  const CIRCLE_SEGMENTS = 64;
  const SHAPE_LABELS = Object.freeze({
    line: '直线',
    rectangle: '矩形',
    circle: '圆形',
    polygon: '自定义多边形',
  });
  const MODE_LABELS = Object.freeze({ inside: '内部禁行', outside: '外部禁行' });

  const state = {
    container: null,
    statusEl: null,
    hintEl: null,
    listEl: null,
    nameInput: null,
    shapeSelect: null,
    modeSelect: null,
    widthInput: null,
    enabledSelect: null,
    btnToggle: null,
    btnFinish: null,
    btnUndo: null,
    btnSubmit: null,
    btnDraftClear: null,
    btnClear: null,
    overlaySvg: null,
    overlayHost: null,
    overlayWidth: 0,
    overlayHeight: 0,
    overlayScale: 1,
    editing: false,
    busy: false,
    activePointerId: null,
    dragStart: null,
    dragCurrent: null,
    customVertices: [],
    customHover: null,
    draft: null,
    zones: [],
    mapName: null,
    observedMapName: null,
    mapNameInitialized: false,
    generation: 0,
    signature: null,
    updatedAt: null,
    requestSerial: 0,
    previousDrawTick: null,
    resizeObserver: null,
  };

  function numberOrNull(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
  }

  function clamp(value, minimum, maximum) {
    return Math.min(maximum, Math.max(minimum, value));
  }

  function normalizedPoint(value) {
    if (!value || typeof value !== 'object') return null;
    const x = numberOrNull(value.x);
    const y = numberOrNull(value.y);
    return x === null || y === null ? null : { x, y };
  }

  function normalizedVertices(values) {
    if (!Array.isArray(values)) return [];
    return values.map(normalizedPoint).filter(Boolean);
  }

  function newZoneId() {
    if (window.crypto && typeof window.crypto.randomUUID === 'function') {
      return window.crypto.randomUUID();
    }
    return 'zone-' + Date.now().toString(36) + '-' + Math.random().toString(36).slice(2, 10);
  }

  function metadataForZone(raw) {
    const metadata = raw && raw.metadata && typeof raw.metadata === 'object'
      ? { ...raw.metadata }
      : {};
    if (!metadata.name && raw && raw.name) metadata.name = String(raw.name);
    return metadata;
  }

  function normalizeZone(raw) {
    if (!raw || typeof raw !== 'object') return null;
    let shape = String(raw.shape || '').trim().toLowerCase();
    if (shape === 'custom') shape = 'polygon';
    if (!Object.prototype.hasOwnProperty.call(SHAPE_LABELS, shape)) return null;

    let vertices = normalizedVertices(raw.vertices || raw.points);
    if (shape === 'rectangle' && vertices.length > 2) {
      const xs = vertices.map((point) => point.x);
      const ys = vertices.map((point) => point.y);
      vertices = [
        { x: Math.min(...xs), y: Math.min(...ys) },
        { x: Math.max(...xs), y: Math.max(...ys) },
      ];
    }
    const center = normalizedPoint(raw.center);
    const radius = numberOrNull(raw.radius_m);
    const strokeWidth = numberOrNull(raw.stroke_width_m ?? raw.line_width_m);
    const mode = shape === 'line' ? 'inside' : raw.mode === 'outside' ? 'outside' : 'inside';

    if (shape === 'line' && vertices.length !== 2) return null;
    if (shape === 'rectangle' && vertices.length !== 2) return null;
    if (shape === 'polygon' && vertices.length < 3) return null;
    if (shape === 'circle' && (!center || radius === null || radius <= 0)) return null;

    return {
      id: String(raw.id || raw.zone_id || newZoneId()),
      shape,
      mode,
      frame_id: String(raw.frame_id || FRAME_ID),
      vertices: shape === 'circle' ? [] : vertices,
      center: shape === 'circle' ? center : null,
      radius_m: shape === 'circle' ? radius : null,
      stroke_width_m: strokeWidth !== null && strokeWidth > 0 ? strokeWidth : 0.25,
      enabled: raw.enabled !== false,
      metadata: metadataForZone(raw),
    };
  }

  function canonicalZone(zone) {
    return {
      id: zone.id,
      shape: zone.shape,
      mode: zone.shape === 'line' ? 'inside' : zone.mode,
      frame_id: zone.frame_id || FRAME_ID,
      vertices: (zone.vertices || []).map((point) => ({ x: point.x, y: point.y })),
      center: zone.shape === 'circle' && zone.center
        ? { x: zone.center.x, y: zone.center.y }
        : null,
      radius_m: zone.shape === 'circle' ? zone.radius_m : null,
      stroke_width_m: zone.stroke_width_m,
      enabled: zone.enabled !== false,
      metadata: { ...(zone.metadata || {}) },
    };
  }

  function unwrapEnvelope(raw) {
    let current = raw;
    for (let index = 0; index < 3; index += 1) {
      if (!current || typeof current !== 'object') break;
      if (current.maps || current.zones || current.map_name) break;
      const next = current.data || current.result || current.payload;
      if (!next || next === current) break;
      current = next;
    }
    return current && typeof current === 'object' ? current : {};
  }

  function entryFromEnvelope(raw, preferredMapName) {
    const root = unwrapEnvelope(raw);
    let entry = root;
    let mapName = String(root.map_name || '').trim() || null;
    if (root.maps && typeof root.maps === 'object') {
      const keys = Object.keys(root.maps);
      const preferred = String(preferredMapName || '').trim();
      const selectedKey = preferred
        ? Object.prototype.hasOwnProperty.call(root.maps, preferred)
          ? preferred
          : keys.find((key) => String(root.maps[key] && root.maps[key].map_name || '').trim() === preferred) || null
        : keys.length === 1 ? keys[0] : null;
      if (!selectedKey) return null;
      entry = root.maps[selectedKey] || {};
      mapName = String(entry.map_name || selectedKey).trim() || null;
    }
    if (!entry || !Array.isArray(entry.zones)) return null;
    return {
      mapName,
      generation: Math.max(0, Number(entry.generation) || 0),
      signature: String(entry.signature || '').trim() || null,
      updatedAt: entry.updated_at || null,
      zones: entry.zones.map(normalizeZone).filter(Boolean),
    };
  }

  function currentEnvelope() {
    const maps = {};
    if (state.mapName) {
      maps[state.mapName] = {
        map_name: state.mapName,
        generation: state.generation,
        signature: state.signature || '',
        updated_at: state.updatedAt,
        zones: state.zones.map(canonicalZone),
      };
    }
    return { schema_version: 1, maps };
  }

  function firstNonBlank() {
    for (const value of arguments) {
      const text = String(value === null || value === undefined ? '' : value).trim();
      if (text) return text;
    }
    return null;
  }

  function mapNameFromPayload(raw) {
    const queue = [{ value: raw, depth: 0 }];
    const visited = new Set();
    while (queue.length) {
      const item = queue.shift();
      const value = item.value;
      if (!value || typeof value !== 'object' || visited.has(value)) continue;
      visited.add(value);
      const resolved = firstNonBlank(
        value.map_name,
        value.active_map_name,
        value.seed_map_name,
        value.active_workspace && (value.active_workspace.active_map_name || value.active_workspace.map_name),
        value.localization_session && value.localization_session.map_name,
      );
      if (resolved) return resolved;
      if (item.depth >= 4) continue;
      for (const key of [
        'payload', 'data', 'result', 'metadata', 'diagnostics', 'map_snapshot',
        'runtime_governance', 'current_request', 'exploration_request', 'current_goal',
      ]) {
        if (value[key] && typeof value[key] === 'object') {
          queue.push({ value: value[key], depth: item.depth + 1 });
        }
      }
    }
    return null;
  }

  function currentMapName(raw) {
    const observed = mapNameFromPayload(raw);
    if (observed) return observed;
    if (typeof window.__nuwaxActiveMapName !== 'function') return null;
    return firstNonBlank(window.__nuwaxActiveMapName());
  }

  function setStatus(message, stateName) {
    if (!state.statusEl) return;
    state.statusEl.textContent = message;
    state.statusEl.dataset.state = stateName || '';
    if (state.listEl) {
      if (stateName === 'error') state.listEl.dataset.lastError = message;
      else delete state.listEl.dataset.lastError;
    }
  }

  function refreshStatus() {
    if (state.busy) return;
    const mapName = state.mapName || '未绑定地图';
    setStatus(mapName + ' · ' + state.zones.length + ' 个 · G' + state.generation, state.mapName ? 'ready' : 'warning');
  }

  function setBusy(busy, message) {
    state.busy = Boolean(busy);
    for (const button of [state.btnToggle, state.btnFinish, state.btnUndo, state.btnSubmit, state.btnDraftClear, state.btnClear]) {
      if (button) button.disabled = state.busy;
    }
    if (state.busy && message) setStatus(message, 'busy');
    renderList();
    syncDraftButtons();
  }

  function emitZonesUpdated() {
    window.dispatchEvent(new CustomEvent('forbidden-zones:updated', {
      detail: currentEnvelope(),
    }));
  }

  function applyEntry(entry) {
    if (!entry) return false;
    state.mapName = entry.mapName || state.mapName;
    state.generation = entry.generation;
    state.signature = entry.signature;
    state.updatedAt = entry.updatedAt;
    state.zones = entry.zones.slice();
    renderList();
    redrawOverlay();
    refreshStatus();
    emitZonesUpdated();
    return true;
  }

  function errorMessage(error, fallback) {
    if (error && error.message) return error.message;
    return fallback;
  }

  async function fetchJson(url, options) {
    const authenticatedUrl = typeof window.__nuwaxWithAccessToken === 'function'
      ? window.__nuwaxWithAccessToken(url)
      : url;
    const response = await fetch(authenticatedUrl, options);
    let body = null;
    try { body = await response.json(); } catch (_) {}
    if (!response.ok) {
      const detail = body && (body.detail || body.error || body.message);
      const message = typeof detail === 'string'
        ? detail
        : detail && detail.message ? detail.message : 'HTTP ' + response.status;
      const error = new Error(message);
      error.status = response.status;
      error.body = body;
      throw error;
    }
    return body || {};
  }

  async function loadZones(mapName) {
    const serial = ++state.requestSerial;
    const requestedMapName = String(mapName || '').trim() || null;
    const preservingCurrentSnapshot = requestedMapName === null || requestedMapName === state.mapName;
    if (requestedMapName) state.mapName = requestedMapName;
    if (!preservingCurrentSnapshot) {
      state.generation = 0;
      state.signature = null;
      state.updatedAt = null;
      state.zones = [];
    }
    clearDraft();
    renderList();
    redrawOverlay();
    if (!preservingCurrentSnapshot) emitZonesUpdated();
    setBusy(true, requestedMapName ? '加载 ' + requestedMapName : '解析当前地图');
    const url = requestedMapName
      ? API_URL + '?map_name=' + encodeURIComponent(requestedMapName)
      : API_URL;
    try {
      const raw = await fetchJson(url, { headers: { Accept: 'application/json' } });
      if (serial !== state.requestSerial) return;
      const entry = entryFromEnvelope(raw, requestedMapName);
      if (!entry) throw new Error('服务端未返回禁行区地图条目');
      if (requestedMapName && entry.mapName !== requestedMapName) {
        throw new Error('服务端返回了错误的 map_name');
      }
      applyEntry(entry);
      setStatus(entry.mapName + ' · ' + entry.zones.length + ' 个 · G' + entry.generation, 'ready');
    } catch (error) {
      if (serial !== state.requestSerial) return;
      const suffix = preservingCurrentSnapshot
        ? '；继续显示上次成功快照'
        : '；当前地图禁行区状态未知';
      setStatus(errorMessage(error, '禁行区加载失败') + suffix, 'error');
    } finally {
      if (serial === state.requestSerial) setBusy(false);
    }
  }

  async function replaceZones(nextZones, successMessage) {
    if (!state.mapName) {
      setStatus('当前没有可写入的地图', 'error');
      return false;
    }
    const serial = ++state.requestSerial;
    const mapName = state.mapName;
    setBusy(true, '保存 ' + nextZones.length + ' 个禁行区');
    try {
      const raw = await fetchJson(API_URL, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json', Accept: 'application/json' },
        body: JSON.stringify({
          map_name: mapName,
          expected_generation: state.generation,
          zones: nextZones.map(canonicalZone),
        }),
      });
      if (serial !== state.requestSerial || state.mapName !== mapName) return false;
      const entry = entryFromEnvelope(raw, mapName);
      if (!entry || entry.mapName !== mapName) {
        throw new Error('服务端保存响应缺少当前地图的禁行区条目');
      }
      if (entry.generation < state.generation) {
        setStatus('保存完成；已保留更新的服务端禁行区快照', 'ready');
        return true;
      }
      applyEntry(entry);
      setStatus(successMessage || '已保存', 'ready');
      return true;
    } catch (error) {
      if (serial === state.requestSerial) {
        if (error && error.status === 409 && state.mapName === mapName) {
          setStatus('禁行区已被其他编辑器更新，正在重新加载', 'error');
          await loadZones(mapName);
        } else {
          setStatus(errorMessage(error, '禁行区保存失败'), 'error');
        }
      }
      return false;
    } finally {
      if (serial === state.requestSerial) setBusy(false);
    }
  }

  function syncMapName(force, raw) {
    const resolved = currentMapName(raw);
    if (!force && (!resolved || (state.mapNameInitialized && resolved === state.observedMapName))) return;
    state.mapNameInitialized = true;
    state.observedMapName = resolved;
    if (!force && resolved === state.mapName) return;
    void loadZones(resolved);
  }

  function worldFromEvent(event) {
    if (typeof window.__nuwaxMapUnproject !== 'function') return null;
    return normalizedPoint(window.__nuwaxMapUnproject(event.clientX, event.clientY));
  }

  function projectPoint(point) {
    if (!point || typeof window.__nuwaxMapProject !== 'function') return null;
    const projected = window.__nuwaxMapProject(point.x, point.y);
    if (!projected || !Number.isFinite(Number(projected.x)) || !Number.isFinite(Number(projected.y))) return null;
    if (Number.isFinite(Number(projected.scale)) && Number(projected.scale) > 0) state.overlayScale = Number(projected.scale);
    if (Number.isFinite(Number(projected.pixelWidth))) state.overlayWidth = Number(projected.pixelWidth);
    if (Number.isFinite(Number(projected.pixelHeight))) state.overlayHeight = Number(projected.pixelHeight);
    return { x: Number(projected.x), y: Number(projected.y) };
  }

  function rectangleCorners(vertices) {
    if (!vertices || vertices.length < 2) return [];
    const first = vertices[0];
    const second = vertices[1];
    return [
      { x: first.x, y: first.y },
      { x: second.x, y: first.y },
      { x: second.x, y: second.y },
      { x: first.x, y: second.y },
    ];
  }

  function circleVertices(center, radius) {
    const points = [];
    for (let index = 0; index < CIRCLE_SEGMENTS; index += 1) {
      const angle = Math.PI * 2 * index / CIRCLE_SEGMENTS;
      points.push({ x: center.x + Math.cos(angle) * radius, y: center.y + Math.sin(angle) * radius });
    }
    return points;
  }

  function displayVertices(zone) {
    if (!zone) return [];
    if (zone.shape === 'rectangle') return rectangleCorners(zone.vertices);
    if (zone.shape === 'circle') return circleVertices(zone.center, zone.radius_m);
    return (zone.vertices || []).slice();
  }

  function createSvgElement(tag, attributes) {
    const element = document.createElementNS(SVG_NS, tag);
    for (const [name, value] of Object.entries(attributes || {})) {
      if (value !== null && value !== undefined) element.setAttribute(name, String(value));
    }
    return element;
  }

  function projectedPath(points, closed) {
    const projected = points.map(projectPoint).filter(Boolean);
    if (projected.length < (closed ? 3 : 2)) return null;
    const path = projected.map((point, index) => (index ? 'L' : 'M') + point.x + ' ' + point.y).join(' ');
    return { path: path + (closed ? ' Z' : ''), points: projected };
  }

  function zoneStrokeWidth(zone) {
    return clamp(Math.max(2, Number(zone.stroke_width_m || 0.25) * state.overlayScale), 2, 34);
  }

  function appendZoneToOverlay(zone, options) {
    if (!state.overlaySvg || !zone) return;
    const opts = options || {};
    const closed = zone.shape !== 'line';
    const pathInfo = projectedPath(displayVertices(zone), closed);
    if (!pathInfo) return;
    const enabled = zone.enabled !== false;
    const outside = zone.mode === 'outside' && closed;
    const stroke = outside ? '#a855f7' : '#ef4444';
    const fill = outside ? '#7e22ce' : '#dc2626';
    const opacity = opts.draft ? 0.36 : enabled ? 0.22 : 0.06;

    if (enabled && closed) {
      if (outside && state.overlayWidth > 0 && state.overlayHeight > 0) {
        const outer = 'M0 0 H' + state.overlayWidth + ' V' + state.overlayHeight + ' H0 Z ';
        state.overlaySvg.appendChild(createSvgElement('path', {
          d: outer + pathInfo.path,
          fill,
          'fill-opacity': opts.draft ? 0.18 : 0.14,
          'fill-rule': 'evenodd',
          'clip-rule': 'evenodd',
          'pointer-events': 'none',
        }));
      } else {
        state.overlaySvg.appendChild(createSvgElement('path', {
          d: pathInfo.path,
          fill,
          'fill-opacity': opacity,
          'pointer-events': 'none',
        }));
      }
    }

    state.overlaySvg.appendChild(createSvgElement('path', {
      d: pathInfo.path,
      fill: 'none',
      stroke: enabled ? stroke : '#94a3b8',
      'stroke-width': zoneStrokeWidth(zone),
      'stroke-linecap': 'round',
      'stroke-linejoin': 'round',
      'stroke-dasharray': enabled ? (opts.draft ? '10 7' : null) : '7 7',
      'stroke-opacity': opts.draft ? 0.95 : enabled ? 0.92 : 0.65,
      'pointer-events': 'none',
    }));

    if (opts.vertices) {
      for (const point of pathInfo.points) {
        state.overlaySvg.appendChild(createSvgElement('circle', {
          cx: point.x,
          cy: point.y,
          r: 4.5,
          fill: '#fef2f2',
          stroke,
          'stroke-width': 2,
          'pointer-events': 'none',
        }));
      }
    }
  }

  function draftPreview() {
    if (state.draft) return state.draft;
    const shape = selectedShape();
    if (shape === 'polygon') {
      const vertices = state.customVertices.slice();
      if (state.customHover && vertices.length) vertices.push(state.customHover);
      if (vertices.length < 2) return null;
      return {
        shape: vertices.length >= 3 ? 'polygon' : 'line',
        mode: selectedMode(),
        frame_id: FRAME_ID,
        vertices,
        center: null,
        radius_m: null,
        stroke_width_m: selectedStrokeWidth(),
        enabled: true,
      };
    }
    if (!state.dragStart || !state.dragCurrent) return null;
    return geometryFromDrag(shape, state.dragStart, state.dragCurrent);
  }

  function clearOverlay() {
    if (!state.overlaySvg) return;
    state.overlaySvg.replaceChildren();
  }

  function syncOverlayMetrics() {
    const projected = typeof window.__nuwaxMapProject === 'function'
      ? window.__nuwaxMapProject(0, 0)
      : null;
    const canvas = document.getElementById('mapCanvas');
    state.overlayWidth = Number(projected && projected.pixelWidth) || Number(canvas && canvas.width) || 1;
    state.overlayHeight = Number(projected && projected.pixelHeight) || Number(canvas && canvas.height) || 1;
    state.overlayScale = Number(projected && projected.scale) || state.overlayScale || 1;
    if (state.overlaySvg) state.overlaySvg.setAttribute('viewBox', '0 0 ' + state.overlayWidth + ' ' + state.overlayHeight);
  }

  function redrawOverlay() {
    if (!state.overlaySvg) return;
    syncOverlayMetrics();
    clearOverlay();
    for (const zone of state.zones.filter((item) => item.mode === 'outside')) appendZoneToOverlay(zone);
    for (const zone of state.zones.filter((item) => item.mode !== 'outside')) appendZoneToOverlay(zone);
    const preview = draftPreview();
    if (preview) appendZoneToOverlay(preview, { draft: true, vertices: true });
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
    const overlay = createSvgElement('svg', {
      id: 'forbidden-zone-overlay',
      width: '100%',
      height: '100%',
      'aria-label': '导航禁行区覆盖层',
    });
    overlay.style.cssText = [
      'position:absolute',
      'inset:0',
      'width:100%',
      'height:100%',
      'z-index:2',
      'pointer-events:none',
      'touch-action:none',
      'overflow:hidden',
    ].join(';');
    host.appendChild(overlay);
    state.overlaySvg = overlay;
    bindOverlay(overlay);
    state.resizeObserver = new ResizeObserver(redrawOverlay);
    state.resizeObserver.observe(host);
    return overlay;
  }

  function selectedShape() {
    const value = String(state.shapeSelect && state.shapeSelect.value || 'line');
    return value === 'custom' ? 'polygon' : value;
  }

  function selectedMode() {
    return selectedShape() === 'line' ? 'inside' : state.modeSelect && state.modeSelect.value === 'outside' ? 'outside' : 'inside';
  }

  function selectedStrokeWidth() {
    const value = numberOrNull(state.widthInput && state.widthInput.value);
    return value === null ? 0.25 : clamp(value, 0.02, 5);
  }

  function geometryFromDrag(shape, start, end) {
    const distance = Math.hypot(end.x - start.x, end.y - start.y);
    if (distance < MIN_GEOMETRY_METERS) return null;
    const common = {
      shape,
      mode: shape === 'line' ? 'inside' : selectedMode(),
      frame_id: FRAME_ID,
      vertices: [],
      center: null,
      radius_m: null,
      stroke_width_m: selectedStrokeWidth(),
      enabled: true,
    };
    if (shape === 'line' || shape === 'rectangle') {
      common.vertices = [{ ...start }, { ...end }];
      if (shape === 'rectangle' && (Math.abs(end.x - start.x) < MIN_GEOMETRY_METERS || Math.abs(end.y - start.y) < MIN_GEOMETRY_METERS)) return null;
      return common;
    }
    if (shape === 'circle') {
      common.center = { ...start };
      common.radius_m = distance;
      return common;
    }
    return null;
  }

  function polygonAreaTwice(vertices) {
    let sum = 0;
    for (let index = 0; index < vertices.length; index += 1) {
      const current = vertices[index];
      const next = vertices[(index + 1) % vertices.length];
      sum += current.x * next.y - next.x * current.y;
    }
    return Math.abs(sum);
  }

  function cross(a, b, c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  function pointOnSegment(point, start, end) {
    const epsilon = 1e-10;
    return Math.abs(cross(start, end, point)) <= epsilon
      && point.x >= Math.min(start.x, end.x) - epsilon
      && point.x <= Math.max(start.x, end.x) + epsilon
      && point.y >= Math.min(start.y, end.y) - epsilon
      && point.y <= Math.max(start.y, end.y) + epsilon;
  }

  function segmentsIntersect(a, b, c, d) {
    const abC = cross(a, b, c);
    const abD = cross(a, b, d);
    const cdA = cross(c, d, a);
    const cdB = cross(c, d, b);
    if (abC * abD < 0 && cdA * cdB < 0) return true;
    return pointOnSegment(c, a, b) || pointOnSegment(d, a, b)
      || pointOnSegment(a, c, d) || pointOnSegment(b, c, d);
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

  function finishPolygon() {
    const vertices = state.customVertices.slice();
    if (vertices.length < 3) {
      setStatus('自定义多边形至少需要 3 个顶点', 'error');
      return false;
    }
    if (polygonAreaTwice(vertices) <= 1e-8) {
      setStatus('自定义多边形面积不能为零', 'error');
      return false;
    }
    if (polygonSelfIntersects(vertices)) {
      setStatus('自定义多边形不能自相交', 'error');
      return false;
    }
    state.draft = {
      shape: 'polygon',
      mode: selectedMode(),
      frame_id: FRAME_ID,
      vertices,
      center: null,
      radius_m: null,
      stroke_width_m: selectedStrokeWidth(),
      enabled: true,
    };
    state.customHover = null;
    redrawOverlay();
    syncDraftButtons();
    setStatus('多边形草稿已闭合，可保存', 'ready');
    return true;
  }

  function clearDraft() {
    state.activePointerId = null;
    state.dragStart = null;
    state.dragCurrent = null;
    state.customVertices = [];
    state.customHover = null;
    state.draft = null;
    syncDraftButtons();
    redrawOverlay();
  }

  function syncDraftButtons() {
    const custom = selectedShape() === 'polygon';
    if (state.btnFinish) state.btnFinish.hidden = !custom;
    if (state.btnUndo) state.btnUndo.hidden = !custom;
    if (state.btnFinish) state.btnFinish.disabled = state.busy || state.customVertices.length < 3;
    if (state.btnUndo) state.btnUndo.disabled = state.busy || state.customVertices.length < 1;
  }

  function syncShapeUi() {
    const shape = selectedShape();
    if (state.modeSelect) {
      state.modeSelect.disabled = shape === 'line';
      if (shape === 'line') state.modeSelect.value = 'inside';
    }
    if (state.hintEl) {
      state.hintEl.textContent = shape === 'polygon'
        ? '在地图上逐点单击，完成后保存；Esc 取消，Enter 闭合。'
        : '在地图上按住左键拖拽生成' + SHAPE_LABELS[shape] + '。';
    }
    clearDraft();
    syncDraftButtons();
  }

  function syncDraftOptions() {
    if (state.draft) {
      state.draft.mode = state.draft.shape === 'line' ? 'inside' : selectedMode();
      state.draft.stroke_width_m = selectedStrokeWidth();
    }
    redrawOverlay();
  }

  function setV2ActiveFlag(active) {
    try { window.__nuwaxBoundaryActive = Boolean(active); } catch (_) {}
  }

  function requestMapDraw() {
    if (typeof window.__nuwaxMapRequestDraw === 'function') window.__nuwaxMapRequestDraw();
  }

  function is2dMapVisible() {
    return typeof window.__nuwaxMapViewMode !== 'function' || window.__nuwaxMapViewMode() !== '3d';
  }

  function setEditing(active) {
    const next = Boolean(active) && is2dMapVisible();
    const overlay = ensureOverlay();
    if (next && !overlay) {
      setStatus('找不到地图容器 .canvas-wrap', 'error');
      return;
    }
    state.editing = next;
    if (state.overlaySvg) {
      state.overlaySvg.style.pointerEvents = next ? 'auto' : 'none';
      state.overlaySvg.style.cursor = next ? 'crosshair' : 'default';
    }
    setV2ActiveFlag(next);
    if (next) {
      const anchor = typeof window.__nuwaxCameraAnchor === 'function' ? window.__nuwaxCameraAnchor() : null;
      const point = anchor && anchor.point ? anchor.point : { x: 0, y: 0 };
      window.__nuwaxMapFreezeCenter = { x: Number(point.x) || 0, y: Number(point.y) || 0 };
      window.__nuwaxMapFreeze = true;
    } else {
      state.activePointerId = null;
      state.dragStart = null;
      state.dragCurrent = null;
      try { window.__nuwaxMapFreeze = false; } catch (_) {}
      try { delete window.__nuwaxMapFreezeCenter; } catch (_) {}
    }
    if (state.btnToggle) {
      state.btnToggle.textContent = next ? '停止绘制' : '开始绘制';
      state.btnToggle.dataset.active = next ? 'true' : 'false';
    }
    requestMapDraw();
    redrawOverlay();
  }

  function onPointerDown(event) {
    if (!state.editing || (event.pointerType === 'mouse' && event.button !== 0)) return;
    event.preventDefault();
    event.stopPropagation();
    const point = worldFromEvent(event);
    if (!point) return;
    if (selectedShape() === 'polygon') {
      const previous = state.customVertices[state.customVertices.length - 1];
      if (!previous || Math.hypot(point.x - previous.x, point.y - previous.y) >= MIN_GEOMETRY_METERS) {
        state.customVertices.push(point);
        state.draft = null;
      }
      state.customHover = point;
      syncDraftButtons();
      redrawOverlay();
      return;
    }
    state.activePointerId = event.pointerId;
    state.dragStart = point;
    state.dragCurrent = point;
    state.draft = null;
    try { state.overlaySvg.setPointerCapture(event.pointerId); } catch (_) {}
    redrawOverlay();
  }

  function onPointerMove(event) {
    if (!state.editing) return;
    const point = worldFromEvent(event);
    if (!point) return;
    if (selectedShape() === 'polygon') {
      if (state.customVertices.length) {
        state.customHover = point;
        redrawOverlay();
      }
      return;
    }
    if (!state.dragStart || (state.activePointerId !== null && event.pointerId !== state.activePointerId)) return;
    state.dragCurrent = point;
    redrawOverlay();
  }

  function onPointerUp(event) {
    if (!state.editing || selectedShape() === 'polygon' || !state.dragStart) return;
    if (state.activePointerId !== null && event.pointerId !== state.activePointerId) return;
    event.preventDefault();
    const point = worldFromEvent(event) || state.dragCurrent;
    state.draft = point ? geometryFromDrag(selectedShape(), state.dragStart, point) : null;
    state.dragStart = null;
    state.dragCurrent = null;
    state.activePointerId = null;
    try { state.overlaySvg.releasePointerCapture(event.pointerId); } catch (_) {}
    if (!state.draft) setStatus('几何尺寸过小，请重新绘制', 'error');
    else setStatus('草稿已生成，可保存', 'ready');
    redrawOverlay();
  }

  function onPointerCancel(event) {
    if (!state.editing) return;
    state.activePointerId = null;
    state.dragStart = null;
    state.dragCurrent = null;
    try { state.overlaySvg.releasePointerCapture(event.pointerId); } catch (_) {}
    redrawOverlay();
  }

  function bindOverlay(overlay) {
    overlay.addEventListener('pointerdown', onPointerDown);
    overlay.addEventListener('pointermove', onPointerMove);
    overlay.addEventListener('pointerup', onPointerUp);
    overlay.addEventListener('pointercancel', onPointerCancel);
  }

  function zoneName(zone) {
    return String(zone && zone.metadata && zone.metadata.name || zone && zone.id || '未命名禁行区');
  }

  function renderList() {
    if (!state.listEl) return;
    state.listEl.replaceChildren();
    if (!state.zones.length) {
      const empty = document.createElement('li');
      empty.className = 'forbidden-zone-empty';
      empty.textContent = '当前地图暂无禁行区。';
      state.listEl.appendChild(empty);
      return;
    }
    for (const zone of state.zones) {
      const item = document.createElement('li');
      item.className = 'forbidden-zone-item';
      const main = document.createElement('span');
      main.className = 'forbidden-zone-item-main';
      const name = document.createElement('span');
      name.className = 'forbidden-zone-item-name';
      name.textContent = zoneName(zone);
      const meta = document.createElement('span');
      meta.className = 'forbidden-zone-item-meta';
      meta.textContent = (zone.enabled ? '' : '停用 · ') + SHAPE_LABELS[zone.shape] + ' · ' + MODE_LABELS[zone.mode];
      main.append(name, meta);
      const remove = document.createElement('button');
      remove.type = 'button';
      remove.textContent = '删除';
      remove.disabled = state.busy;
      remove.addEventListener('click', function () { void deleteZone(zone.id); });
      item.append(main, remove);
      state.listEl.appendChild(item);
    }
  }

  function draftAsZone() {
    if (selectedShape() === 'polygon' && !state.draft && !finishPolygon()) return null;
    if (!state.draft) return null;
    const name = String(state.nameInput && state.nameInput.value || '').trim();
    if (!name) {
      setStatus('禁行区名称不能为空', 'error');
      return null;
    }
    return normalizeZone({
      ...state.draft,
      id: newZoneId(),
      mode: state.draft.shape === 'line' ? 'inside' : selectedMode(),
      stroke_width_m: selectedStrokeWidth(),
      enabled: !state.enabledSelect || state.enabledSelect.value !== 'false',
      metadata: {
        name,
        created_by: 'debug_panel',
      },
    });
  }

  async function saveDraft() {
    const zone = draftAsZone();
    if (!zone) {
      if (!state.draft) setStatus('请先在地图上绘制禁行区', 'error');
      return;
    }
    const saved = await replaceZones(state.zones.concat(zone), '禁行区已保存');
    if (!saved) return;
    clearDraft();
    if (state.nameInput) state.nameInput.value = '禁行区 ' + (state.zones.length + 1);
  }

  async function deleteZone(zoneId) {
    const next = state.zones.filter((zone) => zone.id !== zoneId);
    if (next.length === state.zones.length) return;
    await replaceZones(next, '禁行区已删除');
  }

  async function clearAllZones() {
    if (!state.zones.length) {
      clearDraft();
      return;
    }
    const cleared = await replaceZones([], '已清空当前地图的禁行区');
    if (cleared) clearDraft();
  }

  function undoPolygonVertex() {
    if (state.draft) state.draft = null;
    state.customVertices.pop();
    state.customHover = null;
    syncDraftButtons();
    redrawOverlay();
  }

  function syncMapView() {
    const is2d = is2dMapVisible();
    if (state.overlaySvg) state.overlaySvg.style.display = is2d ? 'block' : 'none';
    if (!is2d) {
      setEditing(false);
      if (state.container) state.container.hidden = true;
      document.getElementById('mapBoundaryToggle')?.classList.remove('active');
    } else {
      redrawOverlay();
    }
  }

  function onDrawTick() {
    syncMapView();
    redrawOverlay();
  }

  function onForbiddenZonesStream(event) {
    const preferred = state.mapName || state.observedMapName;
    const entry = entryFromEnvelope(event.detail, preferred);
    if (!entry) return;
    if (preferred && entry.mapName && entry.mapName !== preferred) return;
    if (entry.mapName === state.mapName && entry.generation <= state.generation) return;
    applyEntry(entry);
  }

  function onMapContextStream(event) {
    syncMapName(false, event.detail);
  }

  function onKeyDown(event) {
    if (!state.editing) return;
    if (event.key === 'Escape') {
      clearDraft();
      event.preventDefault();
    } else if (event.key === 'Enter' && selectedShape() === 'polygon') {
      finishPolygon();
      event.preventDefault();
    }
  }

  function mount() {
    state.container = document.getElementById('boundary-draw-container');
    if (!state.container) return;
    state.statusEl = state.container.querySelector('[data-role="boundary-status"]');
    state.hintEl = state.container.querySelector('[data-role="boundary-hint"]');
    state.listEl = state.container.querySelector('[data-role="boundary-list"]');
    state.nameInput = state.container.querySelector('[data-role="boundary-name"]');
    state.shapeSelect = state.container.querySelector('[data-role="boundary-shape"]');
    state.modeSelect = state.container.querySelector('[data-role="boundary-mode"]');
    state.widthInput = state.container.querySelector('[data-role="boundary-width"]');
    state.enabledSelect = state.container.querySelector('[data-role="boundary-enabled"]');
    state.btnToggle = state.container.querySelector('[data-role="boundary-toggle"]');
    state.btnFinish = state.container.querySelector('[data-role="boundary-finish"]');
    state.btnUndo = state.container.querySelector('[data-role="boundary-undo"]');
    state.btnSubmit = state.container.querySelector('[data-role="boundary-submit"]');
    state.btnDraftClear = state.container.querySelector('[data-role="boundary-draft-clear"]');
    state.btnClear = state.container.querySelector('[data-role="boundary-clear"]');

    ensureOverlay();
    state.shapeSelect?.addEventListener('change', syncShapeUi);
    state.modeSelect?.addEventListener('change', syncDraftOptions);
    state.widthInput?.addEventListener('input', syncDraftOptions);
    state.btnToggle?.addEventListener('click', function () { setEditing(!state.editing); });
    state.btnFinish?.addEventListener('click', finishPolygon);
    state.btnUndo?.addEventListener('click', undoPolygonVertex);
    state.btnSubmit?.addEventListener('click', function () { void saveDraft(); });
    state.btnDraftClear?.addEventListener('click', clearDraft);
    state.btnClear?.addEventListener('click', function () { void clearAllZones(); });
    document.addEventListener('keydown', onKeyDown);

    const boundaryButton = document.getElementById('mapBoundaryToggle');
    const navigationButton = document.getElementById('mapNavToggle');
    const mapModeButton = document.getElementById('mapModeToggle');
    boundaryButton?.addEventListener('click', function () {
      queueMicrotask(function () { if (state.container.hidden) setEditing(false); });
    });
    navigationButton?.addEventListener('click', function () {
      queueMicrotask(function () { if (state.container.hidden) setEditing(false); });
    });
    mapModeButton?.addEventListener('click', function () { queueMicrotask(syncMapView); });

    state.previousDrawTick = window.__nuwaxMapDrawTick;
    window.__nuwaxMapDrawTick = function () {
      if (typeof state.previousDrawTick === 'function') state.previousDrawTick();
      onDrawTick();
    };
    window.addEventListener('stream:navigation.forbidden_zones', onForbiddenZonesStream);
    window.addEventListener('stream:map.frame', onMapContextStream);
    window.addEventListener('stream:map.heartbeat', onMapContextStream);
    window.addEventListener('stream:runtime.governance', onMapContextStream);
    window.addEventListener('stream:pointcloud_map.meta', onMapContextStream);

    syncShapeUi();
    renderList();
    syncMapView();
    syncMapName(true);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount);
  else mount();

  window.__nuwaxBoundaryDrawModule = {
    clearDraft,
    getEnvelope: currentEnvelope,
    getZones: function () { return state.zones.map(canonicalZone); },
    reload: function () { syncMapName(true); },
    setDrawing: setEditing,
  };
})();
