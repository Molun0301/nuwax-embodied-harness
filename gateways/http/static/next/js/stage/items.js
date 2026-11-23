/* 记忆物品图层：从原控制台 console.js 迁出（loadSemanticInstances、applySemanticInstances、semanticQuality、
   semanticMapInstances、publishSemanticMapInstances、clusterInstances、drawSemanticInstances、hitTestSemanticMarkers 等），
   行为不变：只在当前地图、有位置依据的物品上画标记；屏幕上 28 像素内的标记合并为“N 个物品”，合并变化时 220 毫秒过渡；
   有物体尺寸的画出地面投影；导航目标对应的物品加高亮框，没有对应物品时在目标处画虚线环；读取失败时撤下标记。
   物品列表、详情与“走到它附近”在 P3 语义面板，这里只提供数据与点选回调。 */
import { fitLabel } from './video.js';

const CLUSTER_PX_THRESHOLD = 28;
const MARKER_HEIGHT = 24, MARKER_MIN_WIDTH = 40, MARKER_MAX_WIDTH = 168, MARKER_PADDING_X = 10;

export function instanceName(inst) { return String(inst && (inst.display_name || inst.label || inst.instance_id) || '?').trim() || '?'; }
export function observationOnly(inst) { return inst.observation_only === true || inst.projection_method === 'current_pose_fallback'; }
function finitePose(pose) { return Boolean(pose && [pose.x, pose.y, pose.z].every(v => typeof v === 'number' && Number.isFinite(v))); }
export function isFallbackInstance(inst) { return inst.is_fallback === true || observationOnly(inst) || (!inst.projection_method && inst.navigable !== true); }
export function isUnknownFrame(inst) { return String(inst.frame_id || '').trim() !== 'map'; }
export function instanceQuality(inst) {
  if (observationOnly(inst)) return { key: 'observation', label: '仅观察' };
  if (inst.lifecycle_state === 'stale') return { key: 'uncertain', label: '已过时' };
  if (inst.lifecycle_state === 'removed') return { key: 'uncertain', label: '已移除' };
  if (inst.lifecycle_state === 'uncertain') return { key: 'uncertain', label: '待确认' };
  if (isFallbackInstance(inst) || isUnknownFrame(inst) || !finitePose(inst.pose)) return { key: 'uncertain', label: '待确认' };
  return { key: 'located', label: '有位置' };
}
export function instanceExtent(inst) {
  const extent = inst.object_extent;
  if (!extent || extent.frame_id !== inst.frame_id || extent.frame_id !== 'map' || !['detection3d', 'observed_points'].includes(extent.source)) return null;
  return ['x', 'y', 'z'].every(axis => typeof extent.size_m?.[axis] === 'number' && Number.isFinite(extent.size_m[axis]) && extent.size_m[axis] >= 0)
    && Math.max(extent.size_m.x, extent.size_m.y, extent.size_m.z) > 0 ? extent : null;
}

/* mapName()：当前地图名；highlight()：导航目标高亮 {record_id, target_pose}；requestDraw()：请二维地图重画；
   onPick(hit)：点选物品或聚合，hit 为 {type: 'single', instance} 或 {type: 'cluster', cluster}。 */
export function createItems({ stream, api, mapName, highlight, requestDraw }) {
  const state = { instances: [], meta: {}, error: '', loadPromise: null, streamVersion: 0, snapshotKey: '', fetchedAt: 0,
    selectedId: null, mapCache: null, publishedKey: null, visible: true, anim: null, prevClusters: null };
  const listeners = new Set();
  const changed = () => { for (const fn of listeners) fn(); };

  function load() {
    if (state.loadPromise) return state.loadPromise;
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 8000);
    const requestedMap = mapName();
    const version = state.streamVersion;
    state.loadPromise = api.json('/api/memory/instances?limit=500', { signal: controller.signal })
      .then(body => {
        const src = Array.isArray(body?.instances) ? body : body?.result;
        if (!Array.isArray(src?.instances)) throw new Error('记忆接口返回格式异常');
        if (requestedMap && requestedMap !== mapName()) throw new Error('地图已切换，正在刷新当前记忆库');
        if (version !== state.streamVersion) return;
        apply(src);
      })
      .catch(error => {
        if (version !== state.streamVersion) return;
        state.error = error.name === 'AbortError' ? '读取超时' : String(error.message || error);
        // 失败时保留可查阅记录，但撤下地图标记，避免缓存被当作当前位置。
        publish();
        requestDraw();
      })
      .finally(() => {
        clearTimeout(timeout);
        state.loadPromise = null;
        changed();
        if (requestedMap && requestedMap !== mapName() && !document.hidden) setTimeout(load, 0);
      });
    changed();
    return state.loadPromise;
  }
  function apply(src) {
    if (!Array.isArray(src?.instances)) {
      state.error = src?.message || '空间记忆实时数据不可用';
      publish();
      requestDraw();
      changed();
      return;
    }
    const seen = new Set();
    const list = src.instances.filter(inst => {
      if (!inst || typeof inst !== 'object' || !inst.instance_id || seen.has(inst.instance_id)) return false;
      seen.add(inst.instance_id);
      return true;
    });
    const snapshotKey = JSON.stringify([src.library_name, src.map_name, list]);
    const wasError = Boolean(state.error);
    state.error = '';
    state.fetchedAt = Date.now();
    state.meta = src;
    if (snapshotKey !== state.snapshotKey || wasError) {
      state.snapshotKey = snapshotKey;
      state.instances = list;
      if (!list.some(inst => inst.instance_id === state.selectedId)) state.selectedId = null;
      publish();
      requestDraw();
    }
    changed();
  }
  function scopeMatches(inst) { const current = mapName(); return Boolean(current && inst.map_name && current === inst.map_name); }
  /* 当前地图上有位置依据的物品（地图标记与三维标记用）。 */
  function mapInstances() {
    if (state.error) return [];
    const current = mapName();
    const cache = state.mapCache;
    if (cache?.snapshot === state.snapshotKey && cache.mapName === current) return cache.instances;
    const instances = state.instances.filter(inst => scopeMatches(inst) && instanceQuality(inst).key === 'located');
    state.mapCache = { snapshot: state.snapshotKey, mapName: current, instances };
    return instances;
  }
  function publish() {
    const current = mapName();
    const previous = state.publishedKey;
    if (previous?.snapshot === state.snapshotKey && previous.mapName === current && previous.error === state.error) return;
    if (previous && previous.mapName !== current) select(null);
    state.publishedKey = { snapshot: state.snapshotKey, mapName: current, error: state.error };
    window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-instances', { detail: { instances: mapInstances(), selectedId: state.selectedId } }));
  }
  function select(id) {
    state.selectedId = id;
    window.pointcloud3d?.setSemanticSelectedId(id);
    requestDraw();
    changed();
  }

  /* ---- 二维标记 ---- */
  function markerMetrics(ctx, inst, highlighted) {
    const height = highlighted ? MARKER_HEIGHT + 2 : MARKER_HEIGHT;
    const font = highlighted ? '700 11px sans-serif' : '600 11px sans-serif';
    ctx.font = font;
    const label = fitLabel(ctx, instanceName(inst), MARKER_MAX_WIDTH - MARKER_PADDING_X * 2);
    const width = Math.max(MARKER_MIN_WIDTH, Math.min(MARKER_MAX_WIDTH, Math.ceil(ctx.measureText(label).width) + MARKER_PADDING_X * 2));
    return { width, height, radius: 4, label, font };
  }
  function clusterMarkerMetrics(ctx, count, highlighted) { return markerMetrics(ctx, { display_name: `${count} 个物品` }, highlighted); }
  function roundedRect(ctx, x, y, width, height, radius) {
    const r = Math.min(radius, width / 2, height / 2);
    ctx.beginPath();
    ctx.moveTo(x + r, y); ctx.lineTo(x + width - r, y); ctx.quadraticCurveTo(x + width, y, x + width, y + r);
    ctx.lineTo(x + width, y + height - r); ctx.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
    ctx.lineTo(x + r, y + height); ctx.quadraticCurveTo(x, y + height, x, y + height - r);
    ctx.lineTo(x, y + r); ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }
  function cluster(instances, project, width, height) {
    // 视口裁剪：只处理可见区域加边距；O(n) 网格聚合。
    const margin = CLUSTER_PX_THRESHOLD * 2;
    const visible = [];
    for (const inst of instances) {
      if (!inst.pose) continue;
      const cp = project({ x: inst.pose.x, y: inst.pose.y });
      if (!cp || cp.x < -margin || cp.x > width + margin || cp.y < -margin || cp.y > height + margin) continue;
      visible.push({ inst, cx: cp.x, cy: cp.y });
    }
    if (visible.length > 1000) visible.splice(1000);
    const grid = new Map();
    for (const item of visible) {
      const gx = Math.floor(item.cx / CLUSTER_PX_THRESHOLD), gy = Math.floor(item.cy / CLUSTER_PX_THRESHOLD);
      let merged = false;
      for (let dx = -1; dx <= 1 && !merged; dx++) for (let dy = -1; dy <= 1 && !merged; dy++) {
        for (const cell of grid.get(`${gx + dx},${gy + dy}`) || []) {
          const lx = cell.sumX / cell.members.length, ly = cell.sumY / cell.members.length;
          if (Math.hypot(item.cx - lx, item.cy - ly) < CLUSTER_PX_THRESHOLD) {
            cell.members.push(item.inst); cell.sumX += item.cx; cell.sumY += item.cy; merged = true; break;
          }
        }
      }
      if (!merged) {
        const key = `${gx},${gy}`;
        if (!grid.has(key)) grid.set(key, []);
        grid.get(key).push({ members: [item.inst], sumX: item.cx, sumY: item.cy });
      }
    }
    const clusters = [];
    for (const cells of grid.values()) for (const cell of cells) {
      clusters.push({ key: cell.members.map(m => m.instance_id).sort().join('|'), members: cell.members,
        cx: cell.sumX / cell.members.length, cy: cell.sumY / cell.members.length });
    }
    return clusters;
  }
  function highlightCluster(clusters) {
    const ht = highlight();
    if (!ht) return null;
    if (ht.record_id) for (const cl of clusters) for (const m of cl.members) if (m.instance_id === ht.record_id) return cl.key;
    if (ht.target_pose) {
      const tp = ht.target_pose;
      let bestDist = 80, bestKey = null;
      for (const cl of clusters) for (const m of cl.members) {
        if (!m.pose) continue;
        const d = Math.hypot(m.pose.x - (tp.x || 0), m.pose.y - (tp.y || 0));
        if (d < bestDist) { bestDist = d; bestKey = cl.key; }
      }
      if (bestKey) return bestKey;
    }
    return null;
  }
  function footprint(inst, project) {
    const points = instanceExtent(inst)?.footprint_xy;
    if (!Array.isArray(points) || points.length < 2 || points.length > 8
      || !points.every(point => typeof point.x === 'number' && Number.isFinite(point.x) && typeof point.y === 'number' && Number.isFinite(point.y))) return [];
    return points.map(project).filter(Boolean);
  }
  function drawFootprints(ctx, instances, project) {
    for (const inst of instances) {
      const points = footprint(inst, project);
      if (points.length < 2) continue;
      const partial = inst.object_extent.source === 'observed_points';
      const selected = inst.instance_id === state.selectedId || inst.instance_id === highlight()?.record_id;
      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      for (const point of points.slice(1)) ctx.lineTo(point.x, point.y);
      if (points.length > 2) ctx.closePath();
      ctx.fillStyle = selected ? 'rgba(138,167,184,.22)' : 'rgba(138,167,184,.10)';
      ctx.strokeStyle = selected ? 'rgba(106,150,177,.9)' : 'rgba(138,167,184,.55)';
      ctx.lineWidth = selected ? 2.5 : 1.5;
      ctx.setLineDash(partial ? [5, 3] : []);
      if (points.length > 2) ctx.fill();
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  function drawTargetRing(ctx, project) {
    const tp = highlight()?.target_pose;
    const cp = tp && project({ x: tp.x || 0, y: tp.y || 0 });
    if (!cp) return;
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2.5;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.arc(cp.x, cp.y, 18, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  /* 在像素坐标下画（调用方已重置变换）；project 把地图坐标换算为画布像素。 */
  function draw(ctx, project, width, height) {
    if (!state.visible) return;
    const instances = mapInstances();
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    if (!instances.length) { drawTargetRing(ctx, project); ctx.restore(); return; }
    const clusters = cluster(instances, project, width, height);
    const anim = state.anim, now = performance.now();
    let t = 1;
    if (anim?.active) {
      const raw = Math.min(1, (now - anim.startTs) / anim.durationMs);
      t = raw < 1 ? 1 - Math.pow(1 - raw, 3) : 1;
      if (raw >= 1) anim.active = false;
    }
    startAnimIfChanged(clusters, now);
    state.prevClusters = clusters;
    const hlKey = highlightCluster(clusters);
    drawFootprints(ctx, instances, project);
    for (const cl of clusters) {
      let x = cl.cx, y = cl.cy;
      if (anim?.active && anim.from[cl.key]) { const from = anim.from[cl.key]; x = from.cx + (cl.cx - from.cx) * t; y = from.cy + (cl.cy - from.cy) * t; }
      const highlighted = cl.key === hlKey;
      const fallbackCount = cl.members.filter(m => isFallbackInstance(m) || isUnknownFrame(m)).length;
      if (cl.members.length === 1) drawSingle(ctx, x, y, cl.members[0], fallbackCount === 1, isUnknownFrame(cl.members[0]), highlighted);
      else drawCluster(ctx, x, y, cl, fallbackCount === cl.members.length, fallbackCount, highlighted);
    }
    if (!hlKey) drawTargetRing(ctx, project);
    ctx.restore();
  }
  function drawSingle(ctx, x, y, inst, fallback, unknownFrame, highlighted) {
    const marker = markerMetrics(ctx, inst, highlighted);
    const left = x - marker.width / 2, top = y - marker.height / 2;
    roundedRect(ctx, left, top, marker.width, marker.height, marker.radius);
    if (fallback || unknownFrame) {
      ctx.setLineDash([4, 3]);
      ctx.strokeStyle = highlighted ? 'rgba(189,215,229,.95)' : 'rgba(148,163,184,.55)';
      ctx.lineWidth = 1.8;
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = highlighted ? 'rgba(51,65,85,.84)' : 'rgba(51,65,85,.76)';
      ctx.fill();
    } else {
      ctx.fillStyle = highlighted ? 'rgba(38,55,70,.84)' : 'rgba(38,55,70,.76)';
      ctx.fill();
      ctx.strokeStyle = highlighted ? 'rgba(189,215,229,.95)' : 'rgba(151,177,194,.62)';
      ctx.lineWidth = highlighted ? 2.5 : 1.5;
      ctx.stroke();
    }
    if (highlighted) {
      ctx.strokeStyle = 'rgba(106,150,177,.85)';
      ctx.lineWidth = 2;
      roundedRect(ctx, left - 5, top - 5, marker.width + 10, marker.height + 10, marker.radius + 2);
      ctx.stroke();
    }
    ctx.font = marker.font;
    ctx.fillStyle = highlighted ? '#f1f7fa' : 'rgba(232,241,246,.96)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(marker.label, x, y + 0.5);
    ctx.textBaseline = 'alphabetic';
    if (unknownFrame || fallback) {
      ctx.font = '9px sans-serif';
      ctx.fillStyle = '#6b7280';
      ctx.fillText(unknownFrame ? '坐标系未知' : '待确认', x, top + marker.height + 11);
    }
  }
  function drawCluster(ctx, x, y, cl, allFallback, fallbackCount, highlighted) {
    const marker = clusterMarkerMetrics(ctx, cl.members.length, highlighted);
    const left = x - marker.width / 2, top = y - marker.height / 2;
    if (allFallback) {
      ctx.setLineDash([4, 3]);
      ctx.strokeStyle = highlighted ? 'rgba(189,215,229,.95)' : 'rgba(148,163,184,.55)';
      ctx.lineWidth = 2;
      ctx.fillStyle = highlighted ? 'rgba(51,65,85,.84)' : 'rgba(51,65,85,.76)';
    } else {
      ctx.setLineDash([]);
      ctx.fillStyle = highlighted ? 'rgba(38,55,70,.84)' : 'rgba(38,55,70,.76)';
      ctx.strokeStyle = highlighted ? 'rgba(189,215,229,.95)' : 'rgba(151,177,194,.62)';
      ctx.lineWidth = highlighted ? 2.5 : 2;
    }
    roundedRect(ctx, left, top, marker.width, marker.height, marker.radius);
    ctx.fill();
    ctx.stroke();
    ctx.setLineDash([]);
    if (!allFallback && fallbackCount > 0) {
      const ratio = (cl.members.length - fallbackCount) / cl.members.length, barY = top + marker.height + 4;
      ctx.lineCap = 'butt';
      ctx.lineWidth = 3;
      ctx.beginPath(); ctx.strokeStyle = 'rgba(138,167,184,.65)'; ctx.moveTo(left, barY); ctx.lineTo(left + marker.width * ratio, barY); ctx.stroke();
      ctx.beginPath(); ctx.strokeStyle = 'rgba(148,163,184,.55)'; ctx.moveTo(left + marker.width * ratio, barY); ctx.lineTo(left + marker.width, barY); ctx.stroke();
    }
    if (highlighted) {
      ctx.strokeStyle = 'rgba(106,150,177,.85)';
      ctx.lineWidth = 2;
      roundedRect(ctx, left - 5, top - 5, marker.width + 10, marker.height + 10, marker.radius + 2);
      ctx.stroke();
    }
    ctx.font = marker.font;
    ctx.fillStyle = highlighted ? '#f1f7fa' : 'rgba(232,241,246,.96)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(marker.label, x, y + 0.5);
    ctx.textBaseline = 'alphabetic';
    if (allFallback) { ctx.font = '9px sans-serif'; ctx.fillStyle = '#6b7280'; ctx.fillText('待确认', x, top + marker.height + 11); }
  }
  function startAnimIfChanged(next, now) {
    const prev = state.prevClusters;
    if (!prev || !prev.length) return;
    const prevMap = new Map(prev.map(c => [c.key, c]));
    let differs = prev.length !== next.length;
    for (const c of next) { const o = prevMap.get(c.key); if (!o || Math.abs(o.cx - c.cx) > 1 || Math.abs(o.cy - c.cy) > 1) { differs = true; break; } }
    if (!differs) return;
    const from = {};
    for (const [key, o] of prevMap) from[key] = { cx: o.cx, cy: o.cy };
    if (state.anim?.active) { state.anim.from = from; state.anim.startTs = now; return; }
    state.anim = { active: true, startTs: now, durationMs: 220, from };
    const step = () => {
      if (!state.anim?.active) return;
      requestDraw();
      if (performance.now() - state.anim.startTs >= state.anim.durationMs) { state.anim.active = false; return; }
      requestAnimationFrame(step);
    };
    step();
  }
  /* 点选命中：先找标记，再找物体地面投影；ctx 只用来量文字宽度。 */
  function hitTest(ctx, px, py, project, width, height) {
    if (!state.visible || !state.instances.length) return null;
    const instances = mapInstances();
    const clusters = cluster(instances, project, width, height);
    const hlKey = highlightCluster(clusters);
    let bestDist = Infinity, best = null;
    ctx.save();
    for (const cl of clusters) {
      const highlighted = cl.key === hlKey;
      const marker = cl.members.length === 1 ? markerMetrics(ctx, cl.members[0], highlighted) : clusterMarkerMetrics(ctx, cl.members.length, highlighted);
      const dist = Math.hypot(px - cl.cx, py - cl.cy);
      if (Math.abs(px - cl.cx) <= marker.width / 2 + 4 && Math.abs(py - cl.cy) <= marker.height / 2 + 4 && dist < bestDist) {
        bestDist = dist;
        best = cl.members.length === 1 ? { type: 'single', instance: cl.members[0] } : { type: 'cluster', cluster: cl };
      }
    }
    ctx.restore();
    if (best) return best;
    for (const inst of instances) {
      const points = footprint(inst, project);
      let inside = false;
      for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
        const a = points[i], b = points[j];
        if ((a.y > py) !== (b.y > py) && px < (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x) inside = !inside;
      }
      if (inside) return { type: 'single', instance: inst };
    }
    return null;
  }

  stream.on('memory.instances', frame => { state.streamVersion += 1; apply(frame.payload || {}); });
  stream.onGap(({ topics }) => { if (!topics || topics.has('memory.instances')) void load(); });
  stream.onClose(() => { state.streamVersion += 1; apply({ message: '实时连接已断开，正在重连' }); });

  return {
    load, draw, hitTest, select, publish, mapInstances,
    onChange(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    setVisible(value) {
      state.visible = value;
      window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-visibility', { detail: { visible: value } }));
      requestDraw();
    },
    get visible() { return state.visible; },
    get selectedId() { return state.selectedId; },
    get instances() { return state.instances; },
    get meta() { return state.meta; },
    get error() { return state.error; },
    get loading() { return Boolean(state.loadPromise); },
    get fetchedAt() { return state.fetchedAt; },
  };
}
