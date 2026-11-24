/* 地图核心：从原控制台 console.js 迁出地图、位姿、导航与探索叠加、二维相机与绘制、三维入口，算法与判定不变。
   - 二维：地图帧（含二进制帧）、心跳、显示判定（会话结束、等待栅格）、跟随视角（以机器人为中心、车头朝上）与
     自由视角（左键旋转、中键或右键平移、滚轮缩放、复位适配全图）、轨迹、全局与局部路线、目标、选点、前沿、机器人；
   - 位姿走独立连接（与地图帧分开排队，缺口时隐藏位姿并重连）；只在定位可用时显示位姿与轨迹；
   - 三维：沿用 pointcloud3d.js，按原来的 stream:* 事件与明细喂数据，点云快照二进制分流，增量点云握手成功后撤下全量快照；
   - 面板模块（P3 迁入的 navigation.js、boundary_draw.js 等）用到的 window.__nuwaxMap* 钩子保留。
   不迁：总览卡、定位与导航详情键值表（旧界面）；runtime.governance 分支（网关不发布该主题）；从未调用的显示判定函数；
   探索叠加“规范身份”校验（原代码从未接受过身份，断线一次后前沿帧被永久丢弃，这里断线或缺口时清空叠加、之后照常接收）。 */
import { createStream } from '../stream.js';
import { videoSourceTime } from './video.js';

export const PLANAR_PIPELINE = window.__NUWAX_GEOMETRY_PIPELINE__ === 'slam_2d';
// 叠加色沿用原值；底色与栅格按舞台配色（浅色舞台 / 暗色主题）取。
const COLORS = {
  trajectory: '#b2a0f8', frontier: '#f472b6', frontierStroke: '#fbcfe8', robot: '#fbbf24', robotStroke: '#fef3c7',
  goal: '#a78bfa', goalStroke: '#ddd6fe', navPath: '#a5e27e', localPath: '#69dedc', retainedPath: '#ffba78',
};
const GRID_COLORS = { light: { free: [234, 236, 239], occupied: [34, 38, 43] }, dark: { free: [35, 39, 44], occupied: [141, 153, 163] } };
const TERMINAL_NAV = ['succeeded', 'failed', 'cancelled', 'replaced', 'stopped'];
const ACTIVE_SPATIAL_STATUSES = new Set(['accepted', 'planning', 'running']);
const ACTIVE_SPATIAL_CAPABILITIES = new Set(['explore_area_v2', 'explore_and_find_target', 'navigate_to_pose', 'navigate_to_named_location',
  'find_target', 'inspect_target', 'relocalize_active_map', 'load_map']);
const POINTCLOUD3D_STREAM_TOPICS = new Set(['map.frame', 'map.heartbeat', 'localization.pose', 'localization.trajectory', 'navigation.state',
  'navigation.forbidden_zones', 'exploration.state', 'exploration.frontier', 'pointcloud_map.meta']);
export const MAP_TOPICS = ['map.frame', 'map.heartbeat', 'localization.frame_tree', 'localization.trajectory', 'navigation.state', 'navigation.forbidden_zones',
  'exploration.state', 'exploration.frontier', 'video.detections', 'memory.instances', 'memory.scene_spaces', 'settings.display',
  ...(PLANAR_PIPELINE ? [] : ['pointcloud_map.meta', 'pointcloud_map.snapshot'])];
const NORTH_UP_YAW = Math.PI / 2;

/* ---------- 几何 ---------- */
export function pointFromPose(pose) {
  const pos = pose && pose.position;
  if (!pos) return null;
  const x = Number(pos.x), y = Number(pos.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  return { x, y, z: Number(pos.z || 0) };
}
export function yawFromPose(pose) {
  const q = pose && pose.orientation;
  if (!q) return null;
  const siny = 2 * ((Number(q.w) || 0) * (Number(q.z) || 0) + (Number(q.x) || 0) * (Number(q.y) || 0));
  const cosy = 1 - 2 * ((Number(q.y) || 0) * (Number(q.y) || 0) + (Number(q.z) || 0) * (Number(q.z) || 0));
  return Math.atan2(siny, cosy);
}
export function planarYawFromPose(pose) {
  const q = pose && pose.orientation;
  if (!q) return yawFromPose(pose);
  const fin = (v, d) => (Number.isFinite(Number(v)) ? Number(v) : d);
  const x = fin(q.x, 0), y = fin(q.y, 0), z = fin(q.z, 0), w = fin(q.w, 1);
  const forwardX = 1 - 2 * (y * y + z * z), forwardY = 2 * (x * y + w * z);
  if (Math.hypot(forwardX, forwardY) < 1e-6) return yawFromPose(pose);
  return Math.atan2(forwardY, forwardX);
}
function normalizeFrameId(value, fallback) { return String(value || fallback || 'map').trim().replace(/^\/+/, '') || 'map'; }
function gridResolution(grid) {
  if (!grid) return 1;
  for (const value of [grid.resolution_m, grid.resolution, grid.metadata && grid.metadata.resolution]) {
    const number = Number(value);
    if (Number.isFinite(number) && number > 0) return number;
  }
  return 1;
}
function originPosition(grid) {
  const origin = grid && grid.origin;
  if (!origin) return { x: 0, y: 0, z: 0 };
  if (origin.position) return origin.position;
  if (origin.x !== undefined && origin.y !== undefined) return { x: Number(origin.x), y: Number(origin.y), z: Number(origin.z || 0) };
  return { x: 0, y: 0, z: 0 };
}
function originYaw(grid) { return planarYawFromPose(grid && grid.origin) || 0; }
function mapCorners(grid) {
  if (!grid || !grid.width || !grid.height) return [];
  const origin = originPosition(grid), yaw = originYaw(grid), cos = Math.cos(yaw), sin = Math.sin(yaw);
  const resolution = gridResolution(grid), width = Number(grid.width) * resolution, height = Number(grid.height) * resolution;
  return [[0, 0], [width, 0], [width, height], [0, height]].map(([x, y]) => ({ x: Number(origin.x || 0) + x * cos - y * sin, y: Number(origin.y || 0) + x * sin + y * cos }));
}
function firstFiniteNumber(...values) {
  for (const value of values) {
    if (value === undefined || value === null || value === '') continue;
    const number = Number(value);
    if (Number.isFinite(number)) return number;
  }
  return null;
}
function normalizedCursor(value) {
  if (typeof value === 'string' && /^(0|[1-9]\d*)$/.test(value)) return value;
  if (typeof value === 'number' && Number.isSafeInteger(value) && value >= 0) return String(value);
  return null;
}
function compareCursor(left, right) {
  if (left.length !== right.length) return left.length < right.length ? -1 : 1;
  return left === right ? 0 : left < right ? -1 : 1;
}
function statusOf(envelope) { return envelope && envelope.status ? String(envelope.status) : 'missing'; }
function decodeMapBinaryFrame(buffer) {
  const bytes = new Uint8Array(buffer);
  const magic = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x4d, 0x41, 0x50];
  if (bytes.byteLength < 13 || !magic.every((value, index) => bytes[index] === value) || bytes[8] !== 4) return null;
  const headerLength = new DataView(buffer, 9, 4).getUint32(0, true);
  if (13 + headerLength > bytes.byteLength) return null;
  const message = JSON.parse(new TextDecoder().decode(bytes.subarray(13, 13 + headerLength)));
  const frame = message.frame;
  const grid = frame.payload.grid;
  const cells = grid.width * grid.height;
  if (13 + headerLength + cells + frame.payload.raw_data_length > bytes.byteLength) return null;
  grid.data = new Int8Array(buffer, 13 + headerLength, cells);
  frame.payload.raw_data = new Int8Array(buffer, 13 + headerLength + cells, frame.payload.raw_data_length);
  delete frame.payload.raw_data_length;
  return frame;
}
function streamBytes(value) {
  if (value instanceof Int8Array) return value;
  return Int8Array.from(atob(value.data), item => item.charCodeAt(0));
}
function mapForCanvas(update) {
  if (!update) return {};
  const grid = update.grid;
  return { map_name: grid.map_name, occupancy_grid: { ...grid, map_id: grid.map_name, resolution_m: grid.resolution, data: streamBytes(grid.data),
    origin: { position: { x: grid.origin_x, y: grid.origin_y, z: 0 } } } };
}
function poseForCanvas(pose) {
  return pose ? { position: { x: pose.x, y: pose.y, z: pose.z }, orientation: { x: pose.qx, y: pose.qy, z: pose.qz, w: pose.qw },
    frame_id: pose.frame_id, timestamp: new Date(pose.timestamp_ns / 1e6).toISOString() } : null;
}

/* ctx：外壳上下文；canvas：二维画布；container3d：三维容器；layers：{regions, graph, items}；
   metrics：帧率记录；visible()：舞台当前是否可见；onChange()：状态条需要刷新；
   onPick({type: 'item'|'cluster'|'region', ...})：二维点选物品或区域。 */
export function createMapView({ ctx, canvas, container3d, metrics, visible, onChange, onPick }) {
  const { store, stream } = ctx;
  const state = {
    mapGrid: null, mapZoom: 1, map2dCameraMode: 'follow', mapFreeView: { center: null, zoom: 1, fitSpan: null },
    viewMode: PLANAR_PIPELINE ? '2d' : '3d', pc3dCameraMode: 'free', pc3dZoomPercent: 100, pc3dSourcePoints: 0, pc3dDisplayedPoints: 0,
    pc3dPointCountKnown: false, pc3dReady: false, dense: null, pointStyle: 'height', robotModelText: '',
    mapPanDrag: null, mapRotateDrag: null, regionPointerDown: null, mapStream: null, pose: null, latestPose: null, trajectory: [],
    frontiers: [], goalPose: null, navigationPath: [], navigationLocalPath: [], navigationDisplay: null, highlightTarget: null,
    latestFrames: {}, mapStreamReady: false, appliedMapIdentity: null, mapDrawFrame: null, mapBaseCache: null, mapGridContentKey: '',
    lastMapEnvelope: null, lastLocalizationEnvelope: null, lastMapHeartbeatEnvelope: null, latestFrontierStream: null,
    latestGeometryState: null, displayedPoseMetrics: {}, poseReceipts: new Map(), emptyMessage: '等待地图数据。', streamEpoch: 0, heartbeatEpoch: -1,
  };
  let regions = null, graph = null, items = null;
  const emit = () => onChange();

  /* ---------- 相机 ---------- */
  const usesFreeMapView = () => state.map2dCameraMode === 'free';
  const freeViewYaw = () => (Number.isFinite(Number(state.mapFreeView.yaw)) ? Number(state.mapFreeView.yaw) : NORTH_UP_YAW);
  const mapViewZoom = () => (usesFreeMapView() ? (state.mapFreeView.zoom || 1) : state.mapZoom);
  function defaultMapViewCenter() {
    const posePoint = pointFromPose(state.pose);
    if (posePoint) return { x: posePoint.x, y: posePoint.y };
    const corners = mapCorners(state.mapGrid);
    if (!corners.length) return { x: 0, y: 0 };
    return corners.reduce((acc, item) => ({ x: acc.x + item.x / corners.length, y: acc.y + item.y / corners.length }), { x: 0, y: 0 });
  }
  function getFreeMapViewCenter() {
    const center = state.mapFreeView.center;
    if (center && Number.isFinite(Number(center.x)) && Number.isFinite(Number(center.y))) return { x: Number(center.x), y: Number(center.y) };
    return defaultMapViewCenter();
  }
  function resetFreeMapView() {
    const corners = mapCorners(state.mapGrid);
    const center = corners.length
      ? corners.reduce((sum, point) => ({ x: sum.x + point.x / corners.length, y: sum.y + point.y / corners.length }), { x: 0, y: 0 })
      : defaultMapViewCenter();
    state.mapFreeView = { center, zoom: 1, fitSpan: null, yaw: NORTH_UP_YAW };
    if (window.__nuwaxMapFreeze) window.__nuwaxMapFreezeCenter = { ...center };
    if (state.mapGrid) basePixelsPerMeterFree();
  }
  function cameraAnchor() {
    // 禁行区绘制时冻结相机：中心由 boundary_draw.js 写入，地图视觉静止。
    if (window.__nuwaxMapFreeze && window.__nuwaxMapFreezeCenter) {
      const fc = window.__nuwaxMapFreezeCenter;
      return { point: { x: Number(fc.x) || 0, y: Number(fc.y) || 0, z: 0 }, yaw: 0, locked: false, mapFixed: true };
    }
    if (usesFreeMapView()) {
      const center = getFreeMapViewCenter(), yaw = freeViewYaw();
      const northUp = Math.abs(yaw - NORTH_UP_YAW) < 1e-9;
      return { point: { x: center.x, y: center.y, z: 0 }, yaw: northUp ? 0 : yaw, locked: false, mapFixed: northUp };
    }
    const posePoint = pointFromPose(state.pose);
    if (posePoint) return { point: posePoint, yaw: planarYawFromPose(state.pose) || 0, locked: true, mapFixed: false };
    const fallback = defaultMapViewCenter();
    // 无位姿时按正北朝上显示。
    return { point: { x: fallback.x, y: fallback.y, z: 0 }, yaw: 0, locked: false, mapFixed: true };
  }
  function basePixelsPerMeterFree() {
    const fitSpan = state.map2dCameraMode === 'free' && state.mapFreeView.fitSpan;
    if (fitSpan) return Math.max(4, Math.min(canvas.width / fitSpan.x, canvas.height / fitSpan.y));
    const corners = mapCorners(state.mapGrid);
    if (!corners.length) return 60;
    const xs = corners.map(c => Number(c.x)), ys = corners.map(c => Number(c.y));
    const spanX = Math.max(Math.max(...xs) - Math.min(...xs), 1), spanY = Math.max(Math.max(...ys) - Math.min(...ys), 1);
    // 自由视角只在首次入图、切图或明确复位时适配，地图增量不能推动用户镜头。
    if (state.map2dCameraMode === 'free') state.mapFreeView.fitSpan = { x: spanX * 1.12, y: spanY * 1.12 };
    return Math.max(4, Math.min(canvas.width / (spanX * 1.12), canvas.height / (spanY * 1.12)));
  }
  function basePixelsPerMeter(anchor) {
    if (anchor && anchor.mapFixed) return basePixelsPerMeterFree();
    const corners = mapCorners(state.mapGrid);
    if (!corners.length) return 60;
    let maxX = 1, maxY = 1;
    const cos = Math.cos(anchor.yaw || 0), sin = Math.sin(anchor.yaw || 0);
    for (const corner of corners) {
      const dx = Number(corner.x) - anchor.point.x, dy = Number(corner.y) - anchor.point.y;
      maxX = Math.max(maxX, Math.abs(sin * dx - cos * dy));
      maxY = Math.max(maxY, Math.abs(cos * dx + sin * dy));
    }
    return Math.max(4, Math.min(canvas.width / (maxX * 2.16), canvas.height / (maxY * 2.16)));
  }
  const cameraScale = anchor => basePixelsPerMeter(anchor) * mapViewZoom();
  function worldToCanvasMatrix(anchor, scale) {
    const centerX = canvas.width / 2, centerY = canvas.height / 2;
    if (anchor && anchor.mapFixed) {
      const ax = Number(anchor.point.x) || 0, ay = Number(anchor.point.y) || 0;
      return { a: scale, b: 0, c: 0, d: -scale, e: centerX - scale * ax, f: centerY + scale * ay };
    }
    const sin = Math.sin(anchor.yaw || 0), cos = Math.cos(anchor.yaw || 0);
    const a = scale * sin, b = -scale * cos, c = -scale * cos, d = -scale * sin;
    return { a, b, c, d, e: centerX - a * anchor.point.x - c * anchor.point.y, f: centerY - b * anchor.point.x - d * anchor.point.y };
  }
  function applyCameraTransform(context, anchor, scale) {
    const m = worldToCanvasMatrix(anchor, scale);
    context.setTransform(m.a, m.b, m.c, m.d, m.e, m.f);
  }
  function mapToCanvas(point, anchorOverride, scaleOverride) {
    if (!point) return null;
    const anchor = anchorOverride || cameraAnchor();
    const scale = scaleOverride || cameraScale(anchor);
    const centerX = canvas.width / 2, centerY = canvas.height / 2;
    const dx = Number(point.x) - Number(anchor.point.x), dy = Number(point.y) - Number(anchor.point.y);
    if (anchor.mapFixed) return { x: centerX + dx * scale, y: centerY - dy * scale };
    const yaw = anchor.yaw || 0;
    return { x: centerX + (Math.sin(yaw) * dx - Math.cos(yaw) * dy) * scale, y: centerY - (Math.cos(yaw) * dx + Math.sin(yaw) * dy) * scale };
  }
  function canvasToMap(px, py, anchorOverride) {
    const anchor = anchorOverride || cameraAnchor();
    const scale = cameraScale(anchor);
    if (!Number.isFinite(scale) || scale <= 0) return { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
    const centerX = canvas.width / 2, centerY = canvas.height / 2;
    if (anchor.mapFixed) return { x: Number(anchor.point.x) + (px - centerX) / scale, y: Number(anchor.point.y) - (py - centerY) / scale };
    const { a, b, c, d, e, f } = worldToCanvasMatrix(anchor, scale);
    const det = a * d - b * c;
    if (!Number.isFinite(det) || Math.abs(det) < 1e-12) return { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
    const sx = px - e, sy = py - f;
    return { x: (d * sx - c * sy) / det, y: (-b * sx + a * sy) / det };
  }
  function zoomFreeMapAt(pointer, factor) {
    const anchor = cameraAnchor();
    const scaleBefore = cameraScale(anchor);
    if (!Number.isFinite(scaleBefore) || scaleBefore <= 0) return;
    const world = canvasToMap(pointer.x, pointer.y, anchor);
    state.mapFreeView.zoom = Math.min(8, Math.max(0.25, mapViewZoom() * factor));
    const scaleAfter = cameraScale(cameraAnchor());
    state.mapFreeView.center = { x: world.x - (pointer.x - canvas.width / 2) / scaleAfter, y: world.y + (pointer.y - canvas.height / 2) / scaleAfter };
    if (state.map2dCameraMode === 'free' && window.__nuwaxMapFreeze) window.__nuwaxMapFreezeCenter = { ...state.mapFreeView.center };
  }
  function canvasPointer(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: (event.clientX - rect.left) * canvas.width / Math.max(rect.width, 1), y: (event.clientY - rect.top) * canvas.height / Math.max(rect.height, 1) };
  }
  function setMapZoom(nextZoom) {
    const zoom = Math.min(8, Math.max(0.25, nextZoom));
    if (usesFreeMapView()) state.mapFreeView.zoom = zoom; else state.mapZoom = zoom;
    requestMapDraw(false);
  }
  function setMap2dCameraMode(mode) {
    if (state.mapPanDrag || window.__nuwaxBoundaryActive) return;
    state.map2dCameraMode = mode === 'free' ? 'free' : 'follow';
    if (state.map2dCameraMode === 'free' && !state.mapFreeView.center) resetFreeMapView();
    requestMapDraw(false);
  }
  function centerFreeMapOnRobot() {
    const posePoint = pointFromPose(state.pose);
    if (!posePoint) return;
    state.mapFreeView.center = { x: posePoint.x, y: posePoint.y };
    if (state.map2dCameraMode === 'free' && window.__nuwaxMapFreeze) window.__nuwaxMapFreezeCenter = { ...state.mapFreeView.center };
    requestMapDraw(false);
  }
  function syncCanvasSize() {
    const rect = canvas.getBoundingClientRect();
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    const width = Math.max(1, Math.round(rect.width * ratio)), height = Math.max(1, Math.round(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  }

  /* ---------- 地图会话与显示判定 ---------- */
  function activeMapName() { return state.latestGeometryState?.map_name || state.mapGrid?.map_id || null; }
  function binding() {
    return { map_name: state.mapGrid?.map_id || activeMapName(), frame_id: state.mapGrid?.frame_id || null,
      map_revision: state.mapGrid ? state.lastMapEnvelope?.revision : null, available: Boolean(state.mapGrid) };
  }
  function navigationFrameId() { return normalizeFrameId((state.pose && state.pose.frame_id) || (state.mapGrid && state.mapGrid.frame_id), 'map'); }
  function currentNavigationBodyPose(frameId) {
    const envelope = state.lastLocalizationEnvelope, pose = state.latestPose, position = pose && pose.position;
    if (!envelope || statusOf(envelope) !== 'available' || envelope.stale === true || !position) return null;
    if (![position.x, position.y, position.z].every(value => typeof value === 'number' && Number.isFinite(value))) return null;
    const poseFrame = String(pose.frame_id || envelope.frame_id || '').trim().replace(/^\/+/, '');
    if (!poseFrame || poseFrame !== String(frameId || '').trim().replace(/^\/+/, '')) return null;
    // 只读当前真实定位帧；显示缓存重绘不能刷新定位的采样时间和接收时间。
    const source = pose.timestamp;
    const sourceMs = typeof source === 'number' ? source * (source < 1e12 ? 1000 : 1) : Date.parse(source);
    const receivedMs = Date.parse(envelope.__receivedAt), now = Date.now();
    if (![sourceMs, receivedMs].every(value => Number.isFinite(value) && now - value >= -1000 && now - value <= 2000)) return null;
    return { x: position.x, y: position.y, z: position.z, frame_id: poseFrame };
  }
  function resolveNavigationHeight(input, frameId) {
    if (input && input.validity && input.validity.badInput) throw new Error('目标机体高度无效，请填写有限数值（米）。');
    const value = input ? String(input.value || '').trim() : '';
    if (value) {
      const height = Number(value);
      if (!Number.isFinite(height)) throw new Error('目标机体高度无效，请填写有限数值（米）。');
      return height;
    }
    const current = currentNavigationBodyPose(frameId);
    if (!current) throw new Error('当前机体高度不可用或已过期，请填写目标在地图中的机体高度（米）。');
    return current.z;
  }
  const normalizeStatusValue = value => String((value && typeof value === 'object' && 'value' in value ? value.value : value) || '').trim().toLowerCase().replace(/^(task|navigation|exploration)\./, '');
  const isActiveSpatialStatus = value => ACTIVE_SPATIAL_STATUSES.has(normalizeStatusValue(value));
  const isTerminalSpatialStatus = value => ['succeeded', 'failed', 'cancelled', 'timeout', 'idle'].includes(normalizeStatusValue(value));
  const getNestedState = (payload, key) => (!payload ? {} : payload[key] && typeof payload[key] === 'object' ? payload[key] : payload);
  function activeSpatialContextReason() {
    for (const task of store.get().tasks.active) {
      const capability = String(task.capability || '').trim();
      if (!capability || isTerminalSpatialStatus(task.state)) continue;
      if (ACTIVE_SPATIAL_CAPABILITIES.has(capability) || capability.startsWith('navigate_') || capability.includes('explore')) return `task:${capability}`;
    }
    const navState = getNestedState(state.latestFrames['navigation.state']?.payload, 'navigation_state');
    if (isActiveSpatialStatus(navState.status) || (navState.current_goal_id && !isTerminalSpatialStatus(navState.status))) return 'navigation';
    const explorationState = getNestedState(state.latestFrames['exploration.state']?.payload, 'exploration_state');
    if (isActiveSpatialStatus(explorationState.status) || (explorationState.current_request_id && !isTerminalSpatialStatus(explorationState.status))) return 'exploration';
    const envelope = state.lastLocalizationEnvelope;
    if (envelope && statusOf(envelope) === 'available' && pointFromPose((envelope.data || {}).pose || envelope.data)) return 'localization_pose_stream';
    return null;
  }
  function isMapClearedEnvelope(envelope, diagnostics, status) {
    const reason = diagnostics && (diagnostics.event_type || diagnostics.suppressed_live_map_reason);
    return status === 'cleared' || reason === 'catalog_cleared' || reason === 'workspace_cleared';
  }
  function shouldDisplayMapFrame(grid, status, cleared) {
    if (cleared) return { display: false, status: 'cleared', message: '地图会话已结束。' };
    return grid && grid.data && status === 'available' ? { display: true } : { display: false, status: status || 'missing', message: '等待地图栅格。' };
  }
  function clearLocalizationDisplay() {
    state.pose = null;
    state.displayedPoseMetrics = {};
    state.trajectory = [];
    emit();
  }
  function clearMapOverlayState() {
    state.frontiers = [];
    state.latestFrontierStream = null;
    state.goalPose = null;
    state.navigationPath = [];
    state.navigationLocalPath = [];
    state.navigationDisplay = null;
  }
  function contextChanged() {
    regions?.contextChanged();
    graph?.contextChanged();
    // 物品标记只画当前地图的；地图变化时重新发布给三维图层（同一快照与地图时不重复发布）。
    items?.publish();
    window.dispatchEvent(new CustomEvent('map:context-changed', { detail: binding() }));
  }
  function clearMapDisplay(message) {
    state.mapGrid = null;
    contextChanged();
    state.appliedMapIdentity = null;
    state.mapGridContentKey = '';
    state.mapBaseCache = null;
    state.mapFreeView = { center: null, zoom: 1, fitSpan: null };
    if (!activeSpatialContextReason()) clearMapOverlayState();
    state.emptyMessage = message || '等待地图数据。';
    requestMapDraw(true);
    reevaluateCachedLocalizationDisplay();
    emit();
  }
  function updateMap(envelope) {
    state.lastMapEnvelope = envelope;
    const data = envelope.data || {};
    const grid = data.occupancy_grid;
    const status = statusOf(envelope);
    if (!grid || status !== 'available') {
      state.mapGrid = null;
      state.appliedMapIdentity = null;
      clearMapDisplay('等待地图栅格。');
      return;
    }
    const nextIdentity = JSON.stringify([data.map_name, grid.frame_id]);
    const mapChanged = state.appliedMapIdentity && state.appliedMapIdentity.key !== nextIdentity;
    const nextContentKey = JSON.stringify([grid.map_id, grid.frame_id, envelope.cursor]);
    const baseDirty = state.mapGridContentKey !== nextContentKey;
    state.mapGrid = grid;
    state.mapGridContentKey = nextContentKey;
    state.mapStream = { diagnostics: {}, timestamp: envelope.source_timestamp };
    state.appliedMapIdentity = { key: nextIdentity, cleared: false };
    if (mapChanged) {
      resetFreeMapView();
      state.pc3dCameraMode = 'follow';
    }
    state.emptyMessage = '';
    metrics.record('map2d', videoSourceTime(envelope.source_timestamp), envelope.cursor, data.map_name);
    requestMapDraw(baseDirty);
    contextChanged();
    emit();
  }
  function updateMapHeartbeat(envelope) {
    const cursor = normalizedCursor(envelope.cursor);
    const mapCursor = normalizedCursor(state.lastMapEnvelope?.cursor);
    const previousCursor = state.heartbeatEpoch === state.streamEpoch ? normalizedCursor(state.lastMapHeartbeatEnvelope?.cursor) : null;
    // 主题分别合并后，旧清空心跳可能排在新地图之后执行，必须按原总线游标核对。
    if (cursor === null || (mapCursor !== null && compareCursor(cursor, mapCursor) < 0) || (previousCursor !== null && compareCursor(cursor, previousCursor) <= 0)) return false;
    state.heartbeatEpoch = state.streamEpoch;
    state.lastMapHeartbeatEnvelope = envelope;
    const status = statusOf(envelope);
    const diagnostics = { ...((state.mapStream && state.mapStream.diagnostics) || {}), ...(envelope.metadata || {}), ...(envelope.data || {}) };
    const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
    const grid = cleared ? null : state.mapGrid;
    state.mapStream = { diagnostics, revision: envelope.revision || diagnostics.content_revision, sequence: envelope.sequence || diagnostics.update_sequence, timestamp: envelope.source_timestamp, cleared };
    const decision = shouldDisplayMapFrame(grid, status, cleared);
    if (!decision.display) { clearMapDisplay(decision.message); return true; }
    state.emptyMessage = grid && (Array.isArray(grid.data) || grid.data instanceof Int8Array) ? '' : '地图源在线，等待内容更新帧。';
    emit();
    return true;
  }
  function reevaluateCachedMapDisplay() {
    if (!state.lastMapEnvelope) { reevaluateCachedLocalizationDisplay(); return; }
    const envelope = state.lastMapEnvelope;
    const data = envelope.data || {};
    const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
    const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
    const status = statusOf(envelope);
    const decision = shouldDisplayMapFrame(grid, status, isMapClearedEnvelope(envelope, diagnostics, status));
    if (decision.display || state.mapGrid) updateMap(envelope);
    reevaluateCachedLocalizationDisplay();
  }
  function reevaluateCachedLocalizationDisplay() {
    if (state.lastLocalizationEnvelope) applyLocalizationDisplay(state.lastLocalizationEnvelope, { reevaluateMap: false });
  }
  function poseStreamAvailable() {
    const envelope = state.lastLocalizationEnvelope;
    return Boolean(envelope && statusOf(envelope) === 'available' && pointFromPose((envelope.data || {}).pose || envelope.data));
  }
  function applyLocalizationDisplay(envelope, options) {
    const reevaluateMap = !options || options.reevaluateMap !== false;
    const wasAvailable = poseStreamAvailable();
    state.lastLocalizationEnvelope = envelope;
    const data = envelope.data || {};
    const incomingPose = data.pose || (pointFromPose(data) ? data : null);
    if (pointFromPose(incomingPose)) state.latestPose = incomingPose;
    else if (envelope.type === 'localization.pose') state.latestPose = null;
    if (incomingPose && envelope.metadata?.scan_received_unix_ns) {
      const key = JSON.stringify([state.appliedMapIdentity?.key, incomingPose.frame_id, incomingPose.timestamp]);
      state.poseReceipts.set(key, envelope.metadata.scan_received_unix_ns);
      if (state.poseReceipts.size > 256) state.poseReceipts.delete(state.poseReceipts.keys().next().value);
    }
    if (!(state.latestPose && envelope.status === 'available')) {
      clearLocalizationDisplay();
      requestMapDraw(false);
      return;
    }
    state.pose = incomingPose;
    state.trajectory = Array.isArray(data.trajectory) ? data.trajectory : [];
    // 持续位姿只移动覆盖层；定位可用性改变时才重新评估缓存地图。
    if (reevaluateMap && wasAvailable !== poseStreamAvailable()) reevaluateCachedMapDisplay();
    else requestMapDraw(false);
  }
  function hideMapPose() {
    state.lastLocalizationEnvelope = null;
    state.latestPose = null;
    clearLocalizationDisplay();
    requestMapDraw(false);
    window.dispatchEvent(new CustomEvent('stream:localization.pose', { detail: null }));
  }

  /* ---------- 位姿延迟（采集到画出） ---------- */
  function displayedMapPoseAllowed(kind, pose) {
    if (!pose || pose !== state.pose || state.mapStreamReady !== true || state.lastLocalizationEnvelope?.status !== 'available') return false;
    if (kind !== (state.viewMode === '3d' ? 'map3d' : 'map2d')) return false;
    if (state.lastLocalizationEnvelope?.stale === true || document.hidden || !visible()) return false;
    return true;
  }
  function recordDisplayedMapPose(kind, pose, displayedAt = Date.now()) {
    if (!displayedMapPoseAllowed(kind, pose)) return;
    const mapKey = state.appliedMapIdentity?.key || pose.frame_id;
    const previous = state.displayedPoseMetrics[kind];
    if (previous && previous.mapKey === mapKey && previous.pose?.timestamp === pose.timestamp && previous.pose?.frame_id === pose.frame_id) { previous.pose = pose; return; }
    // 总线信封时间是发布时间；这里只使用实际显示的位姿自身源时间。
    const sourceTime = typeof pose.timestamp === 'string' ? Date.parse(pose.timestamp) : NaN;
    const offset = store.get().clockOffsetMs;
    const age = displayedAt - sourceTime + (offset || 0);
    state.displayedPoseMetrics[kind] = { pose, mapKey, calibrated: offset != null, ageMs: Number.isFinite(sourceTime) && Number.isFinite(age) && age >= 0 ? age : null };
  }
  function poseLatency(kind) {
    const metric = state.displayedPoseMetrics[kind];
    if (!metric || !displayedMapPoseAllowed(kind, metric.pose) || metric.mapKey !== state.appliedMapIdentity?.key || metric.ageMs === null) return null;
    return { ms: metric.ageMs, calibrated: metric.calibrated };
  }

  /* ---------- 导航与探索叠加 ---------- */
  const firstArrayValue = (...values) => values.find(value => Array.isArray(value) && value.length) || [];
  function pathPointFromValue(value) {
    if (!value) return null;
    if (Array.isArray(value) && value.length >= 2) return { x: Number(value[0]), y: Number(value[1]), z: Number(value[2] || 0) };
    for (const key of ['pose', 'target_pose']) if (value[key]) { const point = pointFromPose(value[key]); if (point) return point; }
    if (value.position) { const point = pointFromPose(value); if (point) return point; }
    if (value.x !== undefined && value.y !== undefined) return { x: Number(value.x), y: Number(value.y), z: Number(value.z || 0) };
    return pointFromFrontier(value);
  }
  function pointFromFrontier(frontier) {
    const geometry = frontier && frontier.geometry ? frontier.geometry : frontier;
    if (!geometry) return null;
    if (Array.isArray(geometry.coordinates) && geometry.coordinates.length >= 2) return { x: Number(geometry.coordinates[0]), y: Number(geometry.coordinates[1]), z: Number(geometry.coordinates[2] || 0) };
    if (geometry.position) { const point = pointFromPose(geometry); if (point) return point; }
    if (geometry.pose) { const point = pointFromPose(geometry.pose); if (point) return point; return pointFromFrontier(geometry.pose); }
    if (geometry.centroid) return pointFromFrontier(geometry.centroid);
    if (geometry.x !== undefined && geometry.y !== undefined) return { x: Number(geometry.x), y: Number(geometry.y), z: Number(geometry.z || 0) };
    return null;
  }
  const navigationContextIsPublic = context => Boolean(context && !context.exploration_child);
  function goalPoseFromValue(value) {
    if (!value) return null;
    const pose = value.target_pose || value.goal_pose || value.pose || value;
    const point = pointFromPose(pose) || pathPointFromValue(value);
    if (!point) return null;
    if (value.require_goal_orientation === false) return { position: point };
    if (pose.orientation) return pose;
    const yaw = firstFiniteNumber(pose.yaw, pose.yaw_rad, value.yaw, value.yaw_rad);
    if (yaw === null) return { position: point };
    return { position: point, orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) } };
  }
  function explorationStateMetadata(context) {
    if (!context) return {};
    const stateObj = context.exploration_state || context;
    return (stateObj && stateObj.metadata) || context.metadata || {};
  }
  function extractNavigationPath(context, explorationContext) {
    const metadata = context && context.metadata ? context.metadata : {};
    const stateMetadata = (context && context.navigation_state && context.navigation_state.metadata) || {};
    if (Array.isArray(stateMetadata.planned_path)) return stateMetadata.planned_path.map(pathPointFromValue).filter(point => point && [point.x, point.y, point.z].every(Number.isFinite));
    const explorationMetadata = explorationStateMetadata(explorationContext);
    return firstArrayValue(context && context.planned_path, context && context.current_path, context && context.remaining_path, context && context.nav_path, context && context.waypoints,
      metadata.planned_path, metadata.current_path, metadata.remaining_path, metadata.nav_path, metadata.waypoints,
      stateMetadata.planned_path, stateMetadata.current_path, stateMetadata.remaining_path, stateMetadata.nav_path, stateMetadata.waypoints,
      explorationContext && explorationContext.planned_path, explorationMetadata.planned_path, explorationMetadata.nav_path).map(pathPointFromValue).filter(Boolean);
  }
  function extractNavigationGoal(context, explorationContext) {
    const metadata = context && context.metadata ? context.metadata : {};
    const navigationState = context && context.navigation_state ? context.navigation_state : {};
    const navigationMetadata = navigationState.metadata || {};
    if (navigationContextIsPublic(context) || navigationState.current_goal_id || navigationMetadata.observed_goal_id) {
      return goalPoseFromValue(context.current_goal || navigationState.current_goal || metadata.current_goal || navigationMetadata.current_goal);
    }
    const explorationMetadata = explorationStateMetadata(explorationContext);
    for (const candidate of [context && context.current_goal, context && context.goal_pose, navigationState.current_goal, metadata.current_goal, navigationMetadata.current_goal,
      explorationContext && explorationContext.current_goal, explorationContext && explorationContext.goal_pose, explorationContext && explorationContext.selected_frontier,
      explorationContext && explorationContext.current_frontier, explorationMetadata.current_goal, explorationMetadata.selected_frontier, explorationMetadata.current_frontier]) {
      const pose = goalPoseFromValue(candidate);
      if (pose) return pose;
    }
    return null;
  }
  function resolveGeometrySessionMode(data) {
    const candidates = [data && data.geometry_session_mode, data?.exploration_context?.metadata?.geometry_session_mode,
      data?.exploration_context?.exploration_state?.metadata?.geometry_session_mode];
    const direct = candidates.find(Boolean);
    return direct ? String(direct).trim().toLowerCase() : '';
  }
  function shouldRetainFrontierStreamCache(fr, data) {
    if (!fr || fr.frontier_cleared === true) return false;
    const sessionMode = resolveGeometrySessionMode(data) || String(fr.geometry_session_mode || '').trim().toLowerCase();
    if (sessionMode && sessionMode !== 'explore') return false;
    return fr.exploration_terminal !== true;
  }
  function mergeLatestFrontierStream(data) {
    const fr = state.latestFrontierStream;
    if (!fr) return data;
    const merged = { ...data };
    if (!shouldRetainFrontierStreamCache(fr, merged)) {
      if (!merged.frontiers || !merged.frontiers.length) merged.frontiers = [];
      if (merged.frontier_count === null || merged.frontier_count === undefined) merged.frontier_count = 0;
      if (merged.exploration_terminal === undefined || merged.exploration_terminal === null) merged.exploration_terminal = fr.exploration_terminal === true || fr.frontier_cleared === true;
      return merged;
    }
    if (!merged.explorer_status_text && fr.status_text) merged.explorer_status_text = fr.status_text;
    if ((merged.frontier_count === null || merged.frontier_count === undefined) && fr.frontier_count != null) merged.frontier_count = fr.frontier_count;
    if ((!merged.frontiers || !merged.frontiers.length) && Array.isArray(fr.latest_frontiers) && fr.latest_frontiers.length) merged.frontiers = fr.latest_frontiers;
    if ((!merged.planned_path || !merged.planned_path.length) && Array.isArray(fr.planned_path) && fr.planned_path.length) merged.planned_path = fr.planned_path;
    if (!merged.current_goal && fr.current_goal) merged.current_goal = fr.current_goal;
    if (!merged.current_frontier && fr.current_frontier) merged.current_frontier = fr.current_frontier;
    if (!merged.selected_frontier && fr.selected_frontier) merged.selected_frontier = fr.selected_frontier;
    if (merged.exploration_terminal === undefined || merged.exploration_terminal === null) merged.exploration_terminal = fr.exploration_terminal;
    return merged;
  }
  function mergeNavigationDisplay(previous, context, goal, path, clear = false) {
    if (clear) return null;
    const nav = context?.navigation_state || {};
    const metadata = nav.metadata || {};
    const goalId = String(nav.current_goal_id || metadata.observed_goal_id || context?.current_goal?.goal_id || '');
    const pathRevision = Number(metadata.path_revision || 0);
    const status = String(nav.status || '').toLowerCase();
    const sameGoal = previous && previous.goalId === goalId;
    if (['succeeded', 'failed', 'cancelled', 'timeout'].includes(status)) return { goalId, pathRevision, terminal: true, goal: null, path: [], localPath: [], mode: 'none',
      isPublic: Number(metadata.owner_kind) === 1 || (sameGoal && previous.isPublic === true) };
    if (sameGoal && pathRevision < previous.pathRevision) return previous;
    const rawGoal = context?.current_goal;
    return {
      goalId, pathRevision, terminal: false,
      isPublic: Number(metadata.owner_kind) === 1 || (sameGoal && previous.isPublic === true && !Number(metadata.owner_kind)),
      goal, goalData: rawGoal?.goal_id === goalId ? rawGoal : (sameGoal ? previous.goalData : null), path,
      localPath: status === 'running' && Array.isArray(metadata.local_path) ? metadata.local_path.map(pathPointFromValue).filter(point => point && [point.x, point.y, point.z].every(Number.isFinite)) : [],
      mode: path.length ? (status === 'running' ? 'executing' : 'retained') : 'none',
    };
  }
  function isCanonicalExplorationFrame(frame) {
    const metadata = frame?.payload?.exploration_state?.metadata;
    if (!metadata) return false;
    return String(metadata.state_source || '').trim().toLowerCase() === 'geometry_runtime_grpc'
      || String(metadata.exploration_mode || '').trim().toLowerCase() === 'nuwax_geometry_runtime';
  }
  function updateNavigation(envelope) {
    const data = mergeLatestFrontierStream(envelope.data || {});
    const goalMapName = data.navigation_context?.current_goal?.map_name;
    const mapName = activeMapName();
    if (goalMapName && mapName && goalMapName !== mapName) return;
    const frontierCache = state.latestFrontierStream;
    const explorationFrame = state.latestFrames['exploration.state'];
    const clearSequence = normalizedCursor(frontierCache && frontierCache.sequence);
    const explorationSequence = normalizedCursor(explorationFrame && (explorationFrame.sequence ?? explorationFrame.cursor));
    const explorationIsNewerThanClear = clearSequence !== null && explorationSequence !== null && compareCursor(explorationSequence, clearSequence) > 0;
    const explorationContext = frontierCache && frontierCache.frontier_cleared === true && !explorationIsNewerThanClear ? null : data.exploration_context;
    const explorationState = explorationContext && explorationContext.exploration_state;
    const explorationMetadata = explorationState && explorationState.metadata ? explorationState.metadata : {};
    const explorationTerminal = explorationMetadata.exploration_terminal ?? data.exploration_terminal
      ?? (explorationState && ['succeeded', 'failed', 'cancelled'].includes(String(explorationState.status || '')));
    state.frontiers = Array.isArray(data.frontiers) ? data.frontiers : [];
    const isPublic = navigationContextIsPublic(data.navigation_context);
    const navState = data.navigation_context?.navigation_state;
    state.goalPose = extractNavigationGoal(data.navigation_context, explorationContext)
      || (!isPublic && !navState?.current_goal_id && !navState?.metadata?.observed_goal_id
        ? (goalPoseFromValue(data.current_goal) || goalPoseFromValue(data.current_frontier) || goalPoseFromValue(data.selected_frontier)) : null);
    state.navigationPath = extractNavigationPath(data.navigation_context, explorationContext);
    if (!isPublic && !state.navigationPath.length && !Array.isArray(navState?.metadata?.planned_path) && Array.isArray(data.planned_path)) {
      state.navigationPath = data.planned_path.map(pathPointFromValue).filter(Boolean);
    }
    const canonical = String(explorationMetadata.state_source || '').toLowerCase() === 'geometry_runtime_grpc'
      || String(explorationMetadata.exploration_mode || '').toLowerCase() === 'nuwax_geometry_runtime';
    const explorationStatus = String(explorationState && explorationState.status || '').toLowerCase();
    const canonicalInactive = canonical && (explorationTerminal || explorationStatus === 'idle'
      || (explorationMetadata.geometry_session_mode && String(explorationMetadata.geometry_session_mode).toLowerCase() !== 'explore'));
    const clearNavigation = !isPublic && canonicalInactive;
    if (clearNavigation) { state.goalPose = null; state.navigationPath = []; state.navigationLocalPath = []; }
    state.navigationDisplay = mergeNavigationDisplay(state.navigationDisplay, data.navigation_context, state.goalPose, state.navigationPath, clearNavigation);
    if (state.navigationDisplay) { state.goalPose = state.navigationDisplay.goal; state.navigationPath = state.navigationDisplay.path; }
    state.navigationLocalPath = state.navigationDisplay?.localPath || [];
    reevaluateCachedMapDisplay();
    requestMapDraw(false);
    emit();
  }
  function updateFrontierFrame(frame) {
    const payload = frame.payload || {};
    const frameMetadata = frame.metadata || {};
    const cleared = payload.frontier_cleared === true || frameMetadata.frontier_cleared === true;
    const sessionMode = String(payload.geometry_session_mode || '').trim().toLowerCase();
    const context = { navigation_context: state.latestFrames['navigation.state']?.payload, exploration_context: state.latestFrames['exploration.state']?.payload };
    if (cleared || (sessionMode && sessionMode !== 'explore')) {
      state.latestFrontierStream = { status_text: payload.status_text, frontier_count: 0, latest_frontiers: [], planned_path: [], current_goal: null, current_frontier: null,
        selected_frontier: null, exploration_terminal: true, frontier_cleared: true, geometry_session_mode: sessionMode || 'idle', sequence: frame.sequence, received_at: frame.timestamp };
      state.frontiers = [];
      updateNavigation(envelopeFromFrame(frame, { ...context, frontiers: [], frontier_count: 0, explorer_status_text: payload.status_text, exploration_terminal: true,
        planned_path: [], current_goal: null, current_frontier: null, selected_frontier: null, geometry_session_mode: sessionMode || 'idle' }));
      return;
    }
    const frontiers = Array.isArray(payload.latest_frontiers) ? payload.latest_frontiers : [];
    state.latestFrontierStream = { status_text: payload.status_text, frontier_count: payload.frontier_count, latest_frontiers: frontiers,
      planned_path: Array.isArray(payload.planned_path) ? payload.planned_path : [], current_goal: payload.current_goal || null, current_frontier: payload.current_frontier || null,
      selected_frontier: payload.selected_frontier || null, exploration_terminal: payload.exploration_terminal, geometry_session_mode: sessionMode || 'explore',
      sequence: frame.sequence, received_at: frame.timestamp };
    state.frontiers = frontiers;
    updateNavigation(envelopeFromFrame(frame, { ...context, frontiers, frontier_count: payload.frontier_count ?? frontiers.length, explorer_status_text: payload.status_text,
      exploration_terminal: payload.exploration_terminal, planned_path: payload.planned_path, current_goal: payload.current_goal, current_frontier: payload.current_frontier,
      selected_frontier: payload.selected_frontier }));
  }
  function resetExplorationOverlay(reason, preserveNavigation = true) {
    state.latestFrontierStream = null;
    state.frontiers = [];
    if (!preserveNavigation) {
      state.navigationPath = []; state.navigationLocalPath = []; state.goalPose = null; state.navigationDisplay = null; state.highlightTarget = null;
    } else if (state.navigationDisplay && !state.navigationDisplay.terminal) {
      state.navigationDisplay = { ...state.navigationDisplay, localPath: [], mode: 'retained' };
    }
    state.navigationLocalPath = [];
    for (const topic of ['exploration.state', 'exploration.frontier', 'navigation.state']) delete state.latestFrames[topic];
    requestMapDraw(false);
    window.dispatchEvent(new CustomEvent('stream:exploration.overlay-reset', { detail: { reason, planned_path: state.navigationPath, local_path: state.navigationLocalPath,
      current_goal: state.goalPose, navigation_path_state: state.navigationDisplay?.mode || (state.navigationPath.length ? 'retained' : 'none') } }));
  }
  const resetPointCloudConsumer = reason => window.dispatchEvent(new CustomEvent('stream:pointcloud_map.reset', { detail: { reason } }));

  /* ---------- 三维明细（与原控制台 pointcloud3dStreamDetail 相同） ---------- */
  function explorationOverlayDetail() {
    const frame = state.latestFrames['exploration.state'];
    const exploration = frame?.payload?.exploration_state;
    const metadata = exploration?.metadata || {};
    return { canonical_exploration: isCanonicalExplorationFrame(frame), exploration_identity: null, exploration_terminal: metadata.exploration_terminal === true,
      exploration_status: String(exploration && exploration.status || '').toLowerCase(), geometry_session_mode: String(metadata.geometry_session_mode || '').toLowerCase(),
      frontier_count: exploration && exploration.frontier_count };
  }
  function overlayDetail() {
    return { planned_path: state.navigationPath || [], local_path: state.navigationLocalPath || [],
      navigation_path_state: state.navigationDisplay?.mode || (state.navigationPath.length ? 'retained' : 'none'), current_goal: state.goalPose || null,
      navigation_is_public: navigationContextIsPublic(state.latestFrames['navigation.state']?.payload), navigation_exploration_validated: false, ...explorationOverlayDetail() };
  }
  function pointcloud3dDetail(frame) {
    if (frame.topic === 'localization.pose') return state.latestPose || null;
    if (frame.topic === 'localization.trajectory') return { trajectory: state.trajectory || [] };
    if (frame.topic === 'navigation.state' || frame.topic === 'exploration.state') return overlayDetail();
    if (frame.topic === 'exploration.frontier') return { frontiers: state.frontiers || [], ...overlayDetail() };
    return frame.payload || {};
  }
  function envelopeFromFrame(frame, payloadOverride) {
    return { ok: frame.status !== 'error' && frame.status !== 'missing', type: frame.topic, source: frame.source,
      frame_id: frame.frame_id || (frame.payload && frame.payload.frame_id), source_timestamp: frame.timestamp || (frame.payload && frame.payload.timestamp),
      sequence: frame.sequence ?? frame.cursor, cursor: frame.cursor, revision: frame.revision, status: frame.status || 'available', stale: frame.status === 'stale',
      data: payloadOverride === undefined ? frame.payload : payloadOverride, metadata: frame.metadata || {}, __receivedAt: new Date().toISOString() };
  }
  function syncPointcloud3dState() {
    if (PLANAR_PIPELINE) return;
    window.pointcloud3d?.setMapContext(state.lastMapEnvelope?.data);
    window.dispatchEvent(new CustomEvent('stream:localization.pose', { detail: state.latestPose || null }));
    window.dispatchEvent(new CustomEvent('stream:localization.trajectory', { detail: { trajectory: state.trajectory || [] } }));
    const navigation = state.latestFrames['navigation.state']?.payload;
    const exploration = state.latestFrames['exploration.state']?.payload;
    const overlays = { planned_path: state.navigationPath, local_path: [], current_goal: state.goalPose, navigation_is_public: navigationContextIsPublic(navigation),
      navigation_exploration_validated: navigation?.exploration_child === true };
    window.dispatchEvent(new CustomEvent('stream:exploration.frontier', { detail: { frontiers: [], ...overlays } }));
    window.dispatchEvent(new CustomEvent('stream:exploration.state', { detail: { ...exploration, ...overlays } }));
    window.dispatchEvent(new CustomEvent('stream:navigation.state', { detail: { ...navigation, ...overlays } }));
    items?.publish();
    window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-visibility', { detail: { visible: items ? items.visible : true } }));
    const meta = state.latestFrames['pointcloud_map.meta'];
    if (meta && meta.payload) window.dispatchEvent(new CustomEvent('stream:pointcloud_map.meta', { detail: meta.payload }));
  }
  function recordPointcloudDiagnostic(values) {
    // 诊断只写在三维容器的不可见数据属性上，供排查与验证脚本读取。
    for (const [key, value] of Object.entries(values)) {
      const text = String(value ?? '');
      if (container3d.dataset[key] !== text) container3d.dataset[key] = text;
    }
  }

  /* ---------- 实时帧分发 ---------- */
  function processFrame(frame) {
    if (frame.topic === 'geometry.state') {
      state.latestGeometryState = frame.payload;
      window.dispatchEvent(new CustomEvent('stream:geometry.state', { detail: frame.payload }));
      if (frame.payload.phase === 'idle') { updateMap({ ...envelopeFromFrame(frame, {}), status: 'cleared' }); hideMapPose(); }
      emit();
      return;
    }
    state.latestFrames[frame.topic] = frame;
    if (frame.topic === 'map.frame') {
      updateMap(envelopeFromFrame(frame, mapForCanvas(frame.payload)));
    } else if (frame.topic === 'map.heartbeat') {
      if (updateMapHeartbeat(envelopeFromFrame(frame)) === false) return;
    } else if (frame.topic === 'localization.pose') {
      applyLocalizationDisplay(envelopeFromFrame(frame, { pose: poseForCanvas(frame.payload), trajectory: state.trajectory }), { reevaluateMap: true });
    } else if (frame.topic === 'localization.trajectory') {
      const trajectory = frame.payload && Array.isArray(frame.payload.trajectory) ? frame.payload.trajectory : [];
      state.trajectory = trajectory;
      const localization = state.lastLocalizationEnvelope;
      // 轨迹连接可能晚于位姿到达，不能用旧轨迹的状态和时间覆盖新定位。
      if (localization) applyLocalizationDisplay({ ...localization, data: { ...localization.data, trajectory } }, { reevaluateMap: false });
      else requestMapDraw(false);
    } else if (frame.topic === 'navigation.state') {
      const nav = frame.payload;
      const terminal = TERMINAL_NAV.includes(nav?.status);
      const target = terminal ? null : nav?.target;
      // 后端目标总带四元数；本页提交时未定朝向的目标按不带方向显示。
      const unoriented = target && window.__nuwaxNavigationModule?.goalOrientationUnset(nav.goal_id);
      state.goalPose = target ? { position: { x: target.x, y: target.y, z: target.z }, ...(unoriented ? {} : { orientation: { x: target.qx, y: target.qy, z: target.qz, w: target.qw } }) } : null;
      state.navigationPath = (terminal ? [] : nav?.planned_path || []).map(([x, y, z]) => ({ x, y, z }));
      state.navigationLocalPath = [];
      state.navigationDisplay = { mode: state.navigationPath.length ? 'live' : 'none', isPublic: navigationContextIsPublic(nav), status: nav?.status };
      // 高亮环按平面点 {x, y} 读取，与物品导航写入的格式一致。
      state.highlightTarget = target ? { target_pose: { x: target.x, y: target.y, z: target.z } } : null;
      window.dispatchEvent(new CustomEvent('stream:navigation.state', { detail: nav }));
      requestMapDraw(false);
      emit();
    } else if (frame.topic === 'exploration.state') {
      state.frontiers = [];
      window.dispatchEvent(new CustomEvent('stream:exploration.state', { detail: frame.payload }));
      requestMapDraw(false);
    } else if (frame.topic === 'exploration.frontier') {
      updateFrontierFrame(frame);
    } else if (frame.topic === 'pointcloud_map.meta') {
      emit();
    } else if (frame.topic === 'settings.display') {
      window.__NUWAX_POINTCLOUD3D_CONFIG__ = frame.payload;
      window.dispatchEvent(new CustomEvent('settings:display', { detail: frame.payload }));
      return;
    } else if (frame.topic === 'navigation.forbidden_zones') {
      // 只转给三维图层与禁行区面板。
    } else return;
    if (!PLANAR_PIPELINE && POINTCLOUD3D_STREAM_TOPICS.has(frame.topic)) {
      window.dispatchEvent(new CustomEvent(`stream:${frame.topic}`, { detail: pointcloud3dDetail(frame) }));
    }
  }
  for (const topic of ['geometry.state', 'map.frame', 'map.heartbeat', 'localization.trajectory', 'navigation.state', 'navigation.forbidden_zones',
    'exploration.state', 'exploration.frontier', 'pointcloud_map.meta', 'settings.display']) stream.on(topic, processFrame);
  // 选点导航得知自己提交的目标号时，该目标的状态可能已先到达；按提交时的朝向选择重画一次。
  window.addEventListener('navigation:submitted-goal', event => {
    const frame = state.latestFrames['navigation.state'];
    if (frame?.payload?.goal_id === event.detail.goal_id) processFrame(frame);
  });

  // 主连接：二进制地图帧当作普通帧接收；点云快照二进制转给三维模块。
  stream.onBinary(buffer => {
    const mapFrame = decodeMapBinaryFrame(buffer);
    if (mapFrame) { stream.receive(mapFrame); return; }
    const bytes = new Uint8Array(buffer);
    const magic = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0x00];
    if (bytes.byteLength < 13 || !magic.every((value, index) => bytes[index] === value) || bytes[8] !== 1) return;
    const headerLength = new DataView(buffer, 9, 4).getUint32(0, true);
    if (headerLength <= 0 || headerLength > 64 * 1024 || 13 + headerLength > bytes.byteLength) return;
    let header;
    try { header = JSON.parse(new TextDecoder('utf-8', { fatal: true }).decode(bytes.slice(13, 13 + headerLength))); } catch (_) { return; }
    if (!header || header.topic !== 'pointcloud_map.snapshot') return;
    window.dispatchEvent(new CustomEvent('stream:pointcloud_map.binary', { detail: { buffer, routingHeader: header } }));
  });
  let incrementalReady = false;
  window.addEventListener('stream:pointcloud_map.incremental-ready', () => {
    incrementalReady = true;
    // 能力握手成功后撤下旧全量通道；不重连任务、地图、导航连接。
    stream.drop(['pointcloud_map.snapshot']);
    stream.send({ type: 'pointcloud.detach' });
  });
  window.addEventListener('stream:pointcloud_map.cursor-accepted', event => stream.updateCursor(event.detail?.cursor));
  stream.onOpen(() => { state.streamEpoch += 1; if (incrementalReady) stream.send({ type: 'pointcloud.detach' }); });
  stream.onReplay(message => {
    state.mapStreamReady = true;
    if (message.gap_reason === 'cursor_ahead_of_stream') {
      state.appliedMapIdentity = null;
      const mapFrame = stream.latest('map.frame');
      const mapCursor = normalizedCursor(mapFrame?.cursor), latestCursor = normalizedCursor(message.latest_cursor);
      if (mapCursor !== null && latestCursor !== null && compareCursor(mapCursor, latestCursor) <= 0) stream.receive(mapFrame);
    }
  });
  stream.onGap(() => { resetPointCloudConsumer('gap'); resetExplorationOverlay('gap'); });
  stream.onClose(() => {
    resetPointCloudConsumer('disconnect');
    resetExplorationOverlay('disconnect');
    state.mapStreamReady = false;
    hideMapPose();
  });
  // 位姿独立连接：与地图帧分开排队；缺口时隐藏位姿并重连，断开时隐藏位姿。
  const poseStream = createStream({ topics: ['localization.pose'], params: { map_binary_version: '4' }, immediate: ['localization.pose'], closeOnGap: true });
  poseStream.on('localization.pose', processFrame);
  poseStream.onGap(hideMapPose);
  poseStream.onClose(hideMapPose);
  poseStream.connect();

  /* ---------- 二维绘制 ---------- */
  function requestMapDraw(baseDirty) {
    if (baseDirty) state.mapBaseCache = null;
    if (state.viewMode === '3d' || state.viewMode === 'graph' || state.mapDrawFrame) return;
    state.mapDrawFrame = requestAnimationFrame(() => {
      state.mapDrawFrame = null;
      if (state.viewMode !== '3d' && state.viewMode !== 'graph' && visible()) drawMap();
    });
  }
  function ensureMapBaseCanvas(context, grid) {
    const width = Number(grid.width), height = Number(grid.height);
    const theme = document.documentElement.dataset.theme === 'dark' ? 'dark' : 'light';
    const cache = state.mapBaseCache;
    if (cache && cache.key === state.mapGridContentKey && cache.width === width && cache.height === height && cache.theme === theme) return cache.canvas;
    const colors = GRID_COLORS[theme];
    const image = context.createImageData(width, height);
    for (let row = 0; row < height; row += 1) {
      for (let col = 0; col < width; col += 1) {
        const value = Number(grid.data[row * width + col]);
        if (value < 0) continue;   // 未知栅格透明，露出舞台底色
        const color = value >= 65 ? colors.occupied : colors.free;
        const target = ((height - 1 - row) * width + col) * 4;
        image.data[target] = color[0]; image.data[target + 1] = color[1]; image.data[target + 2] = color[2]; image.data[target + 3] = 255;
      }
    }
    const offscreen = document.createElement('canvas');
    offscreen.width = width;
    offscreen.height = height;
    offscreen.getContext('2d').putImageData(image, 0, 0);
    state.mapBaseCache = { key: state.mapGridContentKey, width, height, theme, canvas: offscreen };
    return offscreen;
  }
  function drawMap() {
    syncCanvasSize();
    const context = canvas.getContext('2d');
    context.setTransform(1, 0, 0, 1, 0, 0);
    context.clearRect(0, 0, canvas.width, canvas.height);
    const grid = state.mapGrid;
    const anchor = cameraAnchor();
    const scale = cameraScale(anchor);
    if (!grid || !(Array.isArray(grid.data) || grid.data instanceof Int8Array) || !grid.width || !grid.height) { emit(); return; }
    const width = Number(grid.width), height = Number(grid.height);
    const offscreen = ensureMapBaseCanvas(context, grid);
    context.save();
    applyCameraTransform(context, anchor, scale);
    const origin = originPosition(grid), resolution = gridResolution(grid);
    context.translate(Number(origin.x || 0), Number(origin.y || 0));
    context.rotate(originYaw(grid));
    context.scale(resolution, -resolution);
    context.imageSmoothingEnabled = false;
    if (state.viewMode === 'semantic') context.globalAlpha = 0.2;
    context.drawImage(offscreen, 0, -height, width, height);
    context.restore();
    const project = point => mapToCanvas(point, anchor, scale);
    regions?.draw2d(context, project);
    graph?.draw2d(context, project);
    drawTrajectory(context, anchor, scale);
    drawNavigationPath(context, anchor, scale);
    const goalPoint = pointFromPose(state.goalPose);
    if (goalPoint) drawGoalMarker(context, anchor, scale, goalPoint, yawFromPose(state.goalPose));
    // 导航面板正在选的点：有朝向画箭头，没有朝向画圆点。
    const pick = window.__nuwaxNavigationModule?.getState().pendingPose;
    if (pick) drawGoalMarker(context, anchor, scale, pick, pick.yaw);
    // 地图卡“指定位置后重定位”指出的位置与朝向，画法同导航目标。
    if (state.seedPose) drawGoalMarker(context, anchor, scale, state.seedPose, state.seedPose.yaw);
    drawFrontiers(context, anchor, scale);
    items?.draw(context, project, canvas.width, canvas.height);
    drawRobot(context, anchor);
    // 禁行区覆盖层每帧同步视口。
    if (typeof window.__nuwaxMapDrawTick === 'function') {
      try { window.__nuwaxMapDrawTick(); } catch (error) { console.error(error); }
    }
    emit();
  }
  function setWorldLineStyle(context, scale, color, pixels) {
    context.strokeStyle = color;
    context.lineWidth = Math.max(1 / scale, pixels / scale);
    context.lineCap = 'round';
    context.lineJoin = 'round';
  }
  function drawWorldPoints(context, anchor, scale, points, color, pixels) {
    if (!points || points.length < 2) return;
    context.save();
    applyCameraTransform(context, anchor, scale);
    setWorldLineStyle(context, scale, color, pixels);
    context.beginPath();
    let started = false;
    for (const point of points) {
      if (!point) continue;
      if (!started) { context.moveTo(point.x, point.y); started = true; } else context.lineTo(point.x, point.y);
    }
    if (started) context.stroke();
    context.restore();
  }
  function drawFrontiers(context, anchor, scale) {
    if (!state.frontiers || !state.frontiers.length) return;
    context.save();
    applyCameraTransform(context, anchor, scale);
    context.fillStyle = COLORS.frontier;
    context.strokeStyle = COLORS.frontierStroke;
    context.lineWidth = Math.max(1 / scale, 1.4 / scale);
    for (const frontier of state.frontiers) {
      const point = pointFromFrontier(frontier);
      if (!point) continue;
      context.beginPath();
      context.arc(point.x, point.y, Math.max(0.08, 5 / scale), 0, Math.PI * 2);
      context.fill();
      context.stroke();
    }
    context.restore();
  }
  function drawTrajectory(context, anchor, scale) { drawWorldPoints(context, anchor, scale, (state.trajectory || []).map(pointFromPose).filter(Boolean), COLORS.trajectory, 2.5); }
  function drawNavigationPath(context, anchor, scale) {
    const retained = state.navigationDisplay?.mode !== 'executing';
    drawWorldPoints(context, anchor, scale, state.navigationPath, retained ? COLORS.retainedPath : COLORS.navPath, 2);
    drawWorldPoints(context, anchor, scale, state.navigationLocalPath, COLORS.localPath, 2);
  }
  function drawOrientedGoal(context, anchor, scale, point, yawRad, pixels) {
    if (!point || !Number.isFinite(Number(point.x)) || !Number.isFinite(Number(point.y))) return;
    context.save();
    applyCameraTransform(context, anchor, scale);
    context.translate(Number(point.x), Number(point.y));
    context.rotate(Number(yawRad) || 0);
    const shaft = Math.max(0.35, 14 / scale), head = Math.max(0.18, 7 / scale);
    context.strokeStyle = COLORS.goalStroke;
    context.fillStyle = COLORS.goal;
    context.lineWidth = Math.max(1 / scale, pixels / scale);
    context.lineCap = 'round';
    context.lineJoin = 'round';
    context.beginPath(); context.moveTo(0, 0); context.lineTo(shaft, 0); context.stroke();
    context.beginPath(); context.moveTo(shaft, 0); context.lineTo(shaft - head, head * 0.55); context.lineTo(shaft - head, -head * 0.55); context.closePath(); context.fill(); context.stroke();
    context.beginPath(); context.arc(0, 0, Math.max(0.08, 4 / scale), 0, Math.PI * 2); context.fill();
    context.restore();
  }
  function drawGoalMarker(context, anchor, scale, point, yaw) {
    if (Number.isFinite(yaw)) { drawOrientedGoal(context, anchor, scale, point, yaw, 2.4); return; }
    context.save();
    applyCameraTransform(context, anchor, scale);
    context.strokeStyle = COLORS.goalStroke;
    context.fillStyle = COLORS.goal;
    context.lineWidth = Math.max(1 / scale, 2 / scale);
    context.beginPath();
    context.arc(point.x, point.y, Math.max(0.12, 7 / scale), 0, Math.PI * 2);
    context.fill();
    context.stroke();
    context.restore();
  }
  function drawRobot(context, anchor) {
    const posePoint = pointFromPose(state.pose);
    if (!posePoint) return;
    context.save();
    context.setTransform(1, 0, 0, 1, 0, 0);
    if (anchor && anchor.locked === true) context.translate(canvas.width / 2, canvas.height / 2);
    else {
      const canvasPoint = mapToCanvas(posePoint);
      if (!canvasPoint) { context.restore(); return; }
      context.translate(canvasPoint.x, canvasPoint.y);
      // 北向地图：屏幕 Y 向上对应世界 +Y，三角形默认指屏幕上方，旋转 π/2 - yaw。
      context.rotate(Math.PI / 2 - (planarYawFromPose(state.pose) || 0));
    }
    context.fillStyle = COLORS.robot;
    context.strokeStyle = COLORS.robotStroke;
    context.lineWidth = 2;
    context.beginPath(); context.moveTo(0, -14); context.lineTo(-9, 9); context.lineTo(0, 5); context.lineTo(9, 9); context.closePath();
    context.fill();
    context.stroke();
    context.restore();
    recordDisplayedMapPose('map2d', state.pose);
  }
  document.addEventListener('neh:theme', () => requestMapDraw(true));
  // 三维只在监控页、3D 视图且标签页可见时运行；机器人模型与完整点云的轮询随之启停。
  document.addEventListener('visibilitychange', () => window.pointcloud3d?.setVisible(state.viewMode === '3d' && visible()));
  new ResizeObserver(() => requestMapDraw(false)).observe(canvas);

  /* ---------- 二维交互 ---------- */
  canvas.addEventListener('contextmenu', event => { if (state.map2dCameraMode === 'free') event.preventDefault(); });
  canvas.addEventListener('pointerdown', event => {
    // 禁行区绘制：事件交给覆盖层。导航选点期间单击拖动照常旋转平移，不做物品与区域点选；双击选点由导航模块处理。
    if (window.__nuwaxBoundaryActive) return;
    const picking = Boolean(window.__nuwaxNavigationActive);
    state.regionPointerDown = event.button === 0 && !picking ? { x: event.clientX, y: event.clientY } : null;
    if (event.button === 0 && !picking && items) {
      const pointer = canvasPointer(event);
      const anchor = cameraAnchor(), scale = cameraScale(anchor);
      const hit = items.hitTest(canvas.getContext('2d'), pointer.x, pointer.y, point => mapToCanvas(point, anchor, scale), canvas.width, canvas.height);
      if (hit) { state.regionPointerDown = null; event.preventDefault(); onPick(hit.type === 'cluster' ? { type: 'cluster', cluster: hit.cluster } : { type: 'item', instance: hit.instance }); return; }
    }
    if (state.map2dCameraMode !== 'free') return;
    const pointer = canvasPointer(event);
    if (event.button === 0) {
      // 与三维一致：左键拖拽旋转视角。
      event.preventDefault();
      state.mapRotateDrag = { pointerStart: pointer, yawStart: freeViewYaw() };
      canvas.setPointerCapture(event.pointerId);
      return;
    }
    if (event.button === 1 || event.button === 2) {
      // 中键或右键拖拽平移。
      event.preventDefault();
      state.mapPanDrag = { pointerStart: pointer, centerStart: getFreeMapViewCenter() };
      canvas.classList.add('is-dragging');
      canvas.setPointerCapture(event.pointerId);
    }
  });
  canvas.addEventListener('pointermove', event => {
    if (window.__nuwaxBoundaryActive) return;
    if (state.mapRotateDrag) {
      state.mapFreeView.yaw = state.mapRotateDrag.yawStart + (canvasPointer(event).x - state.mapRotateDrag.pointerStart.x) * 0.006;
      requestMapDraw(false);
      return;
    }
    if (state.mapPanDrag) {
      const pointer = canvasPointer(event);
      const scale = cameraScale(cameraAnchor());
      if (!Number.isFinite(scale) || scale <= 0) return;
      state.mapFreeView.center = { x: state.mapPanDrag.centerStart.x - (pointer.x - state.mapPanDrag.pointerStart.x) / scale,
        y: state.mapPanDrag.centerStart.y + (pointer.y - state.mapPanDrag.pointerStart.y) / scale };
      requestMapDraw(false);
    }
  });
  function finishDrag(event) {
    state.mapRotateDrag = null;
    if (state.mapPanDrag) { state.mapPanDrag = null; canvas.classList.remove('is-dragging'); }
    if (event.pointerId !== undefined && canvas.hasPointerCapture?.(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  }
  canvas.addEventListener('pointerup', event => {
    if (window.__nuwaxBoundaryActive) return;
    finishDrag(event);
    const start = state.regionPointerDown;
    state.regionPointerDown = null;
    if (start && Math.hypot(event.clientX - start.x, event.clientY - start.y) <= 6 && regions) {
      const point = canvasPointer(event);
      const region = regions.hitAt(canvas.getContext('2d'), point.x, point.y);
      if (region) onPick({ type: 'region', region });
    }
  });
  canvas.addEventListener('pointercancel', event => { if (!window.__nuwaxBoundaryActive) finishDrag(event); });
  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    if (state.map2dCameraMode === 'free') { zoomFreeMapAt(canvasPointer(event), event.deltaY < 0 ? 1.12 : 1 / 1.12); requestMapDraw(false); return; }
    setMapZoom(state.mapZoom * (event.deltaY < 0 ? 1.12 : 1 / 1.12));
  }, { passive: false });

  /* ---------- 面板模块用到的全局钩子（原样保留，P3 迁入 navigation.js、boundary_draw.js 时使用） ---------- */
  window.__nuwaxActiveMapName = activeMapName;
  window.__nuwaxMapRevision = value => { const revision = normalizedCursor(value); return revision && revision !== '0' ? revision : null; };
  window.__nuwaxMapBinding = binding;
  window.__nuwaxMapUnproject = (screenX, screenY) => {
    if (!state.mapGrid || state.viewMode === '3d') return null;
    const pointer = canvasPointer({ clientX: Number(screenX), clientY: Number(screenY) });
    const world = canvasToMap(pointer.x, pointer.y, cameraAnchor());
    return world ? { x: world.x, y: world.y, frame_id: navigationFrameId() } : null;
  };
  window.__nuwaxResolveNavigationHeight = resolveNavigationHeight;
  window.__nuwaxResetMapInteraction = () => { state.mapPanDrag = null; requestMapDraw(false); };
  window.__nuwaxMapToolV2 = { active: () => Boolean(window.__nuwaxBoundaryActive || window.__nuwaxNavigationActive) };
  window.__nuwaxMapWorldViewport = () => {
    const anchor = cameraAnchor(), scale = cameraScale(anchor);
    if (!Number.isFinite(scale) || scale <= 0) return null;
    const center = anchor.point || { x: 0, y: 0 };
    return { cx: Number(center.x) || 0, cy: Number(center.y) || 0, halfX: canvas.width / 2 / scale, halfY: canvas.height / 2 / scale, scale,
      yaw: anchor.yaw || 0, mapFixed: Boolean(anchor.mapFixed), pixelWidth: canvas.width, pixelHeight: canvas.height };
  };
  window.__nuwaxMapProject = (worldX, worldY) => {
    const anchor = cameraAnchor(), scale = cameraScale(anchor);
    const projected = mapToCanvas({ x: Number(worldX), y: Number(worldY) }, anchor, scale);
    if (!projected || !Number.isFinite(projected.x) || !Number.isFinite(projected.y)) return null;
    return { x: projected.x, y: projected.y, scale, pixelWidth: canvas.width, pixelHeight: canvas.height, mapFixed: Boolean(anchor.mapFixed), yaw: Number(anchor.yaw) || 0 };
  };
  window.__nuwaxMapViewMode = () => state.viewMode;
  window.__nuwaxMapRequestDraw = () => requestMapDraw(false);
  window.__nuwaxCameraAnchor = cameraAnchor;
  window.__nuwaxMapFocusRegion = space => {
    if (state.viewMode === '3d') { window.pointcloud3d?.focusSemanticRegion(space.scene_space_id, space.boundary); return; }
    const points = space.boundary?.polygon_points;
    if (!points?.length || window.__nuwaxMapToolV2.active()) return;
    const xs = points.map(p => p.x), ys = points.map(p => p.y);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    state.map2dCameraMode = 'free';
    state.mapFreeView = { center: { x: (minX + maxX) / 2, y: (minY + maxY) / 2 }, zoom: 1, yaw: NORTH_UP_YAW,
      fitSpan: { x: Math.max(maxX - minX, 3) * 1.25, y: Math.max(maxY - minY, 3) * 1.25 } };
    requestMapDraw(false);
    emit();
  };

  /* ---------- 三维 ---------- */
  window.addEventListener('pointcloud3d:map-frame', event => {
    const frame = event.detail;
    if (frame) metrics.record('map3d', frame.source === 'saved' ? null : frame.sourceTime, frame.frameKey, frame.sourceKey);
  });
  window.addEventListener('pointcloud3d:pose-drawn', event => recordDisplayedMapPose('map3d', event.detail?.pose, event.detail?.displayedAt));
  window.addEventListener('pointcloud3d:consumer-ready', () => { recordPointcloudDiagnostic({ pointcloudConsumerReady: true }); setViewMode(state.viewMode); });
  window.addEventListener('pointcloud3d:diagnostic', event => recordPointcloudDiagnostic(event.detail || {}));
  window.addEventListener('pointcloud3d:content-drawn', event => recordPointcloudDiagnostic({ pointcloudDrawnPoints: event.detail?.points, pointcloudDrawnRevision: event.detail?.revision }));
  window.addEventListener('pointcloud3d:ready', () => {
    state.pc3dReady = true;
    recordPointcloudDiagnostic({ pointcloudInitialized: true });
    window.pointcloud3d?.setCameraMode(state.pc3dCameraMode);
    window.pointcloud3d?.setPointStyle(state.pointStyle);
    syncPointcloud3dState();
  });
  window.addEventListener('pointcloud3d:semantic-select', event => {
    const detail = event.detail || {};
    const instances = Array.isArray(detail.instances) ? detail.instances : detail.instance ? [detail.instance] : [];
    if (!instances.length) return;
    onPick(instances.length > 1 ? { type: 'cluster', cluster: { members: instances } } : { type: 'item', instance: instances[0] });
  });
  window.addEventListener('pointcloud3d:semantic-region-select', event => {
    const region = regions?.regions.find(r => r.scene_space_id === event.detail?.scene_space_id);
    if (region) onPick({ type: 'region', region });
  });
  window.addEventListener('pointcloud3d:camera-state', event => {
    const detail = event.detail || {};
    state.pc3dCameraMode = detail.mode === 'follow' ? 'follow' : 'free';
    if (Number.isFinite(Number(detail.zoomPercent)) && Number(detail.zoomPercent) > 0) state.pc3dZoomPercent = Number(detail.zoomPercent);
    emit();
  });
  window.addEventListener('pointcloud3d:style-state', event => { state.pointStyle = String(event.detail?.style || 'height'); emit(); });
  window.addEventListener('pointcloud3d:point-count', event => {
    const detail = event.detail || {};
    const displayed = Number(detail.displayedPoints ?? detail.points), source = Number(detail.sourcePoints ?? detail.points);
    state.pc3dDisplayedPoints = Number.isFinite(displayed) ? Math.max(0, Math.floor(displayed)) : 0;
    state.pc3dSourcePoints = Number.isFinite(source) ? Math.max(0, Math.floor(source)) : state.pc3dDisplayedPoints;
    state.pc3dPointCountKnown = Number.isFinite(displayed) || Number.isFinite(source);
    recordPointcloudDiagnostic({ pointcloudDisplayedPoints: state.pc3dDisplayedPoints, pointcloudSourcePoints: state.pc3dSourcePoints });
    emit();
  });
  window.addEventListener('pointcloud3d:dense-state', event => { state.dense = event.detail || null; emit(); });

  /* 视图模式：2d、3d、semantic（语义地图，基于二维）、graph（关系图）；slam_2d 执行链没有 3d。 */
  function setViewMode(mode) {
    const next = mode === 'semantic' || mode === 'graph' ? mode : !PLANAR_PIPELINE && mode === '3d' ? '3d' : '2d';
    if (state.viewMode !== next) window.__nuwaxBoundaryDrawModule?.setDrawing(false);
    state.viewMode = next;
    window.dispatchEvent(new CustomEvent('map:view-changed', { detail: { mode: next } }));
    canvas.hidden = next === '3d' || next === 'graph';
    container3d.hidden = next !== '3d';
    window.pointcloud3d?.setVisible(next === '3d' && visible());
    if (next === '3d') {
      if (!window.pointcloud3d) return;
      window.pointcloud3d.init(container3d);
      window.pointcloud3d.setVisible(visible());
      window.pointcloud3d.setCameraMode(state.pc3dCameraMode);
      syncPointcloud3dState();
      requestAnimationFrame(() => window.pointcloud3d?.resize());
    } else if (next !== 'graph') {
      reevaluateCachedMapDisplay();
      requestMapDraw(false);
    }
    regions?.sync();
    graph?.sync();
    emit();
  }
  if (!PLANAR_PIPELINE) import('/static/js/pointcloud3d.js');

  return {
    attach(layers) { ({ regions, graph, items } = layers); },
    setViewMode,
    get viewMode() { return state.viewMode; },
    /* 相机：follow 以机器人为中心，free 自由视角；二维与三维共用一个选择。 */
    setCameraMode(mode) {
      if (window.__nuwaxBoundaryActive || state.mapPanDrag) return;
      state.pc3dCameraMode = mode;
      setMap2dCameraMode(mode);
      window.pointcloud3d?.setCameraMode(mode);
      emit();
    },
    cameraMode() { return state.viewMode === '3d' ? state.pc3dCameraMode : state.map2dCameraMode; },
    zoom(factor) {
      if (state.viewMode === '3d') window.pointcloud3d?.zoomBy(1 / factor);
      else setMapZoom(mapViewZoom() * factor);
    },
    zoomPercent() { return Math.round(state.viewMode === '3d' ? state.pc3dZoomPercent : mapViewZoom() * 100); },
    reset() {
      if (state.viewMode === '3d') { window.pointcloud3d?.resetView(); return; }
      if (state.map2dCameraMode === 'free') resetFreeMapView(); else setMapZoom(1);
      requestMapDraw(false);
    },
    centerRobot() { if (state.viewMode === '3d') window.pointcloud3d?.centerOnRobot(); else centerFreeMapOnRobot(); },
    /* 在二维地图上查看某一点：自由视角并以该点为中心（物品“在地图中查看”，调用方先切到二维视图）。 */
    focusPoint(x, y, highlight) {
      setMap2dCameraMode('free');
      state.mapFreeView.center = { x, y };
      if (highlight) state.highlightTarget = highlight;
      requestMapDraw(false);
      emit();
    },
    setHighlight(value) { state.highlightTarget = value; requestMapDraw(false); },
    /* 重定位初值标记 {x, y, yaw}，null 清除。 */
    setSeedPose(pose) { state.seedPose = pose; requestMapDraw(false); },
    setPointStyle(style) { state.pointStyle = style; window.pointcloud3d?.setPointStyle(style); emit(); },
    /* 舞台重新可见或尺寸变化时调用。 */
    refresh() {
      window.pointcloud3d?.setVisible(state.viewMode === '3d' && visible());
      if (state.viewMode === '3d') window.pointcloud3d?.resize(); else requestMapDraw(false);
    },
    requestDraw: () => requestMapDraw(false),
    binding, activeMapName, mapToCanvas,
    highlight: () => state.highlightTarget,
    /* 状态条读取的摘要。 */
    info() {
      const point = pointFromPose(state.latestPose);
      const grid = state.mapGrid;
      const kind = state.viewMode === '3d' ? 'map3d' : 'map2d';
      return {
        mapName: activeMapName(), hasMap: Boolean(grid) || (state.viewMode === '3d' && state.pc3dPointCountKnown && state.pc3dSourcePoints > 0),
        emptyMessage: state.emptyMessage,
        pose: point ? { ...point, yaw: planarYawFromPose(state.latestPose) } : null,
        frameId: navigationFrameId(), rate: metrics.rate(kind), poseLatency: poseLatency(kind),
        grid: grid ? { width: Number(grid.width), height: Number(grid.height), resolution: gridResolution(grid) } : null,
        points: state.pc3dPointCountKnown ? { source: state.pc3dSourcePoints, displayed: state.pc3dDisplayedPoints } : null,
        dense: state.dense, pointStyle: state.pointStyle, cameraMode: state.viewMode === '3d' ? state.pc3dCameraMode : state.map2dCameraMode,
        robotAvailable: Boolean(pointFromPose(state.viewMode === '3d' ? state.latestPose : state.pose)),
        retainedPath: state.navigationPath.length > 0 && state.navigationDisplay?.mode !== 'executing',
        navigationPath: state.navigationPath.length > 0, frontiers: state.frontiers.length > 0,
      };
    },
  };
}
