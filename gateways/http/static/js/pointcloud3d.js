import * as THREE from './three.module.min.js';
import { SemanticRegionLayer } from './semantic-regions3d.js?v=20260922-semantic-map';
import { SceneGraphLayer } from './scene-graph3d.js?v=20260922-scene-graph';
import { OrbitControls } from './OrbitControls.js';
import { PointCloudProcessor } from './pointcloud-processing.js?v=20260918-dense-pointcloud';
import { DensePointCloudLayer } from './dense-pointcloud.js?v=20260918-dense-pointcloud';

const COLORS = {
  robot: 0xfbbf24,
  frontier: 0xf472b6,
  trajectory: 0x22d3ee,
  navPath: 0x22ff66,
  navGoal: 0xa78bfa,
  semantic: 0xf59e0b,
  semanticFallback: 0x94a3b8,
  forbiddenInside: 0xef4444,
  forbiddenOutside: 0xa855f7,
  forbiddenDisabled: 0x94a3b8,
};

const LOD_STRIDE = { preview: 8, medium: 4, full: 1 };
const DEFAULT_POINT_STYLE = 'height';
const DEFAULT_POINTCLOUD3D_CONFIG = Object.freeze({
  spacingM: 0.1,
  size: 1.35,
  color: '#cbd5e1',
  heightColors: Object.freeze(['#38bdf8', '#22d3ee', '#4ade80', '#facc15', '#fb7185']),
  groundColor: '#38bdf8',
  structureColor: '#f8fafc',
  objectColor: '#fbbf24',
});

function normalizePointCloudNumber(value, fallback, minimum, maximum) {
  const number = Number(value);
  return Number.isFinite(number) && number >= minimum && number <= maximum ? number : fallback;
}

function normalizePointCloudHexColor(value, fallback) {
  const color = String(value || '').trim().toLowerCase();
  return /^#[0-9a-f]{6}$/.test(color) ? color : fallback;
}

function pointCloudHexToRgb(value) {
  const hex = value.slice(1);
  return [
    Number.parseInt(hex.slice(0, 2), 16),
    Number.parseInt(hex.slice(2, 4), 16),
    Number.parseInt(hex.slice(4, 6), 16),
  ];
}

function normalizePointCloud3DConfig(rawConfig) {
  const config = rawConfig && typeof rawConfig === 'object' ? rawConfig : {};
  const rawHeightColors = config.heightColors ?? config.height_colors;
  const heightColors = Array.isArray(rawHeightColors)
    && rawHeightColors.length === DEFAULT_POINTCLOUD3D_CONFIG.heightColors.length
    ? rawHeightColors.map((color, index) => normalizePointCloudHexColor(
      color,
      DEFAULT_POINTCLOUD3D_CONFIG.heightColors[index],
    ))
    : [...DEFAULT_POINTCLOUD3D_CONFIG.heightColors];
  return Object.freeze({
    spacingM: normalizePointCloudNumber(
      config.spacingM ?? config.spacing_m,
      DEFAULT_POINTCLOUD3D_CONFIG.spacingM,
      0.01,
      2,
    ),
    size: normalizePointCloudNumber(
      config.size,
      DEFAULT_POINTCLOUD3D_CONFIG.size,
      0.1,
      20,
    ),
    color: normalizePointCloudHexColor(config.color, DEFAULT_POINTCLOUD3D_CONFIG.color),
    heightColors: Object.freeze(heightColors),
    groundColor: normalizePointCloudHexColor(
      config.groundColor ?? config.ground_color,
      DEFAULT_POINTCLOUD3D_CONFIG.groundColor,
    ),
    structureColor: normalizePointCloudHexColor(
      config.structureColor ?? config.structure_color,
      DEFAULT_POINTCLOUD3D_CONFIG.structureColor,
    ),
    objectColor: normalizePointCloudHexColor(
      config.objectColor ?? config.object_color,
      DEFAULT_POINTCLOUD3D_CONFIG.objectColor,
    ),
  });
}

let POINTCLOUD3D_CONFIG = normalizePointCloud3DConfig(
  typeof window === 'undefined' ? null : window.__NUWAX_POINTCLOUD3D_CONFIG__,
);
function buildPointStylePresets() { return Object.freeze({
  full: Object.freeze({
    label: '完整点云',
    denseSource: true,
    size: POINTCLOUD3D_CONFIG.size * (1.25 / 1.35),
    sizeAttenuation: false,
    colorMode: 'height',
  }),
  height: Object.freeze({
    label: '高度',
    preserveSource: true,
    size: POINTCLOUD3D_CONFIG.size,
    sizeAttenuation: false,
    colorMode: 'height',
  }),
  singleFloor: Object.freeze({
    label: '单层',
    sampleStride: 1,
    liveSpacingM: POINTCLOUD3D_CONFIG.spacingM * 0.8,
    size: POINTCLOUD3D_CONFIG.size,
    sizeAttenuation: false,
    colorMode: 'singleFloor',
    floorFilter: 'story',
  }),
  ground: Object.freeze({
    label: '地面',
    sampleStride: 1,
    liveSpacingM: POINTCLOUD3D_CONFIG.spacingM * 0.8,
    size: POINTCLOUD3D_CONFIG.size * (1.55 / 1.35),
    sizeAttenuation: false,
    colorMode: 'ground',
    floorFilter: 'ground',
  }),
  structure: Object.freeze({
    label: '结构',
    sampleStride: 1,
    liveSpacingM: POINTCLOUD3D_CONFIG.spacingM * 0.8,
    size: POINTCLOUD3D_CONFIG.size,
    sizeAttenuation: false,
    colorMode: 'structure',
    floorFilter: 'story',
  }),
  mono: Object.freeze({
    label: '单色',
    sampleStride: 3,
    size: POINTCLOUD3D_CONFIG.size * (1.1 / 1.35),
    sizeAttenuation: false,
    colorMode: 'mono',
  }),
}); }
let POINT_STYLE_PRESETS = buildPointStylePresets();
let HEIGHT_COLOR_STOPS = Object.freeze(
  POINTCLOUD3D_CONFIG.heightColors.map((color) => Object.freeze(pointCloudHexToRgb(color))),
);
let FLOOR_COLORS = Object.freeze({
  ground: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.groundColor)),
  context: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.color)),
  structure: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.structureColor)),
  object: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.objectColor)),
});
const MAX_BUFFER_BYTES = 32 * 1024 * 1024;
const MAX_POINTS = Math.floor(MAX_BUFFER_BYTES / (3 * 4));
const CAMERA_MODES = new Set(['free', 'follow']);
const CAMERA_MIN_DISTANCE = 1.5;
const CAMERA_MAX_DISTANCE = 180;
const FOLLOW_TARGET_HEIGHT = 0.25;
const DEFAULT_CAMERA_DIRECTION = new THREE.Vector3(0, -1, 0.72).normalize();
const SEMANTIC_HIDE_ZOOM_PERCENT = 35;
const SEMANTIC_PICK_CLUSTER_PX = 28;
const FORBIDDEN_CIRCLE_SEGMENTS = 64;

function normalizeForbiddenPoint(value) {
  if (!value || typeof value !== 'object') return null;
  const x = Number(value.x);
  const y = Number(value.y);
  return Number.isFinite(x) && Number.isFinite(y) ? { x, y } : null;
}

function normalizeForbiddenZone(value) {
  if (!value || typeof value !== 'object') return null;
  let shape = String(value.shape || '').toLowerCase();
  if (shape === 'custom') shape = 'polygon';
  if (!['line', 'rectangle', 'circle', 'polygon'].includes(shape)) return null;
  const rawVertices = Array.isArray(value.vertices)
    ? value.vertices
    : Array.isArray(value.points) ? value.points : [];
  let vertices = rawVertices.map(normalizeForbiddenPoint).filter(Boolean);
  if (shape === 'rectangle' && vertices.length > 2) {
    const xs = vertices.map((point) => point.x);
    const ys = vertices.map((point) => point.y);
    vertices = [
      { x: Math.min(...xs), y: Math.min(...ys) },
      { x: Math.max(...xs), y: Math.max(...ys) },
    ];
  }
  const center = normalizeForbiddenPoint(value.center);
  const radius = Number(value.radius_m);
  const strokeWidth = Number(value.stroke_width_m ?? value.line_width_m ?? 0.25);
  if (shape === 'line' && vertices.length !== 2) return null;
  if (shape === 'rectangle' && vertices.length !== 2) return null;
  if (shape === 'polygon' && vertices.length < 3) return null;
  if (shape === 'circle' && (!center || !Number.isFinite(radius) || radius <= 0)) return null;
  return {
    id: String(value.id || value.zone_id || ''),
    shape,
    mode: shape === 'line' ? 'inside' : value.mode === 'outside' ? 'outside' : 'inside',
    vertices: shape === 'circle' ? [] : vertices,
    center: shape === 'circle' ? center : null,
    radius_m: shape === 'circle' ? radius : null,
    stroke_width_m: Number.isFinite(strokeWidth) && strokeWidth > 0 ? strokeWidth : 0.25,
    enabled: value.enabled !== false,
    metadata: value.metadata && typeof value.metadata === 'object' ? { ...value.metadata } : {},
  };
}

function forbiddenZoneEntry(value, preferredMapName = null) {
  let root = value;
  for (let index = 0; index < 3; index += 1) {
    if (!root || typeof root !== 'object' || root.maps || root.zones || root.map_name) break;
    const next = root.data || root.result || root.payload;
    if (!next || next === root) break;
    root = next;
  }
  if (!root || typeof root !== 'object') return null;
  let entry = root;
  let mapName = String(root.map_name || '').trim() || null;
  if (root.maps && typeof root.maps === 'object') {
    const keys = Object.keys(root.maps);
    const preferred = String(preferredMapName || '').trim();
    const key = preferred
      ? Object.prototype.hasOwnProperty.call(root.maps, preferred)
        ? preferred
        : keys.find((candidate) => String(root.maps[candidate] && root.maps[candidate].map_name || '').trim() === preferred) || null
      : keys.length === 1 ? keys[0] : null;
    if (!key) return null;
    entry = root.maps[key] || {};
    mapName = String(entry.map_name || key).trim() || null;
  }
  if (!Array.isArray(entry.zones)) return null;
  return {
    mapName,
    generation: Math.max(0, Number(entry.generation) || 0),
    zones: entry.zones.map(normalizeForbiddenZone).filter(Boolean),
  };
}

const POINTCLOUD_BINARY_MAGIC = Object.freeze([0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0x00]);
const POINTCLOUD_BINARY_VERSION = 1;
const POINTCLOUD_BINARY_PREFIX_BYTES = 13;
const POINTCLOUD_BINARY_MAX_HEADER_BYTES = 64 * 1024;
const POINTCLOUD_MAX_U64 = 18446744073709551615n;
const POINTCLOUD_BINARY_TOPICS = new Set(['pointcloud_map.snapshot', 'pointcloud_map.live_chunk']);

function pointCloudIdentityBigInt(value, fieldName) {
  if (typeof value !== 'string' || !/^(0|[1-9]\d*)$/.test(value)) {
    throw new Error(`${fieldName} 必须是规范十进制 u64 字符串`);
  }
  const parsed = BigInt(value);
  if (parsed > POINTCLOUD_MAX_U64) {
    throw new Error(`${fieldName} 超出 u64 范围`);
  }
  return parsed;
}

export function parsePointCloudMapBinaryEnvelope(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < POINTCLOUD_BINARY_PREFIX_BYTES) {
    throw new Error('点云二进制信封被截断');
  }
  const bytes = new Uint8Array(buffer);
  if (!POINTCLOUD_BINARY_MAGIC.every((value, index) => bytes[index] === value)) {
    throw new Error('未知点云二进制 magic');
  }
  if (bytes[8] !== POINTCLOUD_BINARY_VERSION) {
    throw new Error('不支持的点云二进制版本');
  }
  const headerLength = new DataView(buffer, 9, 4).getUint32(0, true);
  if (headerLength <= 0 || headerLength > POINTCLOUD_BINARY_MAX_HEADER_BYTES) {
    throw new Error('点云 JSON header 长度非法');
  }
  const payloadOffset = POINTCLOUD_BINARY_PREFIX_BYTES + headerLength;
  if (payloadOffset > buffer.byteLength) throw new Error('点云 JSON header 被截断');
  let header;
  try {
    header = JSON.parse(new TextDecoder('utf-8', { fatal: true }).decode(
      bytes.slice(POINTCLOUD_BINARY_PREFIX_BYTES, payloadOffset),
    ));
  } catch (_) {
    throw new Error('点云 JSON header 非法');
  }
  if (!header || typeof header !== 'object' || !POINTCLOUD_BINARY_TOPICS.has(header.topic)) {
    throw new Error('点云 binary topic 非法');
  }
  if (!['live', 'saved'].includes(header.source)) throw new Error('点云 source 非法');
  if (typeof header.source_epoch !== 'string' || !header.source_epoch) throw new Error('点云 source_epoch 缺失');
  if (typeof header.revision !== 'string' || !header.revision) throw new Error('点云 revision 缺失');
  if (typeof header.frame_id !== 'string' || !header.frame_id) throw new Error('点云 frame_id 缺失');
  if (header.format !== 'float32-xyz-le') throw new Error('点云 format 非法');
  const cursor = pointCloudIdentityBigInt(header.cursor, 'cursor');
  const streamGeneration = pointCloudIdentityBigInt(header.stream_generation, 'stream_generation');
  const revisionOrdinal = pointCloudIdentityBigInt(header.revision_ordinal, 'revision_ordinal');
  const sourceTimestampNs = pointCloudIdentityBigInt(header.source_timestamp_ns, 'source_timestamp_ns');
  if (!Number.isSafeInteger(header.point_count) || header.point_count < 0 || header.point_count > MAX_POINTS) {
    throw new Error('点云 point_count 非法');
  }
  if (!Number.isSafeInteger(header.payload_length) || header.payload_length < 0) {
    throw new Error('点云 payload_length 非法');
  }
  const expectedPayloadLength = header.point_count * 12;
  if (header.payload_length !== expectedPayloadLength) throw new Error('点云 header 点数与载荷长度不一致');
  if (buffer.byteLength !== payloadOffset + header.payload_length) throw new Error('点云信封总长度不一致');
  const payloadView = new DataView(buffer, payloadOffset, header.payload_length);
  const xyz = new Float32Array(header.point_count * 3);
  for (let index = 0; index < xyz.length; index += 1) {
    const coordinate = payloadView.getFloat32(index * 4, true);
    if (!Number.isFinite(coordinate)) throw new Error('点云 XYZ 坐标必须全部有限');
    xyz[index] = coordinate;
  }
  return {
    header,
    cursor,
    streamGeneration,
    revisionOrdinal,
    sourceTimestampNs,
    xyz,
  };
}

export function shouldAcceptPointCloudMapEnvelope(current, next) {
  if (!current) return true;
  if (next.streamGeneration > current.streamGeneration) return true;
  if (next.streamGeneration < current.streamGeneration) return false;
  if (next.header.source !== current.header.source || next.header.source_epoch !== current.header.source_epoch) {
    return false;
  }
  return next.revisionOrdinal > current.revisionOrdinal;
}

function explorationOverlayInteger(value, fieldName) {
  if (typeof value === 'number') {
    if (!Number.isSafeInteger(value) || value < 0) throw new Error(`${fieldName} 必须是安全非负整数`);
    return BigInt(value);
  }
  return pointCloudIdentityBigInt(value, fieldName);
}

export function parseExplorationOverlayIdentity(detail) {
  const raw = detail?.exploration_identity;
  if (!raw || typeof raw !== 'object' || Array.isArray(raw)) return null;
  return {
    generation: explorationOverlayInteger(raw.generation, 'exploration_generation'),
    revision: explorationOverlayInteger(raw.revision, 'exploration_revision'),
    timestampNs: explorationOverlayInteger(raw.timestampNs, 'exploration_timestamp_ns'),
    requestId: String(raw.requestId || ''),
  };
}

function compareExplorationOverlayIdentity(current, next) {
  if (next.generation > current.generation) return 1;
  if (next.generation < current.generation) return -1;
  if (next.requestId !== current.requestId) return -1;
  for (const key of ['revision', 'timestampNs']) {
    if (next[key] > current[key]) return 1;
    if (next[key] < current[key]) return -1;
  }
  return 0;
}

export function shouldAcceptExplorationOverlay(current, next) {
  return !current || compareExplorationOverlayIdentity(current, next) > 0;
}

function explorationOverlayIsInactive(detail) {
  const status = String(detail?.exploration_status || '').toLowerCase();
  const sessionMode = String(detail?.geometry_session_mode || '').toLowerCase();
  return detail?.exploration_terminal === true
    || ['succeeded', 'failed', 'cancelled', 'idle'].includes(status)
    || (sessionMode && sessionMode !== 'explore');
}

function navigationEventMatchesExploration(detail, identity) {
  if (detail?.navigation_exploration_validated !== true || !identity) return false;
  let eventIdentity;
  try {
    eventIdentity = parseExplorationOverlayIdentity(detail);
  } catch (_) {
    return false;
  }
  return eventIdentity !== null
    && eventIdentity.generation === identity.generation
    && eventIdentity.requestId === identity.requestId;
}

class PointCloud3DViewer extends PointCloudProcessor {
  constructor() {
    super({ stylePresets: POINT_STYLE_PRESETS, heightColorStops: HEIGHT_COLOR_STOPS, floorColors: FLOOR_COLORS });
    this._container = null;
    this._renderer = null;
    this._scene = null;
    this._camera = null;
    this._controls = null;
    this._pointsMesh = null;
    this._denseCloud = null;
    this._robotMesh = null;
    this._robotGeometryCenter = null;
    this._frontierGroup = null;
    this._forbiddenZoneGroup = null;
    this._semanticGroup = null;
    this._semanticRegionLayer = null;
    this._semanticRegionData = null;
    this._sceneGraphLayer = null;
    this._sceneGraphData = null;
    this._trajectoryLine = null;
    this._navPathLine = null;
    this._navGoalMesh = null;
    this._resizeObserver = null;
    this._interactionActive = false;
    this._hasUserCameraInput = false;
    this._cameraMode = 'free';
    this._cameraReferenceDistance = null;
    this._lastFollowTarget = null;
    this._pointCloudBounds = null;
    this._hasFramedPointCloud = false;
    this._robotPose = null;
    this._frontiers = [];
    this._forbiddenZones = [];
    this._forbiddenZoneMapName = null;
    this._forbiddenZoneGeneration = 0;
    this._trajectory = [];
    this._navigationPath = [];
    this._navigationGoal = null;
    this._semanticInstances = [];
    this._semanticRenderKey = null;
    this._semanticInstancesVisible = true;
    this._semanticOwnedResources = [];
    this._semanticMarkerGeometry = null;
    this._semanticPreciseMaterial = null;
    this._semanticFallbackMaterial = null;
    this._lod = 'preview';
    this._pointStyle = DEFAULT_POINT_STYLE;
    this._displayConfig = POINTCLOUD3D_CONFIG;
    this._rafId = null;
    this._visible = true;
    this._allPositions = new Float32Array(0);
    this._pointCloudMapWatermark = null;
    this._explorationOverlayWatermark = null;
    this._explorationFrontierRenderedForWatermark = false;
    this._explorationOverlayInactive = false;
    this._explorationOverlayRequiresCanonical = false;
    this._hasPointCloudSnapshot = false;
    this._isLiveCloudMapSnapshot = false;
    this._pointCloudSourcePoints = null;
    this._displayedPointCount = 0;
    this._initialized = false;
    this._cloudWorker = null;
    this._cloudWorkerFailed = false;
    this._cloudRunning = null;
    this._cloudPending = null;
    this._cloudEpoch = 0;
    this._cloudJobId = 0;
    this._cloudContextKey = null;
    this._cloudSourceKey = null;
    this._cloudDisposed = false;
    this._cloudLoadToken = 0;
    this._assetCloudUrl = null;
    this._assetCloudSourceKey = null;
    this._assetCloudLoadingUrl = null;
  }

  init(container) {
    this._cloudDisposed = false;
    this._cloudWorkerFailed = false;
    if (this._initialized) {
      this.resize();
      this._requestRender();
      return;
    }
    this._container = container;
    this._setupScene();
    this._setupControls();
    this._setupResizeObserver();
    this._initialized = true;
    this._applyCachedSceneState();
    this._requestRender();
    window.dispatchEvent(new CustomEvent('pointcloud3d:ready'));
    window.dispatchEvent(new CustomEvent('pointcloud3d:status', {
      detail: { state: 'ready', lod: this._lod }
    }));
    this._emitCameraState();
  }

  _setupScene() {
    const w = this._container.clientWidth || 800;
    const h = this._container.clientHeight || 600;

    this._renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: 'low-power' });
    this._renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this._renderer.setSize(w, h);
    this._renderer.domElement.setAttribute('aria-label', '3D 点云地图，可旋转、三轴平移、缩放并点击物品查看详情');
    this._renderer.domElement.setAttribute('tabindex', '0');
    this._container.appendChild(this._renderer.domElement);

    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x0f172a);

    this._camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 500);
    this._camera.position.set(0, -10, 8);
    this._camera.up.set(0, 0, 1);

    this._frontierGroup = new THREE.Group();
    this._scene.add(this._frontierGroup);

    this._forbiddenZoneGroup = new THREE.Group();
    this._forbiddenZoneGroup.name = 'forbidden-zones';
    this._forbiddenZoneGroup.renderOrder = 2;
    this._scene.add(this._forbiddenZoneGroup);

    this._semanticGroup = new THREE.Group();
    this._semanticGroup.visible = this._semanticInstancesVisible;
    this._scene.add(this._semanticGroup);
    this._semanticMarkerGeometry = new THREE.BoxGeometry(0.44, 0.28, 0.24);
    this._semanticPreciseMaterial = new THREE.MeshBasicMaterial({ color: COLORS.semantic });
    this._semanticFallbackMaterial = new THREE.MeshBasicMaterial({
      color: COLORS.semanticFallback,
      transparent: true,
      opacity: 0.62,
      wireframe: true,
    });
    this._semanticRegionLayer = new SemanticRegionLayer(this._scene);
    this._sceneGraphLayer = new SceneGraphLayer(this._scene);
    this._setupSemanticInteraction();
  }

  _setupSemanticInteraction() {
    const canvas = this._renderer?.domElement;
    if (!canvas?.addEventListener) return;
    this._semanticPointerDownHandler = (event) => {
      if (event.isPrimary === false || (event.button ?? 0) !== 0) return;
      this._semanticPointerDown = {
        pointerId: event.pointerId,
        x: Number(event.clientX),
        y: Number(event.clientY),
      };
    };
    this._semanticPointerUpHandler = (event) => {
      const start = this._semanticPointerDown;
      this._semanticPointerDown = null;
      if (!start || event.isPrimary === false || (event.button ?? 0) !== 0) return;
      if (start.pointerId !== undefined && event.pointerId !== undefined && start.pointerId !== event.pointerId) return;
      const x = Number(event.clientX);
      const y = Number(event.clientY);
      if (!Number.isFinite(x) || !Number.isFinite(y) || Math.hypot(x - start.x, y - start.y) > 6) return;
      if (window.__nuwaxNavigationActive) {
        const point = this.pickNavigationPoint(x, y);
        window.dispatchEvent(new CustomEvent('pointcloud3d:navigation-pick', { detail: point }));
        return;
      }
      if (this._emitSemanticSelectionAtClientPoint(x, y)) return;
      if (window.__nuwaxBoundaryActive) return;
      const graphNodeId = this._sceneGraphLayer?.pick(x,y,this._camera,canvas);
      if (graphNodeId) { window.dispatchEvent(new CustomEvent('pointcloud3d:scene-graph-select',{detail:{node_id:graphNodeId}})); return; }
      const regionId = this._semanticRegionLayer?.pick(x,y,this._camera,canvas);
      if (regionId) window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-region-select', { detail: { scene_space_id: regionId } }));
    };
    this._semanticPointerCancelHandler = () => {
      this._semanticPointerDown = null;
    };
    canvas.addEventListener('pointerdown', this._semanticPointerDownHandler);
    canvas.addEventListener('pointerup', this._semanticPointerUpHandler);
    canvas.addEventListener('pointercancel', this._semanticPointerCancelHandler);
  }

  _setupControls() {
    this._controls = new OrbitControls(this._camera, this._renderer.domElement);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.08;
    this._controls.enableRotate = true;
    this._controls.enablePan = this._cameraMode === 'free';
    this._controls.enableZoom = true;
    this._controls.screenSpacePanning = true;
    this._controls.minDistance = CAMERA_MIN_DISTANCE;
    this._controls.maxDistance = CAMERA_MAX_DISTANCE;
    this._controls.minPolarAngle = 0.04;
    this._controls.maxPolarAngle = Math.PI / 2 - 0.02;
    if ('zoomToCursor' in this._controls) this._controls.zoomToCursor = this._cameraMode === 'free';
    this._cameraReferenceDistance = this._camera.position.distanceTo(this._controls.target);
    this._controls.addEventListener('change', () => {
      this._requestRender();
      this._emitCameraState();
    });
    this._controls.addEventListener('start', () => {
      this._interactionActive = true;
      if (this._cameraMode === 'free') this._hasUserCameraInput = true;
      this._requestRender();
    });
    this._controls.addEventListener('end', () => {
      this._interactionActive = false;
      this._requestRender();
      this._emitCameraState();
    });
  }

  pickNavigationPoint(clientX, clientY) {
    const mesh = this._pointStyle === 'full' ? this._denseCloud?.group : this._pointsMesh;
    const identity = this._pointCloudMapWatermark;
    if (!this._visible || !mesh?.visible || !identity || this._renderedCloudSourceKey !== this._cloudSourceKey
        || (this._pointStyle === 'full' && !this._denseCloud?.metadata)) return null;
    const rect = this._renderer.domElement.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    const pointer = new THREE.Vector2((clientX - rect.left) / rect.width * 2 - 1, 1 - (clientY - rect.top) / rect.height * 2);
    this._camera.updateMatrixWorld();
    mesh.updateMatrixWorld();
    const ray = new THREE.Raycaster();
    ray.params.Points.threshold = 0.12;
    ray.setFromCamera(pointer, this._camera);
    let selected = null;
    let nearestPixels = Infinity;
    for (const hit of ray.intersectObject(mesh, true)) {
      // 点云交点是射线上的近似位置；目标必须取命中索引对应的真实地图点。
      const position = hit.object.geometry.getAttribute('position');
      const point = new THREE.Vector3().fromBufferAttribute(position, hit.index).applyMatrix4(hit.object.matrixWorld);
      const projected = point.clone().project(this._camera);
      if (projected.z < -1 || projected.z > 1) continue;
      const pixels = Math.hypot((projected.x - pointer.x) * rect.width / 2, (projected.y - pointer.y) * rect.height / 2);
      if (pixels > 8 || pixels >= nearestPixels - 0.25) continue;
      // 优先贴近光标；四分之一像素内保留更靠近相机的命中，避免选到重叠层。
      nearestPixels = pixels;
      selected = { x: point.x, y: point.y, surface_z: point.z, frame_id: identity.header.frame_id,
        source_key: this._cloudSourceKey, dense_source: this._pointStyle === 'full' ? {
          frame_id: this._denseCloud.metadata.frame_id,
          session_id: this._denseCloud.metadata.session_id,
          session_generation: this._denseCloud.metadata.session_generation,
        } : null };
    }
    return selected;
  }

  // 禁区仍按地图平面坐标保存；首点高度仅用于三维编辑平面的投影。
  boundaryPointAt(clientX, clientY, anchor) {
    if (!anchor) return this.pickNavigationPoint(clientX, clientY);
    if (!this.navigationPointIsCurrent(anchor) || !this._camera || !this._visible) return null;
    const rect = this._renderer.domElement.getBoundingClientRect();
    if (!rect.width || !rect.height) return null;
    const pointer = new THREE.Vector2((clientX - rect.left) / rect.width * 2 - 1, 1 - (clientY - rect.top) / rect.height * 2);
    this._camera.updateMatrixWorld();
    const ray = new THREE.Raycaster();
    ray.setFromCamera(pointer, this._camera);
    const point = ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -anchor.surface_z), new THREE.Vector3());
    return point ? { ...anchor, x: point.x, y: point.y } : null;
  }

  projectBoundaryPoint(x, y, height = 0) {
    if (!this._camera || !this._renderer || !this._visible) return null;
    const rect = this._renderer.domElement.getBoundingClientRect();
    this._camera.updateMatrixWorld();
    const point = new THREE.Vector3(x, y, height).project(this._camera);
    if (point.z < -1 || point.z > 1) return null;
    return { x: (point.x + 1) * rect.width / 2, y: (1 - point.y) * rect.height / 2,
      pixelWidth: rect.width, pixelHeight: rect.height, scale: 1 };
  }

  setBoundaryEditing(active) {
    this._boundaryEditing = Boolean(active);
    if (this._boundaryEditing) this._settleControlsMotion();
    if (this._controls) this._controls.enabled = !this._boundaryEditing;
  }

  navigationPointIsCurrent(point) {
    const source = point?.dense_source;
    const current = this._denseCloud?.metadata;
    // 增量修订只更新地图内容，已经选定的地图坐标不依赖显示加载进度。
    return Boolean(this._pointCloudMapWatermark && point?.source_key === this._cloudSourceKey
      && (!source || (current && source.frame_id === current.frame_id
        && source.session_id === current.session_id && source.session_generation === current.session_generation)));
  }

  previewNavigationPoint(point) {
    if (this._navigationPickMarker) {
      this._scene?.remove(this._navigationPickMarker);
      this._navigationPickMarker.geometry.dispose();
      this._navigationPickMarker.material.dispose();
      this._navigationPickMarker = null;
    }
    if (point && this._scene && Number.isFinite(point.z)) {
      const marker = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 8), new THREE.MeshBasicMaterial({ color: COLORS.navGoal, depthTest: false }));
      marker.position.set(point.x, point.y, point.z);
      marker.renderOrder = 10;
      this._navigationPickMarker = marker;
      this._scene.add(marker);
    }
    this._requestRender();
  }

  _setupResizeObserver() {
    this._resizeObserver = new ResizeObserver(() => this.resize());
    this._resizeObserver.observe(this._container);
  }

  resize() {
    if (!this._container || !this._camera || !this._renderer) return;
    const w = this._container.clientWidth;
    const h = this._container.clientHeight;
    if (!w || !h) return;
    this._camera.aspect = w / h;
    this._camera.updateProjectionMatrix();
    this._renderer.setSize(w, h);
    this._requestRender();
  }

  _requestRender() {
    if (!this._visible || !this._renderer || !this._scene || !this._camera) return;
    if (!this._rafId) {
      this._rafId = requestAnimationFrame(() => this._renderLoop());
    }
  }

  _removeLegacySceneHelpers() {
    if (!this._scene) return 0;
    const helpers = [];
    this._scene.traverse((object) => {
      if (object !== this._scene && (object.type === 'GridHelper' || object.type === 'AxesHelper')) {
        helpers.push(object);
      }
    });
    for (const helper of helpers) {
      helper.parent?.remove(helper);
      helper.geometry?.dispose?.();
      if (Array.isArray(helper.material)) helper.material.forEach((material) => material.dispose?.());
      else helper.material?.dispose?.();
    }
    return helpers.length;
  }

  _renderLoop() {
    this._rafId = null;
    if (!this._visible) return;
    const controlsChanged = Boolean(this._controls?.update());
    this._removeLegacySceneHelpers();
    this._syncSemanticGroupVisibility();
    this._semanticRegionLayer?.updateLabels(this._camera,this._container);
    this._sceneGraphLayer?.updateForCamera(this._camera,this._container);
    this._renderer.render(this._scene, this._camera);
    if (this._robotMesh?.visible && this._robotPoseSource && this._lastPoseDrawnSource !== this._robotPoseSource) {
      this._lastPoseDrawnSource = this._robotPoseSource;
      window.dispatchEvent(new CustomEvent('pointcloud3d:pose-drawn', {
        detail: { pose: this._robotPoseSource, displayedAt: Date.now() },
      }));
    }
    if (this._pointCloudDrawDiagnosticPending) {
      this._pointCloudDrawDiagnosticPending = false;
      window.dispatchEvent(new CustomEvent('pointcloud3d:content-drawn', {
        detail: { points: this._renderer.info?.render?.points ?? null, revision: this._renderedCloudMapRevision ?? this._lastCloudMapRevision ?? null },
      }));
    }
    if (this._interactionActive || controlsChanged) {
      this._requestRender();
    }
  }

  _cameraDistance() {
    const target = this._controls?.target;
    return this._camera && target
      ? this._camera.position.distanceTo(target)
      : null;
  }

  setVisible(visible) {
    this._visible = Boolean(visible);
    if (!this._visible) {
      this._denseCloud?.stop();
      this._lastPoseDrawnSource = null;
      if (this._rafId) cancelAnimationFrame(this._rafId);
      this._rafId = null;
      return;
    }
    if (this._pointStyle === 'full') this._rebuildPointCloud();
    this.resize();
    this._requestRender();
  }

  _setCameraReferenceDistance(distance = this._cameraDistance()) {
    if (Number.isFinite(distance) && distance > 0) this._cameraReferenceDistance = distance;
  }

  _zoomPercent(distance = this._cameraDistance()) {
    if (!Number.isFinite(distance) || distance <= 0) return null;
    if (!Number.isFinite(this._cameraReferenceDistance) || this._cameraReferenceDistance <= 0) {
      this._setCameraReferenceDistance(distance);
    }
    return this._cameraReferenceDistance / distance * 100;
  }

  _emitCameraState() {
    const distance = this._cameraDistance();
    window.dispatchEvent(new CustomEvent('pointcloud3d:camera-state', {
      detail: {
        mode: this._cameraMode,
        distance,
        zoomPercent: this._zoomPercent(distance),
        robotAvailable: Boolean(this._robotPose),
        initialized: this._initialized,
      },
    }));
  }

  _robotTarget(pose = this._robotPose) {
    if (!pose) return null;
    const center = this._robotGeometryCenter?.clone() || new THREE.Vector3(0, 0, FOLLOW_TARGET_HEIGHT);
    if (pose.quaternion) center.applyQuaternion(pose.quaternion);
    return center.add(new THREE.Vector3(pose.x, pose.y, pose.z));
  }

  _settleControlsMotion() {
    if (!this._controls) return;
    const dampingEnabled = this._controls.enableDamping === true;
    this._controls.enableDamping = false;
    this._controls.update();
    this._controls.enableDamping = dampingEnabled;
  }

  _moveCameraTarget(target, preserveOffset = true) {
    if (!target || !this._camera || !this._controls) return false;
    this._settleControlsMotion();
    const previousTarget = this._controls.target.clone();
    if (preserveOffset) {
      this._camera.position.add(target.clone().sub(previousTarget));
    }
    this._controls.target.copy(target);
    this._controls.update();
    this._requestRender();
    return true;
  }

  setCameraMode(mode) {
    if (!CAMERA_MODES.has(mode)) return false;
    this._cameraMode = mode;
    if (this._controls) {
      this._controls.enablePan = mode === 'free';
      if ('zoomToCursor' in this._controls) this._controls.zoomToCursor = mode === 'free';
    }
    if (mode === 'follow' && this._robotPose) {
      const target = this._robotTarget();
      this._moveCameraTarget(target, true);
      this._lastFollowTarget = target.clone();
    } else if (mode === 'free') {
      this._lastFollowTarget = null;
    }
    this._emitCameraState();
    this._requestRender();
    return true;
  }

  zoomBy(scaleFactor) {
    if (!this._camera || !this._controls) return false;
    const factor = Number(scaleFactor);
    if (!Number.isFinite(factor) || factor <= 0) return false;
    const offset = this._camera.position.clone().sub(this._controls.target);
    if (offset.lengthSq() < 1e-8) offset.copy(DEFAULT_CAMERA_DIRECTION);
    const minDistance = Number(this._controls.minDistance) || CAMERA_MIN_DISTANCE;
    const maxDistance = Number(this._controls.maxDistance) || CAMERA_MAX_DISTANCE;
    const distance = THREE.MathUtils.clamp(offset.length() * factor, minDistance, maxDistance);
    this._camera.position.copy(this._controls.target).add(offset.setLength(distance));
    this._controls.update();
    this._emitCameraState();
    this._requestRender();
    return true;
  }

  centerOnRobot() {
    const target = this._robotTarget();
    if (!target) {
      this._emitCameraState();
      return false;
    }
    const moved = this._moveCameraTarget(target, true);
    if (this._cameraMode === 'follow') this._lastFollowTarget = target.clone();
    this._emitCameraState();
    return moved;
  }

  _resetFollowView() {
    const target = this._robotTarget();
    if (!target || !this._camera || !this._controls) return false;
    this._settleControlsMotion();
    const yaw = this._robotPose?.yaw || 0;
    const forward = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
    this._controls.target.copy(target);
    this._camera.position.copy(target)
      .addScaledVector(forward, -6)
      .add(new THREE.Vector3(0, 0, 4.2));
    this._lastFollowTarget = target.clone();
    this._controls.update();
    this._setCameraReferenceDistance();
    this._requestRender();
    return true;
  }

  _framePointCloud() {
    if (!this._pointCloudBounds || !this._camera || !this._controls) return false;
    this._settleControlsMotion();
    const center = this._pointCloudBounds.center;
    const radius = Math.max(0.5, this._pointCloudBounds.radius);
    const halfFov = THREE.MathUtils.degToRad(this._camera.fov * 0.5);
    const fitHalfFov = this._pointStyle === 'full'
      ? Math.atan(Math.tan(halfFov) * Math.min(1, this._camera.aspect)) : halfFov;
    const distance = THREE.MathUtils.clamp(
      radius / Math.max(Math.sin(fitHalfFov), 0.05) * 1.15,
      CAMERA_MIN_DISTANCE,
      CAMERA_MAX_DISTANCE,
    );
    this._controls.target.copy(center);
    this._camera.position.copy(center).add(DEFAULT_CAMERA_DIRECTION.clone().multiplyScalar(distance));
    this._controls.update();
    this._setCameraReferenceDistance(distance);
    this._hasFramedPointCloud = true;
    this._requestRender();
    return true;
  }

  resetView() {
    const reset = this._cameraMode === 'follow'
      ? this._resetFollowView()
      : this._framePointCloud();
    if (!reset && this._camera && this._controls) {
      this._controls.target.set(0, 0, 0);
      this._camera.position.set(0, -10, 8);
      this._controls.update();
      this._setCameraReferenceDistance();
      this._requestRender();
    }
    this._emitCameraState();
    return reset;
  }

  _disposePointsMesh() {
    if (!this._pointsMesh) return;
    this._scene?.remove(this._pointsMesh);
    this._pointsMesh.geometry.dispose();
    this._pointsMesh.material.dispose();
    this._pointsMesh = null;
  }

  _emitPointCount() {
    this._pointCloudDrawDiagnosticPending = true;
    const bufferedPoints = Math.floor(this._allPositions.length / 3);
    const sourcePoints = this._renderedPointCloudSourcePoints ?? (Number.isFinite(this._pointCloudSourcePoints)
      ? Math.max(bufferedPoints, Math.floor(this._pointCloudSourcePoints))
      : bufferedPoints);
    window.dispatchEvent(new CustomEvent('pointcloud3d:point-count', {
      detail: {
        points: this._displayedPointCount,
        displayedPoints: this._displayedPointCount,
        sourcePoints,
        lod: this._lod,
      },
    }));
  }

  _renderPointCloudPositions(positions, sourceMeasurement = null, floorContext = null, colors = null) {
    if (!this._scene) return;
    const previousBounds = this._pointCloudBounds;
    if (!positions.length) {
      if (this._pointsMesh) {
        // 空图立即释放旧点载荷，但保留材质程序供后续点云恢复复用。
        if (this._pointsMesh.geometry.getAttribute('position')) {
          this._pointsMesh.geometry.dispose();
          this._pointsMesh.geometry = new THREE.BufferGeometry();
        }
        this._pointsMesh.visible = false;
      }
      this._displayedPointCount = 0;
      this._pointCloudBounds = null;
      this._emitPointCount();
      if (previousBounds && this._forbiddenZones.length) this._rebuildForbiddenZones();
      this._requestRender();
      return;
    }
    const measurement = sourceMeasurement || this._measurePointCloud(positions);
    const preset = POINT_STYLE_PRESETS[this._pointStyle] || POINT_STYLE_PRESETS[DEFAULT_POINT_STYLE];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.Uint8BufferAttribute(
      colors || this._pointCloudColors(positions, measurement, floorContext),
      3,
      true,
    ));
    // 工作线程只传普通数值；图形资源及其拥有权留在主线程。
    const { center, radius } = measurement.bounds;
    const bounds = new THREE.Sphere(new THREE.Vector3(center.x, center.y, center.z), radius);
    geo.boundingSphere = bounds;
    this._pointCloudBounds = bounds.clone();
    if (this._pointsMesh) {
      this._pointsMesh.geometry.dispose();
      this._pointsMesh.geometry = geo;
      const material = this._pointsMesh.material;
      material.size = preset.size;
      if (material.sizeAttenuation !== preset.sizeAttenuation) {
        material.sizeAttenuation = preset.sizeAttenuation;
        material.needsUpdate = true;
      }
      this._pointsMesh.visible = true;
    } else {
      // 顶点与颜色属于每帧几何，材质跨帧保留以免不断删除和重编译点云程序。
      const material = new THREE.PointsMaterial({
        size: preset.size,
        vertexColors: true,
        sizeAttenuation: preset.sizeAttenuation,
      });
      this._pointsMesh = new THREE.Points(geo, material);
      this._scene.add(this._pointsMesh);
    }
    this._displayedPointCount = Math.floor(positions.length / 3);
    this._emitPointCount();
    if (!this._hasFramedPointCloud && !this._hasUserCameraInput && this._cameraMode === 'free') {
      this._framePointCloud();
    }
    const boundsChanged = !previousBounds
      || previousBounds.center.distanceToSquared(this._pointCloudBounds.center) > 1e-12
      || Math.abs(previousBounds.radius - this._pointCloudBounds.radius) > 1e-6;
    if (boundsChanged && this._forbiddenZones.length) this._rebuildForbiddenZones();
    this._requestRender();
  }

  _invalidatePointCloudWork() {
    this._cloudEpoch += 1;
    this._cloudPending = null;
    this._cloudContextKey = null;
  }

  _pointCloudComputeError(message, fatal = false) {
    if (fatal) {
      this._cloudWorkerFailed = true;
      this._cloudWorker?.terminate();
      this._cloudWorker = null;
      this._cloudRunning = null;
      this._cloudPending = null;
    }
    window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', {
      detail: { pointcloudComputeError: message, pointcloudComputeMode: fatal ? 'worker_failed' : 'worker' },
    }));
    window.dispatchEvent(new CustomEvent('pointcloud3d:status', { detail: { state: 'error', message } }));
  }

  _ensurePointCloudWorker() {
    if (this._cloudWorker || this._cloudWorkerFailed) return this._cloudWorker;
    try {
      const worker = new Worker(new URL('./pointcloud-worker.js?v=20260918-dense-pointcloud', import.meta.url), {
        type: 'module', name: 'nuwax-pointcloud-preprocess',
      });
      this._cloudWorker = worker;
      worker.onmessage = ({ data }) => {
        if (this._cloudDisposed || this._cloudWorker !== worker) return;
        const job = this._cloudRunning;
        if (!job || data?.id !== job.id) {
          this._pointCloudComputeError('点云工作线程返回了不匹配的请求', true);
          return;
        }
        this._cloudRunning = null;
        try {
          if (job.epoch === this._cloudEpoch) {
            if (data.error) this._pointCloudComputeError(String(data.error));
            else this._applyPreparedPointCloud(job, data.result);
          }
        } catch (error) {
          this._pointCloudComputeError(String(error?.message || error), true);
        }
        this._startNextPointCloudWork();
      };
      worker.onerror = (event) => {
        if (this._cloudWorker !== worker) return;
        event.preventDefault?.();
        this._pointCloudComputeError(event.message || '点云工作线程启动或执行失败', true);
      };
      worker.onmessageerror = () => {
        if (this._cloudWorker === worker) this._pointCloudComputeError('无法读取点云工作线程结果', true);
      };
      return worker;
    } catch (error) {
      this._pointCloudComputeError(String(error?.message || error), true);
      return null;
    }
  }

  _startNextPointCloudWork() {
    if (this._cloudDisposed || this._cloudRunning || !this._cloudPending) return;
    const worker = this._ensurePointCloudWorker();
    if (!worker) return;
    const job = this._cloudPending;
    this._cloudPending = null;
    this._cloudRunning = job;
    job.startedAt = performance.now();
    try {
      // 源快照仍用于重绘与样式切换，不能转移其缓冲并使主线程缓存失效。
      // 只在领取单个在途任务时克隆；等待中的候选只保留最新引用。
      worker.postMessage({ id: job.id, positions: job.positions, config: this._processingConfig, options: job.options });
    } catch (error) {
      this._pointCloudComputeError(String(error?.message || error), true);
    }
  }

  _applyPreparedPointCloud(job, result) {
    if (this._cloudDisposed || job.epoch !== this._cloudEpoch) return;
    if (!result || !(result.positions instanceof Float32Array) || !(result.colors instanceof Uint8Array)
        || result.positions.length % 3 !== 0 || result.positions.byteLength > MAX_BUFFER_BYTES
        || result.colors.length !== result.positions.length) {
      throw new Error('点云工作线程返回的显示缓冲非法');
    }
    if (result.positions.length && (!result.measurement?.bounds
        || ![result.measurement.bounds.center?.x, result.measurement.bounds.center?.y,
          result.measurement.bounds.center?.z, result.measurement.bounds.radius].every(Number.isFinite))) {
      throw new Error('点云工作线程返回的显示范围非法');
    }
    const floor = result.floorContext;
    if (floor && this._selectFloorLevel(floor.levels, this._robotPose?.z) !== floor.floorZ) {
      // 计算期间机器人换层；不把旧层安装到当前视图，也不在主线程重扫点云。
      if (job.positions === this._allPositions) {
        this._floorLevelSource = job.positions;
        this._floorLevelCache = floor.levels;
      }
      this._rebuildPointCloud();
      return;
    }
    this._activeFloorZ = floor?.floorZ ?? null;
    if (job.positions === this._allPositions) {
      this._floorLevelSource = floor ? job.positions : null;
      this._floorLevelCache = floor?.levels ?? null;
    }
    this._renderedCloudMapRevision = job.revision;
    this._renderedCloudSourceKey = this._cloudSourceKey;
    this._renderedPointCloudSourcePoints = job.positions.length / 3;
    this._renderPointCloudPositions(result.positions, result.measurement, floor, result.colors);
    window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', {
      detail: {
        pointcloudComputeMode: job.mode,
        pointcloudComputeError: '',
        pointcloudComputeMs: result.computeMs ?? null,
        pointcloudWorkerRoundTripMs: job.startedAt == null ? null : performance.now() - job.startedAt,
      },
    }));
  }

  _rebuildPointCloud() {
    if (this._cloudDisposed) return;
    if (this._pointStyle === 'full') {
      this._invalidatePointCloudWork();
      if (this._pointsMesh) this._pointsMesh.visible = false;
      if (!this._scene) return;
      if (!this._denseCloud) {
        this._denseCloud = new DensePointCloudLayer({
          size: POINT_STYLE_PRESETS.full.size,
          onChange: state => this._applyDenseCloudState(state),
        });
        this._scene.add(this._denseCloud.group);
      }
      if (this._visible) this._denseCloud.start();
      return;
    }
    const src = this._allPositions;
    if (src.byteLength > MAX_BUFFER_BYTES || src.length % 3 !== 0) {
      this._invalidatePointCloudWork();
      this._pointCloudComputeError('点云显示缓冲超过容量或XYZ长度不完整');
      return;
    }
    const preset = POINT_STYLE_PRESETS[this._pointStyle] || POINT_STYLE_PRESETS[DEFAULT_POINT_STYLE];
    const floorTarget = preset.floorFilter && this._floorLevelSource === src
      ? this._selectFloorLevel(this._floorLevelCache || []) : null;
    const contextKey = JSON.stringify([this._cloudSourceKey, this._pointStyle, this._lod, this._isLiveCloudMapSnapshot, floorTarget]);
    if (contextKey !== this._cloudContextKey) {
      this._invalidatePointCloudWork();
      this._cloudContextKey = contextKey;
    }
    if (!this._scene) return;
    const job = {
      id: ++this._cloudJobId, epoch: this._cloudEpoch, positions: src,
      revision: this._lastCloudMapRevision ?? null,
      options: { style: this._pointStyle, lod: this._lod, live: this._isLiveCloudMapSnapshot, robotZ: this._robotPose?.z },
      mode: typeof Worker === 'undefined' ? 'synchronous_no_worker' : 'worker',
    };
    if (!src.length) {
      this._invalidatePointCloudWork();
      job.epoch = this._cloudEpoch;
      this._applyPreparedPointCloud(job, this._preparePointCloud());
    } else if (typeof Worker === 'undefined') {
      // 无工作线程的环境明确报告同步路径，不能伪称隔离完成。
      this._applyPreparedPointCloud(job, this._preparePointCloud());
    } else {
      // 同一来源的持续更新允许在途结果先显示，避免快输入令所有结果饿死。
      // 换来源、换样式、换层、重连或关闭则由代次撤销旧结果。
      this._cloudPending = job;
      this._startNextPointCloudWork();
    }
  }

  _applyDenseCloudState(state) {
    if (this._cloudDisposed || this._pointStyle !== 'full') return;
    this._displayedPointCount = state.count;
    this._renderedPointCloudSourcePoints = state.total;
    this._renderedCloudMapRevision = state.metadata?.revision ?? null;
    this._renderedCloudSourceKey = this._cloudSourceKey;
    this._pointCloudBounds = state.bounds?.clone() || null;
    this._emitPointCount();
    window.dispatchEvent(new CustomEvent('pointcloud3d:dense-state', { detail: { ...state, bounds: undefined, enabled: true } }));
    if (this._pointCloudBounds && !this._hasFramedPointCloud && !this._hasUserCameraInput && this._cameraMode === 'free') this._framePointCloud();
    this._requestRender();
  }

  async loadPointCloud(url, lod = 'preview') {
    if (this._cloudDisposed) return;
    // 制品标识对应不可变内容，视图切换重放同一元数据时复用已有点云。
    if (String(url).includes('/artifacts/') && this._assetCloudUrl === url && this._hasPointCloudSnapshot
        && this._cloudSourceKey === this._assetCloudSourceKey) {
      if (this._lod !== lod) this.setLOD(lod);
      this._requestRender();
      return;
    }
    if (this._assetCloudLoadingUrl === url) return;
    this._assetCloudLoadingUrl = url;
    const token = ++this._cloudLoadToken;
    this._invalidatePointCloudWork();
    this._lod = lod;
    try {
      const resp = await fetch(url);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const buf = await resp.arrayBuffer();
      if (this._cloudDisposed || token !== this._cloudLoadToken) return;
      if (buf.byteLength > MAX_BUFFER_BYTES || buf.byteLength % 12 !== 0) throw new Error('点云文件超过容量或XYZ长度不完整');
      this._cloudSourceKey = JSON.stringify(['asset', url, token]);
      this._lastCloudMapRevision = null;
      this._allPositions = new Float32Array(buf);
      this._assetCloudUrl = url;
      this._assetCloudSourceKey = this._cloudSourceKey;
      this._hasPointCloudSnapshot = true;
      this._isLiveCloudMapSnapshot = false;
      this._pointCloudSourcePoints = Math.floor(this._allPositions.length / 3);
      this._rebuildPointCloud();
      window.dispatchEvent(new CustomEvent('pointcloud3d:status', {
        detail: { state: 'loaded', points: this._allPositions.length / 3, lod }
      }));
    } catch (e) {
      if (this._cloudDisposed || token !== this._cloudLoadToken) return;
      window.dispatchEvent(new CustomEvent('pointcloud3d:status', { detail: { state: 'error', message: e.message } }));
    } finally {
      if (this._assetCloudLoadingUrl === url) this._assetCloudLoadingUrl = null;
    }
  }

  appendPoints(float32Array) {
    if (this._cloudDisposed) return;
    if (float32Array.length % 3 !== 0 || this._allPositions.byteLength + float32Array.byteLength > MAX_BUFFER_BYTES) {
      throw new Error('追加点云超过显示容量或XYZ长度不完整');
    }
    this._cloudLoadToken += 1;
    this._cloudSourceKey = 'legacy-append';
    this._lastCloudMapRevision = null;
    const merged = new Float32Array(this._allPositions.length + float32Array.length);
    merged.set(this._allPositions);
    merged.set(float32Array, this._allPositions.length);
    this._allPositions = merged;
    this._hasPointCloudSnapshot = true;
    this._isLiveCloudMapSnapshot = false;
    this._pointCloudSourcePoints = Math.floor(this._allPositions.length / 3);
    this._rebuildPointCloud();
  }

  _replaceCloudMapBuffer(xyzArray, identity) {
    this._assetCloudUrl = null;
    if (this._cloudDisposed) return;
    this._cloudLoadToken += 1;
    const previousSourceKey = this._cloudSourceKey;
    this._cloudSourceKey = JSON.stringify([identity.header.source, identity.header.source_epoch,
      identity.header.frame_id, String(identity.streamGeneration ?? '')]);
    if (previousSourceKey !== this._cloudSourceKey || xyzArray.length === 0) this._denseCloud?.reset();
    this._lastCloudMapRevision = identity.header.revision;
    this._pointCloudMapWatermark = identity;
    this._hasPointCloudSnapshot = true;
    this._isLiveCloudMapSnapshot = identity.header.source === 'live';
    this._pointCloudSourcePoints = Math.floor(xyzArray.length / 3);

    this._allPositions = xyzArray;
    this._rebuildPointCloud();
  }

  _ensureRobotMesh() {
    if (this._robotMesh || !this._scene) return;
    const group = new THREE.Group();
    const raw = window.__NUWAX_ROBOT_GEOMETRY__;
    const points = raw?.footprint_xy_m;
    const valid = Array.isArray(points) && points.length >= 3
      && points.every(point => Array.isArray(point) && point.length === 2 && point.every(Number.isFinite))
      && Number.isFinite(raw.z_min_m) && Number.isFinite(raw.z_max_m) && raw.z_max_m > raw.z_min_m;
    let label = '机器人位姿（外形未提供）';
    let description = raw?.error || '请在设置中提供机器人模型或实际长宽高。';
    if (valid) {
      // 与探索/导航共用机体坐标凸包；直接使用米制坐标，不居中、不贴地、不随缩放放大。
      const shape = new THREE.Shape(points.map(([x, y]) => new THREE.Vector2(x, y)));
      const geometry = new THREE.ExtrudeGeometry(shape, {
        depth: raw.z_max_m - raw.z_min_m, bevelEnabled: false, steps: 1,
      });
      geometry.translate(0, 0, raw.z_min_m);
      const material = new THREE.MeshBasicMaterial({
        color: COLORS.robot, transparent: true, opacity: 0.18,
        depthWrite: false, side: THREE.DoubleSide,
      });
      const body = new THREE.Mesh(geometry, material);
      body.name = 'robot-envelope';
      group.add(body);
      const edges = new THREE.EdgesGeometry(geometry);
      const edgeMaterial = new THREE.LineBasicMaterial({ color: COLORS.robot, transparent: true, opacity: 0.95 });
      group.add(new THREE.LineSegments(edges, edgeMaterial));

      // 朝向箭头位于上表面内部，不扩大显示外形；机体正X为前方。
      const cx = points.reduce((sum, point) => sum + point[0], 0) / points.length;
      const cy = points.reduce((sum, point) => sum + point[1], 0) / points.length;
      const radius = Math.min(...points.map((a, index) => {
        const b = points[(index + 1) % points.length];
        return Math.abs((b[0] - a[0]) * (cy - a[1]) - (b[1] - a[1]) * (cx - a[0])) / Math.hypot(b[0] - a[0], b[1] - a[1]);
      }));
      const arrow = new THREE.Shape([
        new THREE.Vector2(cx + radius * 0.7, cy),
        new THREE.Vector2(cx - radius * 0.5, cy + radius * 0.5),
        new THREE.Vector2(cx - radius * 0.15, cy),
        new THREE.Vector2(cx - radius * 0.5, cy - radius * 0.5),
      ]);
      const arrowGeometry = new THREE.ShapeGeometry(arrow);
      arrowGeometry.translate(0, 0, raw.z_max_m);
      const arrowMaterial = new THREE.MeshBasicMaterial({ color: COLORS.robot, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 });
      const heading = new THREE.Mesh(arrowGeometry, arrowMaterial);
      heading.name = 'robot-heading';
      group.add(heading);
      group.userData.kind = 'robot-envelope';
      group.userData.disposableResources = [geometry, material, edges, edgeMaterial, arrowGeometry, arrowMaterial];
      const xs = points.map(point => point[0]), ys = points.map(point => point[1]);
      const bounds = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
      this._robotGeometryCenter = new THREE.Vector3((bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2, (raw.z_min_m + raw.z_max_m) / 2);
      label = `${String(raw.source).startsWith('legacy_default') ? '默认参考外形' : '机器人外形'} · ${(bounds[2] - bounds[0]).toFixed(2)} × ${(bounds[3] - bounds[1]).toFixed(2)} × ${(raw.z_max_m - raw.z_min_m).toFixed(2)} 米`;
      description = `模型/手填外包，按真实米制尺寸显示；箭头指向机体前方。来源：${raw.source || '配置外形'}${String(raw.source).startsWith('legacy_default') ? '（未提供完整实际尺寸）' : ''}。模型按静态参考姿态解析，修改设置后重启服务并刷新页面。`;
    } else {
      // 外形未知时只有一个位姿箭头，不把符号大小当作机器人尺寸。
      const geometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(-0.06, 0, 0), new THREE.Vector3(0.08, 0, 0),
        new THREE.Vector3(0.03, 0.04, 0), new THREE.Vector3(0.08, 0, 0),
        new THREE.Vector3(0.03, -0.04, 0),
      ]);
      const material = new THREE.LineBasicMaterial({ color: COLORS.robot });
      group.add(new THREE.Line(geometry, material));
      group.userData.kind = 'robot-pose';
      group.userData.disposableResources = [geometry, material];
    }
    if (typeof document !== 'undefined') {
      const legend = document.getElementById('robotGeometryLegend');
      if (legend) { legend.textContent = label; legend.title = description; }
    }
    group.visible = false;
    this._robotMesh = group;
    this._scene.add(group);
  }

  setRobotPose(x, y, z, yaw, orientation) {
    if (yaw === undefined) {
      yaw = z;
      z = 0;
    }
    const pose = {
      x: Number(x),
      y: Number(y),
      z: Number(z || 0),
      yaw: Number(yaw || 0),
    };
    if (!Number.isFinite(pose.x) || !Number.isFinite(pose.y)) return false;
    if (!Number.isFinite(pose.z)) pose.z = 0;
    if (!Number.isFinite(pose.yaw)) pose.yaw = 0;
    const q = orientation && [orientation.x, orientation.y, orientation.z, orientation.w];
    pose.quaternion = q?.every(Number.isFinite) && Math.hypot(...q) > 0
      ? new THREE.Quaternion(...q).normalize()
      : new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), pose.yaw);
    const previousFollowTarget = this._lastFollowTarget?.clone() || null;
    this._robotPose = pose;

    const preset = POINT_STYLE_PRESETS[this._pointStyle] || POINT_STYLE_PRESETS[DEFAULT_POINT_STYLE];
    if (preset.floorFilter && this._floorLevelSource === this._allPositions) {
      const nextFloorZ = this._selectFloorLevel(this._floorLevelCache || [], pose.z);
      if (Number.isFinite(nextFloorZ) && Math.abs(nextFloorZ - this._activeFloorZ) > 1e-6) {
        this._rebuildPointCloud();
      }
    }

    this._ensureRobotMesh();
    if (this._robotMesh) {
      this._robotMesh.visible = true;
      this._robotMesh.position.set(pose.x, pose.y, pose.z);
      this._robotMesh.quaternion.copy(pose.quaternion);
    }

    if (!this._boundaryEditing && this._cameraMode === 'follow' && this._camera && this._controls) {
      const nextTarget = this._robotTarget(pose);
      if (previousFollowTarget) {
        this._camera.position.add(nextTarget.clone().sub(previousFollowTarget));
        this._controls.target.copy(nextTarget);
        this._controls.update();
      } else {
        this._moveCameraTarget(nextTarget, true);
      }
      this._lastFollowTarget = nextTarget.clone();
    }
    this._emitCameraState();
    this._requestRender();
    return true;
  }

  clearRobotPose() {
    this._robotPose = null;
    this._robotPoseSource = null;
    this._lastPoseDrawnSource = null;
    this._lastFollowTarget = null;
    if (this._robotMesh) this._robotMesh.visible = false;
    const preset = POINT_STYLE_PRESETS[this._pointStyle] || POINT_STYLE_PRESETS[DEFAULT_POINT_STYLE];
    if (preset.floorFilter && this._floorLevelSource === this._allPositions) {
      const nextFloorZ = this._selectFloorLevel(this._floorLevelCache || []);
      if (Number.isFinite(nextFloorZ) && Math.abs(nextFloorZ - this._activeFloorZ) > 1e-6) {
        this._rebuildPointCloud();
      }
    }
    this._emitCameraState();
    this._requestRender();
  }

  _trackSemanticResource(resource) {
    if (resource) this._semanticOwnedResources.push(resource);
    return resource;
  }

  _clearSemanticInstances() {
    this._semanticGroup?.clear();
    for (const resource of this._semanticOwnedResources) resource.dispose?.();
    this._semanticOwnedResources = [];
    this._semanticRenderKey = null;
  }

  _syncSemanticGroupVisibility(distance = this._cameraDistance()) {
    const zoomPercent = this._zoomPercent(distance);
    this._semanticZoomHidden = Number.isFinite(zoomPercent) && zoomPercent < SEMANTIC_HIDE_ZOOM_PERCENT;
    const visible = Boolean(this._semanticInstancesVisible) && !this._semanticZoomHidden;
    if (this._semanticGroup) this._semanticGroup.visible = visible;
    return visible;
  }

  _semanticInstancesAtClientPoint(clientX, clientY) {
    const canvas = this._renderer?.domElement;
    if (!this._semanticGroup?.visible || !this._camera || !canvas?.getBoundingClientRect) return [];
    const rect = canvas.getBoundingClientRect();
    const width = Number(rect.width);
    const height = Number(rect.height);
    const x = Number(clientX);
    const y = Number(clientY);
    if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) return [];
    if (!Number.isFinite(x) || !Number.isFinite(y)) return [];

    this._semanticRaycaster ||= new THREE.Raycaster();
    this._semanticPointer ||= new THREE.Vector2();
    this._semanticPointer.set(
      ((x - rect.left) / width) * 2 - 1,
      -((y - rect.top) / height) * 2 + 1,
    );
    this._scene?.updateMatrixWorld(true);
    this._camera.updateMatrixWorld(true);
    this._semanticRaycaster.setFromCamera(this._semanticPointer, this._camera);
    const selectables = [];
    this._semanticGroup.traverse((object) => {
      if (object.userData?.semanticSelectable && object.userData.semanticInstance) selectables.push(object);
    });
    const intersections = this._semanticRaycaster.intersectObjects(selectables, false);
    if (!intersections.length) return [];

    const primary = intersections[0].object.userData.semanticInstance;
    const primaryObject = this._semanticGroup.children.find((object) => (
      object.userData?.semanticMarker && object.userData.semanticInstance === primary
    )) || intersections[0].object;
    const projectedPrimary = primaryObject.getWorldPosition(new THREE.Vector3()).project(this._camera);
    const primaryX = rect.left + (projectedPrimary.x + 1) * width / 2;
    const primaryY = rect.top + (1 - projectedPrimary.y) * height / 2;
    const instances = [primary];
    const seen = new Set([primary.instance_id || primary]);
    for (const marker of this._semanticGroup.children) {
      const instance = marker.userData?.semanticMarker ? marker.userData.semanticInstance : null;
      if (!instance) continue;
      const key = instance.instance_id || instance;
      if (seen.has(key)) continue;
      const projected = marker.getWorldPosition(new THREE.Vector3()).project(this._camera);
      if (projected.z < -1 || projected.z > 1) continue;
      const screenX = rect.left + (projected.x + 1) * width / 2;
      const screenY = rect.top + (1 - projected.y) * height / 2;
      if (Math.hypot(screenX - primaryX, screenY - primaryY) > SEMANTIC_PICK_CLUSTER_PX) continue;
      seen.add(key);
      instances.push(instance);
    }
    return instances;
  }

  _emitSemanticSelectionAtClientPoint(clientX, clientY) {
    const instances = this._semanticInstancesAtClientPoint(clientX, clientY);
    if (!instances.length) return false;
    window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-select', {
      detail: { instance: instances[0], instances, source: '3d' },
    }));
    return true;
  }

  _drawRoundedRect(ctx, x, y, width, height, radius) {
    const r = Math.min(radius, width / 2, height / 2);
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.lineTo(x + width - r, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + r);
    ctx.lineTo(x + width, y + height - r);
    ctx.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
    ctx.lineTo(x + r, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - r);
    ctx.lineTo(x, y + r);
    ctx.quadraticCurveTo(x, y, x + r, y);
    ctx.closePath();
  }

  _createSemanticLabelSprite(instance, fallback, z) {
    if (typeof document === 'undefined') return null;
    const name = String(instance.display_name || instance.label || '物品').trim() || '物品';
    const shortName = name.length > 12 ? `${name.slice(0, 11)}…` : name;
    const label = `${shortName} · z ${z.toFixed(2)}m`;
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    ctx.font = '600 24px system-ui, sans-serif';
    const textWidth = Math.ceil(ctx.measureText(label).width);
    canvas.width = Math.max(180, Math.min(420, textWidth + 34));
    canvas.height = 54;
    ctx.font = '600 24px system-ui, sans-serif';
    ctx.fillStyle = fallback ? 'rgba(51,65,85,.86)' : 'rgba(120,53,15,.9)';
    this._drawRoundedRect(ctx, 1, 1, canvas.width - 2, canvas.height - 2, 12);
    ctx.fill();
    ctx.strokeStyle = fallback ? '#cbd5e1' : '#fde68a';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, canvas.width / 2, canvas.height / 2 + 1);

    const texture = this._trackSemanticResource(new THREE.CanvasTexture(canvas));
    if ('colorSpace' in texture) texture.colorSpace = THREE.SRGBColorSpace;
    const material = this._trackSemanticResource(new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
    }));
    const sprite = new THREE.Sprite(material);
    const width = THREE.MathUtils.clamp(canvas.width / 150, 1.25, 2.8);
    sprite.scale.set(width, 0.36, 1);
    sprite.renderOrder = 20;
    return sprite;
  }

  _rebuildSemanticInstances() {
    if (!this._semanticGroup) return;
    const renderKey = JSON.stringify(this._semanticInstances.map(instance => [
      instance.instance_id, instance.pose, instance.display_name, instance.label,
      instance.frame_id, instance.is_fallback, instance.observation_only, instance.projection_method,
    ]));
    if (renderKey === this._semanticRenderKey) {
      // 观察次数、时间及置信度更新只替换选择数据，复用几何和标签纹理。
      const byId = new Map(this._semanticInstances.map(instance => [instance.instance_id, instance]));
      for (const child of this._semanticGroup.children) {
        if (child.userData.semanticInstance) child.userData.semanticInstance = byId.get(child.userData.semanticInstance.instance_id);
      }
      return;
    }
    this._clearSemanticInstances();
    this._semanticRenderKey = renderKey;
    this._syncSemanticGroupVisibility();
    for (const instance of this._semanticInstances) {
      const pose = instance?.pose;
      // 坐标必须来自完整地图位置，空值不能经 Number(null) 变成原点。
      if (!pose || ![pose.x, pose.y, pose.z].every(value => typeof value === 'number' && Number.isFinite(value))) continue;
      const { x, y, z } = pose;
      if (instance.observation_only || instance.projection_method === 'current_pose_fallback') continue;
      const fallback = Boolean(instance.is_fallback) || String(instance.frame_id || 'map') !== 'map';
      const marker = new THREE.Mesh(
        this._semanticMarkerGeometry,
        fallback ? this._semanticFallbackMaterial : this._semanticPreciseMaterial,
      );
      marker.position.set(x, y, z + 0.16);
      marker.userData.semanticInstance = instance;
      marker.userData.semanticMarker = true;
      marker.userData.semanticSelectable = true;
      this._semanticGroup.add(marker);

      if (z > 0.05) {
        const stemGeometry = this._trackSemanticResource(
          new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(x, y, 0.02),
            new THREE.Vector3(x, y, z),
          ]),
        );
        const stemMaterial = this._trackSemanticResource(new THREE.LineBasicMaterial({
          color: fallback ? COLORS.semanticFallback : COLORS.semantic,
          transparent: true,
          opacity: fallback ? 0.45 : 0.72,
        }));
        this._semanticGroup.add(new THREE.Line(stemGeometry, stemMaterial));
      }

      const label = this._createSemanticLabelSprite(instance, fallback, z);
      if (label) {
        label.position.set(x, y, z + 0.48);
        label.userData.semanticInstance = instance;
        label.userData.semanticSelectable = true;
        this._semanticGroup.add(label);
      }
    }
    this._requestRender();
  }

  updateSemanticInstances(instances) {
    this._semanticInstances = Array.isArray(instances) ? instances.slice() : [];
    this._rebuildSemanticInstances();
  }

  setSemanticInstancesVisible(visible) {
    this._semanticInstancesVisible = Boolean(visible);
    this._syncSemanticGroupVisibility();
    this._requestRender();
  }

  updateFrontiers(frontiers) {
    this._frontiers = Array.isArray(frontiers) ? frontiers.slice() : [];
    if (!this._frontierGroup) return;
    this._frontierGroup.traverse((object) => {
      object.geometry?.dispose?.();
      if (Array.isArray(object.material)) object.material.forEach((material) => material.dispose?.());
      else object.material?.dispose?.();
    });
    this._frontierGroup.clear();
    const geo = new THREE.SphereGeometry(0.15, 6, 6);
    const mat = new THREE.MeshBasicMaterial({ color: COLORS.frontier });
    for (const f of this._frontiers) {
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(Number(f.x || 0), Number(f.y || 0), Number(f.z || 0.1));
      this._frontierGroup.add(mesh);
    }
    this._requestRender();
  }

  updateTrajectory(trajectory) {
    this._trajectory = Array.isArray(trajectory) ? trajectory.slice() : [];
    if (!this._scene) return;
    const pointCount = this._trajectory.length;
    if (pointCount < 2) {
      if (this._trajectoryLine) {
        this._trajectoryLine.visible = false;
        this._trajectoryLine.geometry.setDrawRange(0, 0);
        this._requestRender();
      }
      return;
    }
    let position = this._trajectoryLine?.geometry.getAttribute('position');
    if (!position || position.count < pointCount) {
      // 只在容量不足时扩容并释放旧几何，材质始终复用，避免逐帧销毁着色程序。
      const capacity = Math.max(32, pointCount, (position?.count || 0) * 2);
      position = new THREE.BufferAttribute(new Float32Array(capacity * 3), 3);
      position.setUsage(THREE.DynamicDrawUsage);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', position);
      if (this._trajectoryLine) {
        this._trajectoryLine.geometry.dispose();
        this._trajectoryLine.geometry = geometry;
      } else {
        this._trajectoryLine = new THREE.Line(
          geometry,
          new THREE.LineBasicMaterial({ color: COLORS.trajectory }),
        );
        this._scene.add(this._trajectoryLine);
      }
    }
    for (let index = 0; index < pointCount; index += 1) {
      const point = this._trajectory[index];
      position.setXYZ(index, Number(point.x || 0), Number(point.y || 0), Number(point.z || 0.05));
    }
    position.clearUpdateRanges();
    position.addUpdateRange(0, pointCount * 3);
    position.needsUpdate = true;
    this._trajectoryLine.geometry.setDrawRange(0, pointCount);
    this._trajectoryLine.geometry.computeBoundingSphere();
    this._trajectoryLine.visible = true;
    this._requestRender();
  }

  updateNavigationPath(path, goalPose) {
    const samePath = !Array.isArray(path) || (path.length === this._navigationPath.length && path.every((p, i) => {
      const old = this._navigationPath[i]; return p.x === old.x && p.y === old.y && p.z === old.z;
    }));
    const previousGoal = this._navigationGoal;
    const sameGoal = goalPose === undefined || (!goalPose && !previousGoal)
      || (goalPose && previousGoal && ['x', 'y', 'z', 'yaw'].every(key => goalPose[key] === previousGoal[key]));
    if (samePath && sameGoal && this._scene && (this._navPathLine || this._navigationPath.length < 2)
      && (this._navigationGoal ? this._navGoalMesh : !this._navGoalMesh)) return;
    if (Array.isArray(path)) this._navigationPath = path.slice();
    if (goalPose !== undefined) this._navigationGoal = goalPose || null;
    if (!this._scene) return;
    if (this._navPathLine) {
      this._scene.remove(this._navPathLine);
      this._navPathLine.geometry.dispose();
      this._navPathLine.material.dispose();
      this._navPathLine = null;
    }
    if (this._navGoalMesh) {
      this._scene.remove(this._navGoalMesh);
      this._navGoalMesh.geometry.dispose();
      this._navGoalMesh.material.dispose();
      this._navGoalMesh = null;
    }
    if (this._navigationPath.length >= 2) {
      const pts = this._navigationPath.map(p => new THREE.Vector3(
        Number(p.x || 0),
        Number(p.y || 0),
        Number.isFinite(p.z) ? p.z : 0.08,
      ));
      const curve = new THREE.CurvePath();
      for (let i = 1; i < pts.length; i++) if (pts[i].distanceToSquared(pts[i - 1]) > 1e-12) {
        curve.add(new THREE.LineCurve3(pts[i - 1], pts[i]));
      }
      // 实线带宽以地图米为单位；独立显示层避免被密集点云遮没。
      const geo = curve.curves.length ? new THREE.TubeGeometry(curve, Math.min(4096, Math.max(8, pts.length * 4)), 0.025, 6, false)
        : new THREE.BufferGeometry();
      const mat = new THREE.MeshBasicMaterial({ color: COLORS.navPath, depthTest: false });
      this._navPathLine = new THREE.Mesh(geo, mat);
      this._navPathLine.renderOrder = 10;
      this._scene.add(this._navPathLine);
    }
    if (this._navigationGoal) {
      const goal = this._navigationGoal;
      const oriented = Number.isFinite(goal.yaw);
      const geo = oriented ? new THREE.ConeGeometry(0.25, 0.6, 8) : new THREE.SphereGeometry(0.12, 16, 12);
      if (oriented) geo.rotateZ(-Math.PI / 2);
      const mat = new THREE.MeshBasicMaterial({ color: COLORS.navGoal });
      this._navGoalMesh = new THREE.Mesh(geo, mat);
      this._navGoalMesh.position.set(Number(goal.x || 0), Number(goal.y || 0), Number(goal.z || 0.3));
      if (goal.yaw !== undefined) this._navGoalMesh.rotation.z = Number(goal.yaw || 0);
      this._scene.add(this._navGoalMesh);
    }
    this._requestRender();
  }

  _clearForbiddenZones() {
    if (!this._forbiddenZoneGroup) return;
    const geometries = new Set();
    const materials = new Set();
    this._forbiddenZoneGroup.traverse((object) => {
      if (object.geometry) geometries.add(object.geometry);
      if (Array.isArray(object.material)) object.material.forEach((material) => materials.add(material));
      else if (object.material) materials.add(object.material);
    });
    this._forbiddenZoneGroup.clear();
    geometries.forEach((geometry) => geometry.dispose?.());
    materials.forEach((material) => material.dispose?.());
  }

  _forbiddenVertices(zone) {
    if (zone.shape === 'rectangle') {
      const first = zone.vertices[0];
      const second = zone.vertices[1];
      return [
        { x: first.x, y: first.y },
        { x: second.x, y: first.y },
        { x: second.x, y: second.y },
        { x: first.x, y: second.y },
      ];
    }
    if (zone.shape === 'circle') {
      const vertices = [];
      for (let index = 0; index < FORBIDDEN_CIRCLE_SEGMENTS; index += 1) {
        const angle = Math.PI * 2 * index / FORBIDDEN_CIRCLE_SEGMENTS;
        vertices.push({
          x: zone.center.x + Math.cos(angle) * zone.radius_m,
          y: zone.center.y + Math.sin(angle) * zone.radius_m,
        });
      }
      return vertices;
    }
    return zone.vertices.slice();
  }

  _forbiddenMaskBounds(zones) {
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (const zone of zones) {
      for (const point of this._forbiddenVertices(zone)) {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minY = Math.min(minY, point.y);
        maxY = Math.max(maxY, point.y);
      }
    }
    const cloud = this._pointCloudBounds;
    if (cloud && Number.isFinite(cloud.radius) && cloud.radius > 0) {
      minX = Math.min(minX, cloud.center.x - cloud.radius);
      maxX = Math.max(maxX, cloud.center.x + cloud.radius);
      minY = Math.min(minY, cloud.center.y - cloud.radius);
      maxY = Math.max(maxY, cloud.center.y + cloud.radius);
    }
    if (![minX, maxX, minY, maxY].every(Number.isFinite)) {
      minX = -20;
      maxX = 20;
      minY = -20;
      maxY = 20;
    }
    const span = Math.max(maxX - minX, maxY - minY, 10);
    const padding = Math.max(30, span * 1.5);
    return {
      minX: minX - padding,
      maxX: maxX + padding,
      minY: minY - padding,
      maxY: maxY + padding,
    };
  }

  _forbiddenShapeFromVertices(vertices) {
    const shape = new THREE.Shape();
    shape.moveTo(vertices[0].x, vertices[0].y);
    for (let index = 1; index < vertices.length; index += 1) {
      shape.lineTo(vertices[index].x, vertices[index].y);
    }
    shape.closePath();
    return shape;
  }

  _forbiddenHole(zone) {
    const hole = new THREE.Path();
    if (zone.shape === 'circle') {
      hole.absarc(zone.center.x, zone.center.y, zone.radius_m, 0, Math.PI * 2, true);
      return hole;
    }
    const vertices = this._forbiddenVertices(zone);
    hole.moveTo(vertices[0].x, vertices[0].y);
    for (let index = 1; index < vertices.length; index += 1) hole.lineTo(vertices[index].x, vertices[index].y);
    hole.closePath();
    return hole;
  }

  _addForbiddenSurface(zone, maskBounds) {
    if (!this._forbiddenZoneGroup || zone.enabled === false || zone.shape === 'line') return;
    const outside = zone.mode === 'outside';
    let geometry;
    let position = null;
    if (outside) {
      const outer = this._forbiddenShapeFromVertices([
        { x: maskBounds.minX, y: maskBounds.minY },
        { x: maskBounds.maxX, y: maskBounds.minY },
        { x: maskBounds.maxX, y: maskBounds.maxY },
        { x: maskBounds.minX, y: maskBounds.maxY },
      ]);
      outer.holes.push(this._forbiddenHole(zone));
      geometry = new THREE.ShapeGeometry(outer);
    } else if (zone.shape === 'circle') {
      geometry = new THREE.CircleGeometry(zone.radius_m, FORBIDDEN_CIRCLE_SEGMENTS);
      position = zone.center;
    } else {
      geometry = new THREE.ShapeGeometry(this._forbiddenShapeFromVertices(this._forbiddenVertices(zone)));
    }
    const material = new THREE.MeshBasicMaterial({
      color: outside ? COLORS.forbiddenOutside : COLORS.forbiddenInside,
      transparent: true,
      opacity: outside ? 0.18 : 0.28,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = `forbidden-zone-surface-${zone.id || zone.shape}`;
    mesh.userData.forbiddenZoneId = zone.id;
    mesh.userData.forbiddenZoneMode = zone.mode;
    mesh.position.set(position?.x || 0, position?.y || 0, outside ? 0.012 : 0.018);
    mesh.renderOrder = outside ? 1 : 2;
    this._forbiddenZoneGroup.add(mesh);
  }

  _addForbiddenBoundary(zone) {
    if (!this._forbiddenZoneGroup) return;
    const vertices = this._forbiddenVertices(zone);
    const closed = zone.shape !== 'line';
    const edgeCount = closed ? vertices.length : vertices.length - 1;
    const width = THREE.MathUtils.clamp(Number(zone.stroke_width_m) || 0.25, 0.02, 5);
    const height = Math.max(0.22, Math.min(0.7, width * 1.5));
    const material = new THREE.MeshBasicMaterial({
      color: zone.enabled === false
        ? COLORS.forbiddenDisabled
        : zone.mode === 'outside' ? COLORS.forbiddenOutside : COLORS.forbiddenInside,
      transparent: true,
      opacity: zone.enabled === false ? 0.38 : 0.82,
      depthWrite: false,
    });
    for (let index = 0; index < edgeCount; index += 1) {
      const start = vertices[index];
      const end = vertices[(index + 1) % vertices.length];
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const length = Math.hypot(dx, dy);
      if (!Number.isFinite(length) || length <= 1e-6) continue;
      const geometry = new THREE.BoxGeometry(length, width, height);
      const wall = new THREE.Mesh(geometry, material);
      wall.name = `forbidden-zone-boundary-${zone.id || zone.shape}-${index}`;
      wall.userData.forbiddenZoneId = zone.id;
      wall.position.set((start.x + end.x) / 2, (start.y + end.y) / 2, height / 2);
      wall.rotation.z = Math.atan2(dy, dx);
      wall.renderOrder = 3;
      this._forbiddenZoneGroup.add(wall);
    }
  }

  _rebuildForbiddenZones() {
    if (!this._forbiddenZoneGroup) return;
    this._clearForbiddenZones();
    const zones = this._forbiddenZones;
    const maskBounds = this._forbiddenMaskBounds(zones);
    for (const zone of zones.filter((item) => item.mode === 'outside')) {
      this._addForbiddenSurface(zone, maskBounds);
      this._addForbiddenBoundary(zone);
    }
    for (const zone of zones.filter((item) => item.mode !== 'outside')) {
      this._addForbiddenSurface(zone, maskBounds);
      this._addForbiddenBoundary(zone);
    }
    this._requestRender();
  }

  updateForbiddenZones(envelope, allowMapSwitch = false) {
    const preferredMapName = allowMapSwitch ? null : this._forbiddenZoneMapName;
    const entry = forbiddenZoneEntry(envelope, preferredMapName);
    if (!entry) return false;
    if (
      this._forbiddenZoneMapName
      && entry.mapName
      && entry.mapName !== this._forbiddenZoneMapName
      && !allowMapSwitch
    ) return false;
    if (
      entry.mapName === this._forbiddenZoneMapName
      && entry.generation <= this._forbiddenZoneGeneration
    ) return false;
    this._forbiddenZoneMapName = entry.mapName || this._forbiddenZoneMapName;
    this._forbiddenZoneGeneration = entry.generation;
    this._forbiddenZones = entry.zones.slice();
    this._rebuildForbiddenZones();
    return true;
  }

  _applyCachedSceneState() {
    if (this._hasPointCloudSnapshot || this._pointStyle === 'full') this._rebuildPointCloud();
    if (this._robotPose) {
      const { x, y, z, yaw, quaternion } = this._robotPose;
      this.setRobotPose(x, y, z, yaw, quaternion);
    }
    this.updateFrontiers(this._frontiers);
    this.updateTrajectory(this._trajectory);
    this.updateNavigationPath(this._navigationPath, this._navigationGoal);
    this._rebuildForbiddenZones();
    this._rebuildSemanticInstances();
    this.setSemanticInstancesVisible(this._semanticInstancesVisible);
    if (this._semanticRegionData) this.updateSemanticRegions(this._semanticRegionData);
    if (this._sceneGraphData) this.updateSceneGraph(this._sceneGraphData);
  }

  updateSceneGraph(detail) {
    this._sceneGraphData = detail;
    this._sceneGraphLayer?.update(detail);
    if (this._container) {
      this._container.dataset.sceneGraphNodeCount = String(this._sceneGraphLayer?.nodes.length || 0);
      this._container.dataset.sceneGraphVisible = String(detail?.visible === true);
    }
    this._requestRender();
  }

  updateSemanticRegions(detail) {
    this._semanticRegionData = detail;
    this._semanticRegionLayer?.update(detail);
    if (this._container) {
      this._container.dataset.semanticRegionCount = String(this._semanticRegionLayer?.entries.length || 0);
      this._container.dataset.semanticRegionsVisible = String(detail?.visible === true);
    }
    this._requestRender();
  }

  focusSemanticRegion(id, boundary = null) {
    let box=this._semanticRegionLayer?.bounds(id);
    const points=boundary?.polygon_points;
    if(!box && points?.length && points.every(p=>[p.x,p.y,p.z].every(Number.isFinite))) {
      box=new THREE.Box3().setFromPoints(points.map(p=>new THREE.Vector3(p.x,p.y,p.z)));
    }
    if (!box || !this._camera || !this._controls) return;
    const center=box.getCenter(new THREE.Vector3());
    const size=box.getSize(new THREE.Vector3());
    const distance=Math.max(5,Math.max(size.x,size.y)*1.3);
    const direction=this._camera.position.clone().sub(this._controls.target).normalize();
    this._controls.target.copy(center);this._camera.position.copy(center).addScaledVector(direction,distance);
    this._hasUserCameraInput=true;this._cameraMode='free';this._controls.update();this._emitCameraState();this._requestRender();
  }

  setLOD(level) {
    if (!(level in LOD_STRIDE)) return;
    this._lod = level;
    this._rebuildPointCloud();
  }

  applyDisplayConfig(value) {
    POINTCLOUD3D_CONFIG = normalizePointCloud3DConfig(value);
    POINT_STYLE_PRESETS = buildPointStylePresets();
    HEIGHT_COLOR_STOPS = POINTCLOUD3D_CONFIG.heightColors.map(pointCloudHexToRgb);
    FLOOR_COLORS = {
      ground: pointCloudHexToRgb(POINTCLOUD3D_CONFIG.groundColor),
      context: pointCloudHexToRgb(POINTCLOUD3D_CONFIG.color),
      structure: pointCloudHexToRgb(POINTCLOUD3D_CONFIG.structureColor),
      object: pointCloudHexToRgb(POINTCLOUD3D_CONFIG.objectColor),
    };
    this._displayConfig = POINTCLOUD3D_CONFIG;
    this._processingConfig = { stylePresets: POINT_STYLE_PRESETS, heightColorStops: HEIGHT_COLOR_STOPS, floorColors: FLOOR_COLORS };
    // 旧工作线程结果不能覆盖新样式，保留点云和相机，只重新计算显示数据。
    this._invalidatePointCloudWork();
    this._cloudContextKey = null;
    if (this._denseCloud) this._denseCloud.material.size = POINT_STYLE_PRESETS.full.size;
    this._rebuildPointCloud();
    this._requestRender();
  }

  setPointStyle(style) {
    if (!Object.prototype.hasOwnProperty.call(POINT_STYLE_PRESETS, style)) return false;
    const changed = this._pointStyle !== style;
    this._pointStyle = style;
    if (style !== 'full') {
      this._denseCloud?.stop();
      window.dispatchEvent(new CustomEvent('pointcloud3d:dense-state', { detail: { enabled: false } }));
    }
    if (changed) this._rebuildPointCloud();
    const preset = POINT_STYLE_PRESETS[style];
    window.dispatchEvent(new CustomEvent('pointcloud3d:style-state', {
      detail: {
        style,
        label: preset.label,
        points: this._displayedPointCount,
      },
    }));
    this._requestRender();
    return true;
  }

  dispose() {
    this._assetCloudUrl = null;
    this._assetCloudSourceKey = null;
    this._assetCloudLoadingUrl = null;
    this._cloudDisposed = true;
    this._denseCloud?.dispose();
    this._denseCloud = null;
    this.previewNavigationPoint(null);
    this._cloudLoadToken += 1;
    this._invalidatePointCloudWork();
    this._cloudWorker?.terminate();
    this._cloudWorker = null;
    this._cloudRunning = null;
    if (this._rafId) cancelAnimationFrame(this._rafId);
    const canvas = this._renderer?.domElement;
    canvas?.removeEventListener?.('pointerdown', this._semanticPointerDownHandler);
    canvas?.removeEventListener?.('pointerup', this._semanticPointerUpHandler);
    canvas?.removeEventListener?.('pointercancel', this._semanticPointerCancelHandler);
    this._resizeObserver?.disconnect();
    this._resizeObserver = null;
    this._disposePointsMesh();
    this._clearForbiddenZones();
    this._clearSemanticInstances();
    this._semanticRegionLayer?.dispose();
    this._semanticRegionLayer = null;
    this._semanticRegionData = null;
    this._sceneGraphLayer?.dispose();
    this._sceneGraphLayer = null;
    this._sceneGraphData = null;
    this._semanticMarkerGeometry?.dispose();
    this._semanticPreciseMaterial?.dispose();
    this._semanticFallbackMaterial?.dispose();
    this._semanticMarkerGeometry = null;
    this._semanticPreciseMaterial = null;
    this._semanticFallbackMaterial = null;
    if (this._robotMesh) {
      for (const resource of this._robotMesh.userData.disposableResources || []) resource.dispose?.();
    }
    this._frontierGroup?.traverse((object) => {
      object.geometry?.dispose?.();
      if (Array.isArray(object.material)) object.material.forEach((material) => material.dispose?.());
      else object.material?.dispose?.();
    });
    for (const object of [this._trajectoryLine, this._navPathLine, this._navGoalMesh]) {
      object?.geometry?.dispose?.();
      if (Array.isArray(object?.material)) object.material.forEach((material) => material.dispose?.());
      else object?.material?.dispose?.();
    }
    this._controls?.dispose();
    this._renderer?.dispose();
    if (canvas?.parentNode) canvas.parentNode.removeChild(canvas);
    this._scene?.clear();
    this._renderer = null;
    this._scene = null;
    this._camera = null;
    this._controls = null;
    this._robotMesh = null;
    this._robotGeometryCenter = null;
    this._frontierGroup = null;
    this._forbiddenZoneGroup = null;
    this._forbiddenZones = [];
    this._forbiddenZoneMapName = null;
    this._forbiddenZoneGeneration = 0;
    this._semanticGroup = null;
    this._trajectoryLine = null;
    this._navPathLine = null;
    this._navGoalMesh = null;
    this._semanticPointerDown = null;
    this._semanticPointerDownHandler = null;
    this._semanticPointerUpHandler = null;
    this._semanticPointerCancelHandler = null;
    this._semanticRaycaster = null;
    this._semanticPointer = null;
    this._semanticZoomHidden = false;
    this._pointCloudMapWatermark = null;
    this._explorationOverlayWatermark = null;
    this._explorationFrontierRenderedForWatermark = false;
    this._explorationOverlayInactive = false;
    this._explorationOverlayRequiresCanonical = false;
    this._rafId = null;
    this._hasFramedPointCloud = false;
    this._hasUserCameraInput = false;
    this._initialized = false;
  }
}

window.pointcloud3d = new PointCloud3DViewer();

function pointFromValue(value) {
  const pose = value?.target_pose ?? value?.pose ?? value?.geometry?.centroid ?? value?.centroid ?? value;
  const position = pose?.position ?? pose;
  if (position?.x === undefined || position?.y === undefined) return null;
  return {
    x: Number(position.x),
    y: Number(position.y),
    z: Number(position.z || 0),
  };
}

function firstDefinedValue(...values) {
  for (const value of values) {
    if (value !== undefined) return value;
  }
  return undefined;
}

function firstAvailableValue(...values) {
  for (const value of values) {
    if (value !== undefined && value !== null) return value;
  }
  return values.some(value => value === null) ? null : undefined;
}

function yawFromValue(value) {
  const pose = value?.target_pose ?? value?.pose ?? value;
  if (pose?.yaw !== undefined) return Number(pose.yaw || 0);
  const q = pose?.orientation;
  if (!q || value?.require_goal_orientation === false) return undefined;
  const siny = 2 * ((Number(q.w) || 0) * (Number(q.z) || 0) + (Number(q.x) || 0) * (Number(q.y) || 0));
  const cosy = 1 - 2 * ((Number(q.y) || 0) * (Number(q.y) || 0) + (Number(q.z) || 0) * (Number(q.z) || 0));
  return Math.atan2(siny, cosy);
}

window.addEventListener('stream:exploration.frontier', (e) => {
  const data = e.detail;
  let identity = null;
  try {
    identity = parseExplorationOverlayIdentity(data);
  } catch (_) {
    return;
  }
  const current = window.pointcloud3d._explorationOverlayWatermark;
  if (
    identity === null
    && (
      data?.canonical_exploration === true
      || window.pointcloud3d._explorationOverlayRequiresCanonical
    )
  ) return;
  if (current && identity === null) return;
  if (current && identity) {
    const comparison = compareExplorationOverlayIdentity(current, identity);
    if (comparison < 0) return;
    if (
      comparison === 0
      && window.pointcloud3d._explorationFrontierRenderedForWatermark
    ) return;
  }
  if (identity) {
    window.pointcloud3d._explorationOverlayWatermark = identity;
    window.pointcloud3d._explorationFrontierRenderedForWatermark = true;
    window.pointcloud3d._explorationOverlayRequiresCanonical = false;
  }
  window.pointcloud3d._explorationOverlayInactive = explorationOverlayIsInactive(data);
  if (window.pointcloud3d._explorationOverlayInactive || data?.frontier_count === 0) {
    window.pointcloud3d.updateFrontiers([]);
    if (data?.navigation_is_public !== true) window.pointcloud3d.updateNavigationPath([], null);
    return;
  }
  const rawFrontiers = Array.isArray(data) ? data
    : data?.latest_frontiers ?? data?.frontiers ?? [];
  const frontiers = rawFrontiers.map(pointFromValue).filter(Boolean);
  window.pointcloud3d.updateFrontiers(frontiers);
  const navigationCorrelated = !identity
    || navigationEventMatchesExploration(data, identity);
  const rawPath = navigationCorrelated
    ? firstDefinedValue(data?.planned_path, data?.nav_path, data?.path)
    : null;
  const path = rawPath === null ? []
    : Array.isArray(rawPath) ? rawPath.map(pointFromValue).filter(Boolean)
      : undefined;
  const rawGoal = navigationCorrelated ? firstAvailableValue(
    data?.current_goal,
    data?.selected_frontier,
    data?.current_frontier,
    data?.metadata?.current_frontier,
  ) : null;
  const goalPoint = pointFromValue(rawGoal);
  const goal = rawGoal === undefined ? undefined
    : goalPoint ? { ...goalPoint, yaw: yawFromValue(rawGoal) }
      : null;
  if (path !== undefined || goal !== undefined) {
    window.pointcloud3d.updateNavigationPath(path, goal);
  }
});

window.addEventListener('stream:exploration.overlay-reset', () => {
  window.pointcloud3d._explorationOverlayWatermark = null;
  window.pointcloud3d._explorationFrontierRenderedForWatermark = false;
  window.pointcloud3d._explorationOverlayInactive = false;
  window.pointcloud3d._explorationOverlayRequiresCanonical = true;
  window.pointcloud3d.updateFrontiers([]);
  window.pointcloud3d.updateNavigationPath([], null);
});

window.addEventListener('stream:localization.pose', (e) => {
  const d = e.detail;
  const pos = d?.position;
  if (!pos || pos.x === undefined) {
    window.pointcloud3d.clearRobotPose();
    return;
  }
  window.pointcloud3d._robotPoseSource = d;
  window.pointcloud3d.setRobotPose(
    Number(pos.x),
    Number(pos.y),
    Number(pos.z || 0),
    yawFromValue(d),
    d.orientation,
  );
});

window.addEventListener('stream:localization.trajectory', (e) => {
  const d = e.detail;
  const rawTrajectory = Array.isArray(d) ? d : d?.trajectory;
  const traj = Array.isArray(rawTrajectory) ? rawTrajectory.map(pointFromValue).filter(Boolean) : null;
  if (traj) window.pointcloud3d.updateTrajectory(traj);
});

function updateNavigationOverlayFromEvent(e) {
  const d = e.detail;
  const publicNavigation = d?.navigation_is_public === true;
  let explorationIdentity = window.pointcloud3d._explorationOverlayWatermark;
  if (e.type === 'stream:exploration.state') {
    let identity;
    try {
      identity = parseExplorationOverlayIdentity(d);
    } catch (_) {
      return;
    }
    if (identity === null) {
      if (
        d?.canonical_exploration === true
        || explorationIdentity
        || window.pointcloud3d._explorationOverlayRequiresCanonical
      ) return;
    } else {
      if (!shouldAcceptExplorationOverlay(explorationIdentity, identity)) return;
      window.pointcloud3d._explorationOverlayWatermark = identity;
      window.pointcloud3d._explorationFrontierRenderedForWatermark = false;
      window.pointcloud3d._explorationOverlayRequiresCanonical = false;
      explorationIdentity = identity;
      window.pointcloud3d._explorationOverlayInactive = explorationOverlayIsInactive(d);
      if (window.pointcloud3d._explorationOverlayInactive) {
        window.pointcloud3d.updateFrontiers([]);
        if (!publicNavigation) window.pointcloud3d.updateNavigationPath([], null);
        if (!publicNavigation) return;
      }
      if (d?.frontier_count === 0) {
        window.pointcloud3d.updateFrontiers([]);
        if (!publicNavigation) window.pointcloud3d.updateNavigationPath([], null);
        if (!publicNavigation) return;
      }
    }
  } else {
    if (
      !publicNavigation && explorationIdentity
      && !navigationEventMatchesExploration(d, explorationIdentity)
    ) return;
    if (
      !publicNavigation && (window.pointcloud3d._explorationOverlayRequiresCanonical
      || window.pointcloud3d._explorationOverlayInactive)
    ) {
      window.pointcloud3d.updateNavigationPath([], null);
      return;
    }
  }
  const navigationCorrelated = publicNavigation || !explorationIdentity
    || navigationEventMatchesExploration(d, explorationIdentity);
  const rawPath = navigationCorrelated
    ? firstDefinedValue(d?.planned_path, d?.nav_path, d?.path)
    : null;
  const path = rawPath === null ? []
    : Array.isArray(rawPath) ? rawPath.map(pointFromValue).filter(Boolean)
      : undefined;
  const rawGoal = navigationCorrelated
    ? firstAvailableValue(d?.current_goal, d?.goal_pose, d?.goal)
    : null;
  const goalPoint = pointFromValue(rawGoal);
  const goal = rawGoal === undefined ? undefined
    : goalPoint ? { ...goalPoint, yaw: yawFromValue(rawGoal) }
      : null;
  if (path !== undefined || goal !== undefined) {
    window.pointcloud3d.updateNavigationPath(path, goal);
  }
}

window.addEventListener('stream:navigation.state', updateNavigationOverlayFromEvent);
window.addEventListener('stream:exploration.state', updateNavigationOverlayFromEvent);

window.addEventListener('pointcloud3d:semantic-instances', (e) => {
  const detail = e.detail;
  const instances = Array.isArray(detail) ? detail : detail?.instances;
  window.pointcloud3d.updateSemanticInstances(instances || []);
});

window.addEventListener('pointcloud3d:semantic-regions', (event) => {
  window.pointcloud3d.updateSemanticRegions(event.detail || {});
});

window.addEventListener('pointcloud3d:scene-graph', (event) => {
  window.pointcloud3d.updateSceneGraph(event.detail || {});
});

window.addEventListener('pointcloud3d:semantic-visibility', (e) => {
  const detail = e.detail;
  const visible = typeof detail === 'boolean' ? detail : detail?.visible;
  window.pointcloud3d.setSemanticInstancesVisible(visible !== false);
});

window.addEventListener('forbidden-zones:updated', (e) => {
  window.pointcloud3d.updateForbiddenZones(e.detail || {}, true);
});

window.addEventListener('stream:navigation.forbidden_zones', (e) => {
  window.pointcloud3d.updateForbiddenZones(e.detail || {});
});

window.addEventListener('stream:pointcloud_map.meta', (e) => {
  const d = e.detail;
  if (d?.url) window.pointcloud3d.loadPointCloud(d.url, d.lod ?? 'preview');
});

// 统一实时/保存点云 binary 信封接收
window.addEventListener('stream:pointcloud_map.binary', (e) => {
  if (!window.pointcloud3d || window.pointcloud3d._cloudDisposed) return;
  const buffer = e.detail?.buffer;
  let envelope;
  try {
    envelope = parsePointCloudMapBinaryEnvelope(buffer);
  } catch (error) {
    window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', {
      detail: { pointcloudParseError: error.message, pointcloudDecision: 'parse_error' },
    }));
    window.dispatchEvent(new CustomEvent('pointcloud3d:status', {
      detail: { state: 'error', message: error.message },
    }));
    return;
  }
  // 完整信封通过严格解析后才确认 transport cursor；语义旧帧仍可确认传输进度。
  window.dispatchEvent(new CustomEvent('stream:pointcloud_map.cursor-accepted', {
    detail: { cursor: envelope.header.cursor },
  }));
  const current = window.pointcloud3d._pointCloudMapWatermark;
  window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', {
    detail: { pointcloudParseError: '', pointcloudCandidateRevision: envelope.header.revision, pointcloudCandidateGeneration: String(envelope.streamGeneration), pointcloudCandidateSourceEpoch: envelope.header.source_epoch },
  }));
  const sourceKey = JSON.stringify([envelope.header.source, envelope.header.source_epoch, envelope.header.frame_id, String(envelope.streamGeneration)]);
  if (!shouldAcceptPointCloudMapEnvelope(current, envelope)) {
    if (current && current.streamGeneration === envelope.streamGeneration
        && current.header.source === envelope.header.source && current.header.source_epoch === envelope.header.source_epoch
        && current.header.frame_id === envelope.header.frame_id && current.header.revision === envelope.header.revision
        && current.revisionOrdinal === envelope.revisionOrdinal) {
      // 完整解析后的同内容重发不重计地图帧率；也不更新机器人位姿指标。
      window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', { detail: { pointcloudDecision: 'unchanged' } }));
    } else window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', { detail: { pointcloudDecision: 'rejected_identity' } }));
    return;
  }
  window.pointcloud3d._replaceCloudMapBuffer(envelope.xyz, envelope);
  window.dispatchEvent(new CustomEvent('pointcloud3d:diagnostic', {
    detail: { pointcloudDecision: 'accepted', pointcloudAcceptedPoints: envelope.header.point_count,
      pointcloudAcceptedRevision: envelope.header.revision, pointcloudAcceptedGeneration: String(envelope.streamGeneration),
      pointcloudAcceptedSourceEpoch: envelope.header.source_epoch },
  }));
  // 已接纳源快照的更新率；实际绘制另由content-drawn报告，不把工作线程完成冒充显示完成。
  window.dispatchEvent(new CustomEvent('pointcloud3d:map-frame', {
    detail: {
      source: envelope.header.source,
      sourceTime: Number(envelope.sourceTimestampNs / 1000000n),
      frameKey: `${envelope.streamGeneration}:${envelope.revisionOrdinal}:${envelope.header.revision}`,
      sourceKey,
    },
  }));
});

window.addEventListener('stream:pointcloud_map.reset', () => {
  if (!window.pointcloud3d) return;
  window.pointcloud3d._invalidatePointCloudWork();
  window.pointcloud3d._cloudLoadToken += 1;
  window.pointcloud3d._pointCloudMapWatermark = null;
  window.pointcloud3d._denseCloud?.reset();
});

// 二进制消费者已注册与三维画布已初始化是两个不同阶段。
window.dispatchEvent(new CustomEvent('pointcloud3d:consumer-ready'));

window.addEventListener('settings:display', event => {
  window.__NUWAX_POINTCLOUD3D_CONFIG__ = event.detail;
  window.pointcloud3d.applyDisplayConfig(event.detail);
});
