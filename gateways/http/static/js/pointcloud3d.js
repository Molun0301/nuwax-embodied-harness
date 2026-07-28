import * as THREE from './three.module.min.js';
import { OrbitControls } from './OrbitControls.js';

const COLORS = {
  robot: 0xfbbf24,
  robotAccent: 0x78350f,
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

const POINTCLOUD3D_CONFIG = normalizePointCloud3DConfig(
  typeof window === 'undefined' ? null : window.__NUWAX_POINTCLOUD3D_CONFIG__,
);
const POINT_STYLE_PRESETS = Object.freeze({
  height: Object.freeze({
    label: '高度',
    sampleStride: 2,
    liveSpacingM: POINTCLOUD3D_CONFIG.spacingM,
    heightGradientScale: 2,
    blueBandHeightM: 0.4,
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
});
const HEIGHT_COLOR_STOPS = Object.freeze(
  POINTCLOUD3D_CONFIG.heightColors.map((color) => Object.freeze(pointCloudHexToRgb(color))),
);
const FLOOR_COLORS = Object.freeze({
  ground: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.groundColor)),
  context: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.color)),
  structure: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.structureColor)),
  object: Object.freeze(pointCloudHexToRgb(POINTCLOUD3D_CONFIG.objectColor)),
});
const FLOOR_BIN_SIZE_M = 0.08;
const FLOOR_BAND_HALF_M = 0.12;
const FLOOR_MIN_SEPARATION_M = 1.5;
const FLOOR_DEFAULT_STORY_HEIGHT_M = 2.8;
const FLOOR_CEILING_GAP_M = 0.2;
const FLOOR_ANALYSIS_MAX_POINTS = 200_000;
const STRUCTURE_COLUMN_SIZE_M = 0.18;
const STRUCTURE_MIN_VERTICAL_SPAN_M = 0.45;
const POINT_SAMPLING_MIN_SOURCE_POINTS = 4096;
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

class PointCloud3DViewer {
  constructor() {
    this._container = null;
    this._renderer = null;
    this._scene = null;
    this._camera = null;
    this._controls = null;
    this._pointsMesh = null;
    this._robotMesh = null;
    this._frontierGroup = null;
    this._forbiddenZoneGroup = null;
    this._semanticGroup = null;
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
    this._semanticInstancesVisible = true;
    this._semanticOwnedResources = [];
    this._semanticMarkerGeometry = null;
    this._semanticPreciseMaterial = null;
    this._semanticFallbackMaterial = null;
    this._lod = 'preview';
    this._pointStyle = DEFAULT_POINT_STYLE;
    this._displayConfig = POINTCLOUD3D_CONFIG;
    this._needsRender = false;
    this._rafId = null;
    this._lastFrameTime = 0;
    this._allPositions = new Float32Array(0);
    this._hasPointCloudSnapshot = false;
    this._isLiveCloudMapSnapshot = false;
    this._pointCloudSourcePoints = null;
    this._displayedPointCount = 0;
    this._initialized = false;
  }

  init(container) {
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
      this._emitSemanticSelectionAtClientPoint(x, y);
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
    this._needsRender = true;
    if (!this._renderer || !this._scene || !this._camera) return;
    if (!this._rafId) {
      this._rafId = requestAnimationFrame((t) => this._renderLoop(t));
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

  _renderLoop(timestamp) {
    this._rafId = null;
    const elapsed = timestamp - this._lastFrameTime;
    if (elapsed < 1000 / 30) {
      if (this._needsRender) {
        this._rafId = requestAnimationFrame((t) => this._renderLoop(t));
      }
      return;
    }
    this._lastFrameTime = timestamp;
    this._needsRender = false;
    const controlsChanged = Boolean(this._controls?.update());
    this._removeLegacySceneHelpers();
    this._syncSemanticGroupVisibility();
    this._renderer.render(this._scene, this._camera);
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
    return new THREE.Vector3(pose.x, pose.y, pose.z + FOLLOW_TARGET_HEIGHT);
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
    const distance = THREE.MathUtils.clamp(
      radius / Math.max(Math.sin(halfFov), 0.2) * 1.15,
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

  _measurePointCloud(positions) {
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let i = 0; i < positions.length; i += 3) {
      const x = positions[i];
      const y = positions[i + 1];
      const z = positions[i + 2];
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }
    const center = new THREE.Vector3(
      (minX + maxX) / 2,
      (minY + maxY) / 2,
      (minZ + maxZ) / 2,
    );
    const radius = Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2;
    return {
      xMin: minX,
      xMax: maxX,
      yMin: minY,
      yMax: maxY,
      zMin: minZ,
      zMax: maxZ,
      bounds: new THREE.Sphere(center, radius),
    };
  }

  _colorByHeight(positions, zMin, zMax, gradientScale = 1, blueBandHeightM = null) {
    const count = positions.length / 3;
    const colors = new Uint8Array(count * 3);
    if (count === 0) return colors;
    const range = zMax - zMin || 1;
    const scale = Number.isFinite(gradientScale) && gradientScale > 0 ? gradientScale : 1;
    const gradientStart = Number.isFinite(blueBandHeightM) && blueBandHeightM >= 0
      ? zMin + blueBandHeightM
      : zMin + range * (0.5 - 0.5 / scale);
    for (let i = 0; i < count; i++) {
      const t = THREE.MathUtils.clamp(
        (positions[i * 3 + 2] - gradientStart) * scale / range,
        0,
        1,
      );
      const scaled = t * (HEIGHT_COLOR_STOPS.length - 1);
      const lower = Math.min(Math.floor(scaled), HEIGHT_COLOR_STOPS.length - 2);
      const blend = scaled - lower;
      const from = HEIGHT_COLOR_STOPS[lower];
      const to = HEIGHT_COLOR_STOPS[lower + 1];
      colors[i * 3] = Math.round(from[0] + (to[0] - from[0]) * blend);
      colors[i * 3 + 1] = Math.round(from[1] + (to[1] - from[1]) * blend);
      colors[i * 3 + 2] = Math.round(from[2] + (to[2] - from[2]) * blend);
    }
    return colors;
  }

  _floorLevels(positions, measurement) {
    if (this._floorLevelSource === positions && this._floorLevelCache) {
      return this._floorLevelCache;
    }

    // A smoothed Z histogram is the browser-only fallback for XYZ clouds without classifications.
    const histogram = new Map();
    const pointCount = Math.floor(positions.length / 3);
    const analysisStride = Math.max(1, Math.ceil(pointCount / FLOOR_ANALYSIS_MAX_POINTS));
    for (let pointIndex = 0; pointIndex < pointCount; pointIndex += analysisStride) {
      const z = positions[pointIndex * 3 + 2];
      if (!Number.isFinite(z)) continue;
      const bin = Math.round(z / FLOOR_BIN_SIZE_M);
      histogram.set(bin, (histogram.get(bin) || 0) + 1);
    }

    const scores = new Map();
    let maxScore = 0;
    for (const [bin, count] of histogram) {
      const score = count
        + 0.65 * ((histogram.get(bin - 1) || 0) + (histogram.get(bin + 1) || 0))
        + 0.25 * ((histogram.get(bin - 2) || 0) + (histogram.get(bin + 2) || 0));
      scores.set(bin, score);
      if (score > maxScore) maxScore = score;
    }

    const minimumScore = Math.max(3, maxScore * 0.12);
    const peaks = [...scores.entries()]
      .filter(([bin, score]) => (
        score >= minimumScore
        && score >= (scores.get(bin - 1) || 0)
        && score >= (scores.get(bin + 1) || 0)
      ))
      .sort((left, right) => right[1] - left[1]);
    const selected = [];
    for (const [bin, score] of peaks) {
      let weightedZ = 0;
      let weight = 0;
      for (let neighbour = -1; neighbour <= 1; neighbour += 1) {
        const count = histogram.get(bin + neighbour) || 0;
        weightedZ += (bin + neighbour) * FLOOR_BIN_SIZE_M * count;
        weight += count;
      }
      const z = weight ? weightedZ / weight : bin * FLOOR_BIN_SIZE_M;
      if (selected.some((candidate) => Math.abs(candidate.z - z) < FLOOR_MIN_SEPARATION_M)) continue;
      selected.push({ z, score });
    }

    const levels = selected.length
      ? selected.map((candidate) => candidate.z).sort((left, right) => left - right)
      : [measurement.zMin];
    this._floorLevelSource = positions;
    this._floorLevelCache = Object.freeze(levels);
    return this._floorLevelCache;
  }

  _selectFloorLevel(levels, robotZ = this._robotPose?.z) {
    if (!levels.length) return null;
    const height = Number(robotZ);
    if (!Number.isFinite(height)) return levels[0];
    const belowRobot = levels.filter((level) => level <= height + 0.45);
    const candidates = belowRobot.length ? belowRobot : levels;
    return candidates.reduce((closest, level) => (
      Math.abs(level - height) < Math.abs(closest - height) ? level : closest
    ));
  }

  _activeFloorContext(positions, measurement) {
    const levels = this._floorLevels(positions, measurement);
    const floorZ = this._selectFloorLevel(levels);
    if (!Number.isFinite(floorZ)) return null;
    const floorIndex = levels.indexOf(floorZ);
    const nextFloorZ = floorIndex >= 0 ? levels[floorIndex + 1] : null;
    const fallbackTop = Math.min(
      measurement.zMax + FLOOR_BAND_HALF_M,
      floorZ + FLOOR_DEFAULT_STORY_HEIGHT_M,
    );
    const storyMax = Math.max(
      floorZ + FLOOR_BAND_HALF_M,
      Number.isFinite(nextFloorZ) ? nextFloorZ - FLOOR_CEILING_GAP_M : fallbackTop,
    );
    return {
      floorZ,
      levels,
      groundMin: floorZ - FLOOR_BAND_HALF_M,
      groundMax: floorZ + FLOOR_BAND_HALF_M,
      storyMax,
    };
  }

  _filterPointCloudForFloor(positions, floorContext, filterMode) {
    if (!floorContext) return positions;
    const upper = filterMode === 'ground' ? floorContext.groundMax : floorContext.storyMax;
    const filtered = new Float32Array(positions.length);
    let writeOffset = 0;
    for (let offset = 0; offset < positions.length; offset += 3) {
      const x = positions[offset];
      const y = positions[offset + 1];
      const z = positions[offset + 2];
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      if (z < floorContext.groundMin || z > upper) continue;
      filtered[writeOffset] = x;
      filtered[writeOffset + 1] = y;
      filtered[writeOffset + 2] = z;
      writeOffset += 3;
    }
    return filtered.subarray(0, writeOffset);
  }

  _pointColumnKey(x, y) {
    return `${Math.floor(x / STRUCTURE_COLUMN_SIZE_M)},${Math.floor(y / STRUCTURE_COLUMN_SIZE_M)}`;
  }

  _verticalStructureColumns(positions, floorContext) {
    const columns = new Map();
    for (let offset = 0; offset < positions.length; offset += 3) {
      const z = positions[offset + 2];
      if (z <= floorContext.groundMax) continue;
      const key = this._pointColumnKey(positions[offset], positions[offset + 1]);
      const span = columns.get(key);
      if (span) {
        if (z < span.min) span.min = z;
        if (z > span.max) span.max = z;
      } else {
        columns.set(key, { min: z, max: z });
      }
    }
    return new Set(
      [...columns.entries()]
        .filter(([, span]) => span.max - span.min >= STRUCTURE_MIN_VERTICAL_SPAN_M)
        .map(([key]) => key),
    );
  }

  _pointCloudColors(positions, measurement, floorContext = null) {
    const preset = POINT_STYLE_PRESETS[this._pointStyle] || POINT_STYLE_PRESETS[DEFAULT_POINT_STYLE];
    if (preset.colorMode === 'height') {
      return this._colorByHeight(
        positions,
        measurement.zMin,
        measurement.zMax,
        preset.heightGradientScale,
        preset.blueBandHeightM,
      );
    }

    const count = Math.floor(positions.length / 3);
    const colors = new Uint8Array(count * 3);
    const verticalColumns = preset.colorMode === 'structure' && floorContext
      ? this._verticalStructureColumns(positions, floorContext)
      : null;
    for (let i = 0; i < count; i++) {
      const offset = i * 3;
      let color = FLOOR_COLORS.context;
      if (preset.colorMode === 'mono') {
        color = FLOOR_COLORS.context;
      } else if (floorContext && positions[offset + 2] <= floorContext.groundMax) {
        color = FLOOR_COLORS.ground;
      } else if (preset.colorMode === 'structure' && floorContext) {
        const column = this._pointColumnKey(positions[offset], positions[offset + 1]);
        color = verticalColumns.has(column) ? FLOOR_COLORS.structure : FLOOR_COLORS.object;
      }
      colors[offset] = color[0];
      colors[offset + 1] = color[1];
      colors[offset + 2] = color[2];
    }
    return colors;
  }

  _disposePointsMesh() {
    if (!this._pointsMesh) return;
    this._scene?.remove(this._pointsMesh);
    this._pointsMesh.geometry.dispose();
    this._pointsMesh.material.dispose();
    this._pointsMesh = null;
  }

  _emitPointCount() {
    const bufferedPoints = Math.floor(this._allPositions.length / 3);
    const sourcePoints = Number.isFinite(this._pointCloudSourcePoints)
      ? Math.max(bufferedPoints, Math.floor(this._pointCloudSourcePoints))
      : bufferedPoints;
    window.dispatchEvent(new CustomEvent('pointcloud3d:point-count', {
      detail: {
        points: this._displayedPointCount,
        displayedPoints: this._displayedPointCount,
        sourcePoints,
        lod: this._lod,
      },
    }));
  }

  _renderPointCloudPositions(positions, sourceMeasurement = null, floorContext = null) {
    if (!this._scene) return;
    const previousBounds = this._pointCloudBounds;
    this._disposePointsMesh();
    if (!positions.length) {
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
      this._pointCloudColors(positions, measurement, floorContext),
      3,
      true,
    ));
    geo.boundingSphere = measurement.bounds.clone();
    this._pointCloudBounds = measurement.bounds.clone();
    const mat = new THREE.PointsMaterial({
      size: preset.size,
      vertexColors: true,
      sizeAttenuation: preset.sizeAttenuation,
    });
    this._pointsMesh = new THREE.Points(geo, mat);
    this._scene.add(this._pointsMesh);
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

  _rebuildPointCloud() {
    const preset = POINT_STYLE_PRESETS[this._pointStyle] || POINT_STYLE_PRESETS[DEFAULT_POINT_STYLE];
    const src = this._allPositions;
    if (this._floorLevelSource && this._floorLevelSource !== src) {
      this._floorLevelSource = null;
      this._floorLevelCache = null;
    }
    if (!src.length) {
      this._activeFloorZ = null;
      this._renderPointCloudPositions(new Float32Array(0));
      return;
    }
    const sourceMeasurement = this._measurePointCloud(src);
    const floorContext = preset.floorFilter
      ? this._activeFloorContext(src, sourceMeasurement)
      : null;
    const visibleSource = floorContext
      ? this._filterPointCloudForFloor(src, floorContext, preset.floorFilter)
      : src;
    const measurement = visibleSource.length ? this._measurePointCloud(visibleSource) : null;
    this._activeFloorZ = floorContext?.floorZ ?? null;
    if (this._isLiveCloudMapSnapshot) {
      const positions = preset.liveSpacingM
        ? this._spatiallySamplePointCloud(visibleSource, preset.liveSpacingM)
        : visibleSource;
      this._renderPointCloudPositions(positions, measurement, floorContext);
      return;
    }
    const totalPts = Math.floor(visibleSource.length / 3);
    const configuredStride = Math.max(1, Math.round((LOD_STRIDE[this._lod] || 1) * preset.sampleStride));
    const stride = totalPts <= POINT_SAMPLING_MIN_SOURCE_POINTS ? 1 : configuredStride;
    const sampledCount = Math.min(Math.ceil(totalPts / stride), MAX_POINTS);
    if (sampledCount === 0) {
      this._renderPointCloudPositions(new Float32Array(0));
      return;
    }

    const pos = new Float32Array(sampledCount * 3);
    for (let i = 0, j = 0; i < sampledCount; i++, j += stride) {
      pos[i * 3] = visibleSource[j * 3];
      pos[i * 3 + 1] = visibleSource[j * 3 + 1];
      pos[i * 3 + 2] = visibleSource[j * 3 + 2];
    }
    this._renderPointCloudPositions(pos, measurement, floorContext);
  }

  _spatiallySamplePointCloud(positions, spacingM) {
    const spacing = Number(spacingM);
    if (!positions.length || !Number.isFinite(spacing) || spacing <= 0) return positions;
    const inverseSpacing = 1 / spacing;
    const occupied = new Set();
    const sampled = new Float32Array(positions.length);
    let writeOffset = 0;
    for (let offset = 0; offset < positions.length; offset += 3) {
      const x = positions[offset];
      const y = positions[offset + 1];
      const z = positions[offset + 2];
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      const gridX = Math.floor(x * inverseSpacing);
      const gridY = Math.floor(y * inverseSpacing);
      const gridZ = Math.floor(z * inverseSpacing);
      const key = (
        Math.imul(gridX, 73856093)
        ^ Math.imul(gridY, 19349663)
        ^ Math.imul(gridZ, 83492791)
      ) >>> 0;
      if (occupied.has(key)) continue;
      occupied.add(key);
      sampled[writeOffset] = (gridX + 0.5) * spacing;
      sampled[writeOffset + 1] = (gridY + 0.5) * spacing;
      sampled[writeOffset + 2] = (gridZ + 0.5) * spacing;
      writeOffset += 3;
    }
    return sampled.slice(0, writeOffset);
  }

  async loadPointCloud(url, lod = 'preview') {
    this._lod = lod;
    try {
      const resp = await fetch(url);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const buf = await resp.arrayBuffer();
      this._allPositions = new Float32Array(buf);
      this._hasPointCloudSnapshot = true;
      this._isLiveCloudMapSnapshot = false;
      this._pointCloudSourcePoints = Math.floor(this._allPositions.length / 3);
      this._rebuildPointCloud();
      window.dispatchEvent(new CustomEvent('pointcloud3d:status', {
        detail: { state: 'loaded', points: this._allPositions.length / 3, lod }
      }));
    } catch (e) {
      window.dispatchEvent(new CustomEvent('pointcloud3d:status', { detail: { state: 'error', message: e.message } }));
    }
  }

  appendPoints(float32Array) {
    const merged = new Float32Array(this._allPositions.length + float32Array.length);
    merged.set(this._allPositions);
    merged.set(float32Array, this._allPositions.length);
    this._allPositions = merged;
    this._hasPointCloudSnapshot = true;
    this._isLiveCloudMapSnapshot = false;
    this._pointCloudSourcePoints = Math.floor(this._allPositions.length / 3);
    this._rebuildPointCloud();
  }

  _replaceCloudMapBuffer(xyzArray, revision) {
    this._lastCloudMapRevision = revision;
    this._hasPointCloudSnapshot = true;
    this._isLiveCloudMapSnapshot = true;
    this._pointCloudSourcePoints = Math.floor(xyzArray.length / 3);

    // 32MB 上限检查
    if (xyzArray.byteLength > MAX_BUFFER_BYTES) {
      const maxPoints = Math.floor(MAX_BUFFER_BYTES / 12);
      xyzArray = xyzArray.slice(0, maxPoints * 3);
    }

    this._allPositions = xyzArray;
    this._rebuildPointCloud();
  }

  _ensureRobotMesh() {
    if (this._robotMesh || !this._scene) return;
    const group = new THREE.Group();
    group.userData.kind = 'robot-dog';

    const bodyGeometry = new THREE.BoxGeometry(0.64, 0.32, 0.22);
    const headGeometry = new THREE.BoxGeometry(0.24, 0.26, 0.22);
    const legGeometry = new THREE.BoxGeometry(0.1, 0.1, 0.3);
    const bodyMaterial = new THREE.MeshBasicMaterial({ color: COLORS.robot });
    const accentMaterial = new THREE.MeshBasicMaterial({ color: COLORS.robotAccent });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.name = 'robot-body';
    body.userData.role = 'body';
    body.position.z = 0.42;
    group.add(body);

    const head = new THREE.Mesh(headGeometry, bodyMaterial);
    head.name = 'robot-head';
    head.userData.role = 'head';
    head.position.set(0.43, 0, 0.46);
    group.add(head);

    for (const [name, x, y] of [
      ['front-left', 0.22, 0.13],
      ['front-right', 0.22, -0.13],
      ['rear-left', -0.22, 0.13],
      ['rear-right', -0.22, -0.13],
    ]) {
      const leg = new THREE.Mesh(legGeometry, accentMaterial);
      leg.name = `robot-leg-${name}`;
      leg.userData.role = `leg-${name}`;
      leg.position.set(x, y, 0.16);
      group.add(leg);
    }

    const headingGeometry = new THREE.ConeGeometry(0.16, 0.34, 8);
    headingGeometry.rotateZ(-Math.PI / 2);
    const heading = new THREE.Mesh(headingGeometry, accentMaterial);
    heading.name = 'robot-heading';
    heading.userData.role = 'heading';
    heading.position.set(0.64, 0, 0.46);
    group.add(heading);

    const ringGeometry = new THREE.RingGeometry(0.25, 0.31, 24);
    const ringMaterial = new THREE.MeshBasicMaterial({
      color: COLORS.robot,
      transparent: true,
      opacity: 0.48,
      side: THREE.DoubleSide,
    });
    const ring = new THREE.Mesh(ringGeometry, ringMaterial);
    ring.name = 'robot-footprint';
    ring.userData.role = 'footprint';
    ring.position.z = 0.01;
    group.add(ring);
    group.userData.disposableResources = [
      bodyGeometry,
      headGeometry,
      legGeometry,
      bodyMaterial,
      accentMaterial,
      headingGeometry,
      ringGeometry,
      ringMaterial,
    ];
    this._robotMesh = group;
    this._scene.add(group);
  }

  setRobotPose(x, y, z, yaw) {
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
      this._robotMesh.rotation.z = pose.yaw;
    }

    if (this._cameraMode === 'follow' && this._camera && this._controls) {
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
    this._clearSemanticInstances();
    this._syncSemanticGroupVisibility();
    for (const instance of this._semanticInstances) {
      const pose = instance?.pose;
      const x = Number(pose?.x);
      const y = Number(pose?.y);
      const zValue = Number(pose?.z || 0);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      const z = Number.isFinite(zValue) ? zValue : 0;
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
    if (this._trajectoryLine) {
      this._scene.remove(this._trajectoryLine);
      this._trajectoryLine.geometry.dispose();
      this._trajectoryLine.material.dispose();
      this._trajectoryLine = null;
    }
    if (!this._scene || this._trajectory.length < 2) return;
    const pts = this._trajectory.map(p => new THREE.Vector3(
      Number(p.x || 0),
      Number(p.y || 0),
      Number(p.z || 0.05),
    ));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color: COLORS.trajectory });
    this._trajectoryLine = new THREE.Line(geo, mat);
    this._scene.add(this._trajectoryLine);
    this._requestRender();
  }

  updateNavigationPath(path, goalPose) {
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
        Number(p.z || 0.08),
      ));
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      const mat = new THREE.LineDashedMaterial({ color: COLORS.navPath, dashSize: 0.3, gapSize: 0.15 });
      this._navPathLine = new THREE.Line(geo, mat);
      this._navPathLine.computeLineDistances();
      this._scene.add(this._navPathLine);
    }
    if (this._navigationGoal) {
      const goal = this._navigationGoal;
      const geo = new THREE.ConeGeometry(0.25, 0.6, 8);
      geo.rotateZ(-Math.PI / 2);
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
    if (this._hasPointCloudSnapshot) this._rebuildPointCloud();
    if (this._robotPose) {
      const { x, y, z, yaw } = this._robotPose;
      this.setRobotPose(x, y, z, yaw);
    }
    this.updateFrontiers(this._frontiers);
    this.updateTrajectory(this._trajectory);
    this.updateNavigationPath(this._navigationPath, this._navigationGoal);
    this._rebuildForbiddenZones();
    this._rebuildSemanticInstances();
    this.setSemanticInstancesVisible(this._semanticInstancesVisible);
  }

  setLOD(level) {
    if (!(level in LOD_STRIDE)) return;
    this._lod = level;
    this._rebuildPointCloud();
  }

  setPointStyle(style) {
    if (!Object.prototype.hasOwnProperty.call(POINT_STYLE_PRESETS, style)) return false;
    const changed = this._pointStyle !== style;
    this._pointStyle = style;
    if (changed && this._hasPointCloudSnapshot) this._rebuildPointCloud();
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
  if (!q) return 0;
  const siny = 2 * ((Number(q.w) || 0) * (Number(q.z) || 0) + (Number(q.x) || 0) * (Number(q.y) || 0));
  const cosy = 1 - 2 * ((Number(q.y) || 0) * (Number(q.y) || 0) + (Number(q.z) || 0) * (Number(q.z) || 0));
  return Math.atan2(siny, cosy);
}

window.addEventListener('stream:exploration.frontier', (e) => {
  const data = e.detail;
  const rawFrontiers = Array.isArray(data) ? data
    : data?.latest_frontiers ?? data?.frontiers ?? [];
  const frontiers = rawFrontiers.map(pointFromValue).filter(Boolean);
  window.pointcloud3d.updateFrontiers(frontiers);
  const rawPath = firstDefinedValue(data?.planned_path, data?.nav_path, data?.path);
  const path = rawPath === null ? []
    : Array.isArray(rawPath) ? rawPath.map(pointFromValue).filter(Boolean)
      : undefined;
  const rawGoal = firstAvailableValue(
    data?.current_goal,
    data?.selected_frontier,
    data?.current_frontier,
    data?.metadata?.current_frontier,
  );
  const goalPoint = pointFromValue(rawGoal);
  const goal = rawGoal === undefined ? undefined
    : goalPoint ? { ...goalPoint, yaw: yawFromValue(rawGoal) }
      : null;
  if (path !== undefined || goal !== undefined) {
    window.pointcloud3d.updateNavigationPath(path, goal);
  }
});

window.addEventListener('stream:localization.pose', (e) => {
  const d = e.detail;
  const pos = d?.position;
  if (!pos || pos.x === undefined) {
    window.pointcloud3d.clearRobotPose();
    return;
  }
  window.pointcloud3d.setRobotPose(
    Number(pos.x),
    Number(pos.y),
    Number(pos.z || 0),
    yawFromValue(d),
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
  const rawPath = firstDefinedValue(d?.planned_path, d?.nav_path, d?.path);
  const path = rawPath === null ? []
    : Array.isArray(rawPath) ? rawPath.map(pointFromValue).filter(Boolean)
      : undefined;
  const rawGoal = firstAvailableValue(d?.current_goal, d?.goal_pose, d?.goal);
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

// 实时累积地图 binary 帧接收
window.addEventListener('stream:pointcloud_map.binary', (e) => {
  if (!window.pointcloud3d) return;
  const buf = e.detail;
  if (!buf || buf.byteLength < 8) return;

  const header = new DataView(buf, 0, 8);
  const revision = header.getUint32(0, true);
  const pointCount = header.getUint32(4, true);

  if (pointCount === 0) {
    window.pointcloud3d._replaceCloudMapBuffer(new Float32Array(0), revision);
    return;
  }
  if (revision <= (window.pointcloud3d._lastCloudMapRevision || 0)) return;

  if (buf.byteLength < 8 + pointCount * 3 * Float32Array.BYTES_PER_ELEMENT) return;
  const xyz = new Float32Array(buf, 8, pointCount * 3);
  window.pointcloud3d._replaceCloudMapBuffer(xyz, revision);
});
