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
};

const LOD_STRIDE = { preview: 8, medium: 4, full: 1 };
const MAX_BUFFER_BYTES = 32 * 1024 * 1024;
const MAX_POINTS = Math.floor(MAX_BUFFER_BYTES / (3 * 4));
const CAMERA_MODES = new Set(['free', 'follow']);
const CAMERA_MIN_DISTANCE = 1.5;
const CAMERA_MAX_DISTANCE = 180;
const FOLLOW_TARGET_HEIGHT = 0.25;
const DEFAULT_CAMERA_DIRECTION = new THREE.Vector3(0, -1, 0.72).normalize();

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
    this._needsRender = false;
    this._rafId = null;
    this._lastFrameTime = 0;
    this._allPositions = new Float32Array(0);
    this._hasPointCloudSnapshot = false;
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
    this._renderer.domElement.setAttribute('aria-label', '3D 点云地图');
    this._container.appendChild(this._renderer.domElement);

    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x0f172a);

    this._camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 500);
    this._camera.position.set(0, -10, 8);
    this._camera.up.set(0, 0, 1);

    this._frontierGroup = new THREE.Group();
    this._scene.add(this._frontierGroup);

    this._semanticGroup = new THREE.Group();
    this._semanticGroup.visible = this._semanticInstancesVisible;
    this._scene.add(this._semanticGroup);
    this._semanticMarkerGeometry = new THREE.OctahedronGeometry(0.16, 0);
    this._semanticPreciseMaterial = new THREE.MeshBasicMaterial({ color: COLORS.semantic });
    this._semanticFallbackMaterial = new THREE.MeshBasicMaterial({
      color: COLORS.semanticFallback,
      transparent: true,
      opacity: 0.62,
      wireframe: true,
    });
  }

  _setupControls() {
    this._controls = new OrbitControls(this._camera, this._renderer.domElement);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.08;
    this._controls.enableRotate = true;
    this._controls.enablePan = this._cameraMode === 'free';
    this._controls.enableZoom = true;
    this._controls.screenSpacePanning = false;
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
      zMin: minZ,
      zMax: maxZ,
      bounds: new THREE.Sphere(center, radius),
    };
  }

  _colorByHeight(positions, zMin, zMax) {
    const count = positions.length / 3;
    const colors = new Uint8Array(count * 3);
    if (count === 0) return colors;
    const range = zMax - zMin || 1;
    const low = [59, 130, 246];
    const mid = [34, 197, 94];
    const high = [239, 68, 68];
    for (let i = 0; i < count; i++) {
      const t = THREE.MathUtils.clamp((positions[i * 3 + 2] - zMin) / range, 0, 1);
      const upperHalf = t >= 0.5;
      const blend = upperHalf ? (t - 0.5) * 2 : t * 2;
      const from = upperHalf ? mid : low;
      const to = upperHalf ? high : mid;
      colors[i * 3] = Math.round(from[0] + (to[0] - from[0]) * blend);
      colors[i * 3 + 1] = Math.round(from[1] + (to[1] - from[1]) * blend);
      colors[i * 3 + 2] = Math.round(from[2] + (to[2] - from[2]) * blend);
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
    window.dispatchEvent(new CustomEvent('pointcloud3d:point-count', {
      detail: {
        points: this._displayedPointCount,
        sourcePoints: Math.floor(this._allPositions.length / 3),
        lod: this._lod,
      },
    }));
  }

  _renderPointCloudPositions(positions) {
    if (!this._scene) return;
    this._disposePointsMesh();
    if (!positions.length) {
      this._displayedPointCount = 0;
      this._pointCloudBounds = null;
      this._emitPointCount();
      this._requestRender();
      return;
    }
    const measurement = this._measurePointCloud(positions);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.Uint8BufferAttribute(
      this._colorByHeight(positions, measurement.zMin, measurement.zMax),
      3,
      true,
    ));
    geo.boundingSphere = measurement.bounds;
    this._pointCloudBounds = measurement.bounds.clone();
    const mat = new THREE.PointsMaterial({ size: 0.05, vertexColors: true, sizeAttenuation: true });
    this._pointsMesh = new THREE.Points(geo, mat);
    this._scene.add(this._pointsMesh);
    this._displayedPointCount = Math.floor(positions.length / 3);
    this._emitPointCount();
    if (!this._hasFramedPointCloud && !this._hasUserCameraInput && this._cameraMode === 'free') {
      this._framePointCloud();
    }
    this._requestRender();
  }

  _rebuildPointCloud() {
    const stride = LOD_STRIDE[this._lod] || 1;
    const src = this._allPositions;
    const totalPts = Math.floor(src.length / 3);
    const sampledCount = Math.min(Math.ceil(totalPts / stride), MAX_POINTS);
    if (sampledCount === 0) {
      this._renderPointCloudPositions(new Float32Array(0));
      return;
    }

    const pos = new Float32Array(sampledCount * 3);
    for (let i = 0, j = 0; i < sampledCount; i++, j += stride) {
      pos[i * 3] = src[j * 3];
      pos[i * 3 + 1] = src[j * 3 + 1];
      pos[i * 3 + 2] = src[j * 3 + 2];
    }
    this._renderPointCloudPositions(pos);
  }

  async loadPointCloud(url, lod = 'preview') {
    this._lod = lod;
    try {
      const resp = await fetch(url);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const buf = await resp.arrayBuffer();
      this._allPositions = new Float32Array(buf);
      this._hasPointCloudSnapshot = true;
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
    this._rebuildPointCloud();
  }

  _replaceCloudMapBuffer(xyzArray, revision) {
    this._lastCloudMapRevision = revision;
    this._hasPointCloudSnapshot = true;

    // 32MB 上限检查
    if (xyzArray.byteLength > MAX_BUFFER_BYTES) {
      const maxPoints = Math.floor(MAX_BUFFER_BYTES / 12);
      xyzArray = xyzArray.slice(0, maxPoints * 3);
    }

    this._allPositions = xyzArray;
    this._renderPointCloudPositions(xyzArray);
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
    this._semanticGroup.visible = this._semanticInstancesVisible;
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
    if (this._semanticGroup) this._semanticGroup.visible = this._semanticInstancesVisible;
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

  _applyCachedSceneState() {
    if (this._hasPointCloudSnapshot) this._rebuildPointCloud();
    if (this._robotPose) {
      const { x, y, z, yaw } = this._robotPose;
      this.setRobotPose(x, y, z, yaw);
    }
    this.updateFrontiers(this._frontiers);
    this.updateTrajectory(this._trajectory);
    this.updateNavigationPath(this._navigationPath, this._navigationGoal);
    this._rebuildSemanticInstances();
    this.setSemanticInstancesVisible(this._semanticInstancesVisible);
  }

  setLOD(level) {
    if (!(level in LOD_STRIDE)) return;
    this._lod = level;
    this._rebuildPointCloud();
  }

  dispose() {
    if (this._rafId) cancelAnimationFrame(this._rafId);
    this._resizeObserver?.disconnect();
    this._resizeObserver = null;
    this._disposePointsMesh();
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
    const canvas = this._renderer?.domElement;
    if (canvas?.parentNode) canvas.parentNode.removeChild(canvas);
    this._scene?.clear();
    this._renderer = null;
    this._scene = null;
    this._camera = null;
    this._controls = null;
    this._robotMesh = null;
    this._frontierGroup = null;
    this._semanticGroup = null;
    this._trajectoryLine = null;
    this._navPathLine = null;
    this._navGoalMesh = null;
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
