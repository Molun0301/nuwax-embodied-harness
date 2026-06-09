import * as THREE from './three.module.min.js';
import { OrbitControls } from './OrbitControls.js';

const COLORS = {
  robot: 0xfbbf24,
  frontier: 0xf472b6,
  trajectory: 0x22d3ee,
  navPath: 0x22ff66,
  navGoal: 0xa78bfa,
  grid: 0x334155,
};

const LOD_STRIDE = { preview: 8, medium: 4, full: 1 };
const MAX_BUFFER_BYTES = 32 * 1024 * 1024;
const MAX_POINTS = Math.floor(MAX_BUFFER_BYTES / (3 * 4));

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
    this._trajectoryLine = null;
    this._navPathLine = null;
    this._navGoalMesh = null;
    this._lod = 'preview';
    this._needsRender = false;
    this._rafId = null;
    this._lastFrameTime = 0;
    this._allPositions = new Float32Array(0);
    this._initialized = false;
  }

  init(container) {
    if (this._initialized) return;
    this._container = container;
    this._setupScene();
    this._setupControls();
    this._setupResizeObserver();
    this._initialized = true;
    this._requestRender();
    window.dispatchEvent(new CustomEvent('pointcloud3d:ready'));
    window.dispatchEvent(new CustomEvent('pointcloud3d:status', {
      detail: { state: 'ready', lod: this._lod }
    }));
  }

  _setupScene() {
    const w = this._container.clientWidth || 800;
    const h = this._container.clientHeight || 600;

    this._renderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: 'low-power' });
    this._renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this._renderer.setSize(w, h);
    this._container.appendChild(this._renderer.domElement);

    this._scene = new THREE.Scene();
    this._scene.background = new THREE.Color(0x0f172a);

    this._camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 500);
    this._camera.position.set(0, -10, 8);
    this._camera.up.set(0, 0, 1);

    const grid = new THREE.GridHelper(40, 40, COLORS.grid, COLORS.grid);
    grid.rotation.x = Math.PI / 2;
    this._scene.add(grid);

    this._scene.add(new THREE.AxesHelper(1));

    this._frontierGroup = new THREE.Group();
    this._scene.add(this._frontierGroup);
  }

  _setupControls() {
    this._controls = new OrbitControls(this._camera, this._renderer.domElement);
    this._controls.enableDamping = true;
    this._controls.dampingFactor = 0.1;
    this._controls.screenSpacePanning = false;
    this._controls.addEventListener('change', () => this._requestRender());
    this._controls.addEventListener('start', () => this._requestRender());
  }

  _setupResizeObserver() {
    const ro = new ResizeObserver(() => {
      const w = this._container.clientWidth;
      const h = this._container.clientHeight;
      this._camera.aspect = w / h;
      this._camera.updateProjectionMatrix();
      this._renderer.setSize(w, h);
      this._requestRender();
    });
    ro.observe(this._container);
  }

  _requestRender() {
    this._needsRender = true;
    if (!this._rafId) {
      this._rafId = requestAnimationFrame((t) => this._renderLoop(t));
    }
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
    this._controls.update();
    this._renderer.render(this._scene, this._camera);
    if (this._controls.enabled && this._controls._needsUpdate) {
      this._requestRender();
    }
  }

  _colorByHeight(positions) {
    const count = positions.length / 3;
    const colors = new Float32Array(count * 3);
    let zMin = Infinity, zMax = -Infinity;
    for (let i = 2; i < positions.length; i += 3) {
      if (positions[i] < zMin) zMin = positions[i];
      if (positions[i] > zMax) zMax = positions[i];
    }
    const range = zMax - zMin || 1;
    const low = new THREE.Color(0x3b82f6);
    const mid = new THREE.Color(0x22c55e);
    const high = new THREE.Color(0xef4444);
    const tmp = new THREE.Color();
    for (let i = 0; i < count; i++) {
      const t = (positions[i * 3 + 2] - zMin) / range;
      if (t < 0.5) tmp.lerpColors(low, mid, t * 2);
      else tmp.lerpColors(mid, high, (t - 0.5) * 2);
      colors[i * 3] = tmp.r;
      colors[i * 3 + 1] = tmp.g;
      colors[i * 3 + 2] = tmp.b;
    }
    return colors;
  }

  _rebuildPointCloud() {
    if (this._pointsMesh) {
      this._scene.remove(this._pointsMesh);
      this._pointsMesh.geometry.dispose();
      this._pointsMesh.material.dispose();
      this._pointsMesh = null;
    }
    const stride = LOD_STRIDE[this._lod] || 1;
    const src = this._allPositions;
    const totalPts = Math.floor(src.length / 3);
    const sampledCount = Math.min(Math.ceil(totalPts / stride), MAX_POINTS);
    if (sampledCount === 0) return;

    const pos = new Float32Array(sampledCount * 3);
    for (let i = 0, j = 0; i < sampledCount; i++, j += stride) {
      pos[i * 3] = src[j * 3];
      pos[i * 3 + 1] = src[j * 3 + 1];
      pos[i * 3 + 2] = src[j * 3 + 2];
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(this._colorByHeight(pos), 3));

    const mat = new THREE.PointsMaterial({ size: 0.05, vertexColors: true, sizeAttenuation: true });
    this._pointsMesh = new THREE.Points(geo, mat);
    this._scene.add(this._pointsMesh);
    this._requestRender();
  }

  async loadPointCloud(url, lod = 'preview') {
    this._lod = lod;
    try {
      const resp = await fetch(url);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const buf = await resp.arrayBuffer();
      this._allPositions = new Float32Array(buf);
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
    this._rebuildPointCloud();
  }

  _replaceCloudMapBuffer(xyzArray, revision) {
    this._lastCloudMapRevision = revision;

    // 32MB 上限检查
    if (xyzArray.byteLength > MAX_BUFFER_BYTES) {
      const maxPoints = Math.floor(MAX_BUFFER_BYTES / 12);
      xyzArray = xyzArray.slice(0, maxPoints * 3);
    }

    // 移除旧点云，避免 appendPoints 的 GC 压力
    if (this._pointsMesh) {
      this._scene.remove(this._pointsMesh);
      this._pointsMesh.geometry.dispose();
      this._pointsMesh.material.dispose();
      this._pointsMesh = null;
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(xyzArray, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(this._colorByHeight(xyzArray), 3));

    const mat = new THREE.PointsMaterial({ size: 0.05, vertexColors: true, sizeAttenuation: true });
    this._pointsMesh = new THREE.Points(geo, mat);
    this._scene.add(this._pointsMesh);
    this._allPositions = xyzArray;

    this._requestRender();
  }

  setRobotPose(x, y, yaw) {
    if (!this._robotMesh) {
      const geo = new THREE.ConeGeometry(0.2, 0.5, 8);
      geo.rotateX(Math.PI / 2);
      const mat = new THREE.MeshBasicMaterial({ color: COLORS.robot });
      this._robotMesh = new THREE.Mesh(geo, mat);
      this._scene.add(this._robotMesh);
    }
    this._robotMesh.position.set(x, y, 0.25);
    this._robotMesh.rotation.z = yaw;
    this._requestRender();
  }

  updateFrontiers(frontiers) {
    this._frontierGroup.clear();
    const geo = new THREE.SphereGeometry(0.15, 6, 6);
    const mat = new THREE.MeshBasicMaterial({ color: COLORS.frontier });
    for (const f of frontiers) {
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(f.x, f.y, 0.1);
      this._frontierGroup.add(mesh);
    }
    this._requestRender();
  }

  updateTrajectory(trajectory) {
    if (this._trajectoryLine) {
      this._scene.remove(this._trajectoryLine);
      this._trajectoryLine.geometry.dispose();
      this._trajectoryLine.material.dispose();
      this._trajectoryLine = null;
    }
    if (!trajectory || trajectory.length < 2) return;
    const pts = trajectory.map(p => new THREE.Vector3(p.x, p.y, 0.05));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color: COLORS.trajectory });
    this._trajectoryLine = new THREE.Line(geo, mat);
    this._scene.add(this._trajectoryLine);
    this._requestRender();
  }

  updateNavigationPath(path, goalPose) {
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
    if (path && path.length >= 2) {
      const pts = path.map(p => new THREE.Vector3(p.x, p.y, 0.08));
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      const mat = new THREE.LineDashedMaterial({ color: COLORS.navPath, dashSize: 0.3, gapSize: 0.15 });
      this._navPathLine = new THREE.Line(geo, mat);
      this._navPathLine.computeLineDistances();
      this._scene.add(this._navPathLine);
    }
    if (goalPose) {
      const geo = new THREE.ConeGeometry(0.25, 0.6, 8);
      geo.rotateX(Math.PI / 2);
      const mat = new THREE.MeshBasicMaterial({ color: COLORS.navGoal });
      this._navGoalMesh = new THREE.Mesh(geo, mat);
      this._navGoalMesh.position.set(goalPose.x, goalPose.y, 0.3);
      if (goalPose.yaw !== undefined) this._navGoalMesh.rotation.z = goalPose.yaw;
      this._scene.add(this._navGoalMesh);
    }
    this._requestRender();
  }

  setLOD(level) {
    if (!(level in LOD_STRIDE)) return;
    this._lod = level;
    this._rebuildPointCloud();
  }

  dispose() {
    if (this._rafId) cancelAnimationFrame(this._rafId);
    this._controls?.dispose();
    this._renderer?.dispose();
    this._scene?.clear();
    this._initialized = false;
  }
}

window.pointcloud3d = new PointCloud3DViewer();

window.addEventListener('stream:exploration.frontier', (e) => {
  const data = e.detail;
  const frontiers = Array.isArray(data) ? data
    : data?.latest_frontiers ?? data?.frontiers ?? [];
  window.pointcloud3d.updateFrontiers(frontiers);
});

window.addEventListener('stream:localization.pose', (e) => {
  const d = e.detail;
  const pos = d?.position;
  if (!pos || pos.x === undefined) return;
  const q = d?.orientation;
  let yaw = 0;
  if (q) {
    const siny = 2 * ((q.w || 0) * (q.z || 0) + (q.x || 0) * (q.y || 0));
    const cosy = 1 - 2 * ((q.y || 0) * (q.y || 0) + (q.z || 0) * (q.z || 0));
    yaw = Math.atan2(siny, cosy);
  }
  window.pointcloud3d.setRobotPose(Number(pos.x), Number(pos.y), yaw);
});

window.addEventListener('stream:localization.trajectory', (e) => {
  const d = e.detail;
  const traj = Array.isArray(d) ? d : d?.trajectory;
  if (traj) window.pointcloud3d.updateTrajectory(traj);
});

window.addEventListener('stream:navigation.state', (e) => {
  const d = e.detail;
  const path = d?.planned_path ?? d?.nav_path ?? d?.path ?? null;
  const goal = d?.current_goal ?? d?.goal_pose ?? d?.goal ?? null;
  window.pointcloud3d.updateNavigationPath(path, goal);
});

window.addEventListener('stream:pointcloud_map.meta', (e) => {
  const d = e.detail;
  if (d?.url) window.pointcloud3d.loadPointCloud(d.url, d.lod ?? 'preview');
});

// 实时累积地图 binary 帧接收
window.addEventListener('stream:pointcloud_map.binary', (e) => {
  if (!window.pointcloud3d || !window.pointcloud3d._scene) return;
  const buf = e.detail;
  if (!buf || buf.byteLength < 8) return;

  const header = new DataView(buf, 0, 8);
  const revision = header.getUint32(0, true);
  const pointCount = header.getUint32(4, true);

  if (pointCount === 0) return;
  if (revision <= (window.pointcloud3d._lastCloudMapRevision || 0)) return;

  const xyz = new Float32Array(buf, 8, pointCount * 3);
  window.pointcloud3d._replaceCloudMapBuffer(xyz, revision);
});
