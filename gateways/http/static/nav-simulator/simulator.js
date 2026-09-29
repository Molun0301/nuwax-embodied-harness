import * as THREE from '../js/three.module.min.js';
import { OrbitControls } from '../js/OrbitControls.js';

const COLORS = { global: 0xa5e27e, local: 0x69dedc, trajectory: 0xb2a0f8, start: 0x69a9ff, goal: 0xa5e27e, robot: 0xf4d28c, obstacle: 0xffba78, retained: 0xffba78 };
const STATUS = { ready: '准备就绪', planning: '正在规划', running: '正在行走', paused: '已暂停', succeeded: '已到达', failed: '未完成', cancelled: '已取消' };
const ACTIVE = new Set(['planning', 'running', 'paused']);

export function poseArray(value) {
  if (!Array.isArray(value) || value.length < 3 || !value.slice(0, 3).every(Number.isFinite)) return null;
  return [value[0], value[1], value[2], Number.isFinite(value[3]) ? value[3] : 0];
}

export function routeViewBounds(start, goal, path = []) {
  const low = [Infinity, Infinity, Infinity], high = [-Infinity, -Infinity, -Infinity];
  for (const value of [start, goal, ...path]) {
    const point = poseArray(value); if (!point) continue;
    for (let axis = 0; axis < 3; axis++) { low[axis] = Math.min(low[axis], point[axis]); high[axis] = Math.max(high[axis], point[axis]); }
  }
  if (!low.every(Number.isFinite)) return null;
  const padding = Math.max(1.2, Math.hypot(high[0] - low[0], high[1] - low[1]) * 0.16);
  return [low[0] - padding, low[1] - padding, low[2] - 0.6, high[0] + padding, high[1] + padding, high[2] + 0.6];
}

export function frameIndexAt(frames, time) {
  let low = 0, high = frames.length - 1;
  while (low <= high) {
    const middle = (low + high) >>> 1;
    if (Number(frames[middle].t) <= time) low = middle + 1;
    else high = middle - 1;
  }
  return frames.length ? Math.max(0, high) : -1;
}

export function measuredSpeed(frames, index = frames.length - 1) {
  if (index < 1) return 0;
  const current = frames[index], previous = frames[index - 1];
  const a = poseArray(previous.pose), b = poseArray(current.pose), dt = Number(current.t) - Number(previous.t);
  return a && b && dt > 0 ? Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / dt : 0;
}

export function appendFrameDelta(frames, snapshot) {
  if (!Array.isArray(snapshot.frames)) return false;
  const incoming = snapshot.frames;
  const offset = Number.isInteger(snapshot.frame_offset) ? snapshot.frame_offset : 0;
  if (offset < 0 || offset > frames.length) throw new Error('轨迹帧有缺口，需要重读完整记录');
  const replaced = offset < frames.length;
  frames.length = offset;
  // 逐条追加，避免长时间模拟超过函数参数数量限制。
  for (const frame of incoming) frames.push(frame);
  return replaced;
}

export function replaySnapshot(frames, time, goal, pathHistory) {
  const index = frameIndexAt(frames, time), frame = frames[index];
  if (!frame) return null;
  const pose = poseArray(frame.pose), target = poseArray(goal);
  return {
    index, frame, pose, speed: measuredSpeed(frames, index),
    distance: pose && target ? Math.hypot(...target.slice(0, 3).map((value, i) => value - pose[i])) : null,
    path: pathHistory.get(String(frame.path_revision)) || null,
  };
}

export class DynamicLine {
  constructor(scene, color, opacity = 1) {
    this.mesh = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color, opacity, transparent: opacity < 1, depthTest: false }));
    this.mesh.renderOrder = 5;
    this.mesh.frustumCulled = false;
    this.mesh.visible = false;
    scene.add(this.mesh);
    this.key = null;
    this.uploadedFrames = 0;
  }
  attribute(count) {
    let position = this.mesh.geometry.getAttribute('position');
    if (!position || position.count < count) {
      const capacity = Math.max(64, 2 ** Math.ceil(Math.log2(Math.max(1, count))));
      this.mesh.geometry.dispose();
      this.mesh.geometry = new THREE.BufferGeometry();
      position = new THREE.BufferAttribute(new Float32Array(capacity * 3), 3);
      position.setUsage(THREE.DynamicDrawUsage);
      this.mesh.geometry.setAttribute('position', position);
      this.uploadedFrames = 0;
    }
    return position;
  }
  set(points, key) {
    if (this.key === key) return false;
    this.key = key;
    const count = points?.length || 0;
    this.mesh.visible = count > 1;
    if (count > 1) {
      const position = this.attribute(count);
      for (let i = 0; i < count; i++) position.setXYZ(i, Number(points[i][0]), Number(points[i][1]), Number(points[i][2]));
      position.clearUpdateRanges(); position.addUpdateRange(0, count * 3); position.needsUpdate = true;
      this.mesh.geometry.setDrawRange(0, count);
    }
    return true;
  }
  setFrames(frames, count) {
    this.mesh.visible = count > 1;
    if (frames.length > 1) {
      const position = this.attribute(frames.length);
      const from = this.uploadedFrames;
      for (let i = from; i < frames.length; i++) {
        const pose = frames[i].pose;
        position.setXYZ(i, pose[0], pose[1], pose[2]);
      }
      if (frames.length > from) {
        position.clearUpdateRanges(); position.addUpdateRange(from * 3, (frames.length - from) * 3); position.needsUpdate = true;
        this.uploadedFrames = frames.length;
      }
      this.mesh.geometry.setDrawRange(0, count);
    }
  }
  reset() { this.key = null; this.uploadedFrames = 0; this.mesh.visible = false; }
}

class SimulationView {
  constructor(container, onPick) {
    this.container = container;
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color(0x0e1622);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'low-power' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    this.renderer.domElement.setAttribute('aria-label', '真实地图点云，可选起点、目标与动态障碍');
    this.renderer.domElement.tabIndex = 0;
    container.prepend(this.renderer.domElement);
    this.perspective = new THREE.PerspectiveCamera(48, 1, 0.02, 5000); this.perspective.up.set(0, 0, 1);
    this.top = new THREE.OrthographicCamera(-10, 10, 10, -10, 0.02, 5000); this.top.up.set(0, 1, 0);
    this.camera = this.perspective;
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true; this.controls.dampingFactor = 0.12;
    this.controls.addEventListener('change', () => { if (this.userInteracting) this.userCamera = true; this.requestRender(); });
    this.controls.addEventListener('start', () => { this.userInteracting = true; });
    this.controls.addEventListener('end', () => { this.userInteracting = false; });
    this.camera.position.set(8, -10, 10); this.controls.update();
    this.global = new DynamicLine(this.scene, COLORS.global);
    this.local = new DynamicLine(this.scene, COLORS.local);
    this.traveled = new DynamicLine(this.scene, COLORS.trajectory, 0.85);
    this.cloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0x879caf, size: 2, sizeAttenuation: false, transparent: true, opacity: 0.74 }));
    this.cloud.frustumCulled = false; this.scene.add(this.cloud);
    this.start = this.createMarker(COLORS.start); this.goal = this.createMarker(COLORS.goal);
    this.body = new THREE.Group();
    const bodyMesh = new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.28, 0.18), new THREE.MeshBasicMaterial({ color: COLORS.robot, transparent: true, opacity: 0.92, depthTest: false }));
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.105, 0.22, 3), new THREE.MeshBasicMaterial({ color: 0xfff4d6, depthTest: false }));
    nose.rotation.z = -Math.PI / 2; nose.position.x = 0.36;
    this.bodyMesh = bodyMesh; this.headingMesh = nose;
    this.body.add(bodyMesh, nose); this.body.renderOrder = 9; this.body.visible = false; this.scene.add(this.body);
    this.obstacles = new THREE.Group(); this.scene.add(this.obstacles);
    this.obstacleGeometry = new THREE.CylinderGeometry(1, 1, 1, 24); this.obstacleGeometry.rotateX(Math.PI / 2);
    this.obstacleMaterial = new THREE.MeshBasicMaterial({ color: COLORS.obstacle, transparent: true, opacity: 0.43 });
    this.obstacleEdges = new THREE.EdgesGeometry(this.obstacleGeometry);
    this.obstacleEdgeMaterial = new THREE.LineBasicMaterial({ color: COLORS.obstacle });
    this.raycaster = new THREE.Raycaster(); this.raycaster.params.Points.threshold = 0.15;
    this.bounds = [0, 0, 0, 10, 10, 1]; this.pickMode = null; this.raf = null;
    this.labels = [['start-label', this.start], ['goal-label', this.goal], ['robot-label', this.body]];
    this.pointerStart = null;
    this.renderer.domElement.addEventListener('pointerdown', event => {
      if (event.button === 0) this.pointerStart = [event.clientX, event.clientY];
    });
    this.renderer.domElement.addEventListener('pointerup', event => {
      const origin = this.pointerStart; this.pointerStart = null;
      if (!this.pickMode || event.button !== 0 || !origin || Math.hypot(event.clientX - origin[0], event.clientY - origin[1]) > 5) return;
      const rect = this.renderer.domElement.getBoundingClientRect();
      const pointer = new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1, -(event.clientY - rect.top) / rect.height * 2 + 1);
      const visibleHeight = this.camera.isOrthographicCamera
        ? (this.camera.top - this.camera.bottom) / this.camera.zoom
        : this.camera.position.distanceTo(this.controls.target) * 2 * Math.tan(this.camera.fov * Math.PI / 360);
      this.raycaster.params.Points.threshold = Math.max(0.015, visibleHeight / rect.height * 5);
      this.raycaster.setFromCamera(pointer, this.camera);
      const hits = this.raycaster.intersectObject(this.cloud, false);
      if (!hits.length) { onPick(null, this.pickMode); return; }
      // 读取真正命中的点坐标，不把射线上最近位置或无限平面当作地图点。
      const position = this.cloud.geometry.getAttribute('position');
      const index = hits[0].index;
      onPick([position.getX(index), position.getY(index), position.getZ(index)], this.pickMode);
    });
    this.renderer.domElement.addEventListener('pointercancel', () => { this.pointerStart = null; });
    this.resizeObserver = new ResizeObserver(() => this.resize()); this.resizeObserver.observe(container);
    this.resize();
  }
  createMarker(color) {
    const group = new THREE.Group();
    const disk = new THREE.Mesh(new THREE.RingGeometry(0.13, 0.2, 32), new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, depthTest: false }));
    const center = new THREE.Mesh(new THREE.SphereGeometry(0.07, 12, 8), new THREE.MeshBasicMaterial({ color, depthTest: false }));
    group.add(disk, center); group.renderOrder = 8; group.visible = false; this.scene.add(group); return group;
  }
  setPose(object, pose, orientation = null) {
    object.visible = Boolean(pose);
    if (pose) {
      object.position.set(pose[0], pose[1], pose[2]);
      const marker = object === this.body ? ['robot-label', '机器人'] : object === this.goal ? ['goal-label', '目标'] : object === this.start ? ['start-label', '起点'] : null;
      if (marker) {
        const label = document.getElementById(marker[0]), text = `${marker[1]} · Z ${Number(pose[2]).toFixed(2)} 米`;
        if (label && label.textContent !== text) label.textContent = text;
      }
      if (Array.isArray(orientation) && orientation.length === 4 && orientation.every(Number.isFinite)) {
        object.quaternion.set(...orientation).normalize();
      } else object.rotation.set(0, 0, pose[3] || 0);
    }
  }
  setCloud(buffer, scene) {
    if (buffer.byteLength % 12) throw new Error('点云字节长度不是完整XYZ浮点数组');
    const points = new Float32Array(buffer);
    this.cloud.geometry.dispose(); this.cloud.geometry = new THREE.BufferGeometry();
    this.cloud.geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
    this.bounds = scene.bounds || this.bounds;
    this.setRobotGeometry(scene.robot);
    this.cloud.geometry.computeBoundingSphere();
    this.fit(); return points.length / 3;
  }
  setRobotGeometry(robot) {
    const footprint = robot?.footprint_xy, height = robot?.z_bounds;
    if (!Array.isArray(footprint) || footprint.length < 3 || !Array.isArray(height) || height.length !== 2) return;
    const shape = new THREE.Shape();
    footprint.forEach((point, index) => index ? shape.lineTo(point[0], point[1]) : shape.moveTo(point[0], point[1]));
    shape.closePath();
    const geometry = new THREE.ExtrudeGeometry(shape, { depth: height[1] - height[0], bevelEnabled: false, steps: 1 });
    geometry.translate(0, 0, height[0]);
    this.bodyMesh.geometry.dispose(); this.bodyMesh.geometry = geometry;
    this.headingMesh.position.set(Math.max(...footprint.map(point => point[0])) + 0.08, 0, height[1] + 0.03);
  }
  fit() { this.frameBounds(this.bounds); }
  focusRoute(start, goal, path = []) {
    const bounds = routeViewBounds(start, goal, path);
    if (bounds) this.frameBounds(bounds);
  }
  frameBounds(b) {
    this.framingBounds = b;
    const span = Math.max(b[3] - b[0], b[4] - b[1], b[5] - b[2], 2);
    this.viewSpan = span * 0.72; this.userCamera = false;
    this.controls.target.set((b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2);
    const center = this.controls.target;
    this.perspective.position.set(center.x + span * 0.45, center.y - span * 0.8, center.z + span * 0.9);
    this.top.position.set(center.x, center.y, center.z + span * 2);
    this.top.zoom = 1; this.perspective.zoom = 1;
    this.camera.lookAt(center); this.controls.update(); this.resize();
  }
  setView(top) {
    const target = this.controls.target.clone();
    const b = this.framingBounds || this.bounds;
    const span = Math.max(b[3] - b[0], b[4] - b[1], 2);
    this.camera = top ? this.top : this.perspective;
    if (top) this.camera.position.set(target.x, target.y, target.z + span * 2);
    this.camera.lookAt(target); this.controls.object = this.camera; this.controls.enableRotate = !top;
    this.controls.mouseButtons.LEFT = top ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
    document.querySelector('.camera-hint').textContent = top ? '拖动平移 · 滚轮缩放' : '拖动旋转 · 滚轮缩放 · 右键平移';
    this.controls.update(); this.resize();
  }
  resize() {
    const width = this.container.clientWidth || 800, height = this.container.clientHeight || 500;
    this.renderer.setSize(width, height, false);
    this.perspective.aspect = width / height; this.perspective.updateProjectionMatrix();
    if (!this.userCamera && this.bounds) {
      const b = this.framingBounds || this.bounds, radius = Math.max(1, Math.hypot(b[3] - b[0], b[4] - b[1], b[5] - b[2]) / 2);
      const vertical = this.perspective.fov * Math.PI / 180;
      const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * width / height);
      const distance = radius / Math.sin(Math.min(vertical, horizontal) / 2) * 1.1;
      this.perspective.position.copy(this.controls.target).add(new THREE.Vector3(0.45, -0.8, 0.9).normalize().multiplyScalar(distance));
      this.perspective.lookAt(this.controls.target);
    }
    const span = this.viewSpan || 8;
    this.top.left = -span * width / height; this.top.right = span * width / height; this.top.top = span; this.top.bottom = -span; this.top.updateProjectionMatrix();
    this.requestRender();
  }
  setObstacles(obstacles) {
    this.obstacles.clear();
    for (const obstacle of obstacles) {
      const group = new THREE.Group();
      group.add(new THREE.Mesh(this.obstacleGeometry, this.obstacleMaterial), new THREE.LineSegments(this.obstacleEdges, this.obstacleEdgeMaterial));
      group.scale.set(obstacle.radius, obstacle.radius, obstacle.height);
      group.position.set(obstacle.x, obstacle.y, (obstacle.z || 0) + obstacle.height / 2);
      this.obstacles.add(group);
    }
    this.requestRender();
  }
  requestRender() {
    if (this.raf !== null && this.raf !== undefined) return;
    this.raf = requestAnimationFrame(() => {
      this.raf = null; this.controls.update(); this.renderer.render(this.scene, this.camera);
      const width = this.container.clientWidth, height = this.container.clientHeight;
      for (const [id, object] of this.labels) {
        const label = document.getElementById(id), projected = object.position.clone().project(this.camera);
        label.hidden = !object.visible || projected.z > 1 || projected.z < -1 || Math.abs(projected.x) > 1.05 || Math.abs(projected.y) > 1.05;
        if (!label.hidden) label.style.transform = `translate(${(projected.x + 1) / 2 * width}px,${(-projected.y + 1) / 2 * height - (object === this.body ? 42 : 14)}px) translate(-50%,-100%)`;
      }
    });
  }
  clearSimulation() {
    for (const line of [this.global, this.local, this.traveled]) line.reset();
    this.body.visible = false; this.requestRender();
  }
}

function initialize() {
  const ids = ['scene', 'case', 'scene-description', 'cloud-count', 'scene-name', 'height-hint', 'speed', 'speed-value', 'run', 'pause', 'reset', 'export', 'status', 'message', 'history-note', 'timeline', 'playback-play', 'playback-speed', 'live', 'replay-time', 'frame-count', 'replay-badge', 'pick-hint', 'viewport-empty', 'requires-orientation', 'goal-yaw-label'];
  const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));
  const store = { scene: null, scenes: [], cases: [], caseId: null, cloudLoaded: false, loadToken: 0, start: [0, 0, 0.4, 0], goal: [7, 0, 0.4, 0], id: null, snapshot: null, frames: [], paths: new Map(), locals: new Map(), lastGlobal: null, obstacles: [], pollTimer: null, pollRequest: null, pollingEpoch: 0, busy: false, operationError: null, replay: false, replayTime: 0, playing: false, playbackRaf: null, pickMode: null };
  let view;
  const setText = (node, text) => { if (node && node.textContent !== String(text)) node.textContent = String(text); };
  const number = (value, digits = 2) => Number.isFinite(Number(value)) && value !== null && value !== undefined ? Number(value).toFixed(digits) : '—';
  function feedback(message, failed = false, fromSnapshot = false) {
    if (fromSnapshot && store.operationError) return;
    if (!fromSnapshot) store.operationError = failed ? message : null;
    setText(el.message, message); el.message.classList.toggle('error', failed);
  }
  async function json(url, options = {}) {
    const response = await fetch(url, { ...options, headers: { 'Content-Type': 'application/json', ...options.headers } });
    let data;
    try { data = await response.json(); } catch { throw new Error(`服务响应无法读取（HTTP ${response.status}）`); }
    if (!response.ok || data.ok === false) {
      const detail = data.detail || data.error?.message || data.message || `HTTP ${response.status}`;
      throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail));
    }
    return data;
  }
  const post = (url, data) => json(url, { method: 'POST', body: JSON.stringify(data) });
  function controls() {
    const active = ACTIVE.has(store.snapshot?.status), locked = active || store.busy;
    el.run.disabled = !store.cloudLoaded || locked;
    el.pause.disabled = !active || store.busy;
    setText(el.pause, store.snapshot?.status === 'paused' ? '继续' : '暂停');
    el.reset.disabled = store.busy;
    el.scene.disabled = locked; el.case.disabled = locked || !store.cases.length;
    el.speed.disabled = locked;
    document.querySelectorAll('.coordinate-grid input, [data-pick="start"], [data-pick="goal"]').forEach(node => { node.disabled = locked; });
    el['requires-orientation'].disabled = locked;
    document.getElementById('goal-yaw').disabled = locked || !el['requires-orientation'].checked;
    setText(el['goal-yaw-label'], el['requires-orientation'].checked ? '到达朝向' : '参考朝向');
    document.getElementById('pick-obstacle').disabled = !store.cloudLoaded || store.busy;
    document.getElementById('clear-obstacles').disabled = !store.obstacles.length || store.busy;
    el.export.disabled = !store.id || store.busy;
    el.timeline.disabled = store.frames.length < 2;
    el['playback-play'].disabled = store.frames.length < 2;
    el.live.disabled = !store.replay;
  }
  function poseInputs(kind) {
    const inputs = ['x', 'y', 'z', 'yaw'].map(axis => document.getElementById(`${kind}-${axis}`));
    // 紧凑表单只缩短显示，不把原始样本坐标与姿态量化到两位小数。
    const values = inputs.map((input, index) => input.value === input.dataset.displayValue
      ? store[kind][index] : Number(input.value) * (index === 3 ? Math.PI / 180 : 1));
    if (!values.every(Number.isFinite) || inputs.some(input => input.value.trim() === '')) throw new Error('起点与目标的XYZ及朝向需要填写有限数值');
    return values;
  }
  function writePose(kind, pose) {
    store[kind] = [...pose];
    ['x', 'y', 'z', 'yaw'].forEach((axis, index) => {
      const input = document.getElementById(`${kind}-${axis}`), value = index === 3 ? pose[index] * 180 / Math.PI : pose[index];
      input.value = number(value, 2); input.dataset.displayValue = input.value; input.title = String(value);
    });
    view.setPose(view[kind], pose); view.requestRender();
  }
  function setPick(mode) {
    store.pickMode = store.pickMode === mode ? null : mode;
    view.pickMode = store.pickMode;
    document.body.dataset.picking = String(Boolean(store.pickMode));
    document.querySelectorAll('[data-pick]').forEach(button => { button.classList.toggle('active', button.dataset.pick === store.pickMode); button.setAttribute('aria-pressed', String(button.dataset.pick === store.pickMode)); });
    el['pick-hint'].hidden = !store.pickMode;
    setText(el['pick-hint'], store.pickMode === 'obstacle' ? '点击地面插入动态障碍 · Esc 退出' : `点击实际地面选择${store.pickMode === 'start' ? '起点' : '目标'} · Esc 退出`);
  }
  function stopPolling() { clearTimeout(store.pollTimer); store.pollTimer = null; store.pollRequest?.abort(); store.pollRequest = null; store.pollingEpoch++; }
  function stopPlayback() { store.playing = false; if (store.playbackRaf !== null) cancelAnimationFrame(store.playbackRaf); store.playbackRaf = null; setText(el['playback-play'], '播放回放'); }
  function clearSimulation() {
    stopPolling(); stopPlayback(); store.id = null; store.snapshot = null; store.frames = []; store.paths.clear(); store.locals.clear(); store.lastGlobal = null; store.replay = false; store.replayTime = 0; view.clearSimulation();
    showState(); controls();
  }
  function rememberPaths(snapshot) {
    const firstPath = !store.lastGlobal?.length;
    const history = snapshot.path_history || {};
    for (const [revision, points] of Object.entries(history)) if (Array.isArray(points) && points.length && !store.paths.get(String(revision))?.length) store.paths.set(String(revision), points);
    const revision = snapshot.path_revision ?? store.frames.at(-1)?.path_revision;
    if (Array.isArray(snapshot.global_path) && snapshot.global_path.length) {
      if (revision !== undefined) {
        if (!store.paths.has(String(revision))) store.paths.set(String(revision), snapshot.global_path);
        store.lastGlobal = store.paths.get(String(revision));
      } else store.lastGlobal = snapshot.global_path;
      // 同XY跨层必须绕行到楼梯或坡道；首次出线后按真实全局路线扩展画面。
      // 后续重规划与用户主动调整镜头时保持当前视角。
      const bounds = view.framingBounds;
      if (firstPath && !view.userCamera && (!bounds || snapshot.global_path.some(point =>
        point.slice(0, 3).some((value, axis) => value < bounds[axis] || value > bounds[axis + 3])))) {
        view.focusRoute(store.start, poseArray(snapshot.goal) || store.goal, snapshot.global_path);
      }
    }
    if (Array.isArray(snapshot.local_path)) {
      const t = snapshot.sim_time_sec ?? store.frames.at(-1)?.t;
      if (t !== undefined) store.locals.set(Number(t), snapshot.local_path);
    }
  }
  function ingest(snapshot) {
    if (!snapshot || typeof snapshot !== 'object') return;
    if (snapshot.id && store.id && String(snapshot.id) !== String(store.id)) return;
    if (appendFrameDelta(store.frames, snapshot)) view.traveled.uploadedFrames = 0;
    store.snapshot = snapshot; rememberPaths(snapshot); showState(); controls();
  }
  function showState() {
    const snapshot = store.snapshot, frames = store.frames, metrics = snapshot?.metrics || {};
    const status = snapshot?.status || 'ready'; el.status.dataset.status = status;
    setText(el.status, STATUS[status] || status);
    // 状态节点保留圆点，同时避免每次查询都重新创建控件。
    if (!el.status.querySelector('i')) el.status.prepend(document.createElement('i'));
    const lastTime = Number(snapshot?.sim_time_sec ?? frames.at(-1)?.t ?? 0);
    el.timeline.max = String(lastTime);
    if (!store.replay) { store.replayTime = lastTime; el.timeline.value = String(lastTime); }
    const history = store.replay ? replaySnapshot(frames, store.replayTime, snapshot?.goal || store.goal, store.paths) : null;
    const pose = history?.pose || poseArray(snapshot?.pose);
    const revision = history?.frame.path_revision ?? snapshot?.path_revision ?? frames.at(-1)?.path_revision ?? 'none';
    let globalPath = history ? history.path : (snapshot?.global_path?.length ? (store.paths.get(String(revision)) || snapshot.global_path) : store.lastGlobal);
    const retained = !store.replay && Boolean(globalPath?.length) && (!snapshot?.global_path?.length || !['running', 'succeeded'].includes(status));
    view.global.mesh.material.color.setHex(retained ? COLORS.retained : COLORS.global);
    document.querySelector('.line.global').style.background = retained ? '#ffba78' : '#a5e27e';
    view.global.set(globalPath || [], `${store.id}:${revision}:${globalPath?.length || 0}`);
    const localPath = history ? history.frame.local_path || store.locals.get(Number(history.frame.t)) || [] : snapshot?.local_path || [];
    view.local.set(localPath, `${store.id}:${history?.frame.t ?? lastTime}:${localPath.length}`);
    view.traveled.setFrames(frames, history ? history.index + 1 : frames.length);
    view.setPose(view.body, pose, history ? history.frame.orientation : snapshot?.orientation); view.setPose(view.goal, poseArray(snapshot?.goal) || store.goal); view.requestRender();
    const currentTime = history ? Number(history.frame.t) : lastTime;
    const distance = history ? history.distance : metrics.distance_remaining_m;
    setText(document.getElementById('metric-speed'), number(history ? history.speed : metrics.speed_mps ?? measuredSpeed(frames)));
    setText(document.getElementById('metric-distance'), number(distance));
    setText(document.getElementById('metric-time'), number(currentTime, 1));
    setText(document.getElementById('metric-plans'), metrics.global_plans ?? 0);
    setText(document.getElementById('wall-sec'), `${number(metrics.elapsed_wall_sec, 1)} 秒`);
    setText(document.getElementById('validation-failures'), `${metrics.motion_validation_failures ?? 0} 次`);
    setText(document.getElementById('plan-ms'), `${number(metrics.last_plan_ms, 2)} 毫秒`);
    setText(document.getElementById('control-ms'), `${number(metrics.last_control_ms, 2)} 毫秒`);
    setText(document.getElementById('memory-mb'), `${number(metrics.max_rss_mb, 1)} MiB`);
    setText(document.getElementById('native-cpu-percent'), `${number(metrics.native_cpu_percent, 1)} %`);
    setText(document.getElementById('native-cpu-seconds'), `${number(metrics.native_cpu_seconds, 3)} 秒`);
    setText(el['replay-time'], `${number(currentTime, 1)} / ${number(lastTime, 1)} 秒`);
    setText(el['frame-count'], `${frames.length.toLocaleString()} 帧实际记录${retained ? ' · 橙色路线等待更新' : ''}`);
    el['replay-badge'].hidden = !store.replay;
    const notes = [];
    if (history && !history.path) notes.push('该时刻全局路线未记录，未显示当前路线作为替代。');
    if (history && !localPath.length) notes.push('该帧未采集局部预测，已隐藏。');
    if (history) notes.push('速度由历史位姿计算；距离为到目标的直线距离。算法统计保留本轮最新值。');
    if (retained) notes.push('保留上一次全局路线（橙色），当前执行状态以算法反馈为准。');
    el['history-note'].hidden = !notes.length; setText(el['history-note'], notes.join(' '));
    if (snapshot?.message) feedback(snapshot.message, status === 'failed', true);
    controls();
  }
  async function poll() {
    const id = store.id, epoch = store.pollingEpoch;
    if (!id || store.pollRequest) return;
    const controller = new AbortController(); store.pollRequest = controller;
    try {
      const snapshot = await json(`/api/simulations/${encodeURIComponent(id)}?after_frame=${store.frames.length}`, { signal: controller.signal });
      if (id !== store.id || epoch !== store.pollingEpoch) return;
      try { ingest(snapshot); } catch (error) {
        if (!error.message.includes('轨迹帧有缺口')) throw error;
        const full = await json(`/api/simulations/${encodeURIComponent(id)}`, { signal: controller.signal });
        if (id === store.id && epoch === store.pollingEpoch) ingest(full);
      }
    } catch (error) { if (error.name !== 'AbortError') feedback(`状态读取失败：${error.message}`, true, true); }
    finally {
      if (id === store.id && epoch === store.pollingEpoch) {
        store.pollRequest = null;
        if (ACTIVE.has(store.snapshot?.status) || !store.snapshot) store.pollTimer = setTimeout(poll, 200);
      }
    }
  }
  async function command(action, extra = {}) {
    if (!store.id) return null;
    const id = store.id;
    // 控制回执先于下一次查询生效，撤销控制前仍在途的旧快照。
    stopPolling();
    let result;
    try { result = await post(`/api/simulations/${encodeURIComponent(id)}/control`, { action, ...extra }); }
    finally { if (id === store.id) store.pollTimer = setTimeout(poll, 200); }
    if (id !== store.id) return null;
    const snapshot = result.state && typeof result.state === 'object' ? result.state : result;
    if (snapshot.status) ingest(snapshot);
    return snapshot;
  }
  async function action(work) {
    if (store.busy) return;
    store.busy = true; store.operationError = null; controls();
    try { await work(); } catch (error) { feedback(error.message, true); }
    finally { store.busy = false; controls(); }
  }
  async function loadScene(id) {
    const selected = store.scenes.find(scene => scene.id === id);
    if (!selected) return;
    const token = ++store.loadToken;
    clearSimulation(); store.scene = selected; store.cloudLoaded = false; store.obstacles = []; view.setObstacles([]); setPick(null);
    el['viewport-empty'].hidden = false; el['viewport-empty'].querySelector('strong').textContent = '正在读取地图点云';
    setText(el['scene-name'], selected.name); setText(el['scene-description'], selected.description || '');
    setText(el['cloud-count'], '载入中');
    setText(el['height-hint'], `地图选点的地面高度加 ${number(selected.body_to_support_z_m ?? 0.4, 2)} 米，得到机体 Z。预设已使用机体坐标。`);
    writePose('start', poseArray(selected.default_start) || [0, 0, 0.4, 0]);
    writePose('goal', poseArray(selected.default_goal) || [7, 0, 0.4, 0]);
    store.cases = []; store.caseId = null; el.case.replaceChildren(new Option('自定义起点与目标', '')); controls();
    try {
      const [cloudResponse, caseResult] = await Promise.all([fetch(`/api/scenes/${encodeURIComponent(id)}/cloud`), json(`/api/scenes/${encodeURIComponent(id)}/cases`)]);
      if (!cloudResponse.ok) throw new Error(`地图点云读取失败（HTTP ${cloudResponse.status}）`);
      const buffer = await cloudResponse.arrayBuffer();
      if (token !== store.loadToken) return;
      const count = view.setCloud(buffer, selected); store.cloudLoaded = true;
      const totalPoints = Number(selected.point_count) || count;
      const compactCount = value => value >= 10000 ? `${(value / 10000).toFixed(1)}万` : value.toLocaleString();
      setText(el['cloud-count'], count === totalPoints ? `${count.toLocaleString()} 点` : `${compactCount(count)} / ${compactCount(totalPoints)} 点`);
      el['cloud-count'].title = `显示 ${count.toLocaleString()} 点，地图共 ${totalPoints.toLocaleString()} 点`;
      store.cases = caseResult.cases || [];
      for (const sample of store.cases) el.case.add(new Option(sample.name, sample.id));
      const defaultCase = store.cases.find(sample => ['start', 'goal'].every(kind => {
        const pose = poseArray(sample[kind]);
        return pose && pose.every((value, index) => Math.abs(value - store[kind][index]) < 1e-6);
      }));
      if (defaultCase) { store.caseId = defaultCase.id; el.case.value = defaultCase.id; }
      view.focusRoute(store.start, store.goal);
      el['viewport-empty'].hidden = true;
      feedback('地图已就绪。选择预设样本，或分别设置起点与目标。'); controls();
    } catch (error) {
      if (token !== store.loadToken) return;
      el['viewport-empty'].querySelector('strong').textContent = '场景读取失败';
      el['viewport-empty'].querySelector('.loading-ring').style.animation = 'none';
      feedback(error.message, true); controls();
    }
  }
  async function handlePick(point, mode) {
    if (!point) { feedback('没有命中地图点。请放大并点击可见地面。'); return; }
    if (mode === 'obstacle') {
      await action(async () => {
        const radius = Number(document.getElementById('obstacle-radius').value), height = Number(document.getElementById('obstacle-height').value);
        if (!Number.isFinite(radius) || !Number.isFinite(height) || radius <= 0 || height <= 0) throw new Error('障碍半径和高度必须大于零');
        const obstacle = { x: point[0], y: point[1], radius, height };
        if (store.id && ACTIVE.has(store.snapshot?.status)) await command('add_obstacle', { obstacle });
        store.obstacles.push({ ...obstacle, z: point[2] }); view.setObstacles(store.obstacles);
        feedback(`已添加动态障碍：X ${number(point[0])}，Y ${number(point[1])}，半径 ${number(radius)} 米。`);
      });
    } else {
      const pose = [point[0], point[1], point[2] + Number(store.scene.body_to_support_z_m ?? 0.4), store[mode][3]];
      if (store.snapshot && !ACTIVE.has(store.snapshot.status)) clearSimulation();
      writePose(mode, pose); if (mode === 'start') store.caseId = null; el.case.value = ''; setPick(null);
      feedback(`已选择${mode === 'start' ? '起点' : '目标'}：X ${number(pose[0])}，Y ${number(pose[1])}，机体 Z ${number(pose[2])} 米。`);
    }
  }
  try { view = new SimulationView(document.getElementById('viewport'), handlePick); }
  catch (error) { feedback(`三维画布初始化失败：${error.message}`, true); el['viewport-empty'].querySelector('strong').textContent = '三维画布不可用'; return; }
  el.scene.addEventListener('change', () => loadScene(el.scene.value));
  el.case.addEventListener('change', () => {
    const sample = store.cases.find(item => item.id === el.case.value);
    store.caseId = sample?.id || null;
    if (sample) { if (store.snapshot) clearSimulation(); writePose('start', poseArray(sample.start)); writePose('goal', poseArray(sample.goal)); view.focusRoute(store.start, store.goal); feedback(sample.description || `已加载样本：${sample.name}`); }
  });
  document.querySelectorAll('.coordinate-grid input').forEach(input => input.addEventListener('change', () => {
    const kind = input.closest('[data-pose]').dataset.pose;
    try { if (store.snapshot && !ACTIVE.has(store.snapshot.status)) clearSimulation(); store[kind] = poseInputs(kind); if (kind === 'start') store.caseId = null; view.setPose(view[kind], store[kind]); view.requestRender(); el.case.value = ''; }
    catch (error) { feedback(error.message, true); }
  }));
  document.querySelectorAll('[data-pick]').forEach(button => button.addEventListener('click', () => setPick(button.dataset.pick)));
  document.addEventListener('keydown', event => { if (event.key === 'Escape') setPick(null); });
  document.getElementById('fit').addEventListener('click', () => view.fit());
  for (const mode of ['perspective', 'top']) document.getElementById(`view-${mode}`).addEventListener('click', () => {
    document.getElementById('view-perspective').setAttribute('aria-pressed', String(mode === 'perspective'));
    document.getElementById('view-top').setAttribute('aria-pressed', String(mode === 'top'));
    view.setView(mode === 'top');
  });
  el['requires-orientation'].addEventListener('change', controls);
  el.speed.addEventListener('input', () => { el['speed-value'].firstChild.textContent = `${Number(el.speed.value).toFixed(1)} `; });
  el.run.addEventListener('click', () => action(async () => {
    const start = poseInputs('start'), goal = poseInputs('goal');
    clearSimulation(); store.start = start; store.goal = goal; setPick(null); view.focusRoute(start, goal);
    feedback('正在创建模拟，等待原生算法首轮规划…');
    const obstacles = store.obstacles.map(({ x, y, radius, height }) => ({ x, y, radius, height }));
    const result = await post('/api/simulations', { scene_id: store.scene.id, case_id: store.caseId, start, goal, requires_orientation: el['requires-orientation'].checked, speed_mps: Number(el.speed.value), obstacles });
    if (!result.id) throw new Error('模拟服务没有返回本轮标识');
    store.id = String(result.id);
    if (result.state && typeof result.state === 'object') ingest(result.state);
    await poll();
  }));
  el.pause.addEventListener('click', () => action(async () => { await command(store.snapshot?.status === 'paused' ? 'resume' : 'pause'); }));
  el.reset.addEventListener('click', () => action(async () => {
    if (store.id) await command('reset');
    clearSimulation(); store.obstacles = []; view.setObstacles([]);
    feedback('已重置本轮模拟，可调整起点、目标与速度。');
  }));
  document.getElementById('clear-obstacles').addEventListener('click', () => action(async () => {
    if (store.id && ACTIVE.has(store.snapshot?.status)) await command('clear_obstacles');
    store.obstacles = []; view.setObstacles([]); feedback('已移除本轮添加的动态障碍。');
  }));
  el.timeline.addEventListener('input', () => { stopPlayback(); store.replay = true; store.replayTime = Number(el.timeline.value); showState(); });
  el.live.addEventListener('click', () => { stopPlayback(); store.replay = false; showState(); });
  el['playback-play'].addEventListener('click', () => {
    if (store.playing) { stopPlayback(); return; }
    if (!store.frames.length) return;
    const end = Number(store.frames.at(-1).t);
    if (!store.replay || store.replayTime >= end) store.replayTime = Number(store.frames[0].t) || 0;
    store.replay = true; store.playing = true; setText(el['playback-play'], '暂停回放');
    let previous = performance.now();
    const tick = now => {
      if (!store.playing) return;
      const endTime = Number(store.frames.at(-1)?.t || 0);
      store.replayTime = Math.min(endTime, store.replayTime + (now - previous) / 1000 * Number(el['playback-speed'].value)); previous = now;
      el.timeline.value = String(store.replayTime); showState();
      if (store.replayTime >= endTime) stopPlayback(); else store.playbackRaf = requestAnimationFrame(tick);
    };
    store.playbackRaf = requestAnimationFrame(tick);
  });
  el.export.addEventListener('click', () => action(async () => {
    const result = await json(`/api/simulations/${encodeURIComponent(store.id)}/result`);
    const blob = new Blob([JSON.stringify(result, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = `导航模拟结果-${store.id}.json`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    feedback('本轮完整模拟结果已导出。');
  }));
  window.addEventListener('beforeunload', () => { stopPolling(); stopPlayback(); });
  json('/api/scenes').then(result => {
    store.scenes = result.scenes || [];
    if (!store.scenes.length) throw new Error('服务尚未提供可用场景');
    el.scene.replaceChildren(...store.scenes.map(scene => new Option(scene.name, scene.id)));
    const selected = result.default_scene || store.scenes[0].id; el.scene.value = selected; return loadScene(selected);
  }).catch(error => { feedback(error.message, true); el['viewport-empty'].querySelector('strong').textContent = '场景服务暂不可用'; });
}

if (typeof document !== 'undefined') initialize();
