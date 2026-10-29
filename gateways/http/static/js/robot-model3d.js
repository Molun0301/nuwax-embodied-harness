import * as THREE from './three.module.min.js';

const POLL_MS = 100;
const MANIFEST_POLL_MS = 2000;
const MAX_STATE_AGE_MS = 500;

function matrixValues(value) {
  return Array.isArray(value) && value.length === 16 && value.every(Number.isFinite);
}

function disposeGroup(group) {
  if (!group) return;
  const resources = new Set();
  group.traverse(object => {
    if (object.geometry) resources.add(object.geometry);
    if (Array.isArray(object.material)) object.material.forEach(value => resources.add(value));
    else if (object.material) resources.add(object.material);
  });
  resources.forEach(resource => resource.dispose());
  group.removeFromParent();
  return resources;
}

function typedView(buffer, Type, offset, count) {
  if (!Number.isSafeInteger(offset) || offset < 0 || offset % Type.BYTES_PER_ELEMENT
      || !Number.isSafeInteger(count) || count < 0
      || offset + count * Type.BYTES_PER_ELEMENT > buffer.byteLength) {
    throw new Error('机器人网格数据范围无效');
  }
  return new Type(buffer, offset, count);
}

// 量化仅影响位置精度；索引保留全部三角面，不能用凸包代替外观。
export function decodeRobotGeometry(descriptor, buffer) {
  const { vertex_count: count, position_offset_m: offset, position_scale_m: scale } = descriptor;
  if (!Number.isSafeInteger(count) || count < 3 || !Array.isArray(offset) || !Array.isArray(scale)
      || offset.length !== 3 || scale.length !== 3 || !offset.every(Number.isFinite)
      || !scale.every(value => Number.isFinite(value) && value >= 0)
      || descriptor.index_count % 3 || !['uint16', 'uint32'].includes(descriptor.index_type)) {
    throw new Error('机器人网格描述无效');
  }
  const quantized = typedView(buffer, Uint16Array, descriptor.position_byte_offset, count * 3);
  const indices = typedView(buffer, descriptor.index_type === 'uint16' ? Uint16Array : Uint32Array,
    descriptor.index_byte_offset, descriptor.index_count);
  for (const index of indices) if (index >= count) throw new Error('机器人网格索引越界');
  const positions = new Float32Array(quantized.length);
  for (let i = 0; i < positions.length; i++) positions[i] = offset[i % 3] + quantized[i] * scale[i % 3];
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

export class RobotModelLayer {
  constructor({ onChange = () => {}, onGeometry = () => {}, onStatus = () => {}, fetcher = (...args) => fetch(...args),
    now = () => performance.now() } = {}) {
    this.root = new THREE.Group();
    this.root.name = 'robot-visual-model';
    this.onChange = onChange;
    this.onGeometry = onGeometry;
    this.onStatus = onStatus;
    this.fetcher = fetcher;
    this.now = now;
    this.manifest = null;
    this.links = new Map();
    this.content = null;
    this.status = { available: false, mode: 'loading', message: '正在加载机器人外观' };
    this.running = false;
    this.disposed = false;
    this.generation = 0;
    this.meshGeneration = 0;
    this.controllers = new Set();
    this.loadedAt = null;
    this.loadDurationMs = null;
    this.state = null;
    this._manifestTask = null;
    this._stateTask = null;
  }

  _status(update) {
    this.status = { ...this.status, ...update };
    this.onStatus(this.status);
    this.onChange();
  }

  async _request(url, binary = false) {
    const controller = new AbortController();
    this.controllers.add(controller);
    const timeout = setTimeout(() => controller.abort(), binary ? 30000 : 10000);
    try {
      const response = await this.fetcher(url, { signal: controller.signal, cache: binary ? 'default' : 'no-store' });
      if (!response.ok) throw new Error(`机器人模型请求失败（${response.status}）`);
      return await (binary ? response.arrayBuffer() : response.json());
    } finally {
      clearTimeout(timeout);
      this.controllers.delete(controller);
    }
  }

  start() {
    if (this.running || this.disposed) return;
    this.running = true;
    this.refresh();
    this.pollState();
    this._watchdog = setInterval(() => this.checkFreshness(), POLL_MS);
  }

  stop() {
    this.running = false;
    clearTimeout(this._manifestTimer);
    clearTimeout(this._stateTimer);
    clearInterval(this._watchdog);
    clearTimeout(this._expiryTimer);
    for (const controller of this.controllers) controller.abort();
    this.useReference('状态更新已暂停');
  }

  refresh() {
    if (this._manifestTask || this.disposed) return this._manifestTask;
    clearTimeout(this._manifestTimer);
    this._manifestRequestedAt = this.now();
    const generation = this.generation;
    this._manifestTask = this._refresh(generation).finally(() => {
      this._manifestTask = null;
      if (this.running) this._manifestTimer = setTimeout(() => this.refresh(), MANIFEST_POLL_MS);
    });
    return this._manifestTask;
  }

  async _refresh(generation) {
    try {
      const manifest = await this._request('/api/robot-model');
      if (this.disposed || generation !== this.generation) return;
      this.onGeometry(manifest.geometry);
      if (!manifest.available) {
        this.clearModel();
        this._status({ available: false, mode: 'unavailable', message: manifest.message || '机器人模型未配置' });
        return;
      }
      if (manifest.asset_id === this.manifest?.asset_id && this.content) return;
      // 配置变化后先移除旧模型，异步旧请求不允许覆盖新配置。
      this.clearModel();
      this.manifest = manifest;
      this._assetAdoptedAt = this.now();
      this._status({ available: false, mode: 'loading', message: '正在加载机器人外观' });
      const started = this.now();
      const buffer = await this._request(manifest.mesh_url, true);
      if (this.disposed || generation !== this.generation || this.manifest !== manifest) return;
      await this.loadMesh(manifest, buffer, generation);
      if (this.content && this.manifest === manifest) {
        this.loadedAt = this.now();
        this.loadDurationMs = this.loadedAt - started;
        this.useReference('等待实时关节状态');
      }
    } catch (error) {
      if (error.code === 'robot_model_cancelled') return;
      if (this.disposed || generation !== this.generation || error.name === 'AbortError' && !this.running) return;
      this.clearModel();
      this._status({ available: false, mode: 'unavailable', message: error.message });
    }
  }

  async loadMesh(manifest, buffer, generation = this.generation) {
    if (manifest.format !== 'nuwax-robot-mesh-v1') throw new Error('机器人网格格式不受支持');
    const geometries = new Map();
    const links = new Map();
    const group = new THREE.Group();
    const meshGeneration = this.meshGeneration;
    try {
      for (let i = 0; i < manifest.geometries.length; i++) {
        const descriptor = manifest.geometries[i];
        geometries.set(descriptor.id, decodeRobotGeometry(descriptor, buffer));
        // 大模型分批准备，给点云绘制和输入事件留出执行机会。
        if (i % 8 === 7) await new Promise(resolve => setTimeout(resolve, 0));
        if (this.disposed || generation !== this.generation || meshGeneration !== this.meshGeneration) {
          const error = new Error('机器人模型加载已取消');
          error.code = 'robot_model_cancelled';
          throw error;
        }
      }
      for (const visual of manifest.visuals) {
        if (!geometries.has(visual.geometry) || !matrixValues(visual.matrix)) throw new Error('机器人外观变换无效');
        let link = links.get(visual.link);
        if (!link) {
          link = new THREE.Group();
          link.name = visual.link;
          link.matrixAutoUpdate = false;
          links.set(visual.link, link);
          group.add(link);
        }
        const rgba = visual.rgba || [0.65, 0.7, 0.78, 1];
        const material = new THREE.MeshPhongMaterial({
          color: new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2]),
          transparent: true, opacity: Math.min(0.65, (rgba[3] ?? 1) * 0.65),
          depthWrite: false, side: THREE.FrontSide, shininess: 35,
        });
        const mesh = new THREE.Mesh(geometries.get(visual.geometry), material);
        mesh.matrixAutoUpdate = false;
        mesh.matrix.fromArray(visual.matrix);
        mesh.renderOrder = 32;
        link.add(mesh);
      }
      const ambient = new THREE.HemisphereLight(0xffffff, 0x4b5970, 2.1);
      ambient.position.set(0, 0, 5);
      const key = new THREE.DirectionalLight(0xffffff, 2.2);
      key.position.set(3, -4, 6);
      group.add(ambient, key, key.target);
      this.content = group;
      this.links = links;
      this.manifest = manifest;
      this._assetAdoptedAt = this.now();
      this.root.add(group);
      if (!this.applyTransforms(manifest.reference_transforms)) throw new Error('模型参考姿态不完整');
      this._status({ available: true, mode: 'reference', message: '参考姿态 · 等待实时关节状态' });
    } catch (error) {
      const released = disposeGroup(group);
      for (const geometry of geometries.values()) if (!released.has(geometry)) geometry.dispose();
      if (this.content === group) { this.content = null; this.links.clear(); }
      throw error;
    }
  }

  applyTransforms(transforms) {
    if (!transforms || [...this.links.keys()].some(name => !matrixValues(transforms[name]))) return false;
    for (const [name, link] of this.links) {
      link.matrix.fromArray(transforms[name]);
      link.matrixWorldNeedsUpdate = true;
    }
    return true;
  }

  applyState(state, requestStarted = this.now()) {
    if (!state.available || state.asset_id !== this.manifest?.asset_id) {
      if (this.manifest && requestStarted < this._assetAdoptedAt) return false;
      // 新版本到达时不把旧网格套在新机器人上。
      const previousId = this.manifest?.asset_id;
      this.clearModel();
      this._status({ available: false, mode: 'unavailable', message: state.message || '模型配置已变化，正在更新' });
      // 缺模型时继续按清单周期重试，不能被每个状态响应放大成高频资产检查。
      if (this.running && state.available && (previousId || this.now() - (this._manifestRequestedAt ?? -Infinity) >= MANIFEST_POLL_MS)) this.refresh();
      return false;
    }
    if (!this.content) return false;
    if (!this.applyTransforms(state.link_transforms)) {
      this.useReference('连杆姿态不完整');
      return false;
    }
    this.state = state;
    this.stateReceivedAt = this.now();
    // 请求开始计时覆盖网络传输耗时，不能把到达浏览器的旧数据重新续期。
    this.stateExpiresAt = requestStarted + Math.min(MAX_STATE_AGE_MS, Math.max(0, Number(state.valid_for_sec) || 0) * 1000);
    clearTimeout(this._expiryTimer);
    if (state.mode === 'live' && this.now() < this.stateExpiresAt && this.poseMatches(state)) {
      this._expiryTimer = setTimeout(() => this.checkFreshness(), Math.max(1, this.stateExpiresAt - this.now()));
      this._status({ available: true, mode: 'live', message: '真实外观 · 实时关节' });
    } else {
      this.useReference(!this.poseMatches(state) ? '地图位姿与关节时刻不匹配' : state.message || '关节状态已过期');
    }
    return true;
  }

  checkFreshness() {
    if (this.status.mode !== 'live') return;
    if (this.now() >= this.stateExpiresAt) this.useReference('关节状态已过期');
    else if (!this.poseMatches(this.state)) this.useReference('地图位姿更新已中断');
  }

  setPoseSource(pose) {
    if (pose !== this.poseSource && (pose?.timestamp !== this.poseSource?.timestamp
        || pose?.frame_id !== this.poseSource?.frame_id || !pose?.timestamp)) this.poseReceivedAt = this.now();
    this.poseSource = pose;
    if (this.status.mode === 'live' && !this.poseMatches(this.state)) this.useReference('地图位姿与关节时刻不匹配');
  }

  poseMatches(state) {
    // 源时钟可来自慢速仿真，真实采集时效与配对由服务端统一判断。
    // 这里另检查浏览器定位流是否中断，不用墙钟重算原始源时间。
    if (!this.poseSource) return true;
    if (state?.pose_frame_id && this.poseSource.frame_id && state.pose_frame_id !== this.poseSource.frame_id) return false;
    return this.now() - this.poseReceivedAt <= MAX_STATE_AGE_MS;
  }

  useReference(message) {
    if (!this.content) return;
    this.applyTransforms(this.manifest.reference_transforms);
    this._status({ available: true, mode: 'reference', message: `参考姿态 · ${message}` });
  }

  pollState() {
    if (this._stateTask || this.disposed) return this._stateTask;
    const started = this.now();
    const generation = this.generation;
    this._stateTask = this._request('/api/robot-model/state').then(state => {
      if (!this.disposed && generation === this.generation) this.applyState(state, started);
    }).catch(error => {
      if (!this.disposed && !(error.name === 'AbortError' && !this.running)) this.useReference('关节状态连接中断');
    }).finally(() => {
      this._stateTask = null;
      if (this.running) this._stateTimer = setTimeout(() => this.pollState(), Math.max(0, POLL_MS - (this.now() - started)));
    });
    return this._stateTask;
  }

  clearModel() {
    this.meshGeneration++;
    clearTimeout(this._expiryTimer);
    disposeGroup(this.content);
    this.content = null;
    this.manifest = null;
    this.state = null;
    this.links.clear();
  }

  dispose() {
    this.stop();
    this.disposed = true;
    this.generation++;
    this.clearModel();
    this.root.removeFromParent();
  }
}
