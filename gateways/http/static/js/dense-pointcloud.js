import * as THREE from './three.module.min.js';

const CHUNK_POINTS = 65_536;
const GEOMETRY_POINTS = CHUNK_POINTS * 4;

export function decodeDenseChunk(metadata, buffer, epoch, offset) {
  const ranges = [];
  let removed = 0;
  let rangeBytes = 0;
  if (metadata.base_epoch !== undefined) {
    rangeBytes = metadata.removed_range_bytes;
    if (!epoch || metadata.base_epoch !== epoch || Number(metadata.base_offset_points) !== offset
        || !Number.isInteger(rangeBytes) || rangeBytes < 0 || rangeBytes > 65536 || rangeBytes % 16
        || !(buffer instanceof ArrayBuffer) || rangeBytes > buffer.byteLength) {
      throw new Error('完整点云增量基准非法');
    }
    const view = new DataView(buffer);
    let end = 0;
    for (let at = 0; at < rangeBytes; at += 16) {
      const start = Number(view.getBigUint64(at, true));
      const count = Number(view.getBigUint64(at + 8, true));
      if (!Number.isSafeInteger(start) || !Number.isSafeInteger(count) || count <= 0 || start < end || start + count > offset) {
        throw new Error('完整点云删除区间非法');
      }
      ranges.push([start, count]);
      end = start + count;
      removed += count;
    }
  }
  const total = Number(metadata.total_points);
  const start = Number(metadata.offset_points);
  if ((epoch && metadata.epoch !== epoch && metadata.base_epoch !== epoch) || start !== offset - removed) {
    throw new Error('完整点云分块基线或位置不符');
  }
  if (!(buffer instanceof ArrayBuffer) || (buffer.byteLength - rangeBytes) % 12
      || buffer.byteLength - rangeBytes > CHUNK_POINTS * 12
      || metadata.point_count !== (buffer.byteLength - rangeBytes) / 12 || total < start + metadata.point_count
      || (metadata.point_count === 0 && start !== total)) {
    throw new Error('完整点云分块载荷长度不符');
  }
  const positions = new Float32Array(buffer, rangeBytes);
  return { positions, total, metadata, ranges };
}

function extendPointCloudBounds(bounds, positions) {
  // 沿原有包围盒计算跳过不可显示点，不增加解析预扫描或改动分块槽位。
  for (let offset = 0; offset < positions.length; offset += 3) {
    const x = positions[offset], y = positions[offset + 1], z = positions[offset + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    bounds.min.x = Math.min(bounds.min.x, x); bounds.max.x = Math.max(bounds.max.x, x);
    bounds.min.y = Math.min(bounds.min.y, y); bounds.max.y = Math.max(bounds.max.y, y);
    bounds.min.z = Math.min(bounds.min.z, z); bounds.max.z = Math.max(bounds.max.z, z);
  }
}

function createMaterial(size) {
  const material = new THREE.PointsMaterial({ size, sizeAttenuation: false });
  // 与对照页相同的高度色相；直接在顶点着色器计算，避免每点再存一份颜色数组。
  material.onBeforeCompile = shader => {
    shader.vertexShader = `varying vec3 denseColor;\n${shader.vertexShader}`;
    shader.vertexShader = shader.vertexShader.replace('#include <begin_vertex>', `
      #include <begin_vertex>
      float denseHeight = clamp((position.z + 0.5) / 4.0, 0.0, 1.0);
      float denseHue = 0.62 - denseHeight * 0.6;
      vec3 denseRgb = clamp(abs(fract(vec3(denseHue) + vec3(0.0, 2.0/3.0, 1.0/3.0)) * 6.0 - 3.0) - 1.0, 0.0, 1.0);
      denseColor = vec3(0.62) + 0.68 * (denseRgb - 0.5) * 0.76;
    `);
    shader.fragmentShader = `varying vec3 denseColor;\n${shader.fragmentShader}`;
    shader.fragmentShader = shader.fragmentShader.replace('#include <color_fragment>', 'diffuseColor.rgb *= denseColor;');
  };
  material.customProgramCacheKey = () => 'dense-height-v1';
  return material;
}

export class DensePointCloudLayer {
  constructor({ size = 1.25, onChange = () => {}, fetchChunk = (...args) => fetch(...args) } = {}) {
    this.group = new THREE.Group();
    this.group.name = 'complete-scan-pointcloud';
    this.group.visible = false;
    this.material = createMaterial(size);
    this.onChange = onChange;
    this.fetchChunk = fetchChunk;
    this.count = 0;
    this.total = 0;
    this.epoch = null;
    this.metadata = null;
    this.bounds = null;
    this.displaySource = null;
    this._active = false;
    this._serial = 0;
    this._timer = null;
    this._abort = null;
  }

  append({ positions, total, metadata, ranges = [] }) {
    const delta = metadata.base_epoch !== undefined;
    if (delta) this._removeRanges(ranges);
    if (!delta && !positions.length && this.metadata?.revision === metadata.revision && total === this.total) return;
    let offset = 0;
    while (offset < positions.length) {
      let mesh = this.group.children.at(-1);
      if (!mesh || mesh.geometry.drawRange.count === GEOMETRY_POINTS) {
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(GEOMETRY_POINTS * 3), 3).setUsage(THREE.DynamicDrawUsage));
        geometry.setDrawRange(0, 0);
        geometry.boundingBox = new THREE.Box3();
        mesh = new THREE.Points(geometry, this.material);
        this.group.add(mesh);
      }
      const geometry = mesh.geometry;
      const attribute = geometry.getAttribute('position');
      const start = geometry.drawRange.count * 3;
      const length = Math.min(attribute.array.length - start, positions.length - offset);
      const data = positions.subarray(offset, offset + length);
      attribute.array.set(data, start);
      attribute.addUpdateRange(start, length);
      attribute.needsUpdate = true;
      geometry.setDrawRange(0, geometry.drawRange.count + length / 3);
      extendPointCloudBounds(geometry.boundingBox, data);
      geometry.boundingSphere = geometry.boundingBox.getBoundingSphere(new THREE.Sphere());
      this.count += length / 3;
      offset += length;
    }
    this.epoch = metadata.epoch;
    this.metadata = metadata;
    this.total = total;
    const bounds = new THREE.Box3();
    for (const mesh of this.group.children) bounds.union(mesh.geometry.boundingBox);
    this.bounds = bounds.isEmpty() ? null : bounds.getBoundingSphere(new THREE.Sphere());
    this.onChange({ count: this.bounds ? this.count : 0, total, metadata, bounds: this.bounds, loading: this.count < total });
  }

  _removeRanges(ranges) {
    if (!ranges.length) return;
    const meshes = this.group.children;
    let read = 0, write = 0;
    const copyUntil = end => {
      while (read < end) {
        const sourceIndex = Math.floor(read / GEOMETRY_POINTS), targetIndex = Math.floor(write / GEOMETRY_POINTS);
        const sourceStart = read % GEOMETRY_POINTS, targetStart = write % GEOMETRY_POINTS;
        const count = Math.min(end - read, GEOMETRY_POINTS - sourceStart, GEOMETRY_POINTS - targetStart);
        if (read !== write) {
          const source = meshes[sourceIndex].geometry.getAttribute('position').array;
          meshes[targetIndex].geometry.getAttribute('position').array.set(source.subarray(sourceStart * 3, (sourceStart + count) * 3), targetStart * 3);
        }
        read += count; write += count;
      }
    };
    for (const [start, count] of ranges) { copyUntil(start); read += count; }
    copyUntil(this.count);
    this.count = write;
    const firstChanged = Math.floor(ranges[0][0] / GEOMETRY_POINTS);
    for (let index = meshes.length - 1; index >= firstChanged; index--) {
      const mesh = meshes[index], geometry = mesh.geometry;
      const count = Math.max(0, Math.min(GEOMETRY_POINTS, write - index * GEOMETRY_POINTS));
      if (!count) { geometry.dispose(); this.group.remove(mesh); continue; }
      const attribute = geometry.getAttribute('position');
      geometry.setDrawRange(0, count);
      const start = index === firstChanged ? ranges[0][0] % GEOMETRY_POINTS : 0;
      // 只上传压缩后实际移动的存活点，并保留同帧尚未上传的追加或删除区间。
      if (start < count) {
        attribute.addUpdateRange(start * 3, (count - start) * 3);
        attribute.needsUpdate = true;
      }
      geometry.boundingBox.makeEmpty();
      extendPointCloudBounds(geometry.boundingBox, attribute.array.subarray(0, count * 3));
      geometry.boundingSphere = geometry.boundingBox.getBoundingSphere(new THREE.Sphere());
    }
  }

  clear() {
    for (const mesh of this.group.children) mesh.geometry.dispose();
    this.group.clear();
    this.count = this.total = 0;
    this.epoch = this.metadata = this.bounds = null;
  }

  setDisplaySource(header, { reset = false } = {}) {
    const next = header ? { source: header.source, source_epoch: header.source_epoch,
      frame_id: header.frame_id, stream_generation: header.stream_generation, assigned_source: header.assigned_source } : null;
    const same = next && this.displaySource && ['source', 'source_epoch', 'frame_id', 'stream_generation']
      .every(key => next[key] === this.displaySource[key]);
    if (same && !reset) return;
    if (!next && !this.displaySource && !reset) return;
    this.displaySource = next;
    this.reset();
  }

  reset() {
    const active = this._active;
    this.stop();
    this.clear();
    this.onChange({ count: 0, total: 0, loading: true });
    if (active) this.start();
  }

  start() {
    if (this._active || !this.displaySource) return;
    this._active = true;
    this.group.visible = true;
    this.onChange({ count: this.count, total: this.total, metadata: this.metadata, bounds: this.bounds, loading: !this.metadata || this.count < this.total });
    this._poll(++this._serial);
  }

  stop() {
    this._active = false;
    this._serial += 1;
    clearTimeout(this._timer);
    this._timer = null;
    this._abort?.abort();
    this._abort = null;
    this.group.visible = false;
  }

  _assignedComplete() {
    // 指定保存资产在同一安装代次内不可变；后续失效由地图/点云来源事件驱动。
    return this.displaySource?.assigned_source && this.metadata && this.count === this.total;
  }

  async _poll(serial) {
    if (!this._active || serial !== this._serial || this._assignedComplete()) return;
    const controller = new AbortController();
    this._abort = controller;
    const timeout = setTimeout(() => controller.abort(), 15000);
    let delay = 500;
    const expectedDisplaySource = this.displaySource;
    try {
      const query = new URLSearchParams({ epoch: this.epoch || '', offset: String(this.count), delta: expectedDisplaySource.assigned_source ? '0' : '1',
        source_epoch: expectedDisplaySource.source_epoch });
      const response = await this.fetchChunk(`/api/pointcloud/dense?${query}`, { signal: controller.signal, cache: 'no-store' });
      if (!this._active || serial !== this._serial) return;
      if (response.status === 409) {
        this.clear();
        this.onChange({ count: 0, total: 0, loading: true });
        delay = expectedDisplaySource.assigned_source ? 500 : 0;
      } else if (!response.ok) {
        const body = await response.json().catch(() => ({}));
        if (!this._active || serial !== this._serial) return;
        this.clear();
        throw new Error(body.error?.message || body.message || `完整点云读取失败（${response.status}）`);
      } else {
        const metadata = JSON.parse(response.headers.get('X-Nuwax-Pointcloud') || 'null');
        const buffer = await response.arrayBuffer();
        if (!this._active || serial !== this._serial) return;
        this.append(decodeDenseChunk(metadata, buffer, this.epoch, this.count));
        delay = this.count < this.total ? 0 : 500;
      }
    } catch (error) {
      if (!this._active || serial !== this._serial) return;
      this.onChange({ count: this.count, total: this.total, error: error.name === 'AbortError' ? '完整点云读取超时' : String(error.message || error) });
      delay = 2000;
    } finally {
      clearTimeout(timeout);
      if (this._active && serial === this._serial) {
        this._abort = null;
        this._timer = this._assignedComplete() ? null : setTimeout(() => this._poll(serial), delay);
      }
    }
  }

  dispose() {
    this.stop();
    this.clear();
    this.group.removeFromParent();
    this.material.dispose();
  }
}
