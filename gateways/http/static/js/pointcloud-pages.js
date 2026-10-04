import * as THREE from './three.module.min.js';

// Canonical slots are owned by the producer. A page replacement never changes
// another source page, and source XYZ is never flattened into a whole-map array.
export const POINT_CLOUD_PAGE_POINTS = 1024;
export const POINT_CLOUD_RENDER_POINTS = POINT_CLOUD_PAGE_POINTS * 16;
const PAGES_PER_SLAB = 16;
const BIN = 0.08;
const BAND = 0.12;
const COLUMN = 0.18;
const VERTICAL_SPAN = 0.45;
const EMPTY = () => [Infinity, Infinity, Infinity, -Infinity, -Infinity, -Infinity];
const DEFAULT_CONFIG = {
  stylePresets: {
    height: { preserveSource: true, colorMode: 'height', size: 1.35 },
    mono: { colorMode: 'mono', size: 1.1 },
    singleFloor: { colorMode: 'singleFloor', floorFilter: 'story', liveSpacingM: 0.08, size: 1.35 },
    ground: { colorMode: 'ground', floorFilter: 'ground', liveSpacingM: 0.08, size: 1.55 },
    structure: { colorMode: 'structure', floorFilter: 'story', liveSpacingM: 0.08, size: 1.35 },
    full: { denseSource: true },
  },
  heightColorStops: [[56, 189, 248], [34, 211, 238], [74, 222, 128], [250, 204, 21], [251, 113, 133]],
  floorColors: { ground: [56, 189, 248], context: [203, 213, 225], structure: [248, 250, 252], object: [251, 191, 36] },
};
function extend(box, x, y, z) {
  if (x < box[0]) box[0] = x; if (y < box[1]) box[1] = y; if (z < box[2]) box[2] = z;
  if (x > box[3]) box[3] = x; if (y > box[4]) box[4] = y; if (z > box[5]) box[5] = z;
  return box;
}
function union(a, b) {
  if (!a) return b || null; if (!b) return a;
  return [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.min(a[2], b[2]),
    Math.max(a[3], b[3]), Math.max(a[4], b[4]), Math.max(a[5], b[5])];
}
function measured(box) {
  if (!box || box[0] === Infinity) return null;
  const center = { x: (box[0] + box[3]) / 2, y: (box[1] + box[4]) / 2, z: (box[2] + box[5]) / 2 };
  return { xMin: box[0], yMin: box[1], zMin: box[2], xMax: box[3], yMax: box[4], zMax: box[5],
    bounds: { center, radius: Math.hypot(box[3] - box[0], box[4] - box[1], box[5] - box[2]) / 2 } };
}
// Sparse segment tree: page/derived-slot extent changes touch O(log capacity)
// metadata; expansion combines existing metadata and never reads point arrays.
class ExtentTree {
  constructor() { this.capacity = 1; this.nodes = new Map(); }
  set(index, box) {
    while (index >= this.capacity) {
      const next = new Map();
      for (const [at, value] of this.nodes) {
        const depth = Math.floor(Math.log2(at));
        next.set(at + 2 ** depth, value);
      }
      this.capacity *= 2; this.nodes = next;
      if (next.has(2)) next.set(1, next.get(2));
    }
    let at = this.capacity + index;
    if (box) this.nodes.set(at, box); else this.nodes.delete(at);
    while (at > 1) {
      at = Math.floor(at / 2);
      const value = union(this.nodes.get(at * 2), this.nodes.get(at * 2 + 1));
      if (value) this.nodes.set(at, value); else this.nodes.delete(at);
    }
  }
  get(index) { return this.nodes.get(this.capacity + index) || null; }
  get bounds() { return this.nodes.get(1) || null; }
}
// Exact, balanced coordinate tuples ordered by (z,x,y). Range bounds are
// aggregate metadata, so one heavily occupied voxel never causes a rescan of
// all of its raw XYZ when a single source page changes. No application hashes.
class CoordinateTree {
  constructor() { this.root = null; this.lastQueryNodes = 0; }
  _height(node) { return node?.height || 0; }
  _refresh(node) {
    node.height = 1 + Math.max(this._height(node.left), this._height(node.right));
    node.bounds = union(union(node.left?.bounds, [node.x, node.y, node.z, node.x, node.y, node.z]), node.right?.bounds);
    return node;
  }
  _rotateLeft(node) {
    const right = node.right; node.right = right.left; right.left = node;
    this._refresh(node); return this._refresh(right);
  }
  _rotateRight(node) {
    const left = node.left; node.left = left.right; left.right = node;
    this._refresh(node); return this._refresh(left);
  }
  _balance(node) {
    this._refresh(node);
    const balance = this._height(node.left) - this._height(node.right);
    if (balance > 1) {
      if (this._height(node.left.left) < this._height(node.left.right)) node.left = this._rotateLeft(node.left);
      return this._rotateRight(node);
    }
    if (balance < -1) {
      if (this._height(node.right.right) < this._height(node.right.left)) node.right = this._rotateRight(node.right);
      return this._rotateLeft(node);
    }
    return node;
  }
  _adjust(node, value, amount) {
    if (!node) {
      if (amount <= 0) throw new Error('Unbalanced canonical voxel contribution');
      return { x: value.x, y: value.y, z: value.z, count: amount, height: 1, left: null, right: null,
        bounds: [value.x, value.y, value.z, value.x, value.y, value.z] };
    }
    const order = value.z - node.z || value.x - node.x || value.y - node.y;
    if (order < 0) node.left = this._adjust(node.left, value, amount);
    else if (order > 0) node.right = this._adjust(node.right, value, amount);
    else {
      node.count += amount;
      if (node.count < 0) throw new Error('Negative canonical voxel contribution');
      if (!node.count) {
        if (!node.left) return node.right;
        if (!node.right) return node.left;
        let successor = node.right; while (successor.left) successor = successor.left;
        node.x = successor.x; node.y = successor.y; node.z = successor.z; node.count = successor.count;
        node.right = this._adjust(node.right, successor, -successor.count);
      }
    }
    return this._balance(node);
  }
  add(value, amount) { this.root = this._adjust(this.root, value, amount); }
  range(lower, upper, exclusiveLower = false) {
    this.lastQueryNodes = 0;
    const visit = node => {
      if (!node) return null; this.lastQueryNodes++;
      const box = node.bounds;
      if (box[2] > upper || (exclusiveLower ? box[5] <= lower : box[5] < lower)) return null;
      if (box[5] <= upper && (exclusiveLower ? box[2] > lower : box[2] >= lower)) return box;
      const included = node.z <= upper && (exclusiveLower ? node.z > lower : node.z >= lower);
      return union(union(visit(node.left), included ? [node.x, node.y, node.z, node.x, node.y, node.z] : null), visit(node.right));
    };
    return visit(this.root);
  }
}
function floorLevels(histogram, zMin) {
  const scores = new Map(); let maximum = 0;
  for (const [bin, entry] of histogram) {
    const count = offset => histogram.get(bin + offset)?.count || 0;
    const score = entry.count + 0.65 * (count(-1) + count(1)) + 0.25 * (count(-2) + count(2));
    scores.set(bin, score); maximum = Math.max(maximum, score);
  }
  const minimum = Math.max(3, maximum * 0.12);
  const firstIndex = bin => {
    let result = Infinity;
    for (const [page, first] of histogram.get(bin).firstByPage) result = Math.min(result, page * POINT_CLOUD_PAGE_POINTS + first);
    return result;
  };
  const peaks = [...scores].filter(([bin, score]) => score >= minimum
    && score >= (scores.get(bin - 1) || 0) && score >= (scores.get(bin + 1) || 0))
    .map(([bin, score]) => ({ bin, score, first: firstIndex(bin) }))
    .sort((a, b) => b.score - a.score || a.first - b.first);
  const selected = [];
  for (const { bin } of peaks) {
    let weighted = 0, weight = 0;
    for (let offset = -1; offset <= 1; offset++) {
      const count = histogram.get(bin + offset)?.count || 0;
      weighted += (bin + offset) * BIN * count; weight += count;
    }
    const z = weight ? weighted / weight : bin * BIN;
    if (!selected.some(level => Math.abs(level - z) < 1.5)) selected.push(z);
  }
  return selected.length ? selected.sort((a, b) => a - b) : [zMin];
}
function selectFloor(levels, robotZ) {
  const height = Number(robotZ);
  if (!Number.isFinite(height)) return levels[0];
  const below = levels.filter(level => level <= height + 0.45);
  return (below.length ? below : levels).reduce((closest, level) =>
    Math.abs(level - height) < Math.abs(closest - height) ? level : closest);
}
const boundaryFloat = new Float32Array(1);
const boundaryBits = new Uint32Array(boundaryFloat.buffer);
function shaderUpperBoundary(value) {
  boundaryFloat[0] = value;
  // Keep <= comparisons identical for Float32 positions even when an exact
  // histogram-derived threshold would otherwise round upward in a GPU uniform.
  if (boundaryFloat[0] > value) boundaryBits[0] += boundaryFloat[0] > 0 ? -1 : 1;
  return boundaryFloat[0];
}
function rgb(value) { return new THREE.Vector3(...value.map(channel => channel / 255)); }
function makeMaterial(layer, derived) {
  const material = new THREE.PointsMaterial({ size: 1.35, sizeAttenuation: false });
  material.onBeforeCompile = shader => {
    Object.assign(shader.uniforms, layer.uniforms);
    const count = layer.config.heightColorStops.length;
    shader.vertexShader = `
      uniform float pcMode, pcZMin, pcZMax, pcGradientScale, pcBlueBand, pcGroundMax;
      uniform vec3 pcStops[${count}];
      uniform vec3 pcContextColor, pcGroundColor, pcStructureColor, pcObjectColor;
      ${derived ? 'attribute float pcStructure;' : ''}
      varying vec3 pcColor;
      ${shader.vertexShader}`;
    shader.vertexShader = shader.vertexShader.replace('#include <begin_vertex>', `
      #include <begin_vertex>
      pcColor = pcContextColor;
      if (pcMode < 0.5) {
        float pcRange = pcZMax - pcZMin;
        if (pcRange == 0.0) pcRange = 1.0;
        float pcStart = pcBlueBand >= 0.0 ? pcZMin + pcBlueBand
          : pcZMin + pcRange * (0.5 - 0.5 / pcGradientScale);
        float pcT = clamp((position.z - pcStart) * pcGradientScale / pcRange, 0.0, 1.0) * ${count - 1}.0;
        pcColor = pcStops[${count - 1}];
        ${Array.from({ length: count - 1 }, (_, i) => `if (pcT >= ${i}.0 && pcT <= ${i + 1}.0) pcColor = mix(pcStops[${i}], pcStops[${i + 1}], pcT - ${i}.0);`).join('\n')}
        pcColor = floor(pcColor * 255.0 + 0.5) / 255.0;
      } ${derived ? `else if (pcMode > 1.5) {
        if (position.z <= pcGroundMax) pcColor = pcGroundColor;
        else if (pcMode > 2.5) pcColor = pcStructure > 0.5 ? pcStructureColor : pcObjectColor;
      }` : ''}
    `);
    shader.fragmentShader = `varying vec3 pcColor;\n${shader.fragmentShader}`;
    shader.fragmentShader = shader.fragmentShader.replace('#include <color_fragment>', 'diffuseColor.rgb *= pcColor;');
  };
  material.customProgramCacheKey = () => `canonical-page-${derived ? 'derived' : 'source'}-${layer.config.heightColorStops.length}`;
  return material;
}
function bucketAdd(index, bin, key, amount) {
  let bucket = index.get(bin);
  if (!bucket) { bucket = new Map(); index.set(bin, bucket); }
  const count = (bucket.get(key) || 0) + amount;
  if (count) bucket.set(key, count); else bucket.delete(key);
  if (!bucket.size) index.delete(bin);
}
function collectBetween(index, left, right, target) {
  if (left === right) return;
  if (!Number.isFinite(left) || !Number.isFinite(right)) {
    for (const bucket of index.values()) for (const key of bucket.keys()) target.add(key);
    return;
  }
  const lower = Math.floor(Math.min(left, right) / BIN), upper = Math.floor(Math.max(left, right) / BIN);
  for (const [bin, bucket] of index) if (bin >= lower && bin <= upper) for (const key of bucket.keys()) target.add(key);
}
function mergeUpdateRange(attribute, start, count) {
  // Do not clear an earlier, not-yet-rendered transaction's pending range.
  let first = start, last = start + count;
  const untouched = [];
  for (const range of attribute.updateRanges) {
    if (range.start + range.count < first || range.start > last) untouched.push(range);
    else { first = Math.min(first, range.start); last = Math.max(last, range.start + range.count); }
  }
  attribute.clearUpdateRanges();
  for (const range of untouched) attribute.addUpdateRange(range.start, range.count);
  attribute.addUpdateRange(first, last - first); attribute.needsUpdate = true;
}
// GPU counters describe scheduled BufferAttribute byte ranges, not GPU-driver
// timings. Initial capacity uploads are charged separately from steady writes.
// gpuDirtyBytes/gpuUploadCount are source-only; gpuTotal* include derived slabs.
const COUNTERS = ['transactions', 'copiedCoordinates', 'scannedCoordinates', 'pageAllocations', 'pageDisposals',
  'gpuDirtyBytes', 'gpuAllocationBytes', 'derivedCopiedCoordinates', 'derivedScannedCoordinates',
  'derivedPageAllocations', 'derivedPageDisposals', 'derivedGpuDirtyBytes', 'derivedRebuilds', 'rebuiltSourcePoints',
  'sourceSlabAllocations', 'sourceSlabDisposals', 'gpuUploadCount', 'derivedGpuUploadCount',
  'voxelBoundsQueryNodes', 'columnBoundsQueryNodes', 'gpuTotalDirtyBytes', 'gpuTotalUploadCount'];
function counters() { return Object.fromEntries(COUNTERS.map(key => [key, 0])); }

export class IncrementalPointCloudLayer {
  constructor({ config = DEFAULT_CONFIG, onChange = () => {}, style = 'height' } = {}) {
    this.config = config; this.onChange = onChange; this.style = style;
    this.group = new THREE.Group(); this.group.name = 'canonical-paged-pointcloud';
    this.sourceGroup = new THREE.Group(); this.sourceGroup.name = 'canonical-source-pages';
    this.derivedGroup = new THREE.Group(); this.derivedGroup.name = 'canonical-display-voxels';
    this.group.add(this.sourceGroup, this.derivedGroup);
    this.pages = []; this.sourceSlabs = []; this.derivedPages = []; this.pointCount = 0; this.displayedPointCount = 0;
    this.stats = { bootstrap: counters(), steady: counters(), explicit: counters(), lastApply: counters(),
      gpuMeasurement: 'scheduled BufferAttribute bytes/ranges; not measured GPU-driver upload time',
      drawCallMeasurement: 'visible slab count before frustum culling; not measured renderer draw calls' };
    this.uniforms = {
      pcMode: { value: 0 }, pcZMin: { value: 0 }, pcZMax: { value: 1 }, pcGradientScale: { value: 1 },
      pcBlueBand: { value: -1 }, pcGroundMax: { value: 0 }, pcStops: { value: [] },
      pcContextColor: { value: rgb(config.floorColors.context) }, pcGroundColor: { value: rgb(config.floorColors.ground) },
      pcStructureColor: { value: rgb(config.floorColors.structure) }, pcObjectColor: { value: rgb(config.floorColors.object) },
    };
    this.material = makeMaterial(this, false); this.derivedMaterial = makeMaterial(this, true);
    this._sourceExtents = new ExtentTree(); this._derivedExtents = new ExtentTree();
    this._histogram = new Map(); this._voxels = new Map(); this._heightBins = new Map();
    this._columns = new Map(); this._displayHeightBins = new Map(); this._slots = [];
    this._spacing = this._preset().floorFilter ? this._preset().liveSpacingM : null; this._indexRange = null; this._indexGroundMax = null;
    this._revision = 0; this._hasBaseline = false; this._disposed = false; this._visible = true;
    this._robotZ = undefined; this.measurement = this.sourceMeasurement = this.bounds = this.floorContext = null;
    this._pendingDerived = new Map(); this._pendingSource = new Set(); this._sourceUploads = new Map();
    this._derivedGeometryExtents = new ExtentTree(); this._derivedLogicalPages = 0; this._syncDisplay();
  }
  _count(key, amount = 1) {
    if (!this._currentStats) return;
    this._currentStats[key] += amount; this.stats.lastApply[key] += amount;
    const total = key === 'gpuDirtyBytes' || key === 'derivedGpuDirtyBytes' ? 'gpuTotalDirtyBytes'
      : key === 'gpuUploadCount' || key === 'derivedGpuUploadCount' ? 'gpuTotalUploadCount' : null;
    if (total) { this._currentStats[total] += amount; this.stats.lastApply[total] += amount; }
  }
  _beginStats(phase) { this.stats.lastApply = counters(); this._currentStats = this.stats[phase]; }
  _endStats() { this._currentStats = null; }
  _preset() { return this.config.stylePresets[this.style] || this.config.stylePresets.height; }
  _metadata(xyz, count, withVoxels = this._spacing !== null) {
    const bounds = EMPTY(), histogram = new Map(), voxels = withVoxels ? new Map() : null;
    for (let i = 0; i < count; i++) {
      const offset = i * 3, x = xyz[offset], y = xyz[offset + 1], z = xyz[offset + 2];
      if (![x, y, z].every(Number.isFinite)) throw new Error('Canonical pointcloud contains non-finite XYZ');
      extend(bounds, x, y, z);
      const bin = Math.round(z / BIN), entry = histogram.get(bin);
      if (entry) entry.count++; else histogram.set(bin, { count: 1, first: i });
      if (voxels) {
        const inverse = 1 / this._spacing;
        const gx = Math.floor(x * inverse), gy = Math.floor(y * inverse), gz = Math.floor(z * inverse);
        const key = `${gx},${gy},${gz}`;
        let cell = voxels.get(key);
        if (!cell) { cell = { grid: [gx, gy, gz], raw: new Map() }; voxels.set(key, cell); }
        const rawKey = `${x},${y},${z}`, raw = cell.raw.get(rawKey);
        if (raw) raw.count++; else cell.raw.set(rawKey, { x, y, z, count: 1 });
      }
    }
    return { bounds: count ? bounds : null, histogram, voxels };
  }
  prepare(transaction) {
    if (this._disposed) throw new Error('Canonical pointcloud layer is disposed');
    const { header, chunks } = transaction || {};
    const count = header?.point_count, size = header?.chunk_size_points;
    if (!Number.isSafeInteger(count) || count < 0 || size !== POINT_CLOUD_PAGE_POINTS || !Array.isArray(chunks)) {
      throw new Error('Invalid canonical pointcloud page transaction');
    }
    const pageCount = Math.ceil(count / POINT_CLOUD_PAGE_POINTS), dirty = new Map();
    for (const chunk of chunks) {
      const index = chunk.chunk_index;
      if (!Number.isSafeInteger(index) || index < 0 || index >= pageCount || dirty.has(index)
          || !(chunk.xyz instanceof Float32Array)) throw new Error('Invalid or duplicate canonical page');
      const length = Math.min(POINT_CLOUD_PAGE_POINTS, count - index * POINT_CLOUD_PAGE_POINTS);
      if (chunk.xyz.length !== length * 3) throw new Error('Canonical page length does not match point_count');
      dirty.set(index, { index, xyz: chunk.xyz, count: length, copy: true });
    }
    if (header.reset || !this._hasBaseline) {
      if (!header.reset) throw new Error('Canonical pointcloud requires a reset baseline');
      if (dirty.size !== pageCount) throw new Error('Incomplete canonical pointcloud reset');
    } else {
      for (let index = this.pages.length; index < pageCount; index++) if (!dirty.has(index)) throw new Error('Missing appended canonical page');
      if (pageCount) {
        const last = pageCount - 1, expected = count - last * POINT_CLOUD_PAGE_POINTS, old = this.pages[last];
        if (old && old.count !== expected && !dirty.has(last)) {
          if (old.count < expected) throw new Error('Missing grown canonical tail page');
          dirty.set(last, { index: last, xyz: old.array.subarray(0, expected * 3), count: expected, copy: false });
        }
      }
      const oldTail = this.pages.length - 1;
      if (oldTail >= 0 && oldTail < pageCount - 1 && this.pages[oldTail].count < POINT_CLOUD_PAGE_POINTS && !dirty.has(oldTail)) {
        throw new Error('Missing completed canonical tail page');
      }
    }
    // Complete validation precedes all visible state changes, copies and allocations.
    for (const page of dirty.values()) page.metadata = this._metadata(page.xyz, page.count);
    return { transaction, revision: this._revision, dirty, pageCount, count };
  }
  _newPage(index, derived) {
    if (!derived) {
      const slabIndex = Math.floor(index / PAGES_PER_SLAB), offsetPoints = index % PAGES_PER_SLAB * POINT_CLOUD_PAGE_POINTS;
      let slab = this.sourceSlabs[slabIndex];
      if (!slab) { slab = this._newRenderSlab(slabIndex, false); this.sourceSlabs[slabIndex] = slab; }
      this._count('pageAllocations'); this._pendingSource.add(slabIndex);
      return { array: slab.array.subarray(offsetPoints * 3, (offsetPoints + POINT_CLOUD_PAGE_POINTS) * 3),
        geometry: slab.geometry, mesh: slab.mesh, slab, slabIndex, offsetPoints, count: 0, metadata: null };
    }
    return this._newRenderSlab(index, true);
  }
  _newRenderSlab(index, derived) {
    const array = new Float32Array(POINT_CLOUD_RENDER_POINTS * 3), geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(array, 3).setUsage(THREE.DynamicDrawUsage));
    if (derived) geometry.setAttribute('pcStructure', new THREE.BufferAttribute(new Uint8Array(POINT_CLOUD_RENDER_POINTS), 1).setUsage(THREE.DynamicDrawUsage));
    geometry.setDrawRange(0, 0);
    const mesh = new THREE.Points(geometry, derived ? this.derivedMaterial : this.material);
    mesh.name = `${derived ? 'display' : 'source'}-slab-${index}`;
    mesh.userData.canonicalLayer = this; mesh.userData.canonicalDerived = derived; mesh.userData.pageIndex = index;
    (derived ? this.derivedGroup : this.sourceGroup).add(mesh);
    this._count(derived ? 'derivedPageAllocations' : 'sourceSlabAllocations');
    this._count('gpuAllocationBytes', array.byteLength + (derived ? POINT_CLOUD_RENDER_POINTS : 0));
    return { array, geometry, mesh, count: 0, fresh: true };
  }
  _disposePage(page, derived) {
    if (!derived) { this._count('pageDisposals'); this._pendingSource.add(page.slabIndex); return; }
    this._disposeRenderSlab(page, true);
  }
  _disposeRenderSlab(page, derived) {
    (derived ? this.derivedGroup : this.sourceGroup).remove(page.mesh); page.geometry.dispose();
    this._count(derived ? 'derivedPageDisposals' : 'sourceSlabDisposals');
  }
  _flushSource() {
    const slabCount = Math.ceil(this.pointCount / POINT_CLOUD_RENDER_POINTS);
    while (this.sourceSlabs.length > slabCount) this._disposeRenderSlab(this.sourceSlabs.pop(), false);
    for (const index of this._pendingSource) {
      const slab = this.sourceSlabs[index]; if (!slab) continue;
      slab.count = Math.min(POINT_CLOUD_RENDER_POINTS, this.pointCount - index * POINT_CLOUD_RENDER_POINTS);
      slab.geometry.setDrawRange(0, slab.count);
      let bounds = null;
      for (let pageIndex = index * PAGES_PER_SLAB; pageIndex < Math.min(this.pages.length, (index + 1) * PAGES_PER_SLAB); pageIndex++) {
        bounds = union(bounds, this.pages[pageIndex].metadata.bounds);
      }
      this._geometryBounds(slab, bounds);
      const uploads = this._sourceUploads.get(index);
      if (uploads?.size) {
        const ordered = [...uploads].sort((a, b) => a[0] - b[0]);
        const ranges = [];
        for (const [start, count] of ordered) {
          const previous = ranges.at(-1);
          if (previous && previous.start + previous.count === start) previous.count += count;
          else ranges.push({ start, count });
        }
        for (const range of ranges) mergeUpdateRange(slab.geometry.getAttribute('position'), range.start, range.count);
        this._count('gpuDirtyBytes', slab.fresh ? slab.array.byteLength : ranges.reduce((sum, range) => sum + range.count * 4, 0));
        this._count('gpuUploadCount', slab.fresh ? 1 : ranges.length);
      }
      slab.fresh = false;
    }
    this._pendingSource.clear(); this._sourceUploads.clear();
  }
  _geometryBounds(page, box) {
    page.geometry.boundingBox = box ? new THREE.Box3(new THREE.Vector3(...box.slice(0, 3)), new THREE.Vector3(...box.slice(3))) : new THREE.Box3();
    page.geometry.boundingSphere = page.geometry.boundingBox.getBoundingSphere(new THREE.Sphere());
  }
  _pageHistogram(index, metadata, sign) {
    if (!metadata) return;
    for (const [bin, value] of metadata.histogram) {
      let entry = this._histogram.get(bin);
      if (!entry) { entry = { count: 0, firstByPage: new Map() }; this._histogram.set(bin, entry); }
      entry.count += sign * value.count;
      if (sign > 0) entry.firstByPage.set(index, value.first); else entry.firstByPage.delete(index);
      if (!entry.count) this._histogram.delete(bin);
    }
  }
  _pageVoxels(metadata, sign, touched) {
    if (!metadata?.voxels) return;
    for (const [key, contribution] of metadata.voxels) {
      touched.add(key);
      let cell = this._voxels.get(key);
      if (!cell) {
        const center = contribution.grid.map(value => Math.fround((value + 0.5) * this._spacing));
        const column = `${Math.floor(center[0] / COLUMN)},${Math.floor(center[1] / COLUMN)}`;
        cell = { key, center, column, points: new CoordinateTree(), count: 0, slot: null, bounds: null };
        this._voxels.set(key, cell);
      }
      for (const value of contribution.raw.values()) {
        cell.points.add(value, sign * value.count);
        cell.count += sign * value.count;
        bucketAdd(this._heightBins, Math.floor(value.z / BIN), key, sign * value.count);
      }
    }
  }
  apply(transaction, prepared = null) {
    const stage = prepared && prepared.transaction === transaction && prepared.revision === this._revision
      ? prepared : this.prepare(transaction);
    this._beginStats(transaction.header.reset || !this._hasBaseline ? 'bootstrap' : 'steady');
    this._count('transactions');
    const touched = new Set();
    if (transaction.header.reset) this._clearData();
    for (let index = this.pages.length - 1; index >= stage.pageCount; index--) {
      const page = this.pages.pop();
      this._pageHistogram(index, page.metadata, -1); this._pageVoxels(page.metadata, -1, touched);
      this._sourceExtents.set(index, null); this._disposePage(page, false);
    }
    for (const entry of stage.dirty.values()) {
      let page = this.pages[entry.index];
      if (!page) { page = this._newPage(entry.index, false); this.pages[entry.index] = page; }
      this._pageHistogram(entry.index, page.metadata, -1); this._pageVoxels(page.metadata, -1, touched);
      if (entry.copy) {
        page.array.set(entry.xyz); this._count('copiedCoordinates', entry.xyz.length);
        let uploads = this._sourceUploads.get(page.slabIndex);
        if (!uploads) { uploads = new Map(); this._sourceUploads.set(page.slabIndex, uploads); }
        uploads.set(page.offsetPoints * 3, entry.xyz.length);
      }
      this._count('scannedCoordinates', entry.count * 3);
      page.count = entry.count; page.metadata = entry.metadata; this._pendingSource.add(page.slabIndex);
      this._sourceExtents.set(entry.index, entry.metadata.bounds);
      this._pageHistogram(entry.index, page.metadata, 1); this._pageVoxels(page.metadata, 1, touched);
    }
    this.pointCount = stage.count; this.sourceMeasurement = measured(this._sourceExtents.bounds);
    this._hasBaseline = true; this._revision++;
    this._refreshFloor();
    if (this._spacing !== null) this._reconcile(touched);
    this._flushSource(); this._syncDisplay(); this._flushDerived();
    const result = this._summary(); this._endStats(); this.onChange(result); return result;
  }
  _refreshFloor() {
    const preset = this._preset();
    if (!preset.floorFilter || !this.sourceMeasurement) { this.floorContext = null; return; }
    const levels = floorLevels(this._histogram, this.sourceMeasurement.zMin), floorZ = selectFloor(levels, this._robotZ);
    const next = levels[levels.indexOf(floorZ) + 1];
    const fallback = Math.min(this.sourceMeasurement.zMax + BAND, floorZ + 2.8);
    this.floorContext = { floorZ, levels, groundMin: floorZ - BAND, groundMax: floorZ + BAND,
      storyMax: Math.max(floorZ + BAND, Number.isFinite(next) ? next - 0.2 : fallback) };
  }
  _rebuildVoxels(spacing) {
    this._clearDerived(); this._spacing = spacing;
    this._count('derivedRebuilds');
    const touched = new Set();
    for (const page of this.pages) {
      page.metadata.voxels = this._metadata(page.array, page.count, true).voxels;
      this._count('rebuiltSourcePoints', page.count);
      this._pageVoxels(page.metadata, 1, touched);
    }
    this._reconcile(touched);
  }
  _markDerived(pageIndex, kind, point) {
    let dirty = this._pendingDerived.get(pageIndex);
    if (!dirty) { dirty = { position: new Set(), structure: new Set(), boundsPages: new Set() }; this._pendingDerived.set(pageIndex, dirty); }
    if (kind) dirty[kind].add(point);
    if (!kind || kind === 'position') dirty.boundsPages.add(Math.floor(point / POINT_CLOUD_PAGE_POINTS));
  }
  _writeSlot(slot, cell, position = true) {
    const pageIndex = Math.floor(slot / POINT_CLOUD_RENDER_POINTS), offset = slot % POINT_CLOUD_RENDER_POINTS;
    let page = this.derivedPages[pageIndex];
    if (!page) { page = this._newPage(pageIndex, true); this.derivedPages[pageIndex] = page; }
    if (position) {
      page.array.set(cell.center, offset * 3); this._count('derivedCopiedCoordinates', 3);
      this._markDerived(pageIndex, 'position', offset);
    }
    const flag = this._columns.get(cell.column)?.vertical ? 1 : 0;
    const attribute = page.geometry.getAttribute('pcStructure');
    if (position || attribute.array[offset] !== flag) {
      attribute.array[offset] = flag; this._markDerived(pageIndex, 'structure', offset);
    }
  }
  _activate(cell, columns) {
    cell.slot = this._slots.length; this._slots.push(cell); this._writeSlot(cell.slot, cell);
    let column = this._columns.get(cell.column);
    if (!column) { column = { cells: new Set(), points: new CoordinateTree(), vertical: false }; this._columns.set(cell.column, column); }
    column.cells.add(cell.key); column.points.add({ x: cell.center[0], y: cell.center[1], z: cell.center[2] }, 1); columns.add(cell.column);
    bucketAdd(this._displayHeightBins, Math.floor(cell.center[2] / BIN), cell.key, 1);
  }
  _deactivate(cell, columns) {
    const slot = cell.slot, last = this._slots.pop();
    if (last !== cell) {
      this._slots[slot] = last; last.slot = slot; this._writeSlot(slot, last);
      this._derivedExtents.set(slot, last.bounds);
    }
    this._derivedExtents.set(this._slots.length, null); cell.slot = null;
    const column = this._columns.get(cell.column); column.cells.delete(cell.key);
    column.points.add({ x: cell.center[0], y: cell.center[1], z: cell.center[2] }, -1); columns.add(cell.column);
    bucketAdd(this._displayHeightBins, Math.floor(cell.center[2] / BIN), cell.key, -1);
    const tail = Math.floor(this._slots.length / POINT_CLOUD_RENDER_POINTS);
    this._markDerived(tail, null, this._slots.length % POINT_CLOUD_RENDER_POINTS);
  }
  _reconcile(touched) {
    const preset = this._preset(), context = this.floorContext;
    const range = context ? [context.groundMin, preset.floorFilter === 'ground' ? context.groundMax : context.storyMax] : this._indexRange;
    if (!range) { for (const key of touched) if (!this._voxels.get(key)?.count) this._voxels.delete(key); return; }
    const groundMax = context?.groundMax ?? this._indexGroundMax;
    if (!this._indexRange) for (const key of this._voxels.keys()) touched.add(key);
    else {
      collectBetween(this._heightBins, this._indexRange[0], range[0], touched);
      collectBetween(this._heightBins, this._indexRange[1], range[1], touched);
    }
    const columns = new Set(), groundCrossings = new Set();
    if (this._indexGroundMax !== null) collectBetween(this._displayHeightBins, this._indexGroundMax, groundMax, groundCrossings);
    for (const key of groundCrossings) columns.add(this._voxels.get(key).column);
    for (const key of touched) {
      const cell = this._voxels.get(key); if (!cell) continue;
      const box = cell.points.range(range[0], range[1]), eligible = box !== null;
      this._count('voxelBoundsQueryNodes', cell.points.lastQueryNodes);
      cell.bounds = box;
      if (eligible) {
        if (cell.slot === null) this._activate(cell, columns);
        this._derivedExtents.set(cell.slot, box);
      } else if (cell.slot !== null) this._deactivate(cell, columns);
      if (!cell.count) this._voxels.delete(key);
    }
    this._indexRange = range; this._indexGroundMax = groundMax;
    for (const key of columns) {
      const column = this._columns.get(key); if (!column) continue;
      if (!column.cells.size) { this._columns.delete(key); continue; }
      const span = column.points.range(groundMax, Infinity, true);
      this._count('columnBoundsQueryNodes', column.points.lastQueryNodes);
      const vertical = !!span && span[5] - span[2] >= VERTICAL_SPAN;
      if (vertical !== column.vertical) {
        column.vertical = vertical;
        for (const cellKey of column.cells) this._writeSlot(this._voxels.get(cellKey).slot, this._voxels.get(cellKey), false);
      }
    }
  }
  _flushDerived() {
    const pageCount = Math.ceil(this._slots.length / POINT_CLOUD_RENDER_POINTS);
    const logicalPages = Math.ceil(this._slots.length / POINT_CLOUD_PAGE_POINTS);
    for (let index = logicalPages; index < this._derivedLogicalPages; index++) this._derivedGeometryExtents.set(index, null);
    this._derivedLogicalPages = logicalPages;
    while (this.derivedPages.length > pageCount) this._disposePage(this.derivedPages.pop(), true);
    for (const [index, dirty] of this._pendingDerived) {
      const page = this.derivedPages[index]; if (!page) continue;
      page.count = Math.min(POINT_CLOUD_RENDER_POINTS, this._slots.length - index * POINT_CLOUD_RENDER_POINTS);
      page.geometry.setDrawRange(0, page.count);
      for (const [kind, points] of [['position', dirty.position], ['pcStructure', dirty.structure]]) {
        const attribute = page.geometry.getAttribute(kind), width = kind === 'position' ? 3 : 1;
        const ordered = [...points].filter(point => point < page.count).sort((a, b) => a - b);
        let start = null, end = null, bytes = 0, uploads = 0;
        const flush = () => { if (start === null) return; mergeUpdateRange(attribute, start * width, (end - start + 1) * width); bytes += (end - start + 1) * width * attribute.array.BYTES_PER_ELEMENT; uploads++; };
        for (const point of ordered) {
          if (start === null) { start = end = point; } else if (point === end + 1) end = point;
          else { flush(); start = end = point; }
        }
        flush();
        this._count('derivedGpuDirtyBytes', page.fresh ? attribute.array.byteLength : bytes);
        this._count('derivedGpuUploadCount', page.fresh ? 1 : uploads);
      }
      if (dirty.boundsPages.size || page.fresh) {
        for (const logical of dirty.boundsPages) {
          const start = logical * POINT_CLOUD_PAGE_POINTS, count = Math.max(0, Math.min(POINT_CLOUD_PAGE_POINTS, page.count - start));
          const box = EMPTY();
          for (let i = start * 3; i < (start + count) * 3; i += 3) extend(box, page.array[i], page.array[i + 1], page.array[i + 2]);
          this._count('derivedScannedCoordinates', count * 3);
          this._derivedGeometryExtents.set(index * PAGES_PER_SLAB + logical, count ? box : null);
        }
        let bounds = null;
        for (let logical = 0; logical < PAGES_PER_SLAB; logical++) bounds = union(bounds, this._derivedGeometryExtents.get(index * PAGES_PER_SLAB + logical));
        this._geometryBounds(page, bounds);
      }
      page.fresh = false;
    }
    this._pendingDerived.clear();
  }
  _syncDisplay() {
    const preset = this._preset(), derived = !!preset.floorFilter;
    this.sourceGroup.visible = !derived && !preset.denseSource;
    this.derivedGroup.visible = derived && !preset.denseSource;
    this.group.visible = this._visible && !preset.denseSource;
    this.measurement = derived ? measured(this._derivedExtents.bounds) : this.sourceMeasurement;
    this.displayedPointCount = preset.denseSource ? 0 : derived ? this._slots.length : this.pointCount;
    const b = this.measurement?.bounds;
    this.bounds = b ? new THREE.Sphere(new THREE.Vector3(b.center.x, b.center.y, b.center.z), b.radius) : null;
    this.uniforms.pcMode.value = preset.colorMode === 'height' ? 0 : preset.colorMode === 'mono' ? 1 : preset.colorMode === 'structure' ? 3 : 2;
    this.uniforms.pcZMin.value = this.measurement?.zMin ?? 0; this.uniforms.pcZMax.value = this.measurement?.zMax ?? 1;
    this.uniforms.pcGradientScale.value = Number.isFinite(preset.heightGradientScale) && preset.heightGradientScale > 0 ? preset.heightGradientScale : 1;
    this.uniforms.pcBlueBand.value = Number.isFinite(preset.blueBandHeightM) && preset.blueBandHeightM >= 0 ? preset.blueBandHeightM : -1;
    this.uniforms.pcGroundMax.value = shaderUpperBoundary(this.floorContext?.groundMax ?? 0);
    this.uniforms.pcStops.value = this.config.heightColorStops.map(rgb);
    for (const [name, key] of [['pcContextColor', 'context'], ['pcGroundColor', 'ground'], ['pcStructureColor', 'structure'], ['pcObjectColor', 'object']]) {
      this.uniforms[name].value.copy(rgb(this.config.floorColors[key]));
    }
    for (const material of [this.material, this.derivedMaterial]) {
      material.size = preset.size ?? 1.35;
      if (material.sizeAttenuation !== !!preset.sizeAttenuation) { material.sizeAttenuation = !!preset.sizeAttenuation; material.needsUpdate = true; }
    }
  }
  configure(config) {
    const oldStops = this.config.heightColorStops.length;
    this.config = config; this._revision++; this._beginStats('explicit');
    this._refreshFloor();
    const spacing = this._preset().floorFilter ? this._preset().liveSpacingM : null;
    if (spacing != null && (!(spacing > 0) || !Number.isFinite(spacing))) throw new Error('Floor display requires positive voxel spacing');
    if (spacing != null && spacing !== this._spacing) this._rebuildVoxels(spacing);
    else if (this._spacing !== null) this._reconcile(new Set());
    if (oldStops !== config.heightColorStops.length) { this.material.needsUpdate = true; this.derivedMaterial.needsUpdate = true; }
    this._syncDisplay(); this._flushDerived(); this._endStats(); this.onChange(this._summary());
  }
  setStyle(style) {
    if (!Object.hasOwn(this.config.stylePresets, style)) return false;
    if (this.style === style) return true;
    this.style = style; this.configure(this.config); return true;
  }
  setRobotPose(pose) {
    const z = typeof pose === 'number' ? pose : pose?.z;
    if (Object.is(this._robotZ, z)) return;
    this._robotZ = z; this._revision++; this._beginStats('explicit'); this._refreshFloor();
    if (this._spacing !== null) this._reconcile(new Set());
    this._syncDisplay(); this._flushDerived(); this._endStats(); this.onChange(this._summary());
  }
  setVisible(visible) { this._visible = !!visible; this.group.visible = this._visible && !this._preset().denseSource; this._summary(); }
  filterIntersection(hit) {
    if (this._disposed || !hit?.object || !this.group.visible || hit.object.userData.canonicalLayer !== this) return false;
    const derived = hit.object.userData.canonicalDerived;
    return (derived ? this.derivedGroup.visible : this.sourceGroup.visible)
      && Number.isInteger(hit.index) && hit.index >= 0 && hit.index < hit.object.geometry.drawRange.count;
  }
  _summary() {
    this.stats.gauges = { sourceDrawCalls: this.sourceGroup.visible && this.group.visible ? this.sourceSlabs.length : 0,
      derivedDrawCalls: this.derivedGroup.visible && this.group.visible ? this.derivedPages.length : 0,
      allocatedSourcePoints: this.sourceSlabs.length * POINT_CLOUD_RENDER_POINTS,
      allocatedDerivedPoints: this.derivedPages.length * POINT_CLOUD_RENDER_POINTS,
      sourcePages: this.pages.length, sourceSlabs: this.sourceSlabs.length, derivedSlabs: this.derivedPages.length };
    return { count: this.displayedPointCount, displayedPointCount: this.displayedPointCount, pointCount: this.pointCount,
      measurement: this.measurement, bounds: this.bounds, floorContext: this.floorContext, stats: this.stats };
  }
  _clearDerived() {
    for (const page of this.derivedPages) this._disposePage(page, true);
    this.derivedPages = []; this._pendingDerived.clear(); this._voxels.clear(); this._heightBins.clear();
    this._columns.clear(); this._displayHeightBins.clear(); this._slots = []; this._derivedExtents = new ExtentTree();
    this._indexRange = this._indexGroundMax = null;
    this._derivedGeometryExtents = new ExtentTree(); this._derivedLogicalPages = 0;
  }
  _clearData() {
    for (const page of this.pages) this._disposePage(page, false);
    for (const slab of this.sourceSlabs) this._disposeRenderSlab(slab, false);
    this.pages = []; this.sourceSlabs = []; this._pendingSource.clear(); this._sourceUploads.clear(); this._sourceExtents = new ExtentTree(); this._histogram.clear(); this._clearDerived();
    this.pointCount = this.displayedPointCount = 0;
    this.sourceMeasurement = this.measurement = this.bounds = this.floorContext = null;
  }
  clear() {
    this._beginStats('explicit'); this._clearData(); this._hasBaseline = false; this._revision++;
    this._syncDisplay(); this._endStats(); this.onChange(this._summary());
  }
  dispose() {
    if (this._disposed) return;
    this._beginStats('explicit'); this._clearData(); this.material.dispose(); this.derivedMaterial.dispose(); this.group.removeFromParent();
    this.group.clear(); this._disposed = true; this._endStats(); this._summary();
  }
}
