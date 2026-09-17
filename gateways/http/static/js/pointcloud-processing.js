// 点云预处理的唯一算法实现；不读取DOM（文档对象模型），可在网页工作线程运行。
const LOD_STRIDE = { preview: 8, medium: 4, full: 1 };
const DEFAULT_POINT_STYLE = 'height';
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

const clamp = (value, minimum, maximum) => Math.min(maximum, Math.max(minimum, value));

export class PointCloudProcessor {
  constructor(config) {
    this._processingConfig = config;
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
    const center = { x: (minX + maxX) / 2, y: (minY + maxY) / 2, z: (minZ + maxZ) / 2 };
    const radius = Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2;
    return {
      xMin: minX,
      xMax: maxX,
      yMin: minY,
      yMax: maxY,
      zMin: minZ,
      zMax: maxZ,
      bounds: { center, radius },
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
      const t = clamp(
        (positions[i * 3 + 2] - gradientStart) * scale / range,
        0,
        1,
      );
      const scaled = t * (this._processingConfig.heightColorStops.length - 1);
      const lower = Math.min(Math.floor(scaled), this._processingConfig.heightColorStops.length - 2);
      const blend = scaled - lower;
      const from = this._processingConfig.heightColorStops[lower];
      const to = this._processingConfig.heightColorStops[lower + 1];
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

    // 无分类XYZ点云使用平滑高度直方图；仅改变显示，不写入权威地图。
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
    const preset = this._processingConfig.stylePresets[this._pointStyle] || this._processingConfig.stylePresets[DEFAULT_POINT_STYLE];
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
      let color = this._processingConfig.floorColors.context;
      if (preset.colorMode === 'mono') {
        color = this._processingConfig.floorColors.context;
      } else if (floorContext && positions[offset + 2] <= floorContext.groundMax) {
        color = this._processingConfig.floorColors.ground;
      } else if (preset.colorMode === 'structure' && floorContext) {
        const column = this._pointColumnKey(positions[offset], positions[offset + 1]);
        color = verticalColumns.has(column) ? this._processingConfig.floorColors.structure : this._processingConfig.floorColors.object;
      }
      colors[offset] = color[0];
      colors[offset + 1] = color[1];
      colors[offset + 2] = color[2];
    }
    return colors;
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

  _preparePointCloud() {
    const preset = this._processingConfig.stylePresets[this._pointStyle] || this._processingConfig.stylePresets[DEFAULT_POINT_STYLE];
    const src = this._allPositions;
    if (this._floorLevelSource && this._floorLevelSource !== src) {
      this._floorLevelSource = null;
      this._floorLevelCache = null;
    }
    if (!src.length) {
      this._activeFloorZ = null;
      return this._preparedResult(new Float32Array(0), null, null);
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
    // 完整显示保留收到的每个点及原坐标，不应用显示间隔或细节级别抽样。
    if (preset.preserveSource) {
      return this._preparedResult(visibleSource, measurement, floorContext);
    }
    if (this._isLiveCloudMapSnapshot) {
      const positions = preset.liveSpacingM
        ? this._spatiallySamplePointCloud(visibleSource, preset.liveSpacingM)
        : visibleSource;
      return this._preparedResult(positions, measurement, floorContext);
    }
    const totalPts = Math.floor(visibleSource.length / 3);
    const configuredStride = Math.max(1, Math.round((LOD_STRIDE[this._lod] || 1) * preset.sampleStride));
    const stride = totalPts <= POINT_SAMPLING_MIN_SOURCE_POINTS ? 1 : configuredStride;
    const sampledCount = Math.min(Math.ceil(totalPts / stride), MAX_POINTS);
    if (sampledCount === 0) {
      return this._preparedResult(new Float32Array(0), null, null);
    }

    const pos = new Float32Array(sampledCount * 3);
    for (let i = 0, j = 0; i < sampledCount; i++, j += stride) {
      pos[i * 3] = visibleSource[j * 3];
      pos[i * 3 + 1] = visibleSource[j * 3 + 1];
      pos[i * 3 + 2] = visibleSource[j * 3 + 2];
    }
    return this._preparedResult(pos, measurement, floorContext);
  }

  _preparedResult(positions, measurement, floorContext) {
    return {
      positions,
      measurement,
      floorContext,
      colors: positions.length ? this._pointCloudColors(positions, measurement, floorContext) : new Uint8Array(0),
    };
  }
}
