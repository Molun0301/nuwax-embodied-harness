#!/usr/bin/env node
// 离线检查真实体素身份、原抽样分支与临时分配；不执行旧哈希算法。
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { performance } = require('node:perf_hooks');
const { spawnSync } = require('node:child_process');

const root = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(root, 'gateways/http/static/js/pointcloud-processing.js'), 'utf8');
let forbiddenCalls = 0;
let PointCloudProcessor;
const MAX_POINTS = Math.floor(32 * 1024 * 1024 / 12);

function processor(extra = {}) {
  const p = new PointCloudProcessor({
    stylePresets: {
      height: { sampleStride: 1, colorMode: 'mono', liveSpacingM: 1 },
      raw: { preserveSource: true, colorMode: 'mono', liveSpacingM: 1 },
      direct: { sampleStride: 1, colorMode: 'mono' },
      ground: { sampleStride: 1, colorMode: 'mono', floorFilter: 'ground', liveSpacingM: 1 },
      dense: { denseSource: true },
    },
    floorColors: { context: [10, 20, 30], ground: [40, 50, 60] },
  });
  Object.assign(p, { _allPositions: new Float32Array(), _pointStyle: 'height', _lod: 'full',
    _isLiveCloudMapSnapshot: true }, extra);
  return p;
}

// 小夹具采用逐项坐标比较的独立真值，不共享生产 key，也不复现旧碰撞行为。
function oracle(values, spacing) {
  const cells = [], output = [];
  for (let i = 0; i < values.length; i += 3) {
    const point = [values[i], values[i + 1], values[i + 2]];
    if (!point.every(Number.isFinite)) continue;
    const cell = point.map(value => Math.floor(value * (1 / spacing)));
    if (cells.some(previous => previous.every((value, axis) => value === cell[axis]))) continue;
    cells.push(cell);
    output.push(...cell.map(value => (value + 0.5) * spacing));
  }
  return new Float32Array(output);
}

let checks = 0;
function check(name, run) { run(); checks++; console.log('PASS ' + name); }
function equal(actual, expected) { assert.deepEqual(Array.from(actual), Array.from(expected)); }

function runChecks() {
  check('生产体素路径没有显式乘法哈希或32位截断', () => {
    const start = source.indexOf('  _spatiallySamplePointCloud(');
    const end = source.indexOf('\n  _preparePointCloud(', start);
    const sampler = source.slice(start, end);
    assert.ok(start >= 0 && end > start);
    assert.doesNotMatch(sampler, /Math\.imul|>>>|73856093|19349663|83492791/);
    assert.match(sampler, /`\$\{gridX\},\$\{gridY\},\$\{gridZ\}`/);
  });
  check('同体素合并、不同体素保留并输出中心', () => {
    const values = new Float32Array([0.1, 0.2, 0.3, 0.9, 0.8, 0.7, 1.1, 0.2, 0.3]);
    equal(processor()._spatiallySamplePointCloud(values, 1), [0.5, 0.5, 0.5, 1.5, 0.5, 0.5]);
  });
  check('首次遇到体素的顺序保持', () => {
    const values = new Float32Array([2.2, 0, 0, -1.2, 0, 0, 2.8, 0, 0, 0, 0, 0]);
    equal(processor()._spatiallySamplePointCloud(values, 1), [2.5, 0.5, 0.5, -1.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
  });
  check('负坐标、整格边界和负零沿用floor语义', () => {
    const values = new Float32Array([-1, -0, 0, -0.01, 0, 0, 0, 0, 0, 1, 0, 0]);
    equal(processor()._spatiallySamplePointCloud(values, 1), oracle(values, 1));
  });
  check('三个轴均保留独立身份，坐标串不混淆分隔位置', () => {
    const values = new Float32Array([1, 23, 4, 12, 3, 4, 1, 23, 5, 1, 24, 4, 2, 23, 4]);
    assert.equal(processor()._spatiallySamplePointCloud(values, 1).length, values.length);
  });
  check('相差2^32的真体素不再因32位截断丢失', () => {
    // 相同低32位由算术性质确定，测试不计算旧哈希。
    const values = new Float32Array([0, 0, 0, 2 ** 32, 0, 0, 0, 2 ** 32, 0, 0, 0, 2 ** 32]);
    const output = processor()._spatiallySamplePointCloud(values, 1);
    assert.equal(output.length, 12);
    equal(output, oracle(values, 1));
  });
  check('非有限XYZ和不完整末点不进入输出', () => {
    const values = new Float32Array([NaN, 0, 0, 0, Infinity, 0, 0, 0, -Infinity, 1, 2, 3, 4]);
    equal(processor()._spatiallySamplePointCloud(values, 1), [1.5, 2.5, 3.5]);
  });
  check('空输入及非法间隔保持原对象直通', () => {
    const p = processor(), empty = new Float32Array(), values = new Float32Array([1, 2, 3]);
    assert.equal(p._spatiallySamplePointCloud(empty, 1), empty);
    for (const spacing of [0, -1, NaN, Infinity, undefined, 'invalid']) {
      assert.equal(p._spatiallySamplePointCloud(values, spacing), values);
    }
  });
  check('可转换间隔和非整数间隔保持数值语义', () => {
    const p = processor(), values = new Float32Array([-0.5, 0.2, 0.8, 0.6, 1.1, 2.8]);
    equal(p._spatiallySamplePointCloud(values, '0.25'), oracle(values, 0.25));
  });
  check('多间隔真实坐标真值和重复调用确定性', () => {
    const values = new Float32Array(Array.from({ length: 600 }, (_, i) => ((i * 29) % 101 - 50) / 16));
    const copy = values.slice(), p = processor();
    for (const spacing of [0.0625, 0.25, 1, 2.5]) {
      equal(p._spatiallySamplePointCloud(values, spacing), oracle(values, spacing));
      equal(p._spatiallySamplePointCloud(values, spacing), oracle(values, spacing));
    }
    equal(values, copy);
  });
  check('极小间隔保留已有IEEE数值边界，不新设拒绝阈值', () => {
    const output = processor()._spatiallySamplePointCloud(new Float32Array([1, 1, 1]), Number.MIN_VALUE);
    assert.equal(output.length, 3);
    assert.ok(Array.from(output).every(value => value === Infinity));
  });
  check('原始来源分支保留所有点和坐标，绕过体素抽样', () => {
    const values = new Float32Array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3]);
    const p = processor({ _allPositions: values, _pointStyle: 'raw' });
    p._spatiallySamplePointCloud = () => { throw new Error('不应调用'); };
    const result = p._preparePointCloud();
    assert.equal(result.positions, values);
    assert.equal(result.colors.length, values.length);
  });
  check('实时分支不受LOD步长影响并保留中心、颜色与测量', () => {
    const values = new Float32Array([0.1, 0.2, 0.3, 0.2, 0.3, 0.4, 1.1, 0.2, 0.3]);
    const p = processor({ _allPositions: values, _lod: 'preview' });
    const result = p._preparePointCloud();
    equal(result.positions, oracle(values, 1));
    equal(result.colors, [10, 20, 30, 10, 20, 30]);
    assert.equal(result.measurement.xMin, values[0]);
  });
  check('无实时体素间隔的分支继续直接显示来源', () => {
    const values = new Float32Array([0, 0, 0, 0, 0, 0]);
    const result = processor({ _allPositions: values, _pointStyle: 'direct' })._preparePointCloud();
    assert.equal(result.positions, values);
  });
  check('实际楼层过滤仍先于实时体素抽样', () => {
    const raw = [];
    for (let i = 0; i < 12; i++) raw.push(i, 0, 0);
    raw.push(0, 0, 1, 0, 0, 5);
    const p = processor({ _allPositions: new Float32Array(raw), _pointStyle: 'ground', _robotPose: { z: 0 } });
    const result = p._preparePointCloud();
    assert.equal(result.positions.length, 36);
    assert.equal(result.floorContext.floorZ, 0);
    for (let i = 2; i < result.positions.length; i += 3) assert.equal(result.positions[i], 0.5);
  });
  check('离线小输入不应用预览步长', () => {
    const values = makeCloud(4096, 'sparse');
    const result = processor({ _allPositions: values, _isLiveCloudMapSnapshot: false, _lod: 'preview' })._preparePointCloud();
    equal(result.positions, values);
  });
  check('离线大输入保留预览步长与顺序', () => {
    const values = makeCloud(5000, 'sparse');
    const result = processor({ _allPositions: values, _isLiveCloudMapSnapshot: false, _lod: 'preview' })._preparePointCloud();
    assert.equal(result.positions.length, 625 * 3);
    for (let i = 0; i < 625; i++) equal(result.positions.subarray(i * 3, i * 3 + 3), values.subarray(i * 24, i * 24 + 3));
  });
  check('离线32MiB点预算与实时原有无该上限分支保持', () => {
    const values = makeCloud(MAX_POINTS + 17, 'sparse');
    const p = processor({ _allPositions: values, _isLiveCloudMapSnapshot: false });
    const result = p._preparePointCloud();
    assert.equal(result.positions.length, MAX_POINTS * 3);
    equal(result.positions.subarray(-3), values.subarray((MAX_POINTS - 1) * 3, MAX_POINTS * 3));
    p._isLiveCloudMapSnapshot = true;
    let sampledInput;
    p._spatiallySamplePointCloud = input => { sampledInput = input; return input; };
    assert.equal(p._preparePointCloud().positions.length, values.length);
    assert.equal(sampledInput, values);
  });
  check('完整扫描专用样式继续拒绝预览来源', () => {
    assert.throws(() => processor({ _pointStyle: 'dense' })._preparePointCloud(), /完整点云/);
  });
  check('体素临时集合不会挂到processor形成跨调用缓存', () => {
    const p = processor(), keys = Object.keys(p);
    for (const count of [10, 10000, 100, 10000, 1]) p._spatiallySamplePointCloud(makeCloud(count, 'sparse'), 0.25);
    assert.deepEqual(Object.keys(p), keys);
    assert.equal(forbiddenCalls, 0);
  });

  {
    check('已知安全边界实际使用数字key并与独立真值一致', () => {
      const NativeSet = global.Set, kinds = [];
      global.Set = class extends NativeSet { add(value) { kinds.push(typeof value); return super.add(value); } };
      try {
        const p = processor(), values = new Float32Array([-2, 3, -1, 0, 3, 2, -1, 5, 0, 0, 3, 2]);
        equal(p._spatiallySamplePointCloud(values, 0.25, p._measurePointCloud(values)), oracle(values, 0.25));
        assert.ok(kinds.length && kinds.every(value => value === 'number'));
      } finally { global.Set = NativeSet; }
    });
    check('未知或超安全范围边界回退精确坐标串', () => {
      const NativeSet = global.Set, kinds = [];
      global.Set = class extends NativeSet { add(value) { kinds.push(typeof value); return super.add(value); } };
      try {
        const p = processor(), values = new Float32Array([0, 0, 0, 2 ** 32, 2 ** 32, 1]);
        for (const measurement of [null, p._measurePointCloud(values), { xMin: NaN, xMax: 0 }]) {
          equal(p._spatiallySamplePointCloud(values, 1, measurement), oracle(values, 1));
        }
        assert.ok(kinds.length && kinds.every(value => value === 'string'));
      } finally { global.Set = NativeSet; }
    });
    check('有限长方体数字索引可逐轴逆解且覆盖每个体素一次', () => {
      const NativeSet = global.Set, keys = [], raw = [];
      for (let x = -3; x <= 3; x++) for (let y = -2; y <= 2; y++) for (let z = -1; z <= 1; z++) raw.push(x, y, z);
      global.Set = class extends NativeSet { add(value) { keys.push(value); return super.add(value); } };
      try {
        const p = processor(), values = new Float32Array(raw);
        equal(p._spatiallySamplePointCloud(values, 1, p._measurePointCloud(values)), oracle(values, 1));
      } finally { global.Set = NativeSet; }
      assert.equal(new NativeSet(keys).size, 105);
      for (let i = 0; i < keys.length; i++) {
        const key = keys[i];
        equal([Math.floor(key / 15) - 3, Math.floor((key % 15) / 3) - 2, key % 3 - 1], raw.slice(i * 3, i * 3 + 3));
      }
    });
    check('边界之外的点回退字符串不与内部数字索引别名', () => {
      const p = processor(), values = new Float32Array([0, 1, 0, 1, 0, 0, 0, 0, 0]);
      const measurement = { xMin: 0, xMax: 1, yMin: 0, yMax: 0, zMin: 0, zMax: 0 };
      equal(p._spatiallySamplePointCloud(values, 1, measurement), oracle(values, 1));
    });
    check('复用边界的多维多间隔输入与真值一致', () => {
      const p = processor(), values = new Float32Array(Array.from({ length: 900 }, (_, i) => ((i * 31) % 127 - 60) / 16));
      for (const spacing of [0.0625, 0.2, 0.25, 1, 2.5]) {
        equal(p._spatiallySamplePointCloud(values, spacing, p._measurePointCloud(values)), oracle(values, spacing));
      }
    });
    check('大绝对坐标先作相对差避免中间和越过安全整数', () => {
      const p = processor(), z = 2 ** 53 - 2 ** 29;
      const values = new Float32Array([0, 0, z, 2 ** 29, 0, z, 2 ** 29, 1, z]);
      equal(p._spatiallySamplePointCloud(values, 1, p._measurePointCloud(values)), oracle(values, 1));
    });
    check('正负极大坐标及体积边界保持无损fallback', () => {
      const p = processor();
      for (const raw of [[0, 0, 0, 2 ** 32, 0, 0], [-(2 ** 24), -(2 ** 24), 0, 2 ** 24, 2 ** 24, 1], [2 ** 54, 0, 0, -(2 ** 54), 0, 0]]) {
        const values = new Float32Array(raw);
        equal(p._spatiallySamplePointCloud(values, 1, p._measurePointCloud(values)), oracle(values, 1));
      }
    });
  }
  console.log(JSON.stringify({ checks, status: 'passed', forbidden_hash_calls: forbiddenCalls }));
}

function makeCloud(count, shape) {
  const values = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    const j = shape === 'duplicates' ? i % 1000 : i;
    values[i * 3] = (j % 1000) * 0.25 + 0.03125;
    values[i * 3 + 1] = (Math.floor(j / 1000) % 1000) * 0.25 + 0.03125;
    values[i * 3 + 2] = Math.floor(j / 1000000) * 0.25 + 0.03125;
  }
  return values;
}

function benchmarkCase(count, shape) {
  assert.equal(typeof global.gc, 'function', '测量需使用 --expose-gc');
  const p = processor(), input = makeCloud(count, shape), runs = [];
  const measurement = p._measurePointCloud(input);
  let warmInput = makeCloud(1000, 'sparse');
  let warmOutput = p._spatiallySamplePointCloud(warmInput, 0.25);
  warmInput = warmOutput = null;
  const unique = shape === 'duplicates' ? Math.min(count, 1000) : count;
  for (let i = 0; i < 3; i++) {
    global.gc(); global.gc();
    const before = process.memoryUsage(), started = performance.now();
    let output = p._spatiallySamplePointCloud(input, 0.25, measurement);
    const elapsed = performance.now() - started, after = process.memoryUsage();
    assert.equal(output.length, unique * 3);
    global.gc(); global.gc();
    const outputRetained = process.memoryUsage();
    assert.equal(output.byteLength, unique * 12);
    output = null;
    global.gc(); global.gc();
    const released = process.memoryUsage();
    runs.push({ elapsed_ms: elapsed, heap_growth_before_forced_gc: after.heapUsed - before.heapUsed,
      array_buffers_growth_before_forced_gc: after.arrayBuffers - before.arrayBuffers,
      heap_growth_after_gc_output_live: outputRetained.heapUsed - before.heapUsed,
      array_buffers_growth_after_gc_output_live: outputRetained.arrayBuffers - before.arrayBuffers,
      heap_growth_after_output_release: released.heapUsed - before.heapUsed,
      rss_before: before.rss, rss_after: after.rss });
  }
  console.log(JSON.stringify({ count, shape, unique_voxels: unique, input_bytes: input.byteLength,
    output_bytes: unique * 12, transient_sample_buffer_bytes: input.byteLength,
    median_ms: [...runs].sort((a, b) => a.elapsed_ms - b.elapsed_ms)[1].elapsed_ms,
    process_max_rss_kib: process.resourceUsage().maxRSS, runs, forbidden_hash_calls: forbiddenCalls }));
}

function multiSizeCase() {
  assert.equal(typeof global.gc, 'function');
  const p = processor(), keys = Object.keys(p), runs = [];
  global.gc(); global.gc();
  const initial = process.memoryUsage();
  for (const count of [1000, 100000, 1000000, 100000, 1000]) {
    let input = makeCloud(count, 'sparse');
    const measurement = p._measurePointCloud(input);
    const start = performance.now();
    let output = p._spatiallySamplePointCloud(input, 0.25, measurement);
    const elapsed = performance.now() - start;
    assert.equal(output.length, count * 3);
    input = output = null;
    global.gc(); global.gc();
    const memory = process.memoryUsage();
    runs.push({ count, elapsed_ms: elapsed, heap_growth_after_release: memory.heapUsed - initial.heapUsed,
      array_buffers_growth_after_release: memory.arrayBuffers - initial.arrayBuffers, rss: memory.rss });
  }
  assert.deepEqual(Object.keys(p), keys);
  console.log(JSON.stringify({ shape: 'consecutive_sparse_sizes', runs, process_max_rss_kib: process.resourceUsage().maxRSS,
    forbidden_hash_calls: forbiddenCalls }));
}

async function main() {
  // 直接加载实际ES模块，避免跨VM上下文或替身Math拖慢性能测量。
  ({ PointCloudProcessor } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64')));
  const originalImul = Math.imul;
  Math.imul = () => { forbiddenCalls++; throw new Error('体素路径禁止乘法哈希'); };
  try {
    if (process.argv[2] === '--benchmark-case') {
      benchmarkCase(Number(process.argv[3]), process.argv[4]);
    } else if (process.argv[2] === '--multi-size') {
      multiSizeCase();
    } else if (process.argv[2] === '--benchmark') {
      const results = [];
      for (const count of [1000, 100000, 1000000]) {
        for (const shape of ['sparse', 'duplicates']) {
          const child = spawnSync(process.execPath, ['--expose-gc', __filename, '--benchmark-case', String(count), shape], { encoding: 'utf8' });
          assert.equal(child.status, 0, child.stderr || child.stdout);
          results.push(JSON.parse(child.stdout.trim()));
        }
      }
      const child = spawnSync(process.execPath, ['--expose-gc', __filename, '--multi-size'], { encoding: 'utf8' });
      assert.equal(child.status, 0, child.stderr || child.stdout);
      results.push(JSON.parse(child.stdout.trim()));
      console.log(JSON.stringify({ node: process.version, platform: process.platform, arch: process.arch,
        implementation: 'production exact row-major voxel index with exact-string fallback', measurement: '复用已经计算的measurement，测量实际无哈希生产实现；没有运行旧哈希。GC后heap不是瞬间峰值。', results }, null, 2));
    } else {
      runChecks();
    }
  } finally {
    Math.imul = originalImul;
  }
}

main().catch(error => { console.error(error); process.exitCode = 1; });
