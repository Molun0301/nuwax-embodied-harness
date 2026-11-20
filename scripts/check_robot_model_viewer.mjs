/* 用生产 three.js 与模型模块验证完整网格、关节更新和异步失效。无需浏览器或机器人。 */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const repo = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-robot-viewer-'));
fs.writeFileSync(path.join(temporary, 'package.json'), '{"type":"module"}');
for (const name of ['robot-model3d.js', 'three.module.min.js']) {
  fs.copyFileSync(path.join(repo, 'gateways/http/static/js', name), path.join(temporary, name));
}
const layers = [];
try {
  const { RobotModelLayer, decodeRobotGeometry } = await import(pathToFileURL(path.join(temporary, 'robot-model3d.js')));
  const { Matrix4, Vector3 } = await import(pathToFileURL(path.join(temporary, 'three.module.min.js')));
  const identity = new Matrix4().toArray();
  const reference = { torso: identity, hand: new Matrix4().makeTranslation(0.4, 0, 0).toArray() };
  const buffer = new ArrayBuffer(28);
  new Uint16Array(buffer, 0, 9).set([0, 0, 0, 65535, 0, 0, 0, 65535, 0]);
  new Uint16Array(buffer, 20, 3).set([0, 1, 2]);
  const descriptor = { id: 'triangle', vertex_count: 3, index_count: 3, index_type: 'uint16',
    position_byte_offset: 0, index_byte_offset: 20, position_offset_m: [-0.1, -0.1, 0],
    position_scale_m: [0.2 / 65535, 0.2 / 65535, 0] };
  const manifest = { available: true, asset_id: 'robot-a', format: 'nuwax-robot-mesh-v1',
    mesh_url: '/api/robot-model/robot-a/mesh', geometries: [descriptor],
    visuals: ['torso', 'hand'].map(link => ({ link, geometry: 'triangle', matrix: identity, rgba: [0.4, 0.6, 0.8, 1] })),
    reference_transforms: reference, geometry: { footprint_xy_m: [[0, 0], [1, 0], [0, 1]] } };
  let time = 1000;
  const create = (options = {}) => {
    const layer = new RobotModelLayer({ now: () => time, ...options });
    layers.push(layer);
    return layer;
  };
  const geometry = decodeRobotGeometry(descriptor, buffer);
  assert.equal(geometry.index.count, 3);
  assert.ok(Math.abs(geometry.attributes.position.getX(1) - 0.1) < 1e-7);
  assert.throws(() => decodeRobotGeometry({ ...descriptor, index_byte_offset: 40 }, buffer), /数据范围/);
  const invalid = buffer.slice(0);
  new Uint16Array(invalid, 20, 3)[2] = 9;
  assert.throws(() => decodeRobotGeometry(descriptor, invalid), /索引越界/);
  geometry.dispose();

  const layer = create();
  await layer.loadMesh(manifest, buffer);
  assert.equal(layer.links.size, 2);
  assert.equal(layer.links.get('hand').children[0].geometry, layer.links.get('torso').children[0].geometry);
  assert.equal(layer.links.get('hand').children[0].material.transparent, true);
  const movedHand = new Matrix4().makeRotationZ(Math.PI / 2).toArray();
  const live = { available: true, asset_id: manifest.asset_id, mode: 'live', valid_for_sec: 0.4,
    link_transforms: { torso: identity, hand: movedHand } };
  assert.equal(layer.applyState(live), true);
  assert.equal(layer.status.mode, 'live');
  const point = new Vector3(1, 0, 0).applyMatrix4(layer.links.get('hand').matrix);
  assert.ok(Math.abs(point.x) < 1e-8 && Math.abs(point.y - 1) < 1e-8);
  // 关节每秒取一次：取到时有效的状态保持一个取数周期加其有效期（1000 + 400 毫秒）。
  time += 1000;
  layer.checkFreshness();
  assert.equal(layer.status.mode, 'live');
  time += 401;
  layer.checkFreshness();
  assert.equal(layer.status.mode, 'reference');
  assert.equal(layer.links.get('hand').matrix.elements[12], 0.4);
  assert.match(layer.status.message, /过期/);
  // 传输耗时计入时效，不在响应到达时续签旧关节。
  layer.applyState(live, time - 1500);
  assert.equal(layer.status.mode, 'reference');
  layer.applyState({ ...live, link_transforms: { torso: identity } });
  assert.equal(layer.status.mode, 'reference');
  assert.match(layer.status.message, /不完整/);
  layer.setPoseSource({ frame_id: 'map-b', timestamp: '2026-10-29T09:00:00Z' });
  layer.applyState({ ...live, pose_frame_id: 'map-a', joint_source_timestamp: '2026-10-29T09:00:00Z' });
  assert.equal(layer.status.mode, 'reference');
  layer.setPoseSource({ frame_id: 'map-a', timestamp: '2026-10-29T08:00:00Z' });
  time += 501;
  layer.applyState({ ...live, pose_frame_id: 'map-a', joint_source_timestamp: '2026-10-29T09:00:00Z' });
  assert.equal(layer.status.mode, 'reference');

  let disposedCount = 0;
  layer.links.get('torso').children[0].geometry.addEventListener('dispose', () => disposedCount++);
  layer.applyState({ ...live, asset_id: 'robot-b' });
  assert.equal(layer.content, null);
  assert.equal(disposedCount, 1);

  let failedRequests = 0;
  const failures = create({ fetcher: async () => { failedRequests++; return { ok: true, json: async () => ({ available: false,
    message: '网格文件缺失', geometry: manifest.geometry }) }; } });
  await failures.refresh();
  assert.equal(failures.status.available, false);
  assert.match(failures.status.message, /网格文件缺失/);
  failures.running = true;
  for (let i = 0; i < 20; i++) failures.applyState({ available: false, asset_id: null });
  assert.equal(failedRequests, 1);
  failures.stop();

  let releaseMesh;
  const pendingMesh = new Promise(resolve => { releaseMesh = resolve; });
  const racing = create({ fetcher: async url => ({ ok: true,
    json: async () => manifest, arrayBuffer: async () => pendingMesh }) });
  const loading = racing.refresh();
  await new Promise(resolve => setImmediate(resolve));
  racing.applyState({ ...live, asset_id: 'robot-b' });
  releaseMesh(buffer);
  await loading;
  assert.equal(racing.content, null);
  assert.equal(racing.manifest, null);

  // 分批解码期间换配置，已经展开的部分网格也不能重新挂回场景。
  const decoding = create();
  const many = { ...manifest, geometries: Array.from({ length: 9 }, (_, id) => ({ ...descriptor, id })),
    visuals: [{ ...manifest.visuals[0], geometry: 0 }] };
  const decodingTask = decoding.loadMesh(many, buffer);
  decoding.clearModel();
  await assert.rejects(decodingTask, error => error.code === 'robot_model_cancelled');
  assert.equal(decoding.content, null);

  const fresh = create();
  await fresh.loadMesh(manifest, buffer);
  fresh.applyState({ ...live, asset_id: 'previous-robot' }, time - 100);
  assert.equal(fresh.manifest.asset_id, manifest.asset_id);
  assert.ok(fresh.content);

  const closed = create({ fetcher: async () => ({ ok: true, json: async () => manifest,
    arrayBuffer: async () => buffer }) });
  const closing = closed.refresh();
  closed.dispose();
  await closing;
  assert.equal(closed.content, null);
  assert.equal(closed.root.children.length, 0);

  console.log('PASS 机器人模型：网格解码/索引安全/完整部件复用/矩阵关节运动/原始时效/位姿配对/参考回退/换模型/异步旧响应/资源释放');
} finally {
  for (const layer of layers) layer.dispose();
  fs.rmSync(temporary, { recursive: true, force: true });
}
