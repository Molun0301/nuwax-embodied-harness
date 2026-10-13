#!/usr/bin/env node
// 无浏览器的真实模块检查：虚拟时钟只统计请求，不把耗时当作端到端性能。
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { setImmediate as settle } from 'node:timers/promises';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-dense-polling-'));
for (const name of ['dense-pointcloud.js', 'pointcloud-incremental.js', 'three.module.min.js']) {
  fs.copyFileSync(path.join(root, 'gateways/http/static/js', name), path.join(temporary, name));
}
fs.writeFileSync(path.join(temporary, 'package.json'), '{"type":"module"}');
const { DensePointCloudLayer } = await import(pathToFileURL(path.join(temporary, 'dense-pointcloud.js')));
const originalSetTimeout = globalThis.setTimeout, originalClearTimeout = globalThis.clearTimeout;
let now = 0, serial = 0, checks = 0;
const timers = new Map(), layers = [];
globalThis.setTimeout = (fn, delay) => { const id = ++serial; timers.set(id, { fn, at: now + delay }); return id; };
globalThis.clearTimeout = id => timers.delete(id);
async function advance(milliseconds) {
  const until = now + milliseconds;
  for (;;) {
    const next = [...timers].sort((a, b) => a[1].at - b[1].at)[0];
    if (!next || next[1].at > until) break;
    now = next[1].at; timers.delete(next[0]); next[1].fn(); await settle();
  }
  now = until;
}
function source(generation = '1', kind = 'assigned') {
  const assigned = { identity_schema_version: 2, asset_identity: { asset_id: 'a'.repeat(32) },
    checkpoint_id: 'b'.repeat(32), installation_generation: generation };
  return { source: kind === 'live' ? 'live' : 'saved', frame_id: 'map', stream_generation: '1',
    source_epoch: kind === 'assigned' ? `assigned:${assigned.asset_identity.asset_id}:${assigned.checkpoint_id}:${generation}` : `${kind}:${generation}`,
    ...(kind === 'assigned' ? { assigned_source: assigned } : {}) };
}
function fixture({ kind = 'assigned', total = 2, chunk = 65536, failFirst = false } = {}) {
  const calls = [], states = [];
  const layer = new DensePointCloudLayer({ onChange: state => states.push(state), fetchChunk: async url => {
    const query = new URL(url, 'https://fixture.example').searchParams;
    calls.push(query);
    if (failFirst && calls.length === 1) return { ok: false, status: 503, json: async () => ({ message: 'fixture unavailable' }) };
    const display = layer.displaySource, offset = Number(query.get('offset'));
    const pointCount = Math.min(chunk, total - offset);
    const metadata = { epoch: display.assigned_source ? display.source_epoch : 'dense:1',
      offset_points: String(offset), total_points: String(total), point_count: pointCount,
      frame_id: display.frame_id, session_id: 'fixture', session_generation: '1', revision: String(total + 1),
      source_timestamp_ns: '1', bounds_min: [0, 0, 0], bounds_max: [1, 1, 1], display_source_epoch: display.source_epoch,
      ...(display.assigned_source ? { assigned_source: display.assigned_source } : {}) };
    return { ok: true, status: 200, headers: { get: () => JSON.stringify(metadata) },
      arrayBuffer: async () => new Float32Array(pointCount * 3).buffer };
  } });
  layers.push(layer); layer.setDisplaySource(source('1', kind));
  return { layer, calls, states, setTotal: value => { total = value; } };
}
try {
  const saved = fixture(); saved.layer.start(); await settle(); await advance(60000);
  console.log(JSON.stringify({ virtual_seconds: 60, assigned_requests: saved.calls.length,
    loaded_points: saved.layer.count, pending_timers: timers.size }));
  if (!process.argv.includes('--measure')) {
    assert.equal(saved.calls.length, 1); assert.equal(saved.layer._timer, null);
    assert.equal(saved.layer._abort, null); assert.equal(saved.layer.group.visible, true); checks++;
    const geometry = saved.layer.group.children[0].geometry;
    saved.layer.stop(); saved.layer.start(); await settle(); await advance(1000);
    assert.equal(saved.calls.length, 1); assert.equal(saved.layer.group.children[0].geometry, geometry); checks++;
    saved.layer.setDisplaySource(source()); await settle(); assert.equal(saved.calls.length, 1);
    saved.layer.setDisplaySource(source(), { reset: true }); await settle();
    assert.equal(saved.calls.length, 2); assert.equal(saved.calls[1].get('offset'), '0'); checks++;
    saved.layer.setDisplaySource(source('2')); await settle();
    assert.equal(saved.calls.length, 3); assert.equal(saved.calls[2].get('source_epoch'), source('2').source_epoch); checks++;
    saved.layer.setDisplaySource(null); await advance(1000);
    assert.equal(saved.layer.count, 0); assert.equal(saved.layer._active, false); assert.equal(saved.calls.length, 3); checks++;
    const empty = fixture({ total: 0 }); empty.layer.start(); await settle(); await advance(1000);
    assert.equal(empty.calls.length, 1); assert.equal(empty.layer.count, 0); assert.ok(empty.layer.metadata); checks++;
    const partial = fixture({ total: 5, chunk: 2, failFirst: true }); partial.layer.start(); await settle();
    assert.equal(partial.calls.length, 1); assert.ok(partial.states.some(state => state.error));
    await advance(2000); assert.equal(partial.calls.length, 4); assert.equal(partial.layer.count, 5);
    assert.deepEqual(partial.calls.map(query => query.get('offset')), ['0', '0', '2', '4']);
    await advance(1000); assert.equal(partial.calls.length, 4); checks++;
    for (const kind of ['live', 'legacy']) {
      const dynamic = fixture({ kind }); dynamic.layer.start(); await settle(); await advance(1000);
      assert.equal(dynamic.calls.length, 3); assert.equal(dynamic.calls[0].get('delta'), '1');
      dynamic.setTotal(4); await advance(500); assert.equal(dynamic.layer.count, 4);
      dynamic.layer.stop(); checks++;
    }
    console.log(JSON.stringify({ checks, scope: 'CPU module/fake timer checks; no browser, network or WebGL benchmark' }));
  }
} finally {
  for (const layer of layers) layer.dispose();
  assert.equal(timers.size, 0);
  globalThis.setTimeout = originalSetTimeout; globalThis.clearTimeout = originalClearTimeout;
  fs.rmSync(temporary, { recursive: true, force: true });
}
