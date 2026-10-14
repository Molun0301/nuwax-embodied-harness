#!/usr/bin/env node
// Deterministic CPU protocol/storage properties, not browser/GPU timing or FPS.
// Reference copies are intentional test-only oracle work, never production work.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const js = path.join(root, 'gateways/http/static/js');
const moduleUrl = text => `data:text/javascript;base64,${Buffer.from(text).toString('base64')}`;
const read = name => fs.readFileSync(path.join(js, name), 'utf8');
const threeUrl = moduleUrl(read('three.module.min.js'));
const { IncrementalPointCloudLayer, POINT_CLOUD_PAGE_POINTS: PAGE, POINT_CLOUD_RENDER_POINTS: SLAB } =
  await import(moduleUrl(read('pointcloud-pages.js').replace("'./three.module.min.js'", JSON.stringify(threeUrl))));
const { IncrementalPointCloudConnection, parsePointCloudPagePacket } = await import(moduleUrl(read('pointcloud-incremental.js')));
const originals = { window: globalThis.window, WebSocket: globalThis.WebSocket };
const timers = new Map(); let timerId = 0;
class Socket extends EventTarget {
  static OPEN = 1;
  constructor() { super(); this.readyState = 0; this.sent = []; }
  send(text) { this.sent.push(JSON.parse(text)); }
  emit(type, data) {
    if (type === 'open') this.readyState = 1;
    if (type === 'close') this.readyState = 3;
    const event = new Event(type);
    if (data !== undefined) Object.defineProperty(event, 'data', { value: data });
    this.dispatchEvent(event);
  }
  close() { this.emit('close'); }
}
globalThis.WebSocket = Socket;
globalThis.window = { location: { href: 'https://replay.invalid/' },
  setTimeout(fn) { const id = ++timerId; timers.set(id, fn); return id; },
  clearTimeout(id) { timers.delete(id); } };
let seed = 0x19a742d;
function random(maximum) { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % maximum; }
let pointId = 0;
function newPoints(count) {
  const out = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) { const id = ++pointId; out.set([id / 16, (id % 97) / 8, (id % 29) / 8], i * 3); }
  return out;
}
let revision = 0, generation = 1, transfer = 0;
function encode(header, xyz) {
  const json = new TextEncoder().encode(JSON.stringify(header));
  const padding = (4 - (13 + json.length) % 4) % 4;
  const buffer = new ArrayBuffer(13 + json.length + padding + xyz.byteLength), bytes = new Uint8Array(buffer);
  bytes.set([0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0, 2]);
  new DataView(buffer).setUint32(9, json.length + padding, true);
  bytes.set(json, 13); bytes.fill(32, 13 + json.length, 13 + json.length + padding);
  bytes.set(new Uint8Array(xyz.buffer, xyz.byteOffset, xyz.byteLength), 13 + json.length + padding);
  return buffer;
}
function packets(next, indices, reset = false) {
  const previous = revision; revision++;
  const pages = Math.ceil(next.length / 3 / PAGE);
  const ids = reset ? Array.from({ length: pages }, (_, i) => i) : [...new Set(indices)].sort((a, b) => a - b);
  const packetCount = Math.max(1, Math.ceil(ids.length / 4)), transferId = `replay-${++transfer}`;
  return Array.from({ length: packetCount }, (_, packetIndex) => {
    const chunks = ids.slice(packetIndex * 4, packetIndex * 4 + 4).map(index => ({ chunk_index: index,
      point_count: Math.min(PAGE, next.length / 3 - index * PAGE), revision_ordinal: String(revision) }));
    const xyz = new Float32Array(chunks.reduce((sum, chunk) => sum + chunk.point_count * 3, 0));
    let offset = 0;
    for (const chunk of chunks) { const view = next.subarray(chunk.chunk_index * PAGE * 3, (chunk.chunk_index * PAGE + chunk.point_count) * 3); xyz.set(view, offset); offset += view.length; }
    return encode({ topic: 'pointcloud_map.pages', format: 'float32-xyz-le', source: 'live', source_epoch: `map:${generation}`,
      mapping_source_epoch: `map:${generation}`, frame_id: 'map', stream_generation: String(generation), layout_generation: String(generation),
      revision_ordinal: String(revision), base_revision_ordinal: reset ? '0' : String(previous), revision: `live:${revision}`,
      transfer_id: transferId, source_timestamp_ns: String(revision), reset, source_incremental: true, chunk_size_points: PAGE,
      point_count: next.length / 3, changed_chunk_count: ids.length, packet_count: packetCount, packet_index: packetIndex,
      payload_length: xyz.byteLength, chunks }, xyz);
  });
}
const layer = new IncrementalPointCloudLayer();
let reference = new Float32Array(), expected = null, accepted = 0, errors = 0, lastError = null;
let geometryCreated = 0, geometryDisposed = 0, materialDisposed = 0;
const watched = new WeakSet();
const metrics = { seed: '0x19a742d', acceptedTransactions: 0, expectedResyncs: 0, reconnects: 0,
  maxPoints: 0, maxPages: 0, maxSlabs: 0, maxPendingBytes: 0, maxPendingChunks: 0,
  copiedCoordinates: 0, scannedCoordinates: 0, scheduledUploadBytes: 0, maxAllocatedSourceBytes: 0 };
for (const material of [layer.material, layer.derivedMaterial]) material.addEventListener('dispose', () => materialDisposed++);
function watch() {
  for (const slab of layer.sourceSlabs) if (!watched.has(slab.geometry)) {
    watched.add(slab.geometry); geometryCreated++;
    let count = 0; slab.geometry.addEventListener('dispose', () => { assert.equal(++count, 1); geometryDisposed++; });
  }
}
function verify() {
  assert.equal(layer.pointCount, reference.length / 3);
  assert.equal(layer.pages.length, Math.ceil(layer.pointCount / PAGE));
  assert.equal(layer.sourceSlabs.length, Math.ceil(layer.pointCount / SLAB));
  assert.equal(layer.sourceGroup.children.length, layer.sourceSlabs.length);
  assert.equal(geometryCreated - geometryDisposed, layer.sourceSlabs.length);
  for (let i = 0; i < layer.pages.length; i++) {
    const page = layer.pages[i], start = i * PAGE * 3;
    assert.deepEqual(page.array.subarray(0, page.count * 3), reference.subarray(start, start + page.count * 3));
    assert.equal(page.array.buffer, layer.sourceSlabs[Math.floor(i / 16)].array.buffer);
  }
  assert.equal(layer._pendingSource.size, 0); assert.equal(layer._sourceUploads.size, 0);
  assert.equal(layer.derivedPages.length, 0); assert.equal(layer._voxels.size, 0);
  assert.equal([...layer._histogram.values()].reduce((sum, entry) => sum + entry.count, 0), layer.pointCount);
  assert.ok(layer._sourceExtents.nodes.size <= layer.pages.length * (Math.log2(layer._sourceExtents.capacity) + 1));
  metrics.maxPoints = Math.max(metrics.maxPoints, layer.pointCount);
  metrics.maxPages = Math.max(metrics.maxPages, layer.pages.length);
  metrics.maxSlabs = Math.max(metrics.maxSlabs, layer.sourceSlabs.length);
  metrics.maxAllocatedSourceBytes = Math.max(metrics.maxAllocatedSourceBytes, layer.sourceSlabs.length * SLAB * 12);
}
const connection = new IncrementalPointCloudConnection({ onError(error) { errors++; lastError = error; }, onBatch(batch) {
  assert.ok(expected, 'Unexpected accepted transaction');
  const before = layer.pages.slice(), beforeCounts = before.map(page => page.count), slabs = layer.sourceSlabs.slice();
  for (const slab of slabs) slab.geometry.getAttribute('position').clearUpdateRanges(); // simulated completed render, no WebGL
  const changed = new Set(batch.chunks.map(chunk => chunk.chunk_index));
  for (const chunk of batch.chunks) assert.ok(expected.wire.includes(chunk.xyz.buffer), 'Decoder must retain packet-backed views');
  layer.apply(batch); watch();
  const coordinates = batch.chunks.reduce((sum, chunk) => sum + chunk.xyz.length, 0);
  assert.equal(layer.stats.lastApply.copiedCoordinates, coordinates);
  const tail = Math.ceil(batch.header.point_count / PAGE) - 1;
  const tailCount = batch.header.point_count - tail * PAGE;
  const trimmedTailCoordinates = !batch.header.reset && tail >= 0 && !changed.has(tail)
    && beforeCounts[tail] > tailCount ? tailCount * 3 : 0;
  assert.equal(layer.stats.lastApply.scannedCoordinates, coordinates + trimmedTailCoordinates);
  assert.equal(layer.stats.lastApply.derivedCopiedCoordinates, 0);
  if (!batch.header.reset) {
    for (let i = 0; i < Math.min(before.length, layer.pages.length); i++) assert.equal(layer.pages[i], before[i]);
    for (let i = 0; i < Math.min(slabs.length, layer.sourceSlabs.length); i++) assert.equal(layer.sourceSlabs[i], slabs[i]);
    const newSlabBytes = Math.max(0, layer.sourceSlabs.length - slabs.length) * SLAB * 12;
    assert.ok(layer.stats.lastApply.gpuDirtyBytes <= coordinates * 4 + newSlabBytes);
    for (let i = 0; i < Math.min(slabs.length, layer.sourceSlabs.length); i++) {
      if (![...changed].some(index => Math.floor(index / 16) === i)) assert.equal(slabs[i].geometry.getAttribute('position').updateRanges.length, 0);
    }
  }
  metrics.scannedCoordinates += layer.stats.lastApply.scannedCoordinates; metrics.copiedCoordinates += coordinates; metrics.scheduledUploadBytes += layer.stats.lastApply.gpuDirtyBytes;
  reference = expected.points; accepted++; verify();
} });
function deliver(wire, next) {
  expected = { wire, points: next };
  const count = accepted, before = connection.assembler.base;
  for (let i = 0; i < wire.length; i++) {
    parsePointCloudPagePacket(wire[i]); // valid fixture; delivery order errors are tested separately
    connection.socket.emit('message', wire[i]);
    const pending = connection.assembler.pending;
    if (pending) {
      metrics.maxPendingBytes = Math.max(metrics.maxPendingBytes, pending.bytes);
      metrics.maxPendingChunks = Math.max(metrics.maxPendingChunks, pending.chunks.length);
      assert.ok(pending.bytes <= next.byteLength); assert.ok(pending.chunks.length <= Math.ceil(next.length / 3 / PAGE));
    }
    if (i + 1 < wire.length) { assert.equal(accepted, count); assert.equal(connection.assembler.base, before); }
  }
  assert.equal(accepted, count + 1, lastError?.message); assert.equal(connection.assembler.pending, null);
  assert.equal(connection.assembler.pointCount, next.length / 3);
  assert.equal(connection.socket.sent.at(-1).type, 'pointcloud.ack');
  expected = null;
}
function commit(next, dirty, reset = false) { deliver(packets(next, dirty, reset), next); }
function reconnect() {
  const old = connection.socket, base = connection.assembler.base;
  old.close(); assert.equal(timers.size, 1); assert.equal(connection.assembler.pending, null);
  const [id, callback] = timers.entries().next().value; timers.delete(id); callback();
  connection.socket.emit('open'); assert.equal(timers.size, 0);
  assert.deepEqual(connection.socket.sent[0].base, base);
  metrics.reconnects++; return old;
}
try {
  connection.connect(); connection.socket.emit('open');
  commit(newPoints(PAGE * 17 + 9), [], true);
  for (let round = 0; round < 12; round++) {
    for (let step = 0; step < 16; step++) {
      const count = reference.length / 3;
      if (step % 4 === 0) {
        const extra = newPoints(1 + random(PAGE * 2)), next = new Float32Array(reference.length + extra.length);
        next.set(reference); next.set(extra, reference.length);
        const first = Math.floor(count / PAGE), last = Math.ceil(next.length / 3 / PAGE);
        commit(next, Array.from({ length: last - first }, (_, i) => first + i));
      } else if (step % 4 === 1) {
        const next = reference.slice(), index = random(count); next.set(newPoints(1), index * 3);
        commit(next, [Math.floor(index / PAGE)]);
      } else if (step % 4 === 2) {
        // Producer-style dense-slot compaction: replace a removed slot with the last point.
        const index = random(count - 1), next = reference.slice(0, -3); next.set(reference.subarray(-3), index * 3);
        commit(next, [Math.floor(index / PAGE)]);
      } else {
        // Count-only tail deletion must not copy or upload untouched XYZ.
        const next = reference.slice(0, Math.max(1, count - 1 - random(PAGE * 2)) * 3);
        commit(next, []); assert.equal(layer.stats.lastApply.copiedCoordinates, 0); assert.equal(layer.stats.lastApply.gpuDirtyBytes, 0);
      }
    }
    const interrupted = packets(newPoints(PAGE * 19 + 1), [], true);
    connection.socket.emit('message', interrupted[0]); assert.ok(connection.assembler.pending);
    const old = reconnect(), count = accepted, oldErrors = errors;
    for (const packet of interrupted) old.emit('message', packet);
    old.emit('close'); old.emit('error');
    assert.equal(accepted, count); assert.equal(errors, oldErrors); assert.equal(timers.size, 0);
    assert.equal(connection.assembler.pending, null); verify();
    generation++; revision = 0; commit(newPoints(PAGE * (round % 2 ? 17 : 33) + 7), [], true);
    const outOfOrder = packets(reference, [], true), prior = connection.assembler.base;
    // An ordered WebSocket stream cannot naturally reorder packets; this is explicit resync fault injection.
    connection.socket.emit('message', outOfOrder[1]);
    assert.equal(errors, oldErrors + 1); assert.equal(connection.awaitingReset, true);
    assert.equal(connection.assembler.base, prior); assert.equal(connection.assembler.pending, null); verify();
    connection.socket.emit('message', outOfOrder[1]); assert.equal(errors, oldErrors + 1);
    deliver(outOfOrder, reference); assert.equal(connection.awaitingReset, false);
    // Same-connection earlier generation reset must be refused, without accepting stale data.
    const beforeStale = errors; connection.socket.emit('message', interrupted[0]);
    assert.equal(errors, beforeStale + 1); assert.equal(connection.awaitingReset, true); verify();
    commit(reference, [], true);
  }
  commit(new Float32Array(), []); assert.equal(layer.sourceSlabs.length, 0);
  const last = connection.socket;
  connection.dispose(); layer.dispose(); layer.dispose();
  last.emit('message', packets(newPoints(1), [], true)[0]);
  assert.equal(timers.size, 0); assert.equal(connection.socket, null); assert.equal(connection.assembler.pending, null);
  assert.equal(connection.assembler.base, null); assert.equal(geometryDisposed, geometryCreated); assert.equal(materialDisposed, 2);
  metrics.acceptedTransactions = accepted; metrics.expectedResyncs = errors;
  metrics.geometryCreated = geometryCreated; metrics.geometryDisposed = geometryDisposed;
  metrics.retainedSourceSlabsAfterDispose = layer.sourceSlabs.length;
  console.log('PASS deterministic live-v2 incremental replay, resync, compaction and reconnect retention');
  console.log(JSON.stringify(metrics, null, 2));
  console.log('Counts cover reachable module resources and scheduled uploads, not JS heap/GC, GPU execution or browser performance.');
} finally {
  connection.dispose(); layer.dispose();
  for (const [key, value] of Object.entries(originals)) { if (value === undefined) delete globalThis[key]; else globalThis[key] = value; }
}
