#!/usr/bin/env node
// Self-contained CPU/Three module checks. No fixture generator, browser or WebGL required.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { setImmediate as nextTurn } from 'node:timers/promises';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const repository = path.resolve(process.argv[2] || root);
const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-assigned-pairing-'));
const modules = ['pointcloud3d.js', 'pointcloud-incremental.js', 'pointcloud-pages.js',
  'three.module.min.js', 'semantic-regions3d.js', 'scene-graph3d.js', 'OrbitControls.js',
  'pointcloud-processing.js', 'dense-pointcloud.js'];
for (const name of modules) {
  const staged = path.join(root, 'gateways/http/static/js', name);
  fs.copyFileSync(fs.existsSync(staged) ? staged : path.join(repository, 'gateways/http/static/js', name), path.join(temporary, name));
}
fs.writeFileSync(path.join(temporary, 'package.json'), '{"type":"module"}');
const load = name => import(pathToFileURL(path.join(temporary, name)));
const PAGE = 1024;
const asset = 'a'.repeat(32), checkpoint = 'b'.repeat(32);
const source = (generation = '1', assetId = asset, checkpointId = checkpoint) => ({
  identity_schema_version: 2, asset_identity: { asset_id: assetId },
  checkpoint_id: checkpointId, installation_generation: generation,
});
const epochOf = value => `assigned:${value.asset_identity.asset_id}:${value.checkpoint_id}:${value.installation_generation}`;
const sourceA = source(), sourceB = source('2');
const mapFor = assigned => ({source_epoch: 'saved-recording:7', metadata: {
  identity_schema_version: 2, asset_identity: assigned.asset_identity,
  map_activation_generation: assigned.installation_generation,
  activation_receipt_v2: {request: {checkpoint_id: assigned.checkpoint_id}},
}});
const points = count => Float32Array.from({ length: count * 3 }, (_, coordinate) => {
  const point = Math.floor(coordinate / 3);
  return coordinate % 3 === 2 ? Math.floor(point / 600) * 3 : ((point * (coordinate % 3 ? 13 : 7)) % 41) * .11;
});
let transfer = 0;
function headerFor(assigned = sourceA, overrides = {}) {
  return {
    topic: 'pointcloud_map.pages', format: 'float32-xyz-le', source: 'saved',
    source_epoch: epochOf(assigned), assigned_source: structuredClone(assigned),
    frame_id: 'map', stream_generation: '1', layout_generation: '1', revision_ordinal: '1',
    base_revision_ordinal: '0', revision: 'saved:1', transfer_id: `assigned-transfer-${++transfer}`,
    mapping_source_epoch: epochOf(assigned), source_timestamp_ns: '123000000',
    reset: true, source_incremental: false, chunk_size_points: PAGE, ...overrides,
  };
}
function encode(header, xyz, version = 2) {
  const json = new TextEncoder().encode(JSON.stringify(header));
  const padding = version === 2 ? (4 - (13 + json.length) % 4) % 4 : 0;
  const result = new Uint8Array(13 + json.length + padding + xyz.byteLength);
  result.set([0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0, version]);
  new DataView(result.buffer).setUint32(9, json.length + padding, true);
  result.set(json, 13);
  result.fill(32, 13 + json.length, 13 + json.length + padding);
  result.set(new Uint8Array(xyz.buffer, xyz.byteOffset, xyz.byteLength), 13 + json.length + padding);
  return result.buffer;
}
function packets(header, xyz = points(PAGE * 5 + 7)) {
  const total = xyz.length / 3, chunks = Math.ceil(total / PAGE), packetCount = Math.max(1, Math.ceil(chunks / 4));
  return Array.from({ length: packetCount }, (_, packetIndex) => {
    const startPage = packetIndex * 4, endPage = Math.min(chunks, startPage + 4);
    const payload = xyz.subarray(startPage * PAGE * 3, Math.min(xyz.length, endPage * PAGE * 3));
    return encode({ ...header, point_count: total, changed_chunk_count: chunks, packet_count: packetCount,
      packet_index: packetIndex, payload_length: payload.byteLength,
      chunks: Array.from({ length: endPage - startPage }, (_, index) => {
        const chunkIndex = startPage + index;
        return { chunk_index: chunkIndex, point_count: Math.min(PAGE, total - chunkIndex * PAGE), revision_ordinal: header.revision_ordinal };
      }),
    }, payload);
  });
}
function denseMetadata(display = headerFor(), overrides = {}) {
  return {
    epoch: display.assigned_source ? display.source_epoch : 'dense:1', offset_points: '0', total_points: '2', point_count: 2,
    frame_id: display.frame_id, session_id: 'dense-session', session_generation: '1',
    revision: '1', source_timestamp_ns: '123000000', bounds_min: [0, 0, 0], bounds_max: [5, 5, 5],
    display_source_epoch: display.source_epoch,
    ...(display.assigned_source ? { assigned_source: structuredClone(display.assigned_source) } : {}),
    ...overrides,
  };
}
const denseBytes = () => new Float32Array([0, 1, 2, 3, 4, 5]).buffer;
function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}
function controlledFetch() {
  const calls = [];
  const fetch = (url, options) => { const pending = deferred(); calls.push({ url, options, ...pending }); return pending.promise; };
  return { calls, fetch };
}
function response(metadata, body = denseBytes()) {
  return { ok: true, status: 200, headers: { get: name => name === 'X-Nuwax-Pointcloud' ? JSON.stringify(metadata) : null },
    arrayBuffer: () => Promise.resolve(body) };
}
function watchDisposals(resources) {
  const counts = new Map();
  for (const item of new Set(resources)) { counts.set(item, 0); item.addEventListener('dispose', () => counts.set(item, counts.get(item) + 1)); }
  return { counts, assertOnce() { for (const count of counts.values()) assert.equal(count, 1); } };
}
const original = Object.fromEntries(['setTimeout', 'clearTimeout', 'fetch', 'window', 'document', 'WebSocket', 'CustomEvent'].map(key => [key, globalThis[key]]));
let timerId = 0;
const timers = new Map();
const fakeSetTimeout = (fn, ms) => { const id = ++timerId; timers.set(id, { fn, ms }); return id; };
const fakeClearTimeout = id => timers.delete(id);
function runTimer(id) { const timer = timers.get(id); assert.ok(timer, `Missing scheduled timer ${id}`); timers.delete(id); timer.fn(); }
class Socket extends EventTarget {
  static OPEN = 1;
  static all = [];
  constructor(url) { super(); this.url = String(url); this.readyState = 0; this.sent = []; Socket.all.push(this); }
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
let checks = 0;
let realWireChecked = false;
const failures = [];
async function check(name, run) {
  try { await run(); checks++; console.log(`PASS ${name}`); }
  catch (error) { failures.push({ name, error }); console.error(`FAIL ${name}\n${error.stack}`); }
}
const layers = new Set();
let viewer;
try {
  globalThis.setTimeout = fakeSetTimeout;
  globalThis.clearTimeout = fakeClearTimeout;
  globalThis.WebSocket = Socket;
  globalThis.window = new EventTarget();
  Object.assign(window, { location: { href: 'https://robot.example/console' }, setTimeout: fakeSetTimeout, clearTimeout: fakeClearTimeout });
  globalThis.document = { getElementById: () => null };
  if (!globalThis.CustomEvent) globalThis.CustomEvent = class extends Event { constructor(type, options = {}) { super(type); this.detail = options.detail; } };
  const protocol = await load('pointcloud-incremental.js');
  const { parsePointCloudPagePacket, PointCloudPageAssembler } = protocol;
  const { DensePointCloudLayer, decodeDenseChunk } = await load('dense-pointcloud.js');
  const THREE = await load('three.module.min.js');
  const makeDense = options => { const layer = new DensePointCloudLayer(options); layers.add(layer); return layer; };

  await check('page metadata remains opaque while payload views preserve exact received coordinates', () => {
    const xyz = points(2), header = headerFor(sourceA, { assigned_source: { backend_label: 'opaque' },
      source_epoch: 'server-selected-source', revision_ordinal: 7, future_metadata: 'retained' });
    const buffer = packets(header, xyz)[0], packet = parsePointCloudPagePacket(buffer);
    assert.deepEqual(packet.header.assigned_source, header.assigned_source);
    assert.equal(packet.header.revision_ordinal, 7); assert.equal(packet.header.future_metadata, 'retained');
    assert.deepEqual(packet.chunks[0].xyz, xyz); assert.equal(packet.chunks[0].xyz.buffer, buffer);
  });
  await check('page decoder keeps magic, version, header boundary and payload lengths', () => {
    const buffer = packets(headerFor(), points(2))[0];
    const magic = buffer.slice(0), version = buffer.slice(0), oversized = buffer.slice(0), unaligned = buffer.slice(0);
    new Uint8Array(magic)[0] = 0; new Uint8Array(version)[8] = 9;
    new DataView(oversized).setUint32(9, 65537, true);
    new DataView(unaligned).setUint32(9, new DataView(buffer).getUint32(9, true) - 1, true);
    for (const invalid of [new ArrayBuffer(0), magic, version, oversized, unaligned, buffer.slice(0, -4)]) {
      assert.throws(() => parsePointCloudPagePacket(invalid));
    }
    assert.throws(() => parsePointCloudPagePacket(encode({...headerFor(), point_count: 2,
      payload_length: 24, chunks: [{chunk_index: 0, point_count: 3}]}, points(2))));
  });
  await check('schema-2 map ownership uses installation identity rather than saved recording session', () => {
    const map = { source_epoch: 'old-recording:3', metadata: {
      identity_schema_version: 2, asset_identity: { asset_id: asset }, map_activation_generation: 1,
      activation_receipt_v2: { request: { checkpoint_id: checkpoint } },
      mapping_session_id: 'old-recording', session_generation: 3,
    } };
    assert.equal(protocol.mappingEpochFromMap(map), epochOf(sourceA));
    map.metadata.map_activation_generation = '2';
    assert.equal(protocol.mappingEpochFromMap(map), epochOf(sourceB));
    assert.equal(protocol.mappingEpochFromMap({ source_epoch: JSON.stringify(['map-view-v2', 'legacy-map:9', 'display']) }), 'legacy-map:9');
  });
  await check('assigned saved pages assemble atomically across bounded packets without XYZ flattening', () => {
    const assembler = new PointCloudPageAssembler(), input = packets(headerFor());
    assert.equal(assembler.push(input[0]), null); assert.equal(assembler.base, null);
    const batch = assembler.push(input[1]);
    assert.equal(batch.chunks.length, 6); assert.equal(assembler.base, null);
    assert.equal(batch.chunks[0].xyz.buffer, input[0]); assert.equal(batch.chunks[4].xyz.buffer, input[1]);
    assembler.accept(batch); assert.equal(assembler.pointCount, PAGE * 5 + 7);
    assert.equal(assembler.base.source_epoch, epochOf(sourceA));
    const next = packets(headerFor(sourceB, { stream_generation: '2' }));
    assembler.push(next[0]); assert.equal(assembler.base.source_epoch, epochOf(sourceA));
    assembler.beginConnection(); assert.equal(assembler.pending, null);
    assert.throws(() => assembler.push(next[1])); assert.equal(assembler.base.source_epoch, epochOf(sourceA));
    assembler.discardPending(); let resumed; for (const packet of next) resumed = assembler.push(packet);
    assembler.accept(resumed); assert.equal(assembler.base.source_epoch, epochOf(sourceB));
  });
  await check('new map selection discards the prior complete base and staged pages', () => {
    const assembler = new PointCloudPageAssembler(); assembler.setMapEpoch(epochOf(sourceA));
    assembler.accept(assembler.push(packets(headerFor(), points(2))[0]));
    assembler.push(packets(headerFor())[0]); assert.ok(assembler.pending);
    assert.equal(assembler.setMapEpoch(epochOf(sourceB)), true);
    assert.equal(assembler.base, null); assert.equal(assembler.pending, null);
    assert.equal(assembler.setMapEpoch(epochOf(sourceB)), false);
    assembler.accept(assembler.push(packets(headerFor(sourceB), points(2))[0]));
    assert.equal(assembler.setMapEpoch(''), true); assert.equal(assembler.base, null);
  });
  await check('dense chunks use byte lengths and incremental offsets without recreating assigned identity', () => {
    const valid = denseMetadata(headerFor(), {assigned_source: {backend_label: 'opaque'}, frame_id: 'server-frame'});
    assert.deepEqual(decodeDenseChunk(valid, denseBytes(), '', 0).positions, new Float32Array(denseBytes()));
    assert.throws(() => decodeDenseChunk(valid, denseBytes().slice(0, -4), '', 0));
    assert.throws(() => decodeDenseChunk(valid, denseBytes(), 'different-base', 0));
    assert.throws(() => decodeDenseChunk({...valid, offset_points: '1'}, denseBytes(), '', 0));
    assert.throws(() => decodeDenseChunk({...valid, base_epoch: 'missing', base_offset_points: '0', removed_range_bytes: 0}, denseBytes(), '', 0));
  });
  await check('dense polling sends the selected source and resets resources when the selection changes', async () => {
    const io = controlledFetch(), layer = makeDense({fetchChunk: io.fetch});
    try {
      layer.start(); assert.equal(io.calls.length, 0);
      layer.setDisplaySource(headerFor()); layer.start();
      const query = new URL(io.calls[0].url, 'https://robot.example').searchParams;
      assert.equal(query.get('epoch'), ''); assert.equal(query.get('offset'), '0');
      assert.equal(query.get('source_epoch'), epochOf(sourceA)); assert.equal(query.get('delta'), '0');
      io.calls[0].resolve(response(denseMetadata(headerFor()))); await nextTurn(); assert.equal(layer.count, 2);
      const serial = layer._serial; layer.setDisplaySource(headerFor()); assert.equal(layer._serial, serial);
      const resources = watchDisposals(layer.group.children.map(mesh => mesh.geometry));
      layer.setDisplaySource(headerFor(), {reset: true}); resources.assertOnce(); assert.equal(io.calls.length, 2);
      layer.setDisplaySource(null); assert.equal(io.calls[1].options.signal.aborted, true);
      io.calls[1].resolve(response(denseMetadata(headerFor()))); await nextTurn(); assert.equal(layer.count, 0);
      const legacy = headerFor(sourceA, {assigned_source: undefined, source_epoch: 'saved-legacy'});
      layer.setDisplaySource(legacy); layer.start();
      assert.equal(new URL(io.calls[2].url, 'https://robot.example').searchParams.get('delta'), '1');
      io.calls[2].resolve(response(denseMetadata(legacy))); await nextTurn(); assert.equal(layer.count, 2);
    } finally {layer.dispose(); layers.delete(layer);}
  });
  await check('new installation fences late fetch and late arrayBuffer before they touch dense resources', async () => {
    const io = controlledFetch(), layer = makeDense({ fetchChunk: io.fetch });
    try {
      const a = headerFor(), b = headerFor(sourceB);
      layer.setDisplaySource(a); layer.start(); const first = io.calls[0];
      layer.setDisplaySource(b); assert.equal(first.options.signal.aborted, true);
      first.resolve(response(denseMetadata(a))); await nextTurn(); assert.equal(layer.count, 0);
      const lateBody = deferred(); io.calls[1].resolve(response(denseMetadata(b), lateBody.promise)); await nextTurn();
      layer.setDisplaySource(a); assert.equal(io.calls[1].options.signal.aborted, true);
      io.calls[2].resolve(response(denseMetadata(a))); await nextTurn(); assert.equal(layer.count, 2);
      const geometry = layer.group.children[0].geometry, before = geometry.getAttribute('position').array.slice(0, 6);
      lateBody.resolve(denseBytes()); await nextTurn();
      assert.equal(layer.count, 2); assert.equal(layer.group.children[0].geometry, geometry);
      assert.deepEqual(geometry.getAttribute('position').array.slice(0, 6), before);
      assert.equal(layer.metadata.display_source_epoch, a.source_epoch);
    } finally { layer.dispose(); layers.delete(layer); }
  });
  await check('late non-OK JSON body cannot clear a newer successful dense installation', async () => {
    const io = controlledFetch(), changes = [], layer = makeDense({ fetchChunk: io.fetch, onChange: value => changes.push(value) });
    try {
      layer.setDisplaySource(headerFor()); layer.start(); const lateError = deferred();
      io.calls[0].resolve({ ok: false, status: 500, json: () => lateError.promise }); await nextTurn();
      layer.setDisplaySource(headerFor(sourceB)); io.calls[1].resolve(response(denseMetadata(headerFor(sourceB)))); await nextTurn();
      const geometry = layer.group.children[0].geometry, changeCount = changes.length;
      lateError.resolve({ error: { message: 'stale installation error' } }); await nextTurn();
      assert.equal(layer.count, 2); assert.equal(layer.group.children[0].geometry, geometry); assert.equal(changes.length, changeCount);
      assert.equal(layer.metadata.display_source_epoch, epochOf(sourceB));
    } finally { layer.dispose(); layers.delete(layer); }
  });
  await check('dense 409, repeat reset, hidden stop and dispose preserve owner and fence responses', async () => {
    const io = controlledFetch(), layer = makeDense({ fetchChunk: io.fetch });
    try {
      layer.setDisplaySource(headerFor()); layer.start(); io.calls[0].resolve({ ok: false, status: 409 }); await nextTurn();
      assert.equal(layer.displaySource.source_epoch, epochOf(sourceA)); assert.equal(layer.count, 0);
      runTimer(layer._timer); io.calls[1].resolve(response(denseMetadata(headerFor()))); await nextTurn();
      const resources = watchDisposals([layer.group.children[0].geometry]);
      layer.reset(); resources.assertOnce(); const old = io.calls[2]; layer.reset(); assert.equal(old.options.signal.aborted, true);
      old.resolve(response(denseMetadata(headerFor()))); await nextTurn(); assert.equal(layer.count, 0);
      layer.stop(); assert.equal(layer.group.visible, false); assert.equal(io.calls[3].options.signal.aborted, true);
      io.calls[3].resolve(response(denseMetadata(headerFor()))); await nextTurn(); assert.equal(layer.count, 0);
      layer.start(); io.calls[4].resolve(response(denseMetadata(headerFor()))); await nextTurn();
      const remaining = watchDisposals([layer.group.children[0].geometry, layer.material]);
      layer.dispose(); remaining.assertOnce(); assert.equal(layer.group.children.length, 0); assert.equal(layer._timer, null); layers.delete(layer);
    } finally { if (layers.delete(layer)) layer.dispose(); }
  });

  await check('dense geometry bounds ignore non-finite vertices and recover after an all-invalid chunk', () => {
    const layer = makeDense({});
    try {
      const raw = new Float32Array([-1, -2, 0, 2, 3, 1, NaN, 0, 0, 0, Infinity, 0, 0, 0, -Infinity]);
      const metadata = denseMetadata(headerFor(), {point_count: raw.length / 3, total_points: String(raw.length / 3)});
      layer.append(decodeDenseChunk(metadata, raw.buffer, '', 0));
      assert.equal(layer.count, 5); assert.ok(Number.isFinite(layer.bounds.radius));
      assert.deepEqual(layer.group.children[0].geometry.boundingBox.min.toArray(), [-1, -2, 0]);
      assert.deepEqual(layer.group.children[0].geometry.boundingBox.max.toArray(), [2, 3, 1]);
      layer.clear();
      const invalid = new Float32Array([NaN, 0, 0]);
      layer.append(decodeDenseChunk({...metadata, point_count: 1, total_points: '1'}, invalid.buffer, '', 0));
      assert.equal(layer.bounds, null); layer.clear();
      layer.append(decodeDenseChunk(denseMetadata(headerFor()), denseBytes(), '', 0));
      assert.ok(Number.isFinite(layer.bounds.radius));
    } finally {layer.dispose(); layers.delete(layer);}
  });
  const viewerIo = controlledFetch(); globalThis.fetch = viewerIo.fetch;
  const diagnostics = [];
  window.addEventListener('pointcloud3d:diagnostic', event => diagnostics.push(event.detail));
  const viewerModule = await load('pointcloud3d.js'); viewer = window.pointcloud3d;
  let socket = viewer._incrementalConnection.socket;
  await check('assigned metadata arriving before first page fences in-flight generic loading and new raw fallback', async () => {
    const loading = viewer.loadPointCloud('/artifacts/legacy-in-flight/points');
    assert.equal(viewerIo.calls.length, 1); const token = viewer._cloudLoadToken;
    viewer.setMapContext({ source_epoch: 'saved-recording:7', metadata: {
      identity_schema_version: 2, asset_identity: { asset_id: asset }, map_activation_generation: 1,
      activation_receipt_v2: { request: { checkpoint_id: checkpoint } },
      mapping_session_id: 'saved-recording', session_generation: 7,
    } });
    assert.ok(viewer._cloudLoadToken > token); assert.equal(viewer._assignedMapEpoch, epochOf(sourceA));
    viewerIo.calls[0].resolve({ ok: true, arrayBuffer: () => Promise.resolve(denseBytes()) }); await loading;
    assert.equal(viewer._allPositions.length, 0); assert.equal(viewer._hasPointCloudSnapshot, false);
    assert.equal(viewer._pointCloudMapWatermark, null);
    const guarded = viewer.loadPointCloud('/artifacts/new-unpaired/points'); assert.equal(viewerIo.calls.length, 1); await guarded;
    assert.throws(() => viewer.appendPoints(new Float32Array([9, 9, 9])));
  });
  await check('unaligned binary-v1 retains non-finite points without rejecting other vertices', () => {
    const xyz = new Float32Array([-1, -2, 0, 2, 3, 1, NaN, 0, 0, 0, Infinity, 0, 0, 0, NaN]);
    const common = {topic: 'pointcloud_map.snapshot', cursor: '0001', point_count: xyz.length / 3, payload_length: xyz.byteLength};
    let header = headerFor(sourceA, common), buffer = encode(header, xyz, 1);
    while ((13 + new DataView(buffer).getUint32(9, true)) % 4 === 0) {
      header = {...header, padding_label: (header.padding_label || '') + 'x'}; buffer = encode(header, xyz, 1);
    }
    const parsed = viewerModule.parsePointCloudMapBinaryEnvelope(buffer);
    assert.deepEqual(parsed.xyz, xyz); assert.equal(parsed.xyz.byteOffset, 0);
    assert.throws(() => viewerModule.parsePointCloudMapBinaryEnvelope(buffer.slice(0, -4)));
  });
  await check('legacy full-frame preprocessing keeps finite geometry, colors and floor output', async () => {
    const {PointCloudProcessor} = await load('pointcloud-processing.js');
    const finite = new Float32Array([-1, -2, 0, 2, 3, 1]);
    const mixed = new Float32Array([...finite, NaN, 0, 0, 0, Infinity, 0, 0, 0, NaN, 0, 0, -Infinity]);
    const prepare = (raw, style) => {
      const processor = new PointCloudProcessor(viewer._processingConfig);
      Object.assign(processor, {_allPositions: raw, _pointStyle: style, _lod: 'full', _isLiveCloudMapSnapshot: false});
      return processor._preparePointCloud();
    };
    for (const style of ['height', 'ground', 'structure']) {
      const result = prepare(mixed, style), expected = prepare(finite, style);
      assert.deepEqual(result.measurement, expected.measurement);
      assert.deepEqual(result.floorContext, expected.floorContext);
      if (style === 'height') {
        assert.deepEqual(result.positions, mixed);
        assert.deepEqual(result.colors.subarray(0, finite.length), expected.colors);
      } else {
        assert.deepEqual(result.positions, expected.positions); assert.deepEqual(result.colors, expected.colors);
      }
      const empty = prepare(new Float32Array([NaN, 0, 0, 0, Infinity, 0]), style);
      assert.equal(empty.positions.length, 0); assert.equal(empty.measurement, null);
    }
  });
  await check('explicit pending parking keeps exploration frontiers until the backend confirms terminal', () => {
    const detail = {canonical_exploration: true, exploration_status: 'failed', exploration_terminal: false,
      exploration_identity: {generation: '1', revision: '1', timestampNs: '1', requestId: 'pending-parking'},
      latest_frontiers: [{x: 1, y: 2, z: 0}], frontier_count: 1};
    window.dispatchEvent(new CustomEvent('stream:exploration.frontier', {detail}));
    assert.equal(viewer._explorationOverlayInactive, false); assert.equal(viewer._frontiers.length, 1);
    window.dispatchEvent(new CustomEvent('stream:exploration.frontier', {detail: {...detail,
      exploration_terminal: true, exploration_identity: {...detail.exploration_identity, revision: '2'}}}));
    assert.equal(viewer._explorationOverlayInactive, true); assert.equal(viewer._frontiers.length, 0);
  });
  const activeSocket = () => {
    socket = viewer._incrementalConnection.socket;
    if (socket?.readyState === 0) socket.emit('open');
    return socket;
  };
  activeSocket();
  const send = (header, xyz) => {
    const current = activeSocket();
    for (const packet of packets(header, xyz)) current.emit('message', packet);
  };
  await check('clear before first page discards staging and blocks delayed assigned packets and raw fallback', async () => {
    const input = packets(headerFor()); socket.emit('message', input[0]);
    assert.ok(viewer._incrementalConnection.assembler.pending); assert.equal(viewer._pointCloudMapWatermark, null);
    viewer.setMapContext(null);
    const assembler = viewer._incrementalConnection.assembler, messageCount = socket.sent.length;
    assert.equal(assembler.pending, null); assert.equal(assembler.base, null); assert.equal(assembler.mapEpoch, '');
    socket.emit('message', input[1]); socket.emit('message', input[0]);
    assert.equal(socket.sent.length, messageCount, 'Cleared packets must not cause ACKs or a resync loop');
    const requests = viewerIo.calls.length, guarded = viewer.loadPointCloud('/artifacts/cleared-fallback/points');
    assert.equal(viewerIo.calls.length, requests); await guarded;
    assert.throws(() => viewer.appendPoints(new Float32Array([9, 9, 9])));
    const legacyBytes = encode(headerFor(sourceA, { topic: 'pointcloud_map.snapshot', cursor: '1', point_count: 2, payload_length: 24,
      assigned_source: undefined, source_epoch: 'saved-legacy', mapping_source_epoch: 'legacy-map' }), points(2), 1);
    window.dispatchEvent(new CustomEvent('stream:pointcloud_map.binary', { detail: { buffer: legacyBytes } }));
    assert.equal(viewer._pointCloudMapWatermark, null); assert.equal(viewer._allPositions.length, 0);
    viewer.setMapContext(mapFor(sourceA)); activeSocket();
  });
  socket.emit('message', JSON.stringify({ type: 'pointcloud.ready', version: 2 }));
  await check('actual viewer installs assigned saved pages atomically with no cumulative legacy XYZ buffer', () => {
    assert.equal(socket.sent[0].mapping_source_epoch, epochOf(sourceA));
    const input = packets(headerFor()); socket.emit('message', input[0]);
    assert.equal(viewer._incrementalActive, false); assert.equal(viewer._allPositions.length, 0);
    assert.equal(socket.sent.filter(message => message.type === 'pointcloud.ack').length, 0);
    socket.emit('message', input[1]);
    assert.equal(viewer._incrementalActive, true); assert.equal(viewer._allPositions.length, 0);
    assert.equal(viewer._pointCloudSourcePoints, PAGE * 5 + 7); assert.equal(viewer._isLiveCloudMapSnapshot, false);
    assert.equal(viewer._pointCloudMapWatermark.header.source_epoch, epochOf(sourceA));
    assert.equal(socket.sent.at(-1).type, 'pointcloud.ack');
    assert.equal(socket.sent.at(-1).base.source_epoch, epochOf(sourceA));
  });
  await check('assigned viewer visibility, styles and robot floor changes keep canonical resource ownership', () => {
    const layer = viewer._incrementalCloud, arrays = layer.pages.map(page => page.array), geometries = layer.sourceSlabs.map(slab => slab.geometry);
    viewer.setVisible(false); assert.equal(layer.group.visible, false); viewer.setVisible(true); assert.equal(layer.group.visible, true);
    for (const style of ['mono', 'singleFloor', 'ground', 'structure', 'height']) {
      assert.equal(viewer.setPointStyle(style), true); assert.equal(viewer._allPositions.length, 0);
      layer.pages.forEach((page, index) => assert.equal(page.array, arrays[index]));
      layer.sourceSlabs.forEach((slab, index) => assert.equal(slab.geometry, geometries[index]));
    }
    viewer.setPointStyle('ground'); viewer.setRobotPose(0, 0, 3, 0); assert.equal(layer._robotZ, 3);
    viewer.clearRobotPose(); assert.equal(layer._robotZ, undefined); viewer.setPointStyle('height');
  });
  await check('selected paged asset keeps generic file loading and raw append inactive', async () => {
    const before = viewer._pointCloudMapWatermark, count = viewerIo.calls.length, cloudKey = viewer._cloudSourceKey;
    const guarded = viewer.loadPointCloud('/artifacts/unpaired-cloud/points');
    assert.equal(viewerIo.calls.length, count); await guarded; assert.throws(() => viewer.appendPoints(new Float32Array([9, 9, 9])));
    assert.equal(viewer._pointCloudMapWatermark, before); assert.equal(viewer._cloudSourceKey, cloudKey); assert.equal(viewer._allPositions.length, 0);
    assert.equal(viewer._pointCloudMapWatermark, before); assert.equal(viewer._cloudSourceKey, cloudKey);
  });
  await check('same asset new installation and repeated reset replace only owned page resources', () => {
    const layer = viewer._incrementalCloud, old = watchDisposals(layer.sourceSlabs.map(slab => slab.geometry));
    const second = headerFor(sourceB, { stream_generation: '2', revision: 'saved:2' });
    viewer.setMapContext(mapFor(sourceB));
    send(second, points(13)); old.assertOnce();
    assert.equal(viewer._incrementalCloud, layer); assert.equal(layer.pointCount, 13); assert.equal(viewer._allPositions.length, 0);
    assert.equal(viewer._pointCloudMapWatermark.header.source_epoch, epochOf(sourceB));
    send(second, points(13)); assert.equal(layer.pointCount, 13); assert.equal(viewer._allPositions.length, 0);
    assert.equal(viewer._pointCloudMapWatermark.header.assigned_source.installation_generation, '2');
    const emptyReset = watchDisposals(layer.sourceSlabs.map(slab => slab.geometry));
    send(second, new Float32Array()); emptyReset.assertOnce(); assert.equal(layer.pointCount, 0); assert.equal(layer.pages.length, 0);
    assert.equal(viewer._pointCloudSourcePoints, 0); assert.equal(viewer._allPositions.length, 0);
    send(second, points(13)); assert.equal(layer.pointCount, 13);
  });
  await check('viewer full style pairs captured display identity and visibility fences dense responses', async () => {
    viewer._scene = new THREE.Scene(); viewer.setPointStyle('full');
    const dense = viewer._denseCloud, first = viewerIo.calls.at(-1);
    assert.equal(dense.displaySource.source_epoch, epochOf(sourceB));
    assert.equal(new URL(first.url, 'https://robot.example').searchParams.get('source_epoch'), epochOf(sourceB));
    assert.equal(viewer._incrementalCloud.group.visible, false); assert.equal(viewer._allPositions.length, 0);
    viewer.setVisible(false); assert.equal(first.options.signal.aborted, true); assert.equal(dense._active, false);
    first.resolve(response(denseMetadata(headerFor(sourceB)))); await nextTurn(); assert.equal(dense.count, 0);
    viewer.setVisible(true); viewerIo.calls.at(-1).resolve(response(denseMetadata(headerFor(sourceB)))); await nextTurn(); assert.equal(dense.count, 2);
    const geometry = dense.group.children[0].geometry, resource = watchDisposals([geometry]);
    send(headerFor(sourceB, { stream_generation: '2', revision: 'saved:2' }), points(13));
    resource.assertOnce(); assert.equal(dense.count, 0); assert.equal(dense.displaySource.source_epoch, epochOf(sourceB));
  });
  await check('viewer disconnect, interrupted packets, reconnect and resync cannot revive stale dense work', async () => {
    const dense = viewer._denseCloud, oldSocket = socket, staleRequest = viewerIo.calls.at(-1);
    const interrupted = packets(headerFor(sourceB, { stream_generation: '2', revision: 'saved:2' }));
    socket.emit('message', interrupted[0]); const base = viewer._incrementalConnection.assembler.base;
    socket.close(); assert.equal(dense._active, false); assert.equal(staleRequest.options.signal.aborted, true);
    assert.equal(viewer._incrementalConnection.assembler.pending, null);
    staleRequest.resolve(response(denseMetadata(headerFor(sourceB)))); await nextTurn(); assert.equal(dense.count, 0);
    const disconnectedRequests = viewerIo.calls.length;
    viewer.setVisible(false); viewer.setVisible(true); viewer.setPointStyle('height'); viewer.setPointStyle('full');
    assert.equal(dense._active, false); assert.equal(viewerIo.calls.length, disconnectedRequests, 'Visibility/style must not restart a disconnected owner');
    runTimer(viewer._incrementalConnection.timer); socket = viewer._incrementalConnection.socket; socket.emit('open');
    assert.deepEqual(socket.sent[0].base, base);
    oldSocket.emit('message', interrupted[1]); assert.equal(viewer._incrementalConnection.assembler.pending, null);
    socket.emit('message', JSON.stringify({ type: 'pointcloud.ready', version: 2 })); assert.equal(dense._active, true);
    assert.equal(new URL(viewerIo.calls.at(-1).url, 'https://robot.example').searchParams.get('source_epoch'), epochOf(sourceB));
    socket.emit('message', new ArrayBuffer(1)); assert.equal(dense._active, false); assert.equal(viewerIo.calls.at(-1).options.signal.aborted, true);
    assert.equal(socket.sent.at(-1).type, 'pointcloud.resync');
    const resyncRequests = viewerIo.calls.length;
    viewer.setVisible(false); viewer.setVisible(true); viewer.setPointStyle('height'); viewer.setPointStyle('full');
    assert.equal(dense._active, false); assert.equal(viewerIo.calls.length, resyncRequests, 'Visibility/style must not restart while awaiting resync');
    send(headerFor(sourceB, { stream_generation: '2', revision: 'saved:2' }), points(13)); assert.equal(dense._active, true);
    assert.equal(dense.displaySource.source_epoch, epochOf(sourceB));
  });
  await check('clear closes the old connection and ignores queued pages until a new map is selected', async () => {
    const dense = viewer._denseCloud, stale = viewerIo.calls.at(-1), oldSocket = socket;
    viewer.setMapContext(null);
    assert.equal(stale.options.signal.aborted, true); assert.equal(dense.displaySource, null);
    assert.equal(viewer._incrementalConnection.socket, null); assert.equal(viewer._incrementalConnection.timer, null);
    const messages = oldSocket.sent.length;
    for (const packet of packets(headerFor(sourceB), points(13))) oldSocket.emit('message', packet);
    oldSocket.close(); assert.equal(oldSocket.sent.length, messages);
    viewer.setVisible(false); viewer.setVisible(true); assert.equal(dense._active, false);
    assert.equal(viewer._pointCloudMapWatermark, null);
    stale.resolve(response(denseMetadata(headerFor(sourceB)))); await nextTurn(); assert.equal(dense.count, 0);
    viewer.setMapContext(mapFor(sourceB)); send(headerFor(sourceB, {stream_generation: '2'}), points(13));
    assert.notEqual(socket, oldSocket); assert.equal(viewer._incrementalActive, true); assert.equal(dense._active, true);
  });
  await check('explicit legacy saved remains separate and later assigned pages reclaim ownership', () => {
    viewer.setPointStyle('height');
    viewer.setMapContext({ source_epoch: 'legacy-map:3' });
    const legacy = headerFor(sourceB, { assigned_source: undefined, source_epoch: 'saved-legacy', mapping_source_epoch: 'legacy-map:3', stream_generation: '3' });
    send(legacy, new Float32Array([1, 2, 3]));
    assert.equal(viewer._incrementalActive, false); assert.deepEqual([...viewer._allPositions], [1, 2, 3]);
    assert.ok(diagnostics.some(state => state.pointcloudDecision === 'saved_snapshot'));
    viewer.setMapContext(mapFor(sourceB));
    send(headerFor(sourceB, { stream_generation: '4' }), points(17));
    assert.equal(viewer._incrementalActive, true); assert.equal(viewer._allPositions.length, 0); assert.equal(viewer._pointsMesh?.visible, false);
    assert.equal(viewer._pointCloudMapWatermark.header.source_epoch, epochOf(sourceB));
  });
  const realFixture = process.env.NEH_POINTCLOUD_FIXTURE_DIR
    ? path.join(process.env.NEH_POINTCLOUD_FIXTURE_DIR, 'assigned_saved_wire.json') : null;
  if (realFixture && fs.existsSync(realFixture)) {
    await check('optional installed-checkpoint wire sample pairs real Python packets with actual JS viewer resources', () => {
      // Bound optional qualification input independently of production's unchanged 32 MiB capacity.
      assert.ok(fs.statSync(realFixture).size <= 16 * 1024 * 1024, 'Real-wire fixture must be a bounded qualification sample');
      const fixture = JSON.parse(fs.readFileSync(realFixture, 'utf8'));
      assert.ok(Number.isSafeInteger(fixture.point_count) && fixture.point_count > 0 && fixture.point_count <= 100000);
      assert.ok(Array.isArray(fixture.expected_xyz) && fixture.expected_xyz.length === fixture.point_count * 3);
      assert.ok(Array.isArray(fixture.packets) && fixture.packets.length > 0 && fixture.packets.length <= 25);
      assert.equal(protocol.mappingEpochFromMap(fixture.map_snapshot), fixture.source_epoch);
      const wire = fixture.packets.map(encoded => {
        assert.equal(typeof encoded, 'string'); assert.ok(encoded.length <= 176 * 1024);
        const buffer = Buffer.from(encoded, 'base64'); return buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
      });
      const assembler = new PointCloudPageAssembler(); assembler.setMapEpoch(fixture.source_epoch);
      let batch;
      for (let index = 0; index < wire.length; index++) {
        batch = assembler.push(wire[index]);
        if (index < wire.length - 1) { assert.equal(batch, null); assert.equal(assembler.base, null); }
      }
      assert.ok(batch); assembler.accept(batch); assert.equal(assembler.pointCount, fixture.point_count);
      assert.equal(assembler.base.source_epoch, fixture.source_epoch); assert.equal(batch.header.source_incremental, false);
      const expected = Float32Array.from(fixture.expected_xyz);
      for (const chunk of batch.chunks) assert.deepEqual(chunk.xyz, expected.subarray(chunk.chunk_index * PAGE * 3, chunk.chunk_index * PAGE * 3 + chunk.xyz.length));
      viewer.setMapContext(fixture.map_snapshot); activeSocket();
      viewer._incrementalConnection.assembler.beginConnection();
      for (const packet of wire) socket.emit('message', packet);
      assert.equal(viewer._pointCloudMapWatermark.header.source_epoch, fixture.source_epoch);
      assert.equal(viewer._incrementalActive, true); assert.equal(viewer._allPositions.length, 0);
      assert.equal(viewer._incrementalCloud.pointCount, fixture.point_count);
      viewer._incrementalCloud.pages.forEach((page, index) => {
        assert.deepEqual(page.array.subarray(0, page.count * 3), expected.subarray(index * PAGE * 3, (index * PAGE + page.count) * 3));
      });
      assert.equal(socket.sent.at(-1).type, 'pointcloud.ack'); assert.equal(socket.sent.at(-1).base.source_epoch, fixture.source_epoch);
      realWireChecked = true;
    });
  }
  await check('map invalidation and viewer dispose clear ownership, abort fetch and dispose actual Three resources', async () => {
    viewer.setPointStyle('full'); const dense = viewer._denseCloud, pending = viewerIo.calls.at(-1);
    viewer.setMapContext({ source_epoch: 'mapping:2' });
    assert.equal(pending.options.signal.aborted, true); assert.equal(dense.displaySource, null); assert.equal(dense._active, false);
    assert.equal(viewer._pointCloudMapWatermark, null); assert.equal(viewer._incrementalActive, false); assert.equal(viewer._allPositions.length, 0);
    const layer = viewer._incrementalCloud;
    const resources = watchDisposals([...layer.sourceSlabs.map(slab => slab.geometry), ...layer.derivedPages.map(page => page.geometry), layer.material, layer.derivedMaterial, dense.material]);
    viewer.dispose(); resources.assertOnce(); assert.equal(viewer._incrementalConnection, null); assert.equal(viewer._incrementalCloud, null);
    assert.equal(viewer._denseCloud, null); assert.equal(socket.readyState, 3); assert.equal(layer._disposed, true);
    for (const call of viewerIo.calls) call.resolve(response(denseMetadata(headerFor(sourceB))));
    await nextTurn();
    assert.equal(timers.size, 0, 'All response bodies must settle or abort and all timers must be released');
  });
} finally {
  viewer?.dispose();
  for (const layer of layers) layer.dispose();
  for (const [key, value] of Object.entries(original)) { if (value === undefined) delete globalThis[key]; else globalThis[key] = value; }
  fs.rmSync(temporary, { recursive: true, force: true });
}
console.log(JSON.stringify({ checks, failures: failures.length, fixtures: realWireChecked ? 'JavaScript-generated plus supplied installed-checkpoint wire sample' : 'generated entirely in JavaScript', scope: 'CPU module integration; no browser or WebGL validation' }));
if (failures.length) process.exitCode = 1;
