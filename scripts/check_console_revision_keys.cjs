#!/usr/bin/env node
/* 离线核对生产控制台的修订键与重绘路径；不连接浏览器、网络或机器人。 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../gateways/http/static/js/console.js'), 'utf8');

function productionFunction(name) {
  const marker = `      function ${name}(`;
  const start = source.indexOf(marker);
  assert.ok(start >= 0, `缺少生产函数 ${name}`);
  const end = source.indexOf('\n      function ', start + marker.length);
  assert.ok(end > start, `缺少生产函数边界 ${name}`);
  return source.slice(start, end);
}

function harness() {
  const health = [];
  const state = {mapLegacyFrameSequence: 0, mapGridContentKey: '', mapBaseCache: null,
    mapDrawFrame: null, mapViewMode: '2d', map2dCameraMode: 'follow',
    receivedMapIdentity: {key: 'source'}, lastRobotStateChangeKey: null};
  const context = vm.createContext({state, JSON, Int8Array,
    els: {mapStatus: {}, mapEmpty: {}},
    window: {requestAnimationFrame: () => 1, dispatchEvent() {}},
    CustomEvent: class {constructor(type, options) {this.type = type; this.detail = options.detail;}},
    planarYawFromPose: pose => pose?.yaw || 0,
    acceptAppliedMapIdentity: () => true,
    statusOf: envelope => envelope.status,
    isMapClearedEnvelope: () => false,
    shouldDisplayMapFrame: () => ({display: true}),
    shouldPinLocalizationCheckpointDisplay: () => false,
    mapSnapshotIsOffline: () => false,
    setPill() {}, recordMediaFrame() {}, renderMapSummary() {}, updateMapStreamChip() {},
    videoSourceTime: () => null, releasePendingMapPose() {}, resetFreeMapView() {},
    updateHealth: value => health.push(value),
  });
  for (const name of ['gridResolution', 'originPosition', 'originYaw', 'requestMapDraw',
    'mapGridContentKey', 'updateMap', 'robotStateChangeKey', 'updateRobotStateFrame']) {
    vm.runInContext(productionFunction(name), context);
  }
  return {context, state, health};
}

function mapEnvelope({epoch = 'run-1', revision = 1, mapName = '中文地图', cells = [0, 100], sourceName = 'mapping'} = {}) {
  return {status: 'available', source: 'map-service', revision: String(revision), data: {
    map_name: mapName, source_name: sourceName, source_epoch: epoch, update_sequence: revision,
    occupancy_grid: {map_id: mapName, frame_id: 'map', width: 2, height: 1, resolution_m: 0.1,
      origin: {x: 0, y: 0}, data: cells}, metadata: {},
  }};
}

let checks = 0;
function check(name, run) {run(); checks++; console.log(`PASS ${name}`);}
const key = (ctx, envelope) => ctx.mapGridContentKey(envelope.data.occupancy_grid, envelope, envelope.data.metadata);

check('same owner epoch and revision reuse the rendered base', () => {
  const {context, state} = harness();
  context.updateMap(mapEnvelope());
  const cached = state.mapBaseCache = {canvas: 'existing'};
  context.updateMap(mapEnvelope());
  assert.equal(state.mapBaseCache, cached);
});
check('changed revision redraws even for equal size and geometry', () => {
  const {context, state} = harness();
  context.updateMap(mapEnvelope());
  state.mapBaseCache = {canvas: 'old'};
  context.updateMap(mapEnvelope({revision: 2, cells: [100, 0]}));
  assert.equal(state.mapBaseCache, null);
  assert.equal(state.mapGrid.data[0], 100);
});
check('same revision in another epoch, owner or map cannot alias', () => {
  const {context} = harness();
  const original = key(context, mapEnvelope());
  for (const changes of [{epoch: 'run-2'}, {sourceName: 'other'}, {mapName: '另一张图'}]) {
    assert.notEqual(key(context, mapEnvelope(changes)), original);
  }
});
check('zero is a real revision and metadata-only envelope changes retain occupancy revision', () => {
  const {context} = harness();
  const first = mapEnvelope({revision: 0});
  first.data.metadata.occupancy_content_revision = 0;
  const later = mapEnvelope({revision: 3});
  later.data.metadata.occupancy_content_revision = 0;
  assert.equal(key(context, first), key(context, later));
});
check('geometry and frame changes invalidate the base', () => {
  const {context} = harness();
  const original = key(context, mapEnvelope());
  for (const changes of [{frame_id: 'other-map'}, {width: 1}, {resolution_m: 0.2}, {origin: {x: 3, y: 0}}]) {
    const envelope = mapEnvelope();
    Object.assign(envelope.data.occupancy_grid, changes);
    assert.notEqual(key(context, envelope), original);
  }
});
check('legacy unversioned updates always redraw, including same-array mutation', () => {
  const {context, state} = harness();
  const envelope = mapEnvelope();
  delete envelope.revision;
  delete envelope.data.update_sequence;
  context.updateMap(envelope);
  const first = state.mapGridContentKey;
  state.mapBaseCache = {canvas: 'old'};
  envelope.data.occupancy_grid.data[0] = 100;
  context.updateMap(envelope);
  assert.notEqual(state.mapGridContentKey, first);
  assert.equal(state.mapBaseCache, null);
});
check('revision without an owner epoch cannot silently freeze after a restart', () => {
  const {context} = harness();
  const envelope = mapEnvelope();
  delete envelope.data.source_epoch;
  assert.notEqual(key(context, envelope), key(context, envelope));
});
check('nested map snapshots retain the same authoritative identity', () => {
  const {context} = harness();
  const envelope = mapEnvelope();
  const original = key(context, envelope);
  const nested = {...envelope, data: {map_snapshot: envelope.data}};
  assert.equal(context.mapGridContentKey(envelope.data.occupancy_grid, nested, {}), original);
});
check('revision key construction never reads occupancy cells', () => {
  const {context} = harness();
  const measurements = [];
  for (const count of [1, 250000, 1000000]) {
    let cellReads = 0;
    const cells = new Proxy(new Array(count), {get(target, property) {
      if (/^\d+$/.test(String(property))) {cellReads++; throw new Error('修订键读取了栅格内容');}
      return Reflect.get(target, property);
    }});
    const envelope = mapEnvelope({cells});
    for (let revision = 0; revision < 1000; revision++) {
      envelope.data.update_sequence = revision;
      key(context, envelope);
    }
    assert.equal(cellReads, 0);
    measurements.push({cells: count, key_updates: 1000, cell_reads: cellReads});
  }
  console.log(JSON.stringify({diagnostic: 'synthetic_revision_key_work', measurements}));
});

function robotFrame(epoch = 'robot-run-1', revision = 1) {
  return {source: 'robot_state_service', revision: `robot-state:${epoch}:${revision}`, payload: {
    robot_name: 'robot', robot_model: 'synthetic', robot_state: {mode: 'idle'}, assembly_status: {},
  }, metadata: {source_epoch: epoch, revision, robot_name: 'robot'}};
}
check('robot state repeats are skipped only within the same owner epoch and revision', () => {
  const {context, health} = harness();
  assert.equal(context.updateRobotStateFrame(robotFrame()), true);
  assert.equal(context.updateRobotStateFrame(robotFrame()), false);
  assert.equal(context.updateRobotStateFrame(robotFrame('robot-run-1', 2)), true);
  assert.equal(health.length, 2);
});
check('robot service restart with revision reset updates the UI', () => {
  const {context, health} = harness();
  context.updateRobotStateFrame(robotFrame('robot-run-1', 1));
  assert.equal(context.updateRobotStateFrame(robotFrame('robot-run-2', 1)), true);
  assert.equal(health.length, 2);
});
check('old frontend revision fallback handles new backend restart without compatibility digest fields', () => {
  // 旧控制台在摘要字段缺省时直接使用信封 revision，不会解析其数字格式。
  const legacyKey = frame => frame.metadata.content_hash || frame.metadata.signature || frame.revision || null;
  assert.equal(legacyKey(robotFrame()), legacyKey(robotFrame()));
  assert.notEqual(legacyKey(robotFrame('robot-run-1', 1)), legacyKey(robotFrame('robot-run-2', 1)));
});
check('legacy robot messages without epoch or revision are never frozen', () => {
  const {context, health} = harness();
  const noEpoch = robotFrame();
  delete noEpoch.metadata.source_epoch;
  assert.equal(context.updateRobotStateFrame(noEpoch), true);
  assert.equal(context.updateRobotStateFrame(noEpoch), true);
  const noRevision = robotFrame();
  delete noRevision.revision;
  delete noRevision.metadata.revision;
  assert.equal(context.updateRobotStateFrame(noRevision), true);
  assert.equal(context.updateRobotStateFrame(noRevision), true);
  assert.equal(health.length, 4);
});
check('robot identity participates in the revision key and invalid payloads remain ignored', () => {
  const {context} = harness();
  const original = robotFrame();
  const other = robotFrame();
  other.metadata.robot_name = 'other-robot';
  assert.notEqual(context.robotStateChangeKey(original), context.robotStateChangeKey(other));
  assert.equal(context.updateRobotStateFrame({...original, payload: []}), false);
});
console.log(JSON.stringify({checks, status: 'passed', scope: 'offline production functions; no browser rendering or device claims'}));
