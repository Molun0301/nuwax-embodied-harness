#!/usr/bin/env node
// Actual bundled Three CPU objects and stub Canvas2D. This does not exercise WebGL.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath, pathToFileURL } from 'node:url';
const repo = path.resolve(process.argv[2] || path.join(path.dirname(fileURLToPath(import.meta.url)), '..'));
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-semantic-overlay-'));
for (const name of ['pointcloud3d', 'pointcloud-incremental', 'pointcloud-pages', 'three.module.min',
  'semantic-regions3d', 'scene-graph3d', 'OrbitControls', 'pointcloud-processing', 'dense-pointcloud']) {
  fs.copyFileSync(path.join(repo, 'gateways/http/static/js', `${name}.js`), path.join(temp, `${name}.js`));
}
fs.writeFileSync(path.join(temp, 'package.json'), '{"type":"module"}');
class Socket extends EventTarget {
  static OPEN = 1;
  constructor() { super(); this.readyState = 0; }
  emit(type, data) {
    if (type === 'open') this.readyState = 1;
    if (type === 'close') this.readyState = 3;
    const event = new Event(type);
    if (data !== undefined) Object.defineProperty(event, 'data', { value: data });
    this.dispatchEvent(event);
  }
  close() { this.emit('close'); }
  send() {}
}
globalThis.WebSocket = Socket;
globalThis.window = new EventTarget();
Object.assign(window, { location: { href: 'https://robot.example/' }, setTimeout, clearTimeout });
globalThis.CustomEvent = class extends Event {
  constructor(name, options) { super(name); this.detail = options?.detail; }
};
let canvases = 0, missingContext = false;
globalThis.document = {
  getElementById: () => null,
  createElement(name) {
    assert.equal(name, 'canvas'); canvases++;
    const canvas = { width: 300, height: 150 };
    const context = { measureText: text => ({ width: text.length * 12 }), beginPath() {}, moveTo() {},
      lineTo() {}, quadraticCurveTo() {}, closePath() {}, fill() { canvas.background = this.fillStyle; },
      stroke() { canvas.border = this.strokeStyle; }, fillText(text) { canvas.text = text; } };
    canvas.getContext = () => missingContext ? null : context;
    return canvas;
  },
};
let viewer;
try {
  const THREE = await import(pathToFileURL(path.join(temp, 'three.module.min.js')));
  await import(pathToFileURL(path.join(temp, 'pointcloud3d.js')));
  viewer = window.pointcloud3d;
  let renders = 0;
  viewer._requestRender = () => { renders++; };
  function setup() {
    viewer._scene = new THREE.Scene(); viewer._semanticGroup = new THREE.Group();
    viewer._scene.add(viewer._semanticGroup);
    viewer._semanticMarkerGeometry = new THREE.BoxGeometry(1, 1, 1);
    viewer._semanticPreciseMaterial = new THREE.MeshBasicMaterial({ color: 0x8aa7b8, transparent: true, opacity: .12, side: THREE.DoubleSide, depthWrite: false });
    viewer._semanticFallbackMaterial = new THREE.MeshBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: .42, wireframe: true, depthWrite: false });
    viewer._semanticSelectedMaterial = viewer._semanticPreciseMaterial.clone();
    viewer._semanticSelectedMaterial.opacity = .26;
    viewer._semanticSelectedFallbackMaterial = viewer._semanticFallbackMaterial.clone();
    viewer._semanticSelectedFallbackMaterial.opacity = .72;
  }
  setup();
  const owned = () => viewer._semanticGroup.children.flatMap(object => object.isLine
    ? [object.geometry, object.material] : object.isSprite ? [object.material.map, object.material] : []);
  const counts = new Map();
  function watch() {
    for (const resource of [...owned(), viewer._semanticMarkerGeometry,
      viewer._semanticPreciseMaterial, viewer._semanticFallbackMaterial, viewer._semanticSelectedMaterial, viewer._semanticSelectedFallbackMaterial]) if (resource && !counts.has(resource)) {
      counts.set(resource, 0); resource.addEventListener('dispose', () => counts.set(resource, counts.get(resource) + 1));
    }
  }
  const source = fs.readFileSync(path.join(temp, 'three.module.min.js'), 'utf8');
  const comparators = Object.fromEntries(['Al', 'El'].map(name => {
    const body = source.match(new RegExp(`function ${name}\\(t,e\\)\\{[^}]+\\}`))?.[0];
    assert.ok(body, `bundled Three comparator ${name}`);
    return [name, new Function(`${body}; return ${name};`)()];
  }));
  function assertOrder() {
    const children = viewer._semanticGroup.children;
    assert.equal(new Set(children).size, children.length);
    for (const object of children) assert.equal(object.parent, viewer._semanticGroup);
    for (const transparent of [false, true]) {
      const entries = children.filter(o => o.material.transparent === transparent).map(object => ({
        id: object.id, material: object.material, object, groupOrder: 0, renderOrder: object.renderOrder, z: 0,
      }));
      const expected = entries.slice().sort((a, b) => a.renderOrder - b.renderOrder
        || (transparent ? 0 : a.material.id - b.material.id)).map(e => e.object);
      assert.deepEqual(entries.sort(transparent ? comparators.Al : comparators.El).map(e => e.object), expected,
        'actual Three tie order matches a rebuild in logical input order');
    }
    const expected = viewer._semanticRecords.flatMap(record => [record.marker, record.stem, record.label].filter(Boolean));
    assert.deepEqual(children, expected, 'flat marker/stem/label order');
  }
  function update(data) {
    for (const item of data) if (item?.object_extent?.syntheticCentered) {
      item.object_extent.center = { ...item.pose };
      item.object_extent.top_z_m = item.pose.z + item.object_extent.size_m.z / 2;
    }
    viewer.updateSemanticInstances(data); watch(); assertOrder();
    assert.ok(viewer._semanticRecords.every(record => record.stem === null), 'no invented z=0 floor stems');
  }
  const instance = (id, overrides = {}) => ({ instance_id: id, frame_id: 'map', pose: { x: 1, y: 2, z: 1 },
    display_name: `Item ${id}`, object_extent: { syntheticCentered: true, frame_id: 'map',
      size_m: { x: 1, y: 2, z: 3 }, orientation: { x: 0, y: 0, z: 0, w: 1 }, source: 'detection3d' }, ...overrides });
  let data = [instance('a'), instance('b'), instance('c')];
  update(data);
  const record = id => viewer._semanticRecords.find(r => r.id === id);
  const initial = owned(), wrappers = viewer._semanticGroup.children.slice();
  const renderCount = renders, canvasCount = canvases;
  data = data.map(item => ({ ...item, pose: { ...item.pose }, confidence: .8, observations: 8 }));
  update(data);
  assert.deepEqual(owned(), initial); assert.deepEqual(viewer._semanticGroup.children, wrappers);
  assert.equal(renders, renderCount); assert.equal(canvases, canvasCount);
  for (const item of data) {
    assert.equal(record(item.instance_id).marker.userData.semanticInstance, item);
    assert.equal(record(item.instance_id).label.userData.semanticInstance, item);
  }
  // Mutable inputs cannot hide pose, extent, orientation, label or identity changes.
  const bResources = [record('b').marker.geometry, record('b').label.material];
  const a = record('a');
  data[0].pose.x = 9; update(data);
  assert.equal(record('a'), a); assert.deepEqual(owned(), initial); assert.equal(canvases, canvasCount);
  assert.equal(a.marker.position.x, 9); assert.equal(a.label.position.x, 9);
  data[0].pose.z = -1.001; update(data); assert.equal(canvases, canvasCount, 'position-only updates reuse label');
  assert.equal(a.marker.position.z, -1.001); assert.equal(a.label.position.z, -1.001 + 1.5 + .24);
  const oldLabelMaterial = a.label.material, oldLabelTexture = oldLabelMaterial.map, stableSprite = a.label;
  data[0].object_extent.size_m.x = 2; update(data);
  assert.equal(a.marker.scale.x, 2); assert.equal(a.label, stableSprite); assert.equal(canvases, canvasCount, 'extent-only changes reuse the name label');
  assert.equal(counts.get(oldLabelMaterial), 0); assert.equal(counts.get(oldLabelTexture), 0);
  assert.equal(a.label.material.map.image.text, 'Item a');
  data[0].display_name = '  Long name abcdefghijk '; update(data);
  assert.equal(a.label, stableSprite);
  assert.equal(a.label.material.map.image.text, 'Long name a…');
  assert.equal(counts.get(oldLabelMaterial), 1); assert.equal(counts.get(oldLabelTexture), 1);
  const longLabel = a.label.material;
  data[0].display_name = 'Long name abcZZZZZZZZ'; update(data);
  assert.equal(a.label.material, longLabel, 'clipped-equivalent name does not redraw');
  data[0].is_fallback = true; update(data);
  assert.equal(a.marker.material, viewer._semanticFallbackMaterial);
  assert.equal(a.label.material.map.image.background, 'rgba(51,65,85,.5)');
  data[0].is_fallback = false; data[0].frame_id = 'odom'; update(data);
  assert.equal(a.marker.material, viewer._semanticFallbackMaterial);
  assert.equal(a.label.material.map.image.text, 'Long name a…');
  data[0].frame_id = 'map'; update(data);
  assert.equal(a.marker.material, viewer._semanticPreciseMaterial);
  assert.deepEqual([record('b').marker.geometry, record('b').label.material], bResources);
  // Reorder and insertion need wrapper tie repair, never owned-resource replacement.
  const beforeReorder = new Set(owned());
  data.reverse(); update(data); assert.deepEqual(new Set(owned()), beforeReorder);
  data.unshift(instance('new')); update(data);
  assert.equal(owned().filter(r => beforeReorder.has(r)).length, beforeReorder.size);
  const dropped = record('new'); data.shift(); update(data);
  for (const resource of [dropped.label.material, dropped.label.material.map]) assert.equal(counts.get(resource), 1);
  // Metadata is never serialized, even when wrapper tie order needs repairing.
  data[0].opaque = { bigint: 1n }; data[0].opaque.self = data[0].opaque;
  data.reverse(); update(data);
  for (const item of data) assert.equal(record(item.instance_id).label.userData.semanticInstance, item);
  const changedId = data[0], previousId = changedId.instance_id, oldRecord = record(previousId);
  changedId.instance_id = 'renamed-id'; update(data);
  assert.equal(record(previousId), undefined); assert.notEqual(record('renamed-id'), oldRecord);
  assert.equal(counts.get(oldRecord.label.material), 1);
  // Duplicate, empty, and absent IDs preserve each occurrence and selection object.
  data = [instance(''), instance(''), instance('same'), instance('same'), instance(undefined), instance(undefined)];
  update(data); const duplicateResources = owned();
  data = data.map((item, index) => ({ ...item, confidence: index })); update(data);
  assert.equal(viewer._semanticRecords.length, 6); assert.deepEqual(owned(), duplicateResources);
  viewer._semanticRecords.forEach((entry, index) => {
    assert.equal(entry.marker.userData.semanticInstance, data[index]); assert.equal(entry.label.userData.semanticInstance, data[index]);
  });
  data.splice(0, 1); update(data); assert.equal(viewer._semanticRecords.length, 5);
  const invalid = [null, {}, instance('null', { pose: { x: null, y: 1, z: 1 } }),
    instance('nan', { pose: { x: NaN, y: 1, z: 1 } }), instance('inf', { pose: { x: 1, y: Infinity, z: 1 } }),
    instance('missing', { pose: { x: 1, y: 1 } }), instance('observation', { observation_only: true }),
    instance('projection', { projection_method: 'current_pose_fallback' })];
  update([...data, ...invalid]); assert.equal(viewer._semanticRecords.length, 5);
  data[0].observation_only = true; update(data); assert.equal(viewer._semanticRecords.length, 4);
  data[0].observation_only = false; update(data); assert.equal(viewer._semanticRecords.length, 5);
  // Same-length fast-path rejection after earlier selection references refreshed.
  data = [instance('early'), instance('late')]; update(data);
  const earlyResources = [record('early').marker.geometry, record('early').label.material];
  data = data.map(item => ({ ...item, pose: { ...item.pose } }));
  data[1].display_name = 'Late rename'; update(data);
  assert.equal(record('early').marker.userData.semanticInstance, data[0]);
  assert.equal(record('late').label.userData.semanticInstance, data[1]);
  assert.deepEqual([record('early').marker.geometry, record('early').label.material], earlyResources);
  data[1].pose.x = null; update(data); assert.equal(record('late'), undefined);
  data[1].pose.x = 1; update(data); assert.ok(record('late'));
  let mutableName = 'Original name';
  data[1].display_name = { toString: () => mutableName }; update(data);
  mutableName = 'Changed name'; update(data);
  assert.equal(record('late').label.material.map.image.text, 'Changed name');
  const extent = data[1].object_extent;
  extent.size_m = { x: 4, y: .5, z: 2 };
  extent.orientation = { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 };
  data[1].pose.z = -2; update(data);
  const rotated = record('late').marker;
  viewer._scene.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(rotated);
  assert.ok(Math.abs(box.max.x - box.min.x - .5) < 1e-12);
  assert.ok(Math.abs(box.max.y - box.min.y - 4) < 1e-12);
  assert.equal(box.min.z, -3); assert.equal(box.max.z, -1);
  assert.equal(record('late').label.position.z, -.76);
  assert.equal(rotated.geometry, viewer._semanticMarkerGeometry, 'all sizes reuse one unit geometry');
  const stableGeometry = rotated.geometry, stableMaterial = rotated.material;
  const geometryUpdateCanvases = canvases, geometryUpdateLabel = record('late').label.material;
  extent.size_m.x = 4.004; update(data);
  assert.equal(record('late').marker.geometry, stableGeometry);
  assert.equal(record('late').marker.material, stableMaterial);
  assert.equal(record('late').label.material, geometryUpdateLabel);
  assert.equal(canvases, geometryUpdateCanvases, 'sub-label-precision geometry updates allocate no canvas');
  const fullBox = extent.size_m;
  extent.source = 'observed_points'; extent.size_m = { x: 4, y: 0, z: 2 }; update(data);
  assert.equal(record('late').marker.scale.y, 0, 'no invented surface thickness');
  assert.equal(record('late').label.material.map.image.text, 'Changed name');
  data[1].object_extent = null; update(data);
  assert.equal(record('late').marker.material, viewer._semanticFallbackMaterial);
  assert.deepEqual(record('late').marker.scale.toArray(), [.16, .16, .16]);
  assert.equal(record('late').label.material.map.image.text, 'Changed name');
  data[1].object_extent = extent; extent.source = 'detection3d'; extent.size_m = fullBox; update(data);
  const selectionCanvases = canvases, selectionResources = owned();
  const selectedBox = record('late').marker.scale.toArray();
  viewer.setSemanticSelectedId('late');
  assert.equal(record('late').marker.material, viewer._semanticSelectedMaterial);
  assert.equal(record('late').label.material.opacity, 1);
  assert.equal(record('early').label.material.opacity, .9);
  assert.deepEqual(record('late').marker.scale.toArray(), selectedBox, 'selection preserves measured geometry');
  viewer.setSemanticSelectedId('late'); update(data.map(item => ({ ...item })));
  assert.equal(record('late').marker.material, viewer._semanticSelectedMaterial);
  assert.equal(canvases, selectionCanvases); assert.deepEqual(owned(), selectionResources);
  data[1].object_extent = null; update(data);
  assert.equal(record('late').marker.material, viewer._semanticSelectedFallbackMaterial);
  data[1].object_extent = extent; update(data);
  viewer.setSemanticSelectedId('early');
  assert.equal(record('late').marker.material, viewer._semanticPreciseMaterial);
  viewer.setSemanticSelectedId(null);
  assert.equal(record('early').marker.material, viewer._semanticPreciseMaterial);
  assert.equal(record('late').label.material.opacity, .9);
  const beforeVisibility = owned(), beforeVisibilityWrappers = viewer._semanticGroup.children.slice();
  viewer.setSemanticInstancesVisible(false); assert.equal(viewer._semanticGroup.visible, false);
  update(data.map(item => ({ ...item })));
  viewer.setSemanticInstancesVisible(true); assert.equal(viewer._semanticGroup.visible, true);
  viewer._zoomPercent = () => 10; viewer._syncSemanticGroupVisibility(); assert.equal(viewer._semanticGroup.visible, false);
  viewer._zoomPercent = () => 80; viewer._syncSemanticGroupVisibility(); assert.equal(viewer._semanticGroup.visible, true);
  viewer.setPointStyle('mono'); viewer._rebuildSemanticInstances();
  assert.deepEqual(owned(), beforeVisibility); assert.deepEqual(viewer._semanticGroup.children, beforeVisibilityWrappers);
  const connection = viewer._incrementalConnection;
  connection.socket.emit('open');
  connection.socket.emit('message', JSON.stringify({ type: 'pointcloud.ready', version: 2 }));
  connection.resync(); connection.socket.close();
  window.clearTimeout(connection.timer); connection.timer = null; connection.connect();
  connection.socket.emit('open');
  connection.socket.emit('message', JSON.stringify({ type: 'pointcloud.ready', version: 2 }));
  update(data.map(item => ({ ...item })));
  assert.deepEqual(owned(), beforeVisibility, 'resync/disconnect/reconnect keeps semantic resources');
  assert.deepEqual(viewer._semanticGroup.children, beforeVisibilityWrappers);
  // Actual Three raycast and clustering use the latest objects after metadata refresh.
  data = [instance('picked', { pose: { x: 0, y: 0, z: 1 } })]; update(data);
  const fresh = { ...data[0], confidence: .123 }; update([fresh]);
  viewer._camera = new THREE.PerspectiveCamera(60, 1, .1, 100); viewer._camera.position.set(0, 0, 10); viewer._camera.lookAt(0, 0, 0);
  viewer._renderer = { domElement: { getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 800 }) } };
  assert.equal(viewer._semanticInstancesAtClientPoint(400, 400)[0], fresh);
  viewer._renderer = null;
  update([]); update([]); assert.equal(viewer._semanticGroup.children.length, 0);
  const shared = [viewer._semanticMarkerGeometry, viewer._semanticPreciseMaterial, viewer._semanticFallbackMaterial, viewer._semanticSelectedMaterial, viewer._semanticSelectedFallbackMaterial];
  for (const [resource, count] of counts) assert.equal(count, shared.includes(resource) ? 0 : 1);
  missingContext = true; update([instance('no-canvas')]); assert.equal(viewer._semanticRecords[0].label, null);
  const failedCanvasCount = canvases; update([instance('no-canvas')]);
  assert.equal(canvases, failedCanvasCount, 'unchanged input does not retry unavailable canvas');
  missingContext = false; update([instance('no-canvas', { pose: { x: 2, y: 2, z: 1 } })]);
  assert.ok(viewer._semanticRecords[0].label, 'a visual move retries an unavailable canvas context');
  viewer.dispose(); viewer.dispose();
  for (const count of counts.values()) assert.equal(count, 1, 'owned and shared resources disposed exactly once');
  // Reinitialization recreates owned resources from preserved instance input.
  setup(); viewer._rebuildSemanticInstances(); watch(); assert.equal(viewer._semanticRecords.length, 1);
  assert.equal(viewer._semanticRecords[0].marker.userData.semanticInstance, viewer._semanticInstances[0]);
  viewer.dispose(); viewer.dispose();
  for (const count of counts.values()) assert.equal(count, 1);
  console.log(`PASS semantic overlay: identity/metadata, pose/bounds, rendered label equivalence, fallback, thresholds, insert/remove/reorder, duplicate/missing IDs, invalid/excluded poses, opaque metadata, actual Three sort/raycast, visibility/style/cache/reconnect, clear/reinitialize/disposal (${counts.size} resources; Three ${THREE.REVISION}; CPU only, no WebGL)`);
} finally { viewer?.dispose(); fs.rmSync(temp, { recursive: true, force: true }); }
