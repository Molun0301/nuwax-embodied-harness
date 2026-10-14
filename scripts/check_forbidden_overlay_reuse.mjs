import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath, pathToFileURL } from 'node:url';
const repo = path.resolve(process.argv[2] || path.join(path.dirname(fileURLToPath(import.meta.url)), '..'));
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-forbidden-overlay-'));
const modules = ['pointcloud3d.js', 'pointcloud-incremental.js', 'pointcloud-pages.js', 'three.module.min.js',
  'semantic-regions3d.js', 'scene-graph3d.js', 'OrbitControls.js', 'pointcloud-processing.js', 'dense-pointcloud.js'];
for (const name of modules) fs.copyFileSync(path.join(repo, 'gateways/http/static/js', name), path.join(temp, name));
fs.writeFileSync(path.join(temp, 'package.json'), '{"type":"module"}');
class Socket extends EventTarget { static OPEN = 1; constructor() { super(); this.readyState = 0; } close() { this.readyState = 3; } send() {} }
globalThis.WebSocket = Socket;
globalThis.window = new EventTarget();
Object.assign(window, { location: { href: 'https://generic-platform.example/' }, setTimeout, clearTimeout });
globalThis.document = { getElementById: () => null };
let viewer;
try {
  const THREE = await import(pathToFileURL(path.join(temp, 'three.module.min.js')));
  await import(pathToFileURL(path.join(temp, 'pointcloud3d.js')));
  const threeSource = fs.readFileSync(path.join(temp, 'three.module.min.js'), 'utf8');
  const comparatorSource = threeSource.match(/function Al\(t,e\)\{[^}]+\}/)?.[0];
  assert.ok(comparatorSource, 'inspect the bundled Three 170 transparent comparator');
  const transparentSort = new Function(`${comparatorSource}; return Al;`)();
  viewer = window.pointcloud3d;
  viewer._scene = new THREE.Scene();
  viewer._forbiddenZoneGroup = new THREE.Group();
  viewer._scene.add(viewer._forbiddenZoneGroup);
  viewer._hasUserCameraInput = true;
  const rectangle = (id, mode = 'inside', enabled = true) => ({ id, shape: 'rectangle', mode, enabled,
    vertices: [{ x: -2, y: -2 }, { x: 2, y: 2 }] });
  const counts = new Map();
  const watch = () => viewer._forbiddenZoneGroup.traverse(object => {
    for (const resource of [object.geometry, object.material]) if (resource && !counts.has(resource)) {
      counts.set(resource, 0);
      resource.addEventListener('dispose', () => counts.set(resource, counts.get(resource) + 1));
    }
  });
  const resources = () => viewer._forbiddenZoneGroup.children.flatMap(object => [object.geometry, object.material]);
  const record = id => viewer._forbiddenZoneResources.find(entry => entry.zone.id === id);
  const assertTieOrder = () => {
    const objects = viewer._forbiddenZoneGroup.children;
    const entries = objects.map(object => ({ id: object.id, object, groupOrder: 2, renderOrder: object.renderOrder, z: 0 }));
    const expected = entries.slice().sort((a, b) => a.renderOrder - b.renderOrder).map(entry => entry.object);
    assert.deepEqual(entries.sort(transparentSort).map(entry => entry.object), expected,
      'actual Three transparent comparator preserves baseline logical order at equal depth');
  };
  let revision = 0;
  const batch = extent => {
    const xyz = new Float32Array(1024 * 3);
    xyz.set([-extent, -extent, 0]); xyz.set([extent, extent, 0], 3);
    viewer._acceptCloudPageBatch({ header: { source: 'live', source_epoch: 'generic-source', frame_id: 'map',
      stream_generation: '1', layout_generation: '1', revision_ordinal: String(++revision), revision: `cloud:${revision}`,
      source_timestamp_ns: '1', source_incremental: true, point_count: 1024, chunk_size_points: 1024,
      changed_chunk_count: 1, reset: revision === 1 }, chunks: [{ chunk_index: 0, xyz }] });
    watch(); assertTieOrder();
  };
  const update = (generation, zones, map_name = 'map-a', allow = false) => {
    const accepted = viewer.updateForbiddenZones({ map_name, generation, zones }, allow); watch(); assertTieOrder(); return accepted;
  };
  const inside = rectangle('inside'), outside = rectangle('outside', 'outside'), disabled = rectangle('disabled', 'outside', false);
  assert.equal(update(1, [inside, outside, disabled]), true);
  batch(1);
  const stable = resources();
  for (let index = 0; index < 20; index++) batch(1);
  assert.deepEqual(resources(), stable, 'identical real page batches retain every resource');
  const keptInside = record('inside'), keptDisabled = record('disabled');
  const outsideSurface = record('outside').surface, outsideWalls = record('outside').walls.slice();
  batch(10);
  assert.equal(record('inside'), keptInside); assert.equal(record('disabled'), keptDisabled);
  assert.notEqual(record('outside').surface, outsideSurface);
  assert.deepEqual(record('outside').walls, outsideWalls);
  assert.equal(counts.get(outsideSurface.geometry), 1); assert.equal(counts.get(outsideSurface.material), 1);
  const afterBounds = resources();
  assert.equal(update(2, [inside, outside, disabled]), true);
  assert.deepEqual(resources(), afterBounds, 'new generation with identical visual values reuses objects');
  assert.equal(update(2, [rectangle('changed')]), false);
  assert.equal(update(1, [], 'map-b'), false);
  assert.deepEqual(resources(), afterBounds, 'stale generations and unapproved map switches preserve overlay');
  const edited = { ...inside, stroke_width_m: 0.7 };
  const beforeEditOutside = record('outside');
  update(3, [edited, outside, disabled]);
  assert.notEqual(record('inside'), keptInside); assert.equal(record('outside'), beforeEditOutside);
  for (const object of [keptInside.surface, ...keptInside.walls]) assert.equal(counts.get(object.geometry), 1);
  const enabledDisabled = { ...disabled, enabled: true };
  update(4, [edited, outside, enabledDisabled]);
  assert.ok(record('disabled').surface);
  update(5, [edited, outside]);
  assert.equal(record('disabled'), undefined);
  const beforeSwitch = resources();
  assert.equal(update(1, [edited, outside], 'map-b', true), true);
  for (const resource of new Set(beforeSwitch)) assert.equal(counts.get(resource), 1);
  const visibleResources = resources();
  viewer.setVisible(false); batch(10); viewer.setVisible(true);
  assert.deepEqual(resources(), visibleResources, 'hidden/reveal and page update reuse overlay');
  const circle = { id: 'circle', shape: 'circle', mode: 'outside', center: { x: 0, y: 0 }, radius_m: 2 };
  const polygon = { id: 'polygon', shape: 'polygon', vertices: [{ x: 0, y: 0 }, { x: 2, y: 0 }, { x: 1, y: 2 }] };
  const line = { id: 'line', shape: 'line', vertices: [{ x: 0, y: 0 }, { x: 2, y: 2 }] };
  update(2, [circle, polygon, line, rectangle(''), rectangle(''), rectangle('duplicate'), rectangle('duplicate')], 'map-b');
  assert.equal(viewer._forbiddenZoneResources.length, 7);
  assert.equal(record('line').surface, undefined); assert.equal(record('line').walls.length, 1);
  const shapes = resources(); batch(10); assert.deepEqual(resources(), shapes);
  // Reorder and partial edits preserve renderer tie order, while GPU resources
  // survive the explicitly permitted lightweight Mesh wrapper replacement.
  const first = rectangle('first'), second = rectangle('second');
  update(3, [first, second], 'map-b');
  const firstGeometry = record('first').walls[0].geometry, secondGeometry = record('second').walls[0].geometry;
  update(4, [second, first], 'map-b');
  assert.equal(record('first').walls[0].geometry, firstGeometry);
  assert.equal(record('second').walls[0].geometry, secondGeometry);
  update(5, [{ ...second, enabled: false }, first], 'map-b');
  assert.equal(record('first').walls[0].geometry, firstGeometry);
  update(6, [first], 'map-b');
  assert.equal(record('first').walls[0].geometry, firstGeometry);
  const onlyInside = resources(); batch(200); batch(1);
  assert.deepEqual(resources(), onlyInside, 'inside-only resources ignore cloud bounds');
  const enclosing = { ...rectangle('enclosing', 'outside'), vertices: [{ x: -1000, y: -1000 }, { x: 1000, y: 1000 }] };
  update(7, [enclosing], 'map-b');
  const enclosingResources = resources(); batch(2); batch(3);
  assert.deepEqual(resources(), enclosingResources, 'changed sphere with identical effective mask preserves outside surface');
  update(8, [rectangle('off', 'outside', false)], 'map-b');
  const onlyDisabled = resources(); batch(100);
  assert.deepEqual(resources(), onlyDisabled, 'disabled-only resources ignore cloud bounds');
  update(9, [rectangle(''), rectangle(''), rectangle('same'), rectangle('same')], 'map-b');
  const duplicateKept = viewer._forbiddenZoneResources[2].walls[0].geometry;
  update(10, [rectangle(''), rectangle('same'), { ...rectangle('same'), enabled: false }], 'map-b');
  assert.equal(viewer._forbiddenZoneResources[1].walls[0].geometry, duplicateKept);
  // Degenerate boundaries allocate no attached walls; their unused material
  // still must be disposed immediately, rather than waiting for group cleanup.
  const originalDispose = THREE.Material.prototype.dispose;
  let immediateDisposals = 0;
  THREE.Material.prototype.dispose = function () { immediateDisposals++; return originalDispose.call(this); };
  try {
    assert.deepEqual(viewer._addForbiddenBoundary({ shape: 'line', mode: 'inside', enabled: true,
      vertices: [{ x: 0, y: 0 }, { x: 0, y: 0 }] }), []);
    assert.equal(immediateDisposals, 1);
  } finally { THREE.Material.prototype.dispose = originalDispose; }
  update(11, [], 'map-b'); assert.equal(viewer._forbiddenZoneGroup.children.length, 0);
  for (const count of counts.values()) assert.equal(count, 1, 'every removed resource disposed exactly once');
  update(12, [circle, polygon, line], 'map-b');
  viewer.dispose(); viewer.dispose();
  for (const count of counts.values()) assert.equal(count, 1, 'dispose remains exactly once including shared boundary materials');
  console.log(`PASS forbidden-zone reuse: repeated real page batches, bounds-only outside surface refresh, generation/map fences, edits/deletes, shapes/duplicate IDs, hidden/reveal, exactly-once disposal (${counts.size} resources; Three ${THREE.REVISION}; CPU only, no WebGL)`);
} finally { viewer?.dispose(); fs.rmSync(temp, { recursive: true, force: true }); }
