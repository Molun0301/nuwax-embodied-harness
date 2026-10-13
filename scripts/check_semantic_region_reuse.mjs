#!/usr/bin/env node
// Real bundled Three CPU objects and stub Canvas2D; no browser or WebGL claim.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath, pathToFileURL } from 'node:url';
const repo = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-region-reuse-'));
for (const name of ['semantic-regions3d', 'three.module.min']) fs.copyFileSync(
  path.join(repo, 'gateways/http/static/js', `${name}.js`), path.join(temp, `${name}.js`));
fs.writeFileSync(path.join(temp, 'package.json'), '{"type":"module"}');
let canvases = 0;
globalThis.document = { createElement(name) {
  assert.equal(name, 'canvas'); canvases++;
  const canvas = { width: 300, height: 150 };
  canvas.getContext = () => ({ measureText: text => ({ width: text.length * 12 }),
    fillRect() { canvas.surface = this.fillStyle; }, fillText(text) { canvas.text = text; canvas.ink = this.fillStyle; } });
  return canvas;
} };
let layer;
try {
  const THREE = await import(pathToFileURL(path.join(temp, 'three.module.min.js')));
  const { SemanticRegionLayer } = await import(pathToFileURL(path.join(temp, 'semantic-regions3d.js')));
  let OracleLayer = SemanticRegionLayer;
  const baseline = process.argv.indexOf('--baseline');
  if (baseline >= 0) {
    fs.copyFileSync(path.resolve(process.argv[baseline + 1]), path.join(temp, 'baseline.js'));
    ({ SemanticRegionLayer: OracleLayer } = await import(pathToFileURL(path.join(temp, 'baseline.js'))));
  }
  const scene = new THREE.Scene(); layer = new SemanticRegionLayer(scene);
  const comparatorSource = fs.readFileSync(path.join(temp, 'three.module.min.js'), 'utf8').match(/function Al\(t,e\)\{[^}]+\}/)?.[0];
  assert.ok(comparatorSource); const sort = new Function(`${comparatorSource}; return Al;`)();
  const counts = new Map();
  const resources = () => layer.entries.flatMap(e => [e.mesh.geometry, e.mesh.material,
    ...e.lines.flatMap(l => [l.geometry, l.material]), e.label.material.map, e.label.material]);
  function watch() { for (const r of resources()) if (!counts.has(r)) {
    counts.set(r, 0); r.addEventListener('dispose', () => counts.set(r, counts.get(r) + 1));
  } }
  const region = (id, x = 0) => ({ scene_space_id: id, frame_id: 'map', metadata: { scene_revision: 1 },
    primary_label: `Region ${id}`, display_color: '#648fb6', boundary: { frame_id: 'map',
      polygon_points: [{ x, y: 0, z: 0 }, { x: x + 2, y: 0, z: 0 }, { x: x + 2, y: 2, z: 0 }, { x, y: 2, z: 0 }] } });
  let detail = { visible: true, binding: { frame_id: 'map', source_epoch: 'a' }, regions: [region('a'), region('b', 3), region('c', 6)] };
  const positions = geometry => Object.fromEntries(Object.entries(geometry.attributes).map(([k, a]) => [k, [...a.array]]));
  const geometry = g => ({ attributes: positions(g), index: g.index ? [...g.index.array] : null });
  const material = m => [m.type, m.color.getHex(), m.opacity, m.transparent, m.depthTest, m.depthWrite, m.side, m.polygonOffset, m.polygonOffsetFactor];
  function snapshot(value) { return value.entries.map(e => ({ id: e.id,
    mesh: [geometry(e.mesh.geometry), material(e.mesh.material), e.mesh.position.toArray(), e.mesh.userData],
    lines: e.lines.map(l => [geometry(l.geometry), material(l.material)]),
    label: [e.label.position.toArray(), e.label.scale.toArray(), e.label.userData,
      e.label.material.map.image.text, e.label.material.map.image.surface, e.label.material.map.image.ink],
    bounds: [e.box.min.toArray(), e.box.max.toArray()],
  })); }
  function order() {
    const objects = layer.group.children, expected = layer.entries.flatMap(e => [e.mesh, ...e.lines, e.label]);
    assert.deepEqual(objects, expected); assert.equal(new Set(objects).size, objects.length);
    for (const o of objects) assert.equal(o.parent, layer.group);
    const render = object => ({ id: object.id, object, groupOrder: 0, renderOrder: object.renderOrder, z: 0 });
    assert.deepEqual(objects.map(render).sort(sort).map(e => e.object), objects.slice().sort((a, b) => a.renderOrder - b.renderOrder));
    return render;
  }
  let checks = 0, spriteGeometryDisposals = 0;
  function update({ compare = true } = {}) {
    layer.update(detail); watch(); order(); checks++;
    if (compare) {
      const oracle = new OracleLayer(new THREE.Scene()); oracle.update(detail);
      assert.deepEqual(snapshot(layer), snapshot(oracle), 'matches a fresh rebuild of current visible primitives');
      // Original clear() also disposed Three's shared Sprite quad; exclude the
      // reference layer from candidate ownership accounting.
      const saved = spriteGeometryDisposals;
      oracle.dispose(); spriteGeometryDisposals = saved;
    }
  }
  update(); const initial = resources(), wrappers = layer.group.children.slice();
  const externalSprite = new THREE.Sprite(new THREE.SpriteMaterial()); scene.add(externalSprite);
  const sharedSpriteGeometry = layer.entries[0].label.geometry;
  assert.equal(externalSprite.geometry, sharedSpriteGeometry);
  assert.ok(layer.entries.every(e => e.label.geometry === sharedSpriteGeometry));
  // Count only the candidate's lifecycle, not the old oracle's known blanket traversal.
  const trackSpriteDisposal = () => { spriteGeometryDisposals++; };
  sharedSpriteGeometry.addEventListener('dispose', trackSpriteDisposal);
  detail = structuredClone(detail); update(); assert.deepEqual(resources(), initial); assert.deepEqual(layer.group.children, wrappers);
  detail.regions[1].metadata.scene_revision++; update(); assert.deepEqual(resources(), initial);
  detail.selectedId = 'b'; update(); assert.equal(layer.entries[1].mesh.material.opacity, .28);
  assert.equal(layer.entries[0].lines[0].material.opacity, .7);
  const beforeLabel = resources(); detail.regions[1].primary_label = 'Edited'; update();
  assert.equal(resources().filter(r => beforeLabel.includes(r)).length, 16);
  const beforeColor = resources(); detail.regions[1].display_color = '#ABCDEF'; update(); assert.deepEqual(resources(), beforeColor);
  detail.regions[1].display_color = 'invalid'; update(); assert.deepEqual(resources(), beforeColor);
  detail.theme = { surface: '#222222', ink: '#abcdef' }; const beforeTheme = resources(); update();
  assert.equal(resources().filter(r => beforeTheme.includes(r)).length, 12, 'theme replaces label resources only');
  const beforePolygon = resources(); detail.regions[1].boundary.polygon_points[1].x += .1; update();
  assert.equal(resources().filter(r => beforePolygon.includes(r)).length, 16, 'polygon replaces affected fill/outline geometries only');
  detail.regions[1].boundary.metadata = { holes: [[[3.2,.2],[3.5,.2],[3.5,.5]], [[0,0]], [[0,0],[NaN,1],[1,1]]] }; update();
  assert.equal(layer.entries[1].lines.length, 2);
  detail.regions[1].boundary.metadata.holes[0][0][0] += .05; update();
  detail.regions[1].boundary.metadata.holes = []; update(); assert.equal(layer.entries[1].lines.length, 1);
  detail.regions[1].boundary.centroid = { position: { x: 4, y: 1, z: .8 } }; update();
  detail.regions[1].boundary.centroid.position.x = 5; const beforeCenter = resources(); update(); assert.deepEqual(resources(), beforeCenter);
  detail.regions[1].boundary.centroid.position.x = null; update();
  detail.regions[0].boundary.polygon_points[2].z = 2; update(); assert.equal(layer.bounds('a').max.z, 2);
  detail.regions.reverse(); const beforeOrder = new Set(resources()); update(); assert.deepEqual(new Set(resources()), beforeOrder);
  detail.regions.splice(1, 0, region('inserted', 9)); update();
  detail.regions.push(region('a', 12), region(undefined, 15), region('', 18)); update();
  detail.regions.reverse(); update(); assert.equal(layer.entries.length, 7);
  detail.regions.push({ ...region('wrong'), frame_id: 'other' }, { ...region('bad'), boundary: { frame_id: 'map', polygon_points: [{x:0,y:0,z:0}] } }); update();
  assert.equal(layer.entries.length, 7);
  detail.regions = detail.regions.filter(r => r.scene_space_id !== 'inserted'); update();
  const peer = new THREE.Mesh(new THREE.BoxGeometry(1,1,1), new THREE.MeshBasicMaterial({ transparent: true })); peer.renderOrder = 3; scene.add(peer);
  detail.regions[0].metadata.scene_revision++; const beforePeer = new Set(resources()); update();
  assert.deepEqual(new Set(resources()), beforePeer);
  const render = order();
  assert.equal([peer, ...layer.entries.map(e => e.mesh)].map(render).sort(sort)[0].object, peer);
  const afterPeer = layer.group.children.slice(); detail.regions[0].metadata.scene_revision++; update(); assert.deepEqual(layer.group.children, afterPeer);
  peer.removeFromParent(); peer.geometry.dispose(); peer.material.dispose();
  detail.visible = false; const beforeHide = resources(); update(); assert.equal(layer.group.visible, false);
  detail.visible = true; update(); assert.deepEqual(resources(), beforeHide);
  detail.binding.source_epoch = 'b'; update(); assert.equal(resources().filter(r => beforeHide.includes(r)).length, 0, 'binding change resets resources');
  detail.regions = [region('under'), region('top')]; detail.selectedId = 'top'; update();
  const camera = new THREE.PerspectiveCamera(60, 1, .1, 100); camera.position.set(1, 1, 10); camera.lookAt(1,1,0); camera.updateMatrixWorld(true);
  layer.updateLabels(camera, { clientWidth: 800, clientHeight: 800 });
  assert.equal(layer.entries[1].label.visible, true); assert.equal(layer.entries[0].label.visible, false);
  scene.updateMatrixWorld(true);
  const canvas = { getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 800 }) };
  assert.equal(layer.pick(400, 400, camera, canvas), 'under', 'equal-distance picking keeps original input order');
  detail.visible = false; update(); assert.equal(layer.pick(400,400,camera,canvas), null);
  detail.visible = true; detail.regions[0].primary_label = ''; detail.regions[0].display_index = 7; update();
  const priorText = layer.entries[0].label.material.map.image.text;
  detail.regions[0].display_index = 8; update({ compare: false }); assert.equal(layer.entries[0].label.material.map.image.text, priorText, 'legacy same-key display_index gate');
  detail.regions[1].metadata.scene_revision++; update(); assert.equal(layer.entries[0].label.material.map.image.text, '8');
  detail.regions = []; update(); assert.equal(layer.group.children.length, 0);
  detail.regions = [region('after-clear')]; update(); layer.clear(); assert.equal(layer.key, null); assert.equal(layer.bindingKey, null);
  update(); layer.dispose(); assert.equal(layer.group.parent, null);
  assert.equal(spriteGeometryDisposals, 0, 'shared Three Sprite geometry is never disposed by a region');
  assert.equal(externalSprite.parent, scene, 'region lifecycle does not remove external sprites');
  assert.equal(externalSprite.geometry, sharedSpriteGeometry);
  externalSprite.removeFromParent(); externalSprite.material.dispose();
  sharedSpriteGeometry.removeEventListener('dispose', trackSpriteDisposal);
  assert.ok([...counts.values()].every(n => n === 1), 'each owned resource disposed exactly once');
  console.log(`PASS semantic region reuse: ${checks} updates; geometry/theme/label separation, in-place edits, holes, duplicates, binding reset, order/peer ties, selection, picking, visibility, empty/clear/dispose. Real Three CPU only; no WebGL.`);
} finally { layer?.dispose(); fs.rmSync(temp, { recursive: true, force: true }); }
