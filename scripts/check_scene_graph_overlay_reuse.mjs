#!/usr/bin/env node
// Actual bundled Three CPU checks; no renderer or WebGL context.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import assert from 'node:assert/strict';
import { fileURLToPath, pathToFileURL } from 'node:url';
const repo = path.resolve(process.argv[2] || path.join(path.dirname(fileURLToPath(import.meta.url)), '..'));
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'neh-graph-reuse-'));
for (const name of ['scene-graph3d', 'three.module.min']) fs.copyFileSync(
  path.join(repo, 'gateways/http/static/js', `${name}.js`), path.join(temp, `${name}.js`));
fs.writeFileSync(path.join(temp, 'package.json'), '{"type":"module"}');
let layer;
try {
  const THREE = await import(pathToFileURL(path.join(temp, 'three.module.min.js')));
  const { SceneGraphLayer } = await import(pathToFileURL(path.join(temp, 'scene-graph3d.js')));
  const scene = new THREE.Scene(); layer = new SceneGraphLayer(scene);
  const threeSource = fs.readFileSync(path.join(temp, 'three.module.min.js'), 'utf8');
  const comparatorSource = threeSource.match(/function Al\(t,e\)\{[^}]+\}/)?.[0];
  assert.ok(comparatorSource); const transparentSort = new Function(`${comparatorSource}; return Al;`)();
  const counts = new Map();
  const resources = () => layer.group.children.flatMap(o => [o.geometry, o.material]);
  const attributes = () => layer.edges.flatMap(e => [e.line.geometry.attributes.position, e.line.geometry.attributes.lineDistance]);
  function watch() {
    for (const resource of resources()) if (!counts.has(resource)) {
      counts.set(resource, 0); resource.addEventListener('dispose', () => counts.set(resource, counts.get(resource) + 1));
    }
  }
  const renderEntry = object => ({ id: object.id, object, groupOrder: 0, renderOrder: object.renderOrder, z: 0 });
  function assertOrder() {
    const objects = layer.group.children;
    assert.equal(new Set(objects).size, objects.length);
    for (const object of objects) assert.equal(object.parent, layer.group);
    assert.deepEqual(objects, [...layer.nodes.map(n => n.marker), ...layer.edges.map(e => e.line)]);
    const expected = objects.slice().sort((a, b) => a.renderOrder - b.renderOrder);
    assert.deepEqual(objects.map(renderEntry).sort(transparentSort).map(e => e.object), expected);
  }
  let revision = 0;
  const node = (id, x = 0, kind = 'region') => ({ node_id: id, kind, position: { x, y: .2, z: 0 } });
  const edge = (source, target, id = 'edge') => ({ edge_id: id, source, target, kind: 'access' });
  let graph = { nodes: [node('a'), node('b', 1), node('c', 2)], edges: [edge('a', 'b')] };
  function update(options = {}, bump = true) {
    if (bump) graph.graph_revision = String(++revision);
    layer.update({ visible: true, graph, ...options }); watch(); assertOrder();
  }
  layer.update({ visible: true, graph }); assert.equal(layer.nodes.length, 0, 'initial missing revision retains legacy gate');
  update();
  const initial = resources(), initialObjects = layer.group.children.slice(), initialAttributes = attributes();
  const positions = layer.edges[0].line.geometry.attributes.position;
  const distances = layer.edges[0].line.geometry.attributes.lineDistance;
  const initialVersions = [positions.version, distances.version];
  graph = structuredClone(graph); graph.edges[0].edge_id = 'regenerated-uuid'; update();
  assert.deepEqual(resources(), initial); assert.deepEqual(layer.group.children, initialObjects);
  assert.deepEqual(attributes(), initialAttributes); assert.deepEqual([positions.version, distances.version], initialVersions);
  graph.nodes[1].position.x = 20; update({ selectedId: 'b' }, false);
  assert.equal(layer.nodes[1].marker.position.x, 1, 'same revision mutations remain ignored');
  assert.equal(layer.nodes[1].marker.scale.x, 1.5, 'same revision selection still updates');
  graph.nodes[1].position.x = 1;
  graph.edges.push(edge('b', 'c', 'new')); update();
  assert.equal(resources().filter(r => initial.includes(r)).length, initial.length);
  assert.equal(resources().length - initial.length, 2, 'one edge creates only geometry/material');
  const beforeMove = resources(), moveAttributes = attributes();
  const aMaterial = layer.nodes[0].marker.material;
  for (const e of layer.edges) { e.line.geometry.computeBoundingBox(); e.line.geometry.computeBoundingSphere(); }
  graph.nodes[1].position = { x: 1.123456789, y: 3.123456789, z: 2.012345678 }; update();
  assert.deepEqual(resources(), beforeMove); assert.deepEqual(attributes(), moveAttributes);
  for (const e of layer.edges) {
    const geometry = e.line.geometry, p = geometry.attributes.position;
    const start = new THREE.Vector3().fromBufferAttribute(p, 0), end = new THREE.Vector3().fromBufferAttribute(p, 1);
    assert.equal(geometry.attributes.lineDistance.getX(0), 0);
    assert.equal(geometry.attributes.lineDistance.getX(1), Math.fround(start.distanceTo(end)));
    assert.ok(geometry.boundingBox.containsPoint(start) && geometry.boundingBox.containsPoint(end));
    assert.ok(geometry.boundingSphere.center.distanceTo(start) <= geometry.boundingSphere.radius + 1e-12);
  }
  const stableVersions = attributes().map(a => a.version); update();
  assert.deepEqual(attributes().map(a => a.version), stableVersions, 'unchanged decimal endpoints do not upload again');
  const formerGeometry = layer.nodes[0].marker.geometry;
  graph.nodes[0].kind = 'passage'; update();
  assert.equal(counts.get(formerGeometry), 1); assert.equal(layer.nodes[0].marker.material, aMaterial);
  assert.equal(layer.nodes[0].marker.geometry.parameters.radius, .085); assert.equal(layer.nodes[0].baseRadius, .085);
  assert.equal(layer.nodes[0].marker.material.color.getHex(), 0xc68a3b);
  const resourcesBeforeReorder = new Set(resources());
  graph.nodes.reverse(); graph.edges.reverse(); graph.edges.forEach((e, i) => { e.edge_id = `uuid-${i}`; }); update();
  assert.deepEqual(new Set(resources()), resourcesBeforeReorder);
  graph.nodes.splice(1, 0, node('inserted', 7)); graph.edges.splice(1, 0, edge('a', 'inserted')); update();
  assert.equal(resources().filter(r => resourcesBeforeReorder.has(r)).length, resourcesBeforeReorder.size);
  // A newer navigation preview shares transparent renderOrder=10 with edges.
  const peer = new THREE.Mesh(new THREE.SphereGeometry(.1), new THREE.MeshBasicMaterial({ transparent: true }));
  peer.renderOrder = 10; scene.add(peer);
  const beforePeerRevision = new Set(resources()); update();
  assert.deepEqual(new Set(resources()), beforePeerRevision);
  const tied = [...layer.edges.map(e => e.line), peer].map(renderEntry).sort(transparentSort).map(e => e.object);
  assert.equal(tied[0], peer, 'new graph revision remains after pre-existing external peer at equal depth');
  const wrappersAfterPeer = layer.group.children.slice(); update();
  assert.deepEqual(layer.group.children, wrappersAfterPeer, 'older external peer does not repeatedly rewrap');
  peer.removeFromParent(); peer.geometry.dispose(); peer.material.dispose();
  update({ selectedId: 'a', route: { region_ids: ['a', 'inserted'], passages: [] } });
  const a = layer.nodes.find(n => n.id === 'a'), selectedEdge = layer.edges.find(e => e.source === 'a' && e.target === 'inserted');
  assert.equal(a.selected, true); assert.equal(a.marker.scale.x, 1.5); assert.equal(a.marker.material.color.getHex(), 0x4a9cda);
  assert.equal(selectedEdge.line.material.color.getHex(), 0x4a9cda); assert.equal(selectedEdge.line.material.opacity, 1);
  const camera = new THREE.PerspectiveCamera(60, 1, .1, 100); camera.position.set(0, .2, 10); camera.lookAt(0, .2, 0);
  layer.updateForCamera(camera, { clientHeight: 800 });
  const expectedScale = Math.max(.5, (2 * camera.position.distanceTo(a.marker.position) * Math.tan(Math.PI / 6) / 800) * 2.8 / .085) * 1.5;
  assert.ok(Math.abs(a.marker.scale.x - expectedScale) < 1e-12);
  const visibleResources = resources(); update({ visible: false }, false); assert.equal(layer.group.visible, false);
  assert.equal(layer.pick(400, 400, camera, { getBoundingClientRect: () => ({ width: 800, height: 800 }) }), null);
  update({}, false); assert.equal(layer.group.visible, true); assert.deepEqual(resources(), visibleResources);
  // Same pair duplicates remain distinct; raw IDs cannot collide on delimiters.
  graph = { nodes: [node('a|b', 0), node('c', 1), node('a', 2), node('b|c', 3)],
    edges: [edge('a|b', 'c'), edge('a', 'b|c'), edge('a|b', 'c'), edge('c', 'a|b')] }; update();
  assert.equal(layer.edges.length, 4); const duplicateResources = resources(); update(); assert.deepEqual(resources(), duplicateResources);
  graph.edges.shift(); update(); assert.equal(layer.edges.length, 3);
  graph = { nodes: [node('duplicate', 1), node('duplicate', 9), node(undefined, 3), node('', 4), node('other', 2)],
    edges: [edge('duplicate', 'other'), edge(undefined, ''), edge('', undefined)] }; update();
  assert.equal(layer.nodes.length, 5); assert.equal(layer.edges.length, 3);
  assert.equal(layer.edges[0].line.geometry.attributes.position.getX(0), 9, 'last valid duplicate owns endpoint position');
  const duplicates = resources(); update(); assert.deepEqual(resources(), duplicates);
  graph.nodes.push({ node_id: 'bad', kind: 'region', position: { x: null, y: 0, z: 0 } },
    { node_id: 'bad2', kind: 'passage', position: { x: Infinity, y: 0, z: 0 } }, { node_id: 'bad3', kind: 'region' },
    node('excluded', 1, 'object'));
  graph.edges.push(edge('bad', 'other'), { ...edge('duplicate', 'other'), kind: 'contains' }, edge('missing', 'other')); update();
  assert.equal(layer.nodes.length, 5); assert.equal(layer.edges.length, 3);
  graph.nodes = graph.nodes.filter(n => n.node_id !== 'other'); update(); assert.equal(layer.edges.length, 2);
  graph = { nodes: [node('top', 0), node('under', 0)], edges: [] }; update();
  scene.updateMatrixWorld(true); camera.updateMatrixWorld(true);
  const canvas = { getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 800 }) };
  assert.equal(layer.pick(400, 400, camera, canvas), 'top');
  graph.nodes.reverse(); update(); scene.updateMatrixWorld(true);
  assert.equal(layer.pick(400, 400, camera, canvas), 'under');
  layer.update({ graph: null, visible: true }); assert.equal(layer.group.visible, false); assert.equal(layer.nodes.length, 0);
  for (const count of counts.values()) assert.equal(count, 1);
  layer.clear(); layer.clear(); update(); layer.dispose(); layer.dispose();
  assert.equal(layer.group.parent, null);
  for (const count of counts.values()) assert.equal(count, 1, 'every owned resource disposed exactly once');
  console.log(`PASS scene-graph reconcile: revision gate, UUID churn, edge add/remove, mutable node pose/kind, Float32 dash buffers/bounds, duplicate/raw IDs, endpoint filtering, actual transparent sort/external preview, selection/camera/raycast, visibility/clear/disposal (${counts.size} resources; Three ${THREE.REVISION}; CPU only, no WebGL)`);
} finally { layer?.dispose(); fs.rmSync(temp, { recursive: true, force: true }); }
