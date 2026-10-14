#!/usr/bin/env node
// API-bounded counts: the graph endpoint and frontend cap nodes at 500.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const js = `${root}/gateways/http/static/js`;
const url = text => `data:text/javascript;base64,${Buffer.from(text).toString('base64')}`;
const threeUrl = url(fs.readFileSync(`${js}/three.module.min.js`, 'utf8'));
const THREE = await import(threeUrl);
const source = fs.readFileSync(`${js}/scene-graph3d.js`, 'utf8');
const { SceneGraphLayer } = await import(url(source.replace("'./three.module.min.js'", JSON.stringify(threeUrl))));
// Legacy arithmetic retained only as an independent numerical reference.
function baseline(camera, container) {
  if (!this.group.visible || !camera || !container?.clientHeight) return;
  for (const node of this.nodes) {
    const distance = camera.position.distanceTo(node.marker.position);
    const worldPerPixel = 2 * distance * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) / container.clientHeight;
    const pixels = node.kind === 'region' ? 4.5 : 2.8;
    node.marker.scale.setScalar(Math.max(.5, worldPerPixel * pixels / node.baseRadius) * (node.selected ? 1.5 : 1));
  }
}
assert.match(fs.readFileSync(`${js}/pointcloud3d.js`, 'utf8'), /scene-graph3d\.js\?v=20261013-graph-camera/);
assert.match(fs.readFileSync(`${root}/gateways/http/static/console.html`, 'utf8'), /pointcloud3d\.js\?v=20261014-measure-reuse/);
const results = [];
for (const count of [100, 500]) {
  const layer = new SceneGraphLayer(new THREE.Scene());
  layer.update({ visible: true, graph: { graph_revision: '1', nodes: Array.from({length: count}, (_, i) => ({
    node_id: `n${i}`, kind: i % 3 ? 'region' : 'passage', position: { x: i % 40, y: Math.floor(i / 40), z: (i % 9) / 2 }
  })), edges: [] }});
  const camera = new THREE.PerspectiveCamera(60, 1, .1, 1000);
  let height = 800, heightReads = 0, tangentCalls = 0, fovReads = 0, fov = camera.fov;
  Object.defineProperty(camera, 'fov', { get() { fovReads++; return fov; }, set(value) { fov = value; } });
  const container = { get clientHeight() { heightReads++; return height; } };
  const originalTan = Math.tan;
  const totals = { nodes: count, frames: 20, baselineHeightReads: 0, optimizedHeightReads: 0, baselineTangentCalls: 0, optimizedTangentCalls: 0, baselineFovReads: 0, optimizedFovReads: 0 };
  const identities = layer.nodes.map(node => [node.marker, node.marker.geometry, node.marker.material]);
  try {
    Math.tan = x => { tangentCalls++; return originalTan(x); };
    for (let frame = 0; frame < 20; frame++) {
      camera.position.set(frame % 4, frame % 7, 10 + frame * 3);
      camera.fov = 30 + frame * 2; height = 320 + frame * 40;
      layer.nodes.forEach((node, i) => { node.selected = i % 11 === frame % 11; });
      heightReads = tangentCalls = fovReads = 0;
      baseline.call(layer, camera, container);
      const scales = layer.nodes.map(node => node.marker.scale.toArray());
      totals.baselineHeightReads += heightReads; totals.baselineTangentCalls += tangentCalls; totals.baselineFovReads += fovReads;
      heightReads = tangentCalls = fovReads = 0;
      layer.updateForCamera(camera, container);
      assert.deepEqual(layer.nodes.map(node => node.marker.scale.toArray()), scales);
      totals.optimizedHeightReads += heightReads; totals.optimizedTangentCalls += tangentCalls; totals.optimizedFovReads += fovReads;
    }
    for (const [visible, h, cam] of [[false, 800, camera], [true, 0, camera], [true, 800, null]]) {
      layer.group.visible = visible; height = h;
      const scales = layer.nodes.map(node => node.marker.scale.toArray());
      baseline.call(layer, cam, container); layer.updateForCamera(cam, container);
      assert.deepEqual(layer.nodes.map(node => node.marker.scale.toArray()), scales);
    }
    assert.deepEqual(layer.nodes.map(node => [node.marker, node.marker.geometry, node.marker.material]), identities);
    assert.equal(totals.baselineHeightReads, 20 * (count + 1));
    assert.equal(totals.baselineTangentCalls, 20 * count);
    assert.equal(totals.optimizedHeightReads, 20); assert.equal(totals.optimizedTangentCalls, 20);
    assert.equal(totals.baselineFovReads, 20 * count); assert.equal(totals.optimizedFovReads, 20);
    layer.clear(); layer.group.visible = true; heightReads = tangentCalls = fovReads = 0;
    layer.updateForCamera(camera, container);
    assert.equal(heightReads, 1); assert.equal(tangentCalls, 0); assert.equal(fovReads, 0);
  } finally { Math.tan = originalTan; layer.dispose(); }
  results.push(totals);
}
console.log(JSON.stringify({ scope: 'Actual Three/SceneGraph CPU operation counts; production scalar hoist versus legacy arithmetic; no browser layout, timing or FPS claim', exactScaleEquivalence: true, results }, null, 2));
