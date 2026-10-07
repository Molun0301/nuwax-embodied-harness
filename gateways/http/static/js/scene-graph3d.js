import * as THREE from './three.module.min.js';

// 真实坐标上的节点与虚线关系；这些连线不表示运动轨迹。
export class SceneGraphLayer {
  constructor(scene) {
    this.group = new THREE.Group(); this.group.name = 'scene-graph'; this.group.visible = false; scene.add(this.group);
    this.nodes = []; this.edges = []; this.key = null; this.raycaster = new THREE.Raycaster(); this.pointer = new THREE.Vector2();
  }

  update(detail) {
    const graph = detail?.graph;
    this.group.visible = detail?.visible === true && Boolean(graph);
    const key = graph?.graph_revision || null;
    if (key !== this.key) {
      this._reconcileGraph(graph);
      this.key = key;
    }

    const route = new Set([...(detail?.route?.region_ids || []),...(detail?.route?.passages || []).map(p=>p.node_id)]);
    for (const node of this.nodes) { node.selected=node.id===detail?.selectedId;node.marker.material.color.set(node.selected || route.has(node.id) ? '#4a9cda' : node.color); node.marker.scale.setScalar(node.selected ? 1.5 : 1); }
    for (const edge of this.edges) { const selected=route.has(edge.source)&&route.has(edge.target);edge.line.material.color.set(selected?'#4a9cda':'#85939f');edge.line.material.opacity=selected?1:.65; }
  }

  _disposeObject(object) {
    object.removeFromParent();
    object.geometry.dispose();
    object.material.dispose();
  }

  _reconcileGraph(graph) {
    const nodeBuckets = new Map();
    for (const node of this.nodes) {
      let bucket = nodeBuckets.get(node.id);
      if (!bucket) nodeBuckets.set(node.id, bucket = { records: [], used: 0 });
      bucket.records.push(node);
    }
    const nodes = [], positions = new Map();
    for (const node of graph?.nodes || []) {
      if (!['region', 'passage'].includes(node.kind) || !node.position
        || ![node.position.x, node.position.y, node.position.z].every(Number.isFinite)) continue;
      const color = node.kind === 'region' ? '#648fb6' : '#c68a3b';
      const baseRadius = node.kind === 'region' ? .14 : .085;
      const bucket = nodeBuckets.get(node.node_id);
      let record = bucket?.records[bucket.used++];
      if (!record) {
        const marker = new THREE.Mesh(new THREE.SphereGeometry(baseRadius, 10, 8),
          new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: .9 }));
        marker.userData.graphNodeId = node.node_id; marker.renderOrder = 12;
        this.group.add(marker);
        record = { id: node.node_id, marker, kind: node.kind, selected: false };
      } else if (record.kind !== node.kind) {
        record.marker.geometry.dispose();
        record.marker.geometry = new THREE.SphereGeometry(baseRadius, 10, 8);
      }
      record.marker.position.set(node.position.x, node.position.y, node.position.z + .2);
      Object.assign(record, { color, kind: node.kind, baseRadius });
      nodes.push(record);
      // Preserve the existing last-occurrence endpoint rule for duplicate IDs.
      positions.set(node.node_id, record.marker.position);
    }
    for (const bucket of nodeBuckets.values()) {
      for (let index = bucket.used; index < bucket.records.length; index++) this._disposeObject(bucket.records[index].marker);
    }

    // Backend edge IDs are fresh UUIDs on each graph revision. Access-edge
    // visuals are identified by directed endpoints and duplicate occurrence.
    const edgeBuckets = new Map();
    for (const edge of this.edges) {
      let targets = edgeBuckets.get(edge.source);
      if (!targets) edgeBuckets.set(edge.source, targets = new Map());
      let bucket = targets.get(edge.target);
      if (!bucket) targets.set(edge.target, bucket = { records: [], used: 0 });
      bucket.records.push(edge);
    }
    const edges = [];
    for (const edge of graph?.edges || []) {
      if (edge.kind !== 'access' || !positions.has(edge.source) || !positions.has(edge.target)) continue;
      const start = positions.get(edge.source), end = positions.get(edge.target);
      const bucket = edgeBuckets.get(edge.source)?.get(edge.target);
      let record = bucket?.records[bucket.used++];
      if (!record) {
        const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);
        const line = new THREE.Line(geometry, new THREE.LineDashedMaterial({
          color: '#85939f', dashSize: .18, gapSize: .16,
          transparent: true, opacity: .65, depthTest: false, depthWrite: false,
        }));
        line.computeLineDistances(); line.renderOrder = 10; this.group.add(line);
        record = { source: edge.source, target: edge.target, line };
      } else {
        const geometry = record.line.geometry, position = geometry.attributes.position;
        if (position.getX(0) !== Math.fround(start.x) || position.getY(0) !== Math.fround(start.y)
          || position.getZ(0) !== Math.fround(start.z) || position.getX(1) !== Math.fround(end.x)
          || position.getY(1) !== Math.fround(end.y) || position.getZ(1) !== Math.fround(end.z)) {
          position.setXYZ(0, start.x, start.y, start.z); position.setXYZ(1, end.x, end.y, end.z);
          position.needsUpdate = true;
          // Match computeLineDistances' Float32 endpoints without replacing its buffer.
          const dx = position.getX(1) - position.getX(0), dy = position.getY(1) - position.getY(0);
          const dz = position.getZ(1) - position.getZ(0);
          geometry.attributes.lineDistance.setX(1, Math.sqrt(dx * dx + dy * dy + dz * dz));
          geometry.attributes.lineDistance.needsUpdate = true;
          geometry.computeBoundingSphere();
          if (geometry.boundingBox) geometry.computeBoundingBox();
        }
      }
      edges.push(record);
    }
    for (const targets of edgeBuckets.values()) for (const bucket of targets.values()) {
      for (let index = bucket.used; index < bucket.records.length; index++) this._disposeObject(bucket.records[index].line);
    }
    this.nodes = nodes; this.edges = edges;

    // Transparent equal-depth ties use immutable Object3D IDs, not child order.
    // Rewrap only an inverted cohort, leaving geometry/material ownership intact.
    // Navigation previews share renderOrder=10. A peer created since the last
    // graph revision must still precede graph wrappers, as it did on rebuild.
    const externalPeerIds = new Map();
    this.group.parent?.traverse(object => {
      if (object === this.group || object.parent === this.group || !object.material) return;
      const transparent = Array.isArray(object.material)
        ? object.material.some(material => material.transparent) : object.material.transparent;
      if (transparent && (object.renderOrder === 10 || object.renderOrder === 12)) {
        externalPeerIds.set(object.renderOrder, Math.max(externalPeerIds.get(object.renderOrder) ?? -1, object.id));
      }
    });
    let rewrapped = false;
    for (const [records, key] of [[nodes, 'marker'], [edges, 'line']]) {
      if (!records.some((record, index) => (index && records[index - 1][key].id > record[key].id)
        || record[key].id < (externalPeerIds.get(record[key].renderOrder) ?? -1))) continue;
      for (const record of records) {
        const object = record[key];
        const replacement = key === 'marker' ? new THREE.Mesh(object.geometry, object.material)
          : new THREE.Line(object.geometry, object.material);
        replacement.position.copy(object.position); replacement.scale.copy(object.scale);
        replacement.quaternion.copy(object.quaternion); replacement.renderOrder = object.renderOrder;
        replacement.userData = { ...object.userData };
        record[key] = replacement;
      }
      rewrapped = true;
    }
    if (rewrapped) this.group.clear();
    let index = 0;
    for (const object of [...nodes.map(node => node.marker), ...edges.map(edge => edge.line)]) {
      if (rewrapped) this.group.add(object);
      else this.group.children[index++] = object;
    }
  }

  updateForCamera(camera,container) {
    if(!this.group.visible||!camera||!container?.clientHeight)return;
    for(const node of this.nodes){
      const distance=camera.position.distanceTo(node.marker.position);
      const worldPerPixel=2*distance*Math.tan(THREE.MathUtils.degToRad(camera.fov)/2)/container.clientHeight;
      const pixels=node.kind==='region'?4.5:2.8;
      node.marker.scale.setScalar(Math.max(.5,worldPerPixel*pixels/node.baseRadius)*(node.selected?1.5:1));
    }
  }

  pick(clientX,clientY,camera,canvas) {
    if(!this.group.visible)return null;const rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return null;
    this.pointer.set((clientX-rect.left)/rect.width*2-1,-(clientY-rect.top)/rect.height*2+1);
    this.raycaster.setFromCamera(this.pointer,camera);
    return this.raycaster.intersectObjects(this.nodes.map(n=>n.marker),false)[0]?.object.userData.graphNodeId || null;
  }

  clear() { this.group.traverse(object=>{object.geometry?.dispose();object.material?.dispose();});this.group.clear();this.nodes=[];this.edges=[];this.key=null; }
  dispose() { this.clear();this.group.parent?.remove(this.group); }
}
