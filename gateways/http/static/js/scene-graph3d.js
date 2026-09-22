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
      this.clear(); this.key = key;
      const positions = new Map();
      for (const node of graph?.nodes || []) {
        if (!['region','passage'].includes(node.kind) || !node.position
          || ![node.position.x,node.position.y,node.position.z].every(Number.isFinite)) continue;
        const color = node.kind === 'region' ? '#648fb6' : '#c68a3b';
        const marker = new THREE.Mesh(new THREE.SphereGeometry(node.kind === 'region' ? .14 : .085,10,8),
          new THREE.MeshBasicMaterial({color,depthTest:false,transparent:true,opacity:.9}));
        marker.position.set(node.position.x,node.position.y,node.position.z+.2);
        marker.userData.graphNodeId=node.node_id;marker.renderOrder=12;this.group.add(marker);
        this.nodes.push({id:node.node_id,marker,color,kind:node.kind,selected:false,baseRadius:node.kind==='region'?.14:.085});positions.set(node.node_id,marker.position);
      }
      for (const edge of graph?.edges || []) {
        if (edge.kind !== 'access' || !positions.has(edge.source) || !positions.has(edge.target)) continue;
        const geometry = new THREE.BufferGeometry().setFromPoints([positions.get(edge.source),positions.get(edge.target)]);
        const line = new THREE.Line(geometry,new THREE.LineDashedMaterial({color:'#85939f',dashSize:.18,gapSize:.16,
          transparent:true,opacity:.65,depthTest:false,depthWrite:false}));
        line.computeLineDistances();line.renderOrder=10;this.group.add(line);this.edges.push({source:edge.source,target:edge.target,line});
      }
    }
    const route = new Set([...(detail?.route?.region_ids || []),...(detail?.route?.passages || []).map(p=>p.node_id)]);
    for (const node of this.nodes) { node.selected=node.id===detail?.selectedId;node.marker.material.color.set(node.selected || route.has(node.id) ? '#4a9cda' : node.color); node.marker.scale.setScalar(node.selected ? 1.5 : 1); }
    for (const edge of this.edges) { const selected=route.has(edge.source)&&route.has(edge.target);edge.line.material.color.set(selected?'#4a9cda':'#85939f');edge.line.material.opacity=selected?1:.65; }
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
