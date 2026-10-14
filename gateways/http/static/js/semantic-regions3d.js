import * as THREE from './three.module.min.js';

// 三维语义图层仅绘制已绑定的区域；面位于来源轮廓高度，不推导可通行地面。
export class SemanticRegionLayer {
  constructor(scene) {
    this.group = new THREE.Group();
    this.group.name = 'semantic-regions';
    this.group.visible = false;
    scene.add(this.group);
    this.entries = [];
    this.key = null;
    this.bindingKey = null;
    this.selectedId = null;
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
  }

  update(detail) {
    this.group.visible = detail?.visible === true;
    this.selectedId = detail?.selectedId || null;
    this.theme = detail?.theme || {};
    const regions = (detail?.regions || []).filter(r => r.frame_id === detail?.binding?.frame_id
      && r.boundary?.frame_id === r.frame_id && Array.isArray(r.boundary.polygon_points)
      && r.boundary.polygon_points.length >= 3
      && r.boundary.polygon_points.every(p => [p.x,p.y,p.z].every(Number.isFinite)));
    const key = JSON.stringify([detail?.binding, this.theme, regions.map(r => [r.scene_space_id,r.metadata?.scene_revision,r.primary_label,r.display_color,r.boundary])]);
    if (key !== this.key) {
      const bindingKey = JSON.stringify(detail?.binding);
      if (bindingKey !== this.bindingKey) this.clear();
      this.bindingKey = bindingKey;
      this._reconcile(regions);
      this.key = key;
    }
    for (const entry of this.entries) {
      const selected = entry.id === this.selectedId;
      entry.mesh.material.opacity = selected ? .28 : .13;
      for (const line of entry.lines) line.material.opacity = selected ? 1 : .7;
    }
  }

  _disposeEntry(entry) {
    for (const object of [entry.mesh, ...entry.lines, entry.label]) {
      object.removeFromParent();
      // Three sprites share one module-wide quad geometry; the layer does not own it.
      if (object !== entry.label) object.geometry.dispose();
      object.material.map?.dispose(); object.material.dispose();
    }
  }

  _reconcile(regions) {
    const available = new Map();
    for (const entry of this.entries) {
      let bucket = available.get(entry.id);
      if (!bucket) available.set(entry.id, bucket = { records: [], used: 0 });
      bucket.records.push(entry);
    }
    const next = [];
    for (const region of regions) {
      const bucket = available.get(region.scene_space_id);
      next.push(this.add(region, bucket?.records[bucket.used++]));
    }
    for (const bucket of available.values()) for (let i = bucket.used; i < bucket.records.length; i++) {
      this._disposeEntry(bucket.records[i]);
    }
    this.entries = next;

    // Match rebuild ordering for equal-depth transparent objects, including
    // forbidden walls (renderOrder 3) created since the previous region update.
    const peers = new Map(), cohorts = new Map();
    this.group.parent?.traverse(object => {
      if (object === this.group || object.parent === this.group || !object.material) return;
      const transparent = Array.isArray(object.material)
        ? object.material.some(material => material.transparent) : object.material.transparent;
      if (transparent && [3, 4, 24].includes(object.renderOrder)) {
        peers.set(object.renderOrder, Math.max(peers.get(object.renderOrder) ?? -1, object.id));
      }
    });
    for (const entry of next) for (const [key, index, object] of [
      ['mesh', null, entry.mesh], ...entry.lines.map((line, i) => ['lines', i, line]), ['label', null, entry.label],
    ]) {
      let cohort = cohorts.get(object.renderOrder);
      if (!cohort) cohorts.set(object.renderOrder, cohort = []);
      cohort.push({ entry, key, index, object });
    }
    let rewrapped = false;
    for (const cohort of cohorts.values()) {
      if (!cohort.some(({ object }, i) => (i && cohort[i - 1].object.id > object.id)
        || object.id < (peers.get(object.renderOrder) ?? -1))) continue;
      for (const { entry, key, index, object } of cohort) {
        const replacement = key === 'label' ? new THREE.Sprite(object.material)
          : key === 'lines' ? new THREE.LineLoop(object.geometry, object.material)
            : new THREE.Mesh(object.geometry, object.material);
        replacement.position.copy(object.position); replacement.scale.copy(object.scale);
        replacement.quaternion.copy(object.quaternion); replacement.renderOrder = object.renderOrder;
        replacement.userData = { ...object.userData };
        // A rebuild makes labels visible until the next label-layout pass.
        if (key === 'lines') entry.lines[index] = replacement; else entry[key] = replacement;
      }
      rewrapped = true;
    }
    if (rewrapped) this.group.clear();
    let index = 0;
    for (const entry of next) for (const object of [entry.mesh, ...entry.lines, entry.label]) {
      if (rewrapped) this.group.add(object); else this.group.children[index++] = object;
    }
  }

  add(region, entry = null) {
    const points = region.boundary.polygon_points;
    const holes = (region.boundary.metadata?.holes || []).filter(ring => Array.isArray(ring)
      && ring.length >= 3 && ring.every(p => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1])));
    const z = region.boundary.centroid?.position?.z ?? points[0].z;
    const center = region.boundary.centroid?.position;
    const color = /^#[0-9a-f]{6}$/i.test(region.display_color || '') ? region.display_color : '#648fb6';
    const samePoints = entry && entry.points.length === points.length
      && points.every((p, i) => p.x === entry.points[i][0] && p.y === entry.points[i][1] && p.z === entry.points[i][2]);
    const sameHoles = entry && entry.holes.length === holes.length && holes.every((ring, i) =>
      ring.length === entry.holes[i].length && ring.every((p, j) => p[0] === entry.holes[i][j][0] && p[1] === entry.holes[i][j][1]));
    let geometry = null;
    if (!samePoints || !sameHoles || !Object.is(entry?.z, z)) {
      const shape = new THREE.Shape(points.map(p => new THREE.Vector2(p.x,p.y)));
      for (const ring of holes) shape.holes.push(new THREE.Path(ring.map(p => new THREE.Vector2(p[0],p[1]))));
      geometry = new THREE.ShapeGeometry(shape);
    }
    if (!entry) {
      const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
        color, transparent:true, opacity:.13, side:THREE.DoubleSide, depthWrite:false, polygonOffset:true, polygonOffsetFactor:-1,
      }));
      mesh.userData.sceneSpaceId = region.scene_space_id; mesh.renderOrder = 3; this.group.add(mesh);
      entry = { id: region.scene_space_id, mesh, lines: [], label: null };
    }
    if (geometry) {
      if (entry.mesh.geometry !== geometry) { entry.mesh.geometry.dispose(); entry.mesh.geometry = geometry; }
      entry.mesh.position.z = z + .025;
      const rings = [points.map(p=>[p.x,p.y]), ...holes];
      while (entry.lines.length > rings.length) {
        const line = entry.lines.pop(); line.removeFromParent(); line.geometry.dispose(); line.material.dispose();
      }
      rings.forEach((ring, index) => {
        const geometry = new THREE.BufferGeometry().setFromPoints(ring.map(p=>new THREE.Vector3(p[0],p[1],z+.04)));
        let line = entry.lines[index];
        if (line) { line.geometry.dispose(); line.geometry = geometry; }
        else {
          line = new THREE.LineLoop(geometry,new THREE.LineBasicMaterial({color,transparent:true,opacity:.7,depthWrite:false}));
          line.renderOrder=4; this.group.add(line); entry.lines.push(line);
        }
      });
      entry.box = new THREE.Box3().setFromPoints(points.map(p=>new THREE.Vector3(p.x,p.y,p.z)));
      // Saved primitives detect in-place payload changes without hashing or
      // retaining mutable boundary arrays as an equality certificate.
      entry.points = points.map(p => [p.x, p.y, p.z]);
      entry.holes = holes.map(ring => ring.map(p => [p[0], p[1]])); entry.z = z;
    }
    entry.mesh.material.color.set(color);
    for (const line of entry.lines) line.material.color.set(color);
    const text=String(region.primary_label || region.display_index || '区域');
    const surface=/^#[0-9a-f]{6}$/i.test(this.theme?.surface || '') ? this.theme.surface : '#ffffff';
    const ink=/^#[0-9a-f]{6}$/i.test(this.theme?.ink || '') ? this.theme.ink : '#16191d';
    if (!entry.label || entry.text !== text || entry.surface !== surface || entry.ink !== ink) {
      const label = this.label(text);
      if (entry.label) {
        entry.label.material.map.dispose(); entry.label.material.dispose();
        entry.label.material = label.material; entry.label.scale.copy(label.scale);
        entry.label.userData.aspect = label.userData.aspect;
      } else { entry.label = label; this.group.add(label); }
      entry.text = text; entry.surface = surface; entry.ink = ink;
    }
    const label = entry.label;
    if(center && [center.x,center.y,center.z].every(Number.isFinite))label.position.set(center.x,center.y,center.z+.12);
    else label.position.set(points[0].x,points[0].y,z+.12);
    label.userData.sceneSpaceId=region.scene_space_id; label.visible = true;
    return entry;
  }

  label(text) {
    const canvas=document.createElement('canvas'); const ctx=canvas.getContext('2d');
    ctx.font='600 26px system-ui,sans-serif';
    text=text.length>14?text.slice(0,13)+'…':text;
    canvas.width=Math.max(48,Math.ceil(ctx.measureText(text).width)+24);canvas.height=46;
    ctx.fillStyle=/^#[0-9a-f]{6}$/i.test(this.theme?.surface || '') ? this.theme.surface : '#ffffff';ctx.fillRect(0,0,canvas.width,canvas.height);
    ctx.font='600 26px system-ui,sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillStyle=/^#[0-9a-f]{6}$/i.test(this.theme?.ink || '') ? this.theme.ink : '#16191d';ctx.fillText(text,canvas.width/2,canvas.height/2);
    const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
    const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,transparent:true,depthTest:false,depthWrite:false,sizeAttenuation:false}));
    sprite.scale.set(.032*canvas.width/canvas.height,.032,1);sprite.renderOrder=24;
    sprite.userData.aspect=canvas.width/canvas.height;
    return sprite;
  }

  updateLabels(camera, container) {
    if(!this.group.visible)return;
    const width=container.clientWidth,height=container.clientHeight;
    const boxes=[];
    const entries=[...this.entries].sort((a,b)=>Number(b.id===this.selectedId)-Number(a.id===this.selectedId));
    for(const entry of entries){
      const p=entry.label.position.clone().project(camera);
      const h=height*.032/(2*Math.tan(THREE.MathUtils.degToRad(camera.fov)/2)),w=h*entry.label.userData.aspect;
      const x=(p.x+1)*width/2-w/2,y=(1-p.y)*height/2-h/2;
      const selected=entry.id===this.selectedId;
      const visible=p.z>=-1&&p.z<=1&&x+w>=0&&y+h>=0&&x<=width&&y<=height
        &&(selected||!boxes.some(b=>x<b.x+b.w+4&&x+w+4>b.x&&y<b.y+b.h+3&&y+h+3>b.y));
      entry.label.visible=visible;if(visible)boxes.push({x,y,w,h});
    }
  }

  pick(clientX,clientY,camera,canvas) {
    if(!this.group.visible)return null;
    const rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return null;
    this.pointer.set((clientX-rect.left)/rect.width*2-1,-(clientY-rect.top)/rect.height*2+1);
    this.raycaster.setFromCamera(this.pointer,camera);
    const objects=this.entries.flatMap(e=>[e.mesh,...(e.label.visible?[e.label]:[])]);
    return this.raycaster.intersectObjects(objects,false)[0]?.object.userData.sceneSpaceId || null;
  }

  bounds(id) { return this.entries.find(e=>e.id===id)?.box || null; }

  clear() {
    for (const entry of this.entries) this._disposeEntry(entry);
    this.group.clear();this.entries=[];this.key=null;this.bindingKey=null;
  }

  dispose() { this.clear();this.group.parent?.remove(this.group); }
}
