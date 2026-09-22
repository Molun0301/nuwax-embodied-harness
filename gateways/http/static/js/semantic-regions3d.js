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
      this.clear(); this.key = key;
      for (const region of regions) this.add(region);
    }
    for (const entry of this.entries) {
      const selected = entry.id === this.selectedId;
      entry.mesh.material.opacity = selected ? .28 : .13;
      for (const line of entry.lines) line.material.opacity = selected ? 1 : .7;
    }
  }

  add(region) {
    const points = region.boundary.polygon_points;
    const color = /^#[0-9a-f]{6}$/i.test(region.display_color || '') ? region.display_color : '#648fb6';
    const shape = new THREE.Shape(points.map(p => new THREE.Vector2(p.x,p.y)));
    const holes = (region.boundary.metadata?.holes || []).filter(ring => Array.isArray(ring)
      && ring.length >= 3 && ring.every(p => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1])));
    for (const ring of holes) shape.holes.push(new THREE.Path(ring.map(p => new THREE.Vector2(p[0],p[1]))));
    const z = region.boundary.centroid?.position?.z ?? points[0].z;
    const mesh = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({
      color, transparent:true, opacity:.13, side:THREE.DoubleSide, depthWrite:false, polygonOffset:true, polygonOffsetFactor:-1,
    }));
    mesh.position.z = z + .025;
    mesh.userData.sceneSpaceId = region.scene_space_id;
    mesh.renderOrder = 3;
    this.group.add(mesh);
    const lines=[];
    for (const ring of [points.map(p=>[p.x,p.y]), ...holes]) {
      const geometry=new THREE.BufferGeometry().setFromPoints(ring.map(p=>new THREE.Vector3(p[0],p[1],z+.04)));
      const line=new THREE.LineLoop(geometry,new THREE.LineBasicMaterial({color,transparent:true,opacity:.7,depthWrite:false}));
      line.renderOrder=4;this.group.add(line);lines.push(line);
    }
    const text=String(region.primary_label || region.display_index || '区域');
    const label=this.label(text);
    const center=region.boundary.centroid?.position;
    if(center && [center.x,center.y,center.z].every(Number.isFinite))label.position.set(center.x,center.y,center.z+.12);
    else label.position.set(points[0].x,points[0].y,z+.12);
    label.userData.sceneSpaceId=region.scene_space_id;
    this.group.add(label);
    const box=new THREE.Box3().setFromPoints(points.map(p=>new THREE.Vector3(p.x,p.y,p.z)));
    this.entries.push({id:region.scene_space_id,mesh,lines,label,box});
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
    this.group.traverse(object=>{object.geometry?.dispose();object.material?.map?.dispose();object.material?.dispose();});
    this.group.clear();this.entries=[];this.key=null;
  }

  dispose() { this.clear();this.group.parent?.remove(this.group); }
}
