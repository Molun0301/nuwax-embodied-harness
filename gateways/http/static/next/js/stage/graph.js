/* 关系图：从原控制台 scene-graph.js 迁出读取、布局与绘制（get_scene_graph、力导向或按地图位置排布、SVG 节点与连线、
   二维“区域连接”叠加、三维图层），行为不变。关系图在新控制台是一个视图（占满舞台），节点详情、路径查询在 P3 语义面板，
   这里只提供选择回调。
   改正：原来判断关系图是否属于当前地图时比较 source_name、source_epoch、update_sequence，地图绑定里没有这些字段，
   “区域连接”图层因此从不显示；现与语义区域一致，按地图名、坐标系与地图修订比较。 */

const NS = 'http://www.w3.org/2000/svg';
export const NODE_KINDS = { region: '区域', passage: '连接口', object: '物品', observation: '观察' };

/* svg：关系图视图的 <svg>；binding()、mode()、requestDraw() 同语义区域；onSelect(node) 为点选节点。 */
export function createGraph({ api, stream, svg, binding, mode, requestDraw, onSelect }) {
  const state = { graph: null, selected: null, route: null, layer: false, layout: 'relations', request: null, generation: 0,
    positions: new Map(), layoutKey: '', contextKey: '', view: null, needsFit: false, query: '', drag: null, timer: null, error: '' };
  const listeners = new Set();
  const changed = () => { for (const fn of listeners) fn(); };
  const el = (tag, attrs = {}) => { const n = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, String(v)); return n; };
  const interested = () => mode() === 'graph' || state.layer;
  const bindingKey = b => JSON.stringify([b?.map_name, b?.frame_id, b?.map_revision]);
  function current() {
    const b = binding(), g = state.graph?.map_binding;
    return Boolean(b?.available && g && ['map_name', 'frame_id', 'map_revision'].every(key => b[key] != null && b[key] === g[key]));
  }
  function notifyLayer() {
    const visible = Boolean(current() && state.layer);
    window.dispatchEvent(new CustomEvent('pointcloud3d:scene-graph', { detail: { graph: visible ? state.graph : null, visible, selectedId: state.selected, route: state.route } }));
    requestDraw();
  }
  async function load() {
    if (!interested() || !binding()?.available) return;
    if (state.request) return state.request;
    const generation = state.generation;
    state.request = (async () => {
      try {
        const graph = await api.invoke('get_scene_graph', { limit: 500, kinds: ['region', 'passage', 'object'] });
        if (generation !== state.generation) return;
        const b = binding();
        if (!b?.available || bindingKey(b) !== bindingKey(graph.map_binding)) throw new Error('地图已更新，等待对应关系图。');
        if (state.graph?.graph_revision !== graph.graph_revision) state.route = null;
        state.graph = graph;
        state.error = '';
        if (!graph.nodes.some(n => n.node_id === state.selected)) state.selected = null;
        render();
        notifyLayer();
      } catch (error) {
        if (generation !== state.generation) return;
        state.error = error.message;
        notifyLayer();
      }
      changed();
    })().finally(() => { state.request = null; if (generation !== state.generation && interested()) schedule(); });
    return state.request;
  }
  function schedule() { clearTimeout(state.timer); if (interested()) state.timer = setTimeout(load, 300); }
  function layout(nodes, edges) {
    const key = JSON.stringify([state.layout, nodes.map(n => [n.node_id, n.position]), edges.map(e => [e.source, e.target])]);
    if (key === state.layoutKey) return;
    state.layoutKey = key;
    state.positions = new Map();
    state.view = null;
    if (state.layout === 'spatial') {
      for (const n of nodes) if (n.position) state.positions.set(n.node_id, { x: n.position.x * 60, y: -n.position.y * 60 });
      return;
    }
    // 确定性的力导向排布让高连接度区域展开；相同节点与边保持同一布局。
    const count = nodes.length;
    const points = nodes.map((node, i) => ({ id: node.node_id, x: 280 * Math.cos(2 * Math.PI * i / Math.max(count, 1)), y: 220 * Math.sin(2 * Math.PI * i / Math.max(count, 1)) }));
    const byId = new Map(points.map((p, i) => [p.id, i]));
    const links = edges.map(e => [byId.get(e.source), byId.get(e.target)]).filter(([a, b]) => a !== undefined && b !== undefined);
    const ideal = Math.sqrt(800 * 650 / Math.max(count, 1));
    for (let iteration = 0; iteration < 150; iteration++) {
      const forces = points.map(() => ({ x: 0, y: 0 }));
      for (let i = 0; i < count; i++) for (let j = i + 1; j < count; j++) {
        let dx = points[i].x - points[j].x, dy = points[i].y - points[j].y;
        const distance = Math.max(1, Math.hypot(dx, dy));
        if (dx === 0 && dy === 0) { dx = 0.1; dy = 0.1; }
        const magnitude = ideal * ideal / (distance * distance);
        forces[i].x += dx * magnitude; forces[i].y += dy * magnitude; forces[j].x -= dx * magnitude; forces[j].y -= dy * magnitude;
      }
      for (const [a, b] of links) {
        const dx = points[b].x - points[a].x, dy = points[b].y - points[a].y, distance = Math.max(1, Math.hypot(dx, dy)), magnitude = distance / ideal;
        forces[a].x += dx * magnitude; forces[a].y += dy * magnitude; forces[b].x -= dx * magnitude; forces[b].y -= dy * magnitude;
      }
      const temperature = 14 * (1 - iteration / 150) + 0.1;
      for (let i = 0; i < count; i++) {
        forces[i].x -= points[i].x * 0.03; forces[i].y -= points[i].y * 0.03;
        const length = Math.hypot(forces[i].x, forces[i].y) || 1;
        points[i].x += forces[i].x / length * Math.min(length, temperature);
        points[i].y += forces[i].y / length * Math.min(length, temperature);
      }
    }
    for (const p of points) state.positions.set(p.id, p);
  }
  /* 适配全图：按屏幕像素留边，上下避开浮在画面上的视图切换与状态条，左右给节点名称留位置；
     关系图视图隐藏时（尺寸为 0）记下待适配，显示出来后再算。 */
  const FIT_PAD = { top: 76, bottom: 76, left: 56, right: 56 };
  function fit() {
    const width = svg.clientWidth, height = svg.clientHeight;
    if (!width || !height) { state.needsFit = true; return; }
    state.needsFit = false;
    const points = [...state.positions.values()];
    if (!points.length) state.view = { x: 0, y: 0, w: 800, h: 600 };
    else {
      const xs = points.map(p => p.x), ys = points.map(p => p.y);
      const minX = Math.min(...xs), minY = Math.min(...ys), spanX = Math.max(Math.max(...xs) - minX, 1), spanY = Math.max(Math.max(...ys) - minY, 1);
      const innerW = Math.max(width - FIT_PAD.left - FIT_PAD.right, 40), innerH = Math.max(height - FIT_PAD.top - FIT_PAD.bottom, 40);
      // 节点很少时不放得过大：每单位最多 2 像素。
      const scale = Math.min(innerW / spanX, innerH / spanY, 2);
      const x = minX - FIT_PAD.left / scale - (innerW / scale - spanX) / 2, y = minY - FIT_PAD.top / scale - (innerH / scale - spanY) / 2;
      state.view = { x, y, w: width / scale, h: height / scale };
    }
    applyView();
  }
  function applyView() {
    const v = state.view;
    if (!v || !svg.clientWidth) return;
    svg.setAttribute('viewBox', `${v.x} ${v.y} ${v.w} ${v.h}`);
    const scale = Math.max(0.05, Math.min(svg.clientWidth / v.w, svg.clientHeight / v.h));
    for (const group of svg.querySelectorAll('.c-gnode')) {
      const passage = group.dataset.kind === 'passage', radius = (passage ? 3.5 : group.dataset.kind === 'region' ? 6 : 4) / scale;
      const shape = group.querySelector('.c-gnode-shape'), text = group.querySelector('text'), hit = group.querySelector('.c-gnode-hit');
      hit.setAttribute('x', -12 / scale); hit.setAttribute('y', -12 / scale); hit.setAttribute('width', 24 / scale); hit.setAttribute('height', 36 / scale);
      if (passage) shape.setAttribute('d', `M 0 ${-radius} L ${radius} 0 L 0 ${radius} L ${-radius} 0 Z`); else shape.setAttribute('r', radius);
      text.style.fontSize = (passage ? 10 : 11) / scale + 'px';
      text.style.strokeWidth = 3 / scale + 'px';
      text.setAttribute('y', passage ? -8 / scale : 18 / scale);
    }
  }
  function render() {
    if (mode() !== 'graph') return;
    const graph = state.graph;
    if (!graph) { svg.replaceChildren(); return; }
    layout(graph.nodes, graph.edges);
    const fragment = document.createDocumentFragment();
    // 搜索：名称、别名或编号不匹配的节点变淡（与原关系图面板相同）。
    const query = state.query.trim().toLowerCase();
    const matched = new Set(graph.nodes.filter(n => !query || (n.name + ' ' + (n.aliases || []).join(' ') + ' ' + (n.properties.display_code || '')).toLowerCase().includes(query)).map(n => n.node_id));
    const routeNodes = new Set([...(state.route?.region_ids || []), ...(state.route?.passages || []).map(p => p.node_id)]);
    for (const e of graph.edges) {
      const a = state.positions.get(e.source), b = state.positions.get(e.target);
      if (!a || !b) continue;
      fragment.append(el('line', { x1: a.x, y1: a.y, x2: b.x, y2: b.y, class: 'c-gedge', 'data-route': routeNodes.has(e.source) && routeNodes.has(e.target) }));
    }
    for (const n of graph.nodes) {
      const p = state.positions.get(n.node_id);
      if (!p) continue;
      const group = el('g', { transform: `translate(${p.x},${p.y})`, class: 'c-gnode', role: 'button', tabindex: 0,
        'aria-label': `${NODE_KINDS[n.kind]}：${n.name}`, 'aria-pressed': String(state.selected === n.node_id), 'data-kind': n.kind, 'data-node-id': n.node_id, 'data-dimmed': Boolean(query) && !matched.has(n.node_id) });
      const radius = n.kind === 'region' ? 17 : n.kind === 'passage' ? 6 : 9;
      group.append(el('rect', { class: 'c-gnode-hit', x: -12, y: -12, width: 24, height: 36, fill: 'transparent' }));
      group.append(n.kind === 'passage' ? el('path', { d: 'M 0 -7 L 7 0 L 0 7 L -7 0 Z', class: 'c-gnode-shape' }) : el('circle', { r: radius, class: 'c-gnode-shape' }));
      const text = el('text', { x: 0, y: n.kind === 'passage' ? -11 : radius + 17, 'text-anchor': 'middle' });
      const code = n.properties.display_code;
      const label = n.kind === 'passage' ? (state.selected === n.node_id ? code : '') : n.kind === 'region' ? (code + (n.properties.has_label ? ' ' + n.name : '')) : n.name;
      text.textContent = label.length > 18 ? label.slice(0, 17) + '…' : label;
      group.append(text);
      const title = el('title');
      title.textContent = n.name;
      group.append(title);
      group.addEventListener('click', () => select(n.node_id));
      group.addEventListener('keydown', event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); select(n.node_id); } });
      fragment.append(group);
    }
    svg.replaceChildren(fragment);
    if (!state.view || state.needsFit) fit(); else applyView();
  }
  function select(id) {
    const node = state.graph?.nodes.find(n => n.node_id === id);
    if (!node) return;
    state.selected = id;
    render();
    notifyLayer();
    onSelect(node);
    changed();
  }
  /* 二维“区域连接”叠加：通行连线为虚线，连接口为橙点；在查询的路径上的连线为蓝色。 */
  function draw2d(ctx, project) {
    if (!state.layer || !current()) return;
    const ratio = ctx.canvas.width / Math.max(ctx.canvas.getBoundingClientRect().width, 1), nodes = new Map(state.graph.nodes.map(n => [n.node_id, n]));
    const selected = new Set([...(state.route?.region_ids || []), ...(state.route?.passages || []).map(p => p.node_id)]);
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.lineWidth = 1.1 * ratio;
    ctx.setLineDash([4 * ratio, 4 * ratio]);
    for (const edge of state.graph.edges) {
      if (edge.kind !== 'access') continue;
      const a = nodes.get(edge.source)?.position, b = nodes.get(edge.target)?.position;
      if (!a || !b) continue;
      const p = project(a), q = project(b);
      ctx.strokeStyle = selected.has(edge.source) && selected.has(edge.target) ? '#4f8ccc' : '#707d87';
      ctx.globalAlpha = 0.7;
      ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    for (const n of nodes.values()) {
      if (n.kind !== 'passage' || !n.position) continue;
      const p = project(n.position);
      ctx.fillStyle = '#c68a3b';
      ctx.beginPath(); ctx.arc(p.x, p.y, 3.3 * ratio, 0, 2 * Math.PI); ctx.fill();
    }
    ctx.restore();
  }
  /* 生成关系图图像（整图或某节点附近），返回图像地址；关系图已换新时返回 null。 */
  async function exportImage(nodeId) {
    const version = state.graph?.graph_revision;
    if (!version) throw new Error('关系图还没有读取。');
    try {
      const result = await api.invoke('render_scene_graph_image', { ...(nodeId ? { node_id: nodeId } : {}), expected_graph_revision: version, width: 1280 });
      return version === state.graph?.graph_revision ? result.artifact.uri : null;
    } catch (error) { if (error.status === 409) schedule(); throw error; }
  }
  function contextChanged() {
    const b = binding(), key = bindingKey(b) + ':' + Boolean(b?.available);
    if (key === state.contextKey) return;
    state.contextKey = key;
    state.generation += 1;
    state.graph = null; state.route = null; state.selected = null; state.layoutKey = ''; state.error = '';
    render(); notifyLayer(); schedule(); changed();
  }

  svg.addEventListener('wheel', event => {
    if (!state.view) return;
    event.preventDefault();
    const factor = Math.exp(Math.max(-0.4, Math.min(0.4, event.deltaY * 0.0015))), v = state.view;
    if (v.w * factor < 40 || v.w * factor > 100000) return;
    state.view = { x: v.x + v.w * (1 - factor) / 2, y: v.y + v.h * (1 - factor) / 2, w: v.w * factor, h: v.h * factor };
    applyView();
  }, { passive: false });
  svg.addEventListener('pointerdown', event => {
    if (event.target.closest('.c-gnode') || !state.view) return;
    state.drag = { x: event.clientX, y: event.clientY, view: { ...state.view } };
    svg.setPointerCapture(event.pointerId);
  });
  svg.addEventListener('pointermove', event => {
    if (!state.drag) return;
    const rect = svg.getBoundingClientRect(), v = state.drag.view, factor = Math.max(v.w / rect.width, v.h / rect.height);
    state.view = { ...v, x: v.x - (event.clientX - state.drag.x) * factor, y: v.y - (event.clientY - state.drag.y) * factor };
    applyView();
  });
  svg.addEventListener('pointerup', () => { state.drag = null; });
  svg.addEventListener('pointercancel', () => { state.drag = null; });
  new ResizeObserver(() => { if (state.needsFit) fit(); else applyView(); }).observe(svg);
  window.addEventListener('pointcloud3d:ready', notifyLayer);
  window.addEventListener('pointcloud3d:scene-graph-select', event => select(event.detail?.node_id));
  stream.on('memory.scene_spaces', () => { state.generation += 1; state.route = null; state.error = ''; schedule(); });
  stream.on('memory.instances', () => { state.generation += 1; schedule(); });

  return {
    load, render, draw2d, fit, select, contextChanged, exportImage,
    /* 关系图修订冲突（409）后重新读取。 */
    reload: schedule,
    sync() { notifyLayer(); render(); if (interested()) void load(); },
    setLayer(value) { state.layer = value; this.sync(); },
    setLayout(value) { state.layout = value; render(); changed(); },
    setQuery(value) { state.query = value; render(); },
    setRoute(route) { state.route = route; render(); notifyLayer(); },
    onChange(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    get layer() { return state.layer; },
    get layout() { return state.layout; },
    get graph() { return state.graph; },
    get selected() { return state.selected; },
    get error() { return state.error; },
  };
}
