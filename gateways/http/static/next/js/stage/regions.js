/* 语义区域图层：从原控制台 semantic-map.js 迁出数据与绘制部分（读取 get_semantic_map 与语义后台状态、随
   memory.scene_spaces 更新、按当前地图绑定过滤、二维绘制与点选、三维图层），行为不变；区域列表、详情、命名与
   识别启停是 P3 语义面板的内容，这里只提供数据与选择接口。
   显示：语义地图视图里整块着色并写名称；2D、3D 视图里由图层“语义区域”控制（默认打开），画虚线轮廓与名称；
   语义面板打开期间始终显示，便于点选与核对。 */

const PALETTE = ['#4b9c86', '#648fb6', '#a18aba', '#b79461', '#719783', '#828eb4'];
const INACTIVE_COLOR = '#9aa4ab';
const CATEGORY_COLORS = { office: '#4b9c86', meeting_room: '#9580b0', corridor: '#648fb6', kitchen: '#b79461', storage: '#89916b' };
export const REGION_STATUS = { candidate: '候选', auto_labeled: '自动标签', reviewed: '已审核', corrected: '已修正', rejected: '已否定', stale: '已过期' };

/* binding()：当前地图绑定 {map_name, frame_id, map_revision, available}；mode()：当前视图；requestDraw()：请二维重画。 */
export function createRegions({ api, stream, binding, mode, requestDraw }) {
  const state = { data: null, service: null, selected: null, layer: true, panelOpen: false, paths: [], context: null, contextKey: '',
    request: null, generation: 0, streamVersion: 0, statusTimer: null, reloadTimer: null, error: '' };
  const listeners = new Set();
  const changed = () => { for (const fn of listeners) fn(); };
  const bindingKey = b => JSON.stringify([b?.map_name, b?.frame_id, b?.map_revision]);
  const scopeKey = b => JSON.stringify([b?.map_name, b?.frame_id]);
  const regions = () => Array.isArray(state.data?.regions) ? state.data.regions : [];
  // 有效区域：不是已过期、也不是已否定（只画这些）。编号与默认配色按有效区域从 1 起，舞台、列表、三维标签共用。
  const isActive = r => r.review_status !== 'stale' && r.review_status !== 'rejected';
  let activeCache = { data: null, list: [] };
  function active() {
    if (activeCache.data !== state.data) activeCache = { data: state.data, list: regions().filter(isActive) };
    return activeCache.list;
  }
  // 有效区域的编号（从 1 起）；过期与已否定的区域没有编号，返回 null。
  function number(region) { const index = active().indexOf(region); return index < 0 ? null : index + 1; }
  const visible = () => mode() === 'semantic' || (mode() !== 'graph' && state.layer) || state.panelOpen;
  const interested = () => visible();

  // 修订号也一致：命名等编辑要求区域与当前地图内容同一修订。
  function matchesCurrent() {
    const current = binding();
    // 请求暂时失败时保留同一地图的已读标注。
    if (!current?.available || !state.data?.map_binding) return false;
    const bound = state.data.map_binding;
    return ['map_name', 'frame_id', 'map_revision'].every(k => current[k] != null && current[k] === bound[k]);
  }
  /* 绘制只要求同一张地图、同一坐标系。地图内容更新时修订号随每个地图帧变化，新修订的区域要等重新读取才到；
     这期间照旧画已读的区域，新数据到达后一次替换（原来按修订号判断，每次地图更新都会先清空再出现，三维、二维一起闪）。 */
  function sameMap() {
    const current = binding();
    return Boolean(current?.available && state.data?.map_binding && scopeKey(current) === scopeKey(state.data.map_binding));
  }
  function drawable() {
    return sameMap() ? active().filter(r => r.frame_id === state.data.map_binding.frame_id && r.boundary?.frame_id === r.frame_id) : [];
  }
  // 过期与已否定的区域不上地图，列表里用灰色。
  function color(region) {
    if (!isActive(region)) return INACTIVE_COLOR;
    return CATEGORY_COLORS[region.metadata?.category] || PALETTE[(number(region) - 1) % PALETTE.length];
  }
  function emitLayer() {
    state.paths = [];
    const list = visible() ? drawable().map(region => ({ ...region, display_color: color(region), display_index: number(region) })) : [];
    const style = getComputedStyle(document.documentElement);
    window.dispatchEvent(new CustomEvent('pointcloud3d:semantic-regions', { detail: {
      regions: list, binding: state.data?.map_binding || null, visible: mode() === '3d' && visible(), selectedId: state.selected,
      theme: { surface: style.getPropertyValue('--surface').trim(), ink: style.getPropertyValue('--ink').trim() },
    } }));
    requestDraw();
  }
  function accept(data) {
    const current = binding();
    if (!data?.map_binding || !Array.isArray(data.regions)) return false;
    if (current?.map_name && data.map_binding.map_name !== current.map_name) return false;
    if (state.data && scopeKey(state.data.map_binding) !== scopeKey(data.map_binding)) state.selected = null;
    state.data = data;
    state.error = '';
    if (!regions().some(r => r.scene_space_id === state.selected)) state.selected = null;
    emitLayer();
    changed();
    return true;
  }
  async function load() {
    // 没有地图时没有区域可显示，不读取。
    if (state.request || !binding()?.available) return state.request;
    const generation = state.generation, version = state.streamVersion, scope = scopeKey(binding());
    state.request = (async () => {
      const results = await Promise.allSettled([api.invoke('get_scene_mapping_status'), api.invoke('get_semantic_map')]);
      // 只在换了地图时作废；同一地图内容更新期间读到的结果照常采用（设备忙、地图更新快时不会一直读不到）。
      if (scopeKey(binding()) !== scope) return;
      if (results[0].status === 'fulfilled') state.service = results[0].value;
      if (results[1].status === 'fulfilled' && version === state.streamVersion) {
        if (!accept(results[1].value)) { state.error = '地图已切换，等待对应语义记录。'; emitLayer(); }
      } else if (results[1].status === 'rejected') { state.error = results[1].reason.message; emitLayer(); }
      changed();
    })().finally(() => {
      state.request = null;
      scheduleStatus();
      if (generation !== state.generation && interested()) { clearTimeout(state.reloadTimer); state.reloadTimer = setTimeout(load, 100); }
    });
    return state.request;
  }
  // 语义后台运行时每 10 秒重读一次；页面在后台时延后。
  function scheduleStatus() {
    clearTimeout(state.statusTimer);
    if (interested() && (state.service?.running || state.service?.busy)) {
      state.statusTimer = setTimeout(() => { if (document.visibilityState !== 'hidden') load(); else scheduleStatus(); }, 10000);
    }
  }
  function contextChanged() {
    const current = binding();
    const key = bindingKey(current) + ':' + Boolean(current?.available);
    if (key === state.contextKey) return;
    if (state.context && scopeKey(state.context) !== scopeKey(current)) { state.selected = null; state.data = null; }
    state.context = current;
    state.contextKey = key;
    state.generation += 1;
    emitLayer();
    changed();
    clearTimeout(state.reloadTimer);
    if (interested()) state.reloadTimer = setTimeout(load, 600);
  }
  /* 二维绘制：语义地图视图整块着色（0.34），其它视图淡填充（0.14）加虚线轮廓；名称避让重叠，选中的优先。 */
  function draw2d(ctx, project) {
    state.paths = [];
    if (!visible() || !sameMap()) return;
    const semantic = mode() === 'semantic';
    const ratio = ctx.canvas.width / Math.max(ctx.canvas.getBoundingClientRect().width, 1);
    const style = getComputedStyle(document.documentElement);
    const ink = style.getPropertyValue('--ink').trim(), surface = style.getPropertyValue('--surface').trim();
    const labelRects = [], labelItems = [];
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    for (const r of drawable()) {
      const polygon = r.boundary?.polygon_points;
      if (!polygon?.length) continue;
      const path = new Path2D();
      const ring = points => { points.forEach((p, i) => { const q = project({ x: p.x ?? p[0], y: p.y ?? p[1] }); if (i === 0) path.moveTo(q.x, q.y); else path.lineTo(q.x, q.y); }); path.closePath(); };
      ring(polygon);
      for (const hole of r.boundary.metadata?.holes || []) ring(hole);
      const selected = r.scene_space_id === state.selected;
      ctx.fillStyle = color(r);
      ctx.globalAlpha = semantic ? 0.34 : 0.14;
      ctx.fill(path, 'evenodd');
      ctx.globalAlpha = 1;
      ctx.strokeStyle = selected ? ink : color(r);
      ctx.lineWidth = (selected ? 2.6 : 1.2) * ratio;
      ctx.setLineDash(semantic || selected ? [] : [4 * ratio, 3 * ratio]);
      ctx.stroke(path);
      ctx.setLineDash([]);
      state.paths.push({ path, region: r });
      const center = r.boundary.centroid?.position;
      if (center) labelItems.push({ r, center: project(center), selected });
    }
    labelItems.sort((a, b) => Number(b.selected) - Number(a.selected));
    ctx.font = `600 ${11.5 * ratio}px ${style.getPropertyValue('--font')}`;
    for (const item of labelItems) {
      const index = number(item.r);
      let text = item.r.primary_label || String(index);
      if (!item.selected && !semantic && !item.r.primary_label) text = String(index);
      if (text.length > 14) text = text.slice(0, 13) + '…';
      const w = ctx.measureText(text).width + 10 * ratio, h = 19 * ratio, x = item.center.x - w / 2, y = item.center.y - h / 2;
      if (x + w < 0 || y + h < 0 || x > ctx.canvas.width || y > ctx.canvas.height) continue;
      if (!item.selected && labelRects.some(b => x < b.x + b.w + 5 * ratio && x + w + 5 * ratio > b.x && y < b.y + b.h + 4 * ratio && y + h + 4 * ratio > b.y)) continue;
      labelRects.push({ x, y, w, h });
      ctx.fillStyle = surface;
      ctx.globalAlpha = 0.88;
      ctx.fillRect(x, y, w, h);
      ctx.globalAlpha = 1;
      ctx.fillStyle = ink;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, item.center.x, item.center.y);
    }
    ctx.restore();
  }
  /* 点选：返回命中的区域（在 draw2d 记下的路径里找）。 */
  function hitAt(ctx, x, y) {
    if (!visible() || !sameMap()) return null;
    return state.paths.find(p => ctx.isPointInPath(p.path, x, y, 'evenodd'))?.region || null;
  }
  function select(id) {
    if (id !== null && !regions().some(r => r.scene_space_id === id)) return;
    state.selected = id;
    emitLayer();
    changed();
  }

  // 同一主题有两种载荷：语义建图的地图快照（带 map_binding 与 regions）被采用；记忆服务的记录快照不含绑定，不采用，
  // 也不让正在进行的读取作废。
  stream.on('memory.scene_spaces', frame => { if (accept(frame.payload)) state.streamVersion += 1; });
  window.addEventListener('pointcloud3d:ready', emitLayer);
  new MutationObserver(emitLayer).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

  return {
    load, draw2d, hitAt, select, contextChanged, emitLayer,
    /* 视图或图层开关变化后调用：刷新图层并在需要时读取。 */
    sync() { emitLayer(); if (interested()) void load(); },
    setLayer(value) { state.layer = value; this.sync(); },
    setPanelOpen(value) { state.panelOpen = value; this.sync(); },
    /* 启停持续识别后写入后台返回的状态（与原控制台相同，不再另读一次）。 */
    setService(value) { state.service = value; scheduleStatus(); changed(); },
    onChange(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    color, number,
    get layer() { return state.layer; },
    get regions() { return regions(); },
    /* 有效区域（不含已过期、已否定），顺序即编号顺序。 */
    get active() { return active(); },
    get selected() { return state.selected; },
    get service() { return state.service; },
    get error() { return state.error; },
    get matches() { return matchesCurrent(); },
  };
}
