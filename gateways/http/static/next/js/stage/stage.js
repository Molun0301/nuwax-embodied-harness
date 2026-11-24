/* 舞台：地图占满主区域，控件贴边。左上视图切换；右上“图层与图例”与画中画视频；左下状态条（详情）；右下相机组；
   正上方操作提示；业务未运行时整块遮罩。地图核心、视频、语义区域、关系图、记忆物品分别在同目录模块里。 */
import { icon, h, popover, closePopovers, toast } from '/static/shared/neh-ui.js';
import { createMetrics, fmtRate, fmtDelay } from './metrics.js';
import { createVideo } from './video.js';
import { createMapView, PLANAR_PIPELINE, MAP_TOPICS } from './mapview.js';
import { createItems } from './items.js';
import { createRegions } from './regions.js';
import { createGraph } from './graph.js';
import { businessView, GEOMETRY_PHASES } from '../status.js';

const VIEWS = [['2d', '2D', 'grid-2x2', '二维栅格地图'], ['3d', '3D', 'box', '三维点云地图'], ['semantic', '语义地图', 'tag', '区域整块着色并编号'], ['graph', '关系图', 'network', '区域、连接口与物品的关系']]
  .filter(([id]) => !PLANAR_PIPELINE || id !== '3d');
const POINT_STYLES = [['height', '按高度着色'], ['full', '完整点云（加载全部原始扫描点）'], ['singleFloor', '单层（去掉天花板）'], ['ground', '只看地面'], ['structure', '只看结构'], ['mono', '单色']];
// 图例颜色与绘制一致（二维与三维共用叠加色）。
const LEGEND = [['轨迹', 'line', 'var(--stage-trajectory)'], ['全局路线', 'line', 'var(--stage-path)'], ['局部路线', 'line', 'var(--stage-local-path)'],
  ['保留的全局路线（等待执行更新）', 'line', 'var(--stage-retained-path)', 'retained'],
  ['探索前沿', 'dot', 'var(--stage-frontier)'], ['导航目标', 'dot', 'var(--stage-goal)'], ['物品', 'tag', '#f59e0b']];

/* root：舞台元素；onPick(hit)：点选物品、聚合、区域或关系图节点（P3 语义面板接上详情）。 */
export function createStage(root, ctx, { onPick }) {
  const { store, stream, go } = ctx;
  const narrow = () => window.innerWidth <= 760;
  const ui = { pipHidden: narrow(), pipLarge: false, swapped: false, detailOpen: false, layersOpen: false, shown: false };
  const visible = () => ui.shown && !document.hidden;

  /* ---------- 结构 ---------- */
  const canvas = h('canvas', { class: 'c-map2d', 'aria-label': '二维地图：滚轮缩放；自由视角下左键拖动旋转，中键或右键拖动平移' });
  const container3d = h('div', { class: 'c-map3d', hidden: true });
  const graphSvg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  graphSvg.setAttribute('class', 'c-graph');
  graphSvg.setAttribute('hidden', '');
  const mapLayer = h('div', { class: 'c-maplayer' }, canvas, container3d, graphSvg);
  const videoImage = h('img', { class: 'c-video-img', alt: '机器人摄像头画面', hidden: true });
  const videoOverlay = h('canvas', { class: 'c-video-det', hidden: true });
  const videoEmpty = h('div', { class: 'c-video-empty' });
  const videoMedia = h('div', { class: 'c-video' }, videoImage, videoOverlay, videoEmpty);
  const stageVideo = h('div', { class: 'c-stage-video', hidden: true });
  const emptyCard = h('div', { class: 'c-stage-empty', hidden: true });
  const viewSeg = h('div', { class: 'n-seg', role: 'group', 'aria-label': '地图视图' });
  const hintBox = h('div', { class: 'c-tc', hidden: true });
  const layersBtn = h('button', { class: 'n-float n-float-btn c-layers-btn', type: 'button', 'aria-expanded': 'false', html: icon('layers') }, '图层与图例', h('span', { html: icon('chevron-down', 'n-icon--sm') }));
  const layersPanel = h('div', { class: 'n-float c-layers', hidden: true, role: 'dialog', 'aria-label': '图层与图例' });
  const pip = h('div', { class: 'n-float c-pip' });
  const statusBar = h('div', { class: 'n-float c-status' });
  const statusDetail = h('div', { class: 'n-float c-status-detail', hidden: true });
  const camera = h('div', { class: 'n-float c-camera' });
  const halt = h('div', { class: 'c-halt', hidden: true });
  const hudBox = h('div', { class: 'c-bc', hidden: true });
  root.append(mapLayer, stageVideo, emptyCard,
    h('div', { class: 'c-tl' }, h('div', { class: 'n-float c-viewseg' }, viewSeg)),
    hintBox,
    h('div', { class: 'c-tr' }, layersBtn, layersPanel, pip),
    h('div', { class: 'c-bl' }, statusDetail, statusBar),
    h('div', { class: 'c-br' }, camera),
    hudBox,
    halt);

  /* ---------- 模块 ---------- */
  stream.want(MAP_TOPICS);
  if (!PLANAR_PIPELINE) stream.setParam('pointcloud_transport', 'pages');
  const metrics = createMetrics(store);
  let frame = null;
  const refreshSoon = () => { if (!frame) frame = requestAnimationFrame(() => { frame = null; render(); }); };
  const map = createMapView({ ctx, canvas, container3d, metrics, visible, onChange: refreshSoon, onPick });
  const mode = () => map.viewMode;
  const items = createItems({ stream, api: ctx.api, mapName: () => map.activeMapName(), highlight: () => map.highlight(), requestDraw: map.requestDraw });
  const regions = createRegions({ api: ctx.api, stream, binding: map.binding, mode, requestDraw: map.requestDraw });
  const graph = createGraph({ api: ctx.api, stream, svg: graphSvg, binding: map.binding, mode, requestDraw: map.requestDraw,
    onSelect: node => onPick({ type: 'node', node }) });
  map.attach({ regions, graph, items });
  items.onChange(refreshSoon);
  regions.onChange(refreshSoon);
  graph.onChange(refreshSoon);
  const video = createVideo({ stream, api: ctx.api, metrics, media: { root: videoMedia, image: videoImage, overlay: videoOverlay },
    visible: () => visible() && (!ui.pipHidden || ui.swapped), onChange: renderPip });
  void items.load();

  /* ---------- 视图切换 ---------- */
  function setView(next) {
    map.setViewMode(next);
    graphSvg.toggleAttribute('hidden', next !== 'graph');
    if (next === 'graph') graph.render();
    render();
  }
  // 按钮只建一次，刷新时只改按下状态（每秒刷新多次，重建会吞掉点击）。
  viewSeg.append(...VIEWS.map(([id, label, ic, title]) => h('button', { type: 'button', title, dataset: { view: id }, html: icon(ic), onclick: () => setView(id) }, label)));
  function renderViewSeg() {
    for (const button of viewSeg.children) button.setAttribute('aria-pressed', String(mode() === button.dataset.view));
  }

  /* ---------- 图层与图例（常驻面板：三维模块按编号绑定机器人外包轮廓开关与外观说明） ---------- */
  function switchRow(label, swatch, checked, onchange, attrs = {}) {
    const input = h('input', { type: 'checkbox', checked, ...attrs });
    input.addEventListener('change', () => onchange(input.checked));
    return h('label', { class: 'c-legend-row' }, swatch, h('span', {}, label), h('span', { class: 'n-switch' }, input, h('span', { class: 'n-switch-track' })));
  }
  const swatch = (kind, color) => h('span', { class: kind === 'line' ? 'c-line' : 'c-swatch', style: kind === 'line' ? `background:${color}`
    : kind === 'dot' ? `background:${color};border-radius:50%` : kind === 'tag' ? `background:none;box-shadow:inset 0 0 0 1.5px ${color}` : `background:${color}` });
  const styleSelect = h('select', { class: 'n-select', 'aria-label': '三维点云显示样式' }, POINT_STYLES.map(([value, label]) => h('option', { value }, label)));
  styleSelect.addEventListener('change', () => map.setPointStyle(styleSelect.value));
  const retainedRow = h('div', { class: 'c-legend-row c-legend-static', hidden: true });
  const styleSection = h('div', { class: 'c-legend-sec' }, h('div', { class: 'n-group-title' }, '点云样式'), styleSelect);
  layersPanel.append(
    h('div', { class: 'n-group-title' }, '显示'),
    switchRow('语义区域', swatch('box', '#4b9c86'), true, value => regions.setLayer(value)),
    switchRow('区域连接', swatch('line', 'repeating-linear-gradient(90deg,#707d87 0 5px,transparent 5px 9px)'), false, value => graph.setLayer(value)),
    switchRow('记忆物品', swatch('tag', '#94a3b8'), true, value => items.setVisible(value)),
    switchRow('画面检测框', swatch('tag', '#22d3ee'), true, value => video.setShowDetections(value)),
    h('div', { id: 'robotEnvelopeControl', hidden: true },
      switchRow('机器人外包轮廓', swatch('box', 'rgba(232,131,58,.35)'), true, () => {}, { id: 'robotEnvelopeToggle' })),
    h('div', { class: 'n-menu-sep' }),
    h('div', { class: 'n-group-title' }, '图例'),
    ...LEGEND.map(([label, kind, color, key]) => {
      const row = h('div', { class: 'c-legend-row c-legend-static' }, swatch(kind, color), h('span', {}, label));
      if (key === 'retained') { retainedRow.replaceChildren(...row.childNodes); return retainedRow; }
      return row;
    }),
    h('div', { class: 'c-legend-row c-legend-static' }, swatch('dot', '#fbbf24'), h('span', { id: 'robotGeometryLegend' }, '机器人')),
    styleSection);
  function setLayersOpen(open) {
    ui.layersOpen = open;
    layersPanel.hidden = !open;
    layersBtn.setAttribute('aria-expanded', String(open));
  }
  layersBtn.addEventListener('click', () => setLayersOpen(!ui.layersOpen));
  document.addEventListener('pointerdown', event => { if (ui.layersOpen && !layersPanel.contains(event.target) && !layersBtn.contains(event.target)) setLayersOpen(false); }, true);
  document.addEventListener('keydown', event => { if (event.key === 'Escape' && ui.layersOpen) setLayersOpen(false); });

  /* ---------- 画中画 ---------- */
  function swap() {
    ui.swapped = !ui.swapped;
    if (ui.swapped) { stageVideo.append(videoMedia); stageVideo.hidden = false; }
    else { stageVideo.hidden = true; root.prepend(mapLayer); }
    renderPip();
    map.refresh();
    video.refresh();
    render();
  }
  // 画中画的按钮与标签常驻，视频状态每半秒刷新时只改文字与状态。
  const pipBtn = (onclick) => h('button', { class: 'n-float-btn n-tip n-tip--below', type: 'button', onclick });
  const setTip = (button, ic, tip) => {
    if (button.dataset.tip === tip) return;
    button.dataset.tip = tip;
    button.setAttribute('aria-label', tip);
    button.innerHTML = icon(ic, 'n-icon--sm');
  };
  const pipDot = h('span', { class: 'n-dot' }), pipTitle = h('span', {});
  const swapBtn = pipBtn(swap);
  const sizeBtn = pipBtn(() => { ui.pipLarge = !ui.pipLarge; renderPip(); video.refresh(); map.refresh(); });
  const hideBtn = pipBtn(() => { if (ui.swapped) swap(); ui.pipHidden = true; renderPip(); });
  setTip(hideBtn, 'minus', '收起');
  const pipHead = h('div', { class: 'c-pip-head' }, h('div', { class: 'c-pip-title' }, pipDot, pipTitle), swapBtn, sizeBtn, hideBtn);
  const rateChip = h('span', { class: 'c-chip n-num', title: '实际显示帧率' });
  const delayChip = h('span', { class: 'c-chip n-num' });
  const recognize = h('button', { class: 'c-chip c-chip-toggle', type: 'button', onclick: () => video.togglePerception() }, h('i', { class: 'c-mini-switch' }), '识别');
  const pipFoot = h('div', { class: 'c-pip-foot' }, rateChip, delayChip, recognize);
  const pipMedia = h('div', { class: 'c-pip-media' }, videoMedia, pipFoot);
  const pipOpen = h('button', { class: 'n-float-btn', type: 'button', title: '展开摄像头画面', onclick: () => { ui.pipHidden = false; renderPip(); video.refresh(); } });
  function renderPip() {
    const v = video.summary();
    if (ui.pipHidden && !ui.swapped) {
      pip.className = 'n-float c-pip-collapsed';
      const label = v.mode === 'none' ? '摄像头无画面' : '摄像头';
      if (pipOpen.dataset.label !== label) { pipOpen.dataset.label = label; pipOpen.innerHTML = icon('camera'); pipOpen.append(label); }
      if (pip.firstChild !== pipOpen || pip.childNodes.length !== 1) pip.replaceChildren(pipOpen);
      return;
    }
    const empty = v.mode === 'none' && !ui.swapped;
    pip.className = 'n-float c-pip' + (ui.pipLarge ? ' is-large' : '') + (empty ? ' is-empty' : '');
    if (pip.firstChild !== pipHead) pip.replaceChildren(pipHead, pipMedia);
    pipDot.className = 'n-dot ' + (ui.swapped || v.mode === 'live' ? 'n-dot--ok' : v.mode === 'stale' ? 'n-dot--warn' : 'n-dot--off');
    pipTitle.textContent = ui.swapped ? '地图' : '摄像头画面';
    pipTitle.parentElement.title = v.cameraId ? '相机 ' + v.cameraId : '';
    swapBtn.hidden = v.mode === 'none' && !ui.swapped;
    setTip(swapBtn, 'arrow-left-right', ui.swapped ? '地图回到主画面' : '与地图互换，画面放到主区域');
    setTip(sizeBtn, ui.pipLarge ? 'minimize-2' : 'maximize-2', ui.pipLarge ? '缩小' : '放大');
    const slot = ui.swapped ? mapLayer : videoMedia;
    if (slot.parentElement !== pipMedia) pipMedia.prepend(slot);
    pipFoot.hidden = ui.swapped;
    rateChip.hidden = delayChip.hidden = v.mode === 'none';
    rateChip.textContent = fmtRate(v.rate);
    delayChip.textContent = '延迟 ' + fmtDelay(v.delay);
    delayChip.title = v.relayTime ? '从视频中转收到画面到显示' : '从画面采集到显示；还没估算出设备与浏览器的时钟差时显示 --';
    const perception = video.perception();
    recognize.setAttribute('aria-pressed', String(perception.on));
    recognize.disabled = perception.busy || !perception.known || (!perception.available && !perception.on);
    recognize.dataset.tone = perception.error ? 'bad' : '';
    recognize.title = !perception.available && !perception.on ? '视觉识别未开启：在设置的“视频与视觉识别”板块中开启，重启后可用'
      : perception.error || (perception.on ? '关闭持续识别（探索、跟随、找目标用到时仍会识别）' : '开启持续识别画面中的物体');
    videoEmpty.textContent = v.mode === 'stale' ? '画面已中断，保留上次画面' : v.message || '没有收到画面，正在等待视频连接';
    videoEmpty.title = v.detail;
    videoEmpty.hidden = v.mode === 'live';
    videoMedia.dataset.mode = v.mode;
  }

  /* ---------- 状态条 ---------- */
  function statusState() {
    const s = store.get();
    if (s.business !== 'running') return ['n-dot--bad', '业务未运行'];
    if (s.conn.status === 'lost') return ['n-dot--warn', '画面已暂停'];
    // 定位文字与圆点按地图会话阶段，与顶栏、地图卡同一判断（已加载但重定位未成功时位姿在 odom 系，不能算已定位）。
    const [label, tone] = GEOMETRY_PHASES[s.geometry.phase];
    return ['n-dot--' + tone, label];
  }
  const statusDot = h('span', { class: 'n-dot' }), statusName = h('b', {}), statusText = h('span', { class: 'n-muted' });
  const poseX = h('span', { class: 'n-num' }), poseY = h('span', { class: 'n-num' }), poseYaw = h('span', { class: 'n-num' });
  const poseSeg = h('div', { class: 'c-status-seg', 'data-optional': '', title: '机器人在地图坐标系中的位置（米）与朝向' },
    h('span', { class: 'c-k' }, 'x'), poseX, h('span', { class: 'c-k' }, 'y'), poseY, h('span', { class: 'c-k' }, '朝向'), poseYaw);
  const detailBtn = h('button', { class: 'n-float-btn c-status-more', type: 'button', title: '帧率、位姿延迟、点数、机器人外形、坐标系',
    onclick: () => { ui.detailOpen = !ui.detailOpen; render(); } });
  const layoutSeg = h('div', { class: 'n-seg c-status-seg-ctl', role: 'group', 'aria-label': '关系图布局' },
    [['relations', '按关系排布'], ['spatial', '按地图位置']].map(([id, label]) => h('button', { type: 'button', dataset: { layout: id }, onclick: () => graph.setLayout(id) }, label)));
  const graphSearch = h('input', { class: 'n-input c-graph-search', type: 'search', placeholder: '搜索节点', 'aria-label': '搜索关系图节点' });
  graphSearch.addEventListener('input', () => graph.setQuery(graphSearch.value));
  const layoutWrap = h('div', { class: 'c-status-seg', 'data-optional': '' }, layoutSeg, graphSearch);
  statusBar.append(h('div', { class: 'c-status-seg' }, statusDot, statusName, statusText), poseSeg, layoutWrap, detailBtn);
  function renderStatus(info) {
    const graphMode = mode() === 'graph';
    poseSeg.hidden = graphMode || !info.pose;
    layoutWrap.hidden = !graphMode;
    detailBtn.hidden = graphMode;
    if (graphMode) {
      const g = graph.graph;
      statusDot.className = 'n-dot ' + (g ? 'n-dot--ok' : 'n-dot--off');
      statusName.textContent = '关系图';
      statusText.textContent = g ? `${g.coverage?.region_count ?? 0} 个区域，${g.coverage?.passage_count ?? 0} 个连接口` + (g.truncated ? '（只显示前 500 个节点）' : '') : graph.error || '正在读取';
      for (const button of layoutSeg.children) button.setAttribute('aria-pressed', String(graph.layout === button.dataset.layout));
      statusDetail.hidden = true;
      return;
    }
    const [dot, text] = statusState();
    statusDot.className = 'n-dot ' + dot;
    statusName.textContent = info.mapName || text;
    statusText.textContent = info.mapName ? text : '';
    if (info.pose) {
      poseX.textContent = info.pose.x.toFixed(2);
      poseY.textContent = info.pose.y.toFixed(2);
      poseYaw.textContent = Number.isFinite(info.pose.yaw) ? Math.round(info.pose.yaw * 180 / Math.PI) + '°' : '--';
    }
    const detailLabel = ui.detailOpen ? '收起' : '详情';
    if (detailBtn.dataset.label !== detailLabel) {
      detailBtn.dataset.label = detailLabel;
      detailBtn.setAttribute('aria-expanded', String(ui.detailOpen));
      detailBtn.innerHTML = icon(ui.detailOpen ? 'chevron-down' : 'chevron-up', 'n-icon--sm');
      detailBtn.append(detailLabel);
    }
    statusDetail.hidden = !ui.detailOpen;
    if (!ui.detailOpen) return;
    const latency = info.poseLatency;
    const rows = [['地图帧率', fmtRate(info.rate), '每秒收到的新地图内容数'],
      ['位姿延迟', latency ? (latency.calibrated ? '≈' : '') + Math.round(latency.ms) + ' 毫秒' : '--', '当前显示的位姿从采集到画出所用时间；没有有效位姿或页面在后台时显示 --，≈ 表示已扣除估算的时钟差'],
      ...(mode() === '3d'
        ? [['显示点数', info.points ? info.points.displayed.toLocaleString('zh-CN') : '--'], ['地图点数', info.points ? info.points.source.toLocaleString('zh-CN') : '--'],
          ...(info.dense?.enabled ? [['完整点云', info.dense.error || (info.dense.loading ? '加载中 ' : '') + Number(info.dense.count || 0).toLocaleString('zh-CN') + ' / ' + Number(info.dense.total || 0).toLocaleString('zh-CN') + ' 点']] : [])]
        : [['栅格', info.grid ? `${info.grid.width} × ${info.grid.height}` : '--'], ['分辨率', info.grid ? info.grid.resolution.toFixed(2) + ' 米' : '--']]),
      ['坐标系', info.frameId]];
    statusDetail.replaceChildren(...rows.flatMap(([k, v, title]) => [h('span', { class: 'c-k', title: title || '' }, k), h('span', { class: 'n-num' }, v)]), robotModelRow);
  }
  // 三维模块按编号写入机器人外观加载说明，三维视图可见时显示。
  const robotModelRow = h('div', { id: 'robotModelStatusRow', class: 'c-status-wide', hidden: true }, h('span', { class: 'c-k' }, '机器人外形 '), h('span', { id: 'robotModelStatus', role: 'status' }, '正在加载机器人外观'));

  /* ---------- 相机组（按钮常驻，刷新时只改状态） ---------- */
  const camBtn = (ic, title, onclick) => h('button', { class: 'n-float-btn', type: 'button', title, 'aria-label': title, html: icon(ic), onclick });
  const followBtn = camBtn('locate-fixed', '', () => map.setCameraMode(map.cameraMode() === 'follow' ? 'free' : 'follow'));
  const centerBtn = camBtn('crosshair', '把机器人移到画面中心', () => map.centerRobot());
  const zoomValue = h('div', { class: 'c-zoom n-num' });
  const resetBtn = camBtn('scan', '', () => map.reset());
  const fitBtn = camBtn('scan', '适配全图', () => graph.fit());
  // 生成整张关系图图像：完成后轻提示里点“打开”在新标签页查看。
  const imageBtn = camBtn('download', '生成关系图图像', async () => {
    imageBtn.disabled = true;
    try {
      const uri = await graph.exportImage();
      if (uri) toast('关系图图像已生成', { action: { label: '打开', run: () => window.open(uri, '_blank', 'noopener') }, duration: 8000 });
    } catch (error) { toast('生成关系图图像失败：' + error.message, { tone: 'bad' }); }
    finally { imageBtn.disabled = false; }
  });
  const mapCamera = [followBtn, centerBtn, h('div', { class: 'n-float-sep c-camera-sep' }), camBtn('plus', '放大', () => map.zoom(1.25)), zoomValue,
    camBtn('minus', '缩小', () => map.zoom(0.8)), resetBtn];
  camera.append(...mapCamera, imageBtn, fitBtn);
  function renderCamera(info) {
    const graphMode = mode() === 'graph';
    for (const node of mapCamera) node.hidden = graphMode;
    fitBtn.hidden = imageBtn.hidden = !graphMode;
    if (graphMode) return;
    const follow = info.cameraMode === 'follow';
    followBtn.setAttribute('aria-pressed', String(follow));
    followBtn.title = follow ? '正在跟随机器人（以机器人为中心、车头朝上），点一下改为自由视角' : '跟随机器人：以机器人为中心、车头朝上';
    followBtn.disabled = !follow && !info.robotAvailable;
    centerBtn.disabled = follow || !info.robotAvailable;
    zoomValue.textContent = map.zoomPercent() + '%';
    resetBtn.title = follow ? '复位缩放' : '复位视角：适配完整地图';
  }

  /* ---------- 无地图提示与业务停止遮罩 ---------- */
  function renderEmpty(info) {
    let content = null;
    if (mode() === 'graph') {
      if (!graph.graph) content = ['network', '关系图还没有内容', graph.error || '当前地图的语义区域识别后，这里显示区域、连接口与物品的关系。'];
    } else if (!info.hasMap) {
      content = ['map', '还没有地图画面', (info.emptyMessage ? info.emptyMessage + ' ' : '') + '在“地图”页激活已有地图或新建地图。'];
    }
    emptyCard.hidden = !content || store.get().business !== 'running' || ui.swapped;
    if (!content) return;
    const key = content.join('|');
    if (emptyCard.dataset.key === key) return;
    emptyCard.dataset.key = key;
    emptyCard.replaceChildren(h('div', { class: 'n-float', html: icon(content[0]) }, h('div', { class: 'n-t3' }, content[1]), h('div', { class: 'n-meta' }, content[2]),
      mode() !== 'graph' ? h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => go('monitor', 'map', { open: true }) }, '打开地图页') : null));
  }
  function renderHalt() {
    const s = store.get();
    halt.hidden = s.business === 'running';
    if (halt.hidden) return;
    const view = businessView(s);
    const detail = s.business === 'updating' ? (s.manager.activeJob.action === 'stop' ? '地图、视频与机器人数据暂不可用，可在系统页重新启动。' : '地图、视频与机器人数据会在业务重新启动后恢复。')
      : s.business === 'failed' ? (s.manager.startFailure.reason || '') + (s.manager.startFailure.reason ? '。' : '') + '在系统页查看组件日志、回退或修改配置。'
      : '地图、视频与机器人数据暂不可用。在系统页启动业务服务，或查看原因。';
    halt.replaceChildren(h('div', { class: 'n-float c-halt-card', html: icon(view.icon) },
      h('div', { class: 'n-t2' }, view.title),
      h('div', { class: 'n-meta' }, detail),
      h('button', { class: 'n-btn n-btn--primary', type: 'button', onclick: () => go('system') }, view.actionLabel)));
  }

  /* ---------- 整体刷新 ---------- */
  function render() {
    const info = map.info();
    const m = mode();
    // 地图（2D、3D、语义地图、关系图）用舞台底色，随主题；视频在主画面时用深色。
    const dark = ui.swapped;
    root.dataset.tone = dark ? 'dark' : 'light';
    root.classList.toggle('n-tone-dark', dark);
    root.dataset.view = m;
    renderViewSeg();
    retainedRow.hidden = !info.retainedPath;
    styleSection.hidden = m !== '3d';
    if (styleSelect.value !== info.pointStyle) styleSelect.value = info.pointStyle;
    renderStatus(info);
    renderCamera(info);
    renderEmpty(info);
  }
  store.subscribe(['business', 'manager'], () => { renderHalt(); render(); });
  store.subscribe(['conn'], render);
  // 位姿、帧率每秒变化多次，状态条按 4 次每秒刷新。
  setInterval(() => { if (visible()) render(); }, 250);

  setView(PLANAR_PIPELINE ? '2d' : '3d');
  renderPip();
  renderHalt();

  return {
    map, items, regions, graph, video,
    /* 舞台正上方的操作提示：content 为元素，null 清除。 */
    hint(content) { hintBox.hidden = !content; hintBox.replaceChildren(...(content ? [content] : [])); },
    /* 舞台正下方的浮层（运动页的遥控面板）：content 为元素，null 清除。 */
    hud(content) { hudBox.hidden = !content; hudBox.replaceChildren(...(content ? [content] : [])); },
    /* 导航选点时舞台显示十字光标。 */
    setPicking(value) { root.dataset.picking = String(Boolean(value)); },
    /* 画面与地图互换（运动页“视频放到主画面”）。 */
    swapVideo() { if (!ui.swapped) { ui.pipHidden = false; swap(); } },
    canvas, mapLayer, root, setView,
    show() { ui.shown = true; map.refresh(); video.refresh(); render(); },
    hide() { ui.shown = false; closePopovers(); setLayersOpen(false); map.refresh(); },
    resize() { map.refresh(); video.refresh(); },
  };
}
