/* 语义面板：区域、物品两页，以及区域详情、物品详情、聚合物品列表、关系图节点详情。
   区域从 semantic-map.js 迁出：识别启停（start/stop_scene_mapping）、识别当前地图（refresh_semantic_map，可选采用达标名称，
   每 3 秒查询任务结果）、按修订命名（update_scene_space_label，409 重读）、识别候选与观察证据；数据与图层在舞台 regions。
   物品从 console.js 迁出：按空间证据筛选与搜索、记忆摘要、详情字段、在地图中查看（二维自由视角居中并高亮）、
   走到它附近（resolve_navigation_candidates 取本物品的观察位置后导航）；数据与标记在舞台 items。
   节点详情从 scene-graph.js 迁出：get_scene_graph_node、关联节点、设为起点与查询路径（plan_scene_graph_route）、局部图像。
   舞台点选物品、聚合、区域与节点时由监控页调用 show({pick}) 打开对应详情。 */
import { h, icon, toast } from '/static/shared/neh-ui.js';
import { REGION_STATUS } from '../stage/regions.js';
import { NODE_KINDS } from '../stage/graph.js';
import { instanceName, observationOnly, instanceQuality, instanceExtent } from '../stage/items.js';
import { planarYawFromPose } from '../stage/mapview.js';

const TASK_TERMINAL = new Set(['succeeded', 'failed', 'cancelled', 'timeout']);
const FILTERS = [['all', '全部'], ['located', '有位置'], ['observation', '仅观察'], ['uncertain', '待确认']];
const METHOD_NAMES = { support_points_map: '地图支持点', support_points_camera: '相机支持点', explicit_map_pose: '显式地图位置', track_pose: '跟踪位置',
  detection3d_pose: '三维检测位置', map_point_samples: '地图支持点采样', camera_point_samples: '相机支持点采样', depth_camera_projection: '相机深度投影',
  depth_pose_fallback: '深度与观察位姿估计', delayed_pointcloud_projection: '延后点云投影', current_pose_fallback: '观察者位置回退', explicit_position: '显式空间位置',
  pointcloud_projection: '点云投影', pointcloud_depth: '点云深度', depth_projection: '深度投影', ray_triangulation: '多视角三角化',
  deferred_pointcloud_refine: '延后点云修正', tracked_position: '跟踪空间位置' };
const finite = point => Boolean(point && [point.x, point.y, point.z].every(v => typeof v === 'number' && Number.isFinite(v)));
// 换内容时略过条件为假的子元素（replaceChildren 会把 null 写成文字）。
const fill = (el, ...kids) => el.replaceChildren(...kids.filter(kid => kid != null && kid !== false));
function formatLastSeen(ts) {
  if (ts === null || ts === undefined || ts === '') return '未记录';
  const d = new Date(typeof ts === 'number' && ts < 1e12 ? ts * 1000 : ts);
  return Number.isFinite(d.getTime()) ? d.toLocaleString('zh-CN', { year: 'numeric', month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', hour12: false }) : '时间无效';
}
function extentText(inst) {
  const extent = instanceExtent(inst);
  if (!extent) return '尺寸未知（仅位置标记）';
  return ['x', 'y', 'z'].map(axis => extent.size_m[axis].toFixed(2)).join(' × ') + ' 米，' + (extent.source === 'observed_points' ? '可见支持点范围，遮挡部分未知' : '三维检测尺寸估计');
}
function inside(point, polygon) {
  let hit = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[i], b = polygon[j];
    if ((a.y > point.y) !== (b.y > point.y) && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) hit = !hit;
  }
  return hit;
}

export function mountSemantic(root, ctx) {
  const { api, stage, go } = ctx;
  const { regions, items, graph, map } = stage;
  const state = { shown: false, tab: 'regions', page: 'list', regionId: null, itemId: null, cluster: null, nodeId: null, nodeDetail: null, nodeToken: 0,
    from: null, busy: false, task: null, taskTimer: null, filter: 'all', pageKey: '', listKey: '', detailKey: '', summary: { meta: null, error: '', loading: false, instances: null, fetchedAt: 0 } };
  const view = h('div');
  root.append(view);
  const status = h('p', { class: 'c-status-line', role: 'status' });
  // derived：状态行当前显示的是读取失败或后台错误（随数据更新），操作结果会覆盖它。
  let derived = false;
  function say(text, level = '') { derived = false; status.textContent = text; status.dataset.level = level; }
  const regionList = () => regions.regions;
  const regionName = (r, index = regionList().indexOf(r)) => r.primary_label || '区域 ' + (index + 1);
  const regionStatus = r => r.metadata?.label_locked ? '已命名' : REGION_STATUS[r.review_status] || '未知';
  const regionBadge = r => (r.metadata?.label_locked || ['reviewed', 'corrected'].includes(r.review_status) ? 'n-badge--ok'
    : r.review_status === 'auto_labeled' ? 'n-badge--accent' : r.review_status === 'stale' ? 'n-badge--warn' : '');
  const scopeMatches = inst => { const current = map.activeMapName(); return Boolean(current && inst.map_name && current === inst.map_name); };
  const back = (label, onclick) => h('button', { class: 'c-back', type: 'button', html: icon('chevron-left', 'n-icon--sm'), onclick }, label);
  const chip = r => h('span', { class: 'c-regnum', style: 'background:' + regions.color(r) }, regionList().indexOf(r) + 1);

  /* ---------- 页面切换 ---------- */
  function clearSelection() {
    if (state.page === 'region') regions.select(null);
    if (state.page === 'item') {
      items.select(null);
      const target = map.highlight();
      if (target && target.record_id === state.itemId && !target.target_pose) map.setHighlight(null);
    }
  }
  function open(page, fields = {}) {
    clearSelection();
    Object.assign(state, { page, ...fields });
    state.pageKey = '';
    if (!state.task) say('');
    if (page === 'region') regions.select(state.regionId);
    if (page === 'item') {
      const inst = items.instances.find(item => item.instance_id === state.itemId);
      items.select(state.itemId);
      if (inst) map.setHighlight({ record_id: inst.instance_id, target_name: instanceName(inst) });
    }
    if (page === 'node') loadNode(state.nodeId);
    render();
  }
  const toList = tab => open('list', { tab: tab || state.tab, regionId: null, itemId: null, cluster: null, nodeId: null, nodeDetail: null });

  /* ---------- 列表页（骨架只在换页时建一次，数据变化只改列表） ---------- */
  const regionsTab = h('button', { type: 'button', onclick: () => toList('regions') });
  const itemsTab = h('button', { type: 'button', onclick: () => toList('items') });
  const seg = h('div', { class: 'n-seg n-seg--block c-gap-b' }, regionsTab, itemsTab);
  const regionSearch = h('input', { class: 'n-input', type: 'search', placeholder: '搜索区域' });
  const itemSearch = h('input', { class: 'n-input', type: 'search', placeholder: '搜索名称、标签或编号' });
  const listBox = h('div', { class: 'n-list' });
  regionSearch.addEventListener('input', () => { state.listKey = ''; renderList(); });
  itemSearch.addEventListener('input', () => { state.listKey = ''; renderList(); });
  // 区域识别卡
  const recogSub = h('div', { class: 'n-meta' });
  const runningBox = h('input', { type: 'checkbox' });
  const applyBox = h('input', { type: 'checkbox' });
  const refreshBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', title: '重新提取当前地图的区域并识别名称；不会让机器人移动' }, '重新划分区域');
  const recogCard = h('div', { class: 'c-card c-gap-b' },
    h('div', { class: 'c-card-head' }, h('div', { class: 'c-hero-main' }, h('div', { class: 'c-card-title c-card-title--sm' }, '区域识别'), recogSub),
      h('label', { class: 'n-switch', title: '机器人移动时持续补充观察并给区域命名' }, runningBox, h('span', { class: 'n-switch-track' }), h('span', { class: 'n-meta' }, '持续识别'))),
    h('div', { class: 'c-card-actions' }, refreshBtn, h('label', { class: 'n-check', title: '本次识别采用符合门槛的名称' }, applyBox, '达到门槛的名称自动采用')));
  // 物品筛选与摘要
  const filterSeg = h('div', { class: 'n-seg c-gap-b' }, FILTERS.map(([key, label]) => h('button', { type: 'button', dataset: { filter: key },
    onclick: () => { state.filter = key; state.listKey = ''; renderList(); } }, label)));
  const itemStatus = h('p', { class: 'c-status-line', role: 'status' });
  const itemSummary = h('p', { class: 'n-meta' });
  const itemRefresh = h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', title: '手动刷新当前记忆库物品', onclick: () => { void items.load(); renderList(); } }, '刷新');

  function action(fn) {
    if (state.busy) return;
    state.busy = true; renderList();
    Promise.resolve().then(fn).catch(error => say(error.message || '操作未完成。', 'error'))
      .finally(() => { state.busy = false; state.listKey = ''; state.detailKey = ''; render(); });
  }
  runningBox.addEventListener('change', () => {
    const start = runningBox.checked;
    action(async () => {
      const service = await api.invoke(start ? 'start_scene_mapping' : 'stop_scene_mapping');
      regions.setService(service);
      say(start ? '持续识别已启动，采用策略沿当前设置。' : service.stop_pending ? '正在停止，等待在途识别退出。' : '持续识别已停止。');
    });
  });
  refreshBtn.addEventListener('click', () => action(async () => {
    const result = await api.invoke('refresh_semantic_map', { apply_labels: applyBox.checked });
    state.task = result.task_id || result.task?.task_id;
    if (!state.task) throw new Error('服务未返回任务标识。');
    say('更新请求已受理，等待识别结果。');
    pollTask();
  }));
  function pollTask() {
    clearTimeout(state.taskTimer);
    if (!state.task) return;
    state.taskTimer = setTimeout(async () => {
      if (document.visibilityState === 'hidden') { pollTask(); return; }
      try {
        const task = await api.invoke('get_task_status', { task_id: state.task });
        const taskState = task.status?.state || task.state;
        if (TASK_TERMINAL.has(taskState)) {
          state.task = null;
          await regions.load();
          const failed = task.result?.failures?.length || 0;
          say(taskState === 'succeeded' ? (failed ? `${failed} 个区域未完成识别，请查看来源与证据。` : '语义地图已更新。')
            : (task.error_payload?.message || task.error || '任务未完成。'), taskState !== 'succeeded' || failed > 0 ? 'error' : '');
          state.listKey = ''; renderList();
        } else { say(task.status?.message || '正在更新识别…'); pollTask(); }
      } catch (error) { say('暂未读到任务结果，将继续查询同一任务。', 'error'); pollTask(); }
    }, 3000);
  }
  function regionServiceText() {
    const service = regions.service;
    if (!service) return '';
    if (service.enabled === false) return '自动识别未启用，可在设置“记忆与语义”中启用';
    if (service.running) return '持续识别中，' + (service.auto_apply ? '达标名称自动采用' : '名称保留为候选');
    return service.busy ? '正在识别' : '持续识别已停止';
  }
  function renderRecog() {
    const list = regionList(), named = list.filter(r => r.primary_label).length;
    recogSub.textContent = `已命名 ${named} 个，未命名 ${list.length - named} 个` + (regionServiceText() ? '；' + regionServiceText() : '');
    const service = regions.service;
    runningBox.checked = Boolean(service?.running || service?.busy);
    runningBox.disabled = state.busy || service?.enabled === false;
    refreshBtn.disabled = state.busy || Boolean(state.task) || service?.enabled === false;
    applyBox.disabled = state.busy || Boolean(state.task);
    // 读取失败或后台最近一次识别失败写在状态行；没有其它提示时才显示。
    const problem = regions.error || (service?.last_error ? service.last_error.message || service.last_error.code || '最近一次识别未完成。' : '');
    if (!state.task && (derived || !status.textContent)) { status.textContent = problem; status.dataset.level = problem ? 'error' : ''; derived = true; }
  }
  function itemStatusText() {
    const meta = items.meta, count = items.instances.length, library = meta.library_name || meta.active_library_name;
    const suffix = items.fetchedAt ? '，更新于 ' + new Date(items.fetchedAt).toLocaleTimeString('zh-CN', { hour12: false }) : '';
    const mismatch = count > 0 && !items.instances.some(scopeMatches);
    itemStatus.dataset.level = items.error ? 'error' : '';
    itemStatus.textContent = items.error ? `刷新失败：${items.error}。${count ? '正在查看上次缓存，地图标记已隐藏。' : '请重试。'}`
      : items.loading ? '正在读取当前记忆库…'
      : !library && !count ? '当前没有地图记忆库；开始建图或加载地图后自动打开。'
      : `${library ? '记忆库「' + library + '」' : '当前记忆库'}，实时更新${suffix}${mismatch ? '；与当前地图不匹配，标记已隐藏' : ''}`;
    const located = items.instances.filter(inst => instanceQuality(inst).key === 'located').length;
    const observations = items.instances.filter(observationOnly).length;
    const s = state.summary;
    const index = s.error ? '记忆摘要读取失败：' + s.error : s.loading ? '记忆摘要读取中…'
      : s.meta ? '已存向量 ' + (Number.isSafeInteger(s.meta.vector_count) && s.meta.vector_count >= 0 ? s.meta.vector_count : '—') + (s.meta.embedding_error ? '，向量计算失败：' + s.meta.embedding_error : '') : '';
    itemSummary.textContent = `${count} 条记录，${located} 个有位置，${observations} 条仅观察${meta.truncated ? `，已载入 ${count}/${meta.total} 条` : ''}` + (index ? '。' + index : '');
  }
  /* 记忆摘要（向量数）：物品记录换新后重读，库名与当前记忆库不一致时不采用。 */
  async function loadSummary() {
    const s = state.summary;
    if (s.loading || s.instances === items.instances) return;
    const library = items.meta.library_name || items.meta.active_library_name;
    if (!library) return;
    s.instances = items.instances; s.loading = true; s.error = '';
    const controller = new AbortController(), timer = setTimeout(() => controller.abort(), 8000);
    try {
      const body = await api.json('/api/memory/summary', { signal: controller.signal });
      const meta = body?.memory_summary?.metadata;
      if (meta && meta.active_library_name !== library) throw new Error('记忆摘要库名不匹配，请刷新');
      s.meta = meta || null;
    } catch (error) { s.error = error.name === 'AbortError' ? '读取超时' : error.message; }
    finally { clearTimeout(timer); s.loading = false; if (state.shown && state.page === 'list' && state.tab === 'items') itemStatusText(); }
  }
  function regionRow(r) {
    return h('button', { class: 'n-row n-row--hover', type: 'button', dataset: { regionId: r.scene_space_id }, onclick: () => open('region', { regionId: r.scene_space_id }) },
      chip(r),
      h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' + (r.primary_label ? '' : ' c-unnamed') }, r.primary_label || '未命名区域'),
        h('div', { class: 'n-row-sub n-num' }, r.boundary?.area_m2 == null ? '面积未知' : r.boundary.area_m2.toFixed(1) + ' 平方米')),
      h('span', { class: 'n-badge ' + regionBadge(r) }, regionStatus(r)), h('span', { class: 'c-rowchev', html: icon('chevron-right', 'n-icon--sm') }));
  }
  function itemRow(inst) {
    const quality = instanceQuality(inst);
    const observations = Number.isFinite(inst.observation_count) ? `${inst.observation_count} 次观察，` : '';
    const sub = `${inst.label || '未分类'}，${observations}最后见于 ${formatLastSeen(inst.last_seen_ts)}`;
    return h('button', { class: 'n-row n-row--hover', type: 'button', title: sub, dataset: { instanceId: inst.instance_id }, onclick: () => open('item', { itemId: inst.instance_id }) },
      h('span', { class: 'c-row-icon', dataset: { quality: quality.key }, html: icon(quality.key === 'observation' ? 'eye' : quality.key === 'located' ? 'map-pin' : 'circle-dashed', 'n-icon--sm') }),
      h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, instanceName(inst)), h('div', { class: 'n-row-sub' }, sub)),
      h('span', { class: 'n-badge' + (quality.key === 'uncertain' ? ' n-badge--warn' : quality.key === 'located' ? ' n-badge--ok' : '') }, quality.label),
      h('span', { class: 'c-rowchev', html: icon('chevron-right', 'n-icon--sm') }));
  }
  function renderList() {
    if (state.page !== 'list' && state.page !== 'cluster') return;
    regionsTab.setAttribute('aria-pressed', String(state.tab === 'regions'));
    itemsTab.setAttribute('aria-pressed', String(state.tab === 'items'));
    regionsTab.replaceChildren('区域', h('span', { class: 'n-count' }, regionList().length));
    itemsTab.replaceChildren('物品', h('span', { class: 'n-count' }, items.instances.length));
    if (state.page === 'cluster') {
      listBox.replaceChildren(...state.cluster.members.map(itemRow));
      return;
    }
    if (state.tab === 'regions') {
      renderRecog();
      const query = regionSearch.value.trim().toLowerCase();
      const key = JSON.stringify([regionList().map(r => [r.scene_space_id, r.primary_label, r.review_status, r.metadata?.label_locked, regions.color(r)]), query, regions.matches]);
      if (key === state.listKey) return;
      state.listKey = key;
      const rows = regionList().filter((r, index) => !query || regionName(r, index).toLowerCase().includes(query)).map(regionRow);
      listBox.replaceChildren(...(rows.length ? rows : [h('p', { class: 'n-meta' }, query ? '没有匹配的区域。' : '当前地图还没有语义区域。打开持续识别，或点“重新划分区域”。')]));
      return;
    }
    itemStatusText();
    for (const button of filterSeg.children) button.setAttribute('aria-pressed', String(state.filter === button.dataset.filter));
    void loadSummary();
    const query = itemSearch.value.trim().toLocaleLowerCase();
    const key = JSON.stringify([items.instances.length, items.fetchedAt, items.error, query, state.filter]);
    if (key === state.listKey) return;
    state.listKey = key;
    const list = items.instances.filter(inst => {
      const text = `${instanceName(inst)} ${inst.label || ''} ${inst.instance_id}`.toLocaleLowerCase();
      return (!query || text.includes(query)) && (state.filter === 'all' || instanceQuality(inst).key === state.filter);
    });
    listBox.replaceChildren(...(list.length ? list.map(itemRow) : [h('p', { class: 'n-meta' }, items.instances.length ? '没有匹配的记忆，试试其它名称或证据类型。' : '当前记忆库暂无物品。完成场景观察并写入记忆后，可在这里查看。')]));
  }

  /* ---------- 区域详情 ---------- */
  function renderRegion() {
    const r = regionList().find(item => item.scene_space_id === state.regionId);
    if (!r) { toList('regions'); return; }
    const key = JSON.stringify([r.scene_space_id, r.metadata?.scene_revision, r.primary_label, r.review_status, r.metadata?.label_locked, state.busy, regions.matches]);
    if (key === state.detailKey) return;
    const old = view.querySelector('form input');
    const draft = old && view.dataset.regionId === r.scene_space_id && view.dataset.revision === String(r.metadata?.scene_revision) ? old.value : undefined;
    const focused = old && document.activeElement === old;
    state.detailKey = key;
    view.dataset.regionId = r.scene_space_id;
    view.dataset.revision = String(r.metadata?.scene_revision);
    const index = regionList().indexOf(r);
    const input = h('input', { class: 'n-input', type: 'text', maxlength: 120, required: true, placeholder: '为区域命名', 'aria-label': '区域名称', value: draft ?? r.primary_label ?? '' });
    const save = h('button', { class: 'n-btn', type: 'submit', disabled: state.busy || !regions.matches || ['stale', 'rejected'].includes(r.review_status) }, '保存名称');
    const form = h('form', { class: 'c-inline-form' }, input, save);
    form.addEventListener('submit', event => {
      event.preventDefault();
      const value = input.value.trim();
      if (!value) return;
      action(async () => {
        try {
          await api.invoke('update_scene_space_label', { scene_space_id: r.scene_space_id, map_name: r.map_name, expected_revision: r.metadata.scene_revision, label: value });
          await regions.load();
          say('名称已保存，三个视图共用此标注。');
        } catch (error) {
          if (error.status === 409) { await regions.load(); throw new Error('地图或区域已改变，已重新读取。请核对后再次保存。'); }
          throw error;
        }
      });
    });
    const goBtn = h('button', { class: 'n-btn n-btn--sm n-btn--primary', type: 'button', html: icon('navigation', 'n-icon--sm'), disabled: !r.primary_label,
      title: r.primary_label ? '按名称导航到这个区域，到达后连拍核对' : '未命名的区域不能按名称导航，先在下面命名',
      onclick: async () => {
        try {
          await api.invoke('navigate_to_named_location', { target_name: r.primary_label, verify_on_arrival: true });
          toast('已开始导航到「' + r.primary_label + '」');
          go('monitor', 'activity', { open: true });
        } catch (error) { say('导航失败：' + error.message, 'error'); }
      } }, '导航到这里');
    const focusBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('locate-fixed', 'n-icon--sm'), onclick: () => {
      if (map.viewMode === 'graph') stage.setView('2d');
      window.__nuwaxMapFocusRegion?.(r);
    } }, '在地图上查看');
    const relBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('network', 'n-icon--sm'), onclick: async () => {
      stage.setView('graph');
      await graph.load();
      graph.select(r.scene_space_id);
    } }, '查看区域关系');
    const candidates = r.candidate_labels || [];
    const samples = r.observation_samples || [];
    const center = r.boundary?.centroid?.position;
    fill(view, back('全部区域', () => toList('regions')),
      h('div', { class: 'c-pane-head' }, chip(r), h('h2', {}, regionName(r, index)), h('span', { class: 'n-badge ' + regionBadge(r) }, regionStatus(r))),
      h('div', { class: 'c-card-actions c-gap-b' }, goBtn, focusBtn, relBtn),
      status,
      h('dl', { class: 'n-kv c-gap-b' }, h('dt', {}, '面积'), h('dd', { class: 'n-num' }, r.boundary?.area_m2 == null ? '未知' : r.boundary.area_m2.toFixed(2) + ' 平方米'),
        h('dt', {}, '中心'), h('dd', { class: 'n-num' }, center ? center.x.toFixed(2) + ', ' + center.y.toFixed(2) : '未知'),
        h('dt', {}, '能否通行'), h('dd', {}, '以导航检查为准'), h('dt', {}, '区域修订'), h('dd', { class: 'n-num' }, r.metadata?.scene_revision || '—')),
      h('div', { class: 'n-group-title' }, '命名'),
      form, h('p', { class: 'n-meta c-gap-b' }, '人工命名会锁定，自动识别不再覆盖。'),
      h('div', { class: 'n-group-title' }, '名称候选'),
      candidates.length ? h('div', { class: 'c-scores' }, candidates.map(c => h('div', { class: 'c-score' }, h('span', {}, c.label),
        h('div', { class: 'c-scorebar' }, h('span', { style: 'width:' + Math.max(0, Math.min(1, Number(c.confidence) || 0)) * 100 + '%' })),
        h('span', { class: 'n-num n-muted' }, Number(c.confidence).toFixed(3))))) : h('p', { class: 'n-meta' }, '缺少识别证据，当前仅为几何候选。'),
      candidates.length ? h('p', { class: 'n-meta c-gap-t' }, '模型分数尚未校准为房间识别准确率。模型：' + [...new Set(candidates.map(c => c.source_model).filter(Boolean))].join('、')
        + '。自动采用的分数、与次选差距、最少视角门槛在设置“记忆与语义”中调整。') : null,
      h('div', { class: 'n-group-title' }, '观察照片'),
      samples.length ? h('div', { class: 'c-evidence' }, samples.map(sample => h('div', { class: 'c-evidence-item' },
        h('div', { class: 'c-thumbs' }, (sample.images || []).filter(image => image.available).map(image => h('a', { href: image.url, target: '_blank', rel: 'noopener noreferrer', title: '在新标签页查看' }, h('img', { src: image.url, alt: '观察照片', loading: 'lazy' })))),
        h('div', { class: 'n-meta' }, (sample.camera_id || '来源相机未提供') + '，样本 ', h('code', {}, sample.sample_id)))))
        : h('p', { class: 'n-meta' }, '暂无已关联图像和观察。'),
      h('details', { class: 'c-gap-t' }, h('summary', {}, '来源与版本'), h('pre', { class: 'n-pre' }, JSON.stringify({ map_binding: r.metadata?.map_binding, scene_space_id: r.scene_space_id, decision: r.metadata?.last_decision }, null, 2))));
    if (focused) input.focus({ preventScroll: true });
  }

  /* ---------- 物品详情 ---------- */
  function regionOf(inst) {
    if (!finite(inst.pose) || !regions.matches) return null;
    return regionList().find(r => r.boundary?.polygon_points?.length && inside(inst.pose, r.boundary.polygon_points)) || null;
  }
  function renderItem() {
    const inst = items.instances.find(item => item.instance_id === state.itemId);
    if (!inst) { toList('items'); return; }
    const key = JSON.stringify([inst, items.error, map.activeMapName(), Boolean(map.info().grid)]);
    if (key === state.detailKey) return;
    state.detailKey = key;
    const quality = instanceQuality(inst), observation = observationOnly(inst);
    const pose = finite(inst.pose) ? inst.pose : null;
    const region = regionOf(inst);
    const note = observation ? '仅有观察证据：机器人曾在观察位置见过该物品，尚不能确定物品坐标。'
      : quality.key === 'located' ? '已保存物体位置估计。历史位置不保证物品仍在原处，导航前会重新解析可用候选。'
      : '空间证据不足或坐标系不匹配，暂不作为地图物品位置显示。';
    const rows = [
      ['标签', inst.label || '未分类'],
      ['空间证据', quality.label],
      ['所在区域', region ? regionName(region) : '未知'],
      ['地图 / 记忆库', `${inst.map_name || '未绑定地图'} / ${inst.library_name || items.meta.library_name || '未记录'}`],
      [observation ? '观察位置' : '物体位置', pose ? `x ${pose.x.toFixed(2)}，y ${pose.y.toFixed(2)}，z ${pose.z.toFixed(2)} 米` : '未确定'],
      ['坐标系', inst.frame_id || '未记录'],
      ['位置依据', METHOD_NAMES[inst.projection_method] || inst.projection_method || '未记录'],
      ['物品尺寸', extentText(inst)],
      ['空间置信度', Number.isFinite(inst.projection_confidence) ? Math.round(inst.projection_confidence * 100) + '%' : '未记录'],
      ['支持点数', Number.isFinite(inst.support_point_count) ? String(inst.support_point_count) : '未记录'],
      ['观察次数', Number.isFinite(inst.observation_count) ? String(inst.observation_count) : '未记录'],
      ['记忆状态', ({ active: '有效', uncertain: '待确认', stale: '已过时', removed: '已移除' })[inst.lifecycle_state] || '未记录'],
      ['移动状态', ({ static: '静止', dynamic: '移动中', quasi_static: '短时静止', unknown: '未确定' })[inst.dynamic_state] || '未记录'],
      ['最后见到', formatLastSeen(inst.last_seen_ts)],
      ['实例标识', inst.instance_id || '未记录'],
    ];
    const locateOff = quality.key !== 'located' || !scopeMatches(inst) || Boolean(items.error) || !map.info().grid;
    const itemSay = h('p', { class: 'c-status-line', role: 'status' }, items.error ? '当前为上次缓存，请刷新后再操作。'
      : !scopeMatches(inst) ? '此记录与当前地图不匹配。'
      : inst.navigable !== true ? '此记录尚不满足导航条件，需要补充空间观察。' : '导航受理与到点后的物品复核分别确认。');
    const locate = h('button', { class: 'n-btn', type: 'button', html: icon('locate-fixed', 'n-icon--sm'), disabled: locateOff, onclick: () => {
      if (window.__nuwaxMapToolV2?.active()) { itemSay.textContent = '请先结束地图选点或区域绘制，再定位查看物品。'; itemSay.dataset.level = 'error'; return; }
      stage.setView('2d');
      map.focusPoint(inst.pose.x, inst.pose.y, { record_id: inst.instance_id, target_name: instanceName(inst) });
    } }, '在地图中查看');
    const walk = h('button', { class: 'n-btn n-btn--primary', type: 'button', html: icon('navigation', 'n-icon--sm'), disabled: locateOff || inst.navigable !== true,
      onclick: () => walkTo(inst, walk, itemSay) }, '走到它附近');
    view.replaceChildren(back(state.cluster ? '此处的物品' : '全部物品', () => (state.cluster ? open('cluster', { itemId: null }) : toList('items'))),
      h('div', { class: 'c-pane-head' }, h('h2', {}, instanceName(inst)), h('span', { class: 'n-badge' + (quality.key === 'uncertain' ? ' n-badge--warn' : quality.key === 'located' ? ' n-badge--ok' : '') }, quality.label)),
      h('p', { class: 'n-muted c-gap-b' }, note),
      quality.key === 'located' ? h('div', { class: 'c-row-actions c-gap-b' }, walk, locate)
        : h('div', { class: 'n-callout c-gap-b', html: icon('info') }, h('div', { class: 'n-callout-body' }, observation
          ? '这条记录没有物体位置，不能直接导航。可以让机器人到观察位置附近转一圈补充观察。' : '位置还没确认，不能直接导航。补充观察后会更新。')),
      itemSay,
      h('dl', { class: 'n-kv' }, rows.flatMap(([k, v]) => [h('dt', {}, k), h('dd', {}, v)])));
  }
  /* 走到它附近：查本物品的导航候选，取候选的观察位置（平面）与朝向导航；机体高度由导航入口核对，不用物品中心高度。 */
  async function walkTo(inst, button, line) {
    const sayLine = (text, level = '') => { line.textContent = text; line.dataset.level = level; };
    if (items.error || !scopeMatches(inst) || inst.navigable !== true || instanceQuality(inst).key !== 'located') { sayLine('当前记录缺少有效导航证据，请刷新或重新观察。', 'error'); return; }
    const requestedMap = map.activeMapName(), requestedLibrary = items.meta.library_name;
    button.disabled = true;
    sayLine('正在查找此物品的可用导航候选…');
    const controller = new AbortController(), timer = setTimeout(() => controller.abort(), 10000);
    try {
      const body = await api.invoke('resolve_navigation_candidates', { query: inst.label || instanceName(inst), map_name: requestedMap, limit: 20 }, { signal: controller.signal });
      const candidates = (body.result || body).navigation_candidates;
      const candidate = Array.isArray(candidates) && candidates.find(item => item.record_id === inst.instance_id && item.map_name === requestedMap
        && item.metadata?.allowed_for_navigation === true && item.metadata?.resolution_decision === 'navigate');
      if (!candidate) throw new Error('此物品当前没有可用导航候选，请补充观察或在地图中选择可达位置。');
      const pose = candidate.inspection_pose, point = pose?.position, info = map.info();
      if (!finite(point) || pose.frame_id !== info.frameId) throw new Error('候选位置或坐标系无效。');
      if (requestedMap !== map.activeMapName() || requestedLibrary !== items.meta.library_name || items.error) throw new Error('地图或记忆库已变化，请刷新后重试。');
      const yaw = planarYawFromPose(pose) ?? (info.pose ? Math.atan2(point.y - info.pose.y, point.x - info.pose.x) : 0);
      map.setHighlight({ record_id: inst.instance_id, target_pose: point, target_name: instanceName(inst) });
      await api.invoke('navigate_to_pose', { frame_id: pose.frame_id, x: point.x, y: point.y, yaw_rad: yaw });
      toast('已开始导航到「' + instanceName(inst) + '」附近');
      go('monitor', 'activity', { open: true });
    } catch (error) {
      sayLine(error.name === 'AbortError' ? '导航候选查询超时，请重试。' : String(error.message || error), 'error');
    } finally { clearTimeout(timer); button.disabled = false; }
  }

  /* ---------- 关系图节点详情 ---------- */
  async function loadNode(id, related = false) {
    const version = graph.graph?.graph_revision, token = ++state.nodeToken;
    state.nodeDetail = null;
    try {
      const detail = await api.invoke('get_scene_graph_node', { node_id: id, expected_graph_revision: version });
      if (token !== state.nodeToken || graph.graph?.graph_revision !== detail.graph_revision) return;
      state.nodeDetail = detail;
    } catch (error) {
      if (token !== state.nodeToken) return;
      say(error.message, 'error');
      if (error.status === 409) graph.reload();
      if (related) return;
    }
    state.detailKey = '';
    render();
  }
  async function route(to) {
    const version = graph.graph?.graph_revision, from = state.from;
    if (!from || !version) return;
    try {
      const result = await api.invoke('plan_scene_graph_route', { from_node_id: from, to_node_id: to, expected_graph_revision: version });
      if (version !== graph.graph?.graph_revision || from !== state.from) return;
      graph.setRoute(result);
      say(result.status === 'candidate_route' ? `候选路径：${result.region_names.join(' → ')}，经过 ${result.passage_count} 个连接口，待导航检查。` : '当前关系图没有已知连接路径。');
    } catch (error) { say(error.message, 'error'); if (error.status === 409) graph.reload(); }
  }
  function renderNode() {
    const detail = state.nodeDetail;
    if (state.from && !graph.graph?.nodes.some(n => n.node_id === state.from)) state.from = null;
    const key = JSON.stringify([state.nodeId, detail?.graph_revision, detail?.node?.node_id, state.from]);
    if (key === state.detailKey) return;
    state.detailKey = key;
    const head = back('全部区域', () => toList('regions'));
    if (!detail) { view.replaceChildren(head, h('p', { class: 'n-meta' }, '正在读取节点…'), status); return; }
    const n = detail.node, p = n.properties || {};
    const btn = (label, onclick, extra = {}) => h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick, ...extra }, label);
    const actions = [];
    if (n.kind === 'region') actions.push(btn('在地图中查看', () => {
      stage.setView('2d');
      window.__nuwaxMapFocusRegion?.({ scene_space_id: n.node_id, boundary: p.boundary });
      if (regionList().some(r => r.scene_space_id === n.node_id)) open('region', { regionId: n.node_id, nodeId: null, nodeDetail: null });
    }));
    if (n.kind === 'region' || p.region_id) {
      actions.push(btn(state.from === n.node_id ? '已设为起点' : '设为起点', () => {
        state.from = n.node_id; graph.setRoute(null);
        say('起点：' + n.name + '。选择终点后查询候选路径。');
        state.detailKey = ''; render();
      }));
      actions.push(btn('查询到此的路径', () => route(n.node_id), { disabled: !state.from }));
    }
    const imageBtn = btn('查看局部图像', async () => {
      imageBtn.disabled = true;
      try {
        const uri = await graph.exportImage(n.node_id);
        if (uri) { say('局部图像已生成。'); status.append(' ', h('a', { href: uri, target: '_blank', rel: 'noopener noreferrer' }, '打开图像')); }
      } catch (error) { say(error.message, 'error'); }
      finally { imageBtn.disabled = false; }
    });
    actions.push(imageBtn);
    const images = (detail.images || []).filter(image => image.available);
    fill(view, head,
      h('div', { class: 'c-pane-head' }, h('h2', {}, n.name), h('span', { class: 'n-badge' }, NODE_KINDS[n.kind] + (p.display_code ? ' ' + p.display_code : ''))),
      h('dl', { class: 'n-kv c-gap-b' },
        n.position ? [h('dt', {}, '位置'), h('dd', { class: 'n-num' }, `${n.position.x.toFixed(2)}, ${n.position.y.toFixed(2)}, ${n.position.z.toFixed(2)} 米`)] : null,
        p.boundary?.area_m2 != null ? [h('dt', {}, '面积'), h('dd', { class: 'n-num' }, p.boundary.area_m2.toFixed(2) + ' 平方米')] : null,
        p.observed_at || p.last_seen_ts ? [h('dt', {}, '观察时间'), h('dd', {}, String(p.observed_at || p.last_seen_ts))] : null),
      n.kind === 'passage' ? h('p', { class: 'n-muted c-gap-b' }, '当前地图的自由区域在此相接；门类型与实际通行尚未确认。') : null,
      p.summary ? h('p', { class: 'c-gap-b' }, p.summary) : null,
      h('div', { class: 'c-card-actions c-gap-b' }, actions), status,
      h('div', { class: 'n-group-title' }, '关联节点'),
      detail.neighbors.length ? h('div', { class: 'n-list' }, detail.neighbors.map(neighbor => h('button', { class: 'n-row n-row--hover', type: 'button', onclick: () => {
        if (graph.graph?.nodes.some(x => x.node_id === neighbor.node_id)) graph.select(neighbor.node_id);
        else { state.nodeId = neighbor.node_id; void loadNode(neighbor.node_id, true); }
      } }, h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, neighbor.name), h('div', { class: 'n-row-sub' }, NODE_KINDS[neighbor.kind])),
        h('span', { class: 'c-rowchev', html: icon('chevron-right', 'n-icon--sm') })))) : h('p', { class: 'n-meta' }, '暂无已建立的关系。'),
      h('div', { class: 'n-group-title' }, '图像证据'),
      images.length ? h('div', { class: 'c-thumbs' }, images.map(image => h('a', { href: image.url, target: '_blank', rel: 'noopener noreferrer' }, h('img', { src: image.url, alt: '图像证据', loading: 'lazy' }))))
        : h('p', { class: 'n-meta' }, '暂无可用图像。'),
      h('details', { class: 'c-gap-t' }, h('summary', {}, '身份与来源'), h('pre', { class: 'n-pre' }, JSON.stringify({ node_id: n.node_id, map_binding: detail.map_binding, graph_revision: detail.graph_revision, properties: p }, null, 2))));
  }

  /* ---------- 整体渲染 ---------- */
  function render() {
    if (!state.shown) return;
    if (state.page === 'region') { renderRegion(); return; }
    if (state.page === 'item') { renderItem(); return; }
    if (state.page === 'node') { renderNode(); return; }
    const pageKey = state.page + '|' + state.tab;
    if (pageKey !== state.pageKey) {
      state.pageKey = pageKey; state.listKey = ''; state.detailKey = '';
      delete view.dataset.regionId;
      if (state.page === 'cluster') {
        view.replaceChildren(back('全部物品', () => toList('items')), h('div', { class: 'c-pane-head' }, h('h2', {}, `此处有 ${state.cluster.members.length} 个物品`)), listBox);
      } else if (state.tab === 'regions') {
        view.replaceChildren(h('div', { class: 'c-pane-head' }, h('h2', {}, '语义地图')), seg, recogCard, status, h('div', { class: 'c-gap-b' }, regionSearch), listBox,
          h('p', { class: 'n-meta c-gap-t' }, '点一行或直接点地图上的区域，查看详情、命名或前往。区域只表示空间划分，能否通行以导航检查为准。'));
      } else {
        view.replaceChildren(h('div', { class: 'c-pane-head' }, h('h2', {}, '语义地图')), seg, filterSeg,
          h('div', { class: 'c-search-row c-gap-b' }, itemSearch, itemRefresh), itemStatus, itemSummary, listBox,
          h('p', { class: 'n-meta c-gap-t' }, '点一行或地图上的物品标记查看详情。记忆库随当前地图切换，地图上只标出有位置依据的物品。'));
      }
    }
    renderList();
  }
  regions.onChange(() => { if (state.shown) render(); });
  items.onChange(() => { if (state.shown) render(); });
  graph.onChange(() => { if (state.shown && state.page === 'node') render(); });

  return {
    /* opts.pick：舞台点选 {type:'item'|'cluster'|'region'|'node', …}；opts.tab：regions / items。 */
    show(opts = {}) {
      state.shown = true;
      regions.setPanelOpen(true);
      const pick = opts.pick;
      if (pick?.type === 'item') open('item', { tab: 'items', itemId: pick.instance.instance_id, cluster: null });
      else if (pick?.type === 'cluster') open('cluster', { tab: 'items', cluster: pick.cluster, itemId: null });
      else if (pick?.type === 'region') open('region', { tab: 'regions', regionId: pick.region.scene_space_id });
      else if (pick?.type === 'node') open('node', { nodeId: pick.node.node_id, nodeDetail: null });
      else if (opts.tab) toList(opts.tab);
      else { state.pageKey = ''; state.detailKey = ''; render(); }
      if (!items.fetchedAt && !items.loading) void items.load();
    },
    hide() {
      state.shown = false;
      clearSelection();
      if (state.page === 'region' || state.page === 'item') state.page = 'list';
      regions.setPanelOpen(false);
    },
  };
}
