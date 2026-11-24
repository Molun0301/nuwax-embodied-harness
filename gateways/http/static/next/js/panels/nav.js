/* 导航面板：选点、去某处、自动探索、跨楼层。
   选点从 navigation.js 迁出，行为不变：导航页“选点”可见时，单击与拖动照常旋转平移地图；双击地图点选点（不定朝向），
   双击的第二下按住拖动同时设定朝向（按下处为位置、拖动方向为朝向，超过 6 像素才算拖动），两次按下间隔不超过
   500 毫秒、相距不超过 6 像素；双击没命中地图点不选点；提交有朝向时带 yaw_rad；按任务 goal_id 记住是否定朝向。
   去某处从 robot-controls.js 迁出（navigate_to_named_location，候选取标记地点与区域）；自动探索从
   exploration-mission.js 迁出；跨楼层从 building-navigation.js 迁出。 */
import { icon, h, toast } from '/static/shared/neh-ui.js';
import { isTerminalTaskState, normalizeTaskState } from '../status.js';
import { CLICK_PIXELS, poseAt, trackPoseDrag } from '../stage/posedrag.js';

const TABS = [['pick', '选点'], ['named', '去某处'], ['explore', '自动探索'], ['floor', '跨楼层']];
const DOUBLE_CLICK_MS = 500;
const EXPLORE_CAPS = ['explore_area_v2', 'resume_exploration', 'explore_building'];
const FLOOR_CAPS = new Set(['navigate_to_floor', 'explore_building']);
const FLOOR_STATES = { running: '任务运行中', cancelling: '正在取消任务', succeeded: '任务已完成', failed: '任务失败', cancelled: '任务已取消', timed_out: '任务已超时' };

export function mountNav(root, ctx) {
  const { api, store, tasks, stage, taskCards, go } = ctx;
  let tab = 'pick', shown = false;
  const seg = h('div', { class: 'n-seg n-seg--block c-panel-seg', role: 'tablist', 'aria-label': '导航方式' });
  const pages = Object.fromEntries(TABS.map(([id]) => [id, h('div', { class: 'c-nav-page', dataset: { page: id }, hidden: true })]));
  const running = h('div', { class: 'c-nav-running' });
  root.append(h('div', { class: 'c-pane-head' }, h('h2', {}, '导航')), seg, ...Object.values(pages), running);
  function renderSeg() {
    seg.replaceChildren(...TABS.map(([id, label]) => h('button', { type: 'button', role: 'tab', 'aria-pressed': String(tab === id), onclick: () => setTab(id) }, label)));
  }
  function setTab(id) {
    tab = id;
    renderSeg();
    for (const [key, page] of Object.entries(pages)) page.hidden = key !== id;
    syncPicking();
    if (id === 'named' && !named.loaded) loadNames();
    if (id === 'explore') loadCheckpoints();
    if (id === 'floor') loadFloors();
    renderRunning();
  }

  /* ================= 选点 ================= */
  const pick = { active: false, taskId: null, pendingPose: null, submittedPose: null, submittedGoalId: null, submitting: false };
  const pickStatus = h('p', { class: 'c-status-line', role: 'status' });
  const pickDetail = h('p', { class: 'n-meta' });
  const heightInput = h('input', { class: 'n-input n-num', type: 'number', step: 'any', inputmode: 'decimal', placeholder: '留空使用当前定位高度' });
  const pickCard = h('div');
  const submitLabel = h('span', {});
  const submitBtn = h('button', { class: 'n-btn n-btn--primary n-btn--block n-btn--lg', type: 'button', html: icon('navigation', 'n-icon--sm'), onclick: () => submitNavigation() });
  submitBtn.append(submitLabel);
  pages.pick.append(pickCard,
    h('label', { class: 'n-field c-field-gap', title: '目标在地图中的机体 Z 坐标，不是物品中心或脚下地面高度' }, h('span', { class: 'n-label' }, '目标机体高度（米）'), heightInput),
    submitBtn, pickStatus, pickDetail);
  function setPickStatus(text, level = '') { pickStatus.textContent = text; pickStatus.dataset.level = level; }
  function regionLabelAt(x, y) {
    const inside = (poly) => { let c = false; for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const a = poly[i], b = poly[j]; if ((a.y > y) !== (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) c = !c; } return c; };
    const region = stage.regions.active.find(r => r.boundary?.polygon_points?.length && inside(r.boundary.polygon_points.map(p => ({ x: p.x ?? p[0], y: p.y ?? p[1] }))));
    return region ? region.primary_label || '未命名区域 ' + stage.regions.number(region) : '';
  }
  function renderPick() {
    const pose = pick.pendingPose;
    if (!pose) {
      pickCard.replaceChildren(h('ol', { class: 'c-steps-hint' },
        h('li', {}, h('span', { class: 'c-stepn' }, '1'), h('span', {}, '在地图上', h('b', {}, '双击'), '目标位置')),
        h('li', {}, h('span', { class: 'c-stepn' }, '2'), h('span', {}, '想指定到达朝向：双击的第二下按住不放，朝目标方向拖动')),
        h('li', {}, h('span', { class: 'c-stepn' }, '3'), h('span', {}, '确认目标后点“开始导航”'))),
        h('p', { class: 'n-meta c-gap-b' }, '单击拖动照常旋转、平移地图，不会误选点。'));
    } else {
      const region = regionLabelAt(pose.x, pose.y);
      pickCard.replaceChildren(h('div', { class: 'c-card c-gap-b' },
        h('div', { class: 'c-card-head' }, h('div', { class: 'c-hero-main' }, h('div', { class: 'n-meta' }, '已选目标'),
          h('div', { class: 'c-card-title n-num' }, 'x ' + pose.x.toFixed(2) + '　y ' + pose.y.toFixed(2))),
          h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', onclick: () => clearPick() }, '清除')),
        h('dl', { class: 'n-kv c-gap-t' }, h('dt', {}, '到达朝向'), h('dd', { class: 'n-num' }, Number.isFinite(pose.yaw) ? Math.round(pose.yaw * 180 / Math.PI) + '°' : '不指定（双击后拖动可设定）'),
          h('dt', {}, '所在区域'), h('dd', {}, region || '不在已划分的区域内'))));
    }
    submitBtn.disabled = !pose || pick.submitting;
    submitLabel.textContent = pose ? '开始导航' : '先在地图上双击选点';
  }
  function showPick(pose) { window.pointcloud3d?.previewNavigationPoint(pose); stage.map.requestDraw(); }
  function selectMapPoint(x, y, frameId, selection) {
    pick.pendingPose = { ...selection, x, y, frame_id: frameId || 'map' };
    showPick(pick.pendingPose);
    setPickStatus('');
    renderPick();
  }
  function clearPick() { pick.pendingPose = null; showPick(null); renderPick(); }
  function targetHeight(point) {
    if (heightInput.value.trim()) return Number(heightInput.value);
    if (Number.isFinite(point.surface_z)) return point.surface_z - window.pointcloud3d._robotGeometry.z_min_m;
    return null;
  }
  async function submitNavigation() {
    if (!pick.pendingPose || pick.submitting) return;
    const pose = pick.pendingPose;
    const args = { frame_id: pose.frame_id, x: pose.x, y: pose.y };
    const z = targetHeight(pose);
    if (z !== null) args.z = z;
    if (Number.isFinite(pose.yaw)) args.yaw_rad = pose.yaw;
    pick.submitting = true;
    renderPick();
    try {
      const reply = await api.invoke('navigate_to_pose', args);
      pick.taskId = reply.task.task_id; pick.submittedPose = pose; pick.submittedGoalId = null;
      applyPickTask(reply.task);
    } catch (error) { setPickStatus(error.message, 'error'); }
    finally { pick.submitting = false; renderPick(); }
  }
  function applyPickTask(task) {
    if (task.task_id !== pick.taskId) return;
    // 后端回传的目标不区分“未指定朝向”，按任务目标号记住本页提交时的选择，并让地图按选择重画目标。
    if (task.data?.goal_id && task.data.goal_id !== pick.submittedGoalId) {
      pick.submittedGoalId = task.data.goal_id;
      window.dispatchEvent(new CustomEvent('navigation:submitted-goal', { detail: { goal_id: pick.submittedGoalId } }));
    }
    pickDetail.textContent = task.message;
    setPickStatus(task.message, task.state === 'failed' || task.state === 'timed_out' ? 'error' : '');
    if (isTerminalTaskState(task.state)) {
      pick.taskId = null;
      if (pick.pendingPose === pick.submittedPose) clearPick();
    }
  }
  // 三维：pointcloud3d.js 自己处理双击与拖动，选中后发事件。
  window.addEventListener('pointcloud3d:navigation-pick', event => {
    if (!window.__nuwaxNavigationActive) return;
    const point = event.detail;
    if (!point) { setPickStatus('双击处没有地图点，未选点；请放大后双击可见地面', 'warn'); return; }
    selectMapPoint(point.x, point.y, point.frame_id, point);
  });
  // 二维：捕获阶段先于地图画布自身的拖动处理；双击的第二下按住拖动定朝向（stage/posedrag.js）。
  (function capture2d() {
    const mapEl = stage.canvas;
    let press = null, lastClick = null;
    mapEl.addEventListener('pointerdown', ev => {
      if (!window.__nuwaxNavigationActive || ev.button !== 0 || ev.isPrimary === false) return;
      const second = lastClick && ev.timeStamp - lastClick.time <= DOUBLE_CLICK_MS && Math.hypot(ev.clientX - lastClick.x, ev.clientY - lastClick.y) <= CLICK_PIXELS;
      lastClick = null;
      press = second ? null : { id: ev.pointerId, x: ev.clientX, y: ev.clientY, time: ev.timeStamp };
      if (!second) return;
      const point = poseAt('2d', ev);
      if (!point) { setPickStatus('等待可用的二维地图后选点', 'warn'); return; }
      ev.stopImmediatePropagation();
      ev.preventDefault();
      selectMapPoint(point.x, point.y, point.frame_id);
      trackPoseDrag(mapEl, ev, point, '2d', {
        onYaw: yaw => { pick.pendingPose = { ...pick.pendingPose, yaw }; showPick(pick.pendingPose); },
        onEnd: () => renderPick(),
      });
    }, true);
    function finishPress(ev) {
      const start = press;
      press = null;
      // 按下后几乎没动的单击留作双击的第一下。
      if (ev.type === 'pointerup' && start && start.id === ev.pointerId && Math.hypot(ev.clientX - start.x, ev.clientY - start.y) <= CLICK_PIXELS) {
        lastClick = { time: start.time, x: start.x, y: start.y };
      }
    }
    mapEl.addEventListener('pointerup', finishPress);
    mapEl.addEventListener('pointercancel', finishPress);
  })();
  function syncPicking() {
    const active = shown && tab === 'pick';
    if (active === pick.active) return;
    pick.active = active;
    window.__nuwaxNavigationActive = active;
    stage.setPicking(active);
    if (active) window.__nuwaxResetMapInteraction?.();
    // 进入或离开选点都清掉未提交的选点（与原导航模块 setActive 相同）。
    pick.pendingPose = null;
    showPick(null);
    renderPick();
    stage.hint(active ? h('div', { class: 'n-float c-hint', html: icon('target', 'n-icon--sm') }, '双击地图选择导航目标；双击的第二下按住拖动设定朝向') : null);
  }
  // 地图核心按这里画选点，并据此判断提交时没定朝向的目标。
  window.__nuwaxNavigationModule = {
    getState: () => pick,
    goalOrientationUnset: goalId => Boolean(goalId && goalId === pick.submittedGoalId && !Number.isFinite(pick.submittedPose?.yaw)),
    setActive(value) { if (!value) { shown = false; syncPicking(); } },
    submit: submitNavigation,
  };

  /* ================= 去某处 ================= */
  const named = { loaded: false, names: [], taskId: null, target: '' };
  const namedInput = h('input', { class: 'n-input', placeholder: '地点、区域或物品名称', autocomplete: 'off' });
  const namedSugg = h('div', { class: 'n-list c-ask-list' });
  const verify = h('input', { type: 'checkbox', checked: true });
  const namedStatus = h('p', { class: 'c-status-line', role: 'status' }, '按当前地图记忆中的地点、区域或物品名称前往。');
  const namedGo = h('button', { class: 'n-btn n-btn--primary n-btn--block n-btn--lg', type: 'button', html: icon('navigation', 'n-icon--sm'), onclick: () => submitNamed() }, '前往');
  const namedCancel = h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: true, onclick: () => cancelNamed() }, '取消这次前往');
  pages.named.append(h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '要去哪里'), namedInput), namedSugg,
    h('label', { class: 'n-check c-gap-y', title: '到达后连拍核对目标，失败会降权并报告原因' }, verify, '到达后连拍核对目标，核对失败会报告原因'),
    namedGo, h('div', { class: 'c-row-actions' }, namedCancel, h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', onclick: () => loadNames() }, '刷新名称')), namedStatus);
  function namedSay(text, level = '') { namedStatus.textContent = text; namedStatus.dataset.level = level; }
  async function loadNames() {
    const mapName = stage.map.binding().map_name;
    const [locations, spaces] = await Promise.allSettled([api.json('/api/memory/locations?limit=500'),
      api.invoke('list_scene_spaces', mapName ? { limit: 500, map_name: mapName } : { limit: 500 })]);
    const found = new Map();
    for (const item of locations.value?.locations || []) for (const name of [item.name, ...(item.aliases || [])]) if (name) found.set(name, '标记地点');
    for (const item of spaces.value?.scene_spaces || []) for (const name of [item.primary_label, ...(item.human_aliases || [])]) if (name) found.set(name, '区域');
    named.names = [...found].sort((a, b) => a[0].localeCompare(b[0], 'zh-CN'));
    named.loaded = true;
    fillNamed();
    if (!named.taskId) namedSay(found.size ? '当前地图记忆中有 ' + found.size + ' 个名称可选，也可直接输入。' : '当前地图记忆中还没有地点或区域名称，可直接输入。');
  }
  function fillNamed() {
    const q = namedInput.value.trim();
    namedSugg.replaceChildren(...named.names.filter(([name]) => !q || name.includes(q)).slice(0, 7).map(([name, kind]) => h('button', { class: 'n-row', type: 'button',
      onclick: () => { namedInput.value = name; fillNamed(); } },
      h('span', { class: 'c-ask-kind', html: icon(kind === '区域' ? 'land-plot' : 'map-pin', 'n-icon--sm') }),
      h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, name), h('div', { class: 'n-row-sub' }, kind)))));
  }
  namedInput.addEventListener('input', fillNamed);
  namedInput.addEventListener('keydown', event => { if (event.key === 'Enter') submitNamed(); });
  async function submitNamed() {
    const target = namedInput.value.trim();
    if (!target) { namedInput.focus(); return; }
    namedGo.disabled = true;
    namedSay('已提交，等待网关应答。');
    try {
      const reply = await api.invoke('navigate_to_named_location', { target_name: target, verify_on_arrival: verify.checked });
      named.taskId = reply.task.task_id;
      named.target = target;
      applyNamedTask(reply.task);
    } catch (error) {
      namedSay('导航失败：' + error.message, 'error');
    } finally { namedGo.disabled = false; }
  }
  function applyNamedTask(task) {
    if (task.task_id !== named.taskId) return;
    const state = normalizeTaskState(task.state);
    const runningNow = !isTerminalTaskState(state);
    namedCancel.disabled = !runningNow || state === 'cancelling';
    const pct = Number.isFinite(Number(task.progress)) ? ' ' + Math.round(Number(task.progress) * 100) + '%' : '';
    if (runningNow) namedSay('前往「' + named.target + '」：' + task.message + pct);
    else if (state === 'succeeded') namedSay('已到达「' + named.target + '」' + (task.result?.verification ? '并完成核对' : '') + '。');
    else namedSay('前往「' + named.target + '」' + (state === 'cancelled' ? '已取消：' : '失败：') + task.message, 'error');
    if (!runningNow) named.taskId = null;
  }
  async function cancelNamed() {
    namedCancel.disabled = true;
    try { await api.invoke('cancel_task', { task_id: named.taskId }); }
    catch (error) { namedCancel.disabled = false; namedSay('取消失败：' + error.message, 'error'); }
  }

  /* ================= 自动探索 ================= */
  const ex = { taskId: null, checkpointsLoaded: false, loadedCheckpoint: null, busy: false };
  const exMap = h('input', { class: 'n-input', placeholder: '留空使用当前地图' });
  const exTimeout = h('input', { class: 'n-input n-num', type: 'number', min: 1, value: 600 });
  const exDistance = h('input', { class: 'n-input n-num', type: 'number', min: 0.1, step: 0.1, placeholder: '不限' });
  const exReserve = h('input', { class: 'n-input n-num', type: 'number', min: 0, step: 0.1, value: 0 });
  const exReturn = h('input', { type: 'checkbox' });
  const exStart = h('button', { class: 'n-btn n-btn--primary n-btn--block n-btn--lg', type: 'button', html: icon('compass', 'n-icon--sm'), onclick: () => startExplore() }, '开始探索');
  const exForm = h('div', {},
    h('label', { class: 'n-field c-field-gap' }, h('span', { class: 'n-label' }, '地图名称'), exMap),
    h('div', { class: 'c-form-2' },
      h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '最长运行'), h('div', { class: 'n-input-group' }, exTimeout, h('span', { class: 'n-suffix' }, '秒'))),
      h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '总路程上限'), h('div', { class: 'n-input-group' }, exDistance, h('span', { class: 'n-suffix' }, '米'))),
      h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '返航余量'), h('div', { class: 'n-input-group' }, exReserve, h('span', { class: 'n-suffix' }, '米')))),
    h('label', { class: 'n-switch c-gap-y' }, exReturn, h('span', { class: 'n-switch-track' }), '结束后返回起点'),
    exStart);
  const exCard = h('div', { class: 'c-act-focus' });
  const exStatus = h('p', { class: 'c-status-line', role: 'status' });
  const exCheckpoints = h('div', { class: 'n-list' });
  pages.explore.append(exCard, exForm, exStatus,
    h('div', { class: 'n-group-title' }, '暂停过的探索', h('span', { class: 'n-group-actions' }, h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', onclick: () => loadCheckpoints() }, '刷新'))),
    exCheckpoints, h('p', { class: 'n-meta c-gap-t' }, '暂停会停止探索后保存地图和进度；继续时会先重新定位。'));
  function exSay(text, level = '') { exStatus.textContent = text; exStatus.dataset.level = level; }
  async function startExplore() {
    if (ex.busy) return;
    ex.busy = true; exStart.disabled = true;
    try {
      const args = { timeout_sec: Number(exTimeout.value) };
      if (exMap.value.trim()) args.map_name = exMap.value.trim();
      if (exDistance.value !== '' || exReturn.checked) {
        args.return_policy = { reserve_distance_m: Number(exReserve.value), return_on_completion: exReturn.checked };
        if (exDistance.value !== '') args.return_policy.maximum_distance_m = Number(exDistance.value);
      }
      const reply = await api.invoke('explore_area_v2', args);
      ex.taskId = reply.task.task_id; ex.loadedCheckpoint = null;
      exSay(reply.task.message);
    } catch (error) { exSay(error.message, 'error'); }
    finally { ex.busy = false; exStart.disabled = false; }
  }
  async function loadCheckpoints() {
    try {
      const data = await api.invoke('list_exploration_checkpoints');
      exCheckpoints.replaceChildren(...(data.checkpoints.length ? data.checkpoints.map(item => h('div', { class: 'n-row n-row--hover' },
        h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, item.map_name),
          h('div', { class: 'n-row-sub' }, new Date(item.created_at).toLocaleString('zh-CN', { hour12: false }) + ' 暂停' + (item.available ? '' : '，地图已删除'))),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: !item.available || ex.busy, title: item.available ? '加载这张地图、重新定位后从保存的进度继续' : '这张地图已删除，不能继续',
          onclick: () => resume(item) }, '继续'))) : [h('p', { class: 'n-meta' }, '还没有暂停过的探索。')]));
    } catch (error) { exSay('读取暂停过的探索失败：' + error.message, 'error'); }
  }
  async function resume(item) {
    if (ex.busy) return;
    ex.busy = true;
    try {
      const reply = await api.invoke('resume_exploration', { checkpoint_id: item.checkpoint_id, timeout_sec: Number(exTimeout.value) });
      ex.taskId = reply.task.task_id; ex.loadedCheckpoint = null;
      exSay('已从保存的进度继续探索，正在重新定位。');
    } catch (error) { exSay(error.message, 'error'); }
    finally { ex.busy = false; }
  }
  function applyExploreTask(task) {
    if (task.task_id !== ex.taskId) return;
    const saved = task.result;
    if (saved?.paused) {
      exSay('已停止探索并保存地图和进度。');
      if (ex.loadedCheckpoint !== saved.checkpoint_id) { ex.loadedCheckpoint = saved.checkpoint_id; loadCheckpoints(); }
    } else exSay(task.message, ['failed', 'timed_out'].includes(task.state) ? 'error' : '');
  }

  /* ================= 跨楼层 ================= */
  const fl = { buildings: [], task: null, busy: false, loaded: false };
  const flBuilding = h('select', { class: 'n-select', 'aria-label': '目标建筑' });
  const flTarget = h('select', { class: 'n-select', 'aria-label': '目标楼层' });
  const flX = h('input', { class: 'n-input n-num', type: 'number', step: 'any', required: true });
  const flY = h('input', { class: 'n-input n-num', type: 'number', step: 'any', required: true });
  const flYaw = h('input', { class: 'n-input n-num', type: 'number', step: 'any', value: 0 });
  const flSubmit = h('button', { class: 'n-btn n-btn--primary n-btn--block', type: 'button', onclick: () => submitFloor(false) }, '前往目标层');
  const flCancel = h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: true, onclick: () => cancelFloor() }, '停止任务');
  const flExplore = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => submitFloor(true) }, '逐层探索建筑');
  const flImport = h('input', { type: 'file', accept: 'application/json,.json', hidden: true });
  const flImportBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => flImport.click() }, '导入楼层配置…');
  const flStatus = h('p', { class: 'c-status-line', role: 'status' }, '正在读取楼层目录。');
  const flDetail = h('pre', { class: 'n-pre', hidden: true });
  pages.floor.append(
    h('div', { class: 'c-form-2' }, h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '建筑'), flBuilding), h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '目标楼层'), flTarget)),
    h('div', { class: 'c-form-3 c-gap-t' }, h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, 'X（米）'), flX), h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, 'Y（米）'), flY),
      h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '朝向（弧度）'), flYaw)),
    h('p', { class: 'n-meta c-gap-y' }, '通过已登记的设施前往目标层，坐标填目标层地图里的位置。'),
    flSubmit, h('div', { class: 'c-row-actions' }, flCancel, flExplore, flImportBtn, flImport), flStatus, flDetail);
  const building = () => fl.buildings.find(value => value.building_id === flBuilding.value);
  const floorRunning = () => Boolean(fl.task && !isTerminalTaskState(fl.task.state));
  function flSay(text, level = '') { flStatus.textContent = text; flStatus.dataset.level = level; }
  function floorButtons() {
    const disabled = fl.busy || floorRunning();
    flSubmit.disabled = disabled || !flTarget.value;
    flCancel.disabled = fl.busy || !floorRunning() || fl.task.state === 'cancelling';
    flExplore.disabled = disabled || !building();
    flImportBtn.disabled = disabled;
    for (const element of [flBuilding, flTarget, flX, flY, flYaw]) element.disabled = disabled;
  }
  function renderFloors() {
    const selected = flTarget.value;
    flTarget.replaceChildren(...(building()?.floors || []).map(floor => { const option = new Option(floor.display_name + '，' + floor.map_name, floor.floor_id); option.disabled = floor.enabled === false; return option; }));
    flTarget.value = [...flTarget.options].find(o => o.value === selected && !o.disabled)?.value || [...flTarget.options].find(o => !o.disabled)?.value || '';
    floorButtons();
  }
  async function loadFloors() {
    try {
      const [reply, snapshot] = await Promise.all([api.invoke('list_buildings'), api.json('/api/tasks')]);
      fl.buildings = reply.buildings;
      const selected = flBuilding.value;
      flBuilding.replaceChildren(...fl.buildings.map(value => new Option(value.display_name, value.building_id)));
      if (fl.buildings.some(value => value.building_id === selected)) flBuilding.value = selected;
      renderFloors();
      fl.task = snapshot.active_tasks.find(value => FLOOR_CAPS.has(value.capability)) || snapshot.history_tasks.find(value => FLOOR_CAPS.has(value.capability)) || fl.task;
      renderFloorTask();
      if (!fl.task) flSay(fl.buildings.length ? '请选择目标楼层并填写坐标。' : '尚未登记楼栋，请导入楼层与设施配置。');
    } catch (error) { flSay(error.message, 'error'); }
  }
  function renderFloorTask() {
    if (!fl.task) { flDetail.hidden = true; floorButtons(); return; }
    const t = fl.task, data = t.data || {};
    const lines = ['任务：' + t.task_id, '功能：' + t.capability, '阶段：' + t.stage, '进度：' + Math.round(t.progress * 100) + '%', t.message];
    if (data.floor_id) lines.push('楼层：' + data.floor_id);
    if (data.completed_floors) lines.push('已保存楼层：' + data.completed_floors.join('、'));
    if (t.error_code) lines.push('错误码：' + t.error_code);
    if (t.result) lines.push(JSON.stringify(t.result, null, 2));
    flDetail.textContent = lines.join('\n');
    flDetail.hidden = false;
    flSay(FLOOR_STATES[normalizeTaskState(t.state)] || t.state, ['failed', 'timed_out'].includes(t.state) ? 'error' : '');
    floorButtons();
  }
  async function submitFloor(explore) {
    if (fl.busy) return;
    fl.busy = true; floorButtons();
    try {
      const args = explore ? { building_id: flBuilding.value, floor_ids: building().floors.filter(v => v.enabled !== false).map(v => v.floor_id), timeout_sec: 7200 }
        : { building_id: flBuilding.value, floor_id: flTarget.value, x: parseFloat(flX.value), y: parseFloat(flY.value), yaw_rad: parseFloat(flYaw.value), timeout_sec: 600 };
      fl.task = (await api.invoke(explore ? 'explore_building' : 'navigate_to_floor', args)).task;
      renderFloorTask();
    } catch (error) { flSay(error.message, 'error'); }
    finally { fl.busy = false; floorButtons(); }
  }
  async function cancelFloor() {
    fl.busy = true; floorButtons();
    try {
      await api.json('/api/tasks/' + encodeURIComponent(fl.task.task_id) + '/cancel', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
      if (floorRunning()) flSay('取消请求已发送，等待原任务退出。');
    } catch (error) { flSay(error.message, 'error'); }
    finally { fl.busy = false; floorButtons(); }
  }
  flBuilding.addEventListener('change', () => { flX.value = ''; flY.value = ''; renderFloors(); });
  flTarget.addEventListener('change', () => { flX.value = ''; flY.value = ''; });
  flImport.addEventListener('change', async () => {
    const file = flImport.files?.[0];
    if (!file) return;
    fl.busy = true; floorButtons();
    try {
      await api.invoke('register_building', { topology: JSON.parse(await file.text()) });
      await loadFloors(); flSay('楼层配置已登记。');
    } catch (error) { flSay(error.message, 'error'); }
    finally { flImport.value = ''; fl.busy = false; floorButtons(); }
  });

  /* ================= 正在执行（不论谁发起） ================= */
  function renderRunning() {
    const active = store.get().tasks.active;
    const navTask = active.find(task => ['navigate_to_pose', 'navigate_to_named_location', 'navigate_to_place', 'navigate_to_floor'].includes(task.capability));
    const exploreTask = active.find(task => EXPLORE_CAPS.includes(task.capability));
    // 探索页：有探索任务时任务块代替表单。
    exForm.hidden = Boolean(exploreTask);
    taskCards.place(exCard, exploreTask ? [exploreTask] : [], 'nav-explore');
    const other = tab === 'explore' ? navTask : navTask || exploreTask;
    running.hidden = !other;
    if (!other) { running.replaceChildren(); return; }
    if (!running.firstChild) running.append(h('div', { class: 'n-group-title' }, '正在执行'), h('div', { class: 'c-act-focus' }));
    taskCards.place(running.lastChild, [other], 'nav-running');
  }

  tasks.onUpdate(task => {
    applyPickTask(task);
    applyNamedTask(task);
    applyExploreTask(task);
    if (FLOOR_CAPS.has(task.capability) && (!fl.task || task.task_id === fl.task.task_id || isTerminalTaskState(fl.task.state))) { fl.task = task; renderFloorTask(); }
  });
  store.subscribe(['tasks'], () => { if (shown) renderRunning(); });
  setInterval(() => { if (shown && store.get().tasks.active.length) renderRunning(); }, 1000);
  renderSeg();
  setTab('pick');
  return {
    show(opts = {}) { shown = true; setTab(opts.tab && pages[opts.tab] ? opts.tab : tab); },
    hide() { shown = false; syncPicking(); },
  };
}
