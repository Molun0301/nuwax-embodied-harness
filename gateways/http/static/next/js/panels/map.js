/* 地图面板：当前地图卡片、已保存地图列表与地图详情、清除残影。
   从 map-manager.js 迁出，能力与参数不变：激活 load_map、重定位 relocalize_active_map、新建与续建 start_mapping、
   保存与另存 save_map、结束会话 end_map_session、删除 delete_map；激活、新建、续建、结束会话、删除先确认并写清后果；
   任务类按任务进度显示，失败写后端原因；当前会话随 geometry.state 更新。
   清除残影从 map-cleanup.js 迁出：对已保存地图离线预览删除候选（原图、删除候选、清除后三层点云），确认后另存为新地图，
   原图保留；改力度或换地图即作废旧预览。 */
import { h, icon, toast, confirmDialog } from '/static/shared/neh-ui.js';
import { isTerminalTaskState, GEOMETRY_PHASES } from '../status.js';
import { poseAt, trackPoseDrag } from '../stage/posedrag.js';

const BADGE = { off: '', busy: 'n-badge--accent', warn: 'n-badge--warn', ok: 'n-badge--ok', bad: 'n-badge--bad' };
const CLEANUP_LAYERS = [{ kind: 'source', label: '原图', color: '#748997', size: 1, checked: true },
  { kind: 'candidates', label: '删除候选', color: '#ff615c', size: 2, checked: true }, { kind: 'result', label: '清除后', color: '#66d6a5', size: 1, checked: false }];
// 换内容时略过条件为假的子元素（replaceChildren 会把 null 写成文字）。
const fill = (el, ...kids) => el.replaceChildren(...kids.filter(kid => kid != null && kid !== false));
const formatTime = value => (value ? new Date(value).toLocaleString('zh-CN', { hour12: false }) : '时间未知');
const formatSize = bytes => (bytes >= 1048576 ? (bytes / 1048576).toFixed(1) + ' MB' : Math.max(1, Math.round(bytes / 1024)) + ' KB');

export function mountMap(root, ctx) {
  const { api, stream, tasks, stage } = ctx;
  const state = { maps: [], phase: null, current: '', busy: false, task: null, page: 'list', detail: null, query: '', shown: false };
  const status = h('p', { class: 'c-status-line', role: 'status' });
  const view = h('div');
  root.append(view);
  const hasSession = () => Boolean(state.phase && state.phase !== 'idle');
  function say(text, level = '') { status.textContent = text; status.dataset.level = level; }
  async function load() {
    try {
      const data = await api.invoke('list_maps');
      state.maps = data.maps;
      state.phase = data.phase;
      state.current = data.map_name;
      render();
    } catch (error) { say('读取地图列表失败：' + error.message, 'error'); }
  }
  // 动作类能力直接返回结果；任务类能力返回任务，进度随任务更新。end() 在结束时调用（成功、失败或取消）。
  async function run(label, name, args, done, end) {
    if (state.busy) return;
    state.busy = true; render();
    say(label + '：已提交，等待网关应答。');
    try {
      const reply = await api.invoke(name, args);
      if (reply.task) { state.task = { id: reply.task.task_id, label, done, end }; say(label + '：' + reply.task.message); }
      else { say(done(reply)); toast(done(reply)); end?.(); await load(); }
    } catch (error) {
      say(label + '失败：' + error.message, 'error');
      end?.();
    } finally { state.busy = false; render(); }
  }
  tasks.onUpdate(task => {
    if (!state.task || task.task_id !== state.task.id) return;
    const { label, done, end } = state.task;
    if (!isTerminalTaskState(task.state)) { say(label + '：' + task.message); return; }
    state.task = null;
    end?.();
    if (task.state === 'succeeded') { const text = done(task.result || {}); say(text); toast(text); }
    else say(label + (task.state === 'cancelled' ? '已取消：' : '失败：') + task.message, 'error');
    void load();
  });
  stream.on('geometry.state', frame => {
    const snapshot = frame.payload;
    if (!snapshot || (snapshot.phase === state.phase && snapshot.map_name === state.current)) return;
    const listed = state.maps.some(map => map.name === snapshot.map_name);
    state.phase = snapshot.phase;
    state.current = snapshot.map_name;
    if (!state.shown) return;
    render();
    if (snapshot.map_name && !listed) void load();
  });

  const relocalized = result => '重定位成功' + (Number.isFinite(result.fitness) ? '，匹配度 ' + result.fitness.toFixed(3) : '');

  /* ---------- 指定位置后重定位 ----------
     舞台进入指定模式：按下处是机器狗位置，拖动方向是它面向的方向（stage/posedrag.js，与导航选点同一套），
     松手后画出位置与朝向标记，调用 relocalize_active_map({planar_seed: [x, y, yaw]})；重定位结束后标记消失。 */
  const seed = { picking: false };
  function showSeed(pose) { stage.map.setSeedPose(pose); window.pointcloud3d?.previewNavigationPoint(pose); }
  function startSeed() {
    if (stage.map.viewMode === 'graph') stage.setView('2d');
    seed.picking = true;
    stage.setPicking(true);
    window.__nuwaxResetMapInteraction?.();
    stage.hint(h('div', { class: 'n-float c-hint', html: icon('locate-fixed', 'n-icon--sm') }, '在地图上按下机器狗所在位置，拖动指出它面向的方向；Esc 取消',
      h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => cancelSeed() }, '取消')));
    say('指定位置：在地图上按下机器狗所在位置，按住拖出它面向的方向。');
  }
  function endSeed() { seed.picking = false; stage.setPicking(false); stage.hint(null); }
  function cancelSeed() { endSeed(); showSeed(null); say('已取消指定位置。'); }
  stage.mapLayer.addEventListener('pointerdown', event => {
    if (!seed.picking || event.button !== 0 || event.isPrimary === false) return;
    // 捕获阶段拦下这次按下，地图不旋转平移，也不点选物品或区域。
    event.stopPropagation();
    event.preventDefault();
    const view = stage.map.viewMode === '3d' ? '3d' : '2d';
    const point = poseAt(view, event);
    if (!point) { say('按下处没有地图，请在地图上按下机器狗所在位置。', 'warn'); return; }
    showSeed(point);
    if (view === '3d') window.pointcloud3d?.setBoundaryEditing(true);
    trackPoseDrag(stage.mapLayer, event, point, view, {
      onYaw: yaw => showSeed({ ...point, yaw }),
      onEnd: yaw => {
        if (view === '3d') window.pointcloud3d?.setBoundaryEditing(false);
        if (!seed.picking) return;
        if (yaw === undefined) {
          showSeed(null);
          say('需要按住拖出机器狗面向的方向，这次没有发送重定位。', 'warn');
          toast('按住后拖动，指出机器狗面向的方向', { tone: 'warn' });
          return;
        }
        endSeed();
        showSeed({ ...point, yaw });
        run('按指定位置重定位', 'relocalize_active_map', { planar_seed: [point.x, point.y, yaw] }, relocalized, () => showSeed(null));
      },
    });
  }, true);
  document.addEventListener('keydown', event => { if (seed.picking && event.key === 'Escape') { event.preventDefault(); cancelSeed(); } });

  /* ---------- 确认后执行 ---------- */
  async function activate(name) {
    if (!await confirmDialog({ title: '激活「' + name + '」', text: '会加载这张地图并重新定位，当前地图会话被替换；正在执行的导航与探索会停止。', confirmText: '激活并定位' })) return;
    state.page = 'list';
    run('激活「' + name + '」', 'load_map', { map_name: name }, result => (result.localized === false ? '已加载「' + name + '」，未定位：' + result.reason : '已激活「' + name + '」并完成定位'));
  }
  async function startMapping(name, continueSaved) {
    const ok = await confirmDialog({ title: continueSaved ? '在「' + name + '」上继续建图' : '新建地图「' + name + '」',
      text: (continueSaved ? '从机器人当前位置继续扫描，补全之前没走到的地方。' : '机器人从当前位置开始建图。') + '当前地图会话会被替换（已保存的地图内容保留）。',
      confirmText: continueSaved ? '继续建图' : '开始建图' });
    if (!ok) return;
    state.page = 'list';
    run(continueSaved ? '续建「' + name + '」' : '新建「' + name + '」', 'start_mapping', { map_name: name, continue_saved: continueSaved }, () => '已开始在「' + name + '」上建图');
  }
  async function remove(name) {
    const current = hasSession() && name === state.current ? '这是当前地图，会先结束地图会话。' : '';
    if (!await confirmDialog({ title: '删除地图「' + name + '」', text: current + '会同时删除这张地图同目录的记忆库（语义区域与空间记忆），删除后无法恢复。', confirmText: '删除地图', danger: true })) return;
    state.page = 'list';
    run('删除「' + name + '」', 'delete_map', { map_name: name }, () => '已删除地图「' + name + '」');
  }
  async function endSession() {
    if (!await confirmDialog({ title: '结束地图会话', text: '结束后机器人不再在这张地图上定位，导航和探索会停止；未保存的建图内容会丢失。地图本身保留，可随时重新激活。', confirmText: '结束会话' })) return;
    run('结束会话', 'end_map_session', {}, () => '已结束地图会话');
  }
  async function newMap() {
    const input = h('input', { class: 'n-input', placeholder: '例如 三楼办公区', maxlength: 64 });
    setTimeout(() => input.focus(), 40);
    const ok = await confirmDialog({ title: '新建地图', text: '机器人会从当前位置开始建图。当前地图会话会结束（已保存的地图内容保留）。',
      extra: h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '地图名称'), input), confirmText: '开始建图' });
    const name = input.value.trim();
    if (!ok || !name) return;
    state.page = 'list';
    run('新建「' + name + '」', 'start_mapping', { map_name: name, continue_saved: false }, () => '已开始在「' + name + '」上建图');
  }
  function saveAs(row) {
    const input = h('input', { class: 'n-input', placeholder: '另存为的新名称', maxlength: 64 });
    const confirm = () => {
      const name = input.value.trim();
      if (!name) { input.focus(); return; }
      row.replaceChildren();
      run('另存为「' + name + '」', 'save_map', { map_name: name }, result => '已另存为「' + result.map_name + '」');
    };
    const ok = h('button', { class: 'n-btn n-btn--sm n-btn--primary', type: 'button', onclick: confirm }, '另存');
    // 回车确认；Esc 只取消输入，不收起面板。
    input.addEventListener('keydown', event => { if (event.key === 'Enter') { event.preventDefault(); confirm(); } if (event.key === 'Escape') { event.preventDefault(); event.stopPropagation(); row.replaceChildren(); } });
    row.replaceChildren(h('div', { class: 'c-inline-form' }, input, ok, h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => row.replaceChildren() }, '取消')));
    input.focus();
  }

  /* ---------- 页面 ---------- */
  function render() {
    if (state.page === 'cleanup') return;   // 清除残影子页自己管理内容
    if (state.page === 'detail') { renderDetail(); return; }
    const session = hasSession();
    const [phaseLabel, phaseTone] = GEOMETRY_PHASES[state.phase] || GEOMETRY_PHASES.idle;
    const phaseClass = BADGE[phaseTone];
    const currentEntry = state.maps.find(map => map.name === state.current);
    const nameRow = h('div');
    const btn = (label, onclick, extra = {}) => h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: state.busy || !session, onclick, ...extra }, label);
    const card = h('div', { class: 'c-card' },
      h('div', { class: 'c-card-head' }, h('div', { class: 'c-hero-main' }, h('div', { class: 'n-meta' }, '当前地图'),
        h('div', { class: 'c-card-title' }, session ? state.current || '未命名地图' : '没有加载地图')), h('span', { class: 'n-badge ' + phaseClass }, phaseLabel)),
      session && currentEntry ? h('div', { class: 'c-card-stats' }, h('span', {}, '保存于 ', h('b', { class: 'n-num' }, currentEntry.saved ? formatTime(currentEntry.updated_at) : '尚未保存')),
        h('span', {}, '大小 ', h('b', { class: 'n-num' }, formatSize(currentEntry.size_bytes)))) : null,
      nameRow,
      session ? h('div', { class: 'c-card-actions' },
        btn('重定位', () => run('重定位', 'relocalize_active_map', {}, relocalized), { title: '在当前地图上重新估计机器人位置' }),
        btn('指定位置后重定位', startSeed, { title: '机器狗离建图起点较远、直接重定位失败时，在地图上指出它的位置和朝向作为初值再重定位' }),
        btn('保存', () => run('保存「' + state.current + '」', 'save_map', {}, result => '已保存地图「' + result.map_name + '」')),
        btn('另存为…', () => saveAs(nameRow)),
        btn('清除残影…', () => openCleanup(state.current), { disabled: state.busy || !currentEntry?.saved, title: currentEntry?.saved ? '去掉行人等移动物体留下的点，另存为新地图' : '先保存地图才能清除残影' }),
        h('button', { class: 'n-btn n-btn--sm n-btn--quiet', type: 'button', disabled: state.busy, onclick: endSession }, '结束会话'))
        : h('p', { class: 'n-meta c-gap-t' }, '在下面激活一张已保存的地图，或新建地图后遥控或自动探索走一圈再保存。'));
    const search = h('input', { class: 'n-input', type: 'search', placeholder: '搜索地图', value: state.query });
    const list = h('div', { class: 'n-list' });
    const fill = () => {
      state.query = search.value;
      const q = search.value.trim().toLowerCase();
      const maps = state.maps.filter(map => (map.display_name || map.name).toLowerCase().includes(q));
      list.replaceChildren(...(maps.length ? maps.map(map => {
        const current = session && map.name === state.current;
        const open = () => { state.page = 'detail'; state.detail = map.name; render(); };
        return h('div', { class: 'n-row n-row--hover c-maprow' },
          h('button', { class: 'c-maprow-main', type: 'button', onclick: open, title: '查看这张地图能做的事' }, h('span', { class: 'c-row-icon', html: icon('map', 'n-icon--sm') }),
            h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title', title: map.name }, map.display_name || map.name),
              h('div', { class: 'n-row-sub n-num' }, (map.saved ? formatTime(map.updated_at) : '未保存') + '　' + formatSize(map.size_bytes)))),
          current ? h('span', { class: 'n-badge n-badge--accent' }, '当前') : h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: state.busy, onclick: () => activate(map.name) }, '激活'),
          h('button', { class: 'n-btn n-btn--quiet n-btn--icon n-btn--sm', type: 'button', title: '继续建图、清除残影、删除', 'aria-label': '打开「' + (map.display_name || map.name) + '」', html: icon('chevron-right', 'n-icon--sm'), onclick: open }));
      }) : [h('p', { class: 'n-meta' }, state.maps.length ? '没有匹配的地图。' : '还没有地图。点“新建地图”开始建图。')]));
    };
    search.addEventListener('input', fill);
    fill();
    view.replaceChildren(h('div', { class: 'c-pane-head' }, h('h2', {}, '地图'),
      h('div', { class: 'c-pane-actions' }, h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: state.busy, html: icon('plus', 'n-icon--sm'), onclick: newMap }, '新建地图'))),
      card, status,
      h('div', { class: 'n-group-title' }, '已保存的地图（' + state.maps.length + '）', h('span', { class: 'n-group-actions' }, h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', onclick: () => load().then(() => say('地图列表已刷新。')) }, '刷新'))),
      h('div', { class: 'c-gap-b' }, search), list);
  }
  function renderDetail() {
    const map = state.maps.find(item => item.name === state.detail);
    if (!map) { state.page = 'list'; render(); return; }
    const current = hasSession() && map.name === state.current;
    const act = (ic, title, desc, onclick, danger = false, disabled = false) => h('button', { class: 'n-row n-row--hover c-actrow' + (danger ? ' c-actrow--danger' : ''), type: 'button', onclick, disabled: disabled || state.busy },
      h('span', { class: 'c-actrow-icon', html: icon(ic, 'n-icon--sm') }), h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, title), h('div', { class: 'n-row-sub c-wrap' }, desc)),
      h('span', { class: 'c-rowchev', html: icon('chevron-right', 'n-icon--sm') }));
    fill(view, 
      h('button', { class: 'c-back', type: 'button', html: icon('chevron-left', 'n-icon--sm'), onclick: () => { state.page = 'list'; render(); } }, '已保存的地图'),
      h('div', { class: 'c-pane-head' }, h('h2', { title: map.name }, map.display_name || map.name), current ? h('span', { class: 'n-badge n-badge--accent' }, '当前') : null,
        map.saved ? null : h('span', { class: 'n-badge n-badge--warn' }, '未保存')),
      h('dl', { class: 'n-kv c-gap-b' }, h('dt', {}, '保存于'), h('dd', { class: 'n-num' }, map.saved ? formatTime(map.updated_at) : '尚未保存'), h('dt', {}, '大小'), h('dd', { class: 'n-num' }, formatSize(map.size_bytes))),
      current ? null : h('button', { class: 'n-btn n-btn--primary n-btn--block c-gap-b', type: 'button', disabled: state.busy, onclick: () => activate(map.name) }, '激活并定位'),
      h('div', { class: 'n-list' },
        act('pencil', '在此地图上继续建图', '从机器人当前位置继续扫描，补全之前没走到的地方；当前地图会话会被替换。', () => startMapping(map.name, true)),
        act('eraser', '清除残影', map.saved ? '去掉行人等移动物体留下的点，另存为新地图，原图保留。' : '这张地图还没保存，保存后才能清除残影。', () => openCleanup(map.name), false, !map.saved),
        act('trash-2', '删除地图', '同时删除这张地图的语义区域与空间记忆，无法恢复。', () => remove(map.name), true)),
      status);
  }

  /* ---------- 清除残影 ---------- */
  const cl = { job: null, task: null, epoch: 0, timer: null, points: {}, previewReady: false, cancelling: null, submitting: false, angle: -0.6, tilt: 0.65, zoom: 1, source: '' };
  const clCanvas = h('canvas', { class: 'c-cleanup-canvas', hidden: true, 'aria-label': '原图、删除候选和清除后三层点云预览：拖动旋转，滚轮缩放' });
  const clLegend = h('div', { class: 'c-cleanup-legend', hidden: true }, CLEANUP_LAYERS.map(layer => {
    const box = h('input', { type: 'checkbox', checked: layer.checked, dataset: { kind: layer.kind } });
    box.addEventListener('change', drawCleanup);
    return h('label', { class: 'n-check', style: 'color:' + layer.color }, box, layer.label);
  }));
  const clPct = h('output', { class: 'n-num' }, '50%');
  const clStrength = h('input', { type: 'range', min: 0, max: 100, step: 1, value: 50, class: 'c-range', 'aria-label': '清除力度：0% 保守，100% 激进' });
  const clStatus = h('p', { class: 'c-status-line', role: 'status' });
  const clDetails = h('pre', { class: 'n-pre c-cleanup-details', hidden: true });
  const clPreview = h('button', { class: 'n-btn n-btn--block', type: 'button', onclick: () => previewCleanup() }, '预览删除候选');
  const clCancel = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: async () => { if (await invalidate()) clSay('预览已取消，工作目录已删除。'); } }, '取消预览');
  const clName = h('input', { class: 'n-input' });
  const clApply = h('button', { class: 'n-btn n-btn--primary n-btn--block', type: 'button', onclick: () => applyCleanup() }, '另存为新地图');
  const clSay = text => { clStatus.textContent = text; };
  const cleanupApi = (path, body) => api.json('/api/maps/cleanup' + path, body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const taskActive = () => cl.task !== null && !isTerminalTaskState(cl.task.state);
  function clButtons() {
    const saving = cl.submitting || taskActive();
    clApply.disabled = saving || !cl.previewReady || cl.job?.state !== 'ready' || Boolean(cl.cancelling);
    clPreview.disabled = saving || Boolean(cl.cancelling);
    clStrength.disabled = saving;
    clCancel.disabled = saving || Boolean(cl.cancelling) || !cl.job || ['applied', 'cancelled'].includes(cl.job.state);
  }
  function clearPreview() {
    cl.job = cl.task = null; cl.points = {}; cl.previewReady = false;
    clCanvas.hidden = clLegend.hidden = true;
    clDetails.hidden = true; clDetails.textContent = '';
    clButtons();
  }
  async function invalidate() {
    if (cl.cancelling) return cl.cancelling;
    const old = cl.job, visible = cl.previewReady, epoch = ++cl.epoch;
    clearTimeout(cl.timer);
    cl.previewReady = false;
    if (!old || ['applied', 'cancelled'].includes(old.state)) { clearPreview(); return true; }
    cl.cancelling = (async () => {
      try { await cleanupApi('/' + encodeURIComponent(old.id) + '/cancel', {}); clearPreview(); return true; }
      catch (error) { cl.previewReady = visible; clSay(error.message); if (['queued', 'running'].includes(old.state)) schedule(old.id, epoch); return false; }
      finally { cl.cancelling = null; clButtons(); }
    })();
    clButtons();
    return cl.cancelling;
  }
  function schedule(id, epoch) { clearTimeout(cl.timer); cl.timer = setTimeout(() => poll(id, epoch), 1000); }
  async function loadPoints(job, epoch) {
    const values = await Promise.all(CLEANUP_LAYERS.map(async layer => {
      const response = await fetch('/api/maps/cleanup/' + encodeURIComponent(job.id) + '/points/' + layer.kind, { credentials: 'same-origin' });
      if (!response.ok) throw new Error(layer.label + '点云读取失败');
      return [layer.kind, new Float32Array(await response.arrayBuffer())];
    }));
    if (epoch !== cl.epoch) return;
    cl.points = Object.fromEntries(values); cl.zoom = 1;
    clCanvas.hidden = clLegend.hidden = false;
    cl.previewReady = true; drawCleanup(); clButtons();
  }
  async function poll(id, epoch) {
    try {
      const taskId = cl.task?.task_id;
      const [job, value] = await Promise.all([cleanupApi('/' + encodeURIComponent(id)), taskId ? api.json('/api/tasks/' + encodeURIComponent(taskId)) : Promise.resolve(null)]);
      if (epoch !== cl.epoch) return;
      cl.job = job;
      if (value !== null) cl.task = value.task;
      if (cl.task !== null) {
        if (cl.task.state === 'succeeded') { cl.job = cl.task.result; clSay(`已另存为「${cl.job.result_map}」，原图「${cl.job.map_name}」保留，当前地图不变。`); void load(); }
        else if (isTerminalTaskState(cl.task.state)) { if (job.state === 'cancelled') clearPreview(); clSay(cl.task.message); }
        else { clSay(cl.task.message || '另存任务正在执行。'); schedule(id, epoch); }
      } else if (job.state === 'ready') {
        clDetails.textContent = `力度 ${job.strength_percent}%。勾选切换原图、删除候选和清除后；拖动旋转，滚轮缩放。大图按显示抽样。\n工具统计摘要：\n` + JSON.stringify(job.report, null, 2);
        clDetails.hidden = false;
        if (!cl.previewReady) await loadPoints(job, epoch);
        if (epoch === cl.epoch) clSay('预览已就绪：红色是删除候选。核对后填写新地图名称并另存。');
      } else if (job.state === 'applied') { clSay(`已另存为「${job.result_map}」，原图「${job.map_name}」保留。`); }
      else if (job.state === 'cancelled') { clearPreview(); clSay('预览已取消。'); }
      else if (job.state === 'failed') { clSay(job.error); }
      else { clSay('正在计算删除候选。'); schedule(id, epoch); }
      clButtons();
    } catch (error) {
      if (epoch === cl.epoch) {
        clSay(error.message); clButtons();
        if (taskActive() || ['queued', 'running', 'applying'].includes(cl.job?.state)) schedule(id, epoch);
      }
    }
  }
  async function previewCleanup() {
    if (!await invalidate()) return;
    const epoch = cl.epoch;
    clSay('正在准备预览。');
    try {
      const job = await cleanupApi('', { map_name: cl.source, strength_percent: Number(clStrength.value) });
      if (epoch !== cl.epoch) { await cleanupApi('/' + encodeURIComponent(job.id) + '/cancel', {}); return; }
      cl.job = job; clButtons(); await poll(job.id, epoch);
    } catch (error) { if (epoch === cl.epoch) clSay(error.message); }
  }
  async function applyCleanup() {
    const job = cl.job, name = clName.value.trim(), epoch = cl.epoch;
    if (!job || job.state !== 'ready' || !cl.previewReady) return;
    if (!name) { clSay('请填写新地图名称。'); clName.focus(); return; }
    if (!await confirmDialog({ title: '另存为新地图', text: `按 ${job.strength_percent}% 力度去掉删除候选，另存为「${name}」。原图「${job.map_name}」保留，当前地图不变。`, confirmText: '另存为新地图' })) return;
    cl.submitting = true; clButtons(); clSay('正在提交另存任务。');
    try {
      const submitted = await cleanupApi('/' + encodeURIComponent(job.id) + '/apply', { new_map_name: name, confirmed: true });
      cl.task = submitted.task;
      clSay(cl.task.message || '另存任务已提交，等待执行结果。');
      await poll(job.id, epoch);
    } catch (error) { clSay(error.message); }
    finally { cl.submitting = false; clButtons(); }
  }
  function drawCleanup() {
    const context = clCanvas.getContext('2d');
    clCanvas.width = Math.max(280, clCanvas.clientWidth || 320); clCanvas.height = 240;
    context.fillStyle = '#111b24'; context.fillRect(0, 0, clCanvas.width, clCanvas.height);
    const reference = CLEANUP_LAYERS.map(layer => cl.points[layer.kind]).find(points => points?.length);
    if (!reference) return;
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < reference.length; i += 3) for (let a = 0; a < 3; a++) { lo[a] = Math.min(lo[a], reference[i + a]); hi[a] = Math.max(hi[a], reference[i + a]); }
    const center = lo.map((value, axis) => (value + hi[axis]) / 2);
    const span = Math.max(...hi.map((value, axis) => value - lo[axis]), 0.1);
    const scale = Math.min(clCanvas.width, clCanvas.height) * 0.8 / span * cl.zoom;
    const ca = Math.cos(cl.angle), sa = Math.sin(cl.angle), ct = Math.cos(cl.tilt), st = Math.sin(cl.tilt);
    for (const layer of CLEANUP_LAYERS) {
      if (!clLegend.querySelector(`[data-kind="${layer.kind}"]`).checked) continue;
      const points = cl.points[layer.kind];
      if (!points) continue;
      context.fillStyle = layer.color;
      const stride = Math.max(1, Math.ceil(points.length / 3 / 200000)) * 3;
      for (let i = 0; i < points.length; i += stride) {
        const x = points[i] - center[0], y = points[i + 1] - center[1], z = points[i + 2] - center[2];
        context.fillRect(clCanvas.width / 2 + (x * ca - y * sa) * scale, clCanvas.height / 2 + ((x * sa + y * ca) * st - z * ct) * scale, layer.size, layer.size);
      }
    }
  }
  let drag = null;
  clCanvas.addEventListener('pointerdown', event => { drag = [event.clientX, event.clientY]; clCanvas.setPointerCapture(event.pointerId); });
  clCanvas.addEventListener('pointermove', event => {
    if (!drag) return;
    cl.angle += (event.clientX - drag[0]) * 0.01;
    cl.tilt = Math.max(-1.5, Math.min(1.5, cl.tilt + (event.clientY - drag[1]) * 0.01));
    drag = [event.clientX, event.clientY]; drawCleanup();
  });
  clCanvas.addEventListener('pointerup', () => { drag = null; });
  clCanvas.addEventListener('pointercancel', () => { drag = null; });
  window.addEventListener('resize', () => { if (state.page === 'cleanup' && cl.previewReady) drawCleanup(); });
  clCanvas.addEventListener('wheel', event => { event.preventDefault(); cl.zoom = Math.max(0.1, Math.min(20, cl.zoom * Math.exp(-event.deltaY * 0.001))); drawCleanup(); }, { passive: false });
  clStrength.addEventListener('input', async () => { clPct.textContent = clStrength.value + '%'; if (await invalidate()) clSay('力度已改变，请重新预览。'); });
  async function openCleanup(name) {
    const from = state.page;
    if (cl.source !== name) { await invalidate(); cl.source = name; clName.value = name + '-去残影'; clSay('选择清除力度，点“预览删除候选”。'); }
    state.page = 'cleanup';
    view.replaceChildren(
      h('button', { class: 'c-back', type: 'button', html: icon('chevron-left', 'n-icon--sm'), onclick: () => { state.page = from === 'detail' ? 'detail' : 'list'; render(); } }, from === 'detail' ? '返回「' + name + '」' : '地图'),
      h('div', { class: 'c-pane-head' }, h('h2', {}, '清除残影')),
      h('p', { class: 'n-muted c-gap-b' }, '对已保存的地图离线去掉行人等移动物体留下的点。先预览删除候选，确认后另存为新地图，原图保留，不切换当前地图。'),
      h('dl', { class: 'n-kv c-gap-b' }, h('dt', {}, '地图'), h('dd', {}, name)),
      h('div', { class: 'n-field' }, h('div', { class: 'c-field-head' }, h('span', { class: 'n-label' }, '清除力度'), clPct), clStrength,
        h('div', { class: 'c-range-ends' }, h('span', {}, '保守，只删明显残影'), h('span', {}, '激进，可能误删'))),
      h('div', { class: 'c-row-actions c-gap-t' }, clPreview, clCancel),
      clStatus, clLegend, clCanvas, clDetails,
      h('label', { class: 'n-field c-gap-t' }, h('span', { class: 'n-label' }, '新地图名称'), clName), h('div', { class: 'c-gap-t' }, clApply));
    clButtons();
    drawCleanup();
  }

  return {
    show() { state.shown = true; render(); void load(); },
    hide() { state.shown = false; if (seed.picking) cancelSeed(); },
  };
}
