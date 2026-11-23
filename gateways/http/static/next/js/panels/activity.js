/* 活动面板（方案 5.3）：焦点是执行中的任务；空闲时问“让机器人去哪里？”并给出常用操作；
   下面是需要注意（只收警告、错误与已退出的服务，不收网页请求日志）、最近结束，底部固定服务区。
   任务、需要注意、服务都读统一状态，不论网页还是智能体发起。 */
import { icon, h, toast } from '/static/shared/neh-ui.js';
import { describeTask, fmtDuration, fmtClock, taskSource, normalizeTaskState } from '../status.js';

// 定位建图或导航栈退出时不能导航（遥控移动不依赖定位）。
const NAVIGATION_COMPONENTS = ['geometry', 'nav_stack'];

export function mountActivity(root, ctx) {
  const { store, api, tasks, go } = ctx;
  const focusBox = h('div', { class: 'c-act-focus' });
  const attentionBox = h('section', { class: 'n-sec c-act-sec', id: 'attention' });
  const recentBox = h('section', { class: 'n-sec c-act-sec' });
  const servicesBox = h('section', { class: 'c-svcs' });
  root.append(h('div', { class: 'c-act' }, focusBox, attentionBox, recentBox, servicesBox));
  let capabilities = null;
  api.capabilities().then(list => { capabilities = new Map(list.map(cap => [cap.name, cap])); render(); }).catch(() => {});

  /* ---------- 焦点：执行中的任务 ---------- */
  // 任务块按秒刷新用时与进度：结构（标题、按钮、取消状态）不变时只改文字，避免重建吞掉点击。
  const cards = new Map();
  function taskView(task) {
    const s = store.get();
    return describeTask(task, { navRange: tasks.navRange, clockOffsetMs: s.clockOffsetMs, now: Date.now(), pending: s.tasks.pending, errors: s.tasks.errors, capabilities });
  }
  function taskStats(task, view) {
    const exploration = task.data?.exploration;
    const pct = view.progress == null ? null : Math.round(view.progress * 100);
    const stats = [];
    if (view.kind === 'nav' && Number.isFinite(Number(task.data?.remaining_distance_m))) stats.push([Number(task.data.remaining_distance_m).toFixed(1) + ' 米', '剩余']);
    if (view.kind === 'explore' && Number.isFinite(Number(exploration?.traveled_distance_m))) stats.push([Number(exploration.traveled_distance_m).toFixed(1) + ' 米', '已行驶']);
    if (view.kind === 'explore' && Number.isFinite(Number(exploration?.frontier_count))) stats.push([String(exploration.frontier_count), '剩余边界']);
    stats.push([fmtDuration(view.elapsedMs), '用时']);
    if (pct != null) stats.push([pct + '%', '进度']);
    return { pct, stats };
  }
  function taskCard(task) {
    const view = taskView(task);
    const key = [view.kind, view.target, view.cancelling, view.cancelError, view.title].join('|');
    const existing = cards.get(task.task_id);
    if (existing && existing.key === key) { existing.update(task); return existing.el; }
    const title = view.kind === 'nav' ? (view.target ? '导航到「' + view.target + '」' : '导航') : view.kind === 'explore' ? '自动探索' + (view.target ? '「' + view.target + '」' : '') : view.title;
    const stage = h('div', { class: 'n-meta' });
    const bar = h('span', {});
    const progress = h('div', { class: 'n-progress' }, bar);
    const statsBox = h('div', { class: 'c-hero-stats' });
    const actions = [];
    // 探索任务一经受理即可暂停与停止；探索尚未启动时不带编号，由后端按当前探索任务处理。
    if (view.kind === 'explore') actions.push(h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: view.cancelling, onclick: () => pauseExploration(cards.get(task.task_id).task) }, '暂停并保存'));
    actions.push(h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: view.cancelling, onclick: () => tasks.stop(cards.get(task.task_id).task) }, view.cancelling ? '正在取消…' : view.stopLabel));
    const el = h('div', { class: 'n-hl c-hero', dataset: { task: task.task_id } },
      h('div', { class: 'c-hero-head' }, h('div', { class: 'c-hero-main' }, h('div', { class: 'c-hero-title' }, title), stage), h('span', { class: 'c-src' }, view.source + '发起')),
      progress, statsBox,
      view.cancelError ? h('div', { class: 'n-meta', style: 'color:var(--bad)' }, '取消失败：' + view.cancelError) : null,
      h('div', { class: 'c-card-actions' }, actions));
    const card = { el, key, task, update(next) {
      card.task = next;
      const { pct, stats } = taskStats(next, taskView(next));
      stage.textContent = stageText(next);
      progress.classList.toggle('n-progress--indeterminate', pct == null);
      bar.style.width = (pct ?? 0) + '%';
      statsBox.replaceChildren(...stats.map(([value, label]) => h('span', {}, h('b', { class: 'n-num' }, value), label)));
    } };
    card.update(task);
    cards.set(task.task_id, card);
    return el;
  }
  function stageText(task) {
    const state = normalizeTaskState(task.state);
    if (state === 'cancelling') return task.message || '正在取消';
    return task.stage !== 'running' && task.message ? task.message.replace(/。$/, '') : '已受理，正在执行';
  }
  async function pauseExploration(task) {
    const explorationId = task.data?.exploration_id;
    try {
      const reply = await api.invoke('pause_exploration', explorationId ? { exploration_id: explorationId } : {});
      toast(reply.checkpoint_pending ? '暂停已受理，正在停止探索并保存地图与进度' : '探索尚未开始，已结束任务，没有可保存的进度', { tone: 'info' });
    } catch (error) {
      toast('暂停探索失败：' + error.message, { tone: 'bad' });
    }
  }

  /* ---------- 焦点：空闲时去哪里 ---------- */
  const ask = h('input', { class: 'n-input c-ask', placeholder: '输入区域、物品或地点名称', autocomplete: 'off', 'aria-label': '目的地名称' });
  const suggestions = h('div', { class: 'n-list c-ask-list' });
  let names = [], namesLoaded = false;
  async function loadNames() {
    // 名称候选取当前地图记忆里的标记地点与区域（与原导航面板“按名称”相同）。
    const mapName = window.__nuwaxMapBinding?.().map_name;
    const [locations, spaces] = await Promise.allSettled([api.json('/api/memory/locations?limit=500'),
      api.invoke('list_scene_spaces', mapName ? { limit: 500, map_name: mapName } : { limit: 500 })]);
    const found = new Map();
    for (const item of locations.value?.locations || []) for (const name of [item.name, ...(item.aliases || [])]) if (name) found.set(name, '地点');
    for (const item of spaces.value?.scene_spaces || []) for (const name of [item.primary_label, ...(item.human_aliases || [])]) if (name) found.set(name, '区域');
    names = [...found].sort((a, b) => a[0].localeCompare(b[0], 'zh-CN'));
    namesLoaded = true;
    fillSuggestions();
  }
  function fillSuggestions() {
    const q = ask.value.trim();
    const list = q ? names.filter(([name]) => name.includes(q)).slice(0, 6) : [];
    suggestions.replaceChildren(...list.map(([name, kind]) => h('button', { class: 'n-row', type: 'button', onclick: () => navigateTo(name) },
      h('span', { class: 'c-ask-kind', html: icon(kind === '区域' ? 'land-plot' : 'map-pin', 'n-icon--sm') }),
      h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, name), h('div', { class: 'n-row-sub' }, kind)), h('span', { class: 'n-meta' }, '前往'))));
  }
  ask.addEventListener('focus', () => { if (!namesLoaded) loadNames().catch(() => {}); });
  ask.addEventListener('input', fillSuggestions);
  ask.addEventListener('keydown', event => { if (event.key === 'Enter' && ask.value.trim()) navigateTo(ask.value.trim()); });
  async function navigateTo(name) {
    try {
      await api.invoke('navigate_to_named_location', { target_name: name, verify_on_arrival: true });
      ask.value = '';
      suggestions.replaceChildren();
    } catch (error) {
      toast('前往「' + name + '」失败：' + error.message, { tone: 'bad' });
    }
  }
  function quickActions(disabledNav) {
    const motion = !capabilities || capabilities.has('relative_move');
    return h('div', { class: 'c-quick' },
      h('button', { class: 'n-btn', type: 'button', disabled: disabledNav, html: icon('target'), onclick: () => go('monitor', 'nav', { open: true, tab: 'pick' }) }, '地图选点'),
      h('button', { class: 'n-btn', type: 'button', disabled: disabledNav, html: icon('compass'), onclick: () => go('monitor', 'nav', { open: true, tab: 'explore' }) }, '自动探索'),
      h('button', { class: 'n-btn', type: 'button', disabled: !motion, title: motion ? '' : '这台机器人没有移动能力', html: icon('move'), onclick: () => go('monitor', 'motion', { open: true }) }, '遥控移动'));
  }
  let focusKey = '';
  function renderFocus() {
    const s = store.get();
    const exitedNav = Object.entries(s.services || {}).filter(([name, item]) => NAVIGATION_COMPONENTS.includes(name) && item.state === 'exited');
    let key, build;
    if (s.business !== 'running') {
      key = 'business:' + s.business;
      build = () => [h('div', { class: 'n-hl n-hl--bad c-hero' },
        h('div', { class: 'c-hero-title' }, s.business === 'updating' ? '正在切换到新版本' : '业务服务已停止'),
        h('div', { class: 'n-meta' }, '导航、探索与遥控暂不可用；已保存的地图、语义区域和配置不受影响。'),
        h('div', {}, h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => go('system') }, '打开系统页')))];
    } else if (s.tasks.active.length) {
      key = 'tasks';
      build = () => {
        const list = s.tasks.active.slice().sort((a, b) => Number(b.created_at || 0) - Number(a.created_at || 0));
        for (const id of cards.keys()) if (!list.some(task => task.task_id === id)) cards.delete(id);
        return list.map(taskCard);
      };
    } else if (exitedNav.length) {
      key = 'blocked';
      build = () => [h('div', { class: 'c-hero' }, h('div', {}, h('div', { class: 'c-hero-title' }, '暂时不能导航'),
        h('div', { class: 'n-meta' }, exitedNav.map(([name, item]) => item.title || name).join('、') + '已退出，恢复后才能前往目的地或自动探索；遥控移动不受影响。')),
        quickActions(true))];
    } else {
      const noMap = !s.geometry.mapName || s.geometry.phase === 'idle';
      key = 'idle:' + noMap + ':' + Boolean(capabilities);
      build = () => [h('div', { class: 'c-hero' },
        h('div', { class: 'c-hero-title' }, noMap ? '先加载一张地图' : '让机器人去哪里？'),
        noMap ? h('div', { class: 'n-meta' }, '在“地图”页激活已有地图或新建地图后，就可以下达任务。') : null,
        noMap ? null : h('div', { class: 'n-search', html: icon('navigation', 'n-icon--sm') }, ask), noMap ? null : suggestions,
        quickActions(noMap))];
    }
    // 任务块按秒刷新（用时）；空闲表单只在状态变化时重建，避免打断输入。
    if (key === focusKey && key !== 'tasks') return;
    focusKey = key;
    const active = document.activeElement;
    const nodes = build();
    // 已在页面上的任务块原地保留（只调整顺序），新块插入。
    if (key !== 'tasks' || nodes.some((node, index) => focusBox.children[index] !== node) || focusBox.children.length !== nodes.length) focusBox.replaceChildren(...nodes);
    if (key !== 'tasks') cards.clear();
    if (active === ask && ask.isConnected) ask.focus();
  }

  /* ---------- 需要注意 ---------- */
  let attentionKey = '';
  function renderAttention() {
    const list = store.get().attention;
    const key = list.map(item => item.id).join('|');
    if (key === attentionKey) return;
    attentionKey = key;
    attentionBox.hidden = !list.length;
    attentionBox.replaceChildren(h('div', { class: 'n-sec-head' }, h('h3', {}, '需要注意'), h('span', { class: 'n-meta' }, list.length + ' 条')),
      ...list.slice(0, 20).map(item => h('div', { class: 'n-hl c-attn ' + (item.tone === 'bad' ? 'n-hl--bad' : 'n-hl--warn') },
        h('span', { class: 'c-attn-icon', dataset: { tone: item.tone }, html: icon(item.tone === 'bad' ? 'circle-alert' : 'triangle-alert', 'n-icon--sm') }),
        h('div', { class: 'c-attn-main' }, h('div', { class: 'c-attn-text' }, item.text), h('div', { class: 'n-meta' }, fmtClock(item.at) + (item.comp ? '　来自 ' + item.comp : ''))),
        h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', title: '在日志页查看这个组件的记录', onclick: () => go('logs', '', { component: item.comp, taskId: item.taskId, requestId: item.requestId }) }, '日志'))));
  }

  /* ---------- 最近结束 ---------- */
  let recentKey = '';
  function renderRecent() {
    const list = store.get().tasks.history.slice(0, 4);
    const key = list.map(task => task.task_id + task.state).join('|');
    if (key === recentKey) return;
    recentKey = key;
    recentBox.hidden = !list.length;
    recentBox.replaceChildren(h('div', { class: 'n-sec-head' }, h('h3', {}, '最近结束')),
      ...list.map(task => {
        const state = normalizeTaskState(task.state);
        const ok = state === 'succeeded', failed = state === 'failed' || state === 'timed_out';
        const view = describeTask(task, { navRange: tasks.navRange, clockOffsetMs: 0, now: 0, pending: new Set(), errors: new Map(), capabilities });
        const name = view.kind === 'nav' ? '导航' + (view.target ? '「' + view.target + '」' : '') : view.kind === 'explore' ? '自动探索' + (view.target ? '「' + view.target + '」' : '') : view.title;
        const result = ok ? '完成' : failed ? (state === 'timed_out' ? '超时' : '失败') : '已取消';
        return h('div', { class: 'c-line-row' },
          h('span', { class: 'c-line-icon', dataset: { tone: ok ? 'ok' : failed ? 'bad' : '' }, html: icon(ok ? 'check' : failed ? 'circle-alert' : 'x', 'n-icon--sm') }),
          h('div', { class: 'c-line-main' }, h('div', { class: 'c-line-title' }, name + result, h('span', { class: 'c-src' }, taskSource(task))),
            !ok && task.message ? h('div', { class: 'n-meta' }, task.message) : null),
          h('span', { class: 'c-feed-time n-num' }, task.finished_at ? fmtClock(task.finished_at * 1000) : ''));
      }));
  }

  /* ---------- 服务（底部固定） ---------- */
  let servicesKey = '';
  function renderServices() {
    const { services, business } = store.get();
    const key = JSON.stringify([business, services && Object.entries(services).map(([name, item]) => [name, item.state, item.enabled, item.title, item.state_title])]);
    if (key === servicesKey) return;
    servicesKey = key;
    const down = business !== 'running';
    const entries = Object.entries(services || {});
    const running = entries.filter(([, item]) => item.state === 'running').length;
    const exited = entries.filter(([, item]) => item.state === 'exited').length;
    const summary = down ? '业务服务未运行' : exited ? exited + ' 个已退出' : services === null ? '正在读取' : entries.length ? running + ' 个运行中' : '';
    servicesBox.replaceChildren(
      h('div', { class: 'n-sec-head' }, h('h3', {}, '服务'), h('span', { class: 'n-meta', style: exited || down ? 'color:var(--bad)' : '' }, summary),
        h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', style: 'margin-left:auto', title: '在系统页查看进程、资源与日志', onclick: () => go('system') }, '管理')),
      entries.length ? h('div', { class: 'c-svc-grid' }, entries.map(([name, item]) => {
        // 状态与中文名来自监督者：运行绿点；按需启动未运行空心圈；未启用灰点；已退出红点红字；启动中青点。
        const [tone, note] = down ? ['off', ''] : !item.enabled ? ['off', '未启用'] : item.state === 'running' ? ['ok', '']
          : item.state === 'exited' ? ['bad', '已退出'] : item.state === 'starting' ? ['busy', '启动中'] : ['idle', item.state_title?.includes('按需') ? '按需' : '未运行'];
        const tip = [item.title || name, item.state_title, item.exit_code != null ? '退出码 ' + item.exit_code : '', item.error || ''].filter(Boolean).join('：');
        return h('button', { class: 'c-svc-item', type: 'button', dataset: { tone }, title: tip, onclick: () => go('system') },
          h('i', { class: 'c-svc-dot' }), h('span', { class: 'c-svc-label' }, item.title || name), note ? h('span', { class: 'c-svc-note' }, note) : null);
      })) : h('div', { class: 'n-meta c-svc-empty' }, services === null ? '正在读取服务状态。' : '监督者没有报告服务（网关不是由业务监督者启动的）。'));
  }

  function render() { renderFocus(); renderAttention(); renderRecent(); renderServices(); }
  store.subscribe(['business', 'tasks', 'services', 'geometry', 'attention', 'clockOffsetMs'], render);
  setInterval(() => { if (!root.hidden && store.get().tasks.active.length) renderFocus(); }, 1000);
  render();
  return { show(opts = {}) { if (opts.focus) setTimeout(() => root.querySelector('#' + opts.focus)?.scrollIntoView({ block: 'start', behavior: 'smooth' }), 30); } };
}
