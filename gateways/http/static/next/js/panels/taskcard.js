/* 任务块：活动、导航、探索页共用。标题、阶段、进度、剩余或已行驶、剩余边界、用时、进度百分比、发起方与操作。
   每秒刷新用时与进度时只改文字，结构（标题、按钮、取消状态）变化才重建，避免重建吞掉点击。 */
import { h, toast } from '/static/shared/neh-ui.js';
import { describeTask, fmtDuration, normalizeTaskState } from '../status.js';

export function createTaskCards(ctx) {
  const { store, api, tasks } = ctx;
  const cards = new Map();
  let capabilities = null;
  api.capabilities().then(list => { capabilities = new Map(list.map(cap => [cap.name, cap])); }).catch(() => {});
  function view(task) {
    const s = store.get();
    return describeTask(task, { navRange: tasks.navRange, clockOffsetMs: s.clockOffsetMs, now: Date.now(), pending: s.tasks.pending, errors: s.tasks.errors, capabilities });
  }
  function stageText(task) {
    if (normalizeTaskState(task.state) === 'cancelling') return task.message || '正在取消';
    return task.stage !== 'running' && task.message ? task.message.replace(/。$/, '') : '已受理，正在执行';
  }
  function stats(task, v) {
    const exploration = task.data?.exploration;
    const pct = v.progress == null ? null : Math.round(v.progress * 100);
    const out = [];
    if (v.kind === 'nav' && Number.isFinite(Number(task.data?.remaining_distance_m))) out.push([Number(task.data.remaining_distance_m).toFixed(1) + ' 米', '剩余']);
    if (v.kind === 'explore' && Number.isFinite(Number(exploration?.traveled_distance_m))) out.push([Number(exploration.traveled_distance_m).toFixed(1) + ' 米', exploration.returning ? '已行驶，返航中' : '已行驶']);
    if (v.kind === 'explore' && Number.isFinite(Number(exploration?.frontier_count))) out.push([String(exploration.frontier_count), '剩余边界']);
    out.push([fmtDuration(v.elapsedMs), '用时']);
    if (pct != null) out.push([pct + '%', '进度']);
    return { pct, items: out };
  }
  /* 探索暂停：任务一经受理即可暂停；探索尚未启动时不带编号，由后端按当前探索任务处理。 */
  async function pauseExploration(task) {
    const explorationId = task.data?.exploration_id;
    try {
      const reply = await api.invoke('pause_exploration', explorationId ? { exploration_id: explorationId } : {});
      toast(reply.checkpoint_pending ? '暂停已受理，正在停止探索并保存地图与进度' : '探索尚未开始，已结束任务，没有可保存的进度', { tone: 'info' });
    } catch (error) {
      toast('暂停探索失败：' + error.message, { tone: 'bad' });
    }
  }
  /* 返回任务块元素；scope 区分不同页面的同一任务（各自一份元素）。 */
  function card(task, scope = '') {
    const v = view(task);
    const key = [v.kind, v.target, v.cancelling, v.cancelError, v.title].join('|');
    const id = scope + '|' + task.task_id;
    const existing = cards.get(id);
    if (existing && existing.key === key) { existing.update(task); return existing.el; }
    const title = v.kind === 'nav' ? (v.target ? '导航到「' + v.target + '」' : '导航') : v.kind === 'explore' ? '自动探索' + (v.target ? '「' + v.target + '」' : '') : v.title;
    const stage = h('div', { class: 'n-meta' });
    const bar = h('span', {});
    const progress = h('div', { class: 'n-progress' }, bar);
    const statsBox = h('div', { class: 'c-hero-stats' });
    const actions = [];
    if (v.kind === 'explore') actions.push(h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: v.cancelling, onclick: () => pauseExploration(cards.get(id).task) }, '暂停并保存'));
    actions.push(h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: v.cancelling, onclick: () => tasks.stop(cards.get(id).task) }, v.cancelling ? '正在取消…' : v.stopLabel));
    const el = h('div', { class: 'n-hl c-hero', dataset: { task: task.task_id } },
      h('div', { class: 'c-hero-head' }, h('div', { class: 'c-hero-main' }, h('div', { class: 'c-hero-title' }, title), stage), h('span', { class: 'c-src' }, v.source + '发起')),
      progress, statsBox,
      v.cancelError ? h('div', { class: 'n-meta', style: 'color:var(--bad)' }, '取消失败：' + v.cancelError) : null,
      h('div', { class: 'c-card-actions' }, actions));
    const entry = { el, key, task, update(next) {
      entry.task = next;
      const { pct, items } = stats(next, view(next));
      stage.textContent = stageText(next);
      progress.classList.toggle('n-progress--indeterminate', pct == null);
      bar.style.width = (pct ?? 0) + '%';
      statsBox.replaceChildren(...items.map(([value, label]) => h('span', {}, h('b', { class: 'n-num' }, value), label)));
    } };
    entry.update(task);
    cards.set(id, entry);
    return el;
  }
  /* 把任务块放进容器：已在页面上的原地保留，只在顺序或数量变化时替换。 */
  function place(container, list, scope = '') {
    const nodes = list.map(task => card(task, scope));
    for (const id of [...cards.keys()]) if (id.startsWith(scope + '|') && !list.some(task => scope + '|' + task.task_id === id)) cards.delete(id);
    if (nodes.length !== container.children.length || nodes.some((node, index) => container.children[index] !== node)) container.replaceChildren(...nodes);
  }
  return { card, place, pauseExploration };
}
