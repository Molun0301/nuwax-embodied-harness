/* 监控页：舞台（地图）+ 右侧面板。舞台见 stage/stage.js；面板六个页签：活动（panels/activity.js）、
   地图、语义、导航、运动、禁行区（P3 接入）。窄屏（≤760px）时面板变为底部抽屉，收起时只露一行当前状态。 */
import { icon, h } from '/static/shared/neh-ui.js';
import { deriveActivity } from '../status.js';
import { createStage } from '../stage/stage.js';
import { mountActivity } from '../panels/activity.js';

export const PANELS = [['activity', '活动', 'activity'], ['map', '地图', 'map'], ['semantic', '语义', 'tag'],
  ['nav', '导航', 'navigation'], ['motion', '运动', 'move'], ['zones', '禁行区', 'ban']];
const PANEL_IDS = PANELS.map(([id]) => id);

export function mount(root, ctx) {
  const { store, go } = ctx;
  const narrow = () => window.innerWidth <= 760;
  let side = 'activity';
  let collapsed = narrow();

  const tabs = h('div', { class: 'c-side-tabs', role: 'tablist', 'aria-label': '工具与活动' });
  const body = h('div', { class: 'c-side-body' });
  const collapseBtn = h('button', { class: 'c-collapse', type: 'button' });
  const stageEl = h('div', { class: 'c-stage n-tone-dark', dataset: { tone: 'dark' } });
  const aside = h('aside', { class: 'c-side', 'aria-label': '工具与活动' }, tabs, body);
  const monitor = h('div', { class: 'c-monitor' + (collapsed ? ' is-collapsed' : '') }, stageEl, aside);
  root.append(monitor);

  /* 面板内容：每个页签一个容器；活动页本批接入，其余页签在 P3 由各面板模块填充。 */
  const panes = Object.fromEntries(PANELS.map(([id, label]) => [id, h('div', { class: 'c-pane', dataset: { pane: id }, hidden: true },
    id === 'activity' ? null : h('div', { class: 'c-pane-empty' }, h('div', { class: 'n-t3' }, label), h('div', { class: 'n-meta' }, '这一页的内容在下一批接入。')))]));
  body.append(...Object.values(panes));
  const activity = mountActivity(panes.activity, ctx);

  /* 舞台点选物品、区域或关系图节点：选中并打开“语义”页（详情在 P3 语义面板）。 */
  const stage = createStage(stageEl, ctx, { onPick(hit) {
    if (hit.type === 'item') stage.items.select(hit.instance.instance_id);
    if (hit.type === 'region') stage.regions.select(hit.region.scene_space_id);
    go('monitor', 'semantic', { open: true, pick: hit });
  } });
  ctx.stage = stage;
  // 收起或展开侧栏后舞台尺寸改变。
  new ResizeObserver(() => stage.resize()).observe(stageEl);

  function peekText() {
    const s = store.get();
    if (s.business !== 'running') return '业务服务未运行';
    const a = deriveActivity(s, Date.now(), { navRange: ctx.tasks.navRange, capabilities: null });
    return a.tone === 'idle' ? '让机器人去哪里？' : a.text + (a.quote ? '「' + a.quote + '」' : '');
  }
  function renderTabs() {
    if (narrow() && collapsed) {
      tabs.replaceChildren(h('button', { class: 'c-peek', type: 'button', onclick: () => setCollapsed(false) },
        h('i', { class: 'c-peek-handle' }), h('span', { class: 'c-peek-text' }, peekText()),
        h('span', { class: 'c-peek-more', html: icon('chevron-up', 'n-icon--sm') }, '工具')));
      return;
    }
    const s = store.get();
    tabs.replaceChildren(...PANELS.map(([id, label, ic]) => {
      // 活动页签：有未读的需要注意时红点，有任务在执行而当前不在活动页时青点。
      const dot = id === 'activity' && s.unread ? h('span', { class: 'c-tab-dot', dataset: { tone: 'bad' } })
        : id === 'activity' && s.tasks.active.length && side !== 'activity' ? h('span', { class: 'c-tab-dot' }) : null;
      return h('button', { class: 'c-side-tab', type: 'button', role: 'tab', 'aria-selected': String(side === id), html: icon(ic),
        onclick: () => {
          if (narrow() && id === side && !collapsed) { setCollapsed(true); return; }
          go('monitor', id, { open: true });
        } }, label, dot);
    }));
  }
  function setCollapsed(value) {
    collapsed = value;
    monitor.classList.toggle('is-collapsed', collapsed);
    collapseBtn.innerHTML = icon(collapsed ? 'chevron-left' : 'chevron-right');
    collapseBtn.title = collapsed ? '展开侧栏' : '收起侧栏';
    renderTabs();
  }
  collapseBtn.addEventListener('click', () => setCollapsed(!collapsed));
  stageEl.append(collapseBtn);
  /* opts.open：从页签、铃铛等明确要看面板的入口进来时展开收起的面板；opts.focus：滚动到面板内该编号的区块。 */
  function openSide(id, opts = {}) {
    side = id;
    for (const [key, pane] of Object.entries(panes)) pane.hidden = key !== id;
    if (collapsed && opts.open) setCollapsed(false); else renderTabs();
    if (id === 'activity') activity.show(opts);
  }
  store.subscribe(['business', 'tasks', 'unread', 'conn'], renderTabs);

  setCollapsed(collapsed);
  openSide(side);
  return {
    show(sub, opts = {}) { openSide(PANEL_IDS.includes(sub) ? sub : side, opts); stage.show(); },
    hide() { stage.hide(); },
  };
}
