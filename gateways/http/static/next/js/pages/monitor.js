/* 监控页：舞台（地图）+ 右侧面板。本批只有框架：舞台占位、业务未运行时的遮罩、六个面板页签与收起；
   窄屏（≤760px）时面板变为底部抽屉，收起时只露一行当前状态。舞台控件与各面板内容在后续批次接入。 */
import { icon, h } from '/static/shared/neh-ui.js';
import { deriveActivity } from '../status.js';

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
  const halt = h('div', { class: 'c-halt', hidden: true });
  const collapseBtn = h('button', { class: 'c-collapse', type: 'button' });
  const stage = h('div', { class: 'c-stage n-tone-dark', dataset: { tone: 'dark' } },
    h('div', { class: 'c-stage-empty' }, h('div', { class: 'n-float', html: icon('map') },
      h('div', { class: 'n-t3', style: 'color:var(--glass-ink)' }, '地图舞台'),
      h('div', { class: 'n-meta' }, '二维、三维、语义地图与关系图在下一批接入。'))),
    halt, collapseBtn);
  const aside = h('aside', { class: 'c-side', 'aria-label': '工具与活动' }, tabs, body);
  const monitor = h('div', { class: 'c-monitor' + (collapsed ? ' is-collapsed' : '') }, stage, aside);
  root.append(monitor);

  /* 面板内容：每个页签一个容器，后续批次由各面板模块填充。 */
  const panes = Object.fromEntries(PANELS.map(([id, label]) => [id, h('div', { class: 'c-pane', dataset: { pane: id }, hidden: true },
    h('div', { class: 'c-pane-empty' }, h('div', { class: 'n-t3' }, label), h('div', { class: 'n-meta' }, '这一页的内容在下一批接入。')))]));
  body.append(...Object.values(panes));

  function peekText() {
    const s = store.get();
    if (s.business !== 'running') return '业务服务未运行';
    const a = deriveActivity(s, Date.now(), { labels: ctx.tasks.labels, navRange: ctx.tasks.navRange, capabilities: null });
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
      return h('button', { class: 'c-side-tab', type: 'button', role: 'tab', 'aria-selected': String(side === id), html: icon(id === 'activity' ? 'activity' : ic),
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
  /* opts.open：从页签、铃铛等明确要看面板的入口进来时展开收起的面板；opts.focus：滚动到面板内该编号的区块。 */
  function openSide(id, opts = {}) {
    side = id;
    for (const [key, pane] of Object.entries(panes)) pane.hidden = key !== id;
    if (collapsed && opts.open) setCollapsed(false); else renderTabs();
    if (opts.focus) setTimeout(() => body.querySelector('#' + opts.focus)?.scrollIntoView({ block: 'start', behavior: 'smooth' }), 30);
  }

  /* 业务未运行：舞台蒙上遮罩，给出与顶栏相同的入口。 */
  function renderHalt() {
    const business = store.get().business;
    halt.hidden = business === 'running';
    if (halt.hidden) return;
    const updating = business === 'updating';
    halt.replaceChildren(h('div', { class: 'n-float c-halt-card', html: icon(updating ? 'download' : 'power') },
      h('div', { class: 'n-t2' }, updating ? '正在切换到新版本' : '业务服务已停止'),
      h('div', { class: 'n-meta' }, updating ? '地图、视频与机器人数据会在业务重新启动后恢复。' : '地图、视频与机器人数据暂不可用。在系统页启动业务服务，或查看原因。'),
      h('button', { class: 'n-btn n-btn--primary', type: 'button', onclick: () => go('system') }, updating ? '查看进度' : '打开系统页')));
  }
  store.subscribe(['business'], () => { renderHalt(); renderTabs(); });
  store.subscribe(['tasks', 'unread', 'conn'], renderTabs);

  setCollapsed(collapsed);
  openSide(side);
  renderHalt();
  return {
    show(sub, opts = {}) { openSide(PANEL_IDS.includes(sub) ? sub : side, opts); },
    hide() {},
  };
}
