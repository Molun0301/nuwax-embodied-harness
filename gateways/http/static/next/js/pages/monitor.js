/* 监控页：舞台（地图）+ 右侧面板。舞台见 stage/stage.js；面板六个页签：活动、地图、语义、导航、运动、禁行区，
   各在 panels/ 下一个模块，接口为 mountX(root, ctx) → {show(opts), hide()}；同一时间只显示一个，切走时调用 hide
   （运动页停车、导航页结束选点、禁行区结束绘制）。窄屏（≤760px）时面板变为底部抽屉，收起时只露一行当前状态。 */
import { icon, h } from '/static/shared/neh-ui.js';
import { deriveActivity } from '../status.js';
import { createStage } from '../stage/stage.js';
import { mountActivity } from '../panels/activity.js';
import { createTaskCards } from '../panels/taskcard.js';
import { mountMap } from '../panels/map.js';
import { mountSemantic } from '../panels/semantic.js';
import { mountNav } from '../panels/nav.js';
import { mountMotion } from '../panels/motion.js';
import { mountZones } from '../panels/zones.js';
import { play, slideIn } from '../motion.js';

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

  /* 面板内容：每个页签一个容器。 */
  const panes = Object.fromEntries(PANELS.map(([id]) => [id, h('div', { class: 'c-pane', dataset: { pane: id }, hidden: true })]));
  body.append(...Object.values(panes));

  /* 舞台点选物品、区域或关系图节点：选中并打开“语义”页对应详情。 */
  const stage = createStage(stageEl, ctx, { onPick(hit) {
    if (hit.type === 'item') stage.items.select(hit.instance.instance_id);
    if (hit.type === 'region') stage.regions.select(hit.region.scene_space_id);
    go('monitor', 'semantic', { open: true, pick: hit });
  } });
  ctx.stage = stage;
  ctx.taskCards = createTaskCards(ctx);
  const modules = { activity: mountActivity(panes.activity, ctx), map: mountMap(panes.map, ctx), semantic: mountSemantic(panes.semantic, ctx),
    nav: mountNav(panes.nav, ctx), motion: mountMotion(panes.motion, ctx), zones: mountZones(panes.zones, ctx) };
  let shownModule = null;
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
      tabs.replaceChildren(h('button', { class: 'c-peek', type: 'button', onclick: () => setCollapsed(false, true) },
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
          if (narrow() && id === side && !collapsed) { setCollapsed(true, true); return; }
          go('monitor', id, { open: true });
        } }, label, dot);
    }));
  }
  // animate：窄屏底部抽屉由用户拉起或收起时，从原来的高度滑到新的位置（首次进入不动）。
  function setCollapsed(value, animate = false) {
    const before = animate && narrow() ? aside.getBoundingClientRect().top : null;
    collapsed = value;
    monitor.classList.toggle('is-collapsed', collapsed);
    if (before !== null) {
      const dy = before - aside.getBoundingClientRect().top;
      if (dy) play(aside, [{ transform: `translateY(${dy}px)` }, { transform: 'none' }], { speed: 'slow', ease: collapsed ? 'in' : 'out' });
    }
    collapseBtn.innerHTML = icon(collapsed ? 'chevron-left' : 'chevron-right');
    collapseBtn.title = collapsed ? '展开侧栏' : '收起侧栏';
    renderTabs();
  }
  collapseBtn.addEventListener('click', () => setCollapsed(!collapsed, true));
  stageEl.append(collapseBtn);
  /* opts.open：从页签、铃铛等明确要看面板的入口进来时展开收起的面板；opts.focus：滚动到面板内该编号的区块。 */
  function openSide(id, opts = {}) {
    if (shownModule && shownModule !== id) modules[shownModule].hide?.();
    const previous = side;
    side = id;
    shownModule = id;
    for (const [key, pane] of Object.entries(panes)) pane.hidden = key !== id;
    if (collapsed && opts.open) setCollapsed(false, true); else renderTabs();
    modules[id].show(opts);
    // 换页签：往右边的页签从右侧进，往左边的从左侧进。
    if (previous !== id) slideIn(panes[id], Math.sign(PANEL_IDS.indexOf(id) - PANEL_IDS.indexOf(previous)));
  }
  store.subscribe(['business', 'tasks', 'unread', 'conn'], renderTabs);

  setCollapsed(collapsed);
  openSide(side);
  return {
    show(sub, opts = {}) { openSide(PANEL_IDS.includes(sub) ? sub : side, opts); stage.show(); },
    hide() {
      if (shownModule) modules[shownModule].hide?.();
      shownModule = null;
      stage.hide();
    },
  };
}
