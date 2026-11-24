/* NEH 共用界面组件：图标、元素构造、轻提示、确认框、主题、弹出层、菜单、可折叠板块。
   控制台与管理服务恢复页共用，原生 ES 模块，不挂全局变量。
   主题：页面在 <head> 里用同一个存储键先写入 data-theme（避免首屏闪烁），之后由 setTheme 切换。 */
import { ICONS } from './icons.js';

export const THEME_KEY = 'nuwax-theme';

/* 图标：name 为 Lucide 图标名，返回 <svg> 字符串，可放进 h() 的 html 属性或 innerHTML。 */
export function icon(name, cls) {
  return '<svg class="n-icon' + (cls ? ' ' + cls : '') + '" viewBox="0 0 24 24" aria-hidden="true">' + (ICONS[name] || '') + '</svg>';
}

/* 元素构造：h('button', {class, onclick, dataset, html}, 子节点或文字…)；值为 null/false 的属性与子节点跳过。 */
export function h(tag, attrs, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v == null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'html') el.innerHTML = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else if (k === 'dataset') Object.assign(el.dataset, v);
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const kid of kids.flat()) if (kid != null && kid !== false) el.append(kid.nodeType ? kid : document.createTextNode(String(kid)));
  return el;
}

/* ---------- 动效 ----------
   时长与曲线取 neh-ui.css 的 --motion-fast/base/slow 与 --ease-out/--ease-in（系统设置减少动效时时长为 1ms）。
   能用 CSS 过渡的都在样式表里；下面是需要脚本的几类：元素离开后再移除或隐藏、高度展开收起、分段与页签的选中指示滑动。 */
const rootStyle = () => getComputedStyle(document.documentElement);
export function motionMs(speed = 'base') { return parseFloat(rootStyle().getPropertyValue('--motion-' + speed)) || 0; }
const easing = kind => rootStyle().getPropertyValue(kind === 'in' ? '--ease-in' : '--ease-out').trim() || 'ease';

/* 进入：从 from（默认下移 4px、透明）回到原位；正在离开的元素会先停止离开。 */
export function enter(el, { from = 'translateY(4px)', speed = 'base' } = {}) {
  if (!el?.animate) return null;
  if (el._leave) { el._leave.cancel(); el._leave = null; el.style.pointerEvents = ''; }
  return el.animate([{ opacity: 0, transform: from }, { opacity: 1, transform: 'none' }], { duration: motionMs(speed), easing: easing('out') });
}
/* 离开：淡出并移向 to，结束后移除（hide 为真时改为设 hidden）；离开期间不接收点击。 */
export function leave(el, { to = 'translateY(4px)', speed = 'base', hide = false } = {}) {
  if (!el || el._leave) return;
  if (!el.animate || !el.isConnected) { if (hide) el.hidden = true; else el.remove(); return; }
  el.style.pointerEvents = 'none';
  const anim = el.animate([{ opacity: 1, transform: 'none' }, { opacity: 0, transform: to }], { duration: motionMs(speed), easing: easing('in'), fill: 'forwards' });
  el._leave = anim;
  anim.finished.then(() => {
    if (el._leave !== anim) return;
    el._leave = null;
    if (!hide) { el.remove(); return; }
    el.hidden = true; el.style.pointerEvents = ''; anim.cancel();
  }).catch(() => {});
}
/* 高度展开收起：body 收起时由样式隐藏（display: none），动画期间加 n-collapsing 保持显示并裁剪。
   在改变展开状态之前调用本函数记下起始高度，改完后调用返回的函数开始动画。 */
export function collapse(body) {
  if (!body?.animate || !body.isConnected) return () => {};
  const from = body.getBoundingClientRect().height;
  body._height?.cancel();
  return open => {
    body.classList.add('n-collapsing');
    const to = open ? body.scrollHeight : 0;
    const anim = body.animate([{ height: from + 'px' }, { height: to + 'px' }], { duration: motionMs('base'), easing: easing(open ? 'out' : 'in') });
    body._height = anim;
    anim.finished.then(() => { if (body._height === anim) { body._height = null; body.classList.remove('n-collapsing'); } }).catch(() => {});
  };
}
/* 整块新建或即将移除的区域按高度展开收起：open 为真时从 0 展开到现有高度，否则从现有高度收到 0（停在 0，由调用方随后移除）。
   返回动画结束的 Promise。 */
export function slideHeight(el, open) {
  if (!el?.animate || !el.isConnected) return Promise.resolve();
  const full = el.getBoundingClientRect().height + 'px';
  el.style.overflow = 'hidden';
  const anim = el.animate([{ height: open ? '0px' : full }, { height: open ? full : '0px' }],
    { duration: motionMs('base'), easing: easing(open ? 'out' : 'in'), fill: open ? 'none' : 'forwards' });
  return anim.finished.then(() => { if (open) el.style.overflow = ''; }, () => {});
}
/* 进度条：页面每次刷新都整块重建时，按 key 记住上次的进度，新建的条从上次宽度过渡到新宽度。percent 为 null 时是不确定进度。 */
const progressWidths = new Map();
export function progressBar(key, percent, cls = '') {
  const fill = h('span', { style: percent == null ? null : 'width:' + percent + '%' });
  const last = progressWidths.get(key);
  if (percent != null && last != null && last !== percent && fill.animate) {
    fill.animate([{ width: last + '%' }, { width: percent + '%' }], { duration: motionMs('slow'), easing: easing('out') });
  }
  progressWidths.set(key, percent);
  return h('div', { class: 'n-progress' + (percent == null ? ' n-progress--indeterminate' : '') + (cls ? ' ' + cls : '') }, fill);
}
/* 文字切换淡入：内容变化时（只比较文字）新内容淡入，用于一句话状态这类会整体换掉的文字。 */
export function fadeOnChange(el) {
  let last = el.textContent;
  new MutationObserver(() => {
    if (el.textContent === last) return;
    last = el.textContent;
    el.animate?.([{ opacity: 0 }, { opacity: 1 }], { duration: motionMs('base'), easing: easing('out') });
  }).observe(el, { childList: true, characterData: true, subtree: true });
}

/* details 展开收起：点摘要时按高度动画，收起动画结束后才去掉 open。
   收起期间带 data-closing，箭头样式据此在开始收起时就转回；收起途中再点改为重新展开。 */
document.addEventListener('click', e => {
  const summary = e.target.closest?.('summary');
  const details = summary?.parentElement;
  if (!details || details.tagName !== 'DETAILS' || details.firstElementChild !== summary || !details.animate) return;
  e.preventDefault();
  const from = details.getBoundingClientRect().height;
  details._height?.cancel();
  const closing = details.open && details.dataset.closing !== 'true';
  details.open = true;
  if (closing) details.dataset.closing = 'true'; else delete details.dataset.closing;
  const to = closing ? summary.getBoundingClientRect().height : details.getBoundingClientRect().height;
  details.style.overflow = 'hidden';
  const anim = details.animate([{ height: from + 'px' }, { height: to + 'px' }], { duration: motionMs('base'), easing: easing(closing ? 'in' : 'out') });
  details._height = anim;
  anim.finished.then(() => {
    if (details._height !== anim) return;
    details._height = null; details.style.overflow = '';
    if (closing) { details.open = false; delete details.dataset.closing; }
  }).catch(() => {});
});

/* 分段控件与下划线页签：选中指示是一块单独的底，换选中项时从原位置滑过去（只动 transform）。
   自动作用于 .n-seg、.n-tabs 与带 data-ind 的容器（data-ind="underline" 为下划线）；容器里的按钮整体重建时也接着滑。 */
const IND_BOX = '.n-seg, .n-tabs, [data-ind]';
const pendingInd = new Set();
let indFrame = 0;
function placeIndicator(box) {
  const selected = box.querySelector(':scope > [aria-pressed="true"], :scope > [aria-selected="true"]');
  let ind = box.querySelector(':scope > .n-ind');
  if (!box.isConnected) { indResize.unobserve(box); box._indWatched = false; return; }
  if (!selected) { ind?.remove(); box.classList.remove('n-has-ind'); box._ind = null; return; }
  const underline = box.matches('.n-tabs, [data-ind="underline"]');
  const x = selected.offsetLeft + (underline ? 8 : 0), w = Math.max(0, selected.offsetWidth - (underline ? 16 : 0));
  if (!w) return;   // 容器还没显示：等显示后尺寸变化再放
  if (!ind) { ind = h('span', { class: 'n-ind', 'aria-hidden': 'true' }); box.prepend(ind); }
  box.classList.add('n-has-ind');
  ind.style.left = x + 'px'; ind.style.width = w + 'px';
  if (!underline) { ind.style.top = selected.offsetTop + 'px'; ind.style.height = selected.offsetHeight + 'px'; }
  const prev = box._ind;
  box._ind = { x, w };
  if (prev && prev.w && (prev.x !== x || prev.w !== w))
    ind.animate([{ transform: 'translateX(' + (prev.x - x) + 'px) scaleX(' + prev.w / w + ')' }, { transform: 'none' }], { duration: motionMs('base'), easing: easing('out') });
}
function queueIndicator(box) {
  pendingInd.add(box);
  if (!indFrame) indFrame = requestAnimationFrame(() => { indFrame = 0; const boxes = [...pendingInd]; pendingInd.clear(); boxes.forEach(placeIndicator); });
}
const indResize = new ResizeObserver(entries => entries.forEach(entry => queueIndicator(entry.target)));
function watchBoxes(node) {
  if (node.nodeType !== 1) return;
  const boxes = node.matches(IND_BOX) ? [node, ...node.querySelectorAll(IND_BOX)] : node.querySelectorAll(IND_BOX);
  for (const box of boxes) { if (!box._indWatched) { box._indWatched = true; indResize.observe(box); } queueIndicator(box); }
}
new MutationObserver(records => {
  for (const r of records) {
    if (r.type === 'attributes') { if (r.target.parentElement?.matches(IND_BOX)) queueIndicator(r.target.parentElement); continue; }
    if (r.target.matches?.(IND_BOX) && [...r.addedNodes].some(n => !n.classList?.contains('n-ind'))) queueIndicator(r.target);
    r.addedNodes.forEach(watchBoxes);
  }
}).observe(document.documentElement, { subtree: true, childList: true, attributes: true, attributeFilter: ['aria-pressed', 'aria-selected'] });
if (document.body) watchBoxes(document.body); else document.addEventListener('DOMContentLoaded', () => watchBoxes(document.body));

function toasts() {
  let el = document.querySelector('.n-toasts');
  if (!el) { el = h('div', { class: 'n-toasts', role: 'status', 'aria-live': 'polite' }); document.body.append(el); }
  return el;
}

/* 轻提示：tone 为 ok / info / warn / bad；action = {label, run} 时带一个按钮。 */
export function toast(message, opts = {}) {
  const tone = opts.tone || 'ok';
  const el = h('div', { class: 'n-toast n-toast--' + tone, html: icon(tone === 'bad' ? 'circle-alert' : tone === 'warn' ? 'triangle-alert' : tone === 'info' ? 'info' : 'check') });
  el.append(h('span', {}, message));
  if (opts.action) el.append(h('button', { class: 'n-toast-action', type: 'button', onclick: () => { leave(el, { to: 'translateY(8px)' }); opts.action.run(); } }, opts.action.label));
  toasts().append(el);
  setTimeout(() => leave(el, { to: 'translateY(8px)' }), opts.duration || 4200);
}

/* 复制文字：优先剪贴板接口；不可用或被拒时用隐藏文本框加原生复制（设备常经 http 地址打开，非安全来源没有剪贴板接口）。
   返回是否复制成功。 */
export async function copyText(text) {
  try {
    if (navigator.clipboard?.writeText) { await navigator.clipboard.writeText(text); return true; }
  } catch (e) { /* 权限被拒时继续尝试原生复制 */ }
  const focused = document.activeElement;
  const area = h('textarea', { readonly: true, 'aria-hidden': 'true', style: 'position:fixed;left:-9999px' });
  area.value = text;
  document.body.append(area);
  let copied = false;
  try { area.select(); copied = document.execCommand('copy') === true; } catch (e) { copied = false; }
  area.remove();
  focused?.focus?.({ preventScroll: true });
  return copied;
}

/* 确认框：返回 Promise<boolean>。danger 为真时确认按钮用红色实心，标题图标标红。 */
export function confirmDialog({ title, text, items, confirmText = '确认', danger = false, extra, icon: ic }) {
  return new Promise(resolve => {
    const dlg = h('dialog', { class: 'n-dialog' });
    const body = h('div', { class: 'n-dialog-body' },
      h('div', { class: 'n-dialog-title', html: (ic || danger ? icon(ic || 'triangle-alert') : '') }, title),
      text ? h('p', { class: 'n-dialog-text' }, text) : null,
      items && items.length ? h('ul', { class: 'n-dialog-list' }, items.map(t => h('li', {}, t))) : null,
      extra || null);
    if (danger) body.querySelector('.n-dialog-title .n-icon')?.style.setProperty('color', 'var(--bad)');
    // 关闭时先播放退出过渡（样式表里的对话框与背景淡出），结束后再移除。
    const done = v => { dlg.close(); setTimeout(() => dlg.remove(), motionMs('slow') + 20); resolve(v); };
    const actions = h('div', { class: 'n-dialog-actions' },
      h('button', { class: 'n-btn', type: 'button', onclick: () => done(false) }, '取消'),
      h('button', { class: 'n-btn ' + (danger ? 'n-btn--danger-fill' : 'n-btn--primary'), type: 'button', onclick: () => done(true) }, confirmText));
    dlg.append(body, actions);
    dlg.addEventListener('cancel', e => { e.preventDefault(); done(false); });
    document.body.append(dlg); dlg.showModal();
    actions.lastChild.focus();
  });
}

/* 主题：写入页面与本机存储，并广播 neh:theme 事件（三维舞台等据此换色）。 */
export function setTheme(theme) {
  document.documentElement.dataset.theme = theme;
  try { localStorage.setItem(THEME_KEY, theme); } catch (e) { /* 无痕模式下只本次有效 */ }
  document.dispatchEvent(new CustomEvent('neh:theme', { detail: theme }));
}
export function currentTheme() { return document.documentElement.dataset.theme === 'dark' ? 'dark' : 'light'; }

/* 弹出层：贴着 anchor 打开，点外部或 Esc 关闭；同一时间只有一个。 */
export function popover(anchor, content, { align = 'start', side = 'bottom' } = {}) {
  closePopovers();
  const pop = h('div', { class: 'n-popover', role: 'menu', dataset: { side } }); pop.append(content);
  document.body.append(pop);
  const r = anchor.getBoundingClientRect(); const pw = pop.offsetWidth; const ph = pop.offsetHeight;
  let x = align === 'end' ? r.right - pw : r.left; let y = side === 'top' ? r.top - ph - 6 : r.bottom + 6;
  x = Math.max(8, Math.min(x, innerWidth - pw - 8)); y = Math.max(8, Math.min(y, innerHeight - ph - 8));
  pop.style.left = x + 'px'; pop.style.top = y + 'px'; pop.style.position = 'fixed';
  setTimeout(() => document.addEventListener('pointerdown', onOutside, true));
  function onOutside(e) { if (!pop.contains(e.target) && !anchor.contains(e.target)) closePopovers(); }
  pop._cleanup = () => document.removeEventListener('pointerdown', onOutside, true);
  return pop;
}
export function closePopovers() {
  document.querySelectorAll('.n-popover').forEach(p => { p._cleanup?.(); leave(p, { to: p.dataset.side === 'top' ? 'translateY(-4px)' : 'translateY(4px)' }); });
}
document.addEventListener('keydown', e => { if (e.key === 'Escape') closePopovers(); });

/* 菜单：items 为 {label, icon, danger, run} 或分隔线 '-'。 */
export function menu(anchor, items, opts) {
  const box = h('div', {});
  for (const it of items) {
    if (it === '-') { box.append(h('div', { class: 'n-menu-sep' })); continue; }
    box.append(h('button', { class: 'n-menu-item' + (it.danger ? ' n-menu-item--danger' : ''), type: 'button', html: it.icon ? icon(it.icon, 'n-icon--sm') : '',
      onclick: () => { closePopovers(); it.run(); } }, it.label));
  }
  return popover(anchor, box, opts);
}

/* 可折叠板块：图标 + 标题 + 摘要 + 侧边内容 + 折叠箭头；点击标题行切换，侧边内容（开关、按钮）不触发折叠。
   返回板块元素，内容放进 el.body，el.setOpen(布尔) 设置展开状态。 */
export function block({ icon: ic, title, summary, aside, open = true, state = 'on', id, onToggle } = {}) {
  const el = h('section', { class: 'n-block', id, dataset: { open: String(open), state } });
  const chev = h('button', { class: 'n-block-chev', type: 'button', 'aria-expanded': String(open), 'aria-label': open ? '收起' : '展开', html: icon('chevron-right') });
  const head = h('div', { class: 'n-block-head' },
    ic ? h('span', { class: 'n-block-icon', html: icon(ic) }) : null,
    h('div', { class: 'n-block-title' }, h('h2', {}, title), summary ? h('p', {}, summary) : null),
    aside ? h('div', { class: 'n-block-aside', onclick: e => e.stopPropagation() }, aside) : null, chev);
  el.body = h('div', { class: 'n-block-body' });
  el.setOpen = v => {
    const animate = collapse(el.body);
    el.dataset.open = String(v); chev.setAttribute('aria-expanded', String(v)); chev.setAttribute('aria-label', v ? '收起' : '展开');
    animate(v);
  };
  head.addEventListener('click', () => { const v = el.dataset.open !== 'true'; el.setOpen(v); onToggle?.(v); });
  el.append(head, el.body);
  return el;
}

/* 板块内的子分组：标题、数量、侧边内容（如子类开关），可折叠。 */
export function sub({ title, count, aside, open = true, disabled = false, onToggle } = {}) {
  const el = h('div', { class: 'n-sub', dataset: { open: String(open), disabled: String(disabled) } });
  const head = h('div', { class: 'n-sub-head', role: 'button', tabindex: 0, 'aria-expanded': String(open), html: icon('chevron-right', 'n-sub-chev') },
    h('h3', {}, title), count != null ? h('span', { class: 'n-sub-count' }, count) : null,
    aside ? h('div', { class: 'n-sub-aside', onclick: e => e.stopPropagation() }, aside) : null);
  el.body = h('div', { class: 'n-sub-body' });
  el.setOpen = v => {
    const animate = collapse(el.body);
    el.dataset.open = String(v); head.setAttribute('aria-expanded', String(v));
    animate(v);
  };
  const toggle = () => { const v = el.dataset.open !== 'true'; el.setOpen(v); onToggle?.(v); };
  head.addEventListener('click', toggle);
  head.addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); toggle(); } });
  el.append(head, el.body);
  return el;
}
