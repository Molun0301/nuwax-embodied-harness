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
  if (opts.action) el.append(h('button', { class: 'n-toast-action', type: 'button', onclick: () => { el.remove(); opts.action.run(); } }, opts.action.label));
  toasts().append(el);
  setTimeout(() => el.remove(), opts.duration || 4200);
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
    const done = v => { dlg.close(); dlg.remove(); resolve(v); };
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
  const pop = h('div', { class: 'n-popover', role: 'menu' }); pop.append(content);
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
export function closePopovers() { document.querySelectorAll('.n-popover').forEach(p => { p._cleanup?.(); p.remove(); }); }
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
  el.setOpen = v => { el.dataset.open = String(v); chev.setAttribute('aria-expanded', String(v)); chev.setAttribute('aria-label', v ? '收起' : '展开'); };
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
  el.setOpen = v => { el.dataset.open = String(v); head.setAttribute('aria-expanded', String(v)); };
  const toggle = () => { const v = el.dataset.open !== 'true'; el.setOpen(v); onToggle?.(v); };
  head.addEventListener('click', toggle);
  head.addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); toggle(); } });
  el.append(head, el.body);
  return el;
}
