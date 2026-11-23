/* 日志页（车道 P 实现）。本批只占位，挂载约定见 shell.js：mount(root, ctx) 返回 {show(sub, opts), hide()}。 */
import { icon, h } from '/static/shared/neh-ui.js';

export function mount(root) {
  root.append(h('div', { class: 'n-page--full' },
    h('div', { class: 'n-toolbar' }, h('h1', {}, '日志')),
    h('div', { class: 'n-page-body' }, h('div', { class: 'n-empty', html: icon('scroll-text') },
      h('div', { class: 'n-empty-title' }, '日志页在下一批实现'), h('div', {}, '表格铺满，点一行在右侧抽屉看完整记录。')))));
  return { show() {}, hide() {} };
}
