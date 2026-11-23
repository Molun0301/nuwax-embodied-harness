/* 设置页（车道 P 实现）。本批只占位，挂载约定见 shell.js：mount(root, ctx) 返回 {show(sub, opts), hide()}。 */
import { icon, h } from '/static/shared/neh-ui.js';

export function mount(root) {
  root.append(h('div', { class: 'n-page--full' },
    h('div', { class: 'n-toolbar' }, h('h1', {}, '设置')),
    h('div', { class: 'n-page-body' }, h('div', { class: 'n-empty', html: icon('settings') },
      h('div', { class: 'n-empty-title' }, '设置页在下一批实现'), h('div', {}, '按功能板块列出全部配置，开关即时保存。')))));
  return { show() {}, hide() {} };
}
