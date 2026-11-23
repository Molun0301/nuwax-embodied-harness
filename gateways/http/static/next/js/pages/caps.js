/* 能力页（车道 P 实现）。本批只占位，挂载约定见 shell.js：mount(root, ctx) 返回 {show(sub, opts), hide()}。 */
import { icon, h } from '/static/shared/neh-ui.js';

export function mount(root) {
  root.append(h('div', { class: 'n-page--full' },
    h('div', { class: 'n-toolbar' }, h('h1', {}, '能力')),
    h('div', { class: 'n-page-body' }, h('div', { class: 'n-empty', html: icon('puzzle') },
      h('div', { class: 'n-empty-title' }, '能力页在下一批实现'), h('div', {}, '左侧按分组列出全部能力，中间填写参数，右侧看执行结果。')))));
  return { show() {}, hide() {} };
}
