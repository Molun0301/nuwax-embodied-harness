/* 系统页（车道 P 实现）。本批只占位，挂载约定见 shell.js：mount(root, ctx) 返回 {show(sub, opts), hide()}。 */
import { icon, h } from '/static/shared/neh-ui.js';

export function mount(root) {
  root.append(h('div', { class: 'n-page--full' },
    h('div', { class: 'n-toolbar' }, h('h1', {}, '系统')),
    h('div', { class: 'n-page-body' }, h('div', { class: 'n-empty', html: icon('server') },
      h('div', { class: 'n-empty-title' }, '系统页在下一批实现'), h('div', {}, '业务状态与启停、服务、版本更新与回退、功能模块、安装与更新记录。')))));
  return { show() {}, hide() {} };
}
