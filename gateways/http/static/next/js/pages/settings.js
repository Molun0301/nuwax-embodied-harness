/* 设置页：挂载共用设置编辑器（web/shared/settings-editor.js），恢复页挂载同一模块。
   地址 #/settings/<板块名> 展开并滚到该板块；show 的 opts.key 搜索某个键（其它页面链接到具体设置时用）。 */
import { mount as mountEditor } from '/static/shared/settings-editor.js';

export function mount(root, ctx) {
  ctx.loadCss('/static/shared/settings-editor.css');
  const editor = mountEditor(root, {
    title: '设置',
    request: (path, init) => ctx.api.manager(path, init),
    // 重启经设备管理程序的任务执行，进度在系统页。
    onRestart: () => ctx.ui.toast('已安排重启业务服务，完成前网页连接会短暂断开', { tone: 'info', duration: 8000,
      action: { label: '查看进度', run: () => ctx.go('system') } }),
  });
  let loaded = null;
  return {
    async show(sub, opts = {}) {
      // 第一次打开时读取；之后每次回到本页重读一次（保留未保存的输入），反映别处做的修改。
      loaded = loaded ? loaded.then(() => editor.load()) : editor.load();
      await loaded;
      if (opts.key) editor.focusKey(opts.key);
      else if (sub) editor.openPanel(sub);
    },
    hide() {},
  };
}
