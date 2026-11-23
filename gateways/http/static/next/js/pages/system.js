/* 系统页：挂载共用系统页模块（web/shared/system-pages.js），恢复页挂载同一模块。
   服务进程表取外壳已有的数据：组件来自 /api/health 的监督者组件表（store.services），处理器与内存按进程号取 system.metrics。 */
import { mountSystem, managerRequest } from '/static/shared/system-pages.js';

export function mount(root, ctx) {
  ctx.loadCss('/static/shared/system-pages.css');
  let processes = ctx.stream.latest('system.metrics')?.payload?.processes || [];
  const page = mountSystem(root, {
    mode: 'console',
    request: managerRequest,
    openSettings: panel => ctx.go('settings', panel || ''),
    openLogs: () => ctx.go('logs'),
    services(push) {
      const send = () => push({ components: ctx.store.get().services, processes, telemetry: ctx.store.get().metrics.status });
      ctx.stream.on('system.metrics', frame => { processes = frame.payload?.processes || []; send(); });
      ctx.store.subscribe(['services', 'metrics'], send);
      send();
    },
    // 驱动、定位执行链与控制方式来自控制台已读到的数据。
    device: () => [['控制方式', ctx.store.get().robot.controlMode || '未知'], ['定位执行链', window.__NUWAX_GEOMETRY_PIPELINE__ || '未知']],
  });
  return { show() { page.show(); }, hide() { page.hide(); } };
}
