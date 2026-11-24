/* 外壳：页面栏、顶栏（机器人与地图、一句话状态、设备指标、音量、需要注意、急停）、视图切换与地址同步。
   页面模块约定：export function mount(root, ctx) 返回 {show(sub, opts), hide()}；监控页随外壳加载，其余页面第一次打开时加载。 */
import { icon, h, toast, popover, setTheme, currentTheme } from '/static/shared/neh-ui.js';
import * as ui from '/static/shared/neh-ui.js';
import { deriveActivity, fmtGB, batteryTone, latencyTone, loadTone, ESTOP_NOTICE_MS } from './status.js';
import * as monitorPage from './pages/monitor.js';

export const VIEWS = ['monitor', 'caps', 'logs', 'settings', 'system'];
const LAZY_PAGES = { caps: () => import('./pages/caps.js'), logs: () => import('./pages/logs.js'),
  settings: () => import('./pages/settings.js'), system: () => import('./pages/system.js') };

export function createShell({ store, stream, api, tasks, feeds }) {
  const $ = id => document.getElementById(id);
  document.querySelectorAll('[data-icon]').forEach(el => { el.outerHTML = icon(el.dataset.icon); });

  /* ---------- 视图与地址：#/视图/子页 ---------- */
  const pages = {};
  const lastSub = {};
  let current = null;
  const loadedCss = new Set();
  const ctx = {
    store, stream, api, tasks, feeds, ui,
    go,
    /* 页面模块自带的样式只加载一次。 */
    loadCss(href) {
      if (loadedCss.has(href)) return;
      loadedCss.add(href);
      document.head.append(h('link', { rel: 'stylesheet', href }));
    },
  };
  function parseHash() {
    const [view, sub = ''] = location.hash.replace(/^#\/?/, '').split('/');
    return VIEWS.includes(view) ? { view, sub: decodeURIComponent(sub) } : { view: 'monitor', sub: '' };
  }
  /* 切换视图：换页面写入浏览器历史（可后退），同一页面内换子页只替换地址。sub 省略时回到该页上次的子页。 */
  function go(view, sub, opts = {}) {
    if (!VIEWS.includes(view)) view = 'monitor';
    if (sub === undefined) sub = lastSub[view] || '';
    const hash = '#/' + view + (sub ? '/' + encodeURIComponent(sub) : '');
    if (location.hash !== hash) {
      if (current && current.view === view) history.replaceState(null, '', hash);
      else history.pushState(null, '', hash);
    }
    void show(view, sub, opts);
  }
  async function page(view) {
    if (!pages[view]) {
      const mod = view === 'monitor' ? monitorPage : await LAZY_PAGES[view]();
      pages[view] = mod.mount($('view-' + view), ctx);
    }
    return pages[view];
  }
  async function show(view, sub, opts = {}) {
    lastSub[view] = sub;
    const previous = current?.view;
    current = { view, sub };
    store.set({ route: { view, sub } });
    let entering = null;
    if (previous !== view) {
      document.querySelectorAll('.n-rail-item[data-view]').forEach(b => b.setAttribute('aria-current', b.dataset.view === view ? 'page' : 'false'));
      for (const name of VIEWS) $('view-' + name).hidden = name !== view;
      if (previous && pages[previous]) pages[previous].hide();
      // 从页面栏换到另一个视图时，主内容区整体淡入并上移几像素（首次打开不做）。
      if (previous) entering = ui.enter($('view-' + view), { from: 'translateY(6px)', speed: 'slow' });
    }
    const target = await page(view);
    if (current.view !== view) return;
    target.show(sub, opts);
    // 页面在显示时自己开始的进入动画（如监控页右侧面板换页签的滑入）直接放到结束，免得和整体淡入叠成两次。
    if (entering) {
      for (const a of $('view-' + view).getAnimations({ subtree: true })) {
        if (a !== entering && !(a instanceof CSSAnimation) && !(a instanceof CSSTransition) && a.effect?.getTiming().iterations !== Infinity) a.finish();
      }
    }
  }
  window.addEventListener('popstate', () => { const r = parseHash(); if (!current || r.view !== current.view || r.sub !== current.sub) void show(r.view, r.sub); });
  window.addEventListener('hashchange', () => { const r = parseHash(); if (!current || r.view !== current.view || r.sub !== current.sub) void show(r.view, r.sub); });
  document.querySelectorAll('.n-rail-item[data-view]').forEach(b => b.addEventListener('click', () => go(b.dataset.view)));
  document.querySelector('.n-rail-brand').addEventListener('click', e => { e.preventDefault(); go('monitor'); });

  /* ---------- 主题 ---------- */
  function syncThemeButton() {
    const dark = currentTheme() === 'dark';
    $('themeBtn').innerHTML = icon(dark ? 'sun' : 'moon') + (dark ? '亮色' : '暗色');
  }
  $('themeBtn').addEventListener('click', () => { setTheme(currentTheme() === 'dark' ? 'light' : 'dark'); syncThemeButton(); });
  syncThemeButton();

  /* ---------- 顶栏左侧：机器人与当前地图（纯文字） ---------- */
  function renderRobot() {
    const s = store.get();
    $('robotName').textContent = s.robot.name || '机器人未命名';
    $('robotSub').textContent = s.geometry.mapName && s.geometry.phase !== 'idle' ? s.geometry.mapName : '未加载地图';
    const down = s.conn.status === 'lost' || s.business !== 'running';
    $('connDot').className = 'n-dot ' + (down ? 'n-dot--bad' : s.conn.status === 'open' ? 'n-dot--ok' : '');
    $('connDot').title = down ? '与网关断开' : s.conn.status === 'open' ? '已连接网关' : '正在连接网关';
  }
  store.subscribe(['robot', 'geometry', 'conn', 'business'], renderRobot);

  /* ---------- 一句话状态 ---------- */
  const extra = { navRange: tasks.navRange, capabilities: null };
  api.capabilities().then(list => { extra.capabilities = new Map(list.map(cap => [cap.name, cap])); renderActivity(); }).catch(() => {});
  let actionKey = '', currentAction = null;
  // 进度条常驻复用：每秒刷新时不重建，不确定进度的动画才不会每秒从头开始。
  const progressBar = h('div', { class: 'n-progress' }, h('span', {}));
  function metaNode(item) {
    if (typeof item === 'string') return h('span', { class: 'n-num' }, item);
    if (item.kind === 'source') return h('span', { class: 'c-src' }, item.text);
    return h('span', { class: 'c-activity-err', title: item.text }, item.text);
  }
  function renderMeta(items) {
    const meta = $('activityMeta');
    const progress = items.find(item => item.kind === 'progress');
    const rest = items.filter(item => item.kind !== 'progress').map(metaNode);
    if (progress) {
      progressBar.classList.toggle('n-progress--indeterminate', progress.value == null);
      progressBar.firstChild.style.width = Math.round((progress.value ?? 0) * 100) + '%';
      if (meta.firstChild !== progressBar) meta.prepend(progressBar);
      while (progressBar.nextSibling) progressBar.nextSibling.remove();
      meta.append(...rest);
    } else meta.replaceChildren(...rest);
  }
  function runAction() {
    const action = currentAction;
    if (!action || action.disabled) return;
    if (action.kind === 'system') go('system');
    else if (action.kind === 'reconnect') stream.reconnectNow();
    else if (action.kind === 'stop') void tasks.stop(action.task);
  }
  function renderActivity() {
    const s = store.get();
    const a = deriveActivity(s, Date.now(), extra);
    const box = $('activity');
    box.dataset.tone = a.tone;
    const iconHtml = icon(a.icon, 'n-icon--sm');
    if ($('activityIcon').innerHTML !== iconHtml) $('activityIcon').innerHTML = iconHtml;
    if (a.quote) $('activityText').replaceChildren(a.text, h('q', {}, a.quote));
    else $('activityText').textContent = a.text;
    $('activityText').title = a.quote ? a.text + '「' + a.quote + '」' : a.text;
    renderMeta(a.meta);
    // 按钮只在文字或状态变化时重建，避免每秒刷新时吞掉正在进行的点击；点击时取最新的任务快照。
    currentAction = a.action || null;
    const key = a.action ? [a.action.kind, a.action.label, a.action.disabled, a.action.task?.task_id].join('|') : '';
    if (key !== actionKey) {
      actionKey = key;
      $('activityAction').replaceChildren(...(a.action ? [h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: a.action.disabled, onclick: runAction }, a.action.label)] : []));
    }
    $('topProgress').style.width = (a.tone === 'busy' && a.progress != null ? Math.round(a.progress * 100) : 0) + '%';
  }
  store.subscribe(['business', 'manager', 'conn', 'services', 'tasks', 'estop', 'geometry', 'clockOffsetMs'], renderActivity);
  ui.fadeOnChange($('activityText'));
  // 用时与急停提示按秒刷新；没有任务、急停提示也已过期时不重绘。
  setInterval(() => {
    const s = store.get();
    if (s.tasks.active.length || (s.estop && Date.now() - s.estop.at < ESTOP_NOTICE_MS + 1000)) renderActivity();
  }, 1000);

  /* ---------- 设备指标：电量、连接延迟、处理器、内存与音量 ---------- */
  const volumeButton = h('button', { class: 'c-stat c-stat--btn', type: 'button', title: '调整机器人扬声器音量' });
  volumeButton.addEventListener('click', () => volumePopover());
  function stat(label, value, opts = {}) {
    return h('div', { class: 'c-stat', dataset: Object.assign({ tone: opts.tone || '' }, opts.optional ? { optional: '' } : {}), title: opts.title || '' },
      h('span', { class: 'c-stat-label' }, label), h('span', { class: 'c-stat-value n-num' }, value, opts.extra ? h('small', {}, opts.extra) : null));
  }
  function renderVitals() {
    const { robot, conn, metrics, volume, business } = store.get();
    const lost = conn.status === 'lost';
    const battery = robot.battery;
    // 断开或业务未运行时处理器、内存不再更新，不留断开前的旧数。
    const stale = lost || business !== 'running';
    const cpu = stale ? null : metrics.cpuPercent, mem = stale ? null : metrics.memPercent;
    const latency = lost ? '已断开' : conn.waitingMs != null ? '等待 ' + (conn.waitingMs / 1000).toFixed(1) + ' 秒'
      : conn.rttMs != null ? conn.rttMs + ' 毫秒' : '--';
    const off = metrics.status === 'disabled';
    const offTitle = '性能监控未开启，可在 设置 → 性能监控 开启';
    const items = [
      stat('电量', battery != null ? battery + '%' : '--', { tone: batteryTone(battery), title: battery == null ? '机器人没有上报电量' : '机器人电量' }),
      stat('连接延迟', latency, { tone: lost ? 'bad' : latencyTone(conn.waitingMs ?? conn.rttMs), title: '网页到网关的往返时间' }),
      stat('处理器', off && !stale ? '未开启' : cpu != null ? Math.round(cpu) + '%' : '--', { optional: true, tone: loadTone(cpu),
        title: off ? offTitle : '设备处理器占用' + (metrics.cpuThreads ? '，' + metrics.cpuThreads + ' 线程' : '') }),
      stat('内存', off && !stale ? '未开启' : mem != null ? Math.round(mem) + '%' : '--', { optional: true, tone: loadTone(mem),
        title: off ? offTitle : metrics.memTotalBytes ? '设备内存 ' + fmtGB(metrics.memUsedBytes) + ' / ' + fmtGB(metrics.memTotalBytes) + ' GB' : '设备内存占用' }),
    ];
    if (volume) {
      volumeButton.replaceChildren(h('span', { class: 'c-stat-label' }, volume.error ? '音量（失败）' : '音量'),
        h('span', { class: 'c-stat-value n-num' }, volume.percent != null ? volume.percent + '%' : '--', h('span', { html: icon('chevron-down', 'n-icon--sm') })));
      volumeButton.title = volume.error ? '音量操作失败：' + volume.error : '调整机器人扬声器音量';
      volumeButton.dataset.tone = volume.error ? 'bad' : '';
      items.push(h('span', { class: 'n-topbar-sep c-vital-sep' }), volumeButton);
    }
    $('vitals').replaceChildren(...items);
  }
  store.subscribe(['robot', 'conn', 'metrics', 'volume', 'business'], renderVitals);

  /* 音量：机器人有 get_volume 与 set_volume 时显示；打开页面读一次，拖动松手后设置（与原控制台相同）。 */
  function volumeState(reply) { return { percent: reply.volume == null ? null : Math.round(reply.volume * 100), error: reply.last_error || '' }; }
  api.capabilities().then(async list => {
    const names = new Set(list.map(cap => cap.name));
    if (!names.has('get_volume') || !names.has('set_volume')) return;
    store.set({ volume: { percent: null, error: '' } });
    try { store.set({ volume: volumeState((await api.invoke('get_volume')).volume_state) }); }
    catch (error) { store.set({ volume: { percent: null, error: error.message } }); }
  }).catch(() => {});
  function volumePopover() {
    const value = store.get().volume?.percent ?? 0;
    const out = h('output', { class: 'n-num' }, value + '%');
    const slider = h('input', { type: 'range', min: 0, max: 100, step: 1, value, style: 'width:100%;accent-color:var(--accent)', 'aria-label': '机器人音量' });
    slider.addEventListener('input', () => { out.textContent = slider.value + '%'; });
    slider.addEventListener('change', async () => {
      try { store.set({ volume: volumeState((await api.invoke('set_volume', { volume: Number(slider.value) / 100 })).volume_state) }); }
      catch (error) { store.set({ volume: { percent: null, error: error.message } }); }
    });
    popover(volumeButton, h('div', { style: 'padding:12px 14px;width:260px;display:grid;gap:10px' },
      h('div', { style: 'display:flex;justify-content:space-between;align-items:baseline' }, h('span', { class: 'n-t3' }, '机器人音量'), out),
      slider, h('span', { class: 'n-meta' }, '机器人说话和提示音的音量，松手后立即生效。')), { align: 'end' });
  }

  /* ---------- 需要注意 ---------- */
  function renderBell() {
    const { unread } = store.get();
    $('alertCount').hidden = !unread;
    $('alertCount').textContent = unread > 99 ? '99+' : String(unread);
  }
  store.subscribe(['unread'], renderBell);
  $('alertBtn').addEventListener('click', () => { feeds.markAttentionRead(); go('monitor', 'activity', { focus: 'attention', open: true }); });

  /* ---------- 系统页圆点：有新版本 ---------- */
  store.subscribe(['manager'], s => { $('updateDot').hidden = !s.manager.updateAvailable; });

  /* ---------- 急停：任何视图可按，不确认、不禁用，可连续发送 ---------- */
  $('estop').addEventListener('click', async () => {
    const at = Date.now();
    const cancelled = store.get().tasks.active.filter(task => task.motion).length;
    store.set({ estop: { at, cancelled: 0, issues: [], error: '', pending: true } });
    $('estop').dataset.fired = 'true';
    setTimeout(() => { $('estop').dataset.fired = 'false'; }, 600);
    try {
      const result = await api.invoke('stop_all_motion', { reason: '网页急停' });
      const issues = Object.entries(result).filter(([, value]) => value !== 'ok')
        .map(([key, value]) => ({ guard: '守卫', task: '任务', geometry: '导航' }[key] || key) + '：' + value);
      store.set({ estop: { at, cancelled, issues, error: '' } });
      toast('已急停：机器人停止运动' + (cancelled ? '，已取消 ' + cancelled + ' 个任务' : '') + (issues.length ? '（' + issues.join('；') + '）' : ''), { tone: 'warn' });
    } catch (error) {
      store.set({ estop: { at, cancelled: 0, issues: [], error: error.message } });
      toast('急停失败：' + error.message, { tone: 'bad' });
    }
  });

  renderRobot(); renderActivity(); renderVitals(); renderBell();
  const start = parseHash();
  history.replaceState(null, '', '#/' + start.view + (start.sub ? '/' + encodeURIComponent(start.sub) : ''));
  void show(start.view, start.sub);
  return ctx;
}
