/* 日志页：一行工具栏（等级按钮带数量、组件、搜索、隐藏网页请求记录、暂停滚动、导出），表格铺满，点一行在右侧抽屉看完整记录。
   数据沿用原日志页（console-logs.js，06 第 6 节）：首屏 /api/logs，before 游标翻页，实时 logs.entry 追加（独立连接，日志洪峰不挤主连接），
   离开本页时关闭实时连接、回来时先重读首屏再接上；缺口时重读最近一页；页面可见时每 3 秒补取独立组件写入文件的日志，不可见时停止、恢复可见立即取一次；导出 /api/logs/export。
   本地最多保留 5000 条，超出时优先保留符合当前筛选的记录。 */
import { icon, h, toast, menu, copyText } from '/static/shared/neh-ui.js';
import { createStream } from '../stream.js';

const CACHE_LIMIT = 5000;
const PAGE_SIZE = 200;
const POLL_MS = 3000;
const LEVEL_NAMES = { trace: '跟踪', debug: '调试', info: '信息', warning: '警告', error: '错误', critical: '严重' };
const SOURCE_NAMES = { python: '平台服务', rust: 'Rust 运行时', native: '原生组件' };
// 等级按钮：错误含严重，调试含跟踪；默认显示错误、警告、信息。
const CHIPS = [['error', '错误', ['error', 'critical']], ['warning', '警告', ['warning']], ['info', '信息', ['info']], ['debug', '调试', ['debug', 'trace']]];
const chipOf = level => CHIPS.find(([, , levels]) => levels.includes(level))?.[0] || 'info';

/* 统一条目：历史查询与实时帧字段名略有不同，按原日志页规则归一。 */
let serial = 0, historicalSerial = 0;
function normalize(raw, historical) {
  const details = raw.details || raw.details_compact || {};
  const source = SOURCE_NAMES[raw.source] ? raw.source : 'python';
  const component = String(raw.component || (source !== raw.source && raw.source) || raw.logger || '未标注组件');
  let timestamp = raw.timestamp_iso || raw.timestamp || raw.ts || '';
  if (typeof timestamp === 'number') timestamp = new Date(timestamp < 1e12 ? timestamp * 1000 : timestamp).toISOString();
  const level = String(raw.level || raw.severity || 'info').toLowerCase();
  const text = String(raw.message ?? raw.msg ?? raw.display_message ?? '');
  return {
    id: String(raw.id || timestamp + '|' + (raw.sequence ?? '') + '|' + component + '|' + text),
    timestamp: String(timestamp), level: level === 'warn' ? 'warning' : level, source, component,
    event: raw.event || raw.event_type || raw.runtime_event_type || null,
    message: text, request_id: raw.request_id || null, task_id: raw.task_id || null,
    capability_name: raw.capability_name || null, exception: raw.exception || null,
    details: typeof details === 'object' && details !== null ? details : { detail: details },
    _order: historical ? --historicalSerial : ++serial,
  };
}
function fromFrame(frame) {
  const payload = frame.payload || {};
  const details = payload.details || payload.details_compact || {};
  return {
    ...payload,
    id: payload.id || (frame.source || 'log') + ':' + (frame.cursor ?? frame.sequence),
    timestamp: payload.timestamp || payload.timestamp_iso || frame.timestamp,
    level: String(payload.level || payload.severity || frame.status || 'info').toLowerCase(),
    source: payload.source || 'python',
    component: payload.component || payload.logger || frame.source || '平台服务',
    event: payload.event || payload.event_type || details.event,
    message: payload.message || payload.display_message || payload.runtime_message || '运行记录',
    request_id: payload.request_id || details.request_id,
    task_id: payload.task_id || details.task_id,
    details,
  };
}
const publicEntry = ({ _order, ...entry }) => entry;
const readable = e => e.details.feedback?.message || (e.message === '运行时事件' && e.details.runtime_message ? e.details.runtime_message : e.message);
const componentName = e => e.details.feedback?.source?.display_name || e.component;
// 网页请求记录：成功完成的 HTTP 请求（信息与调试级）；失败、慢请求是警告或错误，照常显示。
const isWebRequest = e => e.event === 'http.request.completed' && (e.level === 'info' || e.level === 'debug');
function asText(e) {
  const context = [['事件', 'event'], ['请求', 'request_id'], ['任务', 'task_id'], ['能力', 'capability_name']]
    .filter(([, key]) => e[key]).map(([label, key]) => label + '（' + key + '）：' + e[key]);
  return [e.timestamp + ' [' + e.level.toUpperCase() + '] [' + e.source + '/' + e.component + ']', e.message, ...context,
    Object.keys(e.details).length ? '详细字段：\n' + JSON.stringify(e.details, null, 2) : '', e.exception ? '异常栈：\n' + e.exception : ''].filter(Boolean).join('\n');
}
function clock(ts) {
  const d = new Date(ts);
  if (Number.isNaN(d.getTime())) return '时间未知';
  const time = d.toLocaleTimeString('zh-CN', { hour12: false });
  return d.toDateString() === new Date().toDateString() ? time : (d.getMonth() + 1) + '-' + d.getDate() + ' ' + time;
}

export function mount(root, ctx) {
  ctx.loadCss('/static/next/pages.css');
  root.classList.add('n-page--full', 'c-logs');
  const records = new Map();
  const rowNodes = new Map();
  const st = { levels: new Set(['error', 'warning', 'info']), component: '', query: '', task: '', request: '', hideWeb: true,
    paused: false, pending: 0, displayLimit: PAGE_SIZE, nextBefore: null, hasMore: false, loading: false, request_seq: 0,
    note: '', gapNote: '', selected: null, visible: false, renderTimer: null, filterTimer: null, gapReload: null };

  const toolbar = h('div', { class: 'n-toolbar' });
  const notice = h('div', { class: 'c-log-notice', hidden: true });
  const table = h('table', { class: 'c-logtable' },
    h('colgroup', {}, h('col', { class: 'c-col-time' }), h('col', { class: 'c-col-level' }), h('col', { class: 'c-col-comp' }), h('col', {}), h('col', { class: 'c-col-task' }), h('col', { class: 'c-col-chev' })));
  const tbody = h('tbody');
  table.append(tbody);
  const footNote = h('span', { class: 'n-meta' });
  const moreBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => void more() }, '加载更早的记录');
  const pageBody = h('div', { class: 'n-page-body c-log-body' }, notice, h('div', { class: 'c-logwrap' }, table), h('div', { class: 'c-log-foot' }, footNote, moreBtn));
  root.append(toolbar, pageBody);

  /* ---------- 筛选与查询参数 ---------- */
  const levelList = () => CHIPS.filter(([key]) => st.levels.has(key)).flatMap(([, , levels]) => levels);
  function params() {
    const query = new URLSearchParams();
    if (st.levels.size < CHIPS.length) query.set('level', levelList().join(','));
    for (const [key, value] of [['component', st.component], ['query', st.query], ['task_id', st.task], ['request_id', st.request]]) if (value) query.set(key, value);
    return query;
  }
  function matchesExceptLevel(e) {
    if (st.hideWeb && isWebRequest(e)) return false;
    if (st.component && !e.component.toLowerCase().includes(st.component.toLowerCase())) return false;
    if (st.task && e.task_id !== st.task) return false;
    if (st.request && e.request_id !== st.request) return false;
    return !st.query || JSON.stringify(publicEntry(e)).toLowerCase().includes(st.query.toLowerCase());
  }
  const matches = e => st.levels.has(chipOf(e.level)) && matchesExceptLevel(e);
  const ordered = () => [...records.values()].sort((a, b) => (Date.parse(b.timestamp) || 0) - (Date.parse(a.timestamp) || 0) || b._order - a._order);
  // 改筛选：立即废弃旧查询，防抖后按新条件重读第一页。
  function changed(immediate) {
    st.displayLimit = PAGE_SIZE;
    st.gapNote = '';
    clearTimeout(st.filterTimer);
    st.request_seq += 1;
    syncToolbar();
    scheduleRender();
    st.filterTimer = setTimeout(reload, immediate ? 0 : 250);
  }

  /* ---------- 工具栏：控件只建一次，实时记录到达时只更新数量与按钮文字 ---------- */
  const chips = CHIPS.map(([key, label]) => {
    const count = h('b', { class: 'n-num' }, '0');
    const btn = h('button', { class: 'c-levelchip n-tip n-tip--below', type: 'button', 'data-tip': '已读取记录中的条数，点击显示或隐藏这一级',
      onclick: () => { if (st.levels.has(key)) st.levels.delete(key); else st.levels.add(key); changed(true); } },
      key === 'error' || key === 'warning' ? h('span', { class: 'n-dot ' + (key === 'error' ? 'n-dot--bad' : 'n-dot--warn') }) : null, label, count);
    return { key, btn, count };
  });
  const comp = h('select', { class: 'n-select c-log-comp', 'aria-label': '组件' });
  comp.addEventListener('change', () => { st.component = comp.value; changed(true); });
  const q = h('input', { class: 'n-input', type: 'search', placeholder: '搜索消息、事件、请求或任务标识', 'aria-label': '搜索日志' });
  q.addEventListener('input', () => { st.query = q.value.trim(); changed(false); });
  const idFilters = h('span', { class: 'c-log-idfilters' });
  const web = h('input', { type: 'checkbox', checked: st.hideWeb });
  web.addEventListener('change', () => { st.hideWeb = web.checked; syncToolbar(); scheduleRender(); });
  const pause = h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => { st.paused = !st.paused; syncToolbar(); scheduleRender(); } });
  const exp = h('button', { class: 'n-btn n-btn--sm', type: 'button', 'aria-haspopup': 'menu', html: icon('download', 'n-icon--sm') }, '导出', h('span', { html: icon('chevron-down', 'n-icon--sm'), style: 'display:flex;color:var(--muted)' }));
  exp.addEventListener('click', () => menu(exp, [
    { label: '下载文本日志（.txt）', icon: 'download', run: () => exportLogs('text') },
    { label: '下载结构化日志（.jsonl）', icon: 'download', run: () => exportLogs('jsonl') }], { align: 'end' }));
  toolbar.append(h('h1', {}, '日志'), ...chips.map(c => c.btn), h('span', { class: 'n-toolbar-sep' }), comp,
    h('div', { class: 'n-search c-log-search', html: icon('search', 'n-icon--sm') }, q), idFilters,
    h('label', { class: 'n-switch n-meta n-tip n-tip--below', 'data-tip': '隐藏成功完成的网页请求（HTTP 请求完成）；失败和慢请求仍显示' }, web, h('span', { class: 'n-switch-track' }), '隐藏网页请求记录'),
    h('span', { class: 'n-toolbar-spacer' }), pause, exp);
  let componentsShown = '';
  function syncToolbar() {
    const all = [...records.values()].filter(matchesExceptLevel);
    for (const c of chips) {
      c.btn.setAttribute('aria-pressed', String(st.levels.has(c.key)));
      c.count.textContent = all.filter(e => chipOf(e.level) === c.key).length;
    }
    // 组件下拉在使用中不重建，避免打开的下拉被收起。
    const components = [...new Set([...records.values()].map(e => e.component))].sort();
    if (st.component && !components.includes(st.component)) components.unshift(st.component);
    if (document.activeElement !== comp && components.join('\n') !== componentsShown) {
      componentsShown = components.join('\n');
      comp.replaceChildren(h('option', { value: '' }, '全部组件'), ...components.map(c => h('option', { value: c, selected: st.component === c }, c)));
    }
    const idChip = (label, value, clear) => value ? h('span', { class: 'c-log-idfilter' }, label, h('span', { class: 'n-code' }, value),
      h('button', { type: 'button', title: '取消这个筛选', 'aria-label': '取消' + label + '筛选', html: icon('x', 'n-icon--sm'), onclick: () => { clear(); changed(true); } })) : null;
    idFilters.replaceChildren(...[idChip('任务', st.task, () => { st.task = ''; }), idChip('请求', st.request, () => { st.request = ''; })].filter(Boolean));
    pause.setAttribute('aria-pressed', String(st.paused));
    pause.innerHTML = icon(st.paused ? 'play' : 'pause', 'n-icon--sm');
    pause.append(st.paused ? (st.pending ? '继续（' + st.pending + ' 条新记录）' : '继续滚动') : '暂停滚动');
  }

  /* ---------- 数据 ---------- */
  function addEntries(entries, historical = false) {
    let count = 0;
    for (const raw of entries) {
      if (!raw) continue;
      const entry = normalize(raw, historical);
      if (records.has(entry.id)) continue;
      records.set(entry.id, entry);
      count += 1;
    }
    if (records.size > CACHE_LIMIT) {
      const sorted = ordered();
      const keep = new Set([...sorted.filter(matches), ...sorted.filter(e => !matches(e))].slice(0, CACHE_LIMIT).map(e => e.id));
      for (const id of records.keys()) if (!keep.has(id)) { records.delete(id); rowNodes.delete(id); }
    }
    if (count) { st.pending += count; scheduleRender(); }
  }
  async function load(append = false, quiet = false) {
    if (append && st.loading) return;
    const seq = ++st.request_seq;
    st.loading = true;
    const query = params();
    query.set('limit', String(PAGE_SIZE));
    if (append && st.nextBefore) query.set('before', st.nextBefore);
    try {
      const result = await ctx.api.json('/api/logs?' + query);
      if (seq !== st.request_seq) return;
      // 定时补取只追加新记录，不改翻页游标。
      if (!quiet || !st.nextBefore) { st.nextBefore = result.next_before; st.hasMore = Boolean(result.has_more); }
      addEntries(result.entries || [], true);
      st.note = result.cursor_expired ? '所选历史已超出文件保留范围，请改筛选条件重新查询。' : '';
    } catch (error) {
      if (seq === st.request_seq) st.note = '日志暂时无法读取，将自动重试。原因：' + error.message;
    } finally {
      if (seq === st.request_seq) { st.loading = false; scheduleRender(); }
    }
  }
  function reload() { st.nextBefore = null; st.hasMore = false; return load(false); }
  async function more() {
    if (st.displayLimit >= CACHE_LIMIT) { st.note = '最多显示 5000 条，请缩小筛选范围或导出历史。'; scheduleRender(); return; }
    st.displayLimit = Math.min(CACHE_LIMIT, st.displayLimit + PAGE_SIZE);
    if (st.hasMore) await load(true);
    scheduleRender();
  }
  async function exportLogs(format) {
    const query = params();
    query.set('format', format);
    query.set('limit', '100000');
    toast('正在准备日志文件…', { tone: 'info' });
    try {
      const response = await fetch('/api/logs/export?' + query, { cache: 'no-store' });
      if (!response.ok) throw new Error('服务返回 ' + response.status);
      const url = URL.createObjectURL(await response.blob());
      const anchor = h('a', { href: url, download: 'NEH日志_' + new Date().toISOString().slice(0, 10) + '.' + (format === 'text' ? 'txt' : 'jsonl') });
      document.body.append(anchor); anchor.click(); anchor.remove();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      toast(response.headers.get('X-Logs-Has-More') === 'true' ? '已导出最近 100000 条匹配的日志；更早的记录请缩小筛选范围后再导出' : '日志文件已开始下载', { tone: 'info' });
    } catch (error) { toast('日志导出失败：' + error.message, { tone: 'bad' }); }
  }

  /* ---------- 实时日志：独立连接，离开本页时关闭，回来时先重读首屏再接上；缺口时重读最近一页 ---------- */
  let stream = null;
  function openStream() {
    stream = createStream({ topics: ['logs.entry'], queued: ['logs.entry'] });
    stream.on('logs.entry', frame => addEntries([fromFrame(frame)]));
    let initialReplay = true;
    stream.onReplay(() => { initialReplay = false; });
    stream.onGap(({ kind }) => {
      // 连接刚打开时的回放缺口由刚重读的首屏覆盖，不另行提示。
      if (kind === 'replay' && initialReplay) return;
      st.gapNote = '实时日志有缺口，已重读最近 ' + PAGE_SIZE + ' 条匹配的记录；更早的记录请加载或导出。';
      if (!st.gapReload) st.gapReload = reload().finally(() => { st.gapReload = null; });
    });
    stream.connect();
  }

  /* ---------- 表格 ---------- */
  function selectedText() {
    const sel = window.getSelection?.();
    return Boolean(sel && !sel.isCollapsed && sel.rangeCount && (table.contains(sel.anchorNode) || table.contains(sel.focusNode)));
  }
  function scheduleRender() { if (st.renderTimer === null) st.renderTimer = setTimeout(render, 100); }
  function row(e) {
    const level = chipOf(e.level);
    const d = e.details;
    const context = e.event === 'http.request.completed' || e.event === 'http.request.failed'
      ? [[d.method, d.path].filter(Boolean).join(' '), d.status_code, Number.isFinite(Number(d.duration_ms)) ? Number(d.duration_ms).toFixed(1) + ' 毫秒' : ''].filter(v => v !== undefined && v !== '').join('  ') : '';
    const tr = h('tr', { class: 'c-logrow', dataset: { level, id: e.id }, onclick: () => openDrawer(e) },
      h('td', { class: 'n-num c-log-time', title: e.timestamp }, clock(e.timestamp)),
      h('td', {}, h('span', { class: 'c-loglevel', html: level === 'error' ? icon('circle-alert', 'n-icon--sm') : level === 'warning' ? icon('triangle-alert', 'n-icon--sm') : '' }, LEVEL_NAMES[e.level] || e.level)),
      h('td', { class: 'c-col-comp' }, h('div', { class: 'c-logcomp', title: e.component }, componentName(e))),
      h('td', {}, h('div', { class: 'c-logmsg', title: readable(e) }, readable(e), context ? h('span', { class: 'c-logctx' }, context) : null)),
      h('td', { style: 'text-align:right' }, e.task_id ? h('button', { class: 'c-chipid', type: 'button', title: '只看这个任务的记录',
        onclick: ev => { ev.stopPropagation(); st.task = e.task_id; changed(true); } }, e.task_id) : null),
      h('td', { class: 'c-col-chev', title: '查看完整记录' }, h('span', { class: 'c-rowchev', html: icon('chevron-right', 'n-icon--sm') })));
    return tr;
  }
  function render() {
    st.renderTimer = null;
    if (!st.visible) return;
    if (st.paused || selectedText()) { syncToolbar(); return; }
    st.pending = 0;
    const filtered = ordered().filter(matches);
    const shown = filtered.slice(0, st.displayLimit);
    const wanted = shown.map(e => {
      if (!rowNodes.has(e.id)) rowNodes.set(e.id, row(e));
      const tr = rowNodes.get(e.id);
      tr.setAttribute('aria-selected', String(st.selected === e.id));
      return tr;
    });
    if (!wanted.length) wanted.push(h('tr', {}, h('td', { colspan: 6 }, h('div', { class: 'n-empty', html: icon(st.loading ? 'scroll-text' : 'search') },
      h('div', { class: 'n-empty-title' }, st.loading ? '正在读取日志…' : '没有符合条件的记录'), st.loading ? null : h('div', {}, '放宽等级或组件筛选再看看。')))));
    const keep = new Set(wanted);
    for (const node of [...tbody.children]) if (!keep.has(node)) node.remove();
    wanted.forEach((node, index) => { if (tbody.children[index] !== node) tbody.insertBefore(node, tbody.children[index] || null); });
    const notes = [st.note, st.gapNote].filter(Boolean);
    notice.hidden = !notes.length;
    notice.textContent = notes.join(' ');
    footNote.textContent = '显示 ' + shown.length + ' 条' + (filtered.length > shown.length ? '，已读取 ' + filtered.length + ' 条' : '');
    moreBtn.disabled = st.loading || st.displayLimit >= CACHE_LIMIT || (!st.hasMore && filtered.length <= shown.length);
    moreBtn.textContent = st.loading ? '正在读取…' : '加载更早的记录';
    syncToolbar();
  }
  document.addEventListener('selectionchange', () => { if (st.visible && st.pending && !selectedText()) scheduleRender(); });

  /* ---------- 详情抽屉 ---------- */
  let drawer = null;
  function closeDrawer() { drawer?.remove(); drawer = null; st.selected = null; tbody.querySelectorAll('tr[aria-selected="true"]').forEach(tr => tr.setAttribute('aria-selected', 'false')); }
  function openDrawer(e) {
    drawer?.remove();
    st.selected = e.id;
    tbody.querySelectorAll('tr.c-logrow').forEach(tr => tr.setAttribute('aria-selected', String(tr.dataset.id === e.id)));
    const level = chipOf(e.level);
    const kv = [['时间', new Date(e.timestamp).toLocaleString('zh-CN', { hour12: false }), e.timestamp], ['来源', SOURCE_NAMES[e.source] + '（' + e.source + '）'],
      ['组件', e.component], ['事件', e.event], ['请求', e.request_id], ['任务', e.task_id], ['能力', e.capability_name], ['记录标识', e.id]].filter(([, v]) => v);
    const copy = async (text, what) => toast(await copyText(text) ? '已复制' + what : '复制失败，请选中内容复制或导出日志', { tone: 'info' });
    drawer = h('aside', { class: 'n-drawer', 'aria-label': '日志详情', style: 'top:' + toolbar.offsetHeight + 'px' },
      h('div', { class: 'n-drawer-head' }, h('span', { class: 'n-badge ' + ({ error: 'n-badge--bad', warning: 'n-badge--warn' }[level] || '') }, LEVEL_NAMES[e.level] || e.level),
        h('h2', { title: readable(e) }, readable(e)),
        h('button', { class: 'n-btn n-btn--quiet n-btn--icon', type: 'button', title: '关闭（Esc）', 'aria-label': '关闭', html: icon('x'), onclick: closeDrawer })),
      h('div', { class: 'n-drawer-body' },
        h('dl', { class: 'n-kv' }, kv.flatMap(([label, value, title]) => [h('dt', {}, label), h('dd', { class: label === '时间' || label === '来源' ? 'n-num' : 'n-code', title }, value)])),
        h('div', { class: 'n-sec', style: 'margin-top:20px' }, h('div', { class: 'n-sec-head' }, h('h3', {}, '消息')), h('pre', { class: 'n-pre c-log-pre' }, e.message)),
        Object.keys(e.details).length ? h('div', { class: 'n-sec' }, h('div', { class: 'n-sec-head' }, h('h3', {}, '详细字段')), h('pre', { class: 'n-pre c-json c-log-pre' }, JSON.stringify(e.details, null, 2))) : null,
        e.exception ? h('div', { class: 'n-sec' }, h('div', { class: 'n-sec-head' }, h('h3', {}, '异常栈')), h('pre', { class: 'n-pre c-log-pre' }, e.exception)) : null),
      h('div', { class: 'n-drawer-foot' },
        e.task_id ? h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => { st.task = e.task_id; changed(true); } }, '只看这个任务') : null,
        e.request_id ? h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => { st.request = e.request_id; changed(true); } }, '只看这个请求') : null,
        h('span', { class: 'n-toolbar-spacer' }),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('copy', 'n-icon--sm'), onclick: () => copy(asText(e), '这条记录') }, '复制文本'),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => copy(JSON.stringify(publicEntry(e), null, 2), ' JSON') }, '复制 JSON')));
    root.append(drawer);
  }
  document.addEventListener('keydown', ev => { if (ev.key === 'Escape' && st.visible && drawer && !document.querySelector('dialog[open], .n-popover')) closeDrawer(); });

  /* ---------- 可见性：可见时每 3 秒补取，不可见时停止 ---------- */
  setInterval(() => { if (st.visible && !document.hidden && !st.loading) void load(false, true); }, POLL_MS);
  document.addEventListener('visibilitychange', () => { if (st.visible && !document.hidden && !st.loading) void load(false, true); });

  syncToolbar();
  return {
    // 筛选、暂停、已读取的记录与选中项都保留；回来时重读首屏（翻页游标从头算），读完再接实时流。
    show() {
      st.visible = true;
      void reload().then(() => { if (st.visible && !stream) openStream(); });
      scheduleRender();
    },
    hide() {
      st.visible = false;
      stream?.close();
      stream = null;
    },
  };
}
