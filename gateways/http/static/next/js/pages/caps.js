/* 能力页：三栏，左侧按分组列出能力（分组可折叠），中间是标题区与参数表单，右侧是执行结果。
   行为沿用原能力页（capability-panel.js，06 第 8 节）：经 list_capabilities 取全部能力，按 input_schema 生成表单，
   留空的字段不发送；motion 为真或名称以 delete_ 开头的能力先确认；任务类按任务流显示状态与进度、可取消，结束后显示结果；
   失败显示后端原因与错误码。新增能力不改网页即出现。地址 #/caps/<能力名> 打开该能力。 */
import { icon, h, toast, confirmDialog, sub as subBlock, copyText, collapse, enter, progressBar } from '/static/shared/neh-ui.js';
import { isTerminalTaskState, normalizeTaskState, taskKind } from '../status.js';

// 类型用含义贴切的图标：查询看一眼（眼睛），动作立即生效（闪电），任务要花时间（沙漏）。
const KIND = {
  query: ['查询', 'eye', '读取状态，立即返回结果，不改变机器人。'],
  action: ['动作', 'zap', '执行一次，立即返回结果。'],
  task: ['任务', 'hourglass', '提交后在后台执行，进度显示在顶栏和监控页“活动”，可随时取消；结束时返回结果。'],
};
const TYPE_NAMES = { number: '数值', integer: '整数', string: '文字', boolean: '是或否', enum: '选项', json: '结构（JSON）' };
const TASK_STATES = { running: '运行中', cancelling: '正在取消', succeeded: '已完成', failed: '失败', cancelled: '已取消', timed_out: '超时' };
const RESULT_LIMIT = 200000;

/* 参数结构：解析本地引用，可空字段取非空分支（与原能力页相同）。 */
function resolve(schema, root) {
  let value = schema || {};
  if (value.$ref && value.$ref.startsWith('#/$defs/')) value = { ...root.$defs?.[value.$ref.slice(8)], ...value, $ref: undefined };
  const options = value.anyOf || value.oneOf;
  if (options && !value.type) {
    const concrete = options.filter(item => item.type !== 'null');
    if (concrete.length === 1) value = { ...resolve(concrete[0], root), description: value.description, default: value.default };
  }
  return value;
}
function controlKind(schema) {
  if (Array.isArray(schema.enum)) return 'enum';
  if (['string', 'number', 'integer', 'boolean'].includes(schema.type)) return schema.type;
  return 'json';
}
function hintText(schema) {
  const parts = [];
  if (schema.description) parts.push(schema.description);
  const range = [['minimum', '最小 '], ['exclusiveMinimum', '大于 '], ['maximum', '最大 '], ['exclusiveMaximum', '小于 ']]
    .filter(([key]) => schema[key] !== undefined).map(([key, label]) => label + schema[key]);
  if (range.length) parts.push(range.join('，'));
  if (schema.default !== undefined && schema.default !== null) parts.push('默认 ' + JSON.stringify(schema.default));
  return parts.join('；');
}
const escapeHtml = text => text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
// 返回内容按 JSON 着色：键、字符串、数值、布尔各一色，代码块随主题用浅色或深色底。
function consoleHtml(text) {
  return escapeHtml(text).replace(/"([^"\n]+)":/g, '<span class="k">"$1"</span>:').replace(/: "([^"\n]*)"/g, ': <span class="s">"$1"</span>')
    .replace(/: (-?\d+\.?\d*(?:e[+-]?\d+)?)/gi, ': <span class="n">$1</span>').replace(/: (true|false|null)/g, ': <span class="b">$1</span>');
}
function limited(value) {
  const text = typeof value === 'string' ? value : JSON.stringify(value, null, 2);
  return text.length > RESULT_LIMIT ? text.slice(0, RESULT_LIMIT) + '\n…（内容过长已截断，共 ' + text.length + ' 字符）' : text;
}
const titleOf = cap => String(cap.description || cap.name).split('\n')[0].replace(/。$/, '');
const kindIcon = kind => h('span', { class: 'c-kind', dataset: { kind }, html: icon(KIND[kind]?.[1] || 'puzzle', 'n-icon--sm') });

export function mount(root, ctx) {
  ctx.loadCss('/static/next/pages.css');
  const st = { caps: [], selected: null, kind: 'all', query: '', openGroups: new Set(), fields: [], busy: false };
  const results = new Map();    // 能力名 → 最近一次执行记录（切回该能力时仍能看到，任务结束时更新）

  const search = h('input', { class: 'n-input', type: 'search', placeholder: '搜索名称、说明或分组', 'aria-label': '搜索能力' });
  const kindSeg = h('div', { class: 'n-seg n-seg--block', id: 'capKind', role: 'group', 'aria-label': '按类型筛选' });
  const list = h('div', { class: 'c-split-scroll' });
  const main = h('div', { class: 'c-caps-main' });
  const result = h('aside', { class: 'c-caps-result', 'aria-label': '执行结果' });
  const split = h('div', { class: 'c-caps' },
    h('div', { class: 'c-caps-list' }, h('div', { class: 'c-split-list-head' }, h('div', { class: 'n-search', html: icon('search', 'n-icon--sm') }, search), kindSeg), list),
    main, result);
  root.append(split);
  // 窄屏列表与详情互换时，新内容从前进方向淡入。
  const narrow = matchMedia('(max-width: 760px)');
  function showDetail(on) {
    split.classList.toggle('is-detail', on);
    if (narrow.matches) enter(split, { from: on ? 'translateX(12px)' : 'translateX(-12px)' });
  }
  search.addEventListener('input', () => { st.query = search.value.trim().toLowerCase(); renderList(); });

  /* ---------- 左侧：类型筛选与分组列表 ---------- */
  function renderKinds() {
    kindSeg.replaceChildren(...[['all', '全部'], ['query', '查询'], ['action', '动作'], ['task', '任务']].map(([k, label]) =>
      h('button', { type: 'button', 'aria-pressed': String(st.kind === k), onclick: () => { st.kind = k; renderKinds(); renderList(); } }, k === 'all' ? null : kindIcon(k), label)));
  }
  function renderList() {
    const q = st.query;
    const caps = st.caps.filter(c => (st.kind === 'all' || c.kind === st.kind) && (!q || [c.name, c.description, c.group].some(t => String(t || '').toLowerCase().includes(q))));
    const groups = [...new Set(caps.map(c => c.group || '其它'))];
    if (!groups.length) {
      list.replaceChildren(h('div', { class: 'n-empty', html: icon('search') }, h('div', { class: 'n-empty-title' }, '没有匹配的能力'), h('div', {}, '换个关键词，或改选“全部”类型。')));
      return;
    }
    list.replaceChildren(...groups.map(g => {
      const items = caps.filter(c => (c.group || '其它') === g);
      const open = Boolean(q) || st.openGroups.has(g);
      return h('div', { class: 'c-capgroup', dataset: { open: String(open) } },
        // 就地展开收起（高度过渡），不重建整个列表。
        h('button', { class: 'c-capgroup-head', type: 'button', 'aria-expanded': String(open), onclick: ev => {
          const head = ev.currentTarget, group = head.parentElement;
          const next = group.dataset.open !== 'true';
          if (next) st.openGroups.add(g); else st.openGroups.delete(g);
          const animate = collapse(group.lastElementChild);
          group.dataset.open = String(next); head.setAttribute('aria-expanded', String(next));
          animate(next);
        } }, h('span', { class: 'c-gicon', html: icon(items[0].group_icon || 'puzzle') }), h('b', {}, g), h('span', { class: 'n-meta' }, items.length), h('span', { html: icon('chevron-right', 'c-chev') })),
        h('div', { class: 'c-capgroup-body' }, items.map(c => h('button', { class: 'n-row c-cap-row', type: 'button', 'aria-selected': String(st.selected === c), title: c.description,
          onclick: () => { select(c); showDetail(true); ctx.go('caps', c.name); } },
          h('span', { class: 'n-tip n-tip--right', 'data-tip': KIND[c.kind]?.[0] || c.kind, style: 'display:flex;align-self:flex-start;margin-top:2px' }, kindIcon(c.kind)),
          h('div', { class: 'n-row-main' }, h('div', { class: 'n-row-title' }, titleOf(c)), h('div', { class: 'n-row-sub' }, c.name)),
          c.motion ? h('span', { class: 'n-tip n-tip--left c-motion', 'data-tip': '会让机器人移动', html: icon('footprints', 'n-icon--sm') }) : null))));
    }));
  }

  /* ---------- 中间：标题区与参数表单 ---------- */
  function buildField(name, schema, required, root) {
    const kind = controlKind(schema);
    let control;
    if (kind === 'enum' || kind === 'boolean') {
      control = h('select', { class: 'n-select' });
      if (!required) control.append(h('option', { value: '' }, schema.default === undefined || schema.default === null ? '不指定' : '不指定（默认 ' + JSON.stringify(schema.default) + '）'));
      const values = kind === 'enum' ? schema.enum : [true, false];
      for (const value of values) control.append(h('option', { value: JSON.stringify(value) }, kind === 'boolean' ? (value ? '是（true）' : '否（false）') : String(value)));
      if (required && schema.default !== undefined) control.value = JSON.stringify(schema.default);
    } else if (kind === 'json') {
      control = h('textarea', { class: 'n-textarea n-input--code', rows: 3, spellcheck: 'false',
        placeholder: schema.default !== undefined ? JSON.stringify(schema.default) : schema.type === 'array' ? '填写 JSON 数组，例如 []' : '填写 JSON 对象，例如 {}' });
    } else {
      control = h('input', { class: 'n-input' + (kind === 'string' ? '' : ' n-num'), type: 'text', inputmode: kind === 'string' ? null : 'decimal', autocomplete: 'off',
        placeholder: schema.default !== undefined && schema.default !== null ? '默认 ' + schema.default : '' });
    }
    control.id = 'cap-field-' + name;
    const hint = hintText(schema);
    const field = h('div', { class: 'n-field' + (kind === 'json' ? ' n-field--wide' : ''), style: 'margin-bottom:14px' },
      h('label', { class: 'n-label', for: control.id, style: 'display:flex;gap:8px;align-items:baseline' },
        h('span', { class: 'n-code', style: 'font-weight:600;color:var(--ink)' }, name),
        h('span', { class: 'n-faint', style: 'font-weight:400' }, TYPE_NAMES[kind])),
      control, hint ? h('span', { class: 'n-help' }, hint) : null,
      kind === 'json' ? h('details', { class: 'c-schema' }, h('summary', {}, '参数结构'), h('pre', { class: 'c-console' }, JSON.stringify(root.properties[name], null, 2))) : null);
    st.fields.push({ name, kind, control });
    return field;
  }
  // 留空的字段不发送，由能力按自身默认值处理；必填与取值范围由能力统一检查。
  function collect() {
    const args = {};
    for (const { name, kind, control } of st.fields) {
      const raw = control.value.trim();
      if (raw === '') continue;
      if (kind === 'number' || kind === 'integer') args[name] = Number(raw);
      else if (kind === 'string') args[name] = control.value;
      else if (kind === 'json') {
        try { args[name] = JSON.parse(raw); } catch (error) { throw new Error('参数 ' + name + ' 不是有效的 JSON：' + error.message); }
      } else args[name] = JSON.parse(raw);
    }
    return args;
  }
  function select(cap) {
    if (!cap) return;
    st.selected = cap;
    st.openGroups.add(cap.group || '其它');
    renderList();
    const schema = cap.input_schema || {};
    const required = new Set(schema.required || []);
    const props = Object.keys(schema.properties || {});
    const must = props.filter(n => required.has(n)), opt = props.filter(n => !required.has(n));
    st.fields = [];
    const grid = names => h('div', { class: 'c-form-grid' }, names.map(n => buildField(n, resolve(schema.properties[n], schema), required.has(n), schema)));
    const optSec = opt.length ? subBlock({ title: '可选参数', count: opt.length + ' 项，留空的不发送，由能力使用默认值', open: !must.length }) : null;
    if (optSec) optSec.body.append(h('div', { class: 'c-opt-body' }, grid(opt)));
    const rest = String(cap.description || '').split('\n').slice(1).join('\n').trim();
    const run = h('button', { class: 'n-btn n-btn--primary n-btn--lg', type: 'submit', html: icon('play', 'n-icon--sm'), disabled: st.busy }, cap.kind === 'task' ? '开始任务' : '执行');
    const form = h('form', { class: 'c-caps-form', novalidate: true },
      must.length ? h('section', { class: 'c-param-sec' }, h('div', { class: 'n-sec-head' }, h('h3', {}, '必填参数'), h('span', { class: 'n-meta' }, must.length + ' 项')), grid(must)) : null,
      optSec,
      !props.length ? h('p', { class: 'n-meta', style: 'margin:4px 0' }, '这个能力没有参数，直接执行即可。') : null);
    const hero = h('form', { class: 'c-caphero' },
      h('button', { class: 'c-back', type: 'button', html: icon('chevron-left', 'n-icon--sm'), onclick: () => showDetail(false) }, '全部能力'),
      h('span', { class: 'n-hero-icon', html: icon(cap.group_icon || 'puzzle') }),
      h('div', { style: 'flex:1;min-width:0' },
        h('div', { class: 'n-meta c-capkind' }, kindIcon(cap.kind), KIND[cap.kind]?.[0] || cap.kind, h('span', { class: 'n-faint' }, cap.group || '其它'),
          cap.agent_tool ? null : h('span', { class: 'n-badge n-tip', 'data-tip': '智能体工具列表里没有这个能力' }, '仅网页与接口')),
        h('h1', { class: 'n-t1' }, titleOf(cap)),
        h('button', { class: 'n-set-key n-tip', type: 'button', 'data-tip': '点击复制能力名', style: 'font-size:12px;margin-top:2px', onclick: async () => {
          toast(await copyText(cap.name) ? '已复制 ' + cap.name : '复制失败，请手动选中能力名', { tone: 'info' });
        } }, cap.name),
        rest ? h('p', { class: 'c-capmode' }, rest) : null,
        h('p', { class: 'c-capmode' }, KIND[cap.kind]?.[2] || ''),
        cap.motion ? h('p', { class: 'c-capmotion', html: icon('footprints', 'n-icon--sm') }, h('span', {}, h('b', {}, '会让机器人移动。'), '执行前会再确认一次；新目标会覆盖正在执行的运动。')) : null),
      run);
    // 回车与按钮都执行；两个表单共用一次执行。
    for (const f of [hero, form]) f.addEventListener('submit', event => { event.preventDefault(); void execute(cap); });
    form.append(h('button', { type: 'submit', hidden: true, tabindex: -1 }));
    main.replaceChildren(hero, form);
    renderResult();
  }

  /* ---------- 执行 ---------- */
  async function execute(cap) {
    if (st.busy || cap !== st.selected) return;
    let args;
    try { args = collect(); } catch (error) { results.set(cap.name, { status: 'fail', args: null, message: error.message, code: '' }); renderResult(); return; }
    if (cap.motion && !await confirmDialog({ title: '执行「' + cap.name + '」', text: '这个能力会让机器人运动。确认周围安全后执行；急停随时可用。', confirmText: '执行' })) return;
    if (cap.name.startsWith('delete_') && !await confirmDialog({ title: '执行「' + cap.name + '」', text: '这是删除操作，执行后无法恢复。', confirmText: '删除', danger: true })) return;
    st.busy = true;
    syncRun();
    results.set(cap.name, { status: 'running', args });
    renderResult();
    // 窄屏时结果在参数下方，滚过去让人看到正在执行。
    if (matchMedia('(max-width: 760px)').matches) result.scrollIntoView({ block: 'start', behavior: 'smooth' });
    const started = performance.now();
    try {
      const reply = await ctx.api.invoke(cap.name, args);
      if (cap.kind === 'task' && reply.task) {
        // 任务事件可能先于应答到达，取任务表里较新的快照。
        results.set(cap.name, { status: 'task', args, task: ctx.tasks.get(reply.task.task_id) || reply.task });
      } else results.set(cap.name, { status: 'done', args, reply, ms: Math.round(performance.now() - started) });
    } catch (error) {
      results.set(cap.name, { status: 'fail', args, message: error.message, code: error.code || '' });
    } finally {
      st.busy = false;
      syncRun();
      if (cap === st.selected) renderResult();
    }
  }
  function syncRun() { const run = main.querySelector('.c-caphero .n-btn--primary'); if (run) run.disabled = st.busy; }
  ctx.tasks.onUpdate(task => {
    for (const [name, record] of results) {
      if (record.status !== 'task' || record.task.task_id !== task.task_id) continue;
      record.task = task;
      if (st.selected?.name === name) renderResult();
    }
  });
  // 取消中与取消失败写在任务表里（与顶栏同一份）。
  ctx.store.subscribe(['tasks'], () => { if (results.get(st.selected?.name)?.status === 'task') renderResult(); });

  /* ---------- 右侧：结果 ---------- */
  function renderResult() {
    const cap = st.selected;
    const record = cap && results.get(cap.name);
    const head = (badge, extra) => h('div', { class: 'n-toolbar', style: 'position:static' }, h('h1', {}, '结果'), badge, h('span', { class: 'n-toolbar-spacer' }), extra);
    if (!record) {
      result.replaceChildren(head(null, null), h('div', { class: 'c-result-empty', html: icon('square-terminal') },
        h('b', {}, '还没有运行'), h('span', {}, '点击“' + (cap?.kind === 'task' ? '开始任务' : '执行') + '”后，后端返回的内容显示在这里。')));
      return;
    }
    const request = '> POST /api/capabilities/' + cap.name + '/invoke' + (record.args && Object.keys(record.args).length ? '\n' + JSON.stringify({ arguments: record.args }) : '');
    const consoleBlock = value => h('pre', { class: 'c-console', html: '<span class="c-console-empty">' + escapeHtml(request) + '</span>' + (value === undefined ? '' : '\n' + consoleHtml(limited(value))) });
    const copyBtn = value => h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', html: icon('copy', 'n-icon--sm'), onclick: async () => {
      toast(await copyText(limited(value)) ? '已复制结果' : '复制失败，请手动选中内容', { tone: 'info' });
    } }, '复制');
    const spin = () => h('span', { class: 'n-spin', style: 'width:10px;height:10px;border-width:1.5px' });
    if (record.status === 'running') {
      result.replaceChildren(head(h('span', { class: 'n-badge n-badge--accent' }, spin(), '执行中'), null), h('div', { class: 'c-result-body' }, consoleBlock()));
    } else if (record.status === 'done') {
      result.replaceChildren(head(h('span', { class: 'n-badge n-badge--ok' }, '成功，用时 ' + record.ms + ' 毫秒'), copyBtn(record.reply)),
        h('div', { class: 'c-result-body' }, consoleBlock(record.reply)));
    } else if (record.status === 'fail') {
      result.replaceChildren(head(h('span', { class: 'n-badge n-badge--bad' }, '失败'), null),
        h('div', { class: 'c-result-body' }, h('div', { class: 'n-hl n-hl--bad', style: 'font-size:13px;line-height:1.6' },
          record.message, record.code ? h('div', { class: 'n-meta' }, '错误码 ', h('span', { class: 'n-code' }, record.code)) : null),
        record.args ? consoleBlock() : null));
    } else {
      const task = record.task;
      const state = normalizeTaskState(task.state);
      const done = isTerminalTaskState(state);
      const tasks = ctx.store.get().tasks;
      const cancelError = tasks.errors.get(task.task_id);
      const cancelling = state === 'cancelling' || tasks.pending.has(task.task_id) || (record.cancelling && !cancelError);
      const progress = Number(task.progress);
      const percent = Number.isFinite(progress) && progress > 0 ? Math.round(Math.min(1, progress) * 100) : null;
      const tone = state === 'succeeded' ? 'n-badge--ok' : ['failed', 'timed_out'].includes(state) ? 'n-badge--bad' : state === 'cancelled' ? '' : 'n-badge--accent';
      const badge = h('span', { class: 'n-badge ' + tone }, done ? null : spin(), (TASK_STATES[state] || state) + (!done && percent != null ? ' ' + percent + '%' : ''));
      const stop = done ? null : h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: cancelling, onclick: () => { record.cancelling = true; renderResult(); void ctx.tasks.stop(task); } },
        cancelling ? '正在取消…' : taskKind(task) === 'explore' ? '停止探索' : '取消任务');
      const value = done ? (task.result ?? { state, message: task.message, error_code: task.error_code }) : task;
      result.replaceChildren(head(badge, done ? copyBtn(value) : stop),
        h('div', { class: 'c-result-body' },
          done ? (['failed', 'timed_out'].includes(state) ? h('div', { class: 'n-hl n-hl--bad', style: 'font-size:13px;line-height:1.6' }, task.message || '任务没有完成',
            task.error_code ? h('div', { class: 'n-meta' }, '错误码 ', h('span', { class: 'n-code' }, task.error_code)) : null) : null)
            : h('div', { class: 'n-hl', style: 'display:grid;gap:8px;font-size:13px' },
              h('div', {}, task.message || task.stage || '已受理，后台执行中'),
              progressBar('cap-task:' + task.task_id, percent),
              h('div', { class: 'n-meta' }, '进度也显示在顶栏与监控页“活动”。任务 ', h('span', { class: 'n-code' }, task.task_id)),
              cancelError ? h('div', { style: 'color:var(--bad)' }, '取消失败：' + cancelError) : null),
          consoleBlock(value)));
    }
  }

  /* ---------- 读取能力目录 ---------- */
  let loading = null;
  function load() {
    if (!loading) {
      list.replaceChildren(h('div', { class: 'n-empty' }, h('span', { class: 'n-spin' }), h('div', {}, '正在读取能力清单…')));
      loading = ctx.api.capabilities().then(caps => { st.caps = caps; renderKinds(); renderList(); return caps; })
        .catch(error => {
          loading = null;
          list.replaceChildren(h('div', { class: 'n-empty', html: icon('circle-alert') }, h('div', { class: 'n-empty-title' }, '读取能力清单失败'), h('div', {}, error.message),
            h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => { void open(); } }, '重新读取')));
          return null;
        });
    }
    return loading;
  }
  let pendingSub = '';
  async function open(sub = pendingSub) {
    pendingSub = sub;
    const caps = await load();
    if (!caps) return;
    const cap = caps.find(c => c.name === sub) || (st.selected ? null : caps[0]);
    if (cap && cap !== st.selected) select(cap);
    if (!st.selected) renderResult();
  }
  renderKinds();
  return {
    show(sub) { void open(sub); },
    hide() {},
  };
}
