/* 全部能力：按 list_capabilities 返回的参数结构生成表单，经与智能体相同的能力调用执行。 */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  const KIND_LABELS = {query: '查询', action: '动作', task: '任务'};
  const TASK_STATES = {running: '运行中', cancelling: '正在取消', succeeded: '已完成', failed: '失败', cancelled: '已取消', timed_out: '超时'};
  const SUBMIT_LABELS = {query: '查询', action: '执行', task: '提交任务'};
  const RESULT_LIMIT = 200000;

  async function invoke(name, args = {}) {
    const response = await fetch('/api/capabilities/' + encodeURIComponent(name) + '/invoke', {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: JSON.stringify({arguments: args})});
    const value = await response.json();
    if (!response.ok) {
      const error = new Error(value.error?.message || '服务返回 ' + response.status);
      error.code = value.error?.code || '';
      throw error;
    }
    return value;
  }
  // 能力清单只取一次，供本面板与机器人控制入口共用。
  let catalog = null;
  function load() {
    if (!catalog) catalog = invoke('list_capabilities').then(value => value.capabilities)
      .catch(error => { catalog = null; throw error; });
    return catalog;
  }
  window.nuwaxCapabilities = {load, invoke};

  const el = {
    view: $('capabilities-view'), search: $('capabilitySearch'), nav: $('capabilityNav'), title: $('capabilityTitle'),
    meta: $('capabilityMeta'), description: $('capabilityDescription'), form: $('capabilityForm'),
    submit: $('capabilitySubmit'), cancel: $('capabilityCancel'), status: $('capabilityStatus'), result: $('capabilityResult'),
  };
  if (!el.view) return;
  const state = {caps: [], selected: null, fields: [], taskId: null, busy: false};

  function say(text, level = '') { el.status.textContent = text; el.status.dataset.level = level; }
  function showResult(value) {
    const text = typeof value === 'string' ? value : JSON.stringify(value, null, 2);
    el.result.textContent = text.length > RESULT_LIMIT
      ? text.slice(0, RESULT_LIMIT) + '\n…（内容过长已截断，共 ' + text.length + ' 字符）' : text;
    el.result.hidden = false;
  }

  // 参数结构：解析本地引用，可空字段取非空分支。
  function resolve(schema, root) {
    let value = schema || {};
    if (value.$ref && value.$ref.startsWith('#/$defs/')) value = {...root.$defs?.[value.$ref.slice(8)], ...value, $ref: undefined};
    const options = value.anyOf || value.oneOf;
    if (options && !value.type) {
      const concrete = options.filter(item => item.type !== 'null');
      if (concrete.length === 1) value = {...resolve(concrete[0], root), description: value.description, default: value.default};
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
    for (const [key, label] of [['minimum', '最小 '], ['exclusiveMinimum', '大于 '], ['maximum', '最大 '], ['exclusiveMaximum', '小于 ']]) {
      if (schema[key] !== undefined) parts.push(label + schema[key]);
    }
    if (schema.default !== undefined) parts.push('默认 ' + JSON.stringify(schema.default));
    return parts.join(' · ');
  }
  function option(value, label) {
    const item = document.createElement('option');
    item.value = value;
    item.textContent = label;
    return item;
  }
  function buildControl(name, schema, required) {
    const kind = controlKind(schema);
    let control;
    if (kind === 'enum' || kind === 'boolean') {
      control = document.createElement('select');
      if (!required) control.append(option('', schema.default === undefined ? '（不填）' : '（不填，默认 ' + JSON.stringify(schema.default) + '）'));
      const values = kind === 'enum' ? schema.enum : [true, false];
      for (const value of values) control.append(option(JSON.stringify(value), kind === 'boolean' ? (value ? '是（true）' : '否（false）') : String(value)));
      if (required && schema.default !== undefined) control.value = JSON.stringify(schema.default);
    } else if (kind === 'json') {
      control = document.createElement('textarea');
      control.rows = 3;
      control.spellcheck = false;
      control.placeholder = schema.default !== undefined ? JSON.stringify(schema.default)
        : schema.type === 'array' ? '填写 JSON 数组，例如 []' : '填写 JSON 对象，例如 {}';
    } else {
      control = document.createElement('input');
      control.type = kind === 'string' ? 'text' : 'number';
      if (kind !== 'string') {
        control.step = kind === 'integer' ? '1' : 'any';
        if (schema.minimum !== undefined) control.min = schema.minimum;
        if (schema.maximum !== undefined) control.max = schema.maximum;
      }
      if (schema.default !== undefined) control.placeholder = '默认 ' + schema.default;
      control.autocomplete = 'off';
    }
    control.className = 'field';
    control.id = 'capability-field-' + name;
    return {control, kind};
  }
  function renderForm(cap) {
    const root = cap.input_schema || {};
    const required = new Set(root.required || []);
    const names = Object.keys(root.properties || {}).sort((a, b) => Number(required.has(b)) - Number(required.has(a)));
    state.fields = [];
    const fragment = document.createDocumentFragment();
    if (!names.length) {
      const empty = document.createElement('p');
      empty.className = 'muted small';
      empty.textContent = '此能力不需要参数。';
      fragment.append(empty);
    }
    for (const name of names) {
      const schema = resolve(root.properties[name], root);
      const {control, kind} = buildControl(name, schema, required.has(name));
      const row = document.createElement('div');
      row.className = 'cap-field';
      const label = document.createElement('label');
      label.htmlFor = control.id;
      const key = document.createElement('span');
      key.className = 'mono';
      key.textContent = name;
      const type = document.createElement('span');
      type.className = 'cap-type';
      type.textContent = (required.has(name) ? '必填 · ' : '可选 · ') + (Array.isArray(schema.enum) ? '枚举' : schema.type || 'JSON');
      label.append(key, type);
      row.append(label, control);
      const hint = hintText(schema);
      if (hint) {
        const note = document.createElement('div');
        note.className = 'cap-hint';
        note.textContent = hint;
        row.append(note);
      }
      if (kind === 'json') {
        const detail = document.createElement('details');
        detail.className = 'cap-schema';
        const summary = document.createElement('summary');
        summary.textContent = '参数结构';
        const pre = document.createElement('pre');
        pre.textContent = JSON.stringify(root.properties[name], null, 2);
        detail.append(summary, pre);
        row.append(detail);
      }
      fragment.append(row);
      state.fields.push({name, kind, control});
    }
    el.form.replaceChildren(fragment);
  }
  // 留空的字段不发送，由能力按自身默认值处理；必填与取值范围由能力统一检查。
  function collect() {
    const args = {};
    for (const {name, kind, control} of state.fields) {
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

  function select(name) {
    const cap = state.caps.find(item => item.name === name);
    if (!cap) return;
    state.selected = cap;
    state.taskId = null;
    for (const button of el.nav.querySelectorAll('button[data-capability]')) button.classList.toggle('on', button.dataset.capability === name);
    el.title.textContent = cap.name;
    el.meta.replaceChildren();
    const kind = document.createElement('span');
    kind.className = 'pill info';
    kind.textContent = KIND_LABELS[cap.kind] || cap.kind;
    el.meta.append(kind);
    if (cap.group) {
      const group = document.createElement('span');
      group.className = 'pill idle';
      group.textContent = cap.group;
      el.meta.append(group);
    }
    if (cap.motion) {
      const motion = document.createElement('span');
      motion.className = 'pill warn';
      motion.textContent = '机器人会运动';
      el.meta.append(motion);
    }
    if (!cap.agent_tool) {
      const tool = document.createElement('span');
      tool.className = 'pill idle';
      tool.textContent = '仅网页与接口';
      el.meta.append(tool);
    }
    el.description.textContent = cap.description;
    renderForm(cap);
    el.submit.textContent = SUBMIT_LABELS[cap.kind] || '执行';
    el.submit.hidden = false;
    el.submit.disabled = false;
    el.cancel.hidden = true;
    el.result.hidden = true;
    say('');
  }
  function renderNav() {
    const query = el.search.value.trim().toLowerCase();
    const groups = new Map();
    for (const cap of state.caps) {
      if (query && ![cap.name, cap.description, cap.group].some(text => String(text || '').toLowerCase().includes(query))) continue;
      const group = cap.group || '其它';
      if (!groups.has(group)) groups.set(group, []);
      groups.get(group).push(cap);
    }
    const fragment = document.createDocumentFragment();
    for (const [group, caps] of groups) {
      const head = document.createElement('div');
      head.className = 'hd eyebrow';
      head.textContent = group + ' · ' + caps.length;
      fragment.append(head);
      for (const cap of caps) {
        const button = document.createElement('button');
        button.type = 'button';
        button.dataset.capability = cap.name;
        button.className = cap.name === state.selected?.name ? 'on' : '';
        const name = document.createElement('span');
        name.className = 'mono';
        name.textContent = cap.name;
        const summary = document.createElement('span');
        summary.className = 'cap-summary';
        summary.textContent = cap.description.split('\n')[0];
        button.append(name, summary);
        button.title = cap.description;
        fragment.append(button);
      }
    }
    if (!groups.size) {
      const empty = document.createElement('p');
      empty.className = 'muted small cap-empty';
      empty.textContent = '没有匹配的能力。';
      fragment.append(empty);
    }
    el.nav.replaceChildren(fragment);
  }
  function renderTask(task) {
    const progress = Number(task.progress);
    const percent = Number.isFinite(progress) ? ' · 进度 ' + Math.round(progress * 100) + '%' : '';
    const failed = ['failed', 'timed_out'].includes(task.state);
    say('任务 ' + task.task_id + '：' + (TASK_STATES[task.state] || task.state) + percent + (task.message ? ' · ' + task.message : ''),
      failed ? 'error' : '');
    if (terminal.includes(task.state)) {
      el.cancel.hidden = true;
      showResult(task.result ?? {state: task.state, message: task.message, error_code: task.error_code});
    } else {
      el.cancel.hidden = false;
      el.cancel.disabled = task.state === 'cancelling';
    }
  }
  window.addEventListener('stream:tasks.state', event => {
    if (state.taskId && event.detail?.task_id === state.taskId) renderTask(event.detail);
  });

  async function execute() {
    const cap = state.selected;
    if (!cap || state.busy) return;
    let args;
    try { args = collect(); } catch (error) { say(error.message, 'error'); return; }
    // 会让机器人运动或删除数据的能力先确认。
    const risk = cap.motion ? '机器人会运动。' : cap.name.startsWith('delete_') ? '会删除数据，无法恢复。' : '';
    if (risk && !window.confirm('执行「' + cap.name + '」？' + risk)) return;
    state.busy = true;
    el.submit.disabled = true;
    el.result.hidden = true;
    say('已提交，等待网关应答。');
    const started = performance.now();
    try {
      const reply = await invoke(cap.name, args);
      if (cap !== state.selected) return;
      if (cap.kind === 'task' && reply.task) {
        state.taskId = reply.task.task_id;
        renderTask(reply.task);
      } else {
        say('完成，用时 ' + Math.round(performance.now() - started) + ' 毫秒。');
        showResult(reply);
      }
    } catch (error) {
      if (cap === state.selected) say('失败：' + error.message + (error.code ? '（' + error.code + '）' : ''), 'error');
    } finally {
      state.busy = false;
      el.submit.disabled = false;
    }
  }
  el.form.addEventListener('submit', event => { event.preventDefault(); execute(); });
  el.submit.addEventListener('click', execute);
  el.cancel.addEventListener('click', async () => {
    const taskId = state.taskId;
    el.cancel.disabled = true;
    try { await invoke('cancel_task', {task_id: taskId}); }
    catch (error) { say('取消失败：' + error.message, 'error'); el.cancel.disabled = false; }
  });
  el.search.addEventListener('input', renderNav);
  el.nav.addEventListener('click', event => {
    const button = event.target.closest('button[data-capability]');
    if (button) select(button.dataset.capability);
  });

  window.nuwaxCapabilityPanel = {
    open() {
      if (state.caps.length) return;
      say('正在读取能力清单。');
      load().then(caps => { state.caps = caps; renderNav(); say('共 ' + caps.length + ' 个能力，选择左侧能力填写参数后执行。'); })
        .catch(error => say('读取能力清单失败：' + error.message, 'error'));
    },
    select(name) { select(name); },
  };
})();
