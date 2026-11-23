/* 共用设置编辑器：控制台“设置”页与管理服务恢复页“设置”挂载同一模块。
   数据经设备管理程序 /api/manager/config*（06 第 3 节：管理程序是唯一写入方，校验只在 ConfigStore 做一次）。
   分组、次序、开关位置与名称全部来自配置模式（panel / category / switch / advanced / title / unit / summary / apply）。
   每项一行属性表：名称与键名 | 控件 | 说明。板块与子类标题栏的开关切换即单独保存；其余项进入待保存列表，由底部保存条统一提交。
   宿主传入 request(path, {method, body, timeout})（控制台为 ctx.api.manager，写请求自动带 X-NEH-CSRF），失败时抛出带原因的错误。 */
import { icon, h, toast, confirmDialog, menu, block, sub } from './neh-ui.js';

const SINGLE_COLOR_KEYS = new Set(['gateway.pointcloud3d_color', 'gateway.pointcloud3d_ground_color',
  'gateway.pointcloud3d_structure_color', 'gateway.pointcloud3d_object_color']);
const HEIGHT_COLORS_KEY = 'gateway.pointcloud3d_height_colors';
const FINE_CLOUD_CACHE_KEY = 'geometry.fine_cloud_cache_mib';
const FINE_CLOUD_CACHE_MAX_KEY = 'geometry.fine_cloud_cache_auto_max_mib';
const PIPELINE_KEY = 'geometry.pipeline';
// 二维执行链（slam_2d）不使用的三维设置：执行链选二维时这些行置灰（与原设置页相同）。
const SPATIAL_KEYS = new Set(['geometry.lio_backend', 'geometry.mapping_backend', 'geometry.correction_backend', 'geometry.pgo_options',
  FINE_CLOUD_CACHE_KEY, FINE_CLOUD_CACHE_MAX_KEY, 'navigation.surface_voxel_size_m', 'navigation.min_stair_tread_m',
  'navigation.traversability_max_step_height_m', 'navigation.traversability_max_slope_deg']);
const HEX_COLOR = /^#[0-9a-fA-F]{6}$/;
const TIMEOUT = 60000;

/* 复制文字：设备多经局域网 http 地址打开，不是安全来源时浏览器不提供剪贴板接口，退回原生复制命令（沿用原日志页做法）。 */
export async function copyText(text) {
  try {
    if (navigator.clipboard?.writeText) { await navigator.clipboard.writeText(text); return true; }
  } catch (e) { /* 权限被拒时继续尝试原生复制 */ }
  const focused = document.activeElement;
  const area = h('textarea', { readonly: true, 'aria-hidden': 'true', style: 'position:fixed;left:-9999px' });
  area.value = text;
  document.body.append(area);
  let copied = false;
  try { area.select(); copied = document.execCommand('copy') === true; } catch (e) { copied = false; }
  area.remove();
  focused?.focus?.({ preventScroll: true });
  return copied;
}

const titleOf = f => f.title || f.zh_name || f.key;
const numeric = f => f.type === 'int' || f.type === 'float';
const num = v => typeof v === 'number' && !Number.isInteger(v) ? String(+v.toFixed(3)) : String(v);
/* 比较用的规范文字：列表按逗号拼接、布尔为 true/false、数值去掉多余写法；特殊值（如 auto、留空）原样。 */
function norm(f, v) {
  if (v == null) return '';
  if (Array.isArray(v)) return v.join(',');
  if (f.type === 'bool') return String(v === true || v === 'true');
  if (f.type === 'csv') return String(v).split(',').map(s => s.trim()).filter(Boolean).join(',');
  if (numeric(f) && String(v).trim() !== '' && Number.isFinite(Number(v))) return String(Number(v));
  return String(v);
}
/* 输入框里的文字：设置读取已把列表拼成逗号文字。 */
const inputText = v => v == null ? '' : Array.isArray(v) ? v.join(',') : String(v);
/* 提交时的类型：与原设置页相同，布尔、逗号列表、数值（留空与特殊值原样交给模式校验）。 */
function typed(f, raw) {
  if (f.type === 'bool') return raw === true || raw === 'true';
  if (f.type === 'csv') return String(raw).split(',').map(s => s.trim()).filter(Boolean);
  if (numeric(f) && raw !== '' && !(f.special_values || []).includes(raw)) return Number(raw);
  return raw;
}
function shown(f, v) {
  if (f.type === 'bool') return norm(f, v) === 'true' ? '开' : '关';
  const text = Array.isArray(v) ? v.join(', ') : inputText(v);
  return text === '' ? '留空' : text === 'auto' ? '自动' : f.unit && numeric(f) ? text + ' ' + f.unit : text;
}
function rangeText(f) {
  const lo = f.min != null ? (f.min_exclusive ? '大于 ' : '不小于 ') + num(f.min) : '';
  const hi = f.max != null ? (f.max_exclusive ? '小于 ' : '不大于 ') + num(f.max) : '';
  return f.min != null && f.max != null && !f.min_exclusive && !f.max_exclusive ? '范围 ' + num(f.min) + '～' + num(f.max) : [lo, hi].filter(Boolean).join('，');
}

export function mount(root, opts) {
  const request = opts.request;
  const pending = new Map();            // 键名 → 未保存的输入文字（布尔为 'true'/'false'）
  const state = { schema: null, sections: [], byKey: new Map(), profile: null, query: '', filter: 'all', advanced: false,
    open: new Set(), closedSubs: new Set(), saving: false, saveError: '', loadSeq: 0, loaded: false };

  const main = h('div', { class: 'n-settings-main' });
  const bar = h('div', { class: 'n-toolbar' });
  const body = h('div', { class: 'n-settings-body' });
  const savebar = h('div', { class: 'n-savebar', hidden: true, role: 'region', 'aria-label': '未保存的改动' });
  main.append(bar, body, savebar);
  root.replaceChildren(h('div', { class: 'n-settings' }, main));
  // 展开的板块标题吸顶在工具栏下方，工具栏换行时跟着调整。
  new ResizeObserver(() => main.style.setProperty('--bar-h', bar.offsetHeight + 'px')).observe(bar);

  /* ---------- 工具栏：标题、方案、搜索、筛选、找到数量、显示高级项、全部展开 ---------- */
  const profileBtn = h('button', { class: 'n-btn n-btn--sm', type: 'button', 'aria-haspopup': 'menu', hidden: true });
  const search = h('input', { class: 'n-input', type: 'search', placeholder: '搜索名称、键名或说明', 'aria-label': '搜索设置' });
  const resultNote = h('span', { class: 'n-meta' });
  const filterSeg = h('div', { class: 'n-seg', role: 'group', 'aria-label': '筛选' });
  const advInput = h('input', { type: 'checkbox' });
  const advSwitch = h('label', { class: 'n-switch n-meta' }, advInput, h('span', { class: 'n-switch-track' }), '显示高级项');
  const toggleAll = h('button', { class: 'n-btn n-btn--quiet n-btn--sm', type: 'button', onclick: () => {
    if (state.open.size) { state.open.clear(); render(); main.scrollTo({ top: 0 }); }
    else { state.sections.forEach(s => state.open.add(s.section_id)); state.closedSubs.clear(); render(); }
  } }, '全部展开');
  bar.append(h('h1', {}, opts.title || '设置'), profileBtn,
    h('div', { class: 'n-search', html: icon('search', 'n-icon--sm') }, search),
    filterSeg, resultNote, h('span', { class: 'n-toolbar-spacer' }), advSwitch, h('span', { class: 'n-toolbar-sep' }), toggleAll);
  let searchTimer = null;
  search.addEventListener('input', () => { clearTimeout(searchTimer); searchTimer = setTimeout(() => { state.query = search.value.trim().toLowerCase(); render(); }, 120); });
  advInput.addEventListener('change', () => { state.advanced = advInput.checked; render(); });

  /* ---------- 方案：切换编辑对象、新建、复制、设为运行方案、删除 ---------- */
  function renderProfile() {
    const c = state.schema?.profile_context;
    profileBtn.hidden = !c;
    if (!c) return;
    const running = c.profile === c.selected_profile;
    const path = state.sections[0]?.fields[0]?.target_path;
    profileBtn.title = '正在编辑的方案' + (path ? '，保存位置：' + path : '');
    profileBtn.replaceChildren(h('span', { class: 'n-meta' }, '方案'), h('span', { style: 'font-weight:600' }, c.profile),
      running ? h('span', { class: 'n-badge n-badge--accent' }, '运行中') : h('span', { class: 'n-badge' }, '未运行'));
    profileBtn.insertAdjacentHTML('beforeend', icon('chevron-down', 'n-icon--sm'));
  }
  // 切换编辑对象与新建、复制、删除前先处理未保存的改动，避免改动落到别的方案或被丢掉。
  function idle() {
    if (!pending.size && !state.saving) return true;
    toast(state.saving ? '正在保存，请稍候再操作方案' : '先保存或放弃当前改动，再操作方案', { tone: 'warn' });
    return false;
  }
  profileBtn.addEventListener('click', () => {
    const c = state.schema.profile_context;
    const items = c.profiles.map(name => ({ label: name + (name === c.selected_profile ? '（运行中）' : ''), icon: name === c.profile ? 'check' : 'blank',
      run: () => { if (name !== c.profile && idle()) { state.profile = name; void load().then(ok => ok && toast('正在编辑方案「' + name + '」', { tone: 'info' })); } } }));
    items.push('-',
      { label: '新建方案…', icon: 'plus', run: () => idle() && askName('新建方案', '新方案从默认值开始。', '创建', name => profileOp('POST', 'config/profiles', { name }, '已新建方案「' + name + '」', name)) },
      { label: '复制当前方案…', icon: 'copy', run: () => idle() && askName('复制方案', '复制「' + c.profile + '」已保存的设置。', '复制', name => profileOp('POST', 'config/profiles', { name, source: c.profile }, '已复制为「' + name + '」', name)) });
    if (c.profile !== c.selected_profile) items.push({ label: '设为运行方案…', icon: 'play', run: async () => {
      if (!idle()) return;
      if (await confirmDialog({ title: '把「' + c.profile + '」设为运行方案', text: '业务服务会停止并按这个方案重新启动，正在执行的任务会被中断，网页连接会短暂断开。', confirmText: '切换并重启业务' }))
        profileOp('POST', 'config/activate', { name: c.profile }, '已切换运行方案，业务按「' + c.profile + '」重新启动', c.profile, 180000);
    } });
    items.push('-', { label: '删除方案…', icon: 'trash-2', danger: true, run: async () => {
      if (c.profile === c.selected_profile) { toast('不能删除运行中的方案，请先把其它方案设为运行方案', { tone: 'warn' }); return; }
      if (!idle()) return;
      if (await confirmDialog({ title: '删除方案「' + c.profile + '」', text: '删除后无法恢复。', confirmText: '删除方案', danger: true }))
        profileOp('DELETE', 'config/profiles', { name: c.profile }, '已删除方案「' + c.profile + '」', null);
    } });
    menu(profileBtn, items);
  });
  async function askName(title, text, confirmText, done) {
    const input = h('input', { class: 'n-input', maxlength: 64, placeholder: '例如 室外巡检' });
    setTimeout(() => input.focus(), 50);
    if (await confirmDialog({ title, text, extra: h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '方案名称'), input), confirmText }) && input.value.trim()) done(input.value.trim());
  }
  async function profileOp(method, path, payload, message, next, timeout = TIMEOUT) {
    try {
      await request(path, { method, body: payload, timeout });
      state.profile = next;
      await load();
      toast(message);
    } catch (error) { toast('方案操作没有完成：' + error.message, { tone: 'bad', duration: 8000 }); }
  }

  /* ---------- 读取 ---------- */
  async function load() {
    const seq = ++state.loadSeq;
    if (!state.loaded) body.replaceChildren(h('div', { class: 'n-empty' }, h('span', { class: 'n-spin' }), h('div', {}, '正在读取设置…')));
    try {
      const schema = await request('config' + (state.profile ? '?profile=' + encodeURIComponent(state.profile) : ''), { timeout: TIMEOUT });
      if (seq !== state.loadSeq) return false;
      if (schema.available === false) {
        state.schema = null; renderProfile();
        body.replaceChildren(h('div', { class: 'n-empty', html: icon('settings') }, h('div', { class: 'n-empty-title' }, '还不能编辑设置'), h('div', {}, schema.message || '')));
        return false;
      }
      state.schema = schema;
      state.sections = schema.sections || [];
      state.profile = schema.profile_context?.profile || null;
      state.byKey = new Map(state.sections.flatMap(s => s.fields.map(f => [f.key, f])));
      // 读取到的新值与未保存输入相同时，这项改动已不存在。
      for (const [key, raw] of pending) {
        const f = state.byKey.get(key);
        if (!f || (!f.sensitive && norm(f, raw) === norm(f, f.value))) pending.delete(key);
      }
      state.loaded = true;
      renderProfile(); render(); syncSavebar();
      return true;
    } catch (error) {
      if (seq !== state.loadSeq) return false;
      // 读取失败时编辑对象回到页面上显示的方案，之后的保存不会写到别的方案。
      if (state.schema) state.profile = state.schema.profile_context?.profile || null;
      if (!state.loaded) body.replaceChildren(h('div', { class: 'n-empty', html: icon('circle-alert') }, h('div', { class: 'n-empty-title' }, '读取设置失败'),
        h('div', {}, error.message), h('button', { class: 'n-btn n-btn--sm', type: 'button', style: 'margin-top:8px', onclick: () => load() }, '重新读取')));
      else toast('读取设置失败：' + error.message + '，页面保留上次读取的内容', { tone: 'bad' });
      return false;
    }
  }

  /* ---------- 行 ---------- */
  function current(f) { return pending.has(f.key) ? pending.get(f.key) : f.type === 'bool' ? norm(f, f.value) : f.sensitive ? '' : inputText(f.value); }
  // 敏感项读取时不回显，无法和默认值比较，不计入“已改默认值”。
  function isCustom(f) { return !f.sensitive && norm(f, f.value) !== norm(f, f.default_value); }
  function matches(f) {
    if (state.filter === 'custom' && !isCustom(f)) return false;
    if (state.filter === 'pending' && !pending.has(f.key)) return false;
    if (!state.query) return true;
    return [titleOf(f), f.key, f.description, f.panel, f.category, f.switch,
      ...Object.entries(f.options_schema || {}).map(([k, spec]) => k + ' ' + (spec.description || ''))].join(' ').toLowerCase().includes(state.query);
  }
  function set(f, raw) {
    if (f.sensitive ? raw === '' : norm(f, raw) === norm(f, f.value)) pending.delete(f.key);
    else pending.set(f.key, raw);
    state.saveError = '';
    syncRow(f); syncSavebar(); renderFilters();
    if (f.key === PIPELINE_KEY) syncPipeline();
    if (f.key === FINE_CLOUD_CACHE_KEY) syncFineCloud();
  }
  function colorPicker(value, label, onPick) {
    const picker = h('input', { type: 'color', class: 'n-set-color', 'aria-label': label, title: label });
    picker.value = HEX_COLOR.test(String(value).trim()) ? String(value).trim().toLowerCase() : '#000000';
    picker.addEventListener('input', () => onPick(picker.value.toLowerCase()));
    return picker;
  }
  function control(f) {
    const v = current(f);
    if (f.readonly) return h('div', { class: 'n-set-control' }, h('input', { class: 'n-input', value: inputText(f.value), disabled: true }),
      h('span', { class: 'n-set-default n-set-readonly' }, f.key === 'gateway.port' ? '业务端口在“系统 › 管理选项”中修改' : '只读'));
    if (f.type === 'bool') {
      const input = h('input', { type: 'checkbox', checked: v === 'true' });
      const lab = h('span', { class: 'n-meta' }, v === 'true' ? '开' : '关');
      input.addEventListener('change', () => { lab.textContent = input.checked ? '开' : '关'; set(f, String(input.checked)); });
      return h('div', { class: 'n-set-control' }, h('label', { class: 'n-switch', 'aria-label': titleOf(f) }, input, h('span', { class: 'n-switch-track' }), lab));
    }
    if (f.choices) {
      const choices = f.choices.includes(v) || v === '' ? f.choices : [...f.choices, v];
      const sel = h('select', { class: 'n-select', 'aria-label': titleOf(f) }, choices.map(c => h('option', { value: c, selected: v === c },
        c === '' ? '留空（不指定）' : f.choices.includes(c) ? c : c + '（当前）')));
      sel.addEventListener('change', () => set(f, sel.value));
      return h('div', { class: 'n-set-control' }, sel);
    }
    if (f.key === FINE_CLOUD_CACHE_KEY) return fineCloudControl(f, v);
    if (f.options_schema || f.type === 'json' || f.key === 'geometry.slam2d_config') {
      const ta = h('textarea', { class: 'n-textarea n-input--code', rows: 2, spellcheck: 'false', 'aria-label': titleOf(f) });
      ta.value = v;
      ta.addEventListener('input', () => set(f, ta.value));
      return h('div', { class: 'n-set-control' }, ta);
    }
    const input = h('input', { class: 'n-input' + (numeric(f) ? ' n-num' : ''), type: f.sensitive ? 'password' : 'text', inputmode: numeric(f) ? 'decimal' : null, autocomplete: 'off',
      'aria-label': titleOf(f),
      placeholder: f.sensitive ? (f.is_set ? '已设置，留空保持不变' : '未设置') : f.key === FINE_CLOUD_CACHE_MAX_KEY ? '32～4096；留空使用 256'
        : f.type === 'csv' ? '多个值用逗号分隔' : (f.special_values || []).includes('') ? '留空使用自动值' : '' });
    input.value = v;
    const pickers = [];
    input.addEventListener('input', () => {
      if (SINGLE_COLOR_KEYS.has(f.key) && HEX_COLOR.test(input.value.trim())) pickers[0].value = input.value.trim().toLowerCase();
      const colors = input.value.split(',').map(s => s.trim());
      if (f.key === HEIGHT_COLORS_KEY && colors.length === 5 && colors.every(c => HEX_COLOR.test(c))) pickers.forEach((p, i) => { p.value = colors[i].toLowerCase(); });
      set(f, input.value);
    });
    const field = f.unit ? h('div', { class: 'n-input-group' }, input, h('span', { class: 'n-suffix' }, f.unit)) : input;
    if (SINGLE_COLOR_KEYS.has(f.key)) {
      pickers.push(colorPicker(v, titleOf(f) + '色盘', color => { input.value = color; set(f, color); }));
      return h('div', { class: 'n-set-control' }, h('div', { class: 'n-set-colorrow' }, field, pickers[0]));
    }
    if (f.key === HEIGHT_COLORS_KEY) {
      const colors = v.split(',').map(s => s.trim());
      for (let i = 0; i < 5; i += 1) pickers.push(colorPicker(colors[i], '高度颜色第' + (i + 1) + '档', () => {
        input.value = pickers.map(p => p.value.toLowerCase()).join(',');
        set(f, input.value);
      }));
      return h('div', { class: 'n-set-control' }, field, h('div', { class: 'n-set-colorrow' }, pickers));
    }
    return h('div', { class: 'n-set-control' }, field);
  }
  // 精细点云缓存：自动或固定预算（MiB）两种模式，固定时才填数值；自动时显示自动上限一项。
  function fineCloudControl(f, v) {
    const fixed = Boolean(v) && v !== 'auto';
    const mode = h('select', { class: 'n-select', 'aria-label': '精细点云缓存模式' }, h('option', { value: 'auto', selected: !fixed }, '自动'), h('option', { value: 'fixed', selected: fixed }, '固定'));
    const amount = h('input', { class: 'n-input n-num', inputmode: 'numeric', placeholder: '16～4096；留空恢复自动', 'aria-label': '精细点云固定缓存预算（MiB）' });
    amount.value = fixed ? v : '32';
    const group = h('div', { class: 'n-input-group', hidden: !fixed }, amount, h('span', { class: 'n-suffix' }, 'MiB'));
    mode.addEventListener('change', () => { group.hidden = mode.value !== 'fixed'; set(f, mode.value === 'auto' ? 'auto' : amount.value); });
    amount.addEventListener('input', () => set(f, amount.value));
    return h('div', { class: 'n-set-control' }, mode, group);
  }
  // 结构化参数（位姿图、Point-LIO 选项）：原始 JSON 之外按参数逐项填写，两边同步。
  function optionsEditor(f, row) {
    const ta = row.querySelector('textarea');
    const parse = () => { try { const value = JSON.parse(ta.value || '{}'); return value && typeof value === 'object' && !Array.isArray(value) ? value : null; } catch (e) { return null; } };
    const groups = new Map();
    const controls = new Map();
    for (const [name, spec] of Object.entries(f.options_schema)) {
      const group = f.key === 'geometry.pgo_options'
        ? (name.startsWith('scan_context_') ? '扫描上下文：地点检索' : /^(odometry_|gravity_|loop_.*sigma_|optimizer_)/.test(name) ? '位姿图：约束权重与优化' : '回环：配准、确认与关键帧')
        : '参数明细';
      if (!groups.has(group)) groups.set(group, h('div', { class: 'n-set-options-grid' }));
      let input;
      if (spec.type === 'bool' || spec.type === 'enum') {
        input = h('select', { class: 'n-select' }, h('option', { value: '' }, '使用算法默认'),
          (spec.type === 'bool' ? [['true', '是'], ['false', '否']] : (spec.choices || []).map(c => [c, c])).map(([value, label]) => h('option', { value }, label)));
      } else input = h('input', { class: 'n-input' + (['int', 'number'].includes(spec.type) ? ' n-num' : ''), placeholder: ['int', 'number'].includes(spec.type) ? '使用算法默认' : '留空使用默认；数组按结构化格式填写' });
      input.addEventListener('change', () => {
        const value = parse();
        if (!value) { toast('先修正原始配置：它必须是 JSON 对象', { tone: 'warn' }); return; }
        const raw = input.value.trim();
        if (!raw) delete value[name];
        else if (spec.type === 'bool') value[name] = raw === 'true';
        else if (spec.type === 'enum') value[name] = raw;
        else if (['int', 'number'].includes(spec.type)) {
          const parsed = Number(raw);
          if (!Number.isFinite(parsed) || (spec.type === 'int' && !Number.isInteger(parsed))) return;
          value[name] = parsed;
        } else {
          try { value[name] = JSON.parse(raw); } catch (e) { toast(name + ' 的数组格式无效，输入已保留', { tone: 'warn' }); return; }
        }
        ta.value = JSON.stringify(value);
        set(f, ta.value);
      });
      const range = [spec.minimum != null ? '最小 ' + spec.minimum : '', spec.maximum != null ? '最大 ' + spec.maximum : ''].filter(Boolean).join('，');
      groups.get(group).append(h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, spec.description || name),
        input, h('span', { class: 'n-help' }, h('span', { class: 'n-code' }, name), range ? '　' + range : '')));
      controls.set(name, input);
    }
    const sync = () => {
      const value = parse();
      if (!value) return;
      for (const [name, input] of controls) input.value = value[name] == null ? '' : typeof value[name] === 'object' ? JSON.stringify(value[name]) : String(value[name]);
    };
    ta.addEventListener('input', sync);
    sync();
    return h('details', { class: 'n-set-options', open: Boolean(state.query) || null },
      h('summary', { html: icon('chevron-right') + '逐项填写（' + controls.size + ' 项，留空使用算法默认）' }),
      [...groups].map(([title, grid]) => h('div', { class: 'n-set-options-group' }, groups.size > 1 ? h('div', { class: 'n-label' }, title) : null, grid)));
  }
  function row(f) {
    const range = rangeText(f);
    const r = h('div', { class: 'n-prop', dataset: { key: f.key } },
      h('div', { class: 'n-prop-name' },
        h('b', { title: titleOf(f) }, titleOf(f), f.apply === 'hot' ? h('span', { class: 'n-badge n-badge--accent n-tip', style: 'margin-left:6px', 'data-tip': '保存后网关立即采用，不用重启' }, '即时生效') : null),
        h('button', { class: 'n-set-key n-tip', type: 'button', 'data-tip': '点击复制键名', onclick: async () => {
          toast(await copyText(f.key) ? '已复制 ' + f.key : '复制失败，请手动选中键名', { tone: 'info' });
        } }, f.key)),
      control(f),
      h('div', { class: 'n-prop-desc' }, f.description, range ? h('span', { class: 'n-faint' }, '　' + range) : null));
    if (f.options_schema) r.append(optionsEditor(f, r));
    return r;
  }
  function syncRow(f) {
    const r = body.querySelector('.n-prop[data-key="' + CSS.escape(f.key) + '"]');
    if (!r) return;
    r.dataset.modified = String(pending.has(f.key));
    const ctl = r.querySelector('.n-set-control');
    ctl.querySelector('.n-set-default:not(.n-set-readonly)')?.remove();
    if (f.readonly || f.sensitive || norm(f, current(f)) === norm(f, f.default_value)) return;
    const dv = shown(f, f.default_value);
    ctl.append(h('div', { class: 'n-set-default', title: '默认 ' + dv }, h('span', { class: 'n-set-default-val' }, '默认 ' + dv),
      h('button', { class: 'n-set-reset', type: 'button', onclick: () => {
        const raw = f.type === 'bool' ? norm(f, f.default_value) : inputText(f.default_value);
        if (norm(f, raw) === norm(f, f.value)) pending.delete(f.key); else pending.set(f.key, raw);
        render(); syncSavebar();
      } }, '恢复默认')));
  }
  function syncPipeline() {
    const f = state.byKey.get(PIPELINE_KEY);
    const planar = Boolean(f) && current(f) === 'slam_2d';
    for (const r of body.querySelectorAll('.n-prop[data-key]')) {
      if (!SPATIAL_KEYS.has(r.dataset.key)) continue;
      for (const input of r.querySelectorAll('input,select,textarea,button.n-set-reset')) input.disabled = planar;
      r.style.opacity = planar ? '0.5' : '';
      r.title = planar ? '该设置仅用于三维执行链' : '';
    }
  }
  function syncFineCloud() {
    const f = state.byKey.get(FINE_CLOUD_CACHE_KEY);
    const budget = f ? String(current(f)).trim() : 'auto';
    const r = body.querySelector('.n-prop[data-key="' + FINE_CLOUD_CACHE_MAX_KEY + '"]');
    if (r) r.hidden = Boolean(budget) && budget !== 'auto';
  }

  /* ---------- 开关：切换即单独保存这一项；失败时恢复原状并就地写出原因 ---------- */
  function switchControl(f) {
    const input = h('input', { type: 'checkbox', role: 'switch', checked: norm(f, f.value) === 'true', disabled: Boolean(f.readonly) });
    const note = h('span', { class: 'n-set-switch-err', hidden: true });
    input.addEventListener('change', async () => {
      const on = input.checked;
      input.disabled = true;
      note.hidden = true;
      try {
        const result = await request('config', { method: 'PUT', body: { values: { [f.key]: on }, profile: state.profile }, timeout: TIMEOUT });
        f.value = on;
        report(result, (on ? '已开启' : '已关闭') + '「' + f.switch + '」');
        await load();
      } catch (error) {
        input.checked = !on;
        note.textContent = note.title = '保存失败：' + error.message;
        note.hidden = false;
        toast('「' + f.switch + '」没有保存：' + error.message, { tone: 'bad', duration: 8000 });
      } finally { input.disabled = Boolean(f.readonly); }
    });
    return h('span', { class: 'n-set-switch' }, note, h('label', { class: 'n-switch n-tip', 'data-tip': f.apply === 'hot' ? '切换后立即保存并生效' : '切换后立即保存，重启业务后生效' },
      input, h('span', { class: 'n-switch-track' }), h('span', { class: 'n-meta' }, f.switch)));
  }
  /* 保存结果说明：编辑的不是运行方案时只写入方案；热加载失败写出原因；需重启时给出“立即重启”。 */
  function report(result, label) {
    const errors = Object.entries(result.apply_errors || {});
    if (!result.is_active) toast(label + '，已保存到方案「' + state.profile + '」，设为运行方案后生效', { tone: 'info' });
    else if (errors.length) toast(label + '，已保存，但没能立即生效：' + errors.map(([key, message]) => key + '：' + message).join('；'), { tone: 'warn', duration: 8000 });
    else if (result.restart_required) toast(label + '，重启业务后生效', { action: { label: '立即重启', run: restart }, duration: 8000 });
    else if ((result.hot_keys || []).length && !(result.hot_applied || []).length) toast(label + '，已保存，业务启动后生效');
    else toast(label + '，已生效');
  }
  async function restart() {
    try {
      const job = await request('config/restart', { method: 'POST', body: {}, timeout: TIMEOUT });
      if (opts.onRestart) opts.onRestart(job);
      else toast('已安排重启业务服务，完成前网页连接会短暂断开', { tone: 'info' });
    } catch (error) { toast('无法重启业务：' + error.message, { tone: 'bad', duration: 8000 }); }
  }

  /* ---------- 渲染：每个功能板块一个可折叠块，收起时就是一张功能模块清单 ---------- */
  function summaryOf(s) {
    return s.fields.filter(f => f.summary).map(f => {
      const v = current(f);
      const text = f.type === 'csv' ? (v ? v.split(',').filter(Boolean) : []) : null;
      const empty = (f.special_values || []).includes('') ? '自动' : '未设置';
      return titleOf(f) + ' ' + (text ? (text.length > 2 ? text.length + ' 项' : text.join('、') || empty) : f.sensitive ? (f.is_set ? '已设置' : '未设置') : v === '' ? empty : shown(f, v));
    }).join('，');
  }
  function renderFilters() {
    const all = [...state.byKey.values()];
    if (state.filter === 'pending' && !pending.size) state.filter = 'all';
    filterSeg.replaceChildren(...[['all', '全部', all.length], ['custom', '已改默认值', all.filter(isCustom).length]].map(([id, label, n]) =>
      h('button', { type: 'button', 'aria-pressed': String(state.filter === id), onclick: () => { state.filter = id; render(); syncSavebar(); } }, label, h('span', { class: 'n-count' }, n))));
  }
  function render() {
    renderFilters();
    body.replaceChildren();
    let total = 0;
    const filtering = Boolean(state.query) || state.filter !== 'all';
    const showAdv = state.advanced || filtering;
    for (const s of state.sections) {
      const id = s.section_id;
      const master = s.fields.find(f => f.switch && !f.category);
      const changed = s.fields.filter(f => isCustom(f) || pending.has(f.key)).length;
      const subs = [];
      let shownCount = 0;
      for (const c of s.categories) {
        const fields = s.fields.filter(f => f.category === c);
        // 精细点云缓存模式放在自动上限之前。
        fields.sort((a, b) => Number(b.key === FINE_CLOUD_CACHE_KEY) - Number(a.key === FINE_CLOUD_CACHE_KEY));
        const catSwitch = fields.find(f => f.switch);
        const plain = fields.filter(f => f !== catSwitch && matches(f));
        const basic = plain.filter(f => !f.advanced);
        const adv = plain.filter(f => f.advanced);
        if (!basic.length && !(adv.length && showAdv) && !(catSwitch && matches(catSwitch))) continue;
        const key = id + '|' + c;
        const sb = sub({ title: c, count: (basic.length + adv.length) + ' 项', open: filtering || !state.closedSubs.has(key),
          disabled: (catSwitch && norm(catSwitch, catSwitch.value) === 'false') || (master && norm(master, master.value) === 'false'),
          aside: catSwitch ? switchControl(catSwitch) : null,
          onToggle: v => { if (v) state.closedSubs.delete(key); else state.closedSubs.add(key); } });
        basic.forEach(f => sb.body.append(row(f)));
        if (adv.length) {
          if (showAdv) adv.forEach(f => sb.body.append(row(f)));
          else sb.body.append(h('details', { class: 'n-set-adv' }, h('summary', { html: icon('chevron-right') + '高级（' + adv.length + '）' }), adv.map(row)));
        }
        shownCount += basic.length + (showAdv ? adv.length : 0);
        subs.push(sb);
      }
      if (!shownCount && !(master && matches(master))) continue;
      total += shownCount;
      const aside = h('div', { style: 'display:flex;align-items:center;gap:12px' },
        changed ? h('span', { class: 'n-badge n-badge--accent' }, '已改 ' + changed + ' 项') : h('span', { class: 'n-meta' }, s.fields.length + ' 项'),
        master ? switchControl(master) : null);
      const blk = block({ icon: s.icon || 'sliders-horizontal', title: s.section_title, summary: summaryOf(s), aside, id: 'set-' + id,
        open: filtering || state.open.has(id), state: master && norm(master, master.value) === 'false' ? 'off' : 'on',
        onToggle: v => { if (v) state.open.add(id); else state.open.delete(id); toggleAll.textContent = state.open.size ? '全部收起' : '全部展开'; } });
      blk.classList.add('n-set-block');
      blk.dataset.panel = id;
      if (subs.length) blk.body.append(...subs);
      else blk.body.append(h('div', { class: 'n-block-pad n-meta' }, '这个功能只有总开关。'));
      body.append(blk);
    }
    if (!body.children.length) body.append(h('div', { class: 'n-empty', html: icon('search') }, h('div', { class: 'n-empty-title' }, '没有匹配的设置'), h('div', {}, '换个关键词，或清除筛选条件。')));
    state.byKey.forEach(syncRow);
    syncPipeline();
    syncFineCloud();
    resultNote.textContent = filtering ? '找到 ' + total + ' 项' : '';
    toggleAll.textContent = state.open.size ? '全部收起' : '全部展开';
  }

  /* ---------- 保存条 ---------- */
  function syncSavebar() {
    const n = pending.size;
    savebar.hidden = n === 0;
    if (!n) return;
    const c = state.schema?.profile_context;
    const restartCount = [...pending.keys()].filter(k => state.byKey.get(k)?.apply === 'restart').length;
    const note = state.saveError ? h('small', { class: 'n-set-save-err' }, '保存失败：' + state.saveError + '；输入已保留')
      : c && c.profile !== c.selected_profile ? h('small', {}, '保存到方案「' + c.profile + '」，设为运行方案后生效')
        : h('small', {}, restartCount ? '其中 ' + restartCount + ' 项需要重启业务后生效' : '保存后立即生效');
    savebar.replaceChildren(
      h('div', { class: 'n-savebar-text' }, '有 ' + n + ' 项改动未保存', note),
      h('button', { class: 'n-btn n-btn--sm', type: 'button', 'aria-pressed': String(state.filter === 'pending'), onclick: () => { state.filter = state.filter === 'pending' ? 'all' : 'pending'; render(); syncSavebar(); } }, state.filter === 'pending' ? '显示全部' : '只看改动'),
      h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: state.saving, onclick: () => { pending.clear(); state.saveError = ''; render(); syncSavebar(); toast('已放弃改动', { tone: 'info' }); } }, '放弃改动'),
      h('button', { class: 'n-btn n-btn--sm n-btn--primary', type: 'button', disabled: state.saving, onclick: save }, state.saving ? '正在保存…' : '保存'));
  }
  async function save() {
    if (state.saving || !pending.size) return;
    const submitted = new Map(pending);
    const values = {};
    for (const [key, raw] of submitted) values[key] = typed(state.byKey.get(key), raw);
    state.saving = true; state.saveError = ''; syncSavebar();
    try {
      const result = await request('config', { method: 'PUT', body: { values, profile: state.profile }, timeout: TIMEOUT });
      // 只移除本次提交且保存期间没有继续编辑的项；回执即新基线，之后读取失败也不回到旧值。
      for (const [key, raw] of submitted) {
        const f = state.byKey.get(key);
        if (f.sensitive) f.is_set = f.is_set || raw !== '';
        else f.value = f.type === 'bool' ? raw === 'true' : raw;
        if (pending.get(key) === raw) pending.delete(key);
      }
      state.saving = false;
      render(); syncSavebar();
      void load();
      const restartFields = (result.restart_keys || []).map(key => state.byKey.get(key)).filter(Boolean);
      if (result.is_active && result.restart_required && restartFields.length) {
        const ok = await confirmDialog({ title: '已保存 ' + submitted.size + ' 项', icon: 'info',
          text: '以下 ' + restartFields.length + ' 项要在业务重启后才生效。重启会中断正在执行的任务，网页连接会短暂断开。',
          items: restartFields.slice(0, 6).map(titleOf).concat(restartFields.length > 6 ? ['另有 ' + (restartFields.length - 6) + ' 项'] : []), confirmText: '立即重启业务' });
        if (ok) await restart(); else toast('已保存，稍后可在“系统”页重启业务', { tone: 'info' });
      } else report(result, '已保存 ' + submitted.size + ' 项');
    } catch (error) {
      state.saveError = error.message;
      toast('保存失败：' + error.message, { tone: 'bad', duration: 8000 });
    } finally {
      state.saving = false;
      syncSavebar();
    }
  }

  return {
    load,
    hasPending: () => pending.size > 0,
    /* 搜索某个键（其它页面链接到具体设置时用）。 */
    focusKey(key) { search.value = key; state.query = key.toLowerCase(); render(); },
    /* 展开并滚到某个板块（地址 #/settings/<板块名>）。 */
    openPanel(id) {
      if (!state.sections.some(s => s.section_id === id)) return;
      state.open.add(id); render();
      body.querySelector('[data-panel="' + CSS.escape(id) + '"]')?.scrollIntoView({ block: 'start' });
    },
  };
}
