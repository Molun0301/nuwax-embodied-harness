/* 平台设置：来源、生效状态与未保存编辑独立维护。 */
(() => {
"use strict";
const configState = { loaded: false, loading: false, saving: false, saveAcknowledged: false, saveGeneration: 0,
  schemaRequest: 0, schemaPromise: null, pendingUpdates: null,
  dirty: new Map(), errors: new Map(), acknowledgedValues: new Map(), lastSchema: null, robot: null, profile: null };
function profileIdentity() {
  const context = configState.lastSchema?.profile_context;
  return context ? { robot: context.robot, profile: context.profile, revision: context.revision } : {};
}
async function configRequest(url, payload) {
  const response = await fetch(url, payload === undefined ? undefined : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error?.message || result.error || result.detail || ('请求失败：' + response.status));
  return result;
}
function setConfigStatus(state, text) {
  const status = document.getElementById("configStatus");
  status.className = "pill " + state;
  status.textContent = text;
}
const CONFIG_SINGLE_COLOR_KEYS = new Set([
  "NUWAX_GATEWAY_POINTCLOUD3D_COLOR",
  "NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR",
  "NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR",
  "NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR",
]);
const CONFIG_HEIGHT_COLORS_KEY = "NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS";
const CONFIG_HEX_COLOR_PATTERN = /^#[0-9a-fA-F]{6}$/;
function configEscape(text) {
  return String(text == null ? "" : text).replace(/[&<>"']/g, (c) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
  ));
}
function configBooleanValue(value, fallback) {
  const raw = String(value ?? '').trim().toLowerCase();
  if (!raw && fallback !== undefined) return configBooleanValue(fallback);
  if (['true', '1', 'yes', 'on'].includes(raw)) return 'true';
  if (['false', '0', 'no', 'off'].includes(raw)) return 'false';
  return null;
}
function configFieldInputValue(field) {
  const raw = String(field.value ?? '');
  return field.type === 'bool' ? configBooleanValue(raw, field.default_value) ?? raw : raw;
}
function configHexColor(value) {
  const candidate = String(value == null ? "" : value).trim();
  return CONFIG_HEX_COLOR_PATTERN.test(candidate) ? candidate.toLowerCase() : null;
}
function configPaletteColors(value) {
  const colors = String(value == null ? "" : value).split(",").map((item) => configHexColor(item));
  return colors.length === 5 && colors.every(Boolean) ? colors : null;
}
function createConfigColorPicker(value, label) {
  const picker = document.createElement("input");
  picker.type = "color";
  picker.className = "config-color-picker";
  picker.value = configHexColor(value) || "#000000";
  picker.setAttribute("aria-label", label);
  picker.title = label;
  return picker;
}
function renderProfileControls(schema) {
  const context = schema.profile_context;
  const robotSelect = document.getElementById('configRobotSelect');
  const profileSelect = document.getElementById('configProfileSelect');
  if (!robotSelect || !profileSelect) return;
  for (const [select, options, selected] of [[robotSelect, context?.robots || [], context?.robot],
    [profileSelect, context?.profiles || [], context?.profile]]) {
    select.replaceChildren(...options.map(value => Object.assign(document.createElement('option'), { value, textContent: value })));
    if (selected) select.value = selected;
    select.disabled = !context;
  }
  if (context) {
    configState.robot = context.robot;
    configState.profile = context.profile;
  }
  document.getElementById('configProfileStatus').textContent = context
    ? ('当前编辑：' + context.robot + ' / ' + context.profile + '。' + (context.is_active ? '这是已选择的方案。' : '应用此方案后才改变运行设置。')
      + (context.migration_required ? '首次保存会迁移该机器人的旧配置。' : '')
      + (context.running_robot ? ' 启动时：' + context.running_robot + ' / ' + (context.running_profile || '旧配置入口') : ''))
    : '当前使用显式配置文件入口。';
  document.getElementById('configProfileApply').disabled = !context;
  document.getElementById('configProfileSaveAs').disabled = !context;
  document.getElementById('configRestartPending').disabled = !schema.application?.restart_available;
  document.getElementById('configRestartPending').title = schema.application?.restart_unavailable_reason || '';
}
function configDisclosure(title, count) {
  const section = document.createElement('details');
  section.className = 'config-disclosure';
  const summary = document.createElement('summary');
  const label = document.createElement('span');
  label.className = 'config-disclosure-title';
  label.textContent = title;
  summary.appendChild(label);
  if (count !== undefined) {
    const badge = document.createElement('span');
    badge.className = 'config-badge'; badge.textContent = count;
    summary.appendChild(badge);
  }
  const action = document.createElement('span');
  action.className = 'config-disclosure-action';
  action.setAttribute('aria-hidden', 'true');
  summary.appendChild(action);
  section.appendChild(summary);
  return section;
}
function buildOptionsEditor(field, rawInput, changed) {
  const host = document.createElement('div');
  host.className = 'config-options-editor';
  const groups = new Map();
  const controls = new Map();
  for (const [name, spec] of Object.entries(field.options_schema)) {
    const groupName = field.key === 'NUWAX_GEOMETRY_PGO_OPTIONS'
      ? (name.startsWith('scan_context_') ? '扫描上下文：地点检索' : /^(odometry_|gravity_|loop_.*sigma_|optimizer_)/.test(name)
        ? '位姿图：约束权重与优化' : '回环：配准、确认与关键帧') : '参数明细';
    if (!groups.has(groupName)) {
      const section = configDisclosure(groupName);
      section.classList.add('config-options-group');
      const grid = document.createElement('div'); grid.className = 'config-options-grid';
      section.appendChild(grid); host.append(section); groups.set(groupName, { section, grid });
      const query = document.getElementById('configSearch').value.trim().toLowerCase();
      section.open = Boolean(query) || groupName.startsWith('扫描上下文');
    }
    const label = document.createElement('label');
    label.className = 'config-option-field';
    const title = document.createElement('span'); title.textContent = spec.description || name;
    const key = document.createElement('code'); key.textContent = name;
    const input = document.createElement(['bool', 'enum'].includes(spec.type) ? 'select' : 'input');
    input.className = 'field';
    input.setAttribute('aria-label', spec.description || name);
    input.dataset.option = name;
    if (spec.type === 'bool') {
      input.replaceChildren(...[['', '使用算法默认'], ['true', '是'], ['false', '否']].map(([value, textContent]) =>
        Object.assign(document.createElement('option'), { value, textContent })));
    } else if (spec.type === 'enum') {
      input.replaceChildren(...['', ...(spec.choices || [])].map(value =>
        Object.assign(document.createElement('option'), { value, textContent: value || '使用算法默认' })));
    } else if (['int', 'number'].includes(spec.type)) {
      input.type = 'number'; input.step = spec.type === 'int' ? '1' : 'any';
      if (spec.minimum !== undefined) input.min = spec.minimum;
      if (spec.maximum !== undefined) input.max = spec.maximum;
      input.placeholder = '使用算法默认';
    } else {
      input.type = 'text'; input.placeholder = '留空使用默认；数组按结构化格式填写';
    }
    input.addEventListener('change', () => {
      let value;
      try { value = JSON.parse(rawInput.value || '{}'); }
      catch (_) { setConfigStatus('error', '请先修正原始配置格式'); return; }
      if (!value || Array.isArray(value) || typeof value !== 'object') {
        setConfigStatus('error', '原始配置必须是对象'); return;
      }
      if (!input.value.trim()) delete value[name];
      else if (spec.type === 'bool') value[name] = input.value === 'true';
      else if (spec.type === 'enum') value[name] = input.value;
      else if (['int', 'number'].includes(spec.type)) {
        const parsed = Number(input.value);
        if (!Number.isFinite(parsed) || (spec.type === 'int' && !Number.isInteger(parsed))) return;
        value[name] = parsed;
      } else {
        try { value[name] = JSON.parse(input.value); }
        catch (_) { setConfigStatus('error', '数组格式无效，输入已保留'); return; }
      }
      rawInput.value = JSON.stringify(value);
      changed();
    });
    label.append(title, key, input); groups.get(groupName).grid.append(label); controls.set(name, input);
  }
  const sync = () => {
    let value;
    try { value = JSON.parse(rawInput.value || '{}'); }
    catch (_) { return; }
    if (!value || typeof value !== 'object' || Array.isArray(value)) return;
    for (const [name, input] of controls) input.value = value[name] == null ? ''
      : typeof value[name] === 'object' ? JSON.stringify(value[name]) : String(value[name]);
  };
  rawInput.addEventListener('input', sync); sync();
  return host;
}
function showRestartChoice(result = {}) {
  const application = configState.lastSchema?.application || {};
  document.getElementById('configApplyMessage').textContent = (result.effect_note || '已保存的参数可在下次启动时生效。')
    + ' 重启范围：' + (application.restart_scope || '相关运行组件') + '。'
    + (application.restart_unavailable_reason || '');
  document.getElementById('configRestartNow').disabled = !application.restart_available;
  document.getElementById('configApplyDialog').showModal();
}
async function restartConfiguration() {
  document.getElementById('configRestartNow').disabled = true;
  try {
    await configRequest('/api/config/restart', profileIdentity());
    document.getElementById('configApplyDialog').close();
    setConfigStatus('stale', '正在重启');
    document.getElementById('configDirty').textContent = '已安排重启，连接会短暂断开；新网关返回后更新状态。';
    const deadline = Date.now() + 180000;
    const check = async () => {
      try {
        const status = await configRequest('/api/config/status');
        if (status.restart?.state === 'restarted') {
          await loadConfigSchema();
          document.getElementById('configDirty').textContent = status.restart.message;
          return;
        }
        if (status.restart?.state === 'failed') throw Object.assign(new Error(status.restart.message), { terminal: true });
      } catch (error) {
        if (error.terminal) { setConfigStatus('error', '重启失败'); document.getElementById('configDirty').textContent = error.message; return; }
      }
      if (Date.now() < deadline) window.setTimeout(check, 1500);
      else { setConfigStatus('error', '尚未恢复连接'); document.getElementById('configDirty').textContent = '请检查启动日志；若修改了网关地址，请使用新地址访问。'; }
    };
    window.setTimeout(check, 1500);
  } catch (error) {
    setConfigStatus('error', '无法重启'); document.getElementById('configApplyMessage').textContent = error.message;
  }
}
async function profileAction(action) {
  if (configState.saving || configState.loading) return;
  if (configState.dirty.size) {
    document.getElementById('configDirty').textContent = '请先保存当前编辑，再选择或另存方案。'; return;
  }
  configState.saving = true;
  try {
    const identity = profileIdentity();
    if (action === 'create') {
      const name = document.getElementById('configProfileName').value.trim();
      if (!name) throw new Error('请填写新方案名称');
      const result = await configRequest('/api/config/profiles', { ...identity, name });
      configState.profile = result.profile;
      document.getElementById('configProfileName').value = '';
      await loadConfigSchema();
    } else {
      const result = await configRequest('/api/config/activate', identity);
      await loadConfigSchema();
      const failures = Object.entries(result.apply_errors || {});
      if (failures.length) {
        setConfigStatus('error', '已保存，部分热加载失败');
        document.getElementById('configDirty').textContent = failures.map(([key, message]) => key + '：' + message).join('；');
      }
      if (result.restart_required) showRestartChoice(result);
      else if (!failures.length) document.getElementById('configDirty').textContent = result.effect_note;
    }
  } catch (error) {
    setConfigStatus('error', '操作失败'); document.getElementById('configDirty').textContent = error.message;
  } finally { configState.saving = false; }
}
function loadConfigSchema(options) {
  const pending = fetchConfigSchema(options);
  configState.schemaPromise = pending;
  return pending;
}
async function fetchConfigSchema({ afterSave = false } = {}) {
  // 保存回执必须触发新读取；旧的并发刷新不能覆盖保存后的 schema。
  const request = ++configState.schemaRequest;
  // 在请求发出时固定阶段：回执后的手动刷新也可清理已保存的同值编辑，
  // 回执前的读取则不能因为响应较晚就取得这项权限。
  const saveGeneration = configState.saveGeneration;
  const canReconcile = !configState.saving || afterSave || configState.saveAcknowledged;
  const acknowledgedAtRequest = new Map(configState.acknowledgedValues);
  configState.loading = true;
  try {
    const query = new URLSearchParams();
    if (configState.robot) query.set('robot', configState.robot);
    if (configState.profile) query.set('profile', configState.profile);
    const resp = await fetch("/api/config/schema?" + query);
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    const schema = await resp.json();
    if (request !== configState.schemaRequest) return false;
    // 读取开始后若又发起保存，它就不能清理这次保存期间产生的新编辑。
    const reconcileSaved = canReconcile && saveGeneration === configState.saveGeneration;
    // 刷新来源信息时保留尚未保存或被拒绝的输入。
    for (const section of schema.sections || []) {
      for (const field of section.fields || []) {
        if (reconcileSaved && !field.sensitive
            && configState.dirty.get(field.key) === configFieldInputValue(field)) {
          configState.dirty.delete(field.key);
          configState.errors.delete(field.key);
        }
        // 只有回执后的权威读取才能替换该回执基线；读取失败时继续保留。
        if (reconcileSaved && acknowledgedAtRequest.has(field.key)
            && configState.acknowledgedValues.get(field.key) === acknowledgedAtRequest.get(field.key)) {
          configState.acknowledgedValues.delete(field.key);
        }
      }
    }
    configState.lastSchema = schema;
    renderProfileControls(schema);
    renderConfigSchema(schema);
    configState.loaded = true;
    setConfigStatus(configState.errors.size ? "error" : "available", configState.errors.size ? "部分未保存" : "已加载");
    document.getElementById("configEffectNote").textContent = schema.effect_note || "";
    const paths = [...new Set((schema.sections || []).flatMap(section => (section.fields || []).map(field => field.target_path)).filter(Boolean))];
    document.getElementById("configStoragePath").textContent = paths.length ? "保存位置：" + paths.join("；") : "";
    updateConfigDirty();
    return true;
  } catch (err) {
    if (request !== configState.schemaRequest) return false;
    setConfigStatus("error", "加载失败");
    document.getElementById("configDirty").textContent = "加载配置失败：" + err.message + "；未保存输入已保留。";
    return false;
  } finally {
    if (request === configState.schemaRequest) configState.loading = false;
  }
}
function renderConfigSchema(schema) {
  const host = document.getElementById("configSections");
  host.replaceChildren();
  const nav = document.getElementById("configNav");
  if (nav) nav.replaceChildren(Object.assign(document.createElement("div"), { className: "hd eyebrow", textContent: "设置分类" }));
  const navLinks = [];
  const forceAdvanced = document.getElementById("configShowAdvanced").checked;
  const query = (document.getElementById("configSearch").value || "").trim().toLowerCase();
  const match = (f) => !query || f.key.toLowerCase().includes(query) || (f.zh_name || "").toLowerCase().includes(query) || (f.description || "").toLowerCase().includes(query)
    || Object.entries(f.options_schema || {}).some(([key, spec]) => (key + ' ' + (spec.description || '')).toLowerCase().includes(query));
  let sectionIndex = 0;
  for (const section of schema.sections || []) {
    const fields = (section.fields || []).filter(match);
    if (!fields.length) continue;
    const normal = fields.filter((f) => !f.advanced);
    const advanced = fields.filter((f) => f.advanced);
    const block = document.createElement("div");
    block.className = "config-section-block";
    const blockId = "section-" + (section.section_id || ("cfgsec-" + sectionIndex++)).replace(/_/g, "-");
    block.id = blockId;
    const title = document.createElement("div");
    title.className = "config-section-title";
    title.innerHTML = configEscape(section.section_title) +
      ' <span class="config-badge">' + fields.length + "</span>";
    block.appendChild(title);
    if (nav) {
      const link = document.createElement("a");
      link.dataset.target = blockId;
      link.innerHTML = configEscape(section.section_title) +
        ' <span class="config-badge">' + fields.length + "</span>";
      link.addEventListener("click", () => {
        block.scrollIntoView({ behavior: "smooth", block: "start" });
        for (const l of navLinks) l.classList.toggle("on", l === link);
      });
      nav.appendChild(link);
      navLinks.push(link);
    }
    if (normal.length) {
      const grid = document.createElement("div");
      grid.className = "config-grid";
      for (const f of normal) { grid.appendChild(buildConfigRow(f)); }
      block.appendChild(grid);
    }
    if (advanced.length) {
      // 高级设置只有这一层入口，与参数分组共用折叠控件。
      const wrap = configDisclosure('高级项', advanced.length);
      wrap.classList.add('config-adv');
      wrap.open = forceAdvanced || Boolean(query);
      const grid = document.createElement("div");
      grid.className = "config-grid";
      for (const f of advanced) { grid.appendChild(buildConfigRow(f)); }
      wrap.appendChild(grid);
      block.appendChild(wrap);
    }
    host.appendChild(block);
  }
  syncGeometryPipelineFields();
  host.classList.toggle("show-adv", forceAdvanced);
  if (navLinks.length) navLinks[0].classList.add("on");
  setupConfigScrollSpy(host, navLinks);
  if (!host.children.length) {
    host.innerHTML = '<div class="empty panel-meta">无匹配设置项。</div>';
  }
}
function syncGeometryPipelineFields() {
  const selected = document.getElementById("cfg__NUWAX_GEOMETRY_PIPELINE")?.value
    || configState.dirty.get("NUWAX_GEOMETRY_PIPELINE")
    || window.__NUWAX_GEOMETRY_PIPELINE__ || "lio_3d";
  const planar = selected === "slam_2d";
  for (const row of document.querySelectorAll("#configSections [data-key]")) {
    const key = row.dataset.key;
    const spatial = ["NUWAX_GEOMETRY_LIO_BACKEND", "NUWAX_GEOMETRY_MAPPING_BACKEND",
      "NUWAX_GEOMETRY_CORRECTION_BACKEND", "NUWAX_GEOMETRY_PGO_OPTIONS", "NUWAX_NAV_BODY_TO_SUPPORT_Z_M",
      "NUWAX_NAV_LOCAL_PLANNING_ENABLED",
      "NUWAX_NAV_ROBOT_HEIGHT_M", "NUWAX_NAV_SURFACE_VOXEL_SIZE_M",
      "NUWAX_NAV_MIN_STAIR_TREAD_M", "NUWAX_GEOMETRY_TRAVERSABILITY_MAX_STEP_HEIGHT_M",
      "NUWAX_GEOMETRY_TRAVERSABILITY_MAX_SLOPE_DEG"].includes(key)
      || key.startsWith("NUWAX_GATEWAY_POINTCLOUD3D_");
    const disabled = planar && spatial;
    for (const input of row.querySelectorAll("input,select,textarea")) input.disabled = disabled || input.dataset.readonly === 'true';
    row.style.opacity = disabled ? "0.5" : "";
    if (spatial) row.title = disabled ? "该设置仅用于三维执行链" : "";
  }
}
function setupConfigScrollSpy(host, navLinks) {
  if (host._spy) host.removeEventListener("scroll", host._spy);
  if (!navLinks.length) return;
  host._spy = () => {
    const top = host.getBoundingClientRect().top;
    let current = navLinks[0];
    for (const link of navLinks) {
      const block = document.getElementById(link.dataset.target);
      if (block && block.getBoundingClientRect().top - top <= 12) current = link;
    }
    for (const l of navLinks) l.classList.toggle("on", l === current);
  };
  host.addEventListener("scroll", host._spy, { passive: true });
}
function buildConfigRow(f) {
  const row = document.createElement("div");
  row.className = "config-row" + (f.pending_restart ? " pending-restart" : "");
  row.dataset.key = f.key;
  const label = document.createElement("label");
  label.htmlFor = "cfg__" + f.key;
  let badges = "";
  if (f.advanced) badges += ' <span class="config-badge adv">高级</span>';
  if (f.sensitive) badges += ' <span class="config-badge sec">敏感</span>';
  label.innerHTML = '<span class="config-label-name">' + configEscape(f.zh_name || "未命名设置") + badges + '</span>' +
    '<code class="ckey">' + configEscape(f.key) + "</code>";
  if (f.description) label.title = f.description;
  const cell = document.createElement("div");
  cell.className = "cfield";
  let input;
  if (f.type === "bool") {
    input = document.createElement("select");
    for (const opt of ["true", "false"]) {
      const o = document.createElement("option");
      o.value = opt; o.textContent = opt === "true" ? "是" : "否";
      input.appendChild(o);
    }
    const normalized = configBooleanValue(f.value, f.default_value);
    if (normalized === null) {
      const invalid = document.createElement('option');
      invalid.value = String(f.value ?? '');
      invalid.textContent = '无效布尔值：' + (invalid.value || '（空）');
      input.appendChild(invalid);
    }
    input.value = configFieldInputValue(f);
  } else if (Array.isArray(f.choices) && f.choices.length) {
    input = document.createElement("select");
    const labels = Array.isArray(f.choice_labels) ? f.choice_labels : [];
    f.choices.forEach((opt, index) => {
      const o = document.createElement("option");
      // 显示中文名称，提交规范定义的稳定键值。
      o.value = opt;
      o.textContent = labels[index] || opt;
      input.appendChild(o);
    });
    if (!f.choices.includes(f.value) && f.value) {
      const o = document.createElement("option");
      o.value = f.value; o.textContent = f.value + "（当前）";
      input.appendChild(o);
    }
    input.value = f.value || f.choices[0];
  } else if ((f.type === "int" || f.type === "float") && !f.sensitive) {
    input = document.createElement("input");
    input.type = "number";
    if (f.type === "float") input.step = "any";
    input.value = f.value == null ? "" : f.value;
  } else if ((f.type === "json" || ["NUWAX_GEOMETRY_SLAM2D_CONFIG", "NUWAX_GEOMETRY_SLAM2D_DEVICE", "NUWAX_GEOMETRY_POINTLIO_OPTIONS", "NUWAX_GEOMETRY_PGO_OPTIONS"].includes(f.key)) && !f.sensitive) {
    input = document.createElement("textarea");
    input.rows = 3;
    input.value = f.value == null ? "" : f.value;
  } else {
    input = document.createElement("input");
    input.type = f.sensitive ? "password" : "text";
    input.value = f.sensitive ? "" : (f.value == null ? "" : f.value);
    if (f.sensitive) input.placeholder = f.is_set ? "（已设置，留空不改）" : "（未设置）";
    else if (f.type === "csv") input.placeholder = "逗号分隔，如 a,b,c";
  }
  input.id = "cfg__" + f.key;
  input.className = "field";
  if (f.readonly) { input.disabled = true; input.dataset.readonly = 'true'; }
  if (f.description) input.title = f.description;
  const baseline = configState.acknowledgedValues.has(f.key)
    ? configState.acknowledgedValues.get(f.key) : configFieldInputValue(f);
  if (configState.acknowledgedValues.has(f.key)) input.value = baseline;
  input.dataset.original = (f.sensitive && f.is_set) ? "" : baseline;
  if (configState.dirty.has(f.key)) {
    input.value = configState.dirty.get(f.key);
    row.classList.add("changed");
  }
  let singleColorPicker = null;
  let palettePickers = [];
  if (CONFIG_SINGLE_COLOR_KEYS.has(f.key)) {
    const control = document.createElement("div");
    control.className = "config-color-control";
    singleColorPicker = createConfigColorPicker(input.value, f.zh_name + "色盘");
    control.append(input, singleColorPicker);
    cell.appendChild(control);
  } else {
    cell.appendChild(input);
  }
  if (f.key === CONFIG_HEIGHT_COLORS_KEY) {
    const pickerGroup = document.createElement("div");
    pickerGroup.className = "config-palette-pickers";
    const initialColors = configPaletteColors(input.value) || configPaletteColors(f.value) || [];
    palettePickers = Array.from({ length: 5 }, (_unused, index) => {
      const labelText = "高度颜色第" + (index + 1) + "档";
      const picker = createConfigColorPicker(initialColors[index], labelText);
      pickerGroup.appendChild(picker);
      return picker;
    });
    cell.appendChild(pickerGroup);
  }
  const handleTextInput = () => {
    const color = configHexColor(input.value);
    if (singleColorPicker && color) singleColorPicker.value = color;
    const colors = configPaletteColors(input.value);
    if (colors) palettePickers.forEach((picker, index) => { picker.value = colors[index]; });
    onConfigInput(f.key, input, row);
  };
  input.addEventListener("input", handleTextInput);
  input.addEventListener("change", handleTextInput);
  if (f.options_schema) {
    row.classList.add('config-options-row');
    const editor = buildOptionsEditor(f, input, handleTextInput);
    const raw = configDisclosure('原始结构化配置');
    raw.classList.add('config-raw-editor');
    const content = document.createElement('div');
    content.className = 'config-raw-content';
    content.appendChild(input); raw.appendChild(content);
    try { JSON.parse(input.value || '{}'); } catch (_) { raw.open = true; }
    cell.prepend(editor, raw);
  }
  if (singleColorPicker) {
    const applySingleColor = () => {
      input.value = singleColorPicker.value.toLowerCase();
      handleTextInput();
    };
    singleColorPicker.addEventListener("input", applySingleColor);
    singleColorPicker.addEventListener("change", applySingleColor);
  }
  for (const picker of palettePickers) {
    const applyPalette = () => {
      input.value = palettePickers.map((item) => item.value.toLowerCase()).join(",");
      handleTextInput();
    };
    picker.addEventListener("input", applyPalette);
    picker.addEventListener("change", applyPalette);
  }
  if (f.choice_warnings && Object.keys(f.choice_warnings).length) {
    const prerequisite = document.createElement("div");
    prerequisite.className = "config-help config-prerequisite";
    const updatePrerequisite = () => {
      const detail = f.choice_warnings[input.value];
      prerequisite.textContent = detail ? "前置条件：" + (typeof detail === "object" ? detail.reason || "" : String(detail)) : "";
      prerequisite.hidden = !detail;
    };
    input.addEventListener("change", updatePrerequisite);
    updatePrerequisite();
    cell.appendChild(prerequisite);
  }
  if (f.description) {
    const help = document.createElement("p");
    help.className = "config-help";
    help.textContent = f.description;
    cell.appendChild(help);
  }
  const meta = document.createElement("div");
  meta.className = "config-field-meta";
  const source = document.createElement("span");
  source.textContent = "来源：" + (f.source_label || (f.value_source === "environment" ? "进程环境" : f.value_source === "default" ? "默认值" : "配置文件"));
  if (f.source_path) source.title = f.source_path;
  const scope = document.createElement("span");
  scope.textContent = f.apply_scope || "重启对应服务生效";
  meta.append(source, scope);
  if (f.pending_restart) {
    const pending = document.createElement("span");
    pending.className = "config-pending-label";
    pending.textContent = "已保存，待重启生效";
    meta.appendChild(pending);
  }
  if (f.effective_value_known === false) {
    const unknown = document.createElement("span");
    unknown.textContent = f.effective_value_scope === 'inactive'
      ? '兼容保留，当前流水线不使用'
      : f.effective_value_scope === 'external_process'
        ? '外部进程实际采用值未核实；此处仅为启动配置' : "当前进程值未采集";
    meta.appendChild(unknown);
  } else if (f.pending_restart && !f.sensitive && f.effective_value !== undefined) {
    const effective = document.createElement("span");
    effective.textContent = "当前生效：" + String(f.effective_value);
    meta.appendChild(effective);
  }
  cell.appendChild(meta);
  const err = document.createElement("div");
  err.className = "config-err";
  err.id = "cfgerr__" + f.key;
  err.textContent = configState.errors.get(f.key) || "";
  row.classList.toggle("invalid", configState.errors.has(f.key));
  cell.appendChild(err);
  row.append(label, cell);
  return row;
}
function onConfigInput(key, input, row) {
  configState.errors.delete(key);
  row.classList.remove("invalid");
  const error = row.querySelector(".config-err");
  if (error) error.textContent = "";
  // 保存期间恢复旧值也是新编辑，不能被尚未到达的保存回执吞掉。
  const baseline = configState.acknowledgedValues.has(key) ? configState.acknowledgedValues.get(key) : input.dataset.original;
  const changed = Object.hasOwn(configState.pendingUpdates || {}, key) || input.value !== baseline;
  row.classList.toggle("changed", changed);
  if (changed) configState.dirty.set(key, input.value);
  else configState.dirty.delete(key);
  updateConfigDirty();
  if (key === "NUWAX_GEOMETRY_PIPELINE") syncGeometryPipelineFields();
}
function updateConfigDirty() {
  const el = document.getElementById("configDirty");
  const n = configState.dirty.size;
  el.textContent = n ? ("待保存改动：" + n + " 项") : "无未保存改动。";
}
function collectConfigSaveUpdates() {
  const updates = {};
  for (const [key, value] of configState.dirty) {
    updates[key] = value;
  }
  return updates;
}
async function saveConfig() {
  if (!configState.lastSchema || configState.saving) return;
  const updates = collectConfigSaveUpdates();
  if (!Object.keys(updates).length) { updateConfigDirty(); return; }
  const button = document.getElementById("configSave");
  configState.saving = true;
  configState.saveGeneration++;
  configState.saveAcknowledged = false;
  configState.pendingUpdates = updates;
  button.disabled = true;
  try {
    setConfigStatus("stale", "保存中");
    const resp = await fetch("/api/config/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ updates, ...profileIdentity() }),
    });
    const result = await resp.json();
    if (!resp.ok) throw new Error(result.error?.message || result.error || result.detail || "请求失败：" + resp.status);
    configState.saveAcknowledged = true;
    const rejected = result.rejected || {};
    const fields = new Map((configState.lastSchema.sections || []).flatMap(section => section.fields || []).map(field => [field.key, field]));
    for (const [key, submitted] of Object.entries(updates)) {
      const field = fields.get(key);
      if (!field || field.sensitive || Object.hasOwn(rejected, key) || (result.skipped || []).includes(key)) continue;
      // 保存回执已确认新基线，即使后续刷新失败，也不能再把旧 schema 当作编辑原值。
      const baseline = configFieldInputValue({ ...field, value: submitted });
      configState.acknowledgedValues.set(key, baseline);
      const input = document.getElementById('cfg__' + key);
      if (input) input.dataset.original = baseline;
    }
    // 只移除本次成功保存且没有继续编辑的值，不丢失失败项或保存期间的新输入。
    for (const [key, submitted] of Object.entries(updates)) {
      if (configState.dirty.get(key) !== submitted) continue;
      if (Object.hasOwn(rejected, key)) configState.errors.set(key, rejected[key]);
      else { configState.dirty.delete(key); configState.errors.delete(key); }
    }
    let refresh = loadConfigSchema({ afterSave: true });
    let refreshed = await refresh;
    // 手动刷新可接管读取；等最新拥有方完成后才能解除保存期间的编辑保护。
    while (refresh !== configState.schemaPromise) {
      refresh = configState.schemaPromise;
      refreshed = await refresh;
    }
    const rejectedCount = Object.keys(rejected).length;
    // 重启回执独立于刷新结果；网络错误或另一刷新不能吞掉已保存的事实。
    if (result.restart_required) showRestartChoice(result);
    if (!refreshed) return;
    if (rejectedCount) {
      setConfigStatus("error", "部分未保存");
      document.getElementById("configDirty").textContent = "有 " + rejectedCount + " 项未保存，输入及原因已保留；其余改动已保存。";
    } else {
      setConfigStatus("available", "已保存");
      document.getElementById("configDirty").textContent = configState.dirty.size
        ? "本次改动已保存；仍有 " + configState.dirty.size + " 项新编辑待保存。"
        : (result.hot_applied?.length ? result.hot_applied.length + ' 项已热加载。' : '') + (result.effect_note || '改动已保存。');
      const failures = Object.entries(result.apply_errors || {});
      if (failures.length) {
        setConfigStatus('error', '已保存，部分热加载失败');
        document.getElementById('configDirty').textContent = failures.map(([key, message]) => key + '：' + message).join('；');
      }
    }
  } catch (err) {
    setConfigStatus("error", "保存失败");
    document.getElementById("configDirty").textContent = "保存失败：" + err.message + "；输入已保留。";
  } finally {
    configState.saving = false;
    configState.saveAcknowledged = false;
    configState.pendingUpdates = null;
    button.disabled = false;
  }
}
(function wireConfigPanel() {
  const reload = document.getElementById("configReload");
  const save = document.getElementById("configSave");
  const search = document.getElementById("configSearch");
  const adv = document.getElementById("configShowAdvanced");
  if (reload) reload.addEventListener("click", loadConfigSchema);
  if (save) save.addEventListener("click", saveConfig);
  let searchTimer;
  const rerender = () => {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(() => { if (configState.loaded && configState.lastSchema) renderConfigSchema(configState.lastSchema); }, 120);
  };
  for (const id of ['configRobotSelect', 'configProfileSelect']) document.getElementById(id)?.addEventListener('change', async event => {
    if (configState.dirty.size || configState.saving || configState.loading) {
      renderProfileControls(configState.lastSchema);
      document.getElementById('configDirty').textContent = '请先完成并保存当前编辑，再切换方案。'; return;
    }
    configState.robot = document.getElementById('configRobotSelect').value;
    configState.profile = id === 'configRobotSelect' ? null : event.target.value;
    if (!await loadConfigSchema()) {
      renderProfileControls(configState.lastSchema);
    }
  });
  document.getElementById('configProfileSaveAs')?.addEventListener('click', () => profileAction('create'));
  document.getElementById('configProfileApply')?.addEventListener('click', () => profileAction('activate'));
  document.getElementById('configRestartPending')?.addEventListener('click', () => showRestartChoice());
  document.getElementById('configRestartNow')?.addEventListener('click', restartConfiguration);
  document.getElementById('configRestartLater')?.addEventListener('click', () => document.getElementById('configApplyDialog').close());
  if (search) search.addEventListener("input", rerender);
  // 只切换高级分组状态，保留正在编辑的输入。
  if (adv) adv.addEventListener("change", () => {
    const host = document.getElementById("configSections");
    if (host) {
      host.classList.toggle("show-adv", adv.checked);
      for (const group of host.querySelectorAll(".config-adv")) group.open = adv.checked;
    }
  });
})();
window.nuwaxConsoleSettings = {
  open() { if (!configState.loaded) return loadConfigSchema(); },
  reload: loadConfigSchema,
};
})();
