/* 平台设置：来源、生效状态与未保存编辑独立维护。 */
(() => {
"use strict";
const configState = { loaded: false, loading: false, saving: false, dirty: new Map(), errors: new Map(), lastSchema: null };
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
async function loadConfigSchema() {
  if (configState.loading) return;
  configState.loading = true;
  try {
    const resp = await fetch("/api/config/schema");
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    const schema = await resp.json();
    // 刷新来源信息时保留尚未保存或被拒绝的输入。
    for (const section of schema.sections || []) {
      for (const field of section.fields || []) {
        if (!field.sensitive && configState.dirty.get(field.key) === String(field.value ?? "")) {
          configState.dirty.delete(field.key);
          configState.errors.delete(field.key);
        }
      }
    }
    configState.lastSchema = schema;
    renderConfigSchema(schema);
    configState.loaded = true;
    setConfigStatus(configState.errors.size ? "error" : "available", configState.errors.size ? "部分未保存" : "已加载");
    document.getElementById("configEffectNote").textContent = schema.effect_note || "";
    const paths = [...new Set((schema.sections || []).flatMap(section => (section.fields || []).map(field => field.target_path)).filter(Boolean))];
    document.getElementById("configStoragePath").textContent = paths.length ? "保存位置：" + paths.join("；") : "";
    updateConfigDirty();
    return true;
  } catch (err) {
    setConfigStatus("error", "加载失败");
    document.getElementById("configDirty").textContent = "加载配置失败：" + err.message + "；未保存输入已保留。";
    return false;
  } finally {
    configState.loading = false;
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
  const match = (f) => !query || f.key.toLowerCase().includes(query) || (f.zh_name || "").toLowerCase().includes(query) || (f.description || "").toLowerCase().includes(query);
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
      // 高级项：默认隐藏，勾选「展开高级项」或搜索命中时展开（带高度动画）。
      const wrap = document.createElement("div");
      wrap.className = "config-adv" + (query ? " open" : "");
      const bar = document.createElement("div");
      bar.className = "config-adv-bar";
      bar.innerHTML = '<span>高级项</span><span class="config-badge">' + advanced.length + "</span>";
      const inner = document.createElement("div");
      inner.className = "config-adv-inner";
      const grid = document.createElement("div");
      grid.className = "config-grid";
      for (const f of advanced) { grid.appendChild(buildConfigRow(f)); }
      inner.appendChild(grid);
      wrap.append(bar, inner);
      block.appendChild(wrap);
    }
    host.appendChild(block);
  }
  host.classList.toggle("show-adv", forceAdvanced);
  if (navLinks.length) navLinks[0].classList.add("on");
  setupConfigScrollSpy(host, navLinks);
  if (!host.children.length) {
    host.innerHTML = '<div class="empty panel-meta">无匹配设置项。</div>';
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
    input.value = String(f.value).toLowerCase() === "true" ? "true" : "false";
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
  } else if (f.type === "json" && !f.sensitive) {
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
  if (f.description) input.title = f.description;
  input.dataset.original = (f.sensitive && f.is_set) ? "" : (f.value == null ? "" : String(f.value));
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
    unknown.textContent = "当前进程值未采集";
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
  const changed = input.value !== input.dataset.original;
  row.classList.toggle("changed", changed);
  if (changed) configState.dirty.set(key, input.value);
  else configState.dirty.delete(key);
  updateConfigDirty();
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
  button.disabled = true;
  try {
    setConfigStatus("stale", "保存中");
    const resp = await fetch("/api/config/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ updates }),
    });
    const result = await resp.json();
    if (!resp.ok) throw new Error(typeof result.detail === "string" ? result.detail : "HTTP " + resp.status);
    const rejected = result.rejected || {};
    // 只移除本次成功保存且没有继续编辑的值，不丢失失败项或保存期间的新输入。
    for (const [key, submitted] of Object.entries(updates)) {
      if (configState.dirty.get(key) !== submitted) continue;
      if (Object.hasOwn(rejected, key)) configState.errors.set(key, rejected[key]);
      else { configState.dirty.delete(key); configState.errors.delete(key); }
    }
    if (!await loadConfigSchema()) return;
    const rejectedCount = Object.keys(rejected).length;
    if (rejectedCount) {
      setConfigStatus("error", "部分未保存");
      document.getElementById("configDirty").textContent = "有 " + rejectedCount + " 项未保存，输入及原因已保留；其余改动已保存。";
    } else {
      setConfigStatus("available", "已保存");
      document.getElementById("configDirty").textContent = configState.dirty.size
        ? "本次改动已保存；仍有 " + configState.dirty.size + " 项新编辑待保存。"
        : "改动已保存。请按各项说明重启对应服务。";
    }
  } catch (err) {
    setConfigStatus("error", "保存失败");
    document.getElementById("configDirty").textContent = "保存失败：" + err.message + "；输入已保留。";
  } finally {
    configState.saving = false;
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
  const rerender = () => { if (configState.loaded && configState.lastSchema) renderConfigSchema(configState.lastSchema); };
  if (search) search.addEventListener("input", rerender);
  // 勾选直接切 class 触发高度动画，不重渲染（重渲染会打断过渡）。
  if (adv) adv.addEventListener("change", () => {
    const host = document.getElementById("configSections");
    if (host) host.classList.toggle("show-adv", adv.checked);
  });
})();
window.nuwaxConsoleSettings = {
  open() { if (!configState.loaded) return loadConfigSchema(); },
  reload: loadConfigSchema,
};
})();
