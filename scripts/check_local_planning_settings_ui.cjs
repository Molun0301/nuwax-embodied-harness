#!/usr/bin/env node
/* Offline production settings UI, controlled fetch/DOM; no browser, robot or native process. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const KEY = 'NUWAX_NAV_LOCAL_PLANNING_ENABLED';
const source = fs.readFileSync(path.join(__dirname, '../gateways/http/static/js/console-settings.js'), 'utf8');

class Element {
  constructor(tag) {
    this.tagName = tag.toUpperCase(); this.children = []; this.dataset = {}; this.style = {};
    this.events = {}; this.className = ''; this._value = undefined; this.checked = false; this.modalCount = 0;
    this.classList = {
      contains: name => this.className.split(' ').includes(name),
      add: name => { if (!this.classList.contains(name)) this.className += ' ' + name; },
      remove: name => { this.className = this.className.split(' ').filter(value => value !== name).join(' '); },
      toggle: (name, enabled) => { if (enabled) this.classList.add(name); else this.classList.remove(name); },
    };
  }
  get value() {
    if (this.tagName === 'SELECT') return this._value === undefined ? this.children[0]?.value || ''
      : this.children.some(child => child.value === this._value) ? this._value : '';
    return this._value ?? '';
  }
  set value(value) { this._value = String(value); }
  appendChild(child) {
    if (child.parent) child.parent.children = child.parent.children.filter(item => item !== child);
    child.parent = this; this.children.push(child); return child;
  }
  append(...children) { children.forEach(child => this.appendChild(child)); }
  prepend(...children) { children.reverse().forEach(child => { this.appendChild(child); this.children.pop(); this.children.unshift(child); }); }
  replaceChildren(...children) { this.children = []; this.append(...children); }
  addEventListener(name, callback) { (this.events[name] ||= []).push(callback); }
  removeEventListener() {}
  emit(name) { for (const callback of this.events[name] || []) callback({target: this}); }
  setAttribute(name, value) { this[name] = value; }
  getBoundingClientRect() { return {top: 0}; }
  scrollIntoView() {}
  showModal() { this.modalCount++; this.open = true; }
  close() { this.open = false; }
  matches(selector) {
    return selector === '[data-key]' ? !!this.dataset.key
      : selector.startsWith('.') ? this.classList.contains(selector.slice(1))
      : selector.split(',').map(value => value.trim().toUpperCase()).includes(this.tagName);
  }
  querySelectorAll(selector) { return this.children.flatMap(child => [...(child.matches(selector) ? [child] : []), ...child.querySelectorAll(selector)]); }
  querySelector(selector) { return this.querySelectorAll(selector)[0] || null; }
}
function descendants(element) { return [element, ...element.children.flatMap(descendants)]; }
function text(element) { return descendants(element).map(item => item.textContent || '').join(' '); }
function harness() {
  const nodes = new Map();
  for (const id of ['configSections', 'configShowAdvanced', 'configSearch', 'configStatus', 'configDirty',
    'configEffectNote', 'configStoragePath', 'configApplyMessage', 'configRestartNow', 'configApplyDialog',
    'configSave', 'configReload']) {
    const element = new Element(id === 'configSearch' ? 'input' : 'div'); element.id = id; nodes.set(id, element);
  }
  const document = {
    createElement: tag => new Element(tag),
    getElementById: id => nodes.get(id) || [...nodes.values()].flatMap(descendants).find(node => node.id === id) || null,
    querySelectorAll: selector => selector.startsWith('#configSections ')
      ? nodes.get('configSections').querySelectorAll(selector.replace('#configSections ', '')) : [],
  };
  const requests = [];
  const fetch = (url, options) => new Promise((resolve, reject) => {
    requests.push({url, options, reject, resolve: (data, ok = true) => resolve({ok, status: ok ? 200 : 500, json: async () => data})});
  });
  const window = {setTimeout};
  const context = vm.createContext({document, window, fetch, URLSearchParams, setTimeout, clearTimeout, console});
  const exports = 'window.__test = {configState, configBooleanValue, configFieldInputValue, buildConfigRow, renderConfigSchema, loadConfigSchema, saveConfig};\n';
  vm.runInContext(source.replace('window.nuwaxConsoleSettings = {', exports + 'window.nuwaxConsoleSettings = {'), context);
  return {api: window.__test, window, document, nodes, requests};
}
function schema(value = 'true', extras = {}) {
  return {sections: [{section_id: 'nav_stack', section_title: '导航与探索', fields: [{
    key: KEY, zh_name: '启用局部规划', type: 'bool', value, default_value: 'true', advanced: false,
    apply_mode: 'restart', apply_scope: '保存后重启导航后端生效', effective_value_known: false,
    effective_value_scope: 'external_process', target_path: '/synthetic/main.env', ...extras,
  }]}], application: {restart_available: true, restart_scope: '导航后端'}, effect_note: '按提示重启'};
}
const flush = () => new Promise(resolve => setImmediate(resolve));
async function load(h, value = 'true', extras = {}) {
  const pending = h.api.loadConfigSchema(); h.requests.at(-1).resolve(schema(value, extras)); await pending;
}
function edit(h, value) { const input = h.document.getElementById('cfg__' + KEY); input.value = value; input.emit('change'); }
function receipt(extra = {}) { return {written: {robot_profile_env: [KEY]}, rejected: {}, hot_applied: [],
  restart_required: true, effect_note: '已保存，重启导航后端后生效', ...extra}; }
let checks = 0;
async function check(name, action) { await action(); checks++; console.log('PASS ' + name); }

(async () => {
  await check('aliases and blank default-ON normalize display and original without a false dirty edit', async () => {
    for (const [value, expected] of [['true', 'true'], ['1', 'true'], [' YES ', 'true'], ['on', 'true'],
      ['', 'true'], ['   ', 'true'], [null, 'true'], ['false', 'false'], ['0', 'false'], [' NO ', 'false'], ['off', 'false']]) {
      const h = harness(); await load(h, value);
      const input = h.document.getElementById('cfg__' + KEY);
      assert.equal(input.value, expected, String(value)); assert.equal(input.dataset.original, expected);
      edit(h, expected); assert.equal(h.api.configState.dirty.size, 0);
    }
  });
  await check('invalid nonblank input stays explicit and can be corrected, never silently becomes false', async () => {
    const h = harness(); await load(h, 'perhaps');
    const input = h.document.getElementById('cfg__' + KEY);
    assert.equal(input.value, 'perhaps'); assert.match(text(input), /无效布尔值/);
    edit(h, 'true'); assert.equal(h.api.configState.dirty.get(KEY), 'true');
  });
  await check('default-OFF booleans still render blank as their declared default', async () => {
    const h = harness(); await load(h, '', {default_value: 'false'});
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'false');
  });
  await check('3D-only setting is disabled on the planar pipeline without changing its saved preference', async () => {
    const h = harness(); h.window.__NUWAX_GEOMETRY_PIPELINE__ = 'slam_2d'; await load(h, 'false');
    const input = h.document.getElementById('cfg__' + KEY);
    assert.equal(input.disabled, true); assert.equal(input.value, 'false');
    assert.equal(h.api.configState.dirty.size, 0);
    h.window.__NUWAX_GEOMETRY_PIPELINE__ = 'lio_3d'; await load(h, 'false');
    assert.equal(h.document.getElementById('cfg__' + KEY).disabled, false);
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'false');
  });
  await check('saved disable reloads false with restart receipt and external effective state unknown', async () => {
    const h = harness(); await load(h); edit(h, 'false'); const pending = h.api.saveConfig();
    assert.equal(JSON.parse(h.requests.at(-1).options.body).updates[KEY], 'false');
    h.requests.at(-1).resolve(receipt()); await flush();
    h.requests.at(-1).resolve(schema('false', {pending_restart: true})); await pending;
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'false');
    assert.equal(h.api.configState.dirty.size, 0);
    assert.equal(h.nodes.get('configApplyDialog').modalCount, 1);
    assert.match(text(h.nodes.get('configSections')), /外部进程实际采用值未核实/);
    assert.doesNotMatch(text(h.nodes.get('configSections')), /当前生效/);
  });
  for (const staleAfterAuthoritative of [false, true]) {
    await check('restore old value during save survives overlapping reload; stale arrives ' + (staleAfterAuthoritative ? 'last' : 'first'), async () => {
      const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
      const saveRequest = h.requests.at(-1); edit(h, 'true');
      assert.equal(h.api.configState.dirty.get(KEY), 'true');
      const reload = h.api.loadConfigSchema(); const stale = h.requests.at(-1);
      saveRequest.resolve(receipt()); await flush();
      const authoritative = h.requests.at(-1); assert.notEqual(authoritative, stale);
      assert.match(authoritative.url, /\/api\/config\/schema/);
      if (!staleAfterAuthoritative) { stale.resolve(schema('true')); await reload; }
      authoritative.resolve(schema('false', {pending_restart: true})); await save;
      if (staleAfterAuthoritative) { stale.resolve(schema('true')); await reload; }
      const input = h.document.getElementById('cfg__' + KEY);
      assert.equal(input.value, 'true'); assert.equal(input.dataset.original, 'false');
      assert.equal(h.api.configState.dirty.get(KEY), 'true');
      assert.equal(h.api.configState.lastSchema.sections[0].fields[0].value, 'false');
      assert.equal(h.nodes.get('configApplyDialog').modalCount, 1);
      assert.equal(h.api.configState.loading, false);
    });
  }
  await check('precommit reload matching a restored draft cannot clear that edit', async () => {
    const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
    const saveRequest = h.requests.at(-1); edit(h, 'true');
    await load(h, 'true'); assert.equal(h.api.configState.dirty.get(KEY), 'true');
    saveRequest.resolve(receipt()); await flush();
    h.requests.at(-1).resolve(schema('false')); await save;
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'true');
    assert.equal(h.api.configState.dirty.get(KEY), 'true');
  });
  await check('reload issued before save cannot clear a newer restored edit when it returns during save', async () => {
    const h = harness(); await load(h); edit(h, 'false');
    const reload = h.api.loadConfigSchema(); const beforeSave = h.requests.at(-1);
    const save = h.api.saveConfig(); const saveRequest = h.requests.at(-1);
    edit(h, 'true');
    beforeSave.resolve(schema('true')); await reload;
    assert.equal(h.api.configState.dirty.get(KEY), 'true');
    saveRequest.resolve(receipt()); await flush();
    h.requests.at(-1).resolve(schema('false', {pending_restart: true})); await save;
    const input = h.document.getElementById('cfg__' + KEY);
    assert.equal(input.value, 'true'); assert.equal(input.dataset.original, 'false');
    assert.equal(h.api.configState.dirty.get(KEY), 'true');
    assert.equal(h.nodes.get('configApplyDialog').modalCount, 1);
  });
  for (const automaticReturnsFirst of [false, true]) {
    await check('post-commit manual reload reconciles equal draft when automatic response arrives ' + (automaticReturnsFirst ? 'first' : 'last'), async () => {
      const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
      h.requests.at(-1).resolve(receipt()); await flush();
      const automatic = h.requests.at(-1);
      edit(h, 'true'); edit(h, 'false');
      assert.equal(h.api.configState.dirty.get(KEY), 'false');
      const reload = h.api.loadConfigSchema(); const manual = h.requests.at(-1);
      if (automaticReturnsFirst) { automatic.resolve(schema('false')); await flush(); }
      manual.resolve(schema('false', {pending_restart: true})); await reload;
      if (!automaticReturnsFirst) automatic.resolve(schema('false'));
      await save;
      const input = h.document.getElementById('cfg__' + KEY);
      assert.equal(input.value, 'false'); assert.equal(input.dataset.original, 'false');
      assert.equal(h.api.configState.dirty.size, 0);
      assert.doesNotMatch(h.nodes.get('configDirty').textContent, /待保存改动|项新编辑待保存/);
      assert.equal(h.nodes.get('configApplyDialog').modalCount, 1);
      assert.equal(h.api.configState.saveAcknowledged, false);
    });
  }
  await check('superseded automatic refresh retains edit protection until latest manual refresh finishes', async () => {
    const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
    h.requests.at(-1).resolve(receipt()); await flush();
    const automatic = h.requests.at(-1);
    const reload = h.api.loadConfigSchema(); const manual = h.requests.at(-1);
    automatic.resolve(schema('false')); await flush();
    assert.equal(h.api.configState.saving, true);
    assert.equal(h.api.configState.pendingUpdates[KEY], 'false');
    edit(h, 'true');
    manual.resolve(schema('false', {pending_restart: true})); await Promise.all([reload, save]);
    const input = h.document.getElementById('cfg__' + KEY);
    assert.equal(input.value, 'true'); assert.equal(input.dataset.original, 'false');
    assert.equal(h.api.configState.dirty.get(KEY), 'true');
    assert.equal(h.nodes.get('configApplyDialog').modalCount, 1);
    assert.equal(h.api.configState.saving, false);
  });
  await check('restart receipt survives a failed post-save refresh', async () => {
    const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
    h.requests.at(-1).resolve(receipt()); await flush();
    h.requests.at(-1).reject(new Error('synthetic offline')); await save;
    assert.equal(h.nodes.get('configApplyDialog').modalCount, 1);
    assert.match(h.nodes.get('configDirty').textContent, /加载配置失败/);
    assert.equal(h.api.configState.saving, false);
  });
  await check('acknowledged baseline survives failed refresh and rerender so restoring old value remains dirty', async () => {
    const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
    h.requests.at(-1).resolve(receipt()); await flush();
    h.requests.at(-1).reject(new Error('synthetic offline')); await save;
    assert.equal(h.api.configState.acknowledgedValues.get(KEY), 'false');
    h.api.renderConfigSchema(h.api.configState.lastSchema);
    let input = h.document.getElementById('cfg__' + KEY);
    assert.equal(input.value, 'false'); assert.equal(input.dataset.original, 'false');
    edit(h, 'true'); assert.equal(h.api.configState.dirty.get(KEY), 'true');
    await load(h, 'false', {pending_restart: true});
    input = h.document.getElementById('cfg__' + KEY);
    assert.equal(input.value, 'true'); assert.equal(input.dataset.original, 'false');
    assert.equal(h.api.configState.dirty.get(KEY), 'true');
    assert.equal(h.api.configState.acknowledgedValues.size, 0);
  });
  await check('rejected edit and its error survive the authoritative refresh', async () => {
    const h = harness(); await load(h); edit(h, 'false'); const save = h.api.saveConfig();
    h.requests.at(-1).resolve(receipt({written: {}, rejected: {[KEY]: 'synthetic rejection'}, restart_required: false}));
    await flush(); h.requests.at(-1).resolve(schema('true')); await save;
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'false');
    assert.equal(h.api.configState.dirty.get(KEY), 'false');
    assert.equal(h.api.configState.errors.get(KEY), 'synthetic rejection');
    assert.equal(h.api.configState.acknowledgedValues.size, 0);
    assert.equal(h.nodes.get('configApplyDialog').modalCount, 0);
  });
  await check('late stale failure cannot overwrite a newer loaded schema', async () => {
    const h = harness(); const old = h.api.loadConfigSchema(); const stale = h.requests.at(-1);
    await load(h, 'false'); stale.reject(new Error('old request')); await old;
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'false');
    assert.equal(h.nodes.get('configStatus').textContent, '已加载');
  });
  await check('active hot-apply shows applying and previous adopted value without a restart instruction', async () => {
    const hotKey = 'NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE';
    for (const advanced of [false, true]) {
      const h = harness();
      const data = schema('0.6', {key: hotKey, type: 'float', default_value: '0.2', advanced,
        apply_mode: 'hot', effective_value_known: true, effective_value_scope: 'current_process',
        effective_value: '0.2', pending_restart: true});
      data.profile_context = {is_active: true};
      data.application.hot_apply_pending = [hotKey];
      h.api.renderConfigSchema(data);
      const rendered = text(h.nodes.get('configSections'));
      assert.match(rendered, /已保存，正在应用/);
      assert.match(rendered, /当前生效：0.2/);
      assert(!rendered.includes('已保存，待重启生效'));
    }
  });
  await check('hot-apply status is not attributed to an inactive selected profile', async () => {
    const h = harness();
    const data = schema('false', {pending_restart: true});
    data.profile_context = {is_active: false}; data.application.hot_apply_pending = [KEY];
    h.api.renderConfigSchema(data);
    const rendered = text(h.nodes.get('configSections'));
    assert(!rendered.includes('正在应用'));
    assert.match(rendered, /已保存，待重启生效/);
  });
  await check('completion removes applying badge while preserving an unsaved edit', async () => {
    const h = harness();
    const data = schema('false', {effective_value_known: true, effective_value_scope: 'current_process',
      effective_value: 'true', pending_restart: true});
    data.profile_context = {is_active: true}; data.application.hot_apply_pending = [KEY];
    h.api.renderConfigSchema(data); edit(h, 'true');
    data.application.hot_apply_pending = [];
    data.sections[0].fields[0].pending_restart = false;
    data.sections[0].fields[0].effective_value = 'false';
    h.api.renderConfigSchema(data);
    assert(!text(h.nodes.get('configSections')).includes('正在应用'));
    assert(!text(h.nodes.get('configSections')).includes('待重启生效'));
    assert.equal(h.document.getElementById('cfg__' + KEY).value, 'true');
    assert.equal(h.api.configState.dirty.get(KEY), 'true');
  });
  await check('hot-and-restart retains external restart scope after in-process applying finishes', async () => {
    const h = harness();
    const data = schema('false', {apply_mode: 'hot_and_restart', apply_scope: '本进程热加载；外部进程重启生效',
      pending_restart: true, effective_value_known: true, effective_value_scope: 'current_process', effective_value: 'true'});
    data.profile_context = {is_active: true}; data.application.hot_apply_pending = [KEY];
    h.api.renderConfigSchema(data);
    assert.match(text(h.nodes.get('configSections')), /正在应用/);
    assert.match(text(h.nodes.get('configSections')), /外部进程重启生效/);
    data.application.hot_apply_pending = []; data.sections[0].fields[0].effective_value = 'false';
    h.api.renderConfigSchema(data);
    assert(!text(h.nodes.get('configSections')).includes('正在应用'));
    assert.match(text(h.nodes.get('configSections')), /已保存，待重启生效/);
  });
  await check('applying status does not expose a sensitive adopted value', async () => {
    const h = harness();
    const field = schema('masked').sections[0].fields[0];
    Object.assign(field, {type: 'str', sensitive: true, effective_value_known: true,
      effective_value: 'synthetic-private-value', pending_restart: true});
    const rendered = text(h.api.buildConfigRow(field, true));
    assert.match(rendered, /正在应用/);
    assert(!rendered.includes('synthetic-private-value'));
  });
  console.log(JSON.stringify({checks, status: 'passed', scope: 'production JS with synthetic DOM and fetch; no browser or device claim'}));
})().catch(error => { console.error(error); process.exitCode = 1; });
