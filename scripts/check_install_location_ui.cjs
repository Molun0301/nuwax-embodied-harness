#!/usr/bin/env node
// Exercise production event handlers and rendering with a small offline DOM/fetch harness.
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'scripts/device_manager/static/index.html'), 'utf8');
class Element {
  constructor() {
    this.textContent = ''; this.value = ''; this.hidden = false; this.disabled = false;
    this.dataset = {}; this.listeners = {}; this.children = []; this.open = false;
    this.classList = { toggle() {}, add() {}, remove() {} };
  }
  addEventListener(type, handler) { (this.listeners[type] ||= []).push(handler); }
  emit(type) { return Promise.all((this.listeners[type] || []).map(handler => handler({ target: this, preventDefault() {} }))); }
  click() { return this.disabled ? Promise.resolve() : this.emit('click'); }
  querySelector(selector) { if (selector === '.status-icon') return this.icon ||= new Element(); return null; }
  querySelectorAll() { return []; }
  replaceChildren(...children) { this.children = children; }
  appendChild(child) { this.children.push(child); }
  showModal() { assert.equal(this.open, false, 'Do not reopen an already-open dialog'); this.open = true; }
  close() { this.open = false; this.emit('close'); }
  focus() { this.focused = true; }
}
const nodes = new Map(Array.from(html.matchAll(/\bid="([^"]+)"/g), match => [match[1], new Element()]));
const calls = [];
let response = () => { throw new Error('Unexpected request'); };
const context = vm.createContext({
  document: {
    readyState: 'loading', addEventListener() {}, getElementById: id => nodes.get(id),
    createElement: () => new Element(), querySelector: () => null, querySelectorAll: () => []
  },
  fetch: async (url, options) => { calls.push({ url, options }); return response(url, options); }
});
let source = fs.readFileSync(path.join(root, 'scripts/device_manager/static/app.js'), 'utf8');
const end = source.lastIndexOf('})();');
source = source.slice(0, end) + '\nglobalThis.test = {state, applyStatus, initWizardEvents, loadHardware, renderPlan, generatePlan, submitInstall, loadInstallDirectories};\n' + source.slice(end);
vm.runInContext(source, context);
const { state, applyStatus, initWizardEvents, loadHardware, renderPlan, generatePlan, submitInstall, loadInstallDirectories } = context.test;
const el = id => nodes.get(id);
const json = (data, status = 200) => ({ ok: status < 400, status, headers: { get: () => 'application/json' }, json: async () => data });
const flush = () => new Promise(resolve => setImmediate(resolve));
const deferred = () => { let resolve; return { promise: new Promise(r => { resolve = r; }), resolve: value => resolve(value) }; };
const setPath = async value => { el('installDirInput').value = value; await el('installDirInput').emit('input'); };
const installDir = '/mnt/机器人 data/<literal>& "directory"';
const status = (overrides = {}) => ({ installation: { installed: false, install_dir: installDir, manager_dir: '/home/user/.local/share/neh', location_editable: true, ...overrides } });
(async () => {
  initWizardEvents();
  state.hardwareReady = true;
  state.csrfToken = 'fixture-csrf';
  for (const installed of [false, true, false]) {
    applyStatus(status({ installed, location_editable: !installed }));
    assert.equal(el('setupInstallDir').textContent, installDir);
    assert.equal(el('installDir').textContent, installDir);
    assert.equal(el('setupInstallDir').innerHTML, undefined, 'Device paths must use textContent');
    assert.equal(el('installLocationEditor').hidden, installed);
    assert.equal(el('browseInstallDir').disabled, installed);
  }
  applyStatus({ installation: { installed: false } });
  assert.equal(el('setupInstallDir').textContent, '未知', 'Do not invent missing paths');
  assert.equal(el('installDir').textContent, '未知');
  assert.equal(el('installLocationEditor').hidden, true, 'Unknown capabilities are read-only');
  state.hardwareReady = true;
  applyStatus(status());
  assert.equal(el('nextToModules').disabled, false);

  // Draft paths never change the effective root; stale plans and Continue are disabled.
  state.plan = { id: 'old-plan' };
  await setPath('/mnt/new NEH');
  assert.equal(state.plan, null);
  assert.equal(el('nextToModules').disabled, true);
  assert.equal(el('startInstall').disabled, true);
  assert.equal(el('setupInstallDir').textContent, installDir);
  applyStatus(status());
  assert.equal(el('installDirInput').value, '/mnt/new NEH', 'Status refresh preserves the draft');
  const requestsBeforePlan = calls.length;
  await generatePlan(); await submitInstall();
  assert.equal(calls.length, requestsBeforePlan, 'Unsaved paths cannot generate or submit plans');
  response = () => json({ hardware: {}, recommendation: { modules: ['core'] }, supported: true });
  await loadHardware();
  assert.equal(el('nextToModules').disabled, true, 'Late hardware response cannot bypass unsaved location gate');

  // Save failure retains both the effective location and the draft; retry succeeds once.
  response = () => json({ error: { code: 'location_invalid', message: '目标目录不是空目录' } }, 409);
  await el('saveInstallDir').click();
  assert.match(el('installLocationMessage').textContent, /目标目录不是空目录/);
  assert.equal(el('setupInstallDir').textContent, installDir);
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  const oldHardware = deferred();
  response = () => oldHardware.promise;
  const oldHardwareRequest = loadHardware();
  const pendingSave = deferred();
  response = url => url === '/api/manager/hardware' ? json({ hardware: { free_disk_bytes: 1234 }, supported: true }) : pendingSave.promise;
  const firstSave = el('saveInstallDir').click();
  const beforeDuplicate = calls.length;
  await el('saveInstallDir').click();
  assert.equal(calls.length, beforeDuplicate, 'Repeated save does not duplicate the request');
  assert.equal(el('browseInstallDir').disabled, true);
  pendingSave.resolve(json({ install_dir: '/mnt/new NEH', manager_dir: '/home/user/.local/share/neh', editable: true }));
  await firstSave;
  oldHardware.resolve(json({ hardware: { free_disk_bytes: 999999 }, supported: true }));
  await oldHardwareRequest;
  const request = calls.findLast(call => call.url === '/api/manager/install-location');
  assert.equal(calls.at(-1).url, '/api/manager/hardware', 'Successful save refreshes the selected disk');
  assert.equal(state.hardware.free_disk_bytes, 1234);
  assert.equal(request.url, '/api/manager/install-location');
  assert.equal(request.options.method, 'PUT');
  assert.equal(request.options.headers['X-NEH-CSRF'], 'fixture-csrf');
  assert.deepEqual(JSON.parse(request.options.body), { install_dir: '/mnt/new NEH' });
  assert.equal(el('setupInstallDir').textContent, '/mnt/new NEH');
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  assert.equal(el('nextToModules').disabled, false);
  await setPath('/discard');
  await el('resetInstallDir').click();
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  assert.equal(el('nextToModules').disabled, false);

  // Browser is device-side, uses literal path labels, and selection is only a draft.
  const literal = '<img src=x onerror=alert(1)>& "目录"';
  response = () => json({ path: '/mnt', parent: '/', writable: true, directories: [{ name: literal, path: '/mnt/' + literal }] });
  await el('browseInstallDir').click(); await flush();
  assert.equal(calls.at(-1).url, '/api/manager/install-directories');
  assert.equal(el('installDirectoryDialog').open, true);
  const child = el('directoryList').children[0];
  assert.equal(child.textContent, literal);
  assert.equal(child.innerHTML, undefined, 'Directory names must never be interpreted as HTML');
  response = () => json({ path: '/mnt/' + literal, parent: '/mnt', writable: true, directories: [] });
  await child.click(); await flush();
  assert.equal(calls.at(-1).url, '/api/manager/install-directories?path=' + encodeURIComponent('/mnt/' + literal));
  await el('directorySelect').click();
  assert.equal(el('installDirectoryDialog').open, false);
  assert.equal(el('installDirInput').value, '/mnt/' + literal);
  assert.equal(el('setupInstallDir').textContent, '/mnt/new NEH');
  assert.equal(el('nextToModules').disabled, true);
  await el('resetInstallDir').click();

  // Close/Escape rejects late replies; repeated opens and navigation use newest response.
  const stale = deferred(); response = () => stale.promise;
  await el('browseInstallDir').click();
  const openCalls = calls.length;
  await el('browseInstallDir').click();
  assert.equal(calls.length, openCalls);
  await el('directoryCancel').click();
  stale.resolve(json({ path: '/late', directories: [], writable: true })); await flush();
  assert.equal(state.directory, null);
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  const oldNavigation = deferred(); response = () => oldNavigation.promise;
  await el('browseInstallDir').click();
  response = () => json({ path: '/newest', parent: '/', directories: [], writable: false });
  await loadInstallDirectories('/newest');
  oldNavigation.resolve(json({ path: '/stale', directories: [], writable: true })); await flush();
  assert.equal(el('directoryCurrent').textContent, '/newest');
  assert.equal(el('directorySelect').disabled, true);
  assert.match(el('directoryMessage').textContent, /无法写入/);
  response = () => json({ path: '/', parent: null, directories: [], writable: false });
  await el('directoryParent').click(); await flush();
  assert.equal(calls.at(-1).url, '/api/manager/install-directories?path=%2F');
  assert.equal(el('directoryParent').disabled, true);
  response = () => json({ error: { message: '没有权限读取目录' } }, 403);
  el('directoryPathInput').value = '/blocked';
  await el('directoryPathForm').emit('submit'); await flush();
  assert.match(el('directoryMessage').textContent, /没有权限读取目录/);
  assert.equal(el('directorySelect').disabled, true);
  el('installDirectoryDialog').close(); // Native Escape dispatches the same close event.

  // Existing installations and jobs stay read-only; path refresh invalidates old plans.
  applyStatus(status({ installed: true, location_editable: false }));
  const readonlyCalls = calls.length;
  await el('browseInstallDir').click(); await el('saveInstallDir').click();
  assert.equal(calls.length, readonlyCalls);
  assert.match(el('installLocationReadonly').textContent, /只读/);
  applyStatus({ ...status({ location_editable: false, location_reason: '安装任务执行中' }), active_job: { id: 'job' } });
  assert.equal(el('installLocationReadonly').textContent, '安装任务执行中');
  applyStatus(status());
  state.plan = { id: 'previous-location' };
  applyStatus(status({ install_dir: '/changed-in-another-tab' }));
  assert.equal(state.plan, null);
  assert.equal(state.hardwareReady, false, 'Cross-tab root change invalidates the prior disk check');
  assert.equal(state.hardware, null);
  assert.equal(el('hardwareDetails').hidden, true);
  assert.equal(el('nextToModules').disabled, true);
  assert.equal(el('retryHardware').hidden, false);
  assert.match(el('hardwareStatusText').textContent, /安装位置已变化/);
  renderPlan({ action: 'install', install_dir: installDir, blockers: [], warnings: [], modules: ['core'] }, el('planStatus'));
  assert.match(el('planDetails').innerHTML, /安装位置/);
  assert(el('planDetails').innerHTML.includes('&lt;literal&gt;&amp; &quot;directory&quot;'));
  assert(!source.includes('showDirectoryPicker'), 'Do not select folders on the browser client');
  console.log('Install-location UI: draft/save/errors, CSRF, duplicate requests, directory browse/select/parent, cancellation/races, read-only state and plan-path checks passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
