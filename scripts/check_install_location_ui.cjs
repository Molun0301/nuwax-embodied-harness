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
let previewResponse = (url) => {
  let parentDir = new URL(url, 'http://device').searchParams.get('parent_dir');
  const home = state.status.installation.user_home;
  if (parentDir === '~' || parentDir.startsWith('~/')) parentDir = home + parentDir.slice(1);
  if (!parentDir.startsWith('/')) return json({ error: { message: '请输入绝对路径' } }, 400);
  parentDir = path.posix.resolve(parentDir);
  const installDir = path.posix.basename(parentDir) === 'Nuwax-embodied-harness'
    ? parentDir : path.posix.join(parentDir, 'Nuwax-embodied-harness');
  return json({ install_dir: installDir, parent_dir: path.posix.dirname(installDir), user_home: home });
};
const context = vm.createContext({
  document: {
    readyState: 'loading', addEventListener() {}, getElementById: id => nodes.get(id),
    createElement: () => new Element(), querySelector: () => null, querySelectorAll: () => []
  },
  fetch: async (url, options) => {
    calls.push({ url, options });
    return url.startsWith('/api/manager/install-location?parent_dir=') ? previewResponse(url, options) : response(url, options);
  }
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
const installFolder = 'Nuwax-embodied-harness';
const userHome = '/home/user';
const installDir = '/mnt/机器人 data/<literal>& "directory"';
const newParent = '/mnt/new NEH';
const newInstallDir = newParent + '/' + installFolder;
const status = (overrides = {}) => ({ installation: { installed: false, install_dir: installDir, parent_dir: '/mnt/机器人 data', user_home: userHome, manager_dir: '/home/user/.local/share/neh', location_editable: true, ...overrides } });
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
    assert.equal(el('installDirInput').value, '/mnt/机器人 data');
    assert.equal(el('installLocationPreview').hidden, true, 'An untouched legacy exact location has no proposed replacement');
    assert.equal(el('saveInstallDir').disabled, true);
  }
  applyStatus({ installation: { installed: false } });
  assert.equal(el('setupInstallDir').textContent, '未知', 'Do not invent missing paths');
  assert.equal(el('installDir').textContent, '未知');
  assert.equal(el('installLocationEditor').hidden, true, 'Unknown capabilities are read-only');
  state.hardwareReady = true;
  applyStatus(status());
  assert.equal(el('nextToModules').disabled, false);

  // Named defaults and legacy exact roots are never dirty until explicitly edited.
  const defaultStatus = status({ install_dir: userHome + '/' + installFolder, parent_dir: userHome });
  applyStatus(defaultStatus);
  state.hardwareReady = true;
  state.plan = { id: 'default-plan' };
  applyStatus(defaultStatus);
  assert.equal(el('installDirInput').value, userHome);
  assert.equal(el('nextToModules').disabled, false, 'Default parent must not trigger an unsaved-location gate');
  assert.equal(el('saveInstallDir').disabled, true);
  assert.equal(state.plan.id, 'default-plan');
  for (const input of ['~', '~/', '~/./nested/../', userHome + '/' + installFolder + '/']) {
    await setPath(input);
    assert.equal(el('proposedInstallDir').textContent, userHome + '/' + installFolder);
    assert.equal(el('installLocationPreview').hidden, false);
    assert.equal(el('saveInstallDir').disabled, true, 'Equivalent normalized default paths need no save');
    assert.equal(el('nextToModules').disabled, false);
  }
  for (const [input, expected] of [
    ['/', '/' + installFolder],
    ['////./../', '/' + installFolder],
    ['~/data', userHome + '/data/' + installFolder],
    ['~/data/../other//.', userHome + '/other/' + installFolder],
    ['/mnt/' + installFolder + '/', '/mnt/' + installFolder]
  ]) {
    await setPath(input);
    assert.equal(el('proposedInstallDir').textContent, expected, 'Preview appends exactly one named child after normalization');
    assert.equal(el('nextToModules').disabled, true);
    assert.equal(el('saveInstallDir').disabled, false);
  }
  for (const input of ['', 'relative/path', '~another-user']) {
    await setPath(input);
    assert.equal(el('saveInstallDir').disabled, true, 'Unresolved paths cannot be saved');
    assert.equal(el('nextToModules').disabled, true, 'Invalid drafts still require reset');
  }
  await el('resetInstallDir').click();
  applyStatus(status());
  state.hardwareReady = true;
  applyStatus(status());
  assert.equal(el('nextToModules').disabled, false, 'Untouched legacy exact roots remain usable');
  assert.equal(el('setupInstallDir').textContent, installDir);
  await setPath('/mnt/机器人 data');
  assert.equal(el('nextToModules').disabled, true, 'Explicitly editing a legacy parent proposes the named child');
  assert.equal(el('proposedInstallDir').textContent, '/mnt/机器人 data/' + installFolder);
  await el('resetInstallDir').click();
  assert.equal(el('setupInstallDir').textContent, installDir, 'Reset preserves the legacy exact root');

  // Device previews resolve symlinks and dot segments before Save or Continue can run.
  const normalPreviewResponse = previewResponse;
  const pendingPreview = deferred();
  previewResponse = () => pendingPreview.promise;
  const pendingEdit = setPath('/alias/../data');
  assert.equal(state.locationPreviewLoading, true);
  assert.equal(el('saveInstallDir').disabled, true);
  assert.equal(el('nextToModules').disabled, true);
  assert.match(el('proposedInstallDir').textContent, /正在确认/);
  const previewCalls = calls.length;
  await el('saveInstallDir').click(); await generatePlan(); await submitInstall();
  assert.equal(calls.length, previewCalls, 'Pending canonical preview cannot trigger save, plan, or installation');
  pendingPreview.resolve(json({ install_dir: '/resolved/real-data/' + installFolder, parent_dir: '/resolved/real-data', user_home: userHome }));
  await pendingEdit;
  assert.equal(el('proposedInstallDir').textContent, '/resolved/real-data/' + installFolder, 'Device symlink resolution overrides lexical /data');
  assert.equal(el('saveInstallDir').disabled, false);
  assert.equal(el('setupInstallDir').textContent, installDir, 'Preview never changes the saved root');

  const oldPreview = deferred();
  previewResponse = () => oldPreview.promise;
  const oldEdit = setPath('/older-draft');
  previewResponse = normalPreviewResponse;
  await setPath('/newer-draft');
  oldPreview.resolve(json({ install_dir: '/stale/' + installFolder, user_home: '/stale-home' }));
  await oldEdit;
  assert.equal(el('proposedInstallDir').textContent, '/newer-draft/' + installFolder, 'Late successful preview cannot overwrite newer input');
  assert.equal(state.status.installation.user_home, userHome, 'Late preview cannot overwrite the device home');
  const oldFailure = deferred();
  previewResponse = () => oldFailure.promise;
  const oldFailedEdit = setPath('/old-failing-draft');
  previewResponse = normalPreviewResponse;
  await setPath('/latest-draft');
  oldFailure.resolve(json({ error: { message: 'stale preview failure' } }, 403));
  await oldFailedEdit;
  assert.equal(el('proposedInstallDir').textContent, '/latest-draft/' + installFolder);
  assert(!el('installLocationMessage').textContent.includes('stale preview failure'));
  previewResponse = () => json({ error: { message: '无法解析设备路径' } }, 400);
  await setPath('/retry-preview');
  assert.match(el('installLocationMessage').textContent, /无法解析设备路径/);
  assert.equal(el('installDirInput').value, '/retry-preview');
  assert.equal(el('saveInstallDir').disabled, true);
  assert.equal(el('nextToModules').disabled, true);
  previewResponse = normalPreviewResponse;
  await setPath('/retry-preview');
  assert.equal(el('proposedInstallDir').textContent, '/retry-preview/' + installFolder);
  assert.equal(el('saveInstallDir').disabled, false, 'A fresh edit retries a failed preview');
  const discardedPreview = deferred();
  previewResponse = () => discardedPreview.promise;
  const discardedEdit = setPath('/discard-preview');
  await el('resetInstallDir').click();
  discardedPreview.resolve(json({ install_dir: '/discarded/' + installFolder }));
  await discardedEdit;
  previewResponse = normalPreviewResponse;
  assert.equal(el('installDirInput').value, '/mnt/机器人 data');
  assert.equal(el('installLocationPreview').hidden, true, 'Reset rejects a late typed preview');
  assert.equal(el('nextToModules').disabled, false);

  // Draft paths never change the effective root; stale plans and Continue are disabled.
  state.plan = { id: 'old-plan' };
  await setPath('/mnt/new NEH');
  assert.equal(state.plan, null);
  assert.equal(el('nextToModules').disabled, true);
  assert.equal(el('startInstall').disabled, true);
  assert.equal(el('setupInstallDir').textContent, installDir);
  assert.equal(el('installLocationPreview').hidden, false);
  assert.equal(el('proposedInstallDir').textContent, newInstallDir, 'Final child is displayed before save');
  assert.equal(el('proposedInstallDir').innerHTML, undefined, 'Previews must use textContent');
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
  pendingSave.resolve(json({ install_dir: newInstallDir, parent_dir: newParent, user_home: userHome, manager_dir: '/home/user/.local/share/neh', editable: true }));
  await firstSave;
  oldHardware.resolve(json({ hardware: { free_disk_bytes: 999999 }, supported: true }));
  await oldHardwareRequest;
  const request = calls.findLast(call => call.url === '/api/manager/install-location');
  assert.equal(calls.at(-1).url, '/api/manager/hardware', 'Successful save refreshes the selected disk');
  assert.equal(state.hardware.free_disk_bytes, 1234);
  assert.equal(request.url, '/api/manager/install-location');
  assert.equal(request.options.method, 'PUT');
  assert.equal(request.options.headers['X-NEH-CSRF'], 'fixture-csrf');
  assert.deepEqual(JSON.parse(request.options.body), { parent_dir: newParent }, 'Save sends the chosen parent; the server determines the final child');
  assert.equal(el('setupInstallDir').textContent, newInstallDir);
  assert.match(el('installLocationMessage').textContent, /Nuwax-embodied-harness/);
  assert.equal(el('installLocationPreview').hidden, true);
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  assert.equal(el('nextToModules').disabled, false);
  assert.equal(el('saveInstallDir').disabled, true, 'Saved parent is not compared directly with its child root');
  await setPath('/discard');
  await el('resetInstallDir').click();
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  assert.equal(el('nextToModules').disabled, false);

  // Browser is device-side, uses literal path labels, and selection is only a draft.
  const literal = '<img src=x onerror=alert(1)>& "目录"';
  response = () => json({ path: '/mnt', install_dir: '/mnt/' + installFolder, parent: '/', writable: true, directories: [{ name: literal, path: '/mnt/' + literal }] });
  await el('browseInstallDir').click(); await flush();
  assert.equal(calls.at(-1).url, '/api/manager/install-directories');
  assert.equal(el('installDirectoryDialog').open, true);
  assert.equal(el('directoryInstallPreview').hidden, false);
  assert.equal(el('directoryInstallDir').textContent, '/mnt/' + installFolder, 'Browser previews the child before folder selection');
  const child = el('directoryList').children[0];
  assert.equal(child.textContent, literal);
  assert.equal(child.innerHTML, undefined, 'Directory names must never be interpreted as HTML');
  const canonicalChild = '/resolved-device/' + literal + '/' + installFolder;
  response = () => json({ path: '/mnt/' + literal, install_dir: canonicalChild, parent: '/mnt', writable: true, directories: [] });
  await child.click(); await flush();
  assert.equal(calls.at(-1).url, '/api/manager/install-directories?path=' + encodeURIComponent('/mnt/' + literal));
  assert.equal(el('directoryInstallDir').textContent, canonicalChild, 'Canonical server preview wins over lexical paths');
  assert.equal(el('directoryInstallDir').innerHTML, undefined);
  const callsBeforeSelection = calls.length;
  await el('directorySelect').click();
  assert.equal(calls.length, callsBeforeSelection, 'Canonical browser selection needs no redundant preview request');
  assert.equal(el('installDirectoryDialog').open, false);
  assert.equal(el('installDirInput').value, '/mnt/' + literal);
  assert.equal(el('setupInstallDir').textContent, newInstallDir);
  assert.equal(el('proposedInstallDir').textContent, canonicalChild, 'Selected canonical preview survives closing the browser');
  assert.equal(el('nextToModules').disabled, true);
  applyStatus(status({ install_dir: newInstallDir, parent_dir: newParent }));
  assert.equal(el('proposedInstallDir').textContent, canonicalChild, 'Status refresh preserves canonical draft preview');
  await setPath('/mnt/' + literal);
  assert.equal(el('proposedInstallDir').textContent, '/mnt/' + literal + '/' + installFolder, 'Manual edits request a fresh canonical preview');
  await el('resetInstallDir').click();

  // A named-folder alias must retain its submitted spelling instead of appending a second child to its target.
  response = () => json({ path: '/payload', selection_path: '/disk/' + installFolder, install_dir: '/payload', parent: '/', writable: true, directories: [] });
  await el('browseInstallDir').click(); await flush();
  await el('directorySelect').click();
  assert.equal(el('installDirInput').value, '/disk/' + installFolder);
  assert.equal(el('proposedInstallDir').textContent, '/payload', 'Named alias selection preserves its canonical target without another child');
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
  assert.equal(el('directoryInstallPreview').hidden, true, 'Closed browser cannot expose a late preview');
  assert.equal(el('installDirInput').value, '/mnt/new NEH');
  const oldNavigation = deferred(); response = () => oldNavigation.promise;
  await el('browseInstallDir').click();
  response = () => json({ path: '/newest', install_dir: '/canonical-newest/' + installFolder, parent: '/', directories: [], writable: false });
  await loadInstallDirectories('/newest');
  oldNavigation.resolve(json({ path: '/stale', install_dir: '/stale/' + installFolder, directories: [], writable: true })); await flush();
  assert.equal(el('directoryCurrent').textContent, '/newest');
  assert.equal(el('directoryInstallDir').textContent, '/canonical-newest/' + installFolder, 'Stale navigation cannot replace the current preview');
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
  assert.equal(el('directoryInstallPreview').hidden, true, 'Failed navigation clears the prior preview');
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
  console.log('Install-location UI: canonical async parent/child previews, symlinks, tilde/root/normalization, untouched defaults/legacy roots, preview/save failures and retry, CSRF, duplicate requests, canonical browsing, stale responses, read-only state and plan-path checks passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
