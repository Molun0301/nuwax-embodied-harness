/* 使用 jsdom 检查控制台更新面板的管理程序接口与交互。 */
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const {JSDOM} = require('jsdom');
const root = path.resolve(__dirname, '..');
const dom = new JSDOM(fs.readFileSync(path.join(root, 'gateways/http/static/console.html'), 'utf8'), {url: 'http://localhost:8765/console', runScripts: 'outside-only', pretendToBeVisual: true});
const w = dom.window;
const $ = id => w.document.getElementById(id);
let timer;
let calls = [];
let writes = [];
let fail = false;
let rejectedWrite = '';
let deferred = '';
let token = 0;
let sequence = 0;
let settings = {auto_update: 'notify', check_interval_minutes: 60};
let latest = {
  installation: {revision: 'a'.repeat(40), version: '1'},
  update: {available: true, revision: 'b'.repeat(40), version: '2', check: {checked_at: new Date().toISOString()}, message: '', error: ''},
  active_job: null,
};
const jobs = {};
w.AbortSignal.timeout = () => undefined;
w.setInterval = fn => {timer = fn;};
w.HTMLElement.prototype.scrollIntoView = () => {};
w.HTMLDialogElement.prototype.showModal = function () {this.open = true;};
w.HTMLDialogElement.prototype.close = function () {this.open = false;};
$('viewSwitchConfig').addEventListener('click', () => $('config-view').classList.add('active'));
w.fetch = async (url, opts) => {
  calls.push({url, method: opts.method});
  assert(url.startsWith('/api/manager/'));
  if (fail) throw new Error('offline');
  const endpoint = url.slice('/api/manager/'.length);
  let result;
  if (opts.method === 'POST' || opts.method === 'PUT') {
    assert.equal(calls.at(-2).url, '/api/manager/session');
    assert.equal(opts.headers['X-NEH-CSRF'], 'csrf-' + token);
    assert.equal(opts.headers['Content-Type'], 'application/json');
    writes.push({url, method: opts.method, body: JSON.parse(opts.body)});
    if (rejectedWrite) return {ok: false, status: 409, json: async () => ({error: {message: rejectedWrite}})};
  }
  if (endpoint === 'session') result = {csrf_token: 'csrf-' + ++token};
  else if (endpoint === 'status') result = latest;
  else if (endpoint === 'settings' && opts.method === 'GET') result = settings;
  else if (endpoint === 'settings' && opts.method === 'PUT') result = settings = {...settings, ...JSON.parse(opts.body)};
  else if (endpoint === 'releases?refresh=1') result = {releases: []};
  else if (endpoint === 'update') {
    if (deferred) result = {status: 'deferred', message: deferred};
    else {
      result = {id: 'job-' + ++sequence, status: 'running', message: '准备版本', progress: 20};
      latest.active_job = result;
      jobs[result.id] = result;
    }
  } else if (endpoint.startsWith('jobs/')) result = jobs[endpoint.slice(5)];
  else throw new Error('未预期的接口：' + endpoint);
  return {ok: true, status: 200, json: async () => JSON.parse(JSON.stringify(result))};
};
const tick = () => new Promise(resolve => setImmediate(resolve));
(async () => {
  w.eval(fs.readFileSync(path.join(root, 'gateways/http/static/js/console-updates.js'), 'utf8'));
  await tick();
  assert.equal($('updateNewBadge').hidden, false);
  assert.equal($('updateInstall').disabled, false);
  assert($('updateVersion').textContent.includes('a'.repeat(12)));
  assert(!$('updateCheckedAt').textContent.includes('Invalid Date'));
  assert.equal(calls.some(x => x.url.endsWith('/session')), false, '读取状态不探测管理程序');
  $('updateNewBadge').click();
  assert($('config-view').classList.contains('active'));
  $('updateInstall').click();
  assert.equal($('updateDialog').open, true);
  assert.equal($('updateConfirmVersion').textContent, 'b'.repeat(12));
  $('updateCancel').click();
  assert.equal($('updateDialog').open, false);
  assert.equal(writes.length, 0);

  $('updateAutoCheck').checked = false;
  $('updateAutoCheck').dispatchEvent(new w.Event('change'));
  timer(); await tick();
  assert.equal($('updateAutoCheck').checked, false, '轮询保留未保存的设置');
  $('updateInterval').value = 1;
  $('updateSave').click(); await tick();
  assert.equal(writes.length, 0);
  $('updateInterval').value = 30;
  for (const [autoCheck, autoInstall, mode] of [[false, true, 'idle'], [true, false, 'notify'], [false, false, 'off']]) {
    $('updateAutoCheck').checked = autoCheck;
    $('updateAutoInstall').checked = autoInstall;
    $('updateSave').click(); await tick();
    assert.deepEqual(writes.at(-1), {url: '/api/manager/settings', method: 'PUT', body: {auto_update: mode, check_interval_minutes: 30}});
    assert.equal($('updateAutoCheck').checked, mode !== 'off');
    assert.equal($('updateAutoInstall').checked, mode === 'idle');
  }
  $('updateCheck').click(); $('updateCheck').click(); await tick();
  assert.equal(calls.filter(x => x.url.endsWith('/releases?refresh=1')).length, 1);

  latest.update.error = '检查版本失败'; timer(); await tick();
  assert.equal($('updateMessage').textContent, '检查版本失败');
  latest.update.error = '';
  latest.active_job = {id: 'external-job', status: 'running', message: '准备候选', progress: 35};
  timer(); await tick();
  assert.equal($('updateMessage').textContent, '正在更新：准备候选（35%）');
  for (const id of ['updateCheck', 'updateInstall', 'updateSave', 'updateAutoCheck', 'updateAutoInstall', 'updateInterval']) assert.equal($(id).disabled, true);
  latest.active_job = null; timer(); await tick();
  assert.equal(calls.some(x => x.url.includes('/jobs/')), false, '只查询本页提交的任务结果');

  deferred = '正在建图，延后更新';
  $('updateInstall').click(); $('updateConfirm').click(); await tick();
  assert.deepEqual(writes.at(-1), {url: '/api/manager/update', method: 'POST', body: {}});
  assert.equal($('updateMessage').textContent, deferred);
  assert.equal($('updateInstall').disabled, false);
  deferred = '';
  rejectedWrite = '更新请求失败';
  $('updateInstall').click(); $('updateConfirm').click(); await tick();
  assert.equal($('updateMessage').textContent, rejectedWrite);
  rejectedWrite = '';
  $('updateInstall').click(); $('updateConfirm').click(); await tick();
  assert.equal($('updateMessage').textContent, '正在更新：准备版本（20%）');
  assert.equal($('updateInstall').disabled, true);
  fail = true; timer(); await tick();
  assert.equal($('updateMessage').textContent, 'offline');
  fail = false;
  jobs['job-1'] = {id: 'job-1', status: 'succeeded', message: '已采用新版本'};
  latest.active_job = null;
  latest.update.available = false;
  latest.installation.revision = 'b'.repeat(40);
  timer(); await tick();
  assert.equal($('updateNewBadge').hidden, true);
  assert.equal($('updateMessage').textContent, '更新完成');
  assert.equal(calls.at(-1).url, '/api/manager/jobs/job-1');

  latest.update.available = true; timer(); await tick();
  $('updateInstall').click(); $('updateConfirm').click(); await tick();
  jobs['job-2'] = {id: 'job-2', status: 'failed', message: '构建失败'};
  latest.active_job = null; timer(); await tick();
  assert.equal($('updateMessage').textContent, '构建失败');
  assert.equal($('updateMessage').dataset.state, 'failed');
  assert.equal($('updateInstall').disabled, false);
  assert.equal(calls.at(-1).url, '/api/manager/jobs/job-2');
  rejectedWrite = '保存设置失败';
  $('updateSave').click(); await tick();
  assert.equal($('updateMessage').textContent, rejectedWrite);
  console.log('控制台更新：管理程序接口、设置、任务结果与交互检查通过');
  dom.window.close();
})().catch(error => {console.error(error); dom.window.close(); process.exitCode = 1;});
