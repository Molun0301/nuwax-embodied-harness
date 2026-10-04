#!/usr/bin/env node
/* Offline regression of the production navigation.js IIFE, with virtual time and
 * controllable fetch/body completion. No network, DOM packages, or robot calls.
 *   node scripts/check_navigation_polling.cjs
 *   node scripts/check_navigation_polling.cjs --source /tmp/navigation-before.js --measure-only
 * Timing measurements below are deterministic load-model results, not browser
 * rendering benchmarks or real gateway/robot latency measurements.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const args = process.argv.slice(2);
const sourceIndex = args.indexOf('--source');
const sourcePath = sourceIndex >= 0 ? path.resolve(args[sourceIndex + 1])
  : path.join(root, 'gateways/http/static/js/navigation.js');
const source = fs.readFileSync(sourcePath, 'utf8');
const measureOnly = args.includes('--measure-only');

function eventTarget(properties = {}) {
  const listeners = new Map();
  return Object.assign(properties, {
    addEventListener(type, listener) {
      if (!listeners.has(type)) listeners.set(type, []);
      listeners.get(type).push(listener);
    },
    dispatch(type, event = {}) {
      for (const listener of listeners.get(type) || []) listener(event);
    },
  });
}
const flush = () => new Promise(resolve => setImmediate(resolve));

function harness({latency = null, abortMode = 'reject'} = {}) {
  let now = 0, nextTimer = 1, nextTask = 1, activeReads = 0, maxActiveReads = 0;
  const timers = new Map(), requests = [], renders = [], previews = [];
  const timer = (callback, delay, repeat = false) => {
    const id = nextTimer++;
    timers.set(id, {at: now + Math.max(0, delay || 0), delay, callback, repeat});
    return id;
  };
  const elements = {};
  for (const role of ['nav-status', 'nav-detail', 'nav-submit', 'nav-cancel', 'nav-height']) {
    let value = '';
    elements[role] = eventTarget({dataset: {}, value: '', validity: {badInput: false}});
    Object.defineProperty(elements[role], 'textContent', {
      get: () => value,
      set: text => {value = text; renders.push({at: now, role, text});},
    });
  }
  const container = eventTarget({hidden: false, querySelector: selector => elements[selector.match(/"([^"]+)"/)[1]]});
  const map = eventTarget();
  const document = eventTarget({readyState: 'complete', visibilityState: 'visible', hidden: false,
    getElementById: id => id === 'navigation-container' ? container : id === 'mapCanvas' ? map : null});
  let wsListener;
  const window = eventTarget({
    __nuwaxNavigationActive: true,
    __nuwaxActiveMapName: () => 'generic-map',
    __nuwaxMapViewMode: () => '2d',
    __nuwaxResolveNavigationHeight: () => 0.6,
    __nuwaxWsSubscribe: (_topic, callback) => {wsListener = callback; return 'test-subscription';},
    pointcloud3d: {previewNavigationPoint: pose => previews.push(pose)},
  });
  function fetch(url, options = {}) {
    const isRead = url.startsWith('/api/tasks/') && !options.method;
    const request = {url, options, isRead, startedAt: now, settled: false, aborted: false};
    requests.push(request);
    if (isRead) {activeReads++; maxActiveReads = Math.max(maxActiveReads, activeReads);}
    const promise = new Promise((resolve, reject) => {
      function finish() {
        if (request.settled) return false;
        request.settled = true;
        request.finishedAt = now;
        if (isRead) activeReads--;
        if (request.responseTimer) timers.delete(request.responseTimer);
        return true;
      }
      request.resolve = (payload, {bodyDelay = 0, ok = true, status = 200} = {}) => {
        if (request.headersSent || request.settled) return;
        request.headersSent = true;
        resolve({ok, status, json: () => new Promise((bodyResolve, bodyReject) => {
          request.rejectBody = error => {if (finish()) bodyReject(error);};
          const body = () => {if (finish()) bodyResolve(payload);};
          if (bodyDelay) request.responseTimer = timer(body, bodyDelay); else body();
        })});
      };
      request.reject = error => {
        if (request.headersSent) request.rejectBody?.(error);
        else if (finish()) reject(error);
      };
    });
    request.acknowledgeAbort = () => {
      const error = new Error('The operation was aborted'); error.name = 'AbortError';
      request.reject(error);
    };
    options.signal?.addEventListener('abort', () => {
      request.aborted = true;
      request.abortedAt = now;
      if (abortMode === 'reject') request.acknowledgeAbort();
    }, {once: true});
    if (url.includes('/navigate_to_pose/')) {
      request.resolve({task_id: 'task-' + nextTask++});
    } else if (isRead && latency !== null) {
      request.responseTimer = timer(() => request.resolve(snapshot('running', 'sample@' + request.startedAt)), latency);
    }
    return promise;
  }
  const context = vm.createContext({window, document, fetch, AbortController, console,
    performance: {now: () => now},
    setTimeout: (fn, delay) => timer(fn, delay), clearTimeout: id => timers.delete(id),
    setInterval: (fn, delay) => timer(fn, delay, true), clearInterval: id => timers.delete(id),
  });
  vm.runInContext(source, context, {filename: sourcePath});
  const module = window.__nuwaxNavigationModule;
  module.setActive(true);
  async function advance(milliseconds) {
    const target = now + milliseconds;
    await flush();
    let executed = 0;
    while (true) {
      const entry = [...timers].filter(([, value]) => value.at <= target)
        .sort((a, b) => a[1].at - b[1].at || a[0] - b[0])[0];
      if (!entry) break;
      assert(++executed < 10000, 'timer loop must not spin');
      const [id, value] = entry;
      now = value.at;
      if (value.repeat) value.at += value.delay; else timers.delete(id);
      value.callback();
      await flush();
    }
    now = target;
    await flush();
  }
  async function submit() {
    map.dispatch('nuwax:map-click', {detail: {x: 1, y: 2, frame_id: 'map'}});
    module.submit();
    await flush();
  }
  function active(value) {window.__nuwaxNavigationActive = value; module.setActive(value);}
  function visibility(value) {
    document.visibilityState = value; document.hidden = value === 'hidden';
    document.dispatch('visibilitychange');
  }
  return {module, window, document, elements, requests, renders, previews, advance, submit, active, visibility,
    reads: () => requests.filter(request => request.isRead),
    pending: () => requests.filter(request => request.isRead && !request.settled),
    status: () => elements['nav-status'].textContent,
    detail: () => elements['nav-detail'].textContent,
    ws: data => wsListener?.({data: {state: data}}),
    metrics: () => ({now_ms: now, max_concurrent_reads: maxActiveReads, active_reads: activeReads,
      reads_started: requests.filter(request => request.isRead).length,
      reads_completed: requests.filter(request => request.isRead && request.settled && !request.aborted).length,
      read_start_times_ms: requests.filter(request => request.isRead).map(request => request.startedAt)}),
    timerCount: () => timers.size,
  };
}
function snapshot(state = 'running', message = '正在导航') {return {status: {state, message}};}

async function measureSlowReads() {
  const h = harness({latency: 1200});
  await h.submit();
  await h.advance(5000);
  const result = {scenario: '1200ms task responses, 500ms target cadence, 5000ms observation', ...h.metrics()};
  h.window.dispatch('pagehide');
  await flush();
  return result;
}

async function measureOutOfOrderReads() {
  const h = harness(); await h.submit(); await h.advance(1000);
  const reads = h.pending();
  let staleOverwrite = false;
  if (reads.length > 1) {
    reads.at(-1).resolve(snapshot('running', 'newer snapshot')); await flush();
    reads[0].resolve(snapshot('running', 'older snapshot')); await flush();
    staleOverwrite = h.status().includes('older snapshot');
  }
  h.window.dispatch('pagehide'); await flush();
  return {scenario: 'older response delivered after newer same-task response',
    simultaneous_reads: reads.length, stale_overwrite: staleOverwrite};
}

async function runChecks() {
  const checks = [];
  async function check(name, run) {await run(); checks.push(name);}
  await check('slow reads are single-flight with immediate catch-up', async () => {
    const h = harness({latency: 1200}); await h.submit(); await h.advance(5000);
    assert.equal(h.metrics().max_concurrent_reads, 1);
    assert.deepEqual(h.metrics().read_start_times_ms, [0, 1200, 2400, 3600, 4800]);
    assert(h.detail().includes('sample@3600'));
    h.window.dispatch('pagehide'); await flush();
    assert.equal(h.pending().length, 0); assert.equal(h.timerCount(), 0);
  });
  await check('fast reads retain 2Hz start cadence and full terminal rendering', async () => {
    const h = harness({latency: 100}); await h.submit(); await h.advance(2100);
    assert.deepEqual(h.metrics().read_start_times_ms, [0, 500, 1000, 1500, 2000]);
    h.window.dispatch('pagehide'); await flush();
    for (const terminal of ['succeeded', 'failed', 'cancelled', 'timeout']) {
      const t = harness(); await t.submit(); await t.advance(0);
      t.pending()[0].resolve(snapshot(terminal)); await flush(); await t.advance(3000);
      assert.equal(t.module.getState().inflightRequestId, null);
      assert.equal(t.reads().length, 1); assert.equal(t.timerCount(), 0);
      assert(t.detail().includes(terminal));
      assert.equal(t.previews.at(-1), null);
    }
  });
  await check('network, HTTP, and body failures retry without wedging', async () => {
    const h = harness(); await h.submit(); await h.advance(0);
    h.pending()[0].reject(new Error('offline')); await flush();
    assert(h.status().includes('offline')); await h.advance(500);
    h.pending()[0].resolve({detail: 'temporarily unavailable'}, {ok: false, status: 503}); await flush();
    assert(h.status().includes('temporarily unavailable')); await h.advance(500);
    h.pending()[0].resolve(snapshot(), {bodyDelay: 3000}); await flush();
    h.pending()[0].reject(new SyntaxError('invalid JSON')); await flush(); await h.advance(500);
    h.pending()[0].resolve(snapshot('running', 'recovered')); await flush();
    assert(h.status().includes('recovered')); assert.equal(h.metrics().max_concurrent_reads, 1);
    h.window.dispatch('pagehide'); await flush(); assert.equal(h.timerCount(), 0);
  });
  await check('hung fetch and hung JSON body abort at deadline and recover', async () => {
    for (const body of [false, true]) {
      const h = harness(); await h.submit(); await h.advance(0); const stalled = h.pending()[0];
      if (body) {stalled.resolve(snapshot('running', 'stale body'), {bodyDelay: 30000}); await flush();}
      await h.advance(10000);
      assert.equal(stalled.aborted, true); assert.equal(stalled.abortedAt, 10000);
      assert.equal(h.pending().length, 1); assert.notEqual(h.pending()[0], stalled);
      assert(h.status().includes('超时')); h.pending()[0].resolve(snapshot('running', 'recovered'));
      await flush(); assert(h.status().includes('recovered'));
      assert.equal(h.metrics().max_concurrent_reads, 1);
      h.window.dispatch('pagehide'); await flush(); assert.equal(h.timerCount(), 0);
    }
  });
  await check('tab hide/show fences late success and failure, retaining task identity', async () => {
    for (const rejected of [false, true]) {
      const h = harness({abortMode: 'defer'}); await h.submit(); await h.advance(0);
      const old = h.pending()[0]; h.active(false); await h.advance(3000);
      assert.equal(old.aborted, true); assert.equal(h.reads().length, 1);
      assert.equal(h.module.getState().inflightRequestId, 'task-1');
      h.active(true); h.active(true); await h.advance(0);
      assert.equal(h.reads().length, 1, 'restart waits for actual fetch settlement');
      const status = h.status();
      if (rejected) old.reject(new Error('stale failure'));
      else old.resolve(snapshot('succeeded', 'stale success'));
      await flush(); assert.equal(h.status(), status);
      await h.advance(0); assert.equal(h.pending().length, 1);
      assert.notEqual(h.pending()[0], old); assert.equal(h.metrics().max_concurrent_reads, 1);
      h.pending()[0].resolve(snapshot('running', 'current session')); await flush();
      assert(h.status().includes('current session'));
      h.window.dispatch('pagehide'); await flush(); assert.equal(h.timerCount(), 0);
    }
  });
  await check('browser visibility and pagehide/pageshow pause and resume safely', async () => {
    const h = harness(); await h.submit(); await h.advance(0);
    h.visibility('hidden'); await flush(); await h.advance(5000);
    assert.equal(h.pending().length, 0); assert.equal(h.timerCount(), 0);
    h.visibility('visible'); await h.advance(0); assert.equal(h.pending().length, 1);
    h.window.dispatch('pagehide'); await flush(); await h.advance(5000);
    assert.equal(h.timerCount(), 0); assert.equal(h.pending().length, 0);
    h.visibility('hidden'); h.visibility('visible'); await h.advance(0);
    assert.equal(h.pending().length, 0, 'visibility cannot restart a suspended page');
    h.window.dispatch('pageshow'); await h.advance(0); assert.equal(h.pending().length, 1);
    h.pending()[0].resolve(snapshot('succeeded')); await flush();
    h.window.dispatch('pageshow'); h.active(false); h.active(true); await h.advance(5000);
    assert.equal(h.pending().length, 0); assert.equal(h.timerCount(), 0);
    assert.equal(h.metrics().max_concurrent_reads, 1);
  });
  await check('repeated cancel/start does not abort a healthy poll or multiply timers', async () => {
    const h = harness(); await h.submit(); await h.advance(0); const read = h.pending()[0];
    for (let index = 0; index < 20; index++) {
      h.module.cancel(); const command = h.requests.at(-1);
      assert(command.url.endsWith('/cancel')); assert.equal(command.options.signal, undefined);
      command.resolve({cancelled: true}); await flush(); h.active(true);
    }
    await h.advance(1000); assert.equal(h.reads().length, 1); assert.equal(read.aborted, false);
    read.resolve(snapshot('cancelled')); await flush(); await h.advance(5000);
    assert.equal(h.timerCount(), 0); assert.equal(h.module.getState().inflightRequestId, null);
  });
  await check('50 rapid stop/start cycles preserve one owner and final cleanup', async () => {
    const h = harness(); await h.submit(); await h.advance(0);
    for (let index = 0; index < 50; index++) {
      const old = h.pending()[0]; h.active(false); h.active(true); h.active(true);
      await h.advance(0);
      assert.equal(old.aborted, true); assert.equal(h.pending().length, 1);
      assert.notEqual(h.pending()[0], old);
      assert.equal(h.module.getState().pollRequest.controller.signal.aborted, false);
    }
    assert.equal(h.metrics().max_concurrent_reads, 1);
    h.active(false); await flush(); await h.advance(20000);
    assert.equal(h.pending().length, 0); assert.equal(h.timerCount(), 0);
    assert.equal(h.module.getState().pollRequest, null);
    h.active(true); await h.advance(0); h.pending()[0].resolve(snapshot('succeeded'));
    await flush(); assert.equal(h.timerCount(), 0);
  });
  await check('accepted submission while hidden waits for reopening without cancellation', async () => {
    const h = harness();
    const accepted = h.submit(); h.active(false); await accepted; await h.advance(5000);
    assert.equal(h.module.getState().inflightRequestId, 'task-1');
    assert.equal(h.reads().length, 0); assert.equal(h.timerCount(), 0);
    assert.equal(h.requests.filter(request => request.options.method === 'POST').length, 1);
    h.active(true); await h.advance(0); assert.equal(h.pending().length, 1);
    h.pending()[0].resolve(snapshot('succeeded')); await flush(); assert.equal(h.timerCount(), 0);
  });
  await check('old task and cancel responses cannot overwrite a newer task', async () => {
    for (const failOldCancel of [false, true]) {
      const h = harness(); await h.submit(); await h.advance(0);
      h.module.cancel(); const oldCancel = h.requests.at(-1);
      h.pending()[0].resolve(snapshot('succeeded')); await flush();
      await h.submit(); await h.advance(0); const current = h.pending()[0];
      const status = h.status();
      if (failOldCancel) oldCancel.reject(new Error('old task failed'));
      else oldCancel.resolve({cancelled: true});
      await flush(); assert.equal(h.status(), status); assert.equal(current.aborted, false);
      current.resolve(snapshot('running', 'new task')); await flush(); assert(h.status().includes('new task'));
      h.window.dispatch('pagehide'); await flush(); assert.equal(h.timerCount(), 0);
    }
    // The public state accessor also permits an owner to replace task identity
    // before an outstanding request settles. Exercise that fence explicitly.
    const h = harness({abortMode: 'defer'}); await h.submit(); await h.advance(0);
    const old = h.pending()[0]; h.active(false); h.module.getState().inflightRequestId = null;
    h.active(true); await h.submit(); old.reject(new Error('late old read'));
    await flush(); assert(!h.status().includes('late old read')); await h.advance(0);
    assert(h.pending()[0].url.endsWith('/task-2')); assert.equal(h.metrics().max_concurrent_reads, 1);
    h.window.dispatch('pagehide'); h.pending()[0].acknowledgeAbort(); await flush();
    assert.equal(h.timerCount(), 0);
  });
  await check('late HTTP detail does not replace fresher optional stream detail', async () => {
    const h = harness(); await h.submit(); await h.advance(0);
    h.ws({status: 'running', message: 'fresh streamed detail'});
    h.pending()[0].resolve(snapshot('running', 'older task detail')); await flush();
    assert(h.detail().includes('fresh streamed detail'));
    await h.advance(500); h.pending()[0].resolve(snapshot('succeeded', 'confirmed terminal')); await flush();
    assert(h.detail().includes('confirmed terminal'));
    h.window.dispatch('pagehide'); const detail = h.detail();
    h.ws({message: 'after teardown'}); assert.equal(h.detail(), detail); assert.equal(h.timerCount(), 0);
  });
  return checks;
}
(async () => {
  const result = {kind: 'deterministic production-module polling regression', source: sourcePath,
    node: process.version, measurement: await measureSlowReads(), ordering: await measureOutOfOrderReads()};
  if (!measureOnly) result.passed_checks = await runChecks();
  console.log(JSON.stringify(result, null, 2));
})().catch(error => {console.error(error); process.exitCode = 1;});
