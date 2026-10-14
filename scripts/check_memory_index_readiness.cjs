#!/usr/bin/env node
/* 用内置 Node VM 运行实际控制台记忆函数；合成 DOM、可控请求与虚拟时钟。
 * 无网络、额外依赖或机器人调用。此检查不等同真实浏览器布局/后端性能测试。
 * node scripts/check_memory_index_readiness.cjs
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const sourcePath = path.resolve(__dirname, '../gateways/http/static/js/console.js');
const source = fs.readFileSync(sourcePath, 'utf8');
function between(start, end) {
  const a = source.indexOf(start), b = source.indexOf(end, a);
  assert(a >= 0 && b > a, `production boundaries exist: ${start}`);
  return source.slice(a, b);
}
const stateSource = between('const state = {', 'const els = {');
const functionsSource = between('function loadSemanticInstances()', 'function semanticListRow(');
const lifecycleSource = between('window.addEventListener("stream:map.frame", () => {', '      loadSemanticInstances();\n      connectStreams();');
const flush = () => new Promise(resolve => setImmediate(resolve));
function element() {
  return {textContent: '', dataset: {}, attributes: {}, disabled: false, children: [],
    setAttribute(key, value) {this.attributes[key] = value;},
    removeAttribute(key) {delete this.attributes[key];},
    insertAdjacentElement(position, child) {assert.equal(position, 'afterend'); this.children.push(child);},
    set innerHTML(_) {throw new Error('Never render server values as HTML');}};
}
function harness({abortMode = 'reject'} = {}) {
  let now = Date.UTC(2026, 9, 1, 12), map = 'map-a', nextTimer = 1, listRenders = 0;
  let activeSummary = 0, maxSummary = 0;
  const timers = new Map(), requests = [], events = [], listeners = new Map();
  const els = Object.fromEntries(['semanticMemoryStatus', 'semanticMemorySummary', 'semanticMemoryCount', 'semanticInstancesRefresh']
    .map(key => [key, element()]));
  const window = {
    setTimeout(fn, delay) {const id = nextTimer++; timers.set(id, {at: now + delay, fn}); return id;},
    clearTimeout(id) {timers.delete(id);},
    dispatchEvent(event) {events.push(event);},
    addEventListener(type, listener) {listeners.set(type, listener);},
  };
  const document = {hidden: false, createElement: element};
  function fetch(url, options) {
    const summary = url === '/api/memory/summary';
    assert(summary || url === '/api/memory/instances?limit=500');
    if (summary) {activeSummary++; maxSummary = Math.max(maxSummary, activeSummary);}
    const request = {url, options, summary, settled: false, headers: false};
    requests.push(request);
    function finish() {if (request.settled) return false; request.settled = true; if (summary) activeSummary--; return true;}
    return new Promise((resolve, reject) => {
      request.resolve = (payload, {ok = true, status = 200, deferBody = false} = {}) => {
        request.headers = true;
        resolve({ok, status, json: () => new Promise((bodyResolve, bodyReject) => {
          request.rejectBody = error => {if (finish()) bodyReject(error);};
          request.resolveBody = () => {if (finish()) bodyResolve(payload);};
          if (!deferBody) request.resolveBody();
        })});
        if (!ok) finish();
      };
      request.reject = error => request.headers ? request.rejectBody?.(error) : finish() && reject(error);
      request.acknowledgeAbort = () => {const error = new Error('Aborted'); error.name = 'AbortError'; request.reject(error);};
      options.signal.addEventListener('abort', () => {
        request.aborted = true;
        if (abortMode === 'reject') request.acknowledgeAbort();
      }, {once: true});
    });
  }
  class FakeDate extends Date {constructor(value = now) {super(value);} static now() {return now;}}
  const context = vm.createContext({window, document, els, fetch, AbortController, Date: FakeDate,
    PLANAR_PIPELINE: false, POSE_STREAM_TOPICS: [], LOG_STREAM_TOPICS: [], VIDEO_STREAM_INITIAL_RECONNECT_MS: 500,
    CustomEvent: class {constructor(type, options) {this.type = type; this.detail = options.detail;}},
    activeMapName: () => map, isFallbackInstance: () => false, isUnknownFrame: () => false,
    setText: (el, text) => {if (el) el.textContent = text;},
    reconcileSemanticOverlay: () => {}, requestMapDraw: () => {},
    renderSemanticMemoryList: () => {listRenders++;},
  });
  vm.runInContext(`${stateSource}\n${functionsSource}\n${lifecycleSource}\nglobalThis.test = {
    state, loadSemanticInstances, applySemanticInstances, renderSemanticMemoryStatus,
    loadSemanticIndexSummary, semanticMapInstances, syncSemanticIndexScope
  };`, context, {filename: sourcePath});
  const test = context.test;
  async function advance(ms) {
    now += ms;
    for (const [id, timer] of [...timers]) if (timer.at <= now) {timers.delete(id); timer.fn();}
    await flush();
  }
  function stream(payload) {test.state.semanticStreamVersion++; test.applySemanticInstances(payload);}
  return {...test, els, requests, events, stream, advance,
    setMap(value) {map = value; listeners.get('stream:map.frame')();},
    lifecycle(name) {listeners.get(name)?.();},
    summaries: () => requests.filter(item => item.summary),
    indexText: () => els.semanticMemoryIndexStatus?.textContent || '',
    indexState: () => els.semanticMemoryIndexStatus?.dataset.state,
    listRenders: () => listRenders, timerCount: () => timers.size,
    maxSummary: () => maxSummary,
  };
}
function instances(library = 'library-a', map = 'map-a', extra = {}) {
  return {library_name: library, active_library_name: library, map_name: map, ...extra,
    instances: [{instance_id: 'object', map_name: map, pose: {x: 1, y: 2, z: 0}, ...extra.record}]};
}
function summary(library = 'library-a', changes = {}) {
  return {memory_summary: {metadata: {active_library_name: library, memory_enabled: true,
    vector_provenance_counts: {validated_current: 2, legacy_unverified: 1, known_stale_or_incompatible: 3,
      missing: 4, pending: 5, failed: 6, disabled: 7, not_applicable: 8, unavailable: 9},
    indexing: {durable: true, active_library: library, activation_generation: 1,
      work: [{status: 'backoff', error_category: 'temporary:synthetic', count: 2},
        {status: 'backoff', error_category: 'temporary:other', count: 3},
        {status: 'failed_terminal', error_category: 'permanent:legacy_archive_capability_required', count: 1}],
      reconciliation_pending: 5, cache_cleanup_pending: 4}, ...changes}}};
}
async function populated(options) {
  const h = harness(options);
  h.stream(instances());
  h.summaries()[0].resolve(summary());
  await flush();
  return h;
}
async function main() {
  const passed = [];
  const check = async (name, fn) => {await fn(); passed.push(name);};
  await check('源记录先显示；慢摘要不阻塞地图或刷新按钮；同类请求合并', async () => {
    const h = harness();
    const promise = h.loadSemanticInstances();
    assert.equal(h.loadSemanticInstances(), promise);
    h.requests[0].resolve(instances()); await promise;
    assert.equal(h.state.semanticInstances.length, 1);
    assert.equal(h.semanticMapInstances().length, 1);
    assert.equal(h.els.semanticInstancesRefresh.disabled, false);
    assert.equal(h.els.semanticMemoryStatus.dataset.state, 'ready');
    assert.equal(h.summaries().length, 1);
    assert.match(h.indexText(), /索引摘要读取中/);
    const indexPromise = h.loadSemanticIndexSummary();
    assert.equal(h.loadSemanticIndexSummary(), indexPromise);
    h.summaries()[0].resolve(summary()); await indexPromise;
    assert.equal(h.maxSummary(), 1); assert.equal(h.timerCount(), 0);
  });
  await check('正确区分向量、模态、源记录、工作记录和全库清理计数', async () => {
    const h = await populated(), text = h.indexText();
    assert.match(text, /向量：已验证 2，历史未验证 1，已排除过时\/不兼容 3/);
    assert.match(text, /模态：待处理 5，失败 6，不可用 9，禁用 7，不适用 8/);
    assert.match(text, /无可用向量的记录 4/);
    assert.match(text, /后台任务（记录）：待处理 0，退避 5，计算中 0，完成 0，复用 0，终止失败 1/);
    assert.match(text, /索引核对尚未完成/); assert.match(text, /全部记忆库缓存清理待办 4/);
    assert.match(text, /legacy_archive_capability_required/);
    assert.match(text, /历史向量可兼容读取，尚未验证来源/);
    assert.match(text, /仅影响相应向量；记录仍可查看/);
    assert.match(h.els.semanticMemorySummary.textContent, /^1 条记录 · 1 个有位置/);
    assert.equal(h.els.semanticMemoryIndexStatus.dataset.level, 'warn');
    assert.equal(h.semanticMapInstances().length, 1);
  });
  await check('无元数据和旧服务保持未知，不推断为零或全局就绪', async () => {
    for (const body of [{}, {memory_summary: {}}, summary('library-a', {vector_provenance_counts: null, indexing: null})]) {
      const h = harness(); h.stream(instances()); h.summaries()[0].resolve(body); await flush();
      assert.match(h.indexText(), /索引状态未知/);
      assert.doesNotMatch(h.indexText(), /已验证 0|失败 0|无可用向量的记录 0/);
      assert.equal(h.indexState(), 'unknown'); assert.equal(h.state.semanticError, '');
    }
  });
  await check('部分或无效计数保持未知，明确的零仍显示零', async () => {
    const h = harness(); h.stream(instances());
    h.summaries()[0].resolve(summary('library-a', {vector_provenance_counts: {validated_current: 0, failed: null,
      pending: '0', unavailable: -1, legacy_unverified: Infinity}, indexing: {active_library: 'library-a'}}));
    await flush();
    assert.match(h.indexText(), /已验证 0，历史未验证 —/);
    assert.match(h.indexText(), /模态：待处理 —，失败 —，不可用 —/);
    assert.match(h.indexText(), /后台任务（记录）：待处理 —/);
    assert.match(h.indexText(), /索引核对状态未知/);
    assert.doesNotMatch(h.indexText(), /NaN|Infinity|undefined|null/);
    const promise = h.loadSemanticIndexSummary();
    h.summaries()[1].resolve(summary('library-a', {indexing: {work: [null, {status: 'failed_terminal', count: '1'}]}}));
    await promise; assert.match(h.indexText(), /终止失败 —/);
  });
  await check('错误文本和库名仅按文本渲染；节点不重复创建', async () => {
    const h = harness();
    const name = '<img src=x onerror=alert(1)>';
    h.stream(instances(name));
    const body = summary(name); body.memory_summary.metadata.indexing.work[0].error_category = name;
    h.summaries()[0].resolve(body); await flush();
    assert(h.els.semanticMemoryStatus.textContent.includes(name)); assert(h.indexText().includes(name));
    h.renderSemanticMemoryStatus(); h.renderSemanticMemoryStatus();
    assert.equal(h.els.semanticMemorySummary.children.length, 1);
    const p = h.loadSemanticIndexSummary(); h.summaries()[1].reject(new Error(name)); await p;
    assert(h.indexText().includes(name)); assert.equal(h.state.semanticError, '');
  });
  await check('摘要刷新失败保留并标记上次失败计数，源记录仍可查阅', async () => {
    const h = await populated(); const p = h.loadSemanticIndexSummary();
    assert.match(h.indexText(), /以下为上次摘要/);
    h.summaries()[1].resolve({}, {ok: false, status: 503}); await p;
    assert.match(h.indexText(), /索引摘要读取失败：服务返回 503/);
    assert.match(h.indexText(), /以下为上次摘要/); assert.match(h.indexText(), /失败 6/);
    assert.equal(h.indexState(), 'error'); assert.equal(h.state.semanticError, '');
    assert.equal(h.semanticMapInstances().length, 1);
    assert.equal(h.els.semanticMemoryStatus.dataset.state, 'ready');
    assert.equal(h.els.semanticInstancesRefresh.disabled, false);
  });
  await check('摘要超时独立处理且重试恢复；超时后不接受延迟响应', async () => {
    for (const abortMode of ['reject', 'ignore']) {
      const h = harness({abortMode}); h.stream(instances()); await h.advance(8000);
      if (abortMode === 'ignore') {h.summaries()[0].resolve(summary()); await flush();}
      assert.match(h.indexText(), /读取超时/); assert.equal(h.semanticMapInstances().length, 1);
      const p = h.loadSemanticIndexSummary(); h.summaries()[1].resolve(summary()); await p;
      assert.doesNotMatch(h.indexText(), /读取失败|读取超时/); assert.equal(h.timerCount(), 0);
    }
  });
  await check('较新流记录使摘要过时；重复流不产生数据库查询', async () => {
    const h = await populated(); const renders = h.listRenders();
    h.stream(instances()); assert.equal(h.listRenders(), renders); assert.doesNotMatch(h.indexText(), /源记录已更新/);
    h.stream(instances('library-a', 'map-a', {record: {last_seen_ts: 'new'}}));
    assert.equal(h.indexState(), 'stale'); assert.match(h.indexText(), /查询于/);
    assert.match(h.indexText(), /源记录已更新或不可用，以下为上次摘要/);
    for (let n = 0; n < 20; n++) h.stream(instances('library-a', 'map-a', {record: {last_seen_ts: n}}));
    assert.equal(h.summaries().length, 1);
    const p = h.loadSemanticIndexSummary(); h.summaries()[1].resolve(summary()); await p;
    assert.notEqual(h.indexState(), 'stale');
  });
  await check('请求期间源记录变化也标记摘要过时', async () => {
    const h = harness(); h.stream(instances());
    h.stream(instances('library-a', 'map-a', {record: {last_seen_ts: 'new'}}));
    h.summaries()[0].resolve(summary()); await flush();
    assert.equal(h.indexState(), 'stale'); assert.equal(h.summaries().length, 1);
  });
  await check('手动刷新在较旧摘要未完成时合并一次补读，不并发查询', async () => {
    const h = harness(); h.stream(instances());
    const p = h.loadSemanticInstances();
    h.requests.find(item => !item.summary).resolve(instances('library-a', 'map-a', {record: {last_seen_ts: 'new'}}));
    await p; assert.equal(h.summaries().length, 1);
    h.summaries()[0].resolve(summary()); await flush();
    assert.equal(h.summaries().length, 2); assert.equal(h.maxSummary(), 1);
    h.summaries()[1].resolve(summary()); await flush();
    assert.notEqual(h.indexState(), 'stale'); assert.equal(h.timerCount(), 0);
  });
  await check('A→B→A 切库取消并隔离迟到请求，等待旧读取结束后只发最新请求', async () => {
    const h = harness({abortMode: 'ignore'}); h.stream(instances());
    h.stream(instances('library-b')); h.stream(instances());
    assert.equal(h.summaries().length, 1); assert.equal(h.summaries()[0].aborted, true);
    h.summaries()[0].resolve(summary('library-a', {vector_provenance_counts: {failed: 999}})); await flush();
    assert.doesNotMatch(h.indexText(), /999/); assert.equal(h.summaries().length, 2);
    h.summaries()[1].resolve(summary()); await flush();
    assert.match(h.indexText(), /失败 6/); assert.equal(h.maxSummary(), 1);
  });
  await check('响应体延迟和地图切换同样受代际隔离；不从另一地图借计数', async () => {
    const h = harness({abortMode: 'ignore'}); h.stream(instances());
    h.summaries()[0].resolve(summary('library-a', {vector_provenance_counts: {failed: 999}}), {deferBody: true}); await flush();
    h.setMap('map-b'); assert.doesNotMatch(h.indexText(), /999|失败 6/);
    h.stream(instances('library-b', 'map-b'));
    h.summaries()[0].resolveBody(); await flush();
    assert.equal(h.summaries().length, 2); assert.doesNotMatch(h.indexText(), /999/);
    h.summaries()[1].resolve(summary('library-b')); await flush(); assert.match(h.indexText(), /失败 6/);
    assert.equal(h.semanticMapInstances().length, 1); assert.equal(h.maxSummary(), 1);
  });
  await check('无范围或冲突范围的摘要不发布计数', async () => {
    for (const body of [summary('library-b'), summary('library-a', {indexing: {active_library: 'library-b'}}),
      summary('library-a', {active_library_name: undefined})]) {
      const h = harness(); h.stream(instances()); h.summaries()[0].resolve(body); await flush();
      assert.match(h.indexText(), /范围不匹配/); assert.doesNotMatch(h.indexText(), /失败 6/);
      assert.equal(h.semanticMapInstances().length, 1);
    }
  });
  await check('HTTP 旧源响应不能覆盖更新的流；手动刷新仍更新摘要', async () => {
    const h = await populated(); const p = h.loadSemanticInstances();
    h.stream(instances('library-b'));
    h.requests.find(item => !item.summary).resolve(instances()); await p;
    assert.equal(h.state.semanticMeta.library_name, 'library-b');
    assert.equal(h.summaries().length, 2);
    h.summaries()[1].resolve(summary('library-b')); await flush();
    assert.equal(h.state.semanticIndexMeta.active_library_name, 'library-b');
  });
  await check('刷新源失败和断流原状态保留，地图隐藏与摘要错误各自独立', async () => {
    const h = await populated(); const p = h.loadSemanticInstances();
    h.requests.find(item => !item.summary).reject(new Error('synthetic source failure')); await p;
    assert.equal(h.state.semanticInstances.length, 1); assert.equal(h.semanticMapInstances().length, 0);
    assert.match(h.els.semanticMemoryStatus.textContent, /刷新失败.*正在查看上次缓存，地图标记已隐藏/);
    assert.equal(h.els.semanticInstancesRefresh.textContent, '重试刷新');
    assert.equal(h.summaries().length, 1); assert.equal(h.indexState(), 'stale');
    h.stream({message: '实时连接已断开，正在重连'});
    assert.equal(h.semanticMapInstances().length, 0); assert.match(h.indexText(), /上次摘要/);
  });
  await check('未启用库不请求；空库、地图不匹配和截断计数保持原含义', async () => {
    const h = harness(); h.stream({instances: [], library_name: null});
    assert.equal(h.summaries().length, 0); assert.equal(h.indexText(), '');
    assert.match(h.els.semanticMemoryStatus.textContent, /未启用记忆库/);
    h.stream(instances('library-a', 'other-map', {truncated: true, total: 501}));
    assert.match(h.els.semanticMemoryStatus.textContent, /与当前地图不匹配/);
    assert.match(h.els.semanticMemorySummary.textContent, /已载入 1\/501 条/);
    assert.equal(h.semanticMapInstances().length, 0);
    h.summaries()[0].resolve(summary()); await flush();
    h.stream({instances: [], library_name: 'library-a'});
    assert.match(h.els.semanticMemorySummary.textContent, /^0 条记录/);
    assert.equal(h.els.semanticMemoryCount.textContent, '0');
  });
  await check('离开页面取消摘要并禁止排队重启，返回后可手动恢复', async () => {
    const h = harness({abortMode: 'ignore'}); h.stream(instances());
    h.stream(instances('library-b')); h.lifecycle('pagehide');
    h.summaries()[0].resolve(summary()); await flush();
    assert.equal(h.summaries().length, 1); assert.equal(h.timerCount(), 0);
    h.lifecycle('pageshow'); const p = h.loadSemanticIndexSummary();
    h.summaries()[1].resolve(summary('library-b')); await p;
    assert.match(h.indexText(), /失败 6/);
  });
  console.log(JSON.stringify({passed: passed.length, checks: passed}, null, 2));
}
main().catch(error => {console.error(error); process.exitCode = 1;});
