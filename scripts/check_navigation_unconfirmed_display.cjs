#!/usr/bin/env node
// Offline production-console function seam, matching check_map_snapshot_wire.cjs.
// Candidate only: not executed during this review and does not command hardware.
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const paths = ['gateways/http/static/js/console.js', 'gateways/http/static/preview/js/console.js'];
let checks = 0;
for (const relative of paths) {
  const source = fs.readFileSync(path.join(root, relative), 'utf8');
  function fn(name) {
    const start = source.indexOf(`      function ${name}(`);
    const end = source.indexOf('\n      function ', start + 20);
    assert(start >= 0 && end > start, `missing production function ${name}`);
    return source.slice(start, end);
  }
  const state = {navigationPath: [], navigationDisplay: null};
  const els = {navigationPathLegend: {hidden: true}, navigationPathStatus: {textContent: '', dataset: {}}};
  const context = vm.createContext({state, els, pointFromPose: () => null, pointFromFrontier: () => null});
  for (const name of ['pathPointFromValue', 'navigationContextIsPublic', 'navigationControlWarning',
                       'mergeNavigationDisplay', 'renderNavigationPathStatus']) vm.runInContext(fn(name), context);
  const identity = {owner_kind: 1, parent_request_id: 'owned-goal', goal_generation: 7, producer_runtime_generation: 11};
  const goal = {position: {x: 3, y: 4, z: 1}};
  const route = [{x: 0, y: 0, z: 1}, {x: 3, y: 4, z: 1}];
  function frame(status, metadata = {}, timestamp = '2026-10-18T10:00:00Z') {
    return {current_goal: {goal_id: 'owned-goal', target_pose: goal}, navigation_state: {
      current_goal_id: 'owned-goal', status, updated_at: timestamp,
      metadata: {...identity, path_revision: 3, planner_runtime_generation: 13, ownership_retained: true, authority_current: true,
        dispatch_obligation: {origin_reason_code: 'GOAL_PUBLISH_UNCONFIRMED', connection_epoch: 4,
          identity: {goal_id: 'owned-goal', ...identity}, terminal_observed: false}, ...metadata}}};
  }
  function merge(previous, next, points = route) {return context.mergeNavigationDisplay(previous, next, goal, points);}
  function check(name, run) {run(); checks++; console.log(`PASS ${relative}: ${name}`);}
  check('unknown keeps goal and route nonterminal with warning rather than executing', () => {
    const result = merge(null, frame('unconfirmed'));
    assert.equal(result.terminal, false); assert.equal(result.goal, goal); assert.equal(result.path, route);
    assert.equal(result.mode, 'retained'); assert.match(result.warning, /未确认/); assert.equal(result.localPath.length, 0);
    state.navigationDisplay = result; state.navigationPath = result.path;
    context.renderNavigationPathStatus(); assert.equal(els.navigationPathLegend.hidden, false);
    assert.equal(els.navigationPathStatus.dataset.level, 'warn'); assert.match(els.navigationPathStatus.textContent, /未确认/);
  });
  check('unknown followed by real running and exact terminal clears only on terminal', () => {
    let result = merge(null, frame('unconfirmed', {command_projection_without_source: true}, '2026-10-18T20:00:00Z'));
    result = merge(result, frame('running', {local_path: route}, '2026-10-18T10:00:00Z'));
    assert.equal(result.terminal, false); assert.equal(result.mode, 'executing'); assert.equal(result.warning, '');
    assert.equal(result.timestamp, Date.parse('2026-10-18T10:00:00Z'));
    result = merge(result, frame('succeeded', {ownership_retained: false, actuator_zeroed: true, zero_ack_id: 'exact'}));
    assert.equal(result.terminal, true); assert.equal(result.goal, null); assert.equal(result.path.length, 0);
    assert.equal(merge(result, frame('unconfirmed')), result, 'old unknown cannot reopen terminal');
    assert.equal(merge(result, frame('running')), result, 'ordinary terminal latch unchanged');
  });
  check('unknown cancellation becomes terminal only on canonical cancelled update', () => {
    const unknown = merge(null, frame('unconfirmed'));
    assert.equal(merge(unknown, frame('unconfirmed'), []).goal, goal);
    const cancelled = merge(unknown, frame('cancelled', {ownership_retained: false, actuator_zeroed: true}));
    assert.equal(cancelled.terminal, true); assert.equal(cancelled.path.length, 0);
  });
  check('late source-less projection cannot recolor real execution', () => {
    const running = merge(null, frame('running', {local_path: route}));
    const late = merge(running, frame('unconfirmed', {command_projection_without_source: true}, '2026-10-18T20:00:00Z'));
    assert.equal(late, running); assert.equal(late.mode, 'executing');
  });
  check('history retains original route but visibly loses cancellation authority', () => {
    const running = merge(null, frame('running', {local_path: route}));
    const historyFrame = frame('unconfirmed', {owner_kind: 0, parent_request_id: '', goal_generation: 0,
      producer_runtime_generation: 0, authority_current: false, command_projection_without_source: true,
      path_revision: 0, planner_runtime_generation: 0, dispatch_obligation: {terminal_observed: true},
      historical_navigation_identity: {goal_id: 'owned-goal', ...identity}});
    const history = merge(running, historyFrame, []);
    assert.equal(history.goal, goal); assert.equal(history.path, route); assert.equal(history.mode, 'retained');
    assert.equal(history.authorityCurrent, false); assert.match(history.warning, /没有取消资格/);
    assert.equal(history.terminal, false); assert.equal(context.navigationContextIsPublic(historyFrame), true);
  });
  check('terminal observed without specific status does not invent failure or clear target', () => {
    const result = merge(null, frame('unconfirmed', {dispatch_obligation: {terminal_observed: true}}));
    assert.match(result.warning, /完整停车证明/); assert.equal(result.goal, goal); assert.equal(result.terminal, false);
  });
  check('terminal-observed source-less projection remains stronger than prior running', () => {
    const running = merge(null, frame('running', {local_path: route}));
    const pending = merge(running, frame('unconfirmed', {command_projection_without_source: true,
      dispatch_obligation: {terminal_observed: true}}, '2026-10-18T20:00:00Z'), []);
    assert.notEqual(pending, running); assert.equal(pending.mode, 'retained');
    assert.equal(pending.terminal, false); assert.match(pending.warning, /完整停车证明/);
    assert.equal(merge(pending, frame('running', {}, '2026-10-18T21:00:00Z')), pending,
      'terminal-observed proof pending must not resurrect running');
    assert.equal(merge(pending, frame('failed', {ownership_retained: false, actuator_zeroed: true})).terminal, true);
  });
  check('terminal-seen source-less rejects older path and same-path different planner', () => {
    const running = merge(null, frame('running', {local_path: route}));
    for (const changed of [{path_revision: 2}, {path_revision: 0}, {planner_runtime_generation: 12},
                           {planner_runtime_generation: 14}, {planner_runtime_generation: 0},
                           {owner_kind: 2}, {parent_request_id: 'foreign'},
                           {producer_runtime_generation: 0}, {goal_generation: 0}]) {
      const stale = frame('unconfirmed', {command_projection_without_source: true,
        dispatch_obligation: {terminal_observed: true}, ...changed}, '2026-10-18T20:00:00Z');
      assert.equal(merge(running, stale, []), running);
    }
  });
  check('terminal-seen newer path accepts a numerically smaller planner identity', () => {
    const running = merge(null, frame('running', {local_path: route}));
    const pending = merge(running, frame('unconfirmed', {path_revision: 4, planner_runtime_generation: 12,
      command_projection_without_source: true, dispatch_obligation: {terminal_observed: true}}, '2026-10-18T20:00:00Z'), []);
    assert.notEqual(pending, running); assert.equal(pending.revision, 4); assert.equal(pending.planner, 12);
    assert.equal(pending.terminal, false); assert.equal(pending.mode, 'retained');
    assert.match(pending.warning, /完整停车证明/);
  });
  check('retained dispatch lineage rejects old-path complete ZERO receipt', () => {
    const unknown = merge(null, frame('unconfirmed'));
    const running = merge(unknown, frame('running', {local_path: route}));
    const oldProof = frame('failed', {path_revision: 2, planner_runtime_generation: 13,
      ownership_retained: false, dispatch_obligation: null, actuator_zeroed: true,
      zero_ack_id: 'old-path-proof', zero_applied_monotonic_ns: 123});
    assert.equal(merge(running, oldProof), running);
    assert.equal(running.terminal, false); assert.equal(running.path, route);
  });
  check('retained dispatch receipt uses same-path exact planner and newer-path ordering', () => {
    const unknown = merge(null, frame('unconfirmed'));
    const proof = {ownership_retained: false, dispatch_obligation: null, actuator_zeroed: true,
      zero_ack_id: 'complete-proof', zero_applied_monotonic_ns: 124};
    for (const planner of [12, 14]) {
      assert.equal(merge(unknown, frame('failed', {...proof, planner_runtime_generation: planner})), unknown);
    }
    const newer = merge(unknown, frame('failed', {...proof, path_revision: 4, planner_runtime_generation: 12}));
    assert.equal(newer.terminal, true); assert.equal(newer.revision, 4); assert.equal(newer.planner, 12);
    assert.equal(newer.goal, null); assert.equal(newer.path.length, 0);
    assert.equal(merge(unknown, frame('failed', proof)).terminal, true, 'same path exact planner still settles');
  });
  check('ordinary terminal path behavior is outside the retained-dispatch gate', () => {
    const ordinary = merge(null, frame('running', {ownership_retained: false, dispatch_obligation: null, local_path: route}));
    assert.equal(ordinary.dispatchObligationRetained, false);
    const failed = merge(ordinary, frame('failed', {path_revision: 2, planner_runtime_generation: 12,
      ownership_retained: false, dispatch_obligation: null}));
    assert.equal(failed.terminal, true); assert.equal(failed.path.length, 0);
  });
  check('ordinary FAILED latch remains unchanged', () => {
    const failed = merge(null, frame('failed', {ownership_retained: false, dispatch_obligation: null}));
    assert.equal(failed.terminal, true); assert.equal(failed.goal, null);
    assert.equal(merge(failed, frame('running')), failed);
  });
}
console.log(JSON.stringify({checks, status: 'passed', scope: 'offline production console consumption functions'}));
