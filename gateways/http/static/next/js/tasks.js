/* 统一任务表：首屏 /api/tasks 快照加 tasks.state 事件流，不按“是不是本页提交”过滤。
   顶栏、活动、导航、探索都从这里取数。对账规则从原控制台 console.js 迁出，行为不变：
   - 快照到达前收到的事件先排队（最多 200 条），快照写入后按序补上；
   - 活动列表是权威快照：快照请求发出后才到的事件保留，其余不在快照里的活动任务删除；
   - 已结束的任务不会被迟到的进行中事件拉回；
   - 内存只留最近 50 个活动、200 个已结束任务，列表给出最近 20 个已结束任务；
   - 重连、重放缺口、事件积压时重读快照。 */
import { isTerminalTaskState, normalizeTaskState, taskKind } from './status.js';

const ACTIVE_KEEP = 50, HISTORY_KEEP = 200, HISTORY_LIST = 20, BOOTSTRAP_QUEUE_LIMIT = 200;

export function createTasks({ store, stream, api }) {
  const active = new Map(), history = new Map();
  const pending = new Set(), errors = new Map();
  const navRange = new Map();    // 导航任务见过的最大剩余距离，折算进度用
  const listeners = new Set();
  let snapshotSequence = 0, ready = false, bootstrapQueue = [], loadPromise = null, reloadPending = false, loadError = '', dirty = false;

  const updatedMs = task => Number(task?.finished_at || task?.created_at || 0) * 1000;
  const sorted = (map, limit) => {
    const items = [...map.values()].sort((a, b) => updatedMs(b) - updatedMs(a));
    return limit == null ? items : items.slice(0, limit);
  };
  function trim(map, keep) {
    if (map.size <= keep) return;
    for (const [key] of [...map.entries()].sort((a, b) => updatedMs(b[1]) - updatedMs(a[1])).slice(keep)) map.delete(key);
  }
  function publish() {
    trim(active, ACTIVE_KEEP);
    trim(history, HISTORY_KEEP);
    for (const id of pending) if (!active.has(id)) pending.delete(id);
    store.set({ tasks: { active: sorted(active), history: sorted(history, HISTORY_LIST), pending: new Set(pending),
      errors: new Map(errors), error: loadError, ready } });
  }
  function track(task) {
    const remaining = Number(task.data?.remaining_distance_m);
    if (taskKind(task) === 'nav' && Number.isFinite(remaining)) navRange.set(task.task_id, Math.max(navRange.get(task.task_id) || 0, remaining));
  }

  function applyEvent(frame) {
    const status = frame.payload.payload;
    const key = status.task_id;
    const previous = history.get(key);
    if (previous && isTerminalTaskState(previous.state) && !isTerminalTaskState(status.state)) return;
    const task = { ...status, __streamSequence: frame.__seq };
    track(task);
    if (isTerminalTaskState(task.state)) { active.delete(key); history.set(key, task); }
    else { history.delete(key); active.set(key, task); }
    dirty = true;
    for (const fn of listeners) fn(task);
  }
  function replaceSnapshot(payload, sequence) {
    if (sequence < snapshotSequence) return;
    const list = payload.active_tasks;
    const ids = new Set(list.filter(task => task?.task_id).map(task => String(task.task_id)));
    for (const [key, task] of active) if (!ids.has(key) && (task.__streamSequence || 0) <= sequence) active.delete(key);
    for (const task of list) {
      const key = String(task.task_id);
      const terminal = history.get(key);
      if (terminal && isTerminalTaskState(terminal.state)) continue;
      const existing = active.get(key);
      if (existing && (existing.__streamSequence || 0) > sequence) continue;
      active.set(key, { ...existing, ...task });
      track(task);
    }
    for (const task of payload.history_tasks) {
      const key = String(task.task_id);
      const existing = history.get(key);
      if (isTerminalTaskState(task.state)) active.delete(key);
      if (!existing) history.set(key, task);
      else if ((existing.__streamSequence || 0) <= sequence && updatedMs(task) >= updatedMs(existing)) history.set(key, { ...existing, ...task });
    }
    snapshotSequence = sequence;
  }
  async function load() {
    if (loadPromise) { reloadPending = true; return loadPromise; }
    const sequence = stream.seq();
    loadPromise = (async () => {
      try {
        replaceSnapshot(await api.json('/api/tasks?history_limit=20'), sequence);
        loadError = '';
      } catch (error) {
        loadError = '任务列表读取失败：' + error.message;
      } finally {
        ready = true;
        const queue = bootstrapQueue;
        bootstrapQueue = [];
        for (const frame of queue) applyEvent(frame);
        publish();
      }
    })();
    try { await loadPromise; }
    finally {
      loadPromise = null;
      if (reloadPending) { reloadPending = false; void load(); }
    }
  }

  stream.on('tasks.state', frame => {
    if (!ready) {
      bootstrapQueue.push(frame);
      // 积压超限：丢掉最旧的事件，并在本次快照之后再读一次。
      if (bootstrapQueue.length > BOOTSTRAP_QUEUE_LIMIT) { bootstrapQueue.splice(0, bootstrapQueue.length - BOOTSTRAP_QUEUE_LIMIT); void load(); }
      return;
    }
    applyEvent(frame);
  });
  stream.onFlush(() => { if (dirty) { dirty = false; publish(); } });
  stream.onOpen(({ reconnect }) => { if (reconnect) void load(); });
  stream.onGap(({ topics }) => { if (!topics || topics.has('tasks.state')) void load(); });

  /* 取消或停止：导航与其它任务调用 cancel_task；探索调用 stop_exploration（探索尚未启动时不带编号）。
     发出后按钮显示“正在取消”，失败时写出后端原因。 */
  async function stop(task) {
    const id = task.task_id;
    pending.add(id);
    errors.delete(id);
    publish();
    try {
      if (taskKind(task) === 'explore') {
        const explorationId = task.data?.exploration_id;
        await api.invoke('stop_exploration', explorationId ? { exploration_id: explorationId } : {});
      } else await api.invoke('cancel_task', { task_id: id });
    } catch (error) {
      pending.delete(id);
      errors.set(id, error.message);
      publish();
    }
  }

  return {
    load,
    stop,
    /* 每次任务快照变化：fn(task)，用于跟随本页提交的任务进度。 */
    onUpdate(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    navRange,
    get(taskId) { return active.get(taskId) || history.get(taskId) || null; },
    state: normalizeTaskState,
  };
}
