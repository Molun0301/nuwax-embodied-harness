/* 统一状态：一个对象、按字段整体替换、按字段订阅。
   同一轮事件里的多次写入合并成一次通知（微任务），订阅者只在关心的字段变化时被调用。
   字段说明见 initialState；各字段由唯一的写入方维护（见右侧注释）。 */

export function createStore(initial) {
  let state = { ...initial };
  const subscribers = [];
  let changed = null;
  function notify() {
    const keys = changed;
    changed = null;
    for (const sub of subscribers.slice()) {
      if (!sub.keys || sub.keys.some(key => keys.has(key))) sub.fn(state, keys);
    }
  }
  return {
    get() { return state; },
    /* 写入：值与原值相同（引用相等）的字段不触发通知；嵌套对象须整体换新。 */
    set(patch) {
      const next = { ...state };
      for (const [key, value] of Object.entries(patch)) {
        if (state[key] === value) continue;
        next[key] = value;
        if (!changed) { changed = new Set(); queueMicrotask(notify); }
        changed.add(key);
      }
      state = next;
    },
    /* 订阅：keys 为字段名数组（null 表示全部），返回取消订阅函数。fn(state, changedKeys)。 */
    subscribe(keys, fn) {
      const sub = { keys, fn };
      subscribers.push(sub);
      return () => { const i = subscribers.indexOf(sub); if (i >= 0) subscribers.splice(i, 1); };
    },
  };
}

export function initialState() {
  return {
    route: { view: 'monitor', sub: '' },               // 外壳：当前视图与视图内子页（监控页为右侧面板页签）
    conn: { status: 'connecting', rttMs: null, waitingMs: null },   // stream：主实时连接 connecting / open / lost 与往返延迟
    clockOffsetMs: null,                                // stream：设备时钟减浏览器时钟（毫秒），由往返测量估算
    robot: { name: '', model: '', controlMode: 'unknown', battery: null, batteryRemainingMin: null, canMove: null, warning: '' },  // feeds
    geometry: { phase: 'idle', mapName: '', reason: '', connected: false },  // feeds：地图会话阶段与地图名
    metrics: { status: 'missing', cpuPercent: null, cpuThreads: null, memPercent: null, memUsedBytes: null, memTotalBytes: null },  // feeds
    services: null,                                     // feeds：监督者组件表 {名称: {title, state, state_title, pid, uptime_sec, exit_code, error, enabled, log_file}}
    manager: { reachable: false, runtimeRunning: null, activeJob: null, updateAvailable: false },  // feeds：管理服务状态
    business: 'running',                                // feeds：running / stopped / updating（只在断连时按管理服务判断）
    tasks: { active: [], history: [], pending: new Set(), errors: new Map(), error: '', ready: false },  // tasks：统一任务表
    estop: null,                                        // 外壳：最近一次急停 {at, cancelled, issues[], error}
    volume: null,                                       // 外壳：{percent, error}；机器人没有音量能力时为 null
    attention: [],                                      // feeds：需要注意的事件（新在前）{id, tone, text, at, comp, taskId, requestId}
    unread: 0,                                          // feeds：未读的需要注意条数
  };
}
