/* 实时数据连接（/streams/ws）：连接、断线指数退避重连、游标续传、缺口恢复与按帧分发。
   逻辑从原控制台 console.js（connectStreams 一组函数）迁出，行为不变：
   - 断开后 1 秒起每次翻倍、最长 30 秒重连，URL 带 after_cursor 续传；
   - stream.replay 带 resync_required / replay_gap、stream.gap、队列主题积压超过 200 帧时报告缺口，由订阅方重读权威数据；
   - 普通主题按动画帧合并为最新一帧，按固定顺序分发；任务与日志主题逐帧保留；位姿主题到达即处理；
   - 主连接每 5 秒发一次 stream.ping 测往返，回应超过 1 秒未到时显示已等待的时间，并据回应估算设备与浏览器的时钟差。
   一个页面可以开多条连接（主连接、位姿、日志），各自排队与续传，日志洪峰不挤掉地图帧。 */

const STREAM_CURSOR_MAX_U64 = '18446744073709551615';
const QUEUE_LIMIT = 200;
// 主连接合并帧的分发顺序（与原控制台一致）。
export const FLUSH_ORDER = ['robot.state', 'geometry.state', 'map.frame', 'map.heartbeat', 'localization.trajectory', 'localization.pose',
  'navigation.state', 'navigation.forbidden_zones', 'exploration.frontier', 'exploration.state', 'runtime.governance', 'pointcloud_map.meta',
  'video.frame', 'video.detections', 'memory.instances', 'memory.scene_spaces', 'tasks.state', 'system.metrics', 'settings.display'];

export function normalizedStreamCursor(value) {
  let normalized = null;
  if (typeof value === 'string' && /^(0|[1-9]\d*)$/.test(value)) normalized = value;
  else if (typeof value === 'number' && Number.isSafeInteger(value) && value >= 0) normalized = String(value);
  if (normalized === null) return null;
  if (normalized.length > STREAM_CURSOR_MAX_U64.length
    || (normalized.length === STREAM_CURSOR_MAX_U64.length && normalized > STREAM_CURSOR_MAX_U64)) return null;
  return normalized;
}
export function compareCanonicalInteger(left, right) {
  if (left.length !== right.length) return left.length < right.length ? -1 : 1;
  if (left === right) return 0;
  return left < right ? -1 : 1;
}

// 到达序号跨连接递增；任务快照对账用它判断事件是否晚于快照请求。
let arrivalSequence = 0;

/* 选项：
   topics          订阅主题
   params          额外查询参数（如 map_binary_version、pointcloud_transport），可用 setParam 修改
   immediate       到达即分发的主题（位姿）
   queued          逐帧保留的主题（任务、日志），超过 200 帧报告该主题缺口
   closeOnGap      收到 stream.gap 时关闭本连接重连（位姿连接）
   primary         主连接：维护 store.conn 与 store.clockOffsetMs
   store           primary 为真时写入的统一状态 */
export function createStream({ topics, params = {}, immediate = [], queued = [], closeOnGap = false, primary = false, store = null }) {
  const wanted = new Set(topics);
  const query = { ...params };
  const hooks = { frame: new Map(), flush: new Set(), gap: new Set(), open: new Set(), close: new Set(), replay: new Set(), binary: new Set(), error: new Set() };
  const latest = {};
  let socket = null, epoch = 0, reconnectTimer = null, reconnectDelayMs = 1000, reconnectAttempt = 0, latestCursor = '0';
  let pending = {}, pendingQueued = new Map(queued.map(topic => [topic, []])), flushFrame = null;
  const probe = { current: null, due: 0, seq: 0, samples: [] };

  function emit(kind, ...args) { for (const fn of hooks[kind]) fn(...args); }
  function buildUrl() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const url = new URL(protocol + '//' + window.location.host + '/streams/ws');
    url.searchParams.set('topics', [...wanted].join(','));
    for (const [key, value] of Object.entries(query)) if (value != null) url.searchParams.set(key, String(value));
    if (latestCursor) url.searchParams.set('after_cursor', String(latestCursor));
    return url.toString();
  }
  function updateCursor(value, allowReset = false) {
    const next = normalizedStreamCursor(value);
    if (next === null) return false;
    const current = normalizedStreamCursor(latestCursor) || '0';
    if (allowReset || next.length > current.length || (next.length === current.length && next > current)) {
      latestCursor = next;
      return true;
    }
    return next === current;
  }
  function setConn(patch) { if (primary) store.set({ conn: { ...store.get().conn, ...patch } }); }

  function connect() {
    if (socket && [WebSocket.CONNECTING, WebSocket.OPEN].includes(socket.readyState)) return;
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    const ws = new WebSocket(buildUrl());
    const myEpoch = ++epoch;
    ws.binaryType = 'arraybuffer';
    socket = ws;
    let acceptsMessages = true;
    const isCurrent = () => ws === socket && myEpoch === epoch;
    ws.addEventListener('open', () => {
      if (!isCurrent()) return;
      const wasReconnect = reconnectAttempt > 0;
      reconnectAttempt = 0;
      reconnectDelayMs = 1000;
      if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
      if (primary) {
        probe.current = null;
        probe.due = 0;
        setConn({ status: 'open', waitingMs: null });
        measureLatency();
      }
      emit('open', { reconnect: wasReconnect });
    });
    ws.addEventListener('message', event => {
      if (!isCurrent() || !acceptsMessages) return;
      if (event.data instanceof ArrayBuffer) { emit('binary', event.data); return; }
      const message = JSON.parse(event.data);
      if (message.type === 'stream.pong') { if (primary) acceptPong(message); return; }
      if (message.type === 'keepalive') { updateCursor(message.cursor); return; }
      if (message.type === 'stream.replay') {
        updateCursor(message.latest_cursor, message.resync_required && message.gap_reason === 'cursor_ahead_of_stream');
        if (message.resync_required || message.replay_gap) emit('gap', gapInfo(message, 'replay'));
        emit('replay', message);
        return;
      }
      if (message.type === 'stream.gap') {
        updateCursor(message.latest_cursor);
        emit('gap', gapInfo(message, 'queue'));
        if (closeOnGap) { acceptsMessages = false; ws.close(); }
        return;
      }
      if (message.type !== 'stream.frame' || !message.frame) return;
      if (!wanted.has(message.frame.topic)) return;
      receiveFrame(message.frame);
    });
    ws.addEventListener('close', () => {
      if (!isCurrent()) return;
      acceptsMessages = false;
      if (primary) {
        // 时钟差只对本次连接有效（与原控制台按连接纪元判断一致），断开即作废。
        probe.current = null;
        probe.samples = [];
        setConn({ status: 'lost', rttMs: null, waitingMs: null });
        store.set({ clockOffsetMs: null });
      }
      emit('close');
      reconnectAttempt += 1;
      scheduleReconnect(ws, myEpoch);
    });
    ws.addEventListener('error', () => { if (isCurrent()) emit('error'); });
  }
  function gapInfo(message, kind) {
    const list = Array.isArray(message?.topics) && message.topics.length ? new Set(message.topics) : null;
    return { kind, topics: list, message };
  }
  function scheduleReconnect(ws, myEpoch) {
    const delayMs = Math.min(30000, Math.max(1000, reconnectDelayMs));
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(() => {
      if (ws !== socket || myEpoch !== epoch) return;
      reconnectTimer = null;
      connect();
    }, delayMs);
    reconnectDelayMs = Math.min(30000, delayMs * 2);
  }
  /* 立即重连：断开状态下跳过剩余等待；已连接时不动。 */
  function reconnectNow() {
    if (socket && [WebSocket.CONNECTING, WebSocket.OPEN].includes(socket.readyState)) return;
    reconnectDelayMs = 1000;
    connect();
  }

  function receiveFrame(frame) {
    updateCursor(frame.cursor);
    frame.__seq = ++arrivalSequence;
    latest[frame.topic] = frame;
    if (immediate.includes(frame.topic)) { dispatch(frame); return; }
    const queue = pendingQueued.get(frame.topic);
    if (queue) {
      queue.push(frame);
      if (queue.length > QUEUE_LIMIT) {
        queue.splice(0, queue.length - QUEUE_LIMIT);
        emit('gap', { kind: 'overflow', topics: new Set([frame.topic]), message: null });
      }
    } else pending[frame.topic] = frame;
    if (!flushFrame) flushFrame = requestAnimationFrame(flush);
  }
  function flush() {
    flushFrame = null;
    const frames = pending;
    pending = {};
    for (const topic of FLUSH_ORDER) if (frames[topic]) { dispatch(frames[topic]); delete frames[topic]; }
    for (const topic of Object.keys(frames)) dispatch(frames[topic]);
    for (const [topic, queue] of pendingQueued) {
      if (!queue.length) continue;
      pendingQueued.set(topic, []);
      for (const frame of queue) dispatch(frame);
    }
    emit('flush');
  }
  function dispatch(frame) { for (const fn of hooks.frame.get(frame.topic) || []) fn(frame); }

  /* 往返测量：主连接每秒检查一次，上一次回应后 5 秒再发；等待超过 1 秒时显示已等待的时间。 */
  function measureLatency() {
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    const now = performance.now();
    if (probe.current) {
      const elapsed = now - probe.current.sentAt;
      if (elapsed >= 1000) setConn({ waitingMs: Math.round(elapsed) });
      return;
    }
    if (now < probe.due) return;
    const requestId = epoch + ':' + (++probe.seq);
    probe.current = { requestId, sentAt: now, sentWallAt: Date.now() };
    socket.send(JSON.stringify({ type: 'stream.ping', request_id: requestId }));
  }
  function acceptPong(message) {
    const current = probe.current;
    if (!current || message.request_id !== current.requestId) return;
    const now = performance.now();
    const rttMs = now - current.sentAt;
    setConn({ rttMs: Math.round(rttMs), waitingMs: null });
    const serverMs = Number(message.server_timestamp_ns) / 1e6;
    if (Number.isFinite(serverMs) && serverMs > 0) {
      // 取 30 秒内往返最短的一次估算时钟差：设备时间 − (发出时刻 + 往返/2)。
      const sample = { rttMs, offsetMs: serverMs - (current.sentWallAt + rttMs / 2), measuredAt: now, epoch };
      probe.samples = probe.samples.filter(item => item.epoch === sample.epoch && now - item.measuredAt < 30000);
      probe.samples.push(sample);
      const best = probe.samples.reduce((a, b) => (b.rttMs < a.rttMs ? b : a), sample);
      store.set({ clockOffsetMs: Math.round(best.offsetMs) });
    }
    probe.current = null;
    probe.due = now + 5000;
  }
  if (primary) setInterval(measureLatency, 1000);

  function hook(kind) { return fn => { hooks[kind].add(fn); return () => hooks[kind].delete(fn); }; }
  return {
    connect,
    reconnectNow,
    /* 按主题订阅帧：fn(frame)；frame.payload 为载荷，frame.cursor 为游标。 */
    on(topic, fn) {
      if (!hooks.frame.has(topic)) hooks.frame.set(topic, new Set());
      hooks.frame.get(topic).add(fn);
      return () => hooks.frame.get(topic).delete(fn);
    },
    onFlush: hook('flush'),     // 每批分发结束后
    onGap: hook('gap'),         // fn({kind: replay / queue / overflow, topics: Set 或 null（全部）, message})
    onOpen: hook('open'),       // fn({reconnect})
    onClose: hook('close'),
    onReplay: hook('replay'),   // fn(stream.replay 消息)
    onBinary: hook('binary'),   // fn(ArrayBuffer)
    onError: hook('error'),
    /* 增加订阅主题；已连接时按新主题重连，续传游标不变。 */
    want(list) {
      const before = wanted.size;
      for (const topic of list) wanted.add(topic);
      if (wanted.size === before || !socket || socket.readyState !== WebSocket.OPEN) return;
      // 换掉当前连接：旧连接的关闭事件因不再是当前连接而被忽略，不进入断连状态。
      const old = socket;
      socket = null;
      old.close();
      connect();
    },
    /* 去掉主题：只影响之后重连时的订阅（如三维增量点云握手成功后不再订阅全量点云快照）。 */
    drop(list) { for (const topic of list) wanted.delete(topic); },
    /* 把二进制解出的帧当作普通帧接收（续传游标、合并与分发同文本帧）。 */
    receive: receiveFrame,
    setParam(key, value) { query[key] = value; },
    send(message) { if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message)); },
    latest(topic) { return latest[topic] || null; },
    /* 当前到达序号：发起快照请求前记下，用于判断之后到达的事件是否比快照新。 */
    seq() { return arrivalSequence; },
    updateCursor,
    get cursor() { return latestCursor; },
  };
}
