/* 外壳用到的数据源，写入统一状态：
   - 健康 /api/health：首屏读一次，页面可见时每 3 秒重读（顶栏要随时知道有没有服务退出）；
   - 实时主题 robot.state（机器人名称、电量、运动可用）、geometry.state（地图会话阶段与地图名）、system.metrics（处理器与内存）；
   - 管理服务 /api/manager/status：页面可见时每 30 秒读一次（“系统”上的新版本圆点）；与网关断开时每 5 秒读一次，
     能读到且业务未运行时，顶栏按进行中管理任务的动作写“正在启动 / 停止 / 重启业务服务”“正在切换到新版本”等，
     没有进行中的任务时再读任务列表，最近一次任务在启动阶段失败的写“业务服务启动失败”（deriveStartFailure），否则写“已停止”；
   - 需要注意：/api/feedback/stream 中警告与错误级运行事件（与 MCP 通知同一投影，06 第 5 节），加上监督者报告的已退出服务。 */
import { deriveStartFailure } from '/static/shared/system-pages.js';

const HEALTH_INTERVAL_MS = 3000, MANAGER_INTERVAL_MS = 30000, MANAGER_LOST_INTERVAL_MS = 5000, ATTENTION_KEEP = 50;

export function startFeeds({ store, stream, api }) {
  const set = patch => store.set(patch);
  const merge = (key, patch) => {
    const current = store.get()[key];
    if (Object.entries(patch).every(([k, v]) => current[k] === v)) return;
    set({ [key]: { ...current, ...patch } });
  };

  /* ---- 健康 ---- */
  let healthBusy = false;
  let healthRead = false;
  async function readHealth() {
    if (healthBusy) return;
    healthBusy = true;
    try {
      const health = await api.json('/api/health');
      if (health.robot?.robot_name) merge('robot', { name: health.robot.robot_name });
      applyGeometry(health.geometry?.state, health.geometry?.connected);
      applyServices(health.processes || {}, !healthRead);
      healthRead = true;
    } catch (_) { /* 下一次重读；读取失败不清掉已显示的状态 */ }
    finally { healthBusy = false; }
  }
  function applyGeometry(snapshot, connected) {
    merge('geometry', { phase: snapshot?.phase || 'idle', mapName: snapshot?.map_name || '', reason: snapshot?.reason || '',
      connected: connected ?? store.get().geometry.connected });
  }
  // first：页面打开时第一次读到的已退出服务照常列出，不计未读。
  function applyServices(processes, first) {
    const previous = store.get().services;
    if (JSON.stringify(previous) !== JSON.stringify(processes)) set({ services: processes });
    // 已退出的服务进入“需要注意”；同一进程的退出只记一次。
    for (const [name, item] of Object.entries(processes)) {
      if (item.state !== 'exited') continue;
      addAttention({ id: 'svc:' + name + ':' + (item.pid ?? '') + ':' + (item.exit_code ?? ''), tone: 'bad',
        text: (item.title || name) + '已退出' + (item.exit_code != null ? '（退出码 ' + item.exit_code + '）' : '') + (item.error ? '：' + item.error : ''),
        at: Date.now(), comp: name }, first);
    }
  }

  /* ---- 实时主题 ---- */
  let robotChangeKey = null;
  stream.on('robot.state', frame => {
    const payload = frame.payload;
    // 设备纪元与修订都没变的重复帧不再处理（与原控制台相同）。
    const metadata = frame.metadata || {};
    const revision = metadata.revision ?? frame.revision;
    const key = metadata.source_epoch && revision != null && revision !== ''
      ? JSON.stringify([frame.source || '', metadata.robot_name || payload.robot_name || '', metadata.source_epoch, revision]) : null;
    if (key && key === robotChangeKey) return;
    robotChangeKey = key;
    const robot = payload.robot_state || {};
    const battery = robot.battery || {};
    const raw = Number(battery.percentage);
    const percent = battery.percentage != null ? Math.round(raw <= 1 ? raw * 100 : raw) : battery.level != null ? Number(battery.level) : null;
    const current = store.get().robot;
    merge('robot', { name: current.name || payload.robot_name || '', model: payload.robot_model || current.model,
      controlMode: robot.mode || 'unknown', battery: percent,
      canMove: robot.safety?.can_move ?? null, warning: robot.safety?.warning_message || '' });
  });
  stream.on('geometry.state', frame => applyGeometry(frame.payload));
  stream.on('system.metrics', frame => {
    const snap = frame.payload || {};
    const available = snap.psutil_available === true && snap.status !== 'disabled';
    merge('metrics', { status: available ? 'available' : snap.status || 'missing',
      cpuPercent: available && snap.cpu?.percent != null ? snap.cpu.percent : null,
      cpuThreads: Array.isArray(snap.cpu?.per_core) ? snap.cpu.per_core.length : null,
      memPercent: available && snap.memory?.percent != null ? snap.memory.percent : null,
      memUsedBytes: snap.memory?.used_bytes ?? null, memTotalBytes: snap.memory?.total_bytes ?? null });
  });

  /* ---- 管理服务 ---- */
  let managerTimer = null;
  async function readManager() {
    try {
      const status = await api.manager('status');
      // 业务没运行且没有进行中的任务时，才需要任务列表判断是不是启动失败。
      const startFailure = status.runtime?.running === false && !status.active_job ? deriveStartFailure(status, (await api.manager('jobs')).jobs) : null;
      set({ manager: { reachable: true, runtimeRunning: status.runtime?.running ?? null, activeJob: status.active_job || null,
        updateAvailable: Boolean(status.update?.available), startFailure } });
    } catch (_) {
      merge('manager', { reachable: false });
    }
    syncBusiness();
  }
  // 网关连着时业务一定在运行；断开后能读到管理服务才判断业务是否停止、启动失败或有管理任务在进行。
  function syncBusiness() {
    const { conn, manager } = store.get();
    let business = 'running';
    if (conn.status === 'lost' && manager.reachable) {
      business = manager.activeJob ? 'updating' : manager.runtimeRunning === false ? (manager.startFailure ? 'failed' : 'stopped') : 'running';
    }
    if (store.get().business !== business) set({ business });
  }
  function scheduleManager() {
    clearTimeout(managerTimer);
    const lost = store.get().conn.status === 'lost';
    managerTimer = setTimeout(() => { if (!document.hidden) void readManager(); scheduleManager(); }, lost ? MANAGER_LOST_INTERVAL_MS : MANAGER_INTERVAL_MS);
  }
  store.subscribe(['conn'], state => {
    if (state.conn.status === 'lost') { void readManager(); scheduleManager(); }
    syncBusiness();
  });

  /* ---- 需要注意 ---- */
  const seen = new Set();
  function addAttention(item, read = false) {
    const list = store.get().attention;
    if (list.some(entry => entry.id === item.id)) return;
    if (read) seen.add(item.id);
    const next = [item, ...list].slice(0, ATTENTION_KEEP);
    set({ attention: next, unread: next.filter(entry => !seen.has(entry.id)).length });
  }
  function markAttentionRead() {
    for (const entry of store.get().attention) seen.add(entry.id);
    set({ unread: 0 });
  }
  // 浏览器断线后自动带 Last-Event-ID 续传；反馈缺口只影响“需要注意”的完整性，不重新执行任何动作。
  // 首次连接时 feedback.ready 给出当时的最新游标，此前的事件是回放的历史：照常列出，不计未读。
  const feedback = new EventSource('/api/feedback/stream');
  let history = null;   // {stream, cursor}：首次连接时的总线实例与最新游标；网关重启后实例变化，之后的都是新事件
  feedback.addEventListener('feedback.ready', event => {
    const ready = JSON.parse(event.data);
    if (!history) history = { stream: ready.stream_id, cursor: Number(ready.latest_cursor) };
  });
  feedback.addEventListener('feedback.message', event => {
    const message = JSON.parse(event.data);
    if (message.event_type === 'task.updated') return;
    const detail = message.event || {};
    const [stream, cursor] = event.lastEventId.split(':');
    addAttention({ id: message.message_id, tone: message.severity === 'error' ? 'bad' : 'warn', text: message.message,
      at: Date.parse(message.timestamp) || Date.now(), comp: message.source?.component || '', taskId: detail.task_id || null,
      requestId: detail.request_id || null }, Boolean(history) && stream === history.stream && Number(cursor) <= history.cursor);
  });

  /* ---- 启动与轮询（页面不可见时停止，恢复可见时立即取一次） ---- */
  void readHealth();
  void readManager();
  scheduleManager();
  setInterval(() => { if (!document.hidden) void readHealth(); }, HEALTH_INTERVAL_MS);
  document.addEventListener('visibilitychange', () => { if (!document.hidden) { void readHealth(); void readManager(); } });

  return { readHealth, readManager, markAttentionRead };
}
