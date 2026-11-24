/* 顶栏与面板共用的纯推导：一句话状态的取值顺序、任务呈现、设备指标与格式化。
   不访问页面元素，便于在 Node 中直接测试。 */

export const TERMINAL_TASK_STATES = ['succeeded', 'failed', 'cancelled', 'timed_out'];
// 一句话状态按任务类别选择说法与取消方式；导航用 cancel_task，探索用 stop_exploration。
const NAV_CAPS = new Set(['navigate_to_pose', 'navigate_to_named_location', 'navigate_to_place', 'navigate_to_floor']);
const EXPLORE_CAPS = new Set(['explore_area_v2', 'resume_exploration', 'explore_building']);
export const ESTOP_NOTICE_MS = 8000;

export function normalizeTaskState(value) {
  const raw = value && typeof value === 'object' && 'value' in value ? value.value : value;
  const text = String(raw || 'missing').toLowerCase();
  return text.startsWith('task.') ? text.slice(5) : text;
}
export function isTerminalTaskState(value) { return TERMINAL_TASK_STATES.includes(normalizeTaskState(value)); }
export function taskKind(task) { return NAV_CAPS.has(task.capability) ? 'nav' : EXPLORE_CAPS.has(task.capability) ? 'explore' : 'other'; }
// 发起方取任务快照的 caller：网页与接口调用为 http，智能体与 MCP 为 agent。
export function taskSource(task) { return task.caller === 'agent' ? '智能体' : '网页'; }

export function fmtDuration(ms) {
  const s = Math.max(0, Math.round(ms / 1000));
  const hours = Math.floor(s / 3600), minutes = Math.floor(s % 3600 / 60), seconds = s % 60;
  return (hours ? hours + ':' + String(minutes).padStart(2, '0') : String(minutes)) + ':' + String(seconds).padStart(2, '0');
}
export function fmtClock(ms) { return new Date(ms).toLocaleTimeString('zh-CN', { hour12: false }); }
export function fmtGB(bytes) { const n = Number(bytes); return Number.isFinite(n) && n >= 0 ? (n / 1024 ** 3).toFixed(1) : '--'; }

/* 任务呈现：类别、目标、进度（null 为不确定）、距离、用时、取消方式。
   目标名统一取任务快照的 data.target_label（导航由后端写入，不论谁发起），探索取 data.map_name；
   navRange 为导航任务见过的最大剩余距离，用来折算进度。 */
export function describeTask(task, { navRange, clockOffsetMs, now, pending, errors, capabilities }) {
  const kind = taskKind(task);
  const data = task.data || {};
  const target = data.target_label || (kind === 'explore' ? data.map_name || '' : '');
  const elapsedMs = Math.max(0, now + (clockOffsetMs || 0) - Number(task.created_at || 0) * 1000);
  const state = normalizeTaskState(task.state);
  const view = { task, kind, target, source: taskSource(task), elapsedMs, progress: null, distance: '', title: '',
    cancelling: state === 'cancelling' || pending.has(task.task_id), cancelError: errors.get(task.task_id) || '' };
  if (kind === 'nav') {
    view.title = '导航';
    const remaining = Number(data.remaining_distance_m);
    const range = navRange.get(task.task_id);
    if (Number.isFinite(remaining) && remaining >= 0) {
      view.distance = '剩余 ' + remaining.toFixed(1) + ' 米';
      if (range > 0) view.progress = Math.max(0, Math.min(1, 1 - remaining / range));
    }
    view.stopLabel = '取消导航';
  } else if (kind === 'explore') {
    view.title = '自动探索';
    const traveled = Number(data.exploration?.traveled_distance_m);
    if (Number.isFinite(traveled)) view.distance = '已行驶 ' + traveled.toFixed(1) + ' 米';
    view.stopLabel = '停止探索';
  } else {
    // 其它任务的名称取能力目录里的说明首句（产生方给出的中文），没有目录时用能力名。
    const description = capabilities?.get(task.capability)?.description || '';
    view.title = description.split(/[，。；]/)[0] || task.capability;
    const progress = Number(task.progress);
    if (Number.isFinite(progress) && progress > 0) view.progress = Math.min(1, progress);
    view.stopLabel = '取消';
  }
  return view;
}

/* 业务没在运行时的说法（顶栏、活动页、舞台遮罩共用）：
   updating 按管理任务的动作说“正在启动 / 停止 / 重启业务服务”“正在切换到新版本”等；failed 为启动失败，带原因；stopped 为已停止。
   返回 {tone, icon, title, text, meta, actionLabel}。 */
const JOB_DOING = { update: '正在切换到新版本', rollback: '正在切换到新版本', start: '正在启动业务服务', stop: '正在停止业务服务',
  restart: '正在重启业务服务', modules: '正在调整功能模块', install: '正在安装', 'update-manager': '正在更新管理程序' };
export function businessView(s) {
  if (s.business === 'updating') {
    const action = s.manager.activeJob?.action;
    const title = JOB_DOING[action];
    return { tone: 'busy', icon: 'download', title, text: title + (action === 'stop' ? '' : '，业务服务暂时停止'),
      meta: action === 'stop' ? '停止后地图和机器人数据暂不可用，可在系统页重新启动' : '完成后页面自动恢复，已保存的地图与配置不受影响',
      actionLabel: '查看进度' };
  }
  if (s.business === 'failed') {
    const failure = s.manager.startFailure;
    return { tone: 'bad', icon: 'circle-alert', title: '业务服务启动失败', text: '业务服务启动失败' + (failure.reason ? '：' + failure.reason : ''),
      meta: '失败在“' + failure.stageTitle + '”，在系统页查看组件日志、回退或修改配置', actionLabel: '查看原因' };
  }
  return { tone: 'bad', icon: 'power', title: '业务服务已停止', text: '业务服务已停止，地图和机器人数据暂不可用',
    meta: '可在系统页启动业务服务或查看原因', actionLabel: '打开系统页' };
}

/* 顶栏一句话状态：按“业务未运行 > 断连 > 服务退出 > 执行中任务 > 8 秒内急停 > 空闲”取第一条成立的。
   返回 {tone, icon, text, quote, meta[], action, progress, task}；meta 项为字符串或 {kind, ...}。 */
export function deriveActivity(s, now, extra) {
  if (s.business !== 'running') {
    const view = businessView(s);
    return { tone: view.tone, icon: view.icon, text: view.text, meta: [view.meta], action: { kind: 'system', label: view.actionLabel } };
  }
  if (s.conn.status === 'lost') {
    return { tone: 'bad', icon: 'unplug', text: '与网关的连接已断开，正在重连',
      meta: ['地图与状态暂停更新，已受理的任务继续执行'], action: { kind: 'reconnect', label: '立即重连' } };
  }
  const exited = Object.entries(s.services || {}).filter(([, item]) => item.state === 'exited');
  if (exited.length) {
    const [name, item] = exited[0];
    const code = item.exit_code != null ? '退出码 ' + item.exit_code : '';
    return { tone: 'bad', icon: 'circle-alert',
      text: (item.title || name) + '已退出' + (exited.length > 1 ? '，另有 ' + (exited.length - 1) + ' 个服务已退出' : ''),
      meta: [[code, item.error].filter(Boolean).join('：') || '监督者报告该服务已退出'],
      action: { kind: 'system', label: '查看服务' } };
  }
  const active = s.tasks.active;
  if (active.length) {
    // 焦点任务：最近开始的运动任务优先（导航、探索、遥控），其次最近开始的任务。
    const focus = active.filter(task => task.motion).sort(byCreated)[0] || active.slice().sort(byCreated)[0];
    const view = describeTask(focus, { ...extra, clockOffsetMs: s.clockOffsetMs, now, pending: s.tasks.pending, errors: s.tasks.errors });
    const meta = [{ kind: 'progress', value: view.progress }];
    if (view.distance) meta.push(view.distance);
    meta.push('用时 ' + fmtDuration(view.elapsedMs), { kind: 'source', text: view.source + '发起' });
    if (active.length > 1) meta.push('另有 ' + (active.length - 1) + ' 个任务');
    if (view.cancelError) meta.push({ kind: 'error', text: '取消失败：' + view.cancelError });
    return { tone: 'busy', icon: view.kind === 'nav' ? 'navigation' : view.kind === 'explore' ? 'compass' : 'hourglass',
      text: view.kind === 'nav' ? (view.target ? '正在导航，前往' : '正在导航') : view.kind === 'explore' ? '正在自动探索' : '正在执行：' + view.title,
      quote: view.target, meta, progress: view.progress, task: focus,
      action: { kind: 'stop', label: view.cancelling ? '正在取消…' : view.stopLabel, disabled: view.cancelling, task: focus } };
  }
  if (s.estop && now - s.estop.at < ESTOP_NOTICE_MS) {
    const e = s.estop;
    if (e.pending) return { tone: 'warn', icon: 'circle-stop', text: '正在急停…', meta: [fmtClock(e.at) + ' 由网页触发'] };
    if (e.error) return { tone: 'bad', icon: 'circle-stop', text: '急停请求没有送达', meta: [fmtClock(e.at) + '：' + e.error] };
    const meta = [fmtClock(e.at) + ' 由网页触发' + (e.cancelled ? '，已取消 ' + e.cancelled + ' 个任务' : '')];
    if (e.issues.length) meta.push(e.issues.join('；'));
    return { tone: 'warn', icon: 'circle-stop', text: '已急停，所有运动已停止', meta };
  }
  return { tone: 'idle', icon: 'circle-check', text: '空闲', meta: [idleDetail(s.geometry)] };
}
function byCreated(a, b) { return Number(b.created_at || 0) - Number(a.created_at || 0); }
/* 地图会话阶段（geometry.state.phase）的短说法与色调：地图卡徽标、舞台状态条共用；顶栏一句话的整句说法见 idleDetail。 */
export const GEOMETRY_PHASES = { idle: ['没有地图会话', 'off'], mapping: ['建图中', 'busy'], exploring: ['探索中', 'busy'],
  loaded: ['已加载，未定位', 'warn'], localized: ['已定位', 'ok'], fault: ['故障', 'bad'] };
export function idleDetail(geometry) {
  const name = geometry.mapName;
  if (!name || geometry.phase === 'idle') return '未加载地图';
  if (geometry.phase === 'mapping' || geometry.phase === 'exploring') return '正在建图「' + name + '」';
  if (geometry.phase === 'localized') return '已在「' + name + '」定位';
  if (geometry.phase === 'fault') return '「' + name + '」定位异常' + (geometry.reason ? '：' + geometry.reason : '');
  return '已加载「' + name + '」，尚未定位';
}

/* 设备指标：常态中性色，超过阈值才变色（电量 ≤20% 琥珀、≤10% 红；延迟 >200 毫秒琥珀；处理器、内存 ≥90% 琥珀）。 */
export function batteryTone(percent) { return percent == null ? '' : percent <= 10 ? 'bad' : percent <= 20 ? 'warn' : ''; }
export function latencyTone(ms) { return ms != null && ms > 200 ? 'warn' : ''; }
export function loadTone(percent) { return percent != null && percent >= 90 ? 'warn' : ''; }
