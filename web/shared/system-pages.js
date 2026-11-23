/* 系统页与首次安装向导：控制台“系统”页与设备管理恢复页挂载同一模块，数据全部来自设备管理程序 /api/manager/*。
   导出：
   - managerRequest(path, {method, body, timeout})：带会话校验值（X-NEH-CSRF）的请求；校验值过期时刷新重试，设了访问密码且未登录时弹出密码框，
     登录后重试一次（沿用原设备管理页 apiCall 的做法）。失败抛出带 status、code 的错误；连接中断或超时的错误带 network = true。
   - deriveStartFailure(status, jobs)：业务没运行、没有进行中的任务、最近一次任务在“启动业务”阶段失败时返回失败信息，否则 null（顶栏与系统页共用）。
   - mountSystem(root, opts) → {show(), hide(), refresh()}；mountWizard(root, opts) → {destroy()}。
   业务启停、回退、模块调整都走“生成计划 → 提交任务 → 读任务进度”（plan / jobs），版本更新走一步更新接口 update；
   页面可见时每 10 秒读业务状态，有进行中的任务时每 2 秒读任务进度，不可见时都停止。 */
import { icon, h, toast, confirmDialog, block as uiBlock, sub as uiSub, copyText } from './neh-ui.js';

/* ================= 管理服务请求 ================= */
let csrf = '';
let loginPending = null;
async function rawRequest(path, { method = 'GET', body, timeout = 30000 } = {}) {
  const headers = {};
  if (method !== 'GET') {
    headers['Content-Type'] = 'application/json';
    if (!csrf) csrf = (await rawRequest('session')).csrf_token || '';
    headers['X-NEH-CSRF'] = csrf;
  }
  let response;
  try {
    response = await fetch('/api/manager/' + path, { method, headers, body: body === undefined ? undefined : JSON.stringify(body),
      cache: 'no-store', credentials: 'same-origin', signal: AbortSignal.timeout(timeout) });
  } catch (error) {
    throw Object.assign(new Error(error.name === 'TimeoutError' ? '设备管理程序没有及时回应' : '与设备管理程序的连接中断'), { network: true });
  }
  let value = {};
  try { value = await response.json(); } catch (error) { value = {}; }
  if (!response.ok) {
    const detail = value.error || {};
    const extra = Array.isArray(detail.details) ? '：' + detail.details.join('；') : '';
    throw Object.assign(new Error((detail.message || '设备管理程序返回 ' + response.status) + extra),
      { status: response.status, code: detail.code || '', details: detail.details });
  }
  if (value.csrf_token) csrf = value.csrf_token;
  return value;
}
async function login() {
  if (!loginPending) loginPending = (async () => {
    let note = '设备管理程序设置了访问密码，输入后继续。';
    for (;;) {
      const input = h('input', { class: 'n-input', type: 'password', autocomplete: 'current-password' });
      setTimeout(() => input.focus(), 60);
      const ok = await confirmDialog({ title: '输入访问密码', text: note, confirmText: '登录',
        extra: h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '访问密码'), input) });
      if (!ok) throw new Error('需要访问密码才能继续');
      try { await rawRequest('login', { method: 'POST', body: { password: input.value } }); return; }
      catch (error) { note = error.message; }
    }
  })().finally(() => { loginPending = null; });
  return loginPending;
}
export async function managerRequest(path, init = {}) {
  try { return await rawRequest(path, init); }
  catch (error) {
    if (error.status === 403 && error.code === 'csrf_invalid') { csrf = ''; return rawRequest(path, init); }
    if (error.status === 401 && error.code === 'authentication_required') { await login(); csrf = ''; return rawRequest(path, init); }
    throw error;
  }
}

/* ================= 文字与格式 ================= */
// 任务阶段的中文名（沿用原设备管理页）；日志里保留原始阶段键。
export const STAGE_TITLE = { queued: '等待开始', running: '正在处理', preflight: '复核计划', recovering: '恢复原状态', configuration: '导入配置',
  source_download: '获取源码', prebuilt_pending: '等待云端编译', downloading: '下载程序', verifying: '校验程序', native_build: '构建原生组件',
  source_verify: '验证候选版本', waiting_idle: '等待安全空闲', stopping: '停止业务', switching: '切换版本', starting: '启动业务', complete: '完成',
  failed: '未完成', interrupted: '已中断', system_dependencies: '安装系统依赖', python_environment: '准备运行环境', dependencies: '下载运行依赖',
  runtime_files: '准备启动文件', models: '准备模型', rollback: '恢复原版本', manager_source: '准备管理程序', manager_download: '下载管理程序',
  manager_verify: '校验管理程序', manager_switch: '切换管理程序' };
export const ACTION_TITLE = { install: '首次安装', update: '更新版本', rollback: '回退到上一版本', modules: '调整功能模块',
  start: '启动业务服务', stop: '停止业务服务', restart: '重启业务服务', 'update-manager': '更新管理程序' };
const TERMINAL = new Set(['succeeded', 'failed', 'cancelled', 'interrupted']);
const QUICK = new Set(['start', 'stop', 'restart']);
const START_ACTIONS = new Set(['start', 'restart', 'install', 'update', 'modules', 'rollback']);
// 模块图标按含义选（方案第 6 节图标表）；名称与说明来自管理程序的模块目录。
const MODULE_ICON = { core: 'globe', geometry: 'map', slam2d: 'grid-2x2', fastlio2: 'axis-3d', pointlio: 'axis-3d', navigation: 'navigation',
  perception: 'scan-eye', follow: 'person-standing', memory: 'brain' };
const UPDATE_MODES = [['notify', '检查并提醒'], ['off', '关闭定时检查'], ['idle', '空闲时自动安装']];

const pad = n => String(n).padStart(2, '0');
const ms = v => (typeof v === 'number' ? v : Date.parse(v));
function dayDiff(d) {
  const n = new Date();
  return Math.round((new Date(n.getFullYear(), n.getMonth(), n.getDate()) - new Date(d.getFullYear(), d.getMonth(), d.getDate())) / 86400000);
}
// 今天 09:41 / 昨天 18:20 / 11/15 11:24
function fmtWhen(v, withSec) {
  const d = new Date(ms(v));
  if (Number.isNaN(d.getTime())) return '时间未知';
  const t = pad(d.getHours()) + ':' + pad(d.getMinutes()) + (withSec ? ':' + pad(d.getSeconds()) : '');
  const diff = dayDiff(d);
  if (diff === 0) return '今天 ' + t;
  if (diff === 1) return '昨天 ' + t;
  return (d.getFullYear() === new Date().getFullYear() ? '' : d.getFullYear() + '/') + pad(d.getMonth() + 1) + '/' + pad(d.getDate()) + ' ' + t;
}
const hms = v => { const d = new Date(ms(v)); return pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()); };
function fmtDur(sec) {
  sec = Math.max(0, Math.round(sec));
  if (sec < 60) return sec + ' 秒';
  const m = Math.floor(sec / 60), s = sec % 60;
  if (m < 60) return m + ' 分' + (s ? ' ' + s + ' 秒' : '');
  return Math.floor(m / 60) + ' 小时' + (m % 60 ? ' ' + (m % 60) + ' 分' : '');
}
function fmtBytes(bytes) {
  const n = Number(bytes);
  if (!Number.isFinite(n) || n <= 0) return '未知';
  if (n < 1024 ** 2) return (n / 1024).toFixed(0) + ' KB';
  if (n < 1024 ** 3) return (n / 1024 ** 2).toFixed(0) + ' MB';
  return (n / 1024 ** 3).toFixed(1) + ' GB';
}
function ago(t) { const m = Math.floor((Date.now() - t) / 60000); return m < 1 ? '刚刚' : m < 60 ? m + ' 分钟前' : m < 1440 ? Math.floor(m / 60) + ' 小时前' : fmtWhen(t); }
const spin = cls => h('span', { class: 'n-spin' + (cls ? ' ' + cls : ''), 'aria-hidden': 'true' });
const dotEl = kind => h('span', { class: 'n-dot n-dot--' + kind });
const kvPair = (label, value) => h('div', {}, h('dt', {}, label), h('dd', {}, value));
// 标题里的版本号用等宽字体，避免 i 和 l 难以分辨
const fmtTitle = t => String(t).split(/((?:git|dev)-[0-9a-z-]+)/).map((p, i) => (i % 2 ? h('span', { class: 's-ver' }, p) : p)).filter(Boolean);
const secHead = (title, meta, actions) => h('div', { class: 'n-sec-head' }, h('h2', {}, title),
  meta ? h('span', { class: 'n-meta' }, meta) : null, actions ? h('div', { class: 'n-sec-actions' }, actions) : null);
function lockedBtn(label, tip, ico, cls) {
  return h('span', { class: 'n-tip n-tip--below s-tip-end', 'data-tip': tip },
    h('button', { class: 'n-btn' + (cls ? ' ' + cls : ''), type: 'button', disabled: true, html: ico ? icon(ico, 'n-icon--sm') : '' }, label));
}

/* ================= 任务 ================= */
export function jobTitle(job) {
  const version = job.result?.version;
  if (job.action === 'update' && version) return '更新到 ' + version;
  if (job.action === 'rollback' && version) return '回退到 ' + version;
  return ACTION_TITLE[job.action] || job.action || '管理任务';
}
const jobRunning = job => job && !TERMINAL.has(job.status);
const jobSeconds = job => ((jobRunning(job) ? Date.now() : ms(job.updated_at)) - ms(job.created_at)) / 1000;
/* 失败在哪个阶段：取日志里最后一个不是“未完成 / 已中断 / 恢复”的阶段。 */
export function failedStage(job) {
  const log = job.log || [];
  for (let i = log.length - 1; i >= 0; i -= 1) if (!['failed', 'interrupted', 'recovering'].includes(log[i].stage)) return log[i].stage;
  return job.stage;
}
/* 启动失败：业务没运行、没有进行中的任务，最近一次任务失败在“启动业务”阶段（或错误码是启动失败、健康超时）。
   jobs 为 GET /api/manager/jobs 的 jobs（新在前）。 */
export function deriveStartFailure(status, jobs) {
  if (!status?.installation?.installed || status.runtime?.running || status.active_job) return null;
  const job = jobs?.[0];
  if (!job || job.status !== 'failed' || !START_ACTIONS.has(job.action)) return null;
  const stage = failedStage(job);
  if (stage !== 'starting' && !['start_failed', 'health_timeout'].includes(job.error?.code)) return null;
  return { jobId: job.id, action: job.action, at: job.updated_at, stage, stageTitle: STAGE_TITLE[stage] || stage,
    reason: job.error?.message || job.message || '', diagnostic: job.error?.details?.diagnostic || '' };
}
/* 步骤：日志里相邻的同一阶段合成一步，说明取这一阶段最后一条。 */
function stepsOf(job) {
  const steps = [];
  for (const entry of job.log || []) {
    const last = steps[steps.length - 1];
    if (last && last.stage === entry.stage) last.desc = entry.message;
    else steps.push({ stage: entry.stage, time: entry.time, desc: entry.message, state: 'done' });
  }
  const tail = steps[steps.length - 1];
  if (tail && jobRunning(job)) tail.state = 'active';
  if (tail && tail.stage === 'failed') {
    tail.state = job.status === 'cancelled' ? 'cancelled' : 'bad';
    if (job.status !== 'cancelled' && steps.length > 1) steps[steps.length - 2].state = 'bad';
  }
  return steps;
}
function stepList(job, compact) {
  return h('ol', { class: 'n-steps' + (compact ? ' s-steps-compact' : '') }, stepsOf(job).map(s => {
    const mark = h('span', { class: 'n-step-mark', html: s.state === 'done' ? icon('check') : s.state === 'bad' ? icon('x') : s.state === 'cancelled' ? icon('minus') : '' });
    if (s.state === 'active') mark.append(spin('s-spin-xs'));
    const cls = { done: ' n-step--done', active: ' n-step--active', bad: ' n-step--bad' }[s.state] || '';
    const label = s.stage === 'failed' && job.status === 'cancelled' ? '已取消' : STAGE_TITLE[s.stage] || s.stage;
    return h('li', { class: 'n-step' + cls }, mark,
      h('div', {}, h('div', { class: 'n-step-title' }, label), h('div', { class: 'n-step-sub' }, s.desc)),
      h('span', { class: 'n-step-time n-num' }, s.time ? hms(s.time) : ''));
  }));
}
function markFor(job) {
  if (jobRunning(job)) return h('span', { class: 's-mark s-mark--busy' }, spin());
  if (job.status === 'failed' || job.status === 'interrupted') return h('span', { class: 's-mark s-mark--bad', html: icon('x') });
  if (job.status === 'cancelled') return h('span', { class: 's-mark', html: icon('minus') });
  return h('span', { class: 's-mark s-mark--ok', html: icon('check') });
}
function resultOf(job) {
  if (jobRunning(job)) return ['s-res s-res--busy', (job.status === 'queued' ? '等待开始 ' : '进行中 ') + (job.progress || 0) + '%'];
  if (job.status === 'failed') return ['s-res s-res--bad', '失败'];
  if (job.status === 'interrupted') return ['s-res s-res--bad', '已中断'];
  if (job.status === 'cancelled') return ['s-res s-res--muted', '已取消'];
  return ['s-res', '已完成'];
}
const resEl = job => { const [cls, text] = resultOf(job); return h('span', { class: cls }, text); };
// 管理程序记录的失败原因之外，常见失败给一句下一步（不改变任务结果）。
function adviceOf(job) {
  if (job.status === 'cancelled') return '';
  if (['start_failed', 'health_timeout'].includes(job.error?.code) || failedStage(job) === 'starting')
    return '先看下面的组件日志找到退出的组件；改了配置后出现的，修改配置后重新启动；更新后才出现的，可以回退到上一版本。';
  if (job.action === 'update') return '设备保留原版本，业务服务按原版本运行。处理原因后可以重新更新。';
  if (job.action === 'install') return '可以返回安装计划，处理原因后重新安装；已下载的文件会复用。';
  return '';
}

/* ==========================================================================
   系统页
   opts.mode：'console' | 'recovery'
   opts.request：管理服务请求函数，默认 managerRequest
   opts.openSettings(panel)：“修改配置”“在设置里开启”跳到设置页
   opts.services(fn)：订阅服务进程表，fn({components, processes, telemetry})；不传时业务运行中每 3 秒读 /api/health
   opts.device()：控制台补充的设备信息 [[名称, 值]]
   opts.onState({code, text, tone})：业务状态变化时回调（恢复页顶栏用）
   ========================================================================== */
export function mountSystem(root, opts = {}) {
  const request = opts.request || managerRequest;
  const S = {
    status: null, settings: null, jobs: [], modules: null, hardware: null, job: null, error: '',
    open: { services: true, version: true, modules: true, records: true, options: false, device: false },
    drawer: null, recFilter: 'all', want: null, direct: new Map(), mgrDraft: null, sourceDraft: null, sourceOpen: false,
    checking: false, lastCheck: null, busy: '', jobOpen: false, visible: false,
    services: { components: null, processes: [], telemetry: 'missing' }, moduleTarget: null,
  };
  const output = { id: null, offset: 0, text: '', loading: false };
  const parts = {};   // 局部重绘用：大区、服务、版本、模块、记录各自的元素

  const toolbar = h('div', { class: 'n-toolbar' });
  const body = h('div', { class: 'n-page-body s-body' });
  const shell = h('div', { class: 's-root', dataset: { mode: opts.mode || 'console' } }, h('div', { class: 'n-page--full' }, toolbar, body));
  root.replaceChildren(shell);
  let drawerEl = null, scrimEl = null;

  /* ---------- 数据 ---------- */
  async function loadAll() {
    try {
      const [status, settings, jobs, modules] = await Promise.all([request('status'), request('settings'), request('jobs'), request('modules')]);
      applyStatus(status);
      S.settings = settings; S.jobs = jobs.jobs || []; S.modules = modules; S.error = '';
    } catch (error) { S.error = error.message; }
    renderAll();
  }
  function applyStatus(status) {
    const prev = S.status;
    // 业务版本或管理程序换了，页面也要换成新版本的文件（沿用原设备管理页与控制台软件更新的做法）。
    if (prev && (status.manager_version !== prev.manager_version || status.installation?.version !== prev.installation?.version)) location.reload();
    S.status = status;
    if (status.active_job && (!S.job || S.job.id !== status.active_job.id || TERMINAL.has(S.job.status))) S.job = status.active_job;
    if (!status.active_job && S.job && jobRunning(S.job)) void pollJob();
  }
  async function refreshStatus() {
    try { applyStatus(await request('status')); S.error = ''; }
    catch (error) { S.error = error.message; }
    renderAll();
  }
  async function refreshJobs() {
    try { S.jobs = (await request('jobs')).jobs || []; } catch (error) { /* 记录读取失败保留上次 */ }
  }
  let polling = false;
  async function pollJob() {
    if (!S.job || polling) return;
    polling = true;
    const id = S.job.id;
    try {
      const job = await request('jobs/' + encodeURIComponent(id));
      if (!S.job || S.job.id !== id) return;
      const finished = jobRunning(S.job) && !jobRunning(job);
      S.job = job;
      S.jobs = S.jobs.map(j => (j.id === job.id ? job : j));
      if (finished) await jobFinished(job);
      else { renderJobParts(); if (S.drawer?.type === 'record' && S.drawer.id === id) void pullOutput(id); }
    } catch (error) {
      S.error = error.message;
      renderToolbar();
    } finally { polling = false; }
  }
  async function jobFinished(job) {
    const title = jobTitle(job);
    if (job.status === 'succeeded') toast(job.action === 'update-manager' ? '管理程序已更新，页面会自动重新连接' : title + '已完成');
    else if (job.status === 'cancelled') toast(title + '已取消', { tone: 'info' });
    else toast(title + '没有完成：' + (job.error?.message || job.message), { tone: 'bad', duration: 8000 });
    await refreshJobs();
    await refreshStatus();
  }
  async function trackJob(job) {
    S.job = job;
    S.jobs = [job, ...S.jobs.filter(j => j.id !== job.id)];
    renderAll();
    void refreshStatus();
  }

  /* ---------- 页面可见时轮询 ---------- */
  let statusTimer = null, jobTimer = null, healthTimer = null;
  function startTimers() {
    stopTimers();
    statusTimer = setInterval(() => { if (!document.hidden) void refreshStatus(); }, 10000);
    jobTimer = setInterval(() => { if (!document.hidden && S.job && jobRunning(S.job)) void pollJob(); }, 2000);
    if (!opts.services) healthTimer = setInterval(() => { if (!document.hidden && S.status?.runtime?.running) void readHealth(); }, 3000);
  }
  function stopTimers() { clearInterval(statusTimer); clearInterval(jobTimer); clearInterval(healthTimer); }
  function onVisibility() { if (S.visible && !document.hidden) void refreshStatus(); }
  document.addEventListener('visibilitychange', onVisibility);
  async function readHealth() {
    try {
      const response = await fetch('/api/health', { cache: 'no-store' });
      if (!response.ok) return;
      S.services = { components: (await response.json()).processes || {}, processes: [], telemetry: 'missing' };
      renderServices();
    } catch (error) { /* 下一次重读 */ }
  }
  if (opts.services) opts.services(value => { S.services = value; renderServices(); });

  /* ---------- 业务状态 ---------- */
  const runtime = () => S.status?.runtime || {};
  const installation = () => S.status?.installation || {};
  const failure = () => deriveStartFailure(S.status, S.jobs);
  const longJob = () => (S.job && jobRunning(S.job) && !QUICK.has(S.job.action) ? S.job : null);
  const quickJob = () => (S.job && jobRunning(S.job) && QUICK.has(S.job.action) ? S.job : null);
  function business() {
    const job = S.job && jobRunning(S.job) ? S.job : null;
    if (job) {
      if (job.action === 'start') return { code: 'starting', text: '业务服务正在启动', tone: 'busy' };
      if (job.action === 'stop') return { code: 'stopping', text: '业务服务正在停止', tone: 'busy' };
      if (job.action === 'restart') return { code: 'restarting', text: '业务服务正在重启', tone: 'busy' };
      return { code: 'updating', text: '正在' + jobTitle(job) + ' ' + (job.progress || 0) + '%', tone: 'busy' };
    }
    if (runtime().running) return { code: 'running', text: runtime().healthy ? '业务服务运行中' : '业务服务运行中，网关尚未就绪', tone: runtime().healthy ? 'ok' : 'warn' };
    if (failure()) return { code: 'failed', text: '业务服务启动失败', tone: 'bad' };
    return { code: 'stopped', text: '业务服务已停止', tone: 'off' };
  }
  let lastState = '';
  function notifyState() {
    const b = business();
    const key = b.code + b.text;
    if (key !== lastState) { lastState = key; opts.onState?.(b); }
  }
  function uptimeText() {
    const sec = S.services.components?.gateway?.uptime_sec;
    if (!(sec >= 0)) return '';
    const m = Math.floor(sec / 60);
    return m < 1 ? '刚刚启动' : '已运行 ' + (m >= 60 ? Math.floor(m / 60) + ' 小时 ' : '') + (m % 60) + ' 分';
  }
  const versionLine = () => '版本 ' + (installation().version || '未知') + '，' + (S.settings?.channel || 'main') + ' 通道';

  /* ---------- 操作：计划 → 确认 → 提交任务 ---------- */
  async function runPlan(planRequest, confirm) {
    if (S.busy) return;
    S.busy = planRequest.action;
    renderAll();
    try {
      const plan = await request('plan', { method: 'POST', body: planRequest, timeout: 120000 });
      if (plan.blockers?.length) {
        await confirmDialog({ title: '现在不能' + (ACTION_TITLE[plan.action] || '执行'), icon: 'circle-alert', text: '管理程序报告以下条件还没满足：', items: plan.blockers, confirmText: '知道了' });
        return;
      }
      if (confirm && !await confirm(plan)) return;
      await trackJob(await request('jobs', { method: 'POST', body: { plan_id: plan.id } }));
    } catch (error) {
      toast((ACTION_TITLE[planRequest.action] || '操作') + '没有开始：' + error.message, { tone: 'bad', duration: 8000 });
    } finally { S.busy = ''; renderAll(); }
  }
  function startService() { void runPlan({ action: 'start' }); }
  function stopService() {
    void runPlan({ action: 'stop' }, plan => confirmDialog({ title: '停止业务服务', danger: true, confirmText: '停止业务服务', items: plan.warnings,
      text: '停止后控制台和机器人接口不可用，正在执行的导航、跟随等任务会被中断，机器人会原地停下。' + (opts.mode === 'console' ? '这个页面会断开，可以从设备管理地址重新启动。' : '') }));
  }
  function restartService() {
    void runPlan({ action: 'restart' }, plan => confirmDialog({ title: '重启业务服务', icon: 'triangle-alert', confirmText: '重启业务服务', items: plan.warnings,
      text: '业务服务会先停止再启动，通常需要 20 秒到 1 分钟。期间正在执行的任务会被中断，控制台会暂时断开并自动重连。' }));
  }
  function confirmRollback() {
    const previous = installation().previous_version;
    void runPlan({ action: 'rollback', stop_running: true }, plan => confirmDialog({ title: '回退到 ' + previous, icon: 'triangle-alert', confirmText: '回退到此版本',
      items: plan.warnings, text: '回退使用设备上保留的上一版本，不需要重新构建。业务服务会停止并用这个版本重新启动，正在执行的任务会被中断，控制台会断开约 1 分钟。' }));
  }
  async function confirmUpdate() {
    const update = S.status.update || {};
    const local = h('input', { type: 'checkbox' });
    const extra = h('div', { class: 's-dlg-extra' },
      h('ol', { class: 's-plan' }, ['获取源码，下载云端预编译组件', '在本机增量构建，复用已有缓存', '验证候选版本', '停止业务服务并切换版本，约 1 分钟', '启动业务服务并检查健康状态']
        .map(t => h('li', {}, h('span', { class: 's-plan-title' }, t)))),
      update.requires_confirmation ? h('p', { class: 's-dlg-note' }, '这个版本标记为需要人工确认，自动更新不会安装它。') : null,
      h('label', { class: 's-option' }, local, h('span', { class: 's-option-title' }, '云端未就绪时立即本机编译'),
        h('span', { class: 's-option-help' }, '云端预编译还没完成时不等待，直接在本机编译。本机编译耗时较长，期间处理器占用较高。')));
    const ok = await confirmDialog({ title: '更新到 ' + (update.display_version || update.version), icon: 'triangle-alert', extra, confirmText: '开始更新',
      text: '新版本先在后台准备，准备好后停止业务服务、切换版本并重新启动。切换期间正在执行的任务会被中断，控制台会断开约 1 分钟。' });
    if (!ok || S.busy) return;
    S.busy = 'update';
    renderAll();
    try {
      const result = await request('update', { method: 'POST', body: { local_build: local.checked }, timeout: 120000 });
      if (result.status === 'deferred') toast(result.message || '通道还没有可采用的新版本，保留当前安装', { tone: 'info', duration: 8000 });
      else await trackJob(result);
    } catch (error) { toast('更新没有开始：' + error.message, { tone: 'bad', duration: 8000 }); }
    finally { S.busy = ''; renderAll(); }
  }
  function updateManager() {
    const version = S.status.update?.manager_version;
    void runPlan({ action: 'update-manager', version: version || undefined }, plan => confirmDialog({ title: '更新管理程序', confirmText: '更新管理程序', items: plan.warnings,
      text: '把管理程序更新到 ' + version + '。更新时管理页面会短暂断开并自动恢复，机器人业务不受影响。' }));
  }
  async function cancelJob(job) {
    const ok = await confirmDialog({ title: '取消' + jobTitle(job), icon: 'triangle-alert', confirmText: '确认取消',
      text: '任务会在当前步骤的安全位置停止，已完成的步骤不会回退；如果业务服务已经停下，会按原版本重新启动。' });
    if (!ok) return;
    try { S.job = await request('jobs/' + encodeURIComponent(job.id) + '/cancel', { method: 'POST', body: {} }); toast('已请求取消，正在等待安全位置', { tone: 'info' }); renderAll(); }
    catch (error) { toast('取消没有送达：' + error.message, { tone: 'bad' }); }
  }
  async function checkUpdates() {
    S.checking = true; renderToolbar();
    try {
      const releases = await request('releases?refresh=1', { timeout: 120000 });
      S.lastCheck = Date.now();
      await refreshStatus();
      const update = S.status.update || {};
      toast(releases.error ? '检查新版本失败：' + releases.error : update.available ? '有新版本 ' + (update.display_version || update.version)
        : releases.pending ? (releases.message || '通道稍后再查') : '已是最新版本', { tone: releases.error ? 'bad' : 'info' });
    } catch (error) { toast('检查新版本失败：' + error.message, { tone: 'bad' }); }
    finally { S.checking = false; renderAll(); }
  }

  /* ---------- 渲染 ---------- */
  const block = (id, conf) => uiBlock(Object.assign({ open: S.open[id], onToggle: v => { S.open[id] = v; if (id === 'device' && v) void loadHardware(); renderToolbar(); } }, conf));
  function renderAll() { renderToolbar(); renderBody(); renderDrawer(); notifyState(); }
  function renderToolbar() {
    const allOpen = Object.values(S.open).every(Boolean);
    const checked = S.status?.update?.check?.checked_at ? ms(S.status.update.check.checked_at) : S.lastCheck;
    toolbar.replaceChildren(...h('div', {}, h('h1', {}, '系统'),
      h('span', { class: 'n-meta s-toolbar-meta' }, checked ? '上次检查新版本 ' + ago(checked) : '还没有检查过新版本'),
      S.error ? h('span', { class: 'n-status s-bad', title: S.error }, dotEl('bad'), '读取失败，正在重试') : null,
      h('span', { class: 'n-toolbar-spacer' }),
      h('button', { class: 'n-btn n-btn--sm n-btn--quiet', type: 'button', disabled: S.checking || !S.status, html: S.checking ? '' : icon('refresh-cw', 'n-icon--sm'), onclick: checkUpdates },
        S.checking ? [spin('s-spin-xs'), '正在检查'] : '检查新版本'),
      h('button', { class: 'n-btn n-btn--sm n-btn--quiet', type: 'button', html: icon(allOpen ? 'chevrons-down-up' : 'chevrons-up-down', 'n-icon--sm'),
        onclick: () => { Object.keys(S.open).forEach(k => { S.open[k] = !allOpen; }); if (!allOpen) void loadHardware(); renderAll(); } }, allOpen ? '全部收起' : '全部展开')).childNodes);
  }
  function renderBody() {
    if (!S.status) {
      body.replaceChildren(S.error ? h('div', { class: 'n-empty', html: icon('circle-alert') }, h('div', { class: 'n-empty-title' }, '读不到设备管理程序'), h('div', {}, S.error),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: loadAll }, '重新读取'))
        : h('div', { class: 'n-empty' }, spin(), h('div', {}, '正在读取设备状态…')));
      return;
    }
    const keep = body.scrollTop;
    const bar = h('div', { class: 'n-savebar', role: 'region', 'aria-label': '未保存的管理选项', hidden: true });
    parts.hero = heroEl(); parts.services = servicesBlock(); parts.version = versionBlock(); parts.modules = modulesBlock(); parts.records = recordsBlock();
    body.replaceChildren(h('div', { class: 's-stack' },
      parts.hero, bannerEl(),
      h('div', { class: 'n-cols-2 s-pair' }, parts.services, parts.version),
      h('div', { class: 'n-cols-2 s-pair' }, parts.modules, parts.records),
      optionsBlock(bar), deviceBlock()), bar);
    body.scrollTop = keep;
  }
  // 任务进度只换大区、版本与记录，不重建整页（输入框和展开状态保持）。
  function renderJobParts() {
    if (!parts.hero?.isConnected) { renderAll(); return; }
    for (const [key, build] of [['hero', heroEl], ['version', versionBlock], ['records', recordsBlock], ['modules', modulesBlock]]) {
      const next = build();
      parts[key].replaceWith(next);
      parts[key] = next;
    }
    if (S.drawer?.type === 'record' || S.drawer?.type === 'records') renderDrawer();
    notifyState();
  }
  function renderServices() {
    if (!parts.services?.isConnected) return;
    const next = servicesBlock();
    parts.services.replaceWith(next);
    parts.services = next;
    if (!longJob()) { const hero = heroEl(); parts.hero.replaceWith(hero); parts.hero = hero; }
  }

  /* ---------- 状态大区 ---------- */
  function heroEl() {
    const job = longJob();
    if (job) return jobHero(job);
    const quick = quickJob();
    const f = failure();
    const b = business();
    const tone = quick ? '' : b.tone === 'warn' ? 'warn' : b.tone;
    const sub = quick ? (quick.message || '') : f ? '在“' + f.stageTitle + '”阶段失败，' + fmtWhen(f.at) + '。' + versionLine() + '。'
      : runtime().running ? [uptimeText(), versionLine()].filter(Boolean).join('，') + (runtime().error ? '。' + runtime().error : '')
        : versionLine() + '。控制台和机器人接口暂不可用，启动后大约 20 秒可以使用。';
    return h('section', { class: 'n-hero s-hero' + (tone ? ' n-hero--' + tone : ''), 'aria-label': '业务服务状态' },
      h('span', { class: 'n-hero-icon', html: quick ? '' : icon(f ? 'circle-alert' : 'power') }, quick ? spin('s-spin-hero') : null),
      h('div', { class: 'n-hero-main' }, h('h2', { class: 'n-hero-title' }, b.text), h('p', { class: 'n-hero-sub' }, sub)),
      quick ? null : heroActions(),
      f ? failMore(f) : null);
  }
  function heroActions() {
    const pending = S.busy;
    if (!runtime().running) return h('div', { class: 'n-hero-actions' },
      h('button', { class: 'n-btn n-btn--primary', type: 'button', disabled: Boolean(pending), html: pending === 'start' ? '' : icon('play', 'n-icon--sm'), onclick: startService },
        pending === 'start' ? [spin('s-spin-xs'), '正在提交'] : '启动业务服务'));
    return h('div', { class: 'n-hero-actions' },
      h('button', { class: 'n-btn', type: 'button', disabled: Boolean(pending), html: icon('rotate-ccw', 'n-icon--sm'), onclick: restartService }, '重启业务服务'),
      h('button', { class: 'n-btn n-btn--danger', type: 'button', disabled: Boolean(pending), html: icon('circle-stop', 'n-icon--sm'), onclick: stopService }, '停止业务服务'));
  }
  function failMore(f) {
    return h('div', { class: 's-hero-more' }, h('div', { class: 'n-hl n-hl--bad s-fail', role: 'alert' },
      h('dl', { class: 's-fail-kv' }, h('dt', {}, '原因'), h('dd', {}, f.reason), h('dt', {}, '建议'), h('dd', {}, adviceOf(S.jobs[0]))),
      f.diagnostic ? h('details', { class: 's-diag' }, h('summary', { html: icon('chevron-right', 'n-icon--sm') + '组件日志末尾' }), h('pre', { class: 'n-pre n-terminal' }, f.diagnostic)) : null,
      h('div', { class: 's-fail-actions' },
        h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('scroll-text', 'n-icon--sm'), onclick: () => openDrawer({ type: 'record', id: f.jobId }) }, '查看日志'),
        installation().previous_version ? h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('history', 'n-icon--sm'), onclick: confirmRollback }, '回退到上一版本') : null,
        opts.openSettings ? h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('sliders-horizontal', 'n-icon--sm'), onclick: () => opts.openSettings() }, '修改配置') : null)));
  }
  function jobBizLine(job) {
    if (['stopping', 'switching'].includes(job.stage)) return '业务服务已暂停，完成后会自动启动。';
    if (job.stage === 'starting') return '正在启动业务服务。';
    if (job.action === 'update-manager') return '机器人业务不受影响。';
    return runtime().running ? '业务服务仍在运行，切换时会停止约 1 分钟。' : '';
  }
  function jobHero(job) {
    const percent = (job.progress || 0) + '%';
    const log = job.log || [];
    return h('section', { class: 'n-hero s-hero s-hero--busy', 'aria-label': '正在执行的任务' },
      h('span', { class: 'n-hero-icon' }, spin('s-spin-hero')),
      h('div', { class: 'n-hero-main' }, h('h2', { class: 'n-hero-title' }, job.status === 'queued' ? '等待开始：' : '正在', fmtTitle(jobTitle(job))),
        h('p', { class: 'n-hero-sub' }, h('b', { class: 's-hero-stage' }, STAGE_TITLE[job.stage] || job.stage), '：' + (job.message || ''))),
      h('div', { class: 'n-hero-actions' }, h('span', { class: 'n-t1 n-num' }, percent),
        h('button', { class: 'n-btn', type: 'button', onclick: () => cancelJob(job) }, '取消'),
        h('button', { class: 'n-btn n-btn--quiet', type: 'button', 'aria-expanded': String(S.jobOpen), html: icon(S.jobOpen ? 'chevron-up' : 'chevron-down', 'n-icon--sm'),
          onclick: () => { S.jobOpen = !S.jobOpen; renderJobParts(); } }, S.jobOpen ? '收起步骤' : '展开步骤')),
      h('div', { class: 's-hero-more' },
        h('div', { class: 'n-progress s-hero-bar' }, h('span', { style: 'width:' + percent })),
        h('p', { class: 'n-meta' }, jobBizLine(job) + '开始于 ' + fmtWhen(job.created_at) + '，已用时 ' + fmtDur(jobSeconds(job)) + '。'),
        S.jobOpen ? h('div', { class: 's-job-more' }, stepList(job, true),
          h('div', { class: 'n-pre n-terminal s-tail', 'aria-label': '最新步骤' }, log.slice(-8).map(l => h('div', { class: 's-tail-line' },
            h('span', { class: 't-time' }, hms(l.time) + ' [' + l.stage + '] '), l.message)))) : null));
  }
  // 新版本横幅：只在业务正常运行、没有任务和故障时出现，按钮是本页唯一的实心主按钮。
  function bannerEl() {
    const update = S.status.update || {};
    if (!update.available || !runtime().running || S.job && jobRunning(S.job)) return null;
    return h('section', { class: 'n-hl s-banner', 'aria-label': '有新版本' },
      h('span', { class: 's-banner-icon', html: icon('download') }),
      h('div', { class: 's-banner-main' }, h('div', { class: 'n-t3' }, '有新版本 ', h('span', { class: 's-ver' }, update.display_version || update.version)),
        h('span', { class: 'n-meta' }, [(S.settings?.channel || 'main') + ' 通道', update.revision ? '提交 ' + update.revision.slice(0, 12) : '',
          update.last_failure ? '上次更新到此版本失败：' + update.last_failure.message : ''].filter(Boolean).join('，'))),
      h('button', { class: 'n-btn n-btn--primary', type: 'button', disabled: Boolean(S.busy), html: icon('download', 'n-icon--sm'), onclick: confirmUpdate }, '更新到此版本'));
  }

  /* ---------- 服务：组件名与状态来自监督者，处理器与内存按进程号取性能监控 ---------- */
  function servicesBlock() {
    const components = S.services.components;
    const running = runtime().running;
    const entries = running && components ? Object.entries(components) : [];
    const exited = entries.filter(([, c]) => c.state === 'exited').length;
    const live = entries.filter(([, c]) => c.state === 'running').length;
    const el = block('services', { icon: 'server', title: '服务', state: live ? 'on' : 'off',
      summary: !running ? '业务服务没有运行' : !components ? '正在读取服务状态' : entries.length + ' 个组件，' + (exited ? exited + ' 个已退出' : live + ' 个运行中') });
    if (!running || !components) {
      el.body.append(h('div', { class: 'n-block-pad n-meta' }, running ? '正在读取服务状态…' : '业务服务没有运行，各组件都已停止。启动业务服务后在这里看每个组件的状态。'));
      return el;
    }
    const processes = new Map((S.services.processes || []).map(p => [String(p.pid), p]));
    const rows = entries.map(([name, c]) => {
      const proc = c.pid ? processes.get(String(c.pid)) : null;
      const dot = c.state === 'running' ? 'ok' : c.state === 'starting' ? 'busy' : c.state === 'exited' ? 'bad' : 'off';
      const min = c.uptime_sec >= 0 ? Math.floor(c.uptime_sec / 60) : null;
      const state = c.state === 'running' ? '运行 ' + (min == null ? '' : min < 1 ? '刚刚启动' : min >= 60 ? Math.floor(min / 60) + ' 小时' : min + ' 分')
        : c.state === 'exited' ? '已退出' + (c.exit_code != null ? '，代码 ' + c.exit_code : '') + (c.error ? '：' + c.error : '') : c.state_title || c.state;
      return h('div', { class: 's-svc' + (c.state === 'exited' ? ' s-svc--bad' : c.state === 'running' || c.state === 'starting' ? '' : ' s-svc--off'), role: 'row', title: c.pid ? '进程号 ' + c.pid : '' },
        h('span', { class: 's-svc-name', role: 'cell' }, c.state === 'starting' ? spin('s-spin-xs') : dotEl(dot), h('span', {}, c.title || name)),
        h('span', { class: 's-svc-state', role: 'cell', title: state }, h('span', {}, state),
          c.state === 'exited' && opts.openLogs ? h('button', { class: 's-link', type: 'button', onclick: () => opts.openLogs(name) }, '日志') : null),
        h('span', { class: 's-svc-num', role: 'cell' }, proc ? Number(proc.cpu_percent).toFixed(1) + '%' : ''),
        h('span', { class: 's-svc-num', role: 'cell' }, proc ? fmtBytes(proc.rss_bytes) : ''));
    });
    const hint = opts.services && S.services.telemetry !== 'available'
      ? h('p', { class: 'n-meta s-svc-hint' }, '性能监控没有开启，处理器与内存不显示；', opts.openSettings ? h('button', { class: 's-link', type: 'button', onclick: () => opts.openSettings('性能监控') }, '在设置里开启') : '可在设置的“性能监控”开启') : null;
    el.body.append(h('div', { class: 's-svc-list', role: 'table', 'aria-label': '服务' },
      h('div', { class: 's-svc s-svc-head', role: 'row' }, h('span', { role: 'columnheader' }, '名称'), h('span', { class: 's-svc-state', role: 'columnheader' }, '状态'),
        h('span', { class: 's-svc-num', role: 'columnheader' }, '处理器'), h('span', { class: 's-svc-num', role: 'columnheader' }, '内存')), rows), ...(hint ? [hint] : []));
    return el;
  }

  /* ---------- 版本：新版本 → 当前版本 → 上一版本，下面是管理程序与更新来源 ---------- */
  function tlItem(dot, dotHtml, head, meta, action) {
    return h('li', { class: 'n-tl-item' }, h('span', { class: 'n-tl-dot' + (dot ? ' n-tl-dot--' + dot : ''), html: dotHtml || '' }),
      h('div', { class: 's-tl-main' }, h('div', { class: 's-tl-head' }, head), meta), h('div', { class: 's-tl-action' }, action));
  }
  function versionAction(kind) {
    const job = S.job && jobRunning(S.job) ? S.job : null;
    if (job && job.action === kind) {
      const p = (job.progress || 0) + '%';
      return h('div', { class: 's-progress-mini' }, h('span', { class: 's-progress-text' }, (kind === 'update' ? '正在更新 ' : '正在回退 ') + p), h('div', { class: 'n-progress' }, h('span', { style: 'width:' + p })));
    }
    const label = kind === 'update' ? '更新到此版本' : '回退到此版本';
    const ico = kind === 'update' ? 'download' : 'history';
    if (job) return lockedBtn(label, '正在执行“' + jobTitle(job) + '”，完成后才能操作', ico, 'n-btn--sm');
    return h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: Boolean(S.busy), html: icon(ico, 'n-icon--sm'), onclick: kind === 'update' ? confirmUpdate : confirmRollback }, label);
  }
  function versionBlock() {
    const inst = installation(), update = S.status.update || {};
    const items = [];
    const updating = S.job && jobRunning(S.job) && S.job.action === 'update';
    if (update.available || updating) items.push(tlItem(updating ? 'busy' : 'new', updating ? spin('s-spin-xs').outerHTML : '',
      [h('span', { class: 's-ver' }, update.display_version || update.version || '新版本'), h('span', { class: 'n-badge n-badge--accent' }, '新版本')],
      h('p', { class: 'n-meta' }, [(S.settings?.channel || 'main') + ' 通道', update.revision ? '提交 ' + update.revision.slice(0, 12) : '',
        update.requires_confirmation ? '需要人工确认' : ''].filter(Boolean).join('，')),
      bannerEl() ? null : versionAction('update')));
    const relation = { older: '当前版本领先通道', diverged: '当前版本与通道已分叉' }[update.relation];
    const note = update.error ? '检查新版本失败：' + update.error : update.pending ? update.message || '通道稍后再查' : !update.available && relation ? relation : '';
    items.push(tlItem('current', icon('check'),
      [h('span', { class: 's-ver' }, inst.version || '未知'), h('span', { class: 'n-badge' }, '当前版本')],
      h('p', { class: 'n-meta' }, [inst.revision ? '提交 ' + inst.revision.slice(0, 12) : '', { git: '源码构建' }[inst.source] || inst.source || '', business().text].filter(Boolean).join('，'),
        note ? h('span', { class: 's-ver-note' + (update.error ? ' s-bad' : '') }, note) : null), null));
    if (inst.previous_version) items.push(tlItem('', '', [h('span', { class: 's-ver' }, inst.previous_version), h('span', { class: 'n-badge n-badge--outline' }, '上一版本')],
      h('p', { class: 'n-meta' }, '设备上保留的版本，回退不需要重新构建'), versionAction('rollback')));
    const el = block('version', { icon: 'git-branch', title: '版本',
      summary: '当前 ' + (inst.version || '未知') + (update.available ? '，有新版本' : update.error ? '，检查新版本失败' : '，已是最新') });
    el.body.append(h('div', { class: 'n-block-pad' }, h('ol', { class: 'n-tl s-ver-tl' }, items)), managerRow(), sourceSub());
    return el;
  }
  function managerRow() {
    const st = S.status, update = st.update || {};
    const job = S.job && jobRunning(S.job) ? S.job : null;
    return h('div', { class: 's-mgr-row' },
      h('span', { class: 's-mgr-icon', html: icon('cpu', 'n-icon--sm') }),
      h('div', { class: 's-tl-main' }, h('div', { class: 's-tl-head' }, h('span', { class: 's-mgr-name' }, '管理程序'),
        h('span', { class: 's-ver' }, st.manager_development ? '开发版本' : st.manager_version || '未知')),
        h('p', { class: 'n-meta' }, update.manager_available ? '可更新到 ' + update.manager_version + '，更新时管理页面短暂断开，业务不受影响' : update.manager_version ? '已是最新' : '还没有读到管理程序的发布信息')),
      update.manager_available ? (job ? lockedBtn('更新管理程序', '正在执行“' + jobTitle(job) + '”，完成后才能操作', '', 'n-btn--sm')
        : h('button', { class: 'n-btn n-btn--sm', type: 'button', disabled: Boolean(S.busy), onclick: updateManager }, '更新管理程序')) : h('span'));
  }
  function sourceSub() {
    const saved = S.settings || {};
    const d = S.sourceDraft || { branch: saved.source_branch || 'main', repo: saved.source_repository || '' };
    const subEl = uiSub({ title: '更新来源', count: '源码分支 ' + (saved.source_branch || 'main') + '，仓库 ' + String(saved.source_repository || '未设置').replace(/^https?:\/\//, ''),
      open: S.sourceOpen, onToggle: v => { S.sourceOpen = v; } });
    const branch = h('input', { class: 'n-input n-input--code', value: d.branch, spellcheck: 'false', 'aria-label': '源码分支' });
    const repo = h('input', { class: 'n-input n-input--code', value: d.repo, spellcheck: 'false', 'aria-label': '源码仓库地址' });
    const actions = h('div', { class: 's-src-actions', hidden: !S.sourceDraft },
      h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => { S.sourceDraft = null; renderBody(); } }, '放弃改动'),
      h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: async () => {
        try {
          S.settings = await request('settings', { method: 'PUT', body: { source_repository: repo.value.trim(), source_branch: branch.value.trim() || 'main' } });
          S.sourceDraft = null; renderBody(); toast('已保存更新来源，下次检查新版本时使用');
        } catch (error) { toast('更新来源没有保存：' + error.message, { tone: 'bad', duration: 8000 }); }
      } }, '保存更新来源'));
    const sync = () => {
      const changed = branch.value !== (saved.source_branch || 'main') || repo.value !== (saved.source_repository || '');
      S.sourceDraft = changed ? { branch: branch.value, repo: repo.value } : null;
      actions.hidden = !changed;
    };
    branch.addEventListener('input', sync); repo.addEventListener('input', sync);
    subEl.body.append(h('div', { class: 's-src-body' },
      h('div', { class: 's-src-grid' }, h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '源码分支'), branch),
        h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '源码仓库地址'), repo)),
      h('p', { class: 'n-meta' }, '本机构建时从这里获取源码，一般与更新通道保持一致，不需要修改。'), actions));
    return subEl;
  }

  /* ---------- 功能模块：磁贴，未安装为虚线；调整在抽屉里进行 ---------- */
  const moduleList = () => S.modules?.modules || [];
  const installedSet = () => new Set(S.modules?.selected || installation().modules || []);
  const titleOf = key => moduleList().find(m => m.key === key)?.title || key;
  const titles = keys => moduleList().filter(m => keys.includes(m.key)).map(m => m.title).join('、');
  function modulesBlock() {
    const installed = installedSet();
    const job = S.job && jobRunning(S.job) ? S.job : null;
    const el = block('modules', { icon: 'layers', title: '功能模块', summary: '已安装 ' + installed.size + ' 个，共 ' + moduleList().length + ' 个',
      aside: job ? lockedBtn('调整模块', '正在执行“' + jobTitle(job) + '”，完成后才能调整', 'sliders-horizontal', 'n-btn--sm')
        : h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('sliders-horizontal', 'n-icon--sm'), onclick: () => openDrawer({ type: 'modules' }) }, '调整模块') });
    // 本页提交的模块调整记得目标组合，进行中时在变化的磁贴上标出（任务记录里没有目标组合）。
    const target = job?.action === 'modules' ? S.moduleTarget : null;
    el.body.append(h('div', { class: 'n-block-pad s-tiles-box' }, h('div', { class: 'n-tiles s-tiles' }, moduleList().map(m => {
      const inst = installed.has(m.key);
      const change = target && target.has(m.key) !== inst ? (inst ? '正在移除' : '正在安装') : null;
      return h('div', { class: 'n-tile s-mod-tile' + (inst ? '' : ' n-tile--dashed'), title: m.description },
        h('div', { class: 'n-tile-head' }, h('span', { class: 'n-tile-icon', html: icon(MODULE_ICON[m.key] || 'puzzle') }), h('span', { class: 'n-tile-name' }, m.title)),
        h('div', { class: 'n-tile-sub s-tile-brief' }, m.description),
        h('div', { class: 's-tile-foot' }, change ? h('span', { class: 'n-badge n-badge--accent' }, change) : h('span', { class: 'n-meta' }, inst ? '已安装' : '未安装')));
    }))));
    return el;
  }
  function requiresOf(key) {
    const out = new Set();
    const walk = k => (moduleList().find(m => m.key === k)?.requires || []).forEach(r => { if (!out.has(r)) { out.add(r); walk(r); } });
    walk(key); return [...out];
  }
  function dependantsOf(key) {
    const out = new Set();
    const walk = k => moduleList().forEach(m => { if (m.requires.includes(k) && !out.has(m.key)) { out.add(m.key); walk(m.key); } });
    walk(key); return [...out];
  }

  /* ---------- 安装与更新记录：时间线，点一项在抽屉里看详情 ---------- */
  function recItem(job) {
    const running = jobRunning(job);
    const kind = running ? 'busy' : ['failed', 'interrupted'].includes(job.status) ? 'bad' : job.status === 'cancelled' ? '' : 'ok';
    const dot = h('span', { class: 'n-tl-dot' + (kind ? ' n-tl-dot--' + kind : ''), html: running ? '' : icon(kind === 'bad' ? 'x' : kind ? 'check' : 'minus') });
    if (running) dot.append(spin('s-spin-xs'));
    const go = () => openDrawer({ type: 'record', id: job.id });
    return h('li', { class: 'n-tl-item s-tl-click', role: 'button', tabindex: '0', 'aria-label': jobTitle(job), onclick: go,
      onkeydown: e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); go(); } } },
      dot, h('div', { class: 's-tl-main' }, h('div', { class: 's-tl-title' }, fmtTitle(jobTitle(job))), h('p', { class: 'n-meta' }, fmtWhen(job.created_at) + '，用时 ' + fmtDur(jobSeconds(job)))),
      h('div', { class: 's-tl-action' }, resEl(job), h('span', { class: 's-chev', html: icon('chevron-right', 'n-icon--sm') })));
  }
  function recordsBlock() {
    const jobs = S.jobs;
    const last = jobs[0];
    const el = block('records', { icon: 'history', title: '安装与更新记录',
      summary: last ? '最近一次：' + fmtWhen(last.created_at) + ' ' + jobTitle(last) + '，' + resultOf(last)[1] : '还没有记录',
      aside: h('button', { class: 'n-btn n-btn--sm n-btn--quiet', type: 'button', onclick: () => openDrawer({ type: 'records' }) }, '全部记录') });
    el.body.append(h('div', { class: 'n-block-pad' }, jobs.length ? h('ol', { class: 'n-tl s-rec-tl' }, jobs.slice(0, 5).map(recItem))
      : h('p', { class: 'n-meta' }, '安装、更新、模块调整和启停业务服务后，记录会显示在这里。')));
    return el;
  }

  /* ---------- 管理选项（默认收起）：属性表；前四项改动后出现保存条，密码与开机启动即时生效 ---------- */
  function prop(name, control, desc, key) {
    return h('div', { class: 'n-prop', dataset: key ? { key } : null },
      h('div', { class: 'n-prop-name' }, h('b', {}, name)), h('div', { class: 's-prop-ctl' }, control), h('p', { class: 'n-prop-desc' }, desc));
  }
  function optionsBlock(bar) {
    const saved = S.settings || {};
    const base = { channel: saved.channel || 'main', mode: saved.auto_update || 'notify', window: saved.maintenance_window || '', interval: String(saved.check_interval_minutes ?? 30) };
    const d = S.mgrDraft || base;
    const channel = h('input', { class: 'n-input n-input--code', value: d.channel, placeholder: 'main', spellcheck: 'false', 'aria-label': '更新通道' });
    const mode = h('select', { class: 'n-select', 'aria-label': '更新方式' }, UPDATE_MODES.map(([v, t]) => h('option', { value: v, selected: d.mode === v }, t)));
    const win = h('input', { class: 'n-input n-num', value: d.window, placeholder: '例如 02:00-05:00', 'aria-label': '维护时间窗' });
    const interval = h('input', { class: 'n-input n-num', value: d.interval, inputmode: 'numeric', 'aria-label': '检查间隔' });
    const rows = {
      channel: prop('更新通道', channel, '设备跟随的版本通道，决定能收到哪些新版本。正式版用 main，测试分支写成 branch:分支名。', 'channel'),
      mode: prop('更新方式', mode, '发现新版本后的处理方式。空闲时自动安装会等任务结束、电机停用后再开始。', 'mode'),
      window: prop('维护时间窗', win, '自动安装只在这个时间段内进行，留空表示全天都可以。', 'window'),
      interval: prop('检查间隔', h('div', { class: 'n-input-group' }, interval, h('span', { class: 'n-suffix' }, '分钟')), '后台多久检查一次新版本，范围 15～1440 分钟。', 'interval'),
    };
    const sync = () => {
      const nd = { channel: channel.value.trim(), mode: mode.value, window: win.value.trim(), interval: interval.value.trim() };
      const changed = Object.keys(nd).filter(k => nd[k] !== base[k]);
      S.mgrDraft = changed.length ? nd : null;
      Object.entries(rows).forEach(([k, r]) => { r.dataset.modified = String(changed.includes(k)); });
      bar.hidden = !changed.length;
      if (changed.length) bar.replaceChildren(
        h('div', { class: 'n-savebar-text' }, '有 ' + changed.length + ' 项管理选项未保存', h('small', {}, '保存后从下一次检查新版本开始生效')),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => { S.mgrDraft = null; renderBody(); toast('已放弃改动', { tone: 'info' }); } }, '放弃改动'),
        h('button', { class: 'n-btn n-btn--sm n-btn--primary', type: 'button', onclick: async () => {
          try {
            S.settings = await request('settings', { method: 'PUT', body: { channel: nd.channel, auto_update: nd.mode, maintenance_window: nd.window, check_interval_minutes: Number(nd.interval) } });
            S.mgrDraft = null; renderBody(); toast('已保存管理选项');
          } catch (error) { toast('管理选项没有保存：' + error.message, { tone: 'bad', duration: 8000 }); }
        } }, '保存'));
    };
    for (const el of [channel, win, interval]) el.addEventListener('input', sync);
    mode.addEventListener('change', sync);
    sync();
    const password = S.status.password_enabled;
    const pw = password
      ? [h('span', { class: 'n-status' }, dotEl('ok'), '已设置'),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: () => passwordDialog(true) }, '修改密码'),
        h('button', { class: 'n-btn n-btn--sm n-btn--quiet', type: 'button', onclick: removePassword }, '移除')]
      : [h('span', { class: 'n-status' }, dotEl('off'), '未设置'),
        h('button', { class: 'n-btn n-btn--sm', type: 'button', html: icon('lock', 'n-icon--sm'), onclick: () => passwordDialog(false) }, '设置密码')];
    const auto = h('input', { type: 'checkbox', checked: Boolean(saved.auto_start) });
    auto.addEventListener('change', async () => {
      try {
        S.settings = await request('settings', { method: 'PUT', body: { auto_start: auto.checked } });
        toast(auto.checked ? '已开启开机自动启动' : '已关闭开机自动启动，设备开机后需要手动启动业务服务');
        renderBody();
      } catch (error) { auto.checked = !auto.checked; toast('开机自动启动没有保存：' + error.message, { tone: 'bad', duration: 8000 }); }
    });
    const modeText = (UPDATE_MODES.find(m => m[0] === base.mode) || [, base.mode])[1];
    const el = block('options', { icon: 'sliders-horizontal', title: '管理选项',
      summary: '更新通道 ' + base.channel + '，' + modeText + '，' + (saved.auto_start ? '开机自动启动' : '开机不自动启动') + '，' + (password ? '已设访问密码' : '没有访问密码') });
    el.body.append(rows.channel, rows.mode, rows.window, rows.interval,
      prop('访问密码', pw, password ? '修改配置、执行更新前需要先输入密码。' : '设置后，修改配置、执行更新前需要先输入密码。未设置时，能打开管理页面的人都可以操作。'),
      prop('开机自动启动', h('label', { class: 'n-switch', 'aria-label': '开机自动启动' }, auto, h('span', { class: 'n-switch-track' })), '设备开机后自动启动业务服务。切换后立即生效。'));
    return el;
  }
  async function passwordDialog(change) {
    const field = (label, input) => h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, label), input);
    const current = h('input', { class: 'n-input', type: 'password', autocomplete: 'current-password' });
    const next = h('input', { class: 'n-input', type: 'password', autocomplete: 'new-password' });
    const again = h('input', { class: 'n-input', type: 'password', autocomplete: 'new-password' });
    const fields = [...(change ? [field('当前密码', current)] : []), field('新密码（至少 8 个字符）', next), field('再次输入新密码', again)];
    setTimeout(() => fields[0].querySelector('input').focus(), 60);
    const ok = await confirmDialog({ title: change ? '修改访问密码' : '设置访问密码', confirmText: change ? '修改密码' : '设置密码',
      text: '设置后，修改配置、执行更新前需要先输入密码。', extra: h('div', { class: 's-dlg-fields' }, fields) });
    if (!ok) return;
    if (next.value !== again.value) { toast('两次输入的新密码不一致，密码没有改动', { tone: 'warn' }); return; }
    try {
      await request('password', { method: 'POST', body: { password: next.value, current_password: current.value } });
      toast(change ? '已修改访问密码' : '已设置访问密码');
      await refreshStatus();
    } catch (error) { toast('密码没有保存：' + error.message, { tone: 'bad', duration: 8000 }); }
  }
  async function removePassword() {
    const current = h('input', { class: 'n-input', type: 'password', autocomplete: 'current-password' });
    setTimeout(() => current.focus(), 60);
    const ok = await confirmDialog({ title: '移除访问密码', confirmText: '移除密码', danger: true,
      text: '移除后，能打开管理页面的人都可以修改配置和执行更新。请确认设备处在可信网络里。',
      extra: h('label', { class: 'n-field' }, h('span', { class: 'n-label' }, '当前密码'), current) });
    if (!ok) return;
    try {
      await request('password', { method: 'POST', body: { password: '', current_password: current.value } });
      toast('已移除访问密码');
      await refreshStatus();
    } catch (error) { toast('密码没有移除：' + error.message, { tone: 'bad', duration: 8000 }); }
  }

  /* ---------- 设备信息（默认收起，展开时读一次硬件） ---------- */
  async function loadHardware() {
    if (S.hardware) return;
    try { S.hardware = (await request('hardware', { timeout: 60000 })).hardware || {}; }
    catch (error) { S.hardware = { error: error.message }; }
    renderBody();
  }
  function deviceBlock() {
    const inst = installation(), hw = S.hardware, st = S.status;
    const robotLabel = S.modules?.robot_labels?.[inst.robot] || inst.robot || '未知';
    const os = hw && !hw.error ? [hw.os_id, hw.version_id].filter(v => v && v !== 'unknown').join(' ') || '未知' : '';
    const gpu = hw && !hw.error ? (hw.nvidia_gpu === true ? (hw.is_jetson ? 'NVIDIA（Jetson 集成）' : 'NVIDIA') + (hw.gpu_memory_bytes ? '，显存 ' + fmtBytes(hw.gpu_memory_bytes) : '') : hw.nvidia_gpu === false ? '未检测到' : '未知') : '';
    const items = [['机器人', robotLabel], ...(opts.device ? opts.device() : []),
      ...(hw && !hw.error ? [['系统', os], ['架构', hw.architecture || '未知'], ['处理器', hw.cpu_count != null ? hw.cpu_count + ' 线程' : '未知'],
        ['内存', fmtBytes(hw.memory_total_bytes)], ['安装盘可用', fmtBytes(hw.free_disk_bytes)], ['显卡', gpu],
        ['摄像头节点', hw.camera_detected === true ? '已检测到' : hw.camera_detected === false ? '未检测到' : '未知']] : []),
      ['安装位置', h('span', { class: 's-ver' }, inst.install_dir || '未知')],
      ...(inst.manager_dir && inst.manager_dir !== inst.install_dir ? [['管理程序位置', h('span', { class: 's-ver' }, inst.manager_dir)]] : []),
      ['安装来源', { git: '源码构建' }[inst.source] || inst.source || '未知'],
      ['管理程序', st.manager_development ? '开发版本' : st.manager_version || '未知'],
      ['开机自动启动', S.settings?.auto_start ? '已开启' : '已关闭']];
    const el = block('device', { icon: 'cpu', title: '设备信息',
      summary: [robotLabel, os, hw?.cpu_count ? hw.cpu_count + ' 线程' : '', hw?.memory_total_bytes ? fmtBytes(hw.memory_total_bytes) + ' 内存' : ''].filter(Boolean).join('，') });
    el.body.append(h('div', { class: 's-kv4-box' }, hw?.error ? h('p', { class: 'n-block-pad n-meta' }, '设备硬件信息读取失败：' + hw.error) : null,
      !hw ? h('p', { class: 'n-block-pad n-meta' }, '正在读取设备硬件信息…') : null,
      h('dl', { class: 's-kv4' }, items.map(([k, v]) => kvPair(k, v)))));
    return el;
  }

  /* ---------- 抽屉：记录详情、全部记录、调整模块 ---------- */
  function openDrawer(d) { S.drawer = d; renderDrawer(); if (d.type === 'record') void openRecord(d.id); }
  function closeDrawer() { S.drawer = null; renderDrawer(); }
  async function openRecord(id) {
    if (!S.jobs.some(j => j.id === id && j.log)) {
      try { const job = await request('jobs/' + encodeURIComponent(id)); S.jobs = S.jobs.map(j => (j.id === id ? job : j)); } catch (error) { /* 用列表里的记录 */ }
    }
    output.id = null;
    void pullOutput(id);
    renderDrawer();
  }
  // 详细日志：结束的任务读末尾 256 KB；进行中的任务按字节偏移增量读（沿用原设备管理页）。
  async function pullOutput(id) {
    if (output.loading) return;
    output.loading = true;
    try {
      if (output.id !== id) { output.id = id; output.offset = 0; output.text = ''; }
      for (let round = 0; round < 8; round += 1) {
        const chunk = await request('jobs/' + encodeURIComponent(id) + '/output?' + (output.offset ? 'offset=' + output.offset : 'tail=262144'));
        if (output.id !== id) return;
        output.text = (output.text + (chunk.text || '')).slice(-400000);
        output.offset = chunk.offset;
        if (chunk.offset >= chunk.size) break;
      }
    } catch (error) { output.text = output.text || '读取详细日志失败：' + error.message; }
    finally { output.loading = false; }
    if (S.drawer?.type === 'record' && S.drawer.id === id) renderDrawer();
  }
  function renderDrawer() {
    const d = S.drawer;
    if (!d) { drawerEl?.remove(); scrimEl?.remove(); drawerEl = scrimEl = null; return; }
    const kind = d.type + ':' + (d.id || '');
    const keep = drawerEl && drawerEl.dataset.kind === kind ? drawerEl.querySelector('.n-drawer-body')?.scrollTop || 0 : 0;
    const logBox = drawerEl?.querySelector('.s-log');
    const stick = !logBox || logBox.scrollHeight - logBox.scrollTop - logBox.clientHeight < 40;
    if (!drawerEl) {
      scrimEl = h('div', { class: 's-scrim', onclick: closeDrawer });
      drawerEl = h('aside', { class: 'n-drawer', role: 'dialog', 'aria-modal': 'false' });
      shell.append(scrimEl, drawerEl);
    }
    drawerEl.dataset.kind = kind;
    drawerEl.classList.toggle('s-drawer--wide', d.type !== 'modules');
    const p = d.type === 'modules' ? modulesDrawer() : d.type === 'records' ? recordsDrawer() : recordDrawer(d);
    drawerEl.setAttribute('aria-label', p.label);
    drawerEl.replaceChildren(...[p.head, p.body, p.foot].filter(Boolean));
    p.body.scrollTop = keep;
    const log = drawerEl.querySelector('.s-log');
    if (log && stick) log.scrollTop = log.scrollHeight;
  }
  const closeBtn = () => h('button', { class: 'n-btn n-btn--quiet n-btn--icon', type: 'button', 'aria-label': '关闭', title: '关闭（Esc）', html: icon('x'), onclick: closeDrawer });
  function recordDrawer(d) {
    const job = (S.job && S.job.id === d.id ? S.job : null) || S.jobs.find(j => j.id === d.id);
    if (!job) return { label: '记录', head: h('div', { class: 'n-drawer-head' }, h('h2', {}, '记录'), closeBtn()), body: h('div', { class: 'n-drawer-body' }, h('p', { class: 'n-meta' }, '正在读取记录…')) };
    const failed = ['failed', 'interrupted'].includes(job.status);
    const advice = failed ? adviceOf(job) : '';
    const head = h('div', { class: 'n-drawer-head' },
      d.back ? h('button', { class: 'n-btn n-btn--quiet n-btn--icon', type: 'button', 'aria-label': '返回全部记录', html: icon('chevron-left'), onclick: () => openDrawer({ type: 'records' }) }) : null,
      markFor(job), h('h2', {}, fmtTitle(jobTitle(job))), closeBtn());
    const body = h('div', { class: 'n-drawer-body s-drawer' },
      h('dl', { class: 's-dfacts' }, kvPair('开始时间', fmtWhen(job.created_at, true)), kvPair('用时', fmtDur(jobSeconds(job))), kvPair('结果', resEl(job))),
      failed ? h('div', { class: 'n-hl n-hl--bad s-dfail' }, h('dl', {}, h('dt', {}, '原因'), h('dd', {}, job.error?.message || job.message),
        advice ? h('dt', {}, '建议') : null, advice ? h('dd', {}, advice) : null)) : null,
      job.error?.details?.diagnostic ? h('section', { class: 'n-sec' }, secHead('组件日志末尾'), h('pre', { class: 'n-pre n-terminal s-log' }, job.error.details.diagnostic)) : null,
      job.status === 'cancelled' ? h('p', { class: 'n-meta s-dfail' }, job.message) : null,
      h('section', { class: 'n-sec' }, secHead('步骤'), job.log ? stepList(job) : h('p', { class: 'n-meta' }, '正在读取步骤…')),
      h('section', { class: 'n-sec' }, secHead('详细日志', '安装、构建和启动命令的输出'),
        h('pre', { class: 'n-pre n-terminal s-log', tabindex: '0', 'aria-label': '详细日志' }, output.id === job.id ? output.text || (output.loading ? '正在读取…' : '这个任务没有命令输出。') : '正在读取…')),
      h('div', { class: 's-taskid' }, '任务编号', h('code', {}, job.id),
        h('button', { class: 'n-btn n-btn--quiet n-btn--icon n-btn--sm', type: 'button', 'aria-label': '复制任务编号', html: icon('copy', 'n-icon--sm'),
          onclick: async () => toast(await copyText(job.id) ? '已复制任务编号' : '复制失败，请手动选中', { tone: 'info' }) })));
    const foot = jobRunning(job) ? h('div', { class: 'n-drawer-foot' }, h('button', { class: 'n-btn', type: 'button', onclick: () => cancelJob(job) }, '取消')) : null;
    return { label: jobTitle(job), head, body, foot };
  }
  function recordsDrawer() {
    const match = (j, f) => f === 'all' || (f === 'failed' ? ['failed', 'interrupted'].includes(j.status) : jobRunning(j));
    const list = S.jobs.filter(j => match(j, S.recFilter));
    const seg = h('div', { class: 'n-seg s-drawer-filter', role: 'group', 'aria-label': '筛选记录' },
      [['all', '全部'], ['failed', '失败'], ['running', '进行中']].map(([id, label]) =>
        h('button', { type: 'button', 'aria-pressed': String(S.recFilter === id), onclick: () => { S.recFilter = id; renderDrawer(); } }, label,
          h('span', { class: 'n-count' }, S.jobs.filter(j => match(j, id)).length))));
    const body = h('div', { class: 'n-drawer-body' }, seg,
      list.length ? h('div', { class: 's-drawer-list' }, list.map(j => h('button', { class: 's-rec', type: 'button', onclick: () => openDrawer({ type: 'record', id: j.id, back: true }) },
        markFor(j), h('span', { class: 's-rec-title' }, fmtTitle(jobTitle(j))), h('span', { class: 's-rec-time' }, fmtWhen(j.created_at) + '，用时 ' + fmtDur(jobSeconds(j))),
        resEl(j), h('span', { class: 's-chev', html: icon('chevron-right', 'n-icon--sm') }))))
        : h('div', { class: 'n-empty', html: icon(S.recFilter === 'failed' ? 'check' : 'clock') },
          h('div', { class: 'n-empty-title' }, S.recFilter === 'failed' ? '没有失败的记录' : S.recFilter === 'running' ? '没有进行中的任务' : '还没有记录'),
          h('div', {}, S.recFilter === 'running' ? '开始更新、调整模块或启停业务服务后，进度会显示在这里。' : '安装、更新和启停都已正常完成。')));
    return { label: '安装与更新记录', head: h('div', { class: 'n-drawer-head' }, h('h2', {}, '安装与更新记录'), closeBtn()), body };
  }
  // 调整模块：开关表示是否安装；关掉被依赖的模块会连带移除，打开的模块会补上依赖。
  function modulesDrawer() {
    const installed = installedSet();
    const want = S.want || installed;
    const busy = S.job && jobRunning(S.job);
    const body = h('div', { class: 'n-drawer-body' },
      h('p', { class: 'n-meta', style: 'margin-bottom:8px' }, busy ? '正在执行“' + jobTitle(S.job) + '”，完成后才能调整模块。' : '开关表示模块是否安装。改动先生成安装计划，确认后在后台执行。'),
      h('div', {}, moduleList().map(m => modRow(m, want, installed, busy))));
    let foot = null;
    if (!busy) {
      const diff = modsDiff(installed);
      const parts = [];
      if (diff.add.length) parts.push('安装 ' + diff.add.length + ' 个');
      if (diff.rem.length) parts.push('移除 ' + diff.rem.length + ' 个');
      foot = h('div', { class: 'n-drawer-foot' },
        h('span', { class: 'n-meta s-drawer-foot-note' }, parts.length ? '将' + parts.join('、') + '模块' : '没有改动'),
        parts.length ? h('button', { class: 'n-btn', type: 'button', onclick: () => { S.want = null; S.direct.clear(); renderDrawer(); } }, '放弃改动') : null,
        h('button', { class: 'n-btn n-btn--primary', type: 'button', disabled: !parts.length || Boolean(S.busy), onclick: planModules }, S.busy === 'modules' ? '正在生成计划…' : '生成安装计划'));
    }
    return { label: '调整功能模块', head: h('div', { class: 'n-drawer-head' }, h('h2', {}, '调整功能模块'), closeBtn()), body, foot };
  }
  function modRow(m, want, installed, busy) {
    const on = want.has(m.key), inst = installed.has(m.key);
    const change = on && !inst ? 'add' : !on && inst ? 'remove' : null;
    const input = h('input', { type: 'checkbox', checked: on, disabled: m.key === 'core' || busy, 'aria-label': m.title });
    input.addEventListener('change', () => toggleMod(m.key, input.checked, installed));
    const badge = m.key === 'core' ? h('span', { class: 'n-badge' }, '必选') : change === 'add' ? h('span', { class: 'n-badge n-badge--accent' }, '将安装')
      : change === 'remove' ? h('span', { class: 'n-badge n-badge--warn' }, '将移除') : !inst ? h('span', { class: 'n-badge n-badge--outline' }, '未安装') : null;
    const meta = [m.key === 'core' ? '始终保留，其他模块都依赖它' : m.requires.length ? '依赖 ' + titles(m.requires) : '',
      m.conflicts.length ? '与 ' + titles(m.conflicts) + ' 冲突' : ''].filter(Boolean).join('，');
    const row = h('div', { class: 's-mod', dataset: change ? { change } : null },
      h('div', { class: 's-mod-name' }, m.title, badge),
      h('div', { class: 's-mod-desc' }, m.description, meta ? h('br') : null, meta),
      h('label', { class: 's-mod-sw n-switch' + (m.key === 'core' ? ' n-tip n-tip--left' : ''), 'data-tip': m.key === 'core' ? '基础网关始终保留，不能移除' : null }, input, h('span', { class: 'n-switch-track' })));
    const d = S.direct.get(m.key);
    if (d === 'off' && !want.has(m.key)) {
      const gone = dependantsOf(m.key).filter(k => installed.has(k) && !want.has(k));
      if (gone.length) row.append(h('div', { class: 'n-hl n-hl--warn s-mod-note' }, '关闭后会一并移除依赖它的模块：' + titles(gone) + '。'));
    }
    if (d === 'on' && want.has(m.key)) {
      const add = requiresOf(m.key).filter(k => !installed.has(k) && want.has(k));
      if (add.length) row.append(h('div', { class: 'n-hl s-mod-note' }, '会同时安装它依赖的模块：' + titles(add) + '。'));
    }
    return row;
  }
  function toggleMod(key, on, installed) {
    const want = new Set(S.want || installed);
    if (on) { want.add(key); requiresOf(key).forEach(k => want.add(k)); } else { want.delete(key); dependantsOf(key).forEach(k => want.delete(k)); }
    S.direct.set(key, on ? 'on' : 'off');
    S.want = want.size === installed.size && [...want].every(k => installed.has(k)) ? null : want;
    if (!S.want) S.direct.clear();
    renderDrawer();
  }
  function modsDiff(installed) {
    if (!S.want) return { add: [], rem: [] };
    return { add: moduleList().filter(m => S.want.has(m.key) && !installed.has(m.key)).map(m => m.key),
      rem: moduleList().filter(m => !S.want.has(m.key) && installed.has(m.key)).map(m => m.key) };
  }
  function planModules() {
    const installed = installedSet();
    const { add, rem } = modsDiff(installed);
    const target = [...S.want];
    const parts = [];
    if (add.length) parts.push('安装 ' + titles(add));
    if (rem.length) parts.push('移除 ' + titles(rem));
    void runPlan({ action: 'modules', modules: target, robot: installation().robot, stop_running: true }, async plan => {
      const download = plan.download_bytes ? fmtBytes(plan.download_bytes) : '由缓存和网络决定';
      const extra = h('div', { class: 's-dlg-extra' },
        h('dl', { class: 's-facts' }, kvPair('安装', add.length ? add.length + ' 个' : '无'), kvPair('移除', rem.length ? rem.length + ' 个' : '无'), kvPair('预计下载', download)),
        h('ol', { class: 's-plan' }, (plan.steps || []).map(s => h('li', {}, h('span', { class: 's-plan-title' }, s.title), s.description ? h('span', { class: 's-plan-sub' }, s.description) : null))),
        plan.warnings?.length ? h('div', { class: 'n-hl n-hl--warn s-dlg-note' }, plan.warnings.map(w => h('div', {}, w))) : null,
        h('p', { class: 's-dlg-note' }, plan.requires_stop ? '执行到“切换版本”时业务服务会停止约 1 分钟，正在执行的导航、跟随等任务会被中断。' : '业务服务没有运行，切换后' + (S.settings?.auto_start ? '会自动启动。' : '需要手动启动。')));
      const ok = await confirmDialog({ title: '安装计划：' + parts.join('，'), text: '确认后在后台按以下步骤执行，进度显示在系统页顶部。', extra, confirmText: '开始执行' });
      if (ok) { S.moduleTarget = new Set(target); S.want = null; S.direct.clear(); S.drawer = null; }
      return ok;
    });
  }
  function onKey(e) { if (e.key === 'Escape' && S.drawer && !document.querySelector('dialog[open]')) closeDrawer(); }
  document.addEventListener('keydown', onKey);

  renderAll();
  return {
    show() {
      S.visible = true;
      if (!S.status) void loadAll(); else { void refreshStatus(); void refreshJobs().then(renderAll); }
      startTimers();
    },
    hide() { S.visible = false; stopTimers(); },
    refresh: loadAll,
    destroy() { stopTimers(); document.removeEventListener('keydown', onKey); document.removeEventListener('visibilitychange', onVisibility); },
  };
}

/* ==========================================================================
   首次安装向导：硬件检查 → 功能模块 → 安装计划 → 执行安装
   opts.request：管理服务请求函数；opts.onDone()：安装完成后点“进入管理界面”；opts.consoleUrl：控制台地址
   ========================================================================== */
export function mountWizard(root, opts = {}) {
  const request = opts.request || managerRequest;
  const W = { step: 0, status: null, hardware: null, hwError: '', modules: null, picks: new Set(), robot: '', note: '', localBuild: false,
    plan: null, planError: '', planning: false, job: null, location: '', preview: null, previewError: '', savingLocation: false, checking: false };
  const output = { id: null, offset: 0, text: '' };
  const toolbar = h('div', { class: 'n-toolbar' });
  const body = h('div', { class: 'n-page-body' });
  root.replaceChildren(h('div', { class: 's-root s-wizard' }, h('div', { class: 'n-page--full' }, toolbar, body)));
  let timer = null;

  const moduleList = () => W.modules?.modules || [];
  const titles = keys => moduleList().filter(m => keys.includes(m.key)).map(m => m.title).join('、');
  function requiresOf(key) {
    const out = new Set();
    const walk = k => (moduleList().find(m => m.key === k)?.requires || []).forEach(r => { if (!out.has(r)) { out.add(r); walk(r); } });
    walk(key); return [...out];
  }
  function dependantsOf(key) {
    const out = new Set();
    const walk = k => moduleList().forEach(m => { if (m.requires.includes(k) && !out.has(m.key)) { out.add(m.key); walk(m.key); } });
    walk(key); return [...out];
  }
  const inst = () => W.status?.installation || {};
  const locationDirty = () => inst().location_editable && W.location.trim() !== (inst().parent_dir || '');

  async function start() {
    try {
      W.status = await request('status');
      W.location = inst().parent_dir || '';
      const job = W.status.active_job;
      // 重新打开页面或从命令行开始安装时，直接接回进行中的安装任务。
      if (job && job.action === 'install') { W.job = job; W.step = 3; render(); track(); return; }
    } catch (error) { W.hwError = error.message; }
    render();
    await detect();
  }
  async function detect() {
    W.checking = true; W.hwError = ''; render();
    try {
      const [hardware, modules] = await Promise.all([request('hardware', { timeout: 60000 }), request('modules')]);
      W.hardware = hardware; W.modules = modules;
      W.picks = new Set(hardware.recommendation?.modules || modules.selected || ['core']);
      W.robot = hardware.recommendation?.robot || modules.robot || '';
    } catch (error) { W.hwError = error.message; }
    W.checking = false;
    render();
  }
  function go(step) { W.step = step; body.scrollTop = 0; render(); }
  const primary = (label, run, ico, disabled) => h('button', { class: 'n-btn n-btn--primary', type: 'button', disabled: Boolean(disabled), html: ico ? icon(ico, 'n-icon--sm') : '', onclick: run }, label);
  const back = step => h('button', { class: 'n-btn', type: 'button', onclick: () => go(step) }, '上一步');
  function render() {
    const keep = body.scrollTop;
    // 重绘时保留正在输入的框（安装位置预览返回后会重绘）。
    const active = body.contains(document.activeElement) ? document.activeElement : null;
    const focus = active && active.getAttribute('aria-label') ? { label: active.getAttribute('aria-label'), at: active.selectionStart } : null;
    const done = W.job && W.job.status === 'succeeded';
    const stepper = h('ol', { class: 's-stepper', 'aria-label': '安装步骤' }, ['硬件检查', '功能模块', '安装计划', '执行安装'].map((t, k) => {
      const st = k < W.step || (done && k === 3) ? 'done' : k === W.step ? 'active' : 'todo';
      return h('li', { 'data-state': st, 'aria-current': k === W.step ? 'step' : null },
        h('span', { class: 's-step-num', html: st === 'done' ? icon('check') : '' }, st === 'done' ? null : String(k + 1)), h('span', { class: 's-step-label' }, t));
    }));
    const view = [stepHardware, stepModules, stepPlan, stepInstall][W.step]();
    toolbar.replaceChildren(h('h1', {}, '首次安装'), h('span', { class: 'n-toolbar-sep' }), stepper, h('span', { class: 'n-toolbar-spacer' }), h('div', { class: 's-actions' }, view.actions));
    body.replaceChildren(...view.content.filter(Boolean));
    body.scrollTop = keep;
    if (focus) {
      const el = body.querySelector('[aria-label="' + CSS.escape(focus.label) + '"]');
      if (el) { el.focus(); if (focus.at != null && el.setSelectionRange) el.setSelectionRange(focus.at, focus.at); }
    }
  }

  /* ---------- 第一步：硬件检查与安装位置 ---------- */
  function stepHardware() {
    const hw = W.hardware?.hardware || {};
    const rec = W.hardware?.recommendation || {};
    const gpu = hw.nvidia_gpu === true ? (hw.is_jetson ? 'NVIDIA（Jetson 集成）' : 'NVIDIA') + (hw.gpu_memory_bytes ? '，显存 ' + fmtBytes(hw.gpu_memory_bytes) : '') : hw.nvidia_gpu === false ? '未检测到 NVIDIA 显卡' : '未知';
    const rows = W.hardware ? [
      ['ok', '操作系统', [hw.os_id, hw.version_id].filter(v => v && v !== 'unknown').join(' ') + '，' + (hw.architecture || '未知架构'), ''],
      ['ok', '处理器', hw.cpu_count != null ? hw.cpu_count + ' 线程' : '未知', hw.build_jobs ? '本机编译并行 ' + hw.build_jobs : ''],
      ['ok', '内存', fmtBytes(hw.memory_total_bytes), hw.memory_available_bytes ? '当前可用 ' + fmtBytes(hw.memory_available_bytes) : ''],
      ['ok', '磁盘空间', '安装位置所在磁盘可用 ' + fmtBytes(hw.free_disk_bytes), ''],
      [hw.nvidia_gpu ? 'ok' : 'warn', '显卡', gpu, hw.nvidia_gpu ? '' : '本地视觉感知和语义记忆模型会很慢或不可用'],
      [hw.camera_detected ? 'ok' : 'warn', '摄像头', hw.camera_detected ? '已检测到摄像头节点' : '未检测到摄像头节点', ''],
    ] : [];
    const issues = W.hardware?.issues || [];
    const supported = W.hardware?.supported !== false;
    const checks = W.checking ? [h('div', { class: 's-check' }, h('span', { class: 's-mark s-mark--busy' }, spin()), h('span', { class: 's-check-name' }, '正在检测'), h('span', { class: 's-check-val' }, '读取系统、处理器、内存、磁盘和显卡'))]
      : rows.map(([st, name, val, note]) => h('div', { class: 's-check' }, h('span', { class: 's-mark s-mark--' + st, html: icon(st === 'ok' ? 'check' : 'triangle-alert') }),
        h('span', { class: 's-check-name' }, name), h('span', { class: 's-check-val' }, val), note ? h('span', { class: 's-check-note' }, note) : null));
    const loc = h('input', { class: 'n-input n-input--code', value: W.location, spellcheck: 'false', 'aria-label': '安装位置', disabled: !inst().location_editable || W.savingLocation });
    const preview = h('p', { class: 's-loc-preview' });
    const syncPreview = () => preview.replaceChildren(...(W.previewError ? [h('span', { class: 's-bad' }, W.previewError)]
      : ['实际安装到 ', h('code', {}, W.preview || inst().install_dir || '')]));
    let previewTimer = null;
    loc.addEventListener('input', () => {
      W.location = loc.value;
      clearTimeout(previewTimer);
      previewTimer = setTimeout(async () => {
        // 手输路径由设备解析（符号链接、~ 都按设备上的实际目录算）。
        try { W.preview = (await request('install-location?parent_dir=' + encodeURIComponent(loc.value.trim()))).install_dir; W.previewError = ''; }
        catch (error) { W.preview = null; W.previewError = '无法预览安装位置：' + error.message; }
        syncPreview(); render();
      }, 300);
    });
    syncPreview();
    const dirty = locationDirty();
    const ready = W.hardware && supported && !W.checking && !dirty;
    return {
      actions: [primary('下一步', () => go(1), '', !ready)],
      content: [
        W.hwError ? h('div', { class: 'n-hl n-hl--bad', style: 'margin-bottom:16px' }, '读取设备信息失败：' + W.hwError) : null,
        h('section', { class: 'n-sec' },
          secHead('检测结果', W.checking ? '正在检测' : !supported ? '这台设备不在支持范围内' : issues.length ? issues.length + ' 项需要处理' : '可以安装',
            h('button', { class: 'n-btn n-btn--sm n-btn--quiet', type: 'button', disabled: W.checking, html: icon('refresh-cw', 'n-icon--sm'), onclick: detect }, '重新检测')),
          h('div', { class: 'n-box s-check-box' }, h('div', { class: 's-checks' }, checks)),
          issues.length ? h('div', { class: 'n-hl n-hl--bad' }, issues.map(t => h('div', {}, t))) : null,
          (rec.reasons || []).length ? h('div', { class: 'n-meta' }, rec.reasons.map(t => h('div', {}, t))) : null,
          (hw.evidence || []).length ? h('details', { class: 's-diag' }, h('summary', { html: icon('chevron-right', 'n-icon--sm') + '检测依据（' + hw.evidence.length + ' 条）' }),
            h('div', { class: 'n-meta' }, hw.evidence.map(t => h('div', {}, t)))) : null),
        h('section', { class: 'n-sec' },
          secHead('安装位置', '程序、构建缓存、配置和数据都保存在这里'),
          h('div', { class: 'n-box n-box-pad' },
            inst().location_editable ? [
              h('div', { class: 's-loc' }, loc, h('button', { class: 'n-btn', type: 'button', html: icon('folder', 'n-icon--sm'), disabled: W.savingLocation, onclick: () => browseDir(loc) }, '浏览设备目录'),
                dirty ? h('button', { class: 'n-btn', type: 'button', onclick: () => { W.location = inst().parent_dir || ''; W.preview = null; W.previewError = ''; render(); } }, '放弃修改') : null,
                dirty ? h('button', { class: 'n-btn n-btn--primary', type: 'button', disabled: W.savingLocation || !W.preview, onclick: saveLocation }, W.savingLocation ? '正在保存…' : '保存安装位置') : null),
              h('p', { class: 's-loc-help' }, '会在所选文件夹中创建 Nuwax-embodied-harness 子目录；选择同名目录时直接使用。' + (dirty ? '保存后才能进入下一步。' : '')),
              preview]
              : [h('p', {}, h('code', { class: 's-ver' }, inst().install_dir || '')), h('p', { class: 's-loc-help' }, inst().location_reason || '当前安装位置暂时不能修改。')],
            inst().manager_dir && inst().manager_dir !== inst().install_dir ? h('p', { class: 's-loc-help' }, '管理程序保存在 ', h('code', {}, inst().manager_dir)) : null)),
      ],
    };
  }
  async function saveLocation() {
    W.savingLocation = true; render();
    try {
      const saved = await request('install-location', { method: 'PUT', body: { parent_dir: W.location.trim() } });
      W.status = await request('status');
      W.location = inst().parent_dir || saved.parent_dir || '';
      W.preview = null;
      toast('安装位置已保存：' + saved.install_dir);
      await detect();   // 可用磁盘空间取决于新位置
    } catch (error) { toast('安装位置没有保存：' + error.message, { tone: 'bad', duration: 8000 }); }
    finally { W.savingLocation = false; render(); }
  }
  async function browseDir(input) {
    const path = h('input', { class: 'n-input n-input--code', spellcheck: 'false', 'aria-label': '设备目录路径' });
    const list = h('div', { class: 's-dir-list' });
    const preview = h('div', { class: 's-dir-preview' });
    const up = h('button', { class: 'n-btn', type: 'button', html: icon('chevron-up', 'n-icon--sm') }, '上一级');
    let cur = null;
    async function open(p) {
      list.replaceChildren(h('div', { class: 's-dir-empty' }, '正在读取设备目录…'));
      try {
        cur = await request('install-directories' + (p ? '?path=' + encodeURIComponent(p) : ''));
        path.value = cur.path; up.disabled = !cur.parent;
        list.replaceChildren(...(cur.directories.length
          ? cur.directories.map(d => h('button', { class: 'n-row', type: 'button', html: icon('folder', 'n-icon--sm'), onclick: () => open(d.path) }, h('span', { class: 'n-row-main' }, h('span', { class: 'n-row-title' }, d.name))))
          : [h('div', { class: 's-dir-empty' }, '这个文件夹里没有子文件夹，可以直接选择它。')]));
        preview.replaceChildren(...(cur.writable === false ? [h('span', { class: 's-bad' }, '当前用户不能写入这个文件夹，请选择可写的文件夹。')] : ['安装目标 ', h('code', {}, cur.install_dir)]));
      } catch (error) { list.replaceChildren(h('div', { class: 's-dir-empty' }, '读取失败：' + error.message)); }
    }
    up.addEventListener('click', () => cur?.parent && open(cur.parent));
    path.addEventListener('keydown', e => { if (e.key === 'Enter') { e.preventDefault(); if (path.value.trim()) open(path.value.trim()); } });
    void open(input.value.trim());
    const ok = await confirmDialog({ title: '选择存放安装的文件夹', confirmText: '选择此文件夹',
      text: '浏览运行 NEH 的设备上的文件夹。文件夹不需要为空，会在其中创建 Nuwax-embodied-harness 子目录。',
      extra: h('div', { class: 's-dir' }, h('div', { class: 's-dir-bar' }, up, path), list, preview) });
    if (!ok || !cur) return;
    if (cur.writable === false) { toast('当前用户不能写入这个文件夹，安装位置没有改动', { tone: 'warn' }); return; }
    W.location = cur.selection_path || cur.path;
    W.preview = cur.install_dir; W.previewError = '';
    render();
  }

  /* ---------- 第二步：功能模块与机器人类型 ---------- */
  function pickToggle(key, on) {
    if (on) {
      W.picks.add(key);
      const add = requiresOf(key).filter(k => !W.picks.has(k));
      add.forEach(k => W.picks.add(k));
      W.note = add.length ? '已同时选上它依赖的模块：' + titles(add) + '。' : '';
    } else {
      W.picks.delete(key);
      const gone = dependantsOf(key).filter(k => W.picks.has(k));
      gone.forEach(k => W.picks.delete(k));
      W.note = gone.length ? '已同时取消依赖它的模块：' + titles(gone) + '。' : '';
    }
    W.plan = null;
    render();
  }
  function stepModules() {
    const rec = new Set(W.hardware?.recommendation?.modules || []);
    const rows = moduleList().map(m => {
      const input = h('input', { type: 'checkbox', checked: W.picks.has(m.key), disabled: m.key === 'core' });
      input.addEventListener('change', () => pickToggle(m.key, input.checked));
      const tag = m.key === 'core' ? h('span', { class: 'n-badge' }, '必选') : rec.has(m.key) ? h('span', { class: 'n-badge n-badge--accent' }, '推荐') : null;
      const desc = [m.description, m.requires.length ? '依赖 ' + titles(m.requires) + '。' : '', m.conflicts.length ? '与 ' + titles(m.conflicts) + ' 冲突。' : ''].join('');
      return h('label', { class: 's-pick', dataset: { locked: String(m.key === 'core') } }, input,
        h('span', { class: 's-pick-name' }, m.title, tag), h('span', { class: 's-pick-desc' }, desc));
    });
    const robots = W.modules?.robots || [];
    const robot = h('select', { class: 'n-select', 'aria-label': '机器人类型' }, robots.map(r => h('option', { value: r, selected: r === W.robot }, W.modules.robot_labels?.[r] || r)));
    robot.addEventListener('change', () => { W.robot = robot.value; W.plan = null; });
    const warnings = W.hardware?.recommendation?.warnings || [];
    return {
      actions: [back(0), primary('下一步', () => { W.note = ''; go(2); void makePlan(); })],
      content: [
        h('section', { class: 'n-sec' }, secHead('机器人类型', '按接入的机器人选择，决定驱动与默认配置'),
          h('div', { class: 'n-box n-box-pad' }, h('div', { class: 's-loc' }, robot))),
        h('section', { class: 'n-sec' },
          secHead('功能模块', '已按检测到的硬件预选推荐的模块；选上的模块会自动带上它依赖的模块'),
          warnings.length ? h('div', { class: 'n-hl n-hl--warn' }, warnings.map(t => h('div', {}, t))) : null,
          h('div', { class: 'n-box s-check-box' }, h('div', { class: 's-picks' }, rows),
            h('div', { class: 's-sum' }, h('span', {}, '已选 ', h('b', { class: 'n-t2' }, W.picks.size), ' 个模块'),
              W.note ? h('span', { class: 'n-meta', role: 'status' }, W.note) : null))),
      ],
    };
  }

  /* ---------- 第三步：安装计划 ---------- */
  async function makePlan() {
    W.planning = true; W.plan = null; W.planError = ''; render();
    try {
      W.plan = await request('plan', { method: 'POST', timeout: 120000,
        body: { action: 'install', modules: moduleList().map(m => m.key).filter(k => W.picks.has(k)), robot: W.robot || undefined, local_build: W.localBuild } });
    } catch (error) { W.planError = error.message; }
    W.planning = false;
    render();
  }
  function stepPlan() {
    const plan = W.plan;
    const local = h('input', { type: 'checkbox', checked: W.localBuild });
    local.addEventListener('change', () => { W.localBuild = local.checked; void makePlan(); });
    const breakdown = plan?.download_breakdown || {};
    const size = v => (v === null || v === undefined ? '暂无法预报' : v === 0 ? '无需下载' : fmtBytes(v));
    const blocked = !plan || plan.blockers?.length;
    return {
      actions: [back(1), primary('开始安装', submit, 'download', blocked || W.planning)],
      content: [
        W.planning ? h('div', { class: 'n-empty' }, spin(), h('div', {}, '正在生成安装计划…')) : null,
        W.planError ? h('div', { class: 'n-hl n-hl--bad' }, '生成计划失败：' + W.planError, h('div', {}, h('button', { class: 'n-btn n-btn--sm', type: 'button', style: 'margin-top:8px', onclick: makePlan }, '重新生成'))) : null,
        plan ? h('div', { class: 'n-cols-2' },
          h('section', { class: 'n-sec' }, secHead('步骤', '共 ' + (plan.steps || []).length + ' 步，按顺序执行'),
            h('div', { class: 'n-box n-box-pad' }, h('ol', { class: 's-plan' }, (plan.steps || []).map(s => h('li', {}, h('span', { class: 's-plan-title' }, s.title),
              s.description ? h('span', { class: 's-plan-sub' }, s.description) : null)))),
            plan.blockers?.length ? h('div', { class: 'n-hl n-hl--bad' }, h('b', {}, '需要先处理：'), plan.blockers.map(t => h('div', {}, t))) : null,
            plan.warnings?.length ? h('div', { class: 'n-hl n-hl--warn' }, plan.warnings.map(t => h('div', {}, t))) : null),
          h('div', { class: 's-col' },
            h('section', { class: 'n-sec' }, secHead('资源'),
              h('div', { class: 'n-box' }, h('dl', { class: 's-wz-facts' },
                kvPair('目标版本', h('span', { class: 'n-t3 s-ver' }, plan.version || '通道当前版本')),
                kvPair('功能模块', h('span', { class: 'n-t2' }, (plan.modules || []).length + ' 个')),
                kvPair('源码', size(breakdown.application)), kvPair('依赖与模型', breakdown.dependencies == null && breakdown.models == null ? '按缓存与网络决定' : size((breakdown.dependencies || 0) + (breakdown.models || 0)))))),
            h('section', { class: 'n-sec' }, secHead('选项'),
              h('div', { class: 'n-box n-box-pad' }, h('label', { class: 's-option' }, local, h('span', { class: 's-option-title' }, '云端未就绪时立即本机编译'),
                h('span', { class: 's-option-help' }, '云端预编译还没完成时不等待，直接在本机编译。本机编译耗时较长，期间处理器占用较高。')))),
            h('section', { class: 'n-sec' }, secHead('安装位置', '在第一步修改'),
              h('div', { class: 'n-box n-box-pad' }, h('span', { class: 's-ver' }, plan.install_dir || inst().install_dir))))) : null,
      ],
    };
  }
  async function submit() {
    if (!W.plan || W.plan.blockers?.length) return;
    try {
      W.job = await request('jobs', { method: 'POST', body: { plan_id: W.plan.id } });
      go(3);
      track();
    } catch (error) { toast('安装没有开始：' + error.message, { tone: 'bad', duration: 8000 }); }
  }

  /* ---------- 第四步：执行安装（每 2 秒读进度与命令输出） ---------- */
  function track() {
    clearInterval(timer);
    timer = setInterval(async () => {
      if (document.hidden || !W.job) return;
      try {
        W.job = await request('jobs/' + encodeURIComponent(W.job.id));
        await pullOutput();
        if (!jobRunning(W.job)) clearInterval(timer);
        render();
      } catch (error) { /* 网络抖动不终止，下一次继续读 */ }
    }, 2000);
  }
  async function pullOutput() {
    const id = W.job.id;
    if (output.id !== id) { output.id = id; output.offset = 0; output.text = ''; }
    for (let round = 0; round < 8; round += 1) {
      const chunk = await request('jobs/' + encodeURIComponent(id) + '/output?' + (output.offset ? 'offset=' + output.offset : 'tail=262144'));
      output.text = (output.text + (chunk.text || '')).slice(-400000);
      output.offset = chunk.offset;
      if (chunk.offset >= chunk.size) break;
    }
  }
  function stepInstall() {
    const job = W.job;
    const logPre = h('pre', { class: 'n-pre n-terminal s-log', 'aria-label': '详细日志' }, output.text || '等待命令输出…');
    requestAnimationFrame(() => { logPre.scrollTop = logPre.scrollHeight; });
    const detail = h('div', { class: 'n-cols-2', style: 'margin-top:20px' },
      h('section', { class: 'n-sec' }, secHead('步骤'), h('div', { class: 'n-box n-box-pad' }, stepList(job))),
      h('section', { class: 'n-sec' }, secHead('详细日志'), logPre));
    if (job.status === 'succeeded') {
      const running = job.result?.runtime?.running;
      return {
        actions: [running && opts.consoleUrl ? h('a', { class: 'n-btn', href: opts.consoleUrl, html: icon('external-link', 'n-icon--sm') }, '打开控制台') : null,
          primary('进入管理界面', () => opts.onDone?.())].filter(Boolean),
        content: [h('div', { class: 'n-hl s-done' }, h('span', { class: 's-mark s-mark--ok', html: icon('check') }),
          h('div', {}, h('div', { class: 'n-t2' }, running ? '安装完成，业务服务已经启动' : '安装完成'),
            h('span', { class: 'n-meta' }, '已安装 ' + (job.result?.version || '') + ' 和 ' + (job.result?.modules || []).length + ' 个功能模块，用时 ' + fmtDur(jobSeconds(job)) + '。之后的更新、模块调整和启停都在系统页进行。'))), detail],
      };
    }
    if (!jobRunning(job)) {
      const cancelled = job.status === 'cancelled';
      return {
        actions: [primary('返回安装计划', () => { W.job = null; go(2); void makePlan(); })],
        content: [h('div', { class: 'n-hl ' + (cancelled ? 'n-hl--warn' : 'n-hl--bad') + ' s-done' }, h('span', { class: 's-mark ' + (cancelled ? 's-mark--warn' : 's-mark--bad'), html: icon(cancelled ? 'triangle-alert' : 'x') }),
          h('div', {}, h('div', { class: 'n-t2' }, cancelled ? '安装已取消' : '安装没有完成'),
            h('span', { class: 'n-meta' }, cancelled ? '已在安全位置停止。已下载的文件保留在安装位置，重新安装时可以复用。' : (job.error?.message || job.message) + ' ' + adviceOf(job)))), detail],
      };
    }
    const percent = (job.progress || 0) + '%';
    return {
      actions: [h('button', { class: 'n-btn', type: 'button', onclick: async () => {
        const ok = await confirmDialog({ title: '取消安装', icon: 'triangle-alert', confirmText: '确认取消安装',
          text: '安装会在当前步骤的安全位置停止。已下载的文件保留在安装位置，重新安装时可以复用。' });
        if (!ok) return;
        try { W.job = await request('jobs/' + encodeURIComponent(job.id) + '/cancel', { method: 'POST', body: {} }); toast('已请求取消，正在等待安全位置', { tone: 'info' }); render(); }
        catch (error) { toast('取消没有送达：' + error.message, { tone: 'bad' }); }
      } }, '取消安装')],
      content: [h('section', { class: 'n-hl', 'aria-label': '安装进度' },
        h('div', { class: 's-inst-head' }, h('div', {}, h('div', { class: 'n-t3' }, (STAGE_TITLE[job.stage] || job.stage) + '：' + (job.message || '')),
          h('span', { class: 'n-meta' }, '已用时 ' + fmtDur(jobSeconds(job)) + '，安装到 ' + inst().install_dir + '。关闭这个页面不会中断安装。')), h('span', { class: 'n-t1 n-num' }, percent)),
        h('div', { class: 'n-progress s-inst-bar' }, h('span', { style: 'width:' + percent }))), detail],
    };
  }

  void start();
  return { destroy() { clearInterval(timer); } };
}
