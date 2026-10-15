// NEH 端侧管理台前端逻辑
// 约束：所有状态来自 /api/manager/* 真实响应；不内置示例数据，不预置成功状态。

(function () {
  'use strict';

  // ===== 全局状态 =====
  const state = {
    csrfToken: '',
    passwordEnabled: false,
    platformSupported: true,
    currentView: 'overview',
    status: null,

    // 安装向导
    wizardStep: 'hardware',
    hardware: null,
    recommendation: null,
    modules: [],
    selectedModules: new Set(),
    robot: '',
    plan: null,
    installJobId: null,

    // 配置
    config: null,
    configValues: {},
    configRevision: null,

    // 请求代次：用于丢弃迟到的异步响应
    planGeneration: 0,
    jobGeneration: 0,
    pollTimer: null,
    submitting: false,

    // 设备身份与管理程序重启
    instanceId: null,
    managerRestarting: false
  };

  const POLL_INTERVAL_MS = 2000;

  // ===== DOM 助手 =====
  function $(id) {
    return document.getElementById(id);
  }

  function text(el, value) {
    if (el) el.textContent = value == null ? '' : String(value);
  }

  function show(el, visible) {
    if (el) el.hidden = !visible;
  }

  function setClass(el, className) {
    if (el) el.className = className;
  }

  function robotLabel(key) {
    return (state.robotLabels || {})[key] || key || '--';
  }

  function moduleLabel(key) {
    const entry = state.modules.find(function (item) { return item.key === key; });
    return entry ? (entry.title || key) : key;
  }

  function stageLabel(stage) {
    return ({queued:'等待开始',running:'正在处理',complete:'已完成',downloading:'下载程序',
      verifying:'校验程序',system_dependencies:'安装系统依赖',dependencies:'下载运行依赖',
      python_environment:'准备运行环境',runtime_files:'准备启动文件',stopping:'停止原服务',
      switching:'切换版本',starting:'启动服务',rollback:'恢复原版本',failed:'未完成',
      source_download:'获取源码',native_build:'增量构建',source_verify:'验证候选版本',models:'准备模型',
      manager_source:'从源码准备管理程序',manager_download:'下载管理程序',manager_verify:'校验管理程序',manager_switch:'切换管理程序'})[stage] || '正在处理';
  }

  function escapeHtml(value) {
    if (value == null) return '';
    return String(value)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  function formatTime(value) {
    if (!value) return '--';
    const date = new Date(value);
    if (Number.isNaN(date.getTime())) return String(value);
    return date.toLocaleString('zh-CN', { hour12: false });
  }

  function formatBytes(bytes) {
    if (bytes == null || Number.isNaN(Number(bytes))) return '--';
    const value = Number(bytes);
    if (value < 1024) return `${value} B`;
    if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KB`;
    if (value < 1024 * 1024 * 1024) return `${(value / 1024 / 1024).toFixed(1)} MB`;
    return `${(value / 1024 / 1024 / 1024).toFixed(2)} GB`;
  }

  // ===== 消息提示 =====
  function showMessage(el, message, kind) {
    if (!el) return;
    if (!message) {
      el.hidden = true;
      el.textContent = '';
      return;
    }
    el.hidden = false;
    el.className = 'status-message' + (kind ? ` ${kind}` : '');
    el.textContent = message;
  }

  function inlineMessage(el, message, kind) {
    if (!el) return;
    if (!message) {
      el.hidden = true;
      el.textContent = '';
      return;
    }
    el.hidden = false;
    el.className = 'inline-message' + (kind ? ` ${kind}` : '');
    el.textContent = message;
  }

  // ===== HTTP 层 =====
  const ApiError = class extends Error {
    constructor(message, status, code, details) {
      super(message);
      this.name = 'ApiError';
      this.status = status;
      this.code = code;
      this.details = details;
    }
  };

  async function apiFetch(path, options) {
    const opts = options || {};
    const headers = Object.assign({}, opts.headers);

    if (opts.method && opts.method !== 'GET') {
      if (state.csrfToken) headers['X-NEH-CSRF'] = state.csrfToken;
    }

    let body = opts.body;
    if (body != null && typeof body === 'object') {
      headers['Content-Type'] = 'application/json';
      body = JSON.stringify(body);
    }

    let response;
    try {
      response = await fetch(path, {
        method: opts.method || 'GET',
        headers,
        body,
        credentials: 'same-origin'
      });
    } catch (error) {
      throw new ApiError('无法连接到管理服务，请确认服务是否在运行。', 0, 'network_error');
    }

    const contentType = response.headers.get('content-type') || '';
    let payload = null;
    let parseFailed = false;

    if (contentType.includes('application/json')) {
      try {
        payload = await response.json();
      } catch (error) {
        parseFailed = true;
      }
    } else {
      // 非 JSON 错误（例如反向代理返回的 HTML 错误页）按文本读取
      let raw = '';
      try {
        raw = await response.text();
      } catch (error) {
        raw = '';
      }
      payload = { error: { message: raw ? raw.slice(0, 400) : '' } };
      parseFailed = true;
    }

    if (response.status === 401) {
      throw new ApiError('需要验证密码。', 401, 'auth_required');
    }

    if (!response.ok) {
      const detail = payload && payload.error ? payload.error : {};
      let message = detail.message;
      if (!message) {
        message = parseFailed
          ? `请求失败（HTTP ${response.status}）。服务返回了非预期内容。`
          : `请求失败（HTTP ${response.status}）。`;
      }
      throw new ApiError(message, response.status, detail.code, detail.details);
    }

    if (parseFailed && payload && payload.error) {
      throw new ApiError(
        '服务返回了非预期内容，无法解析为 JSON。',
        response.status,
        'invalid_response'
      );
    }

    return payload;
  }

  // 管理重启会丢弃旧会话；先刷新会话，仅在确实启用密码时要求登录。
  async function apiCall(path, options) {
    try {
      return await apiFetch(path, options);
    } catch (error) {
      if (error instanceof ApiError && error.status === 401) {
        const session = await refreshSession();
        if (session.password_enabled && !session.authenticated) await requestLogin();
        return await apiFetch(path, options);
      }
      if (error instanceof ApiError && error.status === 403 && error.code === 'csrf_invalid') {
        await refreshSession();
        return await apiFetch(path, options);
      }
      throw error;
    }
  }

  // 计划可能因业务正在运行而要求明确停机，这里按后端要求补上 stop_running 后重新生成。
  async function withStopConfirmation(plan, action, params) {
    if (!plan.blockers || !plan.blockers.length) return plan;
    const needsStop = plan.requires_stop && plan.blockers.some(function (item) {
      return item.indexOf('stop_running') !== -1;
    });
    if (!needsStop) return plan;

    const ok = await confirmDialog({
      title: '需要停止业务服务',
      text: '该操作需要先停止正在运行的业务服务。停止会中断当前任务，且任务不会跨版本恢复。',
      items: plan.blockers,
      confirmText: '停止并继续',
      danger: true
    });
    if (!ok) return null;

    return await apiCall('/api/manager/plan', {
      method: 'POST',
      body: Object.assign({}, params, { action: action, stop_running: true })
    });
  }

  // ===== 登录对话框 =====
  let loginResolve = null;

  function requestLogin() {
    return new Promise(function (resolve) {
      loginResolve = resolve;
      const dialog = $('loginDialog');
      const form = $('loginForm');
      const errorEl = $('loginError');
      $('loginPasswordInput').value = '';
      show(errorEl, false);
      if (!dialog.open) dialog.showModal();
      form.onsubmit = null;
      $('loginPasswordInput').focus();

      form.onsubmit = async function (event) {
        event.preventDefault();
        const password = $('loginPasswordInput').value;
        if (!password) return;
        const submitBtn = form.querySelector('button[type="submit"]');
        submitBtn.disabled = true;
        try {
          await apiFetch('/api/manager/login', { method: 'POST', body: { password } });
          await refreshSession();
          dialog.close();
          submitBtn.disabled = false;
          const resolveFn = loginResolve;
          loginResolve = null;
          if (resolveFn) resolveFn();
        } catch (error) {
          submitBtn.disabled = false;
          text(errorEl, error.message || '密码不正确。');
          show(errorEl, true);
        }
      };
    });
  }

  function cancelLogin() {
    const dialog = $('loginDialog');
    if (dialog.open) dialog.close();
    const resolveFn = loginResolve;
    loginResolve = null;
    if (resolveFn) resolveFn();
  }

  async function refreshSession() {
    const session = await apiFetch('/api/manager/session');
    state.csrfToken = session.csrf_token || '';
    state.passwordEnabled = Boolean(session.password_enabled);
    return session;
  }

  // ===== 主题 =====
  function initTheme() {
    const html = document.documentElement;
    const prefersDark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
    const saved = localStorage.getItem('nuwax-theme') || localStorage.getItem('neh-manager-theme') || (prefersDark ? 'dark' : 'light');
    html.setAttribute('data-theme', saved);
    updateThemeIcon(saved);
    $('themeToggle').addEventListener('click', function () {
      const next = html.getAttribute('data-theme') === 'light' ? 'dark' : 'light';
      html.setAttribute('data-theme', next);
      localStorage.setItem('nuwax-theme', next);
      updateThemeIcon(next);
    });
  }

  function updateThemeIcon(theme) {
    const btn = $('themeToggle');
    text(btn, theme === 'light' ? '🌙' : '☀️');
  }

  // ===== 视图切换 =====
  // 三个顶层视图互斥；用 hidden 与 active 同步，避免属性与类不一致造成空白。
  function activateView(name) {
    ['bootView', 'setupView', 'mainView'].forEach(function (id) {
      const el = $(id);
      if (!el) return;
      const current = id === name;
      el.hidden = !current;
      el.classList.toggle('active', current);
    });
  }

  function showBoot(message, retryable) {
    activateView('bootView');
    text($('bootText'), message || '正在读取设备状态…');
    const icon = document.querySelector('#bootView .status-icon');
    if (icon) icon.className = 'status-icon' + (retryable ? ' error' : ' spinner');
    show($('bootActions'), Boolean(retryable));
  }

  function showView(view) {
    document.querySelectorAll('.nav-item').forEach(function (item) {
      item.classList.toggle('active', item.getAttribute('data-view') === view);
    });
    document.querySelectorAll('.main-pane').forEach(function (pane) {
      pane.classList.toggle('active', pane.id === view + 'Pane');
    });
    state.currentView = view;
    activateView('mainView');
    window.location.hash = view === 'overview' ? '' : view;
  }

  function showSetup() {
    activateView('setupView');
  }

  function initNavigation() {
    document.querySelectorAll('.nav-item').forEach(function (item) {
      item.addEventListener('click', function () {
        const view = item.getAttribute('data-view');
        showView(view);
        if (view === 'modules') loadModulesView();
        else if (view === 'updates') loadUpdatesView();
        else if (view === 'jobs') loadJobsView();
        else if (view === 'config') loadConfigView();
        else if (view === 'settings') loadSettingsView();
        else loadOverview();
      });
    });
  }

  // ===== 确认对话框 =====
  function confirmDialog(options) {
    return new Promise(function (resolve) {
      const dialog = $('confirmDialog');
      const okBtn = $('confirmOk');
      const cancelBtn = $('confirmCancel');
      const extra = $('confirmExtra');

      text($('confirmTitle'), options.title || '确认操作');
      text($('confirmText'), options.text || '');

      const items = Array.isArray(options.items) ? options.items : [];
      if (items.length) {
        extra.innerHTML = '<ul>' + items.map(function (item) {
          return '<li>' + escapeHtml(item) + '</li>';
        }).join('') + '</ul>' + (options.foot ? '<p class="muted small">' + escapeHtml(options.foot) + '</p>' : '');
        extra.hidden = false;
      } else {
        extra.innerHTML = '';
        extra.hidden = true;
      }

      okBtn.className = options.danger ? 'btn' : 'btn primary';
      okBtn.textContent = options.confirmText || '确认';

      function cleanup(result) {
        okBtn.onclick = null;
        cancelBtn.onclick = null;
        dialog.onclose = null;
        if (dialog.open) dialog.close();
        resolve(result);
      }

      okBtn.onclick = function () { cleanup(true); };
      cancelBtn.onclick = function () { cleanup(false); };
      dialog.onclose = function () { cleanup(false); };

      openModal(dialog);
    });
  }

  // 同一对话框重复 showModal 会抛异常；统一走这里，已打开时先关闭再复用。
  function openModal(dialog) {
    if (!dialog) return;
    if (dialog.open) dialog.close();
    dialog.showModal();
  }

  // ===== 设备信息（平台）=====
  function applyStatus(status) {
    state.status = status;
    state.passwordEnabled = Boolean(status.password_enabled);
    state.platformSupported = status.platform_supported !== false;

    text($('managerVersion'), '管理器 ' + (status.manager_development ? '开发版本' : (status.manager_version || '--')));
    const installDir = status.installation && status.installation.install_dir;
    text($('setupInstallDir'), installDir || '未知');
    text($('installDir'), installDir || '未知');

    if (status.installation && status.installation.installed) {
      const robot = status.installation.robot ? status.installation.robot : '';
      text($('platformInfo'), robot ? robotLabel(robot) : '已安装');
    } else {
      text($('platformInfo'), '未安装');
    }
  }

  // ===== 概览 =====
  async function loadOverview() {
    const messageEl = $('overviewLoadError');
    show(messageEl, false);
    try {
      const status = await apiCall('/api/manager/status');
      applyStatus(status);
      renderOverview(status);
    } catch (error) {
      show(messageEl, true);
      text(messageEl, '读取设备状态失败：' + error.message);
    }
  }

  function renderOverview(status) {
    const installation = status.installation || {};
    const runtime = status.runtime || {};
    const update = status.update || {};

    // 安装信息
    const installBadge = $('installStatus');
    if (installation.installed) {
      text(installBadge, '已安装');
      setClass(installBadge, 'status-badge good');
      text($('currentVersion'), installation.version || '--');
      text($('robotModel'), robotLabel(installation.robot));
      const modules = Array.isArray(installation.modules) ? installation.modules : [];
      text($('installedModules'), modules.length ? modules.map(moduleLabel).join('、') : '--');
      if (installation.previous_version) {
        show($('previousVersionRow'), true);
        text($('previousVersion'), installation.previous_version);
      } else {
        show($('previousVersionRow'), false);
      }
      if (installation.source) {
        show($('installSourceRow'), true);
        text($('installSource'), ({git:'源码构建',release:'版本包',migrated:'已有安装迁入'})[installation.source] || installation.source);
      } else {
        show($('installSourceRow'), false);
      }
    } else {
      text(installBadge, '未安装');
      setClass(installBadge, 'status-badge warn');
      text($('currentVersion'), '尚未安装');
      text($('robotModel'), '--');
      text($('installedModules'), '--');
      show($('previousVersionRow'), false);
      show($('installSourceRow'), false);
    }

    // 运行状态
    const runtimeBadge = $('runtimeStatus');
    const runtimeError = $('runtimeError');
    if (runtime.running) {
      if (runtime.healthy) {
        text(runtimeBadge, '运行正常');
        setClass(runtimeBadge, 'status-badge running');
        text($('servicesState'), '运行中');
      } else {
        text(runtimeBadge, '运行异常');
        setClass(runtimeBadge, 'status-badge warn');
        text($('servicesState'), '异常');
      }
      if (runtime.error) {
        text(runtimeError, runtime.error);
        show(runtimeError, true);
      } else {
        show(runtimeError, false);
      }
      show($('startRuntime'), false);
      show($('stopRuntime'), true);
      show($('restartRuntime'), true);
    } else {
      text(runtimeBadge, '未运行');
      setClass(runtimeBadge, 'status-badge');
      text($('servicesState'), '已停止');
      if (runtime.error) {
        text(runtimeError, runtime.error);
        show(runtimeError, true);
      } else {
        show(runtimeError, false);
      }
      show($('startRuntime'), installation.installed);
      show($('stopRuntime'), false);
      show($('restartRuntime'), false);
    }

    // 更新：业务版本与管理程序分别判断，两者都可能同时存在。
    const updateBadge = $('updateStatus');
    const boxAvailable = $('updateAvailable');
    const boxNone = $('updateNone');
    const boxError = $('updateError');
    const managerNote = $('managerUpdateHint');
    if (update.error) {
      text(updateBadge, '检查失败');
      setClass(updateBadge, 'status-badge bad');
      show(boxAvailable, false);
      show(boxNone, false);
      show(boxError, true);
      text($('updateErrorText'), update.error);
    } else if (update.available || update.manager_available) {
      text(updateBadge, '有可用更新');
      setClass(updateBadge, 'status-badge info');
      show(boxAvailable, true);
      show(boxNone, false);
      show(boxError, false);
      text($('availableVersion'), update.available ? (update.version || '--') : '仅管理程序有更新');
    } else {
      text(updateBadge, '已是最新');
      setClass(updateBadge, 'status-badge good');
      show(boxAvailable, false);
      show(boxNone, true);
      show(boxError, false);
    }
    if (managerNote) {
      if (update.manager_available && update.manager_version) {
        managerNote.hidden = false;
        text(managerNote, '管理程序可更新到 ' + update.manager_version + '，可在“版本更新”中执行。');
      } else {
        managerNote.hidden = true;
      }
    }

    // 进行中任务
    const job = status.active_job;
    const jobCard = $('activeJobCard');
    if (job) {
      show(jobCard, true);
      const badge = $('activeJobStatus');
      text(badge, jobStatusText(job.status));
      setClass(badge, 'status-badge ' + jobStatusClass(job.status));
      renderJobInto($('activeJobInfo'), job, { compact: true });
    } else {
      show(jobCard, false);
    }

    // 未安装时引导进入向导：概览不渲染为空数据，只说明实际状态。
    const banner = $('overviewLoadError');
    if (!installation.installed) {
      show(banner, true);
      setClass(banner, 'status-message');
      banner.innerHTML = '设备尚未安装业务版本。<button class="btn primary" type="button" id="gotoSetup">开始安装</button>';
      const btn = $('gotoSetup');
      if (btn) btn.addEventListener('click', function () { startSetupWizard(); });
    } else if (banner && banner.querySelector('#gotoSetup')) {
      show(banner, false);
      text(banner, '');
    }
  }

  // ===== 任务渲染 =====
  function jobStatusText(status) {
    const map = {
      queued: '排队中',
      running: '执行中',
      succeeded: '已完成',
      failed: '失败',
      cancelled: '已取消',
      interrupted: '已中断'
    };
    return map[status] || status || '--';
  }

  function jobStatusClass(status) {
    const map = {
      queued: 'info',
      running: 'running',
      succeeded: 'good',
      failed: 'bad',
      cancelled: 'warn',
      interrupted: 'warn'
    };
    return map[status] || '';
  }

  function actionText(action) {
    const map = {
      install: '首次安装',
      update: '版本更新',
      rollback: '版本回退',
      modules: '模块变更',
      start: '启动业务服务',
      stop: '停止业务服务',
      restart: '重启业务服务',
      migrate: '迁移既有安装'
    };
    return map[action] || action || '任务';
  }

  function canCancel(job) {
    return job && (job.status === 'queued' || job.status === 'running');
  }

  // 后端日志为 {time, stage, message} 记录数组，按时间与阶段展开为可读行。
  function formatLog(entries) {
    if (!Array.isArray(entries)) return String(entries == null ? '' : entries);
    return entries.map(function (entry) {
      if (entry == null) return '';
      if (typeof entry === 'string') return entry;
      const time = entry.time ? formatTime(entry.time) : '';
      const stage = entry.stage || '';
      const message = entry.message || '';
      return [time, stage ? '[' + stage + ']' : '', message].filter(Boolean).join(' ');
    }).join('\n');
  }

  function renderJobInto(container, job, options) {
    if (!container) return;
    const opts = options || {};
    const parts = [];

    if (!opts.compact) {
      parts.push(
        '<div class="job-header">' +
          '<div class="job-title">' + escapeHtml(actionText(job.action)) + '</div>' +
          '<span class="status-badge ' + jobStatusClass(job.status) + '">' + escapeHtml(jobStatusText(job.status)) + '</span>' +
        '</div>'
      );
    }

    if (job.status === 'running' || job.status === 'queued') {
      const progress = typeof job.progress === 'number' ? Math.max(0, Math.min(100, job.progress)) : 0;
      parts.push(
        '<div class="job-progress">' +
          '<div class="progress-bar"><div class="progress-fill" style="width:' + progress + '%"></div></div>' +
          '<div class="progress-head">' +
            '<span class="progress-stage">' + escapeHtml(stageLabel(job.stage)) + '</span>' +
            '<span class="progress-percent mono">' + progress + '%</span>' +
          '</div>' +
          (job.message ? '<div class="progress-message">' + escapeHtml(job.message) + '</div>' : '') +
        '</div>'
      );
    } else if (job.message) {
      parts.push('<div class="progress-message">' + escapeHtml(job.message) + '</div>');
    }

    const info = [];
    info.push(['任务编号', job.id]);
    if (job.version) info.push(['版本', job.version]);
    if (job.stage && job.status !== 'running') info.push(['阶段', stageLabel(job.stage)]);
    info.push(['创建时间', formatTime(job.created_at)]);
    if (job.updated_at) info.push(['更新时间', formatTime(job.updated_at)]);

    parts.push('<div class="job-info">' + info.map(function (row) {
      return '<div class="job-info-label">' + escapeHtml(row[0]) + '</div>' +
             '<div class="job-info-value">' + escapeHtml(row[1]) + '</div>';
    }).join('') + '</div>');

    if (job.error) {
      const errText = typeof job.error === 'string' ? job.error : (job.error.message || JSON.stringify(job.error));
      parts.push('<div class="error-message">' + escapeHtml(errText) + '</div>');
    }

    if (!opts.compact && Array.isArray(job.log) && job.log.length) {
      parts.push(
        '<details class="job-log-details" open>' +
          '<summary>任务日志（' + job.log.length + ' 条）</summary>' +
          '<div class="job-log">' + escapeHtml(formatLog(job.log)) + '</div>' +
        '</details>'
      );
    }

    if (canCancel(job)) {
      parts.push(
        '<div class="job-actions">' +
          '<button class="btn" type="button" data-cancel-job="' + escapeHtml(job.id) + '">取消任务</button>' +
        '</div>'
      );
    }

    container.innerHTML = parts.join('');

    const cancelBtn = container.querySelector('[data-cancel-job]');
    if (cancelBtn) {
      cancelBtn.addEventListener('click', function () {
        cancelJob(cancelBtn.getAttribute('data-cancel-job'));
      });
    }
  }

  async function cancelJob(jobId) {
    const ok = await confirmDialog({
      title: '取消任务',
      text: '任务会在安全边界停止。已完成的步骤不会回退。',
      confirmText: '取消任务',
      danger: true
    });
    if (!ok) return;
    try {
      const job = await apiCall('/api/manager/jobs/' + encodeURIComponent(jobId) + '/cancel', { method: 'POST' });
      renderJobInto($('activeJobInfo'), job, { compact: true });
      loadOverview();
      if (state.currentView === 'jobs') loadJobsView();
    } catch (error) {
      showMessage($('overviewLoadError'), '取消任务失败：' + error.message, 'error');
    }
  }

  // ===== 任务轮询 =====
  function trackJob(jobId) {
    state.jobGeneration += 1;
    const generation = state.jobGeneration;
    state.installJobId = jobId;
    if (state.pollTimer) clearInterval(state.pollTimer);

    state.pollTimer = setInterval(async function () {
      if (generation !== state.jobGeneration) {
        clearInterval(state.pollTimer);
        return;
      }
      try {
        const job = await apiCall('/api/manager/jobs/' + encodeURIComponent(jobId));
        if (generation !== state.jobGeneration) return;

        renderJobInto($('activeJobInfo'), job, { compact: true });

        if (state.currentView === 'jobs') {
          renderJobsList([job]);
        }
        if (state.wizardStep === 'install') {
          renderInstallProgress(job);
        }

        if (job.status !== 'running' && job.status !== 'queued') {
          clearInterval(state.pollTimer);
          state.pollTimer = null;
          finalizeWizardInstall(job);
          loadOverview();
        }
      } catch (error) {
        if (generation !== state.jobGeneration) return;
        // 网络抖动不终止轮询，但把真实原因显示出来
        if (state.wizardStep === 'install') {
          text($('installMessage'), '读取任务状态失败：' + error.message);
        }
      }
    }, POLL_INTERVAL_MS);
  }

  // ===== 安装向导 =====
  // HTML 初始用 hidden 标记未展开步骤，这里同时切换 hidden 与 active，
  // 避免 [hidden] 的 display:none!important 覆盖 .active 的显示。
  function setWizardStep(step) {
    state.wizardStep = step;
    const order = ['hardware', 'modules', 'plan', 'install'];
    const index = order.indexOf(step);

    order.forEach(function (name) {
      const section = document.querySelector('.wizard-step[data-step="' + name + '"]');
      if (section) {
        const current = name === step;
        section.hidden = !current;
        section.classList.toggle('active', current);
      }
      const mark = document.querySelector('.wizard-progress-item[data-step-mark="' + name + '"]');
      if (mark) {
        const markIndex = order.indexOf(name);
        mark.classList.toggle('active', name === step);
        mark.classList.toggle('done', markIndex < index);
      }
    });

    if (step === 'hardware') loadHardware();
    if (step === 'modules') loadWizardModules();
  }

  function startSetupWizard() {
    showSetup();
    setWizardStep('hardware');
    loadMigrationSources();
  }

  async function loadMigrationSources() {
    try {
      const data = await apiCall('/api/manager/migration-sources');
      const sources = Array.isArray(data.sources) ? data.sources : [];

      if (sources.length > 0) {
        state.migrationSources = sources;
        state.selectedMigrationSource = null;
        renderMigrationPanel(sources);
      } else {
        show($('migrationPanel'), false);
        loadHardware();
      }
    } catch (error) {
      // 接口不存在或失败时直接进入硬件检测
      show($('migrationPanel'), false);
      loadHardware();
    }
  }

  function renderMigrationPanel(sources) {
    const panel = $('migrationPanel');
    const sourcesContainer = $('migrationSources');
    const migrateBtn = $('migrateFrom');

    sourcesContainer.innerHTML = sources.map(function (src, index) {
      const meta = [];
      if (src.launcher) meta.push('启动器：' + src.launcher);
      if (src.systemd) meta.push('systemd 服务');
      if (src.running) meta.push('运行中');

      return '<div class="migration-source-item" data-source-id="' + escapeHtml(src.id) + '">' +
        '<div class="migration-source-radio"></div>' +
        '<div class="migration-source-info">' +
        '<div class="migration-source-label">' + escapeHtml(src.source) + '</div>' +
        (meta.length ? '<div class="migration-source-meta">' + escapeHtml(meta.join(' · ')) + '</div>' : '') +
        '</div>' +
        '</div>';
    }).join('');

    sourcesContainer.querySelectorAll('.migration-source-item').forEach(function (item) {
      item.addEventListener('click', function () {
        sourcesContainer.querySelectorAll('.migration-source-item').forEach(function (i) {
          i.classList.remove('selected');
        });
        item.classList.add('selected');
        state.selectedMigrationSource = item.getAttribute('data-source-id');
        migrateBtn.disabled = false;
      });
    });

    $('migrateFresh').onclick = function () {
      state.selectedMigrationSource = null;
      show(panel, false);
      loadHardware();
    };

    migrateBtn.onclick = function () {
      if (!state.selectedMigrationSource) return;
      show(panel, false);
      loadHardware();
    };

    show(panel, true);
    show($('hardwareStatus'), false);
  }

  async function loadHardware() {
    const statusCard = $('hardwareStatus');
    const statusText = $('hardwareStatusText');
    setClass(statusCard, 'status-card checking');
    show(statusCard, true);
    statusCard.querySelector('.status-icon').className = 'status-icon spinner';
    text(statusText, '正在检测硬件与系统环境…');
    show($('hardwareDetails'), false);
    show($('hardwareReasons'), false);
    show($('hardwareIssues'), false);
    show($('retryHardware'), false);
    $('nextToModules').disabled = true;

    try {
      const data = await apiCall('/api/manager/hardware');
      state.hardware = data.hardware || {};
      state.recommendation = data.recommendation || {};
      state.robot = state.recommendation.robot || '';
      state.selectedModules = new Set(
        Array.isArray(state.recommendation.modules) ? state.recommendation.modules : []
      );

      if (data.supported === false) {
        setClass(statusCard, 'status-card error');
        statusCard.querySelector('.status-icon').className = 'status-icon error';
        text(statusText, '当前平台不在已支持范围内。');
        show($('retryHardware'), true);
        $('nextToModules').disabled = true;
      } else {
        setClass(statusCard, 'status-card success');
        statusCard.querySelector('.status-icon').className = 'status-icon check';
        text(statusText, '硬件检查完成，已给出推荐配置。');
        show($('retryHardware'), true);
        $('nextToModules').disabled = false;
      }

      renderHardware(data);
    } catch (error) {
      setClass(statusCard, 'status-card error');
      statusCard.querySelector('.status-icon').className = 'status-icon error';
      text(statusText, '硬件检测失败：' + error.message);
      show($('retryHardware'), true);
      $('nextToModules').disabled = true;
    }
  }

  // 布尔量一律转成明确中文描述；未知（null/undefined）不当作“未检测到”。
  function booleanText(value, trueText, falseText) {
    if (value === true) return trueText;
    if (value === false) return falseText;
    return '未知';
  }

  // 容量字段按后端实际字节数换算，0 与缺省都显示为“未知”，不臆造数值。
  function capacityText(bytes) {
    if (bytes == null || bytes === '' || Number(bytes) <= 0) return '未知';
    return formatBytes(bytes);
  }

  function renderHardware(data) {
    const hardware = data.hardware || {};
    const grid = $('hardwareDetails');

    const osText = hardware.os_id && hardware.os_id !== 'unknown'
      ? hardware.os_id + (hardware.version_id && hardware.version_id !== 'unknown' ? ' ' + hardware.version_id : '')
      : '未知';

    // 显卡分三层说明：是否有 NVIDIA 设备、是否 Jetson 平台、已报告显存。
    let gpuText;
    if (hardware.nvidia_gpu === true) {
      gpuText = hardware.is_jetson === true ? '已检测到（Jetson 集成）' : '已检测到';
    } else if (hardware.nvidia_gpu === false) {
      gpuText = '未检测到';
    } else {
      gpuText = '未知';
    }
    if (hardware.gpu_memory_bytes) {
      gpuText += '，显存 ' + formatBytes(hardware.gpu_memory_bytes);
    }

    const rows = [
      ['操作系统', osText],
      ['架构', hardware.architecture && hardware.architecture !== 'unknown' ? hardware.architecture : '未知'],
      ['CPU 核心', hardware.cpu_count != null ? hardware.cpu_count + ' 核' : '未知'],
      ['可用内存', capacityText(hardware.memory_bytes)],
      ['系统内存总量', capacityText(hardware.memory_total_bytes)],
      ['可用磁盘', capacityText(hardware.free_disk_bytes)],
      ['图形设备', gpuText],
      ['摄像头节点', booleanText(hardware.camera_detected, '已检测到', '未检测到')]
    ];

    grid.innerHTML = rows.map(function (row) {
      return '<div class="hardware-item">' +
        '<div class="hardware-item-label">' + escapeHtml(row[0]) + '</div>' +
        '<div class="hardware-item-value">' + escapeHtml(row[1]) + '</div>' +
      '</div>';
    }).join('');
    show(grid, true);

    // 探测证据按原文展示，便于核对来源，不做主观替换。默认折叠。
    const evidence = Array.isArray(hardware.evidence) ? hardware.evidence : [];
    const evidenceBox = $('hardwareEvidence');
    if (evidenceBox) {
      if (evidence.length) {
        evidenceBox.innerHTML = '<details class="evidence-details">' +
          '<summary class="evidence-title">检测详情</summary>' +
          '<div class="evidence-list">' +
          evidence.map(function (item) {
            return '<div class="evidence-item">' + escapeHtml(item) + '</div>';
          }).join('') +
          '</div></details>';
        show(evidenceBox, true);
      } else {
        show(evidenceBox, false);
      }
    }

    const reasons = Array.isArray(data.recommendation && data.recommendation.reasons)
      ? data.recommendation.reasons
      : [];
    const reasonBox = $('hardwareReasons');
    if (reasons.length) {
      reasonBox.innerHTML = reasons.map(function (reason) {
        return '<div class="reason-item">' + escapeHtml(reason) + '</div>';
      }).join('');
      show(reasonBox, true);
    } else {
      show(reasonBox, false);
    }

    const issues = Array.isArray(data.issues) ? data.issues : [];
    const issueBox = $('hardwareIssues');
    if (issues.length) {
      issueBox.innerHTML = issues.map(function (issue) {
        const value = typeof issue === 'string' ? issue : (issue.message || JSON.stringify(issue));
        const level = typeof issue === 'object' && issue.level === 'error' ? ' error' : '';
        return '<div class="issue-item' + level + '">' +
          '<span class="issue-icon">!</span>' +
          '<span class="issue-text">' + escapeHtml(value) + '</span>' +
        '</div>';
      }).join('');
      show(issueBox, true);
    } else {
      show(issueBox, false);
    }
  }

  async function loadWizardModules() {
    const grid = $('wizardModules');
    grid.innerHTML = '<p class="info-text">正在读取模块列表…</p>';
    try {
      const [data, settings] = await Promise.all([
        apiCall('/api/manager/modules'),
        apiCall('/api/manager/settings')
      ]);
      state.modules = Array.isArray(data.modules) ? data.modules : [];
      if (!state.selectedModules.size && Array.isArray(data.selected)) {
        state.selectedModules = new Set(data.selected);
      }
      if (!state.robot && data.robot) state.robot = data.robot;

      // 保存 robots 数组和 robot_labels 映射
      state.robots = Array.isArray(data.robots) ? data.robots : [];
      state.robotLabels = data.robot_labels || {};

      applySourceSettings(settings, 'wizard');
      renderWizardModules();
    } catch (error) {
      grid.innerHTML = '<p class="error-message">读取模块列表失败：' + escapeHtml(error.message) + '</p>';
    }
  }

  function renderWizardModules() {
    const grid = $('wizardModules');
    const hint = $('modulesHint');

    const warnings = Array.isArray(state.recommendation && state.recommendation.warnings)
      ? state.recommendation.warnings
      : [];
    if (warnings.length) {
      hint.innerHTML = warnings.map(function (item) {
        return '<div class="hint-item">' + escapeHtml(item) + '</div>';
      }).join('');
      show(hint, true);
    } else {
      show(hint, false);
    }

    if (!state.modules.length) {
      grid.innerHTML = '<p class="info-text">暂时没有可安装的模块。请先在上方“更新来源设置”中填写版本源地址并保存。</p>';
      return;
    }

    // 构建模块key到title的映射，用于依赖/冲突显示
    const moduleTitles = {};
    state.modules.forEach(function (m) {
      moduleTitles[m.key] = m.title || m.key;
    });

    grid.innerHTML = state.modules.map(function (mod) {
      const selected = state.selectedModules.has(mod.key);
      const tags = [];
      if (mod.installed) tags.push('<span class="module-tag installed">已安装</span>');
      if (mod.key === 'core') tags.push('<span class="module-tag">必需</span>');
      (mod.requires || []).forEach(function (dep) {
        const title = moduleTitles[dep] || dep;
        tags.push('<span class="module-tag">依赖 ' + escapeHtml(title) + '</span>');
      });
      (mod.conflicts || []).forEach(function (conf) {
        const title = moduleTitles[conf] || conf;
        tags.push('<span class="module-tag">冲突 ' + escapeHtml(title) + '</span>');
      });
      return '<div class="module-card' + (selected ? ' selected' : '') + '" data-module="' + escapeHtml(mod.key) + '" role="checkbox" tabindex="0" aria-disabled="' + (mod.key === 'core') + '" aria-checked="' + selected + '">' +
        '<div class="module-header">' +
          '<div class="module-title">' + escapeHtml(mod.title || mod.key) + '</div>' +
          '<div class="module-check"></div>' +
        '</div>' +
        '<div class="module-desc">' + escapeHtml(mod.description || '') + '</div>' +
        '<div class="module-meta">' + tags.join('') + '</div>' +
      '</div>';
    }).join('');

    grid.querySelectorAll('.module-card').forEach(function (card) {
      const key = card.getAttribute('data-module');
      function toggle() {
        if (key === 'core') return;
        if (state.selectedModules.has(key)) state.selectedModules.delete(key);
        else state.selectedModules.add(key);
        renderWizardModules();
        const updated = grid.querySelector('[data-module="' + key + '"]');
        if (updated) updated.focus();
      }
      card.addEventListener('click', toggle);
      card.addEventListener('keydown', function (event) {
        if (event.key === ' ' || event.key === 'Enter') {
          event.preventDefault();
          toggle();
        }
      });
    });

    // 机器人型号：优先使用 robots 数组和 robot_labels
    const robotRow = $('robotSelectRow');
    const robotSelect = $('robotSelect');
    let robotOptions = [];

    if (state.robots && state.robots.length) {
      // 使用新接口返回的 robots 数组
      robotOptions = state.robots;
    } else {
      // 兼容旧逻辑
      if (state.robot) robotOptions.push(state.robot);
      if (state.recommendation && state.recommendation.robot && state.recommendation.robot !== state.robot) {
        robotOptions.push(state.recommendation.robot);
      }
    }

    if (robotOptions.length) {
      robotSelect.innerHTML = robotOptions.map(function (key) {
        const label = (state.robotLabels && state.robotLabels[key]) || key;
        return '<option value=”' + escapeHtml(key) + '”' + (key === state.robot ? ' selected' : '') + '>' + escapeHtml(label) + '</option>';
      }).join('');
      show(robotRow, true);
    } else {
      show(robotRow, false);
    }
  }

  async function generatePlan() {
    const generation = ++state.planGeneration;
    setWizardStep('plan');
    state.plan = null;

    const statusCard = $('planStatus');
    show(statusCard, true);
    setClass(statusCard, 'status-card checking');
    statusCard.querySelector('.status-icon').className = 'status-icon spinner';
    text($('planStatusText'), '正在生成安装计划…');

    show($('planDetails'), false);
    show($('planSteps'), false);
    show($('planWarnings'), false);
    show($('planBlockers'), false);
    show($('regeneratePlan'), false);
    $('startInstall').disabled = true;

    try {
      const plan = await apiCall('/api/manager/plan', {
        method: 'POST',
        body: {
          action: 'install',
          modules: Array.from(state.selectedModules),
          robot: state.robot || undefined
        }
      });
      if (generation !== state.planGeneration) return;
      state.plan = plan;
      renderPlan(plan, statusCard);
    } catch (error) {
      if (generation !== state.planGeneration) return;
      setClass(statusCard, 'status-card error');
      statusCard.querySelector('.status-icon').className = 'status-icon error';
      text($('planStatusText'), '生成计划失败：' + error.message);
      show($('regeneratePlan'), true);
    }
  }
  function renderPlan(plan, statusCard) {
    const blockers = Array.isArray(plan.blockers) ? plan.blockers : [];
    const warnings = Array.isArray(plan.warnings) ? plan.warnings : [];
    const steps = Array.isArray(plan.steps) ? plan.steps : [];

    if (blockers.length) {
      setClass(statusCard, 'status-card error');
      statusCard.querySelector('.status-icon').className = 'status-icon error';
      text($('planStatusText'), '存在阻止安装的问题，需要先处理。');
      $('startInstall').disabled = true;
    } else {
      setClass(statusCard, 'status-card success');
      statusCard.querySelector('.status-icon').className = 'status-icon check';
      text($('planStatusText'), warnings.length ? '计划已生成，请确认下面的提示。' : '计划已生成，可以开始安装。');
      $('startInstall').disabled = false;
    }

    const summary = $('planDetails');

    // 构建下载体积显示（支持 download_breakdown）
    let downloadText = '';
    if (plan.download_breakdown) {
      const bd = plan.download_breakdown;
      const parts = [];

      function formatCategory(value, name) {
        if (value === null) return name + '：暂无法预报';
        if (value === 0) return name + '：无需下载';
        return name + '：' + formatBytes(value);
      }

      if ('application' in bd) parts.push(formatCategory(bd.application, plan.source_revision ? '源码' : '应用'));
      if ('dependencies' in bd) parts.push(formatCategory(bd.dependencies, '依赖'));
      if ('models' in bd) parts.push(formatCategory(bd.models, '模型'));

      if (parts.length) {
        downloadText = parts.join('；');
        if (plan.download_bytes) {
          downloadText += '（上限 ' + formatBytes(plan.download_bytes) + '，已有缓存将复用）';
        }
      } else {
        downloadText = plan.download_bytes ? formatBytes(plan.download_bytes) : '未知';
      }
    } else {
      downloadText = plan.download_bytes ? formatBytes(plan.download_bytes) : '未知';
    }

    const items = [
      ['操作', actionText(plan.action)],
      ['目标版本', plan.version || '当前版本源提供的版本'],
      ['机器人型号', robotLabel(plan.robot || state.robot)],
      ['模块', Array.isArray(plan.modules) && plan.modules.length ? plan.modules.map(moduleLabel).join('、') : '--'],
      ['下载体积', downloadText],
      ['是否需要停机', plan.requires_stop ? '是，安装期间业务服务会停止' : '否']
    ];
    summary.innerHTML = items.map(function (item) {
      return '<div class="plan-item">' +
        '<div class="plan-item-label">' + escapeHtml(item[0]) + '</div>' +
        '<div class="plan-item-value">' + escapeHtml(item[1]) + '</div>' +
      '</div>';
    }).join('');
    show(summary, true);

    if (steps.length) {
      const stepsEl = $('planSteps');
      stepsEl.innerHTML = steps.map(function (step, index) {
        return '<div class="plan-step">' +
          '<div class="plan-step-num">' + (index + 1) + '</div>' +
          '<div class="plan-step-content">' +
            '<div class="plan-step-title">' + escapeHtml(step.title) + '</div>' +
            (step.description ? '<div class="plan-step-desc">' + escapeHtml(step.description) + '</div>' : '') +
          '</div>' +
        '</div>';
      }).join('');
      show(stepsEl, true);
    } else {
      show($('planSteps'), false);
    }

    if (warnings.length) {
      $('planWarnings').innerHTML = warnings.map(function (item) {
        return '<div class="warning-item"><span>' + escapeHtml(item) + '</span></div>';
      }).join('');
      show($('planWarnings'), true);
    } else {
      show($('planWarnings'), false);
    }

    if (blockers.length) {
      $('planBlockers').innerHTML = blockers.map(function (item) {
        return '<div class="blocker-item"><span>' + escapeHtml(item) + '</span></div>';
      }).join('');
      show($('planBlockers'), true);
      show($('regeneratePlan'), true);
    } else {
      show($('planBlockers'), false);
    }
  }

  async function submitInstall() {
    if (state.submitting || !state.plan) return;
    state.submitting = true;
    $('startInstall').disabled = true;

    let plan = state.plan;
    const params = {
      modules: Array.from(state.selectedModules),
      robot: state.robot || undefined
    };

    try {
      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        plan = await withStopConfirmation(plan, 'install', params);
        if (!plan) {
          state.submitting = false;
          $('startInstall').disabled = false;
          return;
        }
      }

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        state.plan = plan;
        renderPlan(plan, $('planStatus'));
        state.submitting = false;
        $('startInstall').disabled = false;
        return;
      }

      if (plan.requires_stop) {
        const ok = await confirmDialog({
          title: '开始安装',
          text: '安装期间业务服务会停止，正在运行的任务不会跨版本恢复。',
          items: Array.isArray(plan.warnings) && plan.warnings.length ? plan.warnings : null,
          confirmText: '开始安装',
          danger: true
        });
        if (!ok) {
          state.submitting = false;
          $('startInstall').disabled = false;
          return;
        }
      }

      state.plan = plan;
      const job = await apiCall('/api/manager/jobs', {
        method: 'POST',
        body: { plan_id: plan.id }
      });
      state.submitting = false;
      setWizardStep('install');
      initInstallStep(job);
      trackJob(job.id);
    } catch (error) {
      state.submitting = false;
      const statusCard = $('planStatus');
      setClass(statusCard, 'status-card error');
      statusCard.querySelector('.status-icon').className = 'status-icon error';
      text($('planStatusText'), '提交安装任务失败：' + error.message);
      $('startInstall').disabled = false;
    }
  }

  function initInstallStep(job) {
    text($('installSubtitle'), '安装任务已提交，正在执行…');
    renderInstallProgress(job);
    show($('installResult'), false);
    show($('cancelInstall'), canCancel(job));
    show($('finishSetup'), false);
    show($('retrySetup'), false);
    text($('installLogContent'), '正在读取任务日志…');
    $('installLog').open = false;
  }

  function renderInstallProgress(job) {
    const progress = typeof job.progress === 'number' ? Math.max(0, Math.min(100, job.progress)) : 0;
    $('installProgressBar').style.width = progress + '%';
    text($('installPercent'), progress + '%');

    // 翻译阶段名称
    const stageMap = {
      'queued': '等待中',
      'running': '执行中',
      'complete': '已完成',
      'download': '下载中',
      'extract': '解压中',
      'install': '安装中',
      'configure': '配置中'
    };
    const stageText = stageMap[job.stage] || stageLabel(job.stage);
    text($('installStage'), stageText);
    text($('installMessage'), job.message || '');

    // 根据任务状态更新标题
    if (job.status === 'succeeded') {
      text($('installTitle'), '安装完成');
      text($('installSubtitle'), '安装完成');
    } else if (['failed', 'cancelled', 'interrupted'].includes(job.status)) {
      const title = job.status === 'cancelled' ? '安装已取消' : job.status === 'interrupted' ? '安装已中断' : '安装未完成';
      text($('installTitle'), title);
      text($('installSubtitle'), job.message || title);
    } else {
      text($('installTitle'), '正在安装');
      text($('installSubtitle'), stageText);
    }

    if (Array.isArray(job.log) && job.log.length) {
      const content = $('installLogContent');
      content.textContent = formatLog(job.log);
      content.scrollTop = content.scrollHeight;
    }

    show($('cancelInstall'), canCancel(job));
  }

  function finalizeWizardInstall(job) {
    const result = $('installResult');
    show(result, true);

    if (job.status === 'succeeded') {
      setClass(result, 'result-card success');
      result.innerHTML = '<h3>安装完成</h3><p>业务版本已安装。现在可以进入管理界面或打开控制台。</p>' +
        (job.result && job.result.version
          ? '<p class="muted small">已安装版本：' + escapeHtml(job.result.version) + '</p>'
          : '');
      show($('finishSetup'), true);
      show($('retrySetup'), false);
      show($('cancelInstall'), false);
      $('installProgressBar').style.width = '100%';
      text($('installPercent'), '100%');
      text($('installStage'), '已完成');
      text($('installMessage'), job.message || '');
    } else if (job.status === 'cancelled') {
      setClass(result, 'result-card error');
      result.innerHTML =
        '<h3>安装已取消</h3>' +
        '<p>任务在安全边界停止，设备保留原状态。</p>';
      show($('finishSetup'), false);
      show($('retrySetup'), true);
      show($('cancelInstall'), false);
    } else {
      const errText = job.error
        ? (typeof job.error === 'string' ? job.error : (job.error.message || JSON.stringify(job.error)))
        : '任务未完成。';
      setClass(result, 'result-card error');
      result.innerHTML =
        '<h3>' + escapeHtml(jobStatusText(job.status)) + '</h3>' +
        '<p>' + escapeHtml(errText) + '</p>' +
        '<p class="muted small">可以查看下方日志了解实际失败位置，处理后重新生成计划再试。</p>';
      show($('finishSetup'), false);
      show($('retrySetup'), true);
      show($('cancelInstall'), false);
    }
  }

  function initWizardEvents() {
    $('retryHardware').addEventListener('click', loadHardware);
    $('nextToModules').addEventListener('click', function () {
      setWizardStep('modules');
      loadWizardModules();
    });
    $('backToHardware').addEventListener('click', function () { setWizardStep('hardware'); });
    $('nextToPlan').addEventListener('click', function () {
      if (!state.selectedModules.size) {
        confirmDialog({
          title: '未选择模块',
          text: '当前没有选择任何功能模块，安装后业务能力可能不可用。是否继续？',
          confirmText: '继续'
        }).then(function (ok) {
          if (ok) generatePlan();
        });
        return;
      }
      generatePlan();
    });
    $('backToModules').addEventListener('click', function () {
      state.planGeneration += 1;
      setWizardStep('modules');
    });
    $('regeneratePlan').addEventListener('click', generatePlan);
    $('startInstall').addEventListener('click', submitInstall);
    $('robotSelect').addEventListener('change', function (event) {
      state.robot = event.target.value;
    });
    $('cancelInstall').addEventListener('click', function () {
      if (state.installJobId) cancelJob(state.installJobId);
    });
    $('finishSetup').addEventListener('click', function () {
      showView('overview');
      loadOverview();
    });
    $('retrySetup').addEventListener('click', function () {
      state.plan = null;
      setWizardStep('plan');
      generatePlan();
    });
    $('wizardSaveSource').addEventListener('click', function () { saveSourceSettings('wizard'); });
    $('wizardClearToken').addEventListener('click', function () { clearSourceToken('wizard'); });
  }

  // ===== 功能模块（主界面）=====
  async function loadModulesView() {
    const container = $('modulesContent');
    container.innerHTML = '<p class="info-text">正在读取模块列表…</p>';
    showMessage($('modulesMessage'), '');
    $('applyModules').disabled = true;

    try {
      const data = await apiCall('/api/manager/modules');
      state.modules = Array.isArray(data.modules) ? data.modules : [];
      state.robot = data.robot || state.robot;
      state.selectedModules = new Set(
        Array.isArray(data.selected) ? data.selected
          : state.modules.filter(function (m) { return m.installed; }).map(function (m) { return m.key; })
      );
      renderModulesView(data);
    } catch (error) {
      container.innerHTML = '';
      showMessage($('modulesMessage'), '读取模块列表失败：' + error.message, 'error');
    }
  }

  function renderModulesView(data) {
    const container = $('modulesContent');
    if (!state.modules.length) {
      container.innerHTML = '<p class="info-text">当前版本源未提供可选模块。</p>';
      return;
    }

    // 构建模块key到title的映射，用于依赖/冲突显示
    const moduleTitles = {};
    state.modules.forEach(function (m) {
      moduleTitles[m.key] = m.title || m.key;
    });

    function mapKeysToTitles(keys) {
      if (!Array.isArray(keys) || !keys.length) return [];
      return keys.map(function (k) { return moduleTitles[k] || k; });
    }

    container.innerHTML = state.modules.map(function (mod) {
      const installed = Boolean(mod.installed);
      const selected = state.selectedModules.has(mod.key);
      const canSelect = data.selected != null;
      const tags = [];
      if (installed) tags.push('<span class="module-tag installed">已安装</span>');

      const requiresTitles = mapKeysToTitles(mod.requires);
      if (requiresTitles.length) tags.push('<span class="module-tag">依赖：' + escapeHtml(requiresTitles.join('、')) + '</span>');

      const conflictsTitles = mapKeysToTitles(mod.conflicts);
      if (conflictsTitles.length) tags.push('<span class="module-tag">冲突：' + escapeHtml(conflictsTitles.join('、')) + '</span>');

      return '<div class="module-list-item">' +
        '<div class="module-list-header">' +
          '<h3 class="module-list-title">' + escapeHtml(mod.title || mod.key) + '</h3>' +
          '<span class="status-badge ' + (installed ? 'good' : '') + '">' + (installed ? '已安装' : '未安装') + '</span>' +
        '</div>' +
        '<p class="module-list-desc">' + escapeHtml(mod.description || '') + '</p>' +
        '<div class="module-list-meta">' + tags.join('') + '</div>' +
        (canSelect
          ? '<div class="module-select-row"><label class="msb-check"><input type="checkbox" data-module-toggle="' + escapeHtml(mod.key) + '"' +
            (selected ? ' checked' : '') + (mod.key === 'core' ? ' disabled' : '') + '> ' +
            (mod.key === 'core' ? '基础模块始终保留' : '安装此模块') + '</label></div>'
          : '') +
      '</div>';
    }).join('');

    container.querySelectorAll('[data-module-toggle]').forEach(function (input) {
      input.addEventListener('change', function () {
        const key = input.getAttribute('data-module-toggle');
        if (input.checked) state.selectedModules.add(key);
        else state.selectedModules.delete(key);
        $('applyModules').disabled = !hasModuleChanges();
      });
    });
  }

  function hasModuleChanges() {
    const installed = new Set(
      state.modules.filter(function (m) { return m.installed; }).map(function (m) { return m.key; })
    );
    if (installed.size !== state.selectedModules.size) return true;
    for (const key of state.selectedModules) {
      if (!installed.has(key)) return true;
    }
    return false;
  }

  async function applyModuleChanges() {
    if (state.submitting) return;
    state.submitting = true;
    $('applyModules').disabled = true;
    showMessage($('modulesMessage'), '');

    const params = {
      modules: Array.from(state.selectedModules),
      robot: state.robot || undefined
    };

    try {
      let plan = await apiCall('/api/manager/plan', {
        method: 'POST',
        body: Object.assign({ action: 'modules' }, params)
      });

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        plan = await withStopConfirmation(plan, 'modules', params);
        if (!plan) {
          state.submitting = false;
          $('applyModules').disabled = false;
          return;
        }
      }

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        showMessage($('modulesMessage'), '无法应用变更：' + plan.blockers.join('；'), 'error');
        state.submitting = false;
        $('applyModules').disabled = false;
        return;
      }

      const ok = await confirmDialog({
        title: '应用模块变更',
        text: '模块变更通过安装任务执行，可能需要重新下载与切换版本。',
        items: Array.isArray(plan.warnings) ? plan.warnings : null,
        confirmText: '开始变更',
        danger: Boolean(plan.requires_stop)
      });

      if (!ok) {
        state.submitting = false;
        $('applyModules').disabled = false;
        return;
      }

      const job = await apiCall('/api/manager/jobs', {
        method: 'POST',
        body: { plan_id: plan.id }
      });
      state.submitting = false;
      showMessage($('modulesMessage'), '模块变更任务已提交，可在任务记录中查看进度。', 'good');
      trackJob(job.id);
      showView('jobs');
      loadJobsView();
    } catch (error) {
      state.submitting = false;
      $('applyModules').disabled = false;
      showMessage($('modulesMessage'), '应用变更失败：' + error.message, 'error');
    }
  }

  // ===== 更新来源与私有源凭据 =====
  // 令牌只写入、不读取回来；UI 只反映“是否已保存”。
  function renderTokenState(prefix, hasToken) {
    const el = $(prefix === 'wizard' ? 'wizardTokenState' : 'tokenState');
    if (el) text(el, hasToken ? '已保存访问令牌' : '未保存访问令牌');
  }

  function applySourceSettings(settings, prefix) {
    const channelEl = $(prefix === 'wizard' ? 'wizardChannel' : 'updateChannel');
    const feedEl = $(prefix === 'wizard' ? 'wizardFeedUrl' : 'feedUrl');
    const branchEl = $(prefix === 'wizard' ? 'wizardSourceBranch' : 'sourceBranch');
    const sourceMode = settings.source_mode || 'git';
    if (channelEl) {
      channelEl.value = settings.channel || 'stable';
      show(channelEl.closest('label'), sourceMode === 'catalog');
    }
    if (branchEl) {
      branchEl.value = settings.source_branch || 'main';
      show(branchEl.closest('label'), sourceMode === 'git');
    }
    if (feedEl) {
      feedEl.value = sourceMode === 'git' ? settings.source_repository : settings.feed_url;
      feedEl.dataset.sourceMode = sourceMode;
      text(feedEl.closest('label').querySelector('.config-label'), sourceMode === 'git' ? '源码仓库地址' : '版本目录地址');
    }
    renderTokenState(prefix, Boolean(settings.has_download_token));
    const summary = $('wizardSourceSummary');
    if (summary) {
      text(summary, settings.feed_url ? settings.feed_url + (sourceMode === 'git' ? ' · ' + (settings.source_branch || 'main') : '') : '未配置');
    }
  }

  async function saveSourceSettings(prefix) {
    const channelEl = $(prefix === 'wizard' ? 'wizardChannel' : 'updateChannel');
    const feedEl = $(prefix === 'wizard' ? 'wizardFeedUrl' : 'feedUrl');
    const tokenEl = $(prefix === 'wizard' ? 'wizardToken' : 'sourceToken');
    const messageEl = $(prefix === 'wizard' ? 'wizardSourceMessage' : 'updateConfigMessage');
    const button = $(prefix === 'wizard' ? 'wizardSaveSource' : 'saveUpdateConfig');

    const branchEl = $(prefix === 'wizard' ? 'wizardSourceBranch' : 'sourceBranch');
    const sourceMode = feedEl ? feedEl.dataset.sourceMode || 'git' : 'git';
    const channel = channelEl ? channelEl.value : 'stable';
    const feedUrl = feedEl ? feedEl.value.trim() : '';
    const token = tokenEl ? tokenEl.value : '';

    if (button) button.disabled = true;

    try {
      const settings = await apiCall('/api/manager/settings', {
        method: 'PUT',
        body: sourceMode === 'git'
          ? { source_mode: 'git', source_repository: feedUrl, source_branch: branchEl ? branchEl.value.trim() || 'main' : 'main' }
          : { source_mode: 'catalog', channel: channel, feed_url: feedUrl }
      });

      // 只有用户实际填写时才提交令牌；留空保持原值。
      if (token) {
        await apiCall('/api/manager/source-credentials', {
          method: 'POST',
          body: { token: token }
        });
      }

      const refreshed = await apiCall('/api/manager/settings');
      applySourceSettings(refreshed, prefix);
      if (tokenEl) tokenEl.value = '';

      inlineMessage(messageEl, token
        ? '来源设置与访问令牌已保存。'
        : '来源设置已保存。', 'good');

      if (prefix === 'wizard') {
        // 保存后立即反映模块与发布可见性，避免停在“无可用版本”。
        await loadWizardModules();
      } else {
        const releases = await apiCall('/api/manager/releases');
        renderReleases(releases);
      }
      return settings;
    } catch (error) {
      inlineMessage(messageEl, '保存失败：' + error.message, 'bad');
      return null;
    } finally {
      if (button) button.disabled = false;
    }
  }

  async function clearSourceToken(prefix) {
    const messageEl = $(prefix === 'wizard' ? 'wizardSourceMessage' : 'updateConfigMessage');
    const ok = await confirmDialog({
      title: '清除访问令牌',
      text: '清除后从私有源下载将不再携带凭据。已保存的令牌内容不会被读回，确认后立即失效。',
      confirmText: '清除令牌',
      danger: true
    });
    if (!ok) return;

    try {
      const result = await apiCall('/api/manager/source-credentials', {
        method: 'POST',
        body: { token: '' }
      });
      renderTokenState(prefix, Boolean(result.has_download_token));
      inlineMessage(messageEl, '访问令牌已清除。', 'good');
    } catch (error) {
      inlineMessage(messageEl, '清除失败：' + error.message, 'bad');
    }
  }

  // ===== 版本更新 =====
  async function loadUpdatesView() {
    showMessage($('releasesStatus'), '');
    const list = $('releasesList');
    list.innerHTML = '<p class="info-text">正在读取版本列表…</p>';
    renderRollback();
    renderManagerUpdate();

    try {
      const [settings, releases] = await Promise.all([
        apiCall('/api/manager/settings'),
        apiCall('/api/manager/releases?refresh=1')
      ]);
      applySourceSettings(settings, 'main');
      renderReleases(releases);
      applyStatus(await apiCall('/api/manager/status'));
      renderManagerUpdate();
    } catch (error) {
      list.innerHTML = '';
      showMessage($('releasesStatus'), '读取版本列表失败：' + error.message, 'error');
    }
  }

  // 管理程序版本独立于业务版本，先取状态确认是否确有可用版本。
  function renderManagerUpdate() {
    const current = state.status ? state.status.manager_version : null;
    const update = (state.status && state.status.update) || {};

    text($('managerCurrentVersion'), current || '--');

    const latestEl = $('managerLatestVersion');
    const noteEl = $('managerUpdateNote');
    const button = $('updateManagerButton');
    const stateEl = $('managerUpdateState');

    text(stateEl, '');

    if (update.manager_available && update.manager_version) {
      text(latestEl, '· 可更新到 ' + update.manager_version);
      text(noteEl, '更新管理程序后，管理网页会短暂断开并自动重启；机器人业务服务保持运行。');
      show(button, true);
      button.disabled = false;
      button.textContent = '更新到 ' + update.manager_version;
    } else if (update.manager_version) {
      text(latestEl, '· 已是最新');
      text(noteEl, '');
      show(button, false);
    } else {
      text(latestEl, '');
      text(noteEl, '版本源未提供管理程序发布信息，暂时无法更新管理程序。');
      show(button, false);
    }
  }

  async function startManagerUpdate() {
    if (state.submitting) return;
    const update = (state.status && state.status.update) || {};
    const version = update.manager_version;

    state.submitting = true;
    showMessage($('releasesStatus'), '');

    try {
      let plan = await apiCall('/api/manager/plan', {
        method: 'POST',
        body: { action: 'update-manager', version: version || undefined }
      });

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        showMessage($('releasesStatus'), '无法更新管理程序：' + plan.blockers.join('；'), 'error');
        state.submitting = false;
        return;
      }

      const ok = await confirmDialog({
        title: '更新管理程序',
        text: '更新期间管理网页会短暂断开并自动重启，机器人业务服务保持运行。此期间页面会显示“正在重启管理服务”。',
        items: Array.isArray(plan.warnings) && plan.warnings.length ? plan.warnings : null,
        confirmText: '开始更新',
        danger: false
      });

      if (!ok) {
        state.submitting = false;
        return;
      }

      const job = await apiCall('/api/manager/jobs', {
        method: 'POST',
        body: { plan_id: plan.id }
      });
      state.submitting = false;
      trackManagerUpdateJob(job.id);
    } catch (error) {
      state.submitting = false;
      showMessage($('releasesStatus'), '启动管理程序更新失败：' + error.message, 'error');
    }
  }

  // 管理程序更新后本进程会被替换，轮询必须容忍连接中断，并把重连作为正常阶段。
  function trackManagerUpdateJob(jobId) {
    state.jobGeneration += 1;
    const generation = state.jobGeneration;
    if (state.pollTimer) clearInterval(state.pollTimer);
    state.managerRestarting = false;

    const button = $('updateManagerButton');
    if (button) button.disabled = true;
    text($('managerUpdateState'), '正在更新管理程序…');

    function failure(message) {
      clearInterval(state.pollTimer);
      state.pollTimer = null;
      state.managerRestarting = false;
      if (button) button.disabled = false;
      text($('managerUpdateState'), '');
      showMessage($('releasesStatus'), message, 'error');
    }

    state.pollTimer = setInterval(async function () {
      if (generation !== state.jobGeneration) {
        clearInterval(state.pollTimer);
        return;
      }
      try {
        const job = await apiCall('/api/manager/jobs/' + encodeURIComponent(jobId));
        if (generation !== state.jobGeneration) return;

        state.managerRestarting = false;
        text($('managerUpdateState'), jobStatusText(job.status) + '：' + (job.message || job.stage || ''));

        if (job.status === 'succeeded') {
          clearInterval(state.pollTimer);
          state.pollTimer = null;
          if (button) button.disabled = false;
          text($('managerUpdateState'), '管理程序已更新');
          // 管理服务会按计划重启；这里按重连流程恢复，不把断线记为失败。
          await handleManagerRestart(job);
          return;
        }

        if (job.status !== 'running' && job.status !== 'queued') {
          const errText = job.error
            ? (typeof job.error === 'string' ? job.error : (job.error.message || JSON.stringify(job.error)))
            : '任务未完成。';
          failure('管理程序更新未完成：' + errText);
        }
      } catch (error) {
        if (generation !== state.jobGeneration) return;
        // 管理进程切换期间的连接失败属于预期，转入重连等待。
        if (!state.managerRestarting) {
          state.managerRestarting = true;
          text($('managerUpdateState'), '管理服务正在重启，等待重新连接…');
        }
        recoverAfterManagerRestart(generation);
      }
    }, POLL_INTERVAL_MS);
  }

  // 连接恢复：刷新会话与 CSRF，重新读取状态并保留当前页面。
  async function recoverAfterManagerRestart(generation) {
    const deadline = Date.now() + 120000;
    while (Date.now() < deadline) {
      if (generation !== state.jobGeneration) return false;
      if (state.pollTimer) {
        clearInterval(state.pollTimer);
        state.pollTimer = null;
      }
      await new Promise(function (resolve) { setTimeout(resolve, 2000); });
      if (generation !== state.jobGeneration) return false;
      try {
        await refreshSession();
        const status = await apiCall('/api/manager/status');
        if (generation !== state.jobGeneration) return false;
        applyStatus(status);
        renderOverview(status);
        if (state.currentView === 'updates') loadUpdatesView();
        if (state.currentView === 'jobs') loadJobsView();
        text($('managerUpdateState'), '管理程序已更新为 ' + (status.manager_version || '--'));
        showMessage($('releasesStatus'), '管理服务已重新连接，管理程序版本：' + (status.manager_version || '--') + '。', 'good');
        return true;
      } catch (error) {
        // 管理服务尚未就绪，继续等待。
      }
    }
    if (generation === state.jobGeneration) {
      text($('managerUpdateState'), '等待重连超时');
      showMessage($('releasesStatus'), '管理服务重启后仍未连接成功，请刷新页面确认实际状态。', 'error');
    }
    return false;
  }

  async function handleManagerRestart(job) {
    const generation = state.jobGeneration;
    text($('managerUpdateState'), '管理服务正在重启，等待重新连接…');
    await recoverAfterManagerRestart(generation);
  }

  // 回退入口只在设备确实保留上一版本时出现。
  function renderRollback() {
    const block = $('rollbackBlock');
    const previous = state.status && state.status.installation
      ? state.status.installation.previous_version
      : null;

    if (!previous) {
      show(block, false);
      return;
    }
    text($('rollbackVersion'), previous);
    show(block, true);
  }

  async function startRollback() {
    if (state.submitting) return;
    const previous = state.status && state.status.installation
      ? state.status.installation.previous_version
      : null;
    if (!previous) return;

    state.submitting = true;
    showMessage($('releasesStatus'), '');

    const params = { version: previous };

    try {
      let plan = await apiCall('/api/manager/plan', {
        method: 'POST',
        body: { action: 'rollback', version: previous }
      });

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        plan = await withStopConfirmation(plan, 'rollback', {});
        if (!plan) {
          state.submitting = false;
          return;
        }
      }

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        showMessage($('releasesStatus'), '无法回退：' + plan.blockers.join('；'), 'error');
        state.submitting = false;
        return;
      }

      const ok = await confirmDialog({
        title: '回退到 ' + previous,
        text: plan.requires_stop
          ? '回退期间业务服务会停止。回退后当前版本仍会保留，可再次更新。'
          : '确认回退到上一版本。',
        items: Array.isArray(plan.warnings) ? plan.warnings : null,
        confirmText: '开始回退',
        danger: Boolean(plan.requires_stop)
      });

      if (!ok) {
        state.submitting = false;
        return;
      }

      const job = await apiCall('/api/manager/jobs', {
        method: 'POST',
        body: { plan_id: plan.id }
      });
      state.submitting = false;
      showMessage($('releasesStatus'), '回退任务已提交，可在任务记录中查看进度。', 'good');
      trackJob(job.id);
      showView('jobs');
      loadJobsView();
    } catch (error) {
      state.submitting = false;
      showMessage($('releasesStatus'), '启动回退失败：' + error.message, 'error');
    }
  }

  function renderReleases(data) {
    const list = $('releasesList');
    const statusEl = $('releasesStatus');

    if (!data || !data.configured) {
      list.innerHTML = '';
      showMessage(statusEl, '尚未配置版本源地址。填写版本源并保存后即可读取可用版本。', 'warn');
      return;
    }

    if (data.error) {
      list.innerHTML = '';
      showMessage(statusEl, '版本源读取失败：' + data.error, 'error');
      return;
    }

    showMessage(statusEl, '');

    const releases = Array.isArray(data.releases) ? data.releases : [];
    if (!releases.length) {
      list.innerHTML = '<p class="info-text">版本源中没有可用版本。</p>';
      return;
    }

    const currentVersion = state.status && state.status.installation
      ? state.status.installation.version
      : null;

    list.innerHTML = releases.map(function (release) {
      const isCurrent = currentVersion && release.version === currentVersion;
      const compatible = release.compatible !== false;
      const badges = [];
      if (isCurrent) badges.push('<span class="status-badge info">当前版本</span>');
      if (release.channel) badges.push('<span class="status-badge">' + escapeHtml(release.channel) + '</span>');
      if (!compatible) badges.push('<span class="status-badge bad">不兼容</span>');

      const actions = [];
      if (!isCurrent && compatible) {
        actions.push('<button class="btn primary" type="button" data-release-action="update" data-version="' +
          escapeHtml(release.version) + '">更新到此版本</button>');
      }
      if (!isCurrent && !compatible) {
        actions.push('<button class="btn" type="button" data-release-action="update" data-version="' +
          escapeHtml(release.version) + '" disabled>不可安装</button>');
      }
      if (isCurrent && state.status && state.status.installation && state.status.installation.previous_version) {
        actions.push('<span class="info-text">可通过任务记录中的回退操作切换版本。</span>');
      }

      return '<div class="release-card' + (isCurrent ? ' current' : '') + '">' +
        '<div class="release-header">' +
          '<div class="release-version">' + escapeHtml(release.version) + '</div>' +
          '<div class="release-badges">' + badges.join('') + '</div>' +
        '</div>' +
        (release.notes ? '<div class="release-notes">' + escapeHtml(release.notes) + '</div>' : '') +
        '<div class="release-meta">' +
          (release.published_at ? '<span>发布时间：' + escapeHtml(formatTime(release.published_at)) + '</span>' : '') +
          (release.modules && release.modules.length ? '<span>模块：' + escapeHtml(release.modules.join('、')) + '</span>' : '') +
          (release.reason ? '<span>' + escapeHtml(release.reason) + '</span>' : '') +
        '</div>' +
        (actions.length ? '<div class="release-actions">' + actions.join('') + '</div>' : '') +
      '</div>';
    }).join('');

    list.querySelectorAll('[data-release-action="update"]').forEach(function (btn) {
      btn.addEventListener('click', function () {
        startReleaseUpdate(btn.getAttribute('data-version'));
      });
    });
  }

  async function startReleaseUpdate(version) {
    if (state.submitting) return;
    state.submitting = true;
    showMessage($('releasesStatus'), '');

    const params = { version: version };

    try {
      let plan = await apiCall('/api/manager/plan', { method: 'POST', body: { action: 'update', version: version } });

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        plan = await withStopConfirmation(plan, 'update', params);
        if (!plan) {
          state.submitting = false;
          return;
        }
      }

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        showMessage($('releasesStatus'), '无法更新：' + plan.blockers.join('；'), 'error');
        state.submitting = false;
        return;
      }

      const ok = await confirmDialog({
        title: '更新到 ' + version,
        text: plan.requires_stop
          ? '更新期间业务服务会停止。正在运行的任务不会跨版本恢复。'
          : '确认开始更新。',
        items: Array.isArray(plan.warnings) ? plan.warnings : null,
        confirmText: '开始更新',
        danger: Boolean(plan.requires_stop)
      });

      if (!ok) {
        state.submitting = false;
        return;
      }

      const job = await apiCall('/api/manager/jobs', {
        method: 'POST',
        body: { plan_id: plan.id }
      });
      state.submitting = false;
      showMessage($('releasesStatus'), '更新任务已提交，可在任务记录中查看进度。', 'good');
      trackJob(job.id);
      showView('jobs');
      loadJobsView();
    } catch (error) {
      state.submitting = false;
      showMessage($('releasesStatus'), '启动更新失败：' + error.message, 'error');
    }
  }

  // ===== 任务记录 =====
  async function loadJobsView() {
    showMessage($('jobsMessage'), '');
    const list = $('jobsList');
    list.innerHTML = '<p class="info-text">正在读取任务记录…</p>';
    try {
      const data = await apiCall('/api/manager/jobs');
      renderJobsList(Array.isArray(data.jobs) ? data.jobs : [], true);
    } catch (error) {
      list.innerHTML = '';
      showMessage($('jobsMessage'), '读取任务记录失败：' + error.message, 'error');
    }
  }

  function renderJobsList(jobs, full) {
    const list = $('jobsList');
    const items = Array.isArray(jobs) ? jobs : [];

    if (full !== false) {
      if (!items.length) {
        list.innerHTML = '<p class="info-text">暂无任务记录。</p>';
        return;
      }
      list.innerHTML = items.map(function (job) {
        return '<div class="job-card" data-job-id="' + escapeHtml(job.id) + '"></div>';
      }).join('');
      items.forEach(function (job) {
        const card = list.querySelector('[data-job-id="' + job.id + '"]');
        if (card) renderJobInto(card, job, {});
      });
      return;
    }

    // 轮询更新：只替换已有卡片内容，避免整表重绘丢失滚动位置
    items.forEach(function (job) {
      const card = list.querySelector('[data-job-id="' + job.id + '"]');
      if (card) renderJobInto(card, job, {});
      else loadJobsView();
    });
  }

  // ===== 设备配置 =====
  async function loadConfigView() {
    const sections = $('configSections');
    const groups = $('configGroups');
    sections.innerHTML = '<p class="info-text">正在读取配置…</p>';
    groups.innerHTML = '';
    showMessage($('configMessage'), '');
    $('configSave').disabled = true;

    try {
      const data = await apiCall('/api/manager/config');
      state.config = data;
      state.configRevision = data.revision != null ? data.revision : null;
      state.configValues = {};
      renderConfig(data);
    } catch (error) {
      sections.innerHTML = '';
      showMessage($('configMessage'), '读取配置失败：' + error.message, 'error');
    }
  }

  function renderConfig(data) {
    const sections = $('configSections');
    const groups = $('configGroups');
    const fields = Array.isArray(data.fields) ? data.fields : [];
    const groupList = Array.isArray(data.groups) ? data.groups : [];

    if (data.available === false) {
      sections.innerHTML = '';
      showMessage($('configMessage'), data.message || '当前没有可用的业务配置入口。', 'warn');
      return;
    }

    if (!fields.length) {
      sections.innerHTML = '<p class="info-text">当前版本没有可编辑的配置项。</p>';
      if (data.message) showMessage($('configMessage'), data.message, null);
      return;
    }

    // 以后端声明的分组顺序为主，未声明的分组追加在后。
    const known = new Map();
    groupList.forEach(function (group) {
      known.set(group.key, group.title || group.key);
    });

    const buckets = new Map();
    fields.forEach(function (field) {
      const groupKey = field.group || field.section_id || 'general';
      if (!buckets.has(groupKey)) buckets.set(groupKey, []);
      buckets.get(groupKey).push(field);
    });

    const groupEntries = Array.from(buckets.keys()).map(function (key) {
      return { key: key, title: known.get(key) || key };
    });

    groups.innerHTML = groupEntries.map(function (group, index) {
      return '<button type="button" data-config-group="' + escapeHtml(group.key) + '"' +
        (index === 0 ? ' class="active"' : '') + '>' + escapeHtml(group.title) + '</button>';
    }).join('');

    sections.innerHTML = groupEntries.map(function (group) {
      const blocks = buckets.get(group.key).map(function (field) {
        return renderConfigField(field);
      }).join('');
      return '<div class="config-block" id="cfg-' + slug(group.key) + '">' +
        '<div class="config-block-title">' + escapeHtml(group.title) + '</div>' +
        '<div class="config-fields">' + blocks + '</div>' +
      '</div>';
    }).join('');

    groups.querySelectorAll('[data-config-group]').forEach(function (btn) {
      btn.addEventListener('click', function () {
        groups.querySelectorAll('button').forEach(function (b) { b.classList.remove('active'); });
        btn.classList.add('active');
        const target = document.getElementById('cfg-' + slug(btn.getAttribute('data-config-group')));
        if (target) target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      });
    });

    sections.querySelectorAll('[data-config-key]').forEach(function (input) {
      if (input.readOnly || input.disabled) return;
      const handler = function () {
        const key = input.getAttribute('data-config-key');
        const field = fields.find(function (f) { return f.key === key; });
        let value;
        if (input.type === 'checkbox') value = input.checked ? 'true' : 'false';
        else value = input.value;
        state.configValues[key] = value;
        const wrapper = input.closest('.config-field');
        if (wrapper) {
          wrapper.classList.toggle('changed', !valuesEqual(field, value));
          const warning = wrapper.querySelector('.config-field-warn');
          if (warning) {
            const detail = field.choice_warnings && field.choice_warnings[value];
            const reason = detail && typeof detail === 'object' ? detail.reason : detail;
            text(warning, reason ? '前置条件：' + reason : '');
            show(warning, Boolean(reason));
          }
        }
        $('configSave').disabled = Object.keys(state.configValues).length === 0;
      };
      input.addEventListener(input.type === 'checkbox' ? 'change' : 'input', handler);
    });

    if (data.revision != null) {
      showMessage($('configMessage'), '已读取配置（修订 ' + data.revision + '）。', null);
    }
    if (data.message) {
      showMessage($('configMessage'), data.message, null);
    }
  }

  function slug(value) {
    return String(value).replace(/[^\w一-龥]+/g, '-');
  }

  function valuesEqual(field, value) {
    const original = field.value;
    if (field.type === 'bool') {
      return String(original).toLowerCase() === String(value).toLowerCase();
    }
    if (original == null) return value === '';
    return String(original) === String(value);
  }

  function renderConfigField(field) {
    const type = (field.type || 'str').toLowerCase();
    const key = field.key;
    const rawValue = field.value == null ? '' : field.value;
    const isMasked = Boolean(field.sensitive) && rawValue !== '' && /^\*+$/.test(String(rawValue));
    const value = rawValue;
    const chips = [];
    if (field.pending_restart) chips.push('<span class="config-chip restart">重启后生效</span>');
    else if (field.apply_mode === 'hot' || field.apply_scope === 'hot') chips.push('<span class="config-chip">热加载</span>');
    if (field.advanced) chips.push('<span class="config-chip adv">高级</span>');
    if (field.sensitive) chips.push('<span class="config-chip secret">敏感</span>');
    if (field.source_label) chips.push('<span class="config-chip">' + escapeHtml(field.source_label) + '</span>');

    const common = 'data-config-key="' + escapeHtml(key) + '" id="cfg-input-' + escapeHtml(key) + '"';
    let control;

    if (type === 'bool') {
      const checked = String(value).toLowerCase() === 'true';
      control = '<label class="msb-check"><input type="checkbox" ' + common + (checked ? ' checked' : '') + '> 启用</label>';
    } else if (Array.isArray(field.choices) && field.choices.length) {
      control = '<select class="field" ' + common + '>' + field.choices.map(function (opt, index) {
        const optLabel = (Array.isArray(field.choice_labels) && field.choice_labels[index]) || opt;
        const disabledReason = field.choice_disabled_reasons && field.choice_disabled_reasons[opt];
        return '<option value="' + escapeHtml(opt) + '"' +
          (String(opt) === String(value) ? ' selected' : '') +
          (disabledReason ? ' disabled title="' + escapeHtml(disabledReason) + '"' : '') + '>' +
          escapeHtml(optLabel) + (disabledReason ? '（不可用）' : '') + '</option>';
      }).join('') + '</select>';
    } else if (type === 'csv' || (field.options_schema && typeof field.options_schema === 'object') ||
               field.key === 'NUWAX_GEOMETRY_PGO_OPTIONS' || field.key === 'NUWAX_GEOMETRY_POINTLIO_OPTIONS') {
      control = '<textarea ' + common + '>' + escapeHtml(value) + '</textarea>';
    } else if (type === 'int') {
      control = '<input class="field" type="number" step="1" ' + common + ' value="' + escapeHtml(value) + '">';
    } else if (type === 'float') {
      control = '<input class="field" type="number" step="any" ' + common + ' value="' + escapeHtml(value) + '">';
    } else if (field.sensitive) {
      control = '<input class="field" type="password" ' + common + ' value="" autocomplete="new-password" placeholder="' +
        (isMasked ? '已设置，留空保持不变' : '未设置') + '">';
    } else {
      control = '<input class="field" type="text" ' + common + ' value="' + escapeHtml(value) + '">';
    }

    const warning = field.choice_warnings && value && field.choice_warnings[value];
    const warningText = warning && typeof warning === 'object' ? warning.reason : warning;
    const readonly = field.readonly ? ' readonly' : '';

    return '<div class="config-field"' + (field.readonly ? ' data-readonly="true"' : '') + '>' +
      '<label class="config-field-name" for="cfg-input-' + escapeHtml(key) + '">' + escapeHtml(field.label || field.zh_name || key) + '</label>' +
      '<span class="config-field-key">' + escapeHtml(key) + '</span>' +
      (field.description ? '<span class="config-field-desc">' + escapeHtml(field.description) + '</span>' : '') +
      control +
      '<span class="config-field-warn"' + (warningText ? '' : ' hidden') + '>' +
        (warningText ? '前置条件：' + escapeHtml(warningText) : '') + '</span>' +
      (chips.length ? '<div class="config-field-foot">' + chips.join('') + '</div>' : '') +
    '</div>';
  }

  async function saveConfig() {
    const keys = Object.keys(state.configValues);
    if (!keys.length) return;

    const fields = state.config ? (state.config.fields || []) : [];
    const values = {};
    const skippedSecrets = [];

    keys.forEach(function (key) {
      const field = fields.find(function (f) { return f.key === key; });
      const value = state.configValues[key];
      if (field && field.sensitive) {
        // 敏感项留空表示保持原值，绝不把掩码写回。
        if (value === '' || value == null || /^\*+$/.test(String(value))) {
          skippedSecrets.push(field.label || field.zh_name || key);
          return;
        }
      }
      values[key] = value;
    });

    if (!Object.keys(values).length) {
      showMessage($('configMessage'), '没有需要保存的改动。', null);
      return;
    }

    const body = { values: values };
    if (state.configRevision != null) body.revision = state.configRevision;

    $('configSave').disabled = true;

    try {
      const result = await apiCall('/api/manager/config', { method: 'PUT', body: body });
      const parts = [];
      if (result.message) parts.push(result.message);
      if (Array.isArray(result.saved) && result.saved.length) {
        parts.push('已保存 ' + result.saved.length + ' 项。');
      }
      if (result.restart_required) {
        parts.push('这些配置在下次业务启动时生效，可在概览中重启服务。');
      }
      if (skippedSecrets.length) {
        parts.push('保持原值的敏感项：' + skippedSecrets.join('、') + '。');
      }
      showMessage($('configMessage'), parts.join(' ') || '配置已保存。', 'good');

      state.configValues = {};
      const refreshed = await apiCall('/api/manager/config');
      state.config = refreshed;
      if (refreshed.revision != null) state.configRevision = refreshed.revision;
      renderConfig(refreshed);
      $('configSave').disabled = true;
    } catch (error) {
      const rejected = error.details && error.details.rejected;
      let extra = '';
      if (rejected && typeof rejected === 'object') {
        extra = '未通过校验的项：' + Object.keys(rejected).join('、') + '。';
      }
      showMessage($('configMessage'), '保存失败：' + error.message + ' ' + extra + '改动未生效。', 'error');
      $('configSave').disabled = false;
    }
  }

  // ===== 设置 =====
  async function loadSettingsView() {
    showMessage($('settingsMessage'), '');
    try {
      const [settings, status] = await Promise.all([
        apiCall('/api/manager/settings'),
        apiCall('/api/manager/status')
      ]);

      $('autoStart').checked = Boolean(settings.auto_start);
      state.passwordEnabled = Boolean(settings.password_enabled);
      updatePasswordControls();

      // 设备信息
      // 设备信息复用同一套中文描述与单位换算，避免键名和原始布尔量直出。
      const hardwareGrid2 = $('settingsHardware');
      if (state.hardware) {
        hardwareGrid2.innerHTML = hardwareSummaryRows(state.hardware).map(function (row) {
          return '<div class="hardware-item">' +
            '<div class="hardware-item-label">' + escapeHtml(row[0]) + '</div>' +
            '<div class="hardware-item-value">' + escapeHtml(row[1]) + '</div>' +
          '</div>';
        }).join('');
      } else {
        try {
          const hw = await apiCall('/api/manager/hardware');
          state.hardware = hw.hardware || {};
          hardwareGrid2.innerHTML = hardwareSummaryRows(state.hardware).map(function (row) {
            return '<div class="hardware-item">' +
              '<div class="hardware-item-label">' + escapeHtml(row[0]) + '</div>' +
              '<div class="hardware-item-value">' + escapeHtml(row[1]) + '</div>' +
            '</div>';
          }).join('');
        } catch (error) {
          hardwareGrid2.innerHTML = '<p class="info-text">设备信息读取失败：' + escapeHtml(error.message) + '</p>';
        }
      }
    } catch (error) {
      showMessage($('settingsMessage'), '读取设置失败：' + error.message, 'error');
    }
  }

  // 设置页与向导共用同一份硬件摘要，保证两处描述一致。
  function hardwareSummaryRows(hardware) {
    const osText = hardware.os_id && hardware.os_id !== 'unknown'
      ? hardware.os_id + (hardware.version_id && hardware.version_id !== 'unknown' ? ' ' + hardware.version_id : '')
      : '未知';
    let gpuText;
    if (hardware.nvidia_gpu === true) {
      gpuText = hardware.is_jetson === true ? '已检测到（Jetson 集成）' : '已检测到';
    } else if (hardware.nvidia_gpu === false) {
      gpuText = '未检测到';
    } else {
      gpuText = '未知';
    }
    if (hardware.gpu_memory_bytes) gpuText += '，显存 ' + formatBytes(hardware.gpu_memory_bytes);
    return [
      ['操作系统', osText],
      ['架构', hardware.architecture && hardware.architecture !== 'unknown' ? hardware.architecture : '未知'],
      ['CPU 核心', hardware.cpu_count != null ? hardware.cpu_count + ' 核' : '未知'],
      ['可用内存', capacityText(hardware.memory_bytes)],
      ['可用磁盘', capacityText(hardware.free_disk_bytes)],
      ['图形设备', gpuText]
    ];
  }

  function updatePasswordControls() {
    text($('passwordStatusText'), state.passwordEnabled ? '已启用' : '未启用');
    show($('setPassword'), !state.passwordEnabled);
    show($('changePassword'), state.passwordEnabled);
    show($('removePassword'), state.passwordEnabled);
  }

  function openPasswordDialog(mode) {
    const dialog = $('passwordDialog');
    const form = $('passwordForm');
    const errorEl = $('passwordError');

    text($('passwordDialogTitle'), mode === 'set' ? '设置密码' : '修改密码');
    text($('passwordDialogDesc'), mode === 'set'
      ? '设置密码后，修改配置与执行任务需要先验证密码。'
      : '请输入当前密码与新密码。');
    show($('currentPasswordField'), mode === 'change');

    form.reset();
    show(errorEl, false);
    openModal(dialog);

    form.onsubmit = async function (event) {
      event.preventDefault();
      const current = $('currentPasswordInput').value;
      const next = $('newPasswordInput').value;
      const confirmValue = $('confirmPasswordInput').value;

      if (!next) {
        text(errorEl, '新密码不能为空。');
        show(errorEl, true);
        return;
      }
      if (next !== confirmValue) {
        text(errorEl, '两次输入的新密码不一致。');
        show(errorEl, true);
        return;
      }

      const submitBtn = $('submitPassword');
      submitBtn.disabled = true;

      try {
        const body = { password: next };
        if (mode === 'change' && current) body.current_password = current;
        await apiCall('/api/manager/password', { method: 'POST', body: body });
        state.passwordEnabled = true;
        updatePasswordControls();
        dialog.close();
        showMessage($('settingsMessage'), '密码已保存。', 'good');
      } catch (error) {
        text(errorEl, error.message);
        show(errorEl, true);
      } finally {
        submitBtn.disabled = false;
      }
    };
  }

  async function removePassword() {
    const ok = await confirmDialog({
      title: '移除密码',
      text: '移除后本机访问不再需要密码。请确认设备处于可信网络内。',
      confirmText: '移除密码',
      danger: true
    });
    if (!ok) return;

    try {
      await apiCall('/api/manager/password', { method: 'POST', body: { password: '' } });
      state.passwordEnabled = false;
      updatePasswordControls();
      showMessage($('settingsMessage'), '密码已移除。', 'good');
    } catch (error) {
      showMessage($('settingsMessage'), '移除密码失败：' + error.message, 'error');
    }
  }

  // ===== 业务服务启停 =====
  async function runtimeAction(action) {
    if (state.submitting) return;
    state.submitting = true;

    const labels = { start: '启动服务', stop: '停止服务', restart: '重启服务' };

    try {
      let plan = await apiCall('/api/manager/plan', { method: 'POST', body: { action: action } });

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        plan = await withStopConfirmation(plan, action, {});
        if (!plan) {
          state.submitting = false;
          return;
        }
      }

      if (Array.isArray(plan.blockers) && plan.blockers.length) {
        showMessage($('overviewLoadError'), '无法' + labels[action] + '：' + plan.blockers.join('；'), 'error');
        state.submitting = false;
        return;
      }

      const ok = await confirmDialog({
        title: labels[action],
        text: action === 'start'
          ? '确认启动业务服务。'
          : '该操作会中断当前任务并停止相关进程。',
        items: Array.isArray(plan.warnings) ? plan.warnings : null,
        confirmText: labels[action],
        danger: action !== 'start'
      });

      if (!ok) {
        state.submitting = false;
        return;
      }

      const job = await apiCall('/api/manager/jobs', {
        method: 'POST',
        body: { plan_id: plan.id }
      });
      state.submitting = false;
      trackJob(job.id);
      loadOverview();
    } catch (error) {
      state.submitting = false;
      showMessage($('overviewLoadError'), labels[action] + '失败：' + error.message, 'error');
    }
  }

  // ===== 更新检查 =====
  async function checkUpdates() {
    try {
      await apiCall('/api/manager/releases?refresh=1');
      await loadOverview();
    } catch (error) {
      showMessage($('overviewLoadError'), '检查更新失败：' + error.message, 'error');
    }
  }

  // ===== 事件绑定 =====
  function initEvents() {
    $('startRuntime').addEventListener('click', function () { runtimeAction('start'); });
    $('stopRuntime').addEventListener('click', function () { runtimeAction('stop'); });
    $('restartRuntime').addEventListener('click', function () { runtimeAction('restart'); });

    $('checkUpdates').addEventListener('click', checkUpdates);
    $('checkUpdatesAlt').addEventListener('click', checkUpdates);
    $('retryUpdateCheck').addEventListener('click', checkUpdates);
    $('goToUpdates').addEventListener('click', function () {
      showView('updates');
      loadUpdatesView();
    });

    $('refreshModules').addEventListener('click', loadModulesView);
    $('applyModules').addEventListener('click', applyModuleChanges);
    $('refreshReleases').addEventListener('click', loadUpdatesView);
    $('rollbackButton').addEventListener('click', startRollback);
    $('refreshJobs').addEventListener('click', loadJobsView);
    $('configReload').addEventListener('click', loadConfigView);
    $('configSave').addEventListener('click', saveConfig);

    $('saveUpdateConfig').addEventListener('click', function () { saveSourceSettings('main'); });
    $('clearSourceToken').addEventListener('click', function () { clearSourceToken('main'); });
    $('updateManagerButton').addEventListener('click', startManagerUpdate);

    $('autoStart').addEventListener('change', async function (event) {
      const target = event.target;
      try {
        await apiCall('/api/manager/settings', {
          method: 'PUT',
          body: { auto_start: target.checked }
        });
        showMessage($('settingsMessage'), '启动行为已保存。', 'good');
      } catch (error) {
        target.checked = !target.checked;
        showMessage($('settingsMessage'), '保存失败：' + error.message, 'error');
      }
    });

    $('setPassword').addEventListener('click', function () { openPasswordDialog('set'); });
    $('changePassword').addEventListener('click', function () { openPasswordDialog('change'); });
    $('removePassword').addEventListener('click', removePassword);
    $('cancelPassword').addEventListener('click', function () { $('passwordDialog').close(); });
    $('cancelLogin').addEventListener('click', cancelLogin);

    window.addEventListener('hashchange', function () {
      const hash = window.location.hash.replace('#', '');
      if (hash && $('mainView').classList.contains('active')) showView(hash);
    });
  }

  // ===== 初始化 =====
  let bound = false;

  // 事件绑定只执行一次；重试连接只重跑状态加载，避免重复监听。
  function bindOnce() {
    if (bound) return;
    bound = true;
    initTheme();
    initNavigation();
    initEvents();
    initWizardEvents();

    const retry = $('bootRetry');
    if (retry) {
      retry.addEventListener('click', function () { boot(); });
    }
  }

  async function boot() {
    // 首屏只显示加载态；设备状态读回前不渲染任何概览内容。
    showBoot('正在读取设备状态…', false);

    let session;
    try {
      session = await refreshSession();
    } catch (error) {
      showBoot('无法连接到管理服务：' + error.message, true);
      return;
    }
    if (session.instance_id) state.instanceId = session.instance_id;

    // 管理程序更新后本进程会被替换，若页面打开时正处于重启窗口，
    // 先等待连接恢复再进入正式状态读取，不把断线当成失败。
    try {
      await ensureManagerReachable();
    } catch (error) {
      showBoot('管理服务暂时不可用：' + error.message + '。更新管理程序期间会短暂断开，请稍候重试。', true);
      return;
    }

    try {
      const [status, metadata] = await Promise.all([apiCall('/api/manager/status'), apiCall('/api/manager/modules')]);
      state.modules = Array.isArray(metadata.modules) ? metadata.modules : [];
      state.robotLabels = metadata.robot_labels || {};
      state.robots = Array.isArray(metadata.robots) ? metadata.robots : [];
      applyStatus(status);

      if (!status.installation || !status.installation.installed) {
        // 重新打开页面或从命令行开始安装时，直接接回持久任务的进度。
        if (status.active_job && ['install', 'migrate'].includes(status.active_job.action)) {
          showSetup();
          setWizardStep('install');
          initInstallStep(status.active_job);
        } else {
          startSetupWizard();
        }
      } else {
        const hash = window.location.hash.replace('#', '');
        const target = hash && document.getElementById(hash + 'Pane') ? hash : 'overview';
        showView(target);
        renderOverview(status);
        if (target === 'modules') loadModulesView();
        else if (target === 'updates') loadUpdatesView();
        else if (target === 'jobs') loadJobsView();
        else if (target === 'config') loadConfigView();
        else if (target === 'settings') loadSettingsView();
      }

      // 页面重新打开时恢复进行中的任务
      if (status.active_job && status.active_job.status && status.active_job.id) {
        if (status.active_job.action === 'update-manager') {
          trackManagerUpdateJob(status.active_job.id);
        } else {
          trackJob(status.active_job.id);
        }
      }
    } catch (error) {
      showBoot('读取设备状态失败：' + error.message, true);
    }
  }

  function init() {
    bindOnce();
    boot();
  }

  // 管理服务可能在更新后重启；这里最多等待一次短窗口，避免首屏直接报错。
  async function ensureManagerReachable() {
    const deadline = Date.now() + 30000;
    let lastError = null;
    while (true) {
      try {
        await apiCall('/api/manager/status');
        return true;
      } catch (error) {
        lastError = error;
        if (Date.now() >= deadline) throw lastError;
        await new Promise(function (resolve) { setTimeout(resolve, 2000); });
        try {
          await refreshSession();
        } catch (ignored) {
          // 管理服务尚未就绪，继续等待。
        }
      }
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
