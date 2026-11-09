/* 软件更新统一交给设备管理程序，页面只读取状态和提交操作。 */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  let snapshot = null;
  let pending = false;
  let polling = false;
  let settingsDirty = false;
  let trackedJob = null;
  const short = value => value ? value.slice(0, 12) : '未知';
  async function manager(path, method = 'GET', body, timeout = 15000) {
    const headers = {};
    if (method !== 'GET') {
      // 写请求前读取当前会话的校验值。
      headers['Content-Type'] = 'application/json';
      headers['X-NEH-CSRF'] = (await manager('session')).csrf_token;
    }
    const response = await fetch('/api/manager/' + path, {
      method, headers, body: body === undefined ? undefined : JSON.stringify(body),
      signal: AbortSignal.timeout(timeout), cache: 'no-store',
    });
    const result = await response.json();
    if (response.status === 401) throw new Error('请先在设备管理页登录');
    if (!response.ok) {
      const details = Array.isArray(result.error?.details) ? '：' + result.error.details.join('；') : '';
      throw new Error((result.error?.message || '设备管理程序请求失败') + details);
    }
    return result;
  }
  async function managerSnapshot() {
    const [status, settings] = await Promise.all([manager('status'), manager('settings')]);
    const job = !status.active_job && trackedJob ? await manager('jobs/' + encodeURIComponent(trackedJob)) : null;
    return {...status, settings, job};
  }
  async function managerAction(path, body) {
    if (path === '/check') {
      await manager('releases?refresh=1', 'GET', undefined, 120000);
    } else if (path === '/install') {
      const job = await manager('update', 'POST', {}, 120000);
      if (job.status === 'deferred') {
        const data = await managerSnapshot();
        data.message = job.message;
        return data;
      }
      trackedJob = job.id;
    } else if (path === '/settings') {
      await manager('settings', 'PUT', {
        auto_update: body.auto_install ? 'idle' : (body.auto_check ? 'notify' : 'off'),
        check_interval_minutes: body.interval_minutes,
      });
    }
    return managerSnapshot();
  }
  function renderControls() {
    const busy = pending || !!snapshot?.active_job;
    $('updateCheck').disabled = busy;
    $('updateInstall').disabled = busy || !snapshot?.update.available;
    $('updateSave').disabled = busy;
    for (const id of ['updateAutoCheck', 'updateAutoInstall', 'updateInterval']) $(id).disabled = busy;
    $('updateInstall').textContent = snapshot?.active_job ? '正在更新…' : '更新并重启';
  }
  function render(data) {
    if (snapshot && (data.manager_version !== snapshot.manager_version || data.installation.version !== snapshot.installation.version)
        || (data.job || data.active_job)?.id === trackedJob && (data.job || data.active_job).status === 'succeeded') {
      location.reload();
    }
    snapshot = data;
    const update = data.update;
    const job = data.job;
    const activeJob = data.active_job;
    $('updateNewBadge').hidden = !update.available;
    $('updateVersion').textContent = '运行版本 ' + short(data.installation.revision || data.installation.version) + ' · 最新版本 ' + short(update.revision || update.version);
    $('updateCheckedAt').textContent = update.check.checked_at ? '上次检查：' + new Date(update.check.checked_at).toLocaleString() : '尚未检查远端版本';
    $('updateMessage').textContent = activeJob
      ? '正在更新：' + activeJob.message + '（' + activeJob.progress + '%）'
      : (data.message || (job ? (job.status === 'succeeded' ? '更新完成' : job.message) : (update.error || update.message || (update.available ? '发现新版本，可以更新' : (update.check.checked_at ? '当前已是检测到的最新版本' : '点击检查更新，或等待自动检测')))));
    $('updateMessage').dataset.state = activeJob ? activeJob.status : (job?.status || (update.error ? 'failed' : 'idle'));
    if (!activeJob && !job && !data.message && update.available && update.last_failure) {
      $('updateMessage').textContent = '上次更新到此版本失败：' + update.last_failure.message;
      $('updateMessage').dataset.state = 'failed';
    }
    $('updateUnavailable').textContent = '';
    renderControls();
    if (!settingsDirty) {
      $('updateAutoCheck').checked = data.settings.auto_update !== 'off';
      $('updateAutoInstall').checked = data.settings.auto_update === 'idle';
      $('updateInterval').value = data.settings.check_interval_minutes;
    }
  }
  async function refresh() {
    if (polling || pending) return;
    polling = true;
    try { render(await managerSnapshot()); }
    catch (error) { $('updateMessage').textContent = error.message; }
    finally { polling = false; }
  }
  async function act(path, body = {}) {
    if (pending) return;
    pending = true;
    renderControls();
    try {
      const data = await managerAction(path, body);
      if (path === '/settings') settingsDirty = false;
      render(data);
    } catch (error) {
      $('updateMessage').textContent = error.message;
    } finally {
      pending = false;
      renderControls();
    }
  }
  $('updateCheck').addEventListener('click', () => act('/check'));
  $('updateInstall').addEventListener('click', () => {
    if (!snapshot?.update.available || pending) return;
    $('updateConfirmVersion').textContent = short(snapshot.update.revision || snapshot.update.version);
    $('updateDialog').showModal();
  });
  $('updateConfirm').addEventListener('click', () => {
    $('updateDialog').close();
    act('/install');
  });
  $('updateCancel').addEventListener('click', () => $('updateDialog').close());
  for (const id of ['updateAutoCheck', 'updateAutoInstall', 'updateInterval']) $(id).addEventListener('change', () => {settingsDirty = true;});
  $('updateSave').addEventListener('click', () => {
    const interval = Number($('updateInterval').value);
    if (!Number.isInteger(interval) || interval < 15 || interval > 1440) {
      $('updateMessage').textContent = '检查间隔必须为15—1440分钟'; return;
    }
    act('/settings', {auto_check: $('updateAutoCheck').checked, auto_install: $('updateAutoInstall').checked, interval_minutes: interval});
  });
  $('updateNewBadge').addEventListener('click', () => {
    $('viewSwitchConfig').click();
    $('updatePanel').scrollIntoView({behavior: 'smooth', block: 'center'});
    $('updateCheck').focus();
  });
  document.addEventListener('visibilitychange', () => {if (!document.hidden) refresh();});
  refresh();
  setInterval(() => {if (!document.hidden) refresh();}, 5000);
})();
