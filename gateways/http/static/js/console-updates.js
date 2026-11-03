/* 更新状态与后台事务分离：页面关闭不取消更新，重连后读取持久回执。
 * 经设备管理程序打开时（一键托管安装），检查、更新与设置直接使用管理程序接口；
 * 否则使用网关的源码安装更新器。 */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const busyStates = new Set(['queued', 'fetching', 'stopping', 'building', 'starting', 'verifying', 'rolling_back']);
  let snapshot = null;
  let pending = false;
  let polling = false;
  let accepted = false;
  let settingsDirty = false;
  let confirmedRevision = null;
  let managed = false;
  let trackedJob = null;
  const short = value => value ? value.slice(0, 12) : '未知';
  async function request(path = '', body) {
    const response = await fetch('/api/updates' + path, {
      ...(body === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)}),
      signal: AbortSignal.timeout(15000), cache: 'no-store',
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error?.message || '更新请求失败');
    return result;
  }
  async function manager(path, method = 'GET', body, timeout = 15000) {
    const headers = {};
    if (method !== 'GET') {
      // 写请求前读取当前会话的校验值，页面长时间打开后也不使用过期值。
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
    const update = status.update || {};
    let transaction = {};
    if (status.active_job) {
      trackedJob = status.active_job.id;
      transaction = {state: 'queued', message: '设备管理程序正在执行：' + (status.active_job.message || status.active_job.stage) + '（' + status.active_job.progress + '%）'};
    } else if (trackedJob) {
      const job = await manager('jobs/' + encodeURIComponent(trackedJob));
      transaction = {state: job.status === 'succeeded' ? 'succeeded' : 'failed', message: (job.status === 'succeeded' ? '更新完成：' : '更新未完成：') + job.message};
    }
    return {
      available: !!update.available,
      running_revision: status.installation?.revision || status.installation?.version,
      latest_revision: update.revision || update.version,
      checked_at: update.check?.checked_at ? Date.parse(update.check.checked_at) / 1000 : null,
      check_error: update.error ? {message: update.error} : null,
      message: update.message || (update.failed ? '该版本上次更新失败，可在设备管理页查看原因后重试' : ''),
      transaction,
      install_available: true,
      install_unavailable_reason: '',
      settings: {auto_check: settings.auto_update !== 'off', auto_install: settings.auto_update === 'idle', interval_minutes: settings.check_interval_minutes},
    };
  }
  async function managerAction(path, body) {
    if (path === '/check') {
      await manager('releases?refresh=1', 'GET', undefined, 120000);
    } else if (path === '/install') {
      const job = await manager('update', 'POST', {});
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
  function render(data) {
    snapshot = data;
    const tx = data.transaction || {};
    const busy = busyStates.has(tx.state);
    const locked = busy || tx.state === 'recovery_required';
    accepted = busy;
    $('updateNewBadge').hidden = !data.available;
    $('updateVersion').textContent = '运行版本 ' + short(data.running_revision) + ' · 最新版本 ' + short(data.latest_revision);
    $('updateCheckedAt').textContent = data.checked_at ? '上次检查：' + new Date(data.checked_at * 1000).toLocaleString() : '尚未检查远端版本';
    $('updateMessage').textContent = busy || ['failed', 'rolled_back', 'recovery_required', 'succeeded'].includes(tx.state)
      ? tx.message : (data.check_error?.message || data.message || (data.available ? '发现新版本，可以更新' : (data.checked_at ? '当前已是检测到的最新版本' : '点击检查更新，或等待自动检测')));
    $('updateMessage').dataset.state = tx.state || data.state || 'idle';
    $('updateUnavailable').textContent = data.install_unavailable_reason || '';
    $('updateCheck').disabled = pending || busy || data.managed;
    $('updateInstall').disabled = pending || locked || !data.available || !data.install_available;
    $('updateSave').disabled = pending || busy || data.managed;
    for (const id of ['updateAutoCheck', 'updateAutoInstall', 'updateInterval']) $(id).disabled = pending || busy || !!data.managed;
    $('updateInstall').textContent = busy ? '正在更新…' : '更新并重启';
    if (!settingsDirty && data.settings) {
      $('updateAutoCheck').checked = !!data.settings.auto_check;
      $('updateAutoInstall').checked = !!data.settings.auto_install;
      $('updateInterval').value = data.settings.interval_minutes;
    }
  }
  async function refresh() {
    if (polling || pending) return;
    polling = true;
    try { render(managed ? await managerSnapshot() : await request()); }
    catch (error) {
      $('updateMessage').textContent = accepted ? '更新期间服务暂时离线，正在等待重连；请勿重复操作' : '无法读取更新状态：' + error.message;
    } finally { polling = false; }
  }
  async function act(path, body = {}) {
    if (pending) return;
    pending = true;
    if (snapshot) render(snapshot);
    try {
      const data = managed ? await managerAction(path, body) : await request(path, body);
      if (path === '/settings') settingsDirty = false;
      render(data);
    } catch (error) {
      $('updateMessage').textContent = error.message + (path === '/install' ? '；请求可能已受理，正在重新查询状态' : '');
      if (path === '/install') accepted = true;
    } finally {
      pending = false;
      if (snapshot) {
        const busy = busyStates.has(snapshot.transaction?.state);
        $('updateCheck').disabled = busy;
        $('updateInstall').disabled = busy || snapshot.transaction?.state === 'recovery_required' || !snapshot.available || !snapshot.install_available;
        $('updateSave').disabled = busy;
        for (const id of ['updateAutoCheck', 'updateAutoInstall', 'updateInterval']) $(id).disabled = busy;
      }
    }
  }
  $('updateCheck').addEventListener('click', () => act('/check'));
  $('updateInstall').addEventListener('click', () => {
    if (!snapshot?.available || pending) return;
    confirmedRevision = snapshot.latest_revision;
    $('updateConfirmVersion').textContent = short(confirmedRevision);
    $('updateDialog').showModal();
  });
  $('updateConfirm').addEventListener('click', () => {
    $('updateDialog').close();
    const revision = confirmedRevision;
    confirmedRevision = null;
    act('/install', {revision});
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
  // 控制台经设备管理程序同源打开时，/api/manager/session 由管理程序应答。
  fetch('/api/manager/session', {cache: 'no-store', signal: AbortSignal.timeout(5000)})
    .then(response => response.ok ? response.json() : null)
    .then(session => {managed = !!session?.manager_version;}, () => {})
    .finally(() => {
      refresh();
      setInterval(() => {if (!document.hidden) refresh();}, 5000);
    });
})();
