/* 更新状态与后台事务分离：页面关闭不取消更新，重连后读取持久回执。 */
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
    $('updateCheck').disabled = pending || busy;
    $('updateInstall').disabled = pending || locked || !data.available || !data.install_available;
    $('updateSave').disabled = pending || busy;
    for (const id of ['updateAutoCheck', 'updateAutoInstall', 'updateInterval']) $(id).disabled = pending || busy;
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
    try { render(await request()); }
    catch (error) {
      $('updateMessage').textContent = accepted ? '更新期间服务暂时离线，正在等待重连；请勿重复操作' : '无法读取更新状态：' + error.message;
    } finally { polling = false; }
  }
  async function act(path, body = {}) {
    if (pending) return;
    pending = true;
    if (snapshot) render(snapshot);
    try {
      const data = await request(path, body);
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
  refresh();
  setInterval(() => {if (!document.hidden) refresh();}, 5000);
})();
