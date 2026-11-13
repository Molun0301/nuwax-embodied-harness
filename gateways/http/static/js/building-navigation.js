// 楼层与整栋任务直接展示任务执行器的快照，目录只提供楼层与设施连接。
(function () {
  'use strict';
  const terminal = new Set(['succeeded', 'failed', 'cancelled', 'timed_out']);
  const capabilities = new Set(['navigate_to_floor', 'explore_building']);
  let panel, elements, buildings = [], task = null, busy = false;

  async function request(url, body) {
    const response = await fetch(url, body === undefined ? {} : {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.error?.message || payload.detail || '楼层请求失败');
    return payload;
  }
  function invoke(name, args = {}) {
    return request('/api/capabilities/' + name + '/invoke', {arguments: args});
  }
  function status(message, level = '') {
    elements.status.textContent = message;
    elements.status.dataset.level = level;
  }
  function building() { return buildings.find(value => value.building_id === elements.building.value); }
  function running() { return Boolean(task && !terminal.has(task.state)); }
  function updateButtons() {
    const disabled = busy || running();
    elements.submit.disabled = disabled || !elements.target.value;
    elements.cancel.disabled = busy || !running() || task.state === 'cancelling';
    elements.explore.disabled = disabled || !building();
    elements.import.disabled = disabled;
    for (const element of [elements.building, elements.target, elements.x, elements.y, elements.yaw]) element.disabled = disabled;
  }
  function renderFloors() {
    const selected = elements.target.value;
    elements.target.replaceChildren();
    for (const floor of building()?.floors || []) {
      const option = new Option(floor.display_name + ' · ' + floor.map_name, floor.floor_id);
      option.disabled = floor.enabled === false;
      elements.target.append(option);
    }
    elements.target.value = [...elements.target.options].find(value => value.value === selected && !value.disabled)?.value
      || [...elements.target.options].find(value => !value.disabled)?.value || '';
    updateButtons();
  }
  async function loadBuildings() {
    const reply = await invoke('list_buildings');
    buildings = reply.buildings;
    const selected = elements.building.value;
    elements.building.replaceChildren();
    for (const value of buildings) elements.building.append(new Option(value.display_name, value.building_id));
    if (buildings.some(value => value.building_id === selected)) elements.building.value = selected;
    renderFloors();
    if (!task) status(buildings.length ? '请选择目标楼层并填写坐标。' : '尚未登记楼栋，请导入楼层与设施配置。');
  }
  function renderTask() {
    if (!task) { elements.detail.textContent = ''; status('当前进程没有楼层任务记录。'); updateButtons(); return; }
    const data = task.data;
    const lines = ['任务：' + task.task_id, '功能：' + task.title + '（' + task.capability + '）', '阶段：' + task.message,
      '进度：' + Math.round(task.progress * 100) + '%'];
    if (data.floor_id) lines.push('楼层：' + data.floor_id);
    if (data.completed_floors) lines.push('已保存楼层：' + data.completed_floors.join('、'));
    if (task.error_code) lines.push('错误码：' + task.error_code);
    if (task.result) lines.push(JSON.stringify(task.result, null, 2));
    elements.detail.textContent = lines.join('\n');
    status('任务' + task.state_title, ['failed', 'timed_out'].includes(task.state) ? 'error' : '');
    updateButtons();
  }
  async function refreshTasks() {
    const snapshot = await request('/api/tasks');
    task = snapshot.active_tasks.find(value => capabilities.has(value.capability))
      || snapshot.history_tasks.find(value => capabilities.has(value.capability)) || null;
    renderTask();
  }
  async function submit(explore) {
    if (busy) return;
    busy = true; updateButtons();
    try {
      const args = explore ? {building_id: elements.building.value,
        floor_ids: building().floors.filter(value => value.enabled !== false).map(value => value.floor_id), timeout_sec: 7200}
        : {building_id: elements.building.value, floor_id: elements.target.value,
          x: parseFloat(elements.x.value), y: parseFloat(elements.y.value), yaw_rad: parseFloat(elements.yaw.value), timeout_sec: 600};
      const reply = await invoke(explore ? 'explore_building' : 'navigate_to_floor', args);
      task = reply.task;
      renderTask();
    } catch (error) { status(error.message, 'error'); }
    finally { busy = false; updateButtons(); }
  }
  function mount() {
    panel = document.querySelector('[data-role="floor-navigation"]');
    if (!panel) return;
    panel.hidden = false;
    elements = {};
    for (const key of ['building', 'target', 'x', 'y', 'yaw', 'submit', 'cancel', 'explore', 'status', 'detail', 'import']) {
      elements[key] = panel.querySelector('[data-role="floor-' + key + '"]');
    }
    elements.building.addEventListener('change', function () { elements.x.value = ''; elements.y.value = ''; renderFloors(); });
    elements.target.addEventListener('change', function () { elements.x.value = ''; elements.y.value = ''; });
    elements.submit.addEventListener('click', () => submit(false));
    elements.explore.addEventListener('click', () => submit(true));
    elements.cancel.addEventListener('click', async function () {
      busy = true; updateButtons();
      try {
        await request('/api/tasks/' + encodeURIComponent(task.task_id) + '/cancel', {});
        if (running()) status('取消请求已发送，等待原任务退出。');
      } catch (error) { status(error.message, 'error'); }
      finally { busy = false; updateButtons(); }
    });
    elements.import.addEventListener('change', async function () {
      const file = elements.import.files?.[0];
      if (!file) return;
      busy = true; updateButtons();
      try {
        await invoke('register_building', {topology: JSON.parse(await file.text())});
        await loadBuildings(); status('楼层配置已登记。');
      } catch (error) { status(error.message, 'error'); }
      finally { elements.import.value = ''; busy = false; updateButtons(); }
    });
    panel.addEventListener('toggle', async function () {
      if (panel.open) {
        try { await Promise.all([loadBuildings(), refreshTasks()]); }
        catch (error) { status(error.message, 'error'); }
      }
    });
    window.addEventListener('stream:tasks.state', function (event) {
      const update = event.detail;
      if (capabilities.has(update.capability) && (!task || update.task_id === task.task_id || terminal.has(task.state))) {
        task = update;
        renderTask();
      }
    });
    loadBuildings().catch(error => status(error.message, 'error'));
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
