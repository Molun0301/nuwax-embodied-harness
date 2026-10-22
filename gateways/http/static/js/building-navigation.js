// 楼层连接使用独立的冻结任务；设施状态和恢复要求由后端权威回执提供。
(function () {
  'use strict';
  const terminal = ['succeeded', 'failed', 'cancelled', 'timeout'];
  const stageNames = { accepted: '准备连接', navigating_waiting: '前往等候点', reserving: '预约设施', calling: '等待设施到达', waiting_entry: '确认进梯条件', boarding: '进入设施', stopping_inside: '确认停车', suspending_mapping: '暂停原楼层建图', transporting: '设施运载中', waiting_arrival: '确认目标停层', activating_target: '加载目标层并定位', exiting: '离开设施', releasing: '释放预约', navigating_final: '前往目标位姿', succeeded: '已到达并停车' };
  let panel, elements, buildings = [], current = null, workflow = null, taskId = null, taskRunning = false, busy = false, timer = null;

  async function request(url, arguments_) {
    const response = await fetch(url, arguments_ === undefined ? {} : arguments_ === null ? { method: 'POST' } : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ arguments: arguments_ }) });
    const payload = await response.json();
    if (!response.ok || payload.ok === false) {
      const error = new Error(payload.error?.message || payload.detail || '楼层请求失败');
      error.status = response.status;
      throw error;
    }
    return payload;
  }
  function invoke(name, arguments_) { return request('/api/capabilities/' + name + '/invoke', arguments_ || {}); }
  function status(message, level) { elements.status.textContent = message; elements.status.dataset.level = level || ''; }
  function building() { return buildings.find(item => item.building_id === elements.building.value); }
  function floorName(id) { const scope = buildings.find(item => item.building_id === current?.request.building_id) || building(); return scope?.floors.find(item => item.floor_id === id)?.display_name || id || '待确认'; }
  function updateButtons() {
    const pendingWorkflow = workflow && !workflow.terminal;
    const pending = pendingWorkflow || (current && !current.terminal);
    elements.submit.disabled = busy || pending || taskRunning || !elements.target.value;
    elements.cancel.disabled = busy || !taskRunning || !taskId;
    elements.resume.hidden = !pending || taskRunning || !!pendingWorkflow;
    elements['resume-explore'].hidden = !pendingWorkflow || taskRunning;
    elements['resume-explore'].disabled = busy;
    elements.explore.disabled = busy || pending || taskRunning || !building();
    elements.resume.disabled = busy;
    elements.import.disabled = busy || pending || taskRunning;
    [elements.building, elements.target, elements.x, elements.y, elements.yaw].forEach(element => { element.disabled = busy || pending || taskRunning; });
  }
  function renderFloors() {
    const selected = elements.target.value;
    elements.target.replaceChildren();
    for (const floor of building()?.floors || []) {
      const option = new Option(floor.display_name + ' · ' + floor.map_name, floor.floor_id);
      option.disabled = !floor.enabled || !floor.asset_identity;
      elements.target.append(option);
    }
    if ([...elements.target.options].some(option => option.value === selected && !option.disabled)) elements.target.value = selected;
    else elements.target.value = [...elements.target.options].find(option => !option.disabled)?.value || '';
    updateButtons();
  }
  async function loadBuildings() {
    const reply = await invoke('list_buildings');
    buildings = reply.result?.buildings || [];
    const selected = elements.building.value;
    elements.building.replaceChildren();
    buildings.forEach(item => elements.building.append(new Option(item.display_name, item.building_id)));
    if (buildings.some(item => item.building_id === selected)) elements.building.value = selected;
    renderFloors();
    if (!buildings.length) status('尚未登记建筑，请先完成楼层和设施配置。');
    else if (!current) status('请选择目标楼层并填写坐标。');
  }
  function renderState() {
    if (workflow && (!workflow.terminal || !current || current.terminal)) {
      const completed = Object.keys(workflow.completed_floors || {}).length;
      const total = workflow.request.floors.length;
      const target = workflow.request.floors[workflow.next_floor_index];
      const lines = ['楼层进度：' + completed + ' / ' + total, '阶段：' + ({accepted:'准备下一层', transfer:'跨层转移', exploring:'层内探索', finalizing:'保存该层地图', succeeded:'逐层任务完成'}[workflow.stage] || workflow.stage)];
      if (target) lines.push('当前目标层：' + target.display_name);
      if (workflow.message) lines.push(workflow.message);
      if (workflow.needs_reconciliation) lines.push('进度已保留，请继续原逐层请求。');
      elements.detail.textContent = lines.join('\n');
      status(workflow.terminal ? '设定的逐层探索已完成。' : workflow.needs_reconciliation ? '逐层探索等待恢复。' : '逐层探索执行中。', workflow.needs_reconciliation ? 'error' : '');
      updateButtons(); return;
    }
    if (!current) { elements.detail.textContent = ''; updateButtons(); return; }
    const lines = ['目标楼层：' + floorName(current.request.target_floor_id), '当前楼层：' + floorName(current.current_floor_id), '阶段：' + (stageNames[current.stage] || current.stage)];
    const facility = current.facility_state;
    if (facility) {
      const stale = Date.now() - Date.parse(facility.observed_at) > 2000;
      lines.push('设施停层：' + floorName(facility.floor_id));
      lines.push('设施：' + (stale ? '反馈已过期' : facility.moving ? '运载中' : facility.door === 'open' ? '已开门' : facility.door === 'closed' ? '门已关闭' : '门状态待确认'));
    }
    if (current.pending_operation && current.pending_operation.state !== 'succeeded') lines.push('设施回执：待确认');
    if (current.needs_reconciliation) lines.push('机器人已请求停车，设施操作需继续核对。');
    if (current.last_message) lines.push(current.last_message);
    elements.detail.textContent = lines.join('\n');
    status(current.needs_reconciliation ? '任务等待对账，可继续原请求。' : stageNames[current.stage] || '楼层任务执行中。', current.needs_reconciliation ? 'error' : '');
    updateButtons();
  }
  async function refreshState() {
    const reply = await invoke('get_floor_navigation_state');
    current = reply.result?.state || null;
    const explorationReply = await invoke('get_building_exploration_state');
    workflow = explorationReply.result?.state || null;
    const owner = workflow && !workflow.terminal ? workflow.owner_task_id : current?.owner_task_id;
    if (owner && (!taskId || !taskRunning)) taskId = owner;
    taskRunning = false;
    if (taskId) {
      try {
        const snapshot = await request('/api/tasks/' + encodeURIComponent(taskId));
        taskRunning = !terminal.includes(snapshot.status?.state);
      } catch (error) { if (error.status !== 404) throw error; }
    }
    renderState();
  }
  function schedule() {
    clearTimeout(timer);
    timer = setTimeout(async function () {
      if (panel.open && document.visibilityState !== 'hidden' && !busy) {
        try { await refreshState(); } catch (error) { status(error.message, 'error'); }
      }
      schedule();
    }, 1000);
  }
  async function submit(resume) {
    if (busy) return;
    busy = true; updateButtons();
    try {
      let args;
      if (resume) {
        if (!current || current.terminal) throw new Error('没有可继续的原请求。');
        args = { request_id: current.request.request_id };
      } else {
        const values = [elements.x.value, elements.y.value, elements.yaw.value];
        if (values.some(value => !value.trim() || !Number.isFinite(Number(value)))) throw new Error('请填写有效的目标层坐标和朝向。');
        if (!building() || !elements.target.value) throw new Error('请选择有完整地图的目标楼层。');
        args = { building_id: elements.building.value, floor_id: elements.target.value, x: Number(values[0]), y: Number(values[1]), yaw_rad: Number(values[2]), timeout_sec: 600 };
      }
      const reply = await invoke(resume ? 'resume_floor_navigation' : 'navigate_to_floor', args);
      taskId = reply.task_id;
      if (!taskId) throw new Error('后端没有返回任务标识。');
      taskRunning = true; status('任务已受理，正在核对执行条件。');
      await refreshState();
    } catch (error) { status(error.message, 'error'); }
    finally { busy = false; updateButtons(); }
  }
  async function exploreBuilding(resume) {
    if (busy) return;
    busy = true; updateButtons();
    try {
      if (!resume && !building()) throw new Error('请先选择建筑。');
      if (resume && (!workflow || workflow.terminal)) throw new Error('没有可继续的逐层探索。');
      const args = resume ? { request_id: workflow.request.request_id } : { building_id: elements.building.value, floor_ids: building().floors.filter(floor => floor.enabled).map(floor => floor.floor_id), timeout_sec: 7200 };
      const reply = await invoke(resume ? 'resume_building_exploration' : 'explore_building', args);
      if (!reply.task_id) throw new Error('后端没有返回逐层任务标识。');
      taskId = reply.task_id; taskRunning = true;
      status('逐层任务已受理，正在核对楼层与设施。');
      await refreshState();
    } catch (error) { status(error.message, 'error'); }
    finally { busy = false; updateButtons(); }
  }
  function mount() {
    panel = document.querySelector('[data-role="floor-navigation"]');
    if (!panel || window.__NUWAX_GEOMETRY_PIPELINE__ !== 'slam_2d') return;
    panel.hidden = false;
    elements = {};
    for (const key of ['building', 'target', 'x', 'y', 'yaw', 'submit', 'cancel', 'resume', 'explore', 'resume-explore', 'status', 'detail', 'import']) elements[key] = panel.querySelector('[data-role="floor-' + key + '"]');
    elements.building.addEventListener('change', function () { elements.x.value = ''; elements.y.value = ''; renderFloors(); });
    elements.target.addEventListener('change', function () { elements.x.value = ''; elements.y.value = ''; });
    elements.submit.addEventListener('click', () => submit(false));
    elements.resume.addEventListener('click', () => submit(true));
    elements.explore.addEventListener('click', () => exploreBuilding(false));
    elements['resume-explore'].addEventListener('click', () => exploreBuilding(true));
    elements.cancel.addEventListener('click', async function () {
      if (!taskId || busy) return;
      busy = true; updateButtons();
      try { await request('/api/tasks/' + encodeURIComponent(taskId) + '/cancel', null); status('已请求停止，等待机器人和设施状态确认。'); }
      catch (error) { status(error.message, 'error'); }
      finally { busy = false; updateButtons(); }
    });
    elements.import.addEventListener('change', async function () {
      const file = elements.import.files?.[0]; if (!file) return;
      busy = true; updateButtons();
      try {
        if (file.size > 4 * 1024 * 1024) throw new Error('楼层配置文件超过大小限制。');
        const topology = JSON.parse(await file.text());
        await invoke('register_building', { topology, expected_revision: topology.revision - 1 });
        await loadBuildings(); status('楼层配置已登记。');
      } catch (error) { status(error.message, 'error'); }
      finally { elements.import.value = ''; busy = false; updateButtons(); }
    });
    panel.addEventListener('toggle', function () { if (panel.open) Promise.all([loadBuildings(), refreshState()]).catch(error => status(error.message, 'error')); });
    loadBuildings().catch(error => status(error.message, 'error')); schedule();
    window.addEventListener('pagehide', () => clearTimeout(timer));
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
