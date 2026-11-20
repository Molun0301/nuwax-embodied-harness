/* 常用机器人控制：急停、音量、移动与转向、按名称导航，全部调用智能体同名能力。 */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  const capabilities = window.nuwaxCapabilities;
  const invoke = capabilities.invoke;
  const clock = () => new Date().toLocaleTimeString();
  // 任务类能力的进度随任务流更新；回调收到每次任务快照。
  const followers = new Map();
  window.addEventListener('stream:tasks.state', event => {
    const task = event.detail;
    const follow = followers.get(task?.task_id);
    if (!follow) return;
    if (terminal.includes(task.state)) followers.delete(task.task_id);
    follow(task);
  });
  function follow(task, callback) {
    if (terminal.includes(task.state)) { callback(task); return; }
    followers.set(task.task_id, callback);
    callback(task);
  }
  function percent(task) {
    const value = Number(task.progress);
    return Number.isFinite(value) ? ' ' + Math.round(value * 100) + '%' : '';
  }

  // ---- 急停：任何页面一键可达，不确认、不禁用，可连续发送 ----
  const stop = $('emergencyStop'), stopStatus = $('emergencyStopStatus');
  stop?.addEventListener('click', async () => {
    stopStatus.textContent = '正在急停…';
    stopStatus.dataset.level = 'warn';
    try {
      const result = await invoke('stop_all_motion', {reason: '网页急停'});
      const issues = Object.entries(result).filter(([, value]) => value !== 'ok')
        .map(([key, value]) => ({guard: '守卫', task: '任务', geometry: '导航'}[key] || key) + '：' + value);
      stopStatus.textContent = '已急停 ' + clock() + (issues.length ? '（' + issues.join('；') + '）' : '');
      stopStatus.dataset.level = issues.length ? 'warn' : '';
    } catch (error) {
      stopStatus.textContent = '急停失败：' + error.message;
      stopStatus.dataset.level = 'error';
    }
    stopStatus.title = stopStatus.textContent;
  });

  // ---- 音量：机器人支持音量能力时显示在顶栏 ----
  const volume = {root: $('volumeStat'), slider: $('volumeSlider'), value: $('volumeValue'), label: $('volumeLabel')};
  function renderVolume(volumeState) {
    const ratio = volumeState.volume;
    volume.value.textContent = ratio === null || ratio === undefined ? '--' : Math.round(ratio * 100) + '%';
    if (ratio !== null && ratio !== undefined) volume.slider.value = Math.round(ratio * 100);
    volume.root.title = volumeState.last_error ? '音量操作失败：' + volumeState.last_error : '';
    volume.label.textContent = volumeState.last_error ? '音量（失败）' : '音量';
    volume.root.classList.toggle('is-error', Boolean(volumeState.last_error));
  }
  async function setupVolume() {
    volume.root.hidden = false;
    try { renderVolume((await invoke('get_volume')).volume_state); }
    catch (error) { renderVolume({volume: null, last_error: error.message}); }
    volume.slider.addEventListener('input', () => { volume.value.textContent = volume.slider.value + '%'; });
    volume.slider.addEventListener('change', async () => {
      try { renderVolume((await invoke('set_volume', {volume: Number(volume.slider.value) / 100})).volume_state); }
      catch (error) { renderVolume({volume: null, last_error: error.message}); }
    });
  }

  // ---- 移动与转向：按住移动、松开即停，按键与面板按钮共用一套逻辑 ----
  // 按住期间每 RENEW_MS 提交一次 HOLD_SEC 秒的 relative_move，新命令接管旧命令；
  // 松开、页面失焦或面板关闭时取消当前任务（cancel_task），任务退出即释放租约，守卫立即零速。
  // 网络断开导致取消发不出去时，最后一条命令最多再走 HOLD_SEC 秒。
  const HOLD_SEC = 2, RENEW_MS = 1500;
  const motion = {panel: $('motionPanel'), toggle: $('mapMotionToggle'), status: $('motionPanelStatus'),
    speed: $('motionSpeed'), turn: $('motionTurnSpeed')};
  // 每个键对应 [前后, 左右, 转向] 方向；方向键映射到面板上的同义按钮。
  const DRIVE_KEYS = {KeyW: [1, 0, 0], KeyS: [-1, 0, 0], KeyA: [0, 1, 0], KeyD: [0, -1, 0], KeyQ: [0, 0, 1], KeyE: [0, 0, -1]};
  const ARROWS = {ArrowUp: 'KeyW', ArrowDown: 'KeyS', ArrowLeft: 'KeyQ', ArrowRight: 'KeyE'};
  const KEY_LABELS = {KeyW: '前进', KeyS: '后退', KeyA: '左移', KeyD: '右移', KeyQ: '左转', KeyE: '右转'};
  const drive = {held: new Set(), vectorKey: '', taskId: null, seq: 0, timer: null};
  function motionSay(text, level = '') { motion.status.textContent = text; motion.status.dataset.level = level; }
  function driveVector() {
    const axis = [0, 0, 0];
    for (const key of drive.held) DRIVE_KEYS[key].forEach((value, index) => { axis[index] += value; });
    const [forward, side, turn] = axis.map(value => Math.max(-1, Math.min(1, value)));
    if (!forward && !side && !turn) return null;
    const linear = Number(motion.speed.value), angular = Number(motion.turn.value);
    return {vx: forward * linear, vy: side * linear, vyaw: turn * angular};
  }
  async function sendDrive() {
    const vector = driveVector();
    const seq = ++drive.seq;
    try {
      const reply = await invoke('relative_move', {...vector, duration_sec: HOLD_SEC});
      if (seq === drive.seq) drive.taskId = reply.task.task_id;
      // 回应到达前已经松开：这条命令立即取消。
      else if (!drive.held.size) invoke('cancel_task', {task_id: reply.task.task_id}).catch(() => {});
    } catch (error) {
      motionSay('移动失败：' + error.message, 'error');
    }
  }
  function stopDrive() {
    window.clearInterval(drive.timer);
    drive.timer = null;
    drive.seq += 1;
    const taskId = drive.taskId;
    drive.taskId = null;
    if (taskId) invoke('cancel_task', {task_id: taskId}).catch(error => motionSay('停止失败：' + error.message, 'error'));
  }
  function applyDrive() {
    for (const button of motion.panel.querySelectorAll('[data-key]')) button.classList.toggle('is-active', drive.held.has(button.dataset.key));
    const vector = driveVector();
    const key = vector ? [vector.vx, vector.vy, vector.vyaw].join() : '';
    if (key === drive.vectorKey) return;
    drive.vectorKey = key;
    if (!vector) {
      stopDrive();
      motionSay('已停止。');
      return;
    }
    motionSay('移动中：' + [...drive.held].map(item => KEY_LABELS[item]).join(' + '));
    window.clearInterval(drive.timer);
    sendDrive();
    drive.timer = window.setInterval(sendDrive, RENEW_MS);
  }
  function releaseAll() {
    if (!drive.held.size) return;
    drive.held.clear();
    applyDrive();
  }
  function typing(target) {
    return target instanceof HTMLElement && (target.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName));
  }
  document.addEventListener('keydown', event => {
    const key = ARROWS[event.code] || event.code;
    if (motion.panel.hidden || !DRIVE_KEYS[key] || event.ctrlKey || event.metaKey || event.altKey || typing(event.target)) return;
    event.preventDefault();
    if (event.repeat || drive.held.has(key)) return;
    drive.held.add(key);
    applyDrive();
  });
  // 松开键在任何焦点下都处理，避免按住时点进输入框后停不下来。
  document.addEventListener('keyup', event => {
    const key = ARROWS[event.code] || event.code;
    if (!drive.held.delete(key)) return;
    applyDrive();
  });
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
  new MutationObserver(() => { if (motion.panel.hidden) releaseAll(); }).observe(motion.panel, {attributes: true, attributeFilter: ['hidden']});
  for (const button of motion.panel.querySelectorAll('[data-key]')) {
    button.addEventListener('pointerdown', event => {
      if (event.button !== 0) return;
      button.setPointerCapture(event.pointerId);
      drive.held.add(button.dataset.key);
      applyDrive();
    });
    const up = () => { if (drive.held.delete(button.dataset.key)) applyDrive(); };
    button.addEventListener('pointerup', up);
    button.addEventListener('pointercancel', up);
    button.addEventListener('lostpointercapture', up);
  }
  function setupMotion(caps) {
    // 只有当前机器人提供 relative_move 时才显示面板入口。
    motion.toggle.hidden = !caps.some(cap => cap.name === 'relative_move');
  }
  motion.toggle?.addEventListener('click', () => {
    if (!motion.panel.hidden) { window.nuwaxMapPanels.close(motion.panel.id, true); return; }
    window.nuwaxMapPanels.activate(motion.panel.id);
    motion.panel.hidden = false;
  });

  // ---- 按名称导航：名称来自当前地图记忆中的地点与区域 ----
  const named = {target: $('namedNavTarget'), names: $('namedNavNames'), verify: $('namedNavVerify'), submit: $('namedNavSubmit'),
    cancel: $('namedNavCancel'), status: $('namedNavStatus'), taskId: null};
  function namedSay(text, level = '') { named.status.textContent = text; named.status.dataset.level = level; }
  async function loadNames() {
    const names = new Set();
    const mapName = window.__nuwaxMapBinding?.().map_name;
    const [locations, spaces] = await Promise.allSettled([
      fetch('/api/memory/locations?limit=500').then(response => response.json()),
      invoke('list_scene_spaces', mapName ? {limit: 500, map_name: mapName} : {limit: 500}),
    ]);
    for (const item of locations.value?.locations || []) [item.name, ...(item.aliases || [])].forEach(name => name && names.add(name));
    for (const item of spaces.value?.scene_spaces || []) [item.primary_label, ...(item.human_aliases || [])].forEach(name => name && names.add(name));
    named.names.replaceChildren(...[...names].sort().map(name => new Option(name)));
    namedSay(names.size ? '当前地图记忆中有 ' + names.size + ' 个名称可选，也可直接输入。' : '当前地图记忆中还没有地点或区域名称，可直接输入。');
  }
  named.submit?.addEventListener('click', async () => {
    const target = named.target.value.trim();
    named.submit.disabled = true;
    namedSay('已提交，等待网关应答。');
    try {
      const reply = await invoke('navigate_to_named_location', {target_name: target, verify_on_arrival: named.verify.checked});
      named.taskId = reply.task.task_id;
      follow(reply.task, task => {
        const running = !terminal.includes(task.state);
        named.cancel.disabled = !running || task.state === 'cancelling';
        named.submit.disabled = false;
        if (running) namedSay('前往「' + target + '」：' + task.message + percent(task));
        else if (task.state === 'succeeded') namedSay('已到达「' + target + '」' + (task.result?.verification ? '并完成核对' : '') + '。');
        else namedSay('前往「' + target + '」' + (task.state === 'cancelled' ? '已取消：' : '失败：') + task.message, 'error');
      });
    } catch (error) {
      named.submit.disabled = false;
      namedSay('导航失败：' + error.message, 'error');
    }
  });
  named.cancel?.addEventListener('click', async () => {
    named.cancel.disabled = true;
    try { await invoke('cancel_task', {task_id: named.taskId}); }
    catch (error) { named.cancel.disabled = false; namedSay('取消失败：' + error.message, 'error'); }
  });
  $('namedNavRefresh')?.addEventListener('click', () => loadNames().catch(error => namedSay('读取名称失败：' + error.message, 'error')));
  document.querySelector('[data-nav-tab="named"]')?.addEventListener('click', () => loadNames().catch(error => namedSay('读取名称失败：' + error.message, 'error')));

  capabilities.load().then(caps => {
    const names = new Set(caps.map(cap => cap.name));
    if (names.has('get_volume') && names.has('set_volume')) setupVolume();
    setupMotion(caps);
  }).catch(error => { if (motion.status) motionSay('读取能力清单失败：' + error.message, 'error'); });
})();
