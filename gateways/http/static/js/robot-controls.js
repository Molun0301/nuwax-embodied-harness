/* 常用机器人控制：急停、音量、运动面板与按名称导航，全部调用智能体同名能力。 */
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

  // ---- 运动面板 ----
  const motion = {
    panel: $('motionPanel'), toggle: $('mapMotionToggle'), status: $('motionStatus'), speed: $('motionSpeed'),
    turn: $('motionTurnSpeed'), duration: $('motionDuration'), action: $('motionAction'), params: $('motionActionParams'),
    level: $('motionSpeedLevel'), mode: $('motionMode'), avoidance: $('motionAvoidance'),
  };
  const MOVES = {
    forward: ['前进', 1, 0, 0], backward: ['后退', -1, 0, 0], left: ['左移', 0, 1, 0],
    right: ['右移', 0, -1, 0], turn_left: ['左转', 0, 0, 1], turn_right: ['右转', 0, 0, -1],
  };
  const MODE_LABELS = {high: '高层', low: '低层'};
  let catalog = {};
  function motionSay(text, level = '') { motion.status.textContent = text; motion.status.dataset.level = level; }
  async function motionRun(label, name, args, done) {
    motionSay(label + '：已发送。');
    try {
      const reply = await invoke(name, args);
      if (reply.task) {
        follow(reply.task, task => {
          if (!terminal.includes(task.state)) motionSay(label + '：' + task.message + percent(task));
          else if (task.state === 'succeeded') motionSay(done(task.result || {}));
          else motionSay(label + (task.state === 'cancelled' ? '已取消：' : '失败：') + task.message, 'error');
        });
      } else motionSay(done(reply));
    } catch (error) {
      motionSay(label + '失败：' + error.message, 'error');
    }
  }
  function renderActionParams() {
    const params = catalog[motion.action.value]?.params_schema?.properties || {};
    const fragment = document.createDocumentFragment();
    for (const [name, schema] of Object.entries(params)) {
      const label = document.createElement('label');
      label.className = 'msb-check';
      label.title = schema.description || name;
      const input = document.createElement('input');
      input.dataset.param = name;
      if (schema.type === 'boolean') {
        input.type = 'checkbox';
        input.checked = schema.default !== false;
        label.append(input, '开启');
      } else {
        input.className = 'msb-input msb-num';
        input.placeholder = name;
        label.append(name, input);
      }
      fragment.append(label);
    }
    motion.params.replaceChildren(fragment);
  }
  function setupMotion(caps) {
    const byName = new Map(caps.map(cap => [cap.name, cap]));
    // 只显示当前机器人实际提供的运动能力。
    const controls = [...motion.panel.querySelectorAll('[data-motion-capability]')];
    for (const control of controls) control.hidden = !byName.has(control.dataset.motionCapability);
    for (const row of motion.panel.querySelectorAll('.motion-settings')) row.hidden = !row.querySelector('[data-motion-capability]:not([hidden])');
    motion.toggle.hidden = !controls.some(control => !control.hidden);
    const sport = byName.get('execute_sport_command')?.input_schema;
    if (sport) {
      catalog = sport['x-nuwax-action-catalog'] || {};
      for (const name of sport.properties.action.enum) {
        motion.action.append(new Option((catalog[name]?.display_name || name) + '（' + name + '）', name));
      }
      renderActionParams();
    }
    const level = byName.get('set_speed_level')?.input_schema.properties.level;
    if (level) for (let value = level.minimum; value <= level.maximum; value += 1) motion.level.append(new Option(value + ' 档', value));
    const mode = byName.get('switch_control_mode')?.input_schema.properties.mode;
    if (mode) for (const value of mode.enum) motion.mode.append(new Option((MODE_LABELS[value] || value) + '（' + value + '）', value));
  }
  motion.panel?.querySelectorAll('[data-move]').forEach(button => button.addEventListener('click', () => {
    const [label, forward, side, turn] = MOVES[button.dataset.move];
    const speed = Number(motion.speed.value), rate = Number(motion.turn.value);
    const args = {vx: forward * speed, vy: side * speed, vyaw: turn * rate, duration_sec: Number(motion.duration.value)};
    motionRun(label, 'relative_move', args, result => label + '完成' + (result.limited ? '（速度已按设备限幅）' : '') + '。');
  }));
  motion.action?.addEventListener('change', renderActionParams);
  $('motionActionRun')?.addEventListener('click', () => {
    const name = motion.action.value;
    const args = {action: name};
    const inputs = [...motion.params.querySelectorAll('[data-param]')];
    if (inputs.length) args.params = Object.fromEntries(inputs.map(input => [input.dataset.param, input.type === 'checkbox' ? input.checked : input.value]));
    const label = catalog[name]?.display_name || name;
    motionRun(label, 'execute_sport_command', args, () => '已执行「' + label + '」。');
  });
  $('motionPoseApply')?.addEventListener('click', () => {
    const args = Object.fromEntries(['roll', 'pitch', 'yaw'].map(key => [key, Number($('motionPose-' + key).value)]));
    motionRun('设置姿态', 'set_body_pose', args, () => '机身姿态已设置。');
  });
  $('motionSpeedLevelApply')?.addEventListener('click', () => {
    const level = Number(motion.level.value);
    motionRun('设置速度档位', 'set_speed_level', {level}, () => '速度档位已设为 ' + level + ' 档。');
  });
  $('motionAvoidanceApply')?.addEventListener('click', () => {
    const enabled = motion.avoidance.value === 'true';
    motionRun('设置实时避障', 'set_obstacle_avoidance', {enabled}, () => '实时避障已' + (enabled ? '开启' : '关闭') + '。');
  });
  $('motionModeApply')?.addEventListener('click', () => {
    const mode = motion.mode.value;
    motionRun('切换控制模式', 'switch_control_mode', {mode}, () => '已切换到' + (MODE_LABELS[mode] || mode) + '控制。');
  });
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
