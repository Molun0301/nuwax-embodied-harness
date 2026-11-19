/* 地图管理：列出已保存地图，激活、重定位、建图、保存、另存与删除都调用智能体同名能力。 */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const panel = $('mapManagerPanel');
  if (!panel) return;
  const terminal = ['succeeded', 'failed', 'cancelled', 'timed_out'];
  const PHASES = {
    mapping: ['建图中', 'info'], exploring: ['探索中', 'info'], loaded: ['已加载，未定位', 'warn'],
    localized: ['已定位', 'available'], fault: ['故障', 'error'],
  };
  const state = {maps: [], phase: null, current: '', busy: false, task: null};
  const el = {
    current: $('mapManagerCurrentName'), phase: $('mapManagerPhase'), list: $('mapManagerList'),
    name: $('mapManagerName'), status: $('mapManagerStatus'), relocalize: $('mapManagerRelocalize'),
    save: $('mapManagerSave'), end: $('mapManagerEnd'), create: $('mapManagerCreate'), saveAs: $('mapManagerSaveAs'),
    refresh: $('mapManagerRefresh'),
  };

  function say(text, level = '') { el.status.textContent = text; el.status.dataset.level = level; }
  async function invoke(name, args = {}) {
    const response = await fetch('/api/capabilities/' + name + '/invoke', {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: JSON.stringify({arguments: args})});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error?.message || '地图操作失败');
    return value;
  }
  // 有会话的阶段才算“当前地图”；空闲时没有激活地图。
  function hasSession() { return Boolean(state.phase && state.phase !== 'idle'); }
  function formatTime(value) { return value ? new Date(value).toLocaleString() : '时间未知'; }
  function formatSize(bytes) { return bytes >= 1048576 ? (bytes / 1048576).toFixed(1) + ' MB' : Math.max(1, Math.round(bytes / 1024)) + ' KB'; }

  function renderSession() {
    const session = hasSession();
    const [label, level] = PHASES[state.phase] || ['没有地图会话', 'idle'];
    el.current.textContent = session ? state.current || '未命名地图' : '—';
    el.phase.className = 'pill ' + level;
    el.phase.textContent = label;
    for (const button of [el.relocalize, el.save, el.end]) button.disabled = state.busy || !session;
    el.saveAs.disabled = state.busy || !session || !el.name.value.trim();
    el.create.disabled = state.busy || !el.name.value.trim();
    el.refresh.disabled = state.busy;
  }
  function actionButton(text, title, handler, danger = false) {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'msb-btn' + (danger ? ' danger' : '');
    button.textContent = text;
    button.title = title;
    button.disabled = state.busy;
    button.addEventListener('click', handler);
    return button;
  }
  function renderList() {
    const fragment = document.createDocumentFragment();
    if (!state.maps.length) {
      const empty = document.createElement('li');
      empty.className = 'map-manager-empty';
      empty.textContent = '还没有地图。填写名称后点“新建并开始建图”。';
      fragment.append(empty);
    }
    for (const map of state.maps) {
      const current = hasSession() && map.name === state.current;
      const item = document.createElement('li');
      item.className = 'map-manager-item';
      item.dataset.current = String(current);
      const main = document.createElement('div');
      main.className = 'map-manager-main';
      const name = document.createElement('strong');
      name.textContent = map.display_name || map.name;
      name.title = map.name;
      main.append(name);
      if (current) {
        const badge = document.createElement('span');
        const [label, level] = PHASES[state.phase] || ['', 'idle'];
        badge.className = 'pill ' + level;
        badge.textContent = '当前 · ' + label;
        main.append(badge);
      }
      if (!map.saved) {
        const unsaved = document.createElement('span');
        unsaved.className = 'pill warn';
        unsaved.textContent = '未保存';
        main.append(unsaved);
      }
      const meta = document.createElement('span');
      meta.className = 'msb-meta';
      meta.textContent = '更新 ' + formatTime(map.updated_at) + ' · ' + formatSize(map.size_bytes);
      main.append(meta);
      const actions = document.createElement('div');
      actions.className = 'map-manager-actions';
      actions.append(
        actionButton('激活', '加载这张地图并重定位', () => activate(map.name)),
        actionButton('续建', '在这张地图上继续建图', () => startMapping(map.name, true)),
        actionButton('删除', '删除地图及同目录的记忆库', () => remove(map.name), true),
      );
      item.append(main, actions);
      fragment.append(item);
    }
    el.list.replaceChildren(fragment);
  }
  function render() { renderSession(); renderList(); }

  async function load() {
    const data = await invoke('list_maps');
    state.maps = data.maps;
    state.phase = data.phase;
    state.current = data.map_name;
    render();
  }
  // 动作类能力直接返回结果；任务类能力返回任务，进度随任务流更新。
  async function run(label, name, args, done) {
    if (state.busy) return;
    state.busy = true; render();
    say(label + '：已提交，等待网关应答。');
    try {
      const reply = await invoke(name, args);
      if (reply.task) {
        state.task = {id: reply.task.task_id, label, done};
        say(label + '：' + reply.task.message);
      } else {
        say(done(reply));
        await load();
      }
    } catch (error) {
      say(label + '失败：' + error.message, 'error');
    } finally {
      state.busy = false; render();
    }
  }
  function confirmed(text) { return window.confirm(text); }

  function activate(name) {
    if (!confirmed('激活地图「' + name + '」？将加载这张地图并重定位，当前地图会话会被替换。')) return;
    run('激活「' + name + '」', 'load_map', {map_name: name}, result => result.localized === false
      ? '已加载「' + name + '」，未定位：' + result.reason : '已激活「' + name + '」并完成定位。');
  }
  function startMapping(name, continueSaved) {
    const text = continueSaved ? '在地图「' + name + '」上继续建图？' : '新建地图「' + name + '」并开始建图？';
    if (!confirmed(text + '当前地图会话会被替换。')) return;
    run(continueSaved ? '续建「' + name + '」' : '新建「' + name + '」', 'start_mapping',
      {map_name: name, continue_saved: continueSaved}, () => '已开始在「' + name + '」上建图。');
  }
  function remove(name) {
    const current = hasSession() && name === state.current ? '这是当前地图，会先结束地图会话。' : '';
    if (!confirmed('删除地图「' + name + '」及同目录的记忆库？' + current + '删除后无法恢复。')) return;
    run('删除「' + name + '」', 'delete_map', {map_name: name}, () => '已删除地图「' + name + '」。');
  }

  window.addEventListener('stream:tasks.state', event => {
    const task = event.detail;
    if (!state.task || task.task_id !== state.task.id) return;
    const {label, done} = state.task;
    if (!terminal.includes(task.state)) { say(label + '：' + task.message); return; }
    state.task = null;
    if (task.state === 'succeeded') say(done(task.result || {}));
    else say(label + (task.state === 'cancelled' ? '已取消：' : '失败：') + task.message, 'error');
    load().catch(error => say('读取地图列表失败：' + error.message, 'error'));
  });
  // 当前会话阶段随 Geometry 状态实时更新，不必重读列表。
  window.addEventListener('stream:geometry.state', event => {
    const snapshot = event.detail;
    if (!snapshot || (snapshot.phase === state.phase && snapshot.map_name === state.current)) return;
    const listed = state.maps.some(map => map.name === snapshot.map_name);
    state.phase = snapshot.phase;
    state.current = snapshot.map_name;
    if (panel.hidden) return;
    render();
    if (snapshot.map_name && !listed) load().catch(error => say('读取地图列表失败：' + error.message, 'error'));
  });

  el.relocalize.addEventListener('click', () => run('重定位', 'relocalize_active_map', {},
    result => '重定位成功' + (Number.isFinite(result.fitness) ? '，匹配度 ' + result.fitness.toFixed(3) : '') + '。'));
  el.save.addEventListener('click', () => run('保存「' + state.current + '」', 'save_map', {},
    result => '已保存地图「' + result.map_name + '」。'));
  el.saveAs.addEventListener('click', () => {
    const name = el.name.value.trim();
    run('另存为「' + name + '」', 'save_map', {map_name: name}, result => '已另存为「' + result.map_name + '」。');
  });
  el.end.addEventListener('click', () => {
    if (!confirmed('结束当前地图会话？未保存的建图内容会丢失。')) return;
    run('结束会话', 'end_map_session', {}, () => '已结束地图会话。');
  });
  el.create.addEventListener('click', () => startMapping(el.name.value.trim(), false));
  el.name.addEventListener('input', renderSession);
  el.refresh.addEventListener('click', () => load().then(() => say('地图列表已刷新。'))
    .catch(error => say('读取地图列表失败：' + error.message, 'error')));
  $('mapManagerToggle').addEventListener('click', () => {
    if (!panel.hidden) { window.nuwaxMapPanels.close(panel.id, true); return; }
    window.nuwaxMapPanels.activate(panel.id);
    panel.hidden = false;
    render();
    load().catch(error => say('读取地图列表失败：' + error.message, 'error'));
  });
})();
