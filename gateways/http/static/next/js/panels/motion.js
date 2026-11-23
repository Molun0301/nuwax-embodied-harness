/* 运动面板：从 robot-controls.js 迁出，行为不变。
   - 只做通用的移动与转向（relative_move）；机身动作、姿态等设备专有能力在能力页调用；
   - 键盘 W/S 前进后退、A/D 左右平移、Q/E 左右转，方向键 ↑/↓ 前进后退、←/→ 左右转，可组合；面板与舞台遥控浮层上的
     六个按键既是图示也可用鼠标按住；焦点在输入框时按键不控制机器人；
   - 按住期间按线速度、角速度每 1.5 秒提交一次 2 秒的 relative_move（新命令接管旧命令），组合变化立即重发；
     松开、页面失焦、标签页隐藏或面板关闭时对当前任务调用 cancel_task；回应到达前已松开的命令到达后立即取消。 */
import { h, icon } from '/static/shared/neh-ui.js';

const HOLD_SEC = 2, RENEW_MS = 1500;
// 每个键对应 [前后, 左右, 转向]；方向键映射到同义按键。
const DRIVE_KEYS = { KeyW: [1, 0, 0], KeyS: [-1, 0, 0], KeyA: [0, 1, 0], KeyD: [0, -1, 0], KeyQ: [0, 0, 1], KeyE: [0, 0, -1] };
const ARROWS = { ArrowUp: 'KeyW', ArrowDown: 'KeyS', ArrowLeft: 'KeyQ', ArrowRight: 'KeyE' };
const KEYS = [['KeyQ', 'Q', '左转'], ['KeyW', 'W', '前进'], ['KeyE', 'E', '右转'], ['KeyA', 'A', '左移'], ['KeyS', 'S', '后退'], ['KeyD', 'D', '右移']];

export function mountMotion(root, ctx) {
  const { api, stage, go } = ctx;
  let shown = false, supported = true;
  const drive = { held: new Set(), vectorKey: '', taskId: null, seq: 0, timer: null };
  const speed = { linear: 0.3, angular: 0.6 };
  const status = h('p', { class: 'c-status-line', role: 'status' }, '按住按键或按钮移动，松开即停。');
  const pads = [];
  function pad(cls) {
    const box = h('div', { class: 'c-pad ' + (cls || ''), role: 'group', 'aria-label': '按住移动，松开停止' }, KEYS.map(([code, key, label]) => {
      const button = h('button', { class: 'c-key', type: 'button', dataset: { code, down: 'false' } }, h('b', {}, key), label);
      button.addEventListener('pointerdown', event => {
        if (event.button !== 0 || !supported) return;
        button.setPointerCapture(event.pointerId);
        drive.held.add(code);
        applyDrive();
      });
      const up = () => { if (drive.held.delete(code)) applyDrive(); };
      button.addEventListener('pointerup', up);
      button.addEventListener('pointercancel', up);
      button.addEventListener('lostpointercapture', up);
      return button;
    }));
    pads.push(box);
    return box;
  }
  function slider(label, key, min, max, step, unit) {
    const out = h('output', { class: 'n-num' }, speed[key].toFixed(2) + ' ' + unit);
    const input = h('input', { type: 'range', min, max, step, value: speed[key], class: 'c-range', 'aria-label': label });
    input.addEventListener('input', () => { speed[key] = Number(input.value); out.textContent = speed[key].toFixed(2) + ' ' + unit; });
    return h('div', { class: 'n-field c-field-gap' }, h('div', { class: 'c-field-head' }, h('span', { class: 'n-label' }, label), out), input);
  }
  const unsupported = h('div', { class: 'n-callout', hidden: true, html: icon('info') }, h('div', { class: 'n-callout-body' }, '这台机器人没有提供移动能力（relative_move），不能遥控移动。'));
  root.append(h('div', { class: 'c-pane-head' }, h('h2', {}, '移动与转向')),
    h('p', { class: 'n-muted c-gap-b' }, '这个面板打开时，按住 W A S D Q E 或方向键移动，松开即停；光标在输入框里时键盘不控制机器人。'),
    unsupported,
    h('div', { class: 'c-pad-wrap' }, pad()),
    slider('线速度', 'linear', 0.05, 1, 0.05, '米/秒'), slider('角速度', 'angular', 0.1, 1.5, 0.05, '弧度/秒'),
    h('p', { class: 'n-meta c-gap-b' }, '方向键：↑ ↓ 前进后退，← → 左右转；可组合，如 W 加 Q 边走边转。'),
    h('button', { class: 'n-btn n-btn--block', type: 'button', html: icon('arrow-left-right', 'n-icon--sm'), onclick: () => stage.swapVideo() }, '画面放到主区域'),
    status,
    h('p', { class: 'n-meta c-gap-t' }, '机身动作、姿态、档位等设备专有功能在', h('button', { class: 'c-link', type: 'button', onclick: () => go('caps') }, '能力页'), '调用。'));
  const hudSpeed = h('span', { class: 'n-muted' });
  const hud = h('div', { class: 'n-float c-hud' }, pad(), h('div', { class: 'c-hud-text' }, h('b', {}, '遥控中'), hudSpeed, h('span', { class: 'n-muted' }, '松开按键即停')));

  function say(text, level = '') { status.textContent = text; status.dataset.level = level; }
  function driveVector() {
    const axis = [0, 0, 0];
    for (const key of drive.held) DRIVE_KEYS[key].forEach((value, index) => { axis[index] += value; });
    const [forward, side, turn] = axis.map(value => Math.max(-1, Math.min(1, value)));
    if (!forward && !side && !turn) return null;
    return { vx: forward * speed.linear, vy: side * speed.linear, vyaw: turn * speed.angular };
  }
  async function sendDrive() {
    const vector = driveVector();
    const seq = ++drive.seq;
    try {
      const reply = await api.invoke('relative_move', { ...vector, duration_sec: HOLD_SEC });
      if (seq === drive.seq) drive.taskId = reply.task.task_id;
      // 回应到达前已经松开：这条命令立即取消。
      else if (!drive.held.size) api.invoke('cancel_task', { task_id: reply.task.task_id }).catch(() => {});
    } catch (error) {
      say('移动失败：' + error.message, 'error');
    }
  }
  function stopDrive() {
    clearInterval(drive.timer);
    drive.timer = null;
    drive.seq += 1;
    const taskId = drive.taskId;
    drive.taskId = null;
    if (taskId) api.invoke('cancel_task', { task_id: taskId }).catch(error => say('停止失败：' + error.message, 'error'));
  }
  function applyDrive() {
    for (const box of pads) for (const button of box.children) button.dataset.down = String(drive.held.has(button.dataset.code));
    const vector = driveVector();
    hudSpeed.textContent = vector ? '正在移动，' + speed.linear.toFixed(2) + ' 米/秒' : '等待按键';
    const key = vector ? [vector.vx, vector.vy, vector.vyaw].join() : '';
    if (key === drive.vectorKey) return;
    drive.vectorKey = key;
    if (!vector) { stopDrive(); say('已停止。'); return; }
    say('移动中：' + [...drive.held].map(item => KEYS.find(k => k[0] === item)[2]).join(' 加 '));
    clearInterval(drive.timer);
    sendDrive();
    drive.timer = setInterval(sendDrive, RENEW_MS);
  }
  function releaseAll() {
    if (!drive.held.size) return;
    drive.held.clear();
    applyDrive();
  }
  const typing = target => target instanceof HTMLElement && (target.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName));
  document.addEventListener('keydown', event => {
    const key = ARROWS[event.code] || event.code;
    if (!shown || !supported || !DRIVE_KEYS[key] || event.ctrlKey || event.metaKey || event.altKey || typing(event.target)) return;
    event.preventDefault();
    if (event.repeat || drive.held.has(key)) return;
    drive.held.add(key);
    applyDrive();
  });
  // 松开键在任何焦点下都处理，避免按住时点进输入框后停不下来。
  document.addEventListener('keyup', event => {
    const key = ARROWS[event.code] || event.code;
    if (drive.held.delete(key)) applyDrive();
  });
  window.addEventListener('blur', releaseAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
  api.capabilities().then(list => {
    supported = list.some(cap => cap.name === 'relative_move');
    unsupported.hidden = supported;
    for (const box of pads) box.toggleAttribute('data-disabled', !supported);
  }).catch(error => say('读取能力清单失败：' + error.message, 'error'));
  applyDrive();

  return {
    show() { shown = true; stage.hud(supported ? hud : null); },
    // 面板关闭即停（与原控制台关闭运动面板相同）。
    hide() { shown = false; releaseAll(); stage.hud(null); },
  };
}
