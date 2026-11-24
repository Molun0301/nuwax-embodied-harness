/* 设备管理恢复页：业务没运行时（控制台打不开）在这里启动业务、更新、回退、调整模块和修改配置；未安装时整页是首次安装向导。
   与控制台“系统”“设置”页挂载同一组共用模块（/manager-assets/shared/，管理包自带的副本）。地址 #/system、#/settings。 */
import { icon, h, setTheme, currentTheme, enter, fadeOnChange } from '/manager-assets/shared/neh-ui.js';
import { mountSystem, mountWizard, managerRequest } from '/manager-assets/shared/system-pages.js';
import { mount as mountSettings } from '/manager-assets/shared/settings-editor.js';

// 新控制台的预览入口；切换到正式入口后改为 /console。
const CONSOLE_URL = '/console-next';
const BRAND = '<svg viewBox="0 0 64 64" aria-hidden="true"><rect width="64" height="64" rx="16"/><path d="M18 46V18l28 28V18"/></svg>';
const app = document.getElementById('app');

function themeBtn() {
  const btn = h('button', { class: 'n-btn n-btn--quiet n-btn--icon', type: 'button', 'aria-label': '切换亮色或暗色', title: '切换亮色或暗色' });
  const sync = () => { btn.innerHTML = icon(currentTheme() === 'dark' ? 'sun' : 'moon'); };
  btn.addEventListener('click', () => { setTheme(currentTheme() === 'dark' ? 'light' : 'dark'); sync(); });
  sync();
  return btn;
}
const top = (...kids) => h('header', { class: 'r-top' }, h('span', { class: 'r-brand', html: BRAND }), h('span', { class: 'r-title' }, 'NEH 设备管理'), ...kids, themeBtn());

/* ---------- 已安装：系统、设置两个页签 ---------- */
function mountInstalled() {
  const status = h('div', { class: 'r-status', role: 'status' });
  const tabs = h('nav', { class: 'r-tabs', role: 'tablist', 'aria-label': '页面', 'data-ind': 'underline' },
    [['system', '系统'], ['settings', '设置']].map(([id, label]) => h('button', { type: 'button', role: 'tab', dataset: { tab: id }, onclick: () => { location.hash = '#/' + id; } }, label)));
  const notice = h('div', { class: 'r-notice', hidden: true, html: icon('info', 'n-icon--sm') },
    h('span', {}, '业务正在运行，日常管理请到控制台的系统页。这里在业务没运行时用于启动、更新、回退和修改配置。'),
    h('a', { class: 'n-btn n-btn--sm', href: CONSOLE_URL, html: icon('external-link', 'n-icon--sm') }, '打开控制台'));
  const sysView = h('div', { class: 'r-view' });
  const setView = h('div', { class: 'r-view', hidden: true });
  app.replaceChildren(h('div', { class: 'r-app' }, top(tabs, h('span', { class: 'r-spacer' }), status), notice, h('main', { class: 'r-main' }, sysView, setView)));
  const sys = mountSystem(sysView, {
    mode: 'recovery', request: managerRequest,
    openSettings: () => { location.hash = '#/settings'; },
    onState(state) {
      status.replaceChildren(state.tone === 'busy' ? h('span', { class: 'n-spin' }) : h('span', { class: 'n-dot n-dot--' + (state.tone === 'off' ? 'off' : state.tone) }), h('span', {}, state.text));
      notice.hidden = state.code !== 'running';
    },
  });
  // 系统页一直读业务状态，顶栏的一句话状态在设置页签也保持最新；文字变化时淡入。
  sys.show();
  fadeOnChange(status);
  let settings = null, shown = '';
  function showTab() {
    const id = location.hash.replace(/^#\/?/, '') === 'settings' ? 'settings' : 'system';
    tabs.querySelectorAll('button').forEach(b => b.setAttribute('aria-selected', String(b.dataset.tab === id)));
    sysView.hidden = id !== 'system';
    setView.hidden = id !== 'settings';
    // 切换页签时新视图淡入并上移几像素；首次打开不做。
    if (shown && shown !== id) enter(id === 'system' ? sysView : setView, { from: 'translateY(6px)', speed: 'slow' });
    shown = id;
    if (id === 'settings') {
      settings = settings || mountSettings(setView, { title: '设置', request: managerRequest, onRestart: () => { location.hash = '#/system'; void sys.refresh(); } });
      void settings.load();
    }
  }
  window.addEventListener('hashchange', showTab);
  showTab();
}

/* ---------- 未安装：整页安装向导 ---------- */
function mountSetup() {
  const main = h('main', { class: 'r-main' });
  app.replaceChildren(h('div', { class: 'r-app r-app--setup' }, top(h('span', { class: 'r-spacer' })), main));
  mountWizard(main, { request: managerRequest, consoleUrl: CONSOLE_URL, onDone: () => location.reload() });
}

async function boot() {
  app.replaceChildren(h('div', { class: 'r-boot' }, h('div', { class: 'n-empty' }, h('span', { class: 'n-spin' }), h('div', {}, '正在读取设备状态…'))));
  try {
    const status = await managerRequest('status');
    if (status.installation?.installed) mountInstalled(); else mountSetup();
  } catch (error) {
    app.replaceChildren(h('div', { class: 'r-boot' }, h('div', { class: 'n-empty', html: icon('circle-alert') }, h('div', { class: 'n-empty-title' }, '读不到设备状态'),
      h('div', {}, error.message), h('button', { class: 'n-btn n-btn--sm', type: 'button', onclick: boot }, '重新读取'))));
  }
}
void boot();
