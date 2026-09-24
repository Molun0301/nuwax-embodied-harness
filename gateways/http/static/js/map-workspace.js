/* 统一地图工具二级栏的互斥、关闭和键盘行为；业务状态仍由各功能拥有方维护。 */
(function () {
  'use strict';
  const root = document.getElementById('live-map');
  if (!root) return;
  const closers = new Map();
  const panels = [...root.querySelectorAll('.map-drawer')];
  const trigger = id => root.querySelector(`[aria-controls="${id}"]`);
  function sync() {
    const active = panels.find(panel => !panel.hidden);
    root.dataset.mapPanel = active?.id || '';
    for (const panel of panels) {
      const button = trigger(panel.id);
      button?.setAttribute('aria-expanded', String(!panel.hidden));
      button?.classList.toggle('active', !panel.hidden);
    }
  }
  function close(id, restoreFocus = false) {
    const panel = document.getElementById(id);
    if (!panel || panel.hidden) return;
    const hadFocus = panel.contains(document.activeElement);
    const closer = closers.get(id);
    if (closer) closer(); else panel.hidden = true;
    sync();
    if (restoreFocus || hadFocus) trigger(id)?.focus({preventScroll: true});
  }
  function activate(id) {
    for (const panel of panels) if (panel.id !== id) close(panel.id);
  }
  window.nuwaxMapPanels = {activate, close, register(id, closer) { closers.set(id, closer); }};
  document.getElementById('mapLayersToggle')?.addEventListener('click', () => {
    const panel = document.getElementById('mapLayersPanel');
    if (!panel.hidden) { close(panel.id, true); return; }
    activate(panel.id); panel.hidden = false; sync();
  });
  root.addEventListener('click', event => {
    const button = event.target.closest('[data-map-panel-close]');
    if (button) close(button.dataset.mapPanelClose, true);
  });
  // Esc收起当前工具：焦点在地图画布或页面上时同样生效；绘制中的Esc仍由禁行区撤销草稿。
  document.addEventListener('keydown', event => {
    if (event.key !== 'Escape' || event.defaultPrevented || document.querySelector('dialog[open]')) return;
    if (window.__nuwaxBoundaryActive || !root.offsetParent) return;
    const target = event.target;
    if (target instanceof HTMLInputElement && target.type === 'search' && target.value) return;
    if (target !== document.body && !root.contains(target)) return;
    const active = panels.find(panel => !panel.hidden);
    if (active) { event.preventDefault(); close(active.id, true); }
  });
  // 标签条只有一行，纵向滚轮转为横向滚动。
  root.addEventListener('wheel', event => {
    const strip = event.target.closest('.msb-chips');
    if (!strip || Math.abs(event.deltaX) > Math.abs(event.deltaY) || strip.scrollWidth <= strip.clientWidth) return;
    strip.scrollLeft += event.deltaY;
    event.preventDefault();
  }, {passive: false});
  // 位于各模块外的关闭行为只撤下界面，不取消已受理的导航或识别任务。
  document.querySelectorAll('[data-view-target]:not([data-view-target="monitor-view"])').forEach(button => {
    button.addEventListener('click', () => panels.forEach(panel => close(panel.id)));
  });
  window.addEventListener('map:view-changed', () => close('sceneGraphPanel'));
  // 二级栏浮在地图顶部；把实际高度交给画布浮层（提示、检查卡、关系图）让位，画布本身尺寸不变。
  const subbar = document.getElementById('mapSubbar');
  if (subbar && window.ResizeObserver) {
    new ResizeObserver(() => root.style.setProperty('--map-subbar-h', `${subbar.offsetHeight}px`)).observe(subbar);
  }
  const observer = new MutationObserver(sync);
  panels.forEach(panel => observer.observe(panel, {attributes:true, attributeFilter:['hidden']}));
  sync();
})();
