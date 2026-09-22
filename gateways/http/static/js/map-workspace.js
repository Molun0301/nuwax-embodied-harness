/* 统一地图面板的互斥、关闭和键盘行为；业务状态仍由各功能拥有方维护。 */
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
    // 图例与工作面板各自有单一入口，避免展开后相互遮挡。
    const legend = document.getElementById('mapLegend');
    if (legend) legend.open = false;
  }
  window.nuwaxMapPanels = {activate, close, register(id, closer) { closers.set(id, closer); }};
  for (const [buttonId, panelId] of [['mapLayersToggle','mapLayersPanel'],['mapCameraToggle','mapCameraPanel']]) {
    document.getElementById(buttonId)?.addEventListener('click', () => {
      const panel = document.getElementById(panelId);
      if (!panel.hidden) { close(panelId, true); return; }
      activate(panelId); panel.hidden = false; sync();
    });
  }
  root.addEventListener('click', event => {
    const button = event.target.closest('[data-map-panel-close]');
    if (button) close(button.dataset.mapPanelClose, true);
  });
  root.addEventListener('keydown', event => {
    if (event.key !== 'Escape' || document.querySelector('dialog[open]')) return;
    const active = panels.find(panel => !panel.hidden);
    if (active) { event.preventDefault(); close(active.id, true); }
  });
  // 位于各模块外的关闭行为只撤下界面，不取消已受理的导航或识别任务。
  document.querySelectorAll('[data-view-target]:not([data-view-target="monitor-view"])').forEach(button => {
    button.addEventListener('click', () => panels.forEach(panel => close(panel.id)));
  });
  window.addEventListener('map:view-changed', () => close('sceneGraphPanel'));
  const observer = new MutationObserver(sync);
  panels.forEach(panel => observer.observe(panel, {attributes:true, attributeFilter:['hidden']}));
  document.getElementById('mapLegend')?.addEventListener('toggle', () => {
    if (document.getElementById('mapLegend').open) panels.forEach(panel => close(panel.id));
  });
  sync();
})();
