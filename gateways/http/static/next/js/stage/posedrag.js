/* 按下拖动定位姿：按下处是位置，拖动方向是朝向。导航选点（双击的第二下）与地图卡“指定位置后重定位”共用。
   二维按地图反投影取点；三维按下处取点云上的地图点，拖动时光标投到按下点所在的水平面（与 pointcloud3d.js 的导航选点相同）。
   拖动不超过 6 像素不算定了朝向。 */
export const CLICK_PIXELS = 6;

/* view：'3d' 用点云拾取，其它用二维地图。没有地图点时返回 null。 */
export function poseAt(view, event) {
  return (view === '3d' ? window.pointcloud3d?.pickNavigationPoint(event.clientX, event.clientY)
    : window.__nuwaxMapUnproject?.(event.clientX, event.clientY)) || null;
}

/* 从按下事件 down 开始跟随这一次拖动：onYaw(yaw) 朝向变化时调用；onEnd(yaw) 松开时调用，没拖出朝向时 yaw 为 undefined。 */
export function trackPoseDrag(el, down, point, view, { onYaw, onEnd }) {
  let yaw;
  el.setPointerCapture(down.pointerId);
  const move = event => {
    if (event.pointerId !== down.pointerId) return;
    if (yaw === undefined && Math.hypot(event.clientX - down.clientX, event.clientY - down.clientY) <= CLICK_PIXELS) return;
    const target = view === '3d' ? window.pointcloud3d?.boundaryPointAt(event.clientX, event.clientY, point)
      : window.__nuwaxMapUnproject?.(event.clientX, event.clientY);
    if (!target) return;
    yaw = Math.atan2(target.y - point.y, target.x - point.x);
    onYaw(yaw);
  };
  const end = event => {
    if (event.pointerId !== down.pointerId) return;
    el.removeEventListener('pointermove', move);
    el.removeEventListener('pointerup', end);
    el.removeEventListener('pointercancel', end);
    onEnd(yaw);
  };
  el.addEventListener('pointermove', move);
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
}
