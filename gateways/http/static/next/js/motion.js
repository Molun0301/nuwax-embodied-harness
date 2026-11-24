/* 监控页动效：时长与缓动取 web/shared/neh-ui.css 的 --motion-fast/base/slow 与 --ease-out/in
   （系统设置“减少动态效果”时这些变量是 1 毫秒）。只动 transform 与 opacity；位姿、数值、地图重绘、视频等高频更新不加动画。 */
const root = document.documentElement;
const cssVar = name => getComputedStyle(root).getPropertyValue(name).trim();

/* speed：fast / base / slow；ease：out（进入）/ in（退出）。返回 Animation。 */
export function play(el, keyframes, { speed = 'base', ease = 'out' } = {}) {
  return el.animate(keyframes, { duration: parseFloat(cssVar('--motion-' + speed)) || 0, easing: cssVar('--ease-' + ease) });
}
/* 淡入。 */
export const fadeIn = (el, speed = 'base') => play(el, [{ opacity: 0 }, { opacity: 1 }], { speed });
/* 横向带方向进入：dir 为 1 从右侧进（前进、往后的页签），-1 从左侧进（返回、往前的页签），0 只淡入。 */
export function slideIn(el, dir, speed = 'base') {
  return play(el, [{ opacity: 0, transform: `translateX(${dir * 18}px)` }, { opacity: 1, transform: 'none' }], { speed });
}
/* 浮层出现：从下方 8 像素升起并淡入；from 为 -1 时从上方落下。 */
export function rise(el, from = 1, speed = 'base') {
  return play(el, [{ opacity: 0, transform: `translateY(${from * 8}px)` }, { opacity: 1, transform: 'none' }], { speed });
}
/* 浮层消失：淡出并移开 8 像素，结束后调用 done（被新的出现打断时返回的动画会被取消，done 不再调用）。 */
export function sink(el, done, from = 1, speed = 'fast') {
  const animation = play(el, [{ opacity: 1, transform: 'none' }, { opacity: 0, transform: `translateY(${from * 8}px)` }], { speed, ease: 'in' });
  animation.onfinish = done;
  return animation;
}
/* 位置或大小改变后的过渡（先记下 before = el.getBoundingClientRect()，改完布局再调用）：从旧位置与大小平移、缩放到新的。 */
export function flip(el, before, speed = 'base') {
  const after = el.getBoundingClientRect();
  if (!after.width || !after.height) return null;
  const dx = before.left - after.left, dy = before.top - after.top, sx = before.width / after.width, sy = before.height / after.height;
  if (Math.abs(dx) < 1 && Math.abs(dy) < 1 && Math.abs(sx - 1) < 0.01 && Math.abs(sy - 1) < 0.01) return null;
  return play(el, [{ transformOrigin: '0 0', transform: `translate(${dx}px, ${dy}px) scale(${sx}, ${sy})` }, { transformOrigin: '0 0', transform: 'none' }], { speed });
}
