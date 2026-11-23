/* 地图与视频的帧率、延迟。从原控制台 console.js 迁出（recordMediaFrame、mediaMetricText），口径不变：
   - 帧率只统计被接受的新帧（重绘、心跳、同一缓存重复应用不计），取最近 5 秒，最后一帧超过 2 秒即为 0；
   - 帧时间来自设备时钟，只有估算出网关与浏览器的时钟差后才算延迟，不直接跨机器相减。 */

export function createMetrics(store) {
  const media = {};
  function record(kind, sourceTime, frameKey, sourceKey = '') {
    let metric = media[kind];
    if (!metric || metric.sourceKey !== sourceKey) metric = media[kind] = { samples: [], sourceKey, frameKey: null, sourceTime: null };
    if (frameKey != null && metric.frameKey === frameKey) return;
    const now = performance.now();
    metric.samples = metric.samples.filter(time => now - time <= 5000).slice(-599);
    metric.samples.push(now);
    metric.frameKey = frameKey;
    metric.sourceTime = Number.isFinite(sourceTime) && sourceTime > 0 ? sourceTime : null;
    metric.displayedAt = Date.now();
  }
  /* 每秒帧数；没有记录时为 null。 */
  function rate(kind, monotonicNow = performance.now()) {
    const metric = media[kind];
    if (!metric) return null;
    metric.samples = metric.samples.filter(time => monotonicNow - time <= 5000);
    const samples = metric.samples;
    const span = samples.length > 1 ? samples[samples.length - 1] - samples[0] : 0;
    return span > 0 && monotonicNow - samples[samples.length - 1] <= 2000 ? (samples.length - 1) * 1000 / span : 0;
  }
  /* 采集到显示的延迟（毫秒）；没有帧时间或尚未估算时钟差时为 null。 */
  function delay(kind) {
    const metric = media[kind];
    const offset = store.get().clockOffsetMs;
    if (!metric || metric.sourceTime === null || offset == null) return null;
    const age = metric.displayedAt - metric.sourceTime + offset;
    return Number.isFinite(age) && age >= 0 ? age : null;
  }
  return { record, rate, delay };
}

export function fmtRate(value) { return value == null ? '--' : value.toFixed(1) + ' 帧/秒'; }
export function fmtDelay(ms) { return ms == null ? '--' : '≈' + Math.round(ms) + ' 毫秒'; }
