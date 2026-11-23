/* 画中画视频：从原控制台 console.js 迁出（startVideoStream、consumeMjpegStream、extractNextJpeg、
   renderDecodedVideoFrame、updateVideoDetections、drawVideoDetections、感知运行开关），行为不变：
   - 页面加载即连 /streams/video.mjpeg；503 rust_bridge_no_frames 时等待重连（0.5 秒起每次 ×1.6，最长 5 秒）；
     video.frame 主题不决定是否连接；
   - 按传输头 Content-Length 切帧（内嵌缩略图的结束标记不能截断主图），读帧头的相机、时间、序号；
   - 视频状态只看浏览器 3 秒内是否收到新帧，不拿设备帧时间与浏览器时钟比；
   - 检测框按帧头与 video.detections 的相机、帧号、像素尺寸对齐，同相机乱序旧结果不覆盖新结果；
   - 识别开关调用 /api/perception/runtime/start|stop，按是否有手动原因显示；detection_enabled 为假时不可开启并说明去设置开启。 */

const DEFAULT_VIDEO_STREAM_URL = '/streams/video.mjpeg';
const VIDEO_STREAM_INITIAL_RECONNECT_MS = 500;
const VIDEO_STREAM_MAX_RECONNECT_MS = 5000;
const VIDEO_JPEG_DECODE_TIMEOUT_MS = 3000;
const MJPEG_MAX_BUFFER_BYTES = 16 * 1024 * 1024;
const VIDEO_DISPLAY_FRESH_MS = 3000;

export function videoSourceTime(value) {
  if (typeof value !== 'string' || !value.trim()) return null;
  const parsed = Date.parse(value);
  return Number.isFinite(parsed) ? parsed : null;
}
function videoFrameIdentity(data) {
  return {
    cameraId: typeof data?.camera_id === 'string' ? data.camera_id : '',
    frameId: typeof data?.frame_id === 'string' ? data.frame_id : '',
    sourceTime: videoSourceTime(data?.frame_timestamp),
    timestampSource: data?.frame_timestamp_source || '',
    sequence: data?.frame_sequence ?? data?.sequence ?? null,
  };
}
export function fitLabel(ctx, label, maxWidth) {
  if (ctx.measureText(label).width <= maxWidth) return label;
  let fitted = '';
  for (const character of Array.from(label)) {
    if (ctx.measureText(`${fitted}${character}…`).width > maxWidth) break;
    fitted += character;
  }
  return fitted ? `${fitted}…` : '…';
}
function formatDetectionScore(score) {
  if (typeof score !== 'number' || !Number.isFinite(score) || score < 0 || score > 1) return '分数无效';
  if (score === 1) return '100%';
  if (score >= 0.999) return '≥99.9%';
  return `${(score * 100).toFixed(1)}%`;
}

/* media：{root（承载画面的容器）, image, overlay（检测框画布）}；visible() 为画面当前是否可见（页面、视图、画中画未收起）。
   onChange 在状态、帧率或识别开关变化时调用，由舞台刷新画中画外框。 */
export function createVideo({ stream, api, metrics, media, visible, onChange }) {
  const state = {
    src: null, controller: null, generation: 0, reconnectTimer: null, reconnectDelayMs: VIDEO_STREAM_INITIAL_RECONNECT_MS,
    status: 'idle', message: '', detail: '', bytes: 0, frames: 0, lastFrameAt: null, objectUrl: null, sourceSize: null,
    displayed: null, detectionFrame: null, detections: [], detectionSignature: '', geometrySignature: '', detectionRevision: 0,
    layoutRevision: 0, drawFrame: null, drawKey: '', showDetections: true,
    perception: null, perceptionBusy: false, perceptionLoad: null, detectionEnabled: null, perceptionError: '',
  };
  const { image, overlay } = media;

  function setStatus(status, message, detail = '') {
    state.status = status;
    state.message = message || '';
    state.detail = detail;
    onChange();
    scheduleDetectionDraw();
  }
  /* 画中画外框读取的摘要：live 有新画面、stale 画面中断保留上次画面、none 还没有画面。 */
  function summary(now = Date.now()) {
    const hasFrame = Boolean(state.lastFrameAt && !image.hidden);
    const fresh = hasFrame && now - state.lastFrameAt <= VIDEO_DISPLAY_FRESH_MS;
    return { mode: fresh ? 'live' : hasFrame ? 'stale' : 'none', message: state.message, detail: state.detail,
      cameraId: state.displayed?.cameraId || '', rate: hasFrame ? metrics.rate('video') : null, delay: hasFrame ? metrics.delay('video') : null,
      relayTime: state.displayed?.timestampSource === 'video_relay.write_time' };
  }

  function scheduleReconnect(reason) {
    if (!state.src || state.reconnectTimer) return;
    const delayMs = state.reconnectDelayMs;
    const nextDelayMs = Math.min(VIDEO_STREAM_MAX_RECONNECT_MS, Math.max(VIDEO_STREAM_INITIAL_RECONNECT_MS, delayMs * 1.6));
    setStatus('reconnecting', '画面中断，' + (delayMs / 1000).toFixed(1) + ' 秒后重连', reason);
    state.reconnectTimer = setTimeout(() => {
      state.reconnectTimer = null;
      state.reconnectDelayMs = nextDelayMs;
      start({ force: true, reason: 'reconnect' });
    }, delayMs);
  }
  function start(options = {}) {
    const videoSrc = new URL(DEFAULT_VIDEO_STREAM_URL, window.location.origin).toString();
    if (!options.force && state.src === videoSrc && state.controller) return;
    const reconnectDelay = options.reason === 'reconnect' ? state.reconnectDelayMs : VIDEO_STREAM_INITIAL_RECONNECT_MS;
    state.controller?.abort();
    state.controller = null;
    if (state.reconnectTimer) { clearTimeout(state.reconnectTimer); state.reconnectTimer = null; }
    state.reconnectDelayMs = reconnectDelay;
    state.generation += 1;
    state.src = videoSrc;
    const generation = state.generation;
    const controller = new AbortController();
    state.controller = controller;
    state.bytes = 0;
    setStatus('connecting', '正在连接摄像头画面');
    consume(videoSrc, controller.signal, generation).catch(error => {
      if (generation !== state.generation || controller.signal.aborted) return;
      state.controller = null;
      if (error?.code === 'video_not_ready') {
        scheduleReconnect('等待摄像头视频帧');
        setStatus('pending', '没有收到画面，正在等待视频连接', error.message);
        return;
      }
      scheduleReconnect('视频流连接失败：' + (error?.message || String(error || '未知错误')));
    });
  }
  async function consume(videoSrc, signal, generation) {
    const response = await fetch(videoSrc, { method: 'GET', cache: 'no-store', signal });
    if (!response.ok) {
      let detail = null;
      try { detail = await response.json(); } catch (_) { /* 非结构化错误保留响应状态 */ }
      if (response.status === 503 && detail?.reason === 'rust_bridge_no_frames') {
        const waiting = new Error(detail.message || '等待摄像头视频帧');
        waiting.code = 'video_not_ready';
        throw waiting;
      }
      const error = new Error(detail?.message || `视频服务返回 ${response.status}`);
      error.code = detail?.error || 'video_stream_error';
      throw error;
    }
    state.reconnectDelayMs = VIDEO_STREAM_INITIAL_RECONNECT_MS;
    setStatus('streaming', '已连接，等待第一帧画面');
    const reader = response.body.getReader();
    let buffer = new Uint8Array(0);
    try {
      while (generation === state.generation) {
        const result = await reader.read();
        if (result.done) break;
        if (!result.value || !result.value.byteLength) continue;
        state.bytes += result.value.byteLength;
        buffer = appendBytes(buffer, result.value);
        if (buffer.byteLength > MJPEG_MAX_BUFFER_BYTES) {
          const soi = findMarker(buffer, 0xff, 0xd8, Math.max(0, buffer.byteLength - 1024 * 1024));
          buffer = soi >= 0 ? buffer.slice(soi) : new Uint8Array(0);
          if (!buffer.byteLength) throw new Error('MJPEG 缓冲区超过上限且未找到 JPEG 起始标记。');
        }
        let parsedAny = false, latestBytes = null, latestIdentity = null;
        while (true) {
          const parsed = extractNextJpeg(buffer);
          if (!parsed) break;
          if (!parsed.frame) { buffer = parsed.remaining; break; }
          parsedAny = true;
          latestBytes = parsed.frame;
          latestIdentity = parsed.identity;
          buffer = parsed.remaining;
        }
        if (parsedAny) buffer = buffer.slice();
        if (latestBytes) {
          try {
            await renderFrame(latestBytes.slice(), generation, latestIdentity || {});
          } catch (error) {
            if (generation !== state.generation || signal.aborted) return;
            state.controller = null;
            scheduleReconnect('JPEG 帧解码失败：' + (error?.message || String(error || '未知错误')));
            return;
          }
        }
      }
    } finally {
      // 解码失败或换源提前退出时也取消字节读取，避免失去旧连接的拥有方。
      try { await reader.cancel(); } catch (_) { /* 已中止或已结束 */ }
      try { reader.releaseLock(); } catch (_) { /* 已释放 */ }
    }
    if (generation === state.generation) {
      state.controller = null;
      scheduleReconnect('MJPEG 字节流已结束');
    }
  }
  function appendBytes(left, right) {
    if (!left || !left.byteLength) return right;
    const merged = new Uint8Array(left.byteLength + right.byteLength);
    merged.set(left, 0);
    merged.set(right, left.byteLength);
    return merged;
  }
  function findMarker(bytes, a, b, start) {
    for (let i = Math.max(0, Number(start || 0)); i < bytes.byteLength - 1; i += 1) if (bytes[i] === a && bytes[i + 1] === b) return i;
    return -1;
  }
  function extractNextJpeg(buffer) {
    const soi = findMarker(buffer, 0xff, 0xd8, 0);
    if (soi < 0) return { frame: null, remaining: buffer.byteLength > 16384 ? buffer.slice(buffer.byteLength - 16384) : buffer };
    const prefix = new TextDecoder().decode(buffer.subarray(Math.max(0, soi - 16384), soi));
    const boundary = prefix.lastIndexOf('--frame\r\n');
    const header = boundary >= 0 ? prefix.slice(boundary) : prefix;
    const lengthHeader = header.match(/^Content-Length:\s*([0-9]+)\s*$/mi)?.[1];
    const length = lengthHeader ? Number(lengthHeader) : null;
    if (length !== null && (!Number.isSafeInteger(length) || length < 4 || length > MJPEG_MAX_BUFFER_BYTES)) throw new Error('视频帧长度超出有效范围。');
    // 有长度头时以传输边界为准，图像元数据中的内嵌缩略图结束标记不能截断主图。
    const eoi = length !== null ? soi + length - 2 : findMarker(buffer, 0xff, 0xd9, soi + 2);
    if (eoi < 0 || eoi + 2 > buffer.byteLength) return { frame: null, remaining: soi > 16384 ? buffer.slice(soi - 16384) : buffer };
    const frame = buffer.subarray(soi, eoi + 2);
    const remaining = buffer.subarray(eoi + 2);
    if (!frame.byteLength) return null;
    const cameraValue = header.match(/^X-Camera-Id:\s*([^\r\n]*)/mi)?.[1] || '';
    let cameraId = '';
    try { cameraId = decodeURIComponent(cameraValue.trim()); } catch (_) { /* 无效头不作为来源依据 */ }
    const timestamp = header.match(/^X-Frame-Timestamp:\s*([^\r\n]*)/mi)?.[1]?.trim();
    const sequence = header.match(/^X-Frame-Sequence:\s*([0-9]+)/mi)?.[1] || null;
    const timestampSource = header.match(/^X-Frame-Timestamp-Source:\s*([^\r\n]*)/mi)?.[1]?.trim() || '';
    return { frame, remaining, identity: { cameraId, sourceTime: videoSourceTime(timestamp), timestampSource, sequence } };
  }
  async function renderFrame(frameBytes, generation, identity) {
    if (!frameBytes || !frameBytes.byteLength) return;
    const blob = new Blob([frameBytes], { type: 'image/jpeg' });
    if (generation !== state.generation) return;
    const nextUrl = await displayBlob(blob, generation);
    if (generation !== state.generation) { URL.revokeObjectURL(nextUrl); return; }
    state.objectUrl = nextUrl;
    state.frames += 1;
    state.lastFrameAt = Date.now();
    state.displayed = { ...identity, receivedAt: state.lastFrameAt };
    metrics.record('video', identity.sourceTime, state.frames, `${state.generation}|${identity.cameraId || ''}`);
    state.sourceSize = { width: image.naturalWidth, height: image.naturalHeight };
    const first = image.hidden;
    state.status = 'rendering';
    image.hidden = false;
    if (first) onChange();
    scheduleDetectionDraw();
  }
  async function displayBlob(blob, generation) {
    const nextUrl = URL.createObjectURL(blob);
    const previousUrl = state.objectUrl;
    try {
      await new Promise((resolve, reject) => {
        let settled = false;
        const timer = setTimeout(() => { if (settled) return; settled = true; cleanup(); reject(new Error('JPEG 显示超时')); }, VIDEO_JPEG_DECODE_TIMEOUT_MS);
        function cleanup() { clearTimeout(timer); image.removeEventListener('load', onLoad); image.removeEventListener('error', onError); }
        function onLoad() { if (settled) return; settled = true; cleanup(); resolve(); }
        function onError() { if (settled) return; settled = true; cleanup(); reject(new Error('JPEG 显示失败')); }
        image.addEventListener('load', onLoad, { once: true });
        image.addEventListener('error', onError, { once: true });
        if (generation !== state.generation) { settled = true; cleanup(); resolve(); return; }
        image.src = nextUrl;
      });
    } catch (error) {
      URL.revokeObjectURL(nextUrl);
      throw error;
    }
    if (generation !== state.generation) return nextUrl;
    // 新图已完成加载，旧对象地址可立即释放。
    if (previousUrl) URL.revokeObjectURL(previousUrl);
    return nextUrl;
  }

  /* ---- 检测框 ---- */
  function updateDetections(frame) {
    const data = frame.payload || {};
    const identity = videoFrameIdentity(data);
    const previous = state.detectionFrame;
    // 同相机的乱序结果不能覆盖更新的观察；其它相机的结果保留明确不匹配状态。
    if (previous?.identity.cameraId === identity.cameraId && identity.sourceTime !== null
      && previous.identity.sourceTime !== null && identity.sourceTime < previous.identity.sourceTime) return;
    const detections = Array.isArray(data.detections_2d) ? data.detections_2d : [];
    const valid = detections.filter(detection => {
      const bbox = detection && detection.bbox;
      return typeof detection?.label === 'string' && !detection.label.startsWith('<truncated:')
        && Number.isFinite(detection.score) && detection.score >= 0 && detection.score <= 1 && bbox
        && [bbox.x_px, bbox.y_px, bbox.width_px, bbox.height_px].every(Number.isFinite)
        && bbox.width_px > 0 && bbox.height_px > 0;
    }).slice(0, 64);
    const status = String(frame.status || 'missing');
    const failed = data.available === false || ['missing', 'failed', 'error', 'unavailable'].includes(status)
      || ['failed', 'error', 'unavailable'].includes(data.stage_status?.detector);
    const geometrySignature = JSON.stringify([valid, identity.cameraId, data.width_px, data.height_px]);
    if (geometrySignature !== state.geometrySignature) {
      state.geometrySignature = geometrySignature;
      state.detectionRevision += 1;
      state.detections = valid;
    }
    state.detectionFrame = { identity, width: data.width_px, height: data.height_px, failed, backend: data.detector_backend || '' };
    scheduleDetectionDraw();
  }
  function displayState() {
    const frame = state.detectionFrame;
    if (image.hidden || !state.sourceSize) return { key: 'no_video', drawable: false };
    if (!frame) return { key: 'waiting', drawable: false };
    const shown = state.displayed;
    // 慢推理继续显示最近有效框；只拒绝已明确的相机或像素尺寸不匹配。
    if (shown?.cameraId && frame.identity.cameraId && frame.identity.cameraId !== shown.cameraId) return { key: 'camera_mismatch', drawable: false };
    if (frame.identity.frameId && shown?.frameId && frame.identity.frameId !== shown.frameId) return { key: 'frame_mismatch', drawable: false };
    const size = state.sourceSize;
    if ((frame.width && size.width !== frame.width) || (frame.height && size.height !== frame.height)) return { key: 'size_mismatch', drawable: false };
    if (state.detections.length) return { key: 'ready', drawable: true };
    return { key: frame.failed ? 'error' : 'empty', drawable: false };
  }
  function scheduleDetectionDraw() {
    if (!visible() || state.drawFrame) return;
    state.drawFrame = requestAnimationFrame(() => { state.drawFrame = null; drawDetections(); });
  }
  function drawDetections() {
    if (!visible()) return;
    const display = displayState();
    const source = state.sourceSize;
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    const drawKey = `${state.detectionRevision}|${state.layoutRevision}|${state.showDetections}|${display.drawable}|${source?.width}|${source?.height}|${ratio}`;
    if (state.drawKey === drawKey) return;
    state.drawKey = drawKey;
    const context = overlay.getContext('2d');
    if (!state.showDetections || image.hidden || !display.drawable || !state.detections.length) {
      overlay.hidden = true;
      context.clearRect(0, 0, overlay.width, overlay.height);
      return;
    }
    const rect = overlay.parentElement.getBoundingClientRect();
    const width = Math.round(rect.width), height = Math.round(rect.height);
    if (!(width > 0 && height > 0)) { overlay.hidden = true; state.drawKey = ''; return; }
    const pixelWidth = Math.round(width * ratio), pixelHeight = Math.round(height * ratio);
    if (overlay.width !== pixelWidth || overlay.height !== pixelHeight) { overlay.width = pixelWidth; overlay.height = pixelHeight; }
    // 画面按 contain 缩放居中，检测框用同一几何换算。
    const scale = Math.min(width / source.width, height / source.height);
    const offsetX = (width - source.width * scale) / 2, offsetY = (height - source.height * scale) / 2;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    context.lineWidth = 2;
    context.font = '12px sans-serif';
    for (const detection of state.detections) {
      const bbox = detection.bbox;
      const left = Math.max(0, bbox.x_px), top = Math.max(0, bbox.y_px);
      const right = Math.min(source.width, bbox.x_px + bbox.width_px);
      const bottom = Math.min(source.height, bbox.y_px + bbox.height_px);
      if (!(right > left && bottom > top)) continue;
      const x = offsetX + left * scale, y = offsetY + top * scale;
      const boxWidth = (right - left) * scale, boxHeight = (bottom - top) * scale;
      const selfBody = detection.self_body === true || detection.attributes?.self_body === true;
      const label = `${selfBody ? '机器人自身' : detection.label} ${formatDetectionScore(detection.score)}`;
      const fitted = fitLabel(context, label, Math.max(20, Math.min(240, width - x - 10)));
      const labelWidth = Math.min(width - x, context.measureText(fitted).width + 10);
      const labelTop = Math.max(offsetY, y - 20);
      context.strokeStyle = selfBody ? '#94a3b8' : '#22d3ee';
      context.strokeRect(x, y, boxWidth, boxHeight);
      context.fillStyle = 'rgba(2, 6, 23, 0.88)';
      context.fillRect(x, labelTop, labelWidth, 20);
      context.fillStyle = selfBody ? '#cbd5e1' : '#e0f2fe';
      context.fillText(fitted, x + 5, labelTop + 14);
    }
    overlay.hidden = false;
  }

  /* ---- 识别开关（感知运行） ---- */
  function loadPerception() {
    if (state.perceptionBusy) return Promise.resolve();
    if (state.perceptionLoad) return state.perceptionLoad;
    state.perceptionLoad = api.json('/api/perception/runtime')
      .then(result => { state.detectionEnabled = result.detection_enabled; state.perception = result.perception_runtime; state.perceptionError = ''; })
      .catch(error => { state.perceptionError = '读取识别状态失败：' + error.message; })
      .finally(() => { state.perceptionLoad = null; onChange(); });
    return state.perceptionLoad;
  }
  async function setManualPerception(enabled) {
    state.perceptionBusy = true;
    onChange();
    try {
      // 先完成在途读取，避免旧查询响应覆盖动作返回的真实原因集合。
      await state.perceptionLoad;
      const result = await api.json('/api/perception/runtime/' + (enabled ? 'start' : 'stop'),
        { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
      state.perception = result.perception_runtime;
      state.perceptionError = '';
    } catch (error) {
      state.perceptionError = (enabled ? '开启' : '关闭') + '识别失败：' + error.message;
    } finally {
      state.perceptionBusy = false;
      onChange();
    }
  }
  /* 识别开关的显示：on 为当前有手动使用原因；available 为假时没有检测后端（本次启动未开启视觉识别）。 */
  function perception() {
    const manual = Boolean(state.perception?.reasons?.includes('manual'));
    return { known: Boolean(state.perception), on: manual, busy: state.perceptionBusy, available: state.detectionEnabled !== false, error: state.perceptionError };
  }

  stream.on('video.detections', updateDetections);
  new ResizeObserver(() => { state.layoutRevision += 1; scheduleDetectionDraw(); }).observe(media.root);
  setInterval(() => { if (visible()) void loadPerception(); }, 3000);
  setInterval(() => { if (visible()) { onChange(); scheduleDetectionDraw(); } }, 500);
  document.addEventListener('visibilitychange', () => { if (!document.hidden) { onChange(); scheduleDetectionDraw(); } });
  // 调用方拿到返回值之后再开始连接（连接状态变化会回调 onChange）。
  queueMicrotask(() => { start(); void loadPerception(); });

  return {
    summary,
    perception,
    togglePerception() { void setManualPerception(!perception().on); },
    setShowDetections(value) { state.showDetections = value; scheduleDetectionDraw(); },
    get showDetections() { return state.showDetections; },
    refresh() { state.layoutRevision += 1; void loadPerception(); scheduleDetectionDraw(); },
  };
}
