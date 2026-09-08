(() => {
  "use strict";

  const CACHE_LIMIT = 5000;
  const PAGE_SIZE = 200;
  const LEVEL_NAMES = { trace: "跟踪", debug: "调试", info: "信息", warning: "警告", error: "错误", critical: "严重" };
  const SOURCE_NAMES = { python: "平台服务", rust: "Rust 运行时", native: "原生组件" };
  const filterIds = { level: "logLevelFilter", source: "logSourceFilter", component: "logComponentFilter", query: "logSearch", request_id: "logRequestFilter", task_id: "logTaskFilter" };
  const records = new Map();
  const rowNodes = new Map();
  const recentRowNodes = new Map();
  const recentState = { entries: [], count: 0, frame: null, pointerActive: false, dirty: false };
  const state = { displayLimit: PAGE_SIZE, visible: [], paused: false, dirty: false, pending: 0, nextBefore: null, hasMore: false, loading: false, opened: false, request: 0, serial: 0, historicalSerial: 0, renderTimer: null, filterTimer: null, gapNotice: "", gapReloadPromise: null };
  const element = (id) => document.getElementById(id);
  const write = (id, value) => { const node = element(id); if (node) node.textContent = String(value); };
  const message = (text) => write("logMessage", state.gapNotice ? `${text} ${state.gapNotice}` : text);

  function filters() {
    return Object.fromEntries(Object.entries(filterIds).map(([key, id]) => [key, String(element(id)?.value || "").trim()]));
  }

  function normalize(input, historical) {
    const raw = input && typeof input === "object" ? (input.payload || input) : {};
    const details = raw.details || raw.details_compact || {};
    const source = SOURCE_NAMES[raw.source] ? raw.source : "python";
    const component = String(raw.component || (source !== raw.source && raw.source) || raw.logger || "未标注组件");
    let timestamp = raw.timestamp_iso || raw.timestamp || raw.ts || "";
    if (typeof timestamp === "number") timestamp = new Date(timestamp < 1e12 ? timestamp * 1000 : timestamp).toISOString();
    const level = String(raw.level || raw.severity || "info").toLowerCase();
    const text = String(raw.message ?? raw.msg ?? raw.display_message ?? "");
    const id = String(raw.id || `${timestamp}|${raw.sequence ?? input?.cursor ?? ""}|${component}|${text}`);
    return {
      id, timestamp: String(timestamp), level: level === "warn" ? "warning" : level, source, component,
      event: raw.event || raw.event_type || raw.runtime_event_type || null,
      message: text, request_id: raw.request_id || null, task_id: raw.task_id || null,
      capability_name: raw.capability_name || null, exception: raw.exception || null,
      details: typeof details === "object" && details !== null ? details : { detail: details },
      _order: historical ? --state.historicalSerial : ++state.serial,
    };
  }

  function publicEntry(entry) {
    const { _order, ...result } = entry;
    return result;
  }

  function matches(entry, selection) {
    if (selection.level && entry.level !== selection.level) return false;
    if (selection.source && entry.source !== selection.source) return false;
    if (selection.component && !entry.component.toLowerCase().includes(selection.component.toLowerCase())) return false;
    if (selection.request_id && entry.request_id !== selection.request_id) return false;
    if (selection.task_id && entry.task_id !== selection.task_id) return false;
    return !selection.query || JSON.stringify(publicEntry(entry)).toLowerCase().includes(selection.query.toLowerCase());
  }

  function ordered() {
    return [...records.values()].sort((left, right) => (Date.parse(right.timestamp) || 0) - (Date.parse(left.timestamp) || 0) || right._order - left._order);
  }

  function selectedText() {
    const selection = window.getSelection?.();
    if (!selection || selection.isCollapsed || !selection.rangeCount) return false;
    return [element("consoleLogRows"), element("recentLogs")].some((root) => root && (root.contains(selection.anchorNode) || root.contains(selection.focusNode)));
  }

  function addEntries(entries, historical = false) {
    let count = 0;
    for (const input of Array.isArray(entries) ? entries : [entries]) {
      if (!input) continue;
      const entry = normalize(input, historical);
      if (records.has(entry.id)) continue;
      records.set(entry.id, entry);
      count += 1;
    }
    if (records.size > CACHE_LIMIT) {
      const current = filters();
      const sorted = ordered();
      // 先保留当前筛选的记录，避免加载较老的匹配项时被无关实时日志立即淘汰。
      const keep = new Set([...sorted.filter((entry) => matches(entry, current)), ...sorted.filter((entry) => !matches(entry, current))].slice(0, CACHE_LIMIT).map((entry) => entry.id));
      for (const id of records.keys()) if (!keep.has(id)) records.delete(id);
    }
    if (count) {
      state.pending += count;
      state.dirty = true;
      scheduleRender();
    }
  }

  function make(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = String(text);
    return node;
  }

  function asText(entry) {
    const context = [
      ["事件", "event"], ["请求", "request_id"], ["任务", "task_id"], ["能力", "capability_name"],
    ].filter(([, key]) => entry[key]).map(([label, key]) => `${label}（${key}）：${entry[key]}`);
    return [
      `${entry.timestamp} [${entry.level.toUpperCase()}] [${entry.source}/${entry.component}]`,
      entry.message, ...context,
      Object.keys(entry.details).length ? `详细字段：\n${JSON.stringify(entry.details, null, 2)}` : "",
      entry.exception ? `异常栈：\n${entry.exception}` : "",
    ].filter(Boolean).join("\n");
  }

  async function copyText(text) {
    try {
      if (navigator.clipboard?.writeText) {
        await navigator.clipboard.writeText(text);
        return true;
      }
    } catch (_) {
      // 非安全来源或浏览器权限拒绝时，仍可在本次用户点击内尝试原生复制命令。
    }
    const selection = window.getSelection?.();
    const ranges = [];
    for (let index = 0; selection && index < selection.rangeCount; index += 1) ranges.push(selection.getRangeAt(index).cloneRange());
    const focused = document.activeElement;
    const area = make("textarea");
    area.value = text;
    area.readOnly = true;
    area.setAttribute("aria-hidden", "true");
    area.style.position = "fixed";
    area.style.left = "-9999px";
    document.body.appendChild(area);
    let copied = false;
    try {
      area.focus({ preventScroll: true });
      area.select();
      area.setSelectionRange?.(0, text.length);
      copied = document.execCommand?.("copy") === true;
    } catch (_) {
      copied = false;
    } finally {
      area.remove();
      focused?.focus?.({ preventScroll: true });
      if (selection && ranges.length) {
        selection.removeAllRanges();
        for (const range of ranges) selection.addRange(range);
      }
    }
    return copied;
  }

  async function copyVisible() {
    if (!state.visible.length) return message("当前没有可复制的日志。");
    const copied = await copyText(state.visible.map(asText).join("\n\n"));
    message(copied ? `已复制当前显示的 ${state.visible.length} 条日志。` : "复制失败；可选中日志内容后使用浏览器复制，或下载日志文件。");
  }

  function copyButton(label, entry, json) {
    const button = make("button", "console-log-copy", label);
    button.type = "button";
    button.addEventListener("click", async (event) => {
      event.preventDefault();
      event.stopPropagation();
      const copied = await copyText(json ? JSON.stringify(publicEntry(entry), null, 2) : asText(entry));
      message(copied ? "已复制这条日志。" : "复制失败；请选中内容复制或使用日志下载。");
    });
    return button;
  }

  function row(entry) {
    const outer = make("details", "console-log-entry");
    outer.dataset.logId = entry.id;
    const summary = make("summary", "console-log-summary");
    const time = make("time", "console-log-time", entry.timestamp ? new Date(entry.timestamp).toLocaleString("zh-CN", { hour12: false }) : "时间未知");
    time.title = entry.timestamp;
    if (entry.timestamp) time.dateTime = entry.timestamp;
    const level = LEVEL_NAMES[entry.level] ? entry.level : "info";
    const readableMessage = entry.message === "运行时事件" && entry.details.runtime_message ? entry.details.runtime_message : entry.message;
    const headline = make("span", "console-log-message", readableMessage);
    const context = [];
    if (entry.details.method || entry.details.path) context.push([entry.details.method, entry.details.path].filter(Boolean).join(" "));
    if (entry.details.status_code !== undefined) context.push("状态 " + entry.details.status_code);
    if (Number.isFinite(Number(entry.details.duration_ms))) context.push(Number(entry.details.duration_ms).toFixed(1) + " 毫秒");
    if (entry.event) context.push(entry.event);
    if (entry.task_id) context.push("任务 " + entry.task_id);
    else if (entry.request_id) context.push("请求 " + entry.request_id);
    if (context.length) headline.appendChild(make("span", "console-log-context", context.join(" · ")));
    summary.append(time, make("span", `console-log-level level-${level}`, LEVEL_NAMES[entry.level] || entry.level), make("span", "console-log-component", entry.component), headline);
    summary.append(copyButton("复制文本", entry, false), copyButton("复制 JSON", entry, true));
    const body = make("div", "console-log-details");
    const fields = make("dl", "console-log-fields");
    for (const [label, key] of [["时间", "timestamp"], ["来源", "source"], ["组件", "component"], ["事件", "event"], ["请求", "request_id"], ["任务", "task_id"], ["能力", "capability_name"], ["记录标识", "id"]]) {
      if (!entry[key]) continue;
      const field = make("div", "console-log-field");
      field.append(make("dt", "", `${label} · ${key}`), make("dd", "", key === "source" ? `${SOURCE_NAMES[entry.source]} (${entry.source})` : entry[key]));
      fields.appendChild(field);
    }
    body.append(fields, make("pre", "console-log-message", entry.message));
    if (Object.keys(entry.details).length) body.append(make("strong", "", "详细字段"), make("pre", "console-log-fields", JSON.stringify(entry.details, null, 2)));
    if (entry.exception) body.append(make("strong", "", "异常栈"), make("pre", "console-log-message", entry.exception));
    outer.append(summary, body);
    return outer;
  }

  function recentRow(entry) {
    const outer = make("details", "console-log-entry recent-log-entry");
    outer.dataset.logId = entry.id;
    const summary = make("summary", "recent-log-summary");
    const level = LEVEL_NAMES[entry.level] ? entry.level : "info";
    const time = make("time", "console-log-time", entry.timestamp ? new Date(entry.timestamp).toLocaleTimeString("zh-CN", { hour12: false }) : "时间未知");
    time.title = entry.timestamp;
    if (entry.timestamp) time.dateTime = entry.timestamp;
    summary.append(time, make("span", `console-log-level level-${level}`, LEVEL_NAMES[entry.level] || entry.level), make("span", "console-log-component", entry.component), make("span", "console-log-message", entry.message));
    summary.title = "点击展开完整日志";
    const body = make("div", "console-log-details");
    body.append(make("pre", "console-log-message", asText(entry)), copyButton("复制文本", entry, false));
    outer.append(summary, body);
    outer.addEventListener("toggle", () => {
      if (recentState.dirty && !outer.open) scheduleRender();
      scheduleRecentLayout();
    });
    return outer;
  }

  function renderRecentPrefix(count) {
    const recent = element("recentLogs");
    if (!recent) return;
    recentState.count = Math.min(count, recentState.entries.length);
    const wanted = recentState.entries.slice(0, recentState.count).map((entry) => {
      if (!recentRowNodes.has(entry.id)) recentRowNodes.set(entry.id, recentRow(entry));
      return recentRowNodes.get(entry.id);
    });
    if (!wanted.length) wanted.push(make("p", "console-log-empty", "暂无日志"));
    const keep = new Set(wanted);
    for (const node of [...recent.children]) if (!keep.has(node)) node.remove();
    wanted.forEach((node, index) => { if (recent.children[index] !== node) recent.insertBefore(node, recent.children[index] || null); });
    for (const id of recentRowNodes.keys()) if (!records.has(id)) recentRowNodes.delete(id);
  }

  function fitRecentLogs() {
    recentState.frame = null;
    const recent = element("recentLogs");
    const host = recent?.parentElement;
    if (!host?.clientHeight || state.paused || selectedText() || recentState.pointerActive) return;
    // 每个动画帧集中测量一次实际排版，预留少量滚动内容；不以固定条数猜测行高。
    const bottom = host.getBoundingClientRect().top + host.clientHeight * 1.25;
    const nodes = [...recent.children];
    const bounds = nodes.map((node) => node.getBoundingClientRect());
    const enoughAt = bounds.findIndex((rect) => rect.bottom >= bottom);
    if (enoughAt >= 0) {
      const keepReading = nodes.reduce((last, node, index) => node.open || node.contains(document.activeElement) ? index + 1 : last, 0);
      const count = Math.max(enoughAt + 1, keepReading);
      if (host.scrollTop <= 4 && count < recentState.count) renderRecentPrefix(count);
      return;
    }
    if (recentState.count >= recentState.entries.length) return;
    const lastBottom = bounds.length ? bounds[bounds.length - 1].bottom : host.getBoundingClientRect().top;
    const average = bounds.length ? Math.max(1, (lastBottom - bounds[0].top) / bounds.length) : 60;
    const batch = Math.max(1, Math.min(40, Math.ceil((bottom - lastBottom) / average)));
    renderRecentPrefix(recentState.count + batch);
    scheduleRecentLayout();
  }

  function scheduleRecentLayout() {
    if (recentState.frame === null) recentState.frame = window.requestAnimationFrame(fitRecentLogs);
  }

  function updateRecentLogs(sorted) {
    const recent = element("recentLogs");
    if (!recent) return;
    if (recentState.pointerActive || recent.contains(document.activeElement)
        || recent.querySelector(".recent-log-entry[open]") || (recent.parentElement.scrollTop > 4 && recentState.count)) {
      recentState.dirty = true;
      return;
    }
    recentState.entries = sorted;
    recentState.dirty = false;
    renderRecentPrefix(recentState.count || 1);
    scheduleRecentLayout();
  }

  function render() {
    state.renderTimer = null;
    if (state.paused || selectedText()) {
      write("logPending", state.pending ? `${state.pending} 条待显示` : "");
      state.dirty = true;
      return;
    }
    const root = element("consoleLogRows");
    if (!root) return;
    const sorted = ordered();
    const filtered = sorted.filter((entry) => matches(entry, filters()));
    state.visible = filtered.slice(0, state.displayLimit);
    const wanted = state.visible.map((entry) => {
      if (!rowNodes.has(entry.id)) rowNodes.set(entry.id, row(entry));
      return rowNodes.get(entry.id);
    });
    if (!wanted.length) wanted.push(make("p", "console-log-empty", state.loading ? "正在读取日志…" : "没有符合当前筛选条件的日志。"));
    const keep = new Set(wanted);
    for (const node of [...root.children]) if (!keep.has(node)) node.remove();
    wanted.forEach((node, index) => { if (root.children[index] !== node) root.insertBefore(node, root.children[index] || null); });
    for (const [id] of rowNodes) if (!records.has(id)) rowNodes.delete(id);
    write("logCount", `${state.visible.length} 条记录`);
    write("logErrorCount", `${state.visible.filter((entry) => ["error", "critical"].includes(entry.level)).length} 条错误`);
    state.pending = 0;
    state.dirty = false;
    write("logPending", "");
    const more = element("logLoadMore");
    if (more) more.disabled = state.loading || state.displayLimit >= CACHE_LIMIT || (!state.hasMore && filtered.length <= state.visible.length);
    updateRecentLogs(sorted);
  }

  function scheduleRender() {
    if (state.renderTimer === null) state.renderTimer = window.setTimeout(render, 100);
  }

  function params() {
    return new URLSearchParams(Object.entries(filters()).filter(([, value]) => value));
  }

  async function load(append = false, quiet = false) {
    if (append && state.loading) return;
    const requestId = ++state.request;
    state.loading = true;
    const query = params();
    query.set("limit", String(PAGE_SIZE));
    if (append && state.nextBefore) query.set("before", state.nextBefore);
    if (!quiet) message(append ? "正在读取更早的日志…" : "正在查询日志…");
    try {
      const response = await fetch(`/api/logs?${query}`, { cache: "no-store" });
      if (!response.ok) throw new Error(`读取失败（${response.status}）`);
      const result = await response.json();
      if (requestId !== state.request) return;
      if (!quiet || !state.nextBefore) {
        state.nextBefore = result.next_before;
        state.hasMore = Boolean(result.has_more);
      }
      addEntries(result.entries || [], true);
      if (!quiet) message(result.cursor_expired ? "所选历史已超出文件保留范围，请刷新查询。" : `${state.paused ? "画面已暂停；" : ""}历史已更新；独立组件日志每 3 秒补取，可展开记录查看完整字段和异常。`);
    } catch (error) {
      if (requestId === state.request) message("日志暂时无法读取，将自动重试。" + (error.message ? "原因：" + error.message : ""));
    } finally {
      if (requestId === state.request) {
        state.loading = false;
        scheduleRender();
      }
    }
  }

  function reload() {
    state.nextBefore = null;
    state.hasMore = false;
    return load(false);
  }

  function recoverGap() {
    state.gapNotice = `实时日志有缺口；重读仅查询最近 ${PAGE_SIZE} 条匹配日志，更早记录请加载历史或导出。`;
    if (state.gapReloadPromise) return state.gapReloadPromise;
    state.gapReloadPromise = reload().finally(() => { state.gapReloadPromise = null; });
    return state.gapReloadPromise;
  }

  async function more() {
    if (state.displayLimit >= CACHE_LIMIT) return message("当前最多显示 5000 条，请缩小筛选范围或导出历史。");
    state.displayLimit = Math.min(CACHE_LIMIT, state.displayLimit + PAGE_SIZE);
    if (state.hasMore) await load(true);
    scheduleRender();
  }

  async function exportLogs(format) {
    const query = params();
    query.set("format", format);
    query.set("limit", "100000");
    message("正在准备日志下载…");
    try {
      const response = await fetch(`/api/logs/export?${query}`, { cache: "no-store" });
      if (!response.ok) throw new Error(`导出失败（${response.status}）`);
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      const anchor = make("a");
      anchor.href = url;
      anchor.download = `Nuwax日志_${new Date().toISOString().slice(0, 10)}.${format === "text" ? "txt" : "jsonl"}`;
      document.body.appendChild(anchor);
      anchor.click();
      anchor.remove();
      window.setTimeout(() => URL.revokeObjectURL(url), 1000);
      message(response.headers.get("X-Logs-Has-More") === "true" ? "已导出最近 100000 条匹配日志；更早记录请缩小筛选范围后导出。" : "日志文件已开始下载。");
    } catch (error) {
      message("日志导出失败，请稍后重试。" + (error.message ? "原因：" + error.message : ""));
    }
  }

  function bind() {
    for (const id of Object.values(filterIds)) element(id)?.addEventListener("input", () => {
      state.displayLimit = PAGE_SIZE;
      window.clearTimeout(state.filterTimer);
      // 立即废弃旧查询，避免旧响应在输入防抖间隙覆盖当前筛选的游标。
      state.request += 1;
      scheduleRender();
      state.filterTimer = window.setTimeout(reload, 250);
    });
    element("logPause")?.addEventListener("click", () => {
      state.paused = !state.paused;
      const button = element("logPause");
      button.textContent = state.paused ? "继续显示" : "暂停显示";
      button.setAttribute("aria-pressed", String(state.paused));
      message(state.paused ? "画面已暂停，新增日志将在继续后显示；复制当前视图保留此刻内容。" : "已恢复实时显示。");
      scheduleRender();
    });
    element("logRefresh")?.addEventListener("click", reload);
    element("logLoadMore")?.addEventListener("click", more);
    element("logCopyAll")?.addEventListener("click", copyVisible);
    element("logExportText")?.addEventListener("click", () => exportLogs("text"));
    element("logExportJson")?.addEventListener("click", () => exportLogs("jsonl"));
    document.addEventListener("selectionchange", () => {
      if (!selectedText()) {
        if (state.dirty) scheduleRender();
        scheduleRecentLayout();
      }
    });
    const recent = element("recentLogs");
    if (recent) {
      recent.addEventListener("pointerdown", () => { recentState.pointerActive = true; });
      recent.addEventListener("focusout", () => { if (recentState.dirty) scheduleRender(); });
      const releasePointer = () => {
        if (!recentState.pointerActive) return;
        recentState.pointerActive = false;
        if (recentState.dirty) scheduleRender();
        scheduleRecentLayout();
      };
      window.addEventListener("pointerup", releasePointer);
      window.addEventListener("pointercancel", releasePointer);
      window.addEventListener("blur", releasePointer);
      recent.parentElement.addEventListener("scroll", () => {
        if (recent.parentElement.scrollTop <= 4 && recentState.dirty) scheduleRender();
        scheduleRecentLayout();
      }, { passive: true });
      if (window.ResizeObserver) new ResizeObserver(scheduleRecentLayout).observe(recent.parentElement);
      else window.addEventListener("resize", scheduleRecentLayout);
      document.addEventListener("visibilitychange", scheduleRecentLayout);
    }
    // 网关流只推送本进程事件；独立Rust/Python服务从共享文件补取，不另开连接。
    window.setInterval?.(() => {
      const pane = element("logs-view");
      if (state.opened && !document.hidden && !state.loading && (!pane || pane.classList.contains("active"))) load(false, true);
    }, 3000);
  }

  window.nuwaxConsoleLogs = {
    ingest: (entries) => addEntries(entries),
    open() {
      scheduleRender();
      if (!state.opened) { state.opened = true; return reload(); }
      return Promise.resolve();
    },
    reload,
    recoverGap,
  };
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", bind, { once: true });
  else bind();
})();
