(() => {
      const CANVAS_THEME = {
        mapBackground: "#020617",
        mapUnknown: [51, 65, 85],
        mapFree: [219, 234, 254],
        mapOccupied: [2, 6, 23],
        trajectory: "#22d3ee",
        frontier: "#f472b6",
        frontierStroke: "#fbcfe8",
        robot: "#fbbf24",
        robotStroke: "#fef3c7",
        goal: "#a78bfa",
        goalStroke: "#ddd6fe",
        goalPick: "#34d399",
        goalPickStroke: "#6ee7b7",
        clickedPoint: "#38bdf8",
        clickedPointStroke: "#7dd3fc",
        initialPosePick: "#f97316",
        initialPosePickStroke: "#fdba74",
        navPath: "#22ff66",
      };
      const overviewNames = ["health", "state", "localization", "map", "navigation", "exploration", "runtime_governance", "pointcloud_map", "pointcloud", "video", "perception_nms", "logs"];
      const overviewLabels = { health: "健康", state: "状态", localization: "定位", map: "地图", navigation: "导航", exploration: "探索", runtime_governance: "服务状态", pointcloud_map: "3D点云地图", pointcloud: "扫描点云", video: "视频", perception_nms: "感知 NMS", logs: "日志" };
      const OVERVIEW_RENDER_INTERVAL_MS = 100;
      const TASK_EVENT_QUEUE_LIMIT = 200;
      const STREAM_FLUSH_ORDER = ["robot.state", "map.frame", "map.heartbeat", "localization.trajectory", "localization.pose", "navigation.state", "navigation.forbidden_zones", "exploration.frontier", "exploration.state", "runtime.governance", "pointcloud_map.meta", "pointcloud.frame", "video.frame", "video.detections", "tasks.state", "system.metrics"];
      const VIDEO_STREAM_INITIAL_RECONNECT_MS = 500;
      const VIDEO_STREAM_MAX_RECONNECT_MS = 5000;
      const VIDEO_JPEG_DECODE_TIMEOUT_MS = 3000;
      const MJPEG_MAX_BUFFER_BYTES = 16 * 1024 * 1024;
      const VIDEO_DISPLAY_FRESH_MS = 3000;
      const VIDEO_DETECTION_ALIGNMENT_MS = 1000;
      const MAP_DISPLAY_FRESH_MS = 20000;
      const MAP_DISPLAY_IDLE_MESSAGE = "未启动建图/导航/探索，隐藏缓存地图。";
      const LOCALIZATION_DISPLAY_IDLE_MESSAGE = "无活跃地图/建图/导航/探索，隐藏定位。";
      const ACTIVE_SPATIAL_STATUSES = new Set(["accepted", "planning", "running"]);
      const ACTIVE_SPATIAL_CAPABILITIES = new Set([
        "explore_area",
        "explore_and_find_target",
        "navigate_to_pose",
        "navigate_to_named_location",
        "find_target",
        "inspect_target",
        "relocalize_active_map",
        "activate_map",
      ]);
      const ACTIVE_LOCALIZATION_STATUSES = new Set([
        "accepted",
        "running",
        "localizing",
        "relocalizing",
        "ready",
      ]);
      const state = {
        mapGrid: null,
        mapViewport: null,
        mapZoom: 1,
        map2dCameraMode: "follow",
        mapFreeView: { center: null, zoom: 1, fitSpan: null },
        mapFreeViewIdentity: "",
        mapViewMode: "2d",
        pointcloud3dCameraMode: "free",
        pointcloud3dDistance: null,
        pointcloud3dZoomPercent: 100,
        pointcloud3dSourcePoints: 0,
        pointcloud3dDisplayedPoints: 0,
        pointcloud3dPointCountKnown: false,
        pointcloud3dReady: false,
        mapRvizView: { center: null, zoom: 1 },
        mapPanDrag: null,
        mapStream: null,
        pose: null,
        latestPose: null,
        trajectory: [],
        frontiers: [],
        goalPose: null,
        goalPickMode: false,
        publishPointMode: false,
        initialPosePickMode: false,
        goalPickDrag: null,
        initialPosePickDrag: null,
        publishPointClick: null,
        clickedPoints: [],
        initialPoses: [],
        pendingNavGoal: null,
        navigationPath: [],
        logCursor: 0,
        streamSocket: null,
        streamConnectionEpoch: 0,
        streamReconnectTimer: null,
        streamReconnectDelayMs: 1000,
        streamReconnectAttempt: 0,
        latestStreamCursor: "0",
        poseStream: {
          topics: ["localization.pose", "map.frame"],
          mapPayload: "identity",
          streamSocket: null,
          streamConnectionEpoch: 0,
          streamReconnectTimer: null,
          streamReconnectDelayMs: 1000,
          streamReconnectAttempt: 0,
          latestStreamCursor: "0",
        },
        poseMapIdentity: null,
        mapStreamReady: false,
        receivedMapIdentity: null,
        appliedMapIdentity: null,
        pendingMapPose: null,
        latestStreamFrames: {},
        pendingStreamFrames: {},
        pendingLogFrames: [],
        pendingTaskFrames: [],
        streamFlushFrame: null,
        videoFrames: 0,
        pendingVideoEnvelope: null,
        videoRenderFrame: null,
        lastVideoFrameKey: null,
        videoStreamSrc: null,
        videoStreamController: null,
        videoStreamGeneration: 0,
        videoStreamReconnectTimer: null,
        videoStreamReconnectDelayMs: VIDEO_STREAM_INITIAL_RECONNECT_MS,
        videoStreamStatus: "idle",
        videoStreamBytes: 0,
        videoDecodedBytes: 0,
        videoDecodeErrors: 0,
        videoLastError: "",
        videoLastFrameAt: null,
        videoCurrentObjectUrl: null,
        showDetections: true,
        videoDetections: [],
        videoSourceSize: null,
        videoDetectionDrawFrame: null,
        videoFrameMetadata: null,
        videoDisplayedIdentity: null,
        videoDetectionFrame: null,
        videoDetectionRevision: 0,
        videoDetectionDrawKey: "",
        videoDetectionLayoutRevision: 0,
        visionLabelsKey: "",
        latestOverview: null,
        overviewRenderTimer: null,
        overviewLastRenderAt: 0,
        overviewReceivedAt: null,
        latestHealth: null,
        latestSystemMetrics: null,
        latestGovernance: null,
        latestServiceHealth: null,
        latestTasks: { active_tasks: [], history_tasks: [] },
        taskActiveById: new Map(),
        taskHistoryById: new Map(),
        taskEventSequence: 0,
        taskSnapshotSequence: 0,
        taskLoadPromise: null,
        taskReloadPending: false,
        taskQueueGapPending: false,
        taskStoreBatching: false,
        bootstrapComplete: { health: false, tasks: false, governance: false },
        bootstrapQueue: { health: [], tasks: [], governance: [] },
        lastRobotStateRevision: null,
        lastRobotStateHash: null,
        mapDrawFrame: null,
        mapBaseCache: null,
        mapGridContentKey: "",
        lastMapEnvelope: null,
        lastLocalizationEnvelope: null,
        lastMapHeartbeatEnvelope: null,
        latestFrontierStream: null,
        latestExplorationOverlayIdentity: null,
        explorationOverlayRequiresCanonical: false,
        semanticInstances: [],
        semanticLoadPromise: null,
        semanticSnapshotKey: "",
        semanticPublishedKey: "",
        semanticMapCache: null,
        semanticLastMapName: "",
        semanticFetchedAt: 0,
        semanticError: "",
        semanticMeta: {},
        semanticSelectedId: null,
        semanticOverlayReturnFocus: null,
        showSemanticInstances: true,
        highlightTarget: null,
        clusterAnim: null,
        _prevClusters: null,
        _semanticPollTimer: null,
        _zoomDebounceTimer: null,
      };
      const els = {
        robotName: document.getElementById("robotName"),
        robotModel: document.getElementById("robotModel"),
        connectionText: document.getElementById("connectionText"),
        runtimeMode: document.getElementById("runtimeMode"),
        batteryMetric: document.getElementById("batteryMetric"),
        networkLatency: document.getElementById("networkLatency"),
        resourceMetric: document.getElementById("resourceMetric"),
        clockNow: document.getElementById("clockNow"),
        motionStatus: document.getElementById("motionStatus"),
        missionStatus: document.getElementById("missionStatus"),
        taskStatus: document.getElementById("taskStatus"),
        taskSummary: document.getElementById("taskSummary"),
        poseStatus: document.getElementById("poseStatus"),
        missionPoseMeta: document.getElementById("missionPoseMeta"),
        sensorStatus: document.getElementById("sensorStatus"),
        sensorSummary: document.getElementById("sensorSummary"),
        lastReceive: document.getElementById("lastReceive"),
        overviewGrid: document.getElementById("overviewGrid"),
        mapCanvas: document.getElementById("mapCanvas"),
        map3dContainer: document.getElementById("map3dContainer"),
        mapModeToggle: document.getElementById("mapModeToggle"),
        map2dCameraControls: document.getElementById("map2dCameraControls"),
        map3dCameraControls: document.getElementById("map3dCameraControls"),
        map2dActionControls: document.getElementById("map2dActionControls"),
        map3dFreeView: document.getElementById("map3dFreeView"),
        map3dFollowView: document.getElementById("map3dFollowView"),
        map3dPointStyle: document.getElementById("map3dPointStyle"),
        map3dZoomOut: document.getElementById("map3dZoomOut"),
        map3dZoomValue: document.getElementById("map3dZoomValue"),
        map3dZoomIn: document.getElementById("map3dZoomIn"),
        map3dResetView: document.getElementById("map3dResetView"),
        map3dCenterRobot: document.getElementById("map3dCenterRobot"),
        mapZoomIn: document.getElementById("mapZoomIn"),
        mapZoomOut: document.getElementById("mapZoomOut"),
        mapZoomReset: document.getElementById("mapZoomReset"),
        map2dCameraToggle: document.getElementById("map2dCameraToggle"),
        mapCenterRobot: document.getElementById("mapCenterRobot"),
        mapZoomValue: document.getElementById("mapZoomValue"),
        mapRvizHint: document.getElementById("mapRvizHint"),
        mapInitialPoseToggle: document.getElementById("mapInitialPoseToggle"),
        mapGoalToggle: document.getElementById("mapGoalToggle"),
        mapGoalHeight: document.getElementById("mapGoalHeight"),
        mapPublishPointToggle: document.getElementById("mapPublishPointToggle"),
        mapGoalStatus: document.getElementById("mapGoalStatus"),
        mapStreamStatus: document.getElementById("mapStreamStatus"),
        mapStatus: document.getElementById("mapStatus"),
        mapEmpty: document.getElementById("mapEmpty"),
        mapPoseMeta: document.getElementById("mapPoseMeta"),
        mapMeta: document.getElementById("mapMeta"),
        mapSummaryMode: document.getElementById("mapSummaryMode"),
        localizationStatus: document.getElementById("localizationStatus"),
        localizationMeta: document.getElementById("localizationMeta"),
        navigationStatus: document.getElementById("navigationStatus"),
        navigationMeta: document.getElementById("navigationMeta"),
        frontierNotice: document.getElementById("frontierNotice"),
        governanceStatus: document.getElementById("governanceStatus"),
        governanceMeta: document.getElementById("governanceMeta"),
        videoStatus: document.getElementById("videoStatus"),
        videoFrame: document.getElementById("videoFrame"),
        videoDetections: document.getElementById("videoDetections"),
        showDetections: document.getElementById("showDetections"),
        visionLabels: document.getElementById("visionLabels"),
        showSemanticInstances: document.getElementById("showSemanticInstances"),
        semanticInstancesRefresh: document.getElementById("semanticInstancesRefresh"),
        semanticMemoryToggle: document.getElementById("semanticMemoryToggle"),
        semanticMemoryCount: document.getElementById("semanticMemoryCount"),
        semanticMemoryPanel: document.getElementById("semanticMemoryPanel"),
        semanticMemoryClose: document.getElementById("semanticMemoryClose"),
        semanticMemoryStatus: document.getElementById("semanticMemoryStatus"),
        semanticMemorySearch: document.getElementById("semanticMemorySearch"),
        semanticMemoryFilter: document.getElementById("semanticMemoryFilter"),
        semanticMemorySummary: document.getElementById("semanticMemorySummary"),
        semanticMemoryList: document.getElementById("semanticMemoryList"),
        videoEmpty: document.getElementById("videoEmpty"),
        videoMeta: document.getElementById("videoMeta"),
        visionModelStatus: document.getElementById("visionModelStatus"),
        videoFpsStatus: document.getElementById("videoFpsStatus"),
        servicesStatus: document.getElementById("servicesStatus"),
        serviceMatrix: document.getElementById("serviceMatrix"),
        serviceMatrixScope: document.getElementById("serviceMatrixScope"),
        logsStatus: document.getElementById("logsStatus"),
        logsMeta: document.getElementById("logsMeta"),
        themeToggle: document.getElementById("themeToggle"),
      };

      function initTheme() {
        const saved = localStorage.getItem("nuwax-theme");
        const prefersDark = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
        const theme = saved || (prefersDark ? "dark" : "light");
        document.documentElement.setAttribute("data-theme", theme);
        updateThemeIcon(theme);
      }
      function toggleTheme() {
        const current = document.documentElement.getAttribute("data-theme") || "dark";
        const next = current === "dark" ? "light" : "dark";
        document.documentElement.setAttribute("data-theme", next);
        localStorage.setItem("nuwax-theme", next);
        updateThemeIcon(next);
      }
      function updateThemeIcon(theme) {
        if (els.themeToggle) els.themeToggle.textContent = theme === "dark" ? "🌙" : "☀️";
      }
      initTheme();
      if (els.themeToggle) els.themeToggle.addEventListener("click", toggleTheme);
      function initTabPanels() {
        for (const panel of document.querySelectorAll(".tab-panel")) {
          const buttons = Array.from(panel.querySelectorAll(".tab-button"));
          const panes = Array.from(panel.querySelectorAll(".tab-pane"));
          for (const button of buttons) {
            button.addEventListener("click", () => {
              const targetId = button.dataset.tabTarget;
              for (const item of buttons) item.classList.toggle("active", item === button);
              for (const pane of panes) pane.classList.toggle("active", pane.id === targetId);
              redrawMapAfterResize();
            });
          }
        }
      }
      initTabPanels();

      function openConsoleView(targetId) {
        const buttons = Array.from(document.querySelectorAll(".view-switch-btn"));
        const panes = Array.from(document.querySelectorAll("#view-host > .view-pane"));
        for (const item of buttons) item.classList.toggle("active", item.dataset.viewTarget === targetId);
        for (const pane of panes) pane.classList.toggle("active", pane.id === targetId);
        if (targetId === "config-view") window.nuwaxConsoleSettings?.open();
        if (targetId === "logs-view") window.nuwaxConsoleLogs?.open();
        if (targetId === "monitor-view") redrawMapAfterResize();
      }
      function initViewSwitch() {
        for (const button of document.querySelectorAll(".view-switch-btn, [data-open-view]")) {
          button.addEventListener("click", () => openConsoleView(button.dataset.viewTarget || button.dataset.openView));
        }
      }
      initViewSwitch();

      function initMediaSwap() {
        const workspace = document.querySelector(".workspace");
        const video = document.getElementById("video-perception");
        const btnMap = document.getElementById("mediaSwapBtn");
        const btnVideo = document.getElementById("mediaSwapBtnVideo");
        if (!workspace) return;
        function toggle() {
          const swapped = workspace.classList.toggle("swapped");
          if (btnMap) btnMap.textContent = swapped ? "⇄ 地图" : "⇄ 视频";
          if (btnVideo) btnVideo.textContent = swapped ? "⇄ 缩小" : "⇄ 放大";
          if (video) video.classList.toggle("rise", false);
          // 槽位尺寸变化后重排地图画布并重绘（视频靠 CSS 自适应，无需干预）
          setTimeout(redrawMapAfterResize, 60);
          setTimeout(redrawMapAfterResize, 320);
        }
        if (btnMap) btnMap.addEventListener("click", toggle);
        if (btnVideo) btnVideo.addEventListener("click", toggle);
      }
      initMediaSwap();

      const STATUS_LABELS = {
        available: "可用",
        ok: "正常",
        ready: "就绪",
        stale: "过期",
        missing: "缺失",
        error: "错误",
        unsupported: "不支持",
        cleared: "已清空",
        idle: "空闲",
        streaming: "实时流",
        stream_bus: "流总线",
        reconnecting: "重连中",
        metadata: "元数据",
        degraded: "降级",
        disabled: "未启用",
        unavailable: "不可用",
        debug: "调试",
        info: "信息",
        warn: "警告",
        warning: "警告",
        critical: "严重",
        completed: "已完成",
        succeeded: "已完成",
        failed: "失败",
        cancelled: "已取消",
        pending: "等待中",
      };
      const TASK_STATE_LABELS = {
        accepted: "已接收",
        pending: "等待中",
        queued: "排队中",
        running: "运行中",
        succeeded: "已完成",
        failed: "失败",
        cancelled: "已取消",
        timeout: "超时",
        missing: "未上报",
        idle: "空闲",
      };
      const BOOL_LABELS = { true: "是", false: "否" };
      const SOURCE_KIND_LABELS = {
        live: "实时",
        stream: "流式",
        stream_bus: "流总线",
        cache: "缓存",
        offline: "离线",
        unavailable: "不可用",
        pointcloud_provider: "点云服务",
        gateway: "网关",
        psutil: "系统采集",
      };
      const COMMON_FIELD_LABELS = {
        latency: "延迟",
        frame: "坐标系",
        source: "来源",
        reason: "原因",
        triggered: "已触发",
        ready: "就绪",
        count: "数量",
        revision: "版本",
      };
      function nowText() { return new Date().toLocaleTimeString(); }
      function parseSourceTimestamp(value) {
        if (value === null || value === undefined || value === "") return null;
        if (value instanceof Date) return Number.isFinite(value.getTime()) ? value : null;
        if (typeof value === "number") {
          const ms = Math.abs(value) < 1e12 ? value * 1000 : value;
          const date = new Date(ms);
          return Number.isFinite(date.getTime()) ? date : null;
        }
        const text = String(value).trim();
        if (!text) return null;
        if (/^\d+(\.\d+)?$/.test(text)) return parseSourceTimestamp(Number(text));
        const normalized = text.includes("T") ? text : text.replace(" ", "T").replace(",", ".");
        const parsed = new Date(normalized);
        return Number.isFinite(parsed.getTime()) ? parsed : null;
      }
      function shortTime(value) {
        const date = parseSourceTimestamp(value);
        return date ? date.toLocaleTimeString() : "--";
      }
      function formatLogTime(entry) {
        if (entry && entry.timestamp_text) return entry.timestamp_text;
        const date = parseSourceTimestamp(entry && (
          entry.timestamp_iso
          || entry.timestamp_unix_ms
          || entry.timestamp_unix_sec
          || entry.log_timestamp
          || entry.timestamp
        ));
        return date ? date.toLocaleTimeString() : "--";
      }
      function fmt(value, suffix) {
        if (value === null || value === undefined || value === "") return "--";
        if (typeof value === "number") return Number.isFinite(value) ? `${Math.round(value * 10) / 10}${suffix || ""}` : "--";
        return `${value}${suffix || ""}`;
      }
      function safeText(value) { return value === null || value === undefined || value === "" ? "--" : String(value); }
      function compactLogValue(value) {
        if (value === null || value === undefined || value === "" || value === "-") return "";
        if (typeof value === "string") return value;
        try {
          return JSON.stringify(value);
        } catch (_error) {
          return String(value);
        }
      }
      function displayStatus(value) {
        const key = String(value === null || value === undefined || value === "" ? "missing" : value).toLowerCase();
        return STATUS_LABELS[key] || TASK_STATE_LABELS[key] || String(value);
      }
      function displayBool(value) {
        if (value === true || value === "true") return BOOL_LABELS.true;
        if (value === false || value === "false") return BOOL_LABELS.false;
        return "未上报";
      }
      function displaySource(value) {
        const key = String(value === null || value === undefined || value === "" ? "missing" : value).toLowerCase();
        return SOURCE_KIND_LABELS[key] || safeText(value);
      }
      function displayValue(value, kind) {
        if (kind === "status") return displayStatus(value);
        if (kind === "bool") return displayBool(value);
        if (kind === "source") return displaySource(value);
        if (kind === "task_state") return TASK_STATE_LABELS[normalizeTaskState(value)] || displayStatus(value);
        if (value === true || value === false) return displayBool(value);
        return safeText(value);
      }
      function statusOf(envelope) { return envelope && envelope.status ? String(envelope.status) : "missing"; }
      function statusClass(status) {
        const raw = String(status || "missing").toLowerCase();
        if (["available", "ready", "ok", "streaming", "succeeded", "completed"].includes(raw)) return "available";
        if (["stale", "degraded", "idle", "disabled", "cleared", "pending", "queued"].includes(raw)) return "stale";
        if (["error", "failed", "timeout", "unavailable", "cancelled"].includes(raw)) return "error";
        if (["unsupported"].includes(raw)) return "unsupported";
        return "missing";
      }
      function setPill(el, status, label) {
        if (!el) return;
        const value = statusClass(status || "missing");
        el.className = `pill ${value}`;
        const shown = displayStatus(status || "missing");
        el.textContent = label ? `${label}：${shown}` : shown;
        el.title = String(status || "missing");
      }
      function setKv(el, rows) {
        if (!el) return;
        const scrollContainer = el.closest(".map-info-grid, .tab-content, .kv");
        const savedScroll = scrollContainer ? scrollContainer.scrollTop : 0;
        el.replaceChildren();
        for (const [key, value] of rows) {
          const k = document.createElement("div");
          const v = document.createElement("div");
          v.className = "mono";
          k.textContent = key;
          v.textContent = safeText(value);
          el.append(k, v);
        }
        if (scrollContainer && savedScroll > 0) {
          scrollContainer.scrollTop = savedScroll;
        }
      }
      function setText(el, value) {
        if (!el) return;
        el.textContent = safeText(value);
      }
      function syncMapCanvasSize() {
        const canvas = els.mapCanvas;
        if (!canvas) return false;
        const rect = canvas.getBoundingClientRect();
        const width = Math.max(360, Math.round(rect.width));
        const height = Math.max(320, Math.round(rect.height));
        if (canvas.width === width && canvas.height === height) return false;
        canvas.width = width;
        canvas.height = height;
        return true;
      }
      function redrawMapAfterResize() {
        requestMapDraw(false);
      }
      function statusSeverity(status) {
        const value = statusClass(status || "missing");
        if (value === "available") return 0;
        if (value === "stale") return 1;
        if (value === "missing" || value === "unsupported") return 2;
        return 3;
      }
      function worstStatus(items) {
        let result = "available";
        for (const item of items) {
          const status = item && item.status ? item.status : "missing";
          if (statusSeverity(status) > statusSeverity(result)) result = status;
        }
        return result;
      }
      function updateClock() {
        setText(els.clockNow, nowText());
      }
      function errorReason(envelope, fallback) {
        const err = envelope && envelope.error;
        return (err && (err.message || err.code)) || fallback || "数据不可用。";
      }
      function buildOverview() {
        els.overviewGrid.replaceChildren();
        for (const name of overviewNames) {
          const section = document.createElement("section");
          section.className = "card status-card";
          section.id = `overview-${name}`;
          section.innerHTML = `<div class="card-head"><h3>${overviewLabels[name]}</h3><span class="pill missing">missing</span></div><div class="kv"></div><div class="muted small"></div>`;
          els.overviewGrid.appendChild(section);
        }
      }
      function summarizeSource(source) {
        const value = String(source || "");
        if (!value) return "--";
        const parts = value.split(/[/.]/).filter(Boolean);
        return parts.length ? parts[parts.length - 1].slice(0, 28) : value.slice(0, 28);
      }
      function updateOverview(envelope) {
        const data = envelope && envelope.data ? envelope.data : {};
        state.latestOverview = data;
        for (const name of overviewNames) {
          // perception_nms 卡片由 video.detections 流（updatePerceptionNmsCard）驱动，
          // 不走通用 overview envelope，避免被 missing 覆盖。
          if (name === "perception_nms") continue;
          const summary = data[name] || {};
          const section = document.getElementById(`overview-${name}`);
          if (!section) continue;
          setPill(section.querySelector(".pill"), summary.status || "missing");
          setKv(section.querySelector(".kv"), [
            ["时间", shortTime(summary.source_timestamp)],
            ["延迟", fmt(summary.latency_ms, " ms")],
            ["过期", summary.stale === true ? "是" : "否"],
            ["来源", summarizeSource(summary.metadata && summary.metadata.source)],
          ]);
          section.querySelector(".small").textContent = `前端接收 ${shortTime(envelope.__receivedAt)}`;
        }
        const mission = worstStatus([data.health, data.state, data.localization, data.map, data.navigation, data.video, data.logs]);
        setPill(els.missionStatus, mission, "总览");
        updateSensorSummary(data);
      }
      function updateSensorSummary(data) {
        state.latestOverview = data || {};
        renderServiceStatus();
      }
      function serviceStatusValue(summary, fallback) {
        if (summary && summary.status) return summary.status;
        if (fallback !== undefined && fallback !== null && fallback !== "") return fallback;
        return "missing";
      }
      function serviceEvidenceFromSummary(summary, positiveText) {
        if (!summary) return "缺少健康字段";
        if (summary.status === "missing") return "未上报健康字段";
        if (summary.status === "stale") return "最近上报已过期";
        if (summary.status === "error") return summary.source_message || "服务报告错误";
        const latency = summary.latency_ms === null || summary.latency_ms === undefined ? "" : `，延迟 ${fmt(summary.latency_ms, " ms")}`;
        return `${positiveText || "最近有上报"}${latency}`;
      }
      function serviceAdvice(status, fallback) {
        const cls = statusClass(status);
        if (cls === "available") return fallback || "无需处理";
        if (cls === "stale") return fallback || "等待服务刷新，必要时查看运行日志";
        if (cls === "error") return fallback || "查看运行日志定位错误";
        if (String(status || "").toLowerCase() === "unsupported") return "当前环境不支持该服务";
        return fallback || "检查服务是否启用并查看运行日志";
      }
      function processByRole(...roles) {
        const snap = state.latestSystemMetrics || {};
        const processes = Array.isArray(snap.processes) ? snap.processes : [];
        return processes.find((proc) => roles.includes(proc.role));
      }
      const SERVICE_SOURCE_LABELS = {
        gateway_runtime: "网关运行时",
        stream_bus: "实时流总线",
        task_manager: "任务管理器",
        mapping_service: "地图服务",
        localization_service: "定位服务",
        navigation_service: "导航服务",
        perception_video_runtime: "感知视频运行时",
        artifact_service: "制品服务",
        platform_geometry_runtime: "统一几何运行时",
        robot_assembly: "机器人数据面",
        "system.metrics": "系统进程采集",
        frontend_fallback: "前端兼容推断",
      };
      // 常显实际连接与运行组件，其余能力状态按需查看；不改变后端健康结论。
      const SERVICE_PRIMARY_IDS = new Set(["gateway_web", "gateway_web_fallback", "robot_data_plane", "geometry_runtime"]);
      const SERVICE_DISPLAY_NAMES = {
        gateway_web: "网页网关", gateway_web_fallback: "网页网关",
        robot_data_plane: "机器人数据接入", geometry_runtime: "几何运行进程",
        person_follow: "人物跟随", artifacts: "记忆与文件存储",
      };
      const SERVICE_ATTENTION_STATUSES = new Set(["error", "missing", "degraded", "stale"]);
      function displayServiceSource(value) {
        const key = String(value || "").trim();
        return SERVICE_SOURCE_LABELS[key] || displaySource(value);
      }
      function normalizeServiceHealthSummary(raw, sourceLabel) {
        if (!raw || typeof raw !== "object") return null;
        const entries = Array.isArray(raw.entries) ? raw.entries : (Array.isArray(raw.services) ? raw.services : []);
        if (!entries.length) return null;
        const normalizedEntries = entries
          .filter(Boolean)
          .map((entry, index) => ({
            ...entry,
            service_id: entry.service_id || entry.id || `service-${index}`,
            display_name: entry.display_name || entry.name || entry.service_id || `服务 ${index + 1}`,
            status: String(entry.status || "missing").toLowerCase(),
            order: Number.isFinite(Number(entry.order)) ? Number(entry.order) : index,
            source: entry.source || sourceLabel || raw.source || "service_health",
          }))
          .sort((a, b) => (Number(a.order) || 0) - (Number(b.order) || 0));
        return {
          ...raw,
          entries: normalizedEntries,
          status: String(raw.status || worstStatus(normalizedEntries)).toLowerCase(),
          source: raw.source || sourceLabel || "service_health",
          authoritative: raw.authoritative !== false,
        };
      }
      function mergeServiceHealth(raw, sourceLabel, mode = "replace") {
        // 任务 4.1：mode 参数；默认 "replace"（向后兼容任务 4.8）
        if (mode === "replace") return mergeServiceHealthReplace(raw, sourceLabel);
        if (mode === "patch") return mergeServiceHealthPatch(raw, sourceLabel);
        return false;
      }
      function mergeServiceHealthReplace(raw, sourceLabel) {
        const normalized = normalizeServiceHealthSummary(raw, sourceLabel);
        if (!normalized) return false;
        state.latestServiceHealth = normalized;
        // replace 模式同时刷一次进程证据（后端 snapshot 已包含 process_evidence，但前端 frame 未带）
        patchServiceHealthProcessEvidence();
        renderServiceStatus();
        return true;
      }
      function mergeServiceHealthPatch(raw, sourceLabel) {
        // 任务 4.3：patch 模式 —— 仅更新 process_evidence/process_sampled_at/process_source
        // 禁止覆盖 status/summary/evidence/reason/advice 主字段
        if (!state.latestServiceHealth || !Array.isArray(state.latestServiceHealth.entries)) return false;
        const snap = state.latestSystemMetrics || {};
        const processes = Array.isArray(snap.processes) ? snap.processes : [];
        if (!processes.length) return false;
        const byRole = new Map();
        for (const proc of processes) {
          const role = proc && proc.role;
          if (!role) continue;
          if (!byRole.has(role)) byRole.set(role, []);
          byRole.get(role).push(proc);
        }
        let changed = false;
        const frameSampledAt = snap.sampled_at;
        const entries = state.latestServiceHealth.entries.map((entry) => {
          const roles = Array.isArray(entry.expected_process_roles) ? entry.expected_process_roles : [];
          if (!roles.length) return entry;
          const processEvidence = [];
          for (const role of roles) {
            for (const proc of byRole.get(role) || []) {
              processEvidence.push({
                role,
                display_name: proc.display_name || proc.name || role,
                pid: proc.pid,
                cpu_percent: proc.cpu_percent,
                rss_bytes: proc.rss_bytes,
                sampled_at: frameSampledAt,
                source: "system.metrics",
              });
            }
          }
          if (!processEvidence.length) return entry;
          changed = true;
          const updated = {
            ...entry,
            process_evidence: processEvidence,
            process_sampled_at: frameSampledAt,
            process_source: "system.metrics",
          };
          // 任务 4.6a：patch 必须使用 frame 自身 sampled_at（dev 模式断言；生产可注释）
          if (typeof console !== "undefined" && console.assert) {
            console.assert(updated.process_sampled_at === frameSampledAt, "process_sampled_at drift between frame and patched entry");
          }
          return updated;
        });
        if (changed) {
          state.latestServiceHealth = { ...state.latestServiceHealth, entries };
          renderServiceStatus();
        }
        return changed;
      }
      function patchServiceHealthProcessEvidence() {
        // 兼容旧调用方（replace 模式内部仍用）；统一走 patch 实现，数据源固定为 state.latestSystemMetrics
        return mergeServiceHealthPatch();
      }
      function serviceHealthAttentionCount(summary) {
        const fromBackend = summary && summary.metadata && Number(summary.metadata.attention_count);
        if (Number.isFinite(fromBackend)) return fromBackend;
        const entries = summary && Array.isArray(summary.entries) ? summary.entries : [];
        return entries.filter((entry) => SERVICE_ATTENTION_STATUSES.has(String(entry.status || "").toLowerCase())).length;
      }
      function serviceProcessText(entry) {
        const evidence = Array.isArray(entry.process_evidence) ? entry.process_evidence : [];
        if (evidence.length) {
          // 任务 6.1.3：命中时显示 ✓ PID xxx 已匹配
          return evidence.slice(0, 2).map((proc) => {
            const name = proc.display_name || proc.role || "进程";
            const pid = proc.pid === undefined || proc.pid === null ? "--" : proc.pid;
            return `✓ PID ${pid} ${name}`;
          }).join(" / ");
        }
        const roles = Array.isArray(entry.expected_process_roles) ? entry.expected_process_roles : [];
        return roles.length ? "期望进程未匹配" : "";
      }
      function serviceHealthChip(label, value, title) {
        const chip = document.createElement("span");
        chip.className = "service-health-chip";
        chip.textContent = `${label}：${safeText(value)}`;
        if (title) chip.title = title;
        return chip;
      }
      function appendServiceHealthCard(entry, options) {
        const status = String(entry.status || "missing").toLowerCase();
        const cls = statusClass(status);
        const card = document.createElement("details");
        card.dataset.serviceId = entry.service_id;
        card.open = Boolean(options && options.openServices && options.openServices.has(entry.service_id));
        card.className = `service-health-card ${cls} ${status}`;
        if (entry.reason) card.title = `原因：${entry.reason}`;

        const top = document.createElement("summary");
        top.className = "service-health-top";
        const title = document.createElement("div");
        title.className = "service-health-title";
        title.textContent = SERVICE_DISPLAY_NAMES[entry.service_id] || safeText(entry.display_name || entry.service_id);
        title.title = safeText(entry.display_name || entry.service_id);
        const pill = document.createElement("span");
        setPill(pill, status);
        top.append(title, pill);
        if (SERVICE_ATTENTION_STATUSES.has(status)) {
          const brief = document.createElement("span");
          brief.className = "service-health-brief";
          brief.textContent = safeText(entry.reason ? displayReasonLabel(entry.reason) : (entry.summary || entry.evidence || "请展开查看诊断"));
          top.appendChild(brief);
        }

        const summary = document.createElement("div");
        summary.className = "service-health-summary";
        summary.textContent = safeText(entry.summary || displayStatus(status));
        const evidence = document.createElement("div");
        evidence.className = "service-health-evidence";
        evidence.textContent = safeText(entry.evidence || entry.reason || entry.advice || "暂无详细证据");

        const foot = document.createElement("div");
        foot.className = "service-health-foot";
        foot.appendChild(serviceHealthChip("来源", displayServiceSource(entry.source || (options && options.source))));
        const updated = entry.updated_at || (options && (options.generated_at || options.timestamp));
        if (updated) foot.appendChild(serviceHealthChip("更新", shortTime(updated), String(updated)));
        const processText = serviceProcessText(entry);
        if (processText) foot.appendChild(serviceHealthChip("进程", processText, JSON.stringify(entry.process_evidence || entry.expected_process_roles || [])));
        // 任务 6.1.3：expected_process_roles —— 未命中时显式列出期望角色（让用户知道哪个进程应当启动）
        const expectedRoles = Array.isArray(entry.expected_process_roles) ? entry.expected_process_roles : [];
        const evidenceList = Array.isArray(entry.process_evidence) ? entry.process_evidence : [];
        if (expectedRoles.length && !evidenceList.length) {
          foot.appendChild(serviceHealthChip("期望角色", expectedRoles.join(", "), "未匹配任何进程"));
        }
        if (entry.advice) foot.appendChild(serviceHealthChip("建议", entry.advice));
        if (options && options.compatibleFallback) foot.appendChild(serviceHealthChip("模式", "兼容推断"));

        // 任务 6.1.1：reason 行（独立 chip，机器可读原因在 title 中展示完整值）
        if (entry.reason) {
          const reasonChip = serviceHealthChip("原因", displayReasonLabel(entry.reason), String(entry.reason));
          reasonChip.classList.add("service-health-reason");
          foot.appendChild(reasonChip);
        }

        const details = document.createElement("div");
        details.className = "service-health-details";
        details.append(summary, evidence, foot);
        card.append(top, details);
        ((options && options.target) || els.governanceMeta).appendChild(card);
      }
      function displayReasonLabel(reason) {
        // 任务 6.1.1：reason 字段为机器可读 token 时映射为中文短标签
        const map = {
          expected_process_absent: "进程未匹配",
          process_evidence_drift: "进程证据漂移",
          no_signal_in_window: "窗口内无信号",
          upstream_disabled: "上游已禁用",
        };
        return map[reason] || String(reason);
      }
      function buildLegacyServiceHealthFallback() {
        const overview = state.latestOverview || {};
        const governance = state.latestGovernance || {};
        const metrics = state.latestSystemMetrics || {};
        const entries = [{
          service_id: "service_health_bootstrap",
          display_name: "服务健康链路",
          status: "degraded",
          summary: "等待后端权威 service_health 快照",
          evidence: "当前为前端兼容推断，不能作为验收成功状态",
          source: "frontend_fallback",
          advice: "检查 /api/health 与 runtime.governance 是否返回 service_health",
          order: 0,
        }];
        const health = overview.health || {};
        entries.push({
          service_id: "gateway_web_fallback",
          display_name: "网关 Web 服务",
          status: serviceStatusValue(health, metrics.psutil_available === false ? "unsupported" : "available"),
          summary: health.status ? "网关健康流已上报" : "前端已连接调试流",
          evidence: health.status ? serviceEvidenceFromSummary(health, "网关健康流已上报") : "等待 /api/health 首屏健康快照",
          source: "frontend_fallback",
          advice: "异常时查看网关运行日志",
          order: 10,
        });
        const map = overview.map || {};
        entries.push({
          service_id: "map_fallback",
          display_name: "地图服务",
          status: serviceStatusValue(map, "idle"),
          summary: map.status ? "地图流已上报" : "地图服务空闲或未上报",
          evidence: serviceEvidenceFromSummary(map, "地图流已上报"),
          source: "frontend_fallback",
          advice: "需要地图时启动建图或加载地图",
          order: 40,
        });
        const localization = overview.localization || {};
        const session = governance.localization_session || {};
        entries.push({
          service_id: "localization_fallback",
          display_name: "定位服务",
          status: serviceStatusValue(localization, session.status || "idle"),
          summary: localization.status ? "定位流已上报" : "定位服务空闲或未上报",
          evidence: localization.status ? serviceEvidenceFromSummary(localization, "定位流已上报") : (session.available ? "定位会话字段已上报" : "等待定位健康字段"),
          source: "frontend_fallback",
          advice: "检查定位服务和运行日志",
          order: 50,
        });
        const navigation = overview.navigation || {};
        entries.push({
          service_id: "navigation_fallback",
          display_name: "导航/探索服务",
          status: serviceStatusValue(navigation, "idle"),
          summary: navigation.status ? "导航流已上报" : "导航服务空闲或未上报",
          evidence: serviceEvidenceFromSummary(navigation, "导航流已上报"),
          source: "frontend_fallback",
          advice: "检查导航服务和运行日志",
          order: 60,
        });
        const video = overview.video || {};
        const videoProc = processByRole("go2_video_streamer");
        entries.push({
          service_id: "perception_video_fallback",
          display_name: "感知/视频服务",
          status: videoProc ? "available" : serviceStatusValue(video, "idle"),
          summary: videoProc ? "视频推流进程已匹配" : "视频/感知等待上报",
          evidence: videoProc ? `进程运行中 PID ${videoProc.pid}` : serviceEvidenceFromSummary(video, "视频/感知流已上报"),
          source: "frontend_fallback",
          advice: videoProc ? "无需处理" : "检查摄像头输入和感知日志",
          order: 70,
        });
        return {
          status: worstStatus(entries),
          authoritative: false,
          generated_at: new Date().toISOString(),
          entries,
          source: "frontend_fallback",
          metadata: { attention_count: entries.filter((entry) => SERVICE_ATTENTION_STATUSES.has(entry.status)).length },
        };
      }
      function renderServiceStatus() {
        if (!els.governanceMeta) return;
        // 高频状态刷新保留用户已展开的服务及其余列表。
        const openServices = new Set(Array.from(els.governanceMeta.querySelectorAll(".service-health-card[open]"), (card) => card.dataset.serviceId));
        const moreOpen = Boolean(els.governanceMeta.querySelector(".service-health-more[open]"));
        const scrollHost = els.governanceMeta.parentElement;
        const scrollTop = scrollHost.scrollTop;
        els.governanceMeta.replaceChildren();
        const summary = state.latestServiceHealth || buildLegacyServiceHealthFallback();
        const entries = Array.isArray(summary.entries) ? summary.entries : [];
        const status = summary.status || worstStatus(entries);
        const attention = serviceHealthAttentionCount(summary);
        const gatewayService = entries.find((entry) => entry.service_id === "gateway_web")
          || entries.find((entry) => entry.service_id === "gateway_web_fallback")
          || entries.find((entry) => entry.display_name === "网关 Web 服务");
        setText(els.connectionText, displayStatus(gatewayService ? gatewayService.status : "missing"));
        setPill(els.governanceStatus, status, attention ? `${attention} 项需处理` : "服务");
        const needsAttention = (entry) => SERVICE_ATTENTION_STATUSES.has(String(entry.status || "missing").toLowerCase());
        const primary = entries.filter((entry) => needsAttention(entry) || SERVICE_PRIMARY_IDS.has(entry.service_id));
        const secondary = entries.filter((entry) => !needsAttention(entry) && !SERVICE_PRIMARY_IDS.has(entry.service_id));
        primary.sort((a, b) => Number(needsAttention(b)) - Number(needsAttention(a)));
        const options = {
          source: summary.source,
          generated_at: summary.generated_at,
          timestamp: summary.timestamp,
          compatibleFallback: summary.authoritative === false,
          openServices,
        };
        for (const entry of primary) appendServiceHealthCard(entry, options);
        if (secondary.length) {
          const more = document.createElement("details");
          more.className = "service-health-more";
          more.open = moreOpen;
          const label = document.createElement("summary");
          label.textContent = `其余服务（${secondary.length}）`;
          const list = document.createElement("div");
          list.className = "service-health-list";
          for (const entry of secondary) appendServiceHealthCard(entry, { ...options, target: list });
          more.append(label, list);
          els.governanceMeta.appendChild(more);
        }
        scrollHost.scrollTop = scrollTop;
      }
      function formatGB(bytes) {
        const n = Number(bytes);
        if (!Number.isFinite(n) || n < 0) return "--";
        return (n / 1024 / 1024 / 1024).toFixed(1) + " GB";
      }
      function updateSystemMetrics(payload) {
        const snap = payload || {};
        state.latestSystemMetrics = snap;
        const available = snap.psutil_available === true;
        // 顶栏 CPU 块（复用 #networkLatency）
        const cpu = snap.cpu || {};
        setText(els.networkLatency, available && cpu.percent !== undefined ? `${Math.round(cpu.percent)}%` : "N/A");
        // 顶栏 运存块（复用 #resourceMetric）
        const mem = snap.memory || {};
        setText(els.resourceMetric, available && mem.percent !== undefined
          ? `${Math.round(mem.percent)}% · ${formatGB(mem.used_bytes)}/${formatGB(mem.total_bytes)}`
          : "N/A");
        renderProcessMatrix(snap);
        // 任务 4.6：system.metrics 改走 patch 模式（仅刷 process_evidence，不覆盖主字段）
        // state.latestServiceHealth 缺失时（即 bootstrap 与流式都未到）则跳过
        if (state.latestServiceHealth) {
          mergeServiceHealth(state.latestServiceHealth, "system.metrics", "patch");
        }
      }
      function renderProcessMatrix(snap) {
        const status = snap && snap.status ? snap.status : "missing";
        setPill(els.servicesStatus, status);
        if (els.serviceMatrixScope) {
          els.serviceMatrixScope.textContent = snap && snap.monitor_scope
            ? `监控范围：${snap.monitor_scope}` : "";
        }
        els.serviceMatrix.replaceChildren();
        const processes = snap && Array.isArray(snap.processes) ? snap.processes : [];
        if (!processes.length) {
          const empty = document.createElement("div");
          empty.className = "empty panel-meta";
          empty.textContent = snap && snap.psutil_available === false
            ? "psutil 不可用，性能监控降级。" : "未匹配到本项目进程。";
          els.serviceMatrix.appendChild(empty);
          return;
        }
        for (const proc of processes) {
          const row = document.createElement("article");
          row.className = "service-row";
          const left = document.createElement("div");
          const title = document.createElement("div");
          title.textContent = `${safeText(proc.display_name || proc.name)} · PID ${safeText(proc.pid)}`;
          title.className = "mono";
          title.title = proc.cmdline_preview || proc.name || "";
          const meta = document.createElement("div");
          meta.className = "meta";
          meta.textContent = `内存 ${formatGB(proc.rss_bytes)} · CPU ${fmt(proc.cpu_percent, "%")} · 角色 ${safeText(proc.role || proc.matched_keyword)}`;
          left.append(title, meta);
          const pill = document.createElement("span");
          pill.className = "pill mono";
          pill.textContent = `${Math.round(Number(proc.cpu_percent) || 0)}%`;
          row.append(left, pill);
          els.serviceMatrix.appendChild(row);
        }
      }
      function updateHealth(payload) {
        // 闸门：bootstrap 未完成时入队，避免流式帧抢在 /api/health 之前写入 state
        if (!state.bootstrapComplete.health) {
          state.bootstrapQueue.health.push(payload);
          return;
        }
        updateHealthImpl(payload);
      }
      function updateHealthImpl(payload) {
        // 状态流只更新其携带字段，不能清掉首次健康响应中的机器人身份与活动地图。
        const changes = Object.fromEntries(Object.entries(payload || {}).filter(([, value]) => value !== undefined));
        const health = { ...(state.latestHealth || {}), ...changes };
        state.latestHealth = health;
        setText(els.robotName, health.robot_name || "nuwax_robot_bridge");
        setText(els.robotModel, `型号：${safeText(health.robot_model)}`);
        const robot = health.latest_robot_snapshot || {};
        const status = health.assembly_status || {};
        const safety = robot.safety || {};
        const battery = robot.battery || {};
        const controlMode = status.control_mode || robot.mode || "unknown";
        setText(els.runtimeMode, ({unknown: "未知", idle: "空闲", high_level: "高级控制", low_level: "低级控制", navigation: "导航", error: "异常"})[controlMode] || controlMode);
        const rawBattery = Number(battery.percentage);
        const batteryValue = battery.percentage !== undefined
          ? `${Math.round(rawBattery <= 1 ? rawBattery * 100 : rawBattery)}%`
          : (battery.level !== undefined ? `${battery.level}%` : "--");
        setText(els.batteryMetric, batteryValue);
        const canMove = safety.can_move;
        setPill(
          els.motionStatus,
          canMove === true ? "available" : canMove === false ? "stale" : "missing",
        );
        setText(els.motionStatus, canMove === true ? "运动可用" : canMove === false ? "运动不可用" : "运动状态未确认");
        els.motionStatus.title = safety.warning_message || (canMove == null ? "设备尚未提供运动状态确认。" : "");
        if (changes.service_health) {
          mergeServiceHealth(changes.service_health, "/api/health", "replace");
        } else {
          renderServiceStatus();
        }
        renderMapSummary();
      }
      function normalizeTaskState(value) {
        const raw = value && typeof value === "object" && "value" in value ? value.value : value;
        const stateValue = String(raw || "missing").toLowerCase();
        if (stateValue.startsWith("task.")) return stateValue.slice(5);
        return stateValue;
      }
      function isTerminalTaskState(value) {
        return ["succeeded", "failed", "cancelled", "timeout"].includes(normalizeTaskState(value));
      }
      function taskUpdatedMs(task) {
        const value = task && (task.updated_at || task.completed_at || task.started_at || task.timestamp || task.created_at);
        const parsed = value ? Date.parse(value) : 0;
        return Number.isFinite(parsed) ? parsed : 0;
      }
      function taskListFromMap(map, limit) {
        const items = Array.from(map.values()).sort((a, b) => taskUpdatedMs(b) - taskUpdatedMs(a));
        return typeof limit === "number" ? items.slice(0, limit) : items;
      }
      function refreshTaskStore() {
        if (state.taskStoreBatching) return;
        // 任务 4.0a Map 内存截断：保留窗口 active=50 / history=200，防止长会话内存增长
        trimMapByUpdatedAt(state.taskActiveById, 50);
        trimMapByUpdatedAt(state.taskHistoryById, 200);
        const active = taskListFromMap(state.taskActiveById);
        const history = taskListFromMap(state.taskHistoryById, 20);
        state.latestTasks = { active_tasks: active, history_tasks: history };
        updateTasks(state.latestTasks);
        // 任务 4.7：active_task_count 不再触发 service_health 重渲染，只更新顶栏文本
        if (els.activeTaskMetric) setText(els.activeTaskMetric, active.length);
        reevaluateCachedMapDisplay();
      }
      function trimMapByUpdatedAt(map, keep) {
        if (!map || map.size <= keep) return;
        const sorted = Array.from(map.entries()).sort((a, b) => taskUpdatedMs(b[1]) - taskUpdatedMs(a[1]));
        for (const [key] of sorted.slice(keep)) map.delete(key);
      }
      function replaceTaskStore(payload, snapshotSequence = state.taskEventSequence) {
        if (snapshotSequence < state.taskSnapshotSequence) return;
        const hasActiveSnapshot = Array.isArray(payload && payload.active_tasks);
        const active = hasActiveSnapshot ? payload.active_tasks : [];
        const activeIds = new Set(active.filter(task => task?.task_id).map(task => String(task.task_id)));
        // 活动列表为完整权威快照；保留请求开始后才收到的本地事件。
        for (const [key, task] of state.taskActiveById) {
          if (hasActiveSnapshot && !activeIds.has(key) && (task.__streamSequence || 0) <= snapshotSequence) state.taskActiveById.delete(key);
        }
        for (const task of active) {
          if (!task || !task.task_id) continue;
          const key = String(task.task_id);
          const terminal = state.taskHistoryById.get(key);
          if (terminal && isTerminalTaskState(terminal.state || terminal.status)) continue;
          const existing = state.taskActiveById.get(key);
          if (existing && (existing.__streamSequence || 0) > snapshotSequence) continue;
          state.taskActiveById.set(key, { ...existing, ...task });
        }
        for (const task of Array.isArray(payload && payload.history_tasks) ? payload.history_tasks : []) {
          if (!task || !task.task_id) continue;
          const key = String(task.task_id);
          const existing = state.taskHistoryById.get(key);
          if (isTerminalTaskState(task.state || task.status)) state.taskActiveById.delete(key);
          if (!existing) {
            state.taskHistoryById.set(key, task);
          } else if ((existing.__streamSequence || 0) <= snapshotSequence && taskUpdatedMs(task) >= taskUpdatedMs(existing)) {
            state.taskHistoryById.set(key, { ...existing, ...task });
          }
        }
        if (hasActiveSnapshot) state.taskSnapshotSequence = snapshotSequence;
        refreshTaskStore();
      }
      function updateTasks(payload) {
        const active = Array.isArray(payload && payload.active_tasks) ? payload.active_tasks : [];
        const history = Array.isArray(payload && payload.history_tasks) ? payload.history_tasks : [];
        state.latestTasks = payload || {};
        els.taskSummary.replaceChildren();
        setPill(els.taskStatus, active.length ? "available" : "idle", active.length ? `${active.length} 个任务` : "空闲");
        const recentFailures = history.filter((task) => ["failed", "timeout"].includes(normalizeTaskState(task.state || task.status))).slice(0, 2);
        const items = active.length
          ? [...active.slice(0, 4), ...recentFailures.filter((task) => !active.some((item) => String(item.task_id) === String(task.task_id)))]
          : history.slice(0, 4);
        if (!items.length) {
          const row = document.createElement("article");
          row.className = "task-row";
          row.innerHTML = '<div><div class="mono">暂无运行任务</div><div class="meta">当前没有活动任务；历史任务为空。</div></div><span class="pill stale">空闲</span>';
          els.taskSummary.appendChild(row);
          renderServiceStatus();
          return;
        }
        for (const task of items) {
          const row = document.createElement("article");
          row.className = "task-row";
          const left = document.createElement("div");
          const title = document.createElement("div");
          title.className = "mono";
          const taskId = safeText(task.task_id || task.id);
          title.textContent = task.capability_name ? `${safeText(task.capability_name)} · ${taskId}` : taskId;
          title.title = taskId;
          const meta = document.createElement("div");
          meta.className = "meta";
          const taskState = normalizeTaskState(task.state || task.status);
          const reason = task.failure_reason || task.waiting_for || task.message || task.error || (["failed", "timeout", "cancelled"].includes(taskState) ? "未上报原因" : "");
          const progressNumber = Number(task.progress);
          const progressPercent = Number.isFinite(Number(task.progress_percent))
            ? Math.round(Number(task.progress_percent))
            : Number.isFinite(progressNumber)
              ? Math.round((progressNumber >= 0 && progressNumber <= 1 ? progressNumber * 100 : progressNumber))
              : null;
          const progressText = progressPercent === null ? "进度未上报" : `进度 ${progressPercent}%`;
          const timeText = task.updated_at || task.completed_at || task.started_at
            ? `更新 ${shortTime(task.updated_at || task.completed_at || task.started_at)}`
            : "时间未上报";
          const nextAction = task.next_action ? ` · 下一步 ${safeText(task.next_action)}` : "";
          const retryText = task.retryable === true ? " · 可重试" : task.retryable === false && ["failed", "timeout"].includes(taskState) ? " · 不建议原样重试" : "";
          meta.textContent = `阶段 ${safeText(task.stage || taskState)} · ${progressText} · ${timeText}${reason ? " · " + reason : ""}${retryText}${nextAction}`;
          left.append(title, meta);
          const pill = document.createElement("span");
          pill.className = `pill ${taskState === "failed" || taskState === "timeout" ? "error" : taskState === "running" || taskState === "accepted" ? "available" : taskState === "succeeded" ? "available" : "stale"}`;
          pill.textContent = displayValue(taskState, "task_state");
          pill.title = taskState;
          row.append(left, pill);
          els.taskSummary.appendChild(row);
        }
        renderServiceStatus();
      }
      async function loadInitialTasks() {
        if (state.taskLoadPromise) {
          state.taskReloadPending = true;
          return state.taskLoadPromise;
        }
        const snapshotSequence = state.taskEventSequence;
        state.taskLoadPromise = (async () => {
          try {
            const response = await fetch("/api/tasks?history_limit=20");
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            const snapshot = await response.json();
            if (!Array.isArray(snapshot.active_tasks) || !Array.isArray(snapshot.history_tasks)) throw new Error("任务快照字段不完整");
            replaceTaskStore(snapshot, snapshotSequence);
          } catch (err) {
            showPanelError("tasks", `任务列表读取失败：${err.message || err}`);
          } finally {
            state.bootstrapComplete.tasks = true;
            flushTaskQueue();
          }
        })();
        try { await state.taskLoadPromise; }
        finally {
          state.taskLoadPromise = null;
          if (state.taskReloadPending) {
            state.taskReloadPending = false;
            void loadInitialTasks();
          }
        }
      }
      async function loadInitialHealth() {
        try {
          const response = await fetch("/api/health");
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          updateHealth(await response.json());
        } catch (err) {
          showPanelError("governance", `服务健康初始化失败：${err.message || err}`);
        } finally {
          // 风险 4 不变量：bootstrapComplete 必须在 resolve/catch 后无条件写 true，
          // 让 flush 队列中后续 runtime.governance 帧能继续触发 merge。
          flushHealthQueue();
          state.bootstrapComplete.health = true;
        }
      }
      function flushHealthQueue() {
        const queue = state.bootstrapQueue.health;
        for (const queued of queue) updateHealthImpl(queued);
        state.bootstrapQueue.health = [];
      }
      function flushTaskQueue() {
        const queue = state.bootstrapQueue.tasks;
        for (const frame of queue) updateTaskFrameImpl(frame);
        state.bootstrapQueue.tasks = [];
      }
      // 任务 4.0：删除 startTaskPolling —— tasks.state WebSocket 流为单一来源；
      // 轮询 replaceTaskStore 会反复擦除流式期间累积的增量。
      function showPanelError(kind, message) {
        if (kind === "governance") {
          setPill(els.governanceStatus, "error");
          els.governanceMeta.replaceChildren();
          appendServiceHealthCard({
            service_id: "service_health_error",
            display_name: "服务健康链路",
            status: "error",
            summary: "服务健康数据读取失败",
            evidence: message,
            source: "frontend_fallback",
            advice: "检查 /api/health、runtime.governance 和网关运行日志",
            updated_at: new Date().toISOString(),
          }, { compatibleFallback: true, source: "frontend_fallback" });
        } else if (kind === "map") {
          setPill(els.mapStatus, "error");
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = message;
        } else if (kind === "localization") {
          setPill(els.localizationStatus, "error");
          setKv(els.localizationMeta, [["错误", message], ["前端接收", nowText()]]);
        } else if (kind === "navigation") {
          setPill(els.navigationStatus, "error");
          setKv(els.navigationMeta, [["错误", message], ["前端接收", nowText()]]);
        } else if (kind === "video") {
          setPill(els.videoStatus, "error");
          els.videoEmpty.hidden = false;
          els.videoEmpty.textContent = message;
        } else if (kind === "tasks") {
          setPill(els.taskStatus, "error", "任务链路异常");
          els.taskSummary.replaceChildren();
          const row = document.createElement("article");
          row.className = "task-row";
          const detail = document.createElement("div");
          const title = document.createElement("div");
          title.className = "mono";
          title.textContent = "任务状态读取失败";
          const meta = document.createElement("div");
          meta.className = "meta";
          meta.textContent = `${message} · 下一步：检查 /api/tasks 与实时流连接，系统会自动重试对账。`;
          detail.append(title, meta);
          row.appendChild(detail);
          els.taskSummary.appendChild(row);
        } else if (kind === "logs") {
          setPill(els.logsStatus, "error");
          setText(document.getElementById("logMessage"), `${message} 历史日志仍可查询，实时流恢复后会自动补充。`);
        }
      }
      function normalizeFrameId(value, fallback) {
        const raw = String(value || fallback || "map").trim().replace(/^\/+/, "");
        return raw || "map";
      }
      function gridResolution(grid) {
        if (!grid) return 1;
        const candidates = [grid.resolution_m, grid.resolution, grid.metadata && grid.metadata.resolution];
        for (const value of candidates) {
          const number = Number(value);
          if (Number.isFinite(number) && number > 0) return number;
        }
        return 1;
      }
      function originPosition(grid) {
        const origin = grid && grid.origin;
        if (!origin) return { x: 0, y: 0, z: 0 };
        if (origin.position) return origin.position;
        if (origin.x !== undefined && origin.y !== undefined) {
          return { x: Number(origin.x), y: Number(origin.y), z: Number(origin.z || 0) };
        }
        return { x: 0, y: 0, z: 0 };
      }
      function originYaw(grid) { return planarYawFromPose(grid && grid.origin) || 0; }
      function pointFromPose(pose) {
        const pos = pose && pose.position;
        if (!pos) return null;
        const x = Number(pos.x);
        const y = Number(pos.y);
        if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
        return { x, y, z: Number(pos.z || 0) };
      }
      function yawFromPose(pose) {
        const q = pose && pose.orientation;
        if (!q) return null;
        const siny = 2 * ((Number(q.w) || 0) * (Number(q.z) || 0) + (Number(q.x) || 0) * (Number(q.y) || 0));
        const cosy = 1 - 2 * ((Number(q.y) || 0) * (Number(q.y) || 0) + (Number(q.z) || 0) * (Number(q.z) || 0));
        return Math.atan2(siny, cosy);
      }
      function planarYawFromPose(pose) {
        const q = pose && pose.orientation;
        if (!q) return yawFromPose(pose);
        const rawX = Number(q.x);
        const rawY = Number(q.y);
        const rawZ = Number(q.z);
        const rawW = Number(q.w);
        const x = Number.isFinite(rawX) ? rawX : 0;
        const y = Number.isFinite(rawY) ? rawY : 0;
        const z = Number.isFinite(rawZ) ? rawZ : 0;
        const w = Number.isFinite(rawW) ? rawW : 1;
        const forwardX = 1 - 2 * (y * y + z * z);
        const forwardY = 2 * (x * y + w * z);
        if (Math.hypot(forwardX, forwardY) < 1e-6) return yawFromPose(pose);
        return Math.atan2(forwardY, forwardX);
      }
      function mapCorners(grid) {
        if (!grid || !grid.width || !grid.height) return [];
        const origin = originPosition(grid);
        const yaw = originYaw(grid);
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        const resolution = gridResolution(grid);
        const width = Number(grid.width) * resolution;
        const height = Number(grid.height) * resolution;
        return [[0, 0], [width, 0], [width, height], [0, height]].map(([x, y]) => ({
          x: Number(origin.x || 0) + x * cos - y * sin,
          y: Number(origin.y || 0) + x * sin + y * cos,
        }));
      }
      function isMapToolActive() {
        return Boolean(state.goalPickMode || state.publishPointMode || state.initialPosePickMode);
      }
      function usesRvizMapView() {
        return Boolean(
          state.map2dCameraMode === "free"
          || isMapToolActive()
          || state.goalPickDrag
          || state.initialPosePickDrag
          || state.mapPanDrag,
        );
      }
      function activeMapWorldView() {
        return state.map2dCameraMode === "free" ? state.mapFreeView : state.mapRvizView;
      }
      function mapViewZoom() {
        return usesRvizMapView() ? (activeMapWorldView().zoom || 1) : state.mapZoom;
      }
      function defaultMapViewCenter() {
        const posePoint = pointFromPose(state.pose);
        if (posePoint) return { x: posePoint.x, y: posePoint.y };
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return { x: 0, y: 0 };
        return corners.reduce((acc, item) => ({ x: acc.x + item.x / corners.length, y: acc.y + item.y / corners.length }), { x: 0, y: 0 });
      }
      function getMapRvizViewCenter() {
        const center = activeMapWorldView().center;
        if (center && Number.isFinite(Number(center.x)) && Number.isFinite(Number(center.y))) {
          return { x: Number(center.x), y: Number(center.y) };
        }
        return defaultMapViewCenter();
      }
      function resetRvizMapView(focusPose) {
        const posePoint = focusPose || pointFromPose(state.pose);
        state.mapRvizView = {
          center: posePoint ? { x: posePoint.x, y: posePoint.y } : defaultMapViewCenter(),
          zoom: 1,
        };
      }
      function resetFreeMapView() {
        const corners = mapCorners(state.mapGrid);
        const center = corners.length
          ? corners.reduce((sum, point) => ({ x: sum.x + point.x / corners.length, y: sum.y + point.y / corners.length }), { x: 0, y: 0 })
          : defaultMapViewCenter();
        state.mapFreeView = { center, zoom: 1, fitSpan: null };
        if (window.__nuwaxMapFreeze) window.__nuwaxMapFreezeCenter = { ...center };
        if (state.mapGrid) basePixelsPerMeterRviz();
      }
      function syncMap2dCameraControls() {
        const button = els.map2dCameraToggle;
        if (!button) return;
        const free = state.map2dCameraMode === "free";
        const busy = Boolean(isMapToolActive() || state.mapPanDrag || window.__nuwaxMapToolV2?.active());
        button.textContent = free ? "自由拖拽" : "机器人跟随";
        button.classList.toggle("active", free);
        button.setAttribute("aria-pressed", free ? "true" : "false");
        button.disabled = busy;
        button.title = busy
          ? "结束选点、绘制或拖拽后可切换二维视角"
          : free
            ? "当前自由视角：拖拽平移、滚轮缩放；点击切回机器人居中朝上的跟随视角"
            : "当前跟随机器人：箭头固定居中朝上，地图随机器人移动和转向；点击切换自由拖拽";
      }
      function setMap2dCameraMode(mode) {
        if (isMapToolActive() || state.mapPanDrag || window.__nuwaxMapToolV2?.active()) return;
        state.map2dCameraMode = mode === "free" ? "free" : "follow";
        if (state.map2dCameraMode === "free" && !state.mapFreeView.center) resetFreeMapView();
        syncMapToolUi();
        requestMapDraw(false);
      }
      function centerRvizMapOnRobot() {
        const posePoint = pointFromPose(state.pose);
        if (!posePoint) return;
        activeMapWorldView().center = { x: posePoint.x, y: posePoint.y };
        if (state.map2dCameraMode === "free" && window.__nuwaxMapFreeze) {
          window.__nuwaxMapFreezeCenter = { ...activeMapWorldView().center };
        }
        requestMapDraw(false);
      }
      function cameraAnchor() {
        // V2 boundary draw freeze：划线模式下冻结相机，强制走 mapFixed 路径——
        // canvas 不再 robotLocked 跟随 pose 旋转/平移，地图视觉静止。
        // 冻结中心由 boundary_draw.js 在 setDrawing(true) 时写入当前 anchor.point；
        // 解冻时清掉（false/未设）：原路径不变。
        if (window.__nuwaxMapFreeze && window.__nuwaxMapFreezeCenter) {
          const fc = window.__nuwaxMapFreezeCenter;
          return {
            point: { x: Number(fc.x) || 0, y: Number(fc.y) || 0, z: 0 },
            yaw: 0,
            locked: false,
            mapFixed: true,
          };
        }
        if (usesRvizMapView()) {
          const center = getMapRvizViewCenter();
          return { point: { x: center.x, y: center.y, z: 0 }, yaw: 0, locked: false, mapFixed: true };
        }
        const posePoint = pointFromPose(state.pose);
        if (posePoint) return { point: posePoint, yaw: planarYawFromPose(state.pose) || 0, locked: true, mapFixed: false };
        const fallback = defaultMapViewCenter();
        return { point: { x: fallback.x, y: fallback.y, z: 0 }, yaw: 0, locked: false, mapFixed: false };
      }
      function basePixelsPerMeterRviz() {
        const canvas = els.mapCanvas;
        const fitSpan = state.map2dCameraMode === "free" && state.mapFreeView.fitSpan;
        if (fitSpan) return Math.max(4, Math.min(canvas.width / fitSpan.x, canvas.height / fitSpan.y));
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return 60;
        let minX = Infinity;
        let maxX = -Infinity;
        let minY = Infinity;
        let maxY = -Infinity;
        for (const corner of corners) {
          minX = Math.min(minX, Number(corner.x));
          maxX = Math.max(maxX, Number(corner.x));
          minY = Math.min(minY, Number(corner.y));
          maxY = Math.max(maxY, Number(corner.y));
        }
        const spanX = Math.max(maxX - minX, 1);
        const spanY = Math.max(maxY - minY, 1);
        // 自由视角只在首次入图、切图或明确复位时适配，地图增量不能推动用户镜头。
        if (state.map2dCameraMode === "free") state.mapFreeView.fitSpan = { x: spanX * 1.12, y: spanY * 1.12 };
        return Math.max(4, Math.min(canvas.width / (spanX * 1.12), canvas.height / (spanY * 1.12)));
      }
      function basePixelsPerMeter(anchor) {
        if (anchor && anchor.mapFixed) return basePixelsPerMeterRviz();
        const canvas = els.mapCanvas;
        const corners = mapCorners(state.mapGrid);
        if (!corners.length) return 60;
        let maxX = 1;
        let maxY = 1;
        const yaw = anchor.yaw || 0;
        const cos = Math.cos(yaw);
        const sin = Math.sin(yaw);
        for (const corner of corners) {
          const dx = Number(corner.x) - anchor.point.x;
          const dy = Number(corner.y) - anchor.point.y;
          const right = sin * dx - cos * dy;
          const up = cos * dx + sin * dy;
          maxX = Math.max(maxX, Math.abs(right));
          maxY = Math.max(maxY, Math.abs(up));
        }
        return Math.max(4, Math.min(canvas.width / (maxX * 2.16), canvas.height / (maxY * 2.16)));
      }
      function cameraScale(anchor) {
        return basePixelsPerMeter(anchor) * mapViewZoom();
      }
      function applyCameraTransform(ctx, anchor, scale) {
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        if (anchor && anchor.mapFixed) {
          const ax = Number(anchor.point.x) || 0;
          const ay = Number(anchor.point.y) || 0;
          ctx.setTransform(scale, 0, 0, -scale, centerX - scale * ax, centerY + scale * ay);
          return;
        }
        const yaw = anchor.yaw || 0;
        const sin = Math.sin(yaw);
        const cos = Math.cos(yaw);
        const a = scale * sin;
        const b = -scale * cos;
        const c = -scale * cos;
        const d = -scale * sin;
        const e = centerX - a * anchor.point.x - c * anchor.point.y;
        const f = centerY - b * anchor.point.x - d * anchor.point.y;
        ctx.setTransform(a, b, c, d, e, f);
      }
      function mapToCanvas(point, anchorOverride, scaleOverride) {
        if (!point) return null;
        const anchor = anchorOverride || cameraAnchor();
        const scale = scaleOverride || cameraScale(anchor);
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        const dx = Number(point.x) - Number(anchor.point.x);
        const dy = Number(point.y) - Number(anchor.point.y);
        if (anchor.mapFixed) {
          return { x: centerX + dx * scale, y: centerY - dy * scale };
        }
        const yaw = anchor.yaw || 0;
        return {
          x: centerX + (Math.sin(yaw) * dx - Math.cos(yaw) * dy) * scale,
          y: centerY - (Math.cos(yaw) * dx + Math.sin(yaw) * dy) * scale,
        };
      }
      function worldToCanvasMatrix(anchor, scale) {
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        if (anchor && anchor.mapFixed) {
          const ax = Number(anchor.point.x) || 0;
          const ay = Number(anchor.point.y) || 0;
          return { a: scale, b: 0, c: 0, d: -scale, e: centerX - scale * ax, f: centerY + scale * ay };
        }
        const yaw = anchor.yaw || 0;
        const sin = Math.sin(yaw);
        const cos = Math.cos(yaw);
        const a = scale * sin;
        const b = -scale * cos;
        const c = -scale * cos;
        const d = -scale * sin;
        const e = centerX - a * anchor.point.x - c * anchor.point.y;
        const f = centerY - b * anchor.point.x - d * anchor.point.y;
        return { a, b, c, d, e, f };
      }
      function canvasToMap(px, py, anchorOverride) {
        const anchor = anchorOverride || cameraAnchor();
        const scale = cameraScale(anchor);
        if (!Number.isFinite(scale) || scale <= 0) {
          return { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
        }
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        if (anchor.mapFixed) {
          return {
            x: Number(anchor.point.x) + (px - centerX) / scale,
            y: Number(anchor.point.y) - (py - centerY) / scale,
          };
        }
        const yaw = anchor.yaw || 0;
        const { a, b, c, d, e, f } = worldToCanvasMatrix(anchor, scale);
        const det = a * d - b * c;
        if (!Number.isFinite(det) || Math.abs(det) < 1e-12) {
          return { x: Number(anchor.point.x) || 0, y: Number(anchor.point.y) || 0 };
        }
        const sx = px - e;
        const sy = py - f;
        return {
          x: (d * sx - c * sy) / det,
          y: (-b * sx + a * sy) / det,
        };
      }
      function zoomRvizMapAt(pointer, factor) {
        const anchor = cameraAnchor();
        const scaleBefore = cameraScale(anchor);
        if (!Number.isFinite(scaleBefore) || scaleBefore <= 0) return;
        const world = canvasToMap(pointer.x, pointer.y, anchor);
        const nextZoom = Math.min(8, Math.max(0.25, mapViewZoom() * factor));
        activeMapWorldView().zoom = nextZoom;
        const anchorAfter = cameraAnchor();
        const scaleAfter = cameraScale(anchorAfter);
        const centerX = els.mapCanvas.width / 2;
        const centerY = els.mapCanvas.height / 2;
        activeMapWorldView().center = {
          x: world.x - (pointer.x - centerX) / scaleAfter,
          y: world.y + (pointer.y - centerY) / scaleAfter,
        };
        if (state.map2dCameraMode === "free" && window.__nuwaxMapFreeze) {
          window.__nuwaxMapFreezeCenter = { ...activeMapWorldView().center };
        }
      }
      function formatMapPoint(point) {
        if (!point) return "--";
        const height = typeof point.z === "number" && Number.isFinite(point.z) ? `, ${point.z.toFixed(2)}` : "";
        return `(${Number(point.x).toFixed(2)}, ${Number(point.y).toFixed(2)}${height})`;
      }
      function planarDistanceMeters(a, b) {
        if (!a || !b) return null;
        const dx = Number(a.x) - Number(b.x);
        const dy = Number(a.y) - Number(b.y);
        if (!Number.isFinite(dx) || !Number.isFinite(dy)) return null;
        return Math.hypot(dx, dy);
      }
      function navGoalMatchesPending(targetPose, pending) {
        if (!targetPose || !pending) return false;
        const point = pointFromPose(targetPose);
        if (!point) return false;
        const dist = planarDistanceMeters(point, pending);
        const z = targetPose.position && targetPose.position.z;
        const frame = String(targetPose.frame_id || "").trim().replace(/^\/+/, "");
        const pendingFrame = String(pending.frame_id || "").trim().replace(/^\/+/, "");
        return typeof z === "number" && Number.isFinite(z)
          && typeof pending.z === "number" && Number.isFinite(pending.z)
          && frame !== "" && frame === pendingFrame
          && dist !== null && Math.hypot(dist, z - pending.z) < 0.35;
      }
      function syncDebugNavGoalFromStream(navPayload) {
        const pending = state.pendingNavGoal;
        if (!pending || pending.status === "sending") return;
        const navState = getNestedState(navPayload, "navigation_state");
        if (!navState) return;
        const status = String(navState.status || "").toLowerCase();
        const metadata = navState.metadata || {};
        const targetPose = metadata.target_pose || null;
        const matches = navGoalMatchesPending(targetPose, pending);
        if (status === "running" || status === "planning") {
          if (!matches) return;
          const remain = navState.remaining_distance_m;
          const remainLabel = remain === null || remain === undefined ? "" : ` · 剩余 ${Number(remain).toFixed(2)}m`;
          updateGoalPickStatus(`导航执行中 → ${formatMapPoint(pending)}${remainLabel}`, "ok");
          pending.status = "accepted";
          return;
        }
        if (status === "succeeded") {
          if (!matches) return;
          updateGoalPickStatus(`已到达 ${formatMapPoint(pending)}`, "ok");
          state.pendingNavGoal = null;
          requestMapDraw(false);
          return;
        }
        if (status === "failed" || status === "cancelled") {
          if (!matches) return;
          const message = navState.message || (status === "cancelled" ? "导航已取消" : "导航失败");
          const failureReason = String(metadata.failure_reason || "");
          if (
            failureReason === "navigation_not_ready"
            || message.includes("nav_runtime 未 ready")
            || message.includes("lifecycle 未全部 active")
            || message.includes("costmap 尚未首帧")
          ) {
            return;
          }
          pending.status = "error";
          pending.message = message;
          updateGoalPickStatus(`${message}（目标 ${formatMapPoint(pending)}）`, "error");
          requestMapDraw(false);
        }
      }
      function mapGoalUsesOfflineGrid() {
        const diagnostics = state.mapStream && state.mapStream.diagnostics ? state.mapStream.diagnostics : {};
        // 定位模式固定 checkpoint 仅作 Debug 底图；导航仍使用已加载静态图。
        if (shouldPinLocalizationCheckpointDisplay(diagnostics)) return false;
        if (activeLocalizationSessionReason()) {
          const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
          const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
          if (sourceKind === "map_asset_bundle" || snapshotKind === "offline_checkpoint") {
            return false;
          }
        }
        const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
        const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
        return sourceKind === "map_asset_bundle" || snapshotKind === "offline_checkpoint";
      }
      function canvasPointer(event) {
        const rect = els.mapCanvas.getBoundingClientRect();
        const scaleX = els.mapCanvas.width / Math.max(rect.width, 1);
        const scaleY = els.mapCanvas.height / Math.max(rect.height, 1);
        return {
          x: (event.clientX - rect.left) * scaleX,
          y: (event.clientY - rect.top) * scaleY,
        };
      }
      function firstNonBlank(...values) {
        for (const value of values) {
          const text = String(value === null || value === undefined ? "" : value).trim();
          if (text) return text;
        }
        return null;
      }
      function pointcloudMapSummary() {
        const frame = state.latestStreamFrames["pointcloud_map.meta"] || {};
        const payload = frame.payload && typeof frame.payload === "object" ? frame.payload : {};
        const metadata = frame.metadata && typeof frame.metadata === "object" ? frame.metadata : {};
        return { frame, payload, metadata };
      }
      function activeMapName() {
        const diagnostics = state.mapStream && state.mapStream.diagnostics ? state.mapStream.diagnostics : {};
        const governance = state.latestGovernance || {};
        const localizationSession = governance.localization_session || {};
        const governanceMetadata = governance.metadata || {};
        const liveDiagnostics = governanceMetadata.live_mapping_diagnostics || {};
        const workspace = governance.active_workspace || {};
        const healthWorkspace = state.latestHealth?.active_map_workspace || {};
        const pointcloud = pointcloudMapSummary().payload;
        const exploration = state.latestStreamFrames["exploration.state"]?.payload || {};
        const explorationRequest = exploration.current_request || exploration.exploration_request || {};
        const navigation = state.latestStreamFrames["navigation.state"]?.payload || {};
        const navigationGoal = navigation.current_goal || {};
        const mapData = state.lastMapEnvelope?.data || {};
        return firstNonBlank(
          mapData.map_name,
          mapData.map_snapshot?.map_name,
          diagnostics.active_map_name,
          diagnostics.map_name,
          governanceMetadata.map_name,
          pointcloud.map_name,
          localizationSession.map_name,
          liveDiagnostics.active_map_name,
          liveDiagnostics.map_name,
          workspace.active_map_name,
          governance.active_map_name,
          governance.map_name,
          explorationRequest.map_name,
          navigationGoal.map_name,
          healthWorkspace.map_name,
        );
      }
      window.__nuwaxActiveMapName = activeMapName;
      function formatPoseMetric(value) {
        const number = Number(value);
        return Number.isFinite(number) ? `${number.toFixed(2)} m` : "--";
      }
      function updateMapPoseHud() {
        const point = pointFromPose(state.latestPose);
        const yaw = planarYawFromPose(state.latestPose);
        setKv(els.mapPoseMeta, [
          ["X", point ? formatPoseMetric(point.x) : "--"],
          ["Y", point ? formatPoseMetric(point.y) : "--"],
          ["Z", point ? formatPoseMetric(point.z) : "--"],
          ["偏航角", Number.isFinite(yaw) ? `${(yaw * 180 / Math.PI).toFixed(1)}°` : "--"],
        ]);
      }
      function renderMapSummary(options) {
        const detail = options || {};
        const is3d = state.mapViewMode === "3d";
        const grid = detail.grid !== undefined ? detail.grid : state.mapGrid;
        const diagnostics = detail.diagnostics || (state.mapStream && state.mapStream.diagnostics) || {};
        const envelope = detail.envelope || state.lastMapEnvelope || {};
        const rows = [["地图名称", activeMapName() || "--"]];
        if (is3d) {
          const pointcloud = pointcloudMapSummary();
          const sourcePointCount = state.pointcloud3dPointCountKnown
            ? Math.max(0, Math.floor(Number(state.pointcloud3dSourcePoints) || 0))
            : null;
          const displayedPointCount = state.pointcloud3dPointCountKnown
            ? Math.max(0, Math.floor(Number(state.pointcloud3dDisplayedPoints) || 0))
            : null;
          rows.push(
            ["点云总数", sourcePointCount === null ? "--" : sourcePointCount.toLocaleString("zh-CN")],
            ["坐标系", pointcloud.payload.frame_id || pointcloud.frame.frame_id || pointcloud.metadata.frame_id || "--"],
            ["显示状态", displayedPointCount === null ? "等待点云" : displayedPointCount > 0 ? "已显示" : "暂无点云"],
          );
          if (sourcePointCount !== displayedPointCount) {
            rows.splice(2, 0, ["显示点数", displayedPointCount === null ? "--" : displayedPointCount.toLocaleString("zh-CN")]);
          }
        } else {
          const width = Number(grid && grid.width);
          const height = Number(grid && grid.height);
          const hasGrid = Number.isFinite(width) && width > 0 && Number.isFinite(height) && height > 0;
          rows.push(
            ["总栅格数", hasGrid ? (width * height).toLocaleString("zh-CN") : "--"],
            ["坐标系", (grid && grid.frame_id) || envelope.frame_id || diagnostics.frame_id || "--"],
            ["尺寸", hasGrid ? `${width} x ${height}` : "--"],
            ["分辨率", hasGrid ? formatPoseMetric(gridResolution(grid)) : "--"],
          );
          if (detail.displayStatus) rows.push(["显示状态", detail.displayStatus]);
          if (detail.message) rows.push(["原因", detail.message]);
        }
        if (els.mapSummaryMode) els.mapSummaryMode.textContent = is3d ? "3D 点云" : "2D 栅格";
        setKv(els.mapMeta, rows);
      }
      function navigationFrameId() {
        const poseFrame = state.pose && state.pose.frame_id;
        const gridFrame = state.mapGrid && state.mapGrid.frame_id;
        return normalizeFrameId(poseFrame || gridFrame, "map");
      }
      function currentNavigationBodyPose(frameId) {
        const envelope = state.lastLocalizationEnvelope;
        const pose = state.latestPose;
        const position = pose && pose.position;
        if (!envelope || statusOf(envelope) !== "available" || envelope.stale === true || !position) return null;
        if (![position.x, position.y, position.z].every(value => typeof value === "number" && Number.isFinite(value))) return null;
        const poseFrame = String(pose.frame_id || envelope.frame_id || "").trim().replace(/^\/+/, "");
        const targetFrame = String(frameId || "").trim().replace(/^\/+/, "");
        if (!poseFrame || poseFrame !== targetFrame) return null;
        // 只读当前真实定位帧；显示缓存重绘不能刷新定位的采样时间和接收时间。
        const source = pose.timestamp;
        const sourceMs = typeof source === "number" ? source * (source < 1e12 ? 1000 : 1) : Date.parse(source);
        const receivedMs = Date.parse(envelope.__receivedAt);
        const now = Date.now();
        if (![sourceMs, receivedMs].every(value => Number.isFinite(value) && now - value >= -1000 && now - value <= 2000)) return null;
        return { x: position.x, y: position.y, z: position.z, frame_id: poseFrame };
      }
      function resolveNavigationHeight(input, frameId) {
        if (input && input.validity && input.validity.badInput) {
          throw new Error("目标机体高度无效，请填写有限数值（米）。");
        }
        const value = input ? String(input.value || "").trim() : "";
        if (value) {
          const height = Number(value);
          if (!Number.isFinite(height)) throw new Error("目标机体高度无效，请填写有限数值（米）。");
          return height;
        }
        const current = currentNavigationBodyPose(frameId);
        if (!current) throw new Error("当前机体高度不可用或已过期，请填写目标在地图中的机体高度（米）。");
        return current.z;
      }
      function updateGoalPickStatus(text, tone) {
        if (!els.mapGoalStatus) return;
        const value = String(text || "").trim();
        if (!value) {
          els.mapGoalStatus.hidden = true;
          els.mapGoalStatus.textContent = "";
          els.mapGoalStatus.className = "map-chip map-goal-chip";
          return;
        }
        els.mapGoalStatus.hidden = false;
        els.mapGoalStatus.textContent = value;
        els.mapGoalStatus.className = `map-chip map-goal-chip${tone ? ` ${tone}` : ""}`;
      }
      function syncMapToolUi() {
        const goalActive = Boolean(state.goalPickMode);
        const pointActive = Boolean(state.publishPointMode);
        const initialPoseActive = Boolean(state.initialPosePickMode);
        const toolActive = goalActive || pointActive || initialPoseActive;
        const free = state.map2dCameraMode === "free";
        syncMap2dCameraControls();
        if (els.mapInitialPoseToggle) els.mapInitialPoseToggle.classList.toggle("active", initialPoseActive);
        if (els.mapGoalToggle) els.mapGoalToggle.classList.toggle("active", goalActive);
        if (els.mapPublishPointToggle) els.mapPublishPointToggle.classList.toggle("active", pointActive);
        if (els.mapCanvas) {
          els.mapCanvas.classList.toggle("goal-pick", toolActive);
          els.mapCanvas.classList.toggle("publish-point", pointActive && !goalActive && !initialPoseActive);
          els.mapCanvas.classList.toggle("initial-pose", initialPoseActive && !goalActive && !pointActive);
          els.mapCanvas.classList.toggle("pan-ready", (toolActive || free) && !state.mapPanDrag);
          els.mapCanvas.classList.toggle("panning", Boolean(state.mapPanDrag));
        }
        if (els.mapCenterRobot) els.mapCenterRobot.hidden = !toolActive && !free;
        if (els.mapRvizHint) {
          els.mapRvizHint.hidden = !toolActive && !free;
          if (initialPoseActive) {
            els.mapRvizHint.textContent = "2D Pose：左键拖拽设初始位姿 · 中/右键平移 · 滚轮缩放";
          } else if (goalActive) {
            els.mapRvizHint.textContent = "2D Goal：左键拖拽设目标 · 中/右键平移 · 滚轮缩放";
          } else if (pointActive) {
            els.mapRvizHint.textContent = "选点：左键发布 /clicked_point · 中/右键平移 · 滚轮缩放";
          } else if (free) {
            els.mapRvizHint.textContent = "自由视角：拖拽平移 · 滚轮缩放 · 复位适配完整地图";
          }
        }
      }
      const syncGoalPickUi = syncMapToolUi;
      function goalPickYawFromDrag(drag) {
        const pointerDx = drag.pointerCurrent.x - drag.pointerStart.x;
        const pointerDy = drag.pointerCurrent.y - drag.pointerStart.y;
        if (Math.hypot(pointerDx, pointerDy) < 8) {
          return planarYawFromPose(state.pose) || 0;
        }
        const mapDx = drag.current.x - drag.start.x;
        const mapDy = drag.current.y - drag.start.y;
        return Math.atan2(mapDy, mapDx);
      }
      function formatNavGoalError(payload, status) {
        const code = String((payload && payload.code) || "").trim();
        const message = String((payload && payload.message) || "").trim();
        if (code === "localization_unavailable") {
          return "平台定位会话未就绪（与 go2.sh 模式无关）。mapping / explore / localization 任一模式均可导航，但需先 activate_map，且定位 session 进入 ready。";
        }
        if (code === "map_not_activated") {
          return "未激活地图。请先调用 activate_map 选择地图。";
        }
        if (code === "access_denied") {
          return "当前令牌无权调用 navigate_to_pose。";
        }
        if (code === "navigation_not_ready") {
          const waitingFor = payload && payload.details && payload.details.waiting_for;
          return waitingFor
            ? `导航未就绪（等待 ${waitingFor}）。${message || ""}`
            : (message || "导航未就绪，无法导航。");
        }
        if (code === "goal_pose_publish_failed" || code === "goal_pose_publish_unsupported") {
          return message || "发布 /goal_pose 失败。";
        }
        if (code === "initial_pose_publish_failed" || code === "initial_pose_publish_unsupported") {
          return message || "发布 /initialpose 失败。";
        }
        if (code === "runtime_provider_unavailable") {
          return message || "导航运行时未初始化。";
        }
        return message || `HTTP ${status}`;
      }
      async function dispatchNavGoal(mapPoint, yawRad) {
        if (!state.mapGrid) {
          updateGoalPickStatus("地图未显示，无法下发导航目标", "error");
          return;
        }
        if (!state.pose || !pointFromPose(state.pose)) {
          updateGoalPickStatus("定位位姿不可用，无法将点击坐标变换到 map 系", "error");
          return;
        }
        if (mapGoalUsesOfflineGrid()) {
          updateGoalPickStatus("当前为离线/缓存地图，与实时代价地图可能不一致，建议等待实时流后再发目标", "error");
          return;
        }
        if (state.pendingNavGoal && state.pendingNavGoal.status === "sending") {
          updateGoalPickStatus("上一条导航目标仍在下发，请稍候", "pending");
          return;
        }
        const frameId = navigationFrameId();
        const mapName = activeMapName();
        const robotPoint = pointFromPose(state.pose);
        const distanceM = planarDistanceMeters(mapPoint, robotPoint);
        let targetHeight;
        try {
          targetHeight = resolveNavigationHeight(els.mapGoalHeight, frameId);
        } catch (error) {
          updateGoalPickStatus(error.message, "error");
          return;
        }
        const body = {
          frame_id: frameId,
          x: Number(mapPoint.x),
          y: Number(mapPoint.y),
          z: targetHeight,
          yaw_rad: Number(yawRad),
        };
        if (mapName) body.map_name = mapName;
        state.pendingNavGoal = {
          x: body.x,
          y: body.y,
          z: body.z,
          yaw_rad: body.yaw_rad,
          frame_id: frameId,
          status: "sending",
        };
        const robotLabel = robotPoint ? formatMapPoint(robotPoint) : "--";
        const distanceLabel = distanceM === null ? "" : ` · 距离 ${distanceM.toFixed(2)}m`;
        updateGoalPickStatus(`下发 map 目标 ${formatMapPoint(body)} · 机器人 ${robotLabel}${distanceLabel} · 坐标系 ${frameId}`, "pending");
        requestMapDraw(false);
        const controller = new AbortController();
        const requestTimeout = window.setTimeout(() => controller.abort(), 5000);
        try {
          const response = await fetch("/api/console/navigation/goal", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
            signal: controller.signal,
          });
          const payload = await response.json().catch(() => ({}));
          if (!response.ok) {
            throw new Error(formatNavGoalError(payload, response.status));
          }
          if (!payload.published) {
            throw new Error(payload.message || "发布 /goal_pose 失败。");
          }
          const apiDistance = payload.distance_m;
          const distanceSuffix = Number.isFinite(Number(apiDistance))
            ? ` · 距离 ${Number(apiDistance).toFixed(2)}m`
            : distanceLabel;
          state.pendingNavGoal = {
            x: body.x,
            y: body.y,
            z: body.z,
            yaw_rad: body.yaw_rad,
            frame_id: frameId,
            status: "published",
            topic: payload.topic || "/goal_pose",
          };
          const topicLabel = payload.topic ? ` · ${payload.topic}` : "";
          const goalLabel = `${formatMapPoint(body)}${distanceSuffix} · ${frameId}`;
          updateGoalPickStatus(`已发布 RViz 目标 ${goalLabel}${topicLabel}`, "ok");
        } catch (error) {
          const message = error && error.name === "AbortError"
            ? "发布 /goal_pose 超时（5s），请确认网关与 go2.sh 已启动。"
            : (error && error.message ? error.message : String(error));
          state.pendingNavGoal = {
            x: body.x,
            y: body.y,
            z: body.z,
            yaw_rad: body.yaw_rad,
            frame_id: frameId,
            status: "error",
            message,
          };
          updateGoalPickStatus(`下发失败：${message}`, "error");
        } finally {
          window.clearTimeout(requestTimeout);
        }
        requestMapDraw(false);
      }
      async function dispatchClickedPoint(mapPoint) {
        if (!state.mapGrid) {
          updateGoalPickStatus("地图未显示，无法发布选点", "error");
          return;
        }
        if (!state.pose || !pointFromPose(state.pose)) {
          updateGoalPickStatus("定位位姿不可用，无法将点击坐标变换到 map 系", "error");
          return;
        }
        const frameId = navigationFrameId();
        const robotPoint = pointFromPose(state.pose);
        const distanceM = planarDistanceMeters(mapPoint, robotPoint);
        const body = {
          frame_id: frameId,
          x: Number(mapPoint.x),
          y: Number(mapPoint.y),
          z: 0,
        };
        const distanceLabel = distanceM === null ? "" : ` · 距机器人 ${distanceM.toFixed(2)}m`;
        updateGoalPickStatus(`发布 /clicked_point ${formatMapPoint(mapPoint)}${distanceLabel} · ${frameId}`, "pending");
        try {
          const response = await fetch("/api/console/map/clicked_point", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
          });
          const payload = await response.json().catch(() => ({}));
          if (!response.ok) {
            const message = String((payload && payload.message) || `HTTP ${response.status}`);
            throw new Error(message);
          }
          state.clickedPoints = [
            {
              x: body.x,
              y: body.y,
              z: body.z,
              frame_id: frameId,
              at: Date.now(),
            },
            ...(state.clickedPoints || []),
          ].slice(0, 30);
          const topic = payload.topic || "/clicked_point";
          updateGoalPickStatus(`已发布 ${topic} ${formatMapPoint(mapPoint)}${distanceLabel} · ${frameId}`, "ok");
        } catch (error) {
          const message = error && error.message ? error.message : String(error);
          updateGoalPickStatus(`选点发布失败：${message}`, "error");
        }
        requestMapDraw(false);
      }
      async function dispatchInitialPose(mapPoint, yawRad) {
        if (!state.mapGrid) {
          updateGoalPickStatus("地图未显示，无法发布初始位姿", "error");
          return;
        }
        const frameId = navigationFrameId();
        const robotPoint = pointFromPose(state.pose);
        const distanceM = planarDistanceMeters(mapPoint, robotPoint);
        const body = {
          frame_id: frameId,
          x: Number(mapPoint.x),
          y: Number(mapPoint.y),
          z: 0,
          yaw_rad: Number(yawRad),
        };
        const distanceLabel = distanceM === null ? "" : ` · 距当前位姿 ${distanceM.toFixed(2)}m`;
        updateGoalPickStatus(`发布 /initialpose ${formatMapPoint(mapPoint)} yaw=${(body.yaw_rad * 180 / Math.PI).toFixed(0)}°${distanceLabel} · ${frameId}`, "pending");
        try {
          const response = await fetch("/api/console/map/initial_pose", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
          });
          const payload = await response.json().catch(() => ({}));
          if (!response.ok) {
            throw new Error(formatNavGoalError(payload, response.status));
          }
          if (!payload.published) {
            throw new Error(payload.message || "发布 /initialpose 失败。");
          }
          state.initialPoses = [
            {
              x: body.x,
              y: body.y,
              z: body.z,
              yaw_rad: body.yaw_rad,
              frame_id: frameId,
              at: Date.now(),
            },
            ...(state.initialPoses || []),
          ].slice(0, 20);
          const topic = payload.topic || "/initialpose";
          updateGoalPickStatus(`已发布 RViz 初始位姿 ${formatMapPoint(mapPoint)} yaw=${(body.yaw_rad * 180 / Math.PI).toFixed(0)}°${distanceLabel} · ${topic} · ${frameId}`, "ok");
        } catch (error) {
          const message = error && error.message ? error.message : String(error);
          updateGoalPickStatus(`初始位姿发布失败：${message}`, "error");
        }
        requestMapDraw(false);
      }
      function drawOrientedGoal(ctx, anchor, scale, point, yawRad, colors, pixels) {
        if (!point || !Number.isFinite(Number(point.x)) || !Number.isFinite(Number(point.y))) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.translate(Number(point.x), Number(point.y));
        ctx.rotate(Number(yawRad) || 0);
        const shaft = Math.max(0.35, 14 / scale);
        const head = Math.max(0.18, 7 / scale);
        ctx.strokeStyle = colors.stroke;
        ctx.fillStyle = colors.fill;
        ctx.lineWidth = Math.max(1 / scale, pixels / scale);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(shaft, 0);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(shaft, 0);
        ctx.lineTo(shaft - head, head * 0.55);
        ctx.lineTo(shaft - head, -head * 0.55);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(0, 0, Math.max(0.08, 4 / scale), 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }
      function setWorldLineStyle(ctx, scale, color, pixels) {
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(1 / scale, pixels / scale);
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
      }
      function requestMapDraw(baseDirty) {
        if (baseDirty) state.mapBaseCache = null;
        if (state.mapViewMode === "3d") return;
        if (state.mapDrawFrame) return;
        state.mapDrawFrame = window.requestAnimationFrame(() => {
          state.mapDrawFrame = null;
          if (state.mapViewMode !== "3d") drawMap();
        });
      }
      function mapGridDataSignature(grid) {
        const data = grid && Array.isArray(grid.data) ? grid.data : [];
        if (!data.length) return "empty";
        let hash = 2166136261;
        for (let index = 0; index < data.length; index += 1) {
          hash ^= (Number(data[index]) + 129) & 255;
          hash = Math.imul(hash, 16777619) >>> 0;
        }
        return `${data.length}:${hash.toString(16)}`;
      }
      function mapGridContentKey(grid, envelope, diagnostics) {
        if (!grid) return "missing";
        const origin = originPosition(grid);
        const revision = (diagnostics && (diagnostics.occupancy_content_hash || diagnostics.content_hash || diagnostics.grid_hash || diagnostics.occupancy_content_revision || diagnostics.content_revision || diagnostics.revision)) || (grid.metadata && (grid.metadata.occupancy_content_hash || grid.metadata.content_hash || grid.metadata.grid_hash || grid.metadata.occupancy_content_revision || grid.metadata.content_revision || grid.metadata.revision)) || envelope.revision || grid.revision || mapGridDataSignature(grid);
        return [grid.frame_id || envelope.frame_id || "", grid.width, grid.height, gridResolution(grid), Number(origin.x || 0), Number(origin.y || 0), originYaw(grid), revision].join("|");
      }
      function normalizeStatusValue(value) {
        const raw = value && typeof value === "object" && "value" in value ? value.value : value;
        return String(raw || "").trim().toLowerCase().replace(/^(task|navigation|exploration)\./, "");
      }
      function isActiveSpatialStatus(value) {
        return ACTIVE_SPATIAL_STATUSES.has(normalizeStatusValue(value));
      }
      function isTerminalSpatialStatus(value) {
        return ["succeeded", "failed", "cancelled", "timeout", "idle"].includes(normalizeStatusValue(value));
      }
      function getNestedState(payload, key) {
        if (!payload) return {};
        if (payload[key] && typeof payload[key] === "object") return payload[key];
        return payload;
      }
      function localizationPoseStreamReason() {
        const envelope = state.lastLocalizationEnvelope;
        if (!envelope || statusOf(envelope) !== "available") return null;
        const data = envelope.data || {};
        return pointFromPose(data.pose || data) ? "localization_pose_stream" : null;
      }
      function goalInteractionReason() {
        if (state.goalPickMode || state.publishPointMode || state.initialPosePickMode || state.pendingNavGoal) {
          return "goal_pick";
        }
        return null;
      }
      function activeSpatialTaskReason() {
        for (const task of state.taskActiveById.values()) {
          const capability = String(task.capability_name || task.name || "").trim();
          const taskState = normalizeStatusValue(task.state || task.status);
          if (!capability || isTerminalSpatialStatus(taskState)) continue;
          if (ACTIVE_SPATIAL_CAPABILITIES.has(capability) || capability.startsWith("navigate_") || capability.includes("explore")) {
            return `task:${capability}`;
          }
        }
        return null;
      }
      function backendSpatialProcessReason() {
        const geoRuntime = state.latestGovernance && state.latestGovernance.metadata && state.latestGovernance.metadata.geometry_runtime;
        const processes = geoRuntime && geoRuntime.processes;
        const runtimeProcess = processes && (processes["nuwax-geometry-runtime"] || processes.nuwax_geometry_runtime);
        if (!runtimeProcess || runtimeProcess.running !== true) return null;
        const cmd = String(runtimeProcess.command_display || (Array.isArray(runtimeProcess.command) ? runtimeProcess.command.join(" ") : runtimeProcess.command) || "").toLowerCase();
        if (cmd.includes("--explore") || cmd.includes("--exploration")) return "exploration";
        if (cmd.includes("--localization") || cmd.includes("--loc")) return "backend_localization";
        if (cmd.includes("--mapping") || cmd.includes("--new")) return "mapping";
        return "backend_geometry";
      }
      function activeSpatialContextReason() {
        const taskReason = activeSpatialTaskReason();
        if (taskReason) return taskReason;
        const navPayload = state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload;
        const navState = getNestedState(navPayload, "navigation_state");
        if (isActiveSpatialStatus(navState.status) || (navState.current_goal_id && !isTerminalSpatialStatus(navState.status))) return "navigation";
        const explorationPayload = state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload;
        const explorationState = getNestedState(explorationPayload, "exploration_state");
        if (isActiveSpatialStatus(explorationState.status) || (explorationState.current_request_id && !isTerminalSpatialStatus(explorationState.status))) return "exploration";
        const backendReason = backendSpatialProcessReason();
        if (backendReason) return backendReason;
        const localizationReason = activeLocalizationSessionReason();
        if (localizationReason) return localizationReason;
        const poseStreamReason = localizationPoseStreamReason();
        if (poseStreamReason) return poseStreamReason;
        const goalReason = goalInteractionReason();
        if (goalReason) return goalReason;
        return null;
      }
      function activeLocalizationSessionReason() {
        const session = state.latestGovernance && state.latestGovernance.localization_session;
        if (!session || session.available !== true) return null;
        const status = normalizeStatusValue(session.status);
        if (!status || !ACTIVE_LOCALIZATION_STATUSES.has(status)) return null;
        return "localization";
      }
      function loadedMapAssetShouldDisplay(diagnostics) {
        if (!diagnostics) return false;
        const mapName = String(diagnostics.active_map_name || diagnostics.map_name || "").trim();
        if (!mapName) return false;
        const manifestSha256 = String(diagnostics.asset_manifest_sha256 || "").trim();
        const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
        const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
        return Boolean(
          manifestSha256 ||
          snapshotKind === "offline_checkpoint" ||
          sourceKind === "map_asset_bundle" ||
          diagnostics.checkpoint_ready === true ||
          diagnostics.geometry_ready === true
        );
      }
      function firstFiniteNumber(...values) {
        for (const value of values) {
          if (value === undefined || value === null || value === "") continue;
          const number = Number(value);
          if (Number.isFinite(number)) return number;
        }
        return null;
      }
      function timestampIsFresh(...values) {
        const now = Date.now();
        for (const value of values) {
          if (!value) continue;
          const parsed = Date.parse(value);
          if (!Number.isFinite(parsed)) continue;
          const age = now - parsed;
          if (age >= -5000 && age <= MAP_DISPLAY_FRESH_MS) return true;
        }
        return false;
      }
      function diagnosticsAgeIsFresh(envelope, diagnostics) {
        const streamAgeMs = firstFiniteNumber(
          diagnostics && diagnostics.stream_age_ms,
          diagnostics && diagnostics.cache_age !== undefined ? Number(diagnostics.cache_age) * 1000 : null,
          diagnostics && diagnostics.source_age !== undefined ? Number(diagnostics.source_age) * 1000 : null
        );
        if (streamAgeMs !== null && streamAgeMs <= MAP_DISPLAY_FRESH_MS) return true;
        return timestampIsFresh(
          envelope && envelope.source_timestamp,
          diagnostics && diagnostics.source_timestamp,
          diagnostics && diagnostics.last_message_at,
          diagnostics && diagnostics.latest_occupancy_received_at,
          diagnostics && diagnostics.latest_occupancy_timestamp
        );
      }
      function mapSnapshotIsOffline(diagnostics) {
        const sourceKind = String(diagnostics && diagnostics.source_kind || "").toLowerCase();
        const snapshotKind = String(diagnostics && diagnostics.map_snapshot_kind || "").toLowerCase();
        return sourceKind === "map_asset_bundle" || snapshotKind === "offline_checkpoint";
      }
      function shouldPinLocalizationCheckpointDisplay(diagnostics) {
        if (!diagnostics) return false;
        if (diagnostics.pin_checkpoint_display === true) return true;
        if (String(diagnostics.suppressed_live_map_reason || "") === "localization_mode_pin_checkpoint_display") {
          return true;
        }
        const backendMode = String(
          diagnostics.backend_mode
          || (diagnostics.geometry_activation && diagnostics.geometry_activation.backend_mode)
          || ""
        ).trim().toLowerCase();
        return backendMode === "mapping_localization" && loadedMapAssetShouldDisplay(diagnostics);
      }
      function hasLiveMapFlag(diagnostics, streamBridge) {
        if (!diagnostics) return false;
        if (mapSnapshotIsOffline(diagnostics)) return false;
        const sourceKind = String(diagnostics.source_kind || "").toLowerCase();
        const snapshotKind = String(diagnostics.map_snapshot_kind || "").toLowerCase();
        const bridgeMode = String((streamBridge && (streamBridge.status || streamBridge.state || streamBridge.mode)) || "").toLowerCase();
        return Boolean(
          sourceKind === "platform_live_map_provider" ||
          snapshotKind === "live_streaming" ||
          diagnostics.source_alive === true ||
          diagnostics.streaming === true ||
          diagnostics.live_mapping_ready === true ||
          diagnostics.mapping_live_streaming_ready === true ||
          bridgeMode === "streaming" ||
          (streamBridge && streamBridge.streaming === true)
        );
      }
      function hasPositiveMapCount(diagnostics) {
        const count = firstFiniteNumber(
          diagnostics && diagnostics.message_count,
          diagnostics && diagnostics.occupancy_message_count,
          diagnostics && diagnostics.occupancy_sequence,
          diagnostics && diagnostics.update_sequence,
          diagnostics && diagnostics.sequence
        );
        return count === null || count > 0;
      }
      function shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared) {
        if (goalInteractionReason()) {
          if (grid && Array.isArray(grid.data) && grid.width && grid.height) {
            return { display: true, reason: "goal_pick" };
          }
        }
        if (cleared) return { display: false, status: "cleared", message: mapClearedMessage(diagnostics) };
        if (!grid || !Array.isArray(grid.data) || !grid.width || !grid.height) {
          return { display: false, status: status === "error" ? "error" : "missing", message: errorReason(envelope, "无 occupancy_grid，占据栅格不可用。") };
        }
        const activeReason = activeSpatialContextReason();
        if (activeReason) return { display: true, reason: activeReason };
        if (status === "available" && loadedMapAssetShouldDisplay(diagnostics)) {
          return { display: true, reason: "loaded_map_asset" };
        }
        if (status !== "available") {
          return { display: false, status: status || "stale", message: status === "stale" ? "地图数据已过期，已隐藏缓存帧。" : errorReason(envelope, MAP_DISPLAY_IDLE_MESSAGE) };
        }
        if (hasLiveMapFlag(diagnostics, streamBridge) && hasPositiveMapCount(diagnostics) && diagnosticsAgeIsFresh(envelope, diagnostics || {})) {
          return { display: true, reason: "live_map_stream" };
        }
        return { display: false, status: "missing", message: MAP_DISPLAY_IDLE_MESSAGE };
      }
      function computeMapDisplayDecision(envelope) {
        if (!envelope) return { display: false, status: "missing", message: MAP_DISPLAY_IDLE_MESSAGE };
        const data = envelope.data || {};
        const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
        const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
        const streamBridge = diagnostics.stream_bridge || {};
        const status = statusOf(envelope);
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        return shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
      }
      function shouldDisplaySpatialPose() {
        if (activeSpatialContextReason()) return true;
        return computeMapDisplayDecision(state.lastMapEnvelope).display === true;
      }
      function clearLocalizationDisplay(message, status) {
        state.pose = null;
        state.trajectory = [];
        updateMapPoseHud();
        setPill(els.localizationStatus, status || "missing");
        setKv(els.localizationMeta, [
          ["显示状态", "已隐藏"],
          ["原因", message || LOCALIZATION_DISPLAY_IDLE_MESSAGE],
          ["坐标系", "--"],
          ["位姿", "--"],
          ["偏航角", "--"],
          ["轨迹点数", 0],
        ]);
      }
      function clearMapOverlayState() {
        state.frontiers = [];
        state.latestFrontierStream = null;
        state.goalPose = null;
        state.navigationPath = [];
      }
      function resolveGeometrySessionMode(data) {
        const direct = data && data.geometry_session_mode;
        if (direct) return String(direct).trim().toLowerCase();
        const explorationContext = data && data.exploration_context;
        const contextMeta = explorationContext && explorationContext.metadata;
        if (contextMeta && contextMeta.geometry_session_mode) {
          return String(contextMeta.geometry_session_mode).trim().toLowerCase();
        }
        const explorationState = explorationContext && explorationContext.exploration_state;
        const stateMeta = explorationState && explorationState.metadata;
        if (stateMeta && stateMeta.geometry_session_mode) {
          return String(stateMeta.geometry_session_mode).trim().toLowerCase();
        }
        return "";
      }
      function shouldRetainFrontierStreamCache(fr, data) {
        if (!fr) return false;
        if (fr.frontier_cleared === true) return false;
        const sessionMode = resolveGeometrySessionMode(data)
          || String(fr.geometry_session_mode || "").trim().toLowerCase();
        if (sessionMode && sessionMode !== "explore") return false;
        if (fr.exploration_terminal === true) return false;
        return true;
      }
      function clearMapDisplay(message, status, diagnostics, streamBridge, envelope, grid) {
        if (state.goalPickMode || state.publishPointMode || state.initialPosePickMode || state.pendingNavGoal) {
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = message || MAP_DISPLAY_IDLE_MESSAGE;
          return;
        }
        state.mapGrid = null;
        state.appliedMapIdentity = null;
        state.mapGridContentKey = "";
        state.mapBaseCache = null;
        state.mapViewport = null;
        state.mapFreeViewIdentity = "";
        state.mapFreeView = { center: null, zoom: 1, fitSpan: null };
        if (!activeSpatialContextReason()) clearMapOverlayState();
        setPill(els.mapStatus, status || "missing");
        els.mapEmpty.hidden = false;
        els.mapEmpty.textContent = message || MAP_DISPLAY_IDLE_MESSAGE;
        requestMapDraw(true);
        const displayDiagnostics = { ...(diagnostics || {}), display_suppressed_reason: message || MAP_DISPLAY_IDLE_MESSAGE };
        renderMapSummary({
          grid,
          diagnostics: displayDiagnostics,
          envelope,
          displayStatus: "已隐藏",
          message: message || MAP_DISPLAY_IDLE_MESSAGE,
        });
        updateMapStreamChip(status || "missing", displayDiagnostics, streamBridge || {}, envelope && envelope.source_timestamp);
        reevaluateCachedLocalizationDisplay();
      }
      function ensureMapBaseCanvas(ctx, grid) {
        const width = Number(grid.width);
        const height = Number(grid.height);
        if (state.mapBaseCache && state.mapBaseCache.key === state.mapGridContentKey && state.mapBaseCache.width === width && state.mapBaseCache.height === height) return state.mapBaseCache.canvas;
        const image = ctx.createImageData(width, height);
        for (let row = 0; row < height; row += 1) {
          for (let col = 0; col < width; col += 1) {
            const sourceIndex = row * width + col;
            const targetIndex = ((height - 1 - row) * width + col) * 4;
            const value = Number(grid.data[sourceIndex]);
            let color = CANVAS_THEME.mapUnknown;
            if (value >= 65) color = CANVAS_THEME.mapOccupied;
            else if (value >= 0) color = CANVAS_THEME.mapFree;
            image.data[targetIndex] = color[0];
            image.data[targetIndex + 1] = color[1];
            image.data[targetIndex + 2] = color[2];
            image.data[targetIndex + 3] = 255;
          }
        }
        const offscreen = document.createElement("canvas");
        offscreen.width = width;
        offscreen.height = height;
        offscreen.getContext("2d").putImageData(image, 0, 0);
        state.mapBaseCache = { key: state.mapGridContentKey, width, height, canvas: offscreen };
        return offscreen;
      }
      function drawMap() {
        syncMapCanvasSize();
        syncMap2dCameraControls();
        const canvas = els.mapCanvas;
        const ctx = canvas.getContext("2d");
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = CANVAS_THEME.mapBackground;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        const grid = state.mapGrid;
        const anchor = cameraAnchor();
        const scale = cameraScale(anchor);
        state.mapViewport = { centerX: canvas.width / 2, centerY: canvas.height / 2, scale, zoom: mapViewZoom(), robotLocked: anchor.locked };
        if (els.mapZoomValue) els.mapZoomValue.textContent = `${Math.round(mapViewZoom() * 100)}%`;
        if (!grid || !Array.isArray(grid.data) || !grid.width || !grid.height) return;
        const width = Number(grid.width);
        const height = Number(grid.height);
        const offscreen = ensureMapBaseCanvas(ctx, grid);
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        const origin = originPosition(grid);
        const resolution = gridResolution(grid);
        ctx.translate(Number(origin.x || 0), Number(origin.y || 0));
        ctx.rotate(originYaw(grid));
        ctx.scale(resolution, -resolution);
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(offscreen, 0, -height, width, height);
        ctx.restore();
        drawTrajectory(ctx, anchor, scale);
        drawNavigationPath(ctx, anchor, scale);
        drawGoal(ctx, anchor, scale);
        drawGoalPickPreview(ctx, anchor, scale);
        drawInitialPosePickPreview(ctx, anchor, scale);
        drawInitialPoses(ctx, anchor, scale);
        drawClickedPoints(ctx, anchor, scale);
        drawFrontiers(ctx, anchor, scale);
        drawSemanticInstances(ctx, anchor, scale);
        drawRobot(ctx, anchor);
        // V2 boundary draw tick：每帧画完后通知 SVG overlay 同步 viewBox，
        // 这样线段跟着机器人 pose / 缩放 / pan 移动。
        if (typeof window.__nuwaxMapDrawTick === 'function') {
          try { window.__nuwaxMapDrawTick(); } catch (_) { /* boundary 模块出错不影响主循环 */ }
        }
      }
      function pointFromFrontier(frontier) {
        const geometry = frontier && frontier.geometry ? frontier.geometry : frontier;
        if (!geometry) return null;
        if (Array.isArray(geometry.coordinates) && geometry.coordinates.length >= 2) return { x: Number(geometry.coordinates[0]), y: Number(geometry.coordinates[1]), z: Number(geometry.coordinates[2] || 0) };
        if (geometry.position) {
          const point = pointFromPose(geometry);
          if (point) return point;
        }
        if (geometry.pose) {
          const point = pointFromPose(geometry.pose);
          if (point) return point;
          return pointFromFrontier(geometry.pose);
        }
        if (geometry.centroid) return pointFromFrontier(geometry.centroid);
        if (geometry.x !== undefined && geometry.y !== undefined) return { x: Number(geometry.x), y: Number(geometry.y), z: Number(geometry.z || 0) };
        return null;
      }
      function drawWorldPoints(ctx, anchor, scale, points, color, pixels) {
        if (!points || points.length < 2) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        setWorldLineStyle(ctx, scale, color, pixels);
        ctx.beginPath();
        let started = false;
        for (const point of points) {
          if (!point) continue;
          if (!started) { ctx.moveTo(point.x, point.y); started = true; }
          else ctx.lineTo(point.x, point.y);
        }
        if (started) ctx.stroke();
        ctx.restore();
      }
      function drawFrontiers(ctx, anchor, scale) {
        if (!state.frontiers || !state.frontiers.length) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.fillStyle = CANVAS_THEME.frontier;
        ctx.strokeStyle = CANVAS_THEME.frontierStroke;
        ctx.lineWidth = Math.max(1 / scale, 1.4 / scale);
        for (const frontier of state.frontiers) {
          const point = pointFromFrontier(frontier);
          if (!point) continue;
          ctx.beginPath();
          ctx.arc(point.x, point.y, Math.max(0.08, 5 / scale), 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }
        ctx.restore();
      }
      function drawTrajectory(ctx, anchor, scale) {
        const points = (state.trajectory || []).map(pointFromPose).filter(Boolean);
        drawWorldPoints(ctx, anchor, scale, points, CANVAS_THEME.trajectory, 2.5);
      }
      function drawNavigationPath(ctx, anchor, scale) {
        drawWorldPoints(ctx, anchor, scale, state.navigationPath, CANVAS_THEME.navPath, 3);
      }
      function drawGoal(ctx, anchor, scale) {
        const pending = state.pendingNavGoal;
        if (pending && pending.status !== "error") {
          drawOrientedGoal(
            ctx,
            anchor,
            scale,
            { x: pending.x, y: pending.y },
            pending.yaw_rad,
            { fill: CANVAS_THEME.goalPick, stroke: CANVAS_THEME.goalPickStroke },
            2.8,
          );
        }
        const point = pointFromPose(state.goalPose);
        if (!point) return;
        const yaw = yawFromPose(state.goalPose);
        if (Number.isFinite(yaw)) {
          drawOrientedGoal(
            ctx,
            anchor,
            scale,
            point,
            yaw,
            { fill: CANVAS_THEME.goal, stroke: CANVAS_THEME.goalStroke },
            2.4,
          );
          return;
        }
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.strokeStyle = CANVAS_THEME.goalStroke;
        ctx.fillStyle = CANVAS_THEME.goal;
        ctx.lineWidth = Math.max(1 / scale, 2 / scale);
        const radius = Math.max(0.12, 7 / scale);
        ctx.beginPath();
        ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
      function drawClickedPoints(ctx, anchor, scale) {
        const points = state.clickedPoints || [];
        if (!points.length) return;
        ctx.save();
        applyCameraTransform(ctx, anchor, scale);
        ctx.fillStyle = CANVAS_THEME.clickedPoint;
        ctx.strokeStyle = CANVAS_THEME.clickedPointStroke;
        for (const point of points) {
          if (!point || !Number.isFinite(Number(point.x)) || !Number.isFinite(Number(point.y))) continue;
          const radius = Math.max(0.1, 6 / scale);
          ctx.lineWidth = Math.max(1 / scale, 2 / scale);
          ctx.beginPath();
          ctx.arc(Number(point.x), Number(point.y), radius, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }
        ctx.restore();
      }
      function drawGoalPickPreview(ctx, anchor, scale) {
        const drag = state.goalPickDrag;
        if (!drag || !drag.start) return;
        const previewAnchor = drag.anchor || anchor;
        const previewScale = drag.anchor ? cameraScale(previewAnchor) : scale;
        const yaw = goalPickYawFromDrag(drag);
        drawOrientedGoal(
          ctx,
          previewAnchor,
          previewScale,
          drag.start,
          yaw,
          { fill: CANVAS_THEME.goalPick, stroke: CANVAS_THEME.goalPickStroke },
          3,
        );
      }
      function drawInitialPosePickPreview(ctx, anchor, scale) {
        const drag = state.initialPosePickDrag;
        if (!drag || !drag.start) return;
        const previewAnchor = drag.anchor || anchor;
        const previewScale = drag.anchor ? cameraScale(previewAnchor) : scale;
        const yaw = goalPickYawFromDrag(drag);
        drawOrientedGoal(
          ctx,
          previewAnchor,
          previewScale,
          drag.start,
          yaw,
          { fill: CANVAS_THEME.initialPosePick, stroke: CANVAS_THEME.initialPosePickStroke },
          3,
        );
      }
      function drawInitialPoses(ctx, anchor, scale) {
        const poses = state.initialPoses || [];
        if (!poses.length) return;
        for (const pose of poses) {
          if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) continue;
          drawOrientedGoal(
            ctx,
            anchor,
            scale,
            { x: pose.x, y: pose.y },
            Number(pose.yaw_rad) || 0,
            { fill: CANVAS_THEME.initialPosePick, stroke: CANVAS_THEME.initialPosePickStroke },
            2.4,
          );
        }
      }
      function drawRobot(ctx, anchor) {
        const posePoint = pointFromPose(state.pose);
        if (!posePoint) return;
        const locked = anchor && anchor.locked === true;
        ctx.save();
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        if (locked) {
          ctx.translate(els.mapCanvas.width / 2, els.mapCanvas.height / 2);
        } else {
          const canvasPoint = mapToCanvas(posePoint);
          if (!canvasPoint) {
            ctx.restore();
            return;
          }
          ctx.translate(canvasPoint.x, canvasPoint.y);
          const yaw = planarYawFromPose(state.pose) || 0;
          // 北向 map 视图：屏幕 Y 向上对应 world +Y，三角形默认指屏幕上方，需用 π/2 - yaw（不是 yaw + π/2）
          ctx.rotate(Math.PI / 2 - yaw);
        }
        ctx.fillStyle = CANVAS_THEME.robot;
        ctx.strokeStyle = CANVAS_THEME.robotStroke;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, -14);
        ctx.lineTo(-9, 9);
        ctx.lineTo(0, 5);
        ctx.lineTo(9, 9);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
      // 空间记忆只读刷新：合并并发请求，避免相同数据反复重建三维标记。
      function loadSemanticInstances() {
        if (state.semanticLoadPromise) return state.semanticLoadPromise;
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), 8000);
        const requestedMap = activeMapName();
        if (els.semanticInstancesRefresh) {
          els.semanticInstancesRefresh.disabled = true;
          els.semanticInstancesRefresh.setAttribute("aria-busy", "true");
          els.semanticInstancesRefresh.textContent = "刷新中…";
        }
        state.semanticLoadPromise = fetch("/api/memory/instances?limit=500", { signal: controller.signal })
          .then(async (response) => {
            if (!response.ok) throw new Error(`服务返回 ${response.status}`);
            const body = await response.json();
            const src = Array.isArray(body?.instances) ? body : body?.result;
            if (!Array.isArray(src?.instances)) throw new Error("记忆接口返回格式异常");
            if (requestedMap && requestedMap !== activeMapName()) throw new Error("地图已切换，正在刷新当前记忆库");
            const seen = new Set();
            const list = src.instances.filter((inst) => {
              if (!inst || typeof inst !== "object" || !inst.instance_id || seen.has(inst.instance_id)) return false;
              seen.add(inst.instance_id);
              return true;
            });
            const snapshotKey = JSON.stringify([src.library_name, src.map_name, list]);
            const wasError = Boolean(state.semanticError);
            state.semanticError = "";
            state.semanticFetchedAt = Date.now();
            const previousLibrary = state.semanticMeta.library_name;
            state.semanticMeta = src;
            if (snapshotKey !== state.semanticSnapshotKey || wasError) {
              state.semanticSnapshotKey = snapshotKey;
              state.semanticInstances = list;
              if (!list.some((inst) => inst.instance_id === state.semanticSelectedId)) state.semanticSelectedId = null;
              reconcileSemanticOverlay(previousLibrary);
              publishSemanticMapInstances();
              requestMapDraw(false);
              renderSemanticMemoryList();
            }
          })
          .catch((error) => {
            state.semanticError = error.name === "AbortError" ? "读取超时" : String(error.message || error);
            reconcileSemanticOverlay(state.semanticMeta.library_name);
            // 失败时保留可查阅记录，但撤下地图标记，避免缓存被当作当前位置。
            publishSemanticMapInstances();
            requestMapDraw(false);
          })
          .finally(() => {
            window.clearTimeout(timeout);
            state.semanticLoadPromise = null;
            if (els.semanticInstancesRefresh) {
              els.semanticInstancesRefresh.disabled = false;
              els.semanticInstancesRefresh.removeAttribute("aria-busy");
              els.semanticInstancesRefresh.textContent = state.semanticError ? "重试刷新" : "刷新物品";
            }
            renderSemanticMemoryStatus();
            if (requestedMap && requestedMap !== activeMapName() && !document.hidden) window.setTimeout(loadSemanticInstances, 0);
          });
        renderSemanticMemoryStatus();
        return state.semanticLoadPromise;
      }
      function semanticObservationOnly(inst) {
        return inst.observation_only === true || inst.projection_method === "current_pose_fallback";
      }
      function semanticQuality(inst) {
        if (semanticObservationOnly(inst)) return { key: "observation", label: "仅观察" };
        if (inst.lifecycle_state === "stale") return { key: "uncertain", label: "已过时" };
        if (inst.lifecycle_state === "removed") return { key: "uncertain", label: "已移除" };
        if (inst.lifecycle_state === "uncertain") return { key: "uncertain", label: "待确认" };
        if (isFallbackInstance(inst) || isUnknownFrame(inst) || !semanticFinitePose(inst.pose)) return { key: "uncertain", label: "待确认" };
        return { key: "located", label: "有位置" };
      }
      function semanticFinitePose(pose) {
        return Boolean(pose && [pose.x, pose.y, pose.z].every((v) => typeof v === "number" && Number.isFinite(v)));
      }
      function semanticScopeMatches(inst) {
        const currentMap = activeMapName();
        const mapName = inst.map_name;
        return Boolean(currentMap && mapName && currentMap === mapName);
      }
      function semanticMapInstances() {
        if (state.semanticError) return [];
        const mapName = activeMapName();
        const cache = state.semanticMapCache;
        if (cache?.snapshot === state.semanticSnapshotKey && cache.mapName === mapName) return cache.instances;
        const instances = state.semanticInstances.filter((inst) => semanticScopeMatches(inst) && semanticQuality(inst).key === "located");
        state.semanticMapCache = { snapshot: state.semanticSnapshotKey, mapName, instances };
        return instances;
      }
      function publishSemanticMapInstances() {
        const mapName = activeMapName();
        const previous = state.semanticPublishedKey;
        if (previous?.snapshot === state.semanticSnapshotKey && previous.mapName === mapName && previous.error === state.semanticError) return;
        state.semanticPublishedKey = { snapshot: state.semanticSnapshotKey, mapName, error: state.semanticError };
        window.dispatchEvent(new CustomEvent("pointcloud3d:semantic-instances", {
          detail: { instances: semanticMapInstances() },
        }));
      }
      function renderSemanticMemoryStatus() {
        const meta = state.semanticMeta;
        const count = state.semanticInstances.length;
        const library = meta.library_name || meta.active_library_name;
        const suffix = state.semanticFetchedAt ? ` · 更新于 ${new Date(state.semanticFetchedAt).toLocaleTimeString()}` : "";
        const mismatch = count > 0 && !state.semanticInstances.some(semanticScopeMatches);
        const status = state.semanticError
          ? `刷新失败：${state.semanticError}。${count ? "正在查看上次缓存，地图标记已隐藏。" : "请重试。"}`
          : state.semanticLoadPromise ? "正在读取当前记忆库…"
          : !library && !count ? "未启用记忆库，请先激活地图。"
          : `${library ? `记忆库：${library}` : "当前记忆库"}${suffix}${mismatch ? " · 与当前地图不匹配，标记已隐藏" : ""}`;
        if (els.semanticMemoryStatus) {
          els.semanticMemoryStatus.textContent = status;
          els.semanticMemoryStatus.dataset.state = state.semanticError ? "error" : mismatch ? "stale" : "ready";
        }
        setText(els.semanticMemoryCount, state.semanticFetchedAt ? String(count) : "—");
        const located = state.semanticInstances.filter((inst) => semanticQuality(inst).key === "located").length;
        const observations = state.semanticInstances.filter(semanticObservationOnly).length;
        setText(els.semanticMemorySummary, `${count} 条记录 · ${located} 个有位置 · ${observations} 条仅观察${meta.truncated ? ` · 已载入 ${count}/${meta.total} 条` : ""}`);
        if (els.semanticInstancesRefresh) els.semanticInstancesRefresh.title = status;
      }
      function semanticListRow(inst) {
        const row = document.createElement("button");
        row.type = "button";
        row.className = "semantic-memory-row";
        row.dataset.instanceId = inst.instance_id;
        row.setAttribute("aria-pressed", String(state.semanticSelectedId === inst.instance_id));
        const name = document.createElement("strong");
        name.textContent = semanticInstanceName(inst);
        const badge = document.createElement("span");
        badge.className = "semantic-memory-badge";
        const quality = semanticQuality(inst);
        badge.dataset.quality = quality.key;
        badge.textContent = quality.label;
        const evidence = document.createElement("small");
        const observations = Number.isFinite(inst.observation_count) ? `${inst.observation_count} 次观察 · ` : "";
        evidence.textContent = `${inst.label || "未分类"} · ${observations}最后见于 ${formatLastSeen(inst.last_seen_ts)}`;
        row.append(name, badge, evidence);
        row.addEventListener("click", () => showSemanticInstanceDetail(inst));
        return row;
      }
      function renderSemanticMemoryList() {
        if (!els.semanticMemoryList) return;
        const query = String(els.semanticMemorySearch?.value || "").trim().toLocaleLowerCase();
        const filter = els.semanticMemoryFilter?.value || "all";
        const list = state.semanticInstances.filter((inst) => {
          const text = `${semanticInstanceName(inst)} ${inst.label || ""} ${inst.instance_id}`.toLocaleLowerCase();
          return (!query || text.includes(query)) && (filter === "all" || semanticQuality(inst).key === filter);
        });
        const fragment = document.createDocumentFragment();
        if (!list.length) {
          const empty = document.createElement("p");
          empty.className = "semantic-memory-empty";
          empty.textContent = state.semanticInstances.length ? "没有匹配的记忆，试试其它名称或证据类型。" : "当前记忆库暂无物品。完成场景观察并写入记忆后，可在这里查看。";
          fragment.append(empty);
        }
        for (const inst of list) fragment.append(semanticListRow(inst));
        els.semanticMemoryList.replaceChildren(fragment);
      }
      function toggleSemanticMemoryPanel(open) {
        if (!els.semanticMemoryPanel) return;
        els.semanticMemoryPanel.hidden = !open;
        els.semanticMemoryToggle?.setAttribute("aria-expanded", String(open));
        if (open) {
          renderSemanticMemoryList();
          renderSemanticMemoryStatus();
          if (!state.semanticFetchedAt || Date.now() - state.semanticFetchedAt >= 30000) loadSemanticInstances();
          els.semanticMemorySearch?.focus();
        }
      }
      function syncPointcloud3dState() {
        const navigationFrame = state.latestStreamFrames["navigation.state"];
        const navigationContext = navigationFrame && navigationFrame.payload;
        const navigationExplorationValidated = navigationGoalMatchesExploration(
          navigationContext,
          state.latestExplorationOverlayIdentity
        );
        window.dispatchEvent(new CustomEvent("stream:localization.pose", { detail: state.latestPose || null }));
        window.dispatchEvent(new CustomEvent("stream:localization.trajectory", {
          detail: { trajectory: state.trajectory || [] },
        }));
        window.dispatchEvent(new CustomEvent("stream:exploration.frontier", {
          detail: {
            frontiers: state.frontiers || [],
            planned_path: state.navigationPath || [],
            current_goal: state.goalPose || null,
            navigation_exploration_validated: navigationExplorationValidated,
            ...currentExplorationOverlayDetail(),
          },
        }));
        window.dispatchEvent(new CustomEvent("stream:exploration.state", {
          detail: {
            planned_path: state.navigationPath || [],
            current_goal: state.goalPose || null,
            navigation_exploration_validated: navigationExplorationValidated,
            ...currentExplorationOverlayDetail(),
          },
        }));
        window.dispatchEvent(new CustomEvent("stream:navigation.state", {
          detail: {
            planned_path: state.navigationPath || [],
            current_goal: state.goalPose || null,
            navigation_exploration_validated: navigationExplorationValidated,
            ...currentExplorationOverlayDetail(),
          },
        }));
        window.dispatchEvent(new CustomEvent("pointcloud3d:semantic-instances", {
          detail: { instances: semanticMapInstances() },
        }));
        window.dispatchEvent(new CustomEvent("pointcloud3d:semantic-visibility", {
          detail: { visible: state.showSemanticInstances },
        }));
        const pointcloudMeta = state.latestStreamFrames["pointcloud_map.meta"];
        if (pointcloudMeta && pointcloudMeta.payload) {
          window.dispatchEvent(new CustomEvent("stream:pointcloud_map.meta", {
            detail: pointcloudMeta.payload,
          }));
        }
      }
      function isFallbackInstance(inst) {
        return inst.is_fallback === true || semanticObservationOnly(inst)
          || (!inst.projection_method && inst.navigable !== true);
      }
      function isUnknownFrame(inst) {
        const fid = String(inst.frame_id || "").trim();
        return fid !== "map";
      }
      const CLUSTER_PX_THRESHOLD = 28;
      const SEMANTIC_MARKER_HEIGHT = 24;
      const SEMANTIC_MARKER_MIN_WIDTH = 40;
      const SEMANTIC_MARKER_MAX_WIDTH = 168;
      const SEMANTIC_MARKER_PADDING_X = 10;
      function semanticInstanceName(inst) {
        return String(inst && (inst.display_name || inst.label || inst.instance_id) || "?").trim() || "?";
      }
      function fitSemanticMarkerLabel(ctx, label, maxWidth) {
        if (ctx.measureText(label).width <= maxWidth) return label;
        const characters = Array.from(label);
        let fitted = "";
        for (const character of characters) {
          if (ctx.measureText(`${fitted}${character}…`).width > maxWidth) break;
          fitted += character;
        }
        return fitted ? `${fitted}…` : "…";
      }
      function semanticMarkerMetrics(ctx, inst, highlighted) {
        const height = highlighted ? SEMANTIC_MARKER_HEIGHT + 2 : SEMANTIC_MARKER_HEIGHT;
        const font = highlighted ? "700 11px sans-serif" : "600 11px sans-serif";
        ctx.font = font;
        const maxTextWidth = SEMANTIC_MARKER_MAX_WIDTH - SEMANTIC_MARKER_PADDING_X * 2;
        const label = fitSemanticMarkerLabel(ctx, semanticInstanceName(inst), maxTextWidth);
        const width = Math.max(
          SEMANTIC_MARKER_MIN_WIDTH,
          Math.min(SEMANTIC_MARKER_MAX_WIDTH, Math.ceil(ctx.measureText(label).width) + SEMANTIC_MARKER_PADDING_X * 2),
        );
        return { width, height, radius: 4, label, font };
      }
      function semanticClusterMarkerMetrics(ctx, count, highlighted) {
        return semanticMarkerMetrics(ctx, { display_name: `${count} 个物品` }, highlighted);
      }
      function drawCanvasRoundedRectPath(ctx, x, y, width, height, radius) {
        const r = Math.min(radius, width / 2, height / 2);
        ctx.beginPath();
        ctx.moveTo(x + r, y);
        ctx.lineTo(x + width - r, y);
        ctx.quadraticCurveTo(x + width, y, x + width, y + r);
        ctx.lineTo(x + width, y + height - r);
        ctx.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
        ctx.lineTo(x + r, y + height);
        ctx.quadraticCurveTo(x, y + height, x, y + height - r);
        ctx.lineTo(x, y + r);
        ctx.quadraticCurveTo(x, y, x + r, y);
        ctx.closePath();
      }
      function clusterInstances(instances, anchor, scale) {
        // 视口裁剪：只处理可见区域 + margin
        const canvas = els.mapCanvas;
        const margin = CLUSTER_PX_THRESHOLD * 2;
        const visible = [];
        for (const inst of instances) {
          if (!inst.pose) continue;
          const cp = mapToCanvas({ x: inst.pose.x, y: inst.pose.y }, anchor, scale);
          if (!cp) continue;
          if (cp.x < -margin || cp.x > canvas.width + margin) continue;
          if (cp.y < -margin || cp.y > canvas.height + margin) continue;
          visible.push({ inst, cx: cp.x, cy: cp.y });
        }
        if (visible.length > 1000) visible.splice(1000);
        // O(n) 网格聚类
        const cellSize = CLUSTER_PX_THRESHOLD;
        const grid = new Map();
        for (const item of visible) {
          const gx = Math.floor(item.cx / cellSize);
          const gy = Math.floor(item.cy / cellSize);
          let merged = false;
          for (let dx = -1; dx <= 1 && !merged; dx++) {
            for (let dy = -1; dy <= 1 && !merged; dy++) {
              const key = `${gx + dx},${gy + dy}`;
              const cells = grid.get(key) || [];
              for (const cell of cells) {
                const lx = cell.sumX / cell.members.length;
                const ly = cell.sumY / cell.members.length;
                const dist = Math.sqrt((item.cx - lx) ** 2 + (item.cy - ly) ** 2);
                if (dist < CLUSTER_PX_THRESHOLD) {
                  cell.members.push(item.inst);
                  cell.sumX += item.cx;
                  cell.sumY += item.cy;
                  merged = true;
                  break;
                }
              }
            }
          }
          if (!merged) {
            const key = `${gx},${gy}`;
            if (!grid.has(key)) grid.set(key, []);
            grid.get(key).push({ members: [item.inst], sumX: item.cx, sumY: item.cy });
          }
        }
        const clusters = [];
        for (const cells of grid.values()) for (const cell of cells) {
          const cx = cell.sumX / cell.members.length;
          const cy = cell.sumY / cell.members.length;
          const ids = cell.members.map((m) => m.instance_id).sort().join("|");
          clusters.push({ key: ids, members: cell.members, cx, cy });
        }
        return clusters;
      }
      function resolveHighlightCluster(clusters) {
        const ht = state.highlightTarget;
        if (!ht) return null;
        // record_id 精确优先
        if (ht.record_id) {
          for (const cl of clusters) {
            for (const m of cl.members) {
              if (m.instance_id === ht.record_id) return cl.key;
            }
          }
        }
        // 坐标就近兜底
        if (ht.target_pose) {
          const tp = ht.target_pose;
          let bestDist = 80;
          let bestKey = null;
          for (const cl of clusters) {
            for (const m of cl.members) {
              if (!m.pose) continue;
              const d = Math.sqrt((m.pose.x - (tp.x || 0)) ** 2 + (m.pose.y - (tp.y || 0)) ** 2);
              if (d < bestDist) { bestDist = d; bestKey = cl.key; }
            }
          }
          if (bestKey) return bestKey;
        }
        return null;
      }
      function drawSemanticInstances(ctx, anchor, scale) {
        if (!state.showSemanticInstances) return;
        const instances = semanticMapInstances();
        if (!instances || !instances.length) {
          // 若有 target_pose 但无实例，画临时高亮环
          if (state.highlightTarget && state.highlightTarget.target_pose) {
            const tp = state.highlightTarget.target_pose;
            const cp = mapToCanvas({ x: tp.x || 0, y: tp.y || 0 }, anchor, scale);
            if (cp) {
              ctx.save();
              ctx.setTransform(1, 0, 0, 1, 0, 0);
              ctx.strokeStyle = "#f59e0b";
              ctx.lineWidth = 2.5;
              ctx.setLineDash([6, 4]);
              ctx.beginPath();
              ctx.arc(cp.x, cp.y, 18, 0, Math.PI * 2);
              ctx.stroke();
              ctx.setLineDash([]);
              ctx.restore();
            }
          }
          return;
        }
        const clusters = clusterInstances(instances, anchor, scale);
        // 动画插值
        const anim = state.clusterAnim;
        const now = performance.now();
        let animT = 1;
        if (anim && anim.active) {
          const elapsed = now - anim.startTs;
          const raw = Math.min(1, elapsed / anim.durationMs);
          animT = raw < 1 ? 1 - Math.pow(1 - raw, 3) : 1; // easeOutCubic
          if (raw >= 1) anim.active = false;
        }
        // 检测簇变化，启动动画
        detectAndStartClusterAnim(clusters, now);
        state._prevClusters = clusters;
        const hlKey = resolveHighlightCluster(clusters);
        ctx.save();
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        for (const cl of clusters) {
          let drawX = cl.cx;
          let drawY = cl.cy;
          if (anim && anim.active && anim.from[cl.key]) {
            const from = anim.from[cl.key];
            drawX = from.cx + (cl.cx - from.cx) * animT;
            drawY = from.cy + (cl.cy - from.cy) * animT;
          }
          const isHighlighted = cl.key === hlKey;
          const allFallback = cl.members.every((m) => isFallbackInstance(m) || isUnknownFrame(m));
          const anyUnknown = cl.members.some((m) => isUnknownFrame(m));
          const fallbackCount = cl.members.filter((m) => isFallbackInstance(m) || isUnknownFrame(m)).length;
          const isSingle = cl.members.length === 1;
          if (isSingle) {
            drawSingleMarker(ctx, drawX, drawY, cl.members[0], allFallback, anyUnknown, isHighlighted);
          } else {
            drawClusterMarker(ctx, drawX, drawY, cl, allFallback, fallbackCount, isHighlighted);
          }
        }
        // target_pose 保底环（无实例命中时）
        if (state.highlightTarget && state.highlightTarget.target_pose && !hlKey) {
          const tp = state.highlightTarget.target_pose;
          const cp = mapToCanvas({ x: tp.x || 0, y: tp.y || 0 }, anchor, scale);
          if (cp) {
            ctx.strokeStyle = "#f59e0b";
            ctx.lineWidth = 2.5;
            ctx.setLineDash([6, 4]);
            ctx.beginPath();
            ctx.arc(cp.x, cp.y, 18, 0, Math.PI * 2);
            ctx.stroke();
            ctx.setLineDash([]);
          }
        }
        ctx.restore();
      }
      function drawSingleMarker(ctx, x, y, inst, fallback, unknownFrame, highlighted) {
        const marker = semanticMarkerMetrics(ctx, inst, highlighted);
        const left = x - marker.width / 2;
        const top = y - marker.height / 2;
        drawCanvasRoundedRectPath(ctx, left, top, marker.width, marker.height, marker.radius);
        if (fallback || unknownFrame) {
          ctx.setLineDash([4, 3]);
          ctx.strokeStyle = highlighted ? "#fbbf24" : "#9ca3af";
          ctx.lineWidth = 1.8;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillStyle = "rgba(51,65,85,0.9)";
          ctx.fill();
        } else {
          ctx.fillStyle = highlighted ? "#fbbf24" : "#f59e0b";
          ctx.fill();
          ctx.strokeStyle = highlighted ? "#fff" : "#fef3c7";
          ctx.lineWidth = highlighted ? 2.5 : 1.5;
          ctx.stroke();
        }
        if (highlighted) {
          ctx.strokeStyle = "#fbbf24";
          ctx.lineWidth = 2;
          ctx.setLineDash([]);
          drawCanvasRoundedRectPath(ctx, left - 5, top - 5, marker.width + 10, marker.height + 10, marker.radius + 2);
          ctx.stroke();
        }
        ctx.font = marker.font;
        ctx.fillStyle = fallback || unknownFrame ? "#f8fafc" : "#111827";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(marker.label, x, y + 0.5);
        ctx.textBaseline = "alphabetic";
        if (unknownFrame) {
          ctx.font = "9px sans-serif";
          ctx.fillStyle = "#6b7280";
          ctx.fillText("坐标系未知", x, top + marker.height + 11);
        } else if (fallback) {
          ctx.font = "9px sans-serif";
          ctx.fillStyle = "#6b7280";
          ctx.fillText("待确认", x, top + marker.height + 11);
        }
      }
      function drawClusterMarker(ctx, x, y, cl, allFallback, fallbackCount, highlighted) {
        const marker = semanticClusterMarkerMetrics(ctx, cl.members.length, highlighted);
        const left = x - marker.width / 2;
        const top = y - marker.height / 2;
        const preciseCount = cl.members.length - fallbackCount;
        if (allFallback) {
          ctx.setLineDash([4, 3]);
          ctx.strokeStyle = highlighted ? "#fbbf24" : "#9ca3af";
          ctx.lineWidth = 2;
          ctx.fillStyle = "rgba(51,65,85,0.9)";
        } else {
          ctx.setLineDash([]);
          ctx.fillStyle = highlighted ? "#fbbf24" : "#f59e0b";
          ctx.strokeStyle = highlighted ? "#fbbf24" : "#f59e0b";
          ctx.lineWidth = highlighted ? 2.5 : 2;
        }
        drawCanvasRoundedRectPath(ctx, left, top, marker.width, marker.height, marker.radius);
        ctx.fill();
        ctx.stroke();
        ctx.setLineDash([]);
        if (!allFallback && fallbackCount > 0) {
          const ratio = preciseCount / cl.members.length;
          const barY = top + marker.height + 4;
          ctx.lineCap = "butt";
          ctx.beginPath();
          ctx.strokeStyle = "#f59e0b";
          ctx.lineWidth = 3;
          ctx.moveTo(left, barY);
          ctx.lineTo(left + marker.width * ratio, barY);
          ctx.stroke();
          ctx.beginPath();
          ctx.strokeStyle = "#6b7280";
          ctx.lineWidth = 3;
          ctx.moveTo(left + marker.width * ratio, barY);
          ctx.lineTo(left + marker.width, barY);
          ctx.stroke();
        }
        if (highlighted) {
          ctx.strokeStyle = "#fbbf24";
          ctx.lineWidth = 2;
          drawCanvasRoundedRectPath(ctx, left - 5, top - 5, marker.width + 10, marker.height + 10, marker.radius + 2);
          ctx.stroke();
        }
        ctx.font = marker.font;
        ctx.fillStyle = allFallback ? "#f8fafc" : "#111827";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(marker.label, x, y + 0.5);
        ctx.textBaseline = "alphabetic";
        if (allFallback) {
          ctx.font = "9px sans-serif";
          ctx.fillStyle = "#6b7280";
          ctx.fillText("待确认", x, top + marker.height + 11);
        }
      }
      function detectAndStartClusterAnim(newClusters, now) {
        const prev = state._prevClusters;
        if (!prev || !prev.length) return;
        const prevMap = new Map(prev.map((c) => [c.key, c]));
        const newMap = new Map(newClusters.map((c) => [c.key, c]));
        let changed = false;
        for (const [key, nc] of newMap) {
          const oc = prevMap.get(key);
          if (!oc || Math.abs(oc.cx - nc.cx) > 1 || Math.abs(oc.cy - nc.cy) > 1) { changed = true; break; }
        }
        if (!changed && prev.length === newClusters.length) return;
        const fromMap = {};
        for (const [key, oc] of prevMap) fromMap[key] = { cx: oc.cx, cy: oc.cy };
        const existing = state.clusterAnim;
        if (existing && existing.active) {
          existing.from = fromMap;
          existing.startTs = now;
        } else {
          state.clusterAnim = { active: true, startTs: now, durationMs: 220, from: fromMap };
          animateClusters();
        }
      }
      function animateClusters() {
        const anim = state.clusterAnim;
        if (!anim || !anim.active) return;
        const elapsed = performance.now() - anim.startTs;
        if (elapsed >= anim.durationMs) { anim.active = false; requestMapDraw(false); return; }
        requestMapDraw(false);
        window.requestAnimationFrame(animateClusters);
      }
      function hitTestSemanticMarkers(px, py) {
        if (!state.showSemanticInstances || !state.semanticInstances.length) return null;
        const anchor = cameraAnchor();
        const scale = cameraScale(anchor);
        const clusters = clusterInstances(semanticMapInstances(), anchor, scale);
        const highlightedKey = resolveHighlightCluster(clusters);
        const ctx = els.mapCanvas.getContext("2d");
        let bestDist = Number.POSITIVE_INFINITY;
        let bestHit = null;
        ctx.save();
        for (const cl of clusters) {
          const highlighted = cl.key === highlightedKey;
          const marker = cl.members.length === 1
            ? semanticMarkerMetrics(ctx, cl.members[0], highlighted)
            : semanticClusterMarkerMetrics(ctx, cl.members.length, highlighted);
          const withinX = Math.abs(px - cl.cx) <= marker.width / 2 + 4;
          const withinY = Math.abs(py - cl.cy) <= marker.height / 2 + 4;
          const dist = Math.sqrt((px - cl.cx) ** 2 + (py - cl.cy) ** 2);
          if (withinX && withinY && dist < bestDist) {
            bestDist = dist;
            bestHit = cl.members.length === 1
              ? { type: "single", instance: cl.members[0] }
              : { type: "cluster", cluster: cl };
          }
        }
        ctx.restore();
        return bestHit;
      }
      function closeSemanticOverlay() {
        const el = document.getElementById("semanticOverlay");
        if (el) {
          el.close();
          el.remove();
          if (state.semanticOverlayReturnFocus?.isConnected) state.semanticOverlayReturnFocus.focus();
          else els.semanticMemorySearch?.focus();
        }
      }
      async function navigateToInstance(inst, button, status) {
        if (state.semanticError || !semanticScopeMatches(inst) || inst.navigable !== true || semanticQuality(inst).key !== "located") {
          status.textContent = "当前记录缺少有效导航证据，请刷新或重新观察。";
          return;
        }
        const requestedMap = activeMapName();
        const requestedLibrary = state.semanticMeta.library_name;
        button.disabled = true;
        status.textContent = "正在查找此物品的可用导航候选…";
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), 10000);
        try {
          const response = await fetch("/api/capabilities/resolve_navigation_candidates/invoke", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ arguments: { query: inst.label || semanticInstanceName(inst), map_name: requestedMap, limit: 20 } }),
            signal: controller.signal,
          });
          const body = await response.json();
          if (!response.ok || body.ok === false) throw new Error(body.error?.message || body.detail || "导航候选查询失败");
          const candidates = (body.result || body).navigation_candidates;
          const candidate = Array.isArray(candidates) && candidates.find((item) =>
            item.record_id === inst.instance_id && item.map_name === requestedMap
            && item.metadata?.allowed_for_navigation === true && item.metadata?.resolution_decision === "navigate");
          if (!candidate) throw new Error("此物品当前没有可用导航候选，请补充观察或在地图中选择可达位置。");
          const pose = candidate.target_pose;
          const point = pose?.position;
          if (!semanticFinitePose(point) || pose.frame_id !== navigationFrameId()) throw new Error("候选位置或坐标系无效。");
          if (requestedMap !== activeMapName() || requestedLibrary !== state.semanticMeta.library_name || state.semanticError) throw new Error("地图或记忆库已变化，请刷新后重试。");
          const robotPoint = pointFromPose(state.pose);
          const yaw = planarYawFromPose(pose) ?? (robotPoint ? Math.atan2(point.y - robotPoint.y, point.x - robotPoint.x) : 0);
          state.highlightTarget = { record_id: inst.instance_id, target_pose: point, target_name: semanticInstanceName(inst) };
          closeSemanticOverlay();
          // 候选决定平面观察位置；机体高度仍由统一导航入口核对，不能使用物品中心高度。
          await dispatchNavGoal(point, yaw);
        } catch (error) {
          status.textContent = error.name === "AbortError" ? "导航候选查询超时，请重试。" : String(error.message || error);
        } finally {
          window.clearTimeout(timeout);
          button.disabled = false;
        }
      }
      function formatLastSeen(ts) {
        if (ts === null || ts === undefined || ts === "") return "未记录";
        const d = new Date(typeof ts === "number" && ts < 1e12 ? ts * 1000 : ts);
        return Number.isFinite(d.getTime()) ? d.toLocaleString(undefined, { year: "numeric", month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit" }) : "时间无效";
      }
      function semanticDetailShell(title) {
        const dialog = document.createElement("dialog");
        dialog.id = "semanticOverlay";
        dialog.className = "semantic-overlay";
        dialog.setAttribute("aria-labelledby", "semanticOverlayTitle");
        const head = document.createElement("div");
        head.className = "semantic-memory-head";
        const heading = document.createElement("strong");
        heading.id = "semanticOverlayTitle";
        heading.textContent = title;
        const close = document.createElement("button");
        close.className = "tool";
        close.type = "button";
        close.dataset.semanticAction = "close";
        close.textContent = "×";
        close.setAttribute("aria-label", "关闭物品详情");
        close.addEventListener("click", closeSemanticOverlay);
        head.append(heading, close);
        dialog.append(head);
        dialog.addEventListener("cancel", (event) => { event.preventDefault(); closeSemanticOverlay(); });
        dialog.addEventListener("click", (event) => {
          if (event.target !== dialog) return;
          const rect = dialog.getBoundingClientRect();
          if (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom) closeSemanticOverlay();
        });
        return dialog;
      }
      function locateSemanticInstance(inst) {
        if (!semanticScopeMatches(inst) || semanticQuality(inst).key !== "located" || state.semanticError) return;
        if (isMapToolActive() || window.__nuwaxMapToolV2?.active()) {
          updateGoalPickStatus("请先结束地图选点或区域绘制，再定位查看物品。", "error");
          return;
        }
        closeSemanticOverlay();
        setMapViewMode("2d");
        setMap2dCameraMode("free");
        state.mapFreeView.center = { x: inst.pose.x, y: inst.pose.y };
        state.highlightTarget = { record_id: inst.instance_id, target_name: semanticInstanceName(inst) };
        requestMapDraw(false);
      }
      function buildDetailCard(inst) {
        const quality = semanticQuality(inst);
        const dialog = semanticDetailShell(semanticInstanceName(inst));
        dialog.dataset.instanceId = inst.instance_id;
        dialog.dataset.snapshot = JSON.stringify([inst, state.semanticError, activeMapName()]);
        const note = document.createElement("p");
        note.className = "semantic-detail-note";
        note.textContent = semanticObservationOnly(inst)
          ? "仅有观察证据：机器人曾在观察位置见过该物品，尚不能确定物品坐标。"
          : quality.key === "located" ? "已保存物体位置估计。历史位置不保证物品仍在原处，导航前会重新解析可用候选。"
          : "空间证据不足或坐标系不匹配，暂不作为地图物品位置显示。";
        const methodNames = { support_points_map: "地图支持点", support_points_camera: "相机支持点", explicit_map_pose: "显式地图位置", track_pose: "跟踪位置", detection3d_pose: "三维检测位置", map_point_samples: "地图支持点采样", camera_point_samples: "相机支持点采样", depth_camera_projection: "相机深度投影", depth_pose_fallback: "深度与观察位姿估计", delayed_pointcloud_projection: "延后点云投影", current_pose_fallback: "观察者位置回退", explicit_position: "显式空间位置", pointcloud_projection: "点云投影", pointcloud_depth: "点云深度", depth_projection: "深度投影", ray_triangulation: "多视角三角化", deferred_pointcloud_refine: "延后点云修正", tracked_position: "跟踪空间位置" };
        const pose = semanticFinitePose(inst.pose) ? inst.pose : null;
        const confidence = Number.isFinite(inst.projection_confidence) ? `${Math.round(inst.projection_confidence * 100)}%` : "未记录";
        const rows = [
          ["标签", inst.label || "未分类"],
          ["空间证据", quality.label],
          ["地图 / 记忆库", `${inst.map_name || "未绑定地图"} / ${inst.library_name || state.semanticMeta.library_name || "未记录"}`],
          [semanticObservationOnly(inst) ? "观察位置" : "物体位置", pose ? `x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · z ${pose.z.toFixed(2)} 米` : "未确定"],
          ["坐标系", inst.frame_id || "未记录"],
          ["位置依据", methodNames[inst.projection_method] || inst.projection_method || "未记录"],
          ["空间置信度", confidence],
          ["支持点数", Number.isFinite(inst.support_point_count) ? String(inst.support_point_count) : "未记录"],
          ["观察次数", Number.isFinite(inst.observation_count) ? String(inst.observation_count) : "未记录"],
          ["记忆状态", ({ active: "有效", uncertain: "待确认", stale: "已过时", removed: "已移除" })[inst.lifecycle_state] || "未记录"],
          ["移动状态", ({ static: "静止", dynamic: "移动中", quasi_static: "短时静止", unknown: "未确定" })[inst.dynamic_state] || "未记录"],
          ["最后见到", formatLastSeen(inst.last_seen_ts)],
          ["实例标识", inst.instance_id || "未记录"],
        ];
        const fields = document.createElement("dl");
        fields.className = "semantic-detail-fields";
        for (const [key, value] of rows) {
          const term = document.createElement("dt");
          const description = document.createElement("dd");
          term.textContent = key;
          description.textContent = value;
          fields.append(term, description);
        }
        const actions = document.createElement("div");
        actions.className = "semantic-detail-actions";
        const locate = document.createElement("button");
        locate.type = "button";
        locate.dataset.semanticAction = "locate";
        locate.className = "btn";
        locate.textContent = "在地图中查看";
        locate.disabled = quality.key !== "located" || !semanticScopeMatches(inst) || Boolean(state.semanticError) || !state.mapGrid;
        locate.addEventListener("click", () => locateSemanticInstance(inst));
        const navigate = document.createElement("button");
        navigate.type = "button";
        navigate.dataset.semanticAction = "navigate";
        navigate.className = "btn primary";
        navigate.textContent = "导航到可用观察点";
        navigate.disabled = locate.disabled || inst.navigable !== true;
        const status = document.createElement("p");
        status.className = "semantic-memory-status";
        status.setAttribute("role", "status");
        status.textContent = state.semanticError ? "当前为上次缓存，请刷新后再操作。"
          : !semanticScopeMatches(inst) ? "此记录与当前地图不匹配。"
          : inst.navigable !== true ? "此记录尚不满足导航条件，需要补充空间观察。" : "导航受理与到点后的物品复核分别确认。";
        navigate.addEventListener("click", () => navigateToInstance(inst, navigate, status));
        actions.append(locate, navigate);
        dialog.append(note, fields, actions, status);
        return dialog;
      }
      function reconcileSemanticOverlay(previousLibrary) {
        const dialog = document.getElementById("semanticOverlay");
        if (!dialog) return;
        const instance = state.semanticInstances.find((item) => item.instance_id === dialog.dataset.instanceId);
        if (!instance || previousLibrary !== state.semanticMeta.library_name) {
          closeSemanticOverlay();
          return;
        }
        const snapshot = JSON.stringify([instance, state.semanticError, activeMapName()]);
        if (dialog.dataset.snapshot === snapshot) return;
        const focusedAction = document.activeElement?.dataset?.semanticAction;
        const scrollTop = dialog.scrollTop;
        const updated = buildDetailCard(instance);
        dialog.replaceChildren(...updated.childNodes);
        dialog.dataset.snapshot = snapshot;
        dialog.scrollTop = scrollTop;
        if (focusedAction) dialog.querySelector(`[data-semantic-action="${focusedAction}"]`)?.focus();
      }
      function openSemanticDialog(dialog) {
        state.semanticOverlayReturnFocus = document.activeElement;
        document.body.appendChild(dialog);
        dialog.showModal();
      }
      function showSemanticInstanceDetail(inst) {
        closeSemanticOverlay();
        state.semanticSelectedId = inst.instance_id;
        state.highlightTarget = { record_id: inst.instance_id, target_name: semanticInstanceName(inst) };
        for (const row of els.semanticMemoryList?.querySelectorAll?.(".semantic-memory-row") || []) {
          row.setAttribute("aria-pressed", String(row.dataset.instanceId === inst.instance_id));
        }
        requestMapDraw(false);
        openSemanticDialog(buildDetailCard(inst));
      }
      function showSemanticClusterList(cluster) {
        closeSemanticOverlay();
        const dialog = semanticDetailShell(`此处有 ${cluster.members.length} 个物品`);
        const list = document.createElement("div");
        list.className = "semantic-memory-list";
        for (const inst of cluster.members) list.append(semanticListRow(inst));
        dialog.append(list);
        openSemanticDialog(dialog);
      }
      function updateMap(envelope) {
        if (!acceptAppliedMapIdentity(envelope)) return;
        state.lastMapEnvelope = envelope;
        const status = statusOf(envelope);
        const data = envelope.data || {};
        const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
        const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
        const streamBridge = diagnostics.stream_bridge || {};
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        state.mapStream = { diagnostics, streamBridge, revision: envelope.revision, sequence: envelope.sequence, timestamp: envelope.source_timestamp, cleared };
        let displayDecision = shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
        if (!displayDecision.display && !cleared && status === "available"
            && grid && Array.isArray(grid.data) && grid.width && grid.height
            && state.pendingMapPose?.frame.status === "available"
            && pendingMapPoseMatches(state.receivedMapIdentity)) {
          displayDecision = { display: true, reason: "localization_pose_stream" };
        }
        if (!displayDecision.display) {
          clearMapDisplay(displayDecision.message, displayDecision.status || (cleared ? "cleared" : status), diagnostics, streamBridge, envelope, grid);
          releasePendingMapPose();
          return;
        }
        setPill(els.mapStatus, status);
        const pinCheckpoint = shouldPinLocalizationCheckpointDisplay(diagnostics);
        const incomingIsLive = !mapSnapshotIsOffline(diagnostics);
        const keepCheckpointGrid = pinCheckpoint && incomingIsLive && state.mapGrid && Array.isArray(state.mapGrid.data);
        if (grid && Array.isArray(grid.data) && !keepCheckpointGrid) {
          const nextContentKey = mapGridContentKey(grid, envelope, diagnostics);
          const baseDirty = state.mapGridContentKey !== nextContentKey;
          state.mapGrid = grid;
          state.appliedMapIdentity = state.receivedMapIdentity;
          state.mapGridContentKey = nextContentKey;
          const source = data.map_snapshot || data;
          recordMediaFrame("map2d", mapSnapshotIsOffline(diagnostics) ? null : videoSourceTime(envelope.source_timestamp),
            JSON.stringify([envelope.sequence, envelope.revision, envelope.source_timestamp, nextContentKey]),
            JSON.stringify([source.map_name, source.source_epoch, grid.frame_id]));
          const freeViewIdentity = JSON.stringify([
            source.map_name || diagnostics.map_name || "", source.source_name || "", source.source_epoch || "",
            grid.frame_id || source.frame_id || envelope.frame_id || "",
          ]);
          if (state.mapFreeViewIdentity !== freeViewIdentity) {
            state.mapFreeViewIdentity = freeViewIdentity;
            state.mapFreeView = { center: null, zoom: 1, fitSpan: null };
            if (state.map2dCameraMode === "free") resetFreeMapView();
          }
          els.mapEmpty.hidden = status === "available";
          els.mapEmpty.textContent = status === "stale" ? "地图数据已过期，保留最后一帧。" : "";
          requestMapDraw(baseDirty);
        } else if (keepCheckpointGrid) {
          els.mapEmpty.hidden = true;
          els.mapEmpty.textContent = "";
          requestMapDraw(false);
        }
        renderMapSummary({ grid, diagnostics, envelope });
        updateMapStreamChip(cleared ? "cleared" : status, diagnostics, streamBridge, envelope.source_timestamp);
        releasePendingMapPose();
      }
      function isMapClearedEnvelope(envelope, diagnostics, status) {
        const cleared = status === "cleared"
          || (typeof (diagnostics && (diagnostics.event_type || diagnostics.suppressed_live_map_reason)) === "string"
              && ((diagnostics.event_type || diagnostics.suppressed_live_map_reason) === "catalog_cleared"
                  || (diagnostics.event_type || diagnostics.suppressed_live_map_reason) === "workspace_cleared"));
        // 地图/工作区切换时清空 2D Pose 历史 marker，避免旧坐标错位被误读为新位姿
        if (cleared && Array.isArray(state.initialPoses) && state.initialPoses.length) {
          state.initialPoses = [];
        }
        return cleared;
      }
      function mapClearedMessage(diagnostics) {
        const cleared = diagnostics && (diagnostics.cleared_map_name || diagnostics.previous_active_map_name);
        const reason = diagnostics && diagnostics.reason;
        if (cleared && reason) return `地图 ${cleared} 已清空（${reason}）。`;
        if (cleared) return `地图 ${cleared} 已清空。`;
        if (reason) return `地图已清空（${reason}）。`;
        return "地图已清空，等待新工作区激活。";
      }
      // 仅统计被接受的新帧，不将重绘、心跳或同一缓存的重复应用计入帧率。
      function recordMediaFrame(kind, sourceTime, frameKey, sourceKey = "") {
        state.mediaMetrics ||= {};
        let metric = state.mediaMetrics[kind];
        if (!metric || metric.sourceKey !== sourceKey) {
          metric = state.mediaMetrics[kind] = { samples: [], sourceKey, frameKey: null, sourceTime: null };
        }
        if (frameKey != null && metric.frameKey === frameKey) return;
        const now = performance.now();
        metric.samples = metric.samples.filter((time) => now - time <= 5000).slice(-599);
        metric.samples.push(now);
        metric.frameKey = frameKey;
        metric.sourceTime = Number.isFinite(sourceTime) && sourceTime > 0 ? sourceTime : null;
      }
      function mediaMetricText(kind, now = Date.now(), monotonicNow = performance.now()) {
        const metric = state.mediaMetrics?.[kind];
        if (!metric) return "-- FPS · 延迟 --";
        metric.samples = metric.samples.filter((time) => monotonicNow - time <= 5000);
        const samples = metric.samples;
        const span = samples.length > 1 ? samples[samples.length - 1] - samples[0] : 0;
        const fps = span > 0 && monotonicNow - samples[samples.length - 1] <= 2000 ? (samples.length - 1) * 1000 / span : 0;
        const age = metric.sourceTime === null ? null : now - metric.sourceTime;
        const delay = age !== null && Number.isFinite(age) && age >= 0 ? `${Math.round(age)} ms` : "--";
        return `${fps.toFixed(1)} FPS · 延迟 ${delay}`;
      }
      function recordVideoFrameMetric() {
        const identity = state.videoDisplayedIdentity || {};
        recordMediaFrame("video", identity.sourceTime, state.videoFrames, `${state.videoStreamGeneration}|${identity.cameraId || ""}`);
      }
      function updateMapStreamChip() {
        if (!els.mapStreamStatus) return;
        const kind = state.mapViewMode === "3d" ? "map3d" : "map2d";
        const visible = kind === "map3d" || Boolean(state.mapGrid);
        setVisionText(els.mapStreamStatus, visible ? mediaMetricText(kind) : "-- FPS · 延迟 --");
        els.mapStreamStatus.title = "地图更新帧率；延迟为最新地图帧时间至当前时刻，不包含未知的传感器采集耗时。";
      }
      function updateMapHeartbeat(envelope) {
        state.lastMapHeartbeatEnvelope = envelope;
        const status = statusOf(envelope);
        const heartbeat = envelope.data || {};
        const diagnostics = { ...((state.mapStream && state.mapStream.diagnostics) || {}), ...(envelope.metadata || {}), ...heartbeat };
        const streamBridge = diagnostics.stream_bridge || {};
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        const grid = cleared ? null : state.mapGrid;
        state.mapStream = { diagnostics, streamBridge, revision: envelope.revision || diagnostics.content_revision, sequence: envelope.sequence || diagnostics.update_sequence, timestamp: envelope.source_timestamp, cleared };
        const displayDecision = shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
        if (!displayDecision.display) {
          clearMapDisplay(displayDecision.message, displayDecision.status || (cleared ? "cleared" : status), diagnostics, streamBridge, envelope, grid);
          return;
        }
        setPill(els.mapStatus, status);
        if (grid && Array.isArray(grid.data)) {
          els.mapEmpty.hidden = true;
          els.mapEmpty.textContent = "";
        } else {
          els.mapEmpty.hidden = false;
          els.mapEmpty.textContent = "地图源在线，等待内容更新帧。";
        }
        renderMapSummary({
          grid,
          diagnostics,
          envelope,
          displayStatus: cleared ? "已清空" : undefined,
        });
        updateMapStreamChip(status, diagnostics, streamBridge, envelope.source_timestamp);
      }
      function reevaluateCachedMapDisplay() {
        if (!state.lastMapEnvelope) {
          reevaluateCachedLocalizationDisplay();
          return;
        }
        const envelope = state.lastMapEnvelope;
        const data = envelope.data || {};
        const grid = data.occupancy_grid || (data.map_snapshot && data.map_snapshot.occupancy_grid) || null;
        const diagnostics = data.diagnostics || data.metadata || (data.map_snapshot && data.map_snapshot.metadata) || {};
        const streamBridge = diagnostics.stream_bridge || {};
        const status = statusOf(envelope);
        const cleared = isMapClearedEnvelope(envelope, diagnostics, status);
        const decision = shouldDisplayMapFrame(envelope, diagnostics, streamBridge, grid, status, cleared);
        if (decision.display || state.mapGrid) updateMap(envelope);
        else if (state.mapGrid) {
          clearMapDisplay(decision.message, decision.status || status, diagnostics, streamBridge, envelope, grid);
        }
        reevaluateCachedLocalizationDisplay();
      }
      function reevaluateCachedLocalizationDisplay() {
        if (!state.lastLocalizationEnvelope) return;
        applyLocalizationDisplay(state.lastLocalizationEnvelope, { reevaluateMap: false });
      }
      function applyLocalizationDisplay(envelope, options) {
        const reevaluateMap = !options || options.reevaluateMap !== false;
        const wasPoseStreamAvailable = localizationPoseStreamReason() !== null;
        state.lastLocalizationEnvelope = envelope;
        const data = envelope.data || {};
        const incomingPose = data.pose || (pointFromPose(data) ? data : null);
        if (pointFromPose(incomingPose)) state.latestPose = incomingPose;
        else if (envelope.type === "localization.pose") state.latestPose = null;
        updateMapPoseHud();
        if (!shouldDisplaySpatialPose()) {
          clearLocalizationDisplay(LOCALIZATION_DISPLAY_IDLE_MESSAGE, "missing");
          requestMapDraw(false);
          return;
        }
        const status = statusOf(envelope);
        state.pose = incomingPose;
        state.trajectory = Array.isArray(data.trajectory) ? data.trajectory : [];
        setPill(els.localizationStatus, status);
        const p = pointFromPose(state.pose);
        const yaw = yawFromPose(state.pose);
        setKv(els.localizationMeta, [
          ["坐标系", envelope.frame_id || (state.pose && state.pose.frame_id)],
          ["位姿", p ? `(${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)})` : errorReason(envelope, "--")],
          ["偏航角", yaw === null ? "--" : fmt(yaw, " rad")],
          ["轨迹点数", state.trajectory.length],
          ["数据时间", shortTime(envelope.source_timestamp)],
          ["延迟", fmt(envelope.latency_ms, " ms")],
          ["已过期", displayBool(envelope.stale === true)],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        // 持续位姿只移动覆盖层；定位可用性改变时才重新评估缓存地图。
        if (reevaluateMap && wasPoseStreamAvailable !== (localizationPoseStreamReason() !== null)) {
          reevaluateCachedMapDisplay();
        } else requestMapDraw(false);
      }
      function updateLocalization(envelope) {
        applyLocalizationDisplay(envelope, { reevaluateMap: true });
      }
      function firstArrayValue(...values) {
        for (const value of values) {
          if (Array.isArray(value) && value.length) return value;
        }
        return [];
      }
      function pathPointFromValue(value) {
        if (!value) return null;
        if (Array.isArray(value) && value.length >= 2) return { x: Number(value[0]), y: Number(value[1]), z: Number(value[2] || 0) };
        if (value.pose) {
          const point = pointFromPose(value.pose);
          if (point) return point;
        }
        if (value.position) {
          const point = pointFromPose(value);
          if (point) return point;
        }
        if (value.target_pose) {
          const point = pointFromPose(value.target_pose);
          if (point) return point;
        }
        if (value.x !== undefined && value.y !== undefined) return { x: Number(value.x), y: Number(value.y), z: Number(value.z || 0) };
        return pointFromFrontier(value);
      }
      function goalPoseFromValue(value) {
        if (!value) return null;
        const pose = value.target_pose || value.goal_pose || value.pose || value;
        const point = pointFromPose(pose) || pathPointFromValue(value);
        if (!point) return null;
        if (pose.orientation) return pose;
        const yaw = firstFiniteNumber(pose.yaw, pose.yaw_rad, value.yaw, value.yaw_rad);
        if (yaw === null) return { position: point };
        return {
          position: point,
          orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
        };
      }
      function explorationStateMetadata(context) {
        if (!context) return {};
        const stateObj = context.exploration_state || context;
        return (stateObj && stateObj.metadata) || context.metadata || {};
      }
      function extractNavigationPath(context, explorationContext) {
        const metadata = context && context.metadata ? context.metadata : {};
        const stateObj = context && context.navigation_state ? context.navigation_state : {};
        const stateMetadata = stateObj && stateObj.metadata ? stateObj.metadata : {};
        const explorationMetadata = explorationStateMetadata(explorationContext);
        const raw = firstArrayValue(
          context && context.planned_path,
          context && context.current_path,
          context && context.remaining_path,
          context && context.nav_path,
          context && context.waypoints,
          metadata.planned_path,
          metadata.current_path,
          metadata.remaining_path,
          metadata.nav_path,
          metadata.waypoints,
          stateMetadata.planned_path,
          stateMetadata.current_path,
          stateMetadata.remaining_path,
          stateMetadata.nav_path,
          stateMetadata.waypoints,
          explorationContext && explorationContext.planned_path,
          explorationMetadata.planned_path,
          explorationMetadata.nav_path
        );
        return raw.map(pathPointFromValue).filter(Boolean);
      }
      function extractNavigationGoal(context, explorationContext) {
        const metadata = context && context.metadata ? context.metadata : {};
        const navigationState = context && context.navigation_state ? context.navigation_state : {};
        const navigationMetadata = navigationState && navigationState.metadata ? navigationState.metadata : {};
        const explorationMetadata = explorationStateMetadata(explorationContext);
        for (const candidate of [
          context && context.current_goal,
          context && context.goal_pose,
          navigationState.current_goal,
          metadata.current_goal,
          navigationMetadata.current_goal,
          explorationContext && explorationContext.current_goal,
          explorationContext && explorationContext.goal_pose,
          explorationContext && explorationContext.selected_frontier,
          explorationContext && explorationContext.current_frontier,
          explorationMetadata.current_goal,
          explorationMetadata.selected_frontier,
          explorationMetadata.current_frontier,
        ]) {
          const pose = goalPoseFromValue(candidate);
          if (pose) return pose;
        }
        return null;
      }
      function normalizeExplorerStatusText(statusText) {
        const text = safeText(statusText);
        if (!text) return "";
        return text.replace(/^\[explorer\]\s*/i, "").trim();
      }
      function isTerminalExplorerStatus(statusText) {
        const upper = normalizeExplorerStatusText(statusText).toUpperCase();
        return upper.includes("DONE") || upper.includes("EXPLORATION FINISHED") || upper.includes("COMPLETE");
      }
      function summarizeContext(context, key) {
        const stateObj = context && context[key];
        if (!stateObj) return "--";
        const metadata = stateObj.metadata || {};
        const explorerStatus = metadata.status_text || metadata.explorer_status_text;
        if (metadata.exploration_terminal || metadata.terminal) {
          const terminal = metadata.terminal_status || stateObj.status || "completed";
          if (explorerStatus && isTerminalExplorerStatus(explorerStatus)) {
            return `${displayStatus(terminal)} · ${normalizeExplorerStatusText(explorerStatus)}`;
          }
          return displayStatus(terminal);
        }
        if (explorerStatus) {
          const base = stateObj.status || stateObj.phase || stateObj.message || stateObj.state || "available";
          return `${displayStatus(base)} · ${normalizeExplorerStatusText(explorerStatus)}`;
        }
        return displayStatus(stateObj.status || stateObj.phase || stateObj.message || stateObj.state || "available");
      }
      function updateGovernance(envelope) {
        const data = envelope.runtime_governance || (envelope.data && envelope.data.runtime_governance) || envelope.data || {};
        state.latestGovernance = data;
        data.status = data.status || (envelope.status ? statusOf(envelope) : undefined);
        if (data.service_health) {
          mergeServiceHealth(data.service_health, "runtime.governance", "replace");
        } else {
          renderServiceStatus();
        }
        reevaluateCachedMapDisplay();
        renderMapSummary();
      }
      function mergeLatestFrontierStream(data) {
        const fr = state.latestFrontierStream;
        if (!fr) return data;
        const merged = { ...data };
        if (!shouldRetainFrontierStreamCache(fr, merged)) {
          if (!merged.frontiers || !merged.frontiers.length) merged.frontiers = [];
          if (merged.frontier_count === null || merged.frontier_count === undefined) merged.frontier_count = 0;
          if (merged.exploration_terminal === undefined || merged.exploration_terminal === null) {
            merged.exploration_terminal = fr.exploration_terminal === true || fr.frontier_cleared === true;
          }
          return merged;
        }
        if (!merged.explorer_status_text && fr.status_text) merged.explorer_status_text = fr.status_text;
        if ((merged.frontier_count === null || merged.frontier_count === undefined) && fr.frontier_count != null) {
          merged.frontier_count = fr.frontier_count;
        }
        if ((!merged.frontiers || !merged.frontiers.length) && Array.isArray(fr.latest_frontiers) && fr.latest_frontiers.length) {
          merged.frontiers = fr.latest_frontiers;
        }
        if ((!merged.planned_path || !merged.planned_path.length) && Array.isArray(fr.planned_path) && fr.planned_path.length) {
          merged.planned_path = fr.planned_path;
        }
        if (!merged.current_goal && fr.current_goal) merged.current_goal = fr.current_goal;
        if (!merged.current_frontier && fr.current_frontier) merged.current_frontier = fr.current_frontier;
        if (!merged.selected_frontier && fr.selected_frontier) merged.selected_frontier = fr.selected_frontier;
        if (merged.exploration_terminal === undefined || merged.exploration_terminal === null) {
          merged.exploration_terminal = fr.exploration_terminal;
        }
        return merged;
      }
      function updateNavigation(envelope) {
        const status = statusOf(envelope);
        const data = mergeLatestFrontierStream(envelope.data || {});
        const frontierCache = state.latestFrontierStream;
        const explorationFrame = state.latestStreamFrames["exploration.state"];
        const frontierClearSequence = normalizedStreamCursor(frontierCache && frontierCache.sequence);
        const explorationSequence = normalizedStreamCursor(
          explorationFrame && (explorationFrame.sequence ?? explorationFrame.cursor)
        );
        const explorationIsNewerThanClear = frontierClearSequence !== null
          && explorationSequence !== null
          && (
            explorationSequence.length > frontierClearSequence.length
            || (
              explorationSequence.length === frontierClearSequence.length
              && explorationSequence > frontierClearSequence
            )
          );
        const explorationContext = frontierCache && frontierCache.frontier_cleared === true && !explorationIsNewerThanClear
          ? null
          : data.exploration_context;
        const explorationState = explorationContext && explorationContext.exploration_state;
        const explorationMetadata = explorationState && explorationState.metadata ? explorationState.metadata : {};
        const latestExplorationIdentity = state.latestExplorationOverlayIdentity;
        if (
          data.navigation_update_frame === true
          && latestExplorationIdentity
          && !navigationGoalMatchesExploration(
            data.navigation_context,
            latestExplorationIdentity
          )
        ) return;
        setPill(els.navigationStatus, status);
        const frontierCount = data.frontier_count !== undefined && data.frontier_count !== null
          ? data.frontier_count
          : explorationState && explorationState.frontier_count;
        const waitingFor = data.waiting_for;
        let explorerStatus = data.explorer_status_text
          || explorationMetadata.status_text
          || explorationMetadata.explorer_status_text;
        const explorationTerminal = data.exploration_terminal
          || explorationMetadata.exploration_terminal
          || (explorationState && ["succeeded", "failed", "cancelled"].includes(String(explorationState.status || "")));
        if (explorationTerminal && explorerStatus && !isTerminalExplorerStatus(explorerStatus)) {
          explorerStatus = explorationMetadata.status_text && isTerminalExplorerStatus(explorationMetadata.status_text)
            ? explorationMetadata.status_text
            : "DONE (exploration finished)";
        }
        const frontiers = Array.isArray(data.frontiers) ? data.frontiers : null;
        state.frontiers = frontiers || [];
        state.goalPose = extractNavigationGoal(data.navigation_context, explorationContext)
          || goalPoseFromValue(data.current_goal)
          || goalPoseFromValue(data.current_frontier)
          || goalPoseFromValue(data.selected_frontier);
        state.navigationPath = extractNavigationPath(data.navigation_context, explorationContext);
        if (!state.navigationPath.length && Array.isArray(data.planned_path)) {
          state.navigationPath = data.planned_path.map(pathPointFromValue).filter(Boolean);
        }
        const canonicalExploration = String(explorationMetadata.state_source || "").toLowerCase() === "geometry_runtime_grpc"
          || String(explorationMetadata.exploration_mode || "").toLowerCase() === "nuwax_geometry_runtime";
        const explorationStatus = String(explorationState && explorationState.status || "").toLowerCase();
        const canonicalInactive = Boolean(latestExplorationIdentity && (
          latestExplorationIdentity.terminal
          || latestExplorationIdentity.status === "idle"
          || latestExplorationIdentity.frontierCount === 0
          || (
            latestExplorationIdentity.sessionMode
            && latestExplorationIdentity.sessionMode !== "explore"
          )
        )) || (canonicalExploration && (
          explorationTerminal
          || explorationStatus === "idle"
          || (
            explorationMetadata.geometry_session_mode
            && String(explorationMetadata.geometry_session_mode).toLowerCase() !== "explore"
          )
        ));
        syncDebugNavGoalFromStream(data.navigation_context || data);
        if (canonicalInactive) {
          state.goalPose = null;
          state.navigationPath = [];
        } else if (
          latestExplorationIdentity
          && !navigationGoalMatchesExploration(
            data.navigation_context,
            latestExplorationIdentity
          )
        ) {
          state.goalPose = null;
          state.navigationPath = [];
        }
        if (state.pendingNavGoal && state.pendingNavGoal.status === "accepted" && state.goalPose) {
          state.pendingNavGoal = null;
        }
        setKv(els.navigationMeta, [
          ["导航", summarizeContext(data.navigation_context, "navigation_state")],
          ["探索", summarizeContext(data.exploration_context, "exploration_state")],
          ["探索器状态", explorerStatus ? normalizeExplorerStatusText(explorerStatus) : "--"],
          ["探索终止", explorationTerminal === true || explorationTerminal === false ? displayBool(explorationTerminal === true) : "--"],
          ["前沿数量", frontierCount === null || frontierCount === undefined ? "--" : frontierCount],
          ["等待原因", waitingFor || "无"],
          ["前沿几何", frontiers && frontiers.length ? `${frontiers.length} 个几何` : "暂无候选"],
          ["目标点", state.goalPose ? "已设置" : "--"],
          ["导航路径", state.navigationPath.length ? `${state.navigationPath.length} 点` : "--"],
          ["数据时间", shortTime(envelope.source_timestamp)],
          ["延迟", fmt(envelope.latency_ms, " ms")],
          ["已过期", displayBool(envelope.stale === true)],
          ["前端接收", shortTime(envelope.__receivedAt)],
        ]);
        if (els.frontierNotice) {
          els.frontierNotice.hidden = Boolean(explorationTerminal || (frontiers && frontiers.length));
          els.frontierNotice.textContent = explorationTerminal
            ? "探索器已报告完成（DONE）。"
            : (frontiers && frontiers.length ? "" : "当前探索上下文还没有 frontier 几何，请确认探索运行态已生成候选。");
        }
        reevaluateCachedMapDisplay();
        requestMapDraw(false);
      }
      function scheduleVideoUpdate(envelope) {
        state.pendingVideoEnvelope = envelope;
        if (state.videoRenderFrame) return;
        state.videoRenderFrame = window.requestAnimationFrame(() => {
          state.videoRenderFrame = null;
          const next = state.pendingVideoEnvelope;
          state.pendingVideoEnvelope = null;
          if (next) updateVideo(next);
        });
      }
      function buildVideoFrameSrc(data) {
        if (data.stream_url) {
          const url = new URL(data.stream_url, window.location.origin);
          if (!url.searchParams.get("fps")) url.searchParams.set("fps", "15");
          return url.toString();
        }
        return data.data_uri || data.uri || "";
      }
      function revokeVideoObjectUrl() {
        if (!state.videoCurrentObjectUrl) return;
        try {
          URL.revokeObjectURL(state.videoCurrentObjectUrl);
        } catch (err) {
          // 忽略浏览器释放失败；下一帧会覆盖显示。
        }
        state.videoCurrentObjectUrl = null;
      }
      function setVideoStreamStatus(status, message) {
        state.videoStreamStatus = status;
        state.videoStreamMessage = message || "";
        renderVideoStreamSummary();
        scheduleVideoDetectionDraw();
      }
      function setVisionText(element, text) {
        if (element && element.textContent !== text) element.textContent = text;
      }
      function videoSourceTime(value) {
        if (typeof value !== "string" || !value.trim()) return null;
        const parsed = Date.parse(value);
        return Number.isFinite(parsed) ? parsed : null;
      }
      function videoFrameIdentity(data) {
        return {
          cameraId: typeof data?.camera_id === "string" ? data.camera_id : "",
          frameId: typeof data?.frame_id === "string" ? data.frame_id : "",
          sourceTime: videoSourceTime(data?.frame_timestamp),
          timestampSource: data?.frame_timestamp_source || "",
          sequence: data?.frame_sequence ?? data?.sequence ?? null,
        };
      }
      function videoHasFreshImage(now = Date.now()) {
        const sourceTime = state.videoDisplayedIdentity?.sourceTime;
        return Boolean(!els.videoFrame.hidden && state.videoLastFrameAt && now - state.videoLastFrameAt <= VIDEO_DISPLAY_FRESH_MS
          && (sourceTime === null || sourceTime === undefined || (now - sourceTime <= VIDEO_DISPLAY_FRESH_MS && sourceTime - now <= VIDEO_DETECTION_ALIGNMENT_MS)));
      }
      function renderVideoStreamSummary() {
        const fresh = videoHasFreshImage();
        const hasFrame = Boolean(state.videoLastFrameAt && !els.videoFrame.hidden);
        const waiting = ["idle", "pending", "connecting", "streaming"].includes(state.videoStreamStatus);
        const failed = state.videoStreamStatus === "error";
        const status = fresh ? "available" : failed ? "error" : hasFrame ? "stale" : waiting ? "pending" : "missing";
        const label = fresh ? "视频可用" : failed ? "视频不可用" : hasFrame ? "画面已过时" : "等待视频";
        const key = `${status}|${label}|${state.videoStreamMessage || ""}`;
        if (key !== state.videoSummaryKey) {
          state.videoSummaryKey = key;
          setPill(els.videoStatus, status, label);
          els.videoEmpty.hidden = fresh;
          if (!fresh) setVisionText(els.videoEmpty, hasFrame ? "视频已中断，当前保留上次画面。" : state.videoStreamMessage || "等待摄像头视频帧。");
        }
        setVisionText(els.videoFpsStatus, hasFrame ? mediaMetricText("video") : "-- FPS · 延迟 --");
        els.videoFpsStatus.title = state.videoDisplayedIdentity?.timestampSource === "video_relay.write_time"
          ? "实际显示帧率；延迟从视频中转写入时间计算。"
          : "实际显示帧率；延迟从图像携带的帧时间计算，缺少时间时显示 --。";
      }
      function visionDisplayState() {
        const frame = state.videoDetectionFrame;
        if (els.videoFrame.hidden || !state.videoSourceSize) return { key: "no_video", drawable: false };
        if (!frame) return { key: "waiting", drawable: false };
        const shown = state.videoDisplayedIdentity;
        // 慢推理继续显示最近有效框；只拒绝已明确的相机或像素尺寸不匹配。
        if (shown?.cameraId && frame.identity.cameraId && frame.identity.cameraId !== shown.cameraId) return { key: "camera_mismatch", drawable: false };
        if (frame.identity.frameId && shown?.frameId && frame.identity.frameId !== shown.frameId) return { key: "frame_mismatch", drawable: false };
        const size = state.videoSourceSize;
        if ((frame.width && size.width !== frame.width) || (frame.height && size.height !== frame.height)) return { key: "size_mismatch", drawable: false };
        if (state.videoDetections.length) return { key: "ready", drawable: true };
        return { key: frame.failed ? "error" : "empty", drawable: false };
      }
      function renderVisionPanel() {
        const display = visionDisplayState();
        if (!videoPanelVisible()) return display;
        const frame = state.videoDetectionFrame;
        setVisionText(els.visionModelStatus, frame?.backend && frame.backend !== "未上报" ? frame.backend : "模型 --");
        const labelsKey = `${display.key}|${state.videoDetectionRevision}`;
        if (els.visionLabels && state.visionLabelsKey !== labelsKey) {
          state.visionLabelsKey = labelsKey;
          const fragment = document.createDocumentFragment();
          if (display.drawable) {
            const grouped = new Map();
            for (const detection of state.videoDetections) grouped.set(detection.label, (grouped.get(detection.label) || 0) + 1);
            for (const [label, count] of Array.from(grouped).slice(0, 8)) {
              const tag = document.createElement("span");
              tag.textContent = `${label}${count > 1 ? ` × ${count}` : ""}`;
              fragment.append(tag);
            }
            if (grouped.size > 8) {
              const more = document.createElement("span");
              more.textContent = `另有 ${grouped.size - 8} 类`;
              fragment.append(more);
            }
          }
          els.visionLabels.replaceChildren(fragment);
        }
        return display;
      }
      function scheduleVideoStreamReconnect(reason) {
        if (!state.videoStreamSrc) return;
        if (state.videoStreamReconnectTimer) return;
        const delayMs = state.videoStreamReconnectDelayMs;
        const nextDelayMs = Math.min(VIDEO_STREAM_MAX_RECONNECT_MS, Math.max(VIDEO_STREAM_INITIAL_RECONNECT_MS, delayMs * 1.6));
        setVideoStreamStatus("reconnecting", `${reason || "MJPEG 视频流中断"}，${Math.round(delayMs)} ms 后重连。`);
        state.videoStreamReconnectTimer = window.setTimeout(() => {
          state.videoStreamReconnectTimer = null;
          state.videoStreamReconnectDelayMs = nextDelayMs;
          startVideoStream({ force: true, reason: "reconnect" });
        }, delayMs);
      }
      function stopVideoStream(reason) {
        state.videoStreamGeneration += 1;
        if (state.videoStreamController) {
          try {
            state.videoStreamController.abort();
          } catch (err) {
            // AbortController 可能已结束。
          }
          state.videoStreamController = null;
        }
        if (state.videoStreamReconnectTimer) {
          window.clearTimeout(state.videoStreamReconnectTimer);
          state.videoStreamReconnectTimer = null;
        }
        setVideoStreamStatus("stopped", reason || "视频流已停止。");
      }
      function clearVideoDisplay(reason) {
        stopVideoStream(reason);
        revokeVideoObjectUrl();
        els.videoFrame.removeAttribute("src");
        els.videoFrame.hidden = true;
        state.videoStreamSrc = null;
        state.videoPendingSingleFrame = null;
        state.lastVideoFrameKey = null;
        state.videoDisplayedIdentity = null;
        state.videoLastFrameAt = null;
        scheduleVideoDetectionDraw();
        renderVisionPanel();
      }
      function startVideoStream(options) {
        const opts = options || {};
        const videoSrc = opts.streamUrl
          ? new URL(opts.streamUrl, window.location.origin).toString()
          : buildVideoFrameSrc({ stream_url: DEFAULT_VIDEO_STREAM_URL });
        if (!videoSrc) return;
        if (state.videoRejectedSource === videoSrc && Date.now() < state.videoRejectedSourceUntil) return;
        if (!opts.force && state.videoStreamSrc === videoSrc && state.videoStreamController) return;
        const reconnectDelay = opts.reason === "reconnect" ? state.videoStreamReconnectDelayMs : VIDEO_STREAM_INITIAL_RECONNECT_MS;
        if (state.videoStreamController) {
          try {
            state.videoStreamController.abort();
          } catch (err) {
            // 旧连接可能已结束。
          }
          state.videoStreamController = null;
        }
        if (state.videoStreamReconnectTimer) {
          window.clearTimeout(state.videoStreamReconnectTimer);
          state.videoStreamReconnectTimer = null;
        }
        state.videoStreamReconnectDelayMs = reconnectDelay;
        state.videoStreamGeneration += 1;
        state.videoStreamSrc = videoSrc;
        const generation = state.videoStreamGeneration;
        const controller = new AbortController();
        state.videoStreamController = controller;
        state.videoStreamBytes = 0;
        state.videoDecodedBytes = 0;
        state.videoLastError = "";
        setVideoStreamStatus("connecting", "正在连接平台接收的摄像头视频流。");
        consumeMjpegStream(videoSrc, controller.signal, generation).catch((err) => {
          if (generation !== state.videoStreamGeneration || controller.signal.aborted) return;
          if (err && err.code === "video_not_ready") {
            state.videoLastError = "";
            state.videoStreamController = null;
            scheduleVideoStreamReconnect("等待摄像头视频帧");
            setVideoStreamStatus("pending", "等待摄像头视频帧，接收到数据后会自动显示。");
            return;
          }
          if (err && err.code === "video_camera_unavailable") {
            state.videoStreamController = null;
            state.videoRejectedSource = videoSrc;
            state.videoRejectedSourceUntil = Date.now() + 30000;
            state.videoDisplayedIdentity = null;
            state.videoLastFrameAt = null;
            els.videoFrame.hidden = true;
            setVideoStreamStatus("error", err.message);
            return;
          }
          const message = err && err.message ? err.message : String(err || "未知错误");
          state.videoLastError = message;
          state.videoStreamErrors = (state.videoStreamErrors || 0) + 1;
          state.videoStreamController = null;
          scheduleVideoStreamReconnect(`视频流连接失败：${message}`);
        });
      }
      function resetVideoStreamFromMetadata(videoSrc) {
        if (!videoSrc) return false;
        const latestRenderedAgeMs = state.videoLastFrameAt ? Date.now() - state.videoLastFrameAt : Infinity;
        if (
          state.videoStreamSrc === videoSrc
          && !state.videoStreamController
          && !state.videoStreamReconnectTimer
          && latestRenderedAgeMs > VIDEO_STREAM_MAX_RECONNECT_MS
        ) {
          startVideoStream({ streamUrl: videoSrc, force: true, reason: "metadata_watchdog" });
          return true;
        }
        return false;
      }
      async function consumeMjpegStream(videoSrc, signal, generation) {
        const response = await fetch(videoSrc, { method: "GET", cache: "no-store", signal });
        if (!response.ok) {
          let detail = null;
          try { detail = await response.json(); } catch (_) { /* 非结构化错误保留响应状态。 */ }
          if (response.status === 503 && detail?.reason === "rust_bridge_no_frames") {
            const waiting = new Error(detail.message || "等待摄像头视频帧");
            waiting.code = "video_not_ready";
            throw waiting;
          }
          const error = new Error(detail?.message || `视频服务返回 ${response.status}`);
          error.code = detail?.error || "video_stream_error";
          throw error;
        }
        if (!response.body || !response.body.getReader) {
          throw new Error("当前浏览器不支持 ReadableStream MJPEG 解码。");
        }
        state.videoStreamReconnectDelayMs = VIDEO_STREAM_INITIAL_RECONNECT_MS;
        setVideoStreamStatus("streaming", "已连接 MJPEG 字节流，等待首帧 JPEG 解码。");
        const reader = response.body.getReader();
        let buffer = new Uint8Array(0);
        try {
          while (generation === state.videoStreamGeneration) {
            const result = await reader.read();
            if (result.done) break;
            if (!result.value || !result.value.byteLength) continue;
            state.videoStreamBytes += result.value.byteLength;
            buffer = appendBytes(buffer, result.value);
            if (buffer.byteLength > MJPEG_MAX_BUFFER_BYTES) {
              const soi = findMarker(buffer, 0xff, 0xd8, Math.max(0, buffer.byteLength - 1024 * 1024));
              buffer = soi >= 0 ? buffer.slice(soi) : new Uint8Array(0);
              if (!buffer.byteLength) throw new Error("MJPEG 缓冲区超过上限且未找到 JPEG 起始标记。");
            }
            let parsedAny = false;
            let latestFrameBytes = null;
            let latestFrameIdentity = null;
            while (true) {
              const parsed = extractNextJpeg(buffer);
              if (!parsed) break;
              if (!parsed.frame) {
                buffer = parsed.remaining;
                break;
              }
              parsedAny = true;
              latestFrameBytes = parsed.frame;
              latestFrameIdentity = parsed.identity;
              buffer = parsed.remaining;
            }
            if (latestFrameBytes) {
              try {
                await renderDecodedVideoFrame(latestFrameBytes, generation, latestFrameIdentity || {});
              } catch (err) {
                if (generation !== state.videoStreamGeneration || signal.aborted) return;
                const message = err && err.message ? err.message : String(err || "未知错误");
                state.videoLastError = message;
                state.videoDecodeErrors += 1;
                state.videoStreamController = null;
                scheduleVideoStreamReconnect(`JPEG 帧解码失败：${message}`);
                return;
              }
            }
            if (!parsedAny && state.videoFrames === 0) {
              setVideoStreamStatus("streaming", `已接收 ${state.videoStreamBytes} 字节，等待完整 JPEG 帧。`);
            }
          }
        } finally {
          // 解码失败或换源提前退出时也取消字节读取，避免失去旧连接的拥有方。
          try { await reader.cancel(); } catch (_) { /* 已中止或已结束的读取器无需再次处理。 */ }
          try {
            reader.releaseLock();
          } catch (err) {
            // 读取器可能已经释放。
          }
        }
        if (generation === state.videoStreamGeneration) {
          state.videoStreamController = null;
          scheduleVideoStreamReconnect("MJPEG 字节流已结束");
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
        const begin = Math.max(0, Number(start || 0));
        for (let i = begin; i < bytes.byteLength - 1; i += 1) {
          if (bytes[i] === a && bytes[i + 1] === b) return i;
        }
        return -1;
      }
      function extractNextJpeg(buffer) {
        const soi = findMarker(buffer, 0xff, 0xd8, 0);
        if (soi < 0) {
          return {
            frame: null,
            remaining: buffer.byteLength > 16384 ? buffer.slice(buffer.byteLength - 16384) : buffer,
          };
        }
        const prefix = new TextDecoder().decode(buffer.subarray(Math.max(0, soi - 16384), soi));
        const boundary = prefix.lastIndexOf("--frame\r\n");
        const header = boundary >= 0 ? prefix.slice(boundary) : prefix;
        const lengthHeader = header.match(/^Content-Length:\s*([0-9]+)\s*$/mi)?.[1];
        const length = lengthHeader ? Number(lengthHeader) : null;
        if (length !== null && (!Number.isSafeInteger(length) || length < 4 || length > MJPEG_MAX_BUFFER_BYTES)) throw new Error("视频帧长度超出有效范围。");
        // 有长度头时以传输边界为准，图像元数据中的内嵌缩略图结束标记不能截断主图。
        const eoi = length !== null ? soi + length - 2 : findMarker(buffer, 0xff, 0xd9, soi + 2);
        if (eoi < 0 || eoi + 2 > buffer.byteLength) {
          return {
            frame: null,
            remaining: soi > 16384 ? buffer.slice(soi - 16384) : buffer,
          };
        }
        const frame = buffer.slice(soi, eoi + 2);
        const remaining = buffer.slice(eoi + 2);
        if (!frame.byteLength) return null;
        const cameraValue = header.match(/^X-Camera-Id:\s*([^\r\n]*)/mi)?.[1] || "";
        let cameraId = "";
        try { cameraId = decodeURIComponent(cameraValue.trim()); } catch (_) { /* 无效头不作为来源依据。 */ }
        const timestamp = header.match(/^X-Frame-Timestamp:\s*([^\r\n]*)/mi)?.[1]?.trim();
        const sequence = header.match(/^X-Frame-Sequence:\s*([0-9]+)/mi)?.[1] || null;
        const timestampSource = header.match(/^X-Frame-Timestamp-Source:\s*([^\r\n]*)/mi)?.[1]?.trim() || "";
        return { frame, remaining, identity: { cameraId, sourceTime: videoSourceTime(timestamp), timestampSource, sequence } };
      }
      async function renderDecodedVideoFrame(frameBytes, generation, identity = {}) {
        if (!frameBytes || !frameBytes.byteLength) return;
        const blob = new Blob([frameBytes], { type: "image/jpeg" });
        if (generation !== state.videoStreamGeneration) return;
        const nextUrl = await displayVideoBlob(blob, generation);
        if (generation !== state.videoStreamGeneration) {
          try {
            URL.revokeObjectURL(nextUrl);
          } catch (err) {
            // 过期帧释放失败不影响下一轮。
          }
          return;
        }
        state.videoCurrentObjectUrl = nextUrl;
        state.videoDecodedBytes += frameBytes.byteLength;
        state.videoFrames += 1;
        state.videoLastFrameAt = Date.now();
        state.videoDisplayedIdentity = { ...identity, receivedAt: state.videoLastFrameAt };
        recordVideoFrameMetric();
        state.videoSourceSize = { width: els.videoFrame.naturalWidth, height: els.videoFrame.naturalHeight };
        state.videoStreamStatus = "rendering";
        els.videoFrame.hidden = false;
        els.videoEmpty.hidden = true;
        scheduleVideoDetectionDraw();
      }
      async function displayVideoBlob(blob, generation) {
        const nextUrl = URL.createObjectURL(blob);
        const previousUrl = state.videoCurrentObjectUrl;
        try {
          await new Promise((resolve, reject) => {
            let settled = false;
            const timer = window.setTimeout(() => {
              if (settled) return;
              settled = true;
              cleanup();
              reject(new Error("JPEG 显示超时"));
            }, VIDEO_JPEG_DECODE_TIMEOUT_MS);
            function cleanup() {
              window.clearTimeout(timer);
              els.videoFrame.removeEventListener("load", onLoad);
              els.videoFrame.removeEventListener("error", onError);
            }
            function onLoad() {
              if (settled) return;
              settled = true;
              cleanup();
              resolve();
            }
            function onError() {
              if (settled) return;
              settled = true;
              cleanup();
              reject(new Error("JPEG 显示失败"));
            }
            els.videoFrame.addEventListener("load", onLoad, { once: true });
            els.videoFrame.addEventListener("error", onError, { once: true });
            if (generation !== state.videoStreamGeneration) {
              settled = true;
              cleanup();
              resolve();
              return;
            }
            els.videoFrame.src = nextUrl;
          });
        } catch (err) {
          try {
            URL.revokeObjectURL(nextUrl);
          } catch (revokeErr) {
            // 忽略失败。
          }
          throw err;
        }
        if (generation !== state.videoStreamGeneration) return nextUrl;
        if (previousUrl) {
          // 新图已完成加载，旧对象地址可立即释放，无需为每帧另起延时器。
          try { URL.revokeObjectURL(previousUrl); } catch (_) { /* 释放失败不阻塞新图。 */ }
        }
        return nextUrl;
      }
      function updateVideo(envelope) {
        const data = envelope.data || {};
        if (Array.isArray(data.detections_2d)) {
          updateVideoDetections(data, envelope);
          return;
        }
        const metadata = videoFrameIdentity(data);
        const previousCamera = state.videoFrameMetadata?.cameraId;
        state.videoFrameMetadata = metadata;
        if (previousCamera && metadata.cameraId && previousCamera !== metadata.cameraId) {
          state.videoDisplayedIdentity = null;
          state.videoLastFrameAt = null;
          els.videoFrame.hidden = true;
          scheduleVideoDetectionDraw();
        }
        const videoSrc = buildVideoFrameSrc(data);
        if (videoSrc && data.available !== false && !["missing", "failed", "error", "stale"].includes(statusOf(envelope))) {
          if (data.stream_url) {
            if (state.videoStreamSrc !== videoSrc || (!state.videoStreamController && !state.videoStreamReconnectTimer)) {
              startVideoStream({ streamUrl: videoSrc, force: state.videoStreamSrc !== videoSrc, reason: "metadata" });
            } else resetVideoStreamFromMetadata(videoSrc);
          } else if (state.lastVideoFrameKey !== videoSrc) {
            stopVideoStream("正在加载图像。");
            revokeVideoObjectUrl();
            state.videoPendingSingleFrame = { src: videoSrc, identity: metadata, generation: state.videoStreamGeneration };
            state.videoStreamSrc = videoSrc;
            state.lastVideoFrameKey = videoSrc;
            els.videoFrame.setAttribute("src", videoSrc);
          }
        } else if (data.available === false || !videoSrc) {
          clearVideoDisplay(errorReason(envelope, "尚未收到摄像头发布的视频帧。"));
        }
        renderVideoStreamSummary();
        renderVisionPanel();
        scheduleVideoDetectionDraw();
      }
      function updateVideoDetections(data, envelope) {
        data = data || {};
        envelope = envelope || {};
        const identity = videoFrameIdentity(data);
        const previous = state.videoDetectionFrame;
        // 同相机的乱序结果不能覆盖更新的观察；其它相机的结果保留明确不匹配状态。
        if (previous?.identity.cameraId === identity.cameraId && identity.sourceTime !== null
          && previous.identity.sourceTime !== null && identity.sourceTime < previous.identity.sourceTime) return;
        const detections = Array.isArray(data.detections_2d) ? data.detections_2d : [];
        const valid = detections.filter((detection) => {
          const bbox = detection && detection.bbox;
          return typeof detection?.label === "string" && !detection.label.startsWith("<truncated:")
            && Number.isFinite(detection.score) && detection.score >= 0 && detection.score <= 1 && bbox
            && [bbox.x_px, bbox.y_px, bbox.width_px, bbox.height_px].every(Number.isFinite)
            && bbox.width_px > 0 && bbox.height_px > 0;
        }).slice(0, 64);
        const total = Number.isSafeInteger(data.detection_count) && data.detection_count >= 0 ? data.detection_count : valid.length;
        const status = String(envelope.status || "missing").startsWith("<truncated:") ? "missing" : statusOf(envelope);
        const detectorStage = data.stage_status?.detector;
        const failed = data.available === false || ["missing", "failed", "error", "unavailable"].includes(status)
          || ["failed", "error", "unavailable"].includes(detectorStage);
        const next = {
          identity, width: data.width_px, height: data.height_px, total,
          truncated: data.detections_truncated === true || total !== valid.length,
          failed, stale: status === "stale" || envelope.stale === true, message: data.message || data.error_message || "",
          backend: data.detector_backend || "未上报",
          inferenceExecution: data.inference_execution || "",
          detectorMs: data.stage_duration_ms?.detector,
          waitMs: data.pipeline_lock_wait_ms,
          fallback: data.provider_fallback_used === true || data.worker_fallback_used === true,
          providerFallback: data.provider_fallback_used === true,
          workerFallback: data.worker_fallback_used === true,
          workerFallbackReason: data.worker_fallback_reason || "",
          fallbackReason: data.provider_fallback_reason || "",
          receivedAt: Date.now(),
        };
        const signature = JSON.stringify([valid, { ...next, receivedAt: undefined }]);
        if (signature !== state.videoDetectionSignature) {
          state.videoDetectionSignature = signature;
          updatePerceptionNmsCard(data, envelope);
        }
        const geometrySignature = JSON.stringify([valid, identity.cameraId, next.width, next.height]);
        if (geometrySignature !== state.videoDetectionGeometrySignature) {
          state.videoDetectionGeometrySignature = geometrySignature;
          state.videoDetectionRevision = (state.videoDetectionRevision || 0) + 1;
          state.videoDetections = valid;
        }
        state.videoDetectionFrame = next;
        renderVisionPanel();
        scheduleVideoDetectionDraw();
      }
      // vis-three-fixes 任务 1.7 / AC3.4：用 video.detections 流里的 nms_stats 填充感知 NMS 卡片。
      function updatePerceptionNmsCard(data, envelope) {
        const section = document.getElementById("overview-perception_nms");
        if (!section) return;
        const nms = (data && data.nms_stats) || {};
        const raw = nms.raw_count;
        const kept = nms.kept_count;
        const suppressed = nms.suppressed_count;
        const hasStats = raw !== undefined || kept !== undefined || suppressed !== undefined;
        setPill(section.querySelector(".pill"), hasStats ? statusOf(envelope) : "missing");
        const stageMs = (data && data.stage_duration_ms) || {};
        setKv(section.querySelector(".kv"), [
          ["原始框", raw === undefined ? "--" : String(raw)],
          ["保留框", kept === undefined ? "--" : String(kept)],
          ["抑制框", suppressed === undefined ? "--" : String(suppressed)],
          ["检测耗时", fmt(stageMs.detector, " ms")],
          ["锁等待", fmt(data && data.pipeline_lock_wait_ms, " ms")],
        ]);
        section.querySelector(".small").textContent = `前端接收 ${shortTime(envelope.__receivedAt)}`;
      }
      function videoPanelVisible() {
        return !document.hidden && document.getElementById("monitor-view")?.classList.contains("active") !== false;
      }
      function scheduleVideoDetectionDraw() {
        if (!videoPanelVisible() || state.videoDetectionDrawFrame) return;
        state.videoDetectionDrawFrame = window.requestAnimationFrame(() => {
          state.videoDetectionDrawFrame = null;
          drawVideoDetections();
        });
      }
      function videoOverlayGeometry(width, height, sourceWidth, sourceHeight) {
        const scale = Math.min(width / sourceWidth, height / sourceHeight);
        return { scale, x: (width - sourceWidth * scale) / 2, y: (height - sourceHeight * scale) / 2 };
      }
      function drawVideoDetections() {
        if (!videoPanelVisible()) return;
        const canvas = els.videoDetections;
        const image = els.videoFrame;
        const display = renderVisionPanel();
        const source = state.videoSourceSize;
        const ratio = Math.min(2, window.devicePixelRatio || 1);
        const drawKey = `${state.videoDetectionRevision}|${state.videoDetectionLayoutRevision}|${state.showDetections}|${display.drawable}|${source?.width}|${source?.height}|${ratio}`;
        if (state.videoDetectionDrawKey === drawKey) return;
        state.videoDetectionDrawKey = drawKey;
        if (!canvas || !image || !state.showDetections || image.hidden || !display.drawable || !state.videoDetections.length) {
          if (canvas) {
            canvas.hidden = true;
            const context = canvas.getContext("2d");
            context?.clearRect(0, 0, canvas.width, canvas.height);
          }
          return;
        }
        const stageRect = canvas.parentElement.getBoundingClientRect();
        const width = Math.round(stageRect.width);
        const height = Math.round(stageRect.height);
        if (!(width > 0 && height > 0)) { canvas.hidden = true; state.videoDetectionDrawKey = ""; return; }
        const pixelWidth = Math.round(width * ratio), pixelHeight = Math.round(height * ratio);
        if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
          canvas.width = pixelWidth;
          canvas.height = pixelHeight;
        }
        const geometry = videoOverlayGeometry(width, height, source.width, source.height);
        const context = canvas.getContext("2d");
        context.setTransform(ratio, 0, 0, ratio, 0, 0);
        context.clearRect(0, 0, width, height);
        context.lineWidth = 2;
        context.font = "12px sans-serif";
        for (const detection of state.videoDetections) {
          const bbox = detection.bbox;
          const left = Math.max(0, bbox.x_px), top = Math.max(0, bbox.y_px);
          const right = Math.min(source.width, bbox.x_px + bbox.width_px);
          const bottom = Math.min(source.height, bbox.y_px + bbox.height_px);
          if (!(right > left && bottom > top)) continue;
          const x = geometry.x + left * geometry.scale, y = geometry.y + top * geometry.scale;
          const boxWidth = (right - left) * geometry.scale, boxHeight = (bottom - top) * geometry.scale;
          const label = `${detection.label} ${Math.round(detection.score * 100)}%`;
          const labelMaxWidth = Math.max(20, Math.min(240, width - x - 10));
          const fitted = fitSemanticMarkerLabel(context, label, labelMaxWidth);
          const labelWidth = Math.min(width - x, context.measureText(fitted).width + 10);
          const labelTop = Math.max(geometry.y, y - 20);
          context.strokeStyle = "#22d3ee";
          context.strokeRect(x, y, boxWidth, boxHeight);
          context.fillStyle = "rgba(2, 6, 23, 0.88)";
          context.fillRect(x, labelTop, labelWidth, 20);
          context.fillStyle = "#e0f2fe";
          context.fillText(fitted, x + 5, labelTop + 14);
        }
        canvas.hidden = false;
      }
      function updateLogs(envelope) {
        const data = envelope.data || {};
        setPill(els.logsStatus, statusOf(envelope));
        window.nuwaxConsoleLogs?.ingest(Array.isArray(data.entries) ? data.entries : []);
      }
      const DEFAULT_VIDEO_STREAM_URL = "/streams/video.mjpeg";
      const STREAM_TOPICS = [
        "robot.state",
        "map.frame",
        "map.heartbeat",
        "localization.pose",
        "localization.frame_tree",
        "localization.trajectory",
        "navigation.state",
        "navigation.forbidden_zones",
        "exploration.state",
        "exploration.frontier",
        "runtime.governance",
        "pointcloud_map.meta",
        "pointcloud_map.snapshot",
        "pointcloud.frame",
        "video.frame",
        "video.detections",
        "logs.entry",
        "tasks.state",
        "system.metrics"
      ];
      const POINTCLOUD3D_STREAM_TOPICS = new Set([
        "map.frame",
        "map.heartbeat",
        "localization.pose",
        "localization.trajectory",
        "navigation.state",
        "navigation.forbidden_zones",
        "exploration.state",
        "exploration.frontier",
        "runtime.governance",
        "pointcloud_map.meta",
      ]);
      function navigationGoalMatchesExploration(navigationContext, identity) {
        const goal = navigationContext && navigationContext.current_goal;
        const metadata = goal && goal.metadata || {};
        if (!identity || !identity.requestId) return false;
        return String(metadata.parent_request_id || "").trim() === identity.requestId
          && normalizedStreamCursor(metadata.exploration_generation) === identity.generation;
      }
      function currentExplorationOverlayDetail() {
        const frame = state.latestStreamFrames["exploration.state"];
        const context = frame && frame.payload;
        const exploration = context && context.exploration_state;
        const metadata = exploration && exploration.metadata || {};
        const identity = state.latestExplorationOverlayIdentity;
        return {
          canonical_exploration: isCanonicalExplorationFrame(frame),
          exploration_identity: identity ? { ...identity } : null,
          exploration_terminal: identity ? identity.terminal : metadata.exploration_terminal === true,
          exploration_status: identity ? identity.status : String(exploration && exploration.status || "").toLowerCase(),
          geometry_session_mode: identity ? identity.sessionMode : String(metadata.geometry_session_mode || "").toLowerCase(),
          frontier_count: identity ? identity.frontierCount : exploration && exploration.frontier_count,
        };
      }
      function pointcloud3dStreamDetail(frame) {
        if (frame.topic === "localization.pose") return state.latestPose || null;
        if (frame.topic === "localization.trajectory") return { trajectory: state.trajectory || [] };
        if (frame.topic === "navigation.state") {
          return {
            planned_path: state.navigationPath || [],
            current_goal: state.goalPose || null,
            navigation_exploration_validated: navigationGoalMatchesExploration(
              frame.payload,
              state.latestExplorationOverlayIdentity
            ),
            ...currentExplorationOverlayDetail(),
          };
        }
        if (frame.topic === "exploration.state") {
          const navigationFrame = state.latestStreamFrames["navigation.state"];
          return {
            planned_path: state.navigationPath || [],
            current_goal: state.goalPose || null,
            navigation_exploration_validated: navigationGoalMatchesExploration(
              navigationFrame && navigationFrame.payload,
              state.latestExplorationOverlayIdentity
            ),
            ...currentExplorationOverlayDetail(),
          };
        }
        if (frame.topic === "exploration.frontier") {
          const navigationFrame = state.latestStreamFrames["navigation.state"];
          return {
            frontiers: state.frontiers || [],
            planned_path: state.navigationPath || [],
            current_goal: state.goalPose || null,
            navigation_exploration_validated: navigationGoalMatchesExploration(
              navigationFrame && navigationFrame.payload,
              state.latestExplorationOverlayIdentity
            ),
            ...currentExplorationOverlayDetail(),
          };
        }
        return frame.payload || {};
      }
      function envelopeFromFrame(frame, payloadOverride) {
        const receivedAt = new Date().toISOString();
        return {
          ok: frame.status !== "error" && frame.status !== "missing",
          type: frame.topic,
          source: frame.source,
          frame_id: frame.frame_id || (frame.payload && frame.payload.frame_id),
          source_timestamp: frame.timestamp || (frame.payload && frame.payload.timestamp),
          gateway_timestamp: frame.timestamp,
          sequence: frame.sequence ?? frame.cursor,
          cursor: frame.cursor,
          revision: frame.revision,
          status: frame.status || "available",
          stale: frame.status === "stale",
          data: payloadOverride === undefined ? frame.payload : payloadOverride,
          metadata: frame.metadata || {},
          __receivedAt: receivedAt,
          __frontendLatencyMs: 0,
        };
      }
      function buildStreamUrl(stream = state) {
        const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
        const url = new URL(`${protocol}//${window.location.host}/streams/ws`);
        // 位姿独立传输，避免大地图的发送队列阻塞小帧；续传游标也按连接隔离。
        const topics = stream.topics || STREAM_TOPICS.filter((topic) => topic !== "localization.pose");
        url.searchParams.set("topics", topics.join(","));
        if (stream.mapPayload) url.searchParams.set("map_payload", stream.mapPayload);
        if (stream.latestStreamCursor) url.searchParams.set("after_cursor", String(stream.latestStreamCursor));
        return url.toString();
      }
      const STREAM_CURSOR_MAX_U64 = "18446744073709551615";
      function normalizedStreamCursor(value) {
        let normalized = null;
        if (typeof value === "string" && /^(0|[1-9]\d*)$/.test(value)) {
          normalized = value;
        } else if (typeof value === "number" && Number.isSafeInteger(value) && value >= 0) {
          normalized = String(value);
        }
        if (normalized === null) return null;
        if (
          normalized.length > STREAM_CURSOR_MAX_U64.length
          || (
            normalized.length === STREAM_CURSOR_MAX_U64.length
            && normalized > STREAM_CURSOR_MAX_U64
          )
        ) return null;
        return normalized;
      }
      function updateLatestStreamCursor(value, allowReset = false, stream = state) {
        const next = normalizedStreamCursor(value);
        if (next === null) return false;
        const current = normalizedStreamCursor(stream.latestStreamCursor) || "0";
        if (allowReset || next.length > current.length || (next.length === current.length && next > current)) {
          stream.latestStreamCursor = next;
          return true;
        }
        return next === current;
      }
      function compareCanonicalInteger(left, right) {
        if (left.length !== right.length) return left.length < right.length ? -1 : 1;
        if (left === right) return 0;
        return left < right ? -1 : 1;
      }
      function mapIdentityKey(identity) {
        const fields = [identity?.map_name, identity?.source_name, identity?.source_epoch, identity?.frame_id];
        return fields.every((field) => typeof field === "string" && field.length > 0)
          ? JSON.stringify(fields) : null;
      }
      function hideMapPose() {
        state.lastLocalizationEnvelope = null;
        state.latestPose = null;
        clearLocalizationDisplay("等待当前地图与定位同步。", "missing");
        requestMapDraw(false);
        window.dispatchEvent(new CustomEvent("stream:localization.pose", { detail: null }));
      }
      function acceptPoseMapIdentity(message) {
        const cursor = normalizedStreamCursor(message.cursor);
        const cleared = message.cleared === true;
        const key = cleared ? null : mapIdentityKey(message);
        if (cursor === null || (!cleared && key === null)) return;
        const current = state.poseMapIdentity;
        if (current && compareCanonicalInteger(cursor, current.cursor) <= 0) return;
        const changed = !current || current.key !== key || current.cleared !== cleared;
        state.poseMapIdentity = { key, cursor, cleared, startCursor: changed ? cursor : current.startCursor };
        if (cleared) state.appliedMapIdentity = null;
        if (changed) {
          state.pendingMapPose = null;
          hideMapPose();
        }
        // 主连接重启后的低游标地图可能先被旧屏障挡住，新屏障到达后重试该候选。
        if (!cleared && (!state.appliedMapIdentity || state.appliedMapIdentity.key !== key)) {
          const mapFrame = state.latestStreamFrames["map.frame"];
          const source = mapFrame?.payload?.map_snapshot || mapFrame?.payload;
          const mapCursor = normalizedStreamCursor(mapFrame?.cursor);
          if (mapCursor !== null && compareCanonicalInteger(mapCursor, state.poseMapIdentity.startCursor) >= 0
              && mapIdentityKey({ ...source, frame_id: source?.frame_id || mapFrame?.frame_id }) === key) {
            state.pendingStreamFrames["map.frame"] = mapFrame;
            scheduleStreamFlush();
          }
        }
        releasePendingMapPose();
      }
      function acceptAppliedMapIdentity(envelope) {
        const cursor = normalizedStreamCursor(envelope.cursor);
        if (cursor === null) return false;
        const source = envelope.data?.map_snapshot || envelope.data;
        const cleared = envelope.status === "cleared" || source === null;
        const key = cleared ? null : mapIdentityKey({ ...source, frame_id: source?.frame_id || envelope.frame_id });
        if (!cleared && key === null) return false;
        const received = state.receivedMapIdentity;
        const barrier = state.poseMapIdentity;
        if (received && compareCanonicalInteger(cursor, received.cursor) < 0) return false;
        if (barrier && key !== barrier.key && compareCanonicalInteger(cursor, barrier.startCursor) <= 0) return false;
        if (!state.appliedMapIdentity && barrier && !cleared && compareCanonicalInteger(cursor, barrier.startCursor) < 0) return false;
        if (!received || received.key !== key || received.cleared !== cleared) hideMapPose();
        state.receivedMapIdentity = { key, cursor, cleared };
        return true;
      }
      function pendingMapPoseMatches(applied = state.appliedMapIdentity) {
        const pending = state.pendingMapPose;
        const barrier = state.poseMapIdentity;
        if (!pending || !barrier || barrier.cleared || !applied || applied.cleared || state.mapStreamReady === false) return false;
        if (pending.mapKey !== barrier.key || applied.key !== barrier.key) return false;
        const cursor = normalizedStreamCursor(pending.frame.cursor);
        return cursor !== null && compareCanonicalInteger(cursor, barrier.startCursor) > 0;
      }
      function releasePendingMapPose(reevaluateMap = false) {
        if (!pendingMapPoseMatches()) {
          if (reevaluateMap && !state.appliedMapIdentity && state.lastMapEnvelope?.status === "available"
              && pendingMapPoseMatches(state.receivedMapIdentity)) updateMap(state.lastMapEnvelope);
          return;
        }
        const pending = state.pendingMapPose;
        state.pendingMapPose = null;
        processStreamFrame(pending.frame);
      }
      function isCanonicalExplorationFrame(frame) {
        const exploration = frame && frame.payload && frame.payload.exploration_state;
        const metadata = exploration && exploration.metadata;
        if (!metadata) return false;
        const stateSource = String(metadata.state_source || "").trim().toLowerCase();
        const explorationMode = String(metadata.exploration_mode || "").trim().toLowerCase();
        return stateSource === "geometry_runtime_grpc"
          || explorationMode === "nuwax_geometry_runtime";
      }
      function canonicalExplorationFrameIdentity(frame) {
        const context = frame && frame.payload;
        const exploration = context && context.exploration_state;
        const metadata = exploration && exploration.metadata;
        if (!exploration || !metadata || !isCanonicalExplorationFrame(frame)) return null;
        const generation = normalizedStreamCursor(metadata.exploration_generation);
        const revision = normalizedStreamCursor(metadata.revision);
        const timestampNs = normalizedStreamCursor(metadata.timestamp_ns);
        if (generation === null || revision === null || timestampNs === null) return null;
        return {
          generation,
          revision,
          timestampNs,
          requestId: String(exploration.current_request_id || metadata.request_id || "").trim(),
          status: String(exploration.status || "").trim().toLowerCase(),
          sessionMode: String(metadata.geometry_session_mode || "").trim().toLowerCase(),
          terminal: metadata.exploration_terminal === true
            || ["succeeded", "failed", "cancelled"].includes(String(exploration.status || "").trim().toLowerCase()),
          frontierCount: Number.isSafeInteger(exploration.frontier_count) && exploration.frontier_count >= 0
            ? exploration.frontier_count
            : null,
        };
      }
      function compareExplorationOverlayIdentity(left, right) {
        const generationComparison = compareCanonicalInteger(left.generation, right.generation);
        if (generationComparison !== 0) return generationComparison;
        if (left.requestId !== right.requestId) return -1;
        for (const key of ["revision", "timestampNs"]) {
          const comparison = compareCanonicalInteger(left[key], right[key]);
          if (comparison !== 0) return comparison;
        }
        return 0;
      }
      function acceptCanonicalExplorationFrame(frame) {
        const canonical = isCanonicalExplorationFrame(frame);
        const identity = canonicalExplorationFrameIdentity(frame);
        if (identity === null) return !canonical && !state.explorationOverlayRequiresCanonical;
        const current = state.latestExplorationOverlayIdentity;
        if (current && compareExplorationOverlayIdentity(identity, current) <= 0) return false;
        state.latestExplorationOverlayIdentity = identity;
        state.explorationOverlayRequiresCanonical = false;
        state.latestFrontierStream = null;
        state.frontiers = [];
        if (
          identity.terminal
          || identity.status === "idle"
          || identity.frontierCount === 0
          || (identity.sessionMode && identity.sessionMode !== "explore")
        ) {
          state.navigationPath = [];
          state.goalPose = null;
        }
        return true;
      }
      function decodeMapBinaryFrame(buffer) {
        const bytes = new Uint8Array(buffer);
        const magic = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x4d, 0x41, 0x50];
        if (bytes.byteLength < 13 || !magic.every((value, index) => bytes[index] === value) || bytes[8] !== 2) return null;
        const headerLength = new DataView(buffer, 9, 4).getUint32(0, true);
        if (headerLength <= 0 || headerLength > 64 * 1024 || 13 + headerLength > bytes.byteLength) return null;
        try {
          const message = JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(bytes.subarray(13, 13 + headerLength)));
          const frame = message.frame;
          const payload = frame?.payload;
          const grid = payload?.occupancy_grid;
          if (message.type !== "stream.frame" || frame?.topic !== "map.frame" || !grid || typeof payload.map_name !== "string" || !payload.map_name) return null;
          const cells = grid.width * grid.height;
          if (![grid.width, grid.height, cells].every(Number.isSafeInteger) || grid.width < 1 || grid.height < 1 || cells > 2000000) return null;
          if (!Number.isFinite(grid.resolution_m) || grid.resolution_m <= 0 || !grid.frame_id || !grid.origin) return null;
          const origin = grid.origin.position;
          const orientation = grid.origin.orientation;
          if (!origin || !orientation || ![origin.x, origin.y, origin.z, orientation.x, orientation.y, orientation.z, orientation.w].every(Number.isFinite)) return null;
          if (bytes.byteLength - 13 - headerLength !== cells) return null;
          const data = new Int8Array(buffer, 13 + headerLength, cells);
          if (data.some((value) => value < -1 || value > 100)) return null;
          grid.data = Array.from(data);
          return frame;
        } catch (_) {
          return null;
        }
      }
      function routePointCloudMapBinary(buffer) {
        const mapFrame = decodeMapBinaryFrame(buffer);
        if (mapFrame) {
          handleStreamFrame(mapFrame);
          return true;
        }
        const bytes = new Uint8Array(buffer);
        const magic = [0x4e, 0x55, 0x57, 0x41, 0x58, 0x50, 0x43, 0x00];
        if (bytes.byteLength < 13 || !magic.every((value, index) => bytes[index] === value)) return false;
        if (bytes[8] !== 1) return false;
        const headerLength = new DataView(buffer, 9, 4).getUint32(0, true);
        if (headerLength <= 0 || headerLength > 64 * 1024 || 13 + headerLength > bytes.byteLength) return false;
        let header;
        try {
          header = JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(bytes.slice(13, 13 + headerLength)));
        } catch (_) {
          return false;
        }
        if (!header || !["pointcloud_map.snapshot", "pointcloud_map.live_chunk"].includes(header.topic)) return false;
        window.dispatchEvent(new CustomEvent("stream:pointcloud_map.binary", {
          detail: { buffer, routingHeader: header },
        }));
        return true;
      }
      function resetPointCloudMapBinaryConsumer(reason) {
        window.dispatchEvent(new CustomEvent("stream:pointcloud_map.reset", {
          detail: { reason },
        }));
      }
      function resetExplorationOverlayConsumer(reason) {
        state.latestExplorationOverlayIdentity = null;
        state.explorationOverlayRequiresCanonical = true;
        state.latestFrontierStream = null;
        state.frontiers = [];
        state.navigationPath = [];
        state.goalPose = null;
        state.highlightTarget = null;
        for (const topic of ["exploration.state", "exploration.frontier", "navigation.state"]) {
          delete state.latestStreamFrames[topic];
          delete state.pendingStreamFrames[topic];
        }
        requestMapDraw(false);
        window.dispatchEvent(new CustomEvent("stream:exploration.overlay-reset", {
          detail: { reason },
        }));
      }
      window.addEventListener("stream:pointcloud_map.cursor-accepted", (event) => {
        updateLatestStreamCursor(event.detail?.cursor);
      });
      function connectStreams(stream = state) {
        if (
          stream.streamSocket
          && [WebSocket.CONNECTING, WebSocket.OPEN].includes(stream.streamSocket.readyState)
        ) return;
        if (stream.streamReconnectTimer) {
          window.clearTimeout(stream.streamReconnectTimer);
          stream.streamReconnectTimer = null;
        }
        const socket = new WebSocket(buildStreamUrl(stream));
        const epoch = stream.streamConnectionEpoch + 1;
        stream.streamConnectionEpoch = epoch;
        socket.binaryType = "arraybuffer";
        stream.streamSocket = socket;
        let acceptsMessages = true;
        const isCurrentConnection = () => (
          socket === stream.streamSocket
          && epoch === stream.streamConnectionEpoch
        );
        socket.addEventListener("open", () => {
          if (!isCurrentConnection()) return;
          const wasReconnect = stream.streamReconnectAttempt > 0;
          stream.streamReconnectAttempt = 0;
          stream.streamReconnectDelayMs = 1000;
          if (stream.streamReconnectTimer) {
            window.clearTimeout(stream.streamReconnectTimer);
            stream.streamReconnectTimer = null;
          }
          if (wasReconnect && stream === state) loadInitialTasks();
        });
        socket.addEventListener("message", (event) => {
          if (!isCurrentConnection() || !acceptsMessages) return;
          if (event.data instanceof ArrayBuffer) {
            routePointCloudMapBinary(event.data);
            return;
          }
          const message = JSON.parse(event.data);
          if (message.type === "stream.map_identity" && stream === state.poseStream) {
            updateLatestStreamCursor(message.cursor, false, stream);
            acceptPoseMapIdentity(message);
            return;
          }
          if (message.type === "keepalive") {
            updateLatestStreamCursor(message.cursor, false, stream);
            return;
          }
          if (message.type === "stream.replay") {
            updateLatestStreamCursor(
              message.latest_cursor,
              message.resync_required && message.gap_reason === "cursor_ahead_of_stream",
              stream,
            );
            if (stream === state && (message.resync_required || message.replay_gap)) {
              resetPointCloudMapBinaryConsumer("replay_gap");
              resetExplorationOverlayConsumer("replay_gap");
              recoverStreamGap(message);
            }
            if (stream === state) {
              state.mapStreamReady = true;
              if (message.gap_reason === "cursor_ahead_of_stream") {
                state.receivedMapIdentity = null;
                state.appliedMapIdentity = null;
                const mapFrame = state.latestStreamFrames["map.frame"];
                const mapCursor = normalizedStreamCursor(mapFrame?.cursor);
                const latestCursor = normalizedStreamCursor(message.latest_cursor);
                if (mapCursor !== null && latestCursor !== null && compareCanonicalInteger(mapCursor, latestCursor) <= 0) {
                  state.pendingStreamFrames["map.frame"] = mapFrame;
                  scheduleStreamFlush();
                }
              }
              releasePendingMapPose(true);
            }
            return;
          }
          if (message.type === "stream.gap") {
            updateLatestStreamCursor(message.latest_cursor, false, stream);
            if (stream !== state) {
              acceptsMessages = false;
              state.poseMapIdentity = null;
              state.pendingMapPose = null;
              hideMapPose();
              socket.close();
              return;
            }
            resetPointCloudMapBinaryConsumer("queue_gap");
            resetExplorationOverlayConsumer("queue_gap");
            recoverStreamGap(message);
            return;
          }
          if (message.type !== "stream.frame" || !message.frame) return;
          handleStreamFrame(message.frame, stream);
        });
        socket.addEventListener("close", () => {
          if (!isCurrentConnection()) return;
          acceptsMessages = false;
          if (stream === state) {
            resetPointCloudMapBinaryConsumer("disconnect");
            resetExplorationOverlayConsumer("disconnect");
            state.mapStreamReady = false;
            delete state.pendingStreamFrames["map.frame"];
          } else {
            state.poseMapIdentity = null;
          }
          state.pendingMapPose = null;
          hideMapPose();
          stream.streamReconnectAttempt += 1;
          scheduleStreamReconnect(socket, epoch, stream);
        });
        socket.addEventListener("error", () => {
          if (!isCurrentConnection()) return;
          showPanelError(stream === state ? "logs" : "localization", "实时数据流连接异常，正在重连。");
        });
      }
      function recoverStreamGap(message) {
        const topics = Array.isArray(message?.topics) && message.topics.length ? new Set(message.topics) : null;
        if (!topics || topics.has("tasks.state")) void loadInitialTasks();
        if (!topics || topics.has("logs.entry")) {
          const logs = window.nuwaxConsoleLogs;
          if (typeof logs?.recoverGap === "function") void logs.recoverGap();
          else {
            void logs?.reload?.();
            showPanelError("logs", "实时日志有缺口；仅重读最近200条，更早记录请加载历史或导出。请刷新页面更新日志组件。");
          }
        }
      }
      function enqueueTaskFrame(queue, frame) {
        queue.push(frame);
        if (queue.length <= TASK_EVENT_QUEUE_LIMIT) return;
        queue.splice(0, queue.length - TASK_EVENT_QUEUE_LIMIT);
        if (state.taskQueueGapPending) return;
        state.taskQueueGapPending = true;
        showPanelError("tasks", "任务事件过多，正在重新读取权威任务状态。");
        void loadInitialTasks().finally(() => { state.taskQueueGapPending = false; });
      }
      function scheduleStreamReconnect(socket, epoch, stream = state) {
        const delayMs = Math.min(30000, Math.max(1000, stream.streamReconnectDelayMs));
        if (stream.streamReconnectTimer) return;
        stream.streamReconnectTimer = window.setTimeout(() => {
          if (socket !== stream.streamSocket || epoch !== stream.streamConnectionEpoch) return;
          stream.streamReconnectTimer = null;
          connectStreams(stream);
        }, delayMs);
        stream.streamReconnectDelayMs = Math.min(30000, delayMs * 2);
      }
      function updateReceiveLatency(frame) {
        const sentAtMs = Date.parse(frame && frame.timestamp ? frame.timestamp : "");
        if (!Number.isFinite(sentAtMs)) {
          setText(els.lastReceive, "--");
          return;
        }
        const latencyMs = Math.max(0, Date.now() - sentAtMs);
        setText(els.lastReceive, `${Math.round(latencyMs)} ms`);
      }
      function handleStreamFrame(frame, stream = state) {
        updateLatestStreamCursor(frame.cursor, false, stream);
        if (
          frame.topic === "exploration.frontier"
          && state.explorationOverlayRequiresCanonical
        ) return;
        if (frame.topic === "exploration.state" && !acceptCanonicalExplorationFrame(frame)) return;
        if (frame.topic === "tasks.state") frame = { ...frame, __taskSequence: ++state.taskEventSequence };
        state.latestStreamFrames[frame.topic] = frame;
        if (frame.topic === "localization.pose") {
          // 状态立即应用，二维与三维各自合并到下一动画帧，不先等待通用面板刷新。
          updateReceiveLatency(frame);
          if (!frame.payload || frame.status === "missing") {
            state.pendingMapPose = null;
            processStreamFrame(frame);
          } else {
            state.pendingMapPose = { frame, mapKey: state.poseMapIdentity?.key || null };
            releasePendingMapPose(true);
          }
          return;
        }
        if (frame.topic === "logs.entry") {
          state.pendingLogFrames.push(frame);
          if (state.pendingLogFrames.length > 200) {
            state.pendingLogFrames.splice(0, state.pendingLogFrames.length - 200);
            recoverStreamGap({ topics: ["logs.entry"] });
          }
        } else if (frame.topic === "tasks.state") {
          enqueueTaskFrame(state.pendingTaskFrames, frame);
        } else {
          state.pendingStreamFrames[frame.topic] = frame;
        }
        updateReceiveLatency(frame);
        scheduleStreamFlush();
      }
      function scheduleStreamFlush() {
        if (state.streamFlushFrame) return;
        state.streamFlushFrame = window.requestAnimationFrame(flushStreamFrames);
      }
      function flushStreamFrames() {
        state.streamFlushFrame = null;
        const pending = state.pendingStreamFrames;
        const logFrames = state.pendingLogFrames;
        const taskFrames = state.pendingTaskFrames;
        state.pendingStreamFrames = {};
        state.pendingLogFrames = [];
        state.pendingTaskFrames = [];
        for (const topic of STREAM_FLUSH_ORDER) {
          const frame = pending[topic];
          if (!frame) continue;
          delete pending[topic];
          processStreamFrame(frame);
        }
        for (const topic of Object.keys(pending)) processStreamFrame(pending[topic]);
        if (taskFrames.length) {
          state.taskStoreBatching = true;
          try { for (const frame of taskFrames) updateTaskFrame(frame); }
          finally { state.taskStoreBatching = false; }
          refreshTaskStore();
        }
        if (logFrames.length) updateLogFrames(logFrames);
      }
      function processStreamFrame(frame) {
        let updateOverviewForFrame = true;
        if (frame.topic === "robot.state") {
          updateOverviewForFrame = updateRobotStateFrame(frame);
        } else if (frame.topic === "map.frame") {
          updateMap(envelopeFromFrame(frame));
        } else if (frame.topic === "map.heartbeat") {
          updateMapHeartbeat(envelopeFromFrame(frame));
        } else if (frame.topic === "localization.pose") {
          updateLocalization(envelopeFromFrame(frame, { pose: frame.payload, trajectory: state.trajectory }));
        } else if (frame.topic === "localization.trajectory") {
          const trajectory = frame.payload && Array.isArray(frame.payload.trajectory) ? frame.payload.trajectory : [];
          state.trajectory = trajectory;
          const localization = state.lastLocalizationEnvelope;
          // 轨迹连接可能晚于位姿到达，不能用旧轨迹的状态和时间覆盖新定位。
          if (localization) {
            applyLocalizationDisplay({ ...localization, data: { ...localization.data, trajectory } }, { reevaluateMap: false });
          } else requestMapDraw(false);
        } else if (frame.topic === "navigation.state") {
          updateNavigation(envelopeFromFrame(frame, {
            navigation_context: frame.payload,
            exploration_context: state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload,
            navigation_update_frame: true,
          }));
          const navCtx = frame.payload || {};
          const goal = navCtx.current_goal || null;
          const navStatus = String((navCtx.status && (navCtx.status.value || navCtx.status)) || "").toLowerCase();
          const terminal = ["succeeded", "failed", "cancelled", "timeout"].includes(navStatus);
          const explorationIdentity = state.latestExplorationOverlayIdentity;
          const canonicalGoalAllowed = !explorationIdentity
            || navigationGoalMatchesExploration(navCtx, explorationIdentity);
          if (goal && !terminal && canonicalGoalAllowed) {
            state.highlightTarget = {
              record_id: goal.metadata && goal.metadata.record_id || null,
              record_kind: goal.metadata && goal.metadata.record_kind || null,
              target_pose: goal.target_pose || null,
              target_name: goal.target_name || null,
            };
          } else if (!explorationIdentity && terminal) {
            state.highlightTarget = null;
          }
          requestMapDraw(false);
        } else if (frame.topic === "exploration.state") {
          const navigationContext = state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload;
          updateNavigation(envelopeFromFrame(frame, { navigation_context: navigationContext, exploration_context: frame.payload }));
          const explorationIdentity = state.latestExplorationOverlayIdentity;
          if (
            explorationIdentity
            && (
              explorationIdentity.terminal
              || explorationIdentity.status === "idle"
              || explorationIdentity.frontierCount === 0
              || !navigationGoalMatchesExploration(navigationContext, explorationIdentity)
            )
          ) state.highlightTarget = null;
        } else if (frame.topic === "exploration.frontier") {
          updateFrontierFrame(frame);
        } else if (frame.topic === "runtime.governance") {
          updateGovernance(envelopeFromFrame(frame));
        } else if (frame.topic === "pointcloud_map.meta") {
          updatePointcloudMapFrame(frame);
        } else if (frame.topic === "pointcloud.frame") {
          updatePointcloudFrame(frame);
        } else if (frame.topic === "video.frame") {
          scheduleVideoUpdate(envelopeFromFrame(frame));
        } else if (frame.topic === "video.detections") {
          updateVideoDetections(frame.payload || {}, envelopeFromFrame(frame));
        } else if (frame.topic === "logs.entry") {
          updateLogFrame(frame);
        } else if (frame.topic === "tasks.state") {
          updateTaskFrame(frame);
        } else if (frame.topic === "system.metrics") {
          updateSystemMetrics(frame.payload);
          updateOverviewForFrame = false;
        } else {
          updateOverviewForFrame = false;
        }
        if (updateOverviewForFrame) updateStreamOverview(frame);
        if (POINTCLOUD3D_STREAM_TOPICS.has(frame.topic)) {
          window.dispatchEvent(new CustomEvent(`stream:${frame.topic}`, {
            detail: pointcloud3dStreamDetail(frame),
          }));
        }
      }
      function robotStateChangeKey(frame) {
        const metadata = frame.metadata || {};
        return metadata.content_hash || metadata.signature || frame.revision || null;
      }
      function updateRobotStateFrame(frame) {
        const payload = frame.payload;
        if (!payload || typeof payload !== "object" || Array.isArray(payload)) return false;
        const changeKey = robotStateChangeKey(frame);
        if (changeKey && changeKey === state.lastRobotStateHash) return false;
        state.lastRobotStateHash = changeKey;
        state.lastRobotStateRevision = frame.revision || (frame.metadata && frame.metadata.revision) || state.lastRobotStateRevision;
        updateHealth({
          runtime_started: true,
          robot_name: payload.robot_name,
          robot_model: payload.robot_model,
          assembly_status: payload.assembly_status,
          latest_robot_snapshot: payload.robot_state,
          active_task_count: state.latestTasks && Array.isArray(state.latestTasks.active_tasks) ? state.latestTasks.active_tasks.length : 0,
          latest_event_cursor: frame.cursor || 0,
        });
        return true;
      }
      function updateFrontierFrame(frame) {
        if (state.explorationOverlayRequiresCanonical) return;
        const payload = frame.payload || {};
        const frameMetadata = frame.metadata || {};
        const cleared = payload.frontier_cleared === true || frameMetadata.frontier_cleared === true;
        const sessionMode = String(payload.geometry_session_mode || "").trim().toLowerCase();
        if (cleared || (sessionMode && sessionMode !== "explore")) {
          state.latestFrontierStream = {
            status_text: payload.status_text,
            frontier_count: 0,
            latest_frontiers: [],
            planned_path: [],
            current_goal: null,
            current_frontier: null,
            selected_frontier: null,
            exploration_terminal: true,
            frontier_cleared: true,
            geometry_session_mode: sessionMode || "idle",
            sequence: frame.sequence,
            received_at: frame.timestamp,
          };
          state.frontiers = [];
          updateNavigation(envelopeFromFrame(frame, {
            navigation_context: state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload,
            exploration_context: state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload,
            frontiers: [],
            frontier_count: 0,
            explorer_status_text: payload.status_text,
            exploration_terminal: true,
            planned_path: [],
            current_goal: null,
            current_frontier: null,
            selected_frontier: null,
            geometry_session_mode: sessionMode || "idle",
          }));
          return;
        }
        state.latestFrontierStream = {
          status_text: payload.status_text,
          frontier_count: payload.frontier_count,
          latest_frontiers: Array.isArray(payload.latest_frontiers) ? payload.latest_frontiers : [],
          planned_path: Array.isArray(payload.planned_path) ? payload.planned_path : [],
          current_goal: payload.current_goal || null,
          current_frontier: payload.current_frontier || null,
          selected_frontier: payload.selected_frontier || null,
          exploration_terminal: payload.exploration_terminal,
          geometry_session_mode: sessionMode || "explore",
          sequence: frame.sequence,
          received_at: frame.timestamp,
        };
        const frontiers = Array.isArray(payload.latest_frontiers) ? payload.latest_frontiers : [];
        state.frontiers = frontiers;
        updateNavigation(envelopeFromFrame(frame, {
          navigation_context: state.latestStreamFrames["navigation.state"] && state.latestStreamFrames["navigation.state"].payload,
          exploration_context: state.latestStreamFrames["exploration.state"] && state.latestStreamFrames["exploration.state"].payload,
          frontiers,
          frontier_count: payload.frontier_count !== undefined && payload.frontier_count !== null ? payload.frontier_count : frontiers.length,
          explorer_status_text: payload.status_text,
          exploration_terminal: payload.exploration_terminal,
          planned_path: payload.planned_path,
          current_goal: payload.current_goal,
          current_frontier: payload.current_frontier,
          selected_frontier: payload.selected_frontier,
        }));
      }
      function updatePointcloudMapFrame(frame) {
        const summary = frame.payload || {};
        const envelope = envelopeFromFrame(frame, summary);
        const data = { pointcloud_map: { status: envelope.status, frame_id: envelope.frame_id, revision: envelope.revision, latency_ms: envelope.latency_ms, source_timestamp: envelope.source_timestamp, metadata: { source: envelope.source, source_kind: summary.source_kind || (frame.metadata && frame.metadata.source_kind), point_count: summary.point_count ?? (frame.metadata && frame.metadata.point_count), pcd_path: summary.pcd_path || (frame.metadata && frame.metadata.pcd_path) } } };
        updateSensorSummary({ ...(state.latestOverview || {}), ...data });
        renderMapSummary();
      }
      function updatePointcloudFrame(frame) {
        const payload = frame.payload || {};
        const envelope = envelopeFromFrame(frame, payload);
        const data = { pointcloud: { status: envelope.status, frame_id: envelope.frame_id, revision: envelope.revision, latency_ms: envelope.latency_ms, source_timestamp: envelope.source_timestamp, metadata: { source: envelope.source, point_count: payload.point_count || (frame.metadata && frame.metadata.point_count) } } };
        updateSensorSummary({ ...(state.latestOverview || {}), ...data });
      }
      function logEntryFromFrame(frame) {
        const payload = frame.payload || {};
        const details = payload.details || payload.details_compact || {};
        return {
          ...payload,
          id: payload.id || `${frame.source || "log"}:${frame.cursor ?? frame.sequence}`,
          timestamp: payload.timestamp || payload.timestamp_iso || frame.timestamp,
          level: String(payload.level || payload.severity || frame.status || "info").toLowerCase(),
          source: payload.source || "python",
          component: payload.component || payload.logger || frame.source || "平台服务",
          event: payload.event || payload.event_type || details.event,
          message: payload.message || payload.display_message || payload.runtime_message || "运行记录",
          request_id: payload.request_id || details.request_id,
          task_id: payload.task_id || details.task_id,
          details,
        };
      }
      function updateLogFrame(frame) {
        updateLogFrames([frame]);
      }
      function updateLogFrames(frames) {
        const latest = frames[frames.length - 1];
        const entries = frames.map(logEntryFromFrame);
        updateLogs(envelopeFromFrame(latest, { entries, next_cursor: latest.cursor, source_status: "streaming", metrics: {} }));
        updateStreamOverview(latest);
      }
      function updateTaskFrame(frame) {
        // 闸门：bootstrap 未完成时入队，避免流式单条事件抢在 /api/tasks 之前写入 Map
        if (!state.bootstrapComplete.tasks) {
          enqueueTaskFrame(state.bootstrapQueue.tasks, frame);
          return;
        }
        updateTaskFrameImpl(frame);
      }
      function updateTaskFrameImpl(frame) {
        const receiveSequence = frame.__taskSequence || ++state.taskEventSequence;
        const framePayload = frame.payload || {};
        if (Array.isArray(framePayload.active_tasks) || Array.isArray(framePayload.history_tasks)) {
          if (receiveSequence <= state.taskSnapshotSequence) return;
          replaceTaskStore(framePayload, receiveSequence);
          return;
        }
        const event = framePayload.event || framePayload;
        const payload = event.payload || {};
        const requestContext = payload.request_context || {};
        const taskId = event.task_id || payload.task_id || requestContext.task_id;
        if (!taskId) return;
        const key = String(taskId);
        const existing = state.taskActiveById.get(key) || state.taskHistoryById.get(key) || {};
        // 任务 3.2.1：state 字段回退链移除 event.event_type 兜底（避免 "task.progress" 字面量进入 state）
        const derivedStage = event.event_type ? String(event.event_type).replace(/^task\./, "") : null;
        const task = {
          ...existing,
          task_id: taskId,
          capability_name: payload.capability_name || requestContext.capability_name || existing.capability_name,
          state: payload.state || event.state || (isTerminalTaskState(derivedStage) || ["accepted", "running"].includes(derivedStage) ? derivedStage : null) || existing.state || null,
          stage: payload.stage || existing.stage || derivedStage,
          message: payload.message || event.message || existing.message,
          progress: payload.progress !== undefined ? payload.progress : existing.progress,
          progress_percent: payload.progress_percent !== undefined ? payload.progress_percent : existing.progress_percent,
          error_code: payload.error_code || (payload.error_payload && payload.error_payload.code) || existing.error_code,
          error_category: payload.error_category || (payload.error_payload && payload.error_payload.error_category) || existing.error_category,
          retryable: payload.retryable !== undefined ? payload.retryable : payload.error_payload && payload.error_payload.retryable !== undefined ? payload.error_payload.retryable : existing.retryable,
          next_action: payload.next_action || (payload.error_payload && payload.error_payload.next_action) || existing.next_action,
          updated_at: payload.updated_at || event.timestamp || frame.timestamp || existing.updated_at,
          completed_at: payload.completed_at || existing.completed_at,
          __streamSequence: receiveSequence,
        };
        // 历史HTTP只有有限窗口，仍保留已收到的终态；旧活动事件不能复活快照已结束任务。
        if (receiveSequence <= state.taskSnapshotSequence && !isTerminalTaskState(task.state)) return;
        if ((existing.__streamSequence || 0) > receiveSequence) return;
        if (isTerminalTaskState(existing.state || existing.status) && !isTerminalTaskState(task.state)) return;
        if (taskUpdatedMs(task) && taskUpdatedMs(existing) > taskUpdatedMs(task)) return;
        // 任务 3.2.3：终态时强制 progress=1.0（如果 state 是 SUCCEEDED 且 payload.progress 为 null/undefined）
        if (isTerminalTaskState(task.state) && task.state === "succeeded" && task.progress === undefined) {
          task.progress = 1.0;
        }
        if (isTerminalTaskState(task.state)) {
          state.taskActiveById.delete(key);
          // 任务 3.3：history 幂等守卫 —— 已存在且更新版本不晚于新值时跳过覆盖
          const existingHistory = state.taskHistoryById.get(key);
          if (!existingHistory || taskUpdatedMs(task) > taskUpdatedMs(existingHistory)) {
            state.taskHistoryById.set(key, task);
          }
        } else {
          state.taskHistoryById.delete(key);
          state.taskActiveById.set(key, task);
        }
        refreshTaskStore();
      }
      function updateStreamOverview(frame) {
        const keyByTopic = {
          "robot.state": "state",
          "map.frame": "map",
          "map.heartbeat": "map",
          "localization.pose": "localization",
          "navigation.state": "navigation",
          "exploration.state": "exploration",
          "runtime.governance": "runtime_governance",
          "pointcloud_map.meta": "pointcloud_map",
          "pointcloud.frame": "pointcloud",
          "video.frame": "video",
          "logs.entry": "logs",
        };
        const key = keyByTopic[frame.topic];
        if (!key) return;
        const payload = frame.payload || {};
        const pointcloudMeta = { source: frame.source, point_count: payload.point_count ?? (frame.metadata && frame.metadata.point_count) };
        const metadata = frame.topic === "pointcloud_map.meta"
          ? { ...pointcloudMeta, source_kind: payload.source_kind || (frame.metadata && frame.metadata.source_kind), pcd_path: payload.pcd_path || (frame.metadata && frame.metadata.pcd_path) }
          : frame.topic === "pointcloud.frame" ? pointcloudMeta
          : frame.topic === "map.heartbeat" ? { source: frame.source, event_type: payload.event_type || (frame.metadata && frame.metadata.event_type), message_count: payload.message_count, update_sequence: payload.update_sequence }
          : { source: frame.source };
        const overview = { ...(state.latestOverview || {}) };
        overview[key] = {
          status: frame.status || "available",
          source_timestamp: frame.timestamp,
          latency_ms: 0,
          stale: frame.status === "stale",
          frame_id: frame.frame_id,
          revision: frame.revision,
          metadata,
        };
        state.latestOverview = overview;
        state.overviewReceivedAt = new Date().toISOString();
        scheduleOverviewRender(false);
      }
      function scheduleOverviewRender(immediate) {
        if (!state.latestOverview) return;
        const render = () => {
          state.overviewRenderTimer = null;
          state.overviewLastRenderAt = Date.now();
          updateOverview({ data: state.latestOverview, __receivedAt: state.overviewReceivedAt || new Date().toISOString(), __frontendLatencyMs: 0 });
        };
        if (immediate) {
          if (state.overviewRenderTimer) window.clearTimeout(state.overviewRenderTimer);
          render();
          return;
        }
        if (state.overviewRenderTimer) return;
        const elapsed = Date.now() - state.overviewLastRenderAt;
        state.overviewRenderTimer = window.setTimeout(render, Math.max(0, OVERVIEW_RENDER_INTERVAL_MS - elapsed));
      }
      buildOverview();
      updateClock();
      loadInitialHealth();
      loadInitialTasks();
      // 任务 4.0：已删除 startTaskPolling 调用 —— tasks.state WebSocket 流为单一来源
      syncMapCanvasSize();
      if (window.ResizeObserver) {
        new ResizeObserver(redrawMapAfterResize).observe(els.mapCanvas);
      } else {
        window.addEventListener("resize", redrawMapAfterResize);
      }
      window.setInterval(updateClock, 1000);
      els.showDetections.addEventListener("change", () => {
        state.showDetections = Boolean(els.showDetections.checked);
        scheduleVideoDetectionDraw();
      });
      els.videoFrame.addEventListener("load", () => {
        const pending = state.videoPendingSingleFrame;
        if (!pending || pending.generation !== state.videoStreamGeneration || els.videoFrame.getAttribute("src") !== pending.src) return;
        state.videoPendingSingleFrame = null;
        state.videoFrames += 1;
        state.videoLastFrameAt = Date.now();
        state.videoDisplayedIdentity = { ...pending.identity, receivedAt: state.videoLastFrameAt };
        recordVideoFrameMetric();
        state.videoSourceSize = { width: els.videoFrame.naturalWidth, height: els.videoFrame.naturalHeight };
        state.videoStreamStatus = "rendering";
        els.videoFrame.hidden = false;
        renderVideoStreamSummary();
        scheduleVideoDetectionDraw();
      });
      els.videoFrame.addEventListener("error", () => {
        if (!state.videoPendingSingleFrame) return;
        state.videoPendingSingleFrame = null;
        state.videoDecodeErrors += 1;
        state.videoLastError = "浏览器无法显示图像。";
        state.videoDisplayedIdentity = null;
        state.videoLastFrameAt = null;
        els.videoFrame.hidden = true;
        setVideoStreamStatus("error", "浏览器无法显示收到的图像，请检查图像编码与来源。");
      });
      const resizeVideoOverlay = () => {
        state.videoDetectionLayoutRevision += 1;
        scheduleVideoDetectionDraw();
      };
      if (window.ResizeObserver) new ResizeObserver(resizeVideoOverlay).observe(els.videoFrame.parentElement);
      else window.addEventListener("resize", resizeVideoOverlay);
      window.setInterval(() => {
        if (!videoPanelVisible()) return;
        updateMapStreamChip();
        renderVideoStreamSummary();
        scheduleVideoDetectionDraw();
      }, 500);
      document.addEventListener("visibilitychange", () => {
        if (!document.hidden) { renderVideoStreamSummary(); scheduleVideoDetectionDraw(); }
      });
      document.getElementById("viewSwitchMonitor")?.addEventListener("click", resizeVideoOverlay);

      startVideoStream();
      function setMapZoom(nextZoom) {
        const zoom = Math.min(8, Math.max(0.25, nextZoom));
        if (usesRvizMapView() || isMapToolActive()) {
          activeMapWorldView().zoom = zoom;
        } else {
          state.mapZoom = zoom;
        }
        requestMapDraw(false);
      }
      els.mapZoomIn.addEventListener("click", () => setMapZoom(mapViewZoom() * 1.25));
      els.mapZoomOut.addEventListener("click", () => setMapZoom(mapViewZoom() / 1.25));
      els.mapZoomReset.addEventListener("click", () => {
        if (state.map2dCameraMode === "free") resetFreeMapView();
        else if (isMapToolActive()) resetRvizMapView();
        else setMapZoom(1);
        requestMapDraw(false);
      });
      els.map2dCameraToggle?.addEventListener("click", () => {
        setMap2dCameraMode(state.map2dCameraMode === "free" ? "follow" : "free");
      });
      if (els.mapCenterRobot) {
        els.mapCenterRobot.addEventListener("click", () => centerRvizMapOnRobot());
      }
      function isMap3dVisible() {
        const container = document.getElementById("map3dContainer");
        return Boolean(container && container.style.display !== "none");
      }
      function finishGoalPick(event) {
        if (!state.goalPickDrag) return;
        if (state.pendingNavGoal && state.pendingNavGoal.status === "sending") {
          state.goalPickDrag = null;
          return;
        }
        const drag = state.goalPickDrag;
        state.goalPickDrag = null;
        const yaw = goalPickYawFromDrag(drag);
        void dispatchNavGoal(drag.start, yaw);
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
        requestMapDraw(false);
      }
      function disableMapTools(clearStatus) {
        state.goalPickMode = false;
        state.publishPointMode = false;
        state.initialPosePickMode = false;
        state.goalPickDrag = null;
        state.initialPosePickDrag = null;
        state.publishPointClick = null;
        state.mapPanDrag = null;
        syncMapToolUi();
        if (clearStatus) updateGoalPickStatus("", "");
        requestMapDraw(false);
      }
      function finishInitialPosePick(event) {
        if (!state.initialPosePickDrag) return;
        const drag = state.initialPosePickDrag;
        state.initialPosePickDrag = null;
        const yaw = goalPickYawFromDrag(drag);
        void dispatchInitialPose(drag.start, yaw);
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
        requestMapDraw(false);
      }
      if (els.mapInitialPoseToggle) {
        els.mapInitialPoseToggle.addEventListener("click", () => {
          const next = !state.initialPosePickMode;
          disableMapTools(false);
          state.initialPosePickMode = next;
          syncMapToolUi();
          if (state.initialPosePickMode) {
            if (state.map2dCameraMode !== "free") resetRvizMapView();
            reevaluateCachedMapDisplay();
            updateGoalPickStatus(
              state.mapGrid
                ? "2D Pose 模式：左键拖拽设初始位姿 · 中/右键平移 · 滚轮缩放"
                : "2D Pose 已开启，但地图未显示。请先 activate_map 并启动 go2.sh。",
              state.mapGrid ? "pending" : "error",
            );
          } else {
            updateGoalPickStatus("", "");
          }
          requestMapDraw(false);
        });
      }
      if (els.mapGoalToggle) {
        els.mapGoalToggle.addEventListener("click", () => {
          const next = !state.goalPickMode;
          disableMapTools(false);
          state.goalPickMode = next;
          syncMapToolUi();
          if (state.goalPickMode) {
            if (state.map2dCameraMode !== "free") resetRvizMapView();
            reevaluateCachedMapDisplay();
            updateGoalPickStatus(
              state.mapGrid
                ? "RViz 模式：左键设目标 · 中/右键平移 · 滚轮缩放"
                : "2D Goal 已开启，但地图未显示。请先 activate_map 并启动 go2.sh（mapping / explore / localization 均可）。",
              state.mapGrid ? "pending" : "error",
            );
          } else {
            updateGoalPickStatus("", "");
          }
          requestMapDraw(false);
        });
      }
      if (els.mapPublishPointToggle) {
        els.mapPublishPointToggle.addEventListener("click", () => {
          const next = !state.publishPointMode;
          disableMapTools(false);
          state.publishPointMode = next;
          syncMapToolUi();
          if (state.publishPointMode) {
            if (state.map2dCameraMode !== "free") resetRvizMapView();
            reevaluateCachedMapDisplay();
            updateGoalPickStatus(
              state.mapGrid
                ? "选点模式：左键发布 /clicked_point（不导航）"
                : "选点已开启，但地图未显示。请先 activate_map 并启动 go2.sh。",
              state.mapGrid ? "pending" : "error",
            );
          } else {
            updateGoalPickStatus("", "");
          }
          requestMapDraw(false);
        });
      }
      function finishMapPan(event) {
        if (!state.mapPanDrag) return;
        state.mapPanDrag = null;
        syncGoalPickUi();
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
      }
      function finishPublishPoint(event) {
        if (!state.publishPointClick) return;
        const click = state.publishPointClick;
        state.publishPointClick = null;
        if (event && event.pointerId !== undefined && els.mapCanvas.releasePointerCapture) {
          try { els.mapCanvas.releasePointerCapture(event.pointerId); } catch (_) {}
        }
        void dispatchClickedPoint(click.mapPoint);
      }
      els.mapCanvas.addEventListener("contextmenu", (event) => {
        if (isMapToolActive() || state.map2dCameraMode === "free") event.preventDefault();
      });
      els.mapCanvas.addEventListener("pointerdown", (event) => {
        // V2 boundary/navigation 模式：把事件让给 SVG overlay，canvas 主循环不参与。
        if (window.__nuwaxMapToolV2 && window.__nuwaxMapToolV2.active()) return;
        if (!isMap3dVisible() && event.button === 0 && !isMapToolActive()) {
          const pointer = canvasPointer(event);
          const hit = hitTestSemanticMarkers(pointer.x, pointer.y);
          if (hit) {
            event.preventDefault();
            if (hit.type === "cluster") showSemanticClusterList(hit.cluster);
            else showSemanticInstanceDetail(hit.instance);
            return;
          }
        }
        if ((!isMapToolActive() && state.map2dCameraMode !== "free") || isMap3dVisible()) return;
        const pointer = canvasPointer(event);
        if (event.button === 1 || event.button === 2 || (event.button === 0 && state.map2dCameraMode === "free" && !isMapToolActive())) {
          event.preventDefault();
          state.mapPanDrag = {
            pointerStart: pointer,
            centerStart: getMapRvizViewCenter(),
          };
          syncMapToolUi();
          if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
          return;
        }
        if (event.button !== 0) return;
        event.preventDefault();
        const anchor = cameraAnchor();
        const mapPoint = canvasToMap(pointer.x, pointer.y, anchor);
        if (state.publishPointMode) {
          state.publishPointClick = { pointer, anchor, mapPoint };
          if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
          return;
        }
        if (state.initialPosePickMode) {
          state.initialPosePickDrag = {
            pointerStart: pointer,
            pointerCurrent: pointer,
            anchor,
            start: mapPoint,
            current: mapPoint,
          };
          if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
          requestMapDraw(false);
          return;
        }
        state.goalPickDrag = {
          pointerStart: pointer,
          pointerCurrent: pointer,
          anchor,
          start: mapPoint,
          current: mapPoint,
        };
        if (els.mapCanvas.setPointerCapture) els.mapCanvas.setPointerCapture(event.pointerId);
        requestMapDraw(false);
      });
      els.mapCanvas.addEventListener("pointermove", (event) => {
        if (window.__nuwaxMapToolV2 && window.__nuwaxMapToolV2.active()) return;
        if (state.mapPanDrag) {
          const pointer = canvasPointer(event);
          const anchor = cameraAnchor();
          const scale = cameraScale(anchor);
          if (!Number.isFinite(scale) || scale <= 0) return;
          const dx = pointer.x - state.mapPanDrag.pointerStart.x;
          const dy = pointer.y - state.mapPanDrag.pointerStart.y;
          activeMapWorldView().center = {
            x: state.mapPanDrag.centerStart.x - dx / scale,
            y: state.mapPanDrag.centerStart.y + dy / scale,
          };
          requestMapDraw(false);
          return;
        }
        if (state.initialPosePickDrag) {
          const pointer = canvasPointer(event);
          state.initialPosePickDrag.pointerCurrent = pointer;
          state.initialPosePickDrag.current = canvasToMap(pointer.x, pointer.y, state.initialPosePickDrag.anchor);
          requestMapDraw(false);
          return;
        }
        if (!state.goalPickDrag) return;
        const pointer = canvasPointer(event);
        state.goalPickDrag.pointerCurrent = pointer;
        state.goalPickDrag.current = canvasToMap(pointer.x, pointer.y, state.goalPickDrag.anchor);
        requestMapDraw(false);
      });
      els.mapCanvas.addEventListener("pointerup", (event) => {
        if (window.__nuwaxMapToolV2 && window.__nuwaxMapToolV2.active()) return;
        if (state.mapPanDrag) {
          finishMapPan(event);
          return;
        }
        if (state.publishPointClick) {
          finishPublishPoint(event);
          return;
        }
        if (state.initialPosePickDrag) {
          finishInitialPosePick(event);
          return;
        }
        finishGoalPick(event);
      });
      els.mapCanvas.addEventListener("pointercancel", (event) => {
        if (window.__nuwaxMapToolV2 && window.__nuwaxMapToolV2.active()) return;
        if (state.mapPanDrag) {
          finishMapPan(event);
          return;
        }
        if (state.publishPointClick) {
          finishPublishPoint(event);
          return;
        }
        if (state.initialPosePickDrag) {
          finishInitialPosePick(event);
          return;
        }
        finishGoalPick(event);
      });
      els.mapCanvas.addEventListener("wheel", (event) => {
        event.preventDefault();
        if (isMapToolActive() || state.map2dCameraMode === "free") {
          const pointer = canvasPointer(event);
          zoomRvizMapAt(pointer, event.deltaY < 0 ? 1.12 : 1 / 1.12);
          requestMapDraw(false);
          return;
        }
        setMapZoom(state.mapZoom * (event.deltaY < 0 ? 1.12 : 1 / 1.12));
      }, { passive: false });
      // V2 navigation-and-exploration-boundary §9.3 全局钩子：
      // boundary_draw.js / navigation.js 拆模块需要从屏幕坐标拿世界坐标，
      // canvasToMap / canvasPointer / cameraAnchor 是 IIFE 闭包私有的，必须挂 window。
      window.__nuwaxMapUnproject = function (screenX, screenY) {
        if (!els.mapCanvas || !state.mapGrid || isMap3dVisible()) return null;
        const fakeEv = { clientX: Number(screenX), clientY: Number(screenY) };
        const pointer = canvasPointer(fakeEv);
        const anchor = cameraAnchor();
        const world = canvasToMap(pointer.x, pointer.y, anchor);
        if (!world) return null;
        return { x: world.x, y: world.y, frame_id: navigationFrameId() };
      };
      window.__nuwaxResolveNavigationHeight = resolveNavigationHeight;
      window.__nuwaxDisableMapTools = disableMapTools;
      // 选点回调：navigation.js 会在启动时 monkey-patch 此函数缓存点击坐标。
      // 这里默认 no-op；publish-point 走的是 isMapToolActive() 的现有 publishPoint 流程。
      window.__nuwaxMapClick = window.__nuwaxMapClick || null;
      // V2 模块互斥开关：boundary_draw.js / navigation.js 在激活时会置 true，
      // 主循环的 pointerdown/pointermove/pointerup 入口据此提前 return 让事件落到 SVG overlay。
      window.__nuwaxMapToolV2 = {
        active: function () { return Boolean(window.__nuwaxBoundaryActive || window.__nuwaxNavigationActive); },
      };
      // V2 boundary 绘制坐标系：返回当前相机可见范围的世界米 viewBox。
      // boundary_draw.js 每帧调一次同步 SVG viewBox，让线段在世界坐标系下跟随 pose/zoom 移动。
      // 返回 null 表示 canvas 还没准备好（首次加载或 grid 缺失）——boundary 应降级为不绘制而不是报错。
      window.__nuwaxMapWorldViewport = function () {
        if (!els.mapCanvas) return null;
        const canvas = els.mapCanvas;
        const anchor = cameraAnchor();
        const scale = cameraScale(anchor);
        if (!Number.isFinite(scale) || scale <= 0) return null;
        const centerWorld = anchor.point || { x: 0, y: 0 };
        return {
          cx: Number(centerWorld.x) || 0,
          cy: Number(centerWorld.y) || 0,
          halfX: canvas.width / 2 / scale,   // 世界米
          halfY: canvas.height / 2 / scale,  // 世界米
          scale: scale,
          yaw: anchor.yaw || 0,
          mapFixed: Boolean(anchor.mapFixed),
          // canvas pixel 尺寸（用于 SVG 元素 width/height 属性）
          pixelWidth: canvas.width,
          pixelHeight: canvas.height,
        };
      };
      // 禁行区使用与 canvas 完全相同的 world -> pixel 投影，包含非零地图中心、
      // robot-locked 旋转视角、RViz 固定视角和 CSS 缩放。
      window.__nuwaxMapProject = function (worldX, worldY) {
        if (!els.mapCanvas) return null;
        const anchor = cameraAnchor();
        const scale = cameraScale(anchor);
        const projected = mapToCanvas({ x: Number(worldX), y: Number(worldY) }, anchor, scale);
        if (!projected || !Number.isFinite(projected.x) || !Number.isFinite(projected.y)) return null;
        return {
          x: projected.x,
          y: projected.y,
          scale,
          pixelWidth: els.mapCanvas.width,
          pixelHeight: els.mapCanvas.height,
          mapFixed: Boolean(anchor.mapFixed),
          yaw: Number(anchor.yaw) || 0,
        };
      };
      window.__nuwaxMapViewMode = function () { return state.mapViewMode; };
      window.__nuwaxMapRequestDraw = function () { requestMapDraw(false); };
      // V2 boundary tick：drawMap() 每帧末尾调用，boundary_draw.js 注册回调同步 viewBox。
      // 解耦：boundary 模块自己挂回调，不在主循环 import boundary 模块。
      window.__nuwaxMapDrawTick = window.__nuwaxMapDrawTick || null;
      // V2 boundary freeze：boundary_draw.js 在 setDrawing(true) 时调一次取当前 anchor
      // 作冻结中心；setDrawing(false) 时清掉。cameraAnchor() 入口的 __nuwaxMapFreeze
      // 短路优先于 usesRvizMapView() 与 pose-locked 路径。
      window.__nuwaxCameraAnchor = function () {
        return cameraAnchor();
      };
      function syncMapViewControls() {
        const is3d = state.mapViewMode === "3d";
        if (els.mapCanvas) els.mapCanvas.style.display = is3d ? "none" : "";
        if (els.map3dContainer) els.map3dContainer.style.display = is3d ? "" : "none";
        if (els.map2dCameraControls) els.map2dCameraControls.hidden = is3d;
        if (els.map2dActionControls) els.map2dActionControls.hidden = is3d;
        if (els.map3dCameraControls) els.map3dCameraControls.hidden = !is3d;
        if (els.mapModeToggle) {
          els.mapModeToggle.textContent = is3d ? "切换 2D" : "切换 3D";
          els.mapModeToggle.dataset.mapView = is3d ? "3d" : "2d";
          els.mapModeToggle.setAttribute("aria-pressed", is3d ? "true" : "false");
        }
        if (els.mapEmpty && is3d) els.mapEmpty.hidden = true;
        renderMapSummary();
      }
      function updatePointcloud3dCameraControls(detail) {
        const cameraState = detail || {};
        const mode = cameraState.mode === "follow" ? "follow" : "free";
        const robotAvailable = cameraState.robotAvailable === true || Boolean(pointFromPose(state.latestPose));
        const distance = Number(cameraState.distance);
        const zoomPercent = Number(cameraState.zoomPercent);
        state.pointcloud3dCameraMode = mode;
        state.pointcloud3dDistance = Number.isFinite(distance) ? distance : null;
        if (Number.isFinite(zoomPercent) && zoomPercent > 0) {
          state.pointcloud3dZoomPercent = zoomPercent;
        }
        if (els.map3dFreeView) {
          els.map3dFreeView.classList.toggle("active", mode === "free");
          els.map3dFreeView.setAttribute("aria-pressed", mode === "free" ? "true" : "false");
        }
        if (els.map3dFollowView) {
          els.map3dFollowView.classList.toggle("active", mode === "follow");
          els.map3dFollowView.setAttribute("aria-pressed", mode === "follow" ? "true" : "false");
          els.map3dFollowView.disabled = !robotAvailable;
          els.map3dFollowView.title = robotAvailable
            ? "相机轨道中心跟随机器人"
            : "等待定位位姿后可跟随机器人";
        }
        if (els.map3dCenterRobot) {
          els.map3dCenterRobot.disabled = !robotAvailable;
          els.map3dCenterRobot.title = robotAvailable
            ? "将 3D 视角定位到机器人"
            : "等待定位位姿后可定位机器人";
        }
        if (els.map3dZoomValue) {
          els.map3dZoomValue.textContent = `${Math.max(1, Math.round(state.pointcloud3dZoomPercent))}%`;
        }
      }
      function setMapViewMode(mode) {
        const nextMode = mode === "3d" ? "3d" : "2d";
        state.mapViewMode = nextMode;
        updateMapStreamChip();
        if (nextMode === "3d") disableMapTools(true);
        syncMapViewControls();
        window.pointcloud3d?.setVisible(nextMode === "3d");
        if (nextMode === "3d") {
          if (!window.pointcloud3d || !els.map3dContainer) {
            updateMapStreamChip();
            return;
          }
          window.pointcloud3d.init(els.map3dContainer);
          window.pointcloud3d.setCameraMode(state.pointcloud3dCameraMode);
          syncPointcloud3dState();
          window.requestAnimationFrame(() => window.pointcloud3d?.resize());
        } else {
          reevaluateCachedMapDisplay();
          requestMapDraw(false);
        }
      }
      window.addEventListener("pointcloud3d:map-frame", (event) => {
        const frame = event.detail;
        if (!frame) return;
        recordMediaFrame("map3d", frame.source === "saved" ? null : frame.sourceTime, frame.frameKey, frame.sourceKey);
        updateMapStreamChip();
      });
      window.addEventListener("pointcloud3d:ready", () => {
        state.pointcloud3dReady = true;
        window.pointcloud3d?.setCameraMode(state.pointcloud3dCameraMode);
        window.pointcloud3d?.setPointStyle(els.map3dPointStyle?.value || "height");
        syncPointcloud3dState();
      });
      window.addEventListener("pointcloud3d:semantic-select", (event) => {
        const detail = event.detail || {};
        const instances = Array.isArray(detail.instances)
          ? detail.instances
          : detail.instance ? [detail.instance] : [];
        if (!instances.length) return;
        if (instances.length > 1) showSemanticClusterList({ members: instances });
        else showSemanticInstanceDetail(instances[0]);
      });
      window.addEventListener("pointcloud3d:camera-state", (event) => {
        updatePointcloud3dCameraControls(event.detail || {});
      });
      window.addEventListener("pointcloud3d:style-state", (event) => {
        const style = String(event.detail?.style || "height");
        if (!els.map3dPointStyle) return;
        els.map3dPointStyle.value = style;
        els.map3dPointStyle.closest(".map-point-style")?.setAttribute("data-point-style", style);
      });
      window.addEventListener("pointcloud3d:point-count", (event) => {
        const detail = event.detail || {};
        const displayedPoints = Number(detail.displayedPoints ?? detail.points);
        const sourcePoints = Number(detail.sourcePoints ?? detail.points);
        const hasDisplayedPoints = Number.isFinite(displayedPoints);
        const hasSourcePoints = Number.isFinite(sourcePoints);
        state.pointcloud3dDisplayedPoints = hasDisplayedPoints ? Math.max(0, Math.floor(displayedPoints)) : 0;
        state.pointcloud3dSourcePoints = hasSourcePoints
          ? Math.max(0, Math.floor(sourcePoints))
          : state.pointcloud3dDisplayedPoints;
        state.pointcloud3dPointCountKnown = hasDisplayedPoints || hasSourcePoints;
        renderMapSummary();
      });
      if (els.showSemanticInstances) {
        els.showSemanticInstances.addEventListener("change", () => {
          state.showSemanticInstances = Boolean(els.showSemanticInstances.checked);
          window.dispatchEvent(new CustomEvent("pointcloud3d:semantic-visibility", {
            detail: { visible: state.showSemanticInstances },
          }));
          requestMapDraw(false);
        });
      }
      if (els.semanticInstancesRefresh) {
        els.semanticInstancesRefresh.addEventListener("click", () => loadSemanticInstances());
      }
      els.semanticMemoryToggle?.addEventListener("click", () => toggleSemanticMemoryPanel(els.semanticMemoryPanel?.hidden));
      els.semanticMemoryClose?.addEventListener("click", () => toggleSemanticMemoryPanel(false));
      document.querySelectorAll('[data-view-target]:not([data-view-target="monitor-view"])').forEach((button) => {
        button.addEventListener("click", () => toggleSemanticMemoryPanel(false));
      });
      els.semanticMemorySearch?.addEventListener("input", renderSemanticMemoryList);
      els.semanticMemoryFilter?.addEventListener("change", renderSemanticMemoryList);
      state._semanticPollTimer = window.setInterval(() => {
        if (!document.hidden && (state.showSemanticInstances || els.semanticMemoryPanel?.hidden === false)) loadSemanticInstances();
      }, 30000);
      document.addEventListener("visibilitychange", () => {
        if (!document.hidden && Date.now() - state.semanticFetchedAt >= 30000) loadSemanticInstances();
      });
      window.addEventListener("stream:map.frame", () => {
        const mapName = activeMapName();
        if (state.semanticLastMapName !== mapName) {
          state.semanticLastMapName = mapName;
          reconcileSemanticOverlay(state.semanticMeta.library_name);
          if (!document.hidden) loadSemanticInstances();
        }
        publishSemanticMapInstances();
        renderSemanticMemoryStatus();
      });
      loadSemanticInstances();
      connectStreams();
      connectStreams(state.poseStream);
      els.mapModeToggle?.addEventListener("click", () => {
        setMapViewMode(state.mapViewMode === "3d" ? "2d" : "3d");
      });
      els.map3dFreeView?.addEventListener("click", () => {
        state.pointcloud3dCameraMode = "free";
        window.pointcloud3d?.setCameraMode("free");
      });
      els.map3dFollowView?.addEventListener("click", () => {
        if (els.map3dFollowView.disabled) return;
        state.pointcloud3dCameraMode = "follow";
        window.pointcloud3d?.setCameraMode("follow");
      });
      els.map3dPointStyle?.addEventListener("change", () => {
        const style = els.map3dPointStyle.value;
        els.map3dPointStyle.closest(".map-point-style")?.setAttribute("data-point-style", style);
        window.pointcloud3d?.setPointStyle(style);
      });
      els.map3dZoomOut?.addEventListener("click", () => window.pointcloud3d?.zoomBy(1.25));
      els.map3dZoomIn?.addEventListener("click", () => window.pointcloud3d?.zoomBy(0.8));
      els.map3dResetView?.addEventListener("click", () => window.pointcloud3d?.resetView());
      els.map3dCenterRobot?.addEventListener("click", () => window.pointcloud3d?.centerOnRobot());
      syncMapViewControls();
      syncMap2dCameraControls();
      updatePointcloud3dCameraControls({ mode: "free", robotAvailable: Boolean(pointFromPose(state.latestPose)) });
      updateMapPoseHud();
    })();
