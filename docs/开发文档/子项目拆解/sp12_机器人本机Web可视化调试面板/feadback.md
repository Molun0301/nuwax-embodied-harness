# feadback：sp12 机器人本机 Web 可视化调试面板

## 1. 用户修正与对齐点

### 2026-04-27

1. 本任务作为 sp12，不放到“补丁文档”。
2. 目录必须是：`docs/开发文档/子项目拆解/sp12_机器人本机Web可视化调试面板/`。
3. 需要创建或更新中文开发文档：`PDR.md`、`todolist.md`、`feadback.md`、`数据可用性矩阵.md`、`接口契约草案.md`、`真机验收清单.md`。
4. MVP（最小可行产品）是机器人本机只读 Web（网页）调试面板。
5. 面板覆盖服务状态、日志、2D SLAM（同步定位与建图）栅格、机器人位姿/轨迹、frontier（探索前沿）高亮、已探索/未知/障碍配色、实时 3D（三维）点云。
6. 不做控制命令。
7. 必须先做真机数据可用性矩阵，再做 UI（用户界面）。
8. 地图/frontier 的 P95（第 95 百分位延迟）目标是小于 1 秒。
9. 点云因为本机运行，不预设强制限频、强制降采样、强制最大点数。
10. 文档可以保留“可选保护配置与性能观测”，但不能写成必须限制点云。
11. 不创建公开文档。
12. 不修改业务代码。

## 2. 设计口径

1. sp12 是调试观察面，不是控制面。
2. sp12 复用平台状态、日志、地图、定位、探索、点云数据面，不新建平行能力主链。
3. Go2 端侧数据面只作为真机输入来源之一，不重新承担平台 geometry（几何）主链职责。
4. UI 中所有不可用数据必须显示明确空态、错误原因或最近更新时间，不能伪造数据。
5. 点云优先真实展示本机数据压力，后续根据观测结果决定是否启用可选保护配置。

## 3. 待后续确认问题

1. 日志面板读取配置日志文件 `runtime.config.logging.log_file`，使用 byte offset cursor（字节偏移游标）增量 tail，并支持 `level`、`source` 过滤。
2. 实时更新采用同源 GET（读取）轮询；当前页面轮询间隔为 500 ms，覆盖 overview、地图、定位、导航、frontier、点云、视频、日志。
3. 3D 点云前端采用内联 canvas（画布）俯视投影渲染，颜色表达 Z（高度）差异；后续真机验收再补充坐标轴规范。
4. frontier 数据由 ExplorationRuntime 只读暴露 `latest_frontiers` 和 `current_frontier`，debug 聚合层不重算 frontier。
5. 面板默认复用同源访问和网关认证；`access_token` URL 参数仅用于局域网直连或手动覆盖。

## 4. 不允许漂移的边界

1. 不加入机器人运动控制按钮。
2. 不加入导航目标下发。
3. 不加入探索启动/停止。
4. 不加入地图保存、删除、重定位等修改型动作。
5. 不把点云保护配置写成 MVP 必须条件。
6. 不把该任务移动到补丁文档目录。

## 5. 2026-04-27 后端最小闭环实现记录

1. 新增 `contracts/debug_views.py`，定义 `DebugEnvelope`、`DebugDataStatus`、`DebugError`、概览/地图/定位/导航/点云/日志视图模型。
2. 新增 `gateways/http/debug_panel.py`，提供 `/debug` HTML（超文本标记语言）骨架和 `/api/debug/overview`、`/api/debug/map`、`/api/debug/localization`、`/api/debug/navigation`、`/api/debug/pointcloud/latest`、`/api/debug/logs/tail`。
3. 修改 `gateways/app.py` 挂载 debug router（路由器），根路由能力清单新增 `debug_panel_path=/debug`。
4. 修改 `gateways/http/__init__.py` 与 `contracts/__init__.py` 导出 debug 能力。
5. 地图、定位、导航读取均复用现有 service/runtime（服务/运行时）快照；frontier 几何由 ExplorationRuntime 只读暴露 `latest_frontiers` / `current_frontier` 元数据，不在 sp12 中重算。
6. 日志 tail 读取配置日志文件 `runtime.config.logging.log_file`，只接收 cursor、limit、level、source，不允许路径参数。
7. 点云读取优先 provider（提供器）和 Go2 data plane（数据面）只读 getter，不传 `max_points`，不默认降采样或限频。
8. 当前开发机不在 Go2 真机上，真机点云、地图、frontier 延迟和浏览器渲染仍需后续验证 worker 在真机完成。
9. 静态验证结果：`python3 -m py_compile contracts/debug_views.py gateways/http/debug_panel.py gateways/app.py gateways/http/__init__.py` 通过；`gateways/http/debug_panel.py` 未发现 POST/PATCH/DELETE/PUT 或控制方法调用。


## 5. 后端只读聚合实现记录

### 2026-04-27

1. 后端新增统一 debug envelope（调试信封），字段包括 `source`、`frame_id`、`source_timestamp`、`gateway_timestamp`、`sequence`、`revision`、`frequency_hz`、`latency_ms`、`stale`、`status`、`data`、`error`。
2. `/debug` 返回内联 HTML（超文本标记语言）骨架，只展示只读接口入口，不包含控制按钮。
3. `/api/debug/overview` 和 `/api/debug/snapshot` 聚合 health、state、localization、map、navigation、exploration、pointcloud、video、logs 的状态摘要。
4. `/api/debug/map` 和 `/api/debug/map/frame/latest` 只读取 `MappingService` 最新快照；没有占据栅格时返回 `missing/source_unavailable`。
5. `/api/debug/localization`、`/api/debug/localization/pose/latest`、`/api/debug/localization/trajectory` 只读取 `LocalizationService` 快照和历史缓存；没有 pose 时返回 `missing/source_unavailable`。
6. `/api/debug/navigation` 只读取 `NavigationService` 导航/探索上下文；`/api/debug/exploration/frontier/latest` 返回 ExplorationRuntime 已暴露的 frontier 几何，未生成候选时返回明确缺失原因，不在 sp12 里重算 frontier。
7. `/api/debug/pointcloud/latest` 和 `/api/debug/pointcloud/frame/latest/meta` 优先读取 provider 或 Go2 data_plane 的最新只读点云；接口不传默认 `max_points`，返回真实 `points` 供页面实时渲染，并在 `protection.default_max_points_applied_by_debug_api=false` 中显式说明调试接口未默认限点。
8. `/api/debug/video/latest` 和 `/api/debug/video/frame/latest` 通过 `ImageProvider.capture_image()` 获取只读视频帧，返回 data URI（数据统一资源标识符）或引用给页面实时展示。
9. `/api/debug/logs/tail` 读取配置日志文件，使用 byte offset cursor（字节偏移游标）/limit/level/source 过滤，不接收任意文件路径。
10. 所有 sp12 debug 路由均为 GET；未新增 POST/PATCH/DELETE 控制接口。
11. 当前开发机不在 Go2 真机上，地图/frontier P95、真机点云压力、视频帧率和浏览器渲染仍需后续真机验收采样。

验证：`python3 -m py_compile` 已通过。`pytest` 目标用例因当前环境缺少 `fastapi` 包在收集阶段失败，需在依赖完整环境重跑。


## 6. 前端只读 Web 调试面板实现记录

### 2026-04-27

1. `/debug` 已实现内联 HTML/CSS/JS（超文本标记语言/层叠样式表/脚本）调试面板，不依赖外网 CDN（内容分发网络）或前端构建链路。
2. 页面约 500 ms 轮询 `/api/debug/overview`、`/api/debug/map`、`/api/debug/localization`、`/api/debug/navigation`、`/api/debug/exploration/frontier/latest`、`/api/debug/pointcloud/latest`、`/api/debug/video/latest`、`/api/debug/logs/tail`。
3. 顶部总览展示 health/state/localization/map/navigation/exploration/pointcloud/video/logs 的状态、来源时间戳、延迟、stale（过期）和前端接收时间。
4. 2D 地图 canvas（画布）在存在 `occupancy_grid` 时按 unknown/free/occupied 三色绘制；缺失、过期或错误时展示明确原因，不使用假地图。
5. 定位面板展示 pose（位姿）、frame（坐标系）、yaw（偏航角）和 trajectory_count（轨迹点数）；地图可用时叠加机器人箭头与真实轨迹。
6. navigation/frontier 面板展示 navigation/exploration 摘要、frontier_count 和 waiting_for；后端返回 frontier 几何时叠加高亮，未生成候选时显示明确缺失原因。
7. 点云面板不引入第三方渲染库；若后端返回内联 points 则做本地 canvas 简单投影，否则只展示 point_count、frame、payload、artifact/data_uri、metrics 和 source_unavailable 状态，不伪造点。
8. 日志面板展示配置日志文件 tail，使用 byte offset cursor（字节偏移游标）增量读取，不提供路径输入。
9. 页面不包含 `<button>`、`<form>`，脚本只使用 GET fetch（读取请求），未新增 POST/PATCH/DELETE/PUT 控制调用。

验证：`python3 -m py_compile gateways/http/debug_panel.py` 通过；提取内联 JS 后 `node --check` 通过；静态检查未发现控制型 DOM（文档对象模型）元素或非 GET 请求。真机浏览器渲染、地图/frontier P95 和点云性能样本仍待后续验收。


目标集成测试验证：`python3 -m pytest tests/integration/test_gateway_integration.py::test_gateway_debug_panel_realtime_readonly_routes` 因当前环境缺少 `fastapi` 包在收集阶段失败，未进入用例执行。


## 6. Web 面板 UI 实现记录

### 2026-04-27

1. `/debug` 页面已实现为无构建链路的内联 HTML/CSS/JavaScript（超文本标记语言/层叠样式表/脚本）只读调试面板。
2. 页面仅发起同源 GET 请求：`/api/debug/overview`、`/api/debug/map`、`/api/debug/localization`、`/api/debug/navigation`、`/api/debug/exploration/frontier/latest`、`/api/debug/pointcloud/latest`、`/api/debug/video/latest`、`/api/debug/logs/tail`。
3. 顶部总览展示 health、state、localization、map、navigation、exploration、pointcloud、video、logs 九类状态卡片，并显示 source_timestamp、latency、stale、前端接收时间。
4. 地图 canvas 按 occupancy_grid 绘制 unknown/free/occupied 三色；定位可用时叠加机器人箭头和轨迹；frontier 几何可用时叠加点位。
5. navigation/frontier 面板展示 navigation/exploration 摘要、frontier_count、waiting_for；后端返回 `frontiers` 时在地图叠加高亮，未返回时明确显示缺失原因。
6. 点云面板不依赖外网 CDN（内容分发网络）或构建工具；仅当返回内联 `points` 时绘制本地 canvas 俯视投影，否则只展示 point_count、frame、metrics、data_uri/artifact_ref、source_unavailable 等元数据。
7. 日志区域使用 byte offset cursor（字节偏移游标）增量读取配置日志文件 tail；所有请求失败均显示 error/stale 空态，不伪造数据。
8. 页面未新增按钮、表单、POST/PATCH/DELETE 调用、运动/导航/探索控制。

验证：`python3 -m py_compile contracts/debug_views.py services/navigation/exploration_runtime.py gateways/http/debug_panel.py tests/integration/test_gateway_integration.py` 通过；抽取内联脚本 `node --check /tmp/sp12_debug_panel.js` 通过；静态只读校验未发现 `<button>`、POST/PATCH/DELETE/PUT、控制型 fetch 方法或 refresh 副作用。当前开发机仍无真机浏览器与真机数据，地图/frontier P95、真实点云流畅度和视频帧率需后续真机验收。

## 7. 复审问题修复记录

### 2026-04-27

1. `/api/debug/*` GET 路径已移除对 `mapping_service.refresh()`、`localization_service.refresh()`、`navigation_service.refresh_navigation()`、`navigation_service.refresh_exploration()` 的调用；调试面板只读取已有缓存，无缓存时返回 `missing`、`stale` 或 `unsupported`，避免由轮询推进平台运行态。
2. 点云 HTTP 调试接口返回真实 `points`，前端使用 canvas 俯视投影实时绘制，并保留 point_count、frame、fields、payload、artifact/data_uri、protection、metrics 等元数据与引用。
3. `/debug` 前端点云区域避免大数组 spread 等额外复制，按点数组循环计算边界与颜色，展示 `browser_render_ms` 观测值。
4. `/debug` 本机/SSH 隧道访问时应由网关从已加载 `.env` 的 `NUWAX_GATEWAY_AGENT_TOKENS` 自动注入 agent token 给页面内部只读 API 轮询；`access_token` URL 参数仅用于局域网直连或手动覆盖。
5. frontier 几何由 ExplorationRuntime 只读暴露；UI 在后端返回 `frontiers` 时叠加高亮，未生成候选时显示 `frontier_count`、`waiting_for` 和明确缺失原因。
6. 日志文件选择采用确定性优先级：优先 `runtime.config.logging.log_file` 主网关日志；仅当主日志不存在且 TTS（文本转语音）日志存在时回退到 `runtime.config.log_tts.log_path`；接口仍不接受路径参数。

## 8. 2026-04-28 用户反馈修正

1. 用户确认当前 Web 主 3D 点云展示不符合目标：它看起来是实时扫描帧，而目标是累计扫描形成的点云地图。
2. 代码阅读确认 `_get_latest_pointcloud()` 只读取 provider（提供器）或 Go2 data plane 的最新点云帧，没有读取 RTAB-Map 累计地图云、`MapVersion.geometry_asset.pcd_path` 或 `cloud_map.pcd`。
3. 后续主视图必须改为 `pointcloud_map` 语义，数据源为 RTAB-Map/PCD 平台几何资产；实时扫描帧只能保留为诊断旁路。
4. 用户给出的 `/api/debug/logs/tail` `duration_ms=23840.337` 是普通 tail 路径，不是 `/events/stream` 长连接路径；当前应按真实性能问题排查，不能用“长连接正常”解释。
5. 局域网 Web 监控延迟目标提升到轻量链路 P95 < 500 ms；需要把轮询、日志、点云地图传输和浏览器渲染拆开治理。
6. 本轮已新增详细计划文档：`2026-04-28_点云地图与低延迟闭环/todolist.md` 与 `feadback.md`。

## 9. 2026-04-28 代码续作记录

1. `contracts/debug_views.py` 新增 `DebugPointCloudMapView`，`DebugOverviewView` 增加 `pointcloud_map` 与 `runtime_governance`，日志 tail 视图增加 `metrics`。
2. `services/geometry/pointcloud_map.py` 新增 PCD（点云数据文件）解析、缓存、revision 与 LOD（细节层级）能力，支持 ASCII（文本）和 binary（二进制）PCD 的 `x/y/z` 字段；`binary_compressed` 仍明确返回 unsupported（不支持）。
3. `gateways/http/debug_panel.py` 新增 `/api/debug/pointcloud-map/latest/meta` 和 `/api/debug/pointcloud-map/latest.bin`。累计地图缺失时返回 `missing/source_unavailable`，不会回退到 Go2 DDS 最新扫描帧。
4. `/debug` UI（用户界面）已重新布局：主面板名称为“3D 点云地图”，使用 `ArrayBuffer`/`Float32Array` 读取二进制点云并按 revision 缓存；“实时扫描帧诊断”单独显示旧点云帧。
5. 前端轮询已从单个 `Promise.allSettled` 同频 500 ms 拆成三组：轻量状态与点云地图 meta 500 ms、扫描帧/视频 1000 ms、日志 1500 ms，避免日志或大点云阻塞整轮刷新。
6. `/api/debug/logs/tail` 已增加 `auth_ms`、`path_resolve_ms`、`file_stat_ms`、`file_read_ms`、`bytes_read`、`raw_lines`、`server_compute_ms` 等分段观测，普通 tail 立即返回，不实现伪长轮询。
7. `gateways/app.py` 对 `/api/debug/*` 成功请求启用日志限流，降低日志 tail 自身导致 `http.request.completed` 日志快速膨胀的风险。
8. 新增 `tests/unit/test_pointcloud_map.py` 与扩展 debug panel 集成测试，覆盖 PCD 解析、缺字段、缓存 LOD、pointcloud-map meta/bin 和扫描帧不冒充地图。用户随后要求“不用测试了”，因此本轮停止继续执行测试；此前只做过语法编译，系统 Python 直接跑 pytest 因缺少 `pydantic`/`fastapi` 未进入断言。

当前未完成项：

1. 未接入在线 RTAB-Map 累计地图云 topic；本轮正式闭环的是当前激活地图版本或配置 PCD 根目录下的 `cloud_map.pcd`。
2. 未在 Go2 真机和真实局域网浏览器中采集 P50/P95/max；因此不能声明 0.5 s 目标已实测达成。
3. 若现场 PCD 为 `binary_compressed`，当前接口会明确返回 unsupported，需要后续增加解压支持或改由 RTAB-Map/PCL 侧导出 ASCII/binary PCD。

## 10. 2026-04-28 WebUI 提示词本地化与优化记录

用户提供的通用提示词要求 React + TypeScript（类型脚本）+ Tailwind CSS（工具优先 CSS 框架）+ mock data（模拟数据）。结合当前仓库实际，本轮修正为：

1. 不引入新的前端构建链路；继续使用 `gateways/http/debug_panel.py` 的内联 HTML/CSS/JavaScript（超文本标记语言/层叠样式表/脚本）。
2. 不使用 mock data；所有模块读取现有只读 API（应用程序接口）：`/api/debug/*`、`/api/health`、`/api/tasks`、`/api/runtime/governance`。
3. 不提供控制按钮；Emergency Stop（紧急停止）仅展示状态，不做点击控制，任务取消也不进入 sp12 只读面板。
4. 主视觉定位为 robotics mission control dashboard（机器人任务控制台），但必须保持工程调试面信息密度、可读性和异常优先级。
5. WebUI 主 3D 视图继续严格使用累计点云地图接口，不回退到实时扫描帧。

本轮 UI 已优化：

1. 顶部增加机器人名称、型号、连接状态、运行模式、电池、HTTP 往返、资源摘要、当前时间、Emergency Stop 状态。
2. 左侧增加 Overview、Live Map、3D Map、Video、Navigation、Services、Logs、Settings 锚点导航。
3. Overview 增加当前任务、位姿与运动、传感器链路摘要。
4. 服务状态增加 service matrix（服务矩阵），汇总 gateway、mapping、localization、navigation、exploration、RTAB-Map 治理、点云地图、扫描帧、视频和日志。
5. 视频区改为“视频与感知”，增加只读 overlay（叠层）展示本地模型状态和帧/延迟。
6. 日志区增加等级过滤和搜索输入，仍只读取配置日志 tail，不暴露文件路径。

## 2026-04-30 全链路实时流式迁移

SP12 的旧轮询设计已被全链路实时流式方案取代。`/debug` 页面改为通过 `/streams/ws` 消费 `StreamBus` 数据帧；数据类 `/api/debug/*` latest/snapshot/logs tail 入口只返回 410 迁移提示，不再返回实时快照。

后续阅读本文件时，以本节为准：历史条目中关于 500 ms 轮询、分组轮询、普通 logs tail 作为实时 UI 数据源、`/api/debug/pointcloud-map/latest.bin` 作为主实时链路等描述，均属于旧实现记录，不应继续作为当前开发方向。

## 2026-05-15 Go2 端侧视频推流监控页彻底修复记录

1. 本轮确认视频主链边界不变：Go2 端侧主动推 `/streams/video.ingest`，平台只接收 latest-only 缓存并转发 `/streams/video.mjpeg`，监控页和 YOLO-World 均不触发平台采集帧。
2. 真机证据显示 `/streams/video.mjpeg` 返回标准 `multipart/x-mixed-replace`，抽出的 JPEG 可被 `file` 和 PIL（Python Imaging Library，Python 图像库）识别为 1920x1080；黑屏根因不是后端 JPEG 不可解码。
3. 前端旧实现把 `video.frame.stream_url` 元数据到达当成 `rendered_frames`，实际 `<img>` 的 MJPEG 连接断开后不会可靠重连，导致 `ingest_fps`、`payload_bytes` 仍变化但画面区域只剩背景。
4. 已将 `/debug` 视频显示改为前端主动 `fetch()` 读取 `/streams/video.mjpeg`：解析 JPEG SOI/EOI，逐帧用浏览器 JPEG 解码能力验证，再用 object URL 替换 `<img>` 当前帧。
5. `rendered_frames` 现在代表浏览器实际完成解码并显示的 JPEG 帧数；同时新增浏览器侧 `browser_stream`、`browser_bytes`、`decoded_bytes`、`decode_errors`、`last_decode_error`、`last_rendered_at` 观测字段。
6. MJPEG 字节流结束、HTTP 错误、JPEG 解码失败或浏览器显示失败时，前端进入明确重连/错误状态，并按退避策略重建 `/streams/video.mjpeg` 连接；不再仅依赖 WebSocket 元数据判断画面可见。
7. 本轮未修改 Go2 端侧依赖配置、未把 `go2_sdk` 放入平台 settings、未恢复平台采集帧，也未改变 YOLO-World 的端侧推流抽最新帧边界。
