# todolist：sp12 机器人本机 Web 可视化调试面板

## 1. 任务原则

1. 本任务是 sp12 独立子项目，不放入“补丁文档”。
2. MVP（最小可行产品）是机器人本机只读 Web（网页）可视化调试面板。
3. 先完成真机数据可用性矩阵，再实现 UI（用户界面）。
4. 不做任何控制命令。
5. 地图和 frontier（探索前沿）刷新 P95（第 95 百分位延迟）必须小于 1 秒。
6. 点云本机运行，不预设强制限频、强制降采样、强制最大点数；只保留可选保护配置与性能观测。

### 2026-04-28 用户修正

用户明确要求 Web 主 3D 视图展示“累计扫描形成的点云地图”，不是 Go2 DDS（数据分发服务）最新扫描帧。后续实现必须把主视图从 `pointcloud.frame` 修正为 `pointcloud_map` 语义；`/api/debug/pointcloud/latest` 只能作为实时扫描帧诊断旁路，不能继续冒充点云地图。

低延迟目标同步提升：局域网轻量监控接口、点云地图元数据变化、日志增量读取目标 P95（第 95 百分位延迟）为 500 ms 以内；大体积点云地图必须通过缓存、revision（修订版本）、LOD（细节层级）和二进制传输避免全量 JSON（JavaScript 对象表示法）反复阻塞页面。

详细计划已落地到：

`docs/开发文档/子项目拆解/sp12_机器人本机Web可视化调试面板/2026-04-28_点云地图与低延迟闭环/todolist.md`

## 2. 依赖

| 依赖项 | 说明 | 阻塞内容 |
| --- | --- | --- |
| SP06 状态、观察与制品 | 提供服务状态、诊断、观察面基础 | 服务状态、日志、数据时间戳展示 |
| SP08 建图、定位与导航 | 提供地图、定位、导航、探索契约 | 2D SLAM（同步定位与建图）栅格、位姿、轨迹、frontier |
| 平台 geometry（几何）主链 | 提供点云、frame（坐标系）和空间数据 | 3D（三维）点云与坐标对齐 |
| Go2 端侧数据面 | 提供真机 DDS（数据分发服务）输入 | 真机数据可用性验收 |
| 宿主机网关 | 承载 HTTP（超文本传输协议）/WebSocket（网页套接字）只读接口 | Web 面板访问入口 |

## 3. 阶段任务

### 阶段 A：文档与边界冻结

- [x] 创建 sp12 专属开发文档目录。
- [x] 编写 PDR.md。
- [x] 编写 todolist.md。
- [x] 编写 feadback.md。
- [x] 编写 数据可用性矩阵.md。
- [x] 编写 接口契约草案.md。
- [x] 编写 真机验收清单.md。

验收：目标目录完整，且没有创建公开文档、没有修改业务代码。

### 阶段 B：真机数据可用性矩阵

- [ ] 确认服务状态数据源可用。
- [ ] 确认日志增量读取数据源可用。
- [ ] 确认 2D SLAM 栅格图数据源可用。
- [ ] 确认地图元数据包含 frame、分辨率、原点、时间戳。
- [ ] 确认机器人当前位姿数据源可用。
- [ ] 确认机器人轨迹历史可由位姿缓存或运行态历史生成。
- [ ] 确认 frontier 数据源可用。
- [ ] 确认 3D 点云数据源可用。
- [ ] 确认各数据源不可用时能返回明确原因。

验收：`数据可用性矩阵.md` 中每项都有真机结论、证据、延迟样本和阻塞判断。

### 阶段 C：只读接口契约冻结

- [x] 确定状态、日志、地图、位姿、轨迹、frontier、点云接口路径。
- [x] 确定每个接口的字段、时间戳、frame、错误语义。
- [x] 确定实时更新使用轮询、SSE（服务器发送事件）或 WebSocket。
- [x] 明确接口只允许 GET（读取）或只读事件流。
- [x] 明确禁止 POST（提交）、PATCH（局部更新）、DELETE（删除）类控制接口进入 MVP。

验收：`接口契约草案.md` 可直接指导后端与前端并行开发。

### 阶段 D：后端只读聚合层

- [x] 复用现有状态服务和诊断数据。
- [x] 复用现有日志来源，提供游标分页或增量读取。
- [x] 复用现有地图快照输出 2D 栅格。
- [x] 复用现有定位快照输出当前位姿。
- [x] 从定位历史或运行态缓存输出轨迹。
- [x] 复用探索状态或 frontier 计算结果输出高亮图层。
- [x] 复用平台点云数据源输出点云帧或点云流。
- [x] 所有响应携带数据时间戳、采集时间、返回时间。

验收：后端不引入控制命令，不新建平行建图导航主链。

### 阶段 E：Web 面板 UI

- [x] 实现服务状态区域。
- [x] 实现日志区域。
- [x] 实现 2D 地图区，包含已探索、未知、障碍配色。
- [x] 在 2D 地图叠加机器人位姿箭头。
- [x] 在 2D 地图叠加机器人轨迹。
- [x] 在 2D 地图叠加 frontier 高亮。（UI 已支持后端返回 frontiers 时叠加；当前未暴露几何时显示“暂未暴露几何”）
- [x] 实现 3D 点云区域。
- [x] 展示点云帧率、点数、延迟和渲染耗时。（有内联 points 时记录 browser_render_ms；真机点云流畅度待验收补样本）
- [x] 所有数据不可用时显示明确空态和错误原因。
- [x] 将主 3D 点云区从“实时扫描帧”修正为“累计点云地图”，并把实时扫描帧下沉为旁路诊断面板。
- [x] 主 3D 点云地图只消费平台 RTAB-Map/PCD 几何资产或在线累计地图云，不回退到 Go2 DDS 最新帧。（本轮已闭环 PCD 几何资产；在线 RTAB-Map 地图云订阅仍待侧车 topic 确认）

验收：本机浏览器可打开面板并完成核心调试观察，页面不包含任何控制按钮。

### 阶段 F：性能与真机验收

- [ ] 采集地图刷新端到端延迟样本。
- [ ] 采集 frontier 刷新端到端延迟样本。
- [ ] 验证地图 P95 延迟 < 1 秒。
- [ ] 验证 frontier P95 延迟 < 1 秒。
- [ ] 记录点云帧率、点数、端到端延迟、浏览器渲染耗时。
- [ ] 观察本机 CPU（中央处理器）、内存、网络、浏览器流畅度。
- [ ] 若点云导致不可用，再评估可选保护配置，不把保护项写成默认强制限制。
- [ ] 采集 Web 轻量接口、日志 tail、点云地图 meta 和点云地图二进制数据的 P50/P95/max，轻量链路目标 P95 < 500 ms。
- [ ] 验证普通 `/api/debug/logs/tail` 不再出现 2 s 以上服务端耗时；若使用 SSE/WS（服务器发送事件/网页套接字）长连接，单独标记连接驻留时间，不计入普通请求延迟。

验收：`真机验收清单.md` 全部完成，性能指标有样本证据。

## 4. 完成定义

1. sp12 文档已在指定目录下落地。
2. 实现前的数据可用性矩阵已经完成。
3. UI 只读且无控制命令。
4. 服务状态、日志、2D 地图、位姿、轨迹、frontier、3D 点云地图均有可用或不可用状态展示。
5. 地图/frontier P95 延迟 < 1 秒。
6. 点云地图主视图来自 RTAB-Map/PCD 累计地图资产；实时扫描帧只作为旁路诊断，不作为 MVP 点云地图通过条件。
7. 点云地图传输和渲染保护只作为可视化承载方式，不改变完整地图资产真值；HTTP 调试接口保留元数据/引用字段用于观测。
7. ExplorationRuntime 已暴露只读 `latest_frontiers`/`current_frontier` 元数据，UI 在后端返回 frontier 几何时叠加高亮；真机 frontier P95 仍需验收采样。

## 6. 2026-04-28 点云地图与低延迟续作记录

1. 已新增累计点云地图主链契约 `DebugPointCloudMapView`，`/api/debug/overview` 增加 `pointcloud_map` 与 `runtime_governance` 摘要。
2. 已新增 `/api/debug/pointcloud-map/latest/meta` 与 `/api/debug/pointcloud-map/latest.bin`。`meta` 返回地图版本、revision、frame、bounds、PCD 路径、LOD（细节层级）与服务端耗时；`.bin` 返回 `float32 xyz` 二进制点云。
3. 已新增 PCD（点云数据文件）解析与缓存模块，支持 ASCII（文本）和 binary（二进制）PCD 的 `x/y/z` 字段，按 `path + mtime + size` 生成 revision，并提供 preview/medium/full LOD。
4. `/debug` 主 3D 区已改为“3D 点云地图”，只消费 `/api/debug/pointcloud-map/*`；旧 `/api/debug/pointcloud/latest` 仅用于“实时扫描帧诊断”，并在响应中暴露 `not_a_pointcloud_map=true`。
5. 前端轮询已拆分为轻量状态/地图/点云地图 meta 500 ms、扫描帧/视频 1000 ms、日志 1500 ms；点云地图 revision 未变化时不重复下载二进制点云。
6. `/api/debug/logs/tail` 已增加认证、路径解析、文件 stat、文件读取、读取字节数、行数与服务端总耗时指标；`/api/debug/*` 普通成功请求在 HTTP 日志中按低价值路径限流，避免日志面板读取日志又持续写入大量完成日志。
7. 已补充 PCD 单元测试和 debug pointcloud-map 集成测试文件；按用户 2026-04-28 明确要求，本轮不继续执行测试。
8. 当前仍未完成真机 P95 样本采集；开发机没有真实 RTAB-Map 在线累计地图云 topic 和 Go2 真机浏览器环境，因此不能声明 0.5 s 真机延迟目标已实测达成。

## 5. 2026-04-27 后端最小闭环进展

1. 已新增只读 debug（调试）契约与 HTTP（超文本传输协议）聚合路由。
2. 已挂载 `/debug` 页面骨架和 `/api/debug/*` 只读接口；页面不包含控制按钮。
3. 地图、定位、导航复用现有 runtime/service（运行时/服务）读取口径，不复制建图、导航或 frontier（探索前沿）计算逻辑。
4. 日志 tail 读取配置日志文件 `runtime.config.logging.log_file`，使用 byte offset cursor（字节偏移游标）增量回放，不暴露任意路径参数。
5. 点云读取不传 `max_points`，不在 debug 接口默认限频、降采样或限制最大点数；HTTP 调试接口返回真实 `points` 供本机页面实时渲染，并保留保护配置和观测指标。
6. 当前开发机无真机数据时，接口按数据情况返回 `missing`、`unsupported`、`stale` 或 `error`，不伪造数据。
7. 已完成静态验证：`python3 -m py_compile` 通过；debug router 未发现 POST/PATCH/DELETE/PUT 控制型路由。


### 阶段 D 实现记录（2026-04-27）

- [x] 新增 `contracts/debug_views.py`，定义调试信封、错误、概览、地图、定位、导航、点云、日志 tail 视图。
- [x] 新增 `gateways/http/debug_panel.py`，挂载 `/debug` 与 `/api/debug/*` 只读接口。
- [x] 修改 `gateways/app.py`，根路由增加 `debug_panel_path=/debug` 并挂载 debug router。
- [x] 修改 `gateways/http/__init__.py` 与 `contracts/__init__.py` 导出调试路由和契约。
- [x] 追加 `tests/integration/test_gateway_integration.py::test_gateway_debug_panel_realtime_readonly_routes` 覆盖只读路由、无控制按钮、POST/PATCH/DELETE 返回 405。

验证记录：`python3 -m py_compile` 已通过；`pytest tests/integration/test_gateway_integration.py::test_gateway_debug_panel_realtime_readonly_routes` 因当前开发环境缺少 `fastapi` 包在收集阶段失败，未进入用例执行。


### 阶段 E 实现记录（2026-04-27）

- [x] `/debug` 已从接口链接页升级为无构建链路的内联 HTML/CSS/JS 本机只读调试面板。
- [x] 顶部总览轮询 `/api/debug/overview`，展示 health/state/localization/map/navigation/exploration/pointcloud/video/logs 的 status、source_timestamp、latency、stale 和前端接收时间。
- [x] 地图区轮询 `/api/debug/map`，有 occupancy_grid 时用 canvas 绘制 unknown/free/occupied 三色；无数据时显示 missing/stale/error 原因。
- [x] 定位区轮询 `/api/debug/localization`，显示 pose、frame、yaw、trajectory_count，并在地图可用时叠加机器人箭头与轨迹。
- [x] 导航区轮询 `/api/debug/navigation` 与 `/api/debug/exploration/frontier/latest`，显示 navigation/exploration 摘要、frontier_count、waiting_for；后端返回 frontier 几何时叠加高亮，否则明确显示缺失原因。
- [x] 点云区轮询 `/api/debug/pointcloud/latest`，无外网 CDN；返回内联 `points` 时 canvas 实时俯视投影渲染，并展示 point_count、frame、payload、metrics、protection 等观测信息。
- [x] 日志区轮询 `/api/debug/logs/tail`，使用 cursor 增量展示运行事件 tail，不提供文件路径输入。
- [x] 页面只发起 GET 请求，不包含表单、按钮、POST/PATCH/DELETE/PUT 调用。

验证记录：`python3 -m py_compile contracts/debug_views.py services/navigation/exploration_runtime.py gateways/http/debug_panel.py tests/integration/test_gateway_integration.py` 通过；提取内联脚本后 `node --check /tmp/sp12_debug_panel.js` 通过；静态检查未发现 `<button>`、`<form>`、控制型 fetch 方法、POST/PATCH/DELETE/PUT 路由或 refresh 副作用。


目标集成测试验证：`python3 -m pytest tests/integration/test_gateway_integration.py::test_gateway_debug_panel_realtime_readonly_routes -q` 因当前环境缺少 `fastapi` 包在收集阶段失败，未进入用例执行。

### 复审修复记录（2026-04-27）

- [x] 移除 `/api/debug/*` GET 路径中的刷新副作用，调试面板只读取已有缓存。
- [x] 点云 HTTP 响应返回真实 `points`，前端实时绘制，并保留元数据、引用、保护配置和性能观测。
- [x] ExplorationRuntime 暴露只读 frontier 几何，UI 在可用时叠加高亮，不可用时显示明确缺失原因。
- [x] 新增 `/api/debug/video/latest` 和 `/api/debug/video/frame/latest`，页面实时展示 ImageProvider 视频帧。
- [x] 日志 tail 改为读取配置日志文件，使用 byte offset cursor（字节偏移游标）增量读取并支持 level/source 过滤。

## 2026-04-30 全链路实时流式迁移

用户已要求坚决杜绝快照轮询、一次性抓取等实时主链。SP12 的实时数据链路已迁移为 `/streams/ws` WebSocket（网页套接字）订阅，历史文档中“轮询 `/api/debug/*`”“500 ms 轮询”“latest/snapshot 主接口”等表述仅代表旧实现背景，不再作为当前实现方案。

当前约束：

- `/debug` 页面只订阅 `/streams/ws`，不再周期请求 `/api/debug/*` 实时数据接口。
- `/api/debug/overview`、`/api/debug/snapshot`、`/api/debug/*/latest`、`/api/debug/logs/tail` 等实时数据 HTTP（超文本传输协议）入口返回 410 迁移提示。
- 实时地图、定位、导航、探索、日志、任务、机器人状态统一走 `StreamBus` topic；点云扫描帧走 Go2 DDS（数据分发服务）源头回调发布 `pointcloud.frame`。
- 视频若没有原生源头流，只发布 `unsupported` 诊断帧，不允许用定时抓帧或固定 interval refresh 冒充流式；累计点云地图已改为平台 PCD 资产驱动的 `pointcloud_map.meta` / `pointcloud_map.chunk`，缺 PCD 时发布 `missing` 诊断且不回退到最新扫描帧。

## 2026-05-15 视频监控页 MJPEG 解码闭环修复

- [x] 确认黑屏断点不是 Go2 未推流，也不是 MJPEG/JPEG 本身不可解码。
- [x] 保持 Go2 端侧主动推流、平台只接收/缓存/转发的边界，不引入平台采集帧。
- [x] 前端从直接 `<img src=/streams/video.mjpeg>` 改为 `fetch()` 消费 MJPEG 字节流，解析 JPEG 并按真实解码结果渲染。
- [x] `rendered_frames` 改为真实浏览器解码帧计数，不再由 `video.frame` 元数据自增。
- [x] 增加 MJPEG 前端解码状态、字节数、解码错误、最后渲染时间和断线重连观测。
- [x] 更新接口契约，明确 `video.frame` 只承载元数据和 `stream_url`，图像字节由 `/streams/video.mjpeg` 承载。
