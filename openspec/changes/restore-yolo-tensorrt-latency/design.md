## 背景

用户已确认实机 `.env` 正确使用 `.engine`/TensorRT，因此本设计把 detector 视为可能已经低延迟，把 `duration_ms=3320.427` 作为完整感知管线 total 证据来分析。

当前执行链路为：`PerceptionVideoRuntime._run_once_locked` 每 tick 调用 `PerceptionService.perceive_current_scene`，后者进入 `PerceptionService._process_image_locked`，依次执行 detector、segmentation、geometry、tracker、scene_description，并把 `stage_duration_ms.total` 写入 `perception.scene_ready` 事件 metadata 的 `duration_ms`。`video.detections` 也在该完整链路返回前后发布，因此 Web 检测框实际跟随完整感知完成频率，而不是裸 YOLO-World detector 频率。

用户明确修正本轮方向：高频 YOLO-World 检测链路应同时服务 Web 检测框和记忆三角测量输入；本轮不做点云预算、不改 `PointCloudProvider` API、不对 `pointcloud.frame` stream/provider 入口做下采样或限流。geometry 仍可作为慢链路存在，但不得阻塞高频检测/2D 跟踪/`video.detections` 发布。

监控 Web 边界：本变更不新增监控页面可见指标或布局元素。Web 只继续使用现有 `video.detections` 绘制检测框；刷新间隔、全链路 total 等诊断通过运行时状态、API、日志或测试钩子采集。

权威边界：OpenSpec specs 定义需求；代码实现链路解耦和反压隔离；部署/HIL 报告验证 detector/overlay cadence 与 全链路 total；详细 docs 用于运维解释并需与 OpenSpec 同步。当前详细方案文档为 `docs/开发文档/子项目拆解/补丁文档/2026-06-24_感知全链路延迟与几何预算/方案设计_感知全链路延迟与几何预算.md`，并作为 `感知跟踪` 与 `部署运维` 的 source-map 来源。

## 目标 / 非目标

**目标：**

- 让 YOLO-World detector + 2D tracker + `video.detections` 按 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC` 目标周期运行，避免被 segmentation/geometry/scene_description/关键帧记忆 同步拖慢。
- 让连续记忆摄入/三角测量使用高频检测观测输入积累 track_id、bbox、相机位姿和射线信息。
- 保留慢链路的 完整感知、geometry grounding、场景摘要 和关键帧记忆写入能力，但将其与高频检测循环解耦。
- 保持 Web UI 不增加新信息，只改善现有检测框刷新来源与频率。

**非目标：**

- 不修改点云预算，不新增或复用 `NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS` 作为本轮感知 geometry 改造项。
- 不扩展 `PointCloudProvider.get_latest_point_cloud` API，不改 Go2 provider 点云拉取签名。
- 不对 `pointcloud.frame` stream/provider 点云入口做下采样、限流或裁剪。
- 不在本轮优化 geometry polygon/投影算法；如果 全链路 total 后续仍慢，另开变更处理。
- 不新增 Web 监控可见字段或布局元素。

## 决策

1. **建立 快速检测观测链路。**
   - 选择：新增或拆出轻量检测入口，仅执行读取最新视频帧、YOLO detector、2D tracker、track_id attach、`video.detections` 发布和连续观测摄入所需的最小 context 构造。
   - 理由：当前 Web 框和连续观测被完整 `process_image` 串行阶段拖慢；detector 低延迟无法体现到 UI 和三角测量输入。
   - 替代方案：只把 `_publish_detection_overlay_frame` 前移。该方案仍无法解决下一轮检测被 geometry/scene/memory 阻塞的问题，因此不采用。

2. **高频检测观测同时服务 Web 与记忆三角测量。**
   - 选择：快速路径产生的 2D detections/tracks 作为 `video.detections` payload，同时作为 `MemoryService.ingest_perception_observation(..., persist=False)` 或等价连续观测接口的输入。
   - 理由：三角测量需要多视角、高频、带 track_id 的 bbox/射线样本；低频关键帧只能决定何时固化，不能决定何时观测。
   - 替代方案：Web 高频、记忆仍低频。该方案会修复 UI 但损害三角测量输入，不符合需求。

3. **慢链路 latest-only/合并，不能反压快速路径。**
   - 选择：segmentation、geometry、scene_description、关键帧记忆 写入作为慢链路消费最新观测；慢链路忙时合并/替换待处理项，而不是排队阻塞 detector。
   - 理由：如果慢链路仍串行卡住 `run_once()`，`NUWAX_PERCEPTION_STREAM_INTERVAL_SEC` 无法生效。
   - 替代方案：为慢链路开无限队列。该方案会制造延迟堆积和过期观测，不采用。

4. **geometry 不在本轮改算法或点云入口。**
   - 选择：geometry 保持现有语义，只从高频检测路径移出；不做 max_points、不改 provider/stream、不做下采样。
   - 理由：用户明确指出点云预算和 stream/provider 限流方向偏离；当前主要问题是链路耦合导致检测框和三角测量观测低频。
   - 替代方案：有界 geometry。该方案本轮不采用，可在确认慢链路本身仍需优化时另开 OpenSpec。

5. **Web 不新增信息。**
   - 选择：继续复用现有 `video.detections` 绘框，不增加 cadence/latency 文本。
   - 理由：监控页面空间有限；诊断字段通过后端状态/API/log/test 采集。

6. **验收必须同时看 快速节奏 与 全链路 total。**
   - 选择：HIL 报告记录 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC`、detector 耗时、`video.detections` interval、快速处理帧数、连续观测摄入计数、全链路 `stage_duration_ms.total`。
   - 理由：只有 total 无法判断 UI/三角测量是否已从慢链路解耦。

## 风险 / 取舍

- [风险] tracker 同时被 快速路径与 完整路径 更新导致 track_id 抖动。→ 缓解：tracker 只在 快速路径更新；慢路径消费带 track_id 的 detection snapshot，不重复 update。
- [风险] 慢路径消费过期帧，写入旧场景。→ 缓解：慢路径使用 latest-only/合并，并记录 source frame_id/timestamp；过期观测可丢弃。
- [风险] 连续记忆摄入 仍做重操作。→ 缓解：保持 `persist=False`，不落盘不向量化；若该路径仍慢，后续单独拆出更轻的 ray sample ingest。
- [风险] Web 底图高频但框高频检测仍不达标。→ 缓解：通过 API/log/test 记录 detector 耗时、发布间隔、快速循环耗时，定位 detector、tracker、发布或视频源。
- [风险] 文档仍残留点云预算方向。→ 缓解：OpenSpec 明确 Non-Goals 禁止本轮改 `NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS`、provider API 和 stream 点云入口。

## 迁移计划

1. 先用 GitNexus impact 分析将要修改的 快速路径、`PerceptionVideoRuntime._run_once_locked`、`PerceptionService` 检测/overlay 发布、memory continuous ingest 相关符号。
2. 新增 快速检测观测 方法或等价拆分：读最新视频帧 → detector → tracker → attach track ids → publish `video.detections` → 返回轻量 observation context。
3. 调整 `PerceptionVideoRuntime`：主循环优先按 interval 执行 快速路径；memory continuous ingest 使用 fast context 和当前 pose/map/localization，关键帧只触发 slow/完整感知 或记忆固化。
4. 将 slow/完整感知 改为 latest-only/合并 后台/阶段处理，确保 segmentation/geometry/scene_description/关键帧记忆 不阻塞下一次 快速检测 tick。
5. 保持 Web 静态页面布局不变；必要诊断仅进入状态/API/log/test。
6. 本地跑单元/OpenSpec/GitNexus detect；实机跑 `.engine` HIL，比较 `video.detections` interval 与 全链路 total。

## 待决问题

- 连续记忆摄入 当前依赖 `PerceptionContext`；实现时应优先复用轻量 `PerceptionContext`，若字段不足，再新增最小契约或适配器，避免把 scene/geometry 重新拉回 快速路径。
- 慢路径 keyframe 固化需要使用 fast snapshot 还是重新跑 完整感知？首版倾向 latest snapshot + slow enrichment，避免重复 detector；若质量不足，再单独设计关键帧复核。
