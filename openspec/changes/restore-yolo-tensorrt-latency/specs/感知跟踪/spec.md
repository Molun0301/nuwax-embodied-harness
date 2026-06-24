## ADDED Requirements

### Requirement: 完整感知阶段延迟可观测
感知服务 SHALL 在完整感知结果中暴露 detector、segmentation、geometry、tracker、scene_description 和 total 的阶段耗时，使运维人员能够区分 YOLO detector 低延迟路径与完整感知慢链路。

#### Scenario: 完整感知事件被发出
- **WHEN** `perception.scene_ready` 事件或最新完整感知状态被发布
- **THEN** payload 或 metadata MUST 包含 `stage_duration_ms.detector`、`stage_duration_ms.geometry` 和 `stage_duration_ms.total`，并保留检测后端 runtime 与模型来源作为轻量诊断字段

#### Scenario: 解释运行时事件耗时
- **WHEN** 运行时日志记录 `perception.scene_ready` 的 `duration_ms`
- **THEN** 该值 MUST 与 `stage_duration_ms.total` 表达同一轮完整感知管线耗时，并且 SHALL NOT 被解释为 detector 单阶段耗时

### Requirement: 快速检测框跟随 YOLO 节奏
视频画面中的检测框 SHALL 跟随高频 YOLO-World 检测和 2D tracker 观测链路刷新，MUST NOT 被 segmentation、geometry、scene_description、关键帧或记忆写入门槛直接限制。

#### Scenario: 快速检测 tick 完成
- **WHEN** 持续感知运行时完成一次 快速检测 tick 的 detector 与 2D tracker 更新
- **THEN** 感知服务 MUST 发布对应的 `video.detections` 帧
- **AND** 该发布 MUST 发生在 segmentation、geometry、scene_description、关键帧记忆 写入、向量化或 全链路 `perception.scene_ready` 等慢阶段之前

#### Scenario: 检测框刷新慢于配置间隔
- **WHEN** `video.detections` 实际发布间隔明显大于 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC`
- **THEN** 运维诊断 SHALL 对比 detector 耗时、快速循环耗时、runtime running 状态、video source 状态和 发布间隔，以判断是 detector/调度/视频源/topic 问题，而不是直接归因于 memory/keyframe 门槛

#### Scenario: Web 监控渲染检测框
- **WHEN** Web 监控面板收到 `video.detections` 帧
- **THEN** 前端 MUST 使用该帧更新检测框，并且 SHALL NOT 仅在 keyframe、memory 写入或场景记忆更新时刷新检测框
- **AND** 本变更 SHOULD NOT 新增监控页面可见诊断字段；刷新间隔与全链路耗时诊断 SHOULD 通过运行时状态、API、日志或测试钩子采集

### Requirement: 高频观测支持记忆三角测量
连续记忆摄入和三角测量 SHALL 使用高频检测/跟踪观测输入积累多视角样本，关键帧只决定何时固化、落盘或向量化。

#### Scenario: 连续观测可用
- **WHEN** 快速检测 tick 产生带 frame_id、timestamp、camera_id、2D bbox、label、score 和 track_id 的观测
- **THEN** 记忆连续摄入 SHOULD 能以 `persist=false` 或等价模式消费该观测，并累积三角测量所需的相机位姿、bbox 和射线样本

#### Scenario: 未达到关键帧阈值
- **WHEN** 当前 tick 未达到 keyframe/memory 写入门槛
- **THEN** 系统 MUST NOT 因此跳过 YOLO 检测、2D tracker 更新、`video.detections` 发布或连续观测摄入

#### Scenario: 达到关键帧阈值
- **WHEN** 当前 tick 达到 keyframe/memory 写入门槛
- **THEN** 系统 MAY 触发 slow/完整感知、场景摘要、记忆固化、落盘或向量化，但这些操作 MUST NOT 阻塞后续 快速检测 ticks

### Requirement: 慢感知链路不得反压快速检测
完整感知慢链路 SHALL 与高频检测链路隔离，segmentation、geometry、scene_description 或记忆固化耗时不得把检测框刷新和连续观测降到记忆频率。

#### Scenario: 慢路径繁忙
- **WHEN** segmentation、geometry、scene_description 或 关键帧记忆 写入仍在处理上一帧
- **THEN** 快速检测路径 MUST 能够继续处理更新帧，或明确报告 fast-path 阻塞
- **AND** 慢路径 SHOULD 使用 latest-only/合并 语义，而不是对过期帧建立无界队列

#### Scenario: Tracker 状态被更新
- **WHEN** 快速检测路径 更新 2D tracker 状态并分配 track_id
- **THEN** slow/完整感知 path SHOULD 消费由此产生的 detection snapshot，而不是对同一帧第二次更新同一个 tracker

### Requirement: 本变更不改变几何与点云 API
本变更 SHALL NOT 改变点云预算语义、点云 provider 签名或 stream/provider 下采样行为；除非后续 OpenSpec change 明确针对 geometry，否则 geometry 仍是慢路径增强事项。

#### Scenario: 实现触碰点云输入
- **WHEN** 实施本变更
- **THEN** 代码 MUST NOT 改变 `PointCloudProvider.get_latest_point_cloud` 签名，MUST NOT 对 provider/stream 路径增加 `max_points` 要求，并且 MUST NOT 将 `NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS` 重新解释为本次快速检测修复的一部分
