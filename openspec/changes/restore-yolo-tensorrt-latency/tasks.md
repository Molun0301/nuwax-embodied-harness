## 1. 诊断确认

- [ ] 1.1 收集实机完整未截断的 `perception.scene_ready` payload 和 `video.detections` stream 样本，记录 detector、segmentation、geometry、tracker、scene_description、total、detection_count、frame_id、timestamp。
- [ ] 1.2 记录 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC`、runtime running/enabled/auto_start、fast processed frame 增量、`video.detections` 发布时间间隔和 Web 检测框观察刷新间隔，区分“检测循环低频”和“记忆/keyframe 低频”。
- [x] 1.3 在修改任何函数/类前，对 `PerceptionVideoRuntime._run_once_locked`、`PerceptionService.perceive_current_scene`、`PerceptionService._process_image_locked`、`PerceptionService._publish_detection_overlay_frame`、tracker update 和 memory continuous ingest 相关符号运行 GitNexus impact，并记录调用方、受影响流程和风险等级。
- [x] 1.4 明确本轮不以“模型文件存在/不存在”作为根因判断；只用 detector 耗时、`video.detections` cadence、快速循环耗时 和 全链路 total 判断耦合问题。

## 2. 高频检测观测链路解耦

- [x] 2.1 新增或拆出 快速检测观测链路：读取最新视频帧、执行 YOLO detector、执行 2D tracker、附加 track_id、构造轻量 observation/context。
- [x] 2.2 `video.detections` 必须在 detector + tracker 后立即发布，不等待 segmentation、geometry、scene_description、关键帧记忆 写入、向量化或 全链路 event。
- [x] 2.3 `PerceptionVideoRuntime` 主循环按 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC` 驱动 快速路径；单次 slow/完整感知 不得阻塞下一次 快速检测 tick。
- [x] 2.4 连续记忆摄入 / 三角测量输入使用 快速路径产生的 detections/tracks、frame_id、timestamp、camera pose/ray metadata；`persist=False` 路径不得落盘或向量化。
- [x] 2.5 keyframe 只决定 slow/完整感知、场景摘要、记忆固化或向量化，不得决定 YOLO 检测频率或 `video.detections` 发布频率。
- [x] 2.6 slow/完整感知 使用 latest-only/合并 处理待增强观测；慢链路忙时丢弃/替换过期待处理项，不建立无限队列。
- [x] 2.7 tracker 状态只由 快速路径更新；慢路径消费带 track_id 的 snapshot，避免重复 tracker update 导致 ID 抖动。

## 3. 明确不做的 geometry/点云改造

- [x] 3.1 本轮不修改 `NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS` 语义，不把它作为感知 快速路径 修复项。
- [x] 3.2 本轮不扩展 `PointCloudProvider.get_latest_point_cloud` 签名，不改 Go2 provider 点云拉取 API。
- [x] 3.3 本轮不对 `pointcloud.frame` stream_bus payload 或 provider fallback 做下采样、限流、裁剪或预算改造。
- [x] 3.4 本轮不优化 geometry polygon/投影算法；geometry 只作为 慢路径 阶段保留，并通过解耦避免阻塞 快速路径。

## 4. 可观测性与文档同步

- [x] 4.1 后端状态/API/log/test 钩子记录 fast detector 耗时、快速循环耗时、`video.detections` 发布间隔、快速处理帧数、continuous ingest count 和最近 全链路 `stage_duration_ms.total`。
- [x] 4.2 不新增监控 Web 可见信息；检测框仍复用现有 `video.detections` 绘制链路。
- [x] 4.3 保持 `perception.scene_ready` 事件 payload 轻量；全链路 诊断仍按 stage_duration 拆分。
- [x] 4.4 维护详细方案文档 `docs/开发文档/子项目拆解/补丁文档/2026-06-24_感知全链路延迟与几何预算/方案设计_感知全链路延迟与几何预算.md`，并在 `openspec/source-map.json` 中绑定到 `感知跟踪` 与 `部署运维`。

## 5. 本地验证

- [x] 5.1 添加或更新单元测试：快速路径 不调用 segmentation/geometry/scene_description；`video.detections` 在 detector/tracker 后发布；慢路径 阻塞替身不影响下一次 fast tick。
- [x] 5.2 添加或更新单元测试：连续记忆摄入 接收 fast context 且 `persist=False` 不落盘不向量化；keyframe 路径仍能触发 slow/完整感知 或记忆固化。
- [x] 5.3 添加或更新静态/UI 测试：Web 订阅 `video.detections` 的绘制链路保持不变，未新增页面可见字段。
- [x] 5.4 运行相关单元测试、`openspec validate --all`、`scripts/check_openspec.sh`。
- [x] 5.5 运行 GitNexus detect-changes，确认变更只影响预期符号与执行流。

## 6. 硬件在环验证

- [ ] 6.1 在目标 Jetson/GPU 上使用 `.engine` 连续执行感知基准，记录 detector 耗时、快速循环耗时、`video.detections` interval、快速处理帧数、continuous ingest count、全链路 total 的 P50/P95/最大值。
- [ ] 6.2 验证 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC=0.5` 时 `video.detections` 与 Web 检测框接近 2Hz；若 全链路 total 仍为秒级，也不得拖慢 快速节奏。
- [ ] 6.3 如果 快速节奏 仍低于配置频率，按 detector、tracker、video source、publish、runtime scheduling 继续拆分，不回退到点云预算/provider/stream 限流方向。
- [x] 6.4 将 HIL 结果或硬件不可用说明回填到详细方案文档或关联实施记录，并保持 OpenSpec 验收口径不漂移。
