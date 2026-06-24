## 原因

实机部署已确认使用 `.engine`/TensorRT，但 Web 监控中的 YOLO-World 检测框刷新明显低于 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC` 期望频率。代码链路显示当前 `video.detections` 在完整 `process_image` 后发布，而完整链路包含 detector、segmentation、geometry、tracker、scene_description 等同步阶段；因此即使 detector 本身接近历史 30ms，检测框和连续观测也会被慢链路拖慢。

记忆三角测量同样需要高频检测/跟踪观测输入；关键帧/记忆落盘不应决定 YOLO 检测频率，也不应决定 Web 检测框刷新频率。

## 变更内容

- 将 YOLO-World 高频检测/2D 跟踪/`video.detections` 发布从完整感知链路中拆出，形成 快速检测观测链路。
- `video.detections` SHALL 在 detector + 2D tracker 完成后发布，MUST NOT 等待 segmentation、geometry、scene_description、关键帧记忆 写入或向量化。
- 连续记忆摄入/三角测量 SHALL 使用同一高频检测观测输入积累 track_id、bbox、相机位姿与射线信息；关键帧只决定低频记忆固化/落盘/向量化。
- 慢链路可继续执行 segmentation、geometry、scene_description 和关键帧记忆写入，但不得对高频检测循环形成同步反压；慢链路忙时应 latest-only/coalesce，而不是排队拖慢检测框。
- 监控 Web 不新增可见信息或布局元素；仅继续消费 `video.detections` 绘制检测框。
- 本轮不修改点云预算、不新增/复用 `NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS` 用于感知、不改 `PointCloudProvider` API、不对 `pointcloud.frame` stream/provider 入口做下采样或限流。
- 保留全链路 `stage_duration_ms` 作为诊断，继续区分 detector 与 全链路 total；但本轮主要修复目标是链路解耦而不是 geometry 算法优化。

## 能力

### 新增能力

无。

### 修改能力

- `感知跟踪`: 增加高频 YOLO 检测观测链路、`video.detections` 刷新语义、连续记忆三角测量输入与慢链路反压隔离的需求约束。
- `部署运维`: 增加 HIL 验收时必须分别报告 detector/overlay cadence 与 全链路 total，确认 Web 检测框和连续观测不再被 keyframe/memory/geometry 慢链路拖慢。

## 影响

- 代码：预计涉及 `services/perception/service.py`、`services/perception/video_runtime.py`、`services/perception/trackers.py`（如需复用 tracker 状态）、`services/memory/service.py` 或其调用适配层、`contracts/runtime_views.py`（仅后端状态字段时）。不涉及 `providers/pointcloud.py`、`drivers/go2/providers.py` 的点云 API 改造。
- 配置与部署：继续使用现有 `NUWAX_PERCEPTION_STREAM_INTERVAL_SEC` 控制高频检测目标周期；不新增 Web 页面配置，不新增点云预算配置。
- 文档与规格：修改 OpenSpec `感知跟踪`、`部署运维` delta spec，维护详细方案文档，并同步 `openspec/source-map.json`。
- 验证：本地测试覆盖 快速路径 不调用 segmentation/geometry/scene_description、`video.detections` 在 detector/tracker 后发布、慢链路忙时 快速路径 仍可推进、连续记忆摄入可接收高频观测；实机记录 detector 耗时、`video.detections` 间隔、processed fast frames、全链路 total。
