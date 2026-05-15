# Go2 端侧实时视频推流改造 — todolist（2026-05-15）

## 目标

将监控网页与 YOLO-World 感知主链从“平台采集图像帧 / MJPEG 拼帧”改为“Go2 端侧主动推实时视频流，平台只接收、缓存、转发与抽关键帧”。

## 硬约束

1. Go2 是视频生产者，平台是视频接收者；平台不得把实时视频主链实现为主动 `capture_image()` 或按 FPS 调 `GetImageSample()`。
2. Go2 端侧可以从机器人本地 `videohub` / 相机源读取原始图像，但必须在端侧形成连续推流客户端，主动连接平台 ingest 端点。
3. 平台实时视频主链只做 ingest、latest-only 缓存、播放转发、状态发布。
4. YOLO-World 只从平台已接收的视频流 latest/keyframe 缓冲抽帧；不得自行触发平台到 Go2 的图像采集。
5. `StreamBus` 只承载视频状态与检测 overlay，不承载实时视频字节主链。
6. 保留单次 `capture_image` 作为设备调试能力，但不得参与监控视频或持续感知主链。

## 终态链路

```text
[Go2 端侧 go2_video_streamer]
  VideoClient/videohub 本地源
    -> 连续编码为 JPEG 帧流事件
    -> WebSocket 推送 /streams/video.ingest

[平台 gateway]
  /streams/video.ingest 接收端
    -> VideoStreamReceiver latest-only 缓存
    -> /streams/video.mjpeg 播放转发
    -> video.frame 状态元数据（含 ingest_status）
    -> PerceptionService 从 receiver latest keyframe 读取 ImageFrame 给 YOLO-World
```

> 说明：本轮在现有依赖内使用 WebSocket 推送 JPEG 事件作为“端侧主动推流”的落地协议；它不是平台采集。浏览器播放仍可通过平台转发 MJPEG 兼容 `<img>`，但 MJPEG 的数据来源必须是 ingest receiver，不是平台抓帧。

## 当前缺口

- [x] Go2 data_plane 内 `_front_video_pull_loop` 仍在同网关进程内读 `VideoClient` 并 callback，语义上仍接近平台内部帧链。
- [x] 平台缺少 video ingest 接收端和 receiver 缓冲。
- [x] `/streams/video.mjpeg` 仍等待 `GatewayRuntime._latest_video_frame`，该缓存由平台内部 callback 写入。
- [x] `PerceptionService.perceive_current_scene()` 仍会在 latest 不可得时调用 `capture_image()`。
- [x] debug panel 仍只知道 `/streams/video.mjpeg`，没有展示推流连接状态。

## 实施任务

- [x] 新增 `services/video/receiver.py`：平台视频接收器，latest-only 存储、等待新帧、状态快照、ImageFrame 输出。
- [x] 新增平台 ingest WebSocket 端点 `/streams/video.ingest`，认证后接收 Go2 推送的 JPEG 帧事件。
- [x] 改造 `/streams/video.mjpeg`：只从 receiver 等新帧并转发；未收到端侧推流时返回 503 或等待状态，不触发采集。
- [x] 改造 `GatewayRuntime`：持有 `VideoStreamReceiver`，启动时发布视频状态刷新，不再订阅 `StreamingImageProvider` 作为视频主链。
- [x] 改造 `PerceptionService`：注入 video frame source；`perceive_current_scene()` 必须从平台接收流 latest/keyframe 取图，不再 fallback 到 `capture_image()`。
- [x] 新增 Go2 端侧推流客户端脚本/模块：从 `VideoClient.GetImageSample()` 本地取图并主动推送到平台 ingest。
- [x] 更新启动脚本：Go2 场景下 gateway 启动后拉起端侧视频推流客户端；停止时清理。
- [x] 更新 debug panel 文案与状态：明确“端侧推流连接/未连接”，不再提示 image provider。
- [x] 更新配置模板：Go2 推流开关、gateway ingest URL、token、FPS。
- [x] 补充针对性测试：receiver、ingest、MJPEG、PerceptionService 不再采集。

## 验收闭环

- [x] 代码搜索：实时视频主链中无 `image_provider.capture_image`。
- [x] 代码搜索：`/streams/video.mjpeg` 不调用 `_publish_video_stream_frame` 或平台采集函数。
- [x] `PerceptionService.perceive_current_scene()` 在没有接收到推流时明确失败，不 fallback 采集。
- [x] 断流或超过 `NUWAX_VIDEO_INGEST_MAX_FRAME_AGE_SEC` 时，平台不得继续把缓存帧标为实时 available。
- [x] Go2 端侧推流客户端主动连接 `/streams/video.ingest`。
- [x] 平台 `video.frame.payload.ingest_status` 显示 connected / last_frame_at / frame_count / fps。
- [x] YOLO-World 输入 `ImageFrame.metadata.source_kind` 来自 `platform_video_ingest_stream`。
