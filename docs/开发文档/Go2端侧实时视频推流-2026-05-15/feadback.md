# Go2 端侧实时视频推流改造 — feadback（2026-05-15）

## 用户校正

用户明确要求：关系不能搞错，Go2 端侧要把采集到的图像转成实时推流视频主动推上来，平台实时接收视频；平台不要采集帧，不要采集。

## 设计决策

1. 本轮不再把 `StreamingImageProvider.subscribe_image()` 作为监控视频主链。
2. 新增平台 `VideoStreamReceiver` 作为唯一实时视频接收缓存。
3. Go2 端侧新增 `go2_video_streamer` 主动连接平台 `/streams/video.ingest`。
4. YOLO-World 通过 `VideoStreamReceiver.get_latest_image()` 抽取平台已接收视频流中的 latest/keyframe。
5. `capture_image()` 仅保留为 device ops/debug 单次能力；持续感知与监控视频不得使用。

## 影响面记录

- `_start_data_frame_stream_sources` GitNexus impact：CRITICAL，影响网关启动和多条 HTTP/MCP 流程。处理策略：不改非视频流源行为，只新增视频 receiver 状态刷新并移除旧 image provider 视频订阅。
- `_front_video_pull_loop` impact：LOW。本轮将废弃其作为主链的职责，避免网关内部采集。
- `_publish_image_stream_frame` impact：LOW。本轮不作为视频主链入口。
- `video_mjpeg_stream` impact：LOW。改为 receiver 转发。
- `perceive_current_scene` impact：LOW，直接影响 `describe_current_scene`。改为推流未就绪时明确失败。

## 协议选择

在当前依赖已有 `websockets`、`fastapi/starlette` 的前提下，采用 WebSocket ingest：

- Go2 端侧作为客户端，连接 `/streams/video.ingest?access_token=...`。
- 首条 JSON：`{"type":"video.start", "camera_id":"front_camera", "encoding":"jpeg", "source_kind":"go2_side_stream_push"}`。
- 后续每帧发送二进制 JPEG；平台按收到时间写入 latest-only 缓冲。
- 可选 JSON 心跳/状态事件用于统计。

这是端侧主动推流协议；平台没有任何到 Go2 的采集动作。

## 偏差与边界

- 浏览器播放端本轮可继续使用 `/streams/video.mjpeg`，但它只是平台对已接收推流的转发格式，不是平台采集主链。
- 如果未来接入 H.264/WebRTC，可替换 WebSocket/JPEG ingest 的传输层；平台 `VideoStreamReceiver` 和 YOLO 抽帧语义不变。

## 实现记录

- 新增 `services/video/receiver.py`，平台只维护端侧推上来的视频 latest-only 缓冲，并输出 `ImageFrame.metadata.source_kind=platform_video_ingest_stream`。
- 新增 `/streams/video.ingest` WebSocket ingest 端点：首条 `video.start` JSON，后续二进制 JPEG 帧；平台只接收，不调用机器人采集接口。
- `/streams/video.mjpeg` 已改为只从 `runtime.video_stream_receiver` 等待新帧；没有收到端侧首帧时返回 503，不触发采集。
- `GatewayRuntime` 持有 `VideoStreamReceiver`，视频刷新线程只发布 receiver 状态/metadata，不再订阅 `StreamingImageProvider` 作为实时视频主链。
- `PerceptionService` 已注入 `video_frame_source`；`perceive_current_scene()` 只从 receiver latest 抽帧，推流未就绪时返回 `video_ingest_not_ready`，不再 fallback 到 `capture_image()`。
- `VideoStreamReceiver` 增加 `NUWAX_VIDEO_INGEST_MAX_FRAME_AGE_SEC` 实时性门禁；端侧断流或最近帧过期后，`latest_frame()` / `get_latest_image()` 不再返回可用帧，`video.frame` 不再给出可播放 `stream_url`。
- `PerceptionVideoRuntime` 在关键帧判断前读取 receiver 状态；推流未就绪或过期时只发布 missing 状态并跳过 YOLO-World，不用缓存旧帧冒充实时输入。
- `PerceptionService._publish_video_frame()` 对来自 `platform_video_ingest_stream` 的帧直接发布 MJPEG 元数据，避免持续感知非关键帧刷新把 `video.frame` 覆盖成 unsupported。
- 新增 `drivers/go2/video_streamer.py`：Go2 端侧使用本机 `VideoClient.GetImageSample()` 读取 videohub，并主动推送到平台 `/streams/video.ingest`。
- `scripts/start_gateway.sh` 在 `NUWAX_ACTIVE_ROBOT=go2`、`GO2_DATA_PLANE_ENABLED=true`、`GO2_VIDEO_STREAM_ENABLED=true` 时随 gateway 拉起/清理 Go2 端侧推流进程。
- Go2 `data_plane.py` 已移除旧 `_front_video_pull_loop` 和 image callback 视频主链；数据面继续保留 DDS 位姿、IMU、点云和运动执行链。
- Debug panel 文案改为端侧推流，并展示 `ingest_status.connected/fps/last_client`。

## 复查修正记录

- 实机 `/home/unitree/nuwax-robot-bridge/drivers/go2/data_plane.env` 已存在并启用 `GO2_DATA_PLANE_ENABLED=true`，但旧文件没有 `GO2_VIDEO_STREAM_*`；根 `.env` 含 `NUWAX_ACTIVE_ROBOT=go2`、`NUWAX_GATEWAY_PORT=8766`、`NUWAX_GATEWAY_AGENT_TOKENS=agent-robot-bridge-token`。本轮配置模板已补齐推流变量；实际部署需同步 Go2 实机 env 文件，否则 streamer 只能依赖代码默认值。
- GitNexus 复查：`PerceptionConfig` 结构变更为 HIGH，`_publish_latest_video_stream_metadata` 为 CRITICAL；本轮限定为新增默认配置与断流/过期状态语义，不改变非视频能力路径。

## 验证记录

- `python3 -m py_compile ...` 已覆盖新增/改动的 receiver、runtime、ws、perception、Go2 settings/data_plane/providers/video_streamer 与相关测试文件。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv_model_download/bin/python -m pytest -q tests/unit/test_perception_service.py tests/unit/test_video_stream_receiver.py`：30 passed。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv_model_download/bin/python -m pytest -q tests/unit/test_go2_ros2_workspace.py::test_go2_data_plane_config_defaults_to_direct_dds_topics`：1 passed。
- 系统 `python3 -m pytest` 受当前环境缺少 `pydantic` 阻塞；虚拟环境直接跑 pytest 受 ROS launch_testing 插件缺少 `lark` 阻塞，因此针对性验证使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。
