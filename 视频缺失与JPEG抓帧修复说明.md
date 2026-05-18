# 视频缺失与JPEG抓帧修复说明

更新时间：2026-05-06

## 背景

`/debug` 页面右侧“视频”卡片最初长期显示 `unsupported` 或 `missing`，页面中还能看到类似提示：

```text
video_provider 未返回图像帧。
```

而用户提供的 Go2 SDK 文档已经明确说明：

- Go2 支持通过 `VideoClient.GetImageSample()` 获取 JPEG 拍照结果。
- Go2 也支持通过 UDP 多播方式获取 H264 图传。
- DDS 不适合直接作为正式图传主链。

因此，本次修复目标不是“先把 UDP H264 全量接进来”，而是先利用已有的 JPEG 抓拍能力，把 debug 页面的视频卡片从 `unsupported/missing` 修复为可用状态。

## 现象复盘

### 1. 页面初始状态是 `unsupported`

debug 页面视频区域最开始不是“真的没有图像能力”，而是网关启动阶段主动发布了一帧静态占位数据：

```text
status=unsupported
reason=no_native_video_stream_subscription
message=当前图像接口没有原生视频流订阅，未启用定时抓帧伪流式。
```

这导致页面还没真正尝试抓图，就已经把 `video.frame` 定义成了“不支持”。

### 2. 接入 JPEG 抓拍后，第一次重启又在启动时崩溃

第一次把 `capture_image()` 接进 `video.frame` 后，网关重启时报错：

```text
UnicodeDecodeError: 'utf-8' codec can't decode byte 0xff in position 0: invalid utf-8
```

错误栈定位到：

```text
gateways/runtime.py -> _publish_video_stream_frame()
gateways/runtime.py -> _serialize()
gateways/serialization.py -> to_jsonable()
fastapi.encoders.jsonable_encoder(...)
```

这说明不是抓拍接口本身失败，而是 JPEG 原始二进制在序列化成 JSON payload 时被错误处理了。

## 根因

### 1. 网关把 `video.frame` 写死成静态 `unsupported`

文件：

`gateways/runtime.py`

原始实现中，`_publish_static_stream_capability_frames()` 会直接发布：

```python
self.stream_bus.publish(
    "video.frame",
    source="image_provider",
    payload={"available": False, "message": "当前图像接口没有原生视频流订阅，未启用定时抓帧伪流式。"},
    payload_kind=StreamPayloadKind.UNSUPPORTED,
    status="unsupported",
    metadata={"reason": "no_native_video_stream_subscription"},
)
```

这相当于在启动时就预先认定：

- 平台没有视频流
- 也不会去抓 JPEG

因此 debug 页面只能看到占位态，而不会看到真实视频帧。

### 2. Go2 实际上已经具备 JPEG 抓拍能力

文件：

- `drivers/go2/assembly.py`
- `drivers/go2/providers.py`

已有链路：

```text
Go2ProviderBundle.capture_image()
    -> Go2RobotAssembly.capture_image_bytes()
    -> video_client.GetImageSample()
```

也就是说，平台并不是缺少视频数据源，而只是没有把这条已有 JPEG 抓拍链路发布为 `video.frame`。

### 3. 第一次修复时错误地把 `ImageFrame` 整体做了 JSON 序列化

第一次补 `_publish_video_stream_frame()` 时，代码里先做了：

```python
payload = self._serialize(image_frame)
```

而 `image_frame.data` 里存放的是原始 JPEG 二进制，例如：

```text
0xff 0xd8 0xff ...
```

`fastapi.encoders.jsonable_encoder()` 在处理这种非 UTF-8 字节序列时，会在内部走到 Pydantic 的 `model_dump()`，最后触发：

```text
UnicodeDecodeError
```

这就是启动时直接崩溃的真正根因。

### 4. 当前需求更适合先走 JPEG 抓帧伪流，而不是直接切 UDP H264

用户给出的 SDK 文档说明：

- H264 图传推荐使用 UDP 多播 + GStreamer/OpenCV。
- JPEG 拍照接口更适合直接在应用层快速接入图像能力。

结合当前 debug 需求，“页面能看到实时画面”优先级高于“先做高性能正式图传”。

因此本次修复优先方案是：

- 先把 JPEG 抓拍做成低频伪流式 `video.frame`
- 后续如需低延迟连续图传，再单独接 H264/UDP/GStreamer

## 修复内容

### 1. 新增 `video.frame` 周期抓帧刷新线程

文件：

`gateways/runtime.py`

新增：

```python
_VIDEO_STREAM_REFRESH_INTERVAL_SEC = 1.0 / 3.0
```

并在 `GatewayRuntime` 中新增：

- `_video_stream_refresh_stop`
- `_video_stream_refresh_thread`
- `_start_video_stream_refresh()`
- `_stop_video_stream_refresh()`
- `_run_video_stream_refresh()`

启动网关后，后台会以约 3Hz 周期抓取一帧 JPEG，并发布到：

```text
video.frame
```

### 2. 启动阶段不再发布静态 `unsupported` 占位帧

文件：

`gateways/runtime.py`

把：

```python
_publish_static_stream_capability_frames()
```

中的固定 `unsupported` 发布逻辑，替换为：

```python
self._publish_video_stream_frame(reason="runtime_stream_start")
```

这意味着启动时会立刻尝试抓一帧真实图片，而不是先给前端喂一份“永远不支持”的占位态。

### 3. 新增 `_publish_video_stream_frame()`

文件：

`gateways/runtime.py`

逻辑分支如下：

#### 没有 `ImageProvider`

发布：

```text
status=missing
reason=image_provider_missing
```

#### `ImageProvider` 暂不可用

发布：

```text
status=missing
reason=image_provider_unavailable
```

#### JPEG 抓拍失败

发布：

```text
status=missing
reason=image_capture_failed
message=<具体异常>
```

#### JPEG 抓拍成功

发布：

- `camera_id`
- `frame_id`
- `width_px`
- `height_px`
- `encoding`
- `data_uri`
- `payload_bytes`
- `timestamp`
- `metadata`

并标记：

```text
status=available
source=image_provider
```

### 4. 不再把原始 JPEG `bytes` 放进 `video.frame` payload

这是本次避免启动崩溃的关键。

修复后，`_publish_video_stream_frame()` 只会：

- 读取 `image_frame.data`
- 在本地转成 `data:image/jpeg;base64,...`
- 只把 `data_uri` 和元数据放进 payload

不会再把：

```python
image_frame.data
```

这种原始二进制直接交给 FastAPI / Pydantic 序列化。

### 5. 失败时允许页面显示 `missing`，但不能拖垮整个网关

修复后的策略是：

- 视频抓拍失败只影响 `video.frame`
- 页面视频卡片显示 `missing`
- 网关主进程继续存活

不再允许因为一帧 JPEG 二进制的序列化问题让整个应用在 startup 阶段退出。

## 测试与验证

### 1. 新增红灯测试：视频帧能从 JPEG 抓拍正确发布

文件：

`tests/integration/test_gateway_integration.py`

新增测试：

```text
test_gateway_runtime_publishes_video_frame_from_image_provider
```

测试点：

- `video.frame` 状态为 `available`
- `source=image_provider`
- `payload["data_uri"]` 以 `data:image/jpeg;base64,` 开头
- `payload["payload_bytes"]` 正确
- `payload` 中不再包含原始 `data` 字段

同时测试里故意使用真实 JPEG 头：

```python
b"\xff\xd8\xff\xdbdemo-jpeg"
```

用于复现并防住 `UnicodeDecodeError`。

### 2. 新增红灯测试：抓拍失败时降级为 `missing`

新增测试：

```text
test_gateway_runtime_publishes_missing_video_frame_when_capture_fails
```

测试点：

- `video.frame.status == "missing"`
- `message` 中保留真实错误原因

### 3. 实际修复后测试结果

已验证：

```bash
PYTHONPATH=. pytest \
  tests/integration/test_gateway_integration.py::test_gateway_runtime_publishes_video_frame_from_image_provider \
  tests/integration/test_gateway_integration.py::test_gateway_runtime_publishes_missing_video_frame_when_capture_fails \
  tests/integration/test_gateway_integration.py::test_go2slam_external_frontier_observation_mode_never_dispatches_navigation_goal \
  -q
```

结果：

```text
3 passed
```

并且：

```bash
git diff --check
```

通过。

## 验收方式

### 1. 重启网关

```bash
bash scripts/start_gateway.sh restart
```

### 2. 页面验证

打开：

```text
http://127.0.0.1:8766/debug
```

预期：

- 视频卡片不再默认显示 `unsupported`
- 若 JPEG 抓拍成功，页面显示图像
- 若抓拍失败，页面显示 `missing`，但网关不崩溃

### 3. 接口验证

可直接检查：

```bash
curl -s http://127.0.0.1:8766/api/debug/video/latest
```

预期成功时返回：

- `status=available`
- `data.data_uri` 存在
- `data.payload_bytes` 大于 0

### 4. 启动日志验证

不应再出现：

```text
UnicodeDecodeError: 'utf-8' codec can't decode byte 0xff ...
```

## 后续建议

本次修复优先解决的是：

```text
debug 页面视频不可用
```

并采用的是：

```text
JPEG 抓拍 -> 定时伪流 video.frame
```

这适合当前调试可视化需求，但它不是最终形态的低延迟正式图传。

如果后续需要更连续、更低延迟的视频体验，可以继续扩展为：

- Go2 UDP H264 多播拉流
- GStreamer 解码
- OpenCV / FFmpeg / 浏览器桥接
- 独立的视频 runtime / WebRTC / MJPEG relay

但这些都应作为下一阶段能力，而不是这次 `video.frame missing/unsupported` 问题的最小修复。

## 结论

这次问题的本质不是：

```text
Go2 没有视频能力
```

而是：

```text
平台原本没有把现成的 JPEG 抓拍能力接成 debug 可消费的视频帧流，
第一次接入时又把原始 JPEG bytes 直接放进了 JSON 序列化链路。
```

最终修复方案是：

- 用现有 `VideoClient.GetImageSample()` 做定时 JPEG 抓帧
- 发布为 `video.frame`
- 只传 `data_uri` 和元数据，不传原始 `bytes`
- 视频失败只标记 `missing`，不允许拖垮网关启动

这样可以在不引入 UDP H264/GStreamer 复杂度的前提下，先把 debug 页面的视频能力恢复到可用状态。
