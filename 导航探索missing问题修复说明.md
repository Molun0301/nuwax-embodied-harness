# 导航探索 missing 问题修复说明

更新时间：2026-05-06

## 背景

启动网关后，`/debug` 页面里地图和定位都能正常显示 `available`，但任务态势面板中的“导航”和“探索”长期显示 `missing`。日志里可以看到 `mapping.snapshot_ready`、`localization.snapshot_ready` 高频刷新，说明 ROS2 地图和 TF 定位链路本身已经在工作，问题集中在导航/探索状态帧如何被发布、缓存和回放给前端。

修复后的状态是：

- `navigation.state` 帧状态为 `available`。
- `exploration.state` 帧状态为 `available`。
- 导航业务状态可能仍为 `failed`，原因是 Nav2 readiness 仍可能报 `Nav2 lifecycle 未全部 active`。这不是 missing 问题，而是下一层 Nav2 生命周期读取/判定问题。
- 页面顶部或聚合卡片中的 `mission: missing` 可能仍出现，它还会受当前任务、3D 点云地图、日志等其他链路影响，不能直接等同于导航/探索 missing。

## 现象复盘

前端通过 WebSocket 订阅：

```text
robot.state,map.frame,localization.pose,localization.frame_tree,localization.trajectory,
navigation.state,exploration.state,exploration.frontier,runtime.governance,
pointcloud_map.meta,pointcloud.frame,video.frame,logs.entry,tasks.state
```

问题发生时，`map.frame` 和 `localization.pose` 会持续高频发布，但 `navigation.state` 和 `exploration.state` 在页面刷新后经常拿不到最近状态帧，因此前端按缺帧处理为 `missing`。

## 根因

### 1. StreamBus retained 窗口被高频帧挤满

`StreamBus` 原本主要靠 `_frames` 保留最近一段历史帧。地图和定位帧频率很高，启动早期发布过的 `navigation.state` / `exploration.state` 很快会被挤出 retained 窗口。

当浏览器较晚连接，或者用旧的 `after_cursor` 重连时，服务端只从 retained 历史里回放帧。此时导航/探索的最新语义状态虽然曾经存在，但历史窗口里已经没有它们，前端就只能看到 missing。

### 2. 旧 after_cursor 会请求一个已经过期的历史区间

前端重连时会带 `after_cursor`。如果这个 cursor 来自旧网关进程，或者已经早于当前 retained 窗口最老 cursor，原逻辑仍按普通 replay 处理，结果可能回放不到某些低频 topic。

导航和探索正好是低频状态 topic，最容易被这种策略误伤。

### 3. 导航 provider 存在但未 ready 时，被当成“不可用”

导航服务以前在刷新导航状态时使用 ready gating 获取 provider。Nav2 action provider 已经存在，但如果 readiness 里 lifecycle 未通过，就可能不发布有诊断价值的状态帧。

这会让“后端存在但暂不可执行”的情况，在 debug 页面上表现成 `missing`，不利于排查。更合理的表现应该是：帧状态 `available`，业务状态放在 payload/metadata 里，例如 `navigation_status=failed`。

### 4. 导航/探索状态缺少周期刷新

地图和定位有自己的持续流，导航/探索如果没有任务变化，状态刷新频率很低。启动时如果首帧丢失，后续前端可能长时间等不到新的导航/探索帧。

### 5. WebSocket 初始回放过大时容易产生断开异常

旧逻辑在某些连接路径下会回放 retained backlog。高频帧较多时，客户端断开可能导致 ASGI 日志里出现 `WebSocketDisconnect` traceback。这不是 missing 的根因，但会干扰判断。

## 修复内容

### 1. 为 StreamBus 增加按 topic 取最新帧能力

文件：`core/stream_bus.py`

新增：

- `oldest_cursor()`：返回当前 retained 窗口最老 cursor。
- `latest_frames(topics=None, limit=None)`：从 `_latest_by_topic` 里取每个 topic 的最新帧，即使该帧已经被 retained 历史窗口淘汰。

这样低频 topic 不再依赖高频历史窗口存活。

### 2. WebSocket/SSE 初始回放改为“最新快照优先”

文件：`gateways/ws/data_streams.py`

新增 `_replay_stream_frames(...)`：

- 如果 `after_cursor <= 0`，说明是新连接，直接回放订阅 topic 的最新快照。
- 如果 `after_cursor` 早于 retained 窗口最老 cursor，说明客户端 cursor 已陈旧，也回放最新快照。
- 只有 cursor 仍在 retained 窗口内时，才使用普通增量 replay。

这保证页面刷新或旧连接恢复时，至少能拿到每个 topic 的最新状态。

同时，WebSocket 初始回放阶段捕获 `WebSocketDisconnect`，避免客户端断开时把正常断连刷成 ASGI 错误栈。

### 3. 网关启动后周期刷新导航/探索状态流

文件：`gateways/runtime.py`

新增导航状态刷新线程：

- 启动流源时立即调用 `refresh_navigation()` 和 `refresh_exploration()` 发布 bootstrap 帧。
- 后台每 2 秒刷新一次导航/探索状态。
- 停止数据流源时同步停止刷新线程。

这样即使前端错过启动首帧，也能很快拿到下一帧，不会长期停留在 missing。

### 4. 导航服务区分“帧可观测”和“业务可执行”

文件：`services/navigation/service.py`

调整：

- `refresh_navigation()` 使用 `_find_navigation_provider()`，允许 provider 存在但未 ready 时仍返回诊断状态。
- 发布 `navigation.state` 时，stream frame 的 `status` 固定为 `available`。
- 真实导航业务状态写入 `metadata.navigation_status`，例如 `failed`、`running`、`succeeded`。
- 发布 `exploration.state` 时同理，stream frame 的 `status` 固定为 `available`，真实探索状态写入 `metadata.exploration_status`。

修复后，前端可以区分：

- `frame.status=missing`：服务状态帧真的缺失。
- `frame.status=available + navigation_status=failed`：导航服务可观测，但 Nav2 当前不可执行或有诊断错误。

### 5. 放宽 Nav2 lifecycle GetState 等待时间

文件：`services/geometry/nav2_action_provider.py`

将 lifecycle `GetState` 等待时间从 `0.2s` 提升到 `0.75s`，减少 ROS2 服务响应略慢时被误判为 state 为空的概率。

这不是 missing 修复的核心，但能降低 Nav2 readiness 误判概率。

## 验证方式

单元和集成验证：

```bash
python -m pytest \
  tests/unit/test_navigation_services.py \
  tests/unit/test_runtime_core.py::test_stream_bus_latest_frames_survive_retention_window \
  tests/integration/test_gateway_integration.py::test_data_stream_websocket_initial_connect_replays_latest_topic_snapshot \
  tests/integration/test_gateway_integration.py::test_data_stream_websocket_replays_latest_frame_when_cursor_is_from_old_process \
  -q
```

验证结果：

```text
23 passed
```

现场验证：

```bash
python - <<'PY'
import asyncio, json, time
import websockets

async def main():
    uri = "ws://127.0.0.1:8766/streams/ws?topics=navigation.state,exploration.state&access_token=agent-robot-bridge-token"
    frames = {}
    async with websockets.connect(uri, ping_interval=None, open_timeout=20) as ws:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and len(frames) < 2:
            msg = await asyncio.wait_for(ws.recv(), timeout=2)
            data = json.loads(msg)
            if data.get("type") == "stream.frame":
                frame = data["frame"]
                frames[frame["topic"]] = {
                    "status": frame.get("status"),
                    "metadata": frame.get("metadata"),
                }
    print(json.dumps(frames, ensure_ascii=False, indent=2))

asyncio.run(main())
PY
```

期望输出形态：

```json
{
  "exploration.state": {
    "status": "available",
    "metadata": {
      "backend_name": "platform_geometry_runtime",
      "exploration_status": "idle"
    }
  },
  "navigation.state": {
    "status": "available",
    "metadata": {
      "backend_name": "platform_geometry_runtime",
      "navigation_status": "failed"
    }
  }
}
```

## 结论

这次修复的重点不是把 Nav2 强行判定为 ready，而是修正 debug 流的语义：

- 导航/探索状态服务存在且能发布诊断时，页面应显示 `available`。
- Nav2 未 ready、lifecycle 未 active、探索无 frontier 等业务状态，应出现在 payload 或 metadata 中。
- 高低频 topic 混合时，低频 topic 的最新状态不能被高频 map/localization retained backlog 淘汰。

因此，`导航 missing / 探索 missing` 已修复；如果后续要让导航真正可执行，需要继续定位 Nav2 lifecycle `GetState` 在网关进程内返回 `state=null` 的问题。
