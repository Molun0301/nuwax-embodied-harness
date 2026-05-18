# 健康 missing 问题修复说明

更新时间：2026-05-06

## 背景

`/debug` 页面右侧“运行态总览”里，“健康”卡片显示 `missing`：

```text
健康 missing
时间 --
延迟 --
过期 否
来源 --
```

但同一个页面顶部机器人型号、网关连接、运行模式、急停状态等信息已经能正常刷新。这说明后端网关并不是不可用，问题更可能出在 debug 前端的总览映射逻辑上。

## 结论

这次 `健康 missing` 不是 `/api/health` 真失败，也不是 `robot.state` 缺失。

根因是 debug 页面总览数组里声明了 `health` 项，但实时流 `updateStreamOverview()` 没有任何 topic 会写入 `overview.health`。于是“健康”卡片一直停留在初始 missing 状态。

修复方式是：当实时流收到 `robot.state` 帧时，同时派生一份 `health` 总览摘要，写入 `overview.health`。这样“健康”和“状态”都能跟随机器人状态流变成 `available`。

## 根因细节

### 1. 页面总览包含 health

文件：`gateways/http/debug_panel.py`

前端总览卡片列表里包含：

```javascript
const overviewNames = [
  "health",
  "state",
  "localization",
  "map",
  "navigation",
  "exploration",
  "runtime_governance",
  "pointcloud_map",
  "pointcloud",
  "video",
  "logs"
];
```

因此页面启动时会创建“健康”卡片。

### 2. 实时流没有 health topic

debug 页面当前订阅的是实时 stream topics：

```javascript
const STREAM_TOPICS = [
  "robot.state",
  "map.frame",
  "localization.pose",
  "localization.frame_tree",
  "localization.trajectory",
  "navigation.state",
  "exploration.state",
  "exploration.frontier",
  "runtime.governance",
  "pointcloud_map.meta",
  "pointcloud.frame",
  "video.frame",
  "logs.entry",
  "tasks.state"
];
```

这里没有单独的 `health` 或 `gateway.health` topic。

### 3. 旧映射只把 robot.state 写成 state

旧逻辑中，`updateStreamOverview(frame)` 的映射表类似：

```javascript
const keyByTopic = {
  "robot.state": "state",
  "map.frame": "map",
  "localization.pose": "localization",
  "navigation.state": "navigation",
  "exploration.state": "exploration",
  "runtime.governance": "runtime_governance",
  "pointcloud_map.meta": "pointcloud_map",
  "pointcloud.frame": "pointcloud",
  "video.frame": "video",
  "logs.entry": "logs",
};
```

`robot.state` 只会更新“状态”卡片，不会更新“健康”卡片。`health` 这个总览 key 永远没有数据源，所以一直 missing。

### 4. 顶部健康信息和总览健康卡片不是同一条更新路径

收到 `robot.state` 后，页面会调用 `updateRobotStateFrame(frame)`，它会刷新顶部的：

- 机器人名称
- 型号
- 运行模式
- 电池
- 资源/任务
- 急停状态

但这些只是顶部 banner 的 UI 状态，不会自动填充“运行态总览”里的 `health` 卡片。

这就是为什么顶部看起来正常，而总览“健康”仍然 missing。

## 修复内容

### 1. 新增 healthOverviewFromRobotFrame

文件：`gateways/http/debug_panel.py`

新增函数：

```javascript
function healthOverviewFromRobotFrame(frame) {
  return {
    status: frame.status || "available",
    source_timestamp: frame.timestamp,
    latency_ms: 0,
    stale: frame.status === "stale",
    frame_id: frame.frame_id,
    revision: frame.revision,
    metadata: {
      source: "gateway_runtime",
      derived_from: frame.source || "robot.state",
      runtime_started: true
    },
  };
}
```

它把 `robot.state` 帧转换成“健康”卡片需要的 `DebugSourceSummary` 风格对象。

### 2. robot.state 到来时同步写 overview.health

在 `updateStreamOverview(frame)` 中增加：

```javascript
if (frame.topic === "robot.state") {
  overview.health = healthOverviewFromRobotFrame(frame);
}
```

然后原本逻辑继续写：

```javascript
overview.state = {
  status: frame.status || "available",
  source_timestamp: frame.timestamp,
  latency_ms: 0,
  stale: frame.status === "stale",
  frame_id: frame.frame_id,
  revision: frame.revision,
  metadata,
};
```

因此一帧 `robot.state` 会同时刷新两个总览项：

- `health`：健康卡片，来源显示 `gateway_runtime`。
- `state`：状态卡片，来源显示 `robot_state_service`。

## 为什么不直接调用 /api/health

`/api/health` 仍然存在，但 debug 页面已经迁移到实时 stream 驱动。页面里多数运行态卡片都通过 WebSocket 更新，而不是靠轮询 HTTP 快照。

如果只为“健康”卡片额外轮询 `/api/health`，会让页面存在两套刷新模型：

- 大多数卡片走 WebSocket stream。
- 健康卡片单独走 HTTP polling。

这会增加状态不同步的概率。因此本次选择从已有 `robot.state` 实时帧派生健康卡片，保持 debug 面板一套实时流模型。

## 修复后的预期表现

收到第一帧 `robot.state` 后，“运行态总览”里的健康卡片应从：

```text
健康 missing
时间 --
延迟 --
来源 --
```

变成类似：

```text
健康 available
时间 13:xx:xx
延迟 0 ms
过期 否
来源 gateway_runtime
```

服务矩阵中 `gateway` 行也会从 `missing` 变为 `available`，因为服务矩阵使用的也是 `data.health`。

## 验证方式

### 1. 静态渲染验证

```bash
python - <<'PY'
from gateways.http.debug_panel import _render_debug_page

html = _render_debug_page("agent-token")
assert "healthOverviewFromRobotFrame" in html
assert "overview.health = healthOverviewFromRobotFrame(frame)" in html
print("health overview fix present")
PY
```

期望输出：

```text
health overview fix present
```

### 2. 编译验证

```bash
python -m compileall gateways/http/debug_panel.py
```

期望无语法错误。

### 3. 现场页面验证

重启网关，让 `/debug` 页面加载新的前端代码：

```bash
bash scripts/start_gateway.sh restart
```

打开：

```text
http://127.0.0.1:8766/debug
```

观察“运行态总览”：

- “健康”应显示 `available`。
- 时间应显示最近一次 `robot.state` 帧时间。
- 来源应显示 `gateway_runtime`。

## 和 mission missing 的关系

`mission` 顶部徽标是多个总览项的最差状态聚合，逻辑大致包括：

```javascript
worstStatus([
  data.health,
  data.state,
  data.localization,
  data.map,
  data.navigation,
  data.video,
  data.logs
])
```

所以修复 `health` 后，只能消除健康项造成的 missing。若 `video`、`logs`、任务或其他可选链路仍为 `missing` / `unsupported`，`mission` 仍可能不是纯 available。

这和“健康卡片 missing”是两个层次：

- 健康卡片 missing：本次修复的问题。
- mission missing：聚合状态，可能还受其他服务项影响。

## 结论

健康卡片 missing 的根因是前端总览映射漏填 `health`，而不是网关健康检查失败。

修复后，`robot.state` 实时帧会同时更新：

- 顶部 banner 健康信息。
- 运行态总览中的“状态”卡片。
- 运行态总览中的“健康”卡片。

这样 debug 页面能正确表达：网关和机器人状态流可观测时，“健康”应为 `available`。
