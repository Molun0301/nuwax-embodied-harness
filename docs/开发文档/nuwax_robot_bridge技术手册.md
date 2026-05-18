# 技术手册

- 文档版本：v1.0
- 更新日期：2026-04-26

## 1. 摘要

`nuwax_robot_bridge` 是部署在机器人宿主机上的具身能力运行时，后续会作为 Nuwaxclaw 的具身端能力补充并入整体系统。

Nuwaxclaw 负责上层智能体、会话、规划、多模型调度、桌面交互与 MCP（模型上下文协议）管理；本平台负责机器人连接、实时控制、感知、地图、定位、导航、探索、空间记忆、音频、任务执行、安全状态与事件回传。

平台的核心价值是把底层机器人 SDK（软件开发工具包）、传感器数据和机器人私有动作，整理成上层智能体可以稳定调用的能力接口。

简化后关系如下：

```text
Nuwaxclaw
  ├─ 对话、规划、任务拆解、用户交互
  ├─ MCP 客户端与工具管理
  └─ 调用具身端能力
        ↓
nuwax_robot_bridge
  ├─ MCP / HTTP / SSE / WebSocket / artifacts
  ├─ 能力注册、任务管理、事件总线、状态存储
  ├─ 感知、地图、定位、导航、探索、记忆、音频
  └─ 机器人驱动、传感器输入、运动执行
        ↓
Robot / Sensor / DDS / ROS2 / SDK
```

本平台不是另一个上层智能体系统，也不是单纯的 Go2 控制脚本。它的目标是成为 Nuwaxclaw 面向真实世界的具身能力执行层。

## 2. 项目定位与边界

### 2.1 本平台负责什么

本平台负责以下内容：

1. 机器人连接与装配。
2. 机器人状态采集。
3. 图像、点云、IMU（惯性测量单元）、里程计等传感器输入接入。
4. 运动控制、停止、取消与安全保护。
5. 图像感知、目标检测、分割、跟踪与场景摘要。
6. 地图资产、地图工作区、定位、重定位、导航、探索。
7. 语义记忆、地点记忆、对象实例、观察事件与向量检索。
8. 音频输出与制品管理。
9. 异步任务、进度、取消、超时与事件回传。
10. 对外提供 MCP、HTTP（超文本传输协议）、SSE（服务端事件）、WebSocket（网络套接字）和 artifacts（制品）接口。

### 2.2 本平台不负责什么

本平台不负责以下内容：

1. 不负责上层对话状态。
2. 不负责 ACP（Agent Communication Protocol / 智能体通信协议）客户端体系。
3. 不负责桌面 UI（用户界面）与用户会话。
4. 不负责多模型调度策略。
5. 不负责 Nuwaxclaw 的 MCP 服务器统一管理界面。
6. 不直接暴露原始 SDK（软件开发工具包）调用；机器人动作必须先收口成 `robot_private` 能力后再作为默认 AI（人工智能）能力面。

### 2.3 和 Nuwaxclaw 的职责划分

| 模块 | 职责 | 是否在本平台 |
|---|---|---|
| 用户会话 | 用户输入、上下文、历史记录 | 否，属于 Nuwaxclaw |
| 智能体规划 | 任务拆解、工具选择、推理 | 否，属于 Nuwaxclaw |
| MCP 客户端管理 | MCP 服务器发现、授权、启停 | 否，属于 Nuwaxclaw |
| 机器人能力服务 | 导航、探索、感知、记忆、运动 | 是，属于本平台 |
| 机器人驱动 | SDK、DDS、传感器、运动控制 | 是，属于本平台 |
| 事件与状态反馈 | 任务进度、运行时状态、制品引用 | 是，属于本平台 |
| 部署桥接 | 宿主机网关、容器 relay、令牌鉴权 | 是，属于本平台 |

## 3. 总体架构

### 3.1 分层结构

本平台可以分为七层：

```text
┌──────────────────────────────────────────────────────────────┐
│  上层系统：Nuwaxclaw                                          │
│  对话 / 规划 / MCP 客户端 / UI / 用户会话                      │
└───────────────────────────┬──────────────────────────────────┘
                            │ MCP / HTTP / SSE / WebSocket
┌───────────────────────────▼──────────────────────────────────┐
│  对外网关层 gateways/                                         │
│  app / mcp / http / ws / relay / auth / artifacts              │
└───────────────────────────┬──────────────────────────────────┘
                            │ invoke_capability / invoke_tool
┌───────────────────────────▼──────────────────────────────────┐
│  平台运行时层 gateways/runtime.py + core/                     │
│  CapabilityRegistry / TaskManager / EventBus / StateStore      │
└───────────────────────────┬──────────────────────────────────┘
                            │ 服务编排
┌───────────────────────────▼──────────────────────────────────┐
│  能力服务层 services/                                         │
│  perception / memory / mapping / localization / navigation     │
│  audio / artifact / observation / robot_state                  │
└───────────────────────────┬──────────────────────────────────┘
                            │ 标准 provider 契约
┌───────────────────────────▼──────────────────────────────────┐
│  Provider 接口层 providers/                                   │
│  image / motion / state / safety / maps / localization         │
│  navigation / exploration / pointcloud                         │
└───────────────────────────┬──────────────────────────────────┘
                            │ 机器人装配
┌───────────────────────────▼──────────────────────────────────┐
│  机器人驱动层 drivers/                                        │
│  common / sensor / go2 / g1 / customrobot                      │
└───────────────────────────┬──────────────────────────────────┘
                            │ SDK / DDS / ROS2 / device
┌───────────────────────────▼──────────────────────────────────┐
│  下层设备                                                     │
│  Robot / Camera / LiDAR / IMU / DDS / ROS2 / Nav2              │
└──────────────────────────────────────────────────────────────┘
```

### 3.2 代码目录职责

| 目录 | 职责 |
|---|---|
| `contracts/` | 稳定数据契约，定义能力、任务、事件、图像、地图、导航、记忆、制品等模型 |
| `core/` | 运行时核心，包括能力注册、任务管理、事件总线、状态存储、安全守卫、资源锁 |
| `gateways/` | 对外网关，包括 MCP、HTTP、SSE、WebSocket、relay、鉴权、错误处理、制品访问 |
| `services/` | 平台正式能力，包括感知、记忆、地图、定位、导航、探索、音频、观察、制品 |
| `providers/` | 服务层消费的统一 provider 契约；设备类 provider 由机器人装配提供，地图、定位、导航、探索 provider 由平台几何运行时持有 |
| `drivers/` | 机器人驱动、机器人装配、传感器注册与私有能力 |
| `adapters/` | 外部系统或数据源适配器，例如定位、地图、导航、感知、视频流 |
| `scripts/` | 启动、部署、检查、bootstrap（引导）脚本 |
| `deploy/` | Docker（容器）与 relay 部署示例 |
| `tests/` | 单元、集成和协议级 e2e（端到端）测试 |
| `docs/` | 开发文档、公开文档和架构说明 |

### 3.3 核心运行时对象

| 对象 | 位置 | 职责 |
|---|---|---|
| `GatewayRuntime` | `gateways/runtime.py` | 平台总运行时，连接能力注册、服务层、机器人装配和外部网关 |
| `CapabilityRegistry` | `core/capability_registry.py` | 管理能力定义、输入输出 schema（结构模式）、风险等级、执行模式 |
| `TaskManager` | `core/task_manager.py` | 管理异步任务、进度、取消、超时、资源锁与任务事件 |
| `EventBus` | `core/event_bus.py` | 发布、订阅、回放运行时事件 |
| `StateStore` | `core/state_store.py` | 保存运行时状态快照 |
| `MapWorkspaceService` | `services/mapping/` | 管理地图资产、地图工作区和同名记忆库绑定 |
| `MemoryService` | `services/memory/service.py` | 管理地点、语义、对象实例、观察事件与向量检索 |
| `PerceptionService` | `services/perception/service.py` | 处理图像感知、检测、分割、跟踪和场景摘要 |
| `NavigationService` | `services/navigation/service.py` | 管理导航状态、导航目标、探索状态和导航上下文 |
| `RobotAssemblyBase` | `drivers/common/assembly.py` | 机器人装配基类，定义机器人启动、停止、状态与适配器注册 |
| `RobotManifest` | `drivers/common/manifest.py` | 机器人清单，声明机器人名称、型号、能力矩阵、组件和默认传感器 |

## 4. 对上通信方式

本平台对 Nuwaxclaw 暴露五类接口：

1. MCP：上层智能体默认工具调用入口。
2. HTTP API（应用程序编程接口）：调试、管理、状态查询和直接能力调用入口。
3. SSE：浏览器或服务端订阅事件流。
4. WebSocket：双向事件连接。
5. artifacts：大对象制品下载，例如图片、音频、地图快照、日志片段。

### 4.1 MCP 主入口

MCP 入口为：

```text
POST /mcp
```

协议形式为 JSON-RPC（JSON 远程过程调用）。当前支持：

| 方法 | 说明 |
|---|---|
| `initialize` | 初始化 MCP 会话，返回协议版本、服务信息、事件地址、制品地址 |
| `tools/list` | 返回工具集合 |
| `tools/call` | 调用工具集合中的具体能力 |

当前 MCP 工具被组织为两个集合：

| 集合 | 含义 | 面向对象 |
|---|---|---|
| `platform` | 平台侧正式功能集合 | Nuwaxclaw 智能体默认使用 |
| `device` | 端侧设备功能集合，包含机器人私有与运维调试能力 | 机器人私有动作默认给智能体使用，运维调试能力用于调试场景 |

`platform` 集合承载导航、探索、地图、记忆、感知、音频、任务等平台能力。`device` 集合承载设备级或机器人私有能力，例如 Go2 私有动作、裸图像抓取、关节状态、IMU 状态、原始快照等。


#### 4.1.1 MCP initialize 示例

请求：

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "initialize",
  "params": {}
}
```

响应中会包含：

```json
{
  "protocolVersion": "2025-11-25",
  "serverInfo": {
    "name": "nuwax_robot_bridge",
    "version": "0.9.0"
  },
  "metadata": {
    "events": {
      "websocket_url": ".../events/ws",
      "sse_url": ".../events/stream"
    },
    "artifacts": {
      "base_url": ".../artifacts"
    }
  }
}
```

#### 4.1.2 MCP tools/list 示例

请求：

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/list",
  "params": {}
}
```

响应中会返回 `platform` 和 `device` 两个工具集合，每个集合的 `inputSchema` 中包含 `x-nuwax-function-catalog`，用于列出集合内已注册能力、输入输出 schema、执行模式、风险等级和能力层。

`tools/list` 是上层智能体启动时的稳定工具发现入口，必须快速返回静态目录，不在枚举阶段触发导航、定位、地图、感知或设备后端的实时可用性探测。具体能力是否可执行由 `tools/call` 的正式能力调用链和 `list_capabilities` 的运行时视图检查。

#### 4.1.3 MCP tools/call 示例

调用平台集合里的某个具体功能：

```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "method": "tools/call",
  "params": {
    "name": "platform",
    "arguments": {
      "function": "navigate_to_pose",
      "parameters": {
        "x": 1.2,
        "y": 0.5,
        "yaw": 0.0
      }
    }
  }
}
```

返回结构包括：

```json
{
  "content": [
    {
      "type": "text",
      "text": "..."
    }
  ],
  "structuredContent": {
    "...": "..."
  }
}
```

若能力是异步任务，`structuredContent` 中通常会包含 `task_id`，上层应继续通过任务接口或事件流追踪执行状态。

### 4.2 HTTP API

HTTP API 主要用于：

1. 健康检查。
2. 能力清单查询。
3. 直接能力调用。
4. 任务查询与取消。
5. 状态、观测、感知、地图、导航、记忆查询。
6. 运维、安全策略和部署布局查看。

所有 HTTP 接口都需要认证。

令牌传递方式：

```text
Authorization: Bearer <token>
```

也支持 query 参数：

```text
?access_token=<token>
```

但推荐使用 Authorization header（认证头）。

#### 4.2.1 基础接口

| 接口 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 返回网关根信息，包括机器人名称、型号和主要路径 |
| `/ops` | GET | 运维 HTML 页面 |
| `/api/health` | GET | 健康摘要 |
| `/api/capabilities` | GET | 能力清单 |
| `/api/tools` | GET | 工具视图；普通 agent 只返回默认暴露能力，admin 可见全部 |
| `/api/security/policy` | GET | 安全策略摘要，不返回令牌明文 |
| `/api/deployment/layout` | GET | 部署目录布局 |

#### 4.2.2 能力调用接口

```text
POST /api/capabilities/{capability_name}/invoke
```

请求体：

```json
{
  "arguments": {
    "key": "value"
  }
}
```

示例：

```bash
curl -X POST http://127.0.0.1:8766/api/capabilities/navigate_to_pose/invoke \
  -H 'Authorization: Bearer agent-robot-bridge-token' \
  -H 'Content-Type: application/json' \
  -d '{"arguments":{"x":1.2,"y":0.5,"yaw":0.0}}'
```

### 4.3 任务接口

| 接口 | 方法 | 说明 |
|---|---|---|
| `/api/tasks` | GET | 查询活动任务和历史任务 |
| `/api/tasks/{task_id}` | GET | 查询单个任务状态 |
| `/api/tasks/{task_id}/cancel` | POST | 取消任务 |

任务状态由 `TaskManager` 统一管理，典型状态包括：

1. `accepted`：任务已接受。
2. `running`：任务执行中。
3. `progress`：任务进度更新。
4. `succeeded`：任务成功。
5. `failed`：任务失败。
6. `cancelled`：任务被取消。
7. `timeout`：任务超时。

Nuwaxclaw 调用长时能力时，不应只依赖同步返回，应以 `task_id + 事件流 + 最终状态` 作为完整闭环。

### 4.4 状态、观测和感知接口

| 接口 | 方法 | 说明 |
|---|---|---|
| `/api/state/latest` | GET | 最新机器人状态 |
| `/api/state/history` | GET | 机器人状态历史 |
| `/api/state/diagnostics` | GET | 状态诊断信息 |
| `/api/observations/latest` | GET | 最新观察上下文 |
| `/api/observations/history` | GET | 观察历史 |
| `/api/perception/latest` | GET | 最新感知上下文 |
| `/api/perception/history` | GET | 感知历史 |
| `/api/perception/runtime` | GET | 持续感知运行时状态 |
| `/api/perception/runtime/start` | POST | 启动持续感知运行时 |
| `/api/perception/runtime/stop` | POST | 停止持续感知运行时 |
| `/streams/video.ingest` | WebSocket | Go2 端侧主动推送 JPEG 视频帧；平台只接收，不采集 |
| `/streams/video.mjpeg` | GET | 平台把已接收的端侧推流转发为 MJPEG，供监控页播放 |

状态接口适合 Nuwaxclaw 在 UI 或调试面展示机器人现状。智能体规划时应优先使用平台能力返回的结构化结果，而不是直接读取裸状态。

实时视频与持续感知的正式主链是：Go2 端侧 `go2_video_streamer` 读取本机 videohub 后主动推送 `/streams/video.ingest`，平台 `VideoStreamReceiver` 维护 latest-only 缓冲，监控页读取 `/streams/video.mjpeg`，YOLO-World 只从该缓冲抽 latest/keyframe。`capture_image` 仅是显式设备调试/单次快照能力，不参与监控视频或持续感知。

### 4.5 地图、定位、导航接口

| 接口 | 方法 | 说明 |
|---|---|---|
| `/api/localization/latest` | GET | 最新定位快照 |
| `/api/localization/session` | GET | 当前定位会话 |
| `/api/localization/relocalize` | POST | 对当前激活地图执行重定位 |
| `/api/localization/history` | GET | 定位历史 |
| `/api/maps/latest` | GET | 最新地图快照 |
| `/api/maps/history` | GET | 地图快照历史 |
| `/api/maps/catalog` | GET | 地图资产目录和当前激活工作区 |
| `/api/maps/catalog` | POST | 创建地图资产 |
| `/api/maps/catalog/{map_name}` | DELETE | 删除地图资产 |
| `/api/maps/activate` | POST | 激活地图工作区 |
| `/api/maps/active` | GET | 查询当前激活地图工作区 |
| `/api/maps/catalog/{map_name}/versions` | GET | 查询指定地图版本 |
| `/api/maps/versions/save` | POST | 保存地图版本 |
| `/api/maps/versions/load` | POST | 加载地图版本 |
| `/api/navigation/latest` | GET | 最新导航与探索上下文 |
| `/api/navigation/semantic_context` | GET | 当前地图下语义导航上下文 |
| `/api/navigation/history` | GET | 导航和探索历史 |

实时地图、定位、导航和探索状态应优先通过 `StreamBus` 对外发布，并由 SSE 或 WebSocket 订阅。`/api/*/latest` 与 `/api/*/history` 只用于读取最近快照或兼容迁移提示，不应作为驱动后台控制循环或伪装实时状态的轮询入口。

地图与记忆采用“一图一库”原则。上层执行导航、探索和语义找物前，应先确保已激活正确地图工作区。

推荐流程：

```text
创建或选择地图
  → 激活地图工作区
  → 定位 / 重定位
  → 执行导航、探索、找物或语义查询
  → 保存地图版本与记忆结果
```

### 4.6 记忆接口

| 接口 | 方法 | 说明 |
|---|---|---|
| `/api/memory/summary` | GET | 记忆服务摘要 |
| `/api/memory/libraries` | GET | 记忆库列表 |
| `/api/memory/libraries` | POST | 创建记忆库 |
| `/api/memory/libraries/{library_name}` | DELETE | 删除记忆库 |
| `/api/memory/locations` | GET | 地点记忆列表 |
| `/api/memory/semantic` | GET | 语义记忆列表 |

记忆库接口保留兼容意义。新的主线应以地图工作区为主入口，因为地图和记忆库应同名绑定。

### 4.7 事件流接口

事件流用于让 Nuwaxclaw 实时知道任务状态和运行时变化。

#### SSE

```text
GET /events/stream
```

支持参数：

| 参数 | 说明 |
|---|---|
| `after_cursor` | 从指定事件游标之后回放 |
| `categories` | 按事件类别过滤，逗号分隔 |
| `event_types` | 按事件类型过滤，逗号分隔 |
| `max_events` | 限制最多返回事件数 |

SSE 返回格式：

```text
id: 123
event: runtime.event
data: {...}
```

#### WebSocket

```text
WS /events/ws
```

支持参数：

| 参数 | 说明 |
|---|---|
| `after_cursor` | 连接后先回放历史事件 |
| `categories` | 按类别过滤 |
| `event_types` | 按事件类型过滤 |

WebSocket 消息示例：

```json
{
  "type": "event",
  "event": {
    "cursor": 123,
    "event_type": "task.succeeded",
    "category": "task",
    "payload": {}
  }
}
```

若无事件，会周期性发送 keepalive（保活）消息。

### 4.8 制品接口

制品用于承载不适合直接塞进 JSON 的大对象，例如图片、音频、地图快照、日志片段等。

| 接口 | 方法 | 说明 |
|---|---|---|
| `/api/artifacts/summary` | GET | 制品存储摘要 |
| `/artifacts/{artifact_id}/meta` | GET | 制品元数据 |
| `/artifacts/{artifact_id}` | GET | 下载制品原始文件 |

制品元数据中包含：

1. `artifact_id`
2. `kind`
3. `mime_type`
4. `size_bytes`
5. `uri`
6. `sha256`
7. `metadata`

Nuwaxclaw 在展示图片、音频、地图或日志时，应保存并使用 `artifact_id` 或 `uri`，不要把大对象内联进会话上下文。

## 5. 能力模型

### 5.1 能力描述

每个能力由 `CapabilityDescriptor` 描述，核心字段包括：

1. 能力名。
2. 显示名。
3. 描述。
4. 输入 schema。
5. 输出 schema。
6. 执行模式。
7. 风险等级。
8. 是否默认暴露给 agent。
9. tags 和 metadata。

能力执行模式通常分为：

| 模式 | 含义 |
|---|---|
| 同步 | 调用后立即返回结果 |
| 异步 | 调用后返回 `task_id`，后续通过任务接口和事件流跟踪 |

风险等级用于鉴权：

| 风险等级 | 默认访问策略 |
|---|---|
| low | agent/admin 可调用 |
| medium | agent/admin 可调用 |
| high | 仅 agent allowlist 或 admin 可调用 |
| admin | 仅 admin 可调用 |

### 5.2 能力分层

平台能力分三层：

| 层 | 含义 | 默认是否给智能体 |
|---|---|---|
| `platform` | 平台正式能力，例如导航、探索、地图、记忆、感知摘要 | 是 |
| `robot_private` | 机器人私有能力，例如 Go2 姿态、音量、控制模式、避障开关 | 是 |
| `ops_debug` | 运维调试能力，例如裸图像、裸关节、IMU、定位/地图/导航快照 | 否 |

### 5.3 典型平台能力

下列能力名称以当前项目实现为基准，实际清单应以 `/api/tools` 或 MCP `tools/list` 返回结果为准。

| 能力类别 | 典型能力 |
|---|---|
| 任务 | 取消任务、查询任务、停止运动 |
| 地图 | 创建地图、激活地图、保存地图版本、加载地图版本、列出地图 |
| 定位 | 查询定位、重定位当前地图 |
| 导航 | 导航到坐标、导航到命名地点、刷新导航状态 |
| 探索 | 探索区域、前沿探索、探索命中停机 |
| 感知 | 描述场景、处理图像、持续感知运行时、端侧视频推流接收 |
| 记忆 | 创建/启用记忆库、标记地点、写入语义记忆、查询语义记忆 |
| 找物 | 根据语义记忆和当前感知查找目标 |
| 音频 | 文本转语音播报 |
| 状态 | 查询运行时健康、能力和上下文 |

### 5.4 调用闭环

推荐 Nuwaxclaw 对长时能力采用如下闭环：

```text
1. MCP tools/call 或 HTTP invoke 发起能力调用
2. 返回同步结果或 task_id
3. 如果返回 task_id，订阅 /events/stream 或 /events/ws
4. 根据 task.started / task.progress / task.succeeded / task.failed 更新 UI 和智能体状态
5. 如需取消，调用 /api/tasks/{task_id}/cancel 或平台 cancel_task 能力
6. 任务完成后读取 final result、artifacts、memory、map、navigation context
```

不要把“HTTP 调用返回 200”当作任务完成。对导航、探索、感知运行时、找物等长时能力，应以任务终态事件作为完成依据。

## 6. 下层机器人接入

### 6.1 统一入口

机器人入口由根配置中的 `NUWAX_ACTIVE_ROBOT` 决定。

当前支持注册项：

| 名称 | 说明 |
|---|---|
| `go2` | Go2 正式参考实现 |
| `g1` | G1 机器人入口骨架 |
| `customrobot` | 自定义机器人入口骨架 |

加载流程：

```text
load_config()
  → 读取 NUWAX_ACTIVE_ROBOT
  → drivers.entry.get_active_robot_registration()
  → drivers.registry.ROBOT_DRIVER_REGISTRY
  → import 对应 drivers/<robot_name>
  → 调用 create_<robot_name>_assembly(config)
  → create_default_gateway_runtime(config, robot)
```

关键文件：

| 文件 | 说明 |
|---|---|
| `drivers/entry.py` | 当前激活机器人统一入口 |
| `drivers/registry.py` | 机器人注册表 |
| `drivers/common/assembly.py` | 机器人装配基类 |
| `drivers/common/manifest.py` | 机器人 manifest（清单）契约 |
| `drivers/<robot_name>/assembly.py` | 具体机器人装配逻辑 |
| `drivers/<robot_name>/providers.py` | 具体机器人设备类 provider 实现 |
| `drivers/<robot_name>/private_capabilities.py` | 机器人私有能力 |

### 6.2 RobotManifest

每个机器人应提供 `RobotManifest`，用于声明：

1. `robot_name`：机器人名称。
2. `robot_model`：机器人型号。
3. `entrypoint`：装配入口。
4. `description`：说明。
5. `capability_matrix`：能力矩阵。
6. `required_components`：必需组件。
7. `optional_components`：可选组件。
8. `default_sensors`：默认传感器。
9. `default_audio_backends`：默认音频后端。
10. `default_adapters`：默认适配器。

Nuwaxclaw 合并后，可以把 manifest 作为机器人能力发现的基础元数据来源。

### 6.3 RobotAssemblyBase

所有机器人装配体应遵守 `RobotAssemblyBase`：

| 方法 | 说明 |
|---|---|
| `start()` | 启动机器人入口 |
| `stop()` | 停止机器人入口 |
| `get_status()` | 返回装配状态 |
| `bind_adapter()` | 绑定外部适配器 |
| `initialize_registered_adapters()` | 初始化所有适配器 |
| `stop_registered_adapters()` | 停止所有适配器 |
| `get_adapter_health_statuses()` | 汇总适配器健康状态 |

装配体应负责把机器人 SDK、DDS、传感器、控制器和 providers 组织成平台可消费的接口集合。

### 6.4 Provider 边界

机器人驱动不应直接把私有 SDK 暴露给平台服务层，而应只实现设备侧必要 provider 或窄数据面。地图、定位、导航、探索主链已经收口到 `PlatformGeometryRuntime`，由平台几何运行时对服务层实现 `LocalizationProvider`、`MapProvider`、`NavigationProvider`、`ExplorationProvider`；其背后可以消费 RTAB-Map/Nav2 侧车、平台地图工作区、Go2 DDS 位姿/点云或外部适配器。

端侧典型 provider：

| Provider | 职责 |
|---|---|
| `ImageProvider` | 抓取图像、提供相机参数 |
| `PointCloudProvider` | 提供点云帧 |
| `StateProvider` | 提供机器人状态、关节状态、IMU 状态 |
| `MotionControl` | 下发速度、停止、取消运动 |
| `SafetyProvider` | 提供安全状态 |

平台侧几何 provider：

| Provider | owner | 职责 |
|---|---|---|
| `LocalizationProvider` | `PlatformGeometryRuntime.providers` | 以 ROS2 TF 中 `map -> odom -> base` 解析出的平台定位 pose 为正式输出；Go2 DDS 位姿只作为受控 odom 输入，不作为机器人端侧 provider 要求 |
| `MapProvider` | `PlatformGeometryRuntime.providers` | 汇总平台地图工作区、实时占据栅格、代价图和语义地图，不作为机器人端侧 provider 要求 |
| `NavigationProvider` | `PlatformGeometryRuntime.providers` | 承载平台导航目标、取消和状态语义；正式执行链唯一为 Nav2 action，`navigation_backend_policy` 必须标明 `execution_chain=nav2_action_only`；已废弃的 `platform_grid` 只允许作为删除诊断出现，不得作为正式后端、回退路径或机器人端侧 provider 要求 |
| `ExplorationProvider` | `PlatformGeometryRuntime.providers` | 承载平台探索任务、命中停机和探索状态语义；探索请求接受后由平台后台循环自主推进 frontier，frontier 目标交给 Nav2 action 执行，不把状态查询当作探索节拍来源，不作为机器人端侧 provider 要求 |

平台正式能力应依赖 provider 契约，而不是依赖某个机器人目录的具体实现；但 provider 契约的 owner 不等同于机器人端侧实现者。

### 6.5 平台 Geometry 默认启动链路

全量部署的默认主链是 `PlatformGeometryRuntime` 管理的 geometry + RTAB-Map + Nav2 sidecar（侧车）闭环：

```text
scripts/start_gateway.sh
  → load_config()
  → GatewayRuntime.start()
  → PlatformGeometryRuntime.start()
  → Ros2OdometryPublisher / Ros2PointCloudPublisher / Ros2LiveMapProvider / Ros2TfLocalizationProvider
  → odom_tf_bridge + static TF + RTAB-Map
  → TF 反哺 LocalizationService / NavigationService
  → Nav2 sidecar
```

正式默认配置为：

| 配置项 | 默认值 | 含义 |
|---|---|---|
| `NUWAX_GEOMETRY_ENABLED` | `true` | 默认启用平台 geometry 主链 |
| `NUWAX_GEOMETRY_REQUIRE_ROS2` | `true` | ROS2 不可用时视为启动失败，而不是静默降级 |
| `NUWAX_NAV2_MODE` | `sidecar` | 由平台启动 Nav2 侧车并暴露 action/lifecycle/costmap 状态 |
| `NUWAX_GEOMETRY_ODOM_TOPIC` | `/nuwax/geometry/odom` | 平台受控 odom 话题，供 RTAB-Map/Nav2 使用 |

`NUWAX_GEOMETRY_ODOM_TOPIC` 不应配置为 `/utlidar/robot_odom`。该话题来自裸 DDS（数据分发服务）发布者，不是平台受控 ROS2（机器人操作系统 2）节点；正式链路应由 Go2 DDS 位姿进入 raw odom 输入，再由 `Ros2OdometryPublisher` 发布 `/nuwax/geometry/odom`，供 `odom_tf_bridge`、RTAB-Map 和 Nav2 消费；平台 `LocalizationService`、导航和探索读取 `Ros2TfLocalizationProvider` 从 `/tf` / `/tf_static` 解析出的 `map` 坐标系 pose。

`scripts/bootstrap.py apply --tier full --geometry` 会写入上述默认主链；`--no-geometry` 会显式写入 `NUWAX_GEOMETRY_ENABLED=false`、`NUWAX_GEOMETRY_REQUIRE_ROS2=false`、`NUWAX_NAV2_MODE=disabled`。因此轻量部署必须显式关闭 geometry，不能依赖默认静默关闭。

`services/geometry/launch_ros2_stack.sh` 只负责准备 ROS2 环境并执行传入命令，不直接编排 RTAB-Map/Nav2；RTAB-Map/Nav2 的正式编排由 `PlatformGeometryRuntime` 承担。历史手工入口 `services/geometry/start_rtabmap_sidecar.sh` 也默认使用 `NUWAX_NAV2_MODE=sidecar`，避免手工启动时回退到 external。

### 6.6 接入新机器人流程

新增一个机器人，例如 `myrobot`，推荐流程如下：

#### 第一步：创建目录

```text
drivers/myrobot/
  __init__.py
  manifest.py
  assembly.py
  providers.py
  private_capabilities.py
```

#### 第二步：实现 manifest

在 `manifest.py` 中声明机器人名称、型号、能力矩阵和组件。

#### 第三步：实现 assembly

在 `assembly.py` 中实现：

1. `MyRobotAssembly`。
2. `start()`。
3. `stop()`。
4. `get_status()`。
5. `create_myrobot_assembly(config, iface=None)`。

#### 第四步：实现 providers

根据机器人实际能力选择性实现设备类 provider 或窄数据面：

1. 图像 provider。
2. 状态 provider。
3. 运动 provider。
4. 安全 provider。
5. 点云、里程计、IMU 等输入流。

最小可运行机器人至少建议提供：

1. `StateProvider`
2. `MotionControl`
3. `SafetyProvider`

如果要接入地图、定位、导航和探索，不应在 `drivers/<robot_name>/providers.py` 中把 `MapProvider`、`NavigationProvider`、`ExplorationProvider`、`LocalizationProvider` 当作端侧主链重新实现；应提供必要的位姿、点云、运动执行输入，或配置平台 geometry 工作空间、ROS2 侧车和适配器，由 `PlatformGeometryRuntime` 持有并对服务层暴露这些 provider。

#### 第五步：实现 private_capabilities

机器人私有能力应只放在 `drivers/myrobot/private_capabilities.py` 中，并通过 `ROBOT_PRIVATE_CAPABILITY_SPECS` 暴露给当前机器人装配。

私有能力默认应暴露给 agent。若某个机器人私有能力确实不应进入默认工具面，必须在对应 `PrivateCapabilitySpec` 中显式设置 `exposed_to_agent=False`，不要依赖全局隐藏策略。

#### 第六步：注册机器人

在 `drivers/registry.py` 增加：

```python
"myrobot": RobotDriverRegistration(
    robot_name="myrobot",
    package="drivers.myrobot",
    factory_name="create_myrobot_assembly",
    private_capabilities_module="drivers.myrobot.private_capabilities",
    description="MyRobot 机器人实现。",
)
```

#### 第七步：配置启用

在 `.env` 中配置：

```dotenv
NUWAX_ACTIVE_ROBOT=myrobot
```

#### 第八步：验证

启动网关：

```bash
bash scripts/start_gateway.sh
```

验证：

```bash
curl http://127.0.0.1:8766/api/health \
  -H 'Authorization: Bearer agent-robot-bridge-token'

curl http://127.0.0.1:8766/api/tools \
  -H 'Authorization: Bearer agent-robot-bridge-token'
```
