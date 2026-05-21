# nuwax_robot_bridge 技术手册

- 文档版本：v2.0
- 更新日期：2026-05-19
- 适用版本：nuwax_robot_bridge 0.9.x

---

## 目录

1. [摘要](#1-摘要)
2. [项目定位与边界](#2-项目定位与边界)
3. [总体架构](#3-总体架构)
4. [核心运行时](#4-核心运行时)
5. [对外通信接口](#5-对外通信接口)
6. [能力模型](#6-能力模型)
7. [服务层](#7-服务层)
8. [Provider 接口层](#8-provider-接口层)
9. [机器人驱动层](#9-机器人驱动层)
10. [适配器框架](#10-适配器框架)
11. [数据契约层](#11-数据契约层)
12. [配置系统](#12-配置系统)
13. [日志系统](#13-日志系统)
14. [安全与鉴权](#14-安全与鉴权)
15. [部署与运维](#15-部署与运维)
16. [接入新机器人](#16-接入新机器人)
17. [附录](#17-附录)

---

## 1. 摘要

`nuwax_robot_bridge` 是部署在机器人宿主机上的具身能力运行时（Embodied Capability Runtime），后续将作为 Nuwaxclaw 的具身端能力补充并入整体系统。

Nuwaxclaw 负责上层智能体、会话、规划、多模型调度、桌面交互与 MCP（Model Context Protocol / 模型上下文协议）管理；本平台负责机器人连接、实时控制、感知、地图、定位、导航、探索、空间记忆、音频、任务执行、安全状态与事件回传。

平台的核心价值是把底层机器人 SDK（Software Development Kit / 软件开发工具包）、传感器数据和机器人私有动作，整理成上层智能体可以稳定调用的能力接口。

### 系统关系概览

```text
Nuwaxclaw（上层智能体系统）
  ├─ 对话、规划、任务拆解、用户交互
  ├─ MCP 客户端与工具管理
  └─ 调用具身端能力
        ↓ MCP / HTTP / SSE / WebSocket
nuwax_robot_bridge（具身能力运行时）
  ├─ 对外网关：MCP / HTTP / SSE / WebSocket / artifacts
  ├─ 核心运行时：能力注册、任务管理、事件总线、状态存储
  ├─ 服务层：感知、地图、定位、导航、探索、记忆、音频
  ├─ Provider 接口层：设备类 + 平台几何类
  └─ 机器人驱动层：装配、传感器、私有能力
        ↓ SDK / DDS / ROS2 / device
Robot / Camera / LiDAR / IMU / DDS / ROS2 / Nav2
```

### 技术栈

| 层级 | 技术选型 |
|------|----------|
| 语言 | Python 3.8.10（Jetson / Ubuntu 20.04 目标环境） |
| Web 框架 | FastAPI + Uvicorn |
| 数据校验 | Pydantic v2 |
| 机器人中间件 | ROS 2（Humble/Iron/Jazzy） |
| SLAM | RTAB-Map |
| 导航 | Nav2 |
| 感知 | YOLO-World + ByteTrack + OpenAI Vision API |
| 向量记忆 | SQLite + BGE-M3 / CLIP / 通义千问 Embedding |
| 音频 | 豆包 TTS |
| 部署 | systemd / Docker / NuwaClaw Relay |

---

## 2. 项目定位与边界

### 2.1 本平台负责什么

1. 机器人连接与装配（Robot Assembly）。
2. 机器人状态采集与诊断。
3. 图像、点云、IMU（Inertial Measurement Unit / 惯性测量单元）、里程计等传感器输入接入。
4. 运动控制、停止、取消与安全保护。
5. 图像感知、目标检测、语义分割、对象跟踪与场景摘要。
6. 地图资产、地图工作区、版本管理、定位、重定位。
7. 路径规划、导航执行、前沿探索（Frontier Exploration）。
8. 语义记忆、地点记忆、对象实例关联、观察事件、向量检索与场景空间理解。
9. 音频输出（TTS）与制品（Artifact）管理。
10. 异步任务、进度追踪、取消、超时与事件回传。
11. 对外提供 MCP、HTTP、SSE、WebSocket 和 artifacts 接口。

### 2.2 本平台不负责什么

1. 上层对话状态管理。
2. ACP（Agent Communication Protocol / 智能体通信协议）客户端体系。
3. 桌面 UI（User Interface / 用户界面）与用户会话。
4. 多模型调度策略。
5. Nuwaxclaw 的 MCP 服务器统一管理界面。
6. 不直接暴露原始 SDK 调用；机器人动作必须先收口成 `robot_private` 能力后再作为默认 AI 能力面。

### 2.3 与 Nuwaxclaw 的职责划分

| 模块 | 职责 | 归属 |
|------|------|------|
| 用户会话 | 用户输入、上下文、历史记录 | Nuwaxclaw |
| 智能体规划 | 任务拆解、工具选择、推理 | Nuwaxclaw |
| MCP 客户端管理 | MCP 服务器发现、授权、启停 | Nuwaxclaw |
| 机器人能力服务 | 导航、探索、感知、记忆、运动 | 本平台 |
| 机器人驱动 | SDK、DDS、传感器、运动控制 | 本平台 |
| 事件与状态反馈 | 任务进度、运行时状态、制品引用 | 本平台 |
| 部署桥接 | 宿主机网关、容器 relay、令牌鉴权 | 本平台 |

---

## 3. 总体架构

### 3.1 分层结构

本平台采用七层分层架构，自上而下依次为：

```text
┌──────────────────────────────────────────────────────────────────┐
│  Layer 0：上层系统 Nuwaxclaw                                      │
│  对话 / 规划 / MCP 客户端 / UI / 用户会话                          │
└────────────────────────────┬─────────────────────────────────────┘
                             │ MCP / HTTP / SSE / WebSocket
┌────────────────────────────▼─────────────────────────────────────┐
│  Layer 1：对外网关层 gateways/                                     │
│  FastAPI App / MCP Protocol / HTTP API / WebSocket Events         │
│  SSE Stream / Relay Server / Auth / Artifacts / Debug Panel       │
└────────────────────────────┬─────────────────────────────────────┘
                             │ invoke_capability / invoke_tool
┌────────────────────────────▼─────────────────────────────────────┐
│  Layer 2：平台运行时层 gateways/runtime.py + core/                 │
│  GatewayRuntime / CapabilityRegistry / TaskManager                │
│  EventBus / StreamBus / StateStore / SafetyGuard / ResourceLock   │
└────────────────────────────┬─────────────────────────────────────┘
                             │ 服务编排
┌────────────────────────────▼─────────────────────────────────────┐
│  Layer 3：能力服务层 services/                                     │
│  Geometry / Navigation / Perception / Memory / Mapping            │
│  Localization / Audio / Observation / Artifact / Evaluation       │
└────────────────────────────┬─────────────────────────────────────┘
                             │ Provider 契约
┌────────────────────────────▼─────────────────────────────────────┐
│  Layer 4：Provider 接口层 providers/                               │
│  Image / PointCloud / Motion / State / Safety                     │
│  Localization / Map / Navigation / Exploration                    │
└────────────────────────────┬─────────────────────────────────────┘
                             │ 机器人装配
┌────────────────────────────▼─────────────────────────────────────┐
│  Layer 5：机器人驱动层 drivers/                                    │
│  common / sensor / go2 / g1 / customrobot                         │
└────────────────────────────┬─────────────────────────────────────┘
                             │ SDK / DDS / ROS2 / device
┌────────────────────────────▼─────────────────────────────────────┐
│  Layer 6：下层设备                                                 │
│  Robot / Camera / LiDAR / IMU / DDS / ROS2 / Nav2                 │
└──────────────────────────────────────────────────────────────────┘
```

### 3.2 代码目录职责

| 目录 | 职责 | 规模 |
|------|------|------|
| `contracts/` | 稳定数据契约，定义能力、任务、事件、图像、地图、导航、记忆、制品等模型 | 24 个契约文件 |
| `core/` | 运行时核心基础设施 | 8 个模块 |
| `gateways/` | 对外网关，包括 MCP、HTTP、SSE、WebSocket、relay、鉴权、制品访问 | ~14,000 行 |
| `services/` | 平台正式能力服务 | ~50,000 行 |
| `providers/` | 服务层消费的统一 Provider 契约 | 9 个 Provider 接口 |
| `drivers/` | 机器人驱动、装配、传感器注册与私有能力 | go2/g1/customrobot |
| `adapters/` | 外部系统或数据源适配器 | 5 个适配器类别 |
| `skills/` | 技能注册与描述 | 技能元数据 |
| `scripts/` | 启动、部署、检查、引导脚本 | 5 个脚本 |
| `deploy/` | Docker、systemd 与 relay 部署配置 | 模板与示例 |
| `tests/` | 单元、集成和协议级端到端测试 | 本地验证用 |
| `docs/` | 开发文档与公开文档 | — |

### 3.3 核心运行时对象

| 对象 | 位置 | 职责 |
|------|------|------|
| `GatewayRuntime` | `gateways/runtime.py` | 平台总运行时，连接能力注册、服务层、机器人装配和外部网关 |
| `CapabilityRegistry` | `core/capability_registry.py` | 管理能力定义、输入输出 schema、风险等级、执行模式 |
| `TaskManager` | `core/task_manager.py` | 管理异步任务、进度、取消、超时、资源锁与任务事件 |
| `EventBus` | `core/event_bus.py` | 发布、订阅、回放运行时事件（游标续播） |
| `StreamBus` | `core/stream_bus.py` | 实时数据流发布、订阅与游标续播 |
| `StateStore` | `core/state_store.py` | 线程安全的运行时状态快照缓存 |
| `SafetyGuard` | `core/safety_guard.py` | 安全策略执行与运动保护 |
| `ResourceLockManager` | `core/resource_lock.py` | 资源互斥锁管理 |

---

## 4. 核心运行时

### 4.1 GatewayRuntime

`GatewayRuntime`（`gateways/runtime.py`，约 7,500 行）是平台的总编排器，负责：

- 创建并管理所有服务实例（Geometry、Navigation、Perception、Memory、Mapping、Localization、Audio、Observation、RobotState、Artifact、Evaluation）
- 加载机器人装配体（Robot Assembly）
- 注册平台能力和机器人私有能力
- 处理能力调用请求（`invoke_capability`、`invoke_tool`）
- 管理请求上下文（request_id、task_id、capability_name 等关联追踪）
- 提供运行时健康摘要和治理信息

#### 启动流程

```text
create_gateway_app_from_env()
  → load_config()
  → create_active_robot_assembly(config)
  → GatewayRuntime(config, robot_assembly)
  → runtime.start()
      → PlatformGeometryRuntime.start()  [如果启用]
      → 各 Service.start()
      → 注册所有能力到 CapabilityRegistry
      → 初始化 EventBus / StreamBus / StateStore
```

### 4.2 EventBus

`EventBus` 是线程安全的内存事件总线，支持游标续播：

```python
class EventBus:
    def publish(event_type, category, source, ...) -> RuntimeEvent
    def subscribe(callback, categories=None, event_types=None) -> str
    def unsubscribe(subscription_id) -> None
    def get_events(after_cursor, categories=None, ...) -> List[RuntimeEvent]
    def latest_cursor() -> int
```

**关键特性：**
- 保留最近 1,000 条事件（可配置）
- 支持按 `category` 和 `event_type` 过滤
- 游标单调递增，支持断线重连后续播
- 线程安全（RLock）

**事件类别（RuntimeEventCategory）：**

| 类别 | 说明 |
|------|------|
| `task` | 任务生命周期事件 |
| `capability` | 能力调用事件 |
| `robot` | 机器人状态变化 |
| `navigation` | 导航状态变化 |
| `exploration` | 探索状态变化 |
| `localization` | 定位状态变化 |
| `mapping` | 地图状态变化 |
| `perception` | 感知事件 |
| `memory` | 记忆写入事件 |
| `safety` | 安全事件 |
| `system` | 系统级事件 |

### 4.3 StreamBus

`StreamBus` 是线程安全的实时数据流总线，用于高频连续数据：

```python
class StreamBus:
    def publish(topic, payload_kind, payload, ...) -> StreamFrame
    def subscribe(callback, topics=None) -> str
    def latest_frame(topic) -> Optional[StreamFrame]
    def latest_frames(topics=None) -> Tuple[StreamFrame, ...]
    def get_frames(after_cursor, topics=None, ...) -> List[StreamFrame]
```

**流话题（Stream Topics）：**

| 话题 | 数据类型 |
|------|----------|
| `robot.state` | 机器人状态帧 |
| `video.frame` | 视频帧 |
| `pointcloud.frame` | 点云帧 |
| `localization.pose` | 定位位姿 |
| `navigation.state` | 导航状态 |
| `exploration.state` | 探索状态 |
| `map.frame` | 地图帧 |

### 4.4 TaskManager

`TaskManager` 管理异步长时任务的完整生命周期：

```python
class TaskManager:
    def submit(spec: TaskSpec, runner: TaskRunner) -> TaskStatus
    def cancel(task_id: str) -> bool
    def get_status(task_id: str) -> Optional[TaskStatus]
    def list_active() -> List[TaskStatus]
    def list_history(limit: int) -> List[TaskStatus]
```

**任务状态机：**

```text
PENDING → RUNNING → SUCCEEDED
                  → FAILED
                  → CANCELLED（外部取消）
                  → TIMEOUT（超时触发）
```

**关键特性：**
- 基于 `ThreadPoolExecutor` 的并发执行
- `Timer` 驱动的超时机制
- 请求上下文穿透（request_id、capability_name 等）
- 资源锁集成（`ResourceLockManager`）
- 任务事件自动发布到 `EventBus`

### 4.5 CapabilityRegistry

`CapabilityRegistry` 管理平台所有能力的元数据和可用性：

```python
class CapabilityRegistry:
    def register(descriptor: CapabilityDescriptor) -> None
    def get_descriptor(name: str) -> Optional[CapabilityDescriptor]
    def list_descriptors(exposed_only: bool) -> List[CapabilityDescriptor]
    def set_availability(name: str, available: bool) -> None
```

**能力 Owner 分类：**

| Owner | 说明 | 典型能力数 |
|-------|------|-----------|
| `platform_geometry_runtime` | 平台几何运行时持有 | ~53 |
| `gateway_runtime` | 网关运行时持有 | 若干 |
| `robot_private_runtime` | 机器人私有能力 | ~23 |
| `ops_debug_runtime` | 运维调试能力 | 若干 |

### 4.6 StateStore

`StateStore` 是线程安全的状态快照缓存：

```python
class StateStore:
    def get(namespace: str, key: str) -> Optional[StateEntry]
    def set(namespace: str, key: str, value: Any, source: str) -> None
    def update(namespace: str, key: str, updater: Callable) -> None
    def list_namespace(namespace: str) -> List[StateEntry]
```

**命名空间：**

| 命名空间 | 存储内容 |
|----------|----------|
| `ROBOT_STATE` | 电池、IMU、关节、运动状态 |
| `LOCALIZATION` | 定位位姿、帧树 |
| `OBSERVATION` | 观察上下文 |
| `PERCEPTION` | 感知结果 |
| `NAVIGATION` | 导航状态 |
| `MAP` | 地图快照 |
| `MEMORY` | 记忆摘要 |
| `SAFETY` | 安全状态 |
| `SYSTEM` | 系统级状态 |

---

## 5. 对外通信接口

本平台对 Nuwaxclaw 暴露五类接口：

| 接口类型 | 用途 | 传输协议 |
|----------|------|----------|
| MCP | 上层智能体默认工具调用入口 | JSON-RPC over HTTP |
| HTTP API | 调试、管理、状态查询和直接能力调用 | RESTful HTTP |
| SSE | 浏览器或服务端订阅事件流 | Server-Sent Events |
| WebSocket | 双向事件与数据流连接 | WebSocket |
| Artifacts | 大对象制品下载 | HTTP GET |

### 5.1 MCP 主入口

MCP 入口为 `POST /mcp`，协议形式为 JSON-RPC 2.0。

**支持的方法：**

| 方法 | 说明 |
|------|------|
| `initialize` | 初始化 MCP 会话，返回协议版本、服务信息、事件地址、制品地址 |
| `tools/list` | 返回工具集合（platform + device） |
| `tools/call` | 调用工具集合中的具体能力 |

**协议版本：** `2025-11-25`（默认），兼容 `2025-06-18`

**工具集合：**

| 集合 | 含义 | 面向对象 |
|------|------|----------|
| `platform` | 平台侧正式功能集合 | Nuwaxclaw 智能体默认使用 |
| `device` | 端侧设备功能集合（robot_private + ops_debug） | 机器人私有动作 + 运维调试 |

#### 5.1.1 initialize 示例

请求：

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "initialize",
  "params": {}
}
```

响应：

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

#### 5.1.2 tools/list 示例

请求：

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/list",
  "params": {}
}
```

响应返回 `platform` 和 `device` 两个工具集合，每个集合的 `inputSchema` 中包含 `x-nuwax-function-catalog`，列出已注册能力、输入输出 schema、执行模式、风险等级和能力层。

`tools/list` 是上层智能体启动时的稳定工具发现入口，必须快速返回静态目录，不在枚举阶段触发后端实时可用性探测。

#### 5.1.3 tools/call 示例

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

返回结构：

```json
{
  "content": [
    { "type": "text", "text": "..." }
  ],
  "structuredContent": {
    "task_id": "task-nav-xxxx",
    "status": "accepted"
  }
}
```

若能力是异步任务，`structuredContent` 中包含 `task_id`，上层应通过事件流追踪执行状态。

### 5.2 HTTP API

所有 HTTP 接口需要认证。令牌传递方式：

```text
Authorization: Bearer <token>
```

也支持 query 参数 `?access_token=<token>`，但推荐使用 Authorization header。

#### 5.2.1 基础接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 网关根信息（机器人名称、型号、主要路径） |
| `/ops` | GET | 运维 HTML 面板（暗色/亮色主题） |
| `/api/health` | GET | 健康摘要 |
| `/api/capabilities` | GET | 能力清单 |
| `/api/tools` | GET | 工具视图（agent 只返回默认暴露能力，admin 可见全部） |
| `/api/security/policy` | GET | 安全策略摘要（不返回令牌明文） |
| `/api/deployment/layout` | GET | 部署目录布局 |

#### 5.2.2 能力调用接口

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

#### 5.2.3 任务接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/api/tasks` | GET | 查询活动任务和历史任务 |
| `/api/tasks/{task_id}` | GET | 查询单个任务状态 |
| `/api/tasks/{task_id}/cancel` | POST | 取消任务 |

#### 5.2.4 状态、观测和感知接口

| 接口 | 方法 | 说明 |
|------|------|------|
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
| `/streams/video.ingest` | WebSocket | Go2 端侧主动推送 JPEG 视频帧 |
| `/streams/video.mjpeg` | GET | 平台转发 MJPEG 流供监控页播放 |

#### 5.2.5 地图、定位、导航接口

| 接口 | 方法 | 说明 |
|------|------|------|
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

#### 5.2.6 记忆接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/api/memory/summary` | GET | 记忆服务摘要 |
| `/api/memory/libraries` | GET | 记忆库列表 |
| `/api/memory/libraries` | POST | 创建记忆库 |
| `/api/memory/libraries/{library_name}` | DELETE | 删除记忆库 |
| `/api/memory/locations` | GET | 地点记忆列表 |
| `/api/memory/semantic` | GET | 语义记忆列表 |

### 5.3 事件流接口

#### SSE（Server-Sent Events）

```text
GET /events/stream
```

| 参数 | 说明 |
|------|------|
| `after_cursor` | 从指定事件游标之后回放 |
| `categories` | 按事件类别过滤（逗号分隔） |
| `event_types` | 按事件类型过滤（逗号分隔） |
| `max_events` | 限制最多返回事件数 |

SSE 返回格式：

```text
id: 123
event: runtime.event
data: {"event_type":"task.succeeded","category":"task","payload":{...}}
```

#### WebSocket

```text
WS /events/ws
```

| 参数 | 说明 |
|------|------|
| `after_cursor` | 连接后先回放历史事件 |
| `categories` | 按类别过滤 |
| `event_types` | 按事件类型过滤 |

WebSocket 消息格式：

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

无事件时周期性发送 keepalive 消息。

### 5.4 制品接口

制品（Artifact）用于承载不适合直接塞进 JSON 的大对象（图片、音频、地图快照、日志片段等）。

| 接口 | 方法 | 说明 |
|------|------|------|
| `/api/artifacts/summary` | GET | 制品存储摘要 |
| `/artifacts/{artifact_id}/meta` | GET | 制品元数据 |
| `/artifacts/{artifact_id}` | GET | 下载制品原始文件 |

制品元数据字段：

| 字段 | 说明 |
|------|------|
| `artifact_id` | 唯一标识 |
| `kind` | 制品类型（image/audio/map/pointcloud/log） |
| `mime_type` | MIME 类型 |
| `size_bytes` | 文件大小 |
| `uri` | 下载 URI |
| `sha256` | 内容哈希 |
| `metadata` | 附加元数据 |

### 5.5 调用闭环

推荐 Nuwaxclaw 对长时能力采用如下闭环：

```text
1. MCP tools/call 或 HTTP invoke 发起能力调用
2. 返回同步结果或 task_id
3. 如果返回 task_id，订阅 /events/stream 或 /events/ws
4. 根据 task.started / task.progress / task.succeeded / task.failed 更新状态
5. 如需取消，调用 /api/tasks/{task_id}/cancel 或 cancel_task 能力
6. 任务完成后读取 final result、artifacts、memory、map、navigation context
```

> **重要：** 不要把"HTTP 调用返回 200"当作任务完成。对导航、探索、感知运行时、找物等长时能力，应以任务终态事件作为完成依据。

---

## 6. 能力模型

### 6.1 能力描述符

每个能力由 `CapabilityDescriptor` 描述，核心字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | str | 能力唯一名称 |
| `display_name` | str | 显示名 |
| `description` | str | 能力描述 |
| `input_schema` | dict | 输入参数 JSON Schema |
| `output_schema` | dict | 输出结果 JSON Schema |
| `execution_mode` | enum | 执行模式（SYNC / ASYNC_TASK / STREAM） |
| `risk_level` | enum | 风险等级 |
| `exposed_to_agent` | bool | 是否默认暴露给智能体 |
| `tags` | list | 标签 |
| `metadata` | dict | 附加元数据 |

### 6.2 执行模式

| 模式 | 含义 | 典型场景 |
|------|------|----------|
| `SYNC` | 调用后立即返回结果 | 状态查询、快照获取 |
| `ASYNC_TASK` | 调用后返回 task_id，后续通过事件流跟踪 | 导航、探索、找物 |
| `STREAM` | 持续输出数据流 | 持续感知、视频流 |

### 6.3 风险等级

| 风险等级 | 默认访问策略 |
|----------|-------------|
| `LOW` | agent / admin 可调用 |
| `MEDIUM` | agent / admin 可调用 |
| `HIGH` | 仅 agent allowlist 或 admin 可调用 |
| `ADMIN` | 仅 admin 可调用 |

### 6.4 能力分层

| 层 | 含义 | 默认暴露给智能体 |
|----|------|-----------------|
| `platform` | 平台正式能力（导航、探索、地图、记忆、感知摘要） | 是 |
| `robot_private` | 机器人私有能力（Go2 姿态、音量、控制模式、避障开关） | 是 |
| `ops_debug` | 运维调试能力（裸图像、裸关节、IMU、定位/地图/导航快照） | 否 |

### 6.5 典型平台能力清单

> 以下为当前实现的典型能力，实际清单以 `/api/tools` 或 MCP `tools/list` 返回为准。

#### 定位能力

| 能力名 | 说明 |
|--------|------|
| `get_localization_snapshot` | 查询当前定位快照 |
| `get_localization_session` | 查询定位会话 |
| `relocalize_active_map` | 对当前激活地图执行重定位 |

#### 地图能力

| 能力名 | 说明 |
|--------|------|
| `get_map_snapshot` | 获取地图快照 |
| `list_maps` | 列出所有地图资产 |
| `create_map` | 创建地图 |
| `activate_map` | 激活地图工作区 |
| `delete_map` | 删除地图 |
| `get_active_map` | 查询当前激活地图 |
| `list_map_versions` | 列出地图版本 |
| `save_active_map_version` | 保存当前地图版本 |
| `load_map_version` | 加载地图版本 |
| `delete_map_version` | 删除地图版本 |

#### 导航能力

| 能力名 | 说明 |
|--------|------|
| `navigate_to_pose` | 导航到指定坐标 |
| `navigate_to_named_location` | 导航到命名地点 |
| `explore_area` | 探索区域 |
| `explore_and_find_target` | 探索并查找目标 |
| `find_target` | 查找目标 |
| `inspect_target` | 检查目标 |
| `follow_target` | 跟随目标 |
| `relative_move` | 相对移动 |
| `get_navigation_snapshot` | 获取导航快照 |
| `get_navigation_semantic_context` | 获取语义导航上下文 |
| `resolve_navigation_candidates` | 解析导航候选 |

#### 感知能力

| 能力名 | 说明 |
|--------|------|
| `describe_scene` | 描述当前场景 |
| `process_image` | 处理图像 |
| `start_perception_runtime` | 启动持续感知 |
| `stop_perception_runtime` | 停止持续感知 |

#### 记忆能力

| 能力名 | 说明 |
|--------|------|
| `create_memory_library` | 创建记忆库 |
| `tag_location` | 标记地点 |
| `write_semantic_memory` | 写入语义记忆 |
| `query_semantic_memory` | 查询语义记忆 |
| `query_scene_spaces` | 查询场景空间 |

#### 其他能力

| 能力名 | 说明 |
|--------|------|
| `cancel_task` | 取消任务 |
| `stop_all_motion` | 停止所有运动 |
| `speak_text` | 文本转语音播报 |
| `list_capabilities` | 列出运行时能力 |

### 6.6 典型设备能力清单

| 能力名 | 说明 |
|--------|------|
| `get_robot_state` | 获取机器人状态 |
| `get_imu_state` | 获取 IMU 状态 |
| `get_joint_state` | 获取关节状态 |
| `get_battery_state` | 获取电池状态 |
| `get_safety_state` | 获取安全状态 |
| `send_motion_command` | 发送运动指令 |
| `execute_sport_command` | 执行运动命令 |
| `set_body_pose` | 设置身体姿态 |
| `set_speed_level` | 设置速度等级 |
| `switch_control_mode` | 切换控制模式 |
| `set_obstacle_avoidance` | 设置避障开关 |
| `capture_image` | 抓取单帧图像 |
| `get_volume` | 获取音量 |
| `set_volume` | 设置音量 |

---

## 7. 服务层

### 7.1 服务总览

| 服务 | 模块路径 | 核心类 | 职责 |
|------|----------|--------|------|
| 几何运行时 | `services/geometry/runtime.py` | `PlatformGeometryRuntime` | RTAB-Map + Nav2 集成、ROS2 桥接 |
| 导航 | `services/navigation/service.py` | `NavigationService` | 路径规划、导航执行、前沿探索 |
| 感知 | `services/perception/service.py` | `PerceptionService` | YOLO-World 检测、跟踪、场景理解 |
| 记忆 | `services/memory/service.py` | `MemoryService` | 向量记忆、空间记忆、实例关联 |
| 地图 | `services/mapping/service.py` | `MappingService` | 地图快照、版本管理、工作区 |
| 定位 | `services/localization/service.py` | `LocalizationService` | TF 定位、位姿估计、帧树 |
| 音频 | `services/audio/service.py` | `AudioService` | TTS 播报 |
| 机器人状态 | `services/robot_state_service.py` | `RobotStateService` | 状态聚合 |
| 观察 | `services/observation_service.py` | `ObservationService` | 观察上下文聚合 |
| 制品 | `services/artifact_service.py` | `ArtifactService` | 制品生命周期管理 |
| 评估 | `services/evaluation/service.py` | `EvaluationService` | 评估指标 |

### 7.2 PlatformGeometryRuntime（几何运行时）

`PlatformGeometryRuntime`（`services/geometry/runtime.py`，约 4,000 行）是地图、定位、导航、探索的核心后端，管理 RTAB-Map + Nav2 侧车闭环。

**职责：**
- 管理 ROS2 上下文和生命周期
- 发布里程计到平台受控话题（`/nuwax/geometry/odom`）
- 发布点云到 RTAB-Map 输入话题
- 从 TF 树解析定位位姿
- 提供实时占据栅格地图
- 代理 Nav2 Action 导航请求
- 前沿探索自主循环

**启动链路：**

```text
GatewayRuntime.start()
  → PlatformGeometryRuntime.start()
      → 初始化 Ros2Context
      → 启动 Ros2OdometryPublisher
      → 启动 Ros2PointCloudPublisher
      → 启动 Ros2LiveMapProvider
      → 启动 Ros2TfLocalizationProvider
      → 启动 odom_tf_bridge + static TF
      → 启动 RTAB-Map 侧车进程
      → 启动 Nav2 侧车进程
      → TF 反哺 LocalizationService / NavigationService
```

**默认配置：**

| 配置项 | 默认值 | 含义 |
|--------|--------|------|
| `NUWAX_GEOMETRY_ENABLED` | `true` | 启用平台 geometry 主链 |
| `NUWAX_GEOMETRY_REQUIRE_ROS2` | `true` | ROS2 不可用时视为启动失败 |
| `NUWAX_NAV2_MODE` | `sidecar` | 由平台启动 Nav2 侧车 |
| `NUWAX_GEOMETRY_ODOM_TOPIC` | `/nuwax/geometry/odom` | 平台受控 odom 话题 |

**数据流向：**

```text
Go2 DDS 位姿 → raw odom 输入
  → Ros2OdometryPublisher → /nuwax/geometry/odom
  → odom_tf_bridge（odom → base TF）
  → RTAB-Map（SLAM 闭环 → map → odom TF）
  → Ros2TfLocalizationProvider（解析 map 坐标系 pose）
  → LocalizationService / NavigationService / MemoryService

Go2 点云 → Ros2PointCloudPublisher → RTAB-Map
  → Ros2LiveMapProvider（占据栅格）
  → MappingService

导航请求 → Nav2ActionNavigationProvider → Nav2 Action Server
  → 路径规划 → 速度指令 → MotionControl
```

### 7.3 NavigationService（导航服务）

管理导航目标、探索状态和语义导航上下文。

**关键组件：**
- `NavigationStateMachine`：导航状态机
- `ExplorationProvider`：前沿探索后端
- `NavigationProvider`：Nav2 导航后端

**导航后端策略：**

正式执行链唯一为 Nav2 Action（`navigation_backend_policy: execution_chain=nav2_action_only`）。已废弃的 `platform_grid` 只允许作为删除诊断出现，不得作为正式后端或回退路径。

### 7.4 PerceptionService（感知服务）

处理视频帧的检测、跟踪、分割和场景理解。

**组件：**

| 组件 | 职责 |
|------|------|
| `DetectorPipeline` | YOLO-World 目标检测 |
| `TrackerBackend` | ByteTrack 2D 对象跟踪 |
| `SegmentationBackend` | 语义分割 |
| `SceneDescriptionBackend` | OpenAI Vision 场景理解 |
| `DetectionGeometryAugmentor` | 检测结果几何增强 |

**视频流链路：**

```text
Go2 端侧 go2_video_streamer
  → 读取本机 videohub
  → 主动推送 /streams/video.ingest（WebSocket JPEG）
  → VideoStreamReceiver（latest-only 缓冲）
  → /streams/video.mjpeg（监控页播放）
  → YOLO-World 按需抽取 latest/keyframe 检测
```

> `capture_image` 仅是显式设备调试/单次快照能力，不参与监控视频或持续感知。

### 7.5 MemoryService（记忆服务）

管理空间语义记忆、地点标记、对象实例关联和向量检索。

**组件：**

| 组件 | 职责 |
|------|------|
| `MemoryRepository` | 持久化存储（SQLite） |
| `VectorStore` | 语义向量嵌入与相似度检索 |
| `SemanticMapBuilder` | 从观察构建语义地图 |
| `InstanceAssociationService` | 跨时间对象实例关联 |
| `VisionToMapProjectionService` | 视觉检测投射到地图坐标 |
| `GroundingQueryPlanner` | 查询接地规划 |
| `InspectionPosePlanner` | 检查位姿规划 |
| `SceneSpaceUnderstanding` | 场景空间理解 |

**"一图一库"原则：**

地图与记忆库同名绑定。上层执行导航、探索和语义找物前，应先确保已激活正确地图工作区：

```text
创建或选择地图
  → 激活地图工作区（自动绑定同名记忆库）
  → 定位 / 重定位
  → 执行导航、探索、找物或语义查询
  → 保存地图版本与记忆结果
```

### 7.6 MappingService（地图服务）

管理地图资产、地图工作区和版本。

**核心类：**
- `MappingService`：地图快照与刷新
- `MapWorkspaceService`：地图工作区生命周期管理

**地图版本管理：**

| 操作 | 说明 |
|------|------|
| `create_map` | 创建新地图资产 |
| `activate_map` | 激活地图工作区（挂载 RTAB-Map DB + 绑定记忆库） |
| `save_active_map_version` | 保存当前地图为新版本 |
| `load_map_version` | 加载历史地图版本 |
| `delete_map` | 删除地图资产及所有版本 |

### 7.7 LocalizationService（定位服务）

基于 TF 树的位姿估计与帧追踪。

**数据源：**
- `Ros2TfLocalizationProvider`：从 `/tf` + `/tf_static` 解析 `map → odom → base` 链
- 正式输出为 `map` 坐标系下的平台定位 pose
- Go2 DDS 位姿只作为受控 odom 输入，不直接作为定位输出

### 7.8 AudioService（音频服务）

TTS（Text-to-Speech / 文本转语音）播报能力。

**后端：** 豆包 TTS API
**播放模式：** `robot_webrtc`（通过机器人端 WebRTC 播放）

---

## 8. Provider 接口层

### 8.1 设计原则

所有 Provider 实现 `BaseProvider` 协议：

```python
class BaseProvider(Protocol):
    provider_name: str
    provider_version: str

    def is_available(self) -> bool: ...
```

Provider 是服务层和驱动层之间的抽象契约，使平台能力不绑定特定机器人实现。

### 8.2 设备类 Provider（机器人装配提供）

| Provider | 职责 | 典型实现 |
|----------|------|----------|
| `ImageProvider` | 抓取图像、提供相机参数 | Go2CameraProvider |
| `StreamingImageProvider` | 流式图像 | Go2StreamingImageProvider |
| `PointCloudProvider` | 提供点云帧 | Go2PointCloudProvider |
| `StateProvider` | 机器人状态、关节、IMU | Go2StateProvider |
| `StreamingStateProvider` | 流式状态 | Go2StreamingStateProvider |
| `MotionControl` | 下发速度、停止、取消运动 | Go2MotionControl |
| `SafetyProvider` | 安全状态 | Go2SafetyProvider |

### 8.3 平台几何 Provider（PlatformGeometryRuntime 持有）

| Provider | 职责 |
|----------|------|
| `LocalizationProvider` | 以 ROS2 TF 中 `map → odom → base` 解析出的平台定位 pose 为正式输出 |
| `MapProvider` | 汇总平台地图工作区、实时占据栅格、代价图和语义地图 |
| `NavigationProvider` | 承载 Nav2 Action 导航目标、取消和状态语义 |
| `ExplorationProvider` | 承载前沿探索任务、命中停机和探索状态语义 |

> 平台几何 Provider 不应由机器人端侧驱动实现。机器人只需提供必要的位姿、点云、运动执行输入。

---

## 9. 机器人驱动层

### 9.1 统一入口

机器人入口由 `NUWAX_ACTIVE_ROBOT` 环境变量决定。

**加载流程：**

```text
load_config()
  → NUWAX_ACTIVE_ROBOT
  → drivers/entry.py: get_active_robot_registration()
  → drivers/registry.py: ROBOT_DRIVER_REGISTRY
  → import drivers/<robot_name>
  → 调用 create_<robot_name>_assembly(config)
  → create_default_gateway_runtime(config, robot)
```

**当前注册项：**

| 名称 | 说明 |
|------|------|
| `go2` | Unitree Go2 正式参考实现 |
| `g1` | Unitree G1 机器人骨架 |
| `customrobot` | 自定义机器人骨架 |

### 9.2 RobotManifest（机器人清单）

每个机器人应提供 `RobotManifest`，用于声明机器人元数据：

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_name` | str | 机器人名称 |
| `robot_model` | str | 机器人型号 |
| `entrypoint` | str | 装配入口 |
| `description` | str | 说明 |
| `capability_matrix` | CapabilityMatrix | 能力矩阵 |
| `required_components` | list | 必需组件 |
| `optional_components` | list | 可选组件 |
| `default_sensors` | list | 默认传感器 |
| `default_audio_backends` | list | 默认音频后端 |
| `default_adapters` | list | 默认适配器 |

### 9.3 RobotAssemblyBase（装配基类）

所有机器人装配体必须实现 `RobotAssemblyBase`：

| 方法 | 说明 |
|------|------|
| `start()` | 启动机器人入口 |
| `stop()` | 停止机器人入口 |
| `get_status()` | 返回装配状态 |
| `bind_adapter(adapter)` | 绑定外部适配器 |
| `initialize_registered_adapters()` | 初始化所有适配器 |
| `stop_registered_adapters()` | 停止所有适配器 |
| `get_adapter_health_statuses()` | 汇总适配器健康状态 |

### 9.4 Go2 驱动（参考实现）

Go2 驱动位于 `drivers/go2/`，是当前唯一完整实现。

**目录结构：**

```text
drivers/go2/
  ├── __init__.py
  ├── assembly.py          # Go2RobotAssembly
  ├── manifest.py          # build_go2_manifest()
  ├── capabilities.py      # GO2_CAPABILITY_DESCRIPTORS_BY_NAME
  ├── private_capabilities.py  # Go2 私有能力
  ├── providers.py         # Go2ProviderBundle
  ├── settings.py          # Go2 配置
  ├── data_plane.py        # Go2DataPlaneRuntime / Go2DeviceDataPlane
  ├── low_level_controller.py  # LowLevelController / JointConfig
  └── video_streamer.py    # Go2VideoStreamer
```

**关键组件：**

| 组件 | 职责 |
|------|------|
| `Go2RobotAssembly` | 装配入口，管理 Go2 全部子系统 |
| `Go2DataPlaneRuntime` | DDS 数据面管理（Unitree SDK2） |
| `Go2DeviceDataPlane` | 设备数据面（状态、关节、IMU） |
| `Go2ProviderBundle` | 统一提供所有设备类 Provider |
| `Go2VideoStreamer` | 从 videohub 读取并推送视频帧 |
| `LowLevelController` | 底层关节控制器 |

**SDK 集成：** Unitree SDK2（懒加载，仅在实机环境导入）

### 9.5 关键文件索引

| 文件 | 说明 |
|------|------|
| `drivers/entry.py` | 当前激活机器人统一入口 |
| `drivers/registry.py` | 机器人注册表（ROBOT_DRIVER_REGISTRY） |
| `drivers/common/assembly.py` | 机器人装配基类 |
| `drivers/common/manifest.py` | RobotManifest 契约 |
| `drivers/common/data_plane.py` | 数据面基类 |
| `drivers/common/private_capabilities.py` | 通用私有能力基础设施 |
| `drivers/sensor/` | 传感器注册表 |

---

## 10. 适配器框架

### 10.1 设计概述

适配器（Adapter）是连接外部系统或数据源的标准化组件，位于 `adapters/` 目录。

**基类：** `AdapterBase`

**适配器类别：**

| 类别 | 说明 | 示例 |
|------|------|------|
| `MAPPING` | 地图数据适配 | `SlamOccupancyAdapter` |
| `LOCALIZATION` | 定位数据适配 | `GeometryStateAdapter`、`OdomPoseAdapter` |
| `NAVIGATION` | 导航状态适配 | `NavigationStateAdapter` |
| `PERCEPTION` | 感知数据适配 | `ExternalPerceptionServiceAdapter` |
| `STREAMS` | 流数据适配 | `RosImageAdapter`、`WebrtcImageAdapter` |

### 10.2 适配器生命周期

```text
CREATED → INITIALIZED → RUNNING
                       → DEGRADED（部分功能受损）
                       → ERROR（不可用）
         → STOPPED
```

### 10.3 AdapterConfig

| 字段 | 说明 |
|------|------|
| `name` | 适配器名称 |
| `category` | 类别 |
| `source_kind` | 数据源类型 |
| `contract_type` | 输出契约类型 |
| `enabled` | 是否启用 |
| `source_ref` | 数据源引用 |
| `healthcheck_timeout_sec` | 健康检查超时 |
| `recoverable` | 是否可恢复 |
| `settings` | 适配器专有设置 |
| `metadata` | 附加元数据 |

---

## 11. 数据契约层

### 11.1 设计原则

所有契约位于 `contracts/` 目录，基于 Pydantic v2 的 `BaseModel`：

```python
class ContractModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",           # 禁止额外字段
        validate_assignment=True, # 赋值时校验
        populate_by_name=True,    # 允许按名称填充
        use_enum_values=False,    # 保留枚举对象
    )

class TimestampedContract(ContractModel):
    timestamp: datetime = Field(default_factory=utc_now)
```

### 11.2 契约文件索引

| 契约文件 | 职责 |
|----------|------|
| `base.py` | ContractModel、TimestampedContract、MetadataDict、utc_now() |
| `capabilities.py` | CapabilityDescriptor、CapabilityAvailability、CapabilityMatrix |
| `tasks.py` | TaskSpec、TaskStatus、TaskEvent、TaskState |
| `events.py` | RuntimeEvent、RuntimeEventCategory、RuntimeEventSeverity |
| `skills.py` | SkillDescriptor、SkillRuntimeView |
| `geometry.py` | Pose、Quaternion、Twist、Vector3 |
| `navigation.py` | NavigationGoal、NavigationStatus、ExploreAreaRequest |
| `memory.py` | MemoryPayloadFilter、空间记忆契约 |
| `spatial_memory.py` | SceneSpace、ObservationEvent、NavigationCandidate |
| `image.py` | ImageFrame、CameraInfo |
| `pointcloud.py` | PointCloudFrame、LaserScanFrame |
| `maps.py` | 地图元数据与版本 |
| `map_workspace.py` | MapAsset、MapWorkspace、MapVersion |
| `robot_state.py` | 电池、IMU、关节状态 |
| `streams.py` | StreamPayloadKind、StreamFrame |
| `artifacts.py` | ArtifactRef、ArtifactRetentionPolicy |
| `geometry_runtime.py` | PlatformGeometryConfig、RtabmapConfig、Nav2Config |
| `runtime_views.py` | RuntimeGovernanceSummary、NavigationContext |
| `debug_views.py` | 调试面板数据 |
| `frame_semantics.py` | 帧 ID 语义 |
| `evaluation.py` | 评估指标 |
| `naming.py` | ID 构建器（build_artifact_id、build_task_id 等） |

---

## 12. 配置系统

### 12.1 配置加载机制

配置由 `settings.py` 统一解析，加载优先级：

```text
环境变量（最高优先级）
  → .env 文件
  → .config 文件（旧部署兼容）
  → 代码默认值
```

> 业务逻辑不应直接散落读取环境变量，而应通过 `settings.py` 提供的结构化配置对象访问。

### 12.2 核心配置项

#### 机器人配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `NUWAX_ACTIVE_ROBOT` | `go2` | 当前激活机器人 |

#### 网关配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `NUWAX_GATEWAY_HOST` | `0.0.0.0` | 监听地址 |
| `NUWAX_GATEWAY_PORT` | `8766` | 监听端口 |
| `NUWAX_GATEWAY_AGENT_TOKENS` | — | Agent 令牌（逗号分隔） |
| `NUWAX_GATEWAY_ADMIN_TOKENS` | — | Admin 令牌（逗号分隔） |
| `NUWAX_GATEWAY_ALLOWED_SOURCE_CIDRS` | `0.0.0.0/0` | 允许的来源 CIDR |

#### 几何栈配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `NUWAX_GEOMETRY_ENABLED` | `true` | 启用几何主链 |
| `NUWAX_GEOMETRY_STACK` | `external_slam_nav2` | 几何栈类型 |
| `NUWAX_GEOMETRY_REQUIRE_ROS2` | `true` | ROS2 不可用时启动失败 |
| `NUWAX_GEOMETRY_SETUP_SCRIPT` | 自动生成 | ROS2 环境准备脚本 |
| `NUWAX_RTABMAP_DATABASE_ROOT` | `runtime_data/rtabmap/` | RTAB-Map 数据库根目录 |
| `NUWAX_NAV2_MODE` | `sidecar` | Nav2 模式（sidecar/external/disabled） |
| `NUWAX_GEOMETRY_ODOM_TOPIC` | `/nuwax/geometry/odom` | 平台受控 odom 话题 |

#### 感知配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND` | `yolo_world` | 默认检测器后端 |
| `NUWAX_PERCEPTION_YOLO_WORLD_MODEL` | — | YOLO-World 模型路径 |
| `NUWAX_PERCEPTION_OPENAI_ENABLED` | `false` | 是否启用 OpenAI Vision |

#### 记忆配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `NUWAX_MEMORY_DB_PATH` | `runtime_data/memory/` | 记忆数据库路径 |
| `NUWAX_MEMORY_TEXT_EMBEDDING_MODEL` | `BAAI/bge-m3` | 文本嵌入模型 |
| `NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL` | `openai/clip-vit-base-patch32` | 图像嵌入模型 |
| `NUWAX_MEMORY_ONLINE_EMBEDDING_MODEL` | `tongyi-embedding-vision-flash` | 在线嵌入模型 |
| `NUWAX_MEMORY_VECTOR_STORE_BACKEND` | `sqlite_named_vectors` | 向量存储后端 |

#### 音频配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `DOUBAO_APP_ID` | — | 豆包 TTS AppID |
| `TTS_PLAYER_BACKEND_MODE` | `robot_webrtc` | TTS 播放模式 |

#### 日志配置

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `NUWAX_LOG_DIR` | 项目根目录 | 日志目录 |
| `NUWAX_LOG_FILE` | `nuwax_robot_bridge.log` | 日志文件名 |
| `NUWAX_LOG_LEVEL` | `INFO` | 日志级别 |
| `NUWAX_LOG_MAX_LINES` | `100000` | 日志最大行数 |

---

## 13. 日志系统

### 13.1 架构设计

日志系统位于 `logging_utils.py`，提供结构化、可追踪、智能降噪的日志能力。

**核心组件：**

| 组件 | 职责 |
|------|------|
| `CorrelationContextFilter` | 注入关联上下文（request_id、task_id、capability_name） |
| `DeduplicatingLogFilter` | 去重重复日志消息 |
| `DropPatternLogFilter` | 按模式丢弃噪声日志 |
| `JsonLineFormatter` | JSON 行格式化（结构化日志） |
| `build_default_formatter()` | 构建默认格式化器 |

### 13.2 关联上下文

日志自动注入以下上下文字段（按优先级排序）：

| 字段 | 说明 |
|------|------|
| `event` | 事件标识 |
| `request_id` | 请求 ID |
| `task_id` | 任务 ID |
| `capability_name` | 能力名称 |
| `method` | HTTP 方法 |
| `path` | 请求路径 |
| `status_code` | 状态码 |
| `duration_ms` | 耗时（毫秒） |
| `error_category` | 错误类别 |
| `category` | 事件类别 |
| `source` | 来源 |
| `severity` | 严重性 |

### 13.3 智能降噪

**默认丢弃模式（Drop Patterns）：**
- DDS 心跳、数据通道消息
- SDP 协商日志
- 通道关闭通知

**低价值事件抑制：**
- `localization.snapshot_ready`
- `mapping.snapshot_ready`
- `robot.state_refreshed`

**日志值压缩：**
- 超长字符串自动截断（默认 240 字符）
- 二进制数据替换为 `<bytes len=N>`
- 深层嵌套对象按深度和条目数压缩

### 13.4 默认日志格式

```text
%(asctime)s [%(levelname)s] [%(name)s] %(message)s
```

JSON 模式下输出结构化 JSON Line，适合日志收集系统（如 ELK、Loki）消费。

---

## 14. 安全与鉴权

### 14.1 GatewayAccessManager

`GatewayAccessManager`（`gateways/auth.py`）负责统一鉴权：

**职责：**
- HTTP 请求认证（Token 校验）
- 能力级授权（风险等级 + 角色匹配）
- 能力列表过滤（按角色返回可见能力）
- 安全策略导出（不返回令牌明文）

### 14.2 访问角色

| 角色 | 权限 |
|------|------|
| `AGENT` | 可调用 LOW/MEDIUM 风险能力 + allowlist 内 HIGH 风险能力 |
| `ADMIN` | 可调用所有能力，包括 ADMIN 级别 |

### 14.3 认证方式

| 方式 | 格式 |
|------|------|
| Authorization Header | `Authorization: Bearer <token>` |
| Query Parameter | `?access_token=<token>` |

### 14.4 来源过滤

通过 `NUWAX_GATEWAY_ALLOWED_SOURCE_CIDRS` 配置允许的客户端 IP 范围（CIDR 格式）。

### 14.5 高风险能力控制

HIGH 风险能力需要在 Agent allowlist 中显式声明才可调用。ADMIN 级能力仅 Admin 令牌可访问。

---

## 15. 部署与运维

### 15.1 部署模式

| 模式 | 说明 | 典型场景 |
|------|------|----------|
| 直连 | 网关直接运行在机器人宿主机 | 开发调试、单机部署 |
| Relay | 通过 relay 服务代理转发 | 跨网络、容器化部署 |
| Docker | 容器化部署 | 与 NuwaClaw 联合部署 |

### 15.2 Bootstrap 脚本

`scripts/bootstrap.py` 提供部署引导：

**命令：**

| 命令 | 说明 |
|------|------|
| `check` | 预部署验证（环境检查） |
| `wizard` | 交互式部署向导 |
| `apply` | 非交互式部署 |

**部署层级（Tier）：**

| 层级 | 包含内容 |
|------|----------|
| `full` | 全量部署（Geometry + Perception + Memory + Navigation） |
| `lite` | 轻量部署（无 Geometry） |
| `minimal` | 最小部署（仅状态和运动控制） |

**apply 选项：**

```bash
python3 scripts/bootstrap.py apply \
  --tier full \
  --robot go2 \
  --geometry \
  --data-plane \
  --memory-mode cloud \
  --perception-mode hybrid \
  --mcp-mode auto
```

> 轻量部署必须显式 `--no-geometry`（写入 `NUWAX_GEOMETRY_ENABLED=false`），不能依赖默认静默关闭。

### 15.3 启动脚本

**scripts/start_gateway.sh：**

```bash
# 启动
bash scripts/start_gateway.sh start

# 停止
bash scripts/start_gateway.sh stop

# 重启
bash scripts/start_gateway.sh restart
```

**启动流程：**
1. 加载 `.env` 和 Go2 数据面配置
2. 清理可能残留的旧进程
3. 启动 Go2 视频推流子进程（如果适用）
4. 通过 uvicorn 启动 FastAPI 应用
5. 管理进程生命周期和重启策略

**实际执行命令：**

```bash
python3 -m uvicorn gateways.app:create_gateway_app_from_env \
  --factory --host 0.0.0.0 --port 8766
```

### 15.4 systemd 服务

**服务文件：** `deploy/systemd/nuwax-host-gateway.service`

```ini
[Unit]
Description=Nuwax Robot Bridge Host Gateway
After=network.target

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/nuwax-robot-bridge
ExecStart=/home/unitree/nuwax-robot-bridge/scripts/start_gateway.sh start
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

**管理命令：**

```bash
# 启用开机自启
sudo systemctl enable nuwax-host-gateway

# 启动服务
sudo systemctl start nuwax-host-gateway

# 查看状态
sudo systemctl status nuwax-host-gateway

# 查看日志
journalctl -u nuwax-host-gateway -f
```

### 15.5 Docker 部署

**示例文件：** `deploy/docker-compose.host-gateway.example.yml`

**服务组成：**

| 服务 | 说明 | 端口 |
|------|------|------|
| `nuwaclaw` | NuwaClaw 客户端容器 | — |
| `nuwax-relay` | 可选 relay 代理 | 9766 |

**环境变量（容器侧）：**

| 变量 | 说明 |
|------|------|
| `NUWAX_EMBODIMENT_MCP_URL` | MCP 入口地址 |
| `NUWAX_EMBODIMENT_API_URL` | HTTP API 地址 |
| `NUWAX_EMBODIMENT_EVENTS_WS_URL` | WebSocket 事件地址 |
| `NUWAX_EMBODIMENT_EVENTS_SSE_URL` | SSE 事件地址 |
| `NUWAX_GATEWAY_AGENT_TOKEN` | 认证令牌 |

### 15.6 MCP 配置同步

`scripts/nuwaclaw_mcp_config_sync.py` 自动同步 MCP 配置到 NuwaClaw 端：

- 生成 `mcp_proxy_config.host_gateway.json`
- 生成 `external_mcp_servers.host_gateway.json`
- 配置模板位于 `deploy/nuwaclaw/`

### 15.7 运维面板

访问 `http://<host>:8766/ops` 可打开运维 HTML 面板：

- 暗色/亮色主题切换
- 运行时状态实时显示
- 目录布局信息
- 安全策略视图
- 调试入口

### 15.8 健康检查

```bash
curl http://127.0.0.1:8766/api/health \
  -H 'Authorization: Bearer <token>'
```

返回包括：机器人连接状态、各服务健康状态、几何栈状态、任务队列状态。

---

## 16. 接入新机器人

### 16.1 准备工作

确认新机器人具备的能力，至少需要：
- 状态查询接口（电池、IMU、关节等）
- 运动控制接口（速度指令、停止）
- 安全状态接口

### 16.2 实施步骤

以接入 `myrobot` 为例：

#### 第一步：创建目录结构

```text
drivers/myrobot/
  ├── __init__.py
  ├── manifest.py
  ├── assembly.py
  ├── providers.py
  └── private_capabilities.py
```

#### 第二步：实现 RobotManifest

在 `manifest.py` 中声明机器人名称、型号、能力矩阵和组件：

```python
from drivers.common.manifest import RobotManifest

def build_myrobot_manifest() -> RobotManifest:
    return RobotManifest(
        robot_name="myrobot",
        robot_model="MyRobot V1",
        entrypoint="drivers.myrobot.assembly",
        description="MyRobot 机器人实现。",
        capability_matrix=...,
        required_components=[...],
        optional_components=[...],
        default_sensors=[...],
    )
```

#### 第三步：实现 RobotAssembly

在 `assembly.py` 中实现装配体：

```python
from drivers.common.assembly import RobotAssemblyBase

class MyRobotAssembly(RobotAssemblyBase):
    async def start(self): ...
    async def stop(self): ...
    def get_status(self): ...

def create_myrobot_assembly(config, iface=None):
    return MyRobotAssembly(config, iface)
```

#### 第四步：实现 Provider

根据机器人实际能力选择性实现设备类 Provider：

```python
from providers.state import StateProvider
from providers.motion import MotionControl
from providers.safety import SafetyProvider

class MyRobotStateProvider(StateProvider): ...
class MyRobotMotionControl(MotionControl): ...
class MyRobotSafetyProvider(SafetyProvider): ...
```

**最小可运行集合：** `StateProvider` + `MotionControl` + `SafetyProvider`

> 如果要接入地图/定位/导航/探索，不应在 `providers.py` 中重新实现 `MapProvider`、`NavigationProvider` 等。应提供必要的位姿、点云输入，由 `PlatformGeometryRuntime` 统一持有和暴露这些 Provider。

#### 第五步：实现私有能力

```python
from drivers.common.private_capabilities import PrivateCapabilitySpec

ROBOT_PRIVATE_CAPABILITY_SPECS = [
    PrivateCapabilitySpec(
        name="myrobot_special_action",
        display_name="MyRobot 特殊动作",
        handler=...,
        exposed_to_agent=True,
    ),
]
```

#### 第六步：注册机器人

在 `drivers/registry.py` 增加注册项：

```python
from drivers.registry import RobotDriverRegistration

ROBOT_DRIVER_REGISTRY["myrobot"] = RobotDriverRegistration(
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

```bash
# 启动网关
bash scripts/start_gateway.sh

# 健康检查
curl http://127.0.0.1:8766/api/health \
  -H 'Authorization: Bearer agent-robot-bridge-token'

# 查看工具列表
curl http://127.0.0.1:8766/api/tools \
  -H 'Authorization: Bearer agent-robot-bridge-token'
```

---

## 17. 附录

### 17.1 错误类别

| 错误代码 | 类别 | 说明 |
|----------|------|------|
| `authentication_required` | auth | 未提供令牌 |
| `invalid_token` | auth | 令牌无效 |
| `unauthorized` | auth | 认证失败 |
| `access_denied` | authorization | 授权不足 |
| `forbidden` | authorization | 禁止访问 |
| `capability_not_allowed` | authorization | 能力不允许 |
| `invalid_params` | validation | 参数无效 |
| `validation_error` | validation | 校验失败 |
| `not_found` | validation | 资源不存在 |
| `capability_not_registered` | capability | 能力未注册 |
| `resource_conflict` | capability | 资源冲突 |
| `execution_failed` | capability | 执行失败 |

### 17.2 任务状态定义

| 状态 | 说明 | 终态 |
|------|------|------|
| `PENDING` | 任务已创建，等待执行 | 否 |
| `RUNNING` | 任务执行中 | 否 |
| `SUCCEEDED` | 任务成功完成 | 是 |
| `FAILED` | 任务执行失败 | 是 |
| `CANCELLED` | 任务被外部取消 | 是 |
| `TIMEOUT` | 任务超时 | 是 |

### 17.3 事件严重性

| 严重性 | 说明 |
|--------|------|
| `DEBUG` | 调试信息 |
| `INFO` | 正常信息 |
| `WARNING` | 警告 |
| `ERROR` | 错误 |
| `CRITICAL` | 严重错误 |

### 17.4 关键端口

| 端口 | 用途 |
|------|------|
| `8766` | 网关主端口（HTTP/MCP/WS/SSE） |
| `9766` | Relay 代理端口 |

### 17.5 关键路径

| 路径 | 用途 |
|------|------|
| `runtime_data/` | 运行时数据根目录 |
| `runtime_data/artifacts/` | 制品存储 |
| `runtime_data/memory/` | 记忆数据库 |
| `runtime_data/rtabmap/` | RTAB-Map 数据库 |
| `runtime_data/models/` | AI 模型文件 |
| `.env` | 主配置文件 |
| `.config` | 旧格式配置文件（兼容） |

### 17.6 代码规模参考

| 模块 | 代码行数 | 说明 |
|------|----------|------|
| `gateways/runtime.py` | ~7,500 | 平台总编排器 |
| `services/geometry/runtime.py` | ~4,000 | 几何运行时 |
| `services/perception/service.py` | ~40,000 | 感知服务（含模型集成） |
| `services/navigation/service.py` | ~800 | 导航服务 |
| `services/memory/service.py` | ~2,000 | 记忆服务 |
| `gateways/http/debug_panel.py` | ~4,000 | 运维面板 |
| **全项目** | **~65,000+** | — |

### 17.7 外部依赖

| 依赖 | 用途 |
|------|------|
| FastAPI | Web 框架 |
| Uvicorn | ASGI 服务器 |
| Pydantic | 数据校验 |
| rclpy | ROS 2 Python 客户端 |
| nav2_msgs | Nav2 消息定义 |
| unitree_sdk2py | Unitree Go2 SDK |
| ultralytics | YOLO-World 推理 |
| openai | OpenAI Vision API |
| dashscope | 通义千问 Embedding |
| sqlite3 | 向量记忆存储 |
| websockets | WebSocket 支持 |

---

*文档结束*
