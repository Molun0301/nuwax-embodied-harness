# nuwax_robot_bridge

`nuwax_robot_bridge` 是运行在机器人宿主机上的具身能力运行时。

它的职责不是做聊天界面，也不是承载上层会话，而是把机器人侧的运动控制、状态采集、视觉感知、地图定位、导航、记忆、音频和制品通道整理成统一能力，再通过稳定接口暴露给上层系统调用。

## 项目定位

这套系统的边界固定如下：

1. 上层系统负责对话、规划、会话和工具编排。
2. `nuwax_robot_bridge` 负责连接机器人、执行能力、维护运行时状态并提供统一接口。
3. 机器人 SDK、传感器差异、地图来源差异尽量收敛在宿主机运行时内部。

当前主线已经具备：

1. 宿主机网关：`MCP + HTTP + SSE/WebSocket + artifacts`
2. Go2 机器人装配与统一 provider 抽象
3. 本地视觉检测与云端语义识别双链路
4. 默认 `YOLO26n + TensorRT` 本地检测链路，保留可替换模型后端
5. 关键帧驱动的视频感知运行时，以及到点 `burst` 复核
6. 地图、定位、导航、探索与任务状态管理
7. 语义空间记忆闭环：命名地点、对象实例、观察事件、向量检索、导航候选与到点复核
8. 制品保存、事件流、权限控制与部署资产

## 功能架构

```text
+---------------------------+            +----------------------------------+
| 上层系统 / 容器客户端     |            | nuwax_robot_bridge               |
|                           |            | 宿主机运行时                     |
| 对话 / 规划 / 工具编排    | -- MCP --> | 能力注册表 / 任务管理            |
| 会话 / UI / 用户入口      | -- HTTP -> | 机器人装配 / 适配器 / 服务层     |
|                           | <- 事件 ---| 感知 / 记忆 / 导航 / 制品        |
|                           | <- 文件 ---| 鉴权 / 安全 / 运行时状态         |
+---------------------------+            +----------------------------------+
```

## 仓库结构

根目录的主要文件夹和文件职责如下：

1. `adapters/`
   外部系统适配层，把 ROS、视频流、地图系统、第三方感知服务等输入转换成平台统一模型。
2. `contracts/`
   平台稳定契约定义，例如图像、地图、感知、导航、记忆、任务、事件等结构。
3. `core/`
   运行时核心能力，包括能力注册、任务管理、事件总线、资源锁和安全守卫。
4. `deploy/`
   部署资产目录，放置 `systemd`、`docker-compose`、环境变量示例和容器接入示例。
5. `docs/`
   项目文档目录。
   `公开介绍文档/` 面向 GitHub 读者；
   `开发文档/` 面向本地研发与架构演进记录。
6. `drivers/`
   机器人入口与装配层，按 `drivers/<robot_name>/ + drivers/common/ + drivers/sensor/` 组织。
   根目录 `.env` / `.env.example` 中的 `NUWAX_ACTIVE_ROBOT` 决定统一入口加载哪个机器人目录。
   其中 Go2 的底层控制实现固定放在 `drivers/go2/control/`。
   Go2 的端侧输入流、官方运动执行、设备服务与兼容数据面实现固定放在 `drivers/go2/`，
   例如 `drivers/go2/data_plane.py`、`drivers/go2/data_plane_entry.py`、
   `drivers/go2/settings.py`、`drivers/go2/data_plane.env.example`。
   其中项目根目录 `.env` 负责平台主配置与几何栈配置，
   包括 `NUWAX_GEOMETRY_*`、`NUWAX_RTABMAP_*`、`NUWAX_NAV2_*`；
   `drivers/go2/data_plane.env` 只负责 Go2 端侧数据面配置，二者不会互相回退读取。
   仓库内保留 `drivers/go2/ros2_ws/` 作为按需启用的 Go2 ROS2 侧车工作空间，
   当前只保留官方话题桥接与平台几何侧车所需依赖；旧的本地 `Nav2` 与 `Point-LIO` 启动链已经移除。
7. `gateways/`
   对外通信入口，包括 MCP、HTTP、事件流、relay 和兼容策略说明。
8. `providers/`
   平台 provider 接口定义，是机器人装配与服务层之间的标准边界。
9. `runtime_data/`
   运行时本地数据目录，例如记忆库和持久化状态。
10. `scripts/`
    部署辅助和运维脚本，例如容器侧外部 MCP 注入脚本、YOLO 权重预下载脚本、记忆向量模型预下载脚本、YOLO TensorRT 引擎导出脚本。
11. `services/`
    平台能力服务层，负责状态、观察、感知、地图、定位、导航、记忆、音频等正式能力。
    其中 `services/memory/` 已包含语义地图构建、视觉投影、实例关联、grounding 查询规划和 inspection pose 规划。
12. `skills/`
    对上层暴露的工具和技能注册层。
13. `tests/`
    单元测试、集成测试和端到端测试。
14. `tts/`
    文本播报和相关音频能力实现。
15. `README.md`
    项目总览和目录说明。
16. `部署文档.md`
    从零部署到实际可用的详细操作手册。
17. `.env.example`
    宿主机运行时配置示例。
    当前必须显式设置 `NUWAX_ACTIVE_ROBOT=<robot_name>`，值需与 `drivers/<robot_name>` 同名；
    记忆向量配置当前统一使用 `NUWAX_MEMORY_TEXT_EMBEDDING_*` 与 `NUWAX_MEMORY_IMAGE_EMBEDDING_*`；
    当前默认文本向量模型目录为 `runtime_data/models/BAAI__bge-m3`，默认图像向量模型目录为
    `runtime_data/models/openai__clip-vit-base-patch32`；
    启动阶段不会自动下载这两个模型，需要先运行 `python3 scripts/prepare_memory_models.py`；
    日志配置当前统一使用 `NUWAX_LOG_*`；
    `settings.py` 仍兼容旧变量名，但仅用于旧部署回退，不建议新环境继续使用。
18. `requirements-memory-models.txt`
    语义空间记忆的模型依赖清单。
    Jetson 场景应先确认 `torch` 可用，再安装本文件。
19. `drivers/go2/data_plane.env.example`
    Go2 官方定位、地图、导航执行与探索能力的专属配置示例。
    它和项目根目录 `.env` 分属不同配置域，不互相覆盖。
20. `drivers/go2/data_plane_entry.py`
    Go2 官方数据面的独立启动入口，可单独拉起官方定位、地图、探索相关数据面，再由平台装配接入。
21. `drivers/go2/ros2_ws/`
    Go2 配套 ROS2 工作空间，当前仅保留官方话题侧车与 `elevation_mapping` 相关依赖。
22. `drivers/go2/build_ros2_workspace.sh`
    构建 Go2 端侧 ROS2 工作空间的示例脚本。
23. `drivers/go2/launch_ros2_stack.sh`
    为 Go2 ROS2 侧车命令准备运行环境的便捷脚本，不再默认启动任何旧导航 launch。
24. `requirements.txt`
    Python 运行时依赖。

## 推荐阅读顺序

如果你第一次接触这个项目，建议按下面顺序阅读：

1. 本文档：先理解项目定位和仓库结构。
2. [部署文档.md](部署文档.md)：按步骤完成宿主机部署、容器接入和开机自启。
   其中已包含 `YOLO26n` 权重准备和 TensorRT 引擎导出步骤；仓库本身不内置模型权重。
3. [文档导航.md](docs/公开介绍文档/文档导航.md)：查看公开文档总入口。
4. [平台架构说明.md](docs/公开介绍文档/平台架构说明.md)：理解能力分层和数据流。
5. [宿主机与容器通信说明.md](docs/公开介绍文档/宿主机与容器通信说明.md)：理解容器内外职责和通信方式。
