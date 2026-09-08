# nuwax_robot_bridge

通用机器人能力平台：机器人提供标准传感器数据、标定与运动执行，平台实现建图、定位、导航、探索、视觉和空间记忆，通过 MCP（模型上下文协议）供智能体调用。

平台计算核心位于 `services/nuwax-workspace/`，由 Rust（系统编程语言）实现；`services/geometry/nuwax_geometry_runtime/` 负责几何运行编排，Python（编程语言）服务和网关组合这些能力。ROS2（机器人操作系统第二版）不再是当前平台计算主链。

## 按需安装

在 Debian/Ubuntu（操作系统）系列机器上，以普通用户在仓库根运行：

```bash
./install.sh
```

安装器自动检测处理器、可用内存、图形设备和已有配置，显示建议模块；回车确认，也可以输入编号增删。基础网关不安装视觉或语义模型；FastLIO2 与 PointLIO（两种激光惯性定位算法）选择一种即可。未知设备先使用标准接入，不根据计算板型号猜测机器人和雷达。

```bash
./install.sh install --modules core --yes  # 只安装基础网关
./install.sh add fastlio2                  # 增加几何建图及原生定位
./install.sh add pointlio                  # 替换为 PointLIO
./install.sh add perception memory         # 增加视觉与本地语义记忆
./install.sh remove perception             # 移除视觉及依赖它的跟随模块
./install.sh list                          # 查看当前安装与配置位置
./install.sh start                         # 前台启动；Ctrl+C 停止
```

添加 `--dry-run` 只查看计划，`--yes` 接受方案，`--repair` 重装修复。修改模块前先停止网关；再次运行沿用当前选择，重建独立依赖环境并在校验后切换，失败保留上一有效安装。默认安装位置是 `runtime_data/installation/`，地图、模型和用户数据不会随模块卸载删除，系统共享依赖也不会被卸载。

安装使用 Python 3.8—3.12；编译并行数按内存和处理器自动限制。Jetson（嵌入式计算平台）的本地推理复用厂商张量库；Go2（宇树四足机器人）接入复用已有设备软件，缺失时可通过 `--wheel-dir` 提供设备安装包。导航、跟随还需配置现有运动安全守卫与执行器，定位需要传感器数据及标定；安装完成不会自动启动机器人。

原有按档案生产部署、恢复和回滚命令保持可用，例如 `./install.sh deploy --help`；其厂商制品与发布准入条件独立于本机源码安装。

## 开发机验证

先使用已安装项目依赖的 Python 环境，在仓库根执行：

```bash
python scripts/diagnostics/offline_platform_smoke.py
```

命令使用显式模拟的静止状态和图像，启动真实本机 HTTP（超文本传输协议）网关，完成 MCP 初始化、发现、状态、抓图、制品下载、感知编排、地图创建和查询，最后关闭进程与后台线程。不会连接机器人，也不会下载或加载识别模型。

保留本次产物：

```bash
python scripts/diagnostics/offline_platform_smoke.py --output-dir runtime_data/offline-check
```

公共 Rust 工作区的开发验证：

```bash
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --workspace --lib
cargo build --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --release --bin nuwax-geometry-runtime
cargo build --manifest-path services/nuwax-workspace/Cargo.toml --release -p nuwax-nav-stack --features simple-2d,replanning-astar
```

两种原生定位程序编译完成后，可用相同的合成传感器场景验证真实算法及进程关闭：

```bash
python scripts/diagnostics/native_lio_smoke.py --backend all --gateway \
  --runtime-binary services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime
```

该验证不连接硬件。图像链验证使用元数据检测后端，不代表视觉模型准确率。

公共工作区声明的 Rust 最低版本为 1.88。原生计算库还需要对应系统开发依赖；完整依赖由构建错误和各库构建文件说明。不要用虚构发布清单或摘要绕过构建问题。

## 接入机器人

`drivers/customrobot/` 提供可用的标准数据装配。接入代码创建装配后发布 `RobotState`（机器人状态）、`ImageFrame`（图像帧）、`CanonicalLidarFrame`（规范雷达帧）与 `CanonicalImuSample`（规范惯性样本）；标定和坐标系必须与实际设备一致。缺少某类数据时对应能力显示不可用。

```python
from drivers.customrobot import create_customrobot_assembly

robot = create_customrobot_assembly(config, sensor_topology=topology)
robot.start()
robot.providers.publish_robot_state(state)
robot.providers.publish_image(image, camera_info)
robot.providers.publish_imu(imu_sample)
robot.providers.publish_point_cloud(lidar_frame)
```

机器人运动只需实现 `providers.motion.MotionControl` 的速度和停止操作。使用 `NUWAX_MOTION_GUARD_BACKEND=motion_control` 与 `NUWAX_MOTION_CONTROL_FACTORY=模块:工厂`，将执行器接入现有运动仲裁服务。Go2（宇树四足机器人）保留官方设备执行后端；G1（宇树人形机器人）等注册入口仍需相应设备实现。

摄像头数据可用于观察和抓图；物体识别需要配置对应模型。原生定位需要符合后端要求的雷达、惯性数据和外参。导航进一步需要可用定位、地图和运动执行。不能仅凭硬件名称宣称全部能力已经可用。

## 配置与运行

本地运行时复制 `.env.example` 为 `.env`，按机器人能力选择需要的配置。`NUWAX_BOOTSTRAP_ENV_FILE` 可显式选择平台配置文件，`GO2_DATA_PLANE_ENV_FILE` 可选择 Go2 数据面配置文件；Python 与启动脚本共用读取规则，显式进程环境变量优先。设置页面会显示实际来源与保存位置。

```bash
bash scripts/start_gateway.sh
```

模块安装使用 `./install.sh start` 和安装器专属设置，配置位置通过 `./install.sh list` 查看；手工开发启动仍可使用上述命令。旧配置引导工具说明可通过 `python scripts/bootstrap.py --help` 查看。生产服务共用 `/var/lib/nuwax_robot_bridge/settings.env` 作为用户设置覆盖文件，由 `NUWAX_USER_CONFIG_ENV_FILE` 指明；控制台保存到这里，各服务在基础配置之后读取。原来的几何、点云专用配置只保留读取兼容。没有指定统一覆盖文件的本地运行仍保存到已有配置来源，不需要修改系统目录权限。

几何启动器 `scripts/systemd/start_geometry_runtime.sh` 直接运行本地构建，不要求 SHA（安全散列算法）摘要或历史验收记录。只有显式设置 `NUWAX_GEOMETRY_RELEASE_MANIFEST` 时才使用已有发布包检查。合法算法组合可以先保存后构建，页面只提示当前缺少的程序或标定。

开发构建的几何程序位于 `services/geometry/nuwax_geometry_runtime/target/release/`，它使用独立构建清单；部署安装器将程序放入所选发布目录的 `bin/`。不要混用公共工作区与独立几何运行时的输出目录。

网关默认端口为 8766，主要入口：

- `/mcp`：工具发现与调用。
- `/api/health`、`/api/tools`：状态与能力。
- `/console`：Nuwax 机器人控制台，提供机器人状态、地图、结构化日志和统一设置；旧 `/debug` 地址跳转到此处。
- `/api/logs`、`/api/logs/export`：日志查询和导出。
- `/api/config/schema`、`/api/config/save`：设置目录和保存。
- `/events/stream`、`/streams/sse`、`/streams/ws`：事件与数据流。

`cancel_navigation` 与 `get_navigation_state` 使用真实导航服务。取消受理与取消完成分别表达；工具不再固定返回“尚未接通”。`capture_image` 返回标准图像制品。

## 控制台、日志和设置

打开 `http://127.0.0.1:8766/console`。设置项上方显示中文名称，下方显示对应键名，可选值使用中文标签，保存的机器值保持规范名称。编辑值、程序默认值、当前进程已读取值以及生效范围分别展示；保存后按提示重启对应服务，外部进程尚未确认的值不会显示为已确认生效。

日志页面按时间、级别、来源和组件组织记录，支持文本搜索、请求或任务筛选、展开异常与结构字段、单条或整页复制，以及文本或 JSONL（逐行结构化文本）导出。网关和独立 Rust、视频、运动组件的日志在同一页面查询。

`NUWAX_LOG_DIR` 指定共同日志目录，默认 `runtime_data/logs`；`NUWAX_LOG_FILE` 指定网关主文件，独立组件使用各自的 `nuwax-*.log` 文件。`NUWAX_LOG_MAX_BYTES` 和 `NUWAX_LOG_BACKUP_COUNT` 控制容量轮转。`NUWAX_LOG_LEVEL` 与 `NUWAX_LOG_LEVEL_*` 控制平台日志级别，`RUST_LOG` 控制 Rust 运行进程的过滤。文件始终保存结构化记录，`NUWAX_LOG_FORMAT` 只改变终端格式；目录不可写时会记录原因并回退到可用输出。

## 验证边界

无实机测试覆盖输入适配、算法场景、真实本机通信、地图读写和生命周期。它不代表实机运动、真实场景识别准确率或建图精度已验收。`--gateway` 场景经 MCP 控制建图，并从正式HTTP事件流读取完整地图。
