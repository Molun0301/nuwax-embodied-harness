# GO2_RTABMAP → GO2_RTABNAV2 全量迁移改动清单

- **文档版本**：v0.4（待审批）
- **日期**：2026-06-26
- **作者**：代码库审计（基于当前 `clean-test` 工作区快照）
- **目标**：将平台几何栈外部脚本与 ROS2 工作区从 `services/geometry/ros2_ws/src/GO2_RTABMAP` 全量切换为 `services/geometry/ros2_ws/src/GO2_RTABNAV2`
- **审批用途**：列出所有需改动点、行为差异与待决策项，供负责人审批后分 PR 实施

---

## 0. 正式入口约定（已确认）

**日常手工启动与平台托管，统一使用顶层 `go2.sh`，不使用子目录副本。**

| 角色 | 路径 |
|------|------|
| **唯一正式入口** | `services/geometry/ros2_ws/src/GO2_RTABNAV2/go2.sh` |
| 平台 `.env` 应指向 | 同上（`NUWAX_GEOMETRY_EXTERNAL_SCRIPT`） |
| 默认工作区 | `${SCRIPT_DIR}/ws` → 即 `GO2_RTABNAV2/ws/`（顶层脚本已写死，无需 `GO2_WS=~/GO2_RTABNAV2/ws`） |
| **非正式入口（勿配平台）** | `unitree-go2-slam-nav2/scripts/go2.sh`（开发遗留副本，默认 `GO2_WS=../../ws`） |

手工启动示例（与当前习惯一致）：

```bash
cd services/geometry/ros2_ws/src/GO2_RTABNAV2
./go2.sh --mapping   runtime_data/geometry/rtabmap/xxx.db
./go2.sh --localization  runtime_data/geometry/rtabmap/xxx.db
./go2.sh --explore     runtime_data/geometry/rtabmap/xxx.db
```

平台 `runtime.py` 通过 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 调用的也是**同一条顶层脚本**；`start_mapping` / `explore_area` / `activate_map` 下发的 `--mapping` / `--explore` / `--localization` 与手工一致。

**对迁移文档的影响**：

- P0 改 `.env`：只需把路径从 `GO2_RTABMAP/go2.sh` 换成 **`GO2_RTABNAV2/go2.sh`（顶层）**
- 审批项 **C** 建议默认选「仓库相对」——顶层 `go2.sh` 已是 `${SCRIPT_DIR}/ws`；仅需修 `ws/source_go2.bash` 的 `$HOME` 回退，避免有人 `source` 时跑偏
- `bootstrap` 默认 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 也应生成顶层路径，**不是** `start_rtabmap_sidecar.sh`、**不是** `unitree-go2-slam-nav2/scripts/go2.sh`

---

## 1. 迁移范围摘要

| 维度 | 当前（GO2_RTABMAP） | 目标（GO2_RTABNAV2） |
|------|---------------------|----------------------|
| 入口脚本 | `GO2_RTABMAP/go2.sh` | `GO2_RTABNAV2/go2.sh` |
| ROS 包名 | `go2_nav` | `go2_slam_nav` |
| colcon 工作区 | `GO2_RTABMAP/`（单包） | `GO2_RTABNAV2/ws/`（多包：go2_slam_nav、go2_cmd_processor、explore_lite、frontier…） |
| 自主探索 | 自研 `frontier_explorer.py` | `m-explore`（`explore_lite` / `explore_node`） |
| 平台 `.env` 注入 | `scripts/load_platform_geometry_env.sh` + `geometry_launch_config.py` | **尚无**（需新增或移植） |
| bootstrap 工作区路径 | `src/go2slam`（已不存在） | 应改为 `src/GO2_RTABNAV2/ws` |
| 实机 `.env` 外部脚本 | 指向 `GO2_RTABMAP/go2.sh` | 应改为 `GO2_RTABNAV2/go2.sh` |
| **Debug 2D 主图** | `NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map` | 应改为 **`/map_obstacles`**（路线 B，与 Nav2/explore 一致） |
| **Debug 前沿可视化** | 订阅 `/frontiers`（`frontier_explorer`） | 应改为 **`explore/frontiers`**（`explore_lite`） |

**结论**：路径替换 alone 不够；需同步改 **部署脚本、bootstrap、进程清理、探索终态检测、平台配置注入、Debug 面板地图/前沿数据源** 等行为适配层。

---

## 2. 两栈关键行为差异（影响平台代码）

### 2.1 go2.sh CLI（平台 `runtime.py` 已发命令）

| 能力 | GO2_RTABMAP | GO2_RTABNAV2 | 平台现状 |
|------|-------------|--------------|----------|
| 遥控建图 | `--new` / `--mapping` / `-m` | `--mapping`（无 `--new`） | `runtime.py` 发 `--mapping` ✅ |
| 定位 | `--localization` / `-l` | `--localization` | ✅ |
| 探索 | `--explore` / `-e` | `--explore` | ✅ |
| 探索日志 | `--explore-log`、`--nav2-log-only` | **不支持** | 平台未传这些 flag，无影响 |
| 仅建图无 Nav2 | 无 | `--mapping-only` | 可选新能力，非阻塞 |

### 2.2 探索后端与终态检测（**必须改代码**）

| 项 | GO2_RTABMAP | GO2_RTABNAV2 |
|----|-------------|--------------|
| 探索节点 | `frontier_explorer` | `explore_node`（explore_lite） |
| 完成日志关键字 | `frontier_explorer is idle`、`Exploration complete` | 需确认 `explore_lite` / `nav_progress_monitor` 输出 |
| `runtime.py` PID 正则 | `frontier_explorer\.py-\d+` | 需改为 `explore_node` 或订阅 `/explore/status` |
| metadata 字段 | `native_go2_frontier_explorer: true` | 应改为 `native_go2_explore_lite` 或通用名 |
| `NUWAX_GEOMETRY_EXPLORATION_*` | 经 launch 注入 frontier_explorer | **当前不读取**；探索参数在 `explore_go2.yaml` |

### 2.3 定位会话与数据库

| 项 | GO2_RTABMAP | GO2_RTABNAV2 |
|----|-------------|--------------|
| 默认 DB 路径 | `$HOME/rtabmap.db` | `$HOME/.ros/rtabmap.db` |
| 定位模式 | 直接读传入 `.db` | 复制到 `~/.ros/rtabmap_localization_session.db` 再定位 |
| 平台 DB 根目录 | `runtime_data/geometry/rtabmap/`（经平台 map workspace） | 兼容；需验证 explore/mapping 传参路径 |

### 2.4 平台配置注入（**缺口，需新增工作**）

GO2_RTABMAP 通过以下链路读取根 `.env` 的 `NUWAX_GEOMETRY_*`：

```text
go2.sh → scripts/load_platform_geometry_env.sh → launch/geometry_launch_config.py → nav2_params 物化
```

GO2_RTABNAV2 **没有** 等价脚本；Nav2 参数在包内 `config/nav2_params_go2.yaml`，**不随平台 `.env` 热更新**。

**审批项 A**：是否在 `GO2_RTABNAV2/go2.sh` 移植 `load_platform_geometry_env.sh`，并实现 `nav2_params` 物化（对齐 `NUWAX_GEOMETRY_ROBOT_RADIUS` 等）？

### 2.5 ROS Bridge env.sh 回退路径

`services/geometry/ros_bridge_client.py` 在 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 父目录查找 `scripts/env.sh`：

```text
{external_script_parent}/scripts/env.sh
```

- GO2_RTABMAP：`GO2_RTABMAP/scripts/env.sh` ✅ 存在
- GO2_RTABNAV2：无 `scripts/env.sh` ❌

**审批项 B**：在 `GO2_RTABNAV2/scripts/env.sh` 新增薄封装，或改 `ros_bridge_client.py` 解析逻辑？

### 2.6 工作区路径硬编码

| 文件 | 问题 |
|------|------|
| `GO2_RTABNAV2/ws/source_go2.bash` | 默认 `GO2_WS=${HOME}/GO2_RTABNAV2/ws`，非仓库内嵌路径 |
| `GO2_RTABNAV2/ws/rebuild_after_rename.sh` | 硬编码 `/home/unitree/GO2_RTABNAV2/...` |
| `GO2_RTABNAV2/go2.sh` | 默认 `GO2_WS=${SCRIPT_DIR}/ws` ✅ 正确 |

**审批项 C**：统一 `source_go2.bash` / `rebuild_after_rename.sh` 为 **仓库相对路径**（`SCRIPT_DIR/ws`）？

### 2.7 GO2_NAV_* 环境变量

`.env` 中的 `GO2_NAV_AUTO_MAP_YAML`、`GO2_NAV_LOCALIZATION_FROZEN_GRID` 由 GO2_RTABMAP launch 读取；GO2_RTABNAV2 使用不同 launch 参数（`localize_only`、`restart_map`、`slam_mode`）。

**审批项 D**：这些键是否废弃，还是映射到 GO2_RTABNAV2 launch 参数？

---

## 3. 必改文件清单（按优先级）

### P0 — 不改则无法启动 / 部署失败

| # | 文件 | 当前引用 | 建议改动 |
|---|------|----------|----------|
| 1 | `.env` | `NUWAX_GEOMETRY_EXTERNAL_SCRIPT=.../GO2_RTABMAP/go2.sh` | 改为 `.../GO2_RTABNAV2/go2.sh` |
| 2 | `.env.bak` | 同上 | 同步（或删除备份） |
| 3 | `scripts/bootstrap.py` | `GO2SLAM_WS_DIR = .../src/go2slam` | 改为 `.../src/GO2_RTABNAV2/ws`；重命名 `go2slam_*` → `go2_rtabnav2_*` |
| 4 | `scripts/bootstrap.py` | `GO2SLAM_RUNTIME_REQUIRED_PACKAGES = (go2_navigation2, go2_slam, ...)` | 改为 `(go2_slam_nav, go2_cmd_processor, explore_lite, rtabmap_slam, ...)` |
| 5 | `scripts/bootstrap.py` | `build_go2slam` CLI / check 文案 `src/go2slam` | 更新路径与包名；默认 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 指向 `GO2_RTABNAV2/go2.sh` |
| 6 | `scripts/start_gateway.sh` | `geometry_root=.../go2slam`；清理模式 `go2_nav/*` | 改为 `GO2_RTABNAV2`；清理模式对齐 `go2_slam_nav`（参考 `GO2_RTABNAV2/go2.sh` `kill_stale`） |

### P1 — 平台运行时行为正确性

| # | 文件 | 改动说明 |
|---|------|----------|
| 7 | `services/geometry/runtime.py` | 日志/消息字符串 `GO2_RTABMAP` → `GO2_RTABNAV2`（约 20+ 处） |
| 8 | `services/geometry/runtime.py` | `exploration_mode` / `exploration_runtime` 元数据：`go2_rtabmap_script` → `go2_rtabnav2_script` |
| 9 | `services/geometry/runtime.py` | `_read_external_frontier_ros_logs`：PID 正则从 `frontier_explorer.py` 改为 `explore_node` |
| 10 | `services/geometry/runtime.py` | `_external_exploration_terminal_from_log_text`：增加 `explore_lite` / `explore_node` 完成标记 |
| 11 | `services/geometry/external_debug_observer.py` | `go2_rtabmap_debug_observer` 命名 → `go2_rtabnav2_debug_observer` |
| 12 | `services/geometry/ros_bridge_client.py` | `_resolve_external_env_script`：支持 GO2_RTABNAV2 无 `scripts/env.sh` 情况 |
| 13 | `GO2_RTABNAV2/go2.sh` | 新增 `source scripts/load_platform_geometry_env.sh`（移植自 RTABMAP） |
| 14 | `GO2_RTABNAV2/go2.sh` 或 launch | 实现 `NUWAX_GEOMETRY_*` → `nav2_params_go2.yaml` 物化（若审批项 A=是） |
| 15 | `GO2_RTABNAV2/ws/source_go2.bash` | 默认 `GO2_WS` 改为 `${SCRIPT_DIR}` 仓库内 `ws/` |
| 16 | `GO2_RTABNAV2/ws/rebuild_after_rename.sh` | 路径改为仓库相对；symlink 指向 `GO2_RTABNAV2/unitree-go2-slam-nav2/...` |

### P2 — 配置、文档、示例同步

| # | 文件 | 改动说明 |
|---|------|----------|
| 17 | `.env` | 注释中 `对齐 GO2_RTABMAP nav2_params.yaml` → `go2_slam_nav/config/nav2_params_go2.yaml` |
| 18 | `.env.example` | 补充默认 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 示例路径；更新 `GO2_NAV_*` 说明 |
| 19 | `drivers/go2/data_plane.env.example` | 注释路径 `GO2_RTABMAP/src/go2_nav/...` → `GO2_RTABNAV2/...` |
| 20 | `docs/公开文档/部署文档.md` | 全文 `go2slam` → `GO2_RTABNAV2/ws`；构建命令与包名更新 |
| 21 | `2026-05-25_RTAB-MAP+Nav2平台接入评测与交接文档.md` | 历史引用更新或标注 deprecated |
| 22 | `services/evaluation/service.py` | `"go2slam" in text` → 增加 `"go2_rtabnav2"` / `"go2_slam_nav"` |
| 23 | `gateways/runtime.py` | 能力描述中 `go2.sh` 行为说明可补充 `--mapping-only`（可选） |
| 24 | `contracts/geometry_runtime.py` | `build_default_external_slam_nav2_script()` 默认仍指向 `start_rtabmap_sidecar.sh`；若 Go2 正式路径统一为 `go2.sh`，考虑改默认值或加注释 |
| 25 | `README.md` | 补充：geometry 工作区为 `GO2_RTABNAV2/ws`，构建命令示例 |

### P3 — GO2_RTABNAV2 子仓库自洽（迁移前建议先修）

| # | 文件 | 问题 |
|---|------|------|
| 26 | `GO2_RTABNAV2/ws/rebuild_after_rename.sh` | 绝对路径 `/home/unitree/GO2_RTABNAV2` |
| 27 | `GO2_RTABNAV2/ws/rebuild_after_rename.log` | 开发机路径残留（可 gitignore，不提交） |
| 28 | `GO2_RTABNAV2/unitree-go2-slam-nav2/scripts/go2.sh` | 与顶层重复；**不作为平台入口**；可删或加注释指向顶层 |
| 29 | `GO2_RTABNAV2/unitree-go2-slam-nav2/scripts/source_go2.bash` | 同 #15 |
| 30 | `GO2_RTABNAV2/m-explore-ros2/` | 确认作为 vendored 依赖纳入构建 |
| 31 | `GO2_RTABNAV2/ws/install/` | 是否纳入 git 或仅 bootstrap 构建（审批项 E） |

### P4 — 可删除 / 归档（审批后执行）

| # | 路径 | 说明 |
|---|------|------|
| 32 | `services/geometry/ros2_ws/src/GO2_RTABMAP/` | 全量迁移验收通过后删除或移至 `archive/` |
| 33 | 文档中对 `go2slam` 工作区的所有引用 | 三层历史命名统一清理 |

**不在本次平台改动范围**（GO2_RTABNAV2 内部文档/日志）：

- `GO2_RTABMAP/项目架构总览.md`、`PROJECT_OVERVIEW.html`、`1.md` 等 — 随目录删除

---

## 4. bootstrap.py 详细改动点（`scripts/bootstrap.py`）

当前代码仍指向**已不存在**的 `src/go2slam`，与实机 `.env`（指向 GO2_RTABMAP）不一致。迁移需一次性对齐：

```python
# 现状
GO2SLAM_WS_DIR = GEOMETRY_WS_DIR / "src" / "go2slam"
GO2SLAM_RUNTIME_REQUIRED_PACKAGES = ("go2_navigation2", "go2_slam", "go2_driver", "go2_perception")

# 建议
GO2_RTABNAV2_WS_DIR = GEOMETRY_WS_DIR / "src" / "GO2_RTABNAV2" / "ws"
GO2_RTABNAV2_RUNTIME_REQUIRED_PACKAGES = (
    "go2_slam_nav",
    "go2_cmd_processor",
    "explore_lite",
    # "frontier",  # 若 explore_go2 不依赖可省略
)
```

涉及函数/符号（建议重命名或保留别名过渡期）：

- `go2slam_setup_path()` → `ws/install/setup.bash`
- `find_go2slam_stale_source()` → 扫描 `GO2_RTABNAV2/ws/src` + `unitree-go2-slam-nav2`
- `missing_go2slam_runtime_packages()` → 检查 `go2_slam_nav` 等
- `build_go2slam_apt_packages()` → 更新 ROS 依赖（去掉旧 unitree C++ 包）
- `assert_go2slam_workspace_ready()` / `assert_go2slam_runtime_ready()`
- CLI：`--build-go2slam` → `--build-go2-rtabnav2`（或保留旧 flag 作 deprecated 别名）
- `apply` 生成的 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 默认值（当前为 `start_rtabmap_sidecar.sh`，实机用 `go2.sh`）

---

## 5. start_gateway.sh 进程清理改动

当前 `cleanup_external_geometry_processes()`：

```bash
geometry_root="${REPO_ROOT}/services/geometry/ros2_ws/src/go2slam"
go2_patterns=(
  "ros2 launch go2_nav"
  "go2_nav/(odom_tf_broadcaster|cloud_to_scan|cmd_vel_bridge|frontier_explorer)[.]py"
  ...
)
```

建议改为（与 `GO2_RTABNAV2/go2.sh` `kill_stale` 对齐）：

```bash
geometry_root="${REPO_ROOT}/services/geometry/ros2_ws/src/GO2_RTABNAV2"
go2_patterns=(
  "ros2 launch go2_slam_nav"
  "lib/go2_slam_nav/sensor_stamp_relay"
  "lib/go2_slam_nav/obstacle_grid_projector"
  "lib/go2_slam_nav/go2_tf_relay"
  "lib/go2_cmd_processor/sport_ctrl"
  "explore_node"
  "explore_preroll"
  "nav2_container"
  ...
)
```

---

## 6. runtime.py 探索终态检测改动草案

### 6.1 日志标记扩展

在 `_external_exploration_terminal_from_log_text()` 的 `complete_markers` 增加：

```python
"exploration finished",           # explore_lite 常见
"no more frontiers",              # m-explore
"explore_node",                   # 视实机日志而定
"preroll warmup complete",        # 仅预热完成，非终态 — 需排除
```

需实机抓取 `explore_go2.launch.py` 一次完整探索的 `~/.ros/log` 样本后定稿。

### 6.2 PID 检测

```python
# 旧
r"\[frontier_explorer\.py-\d+\]: process started with pid \[(\d+)\]"
# 新（候选）
r"\[explore_node-\d+\]: process started with pid \[(\d+)\]"
# 或订阅 ExploreStatus topic（更稳，工作量大）
```

### 6.3 元数据字段

| 字段 | 旧值 | 建议新值 |
|------|------|----------|
| `exploration_mode` | `go2_rtabmap_script` | `go2_rtabnav2_script` |
| `exploration_runtime` | `external_go2_rtabmap` | `external_go2_rtabnav2` |
| `native_go2_frontier_explorer` | `true` | `false` |
| `native_go2_explore_lite` | — | `true`（新增） |

---

## 7. 配置项迁移对照

| 配置键 | GO2_RTABMAP 消费方 | GO2_RTABNAV2 现状 | 动作 |
|--------|-------------------|-------------------|------|
| `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` | go2.sh | 需指向新路径 | **改 .env** |
| `NUWAX_GEOMETRY_ODOM_TOPIC` | geometry_launch_config | launch 内硬编码/参数 | 移植 env 注入 |
| `NUWAX_GEOMETRY_POINT_CLOUD_TOPIC` | 同上 | `GO2_CLOUD_MODE` deskewed/base | 映射或统一 |
| `NUWAX_GEOMETRY_ROBOT_RADIUS` 等 Nav2 | materialize_nav2_params | nav2_params_go2.yaml | 移植物化逻辑 |
| `NUWAX_GEOMETRY_EXPLORATION_*` | frontier_explorer | explore_go2.yaml | 文档标注或移植 |
| `GO2_NAV_LOCALIZATION_FROZEN_GRID` | go2_nav launch | 无直接对应 | 审批项 D |
| `GO2_NAV_AUTO_MAP_YAML` | go2_nav launch | 无直接对应 | 审批项 D |
| `NUWAX_GEOMETRY_SETUP_SCRIPT` | 平台 geometry ws | 仍指向 `ros2_ws/install` | 确认是否还需平台 ws |

---

## 8. 建议实施阶段（审批后可拆 PR）

```text
Phase 0 — GO2_RTABNAV2 自洽
  ├─ 修 source_go2.bash / rebuild 脚本路径
  ├─ colcon build GO2_RTABNAV2/ws 在实机通过
  └─ 手工 ./go2.sh --mapping|--localization|--explore 验收

Phase 1 — 配置与部署入口（P0）
  ├─ .env / bootstrap.py / start_gateway.sh
  └─ bootstrap check + apply 无报错

Phase 2 — 平台运行时适配（P1）
  ├─ runtime.py 探索终态 + 文案
  ├─ external_debug_observer / ros_bridge_client
  └─ 移植 load_platform_geometry_env（若审批 A=是）

Phase 3 — 文档与清理（P2/P4）
  ├─ 部署文档、交接文档、README
  └─ 删除 GO2_RTABMAP（若审批通过）
```

---

## 9. 验收标准（建议）

| # | 场景 | 通过条件 |
|---|------|----------|
| 1 | `bootstrap.py check --robot go2 --geometry` | 识别 `GO2_RTABNAV2/ws/install/setup.bash` 与 `go2_slam_nav` 包 |
| 2 | `start_gateway.sh` 启动 | 无残留 `go2_nav` 误杀/漏杀 |
| 3 | `activate_map` + `start_mapping` | `go2.sh --mapping` 拉起，`/map`、`/tf` 就绪 |
| 4 | `explore_area` | 探索任务终态事件 `task.succeeded`，非超时 |
| 5 | `stop_mapping` | 几何栈进程全部退出 |
| 6 | MCP `navigate_to_pose` | Nav2 action 到点 |
| 7 | `/debug` 面板 | 地图、位姿、导航摘要正常 |
| 8 | 配置热更新 | 改 `.env` 中 `NUWAX_GEOMETRY_ROBOT_RADIUS` 后重启生效（若审批 A=是） |

---

## 10. 待审批决策项汇总

| ID | 问题 | 选项 |
|----|------|------|
| **A** | 是否移植 `NUWAX_GEOMETRY_*` → Nav2 参数物化到 GO2_RTABNAV2？ | 是（推荐）/ 否（仅用包内 yaml） |
| **B** | Rust bridge `env.sh` 回退路径 | 新增 `GO2_RTABNAV2/scripts/env.sh` / 改 Python 解析 |
| **C** | `source_go2.bash` 默认工作区 | **推荐：仓库相对**（与顶层 `go2.sh` 的 `${SCRIPT_DIR}/ws` 一致）；`$HOME/GO2_RTABNAV2/ws` 仅作兼容回退 |
| **D** | `GO2_NAV_*` 环境变量 | 废弃 / 映射到新 launch 参数 |
| **E** | `GO2_RTABNAV2/ws/install` 是否入库 | 仅 bootstrap 构建 / 提交 install 产物 |
| **F** | 旧目录 `GO2_RTABMAP` | 验收后删除 / 保留只读归档 |
| **G** | bootstrap CLI `--build-go2slam` | 重命名 / 保留 deprecated 别名 |
| **H** | 探索后端元数据命名 | `explore_lite` / 保持泛化 `external_go2_exploration` |
| **I** | Debug 主 2D 图显示哪张栅格？ | **仅 `/map_obstacles`**（推荐）/ 双图层可切换（Route B + Raw `/map`） |
| **J** | Rust bridge `NUWAX_MAP_TOPIC` 是否同步改？ | 仅 debug 改 observer / **全局改** occupancy_topic（影响 gRPC 地图流） |

---

## 11. Debug 面板全量审计（GO2_RTABNAV2 适配）

> **话题名纠正**：GO2_RTABNAV2 路线 B 的主规划栅格是 **`/map_obstacles`**（复数），不是 `/map_obstacle`。原始 RTAB 栅格仍为 **`/map`**。

### 11.1 当前数据流（为何 Debug 与实机不一致）

```text
GO2_RTABNAV2 实机
  RTAB-Map ──► /map（原始占据栅格，RViz Map2D_Raw）
  obstacle_grid_projector ──► /map_obstacles + /map_obstacles_updates（路线 B，Nav2/explore 用这张）

平台 Debug 订阅链（今天）
  .env NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map
    → ExternalGeometryDebugObserver 订阅 /map
    → StreamBus map.frame
    → debug_panel.html 画布渲染 occupancy_grid

结果：Debug 显示的是「原始 SLAM 栅格」，Nav2/探索实际用的是「融合障碍图」——二者不一致。
```

### 11.2 受影响模块一览

| 模块 | 当前行为 | 迁移后应改为 |
|------|----------|--------------|
| `.env` / `.env.example` | `NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map` | **`/map_obstacles`** |
| `core/settings.py` 默认值 | `/map` | `/map_obstacles`（或保留 `/map` 仅作 raw 副 topic 配置项） |
| `services/geometry/external_debug_observer.py` | 订阅 `occupancy_topic`（现 `/map`）；前沿 `/frontiers` | 主图 `/map_obstacles`；前沿 **`explore/frontiers`** |
| `services/geometry/nuwax_ros_bridge` | `NUWAX_MAP_TOPIC` ← `NUWAX_GEOMETRY_OCCUPANCY_TOPIC` | 审批项 **J**：是否 gRPC 地图也切到 `/map_obstacles` |
| `services/geometry/runtime.py` | 就绪文案写「等待 /map 首帧」；`go2_rtabmap_debug_observer` 元数据 | 文案与 source 命名改为 RTABNAV2 / `/map_obstacles` |
| `gateways/http/debug_panel.py` | `message_count_warning` 硬编码「/map」 | 动态显示 `source_topic` |
| `gateways/http/static/debug_panel.html` | 地图元数据无「订阅 topic」行；图例无 Route B 说明 | 增加 topic 展示；可选 Raw/RouteB 图层切换 |
| `scripts/bootstrap.py` 生成默认 | `NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map` | `/map_obstacles` |

### 11.3 Debug 面板分区审查

| 分区 | 数据源 | GO2_RTABNAV2 风险 | 优化建议 |
|------|--------|-------------------|----------|
| **2D 地图画布** | `map.frame` ← observer `/map` | 与 Nav2 所见不一致 | P0：改 occupancy 为 `/map_obstacles` |
| **地图摘要** (`#mapMeta`) | 同上 | 无 topic 字段，难排查 | 增加「订阅 topic」「图层角色（Route B）」 |
| **2D Goal / Nav2** | `dispatch_debug_nav2_goal` → Nav2 action | 一般仍可用（Nav2 已起） | 保持；确认 costmap 用 `/map_obstacles` 后路径合理 |
| **前沿叠加** | `exploration.frontier` ← `/frontiers` MarkerArray | **explore_lite 发 `explore/frontiers`**，现订阅收不到 | P1：observer 改 frontier topic；或解析 `explore/status` |
| **前沿提示** (`#frontierNotice`) | 「等待探索前沿候选几何」 | 长期空 | 探索模式改为「m-explore / explore_lite」说明 |
| **导航态势** | `navigation.state` + `exploration.state` | 探索终态检测仍盯 `frontier_explorer` 日志 | 见 §6 + `explore/status` |
| **位姿/轨迹** | TF `map→odom→base` | frame 可能为 `go2_base_link` / `go2_odom` | 验证 TF 链与 `NUWAX_GEOMETRY_BASE_FRAME` |
| **3D 点云地图** | `pointcloud_map.*` / `/cloud_map` | 与 2D 主图独立 | 保持；标注 3D 与 2D Route B 分工 |
| **视频/感知** | `video.frame` SHM | 无 RTABNAV2 直接冲突 | 保持 |
| **进程矩阵** | `runtime.governance` | 进程名从 `go2_nav` → `go2_slam_nav` | 更新匹配模式/展示标签 |
| **配置页** | 读网关配置 | `OCCUPANCY_TOPIC` 显示 `/map` | 随 .env 更新 |

### 11.4 建议实现方案（审批项 I）

**方案 1（推荐，改动小）**：单主图

```dotenv
NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map_obstacles
NUWAX_GEOMETRY_EXTERNAL_FRONTIER_TOPIC=explore/frontiers   # 新增配置键（待实现）
```

- Debug 画布 = Nav2 同款融合图
- 原始 `/map` 仅在 RViz 或后续「高级图层」查看

**方案 2（体验更好，改动中）**：双图层可切换

| 配置键 | 默认 | 用途 |
|--------|------|------|
| `NUWAX_GEOMETRY_OCCUPANCY_TOPIC` | `/map_obstacles` | Debug 默认主图 + Rust bridge（若 J=全局改） |
| `NUWAX_GEOMETRY_RAW_OCCUPANCY_TOPIC` | `/map` | Debug 可选「原始 RTAB」半透明底图 |

- `external_debug_observer` 双订阅，payload 分 `occupancy_grid` / `raw_occupancy_grid`
- `debug_panel.html` 工具栏增加「Route B / Raw」切换

### 11.5 Debug 相关必改文件（追加到 §3）

| 优先级 | 文件 | 改动 |
|--------|------|------|
| P0 | `.env` / `.env.example` | `NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map_obstacles` |
| P0 | `services/geometry/external_debug_observer.py` | frontier topic → `explore/frontiers`；metadata 命名去 `rtabmap` |
| P1 | `gateways/http/debug_panel.py` | warning/诊断文案用动态 topic |
| P1 | `gateways/http/static/debug_panel.html` | `mapMeta` 增加 topic；前沿区文案；可选图层切换 UI |
| P1 | `contracts/geometry_runtime.py` | 可选新增 `raw_occupancy_topic`、`frontier_topic` 契约字段 |
| P1 | `core/settings.py` | 读取新配置键 |
| P2 | `services/geometry/runtime.py` | `/map` 就绪提示 → 可配置 topic 名 |
| P2 | `scripts/bootstrap.py` | 默认 occupancy `/map_obstacles` |

### 11.6 go2.sh 日志汇入主日志（已实现）

平台通过 `_ManagedProcess` 托管 `go2.sh`（进程名 `external_slam_nav2`）时：

- **侧车日志**仍写入 `runtime_data/geometry/logs/external_slam_nav2.log`（供探索终态扫描）
- **同步镜像**每一行 stdout/stderr 到 `NUWAX_LOG_FILE`（默认 `nuwax_robot_bridge.log`）
- 主日志 logger 名为 `[nuwax_robot_bridge.geometry.go2.sh]`，格式与网关日志一致

手工在终端直接 `./go2.sh` 不经过平台托管，不会自动进主日志（需自行重定向或走 `start_mapping` / `explore_area`）。

### 11.7 Debug 验收（追加到 §9）

| # | 检查项 | 通过条件 |
|---|--------|----------|
| D1 | 2D 地图与 Nav2 一致 | Debug 画布障碍分布与 RViz `Map2D_RouteB`（`/map_obstacles`）一致 |
| D2 | 探索前沿 | `--explore` 时 Debug 地图上有前沿标记（来自 `explore/frontiers`）或摘要显示 `frontiers_available` |
| D3 | 地图摘要 | `#mapMeta` 显示 `source_topic=/map_obstacles` |
| D4 | 2D Goal | 点击设目标后 Nav2 有规划线（`/plan`） |
| D5 | 无旧栈幽灵 | 未启动几何时不长期显示 `/map` 缓存帧误导为「已就绪」 |

---

## 12. 文件索引（全文检索命中汇总）

### 11.1 仓库根 & 配置

- `.env`（L43-46, L86-89）
- `.env.bak`
- `.env.example`（L272 GO2_NAV 注释）

### 11.2 脚本 & 入口

- `scripts/bootstrap.py`（全文 `go2slam` / `GO2SLAM_*`）
- `scripts/start_gateway.sh`（L242-258）

### 11.3 平台服务

- `services/geometry/runtime.py`（GO2_RTABMAP 文案、探索检测、GO2_NAV env）
- `services/geometry/external_debug_observer.py`
- `services/geometry/ros_bridge_client.py`（env.sh 回退）
- `services/evaluation/service.py`（L788）
- `contracts/geometry_runtime.py`（`start_rtabmap_sidecar.sh` 默认）

### 11.4 网关 & 驱动

- `gateways/runtime.py`（go2.sh 能力描述，无需路径改，可选文案）
- `drivers/go2/data_plane.env.example`（L82-84）

### 11.5 文档

- `docs/公开文档/部署文档.md`
- `2026-05-25_RTAB-MAP+Nav2平台接入评测与交接文档.md`
- `README.md`（间接：Go2SLAM 工作区提及，建议补充）

### 11.6 不改动（仅随 GO2_RTABMAP 删除）

- `services/geometry/ros2_ws/src/GO2_RTABMAP/**` 全部

### 11.7 迁移目标树（需在 Phase 0 构建通过）

```text
services/geometry/ros2_ws/src/GO2_RTABNAV2/
├── go2.sh                          # ★ 唯一正式入口（手工 + 平台 NUWAX_GEOMETRY_EXTERNAL_SCRIPT）
├── ws/
│   ├── source_go2.bash
│   ├── rebuild_after_rename.sh
│   └── install/setup.bash          # bootstrap 检查目标
├── unitree-go2-slam-nav2/
│   └── go2_slam_nav/               # ROS 包
├── m-explore-ros2/                 # explore_lite 依赖
└── (可选) scripts/
    ├── env.sh                      # Rust bridge 回退
    └── load_platform_geometry_env.sh
```

---

## 13. 风险评估

| 风险 | 等级 | 缓解 |
|------|------|------|
| 探索终态检测失效导致 `explore_area` 永不 complete | 高 | Phase 0 实机抓日志 + Phase 2 单元测试 mock |
| Nav2 参数与旧栈不一致导致导航行为变化 | 中 | 对比 `nav2_params.yaml` vs `nav2_params_go2.yaml` diff |
| bootstrap 构建失败（symlink/m-explore） | 中 | 固化 `rebuild_after_rename.sh` 为仓库相对 |
| 定位 DB 复制语义变化导致地图损坏 | 中 | 验收 `load_map_version` + `--localization` 全流程 |
| 双栈并存期间路径混淆 | 低 | 审批后立即改 `.env`，删除 GO2_RTABMAP |

---

*本文档仅列出改动点与审批项，不包含实际代码修改。审批通过后建议按 Phase 0→3 拆 PR 实施。*