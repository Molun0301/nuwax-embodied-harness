# Phase 7 — feedback（installer / systemd / 实机链路）

> **归属执行方案**：`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 7 段
>
> **本文件用途**：落盘 Phase 7 实施细节、未完成 blocker、跨阶段一致性、给后续 lane 的建议。
>
> **Why**：避免实现漂移；任何 reviewer 看本文件即可知道 Phase 7 完成了什么 / 没完成什么 / 为什么。

---

## A. 实施细节

### A.1 installer phase 复用 Phase 0-6 已落盘的设计，不新增 phase

**Why**：Phase 0-6 已经把 installer 收敛到目标态（preflight + system + python_env + nuwaclaw + geometry_runtime_deps + models + health + mcp + base + systemd + config_env）。Phase 7 复核发现：

- `preflight.py:66-108` 已探测 rustc/cargo/rustup/liblcm/nuwax-workspace
- `system.py:42-54` dry_run 已含 `liblcm-dev/pkg-config/build-essential`
- `geometry_runtime_deps.py:53-97` 已校验 nuwax-geometry-runtime 产物路径

无需新增 phase，无需修改 liblcm-dev 安装步骤（已存在）。

**How to apply**：未来扩展（如加 cuda / 加 python-ros-bridge 替代品）应**修改**现有 phase 而非新增；保留 9 个 phase 上限，便于 wizard/CLI 编排。

### A.2 evidence bundle 设计哲学

**Why**：grpcio / grpcurl / lcm-logger 都不是 dev 机的标准库；强行依赖会让脚本在大多数 dev 机器上崩。

`check_nuwax_runtime.sh` 采用"**降级而非失败**"哲学：

| 段 | 不可用时行为 |
|---|---|
| 1 no_ros2_residual | 本脚本硬依赖；非 0 退出时报"missing" |
| 2 二进制 | 缺则"MISSING"提示实机 build |
| 3 端口 | ss/netstat 都缺则"NOT RUN" |
| 4 ready file | 缺则"NOT RUN" |
| 5 SHM refs | 缺则"NOT RUN" |
| 6 gRPC status | grpc 模块缺则"GRPC_NOT_AVAILABLE" + 提示 pip install |
| 7 env canonical | 全 unset 也正常（gateway 启动前本来就 unset） |
| 8 日志尾部 | 缺文件则"NOT RUN" |

**核心**：脚本**永远 EXIT 0**（evidence 全部产出，允许若干段 NOT READY）；只有 no_ros2_residual 失败时报 blocking。守门层和观测层分离。

**How to apply**：实机冒烟恢复后，目标机 Python 3.8.10 自带 grpcio 1.70.0（memory `realmachine-python-env.md`），§6 输出真实 gRPC status；dev 机可选 `pip install grpcio protobuf` 触发完整 evidence。

### A.3 systemd unit 与现有 gateway service 同形态

**Why**：`nuwax-host-gateway.service` 已被实机验证可用（target: unitree 用户）；`nuwax-geometry-runtime.service` 必须与之同形态（同一 WorkingDirectory/User/Group）才能在生产环境与 start_gateway.sh 协同工作。

`SuccessExitStatus=0 4` 软化 LCM backend fail-fast 退出码（main.rs:148-158 `return ExitCode::from(4)`），避免 LCM backend 启动期 fail-fast 时 systemd 反复重启。

**How to apply**：

- 若未来加 CUDA / GStreamer 等额外依赖，需在 unit 加 `After=` / `Wants=`
- 若 unit 启动顺序需强一致（如 gateway 必须在 runtime 就绪后才能启动），可加 `After=nuwax-geometry-runtime.service` + `Wants=` 到 gateway.unit；当前实现中 gateway 自己拉 runtime，不需要反过来
- unitree 用户的 SSH 配置 / sudo 权限需在目标机 `install.sh` 落地时同步建立

### A.4 启动契约仅一处理论不一致

**Why**：start_gateway.sh 等待 ready_file 上限硬编码 10s（L407），start_nuwax_geometry_runtime.sh 默认 30s（L30）。

**Phase 7 决定不修**：

- 硬 10s 是早于本执行方案的历史值；保留向后兼容
- runtime 启动器的 `NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT` 仅在脱离 gateway 独立调用时生效
- gateway 后续若要放宽（real-lcm 启动可能 > 10s），应单独开 lane

**How to apply**：实机冒烟若发现 10s 内 ready_file 未出现，先排查 runtime 启动慢的根因（real-lcm 握手/nuwax-mapping 初始化），不是放宽 gateway 超时。

---

## B. 跨阶段一致性

### B.1 Phase 6 落盘的 env 全部保留

- `NUWAX_EXPLORATION_LEGACY` — explore split-brain guard
- `NUWAX_NAV_PROFILE` — 6 profile 常量（simple-2d/astar/replanning-astar/cmu-native/exploration-native/mls-3d）
- `NUWAX_EXPLORE_ARBITRATION` — exploration 仲裁策略
- `NUWAX_REAL_LCM_REQUIRED` — real-lcm 启动期 fail-fast 开关

`check_nuwax_runtime.sh` §7 显式列出这 4 个 env（即使 unset），便于运维快速核对。

### B.2 Phase 0 旧命名 alias 0 命中（rg gate 1/2）

```
rg '_nav2_action_provider.*return|return getattr(self, "_nav2_action_provider")|force_nav2_goal|debug_nav2_goal'
  services/geometry/runtime.py gateways/runtime.py → 0 命中
rg '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr(self, "_platform_exploration_runtime")|NUWAX_GEOMETRY_RUNTIME_ARGS'
  services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py → 0 命中
```

执行方案 §7.1 第 1 条要求"`_nav2_action_provider` 不得作为任何 fallback backend 返回"，守门确认 0 命中。

### B.3 installer / systemd 不引入 ROS2

`system.py:51-54` dry_run 列表：

```
+ sudo apt-get update
+ sudo apt-get install -y build-essential pkg-config curl git
+ sudo apt-get install -y liblcm-dev
+ rustup target add aarch64-unknown-linux-gnu（仅当 NUWAX_TARGET_ARCH=aarch64）
```

0 ROS2 包 / 0 rosdep / 0 colcon。

systemd unit ExecStart = `start_nuwax_geometry_runtime.sh`，启动器本身禁止 `source /opt/ros/*/setup.bash`（参见 `services/geometry/scripts/start_nuwax_geometry_runtime.sh:1-16` 不变量段）。

---

## C. 未完成 blocker（本期不开）

| Blocker ID | 状态 | 描述 | 责任方 | 关闭条件 |
|---|---|---|---|---|
| **BLK-PHASE7-1** | NOT RUN | 目标机 SSH 不通（192.168.31.245 / 192.168.123.18 双双 Connection timed out）；HealthCheck 10s 内 ready / LCM 6 channel 观测 / Go2 全链路 全部 NOT RUN | 实机冒烟 lane | 目标机网络恢复后由实机冒烟 lane 补 §3.1 |
| **BLK-PHASE7-2** | SKIP | dev 机无 cargo；`cargo +1.88.0 check/test/tree` 三件套无法在本仓执行；`cargo tree` 用 stub 路径已验 0 命中（rg 路径字符串 0 命中），源码层无 ROS2/DDS/RMW 依赖 | CI 镜像 / 实机冒烟 lane | CI 镜像补 rustup 1.88 + 实机 build_workspace 阶段 |
| **BLK-PHASE7-3** | NOT RUN | 本机系统 python3 无 grpcio；evidence bundle §6 gRPC status 输出 `GRPC_NOT_AVAILABLE`；实机冒烟前需 `pip install grpcio protobuf` 或用 .venv_model_download | 实机冒烟 lane | dev 机可选；目标机（Python 3.8.10）按 memory `realmachine-python-env.md` 应自带 grpcio 1.70.0 |
| **BLK-PHASE7-4** | OPEN | real-lcm backend + MurmurHash64A fingerprint 切换依赖 dimos PR；本期不开 | dimos 侧 | 已在 `2026-07-01_.../07_最终删除与实机验收报告.md` BLK-FINAL-1 归档，本 phase 不重复 |
| **BLK-PHASE7-5** | OPEN | `nuwax-nav-stack` 默认 profile 落地（simple-2d/astar 之外）依赖 dimos PR；本期不开 | dimos 侧 | 已在 `2026-07-01_.../07_最终删除与实机验收报告.md` BLK-FINAL-2 归档 |

---

## D. 给后续 lane 的建议

### D.1 实机冒烟 lane 接手时

1. **首选验证**：SSH 到 Go2 192.168.31.245 后 `bash scripts/diagnostics/check_nuwax_runtime.sh` 输出 §6 真实 gRPC status
2. **不要**先尝试 unit / docker / 远程 cargo build — Phase 7 的 systemd unit 是给目标机用的，不是给 dev 容器用的
3. **LCM channel 观测**：用 `lcm-logger` 订阅 6 类 channel（odom/tf/pointcloud/occupancy/nav_state/frontier），归档 .log 文件到 `docs/开发文档/建图导航探索重构/2026-07-06_.../lcmlogs/`
4. **Go2 全链路**：建图（rtabmap_cli 启 → 点云输入 → 占用栅格 revision）→ 定位（map→odom→base tf chain）→ 探索（frontier discovered → goal handoff）→ 导航（SetNavigationGoal → succeeded / cancelled）→ cmd_vel zero（cancel 后 < 200ms 输出 0）

### D.2 CI 镜像 lane 接手时

1. 镜像加 `rustup 1.88.0` + `liblcm-dev` + `pkg-config`
2. CI job 加 `cargo +1.88.0 check/test/tree` 三件套
3. CI job 加 `pip install grpcio protobuf` 让 evidence bundle §6 跑完整
4. 当前 dev / CI 无 cargo，rg gate 1/2 已是**源码层**最优守门

### D.3 dimos PR 联动

1. **real-lcm PR 落地** → 切 fingerprint → nuwax-geometry-runtime 默认接入；关闭 BLK-PHASE7-4
2. **nuwax-nav-stack PR 落地** → 默认 profile 切 native → runtime_diagnostics.rs `nav_ready=true`；关闭 BLK-PHASE7-5

---

## E. 自我反省（参考 07_最终删除与实机验收报告.md §5.4）

**本 phase 教训**：

1. **不要"全部完成"声明**：本期 14 PASS + 3 NOT RUN + 1 SKIP，必须如实标注 NOT RUN（实机不通）/ SKIP（dev 机无 cargo）。Reviewer 看 §0 验收总览即可知完成度。
2. **installer 复用而非新增**：Phase 7 没新增任何 installer phase — 现有 preflight/system/geometry_runtime_deps 已覆盖。避免"phase 越来越多"的熵增。
3. **降级而非失败**：evidence bundle 在 grpc 缺失时不报 traceback，而是输出 `GRPC_NOT_AVAILABLE` 单行，让 reviewer 一眼看出根因。
4. **systemd unit 软化 fail-fast**：用 `SuccessExitStatus=0 4` 让 LCM backend fail-fast 退出码 4 不滚雪球重启，符合"启动期 fail-fast 不是 runtime bug"的语义。

---

**Why**：`00_执行方案.md` §0.3 ADR 与 §0.2 决策驱动第 1 条明确"目标态不接受旧路径"。**本期不开项明确为 blocker，不伪装完成**。**How to apply**：任何 reviewer 看到本文件的 NOT RUN / SKIP 即可明确知道哪些项目仍待 Phase 5 实机冒烟 lane 与上游 dimos PR 落地。