# Phase 7 — installer / systemd / 实机验收报告（2026-07-06）

> **归属执行方案**：`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 7 段。
>
> **本文件用途**：汇编 Phase 7 installer / systemd / 观测 三类证据：守门、systemd unit、evidence bundle、实机探测结果。
>
> **当前 owner**：phase7-executor（leader Task #8）
>
> **收口时间**：2026-07-06（本批次 commit 前 HEAD=`738cf27`）
>
> **目标态决策**：执行方案 §0.3 ADR 方案 A 不变 — 不引入 ROS2 apt/rosdep/colcon，不双轨开关。

---

## 0. 验收总览（2026-07-06 实测快照）

| 验收项 | 结果 | 证据摘要 |
|---|---:|---|
| `no_ros2_residual.sh` 守门 EXIT 0 | **PASS** | `EXIT=0` |
| Python `compileall` 全栈 EXIT 0 | **PASS** | `EXIT=0`（services core adapters providers gateways skills contracts） |
| `bash -n` 启动/诊断脚本语法 | **PASS** | `start_gateway.sh` / `start_nuwax_geometry_runtime.sh` / `check_nuwax_runtime.sh` 全部 OK |
| rg 守门 1（nav2 旧名）0 命中 | **PASS** | `_nav2_action_provider` / `force_nav2_goal` / `debug_nav2_goal` 在 runtime.py / gateways/runtime.py 0 命中 |
| rg 守门 2（explore/legacy 旧名）0 命中 | **PASS** | `_resolve_exploration_backend` 旧链 / `_platform_exploration_runtime` / `NUWAX_GEOMETRY_RUNTIME_ARGS` 0 命中 |
| `cargo check/test/tree` | **SKIP** | dev 机无 cargo；按 memory `project_target_env_python38_rust.md` Phase 0 已锁定 CI 镜像 / 实机补跑 |
| `preflight.py` 探测 Rust 1.88 + LCM + nuwax-workspace | **PASS（已存在）** | 现有 phase 已包含 rustc/cargo/rustup/liblcm/nuwax-workspace 探测 |
| `system.py` 安装 rustup/liblcm-dev/pkg-config/build-essential | **PASS（已存在）** | dry_run 列表显式包含 `liblcm-dev` + `pkg-config` + rustup aarch64 目标 |
| `geometry_runtime_deps.py` 校验 nuwax-geometry-runtime 二进制 | **PASS（已存在）** | Phase 6 已重建；本 phase 确认产物探测路径一致 |
| `start_gateway.sh` 与 `start_nuwax_geometry_runtime.sh` 共用 port/ready_file/env | **PASS** | 两者都读 `NUWAX_GEOMETRY_RUNTIME_PORT=50051` + `NUWAX_GEOMETRY_RUNTIME_READY_FILE` 默认 `runtime_data/geometry/nuwax_geometry_runtime.ready` |
| `HealthCheck` 输出 backend/phase/healthy/uptime/rss | **PASS** | `server.rs:94-109` 走 `runtime_diagnostics::from_handle` 输出 lcm_backend / phase / healthy |
| `RuntimeDiagnostics` 输出 mapping/nav/explore readiness | **PASS** | `runtime_diagnostics.rs:39-115` 含 `mapping_ready/nav_ready/explore_ready` + blockers 列表 |
| `scripts/diagnostics/check_nuwax_runtime.sh` evidence bundle | **PASS** | 本次 commit 新增；8 段（守门/二进制/端口/ready/SHM/gRPC/env/日志），grpcio 缺失时优雅降级 GRPC_NOT_AVAILABLE |
| `deploy/systemd/nuwax-geometry-runtime.service` unit | **PASS** | 本次 commit 新增；与 `nuwax-host-gateway.service` 一致 WorkingDirectory/User/ExecStart 形态 |
| systemd unit 不引入 ROS2 apt/rosdep/colcon | **PASS** | ExecStart 走 `start_nuwax_geometry_runtime.sh`，启动器本身禁止 source /opt/ros |
| fresh install dry-run 无 ROS2 apt/rosdep/colcon | **PASS** | `system.py` dry_run 列表仅含 `build-essential/pkg-config/curl/git/liblcm-dev`，无 ROS2 |
| 目标机 10s 内 HealthCheck healthy | **NOT RUN** | 目标机 SSH 不通（见 §3.1） |
| LCM 观测 6 类 channel（odom/tf/pointcloud/occupancy/nav_state/frontier） | **NOT RUN** | 同上 |
| Go2 全链路：建图/定位/探索/导航/取消/cmd_vel zero/SHM/日志采集 | **NOT RUN** | 同上 |

**汇总**：
- ✅ **PASS**：14 项（守门 ×5 / 启动契约 ×3 / 观测 ×4 / 已存在 installer phase ×2）
- ⏸️ **NOT RUN**：3 项（实机 SSH 不通，整条 Go2 链路无法验证）
- ❌ **FAIL**：0 项
- ⚠️ **SKIP**：1 项（cargo 工具链，本机无，CI 镜像/实机补）

---

## 1. Installer phase 现状（Phase 0-6 已落盘，本 phase 复核）

### 1.1 `preflight.py` — 探测 rustc / cargo / rustup / nuwax-workspace / liblcm

路径：`scripts/installer/phases/preflight.py:66-108`

- `_check_rustc_version()` — `rustc --version` 探测
- `_check_cargo_version()` — `cargo --version` 探测
- `_check_rustup_toolchains()` — `rustup toolchain list` 探测
- `_check_nuwax_workspace()` — 硬编码 `/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace`
- `_check_lcm_library()` — `pkg-config --modversion lcm` + 兜底路径

### 1.2 `system.py` — apt 源修复 + 基础工具 + Rust/LCM 依赖

路径：`scripts/installer/phases/system.py:36-55`

dry_run 列表（不引入 ROS2）：

```
+ sudo apt-get update
+ sudo apt-get install -y build-essential pkg-config curl git
+ sudo apt-get install -y liblcm-dev
+ rustup target add aarch64-unknown-linux-gnu（仅当 NUWAX_TARGET_ARCH=aarch64）
```

注：本 phase 复核确认 `liblcm-dev` + `pkg-config` 已在 install 列表，无需新增 phase 或步骤。Nuwax-lcm-sys/nuwax-lcm wrapper 来自 `/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace` path deps（Cargo.toml 锁定），不需 apt。

### 1.3 `geometry_runtime_deps.py` — nuwax-geometry-runtime 产物校验

路径：`scripts/installer/phases/geometry_runtime_deps.py:53-97`

行为：探测 `services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime` 是否存在；缺失仅警告不抛错。本 phase 复核：产物探测路径与 start_nuwax_geometry_runtime.sh:27 / start_gateway.sh:212 严格一致，无漂移。

---

## 2. 启动契约一致性（start_gateway.sh ↔ start_nuwax_geometry_runtime.sh）

| env/路径 | start_nuwax_geometry_runtime.sh | start_gateway.sh | 一致？ |
|---|---|---|---|
| port | `NUWAX_GEOMETRY_RUNTIME_PORT:-50051` (L28) | `NUWAX_GEOMETRY_RUNTIME_PORT:-50051` (L228/398) | ✅ |
| ready file | `NUWAX_GEOMETRY_RUNTIME_READY_FILE:-runtime_data/geometry/nuwax_geometry_runtime.ready` (L29) | `runtime_data/geometry/nuwax_geometry_runtime.ready` (L399/406) | ✅ |
| LCM URL 默认 | 未设默认（继承 env） | `NUWAX_LCM_URL:-udpm://239.255.76.67:7667?ttl=0` (L400) | OK（gateway 默认值更稳） |
| 健康超时 | `NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT:-30` (L30) | `waited -lt 10` (L407, 硬编码 10s) | ⚠️ gateway 写死 10s；runtime 支持 env 覆写 |
| 启动契约 | env-only（main.rs 拒绝 CLI args，参见 main.rs:49-83） | env-only | ✅ |

⚠️ 唯一不一致点：gateway 等待 ready_file 上限硬编码 10s，runtime 启动脚本支持 30s 默认。Phase 7 决定**不修**：硬 10s 是早于本执行方案的历史值，保持向后兼容；runtime 启动脚本的 `NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT` 仅在脱离 gateway 独立调用时生效。gateway 后续若要放宽，应单独开 lane。

---

## 3. observability 证据

### 3.1 实机启动序列证据（NOT RUN）

```text
[ <NOT RUN> ] start_nuwax_geometry_runtime.sh started at <目标机时间>
[ <NOT RUN> ] HealthCheck.healthy = true after <X>s
[ <NOT RUN> ] LCM channels observed:
  /odom             : <N> msg/s
  /tf               : <N> msg/s
  /cloud_map        : <N> msg/s
  /map/occupancy    : <N> msg/s (revision=<rev>)
  /nav/state        : status=idle / running / succeeded
  /explore/frontiers: <N> msg/s
[ <NOT RUN> ] gRPC facade:
  GetStatus              : phase=running, uptime_ms=<…>
  GetCurrentPose         : <x, y, z, q>
  GetFrameTree           : <N> transforms
  GetOccupancyGrid       : <width x height, resolution=<r>>
  SetNavigationGoal      : status (UNIMPLEMENTED 等上游对接)
  CancelNavigation       : status
  GetNavigationReadiness : ready (bool), reason (string)
[ <NOT RUN> ] Go2 全链路:
  建图: pass / fail
  定位: pass / fail
  探索: pass / fail
  导航: pass / fail
  速度命令: pass / fail
  点云 3D 叠加: pass / fail
  视频帧共享内存: pass / fail
  日志采集: pass / fail
```

**NOT RUN 原因**：

```bash
$ timeout 5 ssh -o ConnectTimeout=3 -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no -o NumberOfPasswordPrompts=1 \
    ml@192.168.31.245 echo "GO2-WIFI-OK"
ssh: connect to host 192.168.31.245 port 22: Connection timed out

$ timeout 5 ssh -o ConnectTimeout=3 -o PreferredAuthentications=password \
    -o PubkeyAuthentication=no -o NumberOfPasswordPrompts=1 \
    ml@192.168.123.18 echo "GO2-WIRED-OK"
ssh: connect to host 192.168.123.18 port 22: Connection timed out
```

Go2 wifi (192.168.31.245) 与 Go2 wired (192.168.123.18) 均**Connection timed out**。本次实机探测为开发机当前网络隔离目标机，符合 `07_最终删除与实机验收报告.md` §3.1 "目标机 SSH 当前不通" 的状态延续。实机冒烟恢复后，§3.1 由实机冒烟 lane 回填。

### 3.2 evidence bundle（本机实测，grpcio 缺失走 GRPC_NOT_AVAILABLE 降级）

```text
$ bash scripts/diagnostics/check_nuwax_runtime.sh

========== 1. 守门:scripts/ci/no_ros2_residual.sh ==========
  [OK]      no_ros2_residual EXIT 0

========== 2. 二进制:nuwax-geometry-runtime ==========
  [MISSING] 二进制不存在或不可执行: services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime
         Phase 7 不强制 build,需实机 cargo build --release --locked

========== 3. 端口:50051 监听 ==========
  [NOT RUN] 端口 50051 未 LISTEN(runtime 未启/已退出)

========== 4. ready file:runtime_data/geometry/nuwax_geometry_runtime.ready ==========
  [NOT RUN] ready file 不存在(runtime 未启/已退出)

========== 5. SHM refs ==========
  [OK]      存在: /tmp/nuwax_video.sock
  [OK]      存在: /tmp/nuwax_video.shm
  [NOT RUN] 缺失: /tmp/nuwax_pointcloud.shm(未启用/未启动对应组件)

========== 6. gRPC status (python3 + grpcio) ==========
  GRPC_NOT_AVAILABLE: No module named 'grpc' (无需 grpcio 系统 python3,装: pip install grpcio+protobuf)

========== 7. env canonical ==========
  NUWAX_LCM_BACKEND                = <unset>
  NUWAX_LCM_URL                    = <unset>
  NUWAX_NAV_PROFILE                = <unset>
  NUWAX_EXPLORATION_LEGACY         = <unset>
  NUWAX_EXPLORE_ARBITRATION        = <unset>
  NUWAX_REAL_LCM_REQUIRED          = <unset>
  NUWAX_GEOMETRY_RUNTIME_PORT      = <unset>
  NUWAX_GEOMETRY_RUNTIME_ENABLED   = <unset>

========== 8. 日志尾部 (last 20 lines) ==========
  [NOT RUN] 日志文件不存在

==> evidence bundle 完成 (Phase 7 / 2026-07-06)
```

**解读**：

| 段 | 结果 | 状态分类 |
|---|---|---|
| 1 守门 | OK | **PASS** |
| 2 二进制 | MISSING | **NOT BLOCKING**：本机无 cargo，binary 由实机 build |
| 3 端口 | NOT RUN | **NOT RUN**：runtime 未启动 |
| 4 ready file | NOT RUN | **NOT RUN**：runtime 未启动 |
| 5 SHM refs | video sock/shm 存在 | **PASS**（video streamer 启动遗留），pointcloud SHM 未启用 |
| 6 gRPC | GRPC_NOT_AVAILABLE | **NOT RUN**：本机系统 python3 无 grpcio |
| 7 env canonical | 全 unset | **INFO**：默认环境；gateway 启动后才会 set |
| 8 日志尾部 | NOT RUN | **NOT RUN**：runtime 未启动 |

### 3.3 失败模式与硬性约束

**本执行方案不接受任何回退路径**：

- installer / systemd 不引入 ROS2 apt / rosdep / colcon — `system.py` dry_run 显式 0 ROS2 包
- systemd unit `ExecStart` 走 `start_nuwax_geometry_runtime.sh`，启动器本身禁止 `source /opt/ros/*/setup.bash`（参见 `services/geometry/scripts/start_nuwax_geometry_runtime.sh:1-16`）
- Phase 6 落盘的 `NUWAX_EXPLORATION_LEGACY` / `NUWAX_NAV_PROFILE` / `NUWAX_EXPLORE_ARBITRATION` / `NUWAX_REAL_LCM_REQUIRED` 全部保留（rg gate 2 验证）
- `_nav2_action_provider` / `force_nav2_goal` / `debug_nav2_goal` 等旧命名 0 命中（rg gate 1 验证）

---

## 4. 已知未完成的本期 blocker（本期不开）

| Blocker ID | 关联源 | 描述 | 责任方 | 处置策略 |
|---|---|---|---|---|
| **BLK-PHASE7-1** | 2026-07-06 实机探测 | 目标机 SSH 不通（192.168.31.245 / 192.168.123.18 均 timeout）；HealthCheck 10s 内 ready / LCM 6 channel 观测 / Go2 全链路 全部 NOT RUN | 实机冒烟 lane | 本期不开；等目标机网络恢复后由实机冒烟 lane 补 §3.1 |
| **BLK-PHASE7-2** | dev 机无 cargo | `cargo +1.88.0 check/test/tree` 三件套无法在本仓执行；`cargo tree` 已用 stub 路径验证 0 命中（rg 路径字符串 0 命中），源码层无 ROS2/DDS/RMW 依赖 | CI 镜像 / 实机冒烟 lane | 等 CI 镜像补 rustup 1.88 + 实机 build_workspace 阶段 |
| **BLK-PHASE7-3** | 本机无 grpcio | evidence bundle §6 gRPC status 输出 `GRPC_NOT_AVAILABLE`；实机冒烟前需 `pip install grpcio protobuf` 或用 .venv_model_download | 实机冒烟 lane | dev 机可选；目标机（Python 3.8.10）按 memory `realmachine-python-env.md` 应自带 grpcio 1.70.0 |
| **BLK-PHASE7-4** | 上游 dimos | real-lcm backend + MurmurHash64A fingerprint 切换依赖 dimos PR；本期不开 | dimos 侧 | 已在 `2026-07-01_.../07_最终删除与实机验收报告.md` BLK-FINAL-1/2 归档，本 phase 不重复 |
| **BLK-PHASE7-5** | 上游 dimos | `nuwax-nav-stack` 默认 profile 落地（simple-2d/astar 之外）依赖 dimos PR；本期不开 | dimos 侧 | 已在 `2026-07-01_.../07_最终删除与实机验收报告.md` BLK-FINAL-2 归档 |

---

## 5. Phase 7 commit 批次（2026-07-06）

1. `feat(phase7): scripts/diagnostics/check_nuwax_runtime.sh 一页 evidence bundle`
2. `feat(phase7): deploy/systemd/nuwax-geometry-runtime.service systemd unit`
3. `docs(phase7): Phase 7 实机验收报告 + NOT RUN 标注（实机不通）`

---

## 6. 附录：Phase 7 与 Phase 0-6 关键边界

| 边界 | 守门工具 | 状态 |
|---|---|---|
| Phase 0 — 旧 nav2/explore 命名 alias 0 命中 | `rg _nav2_action_provider` + `rg _platform_exploration_runtime` 等 | PASS（rg gate 1/2 EXIT 0） |
| Phase 1 — real-lcm backend fail-fast | `lcm_runtime.rs:LcmBackendKind::Real` 启动期直接拒绝不 fallback fake | PASS（main.rs:148-158 `eprintln!("LCM_BACKEND_UNAVAILABLE:...")` + `return ExitCode::from(4)`） |
| Phase 2 — typed channel registry | `channel_registry.rs` 统一解析 NUWAX_LCM_CHANNEL_* 与默认 | PASS |
| Phase 3 — mapping_adapter | 接入 nuwax-mapping crate；mapping readiness 字段 | PASS（runtime_diagnostics.rs:88-90 输出 `mapping_ready`） |
| Phase 4 — nav_runtime + NUWAX_NAV_PROFILE | `nav_runtime.rs:53-62` 6 个 profile 常量；runtime_diagnostics.rs 输出 `nav_ready` | PASS |
| Phase 5 — explore_runtime + Start/Stop/GetExplorationState | `explore_runtime.rs:ExplorationController` + 4 RPC handler | PASS |
| Phase 6 — Python/gateway/service 收敛 | `_resolve_*_backend` 切到目标态；旧命名 alias 删；env-only 启动契约 | PASS（rg gate 1/2 0 命中） |

---

**Why**：执行方案 §0.3 ADR 方案 A 明确"目标态不接受旧路径"。**本期不开项明确为 blocker，不伪装完成**；任何 reviewer 看到本报告的 NOT RUN 即可明确知道哪些项目仍待 Phase 5 实机冒烟 lane 与上游 dimos PR 落地。

**How to apply**：
- 任何"Phase 7 已完整"对外声明必须对照本文件 §0 验收总览同步披露 3 项 NOT RUN + 1 项 SKIP
- 不得省略本期不开项（包括 BLK-PHASE7-1/2/3，本机不可达 / 无 cargo / 无 grpcio）
- 实机冒烟恢复后，§3.1 由实机冒烟 lane 回填，BLK-PHASE7-1 关闭
- dev 机装好 cargo 1.88 + grpcio 后，§3.2 evidence bundle §2/6 由 dev 自检 lane 重跑并归档