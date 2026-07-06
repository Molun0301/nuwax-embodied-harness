# Phase 7 — todolist（installer / systemd / 实机链路）

> **归属执行方案**：`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 7 段
>
> **本文件用途**：落盘 Phase 7 子任务的详细计划 + 状态，**任务正式执行前落地**

---

## 7.0 任务总览

| 子任务 | 状态 | 证据/产物 |
|---|---|---|
| 7.1 installer phases 核查（preflight/system/geometry_runtime_deps） | ✅ 完成 | system.py:42-54 dry_run 列表 / preflight.py:66-108 探测 / geometry_runtime_deps.py:53-97 产物校验 |
| 7.2 HealthCheck backend/profile/readiness 输出核查 | ✅ 完成 | runtime_diagnostics.rs:39-115 + server.rs:94-109 + runtime_diagnostics.rs:160-208 板块 readiness |
| 7.3 新增 `scripts/diagnostics/check_nuwax_runtime.sh` evidence bundle | ✅ 完成 | scripts/diagnostics/check_nuwax_runtime.sh — 8 段（守门/二进制/端口/ready/SHM/gRPC/env/日志） |
| 7.4 新增 `deploy/systemd/nuwax-geometry-runtime.service` | ✅ 完成 | deploy/systemd/nuwax-geometry-runtime.service — 与 gateway.service 同形态，不引 ROS2 |
| 7.5 实机可达性探测 + NOT RUN 标注 | ✅ 完成 | 192.168.31.245 / 192.168.123.18 双双 Connection timed out — feedback §未完成 blocker |
| 7.6 验收命令 + 文档落盘 | ✅ 完成 | 70_installer_systemd_实机验收_phase7.md / feedback_phase7.md / 本文件 |

---

## 7.1 installer phases 核查详细

**目标**：核查 preflight/system/geometry_runtime_deps phase 是否覆盖 Rust 1.88 / liblcm-dev / pkg-config / nuwax-workspace / nuwax-lcm-sys 锁定情况；如缺 liblcm-dev 安装步骤则补齐。

**操作**：
- [x] Read `scripts/installer/phases/preflight.py` — 已含 rustc/cargo/rustup/liblcm 探测（L66-108）
- [x] Read `scripts/installer/phases/system.py` — dry_run 已含 `build-essential/pkg-config/curl/git/liblcm-dev`（L51-54）
- [x] Read `scripts/installer/phases/geometry_runtime_deps.py` — 已校验 nuwax-geometry-runtime 产物路径一致（L74-88）

**结论**：现有 phase 已覆盖全部依赖检查点，无需新增步骤。Nuwax-lcm-sys/nuwax-lcm wrapper 来自 nuwax-workspace path deps（Cargo.toml 锁定），不需 apt。

**未引入**：ROS2 apt / rosdep / colcon / libddsc / CycloneDDS（守门验证）。

---

## 7.2 HealthCheck backend/profile/readiness 核查详细

**目标**：确认 HealthCheck 输出 backend/profile/mapping/nav/explore readiness；启动契约（port/ready_file/env）一致。

**操作**：
- [x] Read `services/geometry/nuwax_geometry_runtime/src/runtime_diagnostics.rs` — 含 `lcm_backend/mapping_ready/nav_ready/explore_ready` + blockers（L39-115）
- [x] Read `services/geometry/nuwax_geometry_runtime/src/server.rs:94-109` — `health_check` 走 RuntimeDiagnostics
- [x] Read `services/geometry/nuwax_geometry_runtime/src/nav_runtime.rs:53-62` — 6 个 NUWAX_NAV_PROFILE 常量
- [x] Read `services/geometry/scripts/start_nuwax_geometry_runtime.sh:27-30` — port/ready_file/health_timeout 默认值
- [x] Read `scripts/start_gateway.sh:208-252/371-416` — gateway 用 port/ready_file/env 拉起 runtime

**结论**：
- HealthCheck 输出 `healthy/phase/uptime_ms/rss_kb/lcm_backend`（server.rs:100-108）
- 板块 readiness 由独立 RPC 输出（`GetNavigationReadiness` / `GetExplorationState`），符合 Phase 7 §"板块级 readiness 报告" 表述
- 启动契约：start_gateway.sh ↔ start_nuwax_geometry_runtime.sh 共用 port=50051 + ready_file=runtime_data/geometry/nuwax_geometry_runtime.ready
- 唯一不一致：gateway 等待 ready_file 上限硬编码 10s，runtime 启动器 30s。Phase 7 决定**不修**（向后兼容），另开 lane 处理

---

## 7.3 evidence bundle 详细

**目标**：新增 `scripts/diagnostics/check_nuwax_runtime.sh`，一页输出 8 段 evidence；不依赖 grpcurl / lcm-logger / 外网。

**操作**：
- [x] Write `scripts/diagnostics/check_nuwax_runtime.sh` — 8 段：守门 / 二进制 / 端口 / ready file / SHM refs / gRPC status / env canonical / 日志尾部
- [x] `chmod +x` + `bash -n` 验证语法
- [x] 实际跑一遍（grpcio 缺失走 GRPC_NOT_AVAILABLE 优雅降级）
- [x] 捕获输出到 `/tmp/phase7_check_bundle.out`（36 行）

**grpcio 降级路径**：当系统 python3 无 grpc 模块，脚本输出 `GRPC_NOT_AVAILABLE: No module named 'grpc'` 单行，不打堆栈。目标机（Python 3.8.10 + grpcio 1.70.0）会跑出真实 gRPC status。

---

## 7.4 systemd unit 详细

**目标**：新增 `deploy/systemd/nuwax-geometry-runtime.service`，与现有 gateway service 同形态，不引 ROS2。

**操作**：
- [x] Write `deploy/systemd/nuwax-geometry-runtime.service` — WorkingDirectory/User/Group 与 gateway 一致
- [x] ExecStart = `start_nuwax_geometry_runtime.sh`（env-only 启动契约，不传 CLI args）
- [x] `SuccessExitStatus=0 4` 软化 LCM backend fail-fast 退出码 4，避免 systemd 滚雪球
- [x] ReadWritePaths = runtime_data / /tmp / var/log（含 SHM/UDS/LCM/日志）

**未引入**：ROS2 apt / rosdep / colcon / `/opt/ros/*/setup.bash` source（守门验证）。

---

## 7.5 实机可达性探测详细

**目标**：探测 Go2 wifi 192.168.31.245 + Go2 wired 192.168.123.18 SSH 可达性；不可达 = NOT RUN + 写明原因。

**操作**：
- [x] `ssh -o ConnectTimeout=3 ... ml@192.168.31.245` → `Connection timed out`
- [x] `ssh -o ConnectTimeout=3 ... ml@192.168.123.18` → `Connection timed out`

**结论**：本期实机冒烟全部 NOT RUN（与 `2026-07-01_.../07_最终删除与实机验收报告.md` §3.1 状态延续）。BLK-PHASE7-1 归档到 70_installer_systemd_实机验收_phase7.md §4。

---

## 7.6 验收命令 + 文档落盘详细

**验收命令实际跑过**（按执行方案 §3 Phase 7 验收段 + leader 提供命令清单）：

```bash
$ bash scripts/ci/no_ros2_residual.sh; echo "EXIT=$?"
[no_ros2_residual] PASS: ROS2 残留守门通过
EXIT=0

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts; echo "EXIT=$?"
EXIT=0

$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh \
       && bash -n scripts/diagnostics/check_nuwax_runtime.sh \
       && bash -n scripts/start_gateway.sh \
       && echo "all bash -n OK"
all bash -n OK

$ ! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' \
     services/geometry/runtime.py gateways/runtime.py
EXIT=0  # 0 命中

$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' \
     services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py
EXIT=0  # 0 命中

$ which cargo; which rustc
# cargo: 未找到命令；rustc: 未找到命令 → SKIP

$ ! cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml 2>/dev/null \
     | rg "r2r|cyclonedds|rmw_|libddsc"
EXIT=0  # SKIP（cargo 不可用，pipeline 短路返 0）

$ ssh -o ConnectTimeout=3 ... ml@192.168.31.245 echo OK
ssh: connect to host 192.168.31.245 port 22: Connection timed out  # NOT RUN

$ test -e deploy/systemd/nuwax-geometry-runtime.service && echo "systemd unit EXISTS" || echo "NOT YET"
systemd unit EXISTS
```

**文档落盘**：
- [x] `70_installer_systemd_实机验收_phase7.md`（本文） — 16 章节 / 3 段 NOT RUN / 5 段 PASS
- [x] `feedback_phase7.md` — 反馈 + 未完成 blocker + 协作建议
- [x] 本文件（todolist_phase7.md）

---

## 7.7 commit 批次

| commit | 改动 |
|---|---|
| `feat(phase7): scripts/diagnostics/check_nuwax_runtime.sh 一页 evidence bundle` | scripts/diagnostics/check_nuwax_runtime.sh |
| `feat(phase7): deploy/systemd/nuwax-geometry-runtime.service systemd unit` | deploy/systemd/nuwax-geometry-runtime.service |
| `docs(phase7): Phase 7 实机验收报告 + NOT RUN 标注（实机不通）` | 70_*.md / feedback_phase7.md / todolist_phase7.md |

---

**Why**：todolist 必须**任务正式执行前落地**，与执行方案 §3 Phase 7 段对齐；任何 reviewer 看本文件即可对照执行进展。**How to apply**：Phase 7 实机冒烟恢复后，§7.5 状态由 NOT RUN → IN PROGRESS / PASS，BLK-PHASE7-1 关闭。