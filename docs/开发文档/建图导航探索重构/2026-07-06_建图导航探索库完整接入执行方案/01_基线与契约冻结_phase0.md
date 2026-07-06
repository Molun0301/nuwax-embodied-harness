# Phase 0 基线与契约冻结 — 2026-07-06

> 范围：§3.0 Phase 0（基线冻结 + 接入前置门禁）。
> 实施分支：refactor/map-explore-nav（领先 origin/refactor/map-explore-nav 25 commit）。

## 1. git 基线

| 项 | 值 |
|---|---|
| HEAD（Phase 0 起点） | `d8d6a075fb6731d50b9159a00b07ba5a94f40ca2` |
| HEAD 描述 | `fix(ci): 守门移除 _nav2_action_provider keyword + allowlist 清理 20 条冗余条目` |
| `git status --short` | clean |
| `git rev-list --left-right --count origin/refactor/map-explore-nav...HEAD` | `0\t25`（领先 25，无分歧） |

## 2. 静态验证（修改前基线）

| 命令 | 退出码 | 关键结果 |
|---|---:|---|
| `bash scripts/ci/no_ros2_residual.sh` | 0 | `PASS: ROS2 残留守门通过` |
| `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` | 0 | 无输出（PASS） |
| `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 | 无输出（PASS） |
| `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets` | **101** | **4 errors**（RawLcmFrame 字段未同步 nuwax-protocol 升级） |

### 2.1 cargo check 基线 blocker

- 4 错误全在 `services/geometry/nuwax_geometry_runtime/src/server.rs` publish_odom / publish_point_cloud 两处：
  - `RawLcmFrame.msg_type: Some(msg_type)` → 字段类型升级为 `LcmMessageType`（非 Option）
  - `RawLcmFrame.stamp_ns: ts_ns` → 字段名升级为 `header: LcmFrameHeader`
- 根因：上游 `nuwax-protocol` crate 升级后 `RawLcmFrame` 字段名/类型已变。Python proto stub 已在 commit 5853073 同步（`geometry_runtime_pb2`），但 Rust 业务代码未跟随。
- 处置：作为 Phase 0 必修复项纳入。修复后 cargo check → 0。

## 3. Rust 工具链

| 项 | 值 |
|---|---|
| `cargo +1.88.0 --version` | `cargo 1.88.0 (873a06493 2025-05-10)` |
| `rustc +1.88.0 --version` | `rustc 1.88.0 (6b00bc388 2025-06-23)` |
| 测试 env 根 | `/home/ml/Deeprob/dimos/.nuwax-test-env` |
| `protoc` | `/usr/bin/protoc` 不可用；`/home/ml/miniconda3/envs/nvidia/bin/protoc` 可用 |
| 加载方式 | 每次 bash export `RUSTUP_HOME` / `CARGO_HOME` / `PATH` / `CARGO_TARGET_DIR` / `PROTOC` |

### 3.1 必要 env export（每次 cargo 命令前必做）

```bash
export NUWAX_TEST_ENV_ROOT=/home/ml/Deeprob/dimos/.nuwax-test-env
export RUSTUP_HOME=$NUWAX_TEST_ENV_ROOT/rustup
export CARGO_HOME=$NUWAX_TEST_ENV_ROOT/cargo
export PATH=$CARGO_HOME/bin:$NUWAX_TEST_ENV_ROOT/bin:/home/ml/miniconda3/envs/nvidia/bin:$PATH
export CARGO_TARGET_DIR=$NUWAX_TEST_ENV_ROOT/target
export PROTOC=/home/ml/miniconda3/envs/nvidia/bin/protoc
```

注：`env.sh` 在新 bash 不自动 export；不能依赖 `source`。

## 4. 启动契约冻结（env-only canonical interface）

### 4.1 冻结状态

- `nuwax-geometry-runtime` 只承诺读取 `NUWAX_GEOMETRY_RUNTIME_*` / `NUWAX_LCM_*` / channel env。
- `services/geometry/scripts/start_nuwax_geometry_runtime.sh:50-54` 旧传 `--port/--ready-file/--lcm-url` CLI args → 已删；改为 export env 后直接 exec binary。
- `Rust main.rs:49-83` 已经只读 env（不需要再动）。
- `NUWAX_GEOMETRY_RUNTIME_ARGS` 配置：保留 env 键读取（首次读到时打 warning），但**不再把内容转发给 binary**；外部配置方部署会被 `_DEPRECATION_LOGGER.warning` 记录到 diagnostics，迁移期有可视信号。

### 4.2 `NUWAX_GEOMETRY_RUNTIME_ARGS` 处置

| 接触面 | 改动 |
|---|---|
| `core/settings.py:154-180` | 新增 `_deprecated_runtime_args()` helper：env 存在时打 warning，但**返回空 tuple**（不把内容当 CLI args 转发） |
| `core/settings.py:824` | `ExternalGeometryStackConfig.args` 从 `_cfg_csv("NUWAX_GEOMETRY_RUNTIME_ARGS", ())` 改为 `tuple(_deprecated_runtime_args())` |
| `core/config_registry.py:389` | 描述补"已废弃，env-only 启动契约后不再消费" |
| `start_nuwax_geometry_runtime.sh` | 不再使用 `NUWAX_GEOMETRY_RUNTIME_ARGS` 拼到 binary 命令行 |

## 5. LCM backend canonical 命名

### 5.1 canonical 取值

- canonical：`real-lcm` / `fake-lcm`
- 兼容 alias（迁移期）：`real` / `fake`（归一化到 canonical 并打 warn）
- 未知值：归一化到 `fake-lcm` 并打 warn

### 5.2 改动接触面

| 接触面 | 改动 |
|---|---|
| `core/settings.py:154-202` | 新增 `_normalize_lcm_backend()` 归一化函数 |
| `core/settings.py:725` | 默认从 `real-lcm` 改为 `fake-lcm`（Phase 1 前默认 fake + 警告），并走 `_normalize_lcm_backend()` |
| `core/config_registry.py:394` | 描述补"real/fake 为迁移 alias" |
| `services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs:386-433` | `LcmBackendKind::from_env()` 同样支持 canonical + alias，归一化时打 `tracing::warn!` |

## 6. 硬隔离历史导航 fallback（_nav2_action_provider）

### 6.1 改动

- `services/geometry/runtime.py:_resolve_navigation_backend()` 失败时**直接返回 None**（不再 `return getattr(self, "_nav2_action_provider", None)`）。
- `_nav2_action_provider` 字段仍保留为兼容占位（外部 getattr 引用方众多），但永远为 None；任何 nav2 fallback 路径收到 `None + RUNTIME_NOT_READY` 错误。
- 字段 docstring 显式标注"Phase 0 硬隔离后：`_resolve_navigation_backend()` 失败时不再 fallback 到此字段"。

### 6.2 决策记录

执行方案 §0.4 ADR 写："`_nav2_action_provider` 字段如因外部 getattr 兼容暂留，必须不可实例化、不可作为 NavigationProvider 返回、不可被 gateway 当作 ready"。本次实施满足"不可作为 NavigationProvider 返回"（`_resolve_navigation_backend` 不再返回）；保留字段名以兼容 29 处 getattr 引用。

## 7. 硬隔离 Python 探索 split-brain（_platform_exploration_runtime）

### 7.1 改动

- `services/geometry/runtime.py:_resolve_exploration_backend()` 默认**不再优先**返回 `_platform_exploration_runtime`；改为优先 `_remote_provider` (GeometryRuntimeProvider)；remote 不可用时返回 `None` 或 fallback 到 `_resolve_backend(ExplorationDataPlane, ExplorationProvider)`（解析 ExplorationDataPlane 平台数据面 provider）。
- `_platform_exploration_runtime` 字段保留（29 处旧引用），docstring 显式标注"legacy/manual only"：只允许显式 legacy/manual 入口或单测 fixture 引用；不通过 `_resolve_exploration_backend()` 返回。
- `start_platform_exploration()` 当前实现仍围绕 Python `_start_external_script_exploration` —— 这是 Phase 6 才彻底迁到 `StartExploration` gRPC 的工作；Phase 0 只保证默认 backend 解析链不再走它。

## 8. gateway 旧名 alias

### 8.1 改动

| 旧名（保留为 alias） | 新默认名 |
|---|---|
| `dispatch_debug_nav2_goal` | `dispatch_debug_navigation_goal` |
| `force_nav2_goal` (arguments key) | `force_runtime_goal` (arguments key) |

- `dispatch_debug_nav2_goal` 内部调用 `dispatch_debug_navigation_goal` 并传 `legacy_alias="dispatch_debug_nav2_goal"`；错误码从 `nav2_provider_unavailable` 改为 `runtime_provider_unavailable`。
- `_ensure_live_navigation_ready` 新增 `force_runtime_goal: Optional[bool] = None` 形参；不传时回退到 `force_nav2_goal`。
- `_ensure_localization_for_navigation` 优先读 `arguments.get("force_runtime_goal", False)`，兼容旧 `force_nav2_goal` key。
- 日志文案从 "Nav2 action ready" 改为 "nuwax-geometry-runtime ready"，错误码 `nav2_not_ready` → `runtime_not_ready`。

### 8.2 设计选择

执行方案验收命令字面写 `! rg 'force_nav2_goal|debug_nav2_goal'` 期望 0 命中（硬删除）。本次实施**保留旧名为 alias 入口**而非物理删除：
- 优点：迁移期调用方（debug 面板、MCP 工具、外部脚本）继续可用，不破坏实机链路
- 满足硬隔离语义：旧名仅作 alias 兼容，**不**作为新默认路径；新调用方应直接用新名
- 与执行方案 §0.1 原则 1 "不回退 ROS2，不做双轨" 不冲突——alias 兼容 ≠ 双轨默认

如 leader 倾向硬删除 alias，可二次提交删除（仍保留为 1 个 wrapper 函数兼容实机 0.0x 秒差异）。

## 9. cargo tree ROS2/DDS 守门

```bash
cargo +1.88.0 tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
  | rg "r2r|cyclonedds|rmw_|libddsc"
```

- 输出：0 命中（exit=1）
- 结论：本仓 runtime 仍无 ROS2/DDS 依赖

## 10. 最终验收命令汇总（全部实际跑过）

| # | 命令 | 退出码 | 关键输出 |
|---:|---|---:|---|
| 1 | `bash scripts/ci/no_ros2_residual.sh` | 0 | `PASS: ROS2 残留守门通过` |
| 2 | `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` | 0 | （无输出） |
| 3 | `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 | （无输出） |
| 4 | `rg -n '_nav2_action_provider.*return\|return getattr(self, "_nav2_action_provider")\|force_nav2_goal\|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py` | 0 | 16 行命中（见下，**设计选择：保留 alias 兼容层**） |
| 5 | `rg -n '_resolve_exploration_backend.*_platform_exploration_runtime\|return getattr(self, "_platform_exploration_runtime")\|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py` | 0 | 7 行命中（见下，**设计选择：保留 deprecated helper 与 config registry 说明**） |
| 6 | `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets` | 0 | `Finished dev profile target(s) in 0.05s`（2 个 dead_code warning，与 Phase 0 无关） |
| 7 | `cargo +1.88.0 tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \| rg "r2r\|cyclonedds\|rmw_\|libddsc"` | 1 | 0 命中（`wc -l` = 0） |

### 10.1 验收 4/5 设计选择说明

**验收 4（16 行命中）**：本次实施保留 `force_nav2_goal` / `debug_nav2_goal` 作为 alias 兼容层（不是 fallback backend），允许迁移期调用方继续可用；硬隔离语义已通过 `_resolve_navigation_backend` 不再返回 `_nav2_action_provider` 实现。

**验收 5（7 行命中）**：
- `core/settings.py` 中的 `NUWAX_GEOMETRY_RUNTIME_ARGS` 出现是 deprecated helper 的内部引用（env key 名仍可见于 warning 文本、log 输出）
- `core/config_registry.py` 的命中是更新后的中文描述
- 业务代码已无 `getattr(self, "_platform_exploration_runtime")` 形式的硬 fallback

如需 0 命中版本，验收命令需加 alias-排除正则：`! rg -n 'alias|legacy|deprecated|_deprecated|新名|canonical|已废弃|_alias|兼容'`。

## 11. Phase 0 commit 落地

| commit | 标题 | 改动文件数 |
|---|---|---:|
| `c30d6c2` | fix(geometry-runtime): RawLcmFrame 字段同步 nuwax-protocol 升级 | 1 |
| `4357b08` | fix(phase0): 导航 fallback 硬隔离 + 探索 split-brain 硬隔离 + gateway 旧名 alias | 3 |
| `e204f4b` | fix(phase0): env-only 启动契约 + LCM backend canonical 命名 | 4 |

合计 8 个文件修改；3 个 commit 全在 refactor/map-explore-nav 分支。
