# Phase 0 接触面盘点 — 2026-07-06

> 范围：仅 Phase 0（基线冻结 + 接入前置门禁）。后续 Phase 1+ 的接触面不在此文档。
> 命令基线：HEAD=d8d6a075fb6731d50b9159a00b07ba5a94f40ca2，分支 refactor/map-explore-nav（领先 origin 25 commit）。

## 1. grep 命中汇总（执行 §3.0 接触面清单）

执行命令：

```bash
rg -n --no-heading '_nav2_action_provider|force_nav2_goal|debug_nav2_goal|_platform_exploration_runtime|NUWAX_GEOMETRY_RUNTIME_ARGS' \
  services/geometry/runtime.py services/geometry/geometry_runtime_provider.py \
  gateways/runtime.py core/settings.py core/config_registry.py contracts
```

### 1.1 按符号命中计数

| 符号 | 命中数 | 涉及文件 |
|---|---:|---|
| `_platform_exploration_runtime` | 29 | services/geometry/runtime.py |
| `force_nav2_goal` | 15 | gateways/runtime.py |
| `_nav2_action_provider` | 11 | services/geometry/runtime.py |
| `debug_nav2_goal` | 3 | gateways/runtime.py |
| `NUWAX_GEOMETRY_RUNTIME_ARGS` | 1 | core/settings.py |
| `NUWAX_GEOMETRY_RUNTIME_ARGS` | 1 | core/config_registry.py |

完整命中行号见 `/tmp/phase0_grep_residual.txt`。

### 1.2 按文件命中分布

| 文件 | 命中符号数 |
|---|---:|
| services/geometry/runtime.py | `_nav2_action_provider` × 11 + `_platform_exploration_runtime` × 29 |
| gateways/runtime.py | `force_nav2_goal` × 15 + `debug_nav2_goal` × 3 |
| core/settings.py | `NUWAX_GEOMETRY_RUNTIME_ARGS` × 1 |
| core/config_registry.py | `NUWAX_GEOMETRY_RUNTIME_ARGS` × 1 |

### 1.3 关键位置锚点

| 位置 | 作用 |
|---|---|
| services/geometry/runtime.py:497-502 | `_nav2_action_provider` 兼容占位字段声明（保留为 None） |
| services/geometry/runtime.py:1671-1694 | `_navigation_backend_policy()` 仍把 `_nav2_action_provider` 当作 active_backend_kind 兜底 |
| services/geometry/runtime.py:1706-1710 | `_build_nav2_status()` 注释仍保留 Nav2 语义 |
| services/geometry/runtime.py:3206-3218 | `_resolve_navigation_backend()` 失败时 `return getattr(self, "_nav2_action_provider", None)` ← **Phase 0 必须硬删除** |
| services/geometry/runtime.py:3220-3226 | `_resolve_exploration_backend()` 优先返回 `_platform_exploration_runtime` ← **Phase 0 必须硬隔离** |
| services/geometry/runtime.py:3210-3218 注释 | 声称"保持外部引用方不抛 AttributeError"——Phase 0 决定改为 None + RUNTIME_NOT_READY |
| gateways/runtime.py:4880 (`dispatch_debug_nav2_goal`) | gateway 入口函数名含 `nav2_goal` |
| gateways/runtime.py:5163 / 5164 | goal_metadata 写入 `force_nav2_goal` 字段 |
| gateways/runtime.py:6465 / 6479 | `force_nav2_goal` 参数传到 backend |
| gateways/runtime.py:6532-6577 | 默认入口 `trust_live_navigation or force_nav2_goal` 路径 |
| core/settings.py:824 | `NUWAX_GEOMETRY_RUNTIME_ARGS` 注册到 `ExternalGeometryStackConfig.args` |
| core/config_registry.py:389 | `NUWAX_GEOMETRY_RUNTIME_ARGS` 中文说明 |

## 2. LCM backend canonical 命名接触面

| 位置 | 当前值 | Phase 0 改动 |
|---|---|---|
| `.env.example:258` | `NUWAX_LCM_BACKEND=real-lcm` | 已是 canonical，保留 |
| `core/settings.py:725` | 默认 `real-lcm` | 改为 `fake-lcm`（默认显式 fake 直到 Phase 1 完成），同时增加 `real`/`fake` 兼容 alias |
| `core/config_registry.py:394` | 描述 "real-lcm/fake-lcm" | 描述补"real/fake 为兼容 alias" |
| `services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs:386-398` | `from_env` 接受 `real`/`real-lcm`/`fake` | 改 canonical= `real-lcm`/`fake-lcm`，`real`/`fake` → canonical 并 warn |
| `services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs:422-437` | backend_name 输出 `"fake-lcm"` / `"real-lcm-pending"` | 保持，phase 0 仅规范化 env 解析 |

## 3. 启动契约接触面

| 位置 | 当前状态 | Phase 0 改动 |
|---|---|---|
| `services/geometry/scripts/start_nuwax_geometry_runtime.sh:50-54` | 传 `--port/--ready-file/--lcm-url` CLI args | 改为 export env 后直接 exec（env-only） |
| `services/geometry/nuwax_geometry_runtime/src/main.rs:49-83` | 只读 env（已 env-only） | 不动 |
| `core/settings.py:824` | `NUWAX_GEOMETRY_RUNTIME_ARGS` 注册 | 改为 `_cfg_csv("NUWAX_GEOMETRY_RUNTIME_ARGS", (), deprecated=True)` 并 warning 消费 |
| `core/config_registry.py:389` | 注册 | 描述加 "deprecated，env-only 启动契约后不再消费" |

## 4. 编译基线（关键发现）

### 4.1 no-ROS2 gate

- 命令：`bash scripts/ci/no_ros2_residual.sh`
- 结果：**exit=0 PASS**

### 4.2 Python compileall

- 命令：`PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts`
- 结果：**exit=0 PASS**

### 4.3 cargo check（**基线 FAIL**）

- 命令：`cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets`
- 结果：**exit=101 FAIL** — 4 个编译错误
- 错误归因：
  - `services/geometry/nuwax_geometry_runtime/src/server.rs:454` — `msg_type: Some(msg_type)` 应改为 `msg_type`（RawLcmFrame.msg_type 是非 Option）
  - `services/geometry/nuwax_geometry_runtime/src/server.rs:456` — `stamp_ns: ts_ns` 字段名过期，nuwax-protocol 升级后字段为 `header: LcmFrameHeader`
  - `services/geometry/nuwax_geometry_runtime/src/server.rs:507` — 同 454
  - `services/geometry/nuwax_geometry_runtime/src/server.rs:509` — 同 456
- 根因：上游 `nuwax-protocol` crate 升级后 `RawLcmFrame` 字段名/类型已变（`stamp_ns` → `header: LcmFrameHeader`，`msg_type: Option<LcmMessageType>` → `msg_type: LcmMessageType`）。本仓 server.rs 还在用旧字段构造。
- 这是 **基线 blocker**——执行方案 §7.1 验收第 3 条要求 "本仓 runtime `cargo check/test` 通过"，必须在 Phase 0 修复。
- 历史 commit `5853073` 只同步了 Python `geometry_runtime_pb2` 与 Rust proto，未同步 server.rs 业务代码。

### 4.4 cargo tree ROS2/DDS 依赖

未跑（check 已失败）。check 修复后再跑。

### 4.5 protoc 工具链

- 系统 `/usr/bin/protoc` 不可用。
- `/home/ml/miniconda3/envs/nvidia/bin/protoc` 可用，已通过 `PROTOC=` 环境变量验证。
- 后续验收必须 export `PROTOC=/home/ml/miniconda3/envs/nvidia/bin/protoc` 或在测试 env 装 protobuf-compiler（apt 需 sudo，本次跳过）。

## 5. 风险与决策

| 风险 | Phase 0 处置 |
|---|---|
| cargo check 基线破裂 | 作为 Phase 0 必修复项纳入（commit 1） |
| `_nav2_action_provider` 字段外部 getattr 引用方众多 | 保留字段名（永不实例化）但 `_resolve_navigation_backend` 返回 None；metadata 仍可读，但不再被任何 backend 解析路径使用 |
| `_platform_exploration_runtime` 字段被 29 处引用 | 字段保留为 `legacy_exploration_placeholder`（重命名 + 注释），不参与默认 `_resolve_exploration_backend` |
| gateway `force_nav2_goal`/`debug_nav2_goal` 名称 | 保留旧入口函数作为 alias（LEGACY_NAV2_DISABLED 错误），新增 `force_runtime_goal`/`debug_navigation_goal` 为新默认路径 |
| `NUWAX_GEOMETRY_RUNTIME_ARGS` 外部配置方不知已 deprecated | registry 描述 + settings 加载时 warning，启动脚本不再消费 |
