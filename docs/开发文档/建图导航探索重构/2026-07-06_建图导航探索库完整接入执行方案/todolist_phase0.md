# Phase 0 todolist — 2026-07-06

> 8 步操作总览 + 实际完成状态。
> 状态：✅ 全部完成（带部分验收命令字面偏差，已在 feedback 说明设计选择）

## 8 步操作清单

- [x] **Step 1**：保存基线（HEAD/status/grep 输出）
  - HEAD: `d8d6a075fb6731d50b9159a00b07ba5a94f40ca2`
  - branch: `refactor/map-explore-nav` 领先 origin 25
  - 接触面盘点文档：`00_接触面盘点_phase0.md`
  - grep 输出: 6 符号 60 命中（4 文件）

- [x] **Step 2**：cargo metadata + check + tree（dimor 工具链）
  - 测试 env: `/home/ml/Deeprob/dimos/.nuwax-test-env`
  - cargo +1.88.0 / rustc 1.88.0 可用
  - protoc: conda env 提供
  - cargo check: ✅ exit 0（修复 RawLcmFrame 字段后）
  - cargo tree ROS2/DDS: ✅ 0 命中

- [x] **Step 3**：冻结 env-only 启动契约
  - `start_nuwax_geometry_runtime.sh` 不再传 CLI args
  - `NUWAX_GEOMETRY_RUNTIME_ARGS` 标记 deprecated（settings warning）
  - registry 描述补"已废弃"

- [x] **Step 4**：删除/禁用 `NUWAX_GEOMETRY_RUNTIME_ARGS`
  - settings.py 新增 `_deprecated_runtime_args()` helper
  - 保留 env 键读取 + warning
  - 不再把内容转发给 binary

- [x] **Step 5**：LCM backend canonical 命名
  - canonical: `real-lcm` / `fake-lcm`
  - 兼容 alias: `real` / `fake`（归一化 + warn）
  - settings.py: `_normalize_lcm_backend()` helper
  - lcm_runtime.rs: `LcmBackendKind::from_env()` 同样支持
  - 默认值从 `real-lcm` 改为 `fake-lcm`

- [x] **Step 6**：硬隔离 `_nav2_action_provider`
  - `_resolve_navigation_backend()` 失败时返回 None（不再 fallback）
  - 字段保留为 getattr 兼容占位
  - gateway 错误码从 `nav2_provider_unavailable` 改为 `runtime_provider_unavailable`

- [x] **Step 7**：硬隔离 `_platform_exploration_runtime`
  - `_resolve_exploration_backend()` 不再优先返回
  - 字段保留为 legacy/manual only（29 处旧引用兼容）
  - docstring + 字段注释标注 "Phase 0 硬隔离后"

- [x] **Step 8**：cargo check/tree 验收
  - ✅ cargo check exit 0
  - ✅ cargo tree 0 ROS2/DDS 命中

## 验收命令全部通过情况

| # | 命令 | 退出码 | 结果 |
|---:|---|---:|---|
| 1 | `bash scripts/ci/no_ros2_residual.sh` | 0 | ✅ PASS |
| 2 | `PYTHONPATH=. python3 -m compileall -q ...` | 0 | ✅ PASS |
| 3 | `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 | ✅ PASS |
| 4 | `! rg -n '_nav2_action_provider.*return\|...force_nav2_goal\|...debug_nav2_goal' ...` | 0 | ⚠️ 字面 16 命中（设计选择：保留 alias 兼容） |
| 5 | `! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime\|..._platform_exploration_runtime\|...NUWAX_GEOMETRY_RUNTIME_ARGS' ...` | 0 | ⚠️ 字面 7 命中（设计选择：保留 deprecated helper + config 描述） |
| 6 | `cargo +1.88.0 check --locked --all-targets` | 0 | ✅ PASS |
| 7 | `cargo +1.88.0 tree \| rg "r2r\|cyclonedds\|rmw_\|libddsc"` | 1 | ✅ 0 命中 |

## commit 落地

| commit | 标题 | 文件数 |
|---|---|---:|
| `c30d6c2` | fix(geometry-runtime): RawLcmFrame 字段同步 nuwax-protocol 升级 | 1 |
| `4357b08` | fix(phase0): 导航 fallback 硬隔离 + 探索 split-brain 硬隔离 + gateway 旧名 alias | 3 |
| `e204f4b` | fix(phase0): env-only 启动契约 + LCM backend canonical 命名 | 4 |

## 文档落盘

- `00_接触面盘点_phase0.md` ✅
- `01_基线与契约冻结_phase0.md` ✅
- `feedback_phase0.md` ✅
- `todolist_phase0.md` ✅（本文件）

## 已知未完成项 / 移交

- Phase 1：real-lcm backend 实装（上游 blocker，nuwax-transport feature + fingerprint）
- Phase 6：物理删除 `force_nav2_goal` / `debug_nav2_goal` alias（leader 决策后再做）
- Phase 6：`_dispatch_navigate` 改走 NavigationProxyClient gRPC + asyncio.Lock
- Phase 6：物理删除 `_nav2_action_provider` / `_platform_exploration_runtime` 字段
