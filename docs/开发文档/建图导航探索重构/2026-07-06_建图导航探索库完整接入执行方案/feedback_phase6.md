# Phase 6 feedback — Python / gateway / service 收敛

> 2026-07-06
> 作者:phase6-executor
> 关联 plan:`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 6

## 1. 范围闭环

执行方案 §3 Phase 6 要求:让平台只感知 provider 协议,不再依赖历史外部脚本 /
旧命名判断。本 lane 落盘 3 个 commit:

| commit | 范围 | 主要文件 |
|---|---|---|
| `cf3dfef` | provider 新增 mapping/nav diagnostics facade | `services/geometry/geometry_runtime_provider.py` |
| `b2b5fc9` | 物理 rename nav2/explore 旧命名 alias + 错误码 + legacy 标记 | `services/geometry/runtime.py`、`gateways/runtime.py`、`gateways/http/static/debug_panel.html`、`scripts/ci/no_ros2_residual.allowlist` |
| `843f961` | settings/config_registry/.env 增加 3 个 Phase 6 key + fake-lcm profile 校验 | `core/settings.py`、`core/config_registry.py`、`.env.example` |

## 2. 主要决策与权衡

### 2.1 provider diagnostics 用 dict facade,不新增 RPC

本 lane 不增 gRPC RPC,把现有 `HealthCheck` / `GetOccupancyGrid` /
`GetNavigationReadiness` 合成 3 个 diagnostics dict facade。
理由:`HealthCheck` 已是 runtime 顶层健康探针;`GetOccupancyGrid` /
`GetNavigationReadiness` 已有细分字段;组合成 dict 引入新协议面 = 0,
也不破坏现有 caller。Phase 5 已接 `StartExploration` 等 4 RPC,本 phase 6
主要补 mapping/nav 两个 facade,与 Phase 5 对齐成三套。

### 2.2 `_legacy_exploration_placeholder_disabled` 仍保留 29 处 helper

物理 rename 后实例仍存在以驱动 `_platform_exploration_loop` / `tick` /
`block_on_prerequisite` 等 29 处 helper 状态机。`NUWAX_EXPLORATION_LEGACY=false`
时 `_resolve_exploration_backend` 不返回该字段,gateway/service 默认入口不会
下发调用;helper 调用栈仅在 `_platform_exploration_loop` 内部空转
(`is_running` 永远 False)。

1 release 兼容期方案:`NUWAX_EXPLORATION_LEGACY=true` 时旧 Python 状态机
仍可作为 legacy/manual 入口;默认 false 状态下旧 helper 完全不生效,新路径只
走 `_remote_provider` → gRPC → Rust `explore_runtime`。

### 2.3 历史资产 metadata 中的 `nav2` 子键不动

执行方案 §3 Phase 6 明确"旧 Nav2 文案仅允许在历史资产读取分支"。
`runtime.py:4484 / :4500` 的 `"nav2": {...}` 是历史 map asset metadata
(用于描述历史 nav2 amcl 启动报告),属于历史资产命名空间,**保留不动**以确保
历史 checkpoint 探测不被破坏。这一类不在 Phase 6 收敛范围内。

## 3. 验收证据

```bash
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过
# EXIT 0

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
# EXIT 0

$ PYTHONPATH=. python3 -m py_compile services/geometry/geometry_runtime_provider.py services/geometry/runtime.py gateways/runtime.py
# EXIT 0

$ ! grep -rn "_nav2_action_provider\|_platform_exploration_runtime\|_build_nav2_status\|nav2_not_ready" \
    services/geometry/runtime.py gateways/runtime.py services/geometry/geometry_runtime_provider.py \
    gateways/http/static/debug_panel.html
# 0 hit
```

## 4. 与 Phase 0/3-5 的衔接

- Phase 0 已硬隔离 `_resolve_navigation_backend`(永不返回
  `_nav2_action_provider`)、`dispatch_debug_navigation_goal` /
  `force_runtime_goal` rename。本期 commit 2 完成物理 rename,
  把字段名本身改为 `_legacy_nav2_placeholder_disabled` /
  `_legacy_exploration_placeholder_disabled` 让禁用语义直接在字段名里表达。
- Phase 3-5 fixup 已加 `NUWAX_EXPLORATION_LEGACY` guard。本期保留该 guard
  作为 1 release 兼容期开关,未删除;commit 2 把 29 处 helper 物理 rename
  到 `_legacy_exploration_placeholder_disabled`(行为不变)。
- Phase 5 已接 `StartExploration` / `StopExploration` / `GetExplorationState` /
  `GetExplorationDiagnostics` 4 个 gRPC RPC;本期不再动 proto,也不新增 RPC。
- 本期新增 3 个 settings key:
  - `NUWAX_NAV_PROFILE`(默认 `simple-2d`)→ Rust `nav_runtime.rs` 装配
  - `NUWAX_EXPLORE_ARBITRATION`(默认 `prefer_wavefront`)→ Rust `explore_runtime.rs` 装配
  - `NUWAX_REAL_LCM_REQUIRED`(默认 `false`)→ fail-fast 硬约束开关
  三者均对接 nuwax-geometry-runtime (Rust) 装配面,不再走旧 rclpy/Nav2。

## 5. 遗留 blocker

- 本地 Python 解释器缺 `pydantic`,`from core import config_registry` 会
  因 `core/__init__.py → core/capability_registry.py` 报 ModuleNotFoundError。
  这一限制与 Phase 1/2/3/4/5 一致(user memory:`pytest+pydantic 不可用`),
  通过 `.venv_model_download/bin/python3` 验证 pydantic 分支,
  通过系统 python3 验证 settings / config_registry 静态导入分支。
- 单元测试与 Go2 实机验收属 Phase 7 范围,本期不展开。