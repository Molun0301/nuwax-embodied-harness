# Phase 6 — Python / gateway / service 收敛记录

> 2026-07-06
> 作者:phase6-executor
> 关联 plan:`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 6

## 1. 范围闭环

执行方案 §3 Phase 6 要求:让平台只感知 provider 协议,不再依赖历史外部脚本 / 旧命名判断。
本 lane 落盘:

- `services/geometry/geometry_runtime_provider.py` 新增 2 个 diagnostics facade
  方法:`get_mapping_diagnostics()` / `get_navigation_diagnostics()`(与 Phase 5
  `get_exploration_diagnostics()` 对齐成三套统一 dict)。
  失败统一返回 dict 携带 `error` 字段或返回 None,绝不假成功;
- `services/geometry/runtime.py` 物理 rename:
  - `_nav2_action_provider` → `_legacy_nav2_placeholder_disabled`
    (永不实例化,作为外部 getattr 占位字段保留);
  - `_platform_exploration_runtime` → `_legacy_exploration_placeholder_disabled`
    (29 处 helper 同步 rename;`NUWAX_EXPLORATION_LEGACY` env guard 保留
    作为 1 release 兼容期开关,默认 `false` 即默认路径不返回该字段);
  - `_build_nav2_status()` → `_build_navigation_status()`;
  - `runtime_status["nav2"]` → `runtime_status["navigation"]`;
  - 局部变量 `nav2_status` → `navigation_status`(诊断元数据 `nav2_readiness` /
    `nav2_ready` → `navigation_readiness` / `navigation_ready`);
  - 等待中字符串 `nav2_action` / `nav2_lifecycle` → `nuwax_navigation_runtime` /
    `nuwax_navigation_lifecycle`;
  - `_navigation_backend_policy()` 返回 dict 内 `nav2_action_provider_name` /
    `nav2_action_active` → `legacy_nav2_placeholder_provider_name` /
    `legacy_nav2_placeholder_active`;
  - `start_external_mapping` / `_start_external_script_exploration`
    docstring 顶部标注【Phase 6 legacy/manual】(默认路径仍走
    `_remote_provider` + Rust explore_runtime)。
- `gateways/runtime.py::_ensure_live_navigation_ready`:
  - 错误码 `nav2_not_ready` → `navigation_not_ready`;
  - `_build_nav2_status()` 调用 → `_build_navigation_status()`;
  - 错误 message 文案改为 "nuwax-geometry-runtime 导航未就绪"。
- `gateways/http/static/debug_panel.html`:前端错误码比对同步更新
  `failureReason === "navigation_not_ready"` 与 `code === "navigation_not_ready"`。
- `core/settings.py`:
  - 新增 module-level `NAV_PROFILE`(默认 `simple-2d`)/
    `EXPLORE_ARBITRATION`(默认 `prefer_wavefront`)/
    `REAL_LCM_REQUIRED`(默认 `false`)对接 Rust 装配面;
  - 新增 `_FAKE_LCM_ALLOWED_PROFILES = {"ci", "ci-test", "contract-test", "unit-test"}`;
  - `_normalize_lcm_backend` 增加 fake-lcm profile 校验:非允许 profile 设置
    `fake-lcm` 时打 warning,提示启动期 fake-lcm 会被 Rust runtime 拒绝。
- `core/config_registry.py`:`_ZH_NAMES` 注册 3 个 key 中文名(否则 Web 面板
  `/api/config/schema` 500)。
- `.env.example`:增加 Phase 6 段注释 + 3 个默认值。
- `scripts/ci/no_ros2_residual.allowlist`:物理 rename 后行号漂移,
  把旧 line 999 / 4640 更新为 line 1004 / 4651(同一注释,语义不变)。

## 2. 设计判断与权衡

### 2.1 为什么不在 provider 加 RPC 而是只新增 dict facade

本 lane 不新增 gRPC RPC,而是在 Python provider 侧把现有 `HealthCheck` /
`GetOccupancyGrid` / `GetNavigationReadiness` 合成 3 个 diagnostics dict facade。
理由:

1. Phase 5 已经接上 `StartExploration` / `StopExploration` / `GetExplorationState`
   / `GetExplorationDiagnostics` 4 个 RPC;`get_exploration_diagnostics()` 已经
   走 gRPC facade。
2. `HealthCheck` 已是 runtime 顶层健康探针,`GetOccupancyGrid` /
   `GetNavigationReadiness` 已有 readiness 细分字段;组合成 dict 既不引入新
   协议面,也不破坏现有 caller。
3. 平台 caller(debug_panel / MCP tools / web diagnostics)统一通过 dict 字段
   消费,未来 Phase 7 在 Rust 端把 readiness 字段升级(增减 costmap_revision
   / module_node_count / p1_native_profiles)也不破坏 Python 侧契约。

### 2.2 为什么 `_legacy_exploration_placeholder_disabled` 仍保留 29 处 helper

物理 rename 后,`_legacy_exploration_placeholder_disabled` 实例仍存在以驱动
`_platform_exploration_loop` / `tick` / `block_on_prerequisite` 等 29 处 helper
状态机。`NUWAX_EXPLORATION_LEGACY=false` 时:

- `_resolve_exploration_backend()` 不返回该字段;
- gateway/service 默认入口不会下发任何调用;
- helper 调用栈仅在 `_platform_exploration_loop` 内部空转
  (`_legacy_exploration_placeholder_disabled.is_running` 永远为 False)。

这是 1 release 兼容期方案:`NUWAX_EXPLORATION_LEGACY=true` 时旧 Python 状态机
仍可作为 legacy/manual 入口被外部触发;默认 false 状态下旧 helper 完全不
生效,新路径只走 `_remote_provider` → gRPC `StartExploration` → Rust
`explore_runtime`。

### 2.3 历史资产 metadata 中的 `nav2` 子键不动

执行方案 §3 Phase 6 明确"旧 Nav2 文案仅允许在历史资产读取分支"。
`runtime.py:4484 / :4500` 的 `"nav2": {...}` 是历史 map asset metadata
(用于描述历史 nav2 amcl 启动报告),属于历史资产命名空间,**保留不动**以确保
历史 checkpoint 探测不被破坏。

## 3. 验收证据

```bash
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过
# EXIT 0

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
# EXIT 0(无错误输出)

$ PYTHONPATH=. python3 -m py_compile services/geometry/geometry_runtime_provider.py services/geometry/runtime.py gateways/runtime.py
# EXIT 0

$ PYTHONPATH=. python3 -m py_compile core/settings.py core/config_registry.py
# EXIT 0(双解释器:.venv_model_download 验 pydantic 分支,
#  系统 python3 验 settings._normalize_lcm_backend 分支)

$ ! grep -rn "_nav2_action_provider\|_platform_exploration_runtime\|_build_nav2_status\|nav2_not_ready" \
    services/geometry/runtime.py gateways/runtime.py services/geometry/geometry_runtime_provider.py \
    gateways/http/static/debug_panel.html
# 0 hit
```

## 4. 改动文件清单

| 文件 | 变更类型 | 说明 |
|---|---|---|
| `services/geometry/geometry_runtime_provider.py` | +108 行 | 新增 `get_mapping_diagnostics` / `get_navigation_diagnostics` |
| `services/geometry/runtime.py` | rename | 物理 rename `_legacy_*` / `_build_navigation_status` / `navigation_*` |
| `gateways/runtime.py` | edit | 错误码 rename + 文案改 nuwax navigation |
| `gateways/http/static/debug_panel.html` | edit | 错误码比对同步 |
| `core/settings.py` | +module-level | NAV_PROFILE / EXPLORE_ARBITRATION / REAL_LCM_REQUIRED + fake-lcm profile 校验 |
| `core/config_registry.py` | +3 keys | _ZH_NAMES 注册 |
| `.env.example` | +段注释 + 3 keys | 默认值 |
| `scripts/ci/no_ros2_residual.allowlist` | line update | 999 → 1004, 4640 → 4651 |

## 5. commit hash

| commit | hash | 说明 |
|---|---|---|
| 1 | `cf3dfef` | `refactor(phase6): geometry_runtime_provider.py 新增 mapping/nav/explore diagnostics facade` |
| 2 | `b2b5fc9` | `refactor(phase6): runtime.py + gateway 物理 rename nav2/explore 旧命名 alias` |
| 3 | `843f961` | `feat(phase6): settings/config_registry/env 增加 NUWAX_NAV_PROFILE/NUWAX_EXPLORE_ARBITRATION/NUWAX_REAL_LCM_REQUIRED` |