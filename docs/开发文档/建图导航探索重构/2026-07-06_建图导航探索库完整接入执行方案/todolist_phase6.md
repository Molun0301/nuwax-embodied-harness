# Phase 6 todolist — Python / gateway / service 收敛

> 2026-07-06
> lane:phase6-executor

## 必做(本 lane 范围)

- [x] 1. Read 现状
  - `services/geometry/geometry_runtime_provider.py:406-522`(mapping/lcm diagnostics 现状)
  - `services/geometry/runtime.py:516-540`(字段定义)+ `:1690-1770`(nav2 backend 描述)
    + `:3279-3340`(_resolve_*_backend 三件)+ `:5290-5630`(_platform_exploration_runtime
    helper 调用栈 29 处)+ `:6058-6277`(runtime_status["nav2"] callers)
  - `gateways/runtime.py:6500-6625`(`_ensure_live_navigation_ready` / `force_runtime_goal`)
    + `:4826-4970`(`dispatch_debug_navigation_goal` 现状)+ `:4828-4860`
    (`_handle_start_mapping` 调 `start_external_mapping`)
  - `core/settings.py:160-220`(`_normalize_lcm_backend` + NAV_* module-level)
    + `:700-1000`(geometry backend + nav + LCM channels 装配)
  - `core/config_registry.py:380-410`(NUWAX_LCM_BACKEND ZN name 现状)
  - `.env.example:231-310`(LCM/NAV/LCM CHANNELS 现状)
  - `services/geometry/nuwax_geometry_runtime/proto/geometry_runtime.proto:212-215`
    (Phase 5 已加 4 RPC 现状)

- [x] 2. provider diagnostics facade
  - `services/geometry/geometry_runtime_provider.py` 新增 `get_mapping_diagnostics()`
  - 新增 `get_navigation_diagnostics()`
  - Phase 5 `get_exploration_diagnostics()` 不动,保留 3 套统一
  - 失败统一返回 dict 携带 `error` 字段或 None,绝不假成功
  - 编译 OK(`py_compile`)

- [x] 3. runtime.py 物理 rename
  - `_nav2_action_provider` → `_legacy_nav2_placeholder_disabled`
  - `_platform_exploration_runtime` → `_legacy_exploration_placeholder_disabled`
  - `_build_nav2_status` → `_build_navigation_status`
  - `runtime_status["nav2"]` → `runtime_status["navigation"]`
  - `nav2_status` / `nav2_readiness` / `nav2_ready` → `navigation_status` /
    `navigation_readiness` / `navigation_ready`(局部变量 + metadata key)
  - 等待字符串 `nav2_action` / `nav2_lifecycle` → `nuwax_navigation_runtime` /
    `nuwax_navigation_lifecycle`
  - `_navigation_backend_policy()` dict 内 `nav2_action_*` → `legacy_nav2_placeholder_*`
  - `start_external_mapping` / `_start_external_script_exploration` docstring
    顶部加【Phase 6 legacy/manual】标记
  - 物理 rename 后行号漂移 → `no_ros2_residual.allowlist` 同步更新 999 → 1004,
    4640 → 4651
  - 编译 OK

- [x] 4. gateway 错误码 rename + 文案收敛
  - `gateways/runtime.py::_ensure_live_navigation_ready`:
    `nav2_not_ready` → `navigation_not_ready`;文案改"nuwax-geometry-runtime 导航未就绪"
  - `gateways/http/static/debug_panel.html`:
    `failureReason === "navigation_not_ready"` 与 `code === "navigation_not_ready"`
  - 编译 OK

- [x] 5. settings + config_registry + .env
  - `core/settings.py` 新增 module-level `NAV_PROFILE` / `EXPLORE_ARBITRATION` /
    `REAL_LCM_REQUIRED`(默认 simple-2d / prefer_wavefront / false)
  - `core/settings.py` 新增 `_FAKE_LCM_ALLOWED_PROFILES` 约束 fake-lcm
  - `core/settings.py::_normalize_lcm_backend` 增加 fake-lcm profile 校验,
    非允许 profile 设 fake-lcm 打 warning
  - `core/config_registry.py:_ZH_NAMES` 注册 3 个 key 中文名
  - `.env.example` 增 Phase 6 段注释 + 3 个默认值
  - 编译 OK

- [x] 6. 验收脚本跑全
  - `bash scripts/ci/no_ros2_residual.sh` PASS (exit 0)
  - `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` PASS (exit 0)
  - `PYTHONPATH=. python3 -m py_compile services/geometry/geometry_runtime_provider.py services/geometry/runtime.py gateways/runtime.py` PASS
  - `PYTHONPATH=. python3 -m py_compile core/settings.py core/config_registry.py` PASS
  - `! grep -rn "_nav2_action_provider\|_platform_exploration_runtime\|_build_nav2_status\|nav2_not_ready" services/geometry/runtime.py gateways/runtime.py services/geometry/geometry_runtime_provider.py gateways/http/static/debug_panel.html` 0 hit

- [x] 7. 文档落盘
  - `60_Python_gateway_service收敛_phase6.md`
  - `feedback_phase6.md`
  - `todolist_phase6.md`

## 验收命令退出码

- no_ros2_residual:0
- compileall services+core+adapters+providers+gateways+skills+contracts:0
- py_compile provider+runtime+gateway:0
- py_compile settings+config_registry:0

## commit

1. `cf3dfef` `refactor(phase6): geometry_runtime_provider.py 新增 mapping/nav/explore diagnostics facade`
2. `b2b5fc9` `refactor(phase6): runtime.py + gateway 物理 rename nav2/explore 旧命名 alias`
3. `843f961` `feat(phase6): settings/config_registry/env 增加 NUWAX_NAV_PROFILE/NUWAX_EXPLORE_ARBITRATION/NUWAX_REAL_LCM_REQUIRED`