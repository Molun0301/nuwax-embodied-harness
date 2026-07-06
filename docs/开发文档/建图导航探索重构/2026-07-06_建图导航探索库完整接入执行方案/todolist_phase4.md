# Phase 4 todolist — 导航接入 nav_runtime

> 日期：2026-07-06

## 主任务

- [x] P4.1 读 dimos nav-stack / nav-movement-manager 实际 API（`crates/nuwax-nav-stack/src/{lib,builder,profiles}.rs` + `crates/nuwax-nav-movement-manager/src/lib.rs` + `crates/nuwax-nav-basic-path-follower/src/lib.rs`）
- [x] P4.2 新建 `src/nav_runtime.rs`（profile 解析 + module graph + 状态机 + set_goal/cancel/readiness）
- [x] P4.3 lcm_runtime.rs: handle 加 `nav` 字段 + `build_handle_for_tests` + 注入默认 `simple-2d`
- [x] P4.4 server.rs: 3 个 `UNIMPLEMENTED` RPC 实装 + `nav_state_to_proto` 转换工具
- [x] P4.5 Cargo.toml: 加 `nuwax-nav-basic-path-follower` path dep + `uuid` dep
- [x] P4.6 main.rs: 增 `mod nav_runtime;`
- [x] P4.7 Python `geometry_runtime_provider.py:get_navigation_readiness` 解析 JSON reason → 10 字段 dict
- [x] P4.8 单元测试：profile 解析、phase 字符串、readiness roundtrip、set_goal/cancel 状态机、follower step
- [x] P4.9 验收命令全部跑过（no_ros2 / compileall / bash -n / 4 个 rg / cargo check / cargo tree）
- [x] P4.10 落盘 `30_导航接入nav_phase4.md` + `feedback_phase4.md` + `todolist_phase4.md`

## 验证清单（验收命令实际结果）

| 验收命令 | 实际结果 |
|---|---|
| `bash scripts/ci/no_ros2_residual.sh` | 0 PASS |
| `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` | 0 PASS |
| `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 PASS |
| `rg -n '_nav2_action_provider.*return\|return getattr\(self, "_nav2_action_provider"\|force_nav2_goal\|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py` | 0 命中 |
| `rg -n '_resolve_exploration_backend.*_platform_exploration_runtime\|return getattr\(self, "_platform_exploration_runtime"\|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py` | 0 命中 |
| `rg -n "SetNavigationGoal.*UNIMPLEMENTED\|CancelNavigation.*UNIMPLEMENTED" services/geometry/nuwax_geometry_runtime/src/server.rs` | 0 命中 |
| `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets` | 4 errors 全在 explore_runtime.rs (Phase 5) |
| `cargo tree ... \| rg "nuwax-nav-stack\|nuwax-nav-simple-planner\|nuwax-nav-astar\|nuwax-nav-basic-path-follower"` | 4 命中 |
| `cargo tree ... \| rg "r2r\|cyclonedds\|rmw_\|libddsc"` | 0 命中 |
| `PYTHONPATH=. python3 -m py_compile services/geometry/proto_gen/geometry_runtime_pb2.py ...` | 0 |

## 阻塞

- cargo check 4 errors 在 `explore_runtime.rs:362-364` (Phase 5 范围)，
  本阶段无关。

## 提交批次

1. `feat(phase4): nav_runtime.rs + lcm_runtime nav 句柄 + server 3 个 UNIMPLEMENTED 替换`
2. `feat(phase4): get_navigation_readiness 细分 JSON 子项 + Python provider 解析`
3. `docs(phase4): 30_导航接入nav_phase4.md + feedback_phase4.md + todolist_phase4.md`
