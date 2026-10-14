# nuwax-explore-tare-bridge

> **硬前置门**: `cargo check -p nuwax-nav-native-runner` 未通过;E3 处于 `blocked-by-native-runner-preflight` 状态。
>
> 在 native-runner preflight 转绿前,本 crate 的 `cargo test` / `cargo check` 验收不得视为完成;按照方案 §5.5 / §10 Phase E3 硬约束执行。

TARE Planner 原生 C++ 进程 bridge(`result/bin/tare_planner`),仅负责 typed config / topic port 表 / argv 渲染 / 进程生命周期 / supervisor 状态与 diagnostics,不重写算法,不订阅/解码 native `way_point` payload。

## 公共 API 边界

- 暴露:`TarePlannerTopics`、`TareWaypoint`、`TarePlannerBridgeConfig`、`TarePlannerBridge`、`build_specs`、`to_native_args`、`validate_config`、`to_topic_map`、`BridgeSupervisor`、`BridgeSupervisorState`、`BridgeDiagnostics`(方案 §5.5)。
- 不暴露:裸 `NativeRunner` lifecycle、C++ 算法结构、native `way_point` payload decoder。
- `TareWaypoint -> FrontierGoal` 唯一归属 `nuwax-exploration::TareWaypointGoalSourceAdapter`,**不在本 crate 完成**。
- `FrontierGoal -> NavigationGoal/FrontierGoalInput` 唯一归属 `nuwax-exploration::conversions`,**不在本 crate 完成**。

## 模块布局

```
crates/nuwax-explore-tare-bridge/
├── Cargo.toml
├── README.md
├── src/
│   ├── lib.rs        # 公开 API 入口 / 默认值 / re-export
│   ├── spec.rs       # TarePlannerTopics / to_topic_map / build_specs / validate_config / to_native_args
│   ├── waypoint.rs   # TareWaypoint typed schema + validate + error
│   ├── bridge.rs     # TarePlannerBridge lifecycle 封装(经 BridgeSupervisor)
│   └── supervisor.rs # BridgeSupervisor / BridgeSupervisorState / BridgeDiagnostics / RestartPolicy 单一来源
└── tests/
    ├── spec_argv.rs
    ├── config_validation.rs
    ├── integration_mock.rs
    └── waypoint_schema.rs
```

## BridgeSupervisor 默认策略

`RestartPolicy::OnFailure { max_restarts: 3, backoff_millis: 1_000 }`(方案 §5.5 默认值,supervisor 不维护第二份 policy)。

## 测试

```bash
# 硬前置(必须先绿)
cargo check -p nuwax-nav-native-runner
# 默认测试(本 crate)
cargo test -p nuwax-explore-tare-bridge
# Lint
cargo clippy -p nuwax-explore-tare-bridge --all-targets -- -D warnings
```

mock-native 集成测试使用 `nuwax-nav-test-fixtures` 中的 `mock-native` binary;缺资源时按 `cargo test` 自动 skip(不阻塞 CI)。

## 完成定义

参见 `docs/子任务/探索模块重写/探索模块重写方案.md` §5 / AC-6。
