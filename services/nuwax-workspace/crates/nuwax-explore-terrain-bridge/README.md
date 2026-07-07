# nuwax-explore-terrain-bridge

> **STATUS: BLOCKED — blocked-by-native-runner-preflight**
>
> 本 crate 的完整实现 / 全部验收(包括 `cargo test`)以
> `cargo check -p nuwax-nav-native-runner` 通过为硬前置。native-runner 未绿时
> 本 crate 状态必须标为 **blocked**,不得声称 "已完成" 或 spec/argv 纯测试通过。
> 完整生命周期测试(mock-native)与 typed schema 验收在 native-runner 编译绿之后
> 才可执行。

## 职责

包装 CMU `terrain_analysis` C++ 原生进程。本 crate 是 **topic-only bridge**:

- 仅配置 / 监管 native 进程发布 `terrain_map` native topic;
- **不**提供 Rust typed terrain-map point cloud output;
- **不**做 point cloud codec / transport 解码;
- **不**是 terrain-map-ext;**不**做地形扩展算法。

需要 typed terrain-map-ext 数据时,使用 [`nuwax-nav-terrain-map-ext`](../nuwax-nav-terrain-map-ext/)
或 [`nuwax-explore-terrain-map-bridge`](../nuwax-explore-terrain-map-bridge/) adapter。

完成定义参见
`docs/子任务/探索模块重写/探索模块重写方案.md` §6 / AC-7。

## 公共 API

```rust
use nuwax_explore_terrain_bridge::{
    // typed config
    TerrainAnalysisBridgeConfig, cli_name_override, cli_exclude,
    // topics
    TerrainAnalysisTopics,
    // argv 渲染与 spec 构造
    to_native_args, build_specs, validate_config,
    // 顶层 facade
    TerrainAnalysisBridge, TerrainBridgeError, TerrainBridgeResult,
    // bridge-local supervisor
    BridgeSupervisor, BridgeSupervisorState, BridgeDiagnostics, DEFAULT_RESTART_POLICY,
};
```

### Topic 端口

固定 3 个端口:

| Rust 字段 | CLI 渲染 port | 默认 |
|---|---|---|
| `registered_scan` | `--registered_scan` | None |
| `odometry` | `--odometry` | None |
| `terrain_map` | `--terrain_map` | None |

topics 仅通过 `TerrainAnalysisTopics::to_topic_map()` 填入
`NativeProgramSpec.topics`,由 native-runner 统一渲染 `--{port} {topic}`,避免 topic argv 重复。

### Config 字段(30+)

完整迁移自历史 `TerrainAnalysisConfig`。`cli_name_override()` 返回
`BTreeMap<String, String>`,把 Rust snake_case 字段名映射为 C++ 原生 wrapper 的
camelCase 名(`--sensorRange` / `--scanVoxelSize` / `--terrainVoxelHalfWidth` /
`--obstacleHeightThre` / `--useSorting` / `--decayTime` / `--noDecayDis` /
`--clearingDis` / `--clearDyObs` / `--noDataBlockSkipNum` / `--minBlockPointNum` /
`--voxelPointUpdateThre` / `--voxelTimeUpdateThre` / `--minDyObsDis` /
`--absDyObsRelZThre` / `--minDyObsVFOV` / `--maxDyObsVFOV` / `--minDyObsPointNum` /
`--minOutOfFovPointNum` / `--considerDrop` / `--limitGroundLift` /
`--maxGroundLift` / `--disRatioZ` 等)。

`cli_exclude()` 返回 `BTreeSet<String>`,排除 runner 字段
(`executable` / `cwd` / `env` / `build_command` / `build_policy` / `restart_policy`)
使其不进入 argv。

`None` 字段不渲染(可选 typed 字段默认全 `None`)。

### 单一 restart policy 来源

`config.restart_policy` 写入 `NativeProgramSpec.restart`,
`BridgeSupervisor` 只读取同一字段,不维护第二份可漂移 policy(方案 §5.5)。
默认 `DEFAULT_RESTART_POLICY = OnFailure { max_restarts: 3, backoff_millis: 1_000 }`。

## 不做事项

- **不**订阅 / **不**解码 native `terrain_map` topic;
- **不**导出 / **不**实现 Rust typed point cloud output;
- **不**实现 terrain-map-ext 算法(该算法唯一 owner 是 `nuwax-nav-terrain-map-ext`);
- **不**重写 C++ 内部算法;
- **不**直接管理 navigation stack 集成(stack 接线走 `nuwax-nav-stack` /
  `nuwax-exploration`)。

## 验收命令

```bash
# 硬前置(必须先通过):
cargo check -p nuwax-nav-native-runner

# 本 crate:
cargo check -p nuwax-explore-terrain-bridge
cargo clippy -p nuwax-explore-terrain-bridge --all-targets -- -D warnings
cargo test -p nuwax-explore-terrain-bridge
```
