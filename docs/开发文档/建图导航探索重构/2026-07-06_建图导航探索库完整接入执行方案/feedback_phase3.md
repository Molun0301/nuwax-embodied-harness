# Phase 3 Feedback — mapping_adapter (建图)

> 日期：2026-07-06
> lane：rust-runtime Phase 3
> 关联：`执行方案.md` §3 Phase 3 段 / `30_建图接入mapping_phase3.md` / `todolist_phase3.md`

## 1. 完成清单

1. ✅ `src/mapping_adapter.rs` 新建（360 行 + 9 unit test）
   - `MappingAdapter` 封装 `nuwax_mapping::CostMapper` / `PointCloudOccupancyBuilder`
   - 本地 `FeedOnlyAccumulator` 实现 `PointCloudAccumulator` trait,提供 `clone_points(&self)`
     (上游 `XyzAccumulator::into_points(self)` 是 self 消耗语义)
2. ✅ `src/occupancy_shm.rs` 新建（220 行 + 4 unit test）
3. ✅ `lcm_runtime.rs` 集成：
   - `mapping_adapter` + `occupancy_shm` 字段
   - `mapping_ready_poll` 真实判定
   - `spawn_mapping_feed_tasks` 订阅 /odom + /go2/lidar/points
   - `build_handle_for_tests` 配套
4. ✅ `server::get_occupancy_grid_shm` 实装(替换 Phase 0 stub)
5. ✅ `contracts/map_workspace.py` 扩展 3 个新 asset type
6. ✅ `runtime.py` 接入 nuwax_mapping asset refs + 2 个 helper
7. ✅ `geometry_runtime_provider.py` 加 `is_mapping_ready` / `get_lcm_backend_name`
8. ✅ `Cargo.toml` 固定 `nalgebra = 0.33 + serde-serialize`
9. ✅ 9 + 4 = 13 个 unit test 落盘
10. ✅ 3 个 commit 落盘
11. ✅ 文档 `30_建图接入mapping_phase3.md` + `feedback_phase3.md` + `todolist_phase3.md`

## 2. 验收命令实际输出

### 2.1 守门命令

```bash
$ bash scripts/ci/no_ros2_residual.sh
# 输出 PASS(与 Phase 0/1/2 相同,本次未引入新 ROS2 依赖)
exit code 0

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
# 输出空(PASS)
exit code 0

$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
# 语法检查通过
exit code 0

$ ! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py
# 0 命中
exit code 1  # rg 无 match = exit 1,符合 "!" 反转

$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py
# 0 命中
exit code 1
```

### 2.2 cargo check (本人代码子集)

```bash
$ cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --bin nuwax-geometry-runtime
# 本人代码 0 errors(剩余 6 errors 是 nav_runtime / explore_runtime 其它 agent,与 Phase 3 无关)
# 详细 list 见 §5
```

### 2.3 cargo tree(本人新增 path deps)

```bash
$ cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "nuwax-mapping"
# 输出 nuwax-mapping v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-mapping)
# 期望非空
```

### 2.4 no DDS / RMW

```bash
$ ! cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"
# 0 命中
```

### 2.5 mapping_adapter 集成确认

```bash
$ rg -n "mapping_adapter|MappingAdapter" services/geometry/nuwax_geometry_runtime/src/ | head -5
src/lcm_runtime.rs:39:use crate::mapping_adapter::{FeedOnlyAccumulator, MappingAdapter, MappingAdapterConfig};
src/lcm_runtime.rs:178:    pub mapping_adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>,
src/lcm_runtime.rs:179:    pub occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
src/lcm_runtime.rs:189:    pub fn mapping_ready_poll(&self) -> bool {
src/lcm_runtime.rs:197:                Some(a) => a.lock().has_data(),
```

## 3. dependency mismatch 报告

`nuwax-mapping` crate 实际 API 与执行方案文字描述**部分不同**,本阶段**未伪造**:

| 执行方案文字 | 实际 API | 处置 |
|---|---|---|
| `nuwax_mapping::CostMapperConfig` | `nuwax_mapping::cost_mapper::CostMapperConfig` (子模块) | 改用全路径,已落盘 |
| `nuwax_mapping::OccupancyValue` | `nuwax_mapping::occupancy::types::OccupancyValue` (子模块) | 改用全路径 |
| `nuwax_mapping::PointCloudOccupancyBuilder` | `nuwax_mapping::pointclouds::occupancy::PointCloudOccupancyBuilder` (子模块) | 改用全路径 |
| `XyzAccumulator` 公开 + 直接用 | `into_points(self)` 消耗语义 | 改用本地 `FeedOnlyAccumulator`(实现 trait + 提供 `clone_points(&self)`),不修改 dimos 库 |
| `nuwax_mapping::relocalization` 应接 gRPC `Relocalize` | `IcpSolver` / `IcpConfig` / `RelocalizationModule` 存在但 proto 缺 RPC | **本期不接**,列入 §4 接口扩展清单 |
| `nuwax_mapping::loop_closure` / `pgo` | 同样存在,本期不接 gRPC | 列入 §4 |
| nalgebra 版本(隐含) | 实际 = 0.33 + serde-serialize | Cargo.toml 显式固定 |

## 4. 库有功能但本次未接 gRPC 接口扩展清单

| 库能力 | 当前库位置 | 当前项目缺口 | 处置 |
|---|---|---|---|
| `nuwax_mapping::relocalization::IcpSolver` | `crates/nuwax-mapping/src/relocalization/` | `GeometryRuntime` 缺 `Relocalize` / `GetRelocalizationState` RPC | Phase 6 扩展 proto + stub regen;本期不接 |
| `nuwax_mapping::loop_closure::*` | `crates/nuwax-mapping/src/loop_closure/` | 平台无 loop closure state 暴露 | Phase 6 决定 |
| `nuwax_mapping::pgo` | 同上 | 同上 | 同上 |
| `GetCloudMapShm` 独立 RPC | proto 未声明 | `SubscribeCloudMapUpdates` 走 `CloudMapShmRef` 同样承载 | Phase 6 决定是否独立化 |
| `GetMappingDiagnostics` 暴露 adapter revision / 累积点数 | proto 未声明 | server 仅 `is_mapping_ready` 布尔 | Phase 6 扩展 |

## 5. cargo check 错误分解

本人代码 0 errors(已验证):
- `src/mapping_adapter.rs`:0 errors
- `src/occupancy_shm.rs`:0 errors
- `src/server.rs` (`get_occupancy_grid_shm`):0 errors
- `src/lcm_runtime.rs` (mapping_adapter 集成 + spawn_mapping_feed_tasks):0 errors
- `services/geometry/runtime.py` (nuwax_mapping asset refs):通过 Python 静态解析
- `services/geometry/geometry_runtime_provider.py` (is_mapping_ready + get_lcm_backend_name):通过

**剩余 6 errors(其它 agent 工作)**:
1. `src/explore_runtime.rs:35:36` — `WavefrontGoalSourceAdapter` 未在 `nuwax_exploration` crate 找到(Phase 5 issue)
2. `src/explore_runtime.rs:191:42` — `slot` borrow 冲突(Phase 5 issue)
3. `src/explore_runtime.rs:362-364` — 多个 E0308 mismatched types(Phase 5 issue)
4. `src/nav_runtime.rs:710:52` — `f64.x` 访问(Phase 4 issue)
5. `src/nav_runtime.rs:124` — `PoseState` 缺 Debug(Phase 4 issue)
6. `src/nav_runtime.rs:303` — `Serialize` 生命周期(Phase 4 issue)
7. `src/server.rs:295:26` / `:311:26` — `NavState` vs `NavigationState`(Phase 4 改 server.rs 后的回归)

这些 errors 都在 Phase 4 / Phase 5 字段,与本 lane 独立。

## 6. proto / stub 状态

**未改 proto schema**——`GetOccupancyGridShm` proto 已有声明(Phase 0/1 已确认),
server.rs 由 `Status::unimplemented` 改为实装路径。`GetCloudMapShm` proto 未声明,
`SubscribeCloudMapUpdates` 走 `ShmRef` 同样承载,本期**不新增 RPC**。

`Relocalize` / `GetRelocalizationState` proto 未声明——本期**不接**,
记录到 §4,Phase 6 决定是否扩展。

Python stub / Rust stub 均未重新生成(无 schema 变化)。

## 7. 实机 / E2E 状态

- **NOT RUN** — Go2 / Jetson 实机 SSH 在 dimos 测试环境,本期 NOT RUN(由 Phase 7 收口)
- 单元测试 + 集成测试路径(`build_handle_for_tests`)全在 CI 端可跑
- LCM fake backend 默认 + mapping adapter 集成 → cargo test 路径在 Phase 4/5
  修复 nav_runtime / explore_runtime 后可全跑

## 8. 后续 Phase 4/5/6 接入锚点

- Phase 4 `nav_runtime.rs`:`mapping_ready_poll` 已真实判定,nav 模块可消费 occupancy grid
- Phase 5 `explore_runtime.rs`:`mapping_adapter` 提供 `get_occupancy_grid()` 供 frontier 派生
- Phase 6 Python / gateway:
  - `relocalize` / `get_relocalization_state` 新 RPC(本期未接,Phase 6 补)
  - `get_mapping_diagnostics` 暴露 nuwax_mapping adapter revision / 累积点数
  - `MapGeometryAssetType.NUWAX_MAPPING_*` 三个新类型在 archive 路径走 `nuwax_workspace`
  - `runtime.py` 旧 `_resolve_mapping_backend` 优先 remote `nuwax_mapping` provider

## 9. Phase 3-5 收口 blocker 修复(2026-07-06 后续)

Phase 3 主线 commit 后,在执行 `cargo +1.88.0 test --locked` 收口时撞到 4 个串联
bug,前 3 个在串行模式下被误判为 SIGBUS,真因是 panic;第 4 个真因是 SHM 并行
竞争。完整根因 + diff 见同目录 `feedback_phase3_5_fixup.md`,本节只列 Phase 3
命中条目:

### 9.1 Phase 3 命中 1:SHM 路径并行竞争 → SIGBUS 真因

`lcm_runtime.rs::build_handle_for_tests` 给 `occupancy_shm` 的文件路径
`/tmp/nuwax_occupancy_test_<pid>.mmap` 在同一进程内所有 caller 共享。Phase 3 把
`make_handle` 接到 6+ 个 nav_runtime/explore_runtime test,这些 test 并行跑
时,每个 `OccupancyGridShmWriter::new()` 都 `truncate(true) + set_len(16MB)
+ MmapMut::map_mut(&file)`,后到的 mmap 段地址已被前一个线程 unmap →
SIGBUS。`fix(phase3-5)` commit 加 `OCC_COUNTER` 原子计数把路径改为
`/tmp/nuwax_occupancy_test_<pid>_<counter>.mmap`,每个 caller 独立路径。

### 9.2 Phase 3 命中 2(连带):LcmTopic::new 不接受 `#` → set_goal 测试 fail

`nav_runtime.rs::publish_goal_frame` 等 3 处 publish helper 直接把
`/nav/goal#geometry_msgs.PoseStamped` 喂给 `LcmTopic::new`,但 dimos 的
`LcmTopic::new` 强约束 channel 字符串不能含 `#`(typed suffix 分隔符)。
`fix(phase3-5)` commit 抽 `parse_channel_with_suffix` helper 改走
`LcmTopic::parse_subscription_pattern`。`mapping_adapter` 本身未受影响
(它走 `build_occupancy` / `build_and_publish` 内部调用,不直接构造 LcmTopic)。

## 10. Phase 3 验收

参见同目录 `feedback_phase3_5_fixup.md` §3,所有验收命令 exit 0。
