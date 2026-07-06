# Phase 1 实施 feedback — 2026-07-06

> 实施 lane: phase1-executor
> 实施时间: 2026-07-06
> 关联执行方案: `执行方案.md` §3 Phase 1

## 0. TL;DR — Dependency Mismatch Blocker

执行方案 §3 Phase 1 要求"本仓 runtime 接 real LCM 能力（system `liblcm` C ABI）"，
但本轮探查发现 **dimos 上游库未准备好 3 项依赖**，属于 dependency mismatch blocker：

| # | 缺失项 | 证据 | 处置 |
|---|---|---|---|
| 1 | dimos workspace 无 `nuwax-lcm-sys` / `nuwax-lcm` crate | `ls /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/ | rg -i 'lcm'` 仅命中 `nuwax-codec` / `nuwax-transport` / `nuwax-core` 等，0 个 lcm-sys / lcm wrapper | 选项 B — 不补 |
| 2 | `nuwax-transport` `real-lcm` feature 为空桩 | `nuwax-transport/Cargo.toml:26-31` `real-lcm = []` 仅含注释，无任何 deps | 选项 B — 不补 |
| 3 | `nuwax-codec` fingerprint 是 FNV-1a 64-bit，非 lcm-gen MurmurHash64A | `nuwax-codec/src/lcm.rs:85-94` `fn fnv1a_64(...)` + 注释明确"与 lcm-gen MurmurHash64A 不等价" | 选项 B — 不补 |

**Leader 推荐的选项 B 已采纳**：

- 保留 fake-lcm 默认行为（不破坏现有测试）
- `NUWAX_LCM_BACKEND=real-lcm` 启动期直接 fail-fast，**不 fallback fake-lcm**
- stderr 输出 `LCM_BACKEND_UNAVAILABLE` 关键字
- main.rs exit code = 4
- Phase 2+ 由 dimos 侧补齐上游后，本仓即可在 `LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)` 调用

**未采纳的选项 A**（派 tracer 去 dimos 侧补）：会引入跨仓工作且阻塞主线；
leader 在 brief 中明示 "如果发现 dependency mismatch，**不要阻塞主线**，把
dependency mismatch 报告给 leader；leader 决定要不要派 tracer"。

## 1. 实施细节与踩坑

### 1.1 dimos 库现状全量盘点

```
$ ls /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/ | rg -i lcm
nuwax-codec
nuwax-core
nuwax-transport
(no nuwax-lcm / nuwax-lcm-sys found)

$ rg "MurmurHash64A|murmur" /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/
(0 hits)

$ rg "liblcm|liblcm-dev|pkg-config.*lcm" /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/
(0 hits in source; only 1 doc-comment mention in nuwax-transport/Cargo.toml)
```

**结论**：dimos 库在 P0 阶段完全没有 Rust→system-LCM binding 的实现路径。
本仓"消费 dimos 已验证 LCM 能力"的前提不成立——dimos 库当前只有 fake
backend 的实现。

### 1.2 Phase 0 已经完成的 pre-work

本阶段无需重复 Phase 0 已落盘的内容：

- `_normalize_lcm_backend()` canonical 归一化（`core/settings.py:162-?`）
- `LcmBackendKind::from_env()` 迁移 alias 归一化（`lcm_runtime.rs:387-424`）
- 5 类 channel default（`lcm_runtime.rs:473-477`）：OccupancyGrid /
  Transform / PointCloud2 / NavigationState / FrontierList

### 1.3 fail-fast 实现细节

**修改 1**：`lcm_runtime.rs`

- 新增 `RealLcmBackendUnavailable` 错误类型（`#[derive(Debug, Error)]`，
  thiserror 1.0，message 字面量含 `LCM_BACKEND_UNAVAILABLE` 关键字）
- `spawn_geometry_runtime` 签名改为：
  ```rust
  pub fn spawn_geometry_runtime(
      occupancy_channel: &str,
      tf_channel: &str,
      cloud_map_channel: Option<&str>,
      nav_state_channel: Option<&str>,
      frontiers_channel: Option<&str>,
      pose_channel: Option<&str>,
      odometry_channel: Option<&str>,
      cmd_vel_channel: Option<&str>,
      nav_cancel_channel: Option<&str>,
  ) -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable>
  ```
- `LcmBackendKind::Real` 分支：
  ```rust
  LcmBackendKind::Real => {
      return Err(RealLcmBackendUnavailable);
  }
  ```
  **不再**调用 `LcmTransport::new_fake()`。
- `GeometryRuntimeHandle` 新增 `lcm_backend_available: bool` 字段，
  Fake 分支 = true，Real 分支 = false（在 fake 启动时为 true，
  但 Real 分支已经 return Err，所以 false 路径仅作为未来
  binding 初始化失败的兜底信号）。

**修改 2**：`main.rs`

- 返回类型 `Result<(), Box<dyn Error>>` → `ExitCode`
- spawn 返回 Err 时 eprintln `LCM_BACKEND_UNAVAILABLE: ...` 并 `ExitCode::from(4)`
- 4 类新 typed channel env 解析（`NUWAX_GEOMETRY_RUNTIME_POSE_CHANNEL` /
  `ODOMETRY_CHANNEL` / `CMD_VEL_CHANNEL` / `NAV_CANCEL_CHANNEL`）

**修改 3**：`server.rs`

- `health_check` 用 `bridge.lcm_backend_available` 决定 `healthy`
- `lcm_backend` 字段编码真实状态：`fake-lcm` / `real-lcm-unavailable`

### 1.4 channel registry 补齐

按执行方案 §3 Phase 1 操作 4："至少覆盖 PoseStamped / Odometry / Transform /
PointCloud2 / OccupancyGrid / Twist / NavigationState / FrontierList /
CancelNavigation"。

| 类型 | Phase 0 已有 | Phase 1 改动 |
|---|---|---|
| `nav_msgs/Odometry` | `odom` (`LcmChannelConfig`) | 加 `NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL` runtime env |
| `geometry_msgs/Transform` | `tf` | (无改动) |
| `sensor_msgs/PointCloud2` | `point_cloud` | (无改动) |
| `nav_msgs/OccupancyGrid` | `occupancy` | (无改动) |
| `geometry_msgs/Twist` | `cmd_vel` | 加 `NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL` runtime env |
| `nuwax_msgs/NavigationState` | `state_channel` (`NavigationBackendConfig`) | (无改动) |
| `nuwax_msgs/FrontierList` | `frontier` | (无改动) |
| `geometry_msgs/PoseStamped` | (缺) | **新增** `pose_stamped` field + `NUWAX_GEOMETRY_RUNTIME_POSE_CHANNEL` |
| `nuwax_msgs/CancelNavigation` | (缺) | **新增** `nav_cancel` field + `NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL` |

**本阶段只补 channel registry 定义 + env 解析**，未挂接订阅任务（typed
payload 解析待 Phase 2 统一引入）。订阅处理逻辑保持 Phase 0 的 5 类
（occupancy / tf / odom / cloud_map / nav_state / frontiers）不变，
避免引入未实装的 typed payload 解析风险。

启动期 `LCM channel registry: ...` 与 `LCM msg-type registry: ...` 两行
info 日志现在打印全部 9 类 typed channel 名（HealthCheck 调用方
可 grep 这两行确认 runtime 是否按预期加载了全部 channel）。

### 1.5 golden bytes 证据（按"测试文件不提交"约束写在 feedback 而非 tests/）

typed encode/decode roundtrip（已在 dimos 库 `nuwax-codec/src/lcm.rs`
固化；本仓通过 `cargo tree` 复用）：

```text
geometry_msgs/PoseStamped fingerprint (FNV-1a 64-bit, NOT MurmurHash64A):
  FNV_OFFSET = 0xcbf2_9ce4_8422_2325
  FNV_PRIME  = 0x100_0000_01b3
  hash("geometry_msgs/PoseStamped") = 0x? (deterministic, byte-stable in Rust fake backend)
  → 与 lcm-gen MurmurHash64A 数值不一致（已知 P1 follow-up）

LCM frame header bytes (28 bytes):
  bytes[0..4]  = magic LE u32 = 0x3230_434c → "LC02"
  bytes[4..8]  = reserved = 0
  bytes[8..16] = fingerprint 8-byte BE
  bytes[16..24] = seqno 8-byte LE
  bytes[24..28] = payload_len 4-byte LE
  bytes[28..]  = payload bytes

golden_pose_stamped() payload (68 bytes):
  12 bytes marker = b"PoseStamped!"
  7 × f64 LE = x, y, z, qx, qy, qz, qw
  → full frame = 28 + 68 = 96 bytes
```

Phase 2+ 切换 MurmurHash64A 后，golden bytes fingerprint 段会变；上游
实装完成后由本仓的 `nuwax-codec/src/lcm.rs` 替换 hash 实现 + 在
`feedback_phase2.md` 重新计算并贴出新证据。

## 2. 验收命令实际退出码 + 关键输出

### 2.1 Phase 0 守门（回归通过）

```bash
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过
EXIT=0

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
EXIT=0

$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
EXIT=0

$ ! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py
EXIT=0

$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py
EXIT=0
```

### 2.2 Phase 1 新增 cargo / smoke 验收

```bash
$ source /home/ml/Deeprob/dimos/.nuwax-test-env/env.sh
$ cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets
warning: `nuwax-geometry-runtime` (bin "nuwax-geometry-runtime") generated 2 warnings
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.28s
EXIT=0
(2 warnings 为 video_relay.rs 既有 dead_code,与 Phase 1 无关)

$ cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked
    Finished `test` profile [unoptimized + debuginfo] target(s) in 7.02s
test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
EXIT=0
(本仓 runtime 业务代码无 in-crate unit test,fake LCM 与协议一致性测试在 dimos 库)

$ cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "nuwax-protocol|nuwax-transport|nuwax-codec"
├── nuwax-codec v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-codec)
├── nuwax-protocol v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-protocol)
│   ├── nuwax-codec v0.1.0 (...) (*)
├── nuwax-transport v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-transport)
│   ├── nuwax-codec v0.1.0 (...) (*)
│   ├── nuwax-protocol v0.1.0 (...) (*)
EXIT=0
(3 个 path dep 全部命中,验证本仓在消费 dimos 库)

$ cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"
EXIT=1 (= 0 hits,符合预期)

$ bash scripts/ci/smoke_real_lcm_failfast.sh
==> using binary: /home/ml/Deeprob/dimos/.nuwax-test-env/target/debug/nuwax-geometry-runtime
PASS: real-lcm 启动期 fail-fast(exit=4),stderr 含 LCM_BACKEND_UNAVAILABLE
EXIT=0
```

### 2.3 real-lcm fail-fast 实际 stderr 输出

```bash
$ NUWAX_LCM_BACKEND=real-lcm ./target/debug/nuwax-geometry-runtime
2026-07-06T...Z ERROR nuwax_geometry_runtime: LCM_BACKEND_UNAVAILABLE: \
  NUWAX_LCM_BACKEND=real-lcm 不可用 —— 上游 dimos nuwax-transport/real-lcm \
  feature 仍为空桩 (Rust LCM binding 未实装 + fingerprint 未切 MurmurHash64A)。\
  详见 feedback_phase1.md。
LCM_BACKEND_UNAVAILABLE: real-lcm backend requested but upstream dimos \
  nuwax-transport/real-lcm feature is still a stub. Process will exit.
EXIT=4
```

## 3. 已知限制 / P1 跟进项

1. **fingerprint 仍为 FNV-1a**：上游 `nuwax-codec/src/lcm.rs:85-94` 仍是
   FNV-1a 64-bit；切换 MurmurHash64A 是 P1 follow-up（需要 dimos 库 PR）。
2. **订阅 handler 未挂接**：4 类新 channel 只解析 env + 打印到日志，
   Phase 2 在 `src/typed_codec.rs` 实装后才会接 subscribe 任务。
3. **dimos 库补齐 3 项依赖前，real-lcm 永远 fail-fast**：上游 PR 落地后
   在 `LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)` 即可，
   无需改其他订阅语义。
4. **目标机 aarch64 / Ubuntu 20.04 实机验收**：本阶段 NOT RUN（按执行
   方案 §3.0 已声明；与 Phase 0 一致）。

## 4. 给 Leader 的建议（决策点）

如果后续阶段需要真 real-lcm 闭环（cmd_vel → Go2 实机 / PoseStamped
typed wire-format 等），必须先派人去 dimos 侧补 3 项依赖：

- (1) 新增 `nuwax-lcm-sys` crate + `lcm` rust binding（system liblcm C ABI）
- (2) `nuwax-transport` 把 `real-lcm = ["nuwax-lcm-sys", "nuwax-lcm"]` feature 实装
- (3) `nuwax-codec` 切换 `compute_hash` 到 MurmurHash64A（参考 lcm-gen）

本仓在这 3 项未落地前，最快只能停在 fake-lcm + fail-fast 状态。