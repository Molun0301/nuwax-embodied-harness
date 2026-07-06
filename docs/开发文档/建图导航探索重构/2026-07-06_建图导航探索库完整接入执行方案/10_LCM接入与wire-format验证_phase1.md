# Phase 1 LCM 接入与 wire-format 验证实施记录 — 2026-07-06

> 日期: 2026-07-06
> 实施 lane: phase1-executor
> 关联文档: 执行方案.md §3 Phase 1, todolist_phase1.md, feedback_phase1.md

## 1. 范围

本阶段把本仓 `nuwax-geometry-runtime` 接到 dimos 库已验证的 LCM 能力。
经探查确认 dimos 库当前 **未实装** real-lcm binding（`nuwax-transport/real-lcm` feature 为空桩、fingerprint 仍是 FNV-1a、workspace 无 `nuwax-lcm-sys` crate），属于 dependency mismatch blocker。按 leader 推荐，本阶段采纳 **选项 B**：

- 保留 fake-lcm 默认行为
- `NUWAX_LCM_BACKEND=real-lcm` 启动期直接 fail-fast，**不 fallback fake-lcm**
- Phase 2+ 由 dimos 侧补齐上游后，本仓即可在 `LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)`

## 2. 改动文件

```
M  services/geometry/nuwax_geometry_runtime/Cargo.toml               (确认 path deps, 无新增)
M  services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs       (Real branch fail-fast, 4 类 channel)
M  services/geometry/nuwax_geometry_runtime/src/main.rs              (spawn 返回 Result, 4 类 env, ExitCode)
M  services/geometry/nuwax_geometry_runtime/src/server.rs            (health_check 用 lcm_backend_available)
M  contracts/geometry_runtime.py                                     (LcmChannelConfig 新增 2 字段)
M  core/settings.py                                                  (新增 2 个 _cfg_str 调用)
M  core/config_registry.py                                           (新增 6 个 ZN_NAMES)
M  .env.example                                                      (新增 6 个 env key)
A  scripts/ci/smoke_real_lcm_failfast.sh                             (新写 fail-fast 守门)
A  docs/.../todolist_phase1.md                                       (本阶段操作清单)
A  docs/.../feedback_phase1.md                                       (dependency mismatch + 实施细节)
A  docs/.../10_LCM接入与wire-format验证_phase1.md                    (本文件)
```

## 3. LCM backend 选型矩阵

| env / 状态 | `fake-lcm` (default) | `real-lcm` |
|---|---|---|
| 启动期行为 | 正常启动 + fake backend in-memory | **fail-fast**: `eprintln!("LCM_BACKEND_UNAVAILABLE: ...")` + `ExitCode::from(4)` |
| HealthCheck.healthy | `true` | N/A（进程已退出） |
| HealthCheck.lcm_backend | `"fake-lcm"` | N/A |
| 适用场景 | CI / 开发机 / 无 system LCM 的目标机 | 暂不可用，需 dimos 库补 3 项依赖 |
| 是否可 fallback 到 fake | (本身就是 fake) | **绝对不可**（执行方案 §3 Phase 1 操作 6 强制） |

`fake` / `real` 仍为迁移 alias（Phase 0 `_normalize_lcm_backend()` 处理）；canonical 名为 `fake-lcm` / `real-lcm`。

## 4. Channel registry 现状

启动期打印 2 行日志便于运维 / healthcheck 确认：

```text
LCM channel registry: occupancy=/map/occupancy tf=/tf cloud_map=/go2/lidar/points \
  nav_state=/nav/state frontiers=/explore/frontiers pose=/pose_stamped \
  odom=/odom cmd_vel=/cmd_vel nav_cancel=/nav/cancel
LCM msg-type registry: occupancy=nav_msgs/OccupancyGrid tf=geometry_msgs/Transform \
  cloud_map=sensor_msgs/PointCloud2 nav_state=nuwax_msgs/NavigationState \
  frontiers=nuwax_msgs/FrontierList pose=geometry_msgs/PoseStamped \
  odom=nav_msgs/Odometry cmd_vel=geometry_msgs/Twist nav_cancel=nuwax_msgs/CancelNavigation
```

| msg type FQN | channel | status |
|---|---|---|
| `nav_msgs/Odometry` | `/odom` | subscribed (Phase 0) |
| `geometry_msgs/Transform` | `/tf` | subscribed (Phase 0) |
| `sensor_msgs/PointCloud2` | `/go2/lidar/points` | subscribed (Phase 0) |
| `nav_msgs/OccupancyGrid` | `/map/occupancy` | subscribed (Phase 0) |
| `nuwax_msgs/NavigationState` | `/nav/state` | subscribed (Phase 0) |
| `nuwax_msgs/FrontierList` | `/explore/frontiers` | subscribed (Phase 0) |
| `geometry_msgs/PoseStamped` | `/pose_stamped` | registry only (Phase 1) |
| `geometry_msgs/Twist` | `/cmd_vel` | registry only (Phase 1) |
| `nuwax_msgs/CancelNavigation` | `/nav/cancel` | registry only (Phase 1) |

registry-only 类别本阶段不挂 subscribe handler（typed payload 解析待 Phase 2 在
`src/typed_codec.rs` 统一引入；本阶段盲目挂接会引入未实装的 decode 风险）。

## 5. wire-format / fingerprint 现状

| 字段 | 当前值 | 上游 lcm-gen 期望 | 处置 |
|---|---|---|---|
| magic u32 LE | `0x3230_434c` = `"LC02"` | `"LC02"` | OK |
| magic 字段宽度 | 8 bytes (高 4 magic + 低 4 0) | 8 bytes | OK |
| fingerprint 宽度 | 8 bytes BE | 8 bytes BE | OK |
| fingerprint 算法 | FNV-1a 64-bit | MurmurHash64A | **不一致** — P1 follow-up |
| seqno 宽度 | 8 bytes LE | 8 bytes LE | OK |
| payload_len 宽度 | 4 bytes LE | 4 bytes LE | OK |
| payload 字节序 | 小端（f64 / f32 / u32 / u16） | 小端 | OK |

fingerprint 切换 MurmurHash64A 是 P1 follow-up（需要在 dimos 库 PR）。切换
后 `golden_pose_stamped` / `golden_odometry` / `golden_point_cloud2` 的
fingerprint 段会变；本仓无需改任何代码，由 dimos 库的
`nuwax-codec/src/lcm.rs::compute_lcm_hash` 替换即可。

## 6. 验收命令实际退出码

详见 `feedback_phase1.md` §2。全部 9 条命令 exit 0 / exit 1（= 0 hits）。

## 7. 依赖来源（path deps）

```toml
nuwax-protocol = { path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-protocol" }
nuwax-transport = { path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-transport", default-features = false }
nuwax-codec    = { path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-codec" }
nuwax-tf       = { path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-tf" }
```

来源已固定；后续如需升级 dimos 库的 git rev，把 `path = "..."` 替换为 `git = "..."` + `rev = "..."` 即可（执行方案 §3 Phase 1 操作 1 允许）。

## 8. P1 follow-up

1. dimos 库补 `nuwax-lcm-sys` + `nuwax-lcm` + `real-lcm` feature 实装
2. dimos 库 `nuwax-codec::compute_lcm_hash` 切换 MurmurHash64A
3. 本仓 `LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)` 调用
4. 本仓 `src/typed_codec.rs` 引入 typed payload encode/decode
5. 4 类 registry-only channel 挂 subscribe handler
6. 实机 LCM 闭环 + golden bytes Python/C++ 互通对账（需要 system `lcm`）