# Phase 1 todolist — 2026-07-06

> 6 步操作清单 + 实际完成状态。
> 状态：✅ 全部完成。**重要：因 dimos 上游无 `nuwax-lcm-sys`/`nuwax-lcm` 且 `real-lcm` feature 为空桩，本阶段无法实装 real-lcm**——已记录 dependency mismatch blocker 并选择 **选项 B**（fake-lcm 默认 + `NUWAX_LCM_BACKEND=real-lcm` 启动期 fail-fast 返回 `LCM_BACKEND_UNAVAILABLE`，不 fallback）。

## 6 步操作清单

- [x] **Step 1**：固定 dimos path deps 来源
  - 来源：`/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace`
  - 已用 path deps 锁定 4 个 crate：`nuwax-protocol`、`nuwax-transport`（`default-features=false`）、`nuwax-codec`、`nuwax-tf`
  - 验证方式：`rg "path =" services/geometry/nuwax_geometry_runtime/Cargo.toml`

- [x] **Step 2**：dependency mismatch 探测
  - dimos workspace 53 个 crate，**不存在** `nuwax-lcm-sys` 或 `nuwax-lcm`
  - `nuwax-transport/src/lib.rs` 只提供 `LcmTransport::new_fake()` 与 generic `new(backend)`；**没有** `new_real(url)`
  - `nuwax-transport/Cargo.toml:29` `real-lcm = []` 是空 feature stub
  - `nuwax-codec/src/lcm.rs:85-94` fingerprint = FNV-1a 64-bit，**非** MurmurHash64A
  - workspace 内 0 处 `MurmurHash64A`/`murmur` 引用
  - 结论 = dependency mismatch blocker，按 leader 推荐 **选项 B**：保留 fake-lcm 默认，real-lcm 请求启动期 fail-fast 不 fallback

- [x] **Step 3**：`LcmBackendKind::Real` 分支改 fail-fast
  - `lcm_runtime.rs` `spawn_geometry_runtime` 中 `Real` 分支不再 `LcmTransport::new_fake()` 兜底
  - 改为返回 `transport_backend_name = "real-lcm-unavailable"` + 在 HealthCheck 输出 `LCM_BACKEND_UNAVAILABLE`
  - 启动期仍 spawn 异步任务以满足 server 监听，但 `transport.publish/subscribe` 立即返回 error
  - 真实失败由 `server.rs` 的 HealthCheck / Start 路径转换成 status = `RUNTIME_NOT_READY`
  - **不再**有 `LcmBackendKind::Fake` 出现在 `Real` 失败路径

- [x] **Step 4**：channel registry 补齐 4 类
  - 已存在 5 类（来自 Phase 0 默认）：`OccupancyGrid` / `Transform` / `PointCloud2` / `NavigationState` / `FrontierList`
  - 补 4 类：
    - `PoseStamped` → `NUWAX_GEOMETRY_RUNTIME_POSE_CHANNEL`，默认 `/pose_stamped#geometry_msgs.PoseStamped`
    - `Odometry` → `NUWAX_GEOMETRY_RUNTIME_ODOMETRY_CHANNEL`，默认 `/odom#nav_msgs.Odometry`（与 `/odom` 重叠，但 typed 后缀不同）
    - `Twist` → `NUWAX_GEOMETRY_RUNTIME_CMD_VEL_CHANNEL`，默认 `/cmd_vel#geometry_msgs.Twist`
    - `CancelNavigation` → `NUWAX_GEOMETRY_RUNTIME_NAV_CANCEL_CHANNEL`，默认 `/nav/cancel#nuwax_msgs.CancelNavigation`
  - `main.rs` 解析上述 4 个 env key，并传入 `spawn_geometry_runtime` 新增形参
  - `lcm_runtime.rs` 增加 4 个 typed LcmMessageType 常量
  - 本阶段**只**补 channel registry 定义 + env 解析；订阅逻辑保持 Phase 0 的 5 类不变（避免引入未实装的 typed payload 解析风险）

- [x] **Step 5**：backend 命名统一验证
  - `LcmBackendKind::from_env` 已只在 `real-lcm` / `fake-lcm` canonical 之间归一化（Phase 0 已落）
  - 验证 `Real` 失败路径不再出现 `LcmTransport::new_fake()`：`rg "new_fake\(\)" services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs` 仅在 Fake 分支
  - 验证 HealthCheck 输出 backend = `real-lcm-unavailable` / `fake-lcm`（canonical）
  - 验证 `backend_name` 字段仅取 `fake-lcm` / `real-lcm-unavailable`，无裸 `real` / `fake`

- [x] **Step 6**：golden bytes 验收
  - 本阶段不写 `tests/` 下文件（按全局 feedback 约束）
  - 在 feedback_phase1.md 贴 typed encode/decode roundtrip 文字证据（已知 FNV-1a fingerprint，P1 切 MurmurHash64A 后会变）

## 验收命令实际退出码

- `bash scripts/ci/no_ros2_residual.sh`: ✅ exit 0
- `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts`: ✅ exit 0
- `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh`: ✅ exit 0
- `! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py`: ✅ exit 0
- `! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py`: ✅ exit 0
- `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets`: ✅ exit 0
- `cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked`: ✅ exit 0
- `cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "nuwax-protocol|nuwax-transport|nuwax-codec"`: ✅ 非空
- `cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"`: ✅ 0 命中
- `NUWAX_LCM_BACKEND=real-lcm bash scripts/ci/smoke_real_lcm_failfast.sh`（新写）：✅ exit 非 0，输出 `LCM_BACKEND_UNAVAILABLE`

## 实际 commit 批次

1. `feat(phase1): real-lcm backend fail-fast + 移除 fake fallback`
2. `feat(phase1): channel registry 补齐 PoseStamped/Odometry/Twist/CancelNavigation 4 类 typed channel`
3. `docs(phase1): feedback + todolist + 实施记录 + dependency mismatch 报告`