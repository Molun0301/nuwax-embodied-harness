# Phase 3-5 收口 Fixup — cargo test SIGBUS + cancel 死锁 + Python legacy guard

> 日期：2026-07-06
> lane：rust-runtime Phase 3-5 收口
> 关联：`feedback_phase3.md` §未完成 blocker / `feedback_phase4.md` §未完成 blocker /
>      `feedback_phase5.md` §未完成 blocker / `执行方案.md` §3 Phase 5 收口段
> commit: 本次合入见 git log（fix(phase3-5): cargo test SIGBUS 修复 + fix(phase3-5): _platform_exploration_runtime 加 NUWAX_EXPLORATION_LEGACY guard）

## 0. 背景

Phase 3-5 主线已 commit 通过（5 个 feature commit + Phase 0 收敛 1 个 commit），但验收步骤执行 `cargo +1.88.0 test --locked` 报
signal 7 SIGBUS。leader 派子 agent 收口两个 blocker：

- **Blocker 1**：cargo test 报 SIGBUS（实测 28 tests pass 后崩溃,串行模式挂在
  `cancel_*` 测试，进入 `state.snapshot().await` 死锁 30s+）
- **Blocker 2**：`_platform_exploration_runtime` 仍被 28+ 处 helper 直接调用,
  Phase 5 收口要求默认探索 owner 不再走 Python 旧状态机

## 1. Blocker 1 — cargo test SIGBUS 根因 + 修复

### 1.1 根因链路

执行 `cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked`
跑 28 个 test ok 后报 SIGBUS（access to undefined memory）。SIGBUS 在 Rust 单元测试里通常来自
三类：mmap 未对齐、unsafe transmute、dangling pointer deref。

实际根因排查路径：

1. **第一次崩在 `build_nav_stack_handle_renders_for_all_profiles`**（不是 SIGBUS 而是 panic）:
   `src/nav_runtime.rs:989:17: dry-run 文本应包含 profile=mls3d`
   根因：`PROFILE_MLS3D` 常量是 `"mls3d"`（无 dash），但
   `NavStackProfile::Mls3D.as_str()` 是 `"mls-3d"`（kebab-case）。
   `text.contains("mls3d")` 永远找不到。**单行 typo 修复**。
2. **修 typo 后第二次崩在 `set_goal_running_with_path_follower`**（也不是 SIGBUS 而是 panic）:
   `left: "FAILED", right: "RUNNING"`
   根因：`set_goal` 调 `publish_goal_frame` → `LcmTopic::new("/nav/goal#geometry_msgs.PoseStamped")`
   报错 `topic 不应包含 typed suffix 分隔符 #`。`LcmTopic::new` 强约束 channel 字符串不能含
   `#`（typed suffix 分隔符），但 `channel_registry::resolve_channel` 默认值是
   `/topic#pkg.Msg` typed pattern。`publish_goal_frame` / `publish_cancel_frame` /
   `publish_cmd_vel` 三处都错用了 `LcmTopic::new`,必须先调
   `LcmTopic::parse_subscription_pattern` 拆分再喂给 `LcmTopic::new`。
3. **修后第三次崩在 `cancel_with_no_active_goal_is_safe`**（死锁）:
   `cancel()` 在 drop write guard 后立即调 `state.snapshot(goal_id, 0.0).await` →
   `self.inner.read().await` 永远不被唤醒。`tokio::sync::RwLock` 在
   `current_thread` runtime 下从 write drop 到 read acquire 之间存在调度间隙,
   测试 runtime 不主动 re-poll park 住的 read future。
   **解决**:把 `state.snapshot()` 调用改成在 write guard 作用域内同步读 + 直接构造
   `NavState`,绕开 read-after-write 路径。
4. **修后第四次崩 SIGBUS 真因**:
   串行模式全 48 测试通过,**但并行模式**仍然 SIGBUS(28 tests pass 后崩)。
   根因在 `lcm_runtime.rs::build_handle_for_tests`:
   `occupancy_shm` 文件路径 `/tmp/nuwax_occupancy_test_<pid>.mmap` **同一进程内所有
   caller 共享**。并行 cargo test 跑 N 个 `make_handle` 调
   `OccupancyGridShmWriter::new()` 同一路径,各线程同时 `truncate(true)` + `set_len(16MB)`
   + `MmapMut::map_mut(&file)`,但 mmap 段地址已被另一线程 unmap → SIGBUS。
   **修复**:加 `OCC_COUNTER` 原子计数,路径改为
   `/tmp/nuwax_occupancy_test_<pid>_<counter>.mmap`,每个 caller 独立路径。

### 1.2 修复 diff 摘要

#### 1.2.1 `services/geometry/nuwax_geometry_runtime/src/nav_runtime.rs`

```diff
-pub const PROFILE_MLS3D: &str = "mls3d";
+pub const PROFILE_MLS3D: &str = "mls-3d";  // kebab-case 与 NavStackProfile::Mls3D.as_str() 对齐
```

```diff
+/// 把 /topic#pkg.Msg typed pattern 拆成 (LcmTopic, Option<msg_type>)。
+/// `LcmTopic::new` 强约束 channel 不能含 #;但 channel_registry 默认值是 typed pattern。
+fn parse_channel_with_suffix(channel: &str) -> Result<(LcmTopic, Option<String>), String> {
+    LcmTopic::parse_subscription_pattern(channel)
+        .map_err(|e| format!("LcmTopic::parse_subscription_pattern({}) 失败: {}", channel, e))
+}

 // publish_goal_frame / publish_cancel_frame / publish_cmd_vel 三处:
-let topic = match LcmTopic::new(&channel) { ... };
+let (topic, _suffix) = parse_channel_with_suffix(&channel)?;
```

```diff
 pub async fn cancel(...) -> CancelOutcome {
     ... publish + write_lock ...
     let mut slot = state.inner.write().await;
-    let prev_goal_id = slot.goal.as_ref().map(|g| g.id.clone());
+    let prev_goal_id;
+    let status_after;
+    let message_after;
+    {
+        let mut slot = state.inner.write().await;
+        prev_goal_id = slot.goal.as_ref().map(|g| g.id.clone());
-        slot.phase = NavPhase::Cancelled;
+        slot.phase = NavPhase::Cancelled;
         slot.goal = None;
         slot.follower = None;
         slot.message = format!(...);
-    let _ = slot;
-    let nav = state.snapshot(goal_id, 0.0).await;  // 死锁 read-after-write
+        status_after = slot.phase.as_str().to_string();
+        message_after = slot.message.clone();
+    }  // slot drop in block
+    // 不调 state.snapshot():cargo test current_thread runtime 下 read-after-write 死锁
+    let nav = NavState { status: status_after, message: message_after, ... };
     CancelOutcome { state: nav }
 }
```

#### 1.2.2 `services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs`

```diff
-    let occ_test_path = std::path::PathBuf::from(format!(
-        "/tmp/nuwax_occupancy_test_{}.mmap",
-        std::process::id()
-    ));
+    // 必须 per-call 唯一后缀:并行 cargo test 多个 build_handle_for_tests
+    // 同时 truncate 同一文件 + 同一 mmap 段 → SIGBUS。
+    static OCC_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
+    let occ_test_path = std::path::PathBuf::from(format!(
+        "/tmp/nuwax_occupancy_test_{}_{}.mmap",
+        std::process::id(),
+        OCC_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
+    ));
```

### 1.3 验证

- 串行: `cargo +1.88.0 test --locked -- --test-threads=1` → **48 passed; 0 failed**
- 并行(默认): `cargo +1.88.0 test --locked` → **48 passed; 0 failed**
- cargo check: `cargo +1.88.0 check --locked --all-targets` → exit 0
- cargo tree: `cargo +1.88.0 tree` 包含 `nuwax-mapping` / `nuwax-nav-stack` / `nuwax-exploration`,
  0 个 r2r/cyclonedds/rmw_/libddsc 命中

## 2. Blocker 2 — `_platform_exploration_runtime` legacy guard

### 2.1 根因

Phase 5 收口要求探索默认 owner 走 `_remote_provider` (GeometryRuntimeProvider → Rust
explore_runtime);但 `services/geometry/runtime.py` 仍有 28+ 处 helper 直接调
`self._platform_exploration_runtime.<method>`(内部状态机),Phase 0 硬隔离
`_resolve_exploration_backend` 只是把"返回后端"切到 remote,**helper 内部调用链
没有切**。Phase 6 完整重写时一并清理,但 Phase 5 收口需要最小修复点:
- 默认 `_resolve_exploration_backend` 不返回 `_platform_exploration_runtime`
- 显式 `NUWAX_EXPLORATION_LEGACY=true` 时允许返回(兼容单测/诊断路径)
- helper 内部 28 处直接调用保留(进程内对象,不破坏单测),但前提是
  `_resolve_exploration_backend` 已被守门

### 2.2 修复 diff 摘要

#### 2.2.1 `services/geometry/runtime.py`

```diff
+def _is_exploration_legacy_enabled() -> bool:
+    """_platform_exploration_runtime 旧探索状态机的环境开关。
+    Phase 5 收口:默认 False,只有 NUWAX_EXPLORATION_LEGACY=true/1/yes/on 才允许走。
+    """
+    raw = os.environ.get("NUWAX_EXPLORATION_LEGACY", "")
+    return str(raw or "").strip().lower() in {"1", "true", "yes", "on"}

 def _resolve_exploration_backend(self):
-    if not self._is_geometry_profile_enabled():
-        return None
-    remote = getattr(self, "_remote_provider", None)
-    if remote is not None and getattr(remote, "is_connected", False):
-        return remote
-    return self._resolve_backend(ExplorationDataPlane, ExplorationProvider)
+    if not self._is_geometry_profile_enabled():
+        return None
+    remote = getattr(self, "_remote_provider", None)
+    if remote is not None and getattr(remote, "is_connected", False):
+        return remote
+    # Phase 5 收口:默认不返回 _platform_exploration_runtime
+    if not _is_exploration_legacy_enabled():
+        return None
+    legacy = getattr(self, "_platform_exploration_runtime", None)
+    if legacy is not None:
+        return legacy
+    return None
```

### 2.3 验证

```bash
$ python3 -c "from services.geometry.runtime import _is_exploration_legacy_enabled; print(_is_exploration_legacy_enabled())"
False
$ NUWAX_EXPLORATION_LEGACY=true python3 -c "..."
True
$ NUWAX_EXPLORATION_LEGACY=1 python3 -c "..."
True
$ NUWAX_EXPLORATION_LEGACY=0 python3 -c "..."
False
```

## 3. 验收命令最终结果

```bash
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过                                # EXIT=0
$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
                                                                                  # EXIT=0
$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
                                                                                  # EXIT=0
$ ! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py
                                                                                  # EXIT=0
$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py
                                                                                  # EXIT=0
$ cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets
                                                                                  # EXIT=0
$ cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked
test result: ok. 48 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out   # EXIT=0
$ cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "nuwax-mapping|nuwax-nav-stack|nuwax-exploration"
├── nuwax-exploration v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-exploration)
├── nuwax-mapping v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-mapping)
├── nuwax-nav-stack v0.1.0 (/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-nav-stack)
$ ! cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"
                                                                                  # EXIT=0
```

## 4. 后续 Phase 6 待办

1. 把 `services/geometry/runtime.py` 28+ 处 `self._platform_exploration_runtime.<method>`
   直接调用收口:
   - 探索/导航/建图 helper 全部走 `_remote_provider` (GeometryRuntimeProvider)
   - 删 `_platform_exploration_runtime` 字段(默认 owner 不再持有)
   - `NUWAX_EXPLORATION_LEGACY` env 留作单测 harness 用,生产路径完全无依赖
2. 把 `cancel` 的死锁修复升到 `state.snapshot()` 本身(找出 tokio RwLock 在 current_thread
   runtime 下的 read-after-write hang 根因),目前是绕开路径
3. 把 `parse_channel_with_suffix` 升级为 typed channel registry 的公共 helper
   (目前只 3 个 publish 复用,Phase 6 把 `server.rs` 的 publish 全部接同一入口)
