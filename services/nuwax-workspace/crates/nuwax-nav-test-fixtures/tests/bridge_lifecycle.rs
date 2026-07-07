//! 4 个 bridge crate 的 mock-native lifecycle / failed_exit / full_smoke 集成测试
//!
//! 设计要点
//! =========
//!
//! 历史背景:这 4 类测试原本写在 4 个 bridge crate 各自的 `tests/spec_argv.rs`
//! (或 `integration_smoke.rs`)里。它们使用 `mock_native_path()` 解析 mock-native
//! binary 路径,而该函数在没有预构建 workspace `target/` 时会 panic — 也就是
//! "干净 `CARGO_TARGET_DIR` 下 `cargo test -p <bridge> --tests` 失败"的根因。
//!
//! 本文件把所有"启动 mock-native binary"的集成测试统一搬到 `nuwax-nav-test-fixtures`
//! 自身的 integration test 目录,**不**修改任何测试断言(lifecycle Start→Running→Exited
//! / failed_exit / integration_smoke full_lifecycle 的状态断言逐字保留)。
//!
//! 关键机制
//! ---------
//!
//! - `nuwax-nav-test-fixtures` 在其 `Cargo.toml` 内声明了 `[[bin]] mock-native`。
//!   当 `cargo test --tests` 编译本文件所属的 test target 时,cargo 会自动把
//!   `mock-native` 作为构建依赖,并把精确路径注入到环境变量
//!   `CARGO_BIN_EXE_mock-native` 中。
//! - `env!("CARGO_BIN_EXE_mock-native")` 在编译期读取该变量,直接拿到 binary 绝对路径,
//!   不依赖 workspace `target/` 目录预构建,也不依赖 `mock_native_path()` 的回退路径。
//! - 因此本文件能在任意 `CARGO_TARGET_DIR`(包括完全干净的临时目录)下直接通过。
//!
//! 4 个 bridge crate 内的 `lifecycle_start_stop` / `failed_exit` / `integration_smoke`
//! 等测试**已删除**(参见各 bridge `tests/spec_argv.rs` 的删除记录),以避免它们
//! 在 bridge crate 自身上下文里回退到不存在的 `target/debug/mock-native` 而 panic。
//! argv 渲染断言、golden fixture 对比、in-process tf_helper 测试等不依赖 binary 启动
//! 的部分仍保留在 bridge crate 内。
//!
//! 关联
//! -----
//!
//! - `crates/nuwax-nav-test-fixtures/src/bin/mock_native.rs`:mock-native 行为定义
//! - `crates/nuwax-nav-test-fixtures/src/lib.rs::mock_native_path()`:旧 helper
//!   (桥测试中不再使用,但保留以供其他需要 workspace-target 的场景)

#![allow(clippy::needless_raw_string_hashes)]

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Duration;

use nuwax_nav_base::native::NativeRunnerState;
use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeRunner};

// 编译期注入 — cargo 会自动构建 nuwax-nav-test-fixtures 的 [[bin]] mock-native
// 并把绝对路径写进该环境变量,本 crate 直接编译期读出,不再走运行时解析。
fn mock_native_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_mock-native"))
}

// ============================================================================
// FAR Planner bridge
// ============================================================================

mod far_planner {
    use super::*;
    use nuwax_nav_far_planner_bridge::{
        FarPlannerBridge, FarPlannerBridgeConfig, FarPlannerTopics,
    };

    fn cfg_with_env(extra_env: BTreeMap<String, String>) -> FarPlannerBridgeConfig {
        FarPlannerBridgeConfig {
            executable: mock_native_bin(),
            restart_policy: nuwax_nav_far_planner_bridge::FarPlannerRestartPolicy::Never,
            env: extra_env,
            ..FarPlannerBridgeConfig::default()
        }
    }

    fn default_topics() -> FarPlannerTopics {
        FarPlannerTopics {
            odometry: Some("/odom".to_string()),
            goal: Some("/goal".to_string()),
            way_point: Some("/way_point".to_string()),
            goal_path: Some("/goal_path".to_string()),
            registered_scan: Some("/scan".to_string()),
            terrain_map: Some("/terrain".to_string()),
            terrain_map_ext: Some("/terrain_ext".to_string()),
            stop_movement: Some("/stop_movement".to_string()),
            graph_nodes: Some("/graph_nodes".to_string()),
            graph_edges: Some("/graph_edges".to_string()),
            contour_polygons: Some("/contour_polygons".to_string()),
            nav_boundary: Some("/nav_boundary".to_string()),
        }
    }

    /// 远迁移自 `nuwax-nav-far-planner-bridge/tests/integration_smoke.rs`。
    /// 断言集合与原实现完全一致。
    #[tokio::test]
    async fn far_planner_smoke_full_lifecycle() {
        let topics = default_topics();

        let bridge = FarPlannerBridge::new(cfg_with_env(BTreeMap::new()), &topics);
        let argv = bridge.argv();
        assert!(!argv.is_empty());
        let topic_pairs: Vec<(String, String)> = bridge
            .spec
            .topics
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();

        let spec = bridge.spec.clone();
        let mut runner = NativeRunner::new(
            spec,
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner.start().await.expect("start mock-native");

        let expected_topic_count = topic_pairs.len();
        assert!(
            expected_topic_count >= 6,
            "FAR Planner 至少 6 个 topic,得到 {expected_topic_count}"
        );

        // 让 mock-native 完成 250ms sleep
        let mut env = BTreeMap::new();
        env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "250".to_string());
        env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "0".to_string());
        // 上述 env 注入到下一次 start,需要新 runner
        let mut env_cfg = cfg_with_env(env);
        env_cfg.executable = mock_native_bin();
        let bridge2 = FarPlannerBridge::new(env_cfg, &topics);
        let spec2 = bridge2.spec.clone();
        let mut runner2 = NativeRunner::new(
            spec2,
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner2.start().await.expect("restart with sleep env");
        tokio::time::sleep(Duration::from_millis(400)).await;
        let st_after = runner2.poll().await.expect("poll");
        assert!(
            matches!(st_after, NativeRunnerState::Exited { code: Some(0) }),
            "mock-native 应正常退出,得到 {st_after:?}"
        );

        // diagnostics:stdout_tail 必须包含 mock-native-ok + argv 中至少一个 topic
        let snap = runner2.diagnostics_snapshot();
        let tail = &snap.stdout_tail;
        assert!(
            tail.contains("mock-native-ok"),
            "stdout 应包含 mock-native-ok,得到: {tail}"
        );
        let (_, first_topic_value) = topic_pairs.first().expect("at least one topic");
        assert!(
            tail.contains(first_topic_value),
            "stdout 应包含 topic `{first_topic_value}`,得到: {tail}"
        );
        assert!(
            tail.contains("--robot_dim"),
            "stdout 应包含 --robot_dim(CLI 历史名),得到: {tail}"
        );

        // stop 幂等 + 第二次 start
        let stop_res = runner2.stop(Duration::from_millis(100)).await;
        assert!(
            stop_res.is_ok(),
            "stop after exit 应幂等 ok,得到 {stop_res:?}"
        );
        let mut runner3 = NativeRunner::new(
            bridge2.spec.clone(),
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner3.start().await.expect("restart should succeed");
        tokio::time::sleep(Duration::from_millis(80)).await;
        let s_running = runner3.poll().await.expect("poll");
        assert!(
            matches!(
                s_running,
                NativeRunnerState::Running { .. } | NativeRunnerState::Exited { .. }
            ),
            "second run state expected Running or Exited, got {s_running:?}"
        );
        let _ = runner3.stop(Duration::from_millis(50)).await;
        // 静默第一个 runner(只 start 没等到退出)
        let _ = runner.stop(Duration::from_millis(10)).await;
    }

    /// 远迁移自 `nuwax-nav-far-planner-bridge/tests/spec_argv.rs::far_planner_bridge_lifecycle_start_stop`。
    #[tokio::test]
    async fn far_planner_bridge_lifecycle_start_stop() {
        let mut env = BTreeMap::new();
        env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "200".to_string());
        env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "0".to_string());

        let bridge = FarPlannerBridge::new(
            cfg_with_env(env),
            &FarPlannerTopics {
                odometry: Some("/odom".to_string()),
                way_point: Some("/way_point".to_string()),
                ..Default::default()
            },
        );
        let spec = bridge.spec.clone();
        let mut runner = NativeRunner::new(
            spec,
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner.start().await.expect("mock-native should start");

        let state_immediate = runner.poll().await.expect("poll ok");
        assert!(
            matches!(state_immediate, NativeRunnerState::Running { .. }),
            "expected Running shortly after start, got {state_immediate:?}"
        );

        tokio::time::sleep(Duration::from_millis(400)).await;
        let state_after = runner.poll().await.expect("poll ok");
        assert!(
            matches!(state_after, NativeRunnerState::Exited { code: Some(0) }),
            "expected Exited(0) after sleep, got {state_after:?}"
        );

        let snap = runner.diagnostics_snapshot();
        assert!(
            snap.stdout_tail.contains("mock-native-ok"),
            "expected mock-native-ok marker in stdout tail, got: {}",
            snap.stdout_tail
        );

        let stop_res = runner.stop(Duration::from_millis(100)).await;
        assert!(stop_res.is_ok(), "stop after exit should be ok");
    }

    /// 远迁移自 `nuwax-nav-far-planner-bridge/tests/spec_argv.rs::far_planner_failed_exit_yields_failed_state`。
    #[test]
    fn far_planner_failed_exit_yields_failed_state() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();

        rt.block_on(async {
            let mut env = BTreeMap::new();
            env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "10".to_string());
            env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "7".to_string());

            let bridge = FarPlannerBridge::new(
                cfg_with_env(env),
                &FarPlannerTopics {
                    odometry: Some("/odom".to_string()),
                    ..Default::default()
                },
            );
            let spec = bridge.spec.clone();
            let mut runner = NativeRunner::new(
                spec,
                NativeBuildSpec {
                    build_command: None,
                    policy: NativeBuildPolicy::Never,
                    cwd: None,
                    env: BTreeMap::new(),
                },
            );
            runner.start().await.expect("start");
            tokio::time::sleep(Duration::from_millis(80)).await;
            let st = runner.poll().await.expect("poll");
            assert!(
                matches!(
                    st,
                    NativeRunnerState::Exited { code: Some(7) } | NativeRunnerState::Failed { .. }
                ),
                "expected exit 7 or failed, got {st:?}"
            );
        });
    }
}

// ============================================================================
// Local Planner bridge
// ============================================================================

mod local_planner {
    use super::*;
    use nuwax_nav_local_planner_bridge::{
        LocalPlannerBridge, LocalPlannerBridgeConfig, LocalPlannerTopics,
    };

    fn cfg_with_env(extra_env: BTreeMap<String, String>) -> LocalPlannerBridgeConfig {
        LocalPlannerBridgeConfig {
            executable: mock_native_bin(),
            env: extra_env,
            restart_policy: nuwax_nav_local_planner_bridge::LocalPlannerRestartPolicy::Never,
            ..LocalPlannerBridgeConfig::default()
        }
    }

    /// 远迁移自 `nuwax-nav-local-planner-bridge/tests/spec_argv.rs::local_planner_bridge_lifecycle_start_stop`。
    #[tokio::test]
    async fn local_planner_bridge_lifecycle_start_stop() {
        let mut env = BTreeMap::new();
        env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "200".to_string());
        env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "0".to_string());
        let bridge = LocalPlannerBridge::new(
            cfg_with_env(env),
            &LocalPlannerTopics {
                odometry: Some("/odom".to_string()),
                path: Some("/path".to_string()),
                ..Default::default()
            },
        );

        let spec = bridge.spec.clone();
        let mut runner = NativeRunner::new(
            spec,
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner.start().await.expect("start");
        let s_immediate = runner.poll().await.expect("poll");
        assert!(
            matches!(s_immediate, NativeRunnerState::Running { .. }),
            "expected Running, got {s_immediate:?}"
        );

        tokio::time::sleep(Duration::from_millis(400)).await;
        let s_after = runner.poll().await.expect("poll");
        assert!(
            matches!(s_after, NativeRunnerState::Exited { code: Some(0) }),
            "expected Exited(0), got {s_after:?}"
        );
    }

    /// 远迁移自 `nuwax-nav-local-planner-bridge/tests/spec_argv.rs::local_planner_failed_exit`。
    #[test]
    fn local_planner_failed_exit() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async {
            let mut env = BTreeMap::new();
            env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "10".to_string());
            env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "5".to_string());
            let bridge = LocalPlannerBridge::new(
                cfg_with_env(env),
                &LocalPlannerTopics {
                    odometry: Some("/odom".to_string()),
                    ..Default::default()
                },
            );
            let spec = bridge.spec.clone();
            let mut runner = NativeRunner::new(
                spec,
                NativeBuildSpec {
                    build_command: None,
                    policy: NativeBuildPolicy::Never,
                    cwd: None,
                    env: BTreeMap::new(),
                },
            );
            runner.start().await.expect("start");
            tokio::time::sleep(Duration::from_millis(80)).await;
            let st = runner.poll().await.expect("poll");
            assert!(
                matches!(
                    st,
                    NativeRunnerState::Exited { code: Some(5) } | NativeRunnerState::Failed { .. }
                ),
                "expected exit 5 or failed, got {st:?}"
            );
        });
    }
}

// ============================================================================
// Path Follower bridge
// ============================================================================

mod path_follower {
    use super::*;
    use nuwax_nav_path_follower_bridge::{
        PathFollowerBridge, PathFollowerBridgeConfig, PathFollowerTopics,
    };

    fn cfg_with_env(extra_env: BTreeMap<String, String>) -> PathFollowerBridgeConfig {
        PathFollowerBridgeConfig {
            executable: mock_native_bin(),
            env: extra_env,
            restart_policy: nuwax_nav_path_follower_bridge::PathFollowerRestartPolicy::Never,
            ..PathFollowerBridgeConfig::default()
        }
    }

    /// 远迁移自 `nuwax-nav-path-follower-bridge/tests/spec_argv.rs::path_follower_bridge_lifecycle_start_stop`。
    #[tokio::test]
    async fn path_follower_bridge_lifecycle_start_stop() {
        let mut env = BTreeMap::new();
        env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "200".to_string());
        env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "0".to_string());
        let bridge = PathFollowerBridge::new(
            cfg_with_env(env),
            &PathFollowerTopics {
                odometry: Some("/odom".to_string()),
                path: Some("/path".to_string()),
                ..Default::default()
            },
        );

        let spec = bridge.spec.clone();
        let mut runner = NativeRunner::new(
            spec,
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner.start().await.expect("start");
        let s_immediate = runner.poll().await.expect("poll");
        assert!(
            matches!(s_immediate, NativeRunnerState::Running { .. }),
            "expected Running, got {s_immediate:?}"
        );
        tokio::time::sleep(Duration::from_millis(400)).await;
        let s_after = runner.poll().await.expect("poll");
        assert!(
            matches!(s_after, NativeRunnerState::Exited { code: Some(0) }),
            "expected Exited(0), got {s_after:?}"
        );
    }

    /// 远迁移自 `nuwax-nav-path-follower-bridge/tests/spec_argv.rs::path_follower_failed_exit`。
    #[test]
    fn path_follower_failed_exit() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async {
            let mut env = BTreeMap::new();
            env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "10".to_string());
            env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "9".to_string());
            let bridge = PathFollowerBridge::new(
                cfg_with_env(env),
                &PathFollowerTopics {
                    odometry: Some("/odom".to_string()),
                    ..Default::default()
                },
            );
            let spec = bridge.spec.clone();
            let mut runner = NativeRunner::new(
                spec,
                NativeBuildSpec {
                    build_command: None,
                    policy: NativeBuildPolicy::Never,
                    cwd: None,
                    env: BTreeMap::new(),
                },
            );
            runner.start().await.expect("start");
            tokio::time::sleep(Duration::from_millis(80)).await;
            let st = runner.poll().await.expect("poll");
            assert!(
                matches!(
                    st,
                    NativeRunnerState::Exited { code: Some(9) } | NativeRunnerState::Failed { .. }
                ),
                "expected exit 9 or failed, got {st:?}"
            );
        });
    }
}

// ============================================================================
// PGO bridge
// ============================================================================

mod pgo {
    use super::*;
    use nuwax_nav_pgo_bridge::{PgoBridge, PgoBridgeConfig, PgoTopics};

    fn cfg_with_env(extra_env: BTreeMap<String, String>) -> PgoBridgeConfig {
        PgoBridgeConfig {
            executable: mock_native_bin(),
            env: extra_env,
            restart_policy: nuwax_nav_pgo_bridge::PgoRestartPolicy::Never,
            // PGO 默认 cfg.cwd = Some("cpp")(相对路径,运行时无法解析;
            // mock-native 不是 C++ binary,不需要固定 cwd)
            cwd: None,
            ..PgoBridgeConfig::default()
        }
    }

    /// 远迁移自 `nuwax-nav-pgo-bridge/tests/spec_argv.rs::pgo_bridge_lifecycle_start_stop`。
    #[tokio::test]
    async fn pgo_bridge_lifecycle_start_stop() {
        let mut env = BTreeMap::new();
        env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "200".to_string());
        env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "0".to_string());
        let bridge = PgoBridge::new(
            cfg_with_env(env),
            &PgoTopics {
                odometry: Some("/odom".to_string()),
                pgo_tf: Some("/pgo_tf".to_string()),
                ..Default::default()
            },
        );

        let spec = bridge.spec.clone();
        let mut runner = NativeRunner::new(
            spec,
            NativeBuildSpec {
                build_command: None,
                policy: NativeBuildPolicy::Never,
                cwd: None,
                env: BTreeMap::new(),
            },
        );
        runner.start().await.expect("start");
        let s_immediate = runner.poll().await.expect("poll");
        assert!(
            matches!(s_immediate, NativeRunnerState::Running { .. }),
            "expected Running, got {s_immediate:?}"
        );
        tokio::time::sleep(Duration::from_millis(400)).await;
        let s_after = runner.poll().await.expect("poll");
        assert!(
            matches!(s_after, NativeRunnerState::Exited { code: Some(0) }),
            "expected Exited(0), got {s_after:?}"
        );
    }

    /// 远迁移自 `nuwax-nav-pgo-bridge/tests/spec_argv.rs::pgo_bridge_failed_exit`。
    #[test]
    fn pgo_bridge_failed_exit() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async {
            let mut env = BTreeMap::new();
            env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), "10".to_string());
            env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), "13".to_string());
            let bridge = PgoBridge::new(
                cfg_with_env(env),
                &PgoTopics {
                    odometry: Some("/odom".to_string()),
                    ..Default::default()
                },
            );
            let spec = bridge.spec.clone();
            let mut runner = NativeRunner::new(
                spec,
                NativeBuildSpec {
                    build_command: None,
                    policy: NativeBuildPolicy::Never,
                    cwd: None,
                    env: BTreeMap::new(),
                },
            );
            runner.start().await.expect("start");
            tokio::time::sleep(Duration::from_millis(80)).await;
            let st = runner.poll().await.expect("poll");
            assert!(
                matches!(
                    st,
                    NativeRunnerState::Exited { code: Some(13) } | NativeRunnerState::Failed { .. }
                ),
                "expected exit 13 or failed, got {st:?}"
            );
        });
    }
}
