//! `integration_mock` 集成测试
//!
//! 用 `mock-native` binary 跑完整 lifecycle,验证
//! `BridgeSupervisor` 包装下的 start / poll / stop / diagnostics / restart / fatal
//! 行为(方案 §5.5 mock-native 测试模板)。
//!
//! 缺资源(`mock-native` binary 未生成)时所有测试 eprintln SKIP + return,
//! 不阻塞 CI。

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Duration;

use nuwax_explore_terrain_bridge::{
    BridgeSupervisor, BridgeSupervisorState, TerrainAnalysisBridge, TerrainAnalysisBridgeConfig,
    TerrainAnalysisTopics, TerrainBridgeError,
};
use nuwax_nav_base::native::RestartPolicy;
use nuwax_nav_native_runner::NativeBuildPolicy;

/// 定位 `mock-native` binary
fn locate_mock_native() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("CARGO_BIN_EXE_mock-native") {
        return Some(PathBuf::from(p));
    }
    let exe = std::env::current_exe().ok()?;
    let deps = exe.parent()?;
    let profile = deps.parent()?;
    let candidate = profile.join("mock-native");
    if candidate.exists() {
        Some(candidate)
    } else {
        None
    }
}

fn skip_if_no_mock() -> Option<PathBuf> {
    match locate_mock_native() {
        Some(p) => Some(p),
        None => {
            eprintln!(
                "SKIP integration_mock: mock-native binary not found; \
                 run `cargo build -p nuwax-nav-test-fixtures --bin mock-native` first"
            );
            None
        }
    }
}

fn mock_spec(
    exe: &std::path::Path,
    exit_code: &str,
    sleep_ms: &str,
    extra_env: &[(&str, &str)],
) -> (
    nuwax_nav_native_runner::NativeProgramSpec,
    nuwax_nav_native_runner::NativeBuildSpec,
) {
    let mut env = BTreeMap::new();
    env.insert("MOCK_NATIVE_EXIT_CODE".to_string(), exit_code.to_string());
    if !sleep_ms.is_empty() {
        env.insert("MOCK_NATIVE_SLEEP_MS".to_string(), sleep_ms.to_string());
    }
    for (k, v) in extra_env {
        env.insert((*k).to_string(), (*v).to_string());
    }
    let spec = nuwax_nav_native_runner::NativeProgramSpec {
        name: "terrain_bridge_integration_mock".to_string(),
        executable: exe.to_path_buf(),
        topics: Default::default(),
        args: vec![],
        env,
        cwd: None,
        restart: RestartPolicy::Never,
    };
    let build = nuwax_nav_native_runner::NativeBuildSpec {
        build_command: None,
        policy: NativeBuildPolicy::Never,
        cwd: None,
        env: Default::default(),
    };
    (spec, build)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn start_poll_stop_lifecycle() {
    let Some(exe) = skip_if_no_mock() else {
        return;
    };
    let cfg = TerrainAnalysisBridgeConfig {
        executable: exe,
        restart_policy: RestartPolicy::Never,
        ..TerrainAnalysisBridgeConfig::default()
    };
    let bridge = TerrainAnalysisBridge::new(
        cfg,
        &TerrainAnalysisTopics {
            registered_scan: Some("/registered_scan".into()),
            odometry: Some("/odometry".into()),
            terrain_map: Some("/terrain_map".into()),
        },
    )
    .expect("bridge new");
    let mut bridge = bridge;
    bridge.start().await.expect("start");
    tokio::time::sleep(Duration::from_millis(150)).await;
    let _state = bridge.poll().await.expect("poll");
    let diags = bridge.diagnostics();
    assert!(diags.runner.last_exit_code.is_some());
    bridge.stop(Duration::from_millis(50)).await.ok();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn restart_then_fatal_through_supervisor() {
    let Some(exe) = skip_if_no_mock() else {
        return;
    };
    let (mut spec, build) = mock_spec(&exe, "1", "0", &[]);
    spec.restart = RestartPolicy::OnFailure {
        max_restarts: 1,
        backoff_millis: 10,
    };
    let mut sup = BridgeSupervisor::new(spec, build);
    sup.start().await.expect("start");
    tokio::time::sleep(Duration::from_millis(100)).await;
    let state_after = sup.poll_with_restart().await.expect("poll_with_restart");
    assert!(matches!(state_after, BridgeSupervisorState::Fatal { .. }));
    let diags = sup.diagnostics();
    assert!(diags.restart_count() >= 1);
    assert!(diags.last_error.is_some());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_does_not_restart() {
    let Some(exe) = skip_if_no_mock() else {
        return;
    };
    let (mut spec, build) = mock_spec(&exe, "0", "5000", &[]);
    spec.restart = RestartPolicy::OnFailure {
        max_restarts: 3,
        backoff_millis: 10,
    };
    let mut sup = BridgeSupervisor::new(spec, build);
    sup.start().await.expect("start");
    sup.stop(Duration::from_millis(200)).await.ok();
    let diags = sup.diagnostics();
    assert_eq!(diags.restart_count(), 0);
    assert_eq!(sup.state(), &BridgeSupervisorState::Stopped);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn spawn_error_diagnostics_contains_source() {
    let cfg = TerrainAnalysisBridgeConfig {
        executable: PathBuf::from("/this/binary/should/not/exist/anywhere"),
        restart_policy: RestartPolicy::Never,
        ..TerrainAnalysisBridgeConfig::default()
    };
    let mut bridge = TerrainAnalysisBridge::new(
        cfg,
        &TerrainAnalysisTopics {
            registered_scan: Some("/registered_scan".into()),
            odometry: Some("/odometry".into()),
            terrain_map: Some("/terrain_map".into()),
        },
    )
    .expect("bridge new");
    let err = bridge.start().await.unwrap_err();
    let msg = format!("{err:?}");
    assert!(
        matches!(err, TerrainBridgeError::NativeProcess(_)) || msg.contains("NativeProcess"),
        "expected NativeProcess error, got: {msg}"
    );
    let diags = bridge.diagnostics();
    assert!(diags.last_error.is_some());
    assert!(matches!(
        bridge.state(),
        BridgeSupervisorState::Fatal { .. }
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn reset_fatal_allows_new_restart_budget() {
    let Some(exe) = skip_if_no_mock() else {
        return;
    };
    let (mut spec, build) = mock_spec(&exe, "1", "0", &[]);
    spec.restart = RestartPolicy::OnFailure {
        max_restarts: 1,
        backoff_millis: 5,
    };
    let mut sup = BridgeSupervisor::new(spec, build);
    sup.start().await.expect("start");
    tokio::time::sleep(Duration::from_millis(50)).await;
    let _ = sup.poll_with_restart().await;
    assert!(sup.state().is_fatal());
    sup.reset_fatal().expect("reset_fatal");
    assert_eq!(sup.state(), &BridgeSupervisorState::Stopped);
    let _ = sup.start().await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    let _ = sup.poll_with_restart().await;
}
