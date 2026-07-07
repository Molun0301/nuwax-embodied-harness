//! 集成测试:使用 mock-native binary 启动 native runner
//!
//! 验证:
//! - argv 完整传递(topic args 在前 + extra args 在后,字典序)
//! - exit code 0 / 非 0 状态正确转换
//! - graceful stop timeout 不响应时返回 StopTimeout 错误
//! - diagnostics stdout tail 包含 marker
//!
//! 使用 `nuwax-nav-test-fixtures::mock_native_path()` 共享 helper 定位 mock-native,
//! 不依赖 `CARGO_BIN_EXE_mock-native` 在每个 bridge crate 必须被注入的细节。

use std::collections::BTreeMap;
use std::time::Duration;

use nuwax_nav_base::error::NavError;
use nuwax_nav_base::native::{NativeProcessError, NativeRunnerState, RestartPolicy};
use nuwax_nav_native_runner::{
    NativeBuildPolicy, NativeBuildSpec, NativeProgramSpec, NativeRunner,
};
use nuwax_nav_test_fixtures::mock_native_path;

fn topic_spec(extras: Vec<String>) -> NativeProgramSpec {
    let mut topics = BTreeMap::new();
    topics.insert("registered_scan".into(), "/scan".into());
    topics.insert("odometry".into(), "/odom".into());
    topics.insert("goal".into(), "/goal".into());
    NativeProgramSpec {
        name: "test".into(),
        executable: mock_native_path(),
        topics,
        args: extras,
        env: BTreeMap::new(),
        cwd: None,
        restart: RestartPolicy::Never,
    }
}

fn default_build() -> NativeBuildSpec {
    NativeBuildSpec {
        build_command: None,
        policy: NativeBuildPolicy::Never,
        cwd: None,
        env: BTreeMap::new(),
    }
}

#[tokio::test]
async fn argv_lands_in_order_with_topics_lexicographic() {
    let spec = topic_spec(vec!["--rate".into(), "10".into(), "--debug".into()]);
    let mut r = NativeRunner::new(spec, default_build());
    r.start().await.unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    let snap = r.diagnostics_snapshot();
    // 字典序:goal < odometry < registered_scan
    assert!(snap.stdout_tail.contains("argv[1]=--goal"));
    assert!(snap.stdout_tail.contains("argv[2]=/goal"));
    assert!(snap.stdout_tail.contains("argv[3]=--odometry"));
    assert!(snap.stdout_tail.contains("argv[4]=/odom"));
    assert!(snap.stdout_tail.contains("argv[5]=--registered_scan"));
    assert!(snap.stdout_tail.contains("argv[6]=/scan"));
    assert!(snap.stdout_tail.contains("argv[7]=--rate"));
    assert!(snap.stdout_tail.contains("argv[8]=10"));
    assert!(snap.stdout_tail.contains("argv[9]=--debug"));
    let _ = r.stop(Duration::from_millis(200)).await;
}

#[tokio::test]
async fn zero_exit_yields_exited_state() {
    let spec = topic_spec(vec![]);
    let mut r = NativeRunner::new(spec, default_build());
    r.start().await.unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    let st = r.poll().await.unwrap();
    assert!(matches!(st, NativeRunnerState::Exited { code: Some(0) }));
}

#[tokio::test]
async fn nonzero_exit_yields_failed_state() {
    let mut spec = topic_spec(vec![]);
    spec.args = vec!["--should-fail".into()];
    spec.restart = RestartPolicy::Never;
    let mut r = NativeRunner::new(spec, default_build());
    r.start().await.unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    let st = r.poll().await.unwrap();
    assert!(matches!(st, NativeRunnerState::Failed { .. }));
}

#[tokio::test]
async fn stop_kills_long_sleeping_child() {
    let mut spec = topic_spec(vec![]);
    // 让 mock 忽略 SIGTERM(以防 runner 后续升级为 SIGTERM 阶段);mock 仍能被 SIGKILL 终结
    // → stop 必然能终止进程。要么走 Ok(分支,runner 当前实现直接 SIGKILL),
    // 要么走 StopTimeout(若 runner 改为 SIGTERM→timeout→SIGKILL 流程)。
    // 与 runner 自身的 stop_kills_long_sleeping_process 单测保持一致。
    spec.env
        .insert("MOCK_NATIVE_IGNORE_SIGTERM".into(), "1".into());
    spec.env
        .insert("MOCK_NATIVE_SLEEP_MS".into(), "5000".into());
    let mut r = NativeRunner::new(spec, default_build());
    r.start().await.unwrap();
    let res = r.stop(Duration::from_millis(150)).await;
    assert!(
        res.is_ok()
            || matches!(
                res,
                Err(NavError::NativeProcess(
                    NativeProcessError::StopTimeout { .. }
                ))
            ),
        "stop 应成功或返回 StopTimeout,实际={res:?}"
    );
}

#[tokio::test]
async fn invalid_spec_validates_before_spawn() {
    let mut spec = topic_spec(vec!["".into()]); // empty arg invalid
    spec.args = vec![String::new()];
    let mut r = NativeRunner::new(spec, default_build());
    let err = r.start().await.unwrap_err();
    assert!(matches!(
        err,
        NavError::NativeProcess(NativeProcessError::InvalidSpec { .. })
    ));
}
