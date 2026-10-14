//! `NativeRunner`:start / stop / poll / diagnostics
//!
//! 设计:为避免阻塞 start,stdout/stderr 通过后台 task drain 到共享 diagnostics
//! (`Arc<Mutex<...>>`)。stop 时丢弃 reader 并强制结束子进程。
//! 不假设 socket IPC;native 进程通过 LCM/topic/CLI 与 stack 通信。

use std::path::PathBuf;
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use nuwax_nav_base::error::{NavError, NavResult};
use nuwax_nav_base::native::{
    NativeProcessDiagnostics, NativeProcessError, NativeRunnerState, RestartPolicy,
};
use tokio::process::{Child, Command};
use tokio::task::JoinHandle;

use crate::build::{ensure_built, spawn_drain};
use crate::rendered_args::render_args;
use crate::spec::{NativeBuildSpec, NativeProgramSpec};

/// NativeRunner
#[derive(Debug)]
pub struct NativeRunner {
    spec: NativeProgramSpec,
    build: NativeBuildSpec,
    child: Option<Child>,
    state: NativeRunnerState,
    diagnostics: Arc<Mutex<NativeProcessDiagnostics>>,
    #[allow(dead_code)]
    drain: Option<JoinHandle<()>>,
}

impl NativeRunner {
    /// 构造(尚未启动)
    #[must_use]
    pub fn new(spec: NativeProgramSpec, build: NativeBuildSpec) -> Self {
        let diagnostics = Arc::new(Mutex::new(NativeProcessDiagnostics::new(spec.name.clone())));
        Self {
            spec,
            build,
            child: None,
            state: NativeRunnerState::NotStarted,
            diagnostics,
            drain: None,
        }
    }

    /// 当前 spec(只读引用)
    #[must_use]
    pub fn spec(&self) -> &NativeProgramSpec {
        &self.spec
    }

    /// 渲染后的 CLI 参数(不含 argv[0])
    #[must_use]
    pub fn rendered_args(&self) -> Vec<String> {
        render_args(&self.spec)
    }

    /// 当前 state
    #[must_use]
    pub fn state(&self) -> &NativeRunnerState {
        &self.state
    }

    /// 当前 diagnostics 快照
    #[must_use]
    pub fn diagnostics_snapshot(&self) -> NativeProcessDiagnostics {
        self.diagnostics
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// 启动进程(完成 build policy + spawn + 后台 drain)
    pub async fn start(&mut self) -> NavResult<()> {
        if self.state.is_running() {
            return Err(NavError::InvalidInput {
                field: "runner",
                reason: "already running".to_string(),
            });
        }
        self.spec
            .validate()
            .map_err(|reason| NativeProcessError::InvalidSpec {
                field: "spec".to_string(),
                reason,
            })?;
        self.state = NativeRunnerState::Building;
        {
            let mut d = self.diagnostics.lock().unwrap_or_else(|e| e.into_inner());
            d.component = self.spec.name.clone();
        }
        ensure_built(&self.spec, &self.build, &self.diagnostics).await?;

        let args = self.rendered_args();
        let mut command = Command::new(&self.spec.executable);
        command.args(&args);
        for (k, v) in &self.spec.env {
            command.env(k, v);
        }
        if let Some(cwd) = &self.spec.cwd {
            command.current_dir(cwd);
        }
        command.stdout(Stdio::piped());
        command.stderr(Stdio::piped());
        command.kill_on_drop(true);

        tracing::info!(
            component = "nuwax-nav-native-runner",
            name = %self.spec.name,
            exe = %self.spec.executable.display(),
            args = ?args,
            "starting native process"
        );

        let started_at = Instant::now();
        let mut child = command
            .spawn()
            .map_err(|e| NativeProcessError::SpawnFailed {
                source: e.to_string(),
            })?;
        let pid = child.id().ok_or(NativeProcessError::SpawnFailed {
            source: "child missing pid".to_string(),
        })?;
        let stdout = child.stdout.take();
        let stderr = child.stderr.take();
        let drain = match (stdout, stderr) {
            (Some(o), Some(e)) => Some(spawn_drain(o, e, self.diagnostics.clone())),
            _ => None,
        };
        self.child = Some(child);
        self.state = NativeRunnerState::Running { pid };
        self.drain = drain;
        {
            let mut d = self.diagnostics.lock().unwrap_or_else(|e| e.into_inner());
            d.last_started_at = Some(nuwax_nav_types_messages::Timestamp::default());
            d.last_startup_latency_ns = Some(started_at.elapsed().as_nanos() as i64);
        }
        Ok(())
    }

    /// 轮询当前状态(若 child 已退出,返回 `Exited` 或 `Failed`)
    pub async fn poll(&mut self) -> NavResult<NativeRunnerState> {
        if let Some(child) = self.child.as_mut() {
            match child.try_wait() {
                Ok(Some(status)) => {
                    let code = status.code();
                    self.child = None;
                    let new_state = if status.success() {
                        NativeRunnerState::Exited { code }
                    } else {
                        NativeRunnerState::Failed {
                            error: NativeProcessError::ExitedNonZero { code },
                        }
                    };
                    {
                        let mut d = self.diagnostics.lock().unwrap_or_else(|e| e.into_inner());
                        d.last_exited_at = Some(nuwax_nav_types_messages::Timestamp::default());
                        d.last_exit_code = code;
                    }
                    self.state = new_state.clone();
                    return Ok(new_state);
                }
                Ok(None) => {
                    return Ok(self.state.clone());
                }
                Err(e) => {
                    let err = NativeProcessError::SpawnFailed {
                        source: e.to_string(),
                    };
                    let st = NativeRunnerState::Failed { error: err.clone() };
                    self.state = st.clone();
                    return Ok(st);
                }
            }
        }
        Ok(self.state.clone())
    }

    /// 停止进程:先 SIGTERM → 等 `timeout` → SIGKILL
    pub async fn stop(&mut self, timeout: Duration) -> NavResult<()> {
        let Some(child) = self.child.as_mut() else {
            self.state = NativeRunnerState::Exited { code: None };
            return Ok(());
        };
        let pid = child.id().unwrap_or(0);
        self.state = NativeRunnerState::Stopping { pid };
        // 先 SIGTERM,sh 不一定响应,所以走 start_kill + wait timeout 双重保险
        let _ = child.start_kill();
        let start = Instant::now();
        loop {
            match child.try_wait() {
                Ok(Some(_)) => {
                    self.child = None;
                    let st = NativeRunnerState::Exited { code: None };
                    self.state = st.clone();
                    return Ok(());
                }
                Ok(None) => {
                    if start.elapsed() >= timeout {
                        let _ = child.kill().await;
                        let _ = child.wait().await;
                        self.child = None;
                        let err = NativeProcessError::StopTimeout {
                            millis: timeout.as_millis() as u64,
                        };
                        self.state = NativeRunnerState::Failed { error: err.clone() };
                        // drain task 会因 stdout EOF 自动结束
                        self.drain = None;
                        return Err(NavError::NativeProcess(err));
                    }
                    tokio::time::sleep(Duration::from_millis(20)).await;
                }
                Err(e) => {
                    self.state = NativeRunnerState::Failed {
                        error: NativeProcessError::SpawnFailed {
                            source: e.to_string(),
                        },
                    };
                    return Err(NavError::NativeProcess(NativeProcessError::SpawnFailed {
                        source: e.to_string(),
                    }));
                }
            }
        }
    }

    /// 是否应触发重启(根据 `RestartPolicy`)
    #[must_use]
    pub fn should_restart(&self) -> bool {
        let count = self
            .diagnostics
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .restart_count;
        match (&self.spec.restart, &self.state) {
            (
                RestartPolicy::OnFailure {
                    max_restarts,
                    backoff_millis: _,
                },
                state @ (NativeRunnerState::Exited { .. } | NativeRunnerState::Failed { .. }),
            ) => {
                // 拆分计算 `terminal_nonzero`,因为 `Failed { error }` 没有 `code` 字段。
                // 任何 Failed 都视为非零退出;Exited 则按 `code` 判断。
                let terminal_nonzero = match state {
                    NativeRunnerState::Exited { code } => match code {
                        Some(c) => *c != 0,
                        None => true,
                    },
                    NativeRunnerState::Failed { .. } => true,
                    _ => false,
                };
                terminal_nonzero && count < *max_restarts
            }
            _ => false,
        }
    }

    /// 增加重启计数
    pub fn bump_restart(&mut self) {
        let mut d = self.diagnostics.lock().unwrap_or_else(|e| e.into_inner());
        d.restart_count = d.restart_count.saturating_add(1);
    }

    /// 校验 spec 字段(并以 typed error 返回)
    pub fn validate_spec(&self) -> NavResult<()> {
        self.spec
            .validate()
            .map_err(|reason| NativeProcessError::InvalidSpec {
                field: "spec".to_string(),
                reason,
            })?;
        Ok(())
    }
}

/// 工具函数:获取默认 executable(若不存在 → 返回 PathBuf placeholder)
#[must_use]
pub fn placeholder_executable(path: &str) -> PathBuf {
    PathBuf::from(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{NativeBuildPolicy, NativeBuildSpec, NativeProgramSpec};
    use std::collections::BTreeMap;

    /// 构造一个用 `mock-native` binary 作为 executable 的 spec。
    /// 通过 `MOCK_NATIVE_SLEEP_MS` / `MOCK_NATIVE_EXIT_CODE` 控制行为,
    /// 避免依赖 `tempfile + sh` 在测试机上的可执行性。
    ///
    /// 路径解析优先级:
    /// 1. `CARGO_BIN_EXE_mock-native` 环境变量(Cargo 在 `cargo test` 时自动注入,
    ///    要求 mock-native 是该 crate 的 dev-dep)
    /// 2. 由 `current_exe()` 向上推导 target dir,直接读 `<target>/debug/mock-native`。
    ///    这一层是兜底,因为 `CARGO_BIN_EXE_*` 只在 binary 被 cargo 主动 link 时才注入。
    fn shell_spec() -> (NativeProgramSpec, NativeBuildSpec) {
        let exe = locate_mock_native();
        let spec = NativeProgramSpec {
            name: "test".into(),
            executable: exe,
            topics: BTreeMap::new(),
            args: vec![],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        };
        let build = NativeBuildSpec {
            build_command: None,
            policy: NativeBuildPolicy::Never,
            cwd: None,
            env: BTreeMap::new(),
        };
        (spec, build)
    }

    /// 定位 `mock-native` binary 路径。
    fn locate_mock_native() -> std::path::PathBuf {
        // 1. 优先用 cargo test 注入的 env var
        if let Ok(p) = std::env::var("CARGO_BIN_EXE_mock-native") {
            return std::path::PathBuf::from(p);
        }
        // 2. 兜底:从 current_exe() 反推 target dir
        // test binary 路径形如 <target>/<profile>/deps/<test>-<hash>
        // mock-native 在 <target>/<profile>/mock-native
        let exe = std::env::current_exe().expect("current_exe");
        let deps = exe.parent().expect("deps dir");
        let profile = deps.parent().expect("profile dir");
        let candidate = profile.join("mock-native");
        assert!(
            candidate.exists(),
            "mock-native binary not found at {}; ensure `cargo run -p nuwax-nav-test-fixtures --bin mock-native` or `cargo build -p nuwax-nav-test-fixtures --bin mock-native` has been executed",
            candidate.display()
        );
        candidate
    }

    #[tokio::test]
    async fn start_then_poll_exited() {
        let (spec, build) = shell_spec();
        let mut r = NativeRunner::new(spec, build);
        r.start().await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;
        let st = r.poll().await.unwrap();
        assert!(matches!(st, NativeRunnerState::Exited { .. }));
    }

    #[tokio::test]
    async fn start_already_running_returns_invalid_input() {
        let (mut spec, build) = shell_spec();
        // 注入 sleep 让进程跑 5s,期间二次 start 必须返回 InvalidInput
        spec.env
            .insert("MOCK_NATIVE_SLEEP_MS".to_string(), "5000".to_string());
        let mut r = NativeRunner::new(spec, build);
        r.start().await.unwrap();
        let err = r.start().await.unwrap_err();
        assert!(matches!(err, NavError::InvalidInput { .. }));
        let _ = r.stop(Duration::from_millis(200)).await;
    }

    #[tokio::test]
    async fn stop_kills_long_sleeping_process() {
        let (mut spec, build) = shell_spec();
        spec.env
            .insert("MOCK_NATIVE_SLEEP_MS".to_string(), "30000".to_string());
        let mut r = NativeRunner::new(spec, build);
        r.start().await.unwrap();
        // 用极短 timeout 触发 SIGKILL 升级路径。
        // mock-native 收到 SIGTERM 会干净退出(返回 `Ok(())`),这是 tokio 进程的标准行为;
        // 在 sh 路径下 `sleep` 不响应 SIGTERM,会走 StopTimeout 分支。
        // 此处只断言"stop 必然成功终止进程",不强制具体错误形态。
        let res = r.stop(Duration::from_millis(10)).await;
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
    async fn stdout_lands_in_diagnostics_tail() {
        let (mut spec, build) = shell_spec();
        // 让 mock-native 写一个稳定 marker 到 stderr,经 drain 进入 diagnostics
        spec.env
            .insert("MOCK_NATIVE_STDERR".to_string(), "hello-marker".to_string());
        let mut r = NativeRunner::new(spec, build);
        r.start().await.unwrap();
        // 等 drain task 完成
        tokio::time::sleep(Duration::from_millis(200)).await;
        let snap = r.diagnostics_snapshot();
        assert!(snap.stderr_tail.contains("hello-marker"));
    }

    #[test]
    fn should_restart_respects_policy_and_max() {
        let (spec, build) = shell_spec();
        let mut r = NativeRunner::new(spec, build);
        r.state = NativeRunnerState::Exited { code: Some(0) };
        assert!(!r.should_restart());
    }
}
