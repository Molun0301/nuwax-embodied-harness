//! 可保留的原生子进程生命周期 owner。
//!
//! 本模块在 [`crate::supervised`] 的稳定 PID/PGID、guardian 与有界清理之上，
//! 增加类型化 READY 回调、单调重启代际和 `RestartPolicy` 闭环。调用方必须在
//! READY 回调内校验协议 schema、backend、channel 和本次 `restart_generation`；
//! stdout/stderr 只用于有界诊断，绝不作为就绪信号。

use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::{Duration, Instant};

use nuwax_nav_base::native::RestartPolicy;
use thiserror::Error;

use crate::supervised::{
    spawn_supervised, SupervisedLifecycleError, SupervisedNativeChild, SupervisedNativeCommand,
    SupervisedNativeConfig, SupervisedProcessIdentity, SupervisedReadyError, SupervisedShutdown,
    SupervisedStartError, SupervisedTerminal,
};

/// 当前 planner child 与类型化 READY 共用的重启代际环境变量。
pub const PLANNER_RESTART_GENERATION_ENV: &str = "NUWAX_PLANNER_GENERATION";

/// 类型化 READY 边界返回的 future。
pub type TypedReadyFuture =
    Pin<Box<dyn Future<Output = Result<(), SupervisedReadyError>> + Send + 'static>>;

/// 类型化 READY 边界。
///
/// 每次启动都会传入新的单调 `restart_generation` 和稳定进程身份。实现方必须把
/// 两者纳入 READY identity 校验，从而隔离旧 child 的 READY/result/cancel ack。
pub trait TypedNativeReadiness: Send + Sync {
    /// 等待并校验本次 child 的类型化 READY。
    fn wait_ready(
        &self,
        restart_generation: u64,
        identity: SupervisedProcessIdentity,
    ) -> TypedReadyFuture;
}

impl<F> TypedNativeReadiness for F
where
    F: Fn(u64, SupervisedProcessIdentity) -> TypedReadyFuture + Send + Sync,
{
    fn wait_ready(
        &self,
        restart_generation: u64,
        identity: SupervisedProcessIdentity,
    ) -> TypedReadyFuture {
        self(restart_generation, identity)
    }
}

/// 一次被 READY 接受的原生进程身份。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetainedProcessIdentity {
    /// 单调启动/重启代际，首次成功启动为 1。
    pub restart_generation: u64,
    /// 稳定 PID/PGID/guardian PID。
    pub process: SupervisedProcessIdentity,
}

/// retained owner 的固定启动和清理合同。
#[derive(Clone, Debug)]
pub struct RetainedNativeConfig {
    /// 显式 executable/argv/env/cwd。
    pub command: SupervisedNativeCommand,
    /// 进程组 supervisor 配置。
    pub supervisor: SupervisedNativeConfig,
    /// 每次启动等待类型化 READY 的期限。
    pub startup_timeout: Duration,
    /// 正常 shutdown 的 TERM grace。
    pub shutdown_grace: Duration,
    /// 每次 spawn 都覆盖写入当代值的环境变量名。
    pub restart_generation_env: String,
    /// crash 后重启策略。
    pub restart: RestartPolicy,
}

impl RetainedNativeConfig {
    fn validate(&self) -> Result<(), RetainedNativeError> {
        if self.startup_timeout.is_zero() {
            return Err(RetainedNativeError::InvalidConfig(
                "startup_timeout must be positive".to_string(),
            ));
        }
        if self.restart_generation_env.is_empty()
            || self.restart_generation_env.contains(['=', '\0'])
        {
            return Err(RetainedNativeError::InvalidConfig(
                "restart_generation_env must be a non-empty environment key".to_string(),
            ));
        }
        if Instant::now().checked_add(self.startup_timeout).is_none()
            || Instant::now().checked_add(self.shutdown_grace).is_none()
        {
            return Err(RetainedNativeError::InvalidConfig(
                "lifecycle duration is too large".to_string(),
            ));
        }
        Ok(())
    }
}

/// retained owner 的可观察状态。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RetainedNativeState {
    /// 尚未启动。
    NotStarted,
    /// 类型化 READY 已接受，owner 正持有进程组。
    Running(RetainedProcessIdentity),
    /// crash 后等待下一次有界重启。
    RestartBackoff {
        /// 下一次启动将使用的代际。
        next_generation: u64,
        /// 已消费的重启次数。
        restart_count: u32,
        /// 最近一次终态或启动失败原因。
        reason: String,
    },
    /// child 正常退出并已完成进程组清理。
    Exited {
        /// 退出 child 的代际。
        restart_generation: u64,
        /// 终态证据。
        terminal: SupervisedTerminal,
    },
    /// 重启策略耗尽或代际溢出。
    Failed {
        /// 最近一次启动代际。
        restart_generation: u64,
        /// 终态原因。
        reason: String,
    },
    /// shutdown 已证明进程组清空。
    Shutdown,
}

/// retained owner 错误。
#[derive(Debug, Error)]
pub enum RetainedNativeError {
    /// 配置不合法。
    #[error("invalid retained native lifecycle configuration: {0}")]
    InvalidConfig(String),
    /// owner 已持有运行中的 child。
    #[error("retained native owner is already running")]
    AlreadyRunning,
    /// 启动代际耗尽。
    #[error("native restart generation overflow")]
    GenerationOverflow,
    /// 类型化 READY 前启动失败，且 supervisor 已执行 rollback。
    #[error("native startup failed: {0}")]
    Startup(#[source] Box<SupervisedStartError>),
    /// 运行期观察或清理失败。
    #[error("native lifecycle failed: {0}")]
    Lifecycle(#[from] SupervisedLifecycleError),
}

/// 单一、不可复制的 retained 原生进程组 owner。
pub struct RetainedNativeOwner {
    config: RetainedNativeConfig,
    readiness: Arc<dyn TypedNativeReadiness>,
    child: Option<SupervisedNativeChild>,
    identity: Option<RetainedProcessIdentity>,
    state: RetainedNativeState,
    restart_generation: u64,
    restart_count: u32,
    restart_at: Option<Instant>,
}

impl RetainedNativeOwner {
    /// 构造未启动 owner。
    pub fn new(
        config: RetainedNativeConfig,
        readiness: Arc<dyn TypedNativeReadiness>,
    ) -> Result<Self, RetainedNativeError> {
        config.validate()?;
        Ok(Self {
            config,
            readiness,
            child: None,
            identity: None,
            state: RetainedNativeState::NotStarted,
            restart_generation: 0,
            restart_count: 0,
            restart_at: None,
        })
    }

    /// 当前状态快照。
    #[must_use]
    pub fn state(&self) -> &RetainedNativeState {
        &self.state
    }

    /// 当前已接受 READY 的进程身份。
    #[must_use]
    pub const fn identity(&self) -> Option<RetainedProcessIdentity> {
        self.identity
    }

    /// 当前单调启动代际；首次启动前为 0。
    #[must_use]
    pub const fn restart_generation(&self) -> u64 {
        self.restart_generation
    }

    /// 精确判断异步 READY/result/cancel ack 是否仍属于当前 child。
    #[must_use]
    pub fn accepts(&self, identity: RetainedProcessIdentity) -> bool {
        self.child.is_some() && self.identity == Some(identity)
    }

    /// 精确判断只携带代际的异步 result/cancel ack 是否仍属于当前 child。
    #[must_use]
    pub fn accepts_generation(&self, restart_generation: u64) -> bool {
        self.child.is_some()
            && self
                .identity
                .is_some_and(|identity| identity.restart_generation == restart_generation)
    }

    /// 启动并等待类型化 READY；失败时返回 TERM→KILL→reap rollback 证据。
    pub async fn start(&mut self) -> Result<RetainedProcessIdentity, RetainedNativeError> {
        if self.child.is_some() {
            return Err(RetainedNativeError::AlreadyRunning);
        }
        self.restart_at = None;
        match self.spawn_attempt().await {
            Ok(identity) => Ok(identity),
            Err(error) => {
                self.state = RetainedNativeState::Failed {
                    restart_generation: self.restart_generation,
                    reason: error.to_string(),
                };
                Err(error)
            }
        }
    }

    /// 检测 crash，并按策略推进 backoff/重启。
    ///
    /// 返回 `RestartBackoff` 时 owner 已完成旧进程组清理，但尚未到下一次启动时间。
    /// 返回 `Running` 时新的类型化 READY 已通过。失败/正常退出均不会遗留 child。
    pub async fn reconcile(&mut self) -> Result<RetainedNativeState, RetainedNativeError> {
        if let Some(restart_at) = self.restart_at {
            if Instant::now() < restart_at {
                return Ok(self.state.clone());
            }
            self.restart_at = None;
            match self.spawn_attempt().await {
                Ok(_) => return Ok(self.state.clone()),
                Err(error) => {
                    let reason = error.to_string();
                    if self.schedule_restart(reason.clone())? {
                        return Ok(self.state.clone());
                    }
                    self.state = RetainedNativeState::Failed {
                        restart_generation: self.restart_generation,
                        reason,
                    };
                    return Ok(self.state.clone());
                }
            }
        }

        let Some(child) = self.child.as_mut() else {
            return Ok(self.state.clone());
        };
        if !child.is_finished()? {
            return Ok(self.state.clone());
        }

        let terminal = child.wait().await?;
        self.child = None;
        let finished_identity = self
            .identity
            .take()
            .expect("running retained child always has identity");
        if is_failure(&terminal) {
            let reason = format!("native child crashed: {terminal:?}");
            if self.schedule_restart(reason.clone())? {
                return Ok(self.state.clone());
            }
            self.state = RetainedNativeState::Failed {
                restart_generation: finished_identity.restart_generation,
                reason,
            };
        } else {
            self.state = RetainedNativeState::Exited {
                restart_generation: finished_identity.restart_generation,
                terminal,
            };
        }
        Ok(self.state.clone())
    }

    /// TERM→配置的 grace→KILL→wait/reap，并清除所有待重启状态。
    pub async fn shutdown(&mut self) -> Result<Option<SupervisedShutdown>, RetainedNativeError> {
        self.shutdown_with(self.config.shutdown_grace).await
    }

    /// TERM→调用方 grace→KILL→wait/reap，并清除所有待重启状态。
    pub async fn shutdown_with(
        &mut self,
        grace: Duration,
    ) -> Result<Option<SupervisedShutdown>, RetainedNativeError> {
        self.restart_at = None;
        self.identity = None;
        let shutdown = if let Some(child) = self.child.as_mut() {
            let deadline = Instant::now().checked_add(grace).ok_or_else(|| {
                RetainedNativeError::InvalidConfig("shutdown grace deadline overflow".to_string())
            })?;
            Some(child.terminate(deadline).await?)
        } else {
            None
        };
        self.child = None;
        self.state = RetainedNativeState::Shutdown;
        Ok(shutdown)
    }

    async fn spawn_attempt(&mut self) -> Result<RetainedProcessIdentity, RetainedNativeError> {
        let generation = self
            .restart_generation
            .checked_add(1)
            .ok_or(RetainedNativeError::GenerationOverflow)?;
        self.restart_generation = generation;
        let deadline = Instant::now()
            .checked_add(self.config.startup_timeout)
            .ok_or_else(|| {
                RetainedNativeError::InvalidConfig("startup deadline overflow".to_string())
            })?;
        let readiness = self.readiness.clone();
        let mut command = self.config.command.clone();
        command.env(&self.config.restart_generation_env, generation.to_string());
        let started = spawn_supervised(
            command,
            self.config.supervisor.clone(),
            deadline,
            move |process| readiness.wait_ready(generation, process),
        )
        .await
        .map_err(|error| RetainedNativeError::Startup(Box::new(error)))?;
        let identity = RetainedProcessIdentity {
            restart_generation: generation,
            process: started.child.identity(),
        };
        self.child = Some(started.child);
        self.identity = Some(identity);
        self.state = RetainedNativeState::Running(identity);
        Ok(identity)
    }

    fn schedule_restart(&mut self, reason: String) -> Result<bool, RetainedNativeError> {
        let RestartPolicy::OnFailure {
            max_restarts,
            backoff_millis,
        } = self.config.restart
        else {
            return Ok(false);
        };
        if self.restart_count >= max_restarts {
            return Ok(false);
        }
        self.restart_count = self.restart_count.saturating_add(1);
        let backoff = Duration::from_millis(backoff_millis);
        self.restart_at = Some(Instant::now().checked_add(backoff).ok_or_else(|| {
            RetainedNativeError::InvalidConfig("restart backoff is too large".to_string())
        })?);
        self.state = RetainedNativeState::RestartBackoff {
            next_generation: self
                .restart_generation
                .checked_add(1)
                .ok_or(RetainedNativeError::GenerationOverflow)?,
            restart_count: self.restart_count,
            reason,
        };
        Ok(true)
    }
}

fn is_failure(terminal: &SupervisedTerminal) -> bool {
    matches!(terminal, SupervisedTerminal::Crashed { .. })
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use rustix::io::Errno;
    use rustix::process::{test_kill_process, test_kill_process_group, Pid};
    use std::sync::Mutex;
    use tempfile::tempdir;

    fn shell(script: &str) -> SupervisedNativeCommand {
        let mut command = SupervisedNativeCommand::new("/bin/sh");
        command.arg("-c").arg(script);
        command
    }

    fn config(command: SupervisedNativeCommand, restart: RestartPolicy) -> RetainedNativeConfig {
        RetainedNativeConfig {
            command,
            supervisor: SupervisedNativeConfig {
                stdout_capacity: 1024,
                stderr_capacity: 1024,
                startup_rollback_grace: Duration::from_millis(20),
                kill_reap_timeout: Duration::from_secs(1),
            },
            startup_timeout: Duration::from_millis(200),
            shutdown_grace: Duration::from_millis(100),
            restart_generation_env: PLANNER_RESTART_GENERATION_ENV.to_string(),
            restart,
        }
    }

    fn ready() -> Arc<dyn TypedNativeReadiness> {
        Arc::new(|generation, identity: SupervisedProcessIdentity| {
            Box::pin(async move {
                if generation == 0
                    || identity.pid() == 0
                    || identity.pid() != identity.process_group()
                    || identity.guardian_pid() == 0
                {
                    return Err(SupervisedReadyError::Protocol(
                        "invalid generation or process identity".to_string(),
                    ));
                }
                Ok(())
            }) as TypedReadyFuture
        })
    }

    fn raw_pid(pid: u32) -> Pid {
        Pid::from_raw(i32::try_from(pid).expect("pid_t range")).expect("positive pid")
    }

    fn process_exists(pid: u32) -> bool {
        match test_kill_process(raw_pid(pid)) {
            Ok(()) => true,
            Err(Errno::SRCH) => false,
            Err(error) => panic!("failed to inspect pid {pid}: {error}"),
        }
    }

    fn process_group_exists(process_group: u32) -> bool {
        match test_kill_process_group(raw_pid(process_group)) {
            Ok(()) => true,
            Err(Errno::SRCH) => false,
            Err(error) => panic!("failed to inspect process group {process_group}: {error}"),
        }
    }

    #[tokio::test]
    async fn typed_ready_retains_stable_identity_and_term_reaps_group() {
        let mut owner = RetainedNativeOwner::new(
            config(shell("exec sleep 30"), RestartPolicy::Never),
            ready(),
        )
        .expect("owner");
        let identity = owner.start().await.expect("typed ready");
        assert_eq!(identity.restart_generation, 1);
        assert!(owner.accepts(identity));
        assert!(process_exists(identity.process.pid()));

        let shutdown = owner
            .shutdown_with(Duration::from_millis(100))
            .await
            .expect("shutdown")
            .expect("running child");
        assert!(!shutdown.escalated_to_kill);
        assert!(!process_exists(identity.process.pid()));
        assert!(!process_group_exists(identity.process.process_group()));
        assert!(!owner.accepts(identity));
    }

    #[tokio::test]
    async fn startup_timeout_rolls_back_and_reaps_real_child() {
        let seen = Arc::new(Mutex::new(None));
        let seen_by_ready = seen.clone();
        let readiness: Arc<dyn TypedNativeReadiness> =
            Arc::new(move |_generation, identity: SupervisedProcessIdentity| {
                *seen_by_ready.lock().expect("identity lock") = Some(identity);
                Box::pin(std::future::pending()) as TypedReadyFuture
            });
        let mut lifecycle = config(shell("exec sleep 30"), RestartPolicy::Never);
        // 预留实际子进程握手时间，再验证永不就绪的超时和回收。
        lifecycle.startup_timeout = Duration::from_secs(2);
        let mut owner = RetainedNativeOwner::new(lifecycle, readiness).expect("owner");
        let error = owner.start().await.expect_err("startup must time out");
        assert!(
            matches!(
                &error,
                RetainedNativeError::Startup(error)
                    if matches!(
                        &error.failure,
                        crate::supervised::SupervisedStartFailure::ReadyTimedOut
                    ) && error.rollback_error.is_none()
            ),
            "{error:?}"
        );
        let identity = seen
            .lock()
            .expect("identity lock")
            .expect("readiness invoked");
        assert!(!process_exists(identity.pid()));
        assert!(!process_group_exists(identity.process_group()));
    }

    #[tokio::test]
    async fn crash_restarts_with_monotonic_generation_and_rejects_old_identity() {
        let directory = tempdir().expect("tempdir");
        let marker = directory.path().join("first-start");
        let ready_path = directory.path().join("typed-ready-generation");
        let ready_temporary = directory.path().join("typed-ready-generation.tmp");
        let script = format!(
            "printf '%s' \"${PLANNER_RESTART_GENERATION_ENV}\" > '{}'; mv '{}' '{}'; if [ -e '{}' ]; then exec sleep 30; else touch '{}'; sleep 0.05; exit 7; fi",
            ready_temporary.display(),
            ready_temporary.display(),
            ready_path.display(),
            marker.display(),
            marker.display()
        );
        let generations = Arc::new(Mutex::new(Vec::new()));
        let generations_by_ready = generations.clone();
        let ready_path_by_callback = ready_path.clone();
        let readiness: Arc<dyn TypedNativeReadiness> = Arc::new(
            move |generation, identity: SupervisedProcessIdentity| {
                let generations = generations_by_ready.clone();
                let ready_path = ready_path_by_callback.clone();
                Box::pin(async move {
                    let deadline = Instant::now() + Duration::from_secs(1);
                    loop {
                        if let Ok(encoded) = std::fs::read_to_string(&ready_path) {
                            if let Ok(child_generation) = encoded.trim().parse::<u64>() {
                                if child_generation == generation {
                                    generations.lock().expect("generation lock").push((
                                        generation,
                                        child_generation,
                                        identity,
                                    ));
                                    return Ok(());
                                }
                                if child_generation > generation {
                                    return Err(SupervisedReadyError::Protocol(format!(
                                        "future child generation {child_generation} exceeds owner generation {generation}"
                                    )));
                                }
                            }
                        }
                        if Instant::now() >= deadline {
                            return Err(SupervisedReadyError::Transport(format!(
                                "child did not publish generation {generation}"
                            )));
                        }
                        tokio::time::sleep(Duration::from_millis(2)).await;
                    }
                }) as TypedReadyFuture
            },
        );
        let mut owner = RetainedNativeOwner::new(
            config(
                shell(&script),
                RestartPolicy::OnFailure {
                    max_restarts: 1,
                    backoff_millis: 0,
                },
            ),
            readiness,
        )
        .expect("owner");
        let old = owner.start().await.expect("first ready");

        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let state = owner.reconcile().await.expect("reconcile");
            if matches!(state, RetainedNativeState::Running(identity) if identity.restart_generation == 2)
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "restart did not reach generation 2"
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }

        let current = owner.identity().expect("restarted identity");
        assert_eq!(current.restart_generation, 2);
        assert_ne!(old.process.pid(), current.process.pid());
        assert!(!owner.accepts(old));
        assert!(!owner.accepts_generation(old.restart_generation));
        assert!(owner.accepts(current));
        assert!(owner.accepts_generation(current.restart_generation));
        assert_eq!(
            generations
                .lock()
                .expect("generation lock")
                .iter()
                .map(|(owner_generation, child_generation, _)| {
                    (*owner_generation, *child_generation)
                })
                .collect::<Vec<_>>(),
            vec![(1, 1), (2, 2)]
        );

        owner.shutdown().await.expect("shutdown restarted child");
        assert!(!process_exists(old.process.pid()));
        assert!(!process_exists(current.process.pid()));
        assert!(!process_group_exists(old.process.process_group()));
        assert!(!process_group_exists(current.process.process_group()));
    }

    #[tokio::test]
    async fn term_ignoring_group_escalates_to_kill_and_is_reaped() {
        let delayed_ready: Arc<dyn TypedNativeReadiness> =
            Arc::new(|_generation, _identity: SupervisedProcessIdentity| {
                Box::pin(async {
                    tokio::time::sleep(Duration::from_millis(30)).await;
                    Ok(())
                }) as TypedReadyFuture
            });
        let mut owner = RetainedNativeOwner::new(
            config(
                shell("trap '' TERM; while :; do sleep 1; done"),
                RestartPolicy::Never,
            ),
            delayed_ready,
        )
        .expect("owner");
        let identity = owner.start().await.expect("typed ready");
        let shutdown = owner
            .shutdown_with(Duration::from_millis(10))
            .await
            .expect("shutdown")
            .expect("running child");
        assert!(shutdown.escalated_to_kill);
        assert!(!process_exists(identity.process.pid()));
        assert!(!process_group_exists(identity.process.process_group()));
    }
}
