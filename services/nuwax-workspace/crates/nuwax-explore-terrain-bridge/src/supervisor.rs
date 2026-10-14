//! Bridge-local `BridgeSupervisor` 薄层
//!
//! 包装 `nuwax-nav-native-runner::NativeRunner`,为 Terrain Analysis bridge 提供
//! `Restarting` / `Fatal` 状态机与 backoff(方案 §5.5)。
//!
//! 位置约束(方案 §5.5):首版不新增第六个 public crate。TARE / Terrain Analysis
//! 两个 bridge-local `BridgeSupervisor` 接口、默认策略、测试模板必须一致;TARE 与
//! Terrain 两份 supervisor 代码允许局部复制,但测试命名、状态机迁移与诊断字段
//! 必须保持一致,避免一个 bridge 修复 lifecycle bug 而另一个漂移。

use std::sync::{Arc, Mutex};
use std::time::Duration;

use nuwax_nav_base::error::{NavError, NavResult};
use nuwax_nav_base::native::{
    NativeProcessDiagnostics, NativeProcessError, NativeRunnerState, RestartPolicy,
};
use nuwax_nav_native_runner::{NativeBuildSpec, NativeProgramSpec, NativeRunner};

/// 默认 restart 策略(方案 §5.5)
pub const DEFAULT_RESTART_POLICY: RestartPolicy = RestartPolicy::OnFailure {
    max_restarts: 3,
    backoff_millis: 1_000,
};

/// Bridge supervisor 状态
///
/// Terrain bridge-local supervisor 状态;`NativeRunnerState` 当前没有
/// `Restarting` / `Fatal`,因此本类型是 bridge 薄层自有的状态机。
#[derive(Debug, Clone, PartialEq)]
pub enum BridgeSupervisorState {
    /// 未启动或已干净退出
    Stopped,
    /// 运行中(由 `runner.state()` 透传 `Running` / `Building` / `Stopping`)
    Running,
    /// 正在 backoff 等待重启
    Restarting {
        /// 当前尝试次数(从 1 开始)
        attempt: u32,
        /// backoff 毫秒数
        backoff_millis: u64,
    },
    /// 超过 restart budget,不再尝试重启
    Fatal {
        /// 致命原因
        reason: NativeProcessError,
    },
}

impl BridgeSupervisorState {
    /// 是否处于 Fatal 状态
    #[must_use]
    pub fn is_fatal(&self) -> bool {
        matches!(self, Self::Fatal { .. })
    }
}

/// Bridge diagnostics(透传 `NativeProcessDiagnostics` + supervisor state + last_error)
#[derive(Debug, Clone)]
pub struct BridgeDiagnostics {
    /// supervisor 状态
    pub supervisor_state: BridgeSupervisorState,
    /// runner 透传 diagnostics
    pub runner: NativeProcessDiagnostics,
    /// 最近一次失败原因
    pub last_error: Option<NativeProcessError>,
}

impl BridgeDiagnostics {
    /// runner restart_count(诊断透传便捷访问)
    #[must_use]
    pub fn restart_count(&self) -> u32 {
        self.runner.restart_count
    }
}

/// Bridge-local supervisor
///
/// 单一 restart policy 来源 = `config.restart_policy`(由 `NativeProgramSpec.restart` 透传);
/// supervisor 不维护第二份可漂移 policy(方案 §5.5)。
pub struct BridgeSupervisor {
    spec: NativeProgramSpec,
    build: NativeBuildSpec,
    runner: NativeRunner,
    state: BridgeSupervisorState,
    /// supervisor 层累计重启次数(fatal 后由 `reset_fatal` 清零,新 runner 实例自 0 开始)
    restart_budget_used: u32,
    /// supervisor fatal / 最后一次失败原因,直到下次 `start()` 成功才清空
    last_error: Arc<Mutex<Option<NativeProcessError>>>,
}

impl std::fmt::Debug for BridgeSupervisor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BridgeSupervisor")
            .field("spec_name", &self.spec.name)
            .field("state", &self.state)
            .field("restart_budget_used", &self.restart_budget_used)
            .finish_non_exhaustive()
    }
}

impl BridgeSupervisor {
    /// 构造 supervisor
    #[must_use]
    pub fn new(spec: NativeProgramSpec, build: NativeBuildSpec) -> Self {
        let runner = NativeRunner::new(spec.clone(), build.clone());
        Self {
            spec,
            build,
            runner,
            state: BridgeSupervisorState::Stopped,
            restart_budget_used: 0,
            last_error: Arc::new(Mutex::new(None)),
        }
    }

    /// 当前 supervisor state
    #[must_use]
    pub fn state(&self) -> &BridgeSupervisorState {
        &self.state
    }

    /// 启动 native 进程(若 Fatal 则必须先 `reset_fatal`)
    ///
    /// # Errors
    /// - 处于 `Fatal` 时返回 `InvalidInput`
    /// - 启动失败时返回 `NativeProcess(...)` 错误并写入 `last_error`
    pub async fn start(&mut self) -> NavResult<()> {
        if self.state.is_fatal() {
            return Err(NavError::InvalidInput {
                field: "supervisor",
                reason: "bridge supervisor is in Fatal; call reset_fatal() first".to_string(),
            });
        }
        match self.runner.start().await {
            Ok(()) => {
                self.state = BridgeSupervisorState::Running;
                self.record_last_error(None);
                Ok(())
            }
            Err(e) => {
                let native = match &e {
                    NavError::NativeProcess(inner) => inner.clone(),
                    other => NativeProcessError::SpawnFailed {
                        source: format!("{other:?}"),
                    },
                };
                self.record_last_error(Some(native.clone()));
                self.state = BridgeSupervisorState::Fatal { reason: native };
                Err(e)
            }
        }
    }

    /// 主动停止(不计入 restart_count,亦不触发 fatal)
    ///
    /// # Errors
    /// - 透传 `NativeRunner::stop` 错误
    pub async fn stop(&mut self, timeout: Duration) -> NavResult<()> {
        let res = self.runner.stop(timeout).await;
        self.state = BridgeSupervisorState::Stopped;
        res
    }

    /// 轮询 + 必要时 backoff 重启;返回当前 supervisor state
    ///
    /// 重启策略来源 = `self.spec.restart`(由 `build_specs` 写入)。
    /// - 主动 stop / 正常退出(0)不计入 restart_count,亦不触发 fatal
    /// - 非 0 退出 / `Failed` 进入 `Restarting`,等待 backoff 后调用 `runner.start()`
    /// - 达到 `max_restarts` 上限 → `Fatal`
    ///
    /// # Errors
    /// 重启本身失败时返回 `NativeProcess(...)` 并把 supervisor 切到 `Fatal`。
    pub async fn poll_with_restart(&mut self) -> NavResult<BridgeSupervisorState> {
        let polled = self.runner.poll().await?;
        match &polled {
            NativeRunnerState::Exited { code: Some(0) } => {
                self.state = BridgeSupervisorState::Stopped;
                return Ok(self.state.clone());
            }
            NativeRunnerState::Exited { code: None } => {
                self.state = BridgeSupervisorState::Stopped;
                return Ok(self.state.clone());
            }
            NativeRunnerState::Running { .. } | NativeRunnerState::Building => {
                self.state = BridgeSupervisorState::Running;
                return Ok(self.state.clone());
            }
            NativeRunnerState::Stopping { .. } => {
                self.state = BridgeSupervisorState::Running;
                return Ok(self.state.clone());
            }
            NativeRunnerState::NotStarted => {
                return Ok(self.state.clone());
            }
            NativeRunnerState::Exited { code: Some(_) } | NativeRunnerState::Failed { .. } => {}
        }

        let reason = match &polled {
            NativeRunnerState::Exited { code } => NativeProcessError::ExitedNonZero { code: *code },
            NativeRunnerState::Failed { error } => error.clone(),
            _ => NativeProcessError::SpawnFailed {
                source: "unexpected polled state in restart branch".to_string(),
            },
        };
        self.record_last_error(Some(reason.clone()));

        let (max_restarts, backoff_millis) = match &self.spec.restart {
            RestartPolicy::Never => (0_u32, 0_u64),
            RestartPolicy::OnFailure {
                max_restarts,
                backoff_millis,
            } => (*max_restarts, *backoff_millis),
        };
        if max_restarts == 0 || self.restart_budget_used >= max_restarts {
            self.state = BridgeSupervisorState::Fatal { reason };
            return Ok(self.state.clone());
        }

        self.restart_budget_used = self.restart_budget_used.saturating_add(1);
        let attempt = self.restart_budget_used;
        self.state = BridgeSupervisorState::Restarting {
            attempt,
            backoff_millis,
        };
        if backoff_millis > 0 {
            tokio::time::sleep(Duration::from_millis(backoff_millis)).await;
        }

        self.runner.bump_restart();
        match self.runner.start().await {
            Ok(()) => {
                // 等待一小段时间让新进程有机会运行/失败:
                // 一些 native 进程(如 mock-native)会立即非 0 退出,需要在同一轮
                // poll_with_restart 中捕获,避免返回 Running 后下次 poll 才追认 Fatal。
                tokio::time::sleep(Duration::from_millis(20)).await;
                if let Ok(polled_after) = self.runner.poll().await {
                    match &polled_after {
                        NativeRunnerState::Exited { code: Some(0) }
                        | NativeRunnerState::Exited { code: None } => {
                            // 新进程已干净退出(不应触达:刚 start 立即 0 退出),记 Stopped
                            self.state = BridgeSupervisorState::Stopped;
                            Ok(self.state.clone())
                        }
                        NativeRunnerState::Exited { code: Some(_) }
                        | NativeRunnerState::Failed { .. } => {
                            // 新进程再次失败:按 budget 决策
                            let reason = match &polled_after {
                                NativeRunnerState::Exited { code } => {
                                    NativeProcessError::ExitedNonZero { code: *code }
                                }
                                NativeRunnerState::Failed { error } => error.clone(),
                                _ => NativeProcessError::SpawnFailed {
                                    source: "unexpected polled state after restart".to_string(),
                                },
                            };
                            self.record_last_error(Some(reason.clone()));
                            if max_restarts == 0 || self.restart_budget_used >= max_restarts {
                                self.state = BridgeSupervisorState::Fatal { reason };
                                return Ok(self.state.clone());
                            }
                            // budget 还未耗尽,再走一次 restart
                            self.restart_budget_used = self.restart_budget_used.saturating_add(1);
                            let attempt = self.restart_budget_used;
                            self.state = BridgeSupervisorState::Restarting {
                                attempt,
                                backoff_millis,
                            };
                            if backoff_millis > 0 {
                                tokio::time::sleep(Duration::from_millis(backoff_millis)).await;
                            }
                            self.runner.bump_restart();
                            match self.runner.start().await {
                                Ok(()) => {
                                    self.record_last_error(None);
                                    self.state = BridgeSupervisorState::Running;
                                    Ok(self.state.clone())
                                }
                                Err(e) => {
                                    let native = match &e {
                                        NavError::NativeProcess(inner) => inner.clone(),
                                        other => NativeProcessError::SpawnFailed {
                                            source: format!("{other:?}"),
                                        },
                                    };
                                    self.record_last_error(Some(native.clone()));
                                    self.state = BridgeSupervisorState::Fatal { reason: native };
                                    Err(e)
                                }
                            }
                        }
                        _ => {
                            self.record_last_error(None);
                            self.state = BridgeSupervisorState::Running;
                            Ok(self.state.clone())
                        }
                    }
                } else {
                    self.record_last_error(None);
                    self.state = BridgeSupervisorState::Running;
                    Ok(self.state.clone())
                }
            }
            Err(e) => {
                let native = match &e {
                    NavError::NativeProcess(inner) => inner.clone(),
                    other => NativeProcessError::SpawnFailed {
                        source: format!("{other:?}"),
                    },
                };
                self.record_last_error(Some(native.clone()));
                self.state = BridgeSupervisorState::Fatal { reason: native };
                Err(e)
            }
        }
    }

    /// 从 Fatal 状态恢复:用保存的 spec/build 重建 `NativeRunner` 实例,
    /// 重置 supervisor restart_budget / state → `Stopped`。
    /// `last_error` / `fatal_summary` 保留在 `BridgeDiagnostics` 直到下一次 `start()` 成功。
    ///
    /// # Errors
    /// - 非 Fatal 状态返回 `InvalidInput`(不 panic / 不 silent no-op)
    pub fn reset_fatal(&mut self) -> NavResult<()> {
        if !self.state.is_fatal() {
            return Err(NavError::InvalidInput {
                field: "supervisor",
                reason: "reset_fatal() requires Fatal state".to_string(),
            });
        }
        self.runner = NativeRunner::new(self.spec.clone(), self.build.clone());
        self.restart_budget_used = 0;
        self.state = BridgeSupervisorState::Stopped;
        Ok(())
    }

    /// 诊断快照
    #[must_use]
    pub fn diagnostics(&self) -> BridgeDiagnostics {
        let mut runner = self.runner.diagnostics_snapshot();
        runner.state = Some(self.runner.state().clone());
        BridgeDiagnostics {
            supervisor_state: self.state.clone(),
            runner,
            last_error: self
                .last_error
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone(),
        }
    }

    /// 内部便捷访问:当前 `NativeRunner`(仅 `bridge.rs` 内使用,避免暴露 lifecycle)
    pub(crate) fn runner(&mut self) -> &mut NativeRunner {
        &mut self.runner
    }

    fn record_last_error(&self, err: Option<NativeProcessError>) {
        if let Ok(mut guard) = self.last_error.lock() {
            *guard = err;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_spec() -> NativeProgramSpec {
        NativeProgramSpec {
            name: "terrain-bridge-test".to_string(),
            executable: std::path::PathBuf::from("/bin/true"),
            topics: Default::default(),
            args: vec![],
            env: Default::default(),
            cwd: None,
            restart: DEFAULT_RESTART_POLICY,
        }
    }

    fn dummy_build() -> NativeBuildSpec {
        NativeBuildSpec {
            build_command: None,
            policy: nuwax_nav_native_runner::NativeBuildPolicy::Never,
            cwd: None,
            env: Default::default(),
        }
    }

    #[test]
    fn default_policy_is_on_failure_with_3_1000() {
        assert_eq!(
            DEFAULT_RESTART_POLICY,
            RestartPolicy::OnFailure {
                max_restarts: 3,
                backoff_millis: 1_000,
            }
        );
    }

    #[test]
    fn new_supervisor_is_stopped() {
        let s = BridgeSupervisor::new(dummy_spec(), dummy_build());
        assert_eq!(s.state(), &BridgeSupervisorState::Stopped);
        let diag = s.diagnostics();
        assert_eq!(diag.supervisor_state, BridgeSupervisorState::Stopped);
        assert_eq!(diag.restart_count(), 0);
        assert!(diag.last_error.is_none());
    }

    #[test]
    fn reset_fatal_requires_fatal_state() {
        let mut s = BridgeSupervisor::new(dummy_spec(), dummy_build());
        let err = s.reset_fatal().unwrap_err();
        assert!(matches!(err, NavError::InvalidInput { .. }));
    }

    #[test]
    fn diagnostics_synthesizes_state_snapshot() {
        let s = BridgeSupervisor::new(dummy_spec(), dummy_build());
        let diag = s.diagnostics();
        assert!(diag.runner.state.is_some());
    }

    #[test]
    fn fatal_state_predicate() {
        let fatal = BridgeSupervisorState::Fatal {
            reason: NativeProcessError::SpawnFailed {
                source: "x".to_string(),
            },
        };
        assert!(fatal.is_fatal());
        assert!(!BridgeSupervisorState::Stopped.is_fatal());
        assert!(!BridgeSupervisorState::Running.is_fatal());
        assert!(!BridgeSupervisorState::Restarting {
            attempt: 1,
            backoff_millis: 100,
        }
        .is_fatal());
    }
}
