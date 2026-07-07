//! TARE bridge 顶层 facade:封装 `BridgeSupervisor` 暴露 lifecycle,内部构造
//! `NativeRunner` 实例;`pub async fn start/stop/poll_with_restart/diagnostics/reset_fatal`
//! 全部委托 supervisor(方案 §5.3),**不**向上层暴露裸 `NativeRunner` lifecycle。

use std::time::Duration;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::{NativeProcessError, NativeRunnerState};

use crate::spec::{build_specs, TarePlannerBridgeConfig, TarePlannerTopics};
use crate::supervisor::{BridgeDiagnostics, BridgeSupervisor, BridgeSupervisorState};

/// TARE bridge 错误
#[derive(Debug, thiserror::Error)]
pub enum TareBridgeError {
    /// Spec / config 字段非法
    #[error("invalid TARE bridge config: {0}")]
    InvalidConfig(String),
    /// Supervisor 在非 Fatal 状态调用 `reset_fatal`
    #[error("supervisor state error: {0}")]
    SupervisorState(String),
    /// 透传 `NativeProcessError`
    #[error("native process error: {0}")]
    NativeProcess(#[from] NativeProcessError),
}

/// TARE bridge 顶层结果
pub type TareBridgeResult<T> = Result<T, TareBridgeError>;

/// TARE bridge(封装 `BridgeSupervisor` + 配置)
pub struct TarePlannerBridge {
    supervisor: BridgeSupervisor,
    #[allow(dead_code)]
    config: TarePlannerBridgeConfig,
}

impl std::fmt::Debug for TarePlannerBridge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TarePlannerBridge")
            .field("state", &self.supervisor.state())
            .field("config", &self.config)
            .finish_non_exhaustive()
    }
}

impl TarePlannerBridge {
    /// 构造 bridge;自动 `build_specs` + `validate_config`
    ///
    /// # Errors
    /// 配置非法返回 `TareBridgeError::InvalidConfig`。
    pub fn new(
        config: TarePlannerBridgeConfig,
        topics: &TarePlannerTopics,
    ) -> TareBridgeResult<Self> {
        let (spec, build) = build_specs(config.clone(), topics)
            .map_err(|e| TareBridgeError::InvalidConfig(format!("{e:?}")))?;
        Ok(Self {
            supervisor: BridgeSupervisor::new(spec, build),
            config,
        })
    }

    /// 启动 native 进程
    ///
    /// # Errors
    /// 启动失败时把 supervisor 切到 `Fatal` 并返回错误。
    pub async fn start(&mut self) -> TareBridgeResult<()> {
        self.supervisor.start().await.map_err(map_supervisor_err)
    }

    /// 主动停止进程(带 timeout)
    ///
    /// # Errors
    /// 透传 `NativeRunner::stop` 错误。
    pub async fn stop(&mut self, timeout: Duration) -> TareBridgeResult<()> {
        self.supervisor
            .stop(timeout)
            .await
            .map_err(map_supervisor_err)
    }

    /// 轮询 + 必要时 backoff 重启;返回 supervisor state
    ///
    /// # Errors
    /// 重启 / 启动失败时返回错误并把 supervisor 切到 `Fatal`。
    pub async fn poll_with_restart(&mut self) -> TareBridgeResult<BridgeSupervisorState> {
        self.supervisor
            .poll_with_restart()
            .await
            .map_err(map_supervisor_err)
    }

    /// 透传 `NativeRunner::poll`(只读,不触发重启 / 不切 supervisor state)
    pub async fn poll(&mut self) -> TareBridgeResult<NativeRunnerState> {
        self.supervisor.runner().poll().await.map_err(|e| match e {
            nuwax_nav_base::error::NavError::NativeProcess(inner) => {
                TareBridgeError::NativeProcess(inner)
            }
            other => TareBridgeError::InvalidConfig(format!("poll error: {other:?}")),
        })
    }

    /// 当前 supervisor state
    #[must_use]
    pub fn state(&self) -> &BridgeSupervisorState {
        self.supervisor.state()
    }

    /// 诊断快照
    #[must_use]
    pub fn diagnostics(&self) -> BridgeDiagnostics {
        self.supervisor.diagnostics()
    }

    /// 从 Fatal 恢复:重建 `NativeRunner`,重置 restart_budget,state → `Stopped`
    ///
    /// # Errors
    /// 非 Fatal 状态返回 `TareBridgeError::SupervisorState(...)`(不 panic / 不 silent no-op)。
    pub fn reset_fatal(&mut self) -> TareBridgeResult<()> {
        self.supervisor
            .reset_fatal()
            .map_err(|e| TareBridgeError::SupervisorState(format!("{e:?}")))
    }
}

fn map_supervisor_err(e: nuwax_nav_base::error::NavError) -> TareBridgeError {
    match e {
        nuwax_nav_base::error::NavError::NativeProcess(inner) => {
            TareBridgeError::NativeProcess(inner)
        }
        other => TareBridgeError::InvalidConfig(format!("supervisor error: {other:?}")),
    }
}

// 静默使用以避免 `use` 警告
#[allow(dead_code)]
fn _ensure_navresult_used() -> NavResult<()> {
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> TarePlannerBridgeConfig {
        TarePlannerBridgeConfig::default()
    }

    fn topics_all() -> TarePlannerTopics {
        TarePlannerTopics {
            registered_scan: Some("/registered_scan".into()),
            odometry: Some("/odometry".into()),
            way_point: Some("/way_point".into()),
        }
    }

    #[test]
    fn new_with_default_config_is_ok() {
        let b = TarePlannerBridge::new(cfg(), &topics_all()).expect("ok");
        assert_eq!(b.state(), &BridgeSupervisorState::Stopped);
    }

    #[test]
    fn new_rejects_invalid_config() {
        let mut c = cfg();
        c.exploration_range = -1.0;
        let err = TarePlannerBridge::new(c, &topics_all()).unwrap_err();
        assert!(matches!(err, TareBridgeError::InvalidConfig(_)));
    }

    #[test]
    fn diagnostics_includes_supervisor_state_and_runner() {
        let b = TarePlannerBridge::new(cfg(), &topics_all()).expect("ok");
        let d = b.diagnostics();
        assert_eq!(d.supervisor_state, BridgeSupervisorState::Stopped);
        assert_eq!(d.restart_count(), 0);
    }

    #[test]
    fn reset_fatal_returns_supervisor_state_error_when_not_fatal() {
        let mut b = TarePlannerBridge::new(cfg(), &topics_all()).expect("ok");
        let err = b.reset_fatal().unwrap_err();
        assert!(matches!(err, TareBridgeError::SupervisorState(_)));
    }
}
