//! Local Planner C++ native bridge
//!
//! 仅包装 `result/bin/local_planner` 进程，不重写算法。
//! 配置字段按方案 §4.3 完整保留，`Option<f64>` 配置 `None` 不传。
//! `cli_name_override` 覆盖 50+ 个 camelCase C++ 参数名。
//! `goal_reached` 是 typed output，进入 `nuwax-nav-stack` 状态机消费。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod spec;

pub use spec::{
    build_spec, cli_exclude, cli_mapping, cli_name_override, to_native_args, GoalReached,
    LocalPlannerBridge, LocalPlannerBridgeConfig, LocalPlannerRestartPolicy, LocalPlannerTopics,
};

use std::path::Path;
use std::time::Duration;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::NativeProcessDiagnostics;
use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeRunner};

/// Local Planner bridge 错误。
#[derive(Debug, thiserror::Error)]
pub enum LocalPlannerBridgeError {
    /// 配置非法
    #[error("invalid local planner config: {0}")]
    InvalidConfig(String),
    /// paths_dir 路径不存在
    #[error("paths_dir does not exist: {0}")]
    PathsDirMissing(String),
    /// 进程错误
    #[error("native runner error: {0}")]
    Runner(String),
}

/// Local Planner bridge 结果。
pub type LocalPlannerBridgeResult<T> = Result<T, LocalPlannerBridgeError>;

/// 校验配置合法性。
pub fn validate_config(config: &LocalPlannerBridgeConfig) -> Result<(), LocalPlannerBridgeError> {
    if config.executable.as_os_str().is_empty() {
        return Err(LocalPlannerBridgeError::InvalidConfig(
            "executable is empty".to_string(),
        ));
    }
    if let Some(cmd) = &config.build_command {
        if cmd.contains("nix build github:") {
            return Err(LocalPlannerBridgeError::InvalidConfig(format!(
                "build_command must not auto-run 'nix build github:' (got: {cmd})"
            )));
        }
    }
    if !config.paths_dir.is_empty() {
        let p = Path::new(&config.paths_dir);
        if !p.exists() {
            return Err(LocalPlannerBridgeError::PathsDirMissing(
                config.paths_dir.clone(),
            ));
        }
    }
    Ok(())
}

impl LocalPlannerBridge {
    /// 启动 native Local Planner 进程。
    pub async fn start(&mut self) -> LocalPlannerBridgeResult<()> {
        self.runner
            .start()
            .await
            .map_err(|e| LocalPlannerBridgeError::Runner(format!("{e:?}")))
    }

    /// 停止 native Local Planner 进程。
    pub async fn stop(&mut self, timeout: Duration) -> LocalPlannerBridgeResult<()> {
        self.runner
            .stop(timeout)
            .await
            .map_err(|e| LocalPlannerBridgeError::Runner(format!("{e:?}")))
    }

    /// 轮询进程状态。
    pub async fn poll(&mut self) -> NavResult<nuwax_nav_base::native::NativeRunnerState> {
        self.runner.poll().await
    }

    /// 获取诊断快照。
    #[must_use]
    pub fn diagnostics(&self) -> NativeProcessDiagnostics {
        self.runner.diagnostics_snapshot()
    }
}

/// 内部使用：从 config + topics 构造 `NativeRunner`。
pub(crate) fn build_runner(
    config: &LocalPlannerBridgeConfig,
    topics: &LocalPlannerTopics,
) -> NativeRunner {
    let spec = spec::build_spec(config.clone(), topics);
    let build = NativeBuildSpec {
        build_command: config.build_command.clone(),
        policy: NativeBuildPolicy::Never,
        cwd: config.cwd.clone(),
        env: config.env.clone(),
    };
    NativeRunner::new(spec, build)
}
