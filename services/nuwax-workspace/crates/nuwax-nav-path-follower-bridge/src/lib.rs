//! Path Follower C++ native bridge
//!
//! 仅包装 `result/bin/path_follower` 进程（CMU pure pursuit controller），不重写算法。
//! 配置字段按方案 §4.4 完整保留；C++ 历史拼写不一致的参数名（`lookAheadDis`）通过
//! `cli_name_override` 输出历史名。
//!
//! 注意：`max_yaw_rate` 字段保持历史单位 deg/s；转换责任属于 C++ 内部。
//! 不得在 Rust bridge 层做 deg <-> rad 转换。

#![deny(unsafe_code)]
#![warn(missing_docs)]

/// Auditable pure-pursuit controller core.
pub mod core;
pub mod spec;

pub use spec::{
    build_spec, cli_exclude, cli_mapping, cli_name_override, to_native_args, PathFollowerBridge,
    PathFollowerBridgeConfig, PathFollowerRestartPolicy, PathFollowerTopics, VehicleConfig,
};

use std::time::Duration;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::NativeProcessDiagnostics;
use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeRunner};

/// Path Follower bridge 错误。
#[derive(Debug, thiserror::Error)]
pub enum PathFollowerBridgeError {
    /// 配置非法
    #[error("invalid path follower config: {0}")]
    InvalidConfig(String),
    /// 进程错误
    #[error("native runner error: {0}")]
    Runner(String),
}

/// Path Follower bridge 结果。
pub type PathFollowerBridgeResult<T> = Result<T, PathFollowerBridgeError>;

/// 校验配置合法性。
pub fn validate_config(config: &PathFollowerBridgeConfig) -> Result<(), PathFollowerBridgeError> {
    if config.executable.as_os_str().is_empty() {
        return Err(PathFollowerBridgeError::InvalidConfig(
            "executable is empty".to_string(),
        ));
    }
    if let Some(cmd) = &config.build_command {
        if cmd.contains("nix build github:") {
            return Err(PathFollowerBridgeError::InvalidConfig(format!(
                "build_command must not auto-run 'nix build github:' (got: {cmd})"
            )));
        }
    }
    Ok(())
}

impl PathFollowerBridge {
    /// 启动 native Path Follower 进程。
    pub async fn start(&mut self) -> PathFollowerBridgeResult<()> {
        self.runner
            .start()
            .await
            .map_err(|e| PathFollowerBridgeError::Runner(format!("{e:?}")))
    }

    /// 停止 native Path Follower 进程。
    pub async fn stop(&mut self, timeout: Duration) -> PathFollowerBridgeResult<()> {
        self.runner
            .stop(timeout)
            .await
            .map_err(|e| PathFollowerBridgeError::Runner(format!("{e:?}")))
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
    config: &PathFollowerBridgeConfig,
    topics: &PathFollowerTopics,
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
