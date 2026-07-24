//! FAR Planner C++ native bridge
//!
//! 仅包装 `result/bin/far_planner_native` 进程，不重写算法。
//! 端口（topic）与配置字段按方案 §4.2 完整保留；C++ 历史拼写不一致的参数名
//! （如 `robot_dim`）通过 `cli_name_override` 输出历史名。
//!
//! 进程生命周期由 `nuwax-nav-native-runner` 统一管理，本 crate 不自带进程管理逻辑。

#![deny(unsafe_code)]
#![warn(missing_docs)]

/// Auditable standalone visibility-graph fallback core.
pub mod core;
pub mod spec;

pub use spec::{
    build_spec, cli_exclude, cli_mapping, cli_name_override, to_native_args, FarPlannerBridge,
    FarPlannerBridgeConfig, FarPlannerRestartPolicy, FarPlannerTopics,
};

use std::time::Duration;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::NativeProcessDiagnostics;
use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeRunner};

/// FAR Planner bridge 错误。
#[derive(Debug, thiserror::Error)]
pub enum FarPlannerBridgeError {
    /// 配置非法
    #[error("invalid FAR planner config: {0}")]
    InvalidConfig(String),
    /// 进程启动/停止错误（由 `NativeRunner` 上抛）
    #[error("native runner error: {0}")]
    Runner(String),
}

/// FAR Planner bridge 顶层结果。
pub type FarPlannerBridgeResult<T> = Result<T, FarPlannerBridgeError>;

/// 校验 FAR Planner 配置合法性。
pub fn validate_config(config: &FarPlannerBridgeConfig) -> Result<(), FarPlannerBridgeError> {
    if config.executable.as_os_str().is_empty() {
        return Err(FarPlannerBridgeError::InvalidConfig(
            "executable is empty".to_string(),
        ));
    }
    if let Some(cmd) = &config.build_command {
        if cmd.contains("nix build github:") {
            return Err(FarPlannerBridgeError::InvalidConfig(format!(
                "build_command must not auto-run 'nix build github:' (got: {cmd})"
            )));
        }
    }
    Ok(())
}

impl FarPlannerBridge {
    /// 启动 native FAR Planner 进程。
    ///
    /// # Errors
    /// 当配置非法或进程启动失败时返回对应错误。
    pub async fn start(&mut self) -> FarPlannerBridgeResult<()> {
        self.runner
            .start()
            .await
            .map_err(|e| FarPlannerBridgeError::Runner(format!("{e:?}")))
    }

    /// 停止 native FAR Planner 进程（带 timeout）。
    pub async fn stop(&mut self, timeout: Duration) -> FarPlannerBridgeResult<()> {
        self.runner
            .stop(timeout)
            .await
            .map_err(|e| FarPlannerBridgeError::Runner(format!("{e:?}")))
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

/// 内部使用：从 `FarPlannerBridgeConfig` 与 `FarPlannerTopics` 构造 `NativeRunner`。
pub(crate) fn build_runner(
    config: &FarPlannerBridgeConfig,
    topics: &FarPlannerTopics,
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
