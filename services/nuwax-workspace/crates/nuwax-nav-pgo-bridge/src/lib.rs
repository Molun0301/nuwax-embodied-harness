//! PGO (Pose Graph Optimization) C++ native bridge
//!
//! 仅包装 `result/bin/pgo` 进程，不重写 GTSAM iSAM2 / PCL ICP 算法。
//! 端口表按方案 §4.5：In `registered_scan`/`odometry`，Out `corrected_odometry`/`global_map`/`pgo_tf`。
//!
//! **Rust 额外行为**：本 crate 订阅 `pgo_tf`，并通过 `nuwax-tf` 发布 map->odom 变换。
//! 转换逻辑独立为 [`tf_helper::odometry_to_transform_stamped`] 与 [`tf_helper::apply_pgo_tf_to_tree`]，
//! 不藏在 runner 内，可以独立单元测试。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod spec;
pub mod tf_helper;

pub use spec::{
    build_spec, cli_exclude, cli_mapping, cli_name_override, to_native_args, PgoBridge,
    PgoBridgeConfig, PgoRestartPolicy, PgoTopics,
};
pub use tf_helper::{
    apply_pgo_tf_to_tree, odometry_to_transform_stamped, OdometryLike, PgoTfMessage,
};

use std::path::Path;
use std::time::Duration;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::NativeProcessDiagnostics;
use nuwax_nav_native_runner::{NativeBuildPolicy, NativeBuildSpec, NativeRunner};

/// PGO bridge 错误。
#[derive(Debug, thiserror::Error)]
pub enum PgoBridgeError {
    /// 配置非法
    #[error("invalid PGO config: {0}")]
    InvalidConfig(String),
    /// cwd 不存在
    #[error("PGO cwd does not exist: {0}")]
    CwdMissing(String),
    /// 进程错误
    #[error("native runner error: {0}")]
    Runner(String),
}

/// PGO bridge 结果。
pub type PgoBridgeResult<T> = Result<T, PgoBridgeError>;

/// 校验配置合法性。
pub fn validate_config(config: &PgoBridgeConfig) -> Result<(), PgoBridgeError> {
    if config.executable.as_os_str().is_empty() {
        return Err(PgoBridgeError::InvalidConfig(
            "executable is empty".to_string(),
        ));
    }
    if let Some(cmd) = &config.build_command {
        if cmd.contains("nix build github:") {
            return Err(PgoBridgeError::InvalidConfig(format!(
                "build_command must not auto-run 'nix build github:' (got: {cmd})"
            )));
        }
    }
    if let Some(cwd) = &config.cwd {
        if !cwd.as_path().exists() {
            return Err(PgoBridgeError::CwdMissing(cwd.display().to_string()));
        }
    }
    Ok(())
}

/// 便捷构造 cwd：默认指向 `cpp/` 子目录（与 Python 行为一致）。
#[must_use]
pub fn default_cwd() -> std::path::PathBuf {
    Path::new("cpp").to_path_buf()
}

impl PgoBridge {
    /// 启动 native PGO 进程。
    pub async fn start(&mut self) -> PgoBridgeResult<()> {
        self.runner
            .start()
            .await
            .map_err(|e| PgoBridgeError::Runner(format!("{e:?}")))
    }

    /// 停止 native PGO 进程。
    pub async fn stop(&mut self, timeout: Duration) -> PgoBridgeResult<()> {
        self.runner
            .stop(timeout)
            .await
            .map_err(|e| PgoBridgeError::Runner(format!("{e:?}")))
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
pub(crate) fn build_runner(config: &PgoBridgeConfig, topics: &PgoTopics) -> NativeRunner {
    let spec = spec::build_spec(config.clone(), topics);
    let build = NativeBuildSpec {
        build_command: config.build_command.clone(),
        policy: NativeBuildPolicy::Never,
        cwd: config.cwd.clone(),
        env: config.env.clone(),
    };
    NativeRunner::new(spec, build)
}
