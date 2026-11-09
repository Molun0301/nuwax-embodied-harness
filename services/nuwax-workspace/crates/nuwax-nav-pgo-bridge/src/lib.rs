//! PGO (Pose Graph Optimization) C++ core and native bridge.
//!
//! `native/pgo` contains the migrated source responsibility and builds a real weighted
//! pose-graph optimizer. The executable exposes a deterministic batch contract for
//! verification and a signal-aware lifecycle mode for the runner. It deliberately does
//! not publish canonical map-to-odom corrections: runtime arbitration remains external,
//! so this optional core cannot double-write TF with the Rust mapping PGO owner.

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod online;
pub mod spec;

pub use online::*;
pub use spec::{
    build_spec, cli_exclude, cli_mapping, cli_name_override, to_native_args, PgoBridge,
    PgoBridgeConfig, PgoRestartPolicy, PgoTopics,
};

use std::time::Duration;

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::NativeProcessDiagnostics;


/// 返回与当前程序一起安装的原生位姿图优化程序。
#[must_use]
pub fn native_executable() -> std::path::PathBuf {
    std::env::current_exe().expect("当前可执行文件位置")
        .with_file_name("pgo")
}

/// Return the build-time native PGO artifact for side-effect-free discovery only.
#[must_use]
pub fn discovery_native_executable() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("NUWAX_PGO_NATIVE_BIN"))
}

/// Return the build-time guardian artifact for side-effect-free discovery only.
#[must_use]
pub fn discovery_group_guardian_executable() -> std::path::PathBuf {
    nuwax_nav_native_runner::discovery_group_guardian_executable().to_path_buf()
}

impl PgoBridge {
    /// Start the lifecycle-only native artifact. Online correction work must
    /// use [`OnlinePgoOwner`]; this facade has no TF/map apply capability.
    pub async fn start(&mut self) -> NavResult<()> {
        self.runner.start().await
    }

    /// Stop the lifecycle-only native artifact.
    pub async fn stop(&mut self, timeout: Duration) -> NavResult<()> {
        self.runner.stop(timeout).await
    }

    /// Poll the lifecycle-only native artifact.
    pub async fn poll(&mut self) -> NavResult<nuwax_nav_base::native::NativeRunnerState> {
        self.runner.poll().await
    }

    /// Return a bounded lifecycle diagnostic snapshot.
    #[must_use]
    pub fn diagnostics(&self) -> NativeProcessDiagnostics {
        self.runner.diagnostics_snapshot()
    }
}
