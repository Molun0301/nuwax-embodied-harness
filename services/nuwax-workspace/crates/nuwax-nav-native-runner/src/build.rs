//! 构建策略执行(`Never` / `IfMissing` / `Always`)
//!
//! 规则:
//! - `Never` + executable 不存在 → `ExecutableMissing`
//! - `IfMissing` + executable 不存在 → 跑 build;成功仍缺失 → `ExecutableMissingAfterBuild`
//! - `Always` → 每次 start 前跑 build
//! - 构建命令用 shell 执行,继承环境并叠加 `env`,工作目录为 `cwd`
//! - stdout/stderr 写入 diagnostics tail;非 0 exit → `BuildFailed`

use std::path::PathBuf;
use std::process::Stdio;
use std::sync::{Arc, Mutex};

use nuwax_nav_base::error::NavResult;
use nuwax_nav_base::native::NativeProcessError;
use tokio::process::Command;

use crate::spec::{NativeBuildPolicy, NativeBuildSpec, NativeProgramSpec};

/// 在 spawn 前确保 binary 可用(并按 policy 决定是否构建)
///
/// 接收 `&Arc<Mutex<NativeProcessDiagnostics>>` 是为了避免 `MutexGuard` 跨 `.await`
/// 持有(`std::sync::Mutex` 不允许跨 await)。函数内部只持有短临界区,不会跨 await。
pub async fn ensure_built(
    spec: &NativeProgramSpec,
    build: &NativeBuildSpec,
    diagnostics: &Arc<Mutex<nuwax_nav_base::native::NativeProcessDiagnostics>>,
) -> NavResult<()> {
    match build.policy {
        NativeBuildPolicy::Never => {
            if !spec.executable.exists() {
                return Err(NativeProcessError::ExecutableMissing {
                    path: spec.executable.clone(),
                }
                .into());
            }
        }
        NativeBuildPolicy::IfMissing => {
            if !spec.executable.exists() {
                run_build(spec, build, diagnostics).await?;
                if !spec.executable.exists() {
                    return Err(NativeProcessError::ExecutableMissingAfterBuild {
                        path: spec.executable.clone(),
                    }
                    .into());
                }
            }
        }
        NativeBuildPolicy::Always => {
            run_build(spec, build, diagnostics).await?;
            if !spec.executable.exists() {
                return Err(NativeProcessError::ExecutableMissingAfterBuild {
                    path: spec.executable.clone(),
                }
                .into());
            }
        }
    }
    Ok(())
}

async fn run_build(
    spec: &NativeProgramSpec,
    build: &NativeBuildSpec,
    diagnostics: &Arc<Mutex<nuwax_nav_base::native::NativeProcessDiagnostics>>,
) -> NavResult<()> {
    let Some(cmd) = &build.build_command else {
        return Err(NativeProcessError::InvalidSpec {
            field: "build_command".to_string(),
            reason: "policy requires a build_command but none provided".to_string(),
        }
        .into());
    };
    let cwd: PathBuf = build
        .cwd
        .clone()
        .or(spec.cwd.clone())
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")));
    let mut command = Command::new("sh");
    command.arg("-c").arg(cmd);
    command.current_dir(&cwd);
    for (k, v) in &build.env {
        command.env(k, v);
    }
    for (k, v) in &spec.env {
        command.env(k, v);
    }
    command.stdout(Stdio::piped());
    command.stderr(Stdio::piped());
    tracing::info!(
        component = "nuwax-nav-native-runner",
        cmd = cmd,
        cwd = %cwd.display(),
        "running build command"
    );
    let output = command
        .output()
        .await
        .map_err(|e| NativeProcessError::SpawnFailed {
            source: e.to_string(),
        })?;
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    if !stdout.is_empty() {
        let mut g = diagnostics.lock().unwrap_or_else(|e| e.into_inner());
        g.push_stdout(&stdout);
    }
    if !stderr.is_empty() {
        let mut g = diagnostics.lock().unwrap_or_else(|e| e.into_inner());
        g.push_stderr(&stderr);
    }
    if !output.status.success() {
        return Err(NativeProcessError::BuildFailed {
            command: cmd.clone(),
            code: output.status.code(),
        }
        .into());
    }
    Ok(())
}

/// 把 stdout/stderr reader **后台** 派发到 diagnostics tail(不阻塞 caller)
///
/// 返回 join handle,供调用方在 stop 时 `await`(避免 stale writer)。
pub fn spawn_drain(
    stdout: tokio::process::ChildStdout,
    stderr: tokio::process::ChildStderr,
    diagnostics: std::sync::Arc<std::sync::Mutex<nuwax_nav_base::native::NativeProcessDiagnostics>>,
) -> tokio::task::JoinHandle<()> {
    use tokio::io::AsyncBufReadExt;
    let mut out_reader = tokio::io::BufReader::new(stdout);
    let mut err_reader = tokio::io::BufReader::new(stderr);
    tokio::spawn(async move {
        let mut buf = String::new();
        loop {
            buf.clear();
            let n1 = out_reader.read_line(&mut buf).await.unwrap_or(0);
            if n1 > 0 {
                diagnostics
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .push_stdout(&buf);
            }
            buf.clear();
            let n2 = err_reader.read_line(&mut buf).await.unwrap_or(0);
            if n2 > 0 {
                diagnostics
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .push_stderr(&buf);
            }
            if n1 == 0 && n2 == 0 {
                break;
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_base::native::NativeProcessDiagnostics;
    use nuwax_nav_base::native::RestartPolicy;
    use std::collections::BTreeMap;

    fn dummy_spec(path: &std::path::Path) -> NativeProgramSpec {
        NativeProgramSpec {
            name: "x".into(),
            executable: path.to_path_buf(),
            topics: BTreeMap::new(),
            args: vec![],
            env: BTreeMap::new(),
            cwd: None,
            restart: RestartPolicy::Never,
        }
    }

    #[tokio::test]
    async fn ensure_built_never_rejects_missing() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("does_not_exist");
        let spec = dummy_spec(&exe);
        let build = NativeBuildSpec::default();
        let diag = Arc::new(Mutex::new(NativeProcessDiagnostics::new("test")));
        let err = ensure_built(&spec, &build, &diag).await.unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::NativeProcess(
                NativeProcessError::ExecutableMissing { .. }
            )
        ));
    }

    #[tokio::test]
    async fn ensure_built_never_accepts_existing() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("exists.sh");
        std::fs::write(&exe, "#!/bin/sh\n").unwrap();
        let spec = dummy_spec(&exe);
        let build = NativeBuildSpec::default();
        let diag = Arc::new(Mutex::new(NativeProcessDiagnostics::new("test")));
        ensure_built(&spec, &build, &diag).await.unwrap();
    }

    #[tokio::test]
    async fn ensure_built_if_missing_builds() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("out.sh");
        let cmd = format!("printf 'x' > {}", exe.display());
        let build = NativeBuildSpec {
            build_command: Some(cmd),
            policy: NativeBuildPolicy::IfMissing,
            cwd: None,
            env: BTreeMap::new(),
        };
        let spec = dummy_spec(&exe);
        let diag = Arc::new(Mutex::new(NativeProcessDiagnostics::new("test")));
        ensure_built(&spec, &build, &diag).await.unwrap();
        assert!(exe.exists());
    }

    #[tokio::test]
    async fn ensure_built_if_missing_returns_missing_after_build() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("out.sh");
        let build = NativeBuildSpec {
            build_command: Some("echo hello > /dev/null".to_string()),
            policy: NativeBuildPolicy::IfMissing,
            cwd: None,
            env: BTreeMap::new(),
        };
        let spec = dummy_spec(&exe);
        let diag = Arc::new(Mutex::new(NativeProcessDiagnostics::new("test")));
        let err = ensure_built(&spec, &build, &diag).await.unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::NativeProcess(
                NativeProcessError::ExecutableMissingAfterBuild { .. }
            )
        ));
    }

    #[tokio::test]
    async fn ensure_built_always_returns_build_failed_on_nonzero() {
        let tmp = tempfile::tempdir().unwrap();
        let exe = tmp.path().join("out.sh");
        let build = NativeBuildSpec {
            build_command: Some("false".to_string()),
            policy: NativeBuildPolicy::Always,
            cwd: None,
            env: BTreeMap::new(),
        };
        let spec = dummy_spec(&exe);
        let diag = Arc::new(Mutex::new(NativeProcessDiagnostics::new("test")));
        let err = ensure_built(&spec, &build, &diag).await.unwrap_err();
        assert!(matches!(
            err,
            nuwax_nav_base::error::NavError::NativeProcess(NativeProcessError::BuildFailed { .. })
        ));
    }
}
