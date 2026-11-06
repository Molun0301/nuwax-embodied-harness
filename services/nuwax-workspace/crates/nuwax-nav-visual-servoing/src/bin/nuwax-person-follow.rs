//! Standalone production process for typed 3D person following.

#[cfg(test)]
use std::ffi::OsString;
use std::io;
use std::path::{Path, PathBuf};

use nuwax_nav_visual_servoing::lcm_owner::{
    PersonFollowLcmConfig, PersonFollowLcmError, PersonFollowLcmOwner,
};
use nuwax_transport::LcmTransport;



struct InternalReadyMarker {
    path: PathBuf,
}

impl InternalReadyMarker {
    fn from_runtime() -> io::Result<Option<Self>> {
        let path = nuwax_config::resolve_path(nuwax_config::FOLLOW_READY);
        std::fs::create_dir_all(path.parent().expect("就绪文件目录"))?;
        Self::from_path(path).map(Some)
    }

    #[cfg(test)]
    fn from_optional_path(value: Option<OsString>) -> io::Result<Option<Self>> {
        value.map(PathBuf::from).map(Self::from_path).transpose()
    }

    fn from_path(path: PathBuf) -> io::Result<Self> {
        if path.as_os_str().is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("就绪路径不能为空"),
            ));
        }
        remove_file_if_exists(&path)?;
        remove_file_if_exists(&Self::temp_path(&path))?;
        Ok(Self { path })
    }

    fn temp_path(path: &Path) -> PathBuf {
        let mut temporary = path.as_os_str().to_os_string();
        temporary.push(".tmp");
        PathBuf::from(temporary)
    }

    fn publish(&self) -> io::Result<()> {
        let temporary = Self::temp_path(&self.path);
        std::fs::write(&temporary, format!("pid={}\nready=1\n", std::process::id()))?;
        std::fs::rename(temporary, &self.path)
    }
}

impl Drop for InternalReadyMarker {
    fn drop(&mut self) {
        let _ = remove_file_if_exists(&self.path);
        let _ = remove_file_if_exists(&Self::temp_path(&self.path));
    }
}

fn remove_file_if_exists(path: &Path) -> io::Result<()> {
    match std::fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error),
    }
}

#[tokio::main]
async fn main() {
    nuwax_core::logging::init_component_logging("nuwax-person-follow", "info");
    if let Err(error) = run().await {
        tracing::error!(event = "process.failed", error = %error, "跟随进程退出");
        std::process::exit(1);
    }
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let ready_marker = InternalReadyMarker::from_runtime()?;
    let provider = nuwax_config::transport()?.lcm_url.clone();
    nuwax_config::person_follow()?;
    let transport = LcmTransport::new_real(&provider)?;
    let config = config_from_runtime();
    let owner = PersonFollowLcmOwner::new(transport, config)?;
    owner
        .run_until_with_ready(
            async {
                let _ = tokio::signal::ctrl_c().await;
            },
            || match ready_marker.as_ref() {
                Some(marker) => marker.publish().map_err(|error| {
                    PersonFollowLcmError::InvalidConfig(format!(
                        "internal readiness marker: {error}"
                    ))
                }),
                None => Ok(()),
            },
        )
        .await?;
    Ok(())
}

fn config_from_runtime() -> PersonFollowLcmConfig {
    PersonFollowLcmConfig::default()
}



#[cfg(test)]
mod tests {
    use super::*;

    fn test_path(label: &str) -> std::path::PathBuf {
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system time")
            .as_nanos();
        std::env::temp_dir().join(format!(
            "nuwax-person-follow-{label}-{}-{nonce}.ready",
            std::process::id()
        ))
    }

    #[test]
    fn internal_ready_marker_is_atomic_and_removed_on_drop() {
        let path = test_path("atomic");
        let marker = InternalReadyMarker::from_path(path.clone()).expect("marker guard");
        marker.publish().expect("publish marker");

        assert_eq!(
            std::fs::read_to_string(&path).expect("read marker"),
            format!("pid={}\nready=1\n", std::process::id())
        );
        assert!(!InternalReadyMarker::temp_path(&path).exists());

        drop(marker);
        assert!(!path.exists());
    }

    #[test]
    fn stale_internal_marker_is_removed_before_subscriptions() {
        let path = test_path("stale");
        std::fs::write(&path, "pid=1\nready=1\n").expect("write stale marker");
        let marker = InternalReadyMarker::from_path(path.clone()).expect("marker guard");
        assert!(!path.exists());
        drop(marker);
    }

    #[test]
    fn missing_internal_ready_env_keeps_direct_run_compatible() {
        assert!(InternalReadyMarker::from_optional_path(None)
            .expect("missing env is supported")
            .is_none());
    }

    #[test]
    fn explicitly_empty_internal_ready_env_is_rejected() {
        let error = InternalReadyMarker::from_optional_path(Some(OsString::new()))
            .err()
            .expect("empty env must fail");
        assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
    }
}
