//! Standalone production process for typed 3D person following.

use std::ffi::OsString;
use std::io;
use std::path::{Path, PathBuf};

use nuwax_nav_visual_servoing::lcm_owner::{
    PersonFollowLcmConfig, PersonFollowLcmError, PersonFollowLcmOwner,
};
use nuwax_transport::LcmTransport;

const DEFAULT_LCM_URL: &str = "udpm://239.255.76.67:7667?ttl=1";
const INTERNAL_READY_FILE_ENV: &str = "NUWAX_PERSON_FOLLOW_INTERNAL_READY_FILE";

struct InternalReadyMarker {
    path: PathBuf,
}

impl InternalReadyMarker {
    fn from_env() -> io::Result<Option<Self>> {
        Self::from_env_value(std::env::var_os(INTERNAL_READY_FILE_ENV))
    }

    fn from_env_value(value: Option<OsString>) -> io::Result<Option<Self>> {
        value.map(PathBuf::from).map(Self::from_path).transpose()
    }

    fn from_path(path: PathBuf) -> io::Result<Self> {
        if path.as_os_str().is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{INTERNAL_READY_FILE_ENV} must not be empty"),
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
    let ready_marker = InternalReadyMarker::from_env()?;
    let provider = std::env::var("NUWAX_LCM_URL").unwrap_or_else(|_| DEFAULT_LCM_URL.to_string());
    let transport = LcmTransport::new_real(&provider)?;
    let config = config_from_env();
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

fn config_from_env() -> PersonFollowLcmConfig {
    let mut config = PersonFollowLcmConfig::default();
    override_string(
        "NUWAX_PERSON_FOLLOW_TARGET_CHANNEL",
        &mut config.target_channel,
    );
    override_string(
        "NUWAX_PERSON_FOLLOW_POINTCLOUD_CHANNEL",
        &mut config.pointcloud_channel,
    );
    override_string("NUWAX_PERSON_FOLLOW_TF_CHANNEL", &mut config.tf_channel);
    override_string(
        "NUWAX_PERSON_FOLLOW_COMMAND_CHANNEL",
        &mut config.command_channel,
    );
    config
}

fn override_string(name: &str, target: &mut String) {
    if let Ok(value) = std::env::var(name) {
        *target = value;
    }
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
        assert!(InternalReadyMarker::from_env_value(None)
            .expect("missing env is supported")
            .is_none());
    }

    #[test]
    fn explicitly_empty_internal_ready_env_is_rejected() {
        let error = InternalReadyMarker::from_env_value(Some(OsString::new()))
            .err()
            .expect("empty env must fail");
        assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
    }
}
