//! Strict contract for the vendored Dimos FastLIO2 and Point-LIO native programs.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::net::Ipv4Addr;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus};
use std::sync::Mutex;

pub const LIDAR_TOPIC: &str = "/lidar#sensor_msgs.PointCloud2";
pub const ODOMETRY_TOPIC: &str = "/odometry#nav_msgs.Odometry";
pub const ODOM_FRAME: &str = "odom";
pub const BODY_FRAME: &str = "body";
pub const SENSOR_FRAME: &str = "mid360_link";

pub const FASTLIO2_REVISION: &str = "a32c9f599940a94595aa72868e2e4ab436a44b75";
pub const POINTLIO_REVISION: &str = "288e357e5457723c1cce4d4060f76ed7f85b10d4";
pub const DIMOS_LCM_REVISION: &str = "0aa72b7b1bd3a65f50f5c03485ee9b728df56afe";
pub const LIVOX_SDK2_REVISION: &str = "v1.2.5";
pub const LIVOX_SDK2_SOURCE_HASH: &str = "sha256-NGscO/vLiQ17yQJtdPyFzhhMGE89AJ9kTL5cSun/bpU=";

const MAX_TOPIC_BYTES: usize = 1_024;
const MAX_FRAME_BYTES: usize = 255;
const MAX_FREQUENCY_HZ: f64 = 1_000.0;
static ACTIVE_NATIVE_BACKEND: Mutex<Option<NativeBackend>> = Mutex::new(None);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NativeBackend {
    FastLio2,
    PointLio,
}

impl NativeBackend {
    pub fn from_flags(fastlio2: bool, pointlio: bool) -> Result<Self, String> {
        match (fastlio2, pointlio) {
            (true, false) => Ok(Self::FastLio2),
            (false, true) => Ok(Self::PointLio),
            (false, false) => Err("exactly one native LIO backend must be selected".into()),
            (true, true) => Err("FastLIO2 and Point-LIO are mutually exclusive".into()),
        }
    }

    pub const fn source_relpath(self) -> &'static str {
        match self {
            Self::FastLio2 => "native/fastlio2/cpp",
            Self::PointLio => "native/pointlio/cpp",
        }
    }

    pub const fn executable_relpath(self) -> &'static str {
        match self {
            Self::FastLio2 => "result/bin/fastlio2_native",
            Self::PointLio => "result/bin/pointlio_native",
        }
    }

    pub const fn package_name(self) -> &'static str {
        match self {
            Self::FastLio2 => "fastlio2_native",
            Self::PointLio => "pointlio_native",
        }
    }

    pub const fn upstream_revision(self) -> &'static str {
        match self {
            Self::FastLio2 => FASTLIO2_REVISION,
            Self::PointLio => POINTLIO_REVISION,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NativeLioConfig {
    pub backend: NativeBackend,
    pub lidar_topic: String,
    pub odometry_topic: String,
    pub frame_id: String,
    pub child_frame_id: String,
    pub sensor_frame_id: String,
    pub host_ip: String,
    pub lidar_ip: String,
    pub lidar_frequency_hz: f64,
    pub pointcloud_frequency_hz: f64,
    pub odometry_frequency_hz: f64,
    pub debug: bool,
}

impl NativeLioConfig {
    pub fn for_backend(backend: NativeBackend) -> Self {
        Self {
            backend,
            lidar_topic: LIDAR_TOPIC.into(),
            odometry_topic: ODOMETRY_TOPIC.into(),
            frame_id: ODOM_FRAME.into(),
            child_frame_id: BODY_FRAME.into(),
            sensor_frame_id: SENSOR_FRAME.into(),
            host_ip: "192.168.1.5".into(),
            lidar_ip: "192.168.1.155".into(),
            lidar_frequency_hz: 10.0,
            pointcloud_frequency_hz: 10.0,
            odometry_frequency_hz: 30.0,
            debug: false,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        validate_typed_topic(&self.lidar_topic, "sensor_msgs.PointCloud2", "lidar_topic")?;
        validate_typed_topic(&self.odometry_topic, "nav_msgs.Odometry", "odometry_topic")?;
        validate_frame(&self.frame_id, "frame_id")?;
        validate_frame(&self.child_frame_id, "child_frame_id")?;
        validate_frame(&self.sensor_frame_id, "sensor_frame_id")?;
        if self.frame_id == self.child_frame_id
            || self.frame_id == self.sensor_frame_id
            || self.child_frame_id == self.sensor_frame_id
        {
            return Err("odom, body, and sensor frame IDs must be distinct".into());
        }

        let host_ip = validate_ipv4(&self.host_ip, "host_ip")?;
        let lidar_ip = validate_ipv4(&self.lidar_ip, "lidar_ip")?;
        if host_ip == lidar_ip {
            return Err("host_ip and lidar_ip must be distinct".into());
        }

        validate_frequency(self.lidar_frequency_hz, "lidar_frequency_hz")?;
        validate_frequency(self.pointcloud_frequency_hz, "pointcloud_frequency_hz")?;
        validate_frequency(self.odometry_frequency_hz, "odometry_frequency_hz")?;
        Ok(())
    }

    pub fn render_args(&self) -> Result<Vec<String>, String> {
        self.validate()?;
        Ok(vec![
            "--lidar".into(),
            self.lidar_topic.clone(),
            "--odometry".into(),
            self.odometry_topic.clone(),
            "--frame_id".into(),
            self.frame_id.clone(),
            "--child_frame_id".into(),
            self.child_frame_id.clone(),
            "--sensor_frame_id".into(),
            self.sensor_frame_id.clone(),
            "--host_ip".into(),
            self.host_ip.clone(),
            "--lidar_ip".into(),
            self.lidar_ip.clone(),
            "--frequency".into(),
            render_number(self.lidar_frequency_hz),
            "--pointcloud_freq".into(),
            render_number(self.pointcloud_frequency_hz),
            "--odom_freq".into(),
            render_number(self.odometry_frequency_hz),
            "--debug".into(),
            self.debug.to_string(),
        ])
    }

    pub fn contract(&self, crate_root: impl AsRef<Path>) -> Result<NativeContractSpec, String> {
        let args = self.render_args()?;
        let source_dir = crate_root.as_ref().join(self.backend.source_relpath());
        Ok(NativeContractSpec {
            source: NativeSourceSpec {
                source_dir: source_dir.clone(),
                source_lock_manifest: crate_root.as_ref().join("native/SOURCE_LOCKS.json"),
                upstream_revision: self.backend.upstream_revision().into(),
                dimos_lcm_revision: DIMOS_LCM_REVISION.into(),
                livox_sdk2_revision: LIVOX_SDK2_REVISION.into(),
                livox_sdk2_source_hash: LIVOX_SDK2_SOURCE_HASH.into(),
            },
            build: NativeBuildSpec {
                cwd: source_dir.clone(),
                command: format!(
                    "nix build --no-write-lock-file --out-link result .#{}",
                    self.backend.package_name()
                ),
            },
            runtime: NativeRuntimeSpec {
                backend: self.backend,
                cwd: source_dir,
                executable: PathBuf::from(self.backend.executable_relpath()),
                args,
                env: BTreeMap::new(),
            },
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NativeSourceSpec {
    pub source_dir: PathBuf,
    pub source_lock_manifest: PathBuf,
    pub upstream_revision: String,
    pub dimos_lcm_revision: String,
    pub livox_sdk2_revision: String,
    pub livox_sdk2_source_hash: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NativeBuildSpec {
    pub cwd: PathBuf,
    pub command: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NativeRuntimeSpec {
    pub backend: NativeBackend,
    pub cwd: PathBuf,
    pub executable: PathBuf,
    pub args: Vec<String>,
    pub env: BTreeMap<String, String>,
}

impl NativeRuntimeSpec {
    pub fn resolved_executable(&self) -> PathBuf {
        if self.executable.is_absolute() {
            self.executable.clone()
        } else {
            self.cwd.join(&self.executable)
        }
    }

    pub fn command(&self) -> Result<Command, String> {
        self.validate()?;
        let mut command = Command::new(self.resolved_executable());
        command.current_dir(&self.cwd).args(&self.args);
        command.envs(&self.env);
        Ok(command)
    }

    pub fn spawn(&self) -> Result<NativeChild, String> {
        let mut active = ACTIVE_NATIVE_BACKEND
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if let Some(owner) = *active {
            return Err(format!(
                "native LIO backend {owner:?} already owns the runtime slot"
            ));
        }
        let child = self
            .command()?
            .spawn()
            .map_err(|error| format!("failed to spawn native LIO process: {error}"))?;
        *active = Some(self.backend);
        Ok(NativeChild {
            child: Some(child),
            backend: Some(self.backend),
        })
    }

    fn validate(&self) -> Result<(), String> {
        if self.cwd.as_os_str().is_empty() {
            return Err("native runtime cwd must be non-empty".into());
        }
        if self.executable.as_os_str().is_empty() {
            return Err("native runtime executable must be non-empty".into());
        }
        if self.args.iter().any(String::is_empty) {
            return Err("native runtime argv must not contain empty values".into());
        }
        if self
            .env
            .iter()
            .any(|(key, value)| key.is_empty() || key.contains('=') || value.contains('\0'))
        {
            return Err("native runtime environment is invalid".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NativeContractSpec {
    pub source: NativeSourceSpec,
    pub build: NativeBuildSpec,
    pub runtime: NativeRuntimeSpec,
}

#[derive(Debug)]
pub struct NativeChild {
    child: Option<Child>,
    backend: Option<NativeBackend>,
}

impl NativeChild {
    pub fn id(&self) -> Option<u32> {
        self.child.as_ref().map(Child::id)
    }

    pub fn try_wait(&mut self) -> Result<Option<ExitStatus>, String> {
        let status = self
            .child
            .as_mut()
            .ok_or_else(|| "native LIO child has already been reaped".to_string())?
            .try_wait()
            .map_err(|error| format!("failed to poll native LIO process: {error}"))?;
        if status.is_some() {
            self.release_owner();
        }
        Ok(status)
    }

    pub fn wait(&mut self) -> Result<ExitStatus, String> {
        let mut child = self
            .child
            .take()
            .ok_or_else(|| "native LIO child has already been reaped".to_string())?;
        let status = child
            .wait()
            .map_err(|error| format!("failed to wait for native LIO process: {error}"));
        self.release_owner();
        status
    }

    pub fn terminate(&mut self) -> Result<ExitStatus, String> {
        let mut child = self
            .child
            .take()
            .ok_or_else(|| "native LIO child has already been reaped".to_string())?;
        let status = child
            .kill()
            .map_err(|error| format!("failed to terminate native LIO process: {error}"))
            .and_then(|()| {
                child
                    .wait()
                    .map_err(|error| format!("failed to reap native LIO process: {error}"))
            });
        self.release_owner();
        status
    }

    fn release_owner(&mut self) {
        let Some(backend) = self.backend.take() else {
            return;
        };
        let mut active = ACTIVE_NATIVE_BACKEND
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if *active == Some(backend) {
            *active = None;
        }
    }
}

impl Drop for NativeChild {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            if !matches!(child.try_wait(), Ok(Some(_))) {
                let _ = child.kill();
                let _ = child.wait();
            }
        }
        self.release_owner();
    }
}

fn validate_typed_topic(topic: &str, message_type: &str, field: &str) -> Result<(), String> {
    if topic.is_empty() || topic.len() > MAX_TOPIC_BYTES {
        return Err(format!("{field} must contain 1..={MAX_TOPIC_BYTES} bytes"));
    }
    if topic
        .chars()
        .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(format!(
            "{field} must not contain whitespace or control characters"
        ));
    }
    let Some((path, actual_type)) = topic.split_once('#') else {
        return Err(format!("{field} must use '<path>#{message_type}'"));
    };
    if !path.starts_with('/') || path.len() < 2 || path.contains('#') {
        return Err(format!("{field} has an invalid topic path"));
    }
    if actual_type != message_type {
        return Err(format!("{field} must use message type {message_type}"));
    }
    Ok(())
}

fn validate_frame(frame: &str, field: &str) -> Result<(), String> {
    if frame.is_empty() || frame.len() > MAX_FRAME_BYTES {
        return Err(format!("{field} must contain 1..={MAX_FRAME_BYTES} bytes"));
    }
    if frame.starts_with('/')
        || frame.ends_with('/')
        || frame.contains("//")
        || frame
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(format!("{field} is not a valid frame ID"));
    }
    Ok(())
}

fn validate_ipv4(value: &str, field: &str) -> Result<Ipv4Addr, String> {
    let ip = value
        .parse::<Ipv4Addr>()
        .map_err(|_| format!("{field} must be a canonical IPv4 address"))?;
    if ip.to_string() != value {
        return Err(format!("{field} must be a canonical IPv4 address"));
    }
    if ip.is_unspecified() || ip.is_loopback() || ip.is_multicast() || ip == Ipv4Addr::BROADCAST {
        return Err(format!("{field} must be a usable unicast IPv4 address"));
    }
    Ok(ip)
}

fn validate_frequency(value: f64, field: &str) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 || value > MAX_FREQUENCY_HZ {
        return Err(format!(
            "{field} must be finite and within (0, {MAX_FREQUENCY_HZ}]"
        ));
    }
    Ok(())
}

fn render_number(value: f64) -> String {
    value.to_string()
}
