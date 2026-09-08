//! Native FastLIO2/Point-LIO preflight, binding, and child-process ownership.

use std::collections::{BTreeMap, VecDeque};
use std::env;
use std::io::Read;
use std::os::fd::AsRawFd;
use std::os::unix::fs::PermissionsExt;
use std::os::unix::process::{CommandExt, ExitStatusExt};
use std::path::{Path, PathBuf};
use std::process::{ExitStatus, Stdio};
use std::sync::Arc;
use std::time::{Duration, Instant};

use nalgebra::{Matrix3, Point3, Vector3};
use nuwax_codec::lcm::lio::{LIO_BACKEND_FASTLIO2, LIO_BACKEND_POINTLIO};
use nuwax_lio_backends::native::{NativeBackend, NativeLioConfig, NativeRuntimeSpec};
use nuwax_protocol::{LcmMessageType, LcmTopic};
use parking_lot::Mutex;
use serde::Deserialize;
use tokio::io::{AsyncRead, AsyncReadExt};
use tokio::process::{Child, Command};
use tokio::task::JoinHandle;

pub(crate) const BODY_FROM_SENSOR_ENV: &str = "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR";
pub(crate) const NATIVE_EXECUTABLE_ENV: &str = "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE";
pub(crate) const PRIVATE_PREFIX_ENV: &str = "NUWAX_GEOMETRY_NATIVE_LIO_PRIVATE_PREFIX";
pub(crate) const FIRST_PAIR_TIMEOUT_MS_ENV: &str =
    "NUWAX_GEOMETRY_NATIVE_LIO_FIRST_PAIR_TIMEOUT_MS";
pub(crate) const PAIR_MAX_SKEW_MS_ENV: &str = "NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS";
const SCAN_VOXEL_SIZE_M_ENV: &str = "NUWAX_GEOMETRY_LIO_SCAN_VOXEL_SIZE_M";
const MAP_VOXEL_SIZE_M_ENV: &str = "NUWAX_GEOMETRY_LIO_MAP_VOXEL_SIZE_M";
const INITIAL_MAP_FRAMES_ENV: &str = "NUWAX_GEOMETRY_LIO_INITIAL_MAP_FRAMES";

const LCM_URL_ENV: &str = "NUWAX_LCM_URL";
const DEFAULT_LCM_URL: &str = "udpm://239.255.76.67:7667?ttl=1";
const DEFAULT_PRIVATE_PREFIX: &str = "/lio";
const DEFAULT_FIRST_PAIR_TIMEOUT_MS: u64 = 10_000;
const DEFAULT_PAIR_MAX_SKEW_MS: u64 = 100;
const MAX_FIRST_PAIR_TIMEOUT_MS: u64 = 300_000;
const MAX_PAIR_MAX_SKEW_MS: u64 = 60_000;
const LCM_MAX_CHANNEL_NAME_BYTES: usize = 63;
// Leaves room for `/f2/<i64 generation>/c#sensor_msgs.PointCloud2`.
const MAX_PRIVATE_PREFIX_BYTES: usize = 14;
const ROTATION_TOLERANCE: f64 = 1.0e-6;
const OUTPUT_TAIL_BYTES: usize = 64 * 1024;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BodyFromSensorJson {
    translation_m: [f64; 3],
    rotation_row_major: [f64; 9],
}

/// Concrete rigid transform required before a native backend may be spawned.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BodyFromSensorTransform {
    data: Box<BodyFromSensorData>,
}

#[derive(Clone, Debug, PartialEq)]
struct BodyFromSensorData {
    translation_m: Vector3<f64>,
    rotation: Matrix3<f64>,
}

impl BodyFromSensorTransform {
    pub(crate) fn from_env() -> Result<Self, String> {
        let raw = required_env(BODY_FROM_SENSOR_ENV)?;
        Self::from_json(&raw)
    }

    pub(crate) fn from_json(raw: &str) -> Result<Self, String> {
        Self::from_json_for_env(raw, BODY_FROM_SENSOR_ENV)
    }

    pub(crate) fn from_json_for_env(raw: &str, env_name: &str) -> Result<Self, String> {
        if raw.is_empty() || raw != raw.trim() {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED env={env_name} value_is_empty_or_padded"
            ));
        }
        let parsed: BodyFromSensorJson = serde_json::from_str(raw).map_err(|error| {
            format!("NATIVE_LIO_PREFLIGHT_FAILED env={env_name} invalid_json: {error}")
        })?;
        if !parsed.translation_m.iter().all(|value| value.is_finite())
            || !parsed
                .rotation_row_major
                .iter()
                .all(|value| value.is_finite())
        {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED env={env_name} values_must_be_finite"
            ));
        }

        let rotation = Matrix3::from_row_slice(&parsed.rotation_row_major);
        let gram = rotation.transpose() * rotation;
        let orthonormal = (0..3).all(|row| {
            (0..3).all(|column| {
                let expected = if row == column { 1.0 } else { 0.0 };
                (gram[(row, column)] - expected).abs() <= ROTATION_TOLERANCE
            })
        });
        let determinant = rotation.determinant();
        if !orthonormal || (determinant - 1.0).abs() > ROTATION_TOLERANCE {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED env={env_name} rotation_must_be_proper_orthonormal determinant={determinant}"
            ));
        }

        Ok(Self {
            data: Box::new(BodyFromSensorData {
                translation_m: Vector3::from_row_slice(&parsed.translation_m),
                rotation,
            }),
        })
    }

    pub(crate) fn transform_point(&self, sensor_point: &Point3<f64>) -> Point3<f64> {
        Point3::from(self.data.rotation * sensor_point.coords + self.data.translation_m)
    }

    pub(crate) fn translation_m(&self) -> [f64; 3] {
        std::array::from_fn(|index| self.data.translation_m[index])
    }

    pub(crate) fn rotation_row_major(&self) -> [f64; 9] {
        std::array::from_fn(|index| self.data.rotation[(index / 3, index % 3)])
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct NativePrivateChannels {
    pub(crate) source_odometry: (LcmTopic, LcmMessageType),
    pub(crate) source_cloud: (LcmTopic, LcmMessageType),
    /// Generation-private canonical raw sensor ingress consumed by the native child.
    pub(crate) input_cloud: (LcmTopic, LcmMessageType),
    pub(crate) input_imu: (LcmTopic, LcmMessageType),
}

#[derive(Clone, Debug)]
pub(crate) struct BoundNativeLioRuntime {
    pub(crate) backend_code: i32,
    pub(crate) channels: NativePrivateChannels,
    pub(crate) runtime: Box<NativeRuntimeSpec>,
    pub(crate) body_from_sensor: BodyFromSensorTransform,
    pub(crate) first_pair_timeout: Duration,
    pub(crate) pair_max_skew: Duration,
}

#[derive(Clone, Debug)]
pub(crate) struct NativeLioPreflight {
    backend: NativeBackend,
    native_config: NativeLioConfig,
    executable: PathBuf,
    lcm_url: String,
    private_prefix: String,
    body_from_sensor: BodyFromSensorTransform,
    first_pair_timeout: Duration,
    pair_max_skew: Duration,
}

impl NativeLioPreflight {
    pub(crate) fn from_env(backend: NativeBackend) -> Result<Self, String> {
        let executable_path = if env::var_os(NATIVE_EXECUTABLE_ENV).is_some() {
            required_env(NATIVE_EXECUTABLE_ENV)?
        } else {
            // 源码构建沿用本地 result/bin；显式配置仍优先，部署搬迁可指定固定路径。
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../nuwax-workspace/crates/nuwax-lio-backends")
                .join(backend.source_relpath())
                .join(backend.executable_relpath())
                .to_string_lossy()
                .into_owned()
        };
        let executable = canonical_executable(&executable_path)?;
        let body_from_sensor = BodyFromSensorTransform::from_env()?;
        let lcm_url = strict_env_or_default(LCM_URL_ENV, DEFAULT_LCM_URL)?;
        validate_lcm_url(&lcm_url)?;
        let private_prefix = strict_env_or_default(PRIVATE_PREFIX_ENV, DEFAULT_PRIVATE_PREFIX)?;
        validate_private_prefix(&private_prefix)?;

        let first_pair_timeout_ms = strict_millis_env(
            FIRST_PAIR_TIMEOUT_MS_ENV,
            DEFAULT_FIRST_PAIR_TIMEOUT_MS,
            MAX_FIRST_PAIR_TIMEOUT_MS,
        )?;
        let pair_max_skew_ms = strict_millis_env(
            PAIR_MAX_SKEW_MS_ENV,
            DEFAULT_PAIR_MAX_SKEW_MS,
            MAX_PAIR_MAX_SKEW_MS,
        )?;
        if pair_max_skew_ms > first_pair_timeout_ms {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED env={PAIR_MAX_SKEW_MS_ENV} value={pair_max_skew_ms} exceeds_first_pair_timeout_ms={first_pair_timeout_ms}"
            ));
        }

        // 与其他预检值一起捕获；后续代际绑定不再读取可能变化的环境。
        let mut native_config = NativeLioConfig::for_backend(backend);
        native_config.scan_voxel_size_m = optional_number_env(SCAN_VOXEL_SIZE_M_ENV)?;
        native_config.map_voxel_size_m = optional_number_env(MAP_VOXEL_SIZE_M_ENV)?;
        native_config.initial_map_frames = optional_number_env(INITIAL_MAP_FRAMES_ENV)?;
        native_config.validate().map_err(|error| {
            format!("NATIVE_LIO_PREFLIGHT_FAILED invalid native sampling config: {error}")
        })?;
        validate_native_backend_identity(&executable, backend, Duration::from_secs(2))?;

        Ok(Self {
            backend,
            native_config,
            executable,
            lcm_url,
            private_prefix,
            body_from_sensor,
            first_pair_timeout: Duration::from_millis(first_pair_timeout_ms),
            pair_max_skew: Duration::from_millis(pair_max_skew_ms),
        })
    }

    pub(crate) fn bind(
        &self,
        runtime_generation: i64,
        sensor_frame: impl Into<String>,
        body_frame: impl Into<String>,
        odom_frame: impl Into<String>,
    ) -> Result<BoundNativeLioRuntime, String> {
        if runtime_generation <= 0 {
            return Err(format!(
                "NATIVE_LIO_BIND_FAILED invalid runtime_generation={runtime_generation}"
            ));
        }

        let backend_tag = backend_tag(self.backend);
        let channel_root = format!("{}/{backend_tag}/{runtime_generation}", self.private_prefix);
        let channels = NativePrivateChannels {
            source_odometry: typed_binding(&format!("{channel_root}/o"), "nav_msgs.Odometry")?,
            source_cloud: typed_binding(&format!("{channel_root}/c"), "sensor_msgs.PointCloud2")?,
            input_cloud: typed_binding(&format!("{channel_root}/ic"), "sensor_msgs.PointCloud2")?,
            input_imu: typed_binding(&format!("{channel_root}/ii"), "sensor_msgs.Imu")?,
        };

        let odometry_pattern = typed_pattern(&channels.source_odometry);
        let cloud_pattern = typed_pattern(&channels.source_cloud);
        let input_cloud_pattern = typed_pattern(&channels.input_cloud);
        let input_imu_pattern = typed_pattern(&channels.input_imu);
        for (stream, pattern) in [
            ("odometry", &odometry_pattern),
            ("cloud", &cloud_pattern),
            ("input_cloud", &input_cloud_pattern),
            ("input_imu", &input_imu_pattern),
        ] {
            if pattern.len() > LCM_MAX_CHANNEL_NAME_BYTES {
                return Err(format!(
                    "NATIVE_LIO_BIND_FAILED private {stream} typed channel exceeds libLCM limit bytes={} maximum={LCM_MAX_CHANNEL_NAME_BYTES}",
                    pattern.len()
                ));
            }
        }

        let mut config = self.native_config.clone();
        config.lidar_topic = cloud_pattern;
        config.odometry_topic = odometry_pattern;
        let input_cloud_pattern = typed_pattern(&channels.input_cloud);
        let input_imu_pattern = typed_pattern(&channels.input_imu);
        config.input_lidar_topic = Some(input_cloud_pattern);
        config.input_imu_topic = Some(input_imu_pattern);
        config.sensor_frame_id = sensor_frame.into();
        config.child_frame_id = body_frame.into();
        config.frame_id = odom_frame.into();
        config.extrinsic_translation_m = self.body_from_sensor.translation_m();
        config.extrinsic_rotation_row_major = self.body_from_sensor.rotation_row_major();
        let args = config
            .render_args()
            .map_err(|error| format!("NATIVE_LIO_BIND_FAILED invalid native config: {error}"))?;

        let cwd = self
            .executable
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .ok_or_else(|| {
                format!(
                    "NATIVE_LIO_BIND_FAILED executable has no parent: {}",
                    self.executable.display()
                )
            })?
            .to_path_buf();
        let mut runtime_env = BTreeMap::new();
        runtime_env.insert("LCM_DEFAULT_URL".to_owned(), self.lcm_url.clone());
        let runtime = NativeRuntimeSpec {
            backend: self.backend,
            cwd,
            executable: self.executable.clone(),
            args,
            env: runtime_env,
        };
        runtime.command().map_err(|error| {
            format!("NATIVE_LIO_BIND_FAILED invalid native runtime spec: {error}")
        })?;

        Ok(BoundNativeLioRuntime {
            backend_code: backend_code(self.backend),
            channels,
            runtime: Box::new(runtime),
            body_from_sensor: self.body_from_sensor.clone(),
            first_pair_timeout: self.first_pair_timeout,
            pair_max_skew: self.pair_max_skew,
        })
    }
}

fn validate_native_backend_identity(
    executable: &Path,
    backend: NativeBackend,
    timeout: Duration,
) -> Result<(), String> {
    use nix::fcntl::{fcntl, FcntlArg, OFlag};
    use nix::sys::signal::{killpg, Signal};
    use nix::unistd::Pid;

    let expected = match backend {
        NativeBackend::FastLio2 => "nuwax-native-lio-v1:fastlio2",
        NativeBackend::PointLio => "nuwax-native-lio-v1:pointlio",
    };
    let failure = |reason: String| format!(
        "NATIVE_LIO_PREFLIGHT_FAILED backend_identity path={} expected={expected} {reason}; rebuild the selected native backend or correct {NATIVE_EXECUTABLE_ENV}",
        executable.display(),
    );
    let mut child = std::process::Command::new(executable)
        .arg("--nuwax-lio-contract")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .process_group(0)
        .spawn()
        .map_err(|error| failure(format!("spawn_failed={error}")))?;
    let result = (|| {
        let mut stdout = child.stdout.take().ok_or_else(|| failure("stdout_missing".into()))?;
        // 未知旧程序可能持续写入或派生子进程；读取容量和等待时间均有界。
        fcntl(stdout.as_raw_fd(), FcntlArg::F_SETFL(OFlag::O_NONBLOCK))
            .map_err(|error| failure(format!("nonblocking_failed={error}")))?;
        let deadline = Instant::now() + timeout;
        let mut output = Vec::new();
        loop {
            let mut buffer = [0u8; 256];
            loop {
                match stdout.read(&mut buffer) {
                    Ok(0) => break,
                    Ok(count) => {
                        if output.len() + count > 256 {
                            return Err(failure("response_too_large".into()));
                        }
                        output.extend_from_slice(&buffer[..count]);
                    }
                    Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(error) => return Err(failure(format!("read_failed={error}"))),
                }
            }
            if let Some(status) = observe_exit_without_reap(child.id()).map_err(|error| failure(format!("wait_failed={error}")))? {
                // 子进程可能刚在读管道之后退出，再读取一次已提交的短响应。
                let mut tail = [0u8; 256];
                if let Ok(count) = stdout.read(&mut tail) {
                    if output.len() + count > 256 {
                        return Err(failure("response_too_large".into()));
                    }
                    output.extend_from_slice(&tail[..count]);
                }
                let actual = String::from_utf8_lossy(&output);
                return if status.success() && actual.trim_end() == expected {
                    Ok(())
                } else {
                    Err(failure(format!("status={status} actual={actual:?}")))
                };
            }
            if Instant::now() >= deadline {
                return Err(failure("probe_timeout".into()));
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    })();
    // 保留组长身份直到整个探针组收到关闭，再回收组长，避免误用已复用的进程号。
    let group_shutdown = killpg(Pid::from_raw(child.id() as i32), Signal::SIGKILL);
    // 显式路径可能是改变进程组的包装器，组关闭失败时仍须终止直接子进程。
    let direct_shutdown = child.kill();
    let reaped = child.wait().map_err(|error| failure(format!("reap_failed={error}")));
    reaped?;
    if let Err(error) = group_shutdown {
        if error != nix::errno::Errno::ESRCH {
            return Err(failure(format!("group_shutdown_failed={error} direct_shutdown={direct_shutdown:?}")));
        }
    }
    result
}

fn backend_tag(backend: NativeBackend) -> &'static str {
    match backend {
        NativeBackend::FastLio2 => "f2",
        NativeBackend::PointLio => "pl",
    }
}

fn backend_code(backend: NativeBackend) -> i32 {
    match backend {
        NativeBackend::FastLio2 => LIO_BACKEND_FASTLIO2,
        NativeBackend::PointLio => LIO_BACKEND_POINTLIO,
    }
}

fn typed_binding(channel: &str, message_type: &str) -> Result<(LcmTopic, LcmMessageType), String> {
    let topic = LcmTopic::new(channel)
        .map_err(|error| format!("NATIVE_LIO_BIND_FAILED invalid private channel: {error}"))?;
    let message_type = LcmMessageType::new(message_type)
        .map_err(|error| format!("NATIVE_LIO_BIND_FAILED invalid private message type: {error}"))?;
    Ok((topic, message_type))
}

fn typed_pattern(binding: &(LcmTopic, LcmMessageType)) -> String {
    binding.0.as_subscription_pattern(binding.1.fqn())
}

fn required_env(key: &str) -> Result<String, String> {
    match env::var(key) {
        Ok(value) if value.is_empty() => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_empty"
        )),
        Ok(value) if value != value.trim() => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_has_surrounding_whitespace"
        )),
        Ok(value) => Ok(value),
        Err(env::VarError::NotPresent) => {
            Err(format!("NATIVE_LIO_PREFLIGHT_FAILED env={key} is_required"))
        }
        Err(env::VarError::NotUnicode(_)) => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_not_unicode"
        )),
    }
}

fn optional_number_env<T: std::str::FromStr>(key: &str) -> Result<Option<T>, String> {
    match env::var(key) {
        Ok(value) if value.trim().is_empty() => Ok(None),
        Ok(value) if value != value.trim() => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_has_surrounding_whitespace"
        )),
        Ok(value) => value.parse().map(Some).map_err(|_| {
            format!("NATIVE_LIO_PREFLIGHT_FAILED env={key} invalid_number")
        }),
        Err(env::VarError::NotPresent) => Ok(None),
        Err(env::VarError::NotUnicode(_)) => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_not_unicode"
        )),
    }
}

fn strict_env_or_default(key: &str, default: &str) -> Result<String, String> {
    match env::var(key) {
        Ok(value) if value.is_empty() => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_empty"
        )),
        Ok(value) if value != value.trim() => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_has_surrounding_whitespace"
        )),
        Ok(value) => Ok(value),
        Err(env::VarError::NotPresent) => Ok(default.to_owned()),
        Err(env::VarError::NotUnicode(_)) => Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_not_unicode"
        )),
    }
}

fn strict_millis_env(key: &str, default: u64, maximum: u64) -> Result<u64, String> {
    let value = match env::var(key) {
        Ok(value) if value.is_empty() || value != value.trim() => {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_empty_or_padded"
            ));
        }
        Ok(value) => value.parse::<u64>().map_err(|error| {
            format!("NATIVE_LIO_PREFLIGHT_FAILED env={key} invalid_millis: {error}")
        })?,
        Err(env::VarError::NotPresent) => default,
        Err(env::VarError::NotUnicode(_)) => {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED env={key} value_is_not_unicode"
            ));
        }
    };
    if value == 0 || value > maximum {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={key} value={value} outside=1..={maximum}"
        ));
    }
    Ok(value)
}

fn canonical_executable(raw: &str) -> Result<PathBuf, String> {
    let configured = Path::new(raw);
    if !configured.is_absolute() {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={NATIVE_EXECUTABLE_ENV} path_must_be_absolute value={raw}"
        ));
    }
    let executable = configured.canonicalize().map_err(|error| {
        format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={NATIVE_EXECUTABLE_ENV} canonicalize_failed path={raw}: {error}"
        )
    })?;
    let metadata = executable.metadata().map_err(|error| {
        format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={NATIVE_EXECUTABLE_ENV} metadata_failed path={}: {error}",
            executable.display()
        )
    })?;
    if !metadata.file_type().is_file() {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={NATIVE_EXECUTABLE_ENV} not_regular_file path={}",
            executable.display()
        ));
    }
    if metadata.permissions().mode() & 0o111 == 0 {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={NATIVE_EXECUTABLE_ENV} not_executable path={}",
            executable.display()
        ));
    }
    Ok(executable)
}

fn validate_lcm_url(value: &str) -> Result<(), String> {
    let valid_scheme = value
        .split_once("://")
        .is_some_and(|(scheme, remainder)| !scheme.is_empty() && !remainder.is_empty());
    if value.len() > 4_096
        || !valid_scheme
        || value
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={LCM_URL_ENV} invalid_lcm_url"
        ));
    }
    Ok(())
}

fn validate_private_prefix(value: &str) -> Result<(), String> {
    let valid_character = |character: char| {
        character.is_ascii_alphanumeric() || matches!(character, '/' | '_' | '-' | '.')
    };
    if value.len() > MAX_PRIVATE_PREFIX_BYTES {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={PRIVATE_PREFIX_ENV} prefix_too_long bytes={} maximum={MAX_PRIVATE_PREFIX_BYTES} for_liblcm_channel_limit={LCM_MAX_CHANNEL_NAME_BYTES}",
            value.len()
        ));
    }
    if value == "/"
        || !value.starts_with('/')
        || value.ends_with('/')
        || value.contains("//")
        || !value.chars().all(valid_character)
    {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED env={PRIVATE_PREFIX_ENV} invalid_private_prefix"
        ));
    }
    Ok(())
}

#[derive(Debug)]
struct BoundedTail {
    bytes: VecDeque<u8>,
    capacity: usize,
}

impl BoundedTail {
    fn new(capacity: usize) -> Self {
        Self {
            bytes: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    fn extend(&mut self, chunk: &[u8]) {
        if chunk.len() >= self.capacity {
            self.bytes.clear();
            self.bytes
                .extend(chunk[chunk.len() - self.capacity..].iter().copied());
            return;
        }
        let overflow = self
            .bytes
            .len()
            .saturating_add(chunk.len())
            .saturating_sub(self.capacity);
        self.bytes.drain(..overflow);
        self.bytes.extend(chunk.iter().copied());
    }

    #[allow(dead_code)]
    fn snapshot(&self) -> Vec<u8> {
        self.bytes.iter().copied().collect()
    }
}

type SharedTail = Arc<Mutex<BoundedTail>>;
type DrainTask = JoinHandle<Result<(), String>>;

/// Tokio owner for one already-bound native LIO runtime.
#[derive(Debug)]
pub(crate) struct NativeProcessOwner {
    child: Option<Child>,
    pid: u32,
    process_group_active: bool,
    group_cleanup_confirmed: bool,
    observed_status: Option<ExitStatus>,
    reaped_status: Option<ExitStatus>,
    #[allow(dead_code)]
    stdout_tail: SharedTail,
    #[allow(dead_code)]
    stderr_tail: SharedTail,
    stdout_drain: Option<DrainTask>,
    stderr_drain: Option<DrainTask>,
}

impl NativeProcessOwner {
    pub(crate) fn spawn(runtime: &NativeRuntimeSpec) -> Result<Self, String> {
        let mut command = runtime.command().map_err(|error| {
            format!("NATIVE_LIO_PROCESS_FAILED invalid runtime command: {error}")
        })?;
        command.process_group(0);
        let mut command = Command::from(command);
        command.stdin(Stdio::null());
        command.stdout(Stdio::piped());
        command.stderr(Stdio::piped());
        command.kill_on_drop(true);
        let mut child = command.spawn().map_err(|error| {
            format!(
                "NATIVE_LIO_PROCESS_FAILED spawn backend={:?}: {error}",
                runtime.backend
            )
        })?;
        let pid = child.id().ok_or_else(|| {
            "NATIVE_LIO_PROCESS_FAILED spawned child did not expose a pid".to_owned()
        })?;
        let stdout = child.stdout.take().ok_or_else(|| {
            "NATIVE_LIO_PROCESS_FAILED child stdout pipe was not created".to_owned()
        })?;
        let stderr = child.stderr.take().ok_or_else(|| {
            "NATIVE_LIO_PROCESS_FAILED child stderr pipe was not created".to_owned()
        })?;
        let stdout_tail = Arc::new(Mutex::new(BoundedTail::new(OUTPUT_TAIL_BYTES)));
        let stderr_tail = Arc::new(Mutex::new(BoundedTail::new(OUTPUT_TAIL_BYTES)));
        let stdout_drain = spawn_output_drain("stdout", stdout, stdout_tail.clone());
        let stderr_drain = spawn_output_drain("stderr", stderr, stderr_tail.clone());

        Ok(Self {
            child: Some(child),
            pid,
            process_group_active: true,
            group_cleanup_confirmed: false,
            observed_status: None,
            reaped_status: None,
            stdout_tail,
            stderr_tail,
            stdout_drain: Some(stdout_drain),
            stderr_drain: Some(stderr_drain),
        })
    }

    #[allow(dead_code)]
    pub(crate) fn pid(&self) -> u32 {
        self.pid
    }

    #[allow(dead_code)]
    pub(crate) fn stdout_tail(&self) -> Vec<u8> {
        self.stdout_tail.lock().snapshot()
    }

    #[allow(dead_code)]
    pub(crate) fn stderr_tail(&self) -> Vec<u8> {
        self.stderr_tail.lock().snapshot()
    }

    /// Returns `Some` for every exit; a native backend is expected to remain alive.
    #[allow(dead_code)]
    pub(crate) fn poll_unexpected_exit(&mut self) -> Result<Option<ExitStatus>, String> {
        if let Some(status) = self.reaped_status.or(self.observed_status) {
            return Ok(Some(status));
        }
        if self.child.is_none() {
            return Err("NATIVE_LIO_PROCESS_FAILED child ownership missing before poll".to_owned());
        }
        let status = observe_exit_without_reap(self.pid)?;
        if let Some(status) = status {
            self.observed_status = Some(status);
        }
        Ok(status)
    }

    /// Waits for an unexpected terminal status without treating exit zero as healthy.
    pub(crate) async fn wait_unexpected_exit(&mut self) -> Result<ExitStatus, String> {
        loop {
            if let Some(status) = self.poll_unexpected_exit()? {
                return Ok(status);
            }
            tokio::time::sleep(EXIT_STATUS_POLL_INTERVAL).await;
        }
    }

    /// Sends TERM, then KILLs and reaps within caller-supplied bounds.
    pub(crate) async fn shutdown(
        &mut self,
        term_grace: Duration,
        force_reap_timeout: Duration,
    ) -> Result<ExitStatus, String> {
        let shutdown_budget = term_grace.checked_add(force_reap_timeout).ok_or_else(|| {
            "NATIVE_LIO_PROCESS_FAILED shutdown duration budget overflow".to_owned()
        })?;
        let shutdown_deadline = tokio::time::Instant::now()
            .checked_add(shutdown_budget)
            .ok_or_else(|| "NATIVE_LIO_PROCESS_FAILED shutdown deadline overflow".to_owned())?;
        let term_deadline = shutdown_deadline
            .checked_sub(force_reap_timeout)
            .ok_or_else(|| "NATIVE_LIO_PROCESS_FAILED TERM deadline underflow".to_owned())?;
        if let Some(status) = self.reaped_status {
            if !self.group_cleanup_confirmed {
                if !wait_for_process_group_exit(self.pid, shutdown_deadline).await? {
                    return Err(format!(
                        "NATIVE_LIO_PROCESS_FAILED process_group={} remained after group KILL and leader reap",
                        self.pid
                    ));
                }
                self.group_cleanup_confirmed = true;
            }
            self.finish_drains(shutdown_deadline).await?;
            return Ok(status);
        }
        let mut child = self.child.take().ok_or_else(|| {
            "NATIVE_LIO_PROCESS_FAILED child ownership missing before shutdown".to_owned()
        })?;

        let mut status = self.observed_status;
        let mut term_signal_error = None;
        let mut group_kill_error = None;
        let mut child_kill_error = None;
        if self.process_group_active {
            term_signal_error = send_sigterm(self.pid).err();
            if status.is_none() {
                match wait_for_observed_exit(self.pid, term_deadline).await {
                    Ok(observed) => status = observed,
                    Err(error) => {
                        self.child = Some(child);
                        return Err(error);
                    }
                }
            }
            if status.is_some() && tokio::time::Instant::now() < term_deadline {
                // Keep the exited leader unreaped for the full group grace. Its
                // zombie PID pins the PGID so no unrelated group can reuse it.
                tokio::time::sleep_until(term_deadline).await;
            }

            group_kill_error =
                send_process_group_signal(self.pid, nix::sys::signal::Signal::SIGKILL).err();
            if group_kill_error.is_some() {
                self.child = Some(child);
                return Err(format!(
                    "NATIVE_LIO_PROCESS_FAILED group kill failed before leader reap pid={}; term_signal={}; group_kill={}",
                    self.pid,
                    term_signal_error.as_deref().unwrap_or("sent"),
                    group_kill_error.as_deref().unwrap_or("sent")
                ));
            }
            if status.is_none() {
                child_kill_error = child.start_kill().err();
            }
            // No group signal is allowed after this point: successful group KILL
            // has terminated every inheriting descendant while the leader still
            // pins the numeric PGID.
            self.process_group_active = false;
        }

        if status.is_none() {
            status = match wait_for_observed_exit(self.pid, shutdown_deadline).await {
                Ok(status) => status,
                Err(error) => {
                    self.child = Some(child);
                    return Err(error);
                }
            };
        }
        let observed = match status {
            Some(status) => status,
            None => {
                self.child = Some(child);
                return Err(format!(
                    "NATIVE_LIO_PROCESS_FAILED leader pid={} exceeded_shutdown_deadline; term_signal={}; group_kill={}; child_kill={}",
                    self.pid,
                    term_signal_error.as_deref().unwrap_or("sent"),
                    group_kill_error.as_deref().unwrap_or("sent"),
                    child_kill_error
                        .as_ref()
                        .map(ToString::to_string)
                        .unwrap_or_else(|| "sent".to_owned())
                ));
            }
        };

        let reaped = match child.try_wait() {
            Ok(Some(status)) => status,
            Ok(None) => match tokio::time::timeout_at(shutdown_deadline, child.wait()).await {
                Ok(Ok(status)) => status,
                Ok(Err(error)) => {
                    self.child = Some(child);
                    return Err(format!(
                        "NATIVE_LIO_PROCESS_FAILED reap pid={}: {error}",
                        self.pid
                    ));
                }
                Err(_) => {
                    self.child = Some(child);
                    return Err(format!(
                        "NATIVE_LIO_PROCESS_FAILED reap pid={} exceeded_shutdown_deadline",
                        self.pid
                    ));
                }
            },
            Err(error) => {
                self.child = Some(child);
                return Err(format!(
                    "NATIVE_LIO_PROCESS_FAILED pre-reap poll pid={}: {error}",
                    self.pid
                ));
            }
        };

        self.observed_status = Some(observed);
        self.reaped_status = Some(reaped);
        if reaped != observed {
            return Err(format!(
                "NATIVE_LIO_PROCESS_FAILED observed/reaped status mismatch pid={}: observed={observed} reaped={reaped}",
                self.pid
            ));
        }
        self.observed_status = Some(reaped);
        if !wait_for_process_group_exit(self.pid, shutdown_deadline).await? {
            return Err(format!(
                "NATIVE_LIO_PROCESS_FAILED process_group={} remained after group KILL and leader reap",
                self.pid
            ));
        }
        self.group_cleanup_confirmed = true;
        self.finish_drains(shutdown_deadline).await?;
        Ok(reaped)
    }

    async fn finish_drains(&mut self, deadline: tokio::time::Instant) -> Result<(), String> {
        let mut stdout = self.stdout_drain.take();
        let mut stderr = self.stderr_drain.take();
        let joined = tokio::time::timeout_at(deadline, async {
            tokio::join!(
                await_drain("stdout", &mut stdout),
                await_drain("stderr", &mut stderr)
            )
        })
        .await;
        match joined {
            Ok((Ok(()), Ok(()))) => Ok(()),
            Ok((stdout_result, stderr_result)) => {
                let errors = [stdout_result.err(), stderr_result.err()]
                    .into_iter()
                    .flatten()
                    .collect::<Vec<_>>()
                    .join("; ");
                Err(errors)
            }
            Err(_) => {
                abort_drains(&mut stdout, &mut stderr);
                Err("NATIVE_LIO_PROCESS_FAILED output drains exceeded shutdown deadline".to_owned())
            }
        }
    }
}

impl Drop for NativeProcessOwner {
    fn drop(&mut self) {
        if self.process_group_active {
            let _ = send_process_group_signal(self.pid, nix::sys::signal::Signal::SIGKILL);
        }
        if let Some(task) = self.stdout_drain.take() {
            task.abort();
        }
        if let Some(task) = self.stderr_drain.take() {
            task.abort();
        }
        // Group KILL covers forked descendants and Child::kill_on_drop covers the
        // direct child if it escaped the group. Only shutdown reports confirmed reap.
    }
}

fn send_sigterm(pid: u32) -> Result<(), String> {
    send_process_group_signal(pid, nix::sys::signal::Signal::SIGTERM)
}

fn send_process_group_signal(pid: u32, signal: nix::sys::signal::Signal) -> Result<(), String> {
    let raw_pid = i32::try_from(pid)
        .map_err(|_| format!("NATIVE_LIO_PROCESS_FAILED pid={pid} exceeds platform pid range"))?;
    if raw_pid == 0 {
        return Err("NATIVE_LIO_PROCESS_FAILED pid=0 is not a child process group".to_owned());
    }
    match nix::sys::signal::kill(nix::unistd::Pid::from_raw(-raw_pid), signal) {
        Ok(()) | Err(nix::errno::Errno::ESRCH) => Ok(()),
        Err(error) => Err(format!(
            "NATIVE_LIO_PROCESS_FAILED {signal:?} process_group={pid}: {error}"
        )),
    }
}

fn process_group_exists(pid: u32) -> Result<bool, String> {
    let raw_pid = i32::try_from(pid)
        .map_err(|_| format!("NATIVE_LIO_PROCESS_FAILED pid={pid} exceeds platform pid range"))?;
    if raw_pid == 0 {
        return Err("NATIVE_LIO_PROCESS_FAILED pid=0 is not a child process group".to_owned());
    }
    match nix::sys::signal::kill(nix::unistd::Pid::from_raw(-raw_pid), None) {
        Ok(()) | Err(nix::errno::Errno::EPERM) => Ok(true),
        Err(nix::errno::Errno::ESRCH) => Ok(false),
        Err(error) => Err(format!(
            "NATIVE_LIO_PROCESS_FAILED probe process_group={pid}: {error}"
        )),
    }
}

const EXIT_STATUS_POLL_INTERVAL: Duration = Duration::from_millis(5);

fn observe_exit_without_reap(pid: u32) -> Result<Option<ExitStatus>, String> {
    use nix::sys::wait::{waitid, Id, WaitPidFlag, WaitStatus};

    let raw_pid = i32::try_from(pid)
        .map_err(|_| format!("NATIVE_LIO_PROCESS_FAILED pid={pid} exceeds platform pid range"))?;
    let flags = WaitPidFlag::WEXITED | WaitPidFlag::WNOHANG | WaitPidFlag::WNOWAIT;
    let status = waitid(Id::Pid(nix::unistd::Pid::from_raw(raw_pid)), flags).map_err(|error| {
        format!("NATIVE_LIO_PROCESS_FAILED observe_without_reap pid={pid}: {error}")
    })?;
    match status {
        WaitStatus::StillAlive => Ok(None),
        WaitStatus::Exited(_, code) => Ok(Some(ExitStatus::from_raw(code << 8))),
        WaitStatus::Signaled(_, signal, core_dumped) => {
            let core_dump_bit = if core_dumped { 0x80 } else { 0 };
            Ok(Some(ExitStatus::from_raw(signal as i32 | core_dump_bit)))
        }
        other => Err(format!(
            "NATIVE_LIO_PROCESS_FAILED unexpected waitid status pid={pid}: {other:?}"
        )),
    }
}

async fn wait_for_observed_exit(
    pid: u32,
    deadline: tokio::time::Instant,
) -> Result<Option<ExitStatus>, String> {
    loop {
        if let Some(status) = observe_exit_without_reap(pid)? {
            return Ok(Some(status));
        }
        let now = tokio::time::Instant::now();
        if now >= deadline {
            return Ok(None);
        }
        tokio::time::sleep_until(std::cmp::min(deadline, now + EXIT_STATUS_POLL_INTERVAL)).await;
    }
}

async fn wait_for_process_group_exit(
    pid: u32,
    deadline: tokio::time::Instant,
) -> Result<bool, String> {
    loop {
        if !process_group_exists(pid)? {
            return Ok(true);
        }
        let now = tokio::time::Instant::now();
        if now >= deadline {
            return Ok(false);
        }
        tokio::time::sleep_until(std::cmp::min(deadline, now + EXIT_STATUS_POLL_INTERVAL)).await;
    }
}

fn spawn_output_drain<R>(label: &'static str, mut reader: R, tail: SharedTail) -> DrainTask
where
    R: AsyncRead + Unpin + Send + 'static,
{
    tokio::spawn(async move {
        let mut buffer = [0_u8; 8 * 1024];
        loop {
            let read = reader
                .read(&mut buffer)
                .await
                .map_err(|error| format!("NATIVE_LIO_PROCESS_FAILED drain {label}: {error}"))?;
            if read == 0 {
                return Ok(());
            }
            tail.lock().extend(&buffer[..read]);
        }
    })
}

async fn await_drain(label: &str, task: &mut Option<DrainTask>) -> Result<(), String> {
    let Some(task) = task.as_mut() else {
        return Ok(());
    };
    match task.await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(error),
        Err(error) => Err(format!(
            "NATIVE_LIO_PROCESS_FAILED {label} drain task: {error}"
        )),
    }
}

fn abort_drains(stdout: &mut Option<DrainTask>, stderr: &mut Option<DrainTask>) {
    if let Some(task) = stdout.take() {
        task.abort();
    }
    if let Some(task) = stderr.take() {
        task.abort();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::OsString;
    use std::sync::{Mutex as StdMutex, MutexGuard};

    static ENV_LOCK: StdMutex<()> = StdMutex::new(());

    struct EnvRestore {
        values: Vec<(&'static str, Option<OsString>)>,
    }

    impl EnvRestore {
        fn set(values: &[(&'static str, Option<&str>)]) -> (MutexGuard<'static, ()>, Self) {
            let lock = ENV_LOCK
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());
            let previous = values
                .iter()
                .map(|(key, _)| (*key, env::var_os(key)))
                .collect();
            for (key, value) in values {
                match value {
                    Some(value) => env::set_var(key, value),
                    None => env::remove_var(key),
                }
            }
            (lock, Self { values: previous })
        }
    }

    impl Drop for EnvRestore {
        fn drop(&mut self) {
            for (key, value) in self.values.drain(..) {
                match value {
                    Some(value) => env::set_var(key, value),
                    None => env::remove_var(key),
                }
            }
        }
    }

    fn identity_json() -> &'static str {
        r#"{"translation_m":[1.0,2.0,3.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#
    }

    fn rotated_json() -> &'static str {
        r#"{"translation_m":[1.0,2.0,3.0],"rotation_row_major":[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]}"#
    }

    struct ProbeExecutable(PathBuf);

    impl ProbeExecutable {
        fn new(script: &str) -> Self {
            static NEXT_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let directory = env::temp_dir().join(format!(
                "nuwax-native-probe-{}-{}",
                std::process::id(),
                NEXT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
            ));
            std::fs::create_dir(&directory).unwrap();
            let path = directory.join("renamed-program");
            std::fs::write(&path, format!("#!/bin/sh\n{script}\n")).unwrap();
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
            Self(path)
        }

        fn for_backend(backend: NativeBackend) -> Self {
            let tag = match backend {
                NativeBackend::FastLio2 => "fastlio2",
                NativeBackend::PointLio => "pointlio",
            };
            Self::new(&format!("[ \"$1\" = --nuwax-lio-contract ] || exit 9\nprintf 'nuwax-native-lio-v1:{tag}\\n'"))
        }
    }

    impl Drop for ProbeExecutable {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
            let _ = std::fs::remove_dir(self.0.parent().unwrap());
        }
    }

    fn preflight(backend: NativeBackend) -> NativeLioPreflight {
        let probe = ProbeExecutable::for_backend(backend);
        let shell = probe.0.to_str().unwrap();
        let (_lock, _env) = EnvRestore::set(&[
            (NATIVE_EXECUTABLE_ENV, Some(shell)),
            (BODY_FROM_SENSOR_ENV, Some(rotated_json())),
            (LCM_URL_ENV, Some("memq://native-lio-test")),
            (PRIVATE_PREFIX_ENV, Some("/g13")),
            (FIRST_PAIR_TIMEOUT_MS_ENV, Some("1234")),
            (PAIR_MAX_SKEW_MS_ENV, Some("25")),
            (SCAN_VOXEL_SIZE_M_ENV, None),
            (MAP_VOXEL_SIZE_M_ENV, None),
            (INITIAL_MAP_FRAMES_ENV, None),
        ]);
        NativeLioPreflight::from_env(backend).expect("native preflight")
    }

    #[test]
    fn parses_and_applies_proper_body_from_sensor_rotation() {
        let transform = BodyFromSensorTransform::from_json(rotated_json()).expect("transform");
        assert_eq!(
            transform.transform_point(&Point3::new(4.0, 5.0, 6.0)),
            Point3::new(-4.0, 6.0, 9.0)
        );

        let reflection = r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#;
        assert!(BodyFromSensorTransform::from_json(reflection).is_err());
        let scaled = r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[2.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#;
        assert!(BodyFromSensorTransform::from_json(scaled).is_err());
    }

    #[test]
    fn executable_artifact_validation_fails_closed() {
        let (_lock, _env) = EnvRestore::set(&[
            (NATIVE_EXECUTABLE_ENV, Some("relative/native-lio")),
            (BODY_FROM_SENSOR_ENV, Some(identity_json())),
        ]);
        let error = NativeLioPreflight::from_env(NativeBackend::FastLio2)
            .expect_err("relative artifact must fail");
        assert!(error.contains("path_must_be_absolute"));
    }

    #[test]
    fn generation_and_backend_scope_private_typed_channels() {
        let fast = preflight(NativeBackend::FastLio2)
            .bind(41, "sensor", "body", "odom")
            .expect("fast bind");
        let point = preflight(NativeBackend::PointLio)
            .bind(42, "sensor", "body", "odom")
            .expect("point bind");

        assert_eq!(fast.backend_code, LIO_BACKEND_FASTLIO2);
        assert_eq!(point.backend_code, LIO_BACKEND_POINTLIO);
        assert_eq!(fast.channels.source_cloud.0.channel(), "/g13/f2/41/c");
        assert_eq!(point.channels.source_odometry.0.channel(), "/g13/pl/42/o");
        assert_eq!(
            fast.channels.source_cloud.1.fqn(),
            "sensor_msgs.PointCloud2"
        );
        assert_eq!(fast.channels.source_odometry.1.fqn(), "nav_msgs.Odometry");
        assert!(typed_pattern(&fast.channels.source_cloud).len() <= LCM_MAX_CHANNEL_NAME_BYTES);
        assert!(typed_pattern(&point.channels.source_odometry).len() <= LCM_MAX_CHANNEL_NAME_BYTES);
        assert_ne!(fast.channels, point.channels);
    }

    #[test]
    fn private_prefix_reserves_the_complete_liblcm_channel_budget() {
        {
            let shell = Path::new("/bin/sh")
                .canonicalize()
                .expect("canonical shell");
            let shell = shell.to_str().expect("UTF-8 shell path");
            let (_lock, _env) = EnvRestore::set(&[
                (NATIVE_EXECUTABLE_ENV, Some(shell)),
                (BODY_FROM_SENSOR_ENV, Some(identity_json())),
                (PRIVATE_PREFIX_ENV, Some("/12345678901234")),
            ]);
            let error = NativeLioPreflight::from_env(NativeBackend::FastLio2)
                .expect_err("15-byte prefix must fail before startup side effects");
            assert!(error.contains("prefix_too_long"), "{error}");
        }

        let bound = preflight(NativeBackend::PointLio)
            .bind(i64::MAX, "sensor", "body", "odom")
            .expect("maximum generation must fit libLCM");
        assert_eq!(
            typed_pattern(&bound.channels.source_cloud).len(),
            LCM_MAX_CHANNEL_NAME_BYTES - (MAX_PRIVATE_PREFIX_BYTES - "/g13".len())
        );
    }

    #[test]
    fn bind_renders_native_argv_and_lcm_environment() {
        let bound = preflight(NativeBackend::FastLio2)
            .bind(7, "mid360", "base", "odom")
            .expect("bound runtime");
        assert_eq!(
            bound.runtime.env.get("LCM_DEFAULT_URL").map(String::as_str),
            Some("memq://native-lio-test")
        );
        assert_eq!(bound.first_pair_timeout, Duration::from_millis(1234));
        assert_eq!(bound.pair_max_skew, Duration::from_millis(25));
        assert_eq!(
            bound
                .body_from_sensor
                .transform_point(&Point3::new(0.0, 0.0, 0.0)),
            Point3::new(1.0, 2.0, 3.0)
        );
        let args = &bound.runtime.args;
        assert!(args
            .windows(2)
            .any(|pair| { pair == ["--lidar", "/g13/f2/7/c#sensor_msgs.PointCloud2",] }));
        assert!(args.windows(2).any(|pair| pair == ["--frame_id", "odom"]));
        assert!(args
            .windows(2)
            .any(|pair| pair == ["--child_frame_id", "base"]));
        assert!(args
            .windows(2)
            .any(|pair| pair == ["--sensor_frame_id", "mid360"]));
        assert!(args
            .windows(2)
            .any(|pair| pair == ["--extrinsic_t", "1,2,3"]));
        assert!(args
            .windows(2)
            .any(|pair| pair == ["--extrinsic_r", "0,-1,0,1,0,0,0,0,1"]));
    }

    #[test]
    fn optional_sampling_is_captured_before_generation_binding() {
        let shell = Path::new("/bin/sh").canonicalize().unwrap();
        let (_lock, _env) = EnvRestore::set(&[
            (NATIVE_EXECUTABLE_ENV, shell.to_str()),
            (BODY_FROM_SENSOR_ENV, Some(identity_json())),
            (LCM_URL_ENV, Some("memq://native-sampling-test")),
            (PRIVATE_PREFIX_ENV, Some("/sample")),
            (FIRST_PAIR_TIMEOUT_MS_ENV, Some("1000")),
            (PAIR_MAX_SKEW_MS_ENV, Some("100")),
            (SCAN_VOXEL_SIZE_M_ENV, Some("0.1")),
            (MAP_VOXEL_SIZE_M_ENV, Some("0.2")),
            (INITIAL_MAP_FRAMES_ENV, Some("7")),
        ]);
        let captured = [NativeBackend::FastLio2, NativeBackend::PointLio]
            .map(|backend| {
                let probe = ProbeExecutable::for_backend(backend);
                env::set_var(NATIVE_EXECUTABLE_ENV, &probe.0);
                NativeLioPreflight::from_env(backend).unwrap()
            });
        env::set_var(SCAN_VOXEL_SIZE_M_ENV, "invalid-after-preflight");
        env::set_var(MAP_VOXEL_SIZE_M_ENV, "0");
        env::set_var(INITIAL_MAP_FRAMES_ENV, "9");
        for (index, preflight) in captured.into_iter().enumerate() {
            let bound = preflight.bind(index as i64 + 1, "sensor", "body", "odom").unwrap();
            for pair in [["--filter_size_surf", "0.1"], ["--filter_size_map", "0.2"], ["--initial_map_frames", "7"]] {
                assert!(bound.runtime.args.windows(2).any(|args| args == pair));
            }
        }
        assert!(NativeLioPreflight::from_env(NativeBackend::PointLio).is_err());
    }

    #[test]
    fn optional_sampling_empty_values_preserve_native_defaults() {
        let probe = ProbeExecutable::for_backend(NativeBackend::PointLio);
        let shell = &probe.0;
        for value in [None, Some(""), Some("   ")] {
            let (_lock, _env) = EnvRestore::set(&[
                (NATIVE_EXECUTABLE_ENV, shell.to_str()),
                (BODY_FROM_SENSOR_ENV, Some(identity_json())),
                (LCM_URL_ENV, Some("memq://native-sampling-test")),
                (PRIVATE_PREFIX_ENV, Some("/sample")),
                (FIRST_PAIR_TIMEOUT_MS_ENV, Some("1000")),
                (PAIR_MAX_SKEW_MS_ENV, Some("100")),
                (SCAN_VOXEL_SIZE_M_ENV, value),
                (MAP_VOXEL_SIZE_M_ENV, value),
                (INITIAL_MAP_FRAMES_ENV, value),
            ]);
            let bound = NativeLioPreflight::from_env(NativeBackend::PointLio).unwrap()
                .bind(1, "sensor", "body", "odom").unwrap();
            for option in ["--filter_size_surf", "--filter_size_map", "--initial_map_frames"] {
                assert!(!bound.runtime.args.iter().any(|arg| arg == option));
            }
        }
    }

    #[test]
    fn optional_sampling_rejects_invalid_environment_values() {
        let shell = Path::new("/bin/sh").canonicalize().unwrap();
        let (_lock, _env) = EnvRestore::set(&[
            (NATIVE_EXECUTABLE_ENV, shell.to_str()),
            (BODY_FROM_SENSOR_ENV, Some(identity_json())),
            (LCM_URL_ENV, Some("memq://native-sampling-test")),
            (PRIVATE_PREFIX_ENV, Some("/sample")),
            (FIRST_PAIR_TIMEOUT_MS_ENV, Some("1000")),
            (PAIR_MAX_SKEW_MS_ENV, Some("100")),
            (SCAN_VOXEL_SIZE_M_ENV, None),
            (MAP_VOXEL_SIZE_M_ENV, None),
            (INITIAL_MAP_FRAMES_ENV, None),
        ]);
        for key in [SCAN_VOXEL_SIZE_M_ENV, MAP_VOXEL_SIZE_M_ENV] {
            for value in ["0", "-1", "NaN", "inf", "1e-40", "1e40", "0.1tail", " 0.1 "] {
                env::set_var(key, value);
                assert!(NativeLioPreflight::from_env(NativeBackend::PointLio).is_err(), "{key}={value}");
            }
            env::remove_var(key);
        }
        for value in ["0", "-1", "1.5", "2147483648", "7tail"] {
            env::set_var(INITIAL_MAP_FRAMES_ENV, value);
            assert!(NativeLioPreflight::from_env(NativeBackend::FastLio2).is_err(), "initial frames={value}");
        }
    }

    #[test]
    #[ignore = "由临时配置保存和共同环境加载专项驱动，不启动原生进程"]
    fn saved_sampling_configuration_reaches_actual_bind() {
        let shell = Path::new("/bin/sh").canonicalize().unwrap();
        let (_lock, _env) = EnvRestore::set(&[
            (NATIVE_EXECUTABLE_ENV, shell.to_str()),
            (BODY_FROM_SENSOR_ENV, Some(identity_json())),
            (LCM_URL_ENV, Some("memq://saved-native-config")),
            (PRIVATE_PREFIX_ENV, Some("/saved")),
            (FIRST_PAIR_TIMEOUT_MS_ENV, Some("1000")),
            (PAIR_MAX_SKEW_MS_ENV, Some("100")),
        ]);
        // 三个采样值故意不在本测试写入，只接收真实设置保存/环境加载的输出。
        for backend in [NativeBackend::FastLio2, NativeBackend::PointLio] {
            let probe = ProbeExecutable::for_backend(backend);
            env::set_var(NATIVE_EXECUTABLE_ENV, &probe.0);
            let bound = NativeLioPreflight::from_env(backend).unwrap()
                .bind(1, "sensor", "body", "odom").unwrap();
            for pair in [["--filter_size_surf", "0.1"], ["--filter_size_map", "0.2"], ["--initial_map_frames", "7"]] {
                assert!(bound.runtime.args.windows(2).any(|args| args == pair));
            }
            println!("NATIVE_CONFIG_BIND_JSON {}", serde_json::json!({"backend": backend, "args": bound.runtime.args}));
        }
    }

    #[test]
    fn native_backend_identity_accepts_renaming_and_rejects_mismatch_or_old_binary() {
        for backend in [NativeBackend::FastLio2, NativeBackend::PointLio] {
            let probe = ProbeExecutable::for_backend(backend);
            validate_native_backend_identity(&probe.0, backend, Duration::from_secs(1)).unwrap();
            let other = match backend {
                NativeBackend::FastLio2 => NativeBackend::PointLio,
                NativeBackend::PointLio => NativeBackend::FastLio2,
            };
            let error = validate_native_backend_identity(&probe.0, other, Duration::from_secs(1)).unwrap_err();
            assert!(error.contains("backend_identity") && error.contains("actual="), "{error}");
        }
        let old = ProbeExecutable::new("exit 0");
        assert!(validate_native_backend_identity(&old.0, NativeBackend::PointLio, Duration::from_secs(1)).is_err());
    }

    #[test]
    fn native_backend_identity_probe_has_bounded_output_and_wait() {
        let noisy = ProbeExecutable::new("while :; do printf 'unexpected-output'; done");
        let error = validate_native_backend_identity(&noisy.0, NativeBackend::PointLio, Duration::from_secs(1)).unwrap_err();
        assert!(error.contains("response_too_large"), "{error}");
        let hanging = ProbeExecutable::new("sleep 30");
        let started = Instant::now();
        let error = validate_native_backend_identity(&hanging.0, NativeBackend::PointLio, Duration::from_millis(30)).unwrap_err();
        assert!(error.contains("probe_timeout"), "{error}");
        assert!(started.elapsed() < Duration::from_secs(1));
    }

    fn shell_runtime(script: &str) -> NativeRuntimeSpec {
        NativeRuntimeSpec {
            backend: NativeBackend::FastLio2,
            cwd: PathBuf::from("/"),
            executable: PathBuf::from("/bin/sh"),
            args: vec!["-c".to_owned(), script.to_owned()],
            env: BTreeMap::new(),
        }
    }

    async fn wait_for_stdout(owner: &NativeProcessOwner, expected: &str) {
        let deadline = tokio::time::Instant::now() + Duration::from_secs(1);
        while !String::from_utf8_lossy(&owner.stdout_tail()).contains(expected) {
            assert!(
                tokio::time::Instant::now() < deadline,
                "child did not publish readiness marker"
            );
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn shutdown_uses_term_and_reaps_child() {
        let runtime = shell_runtime("trap 'exit 0' TERM; printf 'ready\\n'; while :; do :; done");
        let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn shell");
        assert!(owner.pid() > 0);
        wait_for_stdout(&owner, "ready").await;
        let status = owner
            .shutdown(Duration::from_secs(1), Duration::from_secs(1))
            .await
            .expect("TERM shutdown");
        assert!(status.success(), "TERM trap status: {status}");
        assert!(owner
            .poll_unexpected_exit()
            .expect("post-reap poll")
            .is_some());

        // A cached shutdown must not probe or signal the numeric PGID again.
        // An out-of-range replacement makes any such regression deterministic.
        owner.pid = u32::MAX;
        let repeated = owner
            .shutdown(Duration::ZERO, Duration::ZERO)
            .await
            .expect("cached shutdown");
        assert_eq!(repeated, status);
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn shutdown_falls_back_to_kill_and_reaps_child() {
        use std::os::unix::process::ExitStatusExt;

        let runtime = shell_runtime("trap '' TERM; printf 'ready\\n'; while :; do :; done");
        let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn shell");
        wait_for_stdout(&owner, "ready").await;
        let status = owner
            .shutdown(Duration::from_millis(25), Duration::from_secs(1))
            .await
            .expect("KILL fallback shutdown");
        assert_eq!(status.signal(), Some(nix::libc::SIGKILL));
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn shutdown_terminates_descendants_after_parent_reap() {
        let runtime = shell_runtime(
            "trap 'exit 0' TERM; sh -c 'trap \"\" TERM; exec >/dev/null 2>&1; while :; do sleep 1; done' & descendant=$!; printf 'descendant=%s\\n' \"$descendant\"; while :; do sleep 1; done",
        );
        let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn process tree");
        wait_for_stdout(&owner, "descendant=").await;
        let output = String::from_utf8(owner.stdout_tail()).expect("UTF-8 descendant marker");
        let descendant_pid = output
            .lines()
            .find_map(|line| line.strip_prefix("descendant="))
            .expect("descendant marker")
            .parse::<u32>()
            .expect("descendant pid");

        let status = owner
            .shutdown(Duration::from_millis(25), Duration::from_secs(2))
            .await
            .expect("process-group shutdown");
        assert!(status.success(), "parent TERM trap status: {status}");
        assert!(
            !process_group_exists(owner.pid()).expect("probe process group"),
            "native process group survived acknowledged shutdown"
        );
        assert!(
            !Path::new(&format!("/proc/{descendant_pid}")).exists(),
            "descendant pid {descendant_pid} survived acknowledged shutdown"
        );
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn unexpected_exit_is_observed_before_group_identity_is_reaped() {
        let runtime = shell_runtime("exit 7");
        let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn exiting shell");
        let pid = owner.pid();
        let observed = tokio::time::timeout(Duration::from_secs(1), owner.wait_unexpected_exit())
            .await
            .expect("unexpected-exit observation timeout")
            .expect("observe unexpected exit");
        assert_eq!(observed.code(), Some(7));
        assert!(owner.child.is_some(), "waitid(WNOWAIT) reaped the leader");
        assert!(owner.reaped_status.is_none());
        assert!(
            process_group_exists(pid).expect("probe pinned process group"),
            "unreaped leader no longer pinned its PGID"
        );

        let reaped = owner
            .shutdown(Duration::ZERO, Duration::from_secs(1))
            .await
            .expect("cleanup observed exit");
        assert_eq!(reaped.code(), Some(7));
        assert!(owner.reaped_status.is_some());
        assert!(owner.group_cleanup_confirmed);
        assert!(!process_group_exists(pid).expect("probe reaped process group"));
    }

    #[cfg(unix)]
    #[tokio::test]
    async fn output_is_drained_concurrently_and_retained_with_a_bound() {
        let runtime = shell_runtime(
            "i=0; while [ $i -lt 10000 ]; do printf 'stdout-%08d-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\\n' $i; printf 'stderr-%08d-yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy\\n' $i >&2; i=$((i+1)); done",
        );
        let mut owner = NativeProcessOwner::spawn(&runtime).expect("spawn writer");
        let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
        loop {
            if owner.poll_unexpected_exit().expect("poll writer").is_some() {
                break;
            }
            assert!(tokio::time::Instant::now() < deadline, "writer timed out");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        owner
            .shutdown(Duration::ZERO, Duration::from_secs(1))
            .await
            .expect("finish drains");

        let stdout = owner.stdout_tail();
        let stderr = owner.stderr_tail();
        assert!(stdout.len() <= OUTPUT_TAIL_BYTES);
        assert!(stderr.len() <= OUTPUT_TAIL_BYTES);
        assert!(String::from_utf8_lossy(&stdout).contains("stdout-00009999"));
        assert!(String::from_utf8_lossy(&stderr).contains("stderr-00009999"));
    }
}
