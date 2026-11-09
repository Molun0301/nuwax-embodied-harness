//! Native FastLIO2/Point-LIO preflight, binding, and child-process ownership.

use std::collections::{BTreeMap, VecDeque};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::process::{CommandExt, ExitStatusExt};
use std::path::{Path, PathBuf};
use std::process::{ExitStatus, Stdio};
use std::sync::Arc;
use std::time::Duration;

use nalgebra::{Matrix3, Point3, Vector3};
use nuwax_lio_backends::native::{NativeBackend, NativeLioConfig, NativeRuntimeSpec};
use nuwax_lio_backends::pointlio_options::{PointLioOptions};
use nuwax_protocol::{LcmMessageType, LcmTopic};
use parking_lot::Mutex;
use serde::Deserialize;
use tokio::io::{AsyncRead, AsyncReadExt};
use tokio::process::{Child, Command};
use tokio::task::JoinHandle;


const LCM_MAX_CHANNEL_NAME_BYTES: usize = 63;
const ROTATION_TOLERANCE: f64 = 1.0e-6;
const OUTPUT_TAIL_BYTES: usize = 64 * 1024;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BodyFromSensorJson {
    translation_m: [f64; 3],
    rotation_row_major: [f64; 9],
    /// 物理雷达原点在输入点云坐标中的位置；旧设备原始载荷仍以零点为原点。
    #[serde(default)]
    ray_origin_in_input_m: [f64; 3],
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
    ray_origin_in_input_m: Point3<f64>,
}

impl BodyFromSensorTransform {
    pub(crate) fn from_runtime() -> Result<Self, String> {
        Self::from_json(&nuwax_config::geometry()?.lio_body_from_sensor)
    }

    pub(crate) fn from_json(raw: &str) -> Result<Self, String> {
        Self::from_json_for_key(raw, "geometry.lio_body_from_sensor")
    }

    pub(crate) fn from_json_for_key(raw: &str, key_name: &str) -> Result<Self, String> {
        if raw.is_empty() || raw != raw.trim() {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED config={key_name} value_is_empty_or_padded"
            ));
        }
        let parsed: BodyFromSensorJson = serde_json::from_str(raw).map_err(|error| {
            format!("NATIVE_LIO_PREFLIGHT_FAILED config={key_name} invalid_json: {error}")
        })?;
        if !parsed.translation_m.iter().all(|value| value.is_finite())
            || !parsed
                .ray_origin_in_input_m
                .iter()
                .all(|value| value.is_finite())
            || !parsed
                .rotation_row_major
                .iter()
                .all(|value| value.is_finite())
        {
            return Err(format!(
                "NATIVE_LIO_PREFLIGHT_FAILED config={key_name} values_must_be_finite"
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
                    "NATIVE_LIO_PREFLIGHT_FAILED config={key_name} rotation_must_be_proper_orthonormal determinant={determinant}"
                ));
        }

        Ok(Self {
            data: Box::new(BodyFromSensorData {
                translation_m: Vector3::from_row_slice(&parsed.translation_m),
                rotation,
                ray_origin_in_input_m: Point3::from(parsed.ray_origin_in_input_m),
            }),
        })
    }

    pub(crate) fn transform_point(&self, sensor_point: &Point3<f64>) -> Point3<f64> {
        Point3::from(self.data.rotation * sensor_point.coords + self.data.translation_m)
    }

    pub(crate) fn ray_origin_body(&self) -> Point3<f64> {
        self.transform_point(&self.data.ray_origin_in_input_m)
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
    /// 只由本拥有方启动的原生子进程消费原始传感器输入。
    pub(crate) input_cloud: (LcmTopic, LcmMessageType),
    pub(crate) input_imu: (LcmTopic, LcmMessageType),
    pub(crate) input_ack: Option<(LcmTopic, LcmMessageType)>,
    pub(crate) input_frame: Option<(LcmTopic, LcmMessageType)>,
}

#[derive(Clone, Debug)]
pub(crate) struct BoundNativeLioRuntime {
    pub(crate) channels: NativePrivateChannels,
    pub(crate) runtime: Box<NativeRuntimeSpec>,
    pub(crate) body_from_sensor: BodyFromSensorTransform,
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
    pair_max_skew: Duration,
}

impl NativeLioPreflight {
    pub(crate) fn from_runtime(backend: NativeBackend) -> Result<Self, String> {
        let executable = nuwax_config::sibling_binary(match backend {
            NativeBackend::FastLio2 => "fastlio2_native",
            NativeBackend::PointLio => "pointlio_native",
        })?;
        Self::from_config(
            backend,
            nuwax_config::geometry()?,
            nuwax_config::transport()?,
            &executable,
        )
    }
    pub(crate) fn from_config(
        backend: NativeBackend,
        g: &nuwax_config::sections::Geometry,
        transport: &nuwax_config::sections::Transport,
        executable: &Path,
    ) -> Result<Self, String> {
        let executable = canonical_executable(executable.to_str().ok_or("原生程序路径不是有效文本")?)?;
        let body_from_sensor = BodyFromSensorTransform::from_json(&g.lio_body_from_sensor)?;
        validate_lcm_url(&transport.lcm_url)?;
        let mut native_config = NativeLioConfig::for_backend(backend);
        native_config.scan_voxel_size_m = g.lio_scan_voxel_size_m;
        native_config.map_voxel_size_m = g.lio_map_voxel_size_m;
        native_config.initial_map_frames = g
            .lio_initial_map_frames
            .map(u32::try_from)
            .transpose()
            .map_err(|e| e.to_string())?;
        if backend == NativeBackend::PointLio {
            native_config.pointlio_options = PointLioOptions::from_json(&g.pointlio_options)?;
            let imu = BodyFromSensorTransform::from_json_for_key(
                &g.lio_body_from_imu,
                "geometry.lio_body_from_imu",
            )?;
            native_config.imu_origin_in_body_m = Some(imu.translation_m());
        }
        native_config
            .validate()
            .map_err(|e| format!("NATIVE_LIO_PREFLIGHT_FAILED: {e}"))?;
        Ok(Self {
            backend,
            native_config,
            executable,
            lcm_url: transport.lcm_url.clone(),
            private_prefix: "/lio".into(),
            body_from_sensor,
            pair_max_skew: Duration::from_millis(g.native_lio_pair_max_skew_ms as u64),
        })
    }

    pub(crate) fn bind(
        &self,
        sensor_frame: impl Into<String>,
        body_frame: impl Into<String>,
        odom_frame: impl Into<String>,
    ) -> Result<BoundNativeLioRuntime, String> {
        let backend_tag = backend_tag(self.backend);
        let channel_root = format!("{}/{backend_tag}", self.private_prefix);
        let channels = NativePrivateChannels {
            source_odometry: typed_binding(&format!("{channel_root}/o"), "nav_msgs.Odometry")?,
            source_cloud: typed_binding(&format!("{channel_root}/c"), "sensor_msgs.PointCloud2")?,
            input_cloud: typed_binding(&format!("{channel_root}/ic"), "sensor_msgs.PointCloud2")?,
            input_imu: typed_binding(&format!("{channel_root}/ii"), "sensor_msgs.Imu")?,
            input_frame: if self.backend == NativeBackend::PointLio {
                Some(typed_binding(
                    &format!("{channel_root}/m"),
                    "nuwax_msgs.LioInputFrame",
                )?)
            } else {
                None
            },
            input_ack: if self.backend == NativeBackend::PointLio {
                Some(typed_binding(
                    &format!("{channel_root}/ia"),
                    "nuwax_msgs.LioInputAck",
                )?)
            } else {
                None
            },
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
        if let Some(ack) = &channels.input_ack {
            let pattern = typed_pattern(ack);
            if pattern.len() > LCM_MAX_CHANNEL_NAME_BYTES {
                return Err("原生处理回执通道超过通信库长度限制".into());
            }
            config.input_ack_topic = Some(pattern);
        }
        if let Some(frame) = &channels.input_frame {
            let pattern = typed_pattern(frame);
            if pattern.len() > LCM_MAX_CHANNEL_NAME_BYTES {
                return Err("完整测量通道超过通信库长度限制".into());
            }
            config.input_frame_topic = Some(pattern);
        }
        config.sensor_frame_id = sensor_frame.into();
        config.child_frame_id = body_frame.into();
        config.frame_id = odom_frame.into();
        config.extrinsic_translation_m = self.body_from_sensor.translation_m();
        config.extrinsic_rotation_row_major = self.body_from_sensor.rotation_row_major();
        let mut args = config
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
        args.extend(["--lcm_url".to_owned(), self.lcm_url.clone()]);
        let runtime = NativeRuntimeSpec {
            backend: self.backend,
            cwd,
            executable: self.executable.clone(),
            args,
            env: BTreeMap::new(),
        };
        runtime
            .command()
            .map_err(|error| format!("NATIVE_LIO_BIND_FAILED invalid native runtime spec: {error}"))?;

        Ok(BoundNativeLioRuntime {
            channels,
            runtime: Box::new(runtime),
            body_from_sensor: self.body_from_sensor.clone(),
            pair_max_skew: self.pair_max_skew,
        })
    }
}

fn backend_tag(backend: NativeBackend) -> &'static str {
    match backend {
        NativeBackend::FastLio2 => "f2",
        NativeBackend::PointLio => "pl",
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









fn canonical_executable(raw: &str) -> Result<PathBuf, String> {
    let configured = Path::new(raw);
    if !configured.is_absolute() {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED executable path_must_be_absolute value={raw}"
        ));
    }
    let executable = configured.canonicalize().map_err(|error| {
        format!("NATIVE_LIO_PREFLIGHT_FAILED executable canonicalize_failed path={raw}: {error}")
    })?;
    let metadata = executable.metadata().map_err(|error| {
        format!(
            "NATIVE_LIO_PREFLIGHT_FAILED executable metadata_failed path={}: {error}",
            executable.display()
        )
    })?;
    if !metadata.file_type().is_file() {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED executable not_regular_file path={}",
            executable.display()
        ));
    }
    if metadata.permissions().mode() & 0o111 == 0 {
        return Err(format!(
            "NATIVE_LIO_PREFLIGHT_FAILED executable not_executable path={}",
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
            "NATIVE_LIO_PREFLIGHT_FAILED config=transport.lcm_url invalid_lcm_url"
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

    #[cfg(test)]
    stdout_tail: SharedTail,

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
        let stdout_drain = Some(spawn_output_drain("stdout", stdout, stdout_tail.clone()));
        let stderr_drain = spawn_output_drain("stderr", stderr, stderr_tail.clone());

        Ok(Self {
            child: Some(child),
            pid,
            process_group_active: true,
            group_cleanup_confirmed: false,
            observed_status: None,
            reaped_status: None,
            #[cfg(test)]
            stdout_tail,
            stderr_tail,
            stdout_drain,
            stderr_drain: Some(stderr_drain),
        })
    }

    #[cfg(test)]
    pub(crate) fn pid(&self) -> u32 {
        self.pid
    }

    #[cfg(test)]
    pub(crate) fn stdout_tail(&self) -> Vec<u8> {
        self.stdout_tail.lock().snapshot()
    }

    pub(crate) fn stderr_tail(&self) -> Vec<u8> {
        self.stderr_tail.lock().snapshot()
    }

    /// Returns `Some` for every exit; a native backend is expected to remain alive.

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
            tracing::info!(component = "native_lio", stream = label,
                output = %String::from_utf8_lossy(&buffer[..read]), "原生LIO子进程输出");
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
#[path = "native_lio_owner_tests.rs"]
mod tests;
