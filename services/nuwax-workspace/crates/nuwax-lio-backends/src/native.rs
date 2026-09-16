//! Strict contract for the vendored Dimos FastLIO2 and Point-LIO native programs.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::net::Ipv4Addr;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus};
use std::sync::Mutex;
use crate::pointlio_options::PointLioOptions;

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
const ROTATION_TOLERANCE: f64 = 1.0e-6;
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
    /// Optional canonical raw sensor ingress. When set, the child subscribes
    /// to typed PointCloud2/Imu channels and does not open Livox callbacks.
    pub input_lidar_topic: Option<String>,
    pub input_imu_topic: Option<String>,
    pub frame_id: String,
    pub child_frame_id: String,
    pub sensor_frame_id: String,
    pub extrinsic_translation_m: [f64; 3],
    pub extrinsic_rotation_row_major: [f64; 9],
    pub host_ip: String,
    pub lidar_ip: String,
    /// 设备裸包聚合频率；规范输入本身已经携带完整扫描边界。
    pub lidar_frequency_hz: f64,
    /// 设备裸包模式的点云输出上限；规范完整扫描按每个真实新解交付。
    pub pointcloud_frequency_hz: f64,
    /// 设备裸包模式的位姿输出上限；规范输入仍保留原始源时间递增门。
    pub odometry_frequency_hz: f64,
    pub debug: bool,
    /// 扫描下采样体素边长；未配置时保留原生程序默认值，单位为米。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scan_voxel_size_m: Option<f64>,
    /// 地图插入筛选体素边长，不表示Point的iVox索引格，单位为米。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub map_voxel_size_m: Option<f64>,
    /// 初图最少有效扫描数；Point仍同时遵守原有点数门及静止初始化前提。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_map_frames: Option<u32>,
    /// Point专用参数，已在共享契约中校验，不能任意注入启动参数。
    #[serde(default, skip_serializing_if = "PointLioOptions::is_empty")]
    pub pointlio_options: PointLioOptions,
    /// 惯性测量原点在机体坐标中的位置；仅Point当前接口消费。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imu_origin_in_body_m: Option<[f64; 3]>,
}

impl NativeLioConfig {
    pub fn for_backend(backend: NativeBackend) -> Self {
        Self {
            backend,
            lidar_topic: LIDAR_TOPIC.into(),
            odometry_topic: ODOMETRY_TOPIC.into(),
            input_lidar_topic: None,
            input_imu_topic: None,
            frame_id: ODOM_FRAME.into(),
            child_frame_id: BODY_FRAME.into(),
            sensor_frame_id: SENSOR_FRAME.into(),
            extrinsic_translation_m: [0.0; 3],
            extrinsic_rotation_row_major: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            host_ip: "192.168.1.5".into(),
            lidar_ip: "192.168.1.155".into(),
            lidar_frequency_hz: 10.0,
            pointcloud_frequency_hz: 10.0,
            odometry_frequency_hz: 30.0,
            debug: false,
            scan_voxel_size_m: None,
            map_voxel_size_m: None,
            initial_map_frames: None,
            pointlio_options: PointLioOptions::default(),
            imu_origin_in_body_m: None,
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        self.pointlio_options.validate()?;
        if self.backend != NativeBackend::PointLio
            && (!self.pointlio_options.is_empty() || self.imu_origin_in_body_m.is_some()) {
            return Err("Point专用参数不能传给其他后端".into());
        }
        if self.imu_origin_in_body_m.is_some_and(|p| !p.iter().all(|v| v.is_finite())) {
            return Err("惯性原点必须为有限三维坐标".into());
        }
        validate_typed_topic(&self.lidar_topic, "sensor_msgs.PointCloud2", "lidar_topic")?;
        validate_typed_topic(&self.odometry_topic, "nav_msgs.Odometry", "odometry_topic")?;
        if let Some(topic) = &self.input_lidar_topic {
            validate_typed_topic(topic, "sensor_msgs.PointCloud2", "input_lidar_topic")?;
        }
        if let Some(topic) = &self.input_imu_topic {
            validate_typed_topic(topic, "sensor_msgs.Imu", "input_imu_topic")?;
        }
        if self.input_imu_topic.is_some() != self.input_lidar_topic.is_some() {
            return Err("input_lidar_topic and input_imu_topic must be configured together".into());
        }
        validate_frame(&self.frame_id, "frame_id")?;
        validate_frame(&self.child_frame_id, "child_frame_id")?;
        validate_frame(&self.sensor_frame_id, "sensor_frame_id")?;
        if self.frame_id == self.child_frame_id
            || self.frame_id == self.sensor_frame_id
            || self.child_frame_id == self.sensor_frame_id
        {
            return Err("odom, body, and sensor frame IDs must be distinct".into());
        }
        validate_extrinsic_translation(&self.extrinsic_translation_m)?;
        validate_extrinsic_rotation(&self.extrinsic_rotation_row_major)?;

        if self.input_lidar_topic.is_none() {
            let host_ip = validate_ipv4(&self.host_ip, "host_ip")?;
            let lidar_ip = validate_ipv4(&self.lidar_ip, "lidar_ip")?;
            if host_ip == lidar_ip {
                return Err("host_ip and lidar_ip must be distinct".into());
            }
        }

        validate_frequency(self.lidar_frequency_hz, "lidar_frequency_hz")?;
        validate_frequency(self.pointcloud_frequency_hz, "pointcloud_frequency_hz")?;
        validate_frequency(self.odometry_frequency_hz, "odometry_frequency_hz")?;
        for (name, value) in [
            ("scan_voxel_size_m", self.scan_voxel_size_m),
            ("map_voxel_size_m", self.map_voxel_size_m),
        ] {
            if let Some(value) = value {
                let native_value = value as f32;
                if !value.is_finite() || value <= 0.0
                    || !native_value.is_normal() || native_value <= 0.0
                {
                    return Err(format!(
                        "{name} must remain a positive normal float32 for native voxel operations"
                    ));
                }
            }
        }
        if self.initial_map_frames.is_some_and(|value| value == 0 || value > i32::MAX as u32) {
            return Err("initial_map_frames must be within [1, 2147483647]".into());
        }
        Ok(())
    }

    pub fn render_args(&self) -> Result<Vec<String>, String> {
        self.validate()?;
        let mut args = vec![
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
            "--extrinsic_t".into(),
            render_numbers(&self.extrinsic_translation_m),
            "--extrinsic_r".into(),
            render_numbers(&self.extrinsic_rotation_row_major),
            "--frequency".into(),
            render_number(self.lidar_frequency_hz),
            "--pointcloud_freq".into(),
            render_number(self.pointcloud_frequency_hz),
            "--odom_freq".into(),
            render_number(self.odometry_frequency_hz),
            "--debug".into(),
            self.debug.to_string(),
        ];
        for (option, value) in [
            ("--filter_size_surf", self.scan_voxel_size_m),
            ("--filter_size_map", self.map_voxel_size_m),
        ] {
            if let Some(value) = value {
                args.extend([option.to_owned(), render_number(value)]);
            }
        }
        if let Some(value) = self.initial_map_frames {
            args.extend(["--initial_map_frames".to_owned(), value.to_string()]);
        }
        args.extend(self.pointlio_options.render_args()?);
        if let Some(origin) = self.imu_origin_in_body_m {
            args.extend(["--imu_origin_in_body".to_owned(), render_numbers(&origin)]);
        }
        if let (Some(input_lidar), Some(input_imu)) =
            (&self.input_lidar_topic, &self.input_imu_topic)
        {
            args.extend([
                "--input_lidar".into(),
                input_lidar.clone(),
                "--input_imu".into(),
                input_imu.clone(),
            ]);
        } else {
            // 只有直连硬件驱动需要网络地址；标准传感器通道与雷达厂商无关。
            args.extend([
                "--host_ip".into(), self.host_ip.clone(),
                "--lidar_ip".into(), self.lidar_ip.clone(),
            ]);
        }
        Ok(args)
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

fn validate_extrinsic_translation(translation: &[f64; 3]) -> Result<(), String> {
    if translation.iter().any(|value| !value.is_finite()) {
        return Err("extrinsic_translation_m must contain only finite values".into());
    }
    Ok(())
}

fn validate_extrinsic_rotation(rotation: &[f64; 9]) -> Result<(), String> {
    if rotation.iter().any(|value| !value.is_finite()) {
        return Err("extrinsic_rotation_row_major must contain only finite values".into());
    }

    for row in 0..3 {
        for other_row in 0..3 {
            let dot_product = (0..3)
                .map(|column| rotation[row * 3 + column] * rotation[other_row * 3 + column])
                .sum::<f64>();
            let expected = if row == other_row { 1.0 } else { 0.0 };
            if (dot_product - expected).abs() > ROTATION_TOLERANCE {
                return Err(
                    "extrinsic_rotation_row_major must be an orthonormal rotation matrix".into(),
                );
            }
        }
    }

    let determinant = rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
        - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
        + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
    if (determinant - 1.0).abs() > ROTATION_TOLERANCE {
        return Err(
            "extrinsic_rotation_row_major must be a proper rotation matrix with determinant +1"
                .into(),
        );
    }

    Ok(())
}

fn render_number(value: f64) -> String {
    value.to_string()
}

fn render_numbers(values: &[f64]) -> String {
    values
        .iter()
        .map(|value| render_number(*value))
        .collect::<Vec<_>>()
        .join(",")
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const FASTLIO2_MAIN: &str = include_str!("../native/fastlio2/cpp/main.cpp");
    const POINTLIO_MAIN: &str = include_str!("../native/pointlio/cpp/main.cpp");
    const PAIRING_MARKER: &str = "Publish odometry first whenever a cloud is emitted";

    fn assert_arg_value(args: &[String], option: &str, expected: &str) {
        let option_index = args
            .iter()
            .position(|argument| argument == option)
            .unwrap_or_else(|| panic!("missing native option {option}"));
        assert_eq!(
            args.get(option_index + 1).map(String::as_str),
            Some(expected),
            "unexpected value for native option {option}"
        );
    }

    #[test]
    fn private_topic_argv_is_preserved() {
        let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
        config.lidar_topic =
            "/geometry/lio/private/42/fastlio2/cloud#sensor_msgs.PointCloud2".into();
        config.odometry_topic =
            "/geometry/lio/private/42/fastlio2/odometry#nav_msgs.Odometry".into();

        let args = config.render_args().expect("private topics must be valid");
        assert_arg_value(&args, "--lidar", &config.lidar_topic);
        assert_arg_value(&args, "--odometry", &config.odometry_topic);
    }

    #[test]
    fn typed_input_topics_are_paired_and_rendered() {
        let mut only_lidar = NativeLioConfig::for_backend(NativeBackend::FastLio2);
        only_lidar.input_lidar_topic = Some("/raw/cloud#sensor_msgs.PointCloud2".into());
        assert!(only_lidar.render_args().is_err());

        let mut only_imu = NativeLioConfig::for_backend(NativeBackend::PointLio);
        only_imu.input_imu_topic = Some("/raw/imu#sensor_msgs.Imu".into());
        assert!(only_imu.render_args().is_err());

        let mut typed = NativeLioConfig::for_backend(NativeBackend::PointLio);
        typed.input_lidar_topic = Some("/raw/cloud#sensor_msgs.PointCloud2".into());
        typed.input_imu_topic = Some("/raw/imu#sensor_msgs.Imu".into());
        typed.host_ip.clear();
        typed.lidar_ip.clear();
        let args = typed.render_args().expect("paired typed inputs must render");
        assert_arg_value(&args, "--input_lidar", "/raw/cloud#sensor_msgs.PointCloud2");
        assert_arg_value(&args, "--input_imu", "/raw/imu#sensor_msgs.Imu");
        assert!(!args.iter().any(|argument| argument == "--host_ip" || argument == "--lidar_ip"));

        let direct = NativeLioConfig::for_backend(NativeBackend::FastLio2)
            .render_args()
            .expect("direct Livox mode must remain valid");
        assert!(!direct.iter().any(|argument| argument == "--input_lidar"));
        assert!(!direct.iter().any(|argument| argument == "--input_imu"));
        assert_arg_value(&direct, "--host_ip", "192.168.1.5");
        assert_arg_value(&direct, "--lidar_ip", "192.168.1.155");
    }

    #[test]
    fn non_identity_extrinsic_argv_is_stable() {
        let mut config = NativeLioConfig::for_backend(NativeBackend::PointLio);
        config.extrinsic_translation_m = [0.125, -0.25, 1.5];
        config.extrinsic_rotation_row_major = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

        let args = config
            .render_args()
            .expect("proper extrinsic must be valid");
        assert_arg_value(&args, "--extrinsic_t", "0.125,-0.25,1.5");
        assert_arg_value(&args, "--extrinsic_r", "0,-1,0,1,0,0,0,0,1");
    }

    #[test]
    fn optional_sampling_defaults_preserve_legacy_serialization_and_arguments() {
        for backend in [NativeBackend::FastLio2, NativeBackend::PointLio] {
            let config = NativeLioConfig::for_backend(backend);
            let legacy = serde_json::to_value(&config).expect("serialize defaults");
            for key in ["scan_voxel_size_m", "map_voxel_size_m", "initial_map_frames"] {
                assert!(legacy.get(key).is_none(), "unset {key} must stay omitted");
            }
            let restored: NativeLioConfig = serde_json::from_value(legacy.clone())
                .expect("old serialized configuration must remain readable");
            assert_eq!(restored, config);
            let args = restored.render_args().expect("default arguments");
            for option in ["--filter_size_surf", "--filter_size_map", "--initial_map_frames"] {
                assert!(!args.iter().any(|arg| arg == option));
            }
            let mut unknown = legacy;
            unknown["extra_args"] = serde_json::json!(["--unexpected"]);
            assert!(serde_json::from_value::<NativeLioConfig>(unknown).is_err());
        }
    }

    #[test]
    fn optional_sampling_renders_only_selected_known_arguments() {
        for backend in [NativeBackend::FastLio2, NativeBackend::PointLio] {
            let mut config = NativeLioConfig::for_backend(backend);
            config.scan_voxel_size_m = Some(0.1);
            config.initial_map_frames = Some(7);
            let args = config.render_args().expect("sampling arguments");
            assert_arg_value(&args, "--filter_size_surf", "0.1");
            assert_arg_value(&args, "--initial_map_frames", "7");
            assert!(!args.iter().any(|arg| arg == "--filter_size_map"));
            config.map_voxel_size_m = Some(0.2);
            assert_arg_value(&config.render_args().unwrap(), "--filter_size_map", "0.2");
        }
    }

    #[test]
    fn optional_sampling_rejects_unusable_native_number_ranges() {
        for value in [0.0, -0.1, f64::NAN, f64::INFINITY, 1e-40, 1e-300, 1e40] {
            for scan in [true, false] {
                let mut config = NativeLioConfig::for_backend(NativeBackend::PointLio);
                if scan {
                    config.scan_voxel_size_m = Some(value);
                } else {
                    config.map_voxel_size_m = Some(value);
                }
                assert!(config.render_args().is_err(), "invalid voxel size: {value}");
            }
        }
        for value in [f32::MIN_POSITIVE as f64, 0.1, f32::MAX as f64] {
            let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
            config.scan_voxel_size_m = Some(value);
            assert!(config.render_args().is_ok());
        }
        for value in [0, i32::MAX as u32 + 1] {
            let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
            config.initial_map_frames = Some(value);
            assert!(config.render_args().is_err());
        }
        let mut config = NativeLioConfig::for_backend(NativeBackend::PointLio);
        config.initial_map_frames = Some(i32::MAX as u32);
        assert!(config.render_args().is_ok());
        let mut serialized = serde_json::to_value(config).unwrap();
        serialized["initial_map_frames"] = serde_json::json!(1.5);
        assert!(serde_json::from_value::<NativeLioConfig>(serialized).is_err());
    }

    #[test]
    fn invalid_extrinsic_fails_closed() {
        let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
        config.extrinsic_translation_m[0] = f64::NAN;
        assert!(config.render_args().is_err());

        let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
        config.extrinsic_rotation_row_major[0] = 2.0;
        assert!(config.render_args().is_err());

        let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
        config.extrinsic_rotation_row_major[0] = f64::INFINITY;
        assert!(config.render_args().is_err());

        let mut config = NativeLioConfig::for_backend(NativeBackend::FastLio2);
        config.extrinsic_rotation_row_major = [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!(config.render_args().is_err());
    }

    #[test]
    fn source_lock_manifest_covers_hardened_native_sources() {
        let manifest: Value = serde_json::from_str(include_str!("../native/SOURCE_LOCKS.json"))
            .expect("source-lock manifest must be valid JSON");
        let files = manifest["files"]
            .as_object()
            .expect("source-lock files must be an object");

        assert_eq!(files.len(), 17);
        assert_eq!(
            files["fastlio2/cpp/main.cpp"],
            "4eff4cc0fd7228deb687082e1c4f06a73220e5e586d51ea8de9c1f3d2b3b849f"
        );
        assert_eq!(
            files["pointlio/cpp/main.cpp"],
            "5576ff1fea04b1b6b2e371f633166b62d221f97749efef7b69fa155b4ddb60b5"
        );
        assert_eq!(
            files["livox/cpp/README.md"],
            "3077f45bcaacda21cf6d333a12c30cc486904e1d1bbaf9affddb1c393129af01"
        );
        assert!(include_str!("../native/livox/cpp/README.md")
            .starts_with("# Livox Mid-360 Native Module (C++)"));
    }

    #[test]
    fn native_publishers_use_monotonic_source_timestamp_pairs() {
        for (backend, source) in [("fastlio2", FASTLIO2_MAIN), ("pointlio", POINTLIO_MAIN)] {
            assert!(source.contains("next_source_timestamp"), "{backend}");
            assert!(source.contains("make_source_header"), "{backend}");
            assert!(
                source.contains("odometry.header.stamp.sec")
                    && source.contains("odometry.header.stamp.nsecs"),
                "{backend} must reconstruct the estimator solution timestamp"
            );
            assert!(
                source.contains("std::numeric_limits<int32_t>::max()"),
                "{backend} must reject source timestamps the output cannot represent"
            );
            assert!(
                source.contains("if (odometry.header.frame_id.empty()) { return false; }"),
                "{backend} must reject an uninitialized estimator result"
            );
            assert!(
                source.contains("const bool first_output = !has_last_output_source_timestamp;"),
                "{backend} must emit its first valid solution immediately"
            );
            assert!(
                source.contains("static std::mutex g_lio_mutex;"),
                "{backend} must serialize estimator access"
            );
            assert!(
                source
                    .matches("std::lock_guard<std::mutex> lio_lock(g_lio_mutex);")
                    .count()
                    >= 2,
                "{backend} must lock both callback and snapshot access"
            );
            assert!(
                source.contains(
                    "odometry = make_odometry_message(estimator_odometry, source_timestamp_ns);"
                ),
                "{backend} must build one initialized estimator snapshot before publishing"
            );
            assert!(
                !source.contains("odometry = estimator_odometry;"),
                "{backend}"
            );
            assert!(
                !source.contains("pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0"),
                "{backend} must allow a valid zero-translation pose"
            );
            assert!(
                !source.contains("g_latest_source_timestamp_ns"),
                "{backend}"
            );
            assert!(!source.contains("observe_source_timestamp"), "{backend}");
            assert!(!source.contains("system_clock::now"), "{backend}");
            assert!(!source.contains("get_publish_ts"), "{backend}");
            assert!(
                !source.contains("reinterpret_cast<float*>(pc.data.data()"),
                "{backend} must not write floats through an unaligned byte buffer"
            );
            assert!(
                source.contains("std::memcpy(pc.data.data() + byte_offset + 12"),
                "{backend} must serialize point fields with alignment-safe copies"
            );
            assert!(
                source.contains("g_accumulated_points = std::move(accepted_points);"),
                "{backend} must replace, never merge, pending typed scans"
            );
            assert!(
                !source.contains("g_accumulated_points.insert(g_accumulated_points.end(), accepted_points.begin(), accepted_points.end())"),
                "{backend} must not append independent typed scans"
            );
            // 逐点存储顺序由真实C++回调回归验证，不等同于扫描源时间递增。
            assert!(source.contains("source timestamp must increase"), "{backend}");
            assert!(source.contains("typed_imu_covers_scan"), "{backend}");

            let paired_publish = source
                .split_once(PAIRING_MARKER)
                .unwrap_or_else(|| panic!("{backend} is missing the pairing contract"))
                .1;
            let odometry_position = paired_publish
                .find("publish_odometry")
                .unwrap_or_else(|| panic!("{backend} is missing paired odometry"));
            let cloud_position = paired_publish
                .find("publish_lidar")
                .unwrap_or_else(|| panic!("{backend} is missing paired cloud"));
            assert!(
                odometry_position < cloud_position,
                "{backend} must publish odometry before its paired cloud"
            );
        }

        assert!(POINTLIO_MAIN.contains("!last_pc_publish.has_value()"));
        assert!(POINTLIO_MAIN.contains("!last_odom_publish.has_value()"));
        assert_eq!(POINTLIO_MAIN.matches("last_pc_publish = now;").count(), 1);
        assert_eq!(POINTLIO_MAIN.matches("last_odom_publish = now;").count(), 1);
    }
}
