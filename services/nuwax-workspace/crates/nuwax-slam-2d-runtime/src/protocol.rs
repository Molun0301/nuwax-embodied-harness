//! 生命周期命令与候选协议；控制回执不等同Geometry的规范提交回执。
use base64::Engine;
use serde::{Deserialize, Serialize};
use serde_json::Value;

pub const VERSION: u32 = 4;
pub const WORKER_CONTRACT: &str = "nuwax-slam-2d-worker-v4";
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Identity {
    pub runtime_generation: u64,
    pub owner_nonce: String,
}
impl Identity {
    pub fn validate(&self) -> Result<(), String> {
        if self.runtime_generation == 0
            || self.runtime_generation > i64::MAX as u64
            || self.owner_nonce.len() != 32
            || !self.owner_nonce.bytes().all(|b| b.is_ascii_hexdigit())
        {
            return Err("二维工作进程身份非法".into());
        }
        Ok(())
    }
    pub fn rpc_name(&self) -> String {
        format!("slam2d.{}", self.owner_nonce)
    }
    pub fn scan_prefix(&self) -> String {
        let id = u128::from_str_radix(&self.owner_nonce, 16).expect("已验证的拥有方身份");
        format!(
            "2/{}/",
            base64::engine::general_purpose::URL_SAFE_NO_PAD.encode(id.to_be_bytes())
        )
    }
    pub fn scan_channel(&self, generation: u64) -> String {
        format!(
            "{}{:016x}#nuwax_msgs.PlanarScan",
            self.scan_prefix(),
            generation
        )
    }
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum OdometrySource {
    Device,
    Laser,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Session {
    pub session_id: String,
    pub map_name: String,
    pub session_generation: u64,
    pub sensor_id: String,
    pub frame_id: String,
    pub map_frame_id: String,
    pub odom_frame_id: String,
    pub body_frame_id: String,
    pub clock_domain: String,
    pub clock_id: String,
    pub calibration_ref: String,
    pub odometry_source: OdometrySource,
    pub core_config: Value,
}
impl Session {
    pub fn validate(&self) -> Result<(), String> {
        for text in [
            &self.session_id,
            &self.map_name,
            &self.sensor_id,
            &self.frame_id,
            &self.map_frame_id,
            &self.odom_frame_id,
            &self.body_frame_id,
            &self.calibration_ref,
        ] {
            if text.trim().is_empty() || text.len() > 255 || text.contains('\0') {
                return Err("二维会话字段为空或超限".into());
            }
        }
        if self.clock_id.len() > 255 || self.clock_id.contains('\0') {
            return Err("二维时钟标识超限或含零字节".into());
        }
        if !matches!(
            self.clock_domain.as_str(),
            "unix_ns" | "monotonic_ns" | "device_ns"
        ) || self.clock_domain != "unix_ns" && self.clock_id.is_empty()
        {
            return Err("二维会话必须绑定明确的时钟域和标识".into());
        }
        let map = self.map_frame_id.trim().trim_start_matches('/');
        let odom = self.odom_frame_id.trim().trim_start_matches('/');
        let body = self.body_frame_id.trim().trim_start_matches('/');
        let sensor = self.frame_id.trim().trim_start_matches('/');
        if [map, odom, body, sensor]
            .iter()
            .any(|frame| frame.is_empty())
        {
            return Err("二维坐标规范化后为空".into());
        }
        if map == odom || map == body || odom == body {
            return Err("二维地图、里程计和机体坐标必须有各自的明确身份".into());
        }
        let extrinsic = self
            .core_config
            .get("base_from_laser")
            .and_then(Value::as_array);
        if !extrinsic.is_some_and(|v| {
            v.len() == 3 && v.iter().all(|n| n.as_f64().is_some_and(f64::is_finite))
        }) {
            return Err("二维会话必须显式提供已标定的雷达到机体外参".into());
        }
        if sensor == map || sensor == odom {
            return Err("原始二维雷达坐标不能冒充地图或里程计坐标".into());
        }
        if sensor == body
            && extrinsic
                .unwrap()
                .iter()
                .any(|v| v.as_f64().unwrap().abs() > 1e-9)
        {
            return Err("雷达与机体同名坐标必须确为同一物理坐标".into());
        }
        if self.session_generation == 0 || !self.core_config.is_object() {
            return Err("二维会话代次或算法配置非法".into());
        }
        Ok(())
    }
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "operation", rename_all = "snake_case", deny_unknown_fields)]
pub enum Operation {
    Status,
    Start {
        session: Session,
    },
    Stop {
        session_id: String,
        session_generation: u64,
    },
    Odometry {
        sample: crate::input::TimedPose,
    },
    Snapshot {
        session_id: String,
        session_generation: u64,
    },
    ReleaseSnapshot {
        snapshot_id: u64,
    },
    SetPaused {
        session_id: String,
        session_generation: u64,
        paused: bool,
    },
    ExportGraph {
        session_id: String,
        session_generation: u64,
        unix_offset_ns: i64,
    },
    LoadGraph {
        session: Session,
        artifact: FileArtifact,
        localization: bool,
        seed: [f64; 3],
    },
    ReleaseArtifact {
        artifact: FileArtifact,
    },
    Shutdown,
}
impl Operation {
    pub fn mutates(&self) -> bool {
        !matches!(
            self,
            Self::Status | Self::Snapshot { .. } | Self::Odometry { .. }
        )
    }
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Request {
    pub version: u32,
    pub identity: Identity,
    pub command_sequence: u64,
    pub operation: Operation,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
#[serde(deny_unknown_fields)]
pub struct Estimate {
    pub source_timestamp_ns: u64,
    pub clock_domain: String,
    pub clock_id: String,
    pub source_sequence: u64,
    pub graph_revision: u64,
    pub optimizations: u64,
    pub anchor_odom_from_base: [f64; 3],
    pub odom_from_base: [f64; 3],
    pub map_from_base: [f64; 3],
    pub map_from_odom: [f64; 3],
    pub covariance: [f64; 9],
    pub matching_response: f64,
    pub local_response: f64,
    pub disposition: String,
    pub node_count: u32,
    pub edge_count: u32,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
#[serde(deny_unknown_fields)]
pub struct Status {
    pub motion_compensation: String,
    pub paused: bool,
    pub localization: bool,
    pub graph_revision: u64,
    pub session: Option<Session>,
    pub last_estimate: Option<Estimate>,
    pub received_scans: u64,
    pub dropped_scans: u64,
    pub rejected_scans: u64,
    pub last_error: Option<String>,
    pub active: bool,
    pub stopping: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Snapshot {
    pub snapshot_id: u64,
    pub session: Session,
    pub estimate: Estimate,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin: [f64; 2],
    pub payload_size: u64,
    pub data_sha256: [u8; 32],
    pub file_name: String,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Response {
    pub version: u32,
    pub identity: Identity,
    pub command_sequence: u64,
    pub status: Status,
    pub snapshot: Option<Snapshot>,
    pub artifact: Option<FileArtifact>,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct FileArtifact {
    pub file_name: String,
    pub payload_size: u64,
    pub sha256: [u8; 32],
}
impl FileArtifact {
    pub fn validate(&self) -> Result<(), String> {
        if self.file_name.is_empty()
            || self.file_name.len() > 128
            || !self
                .file_name
                .bytes()
                .all(|c| c.is_ascii_alphanumeric() || c == b'-' || c == b'.')
            || self.file_name.starts_with('.')
            || self.payload_size == 0
            || self.payload_size > crate::archive::MAX_ARCHIVE
            || self.sha256 == [0; 32]
        {
            return Err("二维文件制品描述非法".into());
        }
        Ok(())
    }
}
