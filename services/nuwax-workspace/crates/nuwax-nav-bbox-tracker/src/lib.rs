//! BBox tracker：把 2D detection bbox 中心投影成相机 frame 下导航目标 PoseStamped。
//!
//! 行为对账源：`dimos/navigation/bbox_navigation.py`。
//!
//! 投影公式（与 Python 注释一致）：
//!   x = (center_x - cx) / fx * goal_distance
//!   y = (center_y - cy) / fy * goal_distance
//!   z = goal_distance
//!
//! 最终 PoseStamped.position 是否使用 `(z, -x, -y)` 重排 → 由 `BBoxTrackerConfig::swap_axes` 控制：
//! - `true`（默认）：与 Python 日志对齐 `(z, -x, -y)`；
//! - `false`：直接使用 `(x, y, z)`。
//!
//! 此重排的来源是 Python ROS 习惯（先 z 再 -x/-y 模拟 `map` frame 与 `camera_optical` 之间的轴交换）。
//! 是否保留重排必须以测试固定。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use serde::{Deserialize, Serialize};
use thiserror::Error;

// Foundation 协议面 re-export — 跨 crate PoseStamped / Quaternion 协议语义对齐
pub use nuwax_nav_types_messages::BoundingBox2D as FoundationBoundingBox2D;
pub use nuwax_nav_types_messages::CameraInfo;
pub use nuwax_nav_types_messages::Detection2D;
pub use nuwax_nav_types_messages::Detection2DArray;
pub use nuwax_nav_types_messages::Pose3 as FoundationPose3;
pub use nuwax_nav_types_messages::PoseStamped;
pub use nuwax_nav_types_messages::Quaternion as FoundationQuaternion;

/// 配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BBoxTrackerConfig {
    /// 目标距离（m）
    pub goal_distance: f64,
    /// 是否应用 `(z, -x, -y)` 重排（与 Python ROS 习惯对齐）
    pub swap_axes: bool,
}

impl Default for BBoxTrackerConfig {
    fn default() -> Self {
        Self {
            goal_distance: 1.0,
            swap_axes: true,
        }
    }
}

/// 输出状态
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum BBoxTrackerStatus {
    /// 已生成 goal
    Published(PoseStamped),
    /// 无 detection
    NoDetection,
    /// 缺 camera intrinsics，等待下一帧
    WaitingForIntrinsics,
}

/// BBox tracker 输出错误
#[derive(Debug, Error)]
pub enum BBoxTrackerError {
    /// 配置非法
    #[error("invalid config: {0}")]
    InvalidConfig(String),
}

/// BBox tracker（纯算法核心）
pub struct BBoxTracker {
    config: BBoxTrackerConfig,
    intrinsics: Option<CameraInfo>,
}

impl BBoxTracker {
    /// 构造
    pub fn new(config: BBoxTrackerConfig) -> Result<Self, BBoxTrackerError> {
        if !(config.goal_distance.is_finite() && config.goal_distance > 0.0) {
            return Err(BBoxTrackerError::InvalidConfig(
                "goal_distance must be finite > 0".into(),
            ));
        }
        Ok(Self {
            config,
            intrinsics: None,
        })
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &BBoxTrackerConfig {
        &self.config
    }

    /// 上传一次 camera info
    pub fn update_camera_info(&mut self, info: CameraInfo) {
        self.intrinsics = Some(info);
    }

    /// 处理检测：返回状态
    pub fn process(&self, detections: &Detection2DArray) -> BBoxTrackerStatus {
        let Some(intr) = &self.intrinsics else {
            return BBoxTrackerStatus::WaitingForIntrinsics;
        };
        let Some(det) = detections.first() else {
            return BBoxTrackerStatus::NoDetection;
        };
        let (cx, cy) = det.bbox_center;
        let x_raw = (cx - intr.cx) / intr.fx * self.config.goal_distance;
        let y_raw = (cy - intr.cy) / intr.fy * self.config.goal_distance;
        let z_raw = self.config.goal_distance;
        let (px, py, pz) = if self.config.swap_axes {
            // Python 注释：`(z, -x, -y)`
            (z_raw, -x_raw, -y_raw)
        } else {
            (x_raw, y_raw, z_raw)
        };
        let pose = PoseStamped::from_pose(
            nuwax_nav_types_messages::Pose3 {
                position: nuwax_nav_types_messages::Point3 {
                    x: px,
                    y: py,
                    z: pz,
                },
                orientation: FoundationQuaternion::IDENTITY,
            },
            detections.frame_id.clone(),
        );
        BBoxTrackerStatus::Published(pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nuwax_nav_types_messages::{NavFrameId, Timestamp};

    fn intr() -> CameraInfo {
        CameraInfo::from_k(
            640,
            480,
            &[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        )
    }

    fn dets_with_one(center: (f64, f64)) -> Detection2DArray {
        Detection2DArray {
            frame_id: NavFrameId::from_static("camera_optical"),
            stamp: Timestamp::default(),
            detections: vec![Detection2D::new(center, (100.0, 100.0))],
        }
    }

    #[test]
    fn no_detection_returns_no_detection() {
        let mut t = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        t.update_camera_info(intr());
        let dets = Detection2DArray {
            frame_id: NavFrameId::from_static("camera_optical"),
            stamp: Timestamp::default(),
            detections: vec![],
        };
        match t.process(&dets) {
            BBoxTrackerStatus::NoDetection => {}
            _ => panic!(),
        }
    }

    #[test]
    fn no_intrinsics_returns_waiting() {
        let t = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        let dets = dets_with_one((320.0, 240.0));
        match t.process(&dets) {
            BBoxTrackerStatus::WaitingForIntrinsics => {}
            _ => panic!(),
        }
    }

    #[test]
    fn bbox_center_at_principal_point_with_swap_axes() {
        let mut t = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        t.update_camera_info(intr());
        // center = (320, 240) == (cx, cy)
        let dets = dets_with_one((320.0, 240.0));
        match t.process(&dets) {
            BBoxTrackerStatus::Published(p) => {
                // 中心 -> x = 0, y = 0 -> (z, -x, -y) = (1, 0, 0)
                assert_eq!(p.frame_id.as_str(), "camera_optical");
                assert_relative_eq!(p.pose.position.x, 1.0, epsilon = 1e-9);
                assert_relative_eq!(p.pose.position.y, 0.0, epsilon = 1e-9);
                assert_relative_eq!(p.pose.position.z, 0.0, epsilon = 1e-9);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn bbox_center_at_principal_point_no_swap() {
        let cfg = BBoxTrackerConfig {
            goal_distance: 1.0,
            swap_axes: false,
        };
        let mut t = BBoxTracker::new(cfg).unwrap();
        t.update_camera_info(intr());
        let dets = dets_with_one((320.0, 240.0));
        match t.process(&dets) {
            BBoxTrackerStatus::Published(p) => {
                // 直接 (x, y, z) = (0, 0, 1)
                assert_relative_eq!(p.pose.position.x, 0.0, epsilon = 1e-9);
                assert_relative_eq!(p.pose.position.y, 0.0, epsilon = 1e-9);
                assert_relative_eq!(p.pose.position.z, 1.0, epsilon = 1e-9);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn frame_id_carries_from_detection_header() {
        let mut t = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        t.update_camera_info(intr());
        let dets = dets_with_one((320.0, 240.0));
        match t.process(&dets) {
            BBoxTrackerStatus::Published(p) => {
                assert_eq!(p.frame_id.as_str(), "camera_optical");
            }
            _ => panic!(),
        }
    }

    #[test]
    fn off_center_bbox_projects_farther() {
        let mut t = BBoxTracker::new(BBoxTrackerConfig::default()).unwrap();
        t.update_camera_info(intr());
        let dets = dets_with_one((420.0, 240.0)); // 偏右 100 px
        match t.process(&dets) {
            BBoxTrackerStatus::Published(p) => {
                // x_raw = (420 - 320) / 500 * 1.0 = 0.2
                // swap axes: (z, -x, -y) = (1, -0.2, 0)
                assert_relative_eq!(p.pose.position.x, 1.0, epsilon = 1e-9);
                assert_relative_eq!(p.pose.position.y, -0.2, epsilon = 1e-9);
                assert_relative_eq!(p.pose.position.z, 0.0, epsilon = 1e-9);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn invalid_config() {
        let cfg = BBoxTrackerConfig {
            goal_distance: 0.0,
            ..BBoxTrackerConfig::default()
        };
        assert!(BBoxTracker::new(cfg).is_err());
    }
}
