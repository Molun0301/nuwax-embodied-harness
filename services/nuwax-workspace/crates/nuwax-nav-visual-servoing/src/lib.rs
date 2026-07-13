//! 视觉伺服：2D bbox servo + 3D detection navigation。
//!
//! 行为对账源：`dimos/navigation/visual_servoing/visual_servoing_2d.py` + `detection_navigation.py`。
//!
//! 关键设计：
//! - 2D：bbox center 转 normalized camera x；`distance = assumed_object_width * fx / bbox_width`；
//!   太近后退；turn factor 减小；`rotation_requires_linear_movement` 在 ±0.02 rad/s 内强制前进 0.1。
//! - 3D：缺 TF 或 frame 不匹配返回 typed error，**不**输出 zero command 掩盖错误。
//! - TF 接口使用 trait，测试用 fake TF。
//!
//! ADR（待 Foundation 落地后入仓）：
//! `rotation_requires_linear_movement`：当前 Python 语义为
//!   "if rotation_requires_linear_movement AND |angular_z| < 0.02 THEN linear_x = max(linear_x, 0.1)"
//! 即"小角度且要求前进时强制最小线速度"。该语义与变量名"rotation requires linear movement"一致，
//! 不存在命名/条件不一致。本 crate 锁定该语义；如未来调整需开新 ADR。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use serde::{Deserialize, Serialize};
use thiserror::Error;

// Foundation 协议面 re-export — 允许下游 crate 直接引用共享类型
pub use nuwax_nav_base::ImageView;
pub use nuwax_nav_types_messages::BoundingBox2D as FoundationBoundingBox2D;
pub use nuwax_nav_types_messages::PoseStamped as FoundationPoseStamped;
pub use nuwax_nav_types_messages::Quaternion as FoundationQuaternion;
pub use nuwax_nav_types_messages::Twist2 as FoundationTwist2;
pub use nuwax_nav_types_messages::Twist3 as FoundationTwist3;

pub mod detection3d;
/// Original `PersonFollowSkillContainer` lifecycle and production input adapter.
pub mod person_follow;
pub mod servo2d;

/// Real-LCM production owner for [`person_follow`].
#[cfg(feature = "link-liblcm")]
pub mod lcm_owner;

pub use detection3d::{
    extract_3d_from_detection, extract_3d_from_detection_at, CameraIntrinsics, DetectionNavigation,
    DetectionNavigationConfig, DetectionNavigationModule, NavTwist3D, PointCloud2Like,
    SimplePointCloud, TargetPoint3D, TfLike,
};
pub use servo2d::{VisualServoing2D, VisualServoing2DConfig};

/// 通用错误
#[derive(Debug, Error)]
pub enum VisualServoingError {
    /// 缺少 TF
    #[error("missing TF transform for frame '{0}'")]
    MissingTf(String),
    /// frame 不匹配
    #[error("frame mismatch: expected '{expected}', got '{actual}'")]
    FrameMismatch {
        /// 期望
        expected: String,
        /// 实际
        actual: String,
    },
    /// bbox 无效
    #[error("invalid bbox: {0}")]
    InvalidBbox(String),
    /// 3D 投影失败
    #[error("3D projection failed: {0}")]
    ProjectionFailed(String),
}

/// Twist（自包含，linear/angular 数组形式）
///
/// 内部 servo2d / detection3d 使用 `[f64; 3]` 数组访问（与 Foundation `Twist3::linear: Point3` 不兼容），
/// 故保留本地类型；下游若需要 Foundation `Twist3`，使用 `FoundationTwist3` re-export。
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Twist {
    /// linear x y z
    pub linear: [f64; 3],
    /// angular x y z
    pub angular: [f64; 3],
}

impl Twist {
    /// zero
    #[must_use]
    pub fn zero() -> Self {
        Self::default()
    }
}

/// PoseStamped（自包含，与 Foundation `frame_id: NavFrameId / stamp: Timestamp / pose: Pose3` 不同形状）
///
/// 内部 `detection3d` 使用 `robot_tf.frame_id` / `position[0..2]` / `orientation[0..3]` 形式，
/// 故保留本地类型。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseStamped {
    /// frame_id
    pub frame_id: String,
    /// position xyz
    pub position: [f64; 3],
    /// orientation xyzw
    pub orientation: [f64; 4],
}

impl PoseStamped {
    /// identity pose
    #[must_use]
    pub fn identity(frame_id: impl Into<String>, x: f64, y: f64, z: f64) -> Self {
        Self {
            frame_id: frame_id.into(),
            position: [x, y, z],
            orientation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

/// BBox（自包含 f64 xyxy 像素坐标）
///
/// Foundation `BoundingBox2D` 使用 `f32` + 归一化语义（`from_xyxy` 自动 swap），
/// 本 crate 内部测试使用 `f64` 字面量 + strict `is_valid` 语义（`x1 > x2` 视为 invalid），
/// 故保留本地类型；下游若需要 Foundation `BoundingBox2D`，使用 `FoundationBoundingBox2D` re-export。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BBox {
    /// x1, y1, x2, y2
    pub x1: f64,
    /// y1
    pub y1: f64,
    /// x2
    pub x2: f64,
    /// y2
    pub y2: f64,
}

impl BBox {
    /// 构造
    #[must_use]
    pub fn new(x1: f64, y1: f64, x2: f64, y2: f64) -> Self {
        Self { x1, y1, x2, y2 }
    }

    /// width
    #[must_use]
    pub fn width(&self) -> f64 {
        self.x2 - self.x1
    }

    /// height
    #[must_use]
    pub fn height(&self) -> f64 {
        self.y2 - self.y1
    }

    /// center
    #[must_use]
    pub fn center(&self) -> (f64, f64) {
        ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)
    }

    /// 是否有效
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.width() > 0.0 && self.height() > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bbox_validation() {
        let bad = BBox::new(10.0, 0.0, 5.0, 10.0);
        assert!(!bad.is_valid());
        let ok = BBox::new(0.0, 0.0, 10.0, 10.0);
        assert!(ok.is_valid());
        assert_eq!(ok.center(), (5.0, 5.0));
    }
}
